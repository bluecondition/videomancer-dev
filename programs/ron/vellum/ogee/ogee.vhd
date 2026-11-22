-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ogee v0.1.0 -- Vellum.  The ogee / lantern tiling (Moroccan zellige "arabesque" tiles, Gothic ogee tracery):
-- a checkerboard of diamonds whose every edge is an S of two circular arcs, point-symmetric about the edge's
-- midpoint, so the tiles stay congruent.  One slider runs the family from plain diamonds through lanterns to
-- full round (circles and pinched four-pointed stars).  Two flat tile tones stepping in grey insets toward each
-- tile's centre (a lit relief), a black grout line between the tiles.
--
--   fold:  x~ = x mod 2, X' = |x~| - 1/2 (and Y'), the mirrors at every integer x and y keep the tile colour
--   S:     L = -X' sin b - Y' cos b,  F2 = q^2 (X'^2 + Y'^2),  side 2 when X' > Y' tan b
--          f = L - F2 (side 2) or L + F2 (side 1) = (r^2 - rho^2) / 2r about that side's arc centre
--          t = s 2 q^2 f,  d = f g(t), g = 2 / (1 + sqrt(1 - 2t)) (ROM): the true distance from the tile edge
--   steps: phase = (|d| - grout) * 6 + camera, tone = step clamp(floor(phase), 0, 3) of tile A (f > 0) or B
--
--   K1  X        pan, +-1920 virtual px             K2  Y   pan, +-1088 virtual px
--   K3  Scale    tile size, half diagonal 84 .. 672 virtual px (exponential)
--   K4  Slender  the lattice aspect: squat .. tall lanterns (B / A 0.57 .. 1.75, the arcs become oval)
--   K5  Drift    the tiling slides along its diagonal, one period wraps exactly.  Centre +-24 = still,
--                2^-12 .. 2^-4.15 period per field
--   K6  Camera   luma^2 moves the insets (A's toward the grout, B's toward the centre): lit tiles flatten
--   S7  Glide    Off = the knobs act this field;  On = 1/16 of the gap per field (Drift speed 1/8)
--   S8  Inset    Bands (the insets as filled steps) / Lines (flat tiles with the insets as lines)
--   S9  Tone     Grey (stepped greys) / Ink (black and white, the insets alternating)
--   S10 Weight   Bold / Fine (Bands: a line drawing on black, white grout;  Lines: lines of 1/6 step, not 1/3)
--   S11 Source   Video / Osc (the shell's drifting triangle lattice through the camera's table)
--   P12 Curve    0 % straight diamonds, 50 % lanterns, 100 % full round
--
-- Sections (the integer model .vellum_work/rb_ogee/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide (S7), switches, Drift speed glide
--   [C] FIELD SEQUENCER                  one serial multiplier: the steps, positions, curve and DDA constants
--   [L] LINE ENGINE                      the y DDA and the per-line terms (on the sequencer's multiplier)
--   [P] PIXEL PATH                       x DDA + two product DDAs -> folds -> f -> t -> g ROM -> d -> steps ->
--                                        the nearer of the step and grout edges -> coverage (shift divisor) -> luma
--   [D] SOURCE FIELD                     luma^2 * 3/4 K6 (a per-field table; Osc doubled) -> 128/64-px box ->
--                                        vertical one-pole
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 toward black, blanking gate; U = V = 512
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  Pixels 0 .. W + 1
-- are rendered; output column j is the [1 2 1] filter over pixels j .. j + 2 and lands on screen column j + 4
-- (the generated avid starts 4 clocks after the delayed source avid), so pixel m sits at screen column m + 3.
-- s_gav_sr(S - 3) flags pixels 0 .. W - 1 on their way into a stage-S register.  One latency (32) in every mode.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.ogee_pkg.all;
use work.all;

architecture ogee of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
    constant C_DEF_K4  : integer := 680;
    constant C_DEF_K5  : integer := 660;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;

    constant C_GH : integer := 410;   -- grout half width, Q14 domain units

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps
    ---------------------------------------------------------------------------
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);
            when 4          => return to_unsigned(576, 12);
            when 8          => return to_unsigned(960, 12);
            when 12         => return to_unsigned(480, 12);
            when 5 | 6 | 14 => return to_unsigned(384, 12);
            when 1 | 2 | 10 => return to_unsigned(512, 12);
            when others     => return to_unsigned(256, 12);
        end case;
    end function;
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- a + b or a - b on ONE carry chain (the low column makes the carry-in)
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;
    -- a + b + 1 (inv = 0) or a - b (inv = 1) on ONE carry chain (24-bit signed)
    function f_xadd(a, b : signed(23 downto 0); inv : std_logic) return signed is
        variable v_a, v_b, v_r : signed(25 downto 0);
        variable v_o : signed(23 downto 0);
    begin
        v_a := resize(a, 25) & '1';
        if inv = '1' then v_b := (not resize(b, 25)) & '1'; else v_b := resize(b, 25) & '1'; end if;
        v_r := v_a + v_b;
        v_o := v_r(24 downto 1);
        return v_o;
    end function;
    -- (|v| - 1/2) >> 6 of a 23-bit lattice coordinate (one's complement |v|): Q22 -> Q16, signed 16 bits
    function f_fold(v : signed(22 downto 0)) return signed is
        variable v_m : std_logic_vector(21 downto 0);
    begin
        if v(22) = '1' then v_m := not std_logic_vector(v(21 downto 0));
        else v_m := std_logic_vector(v(21 downto 0)); end if;
        return signed((not v_m(21)) & v_m(20 downto 6));
    end function;
    -- clamp a signed band index to 0 .. 3
    function f_c03(v : signed(6 downto 0)) return unsigned is
    begin
        if v(6) = '1' then return "00";
        elsif v > 3 then return "11";
        else return unsigned(v(1 downto 0)); end if;
    end function;
    -- saturate to 14-bit signed
    function f_sat14(v : signed) return signed is
        variable v_o : signed(13 downto 0);
    begin
        if v > 8191 then v_o := to_signed(8191, 14);
        elsif v < -8192 then v_o := to_signed(-8192, 14);
        else v_o := resize(v, 14); end if;
        return v_o;
    end function;
    -- coverage 0 .. 16 = clamp(8 + v), v = (dd * 16) >> kk (the shift and the clamp are separate stages)
    function f_cov(v_s : signed(17 downto 0)) return unsigned is
        variable v_c : signed(17 downto 0);
    begin
        if v_s < -8 then return to_unsigned(0, 5);
        elsif v_s > 8 then return to_unsigned(16, 5);
        else v_c := v_s + 8; return unsigned(v_c(4 downto 0)); end if;
    end function;
    -- dd = e, or e * 23/32 for the in-between divisor
    function f_dd(e : signed(13 downto 0); half : std_logic) return signed is
    begin
        if half = '1' then return e - shift_right(e, 2) - shift_right(e, 5); else return e; end if;
    end function;
    -- (u * c) >> 4 as gated shifts, c = 0 .. 15 (signed u)
    function f_g4(u : signed(8 downto 0); c : unsigned(3 downto 0)) return signed is
        variable v0, v1, v2, v3 : signed(12 downto 0);
    begin
        v0 := (others => '0'); v1 := (others => '0'); v2 := (others => '0'); v3 := (others => '0');
        if c(0) = '1' then v0 := resize(u, 13); end if;
        if c(1) = '1' then v1 := shift_left(resize(u, 13), 1); end if;
        if c(2) = '1' then v2 := shift_left(resize(u, 13), 2); end if;
        if c(3) = '1' then v3 := shift_left(resize(u, 13), 3); end if;
        return (v0 + v1) + (v2 + v3);
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 15) := (others => '0');  -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: +-1 hold (10 bits), glide (Q3)
    signal h_1 : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal h_2 : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal h_3 : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal h_4 : unsigned(9 downto 0) := to_unsigned(C_DEF_K4, 10);
    signal h_5 : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal h_6 : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal h_p : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal g_1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal g_2 : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal g_3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal g_4 : unsigned(12 downto 0) := to_unsigned(C_DEF_K4 * 8, 13);
    signal g_6 : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);
    signal g_p : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(13 downto 0) := (others => '0');
    signal s_gr, s_gh : unsigned(9 downto 0) := (others => '0');
    signal z_gp, z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun, s_gmv : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_lines, s_ink, s_fine, s_src : std_logic := '0';
    signal s_lg7 : std_logic := '1';                               -- box: 128 samples (1080) / 64
    signal s_ish : unsigned(2 downto 0) := to_unsigned(5, 3);      -- the vertical one-pole's shift
    signal s_vtg, s_vdd : signed(16 downto 0) := (others => '0');
    signal s_vgs : signed(16 downto 0) := to_signed((C_DEF_K5 - 512) * 64, 17);   -- Drift speed glide, Q6

    -- ROMs (read in the zone): 2^(i/256) and the curve
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal r_cv_a : unsigned(7 downto 0) := (others => '0');
    signal r_sb_q, r_cb_q, r_tb_q, r_qb_q : unsigned(15 downto 0) := (others => '0');
    signal z_sb, z_qb : unsigned(14 downto 0) := (others => '0');

    -- [C] the serial multiplier and the sequencer's scratch
    signal f_a, f_acc : unsigned(39 downto 0) := (others => '0');
    signal f_b : unsigned(20 downto 0) := (others => '0');
    signal z_fa : unsigned(25 downto 0) := (others => '0');   -- (the widest operand: 23-bit signed)
    signal z_fb : unsigned(20 downto 0) := (others => '0');
    signal z_sx : std_logic := '0';
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_o3 : unsigned(9 downto 0) := (others => '0');
    signal z_o4 : unsigned(7 downto 0) := (others => '0');
    signal z_eA, z_eB : signed(11 downto 0) := (others => '0');
    signal z_ia, z_ib : unsigned(2 downto 0) := (others => '0');
    signal z_rA, z_rB : unsigned(18 downto 0) := (others => '0');
    signal z_xo, z_yo : signed(17 downto 0) := (others => '0');   -- pixel 0's x / line 0's y (Q4 virtual px)
    signal z_sm : unsigned(19 downto 0) := (others => '0');
    signal z_av  : unsigned(9 downto 0) := (others => '0');
    signal z_vz, z_vn : std_logic := '1';
    signal z_ls  : unsigned(9 downto 0) := (others => '0');
    signal z_spd : unsigned(13 downto 0) := (others => '0');
    signal z_ssh : unsigned(2 downto 0) := (others => '0');
    signal z_dl1 : unsigned(17 downto 0) := (others => '0');          -- the Drift accumulator, Q18 of a period

    -- [C] per-field constants
    signal c_kx  : unsigned(17 downto 0) := (others => '0');          -- domain units per pixel, Q22
    signal c_ky  : unsigned(18 downto 0) := (others => '0');          -- domain units per field line, Q22
    signal c_xt0, c_yt0 : signed(22 downto 0) := (others => '0');     -- pixel 0 / line 0, Q22 mod 2
    signal c_i1, c_iq : unsigned(18 downto 0) := (others => '0');     -- product DDA steps, Q22
    signal c_i1w, c_iqw : signed(23 downto 0) := (others => '0');     -- ... minus the wrap
    signal c_p10, c_q0 : signed(23 downto 0) := (others => '0');      -- ... at pixel 0
    signal c_K   : unsigned(12 downto 0) := (others => '0');          -- q^2 = half the curvature, Q12
    signal c_kk  : unsigned(3 downto 0) := (others => '0');           -- the edge divisor: 2^kk (x 32/23 if half)
    signal c_half : std_logic := '0';
    signal s_g45 : unsigned(10 downto 0) := to_unsigned(1147, 11);   -- 4.5 (K6 >> 5)
    -- the camera gain table fill: A = g45 (i - 16)^2, D = its first difference
    signal z_gA  : unsigned(25 downto 0) := (others => '0');
    signal z_gD  : unsigned(19 downto 0) := (others => '0');
    signal gt_wa : unsigned(7 downto 0) := (others => '0');
    signal gt_wd : unsigned(12 downto 0) := (others => '0');
    signal gt_we : std_logic := '0';

    -- [L] line engine
    signal e_st  : unsigned(6 downto 0) := (others => '1');
    signal n_y   : signed(22 downto 0) := (others => '0');
    signal n_yq  : signed(15 downto 0) := (others => '0');
    signal n_aqy : unsigned(15 downto 0) := (others => '0');
    signal n_lyl, l_lyl : signed(15 downto 0) := (others => '0');
    signal n_xs, l_xs : signed(16 downto 0) := (others => '0');
    signal n_qy, l_qy : unsigned(12 downto 0) := (others => '0');

    -- [P] pixel path: the field (stages 1 .. 19)
    signal b_x : signed(22 downto 0) := (others => '0');
    signal s1_w : std_logic := '0';
    signal b_p, b_q : signed(23 downto 0) := (others => '0');
    signal s2_x16 : signed(15 downto 0) := (others => '0');
    signal s2_sg : std_logic := '0';
    signal s3_lx, s3_u : signed(23 downto 0) := (others => '0');
    signal s4_l, s5_l, s6_l, s7_l, s8_l : signed(15 downto 0) := (others => '0');
    signal s4_aq : unsigned(13 downto 0) := (others => '0');
    signal r_sq_q : unsigned(15 downto 0) := (others => '0');
    signal s5_hl, s6_hl : unsigned(13 downto 0) := (others => '0');
    signal s6_sq : unsigned(12 downto 0) := (others => '0');
    signal s7_qx : unsigned(12 downto 0) := (others => '0');
    signal s8_f2 : unsigned(13 downto 0) := (others => '0');
    signal s9_f : signed(15 downto 0) := (others => '0');
    signal s_sd : std_logic_vector(3 to 9) := (others => '0');       -- the side flag, stages 3 .. 9
    signal s10_af : unsigned(14 downto 0) := (others => '0');
    signal s11_pl, s11_ph : unsigned(17 downto 0) := (others => '0');
    signal s12_tp : unsigned(12 downto 0) := (others => '0');
    signal s13_idx : unsigned(10 downto 0) := (others => '0');
    signal r_g_q, s15_g : unsigned(11 downto 0) := (others => '0');
    signal s16_pl : unsigned(18 downto 0) := (others => '0');
    signal s16_ph : unsigned(19 downto 0) := (others => '0');
    signal s17_a : unsigned(14 downto 0) := (others => '0');
    signal s18_ph : signed(17 downto 0) := (others => '0');
    type t_af is array (11 to 15) of unsigned(13 downto 0);
    signal s_afd : t_af := (others => (others => '0'));             -- |f| >> 1, stages 11 .. 15
    signal s_ps : std_logic_vector(10 to 12) := (others => '0');     -- t >= 0, stages 10 .. 12
    signal s_ow : std_logic_vector(10 to 19) := (others => '0');     -- tile B (f < 0), stages 10 .. 19
    -- the steps, the nearer edge, coverage and luma (stages 19 .. 26)
    signal s19_ph : signed(18 downto 0) := (others => '0');
    signal s19_eg, s20_eg : signed(13 downto 0) := (others => '0');
    signal s20_es : signed(13 downto 0) := (others => '0');
    signal s20_ti, s20_to : unsigned(6 downto 0) := (others => '0');
    signal s21_e, s22_dd : signed(13 downto 0) := (others => '0');
    signal s21_hi, s21_lo, s22_hi, s22_lo, s23_hi, s23_lo, s24_hi, s24_lo, s25_hi, s25_lo : unsigned(6 downto 0) := (others => '0');
    signal s23_v : signed(17 downto 0) := (others => '0');
    signal s24_cv : unsigned(4 downto 0) := (others => '0');
    signal s25_pd : signed(12 downto 0) := (others => '0');
    signal s25_cf : std_logic := '0';
    signal s26_v : unsigned(6 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_ga  : unsigned(7 downto 0) := (others => '0');
    type t_gt is array (0 to 255) of unsigned(15 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt_ram : signal is true;
    signal r_gt_q : unsigned(15 downto 0) := (others => '0');
    signal s_p, s_sfirst : unsigned(12 downto 0) := (others => '0');
    signal s_sv0, s_sv, s_sv1, s_svd, s_bv, s_ev : std_logic := '0';
    signal s_col, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(15 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(7 downto 0) := (others => '0');
    signal s_bsum : unsigned(19 downto 0) := (others => '0');
    signal s_eb  : unsigned(12 downto 0) := (others => '0');
    signal s_eq  : unsigned(15 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal c_a, c_b, s_Ls : unsigned(12 downto 0) := (others => '0');
    signal c_new : std_logic := '0';

    -- [F] output conditioning
    signal s27_y, s28_y, s29_y, s30_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s30_ya : unsigned(11 downto 0) := (others => '0');
    signal s31_y, s32_y, s33_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_OG_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            r_sb_q <= C_OG_SINB(to_integer(r_cv_a));
            r_cb_q <= C_OG_COSB(to_integer(r_cv_a));
            r_tb_q <= C_OG_TANB(to_integer(r_cv_a));
            r_qb_q <= C_OG_QB(to_integer(r_cv_a));
        end if;
    end process p_rom;

    -- the pixel path's ROMs: the squarer (address at stage 4, output at 5) and the distance (13 -> 14); each
    -- output register is absorbed into its block RAM and gets a fabric copy
    p_prom : process(clk)
    begin
        if rising_edge(clk) then
            r_sq_q <= C_OG_SQ(to_integer(s4_aq(13 downto 7)));
            r_g_q  <= C_OG_G(to_integer(s13_idx));
        end if;
    end process p_prom;

    -- the camera gain table (written in the zone, read during active video)
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            r_gt_q <= gt_ram(to_integer(s_ga));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= resize(gt_wd, 16); end if;
        end if;
    end process p_gtram;

    -- box delay: read N samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv1 = '1' then bx_ram(to_integer(bx_wp)) <= resize(s_p, 16); end if;
        end if;
    end process p_bxram;

    -- luma line store (half horizontal resolution): the one-pole's own read, 2 addresses ahead of its write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q <= lb_ram(to_integer(s_col(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(10 downto 0);
        variable v_v  : signed(10 downto 0);
        variable v_n11 : signed(10 downto 0);
        variable v_s14 : unsigned(14 downto 0);
        variable v_s17 : unsigned(16 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_ni : signed(11 downto 0);
        variable v_x  : unsigned(39 downto 0);
        variable v_ab : signed(15 downto 0);
        variable v_ng : signed(16 downto 0);
        variable v_lv : std_logic;
        variable v_sm : unsigned(23 downto 0);
        variable v_p  : unsigned(3 downto 0);
        variable v_nx : std_logic;
        variable v_k20 : unsigned(19 downto 0);
        variable v_fp : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- [A] vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 2047 then
                s_zc <= s_zc + 1;
            end if;

            -- [A] width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_lines <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                if s_xsr = 2 then s_lg7 <= '1'; else s_lg7 <= '0'; end if;
                if s_ys <= 384 then s_ish <= to_unsigned(5, 3);
                elsif s_ys >= 960 then s_ish <= to_unsigned(3, 3);
                else s_ish <= to_unsigned(4, 3); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared hold + glide, six clocks per channel.  Hold: the knob moves the held value only by
            -- 2 counts or more (or to either end).  Glide On: 1/16 of the gap per field, never slower than
            -- one step (1/8 count; X and Y two steps), landing exactly.  Glide Off: the held value, this field.
            -- (The hold test is registered in two steps: the compare chain into the hold's enable was the
            -- critical path.)
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= g_1; s_gh <= h_1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= g_2; s_gh <= h_2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= g_3; s_gh <= h_3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= g_4; s_gh <= h_4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= g_6; s_gh <= h_6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= g_p; s_gh <= h_p; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "001";
                    when 1 =>
                        v_d := signed(resize(s_gr, 11)) - signed(resize(s_gh, 11));
                        if s_gr = 0 or s_gr = 1023 or v_d > 1 or v_d < -1 then s_gmv <= '1'; else s_gmv <= '0'; end if;
                        z_gp <= "101";
                    when 5 =>
                        if s_gmv = '1' then s_gh <= s_gr; end if;
                        z_gp <= "010";
                    when 2 =>
                        s_gd <= signed(resize(s_gh & "000", 14)) - signed(resize(s_gm, 14));
                        z_gp <= "011";
                    when 3 =>
                        if s_glide = '1' then
                            if z_gc(2 downto 1) = "00" then                -- X, Y: at least 2 steps
                                if s_gd > 32 or s_gd < -32 then s_gd <= shift_right(s_gd, 4);
                                elsif s_gd > 2 then s_gd <= to_signed(2, 14);
                                elsif s_gd < -2 then s_gd <= to_signed(-2, 14);
                                end if;
                            else
                                if s_gd > 16 or s_gd < -16 then s_gd <= shift_right(s_gd, 4);
                                elsif s_gd > 1 then s_gd <= to_signed(1, 14);
                                elsif s_gd < -1 then s_gd <= to_signed(-1, 14);
                                end if;
                            end if;
                        end if;
                        z_gp <= "100";
                    when 4 =>
                        case to_integer(z_gc) is
                            when 0      => g_1 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_1 <= s_gh;
                            when 1      => g_2 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_2 <= s_gh;
                            when 2      => g_3 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_3 <= s_gh;
                            when 3      => g_4 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_4 <= s_gh;
                            when 4      => g_6 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_6 <= s_gh;
                            when others => g_p <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_p <= s_gh;
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                    when others => z_gp <= "000";
                end case;
            end if;

            -- [C] serial products: a slot is decoded at zc = P (below), its operands are pre-muxed one
            -- clock later, it loads the clock after that, runs 21 iterations and is stored at P + 24.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 48  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 76  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 104 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 132 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 160 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 188 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 216 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 244 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 272 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 300 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_sx <= '0';
                case to_integer(z_sel) is
                    when 1  => z_fa <= resize(r_exf, 26); z_fb <= to_unsigned(99864, 21);      -- rA (EXP[eA] K84)
                    when 2  => z_fa <= resize(r_exf, 26); z_fb <= to_unsigned(99864, 21);      -- rB
                    when 3  => z_fa <= resize(z_rB, 26); z_fb <= resize(s_ys, 21);             -- per field line
                    -- signed operands are sign-extended into the 44-bit shifter: the product is exact mod 2^44
                    when 4  => z_fa <= unsigned(resize(z_xo, 26)); z_fb <= resize(z_rA, 21); z_sx <= '1';   -- pixel 0's x
                    when 5  => z_fa <= unsigned(resize(z_yo, 26)); z_fb <= resize(z_rB, 21); z_sx <= '1';   -- line 0's y
                    when 6  => z_fa <= resize(z_qb, 26); z_fb <= resize(z_qb, 21);             -- K = q^2
                    when 7  => z_fa <= resize(c_kx, 26); z_fb <= resize(z_sb, 21);             -- DDA steps
                    when 8  => z_fa <= resize(c_kx, 26); z_fb <= resize(z_qb, 21);
                    when 9  => z_fa <= unsigned(resize(c_xt0, 26)); z_fb <= resize(z_sb, 21); z_sx <= '1';   -- at pixel 0
                    when others => z_fa <= unsigned(resize(c_xt0, 26)); z_fb <= resize(z_qb, 21); z_sx <= '1';
                end case;
            end if;
            -- [L] LINE ENGINE on the same multiplier (the sequencer only uses it in the zone, before zc 341): the
            -- y DDA and the NEXT line's terms, started at every line start and once in the zone (zc 360) for line 0:
            -- Lyl = -(Y' cos b) (Q14),  Xs = Y' tan b (Q16),  Qy = (q Y')^2 (Q14); 24 clocks per product
            if s_zone = '1' and s_zc = 360 then
                n_y <= c_yt0; e_st <= to_unsigned(1, 7);
            elsif s_st(0) = '1' then
                l_lyl <= n_lyl; l_xs <= n_xs; l_qy <= n_qy;
                e_st <= (others => '0');
            elsif e_st /= 127 then
                e_st <= e_st + 1;
            end if;
            v_lv := '0';
            case to_integer(e_st) is
                when 0  => n_y <= n_y + signed(resize(c_ky, 23));
                when 1  => n_yq <= f_fold(n_y);
                when 2 | 26 | 50 | 74 => v_lv := '1';
                when 25 => v_ng := -resize(signed(f_acc(31 downto 16)), 17); n_lyl <= v_ng(15 downto 0);
                when 49 => n_xs <= signed(f_acc(30 downto 14));
                when 73 =>
                    v_ab := signed(f_acc(30 downto 15));
                    if v_ab(15) = '1' then n_aqy <= unsigned(-v_ab); else n_aqy <= unsigned(v_ab); end if;
                when 97 => n_qy <= f_acc(28 downto 16);
                when others => null;
            end case;
            if z_ld = '1' then
                f_acc <= (others => '0');
                if z_sx = '1' then f_a <= unsigned(resize(signed(z_fa), 40)); else f_a <= resize(z_fa, 40); end if;
                f_b <= z_fb;
            elsif v_lv = '1' then
                f_acc <= (others => '0');
                case to_integer(e_st) is
                    when 2      => f_a <= unsigned(resize(n_yq, 40)); f_b <= resize(r_cb_q(14 downto 0), 21);
                    when 26     => f_a <= unsigned(resize(n_yq, 40)); f_b <= resize(r_tb_q(14 downto 0), 21);
                    when 50     => f_a <= unsigned(resize(n_yq, 40)); f_b <= resize(z_qb, 21);
                    when others => f_a <= resize(n_aqy, 40); f_b <= resize(n_aqy, 21);
                end case;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(20 downto 1);
            end if;

            -- the camera gain table: T[i] = (g45 max(i - 16, 0)^2) >> 14 by second differences, zc 350 .. 605
            gt_we <= '0';
            if s_zone = '1' and s_zc >= 350 and s_zc < 606 then
                gt_we <= '1';
                gt_wa <= resize(s_zc - 350, 8);
                if s_zc < 366 then
                    gt_wd <= (others => '0');
                    z_gA <= (others => '0'); z_gD <= resize(s_g45, 20);
                else
                    gt_wd <= resize(shift_right(z_gA, 14), 13);
                    z_gA <= z_gA + resize(z_gD, 26);
                    z_gD <= z_gD + resize(s_g45 & '0', 20);
                end if;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- [B] Drift: hold, then the speed glide (1/8 of the gap per field), Q6
                    when 33 =>
                        v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(h_5, 11));
                        if unsigned(registers_in(4)) = 0 or unsigned(registers_in(4)) = 1023 or v_d > 1 or v_d < -1 then
                            h_5 <= unsigned(registers_in(4));
                        end if;
                    when 34 => s_vtg <= shift_left(resize(signed('0' & h_5) - 512, 17), 6);
                    when 35 => s_vdd <= s_vtg - s_vgs;
                    when 36 =>
                        if s_glide = '1' then s_vgs <= s_vgs + shift_right(s_vdd, 3);
                        else s_vgs <= s_vtg; end if;
                        s_g45 <= resize(g_6(12 downto 5) & "00", 11) + resize(g_6(12 downto 6), 11);
                    -- position (cartouche's): X +-1920 virtual px (Q4: (g1 - 4096) * 7.5), Y +-1088 (* 4.25);
                    -- Scale and Slender as octaves: o3 = 3 g3 / 32, o4 = 13 g4 / 512
                    when 37 =>
                        -- 7.5 (g1 - 4096) = 8 g1 - g1 / 2 - 30720;  4.25 (g2 - 4096) = 4 g2 + g2 / 4 - 17408
                        z_xo <= signed(resize(g_1 & "000", 18)) - signed(resize(g_1(12 downto 1), 18)) - to_signed(30720, 18);
                        z_yo <= signed(resize(g_2 & "00", 18)) + signed(resize(g_2(12 downto 2), 18)) - to_signed(17408, 18);
                        v_s14 := resize(g_3, 15) + resize(g_3 & '0', 15);
                        z_o3 <= v_s14(14 downto 5);
                        v_s17 := resize(g_4 & "000", 17) + resize(g_4 & "00", 17) + resize(g_4, 17);
                        z_o4 <= v_s17(16 downto 9);
                    when 38 =>
                        -- eA = o4 - o3 - 104,  eB = 104 - o3 - o4  (1/256 octave)
                        z_eA <= signed(resize(z_o4, 12)) - signed(resize(z_o3, 12)) - to_signed(104, 12);
                        z_eB <= to_signed(104, 12) - signed(resize(z_o3, 12)) - signed(resize(z_o4, 12));
                    when 39 =>
                        z_yo <= signed(resize(s_ys(11 downto 5), 18)) - to_signed(8640, 18) - z_yo;   -- line 0's y, Q4
                        r_ex_a <= unsigned(z_eA(7 downto 0));
                        v_ni := -shift_right(z_eA, 8); z_ia <= unsigned(v_ni(2 downto 0));
                        v_ni := -shift_right(z_eB, 8); z_ib <= unsigned(v_ni(2 downto 0));
                        r_cv_a <= g_p(12 downto 5);
                    when 40 =>
                        case to_integer(s_xs) is                  -- pixel 0's x (it sits at screen column 3), Q4
                            when 4      => z_xo <= to_signed(-11408, 18) - z_xo;
                            when 3      => z_xo <= to_signed(-15276, 18) - z_xo;
                            when others => z_xo <= to_signed(-15304, 18) - z_xo;
                        end case;
                    when 42 =>
                        z_sb <= r_sb_q(14 downto 0); z_qb <= r_qb_q(14 downto 0);
                    when 50 => r_ex_a <= unsigned(z_eB(7 downto 0));
                    -- the stores (P + 24)
                    when 72 =>                                   -- rA = EXP K84 >> (14 + ia)
                        case to_integer(z_ia) is
                            when 0      => v_x := shift_right(f_acc, 14);
                            when 1      => v_x := shift_right(f_acc, 15);
                            when 2      => v_x := shift_right(f_acc, 16);
                            when 3      => v_x := shift_right(f_acc, 17);
                            when others => v_x := shift_right(f_acc, 18);
                        end case;
                        z_rA <= v_x(18 downto 0);
                    when 73 =>                                   -- domain units per pixel: (rA xs / 2) >> 2, Q22
                        case to_integer(s_xs) is
                            when 4      => c_kx <= resize(z_rA(18 downto 1), 18);
                            when 3      => v_k20 := resize(z_rA, 20) + resize(z_rA(18 downto 1), 20); c_kx <= v_k20(19 downto 2);
                            when others => c_kx <= resize(z_rA(18 downto 2), 18);
                        end case;
                    when 100 =>
                        case to_integer(z_ib) is
                            when 0      => v_x := shift_right(f_acc, 14);
                            when 1      => v_x := shift_right(f_acc, 15);
                            when 2      => v_x := shift_right(f_acc, 16);
                            when 3      => v_x := shift_right(f_acc, 17);
                            when others => v_x := shift_right(f_acc, 18);
                        end case;
                        z_rB <= v_x(18 downto 0);
                    when 128 => c_ky  <= f_acc(28 downto 10);
                    -- pixel 0's x and line 0's y, minus the drift (the tiling slides right and down for Drift > 0)
                    when 156 => c_xt0 <= signed(f_acc(28 downto 6)) - signed(shift_left(resize(z_dl1, 23), 5));
                    when 184 => c_yt0 <= signed(f_acc(28 downto 6)) - signed(shift_left(resize(z_dl1, 23), 5));
                    when 212 => c_K   <= f_acc(28 downto 16);
                    when 240 => c_i1  <= f_acc(32 downto 14);
                    when 268 => c_iq  <= f_acc(32 downto 14);
                    -- [B] Drift speed: |v| < 24 still; else 2^-12 .. 2^-4.15 of a period per field
                    when 110 =>
                        v_v := resize(shift_right(s_vgs, 6), 11);
                        z_vn <= v_v(10);
                        v_n11 := -v_v;
                        if v_v(10) = '1' then z_av <= unsigned(v_n11(9 downto 0)); else z_av <= unsigned(v_v(9 downto 0)); end if;
                    when 111 =>
                        if z_av < 24 then z_vz <= '1'; z_av <= (others => '0');
                        else z_vz <= '0'; z_av <= z_av - 24; end if;
                    when 112 => z_ls <= z_av + shift_right(z_av, 5);
                    when 113 => r_ex_a <= z_ls(5 downto 0) & "00";
                    when 116 =>
                        if z_vz = '1' then z_spd <= (others => '0');
                        else z_spd <= resize(r_exf(15 downto 9), 14); end if;
                        z_ssh <= z_ls(8 downto 6);
                    when 117 =>
                        case to_integer(z_ssh) is
                            when 0      => null;
                            when 1      => z_spd <= shift_left(z_spd, 1);
                            when 2      => z_spd <= shift_left(z_spd, 2);
                            when 3      => z_spd <= shift_left(z_spd, 3);
                            when 4      => z_spd <= shift_left(z_spd, 4);
                            when 5      => z_spd <= shift_left(z_spd, 5);
                            when 6      => z_spd <= shift_left(z_spd, 6);
                            when others => z_spd <= shift_left(z_spd, 7);
                        end case;
                    -- one period = 2^18
                    when 118 => z_dl1 <= f_addsub(z_dl1, resize(z_spd, 18), z_vn);
                    when 296 => c_p10 <= signed(f_acc(37 downto 14));
                    when 324 => c_q0  <= signed(f_acc(37 downto 14));
                    -- the steps per pixel: the mean of the x and line steps, x 6; the edge divisor
                    when 342 => z_sm <= resize(c_kx, 20) + resize(c_ky, 20);
                    when 343 =>                                  -- fpb = 3 (kx + ky) >> 10 (Q12 steps per pixel)
                        v_sm := resize(z_sm, 24) + resize(z_sm & '0', 24);
                        v_fp := v_sm(22 downto 10);
                        v_p := (others => '0'); v_nx := '0';
                        for i in 1 to 12 loop
                            if v_fp(i) = '1' then v_p := to_unsigned(i, 4); v_nx := v_fp(i - 1); end if;
                        end loop;
                        if v_nx = '1' then c_kk <= v_p + 1; else c_kk <= v_p; end if;
                        c_half <= not v_nx;
                    when 345 =>
                        c_i1w <= signed(resize(c_i1, 24)) - signed(resize(z_sb & "000000000", 24));
                        c_iqw <= signed(resize(c_iq, 24)) - signed(resize(z_qb & "000000000", 24));
                    when others => null;
                end case;
            end if;

            -- [A] sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the
            -- core on every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 14);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width)
    ---------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;

            if g_hsr = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if g_avr = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;

            if g_start = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- [P] PIXEL PATH
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_x   : signed(22 downto 0);
        variable v_qv  : signed(15 downto 0);
        variable v_f   : signed(15 downto 0);
        variable v_t   : unsigned(13 downto 0);
        variable v_a   : unsigned(23 downto 0);
        variable v_b   : unsigned(24 downto 0);
        variable v_a3  : unsigned(16 downto 0);
        variable v_nf, v_nr : signed(6 downto 0);
        variable v_fr  : unsigned(11 downto 0);
        variable v_ds  : unsigned(11 downto 0);
        variable v_n, v_nn : unsigned(1 downto 0);
        variable v_ti, v_to : unsigned(6 downto 0);
        variable v_ae  : signed(13 downto 0);
        variable v_u   : signed(8 downto 0);
        variable v_g   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: the x DDA (Q24 mod 2 domain units) and its wrap
            if g_start = '1' then b_x <= c_xt0; s1_w <= '0';
            else
                v_x := b_x + signed(resize(c_kx, 23));
                b_x <= v_x;
                s1_w <= (not b_x(22)) and v_x(22);
            end if;
            -- 2: the product DDAs P = x~ sin b, Q = x~ q (each wrap takes 2 sin b / 2 q off); X' (Q16)
            if s_st(0) = '1' then b_p <= c_p10; b_q <= c_q0;
            elsif s1_w = '1' then b_p <= b_p + c_i1w; b_q <= b_q + c_iqw;
            else b_p <= b_p + signed(resize(c_i1, 24)); b_q <= b_q + signed(resize(c_iq, 24)); end if;
            s2_x16 <= f_fold(b_x); s2_sg <= b_x(22);
            -- 3: -X' sin b = sin b / 2 - sgn(x~) P;  +-q X' = Q + q / 2 + 1 (x~ < 0) or Q - q / 2  (Q24, one carry
            -- chain each);  the side: X' > Y' tan b
            s3_lx <= f_xadd(signed(resize(z_sb & "0000000", 24)), b_p, not s2_sg);
            s3_u  <= f_xadd(b_q, signed(resize(z_qb & "0000000", 24)), not s2_sg);
            if resize(s2_x16, 17) > l_xs then s_sd(3) <= '1'; else s_sd(3) <= '0'; end if;
            -- 4: L = -X' sin b - Y' cos b (Q14);  v = |q X'| (Q14)
            s4_l <= s3_lx(23 downto 8) + l_lyl;
            v_qv := s3_u(23 downto 8);
            if v_qv(15) = '1' then v_qv := -v_qv; end if;
            s4_aq <= unsigned(v_qv(13 downto 0));
            -- 5, 6: (q X')^2 = SQ[v >> 7] + ((v >> 7)(v & 127)) >> 6  (the ROM's output at 5, its copy at 6)
            s5_hl <= s4_aq(13 downto 7) * s4_aq(6 downto 0);
            s5_l <= s4_l;
            s6_sq <= r_sq_q(12 downto 0); s6_hl <= s5_hl; s6_l <= s5_l;
            -- 7: Qx;  8: F2 = q^2 (X'^2 + Y'^2)
            s7_qx <= s6_sq + resize(s6_hl(13 downto 6), 13);
            s7_l <= s6_l;
            s8_f2 <= resize(s7_qx, 14) + resize(l_qy, 14);
            s8_l <= s7_l;
            -- 9: f = L - F2 (side 2) or L + F2 (side 1): (r^2 - rho^2) / 2r about that side's centre, Q14
            if s_sd(8) = '1' then s9_f <= s8_l - signed(resize(s8_f2, 16));
            else s9_f <= s8_l + signed(resize(s8_f2, 16)); end if;
            for i in 4 to 9 loop s_sd(i) <= s_sd(i - 1); end loop;
            -- 10: |f|, the tile (B when f < 0), the sign of t = s 2 q^2 f
            if s9_f(15) = '1' then v_f := -s9_f; else v_f := s9_f; end if;
            s10_af <= unsigned(v_f(14 downto 0));
            s_ow(10) <= s9_f(15);
            s_ps(10) <= s_sd(9) xor s9_f(15);
            -- 11, 12: |t| = (|f| >> 3) (K >> 1) >> 10  (Q11), K >> 1 split lo 6 / hi 6
            s11_pl <= s10_af(14 downto 3) * c_K(6 downto 1);
            s11_ph <= s10_af(14 downto 3) * c_K(12 downto 7);
            s_afd(11) <= s10_af(14 downto 1);
            v_a := resize(s11_ph & "000000", 24) + resize(s11_pl, 24);
            s12_tp <= v_a(22 downto 10);
            -- 13: the ROM address: t >= 0: 1024 + min(t 2048, 1023);  t < 0: 1024 - ceil(|t| 256) = (8192 - |t|) >> 3
            if s_ps(12) = '1' then
                if s12_tp > 1023 then s13_idx <= (others => '1'); else s13_idx <= '1' & s12_tp(9 downto 0); end if;
            else
                v_t := to_unsigned(8192, 14) - resize(s12_tp, 14); s13_idx <= v_t(13 downto 3);
            end if;
            for i in 11 to 12 loop s_ps(i) <= s_ps(i - 1); end loop;
            for i in 12 to 15 loop s_afd(i) <= s_afd(i - 1); end loop;
            -- 14: the ROM's output register;  15: its fabric copy
            s15_g <= r_g_q;
            -- 16, 17: d = (|f| >> 1)(g >> 1) >> 10 (the true distance from the tile edge, Q13), g >> 1 split lo 5 / hi 6
            s16_pl <= s_afd(15) * s15_g(5 downto 1);
            s16_ph <= s_afd(15) * s15_g(11 downto 6);
            v_b := resize(s16_ph & "00000", 25) + resize(s16_pl, 25);
            s17_a <= v_b(24 downto 10);
            for i in 11 to 19 loop s_ow(i) <= s_ow(i - 1); end loop;
            -- 18: steps from the grout edge, Q12: 3 d - 615
            v_a3 := resize(s17_a, 17) + resize(s17_a & '0', 17);
            s18_ph <= signed(resize(v_a3, 18)) - to_signed(615, 18);
            -- 19: + the camera (luma^2, Q10 steps): tile A's steps pushed toward the grout, B's toward its centre,
            -- so a lit tile flattens to A's white / B's edge grey;  the grout edge's distance (> 0 in the tile)
            if s_ow(18) = '1' then s19_ph <= resize(s18_ph, 19) - signed(resize(s_Ls & "00", 19));
            else s19_ph <= resize(s18_ph, 19) + signed(resize(s_Ls & "00", 19)); end if;
            s19_eg <= f_sat14(s18_ph);
            -- 20: the nearest step edge and the tones either side (Bands), or the line's edge (Lines, Fine)
            v_nf := s19_ph(18 downto 12);
            v_fr := unsigned(s19_ph(11 downto 0));
            if v_fr(11) = '1' then v_ds := not v_fr; v_nr := v_nf + 1; else v_ds := v_fr; v_nr := v_nf - 1; end if;
            v_n := f_c03(v_nf); v_nn := f_c03(v_nr);
            if s_lines = '1' or s_fine = '1' then
                -- half a line: 1/12 step (Fine, a line of 1/6), 1/6 step (Lines Bold, a line of 1/3)
                if v_n = v_nn then s20_es <= to_signed(-8191, 14);
                elsif s_fine = '1' then s20_es <= to_signed(341, 14) - signed(resize(v_ds, 14));
                else s20_es <= to_signed(682, 14) - signed(resize(v_ds, 14)); end if;
                if s_lines = '0' then                                   -- Fine on black
                    s20_to <= (others => '0');
                    if s_ink = '1' or s_ow(19) = '0' then
                        if s_ink = '1' then s20_ti <= to_unsigned(96, 7); else s20_ti <= to_unsigned(72, 7); end if;
                    else s20_ti <= to_unsigned(43, 7); end if;
                elsif s_ink = '1' then                                  -- flat tiles, lines of the other colour
                    if s_ow(19) = '0' then s20_to <= to_unsigned(96, 7); s20_ti <= (others => '0');
                    else s20_to <= (others => '0'); s20_ti <= to_unsigned(96, 7); end if;
                else
                    if s_ow(19) = '0' then s20_to <= to_unsigned(66, 7); s20_ti <= to_unsigned(96, 7);
                    else s20_to <= to_unsigned(30, 7); s20_ti <= to_unsigned(8, 7); end if;
                end if;
            else
                if v_n /= v_nn then s20_es <= signed(resize(v_ds, 14));
                else s20_es <= to_signed(8191, 14); end if;
                if s_ink = '1' then
                    if (v_n(0) xor s_ow(19)) = '0' then v_ti := to_unsigned(96, 7); else v_ti := (others => '0'); end if;
                    if (v_nn(0) xor s_ow(19)) = '0' then v_to := to_unsigned(96, 7); else v_to := (others => '0'); end if;
                elsif s_ow(19) = '0' then
                    case to_integer(v_n) is
                        when 0 => v_ti := to_unsigned(63, 7); when 1 => v_ti := to_unsigned(75, 7);
                        when 2 => v_ti := to_unsigned(86, 7); when others => v_ti := to_unsigned(96, 7);
                    end case;
                    case to_integer(v_nn) is
                        when 0 => v_to := to_unsigned(63, 7); when 1 => v_to := to_unsigned(75, 7);
                        when 2 => v_to := to_unsigned(86, 7); when others => v_to := to_unsigned(96, 7);
                    end case;
                else
                    case to_integer(v_n) is
                        when 0 => v_ti := to_unsigned(38, 7); when 1 => v_ti := to_unsigned(30, 7);
                        when 2 => v_ti := to_unsigned(21, 7); when others => v_ti := to_unsigned(12, 7);
                    end case;
                    case to_integer(v_nn) is
                        when 0 => v_to := to_unsigned(38, 7); when 1 => v_to := to_unsigned(30, 7);
                        when 2 => v_to := to_unsigned(21, 7); when others => v_to := to_unsigned(12, 7);
                    end case;
                end if;
                s20_ti <= v_ti; s20_to <= v_to;
            end if;
            s20_eg <= s19_eg;
            -- 21: one edge: the grout's when it is nearer (or the pixel is in the grout), else the step's / line's
            if s20_es(13) = '1' then v_ae := -s20_es; else v_ae := s20_es; end if;
            if s20_eg < v_ae then
                s21_e <= s20_eg;
                if s20_es(13) = '0' then s21_hi <= s20_ti; else s21_hi <= s20_to; end if;
                if s_fine = '1' and s_lines = '0' then s21_lo <= to_unsigned(96, 7); else s21_lo <= (others => '0'); end if;
            else
                s21_e <= s20_es; s21_hi <= s20_ti; s21_lo <= s20_to;
            end if;
            -- 22: x 23/32 for the in-between divisor;  23: (dd x 16) >> kk;  24: the coverage 0 .. 16
            s22_dd <= f_dd(s21_e, c_half);
            s22_hi <= s21_hi; s22_lo <= s21_lo;
            s23_v <= shift_right(shift_left(resize(s22_dd, 18), 4), to_integer(c_kk));
            s23_hi <= s22_hi; s23_lo <= s22_lo;
            s24_cv <= f_cov(s23_v);
            s24_hi <= s23_hi; s24_lo <= s23_lo;
            -- 25, 26: v = lo + (hi - lo) cv / 16
            v_u := signed(resize(s24_hi, 9)) - signed(resize(s24_lo, 9));
            s25_pd <= f_g4(v_u, s24_cv(3 downto 0));
            s25_cf <= s24_cv(4);
            s25_hi <= s24_hi; s25_lo <= s24_lo;
            if s25_cf = '1' then s26_v <= s25_hi;
            else v_g := signed(resize(s25_lo, 13)) + shift_right(s25_pd, 4); s26_v <= unsigned(v_g(6 downto 0)); end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: the gain table (luma^2 * 1.1 K6; the Osc lattice reads
    -- the same table) -> box of 128 (1080) or 64 samples, edge-replicated and
    -- reloaded at every line start -> vertical one-pole in the line store (half
    -- horizontal resolution, Q13.3).  The pixel path takes the one-pole's stream
    -- as it is written (the camera is never shown, so it needs no alignment):
    -- L = Q10 steps.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(12 downto 0);
        variable v_d : signed(16 downto 0);
        variable v_u : signed(16 downto 0);
        variable v_mn : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart from this
            -- field's drift in the zone (riley's lattice carried the last field's x drift on line 0)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the table address (luma >> 2, or the lattice), its output (block RAM register), the sample
            if s_src = '1' then s_ga <= s_osum; else s_ga <= unsigned(data_in.y(9 downto 2)); end if;
            s_sv0 <= data_in.avid;
            s_sv <= s_sv0;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;
            s_p <= r_gt_q(12 downto 0);
            s_sv1 <= s_sv; s_svd <= s_sv1;

            -- box: sum of the last N samples; the samples before the line start count as the first one
            if s_sv1 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else
                bx_wp <= (others => '0');
                if s_lg7 = '1' then bx_rp <= to_unsigned(129, 8); else bx_rp <= to_unsigned(193, 8); end if;
            end if;
            if (s_lg7 = '1' and s_bcnt(7) = '0') or (s_lg7 = '0' and s_bcnt(7 downto 6) = "00") then v_tap := s_sfirst;
            else v_tap := bx_q(12 downto 0); end if;
            if s_sv1 = '1' and s_svd = '0' then
                s_sfirst <= s_p;
                if s_lg7 = '1' then s_bsum <= resize(s_p & "0000000", 20); else s_bsum <= resize(s_p & "000000", 20); end if;
                s_bcnt <= to_unsigned(1, 8);
            elsif s_sv1 = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if (s_lg7 = '1' and s_bcnt(7) = '0') or (s_lg7 = '0' and s_bcnt(7 downto 6) = "00") then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv1; s_bcol <= s_col;

            -- vertical one-pole on the odd columns: q += (box - q) >> sh
            if s_lg7 = '1' then s_eb <= s_bsum(19 downto 7); else s_eb <= s_bsum(18 downto 6); end if;
            s_eq <= lb_q;
            s_ev <= s_bv and not s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb & "000", 17)) - signed(resize(s_eq, 17));
            case to_integer(s_ish) is
                when 3      => v_u := shift_right(v_d, 3);
                when 4      => v_u := shift_right(v_d, 4);
                when others => v_u := shift_right(v_d, 5);
            end case;
            lb_wd <= unsigned(signed(s_eq) + v_u(15 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the stream to the pixel path: the latest and the previous column's values, their mean on the
            -- clock in between
            c_new <= lb_we;
            if lb_we = '1' then c_b <= c_a; c_a <= lb_wd(15 downto 3); end if;
            v_mn := resize(c_a, 14) + resize(c_b, 14);
            if c_new = '1' then s_Ls <= v_mn(13 downto 1); else s_Ls <= c_a; end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: luma = 64 + 9 t + t / 8 (white 940), black outside
    -- pixels 0 .. W + 1 (27), luma [1 2 1] / 4 (30, 31), 13/16 toward black (32),
    -- blanking gate (33).
    -- Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_la : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_la := shift_left(resize(s26_v, 10), 3) + resize(s26_v, 10);
            if s_gav_sr(24) = '1' or s_gav_sr(26) = '1' then
                s27_y <= v_la + resize(to_unsigned(64, 7) + resize(s26_v(6 downto 3), 7), 10);
            else s27_y <= to_unsigned(64, 10); end if;
            s28_y <= s27_y; s29_y <= s28_y;
            s30_ya <= resize(s27_y, 12) + resize(s29_y, 12) + 2;
            s30_yb <= s28_y;
            v_y := s30_ya + shift_left(resize(s30_yb, 12), 1);
            s31_y <= v_y(11 downto 2);
            if s31_y > 64 then v_dl := s31_y - 64; else v_dl := (others => '0'); end if;
            s32_y <= s31_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then s33_y <= s32_y; else s33_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s33_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ogee;
