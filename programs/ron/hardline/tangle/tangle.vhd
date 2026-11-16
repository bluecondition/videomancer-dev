-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tangle v0.1.0 -- Hardline.  Truchet tiles in Cyril Stanley Smith's quarter-arc form, drawn with several
-- concentric arcs per tile so the tiles join into endless meandering ribbons of parallel lines; at the Bend
-- ends, chamfered circuit-board elbows and the "10 PRINT" diagonal maze.  Black and white (U = V = 512).
--
--   tile (i, j) of side c, turn bit o;  (u, v) the tile-local coordinate in 0 .. 1 (u mirrored when o = 1),
--   folded across the anti-diagonal onto the nearer of the two arc corners
--   d    = E + Bend * (L - E)        E = sqrt(u^2 + v^2) (a per-pixel CORDIC),  L = max(u, v, 45/64 (u + v))
--                                    (an octagon: chamfered elbows) below 50 %, u + v (straight diagonals) above
--   tri  = triangle(n d)             0 at n d whole, 1 at n d + 1/2: n lines per tile, centred on k + 1/2, so
--                                    the picture is symmetric about every edge midpoint and lines join for any turns
--   ink  = tri > 1 - duty            (Flat)   |   a ramp of tri (Shade: 4 steps or smooth)
--   blob = (d < 1/2) xor checker xor o        (Fill = Blobs: the classic two-coloured Truchet regions, solid white)
--
--   K1  X        pan, 2 virtual px per count                 K2  Y       the same
--   K3  Period   tile side 40 .. 400 virtual px, exponential
--   K4  Lines    1 .. 8 lines per tile (stepped, hysteresis)
--   K5  Order    0 % every tile one way (waves), 50 % random, 100 % strict alternation (a lattice of circles)
--   K6  Camera   luma^2: per tile it forces the turn bit (bright = the wave direction; 3 counts of hysteresis
--                per tile), per pixel it thickens the lines (Shade: steepens the ramp).  0 = the black-input picture
--   S7  Glide    On: X, Y, Period, Order, Camera, Bend ease 1/16 of the gap per field.  Off: no easing
--   S8  Fill     Lines / Blobs
--   S9  Shade    Flat / Shade (a triangle across each band: round pipes)
--   S10 Weight   Bold (equal bands) / Fine (1/6 duty, at least 2 virtual px);  in Shade: 4 steps / smooth
--   S11 Source   Video / Osc
--   P12 BEND     0 % chamfered elbows, 50 % round arcs, 100 % straight diagonals
--
-- Sections (the integer model .hardline_work/rb_tangle/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         glides with a +-1 hold, switches, the Lines step
--   [C] FIELD SEQUENCER                  one serial multiplier: tile size, the beam / line start values, the
--                                        anti-aliasing gain;  the line engine (tile row, row hash)
--   [D] SOURCE FIELD                     Osc lattice, 32-px box, vertical IIR line store, camera tables
--   [E] TILE ENGINE                      beam DDA, tile hash and turn bit, fold, CORDIC, Bend, bands, coverage
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 towards black, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixel n sits at x = x0 + n xs half virtual px, pixels 0 .. W + 7 are
-- rendered, and output column j is the filter centred on pixel j + 4; the generated avid lands 4 clocks
-- after the delayed source avid, which puts the picture centre on the screen centre.  s_gav_sr(S - 2)
-- flags pixels 0 .. W - 1 at stage S.  One latency (48) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tangle_pkg.all;
use work.all;

architecture tangle of program_top is
    constant C_LAT : integer := 48;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 668;
    constant C_DEF_K5  : integer := 512;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;

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
    signal w_dxm  : unsigned(10 downto 0);     -- the beam's pre-pixel: its distance left of the centre (h)

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
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k): pixels 0 .. W - 1 at stage k + 2
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 20) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: glide accumulators (Q4), the +-1 hold, switches, the Lines step
    signal s_a1 : unsigned(13 downto 0) := to_unsigned(C_DEF_K1 * 16, 14);
    signal s_a2 : unsigned(13 downto 0) := to_unsigned(C_DEF_K2 * 16, 14);
    signal s_a3 : unsigned(13 downto 0) := to_unsigned(C_DEF_K3 * 16, 14);
    signal s_a5 : unsigned(13 downto 0) := to_unsigned(C_DEF_K5 * 16, 14);
    signal s_a6 : unsigned(13 downto 0) := to_unsigned(C_DEF_K6 * 16, 14);
    signal s_ap : unsigned(13 downto 0) := to_unsigned(C_DEF_P12 * 16, 14);
    signal s_h1 : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal s_h2 : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal s_h3 : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal s_h5 : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal s_h6 : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal s_hp : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal s_gm, s_gn : unsigned(13 downto 0) := (others => '0');
    signal s_gr, s_gh, s_gt : unsigned(9 downto 0) := (others => '0');
    signal s_gd, s_gs : signed(14 downto 0) := (others => '0');
    signal s_gj, s_gland : std_logic := '0';
    signal z_gp : unsigned(2 downto 0) := (others => '0');
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_blob, s_shade, s_fine, s_src : std_logic := '0';
    signal s_nz : unsigned(2 downto 0) := to_unsigned(3, 3);       -- lines per tile - 1
    signal s_nzv : std_logic := '0';                               -- the step has been taken from the knob once

    -- [C] the exponential ROM
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(13 downto 0) := (others => '0');
    signal z_fr   : unsigned(3 downto 0) := (others => '0');
    signal z_no   : unsigned(3 downto 0) := (others => '0');
    signal z_f11  : unsigned(7 downto 0) := (others => '0');
    signal z_fine : unsigned(16 downto 0) := (others => '0');
    signal z_A, z_B : signed(15 downto 0) := (others => '0');
    signal z_Aabs, z_Babs : unsigned(14 downto 0) := (others => '0');
    signal z_Aneg, z_Bneg : std_logic := '0';
    signal z_wb   : unsigned(9 downto 0) := (others => '0');
    signal z_N1   : unsigned(24 downto 0) := (others => '0');
    signal z_W1   : unsigned(26 downto 0) := (others => '0');
    signal z_gq   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal z_Wt   : unsigned(17 downto 0) := (others => '0');
    signal z_W5   : unsigned(20 downto 0) := (others => '0');
    signal z_nw   : unsigned(17 downto 0) := (others => '0');
    signal z_pm   : unsigned(4 downto 0) := to_unsigned(17, 5);
    signal z_nrun : std_logic := '0';

    -- [C] per-field constants
    signal c_inv  : unsigned(21 downto 0) := to_unsigned(746382, 22);   -- tiles per h, Q28
    signal c_U0, c_V0 : unsigned(37 downto 0) := (others => '0');       -- tile coordinate at the pre-pixel / line 0 (Q10.28)
    signal c_Ud   : unsigned(24 downto 0) := (others => '0');           -- per pixel
    signal c_Vd   : unsigned(25 downto 0) := (others => '0');           -- per line
    signal c_side : std_logic := '0';                                   -- Bend above 50 %: towards the diagonals
    signal c_T1   : unsigned(7 downto 0) := to_unsigned(128, 8);        -- Order: random-turn threshold
    signal c_T2   : unsigned(8 downto 0) := (others => '0');            -- Order: checker-override threshold
    signal c_thf  : unsigned(7 downto 0) := to_unsigned(213, 8);        -- Fine threshold (tri / 256)
    signal c_rm, c_srm : unsigned(1 downto 0) := (others => '0');       -- edge gain mantissa - 4 (flat / smooth)
    signal c_rs, c_srs : unsigned(3 downto 0) := to_unsigned(4, 4);     -- edge gain right shift

    -- [C] line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_V    : unsigned(37 downto 0) := (others => '0');
    signal e_first : std_logic := '0';
    signal e_jb   : unsigned(15 downto 0) := (others => '0');
    signal n_v, l_v, l_vn : unsigned(11 downto 0) := (others => '0');
    signal n_j, l_j : unsigned(9 downto 0) := (others => '0');
    signal n_jh, l_jh : unsigned(15 downto 0) := (others => '0');
    signal n_samp, l_samp : std_logic := '0';

    -- [E] beam DDAs, tile hash, turn bit
    signal b_U  : unsigned(37 downto 0) := (others => '0');
    signal b_UL : unsigned(33 downto 0) := (others => '0');
    signal s3_i : unsigned(9 downto 0) := (others => '0');
    signal s3_p1, s3_q1, s4_a, s5_a, s6_a, s7_a : unsigned(15 downto 0) := (others => '0');
    signal s3_new : std_logic := '0';
    signal s_i0 : std_logic_vector(3 to 7) := (others => '0');
    signal s_cwa, s_cur, s4_hd, s5_cw, s6_cw, s7_cw : unsigned(7 downto 0) := (others => '0');
    signal s8_o, s8_frc : std_logic := '0';
    signal s_pcur, s4_f, s5_p, s6_p, s7_p : std_logic := '0';
    signal s_nw : std_logic_vector(4 to 8) := (others => '0');
    signal s_rsg : std_logic_vector(8 to 28) := (others => '0');
    -- per-tile camera weight, held per tile column (written on a tile row's first line only)
    type t_hd is array (0 to 255) of unsigned(7 downto 0);
    signal hd_ram : t_hd := (others => (others => '0'));
    attribute no_rw_check of hd_ram : signal is true;
    signal hd_q : unsigned(7 downto 0) := (others => '0');
    signal hd_we : std_logic;
    -- each tile's last camera decision (64 x 32 tiles), for its hysteresis: read at stage 2, rewritten once per
    -- tile on the tile row's first line (where the read is latched at the tile's first pixel)
    type t_fm is array (0 to 2047) of unsigned(1 downto 0);
    signal fm_ram : t_fm := (others => (others => '0'));
    attribute no_rw_check of fm_ram : signal is true;
    signal fm_q, fm_wd : unsigned(1 downto 0) := (others => '0');
    signal fm_wa : unsigned(10 downto 0) := (others => '0');
    signal fm_we : std_logic := '0';

    -- [E] fold and CORDIC
    signal s9_um : unsigned(11 downto 0) := (others => '0');
    signal s9_refl : std_logic := '0';
    signal s10_x, s10_y, s11_mx, s12_mx, s12_sm, s13_mx, s13_sm, s13_t45 : unsigned(11 downto 0) := (others => '0');
    signal s12_t5 : unsigned(14 downto 0) := (others => '0');
    type t_cx is array (0 to 7) of unsigned(15 downto 0);
    type t_cy is array (0 to 7) of signed(15 downto 0);
    signal cx : t_cx := (others => (others => '0'));
    signal cy : t_cy := (others => (others => '0'));
    type t_l12 is array (14 to 20) of unsigned(11 downto 0);
    signal s_L : t_l12 := (others => (others => '0'));
    signal s19_a5 : unsigned(18 downto 0) := (others => '0');
    signal s19_b9 : unsigned(19 downto 0) := (others => '0');
    signal s20_E, s21_E, s22_E, s23_E, s24_E : unsigned(13 downto 0) := (others => '0');
    signal s21_diff, s24_prod : unsigned(11 downto 0) := (others => '0');
    signal s23_lo, s23_hi : unsigned(15 downto 0) := (others => '0');
    signal s25_d : unsigned(12 downto 0) := (others => '0');
    signal s26_p1 : unsigned(13 downto 0) := (others => '0');
    signal s26_p2 : unsigned(15 downto 0) := (others => '0');
    signal s27_t : unsigned(15 downto 0) := (others => '0');

    -- [E] Bend table: T(i) = Bend * i for i = 0 .. 63, refilled every field by an accumulator; the pixel path
    -- looks up both halves of |L - E| (two copies: one read port each)
    type t_ft is array (0 to 255) of unsigned(15 downto 0);
    signal fl_ram, fh_ram : t_ft := (others => (others => '0'));
    attribute no_rw_check of fl_ram : signal is true;
    attribute no_rw_check of fh_ram : signal is true;
    signal fl_q, fh_q, ft_wd : unsigned(15 downto 0) := (others => '0');
    signal ft_wa, ft_i : unsigned(5 downto 0) := (others => '0');
    signal ft_acc : unsigned(15 downto 0) := (others => '0');
    signal ft_we, ft_run : std_logic := '0';

    -- [E] bands and coverage
    signal s28_tri, s29_trp : unsigned(11 downto 0) := (others => '0');
    signal s28_tR : signed(16 downto 0) := (others => '0');
    signal s29_EL, s29_ER, s30_EL, s30_ER : signed(12 downto 0) := (others => '0');
    signal s30_seg, s31_seg, s32_seg, s33_seg : unsigned(1 downto 0) := (others => '0');
    signal s31_XL, s31_XR : signed(15 downto 0) := (others => '0');
    signal s32_hL, s32_hR : signed(17 downto 0) := (others => '0');
    signal s33_cL, s33_cR : unsigned(7 downto 0) := (others => '0');
    signal yl_a : unsigned(8 downto 0);
    signal yl_q, yr_q : unsigned(15 downto 0) := (others => '0');
    signal s35_yl, s35_yr, s36_y, s37_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal s_col : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb8, s_eq : unsigned(7 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(7 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(7 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');
    signal s24_q, s24_qp, s25_l, s27_tb, s28_tb : unsigned(7 downto 0) := (others => '0');
    -- camera tables by source level, refilled every field (a second-order DDA, no multiplier):
    --   A = the camera weight min(255, K6 l^2 / 2^17) (per tile: forces the turn bit)
    --   B = the band threshold (Flat) or the ramp steepening (Shade) for that weight (per pixel)
    type t_wt is array (0 to 255) of unsigned(7 downto 0);
    signal ta_ram, tb_ram : t_wt := (others => (others => '0'));
    attribute no_rw_check of ta_ram : signal is true;
    attribute no_rw_check of tb_ram : signal is true;
    signal ta_q, tb_q, ta_wd, tb_wd, wt_wa, wt_i, wt_ci, wt_cw : unsigned(7 downto 0) := (others => '0');
    signal wt_we, wt_run, wt_v : std_logic := '0';
    signal wt_acc : unsigned(26 downto 0) := (others => '0');
    signal wt_d   : unsigned(19 downto 0) := (others => '0');

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yd : t_u10(1 to 5) := (others => to_unsigned(64, 10));
    signal s38_ya : unsigned(11 downto 0) := (others => '0');
    signal s38_yb : unsigned(9 downto 0) := (others => '0');
    signal s39_y, s40_y, s41_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_TG_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            yl_q   <= C_TG_YS(to_integer(yl_a));
            yr_q   <= C_TG_Y(to_integer(s33_cR));
        end if;
    end process p_rom;
    yl_a <= '1' & s33_seg & s33_cL(7 downto 2) when s_shade = '1' and s_fine = '0' else '0' & s33_cL;

    -- Bend tables: written in the vblank zone only, read by the pixel path (stage 21 address)
    p_flram : process(clk)
    begin
        if rising_edge(clk) then
            fl_q <= fl_ram(to_integer(s21_diff(5 downto 0)));
            if ft_we = '1' then fl_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_flram;
    p_fhram : process(clk)
    begin
        if rising_edge(clk) then
            fh_q <= fh_ram(to_integer(s21_diff(11 downto 6)));
            if ft_we = '1' then fh_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_fhram;

    -- camera tables: written in the vblank zone only
    p_taram : process(clk)
    begin
        if rising_edge(clk) then
            ta_q <= ta_ram(to_integer(s_eq));
            if wt_we = '1' then ta_ram(to_integer(wt_wa)) <= ta_wd; end if;
        end if;
    end process p_taram;
    p_tbram : process(clk)
    begin
        if rising_edge(clk) then
            tb_q <= tb_ram(to_integer(s25_l));
            if wt_we = '1' then tb_ram(to_integer(wt_wa)) <= tb_wd; end if;
        end if;
    end process p_tbram;

    -- per-tile hold: written at each tile's first pixel on a tile row's first line; its read is used on
    -- the other lines only
    hd_we <= l_samp and s3_new and (s_gav_sr(1) or s_gav_sr(9));
    p_hdram : process(clk)
    begin
        if rising_edge(clk) then
            hd_q <= hd_ram(to_integer(b_U(35 downto 28)));
            if hd_we = '1' then hd_ram(to_integer(s3_i(7 downto 0))) <= s_cwa; end if;
        end if;
    end process p_hdram;

    p_fmram : process(clk)
    begin
        if rising_edge(clk) then
            fm_q <= fm_ram(to_integer(l_j(4 downto 0) & b_U(33 downto 28)));
            if fm_we = '1' then fm_ram(to_integer(fm_wa)) <= fm_wd; end if;
        end if;
    end process p_fmram;

    -- box delay: read 32 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    -- luma line store (half horizontal resolution): port 1 = the IIR's own read (2 addresses ahead of its
    -- write; also the tile sampler's early tap), port 2 = the pixel path (well behind it)
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 1)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_e  : signed(15 downto 0);
        variable v_st : signed(14 downto 0);
        variable v_p  : unsigned(31 downto 0);
        variable v_m3 : unsigned(2 downto 0);
        variable v_rq : unsigned(1 downto 0);
        variable v_rs : unsigned(4 downto 0);
        variable v_lm : unsigned(8 downto 0);
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
                s_blob  <= registers_in(6)(1);
                s_shade <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- Lines: 8 steps; a new step is taken 8 counts inside it, or at either end of the knob
                -- (and unconditionally on the first field)
                v_k := unsigned(registers_in(3));
                s_nzv <= '1';
                if v_k(9 downto 7) /= s_nz then
                    if s_nzv = '0' or (v_k(6 downto 3) /= "0000" and v_k(6 downto 3) /= "1111")
                       or (v_k(9 downto 3) = "0000000") or (v_k(9 downto 3) = "1111111") then
                        s_nz <= v_k(9 downto 7);
                    end if;
                end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared glide, seven clocks per channel.  The target is the knob behind a +-1 hold (pot
            -- jitter never moves it).  On: 1/16 of the gap per field, never slower than half a count per
            -- field, landing exactly.  Off: the target itself, this field (the accumulator still tracks).
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                z_gp <= z_gp + 1;
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_a1; s_gh <= s_h1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_a2; s_gh <= s_h2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_a3; s_gh <= s_h3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_a5; s_gh <= s_h5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_a6; s_gh <= s_h6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_ap; s_gh <= s_hp; s_gr <= unsigned(registers_in(7));
                        end case;
                    when 1 =>
                        v_d := signed(resize(s_gr, 11)) - signed(resize(s_gh, 11));
                        if v_d > 1 or v_d < -1 then s_gj <= '1'; else s_gj <= '0'; end if;
                    when 2 =>
                        if s_gj = '1' then s_gt <= s_gr; else s_gt <= s_gh; end if;
                    when 3 =>
                        s_gd <= signed(resize(s_gt & "0000", 15)) - signed(resize(s_gm, 15));
                    when 4 =>
                        v_st := shift_right(s_gd, 4);
                        s_gs <= v_st;
                        if s_gd <= 8 and s_gd >= -8 then s_gland <= '1'; else s_gland <= '0'; end if;
                        if s_gd(14) = '1' and v_st > -8 then s_gs <= to_signed(-8, 15);
                        elsif s_gd(14) = '0' and v_st < 8 then s_gs <= to_signed(8, 15); end if;
                    when 5 =>
                        if s_glide = '0' or s_gland = '1' then s_gn <= s_gt & "0000";
                        else s_gn <= unsigned(signed(s_gm) + s_gs(13 downto 0)); end if;
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_a1 <= s_gn; s_h1 <= s_gt;
                            when 1      => s_a2 <= s_gn; s_h2 <= s_gt;
                            when 2      => s_a3 <= s_gn; s_h3 <= s_gt;
                            when 3      => s_a5 <= s_gn; s_h5 <= s_gt;
                            when 4      => s_a6 <= s_gn; s_h6 <= s_gt;
                            when others => s_ap <= s_gn; s_hp <= s_gt;
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] serial products: slot k is decoded at zc = 62 + 24 k, its operands pre-muxed one clock
            -- later, 16 iterations, result stored at 82 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 62  => z_sel <= to_unsigned(0, 4); z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(1, 4); z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(2, 4); z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(3, 4); z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(4, 4); z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(5, 4); z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(6, 4); z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(7, 4); z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(8, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_a3, 40);   z_fb <= to_unsigned(851, 16);       -- octaves above 40 px, Q12
                    when 1  => z_fa <= resize(r_exf, 40);  z_fb <= resize(z_f11, 16);          -- the last 1/16 of a ROM step
                    when 2  => z_fa <= resize(z_fine, 40); z_fb <= to_unsigned(26214, 16);     -- tiles per h
                    when 3  => z_fa <= resize(c_inv, 40);  z_fb <= resize(z_Aabs, 16);         -- tile coordinate at the pre-pixel
                    when 4  => z_fa <= resize(c_inv, 40);  z_fb <= resize(z_Babs, 16);         -- and at line 0
                    when 5  => z_fa <= resize(c_inv, 40);  z_fb <= resize(s_ys, 16);           -- tiles per line
                    when 6  => z_fa <= resize(c_inv, 40);  z_fb <= resize(s_nz, 16) + 1;       -- lines per h
                    when 7  => z_fa <= resize(z_wb, 40);   z_fb <= to_unsigned(53, 16);        -- |grad d| - 1 on the diagonal side
                    when others => z_fa <= resize(z_W1, 40); z_fb <= resize(z_gq, 16);         -- band phase per pixel
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- the anti-aliasing gain's normaliser: shift the phase step up to bit 17
            if z_nrun = '1' then
                if z_nw(17) = '1' or z_pm = 8 then
                    z_nrun <= '0';
                else
                    z_nw <= z_nw(16 downto 0) & '0';
                    z_pm <= z_pm - 1;
                end if;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- pan (2 virtual px per count): the pre-pixel's and line 0's distance from the lattice origin
                    when 48 => z_A <= signed(resize(w_dxm & "00", 16)) + signed(resize(s_a1, 16)) - 8192;   -- Q2 h
                    when 49 =>
                        z_Aneg <= z_A(15);
                        if z_A(15) = '1' then z_Aabs <= unsigned(resize(-z_A, 15)); else z_Aabs <= unsigned(z_A(14 downto 0)); end if;
                        z_B <= signed(resize(s_a2, 16)) - 3872;                                             -- Q3 px: 540 px + pan
                    when 50 =>
                        z_Bneg <= z_B(15);
                        if z_B(15) = '1' then z_Babs <= unsigned(resize(-z_B, 15)); else z_Babs <= unsigned(z_B(14 downto 0)); end if;
                    when 51 =>                     -- Bend: side and depth 0 .. 512
                        c_side <= s_ap(13);
                        if s_ap(13) = '1' then z_wb <= '0' & s_ap(12 downto 4);
                        else z_wb <= to_unsigned(512, 10) - resize(s_ap(12 downto 4), 10); end if;
                    when 52 =>                     -- Order: thresholds on the two hash bytes
                        if s_a5(13) = '1' then
                            c_T1 <= to_unsigned(128, 8);
                            c_T2 <= resize(shift_right(resize(s_a5(12 downto 4), 10) + 1, 1), 9);
                        else
                            c_T1 <= '0' & s_a5(12 downto 6);
                            c_T2 <= (others => '0');
                        end if;

                    -- tile size: tiles per h = 2^28 / 80 * 2^-oct
                    when 82  => z_oct <= f_acc(23 downto 10);
                    when 83  =>
                        v_e := -signed(resize(z_oct, 16));
                        r_ex_a <= unsigned(v_e(11 downto 4));
                        z_fr <= unsigned(v_e(3 downto 0));
                        z_no <= unsigned(-v_e(15 downto 12));
                    when 84  => z_f11 <= resize(z_fr & "000", 8) + resize(z_fr & '0', 8) + resize(z_fr, 8);
                    when 106 => z_fine <= resize(r_exf, 17) + resize(f_acc(31 downto 16), 17);
                    when 130 =>
                        v_p := f_acc(31 downto 0);
                        case to_integer(z_no) is
                            when 0      => c_inv <= v_p(29 downto 8);
                            when 1      => c_inv <= v_p(30 downto 9);
                            when 2      => c_inv <= v_p(31 downto 10);
                            when 3      => c_inv <= '0' & v_p(31 downto 11);
                            when others => c_inv <= "00" & v_p(31 downto 12);
                        end case;
                    when 131 =>
                        case to_integer(s_xs) is
                            when 4      => c_Ud <= resize(c_inv & "00", 25);
                            when 3      => c_Ud <= resize(c_inv & '0', 25) + resize(c_inv, 25);
                            when others => c_Ud <= resize(c_inv & '0', 25);
                        end case;
                    when 154 => c_U0 <= f_addsub(to_unsigned(0, 38), f_acc(39 downto 2), not z_Aneg);
                    when 178 => c_V0 <= f_addsub(to_unsigned(0, 38), f_acc(39 downto 2), not z_Bneg);
                    when 202 => c_Vd <= f_acc(32 downto 7);
                    when 226 =>
                        z_N1 <= f_acc(24 downto 0);
                    when 227 =>
                        case to_integer(s_xs) is
                            when 4      => z_W1 <= z_N1 & "00";
                            when 3      => z_W1 <= resize(z_N1 & '0', 27) + resize(z_N1, 27);
                            when others => z_W1 <= resize(z_N1 & '0', 27);
                        end case;
                        -- Fine: 1/6 duty, but never under 2 virtual px of line
                        v_lm := to_unsigned(256, 9) - resize(z_N1(24 downto 18), 9);
                        if v_lm > 213 then c_thf <= to_unsigned(213, 8); else c_thf <= v_lm(7 downto 0); end if;
                    when 250 =>
                        if c_side = '1' then z_gq <= to_unsigned(256, 9) + resize(f_acc(17 downto 8), 9);
                        else z_gq <= to_unsigned(256, 9); end if;
                    when 274 => z_Wt <= f_acc(36 downto 19);
                    when 275 => z_W5 <= resize(z_Wt & "00", 21) + resize(z_Wt, 21);
                    when 276 =>                    -- past 1/4 cycle per pixel the edge widens (flat grey at Nyquist)
                        if z_Wt > 32768 then z_nw <= resize(z_W5 - 131072, 18); else z_nw <= z_Wt; end if;
                        z_pm <= to_unsigned(17, 5);
                        z_nrun <= '1';
                    when 290 =>                    -- gain = rm * 2^(2 - rs) / 4 ~ 4096 / step
                        v_m3 := z_nw(16 downto 14);
                        case to_integer(v_m3) is
                            when 0      => v_rq := "00"; v_rs := z_pm - 8;
                            when 1      => v_rq := "11"; v_rs := z_pm - 7;
                            when 2      => v_rq := "10"; v_rs := z_pm - 7;
                            when 3 | 4  => v_rq := "01"; v_rs := z_pm - 7;
                            when others => v_rq := "00"; v_rs := z_pm - 7;
                        end case;
                        c_rm <= v_rq; c_rs <= v_rs(3 downto 0);
                        if v_rs > 8 then c_srm <= v_rq; c_srs <= v_rs(3 downto 0);        -- the smooth ramp: 1/16 at most
                        else c_srm <= "00"; c_srs <= to_unsigned(8, 4); end if;
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
            s_st <= g_start & s_st(0 to 19);
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
    -- [C] LINE ENGINE: kicked at each line start (and once in the zone for line
    -- 0), prepares the NEXT line: the tile row j, the row coordinate v, the
    -- row's hash term and whether the row starts on that line (the tile sampler
    -- runs there).  The beam takes them at the next g_start.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_m : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;     -- after the field constants

            if e_zk = '1' then
                e_V <= c_V0;
                e_lc <= (others => '0');
                e_first <= '1';
            elsif g_start = '1' then
                e_V <= e_V + resize(c_Vd, 38);
                e_lc <= (others => '0');
                e_first <= '0';
                l_v <= n_v; l_vn <= not n_v; l_j <= n_j; l_jh <= n_jh; l_samp <= n_samp;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    n_v <= e_V(27 downto 16);
                    n_j <= e_V(37 downto 28);
                    if e_first = '1' or e_V(37 downto 28) /= n_j then n_samp <= '1'; else n_samp <= '0'; end if;
                    e_jb <= resize(e_V(37 downto 28), 16) + shift_left(resize(e_V(37 downto 28), 16), 6);
                when 2 =>
                    e_jb <= e_jb + x"3C91";
                when 3 =>
                    v_m := e_jb xor shift_right(e_jb, 5);
                    e_jb <= v_m + shift_left(v_m, 3);
                when 4 =>
                    v_m := e_jb xor shift_right(e_jb, 7);
                    n_jh <= v_m + shift_left(v_m, 5);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [E] TILE ENGINE, beam: the tile coordinate along the line.  U (stage 2)
    -- gives the tile column for the hash; a copy of its low bits runs 6 clocks
    -- later (stage 8) to meet the turn bit.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then b_U <= c_U0; else b_U <= b_U + resize(c_Ud, 38); end if;
            if s_st(5) = '1' then b_UL <= c_U0(33 downto 0); else b_UL <= b_UL + resize(c_Ud, 34); end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [E] TILE ENGINE, pixel path (stages 3 .. 37)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_m   : unsigned(15 downto 0);
        variable v_ha, v_hb, v_hc : unsigned(7 downto 0);
        variable v_chk, v_o, v_f : std_logic;
        variable v_u   : unsigned(11 downto 0);
        variable v_s13 : unsigned(12 downto 0);
        variable v_t45 : unsigned(17 downto 0);
        variable v_e25 : unsigned(24 downto 0);
        variable v_df  : signed(14 downto 0);
        variable v_pr  : unsigned(22 downto 0);
        variable v_d15 : unsigned(14 downto 0);
        variable v_c12 : unsigned(11 downto 0);
        variable v_t13 : unsigned(12 downto 0);
        variable v_tr  : signed(16 downto 0);
        variable v_x   : signed(15 downto 0);
        variable v_g   : signed(15 downto 0);
        variable v_h   : signed(17 downto 0);
        variable v_mm  : unsigned(1 downto 0);
        variable v_sh  : unsigned(3 downto 0);
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            -- 3: tile column; the hash's first sums; the tile's first pixel
            s3_i  <= b_U(37 downto 28);
            s3_p1 <= resize(b_U(37 downto 28), 16) + shift_left(resize(b_U(37 downto 28), 16), 4);
            s3_q1 <= shift_left(resize(b_U(37 downto 28), 16), 9) + l_jh;
            if s_st(1) = '1' or b_U(37 downto 28) /= s3_i then s3_new <= '1'; else s3_new <= '0'; end if;
            s_i0(3) <= b_U(28);
            for i in 4 to 7 loop s_i0(i) <= s_i0(i - 1); end loop;
            -- the tile sampler (a tile row's first line): the camera weight at the tile's first pixel
            s_cwa <= ta_q;
            if hd_we = '1' then s_cur <= s_cwa; s_pcur <= fm_q(0); end if;
            s_nw(4) <= hd_we;
            for i in 5 to 8 loop s_nw(i) <= s_nw(i - 1); end loop;
            -- 4 .. 7: the hash (integer adds between xor-shifts); the tile's camera weight rides along
            s4_a <= s3_p1 + s3_q1;
            s4_hd <= hd_q; s4_f <= fm_q(0);
            v_m := s4_a xor shift_right(s4_a, 7);
            s5_a <= v_m + shift_left(v_m, 2);
            if l_samp = '1' then s5_cw <= s_cur; s5_p <= s_pcur; else s5_cw <= s4_hd; s5_p <= s4_f; end if;
            v_m := s5_a xor shift_right(s5_a, 9);
            s6_a <= v_m + shift_left(v_m, 3);
            s6_cw <= s5_cw; s6_p <= s5_p;
            v_m := s6_a xor shift_right(s6_a, 6);
            s7_a <= v_m + shift_left(v_m, 5);
            s7_cw <= s6_cw; s7_p <= s6_p;
            -- 8: the turn bit: Order's random / checker mix, then the camera
            v_m := s7_a xor shift_right(s7_a, 8);
            v_ha := v_m(15 downto 8); v_hb := v_m(7 downto 0); v_hc := v_ha xor v_hb;
            v_chk := s_i0(7) xor l_j(0);
            -- the camera forces the turn where the tile's weight beats its hash, with 3 counts of hysteresis
            -- on the tile's last decision (a still camera never makes a tile pop)
            if s7_cw = 255 then v_f := '1';
            elsif s7_p = '1' then
                if s7_cw /= 0 and resize(v_hc, 9) < resize(s7_cw, 9) + 3 then v_f := '1'; else v_f := '0'; end if;
            else
                if resize(v_hc, 9) + 3 < resize(s7_cw, 9) then v_f := '1'; else v_f := '0'; end if;
            end if;
            s8_frc <= v_f;
            fm_we <= s_nw(8); fm_wa <= l_j(4 downto 0) & b_UL(33 downto 28); fm_wd <= '0' & s8_frc;
            if v_f = '1' then v_o := '1';
            elsif resize(v_hb, 9) < c_T2 then v_o := v_chk;
            elsif v_ha < c_T1 then v_o := '1';
            else v_o := '0'; end if;
            s8_o <= v_o;
            s_rsg(8) <= v_chk xor v_o;
            for i in 9 to 28 loop s_rsg(i) <= s_rsg(i - 1); end loop;
            -- 9: mirror u by the turn bit; which arc corner is nearer
            if s8_o = '1' then v_u := not b_UL(27 downto 16); else v_u := b_UL(27 downto 16); end if;
            v_s13 := resize(v_u, 13) + resize(l_v, 13);
            s9_um <= v_u; s9_refl <= v_s13(12);
            -- 10: fold onto that corner
            if s9_refl = '1' then s10_x <= not s9_um; s10_y <= l_vn;
            else s10_x <= s9_um; s10_y <= l_v; end if;
            -- 11 .. 18: CORDIC vectoring, 8 iterations, 3 guard bits: x -> 1.6468 sqrt(x^2 + y^2)
            cx(0) <= shift_left(resize(s10_x, 16) + resize(s10_y, 16), 3);
            cy(0) <= shift_left(signed(resize(s10_y, 16)) - signed(resize(s10_x, 16)), 3);
            for i in 1 to 7 loop
                if cy(i - 1)(15) = '0' then
                    cx(i) <= unsigned(signed(cx(i - 1)) + shift_right(cy(i - 1), i));
                    cy(i) <= cy(i - 1) - signed(shift_right(cx(i - 1), i));
                else
                    cx(i) <= unsigned(signed(cx(i - 1)) - shift_right(cy(i - 1), i));
                    cy(i) <= cy(i - 1) + signed(shift_right(cx(i - 1), i));
                end if;
            end loop;
            -- 11 .. 14: the straight-line metrics: the octagon max(u, v, 45/64 (u + v)), or u + v
            if s10_x > s10_y then s11_mx <= s10_x; else s11_mx <= s10_y; end if;
            s12_sm <= cx(0)(14 downto 3);
            s12_t5 <= resize(cx(0)(14 downto 3), 15) + shift_left(resize(cx(0)(14 downto 3), 15), 2);
            s12_mx <= s11_mx;
            v_t45 := resize(s12_t5, 18) + shift_left(resize(s12_t5, 18), 3);
            s13_t45 <= v_t45(17 downto 6);
            s13_mx <= s12_mx; s13_sm <= s12_sm;
            if c_side = '1' then s_L(14) <= s13_sm;
            elsif s13_t45 > s13_mx then s_L(14) <= s13_t45;
            else s_L(14) <= s13_mx; end if;
            for i in 15 to 20 loop s_L(i) <= s_L(i - 1); end loop;
            -- 19, 20: undo the CORDIC gain (x 39808 / 2^18): E in 1 / 8192 of a tile
            s19_a5 <= resize(cx(7), 19) + shift_left(resize(cx(7), 19), 2);
            s19_b9 <= resize(cx(7), 20) + shift_left(resize(cx(7), 20), 3);
            v_e25 := shift_left(resize(s19_a5, 25), 6) - resize(s19_b9, 25);
            s20_E <= v_e25(24 downto 11);
            -- 21: |L - E|
            if c_side = '1' then v_df := signed(resize(s_L(20) & '0', 15)) - signed(resize(s20_E, 15));
            else v_df := signed(resize(s20_E, 15)) - signed(resize(s_L(20) & '0', 15)); end if;
            if v_df(14) = '1' then s21_diff <= (others => '0'); else s21_diff <= unsigned(v_df(11 downto 0)); end if;
            s21_E <= s20_E;
            -- 22: Bend * (low half) and Bend * (high half) from the Bend tables (p_flram, p_fhram); 23: fabric copies
            s22_E <= s21_E;
            s23_lo <= fl_q; s23_hi <= fh_q; s23_E <= s22_E;
            -- 24: Bend * |L - E|
            v_pr := resize(s23_lo, 23) + shift_left(resize(s23_hi, 23), 6);
            s24_prod <= v_pr(20 downto 9);
            s24_E <= s23_E;
            -- 25: d
            if c_side = '1' then v_d15 := resize(s24_E, 15) + resize(s24_prod, 15);
            else v_d15 := resize(s24_E, 15) - resize(s24_prod, 15); end if;
            if v_d15(14 downto 13) /= "00" then s25_d <= (others => '1'); else s25_d <= v_d15(12 downto 0); end if;
            -- 26, 27: t = n d (bands, Q3.13)
            if s_nz(0) = '1' then s26_p1 <= s25_d & '0'; else s26_p1 <= '0' & s25_d; end if;
            case s_nz(2 downto 1) is
                when "00"   => s26_p2 <= (others => '0');
                when "01"   => s26_p2 <= resize(s25_d & '0', 16);
                when "10"   => s26_p2 <= resize(s25_d & "00", 16);
                when others => s26_p2 <= resize(s25_d & '0', 16) + resize(s25_d & "00", 16);
            end case;
            s27_t <= resize(s26_p1, 16) + s26_p2;
            -- 28: the band triangle (0 at a whole band, 4095 at the line centre); the blob coordinate t - n / 2
            if s27_t(12) = '1' then s28_tri <= not s27_t(11 downto 0); else s28_tri <= s27_t(11 downto 0); end if;
            s28_tR <= signed(resize(s27_t, 17)) - signed(resize((resize(s_nz, 4) + 1) & x"000", 17));
            -- 29: Flat: distance to the band edge.  Shade: the ramp, steepened by the camera.  The blob edge.
            v_c12 := s28_tb & "0000";
            s29_EL <= signed(resize(s28_tri, 13)) - signed(resize(v_c12, 13));
            if s28_tri < v_c12 then v_t13 := resize(s28_tri & '0', 13);
            else v_t13 := resize(s28_tri, 13) + resize(v_c12, 13); end if;
            if v_t13(12) = '1' then s29_trp <= (others => '1'); else s29_trp <= v_t13(11 downto 0); end if;
            if s_rsg(28) = '1' then v_tr := s28_tR; else v_tr := not s28_tR; end if;
            if v_tr(16) = '0' and v_tr(15 downto 12) /= "0000" then s29_ER <= to_signed(4095, 13);
            elsif v_tr(16) = '1' and v_tr(15 downto 12) /= "1111" then s29_ER <= to_signed(-4096, 13);
            else s29_ER <= v_tr(12 downto 0); end if;
            -- 30: Shade: 4 steps (the nearest step edge) or the smooth ramp about its middle
            s30_ER <= s29_ER;
            if s_shade = '0' then
                s30_EL <= s29_EL; s30_seg <= "00";
            elsif s_fine = '1' then
                s30_EL <= signed(resize(s29_trp, 13)) - 2048; s30_seg <= "00";
            elsif s29_trp < 1536 then
                s30_EL <= signed(resize(s29_trp, 13)) - 1024; s30_seg <= "00";
            elsif s29_trp < 2560 then
                s30_EL <= signed(resize(s29_trp, 13)) - 2048; s30_seg <= "01";
            else
                s30_EL <= signed(resize(s29_trp, 13)) - 3072; s30_seg <= "10";
            end if;
            -- 31: x gain mantissa (4 .. 7)
            if s_shade = '1' and s_fine = '1' then v_mm := c_srm; else v_mm := c_rm; end if;
            v_x := shift_left(resize(s30_EL, 16), 2);
            if v_mm(1) = '1' then v_g := shift_left(resize(s30_EL, 16), 1); else v_g := (others => '0'); end if;
            if v_mm(0) = '1' then v_g := v_g + resize(s30_EL, 16); end if;
            s31_XL <= v_x + v_g;
            v_x := shift_left(resize(s30_ER, 16), 2);
            if c_rm(1) = '1' then v_g := shift_left(resize(s30_ER, 16), 1); else v_g := (others => '0'); end if;
            if c_rm(0) = '1' then v_g := v_g + resize(s30_ER, 16); end if;
            s31_XR <= v_x + v_g;
            s31_seg <= s30_seg;
            -- 32: x 2^(2 - rs)
            if s_shade = '1' and s_fine = '1' then v_sh := c_srs; else v_sh := c_rs; end if;
            s32_hL <= shift_right(shift_left(resize(s31_XL, 18), 2), to_integer(v_sh));
            s32_hR <= shift_right(shift_left(resize(s31_XR, 18), 2), to_integer(c_rs));
            s32_seg <= s31_seg;
            -- 33: coverage = clamp(128 + that)
            v_h := s32_hL;
            if v_h(17) = '0' and v_h(16 downto 7) /= "0000000000" then s33_cL <= (others => '1');
            elsif v_h(17) = '1' and v_h(16 downto 7) /= "1111111111" then s33_cL <= (others => '0');
            else s33_cL <= unsigned((not v_h(7)) & v_h(6 downto 0)); end if;
            v_h := s32_hR;
            if v_h(17) = '0' and v_h(16 downto 7) /= "0000000000" then s33_cR <= (others => '1');
            elsif v_h(17) = '1' and v_h(16 downto 7) /= "1111111111" then s33_cR <= (others => '0');
            else s33_cR <= unsigned((not v_h(7)) & v_h(6 downto 0)); end if;
            s33_seg <= s32_seg;
            -- 34: the luma ROMs (p_rom); 35: fabric copies
            s35_yl <= yl_q(9 downto 0); s35_yr <= yr_q(9 downto 0);
            -- 36: Blobs: the solid region over the lines
            if s_blob = '1' and s35_yr > s35_yl then s36_y <= s35_yr; else s36_y <= s35_yl; end if;
            -- 37: black outside pixels 0 .. W + 7
            v_win := s_gav_sr(34) or s_gav_sr(42);
            if v_win = '1' then s37_y <= s36_y; else s37_y <= to_unsigned(64, 10); end if;

            -- Bend table fill (zone, zc 300 .. 364): T(i) = Bend * i by accumulation
            ft_we <= '0';
            if s_zone = '1' and s_zc = 300 then
                ft_run <= '1'; ft_i <= (others => '0'); ft_acc <= (others => '0');
            elsif ft_run = '1' then
                ft_we <= '1'; ft_wa <= ft_i; ft_wd <= ft_acc;
                ft_acc <= ft_acc + resize(z_wb, 16); ft_i <= ft_i + 1;
                if ft_i = 63 then ft_run <= '0'; end if;
            end if;

            -- the luma store's read column for the pixel path
            if s_st(19) = '1' or s_st(20) = '1' then lb_rc <= (others => '0'); else lb_rc <= lb_rc + 1; end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 32-px box, edge-replicated
    -- and reloaded at every line start -> vertical 1/8 IIR in the line store
    -- (half horizontal resolution).  The pixel path reads the store, interpolates
    -- and looks the camera table up: stages 23 .. 28.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d : signed(9 downto 0);
        variable v_a : unsigned(8 downto 0);
        variable v_c : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 samples; the samples before the line start count as the first one
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(225, 8); end if;
            if s_bcnt < 32 then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                s_bsum <= shift_left(resize(s_sl, 13), 5);
                s_bcnt <= to_unsigned(1, 6);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 32 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on the odd columns: q += (box - q + 4) >> 3
            s_eb8 <= s_bsum(12 downto 5); s_eq <= lb_q1;
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb8, 10)) - signed(resize(s_eq, 10)) + 4;
            v_a := unsigned(signed(resize(s_eq, 9)) + resize(shift_right(v_d, 3), 9));
            lb_wd <= v_a(7 downto 0);
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- pixel path: 24 fabric copies of this and the last read; 25 the mean (a linear interpolation of
            -- the half-resolution store); 26 camera table B; 27, 28 fabric copies
            s24_q <= lb_q2; s24_qp <= s24_q;
            v_a := resize(s24_q, 9) + resize(s24_qp, 9) + 1;
            s25_l <= v_a(8 downto 1);
            s27_tb <= tb_q;
            s28_tb <= s27_tb;

            -- camera table fill (zone, zc 300 .. 557): weight(l) = min(255, K6 l^2 / 2^17), the square by
            -- second differences; one clock later the threshold / steepening for that weight
            wt_v <= '0';
            if s_zone = '1' and s_zc = 300 then
                wt_run <= '1'; wt_i <= (others => '0'); wt_acc <= (others => '0'); wt_d <= resize(s_a6(13 downto 4), 20);
            elsif wt_run = '1' then
                wt_v <= '1'; wt_ci <= wt_i;
                if wt_acc(26 downto 25) /= "00" then wt_cw <= (others => '1'); else wt_cw <= wt_acc(24 downto 17); end if;
                wt_acc <= wt_acc + wt_d; wt_d <= wt_d + resize(s_a6(13 downto 4) & '0', 20); wt_i <= wt_i + 1;
                if wt_i = 255 then wt_run <= '0'; end if;
            end if;
            wt_we <= wt_v; wt_wa <= wt_ci; ta_wd <= wt_cw;
            v_c := wt_cw;
            if s_shade = '1' then
                tb_wd <= '0' & v_c(7 downto 1);                                                -- the ramp steepens
            elsif s_fine = '1' then
                tb_wd <= c_thf - (resize(v_c(7 downto 1), 8) - resize(v_c(7 downto 4), 8));    -- duty 1/6 .. 0.6
            else
                tb_wd <= to_unsigned(128, 8) - (resize(v_c(7 downto 2), 8) + resize(v_c(7 downto 3), 8));   -- duty 1/2 .. 0.87
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: luma [1 2 1] / 4 centred on pixel j + 4, luma
    -- 13/16 toward black, blanking gate.  Chroma is a constant 512.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- luma taps y(n - 3), y(n - 4), y(n - 5)
            s_yd(1) <= s37_y;
            for i in 2 to 5 loop s_yd(i) <= s_yd(i - 1); end loop;
            s38_ya <= resize(s_yd(3), 12) + resize(s_yd(5), 12) + 2;
            s38_yb <= s_yd(4);
            v_y := s38_ya + shift_left(resize(s38_yb, 12), 1);
            s39_y <= v_y(11 downto 2);
            -- luma 13/16 toward black
            if s39_y > 64 then v_dl := s39_y - 64; else v_dl := (others => '0'); end if;
            s40_y <= s39_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then s41_y <= s40_y; else s41_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s41_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture tangle;
