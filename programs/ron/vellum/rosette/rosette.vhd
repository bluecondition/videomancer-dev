-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Rosette v0.1.0 -- Vellum.  Offset-printing halftone screens and the rosettes their overlap forms (screen
-- angles 15 / 45 / 75 degrees), Lichtenstein's Ben-Day dots: three translucent square screens of round dots,
-- white films on black.  Luma only: U = V = 512 on every pixel.
--
--   screen k at 45 - s, 45, 45 + s degrees (s = the P12 spread), pitch P, dot radius r (K4 + the camera)
--   (u, v)_k = R(theta_k) (x - centre) / P + phase_k (+ offset_k)         cells, modulo 1 (20-bit DDAs)
--   d_k^2    = SQ(fold u) + SQ(fold v)        squares in anti-aliasing px^2, from per-field tables
--   Bold     cov = clamp(1/2 + (r^2 - d^2) / 2r)                            (the signed distance at the edge)
--   Fine     cov = clamp(1/2 + h - |r^2 + h^2 - d^2| / 2r)                 (a ring of half width h)
--   Grey     luma = PWL(cov_1 + cov_2 + cov_3): 0 / 0.40 / 0.72 / 1 (Two: 0 / 0.55 / 1)
--   Ink      luma = triangle(sum mod 2)  (XOR, anti-aliased)
--
--   K1  X        offset of screen 2 (+) and screen 3 (-) against screen 1, +-1/2 pitch across: the rosette type
--   K2  Y        the same, down
--   K3  Scale    pitch 14 .. 110 virtual px (exponential)
--   K4  Dot      dot radius 0.06 .. 0.52 cell (pinpoints .. touching, merging into lace), at least 1.25 v-px
--   K5  Drift    bipolar speed (Hypnos): screens slide along their own axes at +1, -1, +1/2
--   K6  Camera   halftone: the box-filtered luma swells the dot area toward a 3/4-pitch radius
--   S7  Glide    Off / On      S8 Screens  Three / Two      S9 Tone  Grey / Ink      S10 Weight  Bold / Fine
--   S11 Source   Video / Osc
--   P12 ANGLE    the screens' spread: 0 = one grid, small = giant moire, 50 % = 30 degrees (15 / 45 / 75),
--                100 % = 45 degrees (0 / 45 / 90)
--
-- Screen 3 at 45 + s is screen 1 at 45 - s mirrored about the diagonal, so its steps and start values are
-- screen 1's products swapped; screen 2 at 45 degrees has cos = sin.  The squares and r^2 + h^2 are
-- pre-scaled per field by 1 / 2r (r = the slope radius, one serial division), so they come out in 1/16 px
-- of edge distance and the pixel path is subtractions only.  The slope radius is r0 moved toward the
-- camera's mean level of the last fields, so a black input is exact; the camera's dot area comes from a
-- 256-entry table by camera level.
--
-- The chassis is the Kinetic riley shell (via Hardline tweed):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1-count holds, glides, switches, the Drift speed (Hypnos)
--   [C] ENGINE                           per-field sequencer (one serial multiplier, one serial divider),
--                                        table fills, line DDAs and shared pixel offsets
--   [D] SOURCE FIELD                     luma (or the Osc lattice) -> box -> vertical IIR -> camera level
--   [E] SCREENS                          squares, discs or rings, coverage, tone, fade
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixel n sits at x = x0 + n xs half virtual px.  The output register is
-- at stage C_LAT - 3 and the generated avid lands 4 clocks after the delayed source avid (the shell's
-- parity-locked start), so output column j shows pixel j + 4 on screen column j + 4.  s_gav_sr(S - 2) flags
-- pixels 0 .. W - 1 at stage S.  One latency (32) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.rosette_pkg.all;
use work.all;

architecture rosette of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

    -- TOML defaults: the glides and holds start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 800;
    constant C_DEF_K4  : integer := 400;
    constant C_DEF_K5  : integer := 680;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;

    constant C_NSLOT : integer := 44;      -- serial-product slots 0 .. 44
    constant C_DV_ZC : integer := 984;     -- the divider (after slot 38 has the slope radius and q)
    constant C_SQ_ZC : integer := 1140;    -- the squares and camera table fills start here (after slot 44)

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
    -- the folded lattice fraction: |frac - 1/2| mirrored into 0 .. 511 (index i stands for (i + 1/2) / 1024)
    function f_fold(f : unsigned(9 downto 0)) return unsigned is
    begin
        if f(9) = '1' then return not f(8 downto 0); end if;
        return f(8 downto 0);
    end function;
    -- the common scale shift: q = max(0, ceil(log2 rk) - 8)
    function f_q(rk : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := rk - 1;
        if v(15) = '1' then return to_unsigned(7, 3);       -- (never: rk < 32768)
        elsif v(14) = '1' then return to_unsigned(7, 3);
        elsif v(13) = '1' then return to_unsigned(6, 3);
        elsif v(12) = '1' then return to_unsigned(5, 3);
        elsif v(11) = '1' then return to_unsigned(4, 3);
        elsif v(10) = '1' then return to_unsigned(3, 3);
        elsif v(9) = '1' then return to_unsigned(2, 3);
        elsif v(8) = '1' then return to_unsigned(1, 3);
        else return to_unsigned(0, 3);
        end if;
    end function;

    attribute no_rw_check : boolean;

    type t_u20x3 is array (0 to 2) of unsigned(19 downto 0);
    type t_u12x6 is array (0 to 5) of unsigned(11 downto 0);
    type t_u16x3 is array (0 to 2) of unsigned(15 downto 0);
    type t_s17x3 is array (0 to 2) of signed(16 downto 0);
    type t_s18x3 is array (0 to 2) of signed(17 downto 0);
    type t_s10x3 is array (0 to 2) of signed(9 downto 0);
    type t_s17p  is array (0 to 2) of signed(16 downto 0);
    type t_u7x3  is array (0 to 2) of unsigned(6 downto 0);
    type t_u5x3  is array (0 to 2) of unsigned(4 downto 0);

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_wc   : unsigned(1 downto 0) := (others => '0');     -- AA unit: 1 / 1.5 / 2 / 3 virtual px
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
    signal s_st       : std_logic_vector(0 to 9) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: +-1-count holds, glide accumulators (Q10.6), switches, the Drift speed glide (Q6)
    signal s_h1 : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal s_h2 : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal s_h3 : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal s_h4 : unsigned(9 downto 0) := to_unsigned(C_DEF_K4, 10);
    signal s_h6 : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal s_hp : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal s_a1 : unsigned(15 downto 0) := to_unsigned(C_DEF_K1 * 64, 16);
    signal s_a2 : unsigned(15 downto 0) := to_unsigned(C_DEF_K2 * 64, 16);
    signal s_a3 : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);
    signal s_a4 : unsigned(15 downto 0) := to_unsigned(C_DEF_K4 * 64, 16);
    signal s_a6 : unsigned(15 downto 0) := to_unsigned(C_DEF_K6 * 64, 16);
    signal s_ap : unsigned(15 downto 0) := to_unsigned(C_DEF_P12 * 64, 16);
    signal s_gm : unsigned(15 downto 0) := (others => '0');
    signal s_gr, s_gh, s_gt : unsigned(9 downto 0) := (others => '0');
    signal s_hd : signed(10 downto 0) := (others => '0');
    signal s_gd : signed(16 downto 0) := (others => '0');
    signal z_gp : unsigned(2 downto 0) := (others => '0');
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_two, s_ink, s_fine, s_src : std_logic := '0';
    signal s_k5  : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal s_vtg : signed(16 downto 0) := (others => '0');
    signal s_vgs : signed(16 downto 0) := to_signed((C_DEF_K5 - 512) * 64, 17);
    signal s_vneg, s_vz : std_logic := '0';
    signal s_av  : unsigned(9 downto 0) := (others => '0');
    signal s_u   : unsigned(9 downto 0) := (others => '0');
    signal s_ls  : unsigned(9 downto 0) := (others => '0');
    signal s_scnt : unsigned(2 downto 0) := (others => '0');

    -- [C] ROM: C_RS_TAB (2^(i/256), then a quarter sine)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier, slot counter and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_ph   : unsigned(4 downto 0) := (others => '0');
    signal z_slot : unsigned(5 downto 0) := (others => '0');
    signal z_run, z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_tlow : std_logic := '1';
    signal z_a1   : unsigned(15 downto 0) := (others => '0');
    signal z_ang  : unsigned(15 downto 0) := (others => '0');
    signal z_i8, z_i8n : std_logic := '0';
    signal z_s0, z_s1, z_dS : unsigned(15 downto 0) := (others => '0');
    signal z_oxn, z_oyn, z_s5n, z_s6n : std_logic := '0';
    signal z_oxm, z_oym : unsigned(14 downto 0) := (others => '0');
    signal z_s5m, z_s6m : unsigned(15 downto 0) := (others => '0');
    signal z_ss   : unsigned(15 downto 0) := (others => '0');      -- the spread s
    signal z_r0c, z_rg, z_g, z_kk, z_A0, z_Ac, z_rk : unsigned(15 downto 0) := (others => '0');
    signal z_Ad   : unsigned(16 downto 0) := (others => '0');      -- Acap - A0, then A0 + h^2
    signal z_Ah   : unsigned(16 downto 0) := (others => '0');      -- Acap + h^2
    signal z_Ga, z_H2, z_q0 : unsigned(17 downto 0) := (others => '0');
    -- the start-value accumulators (two, so each product serves two screens): add / clear / operand select
    signal zA, zB : unsigned(19 downto 0) := (others => '0');
    signal za_en, za_clr, za_sub, zb_en, zb_clr, zb_sub : std_logic := '0';
    signal za_sel, zb_sel : unsigned(2 downto 0) := (others => '0');   -- 0 slice 27:4, 1 slice 30:7, 2..4 phases
    signal ld_a, ld_b : unsigned(1 downto 0) := (others => '0');        -- which line DDAs take zA / zB
    signal ld_go : std_logic := '0';

    -- [C] per-field constants
    signal c_sh   : unsigned(1 downto 0) := (others => '0');       -- 2^-oct = M * 2^-sh
    signal c_inv  : unsigned(21 downto 0) := (others => '0');      -- cells per h, Q24
    signal c_pa   : unsigned(15 downto 0) := to_unsigned(4096, 16); -- pitch in AA px, Q8
    signal c_rcap : unsigned(15 downto 0) := (others => '0');      -- the camera's radius cap, Q8
    signal c_tf   : unsigned(1 downto 0) := (others => '0');       -- fade to mid-grey: none, half, flat
    signal c_hb   : unsigned(16 downto 0) := (others => '0');      -- Fine: h in 1/16 px (Bold: 0)
    signal c_rmin, c_hmin : unsigned(15 downto 0) := (others => '0');   -- floors by video mode
    signal c_kpa  : unsigned(15 downto 0) := to_unsigned(7168, 16);    -- pitch to AA px by video mode
    signal c_S1, c_C1 : unsigned(15 downto 0) := (others => '0');  -- sin, cos of 45 - s, Q15
    signal c_cu1, c_su1, c_cu2 : unsigned(21 downto 0) := (others => '0');   -- cos / P, sin / P (Q24 per h)
    signal c_SA, c_SB, c_SC, c_TA, c_TB, c_TC : unsigned(19 downto 0) := (others => '0');   -- DDA steps, Q20
    signal c_r0, c_h : unsigned(15 downto 0) := (others => '0');   -- radius, ring half width (Q8 AA px)
    signal c_q    : unsigned(2 downto 0) := (others => '0');       -- the slope radius's scale shift
    signal c_Km   : unsigned(6 downto 0) := to_unsigned(32, 7);    -- 2^(13 + q) / rk
    signal c_Kf   : unsigned(15 downto 0) := (others => '0');      -- Km 2^(8 - q): X / 2rk = X Kf >> 14
    signal c_A0s, c_Acs, c_Gs : unsigned(15 downto 0) := (others => '0');   -- scaled r^2 + h^2, its cap, gain
    signal c_cq   : unsigned(17 downto 0) := (others => '0');      -- scaled squares-table step, Q20
    signal c_nh   : std_logic := '0';                              -- Fine: no hole (r <= h)
    signal c_spd  : unsigned(19 downto 0) := (others => '0');      -- drift step per field, Q20 pitches
    signal c_sneg : std_logic := '0';
    signal c_ph1, c_ph2, c_ph3 : unsigned(19 downto 0) := (others => '0');   -- drift phases
    signal c_Lm   : unsigned(7 downto 0) := (others => '0');       -- mean camera level of the last fields

    -- [C] serial divider: Km = min(127, 2^(13 + q) / rho), 7 quotient bits
    signal dv_R   : unsigned(20 downto 0) := (others => '0');
    signal dv_D   : unsigned(21 downto 0) := (others => '0');
    signal dv_Q   : unsigned(6 downto 0) := (others => '0');
    signal dv_n   : unsigned(3 downto 0) := (others => '0');
    signal dv_sat, dv_st : std_logic := '0';

    -- [C] table fills
    signal sf_run : std_logic := '0';
    signal sf_i   : unsigned(8 downto 0) := (others => '0');
    signal sf_S   : unsigned(35 downto 0) := (others => '0');
    signal sf_D   : unsigned(29 downto 0) := (others => '0');
    signal sq_we  : std_logic := '0';
    signal sq_wa  : unsigned(8 downto 0) := (others => '0');
    signal sq_wd  : unsigned(15 downto 0) := (others => '0');
    signal rf_run : std_logic := '0';
    signal rf_acc : unsigned(23 downto 0) := (others => '0');
    signal rt_we  : std_logic := '0';
    signal rt_wa  : unsigned(7 downto 0) := (others => '0');
    signal rt_wd  : unsigned(15 downto 0) := (others => '0');

    -- [C] line DDAs, shared pixel offsets, the lattice coordinates (top 12 bits)
    signal l_U, l_V : t_u20x3 := (others => (others => '0'));
    signal l_first : std_logic := '0';
    signal o_A, o_B, o_C : unsigned(19 downto 0) := (others => '0');
    signal s10_c  : t_u12x6 := (others => (others => '0'));

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_p, s_pfirst : unsigned(7 downto 0) := (others => '0');
    signal s_avd : std_logic_vector(1 to 5) := (others => '0');    -- data_in.avid delayed
    signal s_col, s_pcol, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal c_b64 : std_logic := '1';                               -- box length 64 (1080 lines) / 32 pixels
    signal c_brp : unsigned(7 downto 0) := to_unsigned(193, 8);
    signal c_iir : unsigned(2 downto 0) := to_unsigned(5, 3);
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_bsum : unsigned(13 downto 0) := (others => '0');
    signal s_ev, s_fl : std_logic := '0';
    signal s_eb, s_eq : unsigned(11 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(11 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(11 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal h_cur, h_prev, s8_q : unsigned(11 downto 0) := (others => '0');
    signal h_ph : std_logic := '0';
    signal lm_acc : unsigned(27 downto 0) := (others => '0');      -- running mean of the camera level, Q20

    -- [E] squares tables (six identical copies: one read port each) and the camera table
    type t_sq is array (0 to 511) of unsigned(15 downto 0);
    signal sq0_ram, sq1_ram, sq2_ram, sq3_ram, sq4_ram, sq5_ram : t_sq := (others => (others => '0'));
    attribute no_rw_check of sq0_ram : signal is true;
    attribute no_rw_check of sq1_ram : signal is true;
    attribute no_rw_check of sq2_ram : signal is true;
    attribute no_rw_check of sq3_ram : signal is true;
    attribute no_rw_check of sq4_ram : signal is true;
    attribute no_rw_check of sq5_ram : signal is true;
    type t_u16x6 is array (0 to 5) of unsigned(15 downto 0);
    signal sq_q, sq_f : t_u16x6 := (others => (others => '0'));
    type t_rt is array (0 to 255) of unsigned(15 downto 0);
    signal rt_ram : t_rt := (others => (others => '0'));
    attribute no_rw_check of rt_ram : signal is true;
    signal rt_a : unsigned(7 downto 0) := (others => '0');
    signal rt_q, rt_f, rt_g, s13_A : unsigned(15 downto 0) := (others => '0');

    -- [E] pixel path
    signal s13_d2 : t_u16x3 := (others => (others => '0'));
    signal s14_e  : t_s17x3 := (others => (others => '0'));
    signal s15_x  : t_s17x3 := (others => (others => '0'));
    signal s16_c  : t_u5x3 := (others => (others => '0'));
    signal s17_S  : unsigned(5 downto 0) := (others => '0');
    signal tn_a   : unsigned(7 downto 0) := (others => '0');
    signal tn_q, tn_f : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s21_o  : unsigned(9 downto 0) := (others => '0');
    signal s22_o  : unsigned(9 downto 0) := (others => '0');
    signal s23_y  : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yd : t_u10(1 to 3) := (others => to_unsigned(64, 10));
    signal s26_ya : unsigned(11 downto 0) := (others => '0');
    signal s26_yb : unsigned(9 downto 0) := (others => '0');
    signal s27_y, s28_y, s29_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_RS_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            tn_q   <= C_RS_TONE(to_integer(tn_a));
        end if;
    end process p_rom;

    -- box delay: read (box length) samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_avd(2) = '1' then bx_ram(to_integer(bx_wp)) <= s_p; end if;
        end if;
    end process p_bxram;

    -- camera-field line store (half horizontal resolution): read 3 pixels ahead of the write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_pcol(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- squares tables: written in the vblank zone only, read by the pixel path (the folded stage-10 coordinate)
    p_sq0 : process(clk) begin
        if rising_edge(clk) then
            sq_q(0) <= sq0_ram(to_integer(f_fold(s10_c(0)(11 downto 2))));
            if sq_we = '1' then sq0_ram(to_integer(sq_wa)) <= sq_wd; end if;
        end if;
    end process p_sq0;
    p_sq1 : process(clk) begin
        if rising_edge(clk) then
            sq_q(1) <= sq1_ram(to_integer(f_fold(s10_c(1)(11 downto 2))));
            if sq_we = '1' then sq1_ram(to_integer(sq_wa)) <= sq_wd; end if;
        end if;
    end process p_sq1;
    p_sq2 : process(clk) begin
        if rising_edge(clk) then
            sq_q(2) <= sq2_ram(to_integer(f_fold(s10_c(2)(11 downto 2))));
            if sq_we = '1' then sq2_ram(to_integer(sq_wa)) <= sq_wd; end if;
        end if;
    end process p_sq2;
    p_sq3 : process(clk) begin
        if rising_edge(clk) then
            sq_q(3) <= sq3_ram(to_integer(f_fold(s10_c(3)(11 downto 2))));
            if sq_we = '1' then sq3_ram(to_integer(sq_wa)) <= sq_wd; end if;
        end if;
    end process p_sq3;
    p_sq4 : process(clk) begin
        if rising_edge(clk) then
            sq_q(4) <= sq4_ram(to_integer(f_fold(s10_c(4)(11 downto 2))));
            if sq_we = '1' then sq4_ram(to_integer(sq_wa)) <= sq_wd; end if;
        end if;
    end process p_sq4;
    p_sq5 : process(clk) begin
        if rising_edge(clk) then
            sq_q(5) <= sq5_ram(to_integer(f_fold(s10_c(5)(11 downto 2))));
            if sq_we = '1' then sq5_ram(to_integer(sq_wa)) <= sq_wd; end if;
        end if;
    end process p_sq5;

    -- camera table: written in the vblank zone only, read by the pixel path (stage 9 address)
    p_rtram : process(clk)
    begin
        if rising_edge(clk) then
            rt_q <= rt_ram(to_integer(rt_a));
            if rt_we = '1' then rt_ram(to_integer(rt_wa)) <= rt_wd; end if;
        end if;
    end process p_rtram;

    -- [A] the beam's pre-pixel by video mode
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_17  : signed(17 downto 0);
        variable v_st  : signed(16 downto 0);
        variable v_i9  : unsigned(8 downto 0);
        variable v_v   : signed(10 downto 0);
        variable v_sp  : unsigned(19 downto 0);
        variable v_op  : unsigned(19 downto 0);
        variable v_bs  : unsigned(19 downto 0);
        variable v_16  : unsigned(16 downto 0);
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
                s_two   <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_k5    <= unsigned(registers_in(4));
                c_Lm    <= lm_acc(27 downto 20);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is                  -- Osc step, box length, its read offset
                    when 4      => s_oxs <= to_unsigned(204, 9); c_b64 <= '0'; c_brp <= to_unsigned(225, 8);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_b64 <= '0'; c_brp <= to_unsigned(225, 8);
                    when others => s_oxs <= to_unsigned(102, 9); c_b64 <= '1'; c_brp <= to_unsigned(193, 8);
                end case;
                -- the anti-aliasing unit: 1 v-px at 1080p, 1.5 at 1080i / 720p, 2 at 480p / 576p, 3 at 480i / 576i
                if s_xsr = 4 then
                    if s_ys >= 960 then s_wc <= "11"; else s_wc <= "10"; end if;
                elsif s_xsr = 3 or s_ys >= 512 then s_wc <= "01";
                else s_wc <= "00"; end if;
                case to_integer(s_wc) is   -- (last field's class: the mode changes rarely)
                    when 0      => c_kpa <= to_unsigned(7168, 16); c_rmin <= to_unsigned(320, 16); c_hmin <= to_unsigned(256, 16);
                    when 1      => c_kpa <= to_unsigned(4779, 16); c_rmin <= to_unsigned(213, 16); c_hmin <= to_unsigned(171, 16);
                    when 2      => c_kpa <= to_unsigned(3584, 16); c_rmin <= to_unsigned(160, 16); c_hmin <= to_unsigned(128, 16);
                    when others => c_kpa <= to_unsigned(2389, 16); c_rmin <= to_unsigned(107, 16); c_hmin <= to_unsigned(85, 16);
                end case;
                -- the vertical IIR's shift: about 32 .. 48 virtual lines in every mode
                if s_ys >= 960 then c_iir <= to_unsigned(3, 3);
                elsif s_ys >= 480 then c_iir <= to_unsigned(4, 3);
                else c_iir <= to_unsigned(5, 3); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared hold + glide, six clocks per channel (X, Y, Scale, Dot, Camera, Angle):
            --   hold: the knob is re-read only when it has moved more than one count (pot jitter)
            --   glide On: 1/16 of the gap per field, never slower than one count, landing exactly
            --   glide Off: the accumulator takes the held value of THIS field start (no easing, no lag)
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
                            when 3      => s_gm <= s_a4; s_gh <= s_h4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_a6; s_gh <= s_h6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_ap; s_gh <= s_hp; s_gr <= unsigned(registers_in(7));
                        end case;
                    when 1 =>
                        s_hd <= signed(resize(s_gr, 11)) - signed(resize(s_gh, 11));
                    when 2 =>
                        if s_hd > 1 or s_hd < -1 then s_gt <= s_gr; else s_gt <= s_gh; end if;
                    when 3 =>
                        s_gd <= signed(resize(s_gt & "000000", 17)) - signed(resize(s_gm, 17));
                        case to_integer(z_gc) is
                            when 0      => s_h1 <= s_gt;
                            when 1      => s_h2 <= s_gt;
                            when 2      => s_h3 <= s_gt;
                            when 3      => s_h4 <= s_gt;
                            when 4      => s_h6 <= s_gt;
                            when others => s_hp <= s_gt;
                        end case;
                    when 4 =>
                        if s_glide = '0' or (s_gd <= 64 and s_gd >= -64) then
                            null;                                          -- the whole gap
                        elsif s_gd < 1024 and s_gd > -1024 then
                            if s_gd(16) = '1' then s_gd <= to_signed(-64, 17); else s_gd <= to_signed(64, 17); end if;
                        else
                            s_gd <= shift_right(s_gd, 4);
                        end if;
                    when others =>
                        v_st := signed(resize(s_gm, 17)) + s_gd;
                        case to_integer(z_gc) is
                            when 0      => s_a1 <= unsigned(v_st(15 downto 0));
                            when 1      => s_a2 <= unsigned(v_st(15 downto 0));
                            when 2      => s_a3 <= unsigned(v_st(15 downto 0));
                            when 3      => s_a4 <= unsigned(v_st(15 downto 0));
                            when 4      => s_a6 <= unsigned(v_st(15 downto 0));
                            when others => s_ap <= unsigned(v_st(15 downto 0));
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [B] Drift (Hypnos v1.1.0 SPEED): glide 1/8 (Off: this field's knob), |v| < 24 still, else
            -- 2^-12 .. 2^-4 pitch per field (Q20), exponential (mantissa from the 2^(i/256) table, then a serial
            -- shift by the octave); the three phases advance once per field at +1, -1, +1/2
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 8  => s_vtg <= shift_left(resize(signed('0' & s_k5) - 512, 17), 6);
                    when 9  =>
                        if s_glide = '1' then s_vgs <= s_vgs + shift_right(s_vtg - s_vgs, 3);
                        else s_vgs <= s_vtg; end if;
                    when 10 =>
                        v_v := resize(shift_right(s_vgs, 6), 11);
                        s_vneg <= v_v(10);
                        if v_v(10) = '1' then s_av <= unsigned(resize(-v_v, 10)); else s_av <= unsigned(v_v(9 downto 0)); end if;
                    when 11 =>
                        if s_av < 24 then s_vz <= '1'; s_u <= (others => '0');
                        else s_vz <= '0'; s_u <= s_av - 24; end if;
                    when 12 =>
                        s_ls <= s_u + shift_right(s_u, 5) + shift_right(s_u, 6);
                    when 13 =>
                        r_ex_a <= '0' & s_ls(5 downto 0) & "00";
                    when 30 =>
                        v_sp := '0' & c_spd(19 downto 1);
                        c_ph1 <= f_addsub(c_ph1, c_spd, c_sneg);
                        c_ph2 <= f_addsub(c_ph2, c_spd, not c_sneg);
                        c_ph3 <= f_addsub(c_ph3, v_sp, c_sneg);
                    when others => null;
                end case;
            end if;
            if s_zone = '1' and s_zc = 16 then
                if s_vz = '1' then c_spd <= (others => '0');
                else c_spd <= resize(r_exf(15 downto 7), 20); end if;
                c_sneg <= s_vneg;
                s_scnt <= s_ls(8 downto 6);
            elsif s_zone = '1' and s_zc > 16 and s_zc < 26 and s_scnt /= 0 then
                c_spd <= shift_left(c_spd, 1);
                s_scnt <= s_scnt - 1;
            end if;

            -- [C] before the slots (zc 40 .. 42): the offsets' signs and magnitudes, the spread's half
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>                     -- |o| = g - 32768 or 32767 - g (one's complement)
                        z_oxn <= not s_a1(15);
                        if s_a1(15) = '1' then z_oxm <= s_a1(14 downto 0); else z_oxm <= not s_a1(14 downto 0); end if;
                        z_oyn <= not s_a2(15);
                        if s_a2(15) = '1' then z_oym <= s_a2(14 downto 0); else z_oym <= not s_a2(14 downto 0); end if;
                    when 41 =>                     -- ox + oy = gX + gY - 65536, oy - ox = gY - gX
                        v_16 := resize(s_a1, 17) + resize(s_a2, 17);
                        z_s5n <= not v_16(16);
                        if v_16(16) = '1' then z_s5m <= v_16(15 downto 0); else z_s5m <= not v_16(15 downto 0); end if;
                        v_16 := resize(s_a2, 17) - resize(s_a1, 17);
                        z_s6n <= v_16(16);
                        if v_16(16) = '1' then z_s6m <= not v_16(15 downto 0); else z_s6m <= v_16(15 downto 0); end if;
                    when 42 =>
                        if s_ap <= 32768 then z_tlow <= '1'; else z_tlow <= '0'; end if;
                    when others => null;
                end case;
            end if;

            -- [C] serial products, one slot every 24 zone clocks from zc 46: the operands are muxed at phase
            -- 0, loaded at phase 1, 16 iterations, the product is stable from phase 18 and stored at phase 20.
            if s_zone = '0' then
                z_run <= '0';
            elsif s_zc = 46 then
                z_run <= '1'; z_ph <= (others => '0'); z_slot <= (others => '0');
            elsif z_run = '1' then
                if z_ph = 23 then
                    z_ph <= (others => '0');
                    if z_slot = C_NSLOT then z_run <= '0'; else z_slot <= z_slot + 1; end if;
                else
                    z_ph <= z_ph + 1;
                end if;
            end if;
            if z_run = '1' and z_ph = 0 then z_pre <= '1'; else z_pre <= '0'; end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_slot) is
                    when 0  => z_fa <= resize(s_a3, 40);  z_fb <= to_unsigned(761, 16);              -- octaves above 14 v-px, Q8
                    when 1  =>                                                                       -- cells per h
                        z_fa <= resize(r_exf, 40);
                        case to_integer(c_sh) is
                            when 0      => z_fb <= to_unsigned(37448, 16);
                            when 1      => z_fb <= to_unsigned(18724, 16);
                            when 2      => z_fb <= to_unsigned(9362, 16);
                            when others => z_fb <= to_unsigned(4681, 16);
                        end case;
                    when 2  =>                                                                       -- pitch in AA px
                        z_fa <= resize(r_exf, 40);
                        z_fb <= shift_left(c_kpa, to_integer(z_oct(10 downto 8)));
                    when 3  =>                                                                       -- spread
                        if z_tlow = '1' then z_fa <= resize(s_ap, 40); z_fb <= s_ap;
                        else z_fa <= resize(s_ap - 32768, 40); z_fb <= to_unsigned(5472, 16); end if;
                    when 4  => z_fa <= resize(z_a1, 40);  z_fb <= to_unsigned(10923, 16);
                    when 6 | 8 => z_fa <= resize(z_dS, 40); z_fb <= resize(z_ang(6 downto 0), 16);   -- sine interpolation
                    when 9  => z_fa <= resize(c_inv, 40); z_fb <= c_C1;                              -- cos / P
                    when 10 => z_fa <= resize(c_inv, 40); z_fb <= c_S1;                              -- sin / P
                    when 11 => z_fa <= resize(c_inv, 40); z_fb <= to_unsigned(23170, 16);            -- (1/sqrt 2) / P
                    when 12 => z_fa <= resize(c_cu1, 40); z_fb <= resize(s_xs, 16);                  -- per pixel
                    when 13 => z_fa <= resize(c_su1, 40); z_fb <= resize(s_xs, 16);
                    when 14 => z_fa <= resize(c_cu2, 40); z_fb <= resize(s_xs, 16);
                    when 15 => z_fa <= resize(c_su1, 40); z_fb <= resize(s_ys, 16);                  -- per line
                    when 16 => z_fa <= resize(c_cu1, 40); z_fb <= resize(s_ys, 16);
                    when 17 => z_fa <= resize(c_cu2, 40); z_fb <= resize(s_ys, 16);
                    when 18 => z_fa <= resize(c_cu1, 40); z_fb <= resize(w_dxm & "0000", 16);        -- line 0, pre-pixel
                    when 19 => z_fa <= resize(c_su1, 40); z_fb <= to_unsigned(17280, 16);
                    when 20 => z_fa <= resize(z_oxm, 40); z_fb <= c_C1;                              -- offsets
                    when 21 => z_fa <= resize(z_oym, 40); z_fb <= c_S1;
                    when 22 => z_fa <= resize(c_su1, 40); z_fb <= resize(w_dxm & "0000", 16);
                    when 23 => z_fa <= resize(c_cu1, 40); z_fb <= to_unsigned(17280, 16);
                    when 24 => z_fa <= resize(z_oxm, 40); z_fb <= c_S1;
                    when 25 => z_fa <= resize(z_oym, 40); z_fb <= c_C1;
                    when 26 => z_fa <= resize(c_cu2, 40); z_fb <= resize(w_dxm & "0000", 16) + 17280;
                    when 27 => z_fa <= resize(c_cu2, 40); z_fb <= resize(w_dxm & "0000", 16) - 17280;
                    when 28 => z_fa <= resize(z_s5m, 40); z_fb <= to_unsigned(23170, 16);
                    when 29 => z_fa <= resize(z_s6m, 40); z_fb <= to_unsigned(23170, 16);
                    when 30 => z_fa <= resize(s_a4, 40);  z_fb <= to_unsigned(30147, 16);            -- dot radius
                    when 31 => z_fa <= resize(z_r0c, 40); z_fb <= c_pa;
                    when 32 => z_fa <= resize(c_pa, 40);  z_fb <= to_unsigned(21845, 16);            -- ring half width
                    when 33 => z_fa <= resize(c_r0, 40);  z_fb <= c_r0;                              -- r0^2
                    when 34 => z_fa <= resize(c_rcap, 40); z_fb <= c_rcap;                           -- cap^2
                    when 35 => z_fa <= resize(s_a6, 40);  z_fb <= to_unsigned(42181, 16);            -- camera gain
                    when 36 => z_fa <= resize(z_Ad, 40);  z_fb <= z_g;                               -- area per level
                    when 37 => z_fa <= resize(c_Lm, 40);  z_fb <= z_g;                               -- the slope radius
                    when 38 => z_fa <= resize(z_rg, 40);  z_fb <= z_kk;
                    when 39 => z_fa <= resize(c_h, 40);   z_fb <= c_h;                               -- h^2
                    when 40 => z_fa <= resize(z_Ad, 40);  z_fb <= c_Kf;                              -- / 2 rk
                    when 41 => z_fa <= resize(z_Ah, 40);  z_fb <= c_Kf;
                    when 42 => z_fa <= resize(z_Ga, 40);  z_fb <= c_Kf;
                    when 43 => z_fa <= resize(c_pa, 40);  z_fb <= c_pa;                              -- squares step
                    when 44 => z_fa <= resize(z_q0, 40);  z_fb <= c_Kf;
                    when others => null;                                                             -- 5, 7: table reads
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

            -- [C] the start values (slots 18 .. 29): two accumulators, one add each, controls set a clock early
            --   zA: u1 = -pa - pb + ph1 | v1 = pc - pd       | u2 = -pe + (ox + oy) / sqrt 2 + ph2
            --   zB: v3 = pa - pb + C1 ox - S1 oy | u3 = -pc - pd - S1 ox - C1 oy + ph3 | v2 = pf + (oy - ox) / sqrt 2
            za_en <= '0'; zb_en <= '0'; ld_go <= '0';
            if z_run = '1' and z_ph = 19 then
                za_clr <= '0'; zb_clr <= '0'; za_sel <= "000"; zb_sel <= "000";
                case to_integer(z_slot) is
                    when 18 => za_en <= '1'; za_clr <= '1'; za_sub <= '1'; zb_en <= '1'; zb_clr <= '1'; zb_sub <= '0';
                    when 19 => za_en <= '1'; za_sub <= '1'; zb_en <= '1'; zb_sub <= '1';
                    when 20 => zb_en <= '1'; zb_sub <= z_oxn; zb_sel <= "001";
                    when 21 => zb_en <= '1'; zb_sub <= not z_oyn; zb_sel <= "001";
                    when 22 => za_en <= '1'; za_clr <= '1'; za_sub <= '0'; zb_en <= '1'; zb_clr <= '1'; zb_sub <= '1';
                    when 23 => za_en <= '1'; za_sub <= '1'; zb_en <= '1'; zb_sub <= '1';
                    when 24 => zb_en <= '1'; zb_sub <= not z_oxn; zb_sel <= "001";
                    when 25 => zb_en <= '1'; zb_sub <= not z_oyn; zb_sel <= "001";
                    when 26 => za_en <= '1'; za_clr <= '1'; za_sub <= '1';
                    when 27 => zb_en <= '1'; zb_clr <= '1'; zb_sub <= '0';
                    when 28 => za_en <= '1'; za_sub <= z_s5n; za_sel <= "001";
                    when 29 => zb_en <= '1'; zb_sub <= z_s6n; zb_sel <= "001";
                    when others => null;
                end case;
            elsif z_run = '1' and z_ph = 21 then
                za_clr <= '0'; zb_clr <= '0';
                case to_integer(z_slot) is
                    when 21 => za_en <= '1'; za_sub <= '0'; za_sel <= "010";
                    when 25 => zb_en <= '1'; zb_sub <= '0'; zb_sel <= "100";
                    when 29 => za_en <= '1'; za_sub <= '0'; za_sel <= "011";
                    when others => null;
                end case;
            elsif z_run = '1' and z_ph = 23 then
                case to_integer(z_slot) is
                    when 21 => ld_go <= '1'; ld_a <= "00";
                    when 25 => ld_go <= '1'; ld_a <= "01";
                    when 29 => ld_go <= '1'; ld_a <= "10";
                    when others => null;
                end case;
            end if;
            if za_en = '1' then
                case to_integer(za_sel) is
                    when 0      => v_op := f_acc(27 downto 8);
                    when 1      => v_op := f_acc(30 downto 11);
                    when 2      => v_op := c_ph1;
                    when others => v_op := c_ph2;
                end case;
                if za_clr = '1' then v_bs := (others => '0'); else v_bs := zA; end if;
                zA <= f_addsub(v_bs, v_op, za_sub);
            end if;
            if zb_en = '1' then
                case to_integer(zb_sel) is
                    when 0      => v_op := f_acc(27 downto 8);
                    when 1      => v_op := f_acc(30 downto 11);
                    when others => v_op := c_ph3;
                end case;
                if zb_clr = '1' then v_bs := (others => '0'); else v_bs := zB; end if;
                zB <= f_addsub(v_bs, v_op, zb_sub);
            end if;

            if z_run = '1' then
                -- slot 1, phase 1: the pitch mantissa for slot 2
                if z_slot = 1 and z_ph = 1 then r_ex_a <= '0' & z_oct(7 downto 0); end if;
                -- slots 5 and 7: S(idx), S(idx + 1) of the quarter sine (S(256) = 32768 is not in the table)
                if z_slot = 5 or z_slot = 7 then
                    v_i9 := resize(z_ang(14 downto 7), 9) + 1;
                    case to_integer(z_ph) is
                        when 1 => r_ex_a <= '1' & z_ang(14 downto 7); z_i8 <= z_ang(15);
                        when 2 => r_ex_a <= '1' & v_i9(7 downto 0);   z_i8n <= z_ang(15) or v_i9(8);
                        when 4 => if z_i8 = '1' then z_s0 <= to_unsigned(32768, 16); else z_s0 <= r_exf; end if;
                        when 5 => if z_i8n = '1' then z_s1 <= to_unsigned(32768, 16); else z_s1 <= r_exf; end if;
                        when 6 => z_dS <= z_s1 - z_s0;
                        when others => null;
                    end case;
                end if;
                if z_ph = 20 then
                    case to_integer(z_slot) is
                        when 0  => z_oct <= f_acc(26 downto 16);
                        when 1  => c_inv <= f_acc(32 downto 11);
                        when 2  => c_pa <= f_acc(31 downto 16);
                        when 3  => z_a1 <= f_acc(30 downto 15);
                        when 4  =>
                            if z_tlow = '1' then z_ss <= f_acc(30 downto 15);
                            else z_ss <= to_unsigned(10923, 16) + z_a1; end if;
                        when 6  => c_S1 <= z_s0 + f_acc(22 downto 7);
                        when 8  => c_C1 <= z_s0 + f_acc(22 downto 7);
                        when 9  => c_cu1 <= f_acc(36 downto 15);
                        when 10 => c_su1 <= f_acc(36 downto 15);
                        when 11 => c_cu2 <= f_acc(36 downto 15);
                        when 12 => c_SA <= f_acc(23 downto 4);
                        when 13 => c_SB <= f_acc(23 downto 4);
                        when 14 => c_SC <= f_acc(23 downto 4);
                        when 15 => c_TA <= f_acc(30 downto 11);
                        when 16 => c_TB <= f_acc(30 downto 11);
                        when 17 => c_TC <= f_acc(30 downto 11);
                        when 30 => z_r0c <= to_unsigned(3932, 16) + f_acc(31 downto 16);
                        when 31 => c_r0 <= f_acc(31 downto 16);
                        when 32 =>
                            if s_fine = '1' then c_h <= f_acc(33 downto 18); else c_h <= (others => '0'); end if;
                        when 33 => z_A0 <= f_acc(27 downto 12);
                        when 34 => z_Ac <= f_acc(27 downto 12);
                        when 35 => z_g <= f_acc(31 downto 16);
                        when 36 => z_Ga <= f_acc(33 downto 16);
                        when 37 => z_kk <= f_acc(23 downto 8);
                        when 38 => z_rk <= c_r0 + f_acc(31 downto 16);
                        when 39 => z_H2 <= f_acc(29 downto 12);
                        when 40 => c_A0s <= f_acc(29 downto 14);
                        when 41 => c_Acs <= f_acc(29 downto 14);
                        when 42 => c_Gs <= f_acc(29 downto 14);
                        when 43 => z_q0 <= f_acc(29 downto 12);
                        when 44 => c_cq <= f_acc(31 downto 14);
                        when others => null;
                    end case;
                end if;
                if z_ph = 21 then
                    case to_integer(z_slot) is
                        when 0  =>                         -- 2^-oct = table(-oct mod 256) * 2^-sh
                            r_ex_a <= '0' & (not z_oct(7 downto 0) + 1);
                            if z_oct(7 downto 0) = 0 then c_sh <= z_oct(9 downto 8);
                            else c_sh <= z_oct(9 downto 8) + 1; end if;
                        when 2  =>                         -- the camera's cap 3/4 P; fade under 6 AA px
                            c_rcap <= c_pa - shift_right(c_pa, 2);
                            if c_pa >= 1536 then c_tf <= "00";
                            elsif c_pa >= 1152 then c_tf <= "01";
                            else c_tf <= "10"; end if;
                        when 4 | 6 =>                      -- the sine's angle 45 - s, then the cosine's 45 + s
                            if z_slot = 4 then z_ang <= f_addsub(to_unsigned(16384, 16), z_ss, '1');
                            else z_ang <= f_addsub(to_unsigned(16384, 16), z_ss, '0'); end if;
                        when 31 =>                         -- at least 1.25 v-px
                            if c_r0 < c_rmin then c_r0 <= c_rmin; end if;
                        when 32 =>                         -- at least 1 v-px
                            if s_fine = '1' and c_h < c_hmin then c_h <= c_hmin; end if;
                        when others => null;
                    end case;
                end if;
                if z_ph = 22 then
                    case to_integer(z_slot) is
                        when 33 =>                         -- Fine: h in 1/16 px
                            if s_fine = '1' then c_hb <= resize(c_h(15 downto 4), 17); else c_hb <= (others => '0'); end if;
                        when 32 =>                         -- the camera's head-room in radius
                            if c_rcap > c_r0 then z_rg <= c_rcap - c_r0; else z_rg <= (others => '0'); end if;
                        when 34 =>                         -- and in area
                            if z_Ac > z_A0 then z_Ad <= resize(z_Ac - z_A0, 17); else z_Ad <= (others => '0'); end if;
                        when 38 =>                         -- the slope radius's scale shift; no hole when r <= h
                            c_q <= f_q(z_rk);
                            if s_fine = '1' and z_rk <= c_h then c_nh <= '1'; else c_nh <= '0'; end if;
                        when 39 =>                         -- r^2 + h^2 and cap^2 + h^2
                            z_Ad <= resize(z_A0, 17) + resize(z_H2, 17);
                            z_Ah <= resize(z_Ac, 17) + resize(z_H2, 17);
                        when others => null;
                    end case;
                end if;
            end if;

            -- [C] serial divider (zc 984): Km = min(127, 2^(13 + q) / rk), restoring, 7 quotient bits; then
            -- Kf = Km 2^(8 - q), so that X Kf >> 14 = X / 2rk in 1/16 px
            if s_zone = '1' and s_zc = C_DV_ZC then
                dv_R <= shift_left(to_unsigned(1, 21), 13 + to_integer(c_q));
                dv_D <= resize(z_rk & "000000", 22);
                if shift_left(to_unsigned(1, 23), 13 + to_integer(c_q)) >= resize(z_rk & "0000000", 23) then dv_sat <= '1'; else dv_sat <= '0'; end if;
                dv_Q <= (others => '0'); dv_n <= to_unsigned(7, 4); dv_st <= '0';
            elsif dv_n /= 0 then
                if resize(dv_R, 22) >= dv_D then
                    dv_R <= resize(resize(dv_R, 22) - dv_D, 21); dv_Q <= dv_Q(5 downto 0) & '1';
                else
                    dv_Q <= dv_Q(5 downto 0) & '0';
                end if;
                dv_D <= '0' & dv_D(21 downto 1);
                dv_n <= dv_n - 1;
                if dv_n = 1 then dv_st <= '1'; end if;
            elsif dv_st = '1' then
                dv_st <= '0';
                if dv_sat = '1' then c_Km <= to_unsigned(127, 7); else c_Km <= dv_Q; end if;
            end if;
            if s_zone = '1' and s_zc = C_DV_ZC + 12 then
                c_Kf <= shift_left(resize(c_Km, 16), 8 - to_integer(c_q));
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
            s_st <= g_start & s_st(0 to 8);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] TABLE FILLS (vblank zone, after the slots)
    --   squares:  SQ(i) = 16 ((i + 1/2) P_a / 1024)^2 / 2rk by second differences, into all six copies
    --   camera:   T(L) = min(A0s + L Gs, Acs): the dot's (r^2 + h^2) / 2rk by camera level (area-linear)
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_t : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            sq_we <= '0';
            if s_zone = '1' and s_zc = C_SQ_ZC then
                sf_run <= '1'; sf_i <= (others => '0');
                sf_S <= resize(c_cq(17 downto 2), 36); sf_D <= resize(c_cq & '0', 30);
            elsif sf_run = '1' then
                sq_we <= '1'; sq_wa <= sf_i; sq_wd <= sf_S(35 downto 20);
                sf_S <= sf_S + sf_D; sf_D <= sf_D + resize(c_cq & '0', 30); sf_i <= sf_i + 1;
                if sf_i = 511 then sf_run <= '0'; end if;
            end if;

            rt_we <= '0';
            if s_zone = '1' and s_zc = C_SQ_ZC then
                rf_run <= '1'; rt_wa <= (others => '1');
                rf_acc <= resize(c_A0s & x"00", 24);
            elsif rf_run = '1' then
                v_t := rf_acc(23 downto 8);
                rt_we <= '1'; rt_wa <= rt_wa + 1;
                if v_t > c_Acs then rt_wd <= c_Acs; else rt_wd <= v_t; end if;
                rf_acc <= rf_acc + resize(c_Gs, 24);
                if rt_wa = 254 then rf_run <= '0'; end if;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [C] ENGINE, line DDAs and pixel offsets.  l_U / l_V hold this line's
    -- lattice coordinates at the pre-pixel: loaded with line 0 from the
    -- start-value accumulators in the zone, stepped at every line start but
    -- the field's first (so they hold through the line).  The three pixel
    -- offsets n (SA, SB, SC) restart at the pre-pixel; pixel 0 is at stage 9.
    -- Stage 10 adds them on the top 12 bits (Q20, modulo 1 cell):
    --   screen 1 (u +A, v -B), screen 2 (u +C, v -C), screen 3 = screen 1 swapped (u +B, v -A)
    ---------------------------------------------------------------------------
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            if s_zone = '1' then l_first <= '1'; elsif s_st(0) = '1' then l_first <= '0'; end if;
            if ld_go = '1' and ld_a = "00" then l_U(0) <= zA; elsif s_st(0) = '1' and l_first = '0' then l_U(0) <= l_U(0) + c_TA; end if;
            if ld_go = '1' and ld_a = "01" then l_V(0) <= zA; elsif s_st(0) = '1' and l_first = '0' then l_V(0) <= l_V(0) + c_TB; end if;
            if ld_go = '1' and ld_a = "10" then l_U(1) <= zA; elsif s_st(0) = '1' and l_first = '0' then l_U(1) <= l_U(1) + c_TC; end if;
            if ld_go = '1' and ld_a = "10" then l_V(1) <= zB; elsif s_st(0) = '1' and l_first = '0' then l_V(1) <= l_V(1) + c_TC; end if;
            if ld_go = '1' and ld_a = "01" then l_U(2) <= zB; elsif s_st(0) = '1' and l_first = '0' then l_U(2) <= l_U(2) + c_TB; end if;
            if ld_go = '1' and ld_a = "00" then l_V(2) <= zB; elsif s_st(0) = '1' and l_first = '0' then l_V(2) <= l_V(2) + c_TA; end if;

            if s_st(6) = '1' then
                o_A <= (others => '0'); o_B <= (others => '0'); o_C <= (others => '0');
            else
                o_A <= o_A + c_SA; o_B <= o_B + c_SB; o_C <= o_C + c_SC;
            end if;

            -- 10: the lattice coordinates (cells, 12 bits; the low 8 bits of the sum are dropped)
            s10_c(0) <= l_U(0)(19 downto 8) + o_A(19 downto 8);
            s10_c(1) <= l_V(0)(19 downto 8) - o_B(19 downto 8);
            s10_c(2) <= l_U(1)(19 downto 8) + o_C(19 downto 8);
            s10_c(3) <= l_V(1)(19 downto 8) - o_C(19 downto 8);
            s10_c(4) <= l_U(2)(19 downto 8) + o_B(19 downto 8);
            s10_c(5) <= l_V(2)(19 downto 8) - o_A(19 downto 8);
        end if;
    end process p_dda;

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
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> box over 64 virtual px
    -- (48 at 720p), edge-replicated and reloaded at every line start (the box's
    -- resync) -> vertical one-pole IIR in the line store (half horizontal
    -- resolution; the first line of a field loads it) -> camera level 0 .. 255.
    -- The pixel path taps the store's write data (the camera is never shown, so
    -- it need not be aligned to the pixel).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l   : unsigned(7 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_dd  : signed(12 downto 0);
        variable v_n   : signed(12 downto 0);
        variable v_m   : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart in the
            -- zone (Hardline dazzle's fix of riley's line-0 phase)
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

            -- 1: the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_avd <= data_in.avid & s_avd(1 to 4);
            if s_avd(1) = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- 2: a plain register (keeps the shell's timing)
            s_p <= s_sl;
            s_pcol <= s_col;

            -- 3: box: sum of the last N samples; the samples before the line start count as the first one
            if s_avd(2) = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= c_brp; end if;
            if (c_b64 = '1' and s_bcnt /= 64) or (c_b64 = '0' and s_bcnt /= 32) then v_tap := s_pfirst; else v_tap := bx_q; end if;
            if s_avd(2) = '1' and s_avd(3) = '0' then
                s_pfirst <= s_p;
                if c_b64 = '1' then s_bsum <= shift_left(resize(s_p, 14), 6);
                else s_bsum <= shift_left(resize(s_p, 14), 5); end if;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_avd(2) = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if (c_b64 = '1' and s_bcnt /= 64) or (c_b64 = '0' and s_bcnt /= 32) then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bcol <= s_pcol;

            -- 4: fabric copies (the box mean x 16); 5: vertical IIR on the odd columns, q += (box - q) >> k
            -- (the first line: q = box)
            if c_b64 = '1' then s_eb <= s_bsum(13 downto 2); else s_eb <= s_bsum(12 downto 1); end if;
            s_eq <= lb_q1;
            s_ev <= s_avd(3) and s_bcol(0); s_ecol <= s_bcol;
            if s_zone = '1' then s_fl <= '1';
            elsif s_avd(5) = '1' and s_avd(4) = '0' then s_fl <= '0'; end if;
            v_d := signed(resize(s_eb, 13)) - signed(resize(s_eq, 13));
            case to_integer(c_iir) is
                when 3      => v_dd := shift_right(v_d, 3);
                when 4      => v_dd := shift_right(v_d, 4);
                when others => v_dd := shift_right(v_d, 5);
            end case;
            v_n := signed(resize(s_eq, 13)) + v_dd;
            if s_fl = '1' then lb_wd <= s_eb; else lb_wd <= unsigned(v_n(11 downto 0)); end if;
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the pixel path's tap: the last two columns written; stage 8 interpolates between them on the
            -- clock after a write; stage 9 addresses the radius table
            if lb_we = '1' then
                h_cur <= lb_wd; h_prev <= h_cur; h_ph <= '1';
            else
                h_ph <= '0';
            end if;
            v_m := resize(h_cur, 13) + resize(h_prev, 13);
            if h_ph = '1' then s8_q <= v_m(12 downto 1); else s8_q <= h_cur; end if;
            rt_a <= s8_q(11 downto 4);
            -- the camera's running mean level (about one field), for the slope radius
            if s_gav_sr(7) = '1' then lm_acc <= lm_acc + rt_a - lm_acc(27 downto 20); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] SCREENS (stages 11 .. 23); everything in 1/16 px of edge distance
    --   11  squares RAM (address: the folded stage-10 coordinate)    12 fabric
    --   13  d^2 = SQ(u) + SQ(v)        (the camera table: 9 address, 10 RAM, 11 .. 13 fabric)
    --   14  e = T(L) - d^2             15  Bold: x = e     Fine: x = h - |e|  (no hole: x large)
    --   16  coverage clamp(8 + x)      17  S = sum over the screens        18 .. 20  tone ROM
    --   21 .. 23  fade toward mid-grey (per-field weight, shift-adds)
    ---------------------------------------------------------------------------
    p_scr : process(clk)
        variable v_y  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 11 (RAM), 12
            sq_f <= sq_q;
            rt_f <= rt_q; rt_g <= rt_f;
            -- 13
            for k in 0 to 2 loop
                s13_d2(k) <= sq_f(2 * k) + sq_f(2 * k + 1);
            end loop;
            s13_A <= rt_g;
            -- 14
            for k in 0 to 2 loop
                s14_e(k) <= signed(resize(s13_A, 17)) - signed(resize(s13_d2(k), 17));
            end loop;
            -- 15: one add / subtract per screen: Bold x = 0 + e, Fine x = h -+ e (- on the inside of the ring)
            for k in 0 to 2 loop
                if s_fine = '1' and s14_e(k)(16) = '0' and c_nh = '1' then
                    s15_x(k) <= to_signed(4096, 17);
                else
                    s15_x(k) <= signed(f_addsub(c_hb, unsigned(s14_e(k)), s_fine and not s14_e(k)(16)));
                end if;
            end loop;
            -- 16: coverage clamp(8 + x, 0, 16): x in -8 .. 7 when x(16 .. 3) are all equal
            for k in 0 to 2 loop
                if s15_x(k)(16) = '1' and s15_x(k)(15 downto 3) /= "1111111111111" then s16_c(k) <= (others => '0');
                elsif s15_x(k)(16) = '0' and s15_x(k)(15 downto 3) /= "0000000000000" then s16_c(k) <= to_unsigned(16, 5);
                else s16_c(k) <= unsigned(s15_x(k)(4 downto 0)) + 8; end if;
            end loop;
            -- 17: S over the screens (Two: the outer two); 18: the tone ROM address
            if s_two = '1' then s17_S <= resize(s16_c(0), 6) + resize(s16_c(2), 6);
            else s17_S <= resize(s16_c(0), 6) + resize(s16_c(1), 6) + resize(s16_c(2), 6); end if;
            tn_a <= s_ink & s_two & s17_S;
            -- 19 (ROM), 20 fabric, 21: o = luma - 64
            tn_f <= tn_q;
            s21_o <= tn_f - 64;
            -- 22, 23: the fade toward mid-grey: none, half, flat (tf = 0 / 4 / 8 eighths)
            v_y := resize(s21_o, 11) + 438;
            case to_integer(c_tf) is
                when 0      => s22_o <= s21_o;
                when 1      => s22_o <= v_y(10 downto 1);
                when others => s22_o <= to_unsigned(438, 10);
            end case;
            s23_y <= to_unsigned(64, 10) + s22_o;
        end if;
    end process p_scr;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 24 .. 29): luma [1 2 1]/4, luma 13/16
    -- toward black (white 776), blanking gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 24 .. 26: taps y(n + 1), y(n), y(n - 1)
            s_yd(1) <= s23_y;
            for i in 2 to 3 loop s_yd(i) <= s_yd(i - 1); end loop;
            s26_ya <= resize(s_yd(1), 12) + resize(s_yd(3), 12) + 2;
            s26_yb <= s_yd(2);
            -- 27
            v_y := s26_ya + shift_left(resize(s26_yb, 12), 1);
            s27_y <= v_y(11 downto 2);
            -- 28: luma 13/16 toward black
            if s27_y > 64 then v_dl := s27_y - 64; else v_dl := (others => '0'); end if;
            s28_y <= s27_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- 29: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s29_y <= s28_y;
            else
                s29_y <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s29_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture rosette;
