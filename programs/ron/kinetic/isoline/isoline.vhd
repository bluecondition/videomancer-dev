-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Isoline v0.1.0 -- Kinetic.  Metaballs drawn as a topographic map: hypsometric tints and contour lines,
-- islands merging through saddles.  A lava lamp as cartography.
--
-- Element: a blob (four of them).  State: its position along the path, path(phase + Spread * j), and its
-- radius, which pulses with the wave.
--
--   F(x, y)  = sum_j w(q_j) + source,    q_j = ((x - cx_j)^2 + (y - cy_j)^2) / R_j^2,    w(q) = (1 - q)^2, q < 1
--   PHI      = bands * F                 contour c at PHI = c; tint = floor(PHI)
--   coverage from the distance to the nearest contour, |PHI - c| / |grad PHI| (finite differences: the
--   previous pixel and the line above), so every edge and line is anti-aliased per real pixel
--
--   K1  Scale    path size (3:2 ellipse / figure of eight, 0 .. 767 x 511 virtual px), glided
--   K2  Shape    contour interval: 2 .. 32 bands per blob height (few broad tints .. many fine bands)
--   K3  Colour   the 16 shared Kinetic palettes (hysteresis)  [K6 camera pass: was K6; the old K3 Depth,
--                the radial pulsing, is fixed at its default 600 (c_d10 = 600^2 >> 10 = 351)]
--   K4  Rate     travel along the path, bipolar, centre = still
--   K5  Spread   phase step per blob (0 = one stacked hill; square law, 0 .. 1/2 cycle)
--   K6  Camera   how strongly the source field (camera luma or Osc) raises the terrain: linear gain,
--                0 = none (the black-input picture), 20 % = the v0.1.0 strength, 100 % = about 4.8 times
--                (2026-11-15 +25 %: the cored camera sample x 1.25).  Glided in the shared glide (channel 2, +-4
--                deadband), applied to the 8-bit source sample as K6 / 32 (32 steps) by one 8 x 5 multiply before
--                the IIRs (the Osc unchanged: 1.6 x the camera's gain at every position, up to 31 / 8 of v0.1.0's)
--   S7  Wave     Smooth / Snap (the blobs hop between 8 stations with an eased move)
--   S8  Field    Orbit (the ellipse) / Drift (a figure of eight: the blobs cross and merge)
--   S9  Look     Tints (filled bands stepping ground -> A -> B -> accent -> B -> A ..) / Lines (contours alone)
--   S10 Index    Off / On: every fourth contour, from the coastline, heavier and in the accent colour
--   S11 Source   Video / Osc: the luma field adds to F (bright areas rise as terrain)
--   P12 MERGE    blob radius 128 .. 664 virtual px: separate islands .. one continent with saddle contours
--
-- q_j runs as a second-difference DDA along the line (per blob: q += d, d += dd) whose start value is itself
-- a second-difference DDA down the field (nq += e, e += ee); the scale 1 / R_j^2 is folded into the
-- increments, so there is no per-pixel multiply in the distance.  w is one 256-entry square ROM plus an
-- 8 x 6 multiplier per blob; PHI = F * bands is one 16 x 9 multiplier.
--
-- Chassis from riley.vhd (same tags):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         glides, switches, rate deadband, palette hysteresis
--   [C] ENGINE                           per-field sequencer (one serial multiplier: 5 global slots, then 13
--                                        slots per blob), line DDAs, beam DDAs, falloff, PHI, gradient
--   [D] SOURCE FIELD                     Osc lattice, horizontal IIR, vertical IIR line store (16-bit, half res)
--   [E] TINTS + AA BLEND                 coverage divider, band -> palette colour, 9-level shift-add blend
--   [F] OUTPUT CONDITIONING              chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 at the same group delay,
--                                        luma 13/16, blanking gate.  (K6 camera pass: the chroma filter's
--                                        8-sample delay moved from 160 flip-flops to a block-RAM ring, the
--                                        fit headroom for the camera gain stage)
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixel n sits at x = x0 + n xs half virtual px, pixels 0 .. W + 7 are
-- rendered, output column j is the 9-tap filter over pixels j .. j + 8.  s_gav_sr(S - 2) flags pixels
-- 0 .. W - 1 at stage S.  One latency (36) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K3 = Colour (was K6); K3 Depth fixed at 600 (C_KFIX_3).
-- 2026-11-15: K6 camera top +25 % (Ron)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.isoline_pkg.all;
use work.all;

architecture isoline of program_top is
    constant C_KFIX_3 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(600, 10));   -- K6 camera pass: fixed knob
    constant C_LAT : integer := 36;   -- = processing_delay_clks
    constant C_NBL  : integer := 4;    -- blobs

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 520;
    constant C_DEF_K2  : integer := 338;
    constant C_DEF_K6  : integer := 512;   -- K6 Camera (glide channel 2, formerly K3 Depth)
    constant C_DEF_K5  : integer := 647;
    constant C_DEF_P12 : integer := 540;

    constant C_PUL : unsigned(15 downto 0) := to_unsigned(25033, 16);   -- pulse phase offset per blob

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
    -- ys^2 by timing ID
    function f_ys2(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1295044, 21);
            when 4          => return to_unsigned(331776, 21);
            when 8          => return to_unsigned(921600, 21);
            when 12         => return to_unsigned(230400, 21);
            when 5 | 6 | 14 => return to_unsigned(147456, 21);
            when 1 | 2 | 10 => return to_unsigned(262144, 21);
            when others     => return to_unsigned(65536, 21);
        end case;
    end function;
    -- quarter-wave ROM address of a 16-bit phase
    function f_qa(p : unsigned(15 downto 0)) return unsigned is
    begin
        if p(14) = '1' then return not p(13 downto 4); else return p(13 downto 4); end if;
    end function;

    -- band -> palette colour: 0 the ground, then A, B, accent, B, A, B, accent, ..
    function f_tint(b : unsigned(5 downto 0)) return unsigned is
    begin
        if b = 0 then return to_unsigned(0, 2);
        elsif b(1 downto 0) = "00" then return to_unsigned(2, 2);
        else return b(1 downto 0); end if;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ys2  : unsigned(20 downto 0) := to_unsigned(65536, 21);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    -- by video mode: the pre-pixel's distance left of the centre (h), 2 xs^2, xs^2, the index line weight
    signal w_dxm  : unsigned(10 downto 0);
    signal w_dd   : unsigned(5 downto 0);
    signal w_x2   : unsigned(4 downto 0);
    signal w_hwi  : unsigned(5 downto 0);

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

    -- [B] knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal s_gcam : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);   -- K6 Camera, glided (Q3)
    signal s_cm   : unsigned(4 downto 0) := to_unsigned(16, 5);              -- source gain K6 / 32 (x / 16)
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(C_DEF_K5 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_drift, s_lines, s_index, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(6, 4);

    -- [C] ROMs: C_IS_TAB (2^(i/256), then the palettes), C_IS_SIN (a quarter wave), C_IS_SQ (i^2, one per blob)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_sh   : signed(3 downto 0) := (others => '0');
    signal z_sh4  : unsigned(2 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_k31  : unsigned(15 downto 0) := to_unsigned(8192, 16);  -- 2^31 / R^2 (R in h)
    signal c_nb   : unsigned(8 downto 0) := to_unsigned(80, 9);      -- bands per unit height, Q4
    constant c_d10 : unsigned(9 downto 0) := to_unsigned(351, 10);   -- pulse depth: Depth fixed at 600 (600^2 >> 10)
    signal c_sp16 : unsigned(15 downto 0) := (others => '0');        -- spread, cycles Q16 per blob
    signal c_rstep : unsigned(27 downto 0) := (others => '0');       -- phase step per field, Q32
    signal c_ph   : unsigned(31 downto 0) := (others => '0');        -- wave phase, Q32 cycles
    signal w_ax   : unsigned(10 downto 0);                           -- path half-axes (h)
    signal w_ay   : unsigned(9 downto 0);

    -- [C] blob sequencer: z_j blob, z_s slot (0 = the lookups, 1 .. 13 = products), z_p phase 0 .. 23
    signal z_run  : std_logic := '0';
    signal z_j, z_wj : unsigned(2 downto 0) := (others => '0');
    signal z_s    : unsigned(3 downto 0) := (others => '0');
    signal z_p    : unsigned(4 downto 0) := (others => '0');
    signal z_th, z_thp, z_pp : unsigned(15 downto 0) := (others => '0');
    signal z_mv, z_nx, z_ny, z_np, z_smd : std_logic := '0';
    signal z_cx, z_cy, z_xs0 : signed(12 downto 0) := (others => '0');
    signal z_aS, z_Sy : unsigned(11 downto 0) := (others => '0');
    signal z_SX   : signed(14 downto 0) := (others => '0');
    signal z_m2   : signed(16 downto 0) := (others => '0');
    signal z_aMd  : unsigned(14 downto 0) := (others => '0');
    signal z_aMe  : unsigned(19 downto 0) := (others => '0');
    signal z_pd   : unsigned(9 downto 0) := (others => '0');
    signal z_t, z_kj : unsigned(15 downto 0) := (others => '0');
    signal z_s2   : unsigned(23 downto 0) := (others => '0');
    signal z_bus  : unsigned(34 downto 0) := (others => '0');
    signal z_wr : std_logic := '0';
    signal z_wf   : unsigned(2 downto 0) := "000";
    signal z_kick : std_logic_vector(0 to 3) := (others => '0');

    -- [C] per-blob DDA state.  Down the field it lives in a register file (word = blob & field: 0 Q the next
    -- line's start value, 1 E its step, 2 EE the step's step, 3 D0 the pixel step at the pre-pixel, 4 DD its
    -- step), walked once per line; along the line it is the beam (b_q, b_d) and c_dd.
    type t_rf is array (0 to 255) of unsigned(47 downto 0);
    signal rf_ram : t_rf := (others => (others => '0'));
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q, rf_f, rw_d : unsigned(47 downto 0) := (others => '0');
    signal rf_ra, rw_a : unsigned(7 downto 0) := (others => '0');
    signal rw_we : std_logic := '0';
    signal rf_op, rf_sum : unsigned(34 downto 0) := (others => '0');
    signal l_run, l_kick, b_run : std_logic := '0';
    signal l_j : unsigned(1 downto 0) := "00";
    signal l_p : unsigned(2 downto 0) := "000";
    type t_q35 is array (0 to C_NBL - 1) of unsigned(34 downto 0);
    signal b_q, b_d, c_dd : t_q35 := (others => (others => '0'));
    signal e_first, l_first : std_logic := '0';

    -- [C] falloff and field (stages 3 .. 10)
    type t_u16 is array (0 to C_NBL - 1) of unsigned(15 downto 0);
    type t_u9n is array (0 to C_NBL - 1) of unsigned(8 downto 0);
    signal s3_m, s4_m : t_u9n := (others => (others => '0'));
    signal sq_q, s4_sq, s5_w : t_u16 := (others => (others => '0'));
    signal s3_in, s4_in : std_logic_vector(0 to C_NBL - 1) := (others => '0');
    signal s6_a, s6_b : unsigned(16 downto 0) := (others => '0');
    signal s6_c, s7_c : unsigned(17 downto 0) := (others => '0');
    signal s7_ab : unsigned(17 downto 0) := (others => '0');
    signal s8_f : unsigned(15 downto 0) := (others => '0');
    signal s9_p1 : unsigned(19 downto 0) := (others => '0');
    signal s9_p2 : unsigned(20 downto 0) := (others => '0');
    signal s10_phi, s11_phi, s12_phi, s13_phi, s14_phi : unsigned(17 downto 0) := (others => '0');
    signal s11_gx, s11_gy : signed(15 downto 0) := (others => '0');
    signal s12_agx, s12_agy, s13_mx, s13_mn, s14_w : unsigned(12 downto 0) := (others => '0');

    -- [C] PHI line store (the line above at every fourth pixel, for the vertical gradient)
    type t_ph is array (0 to 511) of unsigned(15 downto 0);
    signal ph_ram : t_ph := (others => (others => '0'));
    attribute no_rw_check of ph_ram : signal is true;
    signal ph_ra, s_c10 : unsigned(11 downto 0) := (others => '0');
    signal ph_wa : unsigned(8 downto 0) := (others => '0');
    signal ph_q, s10_ab, ph_wd : unsigned(15 downto 0) := (others => '0');
    signal ph_we : std_logic := '0';

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl : unsigned(7 downto 0) := (others => '0');
    signal s_slg : unsigned(9 downto 0) := (others => '0');   -- s_sl * K6 gain (Q8.2)
    signal s_slp : unsigned(12 downto 0) := (others => '0');  -- s_sl * s_cm
    signal s_sv0, s_sv1, s_sv, s_svd : std_logic := '0';
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_h : unsigned(14 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb : unsigned(14 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd, s_eq, s_eqd, s_srcm : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';

    -- [E] palette (8 bits a channel: ground, A, B, accent)
    type t_pal4 is array (0 to 3) of unsigned(7 downto 0);
    signal c_py : t_pal4 := (others => to_unsigned(16, 8));
    signal c_pu, c_pv : t_pal4 := (others => to_unsigned(128, 8));
    signal t_go : std_logic := '0';

    -- [E] coverage and blend
    signal s15_weff : unsigned(12 downto 0) := (others => '0');
    signal s15_sda : unsigned(11 downto 0) := (others => '0');
    signal s15_c : unsigned(5 downto 0) := (others => '0');
    signal s15_above : std_logic := '0';
    type t_dv13 is array (0 to 4) of unsigned(12 downto 0);
    signal dv_r, dv_w : t_dv13 := (others => (others => '0'));
    type t_dv5 is array (0 to 4) of unsigned(4 downto 0);
    signal dv_q : t_dv5 := (others => (others => '0'));
    type t_dv6 is array (0 to 4) of unsigned(5 downto 0);
    signal dv_c : t_dv6 := (others => (others => '0'));
    signal dv_sat, dv_above : std_logic_vector(0 to 4) := (others => '0');
    signal s21_cov, s22_cov, s23_cov : unsigned(3 downto 0) := (others => '0');
    signal s21_pi, s21_qi : unsigned(1 downto 0) := "00";
    signal s22_p, s22_q, s23_p, s24_p : unsigned(23 downto 0) := x"808010";
    signal s23_dy, s23_du, s23_dv : signed(8 downto 0) := (others => '0');
    signal s24_ay, s24_au, s24_av, s24_by, s24_bu, s24_bv : signed(12 downto 0) := (others => '0');
    signal s25_y, s25_u, s25_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    -- K6 camera pass: the chroma filter's 8-sample delay in a block-RAM ring instead of 160 flip-flops (the
    -- same samples; the 8-sums' per-line resync still reloads them, so RAM start-up contents cannot stick)
    type t_cr is array (0 to 255) of unsigned(31 downto 0);
    signal cr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_q, cr_f : unsigned(31 downto 0) := (others => '0');
    signal cr_wp : unsigned(7 downto 0) := to_unsigned(6, 8);
    signal cr_rp : unsigned(7 downto 0) := (others => '0');
    signal s_yd : t_u10(1 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv2, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4100, 13);
    signal s_hist : std_logic_vector(7 downto 0) := (others => '0');
    signal s26_ya : unsigned(11 downto 0) := (others => '0');
    signal s26_yb : unsigned(9 downto 0) := (others => '0');
    signal s27_y, s28_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s27_u, s27_v, s28_u, s28_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s29_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s29_u, s29_v : unsigned(9 downto 0) := to_unsigned(512, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_IS_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_IS_SIN(to_integer(sn_a));
        end if;
    end process p_rom;

    -- the square ROM, (255 - i)^2, one copy per blob, addressed straight from the beam (stage 3)
    g_sq : for j in 0 to C_NBL - 1 generate
        p_sq : process(clk)
        begin
            if rising_edge(clk) then
                sq_q(j) <= C_IS_SQ(to_integer(b_q(j)(25 downto 18)));
            end if;
        end process p_sq;
    end generate g_sq;

    -- the DDA register file: one read port, one write port
    p_rfram : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf_ram(to_integer(rf_ra));
            rf_f <= rf_q;
            if rw_we = '1' then rf_ram(to_integer(rw_a)) <= rw_d; end if;
        end if;
    end process p_rfram;

    -- chroma delay ring: s25_u / s25_v read back 8 clocks later (6 addresses behind the write, the RAM's
    -- register and a fabric register)
    p_crram : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= cr_ram(to_integer(cr_rp));
            cr_f <= cr_q;
            cr_ram(to_integer(cr_wp)) <= x"000" & s25_v & s25_u;
            cr_wp <= cr_wp + 1; cr_rp <= cr_rp + 1;
        end if;
    end process p_crram;

    -- PHI line store: read ahead of the write, never the same address
    p_phram : process(clk)
    begin
        if rising_edge(clk) then
            ph_q <= ph_ram(to_integer(ph_ra(10 downto 2)));
            if ph_we = '1' then ph_ram(to_integer(ph_wa)) <= ph_wd; end if;
        end if;
    end process p_phram;

    -- luma line store (half horizontal resolution): one read port (the IIR's own, 2 addresses ahead of its
    -- write); the pixel path taps the same read
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- [A] constants by video mode; the path's half-axes
    ---------------------------------------------------------------------------
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);
    w_dd  <= to_unsigned(32, 6) when s_xs = 4 else to_unsigned(18, 6) when s_xs = 3 else to_unsigned(8, 6);
    w_x2  <= to_unsigned(16, 5) when s_xs = 4 else to_unsigned(9, 5) when s_xs = 3 else to_unsigned(4, 5);
    w_hwi <= to_unsigned(16, 6) when s_xs = 4 else to_unsigned(18, 6) when s_xs = 3 else to_unsigned(24, 6);
    w_ay  <= s_g1(12 downto 3);
    w_ax  <= resize(s_g1(12 downto 3), 11) + resize(s_g1(12 downto 4), 11);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_c  : signed(10 downto 0);
        variable v_r  : unsigned(27 downto 0);
        variable v_pi : unsigned(3 downto 0);
        variable v_f, v_g : unsigned(12 downto 0);
        variable v_ph : unsigned(15 downto 0);
        variable v_12 : unsigned(11 downto 0);
        variable v_b  : unsigned(34 downto 0);
        variable v_add : unsigned(2 downto 0);
        variable v_adj : unsigned(15 downto 0);
        variable v_neg : std_logic;
        variable v_s13 : signed(12 downto 0);
        variable v_s17 : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_ys2 <= f_ys2(s_tid);
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

            -- [B] rate: bipolar with a +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;     -- |v| - 8
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_snap  <= registers_in(6)(0);
                s_drift <= registers_in(6)(1);
                s_lines <= registers_in(6)(2);
                s_index <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- palette: 16 steps with hysteresis (leave a step only 8 past its edge)
                v_k := unsigned(registers_in(2));
                v_c := signed(resize(v_k, 11)) - signed(resize(s_pal & "100000", 11));
                if v_c > 40 or v_c < -40 then s_pal <= v_k(9 downto 6); end if;
            end if;
            g_wm1 <= s_w - 1;
            -- [B] K6 camera gain: K6 / 32 (0 .. 31; 8 = the v0.1.0 strength, 16 = the default) on the 8-bit source
            -- sample (32 steps; the Osc takes twice the camera's, as before).  s_gcam only moves in the vblank
            -- zone, so this is quasi-static.
            s_cm <= s_gcam(12 downto 8);
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_gcam; s_gr <= unsigned(registers_in(5));   -- K6 Camera
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        -- deadband +-2 knob steps, +-4 on the camera (channel 2) so pot noise cannot shimmer
                        if (s_gd > 32 or s_gd < -32) or (z_gc /= 2 and (s_gd > 16 or s_gd < -16)) then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_gcam <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] serial products.  Global slot k loads at zc = 32 + 24 k (decoded at 30 + 24 k, operands
            -- pre-muxed at 31 + 24 k), 16 iterations, stored at 50 + 24 k.  Then the blob sequencer: per blob
            -- 14 slots of 24 clocks; slot 0 looks the sines up, slots 1 .. 13 each pre-mux at phase 3 / 4,
            -- load at 5 and store at 22; the phases 23, 0, 1, 2 derive the next operands.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    -- (slot 1, Depth^2, is gone: Depth is fixed, c_d10 is a constant)
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when others => null;
                end case;
            end if;
            if z_run = '1' and z_p = 3 and z_s /= 0 then
                z_sel <= '1' & z_s; z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_gp, 40);  z_fb <= to_unsigned(1217, 16);                 -- radius octaves
                    when 2  => z_fa <= resize(s_g5(12 downto 3), 40); z_fb <= resize(s_g5(12 downto 3), 16);  -- spread^2
                    when 3  => z_fa <= resize(s_vm, 40);  z_fb <= to_unsigned(897, 16);                  -- rate exponent
                    when 4  => z_fa <= resize(r_exf, 40); z_fb <= to_unsigned(9321, 16);                 -- rate step
                    when 17 => z_fa <= resize(sn_q(13 downto 0), 40); z_fb <= resize(w_ax, 16);          -- cx
                    when 18 => z_fa <= resize(sn_q(13 downto 0), 40); z_fb <= resize(w_ay, 16);          -- cy
                    when 19 => z_fa <= resize(sn_q(13 downto 0), 40); z_fb <= resize(c_d10, 16);         -- pulse
                    when 20 => z_fa <= resize(c_k31, 40); z_fb <= resize(z_pd, 16);                      -- k * pulse
                    when 21 => z_fa <= resize(z_aS, 40);  z_fb <= resize(z_aS, 16);                      -- S^2
                    when 22 => z_fa <= resize(z_Sy, 40);  z_fb <= resize(z_Sy, 16);                      -- Sy^2
                    when 23 => z_fa <= resize(z_s2, 40);  z_fb <= z_kj;                                  -- Q = k (S^2 + Sy^2)
                    when 24 => z_fa <= resize(z_aMe, 40); z_fb <= resize(s_ys, 16);                      -- ys (256 Sy - ys)
                    when 25 => z_fa <= resize(f_acc(30 downto 7), 40); z_fb <= z_kj;                     -- E, the line step
                    when 26 => z_fa <= resize(s_ys2, 40); z_fb <= z_kj;                                  -- EE, its step
                    when 27 => z_fa <= resize(z_aMd, 40); z_fb <= z_kj;                                  -- D0, the pixel step
                    when others => z_fa <= resize(w_dd, 40); z_fb <= z_kj;                               -- DD, its step
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

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20 =>                        -- motion (last field's step)
                        c_ph <= f_addsub(c_ph, resize(c_rstep, 32), s_vneg);
                    -- radius: k = 2^(15 - oct), oct 0 .. 4.75
                    when 50  => z_oct <= f_acc(23 downto 13);
                    when 51  =>
                        v_e := -signed(resize(z_oct, 12));
                        r_ex_a <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(11 downto 8);
                    when 54  =>
                        case to_integer(z_sh) is
                            when 0      => c_k31 <= r_exf;
                            when -1     => c_k31 <= shift_right(r_exf, 1);
                            when -2     => c_k31 <= shift_right(r_exf, 2);
                            when -3     => c_k31 <= shift_right(r_exf, 3);
                            when -4     => c_k31 <= shift_right(r_exf, 4);
                            when others => c_k31 <= shift_right(r_exf, 5);
                        end case;
                    -- contour interval: bands = 2 * 2^(k2 / 256)
                    when 60  => r_ex_a <= '0' & s_g2(10 downto 3);
                    when 63  =>
                        case s_g2(12 downto 11) is
                            when "00"   => c_nb <= resize(shift_right(r_exf, 10), 9);
                            when "01"   => c_nb <= resize(shift_right(r_exf, 9), 9);
                            when "10"   => c_nb <= resize(shift_right(r_exf, 8), 9);
                            when others => c_nb <= resize(shift_right(r_exf, 7), 9);
                        end case;
                    when 98  => c_sp16 <= f_acc(20 downto 5);
                    when 122 => r_ex_a <= '0' & f_acc(15 downto 8); z_sh4 <= f_acc(18 downto 16);
                    when 146 =>
                        v_r := resize(f_acc(29 downto 8), 28);
                        if s_vm = 0 then c_rstep <= (others => '0');
                        else c_rstep <= shift_left(v_r, to_integer(z_sh4)); end if;
                    when others => null;
                end case;
            end if;

            -- [C] the blob sequencer
            z_wr <= '0';
            t_go <= '0';
            z_wj <= z_j;
            z_kick <= t_go & z_kick(0 to 2);
            if s_zone = '1' and s_zc = 160 then
                z_run <= '1'; z_j <= (others => '0'); z_s <= (others => '0'); z_p <= (others => '0');
                z_th <= c_ph(31 downto 16); z_pp <= (others => '0');
            elsif z_run = '1' then
                if z_p = 23 then
                    z_p <= (others => '0');
                    if z_s = 12 then
                        z_s <= (others => '0'); z_j <= z_j + 1;
                        z_th <= z_th + c_sp16; z_pp <= z_pp + C_PUL;
                        if z_j = C_NBL - 1 then z_run <= '0'; t_go <= '1'; end if;
                    else
                        z_s <= z_s + 1;
                    end if;
                else
                    z_p <= z_p + 1;
                end if;

                -- slot 0: Snap's eased hop.  Slots 1 .. 3 look their sine up at phase 0 (read at phase 4)
                if z_s = 0 then
                    case to_integer(z_p) is
                        when 0 =>
                            v_f := z_th(12 downto 0); v_g := v_f - 6144;
                            if v_f >= 6144 then z_mv <= '1'; else z_mv <= '0'; end if;
                            sn_a <= v_g(10 downto 1);
                        when 2 =>
                            if s_snap = '0' then z_thp <= z_th;
                            elsif z_mv = '1' then z_thp <= z_th(15 downto 13) & sn_q(13 downto 1);
                            else z_thp <= z_th(15 downto 13) & "0000000000000"; end if;
                        when others => null;
                    end case;
                end if;
                if z_p = 0 then
                    v_ph := z_thp + 16384;
                    case to_integer(z_s) is
                        when 1 => sn_a <= f_qa(v_ph); z_nx <= v_ph(15);
                        when 2 =>
                            if s_drift = '1' then v_ph := z_thp(14 downto 0) & '0'; else v_ph := z_thp; end if;
                            sn_a <= f_qa(v_ph); z_ny <= v_ph(15);
                        when 3 =>
                            v_ph := (z_th(13 downto 0) & "00") + z_pp;
                            sn_a <= f_qa(v_ph); z_np <= v_ph(15);
                        when others => null;
                    end case;
                end if;

                -- products stored at phase 22.  The DDA words go through one bus: shifted to 26 fraction
                -- bits, negated or rounded up (Q + 4, EE + 1, DD + 1 keep q from undershooting zero)
                if z_p = 22 then
                    v_b := f_acc(39 downto 5); v_neg := '0'; v_add := "000";
                    case to_integer(z_s) is
                        when 1 =>
                            v_12 := f_acc(25 downto 14);
                            if z_nx = '1' then z_cx <= -signed('0' & v_12); else z_cx <= signed('0' & v_12); end if;
                        when 2 =>
                            v_12 := f_acc(25 downto 14);
                            if z_ny = '1' then z_cy <= -signed('0' & v_12); else z_cy <= signed('0' & v_12); end if;
                        when 3 => z_pd <= f_acc(23 downto 14);
                        when 4 => z_t <= f_acc(25 downto 10);
                        when 5 => z_s2 <= f_acc(23 downto 0);
                        when 6 => z_s2 <= z_s2 + f_acc(23 downto 0);
                        when 7 => v_add := "100";
                        when 9 => v_b := resize(f_acc(39 downto 12), 35); v_neg := '1';
                        when 10 => v_b := resize(f_acc(39 downto 18), 35); v_add := "001";
                        when 11 => v_neg := z_smd;
                        when 12 => v_add := "001";
                        when others => null;
                    end case;
                    if v_neg = '1' then z_bus <= (not v_b) + 1; else z_bus <= v_b + v_add; end if;
                end if;

                -- derived operands and the register-file writes
                if z_p = 23 then
                    case to_integer(z_s) is
                        when 1 => z_xs0 <= signed(resize(w_dxm, 13)) + z_cx;
                        when 2 => v_s13 := to_signed(1080, 13) + z_cy; z_Sy <= unsigned(v_s13(11 downto 0));
                        when 4 =>
                            v_adj := shift_right(z_t, 1) + shift_right(z_t, 3);
                            z_kj <= f_addsub(c_k31, v_adj, not z_np);
                        when 7  => z_wr <= '1'; z_wf <= "000";
                        when 9  => z_wr <= '1'; z_wf <= "001";
                        when 10 => z_wr <= '1'; z_wf <= "010";
                        when 11 => z_wr <= '1'; z_wf <= "011";
                        when 12 => z_wr <= '1'; z_wf <= "100";
                        when others => null;
                    end case;
                end if;
                if z_p = 0 then
                    case to_integer(z_s) is
                        when 2 =>
                            v_s13 := abs(z_xs0); z_aS <= unsigned(v_s13(11 downto 0));
                            case to_integer(s_xs) is
                                when 4      => z_SX <= shift_left(resize(z_xs0, 15), 2);
                                when 3      => z_SX <= shift_left(resize(z_xs0, 15), 1) + resize(z_xs0, 15);
                                when others => z_SX <= shift_left(resize(z_xs0, 15), 1);
                            end case;
                        when 3 => z_aMe <= shift_left(resize(z_Sy, 20), 8) - resize(s_ys, 20);
                        when others => null;
                    end case;
                end if;
                if z_p = 1 and z_s = 2 then
                    z_m2 <= shift_left(resize(z_SX, 17), 1) - signed(resize(w_x2, 17));
                end if;
                if z_p = 2 and z_s = 2 then
                    v_s17 := abs(z_m2); z_aMd <= unsigned(v_s17(14 downto 0));
                    z_smd <= not z_m2(16);
                end if;
            end if;

            -- [E] palette (zc 512 .. 563): 12 words from the table, one every 4 clocks (8 bits kept)
            if s_zone = '1' and s_zc(10 downto 6) = "01000" and s_zc(1 downto 0) = "00" then
                v_pi := s_zc(5 downto 2);
                r_ex_a <= '1' & (resize(s_pal & "000", 8) + resize(s_pal & "00", 8) + resize(v_pi, 8));
                case to_integer(v_pi) is
                    when 1  => c_py(0) <= r_exf(9 downto 2);
                    when 2  => c_pu(0) <= r_exf(9 downto 2);
                    when 3  => c_pv(0) <= r_exf(9 downto 2);
                    when 4  => c_py(1) <= r_exf(9 downto 2);
                    when 5  => c_pu(1) <= r_exf(9 downto 2);
                    when 6  => c_pv(1) <= r_exf(9 downto 2);
                    when 7  => c_py(2) <= r_exf(9 downto 2);
                    when 8  => c_pu(2) <= r_exf(9 downto 2);
                    when 9  => c_pv(2) <= r_exf(9 downto 2);
                    when 10 => c_py(3) <= r_exf(9 downto 2);
                    when 11 => c_pu(3) <= r_exf(9 downto 2);
                    when 12 => c_pv(3) <= r_exf(9 downto 2);
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
            s_st <= g_start & s_st(0 to 8);
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
    -- [C] ENGINE, line and beam DDAs.  After every line (and once in the zone, after the sequencer) the
    -- register file is walked, 8 clocks per blob: Q += E and E += EE go back into the file, and the beam is
    -- loaded with the pre-pixel of the next line THROUGH ITS OWN ADDERS (no load multiplexers):
    --   phase 3  b_q, b_d cleared; c_dd <- Q        phase 4  b_d += c_dd (= Q)       phase 5  b_q += b_d (= Q)
    --   phase 6  b_d cleared; c_dd <- D0            phase 7  b_d += c_dd (= D0); c_dd <- DD
    -- It then holds until g_start.  Along the line: b_q += b_d, b_d += c_dd (stage 2 = pixel 0).
    ---------------------------------------------------------------------------
    p_dda : process(clk)
        variable v_sel : boolean;
    begin
        if rising_edge(clk) then
            l_kick <= (s_gav_sr(11) and not s_gav_sr(10)) or z_kick(3);
            if l_kick = '1' then
                l_run <= '1'; l_j <= "00"; l_p <= "000";
            elsif l_run = '1' then
                l_p <= l_p + 1;
                if l_p = 7 then
                    if l_j = C_NBL - 1 then l_run <= '0'; else l_j <= l_j + 1; end if;
                end if;
            end if;
            if l_kick = '1' then b_run <= '0'; elsif g_start = '1' then b_run <= '1'; end if;

            rf_ra <= "000" & l_j & l_p;
            rf_op <= rf_f(34 downto 0);
            rf_sum <= rf_op + rf_f(34 downto 0);
            rw_we <= '0';
            if l_run = '1' and (l_p = 5 or l_p = 6) then
                rw_we <= '1'; rw_a <= "000" & l_j & "00" & l_p(1); rw_d <= resize(rf_sum, 48);
            elsif z_wr = '1' then
                rw_we <= '1'; rw_a <= "000" & z_wj(1 downto 0) & z_wf; rw_d <= resize(z_bus, 48);
            end if;

            for j in 0 to C_NBL - 1 loop
                v_sel := l_run = '1' and l_j = j;
                if v_sel and l_p = 3 then
                    b_q(j) <= (others => '0');
                elsif b_run = '1' or (v_sel and l_p = 5) then
                    b_q(j) <= b_q(j) + b_d(j);
                end if;
                if v_sel and (l_p = 3 or l_p = 6) then
                    b_d(j) <= (others => '0');
                elsif b_run = '1' or (v_sel and (l_p = 4 or l_p = 7)) then
                    b_d(j) <= b_d(j) + c_dd(j);
                end if;
                if v_sel and (l_p = 3 or l_p = 6 or l_p = 7) then c_dd(j) <= rf_f(34 downto 0); end if;
            end loop;
            if s_zgo = '1' then
                e_first <= '1';
            elsif g_start = '1' then
                e_first <= '0';
            end if;
            if g_start = '1' then l_first <= e_first; end if;
        end if;
    end process p_dda;

    ---------------------------------------------------------------------------
    -- [C] ENGINE, pixel path (stages 3 .. 14): falloff, field, PHI, gradient
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_m  : unsigned(13 downto 0);
        variable v_f  : unsigned(18 downto 0);
        variable v_p  : unsigned(24 downto 0);
        variable v_a  : unsigned(15 downto 0);
        variable v_w  : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            for j in 0 to C_NBL - 1 loop
                -- 3: u = 1 - q = not q (14 bits) inside the blob; the square ROM (g_sq) and the cross term
                v_m := (not b_q(j)(25 downto 18)) * (not b_q(j)(17 downto 12));
                s3_m(j) <= v_m(13 downto 5);
                if b_q(j)(34 downto 26) = 0 then s3_in(j) <= '1'; else s3_in(j) <= '0'; end if;
                -- 4: fabric copies
                s4_sq(j) <= sq_q(j); s4_m(j) <= s3_m(j); s4_in(j) <= s3_in(j);
                -- 5: w = u^2 (Q16)
                if s4_in(j) = '1' then
                    s5_w(j) <= s4_sq(j) + resize(s4_m(j), 16);
                else
                    s5_w(j) <= (others => '0');
                end if;
            end loop;
            -- 6 .. 8: the field F = sum of the blobs + the source (Q3.16 -> Q3.13)
            s6_a <= resize(s5_w(0), 17) + resize(s5_w(1), 17);
            s6_b <= resize(s5_w(2), 17) + resize(s5_w(3), 17);
            -- the source, x 4 (K6 has already scaled it: K6 = 512 on the camera = twice v0.1.0's half gain)
            s6_c <= s_srcm & "00";
            s7_ab <= resize(s6_a, 18) + resize(s6_b, 18);
            s7_c <= s6_c;
            v_f := resize(s7_ab, 19) + resize(s7_c, 19);
            s8_f <= v_f(18 downto 3);
            -- 9: F * bands, the multiplier split in two
            s9_p1 <= s8_f * c_nb(3 downto 0);
            s9_p2 <= s8_f * c_nb(8 downto 4);
            -- 10: PHI (Q6.12 bands), held at 63.25 past 63.5
            v_p := resize(s9_p1, 25) + (s9_p2 & "0000");
            if v_p(24 downto 23) /= "00" or v_p(22 downto 16) = "1111111" then
                s10_phi <= to_unsigned(16#3F400#, 18);
            else
                s10_phi <= v_p(22 downto 5);
            end if;
            s10_ab <= ph_q;
            -- 11: gradient by finite differences (per real pixel); the line above only at every fourth pixel
            s11_gx <= signed(s10_phi(15 downto 0) - s11_phi(15 downto 0));
            if s_c10(1 downto 0) = "00" then s11_gy <= signed(s10_phi(15 downto 0) - s10_ab); end if;
            s11_phi <= s10_phi;
            ph_wd <= s10_phi(15 downto 0);
            ph_wa <= s_c10(10 downto 2);
            if s_c10(1 downto 0) = "00" then ph_we <= s_gav_sr(8) or s_gav_sr(16); else ph_we <= '0'; end if;
            -- 12: magnitudes, clamped to 2 bands per pixel; no line above on the first line
            if s11_gx(15) = '1' then v_a := unsigned(-s11_gx); else v_a := unsigned(s11_gx); end if;
            if v_a(15 downto 13) /= "000" then s12_agx <= (others => '1'); else s12_agx <= v_a(12 downto 0); end if;
            if s11_gy(15) = '1' then v_a := unsigned(-s11_gy); else v_a := unsigned(s11_gy); end if;
            if l_first = '1' then s12_agy <= (others => '0');
            elsif v_a(15 downto 13) /= "000" then s12_agy <= (others => '1');
            else s12_agy <= v_a(12 downto 0); end if;
            s12_phi <= s11_phi;
            -- 13, 14: |grad PHI| by the octagonal metric max + 3/8 min (bands per pixel, Q12)
            if s12_agx >= s12_agy then s13_mx <= s12_agx; s13_mn <= s12_agy;
            else s13_mx <= s12_agy; s13_mn <= s12_agx; end if;
            s13_phi <= s12_phi;
            v_w := resize(s13_mx, 14) + resize(shift_right(s13_mn, 2), 14) + resize(shift_right(s13_mn, 3), 14);
            if v_w(13) = '1' then s14_w <= (others => '1'); else s14_w <= v_w(12 downto 0); end if;
            s14_phi <= s13_phi;

            -- column counters of the line store
            if s_st(6) = '1' then ph_ra <= (others => '0'); else ph_ra <= ph_ra + 1; end if;
            if s_st(8) = '1' then s_c10 <= (others => '0'); else s_c10 <= s_c10 + 1; end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> horizontal 1/32 IIR, reloaded
    -- at every line start -> vertical 1/32 IIR in the line store
    -- (16-bit state, half horizontal resolution).  The pixel path takes the
    -- store's read (the line above), interpolated.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_lc : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(17 downto 0);
        variable v_a : signed(17 downto 0);
        variable v_m : unsigned(16 downto 0);
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

            -- the source sample: the lattice, or luma cored at about 30 % ((luma >> 2) - 80), so only the
            -- picture's bright areas rise and a dim background leaves the coastlines alone
            -- K6 top +25 % (2026-11-15): the cored camera sample x 1.25 (<= 175 + 43 = 218, 8 bits), so the camera's
            -- K6 / 32 gain reaches 31 / 16 x 1.25 at 100 %.  The Osc is left as it was: it already fills the IIRs
            -- (s_slg <= 988, s_h <= 31616 of 32767, the line store <= 63232 of 65535), so 1.25 x would wrap them
            v_l := unsigned(data_in.y(9 downto 2));
            v_lc := v_l - 80;
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 80 then s_sl <= v_lc + shift_right(v_lc, 2);
            else s_sl <= (others => '0'); end if;
            -- K6 camera pass: the gain, one 8 x 5 multiply in its own stage, then the Osc / camera shift (Q8.2
            -- out: camera s_sl x K6 / 512, Osc twice that; <= 988).  The avid flag takes two more registers so
            -- the source path stays aligned (the field moves 2 px right)
            s_slp <= s_sl * s_cm;
            if s_src = '1' then s_slg <= resize(s_slp(12 downto 3), 10); else s_slg <= resize(s_slp(12 downto 4), 10); end if;
            s_sv0 <= data_in.avid; s_sv1 <= s_sv0; s_sv <= s_sv1; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- horizontal 1/32 IIR (Q10.5 of the Q8.2 sample), reloaded with the first sample at every line start
            if s_sv = '1' and s_svd = '0' then
                s_h <= s_slg & "00000";
            elsif s_sv = '1' then
                s_h <= s_h - shift_right(s_h, 5) + resize(s_slg, 15);
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on the odd columns: q += (2 h - q + 16) >> 5  (q = 64 x the Q8.2 sample = 256 x
            -- the 8-bit sample at unity gain, as before)
            s_eb <= s_h; s_eq <= lb_q1;
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb & "0", 18)) - signed(resize(s_eq, 18)) + 16;
            v_a := signed(resize(s_eq, 18)) + shift_right(v_d, 5);
            lb_wd <= unsigned(v_a(15 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- pixel path: the mean of this read and the last (a linear interpolation of the half-resolution store)
            s_eqd <= s_eq;
            v_m := resize(s_eq, 17) + resize(s_eqd, 17);
            s_srcm <= v_m(16 downto 1);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] COVERAGE + AA BLEND (stages 15 .. 25)
    --   weff = |grad|, widened past 1/4 band per pixel so fine bands fade to their mean before they alias
    --   rho  = 8 * (distance to the nearest contour) / weff, in 1/8 px, 0 .. 31 (a 5-step divider)
    --   a contour with a line: tint of this side <-> line colour, coverage = clamp(hw + 1/2 - rho)
    --   a contour without:     tint below <-> tint above, coverage = clamp(1/2 +- rho)        (9 levels)
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_w5 : unsigned(15 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_g  : signed(12 downto 0);
        variable v_o  : signed(12 downto 0);
        variable v_win : std_logic;
        variable v_c  : unsigned(5 downto 0);
        variable v_heavy, v_line : std_logic;
        variable v_rho : unsigned(4 downto 0);
        variable v_hw : unsigned(5 downto 0);
        variable v_cl, v_ct : unsigned(3 downto 0);
        variable v_x  : unsigned(5 downto 0);
        variable v_lo : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- 15: weff; the nearest contour c, the side, the distance (Q12 bands)
            if s14_w(12 downto 10) /= "000" then
                v_w5 := shift_left(resize(s14_w, 16), 2) + resize(s14_w, 16) - 4096;
                if v_w5(15 downto 13) /= "000" then s15_weff <= (others => '1'); else s15_weff <= v_w5(12 downto 0); end if;
            else
                s15_weff <= s14_w;
            end if;
            s15_above <= not s14_phi(11);
            if s14_phi(11) = '0' then
                s15_c <= s14_phi(17 downto 12);
                s15_sda <= '0' & s14_phi(10 downto 0);
            else
                s15_c <= s14_phi(17 downto 12) + 1;
                s15_sda <= to_unsigned(2048, 12) - resize(s14_phi(10 downto 0), 12);
            end if;
            -- 16: the 2-pixel quotient bit; saturation
            if s15_weff(12 downto 10) = "000" and s15_sda >= (s15_weff(9 downto 0) & "00") then dv_sat(0) <= '1'; else dv_sat(0) <= '0'; end if;
            v_t := s15_weff & '0';
            if resize(s15_sda, 14) >= v_t then
                dv_r(0) <= resize(resize(s15_sda, 14) - v_t, 13); dv_q(0) <= "00001";
            else
                dv_r(0) <= resize(s15_sda, 13); dv_q(0) <= "00000";
            end if;
            dv_w(0) <= s15_weff; dv_c(0) <= s15_c; dv_above(0) <= s15_above;
            -- 17: the 1-pixel bit
            if dv_r(0) >= dv_w(0) then
                dv_r(1) <= dv_r(0) - dv_w(0); dv_q(1) <= dv_q(0)(3 downto 0) & '1';
            else
                dv_r(1) <= dv_r(0); dv_q(1) <= dv_q(0)(3 downto 0) & '0';
            end if;
            dv_w(1) <= dv_w(0);
            -- 18 .. 20: three fraction bits
            for i in 2 to 4 loop
                v_t := dv_r(i - 1) & '0';
                if v_t >= resize(dv_w(i - 1), 14) then
                    dv_r(i) <= resize(v_t - resize(dv_w(i - 1), 14), 13);
                    dv_q(i) <= dv_q(i - 1)(3 downto 0) & '1';
                else
                    dv_r(i) <= v_t(12 downto 0);
                    dv_q(i) <= dv_q(i - 1)(3 downto 0) & '0';
                end if;
                dv_w(i) <= dv_w(i - 1);
            end loop;
            for i in 1 to 4 loop
                dv_sat(i) <= dv_sat(i - 1); dv_above(i) <= dv_above(i - 1); dv_c(i) <= dv_c(i - 1);
            end loop;
            -- 21: which colours this contour blends (as palette indexes); coverage 0 .. 8
            v_c := dv_c(4);
            if s_index = '1' and v_c(1 downto 0) = "01" then v_heavy := '1'; else v_heavy := '0'; end if;
            if v_c /= 0 and (s_lines = '1' or v_heavy = '1') then v_line := '1'; else v_line := '0'; end if;
            if dv_sat(4) = '1' then v_rho := (others => '1'); else v_rho := dv_q(4); end if;
            if v_heavy = '1' then v_hw := w_hwi; else v_hw := to_unsigned(12, 6); end if;
            if resize(v_rho, 6) >= v_hw then v_cl := (others => '0');
            else
                v_x := v_hw - resize(v_rho, 6);
                if v_x >= 8 then v_cl := to_unsigned(8, 4); else v_cl := v_x(3 downto 0); end if;
            end if;
            if v_rho >= 4 then
                if dv_above(4) = '1' then v_ct := to_unsigned(8, 4); else v_ct := (others => '0'); end if;
            elsif dv_above(4) = '1' then v_ct := to_unsigned(4, 4) + resize(v_rho(1 downto 0), 4);
            else v_ct := to_unsigned(4, 4) - resize(v_rho(1 downto 0), 4); end if;
            if v_line = '1' then s21_cov <= v_cl; else s21_cov <= v_ct; end if;
            if v_c = 0 or (v_line = '1' and dv_above(4) = '1') then v_lo := v_c; else v_lo := v_c - 1; end if;
            if s_lines = '1' then s21_pi <= "00"; else s21_pi <= f_tint(v_lo); end if;
            if v_line = '0' then s21_qi <= f_tint(v_c);
            elsif v_heavy = '1' then s21_qi <= "11";
            else s21_qi <= "01"; end if;
            -- 22: the two colours
            s22_p <= c_pv(to_integer(s21_pi)) & c_pu(to_integer(s21_pi)) & c_py(to_integer(s21_pi));
            s22_q <= c_pv(to_integer(s21_qi)) & c_pu(to_integer(s21_qi)) & c_py(to_integer(s21_qi));
            s22_cov <= s21_cov;
            -- 23: the difference
            s23_dy <= signed('0' & s22_q(7 downto 0)) - signed('0' & s22_p(7 downto 0));
            s23_du <= signed('0' & s22_q(15 downto 8)) - signed('0' & s22_p(15 downto 8));
            s23_dv <= signed('0' & s22_q(23 downto 16)) - signed('0' & s22_p(23 downto 16));
            s23_p <= s22_p; s23_cov <= s22_cov;
            -- 24: difference * coverage as two gated terms
            if s23_cov(3) = '1' then
                s24_ay <= shift_left(resize(s23_dy, 13), 3); s24_by <= (others => '0');
                s24_au <= shift_left(resize(s23_du, 13), 3); s24_bu <= (others => '0');
                s24_av <= shift_left(resize(s23_dv, 13), 3); s24_bv <= (others => '0');
            else
                if s23_cov(2) = '1' then v_g := shift_left(resize(s23_dy, 13), 2); else v_g := (others => '0'); end if;
                if s23_cov(1) = '1' then s24_ay <= v_g + shift_left(resize(s23_dy, 13), 1); else s24_ay <= v_g; end if;
                if s23_cov(0) = '1' then s24_by <= resize(s23_dy, 13); else s24_by <= (others => '0'); end if;
                if s23_cov(2) = '1' then v_g := shift_left(resize(s23_du, 13), 2); else v_g := (others => '0'); end if;
                if s23_cov(1) = '1' then s24_au <= v_g + shift_left(resize(s23_du, 13), 1); else s24_au <= v_g; end if;
                if s23_cov(0) = '1' then s24_bu <= resize(s23_du, 13); else s24_bu <= (others => '0'); end if;
                if s23_cov(2) = '1' then v_g := shift_left(resize(s23_dv, 13), 2); else v_g := (others => '0'); end if;
                if s23_cov(1) = '1' then s24_av <= v_g + shift_left(resize(s23_dv, 13), 1); else s24_av <= v_g; end if;
                if s23_cov(0) = '1' then s24_bv <= resize(s23_dv, 13); else s24_bv <= (others => '0'); end if;
            end if;
            s24_p <= s23_p;
            -- 25: (8 P + difference * coverage + 1) / 2 at 10 bits; the ground colour outside pixels 0 .. W + 7
            v_win := s_gav_sr(22) or s_gav_sr(30);
            if v_win = '1' then
                v_o := signed(resize(s24_p(7 downto 0) & "001", 13)) + s24_ay + s24_by;   s25_y <= unsigned(v_o(10 downto 1));
                v_o := signed(resize(s24_p(15 downto 8) & "001", 13)) + s24_au + s24_bu;  s25_u <= unsigned(v_o(10 downto 1));
                v_o := signed(resize(s24_p(23 downto 16) & "001", 13)) + s24_av + s24_bv; s25_v <= unsigned(v_o(10 downto 1));
            else
                s25_y <= c_py(0) & "00"; s25_u <= c_pu(0) & "00"; s25_v <= c_pv(0) & "00";
            end if;
            s_hist <= s_hist(6 downto 0) & v_win;
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 26 .. 29): chroma [1 2 2 2 2 2 2 2 1]/16 as
    -- two adjacent running 8-sums (resynced to the ground colour on every
    -- line), luma [1 2 1]/4 delayed to the chroma's group delay (4 pixels),
    -- luma 13/16 toward black, blanking gate.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 26: running 8-sums (+ 4: rounds the 9-tap sum); they snap to 8 x ground once 8 samples outside
            -- the picture are in, so a glitched sample can't offset the colours for good
            if s_hist = "00000000" then
                s_bu <= (c_pu(0) & "00000") + 4; s_bv2 <= (c_pv(0) & "00000") + 4;
            else
                s_bu  <= s_bu + s25_u - cr_f(9 downto 0);
                s_bv2 <= s_bv2 + s25_v - cr_f(19 downto 10);
            end if;
            s_bud <= s_bu; s_bvd <= s_bv2;
            -- luma taps y(n - 3), y(n - 4), y(n - 5)
            s_yd(1) <= s25_y;
            for i in 2 to 5 loop s_yd(i) <= s_yd(i - 1); end loop;
            s26_ya <= resize(s_yd(3), 12) + resize(s_yd(5), 12) + 2;
            s26_yb <= s_yd(4);
            -- 27
            v_sc := resize(s_bu, 14) + s_bud;  s27_u <= v_sc(13 downto 4);
            v_sc := resize(s_bv2, 14) + s_bvd; s27_v <= v_sc(13 downto 4);
            v_y := s26_ya + shift_left(resize(s26_yb, 12), 1);
            s27_y <= v_y(11 downto 2);
            -- 28: luma 13/16 toward black
            if s27_y > 64 then v_dl := s27_y - 64; else v_dl := (others => '0'); end if;
            s28_y <= s27_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s28_u <= s27_u; s28_v <= s27_v;
            -- 29: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s29_y <= s28_y; s29_u <= s28_u; s29_v <= s28_v;
            else
                s29_y <= to_unsigned(64, 10); s29_u <= to_unsigned(512, 10); s29_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s29_y);
    data_out.u       <= std_logic_vector(s29_u);
    data_out.v       <= std_logic_vector(s29_v);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture isoline;
