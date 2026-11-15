-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Quasi v0.1.1 -- Kinetic.  Quasicrystal interference: the sum of up to eight plane waves at the angles
-- j * pi / N.  N = 5 draws Penrose-like ten-fold rosettes, N = 4 and 8 the Ammann-Beenker family, N = 3 a plain
-- honeycomb; the contour bands of the sum are the picture.
--
-- Element: a plane wave.  State: its phase offset (the "phason" motion: the tiles flip and rearrange in place).
--
--   F(p)   = sum_j a_j * w(k_j . p + phi_j) + source          k_j = k (cos, sin)(j * pi / N + spin)
--   a_j    = clamp(N - j, 0, 1)                               wave j fades in as N passes j
--   phi_j  = Depth * sin(phase + Spread * j) + glide_j        Field = Phason: phase runs; Drift: glide_j runs
--   G      = F * bands;  the contour e = round(G) is drawn as an anti-aliased edge between band colours
--            (Fill) or as a line on the ground (Lines); coverage = 1/2 + distance / |grad G|, 17 steps
--
--   K1  Scale    wavelength 64 .. 724 virtual px (exponential, glided)
--   K2  Shape    the colourizer: two-tone .. many contour bands (1 .. 15.5 bands per 4 wave amplitudes)
--   K3  Depth    phason depth, square law, 0 .. 1 cycle
--   K4  Rate     bipolar, centre = still (60 s per cycle .. 2 cycles per second)
--   K5  Colour   the 16 shared Kinetic palettes (hysteresis; was K6.  Spread, the phase step from wave to
--                wave, is fixed at 300 = 0.04 cycle, square law)
--   K6  Camera   camera / Osc strength: source x 0 .. 3.7 (20 % = v0.1.0's x 1, 50 % = x 2.5; the 2026-11-15
--                +25 % at every position), glided
--   S7  Wave     Smooth (sine) / Angular (triangle: polygonal tiles)
--   S8  Field    Phason (the offsets breathe) / Drift (a rigid glide of the whole pattern)
--   S9  Look     Fill (bands) / Lines (the contours alone)
--   S10 Spin     Off / On (the lattice turns, one turn in 102 s)
--   S11 Source   Video / Osc (the source's luma adds to F: the pattern bends around the picture)
--   P12 SYMMETRY N from 3 (honeycomb) through 5 (Penrose) to 8, continuous
--
-- Per field (vblank zone): glides, the field constants and, for each of the 8 waves, 6 products on one serial
-- multiplier (direction, per-pixel and per-line phase steps, the seed that centres the wave on the screen, the
-- phason offset, the glide step); the results go to a 32-word register file in block RAM.  Then the 8 wave
-- tables (half a cycle, 512 x 8, already scaled by a_j) and the table of pre-blended colours are refilled.
-- Per line: 8 phase accumulators are loaded from the register file through one bus (clear, add the seed, then
-- the step register takes the per-pixel step), and the seeds are advanced by the per-line steps.
-- Per pixel: accumulate, look up 8 waves, sum, add the source, 2-tap box, x bands, gradient by finite
-- differences (previous pixel, line above from a half-resolution line store), coverage, colour table.
--
-- Chassis sections as in riley.vhd: [A] timing and virtual coordinates, [B] controls, [C] wave engine,
-- [D] source field, [E] coverage + colour table, [F] output conditioning.  The integer model
-- .kinetic_work/rb_quasi/imodel.py mirrors each one.
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixels 0 .. W + 7 are rendered and output column j is the 9-tap filter
-- over pixels j .. j + 8.  The window for assigning a stage-S register is s_gav_sr(S - 3) or s_gav_sr(S + 5).
-- One latency (36) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 300 (C_KFIX_5).
-- 2026-11-15: K6 camera top +25 % (Ron)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.quasi_pkg.all;
use work.all;

architecture quasi of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(300, 10));   -- K6 camera pass: fixed knob
    constant C_LAT : integer := 36;   -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 300;
    constant C_DEF_K2  : integer := 220;
    constant C_DEF_K3  : integer := 512;
    constant C_DEF_K5  : integer := 300;
    constant C_DEF_P12 : integer := 410;

    constant C_SPINSTEP : integer := 699051;    -- Q32 cycles per field: one turn in 102 s

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
    -- twice the centre line's number (540 virtual lines down), by timing ID
    function f_ncl2(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(243, 11);
            when 4          => return to_unsigned(480, 11);
            when 8          => return to_unsigned(288, 11);
            when 12         => return to_unsigned(576, 11);
            when 5 | 6 | 14 => return to_unsigned(720, 11);
            when 1 | 2 | 10 => return to_unsigned(540, 11);
            when others     => return to_unsigned(1080, 11);
        end case;
    end function;
    -- K6 Camera: an 8-bit value gated by one gain bit, 10 bits wide
    function f_gu8(x : unsigned(7 downto 0); b : std_logic) return unsigned is
        variable r : unsigned(9 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, 10); end if;
        return r;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ncl2 : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal w_npx  : unsigned(9 downto 0);                          -- pixels from the pre-pixel to the centre

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
    signal s_st       : std_logic_vector(0 to 11) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);     -- K6 Camera (glided)
    signal c_cg : unsigned(5 downto 0) := to_unsigned(40, 6);            -- camera gain, Q4 (16 = x 1)
    signal c_cgw : unsigned(9 downto 0) := to_unsigned(512, 10);         -- K6 top +25 %: the old law in 1/256
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
    signal s_tri, s_drift, s_lines, s_spin, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := (others => '0');

    -- [C] ROMs: C_QS_TAB (2^(i/256), then the palettes), C_QS_SIN (a quarter wave)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q, sn_f : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(9 downto 0) := (others => '0');
    signal z_sh   : signed(3 downto 0) := (others => '0');
    signal z_sh4  : unsigned(2 downto 0) := (others => '0');
    signal z_seg  : unsigned(2 downto 0) := (others => '0');
    signal z_f13  : unsigned(12 downto 0) := (others => '0');
    signal w_dd   : unsigned(15 downto 0);
    signal w_dt   : unsigned(17 downto 0);

    -- [C] per-field constants
    signal c_k32  : unsigned(25 downto 0) := (others => '0');      -- cycles per h (half virtual px), Q32
    signal c_kxs, c_kys : unsigned(25 downto 0) := (others => '0');   -- cycles per pixel / per line, Q28
    signal c_D, c_S : unsigned(15 downto 0) := (others => '0');    -- phason depth, spread (Q16 cycles)
    signal c_delta : unsigned(17 downto 0) := to_unsigned(104858, 18);   -- pi / N, Q20 cycles
    signal c_f6   : unsigned(5 downto 0) := (others => '0');       -- the fading wave's amplitude / 64
    signal c_jf   : unsigned(2 downto 0) := to_unsigned(5, 3);     -- which wave fades
    signal c_sh   : unsigned(1 downto 0) := (others => '0');       -- bands: 2^sh * (32 + m5) / 32
    signal c_m5   : unsigned(4 downto 0) := (others => '0');
    signal c_wmin : unsigned(4 downto 0) := to_unsigned(2, 5);
    signal c_rstep : unsigned(27 downto 0) := (others => '0');     -- phase step per field, Q32
    signal c_ph   : unsigned(31 downto 0) := (others => '0');      -- phason phase, Q32 cycles
    signal c_spin : unsigned(31 downto 0) := (others => '0');      -- lattice angle, Q32 turns
    signal c_gy, c_gu, c_gv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [C] per-wave scratch
    signal w_al   : unsigned(19 downto 0) := (others => '0');      -- direction, Q20 turns
    signal w_th   : unsigned(15 downto 0) := (others => '0');      -- phason argument, Q16 cycles
    signal w_cn, w_sn, w_wn : std_logic;
    signal w_cm, w_sm, w_wm : unsigned(13 downto 0) := (others => '0');
    signal w_cs, w_ss, w_dr : signed(15 downto 0) := (others => '0');
    signal w_dm   : unsigned(14 downto 0) := (others => '0');
    signal w_dn   : std_logic := '0';
    signal w_incm : unsigned(15 downto 0) := (others => '0');
    signal w_dsm, w_gs : unsigned(20 downto 0) := (others => '0');
    signal w_sd, w_g : unsigned(23 downto 0) := (others => '0');

    -- [C] register file: 0..7 seed (this line), 8..15 per-line step, 16..23 per-pixel step << 4, 24..31 glide
    type t_rf is array (0 to 255) of unsigned(31 downto 0);
    signal rf_ram : t_rf := (others => (others => '0'));
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q, rf_d, rf_wd : unsigned(31 downto 0) := (others => '0');
    signal rf_ra, rf_wa : unsigned(4 downto 0);
    signal rf_we : std_logic;
    signal z_ra, e_ra, z_wa, e_wa : unsigned(4 downto 0) := (others => '0');
    signal z_wd, e_wd, e_t : unsigned(23 downto 0) := (others => '0');
    signal z_we, e_we, z_own : std_logic := '0';

    -- [C] line engine and the 8 phase accumulators
    signal e_lc   : unsigned(6 downto 0) := (others => '1');
    signal e_zk, e_trig, e_run, e_first, l_first : std_logic := '0';
    signal e_ld, e_clr, e_stp : std_logic_vector(0 to 7) := (others => '0');
    type t_u20x8 is array (0 to 7) of unsigned(19 downto 0);
    signal a_acc, a_op : t_u20x8 := (others => (others => '0'));

    -- [C] wave tables (in g_tw) and their fill
    type t_u8x8 is array (0 to 7) of unsigned(7 downto 0);
    signal t_q, tf_wd : t_u8x8 := (others => (others => '0'));
    signal tf_wa : unsigned(8 downto 0) := (others => '0');
    signal tf_we, tf_run : std_logic := '0';
    signal tf_c : unsigned(9 downto 0) := (others => '0');
    signal tf_m, tf_m1, tf_m2 : unsigned(7 downto 0) := (others => '0');
    signal tf_pa : unsigned(9 downto 0) := (others => '0');
    signal tf_pb, tf_pab : unsigned(11 downto 0) := (others => '0');
    signal tf_pc, tf_pc1 : unsigned(13 downto 0) := (others => '0');

    -- [C] pixel path
    signal s3_sg : std_logic_vector(0 to 7) := (others => '0');
    type t_s9x8 is array (0 to 7) of signed(8 downto 0);
    signal s4_t : t_s9x8 := (others => (others => '0'));
    type t_s10x4 is array (0 to 3) of signed(9 downto 0);
    signal s5_p : t_s10x4 := (others => (others => '0'));
    signal s6_a, s6_b : signed(10 downto 0) := (others => '0');
    signal s7_f : signed(11 downto 0) := (others => '0');
    signal s8_f, s9_fp : signed(13 downto 0) := (others => '0');   -- K6: one bit wider (Osc up to x 11.75 of the v0.1.0 camera level)
    signal s9_f2 : signed(14 downto 0) := (others => '0');
    signal s10_pa, s10_pb, s10_pc, s11_pab, s11_pc : unsigned(17 downto 0) := (others => '0');
    signal s12_g, s13_gp : unsigned(11 downto 0) := (others => '0');
    signal s13_agx, s13_agy, s13_gl, s14_gl : unsigned(9 downto 0) := (others => '0');
    signal s14_w : unsigned(10 downto 0) := (others => '0');
    signal c10, c12 : unsigned(10 downto 0) := (others => '0');

    -- [C] G line store (the line above at every other pixel, Q1.7 bands)
    type t_ph is array (0 to 1023) of unsigned(7 downto 0);
    signal ph_ram : t_ph := (others => (others => '0'));
    attribute no_rw_check of ph_ram : signal is true;
    signal ph_q, s12_ab, ph_wd : unsigned(7 downto 0) := (others => '0');
    signal ph_wa : unsigned(9 downto 0) := (others => '0');
    signal ph_we : std_logic := '0';

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
    signal lb_q1, lb_wd : unsigned(7 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    -- K6 Camera: the IIR output x c_cg (Q4) as gated sums, in step with v0.1.0's lb_h / lb_d1 / lb_d2 / lb_d3
    signal k_pa, k_pb, k_pc, k_pc1 : unsigned(9 downto 0) := (others => '0');
    signal k_qa : unsigned(11 downto 0) := (others => '0');
    signal k_g2, k_g3 : unsigned(13 downto 0) := (others => '0');
    signal s6_q, s6_qp, s7_l : unsigned(13 downto 0) := (others => '0');   -- source x gain, Q4

    -- [E] coverage
    signal s15_weff, s16_weff, s17_weff, s16_wh : unsigned(11 downto 0) := (others => '0');
    signal s15_dense : std_logic := '0';
    signal s15_hw : unsigned(8 downto 0) := (others => '0');
    signal s15_as : unsigned(9 downto 0) := (others => '0');
    signal s15_s  : signed(9 downto 0) := (others => '0');
    signal s16_d  : signed(10 downto 0) := (others => '0');
    signal s17_e  : signed(13 downto 0) := (others => '0');
    type t_dv12 is array (0 to 4) of unsigned(11 downto 0);
    signal dv_r, dv_w : t_dv12 := (others => (others => '0'));
    type t_dv4 is array (0 to 4) of unsigned(3 downto 0);
    signal dv_q : t_dv4 := (others => (others => '0'));
    signal dv_z, dv_f : std_logic_vector(0 to 4) := (others => '0');
    type t_idx is array (13 to 23) of unsigned(1 downto 0);
    signal s_idx : t_idx := (others => "00");
    signal s23_cov : unsigned(4 downto 0) := (others => '0');

    -- [E] the table of pre-blended colours: (contour & coverage) -> Y / U / V, refilled every field
    type t_bl is array (0 to 255) of unsigned(15 downto 0);
    signal by_ram, bu_ram, bv_ram : t_bl := (others => (others => '0'));
    attribute no_rw_check of by_ram : signal is true;
    attribute no_rw_check of bu_ram : signal is true;
    attribute no_rw_check of bv_ram : signal is true;
    signal bq_y, bq_u, bq_v : unsigned(15 downto 0) := (others => '0');
    signal bl_we : std_logic_vector(2 downto 0) := "000";
    signal bl_wa : unsigned(6 downto 0) := (others => '0');
    signal bl_wd, bl_base : unsigned(9 downto 0) := (others => '0');
    signal bl_diff : signed(10 downto 0) := (others => '0');
    signal bl_acc : signed(14 downto 0) := (others => '0');
    signal bl_c : unsigned(4 downto 0) := (others => '0');
    signal s25_y, s25_u, s25_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cu, s_cv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
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
            r_ex_q <= C_QS_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_QS_SIN(to_integer(sn_a));
            sn_f   <= sn_q;
        end if;
    end process p_rom;

    -- register file: the zone sequencer owns the port in the zone, the line engine afterwards
    rf_ra <= z_ra when z_own = '1' else e_ra;
    rf_we <= z_we or e_we;
    rf_wa <= e_wa when e_we = '1' else z_wa;
    rf_wd <= x"00" & e_wd when e_we = '1' else x"00" & z_wd;
    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf_ram(to_integer(rf_ra));
            rf_d <= rf_q;
            if rf_we = '1' then rf_ram(to_integer(rf_wa)) <= rf_wd; end if;
        end if;
    end process p_rf;

    -- the 8 wave tables: half a cycle each, written in the vblank zone only
    g_tw : for j in 0 to 7 generate
        type t_tw is array (0 to 511) of unsigned(7 downto 0);
        signal tw_ram : t_tw := (others => (others => '0'));
        attribute no_rw_check of tw_ram : signal is true;
    begin
        p_tw : process(clk)
        begin
            if rising_edge(clk) then
                t_q(j) <= tw_ram(to_integer(a_acc(j)(18 downto 10)));
                if tf_we = '1' then tw_ram(to_integer(tf_wa)) <= tf_wd(j); end if;
            end if;
        end process p_tw;
    end generate g_tw;

    -- G line store: read 3 pixels ahead of the write, never the same address
    p_phram : process(clk)
    begin
        if rising_edge(clk) then
            ph_q <= ph_ram(to_integer(c10(10 downto 1)));
            if ph_we = '1' then ph_ram(to_integer(ph_wa)) <= ph_wd; end if;
        end if;
    end process p_phram;

    -- colour tables: written in the vblank zone only
    p_blram : process(clk)
        variable v_a : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            v_a := s_idx(23) & s23_cov;
            bq_y <= by_ram(to_integer(v_a));
            bq_u <= bu_ram(to_integer(v_a));
            bq_v <= bv_ram(to_integer(v_a));
            if bl_we(0) = '1' then by_ram(to_integer(bl_wa)) <= "000000" & bl_wd; end if;
            if bl_we(1) = '1' then bu_ram(to_integer(bl_wa)) <= "000000" & bl_wd; end if;
            if bl_we(2) = '1' then bv_ram(to_integer(bl_wa)) <= "000000" & bl_wd; end if;
        end if;
    end process p_blram;

    -- box delay: read 32 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    -- luma line store (half horizontal resolution): the vertical IIR's own read, 2 addresses ahead of its
    -- write.  The pixel path takes the IIR's output stream through a short delay instead of a second port.
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- combinational helpers: the pre-pixel's distance from the centre, the symmetry segment's table, the signs
    ---------------------------------------------------------------------------
    w_npx <= to_unsigned(361, 10) when s_xs = 4 else to_unsigned(641, 10) when s_xs = 3 else to_unsigned(961, 10);
    w_dt  <= to_unsigned(174763, 18) when z_seg = 0 else to_unsigned(131072, 18) when z_seg = 1 else
             to_unsigned(104858, 18) when z_seg = 2 else to_unsigned(87381, 18) when z_seg = 3 else
             to_unsigned(74898, 18);
    w_dd  <= to_unsigned(43691, 16) when z_seg = 0 else to_unsigned(26214, 16) when z_seg = 1 else
             to_unsigned(17477, 16) when z_seg = 2 else to_unsigned(12483, 16) when z_seg = 3 else
             to_unsigned(9362, 16);
    w_cn <= w_al(19) xor w_al(18);
    w_sn <= w_al(19);
    w_wn <= w_th(15);

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
        variable v_cg : unsigned(10 downto 0);
        variable v_r  : unsigned(27 downto 0);
        variable v_5  : unsigned(15 downto 0);
        variable v_wv : boolean;
        variable v_j  : unsigned(2 downto 0);
        variable v_20 : unsigned(19 downto 0);
        variable v_24 : unsigned(23 downto 0);
        variable v_be : unsigned(1 downto 0);
        variable v_ch : unsigned(1 downto 0);
        variable v_cb, v_ct : unsigned(1 downto 0);
        variable v_pb : unsigned(7 downto 0);
        variable v_bt : unsigned(4 downto 0);
        variable v_ti : unsigned(8 downto 0);
        variable v_tc : unsigned(9 downto 0);
        variable v_pr : unsigned(13 downto 0);
        variable v_g  : unsigned(13 downto 0);
        variable v_e16 : signed(15 downto 0);
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
            elsif s_zc /= 4095 then
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
                s_tri   <= registers_in(6)(0);
                s_drift <= registers_in(6)(1);
                s_lines <= registers_in(6)(2);
                s_spin  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- palette: 16 steps with hysteresis (leave a step only 8 past its edge)
                v_k := unsigned(registers_in(4));
                v_c := signed(resize(v_k, 11)) - signed(resize(s_pal & "100000", 11));
                if v_c > 40 or v_c < -40 then s_pal <= v_k(9 downto 6); end if;
            end if;
            g_wm1 <= s_w - 1;
            -- K6 Camera gain law (Q4, 16 = x 1): k / 16 up to 512 (256 -> x 1 = v0.1.0, 512 -> x 2), then half
            -- the slope, (k + 512) / 32 (1023 -> x 2.94; x 4 left per-line ragged contours in flat picture areas)
            -- K6 top +25 % (2026-11-15): the old law is formed in 1/256 (c_cgw = k, or (k + 512) / 2 past the knee;
            -- old c_cg = c_cgw / 16), then c_cg = 1.25 x it (<= (767 + 191) / 16 = 59 of 63).  Downstream: lb x
            -- c_cg <= 15045 (k_g2, 14 bits), F <= 2047 + 3767 (s8_f, 14-bit signed), G modulo 2^18 by design
            if s_g6(12) = '0' then
                c_cgw <= '0' & s_g6(11 downto 3);
            else
                v_cg := resize(s_g6(12 downto 3), 11) + 512;
                c_cgw <= v_cg(10 downto 1);
            end if;
            c_cg <= resize(shift_right(c_cgw + shift_right(c_cgw, 2), 4), 6);
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            s_ncl2 <= f_ncl2(s_tid);

            -- [B] shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif s_gd > 32 or s_gd < -32 or (z_gc /= 3 and (s_gd > 16 or s_gd < -16)) then
                            s_gd <= shift_right(s_gd, 3);       -- deadband +-2 (K6 Camera: +-4)
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] serial products (16 iterations).  A slot's number is decoded at its step 0 (z_pre), the
            -- operands are pre-muxed at step 1, loaded at step 2, and the product is stable from step 19
            -- (stored at step 20).  Preamble slots: step 0 at zc = 30 + 24 k.  Wave j runs in
            -- zc = 256 (j + 1) .. + 255 as eight sub-slots m of 32 clocks (m = 1 .. 6 multiply).
            v_wv := s_zone = '1' and s_zc(11 downto 8) >= 1 and s_zc(11 downto 8) <= 8;
            v_j  := s_zc(10 downto 8) - 1;
            z_pre <= '0';
            if s_zone = '1' and s_zc(11 downto 8) = 0 then
                case to_integer(s_zc(7 downto 0)) is
                    when 30  => z_sel <= to_unsigned(0, 4); z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4); z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4); z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4); z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4); z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4); z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4); z_pre <= '1';
                    when others => null;
                end case;
            elsif v_wv and s_zc(4 downto 0) = 0 and s_zc(7 downto 5) /= 0 and s_zc(7 downto 5) /= 7 then
                z_sel <= '1' & s_zc(7 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g3(12 downto 3), 40); z_fb <= resize(s_g3(12 downto 3), 16);   -- depth^2
                    when 1  => z_fa <= resize(unsigned(C_KFIX_5), 40); z_fb <= resize(unsigned(C_KFIX_5), 16); -- spread^2 (fixed)
                    when 2  => z_fa <= resize(s_vm, 40);  z_fb <= to_unsigned(897, 16);                 -- rate exponent
                    when 3  => z_fa <= resize(r_exf, 40); z_fb <= to_unsigned(9321, 16);                -- rate step
                    when 4  => z_fa <= resize(w_dd, 40);  z_fb <= resize(z_f13, 16);                    -- pi / N, interpolated
                    when 5  => z_fa <= resize(c_k32, 40); z_fb <= resize(s_xs, 16);                     -- cycles per pixel
                    when 6  => z_fa <= resize(c_k32, 40); z_fb <= resize(s_ys, 16);                     -- cycles per line
                    when 9  => z_fa <= resize(c_kxs, 40); z_fb <= resize(w_cm, 16);                     -- per-pixel step
                    when 10 => z_fa <= resize(c_kys, 40); z_fb <= resize(w_sm, 16);                     -- per-line step
                    when 11 => z_fa <= resize(w_incm, 40); z_fb <= resize(w_npx, 16);                   -- x offset of the centre
                    when 12 => z_fa <= resize(w_dsm, 40); z_fb <= resize(s_ncl2, 16);                   -- y offset of the centre
                    when 13 => z_fa <= resize(c_D, 40);   z_fb <= resize(w_wm, 16);                     -- phason offset
                    when others => z_fa <= resize(c_rstep(27 downto 4), 40); z_fb <= resize(w_dm, 16);  -- glide step
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

            -- the preamble
            if s_zone = '1' and s_zc(11 downto 8) = 0 then
                case to_integer(s_zc(7 downto 0)) is
                    when 20 =>                        -- motion: the phason phase (last field's step), the spin
                        if s_drift = '0' then c_ph <= f_addsub(c_ph, resize(c_rstep, 32), s_vneg); end if;
                        if s_spin = '1' then c_spin <= c_spin + to_unsigned(C_SPINSTEP, 32);
                        else c_spin <= (others => '0'); end if;
                    when 26 =>                        -- symmetry: segment (which wave fades) and the fraction in it
                        v_5 := shift_left(resize(s_gp, 16), 2) + resize(s_gp, 16);
                        z_seg <= v_5(15 downto 13); z_f13 <= v_5(12 downto 0);
                        z_oct <= s_g1(12 downto 3) - resize(shift_right(s_g1(12 downto 3), 3), 10);
                        c_sh <= s_g2(12 downto 11); c_m5 <= s_g2(10 downto 6);
                    when 27 =>
                        v_e := -signed(resize(z_oct, 12));
                        r_ex_a <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(11 downto 8);
                        c_f6 <= z_f13(12 downto 7);
                        c_jf <= z_seg + 3;
                        c_wmin <= resize(shift_right(shift_left(resize('1' & c_m5, 9), to_integer(c_sh)), 4), 5);
                    when 31 =>                        -- wavelength: cycles per h = 2^25 * 2^-oct (Q32)
                        case to_integer(z_sh) is
                            when 0      => c_k32 <= r_exf & "0000000000";
                            when -1     => c_k32 <= resize(r_exf & "000000000", 26);
                            when -2     => c_k32 <= resize(r_exf & "00000000", 26);
                            when -3     => c_k32 <= resize(r_exf & "0000000", 26);
                            when others => c_k32 <= resize(r_exf & "000000", 26);
                        end case;
                    when 50  => c_D <= f_acc(19 downto 4);
                    when 74  => c_S <= f_acc(20 downto 5);
                    when 98  => r_ex_a <= '0' & f_acc(15 downto 8); z_sh4 <= f_acc(18 downto 16);
                    when 122 =>
                        v_r := resize(f_acc(29 downto 8), 28);
                        if s_vm = 0 then c_rstep <= (others => '0');
                        else c_rstep <= shift_left(v_r, to_integer(z_sh4)); end if;
                    when 146 => c_delta <= w_dt - resize(f_acc(28 downto 13), 18);
                    when 170 => c_kxs <= f_acc(29 downto 4);
                    when 194 => c_kys <= f_acc(36 downto 11);
                    when others => null;
                end case;
            end if;

            -- the waves
            z_we <= '0';
            if s_zone = '1' and s_zc(11 downto 8) < 9 then z_own <= '1'; else z_own <= '0'; end if;
            if v_wv then
                case to_integer(s_zc(7 downto 0)) is
                    when 0 =>                         -- the direction and the phason argument step from wave to wave
                        if v_j = 0 then
                            w_al <= c_spin(31 downto 12); w_th <= c_ph(31 downto 16);
                        else
                            w_al <= w_al + resize(c_delta, 20); w_th <= w_th + c_S;
                        end if;
                    when 1 =>                         -- cos, sin, the phason wave: quarter-wave ROM
                        if w_al(18) = '0' then sn_a <= not w_al(17 downto 8); else sn_a <= w_al(17 downto 8); end if;
                    when 2 =>
                        if w_al(18) = '1' then sn_a <= not w_al(17 downto 8); else sn_a <= w_al(17 downto 8); end if;
                    when 3 =>
                        if w_th(14) = '1' then sn_a <= not w_th(13 downto 4); else sn_a <= w_th(13 downto 4); end if;
                    when 4 => w_cm <= sn_f(13 downto 0);
                    when 5 => w_sm <= sn_f(13 downto 0);
                    when 6 => w_wm <= sn_f(13 downto 0);
                    when 7 =>
                        if w_cn = '1' then w_cs <= -signed(resize(w_cm, 16)); else w_cs <= signed(resize(w_cm, 16)); end if;
                        if w_sn = '1' then w_ss <= -signed(resize(w_sm, 16)); else w_ss <= signed(resize(w_sm, 16)); end if;
                    when 8 => w_dr <= w_cs + shift_right(w_ss, 1);          -- the glide's direction (1, 1/2) . k
                    when 9 =>
                        w_dn <= w_dr(15); v_e16 := -w_dr;
                        if w_dr(15) = '1' then w_dm <= unsigned(v_e16(14 downto 0)); else w_dm <= unsigned(w_dr(14 downto 0)); end if;
                    when 52 => w_incm <= f_acc(37 downto 22);              -- |per-pixel step|, Q20 cycles
                    when 53 =>
                        v_20 := resize(w_incm, 20);
                        if w_cn = '1' then v_20 := to_unsigned(0, 20) - v_20; end if;
                        z_we <= '1'; z_wa <= "10" & v_j; z_wd <= v_20 & "0000";
                    when 84 => w_dsm <= f_acc(38 downto 18);               -- |per-line step|, Q24 cycles
                    when 85 =>
                        v_24 := resize(w_dsm, 24);
                        if w_sn = '1' then v_24 := to_unsigned(0, 24) - v_24; end if;
                        z_we <= '1'; z_wa <= "01" & v_j; z_wd <= v_24;
                    when 116 => w_sd <= f_addsub(x"400000", f_acc(19 downto 0) & "0000", not w_cn);   -- a cosine about the centre
                    when 148 => w_sd <= f_addsub(w_sd, f_acc(24 downto 1), not w_sn);
                    when 180 => w_sd <= f_addsub(w_sd, f_acc(29 downto 6), w_wn);                    -- + the phason offset
                    when 202 => z_ra <= "11" & v_j;
                    when 206 => w_g <= rf_d(23 downto 0);
                    when 212 => w_gs <= f_acc(38 downto 18);
                    when 225 =>                       -- the glide runs in Drift and is dropped in Phason
                        if s_drift = '1' then w_g <= f_addsub(w_g, resize(w_gs, 24), w_dn xor s_vneg);
                        else w_g <= (others => '0'); end if;
                    when 226 => w_sd <= w_sd + w_g;
                    when 227 => z_we <= '1'; z_wa <= "00" & v_j; z_wd <= w_sd;
                    when 228 => z_we <= '1'; z_wa <= "11" & v_j; z_wd <= w_g;
                    when others => null;
                end case;
            end if;

            -- [E] the colour table (zc 256 .. 767): 4 contours x 3 channels, 32 clocks each: two palette reads,
            -- then 17 steps from the lower colour to the upper by accumulation.
            --   Fill:  contour e runs from colour e to colour e + 1 (ground, A, B, accent, ground ...)
            --   Lines: every contour runs from the ground to its line colour (A, B, A, accent)
            bl_we <= "000";
            v_be := (not s_zc(8)) & s_zc(7);
            v_ch := s_zc(6 downto 5);
            v_bt := s_zc(4 downto 0);
            if s_lines = '1' then
                v_cb := "00";
                case to_integer(v_be) is
                    when 0 | 2  => v_ct := "01";
                    when 1      => v_ct := "10";
                    when others => v_ct := "11";
                end case;
            else
                v_cb := v_be; v_ct := v_be + 1;
            end if;
            if v_bt = 0 then v_ct := v_cb; end if;
            v_pb := resize(s_pal & "000", 8) + resize(s_pal & "00", 8) + resize(v_ct & '0', 8) + resize(v_ct, 8) + resize(v_ch, 8);
            if s_zone = '1' and (s_zc(11 downto 8) = 1 or s_zc(11 downto 8) = 2) and v_ch /= 3 then
                if v_bt = 0 or v_bt = 1 then
                    r_ex_a <= '1' & v_pb;
                elsif v_bt = 3 then
                    bl_base <= r_exf(9 downto 0);
                    if v_be = 0 then
                        case to_integer(v_ch) is
                            when 0      => c_gy <= r_exf(9 downto 0);
                            when 1      => c_gu <= r_exf(9 downto 0);
                            when others => c_gv <= r_exf(9 downto 0);
                        end case;
                    end if;
                elsif v_bt = 4 then
                    bl_diff <= signed(resize(r_exf(9 downto 0), 11)) - signed(resize(bl_base, 11));
                    bl_acc <= signed(resize(bl_base & "1000", 15));
                    bl_c <= (others => '0');
                elsif v_bt <= 21 then
                    case to_integer(v_ch) is
                        when 0      => bl_we <= "001";
                        when 1      => bl_we <= "010";
                        when others => bl_we <= "100";
                    end case;
                    bl_wa <= v_be & bl_c;
                    bl_wd <= unsigned(bl_acc(13 downto 4));
                    bl_acc <= bl_acc + resize(bl_diff, 15);
                    bl_c <= bl_c + 1;
                end if;
            end if;

            -- [C] the wave tables (from zc 2336, after the waves have finished with the sine ROM): entry i of
            -- the half cycle = sine or triangle, 8 bits; wave j gets it whole (j below the fading wave), scaled
            -- by f6 / 64 (the fading wave) or zero (above it).  One entry per clock through a 6-step pipe.
            tf_we <= '0';
            if s_zone = '1' and s_zc = 2336 then
                tf_run <= '1'; tf_c <= (others => '0');
            elsif tf_run = '1' then
                tf_c <= tf_c + 1;
                if tf_c = 520 then tf_run <= '0'; end if;
                if tf_c(8) = '0' then sn_a <= tf_c(7 downto 0) & "01"; else sn_a <= (not tf_c(7 downto 0)) & "10"; end if;
                v_tc := tf_c - 3;
                if s_tri = '0' then tf_m <= sn_f(13 downto 6);
                elsif v_tc(8) = '0' then tf_m <= v_tc(7 downto 0);
                else tf_m <= not v_tc(7 downto 0); end if;
                -- m * f6 in two steps of gated shifts
                if c_f6(0) = '1' then v_g := resize(tf_m, 14); else v_g := (others => '0'); end if;
                if c_f6(1) = '1' then tf_pa <= resize(v_g + shift_left(resize(tf_m, 14), 1), 10); else tf_pa <= v_g(9 downto 0); end if;
                if c_f6(2) = '1' then v_g := shift_left(resize(tf_m, 14), 2); else v_g := (others => '0'); end if;
                if c_f6(3) = '1' then tf_pb <= resize(v_g + shift_left(resize(tf_m, 14), 3), 12); else tf_pb <= v_g(11 downto 0); end if;
                if c_f6(4) = '1' then v_g := shift_left(resize(tf_m, 14), 4); else v_g := (others => '0'); end if;
                if c_f6(5) = '1' then tf_pc <= v_g + shift_left(resize(tf_m, 14), 5); else tf_pc <= v_g; end if;
                tf_m1 <= tf_m;
                tf_pab <= resize(tf_pa, 12) + tf_pb; tf_pc1 <= tf_pc; tf_m2 <= tf_m1;
                v_pr := resize(tf_pab, 14) + tf_pc1;
                for j in 0 to 7 loop
                    if j < to_integer(c_jf) then tf_wd(j) <= tf_m2;
                    elsif j = to_integer(c_jf) then tf_wd(j) <= v_pr(13 downto 6);
                    else tf_wd(j) <= (others => '0'); end if;
                end loop;
                v_tc := tf_c - 6;
                tf_wa <= v_tc(8 downto 0);
                if tf_c >= 6 and tf_c <= 517 then tf_we <= '1'; end if;
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
            s_st <= g_start & s_st(0 to 10);
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
    -- [C] WAVE ENGINE, line part.  Kicked after every line (once the tail
    -- pixels have left the accumulators) and once at the end of the zone:
    --   steps 0 .. 19   load: for each wave, clear the accumulator, add the seed
    --                   (the phase at the pre-pixel), then leave the per-pixel
    --                   step in the operand register.  The accumulators then
    --                   wait for g_start.
    --   steps 32 .. 64  advance each seed by its per-line step for the next line.
    -- Register-file data is on rf_d three steps after its address is issued.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_u : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 2320 then e_zk <= '1'; end if;     -- after the field constants
            e_trig <= s_gav_sr(11) and not s_gav_sr(10);

            if e_zk = '1' then e_first <= '1'; elsif g_start = '1' then e_first <= '0'; end if;
            if g_start = '1' then l_first <= e_first; end if;

            if e_zk = '1' or e_trig = '1' then
                e_lc <= (others => '0'); e_run <= '0';
            else
                if e_lc /= 127 then e_lc <= e_lc + 1; end if;
                if g_start = '1' then e_run <= '1'; end if;
            end if;

            -- read addresses
            if e_lc(6 downto 4) = "000" then
                e_ra <= e_lc(0) & '0' & e_lc(3 downto 1);          -- seed j, step j
            elsif e_lc(1 downto 0) = "00" then
                e_ra <= "00" & e_lc(4 downto 2);                   -- seed j
            else
                e_ra <= "01" & e_lc(4 downto 2);                   -- per-line step j
            end if;

            -- load strobes: wave j's seed is on rf_d at step 2 j + 3, its step at 2 j + 4
            v_u := resize(e_lc - 2, 5);
            for j in 0 to 7 loop
                e_ld(j) <= '0'; e_clr(j) <= '0'; e_stp(j) <= '0';
                if e_lc >= 2 and e_lc <= 17 and to_integer(v_u(3 downto 1)) = j then
                    e_ld(j) <= '1';
                    e_clr(j) <= not v_u(0);
                    e_stp(j) <= v_u(0);
                end if;
            end loop;

            -- seed advance: seed j on rf_d at step 35 + 4 j, its step at 36 + 4 j, written back at 37 + 4 j
            e_we <= '0';
            if e_lc(1 downto 0) = "11" then e_t <= rf_d(23 downto 0); end if;
            if (e_lc(6 downto 5) = "01" and e_lc(1 downto 0) = "00" and e_lc(4 downto 2) /= 0) or e_lc = 64 then
                e_wd <= e_t + rf_d(23 downto 0);
                e_wa <= "00" & (e_lc(4 downto 2) - 1);
                e_we <= '1';
            end if;
        end if;
    end process p_line;

    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            for j in 0 to 7 loop
                if e_ld(j) = '1' then a_op(j) <= rf_d(23 downto 4); end if;
                if e_clr(j) = '1' then
                    a_acc(j) <= (others => '0');
                elsif e_run = '1' or e_stp(j) = '1' then
                    a_acc(j) <= a_acc(j) + a_op(j);
                end if;
            end loop;
        end if;
    end process p_acc;

    ---------------------------------------------------------------------------
    -- [C] WAVE ENGINE, pixel path (stages 2 .. 14): accumulators (2), tables (3),
    -- signs (4), sum (5 .. 7), + source (8), 2-tap box (9), x bands (10 .. 12),
    -- gradient (13), w = |grad G| (14)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_lt : unsigned(12 downto 0);
        variable v_x, v_g0, v_g1, v_p : unsigned(17 downto 0);
        variable v_gq, v_d1, v_d2 : unsigned(11 downto 0);
        variable v_e1, v_e2 : unsigned(7 downto 0);
        variable v_w : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: the tables' own registers (g_tw); 4: the half-cycle sign (value = m or -m - 1)
            for j in 0 to 7 loop
                s3_sg(j) <= a_acc(j)(19);
                if s3_sg(j) = '1' then s4_t(j) <= signed('1' & (not t_q(j))); else s4_t(j) <= signed('0' & t_q(j)); end if;
            end loop;
            -- 5 .. 7: the sum of the eight
            for i in 0 to 3 loop
                s5_p(i) <= resize(s4_t(2 * i), 10) + resize(s4_t(2 * i + 1), 10);
            end loop;
            s6_a <= resize(s5_p(0), 11) + resize(s5_p(1), 11);
            s6_b <= resize(s5_p(2), 11) + resize(s5_p(3), 11);
            s7_f <= resize(s6_a, 12) + resize(s6_b, 12);
            -- 8: + the source (camera x gain, Osc x 4 gain; s7_l is Q4) and the half-unit offsets of the eight signs
            if s_src = '1' then v_lt := resize(s7_l(13 downto 2), 13) + 4; else v_lt := resize(s7_l(13 downto 4), 13) + 4; end if;
            s8_f <= resize(s7_f, 14) + signed(resize(v_lt, 14));
            -- 9: 2-tap box (halves the table quantisation noise)
            s9_fp <= s8_f;
            s9_f2 <= resize(s8_f, 15) + resize(s9_fp, 15);
            -- 10, 11: x (32 + m5) as three pairs of gated shifts, modulo 2^18
            v_x := unsigned(resize(s9_f2, 18));
            if c_m5(0) = '1' then v_g0 := v_x; else v_g0 := (others => '0'); end if;
            if c_m5(1) = '1' then v_g1 := shift_left(v_x, 1); else v_g1 := (others => '0'); end if;
            s10_pa <= v_g0 + v_g1;
            if c_m5(2) = '1' then v_g0 := shift_left(v_x, 2); else v_g0 := (others => '0'); end if;
            if c_m5(3) = '1' then v_g1 := shift_left(v_x, 3); else v_g1 := (others => '0'); end if;
            s10_pb <= v_g0 + v_g1;
            if c_m5(4) = '1' then v_g0 := shift_left(v_x, 4); else v_g0 := (others => '0'); end if;
            s10_pc <= v_g0 + shift_left(v_x, 5);
            s11_pab <= s10_pa + s10_pb; s11_pc <= s10_pc;
            -- 12: G = bands, Q2.10 modulo 4, offset half a band so that contour e sits at G(9..0) = 512
            v_p := s11_pab + s11_pc;
            case to_integer(c_sh) is
                when 0      => v_gq := v_p(17 downto 6);
                when 1      => v_gq := v_p(16 downto 5);
                when 2      => v_gq := v_p(15 downto 4);
                when others => v_gq := v_p(14 downto 3);
            end case;
            s12_g <= v_gq + 512;
            s12_ab <= ph_q;
            -- 13: |gradient| by finite differences per real pixel: against the previous pixel, and against the
            -- line above at every other pixel (held for the pixel between)
            v_d1 := s12_g - s13_gp; v_d2 := s13_gp - s12_g;
            s13_gp <= s12_g;
            if v_d1(11) = '1' then v_d1 := v_d2; end if;
            if v_d1(11 downto 10) /= "00" then s13_agx <= (others => '1'); else s13_agx <= v_d1(9 downto 0); end if;
            if c12(0) = '0' then
                v_e1 := s12_g(10 downto 3) - s12_ab; v_e2 := s12_ab - s12_g(10 downto 3);
                if v_e1(7) = '1' then v_e1 := v_e2; end if;
                if l_first = '1' then s13_agy <= (others => '0');
                elsif v_e1(7) = '1' then s13_agy <= (others => '1');
                else s13_agy <= v_e1(6 downto 0) & "000"; end if;
            end if;
            ph_wd <= s12_g(10 downto 3); ph_wa <= c12(10 downto 1);
            ph_we <= (s_gav_sr(10) or s_gav_sr(18)) and not c12(0);
            s13_gl <= s12_g(9 downto 0);
            -- 14: w = |grad G| (bands per pixel, Q10), floored at about two table steps
            v_w := resize(s13_agx, 11) + resize(s13_agy, 11);
            if v_w < resize(c_wmin, 11) then s14_w <= resize(c_wmin, 11); else s14_w <= v_w; end if;
            s14_gl <= s13_gl;

            -- pixel counters at stages 10 and 12 (the line store's read and write columns)
            if s_st(8) = '1' then c10 <= (others => '0'); else c10 <= c10 + 1; end if;
            if s_st(10) = '1' then c12 <= (others => '0'); else c12 <= c12 + 1; end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 32-px box, edge-replicated
    -- and reloaded at every line start -> vertical 1/8 IIR in the line store
    -- (half horizontal resolution).  The pixel path interpolates its output stream:
    -- stages 6, 7.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d : signed(9 downto 0);
        variable v_a : unsigned(8 downto 0);
        variable v_a15 : unsigned(14 downto 0);
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

            -- pixel path: the IIR output for store address a (source column 2 a + 1) is held in lb_h for two
            -- clocks and reaches stage 6 as pixels 2 a and 2 a + 1 after 3 or 4 more clocks (the generated
            -- avid starts 3 or 4 clocks after the source's); 7 the mean with the previous pixel (a linear
            -- interpolation of the half-resolution field; pixel 0 has no previous), zero outside pixels 0 .. W + 7
            -- K6 Camera: the gain is applied along this delay chain (k_p* = lb_h's slot, k_qa = lb_d1,
            -- k_g2 = lb_d2, k_g3 = lb_d3), so the source keeps its timing; c_cg = 16 is v0.1.0's x 1
            if lb_we = '1' then
                k_pa <= f_gu8(lb_wd, c_cg(0)) + shift_left(f_gu8(lb_wd, c_cg(1)), 1);
                k_pb <= f_gu8(lb_wd, c_cg(2)) + shift_left(f_gu8(lb_wd, c_cg(3)), 1);
                k_pc <= f_gu8(lb_wd, c_cg(4)) + shift_left(f_gu8(lb_wd, c_cg(5)), 1);
            end if;
            k_qa <= resize(k_pa, 12) + shift_left(resize(k_pb, 12), 2); k_pc1 <= k_pc;
            k_g2 <= resize(k_qa, 14) + shift_left(resize(k_pc1, 14), 4);
            k_g3 <= k_g2;
            if g_sel = '0' then s6_q <= k_g2; else s6_q <= k_g3; end if;
            s6_qp <= s6_q;
            if s_st(5) = '1' then v_a15 := s6_q & '1'; else v_a15 := resize(s6_q, 15) + resize(s6_qp, 15) + 16; end if;
            if (s_gav_sr(4) or s_gav_sr(12)) = '1' then s7_l <= v_a15(14 downto 1); else s7_l <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] COVERAGE + COLOUR TABLE (stages 15 .. 25)
    --   weff = w, widened past 1/4 band per pixel so the bands fade to their mean before they alias
    --   Fill:  d = the signed distance from contour e (bands, Q10)
    --   Lines: d = half the line weight (1.5 px at 1080, 1 px otherwise) - |distance|
    --   cov  = clamp(16 (d + weff / 2) / weff + 1/2);  colour = table(e, cov)
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_w5 : unsigned(13 downto 0);
        variable v_h  : unsigned(11 downto 0);
        variable v_t  : unsigned(12 downto 0);
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            -- 15
            if s14_w(10 downto 8) /= "000" then
                v_w5 := shift_left(resize(s14_w, 14), 2) + resize(s14_w, 14) - 1024;
                if v_w5(13 downto 12) /= "00" then s15_weff <= (others => '1'); else s15_weff <= v_w5(11 downto 0); end if;
                s15_dense <= '1';
            else
                s15_weff <= resize(s14_w, 12);
                s15_dense <= '0';
            end if;
            if s_xs = 2 then v_h := resize(s14_w, 12) + resize(shift_right(s14_w, 1), 12); else v_h := resize(s14_w, 12); end if;
            if v_h > 448 then s15_hw <= to_unsigned(448, 9); else s15_hw <= v_h(8 downto 0); end if;
            if s14_gl(9) = '1' then s15_as <= '0' & s14_gl(8 downto 0);
            else s15_as <= to_unsigned(512, 10) - resize(s14_gl(8 downto 0), 10); end if;
            s15_s <= signed((not s14_gl(9)) & s14_gl(8 downto 0));
            s_idx(13) <= s12_g(11 downto 10);
            for i in 14 to 23 loop s_idx(i) <= s_idx(i - 1); end loop;
            -- 16
            if s_lines = '0' then s16_d <= resize(s15_s, 11);
            elsif s15_dense = '1' then s16_d <= (others => '0');
            else s16_d <= signed(resize(s15_hw, 11)) - signed(resize(s15_as, 11)); end if;
            s16_wh <= shift_right(s15_weff, 1) + shift_right(s15_weff, 5);
            s16_weff <= s15_weff;
            -- 17
            s17_e <= resize(s16_d, 14) + signed(resize(s16_wh, 14));
            s17_weff <= s16_weff;
            -- 18: range tests; 19 .. 22: four quotient bits
            dv_z(0) <= s17_e(13);
            if s17_e >= signed(resize(s17_weff, 14)) then dv_f(0) <= '1'; else dv_f(0) <= '0'; end if;
            dv_r(0) <= unsigned(s17_e(11 downto 0));
            dv_w(0) <= s17_weff;
            for i in 1 to 4 loop
                v_t := dv_r(i - 1) & '0';
                if v_t >= resize(dv_w(i - 1), 13) then
                    dv_r(i) <= resize(v_t - resize(dv_w(i - 1), 13), 12);
                    dv_q(i) <= dv_q(i - 1)(2 downto 0) & '1';
                else
                    dv_r(i) <= v_t(11 downto 0);
                    dv_q(i) <= dv_q(i - 1)(2 downto 0) & '0';
                end if;
                dv_w(i) <= dv_w(i - 1);
                dv_z(i) <= dv_z(i - 1); dv_f(i) <= dv_f(i - 1);
            end loop;
            -- 23: coverage 0 .. 16
            if dv_z(4) = '1' then s23_cov <= (others => '0');
            elsif dv_f(4) = '1' then s23_cov <= to_unsigned(16, 5);
            else s23_cov <= '0' & dv_q(4); end if;
            -- 24: the colour tables' own registers (p_blram)
            -- 25: fabric copies; the ground colour outside pixels 0 .. W + 7
            v_win := s_gav_sr(22) or s_gav_sr(30);
            if v_win = '1' then
                s25_y <= bq_y(9 downto 0); s25_u <= bq_u(9 downto 0); s25_v <= bq_v(9 downto 0);
            else
                s25_y <= c_gy; s25_u <= c_gu; s25_v <= c_gv;
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
            s_cu(0) <= s25_u; s_cv(0) <= s25_v;
            for i in 1 to 7 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            if s_hist = "00000000" then
                s_bu <= (c_gu & "000") + 4; s_bv2 <= (c_gv & "000") + 4;
            else
                s_bu  <= s_bu + s25_u - s_cu(7);
                s_bv2 <= s_bv2 + s25_v - s_cv(7);
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

end architecture quasi;
