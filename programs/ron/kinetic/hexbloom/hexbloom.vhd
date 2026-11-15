-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Hexbloom v0.1.0 -- Kinetic.  Honeycomb op-art: every cell of a hexagonal lattice holds a nest of
-- concentric hexagons (or hexagrams, or triangles) that pumps outwards and back on a travelling wave.
--
-- Element: a hexagonal cell.  State: the ring phase of its nest.
--
--   cell      = the nearer of two lattice centres (this row's, the next row's): one linear test
--   a, b, c   = the three hexagonal axes of the cell-local position (a + b + c = 0)
--   T1 = max(a, b, c)   T2 = max(-a, -b, -c)   D = T2 - T1
--   d         = T1 + g D       g = alpha where D >= 0, beta where D < 0      (Shape: hexagram (0, 1) ..
--                                                                             hexagon (1, 0) .. triangle (0, 0))
--   PHI       = d * K - state        ring = floor(PHI); rings alternate colour A / colour B on the ground
--   state     = Depth * wave(phase + Spread * index) + luma held per cell
--   index     = (h + j) / 2 (Linear: lock-step along one lattice axis) / the hexagonal distance from the
--               centre cell (Radial)
--
--   K1  Scale    cell pitch 64 .. 512 virtual px (exponential, glided)
--   K2  Shape    hexagram .. hexagon (centre) .. triangle
--   K3  Depth    how far the rings pump, 0 .. 4 rings (square law)
--   K4  Rate     wave travel, bipolar, centre = still (60 s per cycle .. 2 cycles per second)
--   K5  Spread   phase step per cell, 0 .. 1/2 cycle (square law)
--   K6  Colour   the 16 shared Kinetic palettes (hysteresis)
--   S7  Wave     Smooth (sine) / Snap (3/4 of the cycle dwells at the extremes, eased flips)
--   S8  Field    Linear / Radial
--   S9  Rings    Bands (half-period rings) / Lines (thin outline rings on the ground)
--   S10 Grout    On (accent line between cells) / Off
--   S11 Source   Video / Osc
--   P12 BLOOM    ring density: 2 rings per cell .. one ring every 12 virtual px
--
-- The pixel path has one multiplier (d * K, 11 x 11, in two halves).  The cell coordinate is one beam accumulator whose
-- integer part is the cell column; the wave index rides a second accumulator that steps when the first one
-- wraps; everything vertical is per line (a small engine in the horizontal blanking, no multiplier).
--
-- Chassis from riley.vhd: [A] timing and virtual coordinates, [B] controls, [C] wave engine (per-field
-- sequencer with one serial multiplier), [D] source (Osc lattice, 32-px box, here held per cell in a
-- 256 x 8 RAM), [E] palette + AA blend (coverage divider, 17-step shift-add blend), [F] output conditioning.
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixels 0 .. W + 7 are rendered, output column j is the 9-tap filter
-- over pixels j .. j + 8.  s_gav_sr(S - 2) flags pixels 0 .. W - 1 at stage S.  One latency (40) in every
-- mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 560 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.hexbloom_pkg.all;
use work.all;

architecture hexbloom of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(560, 10));   -- K6 camera pass: fixed knob
    constant C_LAT : integer := 40;   -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: the K6 camera glide jumps straight to the knob

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 640;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 440;
    constant C_DEF_K5  : integer := 560;
    constant C_DEF_P12 : integer := 512;

    constant C_GW : integer := 48;          -- grout half-width, cell units Q11

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
    -- K6 Camera: gated operand for the gain multiply (a sum of these)
    function f_gu(x : unsigned; b : std_logic; w : natural) return unsigned is
        variable r : unsigned(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
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
    signal s_st       : std_logic_vector(0 to 3) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(C_DEF_K5 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    -- K6 Camera: knob (deadband +-4), glided gain k/256 in Q3 (k = 256: the shipped 1x; 512: 2x; 1023: 4x)
    signal s_kr  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_kt  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_kgd : signed(13 downto 0) := (others => '0');
    signal s_kg  : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_lines, s_goff, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := (others => '0');

    -- [C] ROMs: C_HB_TAB (2^(i/256), then the palettes), C_HB_SIN (a quarter wave)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_sh   : signed(3 downto 0) := (others => '0');
    signal z_sh4  : unsigned(2 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_invp  : unsigned(17 downto 0) := (others => '0');      -- cells per h, Q24
    signal c_invpy : unsigned(18 downto 0) := (others => '0');      -- rows per h, Q24
    signal c_wstep : unsigned(12 downto 0) := (others => '0');      -- rows per line, Q16
    signal c_W0    : signed(21 downto 0) := (others => '0');        -- row coordinate of line 0, Q16
    signal c_US1   : unsigned(28 downto 0) := (others => '0');      -- cell coordinate of the pre-pixel (odd rows), Q22, two's complement
    signal c_ustep : unsigned(17 downto 0) := (others => '0');      -- cells per pixel, Q22
    signal c_Sh    : unsigned(9 downto 0) := (others => '0');       -- half the phase step per cell, 1/4096 cycle
    signal c_AH1, c_AH0, c_AJ0 : signed(17 downto 0) := (others => '0');
    signal c_K     : unsigned(10 downto 0) := to_unsigned(256, 11); -- rings per cell, Q5
    signal c_w     : unsigned(12 downto 0) := to_unsigned(64, 13);  -- rings per pixel, Q12
    signal c_wh    : unsigned(12 downto 0) := to_unsigned(34, 13);  -- w / 2 + w / 32
    signal c_dh    : unsigned(10 downto 0) := to_unsigned(1024, 11);-- half the ring weight, Q12
    signal c_F     : unsigned(11 downto 0) := (others => '0');      -- depth, rings Q9
    signal c_al, c_be : unsigned(4 downto 0) := (others => '0');    -- shape: the two blend weights / 16
    signal c_gsh   : unsigned(2 downto 0) := to_unsigned(2, 3);     -- grout edge: coverage steps per cell unit, as a shift
    signal c_gth   : signed(12 downto 0) := to_signed(1024 - C_GW, 13);
    signal c_rstep : unsigned(27 downto 0) := (others => '0');      -- phase step per field, Q32
    signal c_ph    : unsigned(31 downto 0) := (others => '0');      -- wave phase, Q32 cycles

    -- [E] per-field palette: ground, A, B, accent
    signal c_gy, c_gu, c_gv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_ay, c_au, c_av : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_by, c_bu, c_bv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_cy, c_cu, c_cv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [C] line engine (runs in the horizontal blanking; no multiplier)
    signal e_lc   : unsigned(2 downto 0) := to_unsigned(7, 3);
    signal e_zk, e_trig : std_logic := '0';
    signal e_W    : signed(21 downto 0) := (others => '0');
    signal e_jl, e_inc, e_first, e_gep, e_kick : std_logic := '0';
    signal l_aj, l_ajn, l_jA, l_jB : signed(17 downto 0) := (others => '0');
    signal l_qa, l_aqb : unsigned(10 downto 0) := (others => '0');
    signal l_ta   : signed(12 downto 0) := (others => '0');
    signal l_p, l_samp, l_first : std_logic := '0';

    -- [C] beam accumulators and the pixel path
    signal b_U  : unsigned(28 downto 0) := (others => '0');
    signal b_AH : signed(17 downto 0) := (others => '0');
    signal s_wr : std_logic := '0';
    signal s3_c, s4_c, s4_colb, s5_col : unsigned(6 downto 0) := (others => '0');
    signal s3_f : unsigned(10 downto 0) := (others => '0');
    signal s3_hq : std_logic := '0';
    signal s4_alu : unsigned(9 downto 0) := (others => '0');
    signal s4_sgn, s4_inA, s5_sgn, s5_par, s6_sgn, s7_sgn : std_logic := '0';
    signal s4_ah, s4_ahb, s5_ah, s5_aj, s6_ah, s6_aj, s7_m : signed(17 downto 0) := (others => '0');
    signal s5_alu, s5_aq, s6_alu : unsigned(10 downto 0) := (others => '0');
    signal s6_P, s7_P, s7_Q, s8_T1, s8_M, s9_T1, s10_T1 : unsigned(11 downto 0) := (others => '0');
    signal s6_Qx, s8_D, s9_e : signed(12 downto 0) := (others => '0');
    signal s9_pA, s9_pB, s10_s : signed(16 downto 0) := (others => '0');
    signal s10_es : signed(14 downto 0) := (others => '0');
    signal s11_d  : unsigned(10 downto 0) := (others => '0');
    signal s12_pl : unsigned(16 downto 0) := (others => '0');
    signal s12_ph : unsigned(15 downto 0) := (others => '0');
    signal s13_pr : unsigned(21 downto 0) := (others => '0');
    signal s14_pp, s15_pp, s16_phi : unsigned(15 downto 0) := (others => '0');
    signal s7_aj, s8_s, s9_th, s12_mag : unsigned(11 downto 0) := (others => '0');
    signal s_neg : std_logic_vector(10 to 15) := (others => '0');
    signal s14_lo, s14_hi : unsigned(15 downto 0) := (others => '0');
    signal s15_fw : unsigned(14 downto 0) := (others => '0');
    type t_cg is array (11 to 25) of unsigned(4 downto 0);
    signal s_cg : t_cg := (others => (others => '0'));

    -- [C] Depth table: (F * i) >> 2 for i = 0 .. 63, refilled every field by an accumulator; the pixel path
    -- looks up both halves of the wave magnitude (two copies: one read port each)
    type t_ft is array (0 to 255) of unsigned(15 downto 0);
    signal fl_ram, fh_ram : t_ft := (others => (others => '0'));
    attribute no_rw_check of fl_ram : signal is true;
    attribute no_rw_check of fh_ram : signal is true;
    signal fl_q, fh_q, ft_wd : unsigned(15 downto 0) := (others => '0');
    signal ft_wa, ft_i : unsigned(5 downto 0) := (others => '0');
    signal ft_acc : unsigned(17 downto 0) := (others => '0');
    signal ft_we, ft_run : std_logic := '0';

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    -- per-cell luma hold: 2 row parities x 128 cell columns
    signal lm_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of lm_ram : signal is true;
    signal lm_q, lm_wd, lm_wa : unsigned(7 downto 0) := (others => '0');
    signal lm_we : std_logic := '0';
    signal s_ld7 : unsigned(7 downto 0) := (others => '0');
    -- K6 Camera: the held luma x the glided gain, three stages of gated sums (8 x 10 bits, stages 8 .. 10)
    -- -> 10 bits (1.0 = 256), then delayed to stage 13
    signal m8_p0, m8_p1, m8_p2, m8_p3 : unsigned(11 downto 0) := (others => '0');
    signal m9_a, m9_b : unsigned(13 downto 0) := (others => '0');
    type t_ld is array (10 to 13) of unsigned(9 downto 0);
    signal s_ld : t_ld := (others => (others => '0'));

    -- [E] coverage and blend
    signal s17_af : unsigned(11 downto 0) := (others => '0');
    signal s18_d  : signed(12 downto 0) := (others => '0');
    signal s19_e  : signed(14 downto 0) := (others => '0');
    type t_dv13 is array (0 to 4) of unsigned(12 downto 0);
    signal dv_r : t_dv13 := (others => (others => '0'));
    type t_dv4 is array (0 to 4) of unsigned(3 downto 0);
    signal dv_q : t_dv4 := (others => (others => '0'));
    signal dv_z, dv_f : std_logic_vector(0 to 4) := (others => '0');
    signal s_idx : std_logic_vector(17 to 25) := (others => '0');
    signal s25_cov, s26_cov : unsigned(4 downto 0) := (others => '0');
    signal s26_by, s26_bu, s26_bv, s27_by, s27_bu, s27_bv, s28_by, s28_bu, s28_bv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s26_dy, s26_du, s26_dv : signed(10 downto 0) := (others => '0');
    signal s27_ay, s27_au, s27_av, s27_py, s27_pu, s27_pv : signed(15 downto 0) := (others => '0');
    signal s28_y, s28_u, s28_v : signed(15 downto 0) := (others => '0');
    signal s29_y, s29_u, s29_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cu, s_cv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_yd : t_u10(1 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv2, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4100, 13);
    signal s_hist : std_logic_vector(7 downto 0) := (others => '0');
    signal s30_ya : unsigned(11 downto 0) := (others => '0');
    signal s30_yb : unsigned(9 downto 0) := (others => '0');
    signal s31_y, s32_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s31_u, s31_v, s32_u, s32_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s33_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s33_u, s33_v : unsigned(9 downto 0) := to_unsigned(512, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_HB_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_HB_SIN(to_integer(sn_a));
        end if;
    end process p_rom;

    -- Depth tables: written in the vblank zone only, read by the pixel path (stage 12 address)
    p_flram : process(clk)
    begin
        if rising_edge(clk) then
            fl_q <= fl_ram(to_integer(s12_mag(5 downto 0)));
            if ft_we = '1' then fl_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_flram;
    p_fhram : process(clk)
    begin
        if rising_edge(clk) then
            fh_q <= fh_ram(to_integer(s12_mag(11 downto 6)));
            if ft_we = '1' then fh_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_fhram;

    -- box delay: read 32 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    -- per-cell luma hold: written on each cell row's sample line (before the row's cells appear), read by
    -- the pixel path (stage 5 address)
    p_lmram : process(clk)
    begin
        if rising_edge(clk) then
            lm_q <= lm_ram(to_integer(s5_par & s5_col));
            if lm_we = '1' then lm_ram(to_integer(lm_wa)) <= lm_wd; end if;
        end if;
    end process p_lmram;

    -- [A] the beam's pre-pixel by video mode
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);

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
        variable v_t  : unsigned(9 downto 0);
        variable v_r  : unsigned(27 downto 0);
        variable v_pi : unsigned(3 downto 0);
        variable v_n7 : signed(7 downto 0);
        variable v_kk : unsigned(20 downto 0);
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
                s_rad   <= registers_in(6)(1);
                s_lines <= registers_in(6)(2);
                s_goff  <= registers_in(6)(3);
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
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(C_KFIX_5);
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at 50 + 24 k).  The slot number is decoded at 30 + 24 k and the operands are
            -- pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g1, 40);    z_fb <= to_unsigned(769, 16);                -- octaves above a 64-px cell
                    when 1  => z_fa <= resize(c_invp, 40);  z_fb <= to_unsigned(37837, 16);              -- rows per h (x 2 / sqrt 3)
                    when 2  => z_fa <= resize(c_invpy, 40); z_fb <= resize(s_ys, 16);                    -- rows per line
                    when 3  => z_fa <= resize(c_invpy, 40); z_fb <= to_unsigned(1080, 16);               -- rows above the centre
                    when 4  => z_fa <= resize(c_invp, 40);  z_fb <= resize(w_dxm, 16);                   -- cells left of the centre
                    when 5  => z_fa <= resize(c_invp, 40);  z_fb <= resize(s_xs, 16);                    -- cells per pixel
                    when 6  => z_fa <= resize(s_g5(12 downto 3), 40); z_fb <= resize(s_g5(12 downto 3), 16);   -- spread^2
                    when 7  =>                                                                           -- Sh * -(2 c0 + 1)
                        z_fa <= resize(c_Sh, 40);
                        v_n7 := -(signed(c_US1(28 downto 22)) & '1');
                        z_fb <= resize(unsigned(v_n7), 16);
                    when 8  =>                                                                           -- Sh * -j0
                        z_fa <= resize(c_Sh, 40);
                        z_fb <= resize(unsigned(-c_W0(21 downto 16)), 16);
                    when 9  => z_fa <= resize(s_gp, 40);    z_fb <= resize(z_oct, 16) + 362;             -- ring-density octaves
                    when 10 => z_fa <= resize(c_ustep, 40); z_fb <= resize(c_K, 16);                     -- rings per pixel
                    when 11 => z_fa <= resize(s_g3, 40);    z_fb <= resize(s_g3, 16);                    -- depth^2
                    when 12 => z_fa <= resize(s_vm, 40);    z_fb <= to_unsigned(897, 16);                -- rate exponent
                    when others => z_fa <= resize(r_exf, 40); z_fb <= to_unsigned(9321, 16);             -- rate step
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
                    when 21 =>                        -- shape: hexagram (0, 16) .. hexagon (16, 0) .. triangle (0, 0)
                        v_k := s_g2(12 downto 3);
                        if v_k(9) = '0' then
                            v_t := shift_right(to_unsigned(512, 10) - v_k, 5);
                            c_be <= v_t(4 downto 0);
                        else
                            v_t := shift_right(v_k - 511, 5);
                            c_be <= (others => '0');
                        end if;
                        c_al <= to_unsigned(16, 5) - v_t(4 downto 0);
                    -- K6 Camera: knob with a +-4 deadband (ends snap to 0 / 1023), then ease 1/8 of the gap
                    when 24 => s_kr <= unsigned(registers_in(5));
                    when 25 =>
                        v_c := signed(resize(s_kr, 11)) - signed(resize(s_kt, 11));
                        if s_kr < 4 then s_kt <= (others => '0');
                        elsif s_kr > 1019 then s_kt <= (others => '1');
                        elsif v_c > 4 or v_c < -4 then s_kt <= s_kr; end if;
                    when 26 => s_kgd <= signed(resize(s_kt & "000", 14)) - signed(resize(s_kg, 14));
                    when 27 =>
                        if C_SIM_SNAP or (s_kgd < 8 and s_kgd > -8) then s_kg <= s_kt & "000";
                        else s_kg <= unsigned(signed(s_kg) + resize(shift_right(s_kgd, 3), 13)); end if;
                    when 22 =>
                        if s_goff = '1' then c_gth <= to_signed(1024, 13); else c_gth <= to_signed(1024 - C_GW, 13); end if;
                    -- pitch: cells per h = 2^17 * 2^-oct
                    when 50  => z_oct <= f_acc(23 downto 13);
                    when 51  =>
                        v_e := -signed(resize(z_oct, 12));
                        r_ex_a <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(11 downto 8);
                        if s_xs = 2 then c_gsh <= resize(z_oct(9 downto 8), 3) + 1; else c_gsh <= resize(z_oct(9 downto 8), 3); end if;
                    when 54  =>
                        case to_integer(z_sh) is
                            when 0      => c_invp <= r_exf & "00";
                            when -1     => c_invp <= resize(r_exf & "0", 18);
                            when -2     => c_invp <= resize(r_exf, 18);
                            when others => c_invp <= resize(r_exf(15 downto 1), 18);
                        end case;
                    when 74  => c_invpy <= f_acc(33 downto 15);
                    when 98  => c_wstep <= f_acc(27 downto 15);
                    when 122 => c_W0 <= -signed(f_acc(29 downto 8));
                    when 146 => c_US1 <= unsigned(-signed(f_acc(30 downto 2)));
                    when 170 => c_ustep <= f_acc(19 downto 2);
                    when 194 => c_Sh <= f_acc(19 downto 10);
                    when 218 => c_AH1 <= -signed(f_acc(17 downto 0));
                    when 219 =>
                        if c_US1(21) = '1' then c_AH0 <= c_AH1 + signed(resize(c_Sh, 18));
                        else c_AH0 <= c_AH1 - signed(resize(c_Sh, 18)); end if;
                    when 242 => c_AJ0 <= -signed(f_acc(17 downto 0));
                    when 266 => r_ex_a <= '0' & f_acc(20 downto 13); z_sh4 <= f_acc(23 downto 21);
                    when 269 =>
                        v_kk := shift_left(resize(r_exf, 21), to_integer(z_sh4));
                        c_K <= v_kk(19 downto 9);
                    when 290 => c_w <= f_acc(27 downto 15);
                    when 291 =>
                        c_wh <= shift_right(c_w, 1) + shift_right(c_w, 5);
                        if c_w(12 downto 10) /= "000" then c_dh <= to_unsigned(1024, 11);
                        elsif s_lines = '1' and c_w(9) = '0' then c_dh <= to_unsigned(512, 11);
                        elsif s_lines = '1' then c_dh <= resize(c_w(9 downto 0), 11);
                        else c_dh <= to_unsigned(1024, 11); end if;
                    when 314 => c_F <= '0' & f_acc(25 downto 15);
                    when 338 => r_ex_a <= '0' & f_acc(15 downto 8); z_sh4 <= f_acc(18 downto 16);
                    when 362 =>
                        v_r := resize(f_acc(29 downto 8), 28);
                        if s_vm = 0 then c_rstep <= (others => '0');
                        else c_rstep <= shift_left(v_r, to_integer(z_sh4)); end if;
                    when others => null;
                end case;
            end if;

            -- [E] palette (zc 512 .. 563): 12 words from the table, one every 4 clocks: ground, A, B, accent
            if s_zone = '1' and s_zc(10 downto 6) = "01000" and s_zc(1 downto 0) = "00" then
                v_pi := s_zc(5 downto 2);
                r_ex_a <= '1' & (resize(s_pal & "000", 8) + resize(s_pal & "00", 8) + resize(v_pi, 8));
                case to_integer(v_pi) is
                    when 1  => c_gy <= r_exf(9 downto 0);
                    when 2  => c_gu <= r_exf(9 downto 0);
                    when 3  => c_gv <= r_exf(9 downto 0);
                    when 4  => c_ay <= r_exf(9 downto 0);
                    when 5  => c_au <= r_exf(9 downto 0);
                    when 6  => c_av <= r_exf(9 downto 0);
                    when 7  => c_by <= r_exf(9 downto 0);
                    when 8  => c_bu <= r_exf(9 downto 0);
                    when 9  => c_bv <= r_exf(9 downto 0);
                    when 10 => c_cy <= r_exf(9 downto 0);
                    when 11 => c_cu <= r_exf(9 downto 0);
                    when 12 => c_cv <= r_exf(9 downto 0);
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
            s_st <= g_start & s_st(0 to 2);
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
    -- [C] WAVE ENGINE, line part: runs once the line's last pixel has passed
    -- stage 5 (and once in the zone for line 0) and sets the NEXT line's values:
    --   e_W   row coordinate (Q16): j = floor, fw = fraction (Q11)
    --   l_p   parity of row j;  l_qa = 3/4 fw, l_aqb = 3/4 (1 - fw)  (the vertical axis term of this row's and
    --         the next row's cells);  l_ta = 2 - 2 l_qa  (the pixel is in this row's cell while |lu| <= l_ta)
    --   l_jA, l_jB  Sh * j and Sh * (j + 1)  (their magnitudes in Radial)
    --   l_samp      this line samples the luma of the next row's cells (the first line with fw >= 1/4)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_fw : unsigned(10 downto 0);
        variable v_qa : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;     -- after the field constants
            e_trig <= s_gav_sr(13) and not s_gav_sr(12);

            if e_zk = '1' then
                e_W <= c_W0; l_aj <= c_AJ0;
                e_first <= '1'; e_gep <= '0'; e_kick <= '1';
                e_lc <= (others => '0');
            elsif e_trig = '1' then
                e_W <= e_W + signed(resize(c_wstep, 22));
                e_jl <= e_W(16);
                e_first <= '0'; e_gep <= e_W(15) or e_W(14); e_kick <= '0';
                e_lc <= (others => '0');
            elsif e_lc /= 7 then
                e_lc <= e_lc + 1;
            end if;

            v_fw := unsigned(e_W(15 downto 5));
            v_qa := v_fw - shift_right(v_fw, 2);
            case to_integer(e_lc) is
                when 0 =>
                    e_inc <= (e_W(16) xor e_jl) and not e_kick;
                when 1 =>
                    if e_inc = '1' then l_aj <= l_aj + signed(resize(c_Sh, 18)); end if;
                when 2 =>
                    l_ajn <= l_aj + signed(resize(c_Sh, 18));
                    l_qa <= v_qa;
                    l_p <= e_W(16);
                when 3 =>
                    if s_rad = '1' and l_aj(17) = '1' then l_jA <= -l_aj; else l_jA <= l_aj; end if;
                    if s_rad = '1' and l_ajn(17) = '1' then l_jB <= -l_ajn; else l_jB <= l_ajn; end if;
                    l_aqb <= to_unsigned(1536, 11) - l_qa;
                    l_ta <= to_signed(2048, 13) - signed(resize(l_qa & '0', 13));
                    l_samp <= (e_W(15) or e_W(14)) and not e_gep;
                    l_first <= e_first;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [C] WAVE ENGINE, beam: the cell coordinate (integer part = cell column,
    -- fraction = position across the cell; stage 2) and the wave index
    -- Sh * (2 column + parity) (stage 3: it steps when the coordinate wraps).
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_h  : unsigned(28 downto 0);
        variable v_cw : unsigned(22 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                v_h := (others => '0'); v_h(21) := not l_p;
                b_U <= c_US1 + v_h;
            else
                b_U <= b_U + resize(c_ustep, 29);
            end if;
            v_cw := ('0' & b_U(21 downto 0)) + resize(c_ustep, 23);
            s_wr <= v_cw(22);
            if s_st(0) = '1' then
                if l_p = '1' then b_AH <= c_AH1; else b_AH <= c_AH0; end if;
            elsif s_wr = '1' then
                b_AH <= b_AH + signed(resize(c_Sh & '0', 18));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [C] WAVE ENGINE, pixel path (stages 3 .. 16)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_alu : unsigned(9 downto 0);
        variable v_sg  : std_logic;
        variable v_hm  : unsigned(10 downto 0);
        variable v_g   : unsigned(4 downto 0);
        variable v_a, v_b : signed(16 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_c   : signed(14 downto 0);
        variable v_q   : unsigned(9 downto 0);
        variable v_fw  : unsigned(21 downto 0);
        variable v_src : unsigned(15 downto 0);
        variable v_m18 : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: cell column and the position across the cell (Q11)
            s3_c <= b_U(28 downto 22); s3_f <= b_U(21 downto 11);
            s3_hq <= s3_f(10);
            -- 4: |lu| (distance from this row's centre), which side, the cell test, the next row's candidates
            v_sg := not s3_f(10);
            if s3_f(10) = '1' then v_alu := s3_f(9 downto 0); else v_alu := not s3_f(9 downto 0); end if;
            s4_alu <= v_alu; s4_sgn <= v_sg;
            if signed(resize(v_alu, 13)) <= l_ta then s4_inA <= '1'; else s4_inA <= '0'; end if;
            s4_c <= s3_c; s4_ah <= b_AH;
            if v_sg = '0' then
                s4_ahb <= b_AH + signed(resize(c_Sh, 18));
            else
                s4_ahb <= b_AH - signed(resize(c_Sh, 18));
            end if;
            if l_p = '1' and v_sg = '0' then s4_colb <= s3_c + 1;
            elsif l_p = '0' and v_sg = '1' then s4_colb <= s3_c - 1;
            else s4_colb <= s3_c; end if;
            -- 5: the cell
            if s4_inA = '1' then
                s5_alu <= '0' & s4_alu; s5_sgn <= s4_sgn; s5_aq <= l_qa;
                s5_ah <= s4_ah; s5_aj <= l_jA; s5_par <= l_p; s5_col <= s4_c;
            else
                s5_alu <= to_unsigned(1024, 11) - resize(s4_alu, 11); s5_sgn <= not s4_sgn; s5_aq <= l_aqb;
                s5_ah <= s4_ahb; s5_aj <= l_jB; s5_par <= not l_p; s5_col <= s4_colb;
            end if;
            -- 6: P = aq + |lu| / 2 (the larger slanted axis), Qx = aq - |lu| / 2; |index| in Radial
            v_hm := shift_right(s5_alu + unsigned'("0000000000" & s5_sgn), 1);
            s6_P  <= resize(s5_aq, 12) + resize(v_hm, 12);
            s6_Qx <= signed(resize(s5_aq, 13)) - signed(resize(v_hm, 13));
            s6_alu <= s5_alu; s6_sgn <= s5_sgn;
            if s_rad = '1' and s5_ah(17) = '1' then s6_ah <= -s5_ah; else s6_ah <= s5_ah; end if;
            s6_aj <= s5_aj;
            -- 7: Q = max(|lu|, Qx); Radial: max(|AJ|, |AH|)
            if s6_Qx > signed(resize(s6_alu, 13)) then s7_Q <= unsigned(s6_Qx(11 downto 0)); else s7_Q <= resize(s6_alu, 12); end if;
            s7_P <= s6_P; s7_sgn <= s6_sgn;
            if s_rad = '1' and s6_aj > s6_ah then s7_m <= s6_aj; else s7_m <= s6_ah; end if;
            s7_aj <= unsigned(s6_aj(11 downto 0));
            if l_first = '1' then s_ld7 <= (others => '0'); else s_ld7 <= lm_q; end if;   -- line 0 is being sampled: no hold yet
            -- 8 .. 10: K6 Camera: held luma x the glided gain (k / 256); 11 .. 13: delay
            m8_p0 <= f_gu(s_kg(12 downto 3), s_ld7(0), 12) + shift_left(f_gu(s_kg(12 downto 3), s_ld7(1), 12), 1);
            m8_p1 <= f_gu(s_kg(12 downto 3), s_ld7(2), 12) + shift_left(f_gu(s_kg(12 downto 3), s_ld7(3), 12), 1);
            m8_p2 <= f_gu(s_kg(12 downto 3), s_ld7(4), 12) + shift_left(f_gu(s_kg(12 downto 3), s_ld7(5), 12), 1);
            m8_p3 <= f_gu(s_kg(12 downto 3), s_ld7(6), 12) + shift_left(f_gu(s_kg(12 downto 3), s_ld7(7), 12), 1);
            m9_a <= resize(m8_p0, 14) + shift_left(resize(m8_p1, 14), 2);
            m9_b <= resize(m8_p2, 14) + shift_left(resize(m8_p3, 14), 2);
            v_m18 := resize(m9_a, 18) + shift_left(resize(m9_b, 18), 4);
            s_ld(10) <= v_m18(17 downto 8);
            for i in 11 to 13 loop s_ld(i) <= s_ld(i - 1); end loop;
            -- 8: T1 (the triangle metric), D = T2 - T1, M = the hexagon metric; the wave index
            if s7_sgn = '1' then
                s8_T1 <= s7_P;
                s8_D <= signed(resize(s7_Q, 13)) - signed(resize(s7_P, 13));
            else
                s8_T1 <= s7_Q;
                s8_D <= signed(resize(s7_P, 13)) - signed(resize(s7_Q, 13));
            end if;
            if s7_P > s7_Q then s8_M <= s7_P; else s8_M <= s7_Q; end if;
            s8_s <= s7_aj + unsigned(s7_m(11 downto 0));
            -- 9: g * D as two sums of gated shifts; the grout distance; theta
            if s8_D(12) = '0' then v_g := c_al; else v_g := c_be; end if;
            if v_g(4) = '1' then
                s9_pA <= shift_left(resize(s8_D, 17), 4); s9_pB <= (others => '0');
            else
                if v_g(3) = '1' then v_a := shift_left(resize(s8_D, 17), 3); else v_a := (others => '0'); end if;
                if v_g(2) = '1' then s9_pA <= v_a + shift_left(resize(s8_D, 17), 2); else s9_pA <= v_a; end if;
                if v_g(1) = '1' then v_b := shift_left(resize(s8_D, 17), 1); else v_b := (others => '0'); end if;
                if v_g(0) = '1' then s9_pB <= v_b + resize(s8_D, 17); else s9_pB <= v_b; end if;
            end if;
            s9_T1 <= s8_T1;
            s9_e <= signed(resize(s8_M, 13)) - c_gth;
            s9_th <= f_addsub(c_ph(31 downto 20), s8_s, s_rad);
            -- 10
            s10_s <= s9_pA + s9_pB;
            s10_T1 <= s9_T1;
            case to_integer(c_gsh) is
                when 0      => s10_es <= resize(shift_right(s9_e, 2), 15);
                when 1      => s10_es <= resize(shift_right(s9_e, 1), 15);
                when 2      => s10_es <= resize(s9_e, 15);
                when 3      => s10_es <= shift_left(resize(s9_e, 15), 1);
                when others => s10_es <= shift_left(resize(s9_e, 15), 2);
            end case;
            if s9_th(10) = '1' then v_q := not s9_th(9 downto 0); else v_q := s9_th(9 downto 0); end if;
            if s_snap = '0' then sn_a <= v_q;
            elsif v_q(9 downto 8) /= "00" then sn_a <= (others => '1');
            else sn_a <= v_q(7 downto 0) & "00"; end if;
            s_neg(10) <= s9_th(11);
            for i in 11 to 15 loop s_neg(i) <= s_neg(i - 1); end loop;
            -- 11: d; the grout coverage 0 .. 16
            v_d := signed(resize(s10_T1, 13)) + resize(shift_right(s10_s, 4), 13);
            s11_d <= unsigned(v_d(10 downto 0));
            v_c := s10_es + 8;
            if v_c(14) = '1' then s_cg(11) <= (others => '0');
            elsif v_c > 16 then s_cg(11) <= to_unsigned(16, 5);
            else s_cg(11) <= unsigned(v_c(4 downto 0)); end if;
            for i in 12 to 25 loop s_cg(i) <= s_cg(i - 1); end loop;
            -- 12: d * K in two halves; the wave magnitude (the sine ROM's own register is stage 11)
            s12_pl <= s11_d * c_K(5 downto 0);
            s12_ph <= s11_d * c_K(10 downto 6);
            s12_mag <= sn_q(13 downto 2);
            -- 13: rings (Q16); Depth * (low half), Depth * (high half) from the tables
            s13_pr <= resize(s12_pl, 22) + (s12_ph & "000000");
            -- 14: minus the held luma
            -- (the held source is 10 bits since the K6 camera gain: 256 = the shipped 1x; PHI wraps mod 16 rings)
            if s_src = '1' then v_src := s_ld(13) & "000000"; else v_src := '0' & s_ld(13) & "00000"; end if;
            s14_pp <= s13_pr(19 downto 4) - v_src;
            s14_lo <= fl_q; s14_hi <= fh_q;
            -- 15: the displacement (rings, Q12)
            s15_pp <= s14_pp;
            v_fw := resize(s14_lo, 22) + (s14_hi & "000000");
            s15_fw <= v_fw(21 downto 7);
            -- 16: PHI (Q4.12, mod 16 rings)
            s16_phi <= f_addsub(s15_pp, resize(s15_fw, 16), not s_neg(15));

            -- Depth table fill (zone, zc 400 .. 464): T(i) = (F * i) >> 2 by accumulation
            ft_we <= '0';
            if s_zone = '1' and s_zc = 400 then
                ft_run <= '1'; ft_i <= (others => '0'); ft_acc <= (others => '0');
            elsif ft_run = '1' then
                ft_we <= '1'; ft_wa <= ft_i; ft_wd <= ft_acc(17 downto 2);
                ft_acc <= ft_acc + c_F; ft_i <= ft_i + 1;
                if ft_i = 63 then ft_run <= '0'; end if;
            end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 32-px box, edge-replicated
    -- and reloaded at every line start -> one sample per cell, written into the
    -- hold RAM where the beam crosses the cell's centre column on the row's
    -- sample line (the top row's own cells on the first line).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
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

            -- the hold: the next row's cell centres are where this row's coordinate wraps; the first line
            -- also samples its own row at the half crossings
            lm_wd <= s_bsum(12 downto 5);
            if l_first = '1' and s3_f(10) = '1' and s3_hq = '0' then
                lm_we <= s_gav_sr(1);
                lm_wa <= l_p & s3_c;
            else
                lm_we <= s_wr and l_samp and s_gav_sr(1);
                if l_p = '1' then lm_wa <= '0' & (s3_c + 1); else lm_wa <= '1' & s3_c; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] PALETTE + AA BLEND (stages 17 .. 29)
    --   cov  = clamp(16 (dh - |frac(PHI) - 1/2| + w / 2) / w + 1/2)       (w, dh constant over the field)
    --   ring colour = A / B by the ring's parity, over the ground; where the grout coverage is not zero the
    --   pixel blends from the ring (taken hard) to the grout colour instead
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_t  : unsigned(13 downto 0);
        variable v_g  : signed(15 downto 0);
        variable v_win, v_ing, v_hard : std_logic;
        variable v_ry, v_ru, v_rv, v_ty, v_tu, v_tv, v_by, v_bu, v_bv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 17
            if s16_phi(11) = '1' then s17_af <= '0' & s16_phi(10 downto 0);
            else s17_af <= to_unsigned(2048, 12) - resize(s16_phi(10 downto 0), 12); end if;
            s_idx(17) <= s16_phi(12);
            for i in 18 to 25 loop s_idx(i) <= s_idx(i - 1); end loop;
            -- 18
            s18_d <= signed(resize(c_dh, 13)) - signed(resize(s17_af, 13));
            -- 19
            s19_e <= resize(s18_d, 15) + signed(resize(c_wh, 15));
            -- 20: range tests; 21 .. 24: four quotient bits
            dv_z(0) <= s19_e(14);
            if s19_e >= signed(resize(c_w, 15)) then dv_f(0) <= '1'; else dv_f(0) <= '0'; end if;
            dv_r(0) <= unsigned(s19_e(12 downto 0));
            for i in 1 to 4 loop
                v_t := dv_r(i - 1) & '0';
                if v_t >= resize(c_w, 14) then
                    dv_r(i) <= resize(v_t - resize(c_w, 14), 13);
                    dv_q(i) <= dv_q(i - 1)(2 downto 0) & '1';
                else
                    dv_r(i) <= v_t(12 downto 0);
                    dv_q(i) <= dv_q(i - 1)(2 downto 0) & '0';
                end if;
                dv_z(i) <= dv_z(i - 1); dv_f(i) <= dv_f(i - 1);
            end loop;
            -- 25: coverage 0 .. 16
            if dv_z(4) = '1' then s25_cov <= (others => '0');
            elsif dv_f(4) = '1' then s25_cov <= to_unsigned(16, 5);
            else s25_cov <= '0' & dv_q(4); end if;
            -- 26: base colour, target - base, blend weight
            if s_idx(25) = '1' then v_ry := c_by; v_ru := c_bu; v_rv := c_bv;
            else v_ry := c_ay; v_ru := c_au; v_rv := c_av; end if;
            if s_cg(25) /= 0 then v_ing := '1'; else v_ing := '0'; end if;
            v_hard := v_ing and (s25_cov(4) or s25_cov(3));
            if v_hard = '1' then v_by := v_ry; v_bu := v_ru; v_bv := v_rv;
            else v_by := c_gy; v_bu := c_gu; v_bv := c_gv; end if;
            if v_ing = '0' then v_ty := v_ry; v_tu := v_ru; v_tv := v_rv;
            elsif s_goff = '1' then v_ty := c_gy; v_tu := c_gu; v_tv := c_gv;
            else v_ty := c_cy; v_tu := c_cu; v_tv := c_cv; end if;
            s26_by <= v_by; s26_bu <= v_bu; s26_bv <= v_bv;
            s26_dy <= signed(resize(v_ty, 11)) - signed(resize(v_by, 11));
            s26_du <= signed(resize(v_tu, 11)) - signed(resize(v_bu, 11));
            s26_dv <= signed(resize(v_tv, 11)) - signed(resize(v_bv, 11));
            if v_ing = '1' then s26_cov <= s_cg(25); else s26_cov <= s25_cov; end if;
            -- 27: difference * weight as two sums of gated shifts
            s27_by <= s26_by; s27_bu <= s26_bu; s27_bv <= s26_bv;
            if s26_cov(4) = '1' then
                s27_ay <= shift_left(resize(s26_dy, 16), 4); s27_py <= (others => '0');
                s27_au <= shift_left(resize(s26_du, 16), 4); s27_pu <= (others => '0');
                s27_av <= shift_left(resize(s26_dv, 16), 4); s27_pv <= (others => '0');
            else
                if s26_cov(3) = '1' then v_g := shift_left(resize(s26_dy, 16), 3); else v_g := (others => '0'); end if;
                if s26_cov(2) = '1' then s27_ay <= v_g + shift_left(resize(s26_dy, 16), 2); else s27_ay <= v_g; end if;
                if s26_cov(1) = '1' then v_g := shift_left(resize(s26_dy, 16), 1); else v_g := (others => '0'); end if;
                if s26_cov(0) = '1' then s27_py <= v_g + resize(s26_dy, 16); else s27_py <= v_g; end if;
                if s26_cov(3) = '1' then v_g := shift_left(resize(s26_du, 16), 3); else v_g := (others => '0'); end if;
                if s26_cov(2) = '1' then s27_au <= v_g + shift_left(resize(s26_du, 16), 2); else s27_au <= v_g; end if;
                if s26_cov(1) = '1' then v_g := shift_left(resize(s26_du, 16), 1); else v_g := (others => '0'); end if;
                if s26_cov(0) = '1' then s27_pu <= v_g + resize(s26_du, 16); else s27_pu <= v_g; end if;
                if s26_cov(3) = '1' then v_g := shift_left(resize(s26_dv, 16), 3); else v_g := (others => '0'); end if;
                if s26_cov(2) = '1' then s27_av <= v_g + shift_left(resize(s26_dv, 16), 2); else s27_av <= v_g; end if;
                if s26_cov(1) = '1' then v_g := shift_left(resize(s26_dv, 16), 1); else v_g := (others => '0'); end if;
                if s26_cov(0) = '1' then s27_pv <= v_g + resize(s26_dv, 16); else s27_pv <= v_g; end if;
            end if;
            -- 28
            s28_by <= s27_by; s28_bu <= s27_bu; s28_bv <= s27_bv;
            s28_y <= s27_ay + s27_py;
            s28_u <= s27_au + s27_pu;
            s28_v <= s27_av + s27_pv;
            -- 29: base + (difference * weight + 8) / 16; the ground colour outside pixels 0 .. W + 7
            v_win := s_gav_sr(26) or s_gav_sr(34);
            if v_win = '1' then
                v_g := signed(resize(s28_by & "1000", 16)) + s28_y; s29_y <= unsigned(v_g(13 downto 4));
                v_g := signed(resize(s28_bu & "1000", 16)) + s28_u; s29_u <= unsigned(v_g(13 downto 4));
                v_g := signed(resize(s28_bv & "1000", 16)) + s28_v; s29_v <= unsigned(v_g(13 downto 4));
            else
                s29_y <= c_gy; s29_u <= c_gu; s29_v <= c_gv;
            end if;
            s_hist <= s_hist(6 downto 0) & v_win;
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 30 .. 33): chroma [1 2 2 2 2 2 2 2 1]/16 as
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
            -- 30: running 8-sums (+ 4: rounds the 9-tap sum); they snap to 8 x ground once 8 samples outside
            -- the picture are in, so a glitched sample can't offset the colours for good
            s_cu(0) <= s29_u; s_cv(0) <= s29_v;
            for i in 1 to 7 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            if s_hist = "00000000" then
                s_bu <= (c_gu & "000") + 4; s_bv2 <= (c_gv & "000") + 4;
            else
                s_bu  <= s_bu + s29_u - s_cu(7);
                s_bv2 <= s_bv2 + s29_v - s_cv(7);
            end if;
            s_bud <= s_bu; s_bvd <= s_bv2;
            -- luma taps y(n - 3), y(n - 4), y(n - 5)
            s_yd(1) <= s29_y;
            for i in 2 to 5 loop s_yd(i) <= s_yd(i - 1); end loop;
            s30_ya <= resize(s_yd(3), 12) + resize(s_yd(5), 12) + 2;
            s30_yb <= s_yd(4);
            -- 31
            v_sc := resize(s_bu, 14) + s_bud;  s31_u <= v_sc(13 downto 4);
            v_sc := resize(s_bv2, 14) + s_bvd; s31_v <= v_sc(13 downto 4);
            v_y := s30_ya + shift_left(resize(s30_yb, 12), 1);
            s31_y <= v_y(11 downto 2);
            -- 32: luma 13/16 toward black
            if s31_y > 64 then v_dl := s31_y - 64; else v_dl := (others => '0'); end if;
            s32_y <= s31_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s32_u <= s31_u; s32_v <= s31_v;
            -- 33: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s33_y <= s32_y; s33_u <= s32_u; s33_v <= s32_v;
            else
                s33_y <= to_unsigned(64, 10); s33_u <= to_unsigned(512, 10); s33_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s33_y);
    data_out.u       <= std_logic_vector(s33_u);
    data_out.v       <= std_logic_vector(s33_v);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture hexbloom;
