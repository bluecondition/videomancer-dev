-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- trellis.vhd  (v0.1)
--
-- TRELLIS: a three-oscillator colour synth.  Three independent 2-D wave
-- fields are drawn straight into the red, green and blue guns and summed
-- additively, the way a patched analog video synth would do it, so every
-- overlap makes a new colour (R+B pink, R+G yellow, G+B cyan, all = white).
--
--   RED    sin(wy*y + m*sin(wx*x))          horizontal bands, FM-bent into
--                                            arcs by a column-rate modulator
--   GREEN  cos(wx*x) + ramp(y)               vertical columns whose width
--                                            tapers down the screen
--   BLUE   sin(wy*y + m/2*sin(wx*x)) + a*cos(wx*x)
--                                            a lens lattice: the level set
--                                            of two summed sinusoids pinches
--                                            into eye-shaped holes, and a
--                                            fine vertical grating fills the
--                                            holes (stripes under glass)
--
-- Each field is thresholded (Width) with a soft or hard knee, so the picture
-- is built from shapes, not gradients.  Warp (P12) drives all three
-- deformations at once: at 0 the bands, columns and lattice lines are
-- straight; at 100% the bands fold into deep S-curves, the lattice closes
-- into a diamond grid, and the columns fan from nothing at the top to the
-- full screen at the bottom.
--
-- ENGINE (no framebuffer, one pixel per clock)
--   Four 20-bit phase accumulators step per pixel (modulator, column,
--   lattice, grating) from a per-line start phase; two step per active line
--   (band and lattice rows) plus a taper ramp.  All increments are
--   cycles / measured raster width or height, from a 28-bit serial divider in
--   the vblank sequencer, so a pattern tiles the screen identically at SD
--   and HD.  Six 256 x 10 sine ROMs (one EBR each) supply the waves; three
--   256 x 32 colour ROMs turn each gun into its (Y, Cb, Cr) contribution so
--   RGB->YUV is three adds; a hue rotation of (Cb, Cr) follows.  Output
--   conditioning: [1 2 1]/4 on Y/U/V and luma capped at 764.
--
--   Clean avid: the program generates its own avid (cubist's HW-confirmed
--   recipe: start 2-3 clocks after the source rise at the voted hsync
--   parity, end after the field-latched width) and blanks outside it.
--
-- CONTROLS
--   K1  Bands     band rows per screen height, 1 .. 8
--   K2  Columns   columns per screen width, 1 .. 8 (also the bend rate)
--   K3  Stripes   grating lines per screen width, 8 .. 72
--   K4  Width     shape threshold: hairlines .. fat (lattice runs broader)
--   K5  Speed     motion, bipolar; centre = still.  Bands scroll, bends
--                 travel, the lattice counter-scrolls, stripes slide.
--   K6  Hue       rotates every colour around the wheel
--   S7  Mirror    Off / On: the right half reflects the left
--   S8  Negative  Off / On: every gun inverted
--   S9  Grating   Holes (in the blue lattice only) / Screen (a line screen
--                 cut into every shape, width-modulated by the field)
--   S10 Edges     Soft / Hard
--   S11 Stripes   Vertical / Diagonal
--   P12 WARP      bend + lens pinch + column taper, straight -> folded
--
-- Latency 21 clocks, one for all modes.  EBR 13.  No DSP.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture trellis of program_top is
    -- Synthesis program: processing_delay_clks is declared 0 in the TOML.
    -- The true data_in -> data_out sync latency is C_LAT clocks.
    constant C_PROCESSING_DELAY_CLKS : natural := 0;
    constant C_LAT : integer := 21;

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- ---------------------------------------------------------------
    -- sine ROM: 256 x 16, low 10 bits = round(511*sin), sign-extended.
    -- Every registered read infers its own 256x16 EBR.
    -- ---------------------------------------------------------------
    type t_srom is array (0 to 255) of std_logic_vector(15 downto 0);
    function f_srom return t_srom is
        variable r : t_srom;
        variable s : integer;
    begin
        for i in 0 to 255 loop
            s := integer(round(511.0 * sin(real(i) * MATH_2_PI / 256.0)));
            r(i) := std_logic_vector(to_signed(s, 16));
        end loop;
        return r;
    end function;
    constant C_SIN : t_srom := f_srom;

    -- ---------------------------------------------------------------
    -- colour ROMs: 256 x 32 per gun, (29:20) = Y contribution (unsigned),
    -- (19:10) = Cb contribution, (9:0) = Cr contribution (signed), BT.601
    -- at 0.8 gain so white lands at Y 764 (hardware luma cap).
    -- ---------------------------------------------------------------
    type t_crom is array (0 to 255) of std_logic_vector(31 downto 0);
    function f_crom(ky, ku, kv : integer) return t_crom is
        variable r       : t_crom;
        variable y, u, v : integer;
    begin
        for i in 0 to 255 loop
            y := (ky * i + 128) / 256;
            u := (ku * i) / 256;
            v := (kv * i) / 256;
            r(i) := "00" & std_logic_vector(to_unsigned(y, 10))
                         & std_logic_vector(to_signed(u, 10))
                         & std_logic_vector(to_signed(v, 10));
        end loop;
        return r;
    end function;
    constant C_CR : t_crom := f_crom(210, -121,  360);
    constant C_CG : t_crom := f_crom(413, -238, -301);
    constant C_CB : t_crom := f_crom( 80,  360,  -58);

    -- knee: field units (+-511) -> 8-bit gun.  Soft spans 128 units, hard 16.
    function f_knee(x : signed(12 downto 0); hard : std_logic) return unsigned is
        variable r : unsigned(7 downto 0);
    begin
        if x < 0 then
            r := (others => '0');
        elsif hard = '1' then
            if x >= 16 then r := (others => '1');
            else r := unsigned(x(3 downto 0)) & "0000"; end if;
        else
            if x >= 128 then r := (others => '1');
            else r := unsigned(x(6 downto 0)) & '0'; end if;
        end if;
        return r;
    end function;

    -- one-pole knob glide, Q10.4 accumulator
    function f_glide(acc : unsigned(13 downto 0); k : unsigned(9 downto 0)) return unsigned is
        variable d : signed(15 downto 0);
        variable n : signed(15 downto 0);
    begin
        d := signed("00" & k & "0000") - signed("00" & acc);
        n := signed("00" & acc) + shift_right(d, 3);
        return unsigned(n(13 downto 0));
    end function;

    -- low 20 bits of a signed velocity: modular phase step
    function f_lo20(v : signed(20 downto 0)) return unsigned is
    begin
        return unsigned(v(19 downto 0));
    end function;

    signal s_timing : t_video_timing_port;

    -- ==== raster measurement ====
    signal s_hcnt   : unsigned(11 downto 0) := (others => '0');
    signal s_meas_h : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vcnt   : unsigned(11 downto 0) := (others => '0');
    signal s_meas_v : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_saw, s_vs_go : std_logic := '0';

    -- ==== generated avid ====
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc    : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr   : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt   : unsigned(11 downto 0) := (others => '0');
    signal g_wm1   : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_halfm1 : unsigned(11 downto 0) := to_unsigned(959, 12);
    signal g_avd, g_avf : std_logic := '0';
    signal s_mir   : std_logic := '0';

    -- ==== controls ====
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6, s_k12 : unsigned(9 downto 0) := C_MID;
    signal s_sw : std_logic_vector(4 downto 0) := (others => '0');
    signal s_a1  : unsigned(13 downto 0) := to_unsigned(290 * 16, 14);
    signal s_a2  : unsigned(13 downto 0) := to_unsigned(146 * 16, 14);
    signal s_a3  : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);
    signal s_a4  : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);
    signal s_a5  : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);
    signal s_a6  : unsigned(13 downto 0) := to_unsigned(0, 14);
    signal s_a12 : unsigned(13 downto 0) := to_unsigned(400 * 16, 14);

    -- ==== vblank sequencer ====
    signal q_st   : integer range 0 to 31 := 0;
    signal dv_go, dv_run : std_logic := '0';
    signal dv_cnt : unsigned(4 downto 0) := (others => '0');
    signal dv_n   : unsigned(27 downto 0) := (others => '0');
    signal dv_d   : unsigned(12 downto 0) := to_unsigned(1920, 13);
    signal dv_rem : unsigned(12 downto 0) := (others => '0');
    signal dv_q   : unsigned(27 downto 0) := (others => '0');
    signal mu_go, mu_run : std_logic := '0';
    signal mu_cnt : unsigned(3 downto 0) := (others => '0');
    signal mu_a   : unsigned(19 downto 0) := (others => '0');
    signal mu_b   : unsigned(9 downto 0) := (others => '0');
    signal mu_acc : unsigned(19 downto 0) := (others => '0');

    -- ==== per-frame constants ====
    signal f_c1, f_c2, f_c3 : unsigned(11 downto 0) := to_unsigned(47, 12);
    signal f_tA   : unsigned(9 downto 0) := (others => '0');
    signal f_t    : signed(10 downto 0) := to_signed(322, 11);
    signal f_tb   : signed(10 downto 0) := to_signed(66, 11);
    signal f_g3   : unsigned(11 downto 0) := to_unsigned(1200, 12);
    signal f_dep  : unsigned(11 downto 0) := to_unsigned(312, 12);
    signal f_pin  : unsigned(8 downto 0) := to_unsigned(200, 9);
    signal f_spd  : signed(10 downto 0) := (others => '0');
    signal f_vel  : signed(20 downto 0) := (others => '0');
    signal f_hue  : unsigned(7 downto 0) := (others => '0');
    signal a_h    : unsigned(7 downto 0) := (others => '0');
    signal f_cos  : signed(7 downto 0) := to_signed(127, 8);
    signal f_sin  : signed(7 downto 0) := (others => '0');
    signal f_mirror, f_neg, f_screen, f_hard, f_diag : std_logic := '0';
    signal f_incx, f_incxn, f_incs, f_incsn, f_incy, f_inct : unsigned(19 downto 0) := (others => '0');
    -- motion phases (advance once per field)
    signal ph_r, ph_by, ph_m, ph_g, ph_bx, ph_s : unsigned(19 downto 0) := (others => '0');
    -- per-line accumulators (advance at the end of each active line)
    signal y_r, y_b, y_s : unsigned(19 downto 0) := (others => '0');
    signal y_ramp : signed(21 downto 0) := (others => '0');
    -- line constants (latched at hsync)
    signal ly_r, ly_b : unsigned(19 downto 0) := (others => '0');
    signal l_ramp : signed(11 downto 0) := (others => '0');

    -- ==== pixel pipeline ====
    -- stage 0: x-phase accumulators
    signal p_m, p_g, p_bx, p_s : unsigned(19 downto 0) := (others => '0');
    -- stage 1: EBR outputs
    signal q_m, q_g, q_bx, q_s, q_r, q_b, q_h : std_logic_vector(15 downto 0) := (others => '0');
    signal q_cr, q_cg, q_cb : std_logic_vector(31 downto 0) := (others => '0');
    -- stage 2
    signal s2_m, s2_g, s2_bx, s2_s : signed(9 downto 0) := (others => '0');
    -- stage 3
    signal s3_pmh, s3_pml : signed(16 downto 0) := (others => '0');
    signal s3_pnh : signed(15 downto 0) := (others => '0');
    signal s3_pnl : signed(14 downto 0) := (others => '0');
    signal s3_g, s3_s : signed(9 downto 0) := (others => '0');
    -- stage 4
    signal s4_pm : signed(22 downto 0) := (others => '0');
    signal s4_pn : signed(19 downto 0) := (others => '0');
    signal s4_g, s4_s : signed(9 downto 0) := (others => '0');
    -- stage 5
    signal s5_ar, s5_ab : unsigned(7 downto 0) := (others => '0');
    signal s5_pn, s5_gv : signed(11 downto 0) := (others => '0');
    signal s5_s : signed(9 downto 0) := (others => '0');
    -- stage 6
    signal s6_pn, s6_gv : signed(11 downto 0) := (others => '0');
    signal s6_s : signed(9 downto 0) := (others => '0');
    -- stage 7
    signal s7_r, s7_b : signed(9 downto 0) := (others => '0');
    signal s7_pn, s7_gv : signed(11 downto 0) := (others => '0');
    signal s7_s : signed(9 downto 0) := (others => '0');
    -- stage 8
    signal s8_rv, s8_gv, s8_bv, s8_ero : signed(12 downto 0) := (others => '0');
    signal s8_s : signed(9 downto 0) := (others => '0');
    -- stage 9
    signal s9_r, s9_g, s9_bp, s9_bn : signed(12 downto 0) := (others => '0');
    signal s9_s : signed(9 downto 0) := (others => '0');
    -- stage 10
    signal s10_r8, s10_g8, s10_bp8, s10_bn8, s10_st8 : unsigned(7 downto 0) := (others => '0');
    -- stage 11
    signal s11_hole : unsigned(15 downto 0) := (others => '0');
    signal s11_r8, s11_g8, s11_bp8 : unsigned(7 downto 0) := (others => '0');
    -- stage 12
    signal s12_r8, s12_g8, s12_b8 : unsigned(7 downto 0) := (others => '0');
    -- stage 14
    signal s14_cr, s14_cg, s14_cb : std_logic_vector(31 downto 0) := (others => '0');
    -- stage 15
    signal s15_y : unsigned(9 downto 0) := C_BLK;
    signal s15_u, s15_v : signed(10 downto 0) := (others => '0');
    -- stage 16
    signal s16_uch, s16_ush, s16_vch, s16_vsh : signed(14 downto 0) := (others => '0');
    signal s16_ucl, s16_usl, s16_vcl, s16_vsl : signed(15 downto 0) := (others => '0');
    signal s16_y : unsigned(9 downto 0) := C_BLK;
    -- stage 17
    signal s17_uc, s17_us, s17_vc, s17_vs : signed(18 downto 0) := (others => '0');
    signal s17_y : unsigned(9 downto 0) := C_BLK;
    -- stage 18
    signal s18_u, s18_v : signed(10 downto 0) := (others => '0');
    signal s18_y : unsigned(9 downto 0) := C_BLK;
    -- stages 19..21: smoothing window
    signal s19_y, s19_cb, s19_cr : unsigned(9 downto 0) := C_MID;
    signal s20_y, s20_cb, s20_cr : unsigned(9 downto 0) := C_MID;
    signal s21_y, s21_cb, s21_cr : unsigned(9 downto 0) := C_MID;

    -- ==== sync delay ====
    type t_bsr is array (1 to C_LAT - 1) of std_logic;
    signal d_hs, d_vs, d_fl, d_gv : t_bsr := (others => '0');

    signal o_y, o_cb, o_cr : unsigned(9 downto 0) := C_MID;
    signal o_av, o_hs, o_vs, o_fl : std_logic := '0';

begin

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- Raster measurement + generated avid (aurora / cubist recipe)
    -- ====================================================================
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;    -- hsync_n falling
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;       -- avid rising
            g_avd <= g_avid;
            g_avf <= g_avd and not g_avid;             -- generated avid falling

            -- width: active pixels per line
            if g_hsr = '1' then
                if s_hcnt > 16 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;
            -- height: active lines per field, latched on the guarded tick
            if s_vs_go = '1' then
                if s_vcnt > 8 then s_meas_v <= s_vcnt; end if;
                s_vcnt <= (others => '0');
            elsif g_avr = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;

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

            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            -- mirror: retrace the x phases from the centre column
            if g_hsr = '1' then
                s_mir <= '0';
            elsif g_cnt = s_halfm1 and g_avid = '1' and f_mirror = '1' then
                s_mir <= '1';
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Frame tick, knob glides, vblank sequencer, serial divider/multiplier,
    -- motion phases and the per-line accumulators
    -- ====================================================================
    p_frame : process(clk)
        variable v_r   : unsigned(12 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_g1, v_g2, v_g3, v_g4, v_g5, v_g6, v_g12 : unsigned(9 downto 0);
        variable v_m   : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- once-per-field pulse: the first vsync edge after active video
            s_vs_go <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_vs_go <= '1';
            end if;

            -- serial restoring divider, 28 bits
            dv_go <= '0';
            if dv_go = '1' then
                dv_cnt <= to_unsigned(28, 5);
                dv_rem <= (others => '0');
                dv_q   <= (others => '0');
                dv_run <= '1';
            elsif dv_run = '1' then
                v_r  := dv_rem(11 downto 0) & dv_n(27);
                dv_n <= dv_n(26 downto 0) & '0';
                if v_r >= dv_d then
                    dv_rem <= v_r - dv_d;
                    dv_q   <= dv_q(26 downto 0) & '1';
                else
                    dv_rem <= v_r;
                    dv_q   <= dv_q(26 downto 0) & '0';
                end if;
                dv_cnt <= dv_cnt - 1;
                if dv_cnt = 1 then dv_run <= '0'; end if;
            end if;

            -- serial shift-add multiplier, 10 x 10
            mu_go <= '0';
            if mu_go = '1' then
                mu_cnt <= to_unsigned(10, 4);
                mu_acc <= (others => '0');
                mu_run <= '1';
            elsif mu_run = '1' then
                if mu_b(0) = '1' then mu_acc <= mu_acc + mu_a; end if;
                mu_a <= mu_a(18 downto 0) & '0';
                mu_b <= '0' & mu_b(9 downto 1);
                mu_cnt <= mu_cnt - 1;
                if mu_cnt = 1 then mu_run <= '0'; end if;
            end if;

            v_g1  := s_a1(13 downto 4);
            v_g2  := s_a2(13 downto 4);
            v_g3  := s_a3(13 downto 4);
            v_g4  := s_a4(13 downto 4);
            v_g5  := s_a5(13 downto 4);
            v_g6  := s_a6(13 downto 4);
            v_g12 := s_a12(13 downto 4);

            case q_st is
                when 0 =>
                    if s_vs_go = '1' then q_st <= 1; end if;
                when 1 =>
                    s_k1  <= unsigned(registers_in(0));
                    s_k2  <= unsigned(registers_in(1));
                    s_k3  <= unsigned(registers_in(2));
                    s_k4  <= unsigned(registers_in(3));
                    s_k5  <= unsigned(registers_in(4));
                    s_k6  <= unsigned(registers_in(5));
                    s_k12 <= unsigned(registers_in(7));
                    s_sw  <= registers_in(6)(4 downto 0);
                    g_wm1    <= s_meas_h - 1;
                    s_halfm1 <= ('0' & s_meas_h(11 downto 1)) - 1;
                    q_st <= 2;
                when 2 =>
                    s_a1  <= f_glide(s_a1, s_k1);
                    s_a2  <= f_glide(s_a2, s_k2);
                    s_a3  <= f_glide(s_a3, s_k3);
                    s_a4  <= f_glide(s_a4, s_k4);
                    s_a5  <= f_glide(s_a5, s_k5);
                    s_a6  <= f_glide(s_a6, s_k6);
                    s_a12 <= f_glide(s_a12, s_k12);
                    f_mirror <= s_sw(0);
                    f_neg    <= s_sw(1);
                    f_screen <= s_sw(2);
                    f_hard   <= s_sw(3);
                    f_diag   <= s_sw(4);
                    q_st <= 3;
                when 3 =>
                    -- cycles per screen, Q4: 1..8 bands/columns, 8..72 stripes
                    v_m  := (v_g1 & "000") - ("0000" & v_g1);           -- g*7
                    f_c1 <= to_unsigned(16, 12) + resize(v_m(13 downto 6), 12);
                    v_m  := (v_g2 & "000") - ("0000" & v_g2);
                    f_c2 <= to_unsigned(16, 12) + resize(v_m(13 downto 6), 12);
                    f_c3 <= to_unsigned(128, 12) + resize(v_g3, 12);
                    -- warp terms: g*3 (taper amplitude A = g*3/8), lens pinch
                    -- a = g/2 (Q8: 0 .. 2.0); PM depth = g^2/512 (state 9)
                    f_g3  <= (v_g12 & "0") + ("00" & v_g12);           -- g*3
                    f_pin <= v_g12(9 downto 1);
                    -- width threshold: +384 (hairlines) .. -255 (fat); the
                    -- lattice runs 256 broader so the lens holes stay distinct
                    v_m  := ("0" & v_g4 & "00") + ("0000" & v_g4);      -- g*5
                    f_t  <= to_signed(384, 11) - signed(resize(v_m(12 downto 3), 11));
                    -- hue angle (sine ROM index)
                    f_hue <= v_g6(9 downto 2);
                    -- speed: bipolar with a centre deadband
                    v_d := signed('0' & v_g5) - to_signed(512, 11);
                    if v_d > 14 then
                        f_spd <= v_d - 14;
                    elsif v_d < -14 then
                        f_spd <= v_d + 14;
                    else
                        f_spd <= (others => '0');
                    end if;
                    q_st <= 4;
                when 4 =>
                    f_tA <= '0' & f_g3(11 downto 3);
                    f_tb <= f_t - 256;
                    if f_spd < 0 then
                        mu_a <= resize(unsigned(-f_spd), 20);
                        mu_b <= unsigned(resize(-f_spd, 10));
                    else
                        mu_a <= resize(unsigned(f_spd), 20);
                        mu_b <= unsigned(resize(f_spd, 10));
                    end if;
                    mu_go <= '1';
                    -- bands: cycles * 2^20 / H per line
                    dv_n  <= resize(f_c1, 12) & x"0000";
                    dv_d  <= '0' & s_meas_v;
                    dv_go <= '1';
                    q_st <= 5;
                when 5 =>
                    if dv_go = '0' and dv_run = '0' then
                        f_incy <= dv_q(19 downto 0);
                        dv_n   <= resize(f_c2, 12) & x"0000";
                        dv_d   <= '0' & s_meas_h;
                        dv_go  <= '1';
                        q_st <= 6;
                    end if;
                when 6 =>
                    if dv_go = '0' and dv_run = '0' then
                        f_incx <= dv_q(19 downto 0);
                        dv_n   <= resize(f_c3, 12) & x"0000";
                        dv_d   <= '0' & s_meas_h;
                        dv_go  <= '1';
                        q_st <= 7;
                    end if;
                when 7 =>
                    if dv_go = '0' and dv_run = '0' then
                        f_incs <= dv_q(19 downto 0);
                        -- taper ramp: 2*A*1024 / H per line
                        dv_n   <= "0000000" & f_tA & "00000000000";
                        dv_d   <= '0' & s_meas_v;
                        dv_go  <= '1';
                        q_st <= 8;
                    end if;
                when 8 =>
                    if dv_go = '0' and dv_run = '0' then
                        f_inct <= dv_q(19 downto 0);
                        q_st <= 9;
                    end if;
                when 9 =>
                    f_incxn <= (not f_incx) + 1;
                    f_incsn <= (not f_incs) + 1;
                    if mu_go = '0' and mu_run = '0' then
                        -- velocity = spd^2 / 16, signed (max ~15.4k/field)
                        if f_spd < 0 then
                            f_vel <= -signed(resize(mu_acc(19 downto 4), 21));
                        else
                            f_vel <= signed(resize(mu_acc(19 downto 4), 21));
                        end if;
                        mu_a  <= resize(v_g12, 20);
                        mu_b  <= v_g12;
                        mu_go <= '1';
                        q_st <= 10;
                    end if;
                when 10 => ph_r  <= ph_r  + f_lo20(shift_left(f_vel, 1));   q_st <= 11;
                when 11 => ph_m  <= ph_m  + f_lo20(f_vel);                  q_st <= 12;
                when 12 => ph_g  <= ph_g  - f_lo20(f_vel);                  q_st <= 13;
                when 13 => ph_bx <= ph_bx + f_lo20(shift_right(f_vel, 1));  q_st <= 14;
                when 14 => ph_by <= ph_by - f_lo20(f_vel)
                                          - f_lo20(shift_right(f_vel, 1));  q_st <= 15;
                when 15 => ph_s  <= ph_s  + f_lo20(shift_left(f_vel, 2));   q_st <= 16;
                when 16 => a_h <= f_hue;                                                 q_st <= 17;
                when 17 => a_h <= f_hue + 64;                                            q_st <= 18;
                when 18 => f_sin <= signed(q_h(9 downto 2));                             q_st <= 19;
                when 19 => f_cos <= signed(q_h(9 downto 2));                             q_st <= 20;
                when 20 =>
                    if mu_go = '0' and mu_run = '0' then
                        f_dep <= resize(mu_acc(19 downto 9), 12);      -- g^2/512, 0..2044
                        q_st <= 21;
                    end if;
                when 21 =>
                    -- restart the per-line accumulators for this field
                    y_r    <= ph_r;
                    y_b    <= ph_by;
                    y_s    <= (others => '0');
                    y_ramp <= -signed("00" & f_tA & "0000000000");
                    q_st <= 0;
                when others =>
                    q_st <= 0;
            end case;

            -- per-line accumulators: advance at the end of each active line
            if q_st /= 21 and g_avf = '1' then
                y_r    <= y_r + f_incy;
                y_b    <= y_b + f_incy;
                y_s    <= y_s + f_incs;
                y_ramp <= y_ramp + signed("00" & f_inct);
            end if;
        end if;
    end process p_frame;

    -- ====================================================================
    -- Pixel pipeline
    -- ====================================================================
    -- stage 0: x-phase accumulators (loaded at hsync, stepped per pixel of
    -- the generated avid, retraced after the centre column when mirrored)
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            -- line constants for stage 5: latched as pixel 0 enters stage 0,
            -- so the previous line's tail (SD front porch is short) is drained
            if g_avid = '1' and g_avd = '0' then
                ly_r   <= y_r;
                ly_b   <= y_b;
                l_ramp <= y_ramp(21 downto 10);
            end if;
            if g_hsr = '1' then
                p_m  <= ph_m;
                p_g  <= ph_g;
                p_bx <= ph_bx;
                if f_diag = '1' then
                    p_s <= ph_s + y_s;
                else
                    p_s <= ph_s;
                end if;
            elsif g_avid = '1' then
                if s_mir = '1' then
                    p_m  <= p_m  + f_incxn;
                    p_g  <= p_g  + f_incxn;
                    p_bx <= p_bx + f_incxn;
                    p_s  <= p_s  + f_incsn;
                else
                    p_m  <= p_m  + f_incx;
                    p_g  <= p_g  + f_incx;
                    p_bx <= p_bx + f_incx;
                    p_s  <= p_s  + f_incs;
                end if;
            end if;
        end if;
    end process p_acc;

    -- ROMs (EBR): every address is a register, every read is registered
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            q_m  <= C_SIN(to_integer(p_m(19 downto 12)));
            q_g  <= C_SIN(to_integer(p_g(19 downto 12)));
            q_bx <= C_SIN(to_integer(p_bx(19 downto 12)));
            q_s  <= C_SIN(to_integer(p_s(19 downto 12)));
            q_r  <= C_SIN(to_integer(s5_ar));
            q_b  <= C_SIN(to_integer(s5_ab));
            q_h  <= C_SIN(to_integer(a_h));
            q_cr <= C_CR(to_integer(s12_r8));
            q_cg <= C_CG(to_integer(s12_g8));
            q_cb <= C_CB(to_integer(s12_b8));
        end if;
    end process p_rom;

    p_pix : process(clk)
        variable v_sum : unsigned(19 downto 0);
        variable v_b9  : unsigned(8 downto 0);
        variable v_r8, v_g8, v_b8 : unsigned(7 downto 0);
        variable v_st  : signed(10 downto 0);
        variable v_sh  : signed(10 downto 0);
        variable v_c   : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 2: fabric re-register of the EBR outputs
            s2_m  <= signed(q_m(9 downto 0));
            s2_g  <= signed(q_g(9 downto 0));
            s2_bx <= signed(q_bx(9 downto 0));
            s2_s  <= signed(q_s(9 downto 0));

            -- stage 3: PM depth and lens pinch, split hi/lo
            s3_pmh <= s2_m  * signed('0' & f_dep(11 downto 6));
            s3_pml <= s2_m  * signed('0' & f_dep(5 downto 0));
            s3_pnh <= s2_bx * signed('0' & f_pin(8 downto 4));
            s3_pnl <= s2_bx * signed('0' & f_pin(3 downto 0));
            s3_g <= s2_g;
            s3_s <= s2_s;

            -- stage 4: recombine
            s4_pm <= shift_left(resize(s3_pmh, 23), 6) + resize(s3_pml, 23);
            s4_pn <= shift_left(resize(s3_pnh, 20), 4) + resize(s3_pnl, 20);
            s4_g <= s3_g;
            s4_s <= s3_s;

            -- stage 5: band / lattice row addresses, column value, pinch
            v_sum := ly_r + unsigned(s4_pm(19 downto 0));
            s5_ar <= v_sum(19 downto 12);
            v_sum := ly_b + unsigned(s4_pm(20 downto 1));
            s5_ab <= v_sum(19 downto 12);
            s5_pn <= resize(shift_right(s4_pn, 8), 12);
            s5_gv <= resize(s4_g, 12) + l_ramp;
            s5_s  <= s4_s;

            -- stage 6: (EBR reads of q_r / q_b)
            s6_pn <= s5_pn;
            s6_gv <= s5_gv;
            s6_s  <= s5_s;

            -- stage 7: fabric re-register
            s7_r  <= signed(q_r(9 downto 0));
            s7_b  <= signed(q_b(9 downto 0));
            s7_pn <= s6_pn;
            s7_gv <= s6_gv;
            s7_s  <= s6_s;

            -- stage 8: fields; line-screen erosion
            s8_rv <= resize(s7_r, 13);
            s8_bv <= resize(s7_b, 13) + resize(s7_pn, 13);
            s8_gv <= resize(s7_gv, 13);
            s8_s  <= s7_s;
            if f_screen = '1' then
                v_sh   := to_signed(511, 11) - resize(s7_s, 11);
                s8_ero <= resize(shift_right(v_sh, 1), 13);
            else
                s8_ero <= (others => '0');
            end if;

            -- stage 9: threshold (Width) and erosion
            s9_r  <= s8_rv - f_t - s8_ero;
            s9_g  <= s8_gv - f_t - s8_ero;
            s9_bp <= s8_bv - f_tb - s8_ero;
            s9_bn <= (-s8_bv) - f_tb - s8_ero;
            s9_s  <= s8_s;

            -- stage 10: knee -> 8-bit guns; grating 0..255
            s10_r8  <= f_knee(s9_r,  f_hard);
            s10_g8  <= f_knee(s9_g,  f_hard);
            s10_bp8 <= f_knee(s9_bp, f_hard);
            s10_bn8 <= f_knee(s9_bn, f_hard);
            if f_hard = '1' then
                if s9_s <= -64 then
                    s10_st8 <= (others => '0');
                elsif s9_s >= 64 then
                    s10_st8 <= (others => '1');
                else
                    v_st := resize(s9_s, 11) + 64;
                    s10_st8 <= unsigned(v_st(6 downto 0)) & '0';
                end if;
            else
                v_st := resize(s9_s, 11) + 512;
                s10_st8 <= unsigned(v_st(9 downto 2));
            end if;

            -- stage 11: grating fills the lattice holes
            s11_hole <= s10_bn8 * s10_st8;
            s11_r8   <= s10_r8;
            s11_g8   <= s10_g8;
            s11_bp8  <= s10_bp8;

            -- stage 12: blue = lattice + striped holes; Negative
            v_b9 := ('0' & s11_bp8) + ('0' & s11_hole(15 downto 8));
            if v_b9(8) = '1' then v_b8 := (others => '1');
            else                  v_b8 := v_b9(7 downto 0); end if;
            v_r8 := s11_r8;
            v_g8 := s11_g8;
            if f_neg = '1' then
                s12_r8 <= not v_r8;
                s12_g8 <= not v_g8;
                s12_b8 <= not v_b8;
            else
                s12_r8 <= v_r8;
                s12_g8 <= v_g8;
                s12_b8 <= v_b8;
            end if;

            -- stage 13: (EBR reads of the colour ROMs)
            -- stage 14: fabric re-register
            s14_cr <= q_cr;
            s14_cg <= q_cg;
            s14_cb <= q_cb;

            -- stage 15: RGB -> YUV is three adds
            s15_y <= C_BLK + unsigned(s14_cr(29 downto 20))
                           + unsigned(s14_cg(29 downto 20))
                           + unsigned(s14_cb(29 downto 20));
            s15_u <= resize(signed(s14_cr(19 downto 10)), 11)
                   + resize(signed(s14_cg(19 downto 10)), 11)
                   + resize(signed(s14_cb(19 downto 10)), 11);
            s15_v <= resize(signed(s14_cr(9 downto 0)), 11)
                   + resize(signed(s14_cg(9 downto 0)), 11)
                   + resize(signed(s14_cb(9 downto 0)), 11);

            -- stage 16: hue rotation partial products (cos/sin split hi/lo)
            s16_uch <= s15_u * f_cos(7 downto 4);
            s16_ucl <= s15_u * signed('0' & f_cos(3 downto 0));
            s16_ush <= s15_u * f_sin(7 downto 4);
            s16_usl <= s15_u * signed('0' & f_sin(3 downto 0));
            s16_vch <= s15_v * f_cos(7 downto 4);
            s16_vcl <= s15_v * signed('0' & f_cos(3 downto 0));
            s16_vsh <= s15_v * f_sin(7 downto 4);
            s16_vsl <= s15_v * signed('0' & f_sin(3 downto 0));
            s16_y   <= s15_y;

            -- stage 17: recombine
            s17_uc <= shift_left(resize(s16_uch, 19), 4) + resize(s16_ucl, 19);
            s17_us <= shift_left(resize(s16_ush, 19), 4) + resize(s16_usl, 19);
            s17_vc <= shift_left(resize(s16_vch, 19), 4) + resize(s16_vcl, 19);
            s17_vs <= shift_left(resize(s16_vsh, 19), 4) + resize(s16_vsl, 19);
            s17_y  <= s16_y;

            -- stage 18: rotate
            s18_u <= resize(shift_right(s17_uc - s17_vs, 7), 11);
            s18_v <= resize(shift_right(s17_us + s17_vc, 7), 11);
            s18_y <= s17_y;

            -- stage 19: compose 10-bit codes
            s19_y  <= s18_y;
            v_c := signed('0' & C_MID) + s18_u;
            s19_cb <= unsigned(v_c(9 downto 0));
            v_c := signed('0' & C_MID) + s18_v;
            s19_cr <= unsigned(v_c(9 downto 0));

            -- stages 20, 21: smoothing window
            s20_y  <= s19_y;  s20_cb <= s19_cb;  s20_cr <= s19_cr;
            s21_y  <= s20_y;  s21_cb <= s20_cb;  s21_cr <= s20_cr;

            -- output register: [1 2 1]/4 on the centre pixel (stage 20),
            -- blanked to neutral outside the generated avid
            if d_gv(C_LAT - 1) = '1' then
                o_y  <= resize(shift_right(resize(s19_y, 12) + shift_left(resize(s20_y, 12), 1)
                                           + resize(s21_y, 12), 2), 10);
                o_cb <= resize(shift_right(resize(s19_cb, 12) + shift_left(resize(s20_cb, 12), 1)
                                           + resize(s21_cb, 12), 2), 10);
                o_cr <= resize(shift_right(resize(s19_cr, 12) + shift_left(resize(s20_cr, 12), 1)
                                           + resize(s21_cr, 12), 2), 10);
            else
                o_y  <= C_BLK;
                o_cb <= C_MID;
                o_cr <= C_MID;
            end if;
            o_av <= d_gv(C_LAT - 1);
            o_hs <= d_hs(C_LAT - 1);
            o_vs <= d_vs(C_LAT - 1);
            o_fl <= d_fl(C_LAT - 1);
        end if;
    end process p_pix;

    -- ====================================================================
    -- Sync delay (one latency for every mode)
    -- ====================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            d_hs(1) <= data_in.hsync_n;
            d_vs(1) <= data_in.vsync_n;
            d_fl(1) <= data_in.field_n;
            d_gv(1) <= g_avid;
            for i in 2 to C_LAT - 1 loop
                d_hs(i) <= d_hs(i - 1);
                d_vs(i) <= d_vs(i - 1);
                d_fl(i) <= d_fl(i - 1);
                d_gv(i) <= d_gv(i - 1);
            end loop;
        end if;
    end process p_delay;

    -- the u register carries BT.601 Cr on this hardware
    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_cr);
    data_out.v       <= std_logic_vector(o_cb);
    data_out.avid    <= o_av;
    data_out.hsync_n <= o_hs;
    data_out.vsync_n <= o_vs;
    data_out.field_n <= o_fl;

end architecture trellis;
