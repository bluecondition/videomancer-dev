-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ringdown v0.1.0 -- Patchbay: a square wave through eight resonant filters.
--
-- The patch:
--   H-rate square-wave LFO (K1 Period, K3 Speed drift, S8 Square / Pulse)
--     -> a bank of eight resonant filters tuned alike (K2 Ring, a slow triangle LFO breathing the tuning), their
--        dampings stepped heavy -> light (K4 Spread), all scaled by P12 RESONANCE (which also opens the bank's
--        detune, so the traces weave at high Q)
--     -> eight comparators against the V ramp (S7 Mirror folds the ramp into two squeezed copies), the camera's
--        slewed luma added to the ramp (K6 Camera; S11 Source Video / Osc lattice)
--     -> a resistor ladder: how many traces lie below the beam (soft, slope-aware comparators)
--     -> a colouriser: nine bands with pale fringes (K5 rotation over a 16-colour wheel, S10 Spectrum / Neon,
--        S9 Bands / Traces)
--
-- Trace k is the periodic steady state of the k-th filter, truncated at four edges:
--   y_k = L - D * sum_{n=0..3} (-1)^n 2^(-Sigma_k (q + n)) cos(2 pi Omega_k (q + n))     (q = time since the edge
--   in half-periods; Omega_k ring cycles and Sigma_k octaves of decay per half-period, Sigma_k >= 1)
-- Every line draws the same traces, so the vblank zone computes them once per field (a fill engine: phase and
-- decay accumulators, a log|cos| ROM, an exp2 ROM with interpolation, four terms per sample) into eight
-- 1024 x 8 tables of slope codes (one per two output pixels, with error feedback and a 16-entry preamble that
-- rebuilds the line-start value in hblank).  Per pixel per trace: decode, accumulate, subtract the row, soft
-- compare.
--
--   K1  Period     square-wave cycles across the screen, 0.5 .. 5.7
--   K2  Ring       ring cycles per half-period, 1.5 .. 9
--   K3  Speed      bipolar (+-8 deadband): the square wave drifts along the line
--   K4  Spread     how far apart the eight dampings (and tunings) are: 0 = one heavy line
--   K5  Colour     colouriser rotation, 16 steps over the wheel (centre = the designed colours)
--   K6  Camera     slewed luma added to the V ramp: 0 = the black-input picture .. 100 % (the default)
--   S7  Mirror     Off / On (the V ramp folded at the centre: a butterfly)
--   S8  Drive      Square / Pulse (impulses: symmetric ringing bursts about the centre line)
--   S9  Look       Bands (the count staircase) / Traces (eight glowing scope lines on black)
--   S10 Colours    Spectrum / Neon
--   S11 Source     Video / Osc
--   P12 RESONANCE  0 % = near critical damping (a rounded square wave) .. 100 % = a dense woven lace
--
-- Shell (heirloom/lozenge): virtual 1080-line coordinates from the timing ID, vblank zone (no avid for 1024
-- clocks), generated avid, one latency (C_LAT 20), output conditioning (Y, U and V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16), blanking gate.  Colours in the hardware
-- convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.ringdown_pkg.all;
use work.all;

architecture ringdown of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    -- slope estimate of a code: bit length of the ones'-complement magnitude, + 3 when coarse (0 .. 9)
    function f_clen(c : std_logic_vector(7 downto 0)) return unsigned is
        variable m : std_logic_vector(5 downto 0);
        variable v : unsigned(3 downto 0);
    begin
        if c(6) = '1' then m := not c(5 downto 0); else m := c(5 downto 0); end if;
        if m(5) = '1' then v := to_unsigned(6, 4);
        elsif m(4) = '1' then v := to_unsigned(5, 4);
        elsif m(3) = '1' then v := to_unsigned(4, 4);
        elsif m(2) = '1' then v := to_unsigned(3, 4);
        elsif m(1) = '1' then v := to_unsigned(2, 4);
        elsif m(0) = '1' then v := to_unsigned(1, 4);
        else v := to_unsigned(0, 4); end if;
        if c(7) = '1' then v := v + 3; end if;
        return v;
    end function;
    -- soft comparator: clamp(floor(e / 2^(s + 2)) + 8, 0, 16), s = base (0 bands / 1 traces) + 3 on steep runs
    function f_cov(e : signed(15 downto 0); s : unsigned(2 downto 0)) return unsigned is
        variable u : unsigned(15 downto 0);
        variable w : unsigned(2 downto 0);
        variable hi, lo : std_logic;
        variable r : unsigned(4 downto 0);
    begin
        u := unsigned(e);
        case to_integer(s) is
            when 0      => w := u(4 downto 2);  hi := or u(14 downto 5);  lo := not (and u(14 downto 5));
            when 1      => w := u(5 downto 3);  hi := or u(14 downto 6);  lo := not (and u(14 downto 6));
            when 3      => w := u(7 downto 5);  hi := or u(14 downto 8);  lo := not (and u(14 downto 8));
            when others => w := u(8 downto 6);  hi := or u(14 downto 9);  lo := not (and u(14 downto 9));
        end case;
        if u(15) = '0' then
            if hi = '1' then r := "10000"; else r := "01" & w; end if;
        else
            if lo = '1' then r := "00000"; else r := "00" & w; end if;
        end if;
        return r;
    end function;
    -- traces look: tent core max(0, 4 - |cov - 8|)
    function f_core(c : unsigned(4 downto 0)) return unsigned is
    begin
        case to_integer(c) is
            when 8          => return to_unsigned(4, 3);
            when 7 | 9      => return to_unsigned(3, 3);
            when 6 | 10     => return to_unsigned(2, 3);
            when 5 | 11     => return to_unsigned(1, 3);
            when others     => return to_unsigned(0, 3);
        end case;
    end function;

    attribute no_rw_check : boolean;

    -- trace tables (8 x 1024 x 8, one write bus, one shared read address)
    type t_q8 is array (0 to 7) of std_logic_vector(7 downto 0);
    signal r_tq  : t_q8 := (others => (others => '0'));
    signal r_ta  : unsigned(9 downto 0) := (others => '0');
    signal t_we  : std_logic_vector(7 downto 0) := (others => '0');
    signal t_wa  : unsigned(9 downto 0) := (others => '0');
    signal t_wd  : std_logic_vector(7 downto 0) := (others => '0');

    -- luma cell store: one 12-bit vertical-slew state per 8-pixel cell (read a cell ahead of the write)
    type t_lb is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(7 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(15 downto 0) := x"0000";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(15 downto 0) := x"0000";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav, s_f1, s_f2 : std_logic := '0';
    signal s_lc    : unsigned(2 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(7 downto 0) := (others => '0');
    signal s_acc, s_sum8 : unsigned(10 downto 0) := (others => '0');
    signal s_cur, s_old, s_new : unsigned(11 downto 0) := (others => '0');
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_ga, s_gacc, s_gpr : unsigned(19 downto 0) := (others => '0');
    signal s_gb    : unsigned(6 downto 0) := (others => '0');
    signal s_h1   : unsigned(17 downto 0) := (others => '0');
    signal s_lterm : unsigned(13 downto 0) := (others => '0');

    -- ROMs (one read port each; data registered in the EBR, then a fabric register)
    signal z_exa, t_exa, f_exa : unsigned(7 downto 0) := (others => '0');
    signal ex_q, ex_r : unsigned(15 downto 0) := (others => '0');
    signal lc_a : unsigned(7 downto 0) := (others => '0');
    signal lc_q : unsigned(15 downto 0) := (others => '0');
    signal pal_ra : unsigned(8 downto 0) := (others => '0');
    signal pal_q, pal_r : unsigned(23 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
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

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(420 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(260 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(700 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(800 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k3 : unsigned(9 downto 0) := to_unsigned(650, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_pul, s_trl, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal c_e1   : signed(10 downto 0) := (others => '0');
    signal c_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_em   : unsigned(9 downto 0) := (others => '0');
    signal c_m    : unsigned(15 downto 0) := to_unsigned(8192, 16);     -- ring cycles per half-period, Q12
    signal c_dq   : unsigned(18 downto 0) := (others => '0');           -- half-periods per sample, Q24
    signal c_p2, c_p4 : unsigned(9 downto 0) := (others => '0');
    signal c_lz   : signed(12 downto 0) := (others => '0');             -- log2 of the centre damping, Q8
    signal c_ds2  : unsigned(6 downto 0) := (others => '0');            -- spread per half-trace, Q8 octaves
    signal c_tq   : unsigned(16 downto 0) := (others => '0');
    signal c_det  : unsigned(15 downto 0) := (others => '0');           -- detune spread, Q16
    signal c_dst  : unsigned(13 downto 0) := (others => '0');           -- detune step per trace (2/7 det)
    signal c_fdm0 : unsigned(16 downto 0) := (others => '0');           -- m (1 - det), Q12
    signal c_fdms : unsigned(13 downto 0) := (others => '0');           -- m * detune step, Q12
    signal c_phd  : unsigned(24 downto 0) := to_unsigned(6207570, 25);  -- drift phase, Q24 (mod 2 half-periods)
    signal c_vsd  : unsigned(24 downto 0) := (others => '0');           -- signed drift step (two's complement)
    signal c_phl  : unsigned(15 downto 0) := (others => '0');           -- LFO phase
    signal c_q0   : unsigned(24 downto 0) := (others => '0');           -- square-wave phase at sample -1
    signal c_gf8  : unsigned(7 downto 0) := (others => '0');            -- camera gain (cell rate)
    signal c_ld   : unsigned(15 downto 0) := to_unsigned(44916, 16);    -- log2 |step| in quarter-rows, Q12
    signal c_php  : unsigned(23 downto 0) := (others => '0');           -- pulse: -1/4 cycle (cos -> sin)
    signal c_S    : unsigned(10 downto 0) := to_unsigned(961, 11);      -- samples per line + 1
    signal c_r    : unsigned(3 downto 0) := (others => '0');            -- colouriser rotation

    -- shared serial multiplier (24 x 24, right-shift accumulator; product in mu_p from load + 26)
    signal mu_a   : unsigned(20 downto 0) := (others => '0');
    signal mu_ph  : unsigned(20 downto 0) := (others => '0');
    signal mu_pl  : unsigned(18 downto 0) := (others => '0');
    signal mu_n   : unsigned(4 downto 0) := (others => '0');
    signal mu_p   : unsigned(39 downto 0) := (others => '0');
    signal z_sel, t_sel : unsigned(3 downto 0) := (others => '0');
    signal z_ld   : std_logic := '0';
    signal z_gp   : std_logic_vector(1 downto 0) := "00";
    signal z_gc   : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- trace engine (per trace: constants, then the fill)
    signal t_go, t_busy, t_done, t_ld, t_init : std_logic := '0';
    signal t_st   : unsigned(3 downto 0) := (others => '0');
    signal t_wt   : unsigned(6 downto 0) := (others => '0');
    signal t_k    : unsigned(3 downto 0) := (others => '0');
    signal t_lzr, t_lzk, t_ls : signed(12 downto 0) := (others => '0');
    signal t_sig  : unsigned(18 downto 0) := (others => '0');   -- Q12 octaves per half-period
    signal t_fdm  : unsigned(16 downto 0) := (others => '0');
    signal t_f1   : unsigned(16 downto 0) := (others => '0');
    signal t_om   : unsigned(20 downto 0) := (others => '0');
    signal t_omf, t_dth, t_dD, t_th, t_D, t_qf : unsigned(23 downto 0) := (others => '0');
    signal t_par  : std_logic := '0';
    signal t_sc   : unsigned(10 downto 0) := (others => '0');
    signal t_tn   : unsigned(1 downto 0) := (others => '0');
    signal t_run  : std_logic := '0';
    -- fill datapath, one term per clock
    signal f1_v, f1_par, f1_pre : std_logic := '0';
    signal f1_tn  : unsigned(1 downto 0) := (others => '0');
    signal f1_th  : unsigned(23 downto 0) := (others => '0');
    signal f1_D   : unsigned(25 downto 0) := (others => '0');
    signal f2_v, f2_par, f2_pre, f2_cs, f2_sat : std_logic := '0';
    signal f2_tn  : unsigned(1 downto 0) := (others => '0');
    signal f2_vq  : signed(17 downto 0) := (others => '0');
    signal f3_v, f3_par, f3_pre, f3_cs, f3_sat : std_logic := '0';
    signal f3_tn  : unsigned(1 downto 0) := (others => '0');
    signal f3_vq  : signed(17 downto 0) := (others => '0');
    signal f4_v, f4_par, f4_pre, f4_cs, f4_sat : std_logic := '0';
    signal f4_tn  : unsigned(1 downto 0) := (others => '0');
    signal f4_vq  : signed(17 downto 0) := (others => '0');
    signal f4_lc  : unsigned(15 downto 0) := (others => '0');
    signal f5_v, f5_par, f5_pre, f5_cs, f5_sat : std_logic := '0';
    signal f5_tn  : unsigned(1 downto 0) := (others => '0');
    signal f5_u   : signed(17 downto 0) := (others => '0');
    type t_tag is array (6 to 12) of std_logic_vector(5 downto 0);    -- v, par, pre, neg, tn(1 downto 0)
    signal f_tag  : t_tag := (others => (others => '0'));
    signal f6_ok, f7_ok, f8_ok, f9_ok, f10_ok, f11_ok : std_logic := '0';
    signal f6_r, f7_r, f8_r : unsigned(3 downto 0) := (others => '0');
    type t_it is array (6 to 10) of unsigned(3 downto 0);
    signal f_it   : t_it := (others => (others => '0'));
    signal f9_rs  : unsigned(8 downto 0) := (others => '0');
    signal f9_t   : unsigned(10 downto 0) := (others => '0');
    signal f10_mt : unsigned(15 downto 0) := (others => '0');
    signal f11_mg : unsigned(14 downto 0) := (others => '0');
    signal f12_tm : signed(17 downto 0) := (others => '0');
    signal f13_acc : signed(19 downto 0) := (others => '0');
    signal f13_last, f13_pre : std_logic := '0';
    signal f15_y  : signed(13 downto 0) := (others => '0');
    signal f15_v, f15_pre : std_logic := '0';
    -- encoder
    signal e_y, e_yr : signed(13 downto 0) := (others => '0');
    signal e_rep  : unsigned(4 downto 0) := (others => '0');
    signal e_ph   : unsigned(1 downto 0) := (others => '0');
    signal e_code : std_logic_vector(7 downto 0) := (others => '0');
    signal e_dl   : signed(10 downto 0) := (others => '0');
    signal e_j    : unsigned(9 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line: V ramp (eighth-rows), mirror; preamble
    signal l_vy   : unsigned(18 downto 0) := (others => '0');
    signal l_r8   : unsigned(13 downto 0) := (others => '0');
    signal l_m8   : unsigned(14 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal pr_act : std_logic := '0';
    signal pr_cnt : unsigned(4 downto 0) := (others => '0');

    -- pixel path
    signal a_v, a_f, v1, f1p, v2, f2p, v3, f3p : std_logic := '0';
    type t_cov is array (0 to 7) of unsigned(4 downto 0);
    signal cov7  : t_cov := (others => (others => '0'));
    signal ye5   : unsigned(14 downto 0) := (others => '0');
    type t_s1 is array (0 to 3) of unsigned(5 downto 0);
    signal s1    : t_s1 := (others => (others => '0'));
    type t_c1 is array (0 to 3) of unsigned(3 downto 0);
    signal cp1   : t_c1 := (others => (others => '0'));
    type t_g1 is array (0 to 3) of unsigned(1 downto 0);
    signal gp1   : t_g1 := (others => (others => '0'));
    signal cnt8  : unsigned(7 downto 0) := (others => '0');
    signal nc8   : unsigned(5 downto 0) := (others => '0');
    signal ng8   : unsigned(3 downto 0) := (others => '0');
    signal b9, b10, b11 : unsigned(3 downto 0) := (others => '0');
    signal y12, u12, v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cby : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bayd, s_bby, s_bbyd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    -- chroma 8-back taps of both boxes: 8-bit samples in two block-RAM rings (read 6 behind the write + a
    -- fabric register = 8 samples back)
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ra_ram, rb_ram : t_ring := (others => x"8080");
    attribute no_rw_check of ra_ram : signal is true;
    attribute no_rw_check of rb_ram : signal is true;
    signal r_wp  : unsigned(7 downto 0) := (others => '0');
    signal ra_q, ra_r, rb_q, rb_r : std_logic_vector(15 downto 0) := x"8080";
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = g_avid k + 1 clocks ago

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    g_tt : for k in 0 to 7 generate
        type t_tt is array (0 to 1023) of std_logic_vector(7 downto 0);
        signal ram : t_tt := (others => x"00");
        attribute no_rw_check of ram : signal is true;
    begin
        p_tt : process(clk)
        begin
            if rising_edge(clk) then
                r_tq(k) <= ram(to_integer(r_ta));
                if t_we(k) = '1' then ram(to_integer(t_wa)) <= t_wd; end if;
            end if;
        end process p_tt;
    end generate g_tt;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
        variable v_a : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if t_run = '1' then v_a := f_exa; elsif t_busy = '1' then v_a := t_exa; else v_a := z_exa; end if;
            ex_q <= C_RD_EXP(to_integer(v_a));
            ex_r <= ex_q;
            lc_q <= C_RD_LOGCOS(to_integer(lc_a));
            pal_q <= C_RD_PAL(to_integer(pal_ra));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Shared serial multiplier: P = A * B (24 x 24), operands picked at the load
    -- by the select code (the zone sequencer: 0 .. 10, the trace engine: 11 .. 15)
    ---------------------------------------------------------------------------
    p_mul : process(clk)
        variable v_s  : unsigned(21 downto 0);
        variable v_sl : unsigned(3 downto 0);
        variable v_a  : unsigned(20 downto 0);
        variable v_b  : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            if z_ld = '1' or t_ld = '1' then
                if t_ld = '1' then v_sl := t_sel; else v_sl := z_sel; end if;
                case to_integer(v_sl) is
                    when 0  => v_a := resize(s_gp(12 downto 3), 21); v_b := resize(s_gp(12 downto 3), 19);
                    when 1  => v_a := resize(c_p2, 21);              v_b := resize(c_p2, 19);
                    when 2  => v_a := resize(s_g2(12 downto 3), 21); v_b := to_unsigned(657, 19);
                    when 3  => v_a := resize(c_p4, 21);              v_b := to_unsigned(46500, 19);
                    when 4  => v_a := resize(s_g4(12 downto 3), 21); v_b := resize(c_tq, 19);
                    when 5  => v_a := resize(c_det, 21);             v_b := to_unsigned(18725, 19);
                    when 6  => v_a := resize(s_vm, 21);              v_b := resize(s_vm, 19);
                    when 7  =>
                        case to_integer(s_xs) is
                            when 4      => v_a := to_unsigned(69904, 21);
                            when 3      => v_a := to_unsigned(52428, 21);
                            when others => v_a := to_unsigned(34952, 21);
                        end case;
                        v_b := resize(c_mt, 19);
                    when 8  => v_a := resize(c_m, 21);               v_b := to_unsigned(65536, 19) - resize(c_det, 19);
                    when 9  => v_a := resize(c_m, 21);               v_b := resize(c_dst, 19);
                    when 10 => v_a := resize(c_dq, 21);              v_b := to_unsigned(239, 19);
                    when 11 => v_a := resize(t_f1, 21);              v_b := resize(t_fdm, 19);
                    when 12 => v_a := t_om;                          v_b := c_dq;
                    when 13 => v_a := resize(t_sig, 21);             v_b := c_dq;
                    when 14 => v_a := t_om;                          v_b := resize(c_q0(23 downto 8), 19);
                    when others => v_a := resize(t_sig, 21);         v_b := resize(c_q0(23 downto 8), 19);
                end case;
                mu_a <= v_a; mu_pl <= v_b; mu_ph <= (others => '0'); mu_n <= to_unsigned(19, 5);
            elsif mu_n /= 0 then
                if mu_pl(0) = '1' then v_s := ('0' & mu_ph) + ('0' & mu_a); else v_s := '0' & mu_ph; end if;
                mu_ph <= v_s(21 downto 1);
                mu_pl <= v_s(0) & mu_pl(18 downto 1);
                mu_n  <= mu_n - 1;
            end if;
            mu_p <= mu_ph & mu_pl;
        end if;
    end process p_mul;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_t  : unsigned(15 downto 0);
        variable v_x  : signed(11 downto 0);
        variable v_g  : unsigned(10 downto 0);
        variable v_k  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- vblank zone: no avid for 1024 clocks
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

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            -- speed: bipolar, +-8 deadband
            s_k3 <= unsigned(registers_in(2));
            v_s := signed(resize(s_k3, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;     -- |v| - 8
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mir <= registers_in(6)(0);
                s_pul <= registers_in(6)(1);
                s_trl <= registers_in(6)(2);
                s_pal <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
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
                            when 2      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot s (0 .. 10) loads at zc = 32 + 32 s (select set at 31 + 32 s), read at 62 + 32 s
            z_ld <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 31 and s_zc(10 downto 5) < 11 then
                z_ld <= '1';
                case to_integer(s_zc(8 downto 5)) is
                    when 0      => z_sel <= to_unsigned(7, 4);      -- dq
                    when 1      => z_sel <= to_unsigned(0, 4);      -- p2
                    when 2      => z_sel <= to_unsigned(2, 4);      -- em
                    when 3      => z_sel <= to_unsigned(1, 4);      -- p4
                    when 4      => z_sel <= to_unsigned(3, 4);      -- tq
                    when 5      => z_sel <= to_unsigned(4, 4);      -- det
                    when 6      => z_sel <= to_unsigned(5, 4);      -- detune step
                    when 7      => z_sel <= to_unsigned(6, 4);      -- speed^2
                    when 8      => z_sel <= to_unsigned(8, 4);      -- m (1 - det)
                    when 9      => z_sel <= to_unsigned(9, 4);      -- m * step
                    when others => z_sel <= to_unsigned(10, 4);     -- SD x offset
                end case;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion: last field's drift step, the LFO; per-field mode constants
                    when 20 =>
                        c_phd <= c_phd + c_vsd;
                        c_phl <= c_phl + 99;
                        if s_pul = '1' then
                            c_ld <= to_unsigned(42370, 16); c_php <= to_unsigned(12582912, 24);
                        else
                            c_ld <= to_unsigned(44916, 16); c_php <= (others => '0');
                        end if;
                        c_S <= resize(s_w(11 downto 1), 11) + 1;
                    when 26 =>                      -- e1 = -256 + 7/8 k1; em = 150 + 0.64 k2 + LFO; K4 / K5 / K6
                        v_k := s_g1(12 downto 3);
                        c_e1 <= signed(resize(v_k - shift_right(v_k, 3), 11)) - 256;
                        v_k := s_g4(12 downto 3);
                        c_ds2 <= resize(shift_right(v_k, 4), 7) + resize(shift_right(v_k, 6), 7);
                        c_r <= s_g5(12 downto 9) + 8;
                        v_k := unsigned(registers_in(5));
                        if v_k > 512 then v_g := resize(v_k, 11) + resize(shift_right(v_k - 512, 1), 11);
                        else v_g := resize(v_k, 11); end if;
                        c_gf8 <= v_g(10 downto 3);
                    when 27 => z_exa <= unsigned(c_e1(7 downto 0));
                    when 30 => c_mt <= ("00001" & ex_r(10 downto 0)) sll 4;
                    when 126 =>                     -- em = 150 + 0.64 k2 + LFO (triangle, +-32)
                        if c_phl(15) = '1' then v_t := '0' & (not c_phl(14 downto 0)); else v_t := '0' & c_phl(14 downto 0); end if;
                        v_x := resize(shift_right(signed(v_t) - 16384, 9), 12);
                        v_x := v_x + signed(resize(mu_p(19 downto 10), 12)) + 150;
                        c_em <= unsigned(v_x(9 downto 0));
                    when 127 => z_exa <= c_em(7 downto 0);
                    when 130 =>                     -- m = mant << (em >> 8) >> 3 (Q12)
                        case to_integer(c_em(9 downto 8)) is
                            when 0      => c_m <= resize(shift_right(("00001" & ex_r(10 downto 0)) & "0000", 3), 16);
                            when 1      => c_m <= resize(shift_right(("00001" & ex_r(10 downto 0)) & "0000", 2), 16);
                            when 2      => c_m <= resize(shift_right(("00001" & ex_r(10 downto 0)) & "0000", 1), 16);
                            when others => c_m <= resize(("00001" & ex_r(10 downto 0)) & "0000", 16);
                        end case;
                    when 62 =>                      -- dq = KQ * mant >> (15 - e1 >> 8), e1 >> 8 = -1 .. 2
                        case to_integer(unsigned(c_e1(10 downto 8))) is
                            when 7      => c_dq <= resize(mu_p(32 downto 16), 19);
                            when 0      => c_dq <= resize(mu_p(32 downto 15), 19);
                            when 1      => c_dq <= resize(mu_p(32 downto 14), 19);
                            when others => c_dq <= mu_p(31 downto 13);
                        end case;
                    when 94  => c_p2 <= mu_p(19 downto 10);
                    when 158 =>
                        c_p4 <= mu_p(19 downto 10);
                        c_lz <= to_signed(-189, 13) - signed(resize(c_p2, 13)) - signed(resize(shift_right(c_p2, 2), 13))
                                + signed(resize(shift_right(c_p2, 6), 13));
                    when 190 => c_tq <= to_unsigned(9830, 17) + resize(mu_p(25 downto 10), 17);
                    when 222 => c_det <= mu_p(25 downto 10);
                    when 254 => c_dst <= mu_p(29 downto 16);
                    when 286 =>                     -- drift step = 2 vm^2, signed by the knob side (right = rightward)
                        if s_vneg = '1' then c_vsd <= resize(mu_p(17 downto 0) & '0', 25);
                        else c_vsd <= (not resize(mu_p(17 downto 0) & '0', 25)) + 1; end if;
                    when 318 => c_fdm0 <= resize(mu_p(31 downto 16), 17);
                    when 350 => c_fdms <= mu_p(29 downto 16);
                    when 382 =>                     -- square-wave phase at sample -1 (Q24, mod two half-periods)
                        if s_xs = 4 then
                            c_q0 <= c_phd + resize(mu_p(26 downto 2), 25);
                        else
                            c_q0 <= c_phd - resize(c_dq(18 downto 2), 25);
                        end if;
                    when others => null;
                end case;
            end if;
            t_go <= '0';
            if s_zone = '1' and s_zc = 400 then t_go <= '1'; end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Trace engine (kicked once per field at zc 400): for each trace, the
    -- constants (three exp2 look-ups, five serial products), then the fill: one
    -- term per clock, four terms per sample (sample -1 first, then a pause while
    -- the encoder writes its 16-entry preamble), wrap-exact phase / decay
    -- accumulators at the square-wave edges
    ---------------------------------------------------------------------------
    p_trace : process(clk)
        variable v_mt : unsigned(15 downto 0);
        variable v_sg : unsigned(23 downto 0);
        variable v_q  : unsigned(24 downto 0);
        variable v_d  : signed(25 downto 0);
        variable v_z  : unsigned(15 downto 0);
        variable v_e  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            t_ld <= '0'; t_done <= '0'; t_init <= '0';
            f1_v <= '0';
            if t_wt /= 0 then t_wt <= t_wt - 1; end if;
            if t_go = '1' then
                t_busy <= '1'; t_run <= '0'; t_k <= (others => '0'); t_st <= (others => '0');
                t_fdm <= c_fdm0;
                t_lzr <= c_lz + signed(resize(c_ds2 & "000", 13)) - signed(resize(c_ds2, 13));
                t_wt  <= (others => '0');
            elsif t_busy = '1' and t_wt = 0 then
                case to_integer(t_st) is
                    when 0 =>                       -- clamp the damping exponent (Q8 octaves)
                        if t_lzr < -1701 then t_lzk <= to_signed(-1701, 13);
                        elsif t_lzr > -19 then t_lzk <= to_signed(-19, 13);
                        else t_lzk <= t_lzr; end if;
                        t_st <= to_unsigned(1, 4);
                    when 1 =>                       -- look up zeta^2 = 2^(2 lzk / 256)
                        t_exa <= unsigned(t_lzk(6 downto 0)) & '0';
                        t_ls  <= t_lzk + signed(resize(c_em, 13)) + 815;
                        t_wt  <= to_unsigned(3, 7); t_st <= to_unsigned(2, 4);
                    when 2 =>                       -- f1 = 1 - zeta^2 / 2 (zeta^2 = mant >> (-1 - lzk >> 7))
                        v_mt := ("00001" & ex_r(10 downto 0)) sll 4;
                        v_e := shift_right(t_lzk, 7);              -- -14 .. -1: zeta^2 = mant >> (-1 - v_e)
                        if v_e < -6 then v_z := (others => '0');   -- zeta^2 / 2 < 1/128: f1 = 1
                        else v_z := shift_right(v_mt, to_integer(unsigned(not v_e(2 downto 0)))); end if;
                        t_f1 <= to_unsigned(65536, 17) - resize(v_z(15 downto 1), 17);
                        t_exa <= unsigned(t_ls(7 downto 0));
                        t_wt  <= to_unsigned(3, 7); t_st <= to_unsigned(3, 4);
                    when 3 =>                       -- Sigma (Q16 octaves per half-period) = mant << (ls >> 8 + 1), >= 1
                        v_mt := ("00001" & ex_r(10 downto 0)) sll 4;
                        case to_integer(unsigned(t_ls(11 downto 8))) is
                            when 12     => v_sg := resize(shift_right(v_mt, 3), 24);
                            when 13     => v_sg := resize(shift_right(v_mt, 2), 24);
                            when 14     => v_sg := resize(shift_right(v_mt, 1), 24);
                            when 15     => v_sg := resize(v_mt, 24);
                            when 0      => v_sg := shift_left(resize(v_mt, 24), 1);
                            when 1      => v_sg := shift_left(resize(v_mt, 24), 2);
                            when 2      => v_sg := shift_left(resize(v_mt, 24), 3);
                            when 3      => v_sg := shift_left(resize(v_mt, 24), 4);
                            when 4      => v_sg := shift_left(resize(v_mt, 24), 5);
                            when 5      => v_sg := shift_left(resize(v_mt, 24), 6);
                            when others => v_sg := shift_left(resize(v_mt, 24), 7);
                        end case;
                        if v_sg < 65536 then t_sig <= to_unsigned(4096, 19); else t_sig <= v_sg(22 downto 4); end if;   -- Q12
                        t_sel <= to_unsigned(11, 4); t_ld <= '1';               -- f1 * fdm
                        t_wt <= to_unsigned(28, 7); t_st <= to_unsigned(4, 4);
                    when 4 =>                       -- Omega (Q16 cycles per half-period); Omega * dq
                        t_om <= mu_p(32 downto 12);
                        t_sel <= to_unsigned(12, 4); t_ld <= '1';
                        t_wt <= to_unsigned(28, 7); t_st <= to_unsigned(5, 4);
                    when 5 =>                       -- dtheta (Q24 per sample); Sigma * dq
                        t_dth <= mu_p(39 downto 16);
                        t_sel <= to_unsigned(13, 4); t_ld <= '1';
                        t_wt <= to_unsigned(28, 7); t_st <= to_unsigned(6, 4);
                    when 6 =>                       -- dD (Q16 octaves per sample); Omega * q
                        t_dD <= resize(mu_p(39 downto 20), 24);
                        t_sel <= to_unsigned(14, 4); t_ld <= '1';
                        t_wt <= to_unsigned(28, 7); t_st <= to_unsigned(7, 4);
                    when 7 =>                       -- theta0 (+ pulse offset); Sigma * q
                        t_th <= mu_p(31 downto 8) + c_php;
                        t_sel <= to_unsigned(15, 4); t_ld <= '1';
                        t_wt <= to_unsigned(28, 7); t_st <= to_unsigned(8, 4);
                    when 8 =>                       -- D0; start the fill
                        t_D   <= resize(mu_p(34 downto 12), 24);
                        t_omf <= t_om(15 downto 0) & x"00";
                        t_qf  <= c_q0(23 downto 0); t_par <= c_q0(24);
                        t_sc  <= (others => '0'); t_tn <= "00";
                        t_run <= '1'; t_init <= '1';
                        t_st  <= to_unsigned(9, 4);
                    when 9 =>                       -- fill: one term per clock
                        f1_v <= '1'; f1_tn <= t_tn; f1_par <= t_par;
                        if t_sc = 0 then f1_pre <= '1'; else f1_pre <= '0'; end if;
                        if t_tn = 0 then
                            f1_th <= t_th; f1_D <= resize(t_D, 26);
                        else
                            f1_th <= f1_th + t_omf; f1_D <= f1_D + resize(t_sig & "0000", 26);
                        end if;
                        t_tn <= t_tn + 1;
                        if t_tn = 3 then
                            v_q := ('0' & t_qf) + resize(c_dq, 25);
                            if v_q(24) = '1' then
                                t_par <= not t_par;
                                t_th  <= t_th + t_dth - t_omf;
                                v_d := signed(resize(t_D, 26)) + signed(resize(t_dD, 26)) - signed(resize(t_sig & "0000", 26));
                                if v_d < 0 then t_D <= (others => '0'); else t_D <= unsigned(v_d(23 downto 0)); end if;
                            else
                                t_th <= t_th + t_dth;
                                t_D  <= t_D + t_dD;
                            end if;
                            t_qf <= v_q(23 downto 0);
                            t_sc <= t_sc + 1;
                            if t_sc = 0 then
                                t_wt <= to_unsigned(48, 7);
                            elsif t_sc = c_S then
                                t_wt <= to_unsigned(40, 7); t_st <= to_unsigned(10, 4);
                            end if;
                        end if;
                    when others =>                  -- next trace
                        t_run <= '0';
                        t_k   <= t_k + 1;
                        t_fdm <= t_fdm + resize(c_fdms, 17);
                        t_lzr <= t_lzr - signed(resize(c_ds2 & '0', 13));
                        if t_k = 7 then
                            t_busy <= '0'; t_done <= '1';
                        else
                            t_st <= (others => '0');
                        end if;
                end case;
            end if;
        end if;
    end process p_trace;

    ---------------------------------------------------------------------------
    -- Fill datapath (F2 .. F16: log|cos|, exp2 with interpolation, signed
    -- terms, the 4-term sum, the level) and the slope encoder (error feedback)
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_i  : unsigned(9 downto 0);
        variable v_y  : signed(19 downto 0);
        variable v_df : signed(14 downto 0);
        variable v_q  : signed(14 downto 0);
        variable v_lv : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- F2: folded phase -> log|cos| address; decay -> exponent
            v_i := f1_th(23 downto 14);
            if v_i(8) = '1' then lc_a <= not v_i(7 downto 0); else lc_a <= v_i(7 downto 0); end if;
            f2_cs <= v_i(9) xor v_i(8);
            if f1_D(25 downto 20) /= 0 then f2_sat <= '1'; else f2_sat <= '0'; end if;
            f2_vq <= signed(resize(c_ld, 18)) - signed(resize(f1_D(19 downto 4), 18));
            f2_v <= f1_v; f2_par <= f1_par; f2_pre <= f1_pre; f2_tn <= f1_tn;
            -- F3: ROM
            f3_v <= f2_v; f3_par <= f2_par; f3_pre <= f2_pre; f3_tn <= f2_tn; f3_cs <= f2_cs; f3_sat <= f2_sat; f3_vq <= f2_vq;
            -- F4: fabric copy
            f4_lc <= lc_q;
            f4_v <= f3_v; f4_par <= f3_par; f4_pre <= f3_pre; f4_tn <= f3_tn; f4_cs <= f3_cs; f4_sat <= f3_sat; f4_vq <= f3_vq;
            -- F5: u = log2 |step| - D - log|cos| + 4 octaves (Q12; 1/16 quarter-row resolution)
            f5_u <= f4_vq - signed(resize(f4_lc, 18)) + 16384;
            f5_v <= f4_v; f5_par <= f4_par; f5_pre <= f4_pre; f5_tn <= f4_tn; f5_cs <= f4_cs; f5_sat <= f4_sat;
            -- F6: exp2 address, interpolation fraction, integer part
            f_exa <= unsigned(f5_u(11 downto 4));
            f6_r  <= unsigned(f5_u(3 downto 0));
            f_it(6) <= unsigned(f5_u(15 downto 12));
            f6_ok <= (not f5_u(17)) and (not f5_sat) and f5_v;
            -- term sign: cos sign, alternating edges, and the step direction (square: down after a low half;
            -- pulse: alternating kicks)
            f_tag(6) <= f5_v & f5_par & f5_pre & (f5_cs xor f5_tn(0) xor (s_pul xnor f5_par)) & std_logic_vector(f5_tn);
            -- F7: ROM; F8: fabric copy (ex_r)
            f7_ok <= f6_ok; f7_r <= f6_r; f_it(7) <= f_it(6); f_tag(7) <= f_tag(6);
            f8_ok <= f7_ok; f8_r <= f7_r; f_it(8) <= f_it(7); f_tag(8) <= f_tag(7);
            -- F9: interpolation product
            f9_rs <= f8_r * ex_r(15 downto 11);
            f9_t  <= ex_r(10 downto 0);
            f9_ok <= f8_ok; f_it(9) <= f_it(8); f_tag(9) <= f_tag(8);
            -- F10: mantissa (Q15)
            f10_mt <= ("1" & f9_t & "0000") + resize(f9_rs(8 downto 1), 16);
            f10_ok <= f9_ok; f_it(10) <= f_it(9); f_tag(10) <= f_tag(9);
            -- F11: magnitude (1/16 quarter-rows)
            f11_mg <= resize(shift_right(f10_mt, 15 - to_integer(f_it(10))), 15);
            f11_ok <= f10_ok; f_tag(11) <= f_tag(10);
            -- F12: signed term (cos sign, alternating edges)
            if f11_ok = '0' then
                f12_tm <= (others => '0');
            elsif f_tag(11)(2) = '1' then
                f12_tm <= -signed(resize(f11_mg, 18));
            else
                f12_tm <= signed(resize(f11_mg, 18));
            end if;
            f_tag(12) <= f_tag(11);
            -- F13: the level plus the four signed terms (1/16 quarter-rows)
            if s_pul = '1' then v_lv := to_signed(34560, 20);
            elsif f_tag(12)(4) = '1' then v_lv := to_signed(18560, 20);
            else v_lv := to_signed(50560, 20); end if;
            if f_tag(12)(1 downto 0) = "00" then
                f13_acc <= v_lv + resize(f12_tm, 20);
            else
                f13_acc <= f13_acc + resize(f12_tm, 20);
            end if;
            if f_tag(12)(5) = '1' and f_tag(12)(1 downto 0) = "11" then f13_last <= '1'; else f13_last <= '0'; end if;
            f13_pre <= f_tag(12)(3);
            -- F14: quarter-rows, clamped to -64 .. 1144 rows
            v_y := shift_right(f13_acc + 8, 4);
            if v_y < -256 then f15_y <= to_signed(-256, 14);
            elsif v_y > 4576 then f15_y <= to_signed(4576, 14);
            else f15_y <= v_y(13 downto 0); end if;
            f15_v <= f13_last; f15_pre <= f13_pre;

            -- encoder: fine codes (bit 7 = 0) carry a signed 7-bit quarter-row step, coarse codes eight times it;
            -- the reconstruction e_yr is exactly what the pixel accumulators rebuild (error feedback)
            t_we <= (others => '0');
            if t_init = '1' then
                e_yr <= (others => '0'); e_j <= (others => '0'); e_ph <= "00";
            elsif f15_v = '1' then
                e_y <= f15_y;
                if f15_pre = '1' then e_rep <= to_unsigned(16, 5); else e_rep <= to_unsigned(1, 5); end if;
                e_ph <= "01";
            elsif e_ph = "01" then
                v_df := resize(e_y, 15) - resize(e_yr, 15);
                if v_df >= -64 and v_df <= 63 then
                    e_code <= '0' & std_logic_vector(v_df(6 downto 0));
                    e_dl <= resize(v_df, 11);
                else
                    v_q := shift_right(v_df + 4, 3);
                    if v_q < -64 then v_q := to_signed(-64, 15); elsif v_q > 63 then v_q := to_signed(63, 15); end if;
                    e_code <= '1' & std_logic_vector(v_q(6 downto 0));
                    e_dl <= resize(v_q(7 downto 0) & "000", 11);
                end if;
                e_ph <= "10";
            elsif e_ph = "10" then
                e_yr <= e_yr + resize(e_dl, 14);
                t_wa <= e_j; t_wd <= e_code;
                t_we(to_integer(t_k(2 downto 0))) <= s_zone;
                e_j <= e_j + 1;
                if e_rep = 1 then e_ph <= "00"; else e_rep <= e_rep - 1; e_ph <= "01"; end if;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
    -- rise, field-latched width)
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
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- ((Y >> 2) - 16), averaged over 8-pixel cells, each cell slewed down the
    -- screen (one pole, 1/16 per line; the state lives in the cell store), the
    -- cell scaled by the camera gain (a serial multiply over the cell's eight
    -- clocks; >> 8 camera, >> 7 Osc = 2x), then slewed across by a one-pole
    -- filter (1/16 per pixel) into eighth-rows.  The filters decay, so none needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(18 downto 0);
        variable v_t : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
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

            -- 1: sample
            if s_src = '1' then
                s_lin <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lin <= v_l - 16; else s_lin <= (others => '0'); end if;
            end if;
            s_lav   <= data_in.avid;
            s_lc    <= s_wcol(2 downto 0);
            s_lcell <= s_wcol(10 downto 3);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= s_wcol(10 downto 3) + 1;
            else
                s_wcol  <= (others => '0');
                s_lb_ra <= (others => '0');
            end if;
            -- 2: sum eight; at the eighth, take the cell's state (read a cell ahead) for the update
            s_f1 <= '0';
            if s_lav = '0' then
                s_cur <= unsigned(s_lb_q(11 downto 0));
            else
                if s_lc = 0 then s_acc <= resize(s_lin, 11); else s_acc <= s_acc + s_lin; end if;
                if s_lc = 7 then
                    s_sum8  <= s_acc + s_lin;
                    s_old   <= s_cur;
                    s_cur   <= unsigned(s_lb_q(11 downto 0));
                    s_fcell <= s_lcell;
                    s_f1    <= '1';
                end if;
            end if;
            -- 3: state += (cell average * 16 - state) / 16
            v_d := signed(resize(s_sum8 & '0', 13)) - signed(resize(s_old, 13));
            v_d := signed(resize(s_old, 13)) + shift_right(v_d, 4);
            s_new   <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);

            -- pixel side: the cell stream (the line above's state) times the gain, serially over the cell
            s_lv <= unsigned(s_lb_q(11 downto 0));
            if s_lav = '1' then
                if s_lc = 0 then
                    s_gpr <= s_gacc;
                    s_ga <= shift_left(resize(s_lv, 20), 1);
                    s_gb <= c_gf8(7 downto 1);
                    if c_gf8(0) = '1' then s_gacc <= resize(s_lv, 20); else s_gacc <= (others => '0'); end if;
                else
                    if s_gb(0) = '1' then s_gacc <= s_gacc + s_ga; end if;
                    s_ga <= shift_left(s_ga, 1);
                    s_gb <= '0' & s_gb(6 downto 1);
                end if;
            end if;
            -- one slew across (1/16 per pixel, Q4)
            if s_src = '1' then v_t := resize(shift_right(s_gpr, 7), 14); else v_t := resize(shift_right(s_gpr, 8), 14); end if;
            v_e  := signed(resize(v_t & "0000", 19)) - signed(resize(s_h1, 19));
            v_e  := signed(resize(s_h1, 19)) + shift_right(v_e, 4);
            s_h1 <= unsigned(v_e(17 downto 0));
            s_lterm <= s_h1(17 downto 4);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line: the V ramp at the line centre (eighth-rows; Mirror folds and
    -- squeezes it), the pixel counter, the table read address (pixels, or the
    -- 16-entry preamble after each line and after the fill)
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                l_vy <= (others => '0');
            elsif g_start = '1' then
                l_r8 <= resize(shift_right(l_vy + resize(shift_right(s_ys, 1), 19), 5), 14);
                l_vy <= l_vy + resize(s_ys, 19);
            end if;
            if s_mir = '1' then
                if l_r8 < 4320 then l_m8 <= l_r8 & '0'; else l_m8 <= (to_unsigned(8640, 14) - l_r8) & '0'; end if;
            else
                l_m8 <= resize(l_r8, 15);
            end if;
            ye5 <= l_m8 + resize(s_lterm, 15);

            if (s_gav_sr(1) = '1' and s_gav_sr(0) = '0') or t_done = '1' then
                pr_act <= '1'; pr_cnt <= (others => '0');
            elsif pr_act = '1' then
                pr_cnt <= pr_cnt + 1;
                if pr_cnt = 31 then pr_act <= '0'; end if;
            end if;
            if g_start = '1' then
                b_px <= (others => '0');
            elsif g_avid = '1' then
                b_px <= b_px + 1;
            end if;
            if g_avid = '1' then
                r_ta <= b_px(10 downto 1) + 16; a_v <= '1'; a_f <= '0';
            elsif pr_act = '1' then
                r_ta <= resize(pr_cnt(4 downto 1), 10); a_v <= '1';
                if pr_cnt = 0 then a_f <= '1'; else a_f <= '0'; end if;
            else
                a_v <= '0'; a_f <= '0';
            end if;
            v1 <= a_v; f1p <= a_f;
            v2 <= v1;  f2p <= f1p;
            v3 <= v2;  f3p <= f2p;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel path per trace: 3 fabric copy of the code, 4 decode, 5 accumulate
    -- (eighth-rows) and the slope-aware soft width, 6 minus the beam, 7 soft
    -- comparator (0 .. 16)
    ---------------------------------------------------------------------------
    g_px : for k in 0 to 7 generate
        signal c3 : std_logic_vector(7 downto 0) := (others => '0');
        signal d4 : signed(10 downto 0) := (others => '0');
        signal l4 : unsigned(3 downto 0) := (others => '0');
        signal y5 : signed(14 downto 0) := (others => '0');
        signal s5, s6 : unsigned(2 downto 0) := (others => '0');
        signal e6 : signed(15 downto 0) := (others => '0');
    begin
        p_px : process(clk)
            variable v_s : unsigned(4 downto 0);
        begin
            if rising_edge(clk) then
                c3 <= r_tq(k);
                if c3(7) = '1' then d4 <= resize(signed(c3(6 downto 0)), 8) & "000";
                else d4 <= resize(signed(c3(6 downto 0)), 11); end if;
                l4 <= f_clen(c3);
                if v3 = '1' then
                    if f3p = '1' then y5 <= resize(d4, 15); else y5 <= y5 + resize(d4, 15); end if;
                end if;
                -- steep run: the slope estimate exceeds the soft width (bands: L > 6; traces: L + 3 > 7)
                if (s_trl = '0' and l4 > 6) or (s_trl = '1' and l4 > 4) then v_s := to_unsigned(3, 5); else v_s := (others => '0'); end if;
                if s_trl = '1' then v_s := v_s + 1; end if;
                s5 <= v_s(2 downto 0);
                e6 <= resize(y5, 16) - signed(resize(ye5, 16));
                s6 <= s5;
                cov7(k) <= f_cov(e6, s6);
            end if;
        end process p_px;
    end generate g_px;

    ---------------------------------------------------------------------------
    -- Resistor ladder (count of traces below the beam: 8 sums, 9 address), the
    -- traces-look brightness (tent cores + glow flags), palette (10 ROM, 11
    -- fabric), 12 traces shading, 13 compose (black outside the generated avid)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n  : unsigned(5 downto 0);
        variable v_j  : unsigned(3 downto 0);
        variable v_sh : integer range 0 to 3;
        variable v_y, v_w : unsigned(9 downto 0);
        variable v_y11 : unsigned(10 downto 0);
        variable v_c  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 7: pairs
            for j in 0 to 3 loop
                s1(j)  <= resize(cov7(2 * j), 6) + resize(cov7(2 * j + 1), 6);
                cp1(j) <= resize(f_core(cov7(2 * j)), 4) + resize(f_core(cov7(2 * j + 1)), 4);
                if cov7(2 * j) >= 1 and cov7(2 * j) <= 15 then
                    if cov7(2 * j + 1) >= 1 and cov7(2 * j + 1) <= 15 then gp1(j) <= "10"; else gp1(j) <= "01"; end if;
                elsif cov7(2 * j + 1) >= 1 and cov7(2 * j + 1) <= 15 then gp1(j) <= "01";
                else gp1(j) <= "00"; end if;
            end loop;
            -- 8: sums
            cnt8 <= resize(s1(0), 8) + resize(s1(1), 8) + resize(s1(2), 8) + resize(s1(3), 8);
            nc8  <= resize(cp1(0), 6) + resize(cp1(1), 6) + resize(cp1(2), 6) + resize(cp1(3), 6);
            ng8  <= resize(gp1(0), 4) + resize(gp1(1), 4) + resize(gp1(2), 4) + resize(gp1(3), 4);
            -- 9: palette address (band rotated by K5; bands: + the count's fraction), brightness
            v_j := cnt8(7 downto 4) + c_r;
            if s_trl = '1' then pal_ra <= s_pal & v_j & "0000";
            else pal_ra <= s_pal & v_j & cnt8(3 downto 0); end if;
            v_n := shift_left(nc8, 2) + resize(ng8, 6);
            if nc8 > 3 or v_n > 15 then b9 <= "1111"; else b9 <= v_n(3 downto 0); end if;
            -- 10, 11
            b10 <= b9; b11 <= b10;
            pal_r <= pal_q;
            -- 12: traces shading: colour x 1/8 .. 1 by brightness, a white core above 8
            v_y := pal_r(23 downto 16) & "00";
            if s_trl = '0' then
                y12 <= v_y; u12 <= pal_r(15 downto 8) & "00"; v12 <= pal_r(7 downto 0) & "00";
            elsif b11 = 0 then
                y12 <= C_KY; u12 <= C_KC; v12 <= C_KC;
            else
                if b11 >= 8 then v_sh := 0; elsif b11 >= 4 then v_sh := 1; elsif b11 >= 2 then v_sh := 2; else v_sh := 3; end if;
                if b11 >= 8 then v_w := resize(shift_left(resize(b11(2 downto 0), 10), 5) + shift_left(resize(b11(2 downto 0), 10), 4), 10);
                else v_w := (others => '0'); end if;
                v_y11 := 64 + resize(shift_right(v_y - 64, v_sh), 11) + resize(v_w, 11);
                if v_y11 > 940 then y12 <= to_unsigned(940, 10); else y12 <= v_y11(9 downto 0); end if;
                v_c := signed(resize(pal_r(15 downto 8) & "00", 11)) - 512;
                v_c := shift_right(v_c, v_sh) + 512;
                u12 <= unsigned(v_c(9 downto 0));
                v_c := signed(resize(pal_r(7 downto 0) & "00", 11)) - 512;
                v_c := shift_right(v_c, v_sh) + 512;
                v12 <= unsigned(v_c(9 downto 0));
            end if;
            -- 13: compose
            if s_gav_sr(12) = '0' then
                s_y15 <= C_KY; s_u15 <= C_KC; s_v15 <= C_KC;
            else
                s_y15 <= y12; s_u15 <= u12; s_v15 <= v12;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix), luma 13/16 and the
    -- blanking gate.  The running sums snap back to black once 8 blank samples
    -- are in (resync).
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_wp <= r_wp + 1;
            ra_ram(to_integer(r_wp)) <= std_logic_vector(s_u15(9 downto 2)) & std_logic_vector(s_v15(9 downto 2));
            ra_q <= ra_ram(to_integer(r_wp - 6));
            rb_ram(to_integer(r_wp)) <= std_logic_vector(s_u17(9 downto 2)) & std_logic_vector(s_v17(9 downto 2));
            rb_q <= rb_ram(to_integer(r_wp - 6));
        end if;
    end process p_ring;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_s2 : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            ra_r <= ra_q; rb_r <= rb_q;
            -- box A: luma 10-bit (FF delay line), chroma 8-bit (ring)
            s_cay(0) <= s_y15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= to_unsigned(1024, 11); s_bav <= to_unsigned(1024, 11);
            else
                s_bay <= s_bay + s_y15 - s_cay(7);
                s_bau <= s_bau + s_u15(9 downto 2) - unsigned(ra_r(15 downto 8));
                s_bav <= s_bav + s_v15(9 downto 2) - unsigned(ra_r(7 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_s2 := resize(s_bau, 12) + s_baud; s_u17 <= v_s2(11 downto 2);
                v_s2 := resize(s_bav, 12) + s_bavd; s_v17 <= v_s2(11 downto 2);
            else
                s_y17 <= C_KY; s_u17 <= C_KC; s_v17 <= C_KC;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B
            s_cby(0) <= s_y17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= to_unsigned(1024, 11); s_bbv <= to_unsigned(1024, 11);
            else
                s_bby <= s_bby + s_y17 - s_cby(7);
                s_bbu <= s_bbu + s_u17(9 downto 2) - unsigned(rb_r(15 downto 8));
                s_bbv <= s_bbv + s_v17(9 downto 2) - unsigned(rb_r(7 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
            v_s2 := resize(s_bbu, 12) + s_bbud; s_u19 <= v_s2(11 downto 2);
            v_s2 := resize(s_bbv, 12) + s_bbvd; s_v19 <= v_s2(11 downto 2);
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ringdown;
