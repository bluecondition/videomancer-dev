-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Pulsar v0.1.0 -- Patchbay: stacked scope traces with hidden lines (the PSR B1919+21 ridgeline plot).
--
-- The patch:
--   H ramp -> VCO 1 + VCO 2 -> mixer -> waveshaper --\
--   noise -> AND -> slew ---------------------------- crossfader (Rough) -> VCA <- burst envelope (window on the H ramp)
--   VCA x BURST + camera luma along the trace's own row -> VCA (Height) = the trace's height h_k(x)
--   V ramp + baselines y_k -> one comparator per trace -> priority chain (nearest wins) -> edge differentiator
--   height above the baseline -> colouriser
--
-- Each trace is 241 samples (8 output pixels apart), kept as its first sample and 240 slopes in one of 8
-- rotating 256 x 16 block RAMs.  All gains are
-- kept as octaves: a sample is EXPU(L[waveform] + L[envelope] + C) + EXPU(L[video] + CV), where L is a log ROM,
-- EXPU an exp2 ROM plus a shift, and C the trace's attenuation (Height, BURST, pitch), so the fill engine and
-- the sequencer that sets each trace up need no multipliers (one small serial one for three products a field).
-- A row can be crossed by the past trace and the six coming ones: seven accumulators run e = h(x) - (y_k - y)
-- in parallel (one adder each: the slope of the current 8-pixel cell is added every pixel), the nearest trace
-- with e > -R wins, and the winner's e gives the line's anti-aliased coverage and the Glow ramp.  The
-- baselines travel down the screen (the peaks come toward the viewer); in Perspective the pitch grows
-- geometrically below a horizon and the heights with it.
--
--   K1  Lines     16 .. 80 traces
--   K2  Height    the tallest peak against the 6-pitch window
--   K3  Envelope  burst width: a narrow central ridge .. the full line
--   K4  Speed     bipolar travel (deadband), x up to 3.5 with BURST
--   K5  Rough     VCO swells .. jagged noise
--   K6  Colour    white .. colour by height (centre) .. colour by trace
--   S7  Fill      Black (hidden line) / Glow (a ramp under each trace)
--   S8  Burst     One / Twin
--   S9  Tilt      Flat / Perspective
--   S10 Paper     Off / On (ink on paper)
--   S11 Source    Video / Osc
--   P12 BURST     flat ruled lines .. towering overlapping peaks racing forward
--
-- Shell: virtual 1080-line coordinates, vblank zone, generated avid, one latency (24), output conditioning
-- (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16 with a per-line resync, luma 13/16), blanking gate.  Colours in
-- the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.pulsar_pkg.all;
use work.all;

architecture pulsar of program_top is
    constant C_LAT  : integer := 24;   -- = processing_delay_clks
    constant C_GLIDE : boolean := true;  -- false in the sim-only copy: knobs snap

    constant C_GM   : integer := 2298479;   -- identity accumulator step (26 bits; 0.137 cycle of VCO 1 per trace)
    constant C_WTB  : integer := 77;        -- per-field drift: sway x1, VCO 1 x2, VCO 2 x-4

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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    function f_top1(v : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable r : std_logic_vector(7 downto 0) := (others => '0');
        variable s : std_logic := '0';
    begin
        for i in 7 downto 0 loop
            r(i) := v(i) and not s;
            s := s or v(i);
        end loop;
        return r;
    end function;
    function f_sat16(v : unsigned) return unsigned is
    begin
        if v(v'high downto 16) /= 0 then return to_unsigned(65535, 16); else return v(15 downto 0); end if;
    end function;

    ---------------------------------------------------------------------------
    -- memories
    ---------------------------------------------------------------------------
    -- luma store: 64 rows x 32 columns (30 used) x 8 bits; written by the source side during active video,
    -- read by the fill engine for a row at least a store row below the beam (or in vblank): no collisions
    type t_vs is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal vs : t_vs := (others => x"00");
    attribute no_rw_check : boolean;
    attribute no_rw_check of vs : signal is true;
    signal v_we : std_logic := '0';
    signal v_wa, r_vs_a : unsigned(10 downto 0) := (others => '0');
    signal v_wd, r_vs_q : std_logic_vector(7 downto 0) := x"00";

    -- per-bank trace constants (written by the sequencer, read by the line engine; a bank's words are only
    -- used once its fill is done): word 0 = not baseline (Q.4 rows), word 1 = {10240 / pitch (10), colour offset (6)}
    type t_rf is array (0 to 255) of std_logic_vector(15 downto 0);
    signal rf : t_rf := (others => x"0000");
    attribute no_rw_check of rf : signal is true;
    signal rf_we : std_logic := '0';
    signal rf_wa, rf_ra : unsigned(3 downto 0) := (others => '0');
    signal rf_wd, rf_q : std_logic_vector(15 downto 0) := x"0000";

    -- ROMs (registered address and data)
    signal r_ex_a, r_lg_a, r_ev_a, r_s1_a, r_sh_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_lg_q, r_le_q : unsigned(15 downto 0) := (others => '0');
    signal r_s1_q : signed(7 downto 0) := (others => '0');
    signal r_sh_q : unsigned(7 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- exponential unit: x_v = 32768 * 2^(-t / 1024), four clocks after t
    signal q_t, f_t : unsigned(15 downto 0) := (others => '1');
    signal x_sh1, x_sh2 : unsigned(6 downto 0) := (others => '0');
    signal x_sh3 : unsigned(4 downto 0) := (others => '0');
    signal x_m3, x_v : unsigned(15 downto 0) := (others => '0');

    -- trace banks: words 0..239 = slope of each 8-pixel cell, 254 = first sample, 255 = the line's X (line engine)
    signal bw_we : std_logic_vector(7 downto 0) := (others => '0');
    signal bw_a  : unsigned(7 downto 0) := (others => '0');
    signal bw_d  : std_logic_vector(15 downto 0) := (others => '0');
    signal p_ra  : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- shell: timing / zone / width / knobs
    ---------------------------------------------------------------------------
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_rs   : unsigned(1 downto 0) := "00";                     -- AA range: 1 / 2 / 4 rows
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    signal s_g1 : unsigned(12 downto 0) := to_unsigned(620 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(880 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(430 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(450 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(780 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vpos : std_logic := '0';
    signal s_glow, s_twin, s_tilt, s_paper, s_src : std_logic := '0';

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- store write side
    signal w_lum  : unsigned(7 downto 0) := (others => '0');
    signal w_acc  : unsigned(12 downto 0) := (others => '0');
    signal w_ph   : unsigned(6 downto 0) := (others => '0');
    signal w_c1   : unsigned(5 downto 0) := (others => '0');
    signal w_row  : unsigned(6 downto 0) := (others => '0');
    signal w_racc : unsigned(11 downto 0) := (others => '0');
    signal w_samp, w_av1, w_av2, w_stb : std_logic := '0';
    signal w_col  : unsigned(4 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    ---------------------------------------------------------------------------
    -- sequencer
    ---------------------------------------------------------------------------
    signal q_st  : std_logic_vector(1 downto 0) := "00";              -- 00 idle, 01 steps, 10 fill
    signal q_s   : unsigned(6 downto 0) := (others => '0');
    signal q_first : std_logic := '0';
    signal q_need : unsigned(3 downto 0) := (others => '0');
    signal q_fp  : unsigned(2 downto 0) := (others => '0');
    signal q_la  : unsigned(7 downto 0) := (others => '0');
    signal m_a, m_acc : unsigned(31 downto 0) := (others => '0');
    signal m_b   : unsigned(15 downto 0) := (others => '0');
    signal b_rdy : std_logic_vector(7 downto 0) := (others => '0');

    signal z_tp  : unsigned(12 downto 0) := (others => '0');
    signal z_lv  : unsigned(15 downto 0) := (others => '0');
    signal z_np  : unsigned(11 downto 0) := (others => '0');
    signal z_tdz : unsigned(15 downto 0) := (others => '0');
    signal z_wrap : std_logic_vector(1 downto 0) := "00";
    signal z_pm1 : unsigned(15 downto 0) := (others => '0');
    signal z_tk  : unsigned(15 downto 0) := (others => '0');
    signal z_ca  : unsigned(15 downto 0) := (others => '0');
    signal z_if  : unsigned(9 downto 0) := (others => '0');
    signal z_cm  : unsigned(5 downto 0) := (others => '0');
    signal z_sm  : unsigned(23 downto 0) := (others => '0');

    -- per-field constants
    signal c_eo, c_es, c_dz, c_cpb : unsigned(15 downto 0) := (others => '0');
    signal c_r3   : unsigned(3 downto 0) := (others => '0');
    signal c_sat2 : unsigned(1 downto 0) := "11";
    signal c_m4   : unsigned(2 downto 0) := "000";
    signal c_ce   : unsigned(12 downto 0) := (others => '0');
    signal c_o1, c_o2 : unsigned(15 downto 0) := (others => '0');
    signal c_f1c, c_f2c : unsigned(15 downto 0) := (others => '0');
    signal c_tes  : unsigned(10 downto 0) := (others => '0');
    signal c_cv   : unsigned(10 downto 0) := (others => '0');
    signal c_ic   : unsigned(7 downto 0) := to_unsigned(120, 8);
    signal c_nc   : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal c_fend : unsigned(9 downto 0) := to_unsigned(533, 10);
    signal c_pos0 : signed(11 downto 0) := to_signed(-40, 12);
    signal c_bgy, c_bgu, c_bgv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- motion state
    signal s_fz   : unsigned(15 downto 0) := (others => '0');
    signal s_m    : unsigned(25 downto 0) := (others => '0');          -- identity accumulator of trace -1
    signal s_tb   : unsigned(15 downto 0) := (others => '0');          -- drift accumulator

    -- trace state
    signal t_a    : signed(22 downto 0) := (others => '0');            -- pitch exponent, Q.16 octaves
    signal t_y    : signed(20 downto 0) := (others => '0');            -- baseline, Q8 rows
    signal t_m    : unsigned(25 downto 0) := (others => '0');
    signal t_pk   : unsigned(15 downto 0) := (others => '0');
    signal t_yb   : signed(15 downto 0) := (others => '0');
    signal t_rowv : std_logic := '0';
    signal t_row  : unsigned(5 downto 0) := (others => '0');
    signal t_ca, t_cv : unsigned(15 downto 0) := (others => '1');
    signal t_cit  : unsigned(5 downto 0) := (others => '0');
    signal t_psi  : unsigned(15 downto 0) := (others => '0');
    signal t_ps1, t_ps2 : unsigned(15 downto 0) := (others => '0');
    signal t_en   : signed(27 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- fill engine
    ---------------------------------------------------------------------------
    signal f_go, f_busy, f_done : std_logic := '0';
    signal f_c   : unsigned(9 downto 0) := (others => '0');
    signal f_p1, f_p2 : unsigned(15 downto 0) := (others => '0');
    signal f_en  : signed(27 downto 0) := (others => '0');
    signal f_x   : unsigned(23 downto 0) := (others => '0');
    signal f_pos : signed(11 downto 0) := (others => '0');
    signal f_s1f : signed(7 downto 0) := (others => '0');
    signal f_a5  : unsigned(10 downto 0) := (others => '0');
    signal f_shf : unsigned(7 downto 0) := (others => '0');
    signal f_wn, f_sl, f_nz : unsigned(7 downto 0) := (others => '0');
    signal f_mp  : signed(11 downto 0) := (others => '0');
    signal f_w   : unsigned(7 downto 0) := (others => '0');
    signal f_lg, f_le : unsigned(15 downto 0) := (others => '0');
    signal f_ec  : unsigned(16 downto 0) := (others => '0');
    signal f_h1  : unsigned(13 downto 0) := (others => '0');
    signal f_hs, f_hp : unsigned(12 downto 0) := (others => '0');
    signal f_clip : unsigned(12 downto 0) := (others => '0');
    signal f_vsf : unsigned(7 downto 0) := (others => '0');
    signal f_sm  : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- line engine
    ---------------------------------------------------------------------------
    signal e_vy  : unsigned(18 downto 0) := (others => '0');
    signal l_zk, l_done, l_first : std_logic := '0';
    signal l_c   : unsigned(2 downto 0) := (others => '0');
    signal l_run, l_pre : std_logic := '0';
    signal l_s   : unsigned(3 downto 0) := (others => '0');
    signal l_bi  : unsigned(2 downto 0) := (others => '0');
    signal l_rq  : std_logic_vector(15 downto 0) := (others => '0');
    signal l_vyr : signed(15 downto 0) := (others => '0');
    signal l_rdy : std_logic_vector(7 downto 0) := (others => '0');
    signal l_adv : std_logic := '0';
    signal l_hdn : unsigned(2 downto 0) := (others => '0');
    signal l_x   : signed(15 downto 0) := (others => '0');
    signal l_cmr : unsigned(5 downto 0) := (others => '0');
    signal l_ma, l_acc : unsigned(20 downto 0) := (others => '0');
    signal l_mb  : unsigned(10 downto 0) := (others => '0');
    signal l_csl : unsigned(10 downto 0) := (others => '0');
    type t_u5 is array (0 to 7) of unsigned(4 downto 0);
    signal l_cin : t_u5 := (others => (others => '0'));
    signal l_rel, l_xw : std_logic := '0';
    signal l_gsh : unsigned(3 downto 0) := (others => '1');
    signal l_gb  : std_logic := '0';

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    signal p_x   : unsigned(10 downto 0) := (others => '0');
    signal p_pr  : unsigned(4 downto 0) := (others => '0');            -- pre-roll counter
    signal p_dz, p_eclr : std_logic := '0';
    signal p_hd  : unsigned(2 downto 0) := (others => '0');
    signal p_act, p_lom, p_msk : std_logic_vector(7 downto 0) := (others => '0');
    signal p_ci  : t_u5 := (others => (others => '0'));
    signal p_gsh : unsigned(3 downto 0) := (others => '1');
    signal p_gb  : std_logic := '0';
    type t_s16 is array (0 to 7) of signed(15 downto 0);
    type t_s20 is array (0 to 7) of signed(19 downto 0);
    type t_u16 is array (0 to 7) of unsigned(15 downto 0);
    signal x_d   : t_s16 := (others => (others => '0'));
    signal x_ea  : t_s20 := (others => (others => '0'));
    signal x_e6  : t_u16 := (others => (others => '0'));
    signal p6_sel : std_logic_vector(7 downto 0) := (others => '0');
    signal p6_cv, p8_cv, p9_cv, p10_cv, p11_cv : std_logic := '0';
    signal p8_e  : unsigned(15 downto 0) := (others => '0');
    signal p8_d  : signed(13 downto 0) := (others => '0');
    signal p8_ci, p9_ci, p10_ci, p11_ci, p12_ci, p13_ci : unsigned(4 downto 0) := (others => '0');
    signal p9_e, p10_e : unsigned(13 downto 0) := (others => '0');
    signal p9_ad : unsigned(13 downto 0) := (others => '0');
    signal p9_er : unsigned(6 downto 0) := (others => '0');
    signal p10_t : unsigned(11 downto 0) := (others => '0');
    signal p10_gd : unsigned(8 downto 0) := (others => '0');
    signal p11_mn : signed(15 downto 0) := (others => '0');
    signal p11_gl : unsigned(4 downto 0) := (others => '0');
    signal p12_l, p12b_l, p12c_l, p12d_l, p13_l : unsigned(4 downto 0) := (others => '0');
    signal p13_q : std_logic_vector(31 downto 0) := (others => '0');
    signal p14_ay, p14_by : signed(12 downto 0) := (others => '0');
    signal p14_au, p14_bu, p14_av, p14_bv : signed(9 downto 0) := (others => '0');
    signal p15_my : signed(15 downto 0) := (others => '0');
    signal p15_mu, p15_mv : signed(12 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    -- RAM delays: the chroma sums' 8-back taps (8-bit u, v) and the syncs
    type t_dc is array (0 to 255) of std_logic_vector(15 downto 0);
    signal dc : t_dc := (others => x"8080");
    attribute no_rw_check of dc : signal is true;
    type t_ds is array (0 to 255) of std_logic_vector(3 downto 0);
    signal ds : t_ds := (others => x"7");
    attribute no_rw_check of ds : signal is true;
    signal d_w, d_rc, d_rs : unsigned(7 downto 0) := (others => '0');
    signal d_cq, d_cf : std_logic_vector(15 downto 0) := x"8080";
    signal d_sq, d_so : std_logic_vector(3 downto 0) := x"7";
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y18, s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_lf  : t_u10(19 to 23) := (others => to_unsigned(64, 10));
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories: luma store, trace-constant file, ROMs; the exponential unit
    ---------------------------------------------------------------------------
    p_mem : process(clk)
        variable v_t  : unsigned(15 downto 0);
        variable v_sh : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            r_vs_q <= vs(to_integer(r_vs_a));
            if v_we = '1' then vs(to_integer(v_wa)) <= v_wd; end if;
            rf_q <= rf(to_integer(rf_ra));
            if rf_we = '1' then rf(to_integer(rf_wa)) <= rf_wd; end if;
            r_ex_q <= C_PU_EXP2(to_integer(r_ex_a));
            r_lg_q <= C_PU_LOG(to_integer(r_lg_a));
            r_le_q <= C_PU_LENV(to_integer(r_ev_a));
            r_s1_q <= C_PU_SIN(to_integer(r_s1_a));
            r_sh_q <= C_PU_SHP(to_integer(r_sh_a));
            r_pl_q <= C_PU_PAL(to_integer(r_pl_a));

            -- EXPU: mantissa = EXP2[(-t / 4) mod 256], shifted right by the whole octaves (+ 1 if a fraction)
            if f_busy = '1' then v_t := f_t; else v_t := q_t; end if;
            r_ex_a <= to_unsigned(0, 8) - v_t(9 downto 2);
            v_sh := resize(v_t(15 downto 10), 7);
            if v_t(9 downto 2) /= 0 then v_sh := v_sh + 1; end if;
            x_sh1 <= v_sh;
            x_sh2 <= x_sh1;
            x_m3 <= shift_right(r_ex_q, to_integer(x_sh2(1 downto 0)));
            x_sh3 <= x_sh2(6 downto 2);
            if x_sh3(4 downto 2) /= 0 then
                x_v <= (others => '0');
            else
                case x_sh3(1 downto 0) is
                    when "00"   => x_v <= x_m3;
                    when "01"   => x_v <= "0000" & x_m3(15 downto 4);
                    when "10"   => x_v <= x"00" & x_m3(15 downto 8);
                    when others => x_v <= x"000" & x_m3(15 downto 12);
                end case;
            end if;
        end if;
    end process p_mem;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glide, sync delays
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            if s_ys <= 384 then s_rs <= "00"; elsif s_ys <= 576 then s_rs <= "01"; else s_rs <= "10"; end if;

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            if v_s > 0 then s_vpos <= '1'; else s_vpos <= '0'; end if;
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
                s_glow  <= registers_in(6)(0);
                s_twin  <= registers_in(6)(1);
                s_tilt  <= registers_in(6)(2);
                s_paper <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
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
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if not C_GLIDE then
                            null;
                        elsif s_gd > 16 or s_gd < -16 then
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
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Sequencer: one small operation per step.  Steps 0..96 run once per field from the zone start (the first
    -- 32 let the glide finish): knob exponents, three EXPU look-ups, the motion, two serial products.  Steps
    -- 104..113 set one trace up (three look-ups, a few adds) and start its fill; they repeat while a bank is
    -- waiting (eight times in vblank, then once per baseline the beam passes).
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_k1, v_k2, v_k3, v_k5, v_k6, v_kp : unsigned(9 downto 0);
        variable v_n   : unsigned(9 downto 0);
        variable v17   : unsigned(16 downto 0);
        variable v18   : unsigned(17 downto 0);
        variable v_tk  : unsigned(15 downto 0);
        variable v_s18 : signed(17 downto 0);
        variable v_y   : signed(16 downto 0);
        variable v_c7  : unsigned(6 downto 0);
        variable v_inc, v_dec : std_logic;
        variable v_c3  : unsigned(7 downto 0);
        variable v_g   : unsigned(25 downto 0);
        variable v_sw  : signed(11 downto 0);
        variable v_ta_ld, v_ta_en, v_ta_sub, v_ty_ld, v_ty_en : std_logic;
        variable v_ta_op : unsigned(15 downto 0);
        variable v_b24, v_s24 : unsigned(23 downto 0);
        variable v_a22, v_b22, v_s22 : unsigned(21 downto 0);
    begin
        if rising_edge(clk) then
            v_k1 := s_g1(12 downto 3); v_k2 := s_g2(12 downto 3); v_k3 := s_g3(12 downto 3);
            v_k5 := s_g5(12 downto 3); v_k6 := s_g6(12 downto 3); v_kp := s_gp(12 downto 3);
            f_go <= '0'; l_zk <= '0'; rf_we <= '0';
            v_inc := l_rel; v_dec := '0';

            -- serial multiplier (16 steps after a load)
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            v_ta_ld := '0'; v_ta_en := '0'; v_ta_sub := '0'; v_ta_op := c_eo;
            v_ty_ld := '0'; v_ty_en := '0';
            if q_st = "01" then
                q_s <= q_s + 1;
                case to_integer(q_s) is
                    when 32 => q_la <= v_kp(9 downto 2);                              -- BURST level look-up
                    when 36 =>
                        v17 := resize(f_lg, 17) + resize(f_lg(15 downto 2), 17);
                        c_cpb <= f_sat16(v17);
                        q_la <= s_vm(8 downto 1);
                    when 40 =>
                        z_lv <= f_lg;
                        -- K1: 2.375 octaves of pitch, in 1/1024 octave
                        z_tp <= resize(v_k1 & '0', 13) + resize(v_k1(9 downto 2), 13) + resize(v_k1(9 downto 3), 13) + 945;
                        v_n := not v_kp;
                        z_np <= resize(v_n & '0', 12) - resize(v_n(9 downto 2), 12);
                        v17 := resize(v_k5, 17) + 64; c_r3 <= v17(10 downto 7);
                        if v_k6(9 downto 7) >= 3 then c_sat2 <= "11"; else c_sat2 <= v_k6(8 downto 7); end if;
                        if v_k6 < 608 then c_m4 <= "000";
                        elsif v_k6 < 736 then c_m4 <= "001";
                        elsif v_k6 < 864 then c_m4 <= "010";
                        elsif v_k6 < 992 then c_m4 <= "011";
                        else c_m4 <= "100"; end if;
                    when 41 =>
                        v18 := resize(z_lv & '0', 18) + resize(z_np, 18) + 180;
                        z_tdz <= f_sat16(v18);
                    when 51 => q_t <= resize(z_tp, 16) + 724;                         -- octaves per trace (tilt)
                    when 52 => q_t <= resize(v_k3 & "00", 16) + resize(v_k3, 16) + resize(c_tes, 16);   -- envelope step
                    when 53 => q_t <= z_tdz;                                          -- travel
                    when 56 => if s_tilt = '1' then c_eo <= x_v; else c_eo <= (others => '0'); end if;
                    when 57 => c_es <= x_v;
                    when 58 => c_dz <= x_v;
                    when 59 =>
                        if s_vpos = '1' then
                            v17 := resize(s_fz, 17) + resize(c_dz, 17);
                            if v17(16) = '1' then z_wrap <= "01"; else z_wrap <= "00"; end if;
                        else
                            v17 := resize(s_fz, 17) - resize(c_dz, 17);
                            if v17(16) = '1' then z_wrap <= "10"; else z_wrap <= "00"; end if;
                        end if;
                        s_fz <= v17(15 downto 0);
                    when 60 =>
                        if z_wrap = "01" then v_g := to_unsigned(67108864 - C_GM, 26); else v_g := to_unsigned(C_GM, 26); end if;
                        if z_wrap /= "00" then s_m <= s_m + v_g; end if;
                        s_tb <= s_tb + to_unsigned(C_WTB, 16);
                        v_ta_ld := '1';                                               -- pitch exponent at k + fz = 0
                    when 61 =>
                        v_ta_en := '1';                                               -- + one trace: trace -1 at fz = 0
                        m_a <= resize(c_eo, 32); m_b <= s_fz; m_acc <= (others => '0');
                        v_n := not v_k2;
                        c_ce <= resize(v_n & '0', 13) + resize(v_n(9 downto 1), 13) + resize(c_eo(15 downto 4), 13) - resize(c_eo(15 downto 7), 13);
                    when 62 => q_t <= unsigned(t_a(21 downto 6));                     -- its pitch
                    when 67 => z_pm1 <= x_v;
                    when 78 =>
                        v_ta_en := '1'; v_ta_sub := '1'; v_ta_op := m_acc(31 downto 16);    -- - fz x octaves per trace
                        m_a <= resize(z_pm1, 32); m_b <= not s_fz; m_acc <= (others => '0');
                    when 95 => v_ty_ld := '1';                                        -- baseline of trace -1
                    when 96 =>
                        t_m <= s_m;
                        q_s <= to_unsigned(104, 7);

                    -- one trace
                    when 104 =>
                        if t_a(22) = '1' then v_tk := (others => '0'); else v_tk := unsigned(t_a(21 downto 6)); end if;
                        q_t <= v_tk; z_tk <= v_tk;                                    -- pitch
                    when 105 =>
                        if z_tk > 10933 then q_t <= (others => '0'); else q_t <= to_unsigned(10933, 16) - z_tk; end if;   -- 10240 / pitch
                        v_s18 := signed(resize(c_ce, 18)) + signed(resize(z_tk, 18)) - 2647;
                        if v_s18(17) = '1' then z_ca <= (others => '0'); else z_ca <= unsigned(v_s18(15 downto 0)); end if;
                        if t_y(20) = '0' and t_y(19) = '1' then t_yb <= to_signed(32767, 16);
                        else t_yb <= t_y(19 downto 4); end if;
                    when 106 =>
                        q_t <= z_ca;                                                  -- window (clip)
                        t_ca <= f_sat16(resize(z_ca, 17) + resize(c_cpb, 17));
                        t_cv <= f_sat16(resize(z_ca, 17) + resize(c_cv, 17));
                        rf_we <= '1'; rf_wa <= q_fp & '0'; rf_wd <= std_logic_vector(not t_yb);
                        if t_yb >= 0 and t_yb < 17280 then t_rowv <= '1'; else t_rowv <= '0'; end if;
                        if t_yb(14) = '1' then t_row <= (others => '1'); else t_row <= unsigned(t_yb(13 downto 8)); end if;
                        t_psi <= t_m(25 downto 10) + s_tb;
                        v_c7 := t_m(25 downto 19);
                        if v_c7(6) = '1' then t_cit <= not v_c7(5 downto 0); else t_cit <= v_c7(5 downto 0); end if;
                    when 109 => t_pk <= x_v;
                    when 110 =>
                        if x_v(15 downto 10) /= 0 then z_if <= (others => '1'); else z_if <= x_v(9 downto 0); end if;
                        case to_integer(c_m4) is
                            when 0      => z_cm <= (others => '0');
                            when 1      => z_cm <= "00" & t_cit(5 downto 2);
                            when 2      => z_cm <= '0' & t_cit(5 downto 1);
                            when 3      => v_c3 := resize(t_cit, 8) + resize(t_cit & '0', 8); z_cm <= v_c3(7 downto 2);
                            when others => z_cm <= t_cit;
                        end case;
                    when 111 =>
                        if x_v(15) = '1' then f_clip <= (others => '1'); else f_clip <= x_v(14 downto 2); end if;
                        rf_we <= '1'; rf_wa <= q_fp & '1'; rf_wd <= std_logic_vector(z_if) & std_logic_vector(z_cm);
                        t_ps1 <= t_m(23 downto 8) + (s_tb(14 downto 0) & '0') - c_o1;
                        t_ps2 <= to_unsigned(0, 16) - (s_tb(13 downto 0) & "00") - t_m(22 downto 7) - c_o2;
                        z_sm <= t_m(23 downto 0) xor resize(t_m(23 downto 11), 24);
                        v_sw := to_signed(1024, 12) - resize(f_s1f, 12);              -- envelope centre 1/2 - sway
                        t_en <= signed(resize(unsigned(v_sw), 19)) & "000000000";
                    when 112 =>
                        if c_nc(7) = '1' then t_en <= t_en - signed(resize(c_es & "000000000", 28));
                        else t_en <= t_en - signed(resize(c_es & "00000000", 28)); end if;
                        z_sm <= z_sm + (z_sm(18 downto 0) & "00000");
                    when 113 =>
                        f_go <= '1';
                        v_ta_en := '1'; v_ta_sub := '1';
                        v_ty_en := '1';
                        t_m <= t_m + to_unsigned(C_GM, 26);
                        q_st <= "10";
                    when others => null;
                end case;
            elsif q_st = "10" then
                if f_done = '1' then
                    b_rdy(to_integer(q_fp)) <= '1';
                    q_fp <= q_fp + 1;
                    v_dec := '1';
                    q_st <= "00";
                end if;
            else
                if q_need /= 0 then
                    q_st <= "01"; q_s <= to_unsigned(104, 7);
                elsif q_first = '1' then
                    q_first <= '0'; l_zk <= '1';
                end if;
            end if;

            -- pitch exponent: load / + op / - op on one adder (the sign rides in as the carry)
            if v_ta_ld = '1' then
                if s_tilt = '1' then t_a <= signed(resize((resize(z_tp, 14) + 3565) & "000000", 23));
                else t_a <= signed(resize((resize(z_tp, 14) + 2048) & "000000", 23)); end if;
            elsif v_ta_en = '1' then
                v_b24 := resize(v_ta_op, 23) & v_ta_sub;
                if v_ta_sub = '1' then v_b24(23 downto 1) := not v_b24(23 downto 1); end if;
                v_s24 := (unsigned(t_a) & '1') + v_b24;
                t_a <= signed(v_s24(23 downto 1));
            end if;
            -- baseline: load -(pitch x (1 - fz)) / + pitch
            if v_ty_ld = '1' or v_ty_en = '1' then
                if v_ty_ld = '1' then
                    v_a22 := (others => '0'); v_a22(0) := '1';
                    v_b22 := (not resize(m_acc(31 downto 14), 21)) & '1';
                else
                    v_a22 := unsigned(t_y) & '1';     -- (the carry bit: 1 + 0 here, 1 + 1 for the load)
                    v_b22 := resize(t_pk & "00", 21) & '0';
                end if;
                v_s22 := v_a22 + v_b22;
                t_y <= signed(v_s22(21 downto 1));
            end if;

            if l_rel = '1' then b_rdy(to_integer(p_hd - 1)) <= '0'; end if;
            if v_inc = '1' and v_dec = '0' then q_need <= q_need + 1;
            elsif v_dec = '1' and v_inc = '0' then q_need <= q_need - 1; end if;

            -- per-resolution constants (the sample grid is 8 output pixels; c_o = centre sample x phase step)
            case to_integer(s_xs) is
                when 4      => c_f1c <= to_unsigned(3495, 16); c_f2c <= to_unsigned(9279, 16);
                               c_o1 <= to_unsigned((3495 * 64) mod 65536, 16); c_o2 <= to_unsigned((9279 * 64) mod 65536, 16);
                               c_ic <= to_unsigned(45, 8); c_nc <= to_unsigned(64, 8); c_fend <= to_unsigned(277, 10);
                               c_pos0 <= to_signed(-40, 12); c_tes <= to_unsigned(800, 11);
                when 3      => c_f1c <= to_unsigned(2621, 16); c_f2c <= to_unsigned(6960, 16);
                               c_o1 <= to_unsigned((2621 * 128) mod 65536, 16); c_o2 <= to_unsigned((6960 * 128) mod 65536, 16);
                               c_ic <= to_unsigned(80, 8); c_nc <= to_unsigned(128, 8); c_fend <= to_unsigned(533, 10);
                               c_pos0 <= to_signed(-296, 12); c_tes <= to_unsigned(1225, 11);
                when others => c_f1c <= to_unsigned(1748, 16); c_f2c <= to_unsigned(4640, 16);
                               c_o1 <= to_unsigned((1748 * 128) mod 65536, 16); c_o2 <= to_unsigned((4640 * 128) mod 65536, 16);
                               c_ic <= to_unsigned(120, 8); c_nc <= to_unsigned(128, 8); c_fend <= to_unsigned(533, 10);
                               c_pos0 <= to_signed(-40, 12); c_tes <= to_unsigned(1824, 11);
            end case;
            if s_src = '1' then c_cv <= to_unsigned(425, 11); else c_cv <= to_unsigned(1449, 11); end if;
            if s_paper = '1' then
                c_bgy <= unsigned(C_PU_PAPER(29 downto 20)); c_bgu <= unsigned(C_PU_PAPER(19 downto 10)); c_bgv <= unsigned(C_PU_PAPER(9 downto 0));
            else
                c_bgy <= unsigned(C_PU_BLACK(29 downto 20)); c_bgu <= unsigned(C_PU_BLACK(19 downto 10)); c_bgv <= unsigned(C_PU_BLACK(9 downto 0));
            end if;

            -- field start: every field restarts the engine from step 0 (also the power-up start)
            if s_zgo = '1' then
                q_st <= "01"; q_s <= (others => '0');
                q_first <= '1'; q_need <= to_unsigned(8, 4); q_fp <= (others => '0');
                b_rdy <= (others => '0');
            end if;
        end if;
    end process p_seq;

    ---------------------------------------------------------------------------
    -- Fill engine: two clocks per sample (256 samples; 128 at SD), no multipliers.  Clock c of a sample:
    --   0 VCO 1 address | 1 VCO 2 address | 2, 3 sine ROM | 4 mix -> shaper address | 6 shaper ROM |
    --   7, 8 crossfader with the noise | 9 log address (waveform) | 10 log address (video) |
    --   12 t = L[W] + L[env] + CA | 13 t = L[video] + CV | 17, 18 EXPU | 18 sum, clip | 19 write
    -- The noise, video and envelope chains start 4, 4 and 8 clocks after the VCOs so each meets its stage.
    -- The bank write bus is free on every other clock: the line engine's X words go there.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_c   : integer range 0 to 1023;
        variable v_ph  : std_logic;
        variable v_en  : unsigned(7 downto 0);
        variable v_col : unsigned(4 downto 0);
        variable v_mm  : unsigned(10 downto 0);
        variable v_u8  : unsigned(7 downto 0);
        variable v_s9  : unsigned(10 downto 0);
        variable v_x, v_y : unsigned(23 downto 0);
        variable v_dm, v_m1, v_m2, v_m3 : signed(11 downto 0);
        variable v_xm  : signed(12 downto 0);
        variable v_vd  : signed(8 downto 0);
        variable v_t18 : unsigned(17 downto 0);
        variable v_h   : unsigned(14 downto 0);
        variable v_ad  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            v_c := to_integer(f_c); v_ph := f_c(0);
            f_done <= '0';
            if s_zgo = '1' then
                f_busy <= '0';
            elsif f_go = '1' then
                f_busy <= '1'; f_c <= (others => '0');
                f_p1 <= t_ps1; f_p2 <= t_ps2; f_en <= t_en; f_pos <= c_pos0;
                v_x := z_sm xor resize(z_sm(23 downto 7), 24);
                f_x <= v_x(23 downto 1) & '1';
                f_sl <= (others => '0'); f_sm <= (others => '0');
            elsif f_busy = '1' then
                f_c <= f_c + 1;
                if f_c = c_fend then f_busy <= '0'; f_done <= '1'; end if;
                if v_ph = '0' and v_c >= 8 then                 -- video: one-pole slew along the trace
                    v_vd := signed(resize(f_vsf, 9)) - signed(resize(f_sm, 9));
                    v_vd := signed(resize(f_sm, 9)) + shift_right(v_vd, 2);
                    f_sm <= unsigned(v_vd(7 downto 0));
                end if;
                if v_ph = '1' then
                    f_p1 <= f_p1 + c_f1c; f_p2 <= f_p2 + c_f2c;
                    if v_c >= 5 then f_pos <= f_pos + signed(resize(s_xs & '0', 12)); end if;
                    if v_c >= 5 then                                -- xorshift: x ^= x << 5; x ^= x >> 3
                        v_y := f_x xor (f_x(18 downto 0) & "00000");
                        f_x <= v_y xor resize(v_y(23 downto 3), 24);
                        v_s9 := resize(f_sl, 11) + resize(f_wn, 11);
                        f_sl <= v_s9(8 downto 1);
                    end if;
                    if v_c >= 9 then f_en <= f_en + signed(resize(c_es & "00", 28)); end if;
                end if;
            end if;

            -- VCOs through one sine ROM (which also serves the sway look-up between fills)
            if f_busy = '0' then r_s1_a <= t_psi(15 downto 8);
            elsif v_ph = '0' then r_s1_a <= f_p1(15 downto 8);
            else r_s1_a <= f_p2(15 downto 8); end if;
            f_s1f <= r_s1_q;
            v_u8 := unsigned(f_s1f) xor x"80";                       -- offset binary: no shared sign bits in the adds
            f_a5 <= shift_left(resize(v_u8, 11), 2) + resize(v_u8, 11);
            if v_ph = '0' then                                       -- (5 s1 + 3 s2) / 8 + 128
                v_mm := f_a5 + shift_left(resize(v_u8, 11), 1) + resize(v_u8, 11);
                r_sh_a <= v_mm(10 downto 3);
            end if;
            f_shf <= r_sh_q;

            -- noise: two bytes of the generator ANDed (skewed toward 0), slewed, x 2.25
            f_wn <= f_x(23 downto 16) and f_x(15 downto 8);
            v_s9 := shift_left(resize(f_sl, 11), 1) + resize(f_sl(7 downto 2), 11);
            if v_s9 > 255 then f_nz <= (others => '1'); else f_nz <= v_s9(7 downto 0); end if;

            -- crossfader (Rough, 0..8 eighths): S + (N - S) r
            v_dm := signed(resize(f_nz, 12)) - signed(resize(f_shf, 12));
            v_m1 := (others => '0'); v_m2 := (others => '0'); v_m3 := (others => '0');
            if c_r3(0) = '1' then v_m1 := v_dm; end if;
            if c_r3(1) = '1' then v_m2 := shift_left(v_dm, 1); end if;
            if c_r3(2) = '1' then v_m3 := shift_left(v_dm, 2); end if;
            f_mp <= v_m1 + v_m2 + v_m3;
            v_xm := signed(resize(f_shf & "000", 13)) + resize(f_mp, 13);
            if c_r3(3) = '1' then f_w <= f_nz; else f_w <= unsigned(v_xm(10 downto 3)); end if;

            -- log ROM: the waveform, then the video level (between fills: the sequencer's look-ups)
            if f_busy = '0' then r_lg_a <= q_la;
            elsif v_ph = '1' then r_lg_a <= f_w;
            else r_lg_a <= f_sm; end if;
            f_lg <= r_lg_q;

            -- burst envelope (attenuation ROM) + the trace's attenuation
            if f_en(27) = '1' then v_en := (others => '0');
            elsif f_en(26 downto 20) /= 0 then v_en := (others => '1');
            elsif s_twin = '1' then v_en := unsigned(f_en(18 downto 11));
            else v_en := unsigned(f_en(19 downto 12)); end if;
            r_ev_a <= v_en;
            f_le <= r_le_q;
            f_ec <= resize(f_le, 17) + resize(t_ca, 17);

            -- exponent to the EXPU
            if v_ph = '0' then v_t18 := resize(f_lg, 18) + resize(f_ec, 18);
            else v_t18 := resize(f_lg, 18) + resize(t_cv, 18); end if;
            f_t <= f_sat16(v_t18);

            -- video: the store cell under the sample
            if f_pos(11) = '1' then v_col := (others => '0');
            elsif f_pos(10 downto 5) > 29 then v_col := to_unsigned(29, 5);
            else v_col := unsigned(f_pos(9 downto 5)); end if;
            r_vs_a <= t_row & v_col;
            f_vsf <= unsigned(r_vs_q);

            -- heights: waveform part (odd clock), + video part, clipped to the window (even clock)
            if v_ph = '1' then f_h1 <= x_v(15 downto 2); end if;
            if v_ph = '0' then
                if t_rowv = '1' then v_h := resize(f_h1, 15) + resize(x_v(15 downto 2), 15);
                else v_h := resize(f_h1, 15); end if;
                if v_h > f_clip then f_hs <= f_clip; else f_hs <= v_h(12 downto 0); end if;
            end if;

            -- bank write bus: a sample on the odd clocks of a fill, else the line engine's X word
            v_ad := signed(resize(f_c(9 downto 1), 11)) - 9 - signed(resize(c_nc, 11)) + signed(resize(c_ic, 11));
            bw_we <= (others => '0');
            if f_busy = '1' and v_ph = '1' then
                if v_c >= 19 and f_c < c_fend - 3 and v_ad >= 0 and v_ad <= 240 then
                    bw_we(to_integer(q_fp)) <= '1';
                end if;
                if v_ad = 0 then                  -- the first sample, then the slope of each cell
                    bw_a <= to_unsigned(254, 8);
                    bw_d <= "000" & std_logic_vector(f_hs);
                else
                    bw_a <= unsigned(v_ad(7 downto 0)) - 1;
                    bw_d <= std_logic_vector(resize(signed('0' & f_hs), 16) - resize(signed('0' & f_hp), 16));
                end if;
                f_hp <= f_hs;
            elsif l_xw = '1' then
                bw_we(to_integer(l_bi)) <= '1';
                bw_a <= (others => '1');
                bw_d <= std_logic_vector(l_x);
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
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma, one-pole low-pass along the
    -- line, sampled into the 64 x 30 store on every 16th virtual row at each 64-virtual-px cell centre
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l  : unsigned(7 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_d  : signed(13 downto 0);
        variable v_p  : unsigned(7 downto 0);
        variable v_r  : unsigned(12 downto 0);
        variable v_9  : unsigned(11 downto 0);
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

            -- stage 1: the pixel's level
            w_av1 <= data_in.avid;
            if s_src = '1' then
                w_lum <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then w_lum <= v_l - 16; else w_lum <= (others => '0'); end if;
            end if;
            -- stage 2: low-pass, cell-centre strobe
            w_av2 <= w_av1;
            w_stb <= '0';
            if w_av1 = '1' then
                v_d := signed(resize(w_lum & "00000", 14)) - signed(resize(w_acc, 14));
                if s_xs = 2 then v_d := shift_right(v_d, 5); else v_d := shift_right(v_d, 4); end if;
                v_d := signed(resize(w_acc, 14)) + v_d;
                w_acc <= unsigned(v_d(12 downto 0));
                v_p := resize(w_ph, 8) + resize(s_xs, 8);
                if v_p >= 128 then
                    w_ph <= v_p(6 downto 0);
                    w_c1 <= w_c1 + 1;
                    if w_c1 <= 29 and w_samp = '1' then w_stb <= '1'; end if;
                    w_col <= w_c1(4 downto 0);
                else
                    w_ph <= v_p(6 downto 0);
                end if;
            else
                w_acc <= (others => '0');
                if s_xs = 4 then w_ph <= to_unsigned(32, 7); w_c1 <= to_unsigned(4, 6);
                else w_ph <= to_unsigned(64, 7); w_c1 <= (others => '0'); end if;
            end if;
            -- line end: the next line's store row (one per 16 virtual rows)
            if w_av2 = '1' and w_av1 = '0' then
                v_r := resize(w_racc, 13) + resize(s_ys, 13);
                w_racc <= v_r(11 downto 0);
                if v_r(12) = '1' then
                    if w_row(6) = '0' then w_row <= w_row + 1; end if;
                    if w_row < 63 then w_samp <= '1'; else w_samp <= '0'; end if;
                else
                    w_samp <= '0';
                end if;
            end if;
            if s_zone = '1' then
                w_racc <= (others => '0'); w_row <= (others => '0'); w_samp <= '1';
            end if;
            -- stage 3: write
            v_9 := shift_left(resize(w_acc(12 downto 5), 12), 3) + resize(w_acc(12 downto 5), 12);
            v_we <= w_stb;
            v_wa <= w_row(5 downto 0) & w_col;
            if v_9(11) = '1' then v_wd <= (others => '1'); else v_wd <= std_logic_vector(v_9(10 downto 3)); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once after the vblank fills for line 0), computes the NEXT
    -- line from the trace-constant file, one bank at a time: X = (vy + R + 1) + not baseline (written to the
    -- bank's word 255 for the pre-roll), the zone advance, the row's colour slope from the past trace's pitch,
    -- and each bank's colour index cm + (rows above the baseline) x slope (about 150 clocks in all)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_r   : unsigned(6 downto 0);
        variable v_dn  : unsigned(11 downto 0);
        variable v_ci  : unsigned(18 downto 0);
        variable v_ge  : std_logic;
        variable v_if  : unsigned(9 downto 0);
        variable v_v12 : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            l_done <= '0'; l_rel <= '0';
            case s_rs is
                when "00"   => v_r := to_unsigned(16, 7);
                when "01"   => v_r := to_unsigned(32, 7);
                when others => v_r := to_unsigned(64, 7);
            end case;
            l_rq <= rf_q;

            if l_zk = '1' then
                e_vy <= (others => '0'); l_c <= to_unsigned(1, 3); l_run <= '0'; l_first <= '1';
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19); l_c <= to_unsigned(1, 3); l_run <= '0';
            elsif l_c /= 0 then
                l_c <= l_c + 1;
                case to_integer(l_c) is
                    when 1 =>
                        l_vyr <= signed(resize(e_vy(18 downto 4), 16)) + signed(resize(v_r, 16)) + 1;
                        l_bi <= p_hd + 1; rf_ra <= (p_hd + 1) & '0';
                        l_rdy <= b_rdy;
                    when 4 => l_x <= l_vyr + signed(l_rq);
                    when 5 =>                 -- X of the next trace: has the beam reached its baseline?
                        case s_rs is
                            when "00"   => if l_x(14 downto 4) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                            when "01"   => if l_x(14 downto 5) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                            when others => if l_x(14 downto 6) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                        end case;
                        l_adv <= v_ge and not l_x(15) and l_rdy(to_integer(l_bi));
                    when 6 =>
                        if l_adv = '1' then l_hdn <= p_hd + 1; else l_hdn <= p_hd; end if;
                        l_bi <= (others => '0'); l_s <= (others => '0'); l_run <= '1'; l_pre <= '1';
                        l_c <= (others => '0');
                    when others => null;
                end case;
            elsif l_run = '1' then
                l_s <= l_s + 1;
                -- serial product (the loads below override it)
                if l_mb(0) = '1' then l_acc <= l_acc + l_ma; end if;
                l_ma <= l_ma(19 downto 0) & '0';
                l_mb <= '0' & l_mb(10 downto 1);
                case to_integer(l_s) is
                    when 0 =>
                        if l_pre = '1' then rf_ra <= l_hdn & '1'; else rf_ra <= l_bi & '0'; end if;
                    when 1 =>
                        rf_ra <= l_bi & '1';
                    when 3 =>
                        if l_pre = '1' then   -- past trace: 10240 / pitch -> the row's colour slope and glow ramp
                            v_if := unsigned(l_rq(15 downto 6));
                            case to_integer(c_m4) is              -- slope = pitch^-1 x (4 - m) x 2 / 15
                                when 0      => v_v12 := v_if & "00";
                                when 1      => v_v12 := resize(v_if & '0', 12) + resize(v_if, 12);
                                when 2      => v_v12 := resize(v_if & '0', 12);
                                when 3      => v_v12 := resize(v_if, 12);
                                when others => v_v12 := (others => '0');
                            end case;
                            l_csl <= resize(v_v12(11 downto 3), 11) + resize(v_v12(11 downto 7), 11);
                            l_gsh <= (others => '1'); l_gb <= '0';
                            for i in 0 to 9 loop
                                if v_if(i) = '1' then
                                    l_gsh <= to_unsigned(10 - i, 4);
                                    if i > 0 then l_gb <= v_if(i - 1); else l_gb <= '0'; end if;
                                end if;
                            end loop;
                        else
                            l_x <= l_vyr + signed(l_rq);
                        end if;
                    when 4 =>
                        if l_pre = '0' then
                            l_cmr <= unsigned(l_rq(5 downto 0));
                            -- rows above the baseline = (R - X) >> 4 = (not X) >> 4 + R / 16 (+ 1 when X is a whole row)
                            v_dn := unsigned(not l_x(15 downto 4)) + resize(v_r(6 downto 4), 12);
                            if l_x(3 downto 0) = 0 then v_dn := v_dn + 1; end if;
                            if v_dn(11) = '1' then l_mb <= (others => '0');
                            elsif v_dn(10) = '1' then l_mb <= to_unsigned(1023, 11);
                            else l_mb <= '0' & v_dn(9 downto 0); end if;
                            l_ma <= resize(l_csl, 21); l_acc <= (others => '0');
                            l_xw <= '1';
                        end if;
                    when 6 => l_xw <= '0';
                    when 15 =>
                        if l_pre = '1' then
                            l_pre <= '0';
                        else
                            v_ci := resize(l_acc(20 downto 8), 19) + resize(l_cmr, 19);
                            if v_ci > 63 then l_cin(to_integer(l_bi)) <= (others => '1');
                            else l_cin(to_integer(l_bi)) <= v_ci(5 downto 1); end if;
                            l_bi <= l_bi + 1;
                            if l_bi = 7 then l_run <= '0'; l_done <= '1'; end if;
                        end if;
                    when others => null;
                end case;
            end if;

            -- the line starts: its head, colour indexes, active banks and glow ramp take effect
            if g_start = '1' then
                p_hd <= l_hdn;
                for b in 0 to 7 loop
                    p_ci(b) <= l_cin(b);
                    if b < to_integer(l_hdn) then p_lom(b) <= '1'; else p_lom(b) <= '0'; end if;
                    if to_unsigned(b, 3) + 1 = l_hdn then p_msk(b) <= '0'; else p_msk(b) <= '1'; end if;
                end loop;
                p_act <= l_rdy;
                p_gsh <= l_gsh; p_gb <= l_gb;
                if l_hdn /= p_hd then l_rel <= '1'; end if;
                l_first <= '0';
            end if;
            if s_zgo = '1' then
                p_hd <= (others => '0'); l_hdn <= (others => '0'); p_act <= (others => '0');
                l_c <= (others => '0'); l_run <= '0'; l_first <= '0'; l_xw <= '0';
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Trace banks + the per-bank pixel accumulator.  Stage 1 = the pixel counter.
    --   2 read address (the pixel's cell) | 3 RAM: the cell's slope d | 4 fabric copy; e += d
    -- e = 8 (X + h(x)): preset in hblank by reading the X word for 8 clocks and the first sample for 8
    -- clocks, then one add per pixel; the slopes are exact differences of the samples, so e is exact.
    ---------------------------------------------------------------------------
    g_bank : for b in 0 to 7 generate
        type t_bk is array (0 to 255) of std_logic_vector(15 downto 0);
        signal bk : t_bk := (others => (others => '0'));
        attribute no_rw_check of bk : signal is true;
        signal q1 : std_logic_vector(15 downto 0) := (others => '0');
    begin
        p_bank : process(clk)
        begin
            if rising_edge(clk) then
                q1 <= bk(to_integer(p_ra));
                if bw_we(b) = '1' then bk(to_integer(bw_a)) <= bw_d; end if;

                if p_dz = '1' then
                    x_d(b) <= (others => '0');
                else
                    x_d(b) <= signed(q1);
                end if;
                if p_eclr = '1' then
                    x_ea(b) <= (others => '0');
                else
                    x_ea(b) <= x_ea(b) + resize(x_d(b), 20);
                end if;
                x_e6(b) <= unsigned(x_ea(b)(18 downto 3));
            end if;
        end process p_bank;
    end generate g_bank;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_cov, v_lo, v_hi, v_sel : std_logic_vector(7 downto 0);
        variable v_e   : unsigned(15 downto 0);
        variable v_d   : signed(13 downto 0);
        variable v_c   : unsigned(4 downto 0);
        variable v_r   : unsigned(6 downto 0);
        variable v_wl  : unsigned(6 downto 0);
        variable v_es  : unsigned(13 downto 0);
        variable v_t   : unsigned(16 downto 0);
        variable v_a2  : signed(15 downto 0);
        variable v_mn  : signed(15 downto 0);
        variable v_al  : unsigned(4 downto 0);
        variable v_cd, v_m1, v_m2, v_m3, v_m4 : signed(12 downto 0);
        variable v_cc, v_n1, v_n2, v_n3, v_n4 : signed(9 downto 0);
        variable v_o   : signed(12 downto 0);
        variable v_3   : unsigned(15 downto 0);
        variable v_g9  : unsigned(8 downto 0);
        variable v_trg : std_logic;
    begin
        if rising_edge(clk) then
            case s_rs is
                when "00"   => v_r := to_unsigned(16, 7);  v_wl := to_unsigned(40, 7);
                when "01"   => v_r := to_unsigned(32, 7);  v_wl := to_unsigned(56, 7);
                when others => v_r := to_unsigned(64, 7);  v_wl := to_unsigned(88, 7);
            end case;

            -- stage 1: pixel counter
            if g_start = '1' then
                p_x <= (others => '0');
            elsif g_avid = '1' then
                p_x <= p_x + 1;
            end if;

            -- pre-roll: after the last pixel has left the accumulators (or, for line 0, when the line
            -- engine has finished in vblank): clear, 8 x the X word, 8 x the first sample, hold
            if (s_gav_sr(3) = '0' and s_gav_sr(4) = '1') or (l_done = '1' and l_first = '1') then v_trg := '1'; else v_trg := '0'; end if;
            if v_trg = '1' then
                p_pr <= to_unsigned(1, 5);
            elsif p_pr /= 0 then
                if p_pr = 19 then p_pr <= (others => '0'); else p_pr <= p_pr + 1; end if;
            end if;
            if (p_pr >= 1 and p_pr <= 16) or s_gav_sr(0) = '1' then p_dz <= '0'; else p_dz <= '1'; end if;
            if v_trg = '1' or p_pr = 1 then p_eclr <= '1'; else p_eclr <= '0'; end if;

            -- stage 2: bank read address (pre-roll: the X word in cycles 1..8, the first sample in 9..16)
            if v_trg = '1' or (p_pr >= 1 and p_pr <= 7) then
                p_ra <= to_unsigned(255, 8);
            elsif p_pr >= 8 and p_pr <= 15 then
                p_ra <= to_unsigned(254, 8);
            elsif g_avid = '1' then
                p_ra <= p_x(10 downto 3);
            else
                p_ra <= (others => '0');
            end if;

            -- stage 5: nearest covering trace (highest bank below the head, else highest at or above it)
            for b in 0 to 7 loop
                v_cov(b) := p_act(b) and p_msk(b) and not x_ea(b)(19);
            end loop;
            v_lo := v_cov and p_lom;
            v_hi := v_cov and not p_lom;
            if v_lo /= x"00" then v_sel := f_top1(v_lo); else v_sel := f_top1(v_hi); end if;
            p6_sel <= v_sel;
            if v_cov /= x"00" then p6_cv <= '1'; else p6_cv <= '0'; end if;

            -- stage 6: the winner's depth, slope and colour index
            v_e := (others => '0'); v_d := (others => '0'); v_c := (others => '0');
            for b in 0 to 7 loop
                if p6_sel(b) = '1' then
                    v_e := v_e or x_e6(b); v_d := v_d or resize(x_d(b)(15 downto 3), 14); v_c := v_c or p_ci(b);
                end if;
            end loop;
            p8_e <= v_e; p8_d <= v_d; p8_ci <= v_c; p8_cv <= p6_cv;
            -- stage 7: |slope|, depth clamps
            if p8_d(13) = '1' then p9_ad <= unsigned(-p8_d); else p9_ad <= unsigned(p8_d); end if;
            if p8_e(15 downto 14) /= 0 then v_es := (others => '1'); else v_es := p8_e(13 downto 0); end if;
            p9_e <= v_es;
            if v_es < v_r then p9_er <= (others => '0');
            else
                v_es := v_es - v_r;
                if v_es(13 downto 12) /= 0 then p9_er <= (others => '1'); else p9_er <= v_es(11 downto 5); end if;
            end if;
            p9_ci <= p8_ci; p9_cv <= p8_cv;
            -- stage 8: line weight T = W + 3 px of slope; glow depth in tenths of the pitch (1 or 1.5 x 2^n)
            v_3 := shift_left(resize(p9_ad, 16), 1) + resize(p9_ad, 16);
            case to_integer(s_xs) is
                when 4      => v_t := resize(v_3(15 downto 1), 17) + resize(v_wl, 17);
                when 3      => v_t := resize(p9_ad & '0', 17) + resize(v_wl, 17);
                when others => v_t := resize(v_3, 17) + resize(v_wl, 17);
            end case;
            if v_t(16 downto 12) /= 0 then p10_t <= (others => '1'); else p10_t <= v_t(11 downto 0); end if;
            v_g9 := resize(p9_er & '0', 9);
            if p_gb = '1' then v_g9 := v_g9 + resize(p9_er, 9); end if;
            p10_gd <= shift_right(v_g9, to_integer(p_gsh));
            p10_e <= p9_e; p10_ci <= p9_ci; p10_cv <= p9_cv;
            -- stage 9: coverage = min(e, T + R - e); glow level
            v_a2 := signed(resize(p10_t, 16)) + signed(resize(v_r, 16)) - signed(resize(p10_e, 16));
            if v_a2 < signed(resize(p10_e, 16)) then p11_mn <= v_a2; else p11_mn <= signed(resize(p10_e, 16)); end if;
            if p10_gd >= 10 or s_glow = '0' then p11_gl <= (others => '0');
            else p11_gl <= to_unsigned(10, 5) - resize(p10_gd(3 downto 0), 5); end if;
            p11_cv <= p10_cv;
            p11_ci <= p10_ci;
            -- stage 10: level
            case s_rs is
                when "00"   => v_mn := p11_mn;
                when "01"   => v_mn := shift_right(p11_mn, 1);
                when others => v_mn := shift_right(p11_mn, 2);
            end case;
            if v_mn(15) = '1' then v_al := (others => '0');
            elsif v_mn(14 downto 4) /= 0 then v_al := to_unsigned(16, 5);
            else v_al := '0' & unsigned(v_mn(3 downto 0)); end if;
            if p11_cv = '0' then p12_l <= (others => '0');
            elsif p11_gl > v_al then p12_l <= p11_gl;
            else p12_l <= v_al; end if;
            p12_ci <= p11_ci; p13_ci <= p12_ci;
            -- stage 12: palette address; 13: ROM; 14: fabric copy (colour - ground)
            r_pl_a <= s_paper & c_sat2 & p13_ci;
            p12b_l <= p12_l; p12c_l <= p12b_l; p12d_l <= p12c_l;
            p13_q <= r_pl_q; p13_l <= p12d_l;
            -- stage 15: x level (0..16 sixteenths) as two gated-add pairs per channel (chroma on 8 bits)
            v_cd := resize(signed(p13_q(31 downto 21)), 13);
            v_m1 := (others => '0'); v_m2 := (others => '0'); v_m3 := (others => '0'); v_m4 := (others => '0');
            if p13_l(0) = '1' then v_m1 := v_cd; end if;
            if p13_l(1) = '1' then v_m2 := shift_left(v_cd, 1); end if;
            if p13_l(2) = '1' then v_m3 := v_cd; end if;
            if p13_l(4) = '1' then v_m4 := shift_left(v_cd, 2);
            elsif p13_l(3) = '1' then v_m4 := shift_left(v_cd, 1); end if;
            p14_ay <= v_m1 + v_m2; p14_by <= v_m3 + v_m4;
            for ch in 0 to 1 loop
                if ch = 0 then v_cc := resize(signed(p13_q(20 downto 13)), 10);
                else v_cc := resize(signed(p13_q(12 downto 5)), 10); end if;
                v_n1 := (others => '0'); v_n2 := (others => '0'); v_n3 := (others => '0'); v_n4 := (others => '0');
                if p13_l(0) = '1' then v_n1 := v_cc; end if;
                if p13_l(1) = '1' then v_n2 := shift_left(v_cc, 1); end if;
                if p13_l(2) = '1' then v_n3 := v_cc; end if;
                if p13_l(4) = '1' then v_n4 := shift_left(v_cc, 2);
                elsif p13_l(3) = '1' then v_n4 := shift_left(v_cc, 1); end if;
                if ch = 0 then p14_au <= v_n1 + v_n2; p14_bu <= v_n3 + v_n4;
                else p14_av <= v_n1 + v_n2; p14_bv <= v_n3 + v_n4; end if;
            end loop;
            -- stage 16: the product
            p15_my <= resize(p14_ay, 16) + shift_left(resize(p14_by, 16), 2);
            p15_mu <= resize(p14_au, 13) + shift_left(resize(p14_bu, 13), 2);
            p15_mv <= resize(p14_av, 13) + shift_left(resize(p14_bv, 13), 2);
            -- stage 17: compose over the ground; the ground outside the generated avid
            if s_gav_sr(14) = '0' then
                s_y17 <= c_bgy; s_u17 <= c_bgu; s_v17 <= c_bgv;
            else
                v_o := signed(resize(c_bgy, 13)) + resize(shift_right(p15_my, 4), 13); s_y17 <= unsigned(v_o(9 downto 0));
                v_o := signed(resize(c_bgu, 13)) + resize(shift_right(p15_mu, 2), 13); s_u17 <= unsigned(v_o(9 downto 0));
                v_o := signed(resize(c_bgv, 13)) + resize(shift_right(p15_mv, 2), 13); s_v17 <= unsigned(v_o(9 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums of 8-bit samples, the 8-back
    -- taps from a RAM delay, resynced to the ground once 8 blank samples are in), luma [1 2 1]/4 delayed to the
    -- chroma's centre, luma 13/16, blanking gate; the syncs ride a second RAM delay
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ly : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- RAM delays: q(t) = in(t - 1 - D); chroma + its fabric copy = 8 clocks, syncs + output register = C_LAT
            d_w <= d_w + 1; d_rc <= d_w - 5; d_rs <= d_w - (C_LAT - 3);
            d_cq <= dc(to_integer(d_rc));
            dc(to_integer(d_w)) <= std_logic_vector(s_u17(9 downto 2)) & std_logic_vector(s_v17(9 downto 2));
            d_cf <= d_cq;
            d_sq <= ds(to_integer(d_rs));
            ds(to_integer(d_w)) <= '0' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            d_so <= d_sq;
            -- stage 18: chroma 8-sums
            if s_bah = "00000000" then
                s_bau <= c_bgu(9 downto 2) & "000"; s_bav <= c_bgv(9 downto 2) & "000";
            else
                s_bau <= s_bau + s_u17(9 downto 2) - unsigned(d_cf(15 downto 8));
                s_bav <= s_bav + s_v17(9 downto 2) - unsigned(d_cf(7 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_y18 <= s_y17; s_y19 <= s_y18;
            -- stage 19: chroma out (centred 4 back), luma [1 2 1] (centred on stage 18)
            v_sc := resize(s_bau, 12) + s_baud; s_u19 <= v_sc(11 downto 2);
            v_sc := resize(s_bav, 12) + s_bavd; s_v19 <= v_sc(11 downto 2);
            v_ly := resize(s_y17, 12) + shift_left(resize(s_y18, 12), 1) + resize(s_y19, 12);
            s_lf(19) <= v_ly(11 downto 2);
            for i in 20 to 23 loop s_lf(i) <= s_lf(i - 1); end loop;
            -- stage 24: luma 13/16
            if s_lf(23) > 64 then v_dl := s_lf(23) - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_lf(23) - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u19; s_v24 <= s_v19;     -- the chroma is centred 4 back: already the stage-23 pixel's
            -- stage 25: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= d_so(2);
    data_out.vsync_n <= d_so(1);
    data_out.field_n <= d_so(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture pulsar;
