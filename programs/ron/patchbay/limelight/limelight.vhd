-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Limelight v0.1.0 -- Patchbay #9: soft-keyed beams of gelled light.
--
-- The patch:
--   H ramp, V ramp -> four ramp rotators (one per lamp, its angle swung by LFO 1 with a phase offset per
--   lamp, leaned by the slower LFO 2) -> two rotated ramps per lamp (half-planes through the lamp at
--   theta - w and theta + w) -> soft window comparator = a cone with a linear penumbra
--   V ramp -> full-wave rectifier at the floor line: every rotated ramp mirrors below it (the reflection)
--   V ramp -> level ramp (Reach) + haze texture + camera -> the level each window is clipped to
--   the window again, above a threshold that follows an ellipse profile across the floor line -> the pool
--   four keys -> gels -> additive Y / U / V mixer (or a hard-priority keyer); luma above full pulls the
--   chroma to neutral (white where beams cross)
--
--   K1  Reach     level ramp / floor: the floor line 500 .. 1030 lines below the top, the fade scaled to it
--   K2  Soft      window comparator gain: penumbra 12 .. 384 px
--   K3  Sweep     LFO 1 depth into the rotators: 3 .. 45 degrees
--   K4  Speed     LFO 1 rate, bipolar (square law, +-8 deadband); LFO 2 follows at 5/16 plus its own drift
--   K5  Spread    LFO 1 phase offset per lamp: unison .. half a cycle (scissoring)
--   K6  Colour    gel set (8)
--   S7  Lamps     Top / Foot (the rail below the bottom edge, the fan reversed)
--   S8  Haze      Smooth (two drifting triangle strata) / Rays (ratio comparators on the two ramps: shafts)
--   S9  Mix       Add (light) / Key (opaque layers, lamp 1 on top, dark seams)
--   S10 Pools     On / Off
--   S11 Source    Video (camera luma into the level) / Osc (a drifting diamond gobo)
--   P12 BLAZE     window width: half-angle 1.3 .. 20 degrees, and the level's bias
--
-- Engine: a vblank-zone sequencer (one serial multiplier, 72 slots of 24 clocks) writes every edge's
-- per-pixel step, line wrap, +-line step and seed into a block RAM; a line engine computes the NEXT
-- line's level and pool threshold; in hblank a bus sequencer plays the wraps from the RAM into eight
-- 25-bit pixel accumulators (acc += op, the operand register parked on the bus: no input mux).  Pixel
-- path 10 stages.
-- Virtual 1080-line coordinates.  Generated avid; output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before the blanking gate.  Colours in the hardware
-- convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K2 = Colour (was K6); K2 Soft fixed at 600 (C_KFIX_2).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.limelight_pkg.all;
use work.all;

architecture limelight of program_top is
    constant C_KFIX_2 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(600, 10));   -- K6 camera pass: fixed knob
    constant C_LAT : integer := 16;   -- = processing_delay_clks

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
    -- haze stratum A: phase per field line = 220 / 256 per virtual line (a 298-line period)
    function f_pas(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(977, 10);
            when 4          => return to_unsigned(495, 10);
            when 8          => return to_unsigned(825, 10);
            when 12         => return to_unsigned(412, 10);
            when 5 | 6 | 14 => return to_unsigned(330, 10);
            when 1 | 2 | 10 => return to_unsigned(440, 10);
            when others     => return to_unsigned(220, 10);
        end case;
    end function;
    -- Osc lattice: phase per field line = 242 / 256 per virtual line (a 271-line period)
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1075, 11);
            when 4          => return to_unsigned(544, 11);
            when 8          => return to_unsigned(907, 11);
            when 12         => return to_unsigned(453, 11);
            when 5 | 6 | 14 => return to_unsigned(363, 11);
            when 1 | 2 | 10 => return to_unsigned(484, 11);
            when others     => return to_unsigned(242, 11);
        end case;
    end function;
    function f_tri8(v : unsigned(8 downto 0)) return unsigned is
    begin
        if v(8) = '1' then return not v(7 downto 0); else return v(7 downto 0); end if;
    end function;
    function f_clamp8(v : signed) return unsigned is
        variable x : signed(v'length - 1 downto 0);
        variable r : unsigned(7 downto 0);
        variable hi : std_logic;
    begin
        x := v;
        hi := '0';
        for i in 8 to x'high - 1 loop
            if x(i) = '1' then hi := '1'; end if;
        end loop;
        if x(x'high) = '1' then r := (others => '0');
        elsif hi = '1' then r := (others => '1');
        else r := unsigned(x(7 downto 0)); end if;
        return r;
    end function;
    function f_min8(a, b : unsigned(7 downto 0)) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sn_a : unsigned(10 downto 0) := (others => '0');
    signal r_sn_q, r_sn_f : unsigned(15 downto 0) := (others => '0');
    signal r_s1_a : unsigned(7 downto 0) := (others => '0');
    signal r_s1_q : unsigned(15 downto 0) := (others => '0');

    -- coefficient RAM: word = kind * 8 + edge (edge = 2 * lamp + side).  kinds: 0 pixel step, 1 line wrap
    -- (-N * step), 2 +line step, 3 -line step, 4 seed.  Written by the sequencer in the vblank zone, read
    -- by the bus sequencer (hblank, and once in the zone after the sequencer).
    type t_cram is array (0 to 255) of std_logic_vector(31 downto 0);
    signal cram : t_cram := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of cram : signal is true;
    signal c_ra  : unsigned(5 downto 0) := (others => '0');
    signal c_rq, c_bus : std_logic_vector(31 downto 0) := (others => '0');
    signal q_we : std_logic := '0';
    signal q_wa : unsigned(5 downto 0) := (others => '0');
    signal w_ra : unsigned(5 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 320 x 270 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(204, 9);
    signal s_oys : unsigned(10 downto 0) := to_unsigned(242, 11);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(800 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);   -- K6 camera pass: glided K6 Camera
    signal s_cg : unsigned(2 downto 0) := to_unsigned(4, 3);            -- camera gain step (8 steps)
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(600 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(64 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(560 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_foot, s_rays, s_key, s_nopool, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field sequencer
    signal q_run, q_done : std_logic := '0';
    signal q_sid : unsigned(4 downto 0) := (others => '0');
    signal q_ph  : unsigned(4 downto 0) := (others => '0');
    signal q_i   : unsigned(1 downto 0) := (others => '0');
    signal q_e   : std_logic := '0';
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_n   : unsigned(4 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(15 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_ld  : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');          -- exponent (Q8 octaves)
    signal z_vsq : unsigned(17 downto 0) := (others => '0');
    signal z_snn, z_ans, z_sgy : std_logic := '0';
    signal z_th  : signed(11 downto 0) := (others => '0');
    signal z_aw  : unsigned(12 downto 0) := (others => '0');          -- edge angle theta -+ w
    signal z_p1  : unsigned(12 downto 0) := (others => '0');          -- LFO 1 phase of the lamp
    signal z_cy  : unsigned(13 downto 0) := (others => '0');
    signal z_step : unsigned(14 downto 0) := (others => '0');
    signal z_acc : signed(24 downto 0) := (others => '0');            -- the one adder: seeds, signs, RAM write data
    signal z_ab  : signed(24 downto 0) := (others => '0');            -- its operand, registered (K6 camera pass)
    signal z_clr, z_go, z_sub, q_we2 : std_logic := '0';
    signal q_wa2 : unsigned(5 downto 0) := (others => '0');
    signal c_yf   : unsigned(10 downto 0) := to_unsigned(843, 11);
    signal c_sf   : unsigned(15 downto 0) := (others => '0');
    signal c_g    : unsigned(15 downto 0) := to_unsigned(683, 16);
    signal c_gh   : unsigned(15 downto 0) := to_unsigned(342, 16);
    signal c_off  : unsigned(17 downto 0) := to_unsigned(32768, 18);
    signal c_w    : unsigned(8 downto 0) := to_unsigned(30, 9);
    signal c_ir   : unsigned(15 downto 0) := (others => '0');
    signal c_amp  : unsigned(10 downto 0) := to_unsigned(68, 11);
    signal c_base : signed(7 downto 0) := (others => '0');
    signal c_phi1 : unsigned(23 downto 0) := x"4D0000";            -- LFO 1 phase (2^24 per cycle)
    signal c_phi2 : unsigned(25 downto 0) := to_unsigned(7864320, 26);   -- LFO 2 phase (2^26 per cycle)
    signal c_hza, c_hzb : unsigned(15 downto 0) := (others => '0');
    signal c_has  : unsigned(6 downto 0) := to_unsigned(34, 7);
    signal c_hbs  : unsigned(7 downto 0) := to_unsigned(86, 8);
    signal c_ysm  : unsigned(10 downto 0) := to_unsigned(110, 11);
    signal c_e0   : signed(20 downto 0) := (others => '0');
    signal c_gel  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_pas  : unsigned(9 downto 0) := to_unsigned(220, 10);

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    type t_est is (ST_IDLE, ST_D, ST_M1, ST_F, ST_M2, ST_U, ST_S1A, ST_S1B, ST_WAIT);
    signal e_st, e_ret : t_est := ST_IDLE;
    attribute fsm_encoding : string;
    attribute fsm_encoding of e_st : signal is "none";
    attribute fsm_encoding of e_ret : signal is "none";
    signal e_c    : unsigned(3 downto 0) := (others => '0');
    signal e_e    : signed(20 downto 0) := (others => '0');           -- floor - line (Q8 virtual lines)
    signal e_d    : unsigned(12 downto 0) := (others => '0');
    signal e_first, e_fdone : std_logic := '0';
    signal m_a    : unsigned(15 downto 0) := (others => '0');
    signal m_b    : unsigned(12 downto 0) := (others => '0');
    signal m_n    : unsigned(3 downto 0) := (others => '0');
    signal m_acc  : unsigned(28 downto 0) := (others => '0');
    signal n_fall : unsigned(7 downto 0) := (others => '0');
    signal n_fb   : signed(9 downto 0) := (others => '0');
    signal n_cap, n_t : unsigned(7 downto 0) := (others => '0');
    signal n_bey  : std_logic := '0';
    signal n_pa, n_pb : unsigned(15 downto 0) := (others => '0');
    signal l_fb   : signed(9 downto 0) := (others => '0');
    signal l_cap, l_t : unsigned(7 downto 0) := (others => '0');
    signal l_bey  : std_logic := '0';

    -- bus sequencer, pixel accumulators
    signal w_run, w_seed, w_clr : std_logic := '0';
    signal w_k    : unsigned(2 downto 0) := (others => '0');
    signal w_p    : unsigned(1 downto 0) := (others => '0');
    signal w_k1, w_k2, w_k3, w_k4 : unsigned(2 downto 0) := (others => '0');
    signal w_p1, w_p2, w_p3, w_p4 : unsigned(1 downto 0) := (others => '0');
    signal w_v1, w_v2, w_v3, w_v4 : std_logic := '0';
    signal w_cross : std_logic := '0';
    signal b_run, b_rund : std_logic := '0';
    signal b_cnt  : unsigned(11 downto 0) := (others => '0');
    type t_s25 is array (0 to 7) of signed(24 downto 0);
    type t_u8x8 is array (0 to 7) of unsigned(7 downto 0);
    type t_u8x4 is array (0 to 3) of unsigned(7 downto 0);
    signal b_acc, b_op : t_s25 := (others => (others => '0'));
    signal h_pa, h_pb : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    signal p2_c   : t_u8x8 := (others => (others => '0'));
    signal p2_in  : std_logic_vector(7 downto 0) := (others => '0');
    signal p2_ta, p2_tb : unsigned(7 downto 0) := (others => '0');
    signal p3_m, p4_i, p4_q : t_u8x4 := (others => (others => '0'));
    signal p3_ra, p3_rb : unsigned(9 downto 0) := (others => '0');
    signal p3_rv, p4_par, p4_rv : std_logic := '0';
    signal p3_ts, p4_ts, p5_tex : signed(7 downto 0) := (others => '0');
    signal p4_lm, p5_v : unsigned(7 downto 0) := (others => '0');
    signal p6_src : signed(8 downto 0) := (others => '0');
    signal p6_srcg : signed(10 downto 0) := (others => '0');
    signal p6_tf  : signed(10 downto 0) := (others => '0');
    signal p7_lvl : unsigned(7 downto 0) := (others => '0');
    signal p5_j, p6_j : t_u8x4 := (others => (others => '0'));
    signal p7_ya, p7_yb : unsigned(8 downto 0) := (others => '0');
    signal p7_ua, p7_ub, p7_va, p7_vb : signed(9 downto 0) := (others => '0');
    signal p8_y   : unsigned(9 downto 0) := (others => '0');
    signal p8_u, p8_v, p9_u, p9_v : signed(10 downto 0) := (others => '0');
    signal p9_lim : unsigned(7 downto 0) := (others => '0');
    signal p9_yc  : unsigned(7 downto 0) := (others => '0');
    signal p10_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p10_u, p10_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y11 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cby : t_u10(0 to 7) := (others => to_unsigned(64, 10));
    signal s_cau, s_cav, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bayd, s_bby, s_bbyd : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y13, s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13, s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs and the
    -- coefficient RAM (one write statement; reads never meet a write)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_LL_EXP2(to_integer(r_ex_a));
            r_sn_q <= C_LL_SIN(to_integer(r_sn_a));
            r_sn_f <= r_sn_q;
            r_s1_q <= C_LL_S1(to_integer(r_s1_a));
        end if;
    end process p_rom;

    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            c_rq <= cram(to_integer(unsigned'("00" & c_ra)));
            q_we2 <= q_we; q_wa2 <= q_wa;
            if q_we2 = '1' then cram(to_integer(unsigned'("00" & q_wa2))) <= std_logic_vector(resize(z_acc, 32)); end if;
            c_bus <= c_rq;
            c_ra <= w_ra;
        end if;
    end process p_cram;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_pas <= f_pas(s_tid);

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
                s_foot   <= registers_in(6)(0);
                s_rays   <= registers_in(6)(1);
                s_key    <= registers_in(6)(2);
                s_nopool <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(408, 9);
                                   c_has <= to_unsigned(68, 7); c_hbs <= to_unsigned(172, 8);
                    when 3      => s_oxs <= to_unsigned(306, 9);
                                   c_has <= to_unsigned(51, 7); c_hbs <= to_unsigned(129, 8);
                    when others => s_oxs <= to_unsigned(204, 9);
                                   c_has <= to_unsigned(34, 7); c_hbs <= to_unsigned(86, 8);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            -- K6 camera gain step = round(k / 128), 0 .. 7 -> x 0, 0.625, 1.25, 2, 2.5, 3, 4, 5:
            -- 25 % = x1.25 (shipped), 50 % = x2.5, 100 % = x5 (a two-term shift-add in the pixel path)
            if s_g2(12 downto 9) = "1111" then s_cg <= "111";
            else s_cg <= resize(shift_right(s_g2(12 downto 9) + 1, 1), 3); end if;
            s_oys <= f_oys(s_tid);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(5));   -- K6 Camera
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(1));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                            if z_gc = 1 and s_gd > -33 and s_gd < 33 then s_gd <= (others => '0'); end if;   -- camera: +-4
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

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Per-field sequencer: one serial multiplier, 24 clocks per slot (0 operands, 1 load,
    -- 2..17 iterate, 19..22 store / one add / ROM addresses / RAM write).  Global slots 0..15, then
    -- 14 slots per lamp (16, 17 the lamp's angle; 18..23 edge A, 24..29 edge B: per-pixel step, line
    -- step, wrap, seed).  One adder (z_acc) forms every signed result and is the RAM
    -- write data.  Started at the zone start; nothing free-runs from configuration.
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_sid : integer range 0 to 31;
        variable v_ph  : integer range 0 to 31;
        variable v16   : unsigned(15 downto 0);
        variable v_lxp : unsigned(10 downto 0);
        variable v_fan : signed(11 downto 0);
        variable v_k   : unsigned(2 downto 0);
        variable v_ab : signed(24 downto 0);
        variable v_sub, v_go, v_clr : std_logic;
        variable v_ix  : unsigned(12 downto 0);
        variable v_cos, v_set : std_logic;
        variable v_q   : unsigned(1 downto 0);
        variable v_ta  : signed(11 downto 0);
        variable v_p2  : unsigned(25 downto 0);
        variable v_ms  : unsigned(16 downto 0);
        variable v_y2  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            q_done <= '0';
            z_ld <= '0';
            q_we <= '0';
            v_sid := to_integer(q_sid);
            v_ph  := to_integer(q_ph);
            v_k   := q_i & q_e;

            if s_zgo = '1' then
                q_run <= '1'; q_sid <= (others => '0'); q_ph <= (others => '0'); q_i <= "00"; q_e <= '0';
            elsif q_run = '1' then
                if q_ph = 23 then
                    q_ph <= (others => '0');
                    if q_sid = 29 then
                        if q_i = 3 then q_run <= '0'; q_done <= '1'; end if;
                        q_i <= q_i + 1;
                        q_sid <= to_unsigned(16, 5);
                    else
                        q_sid <= q_sid + 1;
                    end if;
                    if q_sid = 23 then q_e <= '1'; elsif q_sid = 29 then q_e <= '0'; end if;
                else
                    q_ph <= q_ph + 1;
                end if;
            end if;

            -- the multiplier
            -- (shift-right form: the multiplicand z_fa stays put for the 16 steps, the product ends in f_acc)
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_b <= z_fb;
            else
                if f_b(0) = '1' then v_ms := resize(f_acc(31 downto 16), 17) + resize(z_fa, 17);
                else v_ms := resize(f_acc(31 downto 16), 17); end if;
                if f_n /= 0 then
                    f_acc <= v_ms & f_acc(15 downto 1);
                    f_n <= f_n - 1;
                end if;
                f_b <= '0' & f_b(15 downto 1);
            end if;
            if z_ld = '1' then f_n <= to_unsigned(16, 5); end if;

            -- per-lamp constants: static aim; lamp x in pixels from the first pixel; pixels per virtual px
            case q_i is
                when "00"   => v_fan := to_signed(239, 12);
                when "01"   => v_fan := to_signed(80, 12);
                when "10"   => v_fan := to_signed(-80, 12);
                when others => v_fan := to_signed(-239, 12);
            end case;
            if s_foot = '1' then v_fan := -v_fan; end if;
            case to_integer(s_xs) is
                when 4 =>
                    case q_i is
                        when "00"   => v_lxp := to_unsigned(30, 11);
                        when "01"   => v_lxp := to_unsigned(245, 11);
                        when "10"   => v_lxp := to_unsigned(460, 11);
                        when others => v_lxp := to_unsigned(675, 11);
                    end case;
                when 3 =>
                    case q_i is
                        when "00"   => v_lxp := to_unsigned(202, 11);
                        when "01"   => v_lxp := to_unsigned(489, 11);
                        when "10"   => v_lxp := to_unsigned(775, 11);
                        when others => v_lxp := to_unsigned(1062, 11);
                    end case;
                when others =>
                    case q_i is
                        when "00"   => v_lxp := to_unsigned(307, 11);
                        when "01"   => v_lxp := to_unsigned(737, 11);
                        when "10"   => v_lxp := to_unsigned(1167, 11);
                        when others => v_lxp := to_unsigned(1597, 11);
                    end case;
            end case;

            -- exponential results: (EXP2 x constant) >> (15 - octave)
            case to_integer(z_e(10 downto 8)) is
                when 0      => v16 := f_acc(30 downto 15);
                when 1      => v16 := f_acc(29 downto 14);
                when 2      => v16 := f_acc(28 downto 13);
                when 3      => v16 := f_acc(27 downto 12);
                when 4      => v16 := f_acc(26 downto 11);
                when 5      => v16 := f_acc(25 downto 10);
                when others => v16 := f_acc(24 downto 9);
            end case;

            -- the one adder: cleared at phase 18, then z_acc <= z_acc +- B at phases 19 / 20 / 21
            v_ab := signed(f_acc(24 downto 0)); v_sub := '0'; v_go := '0'; v_clr := '0';
            case v_sid is
                when 19 | 25 =>                                                     -- pixel step
                    if v_ph = 18 then v_clr := '1'; end if;
                    if v_ph = 19 then v_ab := signed(resize(z_step, 25)); v_sub := q_e; v_go := '1'; end if;
                when 20 | 26 =>                                                     -- line wrap
                    if v_ph = 18 then v_clr := '1'; end if;
                    if v_ph = 19 then v_sub := not q_e; v_go := '1'; end if;
                when 21 | 27 =>                                                     -- + line step, then - line step
                    if v_ph = 18 or v_ph = 20 then v_clr := '1'; end if;
                    if v_ph = 19 then v_ab := signed(resize(f_acc(24 downto 8), 25)); v_sub := z_sgy; v_go := '1'; end if;
                    if v_ph = 21 then v_ab := signed(resize(f_acc(24 downto 8), 25)); v_sub := not z_sgy; v_go := '1'; end if;
                when 22 | 28 =>                                                     -- seed: offset -+ x part
                    if v_ph = 18 then v_clr := '1'; end if;
                    if v_ph = 19 then v_sub := not q_e; v_go := '1'; end if;
                    if v_ph = 20 then v_ab := signed(resize(c_off, 25)); v_go := '1'; end if;
                when 23 | 29 =>                                                     -- seed: +- y part
                    if v_ph = 19 then v_sub := z_sgy; v_go := '1'; end if;
                when others => null;
            end case;
            -- (K6 camera pass, timing: the adder's controls and operand are registered here and the add runs
            -- one clock later; the RAM write strobe is delayed one clock to match)
            z_clr <= v_clr and q_run; z_go <= v_go and q_run; z_sub <= v_sub; z_ab <= v_ab;
            if z_clr = '1' then z_acc <= (others => '0');
            elsif z_go = '1' then
                if z_sub = '1' then z_acc <= z_acc - z_ab; else z_acc <= z_acc + z_ab; end if;
            end if;

            -- sine / tangent address: one of three angles, folded to the quarter wave
            v_ix := z_p1; v_cos := '0'; v_set := '0';
            if v_ph = 22 then
                case v_sid is
                    when 14 | 25 => v_ix := z_p1; v_set := '1';                                         -- LFO 1 of the (next) lamp
                    when 16      => v_ix := (c_phi2(25 downto 24) + q_i) & c_phi2(23 downto 13); v_set := '1';   -- LFO 2
                    when 17 | 19 => v_ix := z_aw; v_cos := '1'; v_set := '1';                           -- cos of the edge angle
                    when 18 | 24 => v_ix := z_aw; v_set := '1';                                         -- sin, tan of the edge angle
                    when others => null;
                end case;
            end if;
            v_q := v_ix(12 downto 11);
            if v_cos = '1' then v_q := v_q + 1; end if;
            if q_run = '1' and v_set = '1' then
                if v_q(0) = '1' then r_sn_a <= not v_ix(10 downto 0); else r_sn_a <= v_ix(10 downto 0); end if;
                z_snn <= v_q(1);
            end if;

            if q_run = '1' then
                -- 0: operands
                if v_ph = 0 then
                    z_ld <= '1';
                    case v_sid is
                        when 0  => z_fa <= resize(s_vm, 16);                  z_fb <= resize(s_vm, 16);
                        when 1  => z_fa <= resize(not s_g1(12 downto 3), 16); z_fb <= to_unsigned(268, 16);
                        when 2  => z_fa <= r_ex_q;                            z_fb <= to_unsigned(6233, 16);
                        when 3  => z_fa <= resize(not s_gp(12 downto 3), 16); z_fb <= to_unsigned(1009, 16);
                        when 4  => z_fa <= r_ex_q;                            z_fb <= to_unsigned(1443, 16);
                        when 5  => z_fa <= resize(s_g1(12 downto 3), 16);     z_fb <= to_unsigned(268, 16);
                        when 6  => z_fa <= r_ex_q;                            z_fb <= to_unsigned(500, 16);
                        when 7  => z_fa <= resize(not unsigned(C_KFIX_2), 16); z_fb <= to_unsigned(1281, 16);
                        when 8  => z_fa <= r_ex_q;                            z_fb <= to_unsigned(683, 16);
                        when 9  => z_fa <= resize(s_gp(12 downto 3), 16);     z_fb <= to_unsigned(1009, 16);
                        when 10 => z_fa <= r_ex_q;                            z_fb <= to_unsigned(30, 16);
                        when 11 => z_fa <= c_g;                               z_fb <= resize(s_xs, 16);
                        when 13 => z_fa <= resize(s_gp(12 downto 3), 16);     z_fb <= to_unsigned(120, 16);
                        when 14 => z_fa <= resize(s_g3(12 downto 3), 16);     z_fb <= to_unsigned(239, 16);
                        when 15 => z_fa <= c_g;                               z_fb <= to_unsigned(5, 16);
                        when 16 => z_fa <= resize(c_amp, 16);                 z_fb <= r_sn_q;
                        when 17 => z_fa <= to_unsigned(159, 16);              z_fb <= r_sn_q;
                        when 18 | 24 => z_fa <= r_sn_q;                       z_fb <= c_gh;
                        when 19 | 25 => z_fa <= r_sn_q;                       z_fb <= c_g;
                        when 20 | 26 => z_fa <= resize(z_step, 16);           z_fb <= resize(s_w, 16);
                        when 21 | 27 => z_fa <= resize(z_cy, 16);             z_fb <= resize(s_ys, 16);
                        when 22 | 28 => z_fa <= resize(z_step, 16);           z_fb <= resize(v_lxp, 16);
                        when 23 | 29 => z_fa <= resize(z_cy, 16);             z_fb <= resize(c_ysm, 16);
                        when others  => z_fa <= (others => '0');              z_fb <= (others => '0');
                    end case;
                end if;

                -- 19: results
                if v_ph = 19 then
                    case v_sid is
                        when 0 =>
                            z_vsq <= f_acc(17 downto 0);
                            c_hza <= c_hza + 109; c_hzb <= c_hzb - 169;          -- haze strata drift
                            c_gel <= C_LL_GEL(to_integer(s_g6(12 downto 10)));
                        when 1 | 5 | 7 | 9 => z_e <= f_acc(20 downto 10);
                        when 3  => z_e <= z_e + f_acc(20 downto 10);
                        when 2  => c_sf <= v16;
                        when 4  => c_ir <= v16;
                        when 6  => c_yf <= v16(10 downto 0);
                        when 8  => c_g <= v16;
                        when 10 => c_w <= v16(8 downto 0);
                        when 11 => c_gh <= f_acc(17 downto 2);
                        when 13 => c_base <= signed(f_acc(17 downto 10)) - 60;
                        when 14 =>
                            c_amp <= to_unsigned(68, 11) + f_acc(18 downto 8);
                            if s_vneg = '1' then c_phi1 <= c_phi1 - resize(z_vsq, 24);
                            else c_phi1 <= c_phi1 + resize(z_vsq, 24); end if;
                        when 15 => c_off <= f_acc(18 downto 1) + 32768;
                        when 16 | 17 =>
                            -- theta = fan +- Sweep sin(LFO 1) +- 7 deg sin(LFO 2)
                            if v_sid = 16 then v_ta := v_fan; else v_ta := z_th; end if;
                            if z_snn = '1' then z_th <= v_ta - signed(resize(f_acc(26 downto 16), 12));
                            else z_th <= v_ta + signed(resize(f_acc(26 downto 16), 12)); end if;
                        when 18 | 24 => z_step <= f_acc(31 downto 17);
                        when 19 | 25 =>
                            z_cy <= f_acc(31 downto 18);
                            z_sgy <= z_snn xor (not q_e);
                            z_ans <= z_snn;
                        when others => null;
                    end case;
                end if;

                -- 20: second step; RAM writes of what the adder formed at 19
                if v_ph = 20 then
                    r_ex_a <= z_e(7 downto 0);
                    case v_sid is
                        when 6 =>
                            -- seed height: lamp rail to line 0 (top) / to the mirrored line 0 (foot); floor - line 0
                            if s_foot = '1' then
                                v_y2 := (c_yf & '0') - 970; c_ysm <= v_y2(10 downto 0);
                                c_e0 <= shift_left(resize(signed('0' & c_yf), 21) - 1080, 8);
                            else
                                c_ysm <= to_unsigned(110, 11);
                                c_e0 <= shift_left(resize(signed('0' & c_yf), 21), 8);
                            end if;
                        when 14 => z_p1 <= c_phi1(23 downto 11);
                        when 17 => z_aw <= unsigned(resize(z_th, 13) - signed(resize(c_w, 13)));
                        when 19 => z_aw <= unsigned(resize(z_th, 13) + signed(resize(c_w, 13)));
                        when 25 => z_p1 <= z_p1 + resize(s_g5(12 downto 1), 13);
                        when others => null;
                    end case;
                end if;

                -- RAM writes: strobe with the adder's last step, so the write lands one clock later on its result
                case v_sid is
                    when 19 | 25 => if v_ph = 19 then q_we <= '1'; q_wa <= "000" & v_k; end if;
                    when 20 | 26 => if v_ph = 19 then q_we <= '1'; q_wa <= "001" & v_k; end if;
                    when 21 | 27 =>
                        if v_ph = 19 then q_we <= '1'; q_wa <= "010" & v_k; end if;
                        if v_ph = 21 then q_we <= '1'; q_wa <= "011" & v_k; end if;
                    when 23 | 29 => if v_ph = 19 then q_we <= '1'; q_wa <= "100" & v_k; end if;
                    when others => null;
                end case;

                -- LFO 2: its own drift plus a quarter of LFO 1's rate
                if v_sid = 14 and (v_ph = 20 or v_ph = 21) then
                    if v_ph = 20 then v_p2 := resize(z_vsq, 26); else v_p2 := to_unsigned(48094, 26); end if;
                    c_phi2 <= c_phi2 + v_p2;
                end if;
            end if;
        end if;
    end process p_seq;

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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer
    -- write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
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
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone, after the
    -- sequencer, for line 0); computes the NEXT line: floor distance d, the
    -- level (Reach) and the pool threshold (the ellipse profile across the pool
    -- band).  One serial multiplier (13 iterations).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_e  : signed(20 downto 0);
        variable v_d9 : unsigned(16 downto 0);
        variable v_f  : unsigned(11 downto 0);
        variable v_u  : unsigned(16 downto 0);
        variable v_ms : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            e_fdone <= '0';

            -- the multiplier
            if m_b(0) = '1' then v_ms := resize(m_acc(28 downto 13), 17) + resize(m_a, 17);
            else v_ms := resize(m_acc(28 downto 13), 17); end if;
            if m_n /= 0 then
                m_acc <= v_ms & m_acc(12 downto 1);
                m_n <= m_n - 1;
            end if;
            m_b <= '0' & m_b(12 downto 1);
            if e_c /= 0 then e_c <= e_c - 1; end if;

            if q_done = '1' then
                e_e <= c_e0; n_pa <= c_hza; n_pb <= c_hzb;
                e_first <= '1'; e_st <= ST_D;
            elsif g_start = '1' then
                if s_foot = '1' then e_e <= e_e + signed(resize(s_ys, 21)); else e_e <= e_e - signed(resize(s_ys, 21)); end if;
                n_pa <= n_pa + resize(s_pas, 16); n_pb <= n_pb + resize(s_ys & '0', 16);    -- haze strata, per line
                e_first <= '0'; e_st <= ST_D;
                l_fb <= n_fb; l_cap <= n_cap; l_t <= n_t; l_bey <= n_bey;
            else
                case e_st is
                    when ST_D =>
                        n_bey <= e_e(20);
                        if e_e(20) = '1' then v_e := -e_e; else v_e := e_e; end if;
                        e_d <= unsigned(v_e(18 downto 6));
                        e_st <= ST_M1;
                    when ST_M1 =>
                        m_a <= c_sf; m_b <= e_d; m_acc <= (others => '0'); m_n <= to_unsigned(13, 4);
                        e_c <= to_unsigned(14, 4); e_ret <= ST_F; e_st <= ST_WAIT;
                    when ST_F =>
                        if n_bey = '1' then
                            v_d9 := resize(e_d & "000", 17) + resize(e_d, 17);
                            if v_d9(16 downto 7) >= 72 then n_fall <= (others => '0');
                            else n_fall <= to_unsigned(72, 8) - v_d9(14 downto 7); end if;
                        else
                            v_f := to_unsigned(102, 12) + m_acc(28 downto 18);
                            if v_f > 255 then n_fall <= (others => '1'); else n_fall <= v_f(7 downto 0); end if;
                        end if;
                        e_st <= ST_M2;
                    when ST_M2 =>
                        m_a <= c_ir; m_b <= e_d; m_acc <= (others => '0'); m_n <= to_unsigned(13, 4);
                        e_c <= to_unsigned(14, 4); e_ret <= ST_U; e_st <= ST_WAIT;
                        -- level bias and its cap for the line
                        n_fb <= signed(resize(n_fall, 10)) + resize(c_base, 10);
                        if n_fall(7) = '1' then n_cap <= (others => '1'); else n_cap <= n_fall(6 downto 0) & '0'; end if;
                    when ST_U =>
                        v_u := m_acc(28 downto 12);
                        if v_u(16 downto 8) = 0 and s_nopool = '0' then n_t <= to_unsigned(1, 8); else n_t <= (others => '0'); end if;
                        r_s1_a <= v_u(7 downto 0);
                        e_st <= ST_S1A;
                    when ST_S1A => e_st <= ST_S1B;
                    when ST_S1B =>
                        -- pool threshold: 0 on the floor line, up the ellipse profile, 255 (no pool) off the band
                        if n_t = 0 or r_s1_q(8) = '1' then n_t <= (others => '1'); else n_t <= r_s1_q(7 downto 0); end if;
                        e_st <= ST_IDLE;
                        e_fdone <= e_first;
                    when ST_WAIT =>
                        if e_c = 0 then e_st <= e_ret; end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Bus sequencer and the pixel accumulators.  Per edge, four RAM words in a
    -- row: line wrap (or the seed), +-line step (skipped across the floor line
    -- and when seeding), pixel step, a spare.  The word on the bus loads the
    -- edge's operand register; one clock later the accumulator adds it.  During
    -- the line the accumulator adds the pixel step for a fixed count of pixels
    -- (its own counter: a short source line cannot shorten it).
    ---------------------------------------------------------------------------
    p_bus : process(clk)
        variable v_ld, v_st : std_logic;
    begin
        if rising_edge(clk) then
            w_clr <= '0';
            b_rund <= b_run;
            if e_fdone = '1' then
                w_run <= '1'; w_seed <= '1'; w_clr <= '1'; w_k <= "000"; w_p <= "00";
            elsif b_rund = '1' and b_run = '0' then
                w_run <= '1'; w_seed <= '0'; w_k <= "000"; w_p <= "00";
                if l_bey /= n_bey then w_cross <= '1'; else w_cross <= '0'; end if;
            elsif w_run = '1' then
                w_p <= w_p + 1;
                if w_p = 3 then
                    w_k <= w_k + 1;
                    if w_k = 7 then w_run <= '0'; end if;
                end if;
            end if;
            -- address phase (the word of phase p is on the bus four clocks after p)
            case w_p is
                when "00"   => if w_seed = '1' then w_ra <= "100" & w_k; else w_ra <= "001" & w_k; end if;
                when "01"   => if (s_foot xor n_bey) = '0' then w_ra <= "010" & w_k; else w_ra <= "011" & w_k; end if;
                when "10"   => w_ra <= "000" & w_k;
                when others => w_ra <= "000" & w_k;
            end case;
            w_v1 <= w_run; w_k1 <= w_k;  w_p1 <= w_p;
            w_v2 <= w_v1;  w_k2 <= w_k1; w_p2 <= w_p1;
            w_v3 <= w_v2;  w_k3 <= w_k2; w_p3 <= w_p2;
            w_v4 <= w_v3;  w_k4 <= w_k3; w_p4 <= w_p3;

            -- pixel run: a fixed count per line
            if g_start = '1' then
                b_run <= '1'; b_cnt <= (others => '0');
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = g_wm1 then b_run <= '0'; end if;
            end if;
            s_lb_ra <= b_cnt(10 downto 0);

            -- accumulators, operand registers, pool spans
            for k in 0 to 7 loop
                v_ld := '0'; v_st := '0';
                if w_v4 = '1' and to_integer(w_k4) = k then
                    case w_p4 is
                        when "00"   => v_ld := '1';
                        when "01"   => v_ld := '1'; v_st := '1';
                        when "10"   => v_ld := '1'; v_st := (not w_seed) and (not w_cross);
                        when others => null;
                    end case;
                end if;
                if v_ld = '1' then b_op(k) <= signed(c_bus(24 downto 0)); end if;
                if w_clr = '1' then
                    b_acc(k) <= (others => '0');
                elsif b_run = '1' or v_st = '1' then
                    b_acc(k) <= b_acc(k) + b_op(k);
                end if;
            end loop;

            -- haze strata phases
            if g_start = '1' then
                h_pa <= n_pa; h_pb <= n_pb;
            elsif b_run = '1' then
                h_pa <= h_pa + resize(c_has, 16);
                h_pb <= h_pb - resize(c_hbs, 16);
            end if;
        end if;
    end process p_bus;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the accumulators' value for the pixel).  The level
    -- (texture, source) is ready four stages after the windows, so a pixel is
    -- clipped to the level of the pixel four to its left: the texture and the
    -- picture sit 4 px to the right, the beams stay put.
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ra, v_rb : unsigned(9 downto 0);
        variable v_rv : std_logic;
        variable v_c  : std_logic_vector(4 downto 0);
        variable v_t  : unsigned(9 downto 0);
        variable v_s  : unsigned(9 downto 0);
        variable v_ca, v_cb : signed(15 downto 0);
        variable v_l  : signed(11 downto 0);
        variable v_l8 : unsigned(7 downto 0);
        variable v_j  : unsigned(8 downto 0);
        variable v_own : integer range 0 to 4;
        variable v_g  : std_logic_vector(7 downto 0);
        variable v_ty, v_tu, v_tv : t_u8x4;
        variable v_e  : unsigned(9 downto 0);
        variable v_x  : signed(11 downto 0);
        variable v_y7 : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: window ramps clamped, inside flags; haze triangles
            for k in 0 to 7 loop
                p2_c(k) <= f_clamp8(b_acc(k)(24 downto 8));
                if b_acc(k)(24) = '0' and b_acc(k)(23 downto 8) /= 0 then p2_in(k) <= '1'; else p2_in(k) <= '0'; end if;
            end loop;
            p2_ta <= f_tri8(h_pa(15 downto 7));
            p2_tb <= f_tri8(h_pb(15 downto 7));

            -- 3: window = min of the two ramps; the first lamp whose cone holds the pixel hands its two
            -- ramps (one pixel on) to the ray comparators; smooth haze
            for i in 0 to 3 loop
                p3_m(i) <= f_min8(p2_c(2 * i), p2_c(2 * i + 1));
            end loop;
            v_rv := '1';
            if (p2_in(0) and p2_in(1)) = '1' then
                v_ra := unsigned(b_acc(0)(23 downto 14)); v_rb := unsigned(b_acc(1)(23 downto 14));
            elsif (p2_in(2) and p2_in(3)) = '1' then
                v_ra := unsigned(b_acc(2)(23 downto 14)); v_rb := unsigned(b_acc(3)(23 downto 14));
            elsif (p2_in(4) and p2_in(5)) = '1' then
                v_ra := unsigned(b_acc(4)(23 downto 14)); v_rb := unsigned(b_acc(5)(23 downto 14));
            elsif (p2_in(6) and p2_in(7)) = '1' then
                v_ra := unsigned(b_acc(6)(23 downto 14)); v_rb := unsigned(b_acc(7)(23 downto 14));
            else
                v_ra := (others => '0'); v_rb := (others => '0'); v_rv := '0';
            end if;
            p3_ra <= v_ra; p3_rb <= v_rb; p3_rv <= v_rv;
            v_t := resize(p2_ta & '0', 10) + resize(p2_ta, 10) + resize(p2_tb, 10);
            p3_ts <= signed(resize(v_t(9 downto 3), 8)) - 64;

            -- 4: window clipped to the level; pool; ray sector parity (ratio comparators)
            -- pool: the part of the window above the line's threshold (a lozenge on the floor line)
            for i in 0 to 3 loop
                p4_i(i) <= f_min8(p3_m(i), p7_lvl);
                if p3_m(i) > l_t then p4_q(i) <= p3_m(i) - l_t; else p4_q(i) <= (others => '0'); end if;
            end loop;
            v_c := "00000";
            if resize(p3_ra & "00", 12) > resize(p3_rb, 12) then v_c(0) := '1'; end if;
            if resize(p3_ra & '0', 12) > resize(p3_rb, 12) then v_c(1) := '1'; end if;
            if p3_ra > p3_rb then v_c(2) := '1'; end if;
            if resize(p3_ra, 12) > resize(p3_rb & '0', 12) then v_c(3) := '1'; end if;
            if resize(p3_ra, 12) > resize(p3_rb & "00", 12) then v_c(4) := '1'; end if;
            p4_par <= v_c(0) xor v_c(1) xor v_c(2) xor v_c(3) xor v_c(4);
            p4_rv <= p3_rv;
            p4_ts <= p3_ts;
            p4_lm <= unsigned(s_lb_q);

            -- 5: key level = window + pool (saturating); texture; source threshold
            for i in 0 to 3 loop
                v_j := resize(p4_i(i), 9) + resize(p4_q(i), 9);
                if v_j(8) = '1' then p5_j(i) <= (others => '1'); else p5_j(i) <= v_j(7 downto 0); end if;
            end loop;
            if s_rays = '1' then
                if p4_rv = '0' then p5_tex <= (others => '0');
                elsif p4_par = '1' then p5_tex <= to_signed(56, 8);
                else p5_tex <= to_signed(-76, 8); end if;
            else
                p5_tex <= p4_ts;
            end if;
            if s_src = '1' then
                if p4_lm > 133 then p5_v <= p4_lm - 133; else p5_v <= (others => '0'); end if;
            else
                if p4_lm > 44 then p5_v <= p4_lm - 44; else p5_v <= (others => '0'); end if;
            end if;

            -- 6: Mix: Add passes all four keys; Key passes the first lamp that is lit, doubled.  Level: bias
            -- + texture; source gain
            if p5_j(0) /= 0 then v_own := 0;
            elsif p5_j(1) /= 0 then v_own := 1;
            elsif p5_j(2) /= 0 then v_own := 2;
            elsif p5_j(3) /= 0 then v_own := 3;
            else v_own := 4; end if;
            for i in 0 to 3 loop
                if s_key = '0' then
                    p6_j(i) <= p5_j(i);
                elsif v_own = i then
                    if p5_j(i)(7) = '1' then p6_j(i) <= (others => '1');
                    else p6_j(i) <= p5_j(i)(6 downto 0) & '0'; end if;
                else
                    p6_j(i) <= (others => '0');
                end if;
            end loop;
            p6_tf <= resize(l_fb, 11) + resize(p5_tex, 11);
            -- K6 camera pass: the source before the camera gain (video x1, Osc = the shipped gobo / 1.25)
            if s_src = '1' then
                v_s := resize(p5_v & '0', 10) + resize(shift_right(p5_v, 1), 10) + resize(shift_right(p5_v, 2), 10);
                if v_s > 133 then p6_src <= to_signed(56, 9); else p6_src <= signed(resize(v_s(7 downto 0), 9)) - 77; end if;
            else
                p6_src <= signed(resize(p5_v, 9));
            end if;
            -- 6b: camera gain, x (a + b) / 16 by shift-select (its own stage; the source reaches the level
            -- one pixel after the texture)
            v_ca := (others => '0'); v_cb := (others => '0');
            case to_integer(s_cg) is
                when 1      => v_ca := shift_left(resize(p6_src, 16), 3); v_cb := shift_left(resize(p6_src, 16), 1);   -- 10
                when 2      => v_ca := shift_left(resize(p6_src, 16), 4); v_cb := shift_left(resize(p6_src, 16), 2);   -- 20
                when 3      => v_ca := shift_left(resize(p6_src, 16), 5);                                              -- 32
                when 4      => v_ca := shift_left(resize(p6_src, 16), 5); v_cb := shift_left(resize(p6_src, 16), 3);   -- 40
                when 5      => v_ca := shift_left(resize(p6_src, 16), 5); v_cb := shift_left(resize(p6_src, 16), 4);   -- 48
                when 6      => v_ca := shift_left(resize(p6_src, 16), 6);                                              -- 64
                when 7      => v_ca := shift_left(resize(p6_src, 16), 6); v_cb := shift_left(resize(p6_src, 16), 4);   -- 80
                when others => null;
            end case;
            v_ca := v_ca + v_cb;
            p6_srcg <= v_ca(14 downto 4);

            -- 7: the level (0 off the generated avid); gels: per lamp a Y tap (1/4, 1/2, 1) and U, V taps
            -- (0 or +-2), summed in pairs with lamp 1's (lamp 3's) sign carried outside
            v_l := resize(p6_tf, 12) + resize(p6_srcg, 12);
            if v_l < 0 then v_l8 := (others => '0');
            elsif v_l > 255 then v_l8 := (others => '1');
            else v_l8 := unsigned(v_l(7 downto 0)); end if;
            if s_gav_sr(4) = '0' then p7_lvl <= (others => '0');
            else p7_lvl <= f_min8(v_l8, l_cap); end if;
            for i in 0 to 3 loop
                v_g := c_gel(8 * i + 7 downto 8 * i);
                case v_g(1 downto 0) is
                    when "00"   => v_ty(i) := "00" & p6_j(i)(7 downto 2);
                    when "01"   => v_ty(i) := '0' & p6_j(i)(7 downto 1);
                    when others => v_ty(i) := p6_j(i);
                end case;
                if v_g(2) = '1' then v_tu(i) := p6_j(i); else v_tu(i) := (others => '0'); end if;
                if v_g(5) = '1' then v_tv(i) := p6_j(i); else v_tv(i) := (others => '0'); end if;
            end loop;
            p7_ya <= resize(v_ty(0), 9) + resize(v_ty(1), 9);
            p7_yb <= resize(v_ty(2), 9) + resize(v_ty(3), 9);
            -- (lamp 1's chroma taps are never negative: the gel table is ordered that way)
            if c_gel(12) = '1' then p7_ua <= signed(resize(v_tu(0), 10)) - signed(resize(v_tu(1), 10));
            else p7_ua <= signed(resize(v_tu(0), 10)) + signed(resize(v_tu(1), 10)); end if;
            if (c_gel(20) xor c_gel(28)) = '1' then p7_ub <= signed(resize(v_tu(2), 10)) - signed(resize(v_tu(3), 10));
            else p7_ub <= signed(resize(v_tu(2), 10)) + signed(resize(v_tu(3), 10)); end if;
            if c_gel(15) = '1' then p7_va <= signed(resize(v_tv(0), 10)) - signed(resize(v_tv(1), 10));
            else p7_va <= signed(resize(v_tv(0), 10)) + signed(resize(v_tv(1), 10)); end if;
            if (c_gel(23) xor c_gel(31)) = '1' then p7_vb <= signed(resize(v_tv(2), 10)) - signed(resize(v_tv(3), 10));
            else p7_vb <= signed(resize(v_tv(2), 10)) + signed(resize(v_tv(3), 10)); end if;

            -- 8: the mixer's sums (U, V = twice these)
            p8_y <= resize(p7_ya, 10) + resize(p7_yb, 10);
            if c_gel(20) = '1' then p8_u <= resize(p7_ua, 11) - resize(p7_ub, 11); else p8_u <= resize(p7_ua, 11) + resize(p7_ub, 11); end if;
            if c_gel(23) = '1' then p8_v <= resize(p7_va, 11) - resize(p7_vb, 11); else p8_v <= resize(p7_va, 11) + resize(p7_vb, 11); end if;

            -- 9: luma capped; the knee: luma above full narrows the chroma limit (224 down to 0)
            if p8_y > 250 then
                p9_yc <= to_unsigned(250, 8);
                v_e := p8_y - 250;
                if v_e(9 downto 7) /= "000" or v_e(6 downto 0) > 111 then p9_lim <= (others => '0');
                else p9_lim <= to_unsigned(224, 8) - (v_e(6 downto 0) & '0'); end if;
            else
                p9_yc <= p8_y(7 downto 0);
                p9_lim <= to_unsigned(224, 8);
            end if;
            p9_u <= p8_u; p9_v <= p8_v;

            -- 10: Y10 = 64 + 3.5 Y; chroma = 2 x the sums, limited, offset 512
            v_y7 := resize(p9_yc & "000", 11) - resize(p9_yc, 11);
            p10_y <= resize(v_y7(10 downto 1), 10) + 64;
            if p9_u > signed(resize(p9_lim, 11)) then v_x := signed(resize(p9_lim, 12));
            elsif p9_u < -signed(resize(p9_lim, 11)) then v_x := -signed(resize(p9_lim, 12));
            else v_x := resize(p9_u, 12); end if;
            p10_u <= unsigned(v_x(8 downto 0) & '0') xor "1000000000";
            if p9_v > signed(resize(p9_lim, 11)) then v_x := signed(resize(p9_lim, 12));
            elsif p9_v < -signed(resize(p9_lim, 11)) then v_x := -signed(resize(p9_lim, 12));
            else v_x := resize(p9_v, 12); end if;
            p10_v <= unsigned(v_x(8 downto 0) & '0') xor "1000000000";

            -- 11: compose; black outside the generated avid
            if s_gav_sr(8) = '0' then
                s_y11 <= C_KY; s_u11 <= C_KC; s_v11 <= C_KC;
            else
                s_y11 <= p10_y; s_u11 <= p10_u; s_v11 <= p10_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(8);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix), luma 13/16 and the
    -- blanking gate.  The running sums snap back to black once 8 blank samples
    -- are in (resync).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 12 sums, stage 13 output
            s_cay(0) <= s_y11; s_cau(0) <= s_u11; s_cav(0) <= s_v11;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KC & "000"; s_bav <= C_KC & "000";
            else
                s_bay <= s_bay + s_y11 - s_cay(7);
                s_bau <= s_bau + s_u11 - s_cau(7);
                s_bav <= s_bav + s_v11 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid
            if s_gav_sr(10) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y13 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u13 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v13 <= v_sc(13 downto 4);
            else
                s_y13 <= C_KY; s_u13 <= C_KC; s_v13 <= C_KC;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(10);
            -- box B: stage 14 sums, stage 15 output
            s_cby(0) <= s_y13; s_cbu(0) <= s_u13; s_cbv(0) <= s_v13;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KC & "000"; s_bbv <= C_KC & "000";
            else
                s_bby <= s_bby + s_y13 - s_cby(7);
                s_bbu <= s_bbu + s_u13 - s_cbu(7);
                s_bbv <= s_bbv + s_v13 - s_cbv(7);
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y15 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u15 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v15 <= v_sc(13 downto 4);
            -- 16: luma 13/16
            if s_y15 > 64 then v_dl := s_y15 - 64; else v_dl := (others => '0'); end if;
            s_y16 <= s_y15 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u16 <= s_u15; s_v16 <= s_v15;
            -- 17: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y17);
    data_out.u       <= std_logic_vector(s_u17);
    data_out.v       <= std_logic_vector(s_v17);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture limelight;
