-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Streamers v0.1.1 -- Heirloom (phase 3, video clips), from the still WIN_20231104_16_39_25 at 0.5 s.
--
-- v0.1.1: K6 Bend default 400 -> 280 (a textured camera picture no longer shreds the ribbons); the Osc lattice
-- is patched in at twice the camera's level, so Source = Osc still bends the default picture as boldly as before.
-- Under Sync, VCO Q's phase is kept equal to theta Q + d * FQ / FP everywhere (d = P phase since the red
-- leading edge), so no row runs Q free up to its first visible edge (v0.1.0 drew a hard-edged block at the left
-- edge on the rows whose edge fell just before x = 0, and at the centre under Mirror):
--   - each line starts at theta Q + d(first pixel) * FQ / FP (a serial divide and one more product per line);
--   - the first pixel never counts as an edge;
--   - running up, Q restarts at the red leading edge; running down (Mirror's left half) it restarts at the
--     ribbon's other edge at theta Q + FQ / FP, so the mirrored halves match; both restarts land 4 pixels after
--     their edge (the detector's pipeline) and carry those 4 increments.
--
-- The still: bold rainbow ribbons running down the frame, straight at the left edge and swinging wider and
-- wider toward the right, warm ribbons (magenta / orange / yellow / red / cream) crossing cool ones (blue /
-- green / navy / teal / cyan) where their swings disagree, glossy lavender and white highlights along the
-- trailing edges of the red ribbons.  In the clip the swings travel down the frame (one wave per 1.08 s).
--
-- The patch, as fitted (streamers.py has every constant):
--   VCO P and VCO Q are horizontal oscillators reset by hsync (P's phase is constant across lines at
--   x0 = -148 virtual px, Q's at -196), so the ribbons are straight at the left and swing more to the right.
--   Each is FM'd per line by two vertical LFOs (A: 560 lines, B: 760 lines, travelling down) and Q also by a
--   vertical ramp (-21 %):  F(y) = f (1 + ramp yc + D (dA sin(A + a) + dB sin(B + b))),  phase = F (x - x0).
--   P's square wave picks the palette (T_HI on the red half, T_LO on the dark-red half); Q's phase indexes it
--   (two sawtooth teeth in G, a triangle in B).  T_GL adds the glossy highlights behind each red ribbon's
--   trailing edge, fading over a quarter cycle of P.
--
--   K1  Red       VCO P frequency: the warm ribbons' count (x1/3 .. x3, 512 = the still's 6.8 across)
--   K2  Blue      VCO Q frequency: the cool ribbons' count (x1/3 .. x3)
--   K3  Wave      LFO wavelength: 0.5 .. 7.7 swings per frame height (512 = the still's 1.9)
--   K4  Speed     bipolar: the swings travel down (right) or up (left); 768 = the clip's speed
--   K5  Width     the red ribbons' share of each cycle, 9 % .. 89 % (512 = the still's 49 %)
--   K6  Bend      how far the camera (or the Osc, at twice the level) bends both oscillators' phase
--   S7  Mirror    both VCOs reset at the screen centre instead of the left edge: a symmetric pattern
--   S8  Cables    the RGB cables rotated (R <- G, G <- B, B <- R): green / violet / yellow ribbons
--   S9  Sync      VCO Q hard-synced to every red ribbon: one regular colour sequence per ribbon
--                 (continued from the last off-screen edge at the line start)
--   S10 Zigzag    triangle LFOs instead of sines: zigzag ribbons with sharp corners
--   S11 Source    Video / Osc
--   P12 SWING     FM depth D = 0.25 + 7.75 p^2: gently swaying ribbons (0) .. the still (318, D = 1) .. braids (D = 8)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a per-line engine kicked at
-- each line start (computes the NEXT line: 4 LFO sines, the two FM products, the increments and the start
-- phases, on one serial multiplier), two phase DDAs (reversing at the centre for Mirror), a 21-stage pixel
-- path.  Virtual 1080-line coordinates (SD shows the centre 4:3 of the picture).  Generated avid; output
-- conditioning (U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, Y through one, with the running-sum resync,
-- luma 13/16)
-- before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.streamers_pkg.all;
use work.all;

architecture streamers of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (phases: 2^24 per cycle; frequencies Q24 cycles per h = half virtual px)
    constant K_GP    : integer := 29517;       -- VCO P, 1 / 284.2 virtual px
    constant K_GQ    : integer := 13032;       -- VCO Q, 1 / 643.7 virtual px
    constant K_KA    : integer := 29959;       -- LFO A, 2^24 / 560 per virtual line
    constant K_RB    : integer := 48290;       -- LFO B / LFO A = 560 / 760, Q16
    constant K_V     : integer := 1078;        -- speed: d^2 * K_V >> 8 = Q24 per field (the clip at d = 248)
    constant K_VB    : integer := 268;         -- LFO B: d^2 * K_VB >> 8 (= K_V * 1.08 / 4.35)
    constant K_RSP   : integer := 203;         -- |ramp P| per virtual line, Q24 (negative)
    constant K_RSQ   : integer := 3324;        -- |ramp Q| per virtual line, Q24 (negative)
    constant K_R0P   : integer := 109524;      -- ramp P at line 0, Q24
    constant K_R0Q   : integer := 1795128;     -- ramp Q at line 0, Q24
    constant K_THQ   : integer := 13077840;    -- VCO Q phase at its reset point (-0.2205)
    constant K_PR16  : integer := 35062;       -- VCO P phase of the red ribbons' leading edge (0.535), Q16
    constant K_DUTY  : integer := 32113;       -- red share 0.49, Q16
    constant K_DP0   : integer := 296;         -- VCO P reset point to x = 0, h
    constant K_DQ0   : integer := 392;         -- VCO Q reset point to x = 0, h
    constant K_AAP   : integer := 11308315;    -- LFO phase offsets, Q24
    constant K_ABP   : integer := 11348615;
    constant K_AAQ   : integer := 5584260;
    constant K_ABQ   : integer := 16220577;

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

    -- line buffer: (luma >> 2) - 16, clipped at 0
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
    signal r_sn_a : unsigned(9 downto 0) := (others => '0');
    signal r_sn_q : signed(11 downto 0) := (others => '0');
    signal r_t_a, r_tl_a, r_th_a : unsigned(8 downto 0) := (others => '0');   -- T_GL / T_LO (+1) / T_HI (+2) addresses
    signal r_lo_q, r_hi_q, r_gl_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(280 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(318 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_cab, s_syn, s_zig, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_pp   : unsigned(15 downto 0) := (others => '0');
    signal z_ka   : unsigned(17 downto 0) := (others => '0');
    signal z_sq   : unsigned(17 downto 0) := (others => '0');
    signal z_b2   : unsigned(19 downto 0) := (others => '0');
    signal c_gp   : unsigned(17 downto 0) := to_unsigned(K_GP, 18);
    signal c_gq   : unsigned(17 downto 0) := to_unsigned(K_GQ, 18);
    signal c_aap, c_abp, c_aaq, c_abq : unsigned(15 downto 0) := (others => '0');   -- swing amplitudes, Q16
    signal c_da   : unsigned(23 downto 0) := (others => '0');          -- LFO A step per line, Q24
    signal c_db   : unsigned(23 downto 0) := (others => '0');
    signal c_ta   : unsigned(23 downto 0) := (others => '0');          -- LFO time, Q24
    signal c_tb   : unsigned(23 downto 0) := (others => '0');
    signal c_rsp  : unsigned(23 downto 0) := (others => '0');          -- ramp steps per line (negative), Q24
    signal c_rsq  : unsigned(23 downto 0) := (others => '0');
    signal c_duty : unsigned(15 downto 0) := to_unsigned(K_DUTY, 16);
    signal c_fall : unsigned(15 downto 0) := to_unsigned(K_PR16 + K_DUTY - 65536, 16);
    signal c_bg   : unsigned(18 downto 0) := (others => '0');          -- bend, Q24 phase per luma step (x2 for Osc)
    signal c_dp   : unsigned(11 downto 0) := to_unsigned(K_DP0, 12);   -- reset point to the first pixel, h
    signal c_dq   : unsigned(11 downto 0) := to_unsigned(K_DQ0, 12);
    signal c_nc1  : unsigned(10 downto 0) := to_unsigned(959, 11);     -- centre pixel - 1 (Mirror)
    signal c_neg  : std_logic := '0';

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(23 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_pa, e_pb : unsigned(23 downto 0) := (others => '0');     -- LFO phases of the line being computed
    signal e_rp, e_rq : unsigned(23 downto 0) := (others => '0');     -- ramps (two's complement Q24)
    signal e_s    : unsigned(10 downto 0) := (others => '0');          -- |sine|
    signal e_sn   : std_logic := '0';
    signal e_nap, e_nbp, e_naq, e_nbq : std_logic := '0';
    signal e_m    : signed(19 downto 0) := (others => '0');            -- m, Q16 (FM deviation)
    signal e_mgp, e_mgq : unsigned(15 downto 0) := (others => '0');    -- |m| per VCO
    signal e_mnp, e_mnq : std_logic := '0';
    signal e_fp, e_fq : unsigned(18 downto 0) := (others => '0');      -- F per h, Q24
    signal e_ta, e_ta2 : unsigned(11 downto 0) := (others => '0');
    signal dv_r   : unsigned(19 downto 0) := (others => '0');          -- Sync: FQ / FP by restoring division
    signal dv_s   : unsigned(23 downto 0) := (others => '0');
    signal dv_q   : unsigned(23 downto 0) := (others => '0');          -- the ratio, Q16
    signal n_qrd  : unsigned(23 downto 0) := (others => '0');          -- next line: Q restart running down
    signal e_d    : unsigned(15 downto 0) := (others => '0');          -- P phase since the red leading edge, first pixel
    signal l_qru, l_qrd : unsigned(23 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_ip, n_iq : unsigned(23 downto 0) := (others => '0');      -- next line: increments
    signal n_rw, l_rw : unsigned(15 downto 0) := (others => '0');     -- rim width (6 virtual px of P phase), Q16
    signal n_p0, n_q0 : unsigned(23 downto 0) := (others => '0');      -- next line: start phases

    -- beam
    signal b_p, b_q : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_dn  : std_logic := '0';
    signal l_ip, l_iq : unsigned(23 downto 0) := (others => '0');
    signal y2_d  : unsigned(15 downto 0) := (others => '0');
    signal y3_r, y4_r, y3_go, y3_gd, y2_f, y3_f, y2_dn, y3_dn, y4_dn : std_logic := '0';

    -- pixel path
    signal p2_p, p3_p, p4_p, p5_p, p6_p : unsigned(15 downto 0) := (others => '0');   -- P phase bits 23..8
    signal p2_q, p3_q, p4_q, p5_q, p6_q : unsigned(9 downto 0) := (others => '0');    -- Q phase bits 23..14
    signal p4_lm  : unsigned(7 downto 0) := (others => '0');
    signal p5_ea, p5_eb : unsigned(22 downto 0) := (others => '0');
    signal p6_b   : unsigned(23 downto 0) := (others => '0');
    signal p7_p : unsigned(15 downto 0) := (others => '0');
    signal p7_q : unsigned(9 downto 0) := (others => '0');
    signal p8_d, p8_u : unsigned(15 downto 0) := (others => '0');
    signal p9_hi, p10_hi, p11_hi, p12_hi : std_logic := '0';
    signal p9_rim, p10_rim, p11_rim, p12_rim, p13_rim : std_logic := '0';
    signal p9_e, p10_e : unsigned(3 downto 0) := (others => '0');
    signal p10_gl : std_logic_vector(31 downto 0) := (others => '0');
    signal p11_lo : std_logic_vector(31 downto 0) := (others => '0');
    signal p11_gy, p11_gu, p11_gv : signed(14 downto 0) := (others => '0');
    signal p12_y, p12_u, p12_v : signed(11 downto 0) := (others => '0');
    signal p12_hq : std_logic_vector(31 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- the boxes' x(n - 8) taps: one 256 x 64 block-RAM ring {box A Y U V, box B U V}, written every clock at
    -- s_rw, read at s_rw - 6 (EBR + fabric register = the sample 8 clocks old); never the write address
    type t_ring is array (0 to 255) of std_logic_vector(63 downto 0);
    signal ring   : t_ring := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ring : signal is true;
    signal s_rw   : unsigned(7 downto 0) := (others => '0');
    signal s_rr   : unsigned(7 downto 0) := to_unsigned(250, 8);
    signal s_rq   : std_logic_vector(63 downto 0) := (others => '0');
    signal s_rq2  : std_logic_vector(63 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

    -- black in the output filters: Y 64, U = V = 512 (x8 in the 8-sums)
    constant C_K8Y : unsigned(12 downto 0) := to_unsigned(512, 13);
    constant C_K8C : unsigned(12 downto 0) := to_unsigned(4096, 13);

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
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
            r_ex_q <= C_SM_EXP2(to_integer(r_ex_a));
            r_sn_q <= C_SM_SIN(to_integer(r_sn_a));
            r_lo_q <= C_SM_TLO(to_integer(r_tl_a));
            r_hi_q <= C_SM_THI(to_integer(r_th_a));
            r_gl_q <= C_SM_TGL(to_integer(r_t_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_d  : signed(16 downto 0);
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
            -- speed: |k4 - 512| - 8 (deadband +-8), sign
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
                s_mir <= registers_in(6)(0);
                s_cab <= registers_in(6)(1);
                s_syn <= registers_in(6)(2);
                s_zig <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
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
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g1(12 downto 3), 24);  z_fb <= to_unsigned(406, 16);     -- red exponent
                    when 1  => z_fa <= to_unsigned(K_GP, 24);          z_fb <= z_mt;                      -- GP
                    when 2  => z_fa <= resize(s_g2(12 downto 3), 24);  z_fb <= to_unsigned(406, 16);     -- blue exponent
                    when 3  => z_fa <= to_unsigned(K_GQ, 24);          z_fb <= z_mt;                      -- GQ
                    when 4  => z_fa <= resize(s_gp(12 downto 3), 24);  z_fb <= resize(s_gp(12 downto 3), 16);  -- swing^2
                    when 5  => z_fa <= resize(z_pp, 24);               z_fb <= to_unsigned(931, 16);      -- aAP
                    when 6  => z_fa <= resize(z_pp, 24);               z_fb <= to_unsigned(1016, 16);     -- aBP
                    when 7  => z_fa <= resize(z_pp, 24);               z_fb <= to_unsigned(1142, 16);     -- aAQ
                    when 8  => z_fa <= resize(z_pp, 24);               z_fb <= to_unsigned(1029, 16);     -- aBQ
                    when 9  => z_fa <= to_unsigned(K_KA, 24);          z_fb <= z_mt;                      -- kA
                    when 10 => z_fa <= resize(z_ka, 24);               z_fb <= resize(s_ys, 16);          -- dA
                    when 11 => z_fa <= resize(c_da, 24);               z_fb <= to_unsigned(K_RB, 16);     -- dB
                    when 12 => z_fa <= resize(s_vm, 24);               z_fb <= resize(s_vm, 16);          -- speed^2
                    when 13 => z_fa <= resize(z_sq, 24);               z_fb <= to_unsigned(K_V, 16);      -- vA
                    when 14 => z_fa <= resize(z_sq, 24);               z_fb <= to_unsigned(K_VB, 16);     -- vB
                    when 15 => z_fa <= to_unsigned(K_RSP, 24);         z_fb <= resize(s_ys, 16);          -- ramp P step
                    when 16 => z_fa <= to_unsigned(K_RSQ, 24);         z_fb <= resize(s_ys, 16);          -- ramp Q step
                    when 17 => z_fa <= resize(s_g6(12 downto 3), 24); z_fb <= resize(s_g6(12 downto 3), 16); -- bend^2
                    when others => z_fa <= resize(z_b2, 24);         z_fb <= to_unsigned(900, 16);      -- bend gain
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 36); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's speeds); geometry
                    when 20 =>
                        if s_mir = '1' then
                            c_dp <= to_unsigned(1920 + K_DP0, 12) - resize(s_x0, 12);
                            c_dq <= to_unsigned(1920 + K_DQ0, 12) - resize(s_x0, 12);
                        else
                            c_dp <= to_unsigned(K_DP0, 12) + resize(s_x0, 12);
                            c_dq <= to_unsigned(K_DQ0, 12) + resize(s_x0, 12);
                        end if;
                        case to_integer(s_xs) is
                            when 4      => c_nc1 <= to_unsigned(359, 11);
                            when 3      => c_nc1 <= to_unsigned(639, 11);
                            when others => c_nc1 <= to_unsigned(959, 11);
                        end case;
                    when 21 =>
                        -- red share: 0.49 + (k5 - 512) * 51 / 65536 (0.09 .. 0.89)
                        v_d := signed(resize(s_g5(12 downto 3), 17)) - 512;
                        v_d := shift_left(v_d, 5) + shift_left(v_d, 4) + shift_left(v_d, 1) + v_d + K_DUTY;
                        c_duty <= unsigned(v_d(15 downto 0));
                    when 22 =>
                        c_fall <= c_duty + to_unsigned(K_PR16, 16);

                    -- red: e = (k1 * 406 >> 9) - 406, Q8 octaves (-406 .. 405)
                    when 50  => z_e <= signed(resize(f_acc(18 downto 9), 12)) - 406;
                    when 51  => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 53  => z_mt <= r_ex_q;
                    when 74  =>                       -- GP = K_GP * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_gp <= resize(f_acc(35 downto 17), 18);
                            when -1     => c_gp <= f_acc(33 downto 16);
                            when 0      => c_gp <= f_acc(32 downto 15);
                            when others => c_gp <= f_acc(31 downto 14);
                        end case;
                    when 98  => z_e <= signed(resize(f_acc(18 downto 9), 12)) - 406;
                    when 99  => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 101 => z_mt <= r_ex_q;
                    when 122 =>
                        case to_integer(z_sh) is
                            when -2     => c_gq <= resize(f_acc(35 downto 17), 18);
                            when -1     => c_gq <= f_acc(33 downto 16);
                            when 0      => c_gq <= f_acc(32 downto 15);
                            when others => c_gq <= f_acc(31 downto 14);
                        end case;
                    when 146 => z_pp <= f_acc(19 downto 4);                       -- swing^2 >> 4
                    when 170 => c_aap <= f_acc(27 downto 12) + 480;
                    when 194 => c_abp <= f_acc(27 downto 12) + 524;
                    when 218 => c_aaq <= f_acc(27 downto 12) + 588;
                    when 219 =>                       -- wave: e = k3 - 512, Q8 octaves (-512 .. 511)
                        v_e := signed(resize(s_g3(12 downto 3), 12)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 221 => z_mt <= r_ex_q;
                    when 242 => c_abq <= f_acc(27 downto 12) + 530;
                    when 266 =>                       -- kA = K_KA * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => z_ka <= resize(f_acc(35 downto 17), 18);
                            when -1     => z_ka <= f_acc(33 downto 16);
                            when 0      => z_ka <= f_acc(32 downto 15);
                            when others => z_ka <= f_acc(31 downto 14);
                        end case;
                    when 290 => c_da <= f_acc(31 downto 8);
                    when 314 => c_db <= resize(f_acc(35 downto 16), 24);
                    when 338 => z_sq <= f_acc(17 downto 0);
                    when 362 =>                       -- LFO A time += -speed (Q24 per field)
                        if s_vneg = '1' then c_ta <= c_ta - f_acc(31 downto 8); else c_ta <= c_ta + f_acc(31 downto 8); end if;
                    when 386 =>
                        if s_vneg = '1' then c_tb <= c_tb - f_acc(31 downto 8); else c_tb <= c_tb + f_acc(31 downto 8); end if;
                    when 410 => c_rsp <= (not resize(f_acc(31 downto 8), 24)) + 1;
                    when 434 => c_rsq <= (not resize(f_acc(31 downto 8), 24)) + 1;
                    when 458 => z_b2 <= f_acc(19 downto 0);
                    when 482 =>                       -- bend = k6^2 * 900 >> 12; the Osc generator is patched in at
                                                      -- twice the camera's level
                        if s_src = '1' then c_bg <= f_acc(29 downto 11); else c_bg <= f_acc(30 downto 12); end if;
                    when others => null;
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
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result
    -- readable at load + 17):
    --   s   = sin (or Zigzag triangle) of the LFO phases + the four offsets
    --   m   = ramp + (aA |sA| +- aB |sB|) >> 11            (Q16, per VCO)
    --   F   = G +- G |m| >> 16                             (Q24 per h)
    --   inc = F * xs,  start = theta + F * D               (D = reset point to the first pixel, h)
    -- ~145 clocks, so it fits SD's 858-clock line.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_w   : unsigned(11 downto 0);
        variable v_a   : unsigned(11 downto 0);
        variable v_t   : signed(14 downto 0);
        variable v_s   : signed(11 downto 0);
        variable v_ph  : unsigned(23 downto 0);
        variable v_pr  : signed(19 downto 0);
        variable v_f   : unsigned(18 downto 0);
        variable v_i   : unsigned(23 downto 0);
        variable v_dr  : unsigned(20 downto 0);
        variable v_dt  : unsigned(21 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- LFO lookups just before each product: address at lc 2 / 19 / 36 / 53, |s| valid 3 clocks
            -- later (e_ta2 is the phase for Zigzag)
            case to_integer(e_lc(5 downto 4)) is
                when 0      => v_ph := e_pa + to_unsigned(K_AAP, 24);
                when 1      => v_ph := e_pb + to_unsigned(K_ABP, 24);
                when 2      => v_ph := e_pa + to_unsigned(K_AAQ, 24);
                when others => v_ph := e_pb + to_unsigned(K_ABQ, 24);
            end case;
            r_sn_a <= v_ph(23 downto 14);
            e_ta   <= v_ph(23 downto 12);
            e_ta2  <= e_ta;
            -- |s| and its sign, valid 3 clocks after the address clock (sine: ROM; Zigzag: triangle of the
            -- phase, t = 2047 - 4 |w - 2048|, w = phase - 1/4, clipped to +-2047)
            if s_zig = '1' then
                v_w := e_ta2 - to_unsigned(1024, 12);
                if v_w(11) = '1' then v_a := v_w - to_unsigned(2048, 12); else v_a := to_unsigned(2048, 12) - v_w; end if;
                v_t := to_signed(2047, 15) - shift_left(signed(resize(v_a, 15)), 2);
                if v_t < -2047 then v_t := to_signed(-2047, 15); end if;
                v_s := v_t(11 downto 0);
            else
                v_s := r_sn_q;
            end if;
            if v_s(11) = '1' then e_s <= unsigned(not v_s(10 downto 0)) + 1; e_sn <= '1';
            else e_s <= unsigned(v_s(10 downto 0)); e_sn <= '0'; end if;


            case to_integer(e_lc) is
                when 5   => m_a <= resize(c_aap, 36); m_b <= resize(e_s, 16); m_acc <= (others => '0'); e_nap <= e_sn;
                when 22  =>                                  -- P: m = ramp +- aAP |sAP| >> 11
                    v_pr := signed(resize(m_acc(26 downto 11), 20));
                    if e_nap = '1' then v_pr := -v_pr; end if;
                    e_m <= resize(signed(e_rp(23 downto 8)), 20) + v_pr;
                    m_a <= resize(c_abp, 36); m_b <= resize(e_s, 16); m_acc <= (others => '0'); e_nbp <= e_sn;
                when 39  =>                                  -- +- aBP |sBP| >> 11
                    v_pr := signed(resize(m_acc(26 downto 11), 20));
                    if e_nbp = '1' then v_pr := -v_pr; end if;
                    e_m <= e_m + v_pr;
                    m_a <= resize(c_aaq, 36); m_b <= resize(e_s, 16); m_acc <= (others => '0'); e_naq <= e_sn;
                when 40  =>
                    if e_m < 0 then e_mgp <= unsigned(resize(-e_m, 16)); e_mnp <= '1';
                    else e_mgp <= unsigned(resize(e_m, 16)); e_mnp <= '0'; end if;
                when 56  =>                                  -- Q: m = ramp +- aAQ |sAQ| >> 11
                    v_pr := signed(resize(m_acc(26 downto 11), 20));
                    if e_naq = '1' then v_pr := -v_pr; end if;
                    e_m <= resize(signed(e_rq(23 downto 8)), 20) + v_pr;
                    m_a <= resize(c_abq, 36); m_b <= resize(e_s, 16); m_acc <= (others => '0'); e_nbq <= e_sn;
                when 73  =>                                  -- +- aBQ |sBQ| >> 11; FP product
                    v_pr := signed(resize(m_acc(26 downto 11), 20));
                    if e_nbq = '1' then v_pr := -v_pr; end if;
                    e_m <= e_m + v_pr;
                    m_a <= resize(c_gp, 36); m_b <= e_mgp; m_acc <= (others => '0');
                when 74  =>
                    if e_m < 0 then e_mgq <= unsigned(resize(-e_m, 16)); e_mnq <= '1';
                    else e_mgq <= unsigned(resize(e_m, 16)); e_mnq <= '0'; end if;
                when 90  =>                                  -- FP = GP +- GP |mP| >> 16; FQ product
                    if e_mnp = '1' then v_f := resize(c_gp, 19) - resize(m_acc(33 downto 16), 19);
                    else v_f := resize(c_gp, 19) + resize(m_acc(33 downto 16), 19); end if;
                    e_fp <= v_f;
                    m_a <= resize(c_gq, 36); m_b <= e_mgq; m_acc <= (others => '0');
                when 91  =>                                  -- inc P = FP * xs
                    case to_integer(s_xs) is
                        when 4      => v_i := shift_left(resize(e_fp, 24), 2);
                        when 3      => v_i := shift_left(resize(e_fp, 24), 1) + resize(e_fp, 24);
                        when others => v_i := shift_left(resize(e_fp, 24), 1);
                    end case;
                    n_ip <= v_i;
                    -- rim width = FP * 12 h >> 8 (Q16 phase of 6 virtual px)
                    n_rw <= resize(shift_right(shift_left(resize(e_fp, 24), 3) + shift_left(resize(e_fp, 24), 2), 8), 16);
                when 107 =>                                  -- FQ; start P product
                    if e_mnq = '1' then v_f := resize(c_gq, 19) - resize(m_acc(33 downto 16), 19);
                    else v_f := resize(c_gq, 19) + resize(m_acc(33 downto 16), 19); end if;
                    e_fq <= v_f;
                    m_a <= resize(e_fp, 36); m_b <= resize(c_dp, 16); m_acc <= (others => '0');
                when 108 =>
                    case to_integer(s_xs) is
                        when 4      => v_i := shift_left(resize(e_fq, 24), 2);
                        when 3      => v_i := shift_left(resize(e_fq, 24), 1) + resize(e_fq, 24);
                        when others => v_i := shift_left(resize(e_fq, 24), 1);
                    end case;
                    n_iq <= v_i;
                when 124 =>                                  -- start P = FP * DP (theta P = 0); start Q product
                    n_p0 <= m_acc(23 downto 0);
                    m_a <= resize(e_fq, 36); m_b <= resize(c_dq, 16); m_acc <= (others => '0');
                when 125 =>
                    e_d <= n_p0(23 downto 8) - to_unsigned(K_PR16, 16);
                when 134 =>                                  -- down restart: theta Q + one P cycle of Q - 4 px
                    n_qrd <= to_unsigned(K_THQ, 24) + shift_left(dv_q, 8) - shift_left(n_iq, 2);
                when 141 =>                                  -- start Q = theta Q + FQ * DQ; Sync product d * FQ / FP,
                                                             -- d = P phase since the red leading edge at the first pixel
                    n_q0 <= m_acc(23 downto 0) + to_unsigned(K_THQ, 24);
                    m_a <= resize(dv_q, 36); m_b <= e_d; m_acc <= (others => '0');
                when 158 =>                                  -- Sync start replaces it: theta Q + d * FQ / FP
                    if s_syn = '1' then n_q0 <= to_unsigned(K_THQ, 24) + m_acc(31 downto 8); end if;
                when others => null;
            end case;

            -- Sync ratio FQ / FP (Q16): restoring division, loaded at lc 108, 24 iterations at lc 109 .. 132
            if e_lc = 108 then
                dv_r <= resize(e_fq(18 downto 8), 20);
                dv_s <= e_fq(7 downto 0) & x"0000";
                dv_q <= (others => '0');
            elsif e_lc >= 109 and e_lc <= 132 then
                v_dr := dv_r & dv_s(23);
                v_dt := ('0' & v_dr) - ("00" & e_fp);          -- trial subtraction; bit 21 = borrow
                if v_dt(21) = '0' then
                    dv_r <= v_dt(19 downto 0); dv_q <= dv_q(22 downto 0) & '1';
                else
                    dv_r <= v_dr(19 downto 0); dv_q <= dv_q(22 downto 0) & '0';
                end if;
                dv_s <= dv_s(22 downto 0) & '0';
            end if;

            -- per-line DDAs: line-0 values at the zone kick, one step per line after the line's values
            if e_zk = '1' then
                e_pa <= (not c_ta) + 1; e_pb <= (not c_tb) + 1;
                e_rp <= to_unsigned(K_R0P, 24); e_rq <= to_unsigned(K_R0Q, 24);
            elsif e_lc = 142 then
                e_pa <= e_pa + c_da; e_pb <= e_pb + c_db;
                e_rp <= e_rp + c_rsp; e_rq <= e_rq + c_rsq;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the two phase DDAs.  Mirror: both run down from the left edge to the
    -- centre pixel and up after it (the reset point sits at the centre).  Sync:
    -- VCO Q restarts at theta Q four pixels after each red ribbon's leading edge
    -- (in scan order, detected on the unbent P phase), and starts each line at
    -- the phase it would have after the last off-screen edge.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_ip, v_iq : unsigned(23 downto 0);
        variable v_s25      : unsigned(24 downto 0);
    begin
        if rising_edge(clk) then
            -- sync edge detector (pixel of b_p: stage 1; d at 2, bit at 3, edge at 3 vs 4)
            y2_d  <= b_p(23 downto 8) - to_unsigned(K_PR16, 16);
            if y2_d < c_duty then y3_r <= '1'; else y3_r <= '0'; end if;
            y4_r  <= y3_r;
            if b_px = 0 and g_avid = '1' then y2_f <= '1'; else y2_f <= '0'; end if;   -- the line's first pixel
            y3_f  <= y2_f;
            y2_dn <= b_dn; y3_dn <= y2_dn; y4_dn <= y3_dn;      -- y4_dn: direction of the step into the y3 pixel
            -- restarts (the first pixel is never an edge): up = the red leading edge, down = the other edge
            y3_go <= s_syn and y3_r and not y4_r and not y3_f and not y4_dn;
            y3_gd <= s_syn and y4_r and not y3_r and not y3_f and y4_dn;
            if g_start = '1' then
                b_p  <= n_p0;
                b_q  <= n_q0;
                b_px <= (others => '0');
                b_dn <= s_mir;
                l_ip <= n_ip; l_iq <= n_iq; l_rw <= n_rw; l_qrd <= n_qrd;
                l_qru <= to_unsigned(K_THQ, 24) + shift_left(n_iq, 2);   -- up restart, 4 px after the edge
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_dn = '1' then v_ip := not l_ip; v_iq := not l_iq; else v_ip := l_ip; v_iq := l_iq; end if;
                v_s25 := (b_p & '1') + (v_ip & b_dn);
                b_p <= v_s25(24 downto 1);
                v_s25 := (b_q & '1') + (v_iq & b_dn);
                if y3_go = '1' then
                    b_q <= l_qru;
                elsif y3_gd = '1' then
                    b_q <= l_qrd;
                else
                    b_q <= v_s25(24 downto 1);
                end if;
                if b_px = c_nc1 then b_dn <= '0'; end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage n = valid at the n-th clock after g_start; stage 1 = beam)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y, v_u, v_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2..6: carry the phases while the bend is formed
            p2_p <= b_p(23 downto 8);  p2_q <= b_q(23 downto 14);
            p3_p <= p2_p; p3_q <= p2_q;
            p4_p <= p3_p; p4_q <= p3_q;
            p5_p <= p4_p; p5_q <= p4_q;
            p6_p <= p5_p; p6_q <= p5_q;
            -- 4: luma from the line buffer (EBR data at 3, fabric copy)
            p4_lm <= unsigned(s_lb_q);
            -- 5: bend partials (luma nibbles x gain)
            p5_ea <= resize(p4_lm(3 downto 0) * c_bg, 23);
            p5_eb <= resize(p4_lm(7 downto 4) * c_bg, 23);
            -- 6: bend = luma * gain (mod one cycle)
            p6_b  <= resize(p5_ea, 24) + shift_left(resize(p5_eb, 24), 4);
            -- 7: bent phases (Q gets half), at the widths the palette needs
            p7_p  <= p6_p + p6_b(23 downto 8);
            p7_q  <= p6_q + p6_b(23 downto 15);
            -- 8: offsets from the red edges; palette address
            p8_d  <= p7_p - to_unsigned(K_PR16, 16);
            p8_u  <= p7_p - c_fall;
            r_t_a <= s_cab & p7_q(9 downto 2);
            -- 9: red bit, gloss envelope (1 - 4u over the first quarter cycle after the trailing edge)
            if p8_d < c_duty then p9_hi <= '1'; else p9_hi <= '0'; end if;
            if p8_d < l_rw then p9_rim <= '1'; else p9_rim <= '0'; end if;   -- leading-edge rim
            if p8_u(15 downto 14) = "00" then p9_e <= not p8_u(13 downto 10); else p9_e <= (others => '0'); end if;
            -- 10: gloss ROM data (EBR at 9), fabric copy
            r_tl_a <= r_t_a; r_th_a <= r_tl_a;   -- T_LO data at 10, T_HI data at 11
            p10_gl <= r_gl_q;
            p10_hi <= p9_hi; p10_e <= p9_e; p10_rim <= p9_rim;
            -- 11: gloss products (signed 10 x 4)
            p11_gy <= signed(p10_gl(31 downto 22)) * signed('0' & p10_e);
            p11_gu <= signed(p10_gl(21 downto 12)) * signed('0' & p10_e);
            p11_gv <= signed(p10_gl(11 downto 2)) * signed('0' & p10_e);
            p11_lo <= r_lo_q; p11_hi <= p10_hi; p11_rim <= p10_rim;
            -- 12: low palette + gloss / 8
            p12_y <= signed(resize(unsigned(p11_lo(31 downto 22)), 12)) + resize(shift_right(p11_gy, 3), 12);
            p12_u <= signed(resize(unsigned(p11_lo(21 downto 12)), 12)) + resize(shift_right(p11_gu, 3), 12);
            p12_v <= signed(resize(unsigned(p11_lo(11 downto 2)), 12)) + resize(shift_right(p11_gv, 3), 12);
            p12_hq <= r_hi_q; p12_hi <= p11_hi; p12_rim <= p11_rim;
            -- 13: red half or clamped low half
            p13_rim <= p12_rim;
            if p12_hi = '1' then
                p13_y <= unsigned(p12_hq(31 downto 22)); p13_u <= unsigned(p12_hq(21 downto 12)); p13_v <= unsigned(p12_hq(11 downto 2));
            else
                v_y := p12_y; v_u := p12_u; v_v := p12_v;
                if v_y < 64 then p13_y <= to_unsigned(64, 10); elsif v_y > 1000 then p13_y <= to_unsigned(1000, 10);
                else p13_y <= unsigned(v_y(9 downto 0)); end if;
                if v_u < 24 then p13_u <= to_unsigned(24, 10); elsif v_u > 1000 then p13_u <= to_unsigned(1000, 10);
                else p13_u <= unsigned(v_u(9 downto 0)); end if;
                if v_v < 24 then p13_v <= to_unsigned(24, 10); elsif v_v > 1000 then p13_v <= to_unsigned(1000, 10);
                else p13_v <= unsigned(v_v(9 downto 0)); end if;
            end if;
            -- 14: the rim: half way to a pale lavender (923, 517, 585) on the red ribbons' leading edges
            if p13_rim = '1' then
                p14_y <= resize(shift_right(resize(p13_y, 11) + 923, 1), 10);
                p14_u <= resize(shift_right(resize(p13_u, 11) + 517, 1), 10);
                p14_v <= resize(shift_right(resize(p13_v, 11) + 585, 1), 10);
            else
                p14_y <= p13_y; p14_u <= p13_u; p14_v <= p13_v;
            end if;
            -- 15: black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= p14_y; s_u15 <= p14_u; s_v15 <= p14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, Y
    -- through one (each box two adjacent 8-sums: the still's soft chroma and
    -- ~6 px luma edges, and the HD HDMI bar fix), luma 13/16 and the blanking
    -- gate.  Luma leads chroma by 4 px, as the camera's chroma trails in the still.
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            ring(to_integer(s_rw)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15) & std_logic_vector(s_v15)
                                      & std_logic_vector(s_u17) & std_logic_vector(s_v17) & "00000000000000";
            s_rq <= ring(to_integer(s_rr));
        end if;
    end process p_ring;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ay, v_au, v_av, v_bu, v_bv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_rw <= s_rw + 1; s_rr <= s_rr + 1;
            s_rq2 <= s_rq;
            v_ay := unsigned(s_rq2(63 downto 54)); v_au := unsigned(s_rq2(53 downto 44)); v_av := unsigned(s_rq2(43 downto 34));
            v_bu := unsigned(s_rq2(33 downto 24)); v_bv := unsigned(s_rq2(23 downto 14));
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once 8 blank samples
            -- are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            if s_bah = "00000000" then
                s_bay <= C_K8Y; s_bau <= C_K8C; s_bav <= C_K8C;
            else
                s_bay <= s_bay + s_y15 - v_ay;
                s_bau <= s_bau + s_u15 - v_au;
                s_bav <= s_bav + s_v15 - v_av;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B (chroma only; luma takes box A alone, the still's sharper luma edges): stage 18 sums,
            -- stage 19 output.  Luma: two plain stages.
            if s_bbh = "00000000" then
                s_bbu <= C_K8C; s_bbv <= C_K8C;
            else
                s_bbu <= s_bbu + s_u17 - v_bu;
                s_bbv <= s_bbv + s_v17 - v_bv;
            end if;
            s_bbud <= s_bbu; s_bbvd <= s_bbv;
            s_y18 <= s_y17; s_y19 <= s_y18;
            v_sc := resize(s_bbu, 14) + s_bbud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v19 <= v_sc(13 downto 4);
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21: blanking gate
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

end architecture streamers;
