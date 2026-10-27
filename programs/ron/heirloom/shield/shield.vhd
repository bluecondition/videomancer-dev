-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Shield v0.5.0 -- Heirloom #38, rebuilt from the photo WIN_20230529_22_56_41.
--
-- The photo: two shields side by side (axes 941.7 px apart), each a nest of U-shaped bands with
-- vertical walls at the top and a V tip at the bottom, cut by horizontal stripes (73.5-line half
-- period) that turn the bands into an orange / red plaid: orange rows yellow / orange / yellow /
-- orange / red, red rows orange-red / red / crimson / dark red / orange-red.  Around each shield a
-- dark-red ramp, then a navy -> blue ramp, a thin white line on the left arm, then the red M body,
-- bright blue triangles at the bottom, blue pillars by the seams and a column of stripes at each
-- seam.  Soft horizontal edges (10-90 % ~14 px), 1 px line wobble, no noise.
--
-- The patch, as fitted (shield.py has every constant):
--   u   = H triangle about the shield axes, held at its x = 66 value at the line start, tips rounded
--   F2  = u + depth * (V parabola)          nested U shields
--   F1  = A - 2.89 u - V ramp               a steep inverted triangle: the pillars
--   F   = min(F2, F1) + etch * luma         (non-additive mix), wrapped at 1024 px
--   colouriser: bands n = F_k * gain (keyer path, 10 px late) in a stripe-selected table, the
--   dark-red and blue ramps (linear path), the M body and the triangles (keyer); a white one-shot where
--   F_k falls through 530 after exceeding 552 (faded on the steep F1 side by the line's crease
--   peak), a dark one-shot where F rises out of the stripe zone.
--
--   K1  Shields   shields across (0.87 .. 6.5; the photo 2.04)
--   K2  Bands     nested bands per shield (plain stripes .. x3.6)
--   K3  Stripes   stripe count (one stripe .. 26 per frame; the photo 7.3)
--   K4  Speed     bipolar: bands stream out of the tips, the stripes roll
--   K5  Pillars   how far the pillars (F1) climb: none .. they take over
--   K6  Etch      how far the camera (or the Osc) bends the bands
--   S7  Line      the white / dark one-shot lines
--   S8  Tincture  the photo / Azure (chroma inverted)
--   S9  Flip      shields upside down
--   S10 Stripes   on (the photo) / off
--   S11 Source    Video / Osc
--   P12 DEPTH     V parabola: flat banners (0) .. the photo (365) .. deep cascading V's (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each
-- line start that computes the NEXT line (V parabola * depth, F1's line term, crease peak, stripe bit,
-- wobble), a phase DDA beam, a 30-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre
-- 4:3).  Generated avid; output conditioning (Y, U, V through a 17-tap and a 9-tap box: the photo's
-- softness and the HD HDMI bar fix; luma 13/16) before the blanking gate.  u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.shield_pkg.all;
use work.all;

architecture shield of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks (output register = stage 32)

    -- fitted constants (F in quarter px)
    constant C_PH960 : integer := 7728900;    -- tile phase at virtual x = 960 (Q24)
    constant C_PS540 : integer := 13218590;   -- stripe phase at virtual line 540 (Q24)
    constant C_TZ    : integer := 1711;       -- stripe zone end (ramp frame)
    constant C_TM    : integer := 2567;       -- M body (keyer frame)
    constant C_TT    : integer := 3322;       -- bright blue triangles
    constant C_LW    : integer := 2120;       -- white line level
    constant C_LH    : integer := 2208;       -- white line arming level
    constant C_A1K   : integer := 6966;       -- F1 offset
    constant C_FC0   : integer := 2345;       -- crease-peak fade thresholds (white line on F1)
    constant C_FC1   : integer := 2450;
    constant C_FC2   : integer := 2555;
    constant C_FC3   : integer := 2660;

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
    -- output lines per 540 virtual lines, Q1 (= 276480 / vstep)
    function f_n540(t : std_logic_vector(3 downto 0)) return unsigned is
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

    -- palette ROM (registered address and data)
    signal r_pa : unsigned(8 downto 0) := (others => '0');
    signal r_pq : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_ysf  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_n540 : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- mode constants (latched per field)
    signal c_skx  : unsigned(12 downto 0) := to_unsigned(3056, 13);
    signal c_npx  : unsigned(10 downto 0) := to_unsigned(1787, 11);
    signal c_nh   : unsigned(6 downto 0) := to_unsigned(54, 7);
    signal c_dsel : unsigned(1 downto 0) := "00";              -- keyer delay 10 / 7 / 5
    signal c_lww  : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal c_ldw  : unsigned(2 downto 0) := to_unsigned(3, 3);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(213 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(320 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(365 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_line, s_pal, s_flip, s_strp, s_src : std_logic := '0';

    -- per-field constants
    signal z_a    : unsigned(22 downto 0) := (others => '0');
    signal c_step : unsigned(16 downto 0) := to_unsigned(17810, 17);     -- tile phase per pixel (Q24)
    signal c_phi0 : unsigned(23 downto 0) := to_unsigned(16029365, 24);  -- phase at the hold point
    signal c_gb   : unsigned(12 downto 0) := to_unsigned(1404, 13);      -- band gain
    signal c_fs   : unsigned(22 downto 0) := to_unsigned(114063, 23);    -- stripe phase per line (Q24)
    signal c_ps0  : unsigned(23 downto 0) := (others => '0');            -- stripe phase at line 0
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_dfl  : unsigned(13 downto 0) := (others => '0');
    signal c_drl  : unsigned(17 downto 0) := (others => '0');
    signal c_fl   : unsigned(23 downto 0) := (others => '0');            -- band flow, Q16 bands
    signal c_rl   : unsigned(23 downto 0) := (others => '0');            -- stripe roll, Q24 cycles
    signal c_a1   : unsigned(13 downto 0) := to_unsigned(C_A1K, 14);
    signal c_eg   : unsigned(7 downto 0) := to_unsigned(26, 8);
    signal c_d10  : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal c_fl8  : unsigned(15 downto 0) := (others => '0');

    -- field serial multiplier
    signal f_a   : unsigned(39 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(39 downto 0) := (others => '0');
    signal z_fa  : unsigned(39 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_yq   : unsigned(14 downto 0) := (others => '0');
    signal e_dyq  : unsigned(13 downto 0) := (others => '0');
    signal e_dneg : std_logic := '0';
    signal e_g    : unsigned(12 downto 0) := (others => '0');
    signal e_qt   : unsigned(13 downto 0) := (others => '0');
    signal e_c1   : unsigned(13 downto 0) := (others => '0');
    signal e_fc   : unsigned(14 downto 0) := (others => '0');
    signal e_ps   : unsigned(23 downto 0) := (others => '0');
    signal e_lf   : std_logic_vector(15 downto 0) := x"ACE1";
    signal e_j    : signed(2 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    signal n_g    : unsigned(12 downto 0) := (others => '0');
    signal n_c1   : unsigned(13 downto 0) := (others => '0');
    signal n_w1   : unsigned(2 downto 0) := (others => '0');
    signal n_s    : std_logic := '0';
    signal n_phs  : unsigned(23 downto 0) := (others => '0');
    signal l_g    : unsigned(12 downto 0) := (others => '0');
    signal l_c1   : unsigned(13 downto 0) := (others => '0');
    signal l_w1   : unsigned(2 downto 0) := (others => '0');
    signal l_s    : std_logic := '0';
    signal l_phs  : unsigned(23 downto 0) := (others => '0');

    -- beam
    signal b_phi  : unsigned(23 downto 0) := (others => '0');
    signal b_hold : unsigned(6 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal p1_tri : unsigned(10 downto 0) := (others => '0');
    signal p2_a, p2_b : unsigned(10 downto 0) := (others => '0');
    signal p3_u, p4_u : unsigned(10 downto 0) := (others => '0');
    signal p4_tip : unsigned(4 downto 0) := (others => '0');
    signal p4_tv  : std_logic := '0';
    signal p5_ur  : unsigned(10 downto 0) := (others => '0');
    signal p6_f2  : unsigned(13 downto 0) := (others => '0');
    signal p6_s3, p6_sb : unsigned(12 downto 0) := (others => '0');
    signal p7_f2  : unsigned(13 downto 0) := (others => '0');
    signal p7_s1u : unsigned(12 downto 0) := (others => '0');
    signal p8_f2  : unsigned(13 downto 0) := (others => '0');
    signal p8_f1  : signed(14 downto 0) := (others => '0');
    signal p9_f   : unsigned(13 downto 0) := (others => '0');
    signal p9_f1s : std_logic := '0';
    signal p3_lm, p4_lm, p5_lm, p6_lm : unsigned(7 downto 0) := (others => '0');
    signal p7_ea, p7_eb : unsigned(11 downto 0) := (others => '0');
    signal p8_le  : unsigned(15 downto 0) := (others => '0');
    signal p9_et  : unsigned(11 downto 0) := (others => '0');
    signal p10_fw : unsigned(11 downto 0) := (others => '0');
    signal p10_f1s, p11_f1s, p12_f1s : std_logic := '0';
    signal p11_z, p11_m, p11_t, p11_lb, p11_lh, p11_dz : std_logic := '0';
    signal p11_d  : signed(12 downto 0) := (others => '0');
    signal p11_pa, p11_pb : unsigned(16 downto 0) := (others => '0');
    signal p11_pc : unsigned(14 downto 0) := (others => '0');
    signal p12_z, p12_m, p12_t, p12_lb, p12_lh, p12_dz : std_logic := '0';
    signal p12_ri : unsigned(6 downto 0) := (others => '0');
    signal p12_pr : unsigned(24 downto 0) := (others => '0');
    signal p13_z, p13_m, p13_t, p13_lb, p13_lh, p13_dz, p13_f1s : std_logic := '0';
    signal p13_ri : unsigned(6 downto 0) := (others => '0');
    signal p13_n  : unsigned(2 downto 0) := (others => '0');
    type t_kd is array (0 to 9) of std_logic_vector(7 downto 0);
    signal p_kd   : t_kd := (others => (others => '0'));
    signal p14_k  : std_logic_vector(7 downto 0) := (others => '0');
    signal p14_z, p14_dz, p14_zp : std_logic := '0';
    signal p14_ri : unsigned(6 downto 0) := (others => '0');
    signal p15_arm : std_logic := '0';
    signal p15_cnt, p15_dcnt : unsigned(2 downto 0) := (others => '0');
    signal p15_ln, p15_dk : std_logic := '0';
    signal p15_w  : unsigned(2 downto 0) := (others => '0');
    signal p16_ln, p16_dk, p17_ln, p17_dk, p18_ln, p18_dk : std_logic := '0';
    signal p16_w, p17_w, p18_w : unsigned(2 downto 0) := (others => '0');
    signal p17_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p18_dy, p18_du, p18_dv : signed(10 downto 0) := (others => '0');
    signal p18_ey, p18_eu, p18_ev : signed(10 downto 0) := (others => '0');
    signal p19_y, p19_u, p19_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p19_ky, p19_ku, p19_kv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p19_wy, p19_wu, p19_wv : signed(12 downto 0) := (others => '0');
    signal p19_ln, p19_dk : std_logic := '0';
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_wy, c_wu, c_wv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- box delay lines: 256-word block-RAM rings ({Y, U, V, avid, avid}, written every clock; the read
    -- address trails the write address, so they never collide: no_rw_check).  The two avid copies keep
    -- all 32 lanes live (a 30-bit word would map as a 16 + 14 pair) and gate the per-line re-anchor.
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal ring_a, ring_b : t_ring := (others => x"10200800");
    signal s_wa_a, s_ra_a, s_wa_b, s_ra_b : unsigned(7 downto 0) := (others => '0');
    signal s_rq_a, s_rq_b : std_logic_vector(31 downto 0) := x"10200800";
    signal s_qa, s_qb : std_logic_vector(31 downto 0) := x"10200800";
    attribute no_rw_check : boolean;
    attribute no_rw_check of ring_a : signal is true;
    attribute no_rw_check of ring_b : signal is true;
    signal s_bay, s_bayd : unsigned(13 downto 0) := to_unsigned(1024, 14);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(13 downto 0) := to_unsigned(8192, 14);
    signal s_bby, s_bbyd : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bk  : unsigned(5 downto 0) := (others => '0');     -- blank clocks at the box input
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_u10p is array (26 to 31) of unsigned(9 downto 0);
    signal s_py, s_pu, s_pv : t_u10p := (others => to_unsigned(512, 10));
    signal s_yo : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uo, s_vo : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and palette ROM
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_ring_a : process(clk)
    begin
        if rising_edge(clk) then
            s_rq_a <= ring_a(to_integer(s_ra_a));
            ring_a(to_integer(s_wa_a)) <= std_logic_vector(s_y20) & std_logic_vector(s_u20) & std_logic_vector(s_v20) & s_gav_sr(19) & s_gav_sr(19);
        end if;
    end process p_ring_a;

    p_ring_b : process(clk)
    begin
        if rising_edge(clk) then
            s_rq_b <= ring_b(to_integer(s_ra_b));
            ring_b(to_integer(s_wa_b)) <= std_logic_vector(s_y22) & std_logic_vector(s_u22) & std_logic_vector(s_v22) & s_gav_sr(21) & s_gav_sr(21);
        end if;
    end process p_ring_b;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_pq <= C_SH_PAL(to_integer(r_pa));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_t  : unsigned(39 downto 0);
        variable v_d  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_tid  <= registers_in(8)(3 downto 0);
            s_ys   <= f_vstep_y(s_tid);
            s_n540 <= f_n540(s_tid);
            s_xsr  <= f_vstep_x(s_tid);

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
            elsif s_zc /= 1023 then
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
                s_line <= not registers_in(6)(0);       -- S7 Line: on by default (switch up = off)
                s_pal  <= registers_in(6)(1);
                s_flip <= registers_in(6)(2);
                s_strp <= not registers_in(6)(3);       -- S10 Stripes: on by default
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_ysf  <= s_ys;
                case to_integer(s_xsr) is
                    when 4 =>
                        c_skx <= to_unsigned(6112, 13); c_npx <= to_unsigned(695, 11); c_nh <= to_unsigned(0, 7);
                        c_dsel <= "10"; c_lww <= to_unsigned(2, 3); c_ldw <= to_unsigned(2, 3); s_oxs <= to_unsigned(204, 9);
                    when 3 =>
                        c_skx <= to_unsigned(4584, 13); c_npx <= to_unsigned(1191, 11); c_nh <= to_unsigned(32, 7);
                        c_dsel <= "01"; c_lww <= to_unsigned(3, 3); c_ldw <= to_unsigned(2, 3); s_oxs <= to_unsigned(153, 9);
                    when others =>
                        c_skx <= to_unsigned(3056, 13); c_npx <= to_unsigned(1787, 11); c_nh <= to_unsigned(54, 7);
                        c_dsel <= "00"; c_lww <= to_unsigned(4, 3); c_ldw <= to_unsigned(3, 3); s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters
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
                        elsif s_gd > 0 then
                            s_gd <= to_signed(1, 15);
                        elsif s_gd < 0 then
                            s_gd <= to_signed(-1, 15);
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
            -- load + 17 (stored at 50 + 24 k).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k.
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
                    when 0  => z_fa <= resize(s_g1(12 downto 3), 40) + 160;  z_fb <= resize(c_skx, 16);   -- step
                    when 1  => z_fa <= resize(c_step, 40);                   z_fb <= resize(c_npx, 16);   -- phi0
                    when 2  => z_fa <= resize(s_g2(12 downto 3), 40) + 64;   z_fb <= resize(s_g2(12 downto 3), 16) + 64;  -- bands^2
                    when 3  => z_fa <= resize(z_a, 40);                      z_fb <= to_unsigned(4440, 16);
                    when 4  => z_fa <= resize(s_g3(12 downto 3), 40) + 64;   z_fb <= resize(s_g3(12 downto 3), 16) + 64;  -- stripes^2
                    when 5  => z_fa <= resize(z_a, 40);                      z_fb <= resize(s_ysf, 16);
                    when 6  => z_fa <= resize(z_a, 40);                      z_fb <= to_unsigned(22531, 16);
                    when 7  => z_fa <= resize(c_fs, 40);                     z_fb <= resize(s_n540, 16);
                    when 8  => z_fa <= resize(s_vm, 40);                     z_fb <= resize(s_vm, 16);    -- speed^2
                    when 9  => z_fa <= resize(c_vsq, 40);                    z_fb <= to_unsigned(2263, 16);
                    when 10 => z_fa <= resize(c_vsq, 40);                    z_fb <= to_unsigned(141, 16);
                    when 11 =>                                                                           -- pillars
                        if s_g5(12 downto 3) >= 512 then z_fa <= resize(s_g5(12 downto 3) - 512, 40);
                        else z_fa <= resize(512 - s_g5(12 downto 3), 40); end if;
                        z_fb <= to_unsigned(410, 16);
                    when 12 => z_fa <= resize(s_g6(12 downto 3), 40);        z_fb <= to_unsigned(86, 16);  -- etch
                    when others => z_fa <= resize(s_gp(12 downto 3), 40);    z_fb <= to_unsigned(2873, 16); -- depth
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
                    -- motion (last field's steps)
                    when 20 =>
                        if s_vneg = '1' then c_fl <= c_fl + resize(c_dfl, 24); c_rl <= c_rl + resize(c_drl, 24);
                        else c_fl <= c_fl - resize(c_dfl, 24); c_rl <= c_rl - resize(c_drl, 24); end if;
                    when 21 => c_fl8 <= c_fl(23 downto 8);
                    when 50  => c_step <= f_acc(22 downto 6);
                    when 74  =>
                        v_t := f_acc;
                        c_phi0 <= to_unsigned(C_PH960, 24) - v_t(24 downto 1);
                    when 98  => z_a <= f_acc(22 downto 0);
                    when 122 => c_gb <= f_acc(32 downto 20);
                    when 146 => z_a <= f_acc(22 downto 0);
                    when 170 => z_a <= f_acc(30 downto 8);
                    when 194 => c_fs <= f_acc(38 downto 16);
                    when 218 =>
                        v_t := f_acc;
                        c_ps0 <= to_unsigned(C_PS540, 24) - v_t(24 downto 1) + c_rl;
                    when 242 => c_vsq <= f_acc(17 downto 0);
                    when 266 => c_dfl <= f_acc(31 downto 18);
                    when 290 => c_drl <= f_acc(25 downto 8);
                    when 314 =>
                        if s_g5(12 downto 3) >= 512 then c_a1 <= to_unsigned(C_A1K, 14) - f_acc(19 downto 6);
                        else c_a1 <= to_unsigned(C_A1K, 14) + f_acc(19 downto 6); end if;
                    when 338 => c_eg <= f_acc(17 downto 10);
                    when 362 => c_d10 <= f_acc(21 downto 10);
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
            if s_src = '1' then s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17):
    --   g  = (((yq^2 >> 14) * 9986 >> 16) + 17) * depth >> 10      V parabola, Q2
    --   C1 = A1 -+ |dy| * 4089 >> 16 - ((dy^2 >> 12) * 590 >> 16)    F1's line term, Q2
    --   Fc = 0.742 g + 0.258 C1 -> white-line fade on the F1 side
    --   stripe bit, wobble (LFSR walk +-2 px), the line's start phase
    -- n_* are copied to l_* in the horizontal blanking, after the line has drained.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ye : unsigned(18 downto 0);
        variable v_dy : signed(19 downto 0);
        variable v_c  : signed(15 downto 0);
        variable v_ps : unsigned(23 downto 0);
        variable v_b  : std_logic;
        variable v_lf : std_logic_vector(15 downto 0);
        variable v_j  : signed(2 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_ps <= c_ps0 - c_fs;          -- line 0 adds it back
                e_lf <= x"ACE1";
                e_j  <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ysf, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if s_flip = '1' then
                        if e_vy > 276224 then v_ye := (others => '0'); else v_ye := to_unsigned(276224, 19) - e_vy; end if;
                    else
                        v_ye := e_vy;
                    end if;
                    e_yq <= resize(shift_right(v_ye + 573, 4), 15);
                    v_dy := signed(resize(v_ye, 20)) - 179200;
                    if v_dy < 0 then e_dneg <= '1'; v_dy := -v_dy; else e_dneg <= '0'; end if;
                    e_dyq <= unsigned(v_dy(17 downto 4));
                when 2 =>
                    m_a <= resize(e_yq, 32); m_b <= resize(e_yq, 16); m_acc <= (others => '0');
                when 19 =>
                    m_a <= resize(m_acc(29 downto 14), 32); m_b <= to_unsigned(9986, 16); m_acc <= (others => '0');
                when 36 =>
                    m_a <= resize(m_acc(27 downto 16), 32) + 17; m_b <= resize(c_d10, 16); m_acc <= (others => '0');
                when 53 =>
                    e_g <= m_acc(22 downto 10);
                    m_a <= resize(e_dyq, 32); m_b <= resize(e_dyq, 16); m_acc <= (others => '0');
                when 70 =>
                    m_a <= resize(m_acc(27 downto 12), 32); m_b <= to_unsigned(590, 16); m_acc <= (others => '0');
                when 87 =>
                    e_qt <= m_acc(29 downto 16);
                    m_a <= resize(e_dyq, 32); m_b <= to_unsigned(4089, 16); m_acc <= (others => '0');
                when 104 =>
                    v_c := signed(resize(c_a1, 16)) - signed(resize(e_qt, 16));
                    if e_dneg = '1' then v_c := v_c + signed(resize(m_acc(29 downto 16), 16));
                    else v_c := v_c - signed(resize(m_acc(29 downto 16), 16)); end if;
                    if v_c < 0 then e_c1 <= (others => '0');
                    elsif v_c > 16383 then e_c1 <= (others => '1');
                    else e_c1 <= unsigned(v_c(13 downto 0)); end if;
                when 105 =>
                    n_g  <= e_g;
                    n_c1 <= e_c1;
                    e_fc <= resize(e_g, 15) - resize(shift_right(e_g, 2), 15) - resize(shift_right(e_g, 7), 15)
                            + resize(shift_right(e_c1, 2), 15) + resize(shift_right(e_c1, 7), 15);
                when 106 =>
                    if e_fc <= C_FC0 then n_w1 <= to_unsigned(4, 3);
                    elsif e_fc <= C_FC1 then n_w1 <= to_unsigned(3, 3);
                    elsif e_fc <= C_FC2 then n_w1 <= to_unsigned(2, 3);
                    elsif e_fc <= C_FC3 then n_w1 <= to_unsigned(1, 3);
                    else n_w1 <= to_unsigned(0, 3); end if;
                    v_ps := e_ps + resize(c_fs, 24);
                    e_ps <= v_ps;
                    n_s  <= v_ps(23) and s_strp;
                    -- wobble: one LFSR step per line, a +-2 px walk
                    v_b := e_lf(0) xor e_lf(2) xor e_lf(3) xor e_lf(5);
                    v_lf := v_b & e_lf(15 downto 1);
                    e_lf <= v_lf;
                    v_j := e_j;
                    case v_lf(2 downto 0) is
                        when "000"  => if e_j < 2 then v_j := e_j + 1; end if;
                        when "001"  => if e_j > -2 then v_j := e_j - 1; end if;
                        when "010"  => if e_j > 0 then v_j := e_j - 1; elsif e_j < 0 then v_j := e_j + 1; end if;
                        when others => null;
                    end case;
                    e_j <= v_j;
                when 107 =>
                    case to_integer(e_j) is
                        when 2      => n_phs <= c_phi0 + shift_left(resize(c_step, 24), 1);
                        when 1      => n_phs <= c_phi0 + resize(c_step, 24);
                        when -1     => n_phs <= c_phi0 - resize(c_step, 24);
                        when -2     => n_phs <= c_phi0 - shift_left(resize(c_step, 24), 1);
                        when others => n_phs <= c_phi0;
                    end case;
                when others => null;
            end case;

            -- hand-over in the blanking, once the previous line has left every stage that reads l_*
            if g_avid = '0' and s_gav_sr(20) = '0' and s_gav_sr(0) = '0' then
                l_g <= n_g; l_c1 <= n_c1; l_w1 <= n_w1; l_s <= n_s; l_phs <= n_phs;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: tile phase DDA (held at the line-start phase through the blanking
    -- and the first NH pixels), pixel counter for the line buffer
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_avid = '0' then
                b_phi  <= l_phs;
                b_hold <= c_nh;
                b_px   <= (others => '0');
            else
                b_px <= b_px + 1;
                if b_hold /= 0 then
                    b_hold <= b_hold - 1;
                else
                    b_phi <= b_phi + resize(c_step, 24);
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = the beam register of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t  : unsigned(10 downto 0);
        variable v_d  : signed(12 downto 0);
        variable v_k  : std_logic_vector(7 downto 0);
        variable v_cnt : unsigned(2 downto 0);
        variable v_ix : unsigned(10 downto 0);
        variable v_y, v_u, v_v : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: triangle (0 at the shield axis, 2047 at the seam)
            if b_phi(23) = '1' then p1_tri <= not b_phi(22 downto 12); else p1_tri <= b_phi(22 downto 12); end if;
            -- 2-3: u = tri * 0.9199 (quarter px)
            p2_a <= p1_tri - shift_right(p1_tri, 4);
            p2_b <= shift_right(p1_tri, 6) + shift_right(p1_tri, 9);
            p3_u <= p2_a - p2_b;
            p3_lm <= unsigned(s_lb_q);
            -- 4: tip rounding table
            p4_u <= p3_u;
            p4_tip <= C_SH_TIP(to_integer(p3_u(7 downto 2)));
            if p3_u(10 downto 8) = "000" then p4_tv <= '1'; else p4_tv <= '0'; end if;
            p4_lm <= p3_lm;
            -- 5: u' = u + tip
            if p4_tv = '1' then p5_ur <= p4_u + resize(p4_tip, 11); else p5_ur <= p4_u; end if;
            p5_lm <= p4_lm;
            -- 6: F2 = u' + g ; 3 u' and u'/8 - u'/64
            p6_f2 <= resize(p5_ur, 14) + l_g;
            p6_s3 <= shift_left(resize(p5_ur, 13), 1) + resize(p5_ur, 13);
            p6_sb <= resize(shift_right(p5_ur, 3), 13) - resize(shift_right(p5_ur, 6), 13);
            p6_lm <= p5_lm;
            -- 7: s1 u' = 2.8906 u' ; etch partials
            p7_f2 <= p6_f2;
            p7_s1u <= p6_s3 - p6_sb;
            p7_ea <= resize(p6_lm(3 downto 0) * c_eg, 12);
            p7_eb <= resize(p6_lm(7 downto 4) * c_eg, 12);
            -- 8: F1 = C1 - s1 u' ; etch sum
            p8_f2 <= p7_f2;
            p8_f1 <= signed(resize(l_c1, 15)) - signed(resize(p7_s1u, 15));
            p8_le <= resize(p7_ea, 16) + shift_left(resize(p7_eb, 16), 4);
            -- 9: F = min(F2, F1)
            if p8_f1 < 0 then
                p9_f <= (others => '0'); p9_f1s <= '1';
            elsif p8_f1 < signed(resize(p8_f2, 15)) then
                p9_f <= unsigned(p8_f1(13 downto 0)); p9_f1s <= '1';
            else
                p9_f <= p8_f2; p9_f1s <= '0';
            end if;
            p9_et <= p8_le(15 downto 4);
            -- 10: + etch, wrapped at 1024 px
            p10_fw <= p9_f(11 downto 0) + p9_et;
            p10_f1s <= p9_f1s;
            -- 11: level tests, ramp offset, band product partials
            if p10_fw >= C_TZ then p11_z <= '1'; else p11_z <= '0'; end if;
            if p10_fw >= C_TM then p11_m <= '1'; else p11_m <= '0'; end if;
            if p10_fw >= C_TT then p11_t <= '1'; else p11_t <= '0'; end if;
            if p10_fw < C_LW then p11_lb <= '1'; else p11_lb <= '0'; end if;
            if p10_fw > C_LH then p11_lh <= '1'; else p11_lh <= '0'; end if;
            if p10_fw < C_TZ + 240 then p11_dz <= '1'; else p11_dz <= '0'; end if;
            p11_d  <= signed(resize(p10_fw, 13)) - C_TZ;
            p11_pa <= resize(p10_fw * c_gb(4 downto 0), 17);
            p11_pb <= resize(p10_fw * c_gb(9 downto 5), 17);
            p11_pc <= resize(p10_fw * c_gb(12 downto 10), 15);
            p11_f1s <= p10_f1s;
            -- 12: ramp index (2 px steps), band product
            if p11_d < 0 then p12_ri <= (others => '0');
            elsif p11_d(12 downto 3) > 127 then p12_ri <= (others => '1');
            else p12_ri <= unsigned(p11_d(9 downto 3)); end if;
            p12_pr <= resize(p11_pa, 25) + shift_left(resize(p11_pb, 25), 5) + shift_left(resize(p11_pc, 25), 10);
            p12_z <= p11_z; p12_m <= p11_m; p12_t <= p11_t; p12_lb <= p11_lb; p12_lh <= p11_lh; p12_dz <= p11_dz;
            p12_f1s <= p11_f1s;
            -- 13: band index n = ((F * gain) >> 11) + flow, bits 10..8
            v_ix := p12_pr(21 downto 11) + c_fl8(10 downto 0);
            p13_n <= v_ix(10 downto 8);
            p13_z <= p12_z; p13_m <= p12_m; p13_t <= p12_t; p13_lb <= p12_lb; p13_lh <= p12_lh; p13_dz <= p12_dz;
            p13_f1s <= p12_f1s; p13_ri <= p12_ri;
            -- 14: keyer path delayed D pixels (10 / 7 / 5); ramp path of this pixel
            p_kd(0) <= std_logic_vector(p13_n) & p13_f1s & p13_m & p13_t & p13_lb & p13_lh;
            for i in 1 to 9 loop p_kd(i) <= p_kd(i - 1); end loop;
            case c_dsel is
                when "00"   => v_k := p_kd(9);
                when "01"   => v_k := p_kd(6);
                when others => v_k := p_kd(4);
            end case;
            p14_k  <= v_k;
            p14_z  <= p13_z; p14_dz <= p13_dz; p14_ri <= p13_ri;
            p14_zp <= p14_z;
            -- 15: one-shots and the palette address
            if s_gav_sr(13) = '0' then
                p15_arm <= p14_k(0);
                p15_cnt <= (others => '0');
                p15_ln  <= '0';
            else
                v_cnt := p15_cnt;
                if p15_arm = '1' and p14_k(1) = '1' then v_cnt := c_lww; end if;
                if v_cnt /= 0 then p15_ln <= s_line; p15_cnt <= v_cnt - 1; else p15_ln <= '0'; p15_cnt <= v_cnt; end if;
                if p14_k(1) = '1' then p15_arm <= '0'; elsif p14_k(0) = '1' then p15_arm <= '1'; end if;
            end if;
            v_cnt := p15_dcnt;
            if p14_z = '1' and p14_zp = '0' and p14_dz = '1' and s_gav_sr(13) = '1' then v_cnt := c_ldw; end if;
            if v_cnt /= 0 then p15_dk <= s_line; p15_dcnt <= v_cnt - 1; else p15_dk <= '0'; p15_dcnt <= v_cnt; end if;
            if p14_k(4) = '1' then p15_w <= l_w1; else p15_w <= to_unsigned(4, 3); end if;
            if p14_k(2) = '1' then
                r_pa <= s_pal & to_unsigned(145, 8);
            elsif p14_k(3) = '1' then
                r_pa <= s_pal & to_unsigned(144, 8);
            elsif p14_z = '1' then
                r_pa <= s_pal & '0' & p14_ri;
            else
                r_pa <= s_pal & "1000" & l_s & unsigned(p14_k(7 downto 5));
            end if;
            -- 16: ROM data (EBR); 17: fabric copy
            p16_ln <= p15_ln; p16_dk <= p15_dk; p16_w <= p15_w;
            p17_q <= r_pq; p17_ln <= p16_ln; p17_dk <= p16_dk; p17_w <= p16_w;
            -- 18: line arithmetic
            p18_y <= unsigned(p17_q(31 downto 22)); p18_u <= unsigned(p17_q(21 downto 12)); p18_v <= unsigned(p17_q(11 downto 2));
            p18_dy <= signed(resize(unsigned(p17_q(31 downto 22)), 11)) - 64;
            p18_du <= signed(resize(unsigned(p17_q(21 downto 12)), 11)) - 512;
            p18_dv <= signed(resize(unsigned(p17_q(11 downto 2)), 11)) - 512;
            p18_ey <= signed(resize(c_wy, 11)) - signed(resize(unsigned(p17_q(31 downto 22)), 11));
            p18_eu <= signed(resize(c_wu, 11)) - signed(resize(unsigned(p17_q(21 downto 12)), 11));
            p18_ev <= signed(resize(c_wv, 11)) - signed(resize(unsigned(p17_q(11 downto 2)), 11));
            p18_ln <= p17_ln and p17_q(0); p18_dk <= p17_dk and p17_q(1); p18_w <= p17_w;
            -- 19: dark (x 0.6875 toward black) and white (w / 4 toward the line colour) terms
            p19_y <= p18_y; p19_u <= p18_u; p19_v <= p18_v;
            v_y := resize(signed(resize(p18_y, 11)) - shift_right(p18_dy, 2) - shift_right(p18_dy, 4), 13);
            v_u := resize(signed(resize(p18_u, 11)) - shift_right(p18_du, 2) - shift_right(p18_du, 4), 13);
            v_v := resize(signed(resize(p18_v, 11)) - shift_right(p18_dv, 2) - shift_right(p18_dv, 4), 13);
            p19_ky <= unsigned(v_y(9 downto 0)); p19_ku <= unsigned(v_u(9 downto 0)); p19_kv <= unsigned(v_v(9 downto 0));
            case to_integer(p18_w) is
                when 4      => p19_wy <= shift_left(resize(p18_ey, 13), 2); p19_wu <= shift_left(resize(p18_eu, 13), 2); p19_wv <= shift_left(resize(p18_ev, 13), 2);
                when 3      => p19_wy <= shift_left(resize(p18_ey, 13), 1) + resize(p18_ey, 13);
                               p19_wu <= shift_left(resize(p18_eu, 13), 1) + resize(p18_eu, 13);
                               p19_wv <= shift_left(resize(p18_ev, 13), 1) + resize(p18_ev, 13);
                when 2      => p19_wy <= shift_left(resize(p18_ey, 13), 1); p19_wu <= shift_left(resize(p18_eu, 13), 1); p19_wv <= shift_left(resize(p18_ev, 13), 1);
                when 1      => p19_wy <= resize(p18_ey, 13); p19_wu <= resize(p18_eu, 13); p19_wv <= resize(p18_ev, 13);
                when others => p19_wy <= (others => '0'); p19_wu <= (others => '0'); p19_wv <= (others => '0');
            end case;
            p19_ln <= p18_ln; p19_dk <= p18_dk;
            -- 20: compose; black outside the generated avid
            if s_gav_sr(18) = '0' then
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            elsif p19_ln = '1' then
                v_y := signed(resize(p19_y, 13)) + shift_right(p19_wy, 2);
                v_u := signed(resize(p19_u, 13)) + shift_right(p19_wu, 2);
                v_v := signed(resize(p19_v, 13)) + shift_right(p19_wv, 2);
                s_y20 <= unsigned(v_y(9 downto 0)); s_u20 <= unsigned(v_u(9 downto 0)); s_v20 <= unsigned(v_v(9 downto 0));
            elsif p19_dk = '1' then
                s_y20 <= p19_ky; s_u20 <= p19_ku; s_v20 <= p19_kv;
            else
                s_y20 <= p19_y; s_u20 <= p19_u; s_v20 <= p19_v;
            end if;
        end if;
    end process p_pix;
    -- white line colour (palette entry 146 / 402), constant per palette
    c_wy <= to_unsigned(940, 10);
    c_wu <= to_unsigned(528, 10) when s_pal = '0' else to_unsigned(496, 10);
    c_wv <= to_unsigned(515, 10) when s_pal = '0' else to_unsigned(509, 10);

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through a [1 2 .. 2 1]/32 17-tap box and a
    -- [1 2 .. 2 1]/16 9-tap box (the photo's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16, padding to C_LAT, and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sa : unsigned(14 downto 0);
        variable v_sb : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A (16-sum): stage 21 sums, stage 22 output.  s_qa = the stage-20 value 16 pixels back
            -- (read address = write address - 13, then the EBR and a fabric register)
            s_wa_a <= s_wa_a + 1;
            s_ra_a <= s_wa_a - 13;
            s_qa <= s_rq_a;
            -- the running sums re-anchor in every horizontal blanking (both rings hold only black by then),
            -- so a bad block-RAM read can never leave a lasting offset
            if s_gav_sr(19) = '1' then s_bk <= (others => '0'); elsif s_bk /= 63 then s_bk <= s_bk + 1; end if;
            if s_bk = 63 and s_gav_sr(19) = '0' and s_qa(1 downto 0) = "00" then
                s_bay <= to_unsigned(1024, 14); s_bau <= to_unsigned(8192, 14); s_bav <= to_unsigned(8192, 14);
            else
                s_bay <= s_bay + s_y20 - unsigned(s_qa(31 downto 22));
                s_bau <= s_bau + s_u20 - unsigned(s_qa(21 downto 12));
                s_bav <= s_bav + s_v20 - unsigned(s_qa(11 downto 2));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            v_sa := resize(s_bay, 15) + s_bayd; s_y22 <= v_sa(14 downto 5);
            v_sa := resize(s_bau, 15) + s_baud; s_u22 <= v_sa(14 downto 5);
            v_sa := resize(s_bav, 15) + s_bavd; s_v22 <= v_sa(14 downto 5);
            -- box B (8-sum): stage 23 sums, stage 24 output (read address = write address - 5)
            s_wa_b <= s_wa_b + 1;
            s_ra_b <= s_wa_b - 5;
            s_qb <= s_rq_b;
            if s_bk = 63 and s_gav_sr(21) = '0' and s_qb(1 downto 0) = "00" then
                s_bby <= to_unsigned(512, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y22 - unsigned(s_qb(31 downto 22));
                s_bbu <= s_bbu + s_u22 - unsigned(s_qb(21 downto 12));
                s_bbv <= s_bbv + s_v22 - unsigned(s_qb(11 downto 2));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sb := resize(s_bby, 14) + s_bbyd; s_y24 <= v_sb(13 downto 4);
            v_sb := resize(s_bbu, 14) + s_bbud; s_u24 <= v_sb(13 downto 4);
            v_sb := resize(s_bbv, 14) + s_bbvd; s_v24 <= v_sb(13 downto 4);
            -- 25: luma 13/16
            if s_y24 > 64 then v_dl := s_y24 - 64; else v_dl := (others => '0'); end if;
            s_y25 <= s_y24 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u25 <= s_u24; s_v25 <= s_v24;
            -- 26..31: padding
            s_py(26) <= s_y25; s_pu(26) <= s_u25; s_pv(26) <= s_v25;
            for i in 27 to 31 loop s_py(i) <= s_py(i - 1); s_pu(i) <= s_pu(i - 1); s_pv(i) <= s_pv(i - 1); end loop;
            -- 32: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_yo <= s_py(31); s_uo <= s_pu(31); s_vo <= s_pv(31);
            else
                s_yo <= to_unsigned(64, 10); s_uo <= to_unsigned(512, 10); s_vo <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_yo);
    data_out.u       <= std_logic_vector(s_uo);
    data_out.v       <= std_logic_vector(s_vo);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture shield;
