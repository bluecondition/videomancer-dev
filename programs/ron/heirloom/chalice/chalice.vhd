-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Chalice v0.1.0 -- Heirloom from the video clips, built from the still WIN_20230305_00_05_43 t0.5
-- (the clip is static: only the per-line jitter texture changes frame to frame).
--
-- The still: a mirrored chalice drawn by rainbow contour rings.  A bowl of nested U rings with two
-- dark-red lobes and a pink ridge at the top and a green/cyan rim around it, a pinched neck, a
-- trapezoid foot with chevron rings and an orange eye; olive plateau at the top, horizontal bands
-- (yellow, orange, pink, purple, blue) bending up into the bowl, black below the blue line.
--
-- The patch, as fitted (chalice.py has every constant; .heirloom_work/rb_chalice/notes.md the measurements):
--   one field phi (ring units: 1 = one blue-channel period, higher = darker)
--     phi_raw = ramp(y) + S(y) (1 - |u|/U0)^2 - K(y) h(|u|/w(y)) + Kd(y) dimple(|u|/w(y)) + camera luma
--     phi     = -3 + |phi_raw + 3|                       a wavefolder at -3: the rim ring
--   three colourisers on phi:  R = clamp(255 - 58 |phi - 1.525|)   a slow triangle (the orange band is its peak)
--                              G = clamp(128 - 42 (phi - 1.2))     a clamp, delayed 40 virtual px (channel delay)
--                              B = clamp(2 tri(phi_B) - 1/2)       a fast triangle: the contour rings
--     (B sees the ramp clipped at -0.5, so the plateau is ring-free, and its total clipped at 5.5)
--   ramp(y): the vertical ramp (olive plateau to y = 220, three slopes, black at 780); the chalice is the
--   width-scaled profile h (a crater above the neck, a dome below) with the per-line amplitude K, width w,
--   skirt S (the squared H ramp, bending the bands) and lobe dimple Kd, all piecewise-linear in y.
--
--   K1  Ramp      the vertical ramp's slope: band density and where the black starts (x0.4 .. x2.4)
--   K2  Bowl      the bowl's width (x0.5 .. x2)
--   K3  Foot      the foot's flare (x0.4 .. x2.4)
--   K4  Rings     the blue colouriser's rate: contour ring density (x0.25 .. x4)
--   K5  Breathe   a slow triangle LFO sliding the ramp under its plateau: the bands breathe (0 .. 1.5 rings)
--   K6  Camera    how far the picture (or the Osc) bends the field (0 .. 1.9 rings per full luma)
--   S7  Lobes     the two lobes and the ridge at the top of the bowl: On / Off
--   S8  Rim       the wavefolder: On (the rim ring) / Off (a deep well of rings)
--   S9  Flip      the field upside down: the bowl becomes a dome on a black sky
--   S10 Palette   Chalice (the photo) / Ice (red and blue colourisers swapped: a teal chalice with red rings)
--   S11 Source    Video / Osc
--   P12 CHALICE   the chalice amplitude: 0 = plain rainbow bands, 410 = the photo, 100 % = a towering chalice
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide: the knob curves and 32 scaled table
-- constants written into a block-RAM table), a per-line engine kicked at each line start computing the
-- NEXT line (the table read just in time, 5 products, a serial divide for 1/w, the v start, the skirt DDA
-- constants), a beam with two mirrored DDAs (v = |u|/w at stage 1, the skirt parabola at stage 6), a
-- 23-stage pixel path (profile ROM, two multipliers, fold, three shapers, G delay ring, RGB -> YUV
-- matrix).  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning
-- (Y, U, V through two [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps sit in block-RAM rings, luma 13/16)
-- before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.chalice_pkg.all;
use work.all;

architecture chalice of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks

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
    -- segment thresholds (virtual lines, Q8)
    function f_thr(t : unsigned(2 downto 0); seg : unsigned(1 downto 0)) return unsigned is
        variable v : integer := 0;
    begin
        case to_integer(t) is
            when 0 => case seg is when "01" => v := 220; when "10" => v := 455; when "11" => v := 685; when others => v := 0; end case;
            when 1 => case seg is when "01" => v := 480; when "10" => v := 650; when "11" => v := 790; when others => v := 0; end case;
            when 2 => case seg is when "01" => v := 480; when "10" => v := 700; when "11" => v := 1000; when others => v := 0; end case;
            when 3 => case seg is when "01" => v := 100; when "10" => v := 300; when "11" => v := 480; when others => v := 0; end case;
            when others => case seg is when "01" => v := 300; when "10" => v := 600; when others => v := 0; end case;
        end case;
        return to_unsigned(v * 256, 19);
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
    signal r_pf_a : unsigned(9 downto 0) := (others => '0');
    signal r_pf_q : std_logic_vector(19 downto 0) := (others => '0');

    attribute no_rw_check : boolean;

    -- per-field table RAM (starts and slopes of the five piecewise-linear tables, knob-scaled): written
    -- by the zone sequencer, read by the line engine during active lines
    type t_tr is array (0 to 63) of std_logic_vector(15 downto 0);
    signal tr : t_tr := (others => x"0000");
    attribute no_rw_check of tr : signal is true;
    signal tr_we : std_logic := '0';
    signal tr_wa, tr_ra : unsigned(5 downto 0) := (others => '0');
    signal tr_wd, tr_q : std_logic_vector(15 downto 0) := x"0000";

    -- G delay ring (the channel delay, 40 virtual px)
    type t_gd is array (0 to 255) of std_logic_vector(7 downto 0);
    signal gd : t_gd := (others => x"00");
    attribute no_rw_check of gd : signal is true;
    signal gd_wa, gd_ra : unsigned(7 downto 0) := (others => '0');
    signal gd_wd, gd_q : std_logic_vector(7 downto 0) := x"00";

    -- output box rings (the 8-back taps of the two running sums) and the sync delay ring
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ba1, ba2, bb1, bb2 : t_r16 := (others => x"0000");
    attribute no_rw_check of ba1 : signal is true;
    attribute no_rw_check of ba2 : signal is true;
    attribute no_rw_check of bb1 : signal is true;
    attribute no_rw_check of bb2 : signal is true;
    signal rg_wa, rg_ra, sy_ra : unsigned(7 downto 0) := (others => '0');
    signal ba1_q, ba2_q, bb1_q, bb2_q : std_logic_vector(15 downto 0) := x"0000";
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal syr : t_r4 := (others => "1111");
    attribute no_rw_check of syr : signal is true;
    signal syr_q, syr_c : std_logic_vector(3 downto 0) := "1111";

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

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(256 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(410 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_lob, s_rim, s_flip, s_pal, s_src : std_logic := '0';

    -- knob curves (Q8 scales), chalice scale, breathe amplitude, LFO
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_sr, c_sb, c_sf : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_rate : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal c_sp   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_amp  : unsigned(8 downto 0) := to_unsigned(75, 9);
    signal c_ph   : unsigned(15 downto 0) := to_unsigned(16384, 16);   -- breathe starts at its zero crossing (the still)
    signal c_tri  : unsigned(14 downto 0) := (others => '0');
    signal c_tsub : signed(15 downto 0) := (others => '0');
    signal c_tmag : unsigned(14 downto 0) := (others => '0');
    signal c_tneg : std_logic := '0';
    signal c_lfo  : signed(10 downto 0) := (others => '0');
    signal c_cg   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_dly  : unsigned(5 downto 0) := to_unsigned(40, 6);
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal c_u0h  : unsigned(10 downto 0) := to_unsigned(1904, 11);
    signal c_cf0  : unsigned(20 downto 0) := to_unsigned(38655, 21);
    signal c_cd0  : unsigned(14 downto 0) := to_unsigned(1627, 15);
    signal c_ci2  : unsigned(7 downto 0) := to_unsigned(34, 8);
    signal c_wv2, c_ws2 : unsigned(15 downto 0) := (others => '0');

    -- field serial multiplier
    signal f_a   : unsigned(23 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(23 downto 0) := (others => '0');
    signal z_fa  : unsigned(23 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal z_k   : unsigned(4 downto 0) := (others => '0');   -- product slot counter
    signal z_kc  : unsigned(4 downto 0) := (others => '0');   -- clock within the slot

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';
    signal g_ssr : std_logic_vector(5 downto 0) := (others => '0');   -- g_start delayed 1 .. 6

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_yf   : unsigned(18 downto 0) := (others => '0');
    signal e_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal e_rseg, e_kseg, e_wseg, e_sseg, e_dseg : unsigned(1 downto 0) := (others => '0');
    signal e_trd  : unsigned(15 downto 0) := (others => '0');
    signal e_thr, e_dy : unsigned(18 downto 0) := (others => '0');
    signal e_ev   : signed(17 downto 0) := (others => '0');
    signal e_sub  : std_logic := '0';
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal e_ramp : signed(13 downto 0) := (others => '0');
    signal e_s    : unsigned(11 downto 0) := (others => '0');
    signal e_w16  : unsigned(13 downto 0) := to_unsigned(5520, 14);
    signal d_run  : std_logic := '0';
    signal d_i    : unsigned(4 downto 0) := (others => '0');
    signal d_rem  : unsigned(13 downto 0) := (others => '0');
    signal d_q    : unsigned(19 downto 0) := (others => '0');
    signal n_base : unsigned(11 downto 0) := to_unsigned(609, 12);
    signal n_db   : unsigned(5 downto 0) := (others => '0');
    signal n_k    : unsigned(11 downto 0) := (others => '0');
    signal n_kd   : unsigned(10 downto 0) := (others => '0');
    signal n_g    : unsigned(19 downto 0) := (others => '0');
    signal n_a0   : unsigned(31 downto 0) := (others => '0');
    signal n_f0   : unsigned(27 downto 0) := (others => '0');
    signal n_d0   : unsigned(18 downto 0) := (others => '0');
    signal n_i2   : unsigned(11 downto 0) := (others => '0');

    -- beam (stage 1) and the stage-6 skirt DDA
    signal b_x, b_x1, b_x2 : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_px3 : unsigned(10 downto 0) := (others => '0');
    signal b_a   : unsigned(31 downto 0) := (others => '0');
    signal b_al, b_ax : std_logic := '0';
    signal b_axs : std_logic_vector(4 downto 0) := (others => '0');   -- b_ax delayed 1 .. 5
    signal b_f   : signed(28 downto 0) := (others => '0');
    signal b_d   : signed(21 downto 0) := (others => '0');
    signal l_base : unsigned(11 downto 0) := to_unsigned(609, 12);
    signal l_db   : unsigned(5 downto 0) := (others => '0');
    signal l_k    : unsigned(11 downto 0) := (others => '0');
    signal l_kd   : unsigned(10 downto 0) := (others => '0');
    signal l_gx   : unsigned(21 downto 0) := (others => '0');
    signal l_i2   : unsigned(11 downto 0) := (others => '0');

    -- pixel path
    signal p4_h   : unsigned(11 downto 0) := (others => '0');
    signal p4_d   : unsigned(7 downto 0) := (others => '0');
    signal p5_a, p5_b : unsigned(17 downto 0) := (others => '0');
    signal p5_dd  : unsigned(18 downto 0) := (others => '0');
    signal p6_kh  : unsigned(13 downto 0) := (others => '0');
    signal p6_dd  : unsigned(11 downto 0) := (others => '0');
    signal p6_lm  : unsigned(7 downto 0) := (others => '0');
    signal p7_ca  : unsigned(12 downto 0) := (others => '0');
    signal p8_cam : unsigned(8 downto 0) := (others => '0');
    signal p7_t1  : unsigned(12 downto 0) := (others => '0');
    signal p7_t2  : signed(14 downto 0) := (others => '0');
    signal p8_a, p9_a, p10_x : signed(15 downto 0) := (others => '0');
    signal p10_neg : std_logic := '0';
    signal p11_dr, p11_dg, p11_xb : signed(15 downto 0) := (others => '0');
    signal p12_dr : unsigned(13 downto 0) := (others => '0');
    signal p12_dg : signed(15 downto 0) := (others => '0');
    signal p12_pb : signed(15 downto 0) := (others => '0');
    signal p13_t1 : unsigned(17 downto 0) := (others => '0');
    signal p13_g1 : signed(18 downto 0) := (others => '0');
    signal p13_pa : unsigned(11 downto 0) := (others => '0');
    signal p13_pq : unsigned(11 downto 0) := (others => '0');
    signal p14_r, p14_g : unsigned(7 downto 0) := (others => '0');
    signal p14_pp : unsigned(11 downto 0) := (others => '0');
    signal p15_r, p15_g, p16_r, p16_g : unsigned(7 downto 0) := (others => '0');
    signal p15_tb, p16_tri : unsigned(7 downto 0) := (others => '0');
    signal p17_r, p17_g, p17_b : unsigned(7 downto 0) := (others => '0');
    signal p18_r, p18_b, p18_gd, p19_r, p19_b, p19_gd : unsigned(7 downto 0) := (others => '0');
    signal p18_early, p19_early : std_logic := '0';
    signal p18_cnt : unsigned(6 downto 0) := (others => '0');
    signal p20_r, p20_g, p20_b : unsigned(7 downto 0) := (others => '0');
    signal p21_ry, p21_gy, p21_by : unsigned(11 downto 0) := (others => '0');
    signal p21_rr, p21_gr, p21_br, p21_rb, p21_gb, p21_bb : unsigned(13 downto 0) := (others => '0');
    signal p22_y : unsigned(13 downto 0) := (others => '0');
    signal p22_u, p22_v : signed(15 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y25, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28, s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories: line buffer (collisions possible: read-before-write kept),
    -- ROMs, the table RAM, the G delay ring, the box rings, the sync ring
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
            r_ex_q <= C_CH_EXP2(to_integer(r_ex_a));
            r_pf_q <= C_CH_PROF(to_integer(r_pf_a));
        end if;
    end process p_rom;

    p_tr : process(clk)
    begin
        if rising_edge(clk) then
            tr_q <= tr(to_integer(tr_ra));
            if tr_we = '1' then tr(to_integer(tr_wa)) <= tr_wd; end if;
        end if;
    end process p_tr;

    p_gd : process(clk)
    begin
        if rising_edge(clk) then
            gd_q <= gd(to_integer(gd_ra));
            gd(to_integer(gd_wa)) <= gd_wd;
        end if;
    end process p_gd;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y23) & std_logic_vector(s_u23(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u23(3 downto 0)) & std_logic_vector(s_v23) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y25) & std_logic_vector(s_u25(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u25(3 downto 0)) & std_logic_vector(s_v25) & "00";
            syr_q <= syr(to_integer(sy_ra));
            syr(to_integer(rg_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_e  : signed(19 downto 0);
        variable v_p  : unsigned(15 downto 0);
        variable v_m  : unsigned(15 downto 0);
        variable v_s  : signed(14 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_lob  <= not registers_in(6)(0);     -- "On" = label 0
                s_rim  <= not registers_in(6)(1);
                s_flip <= registers_in(6)(2);
                s_pal  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                                   c_dly <= to_unsigned(20, 6); c_u0h <= to_unsigned(1408, 11);
                                   c_cf0 <= to_unsigned(1469928, 21); c_cd0 <= to_unsigned(19931, 15); c_ci2 <= to_unsigned(134, 8);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                                   c_dly <= to_unsigned(27, 6); c_u0h <= to_unsigned(1896, 11);
                                   c_cf0 <= to_unsigned(45357, 21); c_cd0 <= to_unsigned(2655, 15); c_ci2 <= to_unsigned(75, 8);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                                   c_dly <= to_unsigned(40, 6); c_u0h <= to_unsigned(1904, 11);
                                   c_cf0 <= to_unsigned(38655, 21); c_cd0 <= to_unsigned(1627, 15); c_ci2 <= to_unsigned(34, 8);
                end case;
                -- breathe LFO: period 1192 fields
                c_ph <= c_ph + 55;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, seven channels,
            -- run from the zone start by its own phase / channel counters (no zc decode on these enables)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 5      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when 3      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 5      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 6 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- knob curves on the exp2 ROM (Q8 scales): zc 30 .. 46
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 =>                                        -- ramp: 2^(0.64 (k1 - 512) / 256)
                        v_d := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        v_e := shift_left(resize(v_d, 20), 7) + shift_left(resize(v_d, 20), 5) + shift_left(resize(v_d, 20), 2);
                        r_ex_a <= unsigned(v_e(15 downto 8)); z_sh <= v_e(18 downto 16);   -- (d * 164) >> 8, Q8 octaves
                    when 32 => z_mt <= r_ex_q;
                    when 33 =>
                        case to_integer(z_sh) is
                            when -2     => c_sr <= "000" & z_mt(15 downto 9);
                            when -1     => c_sr <= "00" & z_mt(15 downto 8);
                            when 0      => c_sr <= "0" & z_mt(15 downto 7);
                            when others => c_sr <= z_mt(15 downto 6);
                        end case;
                    when 34 =>                                        -- bowl: 2^((k2 - 512) / 512)
                        v_d := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        v_e := shift_right(resize(v_d, 20), 1);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 36 => z_mt <= r_ex_q;
                    when 37 =>
                        case to_integer(z_sh) is
                            when -2     => c_sb <= "000" & z_mt(15 downto 9);
                            when -1     => c_sb <= "00" & z_mt(15 downto 8);
                            when 0      => c_sb <= "0" & z_mt(15 downto 7);
                            when others => c_sb <= z_mt(15 downto 6);
                        end case;
                    when 38 =>                                        -- foot: 2^(0.64 (k3 - 512) / 256)
                        v_d := signed(resize(s_g3(12 downto 3), 12)) - 512;
                        v_e := shift_left(resize(v_d, 20), 7) + shift_left(resize(v_d, 20), 5) + shift_left(resize(v_d, 20), 2);
                        r_ex_a <= unsigned(v_e(15 downto 8)); z_sh <= v_e(18 downto 16);
                    when 40 => z_mt <= r_ex_q;
                    when 41 =>
                        case to_integer(z_sh) is
                            when -2     => c_sf <= "000" & z_mt(15 downto 9);
                            when -1     => c_sf <= "00" & z_mt(15 downto 8);
                            when 0      => c_sf <= "0" & z_mt(15 downto 7);
                            when others => c_sf <= z_mt(15 downto 6);
                        end case;
                    when 42 =>                                        -- rings: 2^((k4 - 512) / 256), Q4
                        v_d := signed(resize(s_g4(12 downto 3), 12)) - 512;
                        r_ex_a <= unsigned(v_d(7 downto 0)); z_sh <= v_d(10 downto 8);
                        v_p := resize(s_gp, 16) + shift_left(resize(s_gp, 16), 2);   -- 40 p: chalice scale (p * 160) >> 8
                        c_sp <= v_p(15 downto 6);
                        v_p := resize(s_g5, 16) + shift_left(resize(s_g5, 16), 1);   -- 24 k5: breathe amplitude (k5 * 384) >> 10
                        c_amp <= v_p(14 downto 6);
                        c_cg <= s_g6(12 downto 3);
                        if c_ph(15) = '1' then c_tri <= not c_ph(14 downto 0); else c_tri <= c_ph(14 downto 0); end if;
                    when 44 => z_mt <= r_ex_q;
                    when 45 =>
                        case to_integer(z_sh) is
                            when -2     => c_rate <= "0000" & z_mt(15 downto 13);
                            when -1     => c_rate <= "000" & z_mt(15 downto 12);
                            when 0      => c_rate <= "00" & z_mt(15 downto 11);
                            when others => c_rate <= "0" & z_mt(15 downto 10);
                        end case;
                        c_tsub <= signed(resize(c_tri, 16)) - 16384;
                    when 46 =>
                        if c_tsub < 0 then c_tmag <= unsigned(resize(-c_tsub, 15)); c_tneg <= '1';
                        else c_tmag <= unsigned(resize(c_tsub, 15)); c_tneg <= '0'; end if;
                    when others => null;
                end case;
            end if;

            -- serial products: slot k (0 .. 31) loads at zc = 48 + 24 k, 16 iterations, result stable from
            -- load + 17.  The slot number is decoded two clocks ahead and the operands pre-muxed into
            -- z_fa / z_fb, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zgo = '1' then
                z_k <= (others => '0'); z_kc <= (others => '0');
            elsif s_zone = '1' and s_zc >= 46 and s_zc < 46 + 32 * 24 then
                if z_kc = 23 then z_kc <= (others => '0'); z_k <= z_k + 1; else z_kc <= z_kc + 1; end if;
                if z_kc = 0 then z_sel <= z_k; z_pre <= '1'; end if;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(549, 24);  z_fb <= resize(c_sr, 16);     -- ramp slope 1
                    when 1  => z_fa <= to_unsigned(746, 24);  z_fb <= resize(c_sr, 16);     -- ramp slope 2
                    when 2  => z_fa <= to_unsigned(1400, 24); z_fb <= resize(c_sr, 16);     -- ramp slope 3
                    when 3  => z_fa <= to_unsigned(504, 24);  z_fb <= resize(c_sr, 16);     -- ramp knee 1 (455)
                    when 4  => z_fa <= to_unsigned(1175, 24); z_fb <= resize(c_sr, 16);     -- ramp knee 2 (685)
                    when 5  => z_fa <= to_unsigned(666, 24);  z_fb <= resize(c_sp, 16);     -- K starts
                    when 6  => z_fa <= to_unsigned(461, 24);  z_fb <= resize(c_sp, 16);
                    when 7  => z_fa <= to_unsigned(198, 24);  z_fb <= resize(c_sp, 16);
                    when 8  => z_fa <= to_unsigned(422, 24);  z_fb <= resize(c_sp, 16);
                    when 9  => z_fa <= to_unsigned(109, 24);  z_fb <= resize(c_sp, 16);     -- K slopes
                    when 10 => z_fa <= to_unsigned(395, 24);  z_fb <= resize(c_sp, 16);
                    when 11 => z_fa <= to_unsigned(410, 24);  z_fb <= resize(c_sp, 16);
                    when 12 => z_fa <= to_unsigned(153, 24);  z_fb <= resize(c_sp, 16);
                    when 13 => z_fa <= to_unsigned(77, 24);   z_fb <= resize(c_sp, 16);     -- S starts (0, 1)
                    when 14 => z_fa <= to_unsigned(538, 24);  z_fb <= resize(c_sp, 16);
                    when 15 => z_fa <= to_unsigned(589, 24);  z_fb <= resize(c_sp, 16);
                    when 16 => z_fa <= to_unsigned(590, 24);  z_fb <= resize(c_sp, 16);     -- S slopes (1 .. 3)
                    when 17 => z_fa <= to_unsigned(73, 24);   z_fb <= resize(c_sp, 16);
                    when 18 => z_fa <= to_unsigned(33, 24);   z_fb <= resize(c_sp, 16);
                    when 19 => z_fa <= to_unsigned(256, 24);  z_fb <= resize(c_sp, 16);     -- Kd starts (0, 1), slopes (0, 1)
                    when 20 => z_fa <= to_unsigned(54, 24);   z_fb <= resize(c_sp, 16);
                    when 21 => z_fa <= to_unsigned(173, 24);  z_fb <= resize(c_sp, 16);
                    when 22 => z_fa <= to_unsigned(46, 24);   z_fb <= resize(c_sp, 16);
                    when 23 => z_fa <= to_unsigned(5520, 24); z_fb <= resize(c_sb, 16);     -- w: bowl starts / slopes
                    when 24 => z_fa <= to_unsigned(1152, 24); z_fb <= resize(c_sb, 16);
                    when 25 => z_fa <= to_unsigned(3360, 24); z_fb <= resize(c_sb, 16);
                    when 26 => z_fa <= to_unsigned(2569, 24); z_fb <= resize(c_sb, 16);
                    when 27 => z_fa <= to_unsigned(1152, 24); z_fb <= resize(c_sb, 16);     -- the neck width
                    when 28 => z_fa <= to_unsigned(3386, 24); z_fb <= resize(c_sf, 16);     -- foot slopes
                    when 29 => z_fa <= to_unsigned(1280, 24); z_fb <= resize(c_sf, 16);
                    when 30 => z_fa <= resize(c_ws2, 24);     z_fb <= to_unsigned(300, 16); -- w at 1000: neck + 300 lines of slope
                    when others => z_fa <= resize(c_amp, 24); z_fb <= resize(c_tmag, 16);   -- breathe offset
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(22 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- stores into the table RAM at z_kc = 19 (slot k's product is in f_acc) and a second entry at
            -- z_kc = 20.  Table map: 0..2 ramp slopes, 37..40 ramp bases (-159, -159, knee 1, knee 2), 5..8 K starts, 9..12 K slopes,
            -- 13..16 S starts, 17..20 S slopes, 21..24 Kd starts, 25..28 Kd slopes, 29..32 w starts, 33..36 w slopes.
            tr_we <= '0';
            v_m := f_acc(23 downto 8);
            if s_zone = '1' and z_kc = 19 then
                tr_we <= '1'; tr_wd <= std_logic_vector(v_m);
                case to_integer(z_k) is
                    when 0  => tr_wa <= to_unsigned(0, 6);
                    when 1  => tr_wa <= to_unsigned(1, 6);
                    when 2  => tr_wa <= to_unsigned(2, 6);
                    when 3  => tr_wa <= to_unsigned(39, 6); tr_wd <= std_logic_vector(v_m - 159);
                    when 4  => tr_wa <= to_unsigned(40, 6); tr_wd <= std_logic_vector(v_m - 159);
                    when 5  => tr_wa <= to_unsigned(5, 6);
                    when 6  => tr_wa <= to_unsigned(6, 6);
                    when 7  => tr_wa <= to_unsigned(7, 6);
                    when 8  => tr_wa <= to_unsigned(8, 6);
                    when 9  => tr_wa <= to_unsigned(9, 6);
                    when 10 => tr_wa <= to_unsigned(10, 6);
                    when 11 => tr_wa <= to_unsigned(11, 6);
                    when 12 => tr_wa <= to_unsigned(12, 6);
                    when 13 => tr_wa <= to_unsigned(13, 6);
                    when 14 => tr_wa <= to_unsigned(15, 6);
                    when 15 => tr_wa <= to_unsigned(16, 6);
                    when 16 => tr_wa <= to_unsigned(18, 6);
                    when 17 => tr_wa <= to_unsigned(19, 6);
                    when 18 => tr_wa <= to_unsigned(20, 6);
                    when 19 => tr_wa <= to_unsigned(21, 6); if s_lob = '0' then tr_wd <= (others => '0'); end if;
                    when 20 => tr_wa <= to_unsigned(22, 6); if s_lob = '0' then tr_wd <= (others => '0'); end if;
                    when 21 => tr_wa <= to_unsigned(25, 6); if s_lob = '0' then tr_wd <= (others => '0'); end if;
                    when 22 => tr_wa <= to_unsigned(26, 6); if s_lob = '0' then tr_wd <= (others => '0'); end if;
                    when 23 => tr_wa <= to_unsigned(29, 6);
                    when 24 => tr_wa <= to_unsigned(33, 6);
                    when 25 => tr_wa <= to_unsigned(30, 6);
                    when 26 => tr_wa <= to_unsigned(34, 6);
                    when 27 => tr_wa <= to_unsigned(31, 6); c_wv2 <= v_m;
                    when 28 => tr_wa <= to_unsigned(35, 6); c_ws2 <= v_m;
                    when 29 => tr_wa <= to_unsigned(36, 6);
                    when 30 => tr_wa <= to_unsigned(32, 6); tr_wd <= std_logic_vector(c_wv2 + v_m);
                    when others =>                                     -- the breathe offset: amp * |tri| >> 14, signed
                        tr_we <= '0';
                        v_s := signed(resize(f_acc(23 downto 14), 15));
                        if c_tneg = '1' then c_lfo <= resize(-v_s, 11); else c_lfo <= resize(v_s, 11); end if;
                end case;
            elsif s_zone = '1' and z_kc = 20 then
                case to_integer(z_k) is
                    when 3  => tr_we <= '1'; tr_wa <= to_unsigned(37, 6); tr_wd <= std_logic_vector(to_signed(-159, 16));  -- ramp base 0
                    when 4  => tr_we <= '1'; tr_wa <= to_unsigned(38, 6); tr_wd <= std_logic_vector(to_signed(-159, 16));  -- ramp base 1
                    when 13 => tr_we <= '1'; tr_wa <= to_unsigned(14, 6); tr_wd <= std_logic_vector(v_m);     -- S start 1 = start 0
                    when 16 => tr_we <= '1'; tr_wa <= to_unsigned(17, 6); tr_wd <= (others => '0');            -- S slope 0 = 0
                    when 19 => tr_we <= '1'; tr_wa <= to_unsigned(23, 6); tr_wd <= (others => '0');            -- Kd start 2 = 0
                    when 20 => tr_we <= '1'; tr_wa <= to_unsigned(24, 6); tr_wd <= (others => '0');            -- Kd start 3 = 0
                    when 21 => tr_we <= '1'; tr_wa <= to_unsigned(27, 6); tr_wd <= (others => '0');            -- Kd slope 2 = 0
                    when 22 => tr_we <= '1'; tr_wa <= to_unsigned(28, 6); tr_wd <= (others => '0');            -- Kd slope 3 = 0
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            g_ssr <= g_ssr(4 downto 0) & g_start;
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
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
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
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line: the five tables (the slope read from the
    -- table RAM 3 clocks before the product load, the start 3 clocks before
    -- the store; one serial product each, 16 iterations, result at load +
    -- 17), 1/w by a serial divide, the v start, the skirt DDA constants
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_yf : unsigned(18 downto 0);
        variable v_y  : unsigned(10 downto 0);
        variable v_rr : signed(14 downto 0);
        variable v_rm : unsigned(14 downto 0);
        variable v_b13 : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_lfsr <= e_lfsr(14 downto 0) & (e_lfsr(15) xor e_lfsr(13) xor e_lfsr(12) xor e_lfsr(10));
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);
            e_trd <= unsigned(tr_q);
            e_dy  <= e_yf - e_thr;
            if e_sub = '1' then
                e_ev <= resize(signed(e_trd), 18) - signed(resize(m_acc(31 downto 16), 18));
            else
                e_ev <= resize(signed(e_trd), 18) + signed(resize(m_acc(31 downto 16), 18));
            end if;

            -- serial divide d_q = 2^28 / w16 (29 iterations from the kick)
            if d_run = '1' then
                if d_i = 28 then v_rm := d_rem & '1'; else v_rm := d_rem & '0'; end if;
                if v_rm >= resize(e_w16, 15) then
                    v_rm := v_rm - resize(e_w16, 15);
                    d_q <= d_q(18 downto 0) & '1';
                else
                    d_q <= d_q(18 downto 0) & '0';
                end if;
                d_rem <= v_rm(13 downto 0);
                if d_i = 0 then d_run <= '0'; else d_i <= d_i - 1; end if;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    if s_flip = '1' then v_yf := to_unsigned(276480, 19) - e_vy; else v_yf := e_vy; end if;
                    if e_vy > 276479 then v_yf := to_unsigned(276479, 19); end if;
                    e_yf <= v_yf;
                when 2 =>
                    v_y := e_yf(18 downto 8);
                    if v_y >= 685 then e_rseg <= "11"; elsif v_y >= 455 then e_rseg <= "10"; elsif v_y >= 220 then e_rseg <= "01"; else e_rseg <= "00"; end if;
                    if v_y >= 790 then e_kseg <= "11"; elsif v_y >= 650 then e_kseg <= "10"; elsif v_y >= 480 then e_kseg <= "01"; else e_kseg <= "00"; end if;
                    if v_y >= 1000 then e_wseg <= "11"; elsif v_y >= 700 then e_wseg <= "10"; elsif v_y >= 480 then e_wseg <= "01"; else e_wseg <= "00"; end if;
                    if v_y >= 480 then e_sseg <= "11"; elsif v_y >= 300 then e_sseg <= "10"; elsif v_y >= 100 then e_sseg <= "01"; else e_sseg <= "00"; end if;
                    if v_y >= 600 then e_dseg <= "10"; elsif v_y >= 300 then e_dseg <= "01"; else e_dseg <= "00"; end if;
                -- One shared evaluator: e_ev = e_trd +- (dy * slope) >> 16 is formed every clock (e_sub
                -- picks the sign) and each table's arm clamps and stores it one clock later.  The
                -- threshold of the table's segment is selected a clock ahead of the shared dy subtract.
                -- ramp: slope at 0 .. 2 (segment - 1; segment 0 multiplies by 0), base at 37 .. 40
                when 3  => tr_ra <= resize(e_rseg, 6) - 1;
                when 4  => e_thr <= f_thr("000", e_rseg);
                when 6  =>
                    m_a <= resize(e_dy, 36); m_acc <= (others => '0');
                    if e_rseg = "00" then m_b <= (others => '0'); else m_b <= e_trd; end if;
                when 21 => tr_ra <= resize(e_rseg, 6) + 37;
                when 22 => tr_ra <= resize(e_kseg, 6) + 9;                 -- K slope
                when 23 => e_sub <= '0'; e_thr <= f_thr("001", e_kseg);
                when 25 =>
                    if e_ev > 1536 then e_ramp <= to_signed(1536, 14); else e_ramp <= e_ev(13 downto 0); end if;
                    m_a <= resize(e_dy, 36); m_acc <= (others => '0'); m_b <= e_trd;
                when 26 =>
                    -- breathe: slide the ramp under its fixed plateau / black clips
                    v_rr := resize(e_ramp, 15) + resize(c_lfo, 15);
                    if v_rr < -159 then v_rr := to_signed(-159, 15); end if;
                    if v_rr > 1536 then v_rr := to_signed(1536, 15); end if;
                    e_ramp <= v_rr(13 downto 0);
                when 27 =>
                    v_b13 := resize(e_ramp, 13) + 768;
                    n_base <= unsigned(v_b13(11 downto 0));
                    if v_b13 < 640 then n_db <= unsigned(resize(640 - v_b13, 6)); else n_db <= (others => '0'); end if;
                when 40 => tr_ra <= resize(e_kseg, 6) + 5;                 -- K start
                when 41 => tr_ra <= resize(e_wseg, 6) + 33;                -- w slope
                when 42 =>
                    if e_kseg = "10" then e_sub <= '0'; else e_sub <= '1'; end if;
                    e_thr <= f_thr("010", e_wseg);
                when 44 =>
                    if e_ev < 0 then n_k <= (others => '0');
                    elsif e_ev > 4095 then n_k <= (others => '1');
                    else n_k <= unsigned(e_ev(11 downto 0)); end if;
                    m_a <= resize(e_dy, 36); m_acc <= (others => '0'); m_b <= e_trd;
                when 59 => tr_ra <= resize(e_wseg, 6) + 29;                -- w start
                when 60 => tr_ra <= resize(e_sseg, 6) + 17;                -- S slope
                when 61 =>
                    e_sub <= not e_wseg(1);
                    e_thr <= f_thr("011", e_sseg);
                when 63 =>
                    if e_ev < 320 then e_w16 <= to_unsigned(320, 14);
                    elsif e_ev > 16383 then e_w16 <= (others => '1');
                    else e_w16 <= unsigned(e_ev(13 downto 0)); end if;
                    m_a <= resize(e_dy, 36); m_acc <= (others => '0'); m_b <= e_trd;
                when 64 =>
                    d_run <= '1'; d_i <= to_unsigned(28, 5); d_rem <= (others => '0'); d_q <= (others => '0');
                when 78 => tr_ra <= resize(e_sseg, 6) + 13;                -- S start
                when 79 => tr_ra <= resize(e_dseg, 6) + 25;                -- Kd slope
                when 80 => e_sub <= '0'; e_thr <= f_thr("100", e_dseg);
                when 82 =>
                    if e_ev > 4000 then e_s <= to_unsigned(4000, 12); else e_s <= unsigned(e_ev(11 downto 0)); end if;
                    m_a <= resize(e_dy, 36); m_acc <= (others => '0'); m_b <= e_trd;
                when 97 => tr_ra <= resize(e_dseg, 6) + 21;                -- Kd start
                when 99 => e_sub <= '1';
                when 101 =>
                    if e_ev < 0 then n_kd <= (others => '0');
                    elsif e_ev > 2047 then n_kd <= (others => '1');
                    else n_kd <= unsigned(e_ev(10 downto 0)); end if;
                -- 1/w ready at 94: v start = u0h * g (+-2 g on jitter lines)
                when 102 =>
                    n_g <= d_q;
                    m_a <= resize(d_q, 36); m_b <= resize(c_u0h, 16); m_acc <= (others => '0');
                when 120 =>
                    if e_lfsr(5 downto 0) = "000000" then
                        if e_lfsr(6) = '1' then n_a0 <= m_acc(31 downto 0) + shift_left(resize(n_g, 32), 1);
                        else n_a0 <= m_acc(31 downto 0) - shift_left(resize(n_g, 32), 1); end if;
                    else
                        n_a0 <= m_acc(31 downto 0);
                    end if;
                -- skirt constants: f0, d0, i2 = S * (per-resolution constants)
                when 121 =>
                    m_a <= resize(c_cf0, 36); m_b <= resize(e_s, 16); m_acc <= (others => '0');
                when 139 => n_f0 <= m_acc(35 downto 8);
                when 140 =>
                    m_a <= resize(c_cd0, 36); m_b <= resize(e_s, 16); m_acc <= (others => '0');
                when 158 => n_d0 <= m_acc(26 downto 8);
                when 159 =>
                    m_a <= resize(c_ci2, 36); m_b <= resize(e_s, 16); m_acc <= (others => '0');
                when 177 => n_i2 <= m_acc(19 downto 8);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): x in h, the v accumulator A = |x - axis| g (mirrored at
    -- the axis); the skirt parabola F by a second-order DDA whose increment
    -- negates at the axis step, run five clocks late so it is the stage-6
    -- pixel's value; the camera luma read two clocks late (stage 6 data)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_x1 <= c_xb0 + s_xs;
                b_x2 <= c_xb0 + shift_left(resize(s_xs, 12), 1);
                b_px <= (others => '0');
                b_a  <= n_a0;
                b_al <= '1';
                b_ax <= '0';
                l_base <= n_base; l_db <= n_db; l_k <= n_k; l_kd <= n_kd;
                case to_integer(s_xs) is                           -- the per-pixel step g * xs
                    when 4      => l_gx <= shift_left(resize(n_g, 22), 2);
                    when 3      => l_gx <= shift_left(resize(n_g, 22), 1) + resize(n_g, 22);
                    when others => l_gx <= shift_left(resize(n_g, 22), 1);
                end case;
                l_i2 <= n_i2;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x1;
                b_x1 <= b_x2;
                b_x2 <= b_x2 + s_xs;
                if b_al = '1' then b_a <= b_a - resize(l_gx, 32); else b_a <= b_a + resize(l_gx, 32); end if;
                -- flags for the next step: x(n+2) = b_x2
                if b_x2 <= 1920 then b_al <= '1'; else b_al <= '0'; end if;
                if b_x2 = 1920 then b_ax <= '1'; else b_ax <= '0'; end if;
            end if;
            b_axs <= b_axs(3 downto 0) & b_ax;
            -- the skirt DDA, five clocks behind the beam (stage 6)
            if g_ssr(4) = '1' then
                b_f <= signed(resize(n_f0, 29));
                b_d <= signed(resize(n_d0, 22));
            elsif s_gav_sr(4) = '1' then
                b_f <= b_f + resize(b_d, 29);
                if b_axs(4) = '1' then b_d <= -b_d; else b_d <= b_d + signed(resize(l_i2, 22)); end if;
            end if;
            -- the camera luma read, three clocks behind the beam (data at stage 6)
            if g_ssr(1) = '1' then
                b_px3 <= (others => '0');
            elsif s_gav_sr(1) = '1' then
                b_px3 <= b_px3 + 1;
            end if;
            s_lb_ra <= b_px3;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t  : unsigned(11 downto 0);
        variable v_g  : signed(13 downto 0);
        variable v_x  : signed(15 downto 0);
        variable v_tri : unsigned(8 downto 0);
        variable v_b  : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_c  : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: profile ROM address (v saturated at 4.0)
            if b_a(31 downto 27) /= "00000" then r_pf_a <= (others => '1'); else r_pf_a <= b_a(26 downto 17); end if;
            -- 3: ROM data (EBR)
            -- 4: fabric copy of the ROM word
            p4_h <= unsigned(r_pf_q(19 downto 8)); p4_d <= unsigned(r_pf_q(7 downto 0));
            -- 5: K h partials (h hi / lo x K), Kd dimple
            p5_a <= resize(p4_h(11 downto 6) * l_k, 18);
            p5_b <= resize(p4_h(5 downto 0) * l_k, 18);
            p5_dd <= resize(p4_d * l_kd, 19);
            -- 6: K h (1/256 ring), Kd d; the camera luma (fabric copy of the line buffer read)
            p6_kh <= resize(shift_right(shift_left(resize(p5_a, 24), 6) + resize(p5_b, 24), 10), 14);
            p6_dd <= resize(shift_right(p5_dd, 7), 12);
            p6_lm <= unsigned(s_lb_q);
            -- 7: base + skirt, K h - Kd d, camera x gain (8 x 5)
            p7_t1 <= resize(l_base, 13) + resize(unsigned(b_f(27 downto 16)), 13);
            p7_t2 <= signed(resize(p6_kh, 15)) - signed(resize(p6_dd, 15));
            p7_ca <= p6_lm * c_cg(9 downto 5);
            -- 8: raw phase (+3 rings bias); camera term
            p8_a <= signed(resize(p7_t1, 16)) - resize(p7_t2, 16);
            p8_cam <= resize(shift_right(p7_ca, 4), 9);
            -- 9: + camera
            p9_a <= p8_a + signed(resize(p8_cam, 16));
            -- 10: the wavefolder (Rim on): |a|
            if s_rim = '1' then p10_x <= abs(p9_a); else p10_x <= p9_a; end if;
            p10_neg <= p9_a(15);
            -- 11: distances to the R peak (1.525 + 3 rings) and the G knee (1.2 + 3); B's copy: ramp clipped at -0.5
            p11_dr <= p10_x - 1158;
            p11_dg <= p10_x - 1075;
            if p10_neg = '1' and s_rim = '1' then p11_xb <= p10_x - signed(resize(l_db, 16));
            else p11_xb <= p10_x + signed(resize(l_db, 16)); end if;
            -- 12: |dr|; B phase clipped at 5.5 rings
            v_x := abs(p11_dr);
            p12_dr <= unsigned(v_x(13 downto 0));
            p12_dg <= p11_dg;
            if p11_xb > 2176 then p12_pb <= to_signed(1408, 16); else p12_pb <= p11_xb - 768; end if;
            -- 13: 15 dr, 5 dg; B phase x rate partials (modular: only the low 12 bits of the product are used)
            p13_t1 <= shift_left(resize(p12_dr, 18), 4) - resize(p12_dr, 18);
            p13_g1 <= shift_left(resize(p12_dg, 19), 2) + resize(p12_dg, 19);
            p13_pa <= resize(unsigned(p12_pb(8 downto 0)) * c_rate(6 downto 3), 12);
            p13_pq <= resize(unsigned(p12_pb(11 downto 0)) * c_rate(2 downto 0), 12);
            -- 14: R = 255 - 15 |dr| / 64, G = 128 - 5 dg / 32 (clamped), the B product
            v_t := resize(p13_t1(17 downto 6), 12);
            if v_t >= 255 then p14_r <= (others => '0'); else p14_r <= 255 - v_t(7 downto 0); end if;
            v_g := resize(p13_g1(18 downto 5), 14);
            if v_g < 0 then p14_g <= to_unsigned(128, 8);
            elsif v_g >= 128 then p14_g <= (others => '0');
            else p14_g <= 128 - unsigned(v_g(7 downto 0)); end if;
            p14_pp <= shift_left(p13_pa, 3) + p13_pq;
            -- 15: the B triangle input
            p15_r <= p14_r; p15_g <= p14_g;
            p15_tb <= p14_pp(11 downto 4);
            -- 16: the B triangle; R, G delayed
            if p15_tb(7) = '0' then v_tri := to_unsigned(255, 9) - (p15_tb(6 downto 0) & '0');
            else v_tri := "0" & p15_tb(6 downto 0) & '1'; end if;
            p16_tri <= v_tri(7 downto 0);
            p16_r <= p15_r; p16_g <= p15_g;
            -- 17: B shaper (2 tri - 128, clipped), palette (Ice swaps the red and blue colourisers)
            if p16_tri < 64 then v_b := (others => '0');
            elsif p16_tri >= 192 then v_b := to_unsigned(255, 10);
            else v_b := resize((p16_tri & '0') - 128, 10); end if;
            if s_pal = '1' then p17_r <= v_b(7 downto 0); p17_b <= p16_r;
            else p17_r <= p16_r; p17_b <= v_b(7 downto 0); end if;
            p17_g <= p16_g;
            -- 18: G delay ring write; the first dly pixels of a line take the undelayed value
            gd_wd <= std_logic_vector(p17_g);
            gd_wa <= gd_wa + 1;
            gd_ra <= gd_wa + 1 - resize(c_dly, 8);
            if s_gav_sr(15) = '1' and s_gav_sr(16) = '0' then v_c := (others => '0');
            elsif p18_cnt /= 127 then v_c := p18_cnt + 1; else v_c := p18_cnt; end if;
            p18_cnt <= v_c;
            if v_c < resize(c_dly, 7) then p18_early <= '1'; else p18_early <= '0'; end if;
            p18_r <= p17_r; p18_b <= p17_b; p18_gd <= p17_g;
            -- 19: ring data (EBR)
            p19_r <= p18_r; p19_b <= p18_b; p19_gd <= p18_gd; p19_early <= p18_early;
            -- 20: fabric copy / early mux
            if p19_early = '1' then p20_g <= p19_gd; else p20_g <= unsigned(gd_q); end if;
            p20_r <= p19_r; p20_b <= p19_b;
            -- 21: matrix products: Y = 64 + (5R + 10G + 2B) / 4; Cr = 512 + (56R - 47G - 9B) / 32;
            --     Cb = 512 + (-19R - 37G + 56B) / 32   (luma pre-scaled 16/13, capped 940)
            p21_ry <= shift_left(resize(p20_r, 12), 2) + resize(p20_r, 12);
            p21_gy <= shift_left(resize(p20_g, 12), 3) + shift_left(resize(p20_g, 12), 1);
            p21_by <= shift_left(resize(p20_b, 12), 1);
            p21_rr <= shift_left(resize(p20_r, 14), 6) - shift_left(resize(p20_r, 14), 3);
            p21_gr <= shift_left(resize(p20_g, 14), 5) + shift_left(resize(p20_g, 14), 4) - resize(p20_g, 14);
            p21_br <= shift_left(resize(p20_b, 14), 3) + resize(p20_b, 14);
            p21_rb <= shift_left(resize(p20_r, 14), 4) + shift_left(resize(p20_r, 14), 1) + resize(p20_r, 14);
            p21_gb <= shift_left(resize(p20_g, 14), 5) + shift_left(resize(p20_g, 14), 2) + resize(p20_g, 14);
            p21_bb <= shift_left(resize(p20_b, 14), 6) - shift_left(resize(p20_b, 14), 3);
            -- 22: sums
            p22_y <= resize(p21_ry, 14) + resize(p21_gy, 14) + resize(p21_by, 14);
            p22_u <= signed(resize(p21_rr, 16)) - signed(resize(p21_gr, 16)) - signed(resize(p21_br, 16));
            p22_v <= signed(resize(p21_bb, 16)) - signed(resize(p21_rb, 16)) - signed(resize(p21_gb, 16));
            -- 23: offsets, luma cap 940, black outside the generated avid
            v_y := resize(p22_y(13 downto 2), 12) + 64;
            if s_gav_sr(20) = '0' then
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            else
                if v_y > 940 then s_y23 <= to_unsigned(940, 10); else s_y23 <= v_y(9 downto 0); end if;
                v_x := shift_right(p22_u, 5) + 512; s_u23 <= unsigned(v_x(9 downto 0));
                v_x := shift_right(p22_v, 5) + 512; s_v23 <= unsigned(v_x(9 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(20);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the 8-back taps come from block-RAM rings:
    -- the still's analog softness, and the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate; the syncs ride a block-RAM ring of the same latency
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- ring addresses: one write per clock; the read, issued two clocks ahead and copied once,
            -- is the sample written 8 clocks before the one being added
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 5;
            sy_ra <= rg_wa - 25;
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 24 sums, stage 25 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y23 - s_cay;
                s_bau <= s_bau + s_u23 - s_cau;
                s_bav <= s_bav + s_v23 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(22) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y25 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u25 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v25 <= v_sc(13 downto 4);
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(22);
            -- box B: stage 26 sums, stage 27 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y25 - s_cby;
                s_bbu <= s_bbu + s_u25 - s_cbu;
                s_bbv <= s_bbv + s_v25 - s_cbv;
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y27 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u27 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v27 <= v_sc(13 downto 4);
            -- 28: luma 13/16
            if s_y27 > 64 then v_dl := s_y27 - 64; else v_dl := (others => '0'); end if;
            s_y28 <= s_y27 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u28 <= s_u27; s_v28 <= s_v27;
            -- 29: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y29 <= s_y28; s_u29 <= s_u28; s_v29 <= s_v28;
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y29);
    data_out.u       <= std_logic_vector(s_u29);
    data_out.v       <= std_logic_vector(s_v29);
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture chalice;
