-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Platter v0.1.1 -- Heirloom from the video clips, built from the still WIN_20231204_21_01_30 t0.5.
--
-- v0.1.1: no single-row sweep line at the right tube's pipe-end step (the warp tables are low-passed on each side of
-- the step and the right zone is ground on its steep rows while the warp is on); Camera default 128, the Osc fed
-- at 3x the camera's level.
--
-- The still: a pink "vinyl" disc (a 4:3 circle shown 16:9, centre 982,538, 561 x 427 px) filled with
-- horizontally smeared dots of noise, coloured orange in the middle and magenta / lavender at the sides, a black
-- spindle hole, four soft balls riding it (a big magenta one on the rim, a grey and a small magenta one orbiting
-- the hole, a faint dark red one), all over a black ground of shaded tubes: a wide yellow-peach tube on the left,
-- an S-shaped yellow tube on the right, pink tubes passing behind the disc above and below it.
--
-- The patch, as rebuilt (platter.py has the tables; .heirloom_work/rb_platter/notes.md the measurements), in
-- virtual 1080-line coordinates:
--   tubes  S = F(x) kf - W(zone, y) - camera      F = |x - 560| + 422 (= |x - 982| left of 560), three zones
--          (x < 560, 560..1320, >= 1320) with per-line warps measured on the still; W = base + m (W_still - base),
--          m = cos(2 pi t / 5.6 s) x Warp: a standing wave breathing through the still's pose and straight columns
--          colour = profile(S mod 1300 px): glow, a pink rim, a dark gap, a peach-yellow body, a pink ghost tube;
--          the centre tubes pinker, the right ones yellower
--   disc   |X| < a sqrt(1 - (Y/b)^2) (a per-line chord); texture = value noise (16 x 5.2 px cells, bilinear in
--          quarter steps, a new pattern every two fields) through the still's luma curve, + camera; Cr 680,
--          Cb = 500 - 85 cos(2 pi (x - 950) / 820 x Bands)
--   balls  per-row chord tables (fixed sizes) at orbit positions computed per field: a grey and a small magenta
--          ball on an 8.8 s ellipse about the hole (counter-clockwise), the big magenta one creeping round the rim
--          at 1/8 of that rate; the black spindle hole on top
--
--   K1  Platter   disc size x0.3 .. x1.35 (the balls' orbits scale with it)
--   K2  Grain     texture blob size x0.5 .. x3
--   K3  Bands     the disc's colour oscillator: one flat colour .. the still's one orange band .. six bands
--   K4  Orbit     bipolar ball orbit speed (+-8 deadband at 512); 700 = the clip (8.8 s, counter-clockwise)
--   K5  Tubes     tube spacing x0.5 .. x3 (fat single tubes .. many thin ones)
--   K6  Camera    how far the picture (or the Osc) brightens the dots and bends the tubes
--   S7  Flip      Off (the still) / On (the tubes' measured warps read bottom-up: the S and the bends upside down)
--   S8  Colours   Rose (the still) / Neon (green / magenta tubes, a cyan-blue platter)
--   S9  Mirror    Off / On (the left half shows the right half's tubes)
--   S10 Window    Disc (the still) / Window (the tubes inside the circle, the dots outside)
--   S11 Source    Video / Osc
--   P12 WARP      tube warp depth: 0 = straight still columns; 440 = the still (breathing on 5.6 s); 100 % = 2.4x,
--                 breathing twice as fast
--
-- Engine: vblank-zone sequencer (shared glide, one serial multiplier with staged operands, a serial divider, sine /
-- exp2 / sqrt ROM), a per-line engine kicked at each line start computing the NEXT line (warp table row, breathing,
-- the disc chord, texture rows; the ball rows in hblank), a 15-stage pixel path, the shell's output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes with block-RAM taps and the resync, luma 13/16) and the blanking gate.
-- Generated avid; one latency (20) in every mode; colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.platter_pkg.all;
use work.all;

architecture platter of program_top is
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
    -- 16-bit xorshift (7, 9, 8): one new noise byte per cell
    function f_xs16(s : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := s xor shift_left(s, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        return v;
    end function;

    -- line buffer (camera, every other pixel): (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_pf_a : unsigned(8 downto 0) := (others => '0');
    signal r_pf_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_wt_a : unsigned(8 downto 0) := (others => '0');
    signal r_wt_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_sq_a, r_sq_az, r_sq_ae : unsigned(9 downto 0) := (others => '0');
    signal r_sq_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_cs_a, r_lm_a, r_bl_a : unsigned(7 downto 0) := (others => '0');
    signal r_cs_q, r_lm_q, r_bl_q : std_logic_vector(15 downto 0) := (others => '0');

    attribute no_rw_check : boolean;

    -- output box rings (the 8-back taps of the two running sums), the pixel-path delay rings, the sync ring
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ba1, ba2, bb1, bb2, dra, drb : t_r16 := (others => x"0000");
    attribute no_rw_check of ba1 : signal is true;
    attribute no_rw_check of ba2 : signal is true;
    attribute no_rw_check of bb1 : signal is true;
    attribute no_rw_check of bb2 : signal is true;
    attribute no_rw_check of dra : signal is true;
    attribute no_rw_check of drb : signal is true;
    signal rg_wa, rg_ra, sy_ra, dr_ra : unsigned(7 downto 0) := (others => '0');
    signal ba1_q, ba2_q, bb1_q, bb2_q, dra_q, drb_q : std_logic_vector(15 downto 0) := x"0000";
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
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(700 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(128 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(440 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_flp, s_neon, s_mir, s_win, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field sequencer: serial multiplier (right-shifting accumulator), divider, scratch
    signal f_a   : unsigned(23 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(39 downto 0) := (others => '0');
    signal f_cnt : unsigned(4 downto 0) := (others => '0');
    signal z_neg : std_logic := '0';
    signal z_pre, z_ld : std_logic := '0';
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_fa  : unsigned(23 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sq  : signed(15 downto 0) := (others => '0');            -- fabric copy of the sine ROM
    signal z_axc : unsigned(9 downto 0) := (others => '0');           -- C_AX(z_k), registered
    signal z_mqm : unsigned(10 downto 0) := (others => '0');
    signal c_xa0m : unsigned(11 downto 0) := (others => '0');
    signal z_x19 : unsigned(10 downto 0) := to_unsigned(1884, 11);
    signal z_sh  : signed(2 downto 0) := (others => '0');
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_dv  : unsigned(16 downto 0) := to_unsigned(109312, 17);
    signal z_rem : unsigned(17 downto 0) := (others => '0');
    signal z_quo : unsigned(8 downto 0) := (others => '0');
    signal z_v   : unsigned(9 downto 0) := (others => '0');
    signal z_vn  : std_logic := '0';
    signal z_sn  : std_logic := '0';
    signal z_k   : unsigned(2 downto 0) := (others => '0');
    signal z_t   : unsigned(23 downto 0) := (others => '0');
    signal z_seed : unsigned(15 downto 0) := x"5A3C";
    signal c_fc   : unsigned(15 downto 0) := (others => '0');
    signal c_kd8  : unsigned(8 downto 0) := to_unsigned(256, 9);     -- disc size, Q8
    signal c_ib   : unsigned(8 downto 0) := to_unsigned(153, 9);     -- 256 / b per virtual line
    signal c_a2   : unsigned(10 downto 0) := to_unsigned(1122, 11);  -- disc half-width a in h
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);    -- beam start x (h)
    signal c_x0h  : signed(12 downto 0) := (others => '0');          -- start x - 1964 (h)
    signal c_xa0  : signed(12 downto 0) := (others => '0');          -- start xa - 1120 (h)
    signal c_th3  : unsigned(18 downto 0) := to_unsigned(56882, 19); -- orbit phase (Q16 turns + 3 bits): the still;
                                                                         -- B1 runs at 1/8 of it (bits 18 .. 3)
    signal c_w3 : signed(15 downto 0) := (others => '0');
    signal c_phb  : unsigned(15 downto 0) := (others => '0');        -- breathing phase (0 = the still's pose)
    signal c_phst : unsigned(15 downto 0) := to_unsigned(195, 16);
    signal c_w8   : unsigned(9 downto 0) := to_unsigned(256, 10);    -- warp depth, Q8
    signal z_mq   : signed(11 downto 0) := (others => '0');
    signal c_m16m : unsigned(15 downto 0) := to_unsigned(12905, 16); -- |m| x 50.41, Q8 (profile units per px)
    signal c_m16s : std_logic := '0';
    signal c_ig12 : unsigned(13 downto 0) := to_unsigned(4096, 14);  -- 1 / grain, Q12
    signal c_cst  : unsigned(15 downto 0) := to_unsigned(4096, 16);  -- texture column step per pixel (Q16 cells)
    signal c_rst  : unsigned(17 downto 0) := to_unsigned(12603, 18); -- texture row step per line (Q16 rows)
    signal c_stq  : unsigned(15 downto 0) := to_unsigned(10230, 16); -- colour oscillator step per h (Q24 turns)
    signal c_hst  : unsigned(17 downto 0) := to_unsigned(20460, 18);
    signal c_ph0  : unsigned(23 downto 0) := (others => '0');
    signal c_kf   : unsigned(14 downto 0) := to_unsigned(6453, 15);  -- tube field gain, Q8 profile units per h
    signal c_fk0  : signed(27 downto 0) := (others => '0');          -- (xa0 - 1120) KF, Q8
    signal c_fks  : unsigned(17 downto 0) := (others => '0');        -- xs KF, Q8
    signal c_fko  : unsigned(15 downto 0) := to_unsigned(21274, 16); -- 844 KF / 256
    signal c_g6c  : unsigned(4 downto 0) := to_unsigned(4, 5);
    type t_b11 is array (0 to 2) of unsigned(10 downto 0);
    signal c_bx, c_by : t_b11 := (others => (others => '0'));          -- ball centres (virtual px / lines)

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
    signal e_vl   : unsigned(10 downto 0) := (others => '0');
    signal e_ay   : unsigned(10 downto 0) := (others => '0');
    signal e_pos  : unsigned(18 downto 0) := (others => '0');
    type t_d3 is array (0 to 2) of unsigned(10 downto 0);
    signal e_d    : t_d3 := (others => (others => '0'));       -- |D| (virtual px)
    signal e_ds   : std_logic_vector(2 downto 0) := "000";     -- sign of D
    signal w_base : unsigned(15 downto 0) := (others => '0');
    signal w_neg  : std_logic := '0';
    signal e_u : unsigned(10 downto 0) := (others => '0');
    signal e_sq   : unsigned(15 downto 0) := (others => '0');
    signal e_r1   : unsigned(23 downto 0) := (others => '0');
    signal e_hv   : unsigned(15 downto 0) := (others => '0');
    signal e_bi   : unsigned(2 downto 0) := (others => '0');
    signal e_bv   : std_logic := '0';
    signal e_dy   : signed(11 downto 0) := (others => '0');
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal m_go   : std_logic := '0';
    signal m_sel  : unsigned(2 downto 0) := (others => '0');
    signal n_hw : unsigned(11 downto 0) := (others => '0');
    signal n_w0, n_w1, n_w2 : unsigned(15 downto 0) := (others => '0');
    signal n_g2, l_g2 : std_logic := '0';
    signal n_fy   : unsigned(1 downto 0) := (others => '0');
    signal n_sa, n_sb : unsigned(15 downto 0) := x"0001";
    type t_hw2 is array (0 to 3) of unsigned(8 downto 0);
    type t_sh  is array (0 to 4) of unsigned(7 downto 0);
    signal l_hw2 : t_hw2 := (others => (others => '0'));
    signal y_cnt : unsigned(5 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_fk  : signed(27 downto 0) := (others => '0');
    signal b_dn  : std_logic := '0';                                    -- Mirror: xa falling
    signal b_ph  : unsigned(23 downto 0) := (others => '0');
    signal b_c1 : unsigned(15 downto 0) := (others => '0');
    signal b_la, b_lb : unsigned(15 downto 0) := x"0001";
    signal b_a0, b_a1, b_b0, b_b1 : unsigned(7 downto 0) := (others => '0');
    signal l_hw : unsigned(11 downto 0) := (others => '0');
    signal l_w0, l_w1, l_w2 : unsigned(15 downto 0) := (others => '0');
    signal l_fy  : unsigned(1 downto 0) := (others => '0');

    -- pixel path
    signal p2_ax : unsigned(11 downto 0) := (others => '0');
    signal p2_z  : unsigned(1 downto 0) := (others => '0');
    signal p2_fk : unsigned(15 downto 0) := (others => '0');
    signal p2_da, p2_db : signed(8 downto 0) := (others => '0');
    signal p2_a0, p2_b0 : unsigned(7 downto 0) := (others => '0');
    signal p2_fx : unsigned(1 downto 0) := (others => '0');
    signal p3_fk, p4_fk : unsigned(15 downto 0) := (others => '0');
    signal p3_z, p4_z, p5_z, p6_z : unsigned(1 downto 0) := (others => '0');
    signal p3_in, p4_in, p5_in, p6_in : std_logic := '0';
    signal p3_g, p4_g, p5_g, p6_g : std_logic := '0';
    type t_dx is array (0 to 2) of signed(12 downto 0);
    signal p2_dx : t_dx := (others => (others => '0'));
    signal p3_bi, p4_bi, p5_bi : std_logic_vector(3 downto 0) := (others => '0');
    signal p3_pa, p3_pb : signed(10 downto 0) := (others => '0');
    signal p3_a0, p3_b0 : unsigned(7 downto 0) := (others => '0');
    signal p3_lm : unsigned(7 downto 0) := (others => '0');
    signal p4_w  : unsigned(15 downto 0) := (others => '0');
    signal p4_na, p4_nb : unsigned(7 downto 0) := (others => '0');
    signal p4_cb, p5_cb, p6_cb : unsigned(9 downto 0) := (others => '0');
    signal p4_cam : signed(16 downto 0) := (others => '0');
    signal p5_s1, p5_ct : unsigned(15 downto 0) := (others => '0');
    signal p5_cy, p6_cy : signed(10 downto 0) := (others => '0');
    signal p5_nd : signed(8 downto 0) := (others => '0');
    signal p5_na, p6_na, p7_n1 : unsigned(7 downto 0) := (others => '0');
    signal p6_s  : unsigned(15 downto 0) := (others => '0');
    signal p6_pn : signed(10 downto 0) := (others => '0');
    signal p6_bs : unsigned(2 downto 0) := (others => '0');
    signal p8_n  : unsigned(7 downto 0) := (others => '0');
    signal p9_ty, p10_ty, p11_ty : unsigned(9 downto 0) := (others => '0');
    signal p9_tu, p9_tv, p10_tu, p10_tv, p11_tu, p11_tv : unsigned(9 downto 0) := (others => '0');
    signal p10_wl, p11_wl : unsigned(1 downto 0) := (others => '0');
    signal p11_dy : unsigned(9 downto 0) := (others => '0');
    signal q11_cb : unsigned(9 downto 0) := (others => '0');
    signal q11_cy : signed(10 downto 0) := (others => '0');
    signal q11_bs, p12_bs : unsigned(2 downto 0) := (others => '0');
    signal q11_z  : unsigned(1 downto 0) := (others => '0');
    signal q11_in, p12_in : std_logic := '0';
    signal p12_ty, p12_tu, p12_tv, p12_dy, p12_du, p12_dv : unsigned(9 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

    -- ball constants: rows ry, ROM bases; colours (pre-scaled luma; Rose / Neon chroma)
    type t_i5 is array (0 to 4) of integer;
    constant C_BRY  : t_i5 := (58, 22, 24, 22, 0);
    constant C_BBASE : t_i5 := (0, 64, 96, 160, 0);
    constant C_BYC  : t_i5 := (398, 428, 391, 81, 0);
    constant C_BU   : t_i5 := (875, 508, 820, 530, 0);
    constant C_BV   : t_i5 := (555, 516, 596, 508, 0);
    constant C_BUN  : t_i5 := (149, 516, 204, 494, 0);
    constant C_BVN  : t_i5 := (773, 514, 781, 519, 0);
    -- orbit amplitudes (virtual px): B1 x, y, B2 x, y, B3 x, y, B4 x, y
    type t_i8 is array (0 to 7) of integer;
    constant C_AX : t_i8 := (520, 200, 240, 72, 187, 69, 0, 0);

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    -- the sine / exp2 / sqrt ROM: the sequencer in the first half of the zone, the line engine otherwise
    r_sq_a <= r_sq_az when s_zone = '1' and s_zc(10) = '0' else r_sq_ae;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_pf_q <= C_PL_PROF(to_integer(r_pf_a));
            r_wt_q <= C_PL_WT(to_integer(r_wt_a));
            r_sq_q <= C_PL_SEQ(to_integer(r_sq_a));
            r_cs_q <= C_PL_COS(to_integer(r_cs_a));
            r_lm_q <= C_PL_LUM(to_integer(r_lm_a));
            r_bl_q <= C_PL_BALL(to_integer(r_bl_a));
        end if;
    end process p_rom;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u15(3 downto 0)) & std_logic_vector(s_v15) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y17) & std_logic_vector(s_u17(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u17(3 downto 0)) & std_logic_vector(s_v17) & "00";
            -- pixel-path delay rings: the stage-6 values come back as stage-11 values
            dra_q <= dra(to_integer(dr_ra));
            dra(to_integer(rg_wa)) <= std_logic_vector(p6_cb) & std_logic_vector(p6_bs) & std_logic_vector(p6_z) & p6_in;
            drb_q <= drb(to_integer(dr_ra));
            drb(to_integer(rg_wa)) <= "00000" & std_logic_vector(p6_cy);
            syr_q <= syr(to_integer(sy_ra));
            syr(to_integer(rg_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(12 downto 0);
        variable v_u  : unsigned(15 downto 0);
        variable v_s  : signed(15 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_t  : unsigned(19 downto 0);
        variable v_r  : unsigned(18 downto 0);
        variable v_m  : unsigned(24 downto 0);
        variable v_b11 : unsigned(10 downto 0);
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
                s_flp  <= registers_in(6)(0);
                s_neon <= registers_in(6)(1);
                s_mir  <= registers_in(6)(2);
                s_win  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                c_fc   <= c_fc + 1;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, four channels
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
                s_g2 <= unsigned(registers_in(1)) & "000";       -- grain, orbit speed and camera gain: not glided
                s_g4 <= unsigned(registers_in(3)) & "000";
                s_g6 <= unsigned(registers_in(5)) & "000";
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 2      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
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
                            when 1      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 3 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial multiplier: right-shifting accumulator, 16 iterations (a x b complete 17 clocks after the load).
            -- A product loaded at zone clock L is decoded into a slot number at L - 2 (z_sel, no other logic on
            -- the zone-counter decode), its operands are muxed from that slot number into z_fa / z_fb at L - 1, and
            -- the multiplier loads from them at L.
            if f_cnt /= 0 then                                -- 16 iterations, then hold the product
                if f_b(0) = '1' then v_m := ('0' & f_acc(39 downto 16)) + resize(f_a, 25);
                else v_m := '0' & f_acc(39 downto 16); end if;
                f_acc <= v_m & f_acc(15 downto 1);
                f_b <= '0' & f_b(15 downto 1);
                f_cnt <= f_cnt - 1;
            end if;
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';      -- disc size kd8
                    when 50  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';      -- b = 427 kd
                    when 80  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';      -- a = 561 kd (h)
                    when 110 | 150 | 190 | 230 | 270 | 310 => z_sel <= to_unsigned(3, 5); z_pre <= '1';   -- A kd
                    when 128 | 168 | 208 | 248 | 288 | 328 => z_sel <= to_unsigned(4, 5); z_pre <= '1';   -- A kd |sin|
                    when 450 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';      -- warp depth w8
                    when 470 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';      -- m = cos x w
                    when 490 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';      -- m x 50.41
                    when 510 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';      -- breathing rate
                    when 530 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';      -- rows: ys x ig
                    when 550 => z_sel <= to_unsigned(10, 5); z_pre <= '1';      -- row step
                    when 570 => z_sel <= to_unsigned(11, 5); z_pre <= '1';      -- colour oscillator start phase
                    when 590 => z_sel <= to_unsigned(12, 5); z_pre <= '1';      -- orbit step
                    when 630 => z_sel <= to_unsigned(14, 5); z_pre <= '1';      -- tube gain KF
                    when 650 => z_sel <= to_unsigned(15, 5); z_pre <= '1';      -- tube field start
                    when 670 => z_sel <= to_unsigned(16, 5); z_pre <= '1';      -- 844 KF
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            z_sq <= signed(r_sq_q);
            z_axc <= to_unsigned(C_AX(to_integer(z_k)), 10);
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 =>
                        if s_g1(12 downto 3) <= 512 then
                            z_fa <= resize(s_g1(12 downto 3), 24); z_fb <= to_unsigned(358, 16); z_neg <= '0';
                        else
                            z_fa <= resize(s_g1(12 downto 3) - 512, 24); z_fb <= to_unsigned(181, 16); z_neg <= '1';
                        end if;
                    when 1  => z_fa <= resize(c_kd8, 24); z_fb <= to_unsigned(427, 16);
                    when 2  => z_fa <= resize(c_kd8, 24); z_fb <= to_unsigned(1122, 16);
                    when 3  => z_fa <= resize(z_axc, 24); z_fb <= resize(c_kd8, 16);
                    when 4 | 6 =>
                        z_sn <= z_sq(15); z_fb <= unsigned(abs(z_sq));
                        if z_sel(1) = '0' then z_fa <= resize(f_acc(17 downto 8), 24); else z_fa <= resize(c_w8, 24); end if;
                    when 5 =>
                        if s_gp(12 downto 3) <= 440 then
                            z_fa <= resize(s_gp(12 downto 3), 24); z_fb <= to_unsigned(149, 16); z_neg <= '0';
                        else
                            z_fa <= resize(s_gp(12 downto 3) - 440, 24); z_fb <= to_unsigned(157, 16); z_neg <= '1';
                        end if;
                    when 7  => z_fa <= resize(z_mqm, 24); z_fb <= to_unsigned(12905, 16);
                    when 8  =>
                        if c_w8 > 256 then z_fa <= resize(c_w8 - 256, 24); else z_fa <= (others => '0'); end if;
                        z_fb <= to_unsigned(140, 16);
                    when 9  => z_fa <= resize(c_ig12, 24); z_fb <= resize(s_ys, 16);
                    when 10 => z_fa <= z_t; z_fb <= to_unsigned(12603, 16);
                    when 11 => z_fa <= resize(z_x19, 24); z_fb <= c_stq;
                    when 12 => z_fa <= resize(z_v, 24); z_fb <= to_unsigned(169, 16);
                    when 14 => z_fa <= to_unsigned(6453, 24); z_fb <= z_mt;
                    when 15 => z_fa <= resize(c_kf, 24); z_fb <= resize(c_xa0m, 16);
                    when others => z_fa <= resize(c_kf, 24); z_fb <= to_unsigned(844, 16);
                end case;
            end if;
            if z_ld = '1' then
                f_a <= z_fa; f_b <= z_fb; f_acc <= (others => '0'); f_cnt <= to_unsigned(16, 5);
            end if;

            -- per-field values: zc 20 .. 691 (the line-0 kick is at 1300)
            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps), start points, simple maps
                    when 20 =>
                        c_th3 <= unsigned(signed(c_th3) + resize(c_w3, 19));
                        c_phb <= c_phb + c_phst;
                        z_seed <= c_fc(15 downto 1) & '1';
                        c_x0h <= signed(resize(c_xb0, 13)) - 1964;
                    when 21 =>
                        -- the field seed (held two fields): xor-shift-add mix of the field count
                        z_seed <= z_seed xor shift_right(z_seed, 7);
                        if s_mir = '1' then
                            if c_x0h < 0 then c_xa0 <= 844 - c_x0h; else c_xa0 <= 844 + c_x0h; end if;
                        else
                            c_xa0 <= c_x0h + 844;
                        end if;
                        z_x19 <= resize(1900 - c_xb0, 11);
                        -- orbit speed: v = k4 - 512 with a +-8 deadband
                        v_d := signed(resize(s_g4(12 downto 3), 13)) - 512;
                        if v_d > 8 then z_v <= unsigned(resize(v_d - 8, 10)); z_vn <= '0';
                        elsif v_d < -8 then z_v <= unsigned(resize(-8 - v_d, 10)); z_vn <= '1';
                        else z_v <= (others => '0'); z_vn <= '0'; end if;
                    when 22 =>
                        z_seed <= z_seed + shift_left(z_seed, 5);
                        c_xa0m <= unsigned(abs(c_xa0(11 downto 0)));
                        -- colour oscillator step (Q24 turns per h): k3 x 20 up to 512, then 10230 + (k3 - 512) x 100
                        if s_g3(12 downto 3) <= 512 then
                            c_stq <= shift_left(resize(s_g3(12 downto 3), 16), 4) + shift_left(resize(s_g3(12 downto 3), 16), 2);
                        else
                            v_u := resize(s_g3(12 downto 3) - 512, 16);
                            c_stq <= 10230 + shift_left(v_u, 6) + shift_left(v_u, 5) + shift_left(v_u, 2);
                        end if;
                        c_g6c <= s_g6(12 downto 8);
                        -- grain exponent: e8 = (k2 - 512) / 2 below the still, (k2 - 512) x 203 / 256 above; read 2^-e8
                        if s_g2(12 downto 3) <= 512 then
                            v_e := -shift_right(signed(resize(s_g2(12 downto 3), 12)) - 512, 1);
                        else
                            v_t := resize(s_g2(12 downto 3) - 512, 20);
                            v_t := shift_left(v_t, 7) + shift_left(v_t, 6) + shift_left(v_t, 3) + shift_left(v_t, 1) + v_t;
                            v_e := -signed(resize(v_t(19 downto 8), 12));
                        end if;
                        r_sq_az <= "01" & unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 23 =>
                        z_seed <= z_seed xor shift_right(z_seed, 9);
                        case to_integer(s_xs) is
                            when 4      => c_hst <= shift_left(resize(c_stq, 18), 2);
                            when 3      => c_hst <= shift_left(resize(c_stq, 18), 1) + resize(c_stq, 18);
                            when others => c_hst <= shift_left(resize(c_stq, 18), 1);
                        end case;
                    when 24 =>
                        z_seed <= z_seed + shift_left(z_seed, 3);
                        z_mt <= unsigned(r_sq_q);
                    when 25 =>                                         -- 1 / grain, Q12 = mt >> (3 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_ig12 <= resize(z_mt(15 downto 5), 14);
                            when -1     => c_ig12 <= resize(z_mt(15 downto 4), 14);
                            when 0      => c_ig12 <= resize(z_mt(15 downto 3), 14);
                            when others => c_ig12 <= resize(z_mt(15 downto 2), 14);
                        end case;
                        -- tube gain exponent: (k5 - 512) / 2 below, (k5 - 512) x 203 / 256 above; read 2^e
                        if s_g5(12 downto 3) <= 512 then
                            v_e := shift_right(signed(resize(s_g5(12 downto 3), 12)) - 512, 1);
                        else
                            v_t := resize(s_g5(12 downto 3) - 512, 20);
                            v_t := shift_left(v_t, 7) + shift_left(v_t, 6) + shift_left(v_t, 3) + shift_left(v_t, 1) + v_t;
                            v_e := signed(resize(v_t(19 downto 8), 12));
                        end if;
                        r_sq_az <= "01" & unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 26 =>
                        -- texture column step per pixel: xs x ig / 2 (Q16 cells; a cell is 16 virtual px at grain 1)
                        case to_integer(s_xs) is
                            when 4      => c_cst <= resize(shift_left(c_ig12, 1), 16);
                            when 3      => c_cst <= resize(c_ig12 + shift_right(c_ig12, 1), 16);
                            when others => c_cst <= resize(c_ig12, 16);
                        end case;
                    when 27 =>
                        z_mt <= unsigned(r_sq_q);

                    -- serial products: load at L, complete at L + 17, stored at L + 18
                    when 50 =>
                        if z_neg = '0' then c_kd8 <= 77 + resize(f_acc(19 downto 10), 9);
                        else c_kd8 <= 256 + resize(f_acc(19 downto 10), 9); end if;
                    when 70 =>
                        z_dv <= f_acc(16 downto 0);
                        z_rem <= to_unsigned(32768, 18); z_quo <= (others => '0');
                    when 71 to 79 =>                                   -- 256 / b = 2^24 / (427 kd8), 9 restoring steps
                        v_r := z_rem & '0';
                        if v_r >= resize(z_dv, 19) then
                            z_rem <= resize(v_r - resize(z_dv, 19), 18); z_quo <= z_quo(7 downto 0) & '1';
                        else
                            z_rem <= v_r(17 downto 0); z_quo <= z_quo(7 downto 0) & '0';
                        end if;
                    when 81 =>
                        c_ib <= z_quo;
                    when 100 =>
                        c_a2 <= f_acc(18 downto 8);
                    when 101 =>
                        z_k <= (others => '0');
                    -- ball positions (zc 112 .. 428), one coordinate per 40 clocks (slot index z_k): A kd, then
                    -- (A kd) |sin| (the amplitude fed straight from the accumulator); x = 982 + A cos, y = 538 - A sin
                    when 126 | 166 | 206 | 246 | 286 | 326 =>
                        case z_k(2 downto 1) is
                            when "00"   => v_u := c_th3(18 downto 3) + 22564;
                            when "01"   => v_u := c_th3(15 downto 0) + 38818;
                            when others => v_u := c_th3(15 downto 0);
                        end case;
                        if z_k(0) = '0' then r_sq_az <= "00" & (v_u(15 downto 8) + 64);
                        else r_sq_az <= "00" & v_u(15 downto 8); end if;
                    when 148 | 188 | 228 | 268 | 308 | 348 =>
                        if z_k(0) = '0' then v_b11 := to_unsigned(982, 11); else v_b11 := to_unsigned(538, 11); end if;
                        if (z_k(0) xor z_sn) = '1' then v_b11 := v_b11 - resize(f_acc(25 downto 15), 11);
                        else v_b11 := v_b11 + resize(f_acc(25 downto 15), 11); end if;
                        if z_k(0) = '0' then c_bx(to_integer(z_k(2 downto 1))) <= v_b11;
                        else c_by(to_integer(z_k(2 downto 1))) <= v_b11; end if;
                        z_k <= z_k + 1;
                    when 452 =>
                        r_sq_az <= "00" & (c_phb(15 downto 8) + 64);   -- cos of the breathing phase
                    when 470 =>
                        if z_neg = '0' then c_w8 <= resize(f_acc(17 downto 8), 10);
                        else c_w8 <= 256 + resize(f_acc(17 downto 8), 10); end if;
                    when 490 =>
                        if z_sn = '1' then z_mq <= -signed(resize(f_acc(25 downto 15), 12));
                        else z_mq <= signed(resize(f_acc(25 downto 15), 12)); end if;
                        z_mqm <= f_acc(25 downto 15);
                    when 510 =>
                        c_m16m <= f_acc(23 downto 8); c_m16s <= z_mq(11);
                    when 530 =>
                        c_phst <= 195 + resize(f_acc(17 downto 8), 16);
                    when 550 =>
                        z_t <= f_acc(23 downto 0);
                    when 570 =>
                        c_rst <= f_acc(37 downto 20);
                    when 590 =>
                        c_ph0 <= f_acc(23 downto 0);
                    when 591 =>
                        c_ph0 <= unsigned(-signed(c_ph0));                     -- the beam starts left of x 950
                    when 610 =>
                        if z_vn = '1' then c_w3 <= -signed(resize(f_acc(17 downto 8), 16));
                        else c_w3 <= signed(resize(f_acc(17 downto 8), 16)); end if;
                    when 650 =>
                        case to_integer(z_sh) is
                            when -1     => c_kf <= resize(f_acc(30 downto 16), 15);
                            when 0      => c_kf <= resize(f_acc(29 downto 15), 15);
                            when others => c_kf <= resize(f_acc(28 downto 14), 15);
                        end case;
                    when 670 =>
                        c_fk0 <= signed(resize(f_acc(26 downto 0), 28));
                    when 671 =>
                        if c_xa0 < 0 then c_fk0 <= -c_fk0; end if;
                    when 690 =>
                        c_fko <= f_acc(23 downto 8);
                    when 691 =>
                        case to_integer(s_xs) is
                            when 4      => c_fks <= shift_left(resize(c_kf, 18), 2);
                            when 3      => c_fks <= shift_left(resize(c_kf, 18), 1) + resize(c_kf, 18);
                            when others => c_fks <= shift_left(resize(c_kf, 18), 1);
                        end case;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
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
    -- write of (luma >> 2) - 16 at half horizontal resolution
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
            s_lb_wa <= s_wcol(10 downto 1);
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
    -- 0), computes the NEXT line: the disc chord, the three warp
    -- table row (sign-magnitude) with the breathing, the texture rows; the
    -- ball rows are read in hblank.  One serial product at a time.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_a, v_b : signed(11 downto 0);
        variable v_t  : signed(12 downto 0);
        variable v_p  : signed(19 downto 0);
        variable v_q  : signed(16 downto 0);
        variable v_dy : signed(11 downto 0);
        variable v_v  : unsigned(15 downto 0);
        variable v_i  : integer range 0 to 7;
        variable v_x  : unsigned(13 downto 0);
        variable v_m  : unsigned(24 downto 0);
        variable v_wr : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_r1 <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_r1 <= e_r1 + resize(c_rst, 24);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            -- serial multiplier: right-shifting accumulator
            if m_cnt /= 0 then                                -- 16 iterations, then hold the product
                if m_b(0) = '1' then v_m := ('0' & m_acc(39 downto 16)) + resize(m_a, 25);
                else v_m := '0' & m_acc(39 downto 16); end if;
                m_acc <= v_m & m_acc(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
                m_cnt <= m_cnt - 1;
            end if;

            -- the warp result: base +- (m x |D|) >> 8, mod 2^16 (one shared adder)
            if w_neg = '1' then v_wr := w_base - m_acc(23 downto 8); else v_wr := w_base + m_acc(23 downto 8); end if;

            -- multiplier loads: slot decoded one clock ahead (m_go / m_sel), operands muxed from the slot register
            m_go <= '0';
            case to_integer(e_lc) is
                when 2 | 19 | 36 | 53 | 70 | 87 => m_go <= '1';
                when others => null;
            end case;
            if e_lc = 2 then m_sel <= "000"; elsif e_lc = 19 then m_sel <= "001"; elsif e_lc = 36 then m_sel <= "010";
            elsif e_lc = 53 then m_sel <= "011"; elsif e_lc = 70 then m_sel <= "100"; elsif e_lc = 87 then m_sel <= "101"; end if;
            if m_go = '1' then
                case m_sel is
                    when "000"  => m_a <= resize(e_ay, 24); m_b <= resize(c_ib, 16);                       -- u = |Y| / b x 256
                    when "001"  => m_a <= resize(m_acc(16 downto 8), 24); m_b <= resize(m_acc(16 downto 8), 16);   -- u^2
                    when "010"  => m_a <= resize(e_d(0), 24); m_b <= c_m16m;                               -- warp, left
                    when "011"  => m_a <= resize(c_a2, 24); m_b <= e_sq;                                   -- disc chord
                    when "100"  => m_a <= resize(e_d(1), 24); m_b <= c_m16m;                               -- warp, centre
                    when others => m_a <= resize(e_d(2), 24); m_b <= c_m16m;                               -- warp, right
                end case;
                m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then e_vl <= to_unsigned(1079, 11); else e_vl <= e_vy(18 downto 8); end if;
                when 2 =>
                    if e_vl >= 538 then e_ay <= e_vl - 538; else e_ay <= 538 - e_vl; end if;
                    -- table row: vl x 15 / 32 (rows 0 .. 505 over 1080 lines); Flip reads it bottom-up (505 - row)
                    e_pos <= resize(shift_left(resize(e_vl, 15), 4) - resize(e_vl, 15), 19);
                when 3 =>
                    if s_flp = '1' then r_wt_a <= 505 - e_pos(13 downto 5); else r_wt_a <= e_pos(13 downto 5); end if;
                when 5 =>
                    e_d(0) <= "00000" & unsigned(r_wt_q(30 downto 25)); e_ds(0) <= r_wt_q(31);
                    e_d(1) <= "000" & unsigned(r_wt_q(23 downto 16));  e_ds(1) <= r_wt_q(24);
                    e_d(2) <= '0' & unsigned(r_wt_q(14 downto 6)) & '0'; e_ds(2) <= r_wt_q(15);
                    -- the right zone's steep rows (the pipe-end step) are ground whenever the warp is visibly on
                    if r_wt_q(5) = '1' and c_m16m > 645 then n_g2 <= '1'; else n_g2 <= '0'; end if;
                when 20 =>
                    e_u <= m_acc(18 downto 8);
                when 37 =>                                              -- (Y/b)^2: sqrt ROM address
                    r_sq_ae <= '1' & m_acc(15 downto 7);
                    w_base <= to_unsigned(35641, 16); w_neg <= e_ds(0) xor c_m16s;
                when 39 =>
                    e_sq <= unsigned(r_sq_q);
                when 54 =>
                    n_w0 <= v_wr;
                when 71 =>                                              -- disc half-width (h), 0 off the disc
                    if e_u(10 downto 8) /= 0 then n_hw <= (others => '0'); else n_hw <= m_acc(26 downto 15); end if;
                    w_neg <= e_ds(1) xor c_m16s;
                when 88 =>
                    n_w1 <= v_wr;
                    w_base <= to_unsigned(15124, 16); w_neg <= e_ds(2) xor c_m16s;
                when 105 =>
                    n_w2 <= v_wr;
                -- texture rows: seeds for rows j and j + 1, each mixed in 4 clocks
                when 160 =>
                    n_fy <= e_r1(15 downto 14);
                    e_hv <= (e_r1(23 downto 16) & not e_r1(23 downto 16)) xor z_seed;
                when 165 =>
                    e_hv <= ((e_r1(23 downto 16) + 1) & not (e_r1(23 downto 16) + 1)) xor z_seed;
                when 161 | 166 => e_hv <= e_hv xor shift_right(e_hv, 7);
                when 162 | 167 => e_hv <= e_hv + shift_left(e_hv, 5);
                when 163 | 168 => e_hv <= e_hv xor shift_right(e_hv, 9);
                when 164 => n_sa <= e_hv(15 downto 1) & '1';
                when 169 => n_sb <= e_hv(15 downto 1) & '1';
                when others => null;
            end case;

            -- ball rows for the next line, written straight into the beam's registers in hblank: 8 clocks after
            -- the generated avid falls (the pixel path is done with them), one row-table read per ball
            if s_gav_sr(0) = '1' and g_avid = '0' then
                y_cnt <= to_unsigned(1, 6);
            elsif y_cnt /= 0 and y_cnt /= 63 then
                y_cnt <= y_cnt + 1;
            end if;
            if y_cnt = 8 or y_cnt = 12 or y_cnt = 16 or y_cnt = 20 then         -- dy for ball e_bi
                v_i := to_integer(e_bi);
                if v_i = 3 then
                    e_dy <= signed(resize(e_vl, 12)) - 538;
                else
                    e_dy <= signed(resize(e_vl, 12)) - signed(resize(c_by(v_i), 12));
                end if;
            end if;
            if y_cnt = 9 or y_cnt = 13 or y_cnt = 17 or y_cnt = 21 then         -- |dy| -> the row table
                v_i := to_integer(e_bi);
                if abs(e_dy) < C_BRY(v_i) then
                    e_bv <= '1'; r_bl_a <= to_unsigned(C_BBASE(v_i), 8) + unsigned(resize(abs(e_dy), 8));
                else
                    e_bv <= '0';
                end if;
            end if;
            if y_cnt = 11 or y_cnt = 15 or y_cnt = 19 or y_cnt = 23 then
                v_i := to_integer(e_bi);
                if e_bv = '1' then
                    l_hw2(v_i) <= unsigned(r_bl_q(15 downto 8)) & '0';
                else
                    l_hw2(v_i) <= (others => '0');
                end if;
                e_bi <= e_bi + 1;
            end if;
            if y_cnt = 1 then e_bi <= (others => '0'); end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): x in h; the tube field (xa - 1120) KF as a DDA
    -- (reversing at the mirror axis); the colour oscillator phase; the texture
    -- cells (a new xorshift byte per cell crossing); the line's values latched
    -- at the line start; the camera read issued from the pixel index
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_c : unsigned(16 downto 0);
        variable v_l : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_px <= (others => '0');
                b_fk <= c_fk0;
                if s_mir = '1' and c_xb0 + s_xs <= 1964 then b_dn <= '1'; else b_dn <= '0'; end if;
                b_ph <= c_ph0;
                b_c1 <= (others => '0');
                b_la <= n_sa; b_a0 <= n_sa(7 downto 0); b_a1 <= n_sa(15 downto 8);
                b_lb <= n_sb; b_b0 <= n_sb(7 downto 0); b_b1 <= n_sb(15 downto 8);
                l_g2 <= n_g2;
                l_hw <= n_hw; l_w0 <= n_w0; l_w1 <= n_w1; l_w2 <= n_w2; l_fy <= n_fy;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
                if b_dn = '1' then b_fk <= b_fk - signed(resize(c_fks, 28));
                else b_fk <= b_fk + signed(resize(c_fks, 28)); end if;
                if s_mir = '1' and b_x + shift_left(resize(s_xs, 12), 1) <= 1964 then b_dn <= '1'; else b_dn <= '0'; end if;
                b_ph <= b_ph + resize(c_hst, 24);
                v_c := resize(b_c1, 17) + c_cst;
                b_c1 <= v_c(15 downto 0);
                if v_c(16) = '1' then
                    v_l := f_xs16(b_la); b_la <= v_l; b_a0 <= b_a1; b_a1 <= v_l(7 downto 0);
                    v_l := f_xs16(b_lb); b_lb <= v_l; b_b0 <= b_b1; b_b1 <= v_l(7 downto 0);
                end if;
            end if;
            s_lb_ra <= b_px(10 downto 1);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_x  : signed(12 downto 0);
        variable v_ax : unsigned(12 downto 0);
        variable v_f  : unsigned(27 downto 0);
        variable v_n  : unsigned(11 downto 0);
        variable v_t  : unsigned(7 downto 0);
        variable v_l  : signed(10 downto 0);
        variable v_yy, v_uu, v_vv : signed(11 downto 0);
        variable v_bs : unsigned(2 downto 0);
        variable v_i  : integer range 0 to 4;
        variable v_zu, v_zv : signed(9 downto 0);
        variable v_sa : signed(9 downto 0);
        variable v_wl : unsigned(9 downto 0);
        variable v_pp : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: |x - 1964| (the disc and Mirror), xa, |tube field|, colour oscillator address, texture differences
            v_x := signed(resize(b_x, 13)) - 1964;
            if v_x(12) = '1' then v_ax := unsigned(not v_x); else v_ax := unsigned(v_x); end if;
            p2_ax <= v_ax(11 downto 0);
            if s_mir = '1' then
                if v_ax(11 downto 0) < 676 then p2_z <= "01"; else p2_z <= "10"; end if;       -- xa = 1964 + |x - 1964|
            elsif b_x < 1120 then p2_z <= "00"; elsif b_x < 2640 then p2_z <= "01"; else p2_z <= "10"; end if;
            for i in 0 to 2 loop
                p2_dx(i) <= signed(resize(b_x, 13)) - signed(resize(c_bx(i) & '0', 13));
            end loop;
            if b_fk(27) = '1' then v_f := unsigned(not b_fk); else v_f := unsigned(b_fk); end if;
            p2_fk <= v_f(23 downto 8);
            r_cs_a <= b_ph(23 downto 16);
            p2_da <= signed(resize(b_a1, 9)) - signed(resize(b_a0, 9));
            p2_db <= signed(resize(b_b1, 9)) - signed(resize(b_b0, 9));
            p2_a0 <= b_a0; p2_b0 <= b_b0;
            p2_fx <= b_c1(15 downto 14);
            -- 3: the zone, inside the disc, FK, ball tests, texture products
            p3_z <= p2_z;
            if p2_z = "10" and l_g2 = '1' then p3_g <= '1'; else p3_g <= '0'; end if;
            if p2_ax < l_hw then p3_in <= '1'; else p3_in <= '0'; end if;
            p3_fk <= p2_fk + c_fko;
            for i in 0 to 2 loop
                if p2_dx(i) >= -signed(resize(l_hw2(i), 13)) and p2_dx(i) < signed(resize(l_hw2(i), 13)) then p3_bi(i) <= '1'; else p3_bi(i) <= '0'; end if;
            end loop;
            if p2_ax < resize(l_hw2(3), 12) then p3_bi(3) <= '1'; else p3_bi(3) <= '0'; end if;
            -- d x f for f = 0 .. 3 quarters: one shift-add each
            v_pp := (others => '0'); if p2_fx(1) = '1' then v_pp := shift_left(resize(p2_da, 11), 1); end if;
            if p2_fx(0) = '1' then v_pp := v_pp + resize(p2_da, 11); end if; p3_pa <= v_pp;
            v_pp := (others => '0'); if p2_fx(1) = '1' then v_pp := shift_left(resize(p2_db, 11), 1); end if;
            if p2_fx(0) = '1' then v_pp := v_pp + resize(p2_db, 11); end if; p3_pb <= v_pp;

            p3_a0 <= p2_a0; p3_b0 <= p2_b0;
            p3_lm <= unsigned(s_lb_q);
            -- 4: the zone's warp; |dx|; texture lerps; the disc colour (EBR); the camera product
            case p3_z is
                when "00"   => p4_w <= l_w0;
                when "01"   => p4_w <= l_w1;
                when others => p4_w <= l_w2;
            end case;
            p4_fk <= p3_fk;
            p4_bi <= p3_bi;
            v_sa := signed(resize(p3_a0, 10)) + resize(shift_right(p3_pa, 2), 10); p4_na <= unsigned(v_sa(7 downto 0));
            v_sa := signed(resize(p3_b0, 10)) + resize(shift_right(p3_pb, 2), 10); p4_nb <= unsigned(v_sa(7 downto 0));
            p4_cb <= unsigned(r_cs_q(9 downto 0));
            -- the camera's luma (0 .. 239), or the Osc lattice made zero-mean and fed at 3x the camera's level
            if s_src = '1' then v_l := signed(resize(p3_lm, 11)) - 112; v_l := shift_left(v_l, 1) + v_l; else v_l := signed(resize(p3_lm, 11)); end if;
            p4_cam <= shift_left(resize(v_l * signed('0' & c_g6c), 17), 2);
            p4_z <= p3_z; p4_in <= p3_in; p4_g <= p3_g;
            -- 5: FK - W; ball inside; the noise difference; the camera terms
            p5_s1 <= p4_fk - p4_w;
            p5_bi <= p4_bi;
            p5_nd <= signed(resize(p4_nb, 9)) - signed(resize(p4_na, 9));
            p5_na <= p4_na;
            p5_ct <= unsigned(p4_cam(15 downto 0));
            p5_cy <= resize(shift_right(p4_cam, 5), 11);
            p5_z <= p4_z; p5_in <= p4_in; p5_cb <= p4_cb; p5_g <= p4_g;
            -- 6: S = FK - W - camera + 400 px (Q16 per 1300-px period); ball priority (hole, B3, B2, B4, B1);
            --    cb, cy, the zone and the flags go into the delay rings here and come back at stage 11
            p6_s <= p5_s1 - p5_ct + 20165;
            if p5_bi(3) = '1' then v_bs := "100";
            elsif p5_bi(2) = '1' then v_bs := "011";
            elsif p5_bi(1) = '1' then v_bs := "010";
            elsif p5_bi(0) = '1' then v_bs := "001";
            else v_bs := "000"; end if;
            p6_bs <= v_bs;
            v_pp := (others => '0'); if l_fy(1) = '1' then v_pp := shift_left(resize(p5_nd, 11), 1); end if;
            if l_fy(0) = '1' then v_pp := v_pp + resize(p5_nd, 11); end if; p6_pn <= v_pp;
            p6_na <= p5_na;
            p6_z <= p5_z; p6_in <= p5_in; p6_cb <= p5_cb; p6_cy <= p5_cy; p6_g <= p5_g;
            -- 7: profile ROM address; octave-1 noise
            if p6_g = '1' then r_pf_a <= s_neon & x"98";               -- ground (s = 374 px: black)
            else r_pf_a <= s_neon & p6_s(15 downto 8); end if;
            v_sa := signed(resize(p6_na, 10)) + resize(shift_right(p6_pn, 2), 10); p7_n1 <= unsigned(v_sa(7 downto 0));
            -- 8: profile (EBR); the noise
            p8_n <= p7_n1;
            -- 9: profile fabric copy; luma curve address
            p9_ty <= unsigned(r_pf_q(29 downto 20)); p9_tu <= unsigned(r_pf_q(19 downto 10)); p9_tv <= unsigned(r_pf_q(9 downto 0));
            r_lm_a <= p8_n;
            -- 10: the zone colour weight (0 .. 3 quarters) from the tube's luma; luma curve (EBR)
            if p9_ty > 84 then v_wl := p9_ty - 84; else v_wl := (others => '0'); end if;
            if v_wl(9 downto 7) > 2 then p10_wl <= "11"; else p10_wl <= v_wl(8 downto 7); end if;
            p10_ty <= p9_ty; p10_tu <= p9_tu; p10_tv <= p9_tv;
            -- 11: luma curve fabric copy; the delay-ring copies (stage-11 values of cb, ball, zone, flags, camera)
            p11_dy <= unsigned(r_lm_q(9 downto 0));
            p11_ty <= p10_ty; p11_tu <= p10_tu; p11_tv <= p10_tv; p11_wl <= p10_wl;
            q11_cb <= unsigned(dra_q(15 downto 6)); q11_bs <= unsigned(dra_q(5 downto 3));
            q11_z <= unsigned(dra_q(2 downto 1)); q11_in <= dra_q(0);
            q11_cy <= signed(drb_q(10 downto 0));
            -- 12: tube colour (+ the centre zone chroma offset x weight); disc colour (+ camera)
            if s_neon = '0' then
                if q11_z = "01" then v_zu := to_signed(125, 10); v_zv := to_signed(100, 10);
                else v_zu := to_signed(0, 10); v_zv := to_signed(0, 10); end if;
            else
                if q11_z = "01" then v_zu := to_signed(-125, 10); v_zv := to_signed(175, 10);
                else v_zu := to_signed(0, 10); v_zv := to_signed(0, 10); end if;
            end if;
            v_uu := signed(resize(p11_tu, 12)); v_vv := signed(resize(p11_tv, 12));
            if p11_wl(1) = '1' then v_uu := v_uu + resize(shift_right(v_zu, 1), 12); v_vv := v_vv + resize(shift_right(v_zv, 1), 12); end if;
            if p11_wl(0) = '1' then v_uu := v_uu + resize(shift_right(v_zu, 2), 12); v_vv := v_vv + resize(shift_right(v_zv, 2), 12); end if;
            p12_tu <= unsigned(v_uu(9 downto 0)); p12_tv <= unsigned(v_vv(9 downto 0));
            p12_ty <= p11_ty;
            v_yy := signed(resize(p11_dy, 12)) + resize(q11_cy, 12);
            if v_yy < 72 then p12_dy <= to_unsigned(72, 10); elsif v_yy > 930 then p12_dy <= to_unsigned(930, 10); else p12_dy <= unsigned(v_yy(9 downto 0)); end if;
            if s_neon = '0' then
                p12_du <= to_unsigned(680, 10);
                p12_dv <= q11_cb;
            else
                p12_du <= to_unsigned(344, 10);
                p12_dv <= q11_cb + 101;
            end if;
            p12_bs <= q11_bs; p12_in <= q11_in;
            -- 13: compose: a ball (inside the true disc), the disc (inverted by Window) or the tubes
            if p12_bs /= "000" and p12_in = '1' then
                v_i := to_integer(p12_bs) - 1;
                p13_y <= to_unsigned(C_BYC(v_i), 10);
                p13_u <= to_unsigned(C_BU(v_i), 10); p13_v <= to_unsigned(C_BV(v_i), 10);
            elsif (p12_in xor s_win) = '1' then
                p13_y <= p12_dy; p13_u <= p12_du; p13_v <= p12_dv;
            else
                p13_y <= p12_ty; p13_u <= p12_tu; p13_v <= p12_tv;
            end if;
            -- 14: spare register; 15: black outside the generated avid
            p14_y <= p13_y; p14_u <= p13_u; p14_v <= p13_v;
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= p14_y; s_u15 <= p14_u; s_v15 <= p14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
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
            sy_ra <= rg_wa - 17;
            dr_ra <= rg_wa - 2;
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y15 - s_cay;
                s_bau <= s_bau + s_u15 - s_cau;
                s_bav <= s_bav + s_v15 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y17 - s_cby;
                s_bbu <= s_bbu + s_u17 - s_cbu;
                s_bbv <= s_bbv + s_v17 - s_cbv;
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
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
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture platter;
