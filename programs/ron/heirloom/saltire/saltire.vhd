-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Saltire v0.1.0 -- Heirloom (video phase), rebuilt from the clip WIN_20230620_00_50_54 at 0.5 s.
--
-- The still: a neon X.  The picture is mirrored about virtual line 517 (v = |y - 517|).  One magenta band per
-- half runs from the left edge (v ~ 266..370) down to a vertex at x ~ 1530 where the two halves nearly touch,
-- then climbs as a yellow arm to the corner: the saltire.  Scallop teeth (the stripe oscillator's 3rd harmonic,
-- rectified) bite the band's inner edge and grow into flames at the vertex, where the band ends in a
-- right-pointing arrowhead of tongues.  A second diagonal arm (yellow | red | magenta) comes down from the top
-- edge; left of it a blue field with soft black blobs (red patches where they cross the band); behind it all,
-- green / teal vertical stripes from a ramp oscillator hard-synced to the line and FM'd by a vertical sine,
-- scrolling down.  Magenta's trailing edges carry a white neon fringe.
--
-- The patch (saltire.py has every table, imodel.py the exact integer arithmetic):
--   ph     = (x (1 - b sin(2 pi (y - yw) / 224)) - 13) * kf / 252       stripe oscillator, Q24 cycles
--   stripes= STRIPE[k = int ph][q = frac ph]                               per-stripe profile (green / teal)
--   tooth  = scallop(3 ph) (Blade: saw(ph), x2), fanned near the vertex by |v - 160|
--   band   = inner max(PIN_L(x), 0.99 (x - 1485)) + A tooth, outer max(POUT_L(x), 1.164 (x - 1393)) (+ flames)
--   blobs  = cos(x) + cos(v) + 0.9 cos(x + 1.2 v) + corner + camera luma > T / m   (quarter steps)
--   colour = 512-entry palette ROM (two gun banks), then the neon fringe (12-px look-ahead)
--
--   K1  Stripes   the stripe (and teeth) oscillator frequency, x1/3 .. x3 (512 = the still)
--   K2  Wave      FM depth of the vertical sine: straight .. the still (512) .. deep zigzags
--   K3  Cross     slides the X (band, arm, blobs) left / right, +-704 px
--   K4  Speed     bipolar: the wave (stripes, teeth, tongues) scrolls down / up; 680 = the clip's 112 lines/s
--   K5  Spread    vertical scale of the X: arms hugging the centre .. the still .. arms out of the frame
--   K6  Blobs     the blob holes and how far the camera (or Osc) punches and bends: none .. the still .. heavy
--   S7  Fold      Mirror (the still) / Quad (two X's stacked)
--   S8  Teeth     Scallop (the still) / Blade (one big saw per stripe, double depth)
--   S9  Twin      one X / two X's facing each other (x folded about the centre)
--   S10 Guns      RGB (the still) / BRG (the guns rotated: yellow band, cyan arms, red field, blue stripes)
--   S11 Source    Video / Osc
--   P12 FLAMES    teeth and tongue depth: smooth bands (0) .. the still (300) .. a shredded inferno (1023)
--
-- Engine: vblank-zone sequencer (one serial 18 x 16 multiplier, 20 slots, shared glide, exp2 ROM); a per-line
-- engine (17 micro-ops dest = base +- (src * K >> sh) on one serial 36 x 17 multiplier) computing the next
-- line, fed by ROM reads in the hblank before; a 15-stage decision path, the fringe look-ahead through a
-- block-RAM ring, the palette ROM.  Virtual 1080-line coordinates, the X spanning the width in every mode.
-- Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps
-- come from block-RAM rings, with the running-sum resync, luma 13/16) before the blanking gate.  Colours in the
-- hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.saltire_pkg.all;
use work.all;

architecture saltire of program_top is
    constant C_LAT    : integer := 40;        -- = processing_delay_clks
    constant C_LEAD   : integer := 4;         -- beam lead (pixels): the output boxes' lag behind the gate window
    constant C_TPH    : integer := -87032;    -- teeth phase offset, Q20 (-0.083 cycle)
    constant C_THETA0 : integer := 43875;     -- wave phase at line 0 of field 0: (0 - 522) / 224 mod 1, Q16

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
    -- x step class: 1080 = 2, 720p = 3, SD = 4
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
    function f_lfsr(r : unsigned(15 downto 0)) return unsigned is
        variable b : std_logic;
    begin
        b := r(0) xor r(2) xor r(3) xor r(5);
        return b & r(15 downto 1);
    end function;
    -- exp2 mantissa (Q15) >> (7 - e) for e = -2 .. 1 (e as its 3-bit two's complement)
    function f_pow(q : unsigned(15 downto 0); e : std_logic_vector(2 downto 0)) return unsigned is
    begin
        case e is
            when "001"  => return resize(shift_right(q, 6), 11);
            when "000"  => return resize(shift_right(q, 7), 11);
            when "111"  => return resize(shift_right(q, 8), 11);
            when others => return resize(shift_right(q, 9), 11);
        end case;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- output-filter 8-back taps: six 256 x 16 block-RAM rings (Y, U, V of box A and box B); the fringe ring
    type t_ram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal or_ya, or_ua, or_va, or_yb, or_ub, or_vb, fr_ram : t_ram := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of or_ya : signal is true;
    attribute no_rw_check of or_ua : signal is true;
    attribute no_rw_check of or_va : signal is true;
    attribute no_rw_check of or_yb : signal is true;
    attribute no_rw_check of or_ub : signal is true;
    attribute no_rw_check of or_vb : signal is true;
    attribute no_rw_check of fr_ram : signal is true;
    attribute no_rw_check of lb : signal is true;     -- written 4 addresses ahead of the read (s_lb_wa)
    signal o_wp, o_rp : unsigned(7 downto 0) := (others => '0');
    signal o_qya, o_qua, o_qva, o_qyb, o_qub, o_qvb : std_logic_vector(15 downto 0) := (others => '0');
    signal o_fya, o_fua, o_fva, o_fyb, o_fub, o_fvb : unsigned(9 downto 0) := (others => '0');
    signal fr_rp : unsigned(7 downto 0) := (others => '0');
    signal fr_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal s_rg  : std_logic_vector(3 downto 0) := "0111";              -- the ring's sync taps (fabric copy)

    -- ROMs (registered address and data)
    signal r_xa  : unsigned(8 downto 0) := (others => '0');
    signal r_xq  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_sa  : unsigned(8 downto 0) := (others => '0');
    signal r_sq  : std_logic_vector(7 downto 0) := (others => '0');
    signal r_ta  : unsigned(8 downto 0) := (others => '0');
    signal r_tq  : signed(7 downto 0) := (others => '0');
    signal r_ca, r_cb : unsigned(7 downto 0) := (others => '0');
    signal r_cqa, r_cqb : signed(11 downto 0) := (others => '0');
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_pa  : unsigned(8 downto 0) := (others => '0');
    signal r_pq  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_rsel : std_logic := '0';                         -- the line engine owns COS A / TOOTH this clock
    signal e_caddr : unsigned(7 downto 0) := (others => '0');
    signal e_taddr : unsigned(8 downto 0) := (others => '0');

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
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_quad, s_saw, s_twin, s_guns, s_src : std_logic := '0';

    -- per-field constants
    signal c_step  : unsigned(5 downto 0) := to_unsigned(16, 6);     -- virtual x per pixel, Q4
    signal c_thst  : unsigned(15 downto 0) := to_unsigned(293, 16);  -- wave phase per line, Q16
    signal c_c1080 : unsigned(17 downto 0) := to_unsigned(66576, 18);
    signal c_cinc  : unsigned(19 downto 0) := to_unsigned(66576, 20); -- stripe phase per pixel, Q24
    signal c_bq    : unsigned(13 downto 0) := to_unsigned(2896, 14);  -- FM depth, Q16
    signal c_gq    : unsigned(10 downto 0) := to_unsigned(256, 11);   -- spread gain, Q8
    signal c_sh    : signed(16 downto 0) := (others => '0');          -- cross shift, Q4 px
    signal c_ftw   : signed(16 downto 0) := to_signed(30720, 17);     -- twin fold: 30720 - shift
    signal c_dx7   : unsigned(12 downto 0) := to_unsigned(1872, 13);  -- step * 117
    signal c_x7s   : signed(23 downto 0) := to_signed(7488, 24);      -- (C_LEAD step - shift) * 117
    signal z_si    : std_logic_vector(2 downto 0) := "000";
    signal c_spd   : unsigned(15 downto 0) := (others => '0');
    signal c_th0   : unsigned(15 downto 0) := to_unsigned(C_THETA0, 16);
    signal c_a0f   : unsigned(8 downto 0) := to_unsigned(50, 9);
    signal c_daf   : unsigned(8 downto 0) := to_unsigned(60, 9);
    signal c_at    : unsigned(8 downto 0) := to_unsigned(109, 9);
    signal c_t1, c_t2, c_t3, c_t4 : signed(11 downto 0) := to_signed(1023, 12);
    signal c_boff  : std_logic := '0';
    signal c_ke    : unsigned(1 downto 0) := "10";                   -- etch: luma >> ke
    signal z_e1, z_e5, z_e6 : signed(10 downto 0) := (others => '0');
    signal z_im    : unsigned(10 downto 0) := (others => '0');
    signal z_k2sq  : unsigned(19 downto 0) := (others => '0');
    signal z_s4sq  : unsigned(17 downto 0) := (others => '0');

    -- field serial multiplier (18 x 16)
    signal f_a   : unsigned(33 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(33 downto 0) := (others => '0');
    signal z_fa  : unsigned(17 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_fa1, z_fa2 : unsigned(17 downto 0) := (others => '0');
    signal z_fb1, z_fb2 : unsigned(15 downto 0) := (others => '0');
    signal z_hi, z_pre2 : std_logic := '0';
    signal z_exq : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_ctr : unsigned(4 downto 0) := (others => '0');
    signal z_run : std_logic := '0';
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
    signal e_kick : std_logic := '0';
    signal e_lc   : unsigned(2 downto 0) := to_unsigned(7, 3);       -- prep clocks 0..3 after a kick
    signal e_run  : std_logic := '0';
    signal e_op   : unsigned(4 downto 0) := (others => '0');
    signal e_pc   : unsigned(4 downto 0) := (others => '0');
    signal e_vy   : unsigned(19 downto 0) := (others => '0');      -- Q8 virtual line of the next vp product
    signal e_th   : unsigned(15 downto 0) := (others => '0');      -- wave phase of the next hblank read
    signal e_vp1, e_vp2, e_vp : unsigned(10 downto 0) := (others => '0');
    signal e_s1   : signed(11 downto 0) := (others => '0');
    signal e_sc1  : signed(7 downto 0) := (others => '0');
    signal e_cv1  : signed(7 downto 0) := (others => '0');
    signal e_hc   : unsigned(4 downto 0) := (others => '0');       -- hblank / zone read sequencer
    signal e_hgo  : std_logic := '0';
    signal e_sneg, e_cneg : std_logic := '0';
    signal e_sa   : unsigned(10 downto 0) := (others => '0');
    signal e_ca   : unsigned(6 downto 0) := (others => '0');
    signal e_bsa  : unsigned(13 downto 0) := (others => '0');
    signal e_y1, e_yq : signed(11 downto 0) := (others => '0');
    signal e_a0   : signed(11 downto 0) := (others => '0');
    signal e_av   : unsigned(10 downto 0) := (others => '0');
    signal e_v2   : unsigned(9 downto 0) := (others => '0');
    signal e_tip  : signed(12 downto 0) := (others => '0');
    signal e_cvph, e_tqp : unsigned(15 downto 0) := (others => '0');
    signal e_lf   : unsigned(15 downto 0) := x"ACE1";
    signal e_n6   : unsigned(2 downto 0) := (others => '0');
    signal e_n2   : std_logic := '0';
    signal e_d1w  : std_logic := '0';
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal e_wp   : unsigned(27 downto 0) := (others => '0');
    signal e_wng  : std_logic := '0';
    signal m_b    : unsigned(16 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    -- next-line values (n_) and the current line's (l_)
    signal n_vp, l_vp : unsigned(10 downto 0) := (others => '0');
    signal n_av3, l_av3 : unsigned(11 downto 0) := (others => '0');
    signal n_xtip, l_xtip : signed(12 downto 0) := (others => '0');
    signal n_xm, l_xm : signed(11 downto 0) := (others => '0');
    signal n_d0, l_d0, n_d1, l_d1 : signed(11 downto 0) := (others => '0');
    signal n_cv, l_cv : signed(7 downto 0) := (others => '0');
    signal n_dl, l_dl : unsigned(15 downto 0) := (others => '0');
    signal n_c0, l_c0 : signed(12 downto 0) := (others => '0');
    signal n_inc, l_inc : unsigned(16 downto 0) := (others => '0');   -- stripe phase per pixel, Q20
    signal n_tinc, l_tinc : unsigned(17 downto 0) := (others => '0');
    signal n_phs  : signed(25 downto 0) := (others => '0');
    signal n_tst  : unsigned(19 downto 0) := (others => '0');
    signal e_vw   : unsigned(16 downto 0) := (others => '0');
    signal n_spl, l_spl : signed(11 downto 0) := to_signed(1530, 12);
    signal n_lt, l_lt : signed(3 downto 0) := (others => '0');
    signal n_jit, l_jit : signed(2 downto 0) := (others => '0');
    signal n_v15, l_v15 : std_logic := '0';

    -- beam
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_xv  : unsigned(15 downto 0) := (others => '0');
    signal b_ph  : signed(25 downto 0) := (others => '0');          -- stripe phase, Q20 cycles
    signal b_tq  : unsigned(19 downto 0) := (others => '0');        -- teeth phase, Q20 mod 1
    signal b_x7  : signed(23 downto 0) := (others => '0');         -- (x - shift) * 117, Q4: the blob x phase

    -- pixel path (pN_ = stage N registers)
    signal p2_xg : signed(16 downto 0) := (others => '0');
    signal p2_kneg, p2_kz : std_logic := '0';
    signal p2_q  : unsigned(5 downto 0) := (others => '0');
    signal p2_qt, p3_qt, p4_qt, p5_qt, p6_qt : unsigned(7 downto 0) := (others => '0');
    signal p2_x117, p3_x117 : unsigned(15 downto 0) := (others => '0');
    signal p3_ax : unsigned(16 downto 0) := (others => '0');
    signal p3_xpx : signed(12 downto 0) := (others => '0');
    signal p3_son, p4_son, p5_son, p3_kp, p4_kp : std_logic := '0';
    signal p3_dg : std_logic := '0';
    signal p4_lu, p5_lu, p6_lu, p7_lu, p8_lu, p9_lu : unsigned(7 downto 0) := (others => '0');
    signal p4_x117 : unsigned(15 downto 0) := (others => '0');
    signal p4_cx : unsigned(13 downto 0) := (others => '0');
    signal p5_etv : unsigned(7 downto 0) := (others => '0');
    signal p4_sr : std_logic_vector(7 downto 0) := (others => '0');
    signal p5_xr : std_logic_vector(31 downto 0) := (others => '0');
    signal p5_cor, p6_cor, p7_cor, p8_cor : unsigned(8 downto 0) := (others => '0');
    signal p6_vv : signed(12 downto 0) := (others => '0');
    signal p5_lvl : signed(7 downto 0) := (others => '0');
    signal p5_teal : std_logic := '0';
    type t_flag is array (4 to 15) of std_logic;
    signal pspl, ptip, pdw : t_flag := (others => '0');       -- per stage: x >= split, x < tip, drip chevron
    signal p4_dx : signed(13 downto 0) := (others => '0');
    type t_ex is array (5 to 12) of signed(9 downto 0);
    signal pex : t_ex := (others => (others => '0'));          -- per stage: x - xM, saturated
    type t_gi is array (6 to 15) of unsigned(7 downto 0);
    signal pgi : t_gi := (others => (others => '0'));          -- per stage: the ground (stripe) index
    type t_n3 is array (6 to 11) of unsigned(2 downto 0);
    signal pnear : t_n3 := (others => (others => '0'));
    signal p6_pin, p6_pout : signed(13 downto 0) := (others => '0');
    signal p6_g  : unsigned(14 downto 0) := (others => '0');
    signal p6_da, p6_da4 : unsigned(10 downto 0) := (others => '0');
    signal p6_pl : signed(10 downto 0) := (others => '0');
    signal p6_lk0 : std_logic := '0';
    signal p7_amp, p8_amp, p9_amp : unsigned(8 downto 0) := (others => '0');
    type t_dp is array (7 to 13) of signed(9 downto 0);
    signal pdpi, pdpo : t_dp := (others => (others => '0'));
    signal p7_lk : std_logic := '0';
    signal p7_cx, p7_cd : signed(7 downto 0) := (others => '0');
    signal p8_b1 : signed(10 downto 0) := (others => '0');
    signal p9_b2 : signed(11 downto 0) := (others => '0');
    signal p9_tooth : signed(7 downto 0) := (others => '0');
    signal p10_ml : unsigned(12 downto 0) := (others => '0');
    signal p10_mh : signed(13 downto 0) := (others => '0');
    signal p10_blob : signed(11 downto 0) := (others => '0');
    signal p11_toff : signed(10 downto 0) := (others => '0');
    signal p11_h, p12_h, p13_h, p14_h : unsigned(2 downto 0) := (others => '0');
    signal p12_inw, p13_inw, p14_inw : std_logic := '0';
    signal p12_tn : signed(13 downto 0) := (others => '0');
    signal p12_tm : signed(10 downto 0) := (others => '0');
    signal p13_vo : signed(13 downto 0) := (others => '0');
    signal p13_exm : signed(10 downto 0) := (others => '0');
    signal p14_dvo : signed(11 downto 0) := (others => '0');
    signal p14_mz, p14_my, p14_mr, p14_bz : std_logic := '0';
    signal p15_ina, p15_vm, p15_bv, p15_mz, p15_my, p15_mr, p15_bz, p15_inw : std_logic := '0';
    signal p15_h : unsigned(2 downto 0) := (others => '0');
    signal p16_idx : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal p16_dark : std_logic := '1';
    signal fd_dk  : std_logic_vector(12 downto 0) := (others => '1');
    signal p30_fr : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal p31_idx : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal p34_y, p34_u, p34_v : unsigned(9 downto 0) := (others => '0');
    signal s_y35 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u35, s_v35 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y37, s_u37, s_v37 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y39, s_u39, s_v39 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y40, s_u40, s_v40 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y41 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u41, s_v41 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), output-filter
    -- rings, the fringe ring, ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_oring : process(clk)
    begin
        if rising_edge(clk) then
            o_qya <= or_ya(to_integer(o_rp)); or_ya(to_integer(o_wp)) <= std_logic_vector(resize(s_y35, 16));
            o_qua <= or_ua(to_integer(o_rp)); or_ua(to_integer(o_wp)) <= std_logic_vector(resize(s_u35, 16));
            o_qva <= or_va(to_integer(o_rp)); or_va(to_integer(o_wp)) <= std_logic_vector(resize(s_v35, 16));
            o_qyb <= or_yb(to_integer(o_rp)); or_yb(to_integer(o_wp)) <= std_logic_vector(resize(s_y37, 16));
            o_qub <= or_ub(to_integer(o_rp)); or_ub(to_integer(o_wp)) <= std_logic_vector(resize(s_u37, 16));
            o_qvb <= or_vb(to_integer(o_rp)); or_vb(to_integer(o_wp)) <= std_logic_vector(resize(s_v37, 16));
        end if;
    end process p_oring;

    -- the fringe look-ahead: the stage-16 index (and the sync / avid delay lines' taps 15) is written at the
    -- free-running pointer and read 12 behind
    -- (data a clock later, then a fabric register): the index re-emerges as the stage-30 value and the delay
    -- lines' copies (s_rg) as their taps 29
    p_fring : process(clk)
    begin
        if rising_edge(clk) then
            fr_q <= fr_ram(to_integer(fr_rp));
            fr_ram(to_integer(o_wp)) <= "000" & s_gav_sr(15) & s_field_sr(15) & s_vsync_sr(15) & s_hsync_sr(15)
                                        & '0' & std_logic_vector(p16_idx);
        end if;
    end process p_fring;

    -- x ROM (band edges, flame zone, drips): address from stage 2
    p_xrom : process(clk)
        variable v_a : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            v_a := shift_right(p2_xg, 6);
            if v_a < 0 then r_xa <= (others => '0');
            elsif v_a > 511 then r_xa <= (others => '1');
            else r_xa <= unsigned(v_a(8 downto 0)); end if;
            r_xq <= C_SL_XROM(to_integer(r_xa));
        end if;
    end process p_xrom;

    -- stripe profile ROM: address from the beam (stage 1): stripe index (clamped 0..15) and phase
    p_srom : process(clk)
    begin
        if rising_edge(clk) then
            if b_ph(25) = '1' then r_sa <= "0000" & unsigned(b_ph(18 downto 14));
            elsif b_ph(24) = '1' then r_sa <= "1111" & unsigned(b_ph(18 downto 14));
            else r_sa <= unsigned(b_ph(23 downto 20)) & unsigned(b_ph(18 downto 14)); end if;
            r_sq <= C_SL_STR(to_integer(r_sa));
        end if;
    end process p_srom;

    -- tooth ROM: the pixel path (stage 6: the fanned teeth phase), the line engine in hblank (the tongue)
    p_trom : process(clk)
    begin
        if rising_edge(clk) then
            if e_rsel = '1' then r_ta <= e_taddr;
            else r_ta <= s_saw & (p6_qt - p6_g(11 downto 4)); end if;
            r_tq <= C_SL_TOOTH(to_integer(r_ta));
        end if;
    end process p_trom;

    -- cosine ROM A: the pixel path (blob x term, from stage 4) and the line engine in hblank; ROM B: the
    -- diagonal term
    p_crom : process(clk)
    begin
        if rising_edge(clk) then
            if e_rsel = '1' then r_ca <= e_caddr;
            else r_ca <= resize(shift_right(p4_x117 - 29250, 8), 8); end if;
            r_cqa <= C_SL_COS(to_integer(r_ca));
            r_cb <= resize(shift_right(p4_x117 + l_dl, 8), 8);
            r_cqb <= C_SL_COS(to_integer(r_cb));
        end if;
    end process p_crom;

    p_erom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_SL_EXP2(to_integer(r_ex_a));
        end if;
    end process p_erom;

    p_prom : process(clk)
    begin
        if rising_edge(clk) then
            r_pa <= s_guns & p31_idx;
            r_pq <= C_SL_PAL(to_integer(r_pa));
        end if;
    end process p_prom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s   : signed(10 downto 0);
        variable v_m   : unsigned(9 downto 0);
        variable v_e   : signed(10 downto 0);
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
            elsif s_zc /= 4095 then
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
                s_quad <= registers_in(6)(0);
                s_saw  <= registers_in(6)(1);
                s_twin <= registers_in(6)(2);
                s_guns <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_g6   <= unsigned(registers_in(5)) & "000";        -- K6 Blobs: per field, no glide
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); c_step <= to_unsigned(43, 6); c_dx7 <= to_unsigned(5031, 13);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_step <= to_unsigned(24, 6); c_dx7 <= to_unsigned(2808, 13);
                    when others => s_oxs <= to_unsigned(102, 9); c_step <= to_unsigned(16, 6); c_dx7 <= to_unsigned(1872, 13);
                end case;
                -- the wave scrolls by the last field's speed
                if s_vneg = '1' then c_th0 <= c_th0 + c_spd; else c_th0 <= c_th0 - c_spd; end if;
            end if;
            g_wm1 <= s_w - 1;

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

            -- serial products on one 18 x 16 multiplier: slot n has z_ctr = 0 at zc = 40 + 24 n; operands
            -- pre-muxed in two halves at z_ctr = 1, selected at 2, loaded at 3, 16 iterations, the result stored
            -- at z_ctr = 20.  A slot's exp2 address is set at its store; its data (a fabric copy, z_exq) is read
            -- a slot later or more.
            if s_zone = '1' and s_zc = 39 then
                z_run <= '1'; z_ctr <= (others => '0'); z_sel <= (others => '0');
            elsif z_run = '1' then
                if z_ctr = 23 then
                    z_ctr <= (others => '0');
                    z_sel <= z_sel + 1;
                    if z_sel = 22 then z_run <= '0'; end if;
                else
                    z_ctr <= z_ctr + 1;
                end if;
            end if;
            if z_run = '1' and z_ctr = 0 then z_pre <= '1'; else z_pre <= '0'; end if;
            z_exq <= r_ex_q;
            if z_pre = '1' then
                if z_sel >= 11 then z_hi <= '1'; else z_hi <= '0'; end if;
                case to_integer(z_sel) is
                    when 0  => z_fa1 <= resize(s_g1(12 downto 3), 18);   z_fb1 <= to_unsigned(203, 16);    -- e1 (stripes)
                    when 1  =>                                                                              -- C1080
                        case std_logic_vector(z_e1(10 downto 8)) is
                            when "001"  => z_fa1 <= to_unsigned(66576, 18);
                            when "000"  => z_fa1 <= to_unsigned(33288, 18);
                            when "111"  => z_fa1 <= to_unsigned(16644, 18);
                            when others => z_fa1 <= to_unsigned(8322, 18);
                        end case;
                        z_fb1 <= z_exq;
                    when 2  => z_fa1 <= c_c1080;                          z_fb1 <= resize(c_step, 16);      -- C per pixel
                    when 4  => z_fa1 <= resize(s_g2(12 downto 3), 18);   z_fb1 <= resize(s_g2(12 downto 3), 16);   -- k2^2
                    when 5  => z_fa1 <= resize(shift_right(z_k2sq, 2), 18); z_fb1 <= to_unsigned(724, 16);  -- FM depth
                    when 6  => z_fa1 <= resize(s_g5(12 downto 3), 18);   z_fb1 <= to_unsigned(147, 16);    -- e5 (spread)
                    when 7  =>                                                                              -- gq = mant << (e5 >> 8) + 2 >> 9
                        case std_logic_vector(z_e5(10 downto 8)) is
                            when "001"  => z_fa1 <= to_unsigned(8, 18);
                            when "000"  => z_fa1 <= to_unsigned(4, 18);
                            when "111"  => z_fa1 <= to_unsigned(2, 18);
                            when others => z_fa1 <= to_unsigned(1, 18);
                        end case;
                        z_fb1 <= z_exq;
                    when 8  => z_fa1 <= resize(s_vm, 18);                 z_fb1 <= resize(s_vm, 16);        -- speed^2
                    when 9  => z_fa1 <= z_s4sq;                           z_fb1 <= to_unsigned(1400, 16);   -- speed step
                    when 10 => z_fa1 <= resize(s_gp(12 downto 3), 18);   z_fb1 <= to_unsigned(171, 16);    -- flames A0
                    when others => z_fa1 <= (others => '0');              z_fb1 <= (others => '0');
                end case;
                case to_integer(z_sel) is
                    when 11 => z_fa2 <= resize(s_gp(12 downto 3), 18);   z_fb2 <= to_unsigned(205, 16);    -- flames dA
                    when 12 => z_fa2 <= resize(s_gp(12 downto 3), 18);   z_fb2 <= to_unsigned(375, 16);    -- tongues
                    when 13 => z_fa2 <= resize(s_g6(12 downto 3), 18);   z_fb2 <= to_unsigned(192, 16);    -- e6 (blobs)
                    when 14 =>                                                                              -- im = 2^-e6
                        case z_si is
                            when "001"  => z_fa2 <= to_unsigned(8, 18);
                            when "000"  => z_fa2 <= to_unsigned(4, 18);
                            when "111"  => z_fa2 <= to_unsigned(2, 18);
                            when others => z_fa2 <= to_unsigned(1, 18);
                        end case;
                        z_fb2 <= z_exq;
                    when 15 => z_fa2 <= to_unsigned(222, 18);             z_fb2 <= resize(z_im, 16);        -- thresholds
                    when 16 => z_fa2 <= to_unsigned(236, 18);             z_fb2 <= resize(z_im, 16);
                    when 17 => z_fa2 <= to_unsigned(250, 18);             z_fb2 <= resize(z_im, 16);
                    when 18 => z_fa2 <= to_unsigned(264, 18);             z_fb2 <= resize(z_im, 16);
                    when 19 => z_fa2 <= resize(s_g3(12 downto 3), 18);   z_fb2 <= to_unsigned(22, 16);     -- cross
                    when 20 => z_fa2 <= resize(s_ys, 18);                 z_fb2 <= to_unsigned(293, 16);    -- wave per line
                    when 21 => z_fa2 <= resize(s_g3(12 downto 3), 18);   z_fb2 <= to_unsigned(2574, 16);   -- blob x start
                    when 22 => z_fa2 <= resize(s_ys, 18);                 z_fb2 <= to_unsigned(121, 16);    -- osc y step
                    when others => z_fa2 <= (others => '0');              z_fb2 <= (others => '0');
                end case;
            end if;
            z_pre2 <= z_pre;
            if z_pre2 = '1' then
                if z_hi = '1' then z_fa <= z_fa2; z_fb <= z_fb2; else z_fa <= z_fa1; z_fb <= z_fb1; end if;
            end if;
            z_ld <= z_pre2;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 34); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(32 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            if z_run = '1' and z_ctr = 20 then
                case to_integer(z_sel) is
                    when 0  =>                                  -- e1 = (k1 - 512) * 203 >> 8 (Q8 octaves)
                        v_e := signed(resize(f_acc(18 downto 8), 11)) - 406;
                        z_e1 <= v_e; r_ex_a <= unsigned(v_e(7 downto 0));
                    when 1  => c_c1080 <= f_acc(31 downto 14);  -- C1080 = (66576 >> (1 - e1 >> 8)) * mant >> 14
                    when 2  => c_cinc <= f_acc(23 downto 4);    -- C = C1080 * step / 16
                    when 4  => z_k2sq <= f_acc(19 downto 0);
                    when 5  => c_bq <= f_acc(27 downto 14);
                    when 6  =>                                  -- e5 = -((k5 - 512) * 147 >> 8)
                        v_e := 294 - signed(resize(f_acc(18 downto 8), 11));
                        z_e5 <= v_e; r_ex_a <= unsigned(v_e(7 downto 0));
                    when 7  => c_gq <= f_acc(19 downto 9);      -- spread gain
                    when 8  => z_s4sq <= f_acc(17 downto 0);
                    when 9  => c_spd <= f_acc(31 downto 16);
                    when 10 => c_a0f <= f_acc(18 downto 10);
                    when 11 => c_daf <= f_acc(18 downto 10);
                    when 12 => c_at <= f_acc(18 downto 10);
                    when 13 =>                                  -- e6 = (k6 - 512) * 3 / 4; im = 2^-e6 (exp2 of -e6)
                        v_e := signed(resize(f_acc(17 downto 8), 11)) - 384;
                        z_e6 <= v_e; r_ex_a <= unsigned(-v_e(7 downto 0));
                        if v_e(7 downto 0) = 0 then z_si <= std_logic_vector(-v_e(10 downto 8));
                        else z_si <= std_logic_vector(not v_e(10 downto 8)); end if;
                        if s_g6(12 downto 3) <= 8 then c_boff <= '1'; else c_boff <= '0'; end if;
                        if v_e < -192 then c_ke <= "11";
                        elsif v_e < 64 then c_ke <= "10";
                        elsif v_e < 256 then c_ke <= "01";
                        else c_ke <= "00"; end if;
                    when 14 => z_im <= f_acc(19 downto 9);
                    when 15 => c_t1 <= signed(resize(f_acc(18 downto 8), 12));
                    when 16 => c_t2 <= signed(resize(f_acc(18 downto 8), 12));
                    when 17 => c_t3 <= signed(resize(f_acc(18 downto 8), 12));
                    when 18 => c_t4 <= signed(resize(f_acc(18 downto 8), 12));
                    when 19 => c_sh <= signed(resize(f_acc(14 downto 0), 17)) - 11264;   -- (k3 - 512) * 22, Q4 px
                    when 20 => c_thst <= resize(f_acc(20 downto 8), 16);
                               c_ftw <= 30720 - c_sh;                -- twin: xg = (30720 - shift) - xv left of centre
                    when 21 =>                                  -- (4 step - shift) * 117 = 4 dx7 + 1317888 - k3 * 2574
                        c_x7s <= signed(resize(shift_left(resize(c_dx7, 15), 2), 24)) + 1317888 - signed(resize(f_acc(21 downto 0), 24));
                    when 22 => s_oys <= f_acc(17 downto 8);
                    when others => null;
                end case;
            end if;
            -- blobs off: thresholds out of reach
            if s_zone = '1' and s_zc = 600 and c_boff = '1' then
                c_t1 <= to_signed(1023, 12); c_t2 <= to_signed(1023, 12);
                c_t3 <= to_signed(1023, 12); c_t4 <= to_signed(1023, 12);
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            -- taps 16..28 ride the fringe ring (p_fring: its data register holds tap 28), a fabric copy is tap 29
            s_rg <= fr_q(12 downto 9);
            s_hsync_sr(30) <= s_rg(0); s_vsync_sr(30) <= s_rg(1); s_field_sr(30) <= s_rg(2); s_gav_sr(30) <= s_rg(3);
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
            s_lb_wa <= s_wcol + 4;                -- 4 ahead: the beam's read never meets the write
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine.  A run (kicked at a line's g_start, or twice in the zone)
    -- computes the NEXT line's values from that line's vp, sin and tongue
    -- tooth (read in the hblank before) and, for the line after, its vp and
    -- the next hblank's read addresses.  Four prep clocks, then 17 micro-ops
    -- dest = base +- (src * K >> sh) on one serial 36 x 17 multiplier, 20
    -- clocks each (operands staged at pc 0, loaded at 1, 17 iterations, the
    -- product and base latched at pc 19, written back at pc 20): ~362
    -- clocks, inside SD's 858-clock line.
    -- The hblank read sequence (also twice in the zone) reads sin(theta), the
    -- tongue's tooth and cos(v) for the line two ahead.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_b  : signed(31 downto 0);
        variable v_r  : signed(31 downto 0);
        variable v_ng : std_logic;
    begin
        if rising_edge(clk) then
            -- kicks: each line's g_start; in the zone R0 (vp of line 0) and R1 (line 0's values)
            e_kick <= '0';
            if g_start = '1' then e_kick <= '1'; end if;
            if s_zone = '1' and (s_zc = 700 or s_zc = 1200) then e_kick <= '1'; end if;
            if s_zone = '1' and s_zc = 660 then
                e_vy <= (others => '0'); e_th <= c_th0;
            end if;
            if s_zone = '1' and s_zc = 1150 then
                e_lf <= x"ACE1"; e_n6 <= (others => '0'); e_n2 <= '0';
            end if;
            -- hblank reads: after the generated avid falls, or in the zone (H0 at 1000, H1 at 1500)
            e_hgo <= '0';
            if (s_gav_sr(0) = '0' and s_gav_sr(1) = '1') or (s_zone = '1' and (s_zc = 1100 or s_zc = 1600)) then
                e_hgo <= '1';
            end if;

            if g_start = '1' then
                l_vp <= n_vp; l_av3 <= n_av3; l_xtip <= n_xtip; l_xm <= n_xm; l_d0 <= n_d0; l_d1 <= n_d1;
                l_cv <= n_cv; l_dl <= n_dl; l_c0 <= n_c0; l_inc <= n_inc; l_tinc <= n_tinc;
                l_spl <= n_spl; l_lt <= n_lt; l_jit <= n_jit; l_v15 <= n_v15;
            end if;

            -- read sequencer: addresses at hc 16 (sin) and 17 (tooth, cos(v)), the ports owned (rsel) during
            -- hc 17..18, data latched at 19 (sin) and 20 (tooth, cos(v)); theta advances, vp1 <= vp2 at 22
            if e_hgo = '1' then
                e_hc <= to_unsigned(1, 5);
            elsif e_hc /= 0 then
                e_hc <= e_hc + 1;
            end if;
            if e_hc = 16 then e_caddr <= e_th(15 downto 8) - 64; end if;
            if e_hc = 17 then
                e_taddr <= s_saw & e_tqp(15 downto 8);
                e_caddr <= e_cvph(15 downto 8);
            end if;
            if e_hc = 16 or e_hc = 17 then e_rsel <= '1'; else e_rsel <= '0'; end if;
            if e_hc = 19 then e_s1 <= r_cqa; end if;
            if e_hc = 20 then e_sc1 <= r_tq; e_cv1 <= resize(shift_right(r_cqa, 4), 8); end if;
            if e_hc = 22 then
                e_th <= e_th + c_thst;
                e_vp1 <= e_vp2;
            end if;

            -- serial multiplier
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(16 downto 1);

            -- prep (lc 0..3), then the micro-ops
            if e_kick = '1' then
                e_lc <= (others => '0'); e_run <= '0';
            elsif e_lc /= 7 then
                e_lc <= e_lc + 1;
                if e_lc = 3 then e_run <= '1'; e_op <= (others => '0'); e_pc <= (others => '0'); end if;
            end if;
            case to_integer(e_lc) is
                when 0 =>
                    e_vp <= e_vp1;
                    if e_s1 < 0 then e_sa <= unsigned(-e_s1(10 downto 0)); e_sneg <= '1';
                    else e_sa <= unsigned(e_s1(10 downto 0)); e_sneg <= '0'; end if;
                    if e_sc1 < 0 then e_ca <= unsigned(-e_sc1(6 downto 0)); e_cneg <= '1';
                    else e_ca <= unsigned(e_sc1(6 downto 0)); e_cneg <= '0'; end if;
                    n_cv <= e_cv1;
                    e_y1 <= signed(resize(e_vy(19 downto 8), 12)) - 517;   -- y - 517 of the vp product's line
                    e_lf <= f_lfsr(e_lf);
                when 1 =>
                    if e_y1 < 0 then e_y1 <= -e_y1; end if;
                    e_a0 <= signed(resize(e_vp, 12)) - 160;
                    -- texture: level ripple held 6 lines, end jitter held 2
                    if e_n6 = 0 then n_lt <= signed('0' & e_lf(2 downto 0)) - 4; end if;
                    if e_n2 = '0' then n_jit <= signed('0' & e_lf(9 downto 8)) - 2; end if;
                    if e_n6 = 5 then e_n6 <= (others => '0'); else e_n6 <= e_n6 + 1; end if;
                    e_n2 <= not e_n2;
                    if e_vp > 15 then n_v15 <= '1'; else n_v15 <= '0'; end if;
                when 2 =>
                    e_yq <= shift_left(e_y1, 1) - 517;                          -- Quad: |2 v - 517|
                    if e_a0 < 0 then e_av <= unsigned(-e_a0(10 downto 0)); else e_av <= unsigned(e_a0(10 downto 0)); end if;
                when 3 =>
                    if s_quad = '1' then
                        if e_yq < 0 then e_v2 <= unsigned(-e_yq(9 downto 0)); else e_v2 <= unsigned(e_yq(9 downto 0)); end if;
                    else
                        e_v2 <= unsigned(e_y1(9 downto 0));
                    end if;
                when others => null;
            end case;

            if e_run = '1' then
                if e_pc = 20 then
                    e_pc <= (others => '0');
                    if e_op = 16 then e_run <= '0'; else e_op <= e_op + 1; end if;
                else
                    e_pc <= e_pc + 1;
                end if;
                if e_pc = 0 then                                     -- 65536 +- |bs| for op 2
                    if e_sneg = '1' then e_vw <= to_unsigned(65536, 17) + resize(e_bsa, 17);
                    else e_vw <= to_unsigned(65536, 17) - resize(e_bsa, 17); end if;
                end if;
                if e_pc = 1 then                                     -- load the op's operands
                    m_acc <= (others => '0');
                    case to_integer(e_op) is
                        when 0  => m_a <= to_unsigned(768, 36);   m_b <= resize(e_av, 17);           -- av3
                        when 1  => m_a <= resize(c_bq, 36);       m_b <= resize(e_sa, 17);           -- |bs|
                        when 2  =>                                                                    -- incl
                            m_a <= resize(c_cinc, 36);
                            m_b <= e_vw;
                        when 3  => m_a <= resize(n_inc, 36);      if s_saw = '1' then m_b <= to_unsigned(256, 17); else m_b <= to_unsigned(768, 17); end if;
                        when 4  => m_a <= resize(c_c1080, 36);    m_b <= to_unsigned(208, 17);       -- 13 C1080 / 16
                        when 5  => m_a <= resize(unsigned(n_phs(19 downto 0)), 36);
                                   if s_saw = '1' then m_b <= to_unsigned(256, 17); else m_b <= to_unsigned(768, 17); end if;
                        when 6  => m_a <= to_unsigned(360, 36);   m_b <= resize(e_av, 17);           -- 1.406 av
                        when 7  => m_a <= to_unsigned(6, 36);     m_b <= resize(e_bsa, 17);          -- displacement
                        when 8  => m_a <= resize(c_at, 36);       m_b <= resize(e_ca, 17);           -- tongue
                        when 9  => m_a <= to_unsigned(202, 36);   m_b <= resize(e_vp, 17);           -- xM
                        when 10 => m_a <= to_unsigned(197, 36);   m_b <= resize(e_vp, 17);           -- drip chevron
                        when 11 => m_a <= to_unsigned(35936, 36); m_b <= resize(e_vp, 17);           -- blob diagonal
                        when 12 => m_a <= to_unsigned(819, 36);   m_b <= resize(e_vp, 17);           -- corner
                        when 13 => m_a <= to_unsigned(6, 36);     m_b <= resize(e_sa, 17);           -- split wiggle
                        when 14 => m_a <= resize(c_gq, 36);       m_b <= resize(e_v2, 17);           -- vp, the line after
                        when 15 => m_a <= to_unsigned(24064, 36); m_b <= resize(e_vp2, 17);          -- cos(v) phase
                        when others => m_a <= to_unsigned(796, 36); m_b <= resize(e_th, 17);         -- tongue phase
                    end case;
                end if;
                if e_pc = 19 then                                    -- the product >> sh, the base and its sign
                    if e_op = 1 then e_wp <= resize(m_acc(35 downto 11), 28);
                    elsif e_op = 2 then e_wp <= resize(m_acc(35 downto 20), 28);
                    else e_wp <= m_acc(35 downto 8); end if;
                    v_ng := '0';
                    case to_integer(e_op) is
                        when 4      => v_b := signed(resize(shift_left(resize(n_inc, 24), 2), 32)); v_ng := '1';
                        when 5      => v_b := to_signed(C_TPH, 32);
                        when 6      => v_b := to_signed(1560, 32); v_ng := '1';
                        when 7      => v_b := resize(e_tip, 32);   v_ng := e_sneg;
                        when 8      => v_b := resize(e_tip, 32);   v_ng := e_cneg;
                        when 9      => v_b := to_signed(1493, 32); v_ng := '1';
                        when 10     => v_b := to_signed(420, 32);  v_ng := '1';
                        when 11     => v_b := to_signed(-70218, 32);
                        when 12     => v_b := to_signed(-1056, 32);
                        when 13     => v_b := to_signed(1530, 32); v_ng := e_sneg;
                        when 15     => v_b := to_signed(-33703, 32);
                        when others => v_b := (others => '0');
                    end case;
                    e_wng <= v_ng;
                end if;
                if e_pc = 20 then                                    -- write back base +- (product >> sh)
                    case to_integer(e_op) is
                        when 4      => v_b := signed(resize(shift_left(resize(n_inc, 24), 2), 32));
                        when 5      => v_b := to_signed(C_TPH, 32);
                        when 6      => v_b := to_signed(1560, 32); null;
                        when 7      => v_b := resize(e_tip, 32);   null;
                        when 8      => v_b := resize(e_tip, 32);   null;
                        when 9      => v_b := to_signed(1493, 32); null;
                        when 10     => v_b := to_signed(420, 32);  null;
                        when 11     => v_b := to_signed(-70218, 32);
                        when 12     => v_b := to_signed(-1056, 32);
                        when 13     => v_b := to_signed(1530, 32); null;
                        when 15     => v_b := to_signed(-33703, 32);
                        when others => v_b := (others => '0');
                    end case;

                    if e_wng = '1' then v_r := v_b - signed(resize(e_wp, 32));
                    else v_r := v_b + signed(resize(e_wp, 32)); end if;
                    case to_integer(e_op) is
                        when 0  => n_av3 <= unsigned(v_r(11 downto 0));
                        when 1  => e_bsa <= unsigned(v_r(13 downto 0));
                        when 2  => n_inc <= unsigned(v_r(16 downto 0));
                        when 3  => n_tinc <= unsigned(v_r(17 downto 0));
                        when 4  => n_phs <= v_r(25 downto 0);
                        when 5  => n_tst <= unsigned(v_r(19 downto 0));
                        when 6 | 7 => e_tip <= v_r(12 downto 0);
                        when 8  => n_xtip <= v_r(12 downto 0);
                        when 9  => n_xm <= v_r(11 downto 0);
                        when 10 => n_d0 <= v_r(11 downto 0); e_d1w <= '1';
                        when 11 => n_dl <= unsigned(v_r(15 downto 0));
                        when 12 => n_c0 <= v_r(12 downto 0);
                        when 13 => n_spl <= v_r(11 downto 0);
                        when 14 => e_vp2 <= unsigned(v_r(10 downto 0));
                        when 15 => e_cvph <= unsigned(v_r(15 downto 0));
                        when others =>                               -- the run's end
                            e_tqp <= unsigned(v_r(15 downto 0));
                            n_vp <= e_vp;
                            e_vy <= e_vy + resize(s_ys, 20);
                    end case;
                end if;
            end if;
            if e_d1w = '1' then n_d1 <= n_d0 + 125; e_d1w <= '0'; end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: virtual x (Q4), the stripe oscillator phase (Q24) and the teeth
    -- phase (3x the stripe phase + offset, Blade: 1x; Q24 mod 1)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px <= to_unsigned(C_LEAD, 11);
                b_xv <= resize(to_unsigned(C_LEAD, 10) * c_step, 16);
                b_ph <= n_phs;
                b_tq <= n_tst;
                b_x7 <= c_x7s;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_xv <= b_xv + c_step;
                b_ph <= b_ph + signed(resize(l_inc, 26));
                b_tq <= b_tq + resize(l_tinc, 20);
                b_x7 <= b_x7 + signed(resize(c_dx7, 24));
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel; block N assigns the
    -- stage-N registers; the pixel's generated avid in block N is
    -- s_gav_sr(N - 3))
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : signed(13 downto 0);
        variable v_c   : signed(17 downto 0);
        variable v_t   : signed(14 downto 0);
        variable v_o   : signed(13 downto 0);
        variable v_i   : unsigned(7 downto 0);
        variable v_lv  : unsigned(5 downto 0);
        variable v_jq  : signed(7 downto 0);
        variable v_g0, v_g1, v_g2 : unsigned(14 downto 0);
        variable v_h0, v_h1, v_h2 : signed(13 downto 0);
        variable v_n   : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: geometry x (twin fold, Cross); stripe index flags and phase; the teeth phase
            if s_twin = '1' and b_xv < 15360 then p2_xg <= c_ftw - signed(resize(b_xv, 17));
            else p2_xg <= signed(resize(b_xv, 17)) - c_sh; end if;
            p2_kneg <= b_ph(25);
            if b_ph(25 downto 20) = 0 then p2_kz <= '1'; else p2_kz <= '0'; end if;
            p2_q <= unsigned(b_ph(19 downto 14));
            p2_qt <= b_tq(19 downto 12);
            p2_x117 <= unsigned(b_x7(19 downto 4));
            -- 3: |x| (corner), x in px; the stripe gate
            p3_x117 <= p2_x117;
            if p2_xg < 0 then p3_ax <= unsigned(-p2_xg); else p3_ax <= unsigned(p2_xg); end if;
            p3_xpx <= resize(shift_right(p2_xg, 4), 13);
            if p2_kneg = '0' and p2_kz = '0' then v_jq := resize(l_jit, 8); else v_jq := (others => '0'); end if;
            if p2_kneg = '0' and signed(resize(p2_q, 8)) < 26 + v_jq and (p2_kz = '0' or p2_q >= 8) then
                p3_son <= '1';
            else
                p3_son <= '0';
            end if;
            if p2_kneg = '0' and p2_kz = '0' then p3_kp <= '1'; else p3_kp <= '0'; end if;
            v_i := p2_qt + 90;
            if v_i < 141 and l_v15 = '1' then p3_dg <= '1'; else p3_dg <= '0'; end if;
            p3_qt <= p2_qt;
            -- 4: corner ramp; the x tests against the line's split / tip / drip window / arm; stripe ROM data
            --    (fabric copy); luma (cos ROM addresses from the blob x phase: p_crom)
            p4_x117 <= p3_x117;
            p4_cx <= resize(shift_right(p3_ax, 3) - shift_right(p3_ax, 7), 14);
            if p3_xpx >= resize(l_spl, 13) then pspl(4) <= '1'; else pspl(4) <= '0'; end if;
            if p3_xpx < l_xtip then ptip(4) <= '1'; else ptip(4) <= '0'; end if;
            if p3_dg = '1' and p3_xpx >= resize(l_d0, 13) and p3_xpx < resize(l_d1, 13) then pdw(4) <= '1';
            else pdw(4) <= '0'; end if;
            p4_dx <= resize(p3_xpx, 14) - resize(l_xm, 14);
            p4_sr <= r_sq;
            p4_lu <= unsigned(s_lb_q);                                   -- the line buffer's data for this pixel
            p4_son <= p3_son; p4_kp <= p3_kp; p4_qt <= p3_qt;
            -- 5: x ROM data (fabric copy); corner; etch; stripe level
            p5_xr <= r_xq;
            v_c := resize(l_c0, 18) - signed(resize(p4_cx, 18));
            if v_c < 0 then p5_cor <= (others => '0');
            elsif v_c > 384 then p5_cor <= to_unsigned(384, 9);
            else p5_cor <= unsigned(v_c(8 downto 0)); end if;
            if c_boff = '1' then p5_etv <= (others => '0');
            else p5_etv <= shift_right(p4_lu, to_integer(c_ke)); end if;
            if p4_kp = '1' then
                p5_lvl <= signed(resize(unsigned(p4_sr(5 downto 0)), 8)) + resize(l_lt, 8);
            else
                p5_lvl <= signed(resize(unsigned(p4_sr(5 downto 0)), 8));
            end if;
            p5_teal <= p4_sr(6);
            p5_son <= p4_son; p5_qt <= p4_qt; p5_lu <= p4_lu;
            pspl(5) <= pspl(4); ptip(5) <= ptip(4); pdw(5) <= pdw(4);
            if p4_dx < -512 then pex(5) <= to_signed(-512, 10);
            elsif p4_dx > 511 then pex(5) <= to_signed(511, 10);
            else pex(5) <= p4_dx(9 downto 0); end if;
            -- 6: band edges (x ROM: max of the L band and the R arm); the fan (shear) and amplitude partials by
            --    near; drip lump edge; ground index (tooth ROM address: p_trom)
            v_n := unsigned(p5_xr(22 downto 20));
            p6_pin <= signed(resize(unsigned(p5_xr(9 downto 0)), 14));
            p6_pout <= signed(resize(unsigned(p5_xr(19 downto 10)), 14));
            if v_n(0) = '1' then v_g0 := resize(l_av3, 15); else v_g0 := (others => '0'); end if;
            if v_n(1) = '1' then v_g1 := shift_left(resize(l_av3, 15), 1); else v_g1 := (others => '0'); end if;
            if v_n(2) = '1' then v_g2 := shift_left(resize(l_av3, 15), 2); else v_g2 := (others => '0'); end if;
            p6_g <= v_g0 + v_g1 + v_g2;
            if v_n(0) = '1' then
                if v_n(1) = '1' then p6_da <= resize(c_daf, 11) + shift_left(resize(c_daf, 11), 1);
                else p6_da <= resize(c_daf, 11); end if;
            else
                if v_n(1) = '1' then p6_da <= shift_left(resize(c_daf, 11), 1);
                else p6_da <= (others => '0'); end if;
            end if;
            if v_n(2) = '1' then p6_da4 <= shift_left(resize(c_daf, 11), 2); else p6_da4 <= (others => '0'); end if;
            pnear(6) <= v_n;
            p6_pl <= signed(resize(unsigned(p5_xr(9 downto 0)), 11)) - signed(resize(unsigned(p5_xr(29 downto 23)), 11));
            if unsigned(p5_xr(29 downto 23)) > 1 then p6_lk0 <= '1'; else p6_lk0 <= '0'; end if;
            if p5_lvl < 1 then v_lv := to_unsigned(1, 6);
            elsif p5_lvl > 63 then v_lv := to_unsigned(63, 6);
            else v_lv := unsigned(p5_lvl(5 downto 0)); end if;
            if p5_son = '1' then pgi(6) <= '0' & p5_teal & v_lv; else pgi(6) <= to_unsigned(128, 8); end if;
            p6_vv <= signed(resize(l_vp, 13)) + signed(resize(p5_etv, 13));
            p6_cor <= p5_cor; p6_qt <= p5_qt; p6_lu <= p5_lu;
            pspl(6) <= pspl(5); ptip(6) <= ptip(5); pdw(6) <= pdw(5); pex(6) <= pex(5);
            -- 7: amplitude; vv against the edges; drip lump; cos data (fabric copies)
            p7_amp <= c_a0f + resize(shift_right(p6_da + p6_da4, 3), 9);
            v_d := resize(p6_vv, 14) - p6_pin;                       -- vv - edges, saturated to +-511 (exact for
            if v_d < -512 then pdpi(7) <= to_signed(-512, 10);       -- every comparison made with them)
            elsif v_d > 511 then pdpi(7) <= to_signed(511, 10);
            else pdpi(7) <= v_d(9 downto 0); end if;
            v_d := resize(p6_vv, 14) - p6_pout;
            if v_d < -512 then pdpo(7) <= to_signed(-512, 10);
            elsif v_d > 511 then pdpo(7) <= to_signed(511, 10);
            else pdpo(7) <= v_d(9 downto 0); end if;
            if p6_lk0 = '1' and resize(p6_vv, 14) > resize(p6_pl, 14) then p7_lk <= '1'; else p7_lk <= '0'; end if;
            p7_cx <= resize(shift_right(r_cqa, 4), 8);
            p7_cd <= resize(shift_right(r_cqb, 4), 8);
            pnear(7) <= pnear(6); pgi(7) <= pgi(6); p7_cor <= p6_cor; p7_lu <= p6_lu;
            pspl(7) <= pspl(6); ptip(7) <= ptip(6); pdw(7) <= pdw(6); pex(7) <= pex(6);
            -- 8: blob x terms; drip flag
            p8_b1 <= resize(p7_cx, 11) + resize(p7_cd, 11) - resize(shift_right(p7_cd, 3), 11);
            pdw(8) <= pdw(7) or p7_lk;
            p8_amp <= p7_amp; p8_cor <= p7_cor; p8_lu <= p7_lu;
            pdpi(8) <= pdpi(7); pdpo(8) <= pdpo(7); pnear(8) <= pnear(7); pgi(8) <= pgi(7);
            pspl(8) <= pspl(7); ptip(8) <= ptip(7); pex(8) <= pex(7);
            -- 9: blob + v term + corner; tooth (fabric copy)
            p9_b2 <= resize(p8_b1, 12) + resize(l_cv, 12) + signed(resize(p8_cor, 12));
            p9_tooth <= r_tq;
            p9_amp <= p8_amp; p9_lu <= p8_lu;
            pdpi(9) <= pdpi(8); pdpo(9) <= pdpo(8); pnear(9) <= pnear(8); pgi(9) <= pgi(8);
            pspl(9) <= pspl(8); ptip(9) <= ptip(8); pdw(9) <= pdw(8); pex(9) <= pex(8);
            -- 10: amp * tooth, split lo / hi nibble; blob + luma (15/16)
            p10_ml <= resize(p9_amp * unsigned(p9_tooth(3 downto 0)), 13);
            p10_mh <= resize(signed('0' & p9_amp) * p9_tooth(7 downto 4), 14);
            p10_blob <= p9_b2 + signed(resize(p9_lu, 12)) - signed(resize(shift_right(p9_lu, 4), 12));
            pdpi(10) <= pdpi(9); pdpo(10) <= pdpo(9); pnear(10) <= pnear(9); pgi(10) <= pgi(9);
            pspl(10) <= pspl(9); ptip(10) <= ptip(9); pdw(10) <= pdw(9); pex(10) <= pex(9);
            -- 11: toff (Blade: x2); hole level (thermometer over the 4 thresholds)
            v_t := shift_left(resize(p10_mh, 15), 4) + signed(resize(p10_ml, 15));
            if s_saw = '1' then p11_toff <= resize(shift_right(v_t, 7), 11);
            else p11_toff <= resize(shift_right(v_t, 8), 11); end if;
            if p10_blob > c_t4 then p11_h <= "100";
            elsif p10_blob > c_t3 then p11_h <= "011";
            elsif p10_blob > c_t2 then p11_h <= "010";
            elsif p10_blob > c_t1 then p11_h <= "001";
            else p11_h <= "000"; end if;
            pdpi(11) <= pdpi(10); pdpo(11) <= pdpo(10); pnear(11) <= pnear(10); pgi(11) <= pgi(10);
            pspl(11) <= pspl(10); ptip(11) <= ptip(10); pdw(11) <= pdw(10); pex(11) <= pex(10);
            -- 12: inside the wedge; toff * near; the arm's toothed edge offset
            if resize(pdpi(11), 12) < resize(p11_toff, 12) then p12_inw <= '1'; else p12_inw <= '0'; end if;
            if pnear(11)(0) = '1' then v_h0 := resize(p11_toff, 14); else v_h0 := (others => '0'); end if;
            if pnear(11)(1) = '1' then v_h1 := shift_left(resize(p11_toff, 14), 1); else v_h1 := (others => '0'); end if;
            if pnear(11)(2) = '1' then v_h2 := shift_left(resize(p11_toff, 14), 2); else v_h2 := (others => '0'); end if;
            p12_tn <= v_h0 + v_h1 + v_h2;
            p12_tm <= shift_right(p11_toff, 1) + shift_right(p11_toff, 3);
            p12_h <= p11_h;
            pdpo(12) <= pdpo(11); pgi(12) <= pgi(11);
            pspl(12) <= pspl(11); ptip(12) <= ptip(11); pdw(12) <= pdw(11); pex(12) <= pex(11);
            -- 13: the outer edge's flame offset; the arm's toothed x
            p13_vo <= shift_right(p12_tn, 3) + shift_right(p12_tn, 5);
            p13_exm <= resize(pex(12), 11) - p12_tm;
            p13_inw <= p12_inw; p13_h <= p12_h;
            pdpo(13) <= pdpo(12); pgi(13) <= pgi(12);
            pspl(13) <= pspl(12); ptip(13) <= ptip(12); pdw(13) <= pdw(12);
            -- 14: vv - vout; the arm's zone flags
            p14_dvo <= resize(pdpo(13), 12) - resize(p13_vo, 12);
            if p13_exm < 0 and p13_exm >= -290 then p14_mz <= '1'; else p14_mz <= '0'; end if;
            if p13_exm >= -122 then p14_my <= '1'; else p14_my <= '0'; end if;
            if p13_exm >= -174 then p14_mr <= '1'; else p14_mr <= '0'; end if;
            if p13_exm < -290 then p14_bz <= '1'; else p14_bz <= '0'; end if;
            p14_inw <= p13_inw; p14_h <= p13_h; pgi(14) <= pgi(13);
            pspl(14) <= pspl(13); ptip(14) <= ptip(13); pdw(14) <= pdw(13);
            -- 15: band / field flags, the soft edge level
            if p14_inw = '0' and p14_dvo < 0 then p15_ina <= '1'; else p15_ina <= '0'; end if;
            if p14_dvo >= -10 then p15_vm <= '1'; else p15_vm <= '0'; end if;
            if p14_dvo >= 0 then p15_bv <= '1'; else p15_bv <= '0'; end if;
            p15_mz <= p14_mz; p15_my <= p14_my; p15_mr <= p14_mr; p15_bz <= p14_bz;
            p15_inw <= p14_inw; p15_h <= p14_h; pgi(15) <= pgi(14);
            pspl(15) <= pspl(14); ptip(15) <= ptip(14); pdw(15) <= pdw(14);
            -- 16: the colour index
            if p15_inw = '1' then
                if pdw(15) = '1' then v_i := to_unsigned(129, 8); else v_i := pgi(15); end if;
            elsif p15_ina = '1' and pspl(15) = '1' then
                v_i := to_unsigned(129, 8);
            elsif p15_ina = '1' and ptip(15) = '1' then
                v_i := to_unsigned(131, 8) + resize(p15_h, 8);
            elsif p15_mz = '1' and p15_vm = '1' then
                if p15_my = '1' then v_i := to_unsigned(129, 8);
                elsif p15_mr = '1' then v_i := to_unsigned(130, 8);
                else v_i := to_unsigned(131, 8) + resize(p15_h, 8); end if;
            elsif p15_bz = '1' and p15_bv = '1' then
                v_i := to_unsigned(136, 8) + resize(p15_h, 8);
            else
                v_i := pgi(15);
            end if;
            p16_idx <= v_i;
            if v_i <= 128 then p16_dark <= '1'; else p16_dark <= '0'; end if;
            -- 17..29: the index rides the fringe ring (p_fring); the dark flags a 13-bit shift register
            fd_dk <= fd_dk(11 downto 0) & p16_dark;
            fr_rp <= o_wp - 11;
            -- 30: the index out of the ring (fabric copy)
            p30_fr <= unsigned(fr_q(7 downto 0));
            -- 31: magenta 1..12 px before a dark pixel -> magenta -> white (144 + 13 - d)
            p31_idx <= p30_fr;
            if p30_fr = 131 then
                if    fd_dk(12) = '1' then p31_idx <= to_unsigned(156, 8);
                elsif fd_dk(11) = '1' then p31_idx <= to_unsigned(155, 8);
                elsif fd_dk(10) = '1' then p31_idx <= to_unsigned(154, 8);
                elsif fd_dk(9) = '1'  then p31_idx <= to_unsigned(153, 8);
                elsif fd_dk(8) = '1'  then p31_idx <= to_unsigned(152, 8);
                elsif fd_dk(7) = '1'  then p31_idx <= to_unsigned(151, 8);
                elsif fd_dk(6) = '1'  then p31_idx <= to_unsigned(150, 8);
                elsif fd_dk(5) = '1'  then p31_idx <= to_unsigned(149, 8);
                elsif fd_dk(4) = '1'  then p31_idx <= to_unsigned(148, 8);
                elsif fd_dk(3) = '1'  then p31_idx <= to_unsigned(147, 8);
                elsif fd_dk(2) = '1'  then p31_idx <= to_unsigned(146, 8);
                elsif fd_dk(1) = '1'  then p31_idx <= to_unsigned(145, 8);
                end if;
            end if;
            -- 32, 33: palette ROM (p_prom); 34: fabric copy
            p34_y <= unsigned(r_pq(29 downto 20));
            p34_u <= unsigned(r_pq(19 downto 10));
            p34_v <= unsigned(r_pq(9 downto 0));
            -- 35: compose; black outside the generated avid
            if s_gav_sr(32) = '0' then
                s_y35 <= C_KY; s_u35 <= C_KC; s_v35 <= C_KC;
            else
                s_y35 <= p34_y; s_u35 <= p34_u; s_v35 <= p34_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(32);
        end if;
    end process p_pix;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            o_wp <= o_wp + 1;
            o_rp <= o_wp - 5;
            o_fya <= unsigned(o_qya(9 downto 0)); o_fua <= unsigned(o_qua(9 downto 0)); o_fva <= unsigned(o_qva(9 downto 0));
            o_fyb <= unsigned(o_qyb(9 downto 0)); o_fub <= unsigned(o_qub(9 downto 0)); o_fvb <= unsigned(o_qvb(9 downto 0));
            -- box A: stage 36 sums, stage 37 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KC & "000"; s_bav <= C_KC & "000";
            else
                s_bay <= s_bay + s_y35 - o_fya;
                s_bau <= s_bau + s_u35 - o_fua;
                s_bav <= s_bav + s_v35 - o_fva;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(34) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y37 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u37 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v37 <= v_sc(13 downto 4);
            else
                s_y37 <= C_KY; s_u37 <= C_KC; s_v37 <= C_KC;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(34);
            -- box B: stage 38 sums, stage 39 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KC & "000"; s_bbv <= C_KC & "000";
            else
                s_bby <= s_bby + s_y37 - o_fyb;
                s_bbu <= s_bbu + s_u37 - o_fub;
                s_bbv <= s_bbv + s_v37 - o_fvb;
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y39 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u39 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v39 <= v_sc(13 downto 4);
            -- 40: luma 13/16
            if s_y39 > 64 then v_dl := s_y39 - 64; else v_dl := (others => '0'); end if;
            s_y40 <= s_y39 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u40 <= s_u39; s_v40 <= s_v39;
            -- 41: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y41 <= s_y40; s_u41 <= s_u40; s_v41 <= s_v40;
            else
                s_y41 <= C_KY; s_u41 <= C_KC; s_v41 <= C_KC;
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y41);
    data_out.u       <= std_logic_vector(s_u41);
    data_out.v       <= std_logic_vector(s_v41);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture saltire;
