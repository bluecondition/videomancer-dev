-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Effigy v0.1.0 -- Heirloom (video clips), rebuilt from the clip still WIN_20231229_00_46_59 at 6.0 s.
--
-- The still: a yellow idol standing in the middle of the frame (a column of hexagonal blobs and a heart,
-- mirrored about x = 1002), cut into windows by horizontal black bands whose halos glow dark red .. bright
-- red toward the right, with orange teeth along their edges; around the idol its own outline echoed outward
-- as nested zigzag contour lines (dark green in the upper left, dark red below, bright red on the right,
-- with orange shards), short orange streaks held at the left edge and a yellow ghost blob against it.
-- In the clip the idol's profile scrolls upward through the fixed bands at ~108 rows/s and the bands bob.
--
-- The patch, as rebuilt (effigy.py has the tables, .heirloom_work/rb_effigy/imodel.py is the spec):
--   F      = w(y) - |x - 1002| + Etch * luma     the idol key: yellow where F > 0, an orange rim below
--   lines  = (w_line(y) - |x - 1002|) / lambda   contour phase; a trapezoid line per period, each segment
--                                                 (line index x 48-row block) hashed to off / dim / bright / orange
--   bands  = four bands from 16 knots per band (centre, half-height, halo gain), evaluated per line,
--            lerped per pixel; black core, red halo near the edge, teeth just outside
--   w(y), w_line(y) = a measured 1024-entry table, scrolled upward (Speed) and stretched (Stretch)
--
--   K1  Idol      idol width x0.25 .. x4 (exponential; 512 = the still)
--   K2  Stretch   the idol table's vertical rate x0.4 .. x2.5: one tall vase .. a stack of small blobs
--   K3  Speed     bipolar: the profile (and its contours, the ghost and the streaks) scrolls up / down;
--                 680 = the clip's 108 rows/s
--   K4  Contours  contour spacing x0.25 .. x4 (62 / 140 px left / right at 512)
--   K5  Bands     band height: none (0) .. the still (512) .. +80 px (1023)
--   K6  Etch      how far the camera (or the Osc) bends the idol and its contours
--   S7  Mirror    Off / On: the right side's flames (bands, halos, wide contours) on both sides
--   S8  Palette   Ember / Jade: green band halos and green contours (the clip's other look)
--   S9  Twin      one idol / two idols side by side
--   S10 Wave      Off / On: the bands undulate (a travelling sine on the band centres)
--   S11 Source    Video / Osc
--   P12 BURN      0 = embers (the idol in black, faint outlines) .. the still (410) .. 100 % = inferno
--                 (every contour a fat orange stripe, blazing halos, long teeth)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 / reciprocal ROM), a per-line
-- engine kicked at each line start that computes the NEXT line (table lookups, the ghost, the streak, the
-- band knots into a ping-pong knot buffer), a 21-stage pixel path.  Virtual 1080-line coordinates (SD
-- shows the centre 4:3).  Generated avid; output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16,
-- luma 13/16) before the blanking gate; one 28-clock latency in every mode.  Colours in the hardware
-- convention (U = Cr, V = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.effigy_pkg.all;
use work.all;

architecture effigy of program_top is
    attribute no_rw_check : boolean;
    constant C_LAT : integer := 28;   -- = processing_delay_clks
    constant C_XC  : integer := 2004; -- idol axis, h (half virtual px)

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

    -- line buffer (every other source pixel): camera luma (Y >> 3) - 8 clipped, or the Osc lattice (signed)
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_tab_a : unsigned(9 downto 0) := (others => '0');
    signal r_tab_q : std_logic_vector(17 downto 0) := (others => '0');
    signal kr_a : unsigned(5 downto 0) := (others => '0');
    signal r_kn, q_kn : std_logic_vector(25 downto 0) := (others => '0');
    signal r_pal_a : unsigned(8 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(23 downto 0) := (others => '0');
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_rnd_a : unsigned(7 downto 0) := (others => '0');
    signal r_rnd_q, p12_h : unsigned(7 downto 0) := (others => '0');
    signal e_exa : unsigned(8 downto 0) := (others => '0');            -- the line engine's EXR address

    -- per-field tables written in the zone: etch offset by source sample (h), line level by class and distance
    type t_et is array (0 to 255) of std_logic_vector(15 downto 0);
    signal etr : t_et := (others => (others => '0'));
    attribute no_rw_check of etr : signal is true;
    signal et_we : std_logic := '0';
    signal et_wa, et_ra : unsigned(7 downto 0) := (others => '0');
    signal et_wd, r_et : std_logic_vector(15 downto 0) := (others => '0');
    signal et_v : signed(17 downto 0) := (others => '0');
    type t_lv is array (0 to 511) of std_logic_vector(7 downto 0);
    signal lvr : t_lv := (others => (others => '0'));
    attribute no_rw_check of lvr : signal is true;
    signal lv_we : std_logic := '0';
    signal lv_wa, lv_ra : unsigned(8 downto 0) := (others => '0');
    signal lv_wd, r_lv : std_logic_vector(7 downto 0) := (others => '0');
    signal lv_v : signed(15 downto 0) := (others => '0');

    -- (attribute declared below with the knot buffer)
    -- knot buffer: per segment {E0, E1 (10 signed), G0, G1 (8, Q5)}, two banks of 16 (ping-pong per line)
    type t_kb is array (0 to 31) of std_logic_vector(35 downto 0);
    signal kbr : t_kb := (others => (others => '0'));
    attribute no_rw_check of kbr : signal is true;
    signal kb_we : std_logic := '0';
    signal kb_wa, kb_ra : unsigned(4 downto 0) := (others => '0');
    signal kb_wd : std_logic_vector(35 downto 0) := (others => '0');
    signal r_kb  : std_logic_vector(35 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 160 x 135 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(408, 10);
    signal s_oys : unsigned(11 downto 0) := to_unsigned(484, 12);

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

    -- knobs: glided (Q3) K1, K2, K4, K5, K6, P12; raw speed; switches
    type t_g is array (0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(512 * 8, 13), to_unsigned(450 * 8, 13), to_unsigned(410 * 8, 13));
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k3 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal f_mir, f_jade, f_twin, f_wave, f_osc : std_logic := '0';

    -- per-field constants
    signal c_wg   : unsigned(9 downto 0) := to_unsigned(128, 10);    -- idol width gain, Q7
    signal c_st   : unsigned(10 downto 0) := to_unsigned(243, 11);   -- table entries per row, Q8
    signal c_ill  : unsigned(11 downto 0) := to_unsigned(529, 12);   -- contour periods per h, Q16 (left)
    signal c_ilr  : unsigned(11 downto 0) := to_unsigned(234, 12);   -- (right)
    signal c_ml, c_mr : unsigned(6 downto 0) := to_unsigned(66, 7);  -- the same as mantissa << shift
    signal c_sl, c_sr : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal c_thick : signed(8 downto 0) := (others => '0');          -- band half-height offset, vpx
    signal c_eg   : unsigned(9 downto 0) := (others => '0');         -- etch gain (h per source step x 64)
    signal c_sa, c_shs, c_stt : unsigned(9 downto 0) := to_unsigned(256, 10);   -- Burn: lines, halo, teeth (Q8)
    signal c_sh5  : unsigned(4 downto 0) := to_unsigned(8, 5);       -- halo scale, Q3
    signal c_t4   : unsigned(3 downto 0) := to_unsigned(5, 4);       -- tooth scale
    signal c_hwa  : unsigned(5 downto 0) := (others => '0');
    signal c_a0, c_a1, c_a2, c_a3 : unsigned(6 downto 0) := (others => '0');   -- line levels per class
    signal c_k0, c_k1, c_k2, c_k3 : unsigned(7 downto 0) := (others => '0');   -- line slopes per class
    signal c_s30, c_s70 : unsigned(6 downto 0) := (others => '0');             -- streak levels
    signal c_spd  : unsigned(12 downto 0) := (others => '0');
    signal c_spn  : std_logic := '0';
    signal c_sc   : unsigned(17 downto 0) := (others => '0');          -- scroll, Q8 table entries
    signal c_srow : unsigned(10 downto 0) := (others => '0');          -- scroll in rows
    signal c_bp   : unsigned(8 downto 0) := to_unsigned(120, 9);       -- bob phase (fields mod 480)
    signal c_bob  : signed(6 downto 0) := (others => '0');
    signal c_phi  : unsigned(12 downto 0) := (others => '0');          -- wave phase
    -- sequencer scratch
    signal z_sh1, z_sh2, z_sh4 : signed(2 downto 0) := (others => '0');
    signal z_f1, z_f2, z_f4 : unsigned(7 downto 0) := (others => '0');
    signal z_d2, z_e2 : signed(10 downto 0) := (others => '0');
    signal z_m  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_lo : std_logic := '1';
    signal z_q  : unsigned(9 downto 0) := (others => '0');
    signal z_ba : unsigned(7 downto 0) := (others => '0');
    signal z_bt : unsigned(9 downto 0) := (others => '0');
    signal z_sac : unsigned(9 downto 0) := (others => '0');
    signal z_hx : unsigned(6 downto 0) := (others => '0');
    signal z_ht : unsigned(7 downto 0) := (others => '0');
    signal z_hw0, z_hw1, z_hw2, z_hw3 : unsigned(6 downto 0) := to_unsigned(23, 7);
    signal z_r0, z_r1, z_r2, z_r3 : unsigned(7 downto 0) := (others => '0');
    -- field serial multiplier (a x b, b shifted out LSB first, 16 iterations)
    signal f_a   : unsigned(30 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(30 downto 0) := (others => '0');
    signal z_fa  : unsigned(17 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

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
    signal e_y    : unsigned(10 downto 0) := (others => '0');
    signal e_bank : std_logic := '0';
    signal m_a    : unsigned(21 downto 0) := (others => '0');
    signal m_b    : unsigned(11 downto 0) := (others => '0');
    signal m_acc  : unsigned(21 downto 0) := (others => '0');
    signal e_wl   : unsigned(8 downto 0) := (others => '0');
    signal e_v0   : unsigned(11 downto 0) := (others => '0');
    signal e_va   : unsigned(5 downto 0) := (others => '0');
    signal e_gon  : std_logic := '0';
    signal e_ys   : unsigned(11 downto 0) := (others => '0');
    signal e_st   : unsigned(5 downto 0) := (others => '0');
    signal e_blk  : unsigned(5 downto 0) := (others => '0');
    signal e_kbt  : unsigned(7 downto 0) := (others => '0');
    -- next-line results
    signal n_wh, n_gwh : signed(12 downto 0) := (others => '0');
    signal n_wlh  : unsigned(11 downto 0) := (others => '0');
    signal n_slev : unsigned(6 downto 0) := (others => '0');
    signal n_slen : unsigned(8 downto 0) := (others => '0');
    signal n_son, n_trow : std_logic := '0';
    signal n_kbl  : unsigned(7 downto 0) := (others => '0');
    -- band knot loop: c = 8 s + 2 b + half (segment s, band b, knot s + half)
    signal e_brun : std_logic := '0';
    signal e_bc, e_bd : unsigned(6 downto 0) := (others => '0');
    signal bv     : std_logic_vector(6 downto 0) := (others => '0');
    signal b_kn1, b_kn2 : unsigned(3 downto 0) := (others => '0');
    signal b_six  : unsigned(4 downto 0) := (others => '0');
    signal b_ycb, b_ycw : signed(12 downto 0) := (others => '0');
    signal b_wv   : signed(7 downto 0) := (others => '0');
    signal b_hh, b_hh2, b_hh3 : signed(10 downto 0) := (others => '0');
    signal b_g, b_g2, b_g3 : unsigned(5 downto 0) := (others => '0');
    signal b_dy   : signed(12 downto 0) := (others => '0');
    signal b_e, b_eh : signed(12 downto 0) := (others => '0');
    signal b_gs, b_gh : unsigned(7 downto 0) := (others => '0');
    signal b_bs : signed(13 downto 0) := (others => '0');
    signal b_be0, b_be1 : signed(12 downto 0) := (others => '0');
    signal b_bg0, b_bg1 : unsigned(7 downto 0) := (others => '0');
    signal b_wr : std_logic := '0';
    signal b_ws : unsigned(3 downto 0) := (others => '0');

    -- beam and line registers
    signal b_hx  : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_hx6, b_hx7 : unsigned(11 downto 0) := (others => '0');   -- h of the stage-6 / stage-7 pixel
    signal l_wh, l_gwh : signed(12 downto 0) := (others => '0');
    signal l_wlh  : unsigned(11 downto 0) := (others => '0');
    signal l_slev : unsigned(6 downto 0) := (others => '0');
    signal l_slen : unsigned(8 downto 0) := (others => '0');
    signal l_son, l_trow, l_bank : std_logic := '0';
    signal l_kbl  : unsigned(7 downto 0) := (others => '0');

    -- pixel path
    type t_u12 is array (natural range <>) of unsigned(11 downto 0);
    type t_u7 is array (natural range <>) of unsigned(6 downto 0);
    type t_u9 is array (natural range <>) of unsigned(8 downto 0);
    signal ph : t_u12(1 to 4) := (others => (others => '0'));       -- h per stage
    signal pr : std_logic_vector(2 to 12) := (others => '0');       -- right side per stage
    signal p1_dx : signed(12 downto 0) := (others => '0');
    signal p2_d0, p3_d0 : unsigned(10 downto 0) := (others => '0');
    signal p3_dt : signed(11 downto 0) := (others => '0');
    signal p3_xm : unsigned(11 downto 0) := (others => '0');
    signal p4_d  : unsigned(11 downto 0) := (others => '0');
    signal p4_ff, p5_ff, p6_ff, p7_ff : unsigned(5 downto 0) := (others => '0');
    signal p5_fb, p5_flb, p5_fg : signed(13 downto 0) := (others => '0');
    signal p5_et : signed(12 downto 0) := (others => '0');
    signal p6_f, p6_fl, p6_fg : signed(13 downto 0) := (others => '0');
    signal p6_kb : std_logic_vector(35 downto 0) := (others => '0');
    signal p7_nf : unsigned(12 downto 0) := (others => '0');
    signal p7_m  : unsigned(6 downto 0) := (others => '0');
    signal p7_s, p8_s : unsigned(2 downto 0) := (others => '0');
    signal p7_de : signed(10 downto 0) := (others => '0');
    signal p7_dg : signed(8 downto 0) := (others => '0');
    signal p7_e0, p8_e0 : signed(9 downto 0) := (others => '0');
    signal p7_g0, p8_g0 : unsigned(7 downto 0) := (others => '0');
    signal p7_rt, p7_gt : unsigned(8 downto 0) := (others => '0');
    signal p7_id, p8_id : std_logic := '0';
    signal p7_tpx : unsigned(12 downto 0) := (others => '0');
    signal p8_pm : unsigned(19 downto 0) := (others => '0');
    signal p8_pe : signed(17 downto 0) := (others => '0');
    signal p8_pg : signed(13 downto 0) := (others => '0');
    signal p8_rim, p8_lg, p8_ls : unsigned(6 downto 0) := (others => '0');
    signal p8_tn : std_logic := '0';
    signal p8_tpx : unsigned(12 downto 0) := (others => '0');
    signal p8_th : unsigned(2 downto 0) := (others => '0');
    signal p9_e  : unsigned(24 downto 0) := (others => '0');
    signal p9_ee : signed(10 downto 0) := (others => '0');
    signal p9_gg : unsigned(7 downto 0) := (others => '0');
    signal p9_tp : unsigned(4 downto 0) := (others => '0');
    signal p9_th : unsigned(2 downto 0) := (others => '0');
    signal p10_fr : unsigned(7 downto 0) := (others => '0');
    signal p10_inb : std_logic := '0';
    signal p10_em : unsigned(5 downto 0) := (others => '0');
    signal p10_gcl, p11_gcl : unsigned(3 downto 0) := (others => '0');
    signal p10_gg : unsigned(7 downto 0) := (others => '0');
    signal p10_ne2, p11_ne2, p12_ne2 : unsigned(11 downto 0) := (others => '0');
    signal p10_thtp : unsigned(6 downto 0) := (others => '0');
    signal p11_dc : unsigned(6 downto 0) := (others => '0');
    signal p11_lb : unsigned(13 downto 0) := (others => '0');
    signal p11_tht : unsigned(10 downto 0) := (others => '0');
    signal p12_levb : unsigned(6 downto 0) := (others => '0');
    signal p12_tl : unsigned(14 downto 0) := (others => '0');
    signal p13_pre : unsigned(8 downto 0) := (others => '0');
    signal p13_ln : std_logic := '0';
    signal p15_lv : unsigned(6 downto 0) := (others => '0');
    -- delay chains
    signal dmr   : t_u7(9 to 15) := (others => (others => '0'));   -- max(rim, ghost rim, streak)
    signal did   : std_logic_vector(9 to 12) := (others => '0');   -- idol / ghost
    signal dtn   : std_logic_vector(9 to 15) := (others => '0');   -- tint
    signal dlok  : std_logic_vector(10 to 15) := (others => '0');  -- contour line present
    signal ddc   : t_u7(12 to 12) := (others => (others => '0'));  -- distance to the line centre
    signal dinb  : std_logic_vector(11 to 12) := (others => '0');  -- inside a band
    signal dpre  : t_u9(14 to 15) := (others => (others => '0'));  -- band / idol / tooth palette address
    signal dln   : std_logic_vector(14 to 15) := (others => '0');  -- use the line / rim level
    signal dpa   : t_u9(17 to 19) := (others => (others => '0'));  -- palette address pad
    signal p21_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p21_u, p21_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- chroma box-filter delay (8 back) in two 256 x 16 block-RAM rings: read pointer = write - 6, RAM register,
    -- fabric register; syncs delayed C_LAT in a 256 x 4 ring (write at rc + C_LAT - 2)
    type t_cring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal cru, crv : t_cring := (others => (others => '0'));
    signal r_cru, r_crv : std_logic_vector(15 downto 0) := (others => '0');
    signal s_csu7, s_csv7 : unsigned(9 downto 0) := to_unsigned(512, 10);
    attribute no_rw_check of cru : signal is true;
    attribute no_rw_check of crv : signal is true;
    signal rc : unsigned(7 downto 0) := (others => '0');
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y22, s_y23, s_y24, s_y25, s_y26, s_y27x, s_yf27 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_sring := (others => (others => '1'));
    signal r_sr, s_so : std_logic_vector(3 downto 0) := (others => '1');
    attribute no_rw_check of sr : signal is true;

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

    p_et : process(clk)
    begin
        if rising_edge(clk) then
            if et_we = '1' then etr(to_integer(et_wa)) <= et_wd; end if;
            r_et <= etr(to_integer(et_ra));
        end if;
    end process p_et;

    p_lv : process(clk)
    begin
        if rising_edge(clk) then
            if lv_we = '1' then lvr(to_integer(lv_wa)) <= lv_wd; end if;
            r_lv <= lvr(to_integer(lv_ra));
        end if;
    end process p_lv;

    p_kb : process(clk)
    begin
        if rising_edge(clk) then
            if kb_we = '1' then kbr(to_integer(kb_wa)) <= kb_wd; end if;
            r_kb <= kbr(to_integer(kb_ra));
        end if;
    end process p_kb;

    p_rom : process(clk)
        variable v_xa : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- the sequencer owns the EXR port in the first 512 zone clocks, the line engine otherwise
            if s_zone = '1' and s_zc(10 downto 9) = "00" then v_xa := r_ex_a; else v_xa := e_exa; end if;
            r_tab_q <= C_EF_TAB(to_integer(r_tab_a));
            r_kn    <= C_EF_KNOT(to_integer("00" & kr_a));
            q_kn    <= r_kn;
            r_pal_q <= C_EF_PAL(to_integer(r_pal_a));
            r_ex_q  <= C_EF_EXR(to_integer(v_xa));
            r_rnd_q <= C_EF_RND(to_integer(r_rnd_a));
        end if;
    end process p_rom;

    -- chroma rings and the sync ring
    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            cru(to_integer(rc + 6)) <= "000000" & std_logic_vector(p21_u);
            crv(to_integer(rc + 6)) <= "000000" & std_logic_vector(p21_v);
            r_cru <= cru(to_integer(rc));
            r_crv <= crv(to_integer(rc));
            s_csu7 <= unsigned(r_cru(9 downto 0));
            s_csv7 <= unsigned(r_crv(9 downto 0));
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            r_sr <= sr(to_integer(rc));
            s_so <= r_sr;
            rc <= rc + 1;
        end if;
    end process p_cring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_il : unsigned(11 downto 0);
        variable v_t  : unsigned(11 downto 0);
        variable v_lv : signed(15 downto 0);
        variable v_et : signed(17 downto 0);
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
            -- K3 Speed: bipolar, deadband 8
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
                f_mir  <= registers_in(6)(0);
                f_jade <= registers_in(6)(1);
                f_twin <= registers_in(6)(2);
                f_wave <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(816, 10);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(612, 10);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(408, 10);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 21) * to_unsigned(484, 9), 8), 12);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g(0); s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g(1); s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g(2); s_gr <= unsigned(registers_in(3));
                            when 3      => s_gm <= s_g(3); s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g(4); s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_g(5); s_gr <= unsigned(registers_in(7));
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
                            when 0      => s_g(0) <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g(1) <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g(2) <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g(3) <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g(4) <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_g(5) <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k decodes at zc = 62 + 24 k, pre-muxes its operands at 63 + 24 k, loads at
            -- 64 + 24 k; 16 iterations; the result is stored at 82 + 24 k
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 62  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 278 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 302 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 326 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 350 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 398 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 422 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 446 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 470 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 494 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 518 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when 566 => z_sel <= to_unsigned(21, 5); z_pre <= '1';
                    when 590 => z_sel <= to_unsigned(22, 5); z_pre <= '1';
                    when 614 => z_sel <= to_unsigned(23, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(243, 18);           z_fb <= z_m;                      -- stretch
                    when 1  => z_fa <= to_unsigned(529, 18);           z_fb <= z_m;                      -- 1 / lambda (left)
                    when 2  => z_fa <= resize(c_ill, 18);              z_fb <= to_unsigned(227, 16);     -- right = 0.443 left
                    when 3  => z_fa <= resize(s_g(4)(12 downto 3), 18); z_fb <= resize(s_g(4)(12 downto 3), 16); -- etch^2
                    when 4  => z_fa <= resize(z_q, 18);                z_fb <= to_unsigned(698, 16);
                    when 5  =>                                                                            -- Burn: lines
                        z_fa <= resize(z_q, 18);
                        if z_lo = '1' then z_fb <= to_unsigned(128, 16); else z_fb <= to_unsigned(171, 16); end if;
                    when 6  =>                                                                            -- halo
                        z_fa <= resize(z_q, 18);
                        if z_lo = '1' then z_fb <= to_unsigned(104, 16); else z_fb <= to_unsigned(128, 16); end if;
                    when 7  =>                                                                            -- teeth
                        z_fa <= resize(z_q, 18);
                        if z_lo = '1' then z_fb <= to_unsigned(160, 16); else z_fb <= to_unsigned(214, 16); end if;
                    when 8  =>                                                                            -- line width
                        z_fa <= resize(z_q, 18);
                        if z_lo = '1' then z_fb <= (others => '0'); else z_fb <= to_unsigned(19, 16); end if;
                    when 9  => z_fa <= resize(c_sa, 18);               z_fb <= to_unsigned(28, 16);      -- dim
                    when 10 => z_fa <= resize(c_sa, 18);               z_fb <= to_unsigned(48, 16);      -- bright
                    when 11 => z_fa <= resize(c_sa, 18);               z_fb <= to_unsigned(72, 16);      -- orange
                    when 12 => z_fa <= resize(z_sac, 18);              z_fb <= to_unsigned(77, 16);      -- off (lit by Burn)
                    when 14 => z_fa <= resize(c_a0, 18);               z_fb <= resize(z_r0, 16);
                    when 15 => z_fa <= resize(c_a1, 18);               z_fb <= resize(z_r1, 16);
                    when 16 => z_fa <= resize(c_a2, 18);               z_fb <= resize(z_r2, 16);
                    when 17 => z_fa <= resize(c_a3, 18);               z_fb <= resize(z_r3, 16);
                    when 18 => z_fa <= resize(z_sac, 18);              z_fb <= to_unsigned(30, 16);      -- streaks
                    when 19 => z_fa <= resize(z_sac, 18);              z_fb <= to_unsigned(70, 16);
                    when 21 => z_fa <= resize(s_vm, 18);               z_fb <= resize(s_vm, 16);         -- speed^2
                    when 22 => z_fa <= f_acc(17 downto 0);             z_fb <= to_unsigned(4475, 16);
                    when others => z_fa <= resize(c_sc(17 downto 8), 18); z_fb <= to_unsigned(270, 16);  -- scroll rows
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 31); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(29 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps)
                    when 4 =>
                        if c_spn = '1' then c_sc <= c_sc - resize(c_spd, 18); else c_sc <= c_sc + resize(c_spd, 18); end if;
                        c_phi <= c_phi + 17;
                        if c_bp = 479 then c_bp <= (others => '0'); else c_bp <= c_bp + 1; end if;
                        s_ot <= s_ot + 1;
                    when 5 =>
                        if c_bp >= 240 then z_ba <= resize(c_bp - 240, 8); else z_ba <= resize(240 - c_bp, 8); end if;
                    when 6 => z_bt <= resize(z_ba, 10) + shift_left(resize(z_ba, 10), 1);
                    when 7 => c_bob <= signed(resize(z_bt(9 downto 4), 7)) - 22;
                    -- Burn: below / above the still's 410
                    when 20 =>
                        v_g := s_g(5)(12 downto 3);
                        if v_g < 410 then z_lo <= '1'; z_q <= v_g; else z_lo <= '0'; z_q <= v_g - 410; end if;
                    -- K5 Bands: (k - 512) / 4 below the still, (k - 512) * 5 / 32 above
                    when 21 => z_d2 <= signed(resize(s_g(3)(12 downto 3), 11)) - 512;
                    when 22 =>
                        if z_d2(10) = '0' then
                            v_t := resize(unsigned(z_d2(9 downto 0)), 12) + shift_left(resize(unsigned(z_d2(9 downto 0)), 12), 2);
                            c_thick <= signed(resize(v_t(11 downto 5), 9));
                        else
                            c_thick <= resize(shift_right(z_d2, 2), 9);
                        end if;
                    -- exponents: K1 (width) +-2 octaves, K2 (stretch) x1.3125, K4 (contours) negated
                    when 38 =>
                        v_e := signed(resize(s_g(0)(12 downto 3), 11)) - 512;
                        z_sh1 <= v_e(10 downto 8); z_f1 <= unsigned(v_e(7 downto 0));
                        z_d2 <= signed(resize(s_g(1)(12 downto 3), 11)) - 512;
                        v_e := 512 - signed(resize(s_g(2)(12 downto 3), 11));
                        if v_e > 511 then v_e := to_signed(511, 11); end if;
                        z_sh4 <= v_e(10 downto 8); z_f4 <= unsigned(v_e(7 downto 0));
                    when 39 =>
                        if z_d2 < 0 then v_e := -z_d2; v_m := unsigned(v_e(9 downto 0)); else v_m := unsigned(z_d2(9 downto 0)); end if;
                        z_e2 <= signed(resize(v_m + shift_right(v_m, 2), 11));
                        z_hx <= resize(shift_right(v_m, 4), 7);
                    when 40 =>
                        v_e := z_e2 + signed(resize(z_hx, 11));
                        if z_d2 < 0 then v_e := -v_e; end if;
                        z_sh2 <= v_e(10 downto 8); z_f2 <= unsigned(v_e(7 downto 0));
                    when 41 => r_ex_a <= resize(z_f1, 9);
                    when 43 => z_m <= r_ex_q;
                    when 44 =>
                        case to_integer(z_sh1) is
                            when -2     => c_wg <= resize(z_m(15 downto 10), 10);
                            when -1     => c_wg <= resize(z_m(15 downto 9), 10);
                            when 0      => c_wg <= resize(z_m(15 downto 8), 10);
                            when others => c_wg <= resize(z_m(15 downto 7), 10);
                        end case;
                        r_ex_a <= resize(z_f2, 9);
                    when 46 => z_m <= r_ex_q;                   -- 2^frac (stretch), for slot 0
                    when 70 => r_ex_a <= resize(z_f4, 9);
                    when 72 => z_m <= r_ex_q;                   -- 2^frac (contours), for slot 1
                    -- slot stores
                    when 82 =>                        -- St = 243 * m >> (15 - sh2), sh2 = -3 .. 2
                        case to_integer(z_sh2) is
                            when -3     => c_st <= resize(f_acc(23 downto 18), 11);
                            when -2     => c_st <= resize(f_acc(23 downto 17), 11);
                            when -1     => c_st <= resize(f_acc(23 downto 16), 11);
                            when 0      => c_st <= resize(f_acc(23 downto 15), 11);
                            when 1      => c_st <= resize(f_acc(23 downto 14), 11);
                            when others => c_st <= f_acc(23 downto 13);
                        end case;
                    when 106 =>                       -- IL = 529 * m >> (15 - sh4), sh4 = -2 .. 1
                        case to_integer(z_sh4) is
                            when -2     => c_ill <= resize(f_acc(25 downto 17), 12);
                            when -1     => c_ill <= resize(f_acc(25 downto 16), 12);
                            when 0      => c_ill <= resize(f_acc(25 downto 15), 12);
                            when others => c_ill <= f_acc(25 downto 14);
                        end case;
                    when 108 | 132 =>                 -- 7-bit mantissa and shift of 1 / lambda
                        if s_zc(7) = '0' then v_il := c_ill; else v_il := c_ilr; end if;   -- 108: left, 132: right
                        if v_il(11) = '1' then    v_m := resize(v_il(11 downto 5), 10); v_u := to_unsigned(5, 11);
                        elsif v_il(10) = '1' then v_m := resize(v_il(10 downto 4), 10); v_u := to_unsigned(4, 11);
                        elsif v_il(9) = '1' then  v_m := resize(v_il(9 downto 3), 10);  v_u := to_unsigned(3, 11);
                        elsif v_il(8) = '1' then  v_m := resize(v_il(8 downto 2), 10);  v_u := to_unsigned(2, 11);
                        elsif v_il(7) = '1' then  v_m := resize(v_il(7 downto 1), 10);  v_u := to_unsigned(1, 11);
                        else                      v_m := resize(v_il(6 downto 0), 10);  v_u := to_unsigned(0, 11); end if;
                        if s_zc(7) = '0' then c_ml <= v_m(6 downto 0); c_sl <= v_u(2 downto 0);
                        else c_mr <= v_m(6 downto 0); c_sr <= v_u(2 downto 0); end if;
                    when 130 => c_ilr <= f_acc(20 downto 9);
                    when 154 => z_q <= f_acc(19 downto 10);
                    when 178 => c_eg <= f_acc(19 downto 10);
                    when 180 =>
                        v_g := s_g(5)(12 downto 3);
                        if v_g < 410 then z_q <= v_g; else z_q <= v_g - 410; end if;
                    when 202 => if z_lo = '1' then c_sa <= 51 + f_acc(17 downto 8); else c_sa <= 256 + f_acc(17 downto 8); end if;
                    when 226 => if z_lo = '1' then c_shs <= 90 + f_acc(17 downto 8); else c_shs <= 256 + f_acc(17 downto 8); end if;
                    when 227 => c_sh5 <= c_shs(9 downto 5);
                    when 250 => if z_lo = '1' then c_stt <= f_acc(17 downto 8); else c_stt <= 256 + f_acc(17 downto 8); end if;
                    when 251 =>
                        v_t := resize(c_stt, 12) + shift_left(resize(c_stt, 12), 2);
                        if v_t(11 downto 8) > 15 then c_t4 <= (others => '1'); else c_t4 <= v_t(11 downto 8); end if;
                    when 274 => c_hwa <= f_acc(13 downto 8);
                    when 298 => if f_acc(16 downto 15) /= "00" then c_a1 <= (others => '1'); else c_a1 <= f_acc(14 downto 8); end if;
                    when 322 => if f_acc(16 downto 15) /= "00" then c_a2 <= (others => '1'); else c_a2 <= f_acc(14 downto 8); end if;
                    when 346 =>
                        if f_acc(16 downto 15) /= "00" then c_a3 <= (others => '1'); else c_a3 <= f_acc(14 downto 8); end if;
                        if c_sa > 307 then z_sac <= c_sa - 307; else z_sac <= (others => '0'); end if;
                    when 370 =>
                        if f_acc(16 downto 15) /= "00" then c_a0 <= (others => '1'); else c_a0 <= f_acc(14 downto 8); end if;
                        if c_sa > 333 then z_sac <= to_unsigned(333, 10); else z_sac <= c_sa; end if;
                    -- line width per class: hw = 23 + (x >> 2) + (x >> 3) + (x >> 5) + hwA, x = max(A - 25, 0)
                    when 371 | 375 | 379 | 383 =>
                        case to_integer(s_zc) is
                            when 371    => v_m := resize(c_a0, 10);
                            when 375    => v_m := resize(c_a1, 10);
                            when 379    => v_m := resize(c_a2, 10);
                            when others => v_m := resize(c_a3, 10);
                        end case;
                        if v_m > 25 then z_hx <= resize(v_m - 25, 7); else z_hx <= (others => '0'); end if;
                    when 372 | 376 | 380 | 384 =>
                        z_ht <= resize(shift_right(z_hx, 2), 8) + resize(shift_right(z_hx, 3), 8);
                    when 373 | 377 | 381 | 385 =>
                        z_ht <= z_ht + resize(shift_right(z_hx, 5), 8) + 23;
                    when 374 | 378 | 382 | 386 =>
                        v_u := resize(z_ht, 11) + resize(c_hwa, 11);
                        if v_u > 127 then v_m := to_unsigned(127, 10); else v_m := v_u(9 downto 0); end if;
                        case to_integer(s_zc) is
                            when 374    => z_hw0 <= v_m(6 downto 0);
                            when 378    => z_hw1 <= v_m(6 downto 0);
                            when 382    => z_hw2 <= v_m(6 downto 0);
                            when others => z_hw3 <= v_m(6 downto 0);
                        end case;
                    when 387 => r_ex_a <= '1' & resize(z_hw0, 8);
                    when 388 => r_ex_a <= '1' & resize(z_hw1, 8);
                    when 389 => r_ex_a <= '1' & resize(z_hw2, 8); z_r0 <= r_ex_q(7 downto 0);
                    when 390 => r_ex_a <= '1' & resize(z_hw3, 8); z_r1 <= r_ex_q(7 downto 0);
                    when 391 => z_r2 <= r_ex_q(7 downto 0);
                    when 392 => z_r3 <= r_ex_q(7 downto 0);
                    when 418 => c_k0 <= f_acc(14 downto 7);
                    when 442 => c_k1 <= f_acc(14 downto 7);
                    when 466 => c_k2 <= f_acc(14 downto 7);
                    when 490 => c_k3 <= f_acc(14 downto 7);
                    when 514 => c_s30 <= f_acc(14 downto 8);
                    when 538 => c_s70 <= f_acc(14 downto 8);
                    when 610 => c_spd <= f_acc(30 downto 18); c_spn <= s_vneg;
                    when 634 => c_srow <= f_acc(18 downto 8);
                    when others => null;
                end case;
            end if;

            -- line-level table (zc 640..1151): per class c and distance d, clip((16 A - d K / 2) >> 3, 0, A)
            lv_we <= '0';
            if s_zone = '1' and s_zc >= 640 and s_zc < 1152 then
                v_u := resize(s_zc - 640, 11);
                case to_integer(v_u(8 downto 7)) is
                    when 0      => v_m := resize(c_a0, 10); v_g := resize(c_k0(7 downto 1), 10);
                    when 1      => v_m := resize(c_a1, 10); v_g := resize(c_k1(7 downto 1), 10);
                    when 2      => v_m := resize(c_a2, 10); v_g := resize(c_k2(7 downto 1), 10);
                    when others => v_m := resize(c_a3, 10); v_g := resize(c_k3(7 downto 1), 10);
                end case;
                if v_u(6 downto 0) = 0 then v_lv := signed(shift_left(resize(v_m, 16), 4)); else v_lv := lv_v; end if;
                lv_we <= '1'; lv_wa <= v_u(8 downto 0);
                if v_lv < 0 then lv_wd <= (others => '0');
                elsif shift_right(v_lv, 3) > signed(resize(v_m, 16)) then lv_wd <= std_logic_vector(resize(v_m, 8));
                else lv_wd <= std_logic_vector(resize(unsigned(shift_right(v_lv, 3)), 8)); end if;
                lv_v <= v_lv - signed(resize(v_g, 16));
            end if;
            -- etch table (zc 1160..1415): offset (h) for source sample s = -128 .. 127: (s * EG / 4) >> 4
            et_we <= '0';
            if s_zone = '1' and s_zc >= 1160 and s_zc < 1416 then
                v_u := resize(s_zc - 1160, 11);
                if v_u(7 downto 0) = 0 then v_et := (others => '0');
                elsif v_u(7 downto 0) = 128 then v_et := -signed(resize(shift_left(resize(c_eg(9 downto 2), 18), 7), 18));
                else v_et := et_v; end if;
                et_we <= '1'; et_wa <= v_u(7 downto 0);
                et_wd <= std_logic_vector(resize(shift_right(v_et, 4), 16));
                et_v <= v_et + signed(resize(c_eg(9 downto 2), 18));
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;


    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 2/3 clocks after the source's avid
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
    -- into the line buffer
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(6 downto 0);
        variable v_s : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 7); end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_lb_we <= data_in.avid and not s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
            if f_osc = '1' then
                s_lb_wd <= std_logic_vector(shift_right(signed(v_s xor x"80"), 1));   -- (tri + tri - 128) / 2
            else
                v_l := unsigned(data_in.y(9 downto 3));
                if v_l > 8 then s_lb_wd <= '0' & std_logic_vector(v_l - 8); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (12 iterations, result at
    -- load + 13), then the band knots into the knot buffer
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t  : unsigned(21 downto 0);
        variable v_v  : signed(12 downto 0);
        variable v_w  : unsigned(8 downto 0);
        variable v_d  : signed(12 downto 0);
        variable v_sb : signed(13 downto 0);
        variable v_e0, v_e1 : signed(12 downto 0);
        variable v_gp : unsigned(10 downto 0);
        variable v_kn : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1450 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_bank <= '0';
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_bank <= not e_bank;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(20 downto 0) & '0';
            m_b <= '0' & m_b(11 downto 1);

            case to_integer(e_lc) is
                when 1 => e_y <= e_vy(18 downto 8);
                when 2 => m_a <= resize(e_y, 22); m_b <= resize(c_st, 12); m_acc <= (others => '0');
                when 15 =>
                    v_t := m_acc + resize(c_sc, 22);
                    r_tab_a <= v_t(17 downto 8);
                    e_v0 <= resize(e_y, 12) + resize(c_srow, 12) + 188;
                    e_ys <= resize(e_y, 12) + resize(c_srow, 12);
                when 16 =>
                    if e_v0 >= 2160 then e_v0 <= e_v0 - 2160; elsif e_v0 >= 1080 then e_v0 <= e_v0 - 1080; end if;
                when 17 =>
                    -- table word: {idol half-width + 32, contour half-width}; (w + 32) Wg - 32 Wg
                    e_wl <= unsigned(r_tab_q(8 downto 0));
                    m_a <= resize(unsigned(r_tab_q(17 downto 9)), 22);
                    m_b <= resize(c_wg, 12); m_acc <= (others => '0');
                    -- ghost: row offset from its centre
                    v_v := signed(resize(e_v0, 13)) - 540;
                    if v_v < 0 then v_d := -v_v; else v_d := v_v; end if;
                    if v_d <= 48 then e_gon <= '1'; else e_gon <= '0'; end if;
                    e_va <= unsigned(v_d(5 downto 0));
                when 30 =>
                    n_wh <= signed(resize(m_acc(18 downto 6), 13)) - signed(resize(c_wg(9 downto 1), 13));
                    m_a <= resize(e_wl, 22); m_b <= resize(c_wg, 12); m_acc <= (others => '0');
                when 40 => e_exa <= "111" & e_va;                                  -- ghost half-width (EXR 448 + |v|)
                when 43 =>
                    n_wlh <= m_acc(17 downto 6);
                    m_a <= resize(r_ex_q(7 downto 0), 22); m_b <= resize(c_wg, 12); m_acc <= (others => '0');
                when 56 =>
                    if e_gon = '1' then n_gwh <= signed(resize(m_acc(16 downto 6), 13));
                    else n_gwh <= to_signed(-2048, 13); end if;
                    m_a <= resize(e_ys, 22); m_b <= to_unsigned(1456, 12); m_acc <= (others => '0');
                when 69 =>
                    -- streak: index (rows + scroll) / 45 (EXR 384 + index), phase within it
                    e_exa <= "110" & m_acc(21 downto 16);
                    if m_acc(15 downto 8) < 61 then n_son <= '1'; else n_son <= '0'; end if;
                when 71 => e_st <= r_ex_q(5 downto 0);
                when 72 =>
                    case to_integer(e_st(5 downto 4)) is
                        when 0      => n_slev <= (others => '0');
                        when 1      => n_slev <= c_s30;
                        when others => n_slev <= c_s70;
                    end case;
                    v_w := shift_left(resize(e_st(3 downto 0), 9), 3) - resize(e_st(3 downto 0), 9) + 58;
                    n_slen <= v_w(7 downto 0) & '0';
                    m_a <= resize(e_y, 22); m_b <= to_unsigned(1365, 12); m_acc <= (others => '0');
                when 85 =>
                    e_blk <= m_acc(21 downto 16);
                    if e_y < 580 then n_trow <= '1'; else n_trow <= '0'; end if;
                when 86 => e_kbt <= shift_left(resize(e_blk, 8), 5) + shift_left(resize(e_blk, 8), 3);
                when 87 => e_kbt <= e_kbt + shift_left(resize(e_blk, 8), 1);
                when 88 => n_kbl <= e_kbt + resize(e_blk, 8);
                when others => null;
            end case;

            -- band knots: c = 8 s + 2 b + half; ROM index b * 16 + (s + half)
            if e_lc = 100 then
                e_brun <= '1'; e_bc <= (others => '0');
            elsif e_brun = '1' then
                if e_bc = 127 then e_brun <= '0'; end if;
                e_bc <= e_bc + 1;
            end if;
            v_kn := e_bc(6 downto 3) + resize(e_bc(0 downto 0), 4);
            kr_a <= e_bc(2 downto 1) & v_kn;
            b_kn1 <= v_kn; b_kn2 <= b_kn1;
            bv <= bv(5 downto 0) & e_brun;
            -- 2 (ROM data in q_kn): sine index; 3: centre + bob, half-height + Bands, wave; 4: + wave;
            -- 5: dy; 6: depth e and scaled halo gain; 7: segments
            b_six <= resize(shift_left(resize(b_kn2, 6), 2) + resize(b_kn2(3 downto 2), 6), 5) + c_phi(12 downto 8);
            b_ycb <= signed(resize(unsigned(q_kn(25 downto 15)), 13)) + resize(c_bob, 13);
            b_hh  <= resize(signed(q_kn(14 downto 6)), 11) + resize(c_thick, 11);
            b_g   <= unsigned(q_kn(5 downto 0));
            if f_wave = '1' then b_wv <= C_EF_SIN(to_integer(b_six)); else b_wv <= (others => '0'); end if;
            b_ycw <= b_ycb + resize(b_wv, 13);
            b_hh2 <= b_hh; b_g2 <= b_g;
            b_dy  <= signed(resize(e_y, 13)) - b_ycw;
            b_hh3 <= b_hh2; b_g3 <= b_g2;
            if b_dy < 0 then v_d := -b_dy; else v_d := b_dy; end if;
            b_e  <= resize(b_hh3, 13) - v_d;
            v_gp := resize(b_g3 * c_sh5, 11);
            b_gs <= v_gp(9 downto 2);
            if e_lc = 100 then
                e_bd <= (others => '0');
            elsif bv(6) = '1' then
                e_bd <= e_bd + 1;
            end if;
            b_wr <= '0';
            if bv(6) = '1' then
                if e_bd(0) = '0' then
                    b_eh <= b_e; b_gh <= b_gs;
                else
                    v_sb := resize(b_eh, 14) + resize(b_e, 14);
                    if e_bd(2 downto 1) = 0 or v_sb > b_bs then
                        b_bs <= v_sb; b_be0 <= b_eh; b_be1 <= b_e; b_bg0 <= b_gh; b_bg1 <= b_gs;
                    end if;
                    if e_bd(2 downto 1) = 3 and e_bd(6 downto 3) /= 15 then b_wr <= '1'; end if;
                end if;
            end if;
            b_ws <= e_bd(6 downto 3);
            kb_we <= '0';
            if b_wr = '1' then
                kb_we <= '1';
                kb_wa <= e_bank & b_ws;
                v_e0 := b_be0; v_e1 := b_be1;
                if v_e0 > 511 then v_e0 := to_signed(511, 13); elsif v_e0 < -512 then v_e0 := to_signed(-512, 13); end if;
                if v_e1 > 511 then v_e1 := to_signed(511, 13); elsif v_e1 < -512 then v_e1 := to_signed(-512, 13); end if;
                kb_wd <= std_logic_vector(v_e0(9 downto 0)) & std_logic_vector(v_e1(9 downto 0)) &
                         std_logic_vector(b_bg0) & std_logic_vector(b_bg1);
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: h position and pixel index; the line registers load at g_start.
    -- b_hx6 follows the stage-6 pixel (restarted by the generated avid's
    -- delayed edge), b_hx7 the stage-7 one.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_hx <= resize(s_x0, 12);
                b_px <= (others => '0');
                l_wh <= n_wh; l_wlh <= n_wlh; l_gwh <= n_gwh;
                l_slev <= n_slev; l_slen <= n_slen; l_son <= n_son; l_trow <= n_trow;
                l_kbl <= n_kbl; l_bank <= e_bank;
            elsif g_avid = '1' then
                b_hx <= b_hx + s_xs;
                b_px <= b_px + 1;
            end if;
            if s_gav_sr(4) = '1' and s_gav_sr(5) = '0' then
                b_hx6 <= resize(s_x0, 12);
            elsif s_gav_sr(4) = '1' then
                b_hx6 <= b_hx6 + s_xs;
            end if;
            b_hx7 <= b_hx6;
            s_lb_ra <= b_px(10 downto 1);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = the beam registers)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a   : signed(13 downto 0);
        variable v_r   : signed(14 downto 0);
        variable v_x   : unsigned(12 downto 0);
        variable v_c   : unsigned(1 downto 0);
        variable v_l   : signed(9 downto 0);
        variable v_m   : unsigned(6 downto 0);
        variable v_g   : signed(9 downto 0);
        variable v_d   : unsigned(7 downto 0);
        variable v_u7  : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: offset from the axis
            p1_dx <= signed(resize(b_hx, 13)) - C_XC;
            ph(1) <= b_hx;
            for i in 2 to 4 loop ph(i) <= ph(i - 1); end loop;
            -- 2: |x - axis|, side
            if p1_dx < 0 then v_a := resize(-p1_dx, 14); else v_a := resize(p1_dx, 14); end if;
            p2_d0 <= unsigned(v_a(10 downto 0));
            pr(2) <= f_mir or not p1_dx(12);
            for i in 3 to 12 loop pr(i) <= pr(i - 1); end loop;
            -- 3: twin offset; knot x (mirrored: axis + |x - axis|); source sample
            p3_dt <= signed(resize(p2_d0, 12)) - 960;
            p3_d0 <= p2_d0;
            if f_mir = '1' then
                v_x := resize(p2_d0, 13) + C_XC;
            else
                v_x := resize(ph(2), 13);
            end if;
            if v_x > 3839 then p3_xm <= to_unsigned(3839, 12); else p3_xm <= v_x(11 downto 0); end if;
            et_ra <= unsigned(s_lb_q);
            -- 4: d (twin: 2 |d - 960|); knot buffer address; etch product
            if f_twin = '1' then
                if p3_dt < 0 then p4_d <= unsigned(shift_left(resize(-p3_dt, 12), 1));
                else p4_d <= unsigned(shift_left(resize(p3_dt, 12), 1)); end if;
            else
                p4_d <= resize(p3_d0, 12);
            end if;
            kb_ra <= l_bank & p3_xm(11 downto 8);
            p4_ff <= p3_xm(7 downto 2);
            -- 5: the three fields before the etch
            p5_fb  <= resize(l_wh, 14) - signed(resize(p4_d, 14));
            p5_flb <= signed(resize(l_wlh, 14)) - signed(resize(p4_d, 14));
            p5_fg  <= resize(l_gwh, 14) - signed(resize(ph(4), 14));
            p5_et  <= signed(r_et(12 downto 0));
            p5_ff <= p4_ff;
            -- 6: + etch; knot data (fabric copy)
            p6_f  <= p5_fb + resize(p5_et, 14);
            p6_fl <= p5_flb + resize(p5_et, 14);
            p6_fg <= p5_fg;
            p6_kb <= r_kb;
            p6_ff <= p5_ff;
            -- 7: contour distance (outside the idol only), knot deltas, rim slopes, tooth saw
            if p6_fl < 0 then v_a := -p6_fl; p7_nf <= unsigned(v_a(12 downto 0)); else p7_nf <= (others => '0'); end if;
            if pr(6) = '1' then p7_m <= c_mr; p7_s <= c_sr; else p7_m <= c_ml; p7_s <= c_sl; end if;
            p7_e0 <= signed(p6_kb(35 downto 26));
            p7_de <= resize(signed(p6_kb(25 downto 16)), 11) - resize(signed(p6_kb(35 downto 26)), 11);
            p7_g0 <= unsigned(p6_kb(15 downto 8));
            p7_dg <= signed(resize(unsigned(p6_kb(7 downto 0)), 9)) - signed(resize(unsigned(p6_kb(15 downto 8)), 9));
            p7_ff <= p6_ff;
            -- rims: u = min(-F, 127) below the edge; right 1.5 u, left 2.5 u (unsigned: no duplicated sign bits)
            if p6_f >= 0 then v_u7 := (others => '0');
            elsif p6_f < -127 then v_u7 := (others => '1');
            else v_a := -p6_f; v_u7 := unsigned(v_a(6 downto 0)); end if;
            if pr(6) = '1' then
                p7_rt <= resize(v_u7, 9) + resize(shift_right(v_u7, 1), 9);
            else
                p7_rt <= shift_left(resize(v_u7, 9), 1) + resize(shift_right(v_u7, 1), 9);
            end if;
            if p6_fg >= 0 then v_u7 := (others => '0');
            elsif p6_fg < -127 then v_u7 := (others => '1');
            else v_a := -p6_fg; v_u7 := unsigned(v_a(6 downto 0)); end if;
            p7_gt <= resize(v_u7, 9) + resize(shift_right(v_u7, 1), 9);
            if p6_f > 0 or p6_fg > 0 then p7_id <= '1'; else p7_id <= '0'; end if;
            v_x := resize(b_hx6(6 downto 0), 13);
            p7_tpx <= shift_left(v_x, 6) + shift_left(v_x, 2);
            -- 8: products: contour phase (mantissa), knot lerps; rims, streak, tint, tooth saw / height
            p8_pm <= resize(p7_nf * p7_m, 20);
            p8_s <= p7_s;
            p8_pe <= resize(p7_de * signed('0' & p7_ff), 18);
            p8_pg <= resize(p7_dg * signed('0' & p7_ff(5 downto 2)), 14);
            p8_e0 <= p7_e0; p8_g0 <= p7_g0;
            if p7_rt >= 90 then p8_rim <= (others => '0'); else p8_rim <= resize(90 - p7_rt, 7); end if;
            if p7_gt >= 90 then p8_lg <= (others => '0'); else p8_lg <= resize(90 - p7_gt, 7); end if;
            p8_id <= p7_id;
            if l_son = '1' and b_hx7 < resize(l_slen, 12) then p8_ls <= l_slev; else p8_ls <= (others => '0'); end if;
            if f_jade = '1' or (l_trow = '1' and b_hx7 < 1640) then p8_tn <= '1'; else p8_tn <= '0'; end if;
            v_x := resize(b_hx7(6 downto 0), 13);
            p8_tpx <= p7_tpx + shift_left(v_x, 1) + v_x;
            p8_th <= 4 + resize(C_EF_TH(to_integer(b_hx7(11 downto 7))), 3);
            -- 9: contour phase, band depth and halo gain; tooth saw; max of the rims and the streak
            case to_integer(p8_s) is
                when 0      => p9_e <= resize(p8_pm, 25);
                when 1      => p9_e <= shift_left(resize(p8_pm, 25), 1);
                when 2      => p9_e <= shift_left(resize(p8_pm, 25), 2);
                when 3      => p9_e <= shift_left(resize(p8_pm, 25), 3);
                when 4      => p9_e <= shift_left(resize(p8_pm, 25), 4);
                when others => p9_e <= shift_left(resize(p8_pm, 25), 5);
            end case;
            p9_ee <= resize(p8_e0, 11) + resize(shift_right(p8_pe, 6), 11);
            v_g := signed(resize(p8_g0, 10)) + resize(shift_right(p8_pg, 4), 10);
            if v_g < 0 then p9_gg <= (others => '0'); else p9_gg <= unsigned(v_g(7 downto 0)); end if;
            if p8_tpx(12) = '1' then p9_tp <= (others => '0'); else p9_tp <= resize(16 - resize(p8_tpx(12 downto 8), 6), 5); end if;
            p9_th <= p8_th;
            v_m := p8_rim;
            if p8_lg > v_m then v_m := p8_lg; end if;
            if p8_ls > v_m then v_m := p8_ls; end if;
            dmr(9) <= v_m; did(9) <= p8_id; dtn(9) <= p8_tn;
            for i in 10 to 15 loop dmr(i) <= dmr(i - 1); dtn(i) <= dtn(i - 1); end loop;
            for i in 10 to 12 loop did(i) <= did(i - 1); end loop;
            -- 10: segment table address (contour index + line offset), phase within the period; band: inside,
            -- depth term, tooth gain and saw x height
            r_rnd_a <= p9_e(23 downto 16) + l_kbl;
            p10_fr  <= p9_e(15 downto 8);
            if p9_e > 32768 then dlok(10) <= '1'; else dlok(10) <= '0'; end if;
            for i in 11 to 15 loop dlok(i) <= dlok(i - 1); end loop;
            if p9_ee >= 0 then
                p10_inb <= '1';
                if p9_ee >= 64 then p10_em <= (others => '0'); else p10_em <= resize(shift_right(64 - unsigned(p9_ee(6 downto 0)), 1), 6); end if;
                p10_ne2 <= (others => '0');
            else
                p10_inb <= '0'; p10_em <= (others => '0');
                p10_ne2 <= unsigned(shift_left(resize(-p9_ee, 12), 1));
            end if;
            if p9_gg <= 16 then p10_gcl <= (others => '0');
            elsif p9_gg >= 48 then p10_gcl <= to_unsigned(8, 4);
            else p10_gcl <= resize(shift_right(p9_gg - 16, 2), 4); end if;
            p10_gg <= p9_gg;
            p10_thtp <= resize(p9_th * p9_tp, 7);
            -- 11: distance to the line centre; halo product; tooth scale
            if p10_fr >= 128 then v_d := p10_fr - 128; else v_d := 128 - p10_fr; end if;
            if v_d(7) = '1' then p11_dc <= (others => '1'); else p11_dc <= v_d(6 downto 0); end if;
            p11_lb  <= resize(p10_gg * p10_em, 14);
            p11_tht <= resize(p10_thtp * c_t4, 11);
            p11_gcl <= p10_gcl;
            p11_ne2 <= p10_ne2; p12_ne2 <= p11_ne2;
            dinb(11) <= p10_inb; dinb(12) <= dinb(11);
            -- 12: halo level; tooth length; segment class bits (fabric copy of the table)
            if p11_lb(13 downto 10) /= "0000" then p12_levb <= (others => '1'); else p12_levb <= p11_lb(9 downto 3); end if;
            p12_tl <= resize(p11_gcl * p11_tht, 15);
            p12_h <= r_rnd_q;
            ddc(12) <= p11_dc;
            -- 13: segment class -> level and slope; band / idol / tooth palette address
            if pr(12) = '1' then
                if p12_h(4 downto 0) < 11 then v_c := "00"; elsif p12_h(4 downto 0) < 21 then v_c := "01";
                elsif p12_h(4 downto 0) < 29 then v_c := "10"; else v_c := "11"; end if;
            else
                if p12_h(4 downto 0) < 10 then v_c := "00"; elsif p12_h(4 downto 0) < 27 then v_c := "01"; else v_c := "11"; end if;
            end if;
            lv_ra <= v_c & ddc(12);
            p13_ln <= '0';
            if dinb(12) = '1' then
                p13_pre <= '1' & f_jade & p12_levb;
            elsif did(12) = '1' then
                p13_pre <= "00" & to_unsigned(127, 7);
            elsif resize(p12_ne2, 15) < shift_right(p12_tl, 6) then
                p13_pre <= "00" & to_unsigned(71, 7);
            else
                p13_pre <= (others => '0'); p13_ln <= '1';
            end if;
            -- 14: line-level table (EBR); 15: fabric copy
            p15_lv <= unsigned(r_lv(6 downto 0));
            dpre(14) <= p13_pre; dpre(15) <= dpre(14);
            dln(14) <= p13_ln; dln(15) <= dln(14);
            -- 16: palette address (line / rim / streak level, or the band / idol / tooth address)
            if dln(15) = '1' then
                if dlok(15) = '1' and p15_lv > dmr(15) then v_m := p15_lv; else v_m := dmr(15); end if;
                dpa(17) <= '0' & dtn(15) & v_m;
            else
                dpa(17) <= dpre(15);
            end if;
            -- 17..19: pad; 20: palette EBR; 21: expand, neutral off the generated avid
            dpa(18) <= dpa(17); dpa(19) <= dpa(18);
            r_pal_a <= dpa(19);
            if s_gav_sr(19) = '1' then
                p21_y <= unsigned(r_pal_q(23 downto 16)) & "00";
                p21_u <= unsigned(r_pal_q(15 downto 8)) & "00";
                p21_v <= unsigned(r_pal_q(7 downto 0)) & "00";
            else
                p21_y <= to_unsigned(64, 10); p21_u <= to_unsigned(512, 10); p21_v <= to_unsigned(512, 10);
            end if;
            s_bxh <= s_bxh(6 downto 0) & s_gav_sr(19);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums,
    -- luma [1 2 1], luma 13/16 (white 940 -> 777), blanking gate.  The running
    -- 8-sums snap back to neutral once 8 blank samples are in.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(14 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_bxh = "00000000" then
                s_boxu <= to_unsigned(4096, 14); s_boxv <= to_unsigned(4096, 14);
            else
                s_boxu <= s_boxu + p21_u - s_csu7;
                s_boxv <= s_boxv + p21_v - s_csv7;
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u27 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v27 <= v_sc(13 downto 4);
            s_y22 <= p21_y; s_y23 <= s_y22; s_y24 <= s_y23; s_y25 <= s_y24;
            s_y26 <= s_y25; s_y27x <= s_y26;
            v_sy := resize(s_y25, 12) + shift_left(resize(s_y26, 12), 1) + resize(s_y27x, 12);
            s_yf27 <= v_sy(11 downto 2);
            -- 28: luma toward black (white 940 -> 777); blanking gate
            if s_yf27 > 64 then v_dl := s_yf27 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(26) = '1' then
                s_y28 <= s_yf27 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u28 <= s_u27; s_v28 <= s_v27;
            else
                s_y28 <= to_unsigned(64, 10); s_u28 <= to_unsigned(512, 10); s_v28 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= s_so(2);
    data_out.vsync_n <= s_so(1);
    data_out.field_n <= s_so(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture effigy;
