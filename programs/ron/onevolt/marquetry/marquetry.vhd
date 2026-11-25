-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Marquetry v0.1.0 -- Onevolt: the Syntonie shape rack.
--
-- The rack (cable by cable):
--   Rampes mirrored H, mirrored V (1 at the centre, 0 at the edges) -> Courbes ch 1, ch 2 (one curve, P12):
--        f(t) = t + c t (1 - t), c = -1 (expo: t^2) .. 0 (linear) .. +1 (log: 2t - t^2)
--   the two curved triangles -> the three Rampes figures: DIAMOND = average, RECTANGLE = minimum, CROSS = maximum
--   DIAMOND -> Seuils IN A, RECTANGLE -> IN B, CROSS -> IN C
--   Seuils SHIFT A / B / C knobs (K1, K2, K3); camera luma (8-px cells, vertical and horizontal slews) -> CV A,
--        normalled to B and C, through the CV attenuverters (+1, -1, +3/4; K6 = their common depth; S11 Osc);
--        Animate's 0 / 90 / 180 degree outputs (23 s) -> the CVs as well
--   Seuils channel: L = (in > thr) or the boolean the switch selects (S7 A&B, S8 A|B, S9 A^B);
--        out = clip(in - thr + (L ? 0 : 1)): with its own comparator a wrap, with a boolean flat 0 / 1 cuts
--   Seuils SHIFT outs -> Isohelie IN A, B, C (S10: Isohelie in the path / out of it)
--   V ramp -> Isohelie CV (normalled), BITSWAP knob K4 (+ a 13 s LFO): three horizontal zones of bit orders
--   Isohelie outs -> Matrice (K5 sweeps eight colourings, blended) -> Sortie
--
--   K1 Shift A  K2 Shift B  K3 Shift C  (one full cycle per knob turn; the ends meet)
--   K4 Bitswap  K5 Colour  K6 Camera  S7 A: Comp / A&B  S8 B: Comp / A|B  S9 C: Comp / A^B
--   S10 Isohelie In / Out  S11 Source Video / Osc  P12 Curve (expo star .. linear diamond .. log ellipse)
--
-- Engine: vblank-zone sequencer (one serial shift-add multiplier, 19 slots: Animate's parabolic sines, the curve,
-- the camera depth and the K5 colouring blend), then two table writers: the Courbes table f(t) (1024 x 16, second
-- differences, no multiplier) and the three Matrice tables (256 x 32 each: the YUV contribution of a channel level).
-- A per-line engine reads the curve at the V ramp (fy, its row-to-row slope for the band-edge coverage, the
-- bitswap zone); the pixel path reads it at the H ramp.  Generated avid; output conditioning (Y, U and V through
-- two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16, a vertical [1 1]/2 with the line above)
-- before the blanking gate.  Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.marquetry_pkg.all;
use work.all;

architecture marquetry of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

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
    -- V ramp step per field line: 2^24 over the field's height (ys x 2^24 / (1080 x 256))
    function f_incy(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(69056, 17);
            when 4          => return to_unsigned(34953, 17);
            when 8          => return to_unsigned(58254, 17);
            when 12         => return to_unsigned(29127, 17);
            when 5 | 6 | 14 => return to_unsigned(23302, 17);
            when 1 | 2 | 10 => return to_unsigned(31069, 17);
            when others     => return to_unsigned(15534, 17);
        end case;
    end function;
    -- the Osc lattice's line step: (ys x 121) >> 8
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(537, 10);
            when 4          => return to_unsigned(272, 10);
            when 8          => return to_unsigned(453, 10);
            when 12         => return to_unsigned(226, 10);
            when 5 | 6 | 14 => return to_unsigned(181, 10);
            when 1 | 2 | 10 => return to_unsigned(242, 10);
            when others     => return to_unsigned(121, 10);
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
    -- Isohelie's multiplexer: the eight bit orders (z = 0 the original)
    function f_perm(z : unsigned(2 downto 0); n : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(z) is
            when 0      => return n;
            when 1      => return n(2) & n(0) & n(1);
            when 2      => return n(1) & n(2) & n(0);
            when 3      => return n(0) & n(1) & n(2);
            when 4      => return n(1) & n(0) & n(2);
            when 5      => return n(0) & n(2) & n(1);
            when 6      => return n(0) & n(1) & n(0);
            when others => return n(1) & n(0) & n(0);
        end case;
    end function;
    -- bit length (0 .. 16)
    function f_blen(v : unsigned(15 downto 0)) return unsigned is
        variable r : unsigned(4 downto 0);
    begin
        r := (others => '0');
        for i in 0 to 15 loop
            if v(i) = '1' then r := to_unsigned(i + 1, 5); end if;
        end loop;
        return r;
    end function;
    -- band-edge coverage: 16 - (d >> (e - 5)) (or d << (5 - e)), 0 when that is 16 or more; e = 0 never blends
    function f_ov(d : unsigned(11 downto 0); e : unsigned(4 downto 0)) return unsigned is
        variable x, y : unsigned(15 downto 0);
    begin
        x := "0000" & d;
        case to_integer(e) is
            when 1      => y := shift_left(x, 4);
            when 2      => y := shift_left(x, 3);
            when 3      => y := shift_left(x, 2);
            when 4      => y := shift_left(x, 1);
            when 5      => y := x;
            when 6      => y := shift_right(x, 1);
            when 7      => y := shift_right(x, 2);
            when 8      => y := shift_right(x, 3);
            when 9      => y := shift_right(x, 4);
            when 10     => y := shift_right(x, 5);
            when 11     => y := shift_right(x, 6);
            when 12     => y := shift_right(x, 7);
            when 13     => y := shift_right(x, 8);
            when 14     => y := shift_right(x, 9);
            when 15     => y := shift_right(x, 10);
            when 16     => y := shift_right(x, 11);
            when others => y := (others => '1');
        end case;
        if y(15 downto 4) = 0 then
            return to_unsigned(16, 5) - resize(y(3 downto 0), 5);
        else
            return to_unsigned(0, 5);
        end if;
    end function;
    -- 13-bit signed -> [lo, hi]
    function f_sat(v : signed(13 downto 0); lo : integer; hi : integer) return unsigned is
    begin
        if v < lo then return to_unsigned(lo, 10);
        elsif v > hi then return to_unsigned(hi, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

    -- luma cell store: one 12-bit vertical-slew state per 8-pixel cell (read a cell ahead of the write)
    type t_lb is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check : boolean;
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
    signal s_h1, s_h2 : unsigned(15 downto 0) := (others => '0');
    signal s_la, s_lbh : unsigned(11 downto 0) := (others => '0');
    signal s_cam   : unsigned(16 downto 0) := (others => '0');
    signal s_cama, s_camc : unsigned(15 downto 0) := (others => '0');

    -- Courbes table f(t), 1024 x 16 over t = 0 .. 1 (written in the zone; read by the line engine and the pixels)
    type t_ct is array (0 to 1023) of std_logic_vector(15 downto 0);
    signal ct : t_ct := (others => (others => '0'));
    attribute no_rw_check of ct : signal is true;
    signal r_c_a  : unsigned(9 downto 0) := (others => '0');
    signal r_c_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal ct_we  : std_logic := '0';
    signal ct_wa  : unsigned(9 downto 0) := (others => '0');
    signal ct_wd  : std_logic_vector(15 downto 0) := (others => '0');
    -- Matrice tables: the YUV contribution of each channel's level (index 0 .. 224), {Y / 2 : 10, U : 11, V : 11}
    type t_mt is array (0 to 255) of std_logic_vector(31 downto 0);
    signal mta, mtb, mtc : t_mt := (others => (others => '0'));
    attribute no_rw_check of mta : signal is true;
    attribute no_rw_check of mtb : signal is true;
    attribute no_rw_check of mtc : signal is true;
    signal r_ma_a, r_mb_a, r_mc_a : unsigned(7 downto 0) := (others => '0');
    signal r_ma_q, r_mb_q, r_mc_q : std_logic_vector(31 downto 0) := (others => '0');
    signal mt_we  : std_logic_vector(2 downto 0) := "000";
    signal mt_wa  : unsigned(7 downto 0) := (others => '0');
    signal mt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    -- colouring ROM (registered address and data, then two fabric copies)
    signal r_k_a  : unsigned(7 downto 0) := (others => '0');
    signal r_k_q, k_f1, k_f2 : std_logic_vector(15 downto 0) := (others => '0');

    -- output filter stores (as the shell)
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da, db : t_dl := (others => (others => '0'));
    attribute no_rw_check of da : signal is true;
    attribute no_rw_check of db : signal is true;
    signal s_dp : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_dr : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_qa, s_qb, s_fa, s_fb : std_logic_vector(31 downto 0) := (others => '0');
    type t_vy is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_vc is array (0 to 511) of std_logic_vector(15 downto 0);
    signal vy : t_vy := (others => x"10");
    signal vc : t_vc := (others => x"8080");
    attribute no_rw_check of vy : signal is true;
    attribute no_rw_check of vc : signal is true;
    signal v_k21, v_k23 : unsigned(10 downto 0) := (others => '0');
    signal v_qy, v_fy, v_wyd : std_logic_vector(7 downto 0) := x"10";
    signal v_qc, v_fc, v_wcd : std_logic_vector(15 downto 0) := x"8080";
    signal v_wya : unsigned(9 downto 0) := (others => '0');
    signal v_wca : unsigned(8 downto 0) := (others => '0');
    signal v_wye, v_wce : std_logic := '0';
    signal v_py, v_pu, v_pv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_incx : unsigned(14 downto 0) := to_unsigned(8738, 15);
    signal s_incy : unsigned(16 downto 0) := to_unsigned(15534, 17);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(420 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(620 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(860 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(0, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_sa, s_sb, s_sc, s_iso, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal z_lfa, z_lfb, z_lfc : unsigned(23 downto 0) := (others => '0');   -- Animate 23 s, breath 13 s, lean 37 s
    signal z_v   : signed(11 downto 0) := (others => '0');                   -- 2 P12 - 1023
    signal z_p   : unsigned(12 downto 0) := (others => '0');                 -- K5 x 7
    signal c_pi  : unsigned(2 downto 0) := (others => '0');
    signal c_al  : unsigned(9 downto 0) := (others => '0');
    signal z_t   : signed(15 downto 0) := (others => '0');
    signal z_u, z_t16, z_l16, z_q16 : unsigned(16 downto 0) := (others => '0');   -- offset-binary shift sums
    signal z_cu  : unsigned(16 downto 0) := (others => '0');
    signal z_lean : signed(15 downto 0) := (others => '0');
    signal z_cq0, c_cq : signed(15 downto 0) := (others => '0');
    signal c_d0  : signed(33 downto 0) := to_signed(4198404, 34);
    signal c_dd  : signed(33 downto 0) := (others => '0');
    signal c_ba, c_bb, c_bc : unsigned(15 downto 0) := (others => '0');     -- Seuils thresholds (SHIFT + Animate)
    signal c_zo  : unsigned(15 downto 0) := (others => '0');                 -- Isohelie BITSWAP offset
    signal c_g   : unsigned(7 downto 0) := to_unsigned(204, 8);              -- camera depth
    signal z_base : signed(11 downto 0) := (others => '0');
    signal c_oy  : signed(11 downto 0) := to_signed(64, 12);
    signal c_ou, c_ov : signed(11 downto 0) := to_signed(512, 12);
    signal c_iay, c_iau, c_iav, c_iby, c_ibu, c_ibv, c_icy, c_icu, c_icv : signed(11 downto 0) := (others => '0');

    -- field serial multiply: acc = +- a x b (mod 2^32); slot frames of 32 zone clocks from zc 40
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : signed(16 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_ng  : std_logic := '0';
    signal z_ld  : std_logic := '0';
    signal z_srun : std_logic := '0';
    signal z_k   : unsigned(4 downto 0) := (others => '0');
    signal z_sc  : unsigned(4 downto 0) := (others => '0');

    -- table writers
    signal cw_run : std_logic := '0';
    signal cw_i  : unsigned(9 downto 0) := (others => '0');
    signal cw_a, cw_d : signed(33 downto 0) := (others => '0');
    signal mw_run : std_logic := '0';
    signal mw_c  : unsigned(9 downto 0) := (others => '0');
    signal mw_ay, mw_au, mw_av : signed(19 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (the V side of the figures, computed for the next line)
    signal le_avq, le_kl : std_logic := '0';
    signal le_p  : unsigned(23 downto 0) := (others => '0');
    signal le_go, le1, le2, le3, le4, le5 : std_logic := '0';
    signal le_first : std_logic := '0';
    signal le_t1 : unsigned(15 downto 0) := (others => '0');
    signal le_z  : unsigned(2 downto 0) := (others => '0');
    signal le_fy : unsigned(15 downto 0) := (others => '0');
    signal le_dd : signed(16 downto 0) := (others => '0');
    signal le_gy : unsigned(15 downto 0) := (others => '0');
    signal le_e  : unsigned(4 downto 0) := (others => '0');
    signal l_fy  : unsigned(15 downto 0) := (others => '0');
    signal l_e, l_ed : unsigned(4 downto 0) := (others => '0');
    signal l_e1, l_e2 : std_logic := '0';
    signal l_za, l_zb, l_zc : unsigned(2 downto 0) := (others => '0');

    -- beam and pixel path
    signal b_x   : unsigned(23 downto 0) := (others => '0');
    signal p4_fx : unsigned(15 downto 0) := (others => '0');
    signal p5_ds : unsigned(16 downto 0) := (others => '0');
    signal p5_fx : unsigned(15 downto 0) := (others => '0');
    signal p5_lt, p5_gt, p6_lt, p6_gt, p7_lt, p7_gt : std_logic := '0';
    signal p6_d, p6_r, p6_c, p6_ta, p6_tb, p6_tc : unsigned(15 downto 0) := (others => '0');
    signal p7_sa, p7_sb, p7_sc : unsigned(15 downto 0) := (others => '0');
    signal p7_ca, p7_cb, p7_cc : std_logic := '0';
    signal p8_sa, p8_sb, p8_sc : unsigned(15 downto 0) := (others => '0');
    signal p8_aa, p8_ab, p8_ac : std_logic := '0';
    signal p9_ca, p9_cb, p9_cc : unsigned(7 downto 0) := (others => '0');
    signal p9_da, p9_db, p9_dc : signed(3 downto 0) := (others => '0');
    signal p9_pa, p9_pb, p9_pc : unsigned(11 downto 0) := (others => '0');
    signal p9_aa, p9_ab, p9_ac : std_logic := '0';
    signal p10_ca, p10_cb, p10_cc : unsigned(7 downto 0) := (others => '0');
    signal p10_da, p10_db, p10_dc : signed(3 downto 0) := (others => '0');
    signal p10_oa, p10_ob, p10_oc : unsigned(4 downto 0) := (others => '0');
    signal p11_ca, p11_cb, p11_cc : unsigned(7 downto 0) := (others => '0');
    signal p11_ra, p11_rb, p11_rc : signed(8 downto 0) := (others => '0');
    signal p14_a, p14_b, p14_c : std_logic_vector(31 downto 0) := (others => '0');
    signal p15_yab, p15_yco : signed(13 downto 0) := (others => '0');
    signal p15_uab, p15_uco, p15_vab, p15_vco : signed(13 downto 0) := (others => '0');
    signal p16_y, p16_u, p16_v : signed(13 downto 0) := (others => '0');
    signal p17_y, p17_u, p17_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KU : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KV : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories (one write statement each, registered reads)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    -- written only in the zone, where the reads are unused
    p_ct : process(clk)
    begin
        if rising_edge(clk) then
            r_c_q <= ct(to_integer(r_c_a));
            if ct_we = '1' then ct(to_integer(ct_wa)) <= ct_wd; end if;
        end if;
    end process p_ct;

    p_mta : process(clk)
    begin
        if rising_edge(clk) then
            r_ma_q <= mta(to_integer(r_ma_a));
            if mt_we(0) = '1' then mta(to_integer(mt_wa)) <= mt_wd; end if;
        end if;
    end process p_mta;
    p_mtb : process(clk)
    begin
        if rising_edge(clk) then
            r_mb_q <= mtb(to_integer(r_mb_a));
            if mt_we(1) = '1' then mtb(to_integer(mt_wa)) <= mt_wd; end if;
        end if;
    end process p_mtb;
    p_mtc : process(clk)
    begin
        if rising_edge(clk) then
            r_mc_q <= mtc(to_integer(r_mc_a));
            if mt_we(2) = '1' then mtc(to_integer(mt_wa)) <= mt_wd; end if;
        end if;
    end process p_mtc;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_k_q <= C_MQ_COL(to_integer(r_k_a));
            k_f1 <= r_k_q;
            k_f2 <= k_f1;
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_q  : unsigned(1 downto 0);
        variable v_r  : unsigned(13 downto 0);
        variable v_s  : signed(15 downto 0);
        variable v_it : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_tid  <= registers_in(8)(3 downto 0);
            s_xsr  <= f_vstep_x(s_tid);
            s_incy <= f_incy(s_tid);
            case to_integer(s_xsr) is
                when 4      => s_incx <= to_unsigned(23302, 15);
                when 3      => s_incx <= to_unsigned(13107, 15);
                when others => s_incx <= to_unsigned(8738, 15);
            end case;

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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_sa  <= registers_in(6)(0);
                s_sb  <= registers_in(6)(1);
                s_sc  <= registers_in(6)(2);
                s_iso <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                z_lfa <= z_lfa + 12157;           -- 23 s
                z_lfb <= z_lfb + 21509;           -- 13 s
                z_lfc <= z_lfc + 7557;            -- 37 s
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

            -- shared glide (1/8 of the gap per field, +-2 deadband; +-4 on the camera channel): four clocks
            -- per channel, run from the zone start by its own phase / channel counters
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
                        if s_gd > 32 or s_gd < -32 or (z_gc /= 5 and (s_gd > 16 or s_gd < -16)) then
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

            -- knob decodes (after the glide)
            if s_zone = '1' and s_zc = 36 then
                z_v <= signed(shift_left(resize(s_gp(12 downto 3), 12), 1)) - to_signed(1023, 12);
                z_p <= shift_left(resize(s_g5(12 downto 3), 13), 3) - resize(s_g5(12 downto 3), 13);
            end if;
            if s_zone = '1' and s_zc = 37 then
                c_pi <= z_p(12 downto 10); c_al <= z_p(9 downto 0);
            end if;

            -- slot frames: slot k occupies zc 40 + 32 k .. 71 + 32 k (k = 0 .. 18); within a frame (z_sc):
            --   0  colouring ROM address (the difference)     4  operands      5  load
            --   8  colouring ROM address (the value)         12  value latched
            --   22 product stable: store                     23 .. 26  the derived adds
            if s_zone = '1' and s_zc = 39 then
                z_srun <= '1'; z_sc <= (others => '1'); z_k <= (others => '1');
            elsif z_srun = '1' then
                z_sc <= z_sc + 1;
                if z_sc = 31 then
                    z_k <= z_k + 1;
                    if z_k = 18 then z_srun <= '0'; end if;
                end if;
            end if;
            if z_sc = 0 then
                r_k_a <= c_pi & (z_k(3 downto 0) - 7) & '1';
            elsif z_sc = 8 then
                r_k_a <= c_pi & (z_k(3 downto 0) - 7) & '0';
            end if;
            if z_sc = 12 then z_base <= signed(k_f2(11 downto 0)); end if;
            if z_srun = '1' and z_sc = 4 then
                z_ng <= '0';
                case to_integer(z_k) is
                    when 0 | 1 | 2 | 3 | 4 =>              -- Animate: parabolic sine t (2 - |t|) of the phase
                        case to_integer(z_k) is
                            when 0      => v_q := z_lfa(23 downto 22);         v_r := z_lfa(21 downto 8);
                            when 1      => v_q := z_lfa(23 downto 22) + 1;     v_r := z_lfa(21 downto 8);
                            when 2      => v_q := z_lfa(23 downto 22) + 2;     v_r := z_lfa(21 downto 8);
                            when 3      => v_q := z_lfb(23 downto 22);         v_r := z_lfb(21 downto 8);
                            when others => v_q := z_lfc(23 downto 22);         v_r := z_lfc(21 downto 8);
                        end case;
                        v_q := v_q + 1;                                        -- w = phase + 1/4
                        if v_q(0) = '1' then
                            z_fa <= signed(resize(v_r, 17));
                            z_fb <= to_unsigned(32768, 16) - resize(v_r, 16);
                        else
                            z_fa <= signed(resize(to_unsigned(16384, 15) - resize(v_r, 15), 17));
                            z_fb <= to_unsigned(16384, 16) + resize(v_r, 16);
                        end if;
                        z_ng <= v_q(1) xnor v_q(0);
                    when 5 =>                                                  -- curve
                        z_fa <= resize(z_v, 17); z_fb <= to_unsigned(1886, 16);
                    when 6 =>                                                  -- camera depth
                        z_fa <= signed(resize(s_g6(12 downto 3), 17)); z_fb <= to_unsigned(205, 16);
                    when others =>                                             -- colouring blend
                        z_fa <= resize(signed(k_f2), 17); z_fb <= resize(c_al, 16);
                end case;
            end if;
            z_ld <= '0';
            if z_srun = '1' and z_sc = 4 then z_ld <= '1'; end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                if z_ng = '1' then f_a <= unsigned(-resize(z_fa, 32)); else f_a <= unsigned(resize(z_fa, 32)); end if;
                f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if z_srun = '1' and z_sc = 22 then
                v_s  := signed(f_acc(29 downto 14));
                v_it := z_base + signed(f_acc(21 downto 10));
                -- the shift sums of a signed s run on u = s + 2^14 >= 0 (no sign net on two LUT inputs: router1
                -- hangs on those); the constant comes off afterwards, exactly
                z_u <= unsigned(resize(v_s, 17) + to_signed(16384, 17));
                case to_integer(z_k) is
                    when 5         => z_cq0 <= signed(f_acc(21 downto 6));
                    when 6         => c_g <= f_acc(17 downto 10);
                    when 7         => c_oy  <= v_it;
                    when 8         => c_ou  <= v_it;
                    when 9         => c_ov  <= v_it;
                    when 10        => c_iay <= v_it;
                    when 11        => c_iau <= v_it;
                    when 12        => c_iav <= v_it;
                    when 13        => c_iby <= v_it;
                    when 14        => c_ibu <= v_it;
                    when 15        => c_ibv <= v_it;
                    when 16        => c_icy <= v_it;
                    when 17        => c_icu <= v_it;
                    when others    => c_icv <= v_it;
                end case;
            end if;
            if z_srun = '1' and z_sc = 23 then
                case to_integer(z_k) is
                    when 0 | 1 | 2 => z_t16 <= shift_right(z_u, 2) + shift_right(z_u, 3) + shift_right(z_u, 4);   -- 7/16
                    when 3         => z_t16 <= shift_right(z_u, 3) + shift_right(z_u, 4);                         -- 3/16
                    when 4         => z_l16 <= shift_right(z_u, 3) + shift_right(z_u, 5);                         -- 5/32
                    when 5         => c_cq <= z_cq0 + z_lean;
                    when others    => null;
                end case;
            end if;
            if z_srun = '1' and z_sc = 24 then
                case to_integer(z_k) is
                    when 0 | 1 | 2 => z_t <= signed(z_t16(15 downto 0)) - to_signed(7168, 16);
                    when 3         => z_t <= signed(z_t16(15 downto 0)) - to_signed(3072, 16);
                    when 4         => z_lean <= signed(z_l16(15 downto 0)) - to_signed(2560, 16);
                    when 5         =>
                        c_d0 <= to_signed(4198404, 34) + shift_left(resize(c_cq, 34), 7);
                        z_cu <= unsigned(resize(c_cq, 17) + to_signed(32768, 17));
                    when others    => null;
                end case;
            end if;
            if z_srun = '1' and z_sc = 25 then
                case to_integer(z_k) is
                    when 0      => c_ba <= shift_left(resize(s_g1(12 downto 3), 16), 6) + unsigned(z_t);
                    when 1      => c_bb <= shift_left(resize(s_g2(12 downto 3), 16), 6) + unsigned(z_t);
                    when 2      => c_bc <= shift_left(resize(s_g3(12 downto 3), 16), 6) + unsigned(z_t);
                    when 3      => c_zo <= shift_left(resize(s_g4(12 downto 3), 16), 6) + unsigned(z_t);
                    when 5      => z_q16 <= shift_right(z_cu, 2) + shift_right(z_cu, 11);              -- cq / 4 + cq / 2048
                    when others => null;
                end case;
            end if;
            if z_srun = '1' and z_sc = 26 and z_k = 5 then
                c_dd <= to_signed(8208, 34) - signed(resize(z_q16, 34));                             -- -(that), exactly
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
    -- Table writers (zone clocks 680 ..): Courbes f(t) by second differences
    -- (Q32: A += D, D += DD), and the three Matrice tables (entry k = k x inc,
    -- for channel A, B, C in turn)
    ---------------------------------------------------------------------------
    p_wr : process(clk)
        variable v_y, v_u, v_v : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            ct_we <= '0';
            if s_zone = '1' and s_zc = 680 then
                cw_run <= '1'; cw_i <= (others => '0');
                cw_a <= (others => '0'); cw_d <= c_d0;
            elsif cw_run = '1' then
                ct_we <= '1'; ct_wa <= cw_i;
                if cw_a(33) = '1' then
                    ct_wd <= (others => '0');
                elsif cw_a(32) = '1' then
                    ct_wd <= (others => '1');
                else
                    ct_wd <= std_logic_vector(cw_a(31 downto 16));
                end if;
                cw_a <= cw_a + cw_d;
                cw_d <= cw_d + c_dd;
                cw_i <= cw_i + 1;
                if cw_i = 1023 then cw_run <= '0'; end if;
            end if;

            mt_we <= "000";
            if s_zone = '1' and s_zc = 680 then
                mw_run <= '1'; mw_c <= (others => '0');
                mw_ay <= (others => '0'); mw_au <= (others => '0'); mw_av <= (others => '0');
            elsif mw_run = '1' then
                mt_wa <= mw_c(7 downto 0);
                mt_wd <= std_logic_vector(mw_ay(18 downto 9)) & std_logic_vector(mw_au(18 downto 8))
                         & std_logic_vector(mw_av(18 downto 8));
                case to_integer(mw_c(9 downto 8)) is
                    when 0      => mt_we <= "001"; v_y := resize(c_iay, 20); v_u := resize(c_iau, 20); v_v := resize(c_iav, 20);
                    when 1      => mt_we <= "010"; v_y := resize(c_iby, 20); v_u := resize(c_ibu, 20); v_v := resize(c_ibv, 20);
                    when others => mt_we <= "100"; v_y := resize(c_icy, 20); v_u := resize(c_icu, 20); v_v := resize(c_icv, 20);
                end case;
                if mw_c(7 downto 0) = 255 then
                    mw_ay <= (others => '0'); mw_au <= (others => '0'); mw_av <= (others => '0');
                else
                    mw_ay <= mw_ay + v_y; mw_au <= mw_au + v_u; mw_av <= mw_av + v_v;
                end if;
                mw_c <= mw_c + 1;
                if mw_c = 767 then mw_run <= '0'; end if;
            end if;
        end if;
    end process p_wr;

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
    -- screen (one pole, 1/32 per line; the state lives in the cell store), then
    -- slewed across by two one-pole filters (1/16 per pixel), times the Seuils
    -- CV depth (the Osc at twice the camera's).  All three filters decay, so
    -- none needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(16 downto 0);
        variable v_p : unsigned(16 downto 0);
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
            -- 3: state += (cell average * 16 - state) / 32
            v_d := signed(resize(s_sum8 & '0', 13)) - signed(resize(s_old, 13));
            v_d := signed(resize(s_old, 13)) + shift_right(v_d, 5);
            s_new   <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);

            -- pixel side: the cell stream (the line above's state), two slews across, the CV depth
            s_lv <= unsigned(s_lb_q(11 downto 0));
            v_e  := signed(resize(s_lv & "0000", 17)) - signed(resize(s_h1, 17));
            v_e  := signed(resize(s_h1, 17)) + shift_right(v_e, 4);
            s_h1 <= unsigned(v_e(15 downto 0));
            v_e  := signed(resize(s_h1, 17)) - signed(resize(s_h2, 17));
            v_e  := signed(resize(s_h2, 17)) + shift_right(v_e, 4);
            s_h2 <= unsigned(v_e(15 downto 0));
            s_la  <= s_h2(11 downto 8) * c_g;
            s_lbh <= s_h2(15 downto 12) * c_g;
            v_p := resize(s_la, 17) + shift_left(resize(s_lbh, 17), 4);   -- luma x depth
            if s_src = '1' then s_cam <= shift_left(v_p, 1); else s_cam <= v_p; end if;
            s_cama <= s_cam(15 downto 0);
            v_p := s_cam - shift_right(s_cam, 2);
            s_camc <= v_p(15 downto 0);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: at each line's end (and once in the zone for line 0) step the
    -- V ramp, read the curve there (fy), its row-to-row change (the coverage
    -- scale) and the bitswap zone; latched by the next line's start
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_z : unsigned(15 downto 0);
        variable v_g : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            le_avq <= g_avid;
            le_kl  <= le_avq and not g_avid;
            if s_zone = '1' and s_zc = 1720 then
                le_p <= resize(s_incy(16 downto 1), 24);
                le_first <= '1';
            elsif le_avq = '1' and g_avid = '0' then
                le_p <= le_p + resize(s_incy, 24);
            end if;
            le_go <= '0';
            if (s_zone = '1' and s_zc = 1722) or le_kl = '1' then le_go <= '1'; end if;
            le1 <= le_go; le2 <= le1; le3 <= le2; le4 <= le3; le5 <= le4;
            -- go: the RAM address (in p_pix); the zone sum
            if le_go = '1' then
                le_t1 <= shift_right(le_p(23 downto 8), 2) + shift_right(le_p(23 downto 8), 3);
            end if;
            if le1 = '1' then
                v_z := c_zo + le_t1;
                le_z <= v_z(15 downto 13);
            end if;
            -- 3: the curve at the V ramp (p4_fx holds the read)
            if le3 = '1' then le_fy <= p4_fx; end if;
            if le4 = '1' then le_dd <= signed('0' & le_fy) - signed('0' & l_fy); end if;
            if le5 = '1' then
                v_g := abs(le_dd);
                if le_first = '1' then le_gy <= (others => '0'); else le_gy <= unsigned(v_g(15 downto 0)); end if;
            end if;
            le_e <= f_blen(le_gy);
            if le_kl = '1' then le_first <= '0'; end if;
            -- the next line's start latches the values
            if g_start = '1' then
                l_fy <= le_fy;
                l_e  <= le_e;
                if le_e > 1 then l_ed <= le_e - 1; l_e2 <= '1'; else l_ed <= (others => '0'); l_e2 <= '0'; end if;
                if le_e > 0 then l_e1 <= '1'; else l_e1 <= '0'; end if;
                l_za <= le_z; l_zb <= le_z + 3; l_zc <= le_z + 5;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the H ramp of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_q   : unsigned(22 downto 0);
        variable v_la, v_lb, v_lc : std_logic;
        variable v_n, v_cu, v_nb : unsigned(2 downto 0);
        variable v_s   : unsigned(15 downto 0);
        variable v_f   : unsigned(12 downto 0);
        variable v_lo, v_hs : std_logic;
        variable v_si  : unsigned(7 downto 0);
        variable v_i   : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: the H ramp (0 .. 2^24 over the line)
            if g_start = '1' then
                b_x <= resize(s_incx(14 downto 1), 24);
            elsif g_avid = '1' then
                b_x <= b_x + resize(s_incx, 24);
            end if;
            -- 2: mirrored ramp -> curve table address (the line engine's read in hblank)
            if le_go = '1' then
                if le_p(23) = '1' then v_q := not le_p(22 downto 0); else v_q := le_p(22 downto 0); end if;
            else
                if b_x(23) = '1' then v_q := not b_x(22 downto 0); else v_q := b_x(22 downto 0); end if;
            end if;
            r_c_a <= v_q(22 downto 13);
            -- 3: the table (EBR); 4: fabric copy
            p4_fx <= unsigned(r_c_q);
            -- 5: Rampes figures: the sum, the compares
            p5_ds <= ('0' & p4_fx) + ('0' & l_fy);
            if l_fy < p4_fx then p5_lt <= '1'; else p5_lt <= '0'; end if;
            if p4_fx < l_fy then p5_gt <= '1'; else p5_gt <= '0'; end if;
            p5_fx <= p4_fx;
            -- 6: DIAMOND, RECTANGLE, CROSS; the Seuils thresholds (SHIFT + Animate + camera CV)
            p6_d <= p5_ds(16 downto 1);
            if p5_lt = '1' then p6_r <= l_fy; p6_c <= p5_fx; else p6_r <= p5_fx; p6_c <= l_fy; end if;
            p6_ta <= c_ba + s_cama;
            p6_tb <= c_bb - s_cama;
            p6_tc <= c_bc + s_camc;
            p6_lt <= p5_lt; p6_gt <= p5_gt;
            -- 7: the shift (in - thr, wrapped) and the comparators
            p7_sa <= p6_d - p6_ta; p7_sb <= p6_r - p6_tb; p7_sc <= p6_c - p6_tc;
            if p6_d > p6_ta then p7_ca <= '1'; else p7_ca <= '0'; end if;
            if p6_r > p6_tb then p7_cb <= '1'; else p7_cb <= '0'; end if;
            if p6_c > p6_tc then p7_cc <= '1'; else p7_cc <= '0'; end if;
            p7_lt <= p6_lt; p7_gt <= p6_gt;
            -- 8: the logic switches; where the chosen logic disagrees with the channel's own comparator the
            --    offset lands on the wrong side and the output clips to 0 (L = 1) or 1 (L = 0)
            if s_sa = '1' then v_la := p7_ca and p7_cb; else v_la := p7_ca; end if;
            if s_sb = '1' then v_lb := p7_ca or p7_cb;  else v_lb := p7_cb; end if;
            if s_sc = '1' then v_lc := p7_ca xor p7_cb; else v_lc := p7_cc; end if;
            if v_la /= p7_ca then
                if v_la = '1' then p8_sa <= (others => '0'); else p8_sa <= (others => '1'); end if;
                p8_aa <= '0';
            else
                p8_sa <= p7_sa; p8_aa <= l_e2;
            end if;
            if v_lb /= p7_cb then
                if v_lb = '1' then p8_sb <= (others => '0'); else p8_sb <= (others => '1'); end if;
                p8_ab <= '0';
            else
                p8_sb <= p7_sb; p8_ab <= p7_lt and l_e1;
            end if;
            if v_lc /= p7_cc then
                if v_lc = '1' then p8_sc <= (others => '0'); else p8_sc <= (others => '1'); end if;
                p8_ac <= '0';
            else
                p8_sc <= p7_sc; p8_ac <= p7_gt and l_e1;
            end if;
            -- 9: Isohelie: code, re-ordered by the zone's bit order, the nearer neighbour's code, the distance to
            --    that step edge; Isohelie out: the shifted level itself, blended only across the seam
            for ch in 0 to 2 loop
                case ch is
                    when 0      => v_s := p8_sa; v_cu := l_za;
                    when 1      => v_s := p8_sb; v_cu := l_zb;
                    when others => v_s := p8_sc; v_cu := l_zc;
                end case;
                v_n := v_s(15 downto 13);
                v_f := v_s(12 downto 0);
                if v_f(12) = '1' then v_nb := f_perm(v_cu, v_n + 1); else v_nb := f_perm(v_cu, v_n - 1); end if;
                v_cu := f_perm(v_cu, v_n);
                if v_n = 0 and v_f(12) = '0' then v_lo := '1'; else v_lo := '0'; end if;
                if v_n = 7 and v_f(12) = '1' then v_hs := '1'; else v_hs := '0'; end if;
                v_si := v_s(15 downto 8) - resize(v_s(15 downto 11), 8);
                case ch is
                    when 0 =>
                        if v_f(12) = '1' then p9_pa <= not v_f(11 downto 0); else p9_pa <= v_f(11 downto 0); end if;
                        if s_iso = '1' then
                            p9_ca <= v_si;
                            if v_lo = '1' then p9_da <= to_signed(7, 4); else p9_da <= to_signed(-7, 4); end if;
                            p9_aa <= p8_aa and (v_lo or v_hs);
                        else
                            p9_ca <= v_cu & "00000";
                            p9_da <= signed(resize(v_nb, 4)) - signed(resize(v_cu, 4));
                            p9_aa <= p8_aa;
                        end if;
                    when 1 =>
                        if v_f(12) = '1' then p9_pb <= not v_f(11 downto 0); else p9_pb <= v_f(11 downto 0); end if;
                        if s_iso = '1' then
                            p9_cb <= v_si;
                            if v_lo = '1' then p9_db <= to_signed(7, 4); else p9_db <= to_signed(-7, 4); end if;
                            p9_ab <= p8_ab and (v_lo or v_hs);
                        else
                            p9_cb <= v_cu & "00000";
                            p9_db <= signed(resize(v_nb, 4)) - signed(resize(v_cu, 4));
                            p9_ab <= p8_ab;
                        end if;
                    when others =>
                        if v_f(12) = '1' then p9_pc <= not v_f(11 downto 0); else p9_pc <= v_f(11 downto 0); end if;
                        if s_iso = '1' then
                            p9_cc <= v_si;
                            if v_lo = '1' then p9_dc <= to_signed(7, 4); else p9_dc <= to_signed(-7, 4); end if;
                            p9_ac <= p8_ac and (v_lo or v_hs);
                        else
                            p9_cc <= v_cu & "00000";
                            p9_dc <= signed(resize(v_nb, 4)) - signed(resize(v_cu, 4));
                            p9_ac <= p8_ac;
                        end if;
                end case;
            end loop;
            -- 10: coverage (0 .. 16 thirty-seconds of the neighbour), from the row-to-row slope's bit length
            if p9_aa = '1' then p10_oa <= f_ov(p9_pa, l_ed); else p10_oa <= (others => '0'); end if;
            if p9_ab = '1' then p10_ob <= f_ov(p9_pb, l_e);  else p10_ob <= (others => '0'); end if;
            if p9_ac = '1' then p10_oc <= f_ov(p9_pc, l_e);  else p10_oc <= (others => '0'); end if;
            p10_ca <= p9_ca; p10_cb <= p9_cb; p10_cc <= p9_cc;
            p10_da <= p9_da; p10_db <= p9_db; p10_dc <= p9_dc;
            -- 11: the blend product
            p11_ra <= resize(p10_da * signed('0' & p10_oa), 9);
            p11_rb <= resize(p10_db * signed('0' & p10_ob), 9);
            p11_rc <= resize(p10_dc * signed('0' & p10_oc), 9);
            p11_ca <= p10_ca; p11_cb <= p10_cb; p11_cc <= p10_cc;
            -- 12: Matrice table addresses
            v_i := signed(resize(p11_ca, 10)) + resize(p11_ra, 10); r_ma_a <= unsigned(v_i(7 downto 0));
            v_i := signed(resize(p11_cb, 10)) + resize(p11_rb, 10); r_mb_a <= unsigned(v_i(7 downto 0));
            v_i := signed(resize(p11_cc, 10)) + resize(p11_rc, 10); r_mc_a <= unsigned(v_i(7 downto 0));
            -- 13: the tables (EBR); 14: fabric copies
            p14_a <= r_ma_q; p14_b <= r_mb_q; p14_c <= r_mc_q;
            -- 15, 16: Matrice sums: o + A + B + C (Y entries are Y / 2)
            p15_yab <= resize(signed(p14_a(31 downto 22)), 14) + resize(signed(p14_b(31 downto 22)), 14);
            p15_yco <= resize(c_oy, 14) + shift_left(resize(signed(p14_c(31 downto 22)), 14), 1);
            p15_uab <= resize(signed(p14_a(21 downto 11)), 14) + resize(signed(p14_b(21 downto 11)), 14);
            p15_uco <= resize(c_ou, 14) + resize(signed(p14_c(21 downto 11)), 14);
            p15_vab <= resize(signed(p14_a(10 downto 0)), 14) + resize(signed(p14_b(10 downto 0)), 14);
            p15_vco <= resize(c_ov, 14) + resize(signed(p14_c(10 downto 0)), 14);
            p16_y <= p15_yco + shift_left(p15_yab, 1);
            p16_u <= p15_uco + p15_uab;
            p16_v <= p15_vco + p15_vab;
            -- 17: the encoder's clip
            p17_y <= f_sat(p16_y, 64, 940);
            p17_u <= f_sat(p16_u, 0, 1023);
            p17_v <= f_sat(p16_v, 0, 1023);
            -- 18: black off the generated avid
            if s_gav_sr(15) = '0' then
                s_y18 <= C_KY; s_u18 <= C_KU; s_v18 <= C_KV;
            else
                s_y18 <= p17_y; s_u18 <= p17_u; s_v18 <= p17_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(15);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output filter stores
    ---------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_dp <= s_dp + 1; s_dr <= s_dr + 1;
            s_qa <= da(to_integer(s_dr));
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y18) & std_logic_vector(s_u18) & std_logic_vector(s_v18);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y20) & std_logic_vector(s_u20) & std_logic_vector(s_v20);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    p_vs : process(clk)
        variable v_s : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(19) = '1' then v_k21 <= v_k21 + 1; else v_k21 <= (others => '0'); end if;   -- column of the stage-21 pixel
            if s_gav_sr(21) = '1' then v_k23 <= v_k23 + 1; else v_k23 <= (others => '0'); end if;   -- column of the stage-23 pixel
            v_qy <= vy(to_integer(v_k21(10 downto 1)));
            v_qc <= vc(to_integer(v_k21(10 downto 2)));
            v_fy <= v_qy; v_fc <= v_qc;
            v_py <= s_y23; v_pu <= s_u23; v_pv <= s_v23;
            v_wye <= s_gav_sr(21) and v_k23(0);
            v_wya <= v_k23(10 downto 1);
            v_s := resize(s_y23, 11) + resize(v_py, 11); v_wyd <= std_logic_vector(v_s(10 downto 3));
            if s_gav_sr(21) = '1' and v_k23(1 downto 0) = "01" then v_wce <= '1'; else v_wce <= '0'; end if;
            v_wca <= v_k23(10 downto 2);
            v_s := resize(s_u23, 11) + resize(v_pu, 11); v_wcd(15 downto 8) <= std_logic_vector(v_s(10 downto 3));
            v_s := resize(s_v23, 11) + resize(v_pv, 11); v_wcd(7 downto 0) <= std_logic_vector(v_s(10 downto 3));
            if v_wye = '1' then vy(to_integer(v_wya)) <= v_wyd; end if;
            if v_wce = '1' then vc(to_integer(v_wca)) <= v_wcd; end if;
        end if;
    end process p_vs;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the rack's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16, a vertical [1 1]/2 with the line above (no
    -- single-row lines), and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_mx : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the black (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 19 sums, stage 20 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y18 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u18 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v18 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(17) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y20 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u20 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v20 <= v_sc(13 downto 4);
            else
                s_y20 <= C_KY; s_u20 <= C_KU; s_v20 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(17);
            -- box B: stage 21 sums, stage 22 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y20 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u20 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v20 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y22 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u22 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v22 <= v_sc(13 downto 4);
            -- 23: luma 13/16
            if s_y22 > 64 then v_dl := s_y22 - 64; else v_dl := (others => '0'); end if;
            s_y23 <= s_y22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u23 <= s_u22; s_v23 <= s_v22;
            -- 24: vertical mix with the line above
            v_mx := resize(s_y23, 11) + resize(unsigned(v_fy) & "10", 11);             s_y24 <= v_mx(10 downto 1);
            v_mx := resize(s_u23, 11) + resize(unsigned(v_fc(15 downto 8)) & "10", 11); s_u24 <= v_mx(10 downto 1);
            v_mx := resize(s_v23, 11) + resize(unsigned(v_fc(7 downto 0)) & "10", 11);  s_v24 <= v_mx(10 downto 1);
            -- 25: blanking gate
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
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture marquetry;
