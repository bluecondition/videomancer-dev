-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Separatrix v0.1.0 -- Patchbay: the phase portrait of a pendulum.
--
-- The patch (an analog-computer plot):
--   V ramp (centred) -> multiplier wired as a squarer -> p^2 (kinetic term)
--   H-locked cosine VCO (Period, Gravity) -> -G cos q ; VCO at twice the rate (Detent) -> -D cos 2q
--   S9 Tilted: + a linear ramp in q (the tilted washboard)
--   camera luma (slewed) or the Osc lattice -> + (K6)
--   mixer -> H = p^2 - G cos q - D cos 2q - F q + camera
--   H -> staircase quantiser (Bands, flow) -> colouriser (Colours) -> chroma transform (K5)
--   H -> window comparator at the critical (saddle) energy -> the separatrix (white, glow) and the colouriser's
--        inside / outside key (closed orbits warm, open orbits cool)
--
--   K1  Period     eyes across the screen, 1 .. 6, detented at the integers
--   K2  Detent     bipolar, linear: left = flattened lozenges, then a double well (the eyes pinch into pairs,
--                  a white figure-eight inside); right = cusped eyes, then small secondary eyes between them
--   K3  Speed      bipolar, square law, +-8 deadband: the portrait drifts along q and the bands flow out (in)
--   K4  Bands      energy per band (staircase density), square law, 0.26x .. 3.2x
--   K5  Colour     eight colourings (U / V swap and inversions; zone 4, the centre, = the designed palette)
--   K6  Camera     camera / Osc strength into H, 0 .. 100 %
--   S7  Look       Bands / Orbits (thin iso-energy lines on black: a scope plot)
--   S8  Separatrix Off / On (the comparator: white line + inside / outside palette split)
--   S9  Drive      Level / Tilted (linear term: fish-shaped eyes, bands run downhill)
--   S10 Colours    Ember / Moss
--   S11 Source     Video / Osc
--   P12 GRAVITY    eye height 32 .. 543 virtual px: 0 % = level bands with a ripple .. 100 % = eyes touch the edges
--
-- Engine: vblank-zone sequencer (one 32-bit serial multiplier, 15 slots, shared knob glide), then a table writer
-- (one pipelined 16 x 10 multiplier) that rewrites the potential table V(q) and its slope V'(q) every field,
-- scaled by Gravity, Detent, Bands and Period, and finds the saddle energy; a per-line second-order DDA for
-- p^2 (and its slope); per-pixel phase / tilt DDAs; a 14-stage pixel path.  Virtual 1080-line coordinates.
-- Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16, a
-- vertical [1 1]/2 with the line above) before the blanking gate.  Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.separatrix_pkg.all;
use work.all;

architecture separatrix of program_top is
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
    -- per-mode line-DDA constants: E7 = (ys (ys - 276480)) >> 7 and ys^2
    function f_e7(t : std_logic_vector(3 downto 0)) return signed is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_signed(-2447963, 24);
            when 4          => return to_signed(-1241568, 24);
            when 8          => return to_signed(-2066400, 24);
            when 12         => return to_signed(-1035000, 24);
            when 5 | 6 | 14 => return to_signed(-828288, 24);
            when 1 | 2 | 10 => return to_signed(-1103872, 24);
            when others     => return to_signed(-552448, 24);
        end case;
    end function;
    function f_ys2(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1295044, 21);
            when 4          => return to_unsigned(331776, 21);
            when 8          => return to_unsigned(921600, 21);
            when 12         => return to_unsigned(230400, 21);
            when 5 | 6 | 14 => return to_unsigned(147456, 21);
            when 1 | 2 | 10 => return to_unsigned(262144, 21);
            when others     => return to_unsigned(65536, 21);
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
    -- Period: 1.0 .. 6.0 in Q8, flat for 256 of every 1024 (k x 5) codes either side of an integer
    function f_det5(k : unsigned(9 downto 0)) return unsigned is
        variable v_s : unsigned(12 downto 0);
        variable v_t : unsigned(9 downto 0);
    begin
        v_s := shift_left(resize(k, 13), 2) + resize(k, 13);
        if v_s(9 downto 8) = "00" then v_t := (others => '0');
        elsif v_s(9) = '1' and v_s(8) = '1' then v_t := to_unsigned(512, 10);
        else v_t := v_s(9 downto 0) - 256; end if;
        return to_unsigned(256, 11) + shift_left(resize(v_s(12 downto 10), 11), 8) + resize(v_t(9 downto 1), 11);
    end function;
    -- bipolar knob: |k - 512| - 8 outside a +-8 deadband
    function f_bm(k : unsigned(9 downto 0)) return unsigned is
        variable v_s : signed(10 downto 0);
        variable v_m : unsigned(9 downto 0);
    begin
        v_s := signed(resize(k, 11)) - 512;
        if v_s > -9 and v_s < 9 then
            v_m := (others => '0');
        elsif v_s(10) = '1' then
            v_m := unsigned(not v_s(9 downto 0)) - 7;
        else
            v_m := unsigned(v_s(9 downto 0)) - 8;
        end if;
        return v_m(8 downto 0);
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
    signal s_la, s_lbh : unsigned(13 downto 0) := (others => '0');
    signal s_lterm : unsigned(12 downto 0) := (others => '0');

    -- potential table V(q) | V'(q): 512 entries over q = 0 .. pi (bin centres), written in the zone
    type t_tab is array (0 to 511) of std_logic_vector(31 downto 0);
    signal tab : t_tab := (others => (others => '0'));
    attribute no_rw_check of tab : signal is true;
    signal r_t_a  : unsigned(8 downto 0) := (others => '0');
    signal r_t_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal wt_we  : std_logic := '0';
    signal wt_a   : unsigned(8 downto 0) := (others => '0');
    signal wt_d   : std_logic_vector(31 downto 0) := (others => '0');
    -- ROMs (registered address and data)
    signal r_s_a  : unsigned(8 downto 0) := (others => '0');
    signal r_s_q  : unsigned(15 downto 0) := (others => '0');
    signal r_p_a  : unsigned(9 downto 0) := (others => '0');
    signal r_p_q  : std_logic_vector(31 downto 0) := (others => '0');

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
    signal vy : t_vy := (others => x"18");
    signal vc : t_vc := (others => x"8080");
    attribute no_rw_check of vy : signal is true;
    attribute no_rw_check of vc : signal is true;
    signal v_k17, v_k19 : unsigned(10 downto 0) := (others => '0');
    signal v_qy, v_fy, v_wyd : std_logic_vector(7 downto 0) := x"18";
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
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(410 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_look, s_sep, s_tilt, s_pal, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal z_n8  : unsigned(10 downto 0) := to_unsigned(768, 11);       -- Period, Q8
    signal z_m2, z_m3 : unsigned(8 downto 0) := (others => '0');
    signal z_n2, z_n3 : std_logic := '0';
    signal z_lfo : unsigned(10 downto 0) := (others => '0');
    signal z_lfv : signed(7 downto 0) := (others => '0');
    signal z_hh  : unsigned(9 downto 0) := to_unsigned(288, 10);
    signal c_dn  : unsigned(11 downto 0) := to_unsigned(690, 12);
    signal c_ga  : unsigned(15 downto 0) := (others => '0');
    signal z_dlr : unsigned(10 downto 0) := (others => '0');
    signal z_dt  : signed(11 downto 0) := (others => '0');
    signal c_gbm : unsigned(15 downto 0) := (others => '0');            -- |Gb| (sign c_gbs)
    signal c_gbs : std_logic := '0';
    signal z_tk  : unsigned(13 downto 0) := (others => '0');
    signal c_ka  : unsigned(13 downto 0) := (others => '0');
    signal c_kbm : unsigned(14 downto 0) := (others => '0');            -- |Kb| (sign c_gbs)
    signal c_inc : unsigned(19 downto 0) := (others => '0');
    signal c_fw  : unsigned(22 downto 0) := (others => '0');
    signal c_fx  : unsigned(11 downto 0) := (others => '0');
    signal z_vsq : unsigned(17 downto 0) := (others => '0');
    signal c_dr  : unsigned(25 downto 0) := (others => '0');
    signal c_fl  : unsigned(17 downto 0) := (others => '0');
    signal c_b0  : unsigned(25 downto 0) := (others => '0');
    signal z_b0old : unsigned(1 downto 0) := (others => '0');
    signal z_bv  : std_logic := '0';
    signal c_finc : unsigned(15 downto 0) := (others => '0');
    signal c_fsw : signed(24 downto 0) := (others => '0');
    signal c_saw0 : unsigned(22 downto 0) := (others => '0');
    signal c_gl  : unsigned(9 downto 0) := (others => '0');
    signal c_dd  : unsigned(17 downto 0) := (others => '0');
    signal c_fw8, z_woff, c_wf0 : unsigned(9 downto 0) := (others => '0');
    signal z_hc16, z_hc2 : signed(25 downto 0) := (others => '0');
    signal c_dh2 : signed(17 downto 0) := (others => '0');
    signal c_e2  : std_logic := '0';
    signal c_k5z : unsigned(2 downto 0) := to_unsigned(0, 3);           -- Colour: chroma transform (zone xor 4)

    -- field serial multiply: acc = +- a x b (mod 2^32)
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : signed(24 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_ng  : std_logic := '0';
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

    -- table writer
    signal w_j   : unsigned(10 downto 0) := (others => '0');
    signal w_run, w_go : std_logic := '0';
    signal w0_v, w1_v, w2_v, w3_v, w5_v, w6_cv : std_logic := '0';
    signal w0_sel, w1_sel, w2_sel, w3_sel, w5_sel : unsigned(1 downto 0) := (others => '0');
    signal w0_n, w1_n, w2_n, w3_n, w5_n, w6_n : unsigned(8 downto 0) := (others => '0');
    signal w0_sp, w1_sp, w0_ng, w1_ng : std_logic := '0';
    signal w2_s, w3_s : std_logic := '0';
    signal w2_mag : unsigned(9 downto 0) := (others => '0');
    signal w2_cm  : unsigned(15 downto 0) := (others => '0');
    signal w3_p0, w3_p1 : unsigned(19 downto 0) := (others => '0');
    signal w3_p2  : unsigned(17 downto 0) := (others => '0');
    signal w45_p  : unsigned(25 downto 0) := (others => '0');
    signal w45_s, w45_v : std_logic := '0';
    signal w45_sel : unsigned(1 downto 0) := (others => '0');
    signal w45_n  : unsigned(8 downto 0) := (others => '0');
    signal w_acc  : signed(27 downto 0) := (others => '0');
    signal w_vr   : std_logic_vector(15 downto 0) := (others => '0');
    signal w_swe  : unsigned(22 downto 0) := (others => '0');
    signal w6_cand : signed(25 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (p^2 DDA)
    signal e_q   : signed(33 downto 0) := (others => '0');
    signal e_d   : signed(25 downto 0) := (others => '0');
    signal e_gy  : unsigned(15 downto 0) := (others => '0');

    -- beam (stage 1) and the late tilt DDAs (saw at stage 6, window offset at stage 7)
    signal b_ph  : unsigned(25 downto 0) := (others => '0');
    signal b_w   : std_logic := '0';
    signal p_w   : std_logic_vector(6 downto 2) := (others => '0');
    signal s_gst : std_logic_vector(5 downto 0) := (others => '0');   -- s_gst(k) = g_start delayed k + 1
    signal p6_saw : signed(24 downto 0) := (others => '0');
    signal p7_wf : unsigned(9 downto 0) := (others => '0');
    signal l_lc  : signed(17 downto 0) := (others => '0');
    signal l_gy  : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    signal p2_sg, p3_sg, p4_sg : std_logic := '0';
    signal p4_v, p4_d : signed(15 downto 0) := (others => '0');
    signal p5_s1 : signed(18 downto 0) := (others => '0');
    signal p5_gxs : signed(16 downto 0) := (others => '0');
    signal p6_s2 : signed(18 downto 0) := (others => '0');
    signal p6_gx : unsigned(15 downto 0) := (others => '0');
    signal p7_sep : signed(18 downto 0) := (others => '0');
    signal p7_gmax, p7_gmin : unsigned(15 downto 0) := (others => '0');
    signal p8_band : unsigned(9 downto 0) := (others => '0');
    signal p8_asep : unsigned(17 downto 0) := (others => '0');
    signal p8_g    : unsigned(16 downto 0) := (others => '0');
    signal p8_in, p9_in, p10_in, p11_in : std_logic := '0';
    signal p8_asep2, p9_amin : unsigned(17 downto 0) := (others => '0');
    signal p9_thr  : unsigned(18 downto 0) := (others => '0');
    signal p9_db   : unsigned(7 downto 0) := (others => '0');
    signal p9_g    : unsigned(16 downto 0) := (others => '0');
    signal p9_cls, p10_cls, p11_cls : unsigned(1 downto 0) := (others => '0');
    signal p10_od, p11_od : std_logic := '0';
    signal p10_lv, p11_lv, p12_lv : unsigned(1 downto 0) := (others => '0');
    signal p11_c : std_logic_vector(31 downto 0) := (others => '0');
    signal p12_by, p12_bu, p12_bv : unsigned(9 downto 0) := (others => '0');
    signal p12_dy, p12_du, p12_dv : signed(10 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := unsigned(C_SX_K(31 downto 22));
    constant C_KU : unsigned(9 downto 0) := unsigned(C_SX_K(21 downto 12));
    constant C_KV : unsigned(9 downto 0) := unsigned(C_SX_K(11 downto 2));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

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

    -- the potential table: written only in the zone (the pixel path's reads there are unused)
    p_tab : process(clk)
    begin
        if rising_edge(clk) then
            r_t_q <= tab(to_integer(r_t_a));
            if wt_we = '1' then tab(to_integer(wt_a)) <= wt_d; end if;
        end if;
    end process p_tab;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_s_q <= C_SX_SIN(to_integer(r_s_a));
            r_p_q <= C_SX_PAL(to_integer(r_p_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_t  : signed(16 downto 0);
        variable v_w  : unsigned(26 downto 0);
        variable v_e7 : signed(23 downto 0);
        variable v_y2 : unsigned(20 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_look <= registers_in(6)(0);
                s_sep  <= registers_in(6)(1);
                s_tilt <= registers_in(6)(2);
                s_pal  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                z_lfo  <= z_lfo + 1;
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
                            when 2      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 3      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 32 or s_gd < -32 or (z_gc /= 3 and (s_gd > 16 or s_gd < -16)) then
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
                            when 3      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial multiply slots: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from load + 17
            -- (stored at load + 18).  The slot number is decoded at 30 + 24 k and the operands and the sign are
            -- pre-muxed at 31 + 24 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc >= 30 and s_zc <= 366 then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
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
                    when 366 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_ng <= '0';
                case to_integer(z_sel) is
                    when 0  => z_fa <= signed(resize(s_g4(12 downto 3), 25)); z_fb <= resize(s_g4(12 downto 3), 16);   -- K4^2
                    when 1  => z_fa <= signed(resize(z_hh, 25));              z_fb <= resize(z_hh, 16);              -- hh^2
                    when 2  => z_fa <= signed(resize(f_acc(18 downto 0), 25)); z_fb <= resize(c_dn, 16);            -- x density
                    when 4  => z_fa <= signed(resize(c_ga, 25));                                                      -- |Gb|
                               if z_dt(11) = '1' then z_fb <= resize(unsigned(-z_dt), 16); else z_fb <= resize(unsigned(z_dt), 16); end if;
                    when 5  => z_fa <= signed(resize(z_n8, 25));                                                      -- phase step
                               case to_integer(s_xs) is
                                   when 4      => z_fb <= to_unsigned(23302, 16);
                                   when 3      => z_fb <= to_unsigned(13107, 16);
                                   when others => z_fb <= to_unsigned(8738, 16);
                               end case;
                    when 6  => z_fa <= signed(resize(c_ga, 25));              z_fb <= resize(z_tk, 16);              -- Ka
                    when 7  => z_fa <= signed(resize(c_gbm, 25));             z_fb <= resize(z_tk, 16);              -- |Kb|
                    when 8  => z_fa <= signed(resize(z_m3, 25));              z_fb <= resize(z_m3, 16);              -- speed^2
                    when 9  => z_fa <= signed(resize(c_fw(22 downto 7), 25)); z_fb <= c_inc(19 downto 4);            -- tilt step
                    when 10 => z_fa <= signed(resize(c_fw(22 downto 7), 25)); z_fb <= c_b0(25 downto 10);            -- saw start
                    when 11 => z_fa <= signed(resize(c_dn, 25));              z_fb <= resize(s_g6(12 downto 3), 16); -- camera gain
                    when 12 => v_e7 := f_e7(s_tid); z_fa <= resize(v_e7(23 downto 4), 25); z_fb <= resize(c_dn, 16);   -- line D0
                    when 13 => v_y2 := f_ys2(s_tid); z_fa <= signed(resize(v_y2(20 downto 2), 25)); z_fb <= resize(c_dn, 16); -- line DD
                    when others => z_fa <= to_signed(583200, 25);             z_fb <= resize(c_dn, 16);              -- line Q0
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                if z_ng = '1' then f_a <= unsigned(-resize(z_fa, 32)); else f_a <= unsigned(resize(z_fa, 32)); end if;
                f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- knob decodes (after the glide)
                    when 28 =>
                        z_n8 <= f_det5(s_g1(12 downto 3));
                        z_hh <= resize(s_gp(12 downto 4), 10) + 32;
                        z_m2 <= f_bm(s_g2(12 downto 3)); z_n2 <= not s_g2(12);
                        z_m3 <= f_bm(unsigned(registers_in(2))); z_n3 <= not registers_in(2)(9);
                        if z_lfo(10) = '1' then v_w := resize(not z_lfo(9 downto 0), 27);
                        else v_w := resize(z_lfo(9 downto 0), 27); end if;
                        v_w := resize(v_w(9 downto 0) * to_unsigned(122, 7), 27);
                        z_lfv <= signed(resize(v_w(16 downto 10), 8)) - 61;
                    when 29 =>                          -- Colour: 8 zones of 128 codes, latched 8 codes inside a zone;
                                                        -- the two ends always select the end zones
                        -- (the transform code is the zone xor 4, so the centre zone 4 is the designed palette)
                        if unsigned(registers_in(4)) < 8 then c_k5z <= "100";
                        elsif unsigned(registers_in(4)) > 1015 then c_k5z <= "011";
                        elsif registers_in(4)(6 downto 3) /= "0000" and registers_in(4)(6 downto 3) /= "1111" then
                            c_k5z <= unsigned(registers_in(4)(9 downto 7)) xor "100";
                        end if;
                    when 50  => c_dn <= resize(f_acc(28 downto 9), 12) + 178;
                    when 98  => c_ga <= f_acc(29 downto 14);
                    when 122 =>                          -- detent (linear): left 2.25 m, right 1.22 m (Q10)
                        if z_n2 = '1' then
                            z_dlr <= shift_left(resize(z_m2, 11), 1) + resize(z_m2(8 downto 2), 11);
                        else
                            z_dlr <= resize(z_m2, 11) + resize(z_m2(8 downto 2), 11) - resize(z_m2(8 downto 5), 11);
                        end if;
                    when 123 =>
                        if z_n2 = '1' then z_dt <= resize(z_lfv, 12) - signed(resize(z_dlr, 12));
                        else z_dt <= resize(z_lfv, 12) + signed(resize(z_dlr, 12)); end if;
                    when 146 => c_gbm <= f_acc(25 downto 10); c_gbs <= z_dt(11);
                    when 170 => c_inc <= f_acc(25 downto 6);
                    when 171 =>
                        z_tk <= resize(c_inc(19 downto 6), 14) + resize(c_inc(19 downto 7), 14) + resize(c_inc(19 downto 10), 14);
                        if s_tilt = '1' then
                            c_fw <= shift_left(resize(c_ga, 23), 7) - shift_left(resize(c_ga, 23), 3);
                        else
                            c_fw <= (others => '0');
                        end if;
                    when 172 => c_fw8 <= c_fw(17 downto 8);
                    when 194 => c_ka <= f_acc(29 downto 16);
                    when 195 =>
                        if s_tilt = '1' then
                            c_fx <= resize(c_ka(13 downto 2), 12) + resize(c_ka(13 downto 5), 12) + resize(c_ka(13 downto 6), 12);
                        else
                            c_fx <= (others => '0');
                        end if;
                    when 218 => c_kbm <= f_acc(29 downto 15);
                    when 242 => z_vsq <= f_acc(17 downto 0);
                    when 243 =>                          -- drift and band flow (speed^2, signed)
                        if z_n3 = '1' then
                            c_dr <= c_dr - shift_left(resize(z_vsq, 26), 1);
                            c_fl <= c_fl - resize(z_vsq(17 downto 6), 18);
                        else
                            c_dr <= c_dr + shift_left(resize(z_vsq, 26), 1);
                            c_fl <= c_fl + resize(z_vsq(17 downto 6), 18);
                        end if;
                    when 244 =>                          -- start phase: drift + pi - Period / 2 cycles
                        c_b0 <= c_dr + to_unsigned(33554432, 26) - shift_left(resize(z_n8, 26), 17);
                    when 245 =>
                        -- the tilt's window offset follows the start phase's window crossings (the first field
                        -- only latches)
                        if z_bv = '1' then
                            if z_b0old = "11" and c_b0(25 downto 24) = "00" then
                                z_woff <= z_woff + c_fw8;
                            elsif z_b0old = "00" and c_b0(25 downto 24) = "11" then
                                z_woff <= z_woff - c_fw8;
                            end if;
                        end if;
                        z_bv <= '1'; z_b0old <= c_b0(25 downto 24);
                    when 266 => c_finc <= f_acc(30 downto 15);
                    when 267 => c_fsw  <= signed(resize(c_finc, 25)) - signed(resize(c_fw, 25));
                    when 290 => c_saw0 <= resize(f_acc(31 downto 9), 23);
                    when 314 => c_gl   <= f_acc(21 downto 12) + resize(f_acc(21 downto 13), 10);
                    when 362 => c_dd   <= f_acc(29 downto 12);
                    -- after the table writer (zc 400 .. 2458): the saddle energy
                    when 2470 =>
                        c_dh2 <= z_hc16(25 downto 8) - z_hc2(25 downto 8);
                        if c_gbs = '1' and shift_left(resize(c_gbm, 18), 2) > resize(c_ga, 18) then c_e2 <= '1'; else c_e2 <= '0'; end if;
                    when 2471 =>
                        c_wf0 <= z_woff + c_fl(17 downto 8) - unsigned(z_hc16(17 downto 8));
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
    -- Table writer (zone clocks 400 .. 2458): for each of 512 bins q_n = pi (n + 1/2) / 512 four products
    -- on one pipelined multiplier: V = -(Ga cos q + Gb cos 2q) / 2^12 (Q8 bands) and V' = (Ka sin q +
    -- Kb sin 2q) / 2^10 (Q12 bands per output pixel); tracks the saddle energy max(V - F q_loc).
    ---------------------------------------------------------------------------
    wt_we <= '1' when w5_v = '1' and w5_sel = "11" else '0';
    wt_a  <= w5_n;
    wt_d  <= w_vr & std_logic_vector(w_acc(25 downto 10));

    p_wr : process(clk)
        variable v_a  : unsigned(10 downto 0);
        variable v_n2 : unsigned(10 downto 0);
        variable v_lo : unsigned(8 downto 0);
        variable v_p, v_q : signed(27 downto 0);
        variable v_x, v_y, v_r : signed(28 downto 0);
    begin
        if rising_edge(clk) then
            w_go <= '0';
            if s_zone = '1' and s_zc = 400 then w_go <= '1'; end if;
            if w_go = '1' then
                w_j <= (others => '0'); w_run <= '1';
            elsif w_run = '1' then
                w_j <= w_j + 1;
                if w_j = 2047 then w_run <= '0'; end if;
            end if;
            -- W0: the angle (2048 per cycle) and the quarter-wave fold
            v_n2 := resize(w_j(10 downto 2), 11);
            case w_j(1 downto 0) is
                when "00"   => v_a := shift_left(v_n2, 1) + 513;     -- cos q   (2n + 1 + 512)
                when "01"   => v_a := shift_left(v_n2, 2) + 514;     -- cos 2q  (4n + 2 + 512)
                when "10"   => v_a := shift_left(v_n2, 1) + 1;       -- sin q
                when others => v_a := shift_left(v_n2, 2) + 2;       -- sin 2q
            end case;
            v_lo := v_a(8 downto 0);
            if v_a(9) = '1' then
                r_s_a <= (not v_lo) + 1;
                if v_lo = 0 then w0_sp <= '1'; else w0_sp <= '0'; end if;
            else
                r_s_a <= v_lo; w0_sp <= '0';
            end if;
            w0_ng <= v_a(10); w0_sel <= w_j(1 downto 0); w0_n <= w_j(10 downto 2); w0_v <= w_run;
            -- W1: the ROM
            w1_ng <= w0_ng; w1_sp <= w0_sp; w1_sel <= w0_sel; w1_n <= w0_n; w1_v <= w0_v;
            -- W2: fabric copy; the coefficient and the product's sign (V's products are negated)
            if w1_sp = '1' then w2_mag <= to_unsigned(1023, 10); else w2_mag <= r_s_q(9 downto 0); end if;
            case w1_sel is
                when "00" => w2_cm <= c_ga;                w2_s <= not w1_ng;
                when "01" => w2_cm <= c_gbm;               w2_s <= not (w1_ng xor c_gbs);
                when "10" => w2_cm <= resize(c_ka, 16);    w2_s <= w1_ng;
                when others => w2_cm <= resize(c_kbm, 16); w2_s <= w1_ng xor c_gbs;
            end case;
            w2_sel <= w1_sel; w2_n <= w1_n; w2_v <= w1_v;
            -- W3: partial products (magnitude split 4 / 4 / 2 bits)
            w3_p0 <= w2_cm * w2_mag(3 downto 0);
            w3_p1 <= w2_cm * w2_mag(7 downto 4);
            w3_p2 <= w2_cm * w2_mag(9 downto 8);
            w3_s <= w2_s; w3_sel <= w2_sel; w3_n <= w2_n; w3_v <= w2_v;
            -- W4, W4b: product
            w45_p <= shift_left(resize(w3_p2, 26), 8) + shift_left(resize(w3_p1, 26), 4) + resize(w3_p0, 26);
            w45_s <= w3_s; w45_sel <= w3_sel; w45_n <= w3_n; w45_v <= w3_v;
            -- W5: accumulate the pair (the sign as an inversion plus a carry-in)
            if w45_sel(0) = '0' then v_q := (others => '0'); else v_q := w_acc; end if;
            v_p := signed(resize(w45_p, 28));
            if w45_s = '1' then v_p := not v_p; end if;
            v_x := v_q & '1'; v_y := v_p & w45_s; v_r := v_x + v_y;
            w_acc <= v_r(28 downto 1);
            w5_sel <= w45_sel; w5_n <= w45_n; w5_v <= w45_v;
            -- W6: V (after the cosine pair; the write follows the sine pair); the saddle candidate
            w6_cv <= '0';
            if w_go = '1' then
                w_swe <= resize(c_fw(22 downto 1), 23) + resize(c_fw(22 downto 11), 23);   -- F q_loc of bin 0, Q16
            elsif w5_v = '1' and w5_sel = "01" then
                w_vr <= std_logic_vector(w_acc(27 downto 12));
                w6_cand <= shift_left(resize(w_acc(27 downto 12), 26), 8) - signed(resize(w_swe, 26));
                w_swe <= w_swe + resize(c_fw(22 downto 10), 23);
                w6_cv <= '1'; w6_n <= w5_n;
            end if;
            -- W7: the saddle energy (max over the bins) and the energy at q = 0 (the hump / the centre)
            if w6_cv = '1' then
                if w6_n = 0 then
                    z_hc16 <= w6_cand; z_hc2 <= w6_cand;
                elsif w6_cand > z_hc16 then
                    z_hc16 <= w6_cand;
                end if;
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
    -- slewed across by two one-pole filters (1/16 per pixel) and scaled by the
    -- source gain (Osc at twice the camera's).  All three filters decay, so none
    -- needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(16 downto 0);
        variable v_p : unsigned(17 downto 0);
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

            -- pixel side: the cell stream (the line above's state), two slews across, gain
            s_lv <= unsigned(s_lb_q(11 downto 0));
            v_e  := signed(resize(s_lv & "0000", 17)) - signed(resize(s_h1, 17));
            v_e  := signed(resize(s_h1, 17)) + shift_right(v_e, 4);
            s_h1 <= unsigned(v_e(15 downto 0));
            v_e  := signed(resize(s_h1, 17)) - signed(resize(s_h2, 17));
            v_e  := signed(resize(s_h2, 17)) + shift_right(v_e, 4);
            s_h2 <= unsigned(v_e(15 downto 0));
            s_la  <= s_h2(11 downto 8) * c_gl;
            s_lbh <= s_h2(15 downto 12) * c_gl;
            v_p := resize(s_la, 18) + shift_left(resize(s_lbh, 18), 4);   -- luma x gain
            if s_src = '1' then s_lterm <= v_p(17 downto 5); else s_lterm <= resize(v_p(17 downto 6), 13); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: p^2 (minus the saddle energy) by second differences, in Q24
    -- bands; loaded in the zone, latched by the beam and advanced at each line start
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ad : signed(26 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 338 then
                e_d <= resize(signed(f_acc(29 downto 4)), 26);
            elsif g_start = '1' then
                e_d <= e_d + signed(resize(c_dd, 26));
            end if;
            if s_zone = '1' and s_zc = 386 then
                e_q <= signed(resize(f_acc(30 downto 0), 34));                        -- dn x 2 x 540^2
            elsif s_zone = '1' and s_zc = 2471 then
                e_q <= e_q - shift_left(resize(z_hc16(25 downto 8), 34), 16);        -- minus the saddle energy
            elsif g_start = '1' then
                e_q <= e_q + resize(e_d, 34);
            end if;
            v_ad := abs(resize(e_d, 27));
            e_gy <= resize(unsigned(v_ad(26 downto 12)), 16);
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the window phase q + pi (2^26 per cycle) with its wrap flag; a
    -- delayed copy of the beam's control runs the tilt saw F q_loc (Q16) at
    -- stage 6 and the band window offset (Q8) at stage 7
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(26 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ph <= c_b0; b_w <= '0';
                l_lc <= e_q(33 downto 16);
                l_gy <= e_gy;
            elsif g_avid = '1' then
                v_n  := resize(b_ph, 27) + resize(c_inc, 27);
                b_ph <= v_n(25 downto 0); b_w <= v_n(26);
            end if;
            s_gst <= s_gst(4 downto 0) & g_start;
            p_w(2) <= b_w;
            for k in 3 to 6 loop p_w(k) <= p_w(k - 1); end loop;
            if s_gst(4) = '1' then
                p6_saw <= signed(resize(c_saw0, 25));
            elsif s_gav_sr(4) = '1' then
                if p_w(5) = '1' then p6_saw <= p6_saw + c_fsw;
                else p6_saw <= p6_saw + signed(resize(c_finc, 25)); end if;
            end if;
            if s_gst(5) = '1' then
                p7_wf <= c_wf0;
            elsif s_gav_sr(5) = '1' and p_w(6) = '1' then
                p7_wf <= p7_wf + c_fw8;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_i   : unsigned(9 downto 0);
        variable v_s   : signed(18 downto 0);
        variable v_a   : unsigned(21 downto 0);
        variable v_o   : integer range 0 to 11;
        variable v_w   : std_logic_vector(31 downto 0);
        variable v_t   : std_logic_vector(31 downto 0);
        variable v_u, v_v : signed(11 downto 0);
        variable v_fr  : unsigned(8 downto 0);
        variable v_g   : signed(16 downto 0);
        variable v_b   : signed(10 downto 0);
        variable v_sh  : integer range 0 to 2;
        variable v_u2, v_v2 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: table address (q folded onto 0 .. pi; the slope's sign flips on the far half)
            v_i := (not b_ph(25)) & b_ph(24 downto 16);
            if v_i(9) = '1' then r_t_a <= not v_i(8 downto 0); else r_t_a <= v_i(8 downto 0); end if;
            p2_sg <= v_i(9);
            -- 3: table (EBR)
            p3_sg <= p2_sg;
            -- 4: fabric copy
            p4_v <= signed(r_t_q(31 downto 16)); p4_d <= signed(r_t_q(15 downto 0)); p4_sg <= p3_sg;
            -- 5: p^2 + V ; the x slope minus the tilt
            p5_s1 <= resize(l_lc, 19) + resize(p4_v, 19);
            if p4_sg = '1' then p5_gxs <= resize(p4_d, 17) + signed(resize(c_fx, 17));
            else p5_gxs <= resize(p4_d, 17) - signed(resize(c_fx, 17)); end if;
            -- 6: + the source ; |slope x|
            p6_s2 <= p5_s1 + signed(resize(s_lterm, 19));
            v_g := abs(p5_gxs); p6_gx <= unsigned(v_g(15 downto 0));
            -- 7: minus the tilt saw = H - Hc (the separatrix comparator's input) ; gradient max / min
            p7_sep <= p6_s2 - resize(p6_saw(24 downto 8), 19);
            if p6_gx > l_gy then p7_gmax <= p6_gx; p7_gmin <= l_gy; else p7_gmax <= l_gy; p7_gmin <= p6_gx; end if;
            -- 8: band phase ; |H - Hc| ; the inner (hump) comparator's input ; |grad H| ~ max + min / 2
            p8_band <= unsigned(p7_sep(9 downto 0)) - p7_wf;
            v_s := abs(p7_sep); p8_asep <= unsigned(v_s(17 downto 0));
            v_s := p7_sep + resize(c_dh2, 19); v_s := abs(v_s); p8_asep2 <= unsigned(v_s(17 downto 0));
            p8_g    <= resize(p7_gmax, 17) + resize(p7_gmin(15 downto 1), 17);
            p8_in   <= p7_sep(18) and s_sep;
            -- 9: the nearer separatrix (the hump's only in a double well) ; comparator window (4 virtual px) ;
            --    density class ; orbit line distance ; colouriser address
            if c_e2 = '1' and p8_asep2 < p8_asep then p9_amin <= p8_asep2; else p9_amin <= p8_asep; end if;
            case to_integer(s_xs) is
                when 2      => p9_thr <= shift_left(resize(p8_g, 19), 2);
                when 3      => p9_thr <= shift_left(resize(p8_g, 19), 1) + resize(p8_g(16 downto 1), 19);
                when others => p9_thr <= shift_left(resize(p8_g, 19), 1);
            end case;
            if p8_g(16 downto 9) = 0 then p9_cls <= "00";
            elsif p8_g(16 downto 10) = 0 then p9_cls <= "01";
            elsif p8_g(16 downto 11) = 0 then p9_cls <= "10";
            else p9_cls <= "11"; end if;
            v_fr := to_unsigned(256, 9) - resize(p8_band(7 downto 0), 9);
            if resize(p8_band(7 downto 0), 9) < v_fr then p9_db <= p8_band(7 downto 0); else p9_db <= v_fr(7 downto 0); end if;
            p9_g <= p8_g;
            r_p_a(9) <= s_pal; r_p_a(8) <= p8_in;
            if p8_g(16 downto 9) = 0 then r_p_a(7 downto 6) <= "00";
            elsif p8_g(16 downto 10) = 0 then r_p_a(7 downto 6) <= "01";
            elsif p8_g(16 downto 11) = 0 then r_p_a(7 downto 6) <= "10";
            else r_p_a(7 downto 6) <= "11"; end if;
            r_p_a(5 downto 0) <= p8_band(9 downto 4);
            p9_in <= p8_in;
            -- 10: colouriser (EBR) ; window compares -> the line level (3 core, 2 glow, 1 halo) ; orbit line
            v_a := shift_left(resize(p9_amin, 22), 4);
            if s_sep = '0' then p10_lv <= "00";
            elsif v_a < resize(p9_thr, 22) then p10_lv <= "11";
            elsif v_a < shift_left(resize(p9_thr, 22), 1) then p10_lv <= "10";
            elsif v_a < shift_left(resize(p9_thr, 22), 2) then p10_lv <= "01";
            else p10_lv <= "00"; end if;
            if shift_left(resize(p9_db, 17), 4) < p9_g then p10_od <= '1'; else p10_od <= '0'; end if;
            p10_cls <= p9_cls; p10_in <= p9_in;
            -- 11: fabric copy
            p11_c <= r_p_q; p11_lv <= p10_lv;
            p11_od <= p10_od; p11_cls <= p10_cls; p11_in <= p10_in;
            -- 12: base colour (the band, or the Orbits look) ; target - base
            if s_look = '1' then
                v_o := 0;
                if s_pal = '1' then v_o := 6; end if;
                if p11_in = '1' then v_o := v_o + 3; end if;
                if p11_cls = "11" then v_o := v_o + 2; elsif p11_cls = "10" then v_o := v_o + 1; end if;
                if p11_cls = "11" or p11_od = '1' then v_w := C_SX_ORB(v_o); else v_w := C_SX_K; end if;
            else
                v_w := p11_c;
            end if;
            if p11_lv = "11" then v_t := C_SX_WHITE; else v_t := C_SX_GLOW; end if;
            p12_by <= unsigned(v_w(31 downto 22)); p12_bu <= unsigned(v_w(21 downto 12)); p12_bv <= unsigned(v_w(11 downto 2));
            p12_dy <= signed(resize(unsigned(v_t(31 downto 22)), 11)) - signed(resize(unsigned(v_w(31 downto 22)), 11));
            p12_du <= signed(resize(unsigned(v_t(21 downto 12)), 11)) - signed(resize(unsigned(v_w(21 downto 12)), 11));
            p12_dv <= signed(resize(unsigned(v_t(11 downto 2)), 11)) - signed(resize(unsigned(v_w(11 downto 2)), 11));
            p12_lv <= p11_lv;
            -- 13: blend base + (target - base) x {0, 1/4, 1/2, 1}
            if p12_lv = "00" then
                p13_y <= p12_by; p13_u <= p12_bu; p13_v <= p12_bv;
            else
                if p12_lv = "11" then v_sh := 0; elsif p12_lv = "10" then v_sh := 1; else v_sh := 2; end if;
                v_b := signed(resize(p12_by, 11)) + shift_right(p12_dy, v_sh); p13_y <= unsigned(v_b(9 downto 0));
                v_b := signed(resize(p12_bu, 11)) + shift_right(p12_du, v_sh); p13_u <= unsigned(v_b(9 downto 0));
                v_b := signed(resize(p12_bv, 11)) + shift_right(p12_dv, v_sh); p13_v <= unsigned(v_b(9 downto 0));
            end if;
            -- 14: Colour (a chroma transform: swap U / V, invert U, invert V); black off the generated avid
            v_u2 := p13_u; v_v2 := p13_v;
            if c_k5z(0) = '1' then v_u2 := p13_v; v_v2 := p13_u; end if;
            if c_k5z(1) = '1' then v_u2 := not v_u2; end if;
            if c_k5z(2) = '1' then v_v2 := not v_v2; end if;
            if s_gav_sr(11) = '0' then
                s_y14 <= C_KY; s_u14 <= C_KU; s_v14 <= C_KV;
            else
                s_y14 <= p13_y; s_u14 <= v_u2; s_v14 <= v_v2;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(11);
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
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y14) & std_logic_vector(s_u14) & std_logic_vector(s_v14);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y16) & std_logic_vector(s_u16) & std_logic_vector(s_v16);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    p_vs : process(clk)
        variable v_s : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(15) = '1' then v_k17 <= v_k17 + 1; else v_k17 <= (others => '0'); end if;   -- column of the stage-17 pixel
            if s_gav_sr(17) = '1' then v_k19 <= v_k19 + 1; else v_k19 <= (others => '0'); end if;   -- column of the stage-19 pixel
            v_qy <= vy(to_integer(v_k17(10 downto 1)));
            v_qc <= vc(to_integer(v_k17(10 downto 2)));
            v_fy <= v_qy; v_fc <= v_qc;
            v_py <= s_y19; v_pu <= s_u19; v_pv <= s_v19;
            v_wye <= s_gav_sr(17) and v_k19(0);
            v_wya <= v_k19(10 downto 1);
            v_s := resize(s_y19, 11) + resize(v_py, 11); v_wyd <= std_logic_vector(v_s(10 downto 3));
            if s_gav_sr(17) = '1' and v_k19(1 downto 0) = "01" then v_wce <= '1'; else v_wce <= '0'; end if;
            v_wca <= v_k19(10 downto 2);
            v_s := resize(s_u19, 11) + resize(v_pu, 11); v_wcd(15 downto 8) <= std_logic_vector(v_s(10 downto 3));
            v_s := resize(s_v19, 11) + resize(v_pv, 11); v_wcd(7 downto 0) <= std_logic_vector(v_s(10 downto 3));
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
            -- box A: stage 15 sums, stage 16 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y14 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u14 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v14 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(13) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y16 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u16 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v16 <= v_sc(13 downto 4);
            else
                s_y16 <= C_KY; s_u16 <= C_KU; s_v16 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(13);
            -- box B: stage 17 sums, stage 18 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y16 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u16 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v16 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y18 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v18 <= v_sc(13 downto 4);
            -- 19: luma 13/16
            if s_y18 > 64 then v_dl := s_y18 - 64; else v_dl := (others => '0'); end if;
            s_y19 <= s_y18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u19 <= s_u18; s_v19 <= s_v18;
            -- 20: vertical mix with the line above
            v_mx := resize(s_y19, 11) + resize(unsigned(v_fy) & "10", 11);             s_y20 <= v_mx(10 downto 1);
            v_mx := resize(s_u19, 11) + resize(unsigned(v_fc(15 downto 8)) & "10", 11); s_u20 <= v_mx(10 downto 1);
            v_mx := resize(s_v19, 11) + resize(unsigned(v_fc(7 downto 0)) & "10", 11);  s_v20 <= v_mx(10 downto 1);
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

end architecture separatrix;
