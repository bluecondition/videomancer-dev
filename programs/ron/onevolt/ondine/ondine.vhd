-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ondine v0.1.0 -- Onevolt 3: the Syntonie oscillator voice.
--
-- The rack (Syntonie): Ondes (two video-rate oscillators) -> Formes (wave shaper) -> Combines (combiner) ->
-- Cadrans (the Fold gain) -> Solaire (four full-wave rectifiers, three mixers) -> Matrice (3 x 3 matrix) -> Sortie.
-- Animate (quadrature LFO) and a VU006 sine as slow modulators; Entree decodes the camera.
--
-- The patch:
--   Ondes A, SYNC H (hard reset at every line start): FREQ A (K1); FM in = Ondes B through the FM A attenuverter
--     (K3) + the camera luma (K6) + VU006 (29 s); AM in = Ondes B through an attenuverter (K4; four-quadrant:
--     full, null, inverted); NORM A (S7): +1V (triangle) / SQR (own square into AM: a saw at twice the rate)
--   Ondes B, SYNC V: FREQ B (K2), Animate sine (13 s) into its FM
--   Formes ch 1 / ch 2 (A / B): Sine or Trapezoid outputs (S8) -> Combines ch 1: AVG or XOR (|A - B|) (S9)
--   Cadrans (P12 Fold): the drive into Solaire rectifier 1, and (multed) the level injected between the folds
--   Solaire mixers B = Ondes B (AC) + Animate cosine + the camera luma; mixer switches ADD / SUB (S10)
--   Matrice (K5): Combines out, rectifier 2 out (1 px later) and rectifier 4 out (2 px later) -> R, G, B
--
--   K1 Freq A    1 .. 20 cycles across (exponential)        S7  Norm A    +1V / SQR
--   K2 Freq B    1 .. 12 cycles down (exponential)          S8  Formes    Sine / Trapezoid
--   K3 FM A      bipolar attenuverter (square law)          S9  Combines  AVG / XOR
--   K4 AM A      bipolar attenuverter                       S10 Solaire   ADD / SUB
--   K5 Colour    Matrice: eight colourings, blended         S11 Source    Video / Osc
--   K6 Camera    Entree level into FM A and the injection   P12 FOLD      Solaire drive x1/16 .. x1
--
-- Engine: vblank-zone sequencer (one serial multiplier, 14 slots, shared glide, the LFOs), then a table writer
-- (three per-field colour tables, one per Matrice input, each 256 x {Y, U, V}); a per-line engine (Ondes B, the
-- AM gain, the FM step, the injected level; one serial multiplier); a camera path (8-px cells, vertical and
-- horizontal slews); a 26-stage pixel path (Ondes A phase from 0 at the line start, AM, Formes, Combines, the
-- Fold gain, four rectifiers, fades of over-fine rings, the three table reads).  Virtual 1080-line coordinates.
-- Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line
-- resync, luma 13/16, a vertical [1 1]/2 with the line above) before the blanking gate.  Colours in the hardware
-- convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.ondine_pkg.all;
use work.all;

architecture ondine of program_top is
    constant C_LAT   : integer := 28;   -- = processing_delay_clks
    constant C_FLOOR : integer := 34953; -- one cycle across the width (2^26 / 1920)

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
    -- 16-bit triangle of a 16-bit phase
    function f_tri16(p : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '0' then v := p(14 downto 0) & '0'; else v := (not p(14 downto 0)) & '0'; end if;
        return v;
    end function;
    -- Ondes linear out: 16-bit triangle of a 26-bit phase (2^26 per cycle)
    function f_tri26(p : unsigned(25 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        if p(25) = '1' then v := not p(24 downto 9); else v := p(24 downto 9); end if;
        return v;
    end function;
    -- Solaire rectifier (frequency doubler): 1 - |2 frac(v) - 1| on a 16-bit value
    function f_fold(f : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        if f(15) = '0' then v := f(14 downto 0) & '0'; else v := (not f(14 downto 0)) & '1'; end if;
        return v;
    end function;
    -- Formes trapezoid: gain 4 and hard clip, clip(4 a - 1.5)
    function f_trap16(a : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
        variable d : unsigned(15 downto 0);
    begin
        d := a - to_unsigned(24576, 16);
        if a < 24576 then v := (others => '0');
        elsif a >= 40960 then v := (others => '1');
        else v := d(13 downto 0) & "00"; end if;
        return v;
    end function;
    -- ring fade toward the mean: 128 + (r / 256 - 128) x fr / 8, fr = 0 .. 8, as shift-adds
    function f_fade(r : unsigned(15 downto 0); fr : unsigned(3 downto 0)) return unsigned is
        variable v, t : signed(7 downto 0);
        variable w : unsigned(7 downto 0);
    begin
        v := signed((not r(15)) & r(14 downto 8));
        t := (others => '0');
        if fr(2) = '1' then t := t + shift_right(v, 1); end if;
        if fr(1) = '1' then t := t + shift_right(v, 2); end if;
        if fr(0) = '1' then t := t + shift_right(v, 3); end if;
        if fr(3) = '1' then t := v; end if;
        w := unsigned((not t(7)) & t(6 downto 0));
        return w;
    end function;
    -- radix-4 Booth multiply, signed a (13 bits) x signed b (10 bits): five partial products of 0, +-a, +-2a,
    -- the negations as inverted rows plus one correction word
    function f_booth(a : signed(12 downto 0); b : signed(9 downto 0)) return signed is
        variable v_s : signed(22 downto 0);
        variable v_p, v_c : signed(22 downto 0);
        variable v_m : signed(14 downto 0);
        variable v_t : std_logic_vector(2 downto 0);
        variable v_n : std_logic;
    begin
        v_s := (others => '0'); v_c := (others => '0');
        for i in 0 to 4 loop
            if i = 0 then v_t := b(1) & b(0) & '0'; else v_t := b(2 * i + 1) & b(2 * i) & b(2 * i - 1); end if;
            case v_t is
                when "001" | "010" => v_m := resize(a, 15);                     v_n := '0';
                when "011"         => v_m := shift_left(resize(a, 15), 1);      v_n := '0';
                when "100"         => v_m := shift_left(resize(a, 15), 1);      v_n := '1';
                when "101" | "110" => v_m := resize(a, 15);                     v_n := '1';
                when others        => v_m := (others => '0');                   v_n := '0';
            end case;
            v_p := resize(v_m, 23);
            if v_n = '1' then v_p := not v_p; v_c(2 * i) := '1'; end if;
            v_s := v_s + shift_left(v_p, 2 * i);
        end loop;
        return v_s + v_c;
    end function;
    -- sine ROM index of a 16-bit value (half-range table, mirrored)
    function f_sidx(a : unsigned(15 downto 0)) return unsigned is
    begin
        if a(15) = '1' then return not a(14 downto 6); else return a(14 downto 6); end if;
    end function;

    -- colour tables (Matrice input k x Y, U, V: index 3 k + channel), 11-bit signed words, written in the zone
    type t_tt is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_tq is array (0 to 8) of std_logic_vector(15 downto 0);
    attribute no_rw_check : boolean;
    signal t0_a, t1_a, t2_a : unsigned(7 downto 0) := (others => '0');
    signal t_q   : t_tq := (others => (others => '0'));
    signal tw_a  : unsigned(7 downto 0) := (others => '0');
    signal tw_d  : std_logic_vector(15 downto 0) := (others => '0');
    signal tw_we : std_logic_vector(8 downto 0) := (others => '0');
    -- luma cell store: one 12-bit vertical-slew state per 8-pixel cell (read a cell ahead of the write)
    type t_lb is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
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
    signal s_h1, s_h2 : unsigned(23 downto 0) := (others => '0');
    signal g_c8  : unsigned(2 downto 0) := (others => '0');
    signal g_ma, g_macc, s_lvg : unsigned(19 downto 0) := (others => '0');
    signal g_mb  : unsigned(7 downto 0) := (others => '0');
    -- camera stream: Ondes A's pixel step (line step + luma x FM gain, floored), the camera's share of the
    -- Solaire injection, the local ring density and the two ring fades
    signal s_cp  : unsigned(19 downto 0) := (others => '0');
    signal s_sv  : signed(26 downto 0) := (others => '0');
    signal s_spx : unsigned(25 downto 0) := to_unsigned(C_FLOOR, 26);
    signal s_civ, s_injp : unsigned(15 downto 0) := (others => '0');

    -- ROMs (registered address and data, then a fabric copy)
    signal r_s_a : unsigned(8 downto 0) := (others => '0');
    signal r_s_q, r_s_f : unsigned(15 downto 0) := (others => '0');
    signal r_e_a : unsigned(7 downto 0) := (others => '0');
    signal r_e_q, r_e_f : unsigned(15 downto 0) := (others => '0');
    signal r_c_a : unsigned(6 downto 0) := (others => '0');
    signal r_c_q, r_c_f : std_logic_vector(15 downto 0) := (others => '0');

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
    signal vy : t_vy := (others => x"08");
    signal vc : t_vc := (others => x"8080");
    attribute no_rw_check of vy : signal is true;
    attribute no_rw_check of vc : signal is true;
    signal v_k29, v_k31 : unsigned(10 downto 0) := (others => '0');
    signal v_qy, v_fy, v_wyd : std_logic_vector(7 downto 0) := x"08";
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
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(560 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(455 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(772 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(960 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(146 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(680 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_s7, s_s8, s_s9, s_s10, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal c_aph  : unsigned(15 downto 0) := (others => '0');            -- Animate (13 s)
    signal c_rc   : unsigned(15 downto 0) := (others => '0');            -- Animate cosine (injection)
    signal c_asn  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_lean : signed(4 downto 0) := (others => '0');
    signal z_d3, z_a4 : unsigned(8 downto 0) := (others => '0');
    signal c_n3, c_n4 : std_logic := '0';
    signal c_gm   : unsigned(15 downto 0) := to_unsigned(32768, 16);    -- Fold gain mantissa (Q15), shift c_gs
    signal c_gs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal z_e1   : unsigned(10 downto 0) := (others => '0');
    signal z_e2   : unsigned(9 downto 0) := (others => '0');
    signal c_stepva : unsigned(21 downto 0) := to_unsigned(C_FLOOR, 22); -- Ondes A step, Q26 cycles per pixel
    signal z_fbv  : unsigned(19 downto 0) := (others => '0');
    signal c_stepb : unsigned(23 downto 0) := (others => '0');          -- Ondes B step per field line, Q26
    signal z_d2   : unsigned(17 downto 0) := (others => '0');
    signal c_fmv  : unsigned(19 downto 0) := (others => '0');
    signal c_an   : unsigned(9 downto 0) := (others => '0');
    signal c_gi   : unsigned(15 downto 0) := (others => '0');
    signal c_gq   : unsigned(11 downto 0) := (others => '0');            -- camera FM gain (pixel units, >> 5)
    signal c_flr  : unsigned(16 downto 0) := to_unsigned(C_FLOOR, 17);   -- one cycle across, pixel units
    -- per mode (virtual px per pixel 1, 1.5, 2.656): FM depth and camera FM gain constants (step A uses c_flr / 2)
    signal c_kf   : unsigned(11 downto 0) := to_unsigned(1415, 12);
    signal c_kq   : unsigned(15 downto 0) := to_unsigned(17218, 16);
    signal c_kg8  : unsigned(7 downto 0) := (others => '0');
    signal z_x10  : unsigned(12 downto 0) := (others => '0');
    signal z_k1   : unsigned(11 downto 0) := (others => '0');
    signal z_ssum : unsigned(23 downto 0) := (others => '0');
    signal z_s8f  : unsigned(7 downto 0) := (others => '0');
    signal c_fr2, c_fr4 : unsigned(3 downto 0) := to_unsigned(8, 4);    -- ring fades (per field)
    signal z_k2   : unsigned(10 downto 0) := (others => '0');
    signal z_j    : unsigned(2 downto 0) := (others => '0');
    signal z_f10  : unsigned(10 downto 0) := (others => '0');
    signal z_fm   : signed(11 downto 0) := (others => '0');
    signal z_f5   : signed(13 downto 0) := (others => '0');
    signal c_w    : unsigned(8 downto 0) := (others => '0');

    -- field serial multiply: acc = a x b
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(31 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

    -- table writer
    signal w_run, w_tab : std_logic := '0';
    signal w_k   : unsigned(1 downto 0) := (others => '0');
    signal w_ln  : unsigned(1 downto 0) := (others => '0');
    signal wa_i  : unsigned(3 downto 0) := (others => '0');
    signal wa_s5 : unsigned(4 downto 0) := (others => '0');
    signal w_ca  : signed(15 downto 0) := (others => '0');
    signal w_d   : signed(13 downto 0) := (others => '0');
    signal w_rq, w_sg : std_logic := '0';
    signal w_opd : unsigned(13 downto 0) := (others => '0');
    signal wb_v  : unsigned(8 downto 0) := (others => '0');
    signal w0_v, w1_v, w2_v : unsigned(7 downto 0) := (others => '0');
    signal w0_ok, w1_ok, w2_ok : std_logic := '0';
    signal w0_inc, w1_inc : std_logic := '0';
    signal w_ly  : signed(12 downto 0) := (others => '0');
    signal w_py  : signed(18 downto 0) := (others => '0');
    signal w_tsel : unsigned(3 downto 0) := (others => '0');
    signal c_oy, c_ou, c_ov : signed(12 downto 0) := to_signed(512, 13);

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (computes the NEXT line), one serial multiplier
    signal e_zk  : std_logic := '0';
    signal e_lc  : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_phb : unsigned(25 downto 0) := (others => '0');
    signal n_bl  : unsigned(15 downto 0) := (others => '0');
    signal e_xa, e_ya : unsigned(11 downto 0) := (others => '0');
    signal e_xn, e_yn : std_logic := '0';
    signal e_rq  : std_logic := '0';                                   -- a product on the field multiplier
    signal e_opa, e_opb : unsigned(11 downto 0) := (others => '0');
    signal n_m   : signed(8 downto 0) := to_signed(128, 9);
    signal n_stepl : signed(24 downto 0) := to_signed(C_FLOOR, 25);
    signal n_inj : unsigned(15 downto 0) := (others => '0');
    signal n_bfs : unsigned(15 downto 0) := (others => '0');
    signal y_cnt : unsigned(4 downto 0) := (others => '0');
    signal y_sel : std_logic := '0';
    -- the current line's values (loaded 20 clocks after the generated avid falls)
    signal l_stepl : signed(24 downto 0) := to_signed(C_FLOOR, 25);    -- the line's Ondes A step, pixel units
    signal l_m   : signed(8 downto 0) := to_signed(128, 9);             -- AM gain, Q7
    signal l_bf  : unsigned(15 downto 0) := (others => '0');
    signal l_inj : unsigned(15 downto 0) := (others => '0');

    -- beam (stage 1): Ondes A phase from 0 at the line start, the pixel's step values and camera luma
    signal b_ph, b_spx : unsigned(25 downto 0) := (others => '0');

    -- pixel path
    signal p2_a16 : unsigned(15 downto 0) := (others => '0');
    signal p2_bw, p3_bw : unsigned(3 downto 0) := (others => '0');
    signal p3_amp : signed(20 downto 0) := (others => '0');
    signal p4_u1, p4_u2 : signed(17 downto 0) := (others => '0');
    signal p5_ap  : unsigned(15 downto 0) := (others => '0');
    signal p6_up, p7_up, p8_up : std_logic := '0';
    signal p6_tr, p7_tr, p8_tr : unsigned(15 downto 0) := (others => '0');
    signal p9_sum : unsigned(16 downto 0) := (others => '0');
    signal p9_dif : signed(16 downto 0) := (others => '0');
    signal p10_cc : unsigned(15 downto 0) := (others => '0');
    signal p11_x0p : unsigned(21 downto 0) := (others => '0');
    signal p12_r1, p13_r2, p14_r3, p15_r4 : unsigned(15 downto 0) := (others => '0');
    signal p14_a1, p15_a1, p16_a1, p17_a1 : unsigned(7 downto 0) := (others => '0');
    signal p16_a2, p17_a2, p18_a2 : unsigned(7 downto 0) := (others => '0');
    type t_a8 is array (natural range <>) of unsigned(7 downto 0);
    signal pa0 : t_a8(11 to 16) := (others => (others => '0'));      -- the Combines tap, carried
    type t_s11 is array (0 to 8) of signed(10 downto 0);
    signal p19_t : t_s11 := (others => (others => '0'));
    signal p20_ya, p20_yb, p20_ua, p20_ub, p20_va, p20_vb : signed(12 downto 0) := (others => '0');
    signal p21_y, p21_u, p21_v : signed(12 downto 0) := (others => '0');
    signal s_y26, s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y28, s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y30, s_u30, s_v30 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y31, s_u31, s_v31 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y32, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
    -- Memories
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    -- the colour tables: written only in the zone (the pixel path's reads there are unused)
    g_tab : for i in 0 to 8 generate
        signal tt : t_tt := (others => (others => '0'));
        attribute no_rw_check of tt : signal is true;
    begin
        process(clk)
        begin
            if rising_edge(clk) then
                if i < 3 then t_q(i) <= tt(to_integer(t0_a));
                elsif i < 6 then t_q(i) <= tt(to_integer(t1_a));
                else t_q(i) <= tt(to_integer(t2_a)); end if;
                if tw_we(i) = '1' then tt(to_integer(tw_a)) <= tw_d; end if;
            end if;
        end process;
    end generate g_tab;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_s_q <= C_OD_SIN(to_integer(r_s_a));
            r_e_q <= C_OD_EXP(to_integer(r_e_a));
            r_c_q <= C_OD_COL(to_integer(r_c_a));
        end if;
    end process p_rom;

    p_romf : process(clk)
    begin
        if rising_edge(clk) then
            r_s_f <= r_s_q; r_e_f <= r_e_q; r_c_f <= r_c_q;
        end if;
    end process p_romf;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : signed(12 downto 0);
        variable v_dn : unsigned(9 downto 0);
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
                s_s7  <= registers_in(6)(0);
                s_s8  <= registers_in(6)(1);
                s_s9  <= registers_in(6)(2);
                s_s10 <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                c_aph <= c_aph + 84;
                case to_integer(s_xsr) is     -- per mode: Osc lattice step; virtual px per pixel (Q5); floor; FM gain
                    when 4 =>
                        s_oxs <= to_unsigned(204, 9); c_flr <= to_unsigned(92843, 17);
                        c_kf <= to_unsigned(3758, 12); c_kq <= to_unsigned(45735, 16);
                    when 3 =>
                        s_oxs <= to_unsigned(153, 9); c_flr <= to_unsigned(52429, 17);
                        c_kf <= to_unsigned(2122, 12); c_kq <= to_unsigned(25827, 16);
                    when others =>
                        s_oxs <= to_unsigned(102, 9); c_flr <= to_unsigned(34953, 17);
                        c_kf <= to_unsigned(1415, 12); c_kq <= to_unsigned(17218, 16);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run from the zone
            -- start by its own phase / channel counters
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

            -- serial multiply slots: slot k loads at zc = 42 + 24 k, 16 iterations, result stable from load + 17
            -- (stored at load + 18).  The slot number is decoded at 40 + 24 k and the operands are pre-muxed
            -- at 41 + 24 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc >= 40 and s_zc <= 232 then
                case to_integer(s_zc) is
                    when 40  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 64  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 88  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 112 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 136 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 160 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 184 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 208 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 232 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre or e_rq or w_rq;
            if e_rq = '1' then
                z_fa <= resize(e_opa, 32); z_fb <= resize(e_opb, 16);
            elsif w_rq = '1' then
                z_fa <= resize(w_opd, 32); z_fb <= resize(c_w, 16);
            elsif z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(c_flr(16 downto 1), 32); z_fb <= r_e_f;                  -- step A
                    when 1  => z_fa <= to_unsigned(62138, 32);        z_fb <= r_e_f;                   -- Freq B
                    when 2  => z_fa <= resize(z_fbv, 32);             z_fb <= resize(s_ys, 16);        -- step B / line
                    when 3  => z_fa <= resize(z_d3, 32);              z_fb <= resize(z_d3, 16);        -- FM A ^2
                    when 4  => z_fa <= resize(z_d2, 32);              z_fb <= resize(c_kf, 16);        -- FM depth
                    when 5  => z_fa <= resize(c_gm, 32);              z_fb <= to_unsigned(19661, 16);  -- injection level
                    when 6  => z_fa <= resize(s_g6(12 downto 3), 32); z_fb <= c_kq;                    -- camera FM gain
                    when 8  => z_fa <= resize(z_s8f, 32);             z_fb <= resize(c_kg8, 16);       -- density x folds
                    when others =>                                                                    -- fold count
                        z_fa <= resize(c_gm(15 downto 4), 32);
                        if s_s8 = '0' and s_s9 = '0' then z_fb <= to_unsigned(683, 16);
                        elsif s_s8 = '1' and s_s9 = '0' then z_fb <= to_unsigned(1707, 16);
                        elsif s_s8 = '0' then z_fb <= to_unsigned(1365, 16);
                        else z_fb <= to_unsigned(3413, 16); end if;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- the LFOs: Animate's triangle outputs (0 and 90 degrees) and VU006
                    when 2 =>
                        c_rc  <= f_tri16(c_aph);
                        c_asn <= f_tri16(c_aph + 16384);
                    -- knob decodes (after the glide)
                    when 30 =>
                        z_lean <= signed((not c_asn(15)) & c_asn(14 downto 11));
                        z_d3 <= f_bm(s_g3(12 downto 3)); c_n3 <= not s_g3(12);
                        z_a4 <= f_bm(s_g4(12 downto 3)); c_n4 <= not s_g4(12);
                        c_gs <= to_unsigned(4, 3) - resize(s_gp(12 downto 11), 3);
                        z_x10 <= shift_left(resize(s_g5(12 downto 3), 13), 3) - resize(s_g5(12 downto 3), 13);
                        z_k1 <= resize(s_g1(12 downto 3), 12) + resize(s_g1(12 downto 7), 12) + resize(s_g1(12 downto 9), 12);
                        z_k2 <= resize(s_g2(12 downto 3), 11) - resize(s_g2(12 downto 6), 11);
                    when 31 =>                          -- Freq A: 1 .. 19.8 cycles across; Freq B: 1 .. 11.3 down
                        z_e1 <= resize(z_k1, 11);
                        v_e := signed(resize(z_k2, 13)) + resize(z_lean, 13);
                        if v_e < 0 then z_e2 <= (others => '0');
                        elsif v_e > 1023 then z_e2 <= to_unsigned(1023, 10);
                        else z_e2 <= unsigned(v_e(9 downto 0)); end if;
                        c_an <= shift_left(resize(z_a4, 10), 1) + resize(z_a4(8 downto 5), 10);
                        z_j <= z_x10(12 downto 10); z_f10 <= resize(z_x10(9 downto 0), 11);
                    when 32 => r_e_a <= z_e1(7 downto 0); z_fm <= signed(resize(z_f10, 12)) - 307;
                    when 33 => z_f5 <= shift_left(resize(z_fm, 14), 2) + resize(z_fm, 14);
                    when 34 =>                          -- Colour: hold 30 %, blend over 40 %: w = clip(5 (f - 307) / 8, 0, 256)
                        if z_f5 < 0 then c_w <= (others => '0');
                        elsif z_f5 >= 2048 then c_w <= to_unsigned(256, 9);
                        else c_w <= unsigned(z_f5(11 downto 3)); end if;
                    when 42 => r_e_a <= z_e2(7 downto 0);
                    when 60 =>                          -- step A = KA x 2^(e1 / 256) (pixel units)
                        case to_integer(z_e1(10 downto 8)) is
                            when 0      => c_stepva <= resize(f_acc(31 downto 14), 22);
                            when 1      => c_stepva <= resize(f_acc(31 downto 13), 22);
                            when 2      => c_stepva <= resize(f_acc(31 downto 12), 22);
                            when 3      => c_stepva <= resize(f_acc(31 downto 11), 22);
                            when others => c_stepva <= f_acc(31 downto 10);
                        end case;
                    when 70 => r_e_a <= s_gp(10 downto 3);
                    when 74 => c_gm <= r_e_f;
                    when 84 =>
                        case to_integer(z_e2(9 downto 8)) is
                            when 0      => z_fbv <= resize(f_acc(31 downto 15), 20);
                            when 1      => z_fbv <= resize(f_acc(31 downto 14), 20);
                            when 2      => z_fbv <= resize(f_acc(31 downto 13), 20);
                            when others => z_fbv <= f_acc(31 downto 12);
                        end case;
                    when 108 => c_stepb <= f_acc(31 downto 8);
                    when 132 => z_d2 <= f_acc(17 downto 0);
                    when 156 => c_fmv <= f_acc(29 downto 10);
                    when 180 =>                         -- injected level 0.6 G (pre-shifted by the Fold octave)
                        case to_integer(c_gs) is
                            when 1      => c_gi <= f_acc(30 downto 15);
                            when 2      => c_gi <= f_acc(31 downto 16);
                            when 3      => c_gi <= resize(f_acc(31 downto 17), 16);
                            when others => c_gi <= resize(f_acc(31 downto 18), 16);
                        end case;
                    when 204 =>                         -- camera FM gain (pixel units >> 5), x2 for Osc
                        if s_src = '1' then c_gq <= f_acc(25 downto 15) & '0';
                        else c_gq <= '0' & f_acc(25 downto 15); end if;
                    when 228 =>
                        case to_integer(c_gs) is
                            when 1      => c_kg8 <= f_acc(23 downto 16);
                            when 2      => c_kg8 <= f_acc(24 downto 17);
                            when 3      => c_kg8 <= f_acc(25 downto 18);
                            when others => c_kg8 <= f_acc(26 downto 19);
                        end case;
                        z_ssum <= resize(c_stepva, 24) + resize(c_fmv, 24);
                    when 229 => z_ssum <= z_ssum + c_stepb;
                    when 230 => z_s8f <= z_ssum(23 downto 16);       -- ring density of the field's fastest row
                    when 252 =>                         -- ring fades: full below a density of 8, gone at 16 (rectifier 4), 4x for 2
                        v_dn := f_acc(15 downto 6);
                        if v_dn >= 16 then c_fr4 <= (others => '0');
                        elsif v_dn <= 8 then c_fr4 <= to_unsigned(8, 4);
                        else c_fr4 <= resize(16 - v_dn(4 downto 0), 4); end if;
                        if v_dn(9 downto 2) >= 16 then c_fr2 <= (others => '0');
                        elsif v_dn(9 downto 2) <= 8 then c_fr2 <= to_unsigned(8, 4);
                        else c_fr2 <= resize(16 - v_dn(6 downto 2), 4); end if;
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
    -- Table writer (zone clocks 400 .. ~3140).  For k = 0 .. 2 and each channel c = Y, U, V: (A) the Matrice
    -- blend of crosspoint (k, c), x = a +- (|b - a| w >> 8) between settings j and j + 1 (ROM reads and a product
    -- on the field multiplier, 30 clocks); (B) colour table (k, c), T(v) = x cv(v) >> 8 with cv = clip(2 v - 128, 0, 254)
    -- (the taps' contrast curve, x2 about mid-grey), exact without a multiplier: the lane adds 2x for
    -- v = 65 .. 191.  Then the three offsets (k = 3).
    ---------------------------------------------------------------------------
    p_wr : process(clk)
        variable v_d : unsigned(7 downto 0);
        variable v_c : signed(12 downto 0);
        variable v_p : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            tw_we <= (others => '0');
            w_rq <= '0';
            if s_zone = '1' and s_zc = 400 then
                w_run <= '1'; w_tab <= '0'; w_k <= (others => '0'); w_ln <= (others => '0');
                wa_s5 <= (others => '0'); wb_v <= (others => '0');
            elsif w_run = '1' and w_tab = '0' then
                -- (A) blend crosspoint (w_k, w_ln)
                case to_integer(wa_s5) is
                    when 1 => r_c_a <= z_j & wa_i;                -- (wa_i follows w_k / w_ln a clock late)
                    when 4 => w_ca <= signed(r_c_f); r_c_a <= (z_j + 1) & wa_i;
                    when 7 => w_d <= resize(signed(r_c_f), 14) - resize(w_ca, 14);
                    when 8 =>                     -- |b - a| x w on the field multiplier (result at 27)
                        w_rq <= '1'; w_sg <= w_d(13);
                        if w_d(13) = '1' then w_opd <= unsigned(-w_d); else w_opd <= unsigned(w_d); end if;
                    when 27 =>
                        if w_sg = '1' then v_c := resize(w_ca, 13) - signed(resize(f_acc(20 downto 8), 13));
                        else v_c := resize(w_ca, 13) + signed(resize(f_acc(20 downto 8), 13)); end if;
                        if w_k = 3 then
                            case to_integer(w_ln) is
                                when 0      => c_oy <= v_c;
                                when 1      => c_ou <= v_c;
                                when others => c_ov <= v_c;
                            end case;
                        else
                            w_ly <= v_c;
                        end if;
                    when others => null;
                end case;
                if wa_s5 = 29 then
                    wa_s5 <= (others => '0');
                    if w_k = 3 then
                        if w_ln = 2 then w_run <= '0'; end if;
                        w_ln <= w_ln + 1;
                    else
                        w_tab <= '1';
                    end if;
                else
                    wa_s5 <= wa_s5 + 1;
                end if;
            elsif w_run = '1' then
                -- (B) table (w_k, w_ln): 264 clocks (entries v = 0 .. 255 enter at 4 .. 259)
                if wb_v = 263 then
                    wb_v <= (others => '0'); w_tab <= '0';
                    if w_ln = 2 then w_ln <= (others => '0'); w_k <= w_k + 1; else w_ln <= w_ln + 1; end if;
                else
                    wb_v <= wb_v + 1;
                end if;
            end if;
            -- the crosspoint index of (w_k, w_ln): Y k, U 3 + k, V 6 + k; offsets 9 .. 11
            if w_k = 3 then wa_i <= 9 + resize(w_ln, 4);
            else
                case to_integer(w_ln) is
                    when 0      => wa_i <= resize(w_k, 4);
                    when 1      => wa_i <= 3 + resize(w_k, 4);
                    when others => wa_i <= 6 + resize(w_k, 4);
                end case;
            end if;
            w_tsel <= resize(w_k, 4) + shift_left(resize(w_k, 4), 1) + resize(w_ln, 4);    -- 3 k + c

            -- B0: v (and whether the contrast curve rises at v)
            v_d := wb_v(7 downto 0) - 4;
            w0_v <= v_d;
            if w_run = '1' and w_tab = '1' and wb_v >= 4 and wb_v <= 259 then w0_ok <= '1'; else w0_ok <= '0'; end if;
            if v_d >= 65 and v_d <= 191 then w0_inc <= '1'; else w0_inc <= '0'; end if;
            -- B1
            w1_v <= w0_v; w1_ok <= w0_ok; w1_inc <= w0_inc;
            -- B2: accumulate the lane (cleared at the start of each table)
            if w_tab = '1' and wb_v = 0 then
                w_py <= (others => '0');
            elsif w1_ok = '1' and w1_inc = '1' then
                w_py <= w_py + shift_left(resize(w_ly, 19), 1);
            end if;
            w2_v <= w1_v; w2_ok <= w1_ok;
            -- B3: write
            tw_a <= w2_v;
            v_p := shift_right(w_py, 8);
            tw_d <= std_logic_vector(resize(v_p(10 downto 0), 16));
            if w2_ok = '1' and w_tsel <= 8 then tw_we(to_integer(w_tsel)) <= '1'; end if;
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
    -- Source side (Entree): oscillator (2-D triangle lattice, drifting) or
    -- camera luma ((Y >> 2) - 16), averaged over 8-pixel cells, each cell
    -- slewed down the screen (one pole, 1/16 per line; the state lives in the
    -- cell store), then slewed across by two one-pole filters (1/16 per pixel).
    -- All three filters decay, so none needs a resync.  Then the camera's
    -- share of Ondes A's FM: step = line step + luma x gain, floored at one
    -- cycle across the width (all in pixel units).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(16 downto 0);
        variable v_h  : signed(25 downto 0);
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
            -- 3: state += (cell average * 16 - state) / 16
            v_d := signed(resize(s_sum8 & '0', 13)) - signed(resize(s_old, 13));
            v_d := signed(resize(s_old, 13)) + shift_right(v_d, 4);
            s_new   <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);

            -- pixel side: the cell stream (the line above's state); the camera FM product once per 8-pixel cell
            -- (a serial 8-step multiply, (state >> 4) x gain, a cell late); then two slews across
            s_lv <= unsigned(s_lb_q(11 downto 0));
            if s_avq = '0' and data_in.avid = '1' then g_c8 <= to_unsigned(5, 3); else g_c8 <= g_c8 + 1; end if;
            if g_c8 = 7 then
                if g_mb(0) = '1' then s_lvg <= g_macc + g_ma; else s_lvg <= g_macc; end if;
                g_macc <= (others => '0'); g_ma <= resize(c_gq, 20); g_mb <= s_lv(11 downto 4);
            else
                if g_mb(0) = '1' then g_macc <= g_macc + g_ma; end if;
                g_ma <= g_ma(18 downto 0) & '0'; g_mb <= '0' & g_mb(7 downto 1);
            end if;
            v_h  := signed(resize(s_lvg & "0000", 26)) - signed(resize(s_h1, 26));
            v_h  := signed(resize(s_h1, 26)) + shift_right(v_h, 4);
            s_h1 <= unsigned(v_h(23 downto 0));
            v_h  := signed(resize(s_h1, 26)) - signed(resize(s_h2, 26));
            v_h  := signed(resize(s_h2, 26)) + shift_right(v_h, 4);
            s_h2 <= unsigned(v_h(23 downto 0));
            -- camera into Ondes A FM: luma x gain, + the line's step (pixel units), floor / cap; the camera's
            -- share of the Solaire injection (the same product, x 1.5 / 1 / 0.5 per mode: the same strength in
            -- virtual px), added to the line's injected level
            s_cp <= s_h2(23 downto 4);
            s_sv <= resize(l_stepl, 27) + signed(resize(s_cp & "00000", 27));
            if s_sv < signed(resize(c_flr, 27)) then s_spx <= resize(c_flr, 26);
            elsif s_sv(26 downto 25) = "01" then s_spx <= to_unsigned(33554431, 26);
            else s_spx <= unsigned(s_sv(25 downto 0)); end if;
            case to_integer(s_xs) is
                when 2      => s_civ <= s_cp(15 downto 0) + s_cp(16 downto 1);
                when 3      => s_civ <= s_cp(15 downto 0);
                when others => s_civ <= s_cp(16 downto 1);
            end case;
            s_injp <= l_inj + s_civ;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line: Ondes B's phase and triangle, the AM gain
    -- m = 1 - 2|K4| B, the FM step A + K3 (2B - 1), the injected level
    -- G (B - 1/2 + Animate cos) on one serial multiplier (16 iterations,
    -- result at load + 17); Formes B through the sine ROM in hblank; all of
    -- it loaded into the current-line registers 20 clocks after the generated
    -- avid falls.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_x : signed(16 downto 0);
        variable v_s : signed(24 downto 0);
        variable v_m : signed(13 downto 0);
        variable v_bp : unsigned(15 downto 0);
        variable v_ij : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 3200 then e_zk <= '1'; end if;
            if e_zk = '1' or g_start = '1' then
                e_lc <= (others => '0');
                if e_zk = '1' then e_phb <= (others => '0'); else e_phb <= e_phb + resize(c_stepb, 26); end if;
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            e_rq <= '0';
            case to_integer(e_lc) is
                when 0 =>
                    n_bl <= f_tri26(e_phb);
                when 1 =>                     -- |2B - 1| and |B - Animate cos| (ones'-complement magnitudes)
                    e_xn <= (not n_bl(15)) xor c_n3;
                    if n_bl(15) = '1' then e_xa <= resize(n_bl(14 downto 4), 12);
                    else e_xa <= resize(not n_bl(14 downto 4), 12); end if;
                    v_x := signed(resize(n_bl, 17)) - signed(resize(c_rc, 17));
                    e_yn <= v_x(16);
                    if v_x(16) = '1' then e_ya <= unsigned(not v_x(15 downto 4)); else e_ya <= unsigned(v_x(15 downto 4)); end if;
                    if c_n4 = '1' then v_bp := not n_bl; else v_bp := n_bl; end if;
                    e_rq <= '1'; e_opa <= resize(c_an, 12); e_opb <= v_bp(15 downto 4);
                when 20 =>                    -- AM gain m = 1 - an B (Q12) -> Q7
                    v_m := to_signed(4096, 14) - signed(resize(f_acc(21 downto 9), 14));
                    n_m <= v_m(13 downto 5);
                    e_rq <= '1'; e_opa <= c_fmv(19 downto 8); e_opb <= e_xa;
                when 39 =>                    -- the line's step A (pixel units) = A +- FM depth |2B - 1|
                    v_s := signed(resize(c_stepva, 25));
                    if e_xn = '1' then v_s := v_s - signed(resize(f_acc(23 downto 3), 25));
                    else v_s := v_s + signed(resize(f_acc(23 downto 3), 25)); end if;
                    n_stepl <= v_s;
                    e_rq <= '1'; e_opa <= c_gi(15 downto 4); e_opb <= e_ya;
                when 58 =>                    -- injected level = G 0.6 (B - Animate cos)
                    v_ij := f_acc(23 downto 8);
                    if e_yn = '1' then n_inj <= (not v_ij) + 1; else n_inj <= v_ij; end if;
                when others => null;
            end case;

            -- hblank: Formes B through the sine ROM (5 clocks after the pixel path's last read), then load the
            -- current-line registers
            if (s_gav_sr(0) = '1' and g_avid = '0') or (s_zone = '1' and s_zc = 3260) then
                y_cnt <= to_unsigned(1, 5);
            elsif y_cnt /= 0 then
                y_cnt <= y_cnt + 1;
            end if;
            if y_cnt = 9 then y_sel <= '1'; else y_sel <= '0'; end if;
            if y_cnt = 13 then
                if n_bl(15) = '1' then n_bfs <= not r_s_f; else n_bfs <= r_s_f; end if;
            end if;
            if y_cnt = 20 then
                l_stepl <= n_stepl; l_m <= n_m; l_inj <= n_inj;
                if s_s8 = '1' then l_bf <= f_trap16(n_bl); else l_bf <= n_bfs; end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: Ondes A's phase, hard reset to 0 at the line start (2^26 per
    -- cycle), stepped by the camera stream's pixel step
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ph <= (others => '0');
                b_spx <= s_spx;
            elsif g_avid = '1' then
                b_ph <= b_ph + b_spx;
                b_spx <= s_spx;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_b   : signed(10 downto 0);
        variable v_a   : signed(11 downto 0);
        variable v_am  : signed(17 downto 0);
        variable v_amb : signed(17 downto 0);
        variable v_x   : unsigned(15 downto 0);
        variable v_af  : unsigned(15 downto 0);
        variable v_ab  : signed(16 downto 0);
        variable v_ba  : signed(12 downto 0);
        variable v_bb  : signed(9 downto 0);
        variable v_xb  : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: Ondes A linear out (triangle, or the saw at twice the rate under NORM SQR); its bandwidth (a
            --    fast oscillation fades to its mean: full below 32 / 2^10 cycles per pixel, gone at 64 / 2^10)
            if s_s7 = '1' then p2_a16 <= b_ph(24 downto 9); else p2_a16 <= f_tri26(b_ph); end if;
            v_b := to_signed(64, 11) - signed(resize(b_spx(25 downto 16), 11));
            v_b := shift_right(v_b, 2);
            if v_b < 0 then p2_bw <= (others => '0');
            elsif v_b > 8 then p2_bw <= to_unsigned(8, 4);
            else p2_bw <= unsigned(v_b(3 downto 0)); end if;
            -- 3: AM (four-quadrant, about mid-grey): (A - 1/2) x m
            v_a := signed((not p2_a16(15)) & p2_a16(14 downto 4));
            v_ba := resize(v_a, 13); v_bb := resize(l_m, 10);
            v_xb := f_booth(v_ba, v_bb);
            p3_amp <= v_xb(20 downto 0);
            p3_bw <= p2_bw;
            -- 4: x bandwidth / 8 (shift-adds)
            v_am := p3_amp(20 downto 3);
            if p3_bw(3) = '1' then p4_u1 <= v_am;
            else
                p4_u1 <= (others => '0');
                if p3_bw(2) = '1' and p3_bw(1) = '1' then p4_u1 <= shift_right(v_am, 1) + shift_right(v_am, 2);
                elsif p3_bw(2) = '1' then p4_u1 <= shift_right(v_am, 1);
                elsif p3_bw(1) = '1' then p4_u1 <= shift_right(v_am, 2); end if;
            end if;
            if p3_bw(0) = '1' and p3_bw(3) = '0' then p4_u2 <= shift_right(v_am, 3); else p4_u2 <= (others => '0'); end if;
            -- 5: A' = 1/2 + AM (|AM| <= 1/2: only the top end saturates)
            v_amb := p4_u1 + p4_u2;
            if v_amb(17) = '0' and v_amb(15) = '1' then p5_ap <= (others => '1');
            else p5_ap <= (not v_amb(15)) & unsigned(v_amb(14 downto 0)); end if;
            -- 6: Formes: sine ROM address (the port is shared with the line engine's B read in hblank) ; trapezoid
            if y_sel = '1' then r_s_a <= f_sidx(n_bl);
            else r_s_a <= f_sidx(p5_ap); end if;
            p6_up <= p5_ap(15); p6_tr <= f_trap16(p5_ap);
            -- 7: ROM (EBR) ; 8: fabric copy
            p7_up <= p6_up; p7_tr <= p6_tr;
            p8_up <= p7_up; p8_tr <= p7_tr;
            -- 9: Formes A, and Combines' sum / difference with Formes B
            if s_s8 = '1' then v_af := p8_tr;
            elsif p8_up = '1' then v_af := not r_s_f;
            else v_af := r_s_f; end if;
            p9_sum <= resize(v_af, 17) + resize(l_bf, 17);
            p9_dif <= signed(resize(v_af, 17)) - signed(resize(l_bf, 17));
            -- 10: Combines: AVG or XOR (|A - B|)
            if s_s9 = '1' then
                v_ab := abs(p9_dif);
                p10_cc <= unsigned(v_ab(15 downto 0));
            else
                p10_cc <= p9_sum(16 downto 1);
            end if;
            -- 11: Cadrans (Fold drive): c x G
            v_ba := signed(resize(p10_cc(15 downto 4), 13)); v_bb := signed(resize(c_gm(15 downto 7), 10));
            v_xb := f_booth(v_ba, v_bb);
            p11_x0p <= unsigned(v_xb(20 downto 0)) & '0';
            pa0(11) <= p10_cc(15 downto 8);
            for i in 12 to 16 loop pa0(i) <= pa0(i - 1); end loop;
            -- 12: >> (5 + gs), Solaire rectifier 1
            case to_integer(c_gs) is
                when 1      => v_x := p11_x0p(21 downto 6);
                when 2      => v_x := resize(p11_x0p(21 downto 7), 16);
                when 3      => v_x := resize(p11_x0p(21 downto 8), 16);
                when others => v_x := resize(p11_x0p(21 downto 9), 16);
            end case;
            p12_r1 <= f_fold(v_x);
            -- 13 .. 15: Solaire: mixer (ADD / SUB) + rectifier, three times; the injected level (Ondes B + Animate,
            --    x G, + the camera) comes from the camera stream
            if s_s10 = '1' then v_x := p12_r1 - s_injp; else v_x := p12_r1 + s_injp; end if;
            p13_r2 <= f_fold(v_x);
            if s_s10 = '1' then v_x := p13_r2 - s_injp; else v_x := p13_r2 + s_injp; end if;
            p14_r3 <= f_fold(v_x);
            if s_s10 = '1' then v_x := p14_r3 - s_injp; else v_x := p14_r3 + s_injp; end if;
            p15_r4 <= f_fold(v_x);
            -- fades of over-fine rings toward their mean (per field): rectifier 2's tap (14), rectifier 4's (16)
            p14_a1 <= f_fade(p13_r2, c_fr2);
            p16_a2 <= f_fade(p15_r4, c_fr4);
            p15_a1 <= p14_a1; p16_a1 <= p15_a1; p17_a1 <= p16_a1;
            p17_a2 <= p16_a2; p18_a2 <= p17_a2;
            -- 17: table addresses: the Combines tap now, rectifier 2's one pixel later, rectifier 4's two
            --     (path delay; at the line start the oldest tap repeats the first pixel)
            t0_a <= pa0(16);
            if s_gav_sr(14) = '1' and s_gav_sr(15) = '0' then
                t1_a <= p16_a1; t2_a <= p16_a2;
            elsif s_gav_sr(15) = '1' and s_gav_sr(16) = '0' then
                t1_a <= p17_a1; t2_a <= p17_a2;
            else
                t1_a <= p17_a1; t2_a <= p18_a2;
            end if;
            -- 18: tables (EBR) ; 19: fabric copies
            for i in 0 to 8 loop p19_t(i) <= signed(t_q(i)(10 downto 0)); end loop;
            -- 20, 21: Matrice sums
            p20_ya <= resize(p19_t(0), 13) + resize(p19_t(3), 13); p20_yb <= resize(p19_t(6), 13) + c_oy;
            p20_ua <= resize(p19_t(1), 13) + resize(p19_t(4), 13); p20_ub <= resize(p19_t(7), 13) + c_ou;
            p20_va <= resize(p19_t(2), 13) + resize(p19_t(5), 13); p20_vb <= resize(p19_t(8), 13) + c_ov;
            p21_y <= p20_ya + p20_yb; p21_u <= p20_ua + p20_ub; p21_v <= p20_va + p20_vb;
            -- 22: the encoder's clip; black off the generated avid
            if s_gav_sr(19) = '0' then
                s_y26 <= C_KY; s_u26 <= C_KU; s_v26 <= C_KV;
            else
                if p21_y < 64 then s_y26 <= to_unsigned(64, 10);
                elsif p21_y > 940 then s_y26 <= to_unsigned(940, 10);
                else s_y26 <= unsigned(p21_y(9 downto 0)); end if;
                if p21_u < 64 then s_u26 <= to_unsigned(64, 10);
                elsif p21_u > 960 then s_u26 <= to_unsigned(960, 10);
                else s_u26 <= unsigned(p21_u(9 downto 0)); end if;
                if p21_v < 64 then s_v26 <= to_unsigned(64, 10);
                elsif p21_v > 960 then s_v26 <= to_unsigned(960, 10);
                else s_v26 <= unsigned(p21_v(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(19);
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
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y26) & std_logic_vector(s_u26) & std_logic_vector(s_v26);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y28) & std_logic_vector(s_u28) & std_logic_vector(s_v28);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    p_vs : process(clk)
        variable v_s : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(23) = '1' then v_k29 <= v_k29 + 1; else v_k29 <= (others => '0'); end if;   -- column of the stage-25 pixel
            if s_gav_sr(25) = '1' then v_k31 <= v_k31 + 1; else v_k31 <= (others => '0'); end if;   -- column of the stage-27 pixel
            v_qy <= vy(to_integer(v_k29(10 downto 1)));
            v_qc <= vc(to_integer(v_k29(10 downto 2)));
            v_fy <= v_qy; v_fc <= v_qc;
            v_py <= s_y31; v_pu <= s_u31; v_pv <= s_v31;
            v_wye <= s_gav_sr(25) and v_k31(0);
            v_wya <= v_k31(10 downto 1);
            v_s := resize(s_y31, 11) + resize(v_py, 11); v_wyd <= std_logic_vector(v_s(10 downto 3));
            if s_gav_sr(25) = '1' and v_k31(1 downto 0) = "01" then v_wce <= '1'; else v_wce <= '0'; end if;
            v_wca <= v_k31(10 downto 2);
            v_s := resize(s_u31, 11) + resize(v_pu, 11); v_wcd(15 downto 8) <= std_logic_vector(v_s(10 downto 3));
            v_s := resize(s_v31, 11) + resize(v_pv, 11); v_wcd(7 downto 0) <= std_logic_vector(v_s(10 downto 3));
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
            -- box A: stage 27 sums, stage 28 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y26 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u26 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v26 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(21) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y28 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u28 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v28 <= v_sc(13 downto 4);
            else
                s_y28 <= C_KY; s_u28 <= C_KU; s_v28 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(21);
            -- box B: stage 29 sums, stage 30 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y28 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u28 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v28 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y30 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u30 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v30 <= v_sc(13 downto 4);
            -- 31: luma 13/16
            if s_y30 > 64 then v_dl := s_y30 - 64; else v_dl := (others => '0'); end if;
            s_y31 <= s_y30 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u31 <= s_u30; s_v31 <= s_v30;
            -- 32: vertical mix with the line above
            v_mx := resize(s_y31, 11) + resize(unsigned(v_fy) & "10", 11);             s_y32 <= v_mx(10 downto 1);
            v_mx := resize(s_u31, 11) + resize(unsigned(v_fc(15 downto 8)) & "10", 11); s_u32 <= v_mx(10 downto 1);
            v_mx := resize(s_v31, 11) + resize(unsigned(v_fc(7 downto 0)) & "10", 11);  s_v32 <= v_mx(10 downto 1);
            -- 33: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y33 <= s_y32; s_u33 <= s_u32; s_v33 <= s_v32;
            else
                s_y33 <= to_unsigned(64, 10); s_u33 <= to_unsigned(512, 10); s_v33 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y33);
    data_out.u       <= std_logic_vector(s_u33);
    data_out.v       <= std_logic_vector(s_v33);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ondine;
