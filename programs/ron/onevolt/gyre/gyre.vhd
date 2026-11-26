-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Gyre v0.1.0 -- Onevolt: a lattice wound into a vortex (Navigator's Rotation CV driven at video rate).
--
-- The rack and the patch:
--   Visual Cortex H, V ramps -> gain (K2 Width, K3 Height) -> Navigator H in, V in
--   Navigator Rotation = K1 + a 13 s LFO sway + the shape gradient (DSG3: Ellipse = two parabolic ramps, or the
--     V ramp; S7) through the Rotation CV attenuverter (P12 Twist, a 19 s breathing) + the camera luma (K6)
--   Navigator H out, V out -> Arch Mirror x2 (S8: In = ramp -> triangle, Out = unmirrored saws)
--   -> Arch logic (S9: Average / Absolute)
--   -> Staircase (K4 Frequency, a 7 s LFO saw on Phase; /1 /2 /4 outputs)  or (S10) Doorway in Outline mode
--      (K4 Gain = outline width, the LFO's triangle on Threshold)
--   -> Color Chords (the three outputs key three coloured layers over black, priority /4 .. /1; K5 Colour
--      crossfades through nine arrangements) -> the Visual Cortex compositor: Fade to black keyed by the camera
--      (AC-coupled key generator: below-average luma closes the key) -> encoder
--   Where the wound-up pattern gets finer than ~4 px it fades toward a dark mean (the encoder's bandwidth).
--
--   K1  Rotation  Navigator Rotation, 0 .. 180 degrees (the mirrored lattice repeats every half turn: the ends meet)
--   K2  Width     H ramp gain: about 1 .. 12 cells across
--   K3  Height    V ramp gain: about 1 .. 12 cells per screen width down the V axis
--   K4  Fold      Staircase Frequency 1 .. 12 bands per half cell (Doorway Gain 4 .. 64)
--   K5  Colour    Color Chords arrangement, nine, crossfaded
--   K6  Camera    camera level into the Rotation CV (up to ~0.1 turn) and the key (0 = the black-input picture)
--   S7  Twist     Ellipse / Ramp
--   S8  Mirror    In / Out
--   S9  Logic     Average / Absolute
--   S10 Folder    Staircase / Doorway (outline)
--   S11 Source    Video / Osc
--   P12 TWIST     gradient depth: 0 = a rigid lattice .. 0.5 turn corner-to-centre (50 %) .. 1.5 turns (100 %)
--
-- Engine: vblank-zone sequencer (one serial multiplier, 21 slots, shared knob glide) and a palette writer (Color
-- Chords mix of 256 Staircase phases or 256 outline levels, every field); per-line second-order DDAs (gradient,
-- V ramp); per-pixel DDAs (gradient, H ramp); an octant sine / cosine ROM and the four-multiplier rotator;
-- 36-clock latency.  Shell (from heirloom/lozenge, patchbay/separatrix): generated avid, one latency, the
-- slewed camera path, output conditioning with the per-line resync, vertical mix, blanking gate, virtual 1080.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.gyre_pkg.all;
use work.all;

architecture gyre of program_top is
    constant C_LAT : integer := 52;   -- = processing_delay_clks (the output register is stage C_LAT + 1, as the shell)

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
    type t_c3 is array (0 to 2) of unsigned(9 downto 0);
    type t_s3 is array (0 to 2) of unsigned(17 downto 0);
    -- one 10-bit channel of a {Y, U, V, 00} word / of a channel triple
    function f_w10(w : std_logic_vector(31 downto 0); ch : unsigned(1 downto 0)) return unsigned is
    begin
        case ch is
            when "00"   => return unsigned(w(31 downto 22));
            when "01"   => return unsigned(w(21 downto 12));
            when others => return unsigned(w(11 downto 2));
        end case;
    end function;
    function f_a3(a : t_c3; ch : unsigned(1 downto 0)) return unsigned is
    begin
        case ch is
            when "00"   => return a(0);
            when "01"   => return a(1);
            when others => return a(2);
        end case;
    end function;
    -- clamp a signed value to 0 .. 256 (9 bits)
    function f_c256(v : signed) return unsigned is
    begin
        if v(v'high) = '1' then return to_unsigned(0, 9);
        elsif v > 256 then return to_unsigned(256, 9);
        else return unsigned(v(8 downto 0)); end if;
    end function;

    -- camera cell store: one 12-bit vertical-slew state per 8-pixel cell (read a cell ahead of the write)
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
    signal s_h1, s_h2 : unsigned(11 downto 0) := (others => '0');
    signal s_la   : unsigned(12 downto 0) := (others => '0');
    signal s_lumk  : unsigned(17 downto 0) := (others => '0');
    signal s_thc   : unsigned(15 downto 0) := (others => '0');
    signal s_kc    : unsigned(4 downto 0) := (others => '0');
    signal s_cfirst, s_cf1 : std_logic := '0';
    signal s_csum : unsigned(25 downto 0) := (others => '0');

    -- palettes (ROM: Staircase 0 .. 255, Doorway 256 .. 511)
    signal r_pal_a : unsigned(9 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal v_k29, v_k31 : unsigned(10 downto 0) := (others => '0');
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
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(12 downto 0) := (others => '0');
    signal zr_zc  : unsigned(9 downto 0) := (others => '0');
    signal zr_zone, zr_hi : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches (latched per field)
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(20 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(650 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(520 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(480 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(0, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal c_ell, c_saw, c_abs, c_door, c_osc : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-mode constants (latched per field)
    signal z_sd   : std_logic := '0';
    signal z_vsh  : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal z_mra  : unsigned(7 downto 0) := (others => '0');
    signal z_mrq  : std_logic_vector(31 downto 0) := (others => '0');

    -- LFO phases
    signal z_swp, z_php, z_dtp : unsigned(15 downto 0) := (others => '0');

    -- per-field intermediates
    signal z_sq2, z_sq3, z_sq4, z_sqp : unsigned(9 downto 0) := (others => '0');
    signal z_t1   : unsigned(13 downto 0) := (others => '0');
    signal z_nw, z_nh, z_f, z_g : unsigned(11 downto 0) := (others => '0');
    signal z_t    : unsigned(16 downto 0) := (others => '0');
    signal z_aq   : unsigned(12 downto 0) := (others => '0');
    signal z_qs   : unsigned(15 downto 0) := (others => '0');
    signal z_sw   : unsigned(10 downto 0) := (others => '0');
    signal z_tri  : unsigned(14 downto 0) := (others => '0');
    signal z_thr  : unsigned(9 downto 0) := (others => '0');
    signal z_mean : unsigned(7 downto 0) := (others => '0');
    signal z_first : std_logic := '0';

    -- per-field results
    signal c_th0  : unsigned(15 downto 0) := (others => '0');
    signal c_qx0  : unsigned(33 downto 0) := (others => '0');
    signal c_dx0  : signed(27 downto 0) := (others => '0');
    signal c_ddx  : unsigned(17 downto 0) := (others => '0');
    signal c_dd   : unsigned(33 downto 0) := (others => '0');
    signal c_dxs  : unsigned(16 downto 0) := (others => '0');
    signal c_x0   : signed(26 downto 0) := (others => '0');
    signal c_dys  : unsigned(17 downto 0) := (others => '0');
    signal c_off  : signed(15 downto 0) := (others => '0');
    signal c_fg   : unsigned(11 downto 0) := to_unsigned(64, 12);
    signal c_phi  : unsigned(7 downto 0) := (others => '0');
    signal c_mk   : unsigned(17 downto 0) := (others => '0');

    -- field serial multiply: acc = a x b
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(25 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(16 downto 0) := (others => '0');
    signal z_fb  : unsigned(25 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

    -- the fade target... (none) ; K5 transform per field
    signal c_k5s  : unsigned(2 downto 0) := (others => '0');
    signal c_k5f  : unsigned(4 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal l_py   : unsigned(47 downto 0) := (others => '0');
    signal l_dy   : signed(41 downto 0) := (others => '0');
    signal l_y    : signed(26 downto 0) := (others => '0');
    signal l_st   : std_logic_vector(1 downto 0) := "00";
    signal l_c1, l_c2 : std_logic := '0';
    signal l_th   : unsigned(15 downto 0) := (others => '0');
    signal l_y12  : signed(14 downto 0) := (others => '0');
    signal l_first : std_logic := '0';
    signal l_s1, l_s2, l_s3 : std_logic := '0';

    -- beam
    signal b_qx  : unsigned(33 downto 0) := (others => '0');
    signal b_dx  : signed(27 downto 0) := (others => '0');
    signal b_x   : signed(26 downto 0) := (others => '0');

    -- pixel path
    signal p2_t, p3_th : unsigned(15 downto 0) := (others => '0');
    signal p2_x, p3_x : signed(14 downto 0) := (others => '0');
    signal q16, q16d : unsigned(3 downto 0) := (others => '0');
    signal cu_x, cu_y, cu_z : signed(15 downto 0) := (others => '0');
    signal ip_px, ip_py, ip_dx, ip_dy : signed(15 downto 0) := (others => '0');
    signal p33_ux, p33_vy : signed(19 downto 0) := (others => '0');
    signal p34_a, p34_b : unsigned(8 downto 0) := (others => '0');
    signal p35_s : unsigned(8 downto 0) := (others => '0');
    signal p36_ph : unsigned(16 downto 0) := (others => '0');
    signal p36_pl : unsigned(15 downto 0) := (others => '0');
    signal p37_p  : unsigned(20 downto 0) := (others => '0');
    signal p38_i, p39_i : unsigned(7 downto 0) := (others => '0');
    signal p38_k  : signed(15 downto 0) := (others => '0');
    signal p42_c  : std_logic_vector(31 downto 0) := (others => '0');
    type t_y3 is array (0 to 2) of unsigned(9 downto 0);
    type t_d3 is array (0 to 2) of signed(10 downto 0);
    signal p43_c, p44_c, p45_c : t_y3 := (others => (others => '0'));
    signal p43_d, p44_d : t_d3 := (others => (others => '0'));
    signal p44_t1, p44_t2 : t_d3 := (others => (others => '0'));
    signal s_y46, s_u46, s_v46 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y48, s_u48, s_v48 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y50, s_u50, s_v50 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y51, s_u51, s_v51 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y52, s_u52, s_v52 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y53 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u53, s_v53 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := unsigned(C_GY_K(31 downto 22));
    constant C_KU : unsigned(9 downto 0) := unsigned(C_GY_K(21 downto 12));
    constant C_KV : unsigned(9 downto 0) := unsigned(C_GY_K(11 downto 2));

    -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel: taps 0, 1 in fabric, 43 .. 52 from a ring delay
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');
    type t_ar is array (0 to 255) of std_logic_vector(1 downto 0);
    signal ar : t_ar := (others => "00");
    attribute no_rw_check of ar : signal is true;
    signal a_rp : unsigned(7 downto 0) := (others => '0');
    signal a_q  : std_logic_vector(1 downto 0) := "00";
    -- hsync / vsync / field delayed C_LAT clocks through a block-RAM ring (write at y_wp, read y_wp - (C_LAT - 2))
    type t_yr is array (0 to 255) of std_logic_vector(3 downto 0);
    signal yr : t_yr := (others => "1111");
    attribute no_rw_check of yr : signal is true;
    signal y_wp, y_rp : unsigned(7 downto 0) := (others => '0');
    signal y_q  : std_logic_vector(3 downto 0) := "1111";   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    -- g_avid through a block-RAM ring: written as s_gav_sr(0) would be, read back as s_gav_sr(42)
    p_ar : process(clk)
    begin
        if rising_edge(clk) then
            a_q <= ar(to_integer(a_rp));
            ar(to_integer(y_wp)) <= '0' & g_avid;
            a_rp <= y_wp - 41;
        end if;
    end process p_ar;

    p_yr : process(clk)
    begin
        if rising_edge(clk) then
            y_q <= yr(to_integer(y_rp));
            yr(to_integer(y_wp)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            y_wp <= y_wp + 1;
            y_rp <= y_wp - (C_LAT - 2);
        end if;
    end process p_yr;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            r_pal_q <= C_GY_PAL(to_integer(r_pal_a));
        end if;
    end process p_pal;



    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_t  : unsigned(13 downto 0);
        variable v_g  : unsigned(12 downto 0);
        variable v_m  : signed(12 downto 0);
        variable v_m2 : signed(12 downto 0);
        variable v_tid : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xsr <= f_vstep_x(s_tid);
            v_tid := to_integer(unsigned(s_tid));

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
            elsif s_zc /= 8191 then
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
                c_ell  <= not registers_in(6)(0);
                c_saw  <= registers_in(6)(1);
                c_abs  <= registers_in(6)(2);
                c_door <= registers_in(6)(3);
                c_osc  <= registers_in(6)(4);
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                z_swp <= z_swp + 84;
                z_php <= z_php + 156;
                z_dtp <= z_dtp + 78;
                if s_xsr = 4 then z_sd <= '1'; else z_sd <= '0'; end if;
                z_first <= '1';
            elsif g_start = '1' then
                z_first <= '0';
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

            -- shared glide (1/8 of the gap per field, +-2 deadband; +-4 on the camera channel): four clocks per
            -- channel, run from the zone start by its own phase / channel counters
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

            -- serial multiply slots: slot k loads at zc = 48 + 32 k, 26 iterations, result stable from load + 27,
            -- captured at load + 28.  The slot number is decoded at load - 2, the operands pre-muxed at load - 1;
            -- a per-mode constant is read from the mode ROM at load - 3 (address) for load - 1 (data).
            z_mrq <= C_GY_MROM(to_integer(z_mra));
            if s_zone = '1' and s_zc = 20 then
                z_mra <= unsigned(s_tid) & "1001";                               -- VSH
            elsif s_zone = '1' and s_zc >= 45 and s_zc <= 621 and s_zc(4 downto 0) = "01101" then
                case to_integer(shift_right(s_zc - 45, 5)) is
                    when 6      => z_mra <= unsigned(s_tid) & "0000";            -- HX2
                    when 7      => z_mra <= unsigned(s_tid) & "0001";            -- DX0C
                    when 8      => z_mra <= unsigned(s_tid) & "0010";            -- DDC
                    when 10     => if c_ell = '1' then z_mra <= unsigned(s_tid) & "0011"; else z_mra <= unsigned(s_tid) & "0101"; end if;
                    when 11     => z_mra <= unsigned(s_tid) & "0100";            -- LDDC
                    when 12     => z_mra <= unsigned(s_tid) & "0110";            -- DXC
                    when 13     => z_mra <= unsigned(s_tid) & "0111";            -- DYC
                    when 17     => z_mra <= unsigned(s_tid) & "1010";            -- X0C
                    when 18     => z_mra <= unsigned(s_tid) & "1011";            -- Y0C
                    when others => z_mra <= unsigned(s_tid) & "1000";            -- R
                end case;
            end if;
            if s_zone = '1' and s_zc = 22 then z_vsh <= unsigned(z_mrq(3 downto 0)); end if;
            z_pre <= '0';
            if s_zone = '1' and s_zc >= 46 and s_zc <= 622 and s_zc(4 downto 0) = "01110" then
                z_sel <= resize(shift_right(s_zc - 46, 5), 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g2(12 downto 3), 17);
                    when 1  => z_fa <= resize(s_g3(12 downto 3), 17);
                    when 2  => z_fa <= resize(s_g4(12 downto 3), 17);
                    when 3  => z_fa <= resize(s_gp(12 downto 3), 17);
                    when 4  => z_fa <= resize(z_swp(14 downto 0), 17);
                               z_fb <= resize(to_unsigned(32768, 16) - resize(z_swp(14 downto 0), 16), 26);
                    when 5  => z_fa <= z_t;               z_fb <= to_unsigned(3540, 26);
                    when 9  => z_fa <= resize(z_aq, 17);  z_fb <= to_unsigned(291600, 26);
                    when 10 => if c_ell = '1' then z_fa <= resize(z_aq, 17); else z_fa <= z_t; end if;
                               z_fb <= unsigned(z_mrq(25 downto 0));
                    when 12 | 17 => z_fa <= resize(z_nw, 17); z_fb <= unsigned(z_mrq(25 downto 0));
                    when 13 | 18 => z_fa <= resize(z_nh, 17); z_fb <= unsigned(z_mrq(25 downto 0));
                    when 14 => z_fa <= resize(z_g, 17);   z_fb <= resize(z_thr, 26);
                    when 15 => z_fa <= unsigned(z_mrq(16 downto 0)); z_fb <= s_csum;
                    when 16 => z_fa <= resize(z_mean, 17); z_fb <= resize(s_g6(12 downto 8) & "00000", 26);
                    when others => z_fa <= resize(z_aq, 17); z_fb <= unsigned(z_mrq(25 downto 0));   -- 6 7 8 11
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 36);
                if z_sel(4 downto 2) = "000" then f_b <= resize(z_fa, 26); else f_b <= z_fb; end if;   -- squares
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(25 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- Color Chords arrangement (after the glide): index and crossfade
                    when 32 =>
                        c_k5s <= '0' & s_g5(12 downto 11);
                        v_g := resize(s_g5(10 downto 3), 13);
                        if v_g < 96 then c_k5f <= (others => '0');
                        elsif v_g >= 160 then c_k5f <= to_unsigned(16, 5);
                        else c_k5f <= resize(shift_right(v_g - 96, 2), 5); end if;
                    -- Doorway threshold (the LFO's triangle): 154 + tri/64 + tri/256 + tri/512
                    when 34 =>
                        if z_dtp(15) = '1' then z_tri <= not z_dtp(14 downto 0); else z_tri <= z_dtp(14 downto 0); end if;
                    when 35 => z_thr <= to_unsigned(154, 10) + resize(z_tri(14 downto 6), 10);
                    when 36 => z_thr <= z_thr + resize(z_tri(14 downto 8), 10) + resize(z_tri(14 downto 9), 10);
                    when 37 => c_phi <= z_php(15 downto 8);
                    when others => null;
                end case;
            end if;
            -- captures (load + 29 = sub-step 12 of slot k + 2, one clock late from the registered zone counter copy)
            -- and the plain laws after them (sub-steps 13 .. 17), decoded as sub-step x slot (two 5-bit compares)
            zr_zone <= s_zone; zr_zc <= s_zc(9 downto 0); zr_hi <= '0';
            if s_zc(12 downto 10) = "000" then zr_hi <= '1'; end if;
            if zr_zone = '1' and zr_hi = '1' then
                case to_integer(zr_zc(4 downto 0)) is
                    when 12 =>
                        case to_integer(zr_zc(9 downto 5)) is
                            when 2 => z_sq2 <= f_acc(19 downto 10);
                            when 3 => z_sq3 <= f_acc(19 downto 10);
                            when 4 => z_sq4 <= f_acc(19 downto 10);
                            when 5 => z_sqp <= f_acc(19 downto 10);
                            when 6 => z_qs <= f_acc(28 downto 13);
                            when 7 => z_aq <= f_acc(28 downto 16);
                            when 8 => if c_ell = '1' then c_qx0 <= f_acc(33 downto 0); else c_qx0 <= (others => '0'); end if;
                            when 9 => if c_ell = '1' then c_dx0 <= -signed(resize(f_acc(26 downto 0), 28)); else c_dx0 <= (others => '0'); end if;
                            when 10 => if c_ell = '1' then c_ddx <= f_acc(17 downto 0); else c_ddx <= (others => '0'); end if;
                            when 13 => if c_ell = '1' then c_dd <= f_acc(33 downto 0); else c_dd <= (others => '0'); end if;
                            when 14 => c_dxs <= f_acc(28 downto 12);
                            when 15 => c_dys <= f_acc(29 downto 12);
                            when 16 => c_off <= signed(resize(f_acc(29 downto 8), 16)) - 128;
                            when 17 =>
                                if f_acc(35 downto 32) /= 0 then z_mean <= (others => '1'); else z_mean <= f_acc(31 downto 24); end if;
                            when 18 => c_mk <= f_acc(17 downto 0);
                            when 19 => c_x0 <= -signed(f_acc(26 downto 0));
                            when others => null;
                        end case;
                    when 13 =>
                        case to_integer(zr_zc(9 downto 5)) is
                            when 2 => z_t1 <= shift_left(resize(z_sq2, 14), 3) + shift_left(resize(z_sq2, 14), 1);
                            when 3 => z_t1 <= shift_left(resize(z_sq3, 14), 3) + shift_left(resize(z_sq3, 14), 1);
                            when 4 => z_t1 <= shift_left(resize(z_sq4, 14), 3) + shift_left(resize(z_sq4, 14), 1);
                            when 5 => z_t <= shift_left(resize(s_gp(12 downto 3), 17), 5) + shift_left(resize(z_sqp, 17), 6);
                            when 6 => z_sw <= resize(z_qs(15 downto 5), 11) + resize(z_qs(15 downto 7), 11) + resize(z_qs(15 downto 10), 11);
                            when others => null;
                        end case;
                    when 14 =>
                        case to_integer(zr_zc(9 downto 5)) is
                            when 2 => z_nw <= to_unsigned(256, 12) + resize(shift_right(z_t1 + resize(z_sq2, 14), 2), 12);
                            when 3 => z_nh <= to_unsigned(256, 12) + resize(shift_right(z_t1 + resize(z_sq3, 14), 2), 12);
                            when 4 => z_f <= to_unsigned(64, 12) + resize(shift_right(z_t1 + resize(z_sq4, 14), 4), 12);
                            when 6 =>
                                if z_swp(15) = '1' then
                                    c_th0 <= shift_left(resize(s_g1(12 downto 3), 16), 5) - resize(z_sw, 16);
                                else
                                    c_th0 <= shift_left(resize(s_g1(12 downto 3), 16), 5) + resize(z_sw, 16);
                                end if;
                            when others => null;
                        end case;
                    when 15 =>
                        case to_integer(zr_zc(9 downto 5)) is
                            when 4 => z_t1 <= shift_left(resize(z_sq4, 14), 4) - resize(z_sq4, 14);
                            when others => null;
                        end case;
                    when 16 =>
                        case to_integer(zr_zc(9 downto 5)) is
                            when 4 => z_g <= to_unsigned(256, 12) + resize(shift_right(z_t1, 2), 12);
                            when others => null;
                        end case;
                    when 17 =>
                        case to_integer(zr_zc(9 downto 5)) is
                            when 4 =>
                                if c_door = '1' then c_fg <= z_g; else c_fg <= z_f; end if;
                            when others => null;
                        end case;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid; s_gav_sr(1) <= s_gav_sr(0);
            s_gav_sr(43) <= a_q(0);
            for i in 44 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
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
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- ((Y >> 2) - 16), averaged over 8-pixel cells, each cell slewed down the
    -- screen (one pole per line, 1/16 .. 1/64 by mode; the first line of a
    -- field loads directly; the state lives in the cell store), then slewed
    -- across by two one-pole filters (1/16 per pixel), times K6.  It drives the
    -- Rotation CV (lumK / 32, Osc / 16) and the AC-coupled key (the field mean
    -- minus lumK, x ~7, clipped to 0 .. 16 sixteenths).  The sum of the cell
    -- states over the field gives the mean.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(12 downto 0);
        variable v_k : signed(18 downto 0);
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
            if c_osc = '1' then
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
            if s_zone = '1' then s_cfirst <= '1';
            elsif s_lav = '1' and data_in.avid = '0' then s_cfirst <= '0'; end if;
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
            s_cf1 <= s_cfirst;
            -- 3: state += (cell average x 16 - state) >> vsh  (the first line of the field loads)
            v_d := signed(resize(s_sum8 & '0', 13)) - signed(resize(s_old, 13));
            case to_integer(z_vsh) is
                when 4      => v_d := signed(resize(s_old, 13)) + shift_right(v_d, 4);
                when 5      => v_d := signed(resize(s_old, 13)) + shift_right(v_d, 5);
                when others => v_d := signed(resize(s_old, 13)) + shift_right(v_d, 6);
            end case;
            if s_cf1 = '1' then s_new <= s_sum8 & '0'; else s_new <= unsigned(v_d(11 downto 0)); end if;
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write; the field sum of the cell states (8-bit) for the mean
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);
            if s_zone = '1' and s_zc = 540 then
                s_csum <= (others => '0');
            elsif s_f2 = '1' then
                s_csum <= s_csum + resize(s_new(11 downto 4), 26);
            end if;

            -- pixel side: the cell stream (the line above's state; on the first line the latest cell sum), two
            -- slews across, x K6
            if s_cfirst = '1' then s_lv <= s_sum8 & '0'; else s_lv <= unsigned(s_lb_q(11 downto 0)); end if;
            v_e  := signed(resize(s_lv, 13)) - signed(resize(s_h1, 13));
            v_e  := signed(resize(s_h1, 13)) + shift_right(v_e, 4);
            s_h1 <= unsigned(v_e(11 downto 0));
            v_e  := signed(resize(s_h1, 13)) - signed(resize(s_h2, 13));
            v_e  := signed(resize(s_h2, 13)) + shift_right(v_e, 4);
            s_h2 <= unsigned(v_e(11 downto 0));
            s_la  <= s_h2(11 downto 4) * s_g6(12 downto 8);                  -- luma x K6 (5 bits) / 32
            s_lumk <= s_la & "00000";
            if c_osc = '1' then s_thc <= resize(s_lumk(17 downto 4), 16); else s_thc <= resize(s_lumk(17 downto 5), 16); end if;
            v_k := signed(resize(c_mk, 19)) - signed(resize(s_lumk, 19));
            if v_k(18) = '1' then s_kc <= (others => '0');
            elsif v_k(18 downto 11) >= 16 then s_kc <= to_unsigned(16, 5);
            else s_kc <= unsigned(v_k(15 downto 11)); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: the gradient (Q48 turns; Ellipse = Aq yq^2 by second differences, Ramp = linear) and the V
    -- ramp (Q23 cells); loaded in the zone, latched at each line start (with the base angle), then advanced
    -- (the 48-bit add in two halves)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_lo : unsigned(24 downto 0);
        variable v_dl : signed(24 downto 0);
        variable v_c1 : unsigned(0 downto 0);
        variable v_c2 : signed(1 downto 0);
    begin
        if rising_edge(clk) then
            -- the zone loads, as strobes registered a clock ahead (no counter compare in the enables)
            if s_zone = '1' and s_zc = 363 then l_s1 <= '1'; else l_s1 <= '0'; end if;
            if s_zone = '1' and s_zc = 395 then l_s2 <= '1'; else l_s2 <= '0'; end if;
            if s_zone = '1' and s_zc = 651 then l_s3 <= '1'; else l_s3 <= '0'; end if;
            if l_s1 = '1' then
                if c_ell = '1' then l_py <= f_acc(31 downto 0) & x"0000"; else l_py <= (others => '0'); end if;
                l_st <= "00";
            elsif l_s2 = '1' then
                if c_ell = '1' then l_dy <= -signed(resize(f_acc(32 downto 0) & x"00", 42));
                else l_dy <= signed(resize(f_acc(32 downto 0) & x"00", 42)); end if;
            elsif l_s3 = '1' then
                l_y <= -signed(f_acc(26 downto 0));
            elsif g_start = '1' then
                l_th <= l_py(47 downto 32) + c_th0;
                l_y12 <= l_y(26 downto 12);
                l_first <= z_first;
                l_st <= "01";
            elsif l_st = "01" then
                v_lo := resize(l_py(23 downto 0), 25) + resize(unsigned(l_dy(23 downto 0)), 25);
                l_py(23 downto 0) <= v_lo(23 downto 0); l_c1 <= v_lo(24);
                v_lo := resize(unsigned(l_dy(23 downto 0)), 25) + resize(c_dd(23 downto 0), 25);
                l_dy(23 downto 0) <= signed(v_lo(23 downto 0)); l_c2 <= v_lo(24);
                l_y <= l_y + signed(resize(c_dys, 27));
                l_st <= "10";
            elsif l_st = "10" then
                v_c1(0) := l_c1;
                v_c2 := (others => '0'); v_c2(0) := l_c2;
                l_py(47 downto 24) <= l_py(47 downto 24) + unsigned(resize(l_dy(41 downto 24), 24)) + v_c1;
                v_dl := resize(l_dy(41 downto 24), 25) + signed(resize(c_dd(33 downto 24), 25)) + v_c2;
                l_dy(41 downto 24) <= v_dl(17 downto 0);
                l_st <= "00";
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the gradient's x part (Q34 turns, Aq hx^2 by second differences) and the H ramp (Q23 cells)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_qx <= c_qx0; b_dx <= c_dx0; b_x <= c_x0;
            elsif g_avid = '1' then
                b_qx <= b_qx + unsigned(resize(b_dx, 34));
                b_dx <= b_dx + signed(resize(c_ddx, 28));
                b_x  <= b_x + signed(resize(c_dxs, 27));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        type t_atn is array (0 to 15) of integer;
        constant C_ATN : t_atn := (8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 0, 0, 0, 0);
        variable v_t   : unsigned(15 downto 0);
        variable v_x, v_y, v_xs, v_ys, v_mk, v_mn, v_at : signed(15 downto 0);
        variable v_z   : signed(15 downto 0);
        variable v_ci, v_cn : signed(1 downto 0);
        variable v_it  : integer range 0 to 15;
        variable v_d   : signed(16 downto 0);
        variable v_a   : unsigned(9 downto 0);
        variable v_f   : signed(10 downto 0);
        variable v_af  : signed(10 downto 0);
        variable v_ab  : signed(9 downto 0);
        variable v_aa  : signed(9 downto 0);
        variable v_s   : unsigned(9 downto 0);
        variable v_k, v_kt : signed(15 downto 0);
        variable v_c   : signed(11 downto 0);
        variable v_u, v_v : unsigned(9 downto 0);
        variable v_l   : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: the gradient's angle (x part + the line's part + base); X / 2^12 (Q11 cells)
            p2_t <= b_qx(33 downto 18) + l_th;
            p2_x <= b_x(26 downto 12);
            -- 3: + the camera
            p3_th <= p2_t + s_thc; p3_x <= p2_x;
            -- The rotator runs at 1/16 of the pixel rate: q16 = the column of the stage-3 pixel mod 16 (it runs on
            -- through the blank; the beam holds its line-end value, so the sample at column W is exact).  One
            -- CORDIC unit: q16 = 0 loads the sample (quadrant pre-rotation: (X, Y) turned by -q quarter turns,
            -- z = the remaining -angle), q16 = 1 .. 12 run iterations 0 .. 11 (shift-and-add, the add / subtract
            -- selected on the carry-in), q16 = 13 hands the result to the interpolator.
            if s_gav_sr(0) = '1' and s_gav_sr(1) = '0' then q16 <= (others => '0'); else q16 <= q16 + 1; end if;
            q16d <= q16;                                     -- = q16 - 1 from q16 = 1 on (the iteration number)
            v_it := to_integer(q16d);
            if q16 = 0 then
                v_t := p3_th + 8192;
                case v_t(15 downto 14) is
                    when "00"   => cu_x <= resize(p3_x, 16);  cu_y <= resize(l_y12, 16);
                    when "01"   => cu_x <= resize(l_y12, 16); cu_y <= -resize(p3_x, 16);
                    when "10"   => cu_x <= -resize(p3_x, 16); cu_y <= -resize(l_y12, 16);
                    when others => cu_x <= -resize(l_y12, 16); cu_y <= resize(p3_x, 16);
                end case;
                cu_z <= to_signed(8192, 16) - signed(resize(v_t(13 downto 0), 16));
            elsif q16 <= 12 then
                v_x := cu_x; v_y := cu_y; v_z := cu_z;
                v_xs := shift_right(v_x, v_it); v_ys := shift_right(v_y, v_it);
                v_at := to_signed(C_ATN(v_it), 16);
                v_ci := (others => '0'); v_cn := (others => '0');
                v_mk := (others => not v_z(15)); v_ci(0) := not v_z(15);     -- subtract y, atan when z >= 0
                v_mn := (others => v_z(15));     v_cn(0) := v_z(15);         -- subtract x when z < 0
                cu_x <= v_x + (v_ys xor v_mk) + v_ci;
                cu_y <= v_y + (v_xs xor v_mn) + v_cn;
                cu_z <= v_z + (v_at xor v_mk) + v_ci;
            end if;
            -- linear interpolation between the samples: at q16 = 13 the new sample's difference to the previous one
            -- is taken and the accumulator reloads with the previous sample; the sixteen pixels between them are
            -- 16 x base + k x d, k = 0 .. 15, so the stage-33 pixel gets base + (k d) / 16
            if q16 = 13 then
                v_d := resize(cu_x, 17) - resize(ip_px, 17); ip_dx <= v_d(15 downto 0);
                v_d := resize(cu_y, 17) - resize(ip_py, 17); ip_dy <= v_d(15 downto 0);
                ip_px <= cu_x; ip_py <= cu_y;
                p33_ux <= shift_left(resize(ip_px, 20), 4); p33_vy <= shift_left(resize(ip_py, 20), 4);
            else
                p33_ux <= p33_ux + resize(ip_dx, 20); p33_vy <= p33_vy + resize(ip_dy, 20);
            end if;
            -- 34: Arch Mirror (|frac - 1/2|) or the saw, on u, v (Q11 cells; the accumulator holds Q15)
            v_a := unsigned(p33_ux(14 downto 5));
            if c_saw = '1' then
                p34_a <= v_a(9 downto 1);
            else
                v_f := signed(resize(v_a, 11)) - 512;
                v_af := abs(v_f);
                if v_f = -512 then p34_a <= to_unsigned(511, 9); else p34_a <= unsigned(v_af(8 downto 0)); end if;
            end if;
            v_a := unsigned(p33_vy(14 downto 5));
            if c_saw = '1' then
                p34_b <= v_a(9 downto 1);
            else
                v_f := signed(resize(v_a, 11)) - 512;
                v_af := abs(v_f);
                if v_f = -512 then p34_b <= to_unsigned(511, 9); else p34_b <= unsigned(v_af(8 downto 0)); end if;
            end if;
            -- 35: Arch logic: Average or Absolute
            if c_abs = '1' then
                v_ab := signed(resize(p34_a, 10)) - signed(resize(p34_b, 10));
                v_aa := abs(v_ab);
                p35_s <= unsigned(v_aa(8 downto 0));
            else
                v_s := resize(p34_a, 10) + resize(p34_b, 10);
                p35_s <= v_s(9 downto 1);
            end if;
            -- 36: Staircase / Doorway gain (partial products) ; 37: the product
            p36_ph <= c_fg * p35_s(8 downto 4);
            p36_pl <= c_fg * p35_s(3 downto 0);
            p37_p <= shift_left(resize(p36_ph, 21), 4) + resize(p36_pl, 21);
            -- 38: Staircase phase index ; Doorway key ; 39: palette index (Doorway: the outline 1 - |1 - key|)
            p38_i <= p37_p(16 downto 9) + c_phi;
            p38_k <= signed(resize(p37_p(20 downto 7), 16)) - c_off;
            if c_door = '1' then
                v_kt := p38_k - to_signed(256, 16);
                v_k := to_signed(256, 16) - abs(v_kt);
                if v_k(15) = '1' then p39_i <= (others => '0');
                elsif v_k > 255 then p39_i <= (others => '1');
                else p39_i <= unsigned(v_k(7 downto 0)); end if;
            else
                p39_i <= p38_i;
            end if;
            -- 40: palette address = mode, key level, phase / level (7 bits).  The Visual Cortex key is baked into
            --     the palette as four levels (0, 1/3, 2/3, black); the level is dithered along the line in a
            --     4-pixel cycle (the output boxes average it away exactly): L = (3 kc + 4 ph + 2) / 16
            v_l := resize(shift_left(resize(s_kc, 7), 1) + resize(s_kc, 7), 7) + resize((q16(1 downto 0) + 3) & "10", 7);
            r_pal_a <= c_door & v_l(5 downto 4) & p39_i(7 downto 1);
            -- 41: palette (EBR) ; 42: fabric copy
            p42_c <= r_pal_q;
            -- 43 .. 45: K5 Colour: the chroma turned by s quarter turns, t = R^s (U, V), R (u, v) = (1023 - v, u),
            --           then crossfaded toward R t by f / 16 in four shifted terms
            v_u := f_w10(p42_c, "01"); v_v := f_w10(p42_c, "10");
            case to_integer(c_k5s) is
                when 0      => p43_c(1) <= v_u;     p43_c(2) <= v_v;
                when 1      => p43_c(1) <= not v_v; p43_c(2) <= v_u;
                when 2      => p43_c(1) <= not v_u; p43_c(2) <= not v_v;
                when others => p43_c(1) <= v_v;     p43_c(2) <= not v_u;
            end case;
            case to_integer(c_k5s) is
                when 0      => p43_d(1) <= signed(resize(not v_v, 11)) - signed(resize(v_u, 11));
                               p43_d(2) <= signed(resize(v_u, 11)) - signed(resize(v_v, 11));
                when 1      => p43_d(1) <= signed(resize(not v_u, 11)) - signed(resize(not v_v, 11));
                               p43_d(2) <= signed(resize(not v_v, 11)) - signed(resize(v_u, 11));
                when 2      => p43_d(1) <= signed(resize(v_v, 11)) - signed(resize(not v_u, 11));
                               p43_d(2) <= signed(resize(not v_u, 11)) - signed(resize(not v_v, 11));
                when others => p43_d(1) <= signed(resize(v_u, 11)) - signed(resize(v_v, 11));
                               p43_d(2) <= signed(resize(v_v, 11)) - signed(resize(not v_u, 11));
            end case;
            p43_c(0) <= f_w10(p42_c, "00");
            for c in 1 to 2 loop
                v_c := (others => '0');
                if c_k5f(3) = '1' then v_c := v_c + resize(shift_right(p43_d(c), 1), 12); end if;
                if c_k5f(2) = '1' then v_c := v_c + resize(shift_right(p43_d(c), 2), 12); end if;
                p44_t1(c) <= v_c(10 downto 0);
                v_c := (others => '0');
                if c_k5f(1) = '1' then v_c := v_c + resize(shift_right(p43_d(c), 3), 12); end if;
                if c_k5f(0) = '1' then v_c := v_c + resize(shift_right(p43_d(c), 4), 12); end if;
                p44_t2(c) <= v_c(10 downto 0);
            end loop;
            p44_c <= p43_c; p44_d <= p43_d;
            p45_c(0) <= p44_c(0);
            for c in 1 to 2 loop
                if c_k5f(4) = '1' then
                    v_c := signed(resize(p44_c(c), 12)) + resize(p44_d(c), 12);                            -- t + d
                else
                    v_c := signed(resize(p44_c(c), 12)) + resize(p44_t1(c), 12) + resize(p44_t2(c), 12);
                end if;
                p45_c(c) <= unsigned(v_c(9 downto 0));
            end loop;
            -- 46: black off the generated avid
            if s_gav_sr(43) = '0' then
                s_y46 <= C_KY; s_u46 <= C_KU; s_v46 <= C_KV;
            else
                s_y46 <= p45_c(0); s_u46 <= p45_c(1); s_v46 <= p45_c(2);
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(43);
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
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y46) & std_logic_vector(s_u46) & std_logic_vector(s_v46);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y48) & std_logic_vector(s_u48) & std_logic_vector(s_v48);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    p_vs : process(clk)
        variable v_s : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(47) = '1' then v_k29 <= v_k29 + 1; else v_k29 <= (others => '0'); end if;   -- column of the stage-49 pixel
            if s_gav_sr(49) = '1' then v_k31 <= v_k31 + 1; else v_k31 <= (others => '0'); end if;   -- column of the stage-51 pixel
            v_qy <= vy(to_integer(v_k29(10 downto 1)));
            v_qc <= vc(to_integer(v_k29(10 downto 2)));
            v_fy <= v_qy; v_fc <= v_qc;
            v_py <= s_y51; v_pu <= s_u51; v_pv <= s_v51;
            v_wye <= s_gav_sr(49) and v_k31(0);
            v_wya <= v_k31(10 downto 1);
            v_s := resize(s_y51, 11) + resize(v_py, 11); v_wyd <= std_logic_vector(v_s(10 downto 3));
            if s_gav_sr(49) = '1' and v_k31(1 downto 0) = "01" then v_wce <= '1'; else v_wce <= '0'; end if;
            v_wca <= v_k31(10 downto 2);
            v_s := resize(s_u51, 11) + resize(v_pu, 11); v_wcd(15 downto 8) <= std_logic_vector(v_s(10 downto 3));
            v_s := resize(s_v51, 11) + resize(v_pv, 11); v_wcd(7 downto 0) <= std_logic_vector(v_s(10 downto 3));
            if v_wye = '1' then vy(to_integer(v_wya)) <= v_wyd; end if;
            if v_wce = '1' then vc(to_integer(v_wca)) <= v_wcd; end if;
        end if;
    end process p_vs;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the encoder's horizontal softness, and the HD
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
            -- box A: stage 47 sums, stage 48 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y46 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u46 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v46 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(45) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y48 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u48 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v48 <= v_sc(13 downto 4);
            else
                s_y48 <= C_KY; s_u48 <= C_KU; s_v48 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(45);
            -- box B: stage 49 sums, stage 50 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y48 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u48 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v48 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y50 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u50 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v50 <= v_sc(13 downto 4);
            -- 51: luma 13/16
            if s_y50 > 64 then v_dl := s_y50 - 64; else v_dl := (others => '0'); end if;
            s_y51 <= s_y50 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u51 <= s_u50; s_v51 <= s_v50;
            -- 52: vertical mix with the line above
            v_mx := resize(s_y51, 11) + resize(unsigned(v_fy) & "10", 11);             s_y52 <= v_mx(10 downto 1);
            v_mx := resize(s_u51, 11) + resize(unsigned(v_fc(15 downto 8)) & "10", 11); s_u52 <= v_mx(10 downto 1);
            v_mx := resize(s_v51, 11) + resize(unsigned(v_fc(7 downto 0)) & "10", 11);  s_v52 <= v_mx(10 downto 1);
            -- 53: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y53 <= s_y52; s_u53 <= s_u52; s_v53 <= s_v52;
            else
                s_y53 <= to_unsigned(64, 10); s_u53 <= to_unsigned(512, 10); s_v53 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y53);
    data_out.u       <= std_logic_vector(s_u53);
    data_out.v       <= std_logic_vector(s_v53);
    data_out.hsync_n <= y_q(0);
    data_out.vsync_n <= y_q(1);
    data_out.field_n <= y_q(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture gyre;
