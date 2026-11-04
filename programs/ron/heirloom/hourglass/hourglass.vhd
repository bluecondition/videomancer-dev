-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Hourglass v0.1.0 -- Heirloom from the video clips, built from the still WIN_20230618_20_23_10 t0.5
-- (clip frame 9 at 0.528 s; the whole 11 s clip is one patch scrolling with a 2.37 s period).
--
-- The still: a rainbow lens pair on olive-black.  A top lens cut by the frame top (black core, blue ring,
-- magenta/yellow stripe pairs, cyan/green tip) and a bottom lens (rainbow rings round a yellow-green core),
-- pinched apart by a gap at the waist, both wrapped in a soft yellow halo.
--
-- The patch, as fitted (hourglass.py has the constants; .heirloom_work/rb_hourglass/notes.md the measurements),
-- in virtual 1080-line coordinates:
--   W(y)  = S(y) + A(y) sin(theta0 + y / lambda)           a scrolling vertical wave on an hourglass profile
--   S(y)  = 107 + kW P(|y - 530| / 540)                    P = 366.7|u| + 80.9u^2 - 144.9|u|^3
--   A(y)  = (219 + 44 (1 - |u|)) gA
--   key   : |X| < W + camera  (X = x - 960 virtual px); outside it the halo, a soft key of e = |X| - W
--   inside: R = T1r[E, pg], G = T1g[E, pg] + T2g[D9, pg], B = T2b[D9, pg] with E = W + 0.2 X (the tilted
--           stripes), D9 = W - 0.9 |X| (depth: the blue ring), pg = |y - 520| / 35; tables measured on the clip.
--
--   K1  Waist     the hourglass profile's depth (x0 .. x2): 0 = a straight chain of lenses, 1023 = a steep vase
--   K2  Bulbs     the wave's wavelength (x2.8 .. x0.35): fewer / more lenses
--   K3  Glow      the halo's width (x0.25 .. x4)
--   K4  Speed     bipolar scroll rate (right = down, left = up, +-8 deadband); 700 = the clip (317 lines/s)
--   K5  Colour    slides both colourisers through the lens (+-307 px): stripes, rainbow and ring move
--   K6  Camera    how far the picture (or the Osc) pushes the lens edges and colours
--   S7  Mirror    the wave mirrored about the waist (lenses meet in the middle)
--   S8  Palette   Photo / Ice (red and blue swapped: cyan halo, red ring)
--   S9  Twin      two half-width hourglasses side by side
--   S10 Halo      Glow / Rings (the halo cut into contour rings)
--   S11 Source    Video / Osc
--   P12 PINCH     the wave's amplitude: 0 = a calm glowing vase, 479 = the photo, 100 % = deep lenses and wide gaps
--
-- Engine: vblank-zone sequencer (shared glide, exp2 knob curves on one serial multiplier), a per-line engine
-- kicked at each line start computing the NEXT line (profile ROM, three serial products, quarter-sine ROM),
-- a 19-stage pixel path (twin fold, camera add, two field sums, dithered table addresses, two 1024 x 16 colour
-- ROMs, the glow ROM through a per-pixel scale, RGB -> YUV matrix), the shell's output conditioning (Y, U, V
-- through two [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps sit in block-RAM rings, luma 13/16) and the
-- blanking gate.  Generated avid; one latency (24) in every mode; colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.hourglass_pkg.all;
use work.all;

architecture hourglass of program_top is
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

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_t1_a, r_t2_a : unsigned(9 downto 0) := (others => '0');
    signal r_t1_q, r_t2_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_sp_a : unsigned(8 downto 0) := (others => '0');
    signal r_sp_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_gl_a : unsigned(8 downto 0) := (others => '0');
    signal r_gl_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : std_logic_vector(15 downto 0) := (others => '0');

    attribute no_rw_check : boolean;

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
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(700 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(384 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(479 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_mir, s_pal, s_twin, s_halo, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_neg  : std_logic := '0';
    signal f_a    : unsigned(35 downto 0) := (others => '0');
    signal f_b    : unsigned(15 downto 0) := (others => '0');
    signal f_acc  : unsigned(35 downto 0) := (others => '0');
    signal c_kw   : unsigned(8 downto 0) := to_unsigned(256, 9);     -- waist, Q8
    signal c_inc  : unsigned(15 downto 0) := to_unsigned(22370, 16); -- wave phase per virtual line (2^24 = 1 cycle)
    signal c_gs   : unsigned(9 downto 0) := to_unsigned(252, 10);    -- glow index scale, Q8
    signal c_dth  : signed(20 downto 0) := to_signed(-118260, 21);   -- wave phase per field
    signal c_th0  : unsigned(23 downto 0) := to_unsigned(4194304, 24); -- the field's wave phase (the still: 1/4)
    signal c_eoff : signed(9 downto 0) := (others => '0');
    signal z_d5   : signed(11 downto 0) := (others => '0');
    signal z_t1, z_t2 : signed(20 downto 0) := (others => '0');
    signal c_g6   : unsigned(4 downto 0) := to_unsigned(12, 5);
    signal c_ga   : unsigned(9 downto 0) := to_unsigned(256, 10);
    type t_c4 is array (0 to 3) of signed(10 downto 0);
    signal c_eb, c_db : t_c4 := (others => (others => '0'));   -- 160 / 256 + eoff + dither offset, per dither phase

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ay, e_py : unsigned(10 downto 0) := (others => '0');
    signal e_ym   : unsigned(10 downto 0) := (others => '0');
    signal e_s1, e_s2 : unsigned(20 downto 0) := (others => '0');
    signal e_au   : unsigned(7 downto 0) := (others => '0');
    signal e_ab   : unsigned(8 downto 0) := (others => '0');
    signal e_s    : unsigned(10 downto 0) := (others => '0');
    signal e_aa   : unsigned(9 downto 0) := (others => '0');
    signal e_th   : unsigned(23 downto 0) := (others => '0');
    signal e_sn   : std_logic := '0';
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_w    : signed(11 downto 0) := (others => '0');
    signal n_pg   : unsigned(6 downto 0) := (others => '0');
    signal e_yp   : std_logic := '0';

    -- beam
    signal b_x, b_x1, b_x2 : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal c_xb0 : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal l_w   : signed(11 downto 0) := (others => '0');
    signal l_pg  : unsigned(6 downto 0) := (others => '0');
    signal l_yp  : std_logic := '0';

    -- pixel path
    signal p2_x, p3_x, p4_x, p5_x : signed(11 downto 0) := (others => '0');
    signal p3_ua  : unsigned(10 downto 0) := (others => '0');
    signal p5_u   : unsigned(10 downto 0) := (others => '0');
    signal p4_lm  : unsigned(7 downto 0) := (others => '0');
    signal p5_cam : signed(13 downto 0) := (others => '0');
    signal p6_wc  : signed(12 downto 0) := (others => '0');
    signal p6_xa  : signed(11 downto 0) := (others => '0');
    signal p6_x6  : signed(11 downto 0) := (others => '0');
    signal p6_ub  : signed(11 downto 0) := (others => '0');
    signal p6_u   : unsigned(10 downto 0) := (others => '0');
    signal p7_e1, p7_d, p8_e, p8_d9, p8_d : signed(13 downto 0) := (others => '0');
    signal p7_x6, p7_ub : signed(11 downto 0) := (others => '0');
    signal p8_k   : unsigned(1 downto 0) := (others => '0');
    signal p9_eb, p9_db : signed(13 downto 0) := (others => '0');
    signal p9_pg  : signed(7 downto 0) := (others => '0');
    signal p9_e   : unsigned(9 downto 0) := (others => '0');
    signal p9_in, p9_edge : std_logic := '0';
    signal p10_ie, p10_id : unsigned(5 downto 0) := (others => '0');
    signal p10_pg : unsigned(3 downto 0) := (others => '0');
    signal p10_ga, p10_gb : unsigned(14 downto 0) := (others => '0');
    signal p11_gi : unsigned(9 downto 0) := (others => '0');
    signal p12_rg : std_logic := '0';
    signal p13_rg : std_logic := '0';
    signal p13_t1, p13_t2 : std_logic_vector(15 downto 0) := (others => '0');
    signal p13_gl : unsigned(7 downto 0) := (others => '0');
    signal p14_g  : unsigned(9 downto 0) := (others => '0');
    signal p14_r, p14_b, p14_gl : unsigned(7 downto 0) := (others => '0');
    signal p15_gr : unsigned(15 downto 0) := (others => '0');
    signal p15_gh : unsigned(7 downto 0) := (others => '0');
    signal p15_r, p15_g, p15_b : unsigned(7 downto 0) := (others => '0');
    signal p16_r, p16_g, p16_b : unsigned(7 downto 0) := (others => '0');
    type t_flg is array (10 to 16) of std_logic;
    signal q_in, q_edge : t_flg := (others => '0');
    signal p17_ry, p17_gy, p17_by : unsigned(11 downto 0) := (others => '0');
    signal p17_rr, p17_gr, p17_br, p17_rb, p17_gb, p17_bb : unsigned(13 downto 0) := (others => '0');
    signal p18_y : unsigned(13 downto 0) := (others => '0');
    signal p18_u, p18_v : signed(15 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_k is array (2 to 8) of std_logic;
    signal q_xp : t_k := (others => '0');

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories: line buffer (collisions possible: read-before-write kept),
    -- ROMs, the box rings, the sync ring
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
            r_t1_q <= C_HG_T1(to_integer(r_t1_a));
            r_t2_q <= C_HG_T2(to_integer(r_t2_a));
            r_sp_q <= C_HG_SP(to_integer(r_sp_a));
            r_gl_q <= C_HG_GLOW(to_integer(r_gl_a));
            r_ex_q <= C_HG_EXP2(to_integer(r_ex_a));
        end if;
    end process p_rom;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y19) & std_logic_vector(s_u19(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u19(3 downto 0)) & std_logic_vector(s_v19) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y21) & std_logic_vector(s_u21(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u21(3 downto 0)) & std_logic_vector(s_v21) & "00";
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
        variable v_e  : signed(13 downto 0);
        variable v_p  : unsigned(19 downto 0);
        variable v_q  : signed(19 downto 0);
        variable v_m  : unsigned(15 downto 0);
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
                s_mir  <= registers_in(6)(0);
                s_pal  <= registers_in(6)(1);
                s_twin <= registers_in(6)(2);
                s_halo <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, seven channels
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

            -- serial multiplier (16 iterations; the product is complete 17 clocks after the load)
            if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
            f_a <= f_a(34 downto 0) & '0';
            f_b <= '0' & f_b(15 downto 1);

            -- per-field values: zc 30 .. 100
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 =>                                        -- bulbs: 2^(0.75 (k2 - 512) / 256)
                        v_d := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        v_e := resize(shift_right(shift_left(resize(v_d, 14), 1) + resize(v_d, 14), 2), 14);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 32 => z_mt <= unsigned(r_ex_q);
                    when 33 =>
                        f_a <= to_unsigned(22370, 36); f_b <= z_mt; f_acc <= (others => '0');
                    when 50 =>
                        v_m := f_acc(30 downto 15);
                        case to_integer(z_sh) is
                            when -2     => c_inc <= "00" & v_m(15 downto 2);
                            when -1     => c_inc <= "0" & v_m(15 downto 1);
                            when 0      => c_inc <= v_m;
                            when others => c_inc <= v_m(14 downto 0) & '0';
                        end case;
                    when 53 =>                                        -- glow: index scale 252 * 2^((512 - k3) / 256)
                        v_d := 512 - signed(resize(s_g3(12 downto 3), 12));
                        r_ex_a <= unsigned(v_d(7 downto 0)); z_sh <= v_d(10 downto 8);
                    when 55 => z_mt <= unsigned(r_ex_q);
                    when 56 =>
                        f_a <= to_unsigned(252, 36); f_b <= z_mt; f_acc <= (others => '0');
                    when 73 =>
                        v_m := resize(f_acc(24 downto 15), 16);
                        case to_integer(z_sh) is
                            when -2     => c_gs <= resize(v_m(9 downto 2), 10);
                            when -1     => c_gs <= resize(v_m(9 downto 1), 10);
                            when 0      => c_gs <= v_m(9 downto 0);
                            when 1      => c_gs <= v_m(8 downto 0) & '0';
                            when others => c_gs <= v_m(7 downto 0) & "00";
                        end case;
                    when 74 =>                                        -- speed: (k4 - 512) with a +-8 deadband, x 657
                        v_d := signed(resize(s_g4(12 downto 3), 12)) - 512;
                        if v_d > 8 then
                            f_b <= unsigned(resize(v_d - 8, 16)); z_neg <= '1';
                        elsif v_d < -8 then
                            f_b <= unsigned(resize(-8 - v_d, 16)); z_neg <= '0';
                        else
                            f_b <= (others => '0'); z_neg <= '0';
                        end if;
                        f_a <= to_unsigned(657, 36); f_acc <= (others => '0');
                    when 91 =>
                        if z_neg = '1' then c_dth <= -signed(resize(f_acc(19 downto 0), 21));
                        else c_dth <= signed(resize(f_acc(19 downto 0), 21)); end if;
                    when 95 =>
                        c_th0 <= unsigned(signed(c_th0) + resize(c_dth, 24));
                        z_d5 <= signed(resize(s_g5(12 downto 3), 12)) - 512;
                    when 96 =>
                        c_kw <= s_g1(12 downto 4);
                        c_g6 <= s_g6(12 downto 8);
                        z_t1 <= signed(shift_left(resize(s_gp(12 downto 3), 21), 9)) - signed(shift_left(resize(s_gp(12 downto 3), 21), 5));
                        z_t2 <= shift_left(resize(z_d5, 21), 7) + shift_left(resize(z_d5, 21), 4);
                    when 97 =>
                        z_t1 <= z_t1 + signed(shift_left(resize(s_gp(12 downto 3), 21), 1));
                        z_t2 <= z_t2 + shift_left(resize(z_d5, 21), 3);
                    when 98 =>
                        c_ga <= 31 + unsigned(z_t1(19 downto 10));
                        z_t2 <= z_t2 + shift_left(resize(z_d5, 21), 1);
                    when 99 =>
                        c_eoff <= z_t2(17 downto 8);
                    when 100 =>                                       -- per dither phase k = (x odd, y odd): -6, +2, -2, +6
                        c_eb(0) <= resize(c_eoff, 11) + 154; c_eb(1) <= resize(c_eoff, 11) + 162;
                        c_eb(2) <= resize(c_eoff, 11) + 158; c_eb(3) <= resize(c_eoff, 11) + 166;
                    when 101 =>
                        c_db(0) <= resize(c_eoff, 11) + 250; c_db(1) <= resize(c_eoff, 11) + 258;
                        c_db(2) <= resize(c_eoff, 11) + 254; c_db(3) <= resize(c_eoff, 11) + 262;
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
    -- 0), computes the NEXT line: |u| -> the hourglass profile P (ROM) and
    -- S = 107 + kW P; the amplitude A = (219 + 44 (1 - |u|)) gA; the wave
    -- phase theta0 + y inc -> quarter-sine ROM; W = S + A sin; the page
    -- |y - 520| / 35 (Q3).  One serial product at a time (16 iterations,
    -- complete at load + 17).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t  : unsigned(7 downto 0);
        variable v_w  : signed(12 downto 0);
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
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then e_ym <= to_unsigned(1079, 11); else e_ym <= e_vy(18 downto 8); end if;
                when 2 =>
                    if e_ym >= 530 then e_ay <= e_ym - 530; else e_ay <= 530 - e_ym; end if;
                    if e_ym >= 520 then e_py <= e_ym - 520; else e_py <= 520 - e_ym; end if;
                when 3 =>                                             -- |u| in 1/256: (ay * 485) >> 10
                    e_s1 <= shift_left(resize(e_ay, 21), 9) - shift_left(resize(e_ay, 21), 5);
                    e_s2 <= shift_left(resize(e_ay, 21), 2) + resize(e_ay, 21);
                    if s_mir = '1' then e_ym <= e_py + 520; end if;  -- Mirror: the wave's phase by |y - 520|
                when 4 =>
                    e_s1 <= e_s1 + e_s2;
                when 5 =>
                    if e_s1(20 downto 10) > 255 then e_au <= (others => '1'); else e_au <= e_s1(17 downto 10); end if;
                when 6 =>
                    r_sp_a <= '1' & e_au;                             -- the profile P (1/8 px)
                    v_t := not e_au;                                  -- 255 - au
                    e_s1 <= shift_left(resize(v_t, 21), 5) + shift_left(resize(v_t, 21), 3);
                    e_s2 <= shift_left(resize(v_t, 21), 2);
                when 7 =>
                    e_s1 <= e_s1 + e_s2;
                when 8 =>
                    e_ab <= to_unsigned(219, 9) + resize(e_s1(15 downto 8), 9);
                    m_a <= resize(unsigned(r_sp_q(11 downto 0)), 36); m_b <= resize(c_kw, 16); m_acc <= (others => '0');
                when 9 =>                                             -- page, Q3: (py * 234) >> 10
                    e_s1 <= shift_left(resize(e_py, 21), 8) - shift_left(resize(e_py, 21), 4);
                    e_s2 <= shift_left(resize(e_py, 21), 2) + shift_left(resize(e_py, 21), 1);
                when 10 =>
                    e_s1 <= e_s1 - e_s2;
                when 11 =>
                    if e_s1(20 downto 10) > 127 then n_pg <= (others => '1'); else n_pg <= e_s1(16 downto 10); end if;
                when 25 =>
                    e_s <= to_unsigned(107, 11) + resize(m_acc(21 downto 11), 11);
                    m_a <= resize(e_ab, 36); m_b <= resize(c_ga, 16); m_acc <= (others => '0');
                when 42 =>
                    e_aa <= m_acc(17 downto 8);
                    m_a <= resize(e_ym, 36); m_b <= c_inc; m_acc <= (others => '0');
                when 59 =>
                    e_th <= c_th0 + m_acc(23 downto 0);
                when 60 =>
                    if e_th(22) = '1' then r_sp_a <= '0' & not e_th(21 downto 14);
                    else r_sp_a <= '0' & e_th(21 downto 14); end if;
                    e_sn <= e_th(23);
                when 62 =>
                    m_a <= resize(e_aa, 36); m_b <= unsigned(r_sp_q); m_acc <= (others => '0');
                when 79 =>
                    if e_sn = '1' then v_w := signed(resize(e_s, 13)) - signed(resize(m_acc(24 downto 15), 13));
                    else v_w := signed(resize(e_s, 13)) + signed(resize(m_acc(24 downto 15), 13)); end if;
                    n_w <= v_w(11 downto 0);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): x in h (half virtual px); the line's W and page latched
    -- at the line start; the camera read issued from the stage-1 pixel index
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zone = '1' then
                l_yp <= '1';
            elsif g_start = '1' then
                b_x  <= c_xb0;
                b_px <= (others => '0');
                l_w  <= n_w;
                l_pg <= n_pg;
                l_yp <= not l_yp;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_l  : signed(8 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_gl : unsigned(8 downto 0);
        variable v_k  : integer range 0 to 3;
        variable v_r, v_gg, v_b : unsigned(7 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_x  : signed(15 downto 0);
        variable v_t  : signed(13 downto 0);
        variable v_a  : signed(11 downto 0);
        variable v_gm : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: X = x - 960 (virtual px)
            p2_x <= signed('0' & b_x(11 downto 1)) - 960;
            q_xp(2) <= b_px(0);
            for i in 3 to 8 loop q_xp(i) <= q_xp(i - 1); end loop;
            -- 3: |X|
            v_a := abs(p2_x); p3_ua <= unsigned(v_a(10 downto 0));
            p3_x  <= p2_x;
            -- 4: Twin folds the frame at its centre into two half-width copies; the camera luma (line buffer copy)
            if s_twin = '1' then p4_x <= signed('0' & p3_ua) + signed('0' & p3_ua) - 960; else p4_x <= p3_x; end if;
            p4_lm <= unsigned(s_lb_q);
            -- 5: U = |X|; camera x gain (Osc is zero-mean)
            v_a := abs(p4_x); p5_u <= unsigned(v_a(10 downto 0));
            p5_x <= p4_x;
            if s_src = '1' then v_l := signed('0' & p4_lm) - 120; else v_l := signed('0' & p4_lm); end if;
            p5_cam <= resize(v_l * signed('0' & c_g6), 14);
            -- 6: Wc = W + camera; partial sums of 0.203 X and 0.094 U
            p6_wc <= resize(l_w, 13) + resize(shift_right(p5_cam, 4), 13);
            p6_xa <= shift_right(p5_x, 3) + shift_right(p5_x, 4);
            p6_x6 <= shift_right(p5_x, 6);
            p6_ub <= signed('0' & shift_right(p5_u, 3)) - signed('0' & shift_right(p5_u, 5));
            p6_u  <= p5_u;
            -- 7: E part, D = Wc - U
            p7_e1 <= resize(p6_wc, 14) + resize(p6_xa, 14);
            p7_d  <= resize(p6_wc, 14) - signed(resize(p6_u, 14));
            p7_x6 <= p6_x6; p7_ub <= p6_ub;
            -- 8: E = Wc + 0.203 X (the stripes' field), D9 = Wc - 0.906 U (the depth), D (the key)
            p8_e  <= p7_e1 + resize(p7_x6, 14);
            p8_d9 <= p7_d + resize(p7_ub, 14);
            p8_d  <= p7_d;
            p8_k  <= q_xp(7) & l_yp;
            -- 9: table offsets with the 2x2 dither; halo distance; inside / edge flags
            v_k := to_integer(p8_k);
            p9_eb <= p8_e + resize(c_eb(v_k), 14);
            p9_db <= p8_d9 + resize(c_db(v_k), 14);
            case v_k is
                when 0      => p9_pg <= signed('0' & l_pg) - 3;
                when 1      => p9_pg <= signed('0' & l_pg) + 1;
                when 2      => p9_pg <= signed('0' & l_pg) - 1;
                when others => p9_pg <= signed('0' & l_pg) + 3;
            end case;
            if p8_d >= 0 then p9_e <= (others => '0');
            elsif p8_d < -1023 then p9_e <= (others => '1');
            else v_t := -p8_d; p9_e <= unsigned(v_t(9 downto 0)); end if;
            if p8_d > 0 then p9_in <= '1'; else p9_in <= '0'; end if;
            if p8_d > 0 and p8_d < 4 then p9_edge <= '1'; else p9_edge <= '0'; end if;
            -- 10: clamped table indices; glow distance x scale (K3)
            if p9_eb < 0 then p10_ie <= (others => '0');
            elsif p9_eb > 1023 then p10_ie <= (others => '1');
            else p10_ie <= unsigned(p9_eb(9 downto 4)); end if;
            if p9_db < 0 then p10_id <= (others => '0');
            elsif p9_db > 1023 then p10_id <= (others => '1');
            else p10_id <= unsigned(p9_db(9 downto 4)); end if;
            if p9_pg < 0 then p10_pg <= (others => '0');
            elsif p9_pg > 127 then p10_pg <= (others => '1');
            else p10_pg <= unsigned(p9_pg(6 downto 3)); end if;
            p10_ga <= p9_e * c_gs(9 downto 5);
            p10_gb <= p9_e * c_gs(4 downto 0);
            q_in(10) <= p9_in; q_edge(10) <= p9_edge;
            for i in 11 to 16 loop q_in(i) <= q_in(i - 1); q_edge(i) <= q_edge(i - 1); end loop;
            -- 11: ROM addresses (T1 by E, T2 by D9, both by page; glow by the scaled distance)
            r_t1_a <= p10_ie & p10_pg;
            r_t2_a <= p10_id & p10_pg;
            v_gm := shift_left(resize(p10_ga, 20), 5) + resize(p10_gb, 20);
            if v_gm(19 downto 17) /= "000" then r_gl_a <= (others => '1'); p11_gi <= (others => '1');
            else r_gl_a <= v_gm(16 downto 8); p11_gi <= resize(v_gm(16 downto 8), 10); end if;
            -- 12: ROM data (EBR); the halo ring phase
            if p11_gi(4 downto 0) < 10 then p12_rg <= '1'; else p12_rg <= '0'; end if;
            -- 13: fabric copies
            p13_t1 <= r_t1_q; p13_t2 <= r_t2_q; p13_gl <= unsigned(r_gl_q); p13_rg <= p12_rg;
            -- 14: G = G1 + G2 (- 80 next), R, B; the halo (Rings: bright rings, dim gaps)
            p14_g <= resize(unsigned(p13_t1(7 downto 0)), 10) + resize(unsigned(p13_t2(15 downto 8)), 10);
            p14_r <= unsigned(p13_t1(15 downto 8));
            p14_b <= unsigned(p13_t2(7 downto 0));
            if s_halo = '1' then
                if p13_rg = '1' then
                    v_gl := p13_gl & '0';
                    if v_gl > 185 then p14_gl <= to_unsigned(185, 8); else p14_gl <= v_gl(7 downto 0); end if;
                else
                    p14_gl <= shift_right(p13_gl, 3);
                end if;
            else
                p14_gl <= p13_gl;
            end if;
            -- 15: clamp G; the halo's red (0.87 of its green)
            if p14_g < 80 then p15_g <= (others => '0');
            elsif p14_g > 335 then p15_g <= (others => '1');
            else v_g := p14_g - 80; p15_g <= v_g(7 downto 0); end if;
            p15_r <= p14_r; p15_b <= p14_b;
            p15_gr <= shift_left(resize(p14_gl, 16), 8) - shift_left(resize(p14_gl, 16), 5) - resize(p14_gl, 16);
            p15_gh <= p14_gl;
            -- 16: inside (tables, the edge line at half) / outside (the halo); palette (Ice swaps R and B)
            if q_in(15) = '1' then
                if q_edge(15) = '1' then
                    v_r := '0' & p15_r(7 downto 1); v_gg := '0' & p15_g(7 downto 1); v_b := '0' & p15_b(7 downto 1);
                else
                    v_r := p15_r; v_gg := p15_g; v_b := p15_b;
                end if;
            else
                v_r := 17 + p15_gr(15 downto 8); v_gg := 15 + p15_gh; v_b := 4 + ("00000" & p15_gh(7 downto 5));
            end if;
            if s_pal = '1' then p16_r <= v_b; p16_b <= v_r; else p16_r <= v_r; p16_b <= v_b; end if;
            p16_g <= v_gg;
            -- 17: matrix products: Y = 64 + (5R + 10G + 2B) / 4; Cr = 512 + (56R - 47G - 9B) / 32;
            --     Cb = 512 + (-19R - 37G + 56B) / 32   (luma pre-scaled 16/13, capped 940)
            p17_ry <= shift_left(resize(p16_r, 12), 2) + resize(p16_r, 12);
            p17_gy <= shift_left(resize(p16_g, 12), 3) + shift_left(resize(p16_g, 12), 1);
            p17_by <= shift_left(resize(p16_b, 12), 1);
            p17_rr <= shift_left(resize(p16_r, 14), 6) - shift_left(resize(p16_r, 14), 3);
            p17_gr <= shift_left(resize(p16_g, 14), 5) + shift_left(resize(p16_g, 14), 4) - resize(p16_g, 14);
            p17_br <= shift_left(resize(p16_b, 14), 3) + resize(p16_b, 14);
            p17_rb <= shift_left(resize(p16_r, 14), 4) + shift_left(resize(p16_r, 14), 1) + resize(p16_r, 14);
            p17_gb <= shift_left(resize(p16_g, 14), 5) + shift_left(resize(p16_g, 14), 2) + resize(p16_g, 14);
            p17_bb <= shift_left(resize(p16_b, 14), 6) - shift_left(resize(p16_b, 14), 3);
            -- 18: sums
            p18_y <= resize(p17_ry, 14) + resize(p17_gy, 14) + resize(p17_by, 14);
            p18_u <= signed(resize(p17_rr, 16)) - signed(resize(p17_gr, 16)) - signed(resize(p17_br, 16));
            p18_v <= signed(resize(p17_bb, 16)) - signed(resize(p17_rb, 16)) - signed(resize(p17_gb, 16));
            -- 19: offsets, luma cap 940, black outside the generated avid
            v_y := resize(p18_y(13 downto 2), 12) + 64;
            if s_gav_sr(16) = '0' then
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            else
                if v_y > 940 then s_y19 <= to_unsigned(940, 10); else s_y19 <= v_y(9 downto 0); end if;
                v_x := shift_right(p18_u, 5) + 512; s_u19 <= unsigned(v_x(9 downto 0));
                v_x := shift_right(p18_v, 5) + 512; s_v19 <= unsigned(v_x(9 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
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
            sy_ra <= rg_wa - 21;
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y19 - s_cay;
                s_bau <= s_bau + s_u19 - s_cau;
                s_bav <= s_bav + s_v19 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y21 - s_cby;
                s_bbu <= s_bbu + s_u21 - s_cbu;
                s_bbv <= s_bbv + s_v21 - s_cbv;
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v23 <= v_sc(13 downto 4);
            -- 24: luma 13/16
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u23; s_v24 <= s_v23;
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
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture hourglass;
