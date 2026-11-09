-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lantern v0.1.0 -- Heirloom (phase 3, video clips), from the clip WIN_20231002_01_20_50 at 0.5 s.
--
-- The still: a solid hot-pink lantern with a pointed apex, wrapped in nested petal contours (thin yellow
-- lines with dark brown cores, purple and pink between them) that cross each other in X shapes in three
-- fixed rows; the left edge holds the line-start value (thick horizontal yellow bars).  In the clip the
-- lantern sinks at ~181 lines/s: it leaves at the bottom, the frame becomes an X hourglass, the next
-- lantern comes in from the top.
--
-- The patch, as fitted (lantern.py has every constant):
--   A   = KA u + KB u^2,  u = |x - 987| (left half: u * (1 + 1/128), + DL)    |x| fold plus x^2 parabola
--   Vg  = G0 + GV (1 - cos(2 pi (y - Y) / 2500)) / 2                           vertical sine (drifting)
--   Lh  = LAM (y - 517.7)                                                      static vertical ramp
--   f1  = A + Vg + Lh,  f2 = A + Vg - Lh                                       two crossing families
--   e_i = floor - f_i below the floor (2), else the distance to the nearest integer
--   s   = min(e1, e2) -> a 1-D palette (yellow line, brown core, purple, pink)
--
--   K1  Rings     ring gain across: one giant lantern (0) .. the still (512) .. 3x the petals (1023)
--   K2  Point     the lantern's shape: round oval rings (0) .. the still .. straight pointed chevrons (1023)
--   K3  Height    the vertical bowl: a tall column over a diamond lattice (0) .. the still .. squat, stacked (1023)
--   K4  Drift     bipolar: the lantern sinks (right) or rises (left); 739 = the clip's 181 lines/s
--   K5  Lantern   the first drawn ring (the lantern's size): inner rings shown (0) .. the still .. 1.6x (1023)
--   K6  Bend      how far the camera (or the Osc) bends the contours
--   S7  Twin      one lantern / two side by side
--   S8  Ink       Neon (the still) / Night (glowing lines on black)
--   S9  Gel       Magenta (the still) / Cyan
--   S10 Mirror    off / the bottom half mirrors the top (a diamond lantern)
--   S11 Source    Video / Osc
--   P12 LATTICE   the tilt between the two families: a clean nested lantern (0) .. the still (400) ..
--                 a dense crossing net (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide) that refills the 1024-entry A(u) table
-- by a second-order DDA; a per-line engine kicked at each line start (computes the NEXT line: the sine by
-- interpolated ROM, Vg and Lh on its own serial multiplier); a 20-stage pixel path.  Virtual 1080-line
-- coordinates (SD shows the centre 4:3 of the picture).  Generated avid; output conditioning (luma [1 2 1],
-- chroma [1 2 2 2 2 2 2 2 1]/16 as running 8-sums that resync to neutral in every blanking, luma 13/16)
-- before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lantern_pkg.all;
use work.all;

architecture lantern of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (h = half virtual px; field values in periods, Q12)
    constant C_XH   : integer := 110;       -- line-start hold (h)
    constant C_DL   : integer := -131;      -- left-half offset, Q12
    constant C_G0   : integer := 2303;      -- bowl minimum, Q12
    constant K_GV   : integer := 10502;     -- bowl peak-to-valley, Q12
    constant C_KP   : integer := 6710;      -- 2^24 / 2500.47: vertical phase per virtual line
    constant K_PH0  : integer := 4834890;   -- 720.55 * C_KP: the valley on line 720.55 at load
    constant K_LAM  : integer := 16145;     -- 0.96234e-3 * 2^24: tilt per virtual line
    constant C_YL8  : integer := 132524;    -- 517.67 * 256: tilt zero line, Q8
    constant C_KA28 : integer := 588129;    -- 2.19095e-3 * 2^28
    constant C_KB36 : integer := 76633;     -- 1.11515e-6 * 2^36

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

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the oscillator)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- A(u) table: Q12 periods by u = virtual px from the axis; written by the per-field DDA in the vblank
    -- zone only, read by the pixel path
    type t_at is array (0 to 1023) of unsigned(15 downto 0);
    signal ta : t_at := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of ta : signal is true;
    signal t_we, t_run, t_sat : std_logic := '0';
    signal t_wa, t_ra : unsigned(9 downto 0) := (others => '0');
    signal t_wd : unsigned(15 downto 0) := (others => '0');
    signal t_rq : unsigned(15 downto 0) := (others => '0');
    signal t_i  : unsigned(9 downto 0) := (others => '0');
    signal t_q  : unsigned(32 downto 0) := (others => '0');
    signal t_d  : unsigned(31 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_cs_a : unsigned(9 downto 0) := (others => '0');
    signal r_cs_q : unsigned(15 downto 0) := (others => '0');
    signal r_pa   : unsigned(9 downto 0) := (others => '0');
    signal r_py, r_pu, r_pv : unsigned(7 downto 0) := (others => '0');

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
    signal s_xc   : unsigned(11 downto 0) := to_unsigned(1974, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(739, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg, s_vnegf : std_logic := '0';
    signal s_twin, s_ink, s_gel, s_mir, s_src : std_logic := '0';

    -- per-field constants
    signal z_k   : unsigned(9 downto 0) := (others => '0');
    signal z_hi  : std_logic := '0';
    signal c_g1  : unsigned(11 downto 0) := to_unsigned(1024, 12);   -- ring gain, Q10
    signal c_h3  : unsigned(11 downto 0) := to_unsigned(1024, 12);   -- height gain, Q10
    signal c_m   : unsigned(12 downto 0) := to_unsigned(1024, 13);   -- lattice gain, Q10
    signal c_k2  : unsigned(10 downto 0) := to_unsigned(512, 11);    -- point
    signal c_k2n : unsigned(10 downto 0) := to_unsigned(512, 11);    -- 1024 - point
    signal c_a1  : unsigned(20 downto 0) := (others => '0');
    signal c_b1  : unsigned(17 downto 0) := (others => '0');
    signal c_ae  : unsigned(21 downto 0) := (others => '0');        -- a_eff, Q28 per virtual px
    signal c_be  : unsigned(11 downto 0) := (others => '0');        -- b_eff, Q28 per virtual px^2
    signal c_gv  : unsigned(15 downto 0) := to_unsigned(K_GV, 16);  -- Q12
    signal c_lam : unsigned(15 downto 0) := to_unsigned(K_LAM, 16); -- Q24 per virtual line
    signal c_fl  : unsigned(13 downto 0) := to_unsigned(8192, 14);  -- floor, Q12
    signal c_vm2 : unsigned(17 downto 0) := (others => '0');
    signal c_stp : unsigned(16 downto 0) := (others => '0');        -- phase step per field (2^24 per cycle)
    signal c_ph0 : unsigned(23 downto 0) := to_unsigned(K_PH0, 24);
    signal c_kq  : unsigned(8 downto 0) := (others => '0');         -- bend gain, Q4 per luma step
    signal c_d0  : unsigned(31 downto 0) := (others => '0');        -- table DDA start step (a + b), Q28
    signal c_d2  : unsigned(31 downto 0) := (others => '0');        -- 2b, Q28
    signal c_xb0 : unsigned(11 downto 0) := to_unsigned(16, 12);    -- beam start x (h): x0 + 8 xs (the generated avid's lag)

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');       -- next line, Q8 virtual lines
    signal e_yv   : unsigned(18 downto 0) := (others => '0');       -- after the mirror
    signal e_lo   : std_logic := '0';
    signal e_yd   : unsigned(17 downto 0) := (others => '0');       -- |y - YL|, Q8
    signal e_yn   : std_logic := '0';                               -- y < YL
    signal e_ph   : unsigned(23 downto 0) := (others => '0');
    signal e_c0, e_c1 : unsigned(15 downto 0) := (others => '0');
    signal e_dl   : unsigned(15 downto 0) := (others => '0');       -- |C1 - C0|
    signal e_dn   : std_logic := '0';                               -- C1 < C0
    signal e_ci   : unsigned(15 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_vg, n_vgl : unsigned(16 downto 0) := to_unsigned(C_G0, 17);
    signal e_vd   : signed(19 downto 0) := (others => '0');
    signal e_cd   : signed(16 downto 0) := (others => '0');
    signal n_lh   : signed(15 downto 0) := (others => '0');

    -- beam
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_vg, l_vgl : unsigned(16 downto 0) := to_unsigned(C_G0, 17);
    signal l_lh  : signed(15 downto 0) := (others => '0');

    -- pixel path (qK = stage K after the beam values)
    signal q1_x  : unsigned(11 downto 0) := (others => '0');
    signal q2_d  : signed(12 downto 0) := (others => '0');
    signal q3_u  : unsigned(11 downto 0) := (others => '0');
    signal q3_lf : std_logic := '0';
    signal q4_ix : unsigned(11 downto 0) := (others => '0');
    signal q4_lf, q5_lf, q6_lf, q7_lf : std_logic := '0';
    signal q7_t  : unsigned(15 downto 0) := (others => '0');
    signal q8_g  : unsigned(17 downto 0) := (others => '0');
    signal q3_l0, q3_l1, q3_l2, q3_l3 : unsigned(7 downto 0) := (others => '0');
    signal q4_ls : unsigned(9 downto 0) := (others => '0');
    signal q5_l  : unsigned(7 downto 0) := (others => '0');
    signal q6_pl, q6_ph : unsigned(12 downto 0) := (others => '0');
    signal q7_b  : unsigned(12 downto 0) := (others => '0');
    signal q8_b  : unsigned(14 downto 0) := (others => '0');
    signal q9_g  : unsigned(17 downto 0) := (others => '0');
    signal q10_f1, q10_f2 : signed(18 downto 0) := (others => '0');
    signal q11_d1, q11_d2 : signed(18 downto 0) := (others => '0');
    signal q11_r1, q11_r2 : unsigned(11 downto 0) := (others => '0');
    signal q12_e1, q12_e2 : unsigned(11 downto 0) := (others => '0');
    signal q13_s : unsigned(11 downto 0) := (others => '0');
    signal q16_y, q16_u, q16_v : unsigned(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cu, s_cv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_wy : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), table, ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_ta : process(clk)
    begin
        if rising_edge(clk) then
            t_rq <= ta(to_integer(t_ra));
            if t_we = '1' then ta(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_ta;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_cs_q <= C_LT_COS(to_integer(r_cs_a));
            r_py <= C_LT_PY(to_integer(r_pa));
            r_pu <= C_LT_PU(to_integer(r_pa));
            r_pv <= C_LT_PV(to_integer(r_pa));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
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
            -- drift: bipolar, +-8 deadband
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
                s_twin <= registers_in(6)(0);
                s_ink  <= registers_in(6)(1);
                s_gel  <= registers_in(6)(2);
                s_mir  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); s_xc <= to_unsigned(1976, 12);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9); s_xc <= to_unsigned(1974, 12);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9); s_xc <= to_unsigned(1974, 12);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
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
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 64 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at load + 18).  The slot number is decoded at 62 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 63 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 62  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 278 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 302 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 326 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  =>                                                                   -- ring gain, low slope
                        z_fa <= resize(s_g1(12 downto 3), 36); z_fb <= to_unsigned(1328, 16);
                    when 1  =>                                                                   -- lattice, two slopes
                        z_fa <= resize(z_k, 36);
                        if z_hi = '1' then z_fb <= to_unsigned(5056, 16); else z_fb <= to_unsigned(2621, 16); end if;
                    when 2  =>                                                                   -- floor, two slopes
                        z_fa <= resize(z_k, 36);
                        if z_hi = '1' then z_fb <= to_unsigned(9728, 16); else z_fb <= to_unsigned(11520, 16); end if;
                    when 3  => z_fa <= to_unsigned(C_KA28, 36);       z_fb <= resize(c_g1, 16);   -- KA * g1
                    when 4  => z_fa <= resize(c_a1, 36);              z_fb <= resize(c_k2, 16);   -- * 2p
                    when 5  => z_fa <= to_unsigned(C_KB36, 36);       z_fb <= resize(c_g1, 16);   -- KB * g1
                    when 6  => z_fa <= resize(c_b1, 36);              z_fb <= resize(c_k2n, 16);  -- * 2(1 - p)
                    when 7  => z_fa <= to_unsigned(K_GV, 36);         z_fb <= resize(c_h3, 16);   -- Gv
                    when 8  => z_fa <= to_unsigned(K_LAM, 36);        z_fb <= resize(c_m, 16);    -- lambda
                    when 9  => z_fa <= resize(s_vm, 36);              z_fb <= resize(s_vm, 16);   -- speed^2
                    when 10 => z_fa <= resize(c_vm2, 36);             z_fb <= to_unsigned(27697, 16);
                    when others => z_fa <= resize(s_g6(12 downto 3), 36); z_fb <= resize(s_g6(12 downto 3), 16); -- bend^2
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's step)
                    when 20 =>
                        if s_vnegf = '1' then c_ph0 <= c_ph0 - resize(c_stp, 24);
                        else c_ph0 <= c_ph0 + resize(c_stp, 24); end if;
                    when 21 =>
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                    -- height gain: 2k (k <= 512), 1024 + 4 (k - 512)
                    when 30 =>
                        if s_g3(12 downto 3) > 512 then
                            c_h3 <= shift_left(resize(s_g3(12 downto 3) - 512, 12), 2) + 1024;
                        else
                            c_h3 <= shift_left(resize(s_g3(12 downto 3), 12), 1);
                        end if;
                        c_k2  <= resize(s_g2(12 downto 3), 11);
                    when 31 =>
                        c_k2n <= to_unsigned(1024, 11) - c_k2;
                    -- lattice operand (P12): k <= 400 low slope, else k - 400 on the high slope
                    when 40 =>
                        if s_gp(12 downto 3) > 400 then z_k <= s_gp(12 downto 3) - 400; z_hi <= '1';
                        else z_k <= s_gp(12 downto 3); z_hi <= '0'; end if;
                    when 84 =>                                    -- ring gain (slot 0)
                        if s_g1(12 downto 3) > 512 then
                            c_g1 <= shift_left(resize(s_g1(12 downto 3) - 512, 12), 2) + 1024;
                        else
                            c_g1 <= f_acc(21 downto 10) + 360;
                        end if;
                    when 108 =>                                   -- lattice gain (slot 1)
                        if z_hi = '1' then c_m <= resize(f_acc(21 downto 10), 13) + 1024;
                        else c_m <= resize(f_acc(20 downto 10), 13); end if;
                        -- floor operand (K5): k <= 512 low slope, else k - 512 on the high slope
                        if s_g5(12 downto 3) > 512 then z_k <= s_g5(12 downto 3) - 512; z_hi <= '1';
                        else z_k <= s_g5(12 downto 3); z_hi <= '0'; end if;
                    when 132 =>                                   -- floor (slot 2), Q12
                        if z_hi = '1' then c_fl <= resize(f_acc(22 downto 10), 14) + 8192;
                        else c_fl <= resize(f_acc(22 downto 10), 14) + 2432; end if;
                    when 156 => c_a1 <= f_acc(30 downto 10);      -- KA28 g1 / 1024
                    when 180 => c_ae <= f_acc(30 downto 9);       -- a_eff (Q28)
                    when 204 => c_b1 <= f_acc(27 downto 10);      -- KB36 g1 / 1024
                    when 228 => c_be <= f_acc(28 downto 17);      -- b_eff (Q28)
                    when 229 => c_d0 <= resize(c_ae, 32) + resize(c_be, 32);
                                c_d2 <= shift_left(resize(c_be, 32), 1);
                    when 252 => c_gv <= f_acc(25 downto 10);
                    when 276 => c_lam <= f_acc(25 downto 10);
                    when 300 => c_vm2 <= f_acc(17 downto 0);
                    when 324 => c_stp <= f_acc(32 downto 16); s_vnegf <= s_vneg;
                    when 348 => c_kq <= f_acc(19 downto 11) - resize(f_acc(19 downto 14), 9);
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
    -- A(u) table fill (vblank zone, zc 400 .. 1423): A(u + 1) = A(u) + D(u), D(u + 1) = D(u) + 2b, so no
    -- multiplier; Q28 periods, the table keeps Q12 and saturates past 16 periods.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            t_we <= '0';
            if s_zone = '1' and s_zc = 400 then
                t_run <= '1'; t_i <= (others => '0'); t_q <= (others => '0'); t_d <= c_d0; t_sat <= '0';
            elsif t_run = '1' then
                t_we <= '1'; t_wa <= t_i;
                if t_sat = '1' or t_q(32) = '1' then
                    t_wd <= (others => '1'); t_sat <= '1';
                else
                    t_wd <= t_q(31 downto 16);
                end if;
                t_q <= t_q + t_d; t_d <= t_d + c_d2; t_i <= t_i + 1;
                if t_i = 1023 then t_run <= '0'; end if;
            end if;
        end if;
    end process p_fill;

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
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on its own serial multiplier (16 iterations, result
    -- at load + 17):
    --   y' = y (Mirror: 1080 - y below the middle)
    --   ph = y' * KP - ph0;  C = cos table at ph, interpolated (8-bit fraction)
    --   Vg = G0 + C * Gv;    Lh = lambda * (y' - YL)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_n : signed(19 downto 0);
        variable v_cn : signed(16 downto 0);
        variable v_t : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1440 then e_zk <= '1'; end if;     -- after the table fill

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if s_mir = '1' and e_vy >= to_unsigned(138240, 19) then e_lo <= '1'; else e_lo <= '0'; end if;
                when 2 =>
                    if e_lo = '1' then e_yv <= to_unsigned(276480, 19) - e_vy; else e_yv <= e_vy; end if;
                when 3 =>
                    m_a <= resize(e_yv, 36); m_b <= to_unsigned(C_KP, 16); m_acc <= (others => '0');
                    e_vd <= signed(resize(e_yv, 20)) - C_YL8;
                when 4 =>
                    v_n := -e_vd;                               -- negate at full width, then slice
                    if e_vd < 0 then e_yd <= unsigned(v_n(17 downto 0)); e_yn <= '1';
                    else e_yd <= unsigned(e_vd(17 downto 0)); e_yn <= '0'; end if;
                when 20 =>
                    e_ph <= m_acc(31 downto 8) - c_ph0;
                    m_a <= resize(e_yd, 36); m_b <= c_lam; m_acc <= (others => '0');
                when 21 =>
                    r_cs_a <= e_ph(23 downto 14);
                when 23 =>
                    e_c0 <= r_cs_q;
                    r_cs_a <= e_ph(23 downto 14) + 1;
                when 25 =>
                    e_c1 <= r_cs_q;
                when 26 =>
                    e_cd <= signed(resize(e_c1, 17)) - signed(resize(e_c0, 17));
                when 27 =>
                    v_cn := -e_cd;
                    if e_cd < 0 then e_dl <= unsigned(v_cn(15 downto 0)); e_dn <= '1';
                    else e_dl <= unsigned(e_cd(15 downto 0)); e_dn <= '0'; end if;
                when 37 =>
                    if e_yn = '1' then n_lh <= -signed(resize(m_acc(34 downto 20), 16));
                    else n_lh <= signed(resize(m_acc(34 downto 20), 16)); end if;
                    m_a <= resize(e_dl, 36); m_b <= resize(e_ph(13 downto 6), 16); m_acc <= (others => '0');
                when 54 =>
                    if e_dn = '1' then e_ci <= e_c0 - m_acc(23 downto 8);
                    else e_ci <= e_c0 + m_acc(23 downto 8); end if;
                when 55 =>
                    m_a <= resize(e_ci, 36); m_b <= c_gv; m_acc <= (others => '0');
                when 72 =>
                    v_t := resize(m_acc(31 downto 16), 17) + C_G0;
                    n_vg <= v_t;
                when 73 =>
                    n_vgl <= n_vg - to_unsigned(-C_DL, 17);     -- the left half's offset, folded in per line
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: pixel index and x in h units (x0 + n xs); the line's vertical terms
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_px <= (others => '0');
                l_vg <= n_vg; l_vgl <= n_vgl; l_lh <= n_lh;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ix : unsigned(11 downto 0);
        variable v_s  : unsigned(9 downto 0);
        variable v_q  : unsigned(8 downto 0);
        variable v_ad : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: line-start hold
            if b_x < C_XH then q1_x <= to_unsigned(C_XH, 12); else q1_x <= b_x; end if;
            -- 2: signed distance to the axis (twin: to the nearer of 960 / 2880 h)
            if s_twin = '1' then
                if q1_x < 1920 then q2_d <= signed(resize(q1_x, 13)) - 960;
                else q2_d <= signed(resize(q1_x, 13)) - 2880; end if;
            else
                q2_d <= signed(resize(q1_x, 13)) - signed(resize(s_xc, 13));
            end if;
            -- 3: |d| (h), left flag (not in twin)
            v_ad := abs(q2_d);
            q3_u <= unsigned(v_ad(11 downto 0));
            if q2_d < 0 and s_twin = '0' then q3_lf <= '1'; else q3_lf <= '0'; end if;
            -- 4: table index in virtual px (twin: 2 u, so the lanterns are half as wide); left: u (1 + 1/128)
            if s_twin = '1' then v_ix := q3_u; else v_ix := shift_right(q3_u, 1); end if;
            if q3_lf = '1' then v_ix := v_ix + shift_right(v_ix, 7); end if;
            q4_ix <= v_ix; q4_lf <= q3_lf;
            -- 5: table address (saturated)
            if q4_ix > 1023 then t_ra <= (others => '1'); else t_ra <= q4_ix(9 downto 0); end if;
            q5_lf <= q4_lf;
            -- 6: table data (EBR); 7: fabric copy
            q6_lf <= q5_lf;
            q7_t <= t_rq; q7_lf <= q6_lf;
            -- 8: G = A + Vg (+ DL on the left: the line's Vg - 131)
            if q7_lf = '1' then q8_g <= resize(q7_t, 18) + resize(l_vgl, 18);
            else q8_g <= resize(q7_t, 18) + resize(l_vg, 18); end if;

            -- luma: 3 the line buffer's pixel and three before it, 4 their sum, 5 the mean,
            -- 6 x bend gain (nibble partials), 7 the bend, 8 x4 for the Osc
            q3_l0 <= unsigned(s_lb_q); q3_l1 <= q3_l0; q3_l2 <= q3_l1; q3_l3 <= q3_l2;
            q4_ls <= resize(q3_l0, 10) + resize(q3_l1, 10) + resize(q3_l2, 10) + resize(q3_l3, 10);
            q5_l  <= q4_ls(9 downto 2);
            q6_pl <= resize(q5_l(3 downto 0) * c_kq, 13);
            q6_ph <= resize(q5_l(7 downto 4) * c_kq, 13);
            q7_b  <= resize(shift_right(resize(q6_pl, 17) + shift_left(resize(q6_ph, 17), 4), 4), 13);
            if s_src = '1' then q8_b <= shift_left(resize(q7_b, 15), 2); else q8_b <= resize(q7_b, 15); end if;

            -- 9: + bend
            q9_g <= q8_g + resize(q8_b, 18);
            -- 10: the two families
            q10_f1 <= signed(resize(q9_g, 19)) + resize(l_lh, 19);
            q10_f2 <= signed(resize(q9_g, 19)) - resize(l_lh, 19);
            -- 11: below the floor? (d > 0), and the fractional part
            q11_d1 <= signed(resize(c_fl, 19)) - q10_f1;
            q11_d2 <= signed(resize(c_fl, 19)) - q10_f2;
            q11_r1 <= unsigned(q10_f1(11 downto 0));
            q11_r2 <= unsigned(q10_f2(11 downto 0));
            -- 12: distance to the nearest drawn level (Q12, saturating)
            if q11_d1 > 0 then
                if q11_d1 > 2047 then q12_e1 <= to_unsigned(2047, 12); else q12_e1 <= unsigned(q11_d1(11 downto 0)); end if;
            elsif q11_r1(11) = '1' then q12_e1 <= not q11_r1;
            else q12_e1 <= q11_r1; end if;
            if q11_d2 > 0 then
                if q11_d2 > 2047 then q12_e2 <= to_unsigned(2047, 12); else q12_e2 <= unsigned(q11_d2(11 downto 0)); end if;
            elsif q11_r2(11) = '1' then q12_e2 <= not q11_r2;
            else q12_e2 <= q11_r2; end if;
            -- 13: s = min(e1, e2)
            if q12_e1 < q12_e2 then q13_s <= q12_e1; else q13_s <= q12_e2; end if;
            -- 14: palette address: gel, ink, s * 512 (saturating at 255)
            v_q := resize(shift_right(resize(q13_s, 13) + 4, 3), 9);
            if v_q > 255 then v_q := to_unsigned(255, 9); end if;
            r_pa <= s_gel & s_ink & v_q(7 downto 0);
            -- 15: palette data (EBR); 16: fabric copy -> s_y17 at 17 with the gate
            -- 17: compose; neutral black outside the generated avid
            if s_gav_sr(15) = '0' then
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            else
                s_y17 <= shift_left(resize(q16_y, 10), 2); s_u17 <= shift_left(resize(q16_u, 10), 2); s_v17 <= shift_left(resize(q16_v, 10), 2);
            end if;
            s_bh <= s_bh(6 downto 0) & s_gav_sr(15);
        end if;
    end process p_pix;

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            q16_y <= r_py; q16_u <= r_pu; q16_v <= r_pv;
        end if;
    end process p_pal;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent running
    -- 8-sums that snap back to neutral once 8 blank samples are in), luma [1 2 1]
    -- centred on the same sample, luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_yf : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 18: running sums, luma window
            s_cu(0) <= s_u17; s_cv(0) <= s_v17;
            for i in 1 to 7 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            if s_bh = "00000000" then
                s_bu <= to_unsigned(4096, 13); s_bv <= to_unsigned(4096, 13);
            else
                s_bu <= s_bu + s_u17 - s_cu(7);
                s_bv <= s_bv + s_v17 - s_cv(7);
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_wy(0) <= s_y17;
            for i in 1 to 5 loop s_wy(i) <= s_wy(i - 1); end loop;
            -- 19: chroma = (S8(t) + S8(t - 1)) / 16, centred 4 samples back; luma [1 2 1] centred there too
            v_sc := resize(s_bu, 14) + s_bud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bv, 14) + s_bvd; s_v19 <= v_sc(13 downto 4);
            v_sy := resize(s_wy(3), 12) + shift_left(resize(s_wy(4), 12), 1) + resize(s_wy(5), 12);
            s_y19 <= v_sy(11 downto 2);
            -- 20: luma 13/16; blanking gate
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            v_yf := s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y20 <= v_yf; s_u20 <= s_u19; s_v20 <= s_v19;
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lantern;
