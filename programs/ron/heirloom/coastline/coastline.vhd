-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Coastline v0.1.0 -- Heirloom (video phase), from the clip WIN_20240117_22_55_32 at 0.5 s.
--
-- The still: a blue field cut by a curved coastline.  On the coast a rainbow ridge (cyan fringe, lime, yellow with a
-- thin white streak, salmon, orange, grey-pink, orange-red, olive, red, black) whose bands wave more and more the
-- further they lie from the coast; a gulf of blue; a second clean arc; deep blue offshore rising into soft cyan with
-- the same waves.  In the clip the waves travel down (a cycle every 2.6 s); the coast and the arc never move.
--
-- The patch, as fitted (coastline.py has every constant):
--   u      = (x - XI(y)) / (XA(y) - XI(y))       a curved ramp, 0 on the coast, 1 on the arc
--   k, f   = floor(u), frac(u)                   a sawtooth: its resets ARE the coast and the arc (clean edges)
--   m      = 1.015 + D sin(y/380 + p1) + 0.352 D sin(y/820 + p2)   a vertical oscillator pair into a VCA
--   w      = | f m + weave sin(x/240 + 1.6 sin(...)) |            the colouriser's input
--   colour = palette[cycle k][w]   (bay / ridge / offshore / far)
--
--   K1  Bend      the coast's curvature: bowed the other way .. straight .. the still (512) .. a hook
--   K2  Reach     the sawtooth's period: a ridge filling the frame .. the still .. a thin ridge with arcs behind it
--   K3  Waves     the wave count (+-3 octaves about the still's 2.8 per frame)
--   K4  Speed     bipolar: the waves travel down (+, 700 = the clip's 2.6 s per wave) or up
--   K5  Weave     a horizontal chevron oscillator into the colouriser (0 = the still; the sibling clip 22_57_37)
--   K6  Etch      how far the camera (or the Osc) bends the coast and every band
--   S7  Coast     Clean (the still: the coast and the arc never wave) / Ripple: wave 1 also moves the ramp
--   S8  Rainbows  only the ridge cycle is a rainbow (the still) / every sawtooth cycle is
--   S9  Mirror    one coast / two coasts mirrored about the centre (a canyon)
--   S10 Sunset    the still's colours / U and V swapped: an amber sea with a blue-violet ridge
--   S11 Source    Video / Osc
--   P12 SURF      the wave depth: 0 = a calm ridge of clean arcs .. the still .. storm surf, tongues to the arc
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line start
-- (computes the NEXT line: the coast and width by Horner on a serial multiplier, the field gain by a restoring
-- divider, the ramp's start value, the two wave sines and m), a beam DDA for u and the weave phase (reversing at the
-- centre for Mirror), a 25-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated
-- avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum resync, luma
-- 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.coastline_pkg.all;
use work.all;

architecture coastline of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    -- fitted constants (h = half virtual px; Q4 h; Q14 h per line; see coastline.py)
    constant C_CI0 : integer := 19795;     -- coast x at dy = 0, Q4 h
    constant C_CI1 : integer := 16254;     -- -coast slope, Q14 h per line (the coefficient is negative)
    constant C_CW0 : integer := 26448;     -- width at dy = 0, Q4 h
    constant C_CW1 : integer := 4942;      -- -width slope, Q14 h per line
    constant C_P1I : integer := 2029355;   -- wave 1 phase at power-up, Q24 cycles (0.76 rad)
    constant C_P2I : integer := 6355197;   -- wave 2 phase (2.38 rad)

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
    -- quarter-wave index: a 1024-step phase -> 256-entry table address
    function f_qidx(p : unsigned(9 downto 0)) return unsigned is
    begin
        if p(8) = '1' then return not p(7 downto 0); else return p(7 downto 0); end if;
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
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_ls_a, r_ws_a : unsigned(7 downto 0) := (others => '0');
    signal r_ls_q, r_ws_q : unsigned(9 downto 0) := (others => '0');
    signal r_pal_a : unsigned(9 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 320 x 270 virtual px, drifting, at 6x the camera's gain
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(204, 9);
    signal s_oys : unsigned(10 downto 0) := to_unsigned(242, 11);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(0, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(464 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_rip, s_rain, s_mirr, s_sun, s_src : std_logic := '0';

    -- per-field constants
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);     -- beam start x (h)
    signal c_xb2  : unsigned(11 downto 0) := to_unsigned(20, 12);     -- x of the third pixel (h)
    signal c_xbq  : unsigned(15 downto 0) := to_unsigned(256, 16);    -- beam start x, Q4 h
    signal c_hinc : unsigned(9 downto 0) := to_unsigned(273, 10);     -- weave phase per pixel
    signal c_h0   : unsigned(15 downto 0) := to_unsigned(2185, 16);   -- weave phase at the beam start
    signal z_bq   : signed(11 downto 0) := (others => '0');            -- bend, Q8 (signed)
    signal c_c2n  : std_logic := '0';                                  -- the curvature terms are negative
    signal c_ci2  : unsigned(15 downto 0) := to_unsigned(13764, 16);   -- |bend| * 13764 >> 8 (Q24 h per line^2)
    signal c_cw2  : unsigned(15 downto 0) := to_unsigned(3467, 16);
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_sh3  : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_rch  : unsigned(9 downto 0) := to_unsigned(256, 10);      -- reach, Q8
    signal c_f1   : unsigned(18 downto 0) := to_unsigned(44150, 19);   -- wave 1 cycles per virtual line, Q24
    signal c_f1s  : unsigned(20 downto 0) := to_unsigned(44150, 21);   -- ... per output line
    signal c_f2s  : unsigned(20 downto 0) := to_unsigned(20449, 21);   -- wave 2 per output line
    signal z_sq   : unsigned(19 downto 0) := (others => '0');          -- scratch: a knob squared
    signal c_v1   : unsigned(19 downto 0) := (others => '0');          -- wave 1 phase step per field, Q24
    signal c_ph1  : unsigned(23 downto 0) := to_unsigned(C_P1I, 24);
    signal c_ph2  : unsigned(23 downto 0) := to_unsigned(C_P2I, 24);
    signal c_d1   : unsigned(9 downto 0) := to_unsigned(180, 10);      -- wave depth, Q10
    signal c_d2   : unsigned(8 downto 0) := to_unsigned(63, 9);
    signal c_wc   : unsigned(8 downto 0) := (others => '0');           -- weave depth, Q10
    signal z_e    : unsigned(12 downto 0) := (others => '0');
    signal z_eo   : unsigned(14 downto 0) := (others => '0');
    signal c_e    : unsigned(14 downto 0) := (others => '0');          -- etch gain, Q20 cycles per luma step
    signal c_eoff : unsigned(25 downto 0) := (others => '0');          -- Osc centring, Q24 cycles

    -- field serial products (on the line engine's multiplier, which is idle in the zone)
    signal z_fa  : unsigned(21 downto 0) := (others => '0');
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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_p1, e_p2 : unsigned(23 downto 0) := (others => '0');
    signal e_ady  : unsigned(9 downto 0) := (others => '0');
    signal e_dyn  : std_logic := '0';
    signal e_q1, e_q2 : std_logic := '0';                 -- sine negative half
    signal e_s1, e_s2 : signed(10 downto 0) := (others => '0');
    signal e_dy2  : unsigned(18 downto 0) := (others => '0');
    signal e_xi   : signed(17 downto 0) := (others => '0');
    signal e_wd   : signed(17 downto 0) := (others => '0');
    signal e_dxs  : signed(17 downto 0) := (others => '0');
    signal d_r    : unsigned(16 downto 0) := (others => '0');
    signal d_d    : unsigned(15 downto 0) := to_unsigned(6400, 16);
    signal d_q    : unsigned(17 downto 0) := (others => '0');
    signal d_run  : std_logic := '0';
    signal e_us   : signed(29 downto 0) := (others => '0');
    signal e_m    : signed(13 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    -- next line (loaded at g_start)
    signal n_us   : signed(29 downto 0) := (others => '0');
    signal n_du   : unsigned(19 downto 0) := (others => '0');
    signal n_am   : unsigned(11 downto 0) := to_unsigned(1039, 12);
    signal n_sg   : std_logic := '0';
    signal n_h0   : unsigned(15 downto 0) := (others => '0');

    -- beam
    signal b_u   : signed(29 downto 0) := (others => '0');
    signal b_du  : unsigned(19 downto 0) := (others => '0');
    signal b_h   : unsigned(15 downto 0) := (others => '0');
    signal b_x2  : unsigned(11 downto 0) := (others => '0');
    signal b_fl  : std_logic := '0';
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_am  : unsigned(11 downto 0) := to_unsigned(1039, 12);
    signal l_sg  : std_logic := '0';

    -- pixel path (the number is the stage the register holds)
    type t_u18 is array (natural range <>) of signed(17 downto 0);
    signal q_u   : t_u18(2 to 8) := (others => (others => '0'));       -- u, Q12
    type t_h10 is array (natural range <>) of unsigned(9 downto 0);
    signal q_h   : t_h10(2 to 6) := (others => (others => '0'));       -- weave phase (top 10 bits)
    signal q7_hn, q8_hn, q9_hn : std_logic := '0';
    signal q4_l, q5_l : unsigned(7 downto 0) := (others => '0');
    signal q5_a, q6_a, q7_a : unsigned(8 downto 0) := (others => '0');
    signal q6_lb : unsigned(7 downto 0) := (others => '0');
    signal q7_el, q7_eh : unsigned(18 downto 0) := (others => '0');
    signal q8_et : unsigned(22 downto 0) := (others => '0');
    signal q9_u  : signed(17 downto 0) := (others => '0');
    signal q9_hm : unsigned(9 downto 0) := (others => '0');
    signal q10_fl, q10_fh : unsigned(16 downto 0) := (others => '0');
    signal q10_wl, q10_wh : unsigned(13 downto 0) := (others => '0');
    signal q10_z, q11_z, q12_z, q13_z : unsigned(1 downto 0) := (others => '0');
    signal q10_wn, q11_wn : std_logic := '0';
    signal q11_fm : unsigned(21 downto 0) := (others => '0');
    signal q11_wv : unsigned(18 downto 0) := (others => '0');
    signal q12_w  : signed(23 downto 0) := (others => '0');
    signal q13_aw : unsigned(22 downto 0) := (others => '0');
    signal q14_a, q15_a : unsigned(9 downto 0) := (others => '0');
    signal p18_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cau, s_cav, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_ly : t_u10(0 to 11) := (others => to_unsigned(64, 10));   -- luma delay (aligns [1 2 1] with the chroma centre)
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
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
            r_ex_q  <= C_CL_EXP2(to_integer(r_ex_a));
            r_ls_q  <= C_CL_SIN(to_integer(r_ls_a));
            r_ws_q  <= C_CL_SIN(to_integer(r_ws_a));
            r_pal_q <= C_CL_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_d  : unsigned(9 downto 0);
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
            elsif s_zc /= 1023 then
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
                s_rip  <= registers_in(6)(0);
                s_rain <= registers_in(6)(1);
                s_mirr <= registers_in(6)(2);
                s_sun  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4 =>
                        c_xb0 <= to_unsigned(512, 12); c_xb2 <= to_unsigned(520, 12); c_xbq <= to_unsigned(8192, 16);
                        c_hinc <= to_unsigned(546, 10); c_h0 <= to_unsigned(4369, 16); s_oxs <= to_unsigned(408, 9);
                    when 3 =>
                        c_xb0 <= to_unsigned(24, 12); c_xb2 <= to_unsigned(30, 12); c_xbq <= to_unsigned(384, 16);
                        c_hinc <= to_unsigned(410, 10); c_h0 <= to_unsigned(3277, 16); s_oxs <= to_unsigned(306, 9);
                    when others =>
                        c_xb0 <= to_unsigned(16, 12); c_xb2 <= to_unsigned(20, 12); c_xbq <= to_unsigned(256, 16);
                        c_hinc <= to_unsigned(273, 10); c_h0 <= to_unsigned(2185, 16); s_oxs <= to_unsigned(204, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(242, 8), 8), 11);

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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
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
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(unsigned(abs(z_bq)), 22);   z_fb <= to_unsigned(13764, 16);   -- coast curvature
                    when 1  => z_fa <= resize(unsigned(abs(z_bq)), 22);   z_fb <= to_unsigned(3467, 16);    -- width curvature
                    when 2  => z_fa <= to_unsigned(44150, 22);            z_fb <= z_mt;                     -- wave 1 frequency
                    when 3  => z_fa <= resize(c_f1, 22);                  z_fb <= resize(s_ys, 16);         -- per output line
                    when 4  => z_fa <= resize(c_f1s, 22);                 z_fb <= to_unsigned(15177, 16);   -- wave 2 = x 0.4632
                    when 5  => z_fa <= resize(s_vm, 22);                  z_fb <= resize(s_vm, 16);         -- speed^2
                    when 6  => z_fa <= resize(z_sq, 22);                 z_fb <= to_unsigned(13700, 16);   -- phase step
                    when 7  => z_fa <= resize(s_gp(12 downto 3), 22);     z_fb <= resize(s_gp(12 downto 3), 16);  -- surf^2
                    when 8  => z_fa <= resize(z_sq, 22);                 z_fb <= to_unsigned(676, 16);     -- depth
                    when 9  => z_fa <= resize(c_d1, 22);                  z_fb <= to_unsigned(11534, 16);   -- depth 2 = x 0.352
                    when 10 => z_fa <= resize(s_g5(12 downto 3), 22);     z_fb <= resize(s_g5(12 downto 3), 16);  -- weave^2
                    when 11 => z_fa <= resize(z_sq, 22);                z_fb <= to_unsigned(462, 16);
                    when 12 => z_fa <= resize(s_g6(12 downto 3), 22);     z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 13 => z_fa <= resize(z_sq, 22);                z_fb <= to_unsigned(275, 16);
                    when others => z_fa <= resize(z_eo, 22);              z_fb <= to_unsigned(111, 16);     -- Osc centring
                end case;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps): the waves travel down for Speed > 0
                    when 20 =>
                        if s_vneg = '1' then
                            c_ph1 <= c_ph1 + resize(c_v1, 24); c_ph2 <= c_ph2 + resize(c_v1(19 downto 1), 24);
                        else
                            c_ph1 <= c_ph1 - resize(c_v1, 24); c_ph2 <= c_ph2 - resize(c_v1(19 downto 1), 24);
                        end if;
                    -- bend b (Q8): -1.5 .. 1 over 0 .. 512, 1 .. 4 over 512 .. 1023; reach exponent
                    when 21 =>
                        v_k := s_g1(12 downto 3);
                        if v_k <= 512 then
                            z_bq <= signed(resize(v_k, 12)) + signed(resize(v_k(9 downto 2), 12)) - 384;
                        else
                            v_d := v_k - 512;
                            z_bq <= to_signed(256, 12) + signed(resize(v_d, 12)) + signed(resize(v_d(9 downto 1), 12));
                        end if;
                        v_e := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 22 =>
                        if z_bq > 0 then c_c2n <= '1'; else c_c2n <= '0'; end if;
                    when 23 => z_mt <= r_ex_q;
                    when 24 =>                       -- reach = mt >> (7 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_rch <= resize(z_mt(15 downto 9), 10);
                            when -1     => c_rch <= resize(z_mt(15 downto 8), 10);
                            when 0      => c_rch <= resize(z_mt(15 downto 7), 10);
                            when others => c_rch <= z_mt(15 downto 6);
                        end case;
                        -- waves exponent = 1.5 (k3 - 512): +-3 octaves
                        v_e := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        v_e := v_e + shift_right(v_e, 1);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh3 <= v_e(10 downto 8);
                    when 26 => z_mt <= r_ex_q;
                    when 50  => c_ci2 <= m_acc(23 downto 8);
                    when 74  => c_cw2 <= m_acc(23 downto 8);
                    when 98  =>                      -- F1 = 44150 mt >> (15 - sh), sh = -3 .. 2
                        case to_integer(z_sh3) is
                            when -3     => c_f1 <= resize(m_acc(35 downto 18), 19);
                            when -2     => c_f1 <= resize(m_acc(35 downto 17), 19);
                            when -1     => c_f1 <= m_acc(34 downto 16);
                            when 0      => c_f1 <= m_acc(33 downto 15);
                            when 1      => c_f1 <= m_acc(32 downto 14);
                            when others => c_f1 <= m_acc(31 downto 13);
                        end case;
                    when 122 => c_f1s <= m_acc(28 downto 8);
                    when 146 => c_f2s <= m_acc(35 downto 15);
                    when 170 => z_sq  <= m_acc(19 downto 0);
                    when 194 => c_v1  <= m_acc(31 downto 12);
                    when 218 => z_sq  <= m_acc(19 downto 0);
                    when 242 => c_d1  <= m_acc(29 downto 20) + 41;          -- a calm swell (0.04) stays at Surf 0
                    when 266 => c_d2  <= m_acc(23 downto 15);
                    when 290 => z_sq  <= m_acc(19 downto 0);
                    when 314 => c_wc  <= m_acc(28 downto 20);
                    when 338 => z_sq  <= m_acc(19 downto 0);
                    when 362 => z_e   <= m_acc(28 downto 16);
                    when 363 =>
                        z_eo <= shift_left(resize(z_e, 15), 2) + shift_left(resize(z_e, 15), 1);   -- the Osc at 6x the camera's gain
                    when 364 =>
                        if s_src = '1' then c_e <= z_eo; else c_e <= resize(z_e, 15); end if;
                    when 386 =>
                        if s_src = '1' then c_eoff <= m_acc(21 downto 0) & "0000"; else c_eoff <= (others => '0'); end if;
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
    -- 0), computes the NEXT line on one serial multiplier (16 iterations, result
    -- at load + 17) and a restoring divider (18 iterations):
    --   XI = CI0 - CI1 dy - b CI2 dy^2, WD = CW0 - CW1 dy - b CW2 dy^2 (Q4 h), G = reach 2^20 / WD (Q24 per h)
    --   u0 = (xb0 - XI) G - eoff, du = G xs
    --   m  = 1039 + D1 s1 + D2 s2 (Q10), the weave's line phase = h0 + 16 s1
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(19 downto 0);
        variable v_t  : signed(17 downto 0);
        variable v_r  : unsigned(16 downto 0);
        variable v_u  : signed(29 downto 0);
        variable v_m  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 420 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_p1 <= c_ph1; e_p2 <= c_ph2;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_p1 <= e_p1 + resize(c_f1s, 24);
                e_p2 <= e_p2 + resize(c_f2s, 24);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- restoring divider: d_q = (reach << 20) / WD, the dividend's low 18 bits are zero
            if d_run = '1' then
                v_r := d_r(15 downto 0) & '0';
                if v_r >= resize(d_d, 17) then
                    d_r <= v_r - resize(d_d, 17); d_q <= d_q(16 downto 0) & '1';
                else
                    d_r <= v_r; d_q <= d_q(16 downto 0) & '0';
                end if;
            end if;

            -- the per-field sequencer's products (zone only: the line engine is idle then)
            if z_ld = '1' then
                m_a <= resize(z_fa, 36); m_b <= z_fb; m_acc <= (others => '0');
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    v_d := signed(resize(e_vy(18 downto 8), 20)) - 540;
                    if v_d < 0 then v_d := -v_d; e_dyn <= '1'; else e_dyn <= '0'; end if;
                    e_ady <= unsigned(v_d(9 downto 0));
                    r_ls_a <= f_qidx(e_p1(23 downto 14)); e_q1 <= e_p1(23);
                when 2 =>
                    m_a <= resize(e_ady, 36); m_b <= resize(e_ady, 16); m_acc <= (others => '0');
                when 3 =>
                    e_s1 <= signed(resize(r_ls_q, 11));
                    r_ls_a <= f_qidx(e_p2(23 downto 14)); e_q2 <= e_p2(23);
                when 4 =>
                    if e_q1 = '1' then e_s1 <= -e_s1; end if;
                when 5 =>
                    e_s2 <= signed(resize(r_ls_q, 11));
                when 6 =>
                    if e_q2 = '1' then e_s2 <= -e_s2; end if;
                when 19 =>
                    e_dy2 <= m_acc(18 downto 0);
                    m_a <= to_unsigned(C_CI1, 36); m_b <= resize(e_ady, 16); m_acc <= (others => '0');
                when 36 =>                    -- XI = CI0 - CI1 dy (the coefficient is negative)
                    if e_dyn = '1' then e_xi <= to_signed(C_CI0, 18) + signed(resize(m_acc(23 downto 10), 18));
                    else e_xi <= to_signed(C_CI0, 18) - signed(resize(m_acc(23 downto 10), 18)); end if;
                    m_a <= to_unsigned(C_CW1, 36); m_b <= resize(e_ady, 16); m_acc <= (others => '0');
                when 53 =>                    -- WD = CW0 - CW1 dy
                    if e_dyn = '1' then e_wd <= to_signed(C_CW0, 18) + signed(resize(m_acc(23 downto 10), 18));
                    else e_wd <= to_signed(C_CW0, 18) - signed(resize(m_acc(23 downto 10), 18)); end if;
                    m_a <= resize(e_dy2, 36); m_b <= c_ci2; m_acc <= (others => '0');
                when 70 =>                    -- XI -= b CI2 dy^2
                    if c_c2n = '1' then e_xi <= e_xi - signed(resize(m_acc(35 downto 20), 18));
                    else e_xi <= e_xi + signed(resize(m_acc(35 downto 20), 18)); end if;
                    m_a <= resize(e_dy2, 36); m_b <= c_cw2; m_acc <= (others => '0');
                when 87 =>                    -- WD -= b CW2 dy^2
                    if c_c2n = '1' then e_wd <= e_wd - signed(resize(m_acc(35 downto 20), 18));
                    else e_wd <= e_wd + signed(resize(m_acc(35 downto 20), 18)); end if;
                when 89 =>
                    if e_wd < 6400 then d_d <= to_unsigned(6400, 16);
                    elsif e_wd > 65535 then d_d <= to_unsigned(65535, 16);
                    else d_d <= unsigned(e_wd(15 downto 0)); end if;
                    e_dxs <= signed(resize(c_xbq, 18)) - e_xi;
                when 90 =>
                    d_r <= resize(c_rch & "00", 17); d_q <= (others => '0'); d_run <= '1';
                when 108 =>
                    d_run <= '0';
                when 109 =>
                    v_t := abs(e_dxs);
                    m_a <= resize(d_q, 36); m_b <= unsigned(v_t(15 downto 0)); m_acc <= (others => '0');
                    case to_integer(s_xs) is
                        when 4      => n_du <= resize(d_q & "00", 20);
                        when 3      => n_du <= resize(d_q & '0', 20) + resize(d_q, 20);
                        when others => n_du <= resize(d_q & '0', 20);
                    end case;
                when 126 =>
                    v_u := signed(m_acc(33 downto 4));
                    if e_dxs < 0 then v_u := -v_u; end if;
                    e_us <= v_u;
                    m_a <= resize(c_d1, 36); m_b <= resize(unsigned(abs(e_s1)), 16); m_acc <= (others => '0');
                when 127 =>
                    e_us <= e_us - signed(resize(c_eoff, 30));
                when 143 =>                   -- m = 1039 + D1 s1 >> 10
                    if e_s1 < 0 then e_m <= to_signed(1039, 14) - signed(resize(m_acc(20 downto 10), 14));
                    else e_m <= to_signed(1039, 14) + signed(resize(m_acc(20 downto 10), 14)); end if;
                    m_a <= resize(c_d2, 36); m_b <= resize(unsigned(abs(e_s2)), 16); m_acc <= (others => '0');
                when 160 =>                   -- + D2 s2 >> 10
                    if e_s2 < 0 then e_m <= e_m - signed(resize(m_acc(20 downto 10), 14));
                    else e_m <= e_m + signed(resize(m_acc(20 downto 10), 14)); end if;
                when 161 =>
                    -- Ripple: wave 1 also moves the ramp itself (the coast and the arc snake), 0.1875 cycle
                    if s_rip = '1' then
                        e_us <= e_us + shift_left(resize(e_s1, 30), 11) + shift_left(resize(e_s1, 30), 10);
                    end if;
                when 162 =>
                    v_m := abs(e_m);
                    if v_m > 4095 then n_am <= (others => '1'); else n_am <= unsigned(v_m(11 downto 0)); end if;
                    n_sg <= e_m(13);
                    n_us <= e_us;
                    n_h0 <= c_h0 + unsigned(resize(shift_left(resize(e_s1, 16), 4), 16));
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: u (Q24 cycles) and the weave phase as DDAs; with Mirror they turn
    -- back once x passes the centre (3840 h / 2).  The turn flag for the next
    -- step is registered a clock ahead.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_u  <= n_us;
                b_du <= n_du;
                b_h  <= n_h0;
                b_x2 <= c_xb2;
                b_fl <= '0';
                b_px <= (others => '0');
                l_am <= n_am; l_sg <= n_sg;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_fl = '1' then
                    b_u <= b_u - signed(resize(b_du, 30));
                    b_h <= b_h - resize(c_hinc, 16);
                else
                    b_u <= b_u + signed(resize(b_du, 30));
                    b_h <= b_h + resize(c_hinc, 16);
                end if;
                if s_mirr = '1' and b_x2 > 1920 then b_fl <= '1'; else b_fl <= '0'; end if;
                b_x2 <= b_x2 + s_xs;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ui : signed(5 downto 0);
        variable v_ix : unsigned(8 downto 0);
        variable v_a  : unsigned(9 downto 0);
        variable v_w  : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            -- u (Q12) and the weave phase ride along
            q_u(2) <= b_u(29 downto 12);
            for i in 3 to 8 loop q_u(i) <= q_u(i - 1); end loop;
            q_h(2) <= b_h(15 downto 6);
            for i in 3 to 6 loop q_h(i) <= q_h(i - 1); end loop;
            -- 3: line buffer data; 4: fabric copy; 5-6: 4-px box (the camera bends rather than shreds)
            q4_l <= unsigned(s_lb_q);
            q5_l <= q4_l;
            q5_a <= resize(q4_l, 9) + q5_l;                     -- L(n) + L(n-1) at stage 5
            q6_a <= q5_a; q7_a <= q6_a;
            q6_lb <= resize(shift_right(resize(q5_a, 10) + q7_a, 2), 8);   -- (L(n) .. L(n-3)) / 4 at stage 6
            -- 7: etch partials (luma nibbles x gain); 8: etch (Q20)
            q7_el <= resize(q6_lb(3 downto 0) * c_e, 19);
            q7_eh <= resize(q6_lb(7 downto 4) * c_e, 19);
            q8_et <= resize(q7_el, 23) + shift_left(resize(q7_eh, 23), 4);
            -- 9: u' = u + etch (Q12)
            q9_u <= q_u(8) + signed(resize(q8_et(22 downto 8), 18));
            -- weave: 7 ROM address (stage-6 phase), 8 ROM data, 9 fabric copy
            r_ws_a <= f_qidx(q_h(6)); q7_hn <= q_h(6)(9);
            q8_hn <= q7_hn;
            q9_hm <= r_ws_q; q9_hn <= q8_hn;
            -- 10: cycle -> zone; f x |m| partials; weave partials
            v_ui := q9_u(17 downto 12);
            if s_rain = '1' or v_ui = 0 then q10_z <= "00";          -- ridge
            elsif v_ui < 0 then q10_z <= "10";                        -- bay
            elsif v_ui = 1 then q10_z <= "01";                        -- offshore
            else q10_z <= "11"; end if;                               -- far
            q10_fl <= resize(unsigned(q9_u(6 downto 2)) * l_am, 17);
            q10_fh <= resize(unsigned(q9_u(11 downto 7)) * l_am, 17);
            q10_wl <= resize(q9_hm(4 downto 0) * c_wc, 14);
            q10_wh <= resize(q9_hm(9 downto 5) * c_wc, 14);
            q10_wn <= q9_hn xor l_sg;
            -- 11: f |m| (Q20), weave (Q20)
            q11_fm <= resize(q10_fl, 22) + shift_left(resize(q10_fh, 22), 5);
            q11_wv <= resize(q10_wl, 19) + shift_left(resize(q10_wh, 19), 5);
            q11_wn <= q10_wn; q11_z <= q10_z;
            -- 12: w = f |m| +- weave
            if q11_wn = '1' then q12_w <= signed(resize(q11_fm, 24)) - signed(resize(q11_wv, 24));
            else q12_w <= signed(resize(q11_fm, 24)) + signed(resize(q11_wv, 24)); end if;
            q12_z <= q11_z;
            -- 13: |w|
            v_w := abs(q12_w);
            q13_aw <= unsigned(v_w(22 downto 0));
            q13_z <= q12_z;
            -- 14: palette address (index saturates at w = 2)
            if q13_aw(22 downto 21) /= "00" then v_ix := (others => '1'); else v_ix := q13_aw(20 downto 12); end if;
            case q13_z is
                when "00"   => v_a := '0' & v_ix;
                when "01"   => v_a := "10" & v_ix(8 downto 1);
                when "10"   => v_a := "110" & v_ix(8 downto 2);
                when others => v_a := "111" & v_ix(8 downto 2);
            end case;
            q14_a <= v_a;
            -- 15: address copy; 16: ROM address; 17: ROM data (EBR); 18: fabric copy
            q15_a <= q14_a;
            r_pal_a <= q15_a;
            p18_q <= r_pal_q;
            -- 19: compose (Sunset swaps U and V); black outside the generated avid
            if s_gav_sr(16) = '0' then
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            elsif s_sun = '1' then
                s_y19 <= unsigned(p18_q(31 downto 22)); s_u19 <= unsigned(p18_q(11 downto 2)); s_v19 <= unsigned(p18_q(21 downto 12));
            else
                s_y19 <= unsigned(p18_q(31 downto 22)); s_u19 <= unsigned(p18_q(21 downto 12)); s_v19 <= unsigned(p18_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums), Y through [1 2 1]/4 on the same centre (the
    -- HD HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_yl : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- chroma box A: stage 20 sums, stage 21 output
            s_cau(0) <= s_u19; s_cav(0) <= s_v19;
            for i in 1 to 7 loop s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bau <= s_bau + s_u19 - s_cau(7);
                s_bav <= s_bav + s_v19 - s_cav(7);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- chroma box B: stage 22 sums, stage 23 output (centred 8 compose samples back)
            s_cbu(0) <= s_u21; s_cbv(0) <= s_v21;
            for i in 1 to 7 loop s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bbu <= to_unsigned(512 * 8, 13); s_bbv <= to_unsigned(512 * 8, 13);
            else
                s_bbu <= s_bbu + s_u21 - s_cbu(7);
                s_bbv <= s_bbv + s_v21 - s_cbv(7);
            end if;
            s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bbu, 14) + s_bbud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v23 <= v_sc(13 downto 4);
            -- luma [1 2 1] / 4 on the same centre: s_ly(k) holds the compose luma of the pixel k - 2 ahead
            s_ly(0) <= s_y19;
            for i in 1 to 11 loop s_ly(i) <= s_ly(i - 1); end loop;
            v_yl := resize(s_ly(9), 12) + shift_left(resize(s_ly(10), 12), 1) + s_ly(11);
            s_y23 <= v_yl(11 downto 2);
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
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture coastline;
