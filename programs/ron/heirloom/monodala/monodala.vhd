-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- 2026-11-15: K6 camera top +25 % (Ron)
--
-- Monodala v0.2.0 -- one colour, topographic rings (from Heirloom Mandala).
--
-- Mandala with its red key at -100 reads as a blue field crossed by light green
-- contour lines that bend around the incoming video.  Monodala is that effect on
-- its own, over the whole screen: concentric contour rings round the screen
-- centre, pushed in and out by the video luma (so they trace the picture like a
-- topographic map), drawn in an accent colour over a ground of ONE solid colour.
-- No face: the field is the exact distance from the centre (a pipelined 8-step
-- CORDIC), round at SD, 720p and 1080.  The line colour is a mix toward the
-- accent by the contour pulse, so lines stay their own colour over any ground.
--
--   K1  Hue      the ground colour, the full colour wheel (default blue)
--   K2  Accent   the line colour, as a hue offset from the ground (default -120 deg: green on blue)
--   K3  Width    line width: washed-out bands .. hair-thin lines (16 banks, default the thinnest)
--   K4  Melt     how far the video (or Osc) bends the rings (Mandala's curve, up to +-3.8 rings)
--   K5  Flow     ring drift, bipolar (centre +-8 = still): + rings stream outward
--   K6  Level    the ground's brightness: black .. full colour (default deep blue); Video ground: brightness
--   S7  Lines    One colour / Spectrum: every ring its own hue (45 deg steps from the accent)
--   S8  Ink      Bright / Dark lines (the accent at 3/8 level)
--   S9  Ground   Solid / Video: the ground is the incoming picture in the ground colour
--   S10 Centres  One / Two: a second fold at +-480 virtual px, two ring systems side by side
--   S11 Source   Video / Osc (what bends the rings)
--   P12 RINGS    ring density: 0 = x0.125 (a few broad rings, 320 px apart), 50 % = 40 px, 100 % = x3.6 moire
--
-- Colour: GT, a 256-entry ground table (level x ground colour, rebuilt by accumulation every
-- vblank), and the line colour from a 512-entry hue ROM (ink & hue); out = G + p (C - G),
-- p = the contour pulse.  Virtual 1080-line geometry.  Generated avid; output conditioning
-- (luma box + [1 2 1], chroma box pair, luma to ~777) before the blanking gate.  Latency 28
-- clocks in every mode, colour at stage 21 (pixel depth parity = Mandala's).  Colours in the
-- hardware convention (U = Cr).

-- K6 camera pass 2026-11-15: K6 = Melt (camera strength, was K4), K4 = Level (was K6); default K6 50 %.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.monodala_pkg.all;
use work.all;

architecture monodala of program_top is
    constant C_LAT : integer := 28;   -- matches processing_delay_clks
    constant C_D8  : unsigned(7 downto 0) := to_unsigned(69, 8);   -- breathe depth (Mandala K6 = 276)
    -- ring units: 512 / (40 px x CORDIC gain 1.64674) = 7.7729 per virtual px, so the CORDIC's output is
    -- the distance in Q9 rings (40 virtual px each at Rings x1)
    constant C_Y0   : integer := 1074528;   -- 540 virtual px, Q8 units
    constant C_FOLD : integer := 3731;      -- 480 virtual px (Centres: Two)

    -- per-timing-ID steps, Q8 ring units per pixel / per line (virtual 1080-line coordinates)
    function f_sx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(3980, 12);
            when 5 | 6 | 14     => return to_unsigned(2985, 12);
            when others         => return to_unsigned(1990, 12);
        end case;
    end function;
    function f_sy(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(8846, 14);
            when 4          => return to_unsigned(4477, 14);
            when 8          => return to_unsigned(7462, 14);
            when 12         => return to_unsigned(3731, 14);
            when 5 | 6 | 14 => return to_unsigned(2985, 14);
            when 1 | 2 | 10 => return to_unsigned(3979, 14);
            when others     => return to_unsigned(1990, 14);
        end case;
    end function;
    -- Osc spatial steps (Q16 cycles per pixel / per line)
    function f_kax(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(89, 9);
            when 5 | 6 | 14     => return to_unsigned(67, 9);
            when others         => return to_unsigned(44, 9);
        end case;
    end function;
    function f_kay(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(270, 9);
            when 4          => return to_unsigned(137, 9);
            when 8          => return to_unsigned(228, 9);
            when 12         => return to_unsigned(114, 9);
            when 5 | 6 | 14 => return to_unsigned(91, 9);
            when 1 | 2 | 10 => return to_unsigned(121, 9);
            when others     => return to_unsigned(61, 9);
        end case;
    end function;
    function f_isb8(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 | 5 | 6 | 14 => return '1';
            when others => return '0';
        end case;
    end function;
    -- 8-bit triangle -> signed -127..127
    function f_tri(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;

    -- timing
    signal t_sx : unsigned(11 downto 0) := to_unsigned(1990, 12);
    signal t_sy : unsigned(13 downto 0) := to_unsigned(1990, 14);
    signal t_kax, t_kay : unsigned(8 downto 0) := (others => '0');
    signal t_b8 : std_logic := '0';   -- 720p / SD (the short box)

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K1, K2, K3, K4, K6, P12 (Q.3), power up at the defaults
    type t_g is array(0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(684 * 8, 13), to_unsigned(686 * 8, 13), to_unsigned(1023 * 8, 13),
                         to_unsigned(824 * 8, 13), to_unsigned(800 * 8, 13), to_unsigned(494 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- serial multiplier (per-field products)
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 5) := (others => '0');
    signal m_a  : unsigned(19 downto 0) := (others => '0');
    signal m_b  : unsigned(17 downto 0) := (others => '0');
    signal m_h  : unsigned(19 downto 0) := (others => '0');
    signal m_l  : unsigned(17 downto 0) := (others => '0');

    -- per-field values
    signal f_hue, f_acc, f_lhue : unsigned(7 downto 0) := (others => '0');
    signal f_wl    : unsigned(3 downto 0) := to_unsigned(15, 4);
    signal f_melt0 : unsigned(8 downto 0) := to_unsigned(100, 9);
    signal f_melt  : unsigned(9 downto 0) := to_unsigned(125, 10); -- f_melt0 x1.25 (K6 top +25 %)
    signal f_lev   : unsigned(7 downto 0) := to_unsigned(200, 8);  -- Level: the ground level (Solid)
    signal f_sv    : unsigned(9 downto 0) := to_unsigned(74, 10);  -- Video ground: black level
    signal f_p8    : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal f_gain  : unsigned(9 downto 0) := to_unsigned(256, 10); -- Q8
    signal f_whc   : unsigned(10 downto 0) := to_unsigned(953, 11);
    signal f_u0    : signed(23 downto 0) := (others => '0');      -- Q8 ring units
    signal f_fm    : unsigned(8 downto 0) := (others => '0');     -- flow magnitude
    signal f_fneg  : std_logic := '0';
    signal f_spd   : unsigned(6 downto 0) := (others => '0');     -- Q9 per field
    -- flow phase, Q9 rings mod 256 rings; the +1/2 ring start puts the ring index boundary half way
    -- between two lines (the pulse ROM gets the phase back by inverting bit 8)
    signal f_flow  : unsigned(16 downto 0) := to_unsigned(256, 17);
    signal f_lph   : unsigned(15 downto 0) := (others => '0');    -- breathe LFO phase
    signal f_la    : unsigned(15 downto 0) := (others => '0');
    signal f_lneg  : std_logic := '0';
    signal f_s9    : unsigned(8 downto 0) := (others => '0');     -- |sin| 0..256
    signal f_off   : signed(11 downto 0) := (others => '0');      -- Q9 breathe offset
    signal f_ta, f_tb : unsigned(15 downto 0) := (others => '0');
    signal f_spec, f_dark, f_vgnd, f_two, f_osc : std_logic := '0';

    -- ground table GT[L] = L x ground colour / 256 (Y - 64, U, V), rebuilt in every vblank zone
    type t_gt is array (0 to 255) of std_logic_vector(29 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    signal gt_we  : std_logic := '0';
    signal gt_wa, r_gta : unsigned(7 downto 0) := (others => '0');
    signal gt_wd  : std_logic_vector(29 downto 0) := (others => '0');
    signal gt_q   : std_logic_vector(29 downto 0) := (others => '0');
    signal c_y    : unsigned(9 downto 0) := (others => '0');
    signal c_u, c_v : signed(9 downto 0) := (others => '0');
    signal a_y    : unsigned(17 downto 0) := (others => '0');
    signal a_u, a_v : signed(17 downto 0) := (others => '0');
    -- hue ROM (line colour; the ground colour in the vblank zone)
    signal r_ha   : unsigned(8 downto 0) := (others => '0');
    signal h_q    : std_logic_vector(29 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of gt_ram : signal is true;

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- beam: x from the screen centre (Q8 ring units, per pixel), y from the centre (per line), Osc phases
    signal b_u  : signed(23 downto 0) := (others => '0');
    signal l_y  : signed(23 downto 0) := to_signed(-C_Y0, 24);
    signal l_ay : signed(15 downto 0) := (others => '0');
    signal b_pa : unsigned(15 downto 0) := (others => '0');
    signal l_pb : unsigned(15 downto 0) := (others => '0');

    -- video ring: rv_q2 holds the input luma of the pixel now in stage 7 (read address s_wp - 4: a delay of
    -- 4 + 3 clocks), rv_d(4) that of the pixel in stage 11
    type t_vring is array (0 to 255) of std_logic_vector(7 downto 0);
    signal rv_ram : t_vring := (others => (others => '0'));
    attribute no_rw_check of rv_ram : signal is true;
    signal rv_ra : unsigned(7 downto 0) := (others => '0');
    signal rv_q, rv_q2 : std_logic_vector(7 downto 0) := (others => '0');
    type t_u8v is array (natural range <>) of unsigned(7 downto 0);
    signal rv_d : t_u8v(1 to 4) := (others => (others => '0'));

    -- pixel path (stage N = N registers after the beam)
    signal q1_ax : unsigned(14 downto 0) := (others => '0');
    signal q1_osc, q8_src : signed(9 downto 0) := (others => '0');
    -- CORDIC (vectoring, first quadrant): stage 2 + i holds iteration i's input; cx(8) = K r, Q9 rings
    type t_s16a is array (natural range <>) of signed(15 downto 0);
    signal cx, cy : t_s16a(0 to 8) := (others => (others => '0'));
    signal q9_m  : signed(18 downto 0) := (others => '0');
    signal q10_om : signed(12 downto 0) := (others => '0');
    signal q11_f  : signed(15 downto 0) := (others => '0');
    signal q12_gh, q12_gl : signed(21 downto 0) := (others => '0');
    signal q12_v19 : unsigned(12 downto 0) := (others => '0');
    signal q13_fg : signed(26 downto 0) := (others => '0');
    signal q13_vl : signed(10 downto 0) := (others => '0');
    signal q14_ix : unsigned(2 downto 0) := (others => '0');
    signal r_ga : unsigned(10 downto 0) := (others => '0');
    signal r_gq : std_logic_vector(7 downto 0) := (others => '0');
    signal q16_pu : unsigned(7 downto 0) := (others => '0');
    signal q16_gy, q17_gy, q18_gy : unsigned(9 downto 0) := (others => '0');
    signal q16_gu, q16_gv, q17_gu, q17_gv, q18_gu, q18_gv : signed(9 downto 0) := (others => '0');
    signal q17_p : unsigned(7 downto 0) := (others => '0');
    signal q17_cy : unsigned(9 downto 0) := (others => '0');
    signal q17_cu, q17_cv : signed(9 downto 0) := (others => '0');
    signal q18_p : unsigned(8 downto 0) := (others => '0');
    signal q18_dy, q18_du, q18_dv : signed(10 downto 0) := (others => '0');
    signal q19_hy, q19_hu, q19_hv : signed(16 downto 0) := (others => '0');
    signal q19_ly, q19_lu, q19_lv : signed(15 downto 0) := (others => '0');
    signal q19_oy, q19_ou, q19_ov, q20_oy, q20_ou, q20_ov : signed(11 downto 0) := (others => '0');
    signal q20_my, q20_mu, q20_mv : signed(20 downto 0) := (others => '0');
    signal q21_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q21_u, q21_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- box ring (luma box + chroma box pair over the same 8-tap (4 at 720p/SD) window) and sync ring
    type t_ring is array (0 to 255) of std_logic_vector(29 downto 0);
    signal rb_ram : t_ring := (others => "0000000000" & "1000000000" & "1000000000");
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal rs_ram : t_sring := (others => "111");
    attribute no_rw_check of rb_ram : signal is true;
    attribute no_rw_check of rs_ram : signal is true;
    signal s_wp, rb_ra, rs_ra : unsigned(7 downto 0) := (others => '0');
    signal rb_wd : std_logic_vector(29 downto 0) := "0000000000" & "1000000000" & "1000000000";
    signal rb_q, rb_q2 : std_logic_vector(29 downto 0) := "0000000000" & "1000000000" & "1000000000";
    signal rs_q, rs_q2 : std_logic_vector(2 downto 0) := "111";
    signal s_ysum, s_usum, s_vsum, s_ud, s_vd : unsigned(13 downto 0) := (others => '0');
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 ring inputs
    signal q23_y, q24_y, q25_y, q25_yf : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q24_u, q24_v, q25_u, q25_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal q27_y, q28_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q26_u, q26_v, q27_u, q27_v, q28_u, q28_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs and the ground table
    ---------------------------------------------------------------------------
    p_prom : process(clk)
    begin
        if rising_edge(clk) then
            r_gq <= C_GP(to_integer(r_ga));
            h_q  <= C_HUE(to_integer(r_ha));
        end if;
    end process p_prom;

    p_gt : process(clk)
    begin
        if rising_edge(clk) then
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
            gt_q <= gt_ram(to_integer(r_gta));
        end if;
    end process p_gt;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field products (one serial multiplier),
    -- the ground table
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(20 downto 0);
        variable v_pr : unsigned(37 downto 0);
        variable v_g3 : unsigned(18 downto 0);
        variable v_md : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            t_sx  <= f_sx(registers_in(8)(3 downto 0));
            t_sy  <= f_sy(registers_in(8)(3 downto 0));
            t_kax <= f_kax(registers_in(8)(3 downto 0));
            t_kay <= f_kay(registers_in(8)(3 downto 0));
            t_b8  <= f_isb8(registers_in(8)(3 downto 0));

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
            g_wm1 <= s_w - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K5 Flow: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(4)), 11)) - 512;
                if v_x > 8 then f_fneg <= '0'; v_u := unsigned(v_x) - 8; f_fm <= v_u(8 downto 0);
                elsif v_x < -8 then f_fneg <= '1'; v_u := unsigned(-v_x) - 8; f_fm <= v_u(8 downto 0);
                else f_fneg <= '0'; f_fm <= (others => '0'); end if;
                f_spec <= registers_in(6)(0);
                f_dark <= registers_in(6)(1);
                f_vgnd <= registers_in(6)(2);
                f_two  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- phases: + flow streams the rings outward (phase down)
                if f_fneg = '1' then f_flow <= f_flow + resize(f_spd, 17);
                else f_flow <= f_flow - resize(f_spd, 17); end if;
                f_lph <= f_lph + 72;
                f_ta <= f_ta + 151; f_tb <= f_tb - 113;
            end if;

            -- glides: ease 1/8 of the gap per field (deadband 4); one channel per four zone clocks (4..27)
            if s_zone = '1' and s_zc >= 4 and s_zc < 28 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(5)) & "000"; gl_o <= s_g(3);
                            when 4 => gl_k <= unsigned(registers_in(3)) & "000"; gl_o <= s_g(4);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(5);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when 4 => s_g(4) <= gl_n;
                                when others => s_g(5) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- knob curves (settle long before the zone uses them)
            f_hue  <= s_g(0)(12 downto 5);                                     -- 256 hues, 171 = blue
            f_acc  <= s_g(1)(12 downto 5);
            f_lhue <= f_hue + f_acc;
            f_wl   <= s_g(2)(12 downto 9);                                     -- 16 banks, 15 = hair-thin
            -- Melt: Mandala's curve, the v0.5 line up to 100, then 2.5x steeper (x1.9 at the top)
            if s_g(3)(12 downto 5) <= 100 then
                f_melt0 <= resize(s_g(3)(12 downto 5), 9);
            else
                v_md := shift_left(resize(s_g(3)(12 downto 5), 11), 1) + resize(s_g(3)(12 downto 6), 11);
                f_melt0 <= resize(v_md - 150, 9);
            end if;
            f_melt <= resize(f_melt0, 10) + resize(f_melt0(8 downto 2), 10);   -- x1.25: 0 .. 608
            -- Level: Solid ground level 0..255; Video ground black level 19 + (255 - level)
            f_lev <= s_g(4)(12 downto 5);
            f_sv  <= to_unsigned(274, 10) - resize(f_lev, 10);
            f_p8 <= s_g(5)(12 downto 5);
            if t_b8 = '1' then f_whc <= s_w(11 downto 1) - 1; else f_whc <= s_w(11 downto 1) - 3; end if;
            f_la <= f_lph;
            if f_lph(15) = '1' then f_la <= (not f_lph) + 1; f_lneg <= '1'; else f_lneg <= '0'; end if;

            -- serial products: slot p = zc/32 (1..5), phase 0 load, 1..18 step, 20 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 5 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 5 and v_ph >= 1 and v_ph <= 18 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 5 loop
                if s_zone = '1' and v_p = k and v_ph = 20 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 1 => m_a <= resize(f_p8, 20);  m_b <= resize(f_p8, 18);
                    when 2 => m_a <= resize(f_whc, 20); m_b <= resize(t_sx, 18);
                    when 3 => m_a <= resize(f_fm, 20);  m_b <= resize(f_fm, 18);
                    when 4 => m_a <= resize(f_la, 20);  m_b <= resize(to_unsigned(32768, 17) - resize(f_la, 17), 18);
                    when others => m_a <= resize(C_D8, 20); m_b <= resize(f_s9, 18);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(17 downto 1);
                m_b <= '0' & m_b(17 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1) = '1' then
                v_g3 := shift_left(resize(v_pr(15 downto 0), 19), 3) - resize(v_pr(15 downto 0), 19);   -- 7 p^2
                f_gain <= to_unsigned(32, 10) + resize(v_g3(18 downto 9), 10);                        -- 0.125 + 3.5 p^2
            end if;
            if q_wr(2) = '1' then f_u0 <= -signed(resize(v_pr(22 downto 0), 24)); end if;   -- -(W/2) x step
            if q_wr(3) = '1' then f_spd <= v_pr(17 downto 11); end if;
            if q_wr(4) = '1' then
                if v_pr(29 downto 20) > 256 then f_s9 <= to_unsigned(256, 9); else f_s9 <= v_pr(28 downto 20); end if;
            end if;
            if q_wr(5) = '1' then
                if f_lneg = '1' then f_off <= -signed(resize(v_pr(16 downto 6), 12));
                else f_off <= signed(resize(v_pr(16 downto 6), 12)); end if;
            end if;

            -- ground table: the hue ROM is addressed with the ground hue for the whole zone (p_pix);
            -- latch it at zone clock 240, then GT[L] = L x colour / 256 for L = zone clock - 256 (256..511),
            -- one entry per clock by accumulation
            if s_zone = '1' and s_zc = 240 then
                c_y <= unsigned(h_q(29 downto 20)); c_u <= signed(h_q(19 downto 10)); c_v <= signed(h_q(9 downto 0));
            end if;
            gt_we <= '0';
            if s_zone = '1' and s_zc = 255 then
                a_y <= (others => '0'); a_u <= (others => '0'); a_v <= (others => '0');
            elsif s_zone = '1' and s_zc(9 downto 8) = "01" then
                gt_we <= '1'; gt_wa <= s_zc(7 downto 0);
                gt_wd <= std_logic_vector(a_y(17 downto 8)) & std_logic_vector(a_u(17 downto 8))
                         & std_logic_vector(a_v(17 downto 8));
                a_y <= a_y + resize(c_y, 18);
                a_u <= a_u + resize(c_u, 18);
                a_v <= a_v + resize(c_v, 18);
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
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
    -- Beam: x and y from the screen centre in Q8 ring units (y per line, at the
    -- source avid rise; reset each field), Osc phases
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                l_pb <= f_tb;
                l_y  <= to_signed(-C_Y0, 24);
            elsif data_in.avid = '1' and s_avq = '0' then
                l_pb <= l_pb + resize(t_kay, 16);
                l_y  <= l_y + signed(resize(t_sy, 24));
            end if;
            if l_y < 0 then l_ay <= resize(-l_y(23 downto 8), 16); else l_ay <= resize(l_y(23 downto 8), 16); end if;
            if g_start = '1' then
                b_u <= f_u0;
                b_pa <= f_ta;
            elsif g_avid = '1' then
                b_u <= b_u + signed(resize(t_sx, 24));
                b_pa <= b_pa + resize(t_kax, 16);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Rings (block RAM): the box window {Y, U, V}, the sync delay, the video
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rb_ram(to_integer(s_wp)) <= rb_wd;
            rb_q <= rb_ram(to_integer(rb_ra));
            rb_q2 <= rb_q;
        end if;
    end process p_ring;

    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            rs_ram(to_integer(s_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            rs_q <= rs_ram(to_integer(rs_ra));
            rs_q2 <= rs_q;
        end if;
    end process p_sring;

    p_vring : process(clk)
    begin
        if rising_edge(clk) then
            rv_ram(to_integer(s_wp)) <= data_in.y(9 downto 2);
            rv_q <= rv_ram(to_integer(rv_ra));
            rv_q2 <= rv_q;
        end if;
    end process p_vring;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a   : signed(15 downto 0);
        variable v_h   : unsigned(16 downto 0);
        variable v_v8  : unsigned(12 downto 0);
        variable v_o   : signed(12 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_m   : signed(18 downto 0);
        variable v_iy, v_iu, v_iv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: |x| (ring units), Osc (smooth: used at stage 8 without re-aligning)
            if b_u < 0 then q1_ax <= unsigned(resize(-b_u(23 downto 8), 15)); else q1_ax <= unsigned(resize(b_u(23 downto 8), 15)); end if;
            q1_osc <= resize(f_tri(b_pa), 10) + resize(f_tri(l_pb), 10);
            -- 2: Centres (Two: a second fold at +-480 virtual px); |y| of the line
            if f_two = '1' then
                v_a := signed(resize(q1_ax, 16)) - C_FOLD;
                if v_a < 0 then cx(0) <= -v_a; else cx(0) <= v_a; end if;
            else
                cx(0) <= signed(resize(q1_ax, 16));
            end if;
            cy(0) <= l_ay;
            -- 3..10: CORDIC vectoring, 8 iterations: rotate (x, y) onto the x axis, x -> K |(x, y)|
            for i in 0 to 7 loop
                if cy(i) >= 0 then
                    cx(i + 1) <= cx(i) + shift_right(cy(i), i);
                    cy(i + 1) <= cy(i) - shift_right(cx(i), i);
                else
                    cx(i + 1) <= cx(i) - shift_right(cy(i), i);
                    cy(i + 1) <= cy(i) + shift_right(cx(i), i);
                end if;
            end loop;

            -- 8: the bend source: the aligned luma as (2 Y8 - 256), or the Osc
            if f_osc = '1' then q8_src <= q1_osc; else q8_src <= signed(shift_left(resize(unsigned(rv_q2), 10), 1)) - 256; end if;
            rv_d(1) <= unsigned(rv_q2);
            for k in 2 to 4 loop rv_d(k) <= rv_d(k - 1); end loop;
            -- 9: the melt term
            q9_m <= resize(q8_src * signed('0' & f_melt), 19);
            -- 10: offset = breathe + melt
            v_m := resize(f_off, 19) + resize(shift_right(q9_m, 6), 19);
            q10_om <= resize(v_m, 13);
            -- 11: the field: distance (Q9 rings) + offset
            q11_f <= cx(8) + resize(q10_om, 16);
            -- 12: x Rings (gain split 5 | 5 bits); Video ground: the aligned luma x 19
            q12_gh <= resize(q11_f * signed('0' & f_gain(9 downto 5)), 22);
            q12_gl <= resize(q11_f * signed('0' & f_gain(4 downto 0)), 22);
            v_v8 := resize(rv_d(4), 13);
            q12_v19 <= shift_left(v_v8, 4) + shift_left(v_v8, 1) + v_v8;
            -- 13: Fg; Video level = luma x 19 / 16 - black level
            q13_fg <= shift_left(resize(q12_gh, 27), 5) + resize(q12_gl, 27);
            q13_vl <= signed(resize(q12_v19(12 downto 4), 11)) - signed(resize(f_sv, 11));
            -- 14: ring phase + flow (Q9 mod 256 rings, +1/2 ring): pulse ROM address (bit 8 back), the ring
            --     index (Spectrum); ground level L -> the GT address
            v_h := unsigned(q13_fg(24 downto 8)) + f_flow;
            r_ga <= f_wl & (not v_h(8)) & v_h(7 downto 2);
            q14_ix <= v_h(11 downto 9);
            if f_vgnd = '1' then
                if q13_vl < 0 then r_gta <= (others => '0'); elsif q13_vl > 255 then r_gta <= (others => '1');
                else r_gta <= unsigned(q13_vl(7 downto 0)); end if;
            else
                r_gta <= f_lev;
            end if;
            -- 15: pulse ROM, GT out (block RAM); line colour address (the ground hue in the vblank zone,
            --     for the ground-table builder)
            if s_zone = '1' then
                r_ha <= '0' & f_hue;
            elsif f_spec = '1' then
                r_ha <= f_dark & (f_lhue(7 downto 5) + q14_ix) & f_lhue(4 downto 0);
            else
                r_ha <= f_dark & f_lhue;
            end if;
            -- 16: fabric copies; hue ROM out (block RAM)
            q16_pu <= unsigned(r_gq);
            q16_gy <= unsigned(gt_q(29 downto 20)); q16_gu <= signed(gt_q(19 downto 10)); q16_gv <= signed(gt_q(9 downto 0));
            -- 17: p; line colour fabric copy
            q17_p <= q16_pu;
            q17_cy <= unsigned(h_q(29 downto 20)); q17_cu <= signed(h_q(19 downto 10)); q17_cv <= signed(h_q(9 downto 0));
            q17_gy <= q16_gy; q17_gu <= q16_gu; q17_gv <= q16_gv;
            -- 18: line - ground; p to 0..256
            q18_dy <= signed(resize(q17_cy, 11)) - signed(resize(q17_gy, 11));
            q18_du <= resize(q17_cu, 11) - resize(q17_gu, 11);
            q18_dv <= resize(q17_cv, 11) - resize(q17_gv, 11);
            q18_p <= resize(q17_p, 9) + resize(q17_p(7 downto 7), 9);
            q18_gy <= q17_gy; q18_gu <= q17_gu; q18_gv <= q17_gv;
            -- 19: p x (line - ground), p split 5 | 4 bits; the ground with its offsets
            q19_hy <= q18_dy * signed('0' & q18_p(8 downto 4));
            q19_ly <= q18_dy * signed('0' & q18_p(3 downto 0));
            q19_hu <= q18_du * signed('0' & q18_p(8 downto 4));
            q19_lu <= q18_du * signed('0' & q18_p(3 downto 0));
            q19_hv <= q18_dv * signed('0' & q18_p(8 downto 4));
            q19_lv <= q18_dv * signed('0' & q18_p(3 downto 0));
            q19_oy <= signed(resize(q18_gy, 12)) + 64;
            q19_ou <= resize(q18_gu, 12) + 512;
            q19_ov <= resize(q18_gv, 12) + 512;
            -- 20: product sums
            q20_my <= shift_left(resize(q19_hy, 21), 4) + resize(q19_ly, 21);
            q20_mu <= shift_left(resize(q19_hu, 21), 4) + resize(q19_lu, 21);
            q20_mv <= shift_left(resize(q19_hv, 21), 4) + resize(q19_lv, 21);
            q20_oy <= q19_oy; q20_ou <= q19_ou; q20_ov <= q19_ov;
            -- 21: Y, U, V = ground + p (line - ground) / 256 (a mix of two in-range colours: no clamp)
            v_o := resize(q20_oy, 13) + resize(shift_right(q20_my, 8), 13);
            q21_y <= unsigned(v_o(9 downto 0));
            v_o := resize(q20_ou, 13) + resize(shift_right(q20_mu, 8), 13);
            q21_u <= unsigned(v_o(9 downto 0));
            v_o := resize(q20_ov, 13) + resize(shift_right(q20_mv, 8), 13);
            q21_v <= unsigned(v_o(9 downto 0));

            -- 22: box sums over the ring window: 8 taps at 1080, 4 at 720p / SD (soft, band-limited lines;
            -- luma box then [1 2 1], chroma box pair).  The ring's input is forced to black/neutral off the
            -- generated avid, so the sums resync to 4 or 8 x neutral in every blanking interval.
            s_wp <= s_wp + 1;
            if t_b8 = '1' then rb_ra <= s_wp - 1; else rb_ra <= s_wp - 5; end if;
            rs_ra <= s_wp - (C_LAT - 3);
            rv_ra <= s_wp - 4;
            if s_gav_sr(21) = '0' then
                v_iy := to_unsigned(64, 10); v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10);
            else
                v_iy := q21_y; v_iu := q21_u; v_iv := q21_v;
            end if;
            rb_wd <= std_logic_vector(v_iy) & std_logic_vector(v_iu) & std_logic_vector(v_iv);
            s_bxh <= s_bxh(6 downto 0) & s_gav_sr(21);
            if s_bxh = "00000000" then
                if t_b8 = '1' then
                    s_ysum <= to_unsigned(256, 14); s_usum <= to_unsigned(2048, 14); s_vsum <= to_unsigned(2048, 14);
                else
                    s_ysum <= to_unsigned(512, 14); s_usum <= to_unsigned(4096, 14); s_vsum <= to_unsigned(4096, 14);
                end if;
            else
                s_ysum <= s_ysum + unsigned(rb_wd(29 downto 20)) - unsigned(rb_q2(29 downto 20));
                s_usum <= s_usum + unsigned(rb_wd(19 downto 10)) - unsigned(rb_q2(19 downto 10));
                s_vsum <= s_vsum + unsigned(rb_wd(9 downto 0)) - unsigned(rb_q2(9 downto 0));
            end if;
            -- 23..24: luma box out; chroma box pair
            if t_b8 = '1' then q23_y <= s_ysum(11 downto 2); else q23_y <= s_ysum(12 downto 3); end if;
            s_ud <= s_usum; s_vd <= s_vsum;
            v_sc := resize(s_usum, 15) + s_ud;
            if t_b8 = '1' then q24_u <= v_sc(12 downto 3); else q24_u <= v_sc(13 downto 4); end if;
            v_sc := resize(s_vsum, 15) + s_vd;
            if t_b8 = '1' then q24_v <= v_sc(12 downto 3); else q24_v <= v_sc(13 downto 4); end if;
            -- 24..25: luma [1 2 1]
            q24_y <= q23_y; q25_y <= q24_y;
            v_sy := resize(q23_y, 12) + shift_left(resize(q24_y, 12), 1) + resize(q25_y, 12);
            q25_yf <= v_sy(11 downto 2);
            q25_u <= q24_u; q25_v <= q24_v;

            q26_u <= q25_u; q26_v <= q25_v;

            -- 27: luma toward black (white 940 -> 777); blanking gate.  (Stages: ring input 22, box
            -- sums 23, box out 24, [1 2 1] centre 26, chroma pair 24 -> 26.)
            if q25_yf > 64 then v_dl := q25_yf - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(26) = '1' then
                q27_y <= q25_yf - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                q27_u <= q26_u; q27_v <= q26_v;
            else
                q27_y <= to_unsigned(64, 10); q27_u <= to_unsigned(512, 10); q27_v <= to_unsigned(512, 10);
            end if;
            -- 28
            q28_y <= q27_y; q28_u <= q27_u; q28_v <= q27_v;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(q28_y);
    data_out.u       <= std_logic_vector(q28_u);
    data_out.v       <= std_logic_vector(q28_v);
    data_out.hsync_n <= rs_q2(2);
    data_out.vsync_n <= rs_q2(1);
    data_out.field_n <= rs_q2(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture monodala;
