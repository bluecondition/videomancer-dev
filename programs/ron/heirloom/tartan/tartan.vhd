-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tartan v0.5.0 -- Heirloom #25, rebuilt 2026-10-27 from the photo WIN_20230410_01_13_24.
--
-- The photo: a big A-frame of nested diagonal stripes (apex at x 984, legs 0.9 lines per px:
-- "/" on the left, "\" on the right) cut by horizontal bands that alternate between a bright
-- colourizer (cyan / green / orange / magenta) and a dark one (black with cyan and green
-- stripes, a yellow fringe on every edge into black).  The bands are a frequency-modulated
-- vertical square wave: ~90-135 lines at the top and bottom, ~26 in the middle.  The left
-- family's stripes run past the apex and curl into hooks; a dotted sync glitch marks the top
-- of every dark band on the left.
--
-- The patch, as fitted (tartan.py has the tables):
--   u      = g ( +-(x - xc) - s y ) + scroll + curl + Bend * luma      the A-frame coordinate
--   phase  = roll + b y + c sin(2 pi (y - y0) / 729)                   FM row oscillator
--   band   = frac(phase) < duty ? bright : dark
--   colour = BAR[side][u][band]                                          two measured colourizers
--   fringe = the last 7 px before a dark-band edge into black -> yellow
--
--   K1  Stripes  colourizer gain: the nested A-frames (0.35 .. 3x, 1 = the photo)
--   K2  Slope    A-frame leg slope (0.2 .. 2.5 lines per px, 0.9 = the photo)
--   K3  Rows     row-oscillator frequency (3 .. 40 bands per frame, 12.3 = the photo)
--   K4  Squeeze  FM depth: even bands .. bands thin in the middle (the photo) .. folding
--   K5  Speed    bipolar: the A-frames climb, the bands roll, the squeeze drifts
--   K6  Bend     the picture (or the Osc lattice) bends the stripes
--   S7  Fringe   the yellow fringes into black (On)
--   S8  Mirror   the left family on both sides: a seamless symmetric A
--   S9  Tear     the dotted sync glitch and a line slip at the top of each dark band (On)
--   S10 Palette  Photo / Night
--   S11 Source   Video / Osc
--   P12 SHADE    the dark-band duty: 0 = all bright, 50 % = the photo, 100 % = all dark
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), per-line engine kicked
-- at each line start (computes the NEXT line: V, the FM phase with a quarter-sine ROM, the band
-- type, the tear), beam DDA (A = g (x - xc)), 32-stage pixel path.  Virtual 1080-line
-- coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (Y/U/V
-- [1 2 .. 2 1]/32 at 1080, /16 at 720p and SD; luma 13/16) before the blanking gate.  Colours in
-- the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tartan_pkg.all;
use work.all;

architecture tartan of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

    constant C_XC2    : integer := 1969;       -- apex, half virtual px
    constant C_SPLW   : integer := 2079;       -- left family reaches xc + 55 px (bright bands)
    constant C_SPLD   : integer := 2119;       -- and xc + 75 px (dark bands)
    constant C_ROLL0  : integer := 239476;     -- row phase at line 0, Q20
    constant C_THOFF0 : integer := 41687;      -- modulator phase offset (y0 = 463), Q16
    constant C_CORN   : integer := 71680;      -- corner: bright bands starting above line 280 (Q8)

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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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

    -- ROMs (registered address and data; a fabric copy follows each)
    signal r_bar_a : unsigned(11 downto 0) := (others => '0');
    signal r_bar_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_sin_a : unsigned(7 downto 0) := (others => '0');
    signal r_sin_q : std_logic_vector(15 downto 0) := (others => '0');

    -- oscillator source (Source = Osc)
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxi : unsigned(15 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(192, 10);
    signal s_oys : unsigned(11 downto 0) := to_unsigned(320, 12);
    signal s_otl : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_fw   : unsigned(2 downto 0) := to_unsigned(7, 3);      -- fringe width, px
    signal t_hd   : std_logic := '1';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_fc   : unsigned(10 downto 0) := (others => '0');

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(507 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(564 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(513 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(498 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(341 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_k5 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(9 downto 0) := (others => '0');
    signal s_vneg, s_vstill : std_logic := '0';
    signal s_fr, s_mir, s_tear, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal c_sq    : unsigned(19 downto 0) := (others => '0');
    signal c_g8    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_s8    : unsigned(9 downto 0) := to_unsigned(230, 10);
    signal c_gs    : unsigned(10 downto 0) := to_unsigned(230, 11);
    signal c_a0    : signed(22 downto 0) := (others => '0');
    signal c_ast   : unsigned(11 downto 0) := to_unsigned(512, 12);
    signal c_b20   : unsigned(15 downto 0) := to_unsigned(11888, 16);
    signal c_c20   : unsigned(20 downto 0) := to_unsigned(969108, 21);
    signal c_cb    : unsigned(9 downto 0) := (others => '0');
    signal c_sp    : unsigned(19 downto 0) := (others => '0');
    signal c_rs    : unsigned(11 downto 0) := (others => '0');
    signal c_rr    : unsigned(15 downto 0) := (others => '0');
    signal c_rd    : unsigned(8 downto 0) := (others => '0');
    signal c_duty  : unsigned(15 downto 0) := to_unsigned(32704, 16);
    signal c_allw, c_alld : std_logic := '0';
    signal c_seed  : unsigned(15 downto 0) := x"ACE1";
    signal s_scr   : unsigned(20 downto 0) := (others => '0');                -- scroll, Q9 barcode px
    signal s_roll  : unsigned(19 downto 0) := to_unsigned(C_ROLL0, 20);       -- row phase offset, Q20
    signal s_thoff : unsigned(15 downto 0) := to_unsigned(C_THOFF0, 16);      -- modulator offset, Q16

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');

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
    signal e_t90  : unsigned(25 downto 0) := (others => '0');
    signal e_th   : unsigned(15 downto 0) := (others => '0');
    signal e_sneg : std_logic := '0';
    signal e_sin  : unsigned(15 downto 0) := (others => '0');
    signal e_vg   : unsigned(20 downto 0) := (others => '0');
    signal e_pb   : unsigned(19 downto 0) := (others => '0');
    signal e_pr   : unsigned(19 downto 0) := (others => '0');
    signal e_phi  : unsigned(19 downto 0) := (others => '0');
    signal e_w, e_pw : std_logic := '1';
    signal e_ds   : unsigned(18 downto 0) := (others => '0');
    signal e_dl   : unsigned(18 downto 0) := (others => '0');
    signal e_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal e_del  : signed(6 downto 0) := (others => '0');
    signal e_dot, e_slip, e_sl1 : std_logic := '0';
    signal e_cor  : std_logic := '1';
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_w, n_dot, n_lowr : std_logic := '1';
    signal n_l9   : unsigned(20 downto 0) := (others => '0');

    -- beam
    signal b_x2  : unsigned(11 downto 0) := (others => '0');
    signal b_a9  : signed(22 downto 0) := (others => '0');
    signal l_w, l_dot, l_lowr : std_logic := '1';
    signal l_l9  : unsigned(20 downto 0) := (others => '0');

    -- video / osc bend tap (source timing, a few px early)
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal v_y8  : t_u8(0 to 8) := (others => (others => '0'));
    signal v_box : unsigned(10 downto 0) := (others => '0');
    signal v_lb  : unsigned(7 downto 0) := (others => '0');

    -- pixel path
    signal p1_sl, p1_dn : std_logic := '0';
    signal p1_a9 : signed(22 downto 0) := (others => '0');
    signal p1_d2 : signed(12 downto 0) := (others => '0');
    signal p1_ls : signed(8 downto 0) := (others => '0');
    signal p1_dot : std_logic := '0';
    signal p2_sel : unsigned(20 downto 0) := (others => '0');
    signal p2_dd  : unsigned(7 downto 0) := (others => '0');
    signal p2_q0, p2_q1 : unsigned(13 downto 0) := (others => '0');
    signal p2_bn  : std_logic := '0';
    signal p3_u   : unsigned(20 downto 0) := (others => '0');
    signal p3_cu  : unsigned(15 downto 0) := (others => '0');
    signal p3_b   : unsigned(17 downto 0) := (others => '0');
    signal p3_bn  : std_logic := '0';
    signal p4_u   : unsigned(20 downto 0) := (others => '0');
    signal p4_b   : unsigned(20 downto 0) := (others => '0');
    signal p5_u   : unsigned(20 downto 0) := (others => '0');
    type t_sl is array (natural range <>) of std_logic;
    signal p_sl   : t_sl(2 to 8) := (others => '0');
    signal p_dot  : t_sl(2 to 16) := (others => '0');
    signal p8_rom : std_logic_vector(7 downto 0) := (others => '0');
    signal p9_c   : unsigned(2 downto 0) := (others => '0');
    signal p9_k   : std_logic := '0';
    type t_c3 is array (natural range <>) of unsigned(2 downto 0);
    signal p_c    : t_c3(10 to 16) := (others => (others => '0'));
    signal p_k    : t_sl(10 to 16) := (others => '0');
    signal p17_c  : unsigned(3 downto 0) := (others => '0');
    signal x18_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal x18_u, x18_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cy : t_u10(0 to 15) := (others => to_unsigned(64, 10));
    signal s_cu, s_cv : t_u10(0 to 15) := (others => to_unsigned(512, 10));
    signal s_by16, s_by16d : unsigned(13 downto 0) := to_unsigned(1024, 14);
    signal s_bu16, s_bu16d, s_bv16, s_bv16d : unsigned(13 downto 0) := to_unsigned(8192, 14);
    signal s_by8, s_by8d : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bu8, s_bu8d, s_bv8, s_bv8d : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_oy16, s_ou16, s_ov16 : unsigned(14 downto 0) := (others => '0');
    signal s_oy8, s_ou8, s_ov8 : unsigned(13 downto 0) := (others => '0');
    signal s_y29 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y30, s_y31, s_y32 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u30, s_v30, s_u31, s_v31, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_bar_q <= C_TT_BAR(to_integer(r_bar_a));
            r_sin_q <= C_TT_SIN(to_integer(r_sin_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_fs : unsigned(15 downto 0);
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
            s_k5 <= unsigned(registers_in(4));
            v_s := signed(resize(s_k5, 11)) - 512;
            if v_s > -9 and v_s < 9 then s_vstill <= '1'; else s_vstill <= '0'; end if;
            s_vneg <= v_s(10);
            if v_s(10) = '1' then v_m := unsigned(not v_s(9 downto 0)) - 7; else v_m := unsigned(v_s(9 downto 0)) - 8; end if;
            s_vm <= v_m;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_fr   <= registers_in(6)(0);
                s_mir  <= registers_in(6)(1);
                s_tear <= registers_in(6)(2);
                s_pal  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_fc   <= s_fc + 1;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_fw <= to_unsigned(4, 3); s_oxs <= to_unsigned(384, 10); s_oxi <= to_unsigned(46080, 16);
                    when 3      => s_x0 <= (others => '0');     s_fw <= to_unsigned(5, 3); s_oxs <= to_unsigned(288, 10); s_oxi <= (others => '0');
                    when others => s_x0 <= (others => '0');     s_fw <= to_unsigned(7, 3); s_oxs <= to_unsigned(192, 10); s_oxi <= (others => '0');
                end case;
                case to_integer(unsigned(s_tid)) is
                    when 0 | 4 | 5 | 6 | 8 | 12 | 14 => t_hd <= '0';
                    when others => t_hd <= '1';
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 14) + shift_left(resize(s_ys, 14), 2), 2), 12);

            -- shared glide (1/8 of the gap per field, +-2 deadband): K1 K2 K3 K4 K6 P12
            if s_zone = '1' and s_zc >= 1 and s_zc <= 18 then
                case to_integer(s_zc) is
                    when 1  => s_gm <= s_g1; s_gd <= signed(resize(unsigned(registers_in(0)) & "000", 15)) - signed(resize(s_g1, 15));
                    when 4  => s_gm <= s_g2; s_gd <= signed(resize(unsigned(registers_in(1)) & "000", 15)) - signed(resize(s_g2, 15));
                    when 7  => s_gm <= s_g3; s_gd <= signed(resize(unsigned(registers_in(2)) & "000", 15)) - signed(resize(s_g3, 15));
                    when 10 => s_gm <= s_g4; s_gd <= signed(resize(unsigned(registers_in(3)) & "000", 15)) - signed(resize(s_g4, 15));
                    when 13 => s_gm <= s_g6; s_gd <= signed(resize(unsigned(registers_in(5)) & "000", 15)) - signed(resize(s_g6, 15));
                    when 16 => s_gm <= s_gp; s_gd <= signed(resize(unsigned(registers_in(7)) & "000", 15)) - signed(resize(s_gp, 15));
                    when 2 | 5 | 8 | 11 | 14 | 17 =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                    when 3  => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 6  => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 9  => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 12 => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 15 => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                end case;
            end if;

            -- motion (last field's rates), duty, LFSR seed
            case to_integer(s_zc) is
                when 20 =>
                    if s_vneg = '1' then
                        s_scr <= s_scr - resize(c_rs, 21); s_roll <= s_roll - resize(c_rr, 20); s_thoff <= s_thoff - resize(c_rd, 16);
                    else
                        s_scr <= s_scr + resize(c_rs, 21); s_roll <= s_roll + resize(c_rr, 20); s_thoff <= s_thoff + resize(c_rd, 16);
                    end if;
                when 21 =>
                    c_duty <= (to_unsigned(1023, 10) - s_gp(12 downto 3)) & "000000";
                    if s_gp(12 downto 3) < 4 then c_allw <= '1'; else c_allw <= '0'; end if;
                    if s_gp(12 downto 3) > 1019 then c_alld <= '1'; else c_alld <= '0'; end if;
                    v_fs := x"ACE1" xor shift_left(resize(s_fc(10 downto 5), 16), 3) xor shift_left(resize(s_fc(10 downto 5), 16), 9);
                    c_seed <= v_fs;
                when others => null;
            end case;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations,
            -- result stable from load + 17 (stores at load + 18)
            if s_zone = '1' and (s_zc = 32 or s_zc = 56 or s_zc = 80 or s_zc = 104 or s_zc = 128 or s_zc = 152 or
               s_zc = 176 or s_zc = 200 or s_zc = 224 or s_zc = 248 or s_zc = 272 or s_zc = 296 or s_zc = 320 or
               s_zc = 344 or s_zc = 368 or s_zc = 392) then
                f_acc <= (others => '0');
                case to_integer(s_zc) is
                    when 32  => f_a <= resize(s_g1(12 downto 3), 36); f_b <= resize(s_g1(12 downto 3), 16);
                    when 56  => f_a <= resize(c_sq, 36);              f_b <= to_unsigned(678, 16);
                    when 80  => f_a <= resize(s_g2(12 downto 3), 36); f_b <= resize(s_g2(12 downto 3), 16);
                    when 104 => f_a <= resize(c_sq, 36);              f_b <= to_unsigned(589, 16);
                    when 128 => f_a <= resize(c_g8, 36);              f_b <= resize(c_s8, 16);
                    when 152 => f_a <= resize(c_g8, 36);              f_b <= to_unsigned(C_XC2, 16) - resize(s_x0, 16);
                    when 176 => f_a <= resize(c_g8, 36);              f_b <= resize(s_xs, 16);
                    when 200 => f_a <= resize(s_g3(12 downto 3), 36); f_b <= resize(s_g3(12 downto 3), 16);
                    when 224 => f_a <= resize(c_sq, 36);              f_b <= to_unsigned(35924, 16);
                    when 248 => f_a <= resize(s_g4(12 downto 3), 36); f_b <= to_unsigned(1946, 16);
                    when 272 => f_a <= resize(s_g6(12 downto 3), 36); f_b <= resize(s_g6(12 downto 3), 16);
                    when 296 => f_a <= resize(c_sq, 36);              f_b <= to_unsigned(842, 16);
                    when 320 => f_a <= resize(s_vm, 36);              f_b <= resize(s_vm, 16);
                    when 344 => f_a <= resize(c_sp, 36);              f_b <= to_unsigned(795, 16);
                    when 368 => f_a <= resize(c_sp, 36);              f_b <= to_unsigned(5433, 16);
                    when others => f_a <= resize(c_sp, 36);           f_b <= to_unsigned(93, 16);
                end case;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 50  => c_sq  <= f_acc(19 downto 0);
                    when 74  => c_g8  <= to_unsigned(90, 10) + resize(f_acc(29 downto 20), 10);
                    when 98  => c_sq  <= f_acc(19 downto 0);
                    when 122 => c_s8  <= to_unsigned(51, 10) + resize(f_acc(29 downto 20), 10);
                    when 146 => c_gs  <= f_acc(18 downto 8);
                    when 170 => c_a0  <= -signed(resize(f_acc(21 downto 0), 23));
                    when 194 => c_ast <= f_acc(11 downto 0);
                    when 218 => c_sq  <= f_acc(19 downto 0);
                    when 242 => c_b20 <= to_unsigned(2913, 16) + resize(f_acc(35 downto 20), 16);
                    when 266 => c_c20 <= f_acc(20 downto 0);
                    when 290 => c_sq  <= f_acc(19 downto 0);
                    when 314 => c_cb  <= f_acc(29 downto 20);
                    when 338 => c_sp  <= f_acc(19 downto 0);
                    when 362 => if s_vstill = '1' then c_rs <= (others => '0'); else c_rs <= f_acc(27 downto 16); end if;
                    when 386 => if s_vstill = '1' then c_rr <= (others => '0'); else c_rr <= f_acc(31 downto 16); end if;
                    when 410 => if s_vstill = '1' then c_rd <= (others => '0'); else c_rd <= f_acc(24 downto 16); end if;
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
    -- Source side: oscillator lattice (341 x 205 virtual px, drifting, 0 .. 127)
    -- and the bend tap (unipolar luma >> 2 - 16, 8-px box), a few px early
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_o : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= s_oxi + shift_left(resize(s_ot, 16), 8) + shift_left(resize(s_ot, 16), 9);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= to_unsigned(0, 16) - shift_left(resize(s_ot, 16), 8) - shift_left(resize(s_ot, 16), 9); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_o := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_otl <= '0' & v_o(7 downto 1);
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then v_y8(0) <= s_otl;
            elsif v_l <= 16 then v_y8(0) <= (others => '0');
            elsif v_l > 235 then v_y8(0) <= to_unsigned(219, 8);
            else v_y8(0) <= v_l - 16; end if;
            for i in 1 to 8 loop v_y8(i) <= v_y8(i - 1); end loop;
            v_box <= v_box + v_y8(0) - v_y8(8);
            v_lb <= v_box(10 downto 3);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier: V g, the FM row
    -- phase (quarter-sine ROM), the band type, the tear
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ph : unsigned(19 downto 0);
        variable v_x  : unsigned(15 downto 0);
        variable v_l  : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_pw <= '1';
                e_cor <= '1';
                e_lfsr <= c_seed;
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
                    m_a <= resize(e_vy, 36); m_b <= resize(c_gs, 16); m_acc <= (others => '0');
                    e_t90 <= shift_left(resize(e_vy, 26), 6) + shift_left(resize(e_vy, 26), 4)
                             + shift_left(resize(e_vy, 26), 3) + shift_left(resize(e_vy, 26), 1);
                when 2 =>
                    e_th <= e_t90(23 downto 8) - s_thoff;
                when 3 =>
                    if e_th(14) = '1' then r_sin_a <= not e_th(13 downto 6); else r_sin_a <= e_th(13 downto 6); end if;
                    e_sneg <= e_th(15);
                when 6 =>
                    e_sin <= unsigned(r_sin_q);
                when 18 =>
                    e_vg <= m_acc(27 downto 7);
                    m_a <= resize(e_vy, 36); m_b <= c_b20; m_acc <= (others => '0');
                when 35 =>
                    e_pb <= m_acc(27 downto 8);
                    m_a <= resize(c_c20, 36); m_b <= e_sin; m_acc <= (others => '0');
                when 52 =>
                    if e_sneg = '1' then e_pr <= (not m_acc(34 downto 15)) + 1; else e_pr <= m_acc(34 downto 15); end if;
                when 53 =>
                    e_phi <= s_roll + e_pb + e_pr;
                when 54 =>
                    if c_allw = '1' then e_w <= '1';
                    elsif c_alld = '1' then e_w <= '0';
                    elsif e_phi(19 downto 4) < c_duty then e_w <= '1';
                    else e_w <= '0'; end if;
                when 55 =>
                    if e_w = '0' and e_pw = '1' then
                        e_ds <= e_vy;
                        if e_lfsr(0) = '1' then
                            v_x := ('0' & e_lfsr(15 downto 1)) xor x"B400";
                        else
                            v_x := '0' & e_lfsr(15 downto 1);
                        end if;
                        e_lfsr <= v_x;
                        e_del <= signed(v_x(6 downto 0));
                        if v_x(9 downto 8) = "00" then e_sl1 <= '1'; else e_sl1 <= '0'; end if;   -- one band in four slips
                    end if;
                    -- the warm top-left corner belongs to the bright bands that start above line 280
                    if e_w = '1' and e_pw = '0' then
                        if e_vy < C_CORN then e_cor <= '1'; else e_cor <= '0'; end if;
                    end if;
                    e_pw <= e_w;
                when 56 =>
                    e_dl <= e_vy - e_ds;
                when 57 =>
                    if s_tear = '1' and e_w = '0' and e_dl < 512 then e_dot <= '1'; else e_dot <= '0'; end if;
                    if s_tear = '1' and e_w = '0' and e_sl1 = '1' and e_dl >= 512 and e_dl < 5120 then e_slip <= '1'; else e_slip <= '0'; end if;
                when 58 =>
                    v_l := s_scr - e_vg;
                    if e_slip = '1' then
                        v_l := v_l + unsigned(shift_left(resize(e_del, 21), 8));   -- delta / 2 px
                    end if;
                    n_l9 <= v_l;
                    n_w <= e_w; n_dot <= e_dot;
                    n_lowr <= not e_cor;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: x (half virtual px) and A = g8 (x - xc), Q9
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x2 <= resize(s_x0, 12);
                b_a9 <= c_a0;
                l_w <= n_w; l_dot <= n_dot; l_lowr <= n_lowr; l_l9 <= n_l9;
            elsif g_avid = '1' then
                b_x2 <= b_x2 + s_xs;
                b_a9 <= b_a9 + signed(resize(c_ast, 23));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t  : unsigned(11 downto 0);
        variable v_lm : unsigned(7 downto 0);
        variable v_c  : unsigned(2 downto 0);
        variable v_r  : std_logic_vector(7 downto 0);
        variable v_f  : std_logic;
        variable v_p  : std_logic_vector(29 downto 0);
        variable v_na : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: side, d, source luma (signed about the Osc mean)
            if s_mir = '1' then
                p1_sl <= '1';
            elsif l_w = '1' then
                if b_x2 < C_SPLW then p1_sl <= '1'; else p1_sl <= '0'; end if;
            else
                if b_x2 < C_SPLD then p1_sl <= '1'; else p1_sl <= '0'; end if;
            end if;
            p1_a9 <= b_a9;
            p1_d2 <= signed(resize(b_x2, 13)) - C_XC2;
            if s_src = '1' then p1_ls <= signed(resize(v_lb, 9)) - 63; else p1_ls <= signed(resize(v_lb, 9)); end if;
            if l_dot = '1' and b_x2 < 1040 and b_x2(5 downto 0) < 22 then p1_dot <= '1'; else p1_dot <= '0'; end if;
            p1_dn <= (not l_w) and (not s_mir);

            -- 2: signed A for the family (mirror: |A|), curl operand, bend partials
            v_na := -p1_a9;
            if s_mir = '1' then
                if p1_a9 < 0 then p2_sel <= unsigned(v_na(20 downto 0)); else p2_sel <= unsigned(p1_a9(20 downto 0)); end if;
            elsif p1_sl = '1' then
                p2_sel <= unsigned(v_na(20 downto 0));
            else
                p2_sel <= unsigned(p1_a9(20 downto 0));
            end if;
            if p1_dn = '1' and p1_sl = '1' and p1_d2 >= 0 and p1_d2 < 256 then
                p2_dd <= unsigned(p1_d2(7 downto 0));
            else
                p2_dd <= (others => '0');
            end if;
            if p1_ls < 0 then v_lm := unsigned(resize(-p1_ls, 8)); p2_bn <= '1';
            else v_lm := unsigned(p1_ls(7 downto 0)); p2_bn <= '0'; end if;
            p2_q0 <= c_cb * v_lm(3 downto 0);
            p2_q1 <= c_cb * v_lm(7 downto 4);

            -- 3: u = A + line term; curl product; bend sum
            p3_u <= p2_sel + l_l9;
            p3_cu <= p2_dd * p2_dd;
            p3_b <= resize(p2_q0, 18) + shift_left(resize(p2_q1, 18), 4);
            p3_bn <= p2_bn;

            -- 4: + curl; bend sign
            p4_u <= p3_u + resize(p3_cu(15 downto 1), 21);
            if p3_bn = '1' then p4_b <= (not resize(p3_b, 21)) + 1; else p4_b <= resize(p3_b, 21); end if;

            -- 5: + bend
            p5_u <= p4_u + p4_b;

            -- 6: barcode index = u / 512 + 1024 (mod 4096), folded at the table ends
            v_t := p5_u(20 downto 9) + 1024;
            if v_t(11) = '1' then
                r_bar_a <= (not p_sl(5)) & (not v_t(10 downto 0));
            else
                r_bar_a <= (not p_sl(5)) & v_t(10 downto 0);
            end if;

            -- 7: EBR out; 8: fabric copy
            p8_rom <= r_bar_q;

            -- 9: colour code (bright / dark colourizer); top-left corner two levels down
            v_r := p8_rom;
            if l_w = '1' then v_c := unsigned(v_r(2 downto 0)); else v_c := unsigned(v_r(5 downto 3)); end if;
            if l_w = '1' and l_lowr = '1' and p_sl(8) = '1' and s_mir = '0' and v_r(6) = '1' then
                if v_c = 3 then v_c := to_unsigned(1, 3);
                elsif v_c = 4 then v_c := to_unsigned(2, 3); end if;
            end if;
            p9_c <= v_c;
            if l_w = '0' and v_c = 0 then p9_k <= '1'; else p9_k <= '0'; end if;

            -- 10..16: 7-px look-ahead window for the fringe
            p_c(10) <= p9_c; p_k(10) <= p9_k;
            for i in 11 to 16 loop p_c(i) <= p_c(i - 1); p_k(i) <= p_k(i - 1); end loop;

            -- 17: fringe (the last F px before black in a dark band) and the sync-glitch dots
            v_f := '0';
            if s_fr = '1' and l_w = '0' and p_k(16) = '0' then
                case to_integer(s_fw) is
                    when 4 => v_f := p_k(15) or p_k(14) or p_k(13) or p_k(12);
                    when 5 => v_f := p_k(15) or p_k(14) or p_k(13) or p_k(12) or p_k(11);
                    when others => v_f := p_k(15) or p_k(14) or p_k(13) or p_k(12) or p_k(11) or p_k(10) or p9_k;
                end case;
            end if;
            if p_dot(16) = '1' then
                p17_c <= s_pal & "111";
            elsif v_f = '1' then
                if p_c(16) = 2 then p17_c <= s_pal & "110"; else p17_c <= s_pal & "101"; end if;
            else
                p17_c <= s_pal & p_c(16);
            end if;

            -- 18: palette; neutral outside the active line (before the conditioning)
            v_p := C_TT_PAL(to_integer(p17_c));
            if s_gav_sr(16) = '0' then
                x18_y <= to_unsigned(64, 10); x18_u <= to_unsigned(512, 10); x18_v <= to_unsigned(512, 10);
            else
                x18_y <= unsigned(v_p(29 downto 20)); x18_u <= unsigned(v_p(19 downto 10)); x18_v <= unsigned(v_p(9 downto 0));
            end if;

            -- 19..28: soft edges = output conditioning: two adjacent 16-sums (1080) or 8-sums
            -- (720p / SD, taps 4..11 so both are centred alike) on Y, U and V
            s_cy(0) <= x18_y; s_cu(0) <= x18_u; s_cv(0) <= x18_v;
            for i in 1 to 15 loop s_cy(i) <= s_cy(i - 1); s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            s_by16 <= s_by16 + x18_y - s_cy(15); s_bu16 <= s_bu16 + x18_u - s_cu(15); s_bv16 <= s_bv16 + x18_v - s_cv(15);
            s_by8 <= s_by8 + s_cy(3) - s_cy(11); s_bu8 <= s_bu8 + s_cu(3) - s_cu(11); s_bv8 <= s_bv8 + s_cv(3) - s_cv(11);
            s_by16d <= s_by16; s_bu16d <= s_bu16; s_bv16d <= s_bv16;
            s_by8d <= s_by8; s_bu8d <= s_bu8; s_bv8d <= s_bv8;
            s_oy16 <= resize(s_by16, 15) + s_by16d; s_ou16 <= resize(s_bu16, 15) + s_bu16d; s_ov16 <= resize(s_bv16, 15) + s_bv16d;
            s_oy8 <= resize(s_by8, 14) + s_by8d; s_ou8 <= resize(s_bu8, 14) + s_bu8d; s_ov8 <= resize(s_bv8, 14) + s_bv8d;
            -- 29: normalise
            if t_hd = '1' then
                s_y29 <= s_oy16(14 downto 5); s_u29 <= s_ou16(14 downto 5); s_v29 <= s_ov16(14 downto 5);
            else
                s_y29 <= s_oy8(13 downto 4); s_u29 <= s_ou8(13 downto 4); s_v29 <= s_ov8(13 downto 4);
            end if;
            -- 30: luma toward black (13/16); blanking gate
            if s_gav_sr(28) = '1' then
                if s_y29 > 64 then
                    s_y30 <= s_y29 - shift_right(s_y29 - 64, 3) - shift_right(s_y29 - 64, 4);
                else
                    s_y30 <= s_y29;
                end if;
                s_u30 <= s_u29; s_v30 <= s_v29;
            else
                s_y30 <= to_unsigned(64, 10); s_u30 <= to_unsigned(512, 10); s_v30 <= to_unsigned(512, 10);
            end if;
            -- 31, 32: pad to the 32-clock latency
            s_y31 <= s_y30; s_u31 <= s_u30; s_v31 <= s_v30;
            s_y32 <= s_y31; s_u32 <= s_u31; s_v32 <= s_v31;

            -- delays
            p_sl(2) <= p1_sl;
            for i in 3 to 8 loop p_sl(i) <= p_sl(i - 1); end loop;
            p_dot(2) <= p1_dot;
            for i in 3 to 16 loop p_dot(i) <= p_dot(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y32);
    data_out.u       <= std_logic_vector(s_u32);
    data_out.v       <= std_logic_vector(s_v32);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture tartan;
