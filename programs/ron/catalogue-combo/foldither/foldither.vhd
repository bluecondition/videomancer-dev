-- Foldither: catalogue combo #117 (seed 20260926) -- ordered dithering
-- (Bayer) + mirror / kaleidoscope.  Three knobs each; the switches and the
-- slider decide how they combine.
--
--   K1 Segment  mirror segment width 1024 / 512 / 256 / 128 / 64 px
--   K2 Slide    slide speed of the mirrored strip, bipolar
--   K3 Skew     per-line skew of the fold axis, bipolar (diagonal mirrors)
--   K4 Bits     luma levels 1..6 bits
--   K5 Dither   dither amount (0..100% of a step)
--   K6 Matrix   2x2 / 4x4 / 8x8 Bayer / line screen
--
--   S7  Fold    mirror (triangle) / repeat (sawtooth tiles)
--   S8  Dither  in source space (the pattern folds with the picture) /
--               in screen space (the pattern stays put)
--   S9  Chroma  chroma dithered with the same matrix / untouched
--   S10 Alt     alternate segments negative
--   S11 Alt     alternate segments drop two more bits
--   P12 Depth   nested folds 1..4 (each halves the segment)
--
-- Nested power-of-two folds (registered per-frame constants) on the read
-- address; threshold dither + floor quantizer on the write port (source)
-- or after the fetch (screen).  Latency 20 clocks, all modes,
-- blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture foldither of program_top is

    constant C_LAT : integer := 20;

    type t_b8 is array (0 to 63) of integer range 0 to 63;
    constant C_B8 : t_b8 := (0, 32, 8, 40, 2, 34, 10, 42, 48, 16, 56, 24, 50, 18, 58, 26, 12, 44, 4, 36, 14, 46, 6, 38, 60, 28, 52, 20, 62, 30, 54, 22, 3, 35, 11, 43, 1, 33, 9, 41, 51, 19, 59, 27, 49, 17, 57, 25, 15, 47, 7, 39, 13, 45, 5, 37, 63, 31, 55, 23, 61, 29, 53, 21);
    type t_b4 is array (0 to 15) of integer range 0 to 15;
    constant C_B4 : t_b4 := (0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5);

    function f_thr(mat : unsigned(1 downto 0); bx, by : unsigned(2 downto 0)) return unsigned is
        variable v_t : unsigned(5 downto 0);
    begin
        case to_integer(mat) is
            when 0 =>
                case to_integer(unsigned'(by(0) & bx(0))) is
                    when 0 => v_t := "001000"; when 1 => v_t := "101000"; when 2 => v_t := "111000"; when others => v_t := "011000";
                end case;
            when 1 => v_t := to_unsigned(C_B4(to_integer(by(1 downto 0) & bx(1 downto 0))), 4) & "10";
            when 2 => v_t := to_unsigned(C_B8(to_integer(by & bx)), 6);
            when others => v_t := by & "100";
        end case;
        return v_t;
    end function;

    function f_q(yv : unsigned(10 downto 0); nm : unsigned(9 downto 0); h : unsigned(9 downto 0)) return unsigned is
        variable v_q : unsigned(9 downto 0);
    begin
        if yv > 1023 then v_q := (others => '1'); else v_q := yv(9 downto 0); end if;
        return (v_q and nm) + h;
    end function;

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_seg, s_k_slide, s_k_skew, s_k_bits, s_k_dith, s_k_mat : unsigned(9 downto 0);
    signal s_sw_repeat, s_sw_screen, s_sw_chroma, s_sw_altneg, s_sw_altdrop : std_logic;
    signal s_p_depth : unsigned(9 downto 0);

    signal s_ks     : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal s_depth  : unsigned(1 downto 0) := "00";
    type t_fc is array (1 to 4) of signed(12 downto 0);
    signal s_fmask, s_fhalf, s_flow : t_fc := (others => (others => '0'));
    signal s_halff  : signed(12 downto 0) := (others => '0');
    signal s_pbit   : signed(12 downto 0) := (others => '0');
    signal s_fen    : std_logic_vector(1 to 4) := (others => '0');
    signal s_skew   : signed(8 downto 0) := (others => '0');
    signal s_slide  : signed(15 downto 0) := (others => '0');
    signal s_shrow  : signed(15 downto 0) := (others => '0');
    signal s_cx     : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_k, s_kb : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_notmask, s_half, s_notmaskb, s_halfb : unsigned(9 downto 0) := (others => '0');
    signal s_dsc, s_dscb : unsigned(9 downto 0) := (others => '0');
    signal s_dl, s_dlb : unsigned(17 downto 0) := (others => '0');
    signal s_stepl, s_steplb : unsigned(9 downto 0) := (others => '0');
    signal s_mat    : unsigned(1 downto 0) := "10";
    signal s_vstep  : unsigned(2 downto 0) := "111";
    signal s_saw_active : std_logic := '0';

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    -- write side (source-space dither)
    signal s_thr2 : unsigned(5 downto 0) := (others => '0');
    signal s_wy2, s_wu2, s_wv2, s_wy3, s_wu3, s_wv3, s_wy4, s_wu4, s_wv4, s_wy5, s_wu5, s_wv5 : unsigned(9 downto 0) := (others => '0');
    signal s_wx2, s_wx3, s_wx4, s_wx5 : unsigned(10 downto 0) := (others => '0');
    signal s_we2, s_we3, s_we4, s_we5, s_wpar2, s_wpar3, s_wpar4, s_wpar5 : std_logic := '0';
    signal s_wd3 : unsigned(16 downto 0) := (others => '0');
    signal s_wyv4, s_wuv4, s_wvv4 : unsigned(10 downto 0) := (others => '0');
    -- read side
    type t_b is array (1 to 13) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_x1 : unsigned(10 downto 0) := (others => '0');
    type t_xd is array (2 to 15) of unsigned(10 downto 0);
    signal s_xd : t_xd := (others => (others => '0'));
    signal s_a3 : signed(12 downto 0) := (others => '0');
    type t_f is array (0 to 4) of signed(12 downto 0);
    signal s_m, s_t : t_f := (others => (others => '0'));
    signal s_par : std_logic_vector(4 to 19) := (others => '0');
    signal s_src12 : signed(12 downto 0) := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg : std_logic := '0';
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_thr16 : unsigned(5 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_d17 : unsigned(16 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_yv18, s_uv18, s_vv18 : unsigned(10 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_seg   <= unsigned(registers_in(0));
    s_k_slide <= unsigned(registers_in(1));
    s_k_skew  <= unsigned(registers_in(2));
    s_k_bits  <= unsigned(registers_in(3));
    s_k_dith  <= unsigned(registers_in(4));
    s_k_mat   <= unsigned(registers_in(5));
    s_sw_repeat  <= registers_in(6)(0);
    s_sw_screen  <= registers_in(6)(1);
    s_sw_chroma  <= registers_in(6)(2);
    s_sw_altneg  <= registers_in(6)(3);
    s_sw_altdrop <= registers_in(6)(4);
    s_p_depth <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_m : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            case to_integer(s_k_seg(9 downto 7)) is
                when 0 | 1 => s_ks <= to_unsigned(10, 4);
                when 2 | 3 => s_ks <= to_unsigned(9, 4);
                when 4 | 5 => s_ks <= to_unsigned(8, 4);
                when 6     => s_ks <= to_unsigned(7, 4);
                when others => s_ks <= to_unsigned(6, 4);
            end case;
            s_depth <= s_p_depth(9 downto 8);
            for i in 1 to 4 loop
                s_fhalf(i) <= shift_left(to_signed(1, 13), to_integer(s_ks) - i + 1);
                s_fmask(i) <= shift_left(to_signed(1, 13), to_integer(s_ks) - i + 2) - 1;
                s_flow(i)  <= shift_left(to_signed(1, 13), to_integer(s_ks) - i + 1) - 1;
                if i <= to_integer(s_depth) + 1 then s_fen(i) <= '1'; else s_fen(i) <= '0'; end if;
            end loop;
            s_halff <= shift_left(to_signed(1, 13), to_integer(s_ks) - to_integer(s_depth) - 1);
            s_pbit <= shift_left(to_signed(1, 13), to_integer(s_ks) - to_integer(s_depth));
            s_skew <= resize(shift_right(signed(resize(s_k_skew, 11)) - 512, 2), 9);
            case to_integer(s_k_bits(9 downto 7)) is
                when 0     => s_k <= to_unsigned(9, 4);
                when 1 | 2 => s_k <= to_unsigned(8, 4);
                when 3     => s_k <= to_unsigned(7, 4);
                when 4 | 5 => s_k <= to_unsigned(6, 4);
                when 6     => s_k <= to_unsigned(5, 4);
                when others => s_k <= to_unsigned(4, 4);
            end case;
            if s_k >= 7 then s_kb <= to_unsigned(9, 4); else s_kb <= s_k + 2; end if;
            s_mat <= s_k_mat(9 downto 8);
            s_cx <= '0' & s_line_width(10 downto 1);
            case to_integer(s_vstep) is
                when 0 =>
                    v_m := shift_left(to_unsigned(1, 10), to_integer(s_k)) - 1;
                    s_notmask <= not v_m; s_half <= shift_left(to_unsigned(1, 10), to_integer(s_k) - 1);
                    s_stepl <= shift_left(to_unsigned(1, 10), to_integer(s_k));
                    v_m := shift_left(to_unsigned(1, 10), to_integer(s_kb)) - 1;
                    s_notmaskb <= not v_m; s_halfb <= shift_left(to_unsigned(1, 10), to_integer(s_kb) - 1);
                    s_steplb <= shift_left(to_unsigned(1, 10), to_integer(s_kb));
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_dl <= s_stepl * s_k_dith(9 downto 2);
                    s_dlb <= s_steplb * s_k_dith(9 downto 2);
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_dsc <= s_dl(17 downto 8); s_dscb <= s_dlb(17 downto 8);
                    s_vstep <= "111";
                when others => null;
            end case;
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                s_shrow <= s_shrow + resize(s_skew, 16);
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then
                    s_slide <= s_slide + resize(shift_right(signed(resize(s_k_slide, 11)) - 512, 3), 16);
                    s_vstep <= (others => '0');
                end if;
                s_shrow <= s_slide;
                s_saw_active <= '0';
            end if;
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 13 loop s_avd(i) <= s_avd(i - 1); end loop;
            s_xd(2) <= s_x1;
            for i in 3 to 15 loop s_xd(i) <= s_xd(i - 1); end loop;
            -- 3: position relative to the centre plus slide / skew
            s_a3 <= signed(resize(s_x1, 13)) - signed(resize(s_cx, 13)) + resize(s_shrow(15 downto 4), 13);
            -- segment parity for the alternate-segment looks
            if (s_a3 and s_pbit) /= 0 then s_par(4) <= '1'; else s_par(4) <= '0'; end if;
            for i in 5 to 19 loop s_par(i) <= s_par(i - 1); end loop;
            -- folds: stage i (i = 1..4) uses W_i = 2^(ks - i + 1)
            s_t(0) <= s_a3;
            for i in 1 to 4 loop
                s_m(i) <= (s_t(i - 1) and s_fmask(i));
                if s_fen(i) = '1' then
                    if s_sw_repeat = '1' then
                        s_t(i) <= s_m(i);
                    elsif (s_m(i) and s_fhalf(i)) /= 0 then
                        s_t(i) <= s_m(i) and s_flow(i);
                    else
                        s_t(i) <= s_fhalf(i) - s_m(i);
                    end if;
                else
                    s_t(i) <= s_t(i - 1);
                end if;
            end loop;
            -- 12: source position = centre - W_final/2 + t
            s_src12 <= signed(resize(s_cx, 13)) - s_halff + s_t(4);
            -- 13
            if s_src12 < 0 or s_src12 >= signed(resize(s_line_width, 13)) or s_avd(12) = '0' then s_bg <= '1'; else s_bg <= '0'; end if;
            s_addr <= unsigned(s_src12(10 downto 0));
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we5 = '1' and s_wpar5 = '0' then lbY0(to_integer(s_wx5)) <= std_logic_vector(s_wy5); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we5 = '1' and s_wpar5 = '1' then lbY1(to_integer(s_wx5)) <= std_logic_vector(s_wy5); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we5 = '1' and s_wpar5 = '0' then lbU0(to_integer(s_wx5(10 downto 1))) <= std_logic_vector(s_wu5(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we5 = '1' and s_wpar5 = '1' then lbU1(to_integer(s_wx5(10 downto 1))) <= std_logic_vector(s_wu5(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we5 = '1' and s_wpar5 = '0' then lbV0(to_integer(s_wx5(10 downto 1))) <= std_logic_vector(s_wv5(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we5 = '1' and s_wpar5 = '1' then lbV1(to_integer(s_wx5(10 downto 1))) <= std_logic_vector(s_wv5(9 downto 2)); end if;
        end if;
    end process;
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            -- write side 2..5: source-space dither + quantize, then write
            s_thr2 <= f_thr(s_mat, s_wr_x(2 downto 0), s_line(2 downto 0));
            s_wy2 <= s_in_y; s_wu2 <= s_in_u; s_wv2 <= s_in_v; s_wx2 <= s_wr_x; s_we2 <= s_we; s_wpar2 <= s_wpar;
            s_wd3 <= (s_thr2 & '1') * s_dsc;
            s_wy3 <= s_wy2; s_wu3 <= s_wu2; s_wv3 <= s_wv2; s_wx3 <= s_wx2; s_we3 <= s_we2; s_wpar3 <= s_wpar2;
            s_wyv4 <= resize(s_wy3, 11) + resize(s_wd3(16 downto 7), 11);
            s_wuv4 <= resize(s_wu3, 11) + resize(s_wd3(16 downto 7), 11) - resize(s_wd3(16 downto 8), 11);
            s_wvv4 <= resize(s_wv3, 11) + resize(s_wd3(16 downto 7), 11) - resize(s_wd3(16 downto 8), 11);
            s_wy4 <= s_wy3; s_wu4 <= s_wu3; s_wv4 <= s_wv3; s_wx4 <= s_wx3; s_we4 <= s_we3; s_wpar4 <= s_wpar3;
            if s_sw_screen = '0' then
                s_wy5 <= f_q(s_wyv4, s_notmask, s_half);
                if s_sw_chroma = '1' then s_wu5 <= f_q(s_wuv4, s_notmask, s_half); s_wv5 <= f_q(s_wvv4, s_notmask, s_half);
                else s_wu5 <= s_wu4; s_wv5 <= s_wv4; end if;
            else
                s_wy5 <= s_wy4; s_wu5 <= s_wu4; s_wv5 <= s_wv4;
            end if;
            s_wx5 <= s_wx4; s_we5 <= s_we4; s_wpar5 <= s_wpar4;
            -- 14 (rd) / 15: fetch
            s_a_bg <= s_bg; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 16: screen-space threshold
            s_thr16 <= f_thr(s_mat, s_xd(15)(2 downto 0), s_line(2 downto 0));
            s_y16 <= s_wet_y; s_u16 <= s_wet_u; s_v16 <= s_wet_v;
            -- 17: dither product (alternate segments: the deeper drop)
            if s_sw_altdrop = '1' and s_par(16) = '1' then s_d17 <= (s_thr16 & '1') * s_dscb; else s_d17 <= (s_thr16 & '1') * s_dsc; end if;
            s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            -- 18: add
            s_yv18 <= resize(s_y17, 11) + resize(s_d17(16 downto 7), 11);
            s_uv18 <= resize(s_u17, 11) + resize(s_d17(16 downto 7), 11) - resize(s_d17(16 downto 8), 11);
            s_vv18 <= resize(s_v17, 11) + resize(s_d17(16 downto 7), 11) - resize(s_d17(16 downto 8), 11);
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            -- 19: quantize (screen order) / pass, alternate negative
            if s_sw_screen = '1' or (s_sw_altdrop = '1' and s_par(18) = '1') then
                if s_sw_altdrop = '1' and s_par(18) = '1' then
                    s_y19 <= f_q(s_yv18, s_notmaskb, s_halfb);
                    if s_sw_chroma = '1' then s_u19 <= f_q(s_uv18, s_notmaskb, s_halfb); s_v19 <= f_q(s_vv18, s_notmaskb, s_halfb);
                    else s_u19 <= s_u18; s_v19 <= s_v18; end if;
                else
                    s_y19 <= f_q(s_yv18, s_notmask, s_half);
                    if s_sw_chroma = '1' then s_u19 <= f_q(s_uv18, s_notmask, s_half); s_v19 <= f_q(s_vv18, s_notmask, s_half);
                    else s_u19 <= s_u18; s_v19 <= s_v18; end if;
                end if;
            else
                s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            end if;
            -- 20: alternate negative + gate
            if s_avid_sr(C_LAT - 2) = '1' then
                if s_sw_altneg = '1' and s_par(19) = '1' then s_y20 <= not s_y19; else s_y20 <= s_y19; end if;
                s_u20 <= s_u19; s_v20 <= s_v19;
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture foldither;
