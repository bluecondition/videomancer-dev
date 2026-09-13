-- Benday: catalogue combo #31 (seed 20260917) -- chromatic glitch / datamosh
-- bars + ASCII / dither-art / halftone.  Three knobs each; the switches
-- and the slider decide how the two combine.
--
--   K1 Cell     halftone cell 4 .. 32 px
--   K2 Ink      dot gain
--   K3 Screen   dot / line / diamond / square
--   K4 Height   mosh band height 8 .. 64 lines
--   K5 Shift    block shift inside a moshed band (px)
--   K6 Rate     re-roll rate
--
--   S7  Ink     dark ink on light / light ink on dark
--   S8  Colour  mono print / dots take the cell colour
--   S9  Mosh    shifts the picture / shifts the screen
--   S10 Bands   chroma hold + U/V swap / luma crush
--   S11 Bands   halftoned / flat cells
--   P12 Density fraction of bands moshed
--
-- Cell average = running box sum (EBR ring) into the line buffer (rows
-- captured every cell); the halftone threshold comes from a 4 x 16 x 16
-- screen ROM (EBR) indexed by the position inside the cell.  Bands are
-- hashed per band and re-rolled on a frame counter.  Latency 12 clocks.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture benday of program_top is

    constant C_LAT : integer := 12;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    function f_round1(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 5);
        return t xor shift_right(t, 6);
    end function;
    function f_round2(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 3);
        return t xor shift_right(t, 8);
    end function;
    function f_round3(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 7);
        t := t xor shift_right(t, 5);
        return t xor shift_left(t, 8);
    end function;

    type t_screen is array (0 to 1023) of integer range 0 to 255;
    constant C_SCREEN : t_screen := (
        16, 31, 45, 58, 68, 77, 83, 86, 86, 83, 77, 68, 58, 45, 31, 16,
        31, 48, 63, 77, 89, 98, 105, 108, 108, 105, 98, 89, 77, 63, 48, 31,
        45, 63, 80, 95, 108, 119, 127, 131, 131, 127, 119, 108, 95, 80, 63, 45,
        58, 77, 95, 112, 127, 139, 148, 153, 153, 148, 139, 127, 112, 95, 77, 58,
        68, 89, 108, 127, 143, 158, 169, 175, 175, 169, 158, 143, 127, 108, 89, 68,
        77, 98, 119, 139, 158, 175, 189, 198, 198, 189, 175, 158, 139, 119, 98, 77,
        83, 105, 127, 148, 169, 189, 207, 219, 219, 207, 189, 169, 148, 127, 105, 83,
        86, 108, 131, 153, 175, 198, 219, 239, 239, 219, 198, 175, 153, 131, 108, 86,
        86, 108, 131, 153, 175, 198, 219, 239, 239, 219, 198, 175, 153, 131, 108, 86,
        83, 105, 127, 148, 169, 189, 207, 219, 219, 207, 189, 169, 148, 127, 105, 83,
        77, 98, 119, 139, 158, 175, 189, 198, 198, 189, 175, 158, 139, 119, 98, 77,
        68, 89, 108, 127, 143, 158, 169, 175, 175, 169, 158, 143, 127, 108, 89, 68,
        58, 77, 95, 112, 127, 139, 148, 153, 153, 148, 139, 127, 112, 95, 77, 58,
        45, 63, 80, 95, 108, 119, 127, 131, 131, 127, 119, 108, 95, 80, 63, 45,
        31, 48, 63, 77, 89, 98, 105, 108, 108, 105, 98, 89, 77, 63, 48, 31,
        16, 31, 45, 58, 68, 77, 83, 86, 86, 83, 77, 68, 58, 45, 31, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
        80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
        112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112,
        143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143,
        175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175,
        207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207,
        239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239,
        239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239,
        207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207,
        175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175,
        143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143, 143,
        112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112,
        80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
        48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        16, 32, 48, 64, 80, 96, 112, 128, 128, 112, 96, 80, 64, 48, 32, 16,
        32, 48, 64, 80, 96, 112, 128, 143, 143, 128, 112, 96, 80, 64, 48, 32,
        48, 64, 80, 96, 112, 128, 143, 159, 159, 143, 128, 112, 96, 80, 64, 48,
        64, 80, 96, 112, 128, 143, 159, 175, 175, 159, 143, 128, 112, 96, 80, 64,
        80, 96, 112, 128, 143, 159, 175, 191, 191, 175, 159, 143, 128, 112, 96, 80,
        96, 112, 128, 143, 159, 175, 191, 207, 207, 191, 175, 159, 143, 128, 112, 96,
        112, 128, 143, 159, 175, 191, 207, 223, 223, 207, 191, 175, 159, 143, 128, 112,
        128, 143, 159, 175, 191, 207, 223, 239, 239, 223, 207, 191, 175, 159, 143, 128,
        128, 143, 159, 175, 191, 207, 223, 239, 239, 223, 207, 191, 175, 159, 143, 128,
        112, 128, 143, 159, 175, 191, 207, 223, 223, 207, 191, 175, 159, 143, 128, 112,
        96, 112, 128, 143, 159, 175, 191, 207, 207, 191, 175, 159, 143, 128, 112, 96,
        80, 96, 112, 128, 143, 159, 175, 191, 191, 175, 159, 143, 128, 112, 96, 80,
        64, 80, 96, 112, 128, 143, 159, 175, 175, 159, 143, 128, 112, 96, 80, 64,
        48, 64, 80, 96, 112, 128, 143, 159, 159, 143, 128, 112, 96, 80, 64, 48,
        32, 48, 64, 80, 96, 112, 128, 143, 143, 128, 112, 96, 80, 64, 48, 32,
        16, 32, 48, 64, 80, 96, 112, 128, 128, 112, 96, 80, 64, 48, 32, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        16, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 16,
        16, 48, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 48, 16,
        16, 48, 80, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 143, 143, 143, 143, 143, 143, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 175, 175, 175, 175, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 207, 207, 207, 207, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 207, 239, 239, 207, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 207, 239, 239, 207, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 207, 207, 207, 207, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 175, 175, 175, 175, 175, 175, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 143, 143, 143, 143, 143, 143, 143, 143, 112, 80, 48, 16,
        16, 48, 80, 112, 112, 112, 112, 112, 112, 112, 112, 112, 112, 80, 48, 16,
        16, 48, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 48, 16,
        16, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_bring is array (0 to 63) of std_logic_vector(31 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal bring : t_bring := (others => x"04080200");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_bring_q : std_logic_vector(31 downto 0) := x"04080200";
    signal s_bring_w : unsigned(5 downto 0) := (others => '0');
    signal s_scr_addr : unsigned(9 downto 0) := (others => '0');
    signal s_scr_q, s_scr_q2 : unsigned(7 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_cell, s_k_ink, s_k_screen, s_k_height, s_k_shift, s_k_rate : unsigned(9 downto 0);
    signal s_sw_light, s_sw_colour, s_sw_moshscreen, s_sw_crush, s_sw_flat : std_logic;
    signal s_p_density : unsigned(9 downto 0);
    signal s_nsh    : unsigned(2 downto 0) := to_unsigned(3, 3);       -- cell shift 2..5
    signal s_mask   : unsigned(10 downto 0) := (others => '0');
    signal s_half   : unsigned(10 downto 0) := (others => '0');
    signal s_ndel   : unsigned(5 downto 0) := to_unsigned(7, 6);
    signal s_rows   : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_gain   : unsigned(7 downto 0) := (others => '0');
    signal s_shape  : unsigned(1 downto 0) := (others => '0');
    signal s_bsh    : unsigned(2 downto 0) := to_unsigned(3, 3);       -- band shift 3..6
    signal s_shift  : unsigned(7 downto 0) := (others => '0');
    signal s_dens   : unsigned(7 downto 0) := (others => '0');
    signal s_roll   : unsigned(15 downto 0) := (others => '0');
    signal s_rate_cnt : unsigned(7 downto 0) := (others => '0');
    signal s_vstep  : unsigned(1 downto 0) := "11";

    -- tracking / per-line
    signal s_rx, s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_vcnt   : unsigned(6 downto 0) := (others => '0');
    signal s_capture, s_wpar, s_rbank : std_logic := '0';
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_hb     : unsigned(15 downto 0) := (others => '0');
    signal s_band_row : std_logic := '0';
    signal s_off_row : signed(10 downto 0) := (others => '0');
    signal s_swap_row : std_logic := '0';
    signal s_ny     : unsigned(3 downto 0) := (others => '0');

    -- input side
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_accy, s_accu, s_accv : unsigned(15 downto 0) := (others => '0');
    signal s_ay3, s_au3, s_av3 : unsigned(9 downto 0) := (others => '0');
    type t_wx is array (1 to 2) of unsigned(10 downto 0);
    signal s_wx : t_wx := (others => (others => '0'));
    signal s_wp, s_wc : std_logic_vector(1 to 2) := (others => '0');
    signal s_wx3 : unsigned(10 downto 0) := (others => '0');
    signal s_wp3, s_we3 : std_logic := '0';

    -- read side
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_xs2    : signed(12 downto 0) := (others => '0');
    signal s_xp2    : unsigned(10 downto 0) := (others => '0');
    signal s_addr3  : unsigned(10 downto 0) := (others => '0');
    signal s_bg3    : std_logic := '0';
    signal s_nx3    : unsigned(3 downto 0) := (others => '0');
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_pg6    : unsigned(15 downto 0) := (others => '0');
    signal s_u6, s_v6 : unsigned(9 downto 0) := (others => '0');
    signal s_lum7   : unsigned(7 downto 0) := (others => '0');
    signal s_thr6, s_thr7 : unsigned(7 downto 0) := (others => '0');
    signal s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    signal s_y7f    : unsigned(9 downto 0) := (others => '0');
    signal s_ink8   : std_logic := '0';
    signal s_u8, s_v8, s_y8f : unsigned(9 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_hu, s_hv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_bandd  : std_logic_vector(2 to 10) := (others => '0');
    signal s_xd     : std_logic_vector(2 to 9) := (others => '0');   -- x low bits zero flag

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_cell   <= unsigned(registers_in(0));
    s_k_ink    <= unsigned(registers_in(1));
    s_k_screen <= unsigned(registers_in(2));
    s_k_height <= unsigned(registers_in(3));
    s_k_shift  <= unsigned(registers_in(4));
    s_k_rate   <= unsigned(registers_in(5));
    s_sw_light      <= registers_in(6)(0);
    s_sw_colour     <= registers_in(6)(1);
    s_sw_moshscreen <= registers_in(6)(2);
    s_sw_crush      <= registers_in(6)(3);
    s_sw_flat       <= registers_in(6)(4);
    s_p_density <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_scr_q  <= to_unsigned(C_SCREEN(to_integer(s_scr_addr)), 8);
            s_scr_q2 <= s_scr_q;
        end if;
    end process;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_bring_w <= s_bring_w + 1;
            s_bring_q <= bring(to_integer(s_bring_w - s_ndel));
            bring(to_integer(s_bring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control + input side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_oy, v_ou, v_ov : unsigned(9 downto 0);
        variable v_n  : unsigned(3 downto 0);
        variable v_k  : unsigned(15 downto 0);
        variable v_o  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);

            -- stage 2: running sums (window N ending at data_in)
            v_oy := unsigned(s_bring_q(29 downto 20)); v_ou := unsigned(s_bring_q(19 downto 10)); v_ov := unsigned(s_bring_q(9 downto 0));
            if data_in.avid = '0' then
                s_accy <= (others => '0'); s_accu <= (others => '0'); s_accv <= (others => '0');
            else
                s_accy <= s_accy + resize(unsigned(data_in.y), 16) - resize(v_oy, 16);
                s_accu <= s_accu + resize(unsigned(data_in.u), 16) - resize(v_ou, 16);
                s_accv <= s_accv + resize(unsigned(data_in.v), 16) - resize(v_ov, 16);
            end if;
            -- stage 3: averages
            s_ay3 <= resize(shift_right(s_accy, to_integer(s_nsh)), 10);
            s_au3 <= resize(shift_right(s_accu, to_integer(s_nsh)), 10);
            s_av3 <= resize(shift_right(s_accv, to_integer(s_nsh)), 10);
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar; s_wc(1) <= data_in.avid and s_capture;
            s_wx(2) <= s_wx(1); s_wp(2) <= s_wp(1); s_wc(2) <= s_wc(1);
            s_wx3 <= s_wx(2); s_wp3 <= s_wp(2); s_we3 <= s_wc(2);

            -- per-line: band hash, offset, ny
            case to_integer(s_lstep) is
                when 0 =>
                    v_k := s_roll + resize(shift_right(s_line, to_integer(s_bsh)), 16);
                    s_hb <= x"3D71" + v_k;
                    case to_integer(s_nsh) is
                        when 2 => s_ny <= s_line(1 downto 0) & "00";
                        when 3 => s_ny <= s_line(2 downto 0) & "0";
                        when 4 => s_ny <= s_line(3 downto 0);
                        when others => s_ny <= s_line(4 downto 1);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 1 => s_hb <= f_round1(s_hb); s_lstep <= s_lstep + 1;
                when 2 => s_hb <= f_round2(s_hb); s_lstep <= s_lstep + 1;
                when 3 => s_hb <= f_round3(s_hb); s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_hb(15 downto 8) < s_dens then s_band_row <= '1'; else s_band_row <= '0'; end if;
                    s_swap_row <= s_hb(0);
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    if s_band_row = '1' then
                        v_o := resize(shift_right((signed('0' & s_hb(7 downto 0)) - to_signed(128, 9)) * signed('0' & s_shift), 5), 13);
                    else
                        v_o := (others => '0');
                    end if;
                    s_off_row <= resize(v_o, 11);
                    s_lstep <= "111";
                when others => null;
            end case;

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    v_n := resize(s_k_cell(9 downto 8), 4) + 2;
                    s_nsh   <= v_n(2 downto 0);
                    s_gain  <= s_k_ink(9 downto 2);
                    s_shape <= s_k_screen(9 downto 8);
                    s_bsh   <= resize(s_k_height(9 downto 8), 3) + 3;
                    s_shift <= s_k_shift(9 downto 2);
                    s_dens  <= s_p_density(9 downto 2);
                    if s_rate_cnt >= resize(not s_k_rate(9 downto 3), 8) then
                        s_rate_cnt <= (others => '0');
                        s_roll <= s_roll + 977;
                    else
                        s_rate_cnt <= s_rate_cnt + 1;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_rows <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)), 7);
                    s_ndel <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)), 7)(5 downto 0) - 1;
                    for i in 0 to 10 loop
                        if i < to_integer(s_nsh) then s_mask(i) <= '1'; else s_mask(i) <= '0'; end if;
                        if i = to_integer(s_nsh) - 1 then s_half(i) <= '1'; else s_half(i) <= '0'; end if;
                    end loop;
                    s_vstep <= "11";
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                if s_vcnt >= s_rows - 1 then s_vcnt <= (others => '0'); else s_vcnt <= s_vcnt + 1; end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_vcnt = 0 then s_capture <= '1'; s_wpar <= not s_wpar; s_rbank <= s_wpar;
                else s_capture <= '0'; s_rbank <= s_wpar; end if;
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_vcnt <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then s_vstep <= (others => '0'); end if;
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

    --------------------------------------------------------------------------
    -- Read address + screen index (stages 1..3)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_a : unsigned(11 downto 0);
        variable v_x : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx;
            -- stage 2: picture shift (mosh on the picture) and screen-phase x
            if s_sw_moshscreen = '0' then
                s_xs2 <= signed(resize(s_x1, 13)) + resize(s_off_row, 13);
                s_xp2 <= s_x1;
            else
                s_xs2 <= signed(resize(s_x1, 13));
                s_xp2 <= unsigned(resize(signed(resize(s_x1, 13)) + resize(s_off_row, 13), 11));
            end if;
            s_bandd(2) <= s_band_row; for i in 3 to 10 loop s_bandd(i) <= s_bandd(i - 1); end loop;
            if s_x1(4 downto 0) = 0 then s_xd(2) <= '1'; else s_xd(2) <= '0'; end if;
            for i in 3 to 9 loop s_xd(i) <= s_xd(i - 1); end loop;
            -- stage 3: cell-hold address, screen index
            if s_xs2 < 0 then
                v_x := (others => '0'); s_bg3 <= '1';
            elsif s_xs2 >= signed(resize(s_line_width, 13)) then
                v_x := (others => '0'); s_bg3 <= '1';
            else
                v_x := unsigned(s_xs2(10 downto 0)); s_bg3 <= '0';
            end if;
            v_a := resize(v_x and not s_mask, 12) + resize(s_half, 12);
            if v_a >= resize(s_line_width, 12) then s_addr3 <= s_line_width - 1; else s_addr3 <= v_a(10 downto 0); end if;
            case to_integer(s_nsh) is
                when 2 => s_nx3 <= s_xp2(1 downto 0) & "00";
                when 3 => s_nx3 <= s_xp2(2 downto 0) & "0";
                when 4 => s_nx3 <= s_xp2(3 downto 0);
                when others => s_nx3 <= s_xp2(4 downto 1);
            end case;
        end if;
    end process p_addr;

    s_scr_addr <= s_shape & s_ny & s_nx3;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 3, read stage 3 -> data 4)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr3));
            if s_we3 = '1' and s_wp3 = '0' then lbY0(to_integer(s_wx3)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr3));
            if s_we3 = '1' and s_wp3 = '1' then lbY1(to_integer(s_wx3)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr3(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '0' then lbU0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr3(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '1' then lbU1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr3(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '0' then lbV0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr3(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '1' then lbV1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 4..12)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ink : std_logic;
    begin
        if rising_edge(clk) then
            -- stage 4 ctx / 5 fetch
            s_a_bg <= s_bg3; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 6: ink gain on the cell luma (above black)
            s_pg6 <= s_wet_y(9 downto 2) * s_gain;
            s_u6 <= s_wet_u; s_v6 <= s_wet_v;
            -- stage 7: 8-bit lum, threshold (ROM address at 3, q 4, q2 5, thr6, thr7)
            if s_pg6(15 downto 7) > 255 then s_lum7 <= x"FF"; else s_lum7 <= s_pg6(14 downto 7); end if;
            s_thr6 <= s_scr_q2;
            s_thr7 <= s_thr6;
            s_u7 <= s_u6; s_v7 <= s_v6;
            s_y7f <= s_wet_y;
            -- stage 8: ink decision
            if s_lum7 < s_thr7 then v_ink := '1'; else v_ink := '0'; end if;
            s_ink8 <= v_ink;
            s_u8 <= s_u7; s_v8 <= s_v7; s_y8f <= s_y7f;
            -- stage 9: compose
            if s_sw_flat = '1' and s_bandd(8) = '1' then
                s_y9 <= s_y8f; s_u9 <= s_u8; s_v9 <= s_v8;
            elsif (s_ink8 xor s_sw_light) = '1' then
                s_y9 <= to_unsigned(64, 10);
                if s_sw_colour = '1' and s_sw_light = '1' then s_u9 <= s_u8; s_v9 <= s_v8; else s_u9 <= to_unsigned(512, 10); s_v9 <= to_unsigned(512, 10); end if;
            else
                s_y9 <= to_unsigned(880, 10);
                if s_sw_colour = '1' and s_sw_light = '0' then s_u9 <= s_u8; s_v9 <= s_v8; else s_u9 <= to_unsigned(512, 10); s_v9 <= to_unsigned(512, 10); end if;
            end if;
            -- stage 10: band artefacts
            if s_xd(9) = '1' then s_hu <= s_v9; s_hv <= s_u9; end if;
            if s_bandd(9) = '1' then
                if s_sw_crush = '1' then
                    s_y10 <= (s_y9 and "1100000000") or "0010000000"; s_u10 <= s_u9; s_v10 <= s_v9;
                else
                    s_y10 <= s_y9; s_u10 <= s_hu; s_v10 <= s_hv;
                end if;
            else
                s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            end if;
            -- stage 11: gate
            if s_avid_sr(9) = '1' then
                s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            else
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            end if;
            -- stage 12: output
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y12);
    data_out.u       <= std_logic_vector(s_u12);
    data_out.v       <= std_logic_vector(s_v12);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture benday;
