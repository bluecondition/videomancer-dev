-- Panemix: catalogue combo #105 (seed 20260925) -- split-screen / quad
-- + multi-source mixing (add, screen, multiply, difference, overlay)
-- + false colour (thermal-style LUT).  Two knobs each; the switches and
-- the slider decide how they combine.
--
--   K1 Panes    1 / 2 / 3 / 4 panes
--   K2 Wipe     pane boundaries slide across the screen
--   K3 Blend    add / screen / multiply / difference / overlay
--   K4 Mix      0 .. 100% of the blend between the panes and the picture
--   K5 Palette  thermal / ice / plasma / x-ray
--   K6 Range    gain into the palette (contrast)
--
--   S7  Mirror  odd panes mirrored
--   S8  Luma    palette luma / the blend's own luma kept
--   S9  Index   palette indexed by the blend luma / by |pane - picture|
--   S10 Invert  palette reversed
--   S11 Iso     black isotherm lines at palette stop boundaries
--   P12 Heat    the palette cycles with time; speed = slider
--
-- Operand A = the pane-mapped picture (line buffer, a line late), operand
-- B = the picture itself; two 10x10 products give all five blend modes;
-- the result indexes a 4 x 64 palette ROM.  Latency 25 clocks, all modes,
-- blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture panemix of program_top is

    constant C_LAT : integer := 25;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

    function f_clamp959(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 959 then return to_unsigned(959, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

    type t_pal16 is array (0 to 255) of std_logic_vector(15 downto 0);
    constant C_PAL_YU : t_pal16 := (
        x"1080", x"1180", x"1280", x"1381", x"1581", x"1681", x"1781", x"1882",
        x"1982", x"1B82", x"1C83", x"1E85", x"2188", x"248C", x"2790", x"2A94",
        x"2D97", x"309B", x"339F", x"36A3", x"39A6", x"3CAA", x"3DAF", x"3EB2",
        x"3FB6", x"40BA", x"41BE", x"42C3", x"43C6", x"44CB", x"45CF", x"46D2",
        x"49D5", x"4FD4", x"54D3", x"59D2", x"5FD2", x"64D2", x"69D1", x"6ED0",
        x"74D0", x"79CF", x"7ECE", x"85CA", x"8BC5", x"91C1", x"98BC", x"9EB8",
        x"A4B3", x"AAAF", x"B0AA", x"B6A6", x"BDA1", x"C29D", x"C69A", x"CA97",
        x"CE94", x"D291", x"D68F", x"DB8B", x"DE89", x"E286", x"E782", x"EB80",
        x"1080", x"117E", x"137E", x"147C", x"157B", x"177A", x"1979", x"1A78",
        x"1C77", x"1D76", x"1F75", x"2074", x"2272", x"2471", x"286E", x"2B6C",
        x"2F69", x"3266", x"3663", x"3A61", x"3E5E", x"415B", x"4559", x"4956",
        x"4C54", x"5051", x"554F", x"5C4E", x"614C", x"684B", x"6E49", x"7448",
        x"7946", x"8045", x"8644", x"8C42", x"9241", x"983F", x"9E3F", x"A242",
        x"A746", x"AC4A", x"B04D", x"B551", x"BA54", x"BE58", x"C35C", x"C85F",
        x"CC63", x"D166", x"D66A", x"D86C", x"DA6E", x"DB6F", x"DD71", x"DF72",
        x"E074", x"E176", x"E377", x"E579", x"E67B", x"E77C", x"E97E", x"EB80",
        x"2379", x"257C", x"277F", x"2982", x"2B85", x"2D88", x"2F8B", x"318F",
        x"3392", x"3595", x"3698", x"399B", x"3B9E", x"3DA1", x"3EA4", x"40A8",
        x"43AA", x"46AB", x"49AC", x"4CAC", x"4FAD", x"53AE", x"56AF", x"59B0",
        x"5CB0", x"5FB1", x"62B2", x"66B3", x"69B4", x"6CB5", x"6FB5", x"72B6",
        x"75B6", x"78B6", x"7BB6", x"7EB6", x"80B5", x"83B5", x"86B4", x"89B4",
        x"8CB4", x"8FB4", x"92B3", x"94B3", x"97B2", x"9AB2", x"9DB2", x"A0B1",
        x"A3AF", x"A6AD", x"A9AB", x"ACA8", x"AEA6", x"B1A3", x"B4A1", x"B79E",
        x"BA9C", x"BD9A", x"C097", x"C395", x"C693", x"C890", x"CB8E", x"CE8B",
        x"DD7A", x"D87A", x"D279", x"CD79", x"C778", x"C278", x"BD77", x"B877",
        x"B276", x"AD76", x"A775", x"A275", x"9D74", x"9774", x"9273", x"8D73",
        x"8873", x"8373", x"7E73", x"7973", x"7573", x"7074", x"6B74", x"6674",
        x"6275", x"5D75", x"5876", x"5376", x"4F76", x"4A77", x"4577", x"4077",
        x"3D77", x"3A78", x"3878", x"3678", x"3478", x"3279", x"2F7A", x"2D7A",
        x"2B7B", x"297B", x"277B", x"257C", x"227C", x"207C", x"1E7D", x"1C7D",
        x"1B7E", x"1A7E", x"197E", x"197E", x"187E", x"177E", x"167E", x"157F",
        x"147F", x"147F", x"137F", x"137F", x"127F", x"117F", x"117F", x"1080"
    );
    constant C_PAL_V : t_pal16 := (
        x"0080", x"0083", x"0086", x"0089", x"008C", x"008F", x"0092", x"0095",
        x"0098", x"009B", x"009E", x"00A0", x"00A2", x"00A2", x"00A4", x"00A5",
        x"00A6", x"00A7", x"00A8", x"00A9", x"00AA", x"00AB", x"00A6", x"00A1",
        x"009B", x"0096", x"0091", x"008B", x"0086", x"0081", x"007B", x"0076",
        x"0071", x"006C", x"0067", x"0062", x"005D", x"0058", x"0053", x"004E",
        x"004A", x"0045", x"0040", x"003C", x"0038", x"0035", x"0031", x"002E",
        x"002A", x"0026", x"0023", x"001F", x"001C", x"001F", x"0028", x"0032",
        x"003C", x"0045", x"004F", x"0059", x"0062", x"006C", x"0076", x"0080",
        x"0080", x"0082", x"0085", x"0087", x"008B", x"008D", x"0090", x"0092",
        x"0095", x"0098", x"009A", x"009D", x"00A0", x"00A2", x"00A4", x"00A7",
        x"00A9", x"00AB", x"00AD", x"00AF", x"00B2", x"00B4", x"00B6", x"00B8",
        x"00BB", x"00BD", x"00BC", x"00BB", x"00BA", x"00B8", x"00B7", x"00B5",
        x"00B5", x"00B3", x"00B1", x"00B0", x"00AF", x"00AE", x"00AC", x"00A9",
        x"00A7", x"00A4", x"00A1", x"009E", x"009C", x"0099", x"0097", x"0094",
        x"0091", x"008F", x"008C", x"008A", x"0089", x"0088", x"0087", x"0087",
        x"0086", x"0085", x"0084", x"0083", x"0082", x"0082", x"0081", x"0080",
        x"00B7", x"00B7", x"00B7", x"00B7", x"00B7", x"00B6", x"00B7", x"00B7",
        x"00B7", x"00B6", x"00B6", x"00B6", x"00B6", x"00B6", x"00B6", x"00B6",
        x"00B5", x"00B2", x"00AE", x"00AB", x"00A8", x"00A4", x"00A1", x"009E",
        x"009A", x"0097", x"0094", x"0090", x"008D", x"008A", x"0086", x"0083",
        x"0080", x"007D", x"0079", x"0076", x"0072", x"006F", x"006B", x"0068",
        x"0064", x"0061", x"005E", x"005A", x"0057", x"0053", x"0050", x"004D",
        x"004A", x"0047", x"0045", x"0042", x"003F", x"003D", x"003A", x"0037",
        x"0034", x"0032", x"002F", x"002D", x"002A", x"0028", x"0025", x"0022",
        x"0088", x"0089", x"008A", x"008C", x"008D", x"008F", x"0090", x"0091",
        x"0093", x"0094", x"0095", x"0097", x"0098", x"009A", x"009B", x"009C",
        x"009D", x"009D", x"009D", x"009D", x"009D", x"009C", x"009C", x"009C",
        x"009C", x"009C", x"009C", x"009C", x"009C", x"009C", x"009C", x"009B",
        x"009B", x"009A", x"0099", x"0098", x"0097", x"0096", x"0095", x"0095",
        x"0094", x"0093", x"0092", x"0091", x"0090", x"008F", x"008E", x"008D",
        x"008C", x"008B", x"008B", x"0089", x"0089", x"0088", x"0087", x"0086",
        x"0086", x"0084", x"0084", x"0083", x"0082", x"0081", x"0081", x"0080"
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_panes, s_k_wipe, s_k_blend, s_k_mix, s_k_pal, s_k_range : unsigned(9 downto 0);
    signal s_sw_mirror, s_sw_keepy, s_sw_didx, s_sw_inv, s_sw_iso : std_logic;
    signal s_p_heat : unsigned(9 downto 0);

    signal s_n : unsigned(2 downto 0) := "001";
    signal s_mode : unsigned(2 downto 0) := (others => '0');
    signal s_mix : unsigned(7 downto 0) := (others => '0');
    signal s_zone : unsigned(1 downto 0) := (others => '0');
    signal s_gain : unsigned(7 downto 0) := (others => '0');
    signal s_phase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep : unsigned(2 downto 0) := "111";
    signal s_w3p : unsigned(26 downto 0) := (others => '0');
    signal s_pw, s_b1, s_b2, s_b3 : unsigned(10 downto 0) := (others => '0');
    signal s_wipe : unsigned(20 downto 0) := (others => '0');
    signal s_wipex : unsigned(10 downto 0) := (others => '0');
    signal s_w2m1 : signed(12 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    type t_x is array (1 to 2) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 6) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_xw2 : unsigned(11 downto 0) := (others => '0');
    signal s_xw3 : unsigned(10 downto 0) := (others => '0');
    signal s_pane4 : unsigned(1 downto 0) := (others => '0');
    signal s_pst4, s_xw4 : unsigned(10 downto 0) := (others => '0');
    signal s_xp4 : unsigned(10 downto 0) := (others => '0');
    signal s_xm5 : signed(12 downto 0) := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg : std_logic := '0';
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    type t_c10 is array (2 to 16) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_c10 := (others => (others => '0'));
    signal s_a9, s_b9 : unsigned(9 downto 0) := (others => '0');
    signal s_ua9, s_va9 : unsigned(9 downto 0) := (others => '0');
    signal s_p10, s_q10 : unsigned(19 downto 0) := (others => '0');
    signal s_sum10 : unsigned(10 downto 0) := (others => '0');
    signal s_dif10 : signed(10 downto 0) := (others => '0');
    signal s_a10 : unsigned(9 downto 0) := (others => '0');
    signal s_c_add11, s_c_scr11, s_c_mul11, s_c_dif11, s_c_ovl11, s_ovh11 : signed(11 downto 0) := (others => '0');
    signal s_alo11 : std_logic := '0';
    signal s_sel12 : signed(11 downto 0) := (others => '0');
    signal s_t13 : unsigned(9 downto 0) := (others => '0');
    type t_d10 is array (12 to 17) of unsigned(9 downto 0);
    signal s_dfd : t_d10 := (others => (others => '0'));
    type t_c10b is array (10 to 13) of unsigned(9 downto 0);
    signal s_uad, s_vad : t_c10b := (others => (others => '0'));
    signal s_dy14, s_du14, s_dv14 : signed(10 downto 0) := (others => '0');
    signal s_py15, s_pu15, s_pv15 : signed(19 downto 0) := (others => '0');
    signal s_my16, s_mu16, s_mv16 : unsigned(9 downto 0) := (others => '0');
    -- palette
    signal s_src17 : signed(11 downto 0) := (others => '0');
    signal s_p18 : signed(20 downto 0) := (others => '0');
    signal s_i19 : signed(12 downto 0) := (others => '0');
    signal s_idx20 : unsigned(9 downto 0) := (others => '0');
    signal s_pal_addr : unsigned(7 downto 0) := (others => '0');
    signal s_iso21, s_iso22, s_iso23 : std_logic := '0';
    signal s_pyu_q, s_pv_q, s_pyu_q2, s_pv_q2 : std_logic_vector(15 downto 0) := (others => '0');
    type t_m10 is array (17 to 23) of unsigned(9 downto 0);
    signal s_myd, s_mud, s_mvd : t_m10 := (others => (others => '0'));
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := (others => '0');
    signal s_y25, s_u25, s_v25 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_panes <= unsigned(registers_in(0));
    s_k_wipe  <= unsigned(registers_in(1));
    s_k_blend <= unsigned(registers_in(2));
    s_k_mix   <= unsigned(registers_in(3));
    s_k_pal   <= unsigned(registers_in(4));
    s_k_range <= unsigned(registers_in(5));
    s_sw_mirror <= registers_in(6)(0);
    s_sw_keepy  <= registers_in(6)(1);
    s_sw_didx   <= registers_in(6)(2);
    s_sw_inv    <= registers_in(6)(3);
    s_sw_iso    <= registers_in(6)(4);
    s_p_heat <= unsigned(registers_in(7));

    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_n <= resize(s_k_panes(9 downto 8), 3) + 1;
            case to_integer(s_k_blend(9 downto 7)) is
                when 0 | 1 => s_mode <= "000";
                when 2 | 3 => s_mode <= "001";
                when 4     => s_mode <= "010";
                when 5 | 6 => s_mode <= "011";
                when others => s_mode <= "100";
            end case;
            s_mix <= s_k_mix(9 downto 2);
            s_zone <= s_k_pal(9 downto 8);
            s_gain <= s_k_range(9 downto 2);
            s_w2m1 <= signed(resize(s_line_width & '0', 13)) - 1;

            case to_integer(s_vstep) is
                when 0 =>
                    s_w3p <= s_line_width * to_unsigned(21845, 16);
                    s_wipe <= s_k_wipe * s_line_width;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    case to_integer(s_n) is
                        when 1 => s_pw <= s_line_width;
                        when 2 => s_pw <= '0' & s_line_width(10 downto 1);
                        when 3 => s_pw <= s_w3p(26 downto 16);
                        when others => s_pw <= "00" & s_line_width(10 downto 2);
                    end case;
                    s_wipex <= s_wipe(20 downto 10);
                    s_vstep <= s_vstep + 1;
                when 2 => s_b1 <= s_pw; s_b2 <= s_pw + s_pw; s_vstep <= s_vstep + 1;
                when 3 => s_b3 <= s_b2 + s_pw; s_vstep <= "111";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then
                    s_phase <= s_phase + resize(s_p_heat(9 downto 2), 16);
                    s_vstep <= (others => '0');
                end if;
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
        variable v_p : unsigned(1 downto 0);
        variable v_st : unsigned(10 downto 0);
        variable n, r : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            s_x(2) <= s_x(1);
            for i in 2 to 6 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- 3: wiped x
            s_xw2 <= resize(s_x(1), 12) + resize(s_wipex, 12);
            if s_xw2 >= resize(s_line_width, 12) then s_xw3 <= resize(s_xw2 - resize(s_line_width, 12), 11); else s_xw3 <= s_xw2(10 downto 0); end if;
            -- 4: pane
            if s_xw3 >= s_b3 and s_n = 4 then v_p := "11"; v_st := s_b3;
            elsif s_xw3 >= s_b2 and s_n >= 3 then v_p := "10"; v_st := s_b2;
            elsif s_xw3 >= s_b1 and s_n >= 2 then v_p := "01"; v_st := s_b1;
            else v_p := "00"; v_st := (others => '0'); end if;
            s_pane4 <= v_p; s_pst4 <= v_st; s_xw4 <= s_xw3; s_xp4 <= s_xw3 - v_st;
            -- 5: mirrored panes
            if s_sw_mirror = '1' and s_pane4(0) = '1' then
                s_xm5 <= signed(resize(s_pst4, 13)) + signed(resize(s_pw, 13)) - 1 - signed(resize(s_xp4, 13));
            else
                s_xm5 <= signed(resize(s_xw4, 13));
            end if;
            -- 6: edges (mirrored)
            n := -s_xm5; r := s_w2m1 - s_xm5;
            if s_avd(5) = '0' then s_bg <= '1'; s_addr <= (others => '0');
            elsif s_xm5 < 0 then
                if n < signed(resize(s_line_width, 13)) then s_bg <= '0'; s_addr <= unsigned(n(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            elsif s_xm5 >= signed(resize(s_line_width, 13)) then
                if r >= 0 then s_bg <= '0'; s_addr <= unsigned(r(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            else s_bg <= '0'; s_addr <= unsigned(s_xm5(10 downto 0)); end if;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_roms : process(clk)
    begin
        if rising_edge(clk) then
            s_pyu_q <= C_PAL_YU(to_integer(s_pal_addr));
            s_pv_q  <= C_PAL_V(to_integer(s_pal_addr));
            s_pyu_q2 <= s_pyu_q; s_pv_q2 <= s_pv_q;
        end if;
    end process;

    p_pix : process(clk)
        variable v_i : signed(12 downto 0);
        variable v_idx : unsigned(9 downto 0);
        variable v_y, v_u, v_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_yd(2) <= s_in_y; s_ud(2) <= s_in_u; s_vd(2) <= s_in_v;
            for i in 3 to 16 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            -- 7 (rd) / 8: operand A fetch
            s_a_bg <= s_bg; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 9: black-relative operands
            if s_wet_y < 64 then s_a9 <= (others => '0'); else s_a9 <= s_wet_y - 64; end if;
            if s_yd(8) < 64 then s_b9 <= (others => '0'); else s_b9 <= s_yd(8) - 64; end if;
            s_ua9 <= s_wet_u; s_va9 <= s_wet_v;
            -- 10: products
            s_p10 <= s_a9 * s_b9;
            s_q10 <= (959 - s_a9) * (959 - s_b9);
            s_sum10 <= resize(s_a9, 11) + resize(s_b9, 11);
            s_dif10 <= signed(resize(s_a9, 11)) - signed(resize(s_b9, 11));
            s_a10 <= s_a9;
            s_uad(10) <= s_ua9; s_vad(10) <= s_va9;
            for i in 11 to 13 loop s_uad(i) <= s_uad(i - 1); s_vad(i) <= s_vad(i - 1); end loop;
            -- 11: candidates
            s_c_add11 <= signed(resize(s_sum10, 12));
            s_c_scr11 <= signed(resize(s_sum10, 12)) - signed(resize(s_p10(19 downto 10), 12));
            s_c_mul11 <= signed(resize(s_p10(19 downto 10), 12));
            if s_dif10 < 0 then s_c_dif11 <= -resize(s_dif10, 12); else s_c_dif11 <= resize(s_dif10, 12); end if;
            s_c_ovl11 <= signed(resize(s_p10(19 downto 9), 12));
            s_ovh11 <= to_signed(959, 12) - signed(resize(s_q10(19 downto 9), 12));
            if s_a10 < 480 then s_alo11 <= '1'; else s_alo11 <= '0'; end if;
            -- 12: select; the difference carried for the palette index
            case to_integer(s_mode) is
                when 0 => s_sel12 <= s_c_add11;
                when 1 => s_sel12 <= s_c_scr11;
                when 2 => s_sel12 <= s_c_mul11;
                when 3 => s_sel12 <= s_c_dif11;
                when others => if s_alo11 = '1' then s_sel12 <= s_c_ovl11; else s_sel12 <= s_ovh11; end if;
            end case;
            s_dfd(12) <= unsigned(s_c_dif11(9 downto 0));
            for i in 13 to 17 loop s_dfd(i) <= s_dfd(i - 1); end loop;
            -- 13: clamp
            s_t13 <= f_clamp959(s_sel12);
            -- 14: differences toward the blend
            if s_yd(13) < 64 then v_y := (others => '0'); else v_y := s_yd(13) - 64; end if;
            s_dy14 <= signed(resize(s_t13, 11)) - signed(resize(v_y, 11));
            s_du14 <= signed(resize(s_uad(13), 11)) - signed(resize(s_ud(13), 11));
            s_dv14 <= signed(resize(s_vad(13), 11)) - signed(resize(s_vd(13), 11));
            -- 15: mix products
            s_py15 <= s_dy14 * signed(resize(s_mix, 9));
            s_pu15 <= s_du14 * signed(resize(s_mix, 9));
            s_pv15 <= s_dv14 * signed(resize(s_mix, 9));
            -- 16: mixed pixel
            s_my16 <= f_clamp10(signed(resize(s_yd(15), 13)) + resize(shift_right(s_py15, 8), 13));
            s_mu16 <= f_clamp10(signed(resize(s_ud(15), 13)) + resize(shift_right(s_pu15, 8), 13));
            s_mv16 <= f_clamp10(signed(resize(s_vd(15), 13)) + resize(shift_right(s_pv15, 8), 13));
            -- 17: palette index source
            if s_sw_didx = '1' then s_src17 <= signed(resize(s_dfd(16), 12)); else s_src17 <= signed(resize(s_my16, 12)); end if;
            s_myd(17) <= s_my16; s_mud(17) <= s_mu16; s_mvd(17) <= s_mv16;
            for i in 18 to 23 loop s_myd(i) <= s_myd(i - 1); s_mud(i) <= s_mud(i - 1); s_mvd(i) <= s_mvd(i - 1); end loop;
            -- 18: gain
            s_p18 <= s_src17 * signed(resize(s_gain, 9));
            -- 19: heat phase
            s_i19 <= resize(shift_right(s_p18, 7), 13) + signed(resize(s_phase(15 downto 6), 13));
            -- 20: wrap / invert
            v_i := s_i19;
            if v_i < 0 then v_i := v_i + 1024; end if;
            if v_i > 1023 then v_i := v_i - 1024; end if;
            v_idx := unsigned(v_i(9 downto 0));
            if s_sw_inv = '1' then v_idx := not v_idx; end if;
            s_idx20 <= v_idx;
            -- 21: ROM address, iso flag
            s_pal_addr <= s_zone & s_idx20(9 downto 4);
            if s_sw_iso = '1' and s_idx20(6 downto 0) < 6 then s_iso21 <= '1'; else s_iso21 <= '0'; end if;
            -- 22 rom, 23 fabric; 24: unpack
            s_iso22 <= s_iso21; s_iso23 <= s_iso22;
            v_y := unsigned(s_pyu_q2(15 downto 8)) & "00";
            v_u := unsigned(s_pyu_q2(7 downto 0)) & "00";
            v_v := unsigned(s_pv_q2(7 downto 0)) & "00";
            if s_sw_keepy = '1' then v_y := s_myd(23); end if;
            if s_iso23 = '1' then v_y := to_unsigned(64, 10); end if;
            s_y24 <= v_y; s_u24 <= v_u; s_v24 <= v_v;
            -- 25: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture panemix;
