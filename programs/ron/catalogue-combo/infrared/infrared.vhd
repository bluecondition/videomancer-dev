-- Infrared: catalogue combo #10 (seed 20260912) -- perspective keystone /
-- corner-pin + false-colour (thermal-style LUT).  Three knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Keystone  bipolar; rows scale linearly with height about the centre
--   K2 Skew      bipolar parallelogram shear
--   K3 Zoom      overall horizontal scale 0.25x .. 4x (log)
--   K4 Palette   thermal / ice / plasma / x-ray (4 zones)
--   K5 Range     luma gain into the palette (contrast)
--   K6 Offset    luma bias into the palette (sweeps the ramp)
--
--   S7  Rows     palette offset drifts with the row (heat falls with distance)
--   S8  Iso      black isotherm lines at palette stop boundaries
--   S9  Pins     chroma keystoned with the mirrored trapezoid
--   S10 Luma     palette replaces luma too / only chroma comes from the palette
--   S11 Invert   palette reversed
--   P12 Heat     the palette cycles with time; speed = slider
--
-- Keystone: kinescope's address engine (u = cx + (x-cx)*k_row + shift_row).
-- False colour: idx = clamp((y-64)*range + offset + row + time) >> 2 into a
-- 4 x 64 entry palette ROM (EBR).  Latency 14 clocks, all modes,
-- blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture infrared of program_top is

    constant C_LAT : integer := 14;

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

    type t_zoom_rom is array (0 to 63) of unsigned(12 downto 0);
    constant C_ZOOM_ROM : t_zoom_rom := (
        to_unsigned(256, 13), to_unsigned(267, 13), to_unsigned(279, 13), to_unsigned(292, 13), to_unsigned(304, 13), to_unsigned(318, 13), to_unsigned(332, 13), to_unsigned(347, 13),
        to_unsigned(362, 13), to_unsigned(378, 13), to_unsigned(395, 13), to_unsigned(412, 13), to_unsigned(431, 13), to_unsigned(450, 13), to_unsigned(470, 13), to_unsigned(490, 13),
        to_unsigned(512, 13), to_unsigned(535, 13), to_unsigned(558, 13), to_unsigned(583, 13), to_unsigned(609, 13), to_unsigned(636, 13), to_unsigned(664, 13), to_unsigned(693, 13),
        to_unsigned(724, 13), to_unsigned(756, 13), to_unsigned(790, 13), to_unsigned(825, 13), to_unsigned(861, 13), to_unsigned(899, 13), to_unsigned(939, 13), to_unsigned(981, 13),
        to_unsigned(1024, 13), to_unsigned(1069, 13), to_unsigned(1117, 13), to_unsigned(1166, 13), to_unsigned(1218, 13), to_unsigned(1272, 13), to_unsigned(1328, 13), to_unsigned(1387, 13),
        to_unsigned(1448, 13), to_unsigned(1512, 13), to_unsigned(1579, 13), to_unsigned(1649, 13), to_unsigned(1722, 13), to_unsigned(1798, 13), to_unsigned(1878, 13), to_unsigned(1961, 13),
        to_unsigned(2048, 13), to_unsigned(2139, 13), to_unsigned(2233, 13), to_unsigned(2332, 13), to_unsigned(2435, 13), to_unsigned(2543, 13), to_unsigned(2656, 13), to_unsigned(2774, 13),
        to_unsigned(2896, 13), to_unsigned(3025, 13), to_unsigned(3158, 13), to_unsigned(3298, 13), to_unsigned(3444, 13), to_unsigned(3597, 13), to_unsigned(3756, 13), to_unsigned(3922, 13)
    );

    --------------------------------------------------------------------------
    -- Line buffers
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_pyu_q, s_pv_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s_pal_addr : unsigned(7 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Controls / per-frame
    --------------------------------------------------------------------------
    signal s_k_key, s_k_skew, s_k_zoom, s_k_pal, s_k_range, s_k_off : unsigned(9 downto 0);
    signal s_sw_rows, s_sw_iso, s_sw_pins, s_sw_luma, s_sw_inv : std_logic;
    signal s_p_heat : unsigned(9 downto 0);
    signal s_vstep   : unsigned(2 downto 0) := "111";
    signal s_div_rem : unsigned(11 downto 0) := (others => '0');
    signal s_div_q   : unsigned(20 downto 0) := (others => '0');
    signal s_div_i   : unsigned(4 downto 0) := (others => '0');
    signal s_inv_h   : unsigned(13 downto 0) := to_unsigned(3883, 14);
    signal s_key     : signed(10 downto 0) := (others => '0');
    signal s_skew    : signed(10 downto 0) := (others => '0');
    signal s_zoom    : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_zone    : unsigned(1 downto 0) := (others => '0');
    signal s_range   : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_offs    : signed(9 downto 0) := (others => '0');
    signal s_heat    : unsigned(9 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    --------------------------------------------------------------------------
    -- Tracking
    --------------------------------------------------------------------------
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line       : unsigned(10 downto 0) := (others => '0');
    signal s_height     : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_half_w     : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-line sequencer (keystone)
    --------------------------------------------------------------------------
    signal s_lstep   : unsigned(3 downto 0) := "1111";
    signal s_tph, s_tpl : unsigned(17 downto 0) := (others => '0');
    signal s_tp      : unsigned(24 downto 0) := (others => '0');
    signal s_t       : unsigned(10 downto 0) := (others => '0');
    signal s_tc      : signed(10 downto 0) := (others => '0');
    signal s_kth, s_skh : signed(16 downto 0) := (others => '0');
    signal s_ktl, s_skl : signed(16 downto 0) := (others => '0');
    signal s_kt, s_sk : signed(21 downto 0) := (others => '0');
    signal s_f1, s_f2 : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_shift   : signed(10 downto 0) := (others => '0');
    signal s_k1h, s_k2h, s_k1l, s_k2l : unsigned(18 downto 0) := (others => '0');
    signal s_k1p, s_k2p : unsigned(24 downto 0) := (others => '0');
    signal s_k1q, s_k2q : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_p1h, s_p2h : unsigned(17 downto 0) := (others => '0');
    signal s_p1l, s_p2l : unsigned(16 downto 0) := (others => '0');
    signal s_p1, s_p2 : unsigned(23 downto 0) := (others => '0');
    signal s_ia1, s_ia2, s_ib1, s_ib2 : signed(24 downto 0) := (others => '0');
    signal s_init1, s_init2 : signed(24 downto 0) := (others => '0');
    signal s_row_off : signed(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel pipeline
    --------------------------------------------------------------------------
    signal s_acc1, s_acc2 : signed(24 downto 0) := (others => '0');
    signal s_u2, s_uc2 : signed(13 downto 0) := (others => '0');
    signal s_addr_y, s_addr_c : unsigned(10 downto 0) := (others => '0');
    signal s_bgy, s_bgc : std_logic := '0';
    signal s_a_bgy, s_a_bgc, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_ym6   : signed(10 downto 0) := (others => '0');
    signal s_gp7   : signed(19 downto 0) := (others => '0');
    signal s_idx8  : unsigned(9 downto 0) := (others => '0');
    signal s_iso9  : std_logic := '0';
    type t_y is array (6 to 12) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_y := (others => (others => '0'));
    type t_b is array (9 to 12) of std_logic;
    signal s_isod : t_b := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_key   <= unsigned(registers_in(0));
    s_k_skew  <= unsigned(registers_in(1));
    s_k_zoom  <= unsigned(registers_in(2));
    s_k_pal   <= unsigned(registers_in(3));
    s_k_range <= unsigned(registers_in(4));
    s_k_off   <= unsigned(registers_in(5));
    s_sw_rows <= registers_in(6)(0);
    s_sw_iso  <= registers_in(6)(1);
    s_sw_pins <= registers_in(6)(2);
    s_sw_luma <= registers_in(6)(3);
    s_sw_inv  <= registers_in(6)(4);
    s_p_heat  <= unsigned(registers_in(7));

    p_roms : process(clk)
    begin
        if rising_edge(clk) then
            s_pyu_q <= C_PAL_YU(to_integer(s_pal_addr));
            s_pv_q  <= C_PAL_V(to_integer(s_pal_addr));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage 1 + sequencers
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_rem : unsigned(12 downto 0);
        variable v_nb  : unsigned(0 downto 0);
        variable v_ktq : signed(13 downto 0);
        variable v_f   : signed(13 downto 0);
        variable v_k1  : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw_active <= '1';
            else
                s_rx <= (others => '0');
            end if;
            if s_in_avid = '1' then
                s_wr_x <= s_wr_x + 1;
            else
                s_wr_x <= (others => '0');
            end if;
            s_we <= data_in.avid;

            if data_in.avid = '1' then
                s_acc1 <= s_acc1 + signed(resize(s_k1q, 25));
                s_acc2 <= s_acc2 + signed(resize(s_k2q, 25));
            else
                s_acc1 <= s_init1;
                s_acc2 <= s_init2;
            end if;

            ------------------------------------------------------------------
            -- Per-line sequencer (keystone rows)
            ------------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>
                    s_tph <= s_line * s_inv_h(13 downto 7);
                    s_tpl <= s_line * s_inv_h(6 downto 0);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_tp <= shift_left(resize(s_tph, 25), 7) + resize(s_tpl, 25);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    if s_tp(24 downto 20) /= 0 then
                        s_t <= to_unsigned(1024, 11);
                    else
                        s_t <= s_tp(20 downto 10);
                    end if;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_tc <= to_signed(512, 11) - signed(s_t);
                    if s_sw_rows = '1' then
                        s_row_off <= signed(s_t(10 downto 1));
                    else
                        s_row_off <= (others => '0');
                    end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_kth <= s_key * s_tc(10 downto 5);
                    s_ktl <= s_key * signed('0' & s_tc(4 downto 0));
                    s_skh <= s_skew * s_tc(10 downto 5);
                    s_skl <= s_skew * signed('0' & s_tc(4 downto 0));
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_kt <= shift_left(resize(s_kth, 22), 5) + resize(s_ktl, 22);
                    s_sk <= shift_left(resize(s_skh, 22), 5) + resize(s_skl, 22);
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    v_ktq := resize(shift_right(s_kt, 8), 14);
                    v_f := to_signed(1024, 14) + v_ktq;
                    if v_f < 128 then v_f := to_signed(128, 14); end if;
                    s_f1 <= unsigned(v_f(11 downto 0));
                    v_f := to_signed(1024, 14) - v_ktq;
                    if v_f < 128 then v_f := to_signed(128, 14); end if;
                    s_f2 <= unsigned(v_f(11 downto 0));
                    s_shift <= resize(shift_right(s_sk, 9), 11);
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_k1h <= s_zoom * s_f1(11 downto 6);
                    s_k1l <= s_zoom * s_f1(5 downto 0);
                    s_k2h <= s_zoom * s_f2(11 downto 6);
                    s_k2l <= s_zoom * s_f2(5 downto 0);
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    s_k1p <= shift_left(resize(s_k1h, 25), 6) + resize(s_k1l, 25);
                    s_k2p <= shift_left(resize(s_k2h, 25), 6) + resize(s_k2l, 25);
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    v_k1 := s_k1p(24 downto 10);
                    if v_k1 < 64 then s_k1q <= to_unsigned(64, 13);
                    elsif v_k1 > 8191 then s_k1q <= to_unsigned(8191, 13);
                    else s_k1q <= v_k1(12 downto 0); end if;
                    v_k1 := s_k2p(24 downto 10);
                    if v_k1 < 64 then s_k2q <= to_unsigned(64, 13);
                    elsif v_k1 > 8191 then s_k2q <= to_unsigned(8191, 13);
                    else s_k2q <= v_k1(12 downto 0); end if;
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    s_p1h <= s_half_w * s_k1q(12 downto 6);
                    s_p1l <= s_half_w * s_k1q(5 downto 0);
                    s_p2h <= s_half_w * s_k2q(12 downto 6);
                    s_p2l <= s_half_w * s_k2q(5 downto 0);
                    s_lstep <= s_lstep + 1;
                when 11 =>
                    s_p1 <= shift_left(resize(s_p1h, 24), 6) + resize(s_p1l, 24);
                    s_p2 <= shift_left(resize(s_p2h, 24), 6) + resize(s_p2l, 24);
                    s_ib1 <= shift_left(resize(s_shift, 25), 10) - signed(resize(s_k1q, 25));
                    s_ib2 <= -shift_left(resize(s_shift, 25), 10) - signed(resize(s_k2q, 25));
                    s_lstep <= s_lstep + 1;
                when 12 =>
                    s_ia1 <= signed(shift_left(resize(s_half_w, 25), 10)) - signed(resize(s_p1, 25));
                    s_ia2 <= signed(shift_left(resize(s_half_w, 25), 10)) - signed(resize(s_p2, 25));
                    s_lstep <= s_lstep + 1;
                when 13 =>
                    s_init1 <= s_ia1 + s_ib1;
                    if s_sw_pins = '1' then
                        s_init2 <= s_ia2 + s_ib2;
                    else
                        s_init2 <= s_ia1 + s_ib1;
                    end if;
                    s_lstep <= s_lstep + 1;
                when others =>
                    null;
            end case;

            ------------------------------------------------------------------
            -- Vblank sequencer
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_half_w <= resize(s_line_width(10 downto 1), 11);
                    s_key    <= signed(resize(s_k_key, 11)) - to_signed(512, 11);
                    s_skew   <= signed(resize(s_k_skew, 11)) - to_signed(512, 11);
                    s_zoom   <= C_ZOOM_ROM(to_integer(s_k_zoom(9 downto 4)));
                    s_zone   <= s_k_pal(9 downto 8);
                    s_range  <= s_k_range(9 downto 2);
                    s_offs   <= signed(resize(s_k_off(9 downto 1), 10)) - to_signed(256, 10);
                    s_heat   <= s_heat + s_p_heat(9 downto 4);
                    s_div_rem <= (others => '0');
                    s_div_q   <= (others => '0');
                    s_div_i   <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    if s_div_i = 0 then v_nb := "1"; else v_nb := "0"; end if;
                    v_rem := s_div_rem & v_nb(0);
                    if v_rem >= resize(s_height, 13) then
                        s_div_rem <= resize(v_rem - resize(s_height, 13), 12);
                        s_div_q   <= s_div_q(19 downto 0) & '1';
                    else
                        s_div_rem <= v_rem(11 downto 0);
                        s_div_q   <= s_div_q(19 downto 0) & '0';
                    end if;
                    if s_div_i = 20 then
                        s_vstep <= s_vstep + 1;
                    end if;
                    s_div_i <= s_div_i + 1;
                when 2 =>
                    if s_height < 16 then
                        s_inv_h <= (others => '0');
                    else
                        s_inv_h <= s_div_q(13 downto 0);
                    end if;
                    s_vstep <= "111";
                when others =>
                    null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_line   <= s_line + 1;
                s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar  <= not s_wpar;
                s_rbank <= s_wpar;
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_wpar <= '0';
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Address chain 2..3
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_w : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            s_u2  <= s_acc1(23 downto 10);
            s_uc2 <= s_acc2(23 downto 10);
            v_w := signed(resize(s_line_width, 14));
            if s_u2 < 0 or s_u2 >= v_w then s_bgy <= '1'; else s_bgy <= '0'; end if;
            if s_uc2 < 0 or s_uc2 >= v_w then s_bgc <= '1'; else s_bgc <= '0'; end if;
            s_addr_y <= unsigned(s_u2(10 downto 0));
            s_addr_c <= unsigned(s_uc2(10 downto 0));
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (read at stage 4)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr_y));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr_y));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr_c(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr_c(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr_c(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr_c(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 4..14: fetch, false colour, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_i : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 4 context
            s_a_bgy <= s_bgy; s_a_bgc <= s_bgc; s_a_rbank <= s_rbank;
            -- stage 5: select
            if s_a_bgy = '1' then
                s_wet_y <= to_unsigned(64, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
            else
                s_wet_y <= unsigned(s_rdY0);
            end if;
            if s_a_bgc = '1' then
                s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 6: luma minus black
            s_ym6 <= signed('0' & s_wet_y) - 64;
            s_yd(6) <= s_wet_y; s_ud(6) <= s_wet_u; s_vd(6) <= s_wet_v;
            -- stage 7: range product (11 x 9)
            s_gp7 <= s_ym6 * signed('0' & s_range);
            -- stage 8: index = product>>6 + offset + row + heat, clamp 0..255 (x4)
            v_i := resize(shift_right(s_gp7, 6), 13) + resize(s_offs, 13) + resize(s_row_off, 13) + signed(resize(s_heat(9 downto 2), 13));
            if s_sw_inv = '1' then
                v_i := to_signed(1023, 13) - v_i;
            end if;
            if s_p_heat > 0 then
                s_idx8 <= unsigned(v_i(9 downto 0));            -- cycling: wrap
            elsif v_i < 0 then
                s_idx8 <= (others => '0');
            elsif v_i > 1023 then
                s_idx8 <= to_unsigned(1023, 10);
            else
                s_idx8 <= unsigned(v_i(9 downto 0));
            end if;
            -- stage 9: palette ROM address; iso flag
            s_pal_addr <= s_zone & s_idx8(9 downto 4);
            if s_idx8(6 downto 0) < 6 then s_iso9 <= '1'; else s_iso9 <= '0'; end if;
            -- stage 10: ROM read lands (p_roms); stage 11: fabric copy
            s_y11 <= unsigned(s_pyu_q(15 downto 8)) & "00";
            s_u11 <= unsigned(s_pyu_q(7 downto 0)) & "00";
            s_v11 <= unsigned(s_pv_q(7 downto 0)) & "00";
            -- stage 12: luma choice, iso lines
            if s_sw_luma = '1' then
                s_y12 <= s_yd(11);
            else
                s_y12 <= s_y11;
            end if;
            if s_sw_iso = '1' and s_isod(11) = '1' then
                s_y12 <= to_unsigned(64, 10);
                s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            else
                s_u12 <= s_u11; s_v12 <= s_v11;
            end if;
            -- stage 13: pass; stage 14: gate
            s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            if s_avid_sr(12) = '1' then
                s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
            -- chains
            for i in 7 to 12 loop
                s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1);
            end loop;
            s_isod(9) <= s_iso9;
            for i in 10 to 12 loop
                s_isod(i) <= s_isod(i - 1);
            end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y14);
    data_out.u       <= std_logic_vector(s_u14);
    data_out.v       <= std_logic_vector(s_v14);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture infrared;
