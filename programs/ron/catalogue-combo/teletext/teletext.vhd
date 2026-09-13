-- Teletext: catalogue combo #47 (seed 20260920) -- chromatic glitch /
-- datamosh bars + ASCII / dither-art + scanline shear.  Two knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Bars     fraction of glyph bands displaced (datamosh)
--   K2 Distance mosh displacement (up to +-512 px)
--   K3 Cell     glyph cell 8 / 16 / 32 px
--   K4 Contrast density -> glyph mapping (100% = unity, up to 400%)
--   K5 Shear    per-line shear amount (+-30 px)
--   K6 Rate     shear wave rate (Sine) / tear probability (Tear)
--
--   S7  Ink     white glyphs on black / black on white
--   S8  Colour  glyphs mono / glyphs take the cell's chroma
--   S9  Mosh    displaces the video / displaces whole glyph cells
--   S10 Shear   triangle wave / random per-line tear
--   S11 Glyphs  ASCII ramp / block-dither set
--   P12 Reveal  video (moshed, sheared) .. glyph rendering
--
-- Line buffer read a line late with the mosh offset; on each band's first
-- line the cell luma mean and chroma are written to a two-bank cell RAM
-- (read one band later); glyph = 8x8 ROM row indexed by 16 density levels.
-- Shear is applied on the output side through a 64-deep ring so the glyph
-- raster tears too.  Latency 53 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture teletext of program_top is

    constant C_D   : integer := 33;
    constant C_LAT : integer := 20 + C_D;

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

    type t_glyph is array (0 to 255) of std_logic_vector(7 downto 0);
    constant C_GLYPH : t_glyph := (
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"18", x"18", x"00",
        x"00", x"00", x"00", x"00", x"00", x"18", x"18", x"30",
        x"00", x"18", x"18", x"00", x"00", x"18", x"18", x"00",
        x"00", x"18", x"18", x"00", x"00", x"18", x"18", x"30",
        x"00", x"00", x"00", x"7e", x"00", x"00", x"00", x"00",
        x"00", x"00", x"32", x"4c", x"00", x"00", x"00", x"00",
        x"00", x"00", x"7e", x"00", x"7e", x"00", x"00", x"00",
        x"00", x"18", x"18", x"7e", x"7e", x"18", x"18", x"00",
        x"00", x"66", x"3c", x"ff", x"3c", x"66", x"00", x"00",
        x"00", x"00", x"3c", x"66", x"66", x"66", x"3c", x"00",
        x"00", x"00", x"66", x"3c", x"18", x"3c", x"66", x"00",
        x"24", x"24", x"ff", x"24", x"24", x"ff", x"24", x"24",
        x"62", x"66", x"0c", x"18", x"30", x"66", x"46", x"00",
        x"3c", x"66", x"66", x"3c", x"66", x"66", x"3c", x"00",
        x"3c", x"66", x"6e", x"6e", x"6e", x"60", x"3c", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"88", x"00", x"00", x"00", x"88", x"00", x"00", x"00",
        x"88", x"00", x"22", x"00", x"88", x"00", x"22", x"00",
        x"aa", x"00", x"22", x"00", x"aa", x"00", x"22", x"00",
        x"aa", x"00", x"aa", x"00", x"aa", x"00", x"aa", x"00",
        x"aa", x"44", x"aa", x"00", x"aa", x"44", x"aa", x"00",
        x"aa", x"44", x"aa", x"11", x"aa", x"44", x"aa", x"11",
        x"aa", x"55", x"aa", x"11", x"aa", x"55", x"aa", x"11",
        x"aa", x"55", x"aa", x"55", x"aa", x"55", x"aa", x"55",
        x"ee", x"55", x"aa", x"55", x"ee", x"55", x"aa", x"55",
        x"ee", x"55", x"bb", x"55", x"ee", x"55", x"bb", x"55",
        x"ff", x"55", x"bb", x"55", x"ff", x"55", x"bb", x"55",
        x"ff", x"55", x"ff", x"55", x"ff", x"55", x"ff", x"55",
        x"ff", x"dd", x"ff", x"55", x"ff", x"dd", x"ff", x"55",
        x"ff", x"dd", x"ff", x"77", x"ff", x"dd", x"ff", x"77",
        x"ff", x"ff", x"ff", x"ff", x"ff", x"ff", x"ff", x"ff"
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    type t_ring is array (0 to 63) of std_logic_vector(31 downto 0);
    signal ring : t_ring := (others => (others => '0'));
    signal s_rrd : std_logic_vector(31 downto 0) := (others => '0');
    type t_dens is array (0 to 255) of std_logic_vector(31 downto 0);
    signal dens0, dens1 : t_dens := (others => (others => '0'));
    signal s_drd0, s_drd1 : std_logic_vector(31 downto 0) := (others => '0');

    signal s_k_bars, s_k_dist, s_k_cell, s_k_con, s_k_shear, s_k_rate : unsigned(9 downto 0);
    signal s_sw_black, s_sw_colour, s_sw_cells, s_sw_tear, s_sw_blocks : std_logic;
    signal s_p_reveal : unsigned(9 downto 0);

    signal s_bars, s_dist, s_con, s_rate : unsigned(7 downto 0) := (others => '0');
    signal s_amt    : unsigned(4 downto 0) := (others => '0');
    signal s_ss     : unsigned(1 downto 0) := "00";
    signal s_cmask  : unsigned(4 downto 0) := "00111";
    signal s_mix    : unsigned(7 downto 0) := (others => '0');
    signal s_lfsr   : unsigned(15 downto 0) := x"ACE1";
    signal s_saw_active : std_logic := '0';
    signal s_wp     : unsigned(5 downto 0) := (others => '0');
    signal s_tph, s_fph : unsigned(7 downto 0) := (others => '0');

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_oline, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_prev_av43 : std_logic := '0';

    -- per-line
    signal s_lstep : unsigned(2 downto 0) := "111";
    signal s_bl0   : std_logic := '0';
    signal s_tri, s_tear : signed(7 downto 0) := (others => '0');
    signal s_shp   : signed(13 downto 0) := (others => '0');
    signal s_off_row : unsigned(5 downto 0) := to_unsigned(33, 6);
    signal s_mp    : signed(18 downto 0) := (others => '0');
    signal s_mosh_band : signed(10 downto 0) := (others => '0');
    signal s_mosh_src : signed(10 downto 0) := (others => '0');
    signal s_moshc : signed(7 downto 0) := (others => '0');
    signal s_wbank : std_logic := '0';

    type t_x is array (2 to 7) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (2 to 7) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_a3 : signed(11 downto 0) := (others => '0');
    signal s_bg4 : std_logic := '0';
    signal s_addr4 : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_csum : unsigned(14 downto 0) := (others => '0');
    signal s_dwe8 : std_logic := '0';
    signal s_da8 : unsigned(7 downto 0) := (others => '0');
    signal s_dd8 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    -- output side
    signal s_xo7 : signed(11 downto 0) := (others => '0');
    signal s_ra7 : unsigned(5 downto 0) := (others => '0');
    signal s_ci8 : unsigned(7 downto 0) := (others => '0');
    signal s_gcol8 : unsigned(2 downto 0) := (others => '0');
    signal s_bgo8 : std_logic := '0';
    signal s_wy9, s_wu9, s_wv9 : unsigned(9 downto 0) := (others => '0');
    signal s_dens10 : unsigned(9 downto 0) := (others => '0');
    signal s_cu10, s_cv10 : unsigned(7 downto 0) := (others => '0');
    signal s_d11 : unsigned(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(17 downto 0) := (others => '0');
    signal s_romaddr13 : unsigned(7 downto 0) := (others => '0');
    signal s_pat14, s_pat15 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_inky16, s_inku16, s_inkv16 : unsigned(9 downto 0) := (others => '0');
    signal s_dy17, s_du17, s_dv17 : signed(10 downto 0) := (others => '0');
    signal s_py18, s_pu18, s_pv18 : signed(19 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');
    type t_c10 is array (9 to 18) of unsigned(9 downto 0);
    signal s_wy, s_wu, s_wv : t_c10 := (others => (others => '0'));
    type t_c8 is array (10 to 15) of unsigned(7 downto 0);
    signal s_cu, s_cv : t_c8 := (others => (others => '0'));
    type t_g3 is array (8 to 15) of unsigned(2 downto 0);
    signal s_gcol : t_g3 := (others => (others => '0'));
    type t_bo is array (8 to 15) of std_logic;
    signal s_bgo : t_bo := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_bars  <= unsigned(registers_in(0));
    s_k_dist  <= unsigned(registers_in(1));
    s_k_cell  <= unsigned(registers_in(2));
    s_k_con   <= unsigned(registers_in(3));
    s_k_shear <= unsigned(registers_in(4));
    s_k_rate  <= unsigned(registers_in(5));
    s_sw_black  <= registers_in(6)(0);
    s_sw_colour <= registers_in(6)(1);
    s_sw_cells  <= registers_in(6)(2);
    s_sw_tear   <= registers_in(6)(3);
    s_sw_blocks <= registers_in(6)(4);
    s_p_reveal <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_t : unsigned(6 downto 0);
        variable v_s : signed(7 downto 0);
        variable v_row : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_wp <= s_wp + 1;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            s_x(2) <= s_rx - 1; s_avd(2) <= s_in_avid;
            for i in 3 to 7 loop s_x(i) <= s_x(i - 1); s_avd(i) <= s_avd(i - 1); end loop;

            s_bars <= s_k_bars(9 downto 2);
            s_dist <= s_k_dist(9 downto 2);
            case to_integer(s_k_cell(9 downto 8)) is
                when 0 | 1 => s_ss <= "00"; s_cmask <= "00111";
                when 2 => s_ss <= "01"; s_cmask <= "01111";
                when others => s_ss <= "10"; s_cmask <= "11111";
            end case;
            s_con <= s_k_con(9 downto 2);
            s_amt <= s_k_shear(9 downto 5);
            s_rate <= s_k_rate(9 downto 2);
            s_mix <= s_p_reveal(9 downto 2);

            -- per-line sequencer
            case to_integer(s_lstep) is
                when 0 =>
                    case to_integer(s_ss) is
                        when 0 => v_row := s_line(2 downto 0); s_wbank <= s_line(3);
                        when 1 => v_row := s_line(3 downto 1); s_wbank <= s_line(4);
                        when others => v_row := s_line(4 downto 2); s_wbank <= s_line(5);
                    end case;
                    if v_row = 0 then s_bl0 <= '1'; else s_bl0 <= '0'; end if;
                    if s_tph(7) = '1' then v_t := not s_tph(6 downto 0); else v_t := s_tph(6 downto 0); end if;
                    s_tri <= signed(resize(v_t, 8)) - 64;
                    if s_lfsr(15 downto 8) < s_rate then s_tear <= signed(resize(s_lfsr(6 downto 0), 8)) - 64; else s_tear <= (others => '0'); end if;
                    s_tph <= s_tph + resize(s_rate(7 downto 3), 8);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    if s_sw_tear = '1' then s_shp <= s_tear * signed(resize(s_amt, 6)); else s_shp <= s_tri * signed(resize(s_amt, 6)); end if;
                    s_mp <= (signed(resize(s_lfsr(8 downto 0), 10)) - 256) * signed(resize(s_dist, 9));
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    v_s := resize(shift_right(s_shp, 6), 8);
                    if v_s > 30 then v_s := to_signed(30, 8); elsif v_s < -30 then v_s := to_signed(-30, 8); end if;
                    s_off_row <= unsigned(resize(to_signed(C_D, 8) + v_s, 6));
                    if s_bl0 = '1' then
                        if s_lfsr(7 downto 0) < s_bars then s_mosh_band <= resize(shift_right(s_mp, 6), 11); else s_mosh_band <= (others => '0'); end if;
                    end if;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    if s_sw_cells = '1' then
                        s_mosh_src <= (others => '0');
                        case to_integer(s_ss) is
                            when 0 => s_moshc <= resize(shift_right(s_mosh_band, 3), 8);
                            when 1 => s_moshc <= resize(shift_right(s_mosh_band, 4), 8);
                            when others => s_moshc <= resize(shift_right(s_mosh_band, 5), 8);
                        end case;
                    else
                        s_mosh_src <= s_mosh_band; s_moshc <= (others => '0');
                    end if;
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
            end if;
            s_prev_av43 <= s_avid_sr(43);
            if s_prev_av43 = '1' and s_avid_sr(43) = '0' then s_oline <= s_oline + 1; end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_oline <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then s_fph <= s_fph + 3; end if;
                s_tph <= s_fph;
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

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr4));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr4));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr4(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr4(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr4(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr4(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rrd <= ring(to_integer(s_ra7));
            ring(to_integer(s_wp)) <= std_logic_vector(s_wet_y) & std_logic_vector(s_wet_u(9 downto 2)) & std_logic_vector(s_wet_v(9 downto 2)) & "000000";
        end if;
    end process;
    p_dens0 : process(clk)
    begin
        if rising_edge(clk) then
            s_drd0 <= dens0(to_integer(s_ci8));
            if s_dwe8 = '1' and s_wbank = '0' then dens0(to_integer(s_da8)) <= s_dd8; end if;
        end if;
    end process;
    p_dens1 : process(clk)
    begin
        if rising_edge(clk) then
            s_drd1 <= dens1(to_integer(s_ci8));
            if s_dwe8 = '1' and s_wbank = '1' then dens1(to_integer(s_da8)) <= s_dd8; end if;
        end if;
    end process;
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_pat14 <= C_GLYPH(to_integer(s_romaddr13));
            s_pat15 <= s_pat14;
        end if;
    end process;

    p_pix : process(clk)
        variable v_w : signed(12 downto 0);
        variable v_lo : unsigned(4 downto 0);
        variable v_ink : std_logic;
        variable v_g : unsigned(5 downto 0);
        variable v_row : unsigned(2 downto 0);
        variable v_obank : std_logic;
        variable v_ci : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: moshed source address
            s_a3 <= signed(resize(s_x(2), 12)) + resize(s_mosh_src, 12);
            -- 4
            v_w := signed(resize(s_line_width, 13));
            if s_a3 < 0 or resize(s_a3, 13) >= v_w or s_avd(3) = '0' then s_bg4 <= '1'; else s_bg4 <= '0'; end if;
            s_addr4 <= unsigned(s_a3(10 downto 0));
            -- 6: fetch
            s_a_rbank <= s_rbank;
            if s_bg4 = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 7: cell sum (d6 pixel), sampled chroma
            v_lo := s_x(6)(4 downto 0) and s_cmask;
            if v_lo = 0 then s_csum <= resize(s_wet_y, 15); else s_csum <= s_csum + resize(s_wet_y, 15); end if;
            s_u7 <= s_wet_u; s_v7 <= s_wet_v;
            -- 8: cell write (d7 pixel is the last of its cell)
            v_lo := s_x(7)(4 downto 0) and s_cmask;
            if v_lo = s_cmask and s_bl0 = '1' and s_avd(7) = '1' then s_dwe8 <= '1'; else s_dwe8 <= '0'; end if;
            case to_integer(s_ss) is
                when 0 => s_da8 <= s_x(7)(10 downto 3); s_dd8 <= std_logic_vector(s_csum(12 downto 3)) & std_logic_vector(s_u7(9 downto 2)) & std_logic_vector(s_v7(9 downto 2)) & "000000";
                when 1 => s_da8 <= resize(s_x(7)(10 downto 4), 8); s_dd8 <= std_logic_vector(s_csum(13 downto 4)) & std_logic_vector(s_u7(9 downto 2)) & std_logic_vector(s_v7(9 downto 2)) & "000000";
                when others => s_da8 <= resize(s_x(7)(10 downto 5), 8); s_dd8 <= std_logic_vector(s_csum(14 downto 5)) & std_logic_vector(s_u7(9 downto 2)) & std_logic_vector(s_v7(9 downto 2)) & "000000";
            end case;

            -- output side: 7: source position and ring address
            s_xo7 <= signed(resize(s_x(6), 12)) - signed(resize(s_off_row, 12));
            s_ra7 <= s_wp - s_off_row;
            -- 8: cell index, glyph column, background
            case to_integer(s_ss) is
                when 0 => v_ci := resize(s_xo7(11 downto 3), 12); s_gcol8 <= unsigned(s_xo7(2 downto 0));
                when 1 => v_ci := resize(s_xo7(11 downto 4), 12); s_gcol8 <= unsigned(s_xo7(3 downto 1));
                when others => v_ci := resize(s_xo7(11 downto 5), 12); s_gcol8 <= unsigned(s_xo7(4 downto 2));
            end case;
            v_ci := v_ci + resize(s_moshc, 12);
            s_ci8 <= unsigned(v_ci(7 downto 0));
            if s_xo7 < 0 or resize(s_xo7, 13) >= v_w or v_ci < 0 or v_ci > 255 then s_bgo8 <= '1'; else s_bgo8 <= '0'; end if;
            -- 9: ring out -> fabric
            s_wy(9) <= unsigned(s_rrd(31 downto 22)); s_wu(9) <= unsigned(s_rrd(21 downto 14)) & "00"; s_wv(9) <= unsigned(s_rrd(13 downto 6)) & "00";
            -- 10: cell RAM out -> fabric (previous band's bank)
            case to_integer(s_ss) is
                when 0 => v_obank := s_oline(3);
                when 1 => v_obank := s_oline(4);
                when others => v_obank := s_oline(5);
            end case;
            if v_obank = '1' then
                s_dens10 <= unsigned(s_drd0(31 downto 22)); s_cu10 <= unsigned(s_drd0(21 downto 14)); s_cv10 <= unsigned(s_drd0(13 downto 6));
            else
                s_dens10 <= unsigned(s_drd1(31 downto 22)); s_cu10 <= unsigned(s_drd1(21 downto 14)); s_cv10 <= unsigned(s_drd1(13 downto 6));
            end if;
            -- 11
            if s_dens10 < 64 then s_d11 <= (others => '0'); else s_d11 <= s_dens10 - 64; end if;
            -- 12
            s_p12 <= s_d11 * s_con;
            -- 13: glyph index and row
            v_g := s_p12(17 downto 12);
            if v_g > 15 then v_g := to_unsigned(15, 6); end if;
            case to_integer(s_ss) is
                when 0 => v_row := s_oline(2 downto 0);
                when 1 => v_row := s_oline(3 downto 1);
                when others => v_row := s_oline(4 downto 2);
            end case;
            s_romaddr13 <= s_sw_blocks & v_g(3 downto 0) & v_row;
            -- 16: ink
            v_ink := s_pat15(7 - to_integer(s_gcol(15)));
            if s_bgo(15) = '1' then v_ink := '0'; end if;
            if (v_ink xor s_sw_black) = '1' then s_inky16 <= to_unsigned(940, 10); else s_inky16 <= to_unsigned(64, 10); end if;
            if v_ink = '1' and s_sw_colour = '1' then s_inku16 <= s_cu(15) & "00"; s_inkv16 <= s_cv(15) & "00";
            else s_inku16 <= to_unsigned(512, 10); s_inkv16 <= to_unsigned(512, 10); end if;
            -- 17
            s_dy17 <= signed(resize(s_inky16, 11)) - signed(resize(s_wy(16), 11));
            s_du17 <= signed(resize(s_inku16, 11)) - signed(resize(s_wu(16), 11));
            s_dv17 <= signed(resize(s_inkv16, 11)) - signed(resize(s_wv(16), 11));
            -- 18
            s_py18 <= s_dy17 * signed(resize(s_mix, 9));
            s_pu18 <= s_du17 * signed(resize(s_mix, 9));
            s_pv18 <= s_dv17 * signed(resize(s_mix, 9));
            -- 19
            s_y19 <= f_clamp10(signed(resize(s_wy(18), 13)) + resize(shift_right(s_py18, 8), 13));
            s_u19 <= f_clamp10(signed(resize(s_wu(18), 13)) + resize(shift_right(s_pu18, 8), 13));
            s_v19 <= f_clamp10(signed(resize(s_wv(18), 13)) + resize(shift_right(s_pv18, 8), 13));
            -- 20: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;

            -- carry chains
            for i in 10 to 18 loop s_wy(i) <= s_wy(i - 1); s_wu(i) <= s_wu(i - 1); s_wv(i) <= s_wv(i - 1); end loop;
            s_cu(10) <= s_cu10; s_cv(10) <= s_cv10;
            for i in 11 to 15 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            s_gcol(8) <= s_gcol8; s_bgo(8) <= s_bgo8;
            for i in 9 to 15 loop s_gcol(i) <= s_gcol(i - 1); s_bgo(i) <= s_bgo(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture teletext;
