-- Dropout: catalogue combo #50 (seed 20260920) -- chromatic glitch /
-- datamosh bars + head-switch noise band.  Three knobs each; the switches
-- and the slider decide how they combine.
--
--   K1 Bars     fraction of bands displaced (datamosh)
--   K2 Distance displacement (up to +-512 px, hashed sign)
--   K3 Hold     frames a displacement pattern is kept (1..64, P-frame repeat)
--   K4 Band     head-switch band height (0..255 lines)
--   K5 Noise    noise amplitude in the band
--   K6 Skew     head-switch flagging: rows in the band shift progressively
--
--   S7  Roll    band fixed at the bottom / rolling upward
--   S8  Dropout displaced bars lose their chroma
--   S9  Smear   displaced bars hold 16-px runs (blocky smear) / plain shift
--   S10 Burst   bands touching the head-switch band are always displaced
--   S11 Noise   white snow / per-line noise lines
--   P12 Bands   band height 8 .. 128 lines
--
-- Line buffer read a line late; per-band hashed offset (seed refreshed every
-- Hold frames), head-switch skew ramp and noise gate per line.  Latency 16
-- clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture dropout of program_top is

    constant C_LAT : integer := 16;

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

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_bars, s_k_dist, s_k_hold, s_k_band, s_k_noise, s_k_skew : unsigned(9 downto 0);
    signal s_sw_roll, s_sw_drop, s_sw_smear, s_sw_burst, s_sw_lines : std_logic;
    signal s_p_bands : unsigned(9 downto 0);

    signal s_bars, s_dist, s_noise, s_skew : unsigned(7 downto 0) := (others => '0');
    signal s_hold : unsigned(5 downto 0) := (others => '0');
    signal s_hcount : unsigned(5 downto 0) := (others => '0');
    signal s_bandh : unsigned(7 downto 0) := (others => '0');
    signal s_bshift : unsigned(2 downto 0) := "011";
    signal s_seed : unsigned(15 downto 0) := x"5A5A";
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal s_saw_active : std_logic := '0';
    signal s_rollpos : unsigned(10 downto 0) := (others => '0');
    signal s_hs_top : unsigned(10 downto 0) := (others => '0');
    signal s_vstep : unsigned(2 downto 0) := "111";

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    -- per-line
    signal s_lstep : unsigned(2 downto 0) := "111";
    signal s_band : unsigned(7 downto 0) := (others => '0');
    signal s_bstart : std_logic := '0';
    signal s_hm : unsigned(23 downto 0) := (others => '0');
    signal s_h1 : unsigned(15 downto 0) := (others => '0');
    signal s_mp : signed(18 downto 0) := (others => '0');
    signal s_inhs : std_logic := '0';
    signal s_hsrow : unsigned(7 downto 0) := (others => '0');
    signal s_near : std_logic := '0';
    signal s_moshed : std_logic := '0';
    signal s_moff : signed(10 downto 0) := (others => '0');
    signal s_sp : unsigned(15 downto 0) := (others => '0');
    signal s_off_row : signed(10 downto 0) := (others => '0');
    signal s_mosh_row, s_hs_row : std_logic := '0';
    signal s_lnoise : signed(9 downto 0) := (others => '0');

    signal s_x1 : unsigned(10 downto 0) := (others => '0');
    signal s_xh2 : unsigned(10 downto 0) := (others => '0');
    signal s_u3 : signed(12 downto 0) := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg : std_logic := '0';
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_nz6 : unsigned(15 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := (others => '0');
    signal s_y7 : signed(11 downto 0) := (others => '0');
    signal s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    type t_c10 is array (9 to 13) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_c10 := (others => (others => '0'));
    type t_b is array (3 to 8) of std_logic;
    signal s_moshd, s_hsd : t_b := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_bars  <= unsigned(registers_in(0));
    s_k_dist  <= unsigned(registers_in(1));
    s_k_hold  <= unsigned(registers_in(2));
    s_k_band  <= unsigned(registers_in(3));
    s_k_noise <= unsigned(registers_in(4));
    s_k_skew  <= unsigned(registers_in(5));
    s_sw_roll  <= registers_in(6)(0);
    s_sw_drop  <= registers_in(6)(1);
    s_sw_smear <= registers_in(6)(2);
    s_sw_burst <= registers_in(6)(3);
    s_sw_lines <= registers_in(6)(4);
    s_p_bands <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_o : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_x1 <= s_rx - 1;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

            s_bars <= s_k_bars(9 downto 2); s_dist <= s_k_dist(9 downto 2);
            s_hold <= s_k_hold(9 downto 4); s_bandh <= s_k_band(9 downto 2);
            s_noise <= s_k_noise(9 downto 2); s_skew <= s_k_skew(9 downto 2);
            s_bshift <= "011" + resize(s_p_bands(9 downto 8), 3);   -- 3..6 -> 8..64 lines; 7 -> 128

            -- per frame
            case to_integer(s_vstep) is
                when 0 =>
                    if s_hcount >= s_hold then s_hcount <= (others => '0'); s_seed <= s_lfsr; else s_hcount <= s_hcount + 1; end if;
                    if s_sw_roll = '1' then s_rollpos <= s_rollpos + 3; else s_rollpos <= (others => '0'); end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    if s_rollpos >= s_height then s_rollpos <= (others => '0'); end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_hs_top <= s_height - resize(s_bandh, 11) - s_rollpos;
                    s_vstep <= "111";
                when others => null;
            end case;

            -- per line
            case to_integer(s_lstep) is
                when 0 =>
                    case to_integer(s_bshift) is
                        when 3 => s_band <= s_line(10 downto 3); if s_line(2 downto 0) = 0 then s_bstart <= '1'; else s_bstart <= '0'; end if;
                        when 4 => s_band <= resize(s_line(10 downto 4), 8); if s_line(3 downto 0) = 0 then s_bstart <= '1'; else s_bstart <= '0'; end if;
                        when 5 => s_band <= resize(s_line(10 downto 5), 8); if s_line(4 downto 0) = 0 then s_bstart <= '1'; else s_bstart <= '0'; end if;
                        when others => s_band <= resize(s_line(10 downto 6), 8); if s_line(5 downto 0) = 0 then s_bstart <= '1'; else s_bstart <= '0'; end if;
                    end case;
                    if s_line >= s_hs_top and s_line < s_hs_top + resize(s_bandh, 11) then s_inhs <= '1'; else s_inhs <= '0'; end if;
                    s_hsrow <= resize(s_line - s_hs_top, 8);
                    if s_line + 32 >= s_hs_top and s_line < s_hs_top + resize(s_bandh, 11) + 32 then s_near <= '1'; else s_near <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_hm <= s_band * to_unsigned(40503, 16);
                    s_sp <= s_hsrow * s_skew;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_h1 <= (s_hm(15 downto 0) xor s_seed) + (s_hm(23 downto 8));
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    if s_bstart = '1' then
                        if s_h1(15 downto 8) < s_bars or (s_sw_burst = '1' and s_near = '1') then s_moshed <= '1'; else s_moshed <= '0'; end if;
                        s_mp <= (signed(resize(s_h1(8 downto 0), 10)) - 256) * signed(resize(s_dist, 9));
                    end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_moshed = '1' then s_moff <= resize(shift_right(s_mp, 6), 11); else s_moff <= (others => '0'); end if;
                    if s_sw_lines = '1' and s_inhs = '1' then
                        s_lnoise <= resize(shift_right((signed(resize(s_lfsr(7 downto 0), 9)) - 128) * signed(resize(s_noise, 9)), 6), 10);
                    else
                        s_lnoise <= (others => '0');
                    end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    v_o := resize(s_moff, 13);
                    if s_inhs = '1' then v_o := v_o + signed(resize(s_sp(15 downto 3), 13)); end if;
                    if v_o > 1023 then v_o := to_signed(1023, 13); elsif v_o < -1023 then v_o := to_signed(-1023, 13); end if;
                    s_off_row <= resize(v_o, 11);
                    s_mosh_row <= s_moshed; s_hs_row <= s_inhs;
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
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

    p_addr : process(clk)
        variable v_w : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: smear hold
            if s_sw_smear = '1' and s_mosh_row = '1' then s_xh2 <= s_x1(10 downto 4) & "0000"; else s_xh2 <= s_x1; end if;
            -- 3
            s_u3 <= signed(resize(s_xh2, 13)) + resize(s_off_row, 13);
            s_moshd(3) <= s_mosh_row; s_hsd(3) <= s_hs_row;
            -- 4
            v_w := signed(resize(s_line_width, 13));
            if s_u3 < 0 or s_u3 >= v_w then s_bg <= '1'; else s_bg <= '0'; end if;
            s_addr <= unsigned(s_u3(10 downto 0));
            for i in 4 to 8 loop s_moshd(i) <= s_moshd(i - 1); s_hsd(i) <= s_hsd(i - 1); end loop;
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

    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            -- 5 (rd) / 6: fetch, noise product
            s_a_bg <= s_bg; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            s_nz6 <= s_lfsr(7 downto 0) * s_noise;
            -- 7: dropout chroma, band noise
            s_y6 <= s_wet_y; s_u6 <= s_wet_u; s_v6 <= s_wet_v;
            if s_hsd(7) = '1' then
                if s_sw_lines = '1' then
                    s_y7 <= signed(resize(s_y6, 12)) + resize(s_lnoise, 12);
                else
                    s_y7 <= signed(resize(s_y6, 12)) + signed(resize(s_nz6(15 downto 8), 12)) - signed(resize(s_noise(7 downto 1), 12));
                end if;
            else
                s_y7 <= signed(resize(s_y6, 12));
            end if;
            if s_sw_drop = '1' and s_moshd(7) = '1' then
                s_u7 <= to_unsigned(512, 10); s_v7 <= to_unsigned(512, 10);
            else
                s_u7 <= s_u6; s_v7 <= s_v6;
            end if;
            -- 8
            s_y8 <= f_clamp10(s_y7); s_u8 <= s_u7; s_v8 <= s_v7;
            s_yd(9) <= s_y8; s_ud(9) <= s_u8; s_vd(9) <= s_v8;
            for i in 10 to 13 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            -- 16: gate
            if s_avid_sr(14) = '1' then
                s_y16 <= s_yd(13); s_u16 <= s_ud(13); s_v16 <= s_vd(13);
            else
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture dropout;
