-- Mirage: catalogue combo #4 (seed 20260912) -- Sabattier solarization +
-- displacement mapping (the picture's own luma pushes it sideways).  Three
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Threshold  solar threshold
--   K2 Gain       fold gain 0.125x .. 8x (log); >1x repeats into bands
--   K3 Lift       level of the folded zone
--   K4 Amount     displacement, bipolar (+-256 px at full map swing)
--   K5 Smooth     horizontal low-pass on the map
--   K6 Pivot      luma level of zero displacement
--
--   S7  Map       displacement map from the solarized luma / the original
--   S8  Order     solarize before the warp (the warp moves the bands) /
--                 after it (the bands are drawn on the warped picture)
--   S9  Region    displacement only inside the solar region
--   S10 Fold      the solar region also inverts chroma
--   S11 Wrap      out-of-range reads wrap instead of edge-filling
--   P12 Bands     displacement amount grows with the solar luma
--                 displacement |ysol - y|: the contour bands tear the image
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- Two solar_fold instances: A on the input (writes the line buffer through
-- a 7-stage-delayed write port, feeds the map), B on the fetched pixel.
-- Latency 25 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture mirage of program_top is

    constant C_LAT : integer := 25;

    function f_sclamp(v : signed; lim : integer; w : integer) return signed is
    begin
        if v > lim then
            return to_signed(lim, w);
        elsif v < -lim then
            return to_signed(-lim, w);
        else
            return resize(v, w);
        end if;
    end function;

    -- fold gain, Q8: 0.125x .. 8x over the knob, log spaced
    type t_gain_rom is array (0 to 63) of unsigned(11 downto 0);
    constant C_GAIN_ROM : t_gain_rom := (
        to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12),
        to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12),
        to_unsigned(32, 12), to_unsigned(36, 12), to_unsigned(41, 12), to_unsigned(47, 12), to_unsigned(54, 12), to_unsigned(61, 12), to_unsigned(70, 12), to_unsigned(79, 12),
        to_unsigned(90, 12), to_unsigned(103, 12), to_unsigned(117, 12), to_unsigned(134, 12), to_unsigned(152, 12), to_unsigned(173, 12), to_unsigned(197, 12), to_unsigned(225, 12),
        to_unsigned(256, 12), to_unsigned(292, 12), to_unsigned(332, 12), to_unsigned(378, 12), to_unsigned(431, 12), to_unsigned(490, 12), to_unsigned(559, 12), to_unsigned(636, 12),
        to_unsigned(725, 12), to_unsigned(825, 12), to_unsigned(940, 12), to_unsigned(1070, 12), to_unsigned(1219, 12), to_unsigned(1388, 12), to_unsigned(1581, 12), to_unsigned(1801, 12),
        to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12),
        to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12)
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

    --------------------------------------------------------------------------
    -- Controls / per-frame
    --------------------------------------------------------------------------
    signal s_k_thr, s_k_gain, s_k_lift, s_k_amount, s_k_smooth, s_k_pivot : unsigned(9 downto 0);
    signal s_sw_map, s_sw_after, s_sw_region, s_sw_fold, s_sw_wrap : std_logic;
    signal s_p_bands : unsigned(9 downto 0);
    signal s_gain    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_base    : signed(11 downto 0) := (others => '0');
    signal s_amount  : signed(10 downto 0) := (others => '0');
    signal s_bn      : unsigned(2 downto 0) := (others => '0');
    signal s_pivot   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_drive   : unsigned(7 downto 0) := (others => '0');
    signal s_en_a, s_en_b : std_logic := '1';

    --------------------------------------------------------------------------
    -- Input / tracking
    --------------------------------------------------------------------------
    signal s_rx      : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_wpar, s_rbank : std_logic := '0';
    type t_x is array (1 to 7) of unsigned(10 downto 0);
    signal s_wx : t_x := (others => (others => '0'));
    type t_b7 is array (1 to 7) of std_logic;
    signal s_wp : t_b7 := (others => '0');
    signal s_we8 : std_logic := '0';
    signal s_wp8 : std_logic := '0';
    signal s_wx8 : unsigned(10 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Solar A / B I/O
    --------------------------------------------------------------------------
    signal s_ya, s_ua, s_va, s_ysola : unsigned(9 downto 0);
    signal s_rega : std_logic;
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0);
    type t_y7 is array (1 to 7) of unsigned(9 downto 0);
    signal s_yo : t_y7 := (others => (others => '0'));
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_yb, s_ub, s_vb, s_ysolb : unsigned(9 downto 0);
    signal s_regb : std_logic;

    --------------------------------------------------------------------------
    -- Map / displacement chain (stages 8..15)
    --------------------------------------------------------------------------
    signal s_ya8, s_yo8 : unsigned(9 downto 0) := (others => '0');
    signal s_macc  : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);
    signal s_m9    : signed(10 downto 0) := (others => '0');
    signal s_dsp9  : unsigned(9 downto 0) := (others => '0');
    signal s_m10   : signed(10 downto 0) := (others => '0');
    signal s_dp10  : unsigned(17 downto 0) := (others => '0');
    signal s_m11   : signed(10 downto 0) := (others => '0');
    signal s_aeff11 : signed(11 downto 0) := (others => '0');
    signal s_ph12 : signed(17 downto 0) := (others => '0');
    signal s_pl12 : signed(16 downto 0) := (others => '0');
    signal s_d13   : signed(9 downto 0) := (others => '0');
    signal s_x14   : unsigned(10 downto 0) := (others => '0');
    signal s_u14   : signed(13 downto 0) := (others => '0');
    signal s_addr  : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    type t_bits is array (8 to 13) of std_logic;
    signal s_reg : t_bits := (others => '0');
    signal s_y25, s_u25, s_v25 : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    -- per-line clamp bounds (registered) so the address compares run on the
    -- source value in parallel with the wrap add / subtract
    signal s_we14, s_wl14, s_wlo14, s_rwe14, s_rhi14 : signed(13 downto 0) := (others => '0');
    function f_edge_addr(v : signed; we : unsigned(10 downto 0)) return unsigned is
    begin
        if v < C_EDGE then
            return to_unsigned(C_EDGE, 11);
        elsif v > signed(resize(we, v'length)) then
            return we;
        else
            return unsigned(v(10 downto 0));
        end if;
    end function;

begin

    s_k_thr     <= unsigned(registers_in(0));
    s_k_gain    <= unsigned(registers_in(1));
    s_k_lift    <= unsigned(registers_in(2));
    s_k_amount  <= unsigned(registers_in(3));
    s_k_smooth  <= unsigned(registers_in(4));
    s_k_pivot   <= unsigned(registers_in(5));
    s_sw_map    <= registers_in(6)(0);
    s_sw_after  <= registers_in(6)(1);
    s_sw_region <= registers_in(6)(2);
    s_sw_fold   <= registers_in(6)(3);
    s_sw_wrap   <= registers_in(6)(4);
    s_p_bands   <= unsigned(registers_in(7));

    s_in_y <= unsigned(data_in.y);
    s_in_u <= unsigned(data_in.u);
    s_in_v <= unsigned(data_in.v);

    --------------------------------------------------------------------------
    -- Solar instances
    --------------------------------------------------------------------------
    solar_a : entity work.solar_fold
        port map (clk => clk, y_in => s_in_y, u_in => s_in_u, v_in => s_in_v,
                  thr => s_k_thr, gain => s_gain, base => s_base, fold_c => s_sw_fold,
                  enable => s_en_a, y_out => s_ya, u_out => s_ua, v_out => s_va,
                  ysol => s_ysola, region => s_rega);

    solar_b : entity work.solar_fold
        port map (clk => clk, y_in => s_wet_y, u_in => s_wet_u, v_in => s_wet_v,
                  thr => s_k_thr, gain => s_gain, base => s_base, fold_c => s_sw_fold,
                  enable => s_en_b, y_out => s_yb, u_out => s_ub, v_out => s_vb,
                  ysol => s_ysolb, region => s_regb);

    --------------------------------------------------------------------------
    -- Per-frame decode, tracking, delayed write port
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_gain   <= C_GAIN_ROM(to_integer(s_k_gain(9 downto 4)));
            s_base   <= signed(resize(s_k_thr, 12)) + signed(resize(s_k_lift, 12)) - to_signed(1024, 12);
            s_amount <= signed(resize(s_k_amount, 11)) - to_signed(512, 11);
            s_bn     <= s_k_smooth(9 downto 7);
            s_pivot  <= s_k_pivot;
            s_drive  <= s_p_bands(9 downto 2);
            s_en_a   <= not s_sw_after;
            s_en_b   <= s_sw_after;

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
            else
                s_rx <= (others => '0');
            end if;
            -- write port delayed to match solar A (7 stages) + 1
            s_wx(1) <= s_rx;
            s_wp(1) <= s_wpar;
            for i in 2 to 7 loop
                s_wx(i) <= s_wx(i - 1);
                s_wp(i) <= s_wp(i - 1);
            end loop;
            s_wx8 <= s_wx(7);
            s_wp8 <= s_wp(7);
            s_we8 <= s_avid_sr(6);
            -- original luma delayed alongside solar A
            s_yo(1) <= s_in_y;
            for i in 2 to 7 loop
                s_yo(i) <= s_yo(i - 1);
            end loop;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we14  <= signed(resize(s_w_edge, 14));
            s_wl14  <= signed(resize(s_line_width, 14));
            s_wlo14 <= to_signed(C_EDGE, 14) - signed(resize(s_line_width, 14));
            s_rwe14 <= signed(resize(s_line_width, 14)) + C_EDGE;
            s_rhi14 <= signed(resize(s_line_width, 14)) + signed(resize(s_w_edge, 14));
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar  <= not s_wpar;
                s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
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
    -- Map and displacement, stages 8..15
    --------------------------------------------------------------------------
    p_map : process(clk)
        variable v_n : integer range 0 to 7;
        variable v_src : unsigned(9 downto 0);
        variable v_a : signed(12 downto 0);
        variable v_u : signed(13 downto 0);
        variable v_wl, v_wr : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 8: map IIR (source = solarized or original luma), regs
            v_n := to_integer(s_bn);
            if s_sw_map = '1' then
                v_src := s_ysola;
            else
                v_src := s_yo(7);
            end if;
            if s_avid_sr(6) = '0' then
                s_macc <= to_unsigned(512 * 16, 14);
            else
                s_macc <= s_macc - shift_right(s_macc, v_n) + shift_right(v_src & "0000", v_n);
            end if;
            s_ya8 <= s_ysola;
            s_yo8 <= s_yo(7);
            s_reg(8) <= s_rega;
            -- stage 9: map value, solar displacement
            s_m9 <= signed('0' & s_macc(13 downto 4)) - signed('0' & s_pivot);
            if s_ya8 >= s_yo8 then
                s_dsp9 <= s_ya8 - s_yo8;
            else
                s_dsp9 <= s_yo8 - s_ya8;
            end if;
            -- stage 10: bands drive product
            s_dp10 <= s_dsp9 * s_drive;
            s_m10  <= s_m9;
            -- stage 11: effective amount
            v_a := resize(s_amount, 13) + signed(resize(s_dp10(17 downto 8), 13));
            s_aeff11 <= f_sclamp(v_a, 1023, 12);
            s_m11 <= s_m10;
            -- stage 12: displacement products (aeff hi 7b signed / lo 5b)
            s_ph12 <= s_m11 * s_aeff11(11 downto 5);
            s_pl12 <= s_m11 * signed('0' & s_aeff11(4 downto 0));
            -- stage 13: d (clamped), region mask
            v_a := resize(shift_right(shift_left(resize(s_ph12, 24), 5) + resize(s_pl12, 24), 10), 13);
            if s_sw_region = '1' and s_reg(12) = '0' then
                s_d13 <= (others => '0');
            else
                s_d13 <= f_sclamp(v_a, 511, 10);
            end if;
            -- stage 14: source column
            if s_avid_sr(12) = '0' then
                s_x14 <= (others => '0');
            else
                s_x14 <= s_x14 + 1;
            end if;
            s_u14 <= signed(resize(s_x14, 14)) + resize(s_d13, 14);
            -- stage 15: wrap / range
            -- every compare is on s_u14 against a registered bound, in parallel
            -- with the wrap add / subtract; the mux is the only serial step
            v_u := s_u14;
            v_wl := v_u + s_wl14; v_wr := v_u - s_wl14;
            if s_sw_wrap = '1' and v_u < 0 then
                if v_u < s_wlo14 then s_addr <= to_unsigned(C_EDGE, 11);
                elsif v_u > -(C_EDGE + 1) then s_addr <= s_w_edge;
                else s_addr <= unsigned(v_wl(10 downto 0)); end if;
            elsif s_sw_wrap = '1' and v_u >= s_wl14 then
                if v_u < s_rwe14 then s_addr <= to_unsigned(C_EDGE, 11);
                elsif v_u > s_rhi14 then s_addr <= s_w_edge;
                else s_addr <= unsigned(v_wr(10 downto 0)); end if;
            elsif v_u < C_EDGE then s_addr <= to_unsigned(C_EDGE, 11);
            elsif v_u > s_we14 then s_addr <= s_w_edge;
            else s_addr <= unsigned(v_u(10 downto 0)); end if;
            -- region chain
            for i in 9 to 13 loop
                s_reg(i) <= s_reg(i - 1);
            end loop;
        end if;
    end process p_map;

    --------------------------------------------------------------------------
    -- Line buffers: written from solar A (stage 8), read at stage 16
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we8 = '1' and s_wp8 = '0' then
                lbY0(to_integer(s_wx8)) <= std_logic_vector(s_ya);
            end if;
        end if;
    end process;

    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we8 = '1' and s_wp8 = '1' then
                lbY1(to_integer(s_wx8)) <= std_logic_vector(s_ya);
            end if;
        end if;
    end process;

    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '0' then
                lbU0(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_ua(9 downto 2));
            end if;
        end if;
    end process;

    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '1' then
                lbU1(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_ua(9 downto 2));
            end if;
        end if;
    end process;

    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '0' then
                lbV0(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_va(9 downto 2));
            end if;
        end if;
    end process;

    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '1' then
                lbV1(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_va(9 downto 2));
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 16..25: fetch, solar B (17 -> 24), gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
                s_wet_u <= unsigned(s_rdU1) & "00";
                s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0);
                s_wet_u <= unsigned(s_rdU0) & "00";
                s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 25: gate (solar B output is stage 24)
            if s_avid_sr(23) = '1' then
                s_y25 <= s_yb; s_u25 <= s_ub; s_v25 <= s_vb;
            else
                s_y25 <= to_unsigned(64, 10);
                s_u25 <= to_unsigned(512, 10);
                s_v25 <= to_unsigned(512, 10);
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

end architecture mirage;
