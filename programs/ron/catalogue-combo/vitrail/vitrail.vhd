-- Vitrail: catalogue combo #20 (seed 20260915) -- bloom / glow + palette
-- quantization + mirror / kaleidoscope.  Two knobs each; the switches and
-- the slider decide how the three combine.
--
--   K1 Strip    kaleidoscope source strip width 64 .. 512 px
--   K2 Axis     where the strip is taken from (0 .. 100 % of the width)
--   K3 Thresh   bloom threshold
--   K4 Radius   bloom blur width 4 .. 64 px
--   K5 Levels   luma levels 2 .. 16
--   K6 Chroma   chroma levels per axis 1 (grey) .. 8
--
--   S7  Bloom   luma glow / white glow (chroma washes toward neutral)
--   S8  Mirror  kaleidoscope fold / plain tiling
--   S9  Order   palette after the mirror / palette before the bloom
--   S10 Axis    fixed / sweeping across the picture
--   S11 Bloom   highlights glow / shadows bloom dark
--   P12 Glow    bloom gain, up to blow-out
--
-- Bloom = running box sum of luma excess over the threshold (EBR ring)
-- applied before the line buffer through a delayed write port; the read
-- side folds x into a strip taken at the axis; posterize is bit masking.
-- Latency 11 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture vitrail of program_top is

    constant C_LAT : integer := 11;

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
    type t_ring is array (0 to 63) of std_logic_vector(15 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal ring : t_ring := (others => x"0000");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_ring_q : std_logic_vector(15 downto 0) := x"0000";
    signal s_ring_w : unsigned(5 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_strip, s_k_axis, s_k_thr, s_k_radius, s_k_levels, s_k_chroma : unsigned(9 downto 0);
    signal s_sw_white, s_sw_tile, s_sw_prepal, s_sw_sweep, s_sw_shadow : std_logic;
    signal s_p_glow : unsigned(9 downto 0);
    signal s_kst    : unsigned(3 downto 0) := to_unsigned(8, 4);     -- strip shift 6..9
    signal s_smask  : unsigned(10 downto 0) := (others => '0');       -- 2^kst - 1
    signal s_thr    : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_nr     : unsigned(2 downto 0) := to_unsigned(3, 3);      -- radius shift 2..6
    signal s_ndel   : unsigned(5 downto 0) := to_unsigned(7, 6);
    signal s_ymask, s_cmask : unsigned(9 downto 0) := (others => '1');
    signal s_yhalf, s_chalf : unsigned(9 downto 0) := (others => '0');
    signal s_gain   : unsigned(7 downto 0) := (others => '0');
    signal s_axis   : unsigned(10 downto 0) := (others => '0');
    signal s_axh, s_axl : unsigned(15 downto 0) := (others => '0');
    signal s_slide  : unsigned(10 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";

    -- tracking
    signal s_rx, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank : std_logic := '0';

    -- input side (stages 1..9)
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_y2, s_u2, s_v2 : unsigned(9 downto 0) := (others => '0');
    signal s_e3     : unsigned(9 downto 0) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := (others => '0');
    signal s_acc    : unsigned(15 downto 0) := (others => '0');
    signal s_y4, s_u4, s_v4 : unsigned(9 downto 0) := (others => '0');
    signal s_bl5    : unsigned(9 downto 0) := (others => '0');
    signal s_y5, s_u5, s_v5 : unsigned(9 downto 0) := (others => '0');
    signal s_g6     : unsigned(16 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := (others => '0');
    signal s_yb7    : unsigned(9 downto 0) := (others => '0');
    signal s_gs7    : unsigned(7 downto 0) := (others => '0');
    signal s_du7, s_dv7 : signed(10 downto 0) := (others => '0');
    signal s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    signal s_yb8    : unsigned(9 downto 0) := (others => '0');
    signal s_pu8, s_pv8 : signed(19 downto 0) := (others => '0');
    signal s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    type t_wx is array (1 to 8) of unsigned(10 downto 0);
    signal s_wx : t_wx := (others => (others => '0'));
    signal s_wp, s_wc : std_logic_vector(1 to 8) := (others => '0');
    signal s_wx9 : unsigned(10 downto 0) := (others => '0');
    signal s_wp9, s_we9 : std_logic := '0';

    -- read side
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_xs2    : unsigned(11 downto 0) := (others => '0');
    signal s_xr3    : unsigned(10 downto 0) := (others => '0');
    signal s_xm4    : unsigned(10 downto 0) := (others => '0');
    signal s_ad5    : unsigned(11 downto 0) := (others => '0');
    signal s_addr6  : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_y9r, s_u9r, s_v9r : unsigned(9 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_strip  <= unsigned(registers_in(0));
    s_k_axis   <= unsigned(registers_in(1));
    s_k_thr    <= unsigned(registers_in(2));
    s_k_radius <= unsigned(registers_in(3));
    s_k_levels <= unsigned(registers_in(4));
    s_k_chroma <= unsigned(registers_in(5));
    s_sw_white  <= registers_in(6)(0);
    s_sw_tile   <= registers_in(6)(1);
    s_sw_prepal <= registers_in(6)(2);
    s_sw_sweep  <= registers_in(6)(3);
    s_sw_shadow <= registers_in(6)(4);
    s_p_glow   <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Bloom ring: luma excess, read N-1 back
    --------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            s_ring_q <= ring(to_integer(s_ring_w - s_ndel));
            ring(to_integer(s_ring_w)) <= "000000" & std_logic_vector(s_e3);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control + input side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e : signed(11 downto 0);
        variable v_y : signed(12 downto 0);
        variable v_n : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;

            -- stage 1
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_x1 <= s_rx;
            -- stage 2: optional pre-palette
            if s_sw_prepal = '1' then
                s_y2 <= (s_in_y and s_ymask) or s_yhalf;
                s_u2 <= (s_in_u and s_cmask) or s_chalf;
                s_v2 <= (s_in_v and s_cmask) or s_chalf;
            else
                s_y2 <= s_in_y; s_u2 <= s_in_u; s_v2 <= s_in_v;
            end if;
            -- stage 3: excess over / under the threshold
            if s_sw_shadow = '1' then
                v_e := signed(resize(s_thr, 12)) - signed(resize(s_y2, 12));
            else
                v_e := signed(resize(s_y2, 12)) - signed(resize(s_thr, 12));
            end if;
            if v_e < 0 or s_avid_sr(1) = '0' then s_e3 <= (others => '0'); else s_e3 <= unsigned(v_e(9 downto 0)); end if;
            s_y3 <= s_y2; s_u3 <= s_u2; s_v3 <= s_v2;
            -- stage 4: running sum
            if s_avid_sr(2) = '0' then
                s_acc <= (others => '0');
            else
                s_acc <= s_acc + resize(s_e3, 16) - resize(unsigned(s_ring_q(9 downto 0)), 16);
            end if;
            s_y4 <= s_y3; s_u4 <= s_u3; s_v4 <= s_v3;
            -- stage 5: average excess
            s_bl5 <= resize(shift_right(s_acc, to_integer(s_nr)), 10);
            s_y5 <= s_y4; s_u5 <= s_u4; s_v5 <= s_v4;
            -- stage 6: gain
            s_g6 <= s_bl5(9 downto 1) * s_gain;
            s_y6 <= s_y5; s_u6 <= s_u5; s_v6 <= s_v5;
            -- stage 7: bloomed luma; chroma pull-to-neutral differences
            if s_sw_shadow = '1' then
                v_y := signed(resize(s_y6, 13)) - signed(resize(s_g6(16 downto 6), 13));
            else
                v_y := signed(resize(s_y6, 13)) + signed(resize(s_g6(16 downto 6), 13));
            end if;
            s_yb7 <= f_clamp10(v_y);
            if s_sw_white = '1' then
                if s_g6(16 downto 14) /= 0 then s_gs7 <= x"FF"; else s_gs7 <= s_g6(13 downto 6); end if;
            else
                s_gs7 <= (others => '0');
            end if;
            s_du7 <= to_signed(512, 11) - signed('0' & s_u6);
            s_dv7 <= to_signed(512, 11) - signed('0' & s_v6);
            s_u7 <= s_u6; s_v7 <= s_v6;
            -- stage 8: products
            s_pu8 <= s_du7 * signed('0' & s_gs7);
            s_pv8 <= s_dv7 * signed('0' & s_gs7);
            s_yb8 <= s_yb7; s_u8 <= s_u7; s_v8 <= s_v7;
            -- stage 9: write data
            s_y9 <= s_yb8;
            s_u9 <= f_clamp10(signed(resize(s_u8, 13)) + resize(shift_right(s_pu8, 8), 13));
            s_v9 <= f_clamp10(signed(resize(s_v8, 13)) + resize(shift_right(s_pv8, 8), 13));
            -- write port: address = rx delayed 9
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar; s_wc(1) <= data_in.avid;
            for i in 2 to 8 loop s_wx(i) <= s_wx(i - 1); s_wp(i) <= s_wp(i - 1); s_wc(i) <= s_wc(i - 1); end loop;
            s_wx9 <= s_wx(8); s_wp9 <= s_wp(8); s_we9 <= s_wc(8);

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_kst  <= resize(s_k_strip(9 downto 8), 4) + 6;
                    s_thr  <= s_k_thr;
                    v_n := resize(s_k_radius(9 downto 7), 4) + 1;
                    if v_n > 6 then v_n := to_unsigned(6, 4); end if;
                    s_nr   <= v_n(2 downto 0);
                    s_gain <= s_p_glow(9 downto 2);
                    s_axh  <= s_k_axis * s_line_width(10 downto 5);
                    s_axl  <= resize(s_k_axis * s_line_width(4 downto 0), 16);
                    case to_integer(s_k_levels(9 downto 8)) is
                        when 0 => s_ymask <= "1000000000"; s_yhalf <= "0100000000";
                        when 1 => s_ymask <= "1100000000"; s_yhalf <= "0010000000";
                        when 2 => s_ymask <= "1110000000"; s_yhalf <= "0001000000";
                        when others => s_ymask <= "1111000000"; s_yhalf <= "0000100000";
                    end case;
                    case to_integer(s_k_chroma(9 downto 8)) is
                        when 0 => s_cmask <= "0000000000"; s_chalf <= "1000000000";
                        when 1 => s_cmask <= "1000000000"; s_chalf <= "0100000000";
                        when 2 => s_cmask <= "1100000000"; s_chalf <= "0010000000";
                        when others => s_cmask <= "1110000000"; s_chalf <= "0001000000";
                    end case;
                    if s_sw_sweep = '1' then
                        if s_slide + 3 >= s_line_width then s_slide <= (others => '0'); else s_slide <= s_slide + 3; end if;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_axis <= resize(shift_right(shift_left(resize(s_axh, 22), 5) + resize(s_axl, 22), 10), 11);
                    s_ndel <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nr)), 7)(5 downto 0) - 1;
                    for i in 0 to 10 loop
                        if i < to_integer(s_kst) then s_smask(i) <= '1'; else s_smask(i) <= '0'; end if;
                    end loop;
                    s_vstep <= "111";
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
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
    -- Read address chain (stages 2..6): slide, wrap, fold, axis, wrap
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_f : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_xs2 <= resize(s_x1, 12) + resize(s_slide, 12);
            if s_xs2 >= resize(s_line_width, 12) then
                s_xr3 <= resize(s_xs2 - resize(s_line_width, 12), 11);
            else
                s_xr3 <= s_xs2(10 downto 0);
            end if;
            v_f := s_xr3 and s_smask;
            if s_sw_tile = '0' and s_xr3(to_integer(s_kst)) = '1' then
                v_f := (not s_xr3) and s_smask;
            end if;
            s_xm4 <= v_f;
            s_ad5 <= resize(s_axis, 12) + resize(s_xm4, 12);
            if s_ad5 >= resize(s_line_width, 12) then
                s_addr6 <= resize(s_ad5 - resize(s_line_width, 12), 11);
            else
                s_addr6 <= s_ad5(10 downto 0);
            end if;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers: written at stage 9, read at stage 6 -> data 7
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr6));
            if s_we9 = '1' and s_wp9 = '0' then lbY0(to_integer(s_wx9)) <= std_logic_vector(s_y9); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr6));
            if s_we9 = '1' and s_wp9 = '1' then lbY1(to_integer(s_wx9)) <= std_logic_vector(s_y9); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr6(10 downto 1)));
            if s_we9 = '1' and s_wp9 = '0' then lbU0(to_integer(s_wx9(10 downto 1))) <= std_logic_vector(s_u9(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr6(10 downto 1)));
            if s_we9 = '1' and s_wp9 = '1' then lbU1(to_integer(s_wx9(10 downto 1))) <= std_logic_vector(s_u9(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr6(10 downto 1)));
            if s_we9 = '1' and s_wp9 = '0' then lbV0(to_integer(s_wx9(10 downto 1))) <= std_logic_vector(s_v9(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr6(10 downto 1)));
            if s_we9 = '1' and s_wp9 = '1' then lbV1(to_integer(s_wx9(10 downto 1))) <= std_logic_vector(s_v9(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fetch, post-palette, gate (stages 7..11)
    --------------------------------------------------------------------------
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            -- stage 7 ctx / 8 fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 9: post-palette
            if s_sw_prepal = '0' then
                s_y9r <= (s_wet_y and s_ymask) or s_yhalf;
                s_u9r <= (s_wet_u and s_cmask) or s_chalf;
                s_v9r <= (s_wet_v and s_cmask) or s_chalf;
            else
                s_y9r <= s_wet_y; s_u9r <= s_wet_u; s_v9r <= s_wet_v;
            end if;
            -- stage 10: gate
            if s_avid_sr(8) = '1' then
                s_y10 <= s_y9r; s_u10 <= s_u9r; s_v10 <= s_v9r;
            else
                s_y10 <= to_unsigned(64, 10); s_u10 <= to_unsigned(512, 10); s_v10 <= to_unsigned(512, 10);
            end if;
            -- stage 11: output
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y11);
    data_out.u       <= std_logic_vector(s_u11);
    data_out.v       <= std_logic_vector(s_v11);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture vitrail;
