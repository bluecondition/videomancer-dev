-- Posterduo: catalogue combo #112 (seed 20260925) -- posterization +
-- duotone / tritone mapping from luma.  Three knobs each; the switches and
-- the slider decide how they combine.
--
--   K1 Luma     posterize luma 1..8 bits
--   K2 Dither   2x2 dither amount before quantizing (0..100% of a step)
--   K3 Pitch    dither cell 1 / 2 / 4 / 8 px
--   K4 Shadow   shadow ink hue (8 hues)
--   K5 Light    highlight ink hue (8 hues)
--   K6 Contrast tone curve contrast (0..400%)
--
--   S7  Order   posterize then tone (banded inks) / tone then posterize the inks
--   S8  Tritone third (neutral) tone in the midtones
--   S9  Luma    luma kept / flattened to the tone ramp
--   S10 Bands   alternate posterize bands swap the inks
--   S11 Chroma  ink chroma posterized too (with Tone-Post)
--   P12 Balance midpoint between the two inks sweeps through the picture
--
-- Per-pixel chain: dithered quantizer around a duotone entity (7 clocks).
-- Latency 14 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture posterduo of program_top is

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

    type t_tab is array (0 to 7) of integer range -256 to 255;
    constant C_TU : t_tab := (200, 141, 0, -141, -200, -141, 0, 141);
    constant C_TV : t_tab := (0, 141, 200, 141, 0, -141, -200, -141);

    signal s_k_lbits, s_k_dith, s_k_pitch, s_k_shadow, s_k_light, s_k_con : unsigned(9 downto 0);
    signal s_sw_tonefirst, s_sw_tri, s_sw_flat, s_sw_bands, s_sw_cpost : std_logic;
    signal s_p_bal : unsigned(9 downto 0);

    signal s_kl : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_lnot, s_lhalf, s_lq : unsigned(9 downto 0) := (others => '0');
    signal s_dsc : unsigned(9 downto 0) := (others => '0');
    signal s_dl : unsigned(17 downto 0) := (others => '0');
    signal s_stepl : unsigned(9 downto 0) := (others => '0');
    signal s_pitch : unsigned(1 downto 0) := "00";
    signal s_su0, s_sv0, s_lu0, s_lv0, s_su, s_sv, s_lu, s_lv : signed(8 downto 0) := (others => '0');
    signal s_con : unsigned(7 downto 0) := (others => '0');
    signal s_bal : unsigned(9 downto 0) := (others => '0');
    signal s_vstep : unsigned(1 downto 0) := "11";
    signal s_saw_active : std_logic := '0';

    signal s_rx : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line : unsigned(10 downto 0) := (others => '0');

    signal s_dz2 : unsigned(1 downto 0) := (others => '0');
    signal s_y2, s_u2, s_v2 : unsigned(9 downto 0) := (others => '0');
    signal s_d3 : unsigned(9 downto 0) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := (others => '0');
    signal s_yv4 : unsigned(10 downto 0) := (others => '0');
    signal s_u4, s_v4 : unsigned(9 downto 0) := (others => '0');
    signal s_yq5 : unsigned(9 downto 0) := (others => '0');
    signal s_ya5 : unsigned(9 downto 0) := (others => '0');
    signal s_u5, s_v5 : unsigned(9 downto 0) := (others => '0');
    signal s_band5 : std_logic := '0';
    signal s_yt, s_ut, s_vt : unsigned(9 downto 0) := (others => '0');   -- tone out at 12
    type t_q is array (6 to 12) of unsigned(9 downto 0);
    signal s_yqd : t_q := (others => (others => '0'));
    type t_bd is array (6 to 12) of std_logic;
    signal s_bandd : t_bd := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_lbits  <= unsigned(registers_in(0));
    s_k_dith   <= unsigned(registers_in(1));
    s_k_pitch  <= unsigned(registers_in(2));
    s_k_shadow <= unsigned(registers_in(3));
    s_k_light  <= unsigned(registers_in(4));
    s_k_con    <= unsigned(registers_in(5));
    s_sw_tonefirst <= registers_in(6)(0);
    s_sw_tri       <= registers_in(6)(1);
    s_sw_flat      <= registers_in(6)(2);
    s_sw_bands     <= registers_in(6)(3);
    s_sw_cpost     <= registers_in(6)(4);
    s_p_bal <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_m : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;

            s_kl <= to_unsigned(9, 4) - resize(s_k_lbits(9 downto 7), 4);
            s_pitch <= s_k_pitch(9 downto 8);
            s_su0 <= to_signed(C_TU(to_integer(s_k_shadow(9 downto 7))), 9); s_sv0 <= to_signed(C_TV(to_integer(s_k_shadow(9 downto 7))), 9);
            s_lu0 <= to_signed(C_TU(to_integer(s_k_light(9 downto 7))), 9);  s_lv0 <= to_signed(C_TV(to_integer(s_k_light(9 downto 7))), 9);
            s_con <= s_k_con(9 downto 2); s_bal <= s_p_bal;

            case to_integer(s_vstep) is
                when 0 =>
                    v_m := shift_left(to_unsigned(1, 10), to_integer(s_kl)) - 1;
                    s_lnot <= not v_m; s_lhalf <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 1);
                    s_lq <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 2);
                    s_stepl <= shift_left(to_unsigned(1, 10), to_integer(s_kl));
                    s_vstep <= s_vstep + 1;
                when 1 => s_dl <= s_stepl * s_k_dith(9 downto 2); s_vstep <= s_vstep + 1;
                when 2 => s_dsc <= s_dl(17 downto 8); s_vstep <= "11";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then s_line <= s_line + 1; end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
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

    -- per-pixel ink swap on alternate bands: swap the ink pair by the band bit
    s_su <= s_lu0 when (s_sw_bands = '1' and s_bandd(6) = '1') else s_su0;
    s_sv <= s_lv0 when (s_sw_bands = '1' and s_bandd(6) = '1') else s_sv0;
    s_lu <= s_su0 when (s_sw_bands = '1' and s_bandd(6) = '1') else s_lu0;
    s_lv <= s_sv0 when (s_sw_bands = '1' and s_bandd(6) = '1') else s_lv0;

    -- 5 -> 12: tone map (input = posterized luma, or raw luma when Tone-Post)
    tone : entity work.duotone
        port map (clk => clk, y_in => s_ya5, u_in => s_u5, v_in => s_v5,
                  su => s_su, sv => s_sv, lu => s_lu, lv => s_lv, con => s_con, bal => s_bal,
                  tri => s_sw_tri, flat => s_sw_flat, enable => '1',
                  y_out => s_yt, u_out => s_ut, v_out => s_vt);

    p_pix : process(clk)
        variable v_d : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: dither value
            case to_integer(s_pitch) is
                when 0 => s_dz2 <= s_rx(0) & s_line(0);
                when 1 => s_dz2 <= s_rx(1) & s_line(1);
                when 2 => s_dz2 <= s_rx(2) & s_line(2);
                when others => s_dz2 <= s_rx(3) & s_line(3);
            end case;
            s_y2 <= s_in_y; s_u2 <= s_in_u; s_v2 <= s_in_v;
            -- 3: dither magnitude (step * amount * bayer2x2/4)
            case to_integer(s_dz2) is
                when 0 => s_d3 <= (others => '0');
                when 1 => s_d3 <= shift_right(s_dsc, 1);
                when 2 => s_d3 <= shift_right(s_dsc, 1) + shift_right(s_dsc, 2);
                when others => s_d3 <= shift_right(s_dsc, 2);
            end case;
            s_y3 <= s_y2; s_u3 <= s_u2; s_v3 <= s_v2;
            -- 4
            s_yv4 <= resize(s_y3, 11) + resize(s_d3, 11);
            s_u4 <= s_u3; s_v4 <= s_v3;
            -- 5: quantize
            if s_yv4 > 1023 then v_d := (others => '1'); else v_d := s_yv4(9 downto 0); end if;
            s_yq5 <= (v_d and s_lnot) + s_lhalf;
            if s_sw_tonefirst = '1' then s_ya5 <= s_y3; else s_ya5 <= (v_d and s_lnot) + s_lhalf; end if;
            s_u5 <= s_u4; s_v5 <= s_v4;
            s_band5 <= v_d(to_integer(s_kl));
            s_yqd(6) <= s_yq5; s_bandd(6) <= s_band5;
            for i in 7 to 12 loop s_yqd(i) <= s_yqd(i - 1); s_bandd(i) <= s_bandd(i - 1); end loop;
            -- 13: post-quantize the toned picture when Tone-Post
            if s_sw_tonefirst = '1' then
                s_y13 <= (s_yt and s_lnot) + s_lhalf;
                if s_sw_cpost = '1' then s_u13 <= (s_ut and s_lnot) + s_lhalf; s_v13 <= (s_vt and s_lnot) + s_lhalf; else s_u13 <= s_ut; s_v13 <= s_vt; end if;
            else
                s_y13 <= s_yt; s_u13 <= s_ut; s_v13 <= s_vt;
            end if;
            -- 14: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y14);
    data_out.u       <= std_logic_vector(s_u14);
    data_out.v       <= std_logic_vector(s_v14);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture posterduo;
