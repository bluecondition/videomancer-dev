-- Negspin: catalogue combo #76 (seed 20260922) -- hue rotation /
-- saturation crush-boost + invert / negative / bit-plane isolation.  Three
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Hue      base hue rotation 0..360 degrees
--   K2 Sat      saturation 0..400% (100% = unity)
--   K3 Spread   extra hue rotation per luma band
--   K4 Plane    0 = negative, 1..8 = luma bit-plane isolated
--   K5 Mix      plane picture mixed over the original (0..100%)
--   K6 Chroma   chroma bit-plane depth (0 = off, 1..7 = planes kept)
--
--   S7  Luma    luma as is / negative
--   S8  Chroma  chroma as is / negative (hue + 180)
--   S9  Flip    the selected plane bit flips the hue by 180 degrees
--   S10 Source  plane taken from luma / from saturation
--   S11 Order   plane before the rotation / after
--   P12 Spin    hue rotation speed, bipolar (centre = still)
--
-- Per-pixel chain, sine-ROM rotation (four 11x10 products), no line memory.
-- Latency 18 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture negspin of program_top is

    constant C_LAT : integer := 18;

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

    function f_clamp511(v : signed) return signed is
    begin
        if v < -511 then
            return to_signed(-511, 11);
        elsif v > 511 then
            return to_signed(511, 11);
        else
            return resize(v, 11);
        end if;
    end function;

    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251,
        -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215,
        -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147,
        -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,
         -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,
          50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,
         142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,
         212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,
         250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,
         250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,
         212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,
         142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,
          50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,
         -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136,
        -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208,
        -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249,
        -250, -251, -252, -253, -254, -254, -255, -255
    );

    signal s_k_hue, s_k_sat, s_k_spread, s_k_plane, s_k_mix, s_k_cpl : unsigned(9 downto 0);
    signal s_sw_negy, s_sw_negc, s_sw_flip, s_sw_satsrc, s_sw_after : std_logic;
    signal s_p_spin : unsigned(9 downto 0);

    signal s_hue : unsigned(7 downto 0) := (others => '0');
    signal s_gain, s_gain_a : unsigned(8 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_gain_a : signal is true;
    signal s_spread : unsigned(7 downto 0) := (others => '0');
    signal s_plane : unsigned(3 downto 0) := to_unsigned(1, 4);   -- power-on value keeps the bit index in range
    signal s_mix : unsigned(7 downto 0) := (others => '0');
    signal s_cmask : unsigned(9 downto 0) := (others => '1');
    signal s_phase : unsigned(15 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_avid : std_logic := '0';

    -- 2
    signal s_u2, s_v2 : signed(10 downto 0) := (others => '0');
    signal s_y2 : unsigned(9 downto 0) := (others => '0');
    signal s_au2, s_av2 : unsigned(9 downto 0) := (others => '0');
    -- 3
    signal s_sat3, s_y3 : unsigned(9 downto 0) := (others => '0');
    signal s_u3, s_v3 : signed(10 downto 0) := (others => '0');
    -- 4
    signal s_bit4 : std_logic := '0';
    signal s_pl4, s_y4 : unsigned(9 downto 0) := (others => '0');
    signal s_u4, s_v4 : signed(10 downto 0) := (others => '0');
    signal s_band4 : unsigned(2 downto 0) := (others => '0');
    -- 5
    signal s_bs5 : unsigned(10 downto 0) := (others => '0');
    signal s_bit5 : std_logic := '0';
    signal s_pl5, s_y5 : unsigned(9 downto 0) := (others => '0');
    signal s_u5, s_v5 : signed(10 downto 0) := (others => '0');
    -- 6
    signal s_th6 : unsigned(7 downto 0) := (others => '0');
    signal s_pl6, s_y6 : unsigned(9 downto 0) := (others => '0');
    signal s_u6, s_v6 : signed(10 downto 0) := (others => '0');
    -- 7: ROM addresses
    signal s_sa7, s_ca7 : unsigned(7 downto 0) := (others => '0');
    -- rom out 8, fabric 9
    signal s_sin8, s_cos8, s_sin9, s_cos9 : signed(9 downto 0) := (others => '0');
    type t_u11 is array (7 to 9) of signed(10 downto 0);
    signal s_ud, s_vd : t_u11 := (others => (others => '0'));
    type t_y10 is array (7 to 17) of unsigned(9 downto 0);
    signal s_pld, s_yd : t_y10 := (others => (others => '0'));
    -- 10..
    signal s_uc10, s_us10, s_vc10, s_vs10 : signed(20 downto 0) := (others => '0');
    signal s_ur11, s_vr11 : signed(21 downto 0) := (others => '0');
    signal s_u12, s_v12 : signed(10 downto 0) := (others => '0');
    signal s_ugh13, s_vgh13 : signed(15 downto 0) := (others => '0');
    signal s_ugl13, s_vgl13 : signed(16 downto 0) := (others => '0');
    signal s_u14, s_v14 : signed(10 downto 0) := (others => '0');
    signal s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');
    signal s_dy15 : signed(10 downto 0) := (others => '0');
    signal s_pm16 : signed(19 downto 0) := (others => '0');
    signal s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_hue    <= unsigned(registers_in(0));
    s_k_sat    <= unsigned(registers_in(1));
    s_k_spread <= unsigned(registers_in(2));
    s_k_plane  <= unsigned(registers_in(3));
    s_k_mix    <= unsigned(registers_in(4));
    s_k_cpl    <= unsigned(registers_in(5));
    s_sw_negy   <= registers_in(6)(0);
    s_sw_negc   <= registers_in(6)(1);
    s_sw_flip   <= registers_in(6)(2);
    s_sw_satsrc <= registers_in(6)(3);
    s_sw_after  <= registers_in(6)(4);
    s_p_spin <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_m : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_saw_active <= '1'; end if;

            s_hue <= s_k_hue(9 downto 2);
            s_gain <= s_k_sat(9 downto 1); s_gain_a <= s_gain;
            s_spread <= s_k_spread(9 downto 2);
            s_plane <= resize(s_k_plane(9 downto 7), 4) + 1;    -- 1..8; 1 means "negative" below
            s_mix <= s_k_mix(9 downto 2);
            case to_integer(s_k_cpl(9 downto 7)) is
                when 0 => s_cmask <= (others => '1');
                when 1 => s_cmask <= "1000000000";
                when 2 => s_cmask <= "1100000000";
                when 3 => s_cmask <= "1110000000";
                when 4 => s_cmask <= "1111000000";
                when 5 => s_cmask <= "1111100000";
                when 6 => s_cmask <= "1111110000";
                when others => s_cmask <= "1111111000";
            end case;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_saw_active = '1' then
                    s_phase <= s_phase + unsigned(resize(shift_right(signed(resize(s_p_spin, 11)) - 512, 1), 16));
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

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin8 <= to_signed(C_SINE(to_integer(s_sa7)), 10);
            s_cos8 <= to_signed(C_SINE(to_integer(s_ca7)), 10);
            s_sin9 <= s_sin8; s_cos9 <= s_cos8;
        end if;
    end process;

    p_pix : process(clk)
        variable v_src : unsigned(9 downto 0);
        variable v_bit : std_logic;
        variable v_pl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2
            s_u2 <= signed(resize(s_in_u, 11)) - 512;
            s_v2 <= signed(resize(s_in_v, 11)) - 512;
            if s_sw_negy = '1' then s_y2 <= not s_in_y; else s_y2 <= s_in_y; end if;
            if s_in_u >= 512 then s_au2 <= s_in_u - 512; else s_au2 <= 512 - s_in_u; end if;
            if s_in_v >= 512 then s_av2 <= s_in_v - 512; else s_av2 <= 512 - s_in_v; end if;
            -- 3
            s_sat3 <= resize(s_au2, 10) + resize(s_av2, 10);
            s_y3 <= s_y2;
            if s_sw_negc = '1' then s_u3 <= -s_u2; s_v3 <= -s_v2; else s_u3 <= s_u2; s_v3 <= s_v2; end if;
            -- 4: plane bit and plane picture
            if s_sw_satsrc = '1' then v_src := s_sat3; else v_src := s_y3; end if;
            if s_plane = 1 then
                v_bit := '0'; v_pl := not s_y3;
            else
                v_bit := v_src(10 - to_integer(s_plane));
                if v_bit = '1' then v_pl := to_unsigned(940, 10); else v_pl := to_unsigned(64, 10); end if;
            end if;
            s_bit4 <= v_bit; s_pl4 <= v_pl; s_y4 <= s_y3; s_u4 <= s_u3; s_v4 <= s_v3;
            s_band4 <= s_y3(9 downto 7);
            -- 5
            s_bs5 <= s_band4 * s_spread;
            s_bit5 <= s_bit4; s_pl5 <= s_pl4; s_y5 <= s_y4;
            -- chroma bit-plane (before rotation when Order = before)
            if s_sw_after = '0' then
                s_u5 <= signed(unsigned(s_u4) and ('1' & s_cmask)); s_v5 <= signed(unsigned(s_v4) and ('1' & s_cmask));
            else
                s_u5 <= s_u4; s_v5 <= s_v4;
            end if;
            -- 6: angle
            if s_sw_flip = '1' and s_bit5 = '1' then
                s_th6 <= s_hue + s_phase(15 downto 8) + s_bs5(10 downto 3) + 128;
            else
                s_th6 <= s_hue + s_phase(15 downto 8) + s_bs5(10 downto 3);
            end if;
            s_pl6 <= s_pl5; s_y6 <= s_y5; s_u6 <= s_u5; s_v6 <= s_v5;
            -- 7: ROM addresses
            s_sa7 <= s_th6; s_ca7 <= s_th6 + 64;
            s_ud(7) <= s_u6; s_vd(7) <= s_v6; s_pld(7) <= s_pl6; s_yd(7) <= s_y6;
            for i in 8 to 9 loop s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            for i in 8 to 17 loop s_pld(i) <= s_pld(i - 1); s_yd(i) <= s_yd(i - 1); end loop;
            -- 10: products
            s_uc10 <= s_ud(9) * s_cos9; s_us10 <= s_ud(9) * s_sin9;
            s_vc10 <= s_vd(9) * s_cos9; s_vs10 <= s_vd(9) * s_sin9;
            -- 11
            s_ur11 <= resize(s_uc10, 22) - resize(s_vs10, 22);
            s_vr11 <= resize(s_us10, 22) + resize(s_vc10, 22);
            -- 12
            s_u12 <= f_clamp511(shift_right(s_ur11, 8));
            s_v12 <= f_clamp511(shift_right(s_vr11, 8));
            -- 13: gain (split)
            s_ugh13 <= s_u12 * signed(resize(s_gain_a(8 downto 5), 5));
            s_ugl13 <= s_u12 * signed(resize(s_gain_a(4 downto 0), 6));
            s_vgh13 <= s_v12 * signed(resize(s_gain_a(8 downto 5), 5));
            s_vgl13 <= s_v12 * signed(resize(s_gain_a(4 downto 0), 6));
            -- 14
            s_u14 <= f_clamp511(shift_right(shift_left(resize(s_ugh13, 22), 5) + resize(s_ugl13, 22), 7));
            s_v14 <= f_clamp511(shift_right(shift_left(resize(s_vgh13, 22), 5) + resize(s_vgl13, 22), 7));
            -- 15: chroma plane after, recentre; luma plane mix difference
            if s_sw_after = '1' then
                s_u15 <= f_clamp10(resize(signed(unsigned(s_u14) and ('1' & s_cmask)), 13) + 512);
                s_v15 <= f_clamp10(resize(signed(unsigned(s_v14) and ('1' & s_cmask)), 13) + 512);
            else
                s_u15 <= f_clamp10(resize(s_u14, 13) + 512);
                s_v15 <= f_clamp10(resize(s_v14, 13) + 512);
            end if;
            s_dy15 <= signed(resize(s_pld(14), 11)) - signed(resize(s_yd(14), 11));
            -- 16
            s_pm16 <= s_dy15 * signed(resize(s_mix, 9));
            s_u16 <= s_u15; s_v16 <= s_v15;
            -- 17
            s_y17 <= f_clamp10(signed(resize(s_yd(16), 13)) + resize(shift_right(s_pm16, 8), 13));
            s_u17 <= s_u16; s_v17 <= s_v16;
            -- 18: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y18);
    data_out.u       <= std_logic_vector(s_u18);
    data_out.v       <= std_logic_vector(s_v18);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture negspin;
