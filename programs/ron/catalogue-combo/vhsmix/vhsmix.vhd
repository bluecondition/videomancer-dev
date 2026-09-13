-- Vhsmix: catalogue combo #87 (seed 20260923) -- posterization + head-switch
-- noise band + multi-source mixing (add, multiply, screen, difference,
-- overlay).  Two knobs each; the switches and the slider decide how they
-- combine.
--
--   K1 Luma     posterize luma 1..8 bits
--   K2 Chroma   posterize chroma 1..8 bits
--   K3 Band     head-switch band height (0..255 lines)
--   K4 Noise    noise amplitude in the band
--   K5 Blend    add / screen / multiply / difference / overlay
--   K6 Mix      blend amount (the posterized picture mixed onto the original)
--
--   S7  Order   blend(posterized, original) / blend(original, posterized)
--   S8  Band    band shows the raw blend / the band is the posterized negative
--   S9  Roll    band fixed at the bottom / rolling upward
--   S10 Noise   noise on luma / noise on chroma too
--   S11 Chroma  blended chroma from the posterized picture / kept original
--   P12 Static  noise fills the whole picture (0 = band only)
--
-- Per-pixel chain, no line memory.  Latency 15 clocks, all modes,
-- blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture vhsmix of program_top is

    constant C_LAT : integer := 15;

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

    function f_clamp959(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 959 then
            return to_unsigned(959, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    signal s_k_lbits, s_k_cbits, s_k_band, s_k_noise, s_k_blend, s_k_mix : unsigned(9 downto 0);
    signal s_sw_swap, s_sw_bandneg, s_sw_roll, s_sw_cnoise, s_sw_keepc : std_logic;
    signal s_p_static : unsigned(9 downto 0);

    signal s_kl, s_kc : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_lnot, s_lhalf, s_cnot, s_chalf : unsigned(9 downto 0) := (others => '0');
    signal s_bandh, s_noise, s_mix, s_static : unsigned(7 downto 0) := (others => '0');
    signal s_mode : unsigned(2 downto 0) := (others => '0');
    signal s_rollpos, s_hs_top, s_height : unsigned(10 downto 0) := (others => '0');
    signal s_vstep : unsigned(2 downto 0) := "111";
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal s_saw_active : std_logic := '0';
    signal s_inhs : std_logic := '0';
    signal s_namp : unsigned(7 downto 0) := (others => '0');

    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line : unsigned(10 downto 0) := (others => '0');

    -- 2
    signal s_yq2, s_uq2, s_vq2 : unsigned(9 downto 0) := (others => '0');
    signal s_nz2 : unsigned(15 downto 0) := (others => '0');
    -- 3
    signal s_a3, s_b3 : unsigned(9 downto 0) := (others => '0');
    signal s_pn3, s_pu3 : unsigned(15 downto 0) := (others => '0');
    signal s_uq3, s_vq3 : unsigned(9 downto 0) := (others => '0');
    -- 4
    signal s_p4, s_q4 : unsigned(19 downto 0) := (others => '0');
    signal s_sum4 : unsigned(10 downto 0) := (others => '0');
    signal s_dif4 : signed(10 downto 0) := (others => '0');
    signal s_a4 : unsigned(9 downto 0) := (others => '0');
    signal s_ny4 : signed(9 downto 0) := (others => '0');
    signal s_nu4 : signed(9 downto 0) := (others => '0');
    -- 5
    signal s_c_add5, s_c_scr5, s_c_mul5, s_c_dif5, s_c_ovl5, s_ovh5 : signed(11 downto 0) := (others => '0');
    signal s_alo5 : std_logic := '0';
    -- 6
    signal s_sel6 : signed(11 downto 0) := (others => '0');
    -- 7
    signal s_t7 : unsigned(9 downto 0) := (others => '0');
    -- 8
    signal s_dy8, s_du8, s_dv8 : signed(10 downto 0) := (others => '0');
    -- 9
    signal s_py9, s_pu9, s_pv9 : signed(19 downto 0) := (others => '0');
    -- 10
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    -- 11
    signal s_y11, s_u11, s_v11 : signed(11 downto 0) := (others => '0');
    -- 12..15
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');
    type t_c10 is array (2 to 10) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd, s_yqd, s_uqd, s_vqd : t_c10 := (others => (others => '0'));
    type t_n is array (4 to 10) of signed(9 downto 0);
    signal s_nyd, s_nud : t_n := (others => (others => '0'));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_lbits <= unsigned(registers_in(0));
    s_k_cbits <= unsigned(registers_in(1));
    s_k_band  <= unsigned(registers_in(2));
    s_k_noise <= unsigned(registers_in(3));
    s_k_blend <= unsigned(registers_in(4));
    s_k_mix   <= unsigned(registers_in(5));
    s_sw_swap    <= registers_in(6)(0);
    s_sw_bandneg <= registers_in(6)(1);
    s_sw_roll    <= registers_in(6)(2);
    s_sw_cnoise  <= registers_in(6)(3);
    s_sw_keepc   <= registers_in(6)(4);
    s_p_static <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_m : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_saw_active <= '1'; end if;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

            s_kl <= to_unsigned(9, 4) - resize(s_k_lbits(9 downto 7), 4);
            s_kc <= to_unsigned(9, 4) - resize(s_k_cbits(9 downto 7), 4);
            v_m := shift_left(to_unsigned(1, 10), to_integer(s_kl)) - 1;
            s_lnot <= not v_m; s_lhalf <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 1);
            v_m := shift_left(to_unsigned(1, 10), to_integer(s_kc)) - 1;
            s_cnot <= not v_m; s_chalf <= shift_left(to_unsigned(1, 10), to_integer(s_kc) - 1);
            s_bandh <= s_k_band(9 downto 2); s_noise <= s_k_noise(9 downto 2); s_mix <= s_k_mix(9 downto 2);
            s_static <= s_p_static(9 downto 2);
            case to_integer(s_k_blend(9 downto 7)) is
                when 0 | 1 => s_mode <= "000";
                when 2 | 3 => s_mode <= "001";
                when 4     => s_mode <= "010";
                when 5 | 6 => s_mode <= "011";
                when others => s_mode <= "100";
            end case;

            case to_integer(s_vstep) is
                when 0 =>
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

            if s_prev_avid = '1' and data_in.avid = '0' then s_line <= s_line + 1; s_height <= s_line + 1; end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_line >= s_hs_top and s_line < s_hs_top + resize(s_bandh, 11) then s_inhs <= '1'; else s_inhs <= '0'; end if;
            end if;
            if s_inhs = '1' then s_namp <= s_noise; else s_namp <= s_static; end if;
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

    p_pix : process(clk)
        variable v_a, v_b : unsigned(9 downto 0);
        variable v_u, v_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: posterize
            s_yq2 <= (s_in_y and s_lnot) + s_lhalf;
            s_uq2 <= (s_in_u and s_cnot) + s_chalf;
            s_vq2 <= (s_in_v and s_cnot) + s_chalf;
            s_nz2 <= s_lfsr;
            s_yd(2) <= s_in_y; s_ud(2) <= s_in_u; s_vd(2) <= s_in_v;
            -- 3: black-relative operands (with swap), noise products
            if s_yq2 < 64 then v_a := (others => '0'); else v_a := s_yq2 - 64; end if;
            if s_yd(2) < 64 then v_b := (others => '0'); else v_b := s_yd(2) - 64; end if;
            if s_sw_swap = '1' then s_a3 <= v_b; s_b3 <= v_a; else s_a3 <= v_a; s_b3 <= v_b; end if;
            s_pn3 <= s_namp * s_nz2(7 downto 0);
            s_pu3 <= s_namp * s_nz2(15 downto 8);
            s_uq3 <= s_uq2; s_vq3 <= s_vq2;
            -- 4
            s_p4 <= s_a3 * s_b3;
            s_q4 <= (959 - s_a3) * (959 - s_b3);
            s_sum4 <= resize(s_a3, 11) + resize(s_b3, 11);
            s_dif4 <= signed(resize(s_a3, 11)) - signed(resize(s_b3, 11));
            s_a4 <= s_a3;
            s_ny4 <= signed(resize(s_pn3(15 downto 7), 10)) - signed(resize(s_namp, 10));
            s_nu4 <= signed(resize(s_pu3(15 downto 7), 10)) - signed(resize(s_namp, 10));
            -- 5
            s_c_add5 <= signed(resize(s_sum4, 12));
            s_c_scr5 <= signed(resize(s_sum4, 12)) - signed(resize(s_p4(19 downto 10), 12));
            s_c_mul5 <= signed(resize(s_p4(19 downto 10), 12));
            if s_dif4 < 0 then s_c_dif5 <= -resize(s_dif4, 12); else s_c_dif5 <= resize(s_dif4, 12); end if;
            s_c_ovl5 <= signed(resize(s_p4(19 downto 9), 12));
            s_ovh5 <= to_signed(959, 12) - signed(resize(s_q4(19 downto 9), 12));
            if s_a4 < 480 then s_alo5 <= '1'; else s_alo5 <= '0'; end if;
            -- 6
            case to_integer(s_mode) is
                when 0 => s_sel6 <= s_c_add5;
                when 1 => s_sel6 <= s_c_scr5;
                when 2 => s_sel6 <= s_c_mul5;
                when 3 => s_sel6 <= s_c_dif5;
                when others => if s_alo5 = '1' then s_sel6 <= s_c_ovl5; else s_sel6 <= s_ovh5; end if;
            end case;
            -- 7
            s_t7 <= f_clamp959(s_sel6);
            -- 8: differences toward the blend and posterized chroma
            if s_yd(7) < 64 then v_b := (others => '0'); else v_b := s_yd(7) - 64; end if;
            s_dy8 <= signed(resize(s_t7, 11)) - signed(resize(v_b, 11));
            if s_sw_keepc = '1' then
                s_du8 <= (others => '0'); s_dv8 <= (others => '0');
            else
                s_du8 <= signed(resize(s_uqd(7), 11)) - signed(resize(s_ud(7), 11));
                s_dv8 <= signed(resize(s_vqd(7), 11)) - signed(resize(s_vd(7), 11));
            end if;
            -- 9
            s_py9 <= s_dy8 * signed(resize(s_mix, 9));
            s_pu9 <= s_du8 * signed(resize(s_mix, 9));
            s_pv9 <= s_dv8 * signed(resize(s_mix, 9));
            -- 10
            s_y10 <= f_clamp10(signed(resize(s_yd(9), 13)) + resize(shift_right(s_py9, 8), 13));
            s_u10 <= f_clamp10(signed(resize(s_ud(9), 13)) + resize(shift_right(s_pu9, 8), 13));
            s_v10 <= f_clamp10(signed(resize(s_vd(9), 13)) + resize(shift_right(s_pv9, 8), 13));
            -- 11: noise, band negative
            if s_inhs = '1' and s_sw_bandneg = '1' then
                s_y11 <= signed(resize(not s_yqd(10), 12)) + resize(s_nyd(10), 12);
                s_u11 <= signed(resize(not s_uqd(10), 12)); s_v11 <= signed(resize(not s_vqd(10), 12));
            else
                s_y11 <= signed(resize(s_y10, 12)) + resize(s_nyd(10), 12);
                if s_sw_cnoise = '1' then
                    s_u11 <= signed(resize(s_u10, 12)) + resize(s_nud(10), 12);
                    s_v11 <= signed(resize(s_v10, 12)) - resize(s_nud(10), 12);
                else
                    s_u11 <= signed(resize(s_u10, 12)); s_v11 <= signed(resize(s_v10, 12));
                end if;
            end if;
            -- 12
            s_y12 <= f_clamp10(s_y11); s_u12 <= f_clamp10(s_u11); s_v12 <= f_clamp10(s_v11);
            s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            -- 15: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            else
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            end if;
            -- chains
            s_yqd(2) <= s_yq2; s_uqd(2) <= s_uq2; s_vqd(2) <= s_vq2;
            for i in 3 to 10 loop
                s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1);
                s_yqd(i) <= s_yqd(i - 1); s_uqd(i) <= s_uqd(i - 1); s_vqd(i) <= s_vqd(i - 1);
            end loop;
            s_nyd(4) <= s_ny4; s_nud(4) <= s_nu4;
            for i in 5 to 10 loop s_nyd(i) <= s_nyd(i - 1); s_nud(i) <= s_nud(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y15);
    data_out.u       <= std_logic_vector(s_u15);
    data_out.v       <= std_logic_vector(s_v15);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture vhsmix;
