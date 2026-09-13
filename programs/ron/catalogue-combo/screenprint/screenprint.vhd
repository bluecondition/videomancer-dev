-- Screenprint: catalogue combo #77 (seed 20260922) -- ASCII / dither-art /
-- halftone + multi-source mixing (add, multiply, screen, difference,
-- overlay).  Three knobs each; the switches and the slider decide how they
-- combine.
--
--   K1 Cell     halftone cell 4 / 8 / 16 / 32 px
--   K2 Angle    screen angle: axis / 45 degrees / steep
--   K3 Dot      dot shape: circle / diamond / square / line
--   K4 Blend    add / screen / multiply / difference / overlay
--   K5 Ink      ink hue (8 hues)
--   K6 Gain     halftone contrast
--
--   S7  Ink     black ink on white paper / coloured ink
--   S8  Invert  halftone inverted
--   S9  Order   halftone over picture / picture over halftone (operand swap)
--   S10 Chroma  picture chroma / ink chroma where there is ink
--   S11 Rosette a second screen at another angle (rosette)
--   P12 Mix     10 .. 100% of the blend
--
-- Per-pixel chain, no line memory: dot screens are cell-local sums of
-- squares / abs values against a gain-scaled luma; blend modes are two 10x10
-- products.  Latency 17 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture screenprint of program_top is

    constant C_LAT : integer := 17;

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

    type t_tab is array (0 to 7) of integer range -256 to 255;
    constant C_TU : t_tab := (200, 141, 0, -141, -200, -141, 0, 141);
    constant C_TV : t_tab := (0, 141, 200, 141, 0, -141, -200, -141);

    signal s_k_cell, s_k_ang, s_k_dot, s_k_blend, s_k_ink, s_k_gain : unsigned(9 downto 0);
    signal s_sw_colink, s_sw_inv, s_sw_swap, s_sw_inkchroma, s_sw_rosette : std_logic;
    signal s_p_mix : unsigned(9 downto 0);

    signal s_kc : unsigned(2 downto 0) := "011";     -- log2 cell
    signal s_cmask : unsigned(5 downto 0) := (others => '0');
    signal s_half : unsigned(5 downto 0) := (others => '0');
    signal s_ang : unsigned(1 downto 0) := "00";
    signal s_dot : unsigned(1 downto 0) := "00";
    signal s_mode : unsigned(2 downto 0) := "000";
    signal s_iu, s_iv : signed(10 downto 0) := (others => '0');
    signal s_gain : unsigned(7 downto 0) := (others => '0');
    signal s_mix : unsigned(7 downto 0) := (others => '0');
    signal s_shc, s_shd, s_shs, s_sh : unsigned(3 downto 0) := (others => '0');   -- normalising shifts

    signal s_rx : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line : unsigned(10 downto 0) := (others => '0');

    -- 2
    signal s_xr2, s_yr2, s_xs2, s_ys2 : unsigned(11 downto 0) := (others => '0');
    signal s_yl2 : unsigned(9 downto 0) := (others => '0');
    -- 3
    signal s_cx3, s_cy3, s_cx3b, s_cy3b : signed(6 downto 0) := (others => '0');
    signal s_yg3 : unsigned(17 downto 0) := (others => '0');
    -- 4
    signal s_ax4, s_ay4, s_ax4b, s_ay4b : unsigned(5 downto 0) := (others => '0');
    signal s_sx4, s_sy4, s_sx4b, s_sy4b : unsigned(11 downto 0) := (others => '0');
    signal s_yg4 : unsigned(10 downto 0) := (others => '0');
    -- 5
    signal s_d5, s_d5b : unsigned(12 downto 0) := (others => '0');
    signal s_yg5 : unsigned(12 downto 0) := (others => '0');
    -- 6
    signal s_t6, s_t6b : unsigned(12 downto 0) := (others => '0');
    signal s_yg6 : unsigned(12 downto 0) := (others => '0');
    -- 7
    signal s_ink7 : std_logic := '0';
    -- 8
    signal s_a8, s_b8 : unsigned(9 downto 0) := (others => '0');
    signal s_ink8 : std_logic := '0';
    -- 9
    signal s_p9, s_q9 : unsigned(19 downto 0) := (others => '0');
    signal s_sum9 : unsigned(10 downto 0) := (others => '0');
    signal s_dif9 : signed(10 downto 0) := (others => '0');
    signal s_a9 : unsigned(9 downto 0) := (others => '0');
    -- 10
    signal s_c_add10, s_c_scr10, s_c_mul10, s_c_dif10, s_c_ovl10, s_ovh10 : signed(11 downto 0) := (others => '0');
    signal s_alo10 : std_logic := '0';
    -- 11
    signal s_sel11 : signed(11 downto 0) := (others => '0');
    -- 12
    signal s_t12 : unsigned(9 downto 0) := (others => '0');
    -- 13
    signal s_dy13, s_du13, s_dv13 : signed(10 downto 0) := (others => '0');
    -- 14
    signal s_py14, s_pu14, s_pv14 : signed(19 downto 0) := (others => '0');
    -- 15
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    type t_c10 is array (2 to 14) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_c10 := (others => (others => '0'));
    type t_ink is array (8 to 12) of std_logic;
    signal s_inkd : t_ink := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_cell  <= unsigned(registers_in(0));
    s_k_ang   <= unsigned(registers_in(1));
    s_k_dot   <= unsigned(registers_in(2));
    s_k_blend <= unsigned(registers_in(3));
    s_k_ink   <= unsigned(registers_in(4));
    s_k_gain  <= unsigned(registers_in(5));
    s_sw_colink    <= registers_in(6)(0);
    s_sw_inv       <= registers_in(6)(1);
    s_sw_swap      <= registers_in(6)(2);
    s_sw_inkchroma <= registers_in(6)(3);
    s_sw_rosette   <= registers_in(6)(4);
    s_p_mix <= unsigned(registers_in(7));

    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; else s_rx <= (others => '0'); end if;

            case to_integer(s_k_cell(9 downto 8)) is
                when 0 => s_kc <= "010"; s_cmask <= "000011"; s_half <= "000010"; s_shc <= "0111"; s_shd <= "1000"; s_shs <= "1001";
                when 1 => s_kc <= "011"; s_cmask <= "000111"; s_half <= "000100"; s_shc <= "0101"; s_shd <= "0111"; s_shs <= "1000";
                when 2 => s_kc <= "100"; s_cmask <= "001111"; s_half <= "001000"; s_shc <= "0011"; s_shd <= "0110"; s_shs <= "0111";
                when others => s_kc <= "101"; s_cmask <= "011111"; s_half <= "010000"; s_shc <= "0001"; s_shd <= "0101"; s_shs <= "0110";
            end case;
            case to_integer(s_k_ang(9 downto 8)) is
                when 0 | 1 => s_ang <= "00"; when 2 => s_ang <= "01"; when others => s_ang <= "10";
            end case;
            s_dot <= s_k_dot(9 downto 8);
            case to_integer(s_dot) is
                when 0 => s_sh <= s_shc;
                when 1 => s_sh <= s_shd;
                when others => s_sh <= s_shs;
            end case;
            case to_integer(s_k_blend(9 downto 7)) is
                when 0 | 1 => s_mode <= "000";
                when 2 | 3 => s_mode <= "001";
                when 4     => s_mode <= "010";
                when 5 | 6 => s_mode <= "011";
                when others => s_mode <= "100";
            end case;
            s_iu <= to_signed(C_TU(to_integer(s_k_ink(9 downto 7))), 11);
            s_iv <= to_signed(C_TV(to_integer(s_k_ink(9 downto 7))), 11);
            s_gain <= s_k_gain(9 downto 2);
            s_mix <= to_unsigned(26, 8) + resize(s_p_mix(9 downto 2), 8) - resize(s_p_mix(9 downto 5), 8);   -- 26..255

            if s_prev_avid = '1' and data_in.avid = '0' then s_line <= s_line + 1; end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then s_line <= (others => '0'); end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    p_pix : process(clk)
        variable v_d, v_db : unsigned(12 downto 0);
        variable v_t, v_tb : unsigned(12 downto 0);
        variable v_ink : std_logic;
        variable v_hy : unsigned(9 downto 0);
        variable v_a, v_b : unsigned(9 downto 0);
        variable v_cu, v_cv : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: screen coordinates (main and rosette screen), luma
            case to_integer(s_ang) is
                when 0 => s_xr2 <= resize(s_rx, 12); s_yr2 <= resize(s_line, 12);
                when 1 => s_xr2 <= resize(s_rx, 12) + resize(s_line, 12); s_yr2 <= resize(s_rx, 12) - resize(s_line, 12);
                when others => s_xr2 <= resize(s_rx, 12) + (resize(s_line, 12) sll 1); s_yr2 <= (resize(s_rx, 12) sll 1) - resize(s_line, 12);
            end case;
            s_xs2 <= resize(s_rx, 12) - resize(s_line, 12); s_ys2 <= resize(s_rx, 12) + resize(s_line, 12);
            if s_in_y < 64 then s_yl2 <= (others => '0'); else s_yl2 <= s_in_y - 64; end if;
            s_yd(2) <= s_in_y; s_ud(2) <= s_in_u; s_vd(2) <= s_in_v;
            -- 3: cell-local coordinates, gain product
            s_cx3 <= signed(resize(s_xr2(5 downto 0) and s_cmask, 7)) - signed(resize(s_half, 7));
            s_cy3 <= signed(resize(s_yr2(5 downto 0) and s_cmask, 7)) - signed(resize(s_half, 7));
            s_cx3b <= signed(resize(s_xs2(5 downto 0) and s_cmask, 7)) - signed(resize(s_half, 7));
            s_cy3b <= signed(resize(s_ys2(5 downto 0) and s_cmask, 7)) - signed(resize(s_half, 7));
            s_yg3 <= s_yl2 * s_gain;
            -- 4: abs and squares
            if s_cx3 < 0 then s_ax4 <= unsigned(resize(-s_cx3, 6)); else s_ax4 <= unsigned(resize(s_cx3, 6)); end if;
            if s_cy3 < 0 then s_ay4 <= unsigned(resize(-s_cy3, 6)); else s_ay4 <= unsigned(resize(s_cy3, 6)); end if;
            if s_cx3b < 0 then s_ax4b <= unsigned(resize(-s_cx3b, 6)); else s_ax4b <= unsigned(resize(s_cx3b, 6)); end if;
            if s_cy3b < 0 then s_ay4b <= unsigned(resize(-s_cy3b, 6)); else s_ay4b <= unsigned(resize(s_cy3b, 6)); end if;
            s_sx4 <= unsigned(resize(s_cx3 * s_cx3, 12)); s_sy4 <= unsigned(resize(s_cy3 * s_cy3, 12));
            s_sx4b <= unsigned(resize(s_cx3b * s_cx3b, 12)); s_sy4b <= unsigned(resize(s_cy3b * s_cy3b, 12));
            s_yg4 <= s_yg3(17 downto 7);
            -- 5: distance measure
            case to_integer(s_dot) is
                when 0 => v_d := resize(s_sx4, 13) + resize(s_sy4, 13); v_db := resize(s_sx4b, 13) + resize(s_sy4b, 13);
                when 1 => v_d := resize(s_ax4, 13) + resize(s_ay4, 13); v_db := resize(s_ax4b, 13) + resize(s_ay4b, 13);
                when 2 =>
                    if s_ax4 > s_ay4 then v_d := resize(s_ax4, 13); else v_d := resize(s_ay4, 13); end if;
                    if s_ax4b > s_ay4b then v_db := resize(s_ax4b, 13); else v_db := resize(s_ay4b, 13); end if;
                when others => v_d := resize(s_ay4, 13); v_db := resize(s_ay4b, 13);
            end case;
            s_d5 <= v_d; s_d5b <= v_db;
            s_yg5 <= shift_right(resize(s_yg4, 13), to_integer(s_sh));      -- luma into distance units (barrel alone)
            -- 6: copies (distance stays in native units)
            s_t6 <= s_d5; s_t6b <= s_d5b; s_yg6 <= s_yg5;
            -- 7: ink decision (dark luma -> big dots)
            v_ink := '0';
            if s_yg6 < s_t6 then v_ink := '1'; end if;
            if s_sw_rosette = '1' and s_yg6 < s_t6b then v_ink := '1'; end if;
            s_ink7 <= v_ink xor s_sw_inv;
            -- 8: black-relative operands a (halftone) and b (picture), with swap
            if s_ink7 = '1' then v_hy := (others => '0'); else v_hy := to_unsigned(876, 10); end if;
            if s_yd(7) < 64 then v_b := (others => '0'); else v_b := s_yd(7) - 64; end if;
            if s_sw_swap = '1' then s_a8 <= v_b; s_b8 <= v_hy; else s_a8 <= v_hy; s_b8 <= v_b; end if;
            s_ink8 <= s_ink7;
            -- 9: products
            s_p9 <= s_a8 * s_b8;
            s_q9 <= (959 - s_a8) * (959 - s_b8);
            s_sum9 <= resize(s_a8, 11) + resize(s_b8, 11);
            s_dif9 <= signed(resize(s_a8, 11)) - signed(resize(s_b8, 11));
            s_a9 <= s_a8;
            -- 10: candidates
            s_c_add10 <= signed(resize(s_sum9, 12));
            s_c_scr10 <= signed(resize(s_sum9, 12)) - signed(resize(s_p9(19 downto 10), 12));
            s_c_mul10 <= signed(resize(s_p9(19 downto 10), 12));
            if s_dif9 < 0 then s_c_dif10 <= -resize(s_dif9, 12); else s_c_dif10 <= resize(s_dif9, 12); end if;
            s_c_ovl10 <= signed(resize(s_p9(19 downto 9), 12));
            s_ovh10 <= to_signed(959, 12) - signed(resize(s_q9(19 downto 9), 12));
            if s_a9 < 480 then s_alo10 <= '1'; else s_alo10 <= '0'; end if;
            -- 11: select
            case to_integer(s_mode) is
                when 0 => s_sel11 <= s_c_add10;
                when 1 => s_sel11 <= s_c_scr10;
                when 2 => s_sel11 <= s_c_mul10;
                when 3 => s_sel11 <= s_c_dif10;
                when others => if s_alo10 = '1' then s_sel11 <= s_c_ovl10; else s_sel11 <= s_ovh10; end if;
            end case;
            -- 12: clamp
            s_t12 <= f_clamp959(s_sel11);
            -- 13: differences toward the blend (luma) and ink chroma
            if s_yd(12) < 64 then v_b := (others => '0'); else v_b := s_yd(12) - 64; end if;
            s_dy13 <= signed(resize(s_t12, 11)) - signed(resize(v_b, 11));
            if s_sw_inkchroma = '1' and s_inkd(11) = '1' and s_sw_colink = '1' then
                v_cu := resize(s_iu, 12) + 512; v_cv := resize(s_iv, 12) + 512;
            elsif s_sw_inkchroma = '1' and s_inkd(11) = '1' then
                v_cu := to_signed(512, 12); v_cv := to_signed(512, 12);
            else
                v_cu := signed(resize(s_ud(12), 12)); v_cv := signed(resize(s_vd(12), 12));
            end if;
            s_du13 <= resize(v_cu - signed(resize(s_ud(12), 12)), 11);
            s_dv13 <= resize(v_cv - signed(resize(s_vd(12), 12)), 11);
            -- 14: mix products
            s_py14 <= s_dy13 * signed(resize(s_mix, 9));
            s_pu14 <= s_du13 * signed(resize(s_mix, 9));
            s_pv14 <= s_dv13 * signed(resize(s_mix, 9));
            -- 15: apply
            s_y15 <= f_clamp10(signed(resize(s_yd(14), 13)) + resize(shift_right(s_py14, 8), 13));
            s_u15 <= f_clamp10(signed(resize(s_ud(14), 13)) + resize(shift_right(s_pu14, 8), 13));
            s_v15 <= f_clamp10(signed(resize(s_vd(14), 13)) + resize(shift_right(s_pv14, 8), 13));
            -- 16, 17 gate
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            for i in 3 to 14 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            s_inkd(8) <= s_ink8;
            for i in 9 to 12 loop s_inkd(i) <= s_inkd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y17);
    data_out.u       <= std_logic_vector(s_u17);
    data_out.v       <= std_logic_vector(s_v17);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture screenprint;
