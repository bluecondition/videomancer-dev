-- Solarcell: catalogue combo #81 (seed 20260923) -- Sabattier solarization
-- + mosaic with per-cell colour averaging.  Three knobs each; the switches
-- and the slider decide how they combine.
--
--   K1 Threshold solarize threshold (fold above it)
--   K2 Gain     fold depth
--   K3 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 lines
--   K4 Cell     cell width 4 / 8 / 16 / 32 / 64 px
--   K5 Mix      mosaic mixed over the picture (0..100%)
--   K6 Offset   solarize threshold offset per cell row (staircase)
--
--   S7  Chroma  solarize luma only / chroma inverted in the fold
--   S8  Order   solarize the mosaic / mosaic of the solarized picture
--   S9  Average luma and chroma averaged / luma averaged, chroma sampled
--   S10 Grid    seamless / thin dark cell borders
--   S11 Iso     black lines where the fold crosses a period
--   P12 Fold    threshold sweep: the fold moves through the picture
--
-- Cells are averaged horizontally on each row block's first line (running
-- sum reset per cell) into a two-bank cell RAM (read by cell index for the
-- whole block); solar_fold before the write or after the read.  Latency 18
-- clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture solarcell of program_top is

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

    type t_cell is array (0 to 511) of std_logic_vector(31 downto 0);
    signal cell0, cell1 : t_cell := (others => (others => '0'));
    signal s_crd0, s_crd1 : std_logic_vector(31 downto 0) := (others => '0');

    signal s_k_thr, s_k_gain, s_k_rows, s_k_cell, s_k_mix, s_k_off : unsigned(9 downto 0);
    signal s_sw_chroma, s_sw_mosfirst, s_sw_sample, s_sw_grid, s_sw_iso : std_logic;
    signal s_p_fold : unsigned(9 downto 0);

    signal s_thr : unsigned(9 downto 0) := (others => '0');
    signal s_gain : unsigned(11 downto 0) := (others => '0');
    signal s_base : signed(11 downto 0) := to_signed(64, 12);
    signal s_rshift : unsigned(2 downto 0) := (others => '0');
    signal s_cshift : unsigned(2 downto 0) := "011";
    signal s_cmask : unsigned(5 downto 0) := (others => '0');
    signal s_mix : unsigned(7 downto 0) := (others => '0');
    signal s_offr : unsigned(7 downto 0) := (others => '0');
    signal s_blk0, s_wbank, s_rbank_blk : std_logic := '0';
    signal s_thr_row : unsigned(9 downto 0) := (others => '0');
    signal s_tp : unsigned(13 downto 0) := (others => '0');
    signal s_lstep : unsigned(1 downto 0) := "11";

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line : unsigned(10 downto 0) := (others => '0');

    -- input-side solar (order = solar first)
    signal s_ya, s_ua, s_va, s_ysola : unsigned(9 downto 0) := (others => '0');
    signal s_rega : std_logic := '0';
    signal s_en_a, s_en_b : std_logic := '0';
    type t_x is array (1 to 12) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 12) of std_logic;
    signal s_avd : t_b := (others => '0');
    -- cell accumulation (input at d8 = solar output)
    signal s_sy : unsigned(15 downto 0) := (others => '0');
    signal s_su, s_sv : unsigned(13 downto 0) := (others => '0');
    signal s_last9 : std_logic := '0';
    signal s_cwe10 : std_logic := '0';
    signal s_cwa10 : unsigned(8 downto 0) := (others => '0');
    signal s_cwd10 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_ulast, s_vlast : unsigned(9 downto 0) := (others => '0');
    -- read side
    signal s_cra : unsigned(8 downto 0) := (others => '0');
    signal s_edge3 : std_logic := '0';
    type t_e is array (3 to 11) of std_logic;
    signal s_edged : t_e := (others => '0');
    signal s_my5, s_mu5, s_mv5 : unsigned(9 downto 0) := (others => '0');
    -- dry chain (input delayed to meet the cell read)
    type t_c10 is array (2 to 5) of unsigned(9 downto 0);
    signal s_dy, s_du, s_dv : t_c10 := (others => (others => '0'));
    signal s_ymix6, s_umix6, s_vmix6 : unsigned(9 downto 0) := (others => '0');
    signal s_dyd6, s_dud6, s_dvd6 : unsigned(9 downto 0) := (others => '0');
    signal s_d7y, s_d7u, s_d7v : signed(10 downto 0) := (others => '0');
    signal s_b7y, s_b7u, s_b7v : unsigned(9 downto 0) := (others => '0');
    signal s_p8y, s_p8u, s_p8v : signed(19 downto 0) := (others => '0');
    signal s_b8y, s_b8u, s_b8v : unsigned(9 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    -- output solar (order = mosaic first): 9 -> 16
    signal s_yb, s_ub, s_vb, s_ysolb : unsigned(9 downto 0) := (others => '0');
    signal s_regb : std_logic := '0';
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');
    type t_e2 is array (9 to 16) of std_logic;
    signal s_edgd2 : t_e2 := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_thr  <= unsigned(registers_in(0));
    s_k_gain <= unsigned(registers_in(1));
    s_k_rows <= unsigned(registers_in(2));
    s_k_cell <= unsigned(registers_in(3));
    s_k_mix  <= unsigned(registers_in(4));
    s_k_off  <= unsigned(registers_in(5));
    s_sw_chroma   <= registers_in(6)(0);
    s_sw_mosfirst <= registers_in(6)(1);
    s_sw_sample   <= registers_in(6)(2);
    s_sw_grid     <= registers_in(6)(3);
    s_sw_iso      <= registers_in(6)(4);
    s_p_fold <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_bl : unsigned(10 downto 0);
        variable v_t : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;

            s_gain <= to_unsigned(256, 12) + (s_k_gain(9 downto 0) & "00");
            case to_integer(s_k_rows(9 downto 7)) is
                when 0 => s_rshift <= "000";
                when 1 | 2 => s_rshift <= "001";
                when 3 => s_rshift <= "010";
                when 4 | 5 => s_rshift <= "011";
                when 6 => s_rshift <= "100";
                when others => s_rshift <= "101";
            end case;
            case to_integer(s_k_cell(9 downto 7)) is
                when 0 | 1 => s_cshift <= "010"; s_cmask <= "000011";
                when 2 | 3 => s_cshift <= "011"; s_cmask <= "000111";
                when 4 | 5 => s_cshift <= "100"; s_cmask <= "001111";
                when 6     => s_cshift <= "101"; s_cmask <= "011111";
                when others => s_cshift <= "110"; s_cmask <= "111111";
            end case;
            s_mix <= s_k_mix(9 downto 2); s_offr <= s_k_off(9 downto 2);
            s_en_a <= not s_sw_mosfirst; s_en_b <= s_sw_mosfirst;

            case to_integer(s_lstep) is
                when 0 =>
                    v_bl := shift_right(s_line, to_integer(s_rshift));
                    if shift_left(v_bl, to_integer(s_rshift)) = s_line then s_blk0 <= '1'; else s_blk0 <= '0'; end if;
                    s_wbank <= v_bl(0); s_rbank_blk <= not v_bl(0);
                    s_tp <= v_bl(5 downto 0) * s_offr;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    v_t := resize(s_k_thr, 12) + resize(s_p_fold(9 downto 1), 12) + resize(s_tp(13 downto 4), 12);
                    if v_t > 1023 then v_t := to_unsigned(1023, 12); end if;
                    s_thr_row <= v_t(9 downto 0);
                    s_lstep <= "11";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then s_line <= s_line + 1; end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then s_lstep <= (others => '0'); end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then s_line <= (others => '0'); end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    -- input-side solar: 1 -> 8
    solar_a : entity work.solar_fold
        port map (clk => clk, y_in => s_in_y, u_in => s_in_u, v_in => s_in_v,
                  thr => s_thr_row, gain => s_gain, base => s_base, fold_c => s_sw_chroma,
                  enable => s_en_a, y_out => s_ya, u_out => s_ua, v_out => s_va,
                  ysol => s_ysola, region => s_rega);

    p_acc : process(clk)
        variable v_lo : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            s_x(1) <= s_wr_x; s_avd(1) <= s_in_avid;
            for i in 2 to 12 loop s_x(i) <= s_x(i - 1); s_avd(i) <= s_avd(i - 1); end loop;
            -- 9: running cell sums of the d8 solar output (x aligned: s_x(7) is the d8 pixel)
            v_lo := s_x(7)(5 downto 0) and s_cmask;
            if v_lo = 0 then
                s_sy <= resize(s_ya, 16); s_su <= resize(s_ua(9 downto 2), 14); s_sv <= resize(s_va(9 downto 2), 14);
            else
                s_sy <= s_sy + resize(s_ya, 16); s_su <= s_su + resize(s_ua(9 downto 2), 14); s_sv <= s_sv + resize(s_va(9 downto 2), 14);
            end if;
            if v_lo = s_cmask then s_last9 <= '1'; else s_last9 <= '0'; end if;
            s_ulast <= s_ua; s_vlast <= s_va;
            -- 10: cell write (sum complete at 9 for the pixel that was last at 8)
            s_cwe10 <= s_last9 and s_blk0 and s_avd(8);
            s_cwa10 <= s_x(8)(10 downto 2);   -- cell index (refined below by shift)
            case to_integer(s_cshift) is
                when 2 => s_cwa10 <= s_x(8)(10 downto 2);
                    s_cwd10 <= std_logic_vector(s_sy(11 downto 2)) & std_logic_vector(s_su(9 downto 2)) & std_logic_vector(s_sv(9 downto 2)) & "000000";
                when 3 => s_cwa10 <= resize(s_x(8)(10 downto 3), 9);
                    s_cwd10 <= std_logic_vector(s_sy(12 downto 3)) & std_logic_vector(s_su(10 downto 3)) & std_logic_vector(s_sv(10 downto 3)) & "000000";
                when 4 => s_cwa10 <= resize(s_x(8)(10 downto 4), 9);
                    s_cwd10 <= std_logic_vector(s_sy(13 downto 4)) & std_logic_vector(s_su(11 downto 4)) & std_logic_vector(s_sv(11 downto 4)) & "000000";
                when 5 => s_cwa10 <= resize(s_x(8)(10 downto 5), 9);
                    s_cwd10 <= std_logic_vector(s_sy(14 downto 5)) & std_logic_vector(s_su(12 downto 5)) & std_logic_vector(s_sv(12 downto 5)) & "000000";
                when others => s_cwa10 <= resize(s_x(8)(10 downto 6), 9);
                    s_cwd10 <= std_logic_vector(s_sy(15 downto 6)) & std_logic_vector(s_su(13 downto 6)) & std_logic_vector(s_sv(13 downto 6)) & "000000";
            end case;
            if s_sw_sample = '1' then
                s_cwd10(21 downto 6) <= std_logic_vector(s_ulast(9 downto 2)) & std_logic_vector(s_vlast(9 downto 2));
            end if;
            -- read address for the current pixel (d2): its cell index; edge flag
            case to_integer(s_cshift) is
                when 2 => s_cra <= s_x(1)(10 downto 2);
                when 3 => s_cra <= resize(s_x(1)(10 downto 3), 9);
                when 4 => s_cra <= resize(s_x(1)(10 downto 4), 9);
                when 5 => s_cra <= resize(s_x(1)(10 downto 5), 9);
                when others => s_cra <= resize(s_x(1)(10 downto 6), 9);
            end case;
            if (s_x(1)(5 downto 0) and s_cmask) = 0 then s_edge3 <= '1'; else s_edge3 <= '0'; end if;
            s_edged(3) <= s_edge3;
            for i in 4 to 11 loop s_edged(i) <= s_edged(i - 1); end loop;
        end if;
    end process p_acc;

    p_c0 : process(clk)
    begin
        if rising_edge(clk) then
            s_crd0 <= cell0(to_integer(s_cra));
            if s_cwe10 = '1' and s_wbank = '0' then cell0(to_integer(s_cwa10)) <= s_cwd10; end if;
        end if;
    end process;
    p_c1 : process(clk)
    begin
        if rising_edge(clk) then
            s_crd1 <= cell1(to_integer(s_cra));
            if s_cwe10 = '1' and s_wbank = '1' then cell1(to_integer(s_cwa10)) <= s_cwd10; end if;
        end if;
    end process;

    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            -- dry chain (input at d1)
            s_dy(2) <= s_in_y; s_du(2) <= s_in_u; s_dv(2) <= s_in_v;
            for i in 3 to 5 loop s_dy(i) <= s_dy(i - 1); s_du(i) <= s_du(i - 1); s_dv(i) <= s_dv(i - 1); end loop;
            -- 5: cell RAM out (read issued at 3, out at 4) -> fabric
            if s_rbank_blk = '1' then
                s_my5 <= unsigned(s_crd1(31 downto 22)); s_mu5 <= unsigned(s_crd1(21 downto 14)) & "00"; s_mv5 <= unsigned(s_crd1(13 downto 6)) & "00";
            else
                s_my5 <= unsigned(s_crd0(31 downto 22)); s_mu5 <= unsigned(s_crd0(21 downto 14)) & "00"; s_mv5 <= unsigned(s_crd0(13 downto 6)) & "00";
            end if;
            -- 6: grid borders
            if s_sw_grid = '1' and s_edged(5) = '1' then s_ymix6 <= to_unsigned(64, 10); else s_ymix6 <= s_my5; end if;
            s_umix6 <= s_mu5; s_vmix6 <= s_mv5;
            s_dyd6 <= s_dy(5); s_dud6 <= s_du(5); s_dvd6 <= s_dv(5);
            -- 7: differences toward the mosaic
            s_d7y <= signed(resize(s_ymix6, 11)) - signed(resize(s_dyd6, 11));
            s_d7u <= signed(resize(s_umix6, 11)) - signed(resize(s_dud6, 11));
            s_d7v <= signed(resize(s_vmix6, 11)) - signed(resize(s_dvd6, 11));
            s_b7y <= s_dyd6; s_b7u <= s_dud6; s_b7v <= s_dvd6;
            -- 8
            s_p8y <= s_d7y * signed(resize(s_mix, 9));
            s_p8u <= s_d7u * signed(resize(s_mix, 9));
            s_p8v <= s_d7v * signed(resize(s_mix, 9));
            s_b8y <= s_b7y; s_b8u <= s_b7u; s_b8v <= s_b7v;
            -- 9
            s_y9 <= f_clamp10(signed(resize(s_b8y, 13)) + resize(shift_right(s_p8y, 8), 13));
            s_u9 <= f_clamp10(signed(resize(s_b8u, 13)) + resize(shift_right(s_p8u, 8), 13));
            s_v9 <= f_clamp10(signed(resize(s_b8v, 13)) + resize(shift_right(s_p8v, 8), 13));
            -- 17: iso lines (solar_b output lands at 16)
            if s_sw_iso = '1' and s_regb = '1' and (s_ysolb(6 downto 0) < 6) then
                s_y17 <= to_unsigned(64, 10);
            else
                s_y17 <= s_yb;
            end if;
            s_u17 <= s_ub; s_v17 <= s_vb;
            -- 18: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    -- output-side solar: 9 -> 16
    solar_b : entity work.solar_fold
        port map (clk => clk, y_in => s_y9, u_in => s_u9, v_in => s_v9,
                  thr => s_thr_row, gain => s_gain, base => s_base, fold_c => s_sw_chroma,
                  enable => s_en_b, y_out => s_yb, u_out => s_ub, v_out => s_vb,
                  ysol => s_ysolb, region => s_regb);

    data_out.y       <= std_logic_vector(s_y18);
    data_out.u       <= std_logic_vector(s_u18);
    data_out.v       <= std_logic_vector(s_v18);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture solarcell;
