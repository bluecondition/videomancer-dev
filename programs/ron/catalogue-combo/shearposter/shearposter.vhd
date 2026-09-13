-- Shearposter: catalogue combo #110 (seed 20260925) -- pixelation +
-- scanline shear / horizontal tearing + posterization.  Two knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Cell     cell width 2 / 4 / 8 / 16 / 32 / 64 px
--   K2 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 lines
--   K3 Shear    per-line shear amplitude (+-255 px)
--   K4 Rate     shear wave rate (Sine) / tear probability (Tear)
--   K5 Luma     posterize luma 1..8 bits
--   K6 Chroma   posterize chroma 1..8 bits
--
--   S7  Shear   sine wave / random tear per line
--   S8  Shear   shifts whole cells (cell-aligned) / shifts freely
--   S9  Cells   cells shear with the rows / cells stay put, content shears
--   S10 Edges   outside the line: black / mirrored
--   S11 Chroma  chroma posterized too / clean
--   P12 Pitch   posterize dither cell 1 / 2 / 4 / 8 px (2x2 dither scaled)
--
-- Block-held line buffer (first line of each row block, banks by block
-- parity) read at x + shear, cell-held address, then a dithered quantizer.
-- Latency 16 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture shearposter of program_top is

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

    signal s_k_cell, s_k_rows, s_k_shear, s_k_rate, s_k_lbits, s_k_cbits : unsigned(9 downto 0);
    signal s_sw_tear, s_sw_cellshift, s_sw_cellshear, s_sw_mirror, s_sw_cpost : std_logic;
    signal s_p_pitch : unsigned(9 downto 0);
    signal s_k_gain, s_k_width, s_k_hold, s_k_lag, s_p_crawl : unsigned(9 downto 0) := (others => '0');
    signal s_sw_onblack, s_sw_rawedge, s_sw_cellchroma, s_sw_white, s_sw_mono : std_logic := '0';
    signal s_amt, s_rate : unsigned(7 downto 0) := (others => '0');
    signal s_kl, s_kc : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_lnot, s_lhalf, s_lq, s_cnot, s_chalf, s_cq : unsigned(9 downto 0) := (others => '0');
    signal s_pitch : unsigned(1 downto 0) := (others => '0');
    signal s_lstep : unsigned(2 downto 0) := "111";
    signal s_tph, s_fph : unsigned(7 downto 0) := (others => '0');
    signal s_tri, s_tearv : signed(7 downto 0) := (others => '0');
    signal s_shp : signed(15 downto 0) := (others => '0');
    signal s_off_row : signed(10 downto 0) := (others => '0');
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal s_saw_active : std_logic := '0';
    signal s_a3 : signed(12 downto 0) := (others => '0');
    signal s_w2m1 : signed(12 downto 0) := (others => '0');
    signal s_dz : unsigned(1 downto 0) := (others => '0');

    signal s_cmask : unsigned(5 downto 0) := (others => '0');
    signal s_rshift : unsigned(2 downto 0) := (others => '0');
    signal s_gain, s_crawl : unsigned(7 downto 0) := (others => '0');
    signal s_wmask : unsigned(2 downto 0) := (others => '0');
    signal s_hmask : unsigned(3 downto 0) := (others => '0');
    signal s_lag : unsigned(3 downto 0) := (others => '0');
    signal s_blk0, s_wbank, s_rbank_blk : std_logic := '0';
    signal s_lphase : unsigned(1 downto 0) := (others => '0');

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_we : std_logic := '0';

    type t_x is array (1 to 10) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 4) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_ah : unsigned(10 downto 0) := (others => '0');
    signal s_bg : std_logic := '0';
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    -- raw gradient path
    signal s_ym2, s_gx2 : unsigned(9 downto 0) := (others => '0');
    type t_g is array (2 to 8) of unsigned(9 downto 0);
    signal s_gd : t_g := (others => (others => '0'));
    -- cell edge
    signal s_prev_cell : unsigned(9 downto 0) := (others => '0');
    signal s_cellstep : unsigned(9 downto 0) := (others => '0');
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    signal s_e7 : unsigned(9 downto 0) := (others => '0');
    signal s_near7 : std_logic := '0';
    signal s_uh, s_vh : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_eg8 : unsigned(17 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_near8 : std_logic := '0';
    signal s_ev9 : unsigned(9 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_y10 : unsigned(9 downto 0) := (others => '0');
    signal s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_ev10 : unsigned(9 downto 0) := (others => '0');
    signal s_cp11 : unsigned(17 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');
    signal s_pi11 : unsigned(1 downto 0) := (others => '0');
    signal s_y12 : unsigned(9 downto 0) := (others => '0');
    signal s_u12, s_v12 : signed(11 downto 0) := (others => '0');
    type t_lagc is array (0 to 15) of unsigned(9 downto 0);
    signal s_ulag, s_vlag : t_lagc := (others => to_unsigned(512, 10));
    signal s_y13, s_y14, s_y15 : unsigned(9 downto 0) := (others => '0');
    signal s_u14, s_v14, s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_cell  <= unsigned(registers_in(0));
    s_k_rows  <= unsigned(registers_in(1));
    s_k_shear <= unsigned(registers_in(2));
    s_k_rate  <= unsigned(registers_in(3));
    s_k_lbits <= unsigned(registers_in(4));
    s_k_cbits <= unsigned(registers_in(5));
    s_sw_tear      <= registers_in(6)(0);
    s_sw_cellshift <= registers_in(6)(1);
    s_sw_cellshear <= registers_in(6)(2);
    s_sw_mirror    <= registers_in(6)(3);
    s_sw_cpost     <= registers_in(6)(4);
    s_p_pitch <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_bl : unsigned(10 downto 0);
        variable v_m : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid and s_blk0;

            case to_integer(s_k_cell(9 downto 7)) is
                when 0 => s_cmask <= "000001";
                when 1 | 2 => s_cmask <= "000011";
                when 3 => s_cmask <= "000111";
                when 4 | 5 => s_cmask <= "001111";
                when 6 => s_cmask <= "011111";
                when others => s_cmask <= "111111";
            end case;
            case to_integer(s_k_rows(9 downto 7)) is
                when 0 => s_rshift <= "000";
                when 1 | 2 => s_rshift <= "001";
                when 3 => s_rshift <= "010";
                when 4 | 5 => s_rshift <= "011";
                when 6 => s_rshift <= "100";
                when others => s_rshift <= "101";
            end case;
            s_gain <= (others => '0'); s_wmask <= (others => '0'); s_hmask <= (others => '0'); s_lag <= (others => '0'); s_crawl <= (others => '0');
            s_amt <= s_k_shear(9 downto 2); s_rate <= s_k_rate(9 downto 2);
            s_kl <= to_unsigned(9, 4) - resize(s_k_lbits(9 downto 7), 4);
            s_kc <= to_unsigned(9, 4) - resize(s_k_cbits(9 downto 7), 4);
            v_m := shift_left(to_unsigned(1, 10), to_integer(s_kl)) - 1;
            s_lnot <= not v_m; s_lhalf <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 1); s_lq <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 2);
            v_m := shift_left(to_unsigned(1, 10), to_integer(s_kc)) - 1;
            s_cnot <= not v_m; s_chalf <= shift_left(to_unsigned(1, 10), to_integer(s_kc) - 1); s_cq <= shift_left(to_unsigned(1, 10), to_integer(s_kc) - 2);
            s_pitch <= s_p_pitch(9 downto 8);
            s_w2m1 <= signed(resize(s_line_width & '0', 13)) - 1;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            -- per-line shear
            case to_integer(s_lstep) is
                when 0 =>
                    if s_tph(7) = '1' then s_tri <= signed(resize(not s_tph(6 downto 0), 8)) - 64; else s_tri <= signed(resize(s_tph(6 downto 0), 8)) - 64; end if;
                    if s_lfsr(15 downto 8) < s_rate then s_tearv <= signed(resize(s_lfsr(6 downto 0), 8)) - 64; else s_tearv <= (others => '0'); end if;
                    s_tph <= s_tph + resize(s_rate(7 downto 3), 8);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    if s_sw_tear = '1' then s_shp <= s_tearv * signed(resize(s_amt, 8)); else s_shp <= s_tri * signed(resize(s_amt, 8)); end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_off_row <= resize(shift_right(s_shp, 6), 11);
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then s_line_width <= s_rx; s_line <= s_line + 1; end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_lstep <= (others => '0');
                v_bl := shift_right(s_line, to_integer(s_rshift));
                if shift_left(v_bl, to_integer(s_rshift)) = s_line then s_blk0 <= '1'; else s_blk0 <= '0'; end if;
                s_wbank <= v_bl(0); s_rbank_blk <= not v_bl(0);
                s_lphase <= s_lphase + 2;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                if s_saw_active = '1' then s_fph <= s_fph + 3; end if;
                s_tph <= s_fph; s_saw_active <= '0';
            end if;
            if data_in.avid = '1' then s_saw_active <= '1'; end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 10 loop s_x(i) <= s_x(i - 1); end loop;
            for i in 2 to 4 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- 2: sheared position (cell-aligned when asked), cells shear with the rows or stay put
            if s_sw_cellshift = '1' then
                s_a3 <= signed(resize(s_x(1), 13)) + resize(signed(s_off_row(10 downto 0) and not resize(signed(resize(s_cmask, 11)), 11)), 13);
            else
                s_a3 <= signed(resize(s_x(1), 13)) + resize(s_off_row, 13);
            end if;
            -- 3: cell hold on the sheared (or screen) position, edges
            if s_avd(2) = '0' or s_a3 < 0 or s_a3 >= signed(resize(s_line_width, 13)) then
                if s_sw_mirror = '1' and s_avd(2) = '1' and s_a3 < 0 and (-s_a3) < signed(resize(s_line_width, 13)) then
                    s_bg <= '0'; s_ah <= unsigned(resize(-s_a3, 11)) and not resize(s_cmask, 11);
                elsif s_sw_mirror = '1' and s_avd(2) = '1' and s_a3 >= signed(resize(s_line_width, 13)) and (s_w2m1 - s_a3) >= 0 then
                    s_bg <= '0'; s_ah <= unsigned(resize(s_w2m1 - s_a3, 11)) and not resize(s_cmask, 11);
                else
                    s_bg <= '1'; s_ah <= (others => '0');
                end if;
            else
                s_bg <= '0';
                if s_sw_cellshear = '1' then s_ah <= unsigned(s_a3(10 downto 0)) and not resize(s_cmask, 11);
                else s_ah <= (s_x(2) and not resize(s_cmask, 11)) + unsigned(s_off_row(10 downto 0)); end if;
            end if;
            s_addr <= s_ah;
            s_gd(2) <= s_gx2;
            for i in 3 to 8 loop s_gd(i) <= s_gd(i - 1); end loop;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we = '1' and s_wbank = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we = '1' and s_wbank = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wbank = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wbank = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wbank = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wbank = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_b : unsigned(5 downto 0);
        variable v_e : unsigned(9 downto 0);
        variable v_ink : std_logic;
        variable v_hb : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 4 (rd) / 5: fetch
            s_a_bg <= s_bg; s_a_rbank <= s_rbank_blk;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 6: dither value (2x2 scaled by pitch), posterize
            case to_integer(s_pitch) is
                when 0 => s_dz <= s_x(5)(0) & s_line(0);
                when 1 => s_dz <= s_x(5)(1) & s_line(1);
                when 2 => s_dz <= s_x(5)(2) & s_line(2);
                when others => s_dz <= s_x(5)(3) & s_line(3);
            end case;
            s_y7 <= s_wet_y; s_u7 <= s_wet_u; s_v7 <= s_wet_v;
            -- 7
            case to_integer(s_dz) is
                when 0 => s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
                when 1 => s_y8 <= f_clamp10(signed(resize(s_y7, 12)) + signed(resize(s_lhalf, 12))); s_u8 <= f_clamp10(signed(resize(s_u7, 12)) + signed(resize(s_chalf, 12))); s_v8 <= f_clamp10(signed(resize(s_v7, 12)) + signed(resize(s_chalf, 12)));
                when 2 => s_y8 <= f_clamp10(signed(resize(s_y7, 12)) + signed(resize(s_lhalf, 12)) + signed(resize(s_lq, 12))); s_u8 <= f_clamp10(signed(resize(s_u7, 12)) + signed(resize(s_chalf, 12)) + signed(resize(s_cq, 12))); s_v8 <= f_clamp10(signed(resize(s_v7, 12)) + signed(resize(s_chalf, 12)) + signed(resize(s_cq, 12)));
                when others => s_y8 <= f_clamp10(signed(resize(s_y7, 12)) + signed(resize(s_lq, 12))); s_u8 <= f_clamp10(signed(resize(s_u7, 12)) + signed(resize(s_cq, 12))); s_v8 <= f_clamp10(signed(resize(s_v7, 12)) + signed(resize(s_cq, 12)));
            end case;
            -- 8: quantize
            s_y9 <= (s_y8 and s_lnot) + s_lhalf;
            if s_sw_cpost = '1' then s_u9 <= (s_u8 and s_cnot) + s_chalf; s_v9 <= (s_v8 and s_cnot) + s_chalf; else s_u9 <= s_u8; s_v9 <= s_v8; end if;
            -- 9..15 copies
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            s_y13 <= s_y11; s_y14 <= s_y13; s_u14 <= s_u11; s_v14 <= s_v11;
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            -- 16: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
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

end architecture shearposter;
