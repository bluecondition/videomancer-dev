-- Cellcam: catalogue combo #86 (seed 20260923) -- edge detect + chroma
-- subsampling artifacts + pixelation.  Two knobs each; the switches and the
-- slider decide how they combine.
--
--   K1 Cell     cell width 2 / 4 / 8 / 16 / 32 / 64 px
--   K2 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 lines
--   K3 Gain     edge gain
--   K4 Width    edge line width 1 / 2 / 4 / 8 px
--   K5 Hold     chroma sample-and-hold 1 / 2 / 4 / 8 / 16 px
--   K6 Lag      chroma delayed 0..15 px behind luma
--
--   S7  Edges   drawn over the cells / cells replaced by edges on black
--   S8  Source  edges between cells / edges of the raw picture (gradient)
--   S9  Chroma  chroma held per cell / held by the Hold knob only
--   S10 Ink     edges black / edges white
--   S11 Chroma  edges keep chroma / edges mono
--   P12 Crawl   NTSC dot crawl on the chroma edges (0 = clean)
--
-- Block-held line buffer (written on the first line of each row block,
-- banks by block parity) gives the pixelated picture; cell-to-cell luma
-- steps make the edges.  Latency 16 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cellcam of program_top is

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

    signal s_k_cell, s_k_rows, s_k_gain, s_k_width, s_k_hold, s_k_lag : unsigned(9 downto 0);
    signal s_sw_onblack, s_sw_rawedge, s_sw_cellchroma, s_sw_white, s_sw_mono : std_logic;
    signal s_p_crawl : unsigned(9 downto 0);

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
    s_k_gain  <= unsigned(registers_in(2));
    s_k_width <= unsigned(registers_in(3));
    s_k_hold  <= unsigned(registers_in(4));
    s_k_lag   <= unsigned(registers_in(5));
    s_sw_onblack    <= registers_in(6)(0);
    s_sw_rawedge    <= registers_in(6)(1);
    s_sw_cellchroma <= registers_in(6)(2);
    s_sw_white      <= registers_in(6)(3);
    s_sw_mono       <= registers_in(6)(4);
    s_p_crawl <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_bl : unsigned(10 downto 0);
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
            s_gain <= s_k_gain(9 downto 2);
            case to_integer(s_k_width(9 downto 8)) is
                when 0 => s_wmask <= "000"; when 1 => s_wmask <= "001"; when 2 => s_wmask <= "011"; when others => s_wmask <= "111";
            end case;
            case to_integer(s_k_hold(9 downto 7)) is
                when 0 | 1 => s_hmask <= "0000";
                when 2 | 3 => s_hmask <= "0001";
                when 4 | 5 => s_hmask <= "0011";
                when 6     => s_hmask <= "0111";
                when others => s_hmask <= "1111";
            end case;
            s_lag <= s_k_lag(9 downto 6);
            s_crawl <= s_p_crawl(9 downto 2);

            if s_prev_avid = '1' and data_in.avid = '0' then s_line_width <= s_rx; s_line <= s_line + 1; end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                v_bl := shift_right(s_line, to_integer(s_rshift));
                if shift_left(v_bl, to_integer(s_rshift)) = s_line then s_blk0 <= '1'; else s_blk0 <= '0'; end if;
                s_wbank <= v_bl(0); s_rbank_blk <= not v_bl(0);
                s_lphase <= s_lphase + 2;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then s_line <= (others => '0'); end if;

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
            -- 2: cell-held address (first column of the cell), raw gradient
            s_ah <= s_x(1) and not resize(s_cmask, 11);
            s_ym2 <= s_in_y;
            if s_in_y >= s_ym2 then s_gx2 <= s_in_y - s_ym2; else s_gx2 <= s_ym2 - s_in_y; end if;
            -- 3
            if s_avd(2) = '0' then s_bg <= '1'; else s_bg <= '0'; end if;
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
            -- 6: cell step = |cell - previous cell| latched at cell starts; near-edge flag
            v_b := s_x(4)(5 downto 0) and s_cmask;
            if v_b = 0 then
                s_prev_cell <= s_wet_y;
                if s_wet_y >= s_prev_cell then s_cellstep <= s_wet_y - s_prev_cell; else s_cellstep <= s_prev_cell - s_wet_y; end if;
            end if;
            if (v_b and not resize(s_wmask, 6)) = 0 then s_near7 <= '1'; else s_near7 <= '0'; end if;
            s_y7 <= s_wet_y;
            -- chroma hold (cell or Hold knob)
            if s_sw_cellchroma = '1' then v_hb := v_b(3 downto 0); else v_hb := s_x(4)(3 downto 0) and s_hmask; end if;
            if v_hb = 0 then s_uh <= s_wet_u; s_vh <= s_wet_v; end if;
            if v_hb = 0 then s_u7 <= s_wet_u; s_v7 <= s_wet_v; else s_u7 <= s_uh; s_v7 <= s_vh; end if;
            -- 7: edge value source
            if s_sw_rawedge = '1' then s_e7 <= s_gd(6); else s_e7 <= s_cellstep; end if;
            -- 8
            s_eg8 <= s_e7 * s_gain;
            s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7; s_near8 <= s_near7;
            -- 9: edge magnitude (only near cell starts for cell edges)
            if s_eg8(17 downto 6) > 876 then v_e := to_unsigned(876, 10); else v_e := s_eg8(15 downto 6); end if;
            if s_sw_rawedge = '0' and s_near8 = '0' then v_e := (others => '0'); end if;
            s_ev9 <= v_e;
            s_y9 <= s_y8; s_u9 <= s_u8; s_v9 <= s_v8;
            -- 10: compose luma
            if s_sw_onblack = '1' then
                if s_sw_white = '1' then s_y10 <= s_ev9 + 64; else s_y10 <= 940 - s_ev9; end if;
            else
                if s_sw_white = '1' then s_y10 <= f_clamp10(signed(resize(s_y9, 12)) + signed(resize(s_ev9, 12)));
                else s_y10 <= f_clamp10(signed(resize(s_y9, 12)) - signed(resize(s_ev9, 12))); end if;
            end if;
            if s_sw_mono = '1' and s_ev9 > 64 then s_u10 <= to_unsigned(512, 10); s_v10 <= to_unsigned(512, 10);
            else s_u10 <= s_u9; s_v10 <= s_v9; end if;
            s_ev10 <= s_ev9;
            -- 11: crawl amplitude from the edge magnitude
            s_cp11 <= s_ev10 * s_crawl;
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            s_pi11 <= s_x(10)(1 downto 0) + s_lphase;
            -- 12: dot crawl on chroma at the edges
            s_y12 <= s_y11;
            case to_integer(s_pi11) is
                when 0 => s_u12 <= signed(resize(s_u11, 12)) + signed(resize(s_cp11(17 downto 9), 12)); s_v12 <= signed(resize(s_v11, 12));
                when 2 => s_u12 <= signed(resize(s_u11, 12)) - signed(resize(s_cp11(17 downto 9), 12)); s_v12 <= signed(resize(s_v11, 12));
                when 1 => s_u12 <= signed(resize(s_u11, 12)); s_v12 <= signed(resize(s_v11, 12)) + signed(resize(s_cp11(17 downto 9), 12));
                when others => s_u12 <= signed(resize(s_u11, 12)); s_v12 <= signed(resize(s_v11, 12)) - signed(resize(s_cp11(17 downto 9), 12));
            end case;
            -- 13: lag line
            s_ulag(0) <= f_clamp10(s_u12); s_vlag(0) <= f_clamp10(s_v12);
            for i in 1 to 15 loop s_ulag(i) <= s_ulag(i - 1); s_vlag(i) <= s_vlag(i - 1); end loop;
            s_y13 <= s_y12;
            -- 14: tap
            s_u14 <= s_ulag(to_integer(s_lag)); s_v14 <= s_vlag(to_integer(s_lag));
            s_y14 <= s_y13;
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

end architecture cellcam;
