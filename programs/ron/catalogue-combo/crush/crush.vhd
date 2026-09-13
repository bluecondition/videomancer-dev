-- Crush: catalogue combo #42 (seed 20260919) -- sync loss / horizontal
-- collapse + mosaic with per-cell colour averaging.  Three knobs each; the
-- switches and the slider decide how the two combine.
--
--   K1 Line     where the picture collapses to (0 .. 100 % of the width)
--   K2 Sync     sync loss: hashed per-line offsets
--   K3 Roll     the collapse line drifts sideways on its own; speed
--   K4 Cells    cell width 2 .. 64 px
--   K5 Aspect   cell height = width / 1, 2, 4, 8
--   K6 Grout    dark grid lines between cells
--
--   S7  Order   cells then collapse (cells squeeze) / collapse then cells (cells stay square)
--   S8  Collapse toward the line from both sides / everything to the left
--   S9  Loss    per-line / 8-line bands
--   S10 Beyond  black outside the picture / edge cells stretched
--   S11 Grout   black / white
--   P12 Collapse the picture squeezes to the line (x1 .. x17)
--
-- Box-averaged cells written to the line buffer; the read address is the
-- collapsed x, either cell-held before collapsing (cells squeeze with the
-- picture) or after (screen-fixed cells).  Latency 14 clocks.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture crush of program_top is

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

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_bring is array (0 to 63) of std_logic_vector(31 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal bring : t_bring := (others => x"04080200");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_bring_q : std_logic_vector(31 downto 0) := x"04080200";
    signal s_bring_w : unsigned(5 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_line, s_k_sync, s_k_roll, s_k_cells, s_k_aspect, s_k_grout : unsigned(9 downto 0);
    signal s_sw_cellsafter, s_sw_left, s_sw_bands, s_sw_stretch, s_sw_white : std_logic;
    signal s_p_collapse : unsigned(9 downto 0);
    signal s_cline  : unsigned(10 downto 0) := to_unsigned(360, 11);
    signal s_clp    : unsigned(20 downto 0) := (others => '0');
    signal s_cdrift : unsigned(14 downto 0) := (others => '0');
    signal s_cgain  : unsigned(8 downto 0) := to_unsigned(16, 9);
    signal s_sync   : unsigned(7 downto 0) := (others => '0');
    signal s_nsh    : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_mask, s_half : unsigned(10 downto 0) := (others => '0');
    signal s_ndel   : unsigned(5 downto 0) := to_unsigned(7, 6);
    signal s_rows   : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_grout  : unsigned(7 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";
    signal s_lfsr   : unsigned(15 downto 0) := x"6E27";

    -- tracking / per-line
    signal s_rx, s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_vcnt   : unsigned(6 downto 0) := (others => '0');
    signal s_capture, s_wpar, s_rbank, s_rowtop : std_logic := '0';
    signal s_lstep  : unsigned(1 downto 0) := "11";
    signal s_hl     : unsigned(9 downto 0) := (others => '0');
    signal s_jp     : signed(19 downto 0) := (others => '0');
    signal s_off_row : signed(10 downto 0) := (others => '0');

    -- input side
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_accy, s_accu, s_accv : unsigned(15 downto 0) := (others => '0');
    signal s_ay3, s_au3, s_av3 : unsigned(9 downto 0) := (others => '0');
    type t_wx is array (1 to 2) of unsigned(10 downto 0);
    signal s_wx : t_wx := (others => (others => '0'));
    signal s_wp, s_wc : std_logic_vector(1 to 2) := (others => '0');
    signal s_wx3 : unsigned(10 downto 0) := (others => '0');
    signal s_wp3, s_we3 : std_logic := '0';

    -- address chain
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_xc2    : unsigned(10 downto 0) := (others => '0');       -- cell-held x (before collapse)
    signal s_gr2    : std_logic := '0';
    signal s_dm3    : signed(11 downto 0) := (others => '0');
    signal s_pm4    : signed(21 downto 0) := (others => '0');
    signal s_am5    : signed(13 downto 0) := (others => '0');
    signal s_addr6  : unsigned(10 downto 0) := (others => '0');
    signal s_bg6    : std_logic := '0';
    signal s_gr6    : std_logic := '0';
    signal s_grd    : std_logic_vector(3 to 12) := (others => '0');

    -- pixel path
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_gy9, s_gu9, s_gv9 : signed(10 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_gc9    : unsigned(7 downto 0) := (others => '0');
    signal s_py10, s_pu10, s_pv10 : signed(19 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_line   <= unsigned(registers_in(0));
    s_k_sync   <= unsigned(registers_in(1));
    s_k_roll   <= unsigned(registers_in(2));
    s_k_cells  <= unsigned(registers_in(3));
    s_k_aspect <= unsigned(registers_in(4));
    s_k_grout  <= unsigned(registers_in(5));
    s_sw_cellsafter <= registers_in(6)(0);
    s_sw_left       <= registers_in(6)(1);
    s_sw_bands      <= registers_in(6)(2);
    s_sw_stretch    <= registers_in(6)(3);
    s_sw_white      <= registers_in(6)(4);
    s_p_collapse <= unsigned(registers_in(7));

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_bring_w <= s_bring_w + 1;
            s_bring_q <= bring(to_integer(s_bring_w - s_ndel));
            bring(to_integer(s_bring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control + input side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_oy, v_ou, v_ov : unsigned(9 downto 0);
        variable v_n : unsigned(3 downto 0);
        variable v_o : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            -- stage 2: running sums
            v_oy := unsigned(s_bring_q(29 downto 20)); v_ou := unsigned(s_bring_q(19 downto 10)); v_ov := unsigned(s_bring_q(9 downto 0));
            if data_in.avid = '0' then
                s_accy <= (others => '0'); s_accu <= (others => '0'); s_accv <= (others => '0');
            else
                s_accy <= s_accy + resize(unsigned(data_in.y), 16) - resize(v_oy, 16);
                s_accu <= s_accu + resize(unsigned(data_in.u), 16) - resize(v_ou, 16);
                s_accv <= s_accv + resize(unsigned(data_in.v), 16) - resize(v_ov, 16);
            end if;
            -- stage 3: averages, write port
            s_ay3 <= resize(shift_right(s_accy, to_integer(s_nsh)), 10);
            s_au3 <= resize(shift_right(s_accu, to_integer(s_nsh)), 10);
            s_av3 <= resize(shift_right(s_accv, to_integer(s_nsh)), 10);
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar; s_wc(1) <= data_in.avid and s_capture;
            s_wx(2) <= s_wx(1); s_wp(2) <= s_wp(1); s_wc(2) <= s_wc(1);
            s_wx3 <= s_wx(2); s_wp3 <= s_wp(2); s_we3 <= s_wc(2);

            -- per-line: sync offset
            case to_integer(s_lstep) is
                when 0 =>
                    if s_sw_bands = '0' or s_line(2 downto 0) = 0 then s_hl <= s_lfsr(9 downto 0); end if;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_jp <= (signed('0' & s_hl) - to_signed(512, 11)) * signed('0' & s_sync);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    v_o := resize(shift_right(s_jp, 8), 13);
                    if v_o > 511 then v_o := to_signed(511, 13); elsif v_o < -511 then v_o := to_signed(-511, 13); end if;
                    s_off_row <= resize(v_o, 11);
                    s_lstep <= "11";
                when others => null;
            end case;

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_clp   <= s_k_line * s_line_width;
                    s_cgain <= to_unsigned(16, 9) + resize(s_p_collapse(9 downto 2), 9);
                    s_sync  <= s_k_sync(9 downto 2);
                    s_cdrift <= s_cdrift + resize(s_k_roll(9 downto 4), 15);
                    v_n := to_unsigned(1, 4) + resize(s_k_cells(9 downto 7), 4);
                    if v_n > 6 then v_n := to_unsigned(6, 4); end if;
                    s_nsh   <= v_n(2 downto 0);
                    s_grout <= s_k_grout(9 downto 2);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    if s_cdrift(14 downto 4) >= s_line_width then s_cdrift <= (others => '0'); end if;
                    case to_integer(s_k_aspect(9 downto 8)) is
                        when 0 => s_rows <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)), 7);
                        when 1 => s_rows <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)) srl 1, 7);
                        when 2 => s_rows <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)) srl 2, 7);
                        when others => s_rows <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)) srl 3, 7);
                    end case;
                    s_ndel <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)), 7)(5 downto 0) - 1;
                    for i in 0 to 10 loop
                        if i < to_integer(s_nsh) then s_mask(i) <= '1'; else s_mask(i) <= '0'; end if;
                        if i = to_integer(s_nsh) - 1 then s_half(i) <= '1'; else s_half(i) <= '0'; end if;
                    end loop;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    if s_rows = 0 then s_rows <= to_unsigned(1, 7); end if;
                    if s_sw_left = '1' then s_cline <= (others => '0'); else s_cline <= s_clp(20 downto 10) + s_cdrift(14 downto 4); end if;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    if s_cline >= s_line_width then s_cline <= s_cline - s_line_width; end if;
                    s_vstep <= "111";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                if s_vcnt >= s_rows - 1 then s_vcnt <= (others => '0'); else s_vcnt <= s_vcnt + 1; end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_vcnt = 0 then s_capture <= '1'; s_wpar <= not s_wpar; s_rbank <= s_wpar; s_rowtop <= '1';
                else s_capture <= '0'; s_rbank <= s_wpar; if s_vcnt = 1 then s_rowtop <= '1'; else s_rowtop <= '0'; end if; end if;
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_vcnt <= (others => '0'); s_wpar <= '0';
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
    -- Address chain (stages 1..6): cell hold, collapse, sync, cell hold again
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_a : unsigned(11 downto 0);
        variable v_x : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx;
            -- stage 2: cells before the collapse (screen-fixed cells) or plain x
            v_a := resize(s_x1 and not s_mask, 12) + resize(s_half, 12);
            if v_a >= resize(s_line_width, 12) then v_x := s_line_width - 1; else v_x := v_a(10 downto 0); end if;
            if s_sw_cellsafter = '0' then s_xc2 <= v_x; else s_xc2 <= s_x1; end if;
            if (s_x1 and s_mask) < 2 or s_rowtop = '1' then s_gr2 <= '1'; else s_gr2 <= '0'; end if;
            s_grd(3) <= s_gr2; for i in 4 to 12 loop s_grd(i) <= s_grd(i - 1); end loop;
            -- stage 3: about the collapse line
            s_dm3 <= signed(resize(s_xc2, 12)) - signed(resize(s_cline, 12));
            -- stage 4: product
            s_pm4 <= s_dm3 * signed('0' & s_cgain);
            -- stage 5: collapsed x + sync
            s_am5 <= resize(shift_right(s_pm4, 4), 14) + signed(resize(s_cline, 14)) + resize(s_off_row, 14);
            -- stage 6: range, cells after the collapse (cells squeeze), grout
            if s_am5 < 0 then
                if s_sw_stretch = '1' then s_addr6 <= (others => '0'); s_bg6 <= '0'; else s_addr6 <= (others => '0'); s_bg6 <= '1'; end if;
            elsif s_am5 >= signed(resize(s_line_width, 14)) then
                if s_sw_stretch = '1' then s_addr6 <= s_line_width - 1; s_bg6 <= '0'; else s_addr6 <= (others => '0'); s_bg6 <= '1'; end if;
            else
                if s_sw_cellsafter = '1' then
                    s_addr6 <= (unsigned(s_am5(10 downto 0)) and not s_mask) or s_half;
                else
                    s_addr6 <= unsigned(s_am5(10 downto 0));
                end if;
                s_bg6 <= '0';
            end if;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (averaged data written at stage 3, read stage 6 -> data 7)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr6));
            if s_we3 = '1' and s_wp3 = '0' then lbY0(to_integer(s_wx3)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr6));
            if s_we3 = '1' and s_wp3 = '1' then lbY1(to_integer(s_wx3)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr6(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '0' then lbU0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr6(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '1' then lbU1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr6(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '0' then lbV0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr6(10 downto 1)));
            if s_we3 = '1' and s_wp3 = '1' then lbV1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 7..14): fetch, grout, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            -- stage 7 ctx / 8 fetch
            s_a_bg <= s_bg6; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_y8 <= to_unsigned(64, 10); s_u8 <= to_unsigned(512, 10); s_v8 <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_y8 <= unsigned(s_rdY1); s_u8 <= unsigned(s_rdU1) & "00"; s_v8 <= unsigned(s_rdV1) & "00";
            else
                s_y8 <= unsigned(s_rdY0); s_u8 <= unsigned(s_rdU0) & "00"; s_v8 <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 9: grout differences
            if s_sw_white = '1' then s_gy9 <= to_signed(768, 11) - signed('0' & s_y8); else s_gy9 <= to_signed(64, 11) - signed('0' & s_y8); end if;
            s_gu9 <= to_signed(512, 11) - signed('0' & s_u8);
            s_gv9 <= to_signed(512, 11) - signed('0' & s_v8);
            if s_grd(8) = '1' then s_gc9 <= s_grout; else s_gc9 <= (others => '0'); end if;
            s_y9 <= s_y8; s_u9 <= s_u8; s_v9 <= s_v8;
            -- stage 10: products
            s_py10 <= s_gy9 * signed('0' & s_gc9);
            s_pu10 <= s_gu9 * signed('0' & s_gc9);
            s_pv10 <= s_gv9 * signed('0' & s_gc9);
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            -- stage 11: apply
            s_y11 <= f_clamp10(signed(resize(s_y10, 13)) + resize(shift_right(s_py10, 8), 13));
            s_u11 <= f_clamp10(signed(resize(s_u10, 13)) + resize(shift_right(s_pu10, 8), 13));
            s_v11 <= f_clamp10(signed(resize(s_v10, 13)) + resize(shift_right(s_pv10, 8), 13));
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;
            -- stage 13: gate
            if s_avid_sr(11) = '1' then
                s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            else
                s_y13 <= to_unsigned(64, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            end if;
            -- stage 14: output
            s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y14);
    data_out.u       <= std_logic_vector(s_u14);
    data_out.v       <= std_logic_vector(s_v14);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture crush;
