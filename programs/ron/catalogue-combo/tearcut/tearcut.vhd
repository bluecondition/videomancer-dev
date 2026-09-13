-- Tearcut: catalogue combo #115 (seed 20260926) -- datamosh / block
-- displacement + scanline shear / horizontal tearing + edge detect.  Two
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Gain     edge gain
--   K2 Thresh   edges below this are dropped
--   K3 Block    block size 8 .. 64 px
--   K4 Density  fraction of blocks moshed
--   K5 Tear     tear amplitude (+-255 px)
--   K6 Bands    tear band height 2 / 4 / 8 / 16 lines (Random) or the
--               wave period (Wave)
--
--   S7  Edges   white / tinted with the picture's colour
--   S8  Tear    random per band / triangle wave down the picture
--   S9  Blocks  plain / moshed blocks go negative
--   S10 Mosh    shift / hold (block repeats its first column)
--   S11 Ink     light edges on dark / dark edges on light
--   P12 Blend   edge picture (0) .. the picture with edges over it (100)
--
-- Edge = |gx| + |gy| from the input and the Y line buffer, written as an
-- 8-bit edge line and read back one line later at the block-displaced,
-- torn address; the picture itself (dry ring) is never torn, so the edges
-- slide over it.  Latency 18 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture tearcut of program_top is

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

    function f_round1(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 5);
        return t xor shift_right(t, 6);
    end function;
    function f_round2(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 3);
        return t xor shift_right(t, 8);
    end function;
    function f_round3(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 7);
        t := t xor shift_right(t, 5);
        return t xor shift_left(t, 8);
    end function;

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_e is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_ring is array (0 to 31) of std_logic_vector(31 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbE0, lbE1 : t_lb_e := (others => x"00");
    signal ring : t_ring := (others => x"04080200");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdE0, s_rdE1 : std_logic_vector(7 downto 0) := x"00";
    signal s_ring_q : std_logic_vector(31 downto 0) := x"04080200";
    signal s_ring_w : unsigned(4 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_gain, s_k_thr, s_k_block, s_k_dens, s_k_tear, s_k_bands : unsigned(9 downto 0);
    signal s_sw_tint, s_sw_wave, s_sw_neg, s_sw_hold, s_sw_dark : std_logic;
    signal s_p_blend : unsigned(9 downto 0);
    signal s_sw_band : std_logic := '0';
    signal s_k_rate : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_tear : unsigned(7 downto 0) := (others => '0');
    signal s_bsh : unsigned(2 downto 0) := (others => '0');
    signal s_lstep : unsigned(1 downto 0) := "11";
    signal s_tkey : unsigned(15 downto 0) := (others => '0');
    signal s_th1 : unsigned(15 downto 0) := (others => '0');
    signal s_tri : unsigned(8 downto 0) := (others => '0');
    signal s_tp : signed(17 downto 0) := (others => '0');
    signal s_tear_row : signed(9 downto 0) := (others => '0');
    signal s_tstep : unsigned(2 downto 0) := "111";
    signal s_gain, s_thr, s_blend, s_shift, s_dens : unsigned(7 downto 0) := (others => '0');
    signal s_bs     : unsigned(2 downto 0) := to_unsigned(4, 3);       -- block shift 3..6
    signal s_bmask  : unsigned(10 downto 0) := (others => '0');
    signal s_roll   : unsigned(15 downto 0) := (others => '0');
    signal s_rate_cnt : unsigned(7 downto 0) := (others => '0');
    signal s_vstep  : unsigned(1 downto 0) := "11";

    -- tracking
    signal s_rx, s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_rowkey : unsigned(15 downto 0) := (others => '0');

    -- edge side (stages 1..8)
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_y2     : unsigned(9 downto 0) := (others => '0');
    signal s_gx2    : signed(10 downto 0) := (others => '0');
    signal s_prev2  : unsigned(9 downto 0) := (others => '0');
    signal s_r_rbank : std_logic := '0';
    signal s_gx3, s_gy3 : signed(10 downto 0) := (others => '0');
    signal s_ax4, s_ay4 : unsigned(9 downto 0) := (others => '0');
    signal s_sum5   : unsigned(10 downto 0) := (others => '0');
    signal s_pe6    : unsigned(17 downto 0) := (others => '0');
    signal s_e7     : signed(12 downto 0) := (others => '0');
    signal s_e8     : unsigned(7 downto 0) := (others => '0');
    type t_wx is array (1 to 7) of unsigned(10 downto 0);
    signal s_wx : t_wx := (others => (others => '0'));
    signal s_wp, s_wc : std_logic_vector(1 to 7) := (others => '0');
    signal s_wx8 : unsigned(10 downto 0) := (others => '0');
    signal s_wp8, s_we8 : std_logic := '0';

    -- address / hash chain (stages 1..8)
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_key2   : unsigned(15 downto 0) := (others => '0');
    signal s_h3, s_h4, s_h5 : unsigned(15 downto 0) := (others => '0');
    signal s_mosh6  : std_logic := '0';
    signal s_po6    : signed(17 downto 0) := (others => '0');
    signal s_u7     : signed(12 downto 0) := (others => '0');
    signal s_addr8  : unsigned(10 downto 0) := (others => '0');
    signal s_bg8    : std_logic := '0';
    type t_xd is array (2 to 6) of unsigned(10 downto 0);
    signal s_xd     : t_xd := (others => (others => '0'));
    signal s_moshd  : std_logic_vector(7 to 15) := (others => '0');

    -- display side (stages 9..18)
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_ed10   : unsigned(7 downto 0) := (others => '0');
    signal s_dy10, s_du10, s_dv10 : unsigned(9 downto 0) := (others => '0');
    signal s_ye11   : unsigned(9 downto 0) := (others => '0');
    signal s_cu11, s_cv11 : signed(10 downto 0) := (others => '0');
    signal s_ed11   : unsigned(7 downto 0) := (others => '0');
    signal s_dy11, s_du11, s_dv11 : unsigned(9 downto 0) := (others => '0');
    signal s_ye12   : unsigned(9 downto 0) := (others => '0');
    signal s_pu12, s_pv12 : signed(19 downto 0) := (others => '0');
    signal s_dy12, s_du12, s_dv12 : unsigned(9 downto 0) := (others => '0');
    signal s_ye13, s_ue13, s_ve13 : unsigned(9 downto 0) := (others => '0');
    signal s_dy13, s_du13, s_dv13 : unsigned(9 downto 0) := (others => '0');
    signal s_by14, s_bu14, s_bv14 : signed(10 downto 0) := (others => '0');
    signal s_ye14, s_ue14, s_ve14 : unsigned(9 downto 0) := (others => '0');
    signal s_py15, s_pu15, s_pv15 : signed(19 downto 0) := (others => '0');
    signal s_ye15, s_ue15, s_ve15 : unsigned(9 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_gain  <= unsigned(registers_in(0));
    s_k_thr   <= unsigned(registers_in(1));
    s_k_block <= unsigned(registers_in(2));
    s_k_dens  <= unsigned(registers_in(3));
    s_k_tear  <= unsigned(registers_in(4));
    s_k_bands <= unsigned(registers_in(5));
    s_sw_tint <= registers_in(6)(0);
    s_sw_wave <= registers_in(6)(1);
    s_sw_neg  <= registers_in(6)(2);
    s_sw_hold <= registers_in(6)(3);
    s_sw_dark <= registers_in(6)(4);
    s_p_blend <= unsigned(registers_in(7));

    -- dry ring: data_in delayed 10 (q at stage 10)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            s_ring_q <= ring(to_integer(s_ring_w - 9));
            ring(to_integer(s_ring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control, tracking, edge side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;

            -- stage 2: horizontal gradient, previous-line pixel
            s_y2  <= s_in_y;
            s_gx2 <= signed('0' & s_in_y) - signed('0' & s_y2);
            s_r_rbank <= s_rbank;
            if s_r_rbank = '1' then s_prev2 <= unsigned(s_rdY1); else s_prev2 <= unsigned(s_rdY0); end if;
            -- stage 3: vertical gradient
            s_gy3 <= signed('0' & s_y2) - signed('0' & s_prev2);
            s_gx3 <= s_gx2;
            -- stage 4: abs
            if s_gx3 < 0 then s_ax4 <= unsigned(resize(-s_gx3, 10)); else s_ax4 <= unsigned(resize(s_gx3, 10)); end if;
            if s_gy3 < 0 then s_ay4 <= unsigned(resize(-s_gy3, 10)); else s_ay4 <= unsigned(resize(s_gy3, 10)); end if;
            -- stage 5: sum
            s_sum5 <= resize(s_ax4, 11) + resize(s_ay4, 11);
            -- stage 6: gain
            s_pe6 <= s_sum5(10 downto 1) * s_gain;
            -- stage 7: threshold
            s_e7 <= signed(resize(s_pe6(17 downto 5), 13)) - signed(resize(s_thr, 13));
            -- stage 8: 8-bit edge
            if s_e7 < 0 then s_e8 <= (others => '0');
            elsif s_e7 > 255 then s_e8 <= x"FF";
            else s_e8 <= unsigned(s_e7(7 downto 0)); end if;
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar; s_wc(1) <= data_in.avid;
            for i in 2 to 7 loop s_wx(i) <= s_wx(i - 1); s_wp(i) <= s_wp(i - 1); s_wc(i) <= s_wc(i - 1); end loop;
            s_wx8 <= s_wx(7); s_wp8 <= s_wp(7); s_we8 <= s_wc(7);

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_gain  <= s_k_gain(9 downto 2);
                    s_thr   <= s_k_thr(9 downto 2);
                    s_blend <= s_p_blend(9 downto 2);
                    s_bs    <= resize(s_k_block(9 downto 8), 3) + 3;
                    s_shift <= to_unsigned(200, 8);
                    s_dens  <= s_k_dens(9 downto 2);
                    s_tear  <= s_k_tear(9 downto 2);
                    s_bsh   <= resize(s_k_bands(9 downto 8), 3) + 1;
                    if s_rate_cnt >= resize(not s_k_rate(9 downto 3), 8) then
                        s_rate_cnt <= (others => '0');
                        s_roll <= s_roll + 1013;
                    else
                        s_rate_cnt <= s_rate_cnt + 1;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    for i in 0 to 10 loop
                        if i < to_integer(s_bs) then s_bmask(i) <= '1'; else s_bmask(i) <= '0'; end if;
                    end loop;
                    s_vstep <= "11";
                when others => null;
            end case;

            -- line / frame events
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
            end if;
            case to_integer(s_tstep) is
                when 0 =>
                    s_tkey <= x"2C9F" + s_roll + shift_left(resize(shift_right(s_line, to_integer(s_bsh)), 16), 5);
                    -- triangle wave: period 2^(bsh+5) lines
                    s_tri <= resize(shift_left(s_line, 4 - to_integer(s_bsh(1 downto 0)))(8 downto 0), 9);
                    s_tstep <= s_tstep + 1;
                when 1 =>
                    s_th1 <= f_round3(f_round1(s_tkey));
                    if s_tri(8) = '1' then s_tri <= 511 - s_tri; end if;
                    s_tstep <= s_tstep + 1;
                when 2 =>
                    if s_sw_wave = '1' then
                        s_tp <= (signed(resize(s_tri, 9)) - 128) * signed(resize(s_tear, 9));
                    else
                        s_tp <= (signed(resize(s_th1(15 downto 8), 9)) - 128) * signed(resize(s_tear, 9));
                    end if;
                    s_tstep <= s_tstep + 1;
                when 3 =>
                    s_tear_row <= resize(shift_right(s_tp, 6), 10);
                    s_tstep <= "111";
                when others => null;
            end case;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
                s_rowkey <= x"6B1D" + s_roll + shift_left(resize(shift_right(s_line, to_integer(s_bs)), 16), 7);
                s_tstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
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
    -- Block hash + display address (stages 1..8)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_x : unsigned(10 downto 0);
        variable v_o : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx;
            -- stage 2: key = row key + block index (or row key only per band)
            if s_sw_band = '1' then
                s_key2 <= s_rowkey;
            else
                s_key2 <= s_rowkey + resize(shift_right(s_x1, to_integer(s_bs)), 16);
            end if;
            s_xd(2) <= s_x1;
            for i in 3 to 6 loop s_xd(i) <= s_xd(i - 1); end loop;
            -- stages 3..5: hash
            s_h3 <= f_round1(s_key2);
            s_h4 <= f_round2(s_h3);
            s_h5 <= f_round3(s_h4);
            -- stage 6: mosh decision, offset product
            if s_h5(15 downto 8) < s_dens then s_mosh6 <= '1'; else s_mosh6 <= '0'; end if;
            s_po6 <= (signed('0' & s_h5(7 downto 0)) - to_signed(128, 9)) * signed('0' & s_shift);
            -- stage 7: displaced x (hold = block start)
            if s_mosh6 = '1' then
                v_o := resize(shift_right(s_po6, 5), 13);
                if v_o > 511 then v_o := to_signed(511, 13); elsif v_o < -511 then v_o := to_signed(-511, 13); end if;
                if s_sw_hold = '1' then v_x := s_xd(6) and not s_bmask; else v_x := s_xd(6); end if;
                s_u7 <= signed(resize(v_x, 13)) + v_o + resize(s_tear_row, 13);
            else
                s_u7 <= signed(resize(s_xd(6), 13)) + resize(s_tear_row, 13);
            end if;
            s_moshd(7) <= s_mosh6;
            for i in 8 to 15 loop s_moshd(i) <= s_moshd(i - 1); end loop;
            -- stage 8: range
            if s_u7 < 0 or s_u7 >= signed(resize(s_line_width, 13)) then s_bg8 <= '1'; else s_bg8 <= '0'; end if;
            s_addr8 <= unsigned(s_u7(10 downto 0));
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers: Y (written stage 1, read at rx -> data 1),
    -- E (written stage 8, read at stage 8 -> data 9)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_rx));
            if s_in_avid = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_rx));
            if s_in_avid = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbE0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdE0 <= lbE0(to_integer(s_addr8));
            if s_we8 = '1' and s_wp8 = '0' then lbE0(to_integer(s_wx8)) <= std_logic_vector(s_e8); end if;
        end if;
    end process;
    p_lbE1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdE1 <= lbE1(to_integer(s_addr8));
            if s_we8 = '1' and s_wp8 = '1' then lbE1(to_integer(s_wx8)) <= std_logic_vector(s_e8); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Display pipeline (stages 9..18)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 9 ctx / 10 fetch + dry
            s_a_bg <= s_bg8; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then s_ed10 <= (others => '0');
            elsif s_a_rbank = '1' then s_ed10 <= unsigned(s_rdE1);
            else s_ed10 <= unsigned(s_rdE0); end if;
            s_dy10 <= unsigned(s_ring_q(29 downto 20)); s_du10 <= unsigned(s_ring_q(19 downto 10)); s_dv10 <= unsigned(s_ring_q(9 downto 0));
            -- stage 11: edge luma, centred dry chroma
            if s_sw_dark = '1' then
                v_y := to_signed(940, 13) - signed(resize(s_ed10 & "00", 13));
            else
                v_y := to_signed(64, 13) + signed(resize(s_ed10 & "00", 13));
            end if;
            if v_y > 940 then v_y := to_signed(940, 13); elsif v_y < 64 then v_y := to_signed(64, 13); end if;
            s_ye11 <= f_clamp10(v_y);
            s_cu11 <= signed('0' & s_du10) - to_signed(512, 11);
            s_cv11 <= signed('0' & s_dv10) - to_signed(512, 11);
            s_ed11 <= s_ed10; s_dy11 <= s_dy10; s_du11 <= s_du10; s_dv11 <= s_dv10;
            -- stage 12: tint products
            s_pu12 <= s_cu11 * signed('0' & s_ed11);
            s_pv12 <= s_cv11 * signed('0' & s_ed11);
            s_ye12 <= s_ye11; s_dy12 <= s_dy11; s_du12 <= s_du11; s_dv12 <= s_dv11;
            -- stage 13: edge chroma
            if s_sw_tint = '1' then
                s_ue13 <= f_clamp10(to_signed(512, 13) + resize(shift_right(s_pu12, 7), 13));
                s_ve13 <= f_clamp10(to_signed(512, 13) + resize(shift_right(s_pv12, 7), 13));
            else
                s_ue13 <= to_unsigned(512, 10); s_ve13 <= to_unsigned(512, 10);
            end if;
            s_ye13 <= s_ye12; s_dy13 <= s_dy12; s_du13 <= s_du12; s_dv13 <= s_dv12;
            -- stage 14: blend differences (dry - edge)
            s_by14 <= signed('0' & s_dy13) - signed('0' & s_ye13);
            s_bu14 <= signed('0' & s_du13) - signed('0' & s_ue13);
            s_bv14 <= signed('0' & s_dv13) - signed('0' & s_ve13);
            s_ye14 <= s_ye13; s_ue14 <= s_ue13; s_ve14 <= s_ve13;
            -- stage 15: blend products
            s_py15 <= s_by14 * signed('0' & s_blend);
            s_pu15 <= s_bu14 * signed('0' & s_blend);
            s_pv15 <= s_bv14 * signed('0' & s_blend);
            s_ye15 <= s_ye14; s_ue15 <= s_ue14; s_ve15 <= s_ve14;
            -- stage 16: apply; moshed blocks negative
            if s_sw_neg = '1' and s_moshd(15) = '1' then
                s_y16 <= not f_clamp10(signed(resize(s_ye15, 13)) + resize(shift_right(s_py15, 8), 13));
            else
                s_y16 <= f_clamp10(signed(resize(s_ye15, 13)) + resize(shift_right(s_py15, 8), 13));
            end if;
            s_u16 <= f_clamp10(signed(resize(s_ue15, 13)) + resize(shift_right(s_pu15, 8), 13));
            s_v16 <= f_clamp10(signed(resize(s_ve15, 13)) + resize(shift_right(s_pv15, 8), 13));
            -- stage 17: gate
            if s_avid_sr(15) = '1' then
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            -- stage 18: output
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y18);
    data_out.u       <= std_logic_vector(s_u18);
    data_out.v       <= std_logic_vector(s_v18);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture tearcut;
