-- Lenscreen: catalogue combo #106 (seed 20260925) -- composite luma/chroma
-- crosstalk + halftone + lens (sphere / tunnel).  Two knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Bulge    lens strength, bipolar (pinch .. bulge)
--   K2 Radius   lens radius in lines
--   K3 Cell     halftone cell 4 / 8 / 16 / 32 px
--   K4 Dot      dot shape: circle / diamond / square / line
--   K5 Cross-Y  chroma leaking into luma (dot crawl) strength
--   K6 Cross-C  fine luma detail leaking into chroma strength
--
--   S7  Lens    sphere / tunnel
--   S8  Screen  dots on the screen grid / dots warped with the lens
--   S9  Ink     black dots on white / dots take the picture colour
--   S10 Cross-Y rattles the dots (before the screen) / flickers the print
--   S11 Fringe  cross-colour phase alternates per line (rainbow comb)
--   P12 Zoom    overall scale 50 .. 200%
--
-- Lens DDA fetch from a dual-bank line buffer; the halftone screen is a
-- cell-local distance test against the luma; crosstalk is two small
-- products on the fetched pixel.  Latency 17 clocks, all modes,
-- blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture lenscreen of program_top is

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

    type t_tab is array (0 to 63) of integer range 0 to 2047;
    constant C_SPH : t_tab := (256, 256, 256, 256, 257, 257, 257, 258, 258, 259, 259, 260, 261, 261, 262, 263, 264, 266, 267, 268, 269, 271, 273, 274, 276, 278, 280, 282, 285, 287, 290, 293, 296, 299, 302, 306, 310, 314, 318, 323, 328, 333, 339, 346, 353, 360, 368, 377, 387, 398, 410, 424, 439, 457, 477, 501, 529, 563, 606, 661, 736, 846, 1032, 1454);
    constant C_TUN : t_tab := (256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 244, 243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 219, 217, 214, 212, 209, 206, 203, 200, 197, 193, 190, 186, 182, 178, 174, 169, 165, 160, 155, 149, 143, 137, 131, 124, 116, 108, 99, 89, 77, 63, 45);

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_bulge, s_k_rad, s_k_cell, s_k_dot, s_k_cy, s_k_cc : unsigned(9 downto 0);
    signal s_sw_tunnel, s_sw_lensed, s_sw_colour, s_sw_after, s_sw_fringe : std_logic;
    signal s_p_zoom : unsigned(9 downto 0);
    signal s_sw_mirror : std_logic := '1';
    signal s_half   : unsigned(5 downto 0) := (others => '0');
    signal s_shc, s_shd, s_shs, s_sh : unsigned(3 downto 0) := (others => '0');
    signal s_dot    : unsigned(1 downto 0) := (others => '0');
    signal s_kcy, s_kcc : unsigned(7 downto 0) := (others => '0');
    type t_src is array (5 to 8) of signed(11 downto 0);
    signal s_srcd : t_src := (others => (others => '0'));
    signal s_ywp1, s_ywp2 : unsigned(9 downto 0) := (others => '0');
    signal s_cx9, s_cy9 : signed(6 downto 0) := (others => '0');
    signal s_uc9, s_ydf9 : signed(10 downto 0) := (others => '0');
    signal s_sx10, s_sy10 : unsigned(11 downto 0) := (others => '0');
    signal s_ax10, s_ay10 : unsigned(5 downto 0) := (others => '0');
    signal s_pcy10, s_pcc10 : signed(19 downto 0) := (others => '0');
    signal s_d11 : unsigned(12 downto 0) := (others => '0');
    signal s_dly11, s_dlc11 : signed(11 downto 0) := (others => '0');
    signal s_dly12 : signed(11 downto 0) := (others => '0');
    signal s_ycx12 : unsigned(9 downto 0) := (others => '0');
    signal s_yl13 : unsigned(9 downto 0) := (others => '0');
    signal s_yg14 : unsigned(12 downto 0) := (others => '0');
    type t_d is array (12 to 14) of unsigned(12 downto 0);
    signal s_dd : t_d := (others => (others => '0'));
    type t_dl is array (13 to 15) of signed(11 downto 0);
    signal s_dld : t_dl := (others => (others => '0'));
    signal s_ink15 : std_logic := '0';
    signal s_yo16 : signed(12 downto 0) := (others => '0');
    signal s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');

    signal s_cmask  : unsigned(5 downto 0) := (others => '0');
    signal s_rshift : unsigned(2 downto 0) := (others => '0');
    signal s_kb     : signed(8 downto 0) := (others => '0');
    signal s_zoom   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_rad    : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_vstep  : unsigned(5 downto 0) := (others => '1');
    signal s_dq     : unsigned(16 downto 0) := (others => '0');
    signal s_dr     : unsigned(11 downto 0) := (others => '0');
    signal s_dbit   : unsigned(4 downto 0) := (others => '0');
    signal s_inv_r  : unsigned(11 downto 0) := (others => '0');
    signal s_cx, s_cy : unsigned(10 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';
    -- per line
    signal s_lstep  : unsigned(3 downto 0) := (others => '1');
    signal s_ady    : unsigned(10 downto 0) := (others => '0');
    signal s_beyond : std_logic := '0';
    signal s_tp     : unsigned(22 downto 0) := (others => '0');
    signal s_t6     : unsigned(5 downto 0) := (others => '0');
    signal s_f      : unsigned(10 downto 0) := (others => '0');
    signal s_fm1    : signed(11 downto 0) := (others => '0');
    signal s_fk     : signed(20 downto 0) := (others => '0');
    signal s_srel   : signed(11 downto 0) := (others => '0');
    signal s_sz     : signed(21 downto 0) := (others => '0');
    signal s_srow   : signed(11 downto 0) := to_signed(256, 12);
    signal s_cxs    : signed(23 downto 0) := (others => '0');
    signal s_src0   : signed(20 downto 0) := (others => '0');
    signal s_w2m1   : signed(12 downto 0) := (others => '0');
    signal s_blk0   : std_logic := '0';
    signal s_wbank, s_rbank_blk : std_logic := '0';

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_we : std_logic := '0';

    type t_x is array (1 to 10) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 6) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_acc  : signed(20 downto 0) := (others => '0');
    signal s_src4 : signed(11 downto 0) := (others => '0');
    signal s_src5 : signed(12 downto 0) := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg   : std_logic := '0';
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    type t_c10 is array (9 to 15) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_c10 := (others => (others => '0'));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_bulge <= unsigned(registers_in(0));
    s_k_rad   <= unsigned(registers_in(1));
    s_k_cell  <= unsigned(registers_in(2));
    s_k_dot   <= unsigned(registers_in(3));
    s_k_cy    <= unsigned(registers_in(4));
    s_k_cc    <= unsigned(registers_in(5));
    s_sw_tunnel <= registers_in(6)(0);
    s_sw_lensed <= registers_in(6)(1);
    s_sw_colour <= registers_in(6)(2);
    s_sw_after  <= registers_in(6)(3);
    s_sw_fringe <= registers_in(6)(4);
    s_p_zoom <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_r : unsigned(12 downto 0);
        variable v_s : signed(11 downto 0);
        variable v_bl : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_rshift <= "000";

            case to_integer(s_k_cell(9 downto 8)) is
                when 0 => s_cmask <= "000011"; s_half <= "000010"; s_shc <= "0111"; s_shd <= "1000"; s_shs <= "1001";
                when 1 => s_cmask <= "000111"; s_half <= "000100"; s_shc <= "0101"; s_shd <= "0111"; s_shs <= "1000";
                when 2 => s_cmask <= "001111"; s_half <= "001000"; s_shc <= "0011"; s_shd <= "0110"; s_shs <= "0111";
                when others => s_cmask <= "011111"; s_half <= "010000"; s_shc <= "0001"; s_shd <= "0101"; s_shs <= "0110";
            end case;
            s_dot <= s_k_dot(9 downto 8);
            case to_integer(s_dot) is
                when 0 => s_sh <= s_shc;
                when 1 => s_sh <= s_shd;
                when others => s_sh <= s_shs;
            end case;
            s_kcy <= s_k_cy(9 downto 2); s_kcc <= s_k_cc(9 downto 2);
            s_kb <= resize(shift_right(signed(resize(s_k_bulge, 11)) - 512, 1), 9);
            s_zoom <= to_unsigned(128, 9) + resize(s_p_zoom(9 downto 2), 9) + resize(s_p_zoom(9 downto 3), 9);
            s_rad <= to_unsigned(16, 11) + resize(s_k_rad(9 downto 1), 11) + resize(s_k_rad(9 downto 2), 11);

            case to_integer(s_vstep) is
                when 0 =>
                    s_dq <= (others => '0'); s_dr <= (others => '0'); s_dbit <= to_unsigned(16, 5);
                    s_cx <= '0' & s_line_width(10 downto 1);
                    s_cy <= '0' & s_height(10 downto 1);
                    s_vstep <= s_vstep + 1;
                when 1 to 17 =>
                    if s_dbit = 16 then v_r := (s_dr & '1'); else v_r := (s_dr & '0'); end if;
                    if v_r >= resize(s_rad, 13) then
                        s_dr <= resize(v_r - resize(s_rad, 13), 12);
                        s_dq <= s_dq(15 downto 0) & '1';
                    else
                        s_dr <= v_r(11 downto 0);
                        s_dq <= s_dq(15 downto 0) & '0';
                    end if;
                    s_dbit <= s_dbit - 1;
                    s_vstep <= s_vstep + 1;
                when 18 =>
                    if s_dq(16 downto 12) /= 0 then s_inv_r <= (others => '1'); else s_inv_r <= s_dq(11 downto 0); end if;
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            case to_integer(s_lstep) is
                when 0 =>
                    if s_line >= s_cy then s_ady <= s_line - s_cy; else s_ady <= s_cy - s_line; end if;
                    s_w2m1 <= signed(resize(s_line_width & '0', 13)) - 1;
                    -- row block: first line of the block writes; banks alternate per block
                    v_bl := shift_right(s_line, to_integer(s_rshift));
                    if shift_left(v_bl, to_integer(s_rshift)) = s_line then s_blk0 <= '1'; else s_blk0 <= '0'; end if;
                    s_wbank <= v_bl(0); s_rbank_blk <= not v_bl(0);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_tp <= s_ady * s_inv_r;
                    if s_ady >= s_rad then s_beyond <= '1'; else s_beyond <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    if s_beyond = '1' then s_t6 <= (others => '0'); else s_t6 <= s_tp(15 downto 10); end if;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    if s_sw_tunnel = '1' then s_f <= to_unsigned(C_TUN(to_integer(s_t6)), 11); else s_f <= to_unsigned(C_SPH(to_integer(s_t6)), 11); end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_beyond = '1' then s_fm1 <= (others => '0'); else s_fm1 <= signed(resize(s_f, 12)) - 256; end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_fk <= s_fm1 * s_kb;
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    v_s := to_signed(256, 12) + resize(shift_right(s_fk, 7), 12);
                    if v_s < 32 then v_s := to_signed(32, 12); elsif v_s > 2047 then v_s := to_signed(2047, 12); end if;
                    s_srel <= v_s;
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_sz <= s_srel * signed(resize(s_zoom, 10));
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    v_s := resize(shift_right(s_sz, 8), 12);
                    if v_s < 16 then v_s := to_signed(16, 12); end if;
                    s_srow <= v_s;
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    s_cxs <= signed(resize(s_cx, 12)) * s_srow;
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    s_src0 <= resize(shift_left(resize(signed(resize(s_cx, 12)), 21), 8) - resize(s_cxs, 21) - resize(s_srow, 21), 21);
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then s_lstep <= (others => '0'); end if;
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

    p_addr : process(clk)
        variable v_neg, v_ref : signed(12 downto 0);
        variable v_c : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 10 loop s_x(i) <= s_x(i - 1); end loop;
            for i in 2 to 6 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- 3: DDA
            if s_avd(1) = '0' then s_acc <= s_src0; else s_acc <= s_acc + resize(s_srow, 21); end if;
            -- 4: source x
            s_src4 <= s_acc(19 downto 8);
            -- 5
            s_src5 <= resize(s_src4, 13);
            s_srcd(5) <= s_src4;
            for i in 6 to 8 loop s_srcd(i) <= s_srcd(i - 1); end loop;
            -- 6: edges
            v_c := s_src5;
            v_neg := -v_c;
            v_ref := s_w2m1 - v_c;
            if s_avd(4) = '0' then
                s_bg <= '1'; s_addr <= (others => '0');
            elsif v_c < 0 then
                if s_sw_mirror = '1' and v_neg < signed(resize(s_line_width, 13)) then s_bg <= '0'; s_addr <= unsigned(v_neg(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            elsif v_c >= signed(resize(s_line_width, 13)) then
                if s_sw_mirror = '1' and v_ref >= 0 then s_bg <= '0'; s_addr <= unsigned(v_ref(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            else
                s_bg <= '0'; s_addr <= unsigned(v_c(10 downto 0));
            end if;
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
    begin
        if rising_edge(clk) then
            -- 7 (rd) / 8: fetch
            s_a_bg <= s_bg; s_a_rbank <= s_rbank_blk;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 9: cell-local coordinates, centred chroma, luma high-pass
            s_ywp1 <= s_wet_y; s_ywp2 <= s_ywp1;
            if s_sw_lensed = '1' then
                s_cx9 <= signed(resize(unsigned(s_srcd(8)(5 downto 0)) and s_cmask, 7)) - signed(resize(s_half, 7));
            else
                s_cx9 <= signed(resize(s_x(7)(5 downto 0) and s_cmask, 7)) - signed(resize(s_half, 7));
            end if;
            s_cy9 <= signed(resize(s_line(5 downto 0) and s_cmask, 7)) - signed(resize(s_half, 7));
            s_uc9 <= signed(resize(s_wet_u, 11)) - 512;
            s_ydf9 <= signed(resize(s_wet_y, 11)) - signed(resize(s_ywp2, 11));
            s_yd(9) <= s_wet_y; s_ud(9) <= s_wet_u; s_vd(9) <= s_wet_v;
            for i in 10 to 15 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            -- 10: squares, abs, crosstalk products
            s_sx10 <= unsigned(resize(s_cx9 * s_cx9, 12)); s_sy10 <= unsigned(resize(s_cy9 * s_cy9, 12));
            if s_cx9 < 0 then s_ax10 <= unsigned(resize(-s_cx9, 6)); else s_ax10 <= unsigned(resize(s_cx9, 6)); end if;
            if s_cy9 < 0 then s_ay10 <= unsigned(resize(-s_cy9, 6)); else s_ay10 <= unsigned(resize(s_cy9, 6)); end if;
            s_pcy10 <= s_uc9 * signed(resize(s_kcy, 9));
            s_pcc10 <= s_ydf9 * signed(resize(s_kcc, 9));
            -- 11: distance measure, crosstalk terms (cross-luma alternates per pixel)
            case to_integer(s_dot) is
                when 0 => s_d11 <= resize(s_sx10, 13) + resize(s_sy10, 13);
                when 1 => s_d11 <= resize(s_ax10, 13) + resize(s_ay10, 13);
                when 2 => if s_ax10 > s_ay10 then s_d11 <= resize(s_ax10, 13); else s_d11 <= resize(s_ay10, 13); end if;
                when others => s_d11 <= resize(s_ay10, 13);
            end case;
            if s_x(9)(0) = '1' then s_dly11 <= -resize(shift_right(s_pcy10, 7), 12); else s_dly11 <= resize(shift_right(s_pcy10, 7), 12); end if;
            s_dlc11 <= resize(shift_right(s_pcc10, 7), 12);
            -- 12: crosstalk applied (cross-luma before the screen when asked)
            if s_sw_after = '0' then s_ycx12 <= f_clamp10(signed(resize(s_yd(11), 13)) + resize(s_dly11, 13)); else s_ycx12 <= s_yd(11); end if;
            s_ud(12) <= f_clamp10(signed(resize(s_ud(11), 13)) + resize(s_dlc11, 13));
            if s_sw_fringe = '1' and s_line(0) = '1' then
                s_vd(12) <= f_clamp10(signed(resize(s_vd(11), 13)) - resize(s_dlc11, 13));
            else
                s_vd(12) <= f_clamp10(signed(resize(s_vd(11), 13)) + resize(s_dlc11, 13));
            end if;
            s_dly12 <= s_dly11;
            s_dd(12) <= s_d11;
            for i in 13 to 14 loop s_dd(i) <= s_dd(i - 1); end loop;
            s_dld(13) <= s_dly12;
            for i in 14 to 15 loop s_dld(i) <= s_dld(i - 1); end loop;
            -- 13: black-relative luma
            if s_ycx12 < 64 then s_yl13 <= (others => '0'); else s_yl13 <= s_ycx12 - 64; end if;
            -- 14: into distance units
            s_yg14 <= shift_right(resize(s_yl13, 13), to_integer(s_sh));
            -- 15: ink decision (dark luma -> big dots)
            if s_yg14 < s_dd(14) then s_ink15 <= '1'; else s_ink15 <= '0'; end if;
            -- 16: compose
            if s_ink15 = '1' then s_yo16 <= to_signed(0, 13); else s_yo16 <= to_signed(876, 13); end if;
            if s_sw_after = '1' then
                if s_ink15 = '1' then s_yo16 <= resize(s_dld(15), 13); else s_yo16 <= to_signed(876, 13) + resize(s_dld(15), 13); end if;
            end if;
            if s_sw_colour = '1' and s_ink15 = '1' then s_u16 <= s_ud(15); s_v16 <= s_vd(15);
            else s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10); end if;
            -- 17: clamp + gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y17 <= f_clamp10(s_yo16); s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y17);
    data_out.u       <= std_logic_vector(s_u17);
    data_out.v       <= std_logic_vector(s_v17);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture lenscreen;
