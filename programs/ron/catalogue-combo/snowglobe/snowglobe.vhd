-- Snowglobe: catalogue combo #68 (seed 20260922) -- pixelation + RF noise
-- / snow / VHS tracking + polar warp (sphere / tunnel).  Two knobs each;
-- the switches and the slider decide how they combine.
--
--   K1 Cell     cell width 2 / 4 / 8 / 16 / 32 / 64 px
--   K2 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 lines
--   K3 Snow     snow amount
--   K4 Tracking fraction of lines that lose tracking (torn and dark)
--   K5 Bulge    lens strength, bipolar (pinch .. bulge)
--   K6 Radius   lens radius in lines
--
--   S7  Cells   cells in lens (source) space / in screen space
--   S8  Snow    per pixel / held per cell
--   S9  Lens    sphere / tunnel
--   S10 Edges   outside the line: black / mirrored
--   S11 Snow    mono / coloured
--   P12 Zoom    overall scale 50 .. 200%
--
-- The line buffer is written only on the first line of each row block and
-- read from the other block-parity bank; the lens DDA address is truncated
-- to cells.  Latency 20 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture snowglobe of program_top is

    constant C_LAT : integer := 20;

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

    signal s_k_cell, s_k_rows, s_k_snow, s_k_track, s_k_bulge, s_k_rad : unsigned(9 downto 0);
    signal s_sw_screen, s_sw_cellsnow, s_sw_tunnel, s_sw_mirror, s_sw_colsnow : std_logic;
    signal s_p_zoom : unsigned(9 downto 0);

    signal s_cmask  : unsigned(5 downto 0) := (others => '0');
    signal s_rshift : unsigned(2 downto 0) := (others => '0');
    signal s_snow, s_track : unsigned(7 downto 0) := (others => '0');
    signal s_kb     : signed(8 downto 0) := (others => '0');
    signal s_zoom   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_rad    : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_vstep  : unsigned(5 downto 0) := (others => '1');
    signal s_dq     : unsigned(16 downto 0) := (others => '0');
    signal s_dr     : unsigned(11 downto 0) := (others => '0');
    signal s_dbit   : unsigned(4 downto 0) := (others => '0');
    signal s_inv_r  : unsigned(11 downto 0) := (others => '0');
    signal s_cx, s_cy : unsigned(10 downto 0) := (others => '0');
    signal s_lfsr   : unsigned(15 downto 0) := x"ACE1";
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
    signal s_lost_row : std_logic := '0';
    signal s_tear_row : signed(10 downto 0) := (others => '0');
    signal s_lnz_row : unsigned(7 downto 0) := (others => '0');
    signal s_blk0   : std_logic := '0';
    signal s_wbank, s_rbank_blk : std_logic := '0';

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_we : std_logic := '0';

    type t_x is array (1 to 7) of unsigned(10 downto 0);
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
    signal s_nzh  : unsigned(15 downto 0) := (others => '0');
    signal s_nz9  : unsigned(15 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_pn10, s_pu10, s_pv10 : unsigned(15 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : signed(11 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    type t_c10 is array (13 to 19) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_c10 := (others => (others => '0'));
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_cell  <= unsigned(registers_in(0));
    s_k_rows  <= unsigned(registers_in(1));
    s_k_snow  <= unsigned(registers_in(2));
    s_k_track <= unsigned(registers_in(3));
    s_k_bulge <= unsigned(registers_in(4));
    s_k_rad   <= unsigned(registers_in(5));
    s_sw_screen   <= registers_in(6)(0);
    s_sw_cellsnow <= registers_in(6)(1);
    s_sw_tunnel   <= registers_in(6)(2);
    s_sw_mirror   <= registers_in(6)(3);
    s_sw_colsnow  <= registers_in(6)(4);
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
            s_we <= data_in.avid and s_blk0;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

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
            s_snow <= s_k_snow(9 downto 2); s_track <= s_k_track(9 downto 2);
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
                    if s_lfsr(7 downto 0) < s_track then s_lost_row <= '1'; else s_lost_row <= '0'; end if;
                    s_tear_row <= signed(resize(s_lfsr(9 downto 0), 11)) - 512;
                    s_lnz_row <= s_lfsr(15 downto 8);
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
                    if s_lost_row = '1' then s_src0 <= resize(shift_left(resize(signed(resize(s_cx, 12)) + resize(s_tear_row, 12), 21), 8) - resize(s_cxs, 21) - resize(s_srow, 21), 21); end if;
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
            for i in 2 to 7 loop s_x(i) <= s_x(i - 1); end loop;
            for i in 2 to 6 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- 3: DDA
            if s_avd(1) = '0' then s_acc <= s_src0; else s_acc <= s_acc + resize(s_srow, 21); end if;
            -- 4: source x, cells in source space
            if s_sw_screen = '1' then
                s_src4 <= s_acc(19 downto 8);
            else
                s_src4 <= s_acc(19 downto 8) and not signed(resize(s_cmask, 12));
            end if;
            -- 5: cells in screen space = hold the address at cell boundaries
            if s_sw_screen = '1' then
                if (s_x(3)(5 downto 0) and s_cmask) = 0 then s_src5 <= resize(s_src4, 13); end if;
            else
                s_src5 <= resize(s_src4, 13);
            end if;
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
            -- 9: noise (held per cell if asked)
            if s_sw_cellsnow = '0' or (s_x(7)(5 downto 0) and s_cmask) = 0 then s_nzh <= s_lfsr; end if;
            s_nz9 <= s_nzh;
            s_y9 <= s_wet_y; s_u9 <= s_wet_u; s_v9 <= s_wet_v;
            -- 10
            s_pn10 <= s_snow * s_nz9(7 downto 0);
            s_pu10 <= s_snow * s_nz9(15 downto 8);
            s_pv10 <= s_snow * s_nz9(11 downto 4);
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            -- 11
            s_y11 <= signed(resize(s_y10, 12)) + signed(resize(s_pn10(15 downto 7), 12)) - signed(resize(s_snow, 12));
            if s_sw_colsnow = '1' then
                s_u11 <= signed(resize(s_u10, 12)) + signed(resize(s_pu10(15 downto 7), 12)) - signed(resize(s_snow, 12));
                s_v11 <= signed(resize(s_v10, 12)) + signed(resize(s_pv10(15 downto 7), 12)) - signed(resize(s_snow, 12));
            else
                s_u11 <= signed(resize(s_u10, 12)); s_v11 <= signed(resize(s_v10, 12));
            end if;
            -- 12
            if s_lost_row = '1' then
                s_y12 <= f_clamp10(shift_right(s_y11, 1) + signed(resize(s_lnz_row, 12)));
                s_u12 <= f_clamp10(shift_right(s_u11 - 512, 2) + 512);
                s_v12 <= f_clamp10(shift_right(s_v11 - 512, 2) + 512);
            else
                s_y12 <= f_clamp10(s_y11); s_u12 <= f_clamp10(s_u11); s_v12 <= f_clamp10(s_v11);
            end if;
            s_yd(13) <= s_y12; s_ud(13) <= s_u12; s_vd(13) <= s_v12;
            for i in 14 to 19 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            -- 20: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y20 <= s_yd(19); s_u20 <= s_ud(19); s_v20 <= s_vd(19);
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture snowglobe;
