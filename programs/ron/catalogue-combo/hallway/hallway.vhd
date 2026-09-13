-- Hallway: catalogue combo #84 (seed 20260923) -- split-screen / quad
-- + lens distortion (barrel, chromatic aberration) + mirror / kaleidoscope.
-- Two knobs each; the switches and the slider decide how they combine.
--
--   K1 Panes    1 / 2 / 3 / 4 panes
--   K2 Wipe     pane boundaries slide across the screen
--   K3 Barrel   lens strength, bipolar (pincushion .. barrel)
--   K4 Fringe   per-channel lens spread (chromatic aberration)
--   K5 Segment  mirror segment 1024 / 512 / 256 / 128 / 64 px
--   K6 Slide    slide speed of the mirrored strip, bipolar
--
--   S7  Mirror  odd panes mirrored
--   S8  Lens    lens on the whole screen / one lens per pane
--   S9  Fold    mirror (triangle) / repeat (sawtooth)
--   S10 Edges   outside the line: black / mirrored
--   S11 Fringe  U wide / V wide
--   P12 Depth   nested folds 1..4 (each halves the segment)
--
-- Pane position -> nested power-of-two folds -> per-channel lens DDAs
-- (three read addresses).  Latency 25 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture hallway of program_top is

    constant C_LAT : integer := 25;

    type t_tab is array (0 to 63) of integer range 0 to 2047;
    constant C_SPH : t_tab := (256, 256, 256, 256, 257, 257, 257, 258, 258, 259, 259, 260, 261, 261, 262, 263, 264, 266, 267, 268, 269, 271, 273, 274, 276, 278, 280, 282, 285, 287, 290, 293, 296, 299, 302, 306, 310, 314, 318, 323, 328, 333, 339, 346, 353, 360, 368, 377, 387, 398, 410, 424, 439, 457, 477, 501, 529, 563, 606, 661, 736, 846, 1032, 1454);

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_panes, s_k_wipe, s_k_bulge, s_k_sep, s_k_seg, s_k_slide : unsigned(9 downto 0);
    signal s_sw_mirror, s_sw_perpane, s_sw_repeat, s_sw_medge, s_sw_flip : std_logic;
    signal s_p_depth : unsigned(9 downto 0);

    signal s_n : unsigned(2 downto 0) := "001";
    signal s_kb : signed(8 downto 0) := (others => '0');
    signal s_sep : signed(8 downto 0) := (others => '0');
    signal s_ks : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal s_depth : unsigned(1 downto 0) := "00";
    signal s_slide, s_shrow : signed(15 downto 0) := (others => '0');
    signal s_vstep : unsigned(5 downto 0) := (others => '1');
    signal s_w3p : unsigned(26 downto 0) := (others => '0');
    signal s_pw, s_b1, s_b2, s_b3 : unsigned(10 downto 0) := (others => '0');
    signal s_wipe : unsigned(20 downto 0) := (others => '0');
    signal s_wipex : unsigned(10 downto 0) := (others => '0');
    signal s_dq : unsigned(16 downto 0) := (others => '0');
    signal s_dr : unsigned(11 downto 0) := (others => '0');
    signal s_dbit : unsigned(4 downto 0) := (others => '0');
    signal s_inv_r : unsigned(11 downto 0) := (others => '0');
    signal s_cx, s_cy : unsigned(10 downto 0) := (others => '0');
    signal s_w2m1 : signed(12 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';
    type t_fc is array (1 to 4) of signed(12 downto 0);
    signal s_fmask, s_fhalf, s_flow : t_fc := (others => (others => '0'));
    signal s_halff : signed(12 downto 0) := (others => '0');
    signal s_fen : std_logic_vector(1 to 4) := (others => '0');
    -- per line lens
    signal s_lstep : unsigned(3 downto 0) := (others => '1');
    signal s_ady : unsigned(10 downto 0) := (others => '0');
    signal s_tp : unsigned(22 downto 0) := (others => '0');
    signal s_t6 : unsigned(5 downto 0) := (others => '0');
    signal s_f : unsigned(10 downto 0) := (others => '0');
    signal s_fm1 : signed(11 downto 0) := (others => '0');
    signal s_fk : signed(20 downto 0) := (others => '0');
    signal s_srow, s_srow_u, s_srow_v : signed(11 downto 0) := to_signed(256, 12);

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    type t_x is array (1 to 4) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 20) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_xw2 : unsigned(11 downto 0) := (others => '0');
    signal s_xw3 : unsigned(10 downto 0) := (others => '0');
    signal s_pane4, s_pane5 : unsigned(1 downto 0) := (others => '0');
    signal s_pst4, s_xw4, s_xp4 : unsigned(10 downto 0) := (others => '0');
    signal s_xm5 : signed(12 downto 0) := (others => '0');
    signal s_pc5 : signed(12 downto 0) := (others => '0');     -- pane centre (or screen centre)
    type t_pc is array (5 to 14) of signed(12 downto 0);
    signal s_pcd : t_pc := (others => (others => '0'));
    signal s_a6 : signed(12 downto 0) := (others => '0');
    type t_f is array (0 to 4) of signed(12 downto 0);
    signal s_m, s_t : t_f := (others => (others => '0'));
    signal s_src15 : signed(12 downto 0) := (others => '0');
    -- lens: per-channel scale about the pane centre: src' = pc + (src - pc) * s
    signal s_dx16, s_dx16u, s_dx16v : signed(12 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_dx16 : signal is true;
    attribute keep of s_dx16u : signal is true;
    attribute keep of s_dx16v : signal is true;
    signal s_pc16 : signed(12 downto 0) := (others => '0');
    signal s_pY17h, s_pU17h, s_pV17h : signed(19 downto 0) := (others => '0');
    signal s_pY17l, s_pU17l, s_pV17l : signed(19 downto 0) := (others => '0');
    signal s_pY18, s_pU18, s_pV18 : signed(25 downto 0) := (others => '0');
    signal s_pc17, s_pc18 : signed(12 downto 0) := (others => '0');
    signal s_aY18, s_aU18, s_aV18 : signed(12 downto 0) := (others => '0');
    signal s_addrY, s_addrU, s_addrV : unsigned(10 downto 0) := (others => '0');
    signal s_bgY, s_bgU, s_bgV : std_logic := '0';
    signal s_a_bgY, s_a_bgU, s_a_bgV, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_panes <= unsigned(registers_in(0));
    s_k_wipe  <= unsigned(registers_in(1));
    s_k_bulge <= unsigned(registers_in(2));
    s_k_sep   <= unsigned(registers_in(3));
    s_k_seg   <= unsigned(registers_in(4));
    s_k_slide <= unsigned(registers_in(5));
    s_sw_mirror  <= registers_in(6)(0);
    s_sw_perpane <= registers_in(6)(1);
    s_sw_repeat  <= registers_in(6)(2);
    s_sw_medge   <= registers_in(6)(3);
    s_sw_flip    <= registers_in(6)(4);
    s_p_depth <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_r : unsigned(12 downto 0);
        variable v_s : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            s_n <= resize(s_k_panes(9 downto 8), 3) + 1;
            s_kb <= resize(shift_right(signed(resize(s_k_bulge, 11)) - 512, 1), 9);
            if s_sw_flip = '1' then s_sep <= -signed(resize(s_k_sep(9 downto 3), 9)); else s_sep <= signed(resize(s_k_sep(9 downto 3), 9)); end if;
            case to_integer(s_k_seg(9 downto 7)) is
                when 0 | 1 => s_ks <= to_unsigned(10, 4);
                when 2 | 3 => s_ks <= to_unsigned(9, 4);
                when 4 | 5 => s_ks <= to_unsigned(8, 4);
                when 6     => s_ks <= to_unsigned(7, 4);
                when others => s_ks <= to_unsigned(6, 4);
            end case;
            s_depth <= s_p_depth(9 downto 8);
            for i in 1 to 4 loop
                s_fhalf(i) <= shift_left(to_signed(1, 13), to_integer(s_ks) - i + 1);
                s_fmask(i) <= shift_left(to_signed(1, 13), to_integer(s_ks) - i + 2) - 1;
                s_flow(i)  <= shift_left(to_signed(1, 13), to_integer(s_ks) - i + 1) - 1;
                if i <= to_integer(s_depth) + 1 then s_fen(i) <= '1'; else s_fen(i) <= '0'; end if;
            end loop;
            s_halff <= shift_left(to_signed(1, 13), to_integer(s_ks) - to_integer(s_depth) - 1);
            s_w2m1 <= signed(resize(s_line_width & '0', 13)) - 1;

            case to_integer(s_vstep) is
                when 0 =>
                    s_w3p <= s_line_width * to_unsigned(21845, 16);
                    s_wipe <= s_k_wipe * s_line_width;
                    s_dq <= (others => '0'); s_dr <= (others => '0'); s_dbit <= to_unsigned(16, 5);
                    s_cx <= '0' & s_line_width(10 downto 1); s_cy <= '0' & s_height(10 downto 1);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    case to_integer(s_n) is
                        when 1 => s_pw <= s_line_width;
                        when 2 => s_pw <= '0' & s_line_width(10 downto 1);
                        when 3 => s_pw <= s_w3p(26 downto 16);
                        when others => s_pw <= "00" & s_line_width(10 downto 2);
                    end case;
                    s_wipex <= s_wipe(20 downto 10);
                    s_vstep <= s_vstep + 1;
                when 2 => s_b1 <= s_pw; s_b2 <= s_pw + s_pw; s_vstep <= s_vstep + 1;
                when 3 => s_b3 <= s_b2 + s_pw; s_vstep <= s_vstep + 1;
                when 4 to 20 =>
                    if s_dbit = 16 then v_r := (s_dr & '1'); else v_r := (s_dr & '0'); end if;
                    if v_r >= resize(s_cy + 1, 13) then
                        s_dr <= resize(v_r - resize(s_cy + 1, 13), 12); s_dq <= s_dq(15 downto 0) & '1';
                    else
                        s_dr <= v_r(11 downto 0); s_dq <= s_dq(15 downto 0) & '0';
                    end if;
                    s_dbit <= s_dbit - 1;
                    s_vstep <= s_vstep + 1;
                when 21 =>
                    if s_dq(16 downto 12) /= 0 then s_inv_r <= (others => '1'); else s_inv_r <= s_dq(11 downto 0); end if;
                    s_vstep <= (others => '1');
                when others => null;
            end case;

            case to_integer(s_lstep) is
                when 0 => if s_line >= s_cy then s_ady <= s_line - s_cy; else s_ady <= s_cy - s_line; end if; s_lstep <= s_lstep + 1;
                when 1 => s_tp <= s_ady * s_inv_r; s_lstep <= s_lstep + 1;
                when 2 => if s_tp(22 downto 16) /= 0 then s_t6 <= (others => '1'); else s_t6 <= s_tp(15 downto 10); end if; s_lstep <= s_lstep + 1;
                when 3 => s_f <= to_unsigned(C_SPH(to_integer(s_t6)), 11); s_lstep <= s_lstep + 1;
                when 4 => s_fm1 <= signed(resize(s_f, 12)) - 256; s_lstep <= s_lstep + 1;
                when 5 => s_fk <= s_fm1 * s_kb; s_lstep <= s_lstep + 1;
                when 6 =>
                    v_s := to_signed(256, 12) + resize(shift_right(s_fk, 7), 12);
                    if v_s < 32 then v_s := to_signed(32, 12); elsif v_s > 2047 then v_s := to_signed(2047, 12); end if;
                    s_srow <= v_s;
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_srow_u <= s_srow + resize(s_sep, 12); s_srow_v <= s_srow - resize(s_sep, 12);
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then
                    s_slide <= s_slide + resize(shift_right(signed(resize(s_k_slide, 11)) - 512, 3), 16);
                    s_vstep <= (others => '0');
                end if;
                s_shrow <= s_slide;
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
        variable v_p : unsigned(1 downto 0);
        variable v_st : unsigned(10 downto 0);
        procedure edge(signal src : in signed(12 downto 0); signal bg : out std_logic; signal addr : out unsigned(10 downto 0)) is
            variable n, r : signed(12 downto 0);
        begin
            n := -src; r := s_w2m1 - src;
            if s_avd(20) = '0' then bg <= '1'; addr <= (others => '0');
            elsif src < 0 then
                if s_sw_medge = '1' and n < signed(resize(s_line_width, 13)) then bg <= '0'; addr <= unsigned(n(10 downto 0)); else bg <= '1'; addr <= (others => '0'); end if;
            elsif src >= signed(resize(s_line_width, 13)) then
                if s_sw_medge = '1' and r >= 0 then bg <= '0'; addr <= unsigned(r(10 downto 0)); else bg <= '1'; addr <= (others => '0'); end if;
            else bg <= '0'; addr <= unsigned(src(10 downto 0)); end if;
        end procedure;
    begin
        if rising_edge(clk) then
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 4 loop s_x(i) <= s_x(i - 1); end loop;
            for i in 2 to 20 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- 2, 3: wiped position
            s_xw2 <= resize(s_x(1), 12) + resize(s_wipex, 12);
            if s_xw2 >= resize(s_line_width, 12) then s_xw3 <= resize(s_xw2 - resize(s_line_width, 12), 11); else s_xw3 <= s_xw2(10 downto 0); end if;
            -- 4: pane
            if s_xw3 >= s_b3 and s_n = 4 then v_p := "11"; v_st := s_b3;
            elsif s_xw3 >= s_b2 and s_n >= 3 then v_p := "10"; v_st := s_b2;
            elsif s_xw3 >= s_b1 and s_n >= 2 then v_p := "01"; v_st := s_b1;
            else v_p := "00"; v_st := (others => '0'); end if;
            s_pane4 <= v_p; s_pst4 <= v_st; s_xw4 <= s_xw3; s_xp4 <= s_xw3 - v_st;
            -- 5: mirrored position, pane centre
            if s_sw_mirror = '1' and s_pane4(0) = '1' then
                s_xm5 <= signed(resize(s_pst4, 13)) + signed(resize(s_pw, 13)) - 1 - signed(resize(s_xp4, 13));
            else
                s_xm5 <= signed(resize(s_xw4, 13));
            end if;
            if s_sw_perpane = '1' then s_pc5 <= signed(resize(s_pst4, 13)) + signed(resize(s_pw(10 downto 1), 13));
            else s_pc5 <= signed(resize(s_cx, 13)); end if;
            s_pane5 <= s_pane4;
            s_pcd(5) <= s_pc5;
            for i in 6 to 14 loop s_pcd(i) <= s_pcd(i - 1); end loop;
            -- 6: fold input (relative to the pane centre, plus slide)
            s_a6 <= s_xm5 - s_pc5 + resize(s_shrow(15 downto 4), 13);
            -- folds 7..14
            s_t(0) <= s_a6;
            for i in 1 to 4 loop
                s_m(i) <= (s_t(i - 1) and s_fmask(i));
                if s_fen(i) = '1' then
                    if s_sw_repeat = '1' then s_t(i) <= s_m(i);
                    elsif (s_m(i) and s_fhalf(i)) /= 0 then s_t(i) <= s_m(i) and s_flow(i);
                    else s_t(i) <= s_fhalf(i) - s_m(i); end if;
                else
                    s_t(i) <= s_t(i - 1);
                end if;
            end loop;
            -- 15: folded source about the pane centre
            s_src15 <= s_pcd(14) - s_halff + s_t(4);
            -- 16: distance from the lens centre
            s_dx16 <= s_src15 - s_pcd(14); s_dx16u <= s_src15 - s_pcd(14); s_dx16v <= s_src15 - s_pcd(14);
            s_pc16 <= s_pcd(14);
            -- 17: per-channel scale partial products (13 x 7 each, scale split hi/lo)
            s_pY17h <= s_dx16 * signed(resize(unsigned(s_srow(11 downto 6)), 7));
            s_pY17l <= s_dx16 * signed(resize(unsigned(s_srow(5 downto 0)), 7));
            s_pU17h <= s_dx16u * signed(resize(unsigned(s_srow_u(11 downto 6)), 7));
            s_pU17l <= s_dx16u * signed(resize(unsigned(s_srow_u(5 downto 0)), 7));
            s_pV17h <= s_dx16v * signed(resize(unsigned(s_srow_v(11 downto 6)), 7));
            s_pV17l <= s_dx16v * signed(resize(unsigned(s_srow_v(5 downto 0)), 7));
            s_pc17 <= s_pc16;
            -- 18: sums
            s_pY18 <= shift_left(resize(s_pY17h, 26), 6) + resize(s_pY17l, 26);
            s_pU18 <= shift_left(resize(s_pU17h, 26), 6) + resize(s_pU17l, 26);
            s_pV18 <= shift_left(resize(s_pV17h, 26), 6) + resize(s_pV17l, 26);
            s_pc18 <= s_pc17;
            -- 19: addresses
            s_aY18 <= s_pc18 + resize(shift_right(s_pY18, 8), 13);
            s_aU18 <= s_pc18 + resize(shift_right(s_pU18, 8), 13);
            s_aV18 <= s_pc18 + resize(shift_right(s_pV18, 8), 13);
            -- 19: edges (rd at 19 -> wet 20 -> gate... see p_pix)
            edge(s_aY18, s_bgY, s_addrY);
            edge(s_aU18, s_bgU, s_addrU);
            edge(s_aV18, s_bgV, s_addrV);
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addrY));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addrY));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addrU(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addrU(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addrV(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addrV(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            s_a_bgY <= s_bgY; s_a_bgU <= s_bgU; s_a_bgV <= s_bgV; s_a_rbank <= s_rbank;
            if s_a_bgY = '1' then s_wet_y <= to_unsigned(64, 10);
            elsif s_a_rbank = '1' then s_wet_y <= unsigned(s_rdY1); else s_wet_y <= unsigned(s_rdY0); end if;
            if s_a_bgU = '1' then s_wet_u <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then s_wet_u <= unsigned(s_rdU1) & "00"; else s_wet_u <= unsigned(s_rdU0) & "00"; end if;
            if s_a_bgV = '1' then s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then s_wet_v <= unsigned(s_rdV1) & "00"; else s_wet_v <= unsigned(s_rdV0) & "00"; end if;
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y20 <= s_wet_y; s_u20 <= s_wet_u; s_v20 <= s_wet_v;
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

end architecture hallway;
