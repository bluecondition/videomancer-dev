-- Globeswap: catalogue combo #134 (seed 20260927) -- polar warp (spherize /
-- tunnel) + colour channel swap / offset + solarization (Sabattier).  Two
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Lens     lens strength, bipolar (tunnel .. flat .. sphere)
--   K2 Size     lens radius in lines
--   K3 Swap     channel mode: none, U<>V, Y->U, U->Y, Y<>V, rotate, -Y, -YUV
--   K4 Offset   per-channel column offset, bipolar (+-256 px chromatic split)
--   K5 Solar    solarization threshold
--   K6 Fold     fold gain 0.125x .. 8x (log); >1x repeats into bands
--
--   S7  Fringe  even split across the picture / split grows toward the lens rim
--   S8  Swap    channel swap everywhere / only inside the solar region
--   S9  Split   U and V pulled apart, luma still / luma pulled against both chroma
--   S10 Lens    row lens across the whole width / lens only inside the disc (the dry picture outside)
--   S11 Fold    solar region folds luma only / also inverts its chroma
--   P12 Globe   climax: grows the lens (sphere) and its radius, spreads the channels, drives the fold
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- Per-row lens scale from a 64-entry sphere / tunnel table (per-line
-- sequencer), DDA source column plus a per-channel offset, three edge-clamped
-- addresses into a dual-bank line buffer (Y, U, V read from different
-- columns); a 32-deep dry ring supplies the unlensed pixel outside the disc;
-- solar_fold on the fetched pixel, then the swap.  Latency 18 clocks, all
-- modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture globeswap of program_top is

    constant C_LAT : integer := 18;

    type t_tab is array (0 to 63) of integer range 0 to 2047;
    constant C_SPH : t_tab := (256, 256, 256, 256, 257, 257, 257, 258, 258, 259, 259, 260, 261, 261, 262, 263, 264, 266, 267, 268, 269, 271, 273, 274, 276, 278, 280, 282, 285, 287, 290, 293, 296, 299, 302, 306, 310, 314, 318, 323, 328, 333, 339, 346, 353, 360, 368, 377, 387, 398, 410, 424, 439, 457, 477, 501, 529, 563, 606, 661, 736, 846, 1032, 1454);
    constant C_TUN : t_tab := (256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 244, 243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 219, 217, 214, 212, 209, 206, 203, 200, 197, 193, 190, 186, 182, 178, 174, 169, 165, 160, 155, 149, 143, 137, 131, 124, 116, 108, 99, 89, 77, 63, 45);
    -- circle half-width per ring, Q8: 256 * sqrt(1 - (t/64)^2)
    constant C_CIRC : t_tab := (256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 244, 243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 219, 217, 214, 212, 209, 206, 203, 200, 197, 193, 190, 186, 182, 178, 174, 169, 165, 160, 155, 149, 143, 137, 131, 124, 116, 108, 99, 89, 77, 63, 45);

    -- fold gain, Q8: 0.125x .. 8x over the knob, log spaced
    type t_gain_rom is array (0 to 63) of unsigned(11 downto 0);
    constant C_GAIN_ROM : t_gain_rom := (
        to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12),
        to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12), to_unsigned(32, 12),
        to_unsigned(32, 12), to_unsigned(36, 12), to_unsigned(41, 12), to_unsigned(47, 12), to_unsigned(54, 12), to_unsigned(61, 12), to_unsigned(70, 12), to_unsigned(79, 12),
        to_unsigned(90, 12), to_unsigned(103, 12), to_unsigned(117, 12), to_unsigned(134, 12), to_unsigned(152, 12), to_unsigned(173, 12), to_unsigned(197, 12), to_unsigned(225, 12),
        to_unsigned(256, 12), to_unsigned(292, 12), to_unsigned(332, 12), to_unsigned(378, 12), to_unsigned(431, 12), to_unsigned(490, 12), to_unsigned(559, 12), to_unsigned(636, 12),
        to_unsigned(725, 12), to_unsigned(825, 12), to_unsigned(940, 12), to_unsigned(1070, 12), to_unsigned(1219, 12), to_unsigned(1388, 12), to_unsigned(1581, 12), to_unsigned(1801, 12),
        to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12),
        to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2048, 12)
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_ring is array (0 to 31) of std_logic_vector(31 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal ring : t_ring := (others => x"04080200");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_ring_q : std_logic_vector(31 downto 0) := x"04080200";
    signal s_ring_w : unsigned(4 downto 0) := (others => '0');

    signal s_k_lens, s_k_size, s_k_swap, s_k_off, s_k_thr, s_k_fold : unsigned(9 downto 0);
    signal s_sw_fringe, s_sw_swapreg, s_sw_ysplit, s_sw_disc, s_sw_foldc : std_logic;
    signal s_p_globe : unsigned(9 downto 0);

    -- per-frame control decode (registered chains)
    signal s_kb     : signed(8 downto 0) := (others => '0');
    signal s_p12h   : unsigned(8 downto 0) := (others => '0');
    signal s_kbe    : signed(10 downto 0) := (others => '0');
    signal s_ksph   : std_logic := '1';
    signal s_kabs   : unsigned(9 downto 0) := (others => '0');
    signal s_rad    : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_rade   : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_rad_l  : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_mode   : unsigned(2 downto 0) := (others => '0');
    signal s_off    : signed(9 downto 0) := (others => '0');
    signal s_offe   : signed(10 downto 0) := (others => '0');
    signal s_thr    : unsigned(9 downto 0) := (others => '1');
    signal s_base   : signed(11 downto 0) := (others => '0');
    signal s_gsum   : unsigned(6 downto 0) := (others => '0');
    signal s_gidx   : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal s_gain   : unsigned(11 downto 0) := to_unsigned(256, 12);
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
    signal s_circ   : unsigned(8 downto 0) := (others => '0');
    signal s_fm1    : signed(11 downto 0) := (others => '0');
    signal s_hwp    : unsigned(19 downto 0) := (others => '0');
    signal s_fk     : signed(22 downto 0) := (others => '0');
    signal s_srel   : signed(11 downto 0) := to_signed(256, 12);
    signal s_hw     : unsigned(10 downto 0) := (others => '0');
    signal s_srow   : signed(11 downto 0) := to_signed(256, 12);
    signal s_dl, s_dr_b : signed(11 downto 0) := (others => '0');
    signal s_dev    : unsigned(11 downto 0) := (others => '0');
    signal s_gfac   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_cxs    : signed(23 downto 0) := (others => '0');
    signal s_src0   : signed(20 downto 0) := (others => '0');
    signal s_offp   : signed(21 downto 0) := (others => '0');
    signal s_offrow : signed(9 downto 0) := (others => '0');

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    function f_edge_addr(v : signed; we : unsigned(10 downto 0)) return unsigned is
    begin
        if v < C_EDGE then
            return to_unsigned(C_EDGE, 11);
        elsif v > signed(resize(we, v'length)) then
            return we;
        else
            return unsigned(v(10 downto 0));
        end if;
    end function;

    type t_x is array (1 to 5) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 5) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_acc  : signed(20 downto 0) := (others => '0');
    signal s_src4 : signed(11 downto 0) := (others => '0');
    signal s_uy5, s_uu5, s_uv5 : signed(12 downto 0) := (others => '0');
    signal s_ge5, s_lt5 : std_logic := '0';
    signal s_ay6, s_au6, s_av6 : unsigned(10 downto 0) := (others => '0');
    signal s_disc6, s_disc7, s_disc8 : std_logic := '0';
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_ys, s_us, s_vs, s_ysol : unsigned(9 downto 0) := (others => '0');
    signal s_regs : std_logic := '0';
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_lens <= unsigned(registers_in(0));
    s_k_size <= unsigned(registers_in(1));
    s_k_swap <= unsigned(registers_in(2));
    s_k_off  <= unsigned(registers_in(3));
    s_k_thr  <= unsigned(registers_in(4));
    s_k_fold <= unsigned(registers_in(5));
    s_sw_fringe  <= registers_in(6)(0);
    s_sw_swapreg <= registers_in(6)(1);
    s_sw_ysplit  <= registers_in(6)(2);
    s_sw_disc    <= registers_in(6)(3);
    s_sw_foldc   <= registers_in(6)(4);
    s_p_globe <= unsigned(registers_in(7));

    -- dry ring: data_in delayed 8 (q aligned with the fetched pixel)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            s_ring_q <= ring(to_integer(s_ring_w - 7));
            ring(to_integer(s_ring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    -- solarization on the fetched pixel (stages 9 -> 16)
    solar : entity work.solar_fold
        port map (clk => clk, y_in => s_y9, u_in => s_u9, v_in => s_v9,
                  thr => s_thr, gain => s_gain, base => s_base, fold_c => s_sw_foldc,
                  enable => '1', y_out => s_ys, u_out => s_us, v_out => s_vs,
                  ysol => s_ysol, region => s_regs);

    p_ctrl : process(clk)
        variable v_r : unsigned(12 downto 0);
        variable v_s : signed(11 downto 0);
        variable v_o : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            -- control decode, one op per register
            s_kb <= resize(shift_right(signed(resize(s_k_lens, 11)) - 512, 1), 9);
            s_p12h <= s_p_globe(9 downto 1);
            s_kbe <= resize(s_kb, 11) + signed(resize(s_p12h, 11));
            if s_kbe >= 0 then s_ksph <= '1'; s_kabs <= unsigned(s_kbe(9 downto 0));
            else s_ksph <= '0'; s_kabs <= unsigned(resize(-s_kbe, 10)); end if;
            s_rad <= to_unsigned(16, 11) + resize(s_k_size(9 downto 1), 11) + resize(s_k_size(9 downto 2), 11);
            s_rade <= s_rad + resize(s_p12h, 11);
            s_mode <= s_k_swap(9 downto 7);
            s_off <= resize(shift_right(signed(resize(s_k_off, 11)) - 512, 1), 10);
            s_offe <= resize(s_off, 11) + signed(resize(s_p_globe(9 downto 2), 11));
            s_thr <= s_k_thr;
            s_base <= signed(resize(s_k_thr, 12)) - 512;
            s_gsum <= resize(s_k_fold(9 downto 4), 7) + resize(s_p_globe(9 downto 6), 7);
            if s_gsum > 63 then s_gidx <= (others => '1'); else s_gidx <= s_gsum(5 downto 0); end if;
            s_gain <= C_GAIN_ROM(to_integer(s_gidx));

            case to_integer(s_vstep) is
                when 0 =>
                    s_dq <= (others => '0'); s_dr <= (others => '0'); s_dbit <= to_unsigned(16, 5);
                    s_rad_l <= s_rade;
                    s_cx <= '0' & s_line_width(10 downto 1);
                    s_cy <= '0' & s_height(10 downto 1);
                    s_vstep <= s_vstep + 1;
                when 1 to 17 =>
                    if s_dbit = 16 then v_r := (s_dr & '1'); else v_r := (s_dr & '0'); end if;
                    if v_r >= resize(s_rad_l, 13) then
                        s_dr <= resize(v_r - resize(s_rad_l, 13), 12);
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
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_tp <= s_ady * s_inv_r;
                    if s_ady >= s_rad_l then s_beyond <= '1'; else s_beyond <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    if s_beyond = '1' then s_t6 <= (others => '0'); else s_t6 <= s_tp(15 downto 10); end if;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    if s_ksph = '1' then s_f <= to_unsigned(C_SPH(to_integer(s_t6)), 11); else s_f <= to_unsigned(C_TUN(to_integer(s_t6)), 11); end if;
                    if s_beyond = '1' then s_circ <= (others => '0'); else s_circ <= to_unsigned(C_CIRC(to_integer(s_t6)), 9); end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_beyond = '1' then s_fm1 <= (others => '0'); else s_fm1 <= signed(resize(s_f, 12)) - 256; end if;
                    s_hwp <= s_rad_l * s_circ;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_fk <= s_fm1 * signed(resize(s_kabs, 11));
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    v_s := to_signed(256, 12) + resize(shift_right(s_fk, 7), 12);
                    if s_fk >= 229376 then v_s := to_signed(2047, 12); elsif v_s < 32 then v_s := to_signed(32, 12); end if;
                    s_srel <= v_s;
                    s_hw <= s_hwp(18 downto 8);
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_srow <= s_srel;
                    s_dl <= signed(resize(s_cx, 12)) - signed(resize(s_hw, 12));
                    s_dr_b <= signed(resize(s_cx, 12)) + signed(resize(s_hw, 12));
                    -- lens deviation of this row, |scale - 1|
                    if s_srel >= 256 then s_dev <= unsigned(s_srel - 256); else s_dev <= unsigned(to_signed(256, 12) - s_srel); end if;
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    s_cxs <= signed(resize(s_cx, 12)) * s_srow;
                    if s_sw_fringe = '0' then s_gfac <= to_unsigned(256, 10);
                    elsif s_dev > 767 then s_gfac <= to_unsigned(1023, 10);
                    else s_gfac <= to_unsigned(256, 10) + s_dev(9 downto 0); end if;
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    s_src0 <= resize(shift_left(resize(signed(resize(s_cx, 12)), 21), 8) - resize(s_cxs, 21) - resize(s_srow, 21), 21);
                    s_offp <= s_offe * signed(resize(s_gfac, 11));
                    s_lstep <= s_lstep + 1;
                when 10 =>
                    v_o := resize(shift_right(s_offp, 8), 14);
                    if v_o > 511 then v_o := to_signed(511, 14); elsif v_o < -511 then v_o := to_signed(-511, 14); end if;
                    s_offrow <= resize(v_o, 10);
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_lstep <= (others => '0');
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
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

    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 5 loop s_x(i) <= s_x(i - 1); s_avd(i) <= s_avd(i - 1); end loop;
            -- 3: DDA
            if s_avd(1) = '0' then s_acc <= s_src0; else s_acc <= s_acc + resize(s_srow, 21); end if;
            -- 4: source x
            s_src4 <= s_acc(19 downto 8);
            -- 5: per-channel source columns; disc bounds test on the screen column
            if s_sw_ysplit = '1' then s_uy5 <= resize(s_src4, 13) - resize(s_offrow, 13); else s_uy5 <= resize(s_src4, 13); end if;
            s_uu5 <= resize(s_src4, 13) + resize(s_offrow, 13);
            if s_sw_ysplit = '1' then s_uv5 <= resize(s_src4, 13) + resize(s_offrow, 13); else s_uv5 <= resize(s_src4, 13) - resize(s_offrow, 13); end if;
            if signed(resize(s_x(3), 12)) >= s_dl then s_ge5 <= '1'; else s_ge5 <= '0'; end if;
            if signed(resize(s_x(3), 12)) < s_dr_b then s_lt5 <= '1'; else s_lt5 <= '0'; end if;
            -- 6: edge clamp per channel, disc flag
            s_ay6 <= f_edge_addr(s_uy5, s_w_edge);
            s_au6 <= f_edge_addr(s_uu5, s_w_edge);
            s_av6 <= f_edge_addr(s_uv5, s_w_edge);
            s_disc6 <= s_ge5 and s_lt5;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_ay6));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_ay6));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_au6(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_au6(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_av6(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_av6(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_outside : std_logic;
        variable v_m : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- 7 (rd) / 8: fetch
            s_a_rbank <= s_rbank; s_disc7 <= s_disc6; s_disc8 <= s_disc7;
            if s_a_rbank = '1' then s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00"; end if;
            -- 9: lensed / dry pick (dry outside the disc in Disc mode) -> solar_fold (9 -> 16)
            v_outside := s_sw_disc and not s_disc8;
            if v_outside = '1' then
                s_y9 <= unsigned(s_ring_q(29 downto 20)); s_u9 <= unsigned(s_ring_q(19 downto 10)); s_v9 <= unsigned(s_ring_q(9 downto 0));
            else
                s_y9 <= s_wet_y; s_u9 <= s_wet_u; s_v9 <= s_wet_v;
            end if;
            -- 17: channel swap (everywhere, or only inside the solar region)
            if s_sw_swapreg = '1' and s_regs = '0' then v_m := "000"; else v_m := s_mode; end if;
            case to_integer(v_m) is
                when 0 => s_y17 <= s_ys; s_u17 <= s_us; s_v17 <= s_vs;
                when 1 => s_y17 <= s_ys; s_u17 <= s_vs; s_v17 <= s_us;
                when 2 => s_y17 <= s_ys; s_u17 <= s_ys; s_v17 <= s_vs;
                when 3 => s_y17 <= s_us; s_u17 <= s_us; s_v17 <= s_vs;
                when 4 => s_y17 <= s_vs; s_u17 <= s_us; s_v17 <= s_ys;
                when 5 => s_y17 <= s_us; s_u17 <= s_vs; s_v17 <= s_ys;
                when 6 => s_y17 <= not s_ys; s_u17 <= s_us; s_v17 <= s_vs;
                when others => s_y17 <= not s_ys; s_u17 <= not s_us; s_v17 <= not s_vs;
            end case;
            -- 18: gate (luma capped at 940)
            if s_avid_sr(C_LAT - 2) = '1' then
                if s_y17 > 940 then s_y18 <= to_unsigned(940, 10); else s_y18 <= s_y17; end if;
                s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y18);
    data_out.u       <= std_logic_vector(s_u18);
    data_out.v       <= std_logic_vector(s_v18);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture globeswap;
