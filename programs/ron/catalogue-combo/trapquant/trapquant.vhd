-- Trapquant: catalogue combo #120 (seed 20260926) -- perspective keystone /
-- corner-pin + posterization (bit-depth reduction per channel).  Three
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Top      width of the picture at the top (25..225%)
--   K2 Bottom   width of the picture at the bottom (25..225%)
--   K3 Slide    horizontal slide, bipolar
--   K4 Luma     posterize luma 1..8 bits
--   K5 Chroma   posterize chroma 1..8 bits
--   K6 Dither   dither amount before quantizing (0..100% of a step)
--
--   S7  Edges   outside the picture: edge fill / mirrored
--   S8  Order   posterize the source (bands stretch with the keystone) /
--               posterize the screen (bands stay put)
--   S9  Ramp    the quantizer offset follows each row's scale: bands
--               slide along the trapezoid
--   S10 Dither  2x2 / alternate lines
--   S11 Chroma  posterized / kept
--   P12 Tilt    rotates the trapezoid: top wide .. bottom wide
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- Line buffer read a line late; per-line DDA source address (scale
-- interpolated top->bottom with a per-frame serial divider); the dither
-- amounts are per-frame products.  Latency 10 clocks, all modes,
-- blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture trapquant of program_top is

    constant C_LAT : integer := 10;

    function f_quant(y : unsigned(9 downto 0); d : unsigned(10 downto 0); nm : unsigned(9 downto 0); h : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        v := resize(y, 12) + resize(d, 12);
        if v > 1023 then v := to_unsigned(1023, 12); end if;
        return (v(9 downto 0) and nm) + h;
    end function;

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_top, s_k_bot, s_k_slide, s_k_lbits, s_k_cbits, s_k_dith : unsigned(9 downto 0);
    signal s_sw_mirror, s_sw_screen, s_sw_ramp, s_sw_lines, s_sw_ckeep : std_logic;
    signal s_p_tilt : unsigned(9 downto 0);

    signal s_slide  : signed(10 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';
    signal s_kl, s_kc : unsigned(3 downto 0) := to_unsigned(5, 4);   -- power-on value keeps the shifts in range
    signal s_lnot, s_lhalf, s_lq, s_lhq : unsigned(9 downto 0) := (others => '0');
    signal s_cnot, s_chalf, s_cq, s_chq : unsigned(9 downto 0) := (others => '0');
    signal s_amt    : unsigned(7 downto 0) := (others => '0');
    -- Shared per-frame multiplier: operand registered one state ahead, product
    -- free-running (no vstep mux in front of the multiply -- it was the HD
    -- critical path); results valid two states after the operand is set.
    signal s_mo     : unsigned(9 downto 0) := (others => '0');
    signal s_pm     : unsigned(17 downto 0) := (others => '0');
    signal s_dsc_ph : signed(19 downto 0) := (others => '0');   -- dtb * inv_h(12..6)
    signal s_dsc_pl : signed(18 downto 0) := (others => '0');   -- dtb * inv_h(5..0)
    signal s_dsc_p  : signed(24 downto 0) := (others => '0');
    signal s_cxs_ph : signed(17 downto 0) := (others => '0');   -- cx * srow(11..6)
    signal s_cxs_pl : signed(18 downto 0) := (others => '0');   -- cx * srow(5..0)
    signal s_cxs_p  : signed(23 downto 0) := (others => '0');
    signal s_d1, s_d2, s_d3, s_c1, s_c2, s_c3 : unsigned(9 downto 0) := (others => '0');
    signal s_ramp   : unsigned(8 downto 0) := (others => '0');
    -- per frame
    signal s_vstep  : unsigned(5 downto 0) := (others => '1');
    signal s_dq     : unsigned(20 downto 0) := (others => '0');
    signal s_dr     : unsigned(11 downto 0) := (others => '0');
    signal s_dbit   : unsigned(4 downto 0) := (others => '0');
    signal s_inv_h  : unsigned(12 downto 0) := to_unsigned(970, 13);
    signal s_top, s_bot : signed(11 downto 0) := to_signed(256, 12);
    signal s_tilt   : signed(11 downto 0) := (others => '0');
    signal s_dtb    : signed(11 downto 0) := (others => '0');
    signal s_dsc    : signed(24 downto 0) := (others => '0');
    signal s_cx     : unsigned(10 downto 0) := to_unsigned(240, 11);
    -- per line
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_acc_s  : signed(32 downto 0) := (others => '0');
    signal s_srow   : signed(11 downto 0) := to_signed(256, 12);
    signal s_src0   : signed(20 downto 0) := (others => '0');
    signal s_w2m1   : signed(12 downto 0) := (others => '0');

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    -- write side
    signal s_wy2, s_wu2, s_wv2, s_wy3, s_wu3, s_wv3 : unsigned(9 downto 0) := (others => '0');
    signal s_wx2, s_wx3 : unsigned(10 downto 0) := (others => '0');
    signal s_we2, s_we3, s_wpar2, s_wpar3 : std_logic := '0';
    signal s_dw2, s_dcw2 : unsigned(10 downto 0) := (others => '0');
    -- read side
    type t_x is array (1 to 8) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 5) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_acc   : signed(20 downto 0) := (others => '0');
    signal s_src3  : signed(11 downto 0) := (others => '0');
    signal s_a4    : signed(12 downto 0) := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_d8, s_dc8 : unsigned(10 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    -- per-line clamp bounds (registered) so the address compares run on the
    -- source value in parallel with the mirror negate / subtract
    signal s_we13, s_wl13, s_nwe13, s_rce13, s_rwe13 : signed(12 downto 0) := (others => '0');
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

    function f_pick(z : unsigned(1 downto 0); d1, d2, d3 : unsigned(9 downto 0)) return unsigned is
    begin
        case to_integer(z) is
            when 0 => return to_unsigned(0, 10);
            when 1 => return d1;
            when 2 => return d2;
            when others => return d3;
        end case;
    end function;

begin

    s_k_top   <= unsigned(registers_in(0));
    s_k_bot   <= unsigned(registers_in(1));
    s_k_slide <= unsigned(registers_in(2));
    s_k_lbits <= unsigned(registers_in(3));
    s_k_cbits <= unsigned(registers_in(4));
    s_k_dith  <= unsigned(registers_in(5));
    s_sw_mirror <= registers_in(6)(0);
    s_sw_screen <= registers_in(6)(1);
    s_sw_ramp   <= registers_in(6)(2);
    s_sw_lines  <= registers_in(6)(3);
    s_sw_ckeep  <= registers_in(6)(4);
    s_p_tilt <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_r : unsigned(12 downto 0);
        variable v_t : signed(11 downto 0);
        variable v_m : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_slide <= resize(shift_right(signed(resize(s_k_slide, 11)) - 512, 0), 11);
            s_tilt <= resize(shift_right(signed(resize(s_p_tilt, 11)) - 512, 1), 12);
            s_kl <= to_unsigned(9, 4) - resize(s_k_lbits(9 downto 7), 4);
            s_kc <= to_unsigned(9, 4) - resize(s_k_cbits(9 downto 7), 4);
            v_m := shift_left(to_unsigned(1, 10), to_integer(s_kl)) - 1;
            s_lnot <= not v_m; s_lhalf <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 1);
            s_lq <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 2);
            v_m := shift_left(to_unsigned(1, 10), to_integer(s_kc)) - 1;
            s_cnot <= not v_m; s_chalf <= shift_left(to_unsigned(1, 10), to_integer(s_kc) - 1);
            s_cq <= shift_left(to_unsigned(1, 10), to_integer(s_kc) - 2);
            s_lhq <= s_lhalf + s_lq; s_chq <= s_chalf + s_cq;
            s_amt <= s_k_dith(9 downto 2);
            -- per-frame: inverse height (2^20 / height, 21-step restoring divider), scale endpoints
            case to_integer(s_vstep) is
                when 0 =>
                    s_dq <= (others => '0'); s_dr <= (others => '0'); s_dbit <= to_unsigned(20, 5);
                    s_cx <= '0' & s_line_width(10 downto 1);
                    s_vstep <= s_vstep + 1;
                when 1 to 21 =>
                    -- numerator 2^20: bit 20 is 1, all lower bits 0
                    if s_dbit = 20 then v_r := (s_dr(11 downto 0) & '1'); else v_r := (s_dr(11 downto 0) & '0'); end if;
                    if v_r >= resize(s_height, 13) then
                        s_dr <= resize(v_r - resize(s_height, 13), 12);
                        s_dq <= s_dq(19 downto 0) & '1';
                    else
                        s_dr <= v_r(11 downto 0);
                        s_dq <= s_dq(19 downto 0) & '0';
                    end if;
                    s_dbit <= s_dbit - 1;
                    s_vstep <= s_vstep + 1;
                when 22 =>
                    s_inv_h <= s_dq(12 downto 0);
                    v_t := signed(resize(s_k_top(9 downto 1), 12)) + 64 + s_tilt;
                    if v_t < 32 then v_t := to_signed(32, 12); end if;
                    s_top <= v_t;
                    v_t := signed(resize(s_k_bot(9 downto 1), 12)) + 64 - s_tilt;
                    if v_t < 32 then v_t := to_signed(32, 12); end if;
                    s_bot <= v_t;
                    s_vstep <= s_vstep + 1;
                when 23 =>
                    s_dtb <= s_bot - s_top;
                    s_vstep <= s_vstep + 1;
                when 24 | 25 =>
                    s_vstep <= s_vstep + 1;                     -- s_dsc_p settles (split product, 2 states)
                when 26 => s_dsc <= s_dsc_p; s_mo <= s_lhalf; s_vstep <= s_vstep + 1;   -- Q20 scale step per line
                when 27 => s_mo <= s_lhq;   s_vstep <= s_vstep + 1;
                when 28 => s_mo <= s_lq;    s_d1 <= s_pm(17 downto 8); s_vstep <= s_vstep + 1;
                when 29 => s_mo <= s_chalf; s_d2 <= s_pm(17 downto 8); s_vstep <= s_vstep + 1;
                when 30 => s_mo <= s_chq;   s_d3 <= s_pm(17 downto 8); s_vstep <= s_vstep + 1;
                when 31 => s_mo <= s_cq;    s_c1 <= s_pm(17 downto 8); s_vstep <= s_vstep + 1;
                when 32 => s_c2 <= s_pm(17 downto 8); s_vstep <= s_vstep + 1;
                when 33 => s_c3 <= s_pm(17 downto 8); s_vstep <= (others => '1');
                when others => null;
            end case;

            -- free-running sequencer products (operands are registered one state ahead)
            s_pm    <= s_mo * s_amt;
            s_dsc_ph <= s_dtb * signed('0' & s_inv_h(12 downto 6));
            s_dsc_pl <= s_dtb * signed('0' & s_inv_h(5 downto 0));
            s_dsc_p  <= shift_left(resize(s_dsc_ph, 25), 6) + resize(s_dsc_pl, 25);
            s_cxs_ph <= signed(resize(s_cx, 12)) * s_srow(11 downto 6);
            s_cxs_pl <= signed(resize(s_cx, 12)) * signed('0' & s_srow(5 downto 0));
            s_cxs_p  <= shift_left(resize(s_cxs_ph, 24), 6) + resize(s_cxs_pl, 24);

            -- per line
            case to_integer(s_lstep) is
                when 0 =>
                    s_srow <= s_top + resize(shift_right(s_acc_s, 20), 12);
                    s_w2m1 <= signed(resize(s_line_width & '0', 13)) - 1;
                    s_lstep <= s_lstep + 1;
                when 1 | 2 =>
                    s_lstep <= s_lstep + 1;                              -- s_cxs_p = cx * scale (Q8) settles (split product, 2 states)
                when 3 =>
                    -- src0 (Q8) = (cx + slide) << 8 - cx * scale
                    s_src0 <= resize(shift_left(resize(signed(resize(s_cx, 12)) + resize(s_slide, 12), 21), 8) - resize(s_cxs_p, 21), 21);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_sw_ramp = '1' then s_ramp <= unsigned(s_srow(9 downto 1)); else s_ramp <= (others => '0'); end if;
                    s_src0 <= s_src0 - resize(s_srow, 21);
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we13  <= signed(resize(s_w_edge, 13));                  -- w_edge
            s_wl13  <= signed(resize(s_line_width, 13));              -- width
            s_nwe13 <= -signed(resize(s_w_edge, 13));                 -- mirrored-left: n > w_edge  <=> src < -w_edge
            s_rce13 <= s_w2m1 - C_EDGE;                               -- mirrored-right: r < C_EDGE <=> src > 2w-1-C_EDGE
            s_rwe13 <= signed(resize(s_line_width, 13)) + C_EDGE;     -- mirrored-right: r > w_edge <=> src < w+C_EDGE
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
                s_acc_s <= s_acc_s + resize(s_dsc, 33);
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0'; s_acc_s <= (others => '0');
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
        variable v_z : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 8 loop s_x(i) <= s_x(i - 1); end loop;
            for i in 2 to 5 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- write side 2: dither values; 3: quantize (source order) and write
            if s_sw_lines = '1' then v_z := s_line(0) & '0'; else v_z := s_wr_x(0) & s_line(0); end if;
            s_dw2 <= resize(f_pick(v_z, s_d1, s_d2, s_d3), 11) + resize(s_ramp, 11);
            s_dcw2 <= resize(f_pick(v_z, s_c1, s_c2, s_c3), 11);
            s_wy2 <= s_in_y; s_wu2 <= s_in_u; s_wv2 <= s_in_v;
            s_wx2 <= s_wr_x; s_we2 <= s_we; s_wpar2 <= s_wpar;
            if s_sw_screen = '0' then
                s_wy3 <= f_quant(s_wy2, s_dw2, s_lnot, s_lhalf);
                if s_sw_ckeep = '1' then s_wu3 <= s_wu2; s_wv3 <= s_wv2;
                else s_wu3 <= f_quant(s_wu2, s_dcw2, s_cnot, s_chalf); s_wv3 <= f_quant(s_wv2, s_dcw2, s_cnot, s_chalf); end if;
            else
                s_wy3 <= s_wy2; s_wu3 <= s_wu2; s_wv3 <= s_wv2;
            end if;
            s_wx3 <= s_wx2; s_we3 <= s_we2; s_wpar3 <= s_wpar2;
            -- 2: DDA (preloaded during blanking)
            if s_avd(1) = '0' then s_acc <= s_src0; else s_acc <= s_acc + resize(s_srow, 21); end if;
            -- 3: source x
            s_src3 <= s_acc(19 downto 8);
            -- 4
            s_a4 <= resize(s_src3, 13);
            -- 5: edges
            -- every compare is on s_a4 against a registered bound, in parallel
            -- with the negate / subtract; the mux is the only serial step
            v_neg := -s_a4;
            v_ref := s_w2m1 - s_a4;
            if s_sw_mirror = '1' and s_a4 < 0 then
                if s_a4 > -C_EDGE then s_addr <= to_unsigned(C_EDGE, 11);
                elsif s_a4 < s_nwe13 then s_addr <= s_w_edge;
                else s_addr <= unsigned(v_neg(10 downto 0)); end if;
            elsif s_sw_mirror = '1' and s_a4 >= s_wl13 then
                if s_a4 > s_rce13 then s_addr <= to_unsigned(C_EDGE, 11);
                elsif s_a4 < s_rwe13 then s_addr <= s_w_edge;
                else s_addr <= unsigned(v_ref(10 downto 0)); end if;
            elsif s_a4 < C_EDGE then s_addr <= to_unsigned(C_EDGE, 11);
            elsif s_a4 > s_we13 then s_addr <= s_w_edge;
            else s_addr <= unsigned(s_a4(10 downto 0)); end if;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we3 = '1' and s_wpar3 = '0' then lbY0(to_integer(s_wx3)) <= std_logic_vector(s_wy3); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we3 = '1' and s_wpar3 = '1' then lbY1(to_integer(s_wx3)) <= std_logic_vector(s_wy3); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we3 = '1' and s_wpar3 = '0' then lbU0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_wu3(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we3 = '1' and s_wpar3 = '1' then lbU1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_wu3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we3 = '1' and s_wpar3 = '0' then lbV0(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_wv3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we3 = '1' and s_wpar3 = '1' then lbV1(to_integer(s_wx3(10 downto 1))) <= std_logic_vector(s_wv3(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_z : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            -- 6 (rd) / 7: fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 8: dither values (screen order)
            if s_sw_lines = '1' then v_z := s_line(0) & '0'; else v_z := s_x(6)(0) & s_line(0); end if;
            s_d8 <= resize(f_pick(v_z, s_d1, s_d2, s_d3), 11) + resize(s_ramp, 11);
            s_dc8 <= resize(f_pick(v_z, s_c1, s_c2, s_c3), 11);
            s_y8 <= s_wet_y; s_u8 <= s_wet_u; s_v8 <= s_wet_v;
            -- 9: quantize
            if s_sw_screen = '1' then
                s_y9 <= f_quant(s_y8, s_d8, s_lnot, s_lhalf);
                if s_sw_ckeep = '1' then s_u9 <= s_u8; s_v9 <= s_v8;
                else s_u9 <= f_quant(s_u8, s_dc8, s_cnot, s_chalf); s_v9 <= f_quant(s_v8, s_dc8, s_cnot, s_chalf); end if;
            else
                s_y9 <= s_y8; s_u9 <= s_u8; s_v9 <= s_v8;
            end if;
            -- 10: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            else
                s_y10 <= to_unsigned(64, 10); s_u10 <= to_unsigned(512, 10); s_v10 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y10);
    data_out.u       <= std_logic_vector(s_u10);
    data_out.v       <= std_logic_vector(s_v10);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture trapquant;
