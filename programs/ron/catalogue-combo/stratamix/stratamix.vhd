-- Stratamix: catalogue combo #132 (seed 20260927) -- depth-from-luma
-- parallax + posterization + multi-source mixing (add, screen, multiply,
-- difference, overlay).  Two knobs each; the switches and the slider decide
-- how they combine.
--
--   K1 Depth    parallax gain, bipolar (luma above the pivot shifts one way, below it the other)
--   K2 Pivot    luma level that does not move
--   K3 Levels   posterize luma to 1 .. 6 bits
--   K4 Tint     chroma tint per luma band (the strata take alternating hues)
--   K5 Mode     add / screen / multiply / difference / overlay
--   K6 Mix      how far the blend replaces the displaced layer
--
--   S7  Order   posterize before the shift (stepped depth: terraces) / after the mix
--   S8  Layers  all strata shift the same way / alternate levels shift opposite ways
--   S9  Chroma  colour from the displaced layer / from the still picture (colour ghosts)
--   S10 Key     blend everywhere / only above the pivot (the background stays put)
--   S11 Contour plain / black contour lines at every level boundary
--   P12 Strata  depth and mix rise together: at 100 % the picture separates into stepped terraces
--
-- Operand A = the line above read at x + (luma - pivot) * depth (edge-filled),
-- operand B = the still picture from a dry EBR ring; two 10x10 products give
-- all five modes; floor quantizers with registered mask / half run before the
-- shift or after the mix.  Latency 23 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture stratamix of program_top is

    constant C_LAT  : integer := 23;
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

    function f_clamp940(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 940 then return to_unsigned(940, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

    function f_clamp959(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 959 then return to_unsigned(959, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

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

    -- floor quantizer: keep the masked bits, sit on the step's half
    function f_quant(y : unsigned(9 downto 0); m : unsigned(9 downto 0); h : unsigned(9 downto 0)) return unsigned is
    begin
        return (y and m) or h;
    end function;

    --------------------------------------------------------------------------
    -- Line buffers + dry ring
    --------------------------------------------------------------------------
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

    --------------------------------------------------------------------------
    -- Controls / per-frame
    --------------------------------------------------------------------------
    signal s_k_depth, s_k_pivot, s_k_levels, s_k_tint, s_k_mode, s_k_mix : unsigned(9 downto 0);
    signal s_sw_after, s_sw_alt, s_sw_bchroma, s_sw_key, s_sw_cont : std_logic;
    signal s_p_strata : unsigned(9 downto 0);

    signal s_gk, s_g_e : signed(9 downto 0) := (others => '0');
    signal s_pivot  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_mask   : unsigned(9 downto 0) := "1111000000";
    signal s_half   : unsigned(9 downto 0) := "0000100000";
    signal s_stepm  : unsigned(9 downto 0) := "0000111111";
    signal s_pbit   : unsigned(9 downto 0) := "0001000000";
    signal s_cw     : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_tint   : unsigned(7 downto 0) := (others => '0');
    signal s_mode   : unsigned(2 downto 0) := (others => '0');
    signal s_mix, s_mix_e : unsigned(7 downto 0) := (others => '0');
    signal s_p      : unsigned(9 downto 0) := (others => '0');
    signal s_mpx    : unsigned(17 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";

    --------------------------------------------------------------------------
    -- Tracking / write path
    --------------------------------------------------------------------------
    signal s_rx, s_wr_x, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank : std_logic := '0';
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_wu2, s_wv2 : unsigned(9 downto 0) := (others => '0');
    signal s_wx2    : unsigned(10 downto 0) := (others => '0');
    signal s_we2, s_wpar2 : std_logic := '0';

    --------------------------------------------------------------------------
    -- Address chain (stages 1..8)
    --------------------------------------------------------------------------
    type t_x is array (1 to 6) of unsigned(10 downto 0);
    signal s_x      : t_x := (others => (others => '0'));
    signal s_dq2    : unsigned(9 downto 0) := (others => '0');
    signal s_par2, s_key2, s_par3 : std_logic := '0';
    signal s_d3, s_d4 : signed(10 downto 0) := (others => '0');
    signal s_g4     : signed(9 downto 0) := (others => '0');
    signal s_pd5    : signed(20 downto 0) := (others => '0');
    signal s_off6   : signed(10 downto 0) := (others => '0');
    signal s_u7     : signed(12 downto 0) := (others => '0');
    signal s_addr8  : unsigned(10 downto 0) := (others => '0');
    signal s_keyd   : std_logic_vector(3 to 17) := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel path (stages 10..23)
    --------------------------------------------------------------------------
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y10, s_wet_u10, s_wet_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_by10, s_bu10, s_bv10 : unsigned(9 downto 0) := (others => '0');
    signal s_a11, s_bq11 : unsigned(9 downto 0) := (others => '0');
    signal s_a12, s_b12, s_na12 : unsigned(9 downto 0) := (others => '0');
    signal s_a13, s_b13, s_na13, s_nb13 : unsigned(9 downto 0) := (others => '0');
    signal s_sum13, s_sum14 : unsigned(10 downto 0) := (others => '0');
    signal s_dif13, s_dif14 : signed(10 downto 0) := (others => '0');
    signal s_p14, s_q14 : unsigned(19 downto 0) := (others => '0');
    signal s_a14    : unsigned(9 downto 0) := (others => '0');
    signal s_c_add15, s_c_scr15, s_c_mul15, s_c_dif15, s_c_ovl15, s_ovh15 : signed(11 downto 0) := (others => '0');
    signal s_alo15  : std_logic := '0';
    signal s_sel16  : signed(11 downto 0) := (others => '0');
    signal s_t17    : unsigned(9 downto 0) := (others => '0');
    signal s_dy18   : signed(10 downto 0) := (others => '0');
    signal s_mx18   : unsigned(7 downto 0) := (others => '0');
    signal s_py19   : signed(19 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := (others => '0');
    signal s_yq21, s_u21, s_v21 : unsigned(9 downto 0) := (others => '0');
    signal s_c21    : std_logic := '0';
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := (others => '0');
    type t_ad is array (11 to 17) of unsigned(9 downto 0);
    signal s_ad     : t_ad := (others => (others => '0'));
    type t_wyd is array (10 to 19) of unsigned(9 downto 0);
    signal s_wyd    : t_wyd := (others => (others => '0'));
    type t_cd is array (11 to 19) of unsigned(9 downto 0);
    signal s_ud, s_vd : t_cd := (others => (others => '0'));
    type t_byd is array (10 to 20) of unsigned(9 downto 0);
    signal s_byd    : t_byd := (others => (others => '0'));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_depth  <= unsigned(registers_in(0));
    s_k_pivot  <= unsigned(registers_in(1));
    s_k_levels <= unsigned(registers_in(2));
    s_k_tint   <= unsigned(registers_in(3));
    s_k_mode   <= unsigned(registers_in(4));
    s_k_mix    <= unsigned(registers_in(5));
    s_sw_after   <= registers_in(6)(0);
    s_sw_alt     <= registers_in(6)(1);
    s_sw_bchroma <= registers_in(6)(2);
    s_sw_key     <= registers_in(6)(3);
    s_sw_cont    <= registers_in(6)(4);
    s_p_strata <= unsigned(registers_in(7));

    -- dry ring: data_in delayed 9 (q at stage 9)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            s_ring_q <= ring(to_integer(s_ring_w - 8));
            ring(to_integer(s_ring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_kl : integer range 4 to 9;
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_w_edge <= s_line_width - (C_EDGE + 1);

            -- per-frame decode (one op per state; the one product has its operands registered a state ahead)
            case to_integer(s_vstep) is
                when 0 =>
                    s_gk    <= signed('0' & s_k_depth(9 downto 1)) - to_signed(256, 10);
                    s_pivot <= s_k_pivot;
                    s_tint  <= s_k_tint(9 downto 2);
                    case to_integer(s_k_mode(9 downto 7)) is
                        when 0 | 1 => s_mode <= "000";
                        when 2 | 3 => s_mode <= "001";
                        when 4     => s_mode <= "010";
                        when 5 | 6 => s_mode <= "011";
                        when others => s_mode <= "100";
                    end case;
                    s_mix   <= s_k_mix(9 downto 2);
                    s_p     <= s_p_strata;
                    case to_integer(s_k_levels(9 downto 6)) is
                        when 0 | 1 | 2   => v_kl := 9;
                        when 3 | 4 | 5   => v_kl := 8;
                        when 6 | 7 | 8   => v_kl := 7;
                        when 9 | 10 | 11 => v_kl := 6;
                        when 12 | 13     => v_kl := 5;
                        when others      => v_kl := 4;
                    end case;
                    for i in 0 to 9 loop
                        if i >= v_kl then s_mask(i) <= '1'; else s_mask(i) <= '0'; end if;
                        if i = v_kl - 1 then s_half(i) <= '1'; else s_half(i) <= '0'; end if;
                        if i < v_kl then s_stepm(i) <= '1'; else s_stepm(i) <= '0'; end if;
                        if i = v_kl then s_pbit(i) <= '1'; else s_pbit(i) <= '0'; end if;
                    end loop;
                    case v_kl is
                        when 9 => s_cw <= to_unsigned(64, 7);
                        when 8 => s_cw <= to_unsigned(32, 7);
                        when 7 => s_cw <= to_unsigned(16, 7);
                        when 6 => s_cw <= to_unsigned(8, 7);
                        when others => s_cw <= to_unsigned(4, 7);
                    end case;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_mpx <= (not s_mix) * s_p;                                         -- (255 - mix) * p
                    if s_gk >= 0 then
                        s_g_e <= s_gk + signed(resize(s_p(9 downto 2), 10));
                    else
                        s_g_e <= s_gk - signed(resize(s_p(9 downto 2), 10));
                    end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_mix_e <= s_mix + s_mpx(17 downto 10);
                    s_vstep <= "111";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
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
    -- Write path (stage 2) + address chain (stages 1..8)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_w : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 1
            s_x(1) <= s_rx;
            for i in 2 to 6 loop s_x(i) <= s_x(i - 1); end loop;
            -- 2: depth luma (posterized before the shift), write data, step parity, key
            if s_sw_after = '0' then s_dq2 <= f_quant(s_in_y, s_mask, s_half); else s_dq2 <= s_in_y; end if;
            s_wu2 <= s_in_u; s_wv2 <= s_in_v;
            s_wx2 <= s_wr_x; s_we2 <= s_in_avid; s_wpar2 <= s_wpar;
            if (s_in_y and s_pbit) /= 0 then s_par2 <= '1'; else s_par2 <= '0'; end if;
            if s_in_y >= s_pivot then s_key2 <= '1'; else s_key2 <= '0'; end if;
            -- 3: depth difference
            s_d3 <= signed(resize(s_dq2, 11)) - signed(resize(s_pivot, 11));
            s_par3 <= s_par2;
            s_keyd(3) <= s_key2;
            for i in 4 to 17 loop s_keyd(i) <= s_keyd(i - 1); end loop;
            -- 4: gain (alternate levels reversed), registered ahead of the product
            if s_sw_alt = '1' and s_par3 = '1' then s_g4 <= -s_g_e; else s_g4 <= s_g_e; end if;
            s_d4 <= s_d3;
            -- 5: displacement product (11 x 10)
            s_pd5 <= s_d4 * s_g4;
            -- 6: offset (+-511 px)
            v_w := resize(shift_right(s_pd5, 8), 13);
            if v_w > 511 then v_w := to_signed(511, 13); elsif v_w < -511 then v_w := to_signed(-511, 13); end if;
            s_off6 <= resize(v_w, 11);
            -- 7: source column
            s_u7 <= signed(resize(s_x(6), 13)) + resize(s_off6, 13);
            -- 8: edge fill
            s_addr8 <= f_edge_addr(s_u7, s_w_edge);
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 2, read stage 8 -> data 9)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr8));
            if s_we2 = '1' and s_wpar2 = '0' then lbY0(to_integer(s_wx2)) <= std_logic_vector(s_dq2); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr8));
            if s_we2 = '1' and s_wpar2 = '1' then lbY1(to_integer(s_wx2)) <= std_logic_vector(s_dq2); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr8(10 downto 1)));
            if s_we2 = '1' and s_wpar2 = '0' then lbU0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wu2(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr8(10 downto 1)));
            if s_we2 = '1' and s_wpar2 = '1' then lbU1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wu2(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr8(10 downto 1)));
            if s_we2 = '1' and s_wpar2 = '0' then lbV0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wv2(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr8(10 downto 1)));
            if s_we2 = '1' and s_wpar2 = '1' then lbV1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wv2(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 10..23): operands, blend, mix toward A, posterize, tint, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_tu, v_tv : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            -- 9 (rd) / 10: operand A (displaced, a line late) and B (still, dry ring)
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then
                s_wet_y10 <= unsigned(s_rdY1); s_wet_u10 <= unsigned(s_rdU1) & "00"; s_wet_v10 <= unsigned(s_rdV1) & "00";
            else
                s_wet_y10 <= unsigned(s_rdY0); s_wet_u10 <= unsigned(s_rdU0) & "00"; s_wet_v10 <= unsigned(s_rdV0) & "00";
            end if;
            s_by10 <= unsigned(s_ring_q(29 downto 20));
            s_bu10 <= unsigned(s_ring_q(19 downto 10));
            s_bv10 <= unsigned(s_ring_q(9 downto 0));
            -- 11: black-relative A, posterized B, chroma source
            if s_wet_y10 < 64 then s_a11 <= (others => '0'); else s_a11 <= s_wet_y10 - 64; end if;
            if s_sw_after = '0' then s_bq11 <= f_quant(s_by10, s_mask, s_half); else s_bq11 <= s_by10; end if;
            if s_sw_bchroma = '1' then s_ud(11) <= s_bu10; s_vd(11) <= s_bv10;
            else s_ud(11) <= s_wet_u10; s_vd(11) <= s_wet_v10; end if;
            for i in 12 to 19 loop s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            s_wyd(10) <= s_wet_y10;
            for i in 11 to 19 loop s_wyd(i) <= s_wyd(i - 1); end loop;
            s_byd(10) <= s_by10;
            for i in 11 to 20 loop s_byd(i) <= s_byd(i - 1); end loop;
            -- 12: black-relative B, inverted A
            if s_bq11 < 64 then s_b12 <= (others => '0'); else s_b12 <= s_bq11 - 64; end if;
            s_a12 <= s_a11;
            s_na12 <= to_unsigned(959, 10) - s_a11;
            s_ad(11) <= s_a11;
            for i in 12 to 17 loop s_ad(i) <= s_ad(i - 1); end loop;
            -- 13: inverted B, sum, difference
            s_nb13 <= to_unsigned(959, 10) - s_b12;
            s_a13 <= s_a12; s_b13 <= s_b12; s_na13 <= s_na12;
            s_sum13 <= resize(s_a12, 11) + resize(s_b12, 11);
            s_dif13 <= signed(resize(s_a12, 11)) - signed(resize(s_b12, 11));
            -- 14: products (10 x 10)
            s_p14 <= s_a13 * s_b13;
            s_q14 <= s_na13 * s_nb13;
            s_sum14 <= s_sum13; s_dif14 <= s_dif13; s_a14 <= s_a13;
            -- 15: candidates
            s_c_add15 <= signed(resize(s_sum14, 12));
            s_c_scr15 <= signed(resize(s_sum14, 12)) - signed(resize(s_p14(19 downto 10), 12));
            s_c_mul15 <= signed(resize(s_p14(19 downto 10), 12));
            if s_dif14 < 0 then s_c_dif15 <= -resize(s_dif14, 12); else s_c_dif15 <= resize(s_dif14, 12); end if;
            s_c_ovl15 <= signed(resize(s_p14(19 downto 9), 12));
            s_ovh15 <= to_signed(959, 12) - signed(resize(s_q14(19 downto 9), 12));
            if s_a14 < 480 then s_alo15 <= '1'; else s_alo15 <= '0'; end if;
            -- 16: select
            case to_integer(s_mode) is
                when 0 => s_sel16 <= s_c_add15;
                when 1 => s_sel16 <= s_c_scr15;
                when 2 => s_sel16 <= s_c_mul15;
                when 3 => s_sel16 <= s_c_dif15;
                when others => if s_alo15 = '1' then s_sel16 <= s_c_ovl15; else s_sel16 <= s_ovh15; end if;
            end case;
            -- 17: clamp
            s_t17 <= f_clamp959(s_sel16);
            -- 18: difference toward the blend, keyed mix amount
            s_dy18 <= signed(resize(s_t17, 11)) - signed(resize(s_ad(17), 11));
            if s_sw_key = '0' or s_keyd(17) = '1' then s_mx18 <= s_mix_e; else s_mx18 <= (others => '0'); end if;
            -- 19: mix product (11 x 9)
            s_py19 <= s_dy18 * signed('0' & s_mx18);
            -- 20: mixed luma on the displaced layer
            s_y20 <= f_clamp940(signed(resize(s_wyd(19), 13)) + resize(shift_right(s_py19, 8), 13));
            s_u20 <= s_ud(19); s_v20 <= s_vd(19);
            -- 21: posterize after the mix, contour flag from the still picture
            if s_sw_after = '1' then s_yq21 <= f_quant(s_y20, s_mask, s_half); else s_yq21 <= s_y20; end if;
            if s_sw_cont = '1' and (s_byd(20) and s_stepm) < resize(s_cw, 10) then s_c21 <= '1'; else s_c21 <= '0'; end if;
            s_u21 <= s_u20; s_v21 <= s_v20;
            -- 22: contour ink, band tint
            if s_c21 = '1' then s_y22 <= to_unsigned(64, 10); else s_y22 <= s_yq21; end if;
            if s_yq21(8) = '1' then v_tu := signed('0' & s_tint); else v_tu := -signed('0' & s_tint); end if;
            if s_yq21(9) = '1' then v_tv := signed('0' & s_tint); else v_tv := -signed('0' & s_tint); end if;
            s_u22 <= f_clamp10(signed(resize(s_u21, 12)) + resize(v_tu, 12));
            s_v22 <= f_clamp10(signed(resize(s_v21, 12)) + resize(v_tv, 12));
            -- 23: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y23 <= s_y22; s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y23);
    data_out.u       <= std_logic_vector(s_u23);
    data_out.v       <= std_logic_vector(s_v23);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture stratamix;
