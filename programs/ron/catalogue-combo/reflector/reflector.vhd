-- Reflector: catalogue combo #8 (seed 20260912) -- mirror / kaleidoscope +
-- bloom / glow.  Three knobs each; the switches and the slider decide how
-- they combine.
--
--   K1 Axis      position of the first mirror axis (0..100% of the width)
--   K2 Folds     0 / 1 / 2 / 3 nested folds (1, 2, 4, 8 panes)
--   K3 Slide     the source scrolls under the mirrors, bipolar
--   K4 Glow      bloom gain
--   K5 Threshold bloom threshold
--   K6 Radius    blur width 9 / 17 / 33 px
--
--   S7  Glow     glow fetched mirrored with the picture / unmirrored
--   S8  Panes    alternate panes shown as negative
--   S9  Tint     warm glow
--   S10 Seams    extra glow along the mirror seams
--   S11 Soft     base picture replaced by its blur
--   P12 Sweep    the first axis swings on a sine, speed = slider
--
-- Nested folds: u = a - |t - a| applied up to three times with halving
-- axes (no division).  Blur written to a fourth line buffer.  Latency 18
-- clocks, all modes, blanking-gated.  22 EBR.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture reflector of program_top is

    constant C_LAT : integer := 18;
    constant C_BLUR_CENTRE : integer := 16;

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

    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251,
        -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215,
        -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147,
        -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,
         -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,
          50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,
         142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,
         212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,
         250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,
         250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,
         212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,
         142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,
          50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,
         -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136,
        -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208,
        -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249,
        -250, -251, -252, -253, -254, -254, -255, -255
    );

    --------------------------------------------------------------------------
    -- Line buffers: Y, U, V, G
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal lbG0, lbG1 : t_lb_c := (others => x"10");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdG0, s_rdG1 : std_logic_vector(7 downto 0) := x"10";

    --------------------------------------------------------------------------
    -- Controls / per-frame
    --------------------------------------------------------------------------
    signal s_k_axis, s_k_folds, s_k_slide, s_k_glow, s_k_thr, s_k_radius : unsigned(9 downto 0);
    signal s_sw_gunm, s_sw_neg, s_sw_tint, s_sw_seams, s_sw_soft : std_logic;
    signal s_p_sweep : unsigned(9 downto 0);
    signal s_gain   : unsigned(7 downto 0) := (others => '0');
    signal s_thr    : unsigned(9 downto 0) := (others => '0');
    signal s_rad    : unsigned(1 downto 0) := "01";
    signal s_folds  : unsigned(1 downto 0) := "01";
    signal s_a1, s_a2, s_a3 : signed(13 downto 0) := (others => '0');
    signal s_axp    : unsigned(20 downto 0) := (others => '0');
    signal s_swp    : signed(20 downto 0) := (others => '0');
    signal s_sweep_ph : unsigned(15 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q  : signed(9 downto 0) := (others => '0');
    signal s_sin_q2 : signed(9 downto 0) := (others => '0');
    signal s_slide  : signed(12 downto 0) := (others => '0');
    signal s_off    : signed(12 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";
    signal s_saw_active : std_logic := '0';

    --------------------------------------------------------------------------
    -- Tracking / delayed write
    --------------------------------------------------------------------------
    signal s_rx      : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_wpar, s_rbank : std_logic := '0';
    type t_x is array (1 to 7) of unsigned(10 downto 0);
    signal s_wx : t_x := (others => (others => '0'));
    type t_b7 is array (1 to 7) of std_logic;
    signal s_wp : t_b7 := (others => '0');
    type t_y7 is array (1 to 7) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_y7 := (others => (others => '0'));
    signal s_we8, s_wp8 : std_logic := '0';
    signal s_wx8 : unsigned(10 downto 0) := (others => '0');
    signal s_wy8, s_wu8, s_wv8 : unsigned(9 downto 0) := (others => '0');
    signal s_wg8 : unsigned(7 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Blur cascade (as halation)
    --------------------------------------------------------------------------
    type t_taps is array (0 to 16) of unsigned(9 downto 0);
    signal s_t1, s_t2 : t_taps := (others => (others => '0'));
    signal s_acc1, s_acc2 : unsigned(14 downto 0) := (others => '0');
    signal s_b1, s_b2 : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Address chain (stages 8..13)
    --------------------------------------------------------------------------
    signal s_x8      : unsigned(10 downto 0) := (others => '0');
    signal s_t9      : signed(13 downto 0) := (others => '0');
    signal s_t10     : signed(13 downto 0) := (others => '0');
    signal s_d11     : signed(13 downto 0) := (others => '0');
    signal s_u11     : signed(13 downto 0) := (others => '0');
    signal s_odd11   : std_logic := '0';
    signal s_seam11  : std_logic := '0';
    signal s_u12     : signed(13 downto 0) := (others => '0');
    signal s_odd12   : std_logic := '0';
    signal s_seam12  : std_logic := '0';
    signal s_u13     : signed(13 downto 0) := (others => '0');
    signal s_odd13   : std_logic := '0';
    signal s_seam13  : std_logic := '0';
    signal s_addr_b, s_addr_g : unsigned(10 downto 0) := (others => '0');
    signal s_bgb, s_bgg : std_logic := '0';
    type t_x14 is array (9 to 13) of unsigned(10 downto 0);
    signal s_xd : t_x14 := (others => (others => '0'));
    type t_bo is array (14 to 17) of std_logic;
    signal s_oddd : t_bo := (others => '0');
    type t_sm is array (14 to 17) of std_logic;
    signal s_seamd : t_sm := (others => '0');
    signal s_a_bgb, s_a_bgg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v, s_wet_g : unsigned(9 downto 0) := (others => '0');
    signal s_gx15   : unsigned(9 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');
    signal s_gp16   : unsigned(17 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_g17, s_tu17, s_tv17 : signed(12 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_axis   <= unsigned(registers_in(0));
    s_k_folds  <= unsigned(registers_in(1));
    s_k_slide  <= unsigned(registers_in(2));
    s_k_glow   <= unsigned(registers_in(3));
    s_k_thr    <= unsigned(registers_in(4));
    s_k_radius <= unsigned(registers_in(5));
    s_sw_gunm  <= registers_in(6)(0);
    s_sw_neg   <= registers_in(6)(1);
    s_sw_tint  <= registers_in(6)(2);
    s_sw_seams <= registers_in(6)(3);
    s_sw_soft  <= registers_in(6)(4);
    s_p_sweep  <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage 1..8: blur cascade, delayed write port, per-frame terms
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_in  : unsigned(9 downto 0);
        variable v_out1, v_out2 : unsigned(9 downto 0);
        variable v_a : signed(13 downto 0);
        variable v_o : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw_active <= '1';
            else
                s_rx <= (others => '0');
            end if;
            s_gain  <= s_k_glow(9 downto 2);
            s_thr   <= s_k_thr;
            s_rad   <= s_k_radius(9 downto 8);
            s_folds <= s_k_folds(9 downto 8);

            -- blur cascade
            v_in := unsigned(data_in.y);
            if data_in.avid = '0' then v_in := to_unsigned(64, 10); end if;
            s_t1(0) <= v_in;
            for i in 1 to 16 loop s_t1(i) <= s_t1(i - 1); end loop;
            case to_integer(s_rad) is
                when 0 => v_out1 := s_t1(4);
                when 1 | 2 => v_out1 := s_t1(8);
                when others => v_out1 := s_t1(16);
            end case;
            if data_in.avid = '0' and s_prev_avid = '0' then
                case to_integer(s_rad) is
                    when 0 => s_acc1 <= to_unsigned(64 * 5, 15);
                    when 1 | 2 => s_acc1 <= to_unsigned(64 * 9, 15);
                    when others => s_acc1 <= to_unsigned(64 * 17, 15);
                end case;
            else
                s_acc1 <= s_acc1 + resize(v_in, 15) - resize(v_out1, 15);
            end if;
            case to_integer(s_rad) is
                when 0 => s_b1 <= resize(shift_right(s_acc1 * 13, 6), 10);
                when 1 | 2 => s_b1 <= resize(shift_right(s_acc1 * 57, 9), 10);
                when others => s_b1 <= resize(shift_right(s_acc1 * 15, 8), 10);
            end case;
            s_t2(0) <= s_b1;
            for i in 1 to 16 loop s_t2(i) <= s_t2(i - 1); end loop;
            case to_integer(s_rad) is
                when 0 => v_out2 := s_t2(4);
                when 1 | 2 => v_out2 := s_t2(8);
                when others => v_out2 := s_t2(16);
            end case;
            if s_avid_sr(1) = '0' and s_avid_sr(2) = '0' then
                case to_integer(s_rad) is
                    when 0 => s_acc2 <= to_unsigned(64 * 5, 15);
                    when 1 | 2 => s_acc2 <= to_unsigned(64 * 9, 15);
                    when others => s_acc2 <= to_unsigned(64 * 17, 15);
                end case;
            else
                s_acc2 <= s_acc2 + resize(s_b1, 15) - resize(v_out2, 15);
            end if;
            case to_integer(s_rad) is
                when 0 => s_b2 <= resize(shift_right(s_acc2 * 13, 6), 10);
                when 1 | 2 => s_b2 <= resize(shift_right(s_acc2 * 57, 9), 10);
                when others => s_b2 <= resize(shift_right(s_acc2 * 15, 8), 10);
            end case;

            s_yd(1) <= unsigned(data_in.y); s_ud(1) <= unsigned(data_in.u); s_vd(1) <= unsigned(data_in.v);
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar;
            for i in 2 to 7 loop
                s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1);
                s_wx(i) <= s_wx(i - 1); s_wp(i) <= s_wp(i - 1);
            end loop;
            s_wy8 <= s_yd(7); s_wu8 <= s_ud(7); s_wv8 <= s_vd(7);
            s_wg8 <= s_b2(9 downto 2);
            s_wx8 <= s_wx(7); s_wp8 <= s_wp(7); s_we8 <= s_avid_sr(6);

            ------------------------------------------------------------------
            -- Per-frame sequencer: axes, slide, sweep
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_axp <= s_k_axis * s_line_width;
                    s_sin_addr <= s_sweep_ph(15 downto 8);
                    s_vstep <= s_vstep + 1;
                when 1 | 2 =>
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_swp <= s_sin_q2 * signed('0' & s_p_sweep);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    v_a := signed(resize(s_axp(20 downto 10), 14)) + resize(shift_right(s_swp, 11), 14);
                    if v_a < 8 then v_a := to_signed(8, 14); end if;
                    if v_a > signed(resize(s_line_width, 14)) - 8 then v_a := signed(resize(s_line_width, 14)) - 8; end if;
                    s_a1 <= v_a;
                    s_a2 <= shift_right(v_a, 1);
                    s_a3 <= shift_right(v_a, 2);
                    -- slide accumulator (px per frame, bipolar)
                    v_o := s_off + resize(shift_right(signed(resize(s_k_slide, 11)) - to_signed(512, 11), 5), 13);
                    if v_o < 0 then v_o := v_o + signed(resize(s_line_width, 13));
                    elsif v_o >= signed(resize(s_line_width, 13)) then v_o := v_o - signed(resize(s_line_width, 13)); end if;
                    s_off <= v_o;
                    s_vstep <= "111";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar  <= not s_wpar;
                s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
                if s_saw_active = '1' then
                    s_sweep_ph <= s_sweep_ph + resize(s_p_sweep(9 downto 3), 16);
                    s_vstep <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Address chain, stages 8..13: slide, wrap, three nested folds
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_w : signed(13 downto 0);
        variable v_t : signed(13 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_s : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            v_w := signed(resize(s_line_width, 14));
            -- stage 8: column
            if s_avid_sr(6) = '0' then
                s_x8 <= (others => '0');
            else
                s_x8 <= s_x8 + 1;
            end if;
            -- stage 9: slide
            s_t9 <= signed(resize(s_x8, 14)) - C_BLUR_CENTRE + resize(s_off, 14);
            s_xd(9) <= s_x8;
            -- stage 10: wrap once
            if s_t9 >= v_w then
                s_t10 <= s_t9 - v_w;
            elsif s_t9 < 0 then
                s_t10 <= s_t9 + v_w;
            else
                s_t10 <= s_t9;
            end if;
            -- stage 11: fold 1 about a1: u = 2a - t when t > a (one subtract + mux)
            v_d := s_t10 - s_a1;
            if s_folds >= 1 and v_d > 0 then
                s_u11 <= shift_left(s_a1, 1) - s_t10;
                s_odd11 <= '1';
            else
                s_u11 <= s_t10;
                s_odd11 <= '0';
            end if;
            if s_folds >= 1 and (v_d(13 downto 5) = 0 or v_d(13 downto 5) = "111111111") then s_seam11 <= '1'; else s_seam11 <= '0'; end if;
            -- stage 12: fold 2 about a2
            v_d := s_u11 - s_a2;
            if s_folds >= 2 and v_d > 0 then
                s_u12 <= shift_left(s_a2, 1) - s_u11;
                s_odd12 <= not s_odd11;
            else
                s_u12 <= s_u11;
                s_odd12 <= s_odd11;
            end if;
            if s_folds >= 2 and (v_d(13 downto 5) = 0 or v_d(13 downto 5) = "111111111") then s_seam12 <= '1'; else s_seam12 <= s_seam11; end if;
            -- stage 13: fold 3 about a3
            v_d := s_u12 - s_a3;
            if s_folds = 3 and v_d > 0 then
                s_u13 <= shift_left(s_a3, 1) - s_u12;
                s_odd13 <= not s_odd12;
            else
                s_u13 <= s_u12;
                s_odd13 <= s_odd12;
            end if;
            if s_folds = 3 and (v_d(13 downto 5) = 0 or v_d(13 downto 5) = "111111111") then s_seam13 <= '1'; else s_seam13 <= s_seam12; end if;
            -- stage 14: addresses (glow unmirrored option reads at the slid, unfolded column)
            if s_u13 < 0 or s_u13 >= v_w then s_bgb <= '1'; else s_bgb <= '0'; end if;
            s_addr_b <= unsigned(s_u13(10 downto 0));
            if s_sw_gunm = '1' then
                v_t := signed(resize(s_xd(13), 14)) - C_BLUR_CENTRE;
                if v_t < 0 then s_bgg <= '1'; else s_bgg <= '0'; end if;
                s_addr_g <= unsigned(v_t(10 downto 0));
            else
                s_bgg <= s_bgb;
                s_addr_g <= unsigned(s_u13(10 downto 0));
            end if;
            s_oddd(14) <= s_odd13; s_seamd(14) <= s_seam13;
            for i in 15 to 17 loop
                s_oddd(i) <= s_oddd(i - 1); s_seamd(i) <= s_seamd(i - 1);
            end loop;
            for i in 10 to 13 loop
                s_xd(i) <= s_xd(i - 1);
            end loop;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 8, read stage 15)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr_b));
            if s_we8 = '1' and s_wp8 = '0' then lbY0(to_integer(s_wx8)) <= std_logic_vector(s_wy8); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr_b));
            if s_we8 = '1' and s_wp8 = '1' then lbY1(to_integer(s_wx8)) <= std_logic_vector(s_wy8); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr_b(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '0' then lbU0(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wu8(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr_b(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '1' then lbU1(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wu8(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr_b(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '0' then lbV0(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wv8(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr_b(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '1' then lbV1(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wv8(9 downto 2)); end if;
        end if;
    end process;
    p_lbG0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdG0 <= lbG0(to_integer(s_addr_g(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '0' then lbG0(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wg8); end if;
        end if;
    end process;
    p_lbG1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdG1 <= lbG1(to_integer(s_addr_g(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '1' then lbG1(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wg8); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 15..17: fetch, glow, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_g : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 15 ctx (read issued at 14 -> data 15)
            s_a_bgb <= s_bgb; s_a_bgg <= s_bgg; s_a_rbank <= s_rbank;
            -- fetch select
            if s_a_bgb = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            if s_a_bgg = '1' then
                s_wet_g <= to_unsigned(64, 10);
            elsif s_a_rbank = '1' then
                s_wet_g <= unsigned(s_rdG1) & "00";
            else
                s_wet_g <= unsigned(s_rdG0) & "00";
            end if;
            -- glow excess, base choice, negative panes
            if s_wet_g > s_thr then s_gx15 <= s_wet_g - s_thr; else s_gx15 <= (others => '0'); end if;
            if s_sw_soft = '1' then
                s_y15 <= s_wet_g;
            elsif s_sw_neg = '1' and s_oddd(15) = '1' then
                s_y15 <= not s_wet_y;
            else
                s_y15 <= s_wet_y;
            end if;
            if s_sw_neg = '1' and s_oddd(15) = '1' then
                s_u15 <= not s_wet_u; s_v15 <= not s_wet_v;
            else
                s_u15 <= s_wet_u; s_v15 <= s_wet_v;
            end if;
            -- stage 16: products
            s_gp16 <= s_gx15 * s_gain;
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
            -- stage 17: glow sum (seam adds a fixed glow), tint terms
            v_g := signed(resize(s_gp16(17 downto 6), 13));
            if s_sw_seams = '1' and s_seamd(16) = '1' then
                v_g := v_g + signed(resize(s_gain, 13)) + signed(resize(s_gain, 13));
            end if;
            s_g17 <= v_g;
            s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            if s_sw_tint = '1' then
                s_tu17 <= signed(resize(s_gp16(17 downto 10), 13));
                s_tv17 <= -signed(resize(s_gp16(17 downto 9), 13));
            else
                s_tu17 <= (others => '0');
                s_tv17 <= (others => '0');
            end if;
            -- stage 18: blend + gate
            if s_avid_sr(16) = '1' then
                s_y18 <= f_clamp10(signed(resize(s_y17, 13)) + s_g17);
                s_u18 <= f_clamp10(signed(resize(s_u17, 13)) + s_tu17);
                s_v18 <= f_clamp10(signed(resize(s_v17, 13)) + s_tv17);
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

end architecture reflector;
