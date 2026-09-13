-- Flutter: catalogue combo #60 (seed 20260921) -- magnetic warp (wobble)
-- + posterization (bit-depth reduction per channel).  Three knobs each;
-- the switches and the slider decide how they combine.
--
--   K1 Wobble   displacement amplitude (+-255 px)
--   K2 Waves    vertical wavelength (phase step per line)
--   K3 Speed    wobble speed, bipolar
--   K4 Luma     luma levels 1..8 bits
--   K5 Chroma   chroma levels 1..8 bits
--   K6 Spread   per-luma-band phase (or amplitude) spread
--
--   S7  Bands   bands differ in phase / in amplitude
--   S8  Order   posterize the wobbled picture / wobble the posterized picture
--   S9  Chroma  chroma wobbles with luma / a quarter wave apart (fringes)
--   S10 Edges   outside the line: edge fill / mirrored
--   S11 Dither  plain quantizer / 2x2 dithered quantizer
--   P12 Twist   horizontal phase term: 0 = pure line wobble, up = 2-D ripple
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- The input pixel's luma band picks the phase; sine ROM per pixel; line
-- buffer read a line late at x + offset (separate chroma address).
-- Latency 14 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture flutter of program_top is

    constant C_LAT : integer := 14;

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

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_amp, s_k_waves, s_k_speed, s_k_lbits, s_k_cbits, s_k_spread : unsigned(9 downto 0);
    signal s_sw_bandamp, s_sw_wobfirst, s_sw_cquarter, s_sw_mirror, s_sw_dither : std_logic;
    signal s_p_twist : unsigned(9 downto 0);

    signal s_amp    : unsigned(7 downto 0) := (others => '0');
    signal s_wstep  : unsigned(7 downto 0) := (others => '0');
    signal s_spread : unsigned(7 downto 0) := (others => '0');
    signal s_twist  : unsigned(7 downto 0) := (others => '0');
    signal s_kl, s_kc : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_lnot, s_lhalf, s_cnot, s_chalf, s_lq, s_cq : unsigned(9 downto 0) := (others => '0');
    signal s_fphase, s_lphase : unsigned(15 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_w2m1 : signed(12 downto 0) := (others => '0');

    -- write path (posterize first)
    signal s_wy2, s_wu2, s_wv2 : unsigned(9 downto 0) := (others => '0');
    signal s_wx2 : unsigned(10 downto 0) := (others => '0');
    signal s_we2, s_wpar2 : std_logic := '0';

    type t_x is array (1 to 7) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 8) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_xph : unsigned(15 downto 0) := (others => '0');
    signal s_band2 : unsigned(2 downto 0) := (others => '0');
    signal s_bs3 : unsigned(10 downto 0) := (others => '0');
    signal s_xph3 : unsigned(15 downto 0) := (others => '0');
    signal s_ph4 : unsigned(7 downto 0) := (others => '0');
    signal s_ba4 : unsigned(8 downto 0) := (others => '0');
    signal s_sa5, s_ca5 : unsigned(7 downto 0) := (others => '0');
    signal s_ampe5 : unsigned(16 downto 0) := (others => '0');
    signal s_ampe6, s_ampe7 : unsigned(9 downto 0) := (others => '0');
    signal s_sin6, s_cos6 : signed(9 downto 0) := (others => '0');
    signal s_sin7, s_cos7 : signed(9 downto 0) := (others => '0');
    signal s_ps8, s_pc8 : signed(20 downto 0) := (others => '0');
    signal s_ay9, s_ac9 : signed(12 downto 0) := (others => '0');
    signal s_addrY, s_addrC : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_dz : unsigned(1 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');

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

    function f_quant(y : unsigned(9 downto 0); d : unsigned(9 downto 0); nm : unsigned(9 downto 0); h : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(10 downto 0);
    begin
        v := resize(y, 11) + resize(d, 11);
        if v > 1023 then v := to_unsigned(1023, 11); end if;
        return (v(9 downto 0) and nm) + h;
    end function;

begin

    s_k_amp    <= unsigned(registers_in(0));
    s_k_waves  <= unsigned(registers_in(1));
    s_k_speed  <= unsigned(registers_in(2));
    s_k_lbits  <= unsigned(registers_in(3));
    s_k_cbits  <= unsigned(registers_in(4));
    s_k_spread <= unsigned(registers_in(5));
    s_sw_bandamp  <= registers_in(6)(0);
    s_sw_wobfirst <= registers_in(6)(1);
    s_sw_cquarter <= registers_in(6)(2);
    s_sw_mirror   <= registers_in(6)(3);
    s_sw_dither   <= registers_in(6)(4);
    s_p_twist <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_m : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            s_amp <= s_k_amp(9 downto 2);
            s_wstep <= s_k_waves(9 downto 2);
            s_spread <= s_k_spread(9 downto 2);
            s_twist <= s_p_twist(9 downto 2);
            s_kl <= to_unsigned(9, 4) - resize(s_k_lbits(9 downto 7), 4);
            s_kc <= to_unsigned(9, 4) - resize(s_k_cbits(9 downto 7), 4);
            v_m := shift_left(to_unsigned(1, 10), to_integer(s_kl)) - 1;
            s_lnot <= not v_m; s_lhalf <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 1);
            s_lq <= shift_left(to_unsigned(1, 10), to_integer(s_kl) - 2);     -- quarter step (dither)
            v_m := shift_left(to_unsigned(1, 10), to_integer(s_kc)) - 1;
            s_cnot <= not v_m; s_chalf <= shift_left(to_unsigned(1, 10), to_integer(s_kc) - 1);
            s_cq <= shift_left(to_unsigned(1, 10), to_integer(s_kc) - 2);
            s_w2m1 <= signed(resize(s_line_width & '0', 13)) - 1;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we13  <= signed(resize(s_w_edge, 13));                  -- w_edge
            s_wl13  <= signed(resize(s_line_width, 13));              -- width
            s_nwe13 <= -signed(resize(s_w_edge, 13));                 -- mirrored-left: n > w_edge  <=> src < -w_edge
            s_rce13 <= s_w2m1 - C_EDGE;                               -- mirrored-right: r < C_EDGE <=> src > 2w-1-C_EDGE
            s_rwe13 <= signed(resize(s_line_width, 13)) + C_EDGE;     -- mirrored-right: r > w_edge <=> src < w+C_EDGE
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                s_lphase <= s_lphase + resize(s_wstep & "00", 16);
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then
                    s_fphase <= s_fphase + unsigned(resize(shift_right(signed(resize(s_k_speed, 11)) - 512, 1), 16));
                end if;
                s_lphase <= s_fphase;
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

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin6 <= to_signed(C_SINE(to_integer(s_sa5)), 10);
            s_cos6 <= to_signed(C_SINE(to_integer(s_ca5)), 10);
            s_sin7 <= s_sin6; s_cos7 <= s_cos6;
        end if;
    end process;

    p_addr : process(clk)
        variable v_d, v_dc : unsigned(9 downto 0);
        variable v_neg, v_ref : signed(12 downto 0);
        procedure edge(signal src : in signed(12 downto 0); signal addr : out unsigned(10 downto 0)) is
            variable n, r : signed(12 downto 0);
        begin
            -- every compare is on src against a registered bound, in parallel
            -- with the negate / subtract; the mux is the only serial step
            n := -src; r := s_w2m1 - src;
            if s_sw_mirror = '1' and src < 0 then
                if src > -C_EDGE then addr <= to_unsigned(C_EDGE, 11);
                elsif src < s_nwe13 then addr <= s_w_edge;
                else addr <= unsigned(n(10 downto 0)); end if;
            elsif s_sw_mirror = '1' and src >= s_wl13 then
                if src > s_rce13 then addr <= to_unsigned(C_EDGE, 11);
                elsif src < s_rwe13 then addr <= s_w_edge;
                else addr <= unsigned(r(10 downto 0)); end if;
            elsif src < C_EDGE then addr <= to_unsigned(C_EDGE, 11);
            elsif src > s_we13 then addr <= s_w_edge;
            else addr <= unsigned(src(10 downto 0)); end if;
        end procedure;
    begin
        if rising_edge(clk) then
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 7 loop s_x(i) <= s_x(i - 1); end loop;
            for i in 2 to 8 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- write path: quantize the input when "wobble the posterized picture"
            s_dz <= s_wr_x(0) & s_line(0);
            if s_sw_dither = '1' then
                case to_integer(s_dz) is
                    when 0 => v_d := (others => '0'); v_dc := (others => '0');
                    when 1 => v_d := s_lhalf; v_dc := s_chalf;
                    when 2 => v_d := s_lhalf + s_lq; v_dc := s_chalf + s_cq;
                    when others => v_d := s_lq; v_dc := s_cq;
                end case;
            else
                v_d := (others => '0'); v_dc := (others => '0');
            end if;
            if s_sw_wobfirst = '0' then
                s_wy2 <= f_quant(s_in_y, v_d, s_lnot, s_lhalf);
                s_wu2 <= f_quant(s_in_u, v_dc, s_cnot, s_chalf);
                s_wv2 <= f_quant(s_in_v, v_dc, s_cnot, s_chalf);
            else
                s_wy2 <= s_in_y; s_wu2 <= s_in_u; s_wv2 <= s_in_v;
            end if;
            s_wx2 <= s_wr_x; s_we2 <= s_in_avid; s_wpar2 <= s_wpar;
            -- 2: horizontal phase DDA, band from the input luma
            if s_in_avid = '0' then s_xph <= s_lphase; else s_xph <= s_xph + resize(s_twist, 16); end if;
            s_band2 <= s_in_y(9 downto 7);
            -- 3
            s_bs3 <= s_band2 * s_spread;
            s_xph3 <= s_xph;
            -- 4: phase and band amplitude
            if s_sw_bandamp = '1' then
                s_ph4 <= s_xph3(15 downto 8);
                s_ba4 <= resize(s_bs3(10 downto 3), 9) + 32;        -- 32..287 (Q8 gain)
            else
                s_ph4 <= s_xph3(15 downto 8) + s_bs3(10 downto 3);
                s_ba4 <= to_unsigned(256, 9);
            end if;
            -- 5: ROM addresses
            s_sa5 <= s_ph4;
            if s_sw_cquarter = '1' then s_ca5 <= s_ph4 + 64; else s_ca5 <= s_ph4; end if;
            s_ampe5 <= s_amp * s_ba4;
            -- 6, 7
            if s_ampe5(16 downto 8) > 511 then s_ampe6 <= to_unsigned(511, 10); else s_ampe6 <= resize(s_ampe5(16 downto 8), 10); end if;
            s_ampe7 <= s_ampe6;
            -- 8: products (ROM out 6, fabric 7)
            s_ps8 <= s_sin7 * signed(resize(s_ampe7, 11));
            s_pc8 <= s_cos7 * signed(resize(s_ampe7, 11));
            -- 9: addresses
            s_ay9 <= signed(resize(s_x(7), 13)) + resize(shift_right(s_ps8, 8), 13);
            s_ac9 <= signed(resize(s_x(7), 13)) + resize(shift_right(s_pc8, 8), 13);
            -- 10: edges
            edge(s_ay9, s_addrY);
            edge(s_ac9, s_addrC);
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addrY));
            if s_we2 = '1' and s_wpar2 = '0' then lbY0(to_integer(s_wx2)) <= std_logic_vector(s_wy2); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addrY));
            if s_we2 = '1' and s_wpar2 = '1' then lbY1(to_integer(s_wx2)) <= std_logic_vector(s_wy2); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wpar2 = '0' then lbU0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wu2(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wpar2 = '1' then lbU1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wu2(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wpar2 = '0' then lbV0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wv2(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wpar2 = '1' then lbV1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_wv2(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_d, v_dc : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 11 (rd) / 12: fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then s_wet_y <= unsigned(s_rdY1); else s_wet_y <= unsigned(s_rdY0); end if;
            if s_a_rbank = '1' then s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00"; end if;
            -- 13: posterize after the wobble
            if s_sw_dither = '1' then
                case to_integer(s_dz) is
                    when 0 => v_d := (others => '0'); v_dc := (others => '0');
                    when 1 => v_d := s_lhalf; v_dc := s_chalf;
                    when 2 => v_d := s_lhalf + s_lq; v_dc := s_chalf + s_cq;
                    when others => v_d := s_lq; v_dc := s_cq;
                end case;
            else
                v_d := (others => '0'); v_dc := (others => '0');
            end if;
            if s_sw_wobfirst = '1' then
                s_y13 <= f_quant(s_wet_y, v_d, s_lnot, s_lhalf);
                s_u13 <= f_quant(s_wet_u, v_dc, s_cnot, s_chalf);
                s_v13 <= f_quant(s_wet_v, v_dc, s_cnot, s_chalf);
            else
                s_y13 <= s_wet_y; s_u13 <= s_wet_u; s_v13 <= s_wet_v;
            end if;
            -- 14: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y14);
    data_out.u       <= std_logic_vector(s_u14);
    data_out.v       <= std_logic_vector(s_v14);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture flutter;
