-- Hueflutter: catalogue combo #111 (seed 20260925) -- hue rotation /
-- saturation + magnetic warp (wobble) + ordered dithering.  Two knobs each;
-- the switches and the slider decide how they combine.
--
--   K1 Wobble   displacement amplitude (+-255 px)
--   K2 Waves    vertical wavelength (phase step per line)
--   K3 Hue      base hue rotation 0..360 degrees
--   K4 Sat      saturation 0..400% (100% = unity)
--   K5 Luma     dither levels 1..8 bits
--   K6 Chroma   chroma levels 1..8 bits
--
--   S7  Bands   luma bands wobble with different phases / amplitudes
--   S8  Order   dither the wobbled picture / wobble the dithered picture
--   S9  Chroma  chroma wobbles with luma / a quarter wave apart
--   S10 Hue     hue follows the wobble phase (rainbow ripples) / static
--   S11 Dither  plain quantizer / 2x2 dithered quantizer
--   P12 Spin    hue rotation speed, bipolar (centre = still)
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- The input pixel's luma band picks the wobble phase; sine ROM per pixel;
-- the fetched chroma is hue-rotated (sine ROM) and scaled, then dithered.
-- Latency 24 clocks, all modes, blanking-gated.  v0.1.2: the output stage
-- (gain recombine + recentre + clamp + quantize) split into three, and the
-- hue-rotation products split hi/lo on the sine operand with keep-duplicated
-- operand registers -- those were the HD critical paths at ~65 MHz.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture hueflutter of program_top is

    constant C_LAT : integer := 24;

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

    signal s_k_amp, s_k_waves, s_k_hue, s_k_sat, s_k_lbits, s_k_cbits : unsigned(9 downto 0);
    signal s_sw_bandamp, s_sw_wobfirst, s_sw_cquarter, s_sw_huewob, s_sw_dither : std_logic;
    signal s_p_spin : unsigned(9 downto 0);
    signal s_k_speed, s_k_spread, s_p_twist : unsigned(9 downto 0) := (others => '0');
    signal s_sw_mirror : std_logic := '0';
    signal s_hue : unsigned(7 downto 0) := (others => '0');
    signal s_gain, s_gain_a : unsigned(8 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_gain_a : signal is true;
    signal s_hphase : unsigned(15 downto 0) := (others => '0');
    type t_ph is array (5 to 14) of unsigned(7 downto 0);
    signal s_phd : t_ph := (others => (others => '0'));
    signal s_th15 : unsigned(7 downto 0) := (others => '0');
    signal s_ha16, s_hc16 : unsigned(7 downto 0) := (others => '0');
    signal s_hs17, s_hcs17 : signed(9 downto 0) := (others => '0');
    -- stage-18 operand copies, one per multiplier (keep: yosys would re-merge them)
    signal s_hs18u, s_hs18v, s_hcs18u, s_hcs18v : signed(9 downto 0) := (others => '0');
    signal s_ua18, s_ub18, s_va18, s_vb18 : signed(10 downto 0) := (others => '0');
    attribute keep of s_hs18u, s_hs18v, s_hcs18u, s_hcs18v, s_ua18, s_ub18, s_va18, s_vb18 : signal is true;
    signal s_uc15, s_vc15 : signed(10 downto 0) := (others => '0');
    type t_u11 is array (16 to 18) of signed(10 downto 0);
    signal s_ucd, s_vcd : t_u11 := (others => (others => '0'));
    type t_y10 is array (15 to 25) of unsigned(9 downto 0);
    signal s_yr : t_y10 := (others => (others => '0'));
    signal s_uc19h, s_us19h, s_vc19h, s_vs19h : signed(15 downto 0) := (others => '0');  -- chroma * sine(9..5)
    signal s_uc19l, s_us19l, s_vc19l, s_vs19l : signed(16 downto 0) := (others => '0');  -- chroma * sine(4..0)
    signal s_uc20, s_us20, s_vc20, s_vs20 : signed(20 downto 0) := (others => '0');
    signal s_ur20, s_vr20 : signed(21 downto 0) := (others => '0');
    signal s_u21, s_v21 : signed(10 downto 0) := (others => '0');
    signal s_ugh22, s_vgh22 : signed(15 downto 0) := (others => '0');
    signal s_ugl22, s_vgl22 : signed(16 downto 0) := (others => '0');
    signal s_ug23, s_vg23 : signed(12 downto 0) := (others => '0');
    signal s_uq24, s_vq24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_d24, s_dc24 : unsigned(9 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := (others => '0');

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
    signal s_ampe6, s_ampe7a, s_ampe7b : unsigned(9 downto 0) := (others => '0');   -- one copy per product
    attribute keep of s_ampe7a, s_ampe7b : signal is true;
    signal s_sin6, s_cos6 : signed(9 downto 0) := (others => '0');
    signal s_sin7, s_cos7 : signed(9 downto 0) := (others => '0');
    signal s_ps8h, s_pc8h, s_ps8l, s_pc8l : signed(15 downto 0) := (others => '0');   -- sine * amp hi / lo halves
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

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else return unsigned(v(9 downto 0)); end if;
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
    s_k_hue    <= unsigned(registers_in(2));
    s_k_sat    <= unsigned(registers_in(3));
    s_k_lbits  <= unsigned(registers_in(4));
    s_k_cbits  <= unsigned(registers_in(5));
    s_sw_bandamp  <= registers_in(6)(0);
    s_sw_wobfirst <= registers_in(6)(1);
    s_sw_cquarter <= registers_in(6)(2);
    s_sw_huewob   <= registers_in(6)(3);
    s_sw_dither   <= registers_in(6)(4);
    s_p_spin <= unsigned(registers_in(7));

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
            s_spread <= x"40";
            s_twist <= (others => '0');
            s_hue <= s_k_hue(9 downto 2); s_gain <= s_k_sat(9 downto 1); s_gain_a <= s_gain;
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
                    s_fphase <= s_fphase + 400;
                    s_hphase <= s_hphase + unsigned(resize(shift_right(signed(resize(s_p_spin, 11)) - 512, 1), 16));
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
            s_hs17 <= to_signed(C_SINE(to_integer(s_ha16)), 10);
            s_hcs17 <= to_signed(C_SINE(to_integer(s_hc16)), 10);
            s_hs18u <= s_hs17; s_hs18v <= s_hs17; s_hcs18u <= s_hcs17; s_hcs18v <= s_hcs17;
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
            s_phd(5) <= s_ph4;
            for i in 6 to 14 loop s_phd(i) <= s_phd(i - 1); end loop;
            if s_sw_cquarter = '1' then s_ca5 <= s_ph4 + 64; else s_ca5 <= s_ph4; end if;
            s_ampe5 <= s_amp * s_ba4;
            -- 6, 7
            if s_ampe5(16 downto 8) > 511 then s_ampe6 <= to_unsigned(511, 10); else s_ampe6 <= resize(s_ampe5(16 downto 8), 10); end if;
            s_ampe7a <= s_ampe6; s_ampe7b <= s_ampe6;
            -- 8: products (ROM out 6, fabric 7)
            s_ps8h <= s_sin7 * signed('0' & s_ampe7a(9 downto 5)); s_ps8l <= s_sin7 * signed('0' & s_ampe7a(4 downto 0));
            s_pc8h <= s_cos7 * signed('0' & s_ampe7b(9 downto 5)); s_pc8l <= s_cos7 * signed('0' & s_ampe7b(4 downto 0));
            -- 9: addresses (half products recombined here: two short adds, no multiply)
            s_ay9 <= signed(resize(s_x(7), 13)) + resize(shift_right(shift_left(resize(s_ps8h, 21), 5) + resize(s_ps8l, 21), 8), 13);
            s_ac9 <= signed(resize(s_x(7), 13)) + resize(shift_right(shift_left(resize(s_pc8h, 21), 5) + resize(s_pc8l, 21), 8), 13);
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
    begin
        if rising_edge(clk) then
            -- 11 (rd) / 12: fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then s_wet_y <= unsigned(s_rdY1); else s_wet_y <= unsigned(s_rdY0); end if;
            if s_a_rbank = '1' then s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00"; end if;
            -- 15: centred chroma, hue angle (follows the wobble phase when asked)
            s_uc15 <= signed(resize(s_wet_u, 11)) - 512;
            s_vc15 <= signed(resize(s_wet_v, 11)) - 512;
            if s_sw_huewob = '1' then s_th15 <= s_hue + s_hphase(15 downto 8) + s_phd(12); else s_th15 <= s_hue + s_hphase(15 downto 8); end if;
            s_yr(15) <= s_wet_y;
            -- 16: ROM addresses (rom 17, fabric 18)
            s_ha16 <= s_th15; s_hc16 <= s_th15 + 64;
            s_ucd(16) <= s_uc15; s_vcd(16) <= s_vc15;
            s_ucd(17) <= s_ucd(16); s_vcd(17) <= s_vcd(16);
            s_ua18 <= s_ucd(17); s_ub18 <= s_ucd(17); s_va18 <= s_vcd(17); s_vb18 <= s_vcd(17);
            for i in 16 to 25 loop s_yr(i) <= s_yr(i - 1); end loop;
            -- 19: half products (sine operand split hi/lo)
            s_uc19h <= s_ua18 * s_hcs18u(9 downto 5); s_uc19l <= s_ua18 * signed('0' & s_hcs18u(4 downto 0));
            s_us19h <= s_ub18 * s_hs18u(9 downto 5);  s_us19l <= s_ub18 * signed('0' & s_hs18u(4 downto 0));
            s_vc19h <= s_va18 * s_hcs18v(9 downto 5); s_vc19l <= s_va18 * signed('0' & s_hcs18v(4 downto 0));
            s_vs19h <= s_vb18 * s_hs18v(9 downto 5);  s_vs19l <= s_vb18 * signed('0' & s_hs18v(4 downto 0));
            -- 20: recombine
            s_uc20 <= shift_left(resize(s_uc19h, 21), 5) + resize(s_uc19l, 21);
            s_us20 <= shift_left(resize(s_us19h, 21), 5) + resize(s_us19l, 21);
            s_vc20 <= shift_left(resize(s_vc19h, 21), 5) + resize(s_vc19l, 21);
            s_vs20 <= shift_left(resize(s_vs19h, 21), 5) + resize(s_vs19l, 21);
            -- 21: rotate
            s_ur20 <= resize(s_uc20, 22) - resize(s_vs20, 22);
            s_vr20 <= resize(s_us20, 22) + resize(s_vc20, 22);
            -- 22: clamp
            if shift_right(s_ur20, 8) > 511 then s_u21 <= to_signed(511, 11); elsif shift_right(s_ur20, 8) < -511 then s_u21 <= to_signed(-511, 11); else s_u21 <= resize(shift_right(s_ur20, 8), 11); end if;
            if shift_right(s_vr20, 8) > 511 then s_v21 <= to_signed(511, 11); elsif shift_right(s_vr20, 8) < -511 then s_v21 <= to_signed(-511, 11); else s_v21 <= resize(shift_right(s_vr20, 8), 11); end if;
            -- 23: gain split
            s_ugh22 <= s_u21 * signed(resize(s_gain_a(8 downto 5), 5));
            s_ugl22 <= s_u21 * signed(resize(s_gain_a(4 downto 0), 6));
            s_vgh22 <= s_v21 * signed(resize(s_gain_a(8 downto 5), 5));
            s_vgl22 <= s_v21 * signed(resize(s_gain_a(4 downto 0), 6));
            -- 24: recombine the gain halves (Q7 -> centred 13-bit chroma)
            s_ug23 <= resize(shift_right(shift_left(resize(s_ugh22, 22), 5) + resize(s_ugl22, 22), 7), 13);
            s_vg23 <= resize(shift_right(shift_left(resize(s_vgh22, 22), 5) + resize(s_vgl22, 22), 7), 13);
            -- 25: recentre + clamp; dither pick registered
            s_uq24 <= f_clamp10(s_ug23 + 512);
            s_vq24 <= f_clamp10(s_vg23 + 512);
            if s_sw_dither = '1' then
                case to_integer(s_dz) is
                    when 0 => s_d24 <= (others => '0'); s_dc24 <= (others => '0');
                    when 1 => s_d24 <= s_lhalf; s_dc24 <= s_chalf;
                    when 2 => s_d24 <= s_lhalf + s_lq; s_dc24 <= s_chalf + s_cq;
                    when others => s_d24 <= s_lq; s_dc24 <= s_cq;
                end case;
            else
                s_d24 <= (others => '0'); s_dc24 <= (others => '0');
            end if;
            -- 26: posterize + gate
            if s_avid_sr(C_LAT - 2) = '1' then
                if s_sw_wobfirst = '1' then
                    s_y23 <= f_quant(s_yr(25), s_d24, s_lnot, s_lhalf);
                    s_u23 <= f_quant(s_uq24, s_dc24, s_cnot, s_chalf);
                    s_v23 <= f_quant(s_vq24, s_dc24, s_cnot, s_chalf);
                else
                    s_y23 <= s_yr(25); s_u23 <= s_uq24; s_v23 <= s_vq24;
                end if;
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

end architecture hueflutter;
