-- Halation: catalogue combo #5 (seed 20260912) -- bloom / glow / soft focus
-- + magnetic warp (wobble).  Three knobs each; the switches and the slider
-- decide how they combine.
--
--   K1 Glow      bloom gain
--   K2 Threshold luma above which the blur blooms
--   K3 Radius    blur width: 9 / 17 / 33 px (two cascaded boxes)
--   K4 Wobble    per-line sine displacement amplitude, 0..+-255 px
--   K5 Rate      wobble roll speed, bipolar, centre = frozen
--   K6 Period    vertical wavelength of the wobble
--
--   S7  Warp     wobble moves both layers / the glow layer only
--   S8  Tint     the glow carries a warm chroma cast
--   S9  Energy   wobble amplitude scaled by each row's glow energy
--   S10 Blend    glow added / screened
--   S11 Soft     the base picture is replaced by its blur (soft focus)
--   P12 Halo     adds the un-warped blur of the line below on top: a
--                second, streaking glow that separates as the wobble grows
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- The blurred luma is written to a fourth (half-rate) line buffer so base
-- and glow can be fetched at different wobbled addresses.  Latency 14
-- clocks, all modes, blanking-gated.  22 EBR.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture halation of program_top is

    constant C_LAT : integer := 14;
    constant C_BLUR_CENTRE : integer := 16;   -- 33-tap kernel centre lag

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
    -- Line buffers: Y, U, V, G (blur)
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
    signal s_k_glow, s_k_thr, s_k_radius, s_k_wob, s_k_rate, s_k_period : unsigned(9 downto 0);
    signal s_sw_glowonly, s_sw_tint, s_sw_energy, s_sw_screen, s_sw_soft : std_logic;
    signal s_p_halo : unsigned(9 downto 0);
    signal s_gain   : unsigned(7 downto 0) := (others => '0');
    signal s_thr    : unsigned(9 downto 0) := (others => '0');
    signal s_rad    : unsigned(1 downto 0) := "01";
    signal s_halo   : unsigned(7 downto 0) := (others => '0');
    signal s_freq   : unsigned(11 downto 0) := (others => '0');
    signal s_rate   : signed(13 downto 0) := (others => '0');
    signal s_frame_phase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";
    signal s_d      : signed(10 downto 0) := (others => '0');
    signal s_absd   : unsigned(9 downto 0) := (others => '0');
    signal s_dsq    : unsigned(19 downto 0) := (others => '0');
    signal s_k6sq   : unsigned(19 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Input / tracking
    --------------------------------------------------------------------------
    signal s_rx      : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
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
    -- Blur: two cascaded running boxes (widths 5/9/17 -> 9/17/33 total)
    --------------------------------------------------------------------------
    type t_taps is array (0 to 16) of unsigned(9 downto 0);
    signal s_t1 : t_taps := (others => (others => '0'));
    signal s_acc1 : unsigned(14 downto 0) := (others => '0');
    signal s_b1  : unsigned(9 downto 0) := (others => '0');
    type t_taps2 is array (0 to 16) of unsigned(9 downto 0);
    signal s_t2 : t_taps2 := (others => (others => '0'));
    signal s_acc2 : unsigned(14 downto 0) := (others => '0');
    signal s_b2  : unsigned(9 downto 0) := (others => '0');
    signal s_blur7 : unsigned(9 downto 0) := (others => '0');
    signal s_esum : unsigned(20 downto 0) := (others => '0');
    signal s_erow : unsigned(9 downto 0) := to_unsigned(512, 10);

    --------------------------------------------------------------------------
    -- Per-line sequencer
    --------------------------------------------------------------------------
    signal s_lstep    : unsigned(3 downto 0) := "1111";
    signal s_line_phase : unsigned(15 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q    : signed(9 downto 0) := (others => '0');
    signal s_sin      : signed(9 downto 0) := (others => '0');
    signal s_amp      : unsigned(9 downto 0) := (others => '0');
    signal s_ampe     : unsigned(19 downto 0) := (others => '0');
    signal s_prod     : signed(20 downto 0) := (others => '0');
    signal s_off_row  : signed(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Address / pixel chain
    --------------------------------------------------------------------------
    signal s_x8      : unsigned(10 downto 0) := (others => '0');
    signal s_ub9, s_ug9 : signed(13 downto 0) := (others => '0');
    signal s_addr_b, s_addr_g : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_yhold : unsigned(9 downto 0) := to_unsigned(64, 10);   -- last active luma, pads blanking
    -- hold*N and b1*N registered ahead of the box-sum resets (the constant
    -- multiplies inside the reset mux were the HD critical path)
    signal s_yh5, s_yh9, s_yh17, s_bh5, s_bh9, s_bh17 : unsigned(14 downto 0) := (others => '0');
    signal s_wet_y, s_wet_u, s_wet_v, s_wet_g : unsigned(9 downto 0) := (others => '0');
    signal s_gx12, s_hx12 : unsigned(9 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_gp13, s_hp13 : unsigned(17 downto 0) := (others => '0');
    signal s_sp13 : unsigned(19 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    type t_bl is array (8 to 12) of unsigned(9 downto 0);
    signal s_bl : t_bl := (others => (others => '0'));

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
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

begin

    s_k_glow    <= unsigned(registers_in(0));
    s_k_thr     <= unsigned(registers_in(1));
    s_k_radius  <= unsigned(registers_in(2));
    s_k_wob     <= unsigned(registers_in(3));
    s_k_rate    <= unsigned(registers_in(4));
    s_k_period  <= unsigned(registers_in(5));
    s_sw_glowonly <= registers_in(6)(0);
    s_sw_tint     <= registers_in(6)(1);
    s_sw_energy   <= registers_in(6)(2);
    s_sw_screen   <= registers_in(6)(3);
    s_sw_soft     <= registers_in(6)(4);
    s_p_halo    <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage 1..8: tracking, blur cascade, delayed write port, sequencers
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_in  : unsigned(9 downto 0);
        variable v_out1, v_out2 : unsigned(9 downto 0);
        variable v_fp  : unsigned(15 downto 0);
        variable v_abs : signed(10 downto 0);
        variable v_mag : unsigned(13 downto 0);
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
            s_gain <= s_k_glow(9 downto 2);
            s_thr  <= s_k_thr;
            s_rad  <= s_k_radius(9 downto 8);
            s_halo <= s_p_halo(9 downto 2);

            -- stage 1: taps of box 1 (width by radius: 5/9/17)
            -- edge padding: the last active luma is held through blanking so the
            -- blur windows never average blanking black into the line ends
            if data_in.avid = '1' then s_yhold <= unsigned(data_in.y); end if;
            s_yh5 <= resize(s_yhold * 5, 15); s_yh9 <= resize(s_yhold * 9, 15); s_yh17 <= resize(s_yhold * 17, 15);
            s_bh5 <= resize(s_b1 * 5, 15);    s_bh9 <= resize(s_b1 * 9, 15);    s_bh17 <= resize(s_b1 * 17, 15);
            v_in := unsigned(data_in.y);
            if data_in.avid = '0' then
                v_in := s_yhold;
            end if;
            s_t1(0) <= v_in;
            for i in 1 to 16 loop
                s_t1(i) <= s_t1(i - 1);
            end loop;
            case to_integer(s_rad) is
                when 0 => v_out1 := s_t1(4);
                when 1 | 2 => v_out1 := s_t1(8);
                when others => v_out1 := s_t1(16);
            end case;
            if data_in.avid = '0' and s_prev_avid = '0' then
                case to_integer(s_rad) is
                    when 0 => s_acc1 <= s_yh5;
                    when 1 | 2 => s_acc1 <= s_yh9;
                    when others => s_acc1 <= s_yh17;
                end case;
            else
                s_acc1 <= s_acc1 + resize(v_in, 15) - resize(v_out1, 15);
            end if;
            -- stage 2: box 1 mean (divide by 5/9/17 ~ via shift of a scaled acc)
            case to_integer(s_rad) is
                when 0 => s_b1 <= resize(shift_right(s_acc1 * 13, 6), 10);      -- /5  ~ 13/64
                when 1 | 2 => s_b1 <= resize(shift_right(s_acc1 * 57, 9), 10);  -- /9  ~ 57/512
                when others => s_b1 <= resize(shift_right(s_acc1 * 15, 8), 10); -- /17 ~ 15/256
            end case;
            -- stage 3: taps of box 2
            s_t2(0) <= s_b1;
            for i in 1 to 16 loop
                s_t2(i) <= s_t2(i - 1);
            end loop;
            case to_integer(s_rad) is
                when 0 => v_out2 := s_t2(4);
                when 1 | 2 => v_out2 := s_t2(8);
                when others => v_out2 := s_t2(16);
            end case;
            if s_avid_sr(1) = '0' and s_avid_sr(2) = '0' then
                case to_integer(s_rad) is
                    when 0 => s_acc2 <= s_bh5;
                    when 1 | 2 => s_acc2 <= s_bh9;
                    when others => s_acc2 <= s_bh17;
                end case;
            else
                s_acc2 <= s_acc2 + resize(s_b1, 15) - resize(v_out2, 15);
            end if;
            -- stage 4: box 2 mean
            case to_integer(s_rad) is
                when 0 => s_b2 <= resize(shift_right(s_acc2 * 13, 6), 10);
                when 1 | 2 => s_b2 <= resize(shift_right(s_acc2 * 57, 9), 10);
                when others => s_b2 <= resize(shift_right(s_acc2 * 15, 8), 10);
            end case;
            -- stages 5..7: align (blur valid at 4, base data delayed to 7)
            s_blur7 <= s_b2;
            -- row energy: sum of blur over the line
            if s_avid_sr(3) = '1' then
                s_esum <= s_esum + resize(s_b2, 21);
            else
                s_esum <= (others => '0');
            end if;

            -- input data delayed 7 for the write port
            s_yd(1) <= unsigned(data_in.y); s_ud(1) <= unsigned(data_in.u); s_vd(1) <= unsigned(data_in.v);
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar;
            for i in 2 to 7 loop
                s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1);
                s_wx(i) <= s_wx(i - 1); s_wp(i) <= s_wp(i - 1);
            end loop;
            -- stage 8: write port registers (blur here is the kernel centred 16 px back)
            s_wy8 <= s_yd(7); s_wu8 <= s_ud(7); s_wv8 <= s_vd(7);
            s_wg8 <= s_blur7(9 downto 2);
            s_wx8 <= s_wx(7); s_wp8 <= s_wp(7); s_we8 <= s_avid_sr(6);

            ------------------------------------------------------------------
            -- Per-line sequencer
            ------------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>
                    s_sin_addr <= s_line_phase(15 downto 8);
                    if s_sw_energy = '1' then
                        s_ampe <= s_k_wob * s_erow;
                    else
                        s_ampe <= s_k_wob & "0000000000";
                    end if;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_amp <= s_ampe(19 downto 10);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_sin <= s_sin_q;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_prod <= s_sin * signed('0' & s_amp);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_off_row <= resize(shift_right(s_prod, 10), 10);
                    s_lstep <= s_lstep + 1;
                when others =>
                    null;
            end case;

            ------------------------------------------------------------------
            -- Per-frame sequencer: rate curve (bipolar, centre detent), period
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_d <= signed('0' & s_k_rate) - to_signed(512, 11);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    v_abs  := abs(s_d);
                    s_absd <= unsigned(v_abs(9 downto 0));
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_dsq  <= s_absd * s_absd;
                    s_k6sq <= s_k_period * s_k_period;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    v_mag := s_dsq(19 downto 6);
                    if s_absd < 8 then
                        s_rate <= (others => '0');
                    elsif s_d(10) = '1' then
                        s_rate <= -signed(v_mag);
                    else
                        s_rate <= signed(v_mag);
                    end if;
                    s_freq <= s_k6sq(19 downto 8);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    v_fp := unsigned(signed(s_frame_phase) + resize(s_rate, 16));
                    s_frame_phase <= v_fp;
                    s_line_phase  <= v_fp;
                    s_vstep <= "111";
                when others =>
                    null;
            end case;

            ------------------------------------------------------------------
            -- Line / frame events
            ------------------------------------------------------------------
            s_w_edge <= s_line_width - (C_EDGE + 1);
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                -- row energy average (approx /2048 at HD, /512 at SD via width bit)
                if s_line_width > 1024 then
                    s_erow <= s_esum(20 downto 11);
                else
                    s_erow <= s_esum(18 downto 9);
                end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar  <= not s_wpar;
                s_rbank <= s_wpar;
                s_line_phase <= s_line_phase + resize(s_freq, 16);
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_wpar <= '0';
                if s_saw_active = '1' then
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
    -- Address chain, stages 8..10
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_w : signed(13 downto 0);
        variable v_x : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 8: column
            if s_avid_sr(6) = '0' then
                s_x8 <= (others => '0');
            else
                s_x8 <= s_x8 + 1;
            end if;
            -- stage 9: base / glow addresses (both 16 px back to meet the blur centre)
            v_x := signed(resize(s_x8, 14)) - C_BLUR_CENTRE;
            if s_sw_glowonly = '1' then
                s_ub9 <= v_x;
            else
                s_ub9 <= v_x + resize(s_off_row, 14);
            end if;
            s_ug9 <= v_x + resize(s_off_row, 14);
            -- stage 10: range
            v_w := signed(resize(s_line_width, 14));
            s_addr_b <= f_edge_addr(s_ub9, s_w_edge);
            s_addr_g <= f_edge_addr(s_ug9, s_w_edge);
            -- current-line blur for the halo, aligned to stage 12
            s_bl(8) <= s_blur7;
            for i in 9 to 12 loop
                s_bl(i) <= s_bl(i - 1);
            end loop;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written at stage 8, read at stage 11)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr_b));
            if s_we8 = '1' and s_wp8 = '0' then
                lbY0(to_integer(s_wx8)) <= std_logic_vector(s_wy8);
            end if;
        end if;
    end process;

    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr_b));
            if s_we8 = '1' and s_wp8 = '1' then
                lbY1(to_integer(s_wx8)) <= std_logic_vector(s_wy8);
            end if;
        end if;
    end process;

    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr_b(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '0' then
                lbU0(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wu8(9 downto 2));
            end if;
        end if;
    end process;

    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr_b(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '1' then
                lbU1(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wu8(9 downto 2));
            end if;
        end if;
    end process;

    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr_b(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '0' then
                lbV0(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wv8(9 downto 2));
            end if;
        end if;
    end process;

    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr_b(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '1' then
                lbV1(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wv8(9 downto 2));
            end if;
        end if;
    end process;

    p_lbG0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdG0 <= lbG0(to_integer(s_addr_g(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '0' then
                lbG0(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wg8);
            end if;
        end if;
    end process;

    p_lbG1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdG1 <= lbG1(to_integer(s_addr_g(10 downto 1)));
            if s_we8 = '1' and s_wp8 = '1' then
                lbG1(to_integer(s_wx8(10 downto 1))) <= std_logic_vector(s_wg8);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 11..14: fetch, glow, blend, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_g, v_h : signed(11 downto 0);
        variable v_y  : signed(12 downto 0);
        variable v_u, v_v : signed(12 downto 0);
        variable v_gt : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 11 context
            s_a_rbank <= s_rbank;
            -- stage 12: fetch select
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            if s_a_rbank = '1' then
                s_wet_g <= unsigned(s_rdG1) & "00";
            else
                s_wet_g <= unsigned(s_rdG0) & "00";
            end if;
            -- stage 13: glow excess above threshold (fetched glow and current blur)
            if s_wet_g > s_thr then
                s_gx12 <= s_wet_g - s_thr;
            else
                s_gx12 <= (others => '0');
            end if;
            if s_bl(12) > s_thr then
                s_hx12 <= s_bl(12) - s_thr;
            else
                s_hx12 <= (others => '0');
            end if;
            if s_sw_soft = '1' then
                s_y12 <= s_wet_g;
            else
                s_y12 <= s_wet_y;
            end if;
            s_u12 <= s_wet_u; s_v12 <= s_wet_v;
            -- stage 14a (registered as 13-named): products
            s_gp13 <= s_gx12 * s_gain;
            s_hp13 <= s_hx12 * s_halo;
            s_sp13 <= s_gx12 * s_y12;           -- for screen
            s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            -- stage 14b: blend + tint + gate
            v_g := signed(resize(s_gp13(17 downto 6), 12));   -- gain Q6
            v_h := signed(resize(s_hp13(17 downto 6), 12));
            v_gt := s_gp13(17 downto 8);
            if s_sw_screen = '1' then
                v_y := signed(resize(s_y13, 13)) + resize(v_g, 13) - signed(resize(s_sp13(19 downto 10), 13)) + resize(v_h, 13);
            else
                v_y := signed(resize(s_y13, 13)) + resize(v_g, 13) + resize(v_h, 13);
            end if;
            v_u := signed(resize(s_u13, 13));
            v_v := signed(resize(s_v13, 13));
            if s_sw_tint = '1' then
                v_u := v_u + signed(resize(v_gt(9 downto 2), 13));
                v_v := v_v - signed(resize(v_gt(9 downto 1), 13));
            end if;
            if s_avid_sr(12) = '1' then
                s_y14 <= f_clamp10(v_y);
                s_u14 <= f_clamp10(v_u);
                s_v14 <= f_clamp10(v_v);
            else
                s_y14 <= to_unsigned(64, 10);
                s_u14 <= to_unsigned(512, 10);
                s_v14 <= to_unsigned(512, 10);
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

end architecture halation;
