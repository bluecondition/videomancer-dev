-- Tremor: catalogue combo #17 (seed 20260915) -- Perlin-style displacement
-- + scanline shear + duotone.  Two knobs each; the switches and the slider
-- decide how the three combine.
--
--   K1 Scale    noise cell size 4 .. 512 px
--   K2 Drift    noise field scrolls vertically (0 = still)
--   K3 Skew     scanline shear: rows lean by (row in period - P/2) * skew
--   K4 Period   shear sawtooth period 2 .. 256 lines
--   K5 Hue      highlight tone hue
--   K6 Spread   angle between shadow and highlight hues (0 = one tint)
--
--   S7  Noise   warps the picture horizontally / grains the luma instead
--   S8  Shear   sawtooth / triangle
--   S9  Bands   every other shear band swaps the two tones
--   S10 Field   smooth value noise / hard cells
--   S11 Tone    duotone keyed from luma / from the noise field itself
--   P12 Amount  displacement (or grain) amplitude, up to +-255 px
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- 2-D value noise: four 16-bit add/xor-shift hashes per pixel on the cell
-- lattice, bilinear lerp with Q7 fractions; per-line shear from an
-- hblank sequencer; duotone chroma vectors from a sine ROM per frame.
-- Latency 28 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture tremor of program_top is

    constant C_LAT  : integer := 28;
    constant C_SEED : unsigned(15 downto 0) := x"5A3C";

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return unsigned(to_unsigned(1023, 10));
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    function f_round1(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 5);
        return t xor shift_right(t, 6);
    end function;

    function f_round2(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 3);
        return t xor shift_right(t, 8);
    end function;

    function f_round3(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 7);
        t := t xor shift_right(t, 5);
        return t xor shift_left(t, 8);
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

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    -- controls / per-frame
    signal s_k_scale, s_k_drift, s_k_skew, s_k_period, s_k_hue, s_k_spread : unsigned(9 downto 0);
    signal s_sw_grain, s_sw_tri, s_sw_flip, s_sw_blocky, s_sw_tonenoise : std_logic;
    signal s_p_amount : unsigned(9 downto 0);
    signal s_k      : unsigned(3 downto 0) := to_unsigned(5, 4);      -- cell shift 2..9
    signal s_mask   : unsigned(10 downto 0) := (others => '0');
    signal s_skew   : unsigned(7 downto 0) := (others => '0');
    signal s_period : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal s_amp    : unsigned(7 downto 0) := (others => '0');
    signal s_nphase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep  : unsigned(4 downto 0) := "11111";
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2 : signed(9 downto 0) := (others => '0');
    signal s_sB, s_cB, s_sA, s_cA : signed(9 downto 0) := (others => '0');
    signal s_uA, s_vA, s_uB, s_vB : signed(11 downto 0) := to_signed(512, 12);
    signal s_du, s_dv : signed(11 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_du : signal is true;
    attribute keep of s_dv : signal is true;

    -- tracking / per-line
    signal s_rx, s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_lp     : unsigned(8 downto 0) := (others => '0');
    signal s_dir, s_band : std_logic := '0';
    signal s_lq     : unsigned(8 downto 0) := (others => '0');
    signal s_lt     : signed(9 downto 0) := (others => '0');
    signal s_shp    : signed(18 downto 0) := (others => '0');
    signal s_sh_off : signed(10 downto 0) := (others => '0');
    signal s_ly     : unsigned(11 downto 0) := (others => '0');
    signal s_cy     : unsigned(9 downto 0) := (others => '0');
    signal s_fy     : unsigned(6 downto 0) := (others => '0');
    signal s_rk0, s_rk0p, s_rk1, s_rk1p : unsigned(15 downto 0) := (others => '0');
    signal s_band_row : std_logic := '0';

    -- noise pipeline
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_cx1    : unsigned(9 downto 0) := (others => '0');
    signal s_fx1    : unsigned(6 downto 0) := (others => '0');
    signal s_ka, s_kb, s_kc, s_kd : unsigned(15 downto 0) := (others => '0');
    signal s_ha3, s_hb3, s_hc3, s_hd3 : unsigned(15 downto 0) := (others => '0');
    signal s_ha4, s_hb4, s_hc4, s_hd4 : unsigned(15 downto 0) := (others => '0');
    signal s_ha5, s_hb5, s_hc5, s_hd5 : unsigned(15 downto 0) := (others => '0');
    signal s_a6, s_b6, s_c6, s_d6 : unsigned(7 downto 0) := (others => '0');
    signal s_a7, s_c7 : unsigned(7 downto 0) := (others => '0');
    signal s_dab7, s_dcd7 : signed(8 downto 0) := (others => '0');
    signal s_a8, s_c8 : unsigned(7 downto 0) := (others => '0');
    signal s_pab8, s_pcd8 : signed(16 downto 0) := (others => '0');
    signal s_top9, s_bot9 : unsigned(7 downto 0) := (others => '0');
    signal s_top10  : unsigned(7 downto 0) := (others => '0');
    signal s_dtb10  : signed(8 downto 0) := (others => '0');
    signal s_top11  : unsigned(7 downto 0) := (others => '0');
    signal s_ptb11  : signed(16 downto 0) := (others => '0');
    signal s_n12    : unsigned(7 downto 0) := (others => '0');
    signal s_nc13   : signed(8 downto 0) := (others => '0');
    signal s_d14    : signed(17 downto 0) := (others => '0');
    signal s_off15  : signed(9 downto 0) := (others => '0');
    signal s_gr15   : signed(7 downto 0) := (others => '0');
    type t_fx is array (2 to 7) of unsigned(6 downto 0);
    signal s_fxd    : t_fx := (others => (others => '0'));
    type t_xs is array (2 to 15) of signed(12 downto 0);
    signal s_xs     : t_xs := (others => (others => '0'));
    type t_n8 is array (13 to 23) of unsigned(7 downto 0);
    signal s_nd     : t_n8 := (others => (others => '0'));
    type t_g8 is array (16 to 19) of signed(7 downto 0);
    signal s_gd     : t_g8 := (others => (others => '0'));

    -- address / fetch / duotone
    signal s_u16    : signed(12 downto 0) := (others => '0');
    signal s_addr17 : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_yc20   : unsigned(9 downto 0) := (others => '0');
    signal s_yo20   : unsigned(9 downto 0) := (others => '0');
    signal s_ys21   : unsigned(11 downto 0) := (others => '0');
    signal s_yo21   : unsigned(9 downto 0) := (others => '0');
    signal s_y8_22  : unsigned(7 downto 0) := (others => '0');
    signal s_yo22   : unsigned(9 downto 0) := (others => '0');
    signal s_y8g23  : unsigned(7 downto 0) := (others => '0');
    signal s_yo23   : unsigned(9 downto 0) := (others => '0');
    signal s_t24    : unsigned(7 downto 0) := (others => '0');
    signal s_yo24   : unsigned(9 downto 0) := (others => '0');
    signal s_pu25, s_pv25 : signed(20 downto 0) := (others => '0');
    signal s_yo25   : unsigned(9 downto 0) := (others => '0');
    signal s_y26, s_u26, s_v26 : unsigned(9 downto 0) := (others => '0');
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := (others => '0');
    signal s_y28, s_u28, s_v28 : unsigned(9 downto 0) := (others => '0');
    signal s_bandd  : std_logic_vector(2 to 23) := (others => '0');

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

    s_k_scale  <= unsigned(registers_in(0));
    s_k_drift  <= unsigned(registers_in(1));
    s_k_skew   <= unsigned(registers_in(2));
    s_k_period <= unsigned(registers_in(3));
    s_k_hue    <= unsigned(registers_in(4));
    s_k_spread <= unsigned(registers_in(5));
    s_sw_grain     <= registers_in(6)(0);
    s_sw_tri       <= registers_in(6)(1);
    s_sw_flip      <= registers_in(6)(2);
    s_sw_blocky    <= registers_in(6)(3);
    s_sw_tonenoise <= registers_in(6)(4);
    s_p_amount <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q  <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control: tracking, per-line and per-frame sequencers
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : integer range 2 to 9;
        variable v_t : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            -- per-line sequencer
            v_k := to_integer(s_k);
            case to_integer(s_lstep) is
                when 0 =>
                    if s_sw_tri = '1' and s_dir = '1' then
                        s_lq <= s_period - 1 - s_lp;
                    else
                        s_lq <= s_lp;
                    end if;
                    s_ly <= resize(s_line, 12) + resize(s_nphase(15 downto 4), 12);
                    s_band_row <= s_band;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_lt <= signed('0' & s_lq) - signed('0' & s_period(8 downto 1));
                    s_cy <= resize(shift_right(s_ly, v_k), 10);
                    case v_k is
                        when 2 => s_fy <= s_ly(1 downto 0) & "00000";
                        when 3 => s_fy <= s_ly(2 downto 0) & "0000";
                        when 4 => s_fy <= s_ly(3 downto 0) & "000";
                        when 5 => s_fy <= s_ly(4 downto 0) & "00";
                        when 6 => s_fy <= s_ly(5 downto 0) & "0";
                        when 7 => s_fy <= s_ly(6 downto 0);
                        when 8 => s_fy <= s_ly(7 downto 1);
                        when others => s_fy <= s_ly(8 downto 2);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_shp <= s_lt * signed('0' & s_skew);
                    s_rk0 <= C_SEED + shift_left(resize(s_cy, 16), 7);
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    v_t := resize(shift_right(s_shp, 5), 13);
                    if v_t > 511 then v_t := to_signed(511, 13); elsif v_t < -511 then v_t := to_signed(-511, 13); end if;
                    s_sh_off <= resize(v_t, 11);
                    s_rk0p <= s_rk0 + 1;
                    s_rk1  <= s_rk0 + 128;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_rk1p <= s_rk1 + 1;
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            -- per-frame sequencer: duotone vectors, decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_sin_addr <= s_k_hue(9 downto 2);
                    s_amp   <= s_p_amount(9 downto 2);
                    s_skew  <= s_k_skew(9 downto 2);
                    s_period <= resize(s_k_period(9 downto 2), 9) + 2;
                    if s_k_scale(9 downto 7) = 7 then s_k <= to_unsigned(9, 4); else s_k <= resize(s_k_scale(9 downto 7), 4) + 2; end if;
                    s_nphase <= s_nphase + resize(s_k_drift(9 downto 3), 16);
                    s_vstep <= s_vstep + 1;
                when 1 | 2 => s_vstep <= s_vstep + 1;
                when 3 =>
                    s_sB <= s_sin_q2;
                    s_sin_addr <= s_k_hue(9 downto 2) + 64;
                    for i in 0 to 10 loop
                        if i < to_integer(s_k) then s_mask(i) <= '1'; else s_mask(i) <= '0'; end if;
                    end loop;
                    s_vstep <= s_vstep + 1;
                when 4 | 5 => s_vstep <= s_vstep + 1;
                when 6 =>
                    s_cB <= s_sin_q2;
                    s_sin_addr <= s_k_hue(9 downto 2) + s_k_spread(9 downto 3);
                    s_vstep <= s_vstep + 1;
                when 7 | 8 => s_vstep <= s_vstep + 1;
                when 9 =>
                    s_sA <= s_sin_q2;
                    s_sin_addr <= s_k_hue(9 downto 2) + s_k_spread(9 downto 3) + 64;
                    s_vstep <= s_vstep + 1;
                when 10 | 11 => s_vstep <= s_vstep + 1;
                when 12 =>
                    s_cA <= s_sin_q2;
                    s_vstep <= s_vstep + 1;
                when 13 =>
                    -- chroma vectors: 512 + 1.5 * sin (+-382)
                    s_uB <= to_signed(512, 12) + resize(s_sB, 12) + resize(shift_right(s_sB, 1), 12);
                    s_vB <= to_signed(512, 12) + resize(s_cB, 12) + resize(shift_right(s_cB, 1), 12);
                    s_uA <= to_signed(512, 12) + resize(s_sA, 12) + resize(shift_right(s_sA, 1), 12);
                    s_vA <= to_signed(512, 12) + resize(s_cA, 12) + resize(shift_right(s_cA, 1), 12);
                    s_vstep <= s_vstep + 1;
                when 14 =>
                    s_du <= s_uB - s_uA;
                    s_dv <= s_vB - s_vA;
                    s_vstep <= "11111";
                when others => null;
            end case;

            -- line / frame events
            s_w_edge <= s_line_width - (C_EDGE + 1);
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_line <= s_line + 1;
                if s_lp >= s_period - 1 then
                    s_lp <= (others => '0');
                    s_dir <= not s_dir;
                    s_band <= not s_band;
                else
                    s_lp <= s_lp + 1;
                end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                s_lp <= (others => '0'); s_dir <= '0'; s_band <= '0';
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
    -- Noise + address pipeline (stages 1..17)
    --------------------------------------------------------------------------
    p_noise : process(clk)
        variable v_k : integer range 2 to 9;
        variable v_t, v_b : signed(10 downto 0);
        variable v_o : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            v_k := to_integer(s_k);
            -- stage 1: cell index and Q7 fraction
            s_x1  <= s_rx;
            s_cx1 <= resize(shift_right(s_rx, v_k), 10);
            case v_k is
                when 2 => s_fx1 <= s_rx(1 downto 0) & "00000";
                when 3 => s_fx1 <= s_rx(2 downto 0) & "0000";
                when 4 => s_fx1 <= s_rx(3 downto 0) & "000";
                when 5 => s_fx1 <= s_rx(4 downto 0) & "00";
                when 6 => s_fx1 <= s_rx(5 downto 0) & "0";
                when 7 => s_fx1 <= s_rx(6 downto 0);
                when 8 => s_fx1 <= s_rx(7 downto 1);
                when others => s_fx1 <= s_rx(8 downto 2);
            end case;
            -- stage 2: lattice keys; sheared x
            s_ka <= s_rk0  + resize(s_cx1, 16);
            s_kb <= s_rk0p + resize(s_cx1, 16);
            s_kc <= s_rk1  + resize(s_cx1, 16);
            s_kd <= s_rk1p + resize(s_cx1, 16);
            s_xs(2) <= signed(resize(s_x1, 13)) + resize(s_sh_off, 13);
            for i in 3 to 15 loop s_xs(i) <= s_xs(i - 1); end loop;
            if s_sw_blocky = '1' then s_fxd(2) <= (others => '0'); else s_fxd(2) <= s_fx1; end if;
            for i in 3 to 7 loop s_fxd(i) <= s_fxd(i - 1); end loop;
            s_bandd(2) <= s_band_row;
            for i in 3 to 23 loop s_bandd(i) <= s_bandd(i - 1); end loop;
            -- stages 3..5: hash rounds
            s_ha3 <= f_round1(s_ka); s_hb3 <= f_round1(s_kb); s_hc3 <= f_round1(s_kc); s_hd3 <= f_round1(s_kd);
            s_ha4 <= f_round2(s_ha3); s_hb4 <= f_round2(s_hb3); s_hc4 <= f_round2(s_hc3); s_hd4 <= f_round2(s_hd3);
            s_ha5 <= f_round3(s_ha4); s_hb5 <= f_round3(s_hb4); s_hc5 <= f_round3(s_hc4); s_hd5 <= f_round3(s_hd4);
            -- stage 6: corner values
            s_a6 <= s_ha5(15 downto 8); s_b6 <= s_hb5(15 downto 8); s_c6 <= s_hc5(15 downto 8); s_d6 <= s_hd5(15 downto 8);
            -- stage 7: x differences
            s_dab7 <= signed('0' & s_b6) - signed('0' & s_a6);
            s_dcd7 <= signed('0' & s_d6) - signed('0' & s_c6);
            s_a7 <= s_a6; s_c7 <= s_c6;
            -- stage 8: x lerp products
            s_pab8 <= s_dab7 * signed('0' & s_fxd(7));
            s_pcd8 <= s_dcd7 * signed('0' & s_fxd(7));
            s_a8 <= s_a7; s_c8 <= s_c7;
            -- stage 9: top / bottom row values
            v_t := signed(resize(s_a8, 10) & '0') + resize(shift_right(s_pab8, 6), 11);
            v_b := signed(resize(s_c8, 10) & '0') + resize(shift_right(s_pcd8, 6), 11);
            s_top9 <= unsigned(v_t(8 downto 1));
            s_bot9 <= unsigned(v_b(8 downto 1));
            -- stage 10: y difference
            s_dtb10 <= signed('0' & s_bot9) - signed('0' & s_top9);
            s_top10 <= s_top9;
            -- stage 11: y lerp product
            if s_sw_blocky = '1' then
                s_ptb11 <= (others => '0');
            else
                s_ptb11 <= s_dtb10 * signed('0' & s_fy);
            end if;
            s_top11 <= s_top10;
            -- stage 12: noise value
            v_t := signed(resize(s_top11, 10) & '0') + resize(shift_right(s_ptb11, 6), 11);
            s_n12 <= unsigned(v_t(8 downto 1));
            -- stage 13: centred
            s_nc13 <= signed('0' & s_n12) - to_signed(128, 9);
            s_nd(13) <= s_n12;
            for i in 14 to 23 loop s_nd(i) <= s_nd(i - 1); end loop;
            -- stage 14: amplitude
            s_d14 <= s_nc13 * signed('0' & s_amp);
            -- stage 15: displacement / grain
            if s_sw_grain = '1' then
                s_off15 <= (others => '0');
                s_gr15  <= s_d14(16 downto 9);
            else
                s_off15 <= s_d14(16 downto 7);
                s_gr15  <= (others => '0');
            end if;
            -- stage 16: source x
            s_u16 <= s_xs(15) + resize(s_off15, 13);
            s_gd(16) <= s_gr15;
            for i in 17 to 19 loop s_gd(i) <= s_gd(i - 1); end loop;
            -- stage 17: address / background
            s_addr17 <= f_edge_addr(s_u16, s_w_edge);
        end if;
    end process p_noise;

    --------------------------------------------------------------------------
    -- Line buffers: written at stage 1 (s_in), read at stage 17 -> data 18
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr17));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr17));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr17(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr17(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr17(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr17(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fetch + duotone (stages 18..28)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y : signed(12 downto 0);
        variable v_s : unsigned(11 downto 0);
        variable v_g : signed(9 downto 0);
        variable v_m : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 18 ctx / 19 fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 20: luma above black; output luma with grain
            if s_wet_y < 64 then s_yc20 <= (others => '0'); else s_yc20 <= s_wet_y - 64; end if;
            v_g := resize(s_gd(19), 10);
            v_y := signed(resize(s_wet_y, 13)) + resize(shift_left(v_g, 2), 13);
            if v_y > 940 then v_y := to_signed(940, 13); end if;
            s_yo20 <= f_clamp10(v_y);
            -- stage 21: stretch x1.156
            s_ys21 <= resize(s_yc20, 12) + resize(shift_right(s_yc20, 3), 12) + resize(shift_right(s_yc20, 5), 12);
            s_yo21 <= s_yo20;
            -- stage 22: 8-bit key
            if s_ys21 > 1023 then s_y8_22 <= x"FF"; else s_y8_22 <= s_ys21(9 downto 2); end if;
            s_yo22 <= s_yo21;
            -- stage 23: grain into the key
            v_g := signed(resize(s_y8_22, 10)) + resize(s_gd(19), 10);   -- (gd(19) is 3 stages stale: per-pixel noise, acceptable)
            if v_g < 0 then s_y8g23 <= (others => '0'); elsif v_g > 255 then s_y8g23 <= x"FF"; else s_y8g23 <= unsigned(v_g(7 downto 0)); end if;
            s_yo23 <= s_yo22;
            -- stage 24: tone key select + band flip
            if (s_bandd(23) and s_sw_flip) = '1' then v_m := x"FF"; else v_m := x"00"; end if;
            if s_sw_tonenoise = '1' then
                s_t24 <= s_nd(23) xor v_m;
            else
                s_t24 <= s_y8g23 xor v_m;
            end if;
            s_yo24 <= s_yo23;
            -- stage 25: chroma lerp products
            s_pu25 <= s_du * signed('0' & s_t24);
            s_pv25 <= s_dv * signed('0' & s_t24);
            s_yo25 <= s_yo24;
            -- stage 26: chroma
            s_u26 <= f_clamp10(resize(s_uA, 13) + resize(shift_right(s_pu25, 8), 13));
            s_v26 <= f_clamp10(resize(s_vA, 13) + resize(shift_right(s_pv25, 8), 13));
            s_y26 <= s_yo25;
            -- stage 27: gate
            if s_avid_sr(25) = '1' then
                s_y27 <= s_y26; s_u27 <= s_u26; s_v27 <= s_v26;
            else
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            end if;
            -- stage 28: output
            s_y28 <= s_y27; s_u28 <= s_u27; s_v28 <= s_v27;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture tremor;
