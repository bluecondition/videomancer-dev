-- Hatchglass: catalogue combo #93 (seed 20260924) -- mirror / kaleidoscope
-- + crosshatch / stipple + chroma / luma key.  Two knobs each; the switches
-- and the slider decide how they combine.
--
--   K1 Segment  mirror segment width 1024 / 512 / 256 / 128 / 64 px
--   K2 Slide    slide speed of the mirrored strip, bipolar
--   K3 Weight   hatch line weight (1..8 px)
--   K4 Pitch    hatch spacing 4 / 8 / 16 / 32 px
--   K5 Level    key threshold
--   K6 Soft     key edge softness
--
--   S7  Key     luma / saturation
--   S8  Invert  key inverted
--   S9  Hatch   hatching where keyed / the mirrored picture where keyed
--   S10 Paper   hatch on white / hatch over the mirrored picture
--   S11 Fold    mirror (triangle) / repeat (sawtooth tiles)
--   P12 Depth   nested folds 1..4 (each halves the segment)
--
-- Nested power-of-two folds (registered per-frame constants); the reflected
-- luma picks 0..4 hatch directions; a luma or sat key from the dry pixel
-- chooses hatch or picture.  Latency 25 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture hatchglass of program_top is

    constant C_LAT : integer := 25;

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

    function f_clamp255(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 8);
        elsif v > 255 then
            return to_unsigned(255, 8);
        else
            return unsigned(v(7 downto 0));
        end if;
    end function;

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    type t_ring is array (0 to 63) of std_logic_vector(31 downto 0);
    signal ring : t_ring := (others => (others => '0'));
    signal s_rrd : std_logic_vector(31 downto 0) := (others => '0');

    signal s_k_seg, s_k_slide, s_k_wt, s_k_pitch, s_k_lvl, s_k_soft : unsigned(9 downto 0);
    signal s_sw_satkey, s_sw_inv, s_sw_hatchkeyed, s_sw_over, s_sw_repeat : std_logic;
    signal s_wt : unsigned(3 downto 0) := (others => '0');
    signal s_pmask : unsigned(5 downto 0) := (others => '0');
    signal s_sw_keywet : std_logic := '0';
    signal s_sw_black : std_logic := '0';
    signal s_pa17, s_pb17, s_pc17, s_pd17 : unsigned(5 downto 0) := (others => '0');
    signal s_dens17 : unsigned(2 downto 0) := (others => '0');
    signal s_hy18, s_hu18, s_hv18 : unsigned(9 downto 0) := (others => '0');
    signal s_hatch18 : std_logic := '0';
    type t_xd is array (2 to 16) of unsigned(10 downto 0);
    signal s_xd : t_xd := (others => (others => '0'));
    signal s_p_depth : unsigned(9 downto 0);

    signal s_ks     : unsigned(3 downto 0) := to_unsigned(9, 4);   -- log2 W: 10..6
    signal s_depth  : unsigned(1 downto 0) := "00";
    signal s_lvl    : unsigned(9 downto 0) := (others => '0');
    signal s_soft   : unsigned(2 downto 0) := (others => '0');
    signal s_tu, s_tv : signed(8 downto 0) := (others => '0');
    signal s_skew   : signed(8 downto 0) := (others => '0');
    signal s_slide  : signed(15 downto 0) := (others => '0');
    signal s_shrow  : signed(15 downto 0) := (others => '0');
    signal s_cx     : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_saw_active : std_logic := '0';
    signal s_wp     : unsigned(5 downto 0) := (others => '0');

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    type t_b is array (1 to 13) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_x1 : unsigned(10 downto 0) := (others => '0');
    signal s_a3 : signed(12 downto 0) := (others => '0');
    type t_f is array (0 to 4) of signed(12 downto 0);
    signal s_m, s_t : t_f := (others => (others => '0'));
    type t_fc is array (1 to 4) of signed(12 downto 0);
    signal s_fmask, s_fhalf, s_flow : t_fc := (others => (others => '0'));
    signal s_halff : signed(12 downto 0) := (others => '0');
    signal s_fen : std_logic_vector(1 to 4) := (others => '0');   -- m(i): masked; t(i): folded, i = fold stage
    signal s_src12 : signed(12 downto 0) := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg : std_logic := '0';
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    -- dry key path (early)
    signal s_au2, s_av2, s_y2 : unsigned(9 downto 0) := (others => '0');
    signal s_key3 : unsigned(9 downto 0) := (others => '0');
    signal s_kd4 : signed(10 downto 0) := (others => '0');
    signal s_w5 : unsigned(7 downto 0) := (others => '0');
    type t_w is array (5 to 19) of unsigned(7 downto 0);
    signal s_wd : t_w := (others => (others => '0'));
    -- wet key path
    signal s_au16, s_av16, s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_key17 : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_kd18 : signed(10 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');
    signal s_ww19 : unsigned(7 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');
    signal s_ra19 : unsigned(5 downto 0) := (others => '0');
    -- blend
    signal s_w20 : unsigned(7 downto 0) := (others => '0');
    signal s_y20 : unsigned(9 downto 0) := (others => '0');
    signal s_u20, s_v20 : signed(11 downto 0) := (others => '0');
    signal s_w21 : unsigned(7 downto 0) := (others => '0');
    signal s_oy20, s_ou20, s_ov20, s_by21, s_bu21, s_bv21 : unsigned(9 downto 0) := (others => '0');
    signal s_oy19, s_ou19, s_ov19 : unsigned(9 downto 0) := (others => '0');
    signal s_wy21, s_wu21, s_wv21 : unsigned(9 downto 0) := (others => '0');
    signal s_dy21, s_du21, s_dv21 : unsigned(9 downto 0) := (others => '0');
    signal s_dy22, s_du22, s_dv22 : signed(10 downto 0) := (others => '0');
    signal s_by22, s_bu22, s_bv22 : unsigned(9 downto 0) := (others => '0');
    signal s_w22 : unsigned(7 downto 0) := (others => '0');
    signal s_py23, s_pu23, s_pv23 : signed(19 downto 0) := (others => '0');
    signal s_by23, s_bu23, s_bv23 : unsigned(9 downto 0) := (others => '0');
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_seg   <= unsigned(registers_in(0));
    s_k_slide <= unsigned(registers_in(1));
    s_k_wt    <= unsigned(registers_in(2));
    s_k_pitch <= unsigned(registers_in(3));
    s_k_lvl   <= unsigned(registers_in(4));
    s_k_soft  <= unsigned(registers_in(5));
    s_sw_satkey     <= registers_in(6)(0);
    s_sw_inv        <= registers_in(6)(1);
    s_sw_hatchkeyed <= registers_in(6)(2);
    s_sw_over       <= registers_in(6)(3);
    s_sw_repeat     <= registers_in(6)(4);
    s_p_depth <= unsigned(registers_in(7));

    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_wp <= s_wp + 1;

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
            s_lvl <= s_k_lvl; s_soft <= s_k_soft(9 downto 7);
            s_tu <= (others => '0'); s_tv <= (others => '0'); s_skew <= (others => '0');
            s_wt <= resize(s_k_wt(9 downto 7), 4) + 1;
            case to_integer(s_k_pitch(9 downto 8)) is
                when 0 => s_pmask <= "000011";
                when 1 => s_pmask <= "000111";
                when 2 => s_pmask <= "001111";
                when others => s_pmask <= "011111";
            end case;
            s_cx <= '0' & s_line_width(10 downto 1);

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                s_shrow <= s_shrow + resize(s_skew, 16);
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then s_slide <= s_slide + resize(shift_right(signed(resize(s_k_slide, 11)) - 512, 3), 16); end if;
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
        variable v_kd : signed(10 downto 0);
        variable v_w : signed(12 downto 0);
        variable v_mask : signed(12 downto 0);
        variable v_half : signed(12 downto 0);
        variable v_k : integer;
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 13 loop s_avd(i) <= s_avd(i - 1); end loop;
            s_xd(2) <= s_x1;
            for i in 3 to 16 loop s_xd(i) <= s_xd(i - 1); end loop;
            -- 3: position relative to the centre plus slide
            s_a3 <= signed(resize(s_x1, 13)) - signed(resize(s_cx, 13)) + resize(s_shrow(15 downto 4), 13);
            -- folds: stage i (i = 1..4) uses W_i = 2^(ks - i + 1); m = a mod 2W ; t = |m - W| (mirror) or m - W (repeat)
            s_t(0) <= s_a3;
            for i in 1 to 4 loop
                s_m(i) <= (s_t(i - 1) and s_fmask(i));
                if s_fen(i) = '1' then
                    if s_sw_repeat = '1' then
                        s_t(i) <= s_m(i);
                    elsif (s_m(i) and s_fhalf(i)) /= 0 then
                        s_t(i) <= s_m(i) and s_flow(i);
                    else
                        s_t(i) <= s_fhalf(i) - s_m(i);
                    end if;
                else
                    s_t(i) <= s_t(i - 1);
                end if;
            end loop;
            -- 12: source position = centre - W_final/2 + t
            s_src12 <= signed(resize(s_cx, 13)) - s_halff + s_t(4);
            -- 13
            if s_src12 < 0 or s_src12 >= signed(resize(s_line_width, 13)) or s_avd(12) = '0' then s_bg <= '1'; else s_bg <= '0'; end if;
            s_addr <= unsigned(s_src12(10 downto 0));

            -- dry key path: 2..5
            if s_in_u >= 512 then s_au2 <= s_in_u - 512; else s_au2 <= 512 - s_in_u; end if;
            if s_in_v >= 512 then s_av2 <= s_in_v - 512; else s_av2 <= 512 - s_in_v; end if;
            s_y2 <= s_in_y;
            if s_sw_satkey = '1' then s_key3 <= resize(shift_right(resize(s_au2, 11) + resize(s_av2, 11), 1), 10); else s_key3 <= s_y2; end if;
            if s_sw_inv = '1' then v_kd := signed(resize(s_lvl, 11)) - signed(resize(s_key3, 11));
            else v_kd := signed(resize(s_key3, 11)) - signed(resize(s_lvl, 11)); end if;
            s_kd4 <= v_kd;
            case to_integer(s_soft) is
                when 0 => v_w := resize(shift_left(resize(s_kd4, 13), 5), 13);
                when 1 => v_w := resize(shift_left(resize(s_kd4, 13), 4), 13);
                when 2 => v_w := resize(shift_left(resize(s_kd4, 13), 3), 13);
                when 3 => v_w := resize(shift_left(resize(s_kd4, 13), 2), 13);
                when 4 => v_w := resize(shift_left(resize(s_kd4, 13), 1), 13);
                when 5 => v_w := resize(s_kd4, 13);
                when 6 => v_w := resize(shift_right(s_kd4, 1), 13);
                when others => v_w := resize(shift_right(s_kd4, 2), 13);
            end case;
            s_w5 <= f_clamp255(v_w);
            s_wd(5) <= s_w5;
            for i in 6 to 19 loop s_wd(i) <= s_wd(i - 1); end loop;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    -- dry ring: input written at d1, read 18 back so the dry pixel lands with the wet one
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rrd <= ring(to_integer(s_ra19));
            ring(to_integer(s_wp)) <= std_logic_vector(s_in_y) & std_logic_vector(s_in_u(9 downto 2)) & std_logic_vector(s_in_v(9 downto 2)) & "000000";
        end if;
    end process;

    p_pix : process(clk)
        variable v_kd : signed(10 downto 0);
        variable v_w : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 14 (rd) / 15: fetch
            s_a_bg <= s_bg; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 16: reflected luma density -> hatch phases (screen x, line)
            if s_wet_y < 128 then s_dens17 <= "100"; else s_dens17 <= resize(3 - s_wet_y(9 downto 8), 3); end if;
            s_pa17 <= (s_xd(16)(5 downto 0) + s_line(5 downto 0)) and s_pmask;
            s_pb17 <= (s_xd(16)(5 downto 0) - s_line(5 downto 0)) and s_pmask;
            s_pc17 <= s_line(5 downto 0) and s_pmask;
            s_pd17 <= s_xd(16)(5 downto 0) and s_pmask;
            s_y16 <= s_wet_y; s_u16 <= s_wet_u; s_v16 <= s_wet_v;
            -- 17: hatch decision
            v_kd := (others => '0');
            if (s_dens17 >= 1 and s_pa17 < resize(s_wt, 6)) or (s_dens17 >= 2 and s_pb17 < resize(s_wt, 6))
               or (s_dens17 >= 3 and s_pc17 < resize(s_wt, 6)) or (s_dens17 >= 4 and s_pd17 < resize(s_wt, 6)) then
                s_hatch18 <= '1';
            else
                s_hatch18 <= '0';
            end if;
            s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            -- 18: hatched pixel
            if s_hatch18 = '1' then
                s_hy18 <= to_unsigned(64, 10); s_hu18 <= to_unsigned(512, 10); s_hv18 <= to_unsigned(512, 10);
            elsif s_sw_over = '1' then
                s_hy18 <= s_y17; s_hu18 <= s_u17; s_hv18 <= s_v17;
            else
                s_hy18 <= to_unsigned(940, 10); s_hu18 <= to_unsigned(512, 10); s_hv18 <= to_unsigned(512, 10);
            end if;
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            -- 19: choose keyed target: hatch vs mirrored picture
            if s_sw_hatchkeyed = '1' then s_y19 <= s_hy18; s_u19 <= s_hu18; s_v19 <= s_hv18;
            else s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18; end if;
            if s_sw_hatchkeyed = '1' then s_oy19 <= s_y18; s_ou19 <= s_u18; s_ov19 <= s_v18;
            else s_oy19 <= s_hy18; s_ou19 <= s_hu18; s_ov19 <= s_hv18; end if;
            s_ww19 <= (others => '0');
            s_ra19 <= s_wp - 18;
            -- 20: weight select, tint on the wet chroma
            s_w20 <= s_wd(19);
            s_y20 <= s_y19; s_oy20 <= s_oy19; s_ou20 <= s_ou19; s_ov20 <= s_ov19;
            s_u20 <= signed(resize(s_u19, 12)) + resize(s_tu, 12);
            s_v20 <= signed(resize(s_v19, 12)) + resize(s_tv, 12);
            -- 21: clamp, dry from the ring (read issued at 19, out at 20)
            s_w21 <= s_w20; s_wy21 <= s_y20; s_wu21 <= f_clamp10(s_u20); s_wv21 <= f_clamp10(s_v20);
            s_by21 <= s_oy20; s_bu21 <= s_ou20; s_bv21 <= s_ov20;
            s_dy21 <= unsigned(s_rrd(31 downto 22)); s_du21 <= unsigned(s_rrd(21 downto 14)) & "00"; s_dv21 <= unsigned(s_rrd(13 downto 6)) & "00";
            -- 22: base and difference
            -- base = whichever of (hatch, picture) is not the keyed target (carried as the other 20-stage value)
            s_by22 <= s_by21; s_bu22 <= s_bu21; s_bv22 <= s_bv21;
            s_dy22 <= signed(resize(s_wy21, 11)) - signed(resize(s_by21, 11));
            s_du22 <= signed(resize(s_wu21, 11)) - signed(resize(s_bu21, 11));
            s_dv22 <= signed(resize(s_wv21, 11)) - signed(resize(s_bv21, 11));
            s_w22 <= s_w21;
            -- 23
            s_py23 <= s_dy22 * signed(resize(s_w22, 9));
            s_pu23 <= s_du22 * signed(resize(s_w22, 9));
            s_pv23 <= s_dv22 * signed(resize(s_w22, 9));
            s_by23 <= s_by22; s_bu23 <= s_bu22; s_bv23 <= s_bv22;
            -- 24: apply + gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y24 <= f_clamp10(signed(resize(s_by23, 13)) + resize(shift_right(s_py23, 8), 13));
                s_u24 <= f_clamp10(signed(resize(s_bu23, 13)) + resize(shift_right(s_pu23, 8), 13));
                s_v24 <= f_clamp10(signed(resize(s_bv23, 13)) + resize(shift_right(s_pv23, 8), 13));
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture hatchglass;
