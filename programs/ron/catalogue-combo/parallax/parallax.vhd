-- Parallax: catalogue combo #38 (seed 20260918) -- depth-from-luma parallax
-- + colour channel swap / offset.  Three knobs each; the switches and the
-- slider decide how the two combine.
--
--   K1 Depth    parallax gain (luma above / below focus shifts sideways)
--   K2 Sway     the camera sways on its own; speed
--   K3 Focus    luma level that does not move
--   K4 Swap     channel mode: none, U<>V, Y->U, U->Y, Y<>V, rotate
--   K5 Offset   chroma read offset (px, bipolar)
--   K6 Split    U and V offset opposite ways (0) .. the same way (100)
--
--   S7  Depth   from luma / from saturation
--   S8  Near    bright is near / dark is near
--   S9  Chroma  displaced with luma / stays put (colour ghosts)
--   S10 Sway    sine / triangle
--   S11 Edges   edge fill / wrap around
--   P12 Position camera position, bipolar (centre = flat)
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- Depth = the current pixel's own luma (dry ring), source = the line above
-- read at x + (luma - focus) * gain; U and V have their own read addresses
-- (free chromatic offset).  Latency 12 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture parallax of program_top is

    constant C_LAT : integer := 12;

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

    -- controls / per-frame
    signal s_k_depth, s_k_sway, s_k_focus, s_k_swap, s_k_offset, s_k_split : unsigned(9 downto 0);
    signal s_sw_sat, s_sw_darknear, s_sw_cstay, s_sw_tri, s_sw_wrap : std_logic;
    signal s_p_pos : unsigned(9 downto 0);
    signal s_focus  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_mode   : unsigned(2 downto 0) := (others => '0');
    signal s_coffu, s_coffv : signed(8 downto 0) := (others => '0');
    signal s_phase  : unsigned(15 downto 0) := (others => '0');
    signal s_pos    : signed(10 downto 0) := (others => '0');
    signal s_sw_s   : signed(9 downto 0) := (others => '0');
    signal s_gp     : signed(19 downto 0) := (others => '0');   -- 11 x 9
    signal s_g      : signed(9 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";

    -- tracking
    signal s_rx, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';

    -- address chain
    signal s_x1, s_x2, s_x3, s_x4, s_x5 : unsigned(10 downto 0) := (others => '0');
    signal s_dy2    : unsigned(9 downto 0) := (others => '0');
    signal s_du2, s_dv2 : signed(10 downto 0) := (others => '0');
    signal s_d3     : signed(11 downto 0) := (others => '0');
    signal s_pd4    : signed(21 downto 0) := (others => '0');
    signal s_off5   : signed(10 downto 0) := (others => '0');
    signal s_uy6, s_uu6, s_uv6 : signed(12 downto 0) := (others => '0');
    signal s_ay7, s_au7, s_av7 : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal s_a_rbank : std_logic := '0';
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');

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
    -- source value in parallel with the wrap add / subtract
    signal s_we13, s_wl13, s_wlo13, s_rwe13, s_rhi13 : signed(12 downto 0) := (others => '0');
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

    s_k_depth  <= unsigned(registers_in(0));
    s_k_sway   <= unsigned(registers_in(1));
    s_k_focus  <= unsigned(registers_in(2));
    s_k_swap   <= unsigned(registers_in(3));
    s_k_offset <= unsigned(registers_in(4));
    s_k_split  <= unsigned(registers_in(5));
    s_sw_sat      <= registers_in(6)(0);
    s_sw_darknear <= registers_in(6)(1);
    s_sw_cstay    <= registers_in(6)(2);
    s_sw_tri      <= registers_in(6)(3);
    s_sw_wrap     <= registers_in(6)(4);
    s_p_pos    <= unsigned(registers_in(7));

    -- dry ring: data_in delayed 2 (q at stage 2)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            s_ring_q <= ring(to_integer(s_ring_w - 1));
            ring(to_integer(s_ring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_t : signed(9 downto 0);
        variable v_o : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_focus <= s_k_focus;
                    if s_k_swap(9 downto 7) > 5 then s_mode <= to_unsigned(5, 3); else s_mode <= s_k_swap(9 downto 7); end if;
                    s_phase <= s_phase + resize(s_k_sway(9 downto 3), 16);
                    s_pos <= signed('0' & s_p_pos) - to_signed(512, 11);
                    v_o := signed('0' & s_k_offset(9 downto 1)) - to_signed(256, 10);
                    s_coffu <= resize(shift_right(v_o, 1), 9);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    -- sway: triangle of the phase (sine approximated by the triangle in tri mode; smooth via 2 segments)
                    if s_phase(15) = '0' then v_t := signed(s_phase(14 downto 5)); else v_t := signed(not s_phase(14 downto 5)); end if;
                    v_t := v_t - to_signed(256, 10);                       -- -256 .. 255
                    if s_sw_tri = '0' then
                        -- crude sine: t * (2 - |t|) style smoothing in 2 steps -> keep linear here, shape next step
                        s_sw_s <= v_t;
                    else
                        s_sw_s <= v_t;
                    end if;
                    -- split: v offset = -u (0) .. +u (100)
                    if s_k_split(9) = '1' then s_coffv <= s_coffu; else s_coffv <= -s_coffu; end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    -- position = manual + sway/2, then gain = position * depth (Q8)
                    s_gp <= (s_pos + resize(shift_right(s_sw_s, 1), 11)) * signed('0' & s_k_depth(9 downto 2));
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    if s_sw_darknear = '1' then s_g <= -resize(shift_right(s_gp, 8), 10); else s_g <= resize(shift_right(s_gp, 8), 10); end if;
                    s_vstep <= "111";
                when others => null;
            end case;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we13  <= signed(resize(s_w_edge, 13));                              -- w_edge
            s_wl13  <= signed(resize(s_line_width, 13));                          -- width
            s_wlo13 <= to_signed(C_EDGE, 13) - signed(resize(s_line_width, 13)); -- wrapped-left: w < C_EDGE <=> u < C_EDGE-width
            s_rwe13 <= signed(resize(s_line_width, 13)) + C_EDGE;                 -- wrapped-right: w < C_EDGE <=> u < width+C_EDGE
            s_rhi13 <= signed(resize(s_line_width, 13)) + signed(resize(s_w_edge, 13)); -- wrapped-right: w > w_edge <=> u > width+w_edge
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
    -- Address chain (stages 1..7)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_sat : unsigned(10 downto 0);
        variable v_au, v_av : unsigned(9 downto 0);
        variable v_w : signed(12 downto 0);
        procedure wrapit(signal a : out unsigned(10 downto 0); u : signed(12 downto 0)) is
            variable wl, wr : signed(12 downto 0);
        begin
            -- every compare is on u against a registered bound, in parallel
            -- with the wrap add / subtract; the mux is the only serial step
            wl := u + s_wl13; wr := u - s_wl13;
            if s_sw_wrap = '1' and u < 0 then
                if u < s_wlo13 then a <= to_unsigned(C_EDGE, 11);
                elsif u > -(C_EDGE + 1) then a <= s_w_edge;
                else a <= unsigned(wl(10 downto 0)); end if;
            elsif s_sw_wrap = '1' and u >= s_wl13 then
                if u < s_rwe13 then a <= to_unsigned(C_EDGE, 11);
                elsif u > s_rhi13 then a <= s_w_edge;
                else a <= unsigned(wr(10 downto 0)); end if;
            elsif u < C_EDGE then a <= to_unsigned(C_EDGE, 11);
            elsif u > s_we13 then a <= s_w_edge;
            else a <= unsigned(u(10 downto 0)); end if;
        end procedure;
    begin
        if rising_edge(clk) then
            -- stage 1
            s_x1 <= s_rx;
            -- stage 2: dry pixel measures
            s_x2 <= s_x1;
            s_dy2 <= unsigned(s_ring_q(29 downto 20));
            s_du2 <= signed('0' & unsigned(s_ring_q(19 downto 10))) - to_signed(512, 11);
            s_dv2 <= signed('0' & unsigned(s_ring_q(9 downto 0))) - to_signed(512, 11);
            -- stage 3: depth difference (luma - focus, or saturation - focus/2)
            s_x3 <= s_x2;
            if s_sw_sat = '1' then
                if s_du2 < 0 then v_au := unsigned(resize(-s_du2, 10)); else v_au := unsigned(resize(s_du2, 10)); end if;
                if s_dv2 < 0 then v_av := unsigned(resize(-s_dv2, 10)); else v_av := unsigned(resize(s_dv2, 10)); end if;
                v_sat := resize(v_au, 11) + resize(v_av, 11);
                s_d3 <= signed(resize(v_sat, 12)) - signed(resize(s_focus(9 downto 1), 12));
            else
                s_d3 <= signed(resize(s_dy2, 12)) - signed(resize(s_focus, 12));
            end if;
            -- stage 4: displacement product
            s_x4 <= s_x3;
            s_pd4 <= s_d3 * s_g;
            -- stage 5: offset (+-511 px)
            s_x5 <= s_x4;
            v_w := resize(shift_right(s_pd4, 8), 13);
            if v_w > 511 then v_w := to_signed(511, 13); elsif v_w < -511 then v_w := to_signed(-511, 13); end if;
            s_off5 <= resize(v_w, 11);
            -- stage 6: per-channel source x
            s_uy6 <= signed(resize(s_x5, 13)) + resize(s_off5, 13);
            if s_sw_cstay = '1' then
                s_uu6 <= signed(resize(s_x5, 13)) + resize(s_coffu, 13);
                s_uv6 <= signed(resize(s_x5, 13)) + resize(s_coffv, 13);
            else
                s_uu6 <= signed(resize(s_x5, 13)) + resize(s_off5, 13) + resize(s_coffu, 13);
                s_uv6 <= signed(resize(s_x5, 13)) + resize(s_off5, 13) + resize(s_coffv, 13);
            end if;
            -- stage 7: wrap / background per channel
            wrapit(s_ay7, s_uy6);
            wrapit(s_au7, s_uu6);
            wrapit(s_av7, s_uv6);
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 1, read stage 7 -> data 8), per-channel addresses
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_ay7));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_ay7));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_au7(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_au7(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_av7(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_av7(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 8..12)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 8 ctx / 9 fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then v_y := unsigned(s_rdY1); v_u := unsigned(s_rdU1) & "00"; v_v := unsigned(s_rdV1) & "00";
            else v_y := unsigned(s_rdY0); v_u := unsigned(s_rdU0) & "00"; v_v := unsigned(s_rdV0) & "00"; end if;
            s_y9 <= v_y; s_u9 <= v_u; s_v9 <= v_v;
            -- stage 10: channel swap
            case to_integer(s_mode) is
                when 0 => s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
                when 1 => s_y10 <= s_y9; s_u10 <= s_v9; s_v10 <= s_u9;
                when 2 => s_y10 <= s_y9; s_u10 <= s_y9; s_v10 <= s_v9;
                when 3 => s_y10 <= s_u9; s_u10 <= s_u9; s_v10 <= s_v9;
                when 4 => s_y10 <= s_v9; s_u10 <= s_u9; s_v10 <= s_y9;
                when others => s_y10 <= s_u9; s_u10 <= s_v9; s_v10 <= s_y9;
            end case;
            -- stage 11: gate (luma capped at 940)
            if s_avid_sr(9) = '1' then
                if s_y10 > 940 then s_y11 <= to_unsigned(940, 10); else s_y11 <= s_y10; end if;
                s_u11 <= s_u10; s_v11 <= s_v10;
            else
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            end if;
            -- stage 12: output
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y12);
    data_out.u       <= std_logic_vector(s_u12);
    data_out.v       <= std_logic_vector(s_v12);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture parallax;
