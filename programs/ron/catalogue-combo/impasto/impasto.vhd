-- Impasto: catalogue combo #23 (seed 20260916) -- Perlin / simplex
-- displacement + oil paint / Kuwahara smoothing.  Three knobs each; the
-- switches and the slider decide how the two combine.
--
--   K1 Scale    noise cell size 4 .. 512 px
--   K2 Drift    noise field scrolls vertically
--   K3 Stroke   noise cells stretched horizontally x1 .. x8
--   K4 Brush    smoothing window 4 .. 32 px each side
--   K5 Edge     0 = plain box blur .. 100 = hard nearest-mean pick
--   K6 Relief   brush-stroke relief (difference of the two window means)
--
--   S7  Noise   warps the picture / steers which window is picked
--   S8  Field   smooth value noise / hard cells
--   S9  Pick    nearest mean / farthest mean (haloed oil)
--   S10 Relief  light from the left / from the right
--   S11 Knife   plain / luma posterized to 8 steps
--   P12 Amount  displacement amplitude, up to +-255 px
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel -- the fill continues the colour
-- leading up to it, with no black seam.
-- Kuwahara-lite: running box sums of luma (EBR ring) give the mean of the
-- N pixels left and right of a centre pixel taken 33 px behind the input
-- (delay ring); the mean nearer the centre value wins.  The result is
-- written through a delayed write port and read back displaced by 2-D
-- value noise (four hashes per pixel).  Latency 30 clocks, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture impasto of program_top is

    constant C_LAT  : integer := 30;
    constant C_SEED : unsigned(15 downto 0) := x"91E3";
    constant C_DLY  : integer := 35;       -- centre pixel delay (input stream)

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

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_ring32 is array (0 to 63) of std_logic_vector(31 downto 0);
    type t_ring16 is array (0 to 63) of std_logic_vector(15 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal dring : t_ring32 := (others => x"04080200");    -- dry, read 33 back
    signal bring : t_ring16 := (others => x"0040");        -- luma, read N-1 back (box sum)
    signal rring : t_ring16 := (others => x"0000");        -- sums, read 32-N back (right window)
    signal lring : t_ring16 := (others => x"0000");        -- sums, read 33 back (left window)
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_dring_q : std_logic_vector(31 downto 0) := x"04080200";
    signal s_bring_q, s_rring_q, s_lring_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ring_w : unsigned(5 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_scale, s_k_drift, s_k_stroke, s_k_brush, s_k_edge, s_k_relief : unsigned(9 downto 0);
    signal s_sw_steer, s_sw_blocky, s_sw_far, s_sw_right, s_sw_knife : std_logic;
    signal s_p_amount : unsigned(9 downto 0);
    signal s_k      : unsigned(3 downto 0) := to_unsigned(5, 4);       -- y cell shift 2..9
    signal s_kx     : unsigned(3 downto 0) := to_unsigned(5, 4);       -- x cell shift = k + stroke, max 10
    signal s_amp    : unsigned(7 downto 0) := (others => '0');
    signal s_nsh    : unsigned(2 downto 0) := to_unsigned(3, 3);       -- brush shift 2..5 (N = 4..32)
    signal s_ndel, s_rdel : unsigned(5 downto 0) := (others => '0');
    signal s_edge   : unsigned(7 downto 0) := (others => '0');
    signal s_relief : unsigned(7 downto 0) := (others => '0');
    signal s_nphase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep  : unsigned(1 downto 0) := "11";

    -- tracking / per-line
    signal s_rx, s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank : std_logic := '0';
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_ly     : unsigned(11 downto 0) := (others => '0');
    signal s_cy     : unsigned(9 downto 0) := (others => '0');
    signal s_fy     : unsigned(6 downto 0) := (others => '0');
    signal s_rk0, s_rk0p, s_rk1, s_rk1p : unsigned(15 downto 0) := (others => '0');

    -- input side: box sums, windows, pick, write port
    signal s_acc    : unsigned(15 downto 0) := (others => '0');
    signal s_y1     : unsigned(9 downto 0) := (others => '0');
    signal s_wl4, s_wr4 : unsigned(9 downto 0) := (others => '0');
    signal s_cy4, s_cu4, s_cv4 : unsigned(9 downto 0) := (others => '0');
    signal s_dl5, s_dr5 : unsigned(9 downto 0) := (others => '0');
    signal s_wl5, s_wr5 : unsigned(9 downto 0) := (others => '0');
    signal s_cy5, s_cu5, s_cv5 : unsigned(9 downto 0) := (others => '0');
    signal s_rl5    : signed(10 downto 0) := (others => '0');
    signal s_pick6  : unsigned(9 downto 0) := (others => '0');
    signal s_avg6   : unsigned(9 downto 0) := (others => '0');
    signal s_cy6, s_cu6, s_cv6 : unsigned(9 downto 0) := (others => '0');
    signal s_pr6h, s_pr6l : signed(15 downto 0) := (others => '0');   -- relief product, hi / lo nibble of Relief
    signal s_e7     : signed(10 downto 0) := (others => '0');
    signal s_avg7   : unsigned(9 downto 0) := (others => '0');
    signal s_cu7, s_cv7 : unsigned(9 downto 0) := (others => '0');
    signal s_rf7    : signed(10 downto 0) := (others => '0');
    signal s_pe8    : signed(19 downto 0) := (others => '0');
    signal s_avg8   : unsigned(9 downto 0) := (others => '0');
    signal s_cu8, s_cv8 : unsigned(9 downto 0) := (others => '0');
    signal s_rf8    : signed(10 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_wcnt   : unsigned(10 downto 0) := (others => '0');
    signal s_wpd    : std_logic_vector(0 to C_DLY + 9) := (others => '0');
    signal s_avd    : std_logic_vector(0 to C_DLY + 9) := (others => '0');
    signal s_we9    : std_logic := '0';
    signal s_wp9    : std_logic := '0';
    signal s_steer  : std_logic := '0';

    -- noise pipeline (mirrors tremor)
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
    type t_fx is array (2 to 7) of unsigned(6 downto 0);
    signal s_fxd    : t_fx := (others => (others => '0'));
    type t_xs is array (2 to 15) of unsigned(10 downto 0);
    signal s_xd     : t_xs := (others => (others => '0'));
    signal s_u16    : signed(12 downto 0) := (others => '0');
    signal s_addr17 : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');
    type t_yd is array (20 to 28) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_yd := (others => (others => '0'));
    signal s_y29, s_u29, s_v29 : unsigned(9 downto 0) := (others => '0');
    signal s_y30, s_u30, s_v30 : unsigned(9 downto 0) := (others => '0');

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
    s_k_stroke <= unsigned(registers_in(2));
    s_k_brush  <= unsigned(registers_in(3));
    s_k_edge   <= unsigned(registers_in(4));
    s_k_relief <= unsigned(registers_in(5));
    s_sw_steer  <= registers_in(6)(0);
    s_sw_blocky <= registers_in(6)(1);
    s_sw_far    <= registers_in(6)(2);
    s_sw_right  <= registers_in(6)(3);
    s_sw_knife  <= registers_in(6)(4);
    s_p_amount <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Rings.  s_acc at cycle c is the N-sum ending at pixel p(c-2).  The
    -- centre C = p(c-35) (dring); the left window [C-N..C-1] = sum ending at
    -- C-1, written 33 cycles earlier (lring, read 34 back); the right window
    -- [C+1..C+N] = sum ending at C+N, written 32-N cycles earlier (rring,
    -- read 33-N back).  Write port: C lands at stage 9, 41 cycles after input.
    --------------------------------------------------------------------------
    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_w <= s_ring_w + 1;
            dring(to_integer(s_ring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
            s_dring_q <= dring(to_integer(s_ring_w - C_DLY));
            bring(to_integer(s_ring_w)) <= "000000" & std_logic_vector(s_y1);
            s_bring_q <= bring(to_integer(s_ring_w - s_ndel));
            rring(to_integer(s_ring_w)) <= std_logic_vector(s_acc);
            s_rring_q <= rring(to_integer(s_ring_w - s_rdel));
            lring(to_integer(s_ring_w)) <= std_logic_vector(s_acc);
            s_lring_q <= lring(to_integer(s_ring_w - 34));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control + input side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : integer range 2 to 10;
        variable v_n : unsigned(3 downto 0);
        variable v_dl, v_dr : unsigned(9 downto 0);
        variable v_t : signed(12 downto 0);
        variable v_near : std_logic;
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            s_avd(0) <= data_in.avid;
            for i in 1 to C_DLY + 9 loop s_avd(i) <= s_avd(i - 1); end loop;

            -- stage 1: luma, held through blanking so the windows straddling a line
            -- end read the edge colour rather than blanking black
            if data_in.avid = '1' then s_y1 <= unsigned(data_in.y); end if;
            -- stage 2: running sum over N (window ending at the stage-1 pixel);
            -- re-synced to hold*N through blanking while the ring refills with the held value
            if s_rx = 0 then
                s_acc <= shift_left(resize(s_y1, 16), to_integer(s_nsh));
            else
                s_acc <= s_acc + resize(s_y1, 16) - resize(unsigned(s_bring_q(9 downto 0)), 16);
            end if;
            -- stage 3: (ring reads land)  stage 4: window means + centre pixel
            --   centre pixel c is the input 33 back (dring, q at stage 1 of p-33 -> use as stage 4 data)
            s_wl4 <= resize(shift_right(unsigned(s_lring_q), to_integer(s_nsh)), 10);
            s_wr4 <= resize(shift_right(unsigned(s_rring_q), to_integer(s_nsh)), 10);
            s_cy4 <= unsigned(s_dring_q(29 downto 20)); s_cu4 <= unsigned(s_dring_q(19 downto 10)); s_cv4 <= unsigned(s_dring_q(9 downto 0));
            -- stage 5: distances, relief difference
            if s_wl4 > s_cy4 then v_dl := s_wl4 - s_cy4; else v_dl := s_cy4 - s_wl4; end if;
            if s_wr4 > s_cy4 then v_dr := s_wr4 - s_cy4; else v_dr := s_cy4 - s_wr4; end if;
            s_dl5 <= v_dl; s_dr5 <= v_dr;
            s_wl5 <= s_wl4; s_wr5 <= s_wr4;
            s_cy5 <= s_cy4; s_cu5 <= s_cu4; s_cv5 <= s_cv4;
            s_rl5 <= signed('0' & s_wl4) - signed('0' & s_wr4);
            -- stage 6: pick, average, relief product
            if s_dl5 <= s_dr5 then v_near := '1'; else v_near := '0'; end if;
            if (v_near xor s_sw_far xor (s_sw_steer and s_steer)) = '1' then s_pick6 <= s_wl5; else s_pick6 <= s_wr5; end if;
            s_avg6 <= resize(shift_right(resize(s_wl5, 11) + resize(s_wr5, 11), 1), 10);
            s_cy6 <= s_cy5; s_cu6 <= s_cu5; s_cv6 <= s_cv5;
            -- (split on the Relief nibbles, sign applied after the product: the negate+mux in
            -- front of an 11x9 multiply was the HD critical path)
            s_pr6h <= s_rl5 * signed('0' & s_relief(7 downto 4));
            s_pr6l <= s_rl5 * signed('0' & s_relief(3 downto 0));
            -- stage 7: pick - avg, relief term
            s_e7 <= signed('0' & s_pick6) - signed('0' & s_avg6);
            s_avg7 <= s_avg6; s_cu7 <= s_cu6; s_cv7 <= s_cv6;
            if s_sw_right = '1' then
                s_rf7 <= -resize(shift_right(shift_left(resize(s_pr6h, 20), 4) + resize(s_pr6l, 20), 7), 11);
            else
                s_rf7 <= resize(shift_right(shift_left(resize(s_pr6h, 20), 4) + resize(s_pr6l, 20), 7), 11);
            end if;
            -- stage 8: edge product
            s_pe8 <= s_e7 * signed('0' & s_edge);
            s_avg8 <= s_avg7; s_cu8 <= s_cu7; s_cv8 <= s_cv7; s_rf8 <= s_rf7;
            -- stage 9: smoothed luma (+ relief), optional knife
            v_t := signed(resize(s_avg8, 13)) + resize(shift_right(s_pe8, 8), 13) + resize(s_rf8, 13);
            if s_sw_knife = '1' then
                s_y9 <= (f_clamp10(v_t) and "1110000000") or "0001000000";
            else
                s_y9 <= f_clamp10(v_t);
            end if;
            s_u9 <= s_cu8; s_v9 <= s_cv8;
            -- delayed write port: avid + parity delayed C_DLY + 9 (centre pixel is 33 behind, +8 stages after stage 1 of it)
            s_wpd(0) <= s_wpar;
            for i in 1 to C_DLY + 9 loop s_wpd(i) <= s_wpd(i - 1); end loop;
            s_we9 <= s_avd(C_DLY + 5);
            s_wp9 <= s_wpd(C_DLY + 4);
            if s_we9 = '1' then s_wcnt <= s_wcnt + 1; else s_wcnt <= (others => '0'); end if;

            -- per-line: row keys
            v_k := to_integer(s_k);
            case to_integer(s_lstep) is
                when 0 =>
                    s_ly <= resize(s_line, 12) + resize(s_nphase(15 downto 4), 12);
                    s_lstep <= s_lstep + 1;
                when 1 =>
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
                    s_rk0 <= C_SEED + shift_left(resize(s_cy, 16), 7);
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_rk0p <= s_rk0 + 1;
                    s_rk1  <= s_rk0 + 128;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_rk1p <= s_rk1 + 1;
                    s_lstep <= "111";
                when others => null;
            end case;

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_amp <= s_p_amount(9 downto 2);
                    if s_k_scale(9 downto 7) = 7 then s_k <= to_unsigned(9, 4); else s_k <= resize(s_k_scale(9 downto 7), 4) + 2; end if;
                    v_n := resize(s_k_brush(9 downto 8), 4) + 2;      -- 2..5
                    s_nsh  <= v_n(2 downto 0);
                    s_edge <= s_k_edge(9 downto 2);
                    s_relief <= s_k_relief(9 downto 2);
                    s_nphase <= s_nphase + resize(s_k_drift(9 downto 3), 16);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    if to_integer(s_k) + to_integer(s_k_stroke(9 downto 8)) > 10 then
                        s_kx <= to_unsigned(10, 4);
                    else
                        s_kx <= s_k + resize(s_k_stroke(9 downto 8), 4);
                    end if;
                    s_ndel <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)), 7)(5 downto 0) - 1;
                    s_rdel <= to_unsigned(33, 6) - resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)), 7)(5 downto 0);
                    s_vstep <= "11";
                when others => null;
            end case;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
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

    --------------------------------------------------------------------------
    -- Noise + address pipeline (stages 1..17), as in tremor
    --------------------------------------------------------------------------
    p_noise : process(clk)
        variable v_kx : integer range 2 to 10;
        variable v_t, v_b : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            v_kx := to_integer(s_kx);
            s_x1  <= s_rx;
            s_cx1 <= resize(shift_right(s_rx, v_kx), 10);
            case v_kx is
                when 2 => s_fx1 <= s_rx(1 downto 0) & "00000";
                when 3 => s_fx1 <= s_rx(2 downto 0) & "0000";
                when 4 => s_fx1 <= s_rx(3 downto 0) & "000";
                when 5 => s_fx1 <= s_rx(4 downto 0) & "00";
                when 6 => s_fx1 <= s_rx(5 downto 0) & "0";
                when 7 => s_fx1 <= s_rx(6 downto 0);
                when 8 => s_fx1 <= s_rx(7 downto 1);
                when 9 => s_fx1 <= s_rx(8 downto 2);
                when others => s_fx1 <= s_rx(9 downto 3);
            end case;
            s_ka <= s_rk0  + resize(s_cx1, 16);
            s_kb <= s_rk0p + resize(s_cx1, 16);
            s_kc <= s_rk1  + resize(s_cx1, 16);
            s_kd <= s_rk1p + resize(s_cx1, 16);
            s_xd(2) <= s_x1;
            for i in 3 to 15 loop s_xd(i) <= s_xd(i - 1); end loop;
            if s_sw_blocky = '1' then s_fxd(2) <= (others => '0'); else s_fxd(2) <= s_fx1; end if;
            for i in 3 to 7 loop s_fxd(i) <= s_fxd(i - 1); end loop;
            s_ha3 <= f_round1(s_ka); s_hb3 <= f_round1(s_kb); s_hc3 <= f_round1(s_kc); s_hd3 <= f_round1(s_kd);
            s_ha4 <= f_round2(s_ha3); s_hb4 <= f_round2(s_hb3); s_hc4 <= f_round2(s_hc3); s_hd4 <= f_round2(s_hd3);
            s_ha5 <= f_round3(s_ha4); s_hb5 <= f_round3(s_hb4); s_hc5 <= f_round3(s_hc4); s_hd5 <= f_round3(s_hd4);
            s_a6 <= s_ha5(15 downto 8); s_b6 <= s_hb5(15 downto 8); s_c6 <= s_hc5(15 downto 8); s_d6 <= s_hd5(15 downto 8);
            s_dab7 <= signed('0' & s_b6) - signed('0' & s_a6);
            s_dcd7 <= signed('0' & s_d6) - signed('0' & s_c6);
            s_a7 <= s_a6; s_c7 <= s_c6;
            s_pab8 <= s_dab7 * signed('0' & s_fxd(7));
            s_pcd8 <= s_dcd7 * signed('0' & s_fxd(7));
            s_a8 <= s_a7; s_c8 <= s_c7;
            v_t := signed(resize(s_a8, 10) & '0') + resize(shift_right(s_pab8, 6), 11);
            v_b := signed(resize(s_c8, 10) & '0') + resize(shift_right(s_pcd8, 6), 11);
            s_top9 <= unsigned(v_t(8 downto 1));
            s_bot9 <= unsigned(v_b(8 downto 1));
            s_dtb10 <= signed('0' & s_bot9) - signed('0' & s_top9);
            s_top10 <= s_top9;
            if s_sw_blocky = '1' then s_ptb11 <= (others => '0'); else s_ptb11 <= s_dtb10 * signed('0' & s_fy); end if;
            s_top11 <= s_top10;
            v_t := signed(resize(s_top11, 10) & '0') + resize(shift_right(s_ptb11, 6), 11);
            s_n12 <= unsigned(v_t(8 downto 1));
            s_nc13 <= signed('0' & s_n12) - to_signed(128, 9);
            s_d14 <= s_nc13 * signed('0' & s_amp);
            if s_sw_steer = '1' then s_off15 <= (others => '0'); else s_off15 <= s_d14(16 downto 7); end if;
            s_steer <= s_n12(7);
            s_u16 <= signed(resize(s_xd(15), 13)) + resize(s_off15, 13);
            s_addr17 <= f_edge_addr(s_u16, s_w_edge);
        end if;
    end process p_noise;

    --------------------------------------------------------------------------
    -- Line buffers: written at stage 9 of the delayed stream, read at 17 -> 18
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr17));
            if s_we9 = '1' and s_wp9 = '0' then lbY0(to_integer(s_wcnt)) <= std_logic_vector(s_y9); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr17));
            if s_we9 = '1' and s_wp9 = '1' then lbY1(to_integer(s_wcnt)) <= std_logic_vector(s_y9); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr17(10 downto 1)));
            if s_we9 = '1' and s_wp9 = '0' then lbU0(to_integer(s_wcnt(10 downto 1))) <= std_logic_vector(s_u9(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr17(10 downto 1)));
            if s_we9 = '1' and s_wp9 = '1' then lbU1(to_integer(s_wcnt(10 downto 1))) <= std_logic_vector(s_u9(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr17(10 downto 1)));
            if s_we9 = '1' and s_wp9 = '0' then lbV0(to_integer(s_wcnt(10 downto 1))) <= std_logic_vector(s_v9(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr17(10 downto 1)));
            if s_we9 = '1' and s_wp9 = '1' then lbV1(to_integer(s_wcnt(10 downto 1))) <= std_logic_vector(s_v9(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fetch + gate (stages 18..30)
    --------------------------------------------------------------------------
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then
                s_y19 <= unsigned(s_rdY1); s_u19 <= unsigned(s_rdU1) & "00"; s_v19 <= unsigned(s_rdV1) & "00";
            else
                s_y19 <= unsigned(s_rdY0); s_u19 <= unsigned(s_rdU0) & "00"; s_v19 <= unsigned(s_rdV0) & "00";
            end if;
            s_yd(20) <= s_y19; s_ud(20) <= s_u19; s_vd(20) <= s_v19;
            for i in 21 to 28 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            if s_avid_sr(27) = '1' then
                s_y29 <= s_yd(28); s_u29 <= s_ud(28); s_v29 <= s_vd(28);
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
            s_y30 <= s_y29; s_u30 <= s_u29; s_v30 <= s_v29;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y30);
    data_out.u       <= std_logic_vector(s_u30);
    data_out.v       <= std_logic_vector(s_v30);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture impasto;
