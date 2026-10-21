-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Medallion v0.1.0 -- Heirloom #12 (2026-10-20), from WIN_20230305_23_51_38.
--
-- The photo: orange / yellow / red horizontal stripes folded into two nested
-- chevron systems (apexes near the left and right edges), and a central
-- circle -- a porthole -- showing a hard CMY / gray plaid.
--
-- The program: a stripe phase that rises down the screen and bends with the
-- distance to the nearer of two chevron centres (so every stripe is a V),
-- painted from a warm palette; a circle in the middle shows the video
-- (or the internal plaid oscillator) posterized hard.
--
--   K1  Pitch    stripe period
--   K2  Slope    chevron slope (flat stripes .. steep V's)
--   K3  Spread   chevron centres: at the edges .. together in the middle
--   K4  Palette  Warm / Cool / Rainbow
--   K5  Offset   circle position, bipolar
--   K6  Speed    stripe scroll, bipolar (centre = still)
--   S7  Plaid    Hard (CMY, 8 lumas) / Soft (4 x 4 chroma) posterize
--   S8  Bands    8 / 4 stripe levels
--   S9  Point    chevrons point outward / inward
--   S10 Ring     white rim on the circle
--   S11 Source   Video / Osc (an internal plaid oscillator replaces the video)
--   P12 MEDALLION  circle radius: a coin .. the whole screen
--
-- Virtual 1080-line geometry; dx^2 by second differences, dy^2 / R^2 /
-- per-frame products on one serial multiplier.  Generated avid; output
-- conditioning before the blanking gate.  Latency 20 clocks.  16 EBR.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture medallion of program_top is
    constant C_LAT : integer := 20;

    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);
            when 4          => return to_unsigned(576, 12);
            when 8          => return to_unsigned(960, 12);
            when 12         => return to_unsigned(480, 12);
            when 5 | 6 | 14 => return to_unsigned(384, 12);
            when 1 | 2 | 10 => return to_unsigned(512, 12);
            when others     => return to_unsigned(256, 12);
        end case;
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- stripe palettes: 3 x 8 {y, u = Cr, v = Cb}
    type t_col is array (0 to 2) of integer;
    type t_pdef is array (0 to 23) of t_col;
    constant C_PDEF : t_pdef := (
        (300, 800, 400), (328, 960, 360), (584, 772, 212), (840, 584, 64), (700, 700, 150), (584, 772, 212), (328, 960, 360), (220, 640, 460),   -- warm
        (200, 440, 760), (164, 440, 960), (400, 300, 800), (678, 64, 663), (500, 200, 760), (400, 300, 800), (164, 440, 960), (220, 480, 640),   -- cool
        (328, 960, 360), (584, 772, 212), (840, 584, 64), (580, 136, 216), (678, 64, 663), (164, 440, 960), (349, 756, 852), (426, 887, 808));   -- rainbow
    function f_pal(i : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(C_PDEF(i)(0), 10)) & std_logic_vector(to_unsigned(C_PDEF(i)(1), 10))
               & std_logic_vector(to_unsigned(C_PDEF(i)(2), 10));
    end function;
    type t_pal is array (0 to 31) of std_logic_vector(29 downto 0);
    function f_mkpal return t_pal is
        variable r : t_pal;
    begin
        for i in 0 to 31 loop
            if i < 24 then r(i) := f_pal(i); else r(i) := f_pal(i - 8); end if;
        end loop;
        return r;
    end function;
    constant C_PAL : t_pal := f_mkpal;

    -- line buffer {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- oscillator source (a CMY plaid from square waves)
    signal s_src : std_logic := '0';
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- timing / controls
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_fs : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s_sl : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_k3 : unsigned(9 downto 0) := (others => '0');
    signal s_j4 : unsigned(1 downto 0) := (others => '0');
    signal s_live4 : std_logic := '0';
    signal s_k5m : unsigned(8 downto 0) := (others => '0');
    signal s_k5s : std_logic := '0';
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_pals : unsigned(1 downto 0) := (others => '0');
    signal s_hard, s_b4, s_inward, s_ring : std_logic := '0';
    signal s_r : unsigned(10 downto 0) := to_unsigned(400, 11);
    signal s_r2 : unsigned(21 downto 0) := (others => '0');
    signal s_r2r : unsigned(22 downto 0) := (others => '0');          -- R^2 + ring
    signal s_cxo : unsigned(11 downto 0) := to_unsigned(960, 12);     -- circle centre x (virtual)
    signal s_cx2 : unsigned(12 downto 0) := to_unsigned(1920, 13);    -- 2 cx
    signal s_cx2sq : unsigned(25 downto 0) := (others => '0');
    signal s_inc0 : signed(15 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_sp : unsigned(11 downto 0) := to_unsigned(64, 12);       -- chevron centre 1 (virtual)
    signal s_c1q, s_c2q, s_cmq : unsigned(12 downto 0) := (others => '0');   -- Q.1 crossing points
    signal s_da : unsigned(9 downto 0) := to_unsigned(256, 10);       -- phase per pixel (sl * xs)
    signal s_a0 : unsigned(19 downto 0) := (others => '0');           -- 2 sl c1
    signal s_lph : unsigned(15 downto 0) := (others => '0');          -- scroll phase
    signal s_off : unsigned(9 downto 0) := (others => '0');
    -- serial multiplier
    signal s_ma : unsigned(25 downto 0) := (others => '0');
    signal s_mb : unsigned(12 downto 0) := (others => '0');
    signal s_macc : unsigned(25 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1, s_fall2 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_ady_n : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_dy2_n, s_dy2 : unsigned(19 downto 0) := to_unsigned(291600, 20);
    signal s_lf_n, s_lf : unsigned(15 downto 0) := (others => '0');   -- line stripe phase

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_vxa : unsigned(12 downto 0) := (others => '0');
    signal s_a : unsigned(15 downto 0) := (others => '0');
    signal s_dir : std_logic := '0';                                  -- '1' = a rising
    signal s_q : unsigned(25 downto 0) := (others => '0');
    signal s_inc : signed(15 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_f1 : unsigned(15 downto 0) := (others => '0');
    signal s_q1 : unsigned(25 downto 0) := (others => '0');
    signal s_rdd : t_u11(1 to 6) := (others => (others => '0'));
    signal s_idx2 : unsigned(2 downto 0) := (others => '0');
    signal s_qi2 : unsigned(23 downto 0) := (others => '0');
    signal s_idx3 : unsigned(2 downto 0) := (others => '0');
    signal s_r2_3 : unsigned(23 downto 0) := (others => '0');
    signal s_in4, s_rg4 : std_logic := '0';
    signal s_sy4, s_su4, s_sv4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_in : t_sl(5 to 10) := (others => '0');
    signal s_rg : t_sl(5 to 10) := (others => '0');
    signal s_syd, s_sud, s_svd : t_u10(5 to 10) := (others => to_unsigned(64, 10));
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y11, s_y12, s_y13, s_y14, s_y15, s_y16x, s_yf16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17, s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17, s_u18, s_v18, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_i : signed(16 downto 0);
        variable v_c : unsigned(12 downto 0);
        variable v_p : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 127 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- knobs
            s_fs <= resize(unsigned(registers_in(0)(9 downto 4)), 7) + 4;      -- K1 Pitch 4 .. 67
            s_sl <= unsigned(registers_in(1)(9 downto 2));                     -- K2 Slope
            s_k3 <= unsigned(registers_in(2));                                 -- K3 Spread
            v_k := unsigned(registers_in(3));                                  -- K4 Palette (3 labels)
            if v_k >= 3 then s_live4 <= '1'; end if;
            if s_live4 = '0' and v_k < 3 then
                s_j4 <= v_k(1 downto 0);
            elsif v_k < 341 then s_j4 <= "00";
            elsif v_k < 683 then s_j4 <= "01";
            else s_j4 <= "10"; end if;
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;        -- K5 Offset, deadband
            if v_s > -8 and v_s < 8 then v_s := (others => '0'); end if;
            s_k5s <= v_s(10);
            if v_s < 0 then s_k5m <= unsigned(resize(-v_s, 9)); else s_k5m <= unsigned(resize(v_s, 9)); end if;
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;        -- K6 Speed, deadband
            if v_s > -8 and v_s < 8 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_pals <= s_j4;
                s_hard   <= not registers_in(6)(0);
                s_b4     <= registers_in(6)(1);
                s_inward <= registers_in(6)(2);
                s_ring   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_lph <= s_lph + unsigned(resize(shift_left(resize(s_k6r, 16), 2), 16));
            end if;

            -- derived
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            -- radius 96 + 0.9 p12 (max ~1016)
            v_p := to_unsigned(96, 17) + resize(s_g(11 downto 2), 17) - resize(s_g(11 downto 6), 17) - resize(s_g(11 downto 7), 17);
            s_r <= v_p(10 downto 0);
            s_r2r <= resize(s_r2, 23) + resize(s_r2(21 downto 4), 23);
            -- circle centre: cx +- off
            if s_k5s = '1' then v_c := resize(s_vw(11 downto 1), 13) - resize(s_off, 13);
            else v_c := resize(s_vw(11 downto 1), 13) + resize(s_off, 13); end if;
            s_cxo <= v_c(11 downto 0);
            s_cx2 <= s_cxo & '0';
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 17) - signed(shift_left(resize(s_cx2, 17), 3));
                               s_inc2 <= to_unsigned(32, 6);
                               s_da <= shift_left(resize(s_sl, 10), 2);
                when 3      => v_i := to_signed(9, 17) - signed(shift_left(resize(s_cx2, 17), 2) + shift_left(resize(s_cx2, 17), 1));
                               s_inc2 <= to_unsigned(18, 6);
                               s_da <= shift_left(resize(s_sl, 10), 1) + resize(s_sl, 10);
                when others => v_i := to_signed(4, 17) - signed(shift_left(resize(s_cx2, 17), 2));
                               s_inc2 <= to_unsigned(8, 6);
                               s_da <= shift_left(resize(s_sl, 10), 1);
            end case;
            s_inc0 <= v_i(15 downto 0);
            -- chevron centres (Q.1): c1 = sp, c2 = vw - sp, mid = vw / 2
            s_c1q <= s_sp & '0';
            s_c2q <= (s_vw - s_sp) & '0';
            s_cmq <= '0' & s_vw;

            -- serial unit: 0 (2cx)^2, 1 R^2, 2 ady^2, 3 off = k5m * vw/2 >> 10,
            -- 4 sp = 64 + k3 * (vw/2 - 64) >> 10, 5 a0 = 2 sl c1
            if s_zc = 2 then
                s_ma <= resize(s_k5m, 26); s_mb <= resize(s_vw(11 downto 1), 13); s_mjob <= "011"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 18 then
                s_ma <= resize(s_cx2, 26); s_mb <= s_cx2; s_mjob <= "000"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 34 then
                s_ma <= resize(s_r, 26); s_mb <= resize(s_r, 13); s_mjob <= "001"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 50 then
                s_ma <= resize(s_k3, 26); s_mb <= resize(s_vw(11 downto 1), 13) - 64; s_mjob <= "100"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 66 then
                s_ma <= resize(s_sl & '0', 26); s_mb <= resize(s_sp, 13); s_mjob <= "101"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 82 or s_fall2 = '1' then
                s_ma <= resize(s_ady_n, 26); s_mb <= resize(s_ady_n, 13); s_mjob <= "010"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(24 downto 0) & '0';
                s_mb <= '0' & s_mb(12 downto 1);
                if s_mi = 12 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 12 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_cx2sq <= s_macc;
                    when "001"  => s_r2 <= s_macc(21 downto 0);
                    when "010"  => s_dy2_n <= s_macc(19 downto 0);
                    when "011"  => s_off <= s_macc(19 downto 10);
                    when "100"  => s_sp <= to_unsigned(64, 12) + resize(s_macc(21 downto 10), 12);
                    when others => s_a0 <= s_macc(19 downto 0);
                end case;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid
    ---------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;
            if g_hsr = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if g_avr = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;
            if g_start = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Source side: plaid oscillator, line-buffer write, per-line values
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_dy : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 2; end if;
            -- plaid: square waves of x and y pick CMY / gray cells
            if (s_ox(8) xor s_oy(8) xor s_ot(3)) = '1' then s_oty <= to_unsigned(760, 10); else s_oty <= to_unsigned(300, 10); end if;
            if (s_ox(7) xor s_ot(2)) = '1' then s_otu <= to_unsigned(704, 10); else s_otu <= to_unsigned(320, 10); end if;
            if (s_oy(7) xor s_ot(4)) = '1' then s_otv <= to_unsigned(704, 10); else s_otv <= to_unsigned(320, 10); end if;
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_oty) & std_logic_vector(s_otu) & std_logic_vector(s_otv) & "00";
            else
                s_lb_wd <= data_in.y & data_in.u & data_in.v & "00";
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;

            s_fall1 <= '0';
            if data_in.avid = '0' and s_avq = '1' then s_fall1 <= '1'; end if;
            s_fall2 <= s_fall1;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_ady_n <= to_unsigned(540, 11);
                s_lf_n <= s_lph;
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_lf_n <= s_lf_n + resize(s_fs & "0000", 16);
            elsif s_fall2 = '1' then
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - 540;
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11);
                else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_dy2 <= s_dy2_n;
                s_lf <= s_lf_n;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: virtual x, chevron phase a = 2 sl * dist(nearest centre), dx^2
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= (others => '0');
                s_vxa <= (others => '0');
                s_a <= s_a0(15 downto 0);
                s_dir <= '0';
                s_q <= s_cx2sq;
                s_inc <= s_inc0;
            elsif g_avid = '1' then
                s_rdp <= s_rdp + 1;
                s_vxa <= s_vxa + s_xs;
                -- direction: falling before c1, rising to mid, falling to c2, rising after
                if s_vxa < s_c1q then s_dir <= '0';
                elsif s_vxa < s_cmq then s_dir <= '1';
                elsif s_vxa < s_c2q then s_dir <= '0';
                else s_dir <= '1'; end if;
                if s_dir = '1' then s_a <= s_a + s_da; else s_a <= s_a - s_da; end if;
                s_q <= unsigned(signed(s_q) + resize(s_inc, 26));
                s_inc <= s_inc + signed(resize(s_inc2, 16));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_c : unsigned(10 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_pi : integer range 0 to 31;
    begin
        if rising_edge(clk) then
            -- 1: stripe phase (line phase +- chevron term), dx^2 copy
            if s_inward = '1' then s_f1 <= s_lf - s_a; else s_f1 <= s_lf + s_a; end if;
            s_q1 <= s_q;
            s_rdd(1) <= s_rdp;

            -- 2: band index; dx^2 integer
            if s_b4 = '1' then s_idx2 <= s_f1(15 downto 14) & '0'; else s_idx2 <= s_f1(15 downto 13); end if;
            s_qi2 <= s_q1(25 downto 2);

            -- 3: r^2
            s_r2_3 <= s_qi2 + resize(s_dy2, 24);
            s_idx3 <= s_idx2;

            -- 4: inside / ring; stripe colour (LUT palette)
            if s_r2_3 <= resize(s_r2, 24) then s_in4 <= '1'; else s_in4 <= '0'; end if;
            if s_r2_3 > resize(s_r2, 24) and s_r2_3 <= resize(s_r2r, 24) then s_rg4 <= '1'; else s_rg4 <= '0'; end if;
            v_pi := to_integer(s_pals & s_idx3);
            s_sy4 <= unsigned(C_PAL(v_pi)(29 downto 20));
            s_su4 <= unsigned(C_PAL(v_pi)(19 downto 10));
            s_sv4 <= unsigned(C_PAL(v_pi)(9 downto 0));

            -- 5..10 delays of the flags / stripe colour
            s_in(5) <= s_in4; s_rg(5) <= s_rg4;
            s_syd(5) <= s_sy4; s_sud(5) <= s_su4; s_svd(5) <= s_sv4;

            -- 6: video read; 7: EBR output; 8: fabric copy
            s_lb_ra <= s_rdd(5);
            s_y8 <= unsigned(s_lb_q(31 downto 22));
            s_u8 <= unsigned(s_lb_q(21 downto 12));
            s_v8 <= unsigned(s_lb_q(11 downto 2));

            -- 9: posterize (hard: 8 lumas, 1-bit chroma; soft: 4 x 4 chroma)
            if s_y8 > 64 then v_c := resize(s_y8 - 64, 11); else v_c := (others => '0'); end if;
            s_y9 <= resize(v_c(9 downto 7) & "0000000", 10) + 64;
            if s_hard = '1' then
                if s_u8(9) = '1' then s_u9 <= to_unsigned(704, 10); else s_u9 <= to_unsigned(320, 10); end if;
                if s_v8(9) = '1' then s_v9 <= to_unsigned(704, 10); else s_v9 <= to_unsigned(320, 10); end if;
            else
                v_c := resize(s_u8, 11) + 64;
                if v_c(10) = '1' then s_u9 <= to_unsigned(960, 10); else s_u9 <= v_c(9 downto 7) & "0000000"; end if;
                v_c := resize(s_v8, 11) + 64;
                if v_c(10) = '1' then s_v9 <= to_unsigned(960, 10); else s_v9 <= v_c(9 downto 7) & "0000000"; end if;
            end if;

            -- 10: compose
            if s_ring = '1' and s_rg(9) = '1' then
                s_y10 <= to_unsigned(940, 10); s_u10 <= to_unsigned(512, 10); s_v10 <= to_unsigned(512, 10);
            elsif s_in(9) = '1' then
                s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            else
                s_y10 <= s_syd(9); s_u10 <= s_sud(9); s_v10 <= s_svd(9);
            end if;

            -- 11..16: output conditioning (centre at stage 15 -> 16)
            s_csu(0) <= s_u10; s_csv(0) <= s_v10;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u10 - s_csu(7);
            s_boxv <= s_boxv + s_v10 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u16 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v16 <= v_sc(13 downto 4);
            s_y11 <= s_y10; s_y12 <= s_y11; s_y13 <= s_y12; s_y14 <= s_y13;
            s_y15 <= s_y14; s_y16x <= s_y15;
            v_sy := resize(s_y14, 12) + shift_left(resize(s_y15, 12), 1) + resize(s_y16x, 12);
            s_yf16 <= v_sy(11 downto 2);
            -- 17: luma toward black; blanking gate
            if s_yf16 > 64 then v_dl := s_yf16 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(15) = '1' then
                s_y17 <= s_yf16 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            -- 18..20
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            -- delays
            for i in 6 to 10 loop
                s_in(i) <= s_in(i - 1); s_rg(i) <= s_rg(i - 1);
                s_syd(i) <= s_syd(i - 1); s_sud(i) <= s_sud(i - 1); s_svd(i) <= s_svd(i - 1);
            end loop;
            for i in 2 to 6 loop s_rdd(i) <= s_rdd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture medallion;
