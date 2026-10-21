-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Crossbeam v0.1.0 -- Heirloom #37 (2026-10-20), from WIN_20230527_01_43.
--
-- The photo: a soft cyan-green X (two crossing diagonal bands) over a field
-- of Mondrian colour blocks (magenta / yellow / blue / green / black), red
-- ripple textures where the blocks are dark, everything torn sideways in
-- row bands.
--
-- The program: a block field (square cells, hashed colours from an 8-entry
-- palette, or the source posterized through the same palette), torn
-- sideways in 8-line bands; a red ripple texture inside the dark blocks;
-- two soft diagonal bands crossing at the centre, cyan with a luma gradient.
--
--   K1  Blocks   block size, 16 .. 271 virtual px
--   K2  Tear     sideways tearing of the blocks and the picture, 8-line bands
--   K3  Ripple   ripple pitch inside the dark blocks
--   K4  Hue      palette rotation (also rotates the source posterization)
--   K5  Angle    slope of the X
--   K6  Drift    the X slides up / down, bipolar (centre = still)
--   S7  Cross    X on / off
--   S8  Ripple   red ripple in the dark blocks on / off
--   S9  Palette  Mondrian / Candy
--   S10 Fill     Source (the picture posterized through the palette) / Flat (hashed blocks)
--   S11 Source   Video / Osc
--   P12 CROSS    X band width: hairline .. the whole screen glows cyan
--
-- Shared Heirloom skeleton (escalator / cocoon / starburst v0.2.0 lineage):
-- timing-ID steps, vblank zone, measured width, virtual 1080-line geometry,
-- one serial multiplier in the zone (and per line), generated avid (beam
-- counters initialised on the g_start edge), Source Video / Osc mux at the
-- line-buffer write, output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16,
-- luma [1 2 1], luma to ~777) before the blanking gate.  Latency 20 clocks
-- in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture crossbeam of program_top is
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
    -- triangle of an 8-bit phase -> 7 bits
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- clamp a 13-bit signed to the 64 .. 960 video range
    function f_clamp(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 960 then return to_unsigned(960, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;
    -- 8-bit saturating add
    function f_sat8(v : unsigned(8 downto 0)) return unsigned is
    begin
        if v(8) = '1' then return to_unsigned(255, 8); else return v(7 downto 0); end if;
    end function;
    -- colour record helpers: {y, u, v} packed 32 bits
    function f_col(y, u, v : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
               & std_logic_vector(to_unsigned(v, 10)) & "00";
    end function;

    -- line buffer {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- internal oscillator source (Source = Osc)
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
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);     -- virtual width = 2 cx
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');            -- glided P12, Q.2
    signal s_g10 : unsigned(9 downto 0) := (others => '0');
    -- serial multiplier
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
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
    signal s_fall1, s_fall2, s_fall3 : std_logic := '0';
    signal s_fcnt : unsigned(4 downto 0) := (others => '0');
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_dy_n, s_dy : signed(11 downto 0) := to_signed(-540, 12);
    signal s_ady_n, s_ady : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_vy : unsigned(10 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');          -- read start pointer

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_vx : unsigned(12 downto 0) := (others => '0');            -- Q.1 virtual x
    signal s_dxh : signed(13 downto 0) := (others => '0');             -- 2 (x - cx)

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y13, s_y14, s_y15, s_y16, s_y17, s_y18x, s_yf18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

    -- 2 palettes x 8 colours
    type t_pal is array (0 to 15) of std_logic_vector(31 downto 0);
    constant C_PAL : t_pal := (
        f_col(426, 887, 808), f_col(840, 584, 64), f_col(164, 440, 960), f_col(580, 136, 216),
        f_col(64, 512, 512), f_col(328, 960, 360), f_col(940, 512, 512), f_col(678, 64, 663),
        f_col(752, 612, 512), f_col(860, 540, 470), f_col(747, 516, 594), f_col(760, 420, 440),
        f_col(260, 512, 512), f_col(584, 772, 212), f_col(740, 470, 600), f_col(349, 756, 852));
    signal s_pal_ra : unsigned(3 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    signal s_k1 : unsigned(9 downto 0) := (others => '0');
    signal s_tk : unsigned(7 downto 0) := (others => '0');
    signal s_fx : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_hue : unsigned(2 downto 0) := (others => '0');
    signal s_ang : unsigned(8 downto 0) := to_unsigned(128, 9);
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_cross, s_ripple, s_palb, s_flat : std_logic := '0';
    -- derived
    signal s_cw : unsigned(8 downto 0) := to_unsigned(64, 9);       -- cell size virtual
    signal s_cw2 : unsigned(9 downto 0) := to_unsigned(128, 10);    -- Q.1
    signal s_cwq : unsigned(16 downto 0) := (others => '0');        -- << 8
    signal s_inca : unsigned(9 downto 0) := (others => '0');        -- a * xs / 2
    signal s_acx : unsigned(23 downto 0) := (others => '0');        -- a * cx
    signal s_drift : unsigned(19 downto 0) := (others => '0');
    signal s_wx : unsigned(15 downto 0) := (others => '0');         -- band half width
    signal s_seed : unsigned(7 downto 0) := x"5A";
    signal s_lfsr : unsigned(15 downto 0) := x"C3A5";
    -- per line
    signal s_cv_n : unsigned(18 downto 0) := (others => '0');
    signal s_cyi_n, s_cyi : unsigned(6 downto 0) := (others => '0');
    signal s_l8 : unsigned(2 downto 0) := (others => '0');
    signal s_tear_n : unsigned(10 downto 0) := (others => '0');
    signal s_mr : unsigned(10 downto 0) := (others => '0');         -- modulo loop remainder
    signal s_mq : unsigned(6 downto 0) := (others => '0');
    signal s_mgo : std_logic := '0';
    signal s_cx0, s_cx0c : unsigned(9 downto 0) := (others => '0');
    signal s_ci0, s_ci0c : unsigned(6 downto 0) := (others => '0');
    signal s_d1s_n, s_d2s_n, s_d1s, s_d2s : signed(23 downto 0) := (others => '0');
    -- beam
    signal s_cxa : unsigned(9 downto 0) := (others => '0');
    signal s_cxi : unsigned(6 downto 0) := (others => '0');
    signal s_d1, s_d2 : signed(23 downto 0) := (others => '0');
    signal s_px : unsigned(11 downto 0) := (others => '0');
    signal s_py : unsigned(9 downto 0) := (others => '0');
    -- pixel
    signal s_h1, s_h2, s_h3, s_h4 : unsigned(7 downto 0) := (others => '0');
    signal s_ad1, s_ad2 : unsigned(23 downto 0) := (others => '0');
    signal s_m1, s_m2 : signed(24 downto 0) := (others => '0');
    signal s_mm3 : signed(24 downto 0) := (others => '0');
    signal s_cl : t_u10(4 to 11) := (others => (others => '0'));
    signal s_tr2 : unsigned(6 downto 0) := (others => '0');
    signal s_rp : t_sl(3 to 10) := (others => '0');
    signal s_py6, s_pu6, s_pv6, s_py7, s_pu7, s_pv7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_pyd, s_pud, s_pvd : t_u10(8 to 11) := (others => to_unsigned(64, 10));

begin

    ---------------------------------------------------------------------------
    -- RAMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            s_pal_q <= C_PAL(to_integer(s_pal_ra));
        end if;
    end process p_pal;


    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);

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

            -- knobs (free-running decode)
            s_k1 <= unsigned(registers_in(0));
            s_tk <= unsigned(registers_in(1)(9 downto 2));
            s_fx <= resize(unsigned(registers_in(2)(9 downto 4)), 7) + 4;
            s_hue <= unsigned(registers_in(3)(9 downto 7));
            s_ang <= resize(unsigned(registers_in(4)(9 downto 2)), 9) + 32;
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;
            if v_s > -8 and v_s < 8 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;

            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_cross  <= registers_in(6)(0);
                s_ripple <= registers_in(6)(1);
                s_palb   <= registers_in(6)(2);
                s_flat   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_drift <= s_drift + unsigned(resize(shift_left(resize(s_k6r, 20), 4), 20));
                s_seed <= s_seed + 1;

            end if;

            -- derived
            s_g10 <= s_g(11 downto 2);
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cw <= resize(s_k1(9 downto 2), 9) + 16;
            s_cw2 <= s_cw & '0';
            s_cwq <= s_cw & x"00";
            case to_integer(s_xs) is
                when 4      => s_inca <= s_ang & '0';
                when 3      => s_inca <= resize(s_ang, 10) + resize(s_ang(8 downto 1), 10);
                when others => s_inca <= '0' & s_ang;
            end case;
            s_wx <= s_g10 & "000000";


            -- serial multiplier: per-frame jobs in the zone, per-line jobs at
            -- the source avid fall (+2 and +18 clocks)
            if s_zc = 4 then
                s_ma <= resize(s_ang, 24); s_mb <= '0' & s_vw(11 downto 1); s_macc <= (others => '0');
                s_mjob <= "000"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000" => s_acx <= s_macc;
                    when others => null;
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
    -- Generated avid (parity-locked start, field-latched width)
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
    -- Source side: oscillator, line-buffer write, per-line values (computed
    -- at the source avid fall, committed at its rise)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_vy : unsigned(19 downto 0);
        variable v_dy : signed(12 downto 0);
        variable v_c : unsigned(19 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_tr : unsigned(10 downto 0);

    begin
        if rising_edge(clk) then
            -- oscillator source: x ramp per source pixel, y ramp per line, phase per field
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_otu <= to_unsigned(320, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot) & '0', 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot), 10);
            s_otv <= to_unsigned(320, 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot), 10);
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
            s_fall3 <= '0';
            if s_fall2 = '1' then s_fcnt <= to_unsigned(1, 5);
            elsif s_fcnt /= 0 then
                s_fcnt <= s_fcnt + 1;
                if s_fcnt = 16 then s_fall3 <= '1'; s_fcnt <= (others => '0'); end if;
            end if;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_dy_n <= to_signed(-540, 12);
                s_ady_n <= to_unsigned(540, 11);
                s_cv_n <= (others => '0');
                s_cyi_n <= (others => '0');
                s_l8 <= (others => '0');
                s_tear_n <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

            elsif s_fall1 = '1' then
                v_vy := s_vy_n + s_ys;
                s_vy_n <= v_vy;
                v_dy := signed(resize(v_vy(19 downto 8), 13)) - 540;
                s_dy_n <= v_dy(11 downto 0);
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11); else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
                v_c := resize(s_cv_n, 20) + s_ys;
                if v_c >= s_cwq then s_cv_n <= resize(v_c - s_cwq, 19); s_cyi_n <= s_cyi_n + 1;
                else s_cv_n <= v_c(18 downto 0); end if;
                s_l8 <= s_l8 + 1;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                if s_l8 = 7 then
                    v_t := s_lfsr(7 downto 0) * s_tk;
                    v_tr := resize(v_t(15 downto 6), 11);
                    if v_tr >= s_wm1(10 downto 0) then s_tear_n <= s_wm1(10 downto 0); else s_tear_n <= v_tr; end if;
                end if;

            end if;
            -- tear offset into cell phase / cell index by repeated subtraction
            -- (offset < 1024 Q.1 units, cell >= 32 Q.1 units: <= 32 steps)
            if s_fall2 = '1' then
                s_mr <= resize(s_tear_n(9 downto 0), 11); s_mq <= (others => '0'); s_mgo <= '1';
            elsif s_mgo = '1' then
                if s_mr >= resize(s_cw2, 11) then s_mr <= s_mr - resize(s_cw2, 11); s_mq <= s_mq + 1;
                else s_mgo <= '0'; s_cx0 <= s_mr(9 downto 0); s_ci0 <= s_mq; end if;
            end if;
            -- X line starts: d = a (x - cx) -+ 256 dy + drift
            s_d1s_n <= signed(resize(s_drift, 24)) - signed(resize(s_acx, 24)) - shift_left(resize(s_dy_n, 24), 8);
            s_d2s_n <= signed(resize(s_drift, 24)) - signed(resize(s_acx, 24)) + shift_left(resize(s_dy_n, 24), 8);


            if data_in.avid = '1' and s_avq = '0' then
                s_vy <= s_vy_n(18 downto 8);
                s_dy <= s_dy_n;
                s_ady <= s_ady_n;
                s_cyi <= s_cyi_n;
                s_cx0c <= s_cx0; s_ci0c <= s_ci0;
                s_d1s <= s_d1s_n; s_d2s <= s_d2s_n;
                if s_tear_n = 0 then s_rst <= (others => '0'); else s_rst <= s_wm1(10 downto 0) + 1 - s_tear_n; end if;
                s_py <= s_py + 3;

            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state (initialised on the g_start edge, advanced while g_avid)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(10 downto 0);

    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
                s_vx <= (others => '0');
                s_dxh <= -signed(resize(s_vw, 14));
                s_cxa <= s_cx0c; s_cxi <= s_ci0c;
                s_d1 <= s_d1s; s_d2 <= s_d2s;
                s_px <= (others => '0');

            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_vx <= s_vx + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 14));
                v_n := resize(s_cxa, 11) + s_xs;
                if v_n >= resize(s_cw2, 11) then s_cxa <= resize(v_n - resize(s_cw2, 11), 10); s_cxi <= s_cxi + 1;
                else s_cxa <= v_n(9 downto 0); end if;
                s_d1 <= s_d1 + signed(resize(s_inca, 24));
                s_d2 <= s_d2 + signed(resize(s_inca, 24));
                s_px <= s_px + s_fx;

            end if;

        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = first registers off the beam state; the line
    -- buffer is read at stage 1, its output register is stage 2, s_y3/u3/v3
    -- the fabric copy at stage 3; the program composes s_y12/u12/v12)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_h : unsigned(13 downto 0);
        variable v_y : signed(12 downto 0);

    begin
        if rising_edge(clk) then
            s_lb_ra <= s_rdp;
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 1: cell hash seed, |d1|, |d2|
            v_h := resize(shift_left(resize(s_cxi, 14), 6), 14) - shift_left(resize(s_cxi, 14), 1) - resize(s_cxi, 14)
                   + shift_left(resize(s_cyi, 14), 7) - shift_left(resize(s_cyi, 14), 4) + resize(s_cyi, 14)
                   + resize(s_seed, 14);
            s_h1 <= v_h(7 downto 0);
            if s_d1 < 0 then s_ad1 <= unsigned(-s_d1); else s_ad1 <= unsigned(s_d1); end if;
            if s_d2 < 0 then s_ad2 <= unsigned(-s_d2); else s_ad2 <= unsigned(s_d2); end if;
            -- 2: hash mix; band margins; ripple triangle
            s_h2 <= s_h1 xor shift_right(s_h1, 3);
            s_m1 <= signed(resize(s_wx, 25)) - signed(resize(s_ad1, 25));
            s_m2 <= signed(resize(s_wx, 25)) - signed(resize(s_ad2, 25));
            s_tr2 <= f_tri8(s_px(11 downto 4) + f_tri8(s_py(9 downto 2)) + s_py(9 downto 2));
            -- 3: hash mix; nearer band
            s_h3 <= s_h2 + shift_left(s_h2, 2) + 7;
            if s_m1 > s_m2 then s_mm3 <= s_m1; else s_mm3 <= s_m2; end if;
            if s_tr2 > 80 then s_rp(3) <= '1'; else s_rp(3) <= '0'; end if;
            -- 4: palette address (flat: hash; source: luma posterized) + hue; cross level
            s_h4 <= s_h3 xor shift_right(s_h3, 2);
            if s_flat = '1' then
                s_pal_ra <= s_palb & (s_h4(7 downto 5) + s_hue);
            else
                s_pal_ra <= s_palb & (s_y3(9 downto 7) + s_hue);
            end if;
            if s_mm3 <= 0 then s_cl(4) <= (others => '0');
            elsif s_mm3 >= 65535 then s_cl(4) <= (others => '1');
            else s_cl(4) <= unsigned(s_mm3(15 downto 6)); end if;
            -- 5: ROM register; 6: fabric copy
            s_py6 <= unsigned(s_pal_q(31 downto 22)); s_pu6 <= unsigned(s_pal_q(21 downto 12)); s_pv6 <= unsigned(s_pal_q(11 downto 2));
            -- 7: ripple in the dark blocks
            if s_ripple = '1' and s_rp(6) = '1' and s_py6 < 300 then
                s_py7 <= to_unsigned(328, 10); s_pu7 <= to_unsigned(960, 10); s_pv7 <= to_unsigned(360, 10);
            else
                s_py7 <= s_py6; s_pu7 <= s_pu6; s_pv7 <= s_pv6;
            end if;
            -- 8 .. 11: copies
            s_pyd(8) <= s_py7; s_pud(8) <= s_pu7; s_pvd(8) <= s_pv7;
            for i in 9 to 11 loop s_pyd(i) <= s_pyd(i - 1); s_pud(i) <= s_pud(i - 1); s_pvd(i) <= s_pvd(i - 1); end loop;
            -- 12: cross overlay (soft: inner core solid cyan, rim half-mixed, outside untouched)
            if s_cross = '1' and s_cl(11) >= 256 then
                v_y := signed(resize(s_cl(11), 13)) + 300;
                s_y12 <= f_clamp(v_y); s_u12 <= to_unsigned(64, 10); s_v12 <= to_unsigned(663, 10);
            elsif s_cross = '1' and s_cl(11) /= 0 then
                v_y := signed(resize(s_pyd(11), 13)) + signed(resize(s_cl(11), 13));
                s_y12 <= f_clamp(v_y);
                s_u12 <= resize(shift_right(resize(s_pud(11), 11) + 64, 1), 10);
                s_v12 <= resize(shift_right(resize(s_pvd(11), 11) + 663, 1), 10);
            else
                s_y12 <= s_pyd(11); s_u12 <= s_pud(11); s_v12 <= s_pvd(11);
            end if;
            -- delays
            for i in 5 to 11 loop s_cl(i) <= s_cl(i - 1); end loop;
            for i in 4 to 10 loop s_rp(i) <= s_rp(i - 1); end loop;


            -- 13..18: output conditioning (centre at stage 17 -> 18)
            s_csu(0) <= s_u12; s_csv(0) <= s_v12;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u12 - s_csu(7);
            s_boxv <= s_boxv + s_v12 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v18 <= v_sc(13 downto 4);
            s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14; s_y16 <= s_y15;
            s_y17 <= s_y16; s_y18x <= s_y17;
            v_sy := resize(s_y16, 12) + shift_left(resize(s_y17, 12), 1) + resize(s_y18x, 12);
            s_yf18 <= v_sy(11 downto 2);
            -- 19: luma toward black; blanking gate
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(17) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            -- 20
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture crossbeam;
