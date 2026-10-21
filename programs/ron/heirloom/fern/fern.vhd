-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Fern v0.1.0 -- Heirloom #46 (2026-10-20), from WIN_20230713_00_17_11.
--
-- The photo: mirrored fronds -- nested pink / orange / yellow arcs pointing
-- right (parabola bands) over a fine teal / green ripple field, a flat
-- blue zone at the left with a small chevron.
--
-- The program: arcs = bands of x - a (y - cy)^2 (nested parabolas opening
-- to the right, mirrored about the centre line) painted pink / orange /
-- yellow by band; between them a teal ripple field (or the source tinted
-- teal), whose luma widens the arcs; flat blue left of a boundary.
--
--   K1  Pitch    arc spacing
--   K2  Duty     arc band width within the pitch
--   K3  Ripple   ripple pitch of the field between the arcs
--   K4  Hue      arc palette rotation
--   K5  Edge     the blue flat zone's boundary
--   K6  Drift    the arcs flow outward / inward, bipolar (centre = still)
--   S7  Mirror   mirrored about the centre line / parabolas from the top
--   S8  Palette  Flame / Ice
--   S9  Field    Ripple / Source (the picture tinted teal, widening the arcs)
--   S10 Blue     the flat blue zone on / off
--   S11 Source   Video / Osc
--   P12 REACH    curvature: near-vertical bars .. tight nested parabolas
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

architecture fern of program_top is
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

    type t_pal is array (0 to 15) of std_logic_vector(31 downto 0);
    constant C_PAL : t_pal := (
        f_col(752, 612, 512), f_col(584, 772, 212), f_col(840, 584, 64), f_col(426, 887, 808),
        f_col(328, 960, 360), f_col(752, 612, 512), f_col(840, 584, 64), f_col(584, 772, 212),
        f_col(678, 64, 663), f_col(164, 440, 960), f_col(940, 512, 512), f_col(349, 756, 852),
        f_col(192, 608, 696), f_col(678, 64, 663), f_col(740, 470, 600), f_col(164, 440, 960));
    signal s_pal_ra : unsigned(3 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    signal s_pk : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_duty : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal s_fx : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_hue : unsigned(2 downto 0) := (others => '0');
    signal s_edge : unsigned(9 downto 0) := (others => '0');
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_mir, s_ice, s_fsrc, s_blue : std_logic := '0';
    -- derived
    signal s_fxs : unsigned(9 downto 0) := (others => '0');
    signal s_ak : unsigned(9 downto 0) := (others => '0');              -- curvature (P12)
    signal s_ph : unsigned(15 downto 0) := (others => '0');
    signal s_edv : unsigned(11 downto 0) := (others => '0');            -- edge x (virtual)
    -- per line
    signal s_dq_n, s_dq : unsigned(23 downto 0) := (others => '0');     -- a (y - cy)^2 (or a y^2)
    signal s_ady2_n : unsigned(10 downto 0) := (others => '0');
    signal s_py : unsigned(9 downto 0) := (others => '0');
    -- beam
    signal s_px : unsigned(11 downto 0) := (others => '0');
    -- pixel
    signal s_f1 : signed(24 downto 0) := (others => '0');
    signal s_r1 : unsigned(7 downto 0) := (others => '0');
    signal s_bl1 : std_logic := '0';
    signal s_f2 : unsigned(24 downto 0) := (others => '0');
    signal s_t2 : unsigned(6 downto 0) := (others => '0');
    signal s_bl : t_sl(2 to 11) := (others => '0');
    signal s_frac3 : unsigned(7 downto 0) := (others => '0');
    signal s_idx3, s_idx4 : unsigned(2 downto 0) := (others => '0');
    signal s_rip3 : unsigned(6 downto 0) := (others => '0');
    signal s_d4 : unsigned(8 downto 0) := (others => '0');
    signal s_frac4 : unsigned(7 downto 0) := (others => '0');
    signal s_rip4 : unsigned(6 downto 0) := (others => '0');
    signal s_arc : t_sl(5 to 11) := (others => '0');
    signal s_ripd : t_u10(5 to 11) := (others => (others => '0'));
    signal s_py7, s_pu7, s_pv7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_pyd, s_pud, s_pvd : t_u10(8 to 11) := (others => to_unsigned(64, 10));
    signal s_yd, s_ud, s_vd : t_u10(4 to 11) := (others => to_unsigned(64, 10));

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
            s_pk <= resize(unsigned(registers_in(0)(9 downto 7)), 4) + 4;      -- band shift 4 .. 11
            s_duty <= unsigned(registers_in(1)(9 downto 2));
            s_fx <= resize(unsigned(registers_in(2)(9 downto 3)), 8) + 2;
            s_hue <= unsigned(registers_in(3)(9 downto 7));
            s_edge <= unsigned(registers_in(4));
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;
            if v_s > -8 and v_s < 8 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;

            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mir  <= registers_in(6)(0);
                s_ice  <= registers_in(6)(1);
                s_fsrc <= registers_in(6)(2);
                s_blue <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_ph <= s_ph + unsigned(resize(shift_left(resize(s_k6r, 16), 2), 16));

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
            case to_integer(s_xs) is
                when 4      => s_fxs <= shift_left(resize(s_fx, 10), 1);
                when 3      => s_fxs <= resize(s_fx, 10) + resize(s_fx(7 downto 1), 10);
                when others => s_fxs <= resize(s_fx, 10);
            end case;
            s_ak <= s_g10;
            s_edv <= shift_left(resize(s_edge(9 downto 2), 12), 2);


            -- serial multiplier: per-frame jobs in the zone, per-line jobs at
            -- the source avid fall (+2 and +18 clocks)
            if s_zc = 4 or s_fall2 = '1' then
                s_ma <= resize(s_ady2_n, 24); s_mb <= resize(s_ady2_n, 12); s_macc <= (others => '0');
                s_mjob <= "000"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000" => s_dq_n <= s_macc;
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
        variable v_q : unsigned(33 downto 0);

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
                s_ady2_n <= to_unsigned(540, 11);
                s_py <= (others => '0');

            elsif s_fall1 = '1' then
                v_vy := s_vy_n + s_ys;
                s_vy_n <= v_vy;
                v_dy := signed(resize(v_vy(19 downto 8), 13)) - 540;
                s_dy_n <= v_dy(11 downto 0);
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11); else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
                s_py <= s_py + 5;
                if s_mir = '1' then
                    if v_dy < 0 then s_ady2_n <= resize(unsigned(-v_dy), 11); else s_ady2_n <= unsigned(v_dy(10 downto 0)); end if;
                else
                    s_ady2_n <= v_vy(18 downto 8);
                end if;

            end if;


            if data_in.avid = '1' and s_avq = '0' then
                s_vy <= s_vy_n(18 downto 8);
                s_dy <= s_dy_n;
                s_ady <= s_ady_n;
                -- a (y - cy)^2: dq (<= 291600 or 1166400) x ak / 2048, in virtual x units
                v_q := s_dq_n * s_ak;
                s_dq <= v_q(33 downto 10);

            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state (initialised on the g_start edge, advanced while g_avid)
    ---------------------------------------------------------------------------
    p_beam : process(clk)

    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
                s_vx <= (others => '0');
                s_dxh <= -signed(resize(s_vw, 14));
                s_px <= (others => '0');

            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_vx <= s_vx + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 14));
                s_px <= s_px + s_fxs;

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
        variable v_i : unsigned(8 downto 0);

    begin
        if rising_edge(clk) then
            s_lb_ra <= s_rdp;
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 1: field f = x - a (y - cy)^2 (virtual units, signed), ripple phase, blue zone
            s_f1 <= signed(resize(s_vx(12 downto 1), 25)) - signed(resize(s_dq, 25)) + signed(resize(s_ph, 25));
            s_r1 <= s_px(11 downto 4) + f_tri8(s_py(9 downto 2) + s_ph(7 downto 0));
            if s_blue = '1' and s_vx(12 downto 1) < s_edv then s_bl1 <= '1'; else s_bl1 <= '0'; end if;
            -- 2: band position (wrapping: the field is read modulo the pitch)
            s_f2 <= unsigned(s_f1);
            s_t2 <= f_tri8(s_r1);
            s_bl(2) <= s_bl1;
            -- 3: fraction within the band (8 bits) and band index, by the pitch shift
            case to_integer(s_pk) is
                when 4  => s_frac3 <= s_f2(3 downto 0) & "0000"; s_idx3 <= s_f2(6 downto 4);
                when 5  => s_frac3 <= s_f2(4 downto 0) & "000";  s_idx3 <= s_f2(7 downto 5);
                when 6  => s_frac3 <= s_f2(5 downto 0) & "00";   s_idx3 <= s_f2(8 downto 6);
                when 7  => s_frac3 <= s_f2(6 downto 0) & '0';    s_idx3 <= s_f2(9 downto 7);
                when 8  => s_frac3 <= s_f2(7 downto 0);          s_idx3 <= s_f2(10 downto 8);
                when 9  => s_frac3 <= s_f2(8 downto 1);          s_idx3 <= s_f2(11 downto 9);
                when 10 => s_frac3 <= s_f2(9 downto 2);          s_idx3 <= s_f2(12 downto 10);
                when others => s_frac3 <= s_f2(10 downto 3);     s_idx3 <= s_f2(13 downto 11);
            end case;
            s_rip3 <= s_t2;
            -- 4: duty widened by the source luma (Field = Source)
            if s_fsrc = '1' then v_i := resize(s_duty, 9) + resize(s_y3(9 downto 3), 9); else v_i := resize(s_duty, 9); end if;
            s_d4 <= v_i;
            s_frac4 <= s_frac3;
            s_rip4 <= s_rip3;
            s_idx4 <= s_idx3 + s_hue;
            s_yd(4) <= s_y3; s_ud(4) <= s_u3; s_vd(4) <= s_v3;
            -- 5: arc hit; ROM address
            if resize(s_frac4, 9) < s_d4 then s_arc(5) <= '1'; else s_arc(5) <= '0'; end if;
            s_pal_ra <= s_ice & s_idx4;
            s_ripd(5) <= resize(s_rip4, 10);
            -- 6: ROM register; 7: fabric copy
            s_py7 <= unsigned(s_pal_q(31 downto 22)); s_pu7 <= unsigned(s_pal_q(21 downto 12)); s_pv7 <= unsigned(s_pal_q(11 downto 2));
            -- 8 .. 11 copies
            s_pyd(8) <= s_py7; s_pud(8) <= s_pu7; s_pvd(8) <= s_pv7;
            -- 12: compose: blue zone, arcs, field (ripple teal / source tinted teal)
            if s_bl(11) = '1' then
                s_y12 <= to_unsigned(200, 10); s_u12 <= to_unsigned(440, 10); s_v12 <= to_unsigned(900, 10);
            elsif s_arc(11) = '1' then
                s_y12 <= s_pyd(11); s_u12 <= s_pud(11); s_v12 <= s_pvd(11);
            elsif s_fsrc = '1' then
                s_y12 <= s_yd(11); s_u12 <= to_unsigned(200, 10); s_v12 <= to_unsigned(600, 10);
            else
                s_y12 <= to_unsigned(200, 10) + shift_left(resize(s_ripd(11)(6 downto 0), 10), 2); s_u12 <= to_unsigned(200, 10); s_v12 <= to_unsigned(600, 10);
            end if;
            -- delays
            for i in 3 to 11 loop s_bl(i) <= s_bl(i - 1); end loop;
            for i in 6 to 11 loop s_arc(i) <= s_arc(i - 1); s_ripd(i) <= s_ripd(i - 1); end loop;
            for i in 9 to 11 loop s_pyd(i) <= s_pyd(i - 1); s_pud(i) <= s_pud(i - 1); s_pvd(i) <= s_pvd(i - 1); end loop;
            for i in 5 to 11 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;


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

end architecture fern;
