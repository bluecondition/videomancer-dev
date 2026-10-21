-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Facet v0.1.0 -- Heirloom #40 (2026-10-20), from WIN_20230602_01_26.
--
-- The photo: flat-shaded geometric facets -- cyan, blue, purple, green and
-- red wedges and arcs mirrored left and right, a red cone with green sides
-- in the centre, every region a smooth luma gradient.
--
-- The program: three linear fields (rotating slopes) and one circle field
-- cut the screen into regions; each region's colour comes from a 16-entry
-- palette indexed by the four sign bits, shaded by a luma gradient built
-- from the fields; the picture is mirrored about the centre column.  The
-- source (video / oscillator) modulates the shading.
--
--   K1  Spread   angle between the three wedge fields
--   K2  Circle   circle size
--   K3  Hue      palette rotation (4 palettes)
--   K4  Shading  gradient depth
--   K5  Spin     continuous rotation, bipolar (centre = still)
--   K6  Split    horizontal position of the mirror / centre
--   S7  Mirror   on / off
--   S8  Palette  Jewel / Earth
--   S9  Relief   the source luma shades the facets on / off
--   S10 Circle   on / off
--   S11 Source   Video / Osc
--   P12 TILT     rotation of the wedges: upright .. a quarter turn
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

architecture facet of program_top is
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

    signal s_jf : std_logic_vector(7 downto 0) := (others => '0');
    -- 4 palettes x 16 regions
    type t_pal is array (0 to 63) of std_logic_vector(31 downto 0);
    function f_pal return t_pal is
        variable r : t_pal;
        type t_c is array (0 to 7) of std_logic_vector(31 downto 0);
        constant J : t_c := (f_col(678, 64, 663), f_col(164, 440, 960), f_col(349, 756, 852), f_col(580, 136, 216),
                             f_col(328, 960, 360), f_col(192, 608, 696), f_col(426, 887, 808), f_col(700, 512, 512));
        constant E : t_c := (f_col(584, 772, 212), f_col(260, 512, 512), f_col(840, 584, 64), f_col(580, 136, 216),
                             f_col(328, 960, 360), f_col(752, 612, 512), f_col(480, 512, 512), f_col(940, 512, 512));
    begin
        for p in 0 to 3 loop
            for i in 0 to 15 loop
                if p < 2 then r(p * 16 + i) := J((i + 3 * p) mod 8); else r(p * 16 + i) := E((i + 3 * p) mod 8); end if;
            end loop;
        end loop;
        return r;
    end function;
    constant C_PAL : t_pal := f_pal;
    signal s_pal_ra : unsigned(5 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');
    -- quarter-wave sine, 64 entries, Q.8
    type t_sin is array (0 to 63) of integer;
    constant C_SIN : t_sin := (0, 6, 13, 19, 25, 31, 38, 44, 50, 56, 62, 68, 74, 80, 86, 92, 98, 104, 109, 115, 121, 126,
        132, 137, 142, 147, 152, 157, 162, 167, 172, 177, 181, 185, 190, 194, 198, 202, 206, 209, 213, 216, 220, 223,
        226, 229, 231, 234, 237, 239, 241, 243, 245, 247, 248, 250, 251, 252, 253, 254, 255, 255, 256, 256);
    function f_sin(ph : unsigned(7 downto 0)) return signed is      -- full wave, Q.8 signed
        variable i : integer;
        variable m : integer;
    begin
        if ph(6) = '1' then i := 63 - to_integer(ph(5 downto 0)); else i := to_integer(ph(5 downto 0)); end if;
        m := C_SIN(i);
        if ph(7) = '1' then return to_signed(-m, 10); else return to_signed(m, 10); end if;
    end function;
    type t_s10 is array (natural range <>) of signed(9 downto 0);
    type t_s12 is array (natural range <>) of signed(11 downto 0);
    type t_s24 is array (natural range <>) of signed(23 downto 0);

    signal s_spr : unsigned(7 downto 0) := to_unsigned(85, 8);
    signal s_rk : unsigned(9 downto 0) := (others => '0');
    signal s_hue : unsigned(1 downto 0) := (others => '0');
    signal s_shk : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_spl : signed(9 downto 0) := (others => '0');
    signal s_mir, s_palb, s_relief, s_circ : std_logic := '0';
    -- derived
    signal s_rot : unsigned(15 downto 0) := (others => '0');          -- spin phase
    signal s_tilt : unsigned(7 downto 0) := (others => '0');
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal s_ang : t_u8(0 to 2) := (others => (others => '0'));
    signal s_ca, s_sa : t_s10(0 to 2) := (others => (others => '0'));  -- cos / sin per field
    signal s_inc : t_s12(0 to 2) := (others => (others => '0'));       -- cos * xs (Q.8 per pixel)
    signal s_cxo : signed(12 downto 0) := (others => '0');             -- centre x offset (virtual)
    signal s_cxv : unsigned(11 downto 0) := to_unsigned(960, 12);      -- centre x
    signal s_acx : t_s24(0 to 2) := (others => (others => '0'));       -- -cos * cx per field
    signal s_r2 : unsigned(21 downto 0) := (others => '0');            -- circle radius^2
    signal s_cx2sq : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_rr : unsigned(10 downto 0) := to_unsigned(300, 11);
    -- per line
    signal s_ls_n, s_ls : t_s24(0 to 2) := (others => (others => '0'));   -- line starts
    signal s_dy2_n, s_dy2 : unsigned(19 downto 0) := (others => '0');
    signal s_sdy : t_s24(0 to 2) := (others => (others => '0'));
    -- beam
    signal s_l : t_s24(0 to 2) := (others => (others => '0'));
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inq : signed(14 downto 0) := (others => '0');
    signal s_dxs : signed(13 downto 0) := (others => '0');              -- 2 (x - cx), Q.1 virtual
    signal s_fold : std_logic := '0';
    -- pixel
    signal s_code1 : unsigned(3 downto 0) := (others => '0');
    signal s_al1 : t_s24(0 to 1) := (others => (others => '0'));
    signal s_qi1 : unsigned(21 downto 0) := (others => '0');
    signal s_code : t_u10(2 to 6) := (others => (others => '0'));
    signal s_sh2 : unsigned(23 downto 0) := (others => '0');
    signal s_r2_2 : unsigned(22 downto 0) := (others => '0');
    signal s_sh3 : unsigned(9 downto 0) := (others => '0');
    signal s_in3 : std_logic := '0';
    signal s_sh4 : unsigned(17 downto 0) := (others => '0');
    signal s_sh5, s_sh6, s_sh7 : unsigned(9 downto 0) := (others => '0');
    signal s_py6, s_pu6, s_pv6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py, s_pu, s_pv : t_u10(7 to 11) := (others => to_unsigned(64, 10));
    signal s_shd : t_u10(8 to 11) := (others => (others => '0'));
    signal s_vl : t_u10(4 to 11) := (others => (others => '0'));

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
        variable v_c : signed(12 downto 0);
        variable v_i : signed(15 downto 0);

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
            s_spr <= resize(unsigned(registers_in(0)(9 downto 3)), 8) + 8;
            s_rk <= unsigned(registers_in(1));
            s_hue <= unsigned(registers_in(2)(9 downto 8));
            s_shk <= unsigned(registers_in(3)(9 downto 2));
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;
            if v_s > -8 and v_s < 8 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;
            s_spl <= v_s(9 downto 0);

            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mir    <= registers_in(6)(0);
                s_palb   <= registers_in(6)(1);
                s_relief <= registers_in(6)(2);
                s_circ   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_rot <= s_rot + unsigned(resize(s_k5r, 16));

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
            s_tilt <= s_g10(9 downto 2) + s_rot(15 downto 8);
            -- field angles: tilt, tilt + spread, tilt + 2 spread (Q.8 sin/cos, 256 = full turn)
            -- (pipelined: angles one clock, sin / cos the next)
            s_ang(0) <= s_tilt;
            s_ang(1) <= s_tilt + s_spr;
            s_ang(2) <= s_tilt + shift_left(s_spr, 1);
            for i in 0 to 2 loop
                s_ca(i) <= f_sin(s_ang(i) + 64);
                s_sa(i) <= f_sin(s_ang(i));
                case to_integer(s_xs) is
                    when 4      => s_inc(i) <= shift_left(resize(s_ca(i), 12), 1);
                    when 3      => s_inc(i) <= resize(s_ca(i), 12) + resize(shift_right(s_ca(i), 1), 12);
                    when others => s_inc(i) <= resize(s_ca(i), 12);
                end case;
            end loop;
            -- centre x = vw/2 + split (-512 .. 511 virtual), clamped inside
            v_c := signed(resize(s_vw(11 downto 1), 13)) + resize(s_spl, 13) + resize(s_spl, 13);
            if v_c < 64 then s_cxv <= to_unsigned(64, 12); elsif v_c > signed(resize(s_vw, 13)) - 64 then s_cxv <= s_vw - 64; else s_cxv <= unsigned(v_c(11 downto 0)); end if;
            s_rr <= resize(s_rk, 11) + 32;
            case to_integer(s_xs) is
                when 4      => s_inc2 <= to_unsigned(32, 6);
                               v_i := to_signed(16, 16) - signed(shift_left(resize(s_cxv, 16), 4));
                when 3      => s_inc2 <= to_unsigned(18, 6);
                               v_i := to_signed(9, 16) - signed(shift_left(resize(s_cxv, 16), 3) + shift_left(resize(s_cxv, 16), 2));
                when others => s_inc2 <= to_unsigned(8, 6);
                               v_i := to_signed(4, 16) - signed(shift_left(resize(s_cxv, 16), 3));
            end case;
            s_inc0 <= v_i(14 downto 0);

            -- registered job-select flags (one clock ahead of the load mux)
            if s_zc = 4 then s_jf(0) <= '1'; else s_jf(0) <= '0'; end if;
            if s_zc = 20 then s_jf(1) <= '1'; else s_jf(1) <= '0'; end if;
            if s_zc = 36 then s_jf(2) <= '1'; else s_jf(2) <= '0'; end if;
            if s_zc = 52 then s_jf(3) <= '1'; else s_jf(3) <= '0'; end if;
            if s_zc = 68 or s_fall1 = '1' then s_jf(4) <= '1'; else s_jf(4) <= '0'; end if;
            if s_zc = 84 or s_fall2 = '1' then s_jf(5) <= '1'; else s_jf(5) <= '0'; end if;
            if s_zc = 100 or s_fall3 = '1' then s_jf(6) <= '1'; else s_jf(6) <= '0'; end if;
            if s_zc = 116 then s_jf(7) <= '1'; else s_jf(7) <= '0'; end if;


            -- serial multiplier: per-frame jobs in the zone, per-line jobs at
            -- the source avid fall (+2 and +18 clocks)
            if s_jf(0) = '1' then
                s_ma <= resize(s_rr, 24); s_mb <= resize(s_rr, 12); s_macc <= (others => '0');
                s_mjob <= "000"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_jf(1) = '1' then
                s_ma <= resize(unsigned(abs(s_ca(0))), 24); s_mb <= s_cxv; s_macc <= (others => '0');
                s_mjob <= "001"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_jf(2) = '1' then
                s_ma <= resize(unsigned(abs(s_ca(1))), 24); s_mb <= s_cxv; s_macc <= (others => '0');
                s_mjob <= "010"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_jf(3) = '1' then
                s_ma <= resize(unsigned(abs(s_ca(2))), 24); s_mb <= s_cxv; s_macc <= (others => '0');
                s_mjob <= "011"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_jf(4) = '1' then
                s_ma <= resize(unsigned(abs(s_sa(0))), 24); s_mb <= resize(s_ady_n, 12); s_macc <= (others => '0');
                s_mjob <= "100"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_jf(5) = '1' then
                s_ma <= resize(unsigned(abs(s_sa(1))), 24); s_mb <= resize(s_ady_n, 12); s_macc <= (others => '0');
                s_mjob <= "101"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_jf(6) = '1' then
                s_ma <= resize(unsigned(abs(s_sa(2))), 24); s_mb <= resize(s_ady_n, 12); s_macc <= (others => '0');
                s_mjob <= "110"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_jf(7) = '1' then
                s_ma <= shift_left(resize(s_cxv, 24), 1); s_mb <= shift_left(resize(s_cxv, 12), 1); s_macc <= (others => '0');
                s_mjob <= "111"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000" => s_r2 <= s_macc(21 downto 0);
                    when "001" => if s_ca(0) < 0 then s_acx(0) <= signed(s_macc); else s_acx(0) <= -signed(s_macc); end if;
                    when "010" => if s_ca(1) < 0 then s_acx(1) <= signed(s_macc); else s_acx(1) <= -signed(s_macc); end if;
                    when "011" => if s_ca(2) < 0 then s_acx(2) <= signed(s_macc); else s_acx(2) <= -signed(s_macc); end if;
                    when "100" => if s_sa(0) < 0 xor s_dy_n < 0 then s_sdy(0) <= -signed(s_macc); else s_sdy(0) <= signed(s_macc); end if;
                    when "101" => if s_sa(1) < 0 xor s_dy_n < 0 then s_sdy(1) <= -signed(s_macc); else s_sdy(1) <= signed(s_macc); end if;
                    when "110" => if s_sa(2) < 0 xor s_dy_n < 0 then s_sdy(2) <= -signed(s_macc); else s_sdy(2) <= signed(s_macc); end if;
                    when "111" => s_cx2sq <= s_macc;
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
        variable v_j : unsigned(3 downto 0);

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
                s_dy2_n <= to_unsigned(291600, 20);

            elsif s_fall1 = '1' then
                v_vy := s_vy_n + s_ys;
                s_vy_n <= v_vy;
                v_dy := signed(resize(v_vy(19 downto 8), 13)) - 540;
                s_dy_n <= v_dy(11 downto 0);
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11); else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
                s_dy2_n <= resize(s_ady_n * s_ady_n, 20);

            end if;
            -- the field 0 per-line product runs at s_fall1 + 1 on its own (the
            -- unit is free then): start it one clock before the shared job 5
            for i in 0 to 2 loop
                s_ls_n(i) <= s_acx(i) + s_sdy(i);
            end loop;


            if data_in.avid = '1' and s_avq = '0' then
                s_vy <= s_vy_n(18 downto 8);
                s_dy <= s_dy_n;
                s_ady <= s_ady_n;
                s_ls <= s_ls_n;
                s_dy2 <= s_dy2_n;

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
                for i in 0 to 2 loop s_l(i) <= s_ls(i); end loop;
                s_q <= s_cx2sq;
                s_inq <= s_inc0;
                s_dxs <= -signed(shift_left(resize(s_cxv, 14), 1));
                s_fold <= '0';

            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_vx <= s_vx + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 14));
                for i in 0 to 2 loop
                    if s_fold = '1' then s_l(i) <= s_l(i) - resize(s_inc(i), 24); else s_l(i) <= s_l(i) + resize(s_inc(i), 24); end if;
                end loop;
                s_q <= unsigned(signed(s_q) + resize(s_inq, 24));
                s_inq <= s_inq + signed(resize(s_inc2, 15));
                s_dxs <= s_dxs + signed(resize(s_xs, 14));
                if s_mir = '1' and s_dxs >= 0 then s_fold <= '1'; end if;

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
        variable v_y : signed(12 downto 0);
        variable v_cd : unsigned(9 downto 0);
        variable v_pi : unsigned(1 downto 0);

    begin
        if rising_edge(clk) then
            s_lb_ra <= s_rdp;
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 1: sign code, |L0| + |L1| for the shading, dx^2
            for i in 0 to 2 loop
                s_code1(i) <= s_l(i)(23);
            end loop;
            s_code1(3) <= '0';
            if s_l(0) < 0 then s_al1(0) <= -s_l(0); else s_al1(0) <= s_l(0); end if;
            if s_l(1) < 0 then s_al1(1) <= -s_l(1); else s_al1(1) <= s_l(1); end if;
            s_qi1 <= s_q(23 downto 2);
            -- 2: shading sum; r^2
            s_sh2 <= unsigned(s_al1(0)) + unsigned(s_al1(1));
            s_r2_2 <= resize(s_qi1, 23) + resize(s_dy2, 23);
            s_code(2) <= resize(s_code1, 10);
            -- 3: shading 10 bits; circle test
            if s_sh2(23 downto 20) /= 0 then s_sh3 <= (others => '1'); else s_sh3 <= s_sh2(19 downto 10); end if;
            if s_circ = '1' and s_r2_2 <= resize(s_r2, 23) then s_in3 <= '1'; else s_in3 <= '0'; end if;
            s_code(3) <= s_code(2);
            -- 4: shading x depth; code with the circle bit; source luma
            s_sh4 <= s_sh3 * s_shk;
            v_cd := s_code(3); v_cd(3) := s_in3; s_code(4) <= v_cd;
            if s_relief = '1' then s_vl(4) <= '0' & s_y3(9 downto 1); else s_vl(4) <= (others => '0'); end if;
            -- 5: palette address
            v_pi := s_hue; if s_palb = '1' then v_pi := v_pi + 2; end if;
            s_pal_ra <= v_pi & s_code(4)(3 downto 0);
            if s_sh4(17 downto 16) /= 0 then s_sh5 <= (others => '1'); else s_sh5 <= s_sh4(15 downto 6); end if;
            -- 6: ROM register; 7: fabric copy
            s_sh6 <= s_sh5;
            s_sh7 <= s_sh6;
            s_py(7) <= unsigned(s_pal_q(31 downto 22)); s_pu(7) <= unsigned(s_pal_q(21 downto 12)); s_pv(7) <= unsigned(s_pal_q(11 downto 2));
            -- 8: shade = facet gradient + source relief (each up to 1023 -> half each)
            s_shd(8) <= resize(s_sh7(9 downto 1), 10) + resize(s_vl(7)(9 downto 1), 10);
            -- 9 .. 11: copies
            -- 12: shaded colour: luma - shade / 2 (dark toward the facet edges), clamped
            v_y := signed(resize(s_py(11), 13)) - signed(resize(s_shd(11)(9 downto 1), 13)) + 160;
            s_y12 <= f_clamp(v_y); s_u12 <= s_pu(11); s_v12 <= s_pv(11);
            -- delays
            for i in 5 to 6 loop s_code(i) <= s_code(i - 1); end loop;
            for i in 5 to 11 loop s_vl(i) <= s_vl(i - 1); end loop;
            for i in 8 to 11 loop s_py(i) <= s_py(i - 1); s_pu(i) <= s_pu(i - 1); s_pv(i) <= s_pv(i - 1); end loop;
            for i in 9 to 11 loop s_shd(i) <= s_shd(i - 1); end loop;


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

end architecture facet;
