-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ovals v0.1.0 -- Heirloom #50 (2026-10-20), from WIN_20231003_23_01_50.
--
-- The photo: a hot-pink ground streaked horizontally, carrying a lattice of
-- overlapping ovals -- green / blue / purple rims around orange, yellow and
-- white cores -- with small diamonds where the rims cross, two rows deep.
--
-- The program: a cell grid of lenses; each lens is coloured by its r^2 from
-- the cell centre through a core-to-rim palette (white, yellow, orange, then
-- a green / blue rim) and the pink ground shows beyond the rim.  Odd rows
-- offset half a cell so the lenses interlock; the picture glows in the lens
-- cores and the ground is streaked.  P12 sets the lens size.
--
--   K1  Cells    lenses across, 2 .. 17
--   K2  Rows     lens rows, 1 .. 8
--   K3  Core     how much of each lens is the picture's core
--   K4  Smear    horizontal streak of the picture
--   K5  Drift    lenses slide sideways, bipolar
--   K6  Aspect   lens shape: wide .. tall
--   S7  Offset   odd rows offset by half a cell
--   S8  Palette  Warm / Cool
--   S9  Rim      Off / On (green / blue rim beyond the orange)
--   S10 Ground   Pink / Picture
--   S11 Source   Video / Osc
--   P12 LENS     lens size: separate ovals .. an overlapping lattice
--
-- Geometry in virtual 1080-line units.  Cell width from one serial divide
-- per frame; dx^2 by second differences, dy^2 per line on the serial
-- multiplier.  Generated avid; output conditioning before the blanking gate.
-- Latency 16 clocks.  EBR: 16 line buffer + 4 palette.  Colours in the
-- hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture ovals of program_top is
    constant C_LAT : integer := 16;

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

    type t_ch is array (0 to 7) of integer;
    constant C_CH : t_ch := (1080, 540, 360, 270, 216, 180, 154, 135);

    -- palettes (core -> rim): A white / yellow / orange / green / blue;
    -- B white / cyan / blue / violet / magenta
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_KA : t_keys := ((0, 940, 512, 512), (48, 840, 584, 64), (120, 584, 772, 212), (176, 584, 772, 212),
                               (192, 580, 136, 216), (255, 164, 440, 960));
    constant C_KB : t_keys := ((0, 940, 512, 512), (48, 678, 64, 663), (120, 164, 440, 960), (176, 164, 440, 960),
                               (192, 349, 756, 852), (255, 426, 887, 808));
    type t_pal is array (0 to 511) of std_logic_vector(31 downto 0);
    function f_lerp(a, b, t, n : integer) return integer is
    begin
        return a + ((b - a) * t) / n;
    end function;
    function f_seg(k : t_keys; i : integer) return std_logic_vector is
        variable y, u, v : integer;
    begin
        y := 64; u := 512; v := 512;
        for j in 0 to k'length - 2 loop
            if i >= k(j).i and i <= k(j + 1).i then
                y := f_lerp(k(j).y, k(j + 1).y, i - k(j).i, k(j + 1).i - k(j).i);
                u := f_lerp(k(j).u, k(j + 1).u, i - k(j).i, k(j + 1).i - k(j).i);
                v := f_lerp(k(j).v, k(j + 1).v, i - k(j).i, k(j + 1).i - k(j).i);
            end if;
        end loop;
        return std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
               & std_logic_vector(to_unsigned(v, 10)) & "00";
    end function;
    function f_pal return t_pal is
        variable r : t_pal;
    begin
        for i in 0 to 255 loop
            r(i) := f_seg(C_KA, i);
            r(256 + i) := f_seg(C_KB, i);
        end loop;
        return r;
    end function;
    signal pal : t_pal := f_pal;
    signal s_pal_ra : unsigned(8 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- line buffer {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- oscillator source
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
    signal s_j1 : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_j2 : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal s_core : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_sh : unsigned(2 downto 0) := (others => '0');
    signal s_spd : signed(5 downto 0) := (others => '0');
    signal s_aspk : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    signal s_offs, s_palb, s_rim, s_gvid : std_logic := '0';
    -- field-latched / derived
    signal s_n : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_m : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal s_cwv : unsigned(11 downto 0) := to_unsigned(240, 12);
    signal s_cwv2 : unsigned(12 downto 0) := to_unsigned(480, 13);
    signal s_ch : unsigned(10 downto 0) := to_unsigned(540, 11);      -- row height
    signal s_chq : unsigned(19 downto 0) := (others => '0');
    signal s_chh : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_zk : unsigned(8 downto 0) := to_unsigned(128, 9);
    signal s_ph : unsigned(12 downto 0) := (others => '0');           -- drift phase (Q.1)
    signal s_dxh0 : signed(13 downto 0) := (others => '0');
    signal s_q0, s_q0s : unsigned(23 downto 0) := (others => '0');
    signal s_inc0, s_inc0s : signed(15 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    -- serial units
    signal s_dnum : unsigned(11 downto 0) := (others => '0');
    signal s_drem : unsigned(12 downto 0) := (others => '0');
    signal s_dquo : unsigned(11 downto 0) := (others => '0');
    signal s_dden : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal s_di : unsigned(3 downto 0) := (others => '0');
    signal s_drun : std_logic := '0';
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
    signal s_fall1, s_fall2 : std_logic := '0';
    signal s_lc : unsigned(6 downto 0) := (others => '1');
    signal s_cv_n : unsigned(19 downto 0) := (others => '0');
    signal s_rodd_n, s_rodd : std_logic := '0';
    signal s_ady : unsigned(10 downto 0) := (others => '0');
    signal s_dy2_n : unsigned(21 downto 0) := (others => '0');
    signal s_dy2 : unsigned(23 downto 0) := (others => '0');

    -- beam state
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_cu : unsigned(12 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(15 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_qa1 : unsigned(21 downto 0) := (others => '0');
    signal s_rr2 : unsigned(23 downto 0) := (others => '0');
    signal s_p3 : unsigned(23 downto 0) := (others => '0');
    signal s_idx4, s_idx5 : unsigned(7 downto 0) := (others => '0');
    signal s_out4, s_out5, s_core5 : std_logic := '0';
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_sm4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_smu4, s_smv4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_sy5, s_su5, s_sv5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py6, s_pu6, s_pv6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_sy6, s_su6, s_sv6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_out6, s_core6 : std_logic := '0';
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y8, s_y9, s_y10, s_y11, s_y12, s_y13x, s_yf13 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y14, s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14, s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

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

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            s_pal_q <= pal(to_integer(s_pal_ra));
        end if;
    end process p_pal;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial divide / products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_r : unsigned(12 downto 0);
        variable v_p : signed(14 downto 0);
        variable v_i : signed(16 downto 0);
        variable v_st : std_logic;
        variable v_jb : unsigned(2 downto 0);
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

            -- K1 Cells (16 zones), K2 Rows (8 zones), deadbands after warm-up
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 3 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 60 or v_k(9 downto 6) = 15)) then
                s_j1 <= v_k(9 downto 6);
            end if;
            v_k := unsigned(registers_in(1));
            if s_warm /= 15 or ((v_k(6 downto 0) >= 6 or v_k(9 downto 7) = 0) and
                                (v_k(6 downto 0) <= 121 or v_k(9 downto 7) = 7)) then
                s_j2 <= v_k(9 downto 7);
            end if;
            s_core <= unsigned(registers_in(2)(9 downto 2));                   -- K3 Core
            s_sh <= unsigned(registers_in(3)(9 downto 7));                     -- K4 Smear (8 steps)
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;        -- K5 Drift, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(5 downto 0); end if;
            s_aspk <= unsigned(registers_in(5)(9 downto 7));                   -- K6 Aspect (8 zones)
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_n <= s_j1;
                s_m <= s_j2;
                s_offs <= registers_in(6)(0);
                s_palb <= registers_in(6)(1);
                s_rim  <= registers_in(6)(2);
                s_gvid <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                -- drift phase in Q.1 cell units, wrapped to the cell
                v_p := signed(resize(s_ph, 15)) + resize(s_spd, 15);
                if v_p < 0 then v_p := v_p + signed(resize(s_cwv2, 15));
                elsif v_p >= signed(resize(s_cwv2, 15)) then v_p := v_p - signed(resize(s_cwv2, 15)); end if;
                s_ph <= unsigned(v_p(12 downto 0));
            end if;

            -- derived
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1); s_inc2 <= to_unsigned(32, 6);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14); s_inc2 <= to_unsigned(18, 6);
                when others => v_vw := resize(s_w, 14); s_inc2 <= to_unsigned(8, 6);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cwv2 <= s_cwv & '0';
            s_ch <= to_unsigned(C_CH(to_integer(s_m)), 11);
            s_chq <= resize(s_ch, 12) & x"00";
            s_chh <= '0' & s_ch(10 downto 1);
            s_zk <= to_unsigned(256, 9) - resize(s_g(11 downto 2), 9) + resize(s_g(11 downto 6), 9);   -- 256 .. 17
            s_dxh0 <= signed(resize(s_ph, 14)) - signed(resize(s_cwv, 14));
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 17) - signed(shift_left(resize(s_cwv, 17), 3));
                               s_inc0s <= resize(to_signed(16, 17) + shift_left(resize(s_dxh0, 17), 3), 16);
                when 3      => v_i := to_signed(9, 17) - signed(shift_left(resize(s_cwv, 17), 2) + shift_left(resize(s_cwv, 17), 1));
                               s_inc0s <= resize(to_signed(9, 17) + shift_left(resize(s_dxh0, 17), 2) + shift_left(resize(s_dxh0, 17), 1), 16);
                when others => v_i := to_signed(4, 17) - signed(shift_left(resize(s_cwv, 17), 2));
                               s_inc0s <= resize(to_signed(4, 17) + shift_left(resize(s_dxh0, 17), 2), 16);
            end case;
            s_inc0 <= v_i(15 downto 0);

            -- serial divide: cwv = vw / N (zone clocks 4 .. 16)
            if s_zc = 4 then
                s_dnum <= s_vw; s_drem <= (others => '0'); s_dden <= resize(s_n, 5) + 2;
                s_drun <= '1'; s_di <= (others => '0');
            elsif s_drun = '1' then
                v_r := s_drem(11 downto 0) & s_dnum(11);
                s_dnum <= s_dnum(10 downto 0) & '0';
                if v_r >= resize(s_dden, 13) then
                    s_drem <= v_r - resize(s_dden, 13);
                    s_dquo <= s_dquo(10 downto 0) & '1';
                else
                    s_drem <= v_r;
                    s_dquo <= s_dquo(10 downto 0) & '0';
                end if;
                if s_di = 11 then s_drun <= '0'; else s_di <= s_di + 1; end if;
            elsif s_di = 11 then
                s_di <= (others => '0');
                s_cwv <= s_dquo;
            end if;

            -- serial products: zone 3 cw^2, 4 |dxh0|^2, 5 the first line's dy^2;
            -- per line 5 dy^2
            v_st := '0'; v_jb := (others => '0');
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 72  => v_st := '1'; v_jb := "011";
                    when 88  => v_st := '1'; v_jb := "100";
                    when 104 => v_st := '1'; v_jb := "101";
                    when others => null;
                end case;
            elsif s_lc = 2 then
                v_st := '1'; v_jb := "101";
            end if;
            if v_st = '1' then
                s_mjob <= v_jb; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
                case to_integer(v_jb) is
                    when 3 => s_ma <= resize(s_cwv, 24); s_mb <= s_cwv;
                    when 4 => if s_dxh0 < 0 then s_ma <= resize(unsigned(-s_dxh0), 24); s_mb <= resize(unsigned(-s_dxh0), 12);
                              else s_ma <= resize(unsigned(s_dxh0), 24); s_mb <= resize(unsigned(s_dxh0), 12); end if;
                    when others => s_ma <= resize(s_ady, 24); s_mb <= resize(s_ady, 12);
                end case;
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case to_integer(s_mjob) is
                    when 3 => s_q0 <= s_macc;
                    when 4 => s_q0s <= s_macc;
                    when others => s_dy2_n <= s_macc(21 downto 0);
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
    -- Source side: line-buffer write; row DDA, |dy|, shading term and row
    -- colour per line
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(20 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_a : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
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
            if s_fall1 = '1' then s_lc <= (others => '0'); elsif s_lc /= 127 then s_lc <= s_lc + 1; end if;

            if s_zone = '1' then
                s_cv_n <= (others => '0');
                s_rodd_n <= '0';
            elsif s_fall1 = '1' then
                v_c := resize(s_cv_n, 21) + s_ys;
                if v_c >= s_chq then
                    s_cv_n <= resize(v_c - s_chq, 20);
                    s_rodd_n <= not s_rodd_n;
                else
                    s_cv_n <= v_c(19 downto 0);
                end if;
            end if;
            -- |dy| (free-running from the next line's row position)
            v_d := signed(resize(s_cv_n(19 downto 8), 13)) - signed(resize(s_chh, 13));
            if v_d < 0 then s_ady <= resize(unsigned(-v_d), 11); else s_ady <= unsigned(v_d(10 downto 0)); end if;

            if data_in.avid = '1' and s_avq = '0' then
                case to_integer(s_aspk) is
                    when 0 => v_a := resize(s_dy2_n(21 downto 1), 24);
                    when 1 => v_a := resize(s_dy2_n(21 downto 1), 24) + resize(s_dy2_n(21 downto 2), 24);
                    when 2 => v_a := resize(s_dy2_n, 24);
                    when 3 => v_a := resize(s_dy2_n, 24) + resize(s_dy2_n(21 downto 2), 24);
                    when 4 => v_a := resize(s_dy2_n, 24) + resize(s_dy2_n(21 downto 1), 24);
                    when 5 => v_a := shift_left(resize(s_dy2_n, 24), 1);
                    when 6 => v_a := shift_left(resize(s_dy2_n, 24), 1) + resize(s_dy2_n, 24);
                    when others => v_a := shift_left(resize(s_dy2_n, 24), 2);
                end case;
                s_dy2 <= v_a;
                s_rodd <= s_rodd_n and s_offs;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state: column, cell x (Q.1), dx^2 by second differences
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_col <= (others => '0');
                if s_rodd = '1' then
                    -- odd rows: half a cell on (the drift is dropped there, a
                    -- small cost for keeping the start constants to one pair)
                    s_cu <= resize(s_cwv, 13);
                    s_q <= (others => '0');
                    s_inc <= signed(resize(s_inc2(5 downto 1), 16));
                else
                    s_cu <= s_ph;
                    s_q <= s_q0s;
                    s_inc <= s_inc0s;
                end if;
            elsif g_avid = '1' then
                s_col <= s_col + 1;
                v_n := resize(s_cu, 14) + s_xs;
                if v_n >= s_cwv2 then
                    s_cu <= resize(v_n - s_cwv2, 13);
                    s_q <= s_q0;
                    s_inc <= s_inc0;
                else
                    s_cu <= v_n(12 downto 0);
                    s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                    s_inc <= s_inc + signed(resize(s_inc2, 16));
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d : signed(11 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: dx^2 (virtual); video read
            s_qa1 <= s_q(23 downto 2);
            s_lb_ra <= s_col;

            -- 2: r^2
            s_rr2 <= resize(s_qa1, 24) + s_dy2;

            -- 3: lens zoom; video (fabric copy)
            s_p3 <= s_rr2(23 downto 8) * resize(s_zk, 8);
            if s_zk(8) = '1' then s_p3 <= s_rr2; end if;
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 4: palette index (saturating: beyond the rim = ground); smear
            if s_p3(23 downto 14) /= 0 then s_idx4 <= (others => '1'); s_out4 <= '1';
            else s_idx4 <= s_p3(13 downto 6); s_out4 <= '0'; end if;
            if s_p3(23 downto 14) = 0 and s_p3(13 downto 6) = 255 then s_out4 <= '1'; end if;
            v_d := signed(resize(s_y3, 12)) - signed(resize(s_sm4, 12));
            if s_gav_sr(2) = '0' then
                s_sm4 <= s_y3; s_smu4 <= s_u3; s_smv4 <= s_v3;
            else
                s_sm4 <= unsigned(resize(signed(resize(s_sm4, 12)) + shift_right(v_d, to_integer(s_sh)), 10));
                v_d := signed(resize(s_u3, 12)) - signed(resize(s_smu4, 12));
                s_smu4 <= unsigned(resize(signed(resize(s_smu4, 12)) + shift_right(v_d, to_integer(s_sh)), 10));
                v_d := signed(resize(s_v3, 12)) - signed(resize(s_smv4, 12));
                s_smv4 <= unsigned(resize(signed(resize(s_smv4, 12)) + shift_right(v_d, to_integer(s_sh)), 10));
            end if;

            -- 5: palette address (no rim: the palette ends at the orange); core flag
            if s_rim = '0' and s_idx4 >= 176 and s_out4 = '0' then s_idx5 <= to_unsigned(176, 8); else s_idx5 <= s_idx4; end if;
            if s_idx4 < s_core then s_core5 <= '1'; else s_core5 <= '0'; end if;
            s_out5 <= s_out4;
            s_sy5 <= s_sm4; s_su5 <= s_smu4; s_sv5 <= s_smv4;
            s_pal_ra <= s_palb & s_idx5;

            -- 6: ROM register (pal_q); delays
            s_out6 <= s_out5; s_core6 <= s_core5;
            s_sy6 <= s_sy5; s_su6 <= s_su5; s_sv6 <= s_sv5;

            -- 7: palette fabric copy and compose
            s_py6 <= unsigned(s_pal_q(31 downto 22));
            s_pu6 <= unsigned(s_pal_q(21 downto 12));
            s_pv6 <= unsigned(s_pal_q(11 downto 2));
            if s_out6 = '1' then
                if s_gvid = '1' then
                    s_y7 <= s_sy6; s_u7 <= s_su6; s_v7 <= s_sv6;
                else
                    s_y7 <= to_unsigned(480, 10); s_u7 <= to_unsigned(880, 10); s_v7 <= to_unsigned(600, 10);
                end if;
            elsif s_core6 = '1' then
                s_y7 <= s_sy6; s_u7 <= s_pu6; s_v7 <= s_pv6;
            else
                s_y7 <= s_py6; s_u7 <= s_pu6; s_v7 <= s_pv6;
            end if;

            -- 8..13: output conditioning (centre at stage 12 -> 13)
            s_csu(0) <= s_u7; s_csv(0) <= s_v7;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u7 - s_csu(7);
            s_boxv <= s_boxv + s_v7 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u13 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v13 <= v_sc(13 downto 4);
            s_y8 <= s_y7; s_y9 <= s_y8; s_y10 <= s_y9; s_y11 <= s_y10;
            s_y12 <= s_y11; s_y13x <= s_y12;
            v_sy := resize(s_y11, 12) + shift_left(resize(s_y12, 12), 1) + resize(s_y13x, 12);
            s_yf13 <= v_sy(11 downto 2);
            -- 14: luma toward black; blanking gate
            if s_yf13 > 64 then v_dl := s_yf13 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(12) = '1' then
                s_y14 <= s_yf13 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
            -- 15..16: pad
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ovals;
