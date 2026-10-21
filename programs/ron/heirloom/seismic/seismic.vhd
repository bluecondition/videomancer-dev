-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Seismic v0.1.0 -- Heirloom #44 (2026-10-20), from WIN_20230610_00_10_54.
--
-- The photo: horizontal bands of a soft rainbow smear alternating with rows
-- of scalloped lenses -- yellow, cyan and magenta blobs on black -- every
-- row glowing brightest along its middle and fading to its edges.
--
-- The program: the screen is cut into rows of one pitch; even rows show the
-- picture smeared along the line under a rainbow that cycles across, odd
-- rows show the picture only inside a row of lenses, on black.  Every row
-- glows along its middle; P12 sets the lens size.
--
--   K1  Rows     row pitch
--   K2  Lenses   lenses across, 2 .. 33
--   K3  Hue      rainbow cycles across the screen
--   K4  Smear    horizontal smear of the picture
--   K5  Glow     row glow: soft wide .. tight bright middle
--   K6  Drift    rainbow drift, bipolar
--   S7  Rows     Smear first / Lenses first
--   S8  Lenses   Round / Bars
--   S9  Palette  Rainbow / CMY
--   S10 Ground   Black / White
--   S11 Source   Video / Osc
--   P12 BLOBS    lens size: dots .. lenses that merge into tubes
--
-- Geometry in virtual 1080-line units.  Lens pitch from one serial divide
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

architecture seismic of program_top is
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

    -- palettes (cyclic): A rainbow, B cyan / magenta / yellow
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_KA : t_keys := ((0, 328, 960, 360), (43, 840, 584, 64), (85, 580, 136, 216), (128, 678, 64, 663),
                               (171, 164, 440, 960), (213, 426, 887, 808), (255, 328, 960, 360));
    constant C_KB : t_keys := ((0, 678, 64, 663), (85, 426, 887, 808), (171, 840, 584, 64), (255, 678, 64, 663));
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
    signal s_rpk : unsigned(8 downto 0) := to_unsigned(120, 9);
    signal s_j2 : unsigned(4 downto 0) := to_unsigned(6, 5);
    signal s_hk : unsigned(7 downto 0) := to_unsigned(32, 8);
    signal s_sh : unsigned(2 downto 0) := (others => '0');
    signal s_gk : unsigned(7 downto 0) := (others => '0');
    signal s_spd : signed(5 downto 0) := (others => '0');
    signal s_hph : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    signal s_swap, s_bars, s_palb, s_white : std_logic := '0';
    -- field-latched / derived
    signal s_n : unsigned(4 downto 0) := to_unsigned(6, 5);
    signal s_cwv : unsigned(11 downto 0) := to_unsigned(240, 12);
    signal s_cwv2 : unsigned(12 downto 0) := to_unsigned(480, 13);
    signal s_ch : unsigned(10 downto 0) := to_unsigned(120, 11);      -- row pitch
    signal s_chq : unsigned(19 downto 0) := (others => '0');
    signal s_chh : unsigned(10 downto 0) := to_unsigned(60, 11);
    signal s_hm : unsigned(10 downto 0) := to_unsigned(60, 11);       -- half of min(cw, ch)
    signal s_r : unsigned(10 downto 0) := to_unsigned(40, 11);
    signal s_r2 : unsigned(21 downto 0) := (others => '0');
    signal s_q0 : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(15 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    -- serial units
    signal s_dnum : unsigned(11 downto 0) := (others => '0');
    signal s_drem : unsigned(12 downto 0) := (others => '0');
    signal s_dquo : unsigned(11 downto 0) := (others => '0');
    signal s_dden : unsigned(5 downto 0) := to_unsigned(8, 6);
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
    signal s_dy2_n, s_dy2 : unsigned(21 downto 0) := (others => '0');
    signal s_ge_n : unsigned(18 downto 0) := (others => '0');
    signal s_env : unsigned(7 downto 0) := (others => '1');

    -- beam state
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_cu : unsigned(12 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(15 downto 0) := (others => '0');
    signal s_hp : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_qa1 : unsigned(21 downto 0) := (others => '0');
    signal s_hp1 : unsigned(7 downto 0) := (others => '0');
    signal s_rr2 : unsigned(22 downto 0) := (others => '0');
    signal s_in3, s_in4, s_in5, s_in6 : std_logic := '0';
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_sm4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ye5 : unsigned(17 downto 0) := (others => '0');
    signal s_pu5, s_pv5 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y7, s_y8, s_y9, s_y10, s_y11, s_y12x, s_yf12 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y13, s_y14, s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13, s_u14, s_v14, s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

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

            s_rpk <= to_unsigned(40, 9) + resize(unsigned(registers_in(0)(9 downto 2)), 9);   -- K1 Rows: pitch 40 .. 295
            -- K2 Lenses (32 zones, deadband after warm-up)
            v_k := unsigned(registers_in(1));
            if s_warm /= 15 or ((v_k(4 downto 0) >= 2 or v_k(9 downto 5) = 0) and
                                (v_k(4 downto 0) <= 29 or v_k(9 downto 5) = 31)) then
                s_j2 <= v_k(9 downto 5);
            end if;
            s_hk <= unsigned(registers_in(2)(9 downto 2)) + 1;                 -- K3 Hue 1 .. 256 (wraps at the top)
            if registers_in(2)(9 downto 2) = "11111111" then s_hk <= (others => '1'); end if;
            s_sh <= unsigned(registers_in(3)(9 downto 7));                     -- K4 Smear (8 steps)
            s_gk <= unsigned(registers_in(4)(9 downto 2));                     -- K5 Glow
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;        -- K6 Drift, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(5 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_n <= s_j2;
                s_ch <= resize(s_rpk, 11);
                s_swap  <= registers_in(6)(0);
                s_bars  <= registers_in(6)(1);
                s_palb  <= registers_in(6)(2);
                s_white <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_hph <= s_hph + unsigned(resize(s_spd, 8));
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
            s_chq <= resize(s_ch, 12) & x"00";
            s_chh <= '0' & s_ch(10 downto 1);
            if s_cwv < resize(s_ch, 12) then s_hm <= s_cwv(11 downto 1); else s_hm <= '0' & s_ch(10 downto 1); end if;
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 17) - signed(shift_left(resize(s_cwv, 17), 3));
                when 3      => v_i := to_signed(9, 17) - signed(shift_left(resize(s_cwv, 17), 2) + shift_left(resize(s_cwv, 17), 1));
                when others => v_i := to_signed(4, 17) - signed(shift_left(resize(s_cwv, 17), 2));
            end case;
            s_inc0 <= v_i(15 downto 0);

            -- serial divide: cwv = vw / N (zone clocks 4 .. 16)
            if s_zc = 4 then
                s_dnum <= s_vw; s_drem <= (others => '0'); s_dden <= resize(s_n, 6) + 2;
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

            -- serial products: zone 0 R = hm x p12 / 1024, 1 R^2, 3 cw^2,
            -- 5 the first line's dy^2; per line 5 dy^2
            v_st := '0'; v_jb := (others => '0');
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 24  => v_st := '1'; v_jb := "000";
                    when 40  => v_st := '1'; v_jb := "001";
                    when 72  => v_st := '1'; v_jb := "011";
                    when 104 => v_st := '1'; v_jb := "101";
                    when others => null;
                end case;
            elsif s_lc = 2 then
                v_st := '1'; v_jb := "101";
            end if;
            if v_st = '1' then
                s_mjob <= v_jb; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
                case to_integer(v_jb) is
                    when 0 => s_ma <= resize(s_hm, 24); s_mb <= resize(s_g(11 downto 2), 12);
                    when 1 => s_ma <= resize(s_r, 24); s_mb <= resize(s_r, 12);
                    when 3 => s_ma <= resize(s_cwv, 24); s_mb <= s_cwv;
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
                    when 0 => s_r <= resize(s_macc(20 downto 10), 11) + 4;
                    when 1 => s_r2 <= s_macc(21 downto 0);
                    when 3 => s_q0 <= s_macc;
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
            -- |dy| and the glow envelope (free-running from the next line's row position)
            v_d := signed(resize(s_cv_n(19 downto 8), 13)) - signed(resize(s_chh, 13));
            if v_d < 0 then s_ady <= resize(unsigned(-v_d), 11); else s_ady <= unsigned(v_d(10 downto 0)); end if;
            s_ge_n <= s_ady * s_gk;

            if data_in.avid = '1' and s_avq = '0' then
                s_dy2 <= s_dy2_n;
                s_rodd <= s_rodd_n xor s_swap;
                if s_ge_n(18 downto 12) /= 0 then s_env <= (others => '0');
                else s_env <= not s_ge_n(11 downto 4); end if;
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
                s_cu <= (others => '0');
                s_q <= s_q0;
                s_inc <= s_inc0;
                s_hp <= (others => '0');
            elsif g_avid = '1' then
                s_col <= s_col + 1;
                s_hp <= s_hp + s_hk;
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
            -- 1: dx^2 (virtual); rainbow phase; video read
            s_qa1 <= s_q(23 downto 2);
            s_hp1 <= s_hp(15 downto 8) + s_hph;
            s_lb_ra <= s_col;

            -- 2: r^2 (bars: |dx| only); palette address
            if s_bars = '1' then s_rr2 <= resize(s_qa1, 23);
            else s_rr2 <= resize(s_qa1, 23) + resize(s_dy2, 23); end if;
            s_pal_ra <= s_palb & s_hp1;

            -- 3: inside test; video (fabric copy)
            if s_rr2 < resize(s_r2, 23) then s_in3 <= '1'; else s_in3 <= '0'; end if;
            s_y3 <= unsigned(s_lb_q(31 downto 22));

            -- 4: smear (one-pole along the line, restarted at the line start)
            v_d := signed(resize(s_y3, 12)) - signed(resize(s_sm4, 12));
            if s_gav_sr(2) = '0' then
                s_sm4 <= s_y3;
            else
                s_sm4 <= unsigned(resize(signed(resize(s_sm4, 12)) + shift_right(v_d, to_integer(s_sh)), 10));
            end if;
            s_in4 <= s_in3;

            -- 5: glow product; palette (fabric copy)
            s_ye5 <= s_sm4 * s_env;
            s_pu5 <= unsigned(s_pal_q(21 downto 12));
            s_pv5 <= unsigned(s_pal_q(11 downto 2));
            s_in5 <= s_in4;

            -- 6: compose: smear rows everywhere, lens rows inside the lenses,
            --    ground elsewhere
            if s_rodd = '0' or s_in5 = '1' then
                s_y6 <= resize(s_ye5(17 downto 8), 10) + 64;
                if s_ye5(17 downto 8) > 876 then s_y6 <= to_unsigned(940, 10); end if;
                s_u6 <= s_pu5; s_v6 <= s_pv5;
            elsif s_white = '1' then
                s_y6 <= to_unsigned(900, 10); s_u6 <= to_unsigned(512, 10); s_v6 <= to_unsigned(512, 10);
            else
                s_y6 <= to_unsigned(64, 10); s_u6 <= to_unsigned(512, 10); s_v6 <= to_unsigned(512, 10);
            end if;

            -- 7..12: output conditioning (centre at stage 11 -> 12)
            s_csu(0) <= s_u6; s_csv(0) <= s_v6;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u6 - s_csu(7);
            s_boxv <= s_boxv + s_v6 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u12 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v12 <= v_sc(13 downto 4);
            s_y7 <= s_y6; s_y8 <= s_y7; s_y9 <= s_y8; s_y10 <= s_y9;
            s_y11 <= s_y10; s_y12x <= s_y11;
            v_sy := resize(s_y10, 12) + shift_left(resize(s_y11, 12), 1) + resize(s_y12x, 12);
            s_yf12 <= v_sy(11 downto 2);
            -- 13: luma toward black; blanking gate
            if s_yf12 > 64 then v_dl := s_yf12 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(11) = '1' then
                s_y13 <= s_yf12 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u13 <= s_u12; s_v13 <= s_v12;
            else
                s_y13 <= to_unsigned(64, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            end if;
            -- 14..16: pad
            s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
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

end architecture seismic;
