-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Bottleneck v0.1.0 -- Heirloom #28 (2026-10-20), from WIN_20230429_22_57_43.
--
-- The photo: two hot-pink bottle silhouettes standing against blue / green /
-- black horizontal bands, lime highlights along their shoulders, everything
-- smeared sideways.
--
-- The program: horizontal bands (a blue / green / black palette by row, the
-- source's luma showing through -- or the picture itself) smeared along the
-- line by a four-way interleaved leaky integrator (P12).  Two bottle
-- silhouettes (a neck above the shoulder line, widening below) key over it in
-- pink, with a lime rim where the picture meets the glass.
--
--   K1  Bands    band pitch
--   K2  Width    bottle width
--   K3  Rim      rim width
--   K4  Hue      band palette offset
--   K5  Spacing  distance between the two bottles
--   K6  Drift    bottles drift sideways, bipolar (centre = still)
--   S7  Bottles  One / Two
--   S8  Rim      Off / On
--   S9  Colours  Pink bottles, lime rim / Lime bottles, pink rim
--   S10 Bands    Still / Rolling
--   S11 Source   Video / Osc
--   P12 SMEAR    sideways smear: none .. everything streaks off the right edge
--
-- Virtual 1080-line geometry; generated avid; output conditioning before the
-- gate.  Latency 20 clocks.  EBR: 16 line buffer + 2 palette.  u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture bottleneck of program_top is
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

    -- palettes: A rainbow (R Y G C B M R), B warm (red -> orange -> yellow -> white)
    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    constant C_KA : t_keys := ((0, 64, 512, 512), (40, 164, 440, 960), (100, 300, 300, 900), (140, 580, 136, 216),
                               (200, 400, 200, 400), (255, 64, 512, 512));
    constant C_KB : t_keys := ((0, 64, 512, 512), (60, 426, 887, 808), (128, 164, 440, 960), (200, 678, 64, 663), (255, 64, 512, 512));
    type t_pal is array (0 to 511) of std_logic_vector(31 downto 0);
    function f_lerp(a, b, t, n : integer) return integer is
    begin
        return a + ((b - a) * t) / n;
    end function;
    function f_pal return t_pal is
        variable r : t_pal;
        variable y, u, v : integer;
    begin
        for i in 0 to 255 loop
            for k in 0 to C_KA'length - 2 loop
                if i >= C_KA(k).i and i <= C_KA(k + 1).i then
                    y := f_lerp(C_KA(k).y, C_KA(k + 1).y, i - C_KA(k).i, C_KA(k + 1).i - C_KA(k).i);
                    u := f_lerp(C_KA(k).u, C_KA(k + 1).u, i - C_KA(k).i, C_KA(k + 1).i - C_KA(k).i);
                    v := f_lerp(C_KA(k).v, C_KA(k + 1).v, i - C_KA(k).i, C_KA(k + 1).i - C_KA(k).i);
                end if;
            end loop;
            r(i) := std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
                    & std_logic_vector(to_unsigned(v, 10)) & "00";
            for k in 0 to C_KB'length - 2 loop
                if i >= C_KB(k).i and i <= C_KB(k + 1).i then
                    y := f_lerp(C_KB(k).y, C_KB(k + 1).y, i - C_KB(k).i, C_KB(k + 1).i - C_KB(k).i);
                    u := f_lerp(C_KB(k).u, C_KB(k + 1).u, i - C_KB(k).i, C_KB(k + 1).i - C_KB(k).i);
                    v := f_lerp(C_KB(k).v, C_KB(k + 1).v, i - C_KB(k).i, C_KB(k + 1).i - C_KB(k).i);
                end if;
            end loop;
            r(256 + i) := std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
                          & std_logic_vector(to_unsigned(v, 10)) & "00";
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
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_cx : unsigned(11 downto 0) := to_unsigned(960, 12);      -- virtual centre
    signal s_cxp : unsigned(10 downto 0) := to_unsigned(960, 11);     -- real centre
    signal s_offv : unsigned(10 downto 0) := (others => '0');          -- second vase offset (virtual)
    signal s_wk : unsigned(9 downto 0) := to_unsigned(200, 10);      -- bottle max half-width
    signal s_rimk : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_huek : unsigned(7 downto 0) := (others => '0');
    signal s_spk : unsigned(9 downto 0) := (others => '0');
    signal s_drk : signed(5 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched
    signal s_bwmax : unsigned(9 downto 0) := to_unsigned(200, 10);
    signal s_rim : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_hue : unsigned(7 downto 0) := (others => '0');
    signal s_sp : unsigned(9 downto 0) := (others => '0');
    signal s_leak : unsigned(8 downto 0) := to_unsigned(256, 9);      -- smear: 256 = none
    signal s_bph : unsigned(7 downto 0) := (others => '0');           -- band roll phase
    signal s_xdr : signed(11 downto 0) := (others => '0');            -- bottle drift (virtual px)
    signal s_two, s_rimon, s_swap, s_roll : std_logic := '0';
    signal s_fy : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";

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
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_dyc : unsigned(9 downto 0) := (others => '0');
    signal s_vp : unsigned(19 downto 0) := (others => '0');
    signal s_bprod : unsigned(17 downto 0) := (others => '0');
    signal s_hw_n, s_hw : unsigned(10 downto 0) := to_unsigned(48, 11);
    signal s_bidx_n : unsigned(7 downto 0) := (others => '0');
    signal s_by, s_bu, s_bv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- this line's band colour
    signal s_cxl : signed(12 downto 0) := to_signed(960, 13);

    -- beam state
    signal s_vxa : unsigned(12 downto 0) := (others => '0');          -- Q.1 virtual x

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_s11 is array (natural range <>) of signed(10 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_dxa1, s_dxb1 : signed(13 downto 0) := (others => '0');
    signal s_adxa2, s_adxb2 : unsigned(12 downto 0) := (others => '0');
    signal s_sga2, s_sgb2 : std_logic := '0';
    signal s_adx3 : unsigned(12 downto 0) := (others => '0');
    signal s_sg : t_sl(3 to 6) := (others => '0');
    signal s_in : t_sl(4 to 11) := (others => '0');
    signal s_e4 : signed(13 downto 0) := (others => '0');
    signal s_ae5 : unsigned(12 downto 0) := (others => '0');
    signal s_rimf : t_sl(6 to 11) := (others => '0');
    signal s_bcol : unsigned(10 downto 0) := (others => '0');
    signal s_sel : unsigned(1 downto 0) := (others => '0');
    signal s_seld : t_sl(1 to 9) := (others => '0');
    signal s_seld2 : t_sl(1 to 9) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y4, s_u4, s_v4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_acc is array (0 to 3) of signed(10 downto 0);
    signal s_accy, s_accu, s_accv : t_acc := (others => (others => '0'));
    signal s_dy5, s_du5, s_dv5 : signed(10 downto 0) := (others => '0');
    signal s_ay5, s_au5, s_av5 : signed(10 downto 0) := (others => '0');
    signal s_pyl6, s_pul6, s_pvl6 : signed(16 downto 0) := (others => '0');
    signal s_pyh6, s_puh6, s_pvh6 : signed(15 downto 0) := (others => '0');
    signal s_py7, s_pu7, s_pv7 : signed(20 downto 0) := (others => '0');
    signal s_ay6, s_au6, s_av6, s_ay7, s_au7, s_av7 : signed(10 downto 0) := (others => '0');
    signal s_y8s, s_u8s, s_v8s : signed(10 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(64, 10);
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
    -- Controls, vblank zone, per-field latches
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
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

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            s_fy <= resize(unsigned(registers_in(0)(9 downto 5)), 6) + 2;        -- K1 Bands 2 .. 33
            s_wk <= resize(unsigned(registers_in(1)(9 downto 1)), 10) + 40;      -- K2 Width 40 .. 551
            s_rimk <= resize(unsigned(registers_in(2)(9 downto 5)), 6) + 2;      -- K3 Rim 2 .. 33
            s_huek <= unsigned(registers_in(3)(9 downto 2));                     -- K4 Hue
            s_spk <= resize(unsigned(registers_in(4)(9 downto 1)), 10) + 60;     -- K5 Spacing 60 .. 571
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;          -- K6 Drift, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_drk <= (others => '0'); else s_drk <= v_s(5 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_bwmax <= s_wk; s_rim <= s_rimk; s_hue <= s_huek; s_sp <= s_spk;
                s_xdr <= s_xdr + resize(s_drk, 12);
                if s_roll = '1' then s_bph <= s_bph + 2; end if;
                s_two   <= registers_in(6)(0);
                s_rimon <= registers_in(6)(1);
                s_swap  <= registers_in(6)(2);
                s_roll  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                -- smear leak: 256 (none) .. 4 (very long)
                s_leak <= to_unsigned(256, 9) - resize(s_g(11 downto 4), 9) + 4;
            end if;

            -- derived
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cx <= '0' & s_vw(11 downto 1);
            s_cxp <= s_w(11 downto 1);
            if s_two = '1' then s_offv <= resize(s_sp, 11); else s_offv <= (others => '0'); end if;

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
    -- Source side: line-buffer write (video or oscillator); per-line profile,
    -- half-width, tear; the 65536 / half-width divide; commit at the rise
    ---------------------------------------------------------------------------
    p_src : process(clk)
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
            s_fall3 <= s_fall2;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
            end if;
            -- bottle half-width of the coming line: a neck (wmax / 4) above the
            -- shoulder at row 270, widening to wmax by row 810
            v_d := signed(resize(s_vy_n(19 downto 8), 13)) - 270;
            if v_d < 0 then s_dyc <= (others => '0'); elsif v_d > 540 then s_dyc <= to_unsigned(540, 10); else s_dyc <= unsigned(v_d(9 downto 0)); end if;
            s_vp <= s_dyc * s_bwmax;                           -- <= 540 * 551 (registered)
            if s_fall3 = '1' or s_zone = '1' then
                s_hw_n <= resize(s_bwmax(9 downto 2), 11) + resize(s_vp(19 downto 10), 11) - resize(s_vp(19 downto 12), 11);
            end if;
            -- band colour of the coming line (palette read in hblank)
            s_bprod <= s_vy_n(19 downto 8) * s_fy;
            s_bidx_n <= s_bprod(7 downto 0) + s_hue + s_bph;
            s_pal_ra <= s_swap & s_bidx_n;

            if data_in.avid = '1' and s_avq = '0' then
                s_hw <= s_hw_n;
                s_cxl <= signed(resize(s_cx, 13)) + resize(s_xdr, 13);
                s_by <= unsigned(s_pal_q(31 downto 22)); s_bu <= unsigned(s_pal_q(21 downto 12)); s_bv <= unsigned(s_pal_q(11 downto 2));
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_vxa <= (others => '0');
                s_bcol <= (others => '0');
                s_sel <= (others => '0');
            elsif g_avid = '1' then
                s_vxa <= s_vxa + s_xs;
                s_bcol <= s_bcol + 1;
                s_sel <= s_sel + 1;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: dx to both centres
            s_dxa1 <= signed(resize(s_vxa(12 downto 1), 14)) - resize(s_cxl, 14) + signed(resize(s_offv, 14));
            s_dxb1 <= signed(resize(s_vxa(12 downto 1), 14)) - resize(s_cxl, 14) - signed(resize(s_offv, 14));
            s_lb_ra <= s_bcol;
            s_seld(1) <= s_sel(0); s_seld2(1) <= s_sel(1);

            -- 2: |dx|, signs
            if s_dxa1 < 0 then s_adxa2 <= resize(unsigned(-s_dxa1), 13); s_sga2 <= '1';
            else s_adxa2 <= unsigned(s_dxa1(12 downto 0)); s_sga2 <= '0'; end if;
            if s_dxb1 < 0 then s_adxb2 <= resize(unsigned(-s_dxb1), 13); s_sgb2 <= '1';
            else s_adxb2 <= unsigned(s_dxb1(12 downto 0)); s_sgb2 <= '0'; end if;

            -- 3: nearest centre
            if s_adxa2 <= s_adxb2 then s_adx3 <= s_adxa2; s_sg(3) <= s_sga2;
            else s_adx3 <= s_adxb2; s_sg(3) <= s_sgb2; end if;

            -- 3: source (fabric copy of the buffer output register)
            s_y3 <= unsigned(s_lb_q(31 downto 22)); s_u3 <= unsigned(s_lb_q(21 downto 12)); s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 4: band mix (band chroma, luma = average with the source); inside test; edge distance
            s_y4 <= resize(shift_right(resize(s_y3, 11) + resize(s_by, 11), 1), 10);
            s_u4 <= s_bu; s_v4 <= s_bv;
            if s_adx3 < resize(s_hw, 13) then s_in(4) <= '1'; else s_in(4) <= '0'; end if;
            s_e4 <= signed(resize(s_adx3, 14)) - signed(resize(s_hw, 14));

            -- 5: smear A: difference from this lane's accumulator (4 lanes, 3-clock loop)
            s_dy5 <= signed(resize(s_y4, 11)) - s_accy(to_integer(unsigned'(s_seld2(4) & s_seld(4))));
            s_du5 <= signed(resize(s_u4, 11)) - s_accu(to_integer(unsigned'(s_seld2(4) & s_seld(4))));
            s_dv5 <= signed(resize(s_v4, 11)) - s_accv(to_integer(unsigned'(s_seld2(4) & s_seld(4))));
            s_ay5 <= s_accy(to_integer(unsigned'(s_seld2(4) & s_seld(4))));
            s_au5 <= s_accu(to_integer(unsigned'(s_seld2(4) & s_seld(4))));
            s_av5 <= s_accv(to_integer(unsigned'(s_seld2(4) & s_seld(4))));
            if s_e4 < 0 then s_ae5 <= resize(unsigned(-s_e4), 13); else s_ae5 <= unsigned(s_e4(12 downto 0)); end if;

            -- 6: smear B: partial products (leak split 5 + 4 bits)
            s_pyl6 <= s_dy5 * signed(resize(s_leak(4 downto 0), 6));
            s_pyh6 <= s_dy5 * signed(resize(s_leak(8 downto 5), 5));
            s_pul6 <= s_du5 * signed(resize(s_leak(4 downto 0), 6));
            s_puh6 <= s_du5 * signed(resize(s_leak(8 downto 5), 5));
            s_pvl6 <= s_dv5 * signed(resize(s_leak(4 downto 0), 6));
            s_pvh6 <= s_dv5 * signed(resize(s_leak(8 downto 5), 5));
            s_ay6 <= s_ay5; s_au6 <= s_au5; s_av6 <= s_av5;
            if s_ae5 < resize(s_rim, 13) then s_rimf(6) <= '1'; else s_rimf(6) <= '0'; end if;

            -- 7: smear C: full products
            s_py7 <= resize(s_pyl6, 21) + shift_left(resize(s_pyh6, 21), 5);
            s_pu7 <= resize(s_pul6, 21) + shift_left(resize(s_puh6, 21), 5);
            s_pv7 <= resize(s_pvl6, 21) + shift_left(resize(s_pvh6, 21), 5);
            s_ay7 <= s_ay6; s_au7 <= s_au6; s_av7 <= s_av6;

            -- 8: smear D: new accumulator value (written back to its lane; the
            --    lane's next read is 4 pixels after its last, exactly this loop)
            s_y8s <= s_ay7 + resize(shift_right(s_py7, 8), 11);
            s_u8s <= s_au7 + resize(shift_right(s_pu7, 8), 11);
            s_v8s <= s_av7 + resize(shift_right(s_pv7, 8), 11);
            if s_gav_sr(6) = '1' then
                s_accy(to_integer(unsigned'(s_seld2(7) & s_seld(7)))) <= s_ay7 + resize(shift_right(s_py7, 8), 11);
                s_accu(to_integer(unsigned'(s_seld2(7) & s_seld(7)))) <= s_au7 + resize(shift_right(s_pu7, 8), 11);
                s_accv(to_integer(unsigned'(s_seld2(7) & s_seld(7)))) <= s_av7 + resize(shift_right(s_pv7, 8), 11);
            end if;

            -- 9: clamp
            if s_y8s < 64 then s_y8 <= to_unsigned(64, 10); elsif s_y8s > 940 then s_y8 <= to_unsigned(940, 10); else s_y8 <= unsigned(s_y8s(9 downto 0)); end if;
            if s_u8s < 64 then s_u8 <= to_unsigned(64, 10); elsif s_u8s > 960 then s_u8 <= to_unsigned(960, 10); else s_u8 <= unsigned(s_u8s(9 downto 0)); end if;
            if s_v8s < 64 then s_v8 <= to_unsigned(64, 10); elsif s_v8s > 960 then s_v8 <= to_unsigned(960, 10); else s_v8 <= unsigned(s_v8s(9 downto 0)); end if;

            -- 10..11: pad; 12: compose bottles (pink) and rim (lime)
            s_y12 <= s_y8; s_u12 <= s_u8; s_v12 <= s_v8;
            if s_rimon = '1' and s_rimf(8) = '1' then
                if s_swap = '1' then s_y12 <= to_unsigned(752, 10); s_u12 <= to_unsigned(612, 10); s_v12 <= to_unsigned(512, 10);
                else s_y12 <= to_unsigned(800, 10); s_u12 <= to_unsigned(300, 10); s_v12 <= to_unsigned(200, 10); end if;
            elsif s_in(9) = '1' then
                if s_swap = '1' then s_y12 <= to_unsigned(800, 10); s_u12 <= to_unsigned(300, 10); s_v12 <= to_unsigned(200, 10);
                else s_y12 <= to_unsigned(752, 10); s_u12 <= to_unsigned(612, 10); s_v12 <= to_unsigned(512, 10); end if;
            end if;

            -- 13..18: output conditioning
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
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(17) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            -- delays
            for i in 4 to 6 loop s_sg(i) <= s_sg(i - 1); end loop;
            for i in 5 to 11 loop s_in(i) <= s_in(i - 1); end loop;
            for i in 7 to 11 loop s_rimf(i) <= s_rimf(i - 1); end loop;
            for i in 2 to 9 loop s_seld(i) <= s_seld(i - 1); s_seld2(i) <= s_seld2(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture bottleneck;
