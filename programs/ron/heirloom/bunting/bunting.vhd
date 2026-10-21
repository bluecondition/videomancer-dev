-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Bunting v0.1.0 -- Heirloom #35 (2026-10-20), from WIN_20230521_23_37_25.
--
-- The photo: a white ground with big yellow and cyan triangles -- a fan of
-- wedges radiating from the screen centre, mirrored top and bottom -- and
-- rows of stepped glyph blocks (red above the centre line, blue below,
-- gray at the right) laid out four across and four down.
--
-- The program: two linear fields L1 = a x + b y and L2 = a x - b y about the
-- centre classify every pixel into wedge sectors (P12 sets the fan angle);
-- K3 chooses which sectors are painted in the K4 colour pair, the rest stay
-- white.  Glyph rows: a 4 x 4-cell staircase glyph repeated on a grid (K1
-- cell size, K2 column pitch, K5 row pitch), red above / blue below (S10
-- swaps), gray in the right quarter; the glyph cells carry the picture
-- (or the oscillator) as a mosaic, mixed in by K6.
--
--   K1  Cell     glyph cell size, 8 .. 135 px (square)
--   K2  Columns  glyph column pitch, 5 .. 12 cells
--   K3  Wedges   which sector classes are painted (15 combinations)
--   K4  Pair     wedge colours: Yellow+Cyan / Magenta+Green / Red+Blue / Orange+Violet
--   K5  Rows     glyph row pitch, 5 .. 20 cells
--   K6  Picture  the picture mixed into the glyph cells, 0 .. 100 %
--   S7  Mirror   wedges mirrored top / bottom (fan from the centre line)
--   S8  Glyphs   On / Off
--   S9  Drift    glyph columns step sideways every field
--   S10 Rows     Red over Blue / Blue over Red
--   S11 Source   Video / Osc
--   P12 FAN      fan angle: steep (wide side wedges) .. flat (wide top and bottom wedges)
--
-- Virtual 1080-line geometry; generated avid; output conditioning before the
-- blanking gate.  Latency 16 clocks.  16 EBR.  Colours: u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture bunting of program_top is
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
    function f_clamp(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 960 then return to_unsigned(960, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- wedge colour pairs {y, u = Cr, v = Cb}: 4 pairs x 2
    type t_col is array (0 to 2) of integer;
    type t_pal is array (0 to 7) of t_col;
    constant C_PAIR : t_pal := (
        (840, 584, 64), (678, 64, 663),       -- yellow, cyan
        (426, 887, 808), (580, 136, 216),     -- magenta, green
        (328, 960, 360), (164, 440, 960),     -- red, blue
        (584, 772, 212), (349, 756, 852));    -- orange, violet
    -- glyph colours: red, blue, gray, dark
    constant C_GLY : t_pal := (
        (328, 960, 360), (164, 440, 960), (480, 512, 512), (200, 512, 512),
        (328, 960, 360), (164, 440, 960), (480, 512, 512), (200, 512, 512));
    -- 4 x 4 staircase glyph, row-major (row 0 at the top)
    constant C_BMP : std_logic_vector(15 downto 0) := "1111" & "1100" & "1111" & "0110";

    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    signal s_src : std_logic := '0';
    signal s_ox, s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_k1c : unsigned(7 downto 0) := to_unsigned(40, 8);
    signal s_k2p : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_k3m : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal s_j4 : unsigned(1 downto 0) := "00";
    signal s_live4 : std_logic := '0';
    signal s_k5p : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal s_k6m : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_cw : unsigned(7 downto 0) := to_unsigned(40, 8);
    signal s_cwm1 : unsigned(7 downto 0) := to_unsigned(39, 8);
    signal s_ch : unsigned(8 downto 0) := to_unsigned(40, 9);
    signal s_chq : unsigned(16 downto 0) := (others => '0');
    signal s_gpitch : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_rpitch : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal s_mask : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal s_pair : unsigned(1 downto 0) := "00";
    signal s_mix : unsigned(7 downto 0) := (others => '0');
    signal s_mirror, s_glyphs, s_drift, s_swap : std_logic := '0';
    signal s_dc : unsigned(3 downto 0) := (others => '0');          -- drift, in cells
    signal s_a : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal s_b : unsigned(7 downto 0) := to_unsigned(160, 8);
    signal s_axs : unsigned(9 downto 0) := to_unsigned(192, 10);     -- a * xs (Q.1 per pixel -> a per virtual unit)
    signal s_amul : unsigned(23 downto 0) := (others => '0');        -- a * vw
    signal s_bys : unsigned(23 downto 0) := (others => '0');         -- b * ys
    signal s_b540 : unsigned(23 downto 0) := (others => '0');        -- b * 540
    signal s_q3 : unsigned(11 downto 0) := to_unsigned(1440, 12);    -- right quarter start (3/4 vw)
    -- serial unit
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(1 downto 0) := (others => '0');

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_cy_n : unsigned(16 downto 0) := (others => '0');
    signal s_gy_n : unsigned(4 downto 0) := (others => '0');
    signal s_rowf_n, s_rowf : std_logic := '1';
    signal s_bl_n : signed(23 downto 0) := (others => '0');          -- b * dy (signed)
    signal s_bl : signed(20 downto 0) := (others => '0');
    signal s_gy : unsigned(4 downto 0) := (others => '0');
    signal s_lower : std_logic := '0';

    -- beam
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_cu, s_cs : unsigned(10 downto 0) := (others => '0');
    signal s_gx : unsigned(3 downto 0) := (others => '0');
    signal s_acc : signed(20 downto 0) := (others => '0');            -- a * dx
    signal s_vx : unsigned(12 downto 0) := (others => '0');

    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_l1, s_l2 : signed(21 downto 0) := (others => '0');
    signal s_gb : t_sl(1 to 7) := (others => '0');
    signal s_rq : t_sl(1 to 3) := (others => '0');
    signal s_wy2, s_wu2, s_wv2 : unsigned(9 downto 0) := to_unsigned(940, 10);
    signal s_wy, s_wu, s_wv : t_u10(3 to 7) := (others => to_unsigned(940, 10));
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_gy3, s_gu3, s_gv3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_dy4, s_du4, s_dv4 : signed(11 downto 0) := (others => '0');
    signal s_gy4, s_gu4, s_gv4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_my5, s_mu5, s_mv5 : signed(20 downto 0) := (others => '0');
    signal s_gy5, s_gu5, s_gv5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
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

    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_ch : unsigned(9 downto 0);
        variable v_dc : unsigned(3 downto 0);
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

            s_k1c <= resize(unsigned(registers_in(0)(9 downto 3)), 8) + 8;      -- K1 Cell 8 .. 135 px
            s_k2p <= resize(unsigned(registers_in(1)(9 downto 7)), 4) + 5;      -- K2 Columns 5 .. 12 cells
            s_k3m <= unsigned(registers_in(2)(9 downto 6)) + 1;                 -- K3 Wedges mask 1 .. 15 (16 wraps to 1)
            v_k := unsigned(registers_in(3));                                  -- K4 Pair (4 labels)
            if v_k >= 4 then s_live4 <= '1'; end if;
            if s_live4 = '0' and v_k < 4 then
                s_j4 <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_j4 <= v_k(9 downto 8);
            end if;
            s_k5p <= resize(unsigned(registers_in(4)(9 downto 6)), 5) + 5;      -- K5 Rows 5 .. 20 cells
            s_k6m <= unsigned(registers_in(5)(9 downto 2));                     -- K6 Picture
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_cw <= s_k1c;
                s_gpitch <= s_k2p;
                if s_k3m = 0 then s_mask <= to_unsigned(15, 4); else s_mask <= s_k3m; end if;
                s_pair <= s_j4;
                s_rpitch <= s_k5p;
                s_mix <= s_k6m;
                s_mirror <= registers_in(6)(0);
                s_glyphs <= not registers_in(6)(1);
                s_drift  <= registers_in(6)(2);
                s_swap   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                if s_drift = '1' then
                    v_dc := s_dc + 1;
                    if v_dc >= s_gpitch then v_dc := (others => '0'); end if;
                    s_dc <= v_dc;
                end if;
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_q3 <= resize(s_vw(11 downto 1), 12) + resize(s_vw(11 downto 2), 12);
            s_cwm1 <= s_cw - 1;
            case to_integer(s_xs) is
                when 4      => v_ch := shift_left(resize(s_cw, 10), 1);
                when 3      => v_ch := resize(s_cw, 10) + resize(s_cw(7 downto 1), 10);
                when others => v_ch := resize(s_cw, 10);
            end case;
            s_ch <= v_ch(8 downto 0);
            s_chq <= s_ch & x"00";
            -- fan: a = 32 + p12 / 8, b = 256 - a
            s_a <= to_unsigned(32, 8) + resize(s_g(11 downto 5), 8);
            s_b <= to_unsigned(255, 8) - s_a;
            case to_integer(s_xs) is
                when 4      => s_axs <= shift_left(resize(s_a, 10), 1);
                when 3      => s_axs <= resize(s_a, 10) + resize(s_a(7 downto 1), 10);
                when others => s_axs <= resize(s_a, 10);
            end case;

            -- serial unit: jobs 0 a*vw, 1 b*ys, 2 b*540
            if s_zc = 4 then
                s_ma <= resize(s_a, 24); s_mb <= s_vw; s_mjob <= "00"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 24 then
                s_ma <= resize(s_b, 24); s_mb <= s_ys; s_mjob <= "01"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_zc = 44 then
                s_ma <= resize(s_b, 24); s_mb <= to_unsigned(540, 12); s_mjob <= "10"; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "00"   => s_amul <= s_macc;
                    when "01"   => s_bys <= s_macc;
                    when others => s_b540 <= s_macc;
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

    p_src : process(clk)
        variable v_c : unsigned(17 downto 0);
        variable v_gy : unsigned(4 downto 0);
        variable v_bl : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_otu <= to_unsigned(320, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot) & '0', 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot), 10);
            s_otv <= to_unsigned(320, 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot), 10);

            s_lb_we <= data_in.avid and s_rowf;
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_oty) & std_logic_vector(s_otu) & std_logic_vector(s_otv) & "00";
            else
                s_lb_wd <= data_in.y & data_in.u & data_in.v & "00";
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_cy_n <= (others => '0');
                s_gy_n <= (others => '0');
                s_rowf_n <= '1';
                s_bl_n <= -signed(s_b540);
            elsif data_in.avid = '0' and s_avq = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_bl_n <= s_bl_n + signed(resize(s_bys(23 downto 8), 24));
                v_c := resize(s_cy_n, 18) + s_ys;
                if v_c >= s_chq then
                    s_cy_n <= resize(v_c - s_chq, 17);
                    s_rowf_n <= '1';
                    v_gy := s_gy_n + 1;
                    if v_gy >= s_rpitch then v_gy := (others => '0'); end if;
                    s_gy_n <= v_gy;
                else
                    s_cy_n <= v_c(16 downto 0);
                    s_rowf_n <= '0';
                end if;
            end if;
            if data_in.avid = '1' and s_avq = '0' then
                s_rowf <= s_rowf_n;
                s_gy <= s_gy_n;
                v_bl := s_bl_n;
                if s_mirror = '1' and v_bl < 0 then v_bl := -v_bl; end if;
                s_bl <= v_bl(20 downto 0);
                if s_vy_n(19 downto 8) >= 540 then s_lower <= '1'; else s_lower <= '0'; end if;
            end if;
        end if;
    end process p_src;

    p_beam : process(clk)
        variable v_gx : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_col <= (others => '0');
                s_cu <= (others => '0');
                s_cs <= (others => '0');
                s_gx <= s_dc;
                s_acc <= -signed(resize(s_amul(23 downto 1), 21));
                s_vx <= (others => '0');
            elsif g_avid = '1' then
                s_col <= s_col + 1;
                s_vx <= s_vx + s_xs;
                s_acc <= s_acc + signed(resize(s_axs, 21));
                if s_cu = resize(s_cwm1, 11) then
                    s_cu <= (others => '0');
                    s_cs <= s_col + 1;
                    v_gx := s_gx + 1;
                    if v_gx >= s_gpitch then v_gx := (others => '0'); end if;
                    s_gx <= v_gx;
                else
                    s_cu <= s_cu + 1;
                end if;
            end if;
        end if;
    end process p_beam;

    p_pix : process(clk)
        variable v_bi : integer range 0 to 15;
        variable v_sc2 : unsigned(1 downto 0);
        variable v_pi : integer range 0 to 7;
        variable v_y, v_u, v_v : signed(12 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: mosaic read; wedge fields; glyph bit; right quarter
            s_lb_ra <= s_cs;
            s_l1 <= resize(s_acc, 22) + resize(s_bl, 22);
            s_l2 <= resize(s_acc, 22) - resize(s_bl, 22);
            if s_glyphs = '1' and s_gx < 4 and s_gy < 4 then
                v_bi := to_integer(s_gy(1 downto 0) & s_gx(1 downto 0));
                s_gb(1) <= C_BMP(15 - v_bi);
            else
                s_gb(1) <= '0';
            end if;
            if s_vx(12 downto 1) >= s_q3 then s_rq(1) <= '1'; else s_rq(1) <= '0'; end if;

            -- 2: sector class -> wedge colour (or white)
            v_sc2 := s_l1(21) & s_l2(21);
            if s_mask(to_integer(v_sc2)) = '1' then
                if v_sc2 = "00" or v_sc2 = "11" then v_pi := to_integer(s_pair & '0'); else v_pi := to_integer(s_pair & '1'); end if;
                s_wy2 <= to_unsigned(C_PAIR(v_pi)(0), 10);
                s_wu2 <= to_unsigned(C_PAIR(v_pi)(1), 10);
                s_wv2 <= to_unsigned(C_PAIR(v_pi)(2), 10);
            else
                s_wy2 <= to_unsigned(940, 10); s_wu2 <= to_unsigned(512, 10); s_wv2 <= to_unsigned(512, 10);
            end if;

            -- 3: video fabric copy; glyph base colour (red / blue by half, gray at the right)
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));
            if s_rq(2) = '1' then
                v_pi := 2;
            elsif (s_lower xor s_swap) = '1' then
                v_pi := 1;
            else
                v_pi := 0;
            end if;
            s_gy3 <= to_unsigned(C_GLY(v_pi)(0), 10);
            s_gu3 <= to_unsigned(C_GLY(v_pi)(1), 10);
            s_gv3 <= to_unsigned(C_GLY(v_pi)(2), 10);
            s_wy(3) <= s_wy2; s_wu(3) <= s_wu2; s_wv(3) <= s_wv2;

            -- 4: differences picture - glyph colour
            s_dy4 <= signed(resize(s_y3, 12)) - signed(resize(s_gy3, 12));
            s_du4 <= signed(resize(s_u3, 12)) - signed(resize(s_gu3, 12));
            s_dv4 <= signed(resize(s_v3, 12)) - signed(resize(s_gv3, 12));
            s_gy4 <= s_gy3; s_gu4 <= s_gu3; s_gv4 <= s_gv3;

            -- 5: mix products
            s_my5 <= s_dy4 * signed(resize(s_mix, 9));
            s_mu5 <= s_du4 * signed(resize(s_mix, 9));
            s_mv5 <= s_dv4 * signed(resize(s_mix, 9));
            s_gy5 <= s_gy4; s_gu5 <= s_gu4; s_gv5 <= s_gv4;

            -- 6: glyph cell colour
            v_y := signed(resize(s_gy5, 13)) + resize(shift_right(s_my5, 8), 13);
            v_u := signed(resize(s_gu5, 13)) + resize(shift_right(s_mu5, 8), 13);
            v_v := signed(resize(s_gv5, 13)) + resize(shift_right(s_mv5, 8), 13);
            s_y6 <= f_clamp(v_y); s_u6 <= f_clamp(v_u); s_v6 <= f_clamp(v_v);

            -- 7: compose: glyph over wedges over white
            if s_gb(6) = '1' then
                s_y7 <= s_y6; s_u7 <= s_u6; s_v7 <= s_v6;
            else
                s_y7 <= s_wy(6); s_u7 <= s_wu(6); s_v7 <= s_wv(6);
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
            if s_yf13 > 64 then v_dl := s_yf13 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(12) = '1' then
                s_y14 <= s_yf13 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;

            for i in 2 to 7 loop s_gb(i) <= s_gb(i - 1); end loop;
            for i in 2 to 3 loop s_rq(i) <= s_rq(i - 1); end loop;
            for i in 4 to 7 loop s_wy(i) <= s_wy(i - 1); s_wu(i) <= s_wu(i - 1); s_wv(i) <= s_wv(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture bunting;
