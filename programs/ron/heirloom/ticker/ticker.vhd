-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ticker v0.1.0 -- Heirloom #20 (2026-10-20), from WIN_20230319_01_27_56.
--
-- The photo: monochrome -- columns of gray and white vertical bars shifted
-- sideways per row band, cyan dashes scattered through them, a dark
-- staircase region at the upper left: a digital readout gone wrong.
--
-- The program: row bands (K2) each slid sideways by a hashed offset scaled
-- by P12; column bars of pitch K1 in hashed grays, quantized to K4 levels;
-- cyan (or white) dashes where a second hash passes the K3 density.  The
-- video (or the oscillator), read through the same band offsets, has its
-- luma quantized to the same levels and its chroma killed, and is mixed
-- with the bars by K6; its brightest parts also light as dashes.
--
--   K1  Bars      bar pitch 16 / 32 / 64 / 128 px
--   K2  Bands     band height, 16 .. 271 lines (1080-line canvas)
--   K3  Dashes    dash density
--   K4  Levels    gray levels, 2 .. 64
--   K5  Reshuffle how often the hashes re-roll: never .. every field
--   K6  Bars      bars over the picture, 0 .. 100 %
--   S7  Dashes    Cyan / White
--   S8  Bands     Slide / Static (no sideways offsets)
--   S9  Invert    luma inverted
--   S10 Bars      Blocks (every band its own bars) / Columns (the same bars in every band)
--   S11 Source    Video / Osc
--   P12 SCRAMBLE  sideways offset amount: aligned .. scrambled
--
-- Band heights in virtual 1080-line units; bar pitch and offsets in real
-- pixels.  Generated avid; output conditioning before the blanking gate.
-- Latency 16 clocks.  16 EBR.  Colours: u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture ticker of program_top is
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

    constant C_CYN_U : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_CYN_V : unsigned(9 downto 0) := to_unsigned(663, 10);

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
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_j1 : unsigned(1 downto 0) := "01";
    signal s_live1 : std_logic := '0';
    signal s_k2h : unsigned(8 downto 0) := to_unsigned(80, 9);
    signal s_k3d : unsigned(7 downto 0) := (others => '0');
    signal s_k4l : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_k5r : unsigned(5 downto 0) := (others => '0');
    signal s_k6p : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_bsh : unsigned(1 downto 0) := "01";           -- block pitch shift (16 << bsh)
    signal s_bmask : unsigned(7 downto 0) := x"1F";
    signal s_bh : unsigned(8 downto 0) := to_unsigned(80, 9);
    signal s_dens : unsigned(7 downto 0) := (others => '0');
    signal s_lev : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_paint : unsigned(7 downto 0) := (others => '0');
    signal s_slide : unsigned(7 downto 0) := (others => '0');
    signal s_white, s_static, s_inv, s_cols : std_logic := '0';
    signal s_dashrow_n, s_dashrow : std_logic := '0';
    signal s_seed : unsigned(7 downto 0) := x"5A";
    signal s_fcnt : unsigned(5 downto 0) := (others => '0');

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line (band DDA)
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_cy_n : unsigned(17 downto 0) := (others => '0');
    signal s_bhq_n : unsigned(17 downto 0) := to_unsigned(80 * 256, 18);
    signal s_band_n : unsigned(5 downto 0) := (others => '0');
    signal s_bandf_n, s_bandf : std_logic := '1';
    signal s_bh8_n : unsigned(7 downto 0) := (others => '0');       -- band hash
    signal s_off_n : unsigned(15 downto 0) := (others => '0');
    signal s_band : unsigned(5 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');        -- read start pointer
    signal s_bst : unsigned(7 downto 0) := (others => '0');         -- block counter start (off mod pitch)
    signal s_bist : unsigned(7 downto 0) := (others => '0');        -- first block index (off / pitch)

    -- beam
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_bu : unsigned(7 downto 0) := (others => '0');
    signal s_bi : unsigned(7 downto 0) := (others => '0');
    signal s_b157 : unsigned(15 downto 0) := (others => '0');       -- bi * 157 (accumulated)
    signal s_bist157 : unsigned(15 downto 0) := (others => '0');    -- first block's bi * 157
    signal s_b61 : unsigned(15 downto 0) := (others => '0');        -- band * 61 + seed (per line)

    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_bi1 : unsigned(15 downto 0) := (others => '0');
    signal s_bs : t_sl(1 to 6) := (others => '0');                  -- block start flags
    signal s_ed : t_sl(1 to 6) := (others => '0');                  -- dash flags
    signal s_h2 : unsigned(15 downto 0) := (others => '0');
    signal s_h3 : unsigned(15 downto 0) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_hu, s_hv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_py4, s_pu4, s_pv4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y4, s_u4, s_v4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_dy5, s_du5, s_dv5 : signed(11 downto 0) := (others => '0');
    signal s_y5, s_u5, s_v5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_my6, s_mu6, s_mv6 : signed(20 downto 0) := (others => '0');
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

            v_k := unsigned(registers_in(0));                                  -- K1 Blocks (4 labels)
            if v_k >= 4 then s_live1 <= '1'; end if;
            if s_live1 = '0' and v_k < 4 then
                s_j1 <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_j1 <= v_k(9 downto 8);
            end if;
            s_k2h <= resize(unsigned(registers_in(1)(9 downto 2)), 9) + 16;     -- K2 Bands 16 .. 271
            s_k3d <= unsigned(registers_in(2)(9 downto 2));                     -- K3 Dashes
            s_k4l <= unsigned(registers_in(3)(9 downto 7));                     -- K4 Levels 0 .. 7
            s_k5r <= unsigned(registers_in(4)(9 downto 4));                     -- K5 Reshuffle 0 .. 63
            s_k6p <= unsigned(registers_in(5)(9 downto 2));                     -- K6 Bars
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_bsh <= s_j1;
                s_bh <= s_k2h;
                s_dens <= s_k3d;
                s_lev <= s_k4l;
                s_paint <= s_k6p;
                s_white  <= registers_in(6)(0);
                s_static <= registers_in(6)(1);
                s_inv    <= registers_in(6)(2);
                s_cols   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                -- reshuffle: the seed re-rolls when the field counter passes 64 - K5
                if s_k5r /= 0 and resize(s_fcnt, 7) + resize(s_k5r, 7) >= 64 then
                    s_fcnt <= (others => '0');
                    s_seed <= s_seed + 97;
                else
                    s_fcnt <= s_fcnt + 1;
                end if;
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_bsh) is
                when 0      => s_bmask <= x"0F";
                when 1      => s_bmask <= x"1F";
                when 2      => s_bmask <= x"3F";
                when others => s_bmask <= x"7F";
            end case;
            if s_static = '1' then s_slide <= (others => '0'); else s_slide <= s_g(11 downto 4); end if;

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

    ---------------------------------------------------------------------------
    -- Source side: line-buffer write; band DDA with hashed heights / offsets
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(18 downto 0);
        variable v_h : unsigned(7 downto 0);
        variable v_bh : unsigned(9 downto 0);
        variable v_o : unsigned(15 downto 0);
        variable v_off : unsigned(10 downto 0);
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

            -- band hash (per band, from its index and the seed)
            v_h := (s_band_n(5 downto 0) & "00") + (s_seed xor (s_band_n & s_band_n(1 downto 0)));
            v_h := v_h xor (v_h(3 downto 0) & v_h(7 downto 4));
            v_bh := resize(s_bh, 10) + resize(v_h(3 downto 0) * s_bh(8 downto 4), 10);
            v_o := v_h * s_slide;

            if s_zone = '1' then
                s_cy_n <= (others => '0');
                s_band_n <= (others => '0');
                s_bandf_n <= '1';
                s_bh8_n <= v_h;
                s_bhq_n <= v_bh(9 downto 0) & x"00";
                s_off_n <= v_o;
            elsif data_in.avid = '0' and s_avq = '1' then
                v_c := resize(s_cy_n, 19) + s_ys;
                if v_c >= s_bhq_n then
                    s_cy_n <= resize(v_c - s_bhq_n, 18);
                    s_band_n <= s_band_n + 1;
                    s_bandf_n <= '1';
                else
                    s_cy_n <= v_c(17 downto 0);
                    s_bandf_n <= '0';
                end if;
            elsif s_bandf_n = '1' and data_in.avid = '0' then
                -- the new band's hash settles during the blanking (band_n updated above)
                s_bh8_n <= v_h;
                s_bhq_n <= v_bh(9 downto 0) & x"00";
                s_off_n <= v_o;
            end if;

            if s_cy_n >= resize(s_bhq_n(17 downto 2), 18) and s_cy_n < resize(s_bhq_n(17 downto 2), 18) + resize(s_bhq_n(17 downto 1), 18) then
                s_dashrow_n <= '1';
            else
                s_dashrow_n <= '0';
            end if;
            if data_in.avid = '1' and s_avq = '0' then
                s_bandf <= s_bandf_n;
                s_band <= s_band_n;
                s_dashrow <= s_dashrow_n;
                -- offset in real pixels (clamped to the width); odd bands slide the other way
                if s_off_n(15 downto 6) >= s_wm1(10 downto 0) then v_off := s_wm1(10 downto 0); else v_off := resize(s_off_n(15 downto 6), 11); end if;
                if s_band_n(0) = '1' then
                    s_rst <= v_off;                                  -- read ahead: slide left
                else
                    if v_off = 0 then s_rst <= (others => '0'); else s_rst <= s_wm1(10 downto 0) + 1 - v_off; end if;
                end if;
                s_bst <= v_off(7 downto 0) and s_bmask;
                case to_integer(s_bsh) is
                    when 0      => s_bist <= v_off(10 downto 4) & '0';
                    when 1      => s_bist <= "00" & v_off(10 downto 5);
                    when 2      => s_bist <= "000" & v_off(10 downto 6);
                    when others => s_bist <= "0000" & v_off(10 downto 7);
                end case;
            end if;
            -- per-line hash terms (settle during the blanking)
            s_bist157 <= shift_left(resize(s_bist, 16), 7) + shift_left(resize(s_bist, 16), 5) - shift_left(resize(s_bist, 16), 2) + resize(s_bist, 16);
            if s_cols = '0' then
                s_b61 <= shift_left(resize(s_band, 16), 6) - shift_left(resize(s_band, 16), 1) - resize(s_band, 16) + resize(s_seed, 16);
            else
                s_b61 <= resize(s_seed, 16);
            end if;
        end if;
    end process p_src;

    -- beam: video read pointer (offset, wrapped), block counter / index
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
                s_bu <= s_bst;
                s_bi <= s_bist;
                s_b157 <= s_bist157;
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                if s_bu = s_bmask then
                    s_bu <= (others => '0');
                    s_bi <= s_bi + 1;
                    s_b157 <= s_b157 + 157;
                else
                    s_bu <= s_bu + 1;
                end if;
            end if;
        end if;
    end process p_beam;

    p_pix : process(clk)
        variable v_a : unsigned(15 downto 0);
        variable v_h : unsigned(15 downto 0);
        variable v_hu, v_hv : unsigned(9 downto 0);
        variable v_bl : unsigned(9 downto 0);
        variable v_bu : unsigned(1 downto 0);
        variable v_l : unsigned(9 downto 0);
        variable v_y, v_u, v_v : signed(12 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: read address; block index; block start / edge flags
            s_lb_ra <= s_rdp;
            s_bi1 <= s_b157;
            if s_bu = 0 then s_bs(1) <= '1'; else s_bs(1) <= '0'; end if;
            case to_integer(s_bsh) is
                when 0      => v_bu := s_bu(3 downto 2);
                when 1      => v_bu := s_bu(4 downto 3);
                when 2      => v_bu := s_bu(5 downto 4);
                when others => v_bu := s_bu(6 downto 5);
            end case;
            if (v_bu = "01" or v_bu = "10") and s_dashrow = '1' then s_ed(1) <= '1'; else s_ed(1) <= '0'; end if;

            -- 2: bar hash A: bi * 157 (accumulated in the beam) + band * 61 + seed (per line)
            s_h2 <= s_bi1 + s_b61;

            -- 3: hash B (xorshift); video fabric copy; chroma held per block
            v_h := s_h2 xor shift_right(s_h2, 5);
            v_h := v_h + shift_left(v_h, 3);
            s_h3 <= v_h;
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            if s_bs(2) = '1' then
                v_hu := unsigned(s_lb_q(21 downto 12)); v_hv := unsigned(s_lb_q(11 downto 2));
            else
                v_hu := s_hu; v_hv := s_hv;
            end if;
            s_hu <= v_hu; s_hv <= v_hv;
            s_u3 <= to_unsigned(512, 10); s_v3 <= to_unsigned(512, 10);

            -- 4: bar gray (hashed) and picture luma, both quantized to the levels;
            --    a dash where the dash hash passes the density (or the picture is
            --    near white); chroma neutral
            if s_y3 > 64 then v_l := s_y3 - 64; else v_l := (others => '0'); end if;
            v_bl := s_h3(9 downto 0);
            case to_integer(s_lev) is
                when 0      => v_l := v_l(9) & "000000000"; v_bl := v_bl(9) & "000000000";
                when 1      => v_l := v_l(9 downto 8) & "00000000"; v_bl := v_bl(9 downto 8) & "00000000";
                when 2      => v_l := v_l(9 downto 7) & "0000000"; v_bl := v_bl(9 downto 7) & "0000000";
                when 3      => v_l := v_l(9 downto 6) & "000000"; v_bl := v_bl(9 downto 6) & "000000";
                when 4      => v_l := v_l(9 downto 5) & "00000"; v_bl := v_bl(9 downto 5) & "00000";
                when others => v_l := v_l(9 downto 4) & "0000"; v_bl := v_bl(9 downto 4) & "0000";
            end case;
            if v_bl > 876 then v_bl := to_unsigned(876, 10); end if;
            if v_l > 876 then v_l := to_unsigned(876, 10); end if;
            s_py4 <= v_bl + 64; s_pu4 <= to_unsigned(512, 10); s_pv4 <= to_unsigned(512, 10);
            s_y4 <= v_l + 64; s_u4 <= s_u3; s_v4 <= s_v3;
            if (s_ed(3) = '1' and s_h3(15 downto 8) < s_dens) or s_y3 > 900 then s_ed(4) <= '1'; else s_ed(4) <= '0'; end if;

            -- 5: differences block - picture
            s_dy5 <= signed(resize(s_py4, 12)) - signed(resize(s_y4, 12));
            s_du5 <= signed(resize(s_pu4, 12)) - signed(resize(s_u4, 12));
            s_dv5 <= signed(resize(s_pv4, 12)) - signed(resize(s_v4, 12));
            s_y5 <= s_y4; s_u5 <= s_u4; s_v5 <= s_v4;

            -- 6: paint products
            s_my6 <= s_dy5 * signed(resize(s_paint, 9));
            s_mu6 <= s_du5 * signed(resize(s_paint, 9));
            s_mv6 <= s_dv5 * signed(resize(s_paint, 9));
            s_y6 <= s_y5; s_u6 <= s_u5; s_v6 <= s_v5;

            -- 7: bars over the picture; dashes; invert
            v_y := signed(resize(s_y6, 13)) + resize(shift_right(s_my6, 8), 13);
            if s_inv = '1' then v_y := to_signed(1004, 13) - v_y; end if;
            if s_ed(6) = '1' then
                if s_white = '1' then
                    s_y7 <= to_unsigned(940, 10); s_u7 <= to_unsigned(512, 10); s_v7 <= to_unsigned(512, 10);
                else
                    s_y7 <= to_unsigned(678, 10); s_u7 <= C_CYN_U; s_v7 <= C_CYN_V;
                end if;
            else
                s_y7 <= f_clamp(v_y); s_u7 <= to_unsigned(512, 10); s_v7 <= to_unsigned(512, 10);
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
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;

            for i in 2 to 6 loop s_bs(i) <= s_bs(i - 1); end loop;
            s_ed(2) <= s_ed(1); s_ed(3) <= s_ed(2); s_ed(5) <= s_ed(4); s_ed(6) <= s_ed(5);
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ticker;
