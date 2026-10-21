-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Starburst v0.1.0 -- Heirloom #3 (2026-10-20), from WIN_20221122_23_37_36.
--
-- The photo: a four-fold mirror kaleidoscope of saddle-shaped contour bands:
-- where |x| - |y| is near zero the bands make gray nested X's, further out
-- they turn into cyan / magenta / yellow / white / black chevrons, every
-- band edge serrated with spiky teeth.
--
-- The program: the screen is a grid of N x N cells; in each, the field
-- s = a |x| - b |y| (measured from the cell centre, so every cell is its own
-- mirror) is cut into bands and each band painted from a 16-colour palette.
-- The video, mirrored horizontally in every cell, etches into the field: its
-- luma pushes the bands, so the picture appears as warped contour bands
-- (P12 = how hard).
--
--   K1  Cells    cells across (and down), 1 .. 8
--   K2  Spread   band zoom: a few wide bands .. hundreds of fine ones
--   K3  Spikes   serration of the band edges (8-line teeth)
--   K4  Tilt     X opening angle: wide-open V's .. tall ones
--   K5  Drift    bands flow outward / inward, bipolar (centre = still)
--   K6  Palette  CMYK / RGB / Warm / Cool / Mono / Neon / Pastel / Ember
--   S7  Lattice  Mirror (each cell folded about its centre) / Tile (sawtooth)
--   S8  Field    Saddle (a|x| - b|y|: X's and chevrons) / Diamond (a|x| + b|y|)
--   S9  Noise    grain added to the field (fringed, sparkling edges)
--   S10 Negative palette inverted
--   S11 Chroma   from the palette / from the mirrored video (palette luma)
--   P12 ETCH     video depth: 0 = the pure pattern .. the picture carved in bands
--
-- Geometry in virtual 1080-line units; the mirrored video read is real
-- pixels.  Per-frame divides (cell widths) and products on one serial unit,
-- the per-line b|y| product on the same unit.  Generated avid; output
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

architecture starburst of program_top is
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

    -- cell height (virtual lines) for 1 .. 8 cells down
    type t_ch is array (0 to 7) of integer;
    constant C_CH : t_ch := (1080, 540, 360, 270, 216, 180, 154, 135);

    -- 8 palettes x 16 bands, {y, u = Cr, v = Cb}
    type t_col is array (0 to 2) of integer;
    constant WHT : t_col := (940, 512, 512);
    constant LGR : t_col := (700, 512, 512);
    constant GRY : t_col := (480, 512, 512);
    constant DGR : t_col := (260, 512, 512);
    constant BLK : t_col := (64, 512, 512);
    constant YEL : t_col := (840, 584, 64);
    constant CYN : t_col := (678, 64, 663);
    constant MAG : t_col := (426, 887, 808);
    constant RED : t_col := (328, 960, 360);
    constant GRN : t_col := (580, 136, 216);
    constant BLU : t_col := (164, 440, 960);
    constant ORG : t_col := (584, 772, 212);
    constant VIO : t_col := (349, 756, 852);
    constant IND : t_col := (192, 608, 696);
    constant PNK : t_col := (752, 612, 512);
    constant LAV : t_col := (747, 516, 594);
    constant MNT : t_col := (760, 420, 440);
    constant CRM : t_col := (860, 540, 470);
    constant SKY : t_col := (740, 470, 600);
    type t_pdef is array (0 to 127) of t_col;
    constant C_PDEF : t_pdef := (
        WHT, LGR, GRY, DGR, BLK, YEL, CYN, WHT, MAG, YEL, BLK, CYN, MAG, WHT, YEL, BLK,   -- CMYK
        WHT, RED, GRN, BLU, BLK, RED, YEL, GRN, CYN, BLU, MAG, WHT, BLK, RED, GRN, BLU,   -- RGB
        WHT, YEL, ORG, RED, DGR, BLK, RED, ORG, YEL, WHT, ORG, RED, BLK, YEL, ORG, RED,   -- Warm
        WHT, CYN, BLU, IND, BLK, VIO, BLU, CYN, WHT, BLU, IND, BLK, CYN, VIO, BLU, CYN,   -- Cool
        WHT, LGR, GRY, DGR, BLK, DGR, GRY, LGR, WHT, LGR, GRY, DGR, BLK, DGR, GRY, LGR,   -- Mono
        BLK, MAG, BLK, CYN, BLK, YEL, BLK, GRN, BLK, MAG, BLK, CYN, BLK, RED, BLK, BLU,   -- Neon
        WHT, PNK, LAV, MNT, CRM, SKY, PNK, WHT, LAV, MNT, CRM, WHT, SKY, PNK, LAV, MNT,   -- Pastel
        BLK, DGR, RED, ORG, YEL, WHT, YEL, ORG, RED, DGR, BLK, RED, ORG, YEL, ORG, RED);  -- Ember
    type t_pal is array (0 to 127) of std_logic_vector(31 downto 0);
    function f_pal return t_pal is
        variable r : t_pal;
    begin
        for i in 0 to 127 loop
            r(i) := std_logic_vector(to_unsigned(C_PDEF(i)(0), 10)) & std_logic_vector(to_unsigned(C_PDEF(i)(1), 10))
                    & std_logic_vector(to_unsigned(C_PDEF(i)(2), 10)) & "00";
        end loop;
        return r;
    end function;
    signal pal : t_pal := f_pal;
    signal s_pal_ra : unsigned(6 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    -- line buffer {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

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
    signal s_j1 : unsigned(2 downto 0) := to_unsigned(3, 3);          -- cells - 1
    signal s_zk : unsigned(8 downto 0) := to_unsigned(32, 9);
    signal s_k3 : unsigned(9 downto 0) := (others => '0');
    signal s_ta : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_j6 : unsigned(2 downto 0) := (others => '0');
    signal s_live6 : std_logic := '0';
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_n : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_cwpx, s_cwv : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_cwpxm1 : unsigned(10 downto 0) := to_unsigned(479, 11);
    signal s_cwv2 : unsigned(12 downto 0) := to_unsigned(960, 13);  -- 2 cwv (Q.1 cell width)
    signal s_ch : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_chq : unsigned(18 downto 0) := (others => '0');       -- ch << 8
    signal s_chh : unsigned(10 downto 0) := to_unsigned(135, 11);  -- ch / 2
    signal s_a, s_b : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_tb : unsigned(7 downto 0) := to_unsigned(127, 8);     -- 255 - ta
    signal s_spk : unsigned(9 downto 0) := (others => '0');
    signal s_ph : unsigned(23 downto 0) := (others => '0');
    signal s_etch : unsigned(9 downto 0) := (others => '0');
    signal s_pali : unsigned(2 downto 0) := (others => '0');
    signal s_tile, s_diam, s_noise, s_neg, s_vchr : std_logic := '0';
    -- serial unit (multiply / divide)
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun, s_mdiv : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');
    signal s_dnum : unsigned(11 downto 0) := (others => '0');
    signal s_drem : unsigned(12 downto 0) := (others => '0');
    signal s_dquo : unsigned(11 downto 0) := (others => '0');
    signal s_lfsr : unsigned(15 downto 0) := x"BEEF";

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
    signal s_cv_n : unsigned(18 downto 0) := (others => '0');
    signal s_fv_n : unsigned(10 downto 0) := (others => '0');
    signal s_fvb_n, s_fvb : unsigned(23 downto 0) := (others => '0');
    signal s_spl_n, s_spl : unsigned(14 downto 0) := (others => '0');

    -- beam state
    signal s_cu : unsigned(12 downto 0) := (others => '0');          -- Q.1 within the cell
    signal s_cupx, s_cdpx, s_cs : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_fu1 : unsigned(12 downto 0) := (others => '0');
    signal s_m1, s_cs1 : unsigned(10 downto 0) := (others => '0');
    signal s_plo2 : unsigned(20 downto 0) := (others => '0');
    signal s_phi2 : unsigned(16 downto 0) := (others => '0');
    signal s_rd2, s_rd3 : unsigned(10 downto 0) := (others => '0');
    signal s_pa3 : unsigned(24 downto 0) := (others => '0');
    signal s_s4, s_s5, s_s6, s_s7, s_s8 : signed(26 downto 0) := (others => '0');
    signal s_y6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ud, s_vd : t_u10(6 to 12) := (others => to_unsigned(512, 10));
    signal s_m7 : unsigned(19 downto 0) := (others => '0');
    signal s_q9 : signed(12 downto 0) := (others => '0');
    signal s_py11, s_pu11, s_pv11 : unsigned(9 downto 0) := to_unsigned(64, 10);
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
            s_pal_q <= pal(to_integer(s_pal_ra));
        end if;
    end process p_pal;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial unit
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_r : unsigned(12 downto 0);
        variable v_ph : signed(24 downto 0);
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

            -- K1 Cells (8 zones, +-6 count deadband after warm-up)
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(6 downto 0) >= 6 or v_k(9 downto 7) = 0) and
                                (v_k(6 downto 0) <= 121 or v_k(9 downto 7) = 7)) then
                s_j1 <= v_k(9 downto 7);
            end if;
            s_zk <= resize(unsigned(registers_in(1)(9 downto 2)), 9) + 1;    -- K2 Spread 1 .. 256
            s_k3 <= unsigned(registers_in(2));                                -- K3 Spikes
            s_ta <= unsigned(registers_in(3)(9 downto 2));                    -- K4 Tilt
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;       -- K5 Drift, deadband
            if v_s > -8 and v_s < 8 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            -- K6 Palette (8 labels; the firmware loads the default as the index)
            v_k := unsigned(registers_in(5));
            if v_k >= 8 then s_live6 <= '1'; end if;
            if s_live6 = '0' and v_k < 8 then
                s_j6 <= v_k(2 downto 0);
            elsif s_warm /= 15 or ((v_k(6 downto 0) >= 6 or v_k(9 downto 7) = 0) and
                                   (v_k(6 downto 0) <= 121 or v_k(9 downto 7) = 7)) then
                s_j6 <= v_k(9 downto 7);
            end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_n <= s_j1;
                s_pali <= s_j6;
                s_spk <= s_k3;
                s_tile  <= registers_in(6)(0);
                s_diam  <= registers_in(6)(1);
                s_noise <= registers_in(6)(2);
                s_neg   <= registers_in(6)(3);
                s_vchr  <= registers_in(6)(4);
                v_ph := signed(resize(s_ph, 25)) + shift_left(resize(s_k5r, 25), 6);
                s_ph <= unsigned(v_ph(23 downto 0));
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
            s_ch <= to_unsigned(C_CH(to_integer(s_n)), 11);
            s_chq <= s_ch & x"00";
            s_chh <= '0' & s_ch(10 downto 1);
            s_tb <= not s_ta;
            s_etch <= s_g(11 downto 2);
            s_cwpxm1 <= s_cwpx(10 downto 0) - 1;
            s_cwv2 <= s_cwv & '0';

            -- serial unit.  Divides: job 0 w / N, job 1 vw / N (12 steps).
            -- Products: job 2 ta x zk, job 3 (255 - ta) x zk, job 4 fv x b.
            if s_zc = 4 then
                s_dnum <= s_w; s_drem <= (others => '0'); s_mb <= resize(s_n, 12) + 1;
                s_mjob <= "000"; s_mdiv <= '1'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 24 then
                s_dnum <= s_vw; s_drem <= (others => '0'); s_mb <= resize(s_n, 12) + 1;
                s_mjob <= "001"; s_mdiv <= '1'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 44 then
                s_ma <= resize(s_ta, 24); s_mb <= resize(s_zk, 12); s_macc <= (others => '0');
                s_mjob <= "010"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 64 then
                s_ma <= resize(s_tb, 24); s_mb <= resize(s_zk, 12); s_macc <= (others => '0');
                s_mjob <= "011"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 84 or s_fall2 = '1' then
                s_ma <= resize(s_fv_n, 24); s_mb <= s_b; s_macc <= (others => '0');
                s_mjob <= "100"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mdiv = '1' then
                    v_r := s_drem(11 downto 0) & s_dnum(11);
                    s_dnum <= s_dnum(10 downto 0) & '0';
                    if v_r >= resize(s_mb, 13) then
                        s_drem <= v_r - resize(s_mb, 13);
                        s_dquo <= s_dquo(10 downto 0) & '1';
                    else
                        s_drem <= v_r;
                        s_dquo <= s_dquo(10 downto 0) & '0';
                    end if;
                else
                    if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                    s_ma <= s_ma(22 downto 0) & '0';
                    s_mb <= '0' & s_mb(11 downto 1);
                end if;
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_cwpx <= s_dquo;
                    when "001"  => s_cwv <= s_dquo;
                    when "010"  => s_a <= s_macc(16 downto 5);
                    when "011"  => s_b <= s_macc(16 downto 5);
                    when others => s_fvb_n <= s_macc;
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
    -- Source side: line-buffer write; per-line cell-row DDA, |dy|, spikes
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(19 downto 0);
        variable v_d : signed(11 downto 0);
        variable v_sp : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            s_lb_wd <= data_in.y & data_in.u & data_in.v & "00";
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;

            s_fall1 <= '0';
            if data_in.avid = '0' and s_avq = '1' then s_fall1 <= '1'; end if;
            s_fall2 <= s_fall1;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_cv_n <= (others => '0');
                s_fv_n <= s_chh;
                s_spl_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                v_c := resize(s_cv_n, 20) + s_ys;
                if v_c >= s_chq then s_cv_n <= resize(v_c - s_chq, 19); else s_cv_n <= v_c(18 downto 0); end if;
            elsif s_fall2 = '1' then
                v_d := signed(resize(s_cv_n(18 downto 8), 12)) - signed(resize(s_chh, 12));
                if v_d < 0 then s_fv_n <= resize(unsigned(-v_d), 11); else s_fv_n <= unsigned(v_d(10 downto 0)); end if;
                -- serration: (line mod 8) x K3, up to ~1.75 bands
                v_sp := (others => '0');
                if s_vy_n(8) = '1' then v_sp := v_sp + resize(s_spk, 13); end if;
                if s_vy_n(9) = '1' then v_sp := v_sp + shift_left(resize(s_spk, 13), 1); end if;
                if s_vy_n(10) = '1' then v_sp := v_sp + shift_left(resize(s_spk, 13), 2); end if;
                s_spl_n <= v_sp & "00";
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_fvb <= s_fvb_n;
                s_spl <= s_spl_n;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state: cell position (Q.1 virtual), real-pixel cell counters
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_cu <= (others => '0');
                s_cupx <= (others => '0');
                s_cdpx <= s_cwpxm1;
                s_cs <= (others => '0');
            elsif g_avid = '1' then
                v_n := resize(s_cu, 14) + s_xs;
                if v_n >= s_cwv2 then s_cu <= resize(v_n - s_cwv2, 13); else s_cu <= v_n(12 downto 0); end if;
                if s_cupx = s_cwpxm1 then
                    s_cupx <= (others => '0');
                    s_cdpx <= s_cwpxm1;
                    s_cs <= s_cs + s_cwpx(10 downto 0);
                else
                    s_cupx <= s_cupx + 1;
                    s_cdpx <= s_cdpx - 1;
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_f : signed(13 downto 0);
        variable v_q : signed(12 downto 0);
        variable v_aq : unsigned(12 downto 0);
        variable v_nz : signed(26 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: folded cell x (|cu - cw|, or the ramp when tiling); mirrored
            --    read offset = min(cupx, cw-1-cupx)
            v_f := signed(resize(s_cu, 14)) - signed(resize(s_cwv, 14));
            if s_tile = '1' then
                s_fu1 <= s_cu;
            elsif v_f < 0 then
                s_fu1 <= resize(unsigned(-v_f), 13);
            else
                s_fu1 <= unsigned(v_f(12 downto 0));
            end if;
            if s_tile = '1' or s_cupx < s_cdpx then s_m1 <= s_cupx; else s_m1 <= s_cdpx; end if;
            s_cs1 <= s_cs;

            -- 2: a |x| as two partial products; read address
            s_plo2 <= s_fu1 * s_a(7 downto 0);
            s_phi2 <= s_fu1 * s_a(11 downto 8);
            s_rd2 <= s_cs1 + s_m1;

            -- 3: a |x|
            s_pa3 <= resize(s_plo2, 25) + shift_left(resize(s_phi2, 25), 8);
            s_rd3 <= s_rd2;

            -- 4: field = a|x| -+ b|y|; issue the video read
            if s_diam = '1' then
                s_s4 <= signed(resize(s_pa3, 27)) + signed(resize(s_fvb, 27));
            else
                s_s4 <= signed(resize(s_pa3, 27)) - signed(resize(s_fvb, 27));
            end if;
            s_lb_ra <= s_rd3;

            -- 5: serration + drift
            s_s5 <= s_s4 + signed(resize(s_spl, 27)) + signed(resize(s_ph, 27));

            -- 6: noise; video (fabric copy)
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            v_nz := (others => '0');
            if s_noise = '1' then v_nz := signed(resize(s_lfsr(6 downto 0) & "0000000", 27)); end if;
            s_s6 <= s_s5 + v_nz;
            s_y6 <= unsigned(s_lb_q(31 downto 22));
            s_ud(6) <= unsigned(s_lb_q(21 downto 12));
            s_vd(6) <= unsigned(s_lb_q(11 downto 2));

            -- 7: etch product
            if s_y6 > 64 then s_m7 <= (s_y6 - 64) * s_etch; else s_m7 <= (others => '0'); end if;
            s_s7 <= s_s6;

            -- 8: etched field
            s_s8 <= s_s7 + signed(resize(s_m7(19 downto 2), 27));

            -- 9: band index -> palette address
            v_q := s_s8(26 downto 14);
            if v_q < 0 then v_aq := unsigned(-v_q); else v_aq := unsigned(v_q); end if;
            s_pal_ra <= s_pali & v_aq(3 downto 0);

            -- 10: palette ROM output register; 11: fabric copy
            s_py11 <= unsigned(s_pal_q(31 downto 22));
            s_pu11 <= unsigned(s_pal_q(21 downto 12));
            s_pv11 <= unsigned(s_pal_q(11 downto 2));

            -- 12: compose: negative, chroma from the video
            if s_neg = '1' then s_y12 <= to_unsigned(1004, 10) - s_py11; else s_y12 <= s_py11; end if;
            if s_vchr = '1' then
                s_u12 <= s_ud(11); s_v12 <= s_vd(11);
            elsif s_neg = '1' then
                s_u12 <= resize(to_unsigned(1024, 11) - resize(s_pu11, 11), 10);
                s_v12 <= resize(to_unsigned(1024, 11) - resize(s_pv11, 11), 10);
            else
                s_u12 <= s_pu11; s_v12 <= s_pv11;
            end if;

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

            -- delays
            for i in 7 to 12 loop s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture starburst;
