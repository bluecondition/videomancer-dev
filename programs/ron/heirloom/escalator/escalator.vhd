-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Escalator v0.1.0 -- Heirloom #1 (2026-10-20), from WIN_20220629_23_49_20.
--
-- The photo: the picture replicated into a grid of ~7 tiles across, every
-- tile a pastel lavender wash of the source except for a pink band across
-- the lower part of each tile row where the content turns vivid (yellow /
-- orange / magenta), and acid-green staircase bands -- sample-and-hold
-- stepped diagonals -- marching down to the right across the whole grid.
--
-- The program: each line is written decimated into a line buffer and read
-- back wrapped, so the picture is tiled across the screen (P12 = how many
-- tiles, continuous 1 .. 8); tile rows are square cells of the same size.
-- Outside the band every tile is washed toward lavender (K2); inside the
-- band (the lower K1 of each tile row) the chroma is boosted (K3) with a
-- pink cast.  Green staircases: a stepped diagonal ramp (steps 1/8 tile
-- wide, K5 rise per step, K4 thickness, one band per tile row) that marches
-- with K6.
--
--   K1  Band    pink band height, 1/8 .. 7/8 of the tile row
--   K2  Wash    lavender wash of the rest, 0 .. 100 %
--   K3  Vivid   chroma gain inside the band, 0 .. 400 %
--   K4  Stairs  staircase thickness, 1/16 .. 1 of the row period
--   K5  Slope   rise per step, bipolar (centre = flat bands)
--   K6  Speed   staircase march, bipolar (centre = still)
--   S7  Brick   odd tile rows offset by half a tile
--   S8  Flip    odd tiles mirrored
--   S9  Twin    a second staircase family with the opposite slope
--   S10 Stairs  Solid green / Tinted (the picture's luma, green chroma)
--   S11 Comb    stair steps comb on alternate lines
--   P12 TILES   1 tile (the plain picture) .. 8 tiles across, continuous
--
-- Geometry runs in virtual 1080-line units (timing ID steps); tile widths
-- are real pixels (a count knob), tile rows are square in virtual units.
-- Generated avid (parity-locked start, field-latched width); output
-- conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777)
-- before the blanking gate.  Latency 16 clocks in every mode.  16 EBR.
-- Generated colours are authored in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture escalator of program_top is
    constant C_LAT : integer := 16;   -- matches processing_delay_clks

    -- generated colours (u = Cr, v = Cb)
    constant C_GRN_Y : unsigned(9 downto 0) := to_unsigned(580, 10);
    constant C_GRN_U : unsigned(9 downto 0) := to_unsigned(136, 10);
    constant C_GRN_V : unsigned(9 downto 0) := to_unsigned(216, 10);
    constant C_LAV_Y : signed(11 downto 0) := to_signed(747, 12);
    constant C_LAV_U : signed(11 downto 0) := to_signed(516, 12);
    constant C_LAV_V : signed(11 downto 0) := to_signed(594, 12);

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
    signal s_zc : unsigned(5 downto 0) := (others => '0');          -- zone clock (sequencer)
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_k1p : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k2, s_k3 : unsigned(7 downto 0) := (others => '0');
    signal s_k4p : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k5r, s_k6r : signed(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched
    signal s_f : unsigned(10 downto 0) := to_unsigned(1024, 11);    -- decimation, Q.10
    signal s_wt : unsigned(10 downto 0) := to_unsigned(1920, 11);   -- tile width, real px
    signal s_wth : unsigned(10 downto 0) := to_unsigned(960, 11);   -- wt / 2
    signal s_wtm1 : unsigned(10 downto 0) := to_unsigned(1919, 11); -- wt - 1
    signal s_th : unsigned(11 downto 0) := to_unsigned(1920, 12);   -- tile size, virtual
    signal s_thq : unsigned(19 downto 0) := (others => '0');        -- th << 8
    signal s_step : unsigned(9 downto 0) := to_unsigned(480, 10);   -- th / 8, Q.1
    signal s_steph : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal s_p : unsigned(15 downto 0) := to_unsigned(30720, 16);   -- stair period, Q.4 lines
    signal s_t : unsigned(15 downto 0) := to_unsigned(4096, 16);    -- stair thickness
    signal s_bandst : unsigned(19 downto 0) := (others => '0');     -- band start, Q.8
    signal s_wash, s_vivid : unsigned(7 downto 0) := (others => '0');
    signal s_rise : signed(9 downto 0) := (others => '0');
    signal s_speed : signed(9 downto 0) := (others => '0');
    signal s_brick, s_flip, s_twin, s_tint, s_comb : std_logic := '0';
    signal s_ph : unsigned(15 downto 0) := (others => '0');         -- march phase
    -- serial multiplier (zone)
    signal s_ma : unsigned(21 downto 0) := (others => '0');
    signal s_mb : unsigned(9 downto 0) := (others => '0');
    signal s_macc : unsigned(21 downto 0) := (others => '0');
    signal s_bandh : unsigned(11 downto 0) := (others => '0');
    signal s_tsum : unsigned(21 downto 0) := (others => '0');
    signal s_tb : unsigned(11 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';                              -- g_avid rises on this edge

    -- source side: decimated write
    signal s_wacc : unsigned(10 downto 0) := (others => '0');
    signal s_wcnt : unsigned(10 downto 0) := (others => '0');
    signal s_wtl  : unsigned(10 downto 0) := to_unsigned(1920, 11);
    -- per line (next / committed)
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_cy_n, s_cy : unsigned(19 downto 0) := (others => '0');
    signal s_rodd_n, s_rodd : std_logic := '0';
    signal s_dl_n, s_dl : unsigned(15 downto 0) := (others => '0');
    signal s_lpar : std_logic := '0';
    signal s_band : std_logic := '0';

    -- beam
    signal s_r, s_rd : unsigned(10 downto 0) := (others => '0');    -- tile position, and wt-1-r
    signal s_k : unsigned(3 downto 0) := (others => '0');           -- tile index
    signal s_sa : unsigned(10 downto 0) := (others => '0');         -- step accumulator, Q.1
    signal s_d, s_d2 : unsigned(15 downto 0) := (others => '0');    -- stair ramps
    signal s_fire1, s_fire2, s_fire3 : std_logic := '0';
    signal s_c1, s_c1b : signed(17 downto 0) := (others => '0');
    signal s_cp, s_cm, s_cpb, s_cmb : signed(17 downto 0) := (others => '0');
    signal s_neg, s_big, s_negb, s_bigb : std_logic := '0';

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_gr1 : std_logic := '0';
    signal s_grn : t_sl(2 to 7) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y4, s_u4, s_v4, s_y5, s_u5, s_v5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_dyl4, s_dul4, s_dvl4, s_du4, s_dv4 : signed(11 downto 0) := (others => '0');
    signal s_pyl5, s_pul5, s_pvl5, s_pu5, s_pv5 : signed(20 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
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
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, per-frame products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_f : unsigned(12 downto 0);
        variable v_t : unsigned(10 downto 0);
        variable v_th : unsigned(12 downto 0);
        variable v_ph : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            -- vblank zone: no avid for 1024 clocks
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
            elsif s_zc /= 63 then
                s_zc <= s_zc + 1;
            end if;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- knobs (free-running decode)
            v_k := unsigned(registers_in(0));                      -- K1 Band: 128 .. 895 / 1024 of the row
            s_k1p <= to_unsigned(128, 10) + resize(v_k(9 downto 1), 10) + resize(v_k(9 downto 2), 10);
            s_k2 <= unsigned(registers_in(1)(9 downto 2));          -- K2 Wash
            s_k3 <= unsigned(registers_in(2)(9 downto 2));          -- K3 Vivid, 0 .. 4x in /64
            v_k := unsigned(registers_in(3));                      -- K4 Stairs: 64 .. 1023
            v_t := to_unsigned(64, 11) + resize(v_k, 11) - resize(v_k(9 downto 4), 11);
            if v_t > 1023 then s_k4p <= (others => '1'); else s_k4p <= v_t(9 downto 0); end if;
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;   -- K5 Slope, deadband
            if v_s > -8 and v_s < 8 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;   -- K6 Speed, deadband
            v_s := shift_right(v_s, 2);
            if v_s > -2 and v_s < 2 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;
            -- P12 Tiles glide (1/8 of the gap a field)
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                if s_wtl /= 0 then s_wt <= s_wtl; end if;
                s_wash <= s_k2; s_vivid <= s_k3;
                s_rise <= s_k5r; s_speed <= s_k6r;
                s_brick <= registers_in(6)(0);
                s_flip  <= registers_in(6)(1);
                s_twin  <= registers_in(6)(2);
                s_tint  <= registers_in(6)(3);
                s_comb  <= registers_in(6)(4);
                -- march
                v_ph := signed(resize(s_ph, 18)) + resize(s_speed, 18);
                if v_ph < 0 then v_ph := v_ph + signed(resize(s_p, 18));
                elsif v_ph >= signed(resize(s_p, 18)) then v_ph := v_ph - signed(resize(s_p, 18)); end if;
                s_ph <= unsigned(v_ph(15 downto 0));
            end if;

            -- derived (settle in the zone's first clocks)
            v_f := to_unsigned(1024, 13) - (resize(s_g(11 downto 2), 13) - resize(s_g(11 downto 5), 13));   -- 1024 - 7/8 p12
            s_f <= v_f(10 downto 0);
            g_wm1 <= s_w - 1;
            s_wth <= '0' & s_wt(10 downto 1);
            s_wtm1 <= s_wt - 1;
            case to_integer(s_xs) is
                when 4      => v_th := resize(s_wt & '0', 13);
                when 3      => v_th := resize(s_wt, 13) + resize(s_wt(10 downto 1), 13);
                when others => v_th := resize(s_wt, 13);
            end case;
            s_th <= v_th(11 downto 0);
            s_thq <= s_th & x"00";
            s_step <= s_th(11 downto 2);
            s_steph <= '0' & s_th(11 downto 3);
            s_p <= s_th & "0000";
            s_bandst <= (s_th - s_bandh) & x"00";

            -- serial products: zone clocks 4..14 th x k1p (band height),
            -- 16..26 th x k4p (stair thickness)
            if s_zc = 4 then
                s_ma <= resize(s_th, 22); s_mb <= s_k1p; s_macc <= (others => '0');
            elsif s_zc = 16 then
                s_ma <= resize(s_th, 22); s_mb <= s_k4p; s_macc <= (others => '0');
            elsif (s_zc >= 5 and s_zc <= 14) or (s_zc >= 17 and s_zc <= 26) then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(20 downto 0) & '0';
                s_mb <= '0' & s_mb(9 downto 1);
            elsif s_zc = 15 then
                s_bandh <= s_macc(21 downto 10);
            elsif s_zc = 27 then
                s_t <= s_macc(21 downto 6);
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
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
    -- rise, field-latched width)
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
    -- Source side: decimated line-buffer write; per-line values computed at
    -- the source avid fall and committed at its rise (before the beam starts)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_a : unsigned(11 downto 0);
        variable v_c : unsigned(20 downto 0);
        variable v_d : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            s_lb_we <= '0';
            if data_in.avid = '1' then
                v_a := resize(s_wacc, 12) + resize(s_f, 12);
                if v_a >= 1024 then
                    s_wacc <= resize(v_a - 1024, 11);
                    s_lb_we <= '1';
                    s_lb_wa <= s_wcnt;
                    s_lb_wd <= data_in.y & data_in.u & data_in.v & "00";
                    s_wcnt <= s_wcnt + 1;
                else
                    s_wacc <= v_a(10 downto 0);
                end if;
            else
                s_wacc <= (others => '0');
                s_wcnt <= (others => '0');
            end if;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_cy_n <= (others => '0');
                s_rodd_n <= '0';
                s_dl_n <= s_ph;
            elsif data_in.avid = '0' and s_avq = '1' then
                -- line end: write count, row DDA, stair ramp
                s_wtl <= s_wcnt;
                s_vy_n <= s_vy_n + s_ys;
                v_c := resize(s_cy_n, 21) + s_ys;
                if v_c >= s_thq then
                    s_cy_n <= resize(v_c - s_thq, 20);
                    s_rodd_n <= not s_rodd_n;
                else
                    s_cy_n <= v_c(19 downto 0);
                end if;
                v_d := resize(s_dl_n, 17) + resize(s_ys(11 downto 4), 17);
                if v_d >= s_p then
                    s_dl_n <= resize(v_d - s_p, 16);
                else
                    s_dl_n <= v_d(15 downto 0);
                end if;
            elsif data_in.avid = '1' and s_avq = '0' then
                s_cy <= s_cy_n;
                s_rodd <= s_rodd_n;
                s_dl <= s_dl_n;
                s_lpar <= not s_lpar;
                if s_cy_n >= s_bandst then s_band <= '1'; else s_band <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: tiled read address, stair ramps
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_sa : unsigned(11 downto 0);
        variable v_d, v_d2 : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s_fire1 <= '0';
            if g_start = '1' then
                -- line start (same edge as the generated avid rise, so the
                -- stage-1 register one clock later sees pixel 0)
                if s_brick = '1' and s_rodd = '1' then
                    s_r <= s_wth;
                    s_rd <= s_wtm1 - s_wth;
                else
                    s_r <= (others => '0');
                    s_rd <= s_wtm1;
                end if;
                s_k <= (others => '0');
                if s_comb = '1' and s_lpar = '1' then s_sa <= '0' & s_steph; else s_sa <= (others => '0'); end if;
                s_d <= s_dl; s_d2 <= s_dl;
            elsif g_avid = '1' then
                if s_r = s_wtm1 then
                    s_r <= (others => '0');
                    s_rd <= s_wtm1;
                    s_k <= s_k + 1;
                else
                    s_r <= s_r + 1;
                    s_rd <= s_rd - 1;
                end if;
                v_sa := resize(s_sa, 12) + s_xs;
                if v_sa >= s_step then
                    s_sa <= resize(v_sa - s_step, 11);
                    s_fire1 <= '1';
                else
                    s_sa <= v_sa(10 downto 0);
                end if;
            end if;
            -- read address (stage 1)
            if s_flip = '1' and s_k(0) = '1' then s_lb_ra <= s_rd; else s_lb_ra <= s_r; end if;

            -- stair ramps: step = subtract (family 1) / add (family 2) the
            -- rise, wrapped into [0, P) over three clocks (steps are >= 11 px apart)
            s_c1 <= signed(resize(s_d, 18)) - resize(s_rise, 18);
            s_c1b <= signed(resize(s_d2, 18)) + resize(s_rise, 18);
            s_fire2 <= s_fire1;
            s_cp <= s_c1 + signed(resize(s_p, 18)); s_cm <= s_c1 - signed(resize(s_p, 18));
            s_cpb <= s_c1b + signed(resize(s_p, 18)); s_cmb <= s_c1b - signed(resize(s_p, 18));
            s_neg <= s_c1(17); s_negb <= s_c1b(17);
            if s_c1 >= signed(resize(s_p, 18)) then s_big <= '1'; else s_big <= '0'; end if;
            if s_c1b >= signed(resize(s_p, 18)) then s_bigb <= '1'; else s_bigb <= '0'; end if;
            s_fire3 <= s_fire2;
            if s_fire3 = '1' then
                if s_neg = '1' then v_d := unsigned(s_cp(15 downto 0));
                elsif s_big = '1' then v_d := unsigned(s_cm(15 downto 0));
                else v_d := unsigned(s_c1(15 downto 0)); end if;
                if s_negb = '1' then v_d2 := unsigned(s_cpb(15 downto 0));
                elsif s_bigb = '1' then v_d2 := unsigned(s_cmb(15 downto 0));
                else v_d2 := unsigned(s_c1b(15 downto 0)); end if;
                s_d <= v_d; s_d2 <= v_d2;
            end if;
            -- green where a ramp is under the thickness (stage 1 flag)
            if s_d < s_t or (s_twin = '1' and s_d2 < s_t) then s_gr1 <= '1'; else s_gr1 <= '0'; end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = read address / green flag)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y, v_u, v_v : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: buffer output register; 3: fabric copy
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 4: differences toward lavender (wash) / about neutral (vivid)
            s_y4 <= s_y3; s_u4 <= s_u3; s_v4 <= s_v3;
            s_dyl4 <= C_LAV_Y - signed(resize(s_y3, 12));
            s_dul4 <= C_LAV_U - signed(resize(s_u3, 12));
            s_dvl4 <= C_LAV_V - signed(resize(s_v3, 12));
            s_du4 <= signed(resize(s_u3, 12)) - 512;
            s_dv4 <= signed(resize(s_v3, 12)) - 512;

            -- 5: products
            s_y5 <= s_y4; s_u5 <= s_u4; s_v5 <= s_v4;
            s_pyl5 <= s_dyl4 * signed(resize(s_wash, 9));
            s_pul5 <= s_dul4 * signed(resize(s_wash, 9));
            s_pvl5 <= s_dvl4 * signed(resize(s_wash, 9));
            s_pu5 <= s_du4 * signed(resize(s_vivid, 9));
            s_pv5 <= s_dv4 * signed(resize(s_vivid, 9));

            -- 6: wash outside the band, vivid + pink cast inside
            if s_band = '1' then
                v_y := signed(resize(s_y5, 13));
                v_u := to_signed(576, 13) + resize(shift_right(s_pu5, 6), 13);
                v_v := to_signed(512, 13) + resize(shift_right(s_pv5, 6), 13);
            else
                v_y := signed(resize(s_y5, 13)) + resize(shift_right(s_pyl5, 8), 13);
                v_u := signed(resize(s_u5, 13)) + resize(shift_right(s_pul5, 8), 13);
                v_v := signed(resize(s_v5, 13)) + resize(shift_right(s_pvl5, 8), 13);
            end if;
            s_y6 <= f_clamp(v_y); s_u6 <= f_clamp(v_u); s_v6 <= f_clamp(v_v);

            -- 7: staircases
            if s_grn(6) = '1' then
                if s_tint = '1' then s_y7 <= s_y6; else s_y7 <= C_GRN_Y; end if;
                s_u7 <= C_GRN_U; s_v7 <= C_GRN_V;
            else
                s_y7 <= s_y6; s_u7 <= s_u6; s_v7 <= s_v6;
            end if;

            -- 8..13: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]; centre at stage 12 -> 13)
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
            -- 14: luma toward black (white 940 -> 777); blanking gate
            if s_yf13 > 64 then v_dl := s_yf13 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(12) = '1' then
                s_y14 <= s_yf13 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
            -- 15..16: pad to the 16-clock latency
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;

            -- delays
            s_grn(2) <= s_gr1;
            for i in 3 to 7 loop s_grn(i) <= s_grn(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture escalator;
