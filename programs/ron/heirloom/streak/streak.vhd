-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Streak v0.1.0 -- Heirloom #7 (2026-10-20), from WIN_20230129_00_40_34.
--
-- The photo: the picture smeared into horizontal rainbow streaks, the hue
-- shifting band by band down the screen, a diagonal boundary below which
-- the colour freezes into magenta / cyan barcode blocks.
--
-- The program: a horizontal IIR smear along every line (P12), the chroma
-- rotated round the hue wheel by a different angle in every horizontal band
-- (K1 band height, K2 step, K5 speed), and on one side of a diagonal (K3)
-- the chroma sampled-and-held in column blocks (K4) -- a barcode.
--
--   K1  Bands    band height
--   K2  Hue      hue step from band to band
--   K3  Slope    the diagonal boundary, bipolar
--   K4  Blocks   barcode block width
--   K5  Spin     hue rotation speed, bipolar
--   K6  Contrast luma contrast 0 .. 200 %
--   S7  Smear    Colour (luma + chroma) / Luma only
--   S8  Barcode  Below / Above the diagonal
--   S9  Luma     Keep / Ramp (a horizontal ramp per band)
--   S10 Bands    Still / Scroll
--   S11 Source   Video / Osc
--   P12 SMEAR    0 = clean .. everything streaks to the right edge
--
-- Generated avid; output conditioning before the blanking gate.  Latency 20
-- clocks.  16 EBR.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture streak of program_top is
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
    -- clamp a 13-bit signed value into the 64 .. 960 video range
    function f_clamp(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 960 then return to_unsigned(960, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;
    -- saturate a 9-bit unsigned to 8 bits
    function f_sat8(v : unsigned(8 downto 0)) return unsigned is
    begin
        if v(8) = '1' then return x"FF"; else return v(7 downto 0); end if;
    end function;

    -- line buffer {y, u, v, 00} (read/write can collide: read-before-write kept)
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- internal oscillator source (Source = Osc)
    signal s_src : std_logic := '0';
    signal s_ox : unsigned(11 downto 0) := (others => '0');
    signal s_oy : unsigned(11 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- timing / controls
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- serial multiplier (12 steps)
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');
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
    signal s_fall1, s_fall2 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_bcol : unsigned(10 downto 0) := (others => '0');
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

    -- horizontal IIR smear state (the smeared pixel)
    signal s_z : unsigned(2 downto 0) := (others => '0');
    signal s_lumonly : std_logic := '0';
    signal s_sy : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_su, s_sv : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_cos is array (0 to 31) of integer range -128 to 127;
    constant C_COS : t_cos := (127, 125, 117, 106, 90, 71, 49, 25, 0, -25, -49, -71, -90, -106, -117, -125, -127, -125, -117, -106, -90, -71, -49, -25, 0, 25, 49, 71, 90, 106, 117, 125);
    signal s_bh : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal s_bhq : unsigned(17 downto 0) := (others => '0');
    signal s_hs : unsigned(7 downto 0) := (others => '0');
    signal s_ds : signed(7 downto 0) := (others => '0');
    signal s_dsy : signed(15 downto 0) := (others => '0');
    signal s_bw : unsigned(7 downto 0) := to_unsigned(32, 8);
    signal s_hv : signed(5 downto 0) := (others => '0');
    signal s_cg : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_hph : unsigned(7 downto 0) := (others => '0');
    signal s_bsc : unsigned(7 downto 0) := (others => '0');
    signal s_xb0 : signed(17 downto 0) := (others => '0');
    signal s_above, s_ramp, s_scroll : std_logic := '0';
    -- per line
    signal s_cy : unsigned(17 downto 0) := (others => '0');
    signal s_band : unsigned(7 downto 0) := (others => '0');
    signal s_xb : signed(17 downto 0) := (others => '0');
    signal s_ang_n, s_ang : unsigned(7 downto 0) := (others => '0');
    signal s_xbi : signed(13 downto 0) := (others => '0');
    signal s_bo : unsigned(9 downto 0) := (others => '0');
    signal s_cs, s_sn : signed(7 downto 0) := (others => '0');
    -- beam
    signal s_vx : unsigned(12 downto 0) := (others => '0');
    signal s_bk : unsigned(7 downto 0) := (others => '0');
    signal s_bs : std_logic := '0';
    -- pixel stages
    signal s_r : t_sl(1 to 4) := (others => '0');
    signal s_bsd : t_sl(1 to 4) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_hu, s_hv5, s_u5, s_v5, s_y5 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_du6, s_dv6, s_dy6 : signed(10 downto 0) := (others => '0');
    signal s_p1, s_p2, s_p3, s_p4 : signed(18 downto 0) := (others => '0');
    signal s_py7 : signed(19 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_vxd : t_u10(1 to 8) := (others => (others => '0'));
    signal s_yd, s_ud, s_vd : t_u10(9 to 11) := (others => to_unsigned(64, 10));

begin

    ---------------------------------------------------------------------------
    -- Line buffer
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;


    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_p : unsigned(15 downto 0);

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
            elsif s_zc /= 127 then
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
            s_bh <= to_unsigned(16, 10) + resize(unsigned(registers_in(0)(9 downto 1)), 10);   -- K1 Bands 16 .. 527
            s_hs <= unsigned(registers_in(1)(9 downto 2));                                   -- K2 Hue step
            v_s := signed(resize(unsigned(registers_in(2)), 11)) - 512;                      -- K3 Slope
            v_s := shift_right(v_s, 2);
            if v_s > -3 and v_s < 3 then s_ds <= (others => '0'); else s_ds <= v_s(7 downto 0); end if;
            s_bw <= resize(unsigned(registers_in(3)(9 downto 3)), 8) + 4;                    -- K4 Blocks 4 .. 131
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;                      -- K5 Spin
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_hv <= (others => '0'); else s_hv <= v_s(5 downto 0); end if;
            s_cg <= unsigned(registers_in(5)(9 downto 2));                                   -- K6 Contrast /128

            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            -- per field
            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_lumonly <= registers_in(6)(0);
                s_above   <= registers_in(6)(1);
                s_ramp    <= registers_in(6)(2);
                s_scroll  <= registers_in(6)(3);
                s_src     <= registers_in(6)(4);
                s_z <= s_g(11 downto 9);
                s_hph <= s_hph + unsigned(resize(s_hv, 8));
                if s_scroll = '1' then s_bsc <= s_bsc + 1; end if;

            end if;

            -- derived (free-running from the latched values)
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_bhq <= s_bh & x"00";
            -- boundary at line 0: vw/2 - 540 ds (Q.4 virtual px)
            s_xb0 <= signed(resize(shift_left(resize(s_vw, 18), 3), 18))
                     - (shift_left(resize(s_ds, 18), 9) + shift_left(resize(s_ds, 18), 4)
                        + shift_left(resize(s_ds, 18), 3) + shift_left(resize(s_ds, 18), 2));

            -- serial unit (12 shift-add steps per job)
            if s_zc = 4 then
                if s_ds < 0 then s_ma <= resize(unsigned(-s_ds), 24); else s_ma <= resize(unsigned(s_ds), 24); end if;
                s_mb <= s_ys; s_mjob <= "000";
                s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when others =>
                        if s_ds < 0 then s_dsy <= -signed(resize(s_macc(23 downto 8), 16));
                        else s_dsy <= signed(resize(s_macc(23 downto 8), 16)); end if;
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
    -- Source side: oscillator, line-buffer write (Video / Osc), per-line
    -- values (computed after the source avid fall, committed at its rise)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(18 downto 0);
        variable v_a : unsigned(15 downto 0);
        variable v_i : unsigned(4 downto 0);
        variable v_b : unsigned(7 downto 0);

    begin
        if rising_edge(clk) then
            -- oscillator: x ramp per source pixel, y ramp per line, phase per field
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

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_cy <= (others => '0');
                s_band <= (others => '0');
                s_xb <= s_xb0;

            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                v_c := resize(s_cy, 19) + s_ys;
                if v_c >= s_bhq then
                    s_cy <= resize(v_c - s_bhq, 18);
                    s_band <= s_band + 1;
                else
                    s_cy <= v_c(17 downto 0);
                end if;
                s_xb <= s_xb + resize(s_dsy, 18);

            elsif s_fall2 = '1' then
                null;

            end if;
            v_a := (s_band + s_bsc) * s_hs;
            s_ang_n <= v_a(7 downto 0) + s_hph;

            if data_in.avid = '1' and s_avq = '0' then
                s_ang <= s_ang_n;
                s_xbi <= s_xb(17 downto 4);
                v_b := s_band + s_bsc;
                s_bo <= v_b(3 downto 0) & "000000";
                v_i := s_ang_n(7 downto 3);
                s_cs <= to_signed(C_COS(to_integer(v_i)), 8);
                s_sn <= to_signed(C_COS(to_integer(v_i + 24)), 8);

            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state (initialised on the g_start edge so stage 1 sees pixel 0)
    ---------------------------------------------------------------------------
    p_beam : process(clk)

    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_bcol <= (others => '0');
                s_vx <= (others => '0');
                s_bk <= (others => '0');
                s_bs <= '1';

            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
                s_vx <= s_vx + s_xs;
                if s_bk = s_bw - 1 then s_bk <= (others => '0'); s_bs <= '1';
                else s_bk <= s_bk + 1; s_bs <= '0'; end if;

            end if;

        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = read address; compose at stage 12)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_d, v_e, v_n : signed(11 downto 0);
        variable v_zc : integer range 0 to 7;
        variable v_h : unsigned(9 downto 0);
        variable v_hu, v_hv : unsigned(9 downto 0);

    begin
        if rising_edge(clk) then
            -- 1: read address; region (left of the boundary, flipped by S8); block start
            s_lb_ra <= s_bcol;
            if signed(resize(s_vx(12 downto 1), 14)) < s_xbi then s_r(1) <= not s_above; else s_r(1) <= s_above; end if;
            s_bsd(1) <= s_bs;
            s_vxd(1) <= s_vx(10 downto 1);
            -- 3: video (fabric copy)
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));
            -- 4: IIR smear
            v_d := signed(resize(s_y3, 12)) - signed(resize(s_sy, 12));
            case to_integer(s_z) is
                when 0 => v_d := v_d;
                when 1 => v_d := shift_right(v_d, 1);
                when 2 => v_d := shift_right(v_d, 2);
                when 3 => v_d := shift_right(v_d, 3);
                when 4 => v_d := shift_right(v_d, 4);
                when 5 => v_d := shift_right(v_d, 5);
                when 6 => v_d := shift_right(v_d, 6);
                when others => v_d := shift_right(v_d, 7);
            end case;
            v_n := signed(resize(s_sy, 12)) + v_d;
            s_sy <= unsigned(v_n(9 downto 0));
            v_d := signed(resize(s_u3, 12)) - signed(resize(s_su, 12));
            v_e := signed(resize(s_v3, 12)) - signed(resize(s_sv, 12));
            if s_lumonly = '1' then
                v_zc := 0;
            else
                v_zc := to_integer(s_z);
            end if;
            case v_zc is
                when 0 => v_d := v_d; v_e := v_e;
                when 1 => v_d := shift_right(v_d, 1); v_e := shift_right(v_e, 1);
                when 2 => v_d := shift_right(v_d, 2); v_e := shift_right(v_e, 2);
                when 3 => v_d := shift_right(v_d, 3); v_e := shift_right(v_e, 3);
                when 4 => v_d := shift_right(v_d, 4); v_e := shift_right(v_e, 4);
                when 5 => v_d := shift_right(v_d, 5); v_e := shift_right(v_e, 5);
                when 6 => v_d := shift_right(v_d, 6); v_e := shift_right(v_e, 6);
                when others => v_d := shift_right(v_d, 7); v_e := shift_right(v_e, 7);
            end case;
            v_n := signed(resize(s_su, 12)) + v_d;
            s_su <= unsigned(v_n(9 downto 0));
            v_n := signed(resize(s_sv, 12)) + v_e;
            s_sv <= unsigned(v_n(9 downto 0));
            -- 5: barcode: chroma held per block on the barcode side
            if s_bsd(4) = '1' then v_hu := s_su; v_hv := s_sv; else v_hu := s_hu; v_hv := s_hv5; end if;
            s_hu <= v_hu; s_hv5 <= v_hv;
            if s_r(4) = '1' then s_u5 <= v_hu; s_v5 <= v_hv; else s_u5 <= s_su; s_v5 <= s_sv; end if;
            s_y5 <= s_sy;
            -- 6: signed chroma / luma
            s_du6 <= signed(resize(s_u5, 11)) - 512;
            s_dv6 <= signed(resize(s_v5, 11)) - 512;
            s_dy6 <= signed(resize(s_y5, 11)) - 512;
            -- 7: rotation and contrast products
            s_p1 <= s_du6 * s_cs;
            s_p2 <= s_dv6 * s_sn;
            s_p3 <= s_du6 * s_sn;
            s_p4 <= s_dv6 * s_cs;
            s_py7 <= s_dy6 * signed(resize(s_cg, 9));
            -- 8: rotated chroma, contrasted luma
            s_u8 <= f_clamp(to_signed(512, 13) + resize(shift_right(s_p1 - s_p2, 7), 13));
            s_v8 <= f_clamp(to_signed(512, 13) + resize(shift_right(s_p3 + s_p4, 7), 13));
            s_y8 <= f_clamp(to_signed(512, 13) + resize(shift_right(s_py7, 7), 13));
            -- 9: luma ramp option
            if s_ramp = '1' then
                v_h := s_vxd(8) + s_bo;
                s_yd(9) <= to_unsigned(64, 10) + v_h - resize(v_h(9 downto 3), 10);
            else
                s_yd(9) <= s_y8;
            end if;
            s_ud(9) <= s_u8; s_vd(9) <= s_v8;
            -- 10..12
            for i in 10 to 11 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            s_y12 <= s_yd(11); s_u12 <= s_ud(11); s_v12 <= s_vd(11);

            -- 13..18: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]; centre at stage 17 -> 18)
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
            -- 19: luma toward black (white 940 -> 777); blanking gate
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(17) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            -- 20
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
            for i in 2 to 4 loop s_r(i) <= s_r(i - 1); s_bsd(i) <= s_bsd(i - 1); end loop;
            for i in 2 to 8 loop s_vxd(i) <= s_vxd(i - 1); end loop;

        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture streak;
