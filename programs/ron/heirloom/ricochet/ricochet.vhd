-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ricochet v0.1.0 -- Heirloom #30 (2026-10-20), from WIN_20230515_23_08_59.
--
-- The photo: an orange ground with nested cyan / lavender chevrons pointing
-- right, mirrored top and bottom about the centre line, and flat orange
-- beyond the last chevron.
--
-- The program: field = |dy| * pitch + x * angle (chevron rings pointing
-- right, mirrored by |dy|) plus the source luma scaled by K6; the ring
-- index paints cyan / lavender / orange / lavender in turn, and where the
-- field passes the Reach (P12) the ground is flat orange.
--
--   K1  Pitch    ring pitch
--   K2  Angle    chevron angle: flat bands .. steep V's
--   K3  Hue      hue rotation of the whole picture
--   K4  Centre   mirror line position
--   K5  Drift    rings flow, bipolar (centre = still)
--   K6  Etch     how hard the source luma pushes the rings
--   S7  Point    Right / Left
--   S8  Mirror   On (chevrons) / Off (diagonal bands)
--   S9  Palette  Cyan-lavender / Pink-lime
--   S10 Rings    Hard / Soft (ring shading)
--   S11 Source   Video / Osc
--   P12 REACH    how far the chevrons extend before the flat ground
--
-- Generated avid; output conditioning before the blanking gate; virtual
-- 1080-line geometry.  Latency 20 clocks.  EBR: 6 (source luma).
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture ricochet of program_top is
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
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    type t_cs is array (0 to 15) of integer range -128 to 127;
    constant C_COS : t_cs := (127, 117, 90, 49, 0, -49, -90, -117, -127, -117, -90, -49, 0, 49, 90, 117);
    constant C_SINR : t_cs := (0, 49, 90, 117, 127, 117, 90, 49, 0, -49, -90, -117, -127, -117, -90, -49);
    type t_col is array (0 to 2) of integer;
    constant CYN : t_col := (678, 64, 663);
    constant LAV : t_col := (747, 516, 594);
    constant ORG : t_col := (584, 772, 212);
    constant PNK : t_col := (752, 612, 512);
    constant LIM : t_col := (760, 420, 440);
    constant IND : t_col := (192, 608, 696);
    type t_pdef is array (0 to 7) of t_col;
    constant C_PDEF : t_pdef := (CYN, LAV, ORG, LAV, PNK, LIM, IND, LIM);
    type t_pal is array (0 to 7) of std_logic_vector(31 downto 0);
    function f_pal return t_pal is
        variable r : t_pal;
    begin
        for i in 0 to 7 loop
            r(i) := std_logic_vector(to_unsigned(C_PDEF(i)(0), 10)) & std_logic_vector(to_unsigned(C_PDEF(i)(1), 10))
                    & std_logic_vector(to_unsigned(C_PDEF(i)(2), 10)) & "00";
        end loop;
        return r;
    end function;
    constant C_PAL : t_pal := f_pal;

    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

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
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_k1, s_k2, s_k3, s_k4, s_k6 : unsigned(7 downto 0) := (others => '0');
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_pit : unsigned(7 downto 0) := to_unsigned(64, 8);          -- field units per virtual line
    signal s_pys : unsigned(11 downto 0) := to_unsigned(64, 12);        -- per field line
    signal s_ang : unsigned(7 downto 0) := to_unsigned(32, 8);          -- field units per virtual x unit
    signal s_axs : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_cos, s_sin : signed(7 downto 0) := (others => '0');
    signal s_cy : unsigned(10 downto 0) := to_unsigned(540, 11);        -- mirror line
    signal s_etch : unsigned(7 downto 0) := (others => '0');
    signal s_reach : unsigned(19 downto 0) := (others => '0');
    signal s_drift : unsigned(19 downto 0) := (others => '0');
    signal s_left, s_mir, s_pal2, s_soft : std_logic := '0';
    -- serial multiplier
    signal s_ma : unsigned(19 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(19 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(1 downto 0) := (others => '0');
    signal s_cyp : unsigned(19 downto 0) := (others => '0');          -- cy x pitch
    signal s_vwa : unsigned(19 downto 0) := (others => '0');          -- vw x angle

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
    signal s_fy_n, s_fy : unsigned(19 downto 0) := (others => '0');    -- |dy| * pitch (unsigned field)
    signal s_adyq : unsigned(19 downto 0) := (others => '0');          -- running |dy|*pitch, Q.8

    -- beam
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_fx : unsigned(19 downto 0) := (others => '0');            -- x * angle

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_f1 : unsigned(19 downto 0) := (others => '0');
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_f3 : unsigned(19 downto 0) := (others => '0');
    signal s_e4 : unsigned(17 downto 0) := (others => '0');
    signal s_f4 : unsigned(19 downto 0) := (others => '0');
    signal s_f5 : unsigned(20 downto 0) := (others => '0');
    signal s_c6 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_sh6 : unsigned(5 downto 0) := (others => '0');
    signal s_fl6 : std_logic := '0';
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_du8, s_dv8 : signed(10 downto 0) := (others => '0');
    signal s_y8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_puc9, s_pus9, s_pvc9, s_pvs9 : signed(18 downto 0) := (others => '0');
    signal s_y9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y11, s_y12, s_y13, s_y14, s_y15, s_y16x, s_yf16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17, s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17, s_u18, s_v18, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

    function f_clamp13(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 960 then return to_unsigned(960, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

begin

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial product
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_g : signed(12 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_c : signed(11 downto 0);
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

            s_k1 <= unsigned(registers_in(0)(9 downto 2));
            s_k2 <= unsigned(registers_in(1)(9 downto 2));
            s_k3 <= unsigned(registers_in(2)(9 downto 2));
            s_k4 <= unsigned(registers_in(3)(9 downto 2));
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;
            v_s := shift_right(v_s, 2);
            if v_s > -2 and v_s < 2 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            s_k6 <= unsigned(registers_in(5)(9 downto 2));
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_pit <= resize(s_k1(7 downto 1), 8) + 24;          -- 24 .. 151 field units / virtual line
                s_ang <= resize(s_k2(7 downto 2), 8) + 4;           -- 4 .. 67 field units / virtual unit
                s_cos <= to_signed(C_COS(to_integer(s_k3(7 downto 4))), 8);
                s_sin <= to_signed(C_SINR(to_integer(s_k3(7 downto 4))), 8);
                v_c := to_signed(540, 12) + signed(resize(s_k4, 12)) - 128;     -- 412 .. 667
                s_cy <= unsigned(v_c(10 downto 0));
                s_etch <= s_k6;
                -- reach: 64k .. 1.1M field units (rings of ~4k each)
                s_reach <= resize(s_g(11 downto 2) & "00000000", 20) + to_unsigned(4096, 20);
                s_left <= registers_in(6)(0);
                s_mir  <= registers_in(6)(1);
                s_pal2 <= registers_in(6)(2);
                s_soft <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_drift <= s_drift + unsigned(resize(shift_left(s_k5r, 5), 20));
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                               s_axs <= shift_left(resize(s_ang, 10), 2);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                               s_axs <= shift_left(resize(s_ang, 10), 1) + resize(s_ang, 10);
                when others => v_vw := resize(s_w, 14);
                               s_axs <= shift_left(resize(s_ang, 10), 1);
            end case;
            s_vw <= v_vw(11 downto 0);

            -- serial: job 0 pitch x ys (Q.8 per field line), job 1 cy x pitch,
            -- job 2 vw x angle
            if s_zc = 4 then
                s_ma <= resize(s_pit, 20); s_mb <= s_ys; s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0'); s_mjob <= "00";
            elsif s_zc = 20 then
                s_ma <= resize(s_pit, 20); s_mb <= resize(s_cy, 12); s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0'); s_mjob <= "01";
            elsif s_zc = 36 then
                s_ma <= resize(s_ang, 20); s_mb <= s_vw; s_macc <= (others => '0'); s_mrun <= '1'; s_mi <= (others => '0'); s_mjob <= "10";
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(18 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "00"   => s_pys <= s_macc(19 downto 8);
                    when "01"   => s_cyp <= s_macc;
                    when others => s_vwa <= s_macc;
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

    ---------------------------------------------------------------------------
    -- Source side: line-buffer write; per-line |dy| * pitch
    ---------------------------------------------------------------------------
    p_src : process(clk)
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

            -- (y - cy) * pitch as a running value: starts at -cy * pitch (Mirror)
            -- or 0 (diagonal bands) and rises by pitch every field line; the
            -- field's y term is its magnitude, a V about the mirror line
            if s_zone = '1' then
                s_vy_n <= (others => '0');
                if s_mir = '1' then s_adyq <= unsigned(-signed(s_cyp)); else s_adyq <= (others => '0'); end if;
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_adyq <= s_adyq + resize(s_pys, 20);
            end if;
            if s_adyq(19) = '1' then s_fy_n <= unsigned(-signed(s_adyq)); else s_fy_n <= s_adyq; end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_fy <= s_fy_n + s_drift;
            end if;
        end if;
    end process p_src;

    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= (others => '0');
                if s_left = '1' then s_fx <= s_vwa; else s_fx <= (others => '0'); end if;
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                if s_left = '1' then s_fx <= s_fx - resize(s_axs, 20); else s_fx <= s_fx + resize(s_axs, 20); end if;
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
        variable v_f : unsigned(20 downto 0);
        variable v_yl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: field without the source: |dy| pitch + x angle (Left: x counts
            --    down from the right edge)
            s_f1 <= s_fy + s_fx;
            s_lb_ra <= s_rdp;
            -- 3: source luma (fabric); field copy
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_f3 <= s_f1;
            -- 4: etch product
            s_e4 <= s_y3 * s_etch;
            s_f4 <= s_f3;
            -- 5: etched field
            s_f5 <= resize(s_f4, 21) + resize(s_e4(17 downto 2), 21);
            -- 6: ring index -> palette (LUT), shading, flat beyond the reach
            s_c6 <= C_PAL(to_integer(s_pal2 & s_f5(13 downto 12)));
            s_sh6 <= s_f5(11 downto 6);
            if s_f5 >= resize(s_reach, 21) then s_fl6 <= '1'; else s_fl6 <= '0'; end if;
            -- 7: compose: flat orange beyond the reach; soft rings shade the luma
            if s_fl6 = '1' then
                s_y7 <= to_unsigned(584, 10); s_u7 <= to_unsigned(772, 10); s_v7 <= to_unsigned(212, 10);
            else
                v_yl := unsigned(s_c6(31 downto 22));
                if s_soft = '1' then v_yl := v_yl - resize(s_sh6 & "00", 10); end if;
                s_y7 <= v_yl; s_u7 <= unsigned(s_c6(21 downto 12)); s_v7 <= unsigned(s_c6(11 downto 2));
            end if;
            -- 8: hue rotation: differences
            s_du8 <= signed(resize(s_u7, 11)) - 512;
            s_dv8 <= signed(resize(s_v7, 11)) - 512;
            s_y8 <= s_y7;
            -- 9: products
            s_puc9 <= s_du8 * s_cos; s_pus9 <= s_du8 * s_sin;
            s_pvc9 <= s_dv8 * s_cos; s_pvs9 <= s_dv8 * s_sin;
            s_y9 <= s_y8;
            -- 10: sums + clamp
            s_u10 <= f_clamp13(resize(shift_right(s_puc9 - s_pvs9, 7), 13) + 512);
            s_v10 <= f_clamp13(resize(shift_right(s_pus9 + s_pvc9, 7), 13) + 512);
            s_y10 <= s_y9;

            -- 11..16: output conditioning (centre at stage 15 -> 16)
            s_csu(0) <= s_u10; s_csv(0) <= s_v10;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u10 - s_csu(7);
            s_boxv <= s_boxv + s_v10 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u16 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v16 <= v_sc(13 downto 4);
            s_y11 <= s_y10; s_y12 <= s_y11; s_y13 <= s_y12; s_y14 <= s_y13;
            s_y15 <= s_y14; s_y16x <= s_y15;
            v_sy := resize(s_y14, 12) + shift_left(resize(s_y15, 12), 1) + resize(s_y16x, 12);
            s_yf16 <= v_sy(11 downto 2);
            -- 17: luma toward black; blanking gate
            if s_yf16 > 64 then v_dl := s_yf16 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(15) = '1' then
                s_y17 <= s_yf16 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
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

end architecture ricochet;
