-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lozenge v0.1.0 -- Heirloom #24 (2026-10-20), from WIN_20230410_01_10_37.
--
-- The photo: nested concentric diamonds in black / hot pink / olive with
-- horizontal pink-olive stripes cutting across them (the stripe parity swaps
-- the two colours), and thin white / yellow highlight lines along one side
-- of every diamond edge.
--
-- The program: the field s = a|x| + b|y| about the centre (or about two
-- centres side by side) is cut into rings; each ring takes a colour from a
-- 3-colour cycle, horizontal stripes swap the two chromatic colours, and a
-- thin highlight is drawn where a ring begins, on one side only.  The video
-- (or the oscillator) etches the field: its luma pushes the rings.
--
--   K1  Aspect    diamond aspect: wide .. tall
--   K2  Stripes   stripe pitch
--   K3  Palette   Pink/Olive, Cyan/Navy, Red/Gold
--   K4  Highlight highlight line width
--   K5  Drift     rings flow outward / inward, bipolar
--   K6  Etch      video depth (how hard the picture pushes the rings)
--   S7  Stripes   on / off
--   S8  Side      highlight on the left / right side
--   S9  Swap      the two chromatic colours exchanged
--   S10 Twin      one diamond / two side by side
--   S11 Source    Video / Osc
--   P12 RINGS     ring count: a single lozenge .. dozens of fine rings
--
-- Virtual 1080-line geometry (cells are counts); the per-pixel a|x| is a
-- split 13x8 + 13x4 multiply, b|y| per line on the serial unit.  Generated
-- avid; output conditioning before the gate.  Latency 20 clocks.  16 EBR.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture lozenge of program_top is
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

    -- 3 palettes x {dark, colour A, colour B, highlight} {y, u = Cr, v = Cb}
    type t_col is array (0 to 2) of integer;
    type t_pdef is array (0 to 11) of t_col;
    constant C_PDEF : t_pdef := (
        (64, 512, 512),  (480, 900, 700), (330, 560, 300), (900, 540, 420),    -- black, hot pink, olive, warm white
        (120, 470, 700), (678, 64, 663),  (940, 512, 512), (840, 584, 64),     -- navy, cyan, white, yellow
        (64, 512, 512),  (328, 960, 360), (700, 650, 180), (940, 512, 512));   -- black, red, gold, white
    type t_pal is array (0 to 15) of std_logic_vector(29 downto 0);
    function f_mkpal return t_pal is
        variable r : t_pal;
        variable k : integer;
    begin
        for i in 0 to 15 loop
            if i < 12 then k := i; else k := i - 4; end if;
            r(i) := std_logic_vector(to_unsigned(C_PDEF(k)(0), 10)) & std_logic_vector(to_unsigned(C_PDEF(k)(1), 10))
                    & std_logic_vector(to_unsigned(C_PDEF(k)(2), 10));
        end loop;
        return r;
    end function;
    constant C_PAL : t_pal := f_mkpal;

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

    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_ta : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_tb : unsigned(7 downto 0) := to_unsigned(127, 8);
    signal s_spk : unsigned(7 downto 0) := to_unsigned(32, 8);
    signal s_j3 : unsigned(1 downto 0) := (others => '0');
    signal s_live3 : std_logic := '0';
    signal s_hw : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_etk : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- latched / derived
    signal s_pals : unsigned(1 downto 0) := (others => '0');
    signal s_stripes, s_side, s_swap, s_twin : std_logic := '0';
    signal s_zk : unsigned(7 downto 0) := to_unsigned(32, 8);
    signal s_cwpx, s_cwv : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_cwpxm1 : unsigned(10 downto 0) := to_unsigned(959, 11);
    signal s_cwv2 : unsigned(12 downto 0) := to_unsigned(1920, 13);
    signal s_a, s_b : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_ph : unsigned(23 downto 0) := (others => '0');
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

    -- source / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1, s_fall2 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_fv_n : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_fvb_n, s_fvb : unsigned(23 downto 0) := (others => '0');
    signal s_sph : unsigned(11 downto 0) := (others => '0');
    signal s_spar : std_logic := '0';

    -- beam
    signal s_cu : unsigned(12 downto 0) := (others => '0');
    signal s_cupx, s_cdpx, s_cs : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_fu1 : unsigned(12 downto 0) := (others => '0');
    signal s_lft1 : std_logic := '0';
    signal s_m1, s_cs1 : unsigned(10 downto 0) := (others => '0');
    signal s_plo2 : unsigned(20 downto 0) := (others => '0');
    signal s_phi2 : unsigned(16 downto 0) := (others => '0');
    signal s_rd2, s_rd3 : unsigned(10 downto 0) := (others => '0');
    signal s_lft : t_sl(2 to 8) := (others => '0');
    signal s_pa3 : unsigned(24 downto 0) := (others => '0');
    signal s_s4, s_s5, s_s7, s_s8 : unsigned(25 downto 0) := (others => '0');
    signal s_y6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_s6 : unsigned(25 downto 0) := (others => '0');
    signal s_m7 : unsigned(17 downto 0) := (others => '0');
    signal s_idx9 : unsigned(1 downto 0) := (others => '0');
    signal s_hl9 : std_logic := '0';
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y11, s_y12, s_y13, s_y14, s_y15, s_y16x, s_yf16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yp : t_u10(17 to 20) := (others => to_unsigned(64, 10));
    signal s_up, s_vp : t_u10(17 to 20) := (others => to_unsigned(512, 10));

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
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
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

            s_ta <= unsigned(registers_in(0)(9 downto 2));                      -- K1 Aspect
            s_spk <= resize(unsigned(registers_in(1)(9 downto 3)), 8) + 2;      -- K2 Stripes 2 .. 129
            v_k := unsigned(registers_in(2));                                   -- K3 Palette (3 labels)
            if v_k >= 3 then s_live3 <= '1'; end if;
            if s_live3 = '0' and v_k < 3 then
                s_j3 <= v_k(1 downto 0);
            elsif v_k < 341 then s_j3 <= "00";
            elsif v_k < 683 then s_j3 <= "01";
            else s_j3 <= "10"; end if;
            s_hw <= resize(unsigned(registers_in(3)(9 downto 3)), 8) + 2;       -- K4 Highlight 2 .. 129
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;         -- K5 Drift
            if v_s > -8 and v_s < 8 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            s_etk <= unsigned(registers_in(5)(9 downto 2));                     -- K6 Etch
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_pals <= s_j3;
                s_stripes <= not registers_in(6)(0);
                s_side    <= registers_in(6)(1);
                s_swap    <= registers_in(6)(2);
                s_twin    <= registers_in(6)(3);
                s_src     <= registers_in(6)(4);
                v_ph := signed(resize(s_ph, 25)) + shift_left(resize(s_k5r, 25), 6);
                s_ph <= unsigned(v_ph(23 downto 0));
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_tb <= not s_ta;
            s_zk <= resize(s_g(11 downto 5), 8) + 4;                           -- 4 .. 131
            if s_twin = '1' then
                s_cwpx <= '0' & s_w(11 downto 1); s_cwv <= '0' & s_vw(11 downto 1);
            else
                s_cwpx <= s_w; s_cwv <= s_vw;
            end if;
            s_cwpxm1 <= s_cwpx(10 downto 0) - 1;
            s_cwv2 <= s_cwv & '0';

            -- serial unit: 0 ta x zk, 1 tb x zk, 2 fv x b (per line)
            if s_zc = 4 then
                s_ma <= resize(s_ta, 24); s_mb <= resize(s_zk, 12); s_macc <= (others => '0');
                s_mjob <= "00"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 24 then
                s_ma <= resize(s_tb, 24); s_mb <= resize(s_zk, 12); s_macc <= (others => '0');
                s_mjob <= "01"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 44 or s_fall2 = '1' then
                s_ma <= resize(s_fv_n, 24); s_mb <= s_b; s_macc <= (others => '0');
                s_mjob <= "10"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "00"   => s_a <= s_macc(16 downto 5);
                    when "01"   => s_b <= s_macc(16 downto 5);
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
    -- Source side: oscillator, line-buffer write, per-line |dy| and stripes
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_d : signed(11 downto 0);
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

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_fv_n <= to_unsigned(540, 11);
                s_sph <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_sph <= s_sph + resize(s_spk, 12);
            elsif s_fall2 = '1' then
                v_d := signed(resize(s_vy_n(19 downto 8), 12)) - 540;
                if v_d < 0 then s_fv_n <= resize(unsigned(-v_d), 11); else s_fv_n <= unsigned(v_d(10 downto 0)); end if;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_fvb <= s_fvb_n;
                s_spar <= s_sph(11) and s_stripes;
            end if;
        end if;
    end process p_src;

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
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_pi : integer range 0 to 15;
        variable v_ci : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: |x| about the cell centre (Q.1), left-of-centre flag, mirrored read offset
            v_f := signed(resize(s_cu, 14)) - signed(resize(s_cwv, 14));
            if v_f < 0 then s_fu1 <= resize(unsigned(-v_f), 13); s_lft1 <= '1';
            else s_fu1 <= unsigned(v_f(12 downto 0)); s_lft1 <= '0'; end if;
            if s_cupx < s_cdpx then s_m1 <= s_cupx; else s_m1 <= s_cdpx; end if;
            s_cs1 <= s_cs;

            -- 2: a|x| partial products; read address
            s_plo2 <= s_fu1 * s_a(7 downto 0);
            s_phi2 <= s_fu1 * s_a(11 downto 8);
            s_rd2 <= s_cs1 + s_m1;
            s_lft(2) <= s_lft1;

            -- 3: a|x|
            s_pa3 <= resize(s_plo2, 25) + shift_left(resize(s_phi2, 25), 8);
            s_rd3 <= s_rd2;

            -- 4: field = a|x| + b|y|; video read issued
            s_s4 <= resize(s_pa3, 26) + resize(s_fvb, 26);
            s_lb_ra <= s_rd3;

            -- 5: drift
            s_s5 <= s_s4 + resize(s_ph, 26);

            -- 6: video luma (fabric copy)
            s_y6 <= unsigned(s_lb_q(31 downto 22));
            s_s6 <= s_s5;

            -- 7: etch product
            if s_y6 > 64 then s_m7 <= (s_y6 - 64) * s_etk; else s_m7 <= (others => '0'); end if;
            s_s7 <= s_s6;

            -- 8: etched field
            s_s8 <= s_s7 + resize(s_m7, 26);

            -- 9: ring index, highlight (ring start on one side)
            s_idx9 <= s_s8(15 downto 14);
            if s_s8(13 downto 6) < s_hw and (s_lft(8) xor s_side) = '1' then s_hl9 <= '1'; else s_hl9 <= '0'; end if;

            -- 10: colour: 0 dark, 1 A, 2 B, 3 A (stripes / Swap exchange A and B)
            v_ci := s_idx9;
            if v_ci = "11" then v_ci := "01"; end if;
            if v_ci /= "00" and (s_spar xor s_swap) = '1' then v_ci := not v_ci; end if;
            if s_hl9 = '1' then v_ci := "11"; end if;
            v_pi := to_integer(s_pals & v_ci);
            s_y10 <= unsigned(C_PAL(v_pi)(29 downto 20));
            s_u10 <= unsigned(C_PAL(v_pi)(19 downto 10));
            s_v10 <= unsigned(C_PAL(v_pi)(9 downto 0));

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
                s_yp(17) <= s_yf16 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_up(17) <= s_u16; s_vp(17) <= s_v16;
            else
                s_yp(17) <= to_unsigned(64, 10); s_up(17) <= to_unsigned(512, 10); s_vp(17) <= to_unsigned(512, 10);
            end if;
            for i in 18 to 20 loop s_yp(i) <= s_yp(i - 1); s_up(i) <= s_up(i - 1); s_vp(i) <= s_vp(i - 1); end loop;

            for i in 3 to 8 loop s_lft(i) <= s_lft(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_yp(20));
    data_out.u       <= std_logic_vector(s_up(20));
    data_out.v       <= std_logic_vector(s_vp(20));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lozenge;
