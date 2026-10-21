-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lava v0.1.0 -- Heirloom #43 (2026-10-20), from WIN_20230605_01_14_56.
--
-- The photo: red / orange flame lobes with white cores dripping down the
-- screen, blue-violet pods along the bottom, a magenta / violet vertical-stripe
-- curtain at the left, everything smeared sideways.
--
-- Two column oscillators (tri of x at two pitches) make the lobes; a threshold
-- that rises down the screen thins them to drips (P12 = how far they reach);
-- the source luma, smeared, glows inside them.  Diamond-metric cells along the
-- bottom make the pods; a striped curtain stands at the left.
--
--   K1  Lobes     lobe pitch across
--   K2  Pods      pod cell size (and the pod band height)
--   K3  Curtain   curtain width at the left
--   K4  Glow      lobe core brightness
--   K5  Smear     horizontal smear of the source
--   K6  Drift     lobes drift sideways, bipolar
--   S7  Curtain   on / off
--   S8  Pods      on / off
--   S9  Palette   Lava / Ice
--   S10 Mirror    lobes folded about the centre column
--   S11 Source    Video / Osc
--   P12 MELT      how far down the lobes drip
--
-- Shared Heirloom plumbing: generated avid (parity-locked start, field-latched
-- width; beam counters initialised on the g_start edge), vblank zone (no avid
-- for 1024 clocks) for every per-field event, virtual 1080-line geometry from
-- the timing ID, a Source switch (Video / Osc: an internal triangle-wave
-- oscillator replaces the video at the line-buffer write, so every video-path
-- control stays live), output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16,
-- luma [1 2 1], luma to ~777) before the blanking gate.  Latency 20 clocks in
-- every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture lava of program_top is
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
    -- 13-bit signed -> legal 10-bit video code
    function f_clamp(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 960 then return to_unsigned(960, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;
    -- saturate a 13-bit unsigned to 8 bits
    function f_sat8(v : unsigned(12 downto 0)) return unsigned is
    begin
        if v(12 downto 8) /= 0 then return to_unsigned(255, 8); else return v(7 downto 0); end if;
    end function;

    type t_key is record
        i, y, u, v : integer;
    end record;
    type t_keys is array (natural range <>) of t_key;
    -- 0 lobes lava: dark red -> red -> orange -> yellow -> white
    constant C_K0 : t_keys := ((0, 200, 800, 420), (64, 328, 960, 360), (128, 584, 772, 212), (192, 840, 584, 64), (255, 940, 512, 512));
    -- 1 pods: violet core -> blue -> dark
    constant C_K1 : t_keys := ((0, 600, 700, 800), (96, 349, 756, 852), (160, 164, 440, 960), (255, 80, 512, 560));
    -- 2 curtain: magenta -> violet -> indigo -> magenta
    constant C_K2 : t_keys := ((0, 426, 887, 808), (96, 349, 756, 852), (176, 192, 608, 696), (255, 426, 887, 808));
    -- 3 ground (by the source luma): black -> olive -> green
    constant C_K3 : t_keys := ((0, 64, 512, 512), (128, 300, 480, 320), (255, 580, 136, 216));
    -- 4 lobes ice: navy -> blue -> cyan -> white -> pink
    constant C_K4 : t_keys := ((0, 100, 470, 780), (80, 164, 440, 960), (160, 678, 64, 663), (224, 940, 512, 512), (255, 752, 612, 512));
    type t_pal is array (0 to 2047) of std_logic_vector(31 downto 0);
    function f_lerp(a, b, t, n : integer) return integer is
    begin
        return a + ((b - a) * t) / n;
    end function;
    function f_seg(k : t_keys; i : integer) return std_logic_vector is
        variable y, u, v : integer := 0;
    begin
        for s in 0 to k'length - 2 loop
            if i >= k(s).i and i <= k(s + 1).i then
                y := f_lerp(k(s).y, k(s + 1).y, i - k(s).i, k(s + 1).i - k(s).i);
                u := f_lerp(k(s).u, k(s + 1).u, i - k(s).i, k(s + 1).i - k(s).i);
                v := f_lerp(k(s).v, k(s + 1).v, i - k(s).i, k(s + 1).i - k(s).i);
            end if;
        end loop;
        return std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
               & std_logic_vector(to_unsigned(v, 10)) & "00";
    end function;
    function f_pal return t_pal is
        variable r : t_pal;
    begin
        for i in 0 to 255 loop
            r(i) := f_seg(C_K0, i);
            r(256 + i) := f_seg(C_K1, i);
            r(512 + i) := f_seg(C_K2, i);
            r(768 + i) := f_seg(C_K3, i);
            r(1024 + i) := f_seg(C_K4, i);
            r(1280 + i) := f_seg(C_K1, i);
            r(1536 + i) := f_seg(C_K2, i);
            r(1792 + i) := f_seg(C_K3, i);
        end loop;
        return r;
    end function;
    signal pal : t_pal := f_pal;
    signal s_pal_ra : unsigned(10 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    -- internal oscillator source (Source = Osc)
    signal s_src : std_logic := '0';
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);     -- virtual width = 2 cx
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');            -- glided P12, Q.2
    -- serial multiplier
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
    signal s_fall1, s_fall2, s_rise : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_vyi : unsigned(10 downto 0) := (others => '0');          -- committed virtual line
    signal s_dy : signed(11 downto 0) := to_signed(-540, 12);         -- vy - 540
    signal s_ady : unsigned(10 downto 0) := to_unsigned(540, 11);

    -- beam state (real column, virtual x in Q.1, 2 dx)
    signal s_bcol : unsigned(10 downto 0) := (others => '0');
    signal s_vx : unsigned(12 downto 0) := (others => '0');
    signal s_dxh : signed(12 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_u7 is array (natural range <>) of unsigned(6 downto 0);
    type t_sl is array (natural range <>) of std_logic;
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

    signal s_fx : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_fxs, s_fxb : unsigned(9 downto 0) := to_unsigned(16, 10);
    signal s_cs : unsigned(9 downto 0) := to_unsigned(200, 10);       -- pod cell size (virtual)
    signal s_cs2 : unsigned(10 downto 0) := to_unsigned(400, 11);
    signal s_csh : unsigned(9 downto 0) := to_unsigned(100, 10);
    signal s_csq : unsigned(17 downto 0) := (others => '0');
    signal s_pcap : unsigned(9 downto 0) := to_unsigned(75, 10);      -- pod radius 3/4 of half a cell
    signal s_podline : unsigned(10 downto 0) := to_unsigned(680, 11);
    signal s_cur2 : unsigned(12 downto 0) := to_unsigned(400, 13);    -- curtain width, Q.1
    signal s_glow : unsigned(7 downto 0) := (others => '0');
    signal s_sk : unsigned(2 downto 0) := (others => '0');
    signal s_spd : signed(6 downto 0) := (others => '0');
    signal s_ph : unsigned(11 downto 0) := (others => '0');
    signal s_reach : signed(12 downto 0) := to_signed(400, 13);
    signal s_curt, s_pods, s_ice, s_mirror : std_logic := '0';
    signal s_xa0, s_xb0 : unsigned(11 downto 0) := (others => '0');
    -- per line
    signal s_cv_n : unsigned(18 downto 0) := (others => '0');
    signal s_fv_n, s_fv : unsigned(9 downto 0) := (others => '0');
    signal s_thr_n, s_thr : unsigned(6 downto 0) := (others => '0');
    signal s_prow_n, s_prow : std_logic := '0';
    signal s_fall3 : std_logic := '0';
    -- beam
    signal s_xa, s_xb, s_xc : unsigned(11 downto 0) := (others => '0');
    signal s_cu : unsigned(10 downto 0) := (others => '0');
    -- pixel path
    signal s_t1, s_t2, s_tc1 : unsigned(6 downto 0) := (others => '0');
    signal s_pr1 : unsigned(9 downto 0) := (others => '0');
    signal s_cur : t_sl(1 to 5) := (others => '0');
    signal s_tc : t_u7(2 to 5) := (others => (others => '0'));
    signal s_l2 : unsigned(6 downto 0) := (others => '0');
    signal s_pr2 : unsigned(10 downto 0) := (others => '0');
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ls3, s_ls4 : signed(8 downto 0) := (others => '0');
    signal s_pod3, s_pod4 : std_logic := '0';
    signal s_pr3, s_pr4 : unsigned(7 downto 0) := (others => '0');
    signal s_sm4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py7, s_pu7, s_pv7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y8, s_u8, s_v8, s_y9, s_u9, s_v9, s_y10, s_u10, s_v10, s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(64, 10);

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

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            s_pal_q <= pal(to_integer(s_pal_ra));
        end if;
    end process p_pal;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
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

            -- knobs (free-running decode)
            s_fx <= resize(unsigned(registers_in(0)(9 downto 4)), 7) + 2;      -- K1 Lobes 2 .. 65
            s_cs <= to_unsigned(64, 10) + resize(unsigned(registers_in(1)(9 downto 1)), 10);   -- K2 Pods 64 .. 575
            s_cur2 <= resize(unsigned(registers_in(2)(9 downto 1)) & '0', 13);                  -- K3 Curtain 0 .. 1022 (Q.1)
            s_glow <= unsigned(registers_in(3)(9 downto 2));                   -- K4 Glow
            s_sk <= unsigned(registers_in(4)(9 downto 7));                     -- K5 Smear 0 .. 7
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;        -- K6 Drift, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(6 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_curt   <= registers_in(6)(0);
                s_pods   <= registers_in(6)(1);
                s_ice    <= registers_in(6)(2);
                s_mirror <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_ph <= s_ph + unsigned(resize(s_spd, 12));
                -- reach: -400 .. 1080 virtual lines (lobes full above it, thinning below)
                s_reach <= signed(resize(s_g(11 downto 2), 13)) + signed(resize(s_g(11 downto 3), 13)) - 400;
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
            case to_integer(s_xs) is
                when 4      => s_fxs <= shift_left(resize(s_fx, 10), 2);
                when 3      => s_fxs <= shift_left(resize(s_fx, 10), 1) + resize(s_fx, 10);
                when others => s_fxs <= shift_left(resize(s_fx, 10), 1);
            end case;
            s_fxb <= s_fxs + resize(s_fxs(9 downto 1), 10);
            s_cs2 <= s_cs & '0';
            s_csh <= '0' & s_cs(9 downto 1);
            s_csq <= s_cs & x"00";
            s_pcap <= resize(s_cs(9 downto 2), 10) + resize(s_cs(9 downto 3), 10);
            s_podline <= to_unsigned(1080, 11) - resize(s_cs, 11) - resize(s_cs(9 downto 1), 11);

            -- serial multiplier (12 steps): A x B -> s_macc
            if s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_xa0 <= s_macc(12 downto 1);
                    when "001"  => s_xb0 <= s_macc(12 downto 1);
                    when others => null;
                end case;
            end if;
            if s_zc = 4 then
                s_ma <= resize(s_vw, 24); s_mb <= resize(s_fxs, 12); s_macc <= (others => '0');
                s_mjob <= "000"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 24 then
                s_ma <= resize(s_vw, 24); s_mb <= resize(s_fxb, 12); s_macc <= (others => '0');
                s_mjob <= "001"; s_mrun <= '1'; s_mi <= (others => '0');
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
    -- Source side: oscillator, line-buffer write, per-line values (next at the
    -- source avid fall, committed at its rise)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_dy : signed(12 downto 0);
        variable v_c : unsigned(19 downto 0);
        variable v_cv : unsigned(9 downto 0);
        variable v_t : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator source: x ramp per source pixel, y ramp per line, phase per field
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
            s_rise <= '0';
            if data_in.avid = '1' and s_avq = '0' then s_rise <= '1'; end if;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
            end if;
            if data_in.avid = '1' and s_avq = '0' then
                s_vyi <= s_vy_n(18 downto 8);
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - 540;
                s_dy <= v_dy(11 downto 0);
                if v_dy < 0 then s_ady <= resize(unsigned(-v_dy), 11); else s_ady <= unsigned(v_dy(10 downto 0)); end if;
            end if;
            if s_zone = '1' then
                s_cv_n <= (others => '0');
            elsif s_fall1 = '1' then
                v_c := resize(s_cv_n, 20) + s_ys;
                if v_c >= s_csq then s_cv_n <= resize(v_c - s_csq, 19); else s_cv_n <= v_c(18 downto 0); end if;
            end if;
            s_fall3 <= s_fall2;
            if s_fall2 = '1' then
                v_cv := s_cv_n(17 downto 8);
                if v_cv >= s_csh then s_fv_n <= v_cv - s_csh; else s_fv_n <= s_csh - v_cv; end if;
                v_t := signed(resize(s_vy_n(19 downto 8), 13)) - s_reach;
                if v_t <= 0 then s_thr_n <= (others => '0');
                elsif v_t >= 508 then s_thr_n <= to_unsigned(127, 7);
                else s_thr_n <= unsigned(v_t(8 downto 2)); end if;
                if s_vy_n(18 downto 8) >= s_podline then s_prow_n <= s_pods; else s_prow_n <= '0'; end if;
            end if;
            if s_rise = '1' then
                s_fv <= s_fv_n;
                s_thr <= s_thr_n;
                s_prow <= s_prow_n;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_bcol <= (others => '0');
                s_vx <= (others => '0');
                s_dxh <= -signed(resize(s_vw, 13));
                if s_mirror = '1' then s_xa <= s_xa0 + s_ph; s_xb <= s_xb0 + s_ph; else s_xa <= s_ph; s_xb <= s_ph; end if;
                s_xc <= (others => '0');
                s_cu <= (others => '0');
            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
                s_vx <= s_vx + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 13));
                if s_mirror = '1' and s_dxh < 0 then
                    s_xa <= s_xa - s_fxs; s_xb <= s_xb - s_fxb;
                else
                    s_xa <= s_xa + s_fxs; s_xb <= s_xb + s_fxb;
                end if;
                s_xc <= s_xc + 24;
                v_n := resize(s_cu, 12) + s_xs;
                if v_n >= s_cs2 then s_cu <= resize(v_n - s_cs2, 11); else s_cu <= v_n(10 downto 0); end if;
            end if;

        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = first registers from the beam state; compose by 12)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_d : signed(10 downto 0);
        variable v_i : unsigned(9 downto 0);
        variable v_cu : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: the two lobe waves, the curtain stripe, pod x distance, video read
            s_t1 <= f_tri8(s_xa(11 downto 4));
            s_t2 <= f_tri8(s_xb(11 downto 4) + 64);
            s_tc1 <= f_tri8(s_xc(11 downto 4));
            if s_vx < s_cur2 then s_cur(1) <= s_curt; else s_cur(1) <= '0'; end if;
            v_cu := s_cu(10 downto 1);
            if v_cu >= s_csh then s_pr1 <= v_cu - s_csh; else s_pr1 <= s_csh - v_cu; end if;
            s_lb_ra <= s_bcol;

            -- 2: lobe level = max of the two waves; pod diamond distance
            if s_t1 > s_t2 then s_l2 <= s_t1; else s_l2 <= s_t2; end if;
            s_pr2 <= resize(s_pr1, 11) + resize(s_fv, 11);
            s_tc(2) <= s_tc1;

            -- 3: lobe strength above the line threshold; pod flag; video fabric copy
            s_ls3 <= signed(resize(s_l2, 9)) - signed(resize(s_thr, 9));
            if s_prow = '1' and s_pr2 < resize(s_pcap, 11) then s_pod3 <= '1'; else s_pod3 <= '0'; end if;
            s_pr3 <= s_pr2(7 downto 0);
            s_y3 <= unsigned(s_lb_q(31 downto 22));

            -- 4: smear the source luma (one-pole along the line)
            v_d := signed(resize(s_y3, 11)) - signed(resize(s_sm4, 11));
            case to_integer(s_sk) is
                when 0 => s_sm4 <= s_y3;
                when 1 => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 1), 10));
                when 2 => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 2), 10));
                when 3 => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 3), 10));
                when 4 => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 4), 10));
                when others => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 5), 10));
            end case;
            s_ls4 <= s_ls3; s_pod4 <= s_pod3; s_pr4 <= s_pr3;

            -- 5: palette address: lobe / pod / curtain / ground
            if s_ls4 > 0 then
                v_i := resize(unsigned(s_ls4(6 downto 0)) & '0', 10) + resize(s_glow, 10) + resize(s_sm4(9 downto 4), 10);
                if v_i > 255 then v_i := to_unsigned(255, 10); end if;
                s_pal_ra <= s_ice & "00" & v_i(7 downto 0);
            elsif s_pod4 = '1' then
                s_pal_ra <= s_ice & "01" & (s_pr4 + s_sm4(9 downto 5));
            elsif s_cur(4) = '1' then
                s_pal_ra <= s_ice & "10" & ('0' & s_tc(4));
            else
                s_pal_ra <= s_ice & "11" & s_sm4(9 downto 2);
            end if;

            -- 6: ROM register; 7: fabric copy
            s_py7 <= unsigned(s_pal_q(31 downto 22));
            s_pu7 <= unsigned(s_pal_q(21 downto 12));
            s_pv7 <= unsigned(s_pal_q(11 downto 2));
            -- 8..12
            s_y8 <= s_py7; s_u8 <= s_pu7; s_v8 <= s_pv7;
            s_y9 <= s_y8; s_u9 <= s_u8; s_v9 <= s_v8;
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;

            for i in 2 to 5 loop s_cur(i) <= s_cur(i - 1); end loop;
            for i in 3 to 5 loop s_tc(i) <= s_tc(i - 1); end loop;

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
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lava;
