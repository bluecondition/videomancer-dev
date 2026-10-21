-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Blossom v0.1.0 -- Heirloom #22 (2026-10-20), from WIN_20230406_23_49_00.
--
-- The photo: mirrored left/right, nested red / green / magenta chevron
-- "petals" opening leftward from the centre line, flat magenta to the right,
-- hot-pink lobes at the petal tips.
--
-- Petal rings: the field |x| + |y mod period - period/2| is cut into bands
-- (Pitch) and painted from an 8-colour cycle; beyond a boundary column the
-- screen is flat magenta, and the boundary sweeps across with P12 so the
-- petals open.  The source luma offsets the field (the picture inside the
-- petals).
--
--   K1  Pitch     ring density
--   K2  Period    petal period down the screen
--   K3  Hue       palette rotation
--   K4  Ragged    boundary raggedness (per-line sawtooth)
--   K5  Drift     rings flow in / out, bipolar
--   K6  Centre    petal centre, left .. right
--   S7  Mirror    petals folded about the centre / a one-sided ramp
--   S8  Palette   Garden / Ember
--   S9  Rings     Colour rings / Solid magenta shading
--   S10 Lobes     pink lobes at the petal tips on / off
--   S11 Source    Video / Osc
--   P12 OPEN      the boundary: all magenta .. the petals fill the screen
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

architecture blossom of program_top is
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

    type t_col is array (0 to 2) of integer;
    type t_pdef is array (0 to 15) of t_col;
    constant C_PDEF : t_pdef := (
        (328, 960, 360), (580, 136, 216), (426, 887, 808), (752, 612, 512), (200, 800, 420), (700, 300, 300), (349, 756, 852), (560, 760, 560),
        (328, 960, 360), (584, 772, 212), (840, 584, 64), (940, 512, 512), (200, 800, 420), (120, 560, 640), (426, 887, 808), (64, 512, 512));

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

    signal s_zk : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_per : unsigned(10 downto 0) := to_unsigned(300, 11);
    signal s_perq : unsigned(18 downto 0) := (others => '0');
    signal s_perh : unsigned(9 downto 0) := to_unsigned(150, 10);
    signal s_lobe : unsigned(7 downto 0) := to_unsigned(37, 8);
    signal s_hue : unsigned(2 downto 0) := (others => '0');
    signal s_rag : unsigned(8 downto 0) := (others => '0');
    signal s_spd : signed(6 downto 0) := (others => '0');
    signal s_off2 : signed(10 downto 0) := (others => '0');
    signal s_drift : unsigned(11 downto 0) := (others => '0');
    signal s_bnd : signed(12 downto 0) := (others => '0');            -- boundary (virtual, signed from centre)
    signal s_mirror, s_ember, s_solid, s_lobes : std_logic := '0';
    -- per line
    signal s_ym_n : unsigned(18 downto 0) := (others => '0');
    signal s_fm_n, s_fm : unsigned(9 downto 0) := (others => '0');
    signal s_lob_n, s_lob : std_logic := '0';
    signal s_bl_n, s_bl : signed(12 downto 0) := (others => '0');
    signal s_fall3 : std_logic := '0';
    signal s_rg : unsigned(11 downto 0) := (others => '0');
    -- beam
    signal s_dxo : signed(13 downto 0) := (others => '0');
    -- pixel path
    signal s_adx1 : unsigned(11 downto 0) := (others => '0');
    signal s_flat : t_sl(1 to 8) := (others => '0');
    signal s_f2, s_f3, s_f4 : unsigned(12 downto 0) := (others => '0');
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_p5 : unsigned(22 downto 0) := (others => '0');
    signal s_idx6 : unsigned(3 downto 0) := (others => '0');
    signal s_y7, s_u7, s_v7, s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y9, s_u9, s_v9, s_y10, s_u10, s_v10, s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(64, 10);

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
            s_zk <= to_unsigned(16, 10) + resize(unsigned(registers_in(0)(9 downto 1)), 10);       -- K1 Pitch 16 .. 527
            s_per <= to_unsigned(128, 11) + resize(unsigned(registers_in(1)), 11);                -- K2 Period 128 .. 1151
            s_hue <= unsigned(registers_in(2)(9 downto 7));                                       -- K3 Hue (8 steps)
            s_rag <= unsigned(registers_in(3)(9 downto 1));                                       -- K4 Ragged 0 .. 511
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;                           -- K5 Drift, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(6 downto 0); end if;
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;                           -- K6 Centre, 2 x offset
            s_off2 <= v_s;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_mirror <= registers_in(6)(0);
                s_ember  <= registers_in(6)(1);
                s_solid  <= registers_in(6)(2);
                s_lobes  <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_drift <= s_drift + unsigned(resize(s_spd, 12));
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
            s_perq <= s_per & x"00";
            s_perh <= s_per(10 downto 1);
            s_lobe <= s_per(10 downto 3);
            -- boundary = (vw x p12) >> 10 - vw/2, from the serial product
            s_bnd <= signed(resize(s_rg, 13)) - signed(resize(s_vw(11 downto 1), 13));

            -- serial multiplier (12 steps): A x B -> s_macc
            if s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_rg <= s_macc(21 downto 10);
                    when others => null;
                end case;
            end if;
            if s_zc = 4 then
                s_ma <= resize(s_vw, 24); s_mb <= resize(s_g(11 downto 2), 12); s_macc <= (others => '0');
                s_mjob <= "000"; s_mrun <= '1'; s_mi <= (others => '0');
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
        variable v_ym : unsigned(10 downto 0);
        variable v_j : unsigned(11 downto 0);
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
                s_ym_n <= (others => '0');
            elsif s_fall1 = '1' then
                v_c := resize(s_ym_n, 20) + s_ys;
                if v_c >= s_perq then s_ym_n <= resize(v_c - s_perq, 19); else s_ym_n <= v_c(18 downto 0); end if;
            end if;
            s_fall3 <= s_fall2;
            if s_fall2 = '1' then
                v_ym := s_ym_n(18 downto 8);
                if v_ym >= resize(s_perh, 11) then s_fm_n <= resize(v_ym - resize(s_perh, 11), 10);
                else s_fm_n <= resize(resize(s_perh, 11) - v_ym, 10); end if;
                -- ragged boundary: a 3-line sawtooth of up to Ragged units
                v_j := (others => '0');
                if s_vy_n(8) = '1' then v_j := v_j + resize(s_rag, 12); end if;
                if s_vy_n(9) = '1' then v_j := v_j + shift_left(resize(s_rag, 12), 1); end if;
                if s_vy_n(10) = '1' then v_j := v_j + shift_left(resize(s_rag, 12), 2); end if;
                s_bl_n <= s_bnd + signed(resize(v_j(11 downto 3), 13));
            elsif s_fall3 = '1' then
                if s_fm_n < resize(s_lobe, 10) then s_lob_n <= s_lobes; else s_lob_n <= '0'; end if;
            end if;
            if s_rise = '1' then
                s_fm <= s_fm_n;
                s_lob <= s_lob_n;
                s_bl <= s_bl_n;
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
                s_bcol <= (others => '0');
                s_vx <= (others => '0');
                s_dxh <= -signed(resize(s_vw, 13));
                s_dxo <= -signed(resize(s_vw, 14)) - resize(s_off2, 14);
            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
                s_vx <= s_vx + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 13));
                s_dxo <= s_dxo + signed(resize(s_xs, 14));
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
        variable v_ad : signed(13 downto 0);
        variable v_i : unsigned(3 downto 0);
        variable v_c : t_col;
    begin
        if rising_edge(clk) then
            -- 1: folded / ramp distance, boundary flag, video read
            if s_mirror = '1' then
                if s_dxo < 0 then v_ad := -s_dxo; else v_ad := s_dxo; end if;
            else
                v_ad := s_dxo + signed(resize(s_vw, 14));
                if v_ad < 0 then v_ad := (others => '0'); end if;
            end if;
            s_adx1 <= unsigned(v_ad(12 downto 1));
            if s_dxo(13 downto 1) > s_bl then s_flat(1) <= '1'; else s_flat(1) <= '0'; end if;
            s_lb_ra <= s_bcol;

            -- 2: field = |x| + folded row + drift
            s_f2 <= resize(s_adx1, 13) + resize(s_fm, 13) + resize(s_drift, 13);

            -- 3: video fabric copy
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_f3 <= s_f2;

            -- 4: the picture offsets the field
            s_f4 <= s_f3 + resize(s_y3(9 downto 1), 13);

            -- 5: ring product
            s_p5 <= s_f4 * s_zk;

            -- 6: band index + hue
            s_idx6 <= s_p5(13 downto 10) + resize(s_hue, 4);

            -- 7: colour from the 8-entry cycle (or solid magenta shading)
            v_i := s_ember & s_idx6(2 downto 0);
            v_c := C_PDEF(to_integer(v_i));
            if s_solid = '1' then
                s_y7 <= to_unsigned(160, 10) + resize(s_idx6 & "00000", 10);
                s_u7 <= to_unsigned(887, 10); s_v7 <= to_unsigned(808, 10);
            else
                s_y7 <= to_unsigned(v_c(0), 10); s_u7 <= to_unsigned(v_c(1), 10); s_v7 <= to_unsigned(v_c(2), 10);
            end if;

            -- 8: flat magenta beyond the boundary; pink lobes at the tips
            if s_flat(7) = '1' then
                s_y8 <= to_unsigned(426, 10); s_u8 <= to_unsigned(887, 10); s_v8 <= to_unsigned(808, 10);
            elsif s_lob = '1' and s_idx6(2 downto 0) = 0 then
                s_y8 <= to_unsigned(752, 10); s_u8 <= to_unsigned(612, 10); s_v8 <= to_unsigned(512, 10);
            else
                s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
            end if;
            -- 9..12
            s_y9 <= s_y8; s_u9 <= s_u8; s_v9 <= s_v8;
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;

            for i in 2 to 8 loop s_flat(i) <= s_flat(i - 1); end loop;

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

end architecture blossom;
