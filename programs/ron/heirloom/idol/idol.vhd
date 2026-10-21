-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Idol v0.1.0 -- Heirloom #31 (2026-10-20), from WIN_20230515_23_14_37.
--
-- The photo: a flat yellow ground; down the centre a column of blue / orange /
-- red interlocking bulbous shapes (stacked masks), mirrored left/right and
-- smeared sideways -- the relief-shaded, mirrored picture.
--
-- Inside a centre column the (mirrored) source is smeared along the line and
-- relief-shaded: the horizontal derivative paints rising edges blue and
-- falling edges orange, luma by the slope.  Outside the column: flat yellow
-- (or the plain picture).
--
--   K1  Smear     horizontal smear of the source
--   K2  Relief    relief gain
--   K3  Palette   blue/orange, green/magenta, cyan/red, violet/yellow
--   K4  Poster    posterize depth for the Flat mode
--   K5  Centre    column position
--   K6  Sway      the column sways side to side
--   S7  Mirror    picture mirrored about the centre / plain
--   S8  Shading   Relief / Flat (posterized picture)
--   S9  Lit       edges lit from the left / right (sign of the relief)
--   S10 Sides     Yellow / the plain picture
--   S11 Source    Video / Osc
--   P12 COLUMN    column width: a sliver .. the whole screen
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

architecture idol of program_top is
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
    type t_pdef is array (0 to 7) of t_col;
    -- pairs: rising-edge colour, falling-edge colour
    constant C_PDEF : t_pdef := (
        (164, 440, 960), (584, 772, 212),
        (580, 136, 216), (426, 887, 808),
        (678, 64, 663), (328, 960, 360),
        (349, 756, 852), (840, 584, 64));

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

    signal s_sk : unsigned(2 downto 0) := (others => '0');
    signal s_gain : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_palk : unsigned(1 downto 0) := (others => '0');
    signal s_pk : unsigned(1 downto 0) := (others => '0');
    signal s_off : signed(9 downto 0) := (others => '0');
    signal s_sway : unsigned(5 downto 0) := (others => '0');
    signal s_sw : unsigned(7 downto 0) := (others => '0');
    signal s_off2 : signed(13 downto 0) := (others => '0');
    signal s_cw2 : unsigned(11 downto 0) := to_unsigned(600, 12);
    signal s_mirror, s_flat, s_lit, s_plain : std_logic := '0';
    -- beam
    signal s_dxo : signed(13 downto 0) := (others => '0');
    signal s_cd : unsigned(10 downto 0) := (others => '0');
    -- pixel path
    signal s_in : t_sl(1 to 11) := (others => '0');
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ud, s_vd : t_u10(4 to 11) := (others => to_unsigned(512, 10));
    signal s_yd : t_u10(4 to 11) := (others => to_unsigned(64, 10));
    signal s_sm4, s_sm5, s_sm6, s_sm7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_smd : t_u10(8 to 10) := (others => to_unsigned(64, 10));
    signal s_d8 : signed(10 downto 0) := (others => '0');
    signal s_ad9 : unsigned(9 downto 0) := (others => '0');
    signal s_p9 : unsigned(17 downto 0) := (others => '0');
    signal s_sg9 : std_logic := '0';
    signal s_y10, s_u10, s_v10, s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(64, 10);

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
        variable v_t : signed(7 downto 0);
        variable v_sp : signed(13 downto 0);
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
            s_sk <= unsigned(registers_in(0)(9 downto 7));                     -- K1 Smear 0 .. 7
            s_gain <= unsigned(registers_in(1)(9 downto 2));                   -- K2 Relief
            s_palk <= unsigned(registers_in(2)(9 downto 8));                   -- K3 Palette
            s_pk <= unsigned(registers_in(3)(9 downto 8));                     -- K4 Poster 0 .. 3
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;        -- K5 Centre
            s_off <= v_s(9 downto 0);
            s_sway <= unsigned(registers_in(5)(9 downto 4));                   -- K6 Sway
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_mirror <= registers_in(6)(0);
                s_flat   <= registers_in(6)(1);
                s_lit    <= registers_in(6)(2);
                s_plain  <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_sw <= s_sw + 2;
                s_cw2 <= to_unsigned(64, 12) + resize(s_g(11 downto 1), 12);
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
            -- column offset: knob + sway triangle x Sway, doubled (Q.1)
            v_t := signed(resize(f_tri8(s_sw), 8)) - 64;
            v_sp := resize(v_t * signed(resize(s_sway, 7)), 14);
            s_off2 <= shift_left(resize(s_off, 14), 1) + shift_right(v_sp, 2);

            -- serial multiplier (12 steps): A x B -> s_macc
            if s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is

                    when others => null;
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
    -- Source side: oscillator, line-buffer write, per-line values (next at the
    -- source avid fall, committed at its rise)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_dy : signed(12 downto 0);

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
                s_dxo <= -signed(resize(s_vw, 14)) - s_off2;
                s_cd <= s_wm1(10 downto 0);
            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
                s_vx <= s_vx + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 13));
                s_dxo <= s_dxo + signed(resize(s_xs, 14));
                s_cd <= s_cd - 1;
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
        variable v_d : signed(10 downto 0);
        variable v_l : unsigned(12 downto 0);
        variable v_c : t_col;
        variable v_i : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: inside the column? mirrored read
            if s_dxo < 0 then v_ad := -s_dxo; else v_ad := s_dxo; end if;
            if unsigned(v_ad(12 downto 0)) < resize(s_cw2, 13) then s_in(1) <= '1'; else s_in(1) <= '0'; end if;
            if s_mirror = '1' and s_cd < s_bcol then s_lb_ra <= s_cd; else s_lb_ra <= s_bcol; end if;

            -- 2: buffer register; 3: fabric copy
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 4: smear (one-pole along the line)
            v_d := signed(resize(s_y3, 11)) - signed(resize(s_sm4, 11));
            case to_integer(s_sk) is
                when 0 => s_sm4 <= s_y3;
                when 1 => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 1), 10));
                when 2 => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 2), 10));
                when 3 => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 3), 10));
                when 4 => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 4), 10));
                when others => s_sm4 <= unsigned(resize(signed(resize(s_sm4, 11)) + shift_right(v_d, 5), 10));
            end case;
            if s_in(3) = '0' then s_sm4 <= s_y3; end if;
            s_yd(4) <= s_y3; s_ud(4) <= s_u3; s_vd(4) <= s_v3;

            -- 5..7: baseline window
            s_sm5 <= s_sm4; s_sm6 <= s_sm5; s_sm7 <= s_sm6;
            -- 8: derivative over 3 px
            if s_lit = '1' then
                s_d8 <= signed(resize(s_sm7, 11)) - signed(resize(s_sm4, 11));
            else
                s_d8 <= signed(resize(s_sm4, 11)) - signed(resize(s_sm7, 11));
            end if;
            s_smd(8) <= s_sm7;
            -- 9: magnitude x gain, sign
            if s_d8 < 0 then s_ad9 <= unsigned(resize(-s_d8, 10)); s_sg9 <= '1'; else s_ad9 <= unsigned(s_d8(9 downto 0)); s_sg9 <= '0'; end if;
            s_p9 <= s_ad9 * s_gain;
            -- 10: relief colour / flat posterized
            v_l := resize(s_p9(17 downto 4), 13) + 300;
            if v_l > 940 then v_l := to_unsigned(940, 13); end if;
            v_i := s_palk & s_sg9;
            v_c := C_PDEF(to_integer(v_i));
            if s_flat = '1' then
                case to_integer(s_pk) is
                    when 0 => s_y10 <= s_smd(9);
                    when 1 => s_y10 <= s_smd(9)(9 downto 6) & "000000";
                    when 2 => s_y10 <= s_smd(9)(9 downto 7) & "0000000";
                    when others => s_y10 <= s_smd(9)(9 downto 8) & "00000000";
                end case;
                s_u10 <= s_ud(9); s_v10 <= s_vd(9);
            else
                s_y10 <= v_l(9 downto 0);
                s_u10 <= to_unsigned(v_c(1), 10); s_v10 <= to_unsigned(v_c(2), 10);
            end if;
            -- 11: compose with the sides
            if s_in(10) = '1' then
                s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            elsif s_plain = '1' then
                s_y11 <= s_yd(10); s_u11 <= s_ud(10); s_v11 <= s_vd(10);
            else
                s_y11 <= to_unsigned(840, 10); s_u11 <= to_unsigned(584, 10); s_v11 <= to_unsigned(64, 10);
            end if;
            -- 12
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;

            for i in 2 to 11 loop s_in(i) <= s_in(i - 1); end loop;
            for i in 5 to 11 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            for i in 9 to 10 loop s_smd(i) <= s_smd(i - 1); end loop;

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

end architecture idol;
