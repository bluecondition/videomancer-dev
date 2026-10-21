-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Waterfall v0.1.0 -- Heirloom #45 (2026-10-20), from WIN_20230610_00_51.
--
-- The photo: pastel colour blocks (olive / cyan / green bands) behind a
-- curtain of thin green vertical lines that bend at the bottom and spill
-- into orange / yellow ripples -- a rainbow fan.
--
-- The program: a curtain of thin lines where a triangle wave of x, bent by
-- the square of the distance below a fold line, crosses a narrow window;
-- behind it a block field (hashed pastel cells, or the source posterized
-- to pastels); below the fold an orange / yellow stripe fan.
--
--   K1  Lines    line pitch of the curtain
--   K2  Width    line width
--   K3  Blocks   block size of the background
--   K4  Fan      stripe pitch of the fan
--   K5  Hue      palette rotation
--   K6  Drift    the curtain scrolls, bipolar (centre = still)
--   S7  Curtain  on / off
--   S8  Ground   Blocks (hashed pastels) / Source (posterized to pastels)
--   S9  Fan      the stripe fan below the fold on / off
--   S10 Palette  Pastel / Vivid
--   S11 Source   Video / Osc
--   P12 FALL     bend of the curtain below the fold: straight .. swept flat
--
-- Shared Heirloom skeleton (escalator / cocoon / starburst v0.2.0 lineage):
-- timing-ID steps, vblank zone, measured width, virtual 1080-line geometry,
-- one serial multiplier in the zone (and per line), generated avid (beam
-- counters initialised on the g_start edge), Source Video / Osc mux at the
-- line-buffer write, output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16,
-- luma [1 2 1], luma to ~777) before the blanking gate.  Latency 20 clocks
-- in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture waterfall of program_top is
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
    -- clamp a 13-bit signed to the 64 .. 960 video range
    function f_clamp(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 960 then return to_unsigned(960, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;
    -- 8-bit saturating add
    function f_sat8(v : unsigned(8 downto 0)) return unsigned is
    begin
        if v(8) = '1' then return to_unsigned(255, 8); else return v(7 downto 0); end if;
    end function;
    -- colour record helpers: {y, u, v} packed 32 bits
    function f_col(y, u, v : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10))
               & std_logic_vector(to_unsigned(v, 10)) & "00";
    end function;

    -- line buffer {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- internal oscillator source (Source = Osc)
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
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);     -- virtual width = 2 cx
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');            -- glided P12, Q.2
    signal s_g10 : unsigned(9 downto 0) := (others => '0');
    -- serial multiplier
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1, s_fall2, s_fall3 : std_logic := '0';
    signal s_fcnt : unsigned(4 downto 0) := (others => '0');
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_dy_n, s_dy : signed(11 downto 0) := to_signed(-540, 12);
    signal s_ady_n, s_ady : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_vy : unsigned(10 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');          -- read start pointer

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_vx : unsigned(12 downto 0) := (others => '0');            -- Q.1 virtual x
    signal s_dxh : signed(13 downto 0) := (others => '0');             -- 2 (x - cx)

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := to_unsigned(64, 10);
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

    type t_pal is array (0 to 15) of std_logic_vector(31 downto 0);
    constant C_PAL : t_pal := (
        f_col(760, 420, 440), f_col(740, 470, 600), f_col(700, 440, 420), f_col(747, 516, 594),
        f_col(860, 540, 470), f_col(752, 612, 512), f_col(700, 512, 512), f_col(780, 400, 560),
        f_col(580, 136, 216), f_col(678, 64, 663), f_col(584, 772, 212), f_col(164, 440, 960),
        f_col(840, 584, 64), f_col(426, 887, 808), f_col(940, 512, 512), f_col(349, 756, 852));
    signal s_pal_ra : unsigned(3 downto 0) := (others => '0');
    signal s_pal_q : std_logic_vector(31 downto 0) := (others => '0');
    type t_fan is array (0 to 3) of std_logic_vector(31 downto 0);
    constant C_FAN : t_fan := (f_col(584, 772, 212), f_col(840, 584, 64), f_col(328, 960, 360), f_col(700, 660, 300));

    signal s_fx : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_lw : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_k3 : unsigned(9 downto 0) := (others => '0');
    signal s_ff : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_hue : unsigned(2 downto 0) := (others => '0');
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_curt, s_gsrc, s_fan, s_viv : std_logic := '0';
    -- derived
    signal s_fxs : unsigned(9 downto 0) := (others => '0');
    signal s_ffs : unsigned(9 downto 0) := (others => '0');
    signal s_bw : unsigned(8 downto 0) := to_unsigned(48, 9);
    signal s_bw2 : unsigned(9 downto 0) := to_unsigned(96, 10);
    signal s_bwq : unsigned(16 downto 0) := (others => '0');
    signal s_bend : unsigned(9 downto 0) := (others => '0');
    signal s_ph : unsigned(11 downto 0) := (others => '0');
    signal s_seed : unsigned(7 downto 0) := x"77";
    -- per line
    signal s_cv_n : unsigned(18 downto 0) := (others => '0');
    signal s_cyi_n, s_cyi : unsigned(6 downto 0) := (others => '0');
    signal s_yb_n : unsigned(9 downto 0) := (others => '0');            -- distance below the fold
    signal s_bphi_n, s_bphi : unsigned(11 downto 0) := (others => '0'); -- bend phase for the line
    signal s_below_n, s_below : std_logic := '0';
    signal s_fy : unsigned(11 downto 0) := (others => '0');
    signal s_fyl : unsigned(11 downto 0) := (others => '0');
    -- beam
    signal s_px : unsigned(11 downto 0) := (others => '0');
    signal s_pf : unsigned(11 downto 0) := (others => '0');
    signal s_bxa : unsigned(9 downto 0) := (others => '0');
    signal s_bxi : unsigned(6 downto 0) := (others => '0');
    -- pixel
    signal s_h1, s_h2, s_h3, s_h4 : unsigned(7 downto 0) := (others => '0');
    signal s_a1, s_f1 : unsigned(7 downto 0) := (others => '0');
    signal s_t2, s_g2 : unsigned(6 downto 0) := (others => '0');
    signal s_line : t_sl(3 to 11) := (others => '0');
    signal s_fani : t_sl(3 to 11) := (others => '0');
    signal s_fci : unsigned(1 downto 0) := (others => '0');
    signal s_fcd : t_u10(4 to 11) := (others => (others => '0'));
    signal s_gy, s_gu, s_gv : t_u10(7 to 11) := (others => to_unsigned(64, 10));

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
            s_pal_q <= C_PAL(to_integer(s_pal_ra));
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
            s_fx <= resize(unsigned(registers_in(0)(9 downto 3)), 8) + 3;
            s_lw <= resize(unsigned(registers_in(1)(9 downto 4)), 7) + 2;
            s_k3 <= unsigned(registers_in(2));
            s_ff <= resize(unsigned(registers_in(3)(9 downto 3)), 8) + 4;
            s_hue <= unsigned(registers_in(4)(9 downto 7));
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;
            if v_s > -8 and v_s < 8 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;

            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_curt <= registers_in(6)(0);
                s_gsrc <= registers_in(6)(1);
                s_fan  <= registers_in(6)(2);
                s_viv  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_ph <= s_ph + unsigned(resize(shift_right(s_k6r, 2), 12));
                s_seed <= s_seed + 1;

            end if;

            -- derived
            s_g10 <= s_g(11 downto 2);
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            case to_integer(s_xs) is
                when 4      => s_fxs <= shift_left(resize(s_fx, 10), 1); s_ffs <= shift_left(resize(s_ff, 10), 1);
                when 3      => s_fxs <= resize(s_fx, 10) + resize(s_fx(7 downto 1), 10); s_ffs <= resize(s_ff, 10) + resize(s_ff(7 downto 1), 10);
                when others => s_fxs <= resize(s_fx, 10); s_ffs <= resize(s_ff, 10);
            end case;
            s_bw <= resize(s_k3(9 downto 2), 9) + 12;
            s_bw2 <= s_bw & '0';
            s_bwq <= s_bw & x"00";
            s_bend <= s_g10;


            -- serial multiplier: per-frame jobs in the zone, per-line jobs at
            -- the source avid fall (+2 and +18 clocks)
            if s_zc = 4 or s_fall2 = '1' then
                s_ma <= resize(s_yb_n, 24); s_mb <= resize(s_yb_n, 12); s_macc <= (others => '0');
                s_mjob <= "000"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000" => s_bphi_n <= s_macc(19 downto 8);
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
    -- Source side: oscillator, line-buffer write, per-line values (computed
    -- at the source avid fall, committed at its rise)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_vy : unsigned(19 downto 0);
        variable v_dy : signed(12 downto 0);
        variable v_c : unsigned(19 downto 0);
        variable v_t : signed(12 downto 0);
        variable v_b : unsigned(21 downto 0);

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
            s_fall3 <= '0';
            if s_fall2 = '1' then s_fcnt <= to_unsigned(1, 5);
            elsif s_fcnt /= 0 then
                s_fcnt <= s_fcnt + 1;
                if s_fcnt = 16 then s_fall3 <= '1'; s_fcnt <= (others => '0'); end if;
            end if;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_dy_n <= to_signed(-540, 12);
                s_ady_n <= to_unsigned(540, 11);
                s_cv_n <= (others => '0');
                s_cyi_n <= (others => '0');
                s_yb_n <= (others => '0');
                s_below_n <= '0';
                s_fy <= (others => '0');

            elsif s_fall1 = '1' then
                v_vy := s_vy_n + s_ys;
                s_vy_n <= v_vy;
                v_dy := signed(resize(v_vy(19 downto 8), 13)) - 540;
                s_dy_n <= v_dy(11 downto 0);
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11); else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
                v_c := resize(s_cv_n, 20) + s_ys;
                if v_c >= s_bwq then s_cv_n <= resize(v_c - s_bwq, 19); s_cyi_n <= s_cyi_n + 1;
                else s_cv_n <= v_c(18 downto 0); end if;
                -- fold at y = 640 virtual: distance below it (0 .. 440)
                v_t := signed(resize(v_vy(19 downto 8), 13)) - 640;
                if v_t < 0 then s_yb_n <= (others => '0'); s_below_n <= '0';
                else s_yb_n <= unsigned(v_t(9 downto 0)); s_below_n <= '1'; end if;
                s_fy <= s_fy + s_ffs(9 downto 2) + 2;

            end if;


            if data_in.avid = '1' and s_avq = '0' then
                s_vy <= s_vy_n(18 downto 8);
                s_dy <= s_dy_n;
                s_ady <= s_ady_n;
                s_cyi <= s_cyi_n;
                s_below <= s_below_n;
                -- bend phase = (y below)^2 * bend / 4096 (per-line product, scaled)
                v_b := s_bphi_n * s_bend;
                s_bphi <= v_b(19 downto 8);
                s_fyl <= s_fy;

            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state (initialised on the g_start edge, advanced while g_avid)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(10 downto 0);

    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= s_rst;
                s_vx <= (others => '0');
                s_dxh <= -signed(resize(s_vw, 14));
                s_px <= (others => '0'); s_pf <= (others => '0');
                s_bxa <= (others => '0'); s_bxi <= (others => '0');

            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_vx <= s_vx + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 14));
                s_px <= s_px + s_fxs;
                s_pf <= s_pf + s_ffs;
                v_n := resize(s_bxa, 11) + s_xs;
                if v_n >= resize(s_bw2, 11) then s_bxa <= resize(v_n - resize(s_bw2, 11), 10); s_bxi <= s_bxi + 1;
                else s_bxa <= v_n(9 downto 0); end if;

            end if;

        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = first registers off the beam state; the line
    -- buffer is read at stage 1, its output register is stage 2, s_y3/u3/v3
    -- the fabric copy at stage 3; the program composes s_y12/u12/v12)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_h : unsigned(13 downto 0);

    begin
        if rising_edge(clk) then
            s_lb_ra <= s_rdp;
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 1: curtain phase (x ramp + bend + scroll), fan phase, block hash seed
            s_a1 <= s_px(11 downto 4) + s_bphi(11 downto 4) + s_ph(7 downto 0);
            s_f1 <= s_pf(11 downto 4) + s_fyl(11 downto 4) + s_ph(7 downto 0);
            v_h := shift_left(resize(s_bxi, 14), 6) - shift_left(resize(s_bxi, 14), 1) - resize(s_bxi, 14)
                   + shift_left(resize(s_cyi, 14), 7) - shift_left(resize(s_cyi, 14), 4) + resize(s_cyi, 14)
                   + resize(s_seed, 14);
            s_h1 <= v_h(7 downto 0);
            -- 2: triangles
            s_t2 <= f_tri8(s_a1);
            s_g2 <= f_tri8(s_f1);
            s_h2 <= s_h1 xor shift_right(s_h1, 3);
            -- 3: line hit (triangle under the width), fan colour index
            if s_curt = '1' and s_t2 < s_lw then s_line(3) <= '1'; else s_line(3) <= '0'; end if;
            if s_fan = '1' and s_below = '1' then s_fani(3) <= '1'; else s_fani(3) <= '0'; end if;
            s_fci <= s_g2(6 downto 5);
            s_h3 <= s_h2 + shift_left(s_h2, 2) + 7;
            -- 4: palette address; fan colour
            s_h4 <= s_h3 xor shift_right(s_h3, 2);
            s_fcd(4) <= resize(s_fci, 10);
            -- 5: palette address (hash, or source luma posterized)
            if s_gsrc = '1' then s_pal_ra <= s_viv & (s_y3(9 downto 7) + s_hue); else s_pal_ra <= s_viv & (s_h4(7 downto 5) + s_hue); end if;
            -- 6: ROM register; 7: fabric copy
            s_gy(7) <= unsigned(s_pal_q(31 downto 22)); s_gu(7) <= unsigned(s_pal_q(21 downto 12)); s_gv(7) <= unsigned(s_pal_q(11 downto 2));
            -- 12: compose: curtain lines (green) over the fan over the ground
            if s_line(11) = '1' then
                s_y12 <= to_unsigned(540, 10); s_u12 <= to_unsigned(200, 10); s_v12 <= to_unsigned(300, 10);
            elsif s_fani(11) = '1' then
                s_y12 <= unsigned(C_FAN(to_integer(s_fcd(11)(1 downto 0)))(31 downto 22));
                s_u12 <= unsigned(C_FAN(to_integer(s_fcd(11)(1 downto 0)))(21 downto 12));
                s_v12 <= unsigned(C_FAN(to_integer(s_fcd(11)(1 downto 0)))(11 downto 2));
            else
                s_y12 <= s_gy(11); s_u12 <= s_gu(11); s_v12 <= s_gv(11);
            end if;
            -- delays
            for i in 4 to 11 loop s_line(i) <= s_line(i - 1); s_fani(i) <= s_fani(i - 1); end loop;
            for i in 5 to 11 loop s_fcd(i) <= s_fcd(i - 1); end loop;
            for i in 8 to 11 loop s_gy(i) <= s_gy(i - 1); s_gu(i) <= s_gu(i - 1); s_gv(i) <= s_gv(i - 1); end loop;


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

end architecture waterfall;
