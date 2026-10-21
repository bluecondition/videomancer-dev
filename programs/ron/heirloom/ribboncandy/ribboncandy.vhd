-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ribboncandy v0.1.0 -- Heirloom #39 (2026-10-20), from WIN_20230530_22_27.
--
-- The photo: vertical S-curved ribbons (green / blue / magenta / orange)
-- wobbling down the screen over a black-and-white glyph-block field with
-- yellow / cyan / magenta dashes.
--
-- The program: up to 8 ribbons, each a vertical band whose x position
-- wobbles with y (a triangle wave per ribbon, phase-offset), painted as a
-- two-colour gradient across its width; behind them a block field (hashed
-- cells: white / black / grays with CMY dashes) or the source held per
-- block column and posterized.
--
--   K1  Ribbons  1 .. 8 ribbons across
--   K2  Width    ribbon width
--   K3  Pitch    wobble wavelength down the screen
--   K4  Blocks   block size of the background
--   K5  Hue      ribbon palette rotation
--   K6  Drift    the wobble travels, bipolar (centre = still)
--   S7  Ribbons  Gradient / Flat
--   S8  Ground   Blocks (hashed) / Source (held per block, posterized)
--   S9  Dashes   CMY dashes in the block field on / off
--   S10 Mirror   ribbon field mirrored about the centre column
--   S11 Source   Video / Osc
--   P12 WOBBLE   wobble amplitude: straight ribbons .. wild S-curves
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

architecture ribboncandy of program_top is
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

    -- ribbon colour pairs (left / right edge), 8 pairs
    type t_pal is array (0 to 15) of std_logic_vector(31 downto 0);
    constant C_PAL : t_pal := (
        f_col(580, 136, 216), f_col(164, 440, 960),   -- green -> blue
        f_col(164, 440, 960), f_col(426, 887, 808),   -- blue -> magenta
        f_col(426, 887, 808), f_col(584, 772, 212),   -- magenta -> orange
        f_col(584, 772, 212), f_col(840, 584, 64),    -- orange -> yellow
        f_col(678, 64, 663), f_col(580, 136, 216),    -- cyan -> green
        f_col(328, 960, 360), f_col(426, 887, 808),   -- red -> magenta
        f_col(349, 756, 852), f_col(678, 64, 663),    -- violet -> cyan
        f_col(840, 584, 64), f_col(328, 960, 360));   -- yellow -> red
    type t_gpal is array (0 to 7) of std_logic_vector(31 downto 0);
    constant C_GPAL : t_gpal := (
        f_col(940, 512, 512), f_col(64, 512, 512), f_col(480, 512, 512), f_col(700, 512, 512),
        f_col(840, 584, 64), f_col(678, 64, 663), f_col(426, 887, 808), f_col(64, 512, 512));
    constant C_GPAL_ND : t_gpal := (
        f_col(940, 512, 512), f_col(64, 512, 512), f_col(480, 512, 512), f_col(700, 512, 512),
        f_col(260, 512, 512), f_col(940, 512, 512), f_col(700, 512, 512), f_col(64, 512, 512));
    type t_s14 is array (natural range <>) of signed(13 downto 0);
    -- ribbon pitch vw / N for vw = 1920 (HD, 720p) and 1440 (SD)
    type t_cw is array (0 to 15) of integer;
    constant C_CW : t_cw := (1920, 960, 640, 480, 384, 320, 274, 240, 1440, 720, 480, 360, 288, 240, 205, 180);
    -- ribbon full width ladder and 65536 / width
    type t_w is array (0 to 7) of integer;
    constant C_W2 : t_w := (16, 24, 32, 48, 64, 96, 128, 192);
    constant C_RCP : t_w := (4096, 2731, 2048, 1365, 1024, 683, 512, 341);

    signal s_j1 : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_n : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_wk : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_fy : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_k4 : unsigned(9 downto 0) := (others => '0');
    signal s_hue : unsigned(2 downto 0) := (others => '0');
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_grad, s_gsrc, s_dash, s_mir : std_logic := '0';
    -- derived
    signal s_amp : unsigned(9 downto 0) := (others => '0');          -- wobble amplitude
    signal s_w2 : unsigned(8 downto 0) := to_unsigned(128, 9);       -- ribbon full width (virtual)
    signal s_rcp : unsigned(12 downto 0) := to_unsigned(1024, 13);   -- 65536 / w
    signal s_cwv : unsigned(11 downto 0) := to_unsigned(480, 12);    -- vw / N
    signal s_cwh : unsigned(11 downto 0) := to_unsigned(240, 12);
    signal s_bw : unsigned(8 downto 0) := to_unsigned(48, 9);        -- block size virtual
    signal s_bw2 : unsigned(9 downto 0) := to_unsigned(96, 10);
    signal s_bwq : unsigned(16 downto 0) := (others => '0');
    signal s_ph : unsigned(11 downto 0) := (others => '0');
    signal s_seed : unsigned(7 downto 0) := x"33";
    signal s_mult_tri : unsigned(16 downto 0) := (others => '0');
    -- per line: ribbon left edges (virtual px, signed) and the block row
    signal s_py : unsigned(11 downto 0) := (others => '0');
    signal s_ri : unsigned(2 downto 0) := (others => '0');            -- per-line ribbon sequencer (3 stages)
    signal s_rrun, s_vA, s_vB : std_logic := '0';
    signal s_rbase, s_bA, s_bB : unsigned(12 downto 0) := (others => '0');
    signal s_trA : unsigned(6 downto 0) := (others => '0');
    signal s_iA, s_iB : unsigned(2 downto 0) := (others => '0');
    signal s_mtB : unsigned(16 downto 0) := (others => '0');
    signal s_rl_n, s_rl : t_s14(0 to 7) := (others => (others => '0'));
    signal s_cv_n : unsigned(18 downto 0) := (others => '0');
    signal s_cyi_n, s_cyi : unsigned(6 downto 0) := (others => '0');
    -- beam
    signal s_bxa : unsigned(9 downto 0) := (others => '0');
    signal s_bxi : unsigned(6 downto 0) := (others => '0');
    signal s_bnew : std_logic := '0';
    -- pixel
    signal s_x1 : unsigned(12 downto 0) := (others => '0');
    signal s_h1, s_h2, s_h3, s_h4 : unsigned(7 downto 0) := (others => '0');
    signal s_pos2 : t_s14(0 to 7) := (others => (others => '0'));
    signal s_hit3 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_pos3 : t_s14(0 to 7) := (others => (others => '0'));
    signal s_sel4 : unsigned(2 downto 0) := (others => '0');
    signal s_any4 : std_logic := '0';
    signal s_pos4 : unsigned(9 downto 0) := (others => '0');
    signal s_gp5 : unsigned(22 downto 0) := (others => '0');
    signal s_sel5, s_sel6 : unsigned(2 downto 0) := (others => '0');
    signal s_any : t_sl(5 to 11) := (others => '0');
    signal s_t6 : unsigned(7 downto 0) := (others => '0');
    signal s_pa_ra, s_pb_ra : unsigned(3 downto 0) := (others => '0');
    signal s_pa_q, s_pb_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_td : t_u10(7 to 9) := (others => (others => '0'));
    signal s_ay8, s_au8, s_av8, s_by8, s_bu8, s_bv8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_dy9, s_du9, s_dv9 : signed(11 downto 0) := (others => '0');
    signal s_ay9, s_au9, s_av9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_my10, s_mu10, s_mv10 : signed(20 downto 0) := (others => '0');
    signal s_ay10, s_au10, s_av10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ry11, s_ru11, s_rv11, s_rzy, s_rzu, s_rzv : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- ground
    signal s_g_ra : unsigned(2 downto 0) := (others => '0');
    signal s_g_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_hy, s_hu, s_hv : unsigned(9 downto 0) := to_unsigned(64, 10);   -- held source colour
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
            s_pa_q <= C_PAL(to_integer(s_pa_ra));
            s_pb_q <= C_PAL(to_integer(s_pb_ra));
            if s_dash = '1' then s_g_q <= C_GPAL(to_integer(s_g_ra)); else s_g_q <= C_GPAL_ND(to_integer(s_g_ra)); end if;
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
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(6 downto 0) >= 6 or v_k(9 downto 7) = 0) and
                                (v_k(6 downto 0) <= 121 or v_k(9 downto 7) = 7)) then
                s_j1 <= v_k(9 downto 7);
            end if;
            s_wk <= unsigned(registers_in(1)(9 downto 2));
            s_fy <= resize(unsigned(registers_in(2)(9 downto 4)), 8) + 2;
            s_k4 <= unsigned(registers_in(3));
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
                s_n <= s_j1;
                s_grad <= registers_in(6)(0);
                s_gsrc <= registers_in(6)(1);
                s_dash <= registers_in(6)(2);
                s_mir  <= registers_in(6)(3);
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
            s_amp <= s_g10;
            s_w2 <= to_unsigned(C_W2(to_integer(s_wk(7 downto 5))), 9);
            s_rcp <= to_unsigned(C_RCP(to_integer(s_wk(7 downto 5))), 13);
            if s_xs = 4 then s_cwv <= to_unsigned(C_CW(8 + to_integer(s_n)), 12); else s_cwv <= to_unsigned(C_CW(to_integer(s_n)), 12); end if;
            s_bw <= resize(s_k4(9 downto 2), 9) + 12;
            s_bw2 <= s_bw & '0';
            s_bwq <= s_bw & x"00";
            s_cwh <= '0' & s_cwv(11 downto 1);


            -- serial multiplier: per-frame jobs in the zone, per-line jobs at
            -- the source avid fall (+2 and +18 clocks)
            if false then
                null;
            elsif s_mrun = '1' then
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
        variable v_e : signed(14 downto 0);

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
                s_py <= (others => '0');

            elsif s_fall1 = '1' then
                v_vy := s_vy_n + s_ys;
                s_vy_n <= v_vy;
                v_dy := signed(resize(v_vy(19 downto 8), 13)) - 540;
                s_dy_n <= v_dy(11 downto 0);
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11); else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
                v_c := resize(s_cv_n, 20) + s_ys;
                if v_c >= s_bwq then s_cv_n <= resize(v_c - s_bwq, 19); s_cyi_n <= s_cyi_n + 1;
                else s_cv_n <= v_c(18 downto 0); end if;
                s_py <= s_py + s_fy;

            end if;
            -- ribbon left edges for the coming line, one ribbon per clock after
            -- the fall, in three stages: A triangle, B amp product, C sum + write
            -- left_i = base_i + amp * tri(py + i * 32) / 64 - amp - w/2
            s_vA <= '0';
            if s_fall2 = '1' then
                s_ri <= (others => '0'); s_rrun <= '1';
                s_rbase <= resize(s_cwh, 13);
            elsif s_rrun = '1' then
                s_trA <= f_tri8(s_py(11 downto 4) + shift_left(resize(s_ri, 8), 5) + s_ph(7 downto 0));
                s_iA <= s_ri; s_bA <= s_rbase; s_vA <= '1';
                s_rbase <= s_rbase + resize(s_cwv, 13);
                if s_ri = 7 then s_rrun <= '0'; else s_ri <= s_ri + 1; end if;
            end if;
            s_mtB <= s_amp * s_trA; s_iB <= s_iA; s_bB <= s_bA; s_vB <= s_vA;
            if s_vB = '1' then
                v_e := signed(resize(s_bB, 15)) + signed(resize(s_mtB(16 downto 6), 15)) - signed(resize(s_amp, 15))
                       - signed(resize(s_w2(8 downto 1), 15));
                s_rl_n(to_integer(s_iB)) <= v_e(13 downto 0);
            end if;


            if data_in.avid = '1' and s_avq = '0' then
                s_vy <= s_vy_n(18 downto 8);
                s_dy <= s_dy_n;
                s_ady <= s_ady_n;
                s_rl <= s_rl_n;
                s_cyi <= s_cyi_n;

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
                s_bxa <= (others => '0'); s_bxi <= (others => '0'); s_bnew <= '1';

            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_vx <= s_vx + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 14));
                v_n := resize(s_bxa, 11) + s_xs;
                if v_n >= resize(s_bw2, 11) then s_bxa <= resize(v_n - resize(s_bw2, 11), 10); s_bxi <= s_bxi + 1; s_bnew <= '1';
                else s_bxa <= v_n(9 downto 0); s_bnew <= '0'; end if;

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
        variable v_x : signed(14 downto 0);
        variable v_p : signed(13 downto 0);
        variable v_g : unsigned(17 downto 0);

    begin
        if rising_edge(clk) then
            s_lb_ra <= s_rdp;
            s_y3 <= unsigned(s_lb_q(31 downto 22));
            s_u3 <= unsigned(s_lb_q(21 downto 12));
            s_v3 <= unsigned(s_lb_q(11 downto 2));

            -- 1: ribbon-field x (mirrored = cx - |x - cx|), block hash seed
            if s_dxh < 0 then v_x := signed(resize(s_vw, 15)) + resize(s_dxh, 15); else v_x := signed(resize(s_vw, 15)) - resize(s_dxh, 15); end if;
            if s_mir = '1' then
                s_x1 <= unsigned(v_x(13 downto 1));
            else
                s_x1 <= '0' & s_vx(12 downto 1);
            end if;
            v_h := shift_left(resize(s_bxi, 14), 6) - shift_left(resize(s_bxi, 14), 1) - resize(s_bxi, 14)
                   + shift_left(resize(s_cyi, 14), 7) - shift_left(resize(s_cyi, 14), 4) + resize(s_cyi, 14)
                   + resize(s_seed, 14);
            s_h1 <= v_h(7 downto 0);
            -- held source colour per block column
            if s_bnew = '1' then s_hy <= s_y3; s_hu <= s_u3; s_hv <= s_v3; end if;
            -- 2: positions inside each ribbon
            for i in 0 to 7 loop
                s_pos2(i) <= signed(resize(s_x1, 14)) - s_rl(i);
            end loop;
            s_h2 <= s_h1 xor shift_right(s_h1, 3);
            -- 3: hit flags (0 <= pos < w, ribbon index < N)
            for i in 0 to 7 loop
                if s_pos2(i) >= 0 and s_pos2(i) < signed(resize(s_w2, 14)) and i <= to_integer(s_n) then s_hit3(i) <= '1'; else s_hit3(i) <= '0'; end if;
                s_pos3(i) <= s_pos2(i);
            end loop;
            s_h3 <= s_h2 + shift_left(s_h2, 2) + 7;
            -- 4: priority select
            s_any4 <= '0'; s_sel4 <= (others => '0'); s_pos4 <= (others => '0');
            for i in 7 downto 0 loop
                if s_hit3(i) = '1' then s_any4 <= '1'; s_sel4 <= to_unsigned(i, 3); s_pos4 <= unsigned(s_pos3(i)(9 downto 0)); end if;
            end loop;
            s_h4 <= s_h3 xor shift_right(s_h3, 2);
            -- 5: gradient position = pos * rcp (rcp ~ 65536 / w): 0 .. 255; palette addresses; ground address
            s_gp5 <= s_pos4 * s_rcp;
            s_sel5 <= s_sel4; s_any(5) <= s_any4;
            s_pa_ra <= (s_sel4 + s_hue) & '0';
            s_pb_ra <= (s_sel4 + s_hue) & '1';
            s_g_ra <= s_h4(7 downto 5);
            -- 6: ROM registers (pa, pb, g); gradient 8 bits
            if s_gp5(22 downto 16) /= 0 then s_t6 <= (others => '1'); else s_t6 <= s_gp5(15 downto 8); end if;
            s_sel6 <= s_sel5;
            -- 7: fabric copies
            s_ay8 <= unsigned(s_pa_q(31 downto 22)); s_au8 <= unsigned(s_pa_q(21 downto 12)); s_av8 <= unsigned(s_pa_q(11 downto 2));
            s_by8 <= unsigned(s_pb_q(31 downto 22)); s_bu8 <= unsigned(s_pb_q(21 downto 12)); s_bv8 <= unsigned(s_pb_q(11 downto 2));
            s_td(7) <= resize(s_t6, 10);
            if s_gsrc = '1' then
                s_gy(7) <= s_hy(9 downto 7) & "0000000"; s_gu(7) <= s_hu(9 downto 8) & "00000000"; s_gv(7) <= s_hv(9 downto 8) & "00000000";
            else
                s_gy(7) <= unsigned(s_g_q(31 downto 22)); s_gu(7) <= unsigned(s_g_q(21 downto 12)); s_gv(7) <= unsigned(s_g_q(11 downto 2));
            end if;
            -- 8: differences b - a
            s_dy9 <= signed(resize(s_by8, 12)) - signed(resize(s_ay8, 12));
            s_du9 <= signed(resize(s_bu8, 12)) - signed(resize(s_au8, 12));
            s_dv9 <= signed(resize(s_bv8, 12)) - signed(resize(s_av8, 12));
            s_ay9 <= s_ay8; s_au9 <= s_au8; s_av9 <= s_av8;
            -- 9: products with the gradient (flat = 0)
            if s_grad = '1' then
                s_my10 <= s_dy9 * signed(resize(s_td(8), 9));
                s_mu10 <= s_du9 * signed(resize(s_td(8), 9));
                s_mv10 <= s_dv9 * signed(resize(s_td(8), 9));
            else
                s_my10 <= (others => '0'); s_mu10 <= (others => '0'); s_mv10 <= (others => '0');
            end if;
            s_ay10 <= s_ay9; s_au10 <= s_au9; s_av10 <= s_av9;
            -- 10: ribbon colour
            s_ry11 <= f_clamp(signed(resize(s_ay10, 13)) + resize(shift_right(s_my10, 8), 13));
            s_ru11 <= f_clamp(signed(resize(s_au10, 13)) + resize(shift_right(s_mu10, 8), 13));
            s_rv11 <= f_clamp(signed(resize(s_av10, 13)) + resize(shift_right(s_mv10, 8), 13));
            -- 11: copy; 12: compose
            s_rzy <= s_ry11; s_rzu <= s_ru11; s_rzv <= s_rv11;
            if s_any(11) = '1' then
                s_y12 <= s_rzy; s_u12 <= s_rzu; s_v12 <= s_rzv;
            else
                s_y12 <= s_gy(11); s_u12 <= s_gu(11); s_v12 <= s_gv(11);
            end if;
            -- delays
            for i in 6 to 11 loop s_any(i) <= s_any(i - 1); end loop;
            for i in 8 to 9 loop s_td(i) <= s_td(i - 1); end loop;
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

end architecture ribboncandy;
