-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Sinew v0.5.0 -- Heirloom #18, rebuilt from the photo WIN_20230315_00_53_56.
--
-- The photo: three horizontal bands.  Top: yellow/orange/green vertical bars.  Bottom: pink/navy/teal
-- bars with U-shaped teal contours.  Middle: seven orange "ropes" S-curving down the frame, crossed by
-- thin blue/pink "sinew" contour lines that bulge into ")" arcs at mid-height and fan into green and
-- black shingles above and below, dense "<" chevrons at the bottom, torn strips where the fan is
-- tightest.  The channel split shows the patch: two line-locked horizontal oscillators -- the ROPE
-- oscillator (R's clipped trapezoid, period ~272 px, its phase and rate set per line) and the SINEW
-- oscillator (B's thin pulses), whose period is set per line by a vertical TRIANGLE (280 px at the bulge,
-- ~120 px at the vertices), phase-modulated by the rope oscillator -- into a colourizer.
--
-- The program: per line, an engine (one serial multiplier, one serial divider) computes the rope
-- oscillator's two increments (rise / fall halves) and start phase from fitted cubics, the sinew
-- oscillator's increment 1/P and start phase from the triangle law, the slice blend and the blue
-- ramp, the line tear.  Per pixel: two DDAs, the wiggle (cosine ROM), the video/Osc melt, then the
-- colour tables fitted to the photo (sinew_pkg, by sinew.py): R(theta, Omega), G = three slices along
-- the triangle blended per line, B(theta, Omega) + ramp; top/bottom band curves; grain (the ropes'
-- marbling); palette routing; RGB -> YUV.
--
--   K1  Ropes    rope count 0.5x .. 2x (the photo: 7 across), all three bands
--   K2  Sinews   sinew line density 0.5x .. 2x
--   K3  Twist    the ropes' S-curve 0 .. 4x (centre = the photo)
--   K4  Speed    bipolar (centre +-8 = still): the pinch rolls down, the sinews stream, the ropes drift
--   K5  Melt     how far the video (or Osc) bends the sinews and ropes
--   K6  Palette  8 channel routings (0 = the photo)
--   S7  Tear     torn strips at the pinch vertices + slips (default On)
--   S8  Grain    the ropes' marbling (default On)
--   S9  Wave     Triangle (the photo) / Saw vertical modulator
--   S10 Bands    the top and bottom bands (default On)
--   S11 Source   Video / Osc
--   P12 PINCH    the vertical modulator's depth: 0 = straight evenly spaced sinews, 50 % = the photo,
--                100 % = dense chevrons and knots
--
-- Virtual 1080-line geometry (x stretched to the width at SD).  Generated avid; output conditioning
-- (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777) before the blanking gate.  Latency 24
-- clocks in every mode.  Generated colours in the hardware convention (U carries Cr, V carries Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.sinew_pkg.all;
use work.all;

architecture sinew of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

    -- timing-ID constants
    function f_sx(t : std_logic_vector(3 downto 0)) return unsigned is   -- virtual px per pixel, Q12
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(10923, 14);
            when 5 | 6 | 14     => return to_unsigned(6144, 14);
            when others         => return to_unsigned(4096, 14);
        end case;
    end function;
    function f_kb(t : std_logic_vector(3 downto 0)) return unsigned is   -- 408.3 / sx, Q8
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(39197, 17);
            when 5 | 6 | 14     => return to_unsigned(69683, 17);
            when others         => return to_unsigned(104525, 17);
        end case;
    end function;
    function f_sy(t : std_logic_vector(3 downto 0)) return unsigned is   -- virtual lines per line, Q12
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(18204, 15);
            when 4          => return to_unsigned(9216, 15);
            when 8          => return to_unsigned(15360, 15);
            when 12         => return to_unsigned(7680, 15);
            when 5 | 6 | 14 => return to_unsigned(6144, 15);
            when 1 | 2 | 10 => return to_unsigned(8192, 15);
            when others     => return to_unsigned(4096, 15);
        end case;
    end function;
    function f_kax(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(356, 9);
            when 5 | 6 | 14     => return to_unsigned(268, 9);
            when others         => return to_unsigned(176, 9);
        end case;
    end function;
    function f_kay(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1080, 11);
            when 4          => return to_unsigned(548, 11);
            when 8          => return to_unsigned(912, 11);
            when 12         => return to_unsigned(456, 11);
            when 5 | 6 | 14 => return to_unsigned(364, 11);
            when 1 | 2 | 10 => return to_unsigned(484, 11);
            when others     => return to_unsigned(244, 11);
        end case;
    end function;
    -- 16-bit phase -> triangle -127..127
    function f_tri(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;
    -- 6-bit table value -> 8 bits
    function f_ex6(v : std_logic_vector(5 downto 0)) return unsigned is
    begin
        return unsigned(v) & unsigned(v(5 downto 4));
    end function;
    function f_xs(s : unsigned(31 downto 0)) return unsigned is   -- xorshift32 step
        variable v : unsigned(31 downto 0);
    begin
        v := s xor shift_left(s, 13);
        v := v xor shift_right(v, 17);
        v := v xor shift_left(v, 5);
        return v;
    end function;

    -- Horner coefficients (highest power first): band 0 mid, 1 top, 2 bottom; poly 0 inc_hi, 1 inc_lo
    -- (Q28 cycles per virtual px), 2 th0 (Q20 cycles); u = (vy - 540)/360 (mid), (vy - 902)/176 (bottom)
    function f_coef(band, hp, k : unsigned(1 downto 0)) return signed is
        variable v : integer;
    begin
        v := 0;
        case to_integer(band) is
            when 1 =>
                if k = 3 then
                    case to_integer(hp) is
                        when 0 => v := 807702; when 1 => v := 1019678; when others => v := 317579;
                    end case;
                end if;
            when 2 =>
                case to_integer(hp) is
                    when 0 => case to_integer(k) is when 1 => v := 2325; when 2 => v := 47458; when 3 => v := 1041502; when others => v := 0; end case;
                    when 1 => case to_integer(k) is when 1 => v := 2234; when 2 => v := 45597; when 3 => v := 1000659; when others => v := 0; end case;
                    when others => case to_integer(k) is when 2 => v := -96298; when 3 => v := 427990; when others => v := 0; end case;
                end case;
            when others =>
                case to_integer(hp) is
                    when 0 => case to_integer(k) is when 0 => v := 13959; when 1 => v := 87107; when 2 => v := 114917; when others => v := 965750; end case;
                    when 1 => case to_integer(k) is when 0 => v := 4268; when 1 => v := 35997; when 2 => v := -5208; when others => v := 1009586; end case;
                    when others => case to_integer(k) is when 0 => v := -182800; when 1 => v := 10728; when 2 => v := 175157; when others => v := 299018; end case;
                end case;
        end case;
        return to_signed(v, 24);
    end function;

    -- timing
    signal t_sx  : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal t_kb  : unsigned(16 downto 0) := to_unsigned(104525, 17);
    signal t_sy  : unsigned(14 downto 0) := to_unsigned(4096, 15);
    signal t_kax : unsigned(8 downto 0) := (others => '0');
    signal t_kay : unsigned(10 downto 0) := (others => '0');

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K1, K2, K3, K5, P12 (Q.3)
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(420 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- field values
    signal f_x1, f_x2, f_x3, f_x5, f_p : unsigned(9 downto 0) := (others => '0');
    signal f_v     : signed(9 downto 0) := (others => '0');       -- speed, deadbanded
    signal f_trph  : unsigned(13 downto 0) := (others => '0');    -- triangle phase, Q4 lines, mod 11008
    signal f_psph  : unsigned(21 downto 0) := (others => '0');    -- sinew stream, Q20
    signal f_roph  : unsigned(23 downto 0) := (others => '0');    -- rope drift, Q20
    signal f_ta, f_tb : unsigned(15 downto 0) := (others => '0');
    signal f_pal   : unsigned(2 downto 0) := (others => '0');
    signal f_tear, f_grain, f_saw, f_bands, f_osc : std_logic := '0';
    signal f_s1    : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal f_rsc   : unsigned(14 downto 0) := to_unsigned(4096, 15);   -- Q12
    signal f_ta2   : unsigned(13 downto 0) := (others => '0');
    signal f_x2sq  : unsigned(19 downto 0) := (others => '0');
    signal f_p0m   : unsigned(13 downto 0) := to_unsigned(4489, 14);   -- Q4 px
    signal f_tw    : unsigned(9 downto 0) := to_unsigned(256, 10);     -- Q8
    signal f_kk    : unsigned(11 downto 0) := to_unsigned(2169, 12);   -- Q12
    signal f_m5    : unsigned(8 downto 0) := (others => '0');          -- Q8

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- engine: sequencer, serial multiplier, serial divider
    signal e_st   : integer range 0 to 127 := 0;
    signal e_kf, e_k0, e_kick : std_logic := '0';
    signal e_V    : unsigned(23 downto 0) := (others => '0');   -- Q12 virtual lines
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(17 downto 0) := (others => '0');
    signal m_h    : unsigned(23 downto 0) := (others => '0');
    signal m_l    : unsigned(17 downto 0) := (others => '0');
    signal m_neg  : std_logic := '0';
    signal m_pre  : std_logic := '0';
    signal m_sp   : signed(42 downto 0) := (others => '0');
    signal m_sa   : signed(24 downto 0) := (others => '0');
    signal m_sb   : signed(18 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal m_busy : std_logic := '0';
    signal d_rem  : unsigned(14 downto 0) := (others => '0');
    signal d_num  : unsigned(16 downto 0) := (others => '0');
    signal d_q    : unsigned(16 downto 0) := (others => '0');
    signal d_den  : unsigned(13 downto 0) := (others => '0');
    signal d_cnt  : unsigned(4 downto 0) := (others => '0');
    signal d_busy : std_logic := '0';
    -- engine registers
    signal e_vy   : unsigned(15 downto 0) := (others => '0');   -- Q4 virtual lines
    signal e_band : unsigned(1 downto 0) := (others => '0');    -- 0 mid, 1 top, 2 bottom
    signal e_uq   : signed(13 downto 0) := (others => '0');     -- Q12
    signal e_acc  : signed(23 downto 0) := (others => '0');
    signal e_cf   : signed(23 downto 0) := (others => '0');
    signal e_hp   : unsigned(1 downto 0) := (others => '0');    -- Horner polynomial 0 IH, 1 IL, 2 TH
    signal e_hk   : unsigned(1 downto 0) := (others => '0');    -- Horner step
    signal e_ihv, e_ilv : unsigned(21 downto 0) := (others => '0');
    signal e_thv  : signed(23 downto 0) := (others => '0');
    signal e_ihp, e_ilp : unsigned(15 downto 0) := (others => '0');
    signal e_q    : signed(16 downto 0) := (others => '0');
    signal e_dvp  : unsigned(13 downto 0) := (others => '0');
    signal e_dv   : signed(14 downto 0) := (others => '0');
    signal e_tr   : unsigned(13 downto 0) := (others => '0');
    signal e_s    : unsigned(13 downto 0) := (others => '0');
    signal e_P    : unsigned(13 downto 0) := to_unsigned(4489, 14);
    signal e_a    : unsigned(16 downto 0) := (others => '0');
    signal e_b    : signed(23 downto 0) := (others => '0');
    signal e_pos  : unsigned(5 downto 0) := (others => '0');
    signal e_px   : signed(16 downto 0) := (others => '0');
    signal e_vb   : signed(8 downto 0) := (others => '0');
    signal e_off  : signed(7 downto 0) := (others => '0');
    signal e_yq   : unsigned(7 downto 0) := (others => '0');
    signal e_xt   : unsigned(31 downto 0) := x"01234567";
    signal e_xg   : unsigned(31 downto 0) := x"089ABCDE";
    signal e_hold : unsigned(1 downto 0) := (others => '0');
    signal e_cur  : signed(7 downto 0) := (others => '0');
    signal e_slen : unsigned(5 downto 0) := (others => '0');
    signal e_slip : signed(7 downto 0) := (others => '0');
    type t_walk is array(0 to 3) of unsigned(7 downto 0);
    signal e_walk : t_walk := (x"00", x"25", x"4A", x"6F");
    -- next-line results and the current line's values
    signal n_ihp, n_ilp, c_ihp, c_ilp : unsigned(15 downto 0) := (others => '0');
    signal n_th0, c_th0 : unsigned(23 downto 0) := (others => '0');
    signal n_a, c_a     : unsigned(16 downto 0) := (others => '0');
    signal n_b, c_b     : unsigned(21 downto 0) := (others => '0');
    signal n_band, c_band : unsigned(1 downto 0) := (others => '0');
    signal n_i0, c_i0   : std_logic := '0';
    signal n_w, c_w     : unsigned(4 downto 0) := (others => '0');
    signal n_vb, c_vb   : signed(8 downto 0) := (others => '0');
    signal n_yi, c_yi, n_yf, c_yf : unsigned(3 downto 0) := (others => '0');

    -- beam
    signal b_th : unsigned(23 downto 0) := (others => '0');
    signal b_ps : unsigned(21 downto 0) := (others => '0');
    signal b_pa : unsigned(15 downto 0) := (others => '0');
    signal l_pb : unsigned(15 downto 0) := (others => '0');

    -- ROM ports
    signal r_cosa : unsigned(7 downto 0) := (others => '0');
    signal r_cosq, r_cosq2 : std_logic_vector(15 downto 0) := (others => '0');
    signal r_la : unsigned(10 downto 0) := (others => '0');
    signal r_lrq, r_lg0q, r_lg1q, r_lg2q, r_lbq : std_logic_vector(5 downto 0) := (others => '0');
    signal r_lrq2, r_lg0q2, r_lg1q2, r_lg2q2, r_lbq2 : std_logic_vector(5 downto 0) := (others => '0');
    signal r_ca : unsigned(7 downto 0) := (others => '0');
    signal r_rcq, r_xcq, r_rcq2, r_xcq2 : std_logic_vector(7 downto 0) := (others => '0');
    signal r_ga : unsigned(9 downto 0) := (others => '0');
    signal r_da : unsigned(7 downto 0) := (others => '0');
    signal r_dq, r_dq2 : std_logic_vector(15 downto 0) := (others => '0');
    signal r_gbq, r_gbq2 : std_logic_vector(11 downto 0) := (others => '0');

    -- pixel path
    signal p1_th, p2_th, p3_th, p4_th : unsigned(23 downto 0) := (others => '0');
    signal p1_ps, p2_ps, p3_ps, p4_ps : unsigned(21 downto 0) := (others => '0');
    signal p1_l  : signed(7 downto 0) := (others => '0');
    signal p2_pm, p3_pm : signed(12 downto 0) := (others => '0');
    signal p5_t8, p6_t8 : unsigned(7 downto 0) := (others => '0');
    signal p5_o8, p6_o8 : unsigned(7 downto 0) := (others => '0');
    signal p5_ri : unsigned(1 downto 0) := (others => '0');
    signal p5_t6, p6_t6, p7_t6, p8_t6 : unsigned(5 downto 0) := (others => '0');
    signal p6_g, p7_g : unsigned(7 downto 0) := (others => '0');
    signal p8_g  : signed(7 downto 0) := (others => '0');
    signal p9_gg, p10_gg : signed(7 downto 0) := (others => '0');
    signal p8_r, p8_ga, p8_b, p8_g0, p8_rc, p8_xc : unsigned(7 downto 0) := (others => '0');
    signal p8_dg, p8_dgb : signed(8 downto 0) := (others => '0');
    signal p9_r, p9_ga, p9_g0, p9_rc, p9_xc : unsigned(7 downto 0) := (others => '0');
    signal p9_pg : signed(13 downto 0) := (others => '0');
    signal p9_pb : signed(12 downto 0) := (others => '0');
    signal p9_b  : signed(9 downto 0) := (others => '0');
    signal p10_r, p10_gm, p10_gb, p10_b, p10_rc, p10_xc : unsigned(7 downto 0) := (others => '0');
    signal p11_r, p11_g, p11_b : unsigned(7 downto 0) := (others => '0');
    signal p12_r, p12_g, p12_b : unsigned(7 downto 0) := (others => '0');
    signal p13_yrg, p13_ugb, p13_vrg : unsigned(16 downto 0) := (others => '0');
    signal p13_yb, p13_ur, p13_vb : unsigned(15 downto 0) := (others => '0');
    signal p14_y : unsigned(16 downto 0) := (others => '0');
    signal p14_u, p14_v : signed(17 downto 0) := (others => '0');
    signal p15_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p15_u, p15_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p16_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y17, s_y18, s_y19, s_y20, s_y21, s_y22x, s_yf22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered reads; a second fabric register in the pixel path)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_cosq <= C_COS(to_integer(r_cosa));
            r_lrq  <= C_LR(to_integer(r_la));
            r_lg0q <= C_LG0(to_integer(r_la));
            r_lg1q <= C_LG1(to_integer(r_la));
            r_lg2q <= C_LG2(to_integer(r_la));
            r_lbq  <= C_LB(to_integer(r_la));
            r_rcq  <= C_RC(to_integer(r_ca));
            r_xcq  <= C_XC(to_integer(r_ca));
            r_gbq  <= C_GB(to_integer(r_ga));
            r_dq   <= C_DEL2(to_integer(r_da));
            r_dq2  <= r_dq;
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls: vblank zone, width, glides, field latches and drift phases
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_t  : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            t_sx  <= f_sx(registers_in(8)(3 downto 0));
            t_kb  <= f_kb(registers_in(8)(3 downto 0));
            t_sy  <= f_sy(registers_in(8)(3 downto 0));
            t_kax <= f_kax(registers_in(8)(3 downto 0));
            t_kay <= f_kay(registers_in(8)(3 downto 0));

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
            elsif s_zc /= 1023 then
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
            g_wm1 <= s_w - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K4 Speed: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 8 then f_v <= resize(v_x - 8, 10);
                elsif v_x < -8 then f_v <= resize(v_x + 8, 10);
                else f_v <= (others => '0'); end if;
                f_pal   <= unsigned(registers_in(5)(9 downto 7));
                f_tear  <= registers_in(6)(0);
                f_grain <= registers_in(6)(1);
                f_saw   <= registers_in(6)(2);
                f_bands <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- drift: the pinch rolls (triangle phase mod 688 lines), the sinews stream, the ropes drift
                v_t := resize(f_trph, 15) + unsigned(resize(shift_right(f_v, 4), 15));
                if f_v < 0 then
                    if v_t(14) = '1' or v_t >= 11008 then v_t := v_t + 11008; end if;
                else
                    if v_t >= 11008 then v_t := v_t - 11008; end if;
                end if;
                f_trph <= v_t(13 downto 0);
                f_psph <= f_psph + unsigned(resize(shift_left(resize(f_v, 22), 4), 22));
                f_roph <= f_roph + unsigned(resize(f_v, 24)) + unsigned(resize(shift_left(resize(f_v, 24), 1), 24));
                f_ta <= f_ta + 151; f_tb <= f_tb - 113;
            end if;

            -- glides: ease 1/8 of the gap per field (deadband 4); one channel per four zone clocks (4..23)
            if s_zone = '1' and s_zc >= 4 and s_zc < 24 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when others => s_g(4) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;
            f_x1 <= s_g(0)(12 downto 3); f_x2 <= s_g(1)(12 downto 3); f_x3 <= s_g(2)(12 downto 3);
            f_x5 <= s_g(3)(12 downto 3); f_p <= s_g(4)(12 downto 3);

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n; s_field_sr(0) <= data_in.field_n;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1); s_field_sr(i) <= s_field_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 2/3 clocks after the source's avid rise, field-latched width)
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
    -- Engine.  Field program (states 1..18, vblank zone clock 64) and line program (states 32..95,
    -- kicked at the source avid rise of line n, computing line n+1; line 0 at zone clock 600).
    -- One serial multiplier (24 x 18, sign-magnitude, 18 clocks) and one serial divider (17 clocks);
    -- the sequencer waits while either is busy.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_p    : unsigned(41 downto 0);
        variable v_r    : signed(31 downto 0);
        variable v_s    : unsigned(24 downto 0);
        variable v_t    : unsigned(15 downto 0);
        variable v_go   : boolean;
        variable v_c    : signed(23 downto 0);
        variable v_n    : signed(23 downto 0);
        variable v_ua   : signed(16 downto 0);
        variable v_q    : signed(17 downto 0);
        variable v_tr   : unsigned(13 downto 0);
        variable v_dd   : signed(10 downto 0);
        variable v_x    : signed(16 downto 0);
        variable v_o    : signed(8 downto 0);
        variable v_z    : unsigned(31 downto 0);
        variable v_st   : unsigned(2 downto 0);
        variable v_wk   : unsigned(7 downto 0);
        variable v_sp   : signed(42 downto 0);
        variable v_sa   : signed(24 downto 0);
        variable v_sb   : signed(18 downto 0);
        -- multiply issue: signed operands; the multiplier takes |a|, |b| in its pre-cycle and the
        -- signed product is formed once (v_sp) for every state
        procedure muls(a : signed; b : signed) is
        begin
            m_sa <= resize(a, 25); m_sb <= resize(b, 19); m_pre <= '1'; m_busy <= '1';
        end procedure;
        procedure mulu(a : unsigned; b : unsigned) is
        begin
            m_sa <= signed(resize(a, 25)); m_sb <= signed(resize(b, 19)); m_pre <= '1'; m_busy <= '1';
        end procedure;
    begin
        if rising_edge(clk) then
            -- serial multiplier (pre-cycle: magnitudes and sign)
            if m_pre = '1' then
                m_pre <= '0';
                v_sa := m_sa; if v_sa < 0 then v_sa := -v_sa; end if;
                v_sb := m_sb; if v_sb < 0 then v_sb := -v_sb; end if;
                m_a <= unsigned(v_sa(23 downto 0)); m_b <= unsigned(v_sb(17 downto 0));
                m_neg <= m_sa(24) xor m_sb(18);
                m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= (others => '0');
            elsif m_busy = '1' then
                if m_cnt = 18 then
                    -- finish cycle: the signed product, registered (no negate-then-add chain in the states)
                    v_p := m_h & m_l; v_sp := signed('0' & v_p); if m_neg = '1' then v_sp := -v_sp; end if;
                    m_sp <= v_sp;
                    m_busy <= '0';
                else
                    if m_b(0) = '1' then v_s := resize(m_h, 25) + resize(m_a, 25); else v_s := resize(m_h, 25); end if;
                    m_h <= v_s(24 downto 1);
                    m_l <= v_s(0) & m_l(17 downto 1);
                    m_b <= '0' & m_b(17 downto 1);
                    m_cnt <= m_cnt + 1;
                end if;
            end if;
            -- serial divider (restoring), quotient 17 bits
            if d_busy = '1' then
                v_t := resize(d_rem, 15) & d_num(16);
                if v_t >= resize(d_den, 16) then
                    d_rem <= resize(v_t - resize(d_den, 16), 15);
                    d_q <= d_q(15 downto 0) & '1';
                else
                    d_rem <= v_t(14 downto 0);
                    d_q <= d_q(15 downto 0) & '0';
                end if;
                d_num <= d_num(15 downto 0) & '0';
                if d_cnt = 16 then d_busy <= '0'; end if;
                d_cnt <= d_cnt + 1;
            end if;
            v_p := m_h & m_l;
            v_sp := m_sp;

            v_go := (m_busy = '0') and (d_busy = '0');
            -- kicks, decoded and registered one clock ahead (field program at zone clock 64, line 0 at 600,
            -- then every source avid rise)
            if s_zone = '1' and s_zc = 63 then e_kf <= '1'; else e_kf <= '0'; end if;
            if s_zone = '1' and s_zc = 599 then e_k0 <= '1'; else e_k0 <= '0'; end if;
            if (s_zone = '1' and (s_zc = 63 or s_zc = 599)) or (data_in.avid = '1' and s_avq = '0') then
                e_kick <= '1';
            else
                e_kick <= '0';
            end if;
            if e_kick = '1' then
                if e_kf = '1' then
                    e_st <= 1;
                elsif e_k0 = '1' then
                    e_V <= (others => '0'); e_st <= 32;
                else
                    -- the line that is starting takes the values computed for it
                    c_ihp <= n_ihp; c_ilp <= n_ilp; c_th0 <= n_th0; c_a <= n_a; c_b <= n_b;
                    c_band <= n_band; c_i0 <= n_i0; c_w <= n_w; c_vb <= n_vb; c_yi <= n_yi; c_yf <= n_yf;
                    e_V <= e_V + resize(t_sy, 24);
                    e_st <= 32;
                end if;
            elsif v_go then
                e_st <= e_st + 1;
                case e_st is
                    when 0 => e_st <= 0;   -- idle
                    ---------------- field program ----------------
                    when 1  => mulu(f_x1, f_x1);
                    when 2  => f_s1 <= to_unsigned(2048, 14) + shift_left(resize(f_x1, 14), 1) + resize(v_p(19 downto 8), 14);
                    when 3  => mulu(f_s1, t_sx);
                    when 4  => f_rsc <= v_p(26 downto 12);
                    when 5  => mulu(f_x2, to_unsigned(11222, 14));
                    when 6  => f_ta2 <= v_p(23 downto 10);
                    when 7  => mulu(f_x2, f_x2);
                    when 8  => f_x2sq <= v_p(19 downto 0);
                    when 9  => mulu(f_x2sq, to_unsigned(4489, 13));
                    when 10 => f_p0m <= to_unsigned(8978, 14) - f_ta2 + resize(v_p(32 downto 20), 14);
                    when 11 => mulu(f_x3, f_x3);
                    when 12 => f_tw <= v_p(19 downto 10);
                    when 13 =>
                        if f_p < 512 then mulu(f_p, to_unsigned(4339, 13));
                        else mulu(f_p - 512, to_unsigned(2703, 12)); end if;
                    when 14 =>
                        if f_p < 512 then f_kk <= resize(v_p(21 downto 10), 12);
                        else f_kk <= to_unsigned(2170, 12) + resize(v_p(21 downto 10), 12); end if;
                    when 15 => mulu(resize(f_x5, 12) + shift_left(resize(f_x5, 12), 1), f_x5);
                    when 16 => f_m5 <= resize(v_p(21 downto 13), 9);
                    when 17 => e_st <= 0;

                    ---------------- line program ----------------
                    when 32 =>
                        e_vy <= e_V(23 downto 8);
                        e_xt <= f_xs(e_xt); e_xg <= f_xs(e_xg);
                        if f_bands = '1' and e_V(23 downto 8) < 2400 then e_band <= "01";
                        elsif f_bands = '1' and e_V(23 downto 8) >= 14432 then e_band <= "10";
                        else e_band <= "00"; end if;
                    when 33 =>
                        -- grain walks: +-2 or +-4 per line
                        for k in 0 to 3 loop
                            if e_xg(2 * k + 1) = '1' then v_wk := to_unsigned(4, 8); else v_wk := to_unsigned(2, 8); end if;
                            if e_xg(2 * k) = '1' then e_walk(k) <= e_walk(k) - v_wk; else e_walk(k) <= e_walk(k) + v_wk; end if;
                        end loop;
                        case to_integer(e_band) is
                            when 0 => v_ua := signed(resize(e_vy, 17)) - 8640; muls(v_ua, to_signed(2913, 14));
                            when 2 => v_ua := signed(resize(e_vy, 17)) - 14432; muls(v_ua, to_signed(5958, 14));
                            when others => mulu(to_unsigned(0, 2), to_unsigned(0, 2));
                        end case;
                    when 34 =>
                        e_uq <= v_sp(25 downto 12);
                        e_hp <= "00"; e_hk <= "00";
                    -- Horner: acc = c3; 3 x (acc = acc * u >> 12 + c)
                    when 35 =>
                        e_acc <= f_coef(e_band, e_hp, "00");
                    when 36 =>
                        muls(e_acc, e_uq);
                        e_cf <= f_coef(e_band, e_hp, e_hk + 1);   -- registered during the multiply
                    when 37 =>
                        v_c := v_sp(35 downto 12) + e_cf;
                        e_acc <= v_c;
                        if e_hk = 2 then
                            case to_integer(e_hp) is
                                when 0 => e_ihv <= unsigned(v_c(21 downto 0));
                                when 1 => e_ilv <= unsigned(v_c(21 downto 0));
                                when others => e_thv <= v_c;
                            end case;
                            e_hk <= "00";
                            if e_hp = 2 then e_st <= 38; else e_hp <= e_hp + 1; e_st <= 35; end if;
                        else
                            e_hk <= e_hk + 1; e_st <= 36;
                        end if;
                    -- Twist (middle band): th0 = c0 + tw (poly - c0) / 256
                    when 38 =>
                        v_c := e_thv - 299018;
                        muls(v_c, signed(resize(f_tw, 11)));
                    when 39 =>
                        if e_band = 0 then
                            e_thv <= to_signed(299018, 24) + v_sp(31 downto 8);
                        end if;
                    when 40 => mulu(e_ihv, f_rsc);
                    when 41 => e_ihp <= v_p(35 downto 20);
                    when 42 => mulu(e_ilv, f_rsc);
                    when 43 =>
                        e_ilp <= v_p(35 downto 20);
                        e_q <= signed(resize(e_vy, 17)) - 3088 - signed(resize(f_trph, 17)) + 22016;
                    -- triangle distance
                    when 44 =>
                        if e_q >= 22016 then e_q <= e_q - 22016; end if;
                    when 45 =>
                        if e_q >= 11008 then v_q := resize(e_q, 18) - 11008; else v_q := resize(e_q, 18); end if;
                        e_dvp <= unsigned(v_q(13 downto 0));
                    when 46 =>
                        e_dv <= signed(resize(e_dvp, 15)) - 5504;
                        if f_saw = '1' then e_tr <= '0' & e_dvp(13 downto 1);
                        elsif e_dvp >= 5504 then e_tr <= e_dvp - 5504;
                        else e_tr <= 5504 - e_dvp; end if;
                    when 47 => mulu(e_tr, e_tr);
                    when 48 =>
                        if e_tr < 1024 then e_s <= resize(v_p(26 downto 11), 14); else e_s <= e_tr - 512; end if;
                    when 49 => mulu(f_kk, e_s);
                    when 50 =>
                        v_x := signed(resize(f_p0m, 17)) - signed(resize(v_p(31 downto 12), 17));
                        if v_x < 384 then e_P <= to_unsigned(384, 14); else e_P <= unsigned(v_x(13 downto 0)); end if;
                    when 51 =>
                        -- a = (sx << 12) / P  (Q20 cycles per pixel)
                        d_den <= e_P; d_rem <= resize(t_sx(13 downto 5), 15);
                        d_num <= t_sx(4 downto 0) & "000000000000"; d_q <= (others => '0');
                        d_cnt <= (others => '0'); d_busy <= '1';
                    when 52 => e_a <= d_q;
                    when 53 => mulu(e_a, t_kb);
                    when 54 => e_b <= signed(resize(v_p(32 downto 8), 24)) - 1513095;
                    when 55 => mulu(e_dvp, to_unsigned(48770, 16));
                    when 56 => r_da <= v_p(28 downto 21);          -- delta ROM (256 steps over the triangle)
                    when 57 => null;
                    when 58 => null;
                    when 59 =>
                        -- the psi offset along the triangle, the sinew stream; the blue ramp (ROM bits 15..10, +8)
                        e_b <= e_b + shift_left(resize(signed(r_dq2(9 downto 0)), 24), 11) + signed(resize(f_psph, 24));
                        e_vb <= signed(resize(unsigned(r_dq2(15 downto 10)), 9)) - 8;
                    -- G slice position: (dv + 3520) / 110 ~ x/128 + x/1024 + x/2048 (one operation per state)
                    when 60 => e_px <= resize(e_dv, 17) + 3520;
                    when 61 =>
                        if e_px <= 0 then e_px <= (others => '0');
                        else e_px <= shift_right(e_px, 7) + shift_right(e_px, 10) + shift_right(e_px, 11); end if;
                    when 62 =>
                        if e_px > 63 then e_pos <= (others => '1'); else e_pos <= unsigned(e_px(5 downto 0)); end if;
                        e_st <= 68;
                    when 68 =>
                        v_z := e_xt;
                        if f_tear = '1' and e_band = 0 and (e_dv > 4800 or e_dv < -4800) then
                            if e_hold = 0 then
                                if v_z(8) = '1' then
                                    e_cur <= resize(signed(resize(v_z(5 downto 0), 8)) - 32 + signed(resize(v_z(13 downto 10), 8)) - 8, 8);
                                else
                                    e_cur <= (others => '0');
                                end if;
                                e_hold <= v_z(10 downto 9);    -- hold 1..4 lines (decrement folded below)
                            else
                                e_hold <= e_hold - 1;
                            end if;
                        else
                            e_cur <= (others => '0'); e_hold <= (others => '0');
                        end if;
                        if f_tear = '1' and e_band = 0 and e_slen = 0 and v_z(20 downto 12) = 0 then
                            v_o := signed(resize(v_z(23 downto 20), 9)) + 4;
                            if v_z(24) = '1' then e_slip <= resize(-v_o, 8); else e_slip <= resize(v_o, 8); end if;
                            e_slen <= resize(v_z(29 downto 25), 6) + 7;
                        elsif e_slen > 0 then
                            e_slen <= e_slen - 1;
                        else
                            e_slip <= (others => '0');
                        end if;
                    when 69 =>
                        if f_tear = '1' then e_off <= e_cur + e_slip; else e_off <= (others => '0'); end if;
                    when 70 =>
                        -- the tear shifts both oscillators' phase (1/256 cycle per unit) and adds the rope drift
                        e_thv <= e_thv + shift_left(resize(e_off, 24), 12) + signed(f_roph);
                        e_b <= e_b + shift_left(resize(e_off, 24), 12);
                    -- bottom-band row index: (vy - 902 lines) * 16 / 178 - 8 ~ x/16 + x/32 - x/256 (one op per state)
                    when 71 => e_px <= signed(resize(e_vy, 17)) - 14432;
                    when 72 =>
                        if e_px < 0 then e_px <= (others => '0');
                        else e_px <= shift_right(e_px, 4) + shift_right(e_px, 5) - shift_right(e_px, 8); end if;
                    when 73 => e_px <= e_px - 8;
                    when 74 =>
                        if e_px < 0 then e_yq <= (others => '0'); elsif e_px > 239 then e_yq <= to_unsigned(239, 8);
                        else e_yq <= unsigned(e_px(7 downto 0)); end if;
                        e_st <= 77;
                    when 77 =>
                        n_ihp <= e_ihp; n_ilp <= e_ilp;
                        n_th0 <= unsigned(e_thv); n_a <= e_a; n_b <= unsigned(e_b(21 downto 0));
                        n_band <= e_band; n_i0 <= e_pos(5); n_w <= e_pos(4 downto 0); n_vb <= e_vb;
                        n_yi <= e_yq(7 downto 4); n_yf <= e_yq(3 downto 0);
                        e_st <= 0;
                    when others => e_st <= 0;
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Beam: the two line-locked oscillators (rope: two-rate; sinew), Osc phases
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                l_pb <= f_tb;
            elsif data_in.avid = '1' and s_avq = '0' then
                l_pb <= l_pb + resize(t_kay, 16);
            end if;
            if g_start = '1' then
                b_th <= c_th0;
                b_ps <= c_b;
                b_pa <= f_ta;
            elsif g_avid = '1' then
                if b_th(19) = '1' then b_th <= b_th + resize(c_ilp, 24); else b_th <= b_th + resize(c_ihp, 24); end if;
                b_ps <= b_ps + resize(c_a, 22);
                b_pa <= b_pa + resize(t_kax, 16);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_l   : signed(9 downto 0);
        variable v_ps  : unsigned(21 downto 0);
        variable v_pm  : signed(22 downto 0);
        variable v_g   : unsigned(8 downto 0);
        variable v_gs  : signed(9 downto 0);
        variable v_gi  : signed(8 downto 0);
        variable v_b   : signed(9 downto 0);
        variable v_c   : signed(9 downto 0);
        variable v_rgb : unsigned(23 downto 0);
        variable v_r, v_gg, v_bb : unsigned(15 downto 0);
        variable v_y   : unsigned(16 downto 0);
        variable v_cs  : signed(17 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_x0, v_x1, v_x2 : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: cosine address; the source luma (or the Osc)
            r_cosa <= b_th(19 downto 12);
            p1_th <= b_th; p1_ps <= b_ps;
            if f_osc = '1' then
                v_l := resize(f_tri(b_pa), 10) + resize(f_tri(l_pb), 10);
                p1_l <= resize(shift_right(v_l, 1), 8);
            else
                p1_l <= resize(shift_right(signed('0' & data_in.y) - 512, 2), 8);
            end if;
            -- 2: cosine ROM; melt product
            p2_th <= p1_th; p2_ps <= p1_ps;
            p2_pm <= resize(signed('0' & f_m5(8 downto 5)) * p1_l, 13);
            -- 3: fabric copy
            r_cosq2 <= r_cosq;
            p3_th <= p2_th; p3_ps <= p2_ps; p3_pm <= p2_pm;
            -- 4: psi + wiggle + melt; theta + melt
            v_pm := shift_left(resize(p3_pm, 23), 9);
            v_ps := p3_ps + unsigned(resize(shift_left(resize(signed(r_cosq2), 22), 4), 22)) + unsigned(resize(v_pm, 22));
            p4_ps <= v_ps;
            p4_th <= p3_th + unsigned(shift_left(resize(p3_pm, 24), 6));
            -- 5: table addresses; grain pieces
            r_la <= p4_th(19 downto 14) & p4_ps(20 downto 16);
            r_ca <= c_band(1) & p4_th(19 downto 13);
            r_ga <= p4_th(19 downto 14) & c_yi;
            p5_t8 <= p4_th(19 downto 12); p5_o8 <= p4_ps(20 downto 13); p5_ri <= p4_th(21 downto 20);
            p5_t6 <= p4_th(19 downto 14);
            -- 6: ROM outputs; grain: 9 theta + walk
            p6_g <= shift_left(p5_t8, 3) + p5_t8 + e_walk(to_integer(p5_ri));
            p6_o8 <= p5_o8; p6_t6 <= p5_t6;
            -- 7: fabric copies; grain + 0.35 Omega
            r_lrq2 <= r_lrq; r_lg0q2 <= r_lg0q; r_lg1q2 <= r_lg1q; r_lg2q2 <= r_lg2q; r_lbq2 <= r_lbq;
            r_rcq2 <= r_rcq; r_xcq2 <= r_xcq; r_gbq2 <= r_gbq;
            p7_g <= p6_g + shift_right(p6_o8, 2) + shift_right(p6_o8, 3);
            p7_t6 <= p6_t6;
            -- 8: expand; slice pair and differences; grain triangle
            p8_r <= f_ex6(r_lrq2);
            v_x0 := f_ex6(r_lg0q2); v_x1 := f_ex6(r_lg1q2); v_x2 := f_ex6(r_lg2q2);
            if c_i0 = '0' then
                p8_ga <= v_x0; p8_dg <= signed(resize(v_x1, 9)) - signed(resize(v_x0, 9));
            else
                p8_ga <= v_x1; p8_dg <= signed(resize(v_x2, 9)) - signed(resize(v_x1, 9));
            end if;
            p8_b <= f_ex6(r_lbq2);
            v_x0 := f_ex6(r_gbq2(11 downto 6)); v_x1 := f_ex6(r_gbq2(5 downto 0));
            p8_g0 <= v_x0; p8_dgb <= signed(resize(v_x1, 9)) - signed(resize(v_x0, 9));
            p8_rc <= unsigned(r_rcq2); p8_xc <= unsigned(r_xcq2);
            if p7_g >= 128 then p8_g <= signed(p7_g - 128) - 64; else p8_g <= signed(128 - p7_g) - 64; end if;
            p8_t6 <= p7_t6;
            -- 9: blend products; blue ramp; grain amplitude
            p9_pg <= resize(p8_dg * signed('0' & c_w), 14);
            p9_pb <= resize(p8_dgb * signed('0' & c_yf), 13);
            p9_b  <= signed(resize(p8_b, 10)) + resize(c_vb, 10);
            p9_r <= p8_r; p9_ga <= p8_ga; p9_g0 <= p8_g0; p9_rc <= p8_rc; p9_xc <= p8_xc;
            if p8_t6 >= 2 and p8_t6 <= 29 then
                p9_gg <= shift_right(p8_g, 1);
            else
                p9_gg <= shift_right(p8_g, 3);
            end if;
            -- 10: blends; clamp B
            v_gs := signed(resize(p9_ga, 10)) + resize(shift_right(p9_pg, 5), 10);
            p10_gm <= unsigned(v_gs(7 downto 0));
            v_gs := signed(resize(p9_g0, 10)) + resize(shift_right(p9_pb, 4), 10);
            p10_gb <= unsigned(v_gs(7 downto 0));
            if p9_b < 0 then p10_b <= (others => '0'); elsif p9_b > 255 then p10_b <= (others => '1');
            else p10_b <= unsigned(p9_b(7 downto 0)); end if;
            p10_r <= p9_r; p10_rc <= p9_rc; p10_xc <= p9_xc;
            if f_grain = '1' then p10_gg <= p9_gg; else p10_gg <= (others => '0'); end if;
            -- 11: band mux; grain onto G (middle band)
            case to_integer(c_band) is
                when 1 =>
                    p11_r <= p10_rc; p11_g <= p10_xc; p11_b <= to_unsigned(8, 8);
                when 2 =>
                    p11_r <= p10_rc; p11_g <= p10_gb; p11_b <= p10_xc;
                when others =>
                    p11_r <= p10_r;
                    v_c := signed(resize(p10_gm, 10)) + resize(p10_gg, 10);
                    if v_c < 0 then p11_g <= (others => '0'); elsif v_c > 255 then p11_g <= (others => '1');
                    else p11_g <= unsigned(v_c(7 downto 0)); end if;
                    p11_b <= p10_b;
            end case;
            -- 12: palette (channel routing; 6 = G inverted, 7 = routing 1 inverted)
            case to_integer(f_pal) is
                when 1 | 7 => v_rgb := p11_r & p11_b & p11_g;
                when 2 => v_rgb := p11_g & p11_r & p11_b;
                when 3 => v_rgb := p11_g & p11_b & p11_r;
                when 4 => v_rgb := p11_b & p11_r & p11_g;
                when 5 => v_rgb := p11_b & p11_g & p11_r;
                when others => v_rgb := p11_r & p11_g & p11_b;
            end case;
            if f_pal = 6 then v_rgb(15 downto 8) := not v_rgb(15 downto 8); end if;
            if f_pal = 7 then v_rgb := not v_rgb; end if;
            p12_r <= v_rgb(23 downto 16); p12_g <= v_rgb(15 downto 8); p12_b <= v_rgb(7 downto 0);
            -- 13: RGB -> YUV terms, Q6.  Y = 81 R + 159 G + 31 B (the 16/13 luma pre-compensation);
            --     U = 112 R - (94 G + 18 B);  V = 112 B - (38 R + 75 G)   (hardware: U = Cr, V = Cb)
            v_r := resize(p12_r, 16); v_gg := resize(p12_g, 16); v_bb := resize(p12_b, 16);
            p13_yrg <= resize(shift_left(v_r, 6), 17) + resize(shift_left(v_r, 4), 17) + resize(v_r, 17)
                       + resize(shift_left(v_gg, 7), 17) + resize(shift_left(v_gg, 5), 17) - resize(v_gg, 17);
            p13_yb  <= shift_left(v_bb, 5) - v_bb;
            p13_ur  <= shift_left(v_r, 7) - shift_left(v_r, 4);
            p13_ugb <= resize(shift_left(v_gg, 6), 17) + resize(shift_left(v_gg, 5), 17) - resize(shift_left(v_gg, 1), 17)
                       + resize(shift_left(v_bb, 4), 17) + resize(shift_left(v_bb, 1), 17);
            p13_vb  <= shift_left(v_bb, 7) - shift_left(v_bb, 4);
            p13_vrg <= resize(shift_left(v_r, 5), 17) + resize(shift_left(v_r, 2), 17) + resize(shift_left(v_r, 1), 17)
                       + resize(shift_left(v_gg, 6), 17) + resize(shift_left(v_gg, 3), 17) + resize(shift_left(v_gg, 1), 17)
                       + resize(v_gg, 17);
            -- 14: sums
            p14_y <= p13_yrg + resize(p13_yb, 17);
            p14_u <= signed(resize(p13_ur, 18)) - signed(resize(p13_ugb, 18));
            p14_v <= signed(resize(p13_vb, 18)) - signed(resize(p13_vrg, 18));
            -- 15: Y (cap 940), U, V
            v_y := shift_right(p14_y, 6);
            if v_y > 876 then p15_y <= to_unsigned(940, 10); else p15_y <= to_unsigned(64, 10) + v_y(9 downto 0); end if;
            v_cs := shift_right(p14_u, 6) + 512;
            if v_cs < 0 then p15_u <= (others => '0'); elsif v_cs > 1023 then p15_u <= (others => '1');
            else p15_u <= unsigned(v_cs(9 downto 0)); end if;
            v_cs := shift_right(p14_v, 6) + 512;
            if v_cs < 0 then p15_v <= (others => '0'); elsif v_cs > 1023 then p15_v <= (others => '1');
            else p15_v <= unsigned(v_cs(9 downto 0)); end if;
            -- 16
            p16_y <= p15_y; p16_u <= p15_u; p16_v <= p15_v;

            -- 17..22: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums,
            -- luma [1 2 1]; centre at stage 21 -> 22)
            s_csu(0) <= p16_u; s_csv(0) <= p16_v;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + p16_u - s_csu(7);
            s_boxv <= s_boxv + p16_v - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u22 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v22 <= v_sc(13 downto 4);
            s_y17 <= p16_y; s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19;
            s_y21 <= s_y20; s_y22x <= s_y21;
            v_sy := resize(s_y20, 12) + shift_left(resize(s_y21, 12), 1) + resize(s_y22x, 12);
            s_yf22 <= v_sy(11 downto 2);
            -- 23: luma toward black (white 940 -> 777); blanking gate
            if s_yf22 > 64 then v_dl := s_yf22 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(21) = '1' then
                s_y23 <= s_yf22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
            -- 24
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture sinew;
