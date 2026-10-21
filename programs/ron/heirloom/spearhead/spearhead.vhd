-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Spearhead v0.5.0 -- Heirloom #5 rebuild (2026-10-21), from WIN_20230103_01_29_30.
--
-- The photo (measured, see SUMMARY.md): two spearheads side by side (one
-- horizontal triangle oscillator, period 940 px), each a nest of outline
-- bands -- pale pink core, yellow, olive, hot pink, maroon, blue, navy,
-- black -- converging to tips at the bottom.  The frame is cut into 41-line
-- rows (a vertical sample-and-hold) and every row re-colours the bands: the
-- colour is PAL[i XOR j], j = the 3-bit band index of the spear field
-- F = |x - xc| - (C0 - k y^2), i = a 3-bit row code from a clipped vertical
-- triangle.  A black mosaic "shadow" (41 x 41 blocks, locked to the tile) is
-- thrown to the right of each spear and punches block holes through it; it
-- is the spear's outer band, mosaic-sampled and displaced row by row by a
-- slow vertical wave.  Behind: pastel rows (a colour per row code) stepped
-- by a zigzag of V chevrons centred between the spears.
--
--   K1  Spears   spears across, 1 .. 5 (2.04 = the photo)
--   K2  Taper    spear taper: blunt bullets .. needle tips
--   K3  Bands    outline band width 0.4x .. 3x
--   K4  Speed    bipolar: the row colours roll through the spears, the
--                shadow wave and the background chevrons travel (centre = still)
--   K5  Mosaic   row height = shadow block size, 12 .. 160 lines (41 = photo)
--   K6  Bend     how hard the source luma (about mid-grey) bends the bands and chevrons
--   S7  Shadow   the mosaic shadow and its holes On / Off
--   S8  Rows     row re-colouring On / Off (Off = plain nested bands)
--   S9  Palette  Photo / Alt (channel-rotated)
--   S10 Point    Down (the photo) / Up
--   S11 Source   Video / Osc (an internal triangle-wave oscillator replaces the video)
--   P12 THRUST   the spears swell and plunge: 0 = short dark needles from the
--                top, ~41 % = the photo, 100 % = neighbouring spears interlock
--
-- Geometry in virtual 1080-line units (timing-ID steps), so SD shows the
-- same spears (the 4:3 frame crops the sides).  Per-field constants and
-- the per-line spear profile (y^2 x taper) on one serial multiplier.
-- Generated avid; output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2
-- 2 1]/16, luma to ~777) before the blanking gate.  Latency 16 clocks in
-- every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture spearhead of program_top is
    constant C_LAT : integer := 16;   -- matches processing_delay_clks

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

    -- knob curves (64 entries, indexed by knob(9:4))
    type t_i64 is array (0 to 63) of integer;
    -- K1: tile period, Q.1 virtual px (idx 28 = 940 px, the photo)
    constant C_TP : t_i64 := (
        3840, 3743, 3649, 3557, 3467, 3380, 3294, 3211, 3130, 3051, 2974, 2899, 2826, 2755, 2685, 2618,
        2552, 2487, 2425, 2363, 2304, 2246, 2189, 2134, 2080, 2028, 1976, 1927, 1880, 1831, 1784, 1739,
        1696, 1653, 1611, 1570, 1531, 1492, 1455, 1418, 1382, 1347, 1313, 1280, 1248, 1216, 1186, 1156,
        1127, 1098, 1071, 1044, 1017, 992, 967, 942, 918, 895, 873, 851, 829, 808, 788, 768);
    -- K2: taper k (2 k y^2 = ((y^2 >> 6) x KY) >> 16 in Q.1 px; idx 32 = the photo)
    constant C_TKY : t_i64 := (
        346, 361, 377, 394, 411, 430, 449, 469, 489, 511, 534, 557, 582, 608, 635, 663,
        692, 723, 755, 788, 823, 859, 897, 937, 979, 1022, 1067, 1114, 1164, 1215, 1269, 1325,
        1384, 1445, 1509, 1576, 1646, 1719, 1795, 1874, 1957, 2044, 2134, 2229, 2328, 2431, 2538, 2651,
        2768, 2891, 3019, 3152, 3292, 3437, 3590, 3749, 3915, 4088, 4269, 4458, 4655, 4861, 5077, 5301);
    -- K3: band width scale, Q.8 (idx 29 = 1.0)
    constant C_TG : t_i64 := (
        102, 106, 109, 113, 116, 120, 124, 128, 132, 137, 141, 146, 150, 155, 160, 165,
        171, 176, 182, 188, 194, 200, 207, 214, 221, 228, 235, 243, 251, 256, 267, 276,
        285, 294, 304, 314, 324, 334, 345, 356, 368, 380, 392, 405, 418, 432, 446, 460,
        475, 491, 507, 523, 540, 558, 576, 595, 614, 634, 655, 676, 698, 720, 744, 768);
    -- K5: row height, Q.8 virtual lines (idx 30 = 41.15)
    constant C_TRH : t_i64 := (
        3072, 3201, 3335, 3475, 3621, 3773, 3931, 4097, 4268, 4448, 4634, 4829, 5031, 5243, 5463, 5692,
        5931, 6180, 6439, 6709, 6991, 7285, 7590, 7909, 8241, 8587, 8947, 9323, 9714, 10122, 10534, 10989,
        11450, 11931, 12432, 12954, 13497, 14064, 14654, 15269, 15910, 16578, 17274, 17999, 18754, 19541, 20361, 21216,
        22106, 23034, 24001, 25008, 26058, 27152, 28291, 29479, 30716, 32005, 33349, 34748, 36207, 37727, 39310, 40960);

    -- band thresholds at g = 1 (Q.1 px): T1..T6, the key threshold, ring low / high (left, right)
    type t_i10 is array (0 to 9) of integer;
    constant C_TC : t_i10 := (62, 116, 174, 216, 266, 314, 294, 160, 240, 440);
    -- per-field products on the serial unit: 0..9 = C_TC x g, 10 = Ki, 11 = KD, 12 = row-phase offsets
    type t_i4 is array (0 to 3) of integer;
    constant C_RK : t_i4 := (100, 95, 50, 48);

    -- colours {y, u = Cr, v = Cb}; y pre-compensated for the 13/16 output scale
    type t_col is array (0 to 2) of integer;
    type t_pal16 is array (0 to 15) of t_col;
    -- spear palette, code order (photo: pale pink, yellow, olive, hot pink, maroon, blue, navy, black), then Alt
    constant C_PF : t_pal16 := (
        (930, 567, 502), (914, 647, 113), (300, 522, 419), (512, 886, 665),
        (281, 647, 520), (249, 460, 820), (188, 501, 561), (171, 511, 496),
        (930, 474, 472), (800, 76, 589), (255, 422, 548), (930, 454, 133),
        (405, 448, 398), (384, 821, 409), (207, 562, 498), (160, 497, 520));
    -- background row colour per 4-bit phase of the row oscillator (fitted), then Alt
    type t_pal32 is array (0 to 31) of t_col;
    constant C_RC : t_pal32 := (
        (569, 511, 610), (569, 474, 583), (560, 491, 620), (581, 514, 734),
        (682, 708, 698), (564, 674, 658), (761, 726, 564), (785, 701, 679),
        (806, 646, 669), (796, 699, 575), (599, 656, 688), (662, 779, 737),
        (585, 662, 774), (608, 691, 729), (544, 716, 729), (528, 660, 703),
        (626, 602, 467), (576, 597, 510), (606, 622, 478), (714, 714, 406),
        (930, 579, 264), (795, 560, 310), (930, 447, 312), (930, 565, 278),
        (930, 585, 328), (930, 470, 328), (831, 596, 311), (930, 577, 186),
        (873, 672, 266), (896, 616, 263), (854, 602, 242), (773, 608, 300));
    -- chevron step (signed y, u, v per level, toward green), photo / Alt
    type t_dl is array (0 to 1) of t_col;
    constant C_DL : t_dl := ((18, -99, -55), (-102, 3, 107));

    function f_col(c : t_col) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(c(0), 10)) & std_logic_vector(to_unsigned(c(1), 10))
               & std_logic_vector(to_unsigned(c(2), 10));
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- line buffer (source luma, 8 bits) for Bend
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => (others => '0'));
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := (others => '0');

    -- internal oscillator source (Source = Osc)
    signal s_src : std_logic := '0';
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- timing / controls
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(8 downto 0) := (others => '0');
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_k1l, s_k2l, s_k3l, s_k5l : unsigned(9 downto 0) := (others => '0');
    signal s_spd : signed(9 downto 0) := (others => '0');
    signal s_bendk : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_gl : unsigned(11 downto 0) := to_unsigned(1696, 12);
    signal s_shad, s_rows, s_alt, s_up : std_logic := '0';

    -- per-field constants
    signal s_P2 : unsigned(11 downto 0) := to_unsigned(1880, 12);   -- tile period (Q.1)
    signal s_Ph : unsigned(11 downto 0) := to_unsigned(940, 12);    -- half period = spear centre
    signal s_KY : unsigned(12 downto 0) := to_unsigned(1384, 13);
    signal s_gq : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_RH : unsigned(15 downto 0) := to_unsigned(10534, 16);  -- row height Q.8
    signal s_BW : unsigned(8 downto 0) := to_unsigned(82, 9);       -- block width Q.1
    type t_u12a is array (natural range <>) of unsigned(11 downto 0);
    signal s_T : t_u12a(0 to 9) := (others => (others => '0'));     -- T1..T6, AT, LO, HIL, HIR
    signal s_Ki, s_KD, s_oi, s_oD : unsigned(15 downto 0) := (others => '0');
    signal s_dri, s_drd : unsigned(15 downto 0) := (others => '0');
    signal s_drz : unsigned(9 downto 0) := (others => '0');
    signal s_C0 : signed(12 downto 0) := to_signed(228, 13);
    signal s_ut : signed(13 downto 0) := (others => '0');
    signal s_u0, s_uA0, s_uG0 : unsigned(11 downto 0) := (others => '0');

    -- serial multiplier (shift-add, 16 steps)
    signal s_ma : unsigned(29 downto 0) := (others => '0');
    signal s_mb : unsigned(15 downto 0) := (others => '0');
    signal s_macc : unsigned(29 downto 0) := (others => '0');
    signal s_mi : unsigned(4 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_jb : unsigned(3 downto 0) := (others => '0');
    signal s_js : unsigned(4 downto 0) := (others => '0');
    signal s_jrun : std_logic := '0';

    -- per-line engine
    signal s_fall1 : std_logic := '0';
    signal s_ls : unsigned(5 downto 0) := (others => '0');
    signal s_first : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_racc : unsigned(16 downto 0) := (others => '0');
    signal s_nrow : std_logic := '0';
    signal s_thi, s_thd : unsigned(15 downto 0) := (others => '0');
    signal s_yf : unsigned(10 downto 0) := (others => '0');
    signal s_m1 : unsigned(21 downto 0) := (others => '0');
    signal s_phi_n, s_phr_n, s_phi, s_phr : signed(12 downto 0) := (others => '0');
    signal s_i_n, s_i : unsigned(2 downto 0) := (others => '0');
    signal s_ib_n, s_ib : unsigned(3 downto 0) := (others => '0');
    signal s_dq : unsigned(8 downto 0) := (others => '0');
    signal s_cb : signed(13 downto 0) := (others => '0');
    signal s_uB0_n, s_uB0 : unsigned(11 downto 0) := (others => '0');
    signal s_L_n, s_L : unsigned(9 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side
    signal s_wcol : unsigned(10 downto 0) := (others => '0');

    -- beam
    signal s_u, s_uA, s_uB, s_uG : unsigned(11 downto 0) := (others => '0');
    signal s_bc : unsigned(8 downto 0) := (others => '0');
    signal s_nb : std_logic := '0';
    signal s_px : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal s_yv2 : unsigned(7 downto 0) := (others => '0');
    signal s_bend3 : signed(17 downto 0) := (others => '0');
    signal s_bend : signed(11 downto 0) := (others => '0');
    signal s1_tq, s1_tqA, s1_dB : unsigned(11 downto 0) := (others => '0');
    signal s1_rB : signed(12 downto 0) := (others => '0');
    signal s1_nb : std_logic := '0';
    signal s2_F, s2_FA, s2_Fs : signed(13 downto 0) := (others => '0');
    signal s2_sd, s2_nb : std_logic := '0';
    signal s2_Zd : signed(13 downto 0) := (others => '0');
    signal s3_th : std_logic_vector(6 downto 0) := (others => '0');
    signal s3_A, s3_B : std_logic := '0';
    signal s3_Zs : unsigned(11 downto 0) := (others => '0');
    signal s4_Z : unsigned(9 downto 0) := (others => '0');
    signal s4_j, s4_i : unsigned(2 downto 0) := (others => '0');
    signal s4_ib : unsigned(3 downto 0) := (others => '0');
    signal s4_k : std_logic := '0';
    signal s5_fg : std_logic_vector(29 downto 0) := (others => '0');
    signal s5_rc : std_logic_vector(29 downto 0) := (others => '0');
    signal s5_n : unsigned(1 downto 0) := (others => '0');
    signal s6_dy, s6_du, s6_dv : signed(10 downto 0) := (others => '0');
    signal s5_k, s6_k, s7_k : std_logic := '0';
    signal s6_fg, s6_rc, s7_fg : std_logic_vector(29 downto 0) := (others => '0');
    signal s7_by, s7_bu, s7_bv : unsigned(9 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y9, s_y10, s_y11, s_y12, s_y13, s_y14x, s_yf14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- RAM
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field constants, per-line engine, serial unit
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_d : signed(10 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_t : signed(13 downto 0);
        variable v_r : unsigned(16 downto 0);
        variable v_th : unsigned(15 downto 0);
        variable v_tr : unsigned(14 downto 0);
        variable v_v : signed(16 downto 0);
        variable v_ph : signed(13 downto 0);
        variable v_cb : signed(13 downto 0);
        variable v_ub : unsigned(12 downto 0);
        variable v_yl : unsigned(12 downto 0);
        variable v_job : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            -- vblank zone = no avid for 1024 clocks
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
            elsif s_zc /= 511 then
                s_zc <= s_zc + 1;
            end if;

            -- source width
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- knobs with a +-6 count hold (table knobs), bipolar speed with a +-8 deadband
            v_k := unsigned(registers_in(0));
            v_d := signed(resize(v_k, 11)) - signed(resize(s_k1l, 11));
            if s_warm /= 15 or v_d > 6 or v_d < -6 then s_k1l <= v_k; end if;
            v_k := unsigned(registers_in(1));
            v_d := signed(resize(v_k, 11)) - signed(resize(s_k2l, 11));
            if s_warm /= 15 or v_d > 6 or v_d < -6 then s_k2l <= v_k; end if;
            v_k := unsigned(registers_in(2));
            v_d := signed(resize(v_k, 11)) - signed(resize(s_k3l, 11));
            if s_warm /= 15 or v_d > 6 or v_d < -6 then s_k3l <= v_k; end if;
            v_k := unsigned(registers_in(4));
            v_d := signed(resize(v_k, 11)) - signed(resize(s_k5l, 11));
            if s_warm /= 15 or v_d > 6 or v_d < -6 then s_k5l <= v_k; end if;
            v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;
            if v_s > -8 and v_s < 8 then s_spd <= (others => '0'); else s_spd <= v_s(9 downto 0); end if;
            s_bendk <= unsigned(registers_in(5)(9 downto 2));
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_gl, 13));

            -- virtual width
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            g_wm1 <= s_w - 1;

            -------------------------------------------------------------------
            -- per-field sequencer (vblank)
            -------------------------------------------------------------------
            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_gl, 13)) + shift_right(s_gdiff, 3);     -- P12 glide, 1/8 a field
                s_gl <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_shad <= not registers_in(6)(0);
                s_rows <= not registers_in(6)(1);
                s_alt  <= registers_in(6)(2);
                s_up   <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                -- motion: row-code oscillator, shadow wave, background chevrons
                s_dri <= s_dri + unsigned(resize(s_spd, 16));
                s_drd <= s_drd - unsigned(resize(s_spd, 16));
                s_drz <= s_drz + unsigned(resize(s_spd(9 downto 5), 10));
            end if;
            if s_zc = 2 then
                s_P2 <= to_unsigned(C_TP(to_integer(s_k1l(9 downto 4))), 12);
                s_KY <= to_unsigned(C_TKY(to_integer(s_k2l(9 downto 4))), 13);
                s_gq <= to_unsigned(C_TG(to_integer(s_k3l(9 downto 4))), 10);
                s_RH <= to_unsigned(C_TRH(to_integer(s_k5l(9 downto 4))), 16);
                -- Thrust: C0 = -100 .. +420 px (Q.1: p + p/64 - 200)
                s_C0 <= signed(resize(s_gl(11 downto 2), 13)) + signed(resize(s_gl(11 downto 8), 13)) - 200;
            end if;
            if s_zc = 3 then
                s_Ph <= '0' & s_P2(11 downto 1);
                s_BW <= s_RH(15 downto 7);
                -- tile origin: the spears sit either side of x = 1002 (the photo); a single
                -- spear slides to the centre (origin shift (P2 - 1880) 15/16 above two across)
                if s_P2 > 1880 then
                    v_t := signed(resize(s_P2, 14)) - 1880;
                    s_ut <= signed(resize(s_P2, 14)) - 84 - signed(resize(s_vw, 14)) - v_t + shift_right(v_t, 4);
                else
                    s_ut <= signed(resize(s_P2, 14)) - 84 - signed(resize(s_vw, 14));
                end if;
            end if;
            if s_zc >= 4 and s_zc <= 9 then
                if s_ut < 0 then s_ut <= s_ut + signed(resize(s_P2, 14));
                elsif s_ut >= signed(resize(s_P2, 14)) then s_ut <= s_ut - signed(resize(s_P2, 14)); end if;
            end if;
            if s_zc = 10 then
                s_u0 <= unsigned(s_ut(11 downto 0));
                v_t := s_ut - 30;
                if v_t < 0 then v_t := v_t + signed(resize(s_P2, 14)); end if;
                s_uA0 <= unsigned(v_t(11 downto 0));
                v_t := s_ut + signed(resize(s_Ph, 14)) + 16;          -- chevron centre 8 px left of the midpoint
                if v_t >= signed(resize(s_P2, 14)) then v_t := v_t - signed(resize(s_P2, 14)); end if;
                s_uG0 <= unsigned(v_t(11 downto 0));
            end if;

            -------------------------------------------------------------------
            -- serial unit: field jobs (zc 32 + 20 k), line jobs (s_ls)
            -------------------------------------------------------------------
            if s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(28 downto 0) & '0';
                s_mb <= '0' & s_mb(15 downto 1);
                if s_mi = 15 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            end if;
            -- 14 field jobs, 20 clocks each: start at step 0, capture at step 18
            if s_zc = 31 then
                s_jb <= (others => '0'); s_js <= (others => '0'); s_jrun <= '1';
            elsif s_jrun = '1' then
                if s_js = 19 then
                    s_js <= (others => '0');
                    if s_jb = 13 then s_jrun <= '0'; else s_jb <= s_jb + 1; end if;
                else
                    s_js <= s_js + 1;
                end if;
            end if;
            if s_jrun = '1' and s_js = 0 then
                v_job := to_integer(s_jb);
                s_macc <= (others => '0'); s_mi <= (others => '0'); s_mrun <= '1';
                if s_jb < 10 then
                    s_ma <= to_unsigned(C_TC(v_job), 30); s_mb <= resize(s_gq, 16);
                else
                    s_ma <= resize(s_RH, 30); s_mb <= to_unsigned(C_RK(v_job - 10), 16);
                end if;
            elsif s_jrun = '1' and s_js = 18 then
                v_job := to_integer(s_jb);
                if s_jb < 10 then
                    s_T(v_job) <= s_macc(19 downto 8);
                elsif s_jb = 10 then s_Ki <= s_macc(23 downto 8);
                elsif s_jb = 11 then s_KD <= s_macc(23 downto 8);
                elsif s_jb = 12 then s_oi <= s_macc(23 downto 8);
                else s_oD <= s_macc(23 downto 8);
                end if;
            end if;

            -- field start of the line engine: row -1 covers lines 0 .. 9.4
            if s_zc = 330 then
                s_vy_n <= (others => '0');
                s_racc <= resize(s_RH, 17) - 2406;
                s_thi <= to_unsigned(57309, 16) - s_oi + s_dri;
                s_thd <= to_unsigned(4889, 16) - s_oD + s_drd;
            end if;

            -------------------------------------------------------------------
            -- per-line engine: computes the NEXT line (kicked at the source
            -- avid fall, and once in the zone for line 0)
            -------------------------------------------------------------------
            s_fall1 <= '0';
            if data_in.avid = '0' and s_avq = '1' then s_fall1 <= '1'; end if;
            if s_zc = 340 then
                s_ls <= to_unsigned(1, 6); s_first <= '1';
            elsif s_fall1 = '1' then
                s_ls <= to_unsigned(1, 6); s_first <= '0';
                s_vy_n <= s_vy_n + s_ys;
            elsif s_ls /= 0 then
                if s_ls = 40 then s_ls <= (others => '0'); else s_ls <= s_ls + 1; end if;
            end if;
            if s_ls = 1 then
                -- spear profile row: y (flipped for Point Up)
                if s_up = '1' then
                    if s_vy_n(18 downto 8) > 1079 then s_yf <= (others => '0');
                    else s_yf <= to_unsigned(1079, 11) - s_vy_n(18 downto 8); end if;
                else
                    s_yf <= s_vy_n(18 downto 8);
                end if;
                if s_first = '0' then s_racc <= s_racc + s_ys; end if;
                s_nrow <= s_first;
            elsif s_ls = 2 then
                s_ma <= resize(s_yf, 30); s_mb <= resize(s_yf, 16);
                s_macc <= (others => '0'); s_mi <= (others => '0'); s_mrun <= '1';
                if s_racc >= resize(s_RH, 17) then
                    s_racc <= s_racc - resize(s_RH, 17);
                    s_nrow <= '1';
                    s_thi <= s_thi + s_Ki;
                    s_thd <= s_thd + s_KD;
                end if;
            elsif s_ls = 19 then
                s_m1 <= s_macc(21 downto 0);
            elsif s_ls = 20 then
                s_ma <= resize(s_m1(21 downto 6), 30); s_mb <= resize(s_KY, 16);
                s_macc <= (others => '0'); s_mi <= (others => '0'); s_mrun <= '1';
            elsif s_ls = 37 then
                -- phi = C0 - k y^2  (Q.1)
                s_phi_n <= s_C0 - signed(resize(s_macc(28 downto 16), 13));
                -- row code: v = 8.3 - 9.6 |2 th - 1|  (x 4096)
                v_th := s_thi;
                if v_th(15) = '1' then v_tr := v_th(14 downto 0); else v_tr := not v_th(14 downto 0); end if;
                v_v := to_signed(33997, 17) - signed(resize(v_tr, 17)) - signed(resize(v_tr(14 downto 3), 17))
                       - signed(resize(v_tr(14 downto 4), 17)) - signed(resize(v_tr(14 downto 7), 17));
                s_ib_n <= s_thi(15 downto 12);
                if s_rows = '0' or v_v < 0 then s_i_n <= "000";
                elsif v_v >= 32768 then s_i_n <= "111";
                else s_i_n <= unsigned(v_v(14 downto 12)); end if;
                -- shadow displacement D = 134 - 256 min(th, 1 - th)  (Q.1: 268 - tD / 128)
                v_th := s_thd;
                if v_th(15) = '1' then v_tr := not v_th(14 downto 0); else v_tr := v_th(14 downto 0); end if;
                s_dq <= to_unsigned(268, 9) - resize(v_tr(14 downto 7), 9);
            elsif s_ls = 38 then
                if s_nrow = '1' then s_phr_n <= s_phi_n; end if;
                -- block-centre sample offset bw/2 - D, wrapped
                v_cb := signed(resize(s_BW(8 downto 1), 14)) - signed(resize(s_dq, 14));
                if v_cb < 0 then v_cb := v_cb + signed(resize(s_P2, 14)); end if;
                s_cb <= v_cb;
                -- background chevron line term: (0.7 y - 950) x 1024/1350 = y (1/2 + 1/32) - 721, + drift
                if s_up = '1' then v_yl := resize(s_yf, 13); else v_yl := resize(s_vy_n(18 downto 8), 13); end if;
                v_yl := resize(v_yl(12 downto 1), 13) + resize(v_yl(12 downto 5), 13) - 721;
                s_L_n <= v_yl(9 downto 0) + s_drz;
            elsif s_ls = 39 then
                v_ub := resize(s_u0, 13) + unsigned(s_cb(12 downto 0));
                if v_ub >= resize(s_P2, 13) then v_ub := v_ub - resize(s_P2, 13); end if;
                s_uB0_n <= v_ub(11 downto 0);
            end if;

            -- load the line's values at the source avid rise
            if data_in.avid = '1' and s_avq = '0' then
                s_phi <= s_phi_n; s_phr <= s_phr_n; s_i <= s_i_n; s_ib <= s_ib_n; s_uB0 <= s_uB0_n; s_L <= s_L_n;
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
    -- Source side: line-buffer write (video luma, or the oscillator)
    ---------------------------------------------------------------------------
    p_src : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_oty(9 downto 2));
            else
                s_lb_wd <= data_in.y(9 downto 2);
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: tile coordinates (Q.1 virtual px), mosaic block counter
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(12 downto 0);
        variable v_b : unsigned(9 downto 0);
        variable v_wr : std_logic;
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_u <= s_u0; s_uA <= s_uA0; s_uG <= s_uG0; s_uB <= s_uB0;
                s_bc <= (others => '0'); s_nb <= '1'; s_px <= (others => '0');
            elsif g_avid = '1' then
                v_n := resize(s_u, 13) + s_xs;
                if v_n >= resize(s_P2, 13) then s_u <= resize(v_n - s_P2, 12); v_wr := '1';
                else s_u <= v_n(11 downto 0); v_wr := '0'; end if;
                v_n := resize(s_uA, 13) + s_xs;
                if v_n >= resize(s_P2, 13) then s_uA <= resize(v_n - s_P2, 12); else s_uA <= v_n(11 downto 0); end if;
                v_n := resize(s_uB, 13) + s_xs;
                if v_n >= resize(s_P2, 13) then s_uB <= resize(v_n - s_P2, 12); else s_uB <= v_n(11 downto 0); end if;
                v_n := resize(s_uG, 13) + s_xs;
                if v_n >= resize(s_P2, 13) then s_uG <= resize(v_n - s_P2, 12); else s_uG <= v_n(11 downto 0); end if;
                v_b := resize(s_bc, 10) + s_xs;
                if v_wr = '1' then
                    s_bc <= (others => '0'); s_nb <= '1';                 -- the block grid restarts every tile
                elsif v_b >= resize(s_BW, 10) then
                    s_bc <= resize(v_b - s_BW, 9); s_nb <= '1';
                else
                    s_bc <= v_b(8 downto 0); s_nb <= '0';
                end if;
                s_px <= s_px + 1;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a : signed(13 downto 0);
        variable v_t9 : unsigned(8 downto 0);
        variable v_c : integer range 0 to 15;
        variable v_c2 : integer range 0 to 31;
        variable v_y, v_u, v_v : signed(11 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- bend: source luma about mid-grey (a few pixels early) x K6, Q.1 px, +-440 at most
            s_lb_ra <= s_px;
            s_yv2 <= unsigned(s_lb_q);
            s_bend3 <= (signed(resize(s_yv2, 9)) - 128) * signed('0' & s_bendk);
            s_bend <= s_bend3(17 downto 6);

            -- 1: folds about the spear centre / chevron centre
            v_a := signed(resize(s_u, 14)) - signed(resize(s_Ph, 14));
            v_a := abs(v_a); s1_tq <= unsigned(v_a(11 downto 0));
            v_a := signed(resize(s_uA, 14)) - signed(resize(s_Ph, 14));
            v_a := abs(v_a); s1_tqA <= unsigned(v_a(11 downto 0));
            v_a := signed(resize(s_uG, 14)) - signed(resize(s_Ph, 14));
            v_a := abs(v_a); s1_dB <= unsigned(v_a(11 downto 0));
            s1_rB <= signed(resize(s_uB, 13)) - signed(resize(s_Ph, 13));
            s1_nb <= s_nb;

            -- 2: spear field F = |x| - phi - bend; key field 15 px earlier; ring sample
            s2_F  <= signed(resize(s1_tq, 14)) - resize(s_phi, 14) - resize(s_bend, 14);
            s2_FA <= signed(resize(s1_tqA, 14)) - resize(s_phi, 14) - resize(s_bend, 14);
            s2_Fs <= signed(resize(unsigned(abs(s1_rB)), 14)) - resize(s_phr, 14);
            if s1_rB > 0 then s2_sd <= '1'; else s2_sd <= '0'; end if;
            s2_nb <= s1_nb;
            s2_Zd <= signed(resize(s1_dB, 14)) + resize(s_bend, 14);

            -- 3: band thresholds, key A, mosaic ring B (held per block), chevron phase x 0.375
            if s2_F >= 0 then s3_th(0) <= '1'; else s3_th(0) <= '0'; end if;
            for k in 1 to 6 loop
                if s2_F >= signed(resize(s_T(k - 1), 14)) then s3_th(k) <= '1'; else s3_th(k) <= '0'; end if;
            end loop;
            if s2_FA < signed(resize(s_T(6), 14)) then s3_A <= '1'; else s3_A <= '0'; end if;
            if s2_nb = '1' then
                if s2_Fs >= signed(resize(s_T(7), 14)) and
                   ((s2_sd = '1' and s2_Fs < signed(resize(s_T(9), 14))) or
                    (s2_sd = '0' and s2_Fs < signed(resize(s_T(8), 14)))) then
                    s3_B <= s_shad;
                else
                    s3_B <= '0';
                end if;
            end if;
            s3_Zs <= unsigned(resize(shift_right(s2_Zd, 2), 12) + resize(shift_right(s2_Zd, 3), 12));   -- x 1024/2700 (wraps)

            -- 4: band index, chevron phase, key
            if    s3_th(6) = '1' then s4_j <= "111";
            elsif s3_th(5) = '1' then s4_j <= "110";
            elsif s3_th(4) = '1' then s4_j <= "101";
            elsif s3_th(3) = '1' then s4_j <= "100";
            elsif s3_th(2) = '1' then s4_j <= "011";
            elsif s3_th(1) = '1' then s4_j <= "010";
            elsif s3_th(0) = '1' then s4_j <= "001";
            else s4_j <= "000"; end if;
            s4_i <= s_i; s4_ib <= s_ib;
            s4_Z <= s3_Zs(9 downto 0) + s_L;
            s4_k <= s3_A xor s3_B;

            -- 5: palettes (spear colour = PAL[i xor j]; background row colour), chevron level
            v_c := to_integer(s4_i xor s4_j);
            if s_alt = '1' then v_c := v_c + 8; end if;
            s5_fg <= f_col(C_PF(v_c));
            v_c2 := to_integer(s4_ib);
            if s_alt = '1' then v_c2 := v_c2 + 16; end if;
            s5_rc <= f_col(C_RC(v_c2));
            if s4_Z(9) = '1' then v_t9 := not s4_Z(8 downto 0); else v_t9 := s4_Z(8 downto 0); end if;
            if v_t9 >= 384 then s5_n <= "10";           -- 3 chevron levels
            elsif v_t9 >= 128 then s5_n <= "01";
            else s5_n <= "00"; end if;
            s5_k <= s4_k;

            -- 6: chevron step (0, 1 or 2 steps toward green)
            if s_alt = '1' then v_c := 1; else v_c := 0; end if;
            case s5_n is
                when "01"   => s6_dy <= to_signed(C_DL(v_c)(0), 11); s6_du <= to_signed(C_DL(v_c)(1), 11); s6_dv <= to_signed(C_DL(v_c)(2), 11);
                when "10"   => s6_dy <= to_signed(2 * C_DL(v_c)(0), 11); s6_du <= to_signed(2 * C_DL(v_c)(1), 11); s6_dv <= to_signed(2 * C_DL(v_c)(2), 11);
                when others => s6_dy <= (others => '0'); s6_du <= (others => '0'); s6_dv <= (others => '0');
            end case;
            s6_rc <= s5_rc; s6_fg <= s5_fg; s6_k <= s5_k;

            -- 7: background = row colour + chevron steps (clamped)
            v_y := signed(resize(unsigned(s6_rc(29 downto 20)), 12)) + resize(s6_dy, 12);
            v_u := signed(resize(unsigned(s6_rc(19 downto 10)), 12)) + resize(s6_du, 12);
            v_v := signed(resize(unsigned(s6_rc(9 downto 0)), 12)) + resize(s6_dv, 12);
            if v_y < 64 then s7_by <= to_unsigned(64, 10); elsif v_y > 930 then s7_by <= to_unsigned(930, 10); else s7_by <= unsigned(v_y(9 downto 0)); end if;
            if v_u < 16 then s7_bu <= to_unsigned(16, 10); elsif v_u > 1008 then s7_bu <= to_unsigned(1008, 10); else s7_bu <= unsigned(v_u(9 downto 0)); end if;
            if v_v < 16 then s7_bv <= to_unsigned(16, 10); elsif v_v > 1008 then s7_bv <= to_unsigned(1008, 10); else s7_bv <= unsigned(v_v(9 downto 0)); end if;
            s7_fg <= s6_fg; s7_k <= s6_k;

            -- 8: compose (key: spear / background)
            if s7_k = '1' then
                s_y8 <= unsigned(s7_fg(29 downto 20)); s_u8 <= unsigned(s7_fg(19 downto 10)); s_v8 <= unsigned(s7_fg(9 downto 0));
            else
                s_y8 <= s7_by; s_u8 <= s7_bu; s_v8 <= s7_bv;
            end if;

            -- 9..14: output conditioning (centre at stage 13 -> 14)
            s_csu(0) <= s_u8; s_csv(0) <= s_v8;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u8 - s_csu(7);
            s_boxv <= s_boxv + s_v8 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u14 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v14 <= v_sc(13 downto 4);
            s_y9 <= s_y8; s_y10 <= s_y9; s_y11 <= s_y10; s_y12 <= s_y11;
            s_y13 <= s_y12; s_y14x <= s_y13;
            v_sy := resize(s_y12, 12) + shift_left(resize(s_y13, 12), 1) + resize(s_y14x, 12);
            s_yf14 <= v_sy(11 downto 2);
            -- 15: luma toward black; blanking gate
            if s_yf14 > 64 then v_dl := s_yf14 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(13) = '1' then
                s_y15 <= s_yf14 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u15 <= s_u14; s_v15 <= s_v14;
            else
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            end if;
            -- 16
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture spearhead;
