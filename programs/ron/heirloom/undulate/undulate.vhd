-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Undulate v0.1.1 -- Heirloom (video phase), from the clip WIN_20230301_23_17_23 at 0.5 s.
-- v0.1.1: S10 Square is a hard full-level bar (48 % duty) for the ripples and a hard bar for the flecks.
--
-- The still: a field of green sine ripples (period 23 .. 110 px) bent into S-curves on the left, a saddle
-- on the right and fine horizontal lines at the top right, over a blue 45-degree folded ramp (blue in the
-- top-left and bottom-right, black in the bottom-left), with red / pink flecks sitting on one flank of
-- every ripple.  The ripple brightness breathes down the screen with a 225-line period (the Y oscillator).
-- In the clip the field is still and the flecks crawl up the ripples at ~70 px/s.
--
-- The patch, as fitted (undulate.py has every table):
--   phi    = N * plane + N * U * D(x, y) + Etch * luma + flow (+ Wobble * sin(theta_y))
--            plane = the phase's plane fit; D = the measured phase field minus that plane, a 60 x 34 table
--            (32-px cells) read bilinearly; N = Ripples, U = Undulate
--   green  = E(x, y) * (1 + depth cos(theta_y)) * wave(phi)      wave = ((1 + cos) / 2)^2, or a square (Wave)
--   blue   = clip(36 + 144 |v| + 80 u) * Blue - green / 4          v = (X + Y) / sqrt2, u = (X - Y) / sqrt2
--   red    = comparator( A(x, y) * ((1 + cos(phi + delta)) / 2)^6 + noise - threshold )
--            delta = 0.33 + 0.39 X - 0.035 Y cycles (+ a per-line jitter): the red flank drifts across the
--            width; A = a table; noise = two LFSR streams seeded per 8-line group (scrolling up 1.19 lines per
--            field), blended down the group in quarter steps, smoothed along the line (decay 1/16); the jitter
--            eases toward the group seed's top byte
--   RGB -> YUV at the end (the picture is RGB-native, like the synth's encoder)
--
--   K1  Ripples   stripe density x0.4 .. x2.5 (512 = the clip)
--   K2  Bands     the breathing bands: 1.6 .. 14 per screen and deeper away from the centre (512 = 4.8, 30 %)
--   K3  Blue      the blue fold's level: none .. x4 (512 = the clip)
--   K4  Flow      bipolar: the ripples stream through the field (centre = still, as the clip)
--   K5  Flecks    red density: sparse .. flooding the flanks
--   K6  Etch      how far the camera (or the Osc) bends the ripples
--   S7  Flip      the field upside down
--   S8  Palette   Neon (the clip) / Ember (the RGB cables swapped: red ripples, green fold, blue flecks)
--   S9  Wobble    the band oscillator also bends the ripples (wavy stripes)
--   S10 Wave      Sine (the clip's glowing ripples) / Square (hard-keyed neon bars: green and red pulses square)
--   S11 Source    Video / Osc
--   P12 UNDULATE  0 = plain diagonal stripes (the bare ramp), 465 = the clip, 100 % = the undulation x2.2
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine (plane start, band
-- oscillator, fold / red-offset line steps, noise seeds), a node prefetch unit that reads the two table rows
-- at every 32-px cell entry (vertical interpolation, D scaled by N U on a radix-8 serial multiplier, the
-- next cell's slope), three value lanes A / B / P, forward beam DDAs, and a 15-stage pixel path (pulse ROMs,
-- fold, comparator, RGB -> YUV) into the output conditioning (one [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V,
-- luma 13/16, gate).  Virtual 1080-line coordinates (SD shows the centre 4:3); generated avid; one latency (C_LAT
-- 20) in every mode; colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.undulate_pkg.all;
use work.all;

architecture undulate of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

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
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- table cell of a table coordinate q (h units, 64 per cell): 0 .. 59
    function f_cell(q : signed(12 downto 0)) return unsigned is
    begin
        if q(12) = '1' then
            return to_unsigned(0, 6);
        elsif unsigned(q(11 downto 6)) >= 59 then
            return to_unsigned(59, 6);
        else
            return unsigned(q(11 downto 6));
        end if;
    end function;
    function f_clip8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a : unsigned(7 downto 0) := (others => '0');
    signal r_cs_q : signed(7 downto 0) := (others => '0');
    signal t_ra   : unsigned(10 downto 0) := (others => '0');
    signal t_rq   : std_logic_vector(15 downto 0) := (others => '0');
    signal m_rq   : unsigned(3 downto 0) := (others => '0');
    signal pg_a   : unsigned(8 downto 0) := (others => '0');
    signal pr_a   : unsigned(8 downto 0) := (others => '0');
    signal pg_q, pr_q : unsigned(7 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw flow, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(320 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(465 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_flip, s_pal, s_wob, s_sq, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal c_n10  : unsigned(11 downto 0) := to_unsigned(1024, 12);   -- Ripples N, Q10
    signal c_u8   : unsigned(9 downto 0) := to_unsigned(256, 10);     -- Undulate U, Q8
    signal c_nu   : unsigned(10 downto 0) := to_unsigned(256, 11);    -- N U, Q8
    signal c_sxh  : unsigned(13 downto 0) := to_unsigned(4926, 14);   -- plane step per h, Q20 cycles
    signal c_sxp  : unsigned(15 downto 0) := to_unsigned(9852, 16);   -- plane step per pixel
    signal c_syl  : unsigned(19 downto 0) := (others => '0');         -- plane step per virtual line, Q20 (negative)
    signal c_base : unsigned(19 downto 0) := (others => '0');         -- plane at (0, 0), Q20
    signal c_px0  : unsigned(19 downto 0) := (others => '0');         -- plane step * xb0
    signal c_fstep : unsigned(15 downto 0) := to_unsigned(291, 16);  -- band oscillator step per line, Q16
    signal c_depth : unsigned(8 downto 0) := to_unsigned(77, 9);     -- band depth, Q8
    signal c_bg   : unsigned(7 downto 0) := to_unsigned(64, 8);       -- blue gain, Q6
    signal c_flstep : unsigned(15 downto 0) := (others => '0');       -- flow per field, Q16 cycles
    signal c_fl   : unsigned(15 downto 0) := (others => '0');         -- flow accumulator
    signal c_scr  : unsigned(15 downto 0) := (others => '0');         -- fleck scroll, Q4 lines
    signal c_thr  : signed(8 downto 0) := to_signed(141, 9);          -- fleck threshold, Q8
    signal c_eq   : unsigned(9 downto 0) := to_unsigned(75, 10);      -- etch, Q16 cycles per luma unit
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);     -- first pixel's h
    signal c_q0   : signed(12 downto 0) := to_signed(0, 13);          -- its table coordinate (h - 16)
    signal c_i0   : unsigned(5 downto 0) := (others => '0');          -- its cell
    signal c_v0   : unsigned(15 downto 0) := to_unsigned(96, 16);     -- fold v offset at xb0, Q14
    signal c_vsp  : unsigned(4 downto 0) := to_unsigned(12, 5);       -- fold step per pixel, Q14
    signal c_vsl  : unsigned(12 downto 0) := to_unsigned(1373, 13);   -- fold step per line, Q20
    signal c_dql  : unsigned(8 downto 0) := to_unsigned(67, 9);       -- red offset step per line, Q20 (subtracted)
    signal c_dq0  : unsigned(16 downto 0) := to_unsigned(3440, 17);   -- red offset at xb0, Q20
    signal c_dsp  : unsigned(9 downto 0) := to_unsigned(430, 10);     -- red offset step per pixel, Q20

    -- field serial multiplier
    signal f_a   : unsigned(27 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(27 downto 0) := (others => '0');
    signal z_fa  : unsigned(27 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (computes the NEXT line; the n_vs / n_uv / n_dq accumulators hold the current line at g_start)
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_vyi  : unsigned(10 downto 0) := (others => '0');
    signal e_vyf  : unsigned(10 downto 0) := (others => '0');
    signal e_j, e_j1 : unsigned(5 downto 0) := (others => '0');
    signal e_sl   : unsigned(11 downto 0) := (others => '0');
    signal e_hs   : unsigned(15 downto 0) := (others => '0');
    signal n_seed, n_seedb : unsigned(15 downto 0) := x"ACE1";        -- noise seeds of the line's 8-line group and the next
    signal n_nf   : unsigned(2 downto 0) := (others => '0');            -- the line's place in its group
    signal n_pj   : signed(9 downto 0) := (others => '0');             -- red jitter, eased per line (Q2)
    signal e_pjt  : signed(7 downto 0) := (others => '0');
    signal e_th   : unsigned(15 downto 0) := (others => '0');
    signal e_cosv, e_sinv : signed(7 downto 0) := (others => '0');
    signal e_wob  : signed(16 downto 0) := (others => '0');
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(23 downto 0) := (others => '0');
    signal n_ja, n_jb : unsigned(10 downto 0) := (others => '0');
    signal n_fy   : unsigned(4 downto 0) := (others => '0');
    signal n_ymod : unsigned(5 downto 0) := to_unsigned(16, 6);         -- band modulation, Q4 (0 .. 32)
    signal n_L    : unsigned(19 downto 0) := (others => '0');
    signal n_pxs  : unsigned(19 downto 0) := (others => '0');
    signal n_dqs  : unsigned(19 downto 0) := (others => '0');
    signal n_vs   : unsigned(21 downto 0) := (others => '0');            -- fold v at h = 0, Q20 (two's complement)
    signal n_uv   : unsigned(21 downto 0) := (others => '0');            -- u - v, Q20
    signal n_dq   : unsigned(19 downto 0) := (others => '0');            -- red offset at h = 0, Q20
    signal y_cnt  : unsigned(5 downto 0) := (others => '0');

    -- prefetch constants (loaded in hblank for the next line)
    signal l_ja, l_jb : unsigned(10 downto 0) := (others => '0');
    signal l_fy   : unsigned(4 downto 0) := (others => '0');
    signal l_ymod : unsigned(5 downto 0) := to_unsigned(16, 6);

    -- prefetch unit (always writes lane P, and the slope from lane B to it)
    signal pf_c   : unsigned(3 downto 0) := (others => '0');
    signal pf_nc  : unsigned(5 downto 0) := (others => '0');
    signal pf_d0, pf_d1 : signed(11 downto 0) := (others => '0');
    signal pf_e0, pf_e1 : unsigned(3 downto 0) := (others => '0');
    signal pf_r0, pf_r1 : unsigned(3 downto 0) := (others => '0');
    signal pf_dd  : signed(12 downto 0) := (others => '0');
    signal pf_de, pf_dr : signed(4 downto 0) := (others => '0');
    signal pf_pd  : signed(17 downto 0) := (others => '0');
    signal pf_pe, pf_pr : signed(6 downto 0) := (others => '0');
    signal pf_vd  : signed(14 downto 0) := (others => '0');
    signal pf_ve  : unsigned(8 downto 0) := (others => '0');
    signal pf_vr  : unsigned(8 downto 0) := (others => '0');
    signal pf_ve2 : unsigned(14 downto 0) := (others => '0');
    signal pf_vr2 : unsigned(14 downto 0) := (others => '0');
    signal pf_acc : signed(25 downto 0) := (others => '0');            -- vd * N U, 3 bits of N U per clock
    signal pf_nu  : unsigned(11 downto 0) := (others => '0');
    -- lanes A (current cell), B (next node), P (prefetched): D scaled by N U, Q16 cycles mod 2^22
    signal la_d, lb_d, lp_d : unsigned(21 downto 0) := (others => '0');
    signal lp_dd, lp_ddx : unsigned(21 downto 0) := (others => '0');   -- P - B, and xs (P - B)
    signal la_e, lb_e, lp_e : unsigned(8 downto 0) := (others => '0');
    signal la_r, lb_r, lp_r : unsigned(9 downto 0) := (others => '0');

    -- beam: DDAs of the stage-1 pixel (the pixel path reads them a few pixels late: constant shears of
    -- 1 .. 4 px, invisible), the D lane (restarted at every cell entry), the noise seed
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_q1   : signed(12 downto 0) := (others => '0');
    signal b_pl   : unsigned(19 downto 0) := (others => '0');
    signal b_dq   : unsigned(19 downto 0) := (others => '0');
    signal b_v    : unsigned(15 downto 0) := (others => '0');
    signal b_uv   : unsigned(9 downto 0) := (others => '0');
    signal b_ce   : std_logic := '0';
    signal b_i1   : unsigned(5 downto 0) := (others => '0');
    signal b_req  : std_logic := '0';
    signal b_reqn : unsigned(6 downto 0) := (others => '0');
    signal b_da   : unsigned(21 downto 0) := (others => '0');          -- D lane: VA * 64 + frac * (VB - VA), mod 2^22
    signal b_ddx  : unsigned(21 downto 0) := (others => '0');          -- xs (VB - VA)
    signal b_seed, b_seedb : unsigned(15 downto 0) := x"ACE1";
    signal b_nf   : unsigned(2 downto 0) := (others => '0');

    -- pixel path
    signal p2_pe  : signed(11 downto 0) := (others => '0');
    signal p2_pr  : signed(12 downto 0) := (others => '0');
    signal p2_ae  : unsigned(8 downto 0) := (others => '0');
    signal p2_ar  : unsigned(9 downto 0) := (others => '0');
    signal p2_v, p2_uv : signed(9 downto 0) := (others => '0');
    signal p3_eq  : unsigned(8 downto 0) := (others => '0');
    signal p3_rq  : unsigned(9 downto 0) := (others => '0');
    signal p3_av, p4_av : unsigned(8 downto 0) := (others => '0');
    signal p3_u, p4_u : signed(9 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ph  : unsigned(15 downto 0) := (others => '0');
    signal p4_el  : unsigned(13 downto 0) := (others => '0');
    signal p5_phi : unsigned(15 downto 0) := (others => '0');
    signal p5_b1  : unsigned(16 downto 0) := (others => '0');
    signal p5_b3  : signed(16 downto 0) := (others => '0');
    signal p6_bs  : unsigned(8 downto 0) := (others => '0');
    signal p7_bsm : unsigned(14 downto 0) := (others => '0');
    signal p8_bs, p9_bs, p10_bs : unsigned(7 downto 0) := (others => '0');
    signal p8_pg, p8_pr : unsigned(7 downto 0) := (others => '0');
    signal p8_er  : unsigned(15 downto 0) := (others => '0');          -- envelope / amplitude back from the ring
    signal p9_g0  : unsigned(15 downto 0) := (others => '0');
    signal p9_w   : unsigned(15 downto 0) := (others => '0');
    signal p9_la, p9_lb : unsigned(15 downto 0) := x"ACE1";
    signal p9_in  : signed(8 downto 0) := (others => '0');
    signal p9_nn  : signed(12 downto 0) := (others => '0');
    signal p9_na  : signed(9 downto 0) := (others => '0');
    signal p10_g  : unsigned(7 downto 0) := (others => '0');
    signal p10_ws : signed(12 downto 0) := (others => '0');
    signal p11_r, p11_g, p11_b : unsigned(7 downto 0) := (others => '0');
    signal p12_yr, p12_yg, p12_yb : unsigned(15 downto 0) := (others => '0');
    signal p12_r25, p12_b2, p13_r25, p13_b2 : unsigned(10 downto 0) := (others => '0');
    signal p13_ys : unsigned(15 downto 0) := (others => '0');
    signal p14_y, p14_cb, p14_cr : unsigned(9 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- block-RAM delay rings (collision-free: the read pointer trails the write pointer)
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal rg_e, rg_a1, rg_a2 : t_ring := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of rg_e : signal is true;
    attribute no_rw_check of rg_a1 : signal is true;
    attribute no_rw_check of rg_a2 : signal is true;
    signal rg_w   : unsigned(7 downto 0) := (others => '0');
    signal rg_e_q, rg_a1_q, rg_a2_q : std_logic_vector(15 downto 0) := (others => '0');
    signal rg_a1_f, rg_a2_f : std_logic_vector(15 downto 0) := (others => '0');

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Line buffer and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_UN_EXP2(to_integer(r_ex_a));
            r_cs_q <= C_UN_COS(to_integer(r_cs_a));
            t_rq   <= C_UN_TAB(to_integer(t_ra));
            m_rq   <= C_UN_RAM(to_integer(t_ra));
            pg_q   <= C_UN_PG(to_integer(pg_a));
            pr_q   <= C_UN_PR(to_integer(pr_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_t  : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

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
            elsif s_zc /= 2047 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_flip <= registers_in(6)(0);
                s_pal  <= registers_in(6)(1);
                s_wob  <= registers_in(6)(2);
                s_sq   <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(496, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is
                when 0          => s_oys <= to_unsigned(537, 10);
                when 4          => s_oys <= to_unsigned(272, 10);
                when 8          => s_oys <= to_unsigned(453, 10);
                when 12         => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10);
                when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others     => s_oys <= to_unsigned(121, 10);
            end case;

            -- shared glide (1/8 of the gap per field, +-2 deadband)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k decoded at zc = 30 + 24 k, operands loaded two clocks later, the product
            -- read at zc = 50 + 24 k
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 486 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_a(9 downto 0), 28);     z_fb <= to_unsigned(338, 16);     -- |k1 - 512| * 338
                    when 1  => z_fa <= resize(s_gp(12 downto 3), 28);   z_fb <= to_unsigned(141, 16);     -- U
                    when 2  => z_fa <= resize(c_n10, 28);               z_fb <= resize(c_u8, 16);         -- N U
                    when 3  => z_fa <= to_unsigned(4926, 28);           z_fb <= resize(c_n10, 16);        -- sxh
                    when 4  => z_fa <= to_unsigned(12373, 28);          z_fb <= resize(c_n10, 16);        -- |syl|
                    when 5  => z_fa <= to_unsigned(38073, 28);          z_fb <= resize(c_n10, 16);        -- base fraction
                    when 6  => z_fa <= resize(z_a(9 downto 0), 28);     z_fb <= to_unsigned(404, 16);     -- |k2 - 512| * 404
                    when 7  => z_fa <= to_unsigned(291, 28);            z_fb <= z_mt;                     -- fstep
                    when 8  => z_fa <= resize(z_a(9 downto 0), 28);     z_fb <= resize(z_a(9 downto 0), 16); -- d^2 (bands)
                    when 9  => z_fa <= resize(z_a(18 downto 1), 28);    z_fb <= to_unsigned(179, 16);     -- depth (d^2 / 2)
                    when 10 => z_fa <= resize(s_g3(12 downto 3), 28);   z_fb <= resize(s_g3(12 downto 3), 16); -- blue
                    when 11 => z_fa <= resize(s_vm, 28);                z_fb <= resize(s_vm, 16);         -- flow^2
                    when 12 => z_fa <= resize(z_a(17 downto 1), 28);    z_fb <= to_unsigned(1092, 16);    -- flow step (m^2 / 2)
                    when 13 =>                                                                             -- threshold
                        if s_g5(12 downto 3) > 512 then
                            z_fa <= resize(s_g5(12 downto 3) - 512, 28); z_fb <= to_unsigned(108, 16);
                        else
                            z_fa <= resize(512 - s_g5(12 downto 3), 28); z_fb <= to_unsigned(77, 16);
                        end if;
                    when 14 => z_fa <= resize(s_g6(12 downto 3), 28);   z_fb <= resize(s_g6(12 downto 3), 16); -- etch^2
                    when 15 => z_fa <= resize(z_a(9 downto 0), 28);     z_fb <= to_unsigned(771, 16);     -- etch
                    when 16 => z_fa <= resize(c_sxh, 28);               z_fb <= resize(c_xb0, 16);        -- plane at xb0
                    when 17 => z_fa <= to_unsigned(1373, 28);           z_fb <= resize(s_ys, 16);         -- fold per line
                    when 18 => z_fa <= to_unsigned(1079, 28);           z_fb <= resize(s_ys, 16);         -- red offset per line
                    when others => z_fa <= to_unsigned(215, 28);        z_fb <= resize(c_xb0, 16);        -- red offset at xb0
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(26 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20 =>
                        if s_vneg = '1' then c_fl <= c_fl - c_flstep; else c_fl <= c_fl + c_flstep; end if;
                        c_scr <= c_scr + 19;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                        case to_integer(s_xs) is
                            when 4      => c_vsp <= to_unsigned(24, 5); c_dsp <= to_unsigned(860, 10);
                            when 3      => c_vsp <= to_unsigned(18, 5); c_dsp <= to_unsigned(645, 10);
                            when others => c_vsp <= to_unsigned(12, 5); c_dsp <= to_unsigned(430, 10);
                        end case;
                        if s_g1(12 downto 3) >= 512 then z_a <= resize(s_g1(12 downto 3) - 512, 20); z_neg <= '0';
                        else z_a <= resize(512 - s_g1(12 downto 3), 20); z_neg <= '1'; end if;
                    when 21 =>
                        c_q0 <= signed(resize(c_xb0, 13)) - 16;
                        -- 6.03 xb0 (Q14 per h of the fold ramp)
                        c_v0 <= shift_left(resize(c_xb0, 16), 2) + shift_left(resize(c_xb0, 16), 1) + resize(c_xb0(11 downto 5), 16);
                    when 22 =>
                        c_i0 <= f_cell(c_q0);
                    -- slot 0 -> N
                    when 50 =>
                        v_e := signed(resize(f_acc(20 downto 9), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        z_e <= v_e;
                    when 51 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 53 => z_mt <= r_ex_q;
                    when 54 =>
                        case to_integer(z_sh) is
                            when 1      => c_n10 <= z_mt(15 downto 4);
                            when 0      => c_n10 <= '0' & z_mt(15 downto 5);
                            when -1     => c_n10 <= "00" & z_mt(15 downto 6);
                            when others => c_n10 <= "000" & z_mt(15 downto 7);
                        end case;
                    when 74  => c_u8 <= f_acc(17 downto 8);
                    when 98  => c_nu <= f_acc(20 downto 10);
                    when 122 => c_sxh <= f_acc(23 downto 10);
                    when 123 =>
                        case to_integer(s_xs) is
                            when 4      => c_sxp <= shift_left(resize(c_sxh, 16), 2);
                            when 3      => c_sxp <= shift_left(resize(c_sxh, 16), 1) + resize(c_sxh, 16);
                            when others => c_sxp <= shift_left(resize(c_sxh, 16), 1);
                        end case;
                    when 146 => c_syl <= (not resize(f_acc(24 downto 10), 20)) + 1;
                    when 170 =>
                        -- base = -(2.58095 N) mod 1: the fraction 0.58095 N (Q26 >> 6) plus 2 N mod 1
                        v_t := f_acc(25 downto 6) + shift_left(resize(c_n10(8 downto 0), 20), 11);
                        c_base <= (not v_t) + 1;
                        if s_g2(12 downto 3) >= 512 then z_a <= resize(s_g2(12 downto 3) - 512, 20); z_neg <= '0';
                        else z_a <= resize(512 - s_g2(12 downto 3), 20); z_neg <= '1'; end if;
                    when 194 =>
                        v_e := signed(resize(f_acc(20 downto 9), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        z_e <= v_e;
                    when 195 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 197 => z_mt <= r_ex_q;
                    when 218 =>
                        case to_integer(z_sh) is
                            when 1      => c_fstep <= resize(f_acc(27 downto 14), 16);
                            when 0      => c_fstep <= resize(f_acc(27 downto 15), 16);
                            when -1     => c_fstep <= resize(f_acc(27 downto 16), 16);
                            when others => c_fstep <= resize(f_acc(27 downto 17), 16);
                        end case;
                    when 242 => z_a <= f_acc(19 downto 0);
                    when 266 => c_depth <= to_unsigned(77, 9) + resize(f_acc(25 downto 17), 9);
                    when 290 => c_bg <= f_acc(19 downto 12);
                    when 314 => z_a <= f_acc(19 downto 0);
                    when 338 => c_flstep <= resize(f_acc(27 downto 17), 16);
                    when 362 =>
                        if s_g5(12 downto 3) > 512 then c_thr <= to_signed(141, 9) - signed(resize(f_acc(16 downto 9), 9));
                        else c_thr <= to_signed(141, 9) + signed(resize(f_acc(16 downto 9), 9)); end if;
                    when 386 => z_a <= resize(f_acc(19 downto 10), 20);
                    when 410 => c_eq <= f_acc(19 downto 10);
                    when 434 => c_px0 <= f_acc(19 downto 0);
                    when 458 => c_vsl <= f_acc(20 downto 8);
                    when 482 => c_dql <= f_acc(20 downto 12);
                    when 506 => c_dq0 <= f_acc(16 downto 0);
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
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
    -- rise, field-latched width)
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
    -- Source side: oscillator lattice, line-buffer write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line's constants on one serial multiplier (24-bit,
    -- every product used mod 2^24 or below)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_vy  : unsigned(10 downto 0);
        variable v_j   : unsigned(5 downto 0);
        variable v_ym  : unsigned(9 downto 0);
        variable v_w   : signed(16 downto 0);
        variable v_pj  : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                n_vs <= (not to_unsigned(1482910, 22)) + 1;      -- v at (X, Y) = (-1, -1): -sqrt2
                n_uv <= to_unsigned(1482910, 22);                -- u - v at Y = -1: +sqrt2
                n_dq <= to_unsigned(350979, 20);                 -- red offset at (X, Y) = (-1, -1): 0.335 cycle
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                n_vs <= n_vs + resize(c_vsl, 22);
                n_uv <= n_uv - shift_left(resize(c_vsl, 22), 1);
                n_dq <= n_dq - resize(c_dql, 20);
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(22 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_vy := e_vy(18 downto 8);
                    if v_vy > 1079 then v_vy := to_unsigned(1079, 11); end if;
                    e_vyi <= v_vy;
                    if s_flip = '1' then e_vyf <= to_unsigned(1079, 11) - v_vy; else e_vyf <= v_vy; end if;
                    e_sl <= resize(v_vy, 12) + resize(c_scr(15 downto 4), 12);
                    n_pj <= n_pj + resize(shift_right(resize(e_pjt, 10) - n_pj, 3), 10);
                when 2 =>
                    v_j := e_vyf(10 downto 5);
                    if v_j > 33 then v_j := to_unsigned(33, 6); end if;
                    e_j <= v_j;
                    if v_j >= 33 then e_j1 <= to_unsigned(33, 6); else e_j1 <= v_j + 1; end if;
                    n_fy <= e_vyf(4 downto 0);
                    n_nf <= e_sl(2 downto 0);
                    m_a <= resize(c_syl, 24); m_b <= resize(e_vyf, 16); m_acc <= (others => '0');
                when 3 =>
                    n_ja <= shift_left(resize(e_j, 11), 6) - shift_left(resize(e_j, 11), 2);
                    n_jb <= shift_left(resize(e_j1, 11), 6) - shift_left(resize(e_j1, 11), 2);
                when 19 =>
                    n_L <= c_base + m_acc(19 downto 0);
                when 20 =>
                    m_a <= resize(c_fstep, 24); m_b <= resize(e_vyi, 16); m_acc <= (others => '0');
                when 37 =>
                    e_th <= m_acc(15 downto 0) - to_unsigned(48933, 16);
                when 38 =>
                    r_cs_a <= e_th(15 downto 8);
                when 41 =>
                    e_cosv <= r_cs_q;
                    r_cs_a <= e_th(15 downto 8) - 64;
                when 42 =>
                    if e_cosv < 0 then m_b <= resize(unsigned(-e_cosv), 16); else m_b <= resize(unsigned(e_cosv), 16); end if;
                    m_a <= resize(c_depth, 24); m_acc <= (others => '0');
                when 44 =>
                    e_sinv <= r_cs_q;
                when 45 =>
                    -- wobble: sin * 0.5 cycle (Q16: sinv * 258)
                    if s_wob = '1' then
                        e_wob <= shift_left(resize(e_sinv, 17), 8) + shift_left(resize(e_sinv, 17), 1);
                    else
                        e_wob <= (others => '0');
                    end if;
                when 46 =>
                    v_w := signed(resize(c_fl, 17)) + e_wob;
                    n_L <= n_L + shift_left(resize(unsigned(v_w(15 downto 0)), 20), 4);
                when 59 =>
                    v_ym := to_unsigned(256, 10);
                    if e_cosv < 0 then v_ym := v_ym - resize(m_acc(15 downto 7), 10);
                    else v_ym := v_ym + resize(m_acc(15 downto 7), 10); end if;
                    n_ymod <= v_ym(9 downto 4);
                when 60 | 96 =>
                    -- noise seeds of the next line's 8-line group and the group after: hash16(group + 1445 (+ 1))
                    if e_lc(6) = '1' then
                        m_a <= resize(resize(e_sl(11 downto 3), 12) + 1446, 24);
                    else
                        m_a <= resize(resize(e_sl(11 downto 3), 12) + 1445, 24);
                    end if;
                    m_b <= to_unsigned(16#9E37#, 16); m_acc <= (others => '0');
                when 77 | 113 =>
                    e_hs <= m_acc(15 downto 0) + x"1234";
                when 78 | 114 =>
                    m_a <= resize(e_hs xor (x"00" & e_hs(15 downto 8)), 24); m_b <= to_unsigned(16#5BD1#, 16); m_acc <= (others => '0');
                when 95 =>
                    n_seed <= m_acc(15 downto 1) & '1';
                    -- the red jitter's target: the seed's top byte (a new target per 8-line group)
                    e_pjt <= signed(m_acc(15 downto 8) xor x"80");
                when 131 =>
                    n_seedb <= m_acc(15 downto 1) & '1';
                when 133 =>
                    n_pxs <= n_L + c_px0;
                    n_dqs <= n_dq + resize(c_dq0, 20);
                when 134 =>
                    -- jitter into the red offset: 20 pj (Q16, up to +-0.04 cycle), << 4 -> Q20
                    v_pj := shift_left(resize(n_pj, 16), 4) + shift_left(resize(n_pj, 16), 2);
                    n_dqs <= n_dqs + shift_left(resize(unsigned(v_pj), 20), 4);
                when others => null;
            end case;

            -- hblank: load the prefetch constants for the next line; the prefetch fetches its first two nodes
            if (s_gav_sr(0) = '1' and g_avid = '0') or (s_zone = '1' and s_zc = 1420) then
                y_cnt <= to_unsigned(1, 6);
            elsif y_cnt /= 0 then
                y_cnt <= y_cnt + 1;
            end if;
            if y_cnt = 12 then
                l_ja <= n_ja; l_jb <= n_jb; l_fy <= n_fy; l_ymod <= n_ymod;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Prefetch unit: one node (two table rows) in 13 clocks into lane P, with
    -- the slope from lane B to it; the lanes shift at every cell entry (and
    -- twice in hblank, after the line's first two nodes)
    ---------------------------------------------------------------------------
    p_pf : process(clk)
        variable v_n  : unsigned(6 downto 0);
        variable v_go : std_logic;
        variable v_f2 : unsigned(1 downto 0);
        variable v_e10 : signed(9 downto 0);
        variable v_m  : signed(25 downto 0);
        variable v_p  : unsigned(21 downto 0);
    begin
        if rising_edge(clk) then
            v_go := '0'; v_n := b_reqn;
            if b_req = '1' then
                v_go := '1';
            elsif y_cnt = 14 then
                v_go := '1'; v_n := resize(c_i0, 7);
            elsif y_cnt = 29 then
                v_go := '1'; v_n := resize(c_i0, 7) + 1;
            end if;
            if v_go = '1' then
                pf_c <= to_unsigned(1, 4);
                if v_n > 59 then pf_nc <= to_unsigned(59, 6); else pf_nc <= v_n(5 downto 0); end if;
            elsif pf_c /= 0 then
                if pf_c = 13 then pf_c <= (others => '0'); else pf_c <= pf_c + 1; end if;
            end if;

            v_f2 := l_fy(4 downto 3);
            case to_integer(pf_c) is
                when 1 =>
                    t_ra <= l_ja + resize(pf_nc, 11);
                when 2 =>
                    t_ra <= l_jb + resize(pf_nc, 11);
                when 3 =>
                    pf_d0 <= signed(t_rq(15 downto 4)); pf_e0 <= unsigned(t_rq(3 downto 0)); pf_r0 <= m_rq;
                when 4 =>
                    pf_d1 <= signed(t_rq(15 downto 4)); pf_e1 <= unsigned(t_rq(3 downto 0)); pf_r1 <= m_rq;
                when 5 =>
                    pf_dd <= resize(pf_d1, 13) - resize(pf_d0, 13);
                    pf_de <= signed(resize(pf_e1, 5)) - signed(resize(pf_e0, 5));
                    pf_dr <= signed(resize(pf_r1, 5)) - signed(resize(pf_r0, 5));
                when 6 =>
                    -- D: 5-bit line fraction; E and R: its top two bits (8-line steps)
                    pf_pd <= resize(pf_dd * signed(resize(l_fy, 6)), 18);
                    case to_integer(v_f2) is
                        when 0      => pf_pe <= (others => '0'); pf_pr <= (others => '0');
                        when 1      => pf_pe <= resize(pf_de, 7); pf_pr <= resize(pf_dr, 7);
                        when 2      => pf_pe <= shift_left(resize(pf_de, 7), 1); pf_pr <= shift_left(resize(pf_dr, 7), 1);
                        when others => pf_pe <= shift_left(resize(pf_de, 7), 1) + resize(pf_de, 7);
                                       pf_pr <= shift_left(resize(pf_dr, 7), 1) + resize(pf_dr, 7);
                    end case;
                when 7 =>
                    pf_vd <= shift_left(resize(pf_d0, 15), 2) + resize(shift_right(pf_pd, 3), 15);
                    v_e10 := signed(shift_left(resize(pf_e0, 10), 5)) + shift_left(resize(pf_pe, 10), 3);
                    if v_e10 < 0 then pf_ve <= (others => '0'); else pf_ve <= unsigned(v_e10(8 downto 0)); end if;
                    v_e10 := signed(shift_left(resize(pf_r0, 10), 5)) + shift_left(resize(pf_pr, 10), 3);
                    if v_e10 < 0 then pf_vr <= (others => '0'); else pf_vr <= unsigned(v_e10(8 downto 0)); end if;
                    pf_acc <= (others => '0'); pf_nu <= '0' & c_nu;
                when 8 | 9 | 10 | 11 =>
                    -- envelope: Q5 node units (16 G levels per unit) x ymod (Q4) -> G units: >> 5
                    pf_ve2 <= pf_ve * l_ymod;
                    -- red amplitude: Q5 node units (1/8 amp per unit) = Q8 amp; x ymod (Q4) -> Q8: >> 4
                    pf_vr2 <= pf_vr * l_ymod;
                    -- D x N U: three bits of N U per clock, most significant first (Q9 x Q8 = Q17)
                    v_m := (others => '0');
                    if pf_nu(11) = '1' then v_m := v_m + shift_left(resize(pf_vd, 26), 2); end if;
                    if pf_nu(10) = '1' then v_m := v_m + shift_left(resize(pf_vd, 26), 1); end if;
                    if pf_nu(9) = '1' then v_m := v_m + resize(pf_vd, 26); end if;
                    pf_acc <= shift_left(pf_acc, 3) + v_m;
                    pf_nu <= pf_nu(8 downto 0) & "000";
                when 12 =>
                    v_p := unsigned(pf_acc(22 downto 1));
                    lp_d <= v_p; lp_e <= pf_ve2(13 downto 5); lp_r <= pf_vr2(13 downto 4);
                    lp_dd <= v_p - lb_d;
                when 13 =>
                    case to_integer(s_xs) is
                        when 4      => lp_ddx <= shift_left(lp_dd, 2);
                        when 3      => lp_ddx <= shift_left(lp_dd, 1) + lp_dd;
                        when others => lp_ddx <= shift_left(lp_dd, 1);
                    end case;
                when others => null;
            end case;

            -- lane shift at a cell entry (the clock the beam restarts the D lane from lane B), or in hblank
            if (g_avid = '1' and b_ce = '1') or y_cnt = 28 or y_cnt = 44 then
                la_d <= lb_d; la_e <= lb_e; la_r <= lb_r;
                lb_d <= lp_d; lb_e <= lp_e; lb_r <= lp_r;
            end if;
        end if;
    end process p_pf;

    ---------------------------------------------------------------------------
    -- Beam: the stage-1 pixel values (table coordinate, plane, red offset,
    -- fold ramp) as forward DDAs; cell entries restart the D lane at lane B
    -- with lane P's slope (exact at the cell's first h; the 0 .. 2 h of a
    -- 720p entry is a sub-pixel shift)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_q : signed(12 downto 0);
        variable v_i : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            b_req <= '0';
            if g_start = '1' then
                b_px <= (others => '0');
                b_q1 <= c_q0;
                b_i1 <= c_i0;
                b_ce <= '0';
                b_pl <= n_pxs;
                b_dq <= n_dqs;
                b_v  <= n_vs(21 downto 6) + c_v0;
                b_uv <= n_uv(21 downto 12);
                b_da <= shift_left(la_d, 6);
                b_ddx <= lp_ddx;
                b_req <= '1'; b_reqn <= resize(c_i0, 7) + 2;
                b_seed <= n_seed; b_seedb <= n_seedb; b_nf <= n_nf;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                v_q := b_q1 + signed(resize(s_xs, 13));
                b_q1 <= v_q;
                b_pl <= b_pl + resize(c_sxp, 20);
                b_dq <= b_dq + resize(c_dsp, 20);
                b_v  <= b_v + resize(c_vsp, 16);
                v_i := f_cell(v_q);
                if v_i /= b_i1 then b_ce <= '1'; else b_ce <= '0'; end if;
                b_i1 <= v_i;
                if b_ce = '1' then
                    b_da <= shift_left(lb_d, 6);
                    b_ddx <= lp_ddx;
                    b_req <= '1'; b_reqn <= resize(b_i1, 7) + 2;
                else
                    b_da <= b_da + b_ddx;
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam registers and lanes)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_i   : unsigned(5 downto 0);
        variable v_fr  : signed(7 downto 0);
        variable v_f2  : signed(2 downto 0);
        variable v_e10 : signed(9 downto 0);
        variable v_r11 : signed(10 downto 0);
        variable v_av  : signed(10 downto 0);
        variable v_b   : signed(17 downto 0);
        variable v_r5  : signed(15 downto 0);
        variable v_bb  : signed(9 downto 0);
        variable v_r16 : unsigned(15 downto 0);
        variable v_y   : unsigned(11 downto 0);
        variable v_c   : signed(11 downto 0);
        variable v_na, v_nb : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: envelope / amplitude interpolation products (2-bit fraction; at a cell entry the lanes
            --    have not shifted yet, so the new cell's node is lane B and the fraction is 0); fold v
            v_i  := f_cell(b_q1);
            v_fr := resize(b_q1 - signed(shift_left(resize(v_i, 13), 6)), 8);
            if v_fr < 0 then v_f2 := (others => '0'); else v_f2 := signed('0' & std_logic_vector(v_fr(5 downto 4))); end if;
            p2_pe <= resize(v_f2 * (signed(resize(lb_e, 10)) - signed(resize(la_e, 10))), 12);
            p2_pr <= resize(v_f2 * (signed(resize(lb_r, 11)) - signed(resize(la_r, 11))), 13);
            if b_ce = '1' then p2_ae <= lb_e; p2_ar <= lb_r; else p2_ae <= la_e; p2_ar <= la_r; end if;
            p2_v  <= signed(b_v(15 downto 6));
            p2_uv <= signed(b_uv);
            -- 3: the interpolated values (into the ring, back at 8); |v|; u; luma
            v_e10 := signed(resize(p2_ae, 10)) + resize(shift_right(p2_pe, 2), 10);
            if v_e10 < 0 then p3_eq <= (others => '0'); else p3_eq <= unsigned(v_e10(8 downto 0)); end if;
            v_r11 := signed(resize(p2_ar, 11)) + resize(shift_right(p2_pr, 2), 11);
            if v_r11 < 0 then p3_rq <= (others => '0'); else p3_rq <= unsigned(v_r11(9 downto 0)); end if;
            v_av := abs(resize(p2_v, 11));
            p3_av <= unsigned(v_av(8 downto 0));
            p3_u  <= p2_v + p2_uv;
            p3_lm <= unsigned(s_lb_q);
            -- 4: plane + D (the beam's registers, 2 px late: a constant shear); etch (Q12) x luma
            p4_ph <= b_pl(19 downto 4) + b_da(21 downto 6);
            p4_el <= c_eq(9 downto 4) * p3_lm;
            p4_av <= p3_av; p4_u <= p3_u;
            -- 5: the phase; fold terms 9216 + 144 |v| and 80 u
            p5_phi <= p4_ph + shift_left(resize(p4_el, 16), 4);
            p5_b1 <= to_unsigned(9216, 17) + shift_left(resize(p4_av, 17), 7) + shift_left(resize(p4_av, 17), 4);
            p5_b3 <= shift_left(resize(p4_u, 17), 6) + shift_left(resize(p4_u, 17), 4);
            -- 6: ROM addresses (Wave picks the pulse tables' half; the red offset from the beam, 4 px late); fold
            pg_a  <= s_sq & p5_phi(15 downto 8);
            v_r16 := p5_phi + b_dq(19 downto 4);
            pr_a  <= s_sq & v_r16(15 downto 8);
            v_b := signed(resize(p5_b1, 18)) + resize(p5_b3, 18);
            if v_b < 0 then p6_bs <= (others => '0'); else p6_bs <= unsigned(v_b(16 downto 8)); end if;
            -- 7: ROM data (EBR); fold x Blue (Q4)
            p7_bsm <= p6_bs * c_bg(7 downto 2);
            -- 8: fabric copies; blue level clipped
            p8_pg <= pg_q; p8_pr <= pr_q; p8_er <= unsigned(rg_e_q);
            if p7_bsm(14 downto 12) /= "000" then p8_bs <= (others => '1'); else p8_bs <= p7_bsm(11 downto 4); end if;
            -- 9: envelope x green pulse, amplitude x red pulse; the fleck noise: two LFSR streams seeded with
            --    the line's 8-line group and the next, each smoothed along the line (decay 1/16), blended by
            --    the line's place in the group (smooth in both directions; the blend runs a clock behind)
            p9_g0 <= p8_er(15 downto 8) * p8_pg;
            p9_w  <= p8_er(7 downto 0) * p8_pr;
            if s_gav_sr(7) = '1' and s_gav_sr(8) = '0' then
                p9_la <= b_seed; p9_lb <= b_seedb;
                p9_nn <= (others => '0');
            else
                p9_la <= p9_la(14 downto 0) & (p9_la(15) xor p9_la(13) xor p9_la(12) xor p9_la(10));
                p9_lb <= p9_lb(14 downto 0) & (p9_lb(15) xor p9_lb(13) xor p9_lb(12) xor p9_lb(10));
                p9_nn <= p9_nn + resize(p9_in, 13) - shift_right(p9_nn, 4);
            end if;
            -- the two streams' bytes blended by the line's place in the group (quarter steps); the IIR is
            -- linear, so this equals blending the two smoothed streams
            v_na := signed(resize(signed(p9_la(7 downto 0) xor x"80"), 9));
            v_nb := signed(resize(signed(p9_lb(7 downto 0) xor x"80"), 9)) - v_na;
            case to_integer(b_nf(2 downto 1)) is
                when 0      => p9_in <= v_na;
                when 1      => p9_in <= v_na + shift_right(v_nb, 2);
                when 2      => p9_in <= v_na + shift_right(v_nb, 1);
                when others => p9_in <= v_na + shift_right(v_nb, 1) + shift_right(v_nb, 2);
            end case;
            p9_na <= resize(shift_right(p9_nn, 3), 10);
            p9_bs <= p8_bs;
            -- 10: green, comparator sum
            if p9_g0(15) = '1' then p10_g <= (others => '1'); else p10_g <= p9_g0(14 downto 7); end if;
            p10_ws <= signed(resize(p9_w(15 downto 6), 13)) + resize(p9_na, 13) - resize(c_thr, 13);
            p10_bs <= p9_bs;
            -- 11: red = clip(5 ws), blue = fold - green / 4; palette (Ember = the cables swapped)
            v_r5 := shift_left(resize(p10_ws, 16), 2) + resize(p10_ws, 16);
            v_bb := signed(resize(p10_bs, 10)) - signed(resize(p10_g(7 downto 2), 10));
            if v_bb < 0 then v_bb := (others => '0'); end if;
            if s_pal = '1' then
                p11_r <= p10_g; p11_g <= unsigned(v_bb(7 downto 0)); p11_b <= f_clip8(v_r5);
            else
                p11_r <= f_clip8(v_r5); p11_g <= p10_g; p11_b <= unsigned(v_bb(7 downto 0));
            end if;
            -- 12: luma partials 80 R, 144 G, 32 B (BT.601 within 4 %); 2.5 R + 512, 2 B + 512
            p12_yr <= shift_left(resize(p11_r, 16), 6) + shift_left(resize(p11_r, 16), 4);
            p12_yg <= shift_left(resize(p11_g, 16), 7) + shift_left(resize(p11_g, 16), 4);
            p12_yb <= shift_left(resize(p11_b, 16), 5);
            p12_r25 <= shift_left(resize(p11_r, 11), 1) + resize(p11_r(7 downto 1), 11) + to_unsigned(512, 11);
            p12_b2  <= shift_left(resize(p11_b, 11), 1) + to_unsigned(512, 11);
            -- 13: Y' * 256
            p13_ys <= p12_yr + p12_yg + p12_yb;
            p13_r25 <= p12_r25; p13_b2 <= p12_b2;
            -- 14: Y = 64 + 4.25 Y' (capped 940), Cb = 512 + 2 (B - Y'), Cr = 512 + 2.5 (R - Y'), clipped 24 .. 1000
            v_y := to_unsigned(64, 12) + shift_left(resize(p13_ys(15 downto 8), 12), 2) + resize(p13_ys(15 downto 10), 12);
            if v_y > 940 then p14_y <= to_unsigned(940, 10); else p14_y <= v_y(9 downto 0); end if;
            v_c := signed(resize(p13_b2, 12)) - signed(shift_left(resize(p13_ys(15 downto 8), 12), 1));
            if v_c < 24 then p14_cb <= to_unsigned(24, 10); elsif v_c > 1000 then p14_cb <= to_unsigned(1000, 10); else p14_cb <= unsigned(v_c(9 downto 0)); end if;
            v_c := signed(resize(p13_r25, 12)) - signed(shift_left(resize(p13_ys(15 downto 8), 12), 1)) - signed(resize(p13_ys(15 downto 9), 12));
            if v_c < 24 then p14_cr <= to_unsigned(24, 10); elsif v_c > 1000 then p14_cr <= to_unsigned(1000, 10); else p14_cr <= unsigned(v_c(9 downto 0)); end if;
            -- 15: compose (u = Cr, v = Cb); black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= C_KY; s_u15 <= C_KC; s_v15 <= C_KC;
            else
                s_y15 <= p14_y; s_u15 <= p14_cr; s_v15 <= p14_cb;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Block-RAM delay rings: the envelope / amplitude pair from stage 3 to 8
    -- (read pointer = write - 3, then the RAM's output register) and the output
    -- box's 8-back tap (write - 6, RAM register, fabric register)
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rg_w <= rg_w + 1;
            rg_e(to_integer(rg_w)) <= std_logic_vector(p3_eq(8 downto 1)) & std_logic_vector(p3_rq(9 downto 2));
            rg_e_q <= rg_e(to_integer(rg_w - 3));
            rg_a1(to_integer(rg_w)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15(9 downto 4));
            rg_a2(to_integer(rg_w)) <= std_logic_vector(s_u15(3 downto 0)) & std_logic_vector(s_v15) & "00";
            rg_a1_q <= rg_a1(to_integer(rg_w - 6)); rg_a2_q <= rg_a2(to_integer(rg_w - 6));
            rg_a1_f <= rg_a1_q; rg_a2_f <= rg_a2_q;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box
    -- (two adjacent 8-sums; the HD HDMI bar fix for the flecks' pixel-rate
    -- chroma edges), luma 13/16 and the blanking gate.  The running 8-sums snap
    -- back to black once 8 blank samples are in (resync).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box: stage 16 sums, stage 17 output (black off the generated avid)
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KC & "000"; s_bav <= C_KC & "000";
            else
                s_bay <= s_bay + s_y15 - unsigned(rg_a1_f(15 downto 6));
                s_bau <= s_bau + s_u15 - unsigned(std_logic_vector'(rg_a1_f(5 downto 0) & rg_a2_f(15 downto 12)));
                s_bav <= s_bav + s_v15 - unsigned(rg_a2_f(11 downto 2));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= C_KY; s_u17 <= C_KC; s_v17 <= C_KC;
            end if;
            -- 18, 19: delay (one latency, C_LAT 20, as the two-box shell)
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture undulate;
