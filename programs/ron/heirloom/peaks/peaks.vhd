-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Peaks v0.1.0 -- Heirloom (video clips), from WIN_20230210_01_00_21 at 3.0 s.
--
-- The still: a sunset of two yellow peaks over a sky of diagonal peach/magenta bands and a green/violet
-- weave, reflected in water that ripples line by line, with a few H-sync slips smearing the sky.
--
-- The patch, as fitted (peaks.py has every table; .heirloom_work/rb_peaks/notes.md the measurements):
--   ramp   = a slightly bent H ramp (the band spacing grows 16 % across the line, as an analog ramp does)
--   band   = VCO1, a saw rising over 7/8 of its cycle, 2.03 cycles per width, leaning -0.7 px per line;
--            colourised mauve / magenta / pink / peach; its palette index drops 1.4 per frame height going
--            down and its key threshold rises 1.05, so the peach is gone by line ~210 and the band by ~470
--   weave  = VCO2, 8.4 cycles per width, leaning +0.6 px per line with a +-20 px wobble; index
--            0.24 (1 - cos) rising 0.76 per frame down the sky (green / violet at the top, violet /
--            magenta at the horizon); it shows wherever the band's key is low
--   sun    = VCO1's triangle output (the clip's own sun VCO ran 3 % slower; here the suns sit 0.175 cycle
--            before the band's trough and do not lean): keyed where 0.8 (y - 395) px exceeds the distance
--            from the trough (apex rounded, r = 60), a 25 px soft edge and a 130 px yellow core
--   water  = from line 636 the sky's per-line values hold (S8 Hold) or run backwards (Mirror); each line
--            is shifted by a sawtooth ripple (~36 lines, 172 px, jittered) and the reflected sun's yellow
--            core breathes
--   slips  = common H slips at lines 85 (+60, glided), 316 (-20), 408 (-20), 468 (-30), 576 (+47), each
--            with smeared rows where the oscillators pause for a stretch of the line
--   video  = the camera's luma (or the Osc lattice) displaces both oscillators' phases
--
--   K1  Bands     band VCO frequency (and so the suns'), x1/4 .. x4 (2.03 cycles per width at 512)
--   K2  Curtains  weave VCO frequency, x1/4 .. x4 (8.4 at 512)
--   K3  Lean      bipolar: the vertical LFO into both VCOs (0 = reversed, 512 = vertical, 700 = the clip)
--   K4  Picture   how far the camera (or the Osc) displaces the sky, 0 .. 300 px (square law)
--   K5  Peaks     the suns' apex: at the horizon (0) .. line 395 (400, the clip) .. the top of the frame
--   K6  Tint      bipolar: blue / cyan sky (left, the clip at 0.5 s) .. the still .. pink / orange (right, 12.5 s)
--   S7  Tears     the H-sync slips and smears On / Off
--   S8  Water     Hold (the clip: the horizon line replayed) / Mirror (a true rippled reflection)
--   S9  Suns      Two / Four
--   S10 Palette   Sunset (the clip) / Night (blue sky, teal weave, white moons)
--   S11 Source    Video / Osc
--   P12 STORM     0 = glassy water, no slips; 400 = the clip; 100 % = 400 px teeth, chaos, 3x slips
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glides), a per-line engine kicked at each
-- line start (computes the NEXT line's start phases, key ramps, ripple and smear window on its own serial
-- multiplier), two Q24 phase DDAs with a decrementing increment (the bend), a 14-stage pixel path, the
-- chroma box's delay line in a block-RAM ring.
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (luma
-- [1 2 1], U and V through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum resync, luma 13/16) before
-- the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.peaks_pkg.all;
use work.all;

architecture peaks of program_top is
    constant C_LAT : integer := 18;   -- = processing_delay_clks

    -- fitted constants (phases Q24 cycles)
    constant C_PB0 : unsigned(23 downto 0) := x"AC9561";   -- band phase at x = 0, y = 0 (peak at 1136 px)
    constant C_PW0 : unsigned(23 downto 0) := x"37047E";   -- weave phase (green at 407 px)
    constant C_SO  : unsigned(23 downto 0) := x"D32558";   -- sun trough: band phase - 0.175 (trough at 474 px)
    constant C_HZ  : unsigned(18 downto 0) := to_unsigned(162816, 19);      -- horizon, Q8 virtual lines (636)
    constant C_HZ2 : unsigned(18 downto 0) := to_unsigned(325632, 19);      -- 2 * horizon
    constant C_KT0 : unsigned(10 downto 0) := to_unsigned(594, 11);         -- band key threshold at line 0 (Q10)

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
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    -- blend step on 8-bit colours: w + (d * a) / 4, a = 0 .. 4, d = b - w.  Computed mod 256 with no clamp:
    -- every input is a palette byte in 6 .. 250 (luma 16 .. 235, chroma clamped 6 .. 250, the tint saturates
    -- at 250), so the exact result stays in 5 .. 250 and the 8-bit wrap never happens.
    function f_mix(w : unsigned(7 downto 0); d : signed(9 downto 0); a : unsigned(2 downto 0)) return unsigned is
        variable v_s1, v_s2 : signed(9 downto 0);
        variable v_h, v_q, v_t : unsigned(7 downto 0);
    begin
        v_s1 := shift_right(d, 1); v_s2 := shift_right(d, 2);
        v_h := unsigned(v_s1(7 downto 0)); v_q := unsigned(v_s2(7 downto 0));
        case to_integer(a) is
            when 0      => v_t := (others => '0');
            when 1      => v_t := v_q;
            when 2      => v_t := v_h;
            when 3      => v_t := v_h + v_q;
            when others => v_t := unsigned(d(7 downto 0));
        end case;
        return w + v_t;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_ws_a : unsigned(7 downto 0) := (others => '0');
    signal r_ws_q : unsigned(7 downto 0) := (others => '0');
    signal r_sd_a : unsigned(7 downto 0) := (others => '0');
    signal r_sd_q : unsigned(9 downto 0) := (others => '0');
    signal r_bd_a, r_wv_a, r_sn_a : unsigned(8 downto 0) := (others => '0');
    signal r_bd_q, r_wv_q, r_sn_q : std_logic_vector(23 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
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
    signal s_mx   : unsigned(8 downto 0) := to_unsigned(256, 9);    -- 512 / xs (vpx -> output px, Q8)
    signal s_mx2  : unsigned(8 downto 0) := to_unsigned(256, 9);    -- 65536 / ys (vlines -> output lines, Q8)
    signal s_c699 : unsigned(13 downto 0) := to_unsigned(2796, 14); -- 699 * xs^2
    signal s_xs16 : unsigned(6 downto 0) := to_unsigned(32, 7);     -- 16 * xs
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_fc   : unsigned(7 downto 0) := to_unsigned(64, 8);    -- field counter (sway, breath drift); 64 = sway 0
    signal s_fc2  : unsigned(7 downto 0) := (others => '0');       -- chaos LFO counter 0 .. 149

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(700 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(320 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_tears, s_mirror, s_twin, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_sh2  : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_mt2  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal z_k3   : unsigned(9 downto 0) := (others => '0');         -- |k3 - 512| - 8
    signal z_k3n  : std_logic := '0';                                -- k3 < 512
    signal z_k6   : unsigned(9 downto 0) := (others => '0');
    signal z_k6n  : std_logic := '0';
    signal z_p    : unsigned(9 downto 0) := (others => '0');         -- p12 - 400 (0 below)
    signal z_hi   : std_logic := '0';                                -- p12 > 400
    signal z_rs   : unsigned(15 downto 0) := (others => '0');        -- ripple step per vline, Q8 h
    signal z_pv   : unsigned(7 downto 0) := to_unsigned(30, 8);      -- ripple period, vlines
    signal z_jit  : unsigned(8 downto 0) := (others => '0');         -- jitter, Q8
    signal z_gn   : unsigned(9 downto 0) := to_unsigned(256, 10);    -- slip gain, Q8
    signal c_fb   : unsigned(17 downto 0) := to_unsigned(8869, 18);  -- VCO frequencies, Q24 cycles per h
    signal c_fw   : unsigned(17 downto 0) := to_unsigned(36788, 18);
    signal c_inb, c_inw : unsigned(23 downto 0) := (others => '0');          -- Q28 per output px
    signal c_dnb, c_dnw : unsigned(10 downto 0) := (others => '0');          -- increment decrement per px (bend)
    signal c_lbv, c_lwv : unsigned(19 downto 0) := (others => '0');          -- lean, Q24 phase per vline (magnitudes)
    signal c_lbn, c_lwn : std_logic := '0';                                  -- lean signs (1 = negative)
    signal c_dep  : unsigned(12 downto 0) := (others => '0');                -- displacement, Q20 phase per luma step
    signal c_hw0  : signed(18 downto 0) := to_signed(-161792, 19);           -- sun half-width at line 0, Q8 h
    signal c_tcr, c_tcb : unsigned(7 downto 0) := (others => '0');           -- tint offsets (8-bit colour units)
    signal c_rst  : unsigned(15 downto 0) := (others => '0');                -- ripple step per output line, Q8 h
    signal c_rpr  : unsigned(7 downto 0) := to_unsigned(30, 8);              -- ripple period, output lines
    signal c_jpr  : unsigned(7 downto 0) := (others => '0');                 -- period jitter range
    type t_sa is array (0 to 4) of unsigned(9 downto 0);
    signal c_sa   : t_sa := (others => (others => '0'));                     -- slip magnitudes, h
    signal c_gst  : unsigned(11 downto 0) := (others => '0');                -- glide step of slip 0 per vline, Q8 h
    signal c_swy  : signed(6 downto 0) := (others => '0');                   -- weave sway, h
    signal c_cha  : std_logic := '0';                                        -- chaos half of the LFO

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
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

    -- line engine
    signal e_lc   : unsigned(8 downto 0) := to_unsigned(511, 9);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');       -- next line, Q8 vlines
    signal e_ysk  : unsigned(18 downto 0) := (others => '0');       -- its sky line (hold / mirror)
    signal e_yv   : unsigned(10 downto 0) := (others => '0');       -- sky line, whole vlines
    signal e_wat  : std_logic := '0';
    signal e_wob  : signed(7 downto 0) := (others => '0');          -- weave wobble, h
    signal e_st   : signed(11 downto 0) := (others => '0');         -- common slip (steps), h
    signal e_gl   : unsigned(5 downto 0) := (others => '0');        -- glide rows of slip 0
    signal e_sh   : signed(12 downto 0) := (others => '0');         -- shift of the line, h (+ = sampled further right)
    signal e_shw  : signed(12 downto 0) := (others => '0');         -- the weave's shift
    signal e_sm   : unsigned(2 downto 0) := to_unsigned(5, 3);      -- smear event (5 = none)
    signal e_dy   : unsigned(4 downto 0) := (others => '0');        -- rows into the smear
    signal e_dx   : unsigned(4 downto 0) := (others => '0');        -- |x0 drift| per row, vpx
    signal e_dxn  : std_logic := '0';
    signal e_x0v  : unsigned(11 downto 0) := (others => '0');       -- hold start, vpx
    signal e_lv   : unsigned(11 downto 0) := (others => '0');       -- hold length, vpx
    signal e_br   : signed(9 downto 0) := (others => '0');          -- breath, h
    signal e_rn   : unsigned(7 downto 0) := (others => '0');        -- ripple line counter
    signal e_rp   : unsigned(7 downto 0) := to_unsigned(30, 8);     -- period of the current ripple cycle
    signal e_rpn  : unsigned(7 downto 0) := to_unsigned(30, 8);     -- period of the next cycle
    signal e_rip  : signed(21 downto 0) := (others => '0');         -- ripple of the line, Q8 h
    signal e_ripn : signed(21 downto 0) := (others => '0');         -- reload value of the next cycle
    signal e_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal e_rld  : std_logic := '1';                               -- the next water line starts a cycle
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_lb, n_lw : unsigned(23 downto 0) := (others => '0');
    signal n_phb, n_phw : unsigned(23 downto 0) := (others => '0');
    signal n_so   : unsigned(11 downto 0) := (others => '0');                -- sun trough offset, Q12
    signal n_d1   : unsigned(9 downto 0) := (others => '0');
    signal n_kt   : unsigned(10 downto 0) := C_KT0;
    signal n_d2   : unsigned(7 downto 0) := (others => '0');
    signal n_hw, n_hwc : signed(11 downto 0) := (others => '0');    -- h
    signal n_hx0, n_hl : unsigned(10 downto 0) := (others => '0');
    signal n_hen  : std_logic := '0';
    signal l_so   : unsigned(11 downto 0) := (others => '0');
    signal l_d1   : unsigned(9 downto 0) := (others => '0');
    signal l_kt   : unsigned(10 downto 0) := C_KT0;
    signal l_d2   : unsigned(7 downto 0) := (others => '0');
    signal l_hw, l_hwc : signed(11 downto 0) := (others => '0');
    signal l_hx0, l_hl : unsigned(10 downto 0) := (others => '0');
    signal l_hen  : std_logic := '0';

    -- DDAs (stage 1) and the displacement
    signal b_phb, b_phw : unsigned(23 downto 0) := (others => '0');
    signal b_inb, b_inw : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_hc  : unsigned(10 downto 0) := (others => '0');
    signal b_run : std_logic := '0';
    signal p0_ea, p0_eb : unsigned(16 downto 0) := (others => '0');
    signal p1_d, p1_ds : unsigned(11 downto 0) := (others => '0');          -- Q12 (the pixel path needs the top bits only)

    -- pixel path
    signal p2_phb, p2_phw, p2_phs : unsigned(11 downto 0) := (others => '0');   -- Q12 phases
    signal p3_pa  : unsigned(10 downto 0) := (others => '0');
    signal p3_pb  : unsigned(13 downto 0) := (others => '0');
    signal p3_sel : std_logic := '0';
    signal p3_df  : unsigned(8 downto 0) := (others => '0');
    signal p4_v   : unsigned(9 downto 0) := (others => '0');
    signal p5_i1  : unsigned(7 downto 0) := (others => '0');
    signal p5_kv  : signed(11 downto 0) := (others => '0');
    signal p5_a   : unsigned(7 downto 0) := (others => '0');
    signal p6_i1  : unsigned(7 downto 0) := (others => '0');
    signal p6_a1  : unsigned(2 downto 0) := (others => '0');
    signal p6_i2  : unsigned(7 downto 0) := (others => '0');
    signal p6_dd  : unsigned(9 downto 0) := (others => '0');
    signal p7_a1  : unsigned(2 downto 0) := (others => '0');
    signal p7_sk, p7_skc : signed(11 downto 0) := (others => '0');
    signal p8_a1, p8_a3 : unsigned(2 downto 0) := (others => '0');
    signal p9_a1, p9_a3 : unsigned(2 downto 0) := (others => '0');
    signal p9_bd, p9_wv : std_logic_vector(23 downto 0) := (others => '0');
    signal p10_a1, p10_a3 : unsigned(2 downto 0) := (others => '0');
    signal p10_dy, p10_du, p10_dv : signed(9 downto 0) := (others => '0');
    signal p10_wy, p10_wu, p10_wv : unsigned(7 downto 0) := (others => '0');
    signal p10_sn : std_logic_vector(23 downto 0) := (others => '0');
    signal p11_a3 : unsigned(2 downto 0) := (others => '0');
    signal p11_y, p11_u, p11_v : unsigned(7 downto 0) := (others => '0');
    signal p11_sy, p11_su, p11_sv : unsigned(7 downto 0) := (others => '0');
    signal p12_a3 : unsigned(2 downto 0) := (others => '0');
    signal p12_y, p12_u, p12_v : unsigned(7 downto 0) := (others => '0');
    signal p12_sy, p12_su, p12_sv : unsigned(7 downto 0) := (others => '0');
    signal p13_a3 : unsigned(2 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(7 downto 0) := (others => '0');
    signal p13_dy, p13_du, p13_dv : signed(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    -- chroma delay ring (U, V >> 2: the composed colours have two zero LSBs), 16 deep in one EBR; the read
    -- (6 behind the write) never meets the write address
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal rg : t_ring := (others => x"8080");
    attribute no_rw_check : boolean;
    attribute no_rw_check of rg : signal is true;
    signal rg_wa, rg_ra : unsigned(7 downto 0) := (others => '0');
    signal rg_wd, rg_q, rg_qf : std_logic_vector(15 downto 0) := x"8080";
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rg_q <= rg(to_integer(rg_ra));
            rg(to_integer(rg_wa)) <= rg_wd;
        end if;
    end process p_ring;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_PK_EXP2(to_integer(r_ex_a));
            r_ws_q <= C_PK_WSIN(to_integer(r_ws_a));
            r_sd_q <= C_PK_SUND(to_integer(r_sd_a));
            r_bd_q <= C_PK_BAND(to_integer(r_bd_a));
            r_wv_q <= C_PK_WEAVE(to_integer(r_wv_a));
            r_sn_q <= C_PK_SUN(to_integer(r_sn_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : signed(11 downto 0);
        variable v_p  : unsigned(19 downto 0);
        variable v_tr : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

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
            elsif s_zc /= 2047 then
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_tears  <= not registers_in(6)(0);      -- On = 0 (first label)
                s_mirror <= registers_in(6)(1);
                s_twin   <= registers_in(6)(2);
                s_pal    <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); s_mx <= to_unsigned(128, 9); s_c699 <= to_unsigned(11184, 14); s_xs16 <= to_unsigned(64, 7);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9); s_mx <= to_unsigned(171, 9); s_c699 <= to_unsigned(6291, 14);  s_xs16 <= to_unsigned(48, 7);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9); s_mx <= to_unsigned(256, 9); s_c699 <= to_unsigned(2796, 14);  s_xs16 <= to_unsigned(32, 7);
                end case;
                case to_integer(s_ys) is
                    when 1138   => s_mx2 <= to_unsigned(58, 9);  s_oys <= to_unsigned(537, 10);
                    when 960    => s_mx2 <= to_unsigned(68, 9);  s_oys <= to_unsigned(453, 10);
                    when 576    => s_mx2 <= to_unsigned(114, 9); s_oys <= to_unsigned(272, 10);
                    when 512    => s_mx2 <= to_unsigned(128, 9); s_oys <= to_unsigned(242, 10);
                    when 480    => s_mx2 <= to_unsigned(137, 9); s_oys <= to_unsigned(226, 10);
                    when 384    => s_mx2 <= to_unsigned(171, 9); s_oys <= to_unsigned(181, 10);
                    when others => s_mx2 <= to_unsigned(256, 9); s_oys <= to_unsigned(121, 10);
                end case;
                s_fc <= s_fc + 1;
                if s_fc2 = 149 then s_fc2 <= (others => '0'); else s_fc2 <= s_fc2 + 1; end if;
            end if;
            g_wm1 <= s_w - 1;
            -- sway: +-32 h triangle over 128 fields; chaos LFO: jitter full for 75 fields, quartered for 75
            v_tr := f_tri7(s_fc);
            c_swy <= signed(resize(v_tr(6 downto 1), 7)) - 32;
            if s_fc2 < 75 then c_cha <= '1'; else c_cha <= '0'; end if;

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 5      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when 3      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 5      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 6 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
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
                    when 510 => z_sel <= to_unsigned(20, 5); z_pre <= '1';
                    when 534 => z_sel <= to_unsigned(21, 5); z_pre <= '1';
                    when 558 => z_sel <= to_unsigned(22, 5); z_pre <= '1';
                    when 582 => z_sel <= to_unsigned(23, 5); z_pre <= '1';
                    when 606 => z_sel <= to_unsigned(24, 5); z_pre <= '1';
                    when 630 => z_sel <= to_unsigned(25, 5); z_pre <= '1';
                    when 654 => z_sel <= to_unsigned(26, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(8869, 36);           z_fb <= z_mt;                       -- fb
                    when 1  => z_fa <= to_unsigned(36788, 36);          z_fb <= z_mt2;                      -- fw
                    when 2  =>                                                                              -- jitter piece
                        if z_hi = '1' then z_fa <= resize(z_p, 36); z_fb <= to_unsigned(53, 16);
                        else z_fa <= resize(s_gp(12 downto 3), 36); z_fb <= to_unsigned(65, 16); end if;
                    when 3  =>                                                                              -- slip gain piece
                        if z_hi = '1' then z_fa <= resize(z_p, 36); z_fb <= to_unsigned(210, 16);
                        else z_fa <= resize(s_gp(12 downto 3), 36); z_fb <= to_unsigned(164, 16); end if;
                    when 4  => z_fa <= resize(c_fb, 36);                z_fb <= resize(s_xs16, 16);         -- inc b (Q28)
                    when 5  => z_fa <= resize(c_fw, 36);                z_fb <= resize(s_xs16, 16);         -- inc w
                    when 6  => z_fa <= resize(c_fb, 36);                z_fb <= resize(s_c699, 16);         -- bend b
                    when 7  => z_fa <= resize(c_fw, 36);                z_fb <= resize(s_c699, 16);         -- bend w
                    when 8  => z_fa <= resize(z_k3, 36);                z_fb <= to_unsigned(488, 16);       -- lean b per unit
                    when 9  => z_fa <= resize(z_a, 36);                 z_fb <= c_fb(17 downto 2);          -- x fb / 4
                    when 10 => z_fa <= resize(z_k3, 36);                z_fb <= to_unsigned(419, 16);       -- lean w per unit
                    when 11 => z_fa <= resize(z_a, 36);                 z_fb <= c_fw(17 downto 2);          -- x fw / 4
                    when 12 => z_fa <= resize(s_g4(12 downto 3), 36);   z_fb <= resize(s_g4(12 downto 3), 16);  -- k4^2
                    when 13 => z_fa <= resize(z_a(19 downto 4), 36);    z_fb <= c_fb(15 downto 0);          -- k4^2 fb / 2^20
                    when 14 => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(602, 16);       -- depth
                    when 15 => z_fa <= resize(to_unsigned(1023, 10) - s_g5(12 downto 3), 36); z_fb <= to_unsigned(231, 16);  -- hw0
                    when 16 => z_fa <= resize(z_k6, 36);                z_fb <= to_unsigned(269, 16);       -- tint
                    when 17 =>                                                                              -- ripple step per vline
                        if z_hi = '1' then z_fa <= resize(z_p, 36); z_fb <= to_unsigned(4350, 16);
                        else z_fa <= resize(s_gp(12 downto 3), 36); z_fb <= to_unsigned(1420, 16); end if;
                    when 18 => z_fa <= resize(z_rs, 36);                z_fb <= resize(s_ys, 16);           -- per output line
                    when 19 => z_fa <= resize(z_pv, 36);                z_fb <= resize(s_mx2, 16);          -- period in output lines
                    when 20 => z_fa <= resize(c_rpr, 36);               z_fb <= resize(z_jit, 16);          -- jitter range
                    when 21 => z_fa <= to_unsigned(120, 36);            z_fb <= resize(z_gn, 16);           -- slips x gain
                    when 22 => z_fa <= to_unsigned(40, 36);             z_fb <= resize(z_gn, 16);
                    when 23 => z_fa <= to_unsigned(40, 36);             z_fb <= resize(z_gn, 16);
                    when 24 => z_fa <= to_unsigned(60, 36);             z_fb <= resize(z_gn, 16);
                    when 25 => z_fa <= to_unsigned(94, 36);             z_fb <= resize(z_gn, 16);
                    when others => z_fa <= resize(c_sa(0), 36);         z_fb <= to_unsigned(1456, 16);      -- glide step
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- exponents: k - 512 in Q8 octaves (+-2 octaves), split into shift / fraction
                    when 20 =>
                        v_e := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 22 => z_mt <= r_ex_q;
                    when 23 =>
                        v_e := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh2 <= v_e(10 downto 8);
                    when 25 => z_mt2 <= r_ex_q;
                    when 26 =>
                        -- lean: |k3 - 512| - 8 (deadband), sign; tint likewise; storm split at 400
                        if s_g3(12 downto 3) >= 520 then z_k3 <= s_g3(12 downto 3) - 520; z_k3n <= '0';
                        elsif s_g3(12 downto 3) <= 504 then z_k3 <= 504 - s_g3(12 downto 3); z_k3n <= '1';
                        else z_k3 <= (others => '0'); z_k3n <= '0'; end if;
                        if s_g6(12 downto 3) >= 520 then z_k6 <= s_g6(12 downto 3) - 520; z_k6n <= '0';
                        elsif s_g6(12 downto 3) <= 504 then z_k6 <= 504 - s_g6(12 downto 3); z_k6n <= '1';
                        else z_k6 <= (others => '0'); z_k6n <= '0'; end if;
                        if s_gp(12 downto 3) > 400 then z_p <= s_gp(12 downto 3) - 400; z_hi <= '1';
                        else z_p <= (others => '0'); z_hi <= '0'; end if;
                    when 27 =>                        -- storm: period 30 - 14 (p - 400) / 623 vlines
                        v_p := resize(z_p, 20);
                        v_p := shift_left(v_p, 2) + shift_left(v_p, 1);
                        z_pv <= to_unsigned(30, 8) - resize(v_p(15 downto 8), 8);
                    when 50  =>                       -- fb = 8869 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_fb <= f_acc(34 downto 17);
                            when -1     => c_fb <= f_acc(33 downto 16);
                            when 0      => c_fb <= f_acc(32 downto 15);
                            when others => c_fb <= f_acc(31 downto 14);
                        end case;
                    when 74  =>
                        case to_integer(z_sh2) is
                            when -2     => c_fw <= f_acc(34 downto 17);
                            when -1     => c_fw <= f_acc(33 downto 16);
                            when 0      => c_fw <= f_acc(32 downto 15);
                            when others => c_fw <= f_acc(31 downto 14);
                        end case;
                    when 98  =>                       -- jitter 0 .. 0.4 (p <= 400) .. 0.9, Q8
                        if z_hi = '1' then z_jit <= to_unsigned(102, 9) + resize(f_acc(16 downto 8), 9);
                        else z_jit <= resize(f_acc(16 downto 8), 9); end if;
                    when 122 =>                       -- slip gain 0 .. 1 .. 3, Q8
                        if z_hi = '1' then z_gn <= to_unsigned(256, 10) + resize(f_acc(17 downto 8), 10);
                        else z_gn <= resize(f_acc(17 downto 8), 10); end if;
                    when 146 => c_inb <= f_acc(23 downto 0);
                    when 170 => c_inw <= f_acc(23 downto 0);
                    when 194 => c_dnb <= f_acc(30 downto 20);
                    when 218 => c_dnw <= f_acc(30 downto 20);
                    when 242 => z_a <= f_acc(19 downto 0);                       -- |k3d| 488 (Q16 h per vline)
                    when 266 =>                       -- lean b = z_a fb / 2^16 = (z_a fb / 4) >> 14, Q24 phase per vline
                        c_lbv <= f_acc(33 downto 14);
                        c_lbn <= not z_k3n;                                     -- band leans left for k3 > 512
                    when 290 => z_a <= f_acc(19 downto 0);
                    when 314 => c_lwv <= f_acc(33 downto 14); c_lwn <= z_k3n;
                    when 338 => z_a <= f_acc(19 downto 0);                       -- k4^2
                    when 362 => z_a <= f_acc(35 downto 16);                      -- k4^2 fb / 2^20
                    when 386 => c_dep <= f_acc(24 downto 12);                    -- x 602 / 4096 (Q20)
                    when 410 => c_hw0 <= -signed(resize(f_acc(17 downto 0), 19));
                    when 434 =>
                        if z_k6n = '1' then c_tcb <= f_acc(18 downto 11); c_tcr <= (others => '0');
                        else c_tcr <= f_acc(18 downto 11); c_tcb <= (others => '0'); end if;
                    when 458 =>
                        if z_hi = '1' then z_rs <= f_acc(23 downto 8) + 2219; else z_rs <= f_acc(23 downto 8); end if;
                    when 482 => c_rst <= f_acc(23 downto 8);
                    when 506 => c_rpr <= f_acc(15 downto 8);
                    when 530 => c_jpr <= f_acc(15 downto 8);
                    when 554 => c_sa(0) <= f_acc(17 downto 8);
                    when 578 => c_sa(1) <= f_acc(17 downto 8);
                    when 602 => c_sa(2) <= f_acc(17 downto 8);
                    when 626 => c_sa(3) <= f_acc(17 downto 8);
                    when 650 => c_sa(4) <= f_acc(17 downto 8);
                    when 674 => c_gst <= f_acc(19 downto 8);
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer
    -- write of (luma >> 2) - 16
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
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17): the sky line (hold / mirror), the slips and the
    -- smear window, the ripple, the two start phases and the sun offset, the
    -- key ramps.  Everything is a function of the sky line, so the mirror
    -- is free.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : unsigned(18 downto 0);
        variable v_r  : signed(12 downto 0);
        variable v_m  : unsigned(12 downto 0);
        variable v_u  : unsigned(11 downto 0);
        variable v_s8 : signed(8 downto 0);
        variable v_s9 : signed(8 downto 0);
        variable v_br : unsigned(8 downto 0);
        variable v_b  : std_logic;
        variable v_h  : signed(19 downto 0);
        variable v_so : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 700 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    -- sky line: before the horizon itself; after it, held (or mirrored)
                    if e_vy < C_HZ then e_ysk <= e_vy; e_wat <= '0';
                    else
                        e_wat <= '1';
                        if s_mirror = '1' then
                            if e_vy < C_HZ2 then e_ysk <= C_HZ2 - e_vy; else e_ysk <= (others => '0'); end if;
                        else
                            e_ysk <= C_HZ;
                        end if;
                    end if;
                when 2 =>
                    e_yv <= e_ysk(18 downto 8);
                    -- weave wobble: +-40 h triangle over 256 vlines (tri7 * 5/8 - 40)
                    v_u := resize(f_tri7(e_ysk(15 downto 8)), 12);
                    v_u := shift_right(v_u, 1) + shift_right(v_u, 3);
                    e_wob <= signed(resize(v_u, 8)) - 40;
                    -- breath of the reflected core: +-224 h, a triangle over 512 vlines below the horizon that
                    -- starts at its peak (the core narrows first), drifting down slowly
                    v_y := e_vy - C_HZ;
                    v_br := v_y(16 downto 8) + resize(s_fc(7 downto 2), 9);
                    if v_br(8) = '1' then v_br := not v_br; end if;
                    v_s9 := to_signed(128, 9) - signed(v_br);
                    e_br <= resize(v_s9, 10) + resize(shift_right(v_s9, 1), 10) + resize(shift_right(v_s9, 2), 10);
                    -- lfsr per line
                    v_b := e_lfsr(15) xor e_lfsr(13) xor e_lfsr(12) xor e_lfsr(10);
                    e_lfsr <= e_lfsr(14 downto 0) & v_b;
                    -- ripple of this line: a new cycle reloads, else the saw steps down; sky lines prime a reload
                    if e_wat = '0' then
                        e_rip <= (others => '0'); e_rn <= (others => '0'); e_rld <= '1';
                    elsif e_rld = '1' then
                        e_rip <= e_ripn; e_rn <= to_unsigned(1, 8); e_rp <= e_rpn;
                        if e_rpn <= 1 then e_rld <= '1'; else e_rld <= '0'; end if;
                    else
                        e_rip <= e_rip - signed(resize(c_rst, 22)); e_rn <= e_rn + 1;
                        if e_rn + 1 >= e_rp then e_rld <= '1'; else e_rld <= '0'; end if;
                    end if;
                when 3 =>
                    -- step slips (events 1 .. 4) and the glide rows of event 0 (85 .. 130)
                    v_r := (others => '0');
                    if s_tears = '1' then
                        if e_yv >= 316 then v_r := v_r - signed(resize(c_sa(1), 13)); end if;
                        if e_yv >= 408 then v_r := v_r - signed(resize(c_sa(2), 13)); end if;
                        if e_yv >= 468 then v_r := v_r - signed(resize(c_sa(3), 13)); end if;
                        if e_yv >= 576 then v_r := v_r + signed(resize(c_sa(4), 13)); end if;
                    end if;
                    e_st <= resize(v_r, 12);
                    if e_yv < 85 then e_gl <= (others => '0');
                    elsif e_yv >= 130 then e_gl <= to_unsigned(45, 6);
                    else e_gl <= resize(e_yv - 85, 6); end if;
                    -- smear event: rows r .. r + sm - 1; hold start / length (vpx) and the start's drift per row
                    e_sm <= to_unsigned(5, 3); e_dy <= (others => '0');
                    e_x0v <= to_unsigned(300, 12); e_lv <= to_unsigned(500, 12); e_dx <= (others => '0'); e_dxn <= '0';
                    if s_tears = '1' then
                        if e_yv >= 128 and e_yv < 138 then
                            e_sm <= to_unsigned(0, 3); e_dy <= resize(e_yv - 128, 5);
                            e_x0v <= to_unsigned(175, 12); e_lv <= to_unsigned(400, 12); e_dx <= to_unsigned(8, 5); e_dxn <= '1';
                        elsif e_yv >= 316 and e_yv < 324 then
                            e_sm <= to_unsigned(1, 3); e_dy <= resize(e_yv - 316, 5);
                            e_x0v <= to_unsigned(0, 12); e_lv <= to_unsigned(1900, 12); e_dx <= to_unsigned(0, 5); e_dxn <= '0';
                        elsif e_yv >= 408 and e_yv < 422 then
                            e_sm <= to_unsigned(2, 3); e_dy <= resize(e_yv - 408, 5);
                            e_x0v <= to_unsigned(240, 12); e_lv <= to_unsigned(760, 12); e_dx <= to_unsigned(20, 5); e_dxn <= '0';
                        elsif e_yv >= 468 and e_yv < 480 then
                            e_sm <= to_unsigned(3, 3); e_dy <= resize(e_yv - 468, 5);
                            e_x0v <= to_unsigned(0, 12); e_lv <= to_unsigned(700, 12); e_dx <= to_unsigned(30, 5); e_dxn <= '0';
                        elsif e_yv >= 576 and e_yv < 584 then
                            e_sm <= to_unsigned(4, 3); e_dy <= resize(e_yv - 576, 5);
                            e_x0v <= to_unsigned(300, 12); e_lv <= to_unsigned(500, 12); e_dx <= to_unsigned(20, 5); e_dxn <= '1';
                        end if;
                    end if;
                when 4 =>
                    if e_sm = 5 then n_hen <= '0'; else n_hen <= '1'; end if;
                -- products (load at L, result at L + 17): lean b (8), lean w (25), glide (42), x0 drift (59),
                -- x0 scale (77), L scale (94), phase b (111), phase w (128), jitter (145), 340 yv (162),
                -- 255 yv (179), 46 yv (196), 410 yv (213), ripple reload (230)
                when 8  => m_a <= resize(c_lbv, 36); m_b <= resize(e_yv, 16); m_acc <= (others => '0');
                when 25 => n_lb <= m_acc(23 downto 0);
                           m_a <= resize(c_lwv, 36); m_b <= resize(e_yv, 16); m_acc <= (others => '0');
                when 26 =>
                    -- sun trough offset from the band phase: the lean cancelled (the suns do not lean); Q12
                    if c_lbn = '1' then v_so := C_SO + n_lb; else v_so := C_SO - n_lb; end if;
                    n_so <= v_so(23 downto 12);
                when 42 => n_lw <= m_acc(23 downto 0);
                           m_a <= resize(c_gst, 36); m_b <= resize(e_gl, 16); m_acc <= (others => '0');
                when 59 =>
                    -- common slip (h) = steps + glide (event 0 is +, to the right); shift = SD offset - slip + ripple
                    if s_tears = '1' then v_r := resize(e_st, 13) + signed(resize(m_acc(20 downto 8), 13)); else v_r := (others => '0'); end if;
                    v_r := signed(resize(s_x0, 13)) - v_r;
                    if e_wat = '1' then v_r := v_r + resize(e_rip(19 downto 8), 13); end if;
                    e_sh <= v_r;
                    e_shw <= v_r - resize(e_wob, 13) - resize(c_swy, 13);
                    m_a <= resize(e_dx, 36); m_b <= resize(e_dy, 16); m_acc <= (others => '0');
                when 76 =>
                    if e_dxn = '1' then e_x0v <= e_x0v - m_acc(11 downto 0); else e_x0v <= e_x0v + m_acc(11 downto 0); end if;
                when 77 => m_a <= resize(e_x0v, 36); m_b <= resize(s_mx, 16); m_acc <= (others => '0');
                when 94 => n_hx0 <= m_acc(18 downto 8);
                           m_a <= resize(e_lv, 36); m_b <= resize(s_mx, 16); m_acc <= (others => '0');
                when 111 => n_hl <= m_acc(18 downto 8);
                           if e_sh < 0 then m_a <= resize(unsigned(-e_sh), 36); else m_a <= resize(unsigned(e_sh), 36); end if;
                           m_b <= c_fb(15 downto 0); m_acc <= (others => '0');
                when 128 =>
                    -- phase b = PB0 -+ lean +- |shift| fb (fb < 2^16; shift in h); two steps
                    if e_sh < 0 then n_phb <= C_PB0 - m_acc(23 downto 0); else n_phb <= C_PB0 + m_acc(23 downto 0); end if;
                    if e_shw < 0 then m_a <= resize(unsigned(-e_shw), 36); else m_a <= resize(unsigned(e_shw), 36); end if;
                    m_b <= c_fw(17 downto 2); m_acc <= (others => '0');
                when 129 =>
                    if c_lbn = '1' then n_phb <= n_phb - n_lb; else n_phb <= n_phb + n_lb; end if;
                when 145 =>
                    -- phase w: the product used fw / 4, so the term is m_acc 4
                    if e_shw < 0 then n_phw <= C_PW0 - (m_acc(21 downto 0) & "00"); else n_phw <= C_PW0 + (m_acc(21 downto 0) & "00"); end if;
                    -- next ripple cycle: period = per +- lfsr * jitter range >> 7 (quartered in the calm half of the chaos LFO)
                    v_s8 := signed(resize(e_lfsr(7 downto 0), 9)) - 128;
                    if v_s8 < 0 then m_a <= resize(unsigned(-v_s8), 36); else m_a <= resize(unsigned(v_s8), 36); end if;
                    if c_cha = '1' then m_b <= resize(c_jpr, 16); else m_b <= resize(c_jpr(7 downto 2), 16); end if;
                    m_acc <= (others => '0');
                when 146 =>
                    if c_lwn = '1' then n_phw <= n_phw - n_lw; else n_phw <= n_phw + n_lw; end if;
                when 162 =>
                    v_m := resize(m_acc(14 downto 7), 13);
                    if e_lfsr(7) = '1' then
                        if resize(c_rpr, 13) > v_m + 4 then e_rpn <= resize(resize(c_rpr, 13) - v_m, 8); else e_rpn <= to_unsigned(4, 8); end if;
                    else
                        e_rpn <= resize(resize(c_rpr, 13) + v_m, 8);
                    end if;
                    m_a <= resize(e_yv, 36); m_b <= to_unsigned(340, 16); m_acc <= (others => '0');
                when 179 =>                       -- drop1 = 340 yv >> 8 (Q10), saturated
                    if m_acc(19 downto 18) /= "00" then n_d1 <= (others => '1'); else n_d1 <= m_acc(17 downto 8); end if;
                    m_a <= resize(e_yv, 36); m_b <= to_unsigned(255, 16); m_acc <= (others => '0');
                when 196 =>                       -- kt = 594 + 255 yv >> 8 (Q10)
                    n_kt <= C_KT0 + m_acc(18 downto 8);
                    m_a <= resize(e_yv, 36); m_b <= to_unsigned(46, 16); m_acc <= (others => '0');
                when 213 =>                       -- drop2 = 46 yv >> 8 (Q8), saturated
                    if m_acc(16) = '1' then n_d2 <= (others => '1'); else n_d2 <= m_acc(15 downto 8); end if;
                    m_a <= resize(e_yv, 36); m_b <= to_unsigned(410, 16); m_acc <= (others => '0');
                when 230 =>                       -- sun half-width = hw0 + 410 yv (Q8 h) -> h; the water core breathes
                    v_h := resize(c_hw0, 20) + signed(resize(m_acc(18 downto 0), 20));
                    n_hw <= v_h(19 downto 8);
                    if e_wat = '1' then n_hwc <= v_h(19 downto 8) - resize(e_br, 12); else n_hwc <= v_h(19 downto 8); end if;
                    m_a <= resize(c_rst, 36); m_b <= resize(e_rpn, 16); m_acc <= (others => '0');
                when 247 =>
                    -- reload value of the next cycle: step * period - 24 h (the saw runs from +amp - 24 h down)
                    e_ripn <= signed(resize(m_acc(21 downto 0), 22)) - to_signed(6144, 22);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- DDAs (stage 1): two Q24 phases stepped by a decrementing Q28 increment
    -- (the bent ramp), paused for the smear window.  The line-buffer read
    -- address runs 4 px ahead so the luma displacement (stage 0 partials,
    -- summed at stage 1) lands in stage 1 with the phases.
    ---------------------------------------------------------------------------
    p_dda : process(clk)
        variable v_pause : std_logic;
        variable v_d     : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_phb <= n_phb; b_phw <= n_phw;
                b_inb <= c_inb; b_inw <= c_inw;
                b_px  <= (others => '0');
                b_hc  <= (others => '0');
                b_run <= '1';
                l_so <= n_so; l_d1 <= n_d1; l_kt <= n_kt; l_d2 <= n_d2; l_hw <= n_hw; l_hwc <= n_hwc;
                l_hx0 <= n_hx0; l_hl <= n_hl; l_hen <= n_hen;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                v_pause := '0';
                if b_hc /= 0 then
                    v_pause := '1';
                    b_hc <= b_hc - 1;
                elsif l_hen = '1' and b_run = '1' and b_px = l_hx0 then
                    v_pause := '1';
                    b_hc <= l_hl - 1;
                    b_run <= '0';
                end if;
                if v_pause = '0' then
                    b_phb <= b_phb + b_inb(23 downto 4); b_inb <= b_inb - resize(c_dnb, 24);
                    b_phw <= b_phw + b_inw(23 downto 4); b_inw <= b_inw - resize(c_dnw, 24);
                end if;
            end if;
            s_lb_ra <= b_px + 4;
            -- stage 0: luma partials d = lm * depth (Q20 phase); stage 1: the sum (kept as Q12), and the sun's
            -- (doubled for Four) plus the line's trough offset
            p0_ea <= unsigned(s_lb_q(3 downto 0)) * c_dep;
            p0_eb <= unsigned(s_lb_q(7 downto 4)) * c_dep;
            v_d := resize(p0_ea, 20) + shift_left(resize(p0_eb, 20), 4);
            p1_d <= v_d(19 downto 8);
            if s_twin = '1' then p1_ds <= (v_d(18 downto 8) & '0') + l_so; else p1_ds <= v_d(19 downto 8) + l_so; end if;
        end if;
    end process p_dda;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = DDA values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p10 : unsigned(9 downto 0);
        variable v_p9  : unsigned(8 downto 0);
        variable v_f   : unsigned(9 downto 0);
        variable v_v   : unsigned(13 downto 0);
        variable v_i   : signed(11 downto 0);
        variable v_a   : unsigned(8 downto 0);
        variable v_u   : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: displaced phases, top 12 bits (the weave 4x: its period is a quarter of the band's); the sun from
            -- the band
            p2_phb <= b_phb(23 downto 12) + p1_d;
            p2_phw <= b_phw(23 downto 12) + (p1_d(9 downto 0) & "00");
            if s_twin = '1' then p2_phs <= b_phb(22 downto 11) + p1_ds; else p2_phs <= b_phb(23 downto 12) + p1_ds; end if;
            -- 3: band saw terms (rise 7/8: p 8/7; fall: (1024 - p) 8), weave ROM address, sun fold
            v_p10 := p2_phb(11 downto 2);
            p3_pa <= resize(v_p10, 11) + resize(shift_right(v_p10, 3), 11) + resize(shift_right(v_p10, 6), 11) + resize(shift_right(v_p10, 9), 11);
            p3_pb <= shift_left(resize(to_unsigned(1024, 11) - resize(v_p10, 11), 14), 3);
            if v_p10 >= 896 then p3_sel <= '1'; else p3_sel <= '0'; end if;
            r_ws_a <= p2_phw(11 downto 4);
            v_p9 := p2_phs(11 downto 3);
            v_f  := to_unsigned(512, 10) - resize(v_p9, 10);
            if v_p9(8) = '1' then p3_df <= v_f(8 downto 0); else p3_df <= v_p9; end if;
            -- 4: saw value (clamped to 1023), sun distance ROM address
            if p3_sel = '1' then v_v := p3_pb; else v_v := resize(p3_pa, 14); end if;
            if v_v > 1023 then p4_v <= (others => '1'); else p4_v <= v_v(9 downto 0); end if;
            if p3_df(8) = '1' then r_sd_a <= (others => '1'); else r_sd_a <= p3_df(7 downto 0); end if;
            -- 5: band index and key value, weave ROM fabric copy
            v_i := signed(resize(p4_v, 12)) - signed(resize(l_d1, 12));
            if v_i < 0 then p5_i1 <= (others => '0'); else p5_i1 <= unsigned(v_i(9 downto 2)); end if;
            p5_kv <= signed(resize(p4_v, 12)) - signed(resize(l_kt, 12));
            p5_a <= r_ws_q;
            -- 6: key level (0 .. 4 over 72 Q10 units), weave index, sun distance copy
            p6_i1 <= p5_i1;
            if p5_kv < 0 then p6_a1 <= "000";
            elsif p5_kv < 18 then p6_a1 <= "001";
            elsif p5_kv < 36 then p6_a1 <= "010";
            elsif p5_kv < 54 then p6_a1 <= "011";
            else p6_a1 <= "100"; end if;
            v_a := resize(p5_a, 9) + resize(l_d2, 9);
            if v_a(8) = '1' then p6_i2 <= (others => '1'); else p6_i2 <= v_a(7 downto 0); end if;
            if s_twin = '1' then p6_dd <= '0' & r_sd_q(9 downto 1); else p6_dd <= r_sd_q; end if;
            -- 7: band and weave ROM addresses, sun key values (px): half-width - distance
            r_bd_a <= s_pal & p6_i1;
            r_wv_a <= s_pal & p6_i2;
            p7_a1 <= p6_a1;
            p7_sk  <= shift_right(l_hw, 1)  - signed(resize(p6_dd, 12));
            p7_skc <= shift_right(l_hwc, 1) - signed(resize(p6_dd, 12));
            -- 8: sun key level (soft edge 25 px), sun ROM address (core depth); band / weave ROM data (EBR)
            p8_a1 <= p7_a1;
            if p7_sk < 0 then p8_a3 <= "000";
            elsif p7_sk < 6 then p8_a3 <= "001";
            elsif p7_sk < 12 then p8_a3 <= "010";
            elsif p7_sk < 19 then p8_a3 <= "011";
            else p8_a3 <= "100"; end if;
            if p7_skc < 0 then r_sn_a <= s_pal & to_unsigned(0, 8);
            elsif p7_skc > 255 then r_sn_a <= s_pal & to_unsigned(255, 8);
            else r_sn_a <= s_pal & unsigned(p7_skc(7 downto 0)); end if;
            -- 9: band / weave fabric copies; sun ROM data (EBR)
            p9_bd <= r_bd_q; p9_wv <= r_wv_q; p9_a1 <= p8_a1; p9_a3 <= p8_a3;
            -- 10: band - weave differences (8-bit colours), sun ROM fabric copy
            p10_dy <= signed(resize(unsigned(p9_bd(23 downto 16)), 10)) - signed(resize(unsigned(p9_wv(23 downto 16)), 10));
            p10_du <= signed(resize(unsigned(p9_bd(15 downto 8)), 10))  - signed(resize(unsigned(p9_wv(15 downto 8)), 10));
            p10_dv <= signed(resize(unsigned(p9_bd(7 downto 0)), 10))   - signed(resize(unsigned(p9_wv(7 downto 0)), 10));
            p10_wy <= unsigned(p9_wv(23 downto 16)); p10_wu <= unsigned(p9_wv(15 downto 8)); p10_wv <= unsigned(p9_wv(7 downto 0));
            p10_sn <= r_sn_q; p10_a1 <= p9_a1; p10_a3 <= p9_a3;
            -- 11: sky = weave + (band - weave) key/4
            p11_y <= f_mix(p10_wy, p10_dy, p10_a1); p11_u <= f_mix(p10_wu, p10_du, p10_a1); p11_v <= f_mix(p10_wv, p10_dv, p10_a1);
            p11_sy <= unsigned(p10_sn(23 downto 16)); p11_su <= unsigned(p10_sn(15 downto 8)); p11_sv <= unsigned(p10_sn(7 downto 0));
            p11_a3 <= p10_a3;
            -- 12: tint (Cr or Cb offset on the sky), clamped
            p12_y <= p11_y;
            v_u := resize(p11_u, 9) + resize(c_tcr, 9);
            if v_u > 250 then p12_u <= to_unsigned(250, 8); else p12_u <= v_u(7 downto 0); end if;
            v_u := resize(p11_v, 9) + resize(c_tcb, 9);
            if v_u > 250 then p12_v <= to_unsigned(250, 8); else p12_v <= v_u(7 downto 0); end if;
            p12_sy <= p11_sy; p12_su <= p11_su; p12_sv <= p11_sv; p12_a3 <= p11_a3;
            -- 13: sun - sky differences
            p13_dy <= signed(resize(p12_sy, 10)) - signed(resize(p12_y, 10));
            p13_du <= signed(resize(p12_su, 10)) - signed(resize(p12_u, 10));
            p13_dv <= signed(resize(p12_sv, 10)) - signed(resize(p12_v, 10));
            p13_y <= p12_y; p13_u <= p12_u; p13_v <= p12_v; p13_a3 <= p12_a3;
            -- 14: compose: sky + (sun - sky) key/4, to 10 bits; neutral black outside the generated avid
            if s_gav_sr(11) = '0' then
                s_y14 <= C_KY; s_u14 <= C_KC; s_v14 <= C_KC;
            else
                s_y14 <= f_mix(p13_y, p13_dy, p13_a3) & "00";
                s_u14 <= f_mix(p13_u, p13_du, p13_a3) & "00";
                s_v14 <= f_mix(p13_v, p13_dv, p13_a3) & "00";
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(11);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the HD HDMI bar fix and the capture's softness), luma
    -- [1 2 1]/4 aligned to the box's centre, luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the neutral black (the off-avid input of the box) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- stage 15: sums (and the luma delay line); stage 16: box output and the luma [1 2 1]
            rg_wd(15 downto 8) <= std_logic_vector(s_u14(9 downto 2)); rg_wd(7 downto 0) <= std_logic_vector(s_v14(9 downto 2));
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 4;
            rg_qf <= rg_q;                                           -- = the input of 8 clocks ago
            s_ly(0) <= s_y14;
            for i in 1 to 5 loop s_ly(i) <= s_ly(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= C_KC & "000"; s_bav <= C_KC & "000";
            else
                s_bau <= s_bau + s_u14 - shift_left(resize(unsigned(rg_qf(15 downto 8)), 10), 2);
                s_bav <= s_bav + s_v14 - shift_left(resize(unsigned(rg_qf(7 downto 0)), 10), 2);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(13) = '1' then
                v_sc := resize(s_bau, 14) + s_baud; s_u16 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v16 <= v_sc(13 downto 4);
                v_sy := resize(s_ly(3), 12) + resize(s_ly(4), 12) + resize(s_ly(4), 12) + resize(s_ly(5), 12);
                s_y16 <= v_sy(11 downto 2);
            else
                s_y16 <= C_KY; s_u16 <= C_KC; s_v16 <= C_KC;
            end if;
            -- 17: luma 13/16
            if s_y16 > 64 then v_dl := s_y16 - 64; else v_dl := (others => '0'); end if;
            s_y17 <= s_y16 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u17 <= s_u16; s_v17 <= s_v16;
            -- 18: copy (even latency)
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            -- 19: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y19);
    data_out.u       <= std_logic_vector(s_u19);
    data_out.v       <= std_logic_vector(s_v19);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture peaks;
