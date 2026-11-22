-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Vase v0.6.0 (phase 2 diag) -- Heirloom #27, rebuilt from the photo WIN_20230424_23_59_47.
--
-- The photo: a camera picture of a glossy lathe object (a vase: neck, upper bulb with a white
-- highlight loop round a dark eye, waist, wide base with three vertical sheens) keyed onto black
-- and colourised.  One slow vertical oscillator (556 lines) drives the colouriser: the body's hue
-- swings yellow-brown / green, a blue tint rises at the waist and the base, and its zones give
-- fast red bars (26.8 lines, hard-synced, widening along the cycle), a gap, a red block, an off
-- zone and a magenta block.  The red only shows inside a window on |x - 1090| (screen x): a
-- no-red channel right of the axis, red bands either side, and an inverted edge zone at the
-- far left (red in the gaps).  Sheens kill the key (white over the bars).  The left edge has a
-- grey halo and a line comb; thin red lines flash on the bar tops.
--
-- The program: the object luma is a 64 x 64 table in lathe coordinates (u = (x - c) / hw) under a
-- per-line profile; u -> asin -> + spin -> fold -> sin gives the turning texture.  A per-line
-- engine (one serial multiplier, one serial divider, 10 slots of 28 clocks) computes the next
-- line's sway, edges, u DDA, cycle zone, bar state and colour curves in virtual 1080-line
-- coordinates; a fill engine writes the next line's y-interpolated table row (128 entries) into a
-- ping-pong line RAM.  The colouriser is RGB arithmetic (ramps from a luma ROM, clip per channel)
-- mixed by the soft red key, then RGB -> YUV.
--
--   K1  Bars     fast bars per colour cycle (photo 20.7)
--   K2  Bands    colour cycles down the frame (photo 1.94, 556 lines)
--   K3  Roll     bipolar: the colour cycle and bars roll down / up (still at the centre)
--   K4  Spin     bipolar: the vase turns (sheens slide round the lathe; helix and sway with it)
--   K5  Key      red-key centre, -624 .. +527 virtual px about the vase axis (photo +67): the
--                no-red channel slides across the vase and off it (red falls in the gaps)
--   K6  Camera   surface strength (K6 camera pass, v0.6.1): the camera (Video) or the stored vase
--                (Osc) x 0 .. 2 (x1 at 50 %, the shipped look), max'd with the vase's own sheens and
--                half its luma; 0 % = what the program draws with a black input.  The v0.6.0 K6
--                Gloss is dropped (fixed x1): K6 in Osc spans its dull .. glazed range
--   S7  Swap     the red key swaps bars and gaps (red falls between the bars, the blocks go olive)
--   S8  Twin     Off / On: a second, mirrored vase (the two at 1/4 and 3/4 of the width)
--   S9  Wrap     Rings (photo) / Helix: the bars wrap round the vase as a spiral that turns with Spin
--   S10 Palette  photo / channel rotation (green bars on violet)
--   S11 Source   Video (camera wrapped round the vase) / Osc (the stored glossy vase)
--   P12 BULGE    profile deviation x0 .. x2.5: straight cone .. photo (40 %) .. hourglass; above
--                the photo the axis also sways (a travelling S-bend, 0 .. +-306 px, phase = 4 x spin)
--
-- The edge comb and the bar-top lines (v0.5 S8 Tear) and the photo zones (v0.5 S9 Blocks) are
-- always on.  Generated avid; output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16,
-- luma 13/16) before the gate.  Latency 32 clocks in every mode.  u = Cr, v = Cb.

-- K6 camera pass 2026-11-15 (v0.6.1): K6 = Camera (new, glided, per-field gains on the camera write
-- side and in the fill engine); K6 Gloss dropped (fixed x1); nothing else moved.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.vase_pkg.all;
use work.all;

architecture vase of program_top is
    constant C_LAT : integer := 32;

    function f_vsx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_vsy(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 11);
            when 4          => return to_unsigned(576, 11);
            when 8          => return to_unsigned(960, 11);
            when 12         => return to_unsigned(480, 11);
            when 5 | 6 | 14 => return to_unsigned(384, 11);
            when 1 | 2 | 10 => return to_unsigned(512, 11);
            when others     => return to_unsigned(256, 11);
        end case;
    end function;
    function f_ryinc(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(17010, 15);
            when 4          => return to_unsigned(8610, 15);
            when 8          => return to_unsigned(14349, 15);
            when 12         => return to_unsigned(7175, 15);
            when 5 | 6 | 14 => return to_unsigned(5740, 15);
            when 1 | 2 | 10 => return to_unsigned(7653, 15);
            when others     => return to_unsigned(3826, 15);
        end case;
    end function;
    function f_winc(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(46603, 17);
            when 5 | 6 | 14     => return to_unsigned(26214, 17);
            when others         => return to_unsigned(17476, 17);
        end case;
    end function;

    function f_clip(v : signed; hi : integer; w : integer) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, w);
        elsif v > hi then return to_unsigned(hi, w);
        else return resize(unsigned(v), w); end if;
    end function;
    -- soft key ramp: clip((v + 4) * 2, 0, 16)
    function f_r(v : signed(13 downto 0)) return unsigned is
        variable t : signed(14 downto 0);
    begin
        t := shift_left(resize(v, 15) + 4, 1);
        if t < 0 then return to_unsigned(0, 5);
        elsif t > 16 then return to_unsigned(16, 5);
        else return unsigned(t(4 downto 0)); end if;
    end function;
    function f_min5(a, b : unsigned(4 downto 0)) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;
    function f_max5(a, b : unsigned(4 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;
    -- edge factor in Q.4: clip(e * 64 / 12, 0, 64) / 4, e = virtual Q.2 distance inside the edge
    function f_fe(e : signed(15 downto 0)) return unsigned is
    begin
        if e <= 0 then return to_unsigned(0, 5);
        elsif e >= 12 then return to_unsigned(16, 5);
        else
            case to_integer(unsigned(e(3 downto 0))) is
                when 1 => return to_unsigned(1, 5);
                when 2 => return to_unsigned(2, 5);
                when 3 => return to_unsigned(4, 5);
                when 4 => return to_unsigned(5, 5);
                when 5 => return to_unsigned(6, 5);
                when 6 => return to_unsigned(8, 5);
                when 7 => return to_unsigned(9, 5);
                when 8 => return to_unsigned(10, 5);
                when 9 => return to_unsigned(12, 5);
                when 10 => return to_unsigned(13, 5);
                when others => return to_unsigned(14, 5);
            end case;
        end if;
    end function;

    ---------------------------------------------------------------------------
    -- ROMs / RAMs
    ---------------------------------------------------------------------------
    signal r_ta : unsigned(11 downto 0) := (others => '0');
    signal r_tq, r_tq2 : std_logic_vector(7 downto 0) := (others => '0');
    signal r_pca : unsigned(7 downto 0) := (others => '0');
    signal r_pcq, r_pcq2 : std_logic_vector(31 downto 0) := (others => '0');
    signal r_asa : unsigned(8 downto 0) := (others => '0');
    signal r_asq : std_logic_vector(9 downto 0) := (others => '0');
    signal r_sna : unsigned(8 downto 0) := (others => '0');
    signal r_snq : std_logic_vector(9 downto 0) := (others => '0');
    signal r_cra : unsigned(7 downto 0) := (others => '0');
    signal r_crq : std_logic_vector(63 downto 0) := (others => '0');

    attribute no_rw_check : boolean;
    -- line RAM: two banks of 128 luma entries
    type t_lr is array (0 to 255) of std_logic_vector(7 downto 0);
    signal lr : t_lr := (others => (others => '0'));
    attribute no_rw_check of lr : signal is true;
    signal lr_wa, lr_ra : unsigned(7 downto 0) := (others => '0');
    signal lr_wd : std_logic_vector(7 downto 0) := (others => '0');
    signal lr_we : std_logic := '0';
    signal lr_q  : std_logic_vector(7 downto 0) := (others => '0');
    -- camera line buffer: two banks of 512 (one source width per turn)
    type t_cb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal cb : t_cb := (others => (others => '0'));
    attribute no_rw_check of cb : signal is true;
    signal cb_wa, cb_ra : unsigned(9 downto 0) := (others => '0');
    signal cb_wd : std_logic_vector(7 downto 0) := (others => '0');
    signal cb_we : std_logic := '0';
    signal cb_q  : std_logic_vector(7 downto 0) := (others => '0');
    -- side-signal ring (stage 5 -> 14): {0, on, env(5), key(5), hal(4)}
    type t_rg is array (0 to 255) of std_logic_vector(15 downto 0);
    signal rg : t_rg := (others => (others => '0'));
    attribute no_rw_check of rg : signal is true;
    signal rg_p  : unsigned(7 downto 0) := (others => '0');
    signal rg_wa, rg_ra : unsigned(7 downto 0) := (others => '0');
    signal rg_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal rg_q  : std_logic_vector(15 downto 0) := (others => '0');
    -- sync delay ring (C_LAT clocks): {0, field, vsync, hsync}
    type t_sr is array (0 to 63) of std_logic_vector(3 downto 0);
    signal sr : t_sr := (others => (others => '1'));
    attribute no_rw_check of sr : signal is true;
    signal sr_p  : unsigned(5 downto 0) := (others => '0');
    signal sr_wa, sr_ra : unsigned(5 downto 0) := (others => '0');
    signal sr_wd : std_logic_vector(3 downto 0) := (others => '1');
    signal sr_q, sr_q2 : std_logic_vector(3 downto 0) := (others => '1');

    ---------------------------------------------------------------------------
    -- timing, zone, width, controls
    ---------------------------------------------------------------------------
    signal t_vsx : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal t_vsy : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_ryi : unsigned(14 downto 0) := to_unsigned(3826, 15);
    signal t_winc : unsigned(16 downto 0) := to_unsigned(17476, 17);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(10 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(314 * 8, 13), to_unsigned(497 * 8, 13), to_unsigned(614 * 8, 13),
                         to_unsigned(410 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    signal s_m3, s_m4 : unsigned(8 downto 0) := (others => '0');
    signal s_n3, s_n4 : std_logic := '0';
    signal f_swp, f_twin, f_hlx, f_pal, f_osc : std_logic := '0';
    signal f_wv  : unsigned(13 downto 0) := to_unsigned(3840, 14);
    signal f_xcv : unsigned(13 downto 0) := to_unsigned(4072, 14);
    signal f_keyx4 : signed(12 downto 0) := to_signed(268, 13);
    signal f_kd  : signed(10 downto 0) := (others => '0');             -- K5 - 614
    signal f_ks  : signed(11 downto 0) := (others => '0');             -- (K5 - 614) x 1.125
    signal f_samp : unsigned(10 downto 0) := (others => '0');          -- sway amplitude (Q.2)
    signal f_x0t, f_x0 : unsigned(14 downto 0) := (others => '0');     -- twin: folded beam start
    signal f_xav : signed(14 downto 0) := to_signed(4344, 15);
    signal f_b   : unsigned(8 downto 0) := to_unsigned(128, 9);
    signal f_cm  : unsigned(5 downto 0) := to_unsigned(19, 6);   -- camera gain /64 on luma/4 (19 = shipped x0.297)
    signal f_sn  : unsigned(4 downto 0) := to_unsigned(16, 5);   -- stored-vase gain Q.4 (Osc)
    signal f_rb  : unsigned(13 downto 0) := to_unsigned(5304, 14);
    signal f_qb  : unsigned(16 downto 0) := to_unsigned(30180, 17);
    signal f_qinc : unsigned(18 downto 0) := to_unsigned(30180, 19);
    signal f_rspd, f_sspd : unsigned(9 downto 0) := (others => '0');
    signal f_roll : unsigned(15 downto 0) := (others => '0');
    signal f_phi  : unsigned(15 downto 0) := (others => '0');
    signal f_phi12 : unsigned(11 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- per-line engine
    ---------------------------------------------------------------------------
    signal le_run : std_logic := '0';
    signal le_sl  : unsigned(3 downto 0) := (others => '0');
    signal le_ph  : unsigned(4 downto 0) := (others => '0');
    signal le_end : unsigned(3 downto 0) := to_unsigned(7, 4);
    signal lm_a  : signed(21 downto 0) := (others => '0');
    signal lm_b  : unsigned(17 downto 0) := (others => '0');
    signal lm_h  : signed(22 downto 0) := (others => '0');
    signal lm_l  : unsigned(17 downto 0) := (others => '0');
    signal ld_d  : unsigned(10 downto 0) := (others => '1');
    signal ld_r  : unsigned(11 downto 0) := (others => '0');
    signal ld_q  : unsigned(17 downto 0) := (others => '0');
    signal n_vy  : unsigned(18 downto 0) := (others => '0');
    signal n_qacc : unsigned(23 downto 0) := (others => '0');
    signal n_ry  : unsigned(22 downto 0) := (others => '0');
    signal n_lf  : unsigned(15 downto 0) := x"ACE6";
    signal n_jlp, n_jrp : signed(4 downto 0) := (others => '0');
    signal n_jl, n_jr : signed(5 downto 0) := (others => '0');
    signal n_jc : signed(3 downto 0) := (others => '0');
    signal n_ck0, n_ck1 : signed(7 downto 0) := (others => '0');
    signal n_dk0, n_dk1 : signed(9 downto 0) := (others => '0');
    signal n_cvo : signed(8 downto 0) := (others => '0');
    signal n_dv  : signed(10 downto 0) := (others => '0');
    signal n_hwm : unsigned(10 downto 0) := (others => '0');
    signal n_hw  : unsigned(10 downto 0) := (others => '0');
    signal n_cvv : unsigned(13 downto 0) := (others => '0');
    signal n_L, n_R : signed(14 downto 0) := (others => '0');
    signal n_stepv : unsigned(17 downto 0) := (others => '0');
    signal n_step : unsigned(18 downto 0) := (others => '0');
    signal n_u0  : signed(29 downto 0) := (others => '0');
    signal n_V, n_top, n_vprev : std_logic := '0';
    signal n_swp : unsigned(15 downto 0) := (others => '0');           -- sway phase of the line
    signal n_swj : signed(11 downto 0) := (others => '0');             -- sway + centre jitter, Q.2
    signal n_ua  : signed(16 downto 0) := (others => '0');             -- u0 operand: centre - beam start
    signal n_bf, n_bd : unsigned(7 downto 0) := (others => '0');      -- bar phase (+ helix spin), duty
    signal n_bz  : std_logic_vector(1 downto 0) := (others => '0');    -- zone
    -- committed line values
    signal l_L, l_R : signed(14 downto 0) := (others => '0');
    signal l_step : unsigned(18 downto 0) := (others => '0');
    signal l_u0  : signed(29 downto 0) := (others => '0');
    signal l_A   : unsigned(4 downto 0) := (others => '0');
    signal l_BO  : signed(6 downto 0) := (others => '0');
    signal l_C   : signed(5 downto 0) := (others => '0');
    signal l_D0  : signed(4 downto 0) := (others => '0');
    signal l_M   : unsigned(3 downto 0) := (others => '0');
    signal l_V, l_top, l_par : std_logic := '0';
    signal l_bf, l_bd : unsigned(7 downto 0) := (others => '0');
    signal l_bz : std_logic_vector(1 downto 0) := (others => '0');
    signal s_ln  : unsigned(10 downto 0) := (others => '0');

    -- fill engine (next line's table row -> line RAM)
    signal fl_run : std_logic := '0';
    signal fl_c   : unsigned(6 downto 0) := (others => '0');
    signal fl_row : unsigned(5 downto 0) := (others => '0');
    signal fl_wy  : unsigned(3 downto 0) := (others => '0');
    signal fl_bank : std_logic := '0';
    signal fl_vs  : std_logic_vector(2 downto 0) := (others => '0');
    signal fl_odd : std_logic_vector(2 downto 0) := (others => '0');
    signal fl_t0, fl_t0d : unsigned(7 downto 0) := (others => '0');
    signal fl_p   : signed(13 downto 0) := (others => '0');
    signal fl_pv  : std_logic := '0';
    signal fl_v, fl_vk, fl_vp : unsigned(7 downto 0) := (others => '0');
    signal fl_k   : unsigned(6 downto 0) := (others => '0');
    signal fl_ma  : unsigned(6 downto 0) := (others => '0');
    signal fl_wv, fl_mid : std_logic := '0';
    -- fill write pipeline (K6 camera pass): writer -> fa (gain product, sheen, half) -> fb (clip, base) -> RAM
    signal fw_wa, fa_wa, fb_wa : unsigned(7 downto 0) := (others => '0');
    signal fw_wd : unsigned(7 downto 0) := (others => '0');
    signal fw_we, fa_we, fb_we : std_logic := '0';
    signal fa_p  : unsigned(12 downto 0) := (others => '0');
    signal fa_sh : unsigned(7 downto 0) := (others => '0');
    signal fa_h  : unsigned(6 downto 0) := (others => '0');
    signal fb_g, fb_b : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- generated avid
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- camera write side
    signal c_y1  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_av1, c_av2, c_av3 : std_logic := '0';
    signal c_p   : unsigned(13 downto 0) := (others => '0');
    signal c_d   : unsigned(9 downto 0) := (others => '0');
    signal c_wacc : unsigned(24 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    signal b_xv : unsigned(14 downto 0) := (others => '0');
    signal b_xf : unsigned(14 downto 0) := (others => '0');            -- folded beam (twin mirror)
    signal b_dn : std_logic := '0';
    signal b_u  : signed(29 downto 0) := (others => '0');
    signal p1_uq : signed(10 downto 0) := (others => '0');
    signal p1_eL, p1_eR, p1_dk : signed(15 downto 0) := (others => '0');
    signal p2_au : unsigned(9 downto 0) := (others => '0');
    signal p2_neg, p2_left : std_logic := '0';
    signal p2_d  : signed(13 downto 0) := (others => '0');
    signal p2_feL, p2_feR : unsigned(4 downto 0) := (others => '0');
    signal p2_hal : unsigned(3 downto 0) := (others => '0');
    signal p3_neg : std_logic := '0';
    signal p3_rA, p3_rB, p3_rG : unsigned(4 downto 0) := (others => '0');
    signal p3_e2, p3_fe : unsigned(4 downto 0) := (others => '0');
    signal p3_hal : unsigned(3 downto 0) := (others => '0');
    signal p4_th : unsigned(9 downto 0) := (others => '0');
    signal p4_neg : std_logic := '0';
    signal p4_kb, p4_kg, p4_ko : unsigned(4 downto 0) := (others => '0');
    signal p4_env : unsigned(4 downto 0) := (others => '0');
    signal p4_on : std_logic := '0';
    signal p4_hal : unsigned(3 downto 0) := (others => '0');
    signal p5_th2 : signed(11 downto 0) := (others => '0');
    signal p6_sneg, p7_sneg, p8_sneg : std_logic := '0';
    signal p8_ua : unsigned(9 downto 0) := (others => '0');
    signal p8_cam : unsigned(7 downto 0) := (others => '0');
    signal p9_cam, p10_cam, p11_cam : unsigned(7 downto 0) := (others => '0');
    signal p11_w : unsigned(7 downto 0) := (others => '0');
    signal p12_src : unsigned(8 downto 0) := (others => '0');
    signal p13_sg : unsigned(8 downto 0) := (others => '0');
    signal p14_s, p15_s, p16_s, p17_s, p18_s, p19_s : unsigned(8 downto 0) := (others => '0');
    signal p14_on, p15_on, p16_on, p17_on, p18_on, p19_on, p20_on, p21_on : std_logic := '0';
    signal p14_key, p15_key, p16_key, p17_key : unsigned(4 downto 0) := (others => '0');
    signal p14_hal, p15_hal, p16_hal, p17_hal, p18_hal : unsigned(3 downto 0) := (others => '0');
    signal p15_as : unsigned(13 downto 0) := (others => '0');
    signal p15_hs, p16_hs, p17_hs : unsigned(8 downto 0) := (others => '0');
    signal p16_bl, p17_bl : signed(10 downto 0) := (others => '0');
    signal p17_cr : std_logic_vector(63 downto 0) := (others => '0');
    signal p18_keff, p19_keff, p20_keff : unsigned(3 downto 0) := (others => '0');
    signal p18_h1 : signed(14 downto 0) := (others => '0');
    signal p18_h2 : signed(9 downto 0) := (others => '0');
    signal p18_bB, p19_bB : unsigned(7 downto 0) := (others => '0');
    signal p18_rrp : unsigned(7 downto 0) := (others => '0');
    signal p18_mgp : unsigned(7 downto 0) := (others => '0');
    signal p18_rg, p18_br, p18_r0 : unsigned(7 downto 0) := (others => '0');
    signal p19_dl : signed(10 downto 0) := (others => '0');
    signal p19_rR, p19_rG, p19_rB : unsigned(7 downto 0) := (others => '0');
    signal p20_bR, p20_bG, p20_bB, p20_rR, p20_rG, p20_rB : unsigned(7 downto 0) := (others => '0');
    signal p21_pR, p21_pG, p21_pB : signed(13 downto 0) := (others => '0');
    signal p21_bR, p21_bG, p21_bB : unsigned(7 downto 0) := (others => '0');
    signal p22_oR, p22_oG, p22_oB : unsigned(7 downto 0) := (others => '0');
    signal p23_y : unsigned(9 downto 0) := (others => '0');
    signal p23_cr, p23_cb : signed(10 downto 0) := (others => '0');
    signal p24_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p24_u8, p24_v8 : unsigned(7 downto 0) := to_unsigned(128, 8);
    -- output conditioning (chroma delay-8 ring in block RAM)
    type t_crg is array (0 to 255) of std_logic_vector(15 downto 0);
    signal crg : t_crg := (others => x"8080");
    attribute no_rw_check of crg : signal is true;
    signal cr_p, cr_wa, cr_ra : unsigned(7 downto 0) := (others => '0');
    signal cr_wd, cr_q : std_logic_vector(15 downto 0) := x"8080";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_b24h : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 p24 chroma samples
    signal s_y25, s_y26, s_y27, s_y28, s_y29, s_y30x, s_yf30 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u30, s_v30 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y31, s_y32 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered address, registered output) and RAMs (1W1R)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_tq  <= C_T(to_integer(r_ta));
            r_pcq <= C_PC(to_integer(r_pca));
            r_asq <= C_ASIN(to_integer(r_asa));
            r_snq <= C_SIN(to_integer(r_sna));
            r_crq <= C_CR(to_integer(r_cra));
        end if;
    end process p_rom;

    p_lr : process(clk)
    begin
        if rising_edge(clk) then
            lr_q <= lr(to_integer(lr_ra));
            if lr_we = '1' then lr(to_integer(lr_wa)) <= lr_wd; end if;
        end if;
    end process p_lr;

    p_cb : process(clk)
    begin
        if rising_edge(clk) then
            cb_q <= cb(to_integer(cb_ra));
            if cb_we = '1' then cb(to_integer(cb_wa)) <= cb_wd; end if;
        end if;
    end process p_cb;

    p_rg : process(clk)
    begin
        if rising_edge(clk) then
            rg_q <= rg(to_integer(rg_ra));
            rg(to_integer(rg_wa)) <= rg_wd;
        end if;
    end process p_rg;

    p_crg : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= crg(to_integer(cr_ra));
            crg(to_integer(cr_wa)) <= cr_wd;
        end if;
    end process p_crg;

    p_sr : process(clk)
    begin
        if rising_edge(clk) then
            sr_q <= sr(to_integer(sr_ra));
            sr(to_integer(sr_wa)) <= sr_wd;
        end if;
    end process p_sr;

    -- sync delay ring: written every clock, read C_LAT - 3 entries back, then two registers
    p_srctl : process(clk)
    begin
        if rising_edge(clk) then
            sr_p <= sr_p + 1;
            sr_wa <= sr_p;
            sr_wd <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            sr_ra <= sr_p - (C_LAT - 3);
            sr_q2 <= sr_q;
        end if;
    end process p_srctl;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, width, glides, per-frame values
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_t  : unsigned(19 downto 0);
        variable v_w  : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            t_vsx <= f_vsx(registers_in(8)(3 downto 0));
            t_vsy <= f_vsy(registers_in(8)(3 downto 0));
            t_ryi <= f_ryinc(registers_in(8)(3 downto 0));
            t_winc <= f_winc(registers_in(8)(3 downto 0));

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
            g_wm1 <= s_w - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- bipolar knobs K3 Roll, K4 Spin (deadband 8)
                v_x := signed(resize(unsigned(registers_in(2)), 11)) - 512;
                if v_x > 7 then s_n3 <= '0'; v_u := unsigned(v_x) - 8; s_m3 <= v_u(8 downto 0);
                elsif v_x < -7 then s_n3 <= '1'; v_u := unsigned(-v_x) - 8; s_m3 <= v_u(8 downto 0);
                else s_n3 <= '0'; s_m3 <= (others => '0'); end if;
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 7 then s_n4 <= '0'; v_u := unsigned(v_x) - 8; s_m4 <= v_u(8 downto 0);
                elsif v_x < -7 then s_n4 <= '1'; v_u := unsigned(-v_x) - 8; s_m4 <= v_u(8 downto 0);
                else s_n4 <= '0'; s_m4 <= (others => '0'); end if;
                f_swp  <= registers_in(6)(0);
                f_twin <= registers_in(6)(1);
                f_hlx  <= registers_in(6)(2);
                f_pal  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- roll and spin accumulate per field
                if s_n3 = '1' then f_roll <= f_roll - resize(f_rspd, 16); else f_roll <= f_roll + resize(f_rspd, 16); end if;
                if s_n4 = '1' then f_phi <= f_phi - resize(f_sspd, 16); else f_phi <= f_phi + resize(f_sspd, 16); end if;
            end if;
            f_phi12 <= f_phi(15 downto 4);

            -- glides: K1, K2, K5, P12, K6 (1/8 of the gap per field, deadband 4); 4 zone clocks each
            if s_zone = '1' and s_zc >= 4 and s_zc < 24 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(5)) & "000"; gl_o <= s_g(4);
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

            -- per-frame values (shift-add; settle long before the engine reads them)
            case to_integer(t_vsx) is
                when 4 => f_wv <= resize(s_w & "00", 14);
                when 3 => f_wv <= resize(s_w, 14) + resize(s_w & '0', 14);
                when others => f_wv <= resize(s_w & '0', 14);
            end case;
            v_w := shift_left(resize(f_wv, 19), 5) - resize(f_wv, 19);           -- wv * 31
            f_xcv <= f_wv + resize(v_w(18 downto 9), 14);
            -- K5 key centre: +67 px at 614 (the photo, exact), slope 1.125: -624 .. +527 virtual px
            v_k := s_g(2)(12 downto 3);
            f_kd <= signed(resize(v_k, 11)) - 614;
            f_ks <= resize(f_kd, 12) + resize(shift_right(f_kd, 3), 12);
            f_keyx4 <= shift_left(resize(f_ks, 13) + 67, 2);
            f_xav <= signed(resize(f_xcv, 15)) + resize(f_keyx4, 15);
            v_k := s_g(3)(12 downto 3);
            f_b <= resize(shift_right(resize(v_k, 12) + shift_left(resize(v_k, 12), 2), 4), 9);
            -- P12 above the photo (410): sway amplitude 2 (Q.2) per step, 0 .. 1226 (306 px)
            if v_k > 410 then f_samp <= (v_k - 410) & '0'; else f_samp <= (others => '0'); end if;
            -- twin: the beam folds at the screen centre Xc (= f_wv in Q.2) so that 3/4 W maps to the axis
            f_x0t <= resize(f_xcv, 15) + resize(f_wv(13 downto 1), 15);
            if f_twin = '1' then f_x0 <= f_x0t; else f_x0 <= (others => '0'); end if;
            -- K6 Camera (glided): camera gain m = k * 19 / 512 (x0 .. x2 of the shipped 19/64, 0 in Osc),
            -- stored-vase gain n = k / 32 (Q.4: x0 .. x1.94, x1 at 512)
            v_k := s_g(4)(12 downto 3);
            v_t := shift_left(resize(v_k, 20), 4) + shift_left(resize(v_k, 20), 1) + resize(v_k, 20);   -- k * 19
            if f_osc = '1' then f_cm <= (others => '0'); else f_cm <= v_t(14 downto 9); end if;
            f_sn <= v_k(9 downto 5);
            v_k := s_g(0)(12 downto 3);
            f_rb <= to_unsigned(1536, 14) + resize(v_k & "000", 14) + resize(v_k & "00", 14);
            v_k := s_g(1)(12 downto 3);
            if v_k <= 512 then
                f_qb <= to_unsigned(12288, 17) + resize(v_k & "00000", 17) + resize(v_k & "00", 17);
            else
                v_k := v_k - 512;
                f_qb <= to_unsigned(30720, 17) + resize(v_k & "000000", 17) + resize(v_k & "0000", 17) + resize(v_k & "000", 17);
            end if;

            -- generated-avid delay line (the gate and avid taps)
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
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
    -- Per-line engine.  Line program (slots 0..9) kicked at the source avid rise
    -- (line 0 at zone clock 1024) computes the NEXT line, committed at the next
    -- source avid rise.  Frame program (slots 10..12) kicked at zone clock 64.
    -- Slots 1-2 (new in v0.6): the sway, amp x 4h(1 - h) of the phase vy/4 + 4 x spin.
    -- Slots of 28 clocks: ph 0 load, 1..23 step, 26 write, 27 follow-up.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_s   : signed(22 downto 0);
        variable v_pr  : signed(40 downto 0);
        variable v_r   : unsigned(11 downto 0);
        variable v_t   : signed(16 downto 0);
        variable v_hw  : signed(14 downto 0);
        variable v_q   : unsigned(15 downto 0);
        variable v_v : std_logic;
        variable v_kick : std_logic;
        variable v_lf  : unsigned(15 downto 0);
        variable v_j   : unsigned(6 downto 0);
        variable v_cv  : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            v_kick := '0';
            if s_zgo = '1' then
                n_vy <= (others => '0');
                n_qacc <= (to_unsigned(2830, 16) - f_roll) & x"00";
                n_ry <= (others => '0');
                s_ln <= (others => '0');
            end if;
            if s_zone = '1' and s_zc = 1024 then
                v_kick := '1';
            end if;
            if data_in.avid = '1' and s_avq = '0' then
                l_L <= n_L; l_R <= n_R; l_step <= n_step; l_u0 <= n_u0;
                -- the curve word read in slot 6 stays in r_pcq2 until the next kick
                l_A <= unsigned(r_pcq2(26 downto 22)); l_BO <= signed(r_pcq2(21 downto 15));
                l_C <= signed(r_pcq2(14 downto 9)); l_D0 <= signed(r_pcq2(8 downto 4)); l_M <= unsigned(r_pcq2(3 downto 0));
                l_V <= n_V; l_top <= n_top;
                l_bf <= n_bf; l_bd <= n_bd; l_bz <= n_bz;
                l_par <= s_ln(0);
                s_ln <= s_ln + 1;
                v_kick := '1';
            end if;
            if v_kick = '1' then
                le_run <= '1'; le_sl <= (others => '0'); le_ph <= (others => '0'); le_end <= to_unsigned(9, 4);
            elsif s_zone = '1' and s_zc = 64 then
                le_run <= '1'; le_sl <= to_unsigned(10, 4); le_ph <= (others => '0'); le_end <= to_unsigned(12, 4);
            elsif le_run = '1' then
                if le_ph = 27 then
                    le_ph <= (others => '0');
                    if le_sl = le_end then le_run <= '0'; else le_sl <= le_sl + 1; end if;
                else
                    le_ph <= le_ph + 1;
                end if;
            end if;

            r_pcq2 <= r_pcq;

            if le_run = '1' then
                -- serial multiplier (ph 1..18, 22 x 18 bits) and divider (ph 1..23)
                if le_ph >= 1 and le_ph <= 18 then
                    if lm_b(0) = '1' then v_s := lm_h + resize(lm_a, 23); else v_s := lm_h; end if;
                    lm_h <= shift_right(v_s, 1);
                    lm_l <= v_s(0) & lm_l(17 downto 1);
                    lm_b <= '0' & lm_b(17 downto 1);
                end if;
                if le_ph >= 1 and le_ph <= 23 then
                    if le_ph = 1 then v_r := ld_r(10 downto 0) & '1'; else v_r := ld_r(10 downto 0) & '0'; end if;
                    if v_r >= resize(ld_d, 12) then
                        ld_r <= v_r - resize(ld_d, 12); ld_q <= ld_q(16 downto 0) & '1';
                    else
                        ld_r <= v_r; ld_q <= ld_q(16 downto 0) & '0';
                    end if;
                end if;
                v_pr := lm_h & signed(lm_l);

                -- slot 0: profile ROM reads, LFSR, fill parameters
                if le_sl = 0 then
                    if le_ph = 0 then
                        r_pca <= '0' & resize(n_vy(18 downto 12), 7);
                        if n_ry(22 downto 16) >= 63 then
                            fl_row <= to_unsigned(62, 6); fl_wy <= to_unsigned(15, 4);
                        else
                            fl_row <= n_ry(21 downto 16); fl_wy <= n_ry(15 downto 12);
                        end if;
                    end if;
                    if le_ph = 2 then n_swp <= n_vy(17 downto 2) + (f_phi(13 downto 0) & "00"); end if;   -- + 4 x spin
                    if le_ph = 1 then
                        if n_vy(18 downto 12) >= 68 then v_j := to_unsigned(68, 7); else v_j := resize(n_vy(18 downto 12), 7) + 1; end if;
                        r_pca <= '0' & v_j;
                    end if;
                    if le_ph = 3 then n_ck0 <= signed(r_pcq2(17 downto 10)); n_dk0 <= signed(r_pcq2(9 downto 0)); end if;
                    if le_ph = 4 then n_ck1 <= signed(r_pcq2(17 downto 10)); n_dk1 <= signed(r_pcq2(9 downto 0)); end if;
                    if le_ph >= 1 and le_ph <= 12 then
                        if n_lf(0) = '1' then v_lf := ('0' & n_lf(15 downto 1)) xor x"B400";
                        else v_lf := '0' & n_lf(15 downto 1); end if;
                        n_lf <= v_lf;
                    end if;
                    if le_ph = 13 then     -- edge comb (always on since v0.6)
                        v_t := signed(resize(n_lf(2 downto 0), 17)) + signed(resize(n_lf(5 downto 3), 17)) - 7;
                        n_jl <= resize(v_t, 6) + resize(n_jlp, 6); n_jlp <= resize(v_t, 5);
                        v_t := signed(resize(n_lf(8 downto 6), 17)) + signed(resize(n_lf(11 downto 9), 17)) - 7;
                        n_jr <= resize(v_t, 6) + resize(n_jrp, 6); n_jrp <= resize(v_t, 5);
                        n_jc <= signed(resize(n_lf(14 downto 12), 4)) - 3;
                    end if;
                end if;

                -- operand loads
                if le_ph = 0 then
                    lm_h <= (others => '0'); lm_l <= (others => '0');
                    ld_r <= (others => '0'); ld_q <= (others => '0');
                    case to_integer(le_sl) is
                        -- sway: p x (32767 - |p|) for the signed phase p (|p| as p xor sign: an odd parabola
                        -- sine, peak 2^28 at |p| = 2^14), then x the amplitude
                        when 1 =>
                            lm_a <= resize(signed(n_swp), 22);
                            lm_b <= resize(unsigned(std_logic_vector(n_swp(14 downto 0)) xnor (14 downto 0 => n_swp(15))), 18);
                        when 2 => lm_a <= resize(v_pr(29 downto 14), 22); lm_b <= resize(f_samp, 18);   -- slot 1's parabola, Q.14
                        when 3 => lm_a <= resize(n_ck1, 22) - resize(n_ck0, 22); lm_b <= resize(n_vy(11 downto 4), 18);
                        when 4 => lm_a <= resize(n_dk1, 22) - resize(n_dk0, 22); lm_b <= resize(n_vy(11 downto 4), 18);
                        when 5 => lm_a <= signed(resize(n_vy, 22)); lm_b <= to_unsigned(184, 18);
                        when 6 => lm_a <= resize(n_dv, 22); lm_b <= resize(f_b, 18);
                        when 7 => ld_d <= n_hw;
                        when 8 =>
                            lm_a <= resize(n_ua, 22); lm_b <= n_stepv;
                            v_q := n_qacc(23 downto 8);
                            r_pca <= '1' & v_q(15 downto 9);
                        when 9 => lm_a <= signed(resize(f_rb, 22)); lm_b <= resize(n_qacc(23 downto 8) - 2004, 18);
                        when 10 => lm_a <= signed(resize(f_qb, 22)); lm_b <= resize(t_vsy, 18);
                        when 11 => lm_a <= signed(resize(s_m3, 22)); lm_b <= resize(s_m3, 18);
                        when 12 => lm_a <= signed(resize(s_m4, 22)); lm_b <= resize(s_m4, 18);
                        when others => null;
                    end case;
                end if;

                -- results
                if le_ph = 26 then
                    case to_integer(le_sl) is
                        when 2 => n_swj <= v_pr(25 downto 14) + resize(n_jc, 12);
                        when 3 => n_cvo <= resize(n_ck0, 9) + resize(v_pr(16 downto 8), 9);
                        when 4 => n_dv <= resize(n_dk0, 11) + resize(v_pr(18 downto 8), 11);
                        when 5 => n_hwm <= to_unsigned(299, 11) + resize(unsigned(v_pr(26 downto 16)), 11);
                        when 6 =>
                            v_hw := signed(resize(n_hwm, 15)) + resize(v_pr(20 downto 7), 15);
                            if v_hw < 24 then n_hw <= to_unsigned(24, 11); else n_hw <= unsigned(v_hw(10 downto 0)); end if;
                        when 7 => n_stepv <= ld_q;
                        when 8 => n_u0 <= -resize(v_pr(31 downto 2), 30);
                        when 9 =>
                            -- zone (r_pcq2(28..27): 1 bars, 2 block) and bar duty (129 + 8 DU) from the curve word;
                            -- the pixel path re-tests the bar per pixel (Helix); the line flag keeps the bar-top lines
                            if f_hlx = '1' then n_bf <= unsigned(v_pr(23 downto 16)) + (f_phi12(6 downto 0) & '0');
                            else n_bf <= unsigned(v_pr(23 downto 16)); end if;
                            n_bz <= r_pcq2(28 downto 27);
                            n_bd <= to_unsigned(129, 8) + (unsigned(r_pcq2(31 downto 29)) & "000");
                            if (r_pcq2(28 downto 27) = "01" and unsigned(v_pr(23 downto 16)) < to_unsigned(129, 8) + (unsigned(r_pcq2(31 downto 29)) & "000"))
                               or r_pcq2(28 downto 27) = "10" then
                                v_v := '1';
                            else
                                v_v := '0';
                            end if;
                            v_v := v_v xor f_swp;
                            n_V <= v_v;
                            n_top <= v_v and not n_vprev and not f_hlx;
                            n_vprev <= v_v;
                        when 10 => f_qinc <= unsigned(v_pr(26 downto 8));
                        when 11 => f_rspd <= unsigned(v_pr(17 downto 8));
                        when 12 => f_sspd <= unsigned(v_pr(18 downto 9));
                        when others => null;
                    end case;
                end if;
                if le_ph = 27 then
                    case to_integer(le_sl) is
                        when 3 =>
                            v_cv := signed(resize(f_xcv, 16)) + resize(n_cvo, 16) + resize(n_swj, 16);
                            n_cvv <= unsigned(v_cv(13 downto 0));
                        -- u0 operand: the centre's distance from the beam start (0, or the twin's folded start)
                        when 4 => n_ua <= signed(resize(n_cvv, 17)) - signed(resize(f_x0, 17));
                        when 6 =>
                            n_L <= signed(resize(n_cvv, 15)) - signed(resize(n_hw, 15)) + resize(n_jl, 15);
                            n_R <= signed(resize(n_cvv, 15)) + signed(resize(n_hw, 15)) + resize(n_jr, 15);
                        when 7 =>
                            case to_integer(t_vsx) is
                                when 4 => n_step <= resize(n_stepv & '0', 19);
                                when 3 => n_step <= resize(n_stepv, 19) + resize(n_stepv(17 downto 1), 19);
                                when others => n_step <= resize(n_stepv, 19);
                            end case;
                        when 9 =>
                            n_vy <= n_vy + resize(t_vsy, 19);
                            n_qacc <= n_qacc + resize(f_qinc, 24);
                            n_ry <= n_ry + resize(t_ryi, 23);
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Fill engine: the next line's y-interpolated table row -> line RAM bank,
    -- 128 entries (columns at even addresses, midpoints at odd); kicked with slot 1
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_row : unsigned(5 downto 0);
        variable v_v : signed(9 downto 0);
        variable v_m : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            r_tq2 <= r_tq;
            fl_vs  <= fl_vs(1 downto 0) & '0';
            fl_odd <= fl_odd(1 downto 0) & '0';
            if le_run = '1' and le_sl = 1 and le_ph = 0 then
                fl_run <= '1'; fl_c <= (others => '0');
                fl_bank <= s_ln(0);
            elsif fl_run = '1' then
                if fl_c(0) = '0' then v_row := fl_row; else v_row := fl_row + 1; end if;
                r_ta <= v_row & fl_c(6 downto 1);
                fl_vs(0) <= '1'; fl_odd(0) <= fl_c(0);
                if fl_c = 127 then fl_run <= '0'; end if;
                fl_c <= fl_c + 1;
            end if;
            -- data for the address set at clock c is in r_tq2 at clock c + 3
            fl_pv <= '0';
            if fl_vs(2) = '1' then
                if fl_odd(2) = '0' then
                    fl_t0 <= unsigned(r_tq2);
                else
                    fl_p <= (signed(resize(unsigned(r_tq2), 9)) - signed(resize(fl_t0, 9))) * signed(resize(fl_wy, 5));
                    fl_t0d <= fl_t0;
                    fl_pv <= '1';
                end if;
            end if;
            fl_wv <= '0';
            if fl_pv = '1' then
                v_v := signed(resize(fl_t0d, 10)) + resize(shift_right(fl_p, 4), 10);
                fl_v <= unsigned(v_v(7 downto 0));
                fl_wv <= '1';
            end if;
            -- writer: v_k at 2k; the next clock (v_{k-1} + v_k) / 2 at 2k - 1; finally v_63 / 2 at 127
            fw_we <= '0'; fl_mid <= '0';
            if le_run = '1' and le_sl = 1 and le_ph = 0 then
                fl_k <= (others => '0');
            elsif fl_wv = '1' then
                fw_wa <= fl_bank & fl_k(5 downto 0) & '0';
                fw_wd <= fl_v;
                fw_we <= '1';
                fl_vk <= fl_v;
                if fl_k = 0 then fl_vp <= fl_v; else fl_mid <= '1'; end if;
                fl_ma <= (fl_k(5 downto 0) & '0') - 1;
                fl_k <= fl_k + 1;
            elsif fl_mid = '1' then
                v_m := resize(fl_vp, 9) + resize(fl_vk, 9);
                fw_wa <= fl_bank & fl_ma;
                fw_wd <= v_m(8 downto 1);
                fw_we <= '1';
                fl_vp <= fl_vk;
            elsif fl_k = 64 then
                fw_wa <= fl_bank & "1111111";
                fw_wd <= shift_right(fl_vk, 1);
                fw_we <= '1';
                fl_k <= fl_k + 1;
            end if;
            -- K6 camera pass: the line RAM holds the stage-12 base W.  Video: max(sheen, half);
            -- Osc: max(n * S, sheen, half) (= S at n = 1).  sheen = 2 (S - 150)+, half = S / 2.
            fa_wa <= fw_wa; fa_we <= fw_we;
            fa_p <= fw_wd * f_sn;
            if fw_wd > 150 then fa_sh <= shift_left(fw_wd - 150, 1); else fa_sh <= (others => '0'); end if;
            fa_h <= fw_wd(7 downto 1);
            fb_wa <= fa_wa; fb_we <= fa_we;
            if fa_p(12) = '1' then fb_g <= (others => '1'); else fb_g <= fa_p(11 downto 4); end if;
            if resize(fa_h, 8) > fa_sh then fb_b <= resize(fa_h, 8); else fb_b <= fa_sh; end if;
            lr_wa <= fb_wa; lr_we <= fb_we;
            if f_osc = '1' and fb_g > fb_b then lr_wd <= std_logic_vector(fb_g); else lr_wd <= std_logic_vector(fb_b); end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- Camera write: source luma -> 8-bit, resampled to 512 entries per line
    ---------------------------------------------------------------------------
    p_cam : process(clk)
        variable v_d : signed(10 downto 0);
        variable v_s : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            c_y1 <= unsigned(data_in.y);
            c_av1 <= data_in.avid;
            -- 1: luma above black
            v_d := signed(resize(c_y1, 11)) - 64;
            if v_d < 0 then c_d <= (others => '0'); else c_d <= unsigned(v_d(9 downto 0)); end if;
            c_av2 <= c_av1;
            -- 2: x K6 camera gain (m / 64; shipped 19 / 64 = 0.297 at 50 %)
            c_p <= c_d(9 downto 2) * f_cm;
            c_av3 <= c_av2;
            -- 3: clipped to 8 bits; write at the resampled address
            v_s := c_p(13 downto 4);
            if v_s > 255 then cb_wd <= (others => '1'); else cb_wd <= std_logic_vector(v_s(7 downto 0)); end if;
            if c_av3 = '1' then
                cb_wa <= l_par & c_wacc(24 downto 16);
                cb_we <= '1';
                c_wacc <= c_wacc + resize(t_winc, 25);
            else
                cb_we <= '0';
                c_wacc <= (others => '0');
            end if;
        end if;
    end process p_cam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u   : signed(19 downto 0);
        variable v_a   : signed(15 downto 0);
        variable v_e   : signed(18 downto 0);
        variable v_hd  : signed(15 downto 0);
        variable v_h   : signed(16 downto 0);
        variable v_kbl, v_kgl : signed(13 downto 0);
        variable v_t2  : signed(12 downto 0);
        variable v_tf  : signed(12 downto 0);
        variable v_t2m : signed(12 downto 0);
        variable v_k   : unsigned(4 downto 0);
        variable v_us  : signed(11 downto 0);
        variable v_tt  : signed(12 downto 0);
        variable v_tu  : unsigned(10 downto 0);
        variable v_p14 : unsigned(13 downto 0);
        variable v_dl  : signed(15 downto 0);
        variable v_bb  : signed(11 downto 0);
        variable v_o   : signed(10 downto 0);
        variable v_y   : unsigned(15 downto 0);
        variable v_c   : signed(16 downto 0);
        variable v_dy  : unsigned(9 downto 0);
        variable v_dyh : unsigned(9 downto 0);
        variable v_dy2 : unsigned(11 downto 0);
        variable v_sc  : unsigned(11 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dlu : unsigned(9 downto 0);
        variable v_hx  : unsigned(15 downto 0);
        variable v_oR, v_oG, v_oB : unsigned(7 downto 0);
        variable v_kill : unsigned(4 downto 0);
        variable v_cu, v_cv : unsigned(9 downto 0);
        variable v_bs, v_t8 : unsigned(7 downto 0);
        variable v_bv  : std_logic;
        variable v_xi  : signed(4 downto 0);
        variable v_st  : signed(29 downto 0);
    begin
        if rising_edge(clk) then
            -- 0: beam (virtual x in Q.2, u DDA).  b_xf is the beam the vase sees: = b_xv, or with Twin
            -- folded at the screen centre (it runs down from f_x0 to the centre, then up again; the u
            -- DDA follows it), so the left half mirrors the right
            if g_start = '1' then
                b_xv <= (others => '0'); b_xf <= f_x0; b_dn <= f_twin; b_u <= l_u0;
            elsif g_avid = '1' then
                b_xv <= b_xv + resize(t_vsx & '0', 15);
                -- down: -inc exactly for the beam, the one's complement -step - 1 for u (a sub-pixel drift)
                v_xi := signed('0' & t_vsx & '0');
                v_st := signed(resize(l_step, 30));
                if b_dn = '1' then v_xi := -v_xi; v_st := not v_st; end if;
                b_xf <= unsigned(signed(b_xf) + resize(v_xi, 15));
                b_u <= b_u + v_st;
                if b_xv >= f_wv then b_dn <= '0'; end if;
            end if;

            -- 1: u (Q.10, saturated); edge distances and key distance (virtual Q.2)
            v_u := b_u(29 downto 10);
            if v_u > 1023 then p1_uq <= to_signed(1023, 11);
            elsif v_u < -1023 then p1_uq <= to_signed(-1023, 11);
            else p1_uq <= v_u(10 downto 0); end if;
            p1_eL <= signed(resize(b_xf, 16)) - resize(l_L, 16);
            p1_eR <= resize(l_R, 16) - signed(resize(b_xf, 16));
            p1_dk <= signed(resize(b_xf, 16)) - resize(f_xav, 16);

            -- 2: |u| -> asin address; key distance; edge factors; halo
            if p1_uq < 0 then v_a := resize(-p1_uq, 16); p2_neg <= '1'; else v_a := resize(p1_uq, 16); p2_neg <= '0'; end if;
            p2_au <= unsigned(v_a(9 downto 0));
            r_asa <= unsigned(v_a(9 downto 1));
            if p1_dk < 0 then v_a := -p1_dk; p2_left <= '1'; else v_a := p1_dk; p2_left <= '0'; end if;
            p2_d <= resize(shift_right(v_a, 2), 14);
            p2_feL <= f_fe(p1_eL); p2_feR <= f_fe(p1_eR);
            v_hd := shift_right(p1_eL, 2);
            v_h := to_signed(36, 17) - shift_left(resize(v_hd, 17), 1);
            if p1_eL < 0 or v_h <= 0 then p2_hal <= (others => '0');
            elsif v_h >= 16 then p2_hal <= to_unsigned(14, 4);
            else
                v_e := resize(shift_left(v_h, 4) - shift_left(v_h, 1), 19);
                p2_hal <= unsigned(v_e(7 downto 4));
            end if;

            -- 3: (asin ROM out) key ramps; envelope parts
            if p2_left = '1' then v_kbl := to_signed(190, 14); v_kgl := to_signed(225, 14);
            else v_kbl := to_signed(183, 14); v_kgl := to_signed(183, 14); end if;
            p3_rA <= f_r(p2_d - 47);
            p3_rB <= f_r(v_kbl - p2_d);
            p3_rG <= f_r(p2_d - v_kgl);
            p3_e2 <= unsigned(C_E2(to_integer(p2_au(9 downto 4))));
            if p2_feL < p2_feR then p3_fe <= p2_feL; else p3_fe <= p2_feR; end if;
            p3_hal <= p2_hal; p3_neg <= p2_neg;

            -- 4: asin fabric copy; key parts; envelope
            p4_th <= unsigned(r_asq); p4_neg <= p3_neg;
            p4_kb <= f_min5(p3_rA, p3_rB); p4_kg <= p3_rG; p4_ko <= p3_rB;
            if p3_e2 < p3_fe then p4_env <= p3_e2; else p4_env <= p3_fe; end if;
            if p3_fe /= 0 then p4_on <= '1'; else p4_on <= '0'; end if;
            p4_hal <= p3_hal;

            -- 5: spun angle; key select; side ring write {on, env, key, hal}, read 8 back
            if p4_neg = '1' then
                p5_th2 <= signed(f_phi12) - signed(resize(p4_th, 12));
            else
                p5_th2 <= signed(f_phi12) + signed(resize(p4_th, 12));
            end if;
            -- bar or gap per pixel: the line's bar phase (+ 2 x spin with Helix) + 2 x asin(u) with Helix,
            -- against the zone's duty (rings: the line flag; helix: 32 bars per turn round the vase)
            v_t8 := p4_th(6 downto 0) & '0';
            if p4_neg = '1' then v_t8 := not v_t8; end if;          -- one's complement: 1/256 bar
            if f_hlx = '0' then v_t8 := (others => '0'); end if;
            v_bs := l_bf + v_t8;
            if (l_bz = "01" and v_bs < l_bd) or l_bz = "10" then v_bv := '1'; else v_bv := '0'; end if;
            v_bv := v_bv xor f_swp;
            -- on a bar-top line (rings only, so the bar is set) the key opens to the outer ramp
            if v_bv = '1' then
                if l_top = '1' then v_k := p4_ko; else v_k := p4_kb; end if;
            else
                v_k := p4_kg;
            end if;
            rg_p <= rg_p + 1;
            rg_wa <= rg_p;
            rg_wd <= '0' & p4_on & std_logic_vector(p4_env) & std_logic_vector(v_k) & std_logic_vector(p4_hal);
            rg_ra <= rg_p - 7;

            -- 6: fold (+ mirror); sin and camera addresses
            v_t2 := resize(p5_th2, 13);
            if v_t2 > 1024 then v_tf := to_signed(2048, 13) - v_t2;
            elsif v_t2 < -1024 then v_tf := to_signed(-2048, 13) - v_t2;
            else v_tf := v_t2; end if;
            v_t2m := v_t2;
            if v_tf < 0 then p6_sneg <= '1'; v_tf := -v_tf; else p6_sneg <= '0'; end if;
            if v_tf >= 1024 then r_sna <= to_unsigned(511, 9); else r_sna <= unsigned(v_tf(9 downto 1)); end if;
            v_t2m := v_t2m + 2048;
            cb_ra <= (not l_par) & unsigned(v_t2m(11 downto 3));

            -- 7: (ROM / buffer output registers)
            p7_sneg <= p6_sneg;
            -- 8: fabric copies
            p8_ua <= unsigned(r_snq); p8_sneg <= p7_sneg;
            p8_cam <= unsigned(cb_q);

            -- 9: table coordinate -> line RAM address
            if p8_sneg = '1' then v_us := -signed(resize(p8_ua, 12)); else v_us := signed(resize(p8_ua, 12)); end if;
            v_tt := resize(v_us, 13) + 1024;
            if v_tt < 0 then v_tu := (others => '0');
            elsif v_tt > 2047 then v_tu := (others => '1');
            else v_tu := unsigned(v_tt(10 downto 0)); end if;
            lr_ra <= l_par & v_tu(10 downto 4);
            p9_cam <= p8_cam;

            -- 10: (line RAM output register)
            p10_cam <= p9_cam;
            -- 11: fabric copy
            p11_w <= unsigned(lr_q); p11_cam <= p10_cam;

            -- 12: source: max(camera x K6, W) -- W from the fill engine: max(sheens, half) in Video,
            -- max(K6 x object, sheens, half) in Osc (the camera is 0 there)
            if p11_cam > p11_w then p12_src <= resize(p11_cam, 9); else p12_src <= resize(p11_w, 9); end if;

            -- 13: (v0.6.0 K6 Gloss, dropped in the K6 camera pass: x1)
            p13_sg <= p12_src;

            -- 14: envelope from the side ring (rg_q = this pixel's stage-5 bundle)
            v_p14 := p13_sg * unsigned(rg_q(13 downto 9));
            p14_s <= v_p14(12 downto 4);
            p14_on <= rg_q(14);
            p14_key <= unsigned(rg_q(8 downto 4)); p14_hal <= unsigned(rg_q(3 downto 0));

            -- 15: colour ROM address; blue gain product; halo blue
            p15_s <= p14_s;
            if p14_s > 255 then r_cra <= (others => '1'); else r_cra <= p14_s(7 downto 0); end if;
            p15_as <= l_A * p14_s;
            case to_integer(p14_hal(3 downto 2)) is
                when 0 => p15_hs <= (others => '0');
                when 1 => p15_hs <= resize(p14_s(8 downto 2), 9);
                when 2 => p15_hs <= resize(p14_s(8 downto 1), 9);
                when others => p15_hs <= resize(p14_s(8 downto 1), 9) + resize(p14_s(8 downto 2), 9);
            end case;
            p15_key <= p14_key; p15_hal <= p14_hal; p15_on <= p14_on;

            -- 16: (colour ROM output register) blue level
            p16_s <= p15_s;
            if p15_s > 8 then
                p16_bl <= signed(resize(p15_as(13 downto 4), 11)) + resize(l_BO, 11);
            else
                p16_bl <= signed(resize(p15_as(13 downto 4), 11));
            end if;
            p16_hs <= p15_hs; p16_key <= p15_key; p16_hal <= p15_hal; p16_on <= p15_on;

            -- 17: fabric copy of the colour ROM
            p17_cr <= r_crq; p17_s <= p16_s; p17_bl <= p16_bl; p17_hs <= p16_hs;
            p17_key <= p16_key; p17_hal <= p16_hal; p17_on <= p16_on;

            -- 18: key; hue products; base blue; red products
            -- C_CR: kill(63..59) ws(58..55) sw(54..47) knee(46..39) rg(38..31) br(30..23) r0(22..15) dr(14..9) mg(8..1)
            v_kill := unsigned(p17_cr(63 downto 59));
            v_k := f_min5(p17_key, v_kill);
            v_k := f_min5(v_k, to_unsigned(16, 5) - resize(p17_hal, 5));
            p18_keff <= v_k(4 downto 1);
            p18_h1 <= l_C * signed(resize(unsigned(p17_cr(54 downto 47)), 9));
            if p17_s <= 8 or unsigned(p17_cr(58 downto 55)) < 3 then
                p18_h2 <= (others => '0');
            elsif unsigned(p17_cr(58 downto 55)) < 6 then
                p18_h2 <= resize(shift_right(l_D0, 1), 10);
            else
                p18_h2 <= resize(l_D0, 10);
            end if;
            v_bb := resize(p17_bl, 12);
            if signed(resize(unsigned(p17_cr(46 downto 39)), 12)) > v_bb then v_bb := signed(resize(unsigned(p17_cr(46 downto 39)), 12)); end if;
            if signed(resize(p17_hs, 12)) > v_bb then v_bb := signed(resize(p17_hs, 12)); end if;
            p18_bB <= f_clip(v_bb, 255, 8);
            case to_integer(l_M) is
                when 0 => p18_rrp <= (others => '0'); p18_mgp <= (others => '0');
                when 2 => p18_rrp <= resize(unsigned(p17_cr(14 downto 11)), 8); p18_mgp <= resize(unsigned(p17_cr(8 downto 3)), 8);
                when 4 => p18_rrp <= resize(unsigned(p17_cr(14 downto 10)), 8); p18_mgp <= resize(unsigned(p17_cr(8 downto 2)), 8);
                when others => p18_rrp <= resize(unsigned(p17_cr(14 downto 9)), 8); p18_mgp <= unsigned(p17_cr(8 downto 1));
            end case;
            p18_rg <= unsigned(p17_cr(38 downto 31)); p18_br <= unsigned(p17_cr(30 downto 23)); p18_r0 <= unsigned(p17_cr(22 downto 15));
            p18_s <= p17_s; p18_hal <= p17_hal; p18_on <= p17_on;

            -- 19: hue offset; red R/B
            v_hx := resize(p18_hal, 8) * 10;
            p19_dl <= resize(shift_right(p18_h1, 7), 11) + resize(p18_h2, 11) - signed(resize(v_hx(11 downto 4), 11));
            p19_rR <= f_clip(signed(resize(p18_r0, 12)) + signed(resize(p18_rrp, 12)), 255, 8);
            p19_rB <= f_clip(signed(resize(p18_br, 12)) + signed(resize(p18_mgp, 12)), 255, 8);
            p19_rG <= p18_rg; p19_bB <= p18_bB; p19_s <= p18_s; p19_keff <= p18_keff; p19_on <= p18_on;

            -- 20: base R/G
            p20_bR <= f_clip(signed(resize(p19_s, 12)) + resize(p19_dl, 12), 255, 8);
            p20_bG <= f_clip(signed(resize(p19_s, 12)) - resize(p19_dl, 12), 255, 8);
            p20_bB <= p19_bB; p20_rR <= p19_rR; p20_rG <= p19_rG; p20_rB <= p19_rB;
            p20_keff <= p19_keff; p20_on <= p19_on;

            -- 21: (red - base) * key
            p21_pR <= (signed(resize(p20_rR, 9)) - signed(resize(p20_bR, 9))) * signed(resize(p20_keff, 5));
            p21_pG <= (signed(resize(p20_rG, 9)) - signed(resize(p20_bG, 9))) * signed(resize(p20_keff, 5));
            p21_pB <= (signed(resize(p20_rB, 9)) - signed(resize(p20_bB, 9))) * signed(resize(p20_keff, 5));
            p21_bR <= p20_bR; p21_bG <= p20_bG; p21_bB <= p20_bB; p21_on <= p20_on;

            -- 22: mix; edge gate; palette rotation
            v_o := signed(resize(p21_bR, 11)) + resize(shift_right(p21_pR, 3), 11); v_oR := f_clip(v_o, 255, 8);
            v_o := signed(resize(p21_bG, 11)) + resize(shift_right(p21_pG, 3), 11); v_oG := f_clip(v_o, 255, 8);
            v_o := signed(resize(p21_bB, 11)) + resize(shift_right(p21_pB, 3), 11); v_oB := f_clip(v_o, 255, 8);
            if p21_on = '0' then v_oR := (others => '0'); v_oG := (others => '0'); v_oB := (others => '0'); end if;
            if f_pal = '1' then
                p22_oR <= v_oB; p22_oG <= v_oR; p22_oB <= v_oG;
            else
                p22_oR <= v_oR; p22_oG <= v_oG; p22_oB <= v_oB;
            end if;

            -- 23: RGB -> YUV: (66 R + 129 G + 25 B, 112 R - 94 G - 18 B, -38 R - 74 G + 112 B) / 64
            v_y := resize(p22_oR * to_unsigned(66, 7), 16) + resize(p22_oG * to_unsigned(129, 8), 16) + resize(p22_oB * to_unsigned(25, 5), 16);
            p23_y <= v_y(15 downto 6);
            v_c := signed(resize(p22_oR * to_unsigned(112, 7), 17)) - signed(resize(p22_oG * to_unsigned(94, 7), 17)) - signed(resize(p22_oB * to_unsigned(18, 5), 17));
            p23_cr <= resize(shift_right(v_c, 6), 11);
            v_c := signed(resize(p22_oB * to_unsigned(112, 7), 17)) - signed(resize(p22_oG * to_unsigned(74, 7), 17)) - signed(resize(p22_oR * to_unsigned(38, 6), 17));
            p23_cb <= resize(shift_right(v_c, 6), 11);

            -- 24: luma curve (greys x1.25 so the 13/16 conditioning restores them; top compressed to 876)
            v_dy := p23_y;
            if v_dy <= 576 then
                v_dy2 := resize(v_dy, 12) + resize(v_dy(9 downto 2), 12);
            else
                v_dyh := v_dy - 576;
                v_dy2 := to_unsigned(720, 12) + resize(v_dyh(9 downto 1), 12)
                         + resize(v_dyh(9 downto 6), 12) + resize(v_dyh(9 downto 8), 12);
            end if;
            if v_dy2 > 876 then v_dy2 := to_unsigned(876, 12); end if;
            p24_y <= to_unsigned(64, 10) + resize(v_dy2, 10);
            v_cu := f_clip(resize(p23_cr, 12) + 512, 1023, 10);
            v_cv := f_clip(resize(p23_cb, 12) + 512, 1023, 10);
            -- chroma neutral off the generated avid, so the box sums resync every blanking interval
            if s_gav_sr(22) = '1' then
                p24_u8 <= v_cu(9 downto 2); p24_v8 <= v_cv(9 downto 2);
            else
                p24_u8 <= to_unsigned(128, 8); p24_v8 <= to_unsigned(128, 8);
            end if;
            s_b24h <= s_b24h(6 downto 0) & s_gav_sr(22);

            -- 25..30: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 from 8-bit chroma and a delay-8
            -- ring; luma [1 2 1])
            cr_p <= cr_p + 1;
            cr_wa <= cr_p; cr_wd <= std_logic_vector(p24_u8) & std_logic_vector(p24_v8);
            cr_ra <= cr_p - 6;
            if s_b24h = "00000000" then
                s_boxu <= to_unsigned(1024, 11); s_boxv <= to_unsigned(1024, 11);
            else
                s_boxu <= s_boxu + resize(p24_u8, 11) - resize(unsigned(cr_q(15 downto 8)), 11);
                s_boxv <= s_boxv + resize(p24_v8, 11) - resize(unsigned(cr_q(7 downto 0)), 11);
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 12) + s_boxud; s_u30 <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 12) + s_boxvd; s_v30 <= v_sc(11 downto 2);
            s_y25 <= p24_y; s_y26 <= s_y25; s_y27 <= s_y26; s_y28 <= s_y27;
            s_y29 <= s_y28; s_y30x <= s_y29;
            v_sy := resize(s_y28, 12) + shift_left(resize(s_y29, 12), 1) + resize(s_y30x, 12);
            s_yf30 <= v_sy(11 downto 2);
            -- 31: luma toward black (white 940 -> 777); blanking gate
            if s_yf30 > 64 then v_dlu := s_yf30 - 64; else v_dlu := (others => '0'); end if;
            if s_gav_sr(29) = '1' then
                s_y31 <= s_yf30 - shift_right(v_dlu, 3) - shift_right(v_dlu, 4);
                s_u31 <= s_u30; s_v31 <= s_v30;
            else
                s_y31 <= to_unsigned(64, 10); s_u31 <= to_unsigned(512, 10); s_v31 <= to_unsigned(512, 10);
            end if;
            -- 32: output register
            s_y32 <= s_y31; s_u32 <= s_u31; s_v32 <= s_v31;

        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y32);
    data_out.u       <= std_logic_vector(s_u32);
    data_out.v       <= std_logic_vector(s_v32);
    data_out.hsync_n <= sr_q2(0);
    data_out.vsync_n <= sr_q2(1);
    data_out.field_n <= sr_q2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture vase;
