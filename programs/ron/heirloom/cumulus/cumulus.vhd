-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cumulus v0.1.0 -- Heirloom (phase 3, video clips), rebuilt from the clip WIN_20231009_00_32_55 at 0.5 s.
--
-- The still: two mirrored tiles of a blue sky (0, 47, 255) holding mint clouds -- an ellipse on the tile seam,
-- two tilted petals, a heart and a teardrop on the tile centre -- each trailed by a dark-blue shadow; below it a
-- yellow / green band whose green "X" wedges sit inside a scalloped bulb, pink around it with white spikes
-- rising from the bottom and magenta fringes; a band of the same colours on top, thin blue tracking lines.  In
-- the clip the bands roll up 5 lines per field and the clouds rise 1.4 lines per field.
--
-- The patch, as read (cumulus.py has the tables, .heirloom_work/rb_cumulus/notes.md every number):
--   hq     = the folded H triangle: two mirrored tiles (axis x 521, period 939 px), 0 .. 1023 per half tile
--   clouds = per line a chord |hq - c| < w; c and w come from the blob tape (512 entries over 1280 lines, the
--            still's rows), Lean pulls c toward the seam / centre, Billow scales w
--   zones  = Psi = Yz - K t^2 (t = 1 - hq/1023) through the zone tape: cloud [50, 750), band [750, 1044),
--            forest [1044, 1600), band [1600, 2098); K = the cusp depth per line (84 at the cloud's foot)
--   colour = cloud: mint / blue (shadow after each cloud, ripple); band: pink + white spikes / R-gradient with
--            dark-green spikes, magenta on the pink -> band edge; forest: a cyan comb, pink / white + spikes
--   camera = luma swells the clouds and bends the zone edges
--
--   K1  Tiles      mirrored tiles across: x1/4 .. x4 (2.045 at 512)
--   K2  Lean       the clouds' centre line: all on the seam (0) .. the still (512) .. hugging the centre (1023)
--   K3  Scallop    the zone edges' cusp depth: inverted arcs (0) .. flat (341) .. the still (512) .. x4 (1023)
--   K4  Speed      bipolar: the bands roll 5 lines / field and the clouds 1.33 at 768 (the clip)
--   K5  Spikes     the spikes' count (x0.35 .. x2.8) and, right of centre, how high they reach
--   K6  Etch       how far the camera (or the Osc) swells the clouds and bends the band edges
--   S7  Twin       a second, mirrored set of clouds (seam <-> centre)
--   S8  Palette    Cumulus (the still) / Orchid (purple sky, cream clouds, yellow and orange band: the 33_43 clip)
--   S9  Invert     the cloud key inverted: mint sky with blue clouds, pink band with yellow bulbs
--   S10 Slip       sync slips: 64-line bands of the picture torn sideways, re-torn about once a second
--   S11 Source     Video / Osc
--   P12 BILLOW     cloud size: clear sky (0) .. the still (400) .. towering clouds (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a per-line engine kicked at
-- each line start (computes the NEXT line on one serial multiplier: the blob tape read and interpolation, Lean,
-- Billow, the cusp depth, the band gradient line, the spike depth, the comb, the slip), a pixel path (the tile
-- DDA folded into hq, chord tests, t^2 x K, the zone compare, spikes, the run counter for shadows / fringes,
-- a palette ROM).  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output
-- conditioning (Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box, its 8-back tap in a block-RAM ring, with the
-- running-sum resync; luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cumulus_pkg.all;
use work.all;

architecture cumulus of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    constant C_TS  : integer := 8936;      -- tile phase per h at 2.045 tiles (2^24 per tile)
    constant C_PH  : integer := 7843348;   -- tile phase at virtual x 960 (0.4675 tile: axis at x 521)
    constant C_KY  : integer := 64;
    constant C_KU  : integer := 512;

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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_bl_a  : unsigned(8 downto 0) := (others => '0');
    signal r_bl_q  : std_logic_vector(23 downto 0) := (others => '0');
    signal r_t2_a  : unsigned(9 downto 0) := (others => '0');
    signal r_t2_q  : std_logic_vector(11 downto 0) := (others => '0');
    signal r_sc_a  : unsigned(7 downto 0) := (others => '0');
    signal r_sc_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal r_pl_a  : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q  : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(384 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_pal, s_inv, s_slip, s_src : std_logic := '0';

    -- per-field scratch
    signal z_e1, z_e5 : signed(10 downto 0) := (others => '0');
    signal z_sh1, z_sh5 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt5 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_d5, z_dp, z_d3 : unsigned(9 downto 0) := (others => '0');
    signal z_n5, z_np, z_n3 : std_logic := '0';
    signal z_l2 : unsigned(9 downto 0) := (others => '0');
    signal z_m2 : unsigned(17 downto 0) := (others => '0');
    signal z_x1 : unsigned(11 downto 0) := to_unsigned(1920, 12);   -- 1920 - x0h

    -- per-field constants
    signal c_tsh   : unsigned(16 downto 0) := to_unsigned(C_TS, 17);       -- tile phase per h
    signal c_tpx   : unsigned(18 downto 0) := to_unsigned(2 * C_TS, 19);   -- per pixel
    signal c_ph0   : unsigned(23 downto 0) := to_unsigned(7463444, 24);       -- (C_PH - 1920 C_TS) mod 2^24
    signal c_kq    : unsigned(9 downto 0) := to_unsigned(197, 10);         -- spike phase per hq (Q8 of a spike)
    signal c_tip   : signed(11 downto 0) := to_signed(902, 12);            -- spike tip line (zone tape)
    signal c_vz    : unsigned(12 downto 0) := to_unsigned(1281, 13);       -- zone tape lines per field, Q8
    signal c_vb    : unsigned(10 downto 0) := to_unsigned(341, 11);        -- blob tape lines per field, Q8
    signal c_L     : unsigned(9 downto 0) := to_unsigned(512, 10);         -- Lean, Q9
    signal c_G     : unsigned(9 downto 0) := to_unsigned(256, 10);         -- Billow gain, Q8
    signal c_O     : signed(9 downto 0) := (others => '0');                -- Billow offset (hq)
    signal c_S     : signed(10 downto 0) := to_signed(255, 11);            -- Scallop, Q8
    signal c_ge    : unsigned(9 downto 0) := to_unsigned(144, 10);         -- Etch gain
    signal c_rstep : unsigned(15 downto 0) := to_unsigned(5041, 16);       -- ripple phase per line
    signal c_cx0   : unsigned(15 downto 0) := (others => '0');             -- comb phase at pixel 0 (x0h * 205)
    signal c_rx0   : unsigned(15 downto 0) := (others => '0');             -- ripple phase at pixel 0 (x0h * 30)
    signal c_cstep : unsigned(9 downto 0) := to_unsigned(410, 10);         -- comb phase per pixel
    signal c_rpx   : unsigned(6 downto 0) := to_unsigned(60, 7);           -- ripple phase per pixel
    signal c_sd29  : unsigned(7 downto 0) := (others => '0');              -- slip seed * 29
    signal s_phz   : unsigned(18 downto 0) := (others => '0');             -- zone tape phase, Q8 lines (mod 2048)
    signal s_phb   : unsigned(18 downto 0) := (others => '0');             -- blob tape phase, Q8 lines (mod 1280)
    signal s_fc    : unsigned(11 downto 0) := (others => '0');             -- field counter

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld1, z_ld : std_logic := '0';
    signal z_oh  : std_logic_vector(14 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');           -- line being computed, Q8 virtual lines
    signal e_rpl  : unsigned(15 downto 0) := (others => '0');           -- ripple phase of that line
    signal e_yv   : unsigned(10 downto 0) := (others => '0');
    signal e_yz   : unsigned(10 downto 0) := (others => '0');
    signal e_ybr  : unsigned(11 downto 0) := (others => '0');
    signal e_yb   : unsigned(10 downto 0) := (others => '0');
    signal e_ka   : unsigned(7 downto 0) := (others => '0');
    signal e_ya   : unsigned(10 downto 0) := (others => '0');
    signal e_sl   : unsigned(10 downto 0) := (others => '0');
    signal e_slneg : std_logic := '0';
    signal e_kd   : unsigned(10 downto 0) := (others => '0');
    signal e_f    : unsigned(2 downto 0) := (others => '0');
    signal e_e1   : unsigned(8 downto 0) := (others => '0');
    signal e_q0, e_q1 : std_logic_vector(23 downto 0) := (others => '0');
    signal e_c0, e_w0 : signed(11 downto 0) := (others => '0');
    signal e_dc, e_dw : signed(12 downto 0) := (others => '0');
    signal e_ic, e_iw : signed(15 downto 0) := (others => '0');
    signal e_cc, e_ww : signed(12 downto 0) := (others => '0');
    signal e_dl   : signed(13 downto 0) := (others => '0');
    signal e_dlneg, e_wneg : std_logic := '0';
    signal e_k0   : signed(9 downto 0) := (others => '0');
    signal e_cl, e_wl : signed(14 downto 0) := (others => '0');
    signal e_kk   : signed(11 downto 0) := (others => '0');
    signal e_yd   : signed(12 downto 0) := (others => '0');
    signal e_w0q  : unsigned(6 downto 0) := (others => '0');
    signal e_d    : unsigned(11 downto 0) := (others => '0');
    signal e_hd   : unsigned(13 downto 0) := (others => '0');
    signal e_sx   : signed(12 downto 0) := (others => '0');
    signal e_s1   : unsigned(11 downto 0) := (others => '0');
    signal e_j1   : unsigned(15 downto 0) := (others => '0');
    signal e_jit  : unsigned(5 downto 0) := (others => '0');
    signal e_cph  : unsigned(15 downto 0) := (others => '0');
    signal e_sv   : unsigned(7 downto 0) := (others => '0');
    signal e_sv2  : unsigned(7 downto 0) := (others => '0');
    signal e_ph0  : unsigned(23 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    -- hand-over (written at the end of the engine run, loaded by the beam at the next line start)
    signal n_c, n_c2, n_w : signed(11 downto 0) := (others => '0');
    signal n_k    : signed(10 downto 0) := (others => '0');
    signal n_yz   : unsigned(10 downto 0) := (others => '0');
    signal n_hd   : unsigned(10 downto 0) := (others => '0');
    signal n_w0   : unsigned(6 downto 0) := (others => '0');
    signal n_s1   : unsigned(7 downto 0) := (others => '0');
    signal n_sd   : unsigned(7 downto 0) := (others => '0');
    signal n_trk, n_comb, n_cw, n_crow : std_logic := '0';
    signal n_cph0, n_rp0 : unsigned(15 downto 0) := (others => '0');
    signal n_ph0  : unsigned(23 downto 0) := (others => '0');
    -- per-line values in use by the pixel path
    signal l_c, l_c2, l_w : signed(11 downto 0) := (others => '0');
    signal l_kneg : std_logic := '0';
    signal l_km   : unsigned(9 downto 0) := (others => '0');
    signal l_yz   : unsigned(10 downto 0) := (others => '0');
    signal l_hd   : unsigned(10 downto 0) := (others => '0');
    signal l_w0   : unsigned(6 downto 0) := (others => '0');
    signal l_s1, l_sd : unsigned(7 downto 0) := (others => '0');
    signal l_trk, l_comb, l_cw, l_crow : std_logic := '0';

    -- beam: pixel count (stage 1); tile / comb / ripple DDAs (stage 5)
    signal b_gsr : std_logic_vector(3 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_ph  : unsigned(23 downto 0) := (others => '0');
    signal b_cph, b_rp : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(13 downto 0) := (others => '0');
    signal p5_le, p6_le, p7_le, p8_le : unsigned(17 downto 0) := (others => '0');
    signal p6_hq  : unsigned(9 downto 0) := (others => '0');
    signal p7_a1, p7_a2, p7_ab : unsigned(12 downto 0) := (others => '0');
    signal p7_hs  : signed(10 downto 0) := (others => '0');
    signal p8_d1, p8_d2 : signed(13 downto 0) := (others => '0');
    signal p8_g   : signed(13 downto 0) := (others => '0');
    signal p8_s0, p8_s1 : signed(15 downto 0) := (others => '0');
    signal p8_s2  : signed(13 downto 0) := (others => '0');
    signal p9_d   : signed(13 downto 0) := (others => '0');
    signal p9_lv  : signed(13 downto 0) := (others => '0');
    signal p9_sp  : signed(21 downto 0) := (others => '0');
    signal p9_t2  : unsigned(11 downto 0) := (others => '0');
    signal p9_eb  : unsigned(10 downto 0) := (others => '0');
    signal p9_ez  : unsigned(8 downto 0) := (others => '0');
    signal p10_dd : signed(14 downto 0) := (others => '0');
    signal p10_lv : unsigned(5 downto 0) := (others => '0');
    signal p10_k0, p10_k1, p10_k2 : unsigned(13 downto 0) := (others => '0');
    signal p10_ze : unsigned(10 downto 0) := (others => '0');
    signal p11_key : std_logic := '0';
    signal p11_p  : unsigned(9 downto 0) := (others => '0');
    signal p11_ze : unsigned(10 downto 0) := (others => '0');
    signal p12_key, p12_prev : std_logic := '0';
    signal p12_rc : unsigned(8 downto 0) := (others => '1');
    signal p12_psi : unsigned(10 downto 0) := (others => '0');
    signal p12_sc : unsigned(7 downto 0) := (others => '0');
    signal p13_zone : unsigned(1 downto 0) := (others => '0');
    signal p13_sw, p13_sd, p13_mag, p13_key : std_logic := '0';
    signal p13_sl : unsigned(1 downto 0) := (others => '0');
    -- carried bits: comb pattern, ripple level (from stage 6), level (from 10), key (from 11)
    type t_c4 is array (6 to 13) of unsigned(3 downto 0);
    signal q_cr   : t_c4 := (others => (others => '0'));     -- {comb pattern, ripple(2:0)}
    type t_c6 is array (11 to 13) of unsigned(5 downto 0);
    signal q_lv   : t_c6 := (others => (others => '0'));
    signal p17_c  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning (one box; its 8-back samples from a block-RAM ring)
    type t_ring is array (0 to 255) of std_logic_vector(29 downto 0);
    signal ob : t_ring := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ob : signal is true;
    signal ob_wa : unsigned(7 downto 0) := (others => '0');
    signal ob_ra : unsigned(7 downto 0) := (others => '0');
    signal ob_q, ob_r : std_logic_vector(29 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19, s_y20, s_u20, s_v20, s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- computing stage N reads s_gav_sr(N - 3)

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

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= unsigned(C_CU_EXP2(to_integer(r_ex_a)));
            r_bl_q <= C_CU_BLOB(to_integer(r_bl_a));
            r_t2_q <= C_CU_T2(to_integer(r_t2_a));
            r_sc_q <= C_CU_SC(to_integer(r_sc_a));
            r_pl_q <= C_CU_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_fa : unsigned(35 downto 0);
        variable v_fb : unsigned(15 downto 0);
        variable v_pb : unsigned(18 downto 0);
        variable v_k  : unsigned(9 downto 0);
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

            -- K4 Speed: bipolar, +-8 deadband, |v| - 8 (square law on the multiplier)
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;     -- |v| - 8
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_twin <= registers_in(6)(0);
                s_pal  <= registers_in(6)(1);
                s_inv  <= registers_in(6)(2);
                s_slip <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
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

            -- serial products: slot k loads at zc = 33 + 24 k, 16 iterations, result read at 50 + 24 k.  The
            -- slot number is set at 30 + 24 k, decoded one-hot at 31, the operands fetched at 32.
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
                    when others => null;
                end case;
            end if;
            z_ld1 <= z_pre; z_ld <= z_ld1;
            for i in 0 to 14 loop
                if z_sel = i then z_oh(i) <= '1'; else z_oh(i) <= '0'; end if;
            end loop;
            v_fa := (others => '0'); v_fb := (others => '0');
            if z_oh(0)  = '1' then v_fa := v_fa or to_unsigned(C_TS, 36);          v_fb := v_fb or z_mt1;                    end if;  -- tiles
            if z_oh(1)  = '1' then v_fa := v_fa or resize(z_x1, 36);               v_fb := v_fb or c_tsh(15 downto 0);       end if;  -- (1920 - x0h) tsh
            if z_oh(2)  = '1' then v_fa := v_fa or to_unsigned(197, 36);           v_fb := v_fb or z_mt5;                    end if;  -- spike phase
            if z_oh(3)  = '1' then v_fa := v_fa or resize(z_d5, 36);               v_fb := v_fb or to_unsigned(154, 16);     end if;  -- tip
            if z_oh(4)  = '1' then v_fa := v_fa or resize(s_vm, 36);               v_fb := v_fb or resize(s_vm, 16);         end if;  -- speed^2
            if z_oh(5)  = '1' then v_fa := v_fa or resize(z_m2, 36);               v_fb := v_fb or to_unsigned(1365, 16);    end if;  -- zone rate
            if z_oh(6)  = '1' then v_fa := v_fa or resize(z_m2, 36);               v_fb := v_fb or to_unsigned(364, 16);     end if;  -- blob rate
            if z_oh(7)  = '1' then v_fa := v_fa or resize(z_l2, 36);               v_fb := v_fb or to_unsigned(307, 16);     end if;  -- lean
            if z_oh(8)  = '1' then v_fa := v_fa or resize(s_gp(12 downto 3), 36);  v_fb := v_fb or to_unsigned(164, 16);     end if;  -- billow gain
            if z_oh(9)  = '1' then v_fa := v_fa or resize(z_dp, 36);               v_fb := v_fb or to_unsigned(56, 16);      end if;  -- billow offset
            if z_oh(10) = '1' then v_fa := v_fa or resize(z_d3, 36);               v_fb := v_fb or to_unsigned(383, 16);     end if;  -- scallop
            if z_oh(11) = '1' then v_fa := v_fa or resize(s_g6(12 downto 3), 36);  v_fb := v_fb or resize(s_g6(12 downto 3), 16); end if;  -- etch^2
            if z_oh(12) = '1' then v_fa := v_fa or resize(s_ys, 36);               v_fb := v_fb or to_unsigned(5041, 16);    end if;  -- ripple / line
            if z_oh(13) = '1' then v_fa := v_fa or resize(s_x0, 36);               v_fb := v_fb or to_unsigned(205, 16);     end if;  -- comb at pixel 0
            if z_oh(14) = '1' then v_fa := v_fa or resize(s_x0, 36);               v_fb := v_fb or to_unsigned(30, 16);      end if;  -- ripple at pixel 0
            z_fa <= v_fa; z_fb <= v_fb;
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
                    -- motion (once per field), exponents and exp2 lookups (address at k, data at k + 2)
                    when 20 =>
                        z_e1 <= signed(resize(s_g1(12 downto 3), 11)) - 512;              -- tiles: +-2 octaves
                        v_s := signed(resize(s_g5(12 downto 3), 11)) - 512;                -- spike count: +-1.5 octaves
                        z_e5 <= v_s - shift_right(v_s, 2);
                        z_x1 <= to_unsigned(1920, 12) - resize(s_x0, 12);
                    when 21 =>
                        r_ex_a <= unsigned(z_e1(7 downto 0)); z_sh1 <= z_e1(10 downto 8);
                        -- |k5 - 512|, |p12 - 400|, |k3 - 341|, the lean's upper slope
                        v_k := s_g5(12 downto 3);
                        if v_k >= 512 then z_d5 <= v_k - 512; z_n5 <= '0'; else z_d5 <= 512 - v_k; z_n5 <= '1'; end if;
                        v_k := s_g3(12 downto 3);
                        if v_k >= 341 then z_d3 <= v_k - 341; z_n3 <= '0'; else z_d3 <= 341 - v_k; z_n3 <= '1'; end if;
                        v_k := s_g2(12 downto 3);
                        if v_k > 512 then z_l2 <= v_k - 512; else z_l2 <= (others => '0'); end if;
                        c_sd29 <= resize(shift_left(resize(s_fc(11 downto 6), 8), 5) - shift_left(resize(s_fc(11 downto 6), 8), 1)
                                  - resize(s_fc(11 downto 6), 8), 8);
                    when 23 =>
                        z_mt1 <= r_ex_q;
                        r_ex_a <= unsigned(z_e5(7 downto 0)); z_sh5 <= z_e5(10 downto 8);
                    when 25 =>
                        z_mt5 <= r_ex_q;
                        -- per-mode steps: comb 205 / ripple 30 per h unit
                        case to_integer(s_xs) is
                            when 4 =>      c_cstep <= to_unsigned(820, 10); c_rpx <= to_unsigned(120, 7);
                            when 3 =>      c_cstep <= to_unsigned(615, 10); c_rpx <= to_unsigned(90, 7);
                            when others => c_cstep <= to_unsigned(410, 10); c_rpx <= to_unsigned(60, 7);
                        end case;

                    when 40 =>                    -- |p12 - 400|, after the glide has written P12 (zc ~27)
                        v_k := s_gp(12 downto 3);
                        if v_k >= 400 then z_dp <= v_k - 400; z_np <= '0'; else z_dp <= 400 - v_k; z_np <= '1'; end if;

                    -- slot results
                    when 50 =>                    -- tile phase per h = C_TS * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_tsh <= resize(f_acc(35 downto 17), 17);
                            when -1     => c_tsh <= resize(f_acc(35 downto 16), 17);
                            when 0      => c_tsh <= resize(f_acc(35 downto 15), 17);
                            when others => c_tsh <= resize(f_acc(35 downto 14), 17);
                        end case;
                    when 51 =>
                        case to_integer(s_xs) is
                            when 4 =>      c_tpx <= shift_left(resize(c_tsh, 19), 2);
                            when 3 =>      c_tpx <= shift_left(resize(c_tsh, 19), 1) + resize(c_tsh, 19);
                            when others => c_tpx <= shift_left(resize(c_tsh, 19), 1);
                        end case;
                    when 74 => c_ph0 <= to_unsigned(C_PH, 24) - f_acc(23 downto 0);
                    when 98 =>                    -- spike phase per hq = 197 * mt >> (15 - sh)
                        case to_integer(z_sh5) is
                            when -2     => c_kq <= resize(f_acc(35 downto 17), 10);
                            when -1     => c_kq <= resize(f_acc(35 downto 16), 10);
                            when 0      => c_kq <= resize(f_acc(35 downto 15), 10);
                            when others => c_kq <= resize(f_acc(35 downto 14), 10);
                        end case;
                    when 122 =>                   -- tip = 902 - (k5 - 512) * 0.6 right of centre, else 902
                        if z_n5 = '1' then c_tip <= to_signed(902, 12);
                        else c_tip <= to_signed(902, 12) - signed(resize(f_acc(17 downto 8), 12)); end if;
                    when 146 => z_m2 <= f_acc(17 downto 0);
                    when 170 => c_vz <= f_acc(28 downto 16);
                    when 194 => c_vb <= f_acc(26 downto 16);
                    when 218 =>                   -- lean: k2 up to 512, then 512 + (k2 - 512) * 0.6
                        if s_g2(12 downto 3) > 512 then c_L <= to_unsigned(512, 10) + resize(f_acc(18 downto 9), 10);
                        else c_L <= s_g2(12 downto 3); end if;
                    when 242 => c_G <= f_acc(17 downto 8);
                    when 266 =>
                        if z_np = '1' then c_O <= -signed(resize(f_acc(16 downto 8), 10));
                        else c_O <= signed(resize(f_acc(16 downto 8), 10)); end if;
                    when 290 =>
                        if z_n3 = '1' then c_S <= -signed(resize(f_acc(17 downto 8), 11));
                        else c_S <= signed(resize(f_acc(17 downto 8), 11)); end if;
                    when 314 => c_ge <= f_acc(19 downto 10);
                    when 338 => c_rstep <= f_acc(23 downto 8);
                    when 362 => c_cx0 <= f_acc(15 downto 0);
                    when 386 => c_rx0 <= f_acc(15 downto 0);
                    when 400 =>                   -- motion (once per field, at this field's rates)
                        if s_vneg = '1' then s_phz <= s_phz - c_vz; else s_phz <= s_phz + c_vz; end if;
                        v_pb := s_phb;
                        if s_vneg = '1' then
                            if v_pb < c_vb then s_phb <= v_pb + to_unsigned(327680, 19) - c_vb; else s_phb <= v_pb - c_vb; end if;
                        else
                            v_pb := v_pb + c_vb;
                            if v_pb >= 327680 then s_phb <= v_pb - to_unsigned(327680, 19); else s_phb <= v_pb; end if;
                        end if;
                        s_fc <= s_fc + 1;
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
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := '0' & s_osum(7 downto 1); s_lb_wd <= std_logic_vector(v_l);   -- Osc at half depth
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17; slots load at 4, 21, 38, 55, 72): the blob tape index, the cusp depth, Lean, Billow, Scallop;
    -- ~90 clocks, so it fits SD's 858-clock line
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t  : signed(15 downto 0);
        variable v_u  : unsigned(15 downto 0);
        variable v_cl : signed(14 downto 0);
        variable v_w  : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy  <= (others => '0');
                e_rpl <= (others => '0');
                e_lc  <= (others => '0');
            elsif g_start = '1' then
                e_vy  <= e_vy + resize(s_ys, 20);
                e_rpl <= e_rpl + c_rstep;
                e_lc  <= (others => '0');
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_yv <= e_vy(18 downto 8);
                when 2 =>
                    e_yz  <= e_yv + s_phz(18 downto 8);
                    e_ybr <= resize(e_yv, 12) + resize(s_phb(18 downto 8), 12);
                when 3 =>
                    if e_ybr >= 1280 then e_yb <= resize(e_ybr - 1280, 11); else e_yb <= e_ybr(10 downto 0); end if;
                    -- cusp depth K(Yz): piecewise linear (ya, ka, slope Q12)
                    if e_yz >= 1750 then    e_ya <= to_unsigned(1750, 11); e_ka <= to_unsigned(40, 8);  e_sl <= to_unsigned(550, 11);  e_slneg <= '1';
                    elsif e_yz >= 1500 then e_ya <= to_unsigned(1500, 11); e_ka <= to_unsigned(145, 8); e_sl <= to_unsigned(1720, 11); e_slneg <= '1';
                    elsif e_yz >= 1044 then e_ya <= to_unsigned(1044, 11); e_ka <= to_unsigned(145, 8); e_sl <= to_unsigned(0, 11);    e_slneg <= '0';
                    elsif e_yz >= 900 then  e_ya <= to_unsigned(900, 11);  e_ka <= to_unsigned(84, 8);  e_sl <= to_unsigned(1735, 11); e_slneg <= '0';
                    elsif e_yz >= 650 then  e_ya <= to_unsigned(650, 11);  e_ka <= to_unsigned(84, 8);  e_sl <= to_unsigned(0, 11);    e_slneg <= '0';
                    elsif e_yz >= 350 then  e_ya <= to_unsigned(350, 11);  e_ka <= to_unsigned(0, 8);   e_sl <= to_unsigned(1147, 11); e_slneg <= '0';
                    else                    e_ya <= to_unsigned(0, 11);    e_ka <= to_unsigned(0, 8);   e_sl <= to_unsigned(0, 11);    e_slneg <= '0';
                    end if;
                when 4 =>
                    m_a <= resize(e_yb, 36); m_b <= to_unsigned(205, 16); m_acc <= (others => '0');   -- slot A: Yb * 205
                    e_kd <= e_yz - e_ya;
                    -- band gradient line: which band, distance to its waist
                    if e_yz >= 1300 then
                        e_yd <= signed(resize(e_yz, 13)) - 2108; e_w0q <= to_unsigned(109, 7);
                    elsif e_yz < 300 then
                        e_yd <= signed(resize(e_yz, 13)) - 60; e_w0q <= to_unsigned(109, 7);
                    else
                        e_yd <= signed(resize(e_yz, 13)) - 905; e_w0q <= to_unsigned(26, 7);
                    end if;
                when 5 =>
                    if e_yd < 0 then e_d <= unsigned(resize(-e_yd, 12)); else e_d <= unsigned(resize(e_yd, 12)); end if;
                    e_sx <= signed(resize(e_yz, 13)) - resize(c_tip, 13);
                when 6 =>
                    e_hd <= to_unsigned(327, 14) + shift_left(resize(e_d, 14), 2) - resize(shift_right(e_d, 4), 14)
                            - resize(shift_right(e_d, 6), 14);
                    if e_sx < 0 then e_s1 <= (others => '0');
                    else e_s1 <= resize(shift_right(unsigned(e_sx(11 downto 0)), 1), 12) + resize(shift_right(unsigned(e_sx(11 downto 0)), 3), 12)
                                 + resize(shift_right(unsigned(e_sx(11 downto 0)), 7), 12); end if;
                when 7 =>
                    if e_hd > 2047 then n_hd <= to_unsigned(2047, 11); else n_hd <= e_hd(10 downto 0); end if;
                    n_w0 <= e_w0q;
                    if e_s1 > 115 then n_s1 <= to_unsigned(115, 8); n_sd <= to_unsigned(64, 8);
                    elsif e_s1 > 51 then n_s1 <= e_s1(7 downto 0); n_sd <= resize(e_s1 - 51, 8);
                    else n_s1 <= e_s1(7 downto 0); n_sd <= (others => '0'); end if;
                    -- tracking lines, the comb, the comb's jitter hash
                    if (e_yz >= 770 and e_yz <= 772) or (e_yz >= 872 and e_yz <= 873) or (e_yz >= 922 and e_yz <= 924)
                       or (e_yz >= 20 and e_yz <= 22) or (e_yz >= 46 and e_yz <= 47) then n_trk <= '1'; else n_trk <= '0'; end if;
                    if e_yz >= 1044 and e_yz < 1160 then n_comb <= '1'; else n_comb <= '0'; end if;
                    if e_yz < 1080 then n_cw <= '1'; else n_cw <= '0'; end if;
                    n_crow <= e_yz(2);
                    e_j1 <= shift_left(resize(e_yz, 16), 5) + shift_left(resize(e_yz, 16), 2) + resize(e_yz, 16);
                    n_yz <= e_yz;
                when 8 =>
                    e_jit <= e_j1(5 downto 0) xor e_yz(7 downto 2);
                    -- slip hash: v = bi * 73 + seed * 29
                    e_sv <= resize(shift_left(resize(e_yz(10 downto 6), 8), 6) + shift_left(resize(e_yz(10 downto 6), 8), 3)
                            + resize(e_yz(10 downto 6), 8) + c_sd29, 8);
                when 9 =>
                    e_cph <= shift_left(resize(e_jit, 16), 8) + shift_left(resize(e_jit, 16), 7) + shift_left(resize(e_jit, 16), 4)
                             + shift_left(resize(e_jit, 16), 3) + shift_left(resize(e_jit, 16), 1);
                    e_sv2 <= e_sv xor shift_left(e_sv, 3) xor shift_right(e_sv, 2);
                when 10 =>
                    n_cph0 <= e_cph + c_cx0;
                    n_rp0  <= e_rpl + c_rx0;
                    if s_slip = '1' and e_sv2(1 downto 0) = "00" then
                        v_t := resize(signed(resize(e_sv2(7 downto 2), 7)) - 32, 16);
                        e_ph0 <= c_ph0 + unsigned(shift_left(resize(v_t, 24), 18));
                    else
                        e_ph0 <= c_ph0;
                    end if;
                when 11 =>
                    n_ph0 <= e_ph0;
                when 21 =>                    -- blob tape entry e = (Yb * 205) >> 9, fraction f (eighths)
                    r_bl_a <= m_acc(17 downto 9); e_e1 <= m_acc(17 downto 9) + 1; e_f <= m_acc(8 downto 6);
                    m_a <= resize(e_kd, 36); m_b <= resize(e_sl, 16); m_acc <= (others => '0');   -- slot B: K slope
                when 22 =>
                    r_bl_a <= e_e1;
                when 23 =>
                    e_q0 <= r_bl_q;
                when 24 =>
                    e_q1 <= r_bl_q;
                    e_c0 <= signed(e_q0(23 downto 12)); e_w0 <= signed(e_q0(11 downto 0));
                when 25 =>
                    e_dc <= resize(signed(e_q1(23 downto 12)), 13) - resize(e_c0, 13);
                    e_dw <= resize(signed(e_q1(11 downto 0)), 13) - resize(e_w0, 13);
                when 26 =>
                    if e_f(0) = '1' then e_ic <= resize(e_dc, 16); e_iw <= resize(e_dw, 16);
                    else e_ic <= (others => '0'); e_iw <= (others => '0'); end if;
                when 27 =>
                    if e_f(1) = '1' then e_ic <= e_ic + shift_left(resize(e_dc, 16), 1); e_iw <= e_iw + shift_left(resize(e_dw, 16), 1); end if;
                when 28 =>
                    if e_f(2) = '1' then e_ic <= e_ic + shift_left(resize(e_dc, 16), 2); e_iw <= e_iw + shift_left(resize(e_dw, 16), 2); end if;
                when 29 =>
                    e_cc <= resize(e_c0, 13) + resize(shift_right(e_ic, 3), 13);
                    e_ww <= resize(e_w0, 13) + resize(shift_right(e_iw, 3), 13);
                when 30 =>
                    e_dl <= to_signed(1023, 14) - resize(e_cc, 14);
                    if e_ww < 0 then e_wneg <= '1'; else e_wneg <= '0'; end if;
                when 38 =>                    -- K0 = ka -+ (kd * slope) >> 12; slot C: |1023 - c| * L
                    if e_slneg = '1' then e_k0 <= signed(resize(e_ka, 10)) - signed(resize(m_acc(19 downto 12), 10));
                    else e_k0 <= signed(resize(e_ka, 10)) + signed(resize(m_acc(19 downto 12), 10)); end if;
                    if e_dl < 0 then m_a <= resize(unsigned(-e_dl), 36); e_dlneg <= '1';
                    else m_a <= resize(unsigned(e_dl), 36); e_dlneg <= '0'; end if;
                    m_b <= resize(c_L, 16); m_acc <= (others => '0');
                when 55 =>                    -- c' = 1023 -+ (|1023 - c| L) >> 9; slot D: |w| * G
                    if e_dlneg = '1' then v_cl := to_signed(1023, 15) + signed(resize(m_acc(21 downto 9), 15));
                    else v_cl := to_signed(1023, 15) - signed(resize(m_acc(21 downto 9), 15)); end if;
                    e_cl <= v_cl;
                    if e_ww < 0 then m_a <= resize(unsigned(-e_ww), 36); else m_a <= resize(unsigned(e_ww), 36); end if;
                    m_b <= resize(c_G, 16); m_acc <= (others => '0');
                when 56 =>
                    if e_cl > 2047 then n_c <= to_signed(2047, 12);
                    elsif e_cl < -2048 then n_c <= to_signed(-2048, 12);
                    else n_c <= e_cl(11 downto 0); end if;
                when 57 =>
                    v_cl := to_signed(1023, 15) - resize(n_c, 15);
                    if v_cl > 2047 then n_c2 <= to_signed(2047, 12);
                    else n_c2 <= v_cl(11 downto 0); end if;
                when 72 =>                    -- w' = +-(|w| G >> 8) + O; slot E: |K0| * |S|
                    if e_wneg = '1' then v_w := -signed(resize(m_acc(20 downto 8), 15));
                    else v_w := signed(resize(m_acc(20 downto 8), 15)); end if;
                    e_wl <= v_w + resize(c_O, 15);
                    if e_k0 < 0 then m_a <= resize(unsigned(-e_k0), 36); else m_a <= resize(unsigned(e_k0), 36); end if;
                    if c_S < 0 then m_b <= resize(unsigned(-c_S), 16); else m_b <= resize(unsigned(c_S), 16); end if;
                    m_acc <= (others => '0');
                when 73 =>
                    if e_wl > 2047 then n_w <= to_signed(2047, 12);
                    elsif e_wl < -2048 then n_w <= to_signed(-2048, 12);
                    else n_w <= e_wl(11 downto 0); end if;
                when 89 =>                    -- K = sign * (|K0| |S|) >> 8
                    if (e_k0 < 0) xor (c_S < 0) then n_k <= -signed(resize(m_acc(18 downto 8), 11));
                    else n_k <= signed(resize(m_acc(18 downto 8), 11)); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam.  Stage 1 only counts pixels (the line-buffer read address); the
    -- tile, comb and ripple DDAs run at stage 5, where the camera's luma product
    -- arrives.  The per-line values are copied from the engine at the same time.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px <= (others => '0');
            elsif g_avid = '1' then
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
            b_gsr <= b_gsr(2 downto 0) & g_start;

            if b_gsr(3) = '1' then
                b_ph  <= n_ph0;
                b_cph <= n_cph0;
                b_rp  <= n_rp0;
                l_c <= n_c; l_c2 <= n_c2; l_w <= n_w;
                if n_k < 0 then l_kneg <= '1'; l_km <= unsigned(resize(-n_k, 10));
                else l_kneg <= '0'; l_km <= unsigned(resize(n_k, 10)); end if;
                l_yz <= n_yz; l_hd <= n_hd; l_w0 <= n_w0; l_s1 <= n_s1; l_sd <= n_sd;
                l_trk <= n_trk; l_comb <= n_comb; l_cw <= n_cw; l_crow <= n_crow;
            elsif s_gav_sr(3) = '1' then
                b_ph  <= b_ph + c_tpx;
                b_cph <= b_cph + c_cstep;
                b_rp  <= b_rp + c_rpx;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 5 = the DDAs of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_hq : unsigned(9 downto 0);
        variable v_a  : signed(13 downto 0);
        variable v_r  : unsigned(2 downto 0);
        variable v_lv : signed(13 downto 0);
        variable v_ps : unsigned(10 downto 0);
        variable v_i  : unsigned(6 downto 0);
        variable v_c  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: luma from the line buffer
            p3_lm <= unsigned(s_lb_q);
            -- 4: etch partials (luma nibbles x gain)
            p4_ea <= p3_lm(3 downto 0) * c_ge;
            p4_eb <= p3_lm(7 downto 4) * c_ge;
            -- 5: luma x etch
            p5_le <= resize(p4_ea, 18) + shift_left(resize(p4_eb, 18), 4);
            -- 6: fold the tile phase into hq; comb pattern, ripple level
            if b_ph(23) = '1' then v_hq := not b_ph(22 downto 13); else v_hq := b_ph(22 downto 13); end if;
            p6_hq <= v_hq;
            if b_rp(15) = '1' then v_r := not b_rp(14 downto 12); else v_r := b_rp(14 downto 12); end if;
            q_cr(6) <= (b_cph(15) xor l_crow) & v_r;
            p6_le <= p5_le;
            -- 7: chord distances, |hq - hd|, hq - 200; the t^2 ROM address
            v_a := signed(resize(p6_hq, 14)) - resize(l_c, 14);
            if v_a < 0 then p7_a1 <= unsigned(resize(-v_a, 13)); else p7_a1 <= unsigned(resize(v_a, 13)); end if;
            v_a := signed(resize(p6_hq, 14)) - resize(l_c2, 14);
            if v_a < 0 then p7_a2 <= unsigned(resize(-v_a, 13)); else p7_a2 <= unsigned(resize(v_a, 13)); end if;
            v_a := signed(resize(p6_hq, 14)) - signed(resize(l_hd, 14));
            if v_a < 0 then p7_ab <= unsigned(resize(-v_a, 13)); else p7_ab <= unsigned(resize(v_a, 13)); end if;
            p7_hs <= signed(resize(p6_hq, 11)) - 200;
            r_t2_a <= not p6_hq;                                              -- t = 1023 - hq
            p7_le <= p6_le;
            q_cr(7) <= q_cr(6);
            -- 8: d = w - |hq - c| (and the twin), the band gradient distance, spike phase partials
            p8_d1 <= resize(l_w, 14) - signed(resize(p7_a1, 14));
            p8_d2 <= resize(l_w, 14) - signed(resize(p7_a2, 14));
            p8_g  <= signed(resize(p7_ab, 14)) - signed(resize(l_w0, 14));
            p8_s0 <= p7_hs * signed('0' & c_kq(3 downto 0));
            p8_s1 <= p7_hs * signed('0' & c_kq(7 downto 4));
            p8_s2 <= p7_hs * signed('0' & c_kq(9 downto 8));
            p8_le <= p7_le;
            q_cr(8) <= q_cr(7);
            -- 9: twin max, gradient level (0.2109 g), spike phase sum, t^2 (fabric copy), luma terms
            if s_twin = '1' and p8_d2 > p8_d1 then p9_d <= p8_d2; else p9_d <= p8_d1; end if;
            p9_lv <= shift_right(p8_g, 2) - shift_right(p8_g, 5) - shift_right(p8_g, 7);
            if p8_g < 0 then p9_lv <= (others => '0'); end if;
            p9_sp <= resize(p8_s0, 22) + shift_left(resize(p8_s1, 22), 4) + shift_left(resize(p8_s2, 22), 8);
            p9_t2 <= unsigned(r_t2_q);
            p9_eb <= p8_le(17 downto 7);
            p9_ez <= p8_le(17 downto 9);
            q_cr(9) <= q_cr(8);
            -- 10: d + luma, level clamp, spike ROM address, K x t^2 partials, Yz + luma
            p10_dd <= resize(p9_d, 15) + signed(resize(p9_eb, 15));
            if p9_lv < 0 then p10_lv <= (others => '0');
            elsif p9_lv > 63 then p10_lv <= to_unsigned(63, 6);
            else p10_lv <= unsigned(p9_lv(5 downto 0)); end if;
            r_sc_a <= unsigned(p9_sp(15 downto 8));
            p10_k0 <= l_km * p9_t2(3 downto 0);
            p10_k1 <= l_km * p9_t2(7 downto 4);
            p10_k2 <= l_km * p9_t2(11 downto 8);
            p10_ze <= l_yz + resize(p9_ez, 11);
            q_cr(10) <= q_cr(9);
            -- 11: key, P = (|K| t^2) >> 12
            if p10_dd > 0 then p11_key <= not s_inv; else p11_key <= s_inv; end if;
            v_ps := resize(shift_right(resize(p10_k0, 22) + shift_left(resize(p10_k1, 22), 4) + shift_left(resize(p10_k2, 22), 8), 12), 11);
            p11_p <= v_ps(9 downto 0);
            p11_ze <= p10_ze;
            q_lv(11) <= p10_lv;
            q_cr(11) <= q_cr(10);
            -- 12: run counter (h units since the last key falling edge), Psi, spike shape (fabric copy)
            if s_gav_sr(9) = '0' then
                p12_rc <= (others => '1'); p12_prev <= '0';
            else
                p12_prev <= p11_key;
                if p12_prev = '1' and p11_key = '0' then
                    p12_rc <= (others => '0');
                elsif p12_rc > 511 - 4 then
                    p12_rc <= (others => '1');
                else
                    p12_rc <= p12_rc + s_xs;
                end if;
            end if;
            p12_key <= p11_key;
            if l_kneg = '1' then p12_psi <= p11_ze + resize(p11_p, 11); else p12_psi <= p11_ze - resize(p11_p, 11); end if;
            p12_sc <= unsigned(r_sc_q);
            q_lv(12) <= q_lv(11);
            q_cr(12) <= q_cr(11);
            -- 13: zone, spikes, shadow level, fringe
            if p12_psi < 50 then p13_zone <= "11";
            elsif p12_psi < 750 then p13_zone <= "00";
            elsif p12_psi < 1044 then p13_zone <= "01";
            elsif p12_psi < 1600 then p13_zone <= "10";
            else p13_zone <= "11"; end if;
            if p12_sc < l_s1 then p13_sw <= '1'; else p13_sw <= '0'; end if;
            if p12_sc < l_sd then p13_sd <= '1'; else p13_sd <= '0'; end if;
            if p12_rc < 190 then p13_sl <= "11";
            elsif p12_rc < 207 then p13_sl <= "10";
            elsif p12_rc < 223 then p13_sl <= "01";
            else p13_sl <= "00"; end if;
            if p12_rc < 24 then p13_mag <= '1'; else p13_mag <= '0'; end if;
            p13_key <= p12_key;
            q_lv(13) <= q_lv(12);
            q_cr(13) <= q_cr(12);
            -- 14: palette index
            v_c := q_cr(13);
            case p13_zone is
                when "00" =>                                                  -- cloud
                    if p13_key = '1' then v_i := to_unsigned(96, 7);
                    else v_i := to_unsigned(64, 7) + resize(p13_sl & v_c(2 downto 0), 7); end if;
                when "10" =>                                                  -- forest
                    if l_comb = '1' then
                        if v_c(3) = '1' then v_i := to_unsigned(101, 7);
                        elsif l_cw = '1' then v_i := to_unsigned(102, 7);
                        else v_i := to_unsigned(103, 7); end if;
                    elsif p13_key = '1' then
                        if p13_sw = '1' then v_i := to_unsigned(98, 7); else v_i := to_unsigned(97, 7); end if;
                    elsif p13_sd = '1' then v_i := to_unsigned(100, 7);
                    elsif p13_mag = '1' then v_i := to_unsigned(99, 7);
                    else v_i := to_unsigned(98, 7); end if;
                when others =>                                                -- bands
                    if l_trk = '1' and p13_key = '0' then v_i := to_unsigned(64, 7) + resize(v_c(2 downto 0), 7);
                    elsif p13_key = '1' then
                        if p13_sw = '1' then v_i := to_unsigned(98, 7); else v_i := to_unsigned(97, 7); end if;
                    elsif p13_sd = '1' then v_i := to_unsigned(100, 7);
                    elsif p13_mag = '1' then v_i := to_unsigned(99, 7);
                    else v_i := resize(q_lv(13), 7); end if;
            end case;
            r_pl_a <= s_pal & v_i;
            -- 15: palette ROM; 16: fabric copy
            p17_c <= r_pl_q;
            -- 17: compose; palette black outside the generated avid
            if s_gav_sr(14) = '0' then
                s_y17 <= to_unsigned(C_KY, 10); s_u17 <= to_unsigned(C_KU, 10); s_v17 <= to_unsigned(C_KU, 10);
            else
                s_y17 <= unsigned(p17_c(31 downto 22)); s_u17 <= unsigned(p17_c(21 downto 12)); s_v17 <= unsigned(p17_c(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the still's softness and the HD HDMI bar fix; the 8-back
    -- samples come from a block-RAM ring), luma 13/16 and the blanking gate.  The
    -- running sums snap back to black once 8 blank samples are in.
    ---------------------------------------------------------------------------
    p_ob : process(clk)
    begin
        if rising_edge(clk) then
            ob_q <= ob(to_integer(ob_ra));
            ob(to_integer(ob_wa)) <= std_logic_vector(s_y17) & std_logic_vector(s_u17) & std_logic_vector(s_v17);
        end if;
    end process p_ob;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            ob_wa <= ob_wa + 1;
            ob_ra <= ob_wa - 5;                          -- the stage-17 sample 8 pixels back, in ob_r at stage 18
            ob_r  <= ob_q;
            -- 18: 8-sums
            if s_bah = "00000000" then
                s_bay <= to_unsigned(C_KY * 8, 13); s_bau <= to_unsigned(C_KU * 8, 13); s_bav <= to_unsigned(C_KU * 8, 13);
            else
                s_bay <= s_bay + s_y17 - unsigned(ob_r(29 downto 20));
                s_bau <= s_bau + s_u17 - unsigned(ob_r(19 downto 10));
                s_bav <= s_bav + s_v17 - unsigned(ob_r(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- 19: box output (black off the generated avid)
            if s_gav_sr(16) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y19 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u19 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v19 <= v_sc(13 downto 4);
            else
                s_y19 <= to_unsigned(C_KY, 10); s_u19 <= to_unsigned(C_KU, 10); s_v19 <= to_unsigned(C_KU, 10);
            end if;
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21 .. 24: carry (latency padding to C_LAT 24)
            s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            s_y22 <= s_y21; s_u22 <= s_u21; s_v22 <= s_v21;
            s_y23 <= s_y22; s_u23 <= s_u22; s_v23 <= s_v22;
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;
            -- 25: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cumulus;
