-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Wildfire v0.1.1 -- Heirloom (phase 3, from the clip WIN_20230509_23_58_21 at 0.5 s).
--
-- v0.1.1: the big black blob top left (low-frequency V knots left of x 330), the lower third in hot pink
-- and red (a stronger lake R/G split with the lace riding it, a hot-pink band along the bottom), navy
-- patches from a per-knot key table (key values above 60 darken), a bolder S8 Deep saw.
--
-- The clip: black flame pits on a yellow ground -- a column of eggs at x ~1000 with thin olive "eyes"
-- between them, a fan of long fins rising to the right that ends in a zig-zag edge, tall tilted bars on the
-- left -- green fringes on the right fan, orange around the eggs, a fine diagonal lace in the lower third
-- with a blue key under it (black -> blue, red -> hot pink), a per-line strip down the left edge.  Everything
-- flows down at ~4.4 lines per field while the lattice morphs.
--
-- The patch, as fitted (wildfire.py has every constant; .heirloom_work/rb_wildfire/notes.md the readings):
--   phi_v  = G (c2(x) Y^2 + c1(x) Y + c0(x)) - flow    V oscillator, per-column frequency / phase from a
--                                                       64-px knot table (the H ramp patched into its FM),
--                                                       Y = the V ramp, soft-clipped below line 450
--   phi_h  = 1.5 + Ge (h(x) - 1.5 + s(x) y) - drift     H oscillator: a slewed staircase (eggs, eyes, fins)
--   phi_w  = fine diagonal oscillator (22 px), VCA'd in from line ~520 (the lace)
--   F      = cos phi_h + cos phi_v + aw cos phi_w - lake
--   R, G   = ROM_R[F - thr - 2d], ROM_G[F - thr + 0.6 d]   high-gain colouriser with an R/G threshold split d
--   B      = blue key under a wavy line, edged by the lace
--
--   K1 Eggs     H oscillator density x0.25 .. x4: continuous fins (0) .. the photo .. a grid of eggs
--   K2 Fringe   R/G threshold split 0 .. x3: plain black/yellow .. the photo .. wide green and red bands
--   K3 Lace     the lower "lake": fine lace amplitude and how high the lace, blue key and dimming reach
--   K4 Speed    bipolar flow (+-8 deadband); 700 = the clip's 4.4 lines per field downward
--   K5 Black    colouriser threshold: thin black .. the photo .. black dominating
--   K6 Etch     how far the camera (or the Osc) bends the V oscillator's phase
--   S7 Fold     the colouriser input folded (pits and peaks both go black: ringed eggs)
--   S8 Zigzag   the photo's small saw teeth / deep saw teeth on every edge
--   S9 Palette  Wildfire (yellow ground) / Ember (red-orange ground, from the same patch's other clip)
--   S10 Invert  the oscillator sum inverted: peaks black, pits gold
--   S11 Source  Video / Osc
--   P12 FLAMES  V oscillator density x0.35 .. x3 (500 = the photo): a few huge slow flames .. fine fire lace
--
-- Engine: vblank-zone sequencer (glide, per-field scalars, a 256-word coefficient loop) and a per-line
-- engine (computes the NEXT line: the V ramp, the envelopes, then 31 knots x {V, H, D/K} into double-
-- banked knot RAMs), one pipelined 16 x 13 multiplier shared by both.  The pixel path interpolates the
-- knot tables with exact DDAs.  Generated avid; output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before the blanking gate.  Colours in the hardware
-- convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.wildfire_pkg.all;
use work.all;

architecture wildfire of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

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
    -- triangle of a Q12 phase: -4096 at 0, +4096 at half a cycle
    function f_tri(p : unsigned(15 downto 0)) return signed is
        variable v_f : signed(13 downto 0);
    begin
        v_f := signed(resize(p(11 downto 0), 14)) - 2048;
        return 4096 - shift_left(abs(v_f), 2);
    end function;

    attribute no_rw_check : boolean;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs
    signal r_kn_a : unsigned(7 downto 0) := (others => '0');
    signal r_kn_q : signed(15 downto 0) := (others => '0');
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_cv_a, r_ch_a, r_cw_a : unsigned(8 downto 0) := (others => '0');
    signal r_cv_q, r_ch_q, r_cw_q : signed(7 downto 0) := (others => '0');
    signal r_rc_a, r_gc_a : unsigned(8 downto 0) := (others => '0');
    signal r_rc_q, r_gc_q : std_logic_vector(23 downto 0) := (others => '0');
    signal r_bc_a : unsigned(4 downto 0) := (others => '0');
    signal r_bc_q : std_logic_vector(31 downto 0) := (others => '0');

    -- coefficient RAM (per field: 32 knots x 8 words), written in the zone, read by the line engine
    type t_w16 is array (0 to 255) of signed(15 downto 0);
    signal cf : t_w16 := (others => (others => '0'));
    attribute no_rw_check of cf : signal is true;
    signal cf_we : std_logic := '0';
    signal cf_wa, cf_ra : unsigned(7 downto 0) := (others => '0');
    signal cf_wd : signed(15 downto 0) := (others => '0');
    signal cf_q, cf_r : signed(15 downto 0) := (others => '0');

    -- knot RAMs (bank * 32 + knot): V and H phases (Q12 cycles mod 16), D|K (8 + 8 bits)
    type t_k64 is array (0 to 63) of std_logic_vector(15 downto 0);
    signal kv_ram, kh_ram, kd_ram : t_k64 := (others => (others => '0'));
    attribute no_rw_check of kv_ram : signal is true;
    attribute no_rw_check of kh_ram : signal is true;
    attribute no_rw_check of kd_ram : signal is true;
    signal kv_we, kh_we, kd_we : std_logic := '0';
    signal kv_wa, kh_wa, kd_wa : unsigned(5 downto 0) := (others => '0');
    signal kv_wd, kh_wd, kd_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal k_ra : unsigned(5 downto 0) := (others => '0');
    signal kv_q, kh_q, kd_q : std_logic_vector(15 downto 0) := (others => '0');

    -- D|K delay ring (stage 2 -> stage 11)
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal rg : t_ring := (others => (others => '0'));
    attribute no_rw_check of rg : signal is true;
    signal rg_wa : unsigned(7 downto 0) := (others => '0');
    signal rg_ra : unsigned(7 downto 0) := (others => '0');
    signal rg_wd, rg_q, rg_r : std_logic_vector(15 downto 0) := (others => '0');

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
    signal s_sd   : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(341 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(160 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(500 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_spd : signed(10 downto 0) := (others => '0');
    signal s_kd  : signed(10 downto 0) := (others => '0');
    signal s_km  : unsigned(9 downto 0) := (others => '0');
    signal s_kn  : std_logic := '0';
    signal s_fold, s_zig, s_emb, s_inv, s_src : std_logic := '0';

    -- per-field scalars
    signal c_epg  : unsigned(9 downto 0) := (others => '0');
    signal c_shg  : unsigned(1 downto 0) := (others => '0');
    signal c_e1   : signed(10 downto 0) := (others => '0');
    signal c_mant : unsigned(15 downto 0) := (others => '0');
    signal c_gq   : unsigned(12 downto 0) := to_unsigned(256, 13);     -- Flames gain, Q8
    signal c_geq  : unsigned(12 downto 0) := to_unsigned(256, 13);     -- Eggs gain, Q8
    signal c_kfq  : unsigned(12 downto 0) := to_unsigned(255, 13);     -- Fringe gain, Q8
    signal c_g273 : unsigned(12 downto 0) := (others => '0');
    signal c_awq  : unsigned(6 downto 0) := to_unsigned(51, 7);        -- lace amplitude, Q6
    signal c_aw273 : signed(15 downto 0) := (others => '0');
    signal c_lake : signed(10 downto 0) := (others => '0');            -- lake shift (virtual lines, + = down)
    signal c_thr  : signed(9 downto 0) := to_signed(-114, 10);         -- colouriser threshold (1/127 F)
    signal c_etq  : unsigned(5 downto 0) := to_unsigned(10, 6);        -- etch gain
    signal c_sv, c_sh, c_sw, c_sp : signed(15 downto 0) := (others => '0');   -- flow steps per field
    signal c_dv, c_dh, c_dw, c_dp : unsigned(15 downto 0) := (others => '0'); -- flow accumulators (Q12)
    signal c_wx0  : unsigned(19 downto 0) := (others => '0');          -- W phase at x0 (SD)
    signal c_wkx  : unsigned(19 downto 0) := (others => '0');          -- W phase step per pixel
    signal c_x0   : unsigned(11 downto 0) := (others => '0');          -- first pixel (h)
    signal c_u    : unsigned(12 downto 0) := (others => '0');          -- breathing LFO 0 .. 4096
    signal c_u4   : unsigned(12 downto 0) := (others => '0');
    signal c_lu   : unsigned(8 downto 0) := (others => '0');

    -- coefficient loop
    signal cl_run : std_logic := '0';
    signal cl_i   : unsigned(7 downto 0) := (others => '0');
    signal cl_v   : std_logic_vector(5 downto 0) := (others => '0');   -- valid pipe
    signal cl_a2, cl_a5 : unsigned(7 downto 0) := (others => '0');      -- word index at stages 2 and 5
    signal r_kn_r : signed(15 downto 0) := (others => '0');
    signal r_kn_l : signed(15 downto 0) := (others => '0');          -- ROM word + lake (key line)

    -- shared multiplier (16 signed x 13 unsigned, result 3 clocks after the operands are set)
    -- operand select codes (registered one clock ahead of the issue)
    constant OP_IDLE : integer := 0;
    constant OP_Z26 : integer := 1;  constant OP_Z28 : integer := 2;  constant OP_Z30 : integer := 3;
    constant OP_Z33 : integer := 4;  constant OP_Z34 : integer := 5;  constant OP_Z36 : integer := 6;
    constant OP_Z37 : integer := 7;  constant OP_Z40 : integer := 8;  constant OP_Z41 : integer := 9;
    constant OP_Z42 : integer := 10; constant OP_Z43 : integer := 11; constant OP_Z44 : integer := 12;
    constant OP_Z45 : integer := 13; constant OP_CF  : integer := 14; constant OP_KN  : integer := 15;
    constant OP_L3  : integer := 16; constant OP_L4  : integer := 17; constant OP_L5  : integer := 18;
    constant OP_L6  : integer := 19; constant OP_L7  : integer := 20; constant OP_L8  : integer := 21;
    constant OP_L10 : integer := 22; constant OP_L11 : integer := 23; constant OP_L12 : integer := 24;
    constant OP_L13 : integer := 25; constant OP_L14 : integer := 26; constant OP_L17 : integer := 27;
    constant OP_L9  : integer := 28; constant OP_Z38 : integer := 29;
    signal m_oh  : std_logic_vector(31 downto 0) := (others => '0');   -- one-hot operand select
    signal m_a  : signed(15 downto 0) := (others => '0');
    signal m_a3 : signed(17 downto 0) := (others => '0');            -- 3 m_a, for the 2-bit partials
    signal m_b  : unsigned(12 downto 0) := (others => '0');
    signal m_p0, m_p1 : signed(20 downto 0) := (others => '0');
    signal m_p2 : signed(21 downto 0) := (others => '0');
    signal m_p  : signed(29 downto 0) := (others => '0');

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
    signal e_vy   : unsigned(19 downto 0) := (others => '0');        -- Q8 virtual line of the line computed
    signal e_yv   : unsigned(10 downto 0) := (others => '0');
    signal e_seg  : unsigned(1 downto 0) := (others => '0');
    signal e_t    : unsigned(8 downto 0) := (others => '0');
    signal e_t450 : unsigned(7 downto 0) := (others => '0');
    signal e_in   : signed(15 downto 0) := (others => '0');
    signal e_tw, e_to : unsigned(12 downto 0) := (others => '0');
    signal e_q    : signed(9 downto 0) := (others => '0');           -- key ramp 0.4 y (Q4)
    signal e_win  : unsigned(8 downto 0) := (others => '0');         -- navy-patch line window 0 .. 256
    signal e_wm   : signed(12 downto 0) := (others => '0');
    signal e_band : std_logic := '0';                                -- the hot-pink band along the bottom
    signal e_tr1  : signed(13 downto 0) := (others => '0');
    signal e_y4   : unsigned(11 downto 0) := (others => '0');
    signal e_y2   : unsigned(12 downto 0) := (others => '0');
    signal e_m    : unsigned(8 downto 0) := (others => '0');
    signal e_dyl  : signed(7 downto 0) := (others => '0');
    signal e_offd : signed(9 downto 0) := (others => '0');
    signal e_ps   : unsigned(15 downto 0) := (others => '0');
    signal e_bank : std_logic := '0';                                 -- bank being written (next line)
    signal p_bank : std_logic := '0';                                 -- bank the pixel path reads
    -- knot loop
    signal kl_run : std_logic := '0';
    signal kl_k   : unsigned(4 downto 0) := (others => '0');
    signal kl_s   : unsigned(2 downto 0) := (others => '0');
    signal kl_v   : std_logic_vector(5 downto 0) := (others => '0');   -- loop valid pipe
    signal kl_s2, kl_s5 : unsigned(2 downto 0) := (others => '0');      -- word at stages 2 and 5
    signal kl_k5  : unsigned(4 downto 0) := (others => '0');
    signal kl_e, kl_hb : signed(15 downto 0) := (others => '0');
    signal kl_va  : signed(15 downto 0) := (others => '0');
    signal kl_dr  : signed(10 downto 0) := (others => '0');
    signal kl_dc  : signed(7 downto 0) := (others => '0');
    signal kl_kk  : signed(7 downto 0) := (others => '0');
    -- per-line values for the next line, and their copies for the current line
    signal n_aw, l_aw : unsigned(6 downto 0) := (others => '0');
    signal n_ot, l_ot : signed(9 downto 0) := (others => '0');       -- off + thr
    signal n_zs, l_zs : unsigned(10 downto 0) := (others => '0');
    signal n_wl, l_wl : unsigned(19 downto 0) := (others => '0');
    signal n_sv, l_sv : unsigned(11 downto 0) := (others => '0');    -- strip V (mod 1 cycle)
    signal n_sd, l_sd : signed(7 downto 0) := (others => '0');       -- strip D
    signal n_sk, l_sk : signed(7 downto 0) := (others => '0');       -- strip K
    -- line-start prefetch / beam / DDAs
    signal f_cnt  : unsigned(3 downto 0) := (others => '0');
    signal b_gsr  : std_logic_vector(4 downto 0) := (others => '0');
    signal l_px   : unsigned(10 downto 0) := (others => '0');          -- luma read counter (2 pixels ahead)
    signal b_x    : unsigned(11 downto 0) := (others => '0');
    signal b_x1   : unsigned(11 downto 0) := (others => '0');          -- the next pixel's position
    signal b_cr   : std_logic := '0';                                 -- the step to b_x1 crosses a knot
    signal b_k2   : unsigned(4 downto 0) := (others => '0');          -- knot to read at the next crossing
    signal b_rdc  : unsigned(1 downto 0) := (others => '0');          -- read-back countdown
    signal v_acc, h_acc : unsigned(18 downto 0) := (others => '0');
    signal v_nxt, v_nx2, h_nxt, h_nx2 : unsigned(15 downto 0) := (others => '0');
    signal v_dxs, v_d2xs, h_dxs, h_d2xs : unsigned(18 downto 0) := (others => '0');
    signal d_acc  : signed(15 downto 0) := (others => '0');
    signal d_nxt, d_nx2 : signed(7 downto 0) := (others => '0');
    signal d_dxs, d_d2xs : signed(15 downto 0) := (others => '0');
    signal k_acc  : signed(15 downto 0) := (others => '0');           -- blue key / navy patches (Q4), x128
    signal k_nxt, k_nx2 : signed(7 downto 0) := (others => '0');
    signal k_dxs, k_d2xs : signed(15 downto 0) := (others => '0');
    signal w_ph   : unsigned(19 downto 0) := (others => '0');

    -- pixel path
    signal p2_v, p2_h : unsigned(11 downto 0) := (others => '0');
    signal p2_w : unsigned(8 downto 0) := (others => '0');
    signal p3_ea, p3_eb : unsigned(9 downto 0) := (others => '0');
    signal p4_e   : unsigned(13 downto 0) := (others => '0');
    signal p7_cv, p7_ch, p7_cw : signed(7 downto 0) := (others => '0');
    signal p8_pa  : signed(12 downto 0) := (others => '0');
    signal p8_pb  : signed(11 downto 0) := (others => '0');
    signal p8_s2, p9_s2 : signed(9 downto 0) := (others => '0');
    signal p9_awc : signed(9 downto 0) := (others => '0');
    signal p10_s, p11_s : signed(10 downto 0) := (others => '0');
    signal p12_kb, p13_kb : signed(9 downto 0) := (others => '0');
    signal p10_awc : signed(9 downto 0) := (others => '0');
    signal p11_kb : signed(9 downto 0) := (others => '0');
    signal p11_d  : signed(10 downto 0) := (others => '0');           -- D + the lace
    signal p11_otk : signed(10 downto 0) := (others => '0');          -- off + thr + navy darkness
    signal p12_d2, p12_dg : signed(11 downto 0) := (others => '0');
    signal p12_u  : signed(11 downto 0) := (others => '0');
    signal p13_ur, p13_ug : signed(12 downto 0) := (others => '0');
    signal p16_rc, p16_gc : std_logic_vector(23 downto 0) := (others => '0');
    signal p16_bc : std_logic_vector(31 downto 0) := (others => '0');
    signal p17_y, p17_u, p17_v : signed(12 downto 0) := (others => '0');
    signal s_y18 : unsigned(9 downto 0) := to_unsigned(C_WF_KY, 10);
    signal s_u18 : unsigned(9 downto 0) := to_unsigned(C_WF_KU, 10);
    signal s_v18 : unsigned(9 downto 0) := to_unsigned(C_WF_KV, 10);

    -- output conditioning (one box; its 8-back samples from a block-RAM ring)
    type t_ob is array (0 to 255) of std_logic_vector(29 downto 0);
    signal ob : t_ob := (others => (others => '0'));
    attribute no_rw_check of ob : signal is true;
    signal ob_wa, ob_ra : unsigned(7 downto 0) := (others => '0');
    signal ob_q, ob_r : std_logic_vector(29 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y20, s_u20, s_v20, s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(C_WF_KY, 10);
    constant C_KU : unsigned(9 downto 0) := to_unsigned(C_WF_KU, 10);
    constant C_KV : unsigned(9 downto 0) := to_unsigned(C_WF_KV, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel
    -- syncs delayed C_LAT in a block-RAM ring (stored inverted, so the zeroed RAM reads as idle syncs)
    type t_sy is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sy : t_sy := (others => (others => '0'));
    attribute no_rw_check of sy : signal is true;
    signal sy_wa, sy_ra : unsigned(7 downto 0) := (others => '0');
    signal sy_q, sy_r : std_logic_vector(3 downto 0) := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories (one write statement each, registered reads)
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
            r_kn_q <= C_WF_KNOT(to_integer(r_kn_a));
            r_ex_q <= C_WF_EXP2(to_integer(r_ex_a));
            r_cv_q <= C_WF_COS(to_integer(r_cv_a));
            r_ch_q <= C_WF_COS(to_integer(r_ch_a));
            r_cw_q <= C_WF_COS(to_integer(r_cw_a));
            r_rc_q <= C_WF_RCOL(to_integer(r_rc_a));
            r_gc_q <= C_WF_GCOL(to_integer(r_gc_a));
            r_bc_q <= C_WF_BCOL(to_integer(r_bc_a));
        end if;
    end process p_rom;

    p_cf : process(clk)
    begin
        if rising_edge(clk) then
            cf_q <= cf(to_integer(cf_ra));
            if cf_we = '1' then cf(to_integer(cf_wa)) <= cf_wd; end if;
        end if;
    end process p_cf;

    p_kv : process(clk)
    begin
        if rising_edge(clk) then
            kv_q <= kv_ram(to_integer(k_ra));
            if kv_we = '1' then kv_ram(to_integer(kv_wa)) <= kv_wd; end if;
        end if;
    end process p_kv;

    p_kh : process(clk)
    begin
        if rising_edge(clk) then
            kh_q <= kh_ram(to_integer(k_ra));
            if kh_we = '1' then kh_ram(to_integer(kh_wa)) <= kh_wd; end if;
        end if;
    end process p_kh;

    p_kd : process(clk)
    begin
        if rising_edge(clk) then
            kd_q <= kd_ram(to_integer(k_ra));
            if kd_we = '1' then kd_ram(to_integer(kd_wa)) <= kd_wd; end if;
        end if;
    end process p_kd;

    p_rg : process(clk)
    begin
        if rising_edge(clk) then
            rg_q <= rg(to_integer(rg_ra));
            rg(to_integer(rg_wa)) <= rg_wd;
        end if;
    end process p_rg;

    ---------------------------------------------------------------------------
    -- Shared multiplier: 16 signed x 13 unsigned, three 4/4/5-bit partials,
    -- then one sum.  Operands set at clock n are readable as m_p at clock n + 3.
    ---------------------------------------------------------------------------
    -- operand AND-OR: every select is its own register (no decode on the multiplier's input)
    p_opm : process(clk)
        variable v_a : signed(15 downto 0);
        variable v_b : unsigned(12 downto 0);
        procedure put(k : integer; a : signed(15 downto 0); b : unsigned(12 downto 0)) is
        begin
            if m_oh(k) = '1' then v_a := v_a or a; v_b := v_b or b; end if;
        end procedure;
        variable v_g, v_n : unsigned(12 downto 0);
        variable v_3 : signed(15 downto 0);
        variable v_r : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            v_a := (others => '0'); v_b := (others => '0');
            case to_integer(cl_a2(2 downto 0)) is
                when 0 | 1 | 2 => v_g := c_gq;
                when 3 | 4     => v_g := c_geq;
                when 5         => v_g := c_kfq;
                when 6         => v_g := to_unsigned(6554, 13);      -- key: (BK + lake) * 0.4
                when others    => v_g := to_unsigned(256, 13);       -- navy depth: x 1
            end case;
            if cl_a2(2 downto 0) = 6 then v_r := r_kn_l; else v_r := r_kn_r; end if;
            case to_integer(kl_s2) is
                when 0      => v_n := e_y2;
                when 1      => v_n := resize(e_y4, 13);
                when 3      => v_n := resize(e_yv, 13);
                when 5      => v_n := resize(e_m, 13);
                when others => v_n := resize(e_win, 13);
            end case;
            case e_seg is
                when "01"   => v_3 := to_signed(655, 16);
                when "10"   => v_3 := to_signed(253, 16);
                when others => v_3 := to_signed(21, 16);
            end case;
            put(OP_Z26, to_signed(794, 16), resize(s_gp(12 downto 3), 13));
            put(OP_Z28, to_signed(2458, 16), c_u);
            put(OP_Z30, to_signed(500, 16), c_u);
            put(OP_Z33, signed('0' & c_mant(15 downto 1)), to_unsigned(90, 13));
            put(OP_Z34, to_signed(100, 16), c_u);
            put(OP_Z36, signed(resize(s_g3(12 downto 3), 16)), to_unsigned(102, 13));
            put(OP_Z37, signed(resize(c_gq, 16)), c_u4);
            put(OP_Z38, to_signed(91, 16), resize(s_km, 13));
            put(OP_Z40, signed(resize(c_awq, 16)), to_unsigned(273, 13));
            put(OP_Z41, signed(resize(c_gq, 16)), to_unsigned(273, 13));
            put(OP_Z42, resize(s_spd, 16), to_unsigned(7, 13));
            put(OP_Z43, resize(s_spd, 16), to_unsigned(541, 13));
            put(OP_Z44, resize(s_spd, 16), to_unsigned(29, 13));
            put(OP_Z45, resize(s_spd, 16), c_g273);
            put(OP_CF,  v_r, v_g);
            put(OP_KN,  cf_r, v_n);
            put(OP_L3,  v_3, resize(e_t, 13));
            put(OP_L4,  to_signed(1311, 16), resize(e_t450, 13));
            put(OP_L5,  to_signed(410, 16), resize(e_t450, 13));
            put(OP_L6,  c_aw273, e_tw);
            put(OP_L7,  e_in, resize(e_t, 13));
            put(OP_L8,  to_signed(365, 16), e_to);
            put(OP_L10, resize(e_dyl, 16), c_kfq);
            put(OP_L11, signed(shift_left(resize(e_y4, 16), 3)), resize(e_y4, 13));
            put(OP_L12, to_signed(1691, 16), resize(e_yv, 13));
            put(OP_L13, to_signed(114, 16), resize(e_yv, 13));
            put(OP_L14, to_signed(12021, 16), resize(e_yv, 13));
            put(OP_L17, resize(e_tr1, 16), to_unsigned(51, 13));
            put(OP_L9,  to_signed(26214, 16), resize(e_yv, 13));
            m_a <= v_a; m_b <= v_b;
            m_a3 <= resize(v_a, 18) + shift_left(resize(v_a, 18), 1);
        end if;
    end process p_opm;

    p_mul : process(clk)
        -- a x 2-bit digit from {0, a, 2a, 3a}: one LUT level, no adder
        function f_d2(a : signed(15 downto 0); a3 : signed(17 downto 0); d : unsigned(1 downto 0)) return signed is
        begin
            case d is
                when "00"   => return to_signed(0, 18);
                when "01"   => return resize(a, 18);
                when "10"   => return shift_left(resize(a, 18), 1);
                when others => return a3;
            end case;
        end function;
        variable v_q4, v_q5, v_q6 : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            -- the same three partials as before (a b[3:0], a b[7:4], a b[12:8]), each one add of 2-bit digits
            m_p0 <= resize(f_d2(m_a, m_a3, m_b(1 downto 0)), 21) + shift_left(resize(f_d2(m_a, m_a3, m_b(3 downto 2)), 21), 2);
            m_p1 <= resize(f_d2(m_a, m_a3, m_b(5 downto 4)), 21) + shift_left(resize(f_d2(m_a, m_a3, m_b(7 downto 6)), 21), 2);
            v_q4 := f_d2(m_a, m_a3, m_b(9 downto 8));
            v_q5 := f_d2(m_a, m_a3, m_b(11 downto 10));
            if m_b(12) = '1' then v_q6 := resize(m_a, 18); else v_q6 := (others => '0'); end if;
            m_p2 <= resize(v_q4, 22) + shift_left(resize(v_q5, 22), 2) + shift_left(resize(v_q6, 22), 4);
            m_p  <= resize(m_p0, 30) + shift_left(resize(m_p1, 30), 4) + shift_left(resize(m_p2, 30), 8);
        end if;
    end process p_mul;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer, line engine (one process: it
    -- alone drives the multiplier operands)
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_s   : signed(10 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_t   : signed(12 downto 0);
        variable v_y   : signed(12 downto 0);
        variable v_w   : unsigned(2 downto 0);
        variable v_r   : signed(29 downto 0);
        variable v_16  : signed(15 downto 0);
        variable v_kv  : signed(10 downto 0);
        variable v_tg  : std_logic_vector(8 downto 0);
        variable v_tr  : signed(13 downto 0);
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
            elsif s_zc /= 4095 then
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

            -- speed: bipolar, +-8 deadband, (|d| - 8) * 91 / 64 (Q8; 700 = 1.0); |d| - 8 pipelined here,
            -- the x 91 on the multiplier in the zone (s_spd at zone clock 41)
            s_k4 <= unsigned(registers_in(3));
            s_kd <= signed(resize(s_k4, 11)) - 512;
            if s_kd > -9 and s_kd < 9 then
                s_km <= (others => '0');
            elsif s_kd(10) = '0' then
                s_km <= unsigned(s_kd(9 downto 0)) - 8;
            else
                s_km <= unsigned(not s_kd(9 downto 0)) - 7;
            end if;
            s_kn <= s_kd(10);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_fold <= registers_in(6)(0);
                s_zig  <= registers_in(6)(1);
                s_emb  <= registers_in(6)(2);
                s_inv  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                if s_xsr = 4 then s_sd <= '1'; else s_sd <= '0'; end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
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

            ------------------------------------------------------------------
            -- per-field scalars (zone clocks 26 .. 60)
            ------------------------------------------------------------------
            -- multiplier operand selects, registered one clock ahead of the issue (p_opm)
            m_oh <= (others => '0');
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 25 => m_oh(OP_Z26) <= '1';
                    when 27 => m_oh(OP_Z28) <= '1';
                    when 29 => m_oh(OP_Z30) <= '1';
                    when 32 => m_oh(OP_Z33) <= '1';
                    when 33 => m_oh(OP_Z34) <= '1';
                    when 35 => m_oh(OP_Z36) <= '1';
                    when 36 => m_oh(OP_Z37) <= '1';
                    when 39 => m_oh(OP_Z40) <= '1';
                    when 40 => m_oh(OP_Z41) <= '1';
                    when 41 => m_oh(OP_Z42) <= '1';
                    when 42 => m_oh(OP_Z43) <= '1';
                    when 43 => m_oh(OP_Z44) <= '1';
                    when 44 => m_oh(OP_Z45) <= '1';
                    when 37 => m_oh(OP_Z38) <= '1';
                    when others => null;
                end case;
            end if;
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 26 =>
                        c_dv <= c_dv + unsigned(c_sv); c_dh <= c_dh + unsigned(c_sh);
                        c_dw <= c_dw + unsigned(c_sw); c_dp <= c_dp + unsigned(c_sp);
                        c_e1 <= signed(resize(s_g1(12 downto 3), 11)) - 512;
                    when 27 =>                                  -- breathing LFO u = 0 (the photo) .. 4096
                        v_tr := f_tri(0 - c_dh);
                        c_u <= unsigned(resize(shift_right(resize(v_tr, 15) + 4096, 1), 13));
                    when 28 =>
                    when 29 =>
                        c_epg <= unsigned(m_p(19 downto 10));
                    when 30 =>
                        r_ex_a <= c_epg(7 downto 0); c_shg <= c_epg(9 downto 8);
                    when 31 =>
                        c_u4 <= unsigned(m_p(24 downto 12));
                    when 32 =>
                        c_mant <= r_ex_q;
                        r_ex_a <= unsigned(c_e1(7 downto 0));
                    when 33 =>
                        c_lu <= unsigned(m_p(20 downto 12));
                    when 34 =>                                  -- breathing threshold: + u * 100 / 4096
                    when 35 =>                                  -- Geq = mant >> (7 - sh1), sh1 = -2 .. 1
                        case to_integer(c_e1(10 downto 8)) is
                            when 6      => c_geq <= resize(shift_right(r_ex_q, 9), 13);   -- -2
                            when 7      => c_geq <= resize(shift_right(r_ex_q, 8), 13);   -- -1
                            when 0      => c_geq <= resize(shift_right(r_ex_q, 7), 13);
                            when others => c_geq <= resize(shift_right(r_ex_q, 6), 13);   -- 1
                        end case;
                    when 36 =>                                  -- Gq = (mant/2 * 90) >> (14 - shg)
                        case to_integer(c_shg) is
                            when 0      => c_gq <= resize(unsigned(m_p(26 downto 14)), 13);
                            when 1      => c_gq <= resize(unsigned(m_p(25 downto 13)), 13);
                            when 2      => c_gq <= resize(unsigned(m_p(24 downto 12)), 13);
                            when others => c_gq <= resize(unsigned(m_p(23 downto 11)), 13);
                        end case;
                    when 37 =>                                  -- Gq * u4 (breathing)
                        c_kfq <= resize(shift_right(resize(s_g2(12 downto 3), 13) * 3, 2), 13);
                        v_s := 512 - signed(resize(s_g3(12 downto 3), 11));
                        c_lake <= resize(shift_right(v_s * 3, 2), 11) + signed(resize(c_lu, 11));
                        c_thr <= -114 + resize(shift_right(signed(resize(s_g5(12 downto 3), 11)) - 512, 2), 10)
                                 + signed(resize(unsigned(m_p(18 downto 12)), 10));
                        c_etq <= s_g6(12 downto 7);
                    when 39 =>
                        c_awq <= unsigned(m_p(16 downto 10));
                    when 40 =>
                        c_gq <= c_gq - unsigned(m_p(24 downto 12));
                    when 43 =>
                        c_aw273 <= m_p(15 downto 0);
                    when 44 =>
                        c_g273 <= resize(unsigned(m_p(20 downto 8)), 13);
                    when 41 =>
                        if s_kn = '1' then s_spd <= -resize(m_p(16 downto 6), 11); else s_spd <= resize(m_p(16 downto 6), 11); end if;
                    when 45 =>
                        c_sh <= -m_p(23 downto 8);
                    when 46 =>
                        c_sw <= -m_p(23 downto 8);
                    when 47 =>
                        c_sp <= -m_p(23 downto 8);
                    when 48 =>
                        c_sv <= -m_p(23 downto 8);
                    when others => null;
                end case;
            end if;

            ------------------------------------------------------------------
            -- coefficient loop (zone clocks 200 .. 463): word i = knot (i >> 3), w = i & 7
            --   ROM address at t, data at t + 1, fabric copy t + 2, multiply issued t + 3,
            --   product t + 6, written t + 7
            ------------------------------------------------------------------
            if s_zone = '1' and s_zc = 200 then
                cl_run <= '1'; cl_i <= (others => '0');
            elsif cl_run = '1' then
                cl_i <= cl_i + 1;
                if cl_i = 255 then cl_run <= '0'; end if;
            end if;
            r_kn_a <= cl_i;
            cl_v <= cl_v(4 downto 0) & cl_run;
            if cl_v(2) = '1' then cl_a2 <= cl_a2 + 1; else cl_a2 <= (others => '0'); end if;
            if cl_v(5) = '1' then cl_a5 <= cl_a5 + 1; else cl_a5 <= (others => '0'); end if;
            cf_we <= '0';

            ------------------------------------------------------------------
            -- line engine: kicked at g_start (and in the zone for line 0) and computes
            -- the NEXT line's values
            ------------------------------------------------------------------
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;
            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + s_ys;
                e_lc <= (others => '0');
                e_bank <= not e_bank;
                p_bank <= e_bank;
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            kv_we <= '0'; kh_we <= '0'; kd_we <= '0';

            r_kn_r <= r_kn_q;
            r_kn_l <= r_kn_q + resize(c_lake, 16);
            if cl_v(1) = '1' then m_oh(OP_CF) <= '1'; end if;          -- coefficient loop: issue at stage 2
            if true then
                case to_integer(e_lc) is
                    when 1 =>
                        e_yv <= e_vy(18 downto 8);
                    when 2 =>
                        if e_yv >= 700 then e_seg <= "11"; e_t <= resize(e_yv - 700, 9);
                        elsif e_yv >= 560 then e_seg <= "10"; e_t <= resize(e_yv - 560, 9);
                        elsif e_yv >= 450 then e_seg <= "01"; e_t <= resize(e_yv - 450, 9);
                        else e_seg <= "00"; e_t <= (others => '0'); end if;
                        if e_yv >= 650 then e_t450 <= to_unsigned(200, 8);
                        elsif e_yv >= 450 then e_t450 <= resize(e_yv - 450, 8);
                        else e_t450 <= (others => '0'); end if;
                        v_t := signed(resize(e_yv, 13)) - 520 - resize(c_lake, 13);
                        if v_t < 0 then e_tw <= (others => '0'); else e_tw <= unsigned(v_t); end if;
                        v_t := signed(resize(e_yv, 13)) - 560 - resize(c_lake, 13);
                        if v_t < 0 then e_to <= (others => '0'); else e_to <= unsigned(v_t); end if;
                        v_t := signed(resize(e_yv, 13)) - 955 - resize(c_lake, 13);
                        e_band <= not v_t(12);
                        m_oh(OP_L3) <= '1';
                    when 3 =>                                 -- (issue A18 * t); navy window: min(y - 520, 690 - y)
                        v_t := signed(resize(e_yv, 13)) - 520 - resize(c_lake, 13);
                        v_y := 690 + resize(c_lake, 13) - signed(resize(e_yv, 13));
                        if v_y < v_t then v_t := v_y; end if;
                        e_wm <= v_t;
                        m_oh(OP_L4) <= '1';
                    when 4 =>                                 -- (issue 1311 t450); window x 3, clipped 0 .. 256
                        v_y := e_wm + shift_left(e_wm, 1);
                        if v_y < 0 then e_win <= (others => '0');
                        elsif v_y > 256 then e_win <= to_unsigned(256, 9);
                        else e_win <= unsigned(v_y(8 downto 0)); end if;
                        m_oh(OP_L5) <= '1';
                    when 5 =>                                 -- (issue 410 t450)
                        m_oh(OP_L6) <= '1';
                    when 6 =>                                 -- inner = S14 - (A18 t >> 4)
                        case e_seg is
                            when "01"   => e_in <= to_signed(16384, 16) - m_p(19 downto 4);
                            when "10"   => e_in <= to_signed(7373, 16) - m_p(19 downto 4);
                            when others => e_in <= to_signed(2949, 16) - m_p(19 downto 4);
                        end case;
                        m_oh(OP_L7) <= '1';
                    when 7 =>
                        if e_t450 = 200 then e_m <= (others => '0');
                        else e_m <= to_unsigned(256, 9) - resize(unsigned(m_p(17 downto 10)), 9); end if;
                        m_oh(OP_L8) <= '1';
                    when 8 =>
                        if e_band = '1' then e_dyl <= to_signed(-127, 8);              -- the bottom band
                        elsif e_t450 = 200 then e_dyl <= to_signed(-80, 8);
                        else e_dyl <= -signed(resize(unsigned(m_p(16 downto 10)), 8)); end if;
                        m_oh(OP_L9) <= '1';
                    when 9 =>                                 -- aw = min(AW273 tw >> 16, Awq)
                        if m_p(29 downto 16) > signed(resize(c_awq, 14)) then n_aw <= c_awq;
                        else n_aw <= unsigned(m_p(22 downto 16)); end if;
                        m_oh(OP_L10) <= '1';
                    when 10 =>                                -- Y4
                        if e_seg = "00" then e_y4 <= resize(e_yv & "00", 12);
                        else
                            case e_seg is
                                when "01"   => e_y4 <= to_unsigned(1800, 12) + unsigned(m_p(23 downto 12));
                                when "10"   => e_y4 <= to_unsigned(2119, 12) + unsigned(m_p(23 downto 12));
                                when others => e_y4 <= to_unsigned(2295, 12) + unsigned(m_p(23 downto 12));
                            end case;
                        end if;
                        m_oh(OP_L11) <= '1';
                    when 11 =>                                -- off + thr
                        if e_band = '1' then n_ot <= 200 + c_thr;                       -- the bottom band
                        elsif m_p(29 downto 10) > 89 then n_ot <= 89 + c_thr;
                        else n_ot <= signed(resize(unsigned(m_p(16 downto 10)), 10)) + c_thr; end if;
                        m_oh(OP_L12) <= '1';
                    when 12 =>                                -- key ramp q = 0.4 y
                        e_q <= m_p(25 downto 16);
                        m_oh(OP_L13) <= '1';
                    when 13 =>
                        e_offd <= resize(shift_right(m_p, 8), 10);
                        m_oh(OP_L14) <= '1';
                    when 14 =>                                -- Y2 = Y4^2 >> 10 = (8 Y4 Y4) >> 13
                        e_y2 <= unsigned(m_p(25 downto 13));

                    when 15 =>                                -- strip phase
                        e_ps <= unsigned(m_p(23 downto 8)) + 397 + c_dp;
                    when 16 =>                                -- saw: (114 yv mod 4096) / 16 (Fine) or 5/8 (Deep)
                        if s_zig = '1' then
                            n_zs <= resize(shift_right(unsigned(m_p(11 downto 0)), 1), 11) + resize(shift_right(unsigned(m_p(11 downto 0)), 3), 11);
                        else
                            n_zs <= resize(shift_right(unsigned(m_p(11 downto 0)), 4), 11);
                        end if;
                        n_sv <= e_ps(11 downto 0);
                        e_tr1 <= f_tri(e_ps + 1415);
                        m_oh(OP_L17) <= '1';
                    when 17 =>                                -- strip key from the same triangle
                        v_kv := 48 - resize(shift_right(e_tr1, 5), 11);
                        if v_kv > 60 then n_sk <= to_signed(60, 8);                    -- (no navy darkness in the strip)
                        elsif v_kv < -127 then n_sk <= to_signed(-127, 8);
                        else n_sk <= resize(v_kv, 8); end if;
                        n_wl <= shift_left(resize(e_yv, 20), 15) + unsigned(m_p(19 downto 0)) + shift_left(resize(c_dw, 20), 8) + c_wx0;
                    when 20 =>
                        n_sd <= resize(shift_right(m_p, 12), 8);
                    when others => null;
                end case;

                -- knot loop: kl_t(n) = tag of the coefficient word read n clocks ago (cf_ra at 0,
                -- cf_q at 1, cf_r at 2); multiply issued at stage 3, product at stage 6
                if kl_v(1) = '1' then m_oh(OP_KN) <= '1'; end if;
            end if;
            if cl_v(5) = '1' then                     -- coefficient loop: store
                v_w := cl_a5(2 downto 0);
                case to_integer(v_w) is
                    when 2      => cf_wd <= m_p(23 downto 8) + signed(c_dv);
                    when 4      => cf_wd <= m_p(23 downto 8) + 6144 + signed(c_dh);
                    when 6      => cf_wd <= -m_p(29 downto 14);           -- key line -0.4 (BK + lake)
                    when others => cf_wd <= m_p(23 downto 8);
                end case;
                cf_wa <= cl_a5; cf_we <= '1';
            end if;

            -- knot loop sequencing (line clocks 24 .. 272): 31 knots x 8 words
            if e_lc = 24 then
                kl_run <= '1'; kl_k <= (others => '0'); kl_s <= (others => '0');
            elsif kl_run = '1' then
                if kl_s = 7 then
                    if kl_k = 30 then kl_run <= '0'; end if;
                    kl_k <= kl_k + 1;
                end if;
                kl_s <= kl_s + 1;
            end if;
            cf_ra <= kl_k & kl_s;
            kl_v <= kl_v(4 downto 0) & kl_run;
            if kl_v(2) = '1' then kl_s2 <= kl_s2 + 1; else kl_s2 <= (others => '0'); end if;
            if kl_v(5) = '1' then
                if kl_s5 = 7 then kl_k5 <= kl_k5 + 1; end if;
                kl_s5 <= kl_s5 + 1;
            else
                kl_s5 <= (others => '0'); kl_k5 <= (others => '0');
            end if;
            cf_r <= cf_q;
            if kl_v(2) = '1' then
                case to_integer(kl_s2) is
                    when 2      => kl_e <= cf_r;
                    when 4      => kl_hb <= cf_r;
                    when 6      =>                    -- key: min(kb + q, 60) (the B key saturates at 60)
                        v_kv := resize(cf_r, 11) + resize(e_q, 11);
                        if v_kv > 60 then kl_kk <= to_signed(60, 8);
                        elsif v_kv < -127 then kl_kk <= to_signed(-127, 8);
                        else kl_kk <= resize(v_kv, 8); end if;
                    when others => null;
                end case;
            end if;
            if kl_v(5) = '1' then                     -- products issued 3 clocks ago (stage 5)
                case to_integer(kl_s5) is
                    when 0 =>
                        kl_va <= m_p(27 downto 12);
                    when 1 =>
                        kv_wd <= std_logic_vector(kl_va + m_p(23 downto 8) + kl_e);
                        kv_wa <= e_bank & kl_k5; kv_we <= '1';
                    when 3 =>
                        kh_wd <= std_logic_vector(m_p(23 downto 8) + kl_hb);
                        kh_wa <= e_bank & kl_k5; kh_we <= '1';
                    when 5 =>                         -- D before the clip (|dxk m >> 8| <= 66)
                        kl_dr <= resize(m_p(17 downto 8), 11) + resize(e_offd, 11);
                    when 6 =>                         -- clip D (one clock later)
                        if kl_dr > 127 then kl_dc <= to_signed(127, 8);
                        elsif kl_dr < -127 then kl_dc <= to_signed(-127, 8);
                        else kl_dc <= resize(kl_dr, 8); end if;
                    when 7 =>                         -- K = key + navy depth x window, write {K, D}
                        v_kv := resize(kl_kk, 11) + resize(m_p(16 downto 8), 11);
                        if v_kv > 127 then v_16 := to_signed(127, 16); else v_16 := resize(v_kv, 16); end if;
                        kd_wd <= std_logic_vector(v_16(7 downto 0)) & std_logic_vector(kl_dc);
                        kd_wa <= e_bank & kl_k5; kd_we <= '1';
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
        end if;
    end process p_seq;

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
    -- Source side: oscillator (2-D triangle lattice, drifting, half amplitude),
    -- line-buffer write of (luma >> 2) - 16
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
            s_osum <= shift_right(v_s, 1) - shift_right(v_s, 4);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: three knot-interpolating DDAs (V, H, D: value = T_k + n * dT * xs >> 7
    -- from the segment's first pixel), the blue-key and lace ramps.  It runs three
    -- clocks behind the generated avid (stage 1 = the DDA values).  At the line start
    -- and at every knot crossing: acc = T_k << 7, slope = (T_k+1 - T_k) xs, and knot
    -- k + 2 is read back.  Knots k0 and k0 + 1 of the next line are prefetched in
    -- hblank (or at zone clock 900 for line 0).
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_nx  : unsigned(11 downto 0);
        variable v_dv, v_dh : unsigned(18 downto 0);
        variable v_dd, v_dk : signed(15 downto 0);
        variable v_go  : std_logic;
    begin
        if rising_edge(clk) then
            b_gsr <= b_gsr(3 downto 0) & g_start;
            if (s_gav_sr(5) = '1' and s_gav_sr(4) = '0') or (s_zone = '1' and s_zc = 900) then
                f_cnt <= to_unsigned(1, 4);
            elsif f_cnt /= 0 then
                f_cnt <= f_cnt + 1;
            end if;
            if b_rdc /= 0 then b_rdc <= b_rdc - 1; end if;

            v_go := '0';
            if b_gsr(4) = '1' then
                b_x  <= c_x0;
                b_x1 <= c_x0 + s_xs;
                b_cr <= '0';                             -- x0 + xs never crosses a knot (x0 = 0 or 512 h)
                b_k2 <= c_x0(11 downto 7) + 3;           -- read address after the first crossing
                k_ra <= p_bank & (c_x0(11 downto 7) + 2);
                l_aw <= n_aw; l_ot <= n_ot; l_zs <= n_zs;
                l_sv <= n_sv; l_sd <= n_sd; l_sk <= n_sk;
                w_ph <= n_wl;
                v_go := '1';
            elsif s_gav_sr(4) = '1' then
                v_nx := b_x1 + s_xs;
                b_x  <= b_x1;
                b_x1 <= v_nx;
                if v_nx(11 downto 7) /= b_x1(11 downto 7) then b_cr <= '1'; else b_cr <= '0'; end if;
                w_ph <= w_ph + c_wkx;
                if b_cr = '1' then
                    b_k2 <= b_k2 + 1;
                    k_ra <= p_bank & b_k2;
                    v_go := '1';
                else
                    v_acc <= v_acc + v_dxs; h_acc <= h_acc + h_dxs; d_acc <= d_acc + d_dxs; k_acc <= k_acc + k_dxs;
                end if;
            end if;
            if v_go = '1' then
                v_acc <= shift_left(resize(v_nxt, 19), 7);
                h_acc <= shift_left(resize(h_nxt, 19), 7);
                d_acc <= shift_left(resize(d_nxt, 16), 7);
                k_acc <= shift_left(resize(k_nxt, 16), 7);
                v_dxs <= v_d2xs; h_dxs <= h_d2xs; d_dxs <= d_d2xs; k_dxs <= k_d2xs;
                v_nxt <= v_nx2; h_nxt <= h_nx2; d_nxt <= d_nx2; k_nxt <= k_nx2;
                b_rdc <= "11";
            end if;
            -- read-back of knot k + 2 (three clocks after the crossing)
            if b_rdc = "01" then
                v_nx2 <= unsigned(kv_q); h_nx2 <= unsigned(kh_q); d_nx2 <= signed(kd_q(7 downto 0));
                k_nx2 <= signed(kd_q(15 downto 8));
            end if;
            -- hblank prefetch of knots k0, k0 + 1 (after any read-back has landed)
            case to_integer(f_cnt) is
                when 5 => k_ra <= e_bank & c_x0(11 downto 7);
                when 6 => k_ra <= e_bank & (c_x0(11 downto 7) + 1);
                when 7 => v_nxt <= unsigned(kv_q); h_nxt <= unsigned(kh_q); d_nxt <= signed(kd_q(7 downto 0));
                          k_nxt <= signed(kd_q(15 downto 8));
                when 8 => v_nx2 <= unsigned(kv_q); h_nx2 <= unsigned(kh_q); d_nx2 <= signed(kd_q(7 downto 0));
                          k_nx2 <= signed(kd_q(15 downto 8));
                when others => null;
            end case;
            -- next-segment slopes (registered; settle long before they are used)
            v_dv := resize(v_nx2 - v_nxt, 19); v_dh := resize(h_nx2 - h_nxt, 19);
            if v_dv(15) = '1' then v_dv(18 downto 16) := (others => '1'); end if;
            if v_dh(15) = '1' then v_dh(18 downto 16) := (others => '1'); end if;
            v_dd := resize(d_nx2, 16) - resize(d_nxt, 16);
            v_dk := resize(k_nx2, 16) - resize(k_nxt, 16);
            case to_integer(s_xs) is
                when 4      => v_d2xs <= shift_left(v_dv, 2); h_d2xs <= shift_left(v_dh, 2); d_d2xs <= shift_left(v_dd, 2);
                               k_d2xs <= shift_left(v_dk, 2);
                when 3      => v_d2xs <= shift_left(v_dv, 1) + v_dv; h_d2xs <= shift_left(v_dh, 1) + v_dh;
                               d_d2xs <= shift_left(v_dd, 1) + v_dd; k_d2xs <= shift_left(v_dk, 1) + v_dk;
                when others => v_d2xs <= shift_left(v_dv, 1); h_d2xs <= shift_left(v_dh, 1); d_d2xs <= shift_left(v_dd, 1);
                               k_d2xs <= shift_left(v_dk, 1);
            end case;
            if b_gsr(2) = '1' then l_px <= (others => '0'); elsif s_gav_sr(2) = '1' then l_px <= l_px + 1; end if;
            s_lb_ra <= l_px + 1;                         -- luma of the pixel at stage 0 (etch is ready at stage 2)
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage s here = stage s + 5 in the generated-avid count; the register names keep the
    -- labels of an earlier, deeper layout: p7 = stage 5 ... s_y18 = stage 16)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u  : unsigned(11 downto 0);
        variable v_s  : signed(10 downto 0);
        variable v_a  : signed(14 downto 0);
        variable v_i  : signed(12 downto 0);
        variable v_y, v_c1, v_c2 : signed(12 downto 0);
        variable v_k  : signed(19 downto 0);
        variable v_kk : signed(7 downto 0);
        variable v_d  : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: DDA values (or the strip's), the saw on H, the lace index; D|K into the ring
            if b_x < 224 then
                p2_v <= l_sv; p2_h <= to_unsigned(1024, 12);
                rg_wd <= std_logic_vector(l_sd) & std_logic_vector(l_sk);
            else
                p2_v <= v_acc(18 downto 7); p2_h <= h_acc(18 downto 7) + l_zs(10 downto 0);
                rg_wd <= std_logic_vector(d_acc(14 downto 7)) & std_logic_vector(k_acc(14 downto 7));
            end if;
            p2_w <= w_ph(19 downto 11);
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 3;                          -- written from stage 2, rg_r read at stage 9 (label p11)
            rg_r  <= rg_q;
            -- 1: etch partials (the luma of the stage-1 pixel is in s_lb_q); 2: etch
            p3_ea <= resize(unsigned(s_lb_q(3 downto 0)) * c_etq, 10);
            p3_eb <= resize(unsigned(s_lb_q(7 downto 4)) * c_etq, 10);
            p4_e <= resize(p3_ea, 14) + shift_left(resize(p3_eb, 14), 4);
            -- 3: cosine addresses
            v_u := p2_v + p4_e(11 downto 0);
            r_cv_a <= v_u(11 downto 3);
            r_ch_a <= p2_h(11 downto 3);
            r_cw_a <= p2_w;
            -- 6: ROM data; 7: fabric copy
            p7_cv <= r_cv_q; p7_ch <= r_ch_q; p7_cw <= r_cw_q;
            -- 8: lace partials, cos sum
            p8_pa <= resize(p7_cw * signed('0' & l_aw(3 downto 0)), 13);
            p8_pb <= resize(p7_cw * signed('0' & l_aw(6 downto 4)), 12);
            p8_s2 <= resize(p7_cv, 10) + resize(p7_ch, 10);
            -- 9: awc = (aw cw) >> 6
            v_a := resize(p8_pa, 15) + shift_left(resize(p8_pb, 15), 4);   -- |aw cw| <= 12954
            p9_awc <= resize(shift_right(v_a, 6), 10);
            p9_s2 <= p8_s2;
            -- 10: S; the lace term rides along
            p10_s <= resize(p9_s2, 11) + resize(p9_awc, 11);
            p10_awc <= p9_awc;
            -- 11: invert / fold
            if s_inv = '1' then
                p11_s <= -p10_s - 40;
            elsif s_fold = '1' then
                p11_s <= 160 - shift_left(abs(p10_s), 1);
            else
                p11_s <= p10_s;
            end if;
            -- D|K back from the ring: the key with the lace edge, D + the lace (it rides the R/G split),
            -- and the navy darkness 4 max(K - 60, 0) folded into off + thr
            v_kk := signed(rg_r(7 downto 0));
            v_d  := signed(rg_r(15 downto 8));
            p11_kb <= resize(v_kk, 10) + resize(shift_right(p10_awc, 2), 10);
            p11_d  <= resize(v_d, 11) + resize(p10_awc, 11);
            if v_kk > 60 then
                p11_otk <= resize(l_ot, 11) + shift_left(resize(v_kk - 60, 11), 2);
            else
                p11_otk <= resize(l_ot, 11);
            end if;
            -- 12: u0 = S - off - thr - navy; 2D for R, 5/8 D - Ember for G
            p12_u <= resize(p11_s, 12) - resize(p11_otk, 12);
            p12_kb <= p11_kb;
            p12_d2 <= shift_left(resize(p11_d, 12), 1);
            if s_emb = '1' then
                p12_dg <= resize(shift_right(p11_d, 1), 12) + resize(shift_right(p11_d, 3), 12) - 150;
            else
                p12_dg <= resize(shift_right(p11_d, 1), 12) + resize(shift_right(p11_d, 3), 12);
            end if;
            -- 13: per-channel inputs
            p13_ur <= resize(p12_u, 13) - resize(p12_d2, 13);
            p13_ug <= resize(p12_u, 13) + resize(p12_dg, 13);
            p13_kb <= p12_kb;
            -- 14: ROM addresses (clipped)
            v_i := shift_right(p13_ur, 1) + 256;
            if v_i < 0 then r_rc_a <= (others => '0'); elsif v_i > 511 then r_rc_a <= (others => '1');
            else r_rc_a <= unsigned(v_i(8 downto 0)); end if;
            v_i := shift_right(p13_ug, 1) + 256;
            if v_i < 0 then r_gc_a <= (others => '0'); elsif v_i > 511 then r_gc_a <= (others => '1');
            else r_gc_a <= unsigned(v_i(8 downto 0)); end if;
            v_s := resize(shift_right(p13_kb, 2), 11) + 16;
            if v_s < 0 then r_bc_a <= (others => '0'); elsif v_s > 31 then r_bc_a <= (others => '1');
            else r_bc_a <= unsigned(v_s(4 downto 0)); end if;
            -- 15: ROM data; 16: fabric copy
            p16_rc <= r_rc_q; p16_gc <= r_gc_q; p16_bc <= r_bc_q;
            -- 17: sums (Y pre-scaled 16/13; Cr, Cb signed contributions)
            v_y  := shift_left(signed(resize(unsigned(p16_rc(23 downto 16)), 13)) + signed(resize(unsigned(p16_gc(23 downto 16)), 13)), 2)
                    + signed(resize(unsigned(p16_bc(31 downto 22)), 13));
            v_c1 := shift_left(resize(signed(p16_rc(15 downto 8)), 13) + resize(signed(p16_gc(15 downto 8)), 13), 2)
                    + resize(signed(p16_bc(21 downto 11)), 13);
            v_c2 := shift_left(resize(signed(p16_rc(7 downto 0)), 13) + resize(signed(p16_gc(7 downto 0)), 13), 2)
                    + resize(signed(p16_bc(10 downto 0)), 13);
            p17_y <= v_y + 64; p17_u <= v_c1 + 512; p17_v <= v_c2 + 512;
            -- 18: clip and compose; black outside the generated avid
            if s_gav_sr(18) = '0' then
                s_y18 <= C_KY; s_u18 <= C_KU; s_v18 <= C_KV;
            else
                if p17_y > 940 then s_y18 <= to_unsigned(940, 10); else s_y18 <= unsigned(p17_y(9 downto 0)); end if;
                if p17_u < 0 then s_u18 <= (others => '0'); elsif p17_u > 1023 then s_u18 <= (others => '1');
                else s_u18 <= unsigned(p17_u(9 downto 0)); end if;
                if p17_v < 0 then s_v18 <= (others => '0'); elsif p17_v > 1023 then s_v18 <= (others => '1');
                else s_v18 <= unsigned(p17_v(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(18);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the photo's softness and the HD HDMI bar fix; the 8-back
    -- samples come from a block-RAM ring), luma 13/16 and the blanking gate.  The
    -- running sums snap back to the palette black once 8 blank samples are in.
    ---------------------------------------------------------------------------
    p_sy : process(clk)
    begin
        if rising_edge(clk) then
            sy_q <= sy(to_integer(sy_ra));
            sy(to_integer(sy_wa)) <= '0' & (not data_in.field_n) & (not data_in.vsync_n) & (not data_in.hsync_n);
            sy_wa <= sy_wa + 1;
            sy_ra <= sy_wa - (C_LAT - 3);                -- written at t, read back at t + C_LAT
            sy_r  <= sy_q;
        end if;
    end process p_sy;

    p_ob : process(clk)
    begin
        if rising_edge(clk) then
            ob_q <= ob(to_integer(ob_ra));
            ob(to_integer(ob_wa)) <= std_logic_vector(s_y18) & std_logic_vector(s_u18) & std_logic_vector(s_v18);
        end if;
    end process p_ob;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            ob_wa <= ob_wa + 1;
            ob_ra <= ob_wa - 5;                          -- the stage-18 sample 8 pixels back, in ob_r at stage 19
            ob_r  <= ob_q;
            -- 19: 8-sums
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y18 - unsigned(ob_r(29 downto 20));
                s_bau <= s_bau + s_u18 - unsigned(ob_r(19 downto 10));
                s_bav <= s_bav + s_v18 - unsigned(ob_r(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- 20: box output (palette black off the generated avid)
            if s_gav_sr(20) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y20 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u20 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v20 <= v_sc(13 downto 4);
            else
                s_y20 <= C_KY; s_u20 <= C_KU; s_v20 <= C_KV;
            end if;
            -- 21: luma 13/16
            if s_y20 > 64 then v_dl := s_y20 - 64; else v_dl := (others => '0'); end if;
            s_y21 <= s_y20 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u21 <= s_u20; s_v21 <= s_v20;
            -- 22: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y22 <= s_y21; s_u22 <= s_u21; s_v22 <= s_v21;
            else
                s_y22 <= to_unsigned(64, 10); s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    -- per mode: W phase step per pixel (-8150 per h) and at the first pixel, the key's step per pixel
    -- and offset, the first pixel (SD: x0 = 512 h = knot 4)
    c_wkx <= to_unsigned(1015976, 20) when s_xs = 4 else to_unsigned(1024126, 20) when s_xs = 3 else to_unsigned(1032276, 20);
    c_wx0 <= to_unsigned(21504, 20) when s_xs = 4 else (others => '0');
    c_x0  <= to_unsigned(512, 12) when s_xs = 4 else (others => '0');

    data_out.y       <= std_logic_vector(s_y22);
    data_out.u       <= std_logic_vector(s_u22);
    data_out.v       <= std_logic_vector(s_v22);
    data_out.hsync_n <= not sy_r(0);
    data_out.vsync_n <= not sy_r(1);
    data_out.field_n <= not sy_r(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture wildfire;
