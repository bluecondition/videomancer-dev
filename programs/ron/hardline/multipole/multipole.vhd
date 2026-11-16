-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- multipole.vhd  (Hardline, v0.2.0) -- field lines of a magnetic multipole / a conformal checkerboard
--
-- Art reference: field-line drawings of magnetic multipoles and conformal-map checkerboards (the
-- stripes of Re z^n and Im z^n).  One pole, black and white, played by hand.
--
--   PHI_re = (2/|n|) (r/R0)^n cos(n theta + turn)      PHI_im = the sine partner
--
-- computed in the LOG domain on the Hypnos polar chassis (no CORDIC, no per-pixel divide):
--
--   log2 r, theta        hi/lo = max/min(|dx|,|dy|) -> two integer logs -> u = log2(hi/lo) -> EBR tables
--   a = n theta + turn   n = -4 .. -1, 1 .. 4: one adder of gated shifts, never a multiplier
--   log2|cos a|, log2|sin a|   octant fold: the LARGE one is -log2 cos(o) from a table, the SMALL one
--                        is log2(o) + log2(sin(o)/o): an integer log (the third port of the log table)
--                        plus a smooth table, so the nodal lines stay exact
--   L = log2(2/|n|) + n (log2 r - log2 R0) + bend + log2|cos or sin|
--   Even space:  phase = +-2^L   (exp2 mantissa table, one barrel shift; the sign of the cosine negates)
--   Log space:   phase = q L     (bands of log2|PHI|; q = 1 or 1.5 times a power of two: shift-adds)
--   coverage             Hypnos's one-pixel box filter of the band square wave, with the ANALYTIC
--                        gradient  log2 w = L0 + log2|n| - log2 r  (Even),  log2(q / ln 2) - log2 r -
--                        log2|cos| (Log); lines finer than about 2 px fade to flat grey
--   Checker              Re band XOR Im band (Fine: OR, a net of field lines and equipotentials)
--
--   K1  X        the pole, +-0.75 screen widths about the centre (detent at the centre)
--   K2  Y        the pole, +-1 screen height about the centre (detent)
--   K3  Period   Even: the reference radius R0, 1448 -> 45 px (exponential, clockwise = denser);
--                Log: lines per octave of r, 0.35 -> 11 in half-octave steps
--   K4  Order    n = -4, -3, -2, -1, 1, 2, 3, 4 (stepped, hysteresis)
--   K5  Turn     rotates the figure: exactly one lobe period over the knob (the ends meet)
--   K6  Camera   how far the box-filtered luma bends the linework: it is added to L (a local zoom
--                about the pole, and exactly a phase shift in Log space), 1.28 octaves at white at
--                100 %; 0 = the clean figure
--   S7  Glide    On: X, Y, Period, Turn, Camera and Speed ease 1/16 per field, never slower than half
--                a knob count (about 1 px of pole travel), landing exactly / Off: no easing at all.
--                Glide eases the SPEED, never the travelling phase: nothing stops the motion but speed 0
--   S8  Weave    Lines (Re only) / Checker
--   S9  Space    Even / Log
--   S10 Weight   Bold (equal bands) / Fine (1/6 lines on the half-integer levels)
--   S11 Source   Video / Osc (a slow triangle lattice in the camera's place, at 1.5 x its level)
--   P12 SPEED    bipolar (Hypnos v1.1.0): centre (+-24 counts) = still, up = the lines stream out of
--                the pole, down = into it; exponential, 2^-12 .. 2^-2 cycle per field.  A per-field
--                accumulator (one cycle = 2^20, wraps without a seam) moves the Re phase.  Bold: added to
--                the signed PHI, so the lines flow through the pole (out of the cos >= 0 lobes, into the
--                others) with every edge anti-aliased; Fine: added to |PHI|, every lobe streams outward;
--                Log: both families shift log2|PHI| together, a zoom flow out of the pole.  n > 0 is
--                reversed so up is outward
--
-- Chassis: timing / generated avid / log-polar engine / box-filter AA from hypnos.vhd v1.1.0
-- (hardware-validated); the source field (lattice, 32-px box, vertical IIR) from kinetic riley.
-- Luma only: U = V = 512 on every pixel.  One latency (48) in every mode.  25 EBR.
--
-- v0.2.0 (2026-11-16): P12 Flow (a static offset + duty swell) became Speed (Ron: the set may move by
-- itself, as Hypnos does); the band duty is now constant.
--
-- Stage numbering: stage N = N registers after s_dx.  o_y is stage 47.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.multipole_pkg.all;

architecture multipole of program_top is
    constant C_PROCESSING_DELAY_CLKS : integer := 48;      -- = processing_delay_clks in the TOML

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_DB    : integer := 14;                 -- X / Y detent deadband
    constant C_GL    : integer := 4;                  -- glide one-pole shift (1/16 per field)
    constant C_GMIN  : integer := 32;                 -- ... never slower than half a knob count (about 1 px) per field
    constant C_LSIN0 : integer := -251470;            -- log2 of one folded-angle unit (octant / 2^15), Q14

    -- TOML defaults (the holds and glides start there)
    constant C_DEF_K1  : integer := 448;
    constant C_DEF_K2  : integer := 558;
    constant C_DEF_K3  : integer := 435;
    constant C_DEF_K5  : integer := 470;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;

    constant ST_OUT : integer := 47;                  -- output register stage
    constant C_XH   : integer := 5;                   -- log2 hi: stage 8 -> used building stage 15
    constant C_XR   : integer := 4;                   -- log2 r:  stage 15 -> EBR output at stage 20
    constant C_XS   : integer := 45;                  -- syncs: EBR output = stage 45

    attribute no_rw_check : boolean;

    ----------------------------------------------------------------------
    -- field-latched controls + per-field terms
    ----------------------------------------------------------------------
    signal s_init   : std_logic := '1';               -- first field: latch and snap unconditionally
    -- the six knobs K1 K2 K3 K5 K6 P12 as two rings (hold, Q6 glide): one datapath serves all six
    type t_h6 is array(0 to 5) of unsigned(9 downto 0);
    type t_g6 is array(0 to 5) of unsigned(15 downto 0);
    signal r_h : t_h6 := (to_unsigned(C_DEF_K1, 10), to_unsigned(C_DEF_K2, 10), to_unsigned(C_DEF_K3, 10),
                          to_unsigned(C_DEF_K5, 10), to_unsigned(C_DEF_K6, 10), to_unsigned(C_DEF_P12, 10));
    signal r_g : t_g6 := (to_unsigned(C_DEF_K1 * 64, 16), to_unsigned(C_DEF_K2 * 64, 16), to_unsigned(C_DEF_K3 * 64, 16),
                          to_unsigned(C_DEF_K5 * 64, 16), to_unsigned(C_DEF_K6 * 64, 16), to_unsigned(C_DEF_P12 * 64, 16));
    signal s_ri     : unsigned(2 downto 0) := (others => '0');
    signal s_rp     : unsigned(1 downto 0) := (others => '0');
    signal s_rh     : unsigned(9 downto 0) := (others => '0');
    signal s_rt     : unsigned(15 downto 0) := (others => '0');
    signal s_kd     : signed(16 downto 0) := (others => '0');
    signal s_zone   : unsigned(2 downto 0) := "011";  -- K4 Order step (n = -1)
    signal s_glide  : std_logic := '1';               -- S7
    signal s_chk    : std_logic := '1';               -- S8  0 Lines / 1 Checker
    signal s_log    : std_logic := '0';               -- S9  0 Even / 1 Log
    signal s_fine   : std_logic := '0';               -- S10 0 Bold / 1 Fine
    signal s_src    : std_logic := '0';               -- S11 0 Video / 1 Osc

    signal s_cx, s_cy : signed(13 downto 0) := (others => '0');
    signal s_t3     : signed(27 downto 0) := (others => '0');
    -- derived constants
    signal s_v5     : unsigned(18 downto 0) := (others => '0');
    signal s_lsub   : signed(19 downto 0) := to_signed(137000, 20);    -- log2 R0 - log2(x step), Q14
    signal s_lq8    : signed(11 downto 0) := (others => '0');
    signal s_hq     : signed(4 downto 0) := (others => '0');           -- Log: lines per octave, half-octave steps
    signal s_hqn    : signed(5 downto 0) := (others => '0');           -- ... divided by |n|
    signal s_lq6    : signed(11 downto 0) := (others => '0');          -- log2(q / ln 2), Q6
    signal s_qh     : std_logic := '0';                                -- q mantissa: 0 = 1, 1 = 1.5
    signal s_qsh    : unsigned(2 downto 0) := to_unsigned(3, 3);       -- 3 - its exponent (0 .. 7)
    signal s_turn   : unsigned(17 downto 0) := (others => '0');
    signal s_gain   : unsigned(7 downto 0) := to_unsigned(191, 8);
    signal s_D16    : unsigned(15 downto 0) := to_unsigned(32768, 16); -- band duty, Q16
    signal s_offre, s_offrn, s_offim : unsigned(15 downto 0) := (others => '0');   -- Re (cos >= 0 / < 0), Im
    -- SPEED: decode, serial shifter, the Re phase accumulator (Q20 cycles)
    signal s_vneg, s_vzero : std_logic := '0';
    signal s_av, s_u, s_ls : unsigned(9 downto 0) := (others => '0');
    signal s_spd    : unsigned(19 downto 0) := (others => '0');
    signal s_ssh    : unsigned(3 downto 0) := (others => '0');
    signal s_anim   : unsigned(19 downto 0) := (others => '0');
    signal s_na     : unsigned(1 downto 0) := "00";                    -- |n| - 1
    signal s_nneg   : std_logic := '1';
    signal s_cA     : signed(15 downto 0) := to_signed(16384, 16);     -- log2(2 / |n|), Q14
    signal s_c1     : signed(8 downto 0) := (others => '0');           -- log2 |n|, Q6

    -- vblank sequencer + serial multiplier (the pole's position)
    signal s_seq   : unsigned(6 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse, s_vs_d : std_logic := '0';
    signal s_saw_avid   : std_logic := '0';
    signal sm_go   : std_logic := '0';
    signal sm_k    : signed(12 downto 0) := (others => '0');
    signal sm_w    : unsigned(11 downto 0) := (others => '0');
    signal sm_a    : signed(24 downto 0) := (others => '0');
    signal sm_b    : unsigned(11 downto 0) := (others => '0');
    signal sm_acc  : signed(25 downto 0) := (others => '0');
    signal sm_n    : unsigned(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- raster measurement + generated avid (hypnos)
    ----------------------------------------------------------------------
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Wl    : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_ystp  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_mcls  : std_logic_vector(1 downto 0) := "00";
    signal s_lacc  : unsigned(19 downto 0) := (others => '0');
    signal s_avid_q, s_avid_r : std_logic := '0';
    signal s_hs_q, s_hs_r     : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_rsr   : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc    : unsigned(3 downto 0) := "1000";
    signal g_cnt   : unsigned(11 downto 0) := (others => '0');

    signal s_dx       : signed(14 downto 0) := (others => '0');
    signal s_dyc      : signed(14 downto 0);
    signal s_dyq      : signed(22 downto 0) := (others => '0');   -- dy, Q8 x-pixel units
    signal g_go       : std_logic;
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- EBR delay lines (one write, one registered read each; 32-bit words)
    ----------------------------------------------------------------------
    type t_dl32 is array(0 to 255) of std_logic_vector(31 downto 0);
    type t_dl16 is array(0 to 255) of std_logic_vector(15 downto 0);
    signal dlh  : t_dl32 := (others => (others => '0'));      -- log2 hi + quadrant flags (stage 8), fold flags (stage 18, 24)
    signal dlr  : t_dl32 := (others => (others => '0'));      -- log2 r (stage 15), its Q6 part again (stage 21)
    signal dls  : t_dl16 := (others => (others => '0'));      -- generated avid + syncs
    attribute no_rw_check of dlh : signal is true;
    attribute no_rw_check of dlr : signal is true;
    attribute no_rw_check of dls : signal is true;
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal dlh_q, dlr_q : std_logic_vector(31 downto 0) := (others => '0');
    signal dlh_d, dlr_d : std_logic_vector(31 downto 0);
    signal dls_q : std_logic_vector(15 downto 0) := x"0006";
    signal dls_d : std_logic_vector(15 downto 0);
    type t_sy is array(46 to 47) of std_logic_vector(3 downto 0);
    signal sy : t_sy := (others => "0110");

    ----------------------------------------------------------------------
    -- [D] source field
    ----------------------------------------------------------------------
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot    : unsigned(7 downto 0) := (others => '0');
    signal s_oxs   : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys   : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum  : unsigned(7 downto 0) := (others => '0');
    -- gain table: (level - 16) * K6 gain (Osc: level * 1.5 gain), refilled every field
    signal wt_ram  : t_dl16 := (others => (others => '0'));
    attribute no_rw_check of wt_ram : signal is true;
    signal wt_ra   : unsigned(7 downto 0);
    signal wt_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal wt_d    : std_logic_vector(15 downto 0);
    signal wt_i    : unsigned(7 downto 0) := (others => '0');
    signal wt_acc  : unsigned(15 downto 0) := (others => '0');
    signal wt_run  : std_logic := '0';
    signal s_sl, s_sfirst : unsigned(9 downto 0) := (others => '0');
    signal s_sv0, s_sv, s_svd : std_logic := '0';
    signal s_col, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal bx_ram  : t_dl16 := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal bx_d    : std_logic_vector(15 downto 0);
    signal s_bcnt  : unsigned(5 downto 0) := (others => '0');
    signal s_bsum, s_eb : unsigned(14 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal lb_ram  : t_dl16 := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal s_eq, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa   : unsigned(7 downto 0) := (others => '0');
    signal lb_we, s_efirst : std_logic := '0';
    -- the linear interpolation between the 8-px samples
    signal s_cprev : unsigned(15 downto 0) := (others => '0');
    signal s_cd    : signed(16 downto 0) := (others => '0');
    signal s_cv    : unsigned(18 downto 0) := (others => '0');          -- the bend, Q14 octaves + 3 bits
    signal s_cn    : unsigned(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    -- S1..S8: |dx|,|dy| -> hi/lo -> log2 hi, log2 lo (Q14)
    signal s1_a, s1_b : unsigned(12 downto 0) := (others => '0');
    signal s1_dxs, s1_dys : std_logic := '0';
    signal s2_hi, s2_lo : unsigned(12 downto 0) := (others => '0');
    signal s2_sw, s2_dxs, s2_dys, s2_lz : std_logic := '0';
    signal s3_hi, s3_lo : unsigned(12 downto 0) := (others => '0');
    signal s3_mh, s3_ml : unsigned(3 downto 0) := (others => '0');
    signal s4_ih, s4_il : unsigned(7 downto 0) := (others => '0');
    signal s4_th, s4_tl, s5_th, s5_tl, s6_th, s6_tl : unsigned(3 downto 0) := (others => '0');
    signal s4_mh, s4_ml, s5_mh, s5_ml, s6_mh, s6_ml, s7_mh, s7_ml : unsigned(3 downto 0) := (others => '0');
    signal lvh_q, lsh_q, lvl_q, lsl_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s6_lvh, s6_lsh, s6_lvl, s6_lsl : std_logic_vector(15 downto 0) := (others => '0');
    signal s7_lph, s7_lpl : unsigned(10 downto 0) := (others => '0');
    signal s7_lvh, s7_lvl : unsigned(13 downto 0) := (others => '0');
    signal s8_lh, s8_ll : unsigned(17 downto 0) := (others => '0');
    type t_q is array(3 to 8) of std_logic_vector(3 downto 0);    -- sw, dxs, dys, lz
    signal qsr : t_q := (others => (others => '0'));

    -- S9..S15: u = log2(hi/lo) -> log2 r, theta
    signal s9_ua  : unsigned(17 downto 0) := (others => '0');
    signal s10_ia : unsigned(7 downto 0) := (others => '0');
    signal s10_tu, s11_tu, s12_tu : unsigned(6 downto 0) := (others => '0');
    signal ug_q, ugs_q, ua_q, uas_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s12_g, s12_gs, s12_a, s12_as : std_logic_vector(15 downto 0) := (others => '0');
    signal s13_gp, s13_ap : signed(21 downto 0) := (others => '0');
    signal s13_g, s13_a : unsigned(15 downto 0) := (others => '0');
    signal s14_g   : unsigned(13 downto 0) := (others => '0');
    signal s14_a   : unsigned(15 downto 0) := (others => '0');
    signal s15_lr  : unsigned(17 downto 0) := (others => '0');   -- log2 r (real pixels), Q14
    signal s15_th  : unsigned(17 downto 0) := (others => '0');   -- polar angle, 2^18 per turn

    -- S16..S25: a = n theta + turn -> log2|cos|, log2|sin| ; L0 = log2(2/|n|) + n (log2 r - log2 R0) + bend
    signal s16_tn, s17_a : unsigned(17 downto 0) := (others => '0');
    signal s18_o   : unsigned(14 downto 0) := (others => '0');
    signal s18_m, s19_m, s20_m, s21_m, s22_m : unsigned(3 downto 0) := (others => '0');
    signal s18_fl  : std_logic_vector(2 downto 0) := "000";      -- sgn sin, sgn cos, cos is the small one
    signal s19_li, s19_ta : unsigned(7 downto 0) := (others => '0');
    signal s19_lt, s20_lt, s21_lt : unsigned(5 downto 0) := (others => '0');
    signal s19_tt, s20_tt, s21_tt : unsigned(6 downto 0) := (others => '0');
    signal l3v_q, l3s_q, lcv_q, lcs_q, snt_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s21_lv, s21_ls, s21_cv, s21_cs, s21_sn : std_logic_vector(15 downto 0) := (others => '0');
    signal s21_lr  : unsigned(17 downto 0) := (others => '0');
    signal s22_lp  : unsigned(11 downto 0) := (others => '0');
    signal s22_cp  : unsigned(11 downto 0) := (others => '0');
    signal s22_sp  : unsigned(8 downto 0) := (others => '0');
    signal s22_lv, s22_cv : unsigned(13 downto 0) := (others => '0');
    signal s22_sv  : unsigned(10 downto 0) := (others => '0');
    signal s22_lrn : signed(19 downto 0) := (others => '0');
    signal s23_logo : unsigned(17 downto 0) := (others => '0');
    signal s23_lc  : unsigned(13 downto 0) := (others => '0');
    signal s23_sn  : unsigned(11 downto 0) := (others => '0');
    signal s23_mn  : signed(21 downto 0) := (others => '0');
    signal s24_ls  : signed(19 downto 0) := (others => '0');
    signal s24_lc  : unsigned(13 downto 0) := (others => '0');
    signal s25_lc  : unsigned(13 downto 0) := (others => '0');
    signal s25_ls  : signed(19 downto 0) := (others => '0');
    signal s25_cs  : std_logic := '0';
    signal s24_l0a, s25_l0 : signed(21 downto 0) := (others => '0');

    -- two phase chains: 0 = Re, 1 = Im
    type t_s22x2 is array(0 to 1) of signed(21 downto 0);
    type t_s6x2  is array(0 to 1) of signed(5 downto 0);
    type t_u19x2 is array(0 to 1) of unsigned(18 downto 0);
    type t_u16x2 is array(0 to 1) of unsigned(15 downto 0);
    type t_u14x2 is array(0 to 1) of unsigned(13 downto 0);
    type t_u11x2 is array(0 to 1) of unsigned(10 downto 0);
    type t_u10x2 is array(0 to 1) of unsigned(9 downto 0);
    type t_u9x2  is array(0 to 1) of unsigned(8 downto 0);
    type t_u8x2  is array(0 to 1) of unsigned(7 downto 0);
    type t_u6x2  is array(0 to 1) of unsigned(5 downto 0);
    type t_u2x2  is array(0 to 1) of unsigned(1 downto 0);
    type t_v16x2 is array(0 to 1) of std_logic_vector(15 downto 0);
    type t_bx2   is array(0 to 1) of std_logic;

    -- S26..S34: phase
    signal s26_l   : t_s22x2 := (others => (others => '0'));
    signal s26_pre : signed(13 downto 0) := (others => '0');
    signal s27_xa  : t_u8x2 := (others => (others => '0'));
    signal s27_t, s28_t, s29_t : t_u6x2 := (others => (others => '0'));
    signal s27_k, s28_k, s29_k, s30_k, s31_k : t_s6x2 := (others => (others => '0'));
    type t_u20x2 is array(0 to 1) of unsigned(19 downto 0);
    signal s27_pl  : t_u20x2 := (others => (others => '0'));
    signal exm_q, exs_q : t_v16x2 := (others => (others => '0'));
    signal s28_pl  : t_u16x2 := (others => (others => '0'));
    signal s29_m, s30_m, s31_mt : t_u16x2 := (others => (others => '0'));
    signal s29_s   : t_u8x2 := (others => (others => '0'));
    signal s30_ep  : t_u14x2 := (others => (others => '0'));
    signal s32_y1  : t_u19x2 := (others => (others => '0'));
    signal s32_f   : t_u2x2 := (others => (others => '0'));
    signal s31_sg, s32_sg : std_logic_vector(1 downto 0) := "00";
    signal s33_ng  : std_logic := '0';
    signal s33_p, s34_phi : t_u16x2 := (others => (others => '0'));

    -- S27..S34: the anti-aliasing width
    signal s27_lw  : signed(13 downto 0) := (others => '0');
    signal s28_lg, s29_lg, s29_tk : signed(11 downto 0) := (others => '0');
    signal s29_fl  : std_logic := '0';
    signal s30_v   : signed(12 downto 0) := (others => '0');
    signal s31_ri, s32_ri, s33_ri, s34_ri, s35_ri, s36_ri, s37_ri : unsigned(3 downto 0) := (others => '0');
    signal s31_rh, s32_rh, s33_rh, s34_rh, s35_rh, s36_rh, s37_rh, s38_rh : std_logic := '0';
    signal s31_sat, s32_sat, s33_sat : std_logic := '0';
    signal s34_wh  : unsigned(14 downto 0) := (others => '1');

    -- S35..S40: one-pixel box filter -> coverage 0..256
    signal s35_lo, s35_hi, s36_mh, s36_ml, s37_n : t_u16x2 := (others => (others => '0'));
    signal s36_k, s36_full, s37_full, s38_full, s39_full : t_bx2 := (others => '0');
    signal s38_nsh : t_u10x2 := (others => (others => '0'));
    signal s39_cp  : t_u11x2 := (others => (others => '0'));
    signal s40_cov : t_u9x2 := (others => (others => '0'));

    -- S41..S47: weave, luma, output conditioning
    signal s41_pp  : unsigned(9 downto 0) := (others => '0');
    signal s41_ab  : unsigned(5 downto 0) := (others => '0');
    signal s41_a   : unsigned(8 downto 0) := (others => '0');
    signal s42_c   : unsigned(8 downto 0) := (others => '0');
    signal s43_y, s44_y, s45_y, s45_f, s46_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal o_y     : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- a + b or a - b on ONE carry chain (riley; the low column makes the carry-in)
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;

    -- Exact signed product a * m, radix-4 Booth on m (hypnos): ceil(len(m)/2) partials of
    -- {0, +-a, +-2a}, negatives as one's complement with the +1s gathered in one correction word.
    function bmul(a : signed; m : signed) return signed is
        constant NA  : natural := a'length;
        constant NML : natural := m'length;
        constant NM  : natural := NML + (NML mod 2);
        constant NP  : natural := NA + NML;
        constant NPP : natural := NM / 2;
        type t_pp is array (0 to 7) of signed(NP - 1 downto 0);
        variable va   : signed(NA - 1 downto 0);
        variable vm   : signed(NM - 1 downto 0);
        variable me   : std_logic_vector(NM downto 0);
        variable a1, a2, mag, pp, corr : signed(NP - 1 downto 0);
        variable d    : std_logic_vector(2 downto 0);
        variable p    : t_pp;
    begin
        va := a;
        vm := resize(m, NM);
        me := std_logic_vector(vm) & '0';
        a1 := resize(va, NP);
        a2 := shift_left(a1, 1);
        corr := (others => '0');
        for i in 0 to 7 loop
            p(i) := (others => '0');
        end loop;
        for i in 0 to NPP - 1 loop
            d := me(2 * i + 2 downto 2 * i);
            case d is
                when "001" | "010" | "101" | "110" => mag := a1;
                when "011" | "100"                 => mag := a2;
                when others                        => mag := (others => '0');
            end case;
            if d(2) = '1' and d /= "111" then
                pp := not mag;
                corr(2 * i) := '1';
            else
                pp := mag;
            end if;
            p(i) := shift_left(pp, 2 * i);
        end loop;
        p(NPP) := corr;
        return ((p(0) + p(1)) + (p(2) + p(3))) + ((p(4) + p(5)) + (p(6) + p(7)));
    end function;

begin

    ------------------------------------------------------------------------
    -- raster measurement + GENERATED AVID (hypnos / cubist v0.3.3): start 2 or 3 clocks after the
    -- source's first avid rise, whichever keeps the hsync -> start parity, fixed field-latched width.
    ------------------------------------------------------------------------
    g_go <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;

            -- per-mode steps from the timing ID (decoded free-running): s_ystp = the field-line
            -- pitch in x-pixel units (Q8); the Osc lattice's steps per pixel / per field line
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_ystp <= to_unsigned(569, 10); s_mcls <= "10"; s_oys <= to_unsigned(537, 10);  -- 480i
                when 4          => s_ystp <= to_unsigned(288, 10); s_mcls <= "10"; s_oys <= to_unsigned(272, 10);  -- 480p
                when 8          => s_ystp <= to_unsigned(480, 10); s_mcls <= "10"; s_oys <= to_unsigned(453, 10);  -- 576i
                when 12         => s_ystp <= to_unsigned(240, 10); s_mcls <= "10"; s_oys <= to_unsigned(226, 10);  -- 576p
                when 5 | 6 | 14 => s_ystp <= to_unsigned(256, 10); s_mcls <= "01"; s_oys <= to_unsigned(181, 10);  -- 720p
                when 1 | 2 | 10 => s_ystp <= to_unsigned(512, 10); s_mcls <= "00"; s_oys <= to_unsigned(242, 10);  -- 1080i
                when others     => s_ystp <= to_unsigned(256, 10); s_mcls <= "00"; s_oys <= to_unsigned(121, 10);  -- 1080p
            end case;
            case s_mcls is
                when "10"   => s_oxs <= to_unsigned(204, 9);
                when "01"   => s_oxs <= to_unsigned(153, 9);
                when others => s_oxs <= to_unsigned(102, 9);
            end case;

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then                 -- avid just fell
                if s_xcnt > 16 and (s_xcnt > s_W + 2 or s_xcnt + 2 < s_W) then
                    s_W <= s_xcnt;
                end if;
                s_xcnt <= (others => '0');
                s_lacc <= s_lacc + s_ystp;
            end if;
            if s_vs_pulse = '1' then
                if s_lacc(19 downto 8) > 8 then
                    s_H <= s_lacc(19 downto 8);
                end if;
                s_lacc <= (others => '0');
            end if;

            if s_hs_r = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if s_avid_r = '1' and g_arm = '1' then
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
            if g_go = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                if g_cnt + 1 >= s_Wl then g_avid <= '0'; end if;
                g_cnt <= g_cnt + 1;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-field control latch + vblank sequencer (one small op per state).  The field tick is the
    -- first vsync EDGE after active video (analog vsync serrates).
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_raw  : unsigned(9 downto 0);
        variable v_e    : signed(10 downto 0);
        variable v_t    : unsigned(15 downto 0);
        variable v_st   : signed(15 downto 0);
        variable v_gn   : unsigned(15 downto 0);
        variable v_r    : unsigned(9 downto 0);
        variable v_cn   : signed(19 downto 0);
        variable v_big  : std_logic;
        variable v_off  : signed(5 downto 0);
        variable v_g3k  : unsigned(11 downto 0);
        variable v_q5   : signed(4 downto 0);
        variable v_u11  : unsigned(10 downto 0);
        variable v_os   : unsigned(15 downto 0);
        variable v_m7   : unsigned(6 downto 0);
        variable v_u27  : unsigned(26 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            s_vs_d       <= s_vs_pulse;
            sm_go <= '0';
            if data_in.avid = '1' then s_saw_avid <= '1'; end if;

            -- SPEED mantissa << (octave + 2), one bit per clock (Hypnos); loaded at sequencer state 39
            -- (a negative speed loads the one's complement and shifts in ones: -(m << k) - 1)
            if s_ssh /= 0 then
                s_spd <= s_spd(18 downto 0) & s_vneg;
                s_ssh <= s_ssh - 1;
            end if;
            if s_seq = 39 and data_in.avid = '0' then
                if s_vzero = '1' then s_spd <= (others => '0'); s_ssh <= (others => '0');
                else
                    v_m7 := '1' & s_ls(5 downto 0);                  -- 64 + f
                    if s_vneg = '1' then v_m7 := not v_m7; end if;
                    s_spd <= (19 downto 9 => s_vneg) & v_m7 & s_vneg & s_vneg;   -- (64 + f) << 2, signed
                    s_ssh <= s_ls(9 downto 6);
                end if;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_saw_avid = '1' then
                s_vs_pulse <= '1';
                s_saw_avid <= '0';

                -- K4 Order: 8 steps; a new step is taken 8 counts inside it, or at either end of the knob
                v_r := unsigned(registers_in(3));
                if s_init = '1' or (v_r(6 downto 3) /= "0000" and v_r(6 downto 3) /= "1111")
                   or (v_r(9 downto 3) = "0000000") or (v_r(9 downto 3) = "1111111") then
                    s_zone <= v_r(9 downto 7);
                end if;
                s_glide <= registers_in(6)(0);
                s_chk   <= registers_in(6)(1);
                s_log   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_Wl    <= s_W;
                s_seq   <= to_unsigned(95, 7);
                s_ri    <= (others => '0');
                s_rp    <= (others => '0');

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                -- KNOB RINGS (states 93 .. 76, three per knob): +-1-count hold against pot jitter, then
                -- the glide: 1/16 of the gap per field, never slower than half a count, landing exactly.
                -- Glide off (or the first field): the held value itself, the same field.
                if s_seq <= 93 and s_seq >= 76 then
                    case s_rp is
                        when "00" =>
                            case to_integer(s_ri) is
                                when 0      => v_raw := unsigned(registers_in(0));
                                when 1      => v_raw := unsigned(registers_in(1));
                                when 2      => v_raw := unsigned(registers_in(2));
                                when 3      => v_raw := unsigned(registers_in(4));
                                when 4      => v_raw := unsigned(registers_in(5));
                                when others => v_raw := unsigned(registers_in(7));
                            end case;
                            v_e := signed('0' & v_raw) - signed('0' & r_h(0));
                            if s_init = '1' or (v_e /= 0 and v_e /= 1 and v_e /= -1) then s_rh <= v_raw; else s_rh <= r_h(0); end if;
                            s_rp <= "01";
                        when "01" =>
                            -- X, Y and Speed: a detent at the centre
                            if (s_ri(2 downto 1) = "00" or s_ri = 5) and s_rh > 512 - C_DB and s_rh < 512 + C_DB then v_t := to_unsigned(32768, 16);
                            else v_t := s_rh & "000000"; end if;
                            s_rt <= v_t;
                            s_kd <= signed('0' & v_t) - signed('0' & r_g(0));
                            s_rp <= "10";
                        when others =>
                            if s_kd < 16 * C_GMIN and s_kd > -16 * C_GMIN then
                                if s_kd(16) = '0' then v_st := to_signed(C_GMIN, 16); else v_st := to_signed(-C_GMIN, 16); end if;
                            else
                                v_st := resize(shift_right(s_kd, C_GL), 16);
                            end if;
                            if s_glide = '0' or s_init = '1' or (s_kd <= C_GMIN and s_kd >= -C_GMIN) then v_gn := s_rt;
                            else v_gn := unsigned(signed(r_g(0)) + v_st); end if;
                            for i in 0 to 4 loop
                                r_g(i) <= r_g(i + 1); r_h(i) <= r_h(i + 1);
                            end loop;
                            r_g(5) <= v_gn; r_h(5) <= s_rh;
                            s_ri <= s_ri + 1;
                            s_rp <= "00";
                    end case;
                end if;

                -- state 19: offim + a (n < 0) / - a (n > 0) ; state 18: the opposite (Fine), the same (Bold)
                v_os := f_addsub(s_offim, s_anim(19 downto 4), s_nneg xor (s_seq(0) or not s_fine));
                case to_integer(s_seq) is
                    -- ORDER: n = zone - 4 (0 .. 3) or zone - 3 (4 .. 7)
                    when 95 =>
                        s_nneg <= not s_zone(2);
                        if s_zone(2) = '0' then s_na <= not s_zone(1 downto 0); else s_na <= s_zone(1 downto 0); end if;
                    when 94 =>
                        case s_na is
                            when "00"   => s_cA <= to_signed(16384, 16);  s_c1 <= to_signed(0, 9);
                            when "01"   => s_cA <= to_signed(0, 16);      s_c1 <= to_signed(64, 9);
                            when "10"   => s_cA <= to_signed(-9585, 16);  s_c1 <= to_signed(101, 9);
                            when others => s_cA <= to_signed(-16384, 16); s_c1 <= to_signed(128, 9);
                        end case;

                    -- the pole: cx = W/2 + (K1 - 512) Wv 3/2048, cy = H/2 + (K2 - 512) H/512 (serial multiplies)
                    when 75 =>
                        sm_k <= signed((not r_g(0)(15)) & r_g(0)(14 downto 3));        -- (g - 32768) >> 3
                        -- X travel in VIRTUAL pixels (the 1920-wide canvas / x step), so SD shows the centre
                        -- 4:3 of the HD picture; the centre itself is the measured W / 2
                        case s_mcls is
                            when "10"   => sm_w <= to_unsigned(960, 12);
                            when "01"   => sm_w <= to_unsigned(1280, 12);
                            when others => sm_w <= to_unsigned(1920, 12);
                        end case;
                        sm_go <= '1';
                    when 61 =>
                        v_u27 := resize(unsigned(sm_acc), 27) + resize(unsigned(sm_acc(24 downto 0)) & '0', 27);
                        s_t3 <= signed(sm_acc(25) & v_u27);        -- 3 x, no sign bit added to itself
                        sm_k <= signed((not r_g(1)(15)) & r_g(1)(14 downto 3)); sm_w <= s_H; sm_go <= '1';
                    when 60 => s_cx <= signed(resize(s_W(11 downto 1), 14)) + resize(shift_right(s_t3, 14), 14);
                    when 47 => s_cy <= signed(resize(s_H(11 downto 1), 14)) + resize(shift_right(sm_acc, 12), 14);

                    -- PERIOD, Even: log2 R0 = 10.5 - K3 5/1024 octaves (R0 1448 -> 45 px; Q14: 172032 - 5 g3 / 4),
                    -- - log2(x step) (720p 0.585, SD 1 octave); n = 3, 4 sit on a radius 0.5 octave larger.
                    -- Log: the pivot radius is fixed at 256 px.
                    when 59 => s_v5 <= shift_left(resize(r_g(2), 19), 2) + resize(r_g(2), 19);
                    when 58 =>
                        if s_log = '0' and s_na(1) = '1' then v_big := '1'; else v_big := '0'; end if;
                        case s_mcls is
                            when "10"   => if v_big = '1' then v_cn := to_signed(8192 - 16384, 20); else v_cn := to_signed(-16384, 20); end if;
                            when "01"   => if v_big = '1' then v_cn := to_signed(8192 - 9585, 20);  else v_cn := to_signed(-9585, 20); end if;
                            when others => if v_big = '1' then v_cn := to_signed(8192, 20);         else v_cn := to_signed(0, 20); end if;
                        end case;
                        if s_log = '1' then s_lsub <= to_signed(131072, 20) + v_cn;
                        else                s_lsub <= to_signed(172032, 20) + v_cn - signed(resize(shift_right(s_v5, 2), 20)); end if;
                    -- PERIOD, Log: lines per octave of r, q = 2^(-1.5 + K3 5/1024), in half-octave steps
                    -- (mantissa 1 or 1.5) with hysteresis and the two end exceptions; the phase runs at q / |n|
                    when 55 => s_lq8 <= to_signed(-384, 12) + signed(resize(shift_right(s_v5, 8), 12));
                    when 54 =>
                        if s_init = '1' or (s_lq8(6 downto 3) /= "0000" and s_lq8(6 downto 3) /= "1111")
                           or r_g(2)(15 downto 10) = "000000" or r_g(2)(15 downto 10) = "111111" then
                            s_hq <= s_lq8(11 downto 7);
                        end if;
                    when 53 =>
                        case s_na is
                            when "00"   => v_off := to_signed(0, 6);
                            when "01"   => v_off := to_signed(2, 6);
                            when "10"   => v_off := to_signed(3, 6);
                            when others => v_off := to_signed(4, 6);
                        end case;
                        s_hqn <= resize(s_hq, 6) - v_off;
                        s_lq6 <= shift_left(resize(s_hq, 12), 5) + 34;
                    when 52 =>
                        s_qh  <= s_hqn(0);
                        v_q5 := to_signed(3, 5) - s_hqn(5 downto 1);
                        s_qsh <= unsigned(v_q5(2 downto 0));
                    -- TURN, CAMERA ; the band duty (constant: Bold 1/2, Fine 1/6 on the half-integer levels)
                    when 51 =>
                        s_turn <= (not r_g(3)(15)) & r_g(3)(14 downto 0) & "00";
                        v_g3k := shift_left(resize(r_g(4)(15 downto 6), 12), 1) + resize(r_g(4)(15 downto 6), 12);
                        s_gain <= v_g3k(11 downto 4);
                        if s_fine = '1' then s_D16 <= to_unsigned(10923, 16); s_offim <= to_unsigned(38229, 16);
                        else                 s_D16 <= to_unsigned(32768, 16); s_offim <= (others => '0'); end if;

                    -- SPEED (P12, Hypnos v1.1.0 shape): bipolar, |v| < 24 counts = still; exponential,
                    -- 2^-12 .. 2^-2 cycle per field (piecewise-linear mantissa 1 + f/64, octave = ls / 64);
                    -- the glided P12 (the knob ring) is the speed, never the phase
                    when 42 =>
                        -- |P12 - 512| as a one's complement (0 and 1023 both give 511)
                        s_vneg <= not r_g(5)(15);
                        if r_g(5)(15) = '1' then s_av <= '0' & r_g(5)(14 downto 6);
                        else                     s_av <= '0' & not r_g(5)(14 downto 6); end if;
                    when 41 =>
                        -- u = |v| - 24; the borrow is the dead zone
                        v_u11 := resize(s_av, 11) - 24;
                        s_vzero <= v_u11(10);
                        s_u <= v_u11(9 downto 0);
                    when 40 =>
                        s_ls <= s_u + shift_right(s_u, 2) + shift_right(s_u, 4);
                    -- (39: the shifter loads, above; it needs at most 10 clocks)
                    -- the Re phase accumulator: one cycle = 2^20, wraps forever without a seam (a negative
                    -- step is the shifter's one's complement: 2^-20 cycle short, invisible)
                    when 20 =>
                        s_anim <= s_anim + s_spd;
                    -- stream offsets (one shared adder; n > 0 reverses the travel so up = outward):
                    --  Bold: one offset for both lobes, added to the SIGNED PHI.  The phase stays continuous
                    --    through the nodal lines (every edge anti-aliased), so the lines flow THROUGH the
                    --    pole: out of the cos >= 0 lobes, into the others.  (Adding it to |PHI| instead would
                    --    stream every lobe outward, but the colour alternation then puts an un-anti-aliased
                    --    seam, crawling with slivers, on every nodal line.)
                    --  Fine: added to |PHI| (before the cosine's sign): every lobe streams out of the pole;
                    --    the lines sit on the half-integer levels, so the mirror image is seamless.
                    --  Log: no sign; both families take the same shift of log2|PHI|, a seamless zoom flow
                    --    out of the pole (the Im offset follows at state 17).
                    when 19 => s_offre <= v_os;
                    when 18 => s_offrn <= v_os;
                    when 17 => if s_log = '1' then s_offim <= s_offre; end if;
                    when 44 => s_init <= '0';
                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    p_smul : process(clk)
    begin
        if rising_edge(clk) then
            if sm_go = '1' then
                sm_a   <= resize(sm_k, 25);
                sm_b   <= sm_w;
                sm_acc <= (others => '0');
                sm_n   <= to_unsigned(12, 4);
            elsif sm_n /= 0 then
                if sm_b(0) = '1' then sm_acc <= sm_acc + resize(sm_a, 26); end if;
                sm_a <= shift_left(sm_a, 1);
                sm_b <= shift_right(sm_b, 1);
                sm_n <= sm_n - 1;
            end if;
        end if;
    end process p_smul;

    ------------------------------------------------------------------------
    -- coordinate accumulators on the GENERATED avid (hypnos): dx in real pixels, dy in x-pixel units
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif g_go = '1' then
                -- one clock before the line: dx = -cx - 1, so the pixel left of the picture is a
                -- real one for the output filter
                if s_newframe = '1' then
                    v_dy0 := shift_left(resize(-s_cy, 23), 8);
                    v_dy0(7) := '1';                                   -- +0.5
                    s_dyq <= v_dy0;
                    s_newframe <= '0';
                else
                    s_dyq <= s_dyq + signed(resize(s_ystp, 23));
                end if;
                s_dx <= not resize(s_cx, 15);
            elsif g_avid = '1' then
                s_dx <= s_dx + 1;
            end if;
        end if;
    end process p_acc;
    s_dyc <= s_dyq(22 downto 8);

    ------------------------------------------------------------------------
    -- EBR delay lines.  Write address = free-running counter; each read runs X behind it (never the
    -- written address).  Spare bits of a word carry flags round again.
    ------------------------------------------------------------------------
    dlh_d <= "00000" & dlh_q(24 downto 23) & s18_fl & qsr(8) & std_logic_vector(s8_lh);
    dlr_d <= "0000" & std_logic_vector(s21_lr(17 downto 8)) & std_logic_vector(s15_lr);
    dls_d <= x"000" & g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_wa <= s_wa + 1;
            dlh(to_integer(s_wa)) <= dlh_d;
            dlh_q <= dlh(to_integer(s_wa - C_XH));
            dlr(to_integer(s_wa)) <= dlr_d;
            dlr_q <= dlr(to_integer(s_wa - C_XR));
            dls(to_integer(s_wa)) <= dls_d;
            dls_q <= dls(to_integer(s_wa - C_XS));
            sy(46) <= dls_q(3 downto 0);
            sy(47) <= sy(46);
        end if;
    end process p_dl;

    -- table ROMs (EBR): every address is a register, every read is registered
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lvh_q <= C_LOGV(to_integer(s4_ih));
            lsh_q <= C_LOGS(to_integer(s4_ih));
            lvl_q <= C_LOGV(to_integer(s4_il));
            lsl_q <= C_LOGS(to_integer(s4_il));
            ug_q  <= C_UG(to_integer(s10_ia));
            ugs_q <= C_UGS(to_integer(s10_ia));
            ua_q  <= C_UA(to_integer(s10_ia));
            uas_q <= C_UAS(to_integer(s10_ia));
            l3v_q <= C_LOGV(to_integer(s19_li));
            l3s_q <= C_LOGS(to_integer(s19_li));
            lcv_q <= C_LCV(to_integer(s19_ta));
            lcs_q <= C_LCS(to_integer(s19_ta));
            snt_q <= C_SNT(to_integer(s19_ta));
            exm_q(0) <= C_EXM(to_integer(s27_xa(0)));
            exs_q(0) <= C_EXS(to_integer(s27_xa(0)));
            exm_q(1) <= C_EXM(to_integer(s27_xa(1)));
            exs_q(1) <= C_EXS(to_integer(s27_xa(1)));
        end if;
    end process p_rom;

    ------------------------------------------------------------------------
    -- [D] SOURCE FIELD (kinetic riley): luma or the Osc lattice -> x K6 (a table refilled every field:
    -- (level - 16) * gain >> 6) -> 32-px box, edge-replicated and reloaded at every line start ->
    -- vertical 1/8 IIR on every 8th column -> linear interpolation back to pixel rate.  The result
    -- s_cv is the bend in Q14 octaves (white at 100 %: 1.28 octaves; Osc up to 1.9); it is tapped
    -- unaligned (about 30 px late: the camera is never shown).
    ------------------------------------------------------------------------
    bx_d  <= "000000" & std_logic_vector(s_sl);
    wt_d  <= "000000" & std_logic_vector(wt_acc(15 downto 6));
    wt_ra <= s_osum when s_src = '1' else unsigned(data_in.y(9 downto 2));
    p_src : process(clk)
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(9 downto 0);
        variable v_d   : signed(17 downto 0);
        variable v_a   : signed(17 downto 0);
        variable v_x, v_y : unsigned(6 downto 0);
        variable v_g   : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- RAMs (one write each)
            wt_q <= wt_ram(to_integer(wt_ra));
            if wt_run = '1' then wt_ram(to_integer(wt_i)) <= wt_d; end if;
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= bx_d; end if;
            lb_q <= lb_ram(to_integer(s_col(10 downto 3)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= std_logic_vector(lb_wd); end if;

            -- gain table fill (vblank, 256 clocks after the sequencer has the gain): entry i =
            -- (max(i - 16, 0) * gain) >> 6, by accumulation; Osc: (i * 1.5 gain) >> 6
            if s_src = '1' then v_g := resize(s_gain, 9) + resize(s_gain(7 downto 1), 9); else v_g := resize(s_gain, 9); end if;
            if s_seq = 43 then
                wt_run <= '1'; wt_i <= (others => '0'); wt_acc <= (others => '0');
            elsif wt_run = '1' then
                if s_src = '1' or wt_i(7 downto 4) /= "0000" then wt_acc <= wt_acc + v_g; end if;
                wt_i <= wt_i + 1;
                if wt_i = 255 then wt_run <= '0'; end if;
            end if;

            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting (the only thing that moves by itself)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avid_q = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_vs_pulse = '1' then s_ot <= s_ot + 1; end if;
            if s_vs_d = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);
            end if;
            if s_ox(15) = '1' then v_x := not s_ox(14 downto 8); else v_x := s_ox(14 downto 8); end if;
            if s_oy(15) = '1' then v_y := not s_oy(14 downto 8); else v_y := s_oy(14 downto 8); end if;
            v_s := resize(v_x, 8) + resize(v_y, 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample (10 bits), already scaled by K6: fabric copy of the table read
            s_sl <= unsigned(wt_q(9 downto 0));
            s_sv0 <= data_in.avid; s_sv <= s_sv0; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 samples; the samples before the line start count as the first one
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(225, 8); end if;
            if s_bcnt < 32 then v_tap := s_sfirst; else v_tap := unsigned(bx_q(9 downto 0)); end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                s_bsum <= shift_left(resize(s_sl, 15), 5);
                s_bcnt <= to_unsigned(1, 6);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 32 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on every 8th column: q += (box - q + 4) >> 3
            s_eb <= s_bsum; s_eq <= unsigned(lb_q);
            if s_bv = '1' and s_bcol(2 downto 0) = "111" then s_ev <= '1'; else s_ev <= '0'; end if;
            s_ecol <= s_bcol;
            v_d := signed(resize(s_eb, 18)) - signed(resize(s_eq, 18)) + 4;
            v_a := signed(resize(s_eq, 18)) + shift_right(v_d, 3);
            lb_wd <= unsigned(v_a(15 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 3);
            if s_ecol(10 downto 3) = 0 then s_efirst <= '1'; else s_efirst <= '0'; end if;

            -- interpolation: 8 equal steps from the previous sample to this one; the first sample of
            -- a line loads directly
            if s_cn /= 0 then
                s_cv <= unsigned(signed(s_cv) + resize(s_cd, 19));
                s_cn <= s_cn - 1;
            end if;
            if lb_we = '1' then
                s_cprev <= lb_wd;
                if s_efirst = '1' then
                    s_cv <= lb_wd & "000";
                    s_cd <= (others => '0');
                    s_cn <= (others => '0');
                else
                    s_cd <= signed('0' & lb_wd) - signed('0' & s_cprev);
                    s_cn <= to_unsigned(8, 4);
                end if;
            end if;
        end if;
    end process p_src;

    ------------------------------------------------------------------------
    -- S1..S8: |dx|,|dy| (13-bit), hi = max, lo = min, and their log2 (Q14)  (hypnos)
    ------------------------------------------------------------------------
    p_logs : process(clk)
        variable va, vb : unsigned(14 downto 0);
        variable vh, vl : unsigned(12 downto 0);
        variable mh, ml : integer range 0 to 12;
    begin
        if rising_edge(clk) then
            va := unsigned(abs(s_dx));
            vb := unsigned(abs(s_dyc));
            s1_a <= va(12 downto 0);          -- the pole stays within 1.25 W / 1.5 H of every pixel: no clamp
            s1_b <= vb(12 downto 0);
            s1_dxs <= s_dx(14);
            s1_dys <= s_dyc(14);
            -- S2
            if s1_a >= s1_b then s2_hi <= s1_a; s2_lo <= s1_b; s2_sw <= '0';
            else                 s2_hi <= s1_b; s2_lo <= s1_a; s2_sw <= '1'; end if;
            if s1_a = 0 or s1_b = 0 then s2_lz <= '1'; else s2_lz <= '0'; end if;
            s2_dxs <= s1_dxs;
            s2_dys <= s1_dys;
            -- S3: msb (the exact centre reads as 1)
            vh := s2_hi; if vh = 0 then vh := to_unsigned(1, 13); end if;
            vl := s2_lo; if vl = 0 then vl := to_unsigned(1, 13); end if;
            mh := 0; ml := 0;
            for i in 0 to 12 loop
                if vh(i) = '1' then mh := i; end if;
                if vl(i) = '1' then ml := i; end if;
            end loop;
            s3_hi <= vh; s3_lo <= vl;
            s3_mh <= to_unsigned(mh, 4); s3_ml <= to_unsigned(ml, 4);
            -- S4: normalise -> 8-bit segment index + 4-bit fraction
            vh := shift_left(s3_hi, 12 - to_integer(s3_mh));
            vl := shift_left(s3_lo, 12 - to_integer(s3_ml));
            s4_ih <= vh(11 downto 4); s4_th <= vh(3 downto 0);
            s4_il <= vl(11 downto 4); s4_tl <= vl(3 downto 0);
            s4_mh <= s3_mh; s4_ml <= s3_ml;
            -- S5, S6 (EBR read, fabric register)
            s5_th <= s4_th; s5_tl <= s4_tl; s5_mh <= s4_mh; s5_ml <= s4_ml;
            s6_th <= s5_th; s6_tl <= s5_tl; s6_mh <= s5_mh; s6_ml <= s5_ml;
            s6_lvh <= lvh_q; s6_lsh <= lsh_q; s6_lvl <= lvl_q; s6_lsl <= lsl_q;
            -- S7
            s7_lph <= resize(unsigned(s6_lsh(6 downto 0)) * s6_th, 11);
            s7_lpl <= resize(unsigned(s6_lsl(6 downto 0)) * s6_tl, 11);
            s7_lvh <= unsigned(s6_lvh(13 downto 0));
            s7_lvl <= unsigned(s6_lvl(13 downto 0));
            s7_mh <= s6_mh; s7_ml <= s6_ml;
            -- S8
            s8_lh <= shift_left(resize(s7_mh, 18), 14) + resize(s7_lvh, 18) + resize(shift_right(s7_lph, 4), 18);
            s8_ll <= shift_left(resize(s7_ml, 18), 14) + resize(s7_lvl, 18) + resize(shift_right(s7_lpl, 4), 18);

            qsr(3) <= s2_sw & s2_dxs & s2_dys & s2_lz;
            for i in 4 to 8 loop qsr(i) <= qsr(i - 1); end loop;
        end if;
    end process p_logs;

    ------------------------------------------------------------------------
    -- S9..S15: u = log2(hi/lo) ; log2 r = log2 hi + 0.5 log2(1 + 4^-u) ; the octant angle
    -- atan(2^-u) (EBR tables, interpolated) ; the full angle from the swap and signs  (hypnos)
    ------------------------------------------------------------------------
    p_polar : process(clk)
        variable vu   : unsigned(17 downto 0);
        variable vmag : unsigned(17 downto 0);
        variable q    : std_logic_vector(3 downto 0);
        variable vlh  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- S9
            vu := s8_lh - s8_ll;
            if qsr(8)(0) = '1' then vu := to_unsigned(16 * 16384 - 1, 18); end if;
            s9_ua <= vu;
            -- S10
            s10_ia <= s9_ua(17 downto 10);
            s10_tu <= s9_ua(9 downto 3);
            -- S11, S12
            s11_tu <= s10_tu;
            s12_tu <= s11_tu;
            s12_g <= ug_q; s12_gs <= ugs_q; s12_a <= ua_q; s12_as <= uas_q;
            -- S13: interpolation products (slopes are negative)
            s13_gp <= resize(bmul(signed(s12_gs(10 downto 0)), signed('0' & s12_tu)), 22);
            s13_ap <= resize(bmul(signed(s12_as(10 downto 0)), signed('0' & s12_tu)), 22);
            s13_g  <= unsigned(s12_g);
            s13_a  <= unsigned(s12_a);
            -- S14
            s14_g <= resize(unsigned(signed(resize(s13_g, 17)) + resize(shift_right(s13_gp, 7), 17)), 14);
            s14_a <= resize(unsigned(signed(resize(s13_a, 17)) + resize(shift_right(s13_ap, 7), 17)), 16);
            -- S15
            vlh := unsigned(dlh_q(17 downto 0));
            s15_lr <= vlh + resize(s14_g, 18);
            q := dlh_q(21 downto 18);
            if q(3) = '1' then vmag := to_unsigned(65536, 18) - resize(s14_a, 18);
            else               vmag := resize(s14_a, 18); end if;
            if    q(2) = '0' and q(1) = '0' then s15_th <= vmag;
            elsif q(2) = '1' and q(1) = '0' then s15_th <= to_unsigned(131072, 18) - vmag;
            elsif q(2) = '1' and q(1) = '1' then s15_th <= to_unsigned(131072, 18) + vmag;
            else                                 s15_th <= to_unsigned(0, 18) - vmag; end if;
        end if;
    end process p_polar;

    ------------------------------------------------------------------------
    -- S16..S25: a = n theta + turn ; log2|cos a| and log2|sin a| ; L0
    --   (negative n: one's complements, 1 LSB of angle / of Q14 log)
    ------------------------------------------------------------------------
    p_field : process(clk)
        variable v_t3  : unsigned(17 downto 0);
        variable v_tn  : unsigned(17 downto 0);
        variable v_f, v_o, v_n : unsigned(14 downto 0);
        variable m     : integer range 0 to 14;
        variable v_m3  : signed(21 downto 0);
        variable v_u21, v_ga, v_gb : unsigned(20 downto 0);
        variable v_mn  : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            -- S16: |n| theta (mod one turn)
            if s_na(0) = '0' then v_t3 := s15_th; else v_t3 := (others => '0'); end if;
            case s_na is
                when "00"   => v_tn := (others => '0');
                when "11"   => v_tn := shift_left(s15_th, 2);
                when others => v_tn := shift_left(s15_th, 1);
            end case;
            v_tn := v_t3 + v_tn;
            if s_nneg = '1' then s16_tn <= not v_tn; else s16_tn <= v_tn; end if;
            -- S17
            s17_a <= s_turn + s16_tn;
            -- S18: fold to the first octant; msb of the folded angle
            v_f := s17_a(14 downto 0);
            if s17_a(15) = '1' then v_o := not v_f; else v_o := v_f; end if;
            m := 0;
            for i in 0 to 14 loop
                if v_o(i) = '1' then m := i; end if;
            end loop;
            s18_o <= v_o;
            s18_m <= to_unsigned(m, 4);
            s18_fl <= s17_a(17) & (s17_a(17) xor s17_a(16)) & (s17_a(16) xor s17_a(15));
            -- S19: normalise for the log ; table addresses
            v_n := shift_left(s18_o, 14 - to_integer(s18_m));
            s19_li <= v_n(13 downto 6); s19_lt <= v_n(5 downto 0);
            s19_m  <= s18_m;
            s19_ta <= s18_o(14 downto 7); s19_tt <= s18_o(6 downto 0);
            -- S20, S21 (EBR read, fabric register)
            s20_lt <= s19_lt; s20_m <= s19_m; s20_tt <= s19_tt;
            s21_lt <= s20_lt; s21_m <= s20_m; s21_tt <= s20_tt;
            s21_lv <= l3v_q; s21_ls <= l3s_q; s21_cv <= lcv_q; s21_cs <= lcs_q; s21_sn <= snt_q;
            s21_lr <= unsigned(dlr_q(17 downto 0));
            -- S22: interpolation products ; log2 r - log2 R0
            s22_lp <= resize(unsigned(s21_ls(6 downto 0)) * s21_lt(5 downto 2), 12);
            s22_cp <= resize(unsigned(s21_cs(6 downto 0)) * s21_tt(6 downto 3), 12);
            s22_sp <= unsigned(s21_sn(15 downto 11)) * s21_tt(6 downto 3);
            s22_lv <= unsigned(s21_lv(13 downto 0));
            s22_cv <= unsigned(s21_cv(13 downto 0));
            s22_sv <= unsigned(s21_sn(10 downto 0));
            s22_m  <= s21_m;
            s22_lrn <= signed(resize(s21_lr, 20)) - s_lsub;
            -- S23: log2(o) ; -log2 cos(o) ; -log2(sin(o)/o) ; n (log2 r - log2 R0)
            s23_logo <= shift_left(resize(s22_m, 18), 14) + resize(s22_lv, 18) + resize(shift_right(s22_lp, 4), 18);
            s23_lc <= s22_cv + resize(shift_right(s22_cp, 4), 14);
            s23_sn <= resize(s22_sv & '0', 12) + resize(shift_right(s22_sp, 4), 12);
            -- |n| x = (x or 0) + (0, 2 x or 4 x): one adder of gated operands; the top bit is x's sign
            -- (never a sign bit added to itself: router1 cannot route one net to two pins of a cell)
            if s_na(0) = '0' then v_ga := unsigned(resize(s22_lrn, 21)); else v_ga := (others => '0'); end if;
            case s_na is
                when "00"   => v_gb := (others => '0');
                when "11"   => v_gb := unsigned(s22_lrn(18 downto 0)) & "00";
                when others => v_gb := resize(unsigned(s22_lrn(19 downto 0)) & '0', 21);
            end case;
            v_u21 := v_ga + v_gb;
            v_mn := signed(s22_lrn(19) & v_u21);
            if s_nneg = '1' then s23_mn <= not v_mn; else s23_mn <= v_mn; end if;
            -- S24: the small one = log2 sin(o) ; the large one = log2 cos(o) ; L0 without the bend
            s24_ls  <= signed(resize(s23_logo, 20)) - signed(resize(s23_sn, 20)) + to_signed(C_LSIN0, 20);
            s24_lc  <= s23_lc;
            s24_l0a <= resize(s_cA, 22) + s23_mn;
            -- S25: + the camera bend
            s25_l0 <= s24_l0a + signed(resize(s_cv(18 downto 3), 22));
            s25_ls <= s24_ls; s25_lc <= s24_lc;
            s25_cs <= dlh_q(22);                       -- 1: the cosine is the small one
        end if;
    end process p_field;

    ------------------------------------------------------------------------
    -- S26..S34: the two phases.  Even: 2^L = mantissa (EBR PWL) << integer part.  Log: q L, which
    -- then rides the mantissa path with a zero shift.  16-bit cycle fractions.
    ------------------------------------------------------------------------
    p_phase : process(clk)
        variable v_la, v_lb : signed(21 downto 0);
        variable v_k   : signed(8 downto 0);
        variable v_l20 : unsigned(19 downto 0);
        variable v_pq  : unsigned(23 downto 0);
        variable v_sh  : unsigned(5 downto 0);
        variable v_w48 : unsigned(47 downto 0);
        variable v_w22 : unsigned(21 downto 0);
        variable v_p   : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- S26: L = L0 + log2|cos| (0) or log2|sin| (1): the two sums, then which is which
            v_la := s25_l0 + resize(s25_ls, 22);
            v_lb := s25_l0 - signed(resize(s25_lc, 22));
            for c in 0 to 1 loop
                if (c = 0) = (s25_cs = '1') then s26_l(c) <= v_la; else s26_l(c) <= v_lb; end if;
                -- S27: exp2 address / fraction / shift ; Log: 2 L or 3 L (lines per octave 1 or 1.5, x 2)
                s27_xa(c) <= unsigned(s26_l(c)(13 downto 6));
                s27_t(c)  <= unsigned(s26_l(c)(5 downto 0));
                v_k := resize(shift_right(s26_l(c), 14), 9) + 2;
                if s_log = '1' then v_k := (others => '0');
                elsif v_k < -16 then v_k := to_signed(-16, 9);
                elsif v_k > 16 then v_k := to_signed(16, 9); end if;
                s27_k(c) <= v_k(5 downto 0);
                v_l20 := unsigned(s26_l(c)(19 downto 0));
                if s_qh = '1' then s27_pl(c) <= shift_left(v_l20, 1) + v_l20;
                else               s27_pl(c) <= shift_left(v_l20, 1); end if;
                -- S28 (EBR read) ; Log phase = q L = (2 L or 3 L) * 2^(exponent + 1), a 16-bit cycle fraction
                v_pq := shift_right(s27_pl(c) & "0000", to_integer(s_qsh));
                s28_pl(c) <= v_pq(15 downto 0);
                s28_t(c) <= s27_t(c); s28_k(c) <= s27_k(c);
                -- S29 (fabric register)
                if s_log = '1' then s29_m(c) <= s28_pl(c); s29_s(c) <= (others => '0');
                else                s29_m(c) <= unsigned(exm_q(c)); s29_s(c) <= unsigned(exs_q(c)(7 downto 0)); end if;
                s29_t(c) <= s28_t(c); s29_k(c) <= s28_k(c);
                -- S30
                s30_ep(c) <= resize(s29_s(c) * s29_t(c)(5 downto 2), 14);
                s30_m(c)  <= s29_m(c); s30_k(c) <= s29_k(c);
                -- S31: the mantissa, Q14
                s31_mt(c) <= s30_m(c) + resize(shift_right(s30_ep(c), 4), 16);
                s31_k(c)  <= s30_k(c);
                -- S32, S33: phase = (mantissa << (k + 16)) bits 31 .. 16, in two steps; the sign of the
                -- cosine / sine negates it (one's complement)
                v_sh := unsigned(resize(s31_k(c), 6) + 16);
                v_w48 := shift_left(resize(s31_mt(c), 48), to_integer(v_sh(5 downto 2) & "00"));
                s32_y1(c) <= v_w48(31 downto 13);
                s32_f(c)  <= v_sh(1 downto 0);
                v_w22 := shift_left(resize(s32_y1(c), 22), to_integer(s32_f(c)));
                v_p := v_w22(18 downto 3);
                if s32_sg(c) = '1' and s_log = '0' then s33_p(c) <= not v_p; else s33_p(c) <= v_p; end if;
            end loop;
            -- signs of the cosine (0) and sine (1): stage 30 from the delay line
            s31_sg <= dlh_q(26) & dlh_q(25);
            s32_sg <= s31_sg;
            s33_ng <= s32_sg(0) and not s_log;
            -- S34: + the stream offset / line centring
            if s33_ng = '1' then s34_phi(0) <= s_offrn + s33_p(0); else s34_phi(0) <= s_offre + s33_p(0); end if;
            s34_phi(1) <= s_offim + s33_p(1);
        end if;
    end process p_phase;

    ------------------------------------------------------------------------
    -- the anti-aliasing width (hypnos S30..S34):  log2 w (Q6) = pre - log2 r ; above the knee it is
    -- widened so the filter reaches a full cycle (flat grey) before the lines alias.  Bold: knee at
    -- 1/4 cycle per px, flat at 1/2.  Fine: knee at 1/12 (a 2-px line), flat near 1/5.
    -- Rounded UP to a half octave so the divide is a shift (edges 1 .. 1.4 px).
    ------------------------------------------------------------------------
    p_grad : process(clk)
        variable v_lw  : signed(13 downto 0);
        variable v_t   : signed(11 downto 0);
        variable v_v   : signed(12 downto 0);
        variable v_rr  : signed(13 downto 0);
        variable v_sh  : integer range -8 to 15;
        variable v_w   : unsigned(15 downto 0);
        variable v_m   : unsigned(6 downto 0);
        variable v_l6  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- S26: Even: L0 + log2|n| ; Log: log2(q / ln 2) - log2|cos| (Checker: of the smaller of the two)
            if s_chk = '1' or s25_cs = '1' then v_l6 := s25_ls(19 downto 8);
            else v_l6 := to_signed(0, 12) - signed(resize(s25_lc(13 downto 8), 12)); end if;
            if s_log = '1' then s26_pre <= resize(s_lq6, 14) - resize(v_l6, 14);
            else                s26_pre <= resize(shift_right(s25_l0, 8), 14) + resize(s_c1, 14); end if;
            -- S27: - log2 r (real pixels), from the delay line
            s27_lw <= s26_pre - signed(resize(unsigned(dlr_q(27 downto 18)), 14));
            -- S28
            v_lw := s27_lw;
            if v_lw > 255 then v_lw := to_signed(255, 14);
            elsif v_lw < -1023 then v_lw := to_signed(-1023, 14); end if;
            s28_lg <= v_lw(11 downto 0);
            -- S29
            if s_fine = '1' then
                v_t := s28_lg + 229;
                s29_tk <= s28_lg + 458;
            else
                v_t := s28_lg + 128;
                s29_tk <= to_signed(128, 12);
            end if;
            if v_t > 0 then s29_fl <= '1'; else s29_fl <= '0'; end if;
            s29_lg <= s28_lg;
            -- S30
            if s29_fl = '1' then s30_v <= shift_left(resize(s29_lg, 13), 1) + resize(s29_tk, 13);
            else                 s30_v <= resize(s29_lg, 13); end if;
            -- S31: quantise up to a half octave ; reciprocal 2^-log2 w = 2^ri * {1, sqrt 2}
            v_v := s30_v;
            if v_v < -1023 then v_v := to_signed(-1023, 13); end if;
            v_v := v_v + 31;
            v_v(4 downto 0) := (others => '0');
            if v_v >= 0 then
                s31_sat <= '1'; s31_ri <= (others => '0'); s31_rh <= '0';
            else
                s31_sat <= '0';
                v_rr := -resize(v_v, 14);
                s31_ri <= unsigned(v_rr(9 downto 6));
                s31_rh <= v_rr(5);
            end if;
            -- S32, S33
            s32_ri <= s31_ri; s32_rh <= s31_rh; s32_sat <= s31_sat;
            s33_ri <= s32_ri; s33_rh <= s32_rh; s33_sat <= s32_sat;
            -- S34: half the filter width, w / 2 (Q16): w = {64, 91} << (10 - ri - rh)
            if s33_sat = '1' then
                v_w := (others => '1');
            else
                v_sh := 10 - to_integer(s33_ri);
                if s33_rh = '1' then v_m := to_unsigned(91, 7); v_sh := v_sh - 1;
                else                 v_m := to_unsigned(64, 7); end if;
                if v_sh >= 0 then v_w := resize(shift_left(resize(v_m, 23), v_sh), 16);
                else              v_w := resize(shift_right(resize(v_m, 16), -v_sh), 16); end if;
                if v_w = 0 then v_w := to_unsigned(1, 16); end if;
            end if;
            s34_wh <= v_w(15 downto 1);
            s34_ri <= s33_ri; s34_rh <= s33_rh;
            s35_ri <= s34_ri; s35_rh <= s34_rh;
            s36_ri <= s35_ri; s36_rh <= s35_rh;
            s37_ri <= s36_ri; s37_rh <= s36_rh;
            s38_rh <= s37_rh;
        end if;
    end process p_grad;

    ------------------------------------------------------------------------
    -- S35..S40: one-pixel box filter of the band square wave (hypnos S36..S41).
    --   ink = frac in [0, D) ; P(x) = floor(x) D + min(frac x, D)
    --   coverage = (P(phi + w/2) - P(phi - w/2)) / w
    ------------------------------------------------------------------------
    p_cov : process(clk)
        variable vn  : signed(17 downto 0);
        variable vs  : unsigned(24 downto 0);
        variable vc  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            for c in 0 to 1 loop
                -- S35
                s35_lo(c) <= s34_phi(c) - resize(s34_wh, 16);
                s35_hi(c) <= s34_phi(c) + resize(s34_wh, 16);
                -- S36
                if s35_hi(c) < s35_lo(c) then s36_k(c) <= '1'; else s36_k(c) <= '0'; end if;
                if s35_hi(c) >= s35_lo(c) and s35_hi(c) < s_D16 then s36_full(c) <= '1'; else s36_full(c) <= '0'; end if;
                if s35_hi(c) < s_D16 then s36_mh(c) <= s35_hi(c); else s36_mh(c) <= s_D16; end if;
                if s35_lo(c) < s_D16 then s36_ml(c) <= s35_lo(c); else s36_ml(c) <= s_D16; end if;
                -- S37
                vn := signed(resize(s36_mh(c), 18)) - signed(resize(s36_ml(c), 18));
                if s36_k(c) = '1' then vn := vn + signed(resize(s_D16, 18)); end if;
                if vn < 0 then vn := (others => '0'); end if;
                s37_n(c) <= unsigned(vn(15 downto 0));
                s37_full(c) <= s36_full(c);
                -- S38: (N << ri) >> 7
                vs := shift_left(resize(s37_n(c), 25), to_integer(s37_ri));
                s38_nsh(c) <= vs(16 downto 7);
                s38_full(c) <= s37_full(c);
                -- S39: / 2, or * sqrt2 / 2 on a half-octave step
                if s38_rh = '1' then
                    s39_cp(c) <= resize(shift_right(s38_nsh(c), 1) + shift_right(s38_nsh(c), 3)
                                        + shift_right(s38_nsh(c), 4) + shift_right(s38_nsh(c), 6), 11);
                else
                    s39_cp(c) <= resize(shift_right(s38_nsh(c), 1), 11);
                end if;
                s39_full(c) <= s38_full(c);
                -- S40: coverage 0..256
                vc := resize(s39_cp(c), 10);
                if vc > 256 or s39_full(c) = '1' then vc := to_unsigned(256, 10); end if;
                s40_cov(c) <= resize(vc, 9);
            end loop;
        end if;
    end process p_cov;

    ------------------------------------------------------------------------
    -- S41..S47: weave (Lines: Re ; Checker Bold: XOR = a + b - 2ab ; Checker Fine: OR = a + b - ab),
    -- luma 64 .. 940, then the output conditioning: [1 2 1]/4, luma 13/16 toward
    -- black (white 775), blanking gate.  U = V = 512 always.  The filter's neighbours at the picture
    -- edges are real pixels: the one left of the line (dx = -cx - 1) and the last one held.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_c  : signed(11 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_s  : unsigned(18 downto 0);
        variable v_f  : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S41
            s41_pp <= s40_cov(0)(8 downto 4) * s40_cov(1)(8 downto 4);          -- 0 .. 256 (coverages in 16ths)
            s41_ab <= resize(s40_cov(0)(8 downto 4), 6) + resize(s40_cov(1)(8 downto 4), 6);
            s41_a  <= s40_cov(0);
            -- S42
            v_p := s41_pp;
            if s_fine = '0' then v_p := shift_left(v_p, 1); end if;
            v_c := signed(resize(s41_ab & "0000", 12)) - signed(resize(v_p, 12));
            if s_chk = '0' then s42_c <= s41_a;
            elsif v_c > 256 then s42_c <= to_unsigned(256, 9);
            elsif v_c < 0 then s42_c <= (others => '0');
            else s42_c <= unsigned(v_c(8 downto 0)); end if;
            -- S43: y = 64 + c * 876 / 256   (876 = 1024 - 128 - 16 - 4)
            v_s := shift_left(resize(s42_c, 19), 10) - shift_left(resize(s42_c, 19), 7)
                   - shift_left(resize(s42_c, 19), 4) - shift_left(resize(s42_c, 19), 2);
            s43_y <= to_unsigned(64, 10) + v_s(17 downto 8);
            -- S44, S45: [1 2 1] / 4 around the stage-44 pixel
            s44_y <= s43_y;
            s45_y <= s44_y;
            v_f := resize(s43_y, 12) + shift_left(resize(s44_y, 12), 1) + resize(s45_y, 12) + 2;
            s45_f <= v_f(11 downto 2);
            -- S46: luma 13/16 toward black
            if s45_f > 64 then v_d := s45_f - 64; else v_d := (others => '0'); end if;
            s46_y <= s45_f - shift_right(v_d, 3) - shift_right(v_d, 4);
            -- S47: blanking gate on the aligned generated avid
            if sy(46)(3) = '1' then o_y <= s46_y; else o_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(C_MID);
    data_out.v       <= std_logic_vector(C_MID);
    data_out.avid    <= sy(47)(3);
    data_out.hsync_n <= sy(47)(2);
    data_out.vsync_n <= sy(47)(1);
    data_out.field_n <= sy(47)(0);

end architecture multipole;
