-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- dipole.vhd  (Hardline, v0.1.0) -- lines of force between two charges / a bipolar checkerboard
--
-- Art reference: Faraday's lines of force between two charges, and the bipolar coordinate chart (one
-- family of circles through both poles, a second family of circles round each pole, meeting everywhere
-- at right angles).  Black and white, played by hand.
--
--   poles at the midpoint +- p, p = a e^{it};  per pixel z - p = r1 e^{i th1},  z + p = r2 e^{i th2}
--   Opposite:  w = log((z - p) / (z + p)):  sigma = th1 - th2,  tau = log2 r1 - log2 r2
--   Like:      w = log(z^2 - p^2) - 2it - 2 log a:  sigma = th1 + th2 - 2t,  tau = log2 r1 + log2 r2 - 2 log2 a
--
-- ONE Hypnos log-polar engine at half rate per pole: pixel k computes z - p (k even) or z + p (k odd)
-- exactly, on Q4 sub-pixel coordinates (the pixel counters plus per-field pole offsets a cos t, a sin t
-- from a table-interpolating serial MAC in vblank); the other pole is the average of the two neighbours
-- (its error is only felt within a few px of a pole, where the cells are grey).  No per-pixel square
-- root, divide or wide multiplier.
--
--   sigma phase = N sigma (gated shift-adds, mod one cycle) ; tau phase = N (ln2 / 2pi) tau (CSD + shift-adds)
--   the map is conformal: one gradient for both families, log2 |dw/dz| = log2(2a) - log2(r1 r2)
--   (Opposite), 1 + log2 rho - log2(r1 r2) (Like; rho ~ max + min/2); Hypnos's one-pixel box filter;
--   cells under 2 px fade to flat grey
--   camera: the sample point moves along the diagonal by the box-filtered luma (up to ~120 virtual px at
--   white, K6 100 %), so the whole net swerves round the picture and every edge stays a smooth curve
--
--   K1  X        the midpoint, +-0.75 screen widths (detent at the centre)
--   K2  Y        the midpoint, +-0.75 screen heights (detent)
--   K3  Period   N bands round each pole: 4 5 6 8 10 12 16 20 24 32 (stepped, hysteresis, end exceptions)
--   K4  Turn     the axis angle, a full turn over the knob
--   K5  Drift    bipolar (Hypnos v1.1.0): the tau bands stream out of one pole into the other, the
--                sigma bands creep round at half the rate the other way; centre = still
--   K6  Camera   how far the luma displaces the net (0 = the clean figure)
--   S7  Glide    On: X, Y, Turn, Drift speed, Camera, Gap ease 1/16 per field / Off: none
--   S8  Weave    Lines (sigma: the lines of force) / Checker (sigma XOR tau)
--   S9  Charge   Opposite (source and sink) / Like (two sources: a saddle, Cassini ovals)
--   S10 Weight   Bold (equal bands) / Fine (1/6 lines on the integer levels)
--   S11 Source   Video / Osc (a slow triangle lattice in the camera's place, at 2 x its level)
--   P12 GAP      the separation 2a: 64 .. 2004 virtual px, exponential
--
-- Chassis: hypnos.vhd v1.1.0 / hardline multipole v0.2.0 (timing, generated avid, knob rings, log-polar
-- engine, box-filter AA, output conditioning) with the source field of kinetic riley.  Luma only:
-- U = V = 512 on every pixel.  One latency (40) in every mode.
--
-- Stage numbering: stage N = N registers after s_dx.  o_y is stage 39.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.dipole_pkg.all;

architecture dipole of program_top is
    constant C_PROCESSING_DELAY_CLKS : integer := 40;      -- = processing_delay_clks in the TOML

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_DB    : integer := 14;                 -- X / Y / Drift detent deadband
    constant C_GL    : integer := 4;                  -- glide one-pole shift (1/16 per field)
    constant C_GMIN  : integer := 32;                 -- ... never slower than half a knob count per field

    -- TOML defaults (the holds and glides start there)
    constant C_DEF_K1  : integer := 560;
    constant C_DEF_K2  : integer := 490;
    constant C_DEF_K4  : integer := 569;
    constant C_DEF_K5  : integer := 700;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 560;

    constant ST_OUT : integer := 39;                  -- output register stage
    constant C_XL   : integer := 5;                   -- log2 hi, flags, log2 rho: stage 9 -> EBR output = stage 15
    constant C_XS   : integer := 37;                  -- syncs: EBR output = stage 38

    -- MAC operand codes
    constant A_ZERO : integer := 0;  constant A_KX  : integer := 1;  constant A_KY  : integer := 2;
    constant A_E0   : integer := 3;  constant A_E1  : integer := 4;  constant A_RT  : integer := 5;
    constant A_AQ4  : integer := 6;  constant A_N   : integer := 7;
    constant B_ZERO : integer := 0;  constant B_3WV : integer := 1;  constant B_3H  : integer := 2;
    constant B_F6   : integer := 3;  constant B_POW : integer := 4;  constant B_RT  : integer := 5;
    constant B_T16  : integer := 6;

    attribute no_rw_check : boolean;

    ----------------------------------------------------------------------
    -- field-latched controls + per-field terms
    ----------------------------------------------------------------------
    signal s_init   : std_logic := '1';               -- first field: latch and snap unconditionally
    -- the six knobs K1 K2 K4 K5 K6 P12 as two rings (hold, Q6 glide): one datapath serves all six
    type t_h6 is array(0 to 5) of unsigned(9 downto 0);
    type t_g6 is array(0 to 5) of unsigned(15 downto 0);
    signal r_h : t_h6 := (to_unsigned(C_DEF_K1, 10), to_unsigned(C_DEF_K2, 10), to_unsigned(C_DEF_K4, 10),
                          to_unsigned(C_DEF_K5, 10), to_unsigned(C_DEF_K6, 10), to_unsigned(C_DEF_P12, 10));
    signal r_g : t_g6 := (to_unsigned(C_DEF_K1 * 64, 16), to_unsigned(C_DEF_K2 * 64, 16), to_unsigned(C_DEF_K4 * 64, 16),
                          to_unsigned(C_DEF_K5 * 64, 16), to_unsigned(C_DEF_K6 * 64, 16), to_unsigned(C_DEF_P12 * 64, 16));
    signal s_ri     : unsigned(2 downto 0) := (others => '0');
    signal s_rp     : unsigned(1 downto 0) := (others => '0');
    signal s_rh     : unsigned(9 downto 0) := (others => '0');
    signal s_rt     : unsigned(15 downto 0) := (others => '0');
    signal s_kd     : signed(16 downto 0) := (others => '0');
    signal s_zone   : unsigned(3 downto 0) := to_unsigned(5, 4);    -- K3 Period step (N = 12)
    signal s_nj     : unsigned(1 downto 0) := "01";                -- N = 2^(nj + 2) (+ 2^nk if s_n2)
    signal s_nk     : unsigned(1 downto 0) := "10";
    signal s_n2     : std_logic := '1';
    signal s_glide  : std_logic := '1';               -- S7
    signal s_chk    : std_logic := '1';               -- S8  0 Lines / 1 Checker
    signal s_like   : std_logic := '0';               -- S9  0 Opposite / 1 Like
    signal s_fine   : std_logic := '0';               -- S10 0 Bold / 1 Fine
    signal s_src    : std_logic := '0';               -- S11 0 Video / 1 Osc

    signal s_N      : unsigned(5 downto 0) := to_unsigned(12, 6);
    signal s_lgn    : signed(8 downto 0) := (others => '0');           -- log2(N / 2 pi), Q6
    signal s_cx, s_cy : signed(13 downto 0) := (others => '0');
    signal s_La     : unsigned(17 downto 0) := to_unsigned(98304, 18); -- log2 a (real px), Q14
    signal s_aq4    : unsigned(14 downto 0) := (others => '0');        -- a, Q4
    signal s_axq, s_ayq : signed(20 downto 0) := (others => '0');      -- a cos t, a sin t, Q8
    signal s_kx     : signed(17 downto 0) := (others => '0');          -- a cos t, Q4
    signal s_ang    : unsigned(15 downto 0) := (others => '0');
    signal s_comp   : unsigned(15 downto 0) := (others => '0');        -- Like: -2N t (sigma frame)
    signal s_Cg     : signed(19 downto 0) := (others => '0');          -- log2(N / 2pi) + 1 (+ log2 a), Q14
    signal s_cl     : unsigned(18 downto 0) := (others => '0');        -- 2 log2 a, Q14
    signal s_3wv    : unsigned(12 downto 0) := to_unsigned(5760, 13);
    signal s_3h     : unsigned(12 downto 0) := to_unsigned(3240, 13);
    signal s_gain   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal s_D16    : unsigned(15 downto 0) := to_unsigned(32768, 16); -- band duty, Q16
    signal s_offs, s_offt : unsigned(15 downto 0) := (others => '0');
    -- DRIFT: decode, serial shifter, the two phase accumulators (Q20 cycles)
    signal s_vneg, s_vzero : std_logic := '0';
    signal s_av, s_u, s_ls : unsigned(9 downto 0) := (others => '0');
    signal s_spd    : unsigned(19 downto 0) := (others => '0');
    signal s_ssh    : unsigned(3 downto 0) := (others => '0');
    signal s_anit, s_anis : unsigned(19 downto 0) := (others => '0');

    -- vblank sequencer, table sub-sequencer
    signal s_seq   : unsigned(6 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse, s_vs_d : std_logic := '0';
    signal s_saw_avid   : std_logic := '0';
    signal tr_st   : unsigned(3 downto 0) := (others => '0');
    signal tr_mode : std_logic := '0';                                 -- 0: 2^x region, 1: sine region
    signal s_ti    : unsigned(8 downto 0) := (others => '0');
    signal s_f6    : unsigned(5 downto 0) := (others => '0');
    signal s_tneg, s_hi0, s_hi1 : std_logic := '0';
    signal s_ta    : unsigned(8 downto 0) := (others => '0');
    signal tab_q, s_ef : std_logic_vector(15 downto 0) := (others => '0');
    signal s_e0, s_e1 : unsigned(15 downto 0) := (others => '0');
    signal s_rtv   : signed(17 downto 0) := (others => '0');

    -- the serial MAC: acc (+)= a * b, b two's complement, 17 steps
    signal mc_go   : std_logic := '0';
    signal mc_ac   : unsigned(2 downto 0) := (others => '0');
    signal mc_bc   : unsigned(2 downto 0) := (others => '0');
    signal mc_pz   : std_logic := '0';                                 -- 1: start from zero, 0: accumulate
    signal mc_ng   : std_logic := '0';
    signal mc_pw   : unsigned(2 downto 0) := (others => '0');
    signal mc_neg  : std_logic := '0';
    signal mc_as   : signed(31 downto 0) := (others => '0');
    signal mc_b    : signed(16 downto 0) := (others => '0');
    signal mc_n    : unsigned(4 downto 0) := (others => '0');
    signal mc_acc  : signed(31 downto 0) := (others => '0');

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
    signal g_go    : std_logic;

    signal s_dx       : signed(14 downto 0) := (others => '0');
    signal s_dyq      : signed(22 downto 0) := (others => '0');   -- dy, Q8 x-pixel units
    signal s_par, s_first : std_logic := '0';                    -- pixel k odd ; k = 0 (pixel -1)
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- EBR delay lines (one write, one registered read each)
    ----------------------------------------------------------------------
    type t_dl32 is array(0 to 255) of std_logic_vector(31 downto 0);
    type t_dl16 is array(0 to 255) of std_logic_vector(15 downto 0);
    signal dll  : t_dl32 := (others => (others => '0'));      -- log2 hi, flags, log2 rho (stage 9)
    signal dls  : t_dl16 := (others => (others => '0'));      -- generated avid + syncs
    attribute no_rw_check of dll : signal is true;
    attribute no_rw_check of dls : signal is true;
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal dll_q : std_logic_vector(31 downto 0) := (others => '0');
    signal dll_d : std_logic_vector(31 downto 0);
    signal dls_q : std_logic_vector(15 downto 0) := x"0006";
    signal dls_d : std_logic_vector(15 downto 0);
    type t_sy is array(38 to 39) of std_logic_vector(3 downto 0);
    signal sy : t_sy := (others => "0110");

    ----------------------------------------------------------------------
    -- [D] source field
    ----------------------------------------------------------------------
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot    : unsigned(7 downto 0) := (others => '0');
    signal s_oxs   : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys   : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum  : unsigned(7 downto 0) := (others => '0');
    signal wt_ram  : t_dl16 := (others => (others => '0'));
    attribute no_rw_check of wt_ram : signal is true;
    signal wt_ra   : unsigned(7 downto 0);
    signal wt_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal wt_d    : std_logic_vector(15 downto 0);
    signal wt_i    : unsigned(7 downto 0) := (others => '0');
    signal wt_acc  : unsigned(16 downto 0) := (others => '0');
    signal wt_run  : std_logic := '0';
    signal s_sl, s_sfirst : unsigned(10 downto 0) := (others => '0');
    signal s_sv0, s_sv, s_svd : std_logic := '0';
    signal s_col, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal bx_ram  : t_dl16 := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal bx_d    : std_logic_vector(15 downto 0);
    signal s_bcnt  : unsigned(6 downto 0) := (others => '0');
    signal s_bsum  : unsigned(16 downto 0) := (others => '0');
    signal s_eb    : unsigned(15 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal lb_ram  : t_dl16 := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal s_eq, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa   : unsigned(7 downto 0) := (others => '0');
    signal lb_we, s_efirst : std_logic := '0';
    signal s_cprev : unsigned(15 downto 0) := (others => '0');
    signal s_cd    : signed(16 downto 0) := (others => '0');
    signal s_cv    : unsigned(18 downto 0) := (others => '0');          -- the camera, 19 bits
    signal s_cn    : unsigned(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pixel pipeline: one engine, poles alternating per pixel
    ----------------------------------------------------------------------
    type t_q is array(3 to 9) of std_logic_vector(5 downto 0);     -- sw, x < 0, y < 0, zero, odd, first
    -- S1..S9: Q4 coordinates -> |x|, |y| -> hi / lo -> log2 hi, log2 lo (Q14)
    signal s1_x    : signed(17 downto 0) := (others => '0');
    signal s1_y    : signed(18 downto 0) := (others => '0');
    signal s1_pf   : std_logic_vector(1 downto 0) := "00";
    signal s2_a, s2_b : unsigned(16 downto 0) := (others => '0');
    signal s2_xs, s2_ys : std_logic := '0';
    signal s2_pf   : std_logic_vector(1 downto 0) := "00";
    signal s3_hi, s3_lo, s4_hi, s4_lo : unsigned(16 downto 0) := (others => '0');
    signal s4_mh, s4_ml : unsigned(4 downto 0) := (others => '0');
    signal s5_ih, s5_il : unsigned(7 downto 0) := (others => '0');
    signal s5_th, s5_tl, s6_th, s6_tl, s7_th, s7_tl : unsigned(3 downto 0) := (others => '0');
    signal s5_mh, s5_ml, s6_mh, s6_ml, s7_mh, s7_ml, s8_mh, s8_ml : unsigned(4 downto 0) := (others => '0');
    signal lvh_q, lsh_q, lvl_q, lsl_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s7_lvh, s7_lsh, s7_lvl, s7_lsl : std_logic_vector(15 downto 0) := (others => '0');
    signal s8_lph, s8_lpl : unsigned(10 downto 0) := (others => '0');
    signal s8_lvh, s8_lvl : unsigned(13 downto 0) := (others => '0');
    signal s9_lh, s9_ll : unsigned(18 downto 0) := (others => '0');
    signal qsr : t_q := (others => (others => '0'));
    -- Like: log2 rho ~ log2(max + min/2), msb + 4-bit mantissa (S1..S4), carried to S9
    signal s1_da, s1_db : unsigned(13 downto 0) := (others => '0');
    signal s2_rs   : unsigned(13 downto 0) := (others => '0');
    signal s3_rs   : unsigned(13 downto 0) := (others => '0');
    signal s3_rm   : unsigned(3 downto 0) := (others => '0');
    type t_lr is array(4 to 9) of std_logic_vector(7 downto 0);
    signal lrr : t_lr := (others => (others => '0'));

    -- S10..S16: u = log2(hi/lo) -> log2 r, angle
    signal s10_ua  : unsigned(17 downto 0) := (others => '0');
    signal s11_ia  : unsigned(7 downto 0) := (others => '0');
    signal s11_tu, s12_tu, s13_tu : unsigned(7 downto 0) := (others => '0');
    signal ug_q, ugs_q, ua_q, uas_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s13_g, s13_gs, s13_a, s13_as : std_logic_vector(15 downto 0) := (others => '0');
    signal s14_gp, s14_ap : signed(21 downto 0) := (others => '0');
    signal s14_g, s14_a : unsigned(15 downto 0) := (others => '0');
    signal s15_g   : unsigned(13 downto 0) := (others => '0');
    signal s15_a   : unsigned(15 downto 0) := (others => '0');
    signal s16_l, s17_l, s18_l : unsigned(18 downto 0) := (others => '0');     -- log2 r (Q4 length), Q14
    signal s16_th, s17_th, s18_th : unsigned(17 downto 0) := (others => '0');  -- the angle, 2^18 per turn
    signal s16_lr, s17_lr, s18_lr : std_logic_vector(7 downto 0) := (others => '0');
    signal s16_pf, s17_pf : std_logic_vector(1 downto 0) := "00";              -- odd, first

    -- S18..S26: the two poles (Q15 / 2^19 per turn), sigma, tau, the gradient, the two phases (Q16)
    signal s18_l1, s18_l2 : unsigned(19 downto 0) := (others => '0');
    signal s18_t1, s18_t2 : unsigned(18 downto 0) := (others => '0');
    signal s19_sig : unsigned(17 downto 0) := (others => '0');
    signal s19_sl  : unsigned(19 downto 0) := (others => '0');
    signal s19_dl  : signed(20 downto 0) := (others => '0');
    signal s19_cg  : signed(20 downto 0) := (others => '0');
    signal s20_tau : signed(20 downto 0) := (others => '0');
    signal s20_lgw : signed(20 downto 0) := (others => '0');
    signal s20_q0 : unsigned(17 downto 0) := (others => '0');
    signal s21_ps  : unsigned(15 downto 0) := (others => '0');
    signal s21_x1, s21_x2 : signed(22 downto 0) := (others => '0');
    signal s21_lw  : signed(13 downto 0) := (others => '0');
    type t_u16a is array(22 to 25) of unsigned(15 downto 0);
    signal sps : t_u16a := (others => (others => '0'));             -- sigma phase + offset, stages 22..25
    signal s22_t1  : unsigned(17 downto 0) := (others => '0');
    signal s23_q0 : unsigned(17 downto 0) := (others => '0');
    signal s24_pt, s25_pt : unsigned(15 downto 0) := (others => '0');
    type t_u16x2 is array(0 to 1) of unsigned(15 downto 0);
    type t_u10x2 is array(0 to 1) of unsigned(9 downto 0);
    type t_u11x2 is array(0 to 1) of unsigned(10 downto 0);
    type t_u9x2  is array(0 to 1) of unsigned(8 downto 0);
    type t_bx2   is array(0 to 1) of std_logic;
    signal s26_phi : t_u16x2 := (others => (others => '0'));

    -- S22..S26: the anti-aliasing width
    signal s22_lg, s23_lg, s23_tk : signed(11 downto 0) := (others => '0');
    signal s23_fl  : std_logic := '0';
    signal s24_v   : signed(12 downto 0) := (others => '0');
    signal s25_ri, s26_ri, s27_ri, s28_ri, s29_ri : unsigned(3 downto 0) := (others => '0');
    signal s25_rh, s26_rh, s27_rh, s28_rh, s29_rh, s30_rh : std_logic := '0';
    signal s25_sat : std_logic := '0';
    signal s26_wh  : unsigned(14 downto 0) := (others => '1');

    -- S27..S32: one-pixel box filter -> coverage 0..256 (0 = sigma, 1 = tau)
    signal s27_lo, s27_hi, s28_mh, s28_ml, s29_n : t_u16x2 := (others => (others => '0'));
    signal s28_k, s28_full, s29_full, s30_full, s31_full : t_bx2 := (others => '0');
    signal s30_nsh : t_u10x2 := (others => (others => '0'));
    signal s31_cp  : t_u11x2 := (others => (others => '0'));
    signal s32_cov : t_u9x2 := (others => (others => '0'));

    -- S33..S39: weave, luma, output conditioning
    signal s33_pp  : unsigned(9 downto 0) := (others => '0');
    signal s33_ab  : unsigned(5 downto 0) := (others => '0');
    signal s33_a   : unsigned(8 downto 0) := (others => '0');
    signal s34_c   : unsigned(8 downto 0) := (others => '0');
    signal s35_y, s36_y, s37_y, s37_f, s38_y : unsigned(9 downto 0) := to_unsigned(64, 10);
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

    -- Exact signed product a * m, radix-4 Booth on m (hypnos)
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

    -- x * N mod 2^18 for N = 2^(j+2) (+ 2^k when two): one adder of two selected shifts
    function nmul(x : unsigned(17 downto 0); j, k : unsigned(1 downto 0); two : std_logic) return unsigned is
        variable va, vb : unsigned(17 downto 0);
    begin
        case to_integer(j) is
            when 0      => va := shift_left(x, 2);
            when 1      => va := shift_left(x, 3);
            when 2      => va := shift_left(x, 4);
            when others => va := shift_left(x, 5);
        end case;
        if two = '0' then vb := (others => '0');
        else
            case to_integer(k) is
                when 0      => vb := x;
                when 1      => vb := shift_left(x, 1);
                when 2      => vb := shift_left(x, 2);
                when others => vb := shift_left(x, 3);
            end case;
        end if;
        return va + vb;
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
            -- per-class: Osc x step, 3 x the X travel width, log2 of the camera displacement (Q6)
            case s_mcls is
                when "10"   => s_oxs <= to_unsigned(204, 9); s_3wv <= to_unsigned(2880, 13);
                when "01"   => s_oxs <= to_unsigned(153, 9); s_3wv <= to_unsigned(3840, 13);
                when others => s_oxs <= to_unsigned(102, 9); s_3wv <= to_unsigned(5760, 13);
            end case;
            s_3h <= resize(s_H, 13) + resize(s_H & '0', 13);

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
    -- per-field control latch + vblank sequencer (one small op per state; a state that needs the MAC
    -- or the table sub-sequencer waits for them) and the table sub-sequencer.
    -- The field tick is the first vsync EDGE after active video (analog vsync serrates).
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_raw  : unsigned(9 downto 0);
        variable v_e    : signed(10 downto 0);
        variable v_t    : unsigned(15 downto 0);
        variable v_st   : signed(15 downto 0);
        variable v_gn   : unsigned(15 downto 0);
        variable v_r    : unsigned(9 downto 0);
        variable v_z    : unsigned(13 downto 0);
        variable v_u11  : unsigned(10 downto 0);
        variable v_m7   : unsigned(6 downto 0);
        variable v_idle : boolean;
        variable v_rr   : unsigned(14 downto 0);
        variable v_ix   : unsigned(8 downto 0);
        variable v_La   : unsigned(17 downto 0);
        variable v_cg   : signed(19 downto 0);

        procedure mac(a : integer; b : integer; pz : std_logic; ng : std_logic) is
        begin
            mc_go <= '1';
            mc_ac <= to_unsigned(a, 3);
            mc_bc <= to_unsigned(b, 3);
            mc_pz <= pz;
            mc_ng <= ng;
        end procedure;
        constant P_ZERO : std_logic := '1';
        constant P_KEEP : std_logic := '0';
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            s_vs_d       <= s_vs_pulse;
            mc_go <= '0';
            if data_in.avid = '1' then s_saw_avid <= '1'; end if;
            v_idle := mc_go = '0' and mc_n = 0;

            -- table ROM: registered address and read, fabric copy
            tab_q <= C_TAB(to_integer(s_ta));
            s_ef  <= tab_q;

            -- DRIFT mantissa << (octave + 2), one bit per clock (Hypnos); loaded by the sequencer
            if s_ssh /= 0 then
                s_spd <= s_spd(18 downto 0) & s_vneg;
                s_ssh <= s_ssh - 1;
            end if;

            ---------------- table sub-sequencer: x = T[i] + (T[i+1] - T[i]) f / 64 (three MACs)
            if tr_st /= 0 and v_idle then
                case to_integer(tr_st) is
                    when 1 =>
                        if tr_mode = '0' then
                            v_ix := '0' & s_La(13 downto 6);
                            s_f6 <= s_La(5 downto 0);
                            s_tneg <= '0';
                        else
                            if s_ang(14) = '1' then v_rr := to_unsigned(16384, 15) - resize(s_ang(13 downto 0), 15);
                            else v_rr := resize(s_ang(13 downto 0), 15); end if;
                            v_ix := v_rr(14 downto 6);
                            s_f6 <= v_rr(5 downto 0);
                            s_tneg <= s_ang(15);
                        end if;
                        s_ti  <= v_ix;
                        s_hi0 <= v_ix(8);
                        if v_ix(8) = '1' or v_ix(7 downto 0) = 255 then s_hi1 <= '1'; else s_hi1 <= '0'; end if;
                        s_ta  <= tr_mode & v_ix(7 downto 0);
                        tr_st <= tr_st + 1;
                    when 2 =>
                        s_ta  <= tr_mode & (s_ti(7 downto 0) + 1);
                        tr_st <= tr_st + 1;
                    when 3 =>
                        tr_st <= tr_st + 1;
                    when 4 =>
                        if s_hi0 = '1' then
                            if tr_mode = '1' then s_e0 <= to_unsigned(32767, 16); else s_e0 <= to_unsigned(32768, 16); end if;
                        else s_e0 <= unsigned(s_ef); end if;
                        tr_st <= tr_st + 1;
                    when 5 =>
                        if s_hi1 = '1' then
                            if tr_mode = '1' then s_e1 <= to_unsigned(32767, 16); else s_e1 <= to_unsigned(32768, 16); end if;
                        else s_e1 <= unsigned(s_ef); end if;
                        mc_pw <= to_unsigned(6, 3);
                        mac(A_E0, B_POW, P_ZERO, s_tneg);
                        tr_st <= tr_st + 1;
                    when 6 =>
                        mac(A_E1, B_F6, P_KEEP, s_tneg);
                        tr_st <= tr_st + 1;
                    when 7 =>
                        mac(A_E0, B_F6, P_KEEP, not s_tneg);
                        tr_st <= tr_st + 1;
                    when others =>
                        s_rtv <= mc_acc(23 downto 6);
                        tr_st <= (others => '0');
                end case;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_saw_avid = '1' then
                s_vs_pulse <= '1';
                s_saw_avid <= '0';

                -- K3 Period: ten steps (K3 * 10 >> 10); a new step is taken about 8 counts inside it, or at
                -- either end of the knob
                v_r := unsigned(registers_in(2));
                v_z := shift_left(resize(v_r, 14), 3) + shift_left(resize(v_r, 14), 1);
                if s_init = '1' or (v_z(9 downto 0) >= 80 and v_z(9 downto 0) <= 943)
                   or (v_r(9 downto 3) = "0000000") or (v_r(9 downto 3) = "1111111") then
                    s_zone <= v_z(13 downto 10);
                end if;
                s_glide <= registers_in(6)(0);
                s_chk   <= registers_in(6)(1);
                s_like  <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_Wl    <= s_W;
                s_seq   <= to_unsigned(127, 7);
                s_ri    <= (others => '0');
                s_rp    <= (others => '0');

            elsif s_seq /= 0 and data_in.avid = '0' and v_idle and tr_st = 0 then
                s_seq <= s_seq - 1;

                -- KNOB RINGS (states 125 .. 108, three per knob): +-1-count hold against pot jitter, then
                -- the glide: 1/16 of the gap per field, never slower than half a count, landing exactly.
                -- Glide off (or the first field): the held value itself, the same field.
                if s_seq <= 125 and s_seq >= 108 then
                    case s_rp is
                        when "00" =>
                            case to_integer(s_ri) is
                                when 0      => v_raw := unsigned(registers_in(0));
                                when 1      => v_raw := unsigned(registers_in(1));
                                when 2      => v_raw := unsigned(registers_in(3));
                                when 3      => v_raw := unsigned(registers_in(4));
                                when 4      => v_raw := unsigned(registers_in(5));
                                when others => v_raw := unsigned(registers_in(7));
                            end case;
                            v_e := signed('0' & v_raw) - signed('0' & r_h(0));
                            if s_init = '1' or (v_e /= 0 and v_e /= 1 and v_e /= -1) then s_rh <= v_raw; else s_rh <= r_h(0); end if;
                            s_rp <= "01";
                        when "01" =>
                            -- X, Y and Drift: a detent at the centre
                            if (s_ri(2 downto 1) = "00" or s_ri = 3) and s_rh > 512 - C_DB and s_rh < 512 + C_DB then v_t := to_unsigned(32768, 16);
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
                case to_integer(s_seq) is
                    when 127 =>
                        -- N = 4 5 6 8 10 12 16 20 24 32 = 2^(nj+2) (+ 2^nk)
                        case to_integer(s_zone) is
                            when 0      => s_N <= to_unsigned(4, 6);  s_nj <= "00"; s_nk <= "00"; s_n2 <= '0';
                            when 1      => s_N <= to_unsigned(5, 6);  s_nj <= "00"; s_nk <= "00"; s_n2 <= '1';
                            when 2      => s_N <= to_unsigned(6, 6);  s_nj <= "00"; s_nk <= "01"; s_n2 <= '1';
                            when 3      => s_N <= to_unsigned(8, 6);  s_nj <= "01"; s_nk <= "00"; s_n2 <= '0';
                            when 4      => s_N <= to_unsigned(10, 6); s_nj <= "01"; s_nk <= "01"; s_n2 <= '1';
                            when 5      => s_N <= to_unsigned(12, 6); s_nj <= "01"; s_nk <= "10"; s_n2 <= '1';
                            when 6      => s_N <= to_unsigned(16, 6); s_nj <= "10"; s_nk <= "00"; s_n2 <= '0';
                            when 7      => s_N <= to_unsigned(20, 6); s_nj <= "10"; s_nk <= "10"; s_n2 <= '1';
                            when 8      => s_N <= to_unsigned(24, 6); s_nj <= "10"; s_nk <= "11"; s_n2 <= '1';
                            when others => s_N <= to_unsigned(32, 6); s_nj <= "11"; s_nk <= "00"; s_n2 <= '0';
                        end case;
                    when 126 => s_lgn <= to_signed(C_LGN6(to_integer(s_zone)), 9);

                    -- the midpoint: cx = W/2 + (K1 - 512) Wv 3/2048, cy = H/2 + (K2 - 512) H 3/2048
                    when 107 => mac(A_KX, B_3WV, P_ZERO, '0');
                    when 106 =>
                        s_cx <= signed(resize(s_W(11 downto 1), 14)) + resize(shift_right(mc_acc, 14), 14);
                        mac(A_KY, B_3H, P_ZERO, '0');
                    when 105 =>
                        s_cy <= signed(resize(s_H(11 downto 1), 14)) + resize(shift_right(mc_acc, 14), 14);
                        -- GAP: log2 a = 6 + P12 4.966 / 1023 - log2(x step)  (Q14; a = 64 .. 2004 virtual px)
                        v_La := to_unsigned(98304, 18) + resize(r_g(5), 18) + resize(r_g(5)(15 downto 2), 18)
                                - resize(r_g(5)(15 downto 7), 18);
                        case s_mcls is
                            when "10"   => s_La <= v_La - 16384;
                            when "01"   => s_La <= v_La - 9585;
                            when others => s_La <= v_La;
                        end case;
                    when 104 =>
                        tr_mode <= '0'; tr_st <= "0001";               -- mantissa of a = 2^frac
                    when 103 =>
                        mc_pw <= resize(s_La(16 downto 14) - 5, 3);
                        mac(A_RT, B_POW, P_ZERO, '0');                 -- a = (mantissa << (int - 5)) >> 5, Q4
                    when 102 =>
                        s_aq4 <= unsigned(mc_acc(19 downto 5));
                        s_ang <= (r_g(2) xor x"8000") + 16384;          -- cos t
                        tr_mode <= '1'; tr_st <= "0001";
                    when 101 => mac(A_AQ4, B_RT, P_ZERO, '0');
                    when 100 =>
                        s_axq <= mc_acc(31 downto 11);                   -- a cos t, Q8
                        s_ang <= r_g(2) xor x"8000";                    -- sin t
                        tr_st <= "0001";
                    when 99 => mac(A_AQ4, B_RT, P_ZERO, '0');
                    when 98 =>
                        s_ayq <= mc_acc(31 downto 11);                   -- a sin t, Q8
                        mac(A_N, B_T16, P_ZERO, '0');
                    when 97 =>
                        -- Like: the sigma frame turns with the axis, -2N t cycles
                        if s_like = '1' then s_comp <= to_unsigned(0, 16) - unsigned(mc_acc(14 downto 0) & '0');
                        else s_comp <= (others => '0'); end if;
                        -- the pole offset, Q4: x1 = dx - a cos t, x2 = dx + a cos t
                        s_kx <= resize(shift_right(s_axq, 4), 18);
                        -- gradient constant and the Like tau reference (Q14; the logs are of Q4 lengths, +8 octaves)
                        v_cg := shift_left(resize(s_lgn, 20), 8) + 16384 + 131072;
                        if s_like = '0' then v_cg := v_cg + signed(resize(s_La, 20)); end if;
                        s_Cg <= v_cg;
                        s_cl <= (s_La & '0') + 131072;
                        -- K6 gain for the camera displacement in real px: 1080 x 1, 720p x 0.656, SD x 0.5
                        case s_mcls is
                            when "10"   => s_gain <= "0" & r_g(4)(15 downto 9);
                            when "01"   => s_gain <= ("0" & r_g(4)(15 downto 9)) + ("000" & r_g(4)(15 downto 11)) + ("00000" & r_g(4)(15 downto 13));
                            when others => s_gain <= r_g(4)(15 downto 8);
                        end case;
                        if s_fine = '1' then s_D16 <= to_unsigned(10923, 16); else s_D16 <= to_unsigned(32768, 16); end if;

                    -- DRIFT (K5, Hypnos v1.1.0 shape): bipolar, |v| < 24 counts = still; exponential,
                    -- 2^-12 .. 2^-2 cycle per field (piecewise-linear mantissa 1 + f/64, octave = ls / 64);
                    -- the glided K5 (the knob ring) is the speed, never the phase
                    when 86 =>
                        s_vneg <= not r_g(3)(15);
                        if r_g(3)(15) = '1' then s_av <= '0' & r_g(3)(14 downto 6);
                        else                     s_av <= '0' & not r_g(3)(14 downto 6); end if;
                    when 85 =>
                        v_u11 := resize(s_av, 11) - 24;
                        s_vzero <= v_u11(10);
                        s_u <= v_u11(9 downto 0);
                    when 84 =>
                        s_ls <= s_u + shift_right(s_u, 2) + shift_right(s_u, 4);
                    when 83 =>
                        if s_vzero = '1' then s_spd <= (others => '0'); s_ssh <= (others => '0');
                        else
                            v_m7 := '1' & s_ls(5 downto 0);
                            if s_vneg = '1' then v_m7 := not v_m7; end if;
                            s_spd <= (19 downto 9 => s_vneg) & v_m7 & s_vneg & s_vneg;
                            s_ssh <= s_ls(9 downto 6);
                        end if;
                    -- the two phase accumulators: tau travels at the step, sigma at half of it the other
                    -- way; one cycle = 2^20, they wrap forever without a seam
                    when 70 =>
                        s_anit <= s_anit + s_spd;
                        s_anis <= s_anis - unsigned(shift_right(signed(s_spd), 1));
                    when 69 =>
                        if s_fine = '1' then
                            s_offs <= s_anis(19 downto 4) + s_comp + 5461;
                            s_offt <= s_anit(19 downto 4) + 5461;
                        else
                            s_offs <= s_anis(19 downto 4) + s_comp;
                            s_offt <= s_anit(19 downto 4);
                        end if;
                    when 68 => s_init <= '0';
                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- the serial MAC: on go, acc = 0 (or kept), then 17 steps of acc += a 2^i for each set bit i of b
    -- (the sign bit subtracts); ng negates the product (the add / subtract sense flips)
    ------------------------------------------------------------------------
    p_mac : process(clk)
        variable va  : signed(17 downto 0);
        variable vb  : signed(16 downto 0);
        variable vs  : std_logic;
        variable vx  : signed(33 downto 0);
    begin
        if rising_edge(clk) then
            if mc_go = '1' then
                case to_integer(mc_ac) is
                    when A_KX   => va := resize(signed((not r_g(0)(15)) & r_g(0)(14 downto 3)), 18);
                    when A_KY   => va := resize(signed((not r_g(1)(15)) & r_g(1)(14 downto 3)), 18);
                    when A_E0   => va := signed(resize(s_e0, 18));
                    when A_E1   => va := signed(resize(s_e1, 18));
                    when A_RT   => va := s_rtv;
                    when A_AQ4  => va := signed(resize(s_aq4, 18));
                    when A_N    => va := signed(resize(s_N, 18));
                    when others => va := (others => '0');
                end case;
                case to_integer(mc_bc) is
                    when B_3WV  => vb := signed(resize(s_3wv, 17));
                    when B_3H   => vb := signed(resize(s_3h, 17));
                    when B_F6   => vb := signed(resize(s_f6, 17));
                    when B_POW  => vb := (others => '0'); vb(to_integer(mc_pw)) := '1';
                    when B_RT   => vb := resize(s_rtv, 17);
                    when B_T16  => vb := signed(resize(r_g(2) xor x"8000", 17));
                    when others => vb := (others => '0');
                end case;
                mc_as  <= resize(va, 32);
                mc_b   <= vb;
                mc_neg <= mc_ng;
                mc_n   <= to_unsigned(17, 5);
                if mc_pz = '1' then mc_acc <= (others => '0'); end if;
            elsif mc_n /= 0 then
                if mc_b(0) = '1' then
                    -- subtract on the sign step, and when the product is negated (one carry chain)
                    if mc_n = 1 then vs := not mc_neg; else vs := mc_neg; end if;
                    if vs = '1' then vx := (mc_acc(31) & mc_acc & '1') + (not (mc_as(31) & mc_as) & '1');
                    else             vx := (mc_acc(31) & mc_acc & '1') + ((mc_as(31) & mc_as) & '0'); end if;
                    mc_acc <= vx(32 downto 1);
                end if;
                mc_as <= shift_left(mc_as, 1);
                mc_b  <= shift_right(mc_b, 1);
                mc_n  <= mc_n - 1;
            end if;
        end if;
    end process p_mac;

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
            -- the engine alternates poles every pixel: k = 0 (pixel -1) is z - p
            if g_go = '1' then s_par <= '0'; s_first <= '1';
            else               s_par <= not s_par; s_first <= '0'; end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- EBR delay lines.  Write address = free-running counter; each read runs X behind it (never the
    -- written address).
    ------------------------------------------------------------------------
    dll_d <= lrr(9) & qsr(9)(1 downto 0) & qsr(9)(5 downto 3) & std_logic_vector(s9_lh);
    dls_d <= x"000" & g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_wa <= s_wa + 1;
            dll(to_integer(s_wa)) <= dll_d;
            dll_q <= dll(to_integer(s_wa - C_XL));
            dls(to_integer(s_wa)) <= dls_d;
            dls_q <= dls(to_integer(s_wa - C_XS));
            sy(38) <= dls_q(3 downto 0);
            sy(39) <= sy(38);
        end if;
    end process p_dl;

    -- table ROMs (EBR): every address is a register, every read is registered
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lvh_q <= C_LOGV(to_integer(s5_ih));
            lsh_q <= C_LOGS(to_integer(s5_ih));
            lvl_q <= C_LOGV(to_integer(s5_il));
            lsl_q <= C_LOGS(to_integer(s5_il));
            ug_q  <= C_UG(to_integer(s11_ia));
            ugs_q <= C_UGS(to_integer(s11_ia));
            ua_q  <= C_UA(to_integer(s11_ia));
            uas_q <= C_UAS(to_integer(s11_ia));
        end if;
    end process p_rom;

    ------------------------------------------------------------------------
    -- [D] SOURCE FIELD (kinetic riley): luma or the Osc lattice -> x K6 (a table refilled every field:
    -- (level - 16) * gain >> 6, Osc level * 2 gain >> 6) -> 64-px box (halved), edge-replicated and reloaded
    -- at every line start -> vertical 1/16 IIR on every 8th column -> linear interpolation back to pixel
    -- rate.  s_cv is tapped unaligned (the camera is never shown).
    ------------------------------------------------------------------------
    bx_d  <= "00000" & std_logic_vector(s_sl);
    wt_d  <= "00000" & std_logic_vector(wt_acc(16 downto 6));
    wt_ra <= s_osum when s_src = '1' else unsigned(data_in.y(9 downto 2));
    p_src : process(clk)
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(10 downto 0);
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

            -- gain table fill (vblank, 256 clocks): entry i = (max(i - 16, 0) * gain) >> 6, by
            -- accumulation; Osc: (i * 2 gain) >> 6
            if s_src = '1' then v_g := s_gain & '0'; else v_g := resize(s_gain, 9); end if;
            if s_seq = 87 then
                wt_run <= '1'; wt_i <= (others => '0'); wt_acc <= (others => '0');
            elsif wt_run = '1' then
                if s_src = '1' or wt_i(7 downto 4) /= "0000" then wt_acc <= wt_acc + v_g; end if;
                wt_i <= wt_i + 1;
                if wt_i = 255 then wt_run <= '0'; end if;
            end if;

            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting (it stands in for a moving camera)
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

            -- the source sample (11 bits), already scaled by K6: fabric copy of the table read
            s_sl <= unsigned(wt_q(10 downto 0));
            s_sv0 <= data_in.avid; s_sv <= s_sv0; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 64 samples; the samples before the line start count as the first one
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(193, 8); end if;
            if s_bcnt < 64 then v_tap := s_sfirst; else v_tap := unsigned(bx_q(10 downto 0)); end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                s_bsum <= shift_left(resize(s_sl, 17), 6);
                s_bcnt <= to_unsigned(1, 7);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 64 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on every 8th column: q += (box / 2 - q + 8) >> 4
            s_eb <= s_bsum(16 downto 1); s_eq <= unsigned(lb_q);
            if s_bv = '1' and s_bcol(2 downto 0) = "111" then s_ev <= '1'; else s_ev <= '0'; end if;
            s_ecol <= s_bcol;
            v_d := signed(resize(s_eb, 18)) - signed(resize(s_eq, 18)) + 8;
            v_a := signed(resize(s_eq, 18)) + shift_right(v_d, 4);
            lb_wd <= unsigned(v_a(15 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 3);
            if s_ecol(10 downto 3) = 0 then s_efirst <= '1'; else s_efirst <= '0'; end if;

            -- until the line's first IIR sample the camera holds the store's column-0 value from the line
            -- above (pixels at the left edge are displaced like their neighbours below)
            if s_sv = '1' and s_svd = '0' then s_cv <= unsigned(lb_q) & "000"; end if;
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
    -- S1..S9: the pole of this pixel (k even: z - p, odd: z + p) on Q4 coordinates, displaced by the
    -- camera -> |x|, |y| -> hi = max, lo = min -> log2 hi, log2 lo (Q14)  (hypnos, 17-bit).
    -- Like: log2 rho ~ log2(max + min/2) of the pixel's own |dx|, |dy|.
    ------------------------------------------------------------------------
    p_logs : process(clk)
        variable vh, vl : unsigned(16 downto 0);
        variable va, vb : signed(18 downto 0);
        variable vy  : unsigned(22 downto 0);
        variable vd  : signed(17 downto 0);
        variable mh, ml : integer range 0 to 16;
        variable vr  : unsigned(13 downto 0);
        variable mr  : integer range 0 to 13;
        variable va15, vb15 : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            -- S1: x = 16 dx + (-+ a cos t) + cam ; y = (dy -+ a sin t) / 16 + cam
            vd := signed(resize(s_cv(18 downto 7), 18));
            s1_x <= signed(f_addsub(unsigned(shift_left(resize(s_dx, 18), 4) + vd), unsigned(s_kx), not s_par));
            vy := f_addsub(unsigned(s_dyq), unsigned(resize(s_ayq, 23)), not s_par);
            s1_y <= signed(vy(22 downto 4)) + resize(vd, 19);
            s1_pf <= s_par & s_first;
            -- S2 (|x| < 74k, |y| < 58k Q4: 17 bits)
            va := abs(resize(s1_x, 19));
            vb := abs(s1_y);
            s2_a <= unsigned(va(16 downto 0));
            s2_b <= unsigned(vb(16 downto 0));
            s2_xs <= s1_x(17);
            s2_ys <= s1_y(18);
            s2_pf <= s1_pf;
            -- S3
            if s2_a >= s2_b then s3_hi <= s2_a; s3_lo <= s2_b; qsr(3)(5) <= '0';
            else                 s3_hi <= s2_b; s3_lo <= s2_a; qsr(3)(5) <= '1'; end if;
            qsr(3)(4) <= s2_xs;
            qsr(3)(3) <= s2_ys;
            if s2_a = 0 or s2_b = 0 then qsr(3)(2) <= '1'; else qsr(3)(2) <= '0'; end if;
            qsr(3)(1 downto 0) <= s2_pf;
            -- S4: msb (zero reads as 1)
            vh := s3_hi; if vh = 0 then vh := to_unsigned(1, 17); end if;
            vl := s3_lo; if vl = 0 then vl := to_unsigned(1, 17); end if;
            mh := 0; ml := 0;
            for i in 0 to 16 loop
                if vh(i) = '1' then mh := i; end if;
                if vl(i) = '1' then ml := i; end if;
            end loop;
            s4_hi <= vh; s4_lo <= vl;
            s4_mh <= to_unsigned(mh, 5); s4_ml <= to_unsigned(ml, 5);
            -- S5: normalise -> 8-bit segment index + 4-bit fraction
            vh := shift_left(s4_hi, 16 - to_integer(s4_mh));
            vl := shift_left(s4_lo, 16 - to_integer(s4_ml));
            s5_ih <= vh(15 downto 8); s5_th <= vh(7 downto 4);
            s5_il <= vl(15 downto 8); s5_tl <= vl(7 downto 4);
            s5_mh <= s4_mh; s5_ml <= s4_ml;
            -- S6, S7 (EBR read, fabric register)
            s6_th <= s5_th; s6_tl <= s5_tl; s6_mh <= s5_mh; s6_ml <= s5_ml;
            s7_th <= s6_th; s7_tl <= s6_tl; s7_mh <= s6_mh; s7_ml <= s6_ml;
            s7_lvh <= lvh_q; s7_lsh <= lsh_q; s7_lvl <= lvl_q; s7_lsl <= lsl_q;
            -- S8
            s8_lph <= resize(unsigned(s7_lsh(6 downto 0)) * s7_th, 11);
            s8_lpl <= resize(unsigned(s7_lsl(6 downto 0)) * s7_tl, 11);
            s8_lvh <= unsigned(s7_lvh(13 downto 0));
            s8_lvl <= unsigned(s7_lvl(13 downto 0));
            s8_mh <= s7_mh; s8_ml <= s7_ml;
            -- S9
            s9_lh <= shift_left(resize(s8_mh, 19), 14) + resize(s8_lvh, 19) + resize(shift_right(s8_lph, 4), 19);
            s9_ll <= shift_left(resize(s8_ml, 19), 14) + resize(s8_lvl, 19) + resize(shift_right(s8_lpl, 4), 19);
            for i in 4 to 9 loop qsr(i) <= qsr(i - 1); end loop;

            -- Like: log2 rho ~ log2(max + min/2) of |dx|, |dy|, msb + 4-bit linear mantissa (the AA width only)
            va15 := abs(s_dx);
            vb15 := abs(s_dyq(22 downto 8));
            s1_da <= unsigned(va15(13 downto 0));
            s1_db <= unsigned(vb15(13 downto 0));
            if s1_da >= s1_db then s2_rs <= s1_da + shift_right(s1_db, 1);
            else                   s2_rs <= s1_db + shift_right(s1_da, 1); end if;
            vr := s2_rs; if vr = 0 then vr := to_unsigned(1, 14); end if;
            mr := 0;
            for i in 0 to 13 loop
                if vr(i) = '1' then mr := i; end if;
            end loop;
            s3_rs <= vr; s3_rm <= to_unsigned(mr, 4);
            vr := shift_left(s3_rs, 13 - to_integer(s3_rm));
            lrr(4) <= std_logic_vector(s3_rm) & std_logic_vector(vr(12 downto 9));
            for i in 5 to 9 loop lrr(i) <= lrr(i - 1); end loop;
        end if;
    end process p_logs;

    ------------------------------------------------------------------------
    -- S10..S16: u = log2(hi/lo) ; log2 r = log2 hi + 0.5 log2(1 + 4^-u) ; the octant angle atan(2^-u)
    -- (EBR tables, 8-bit interpolation) ; the full angle from the swap and signs  (hypnos)
    -- S17, S18: the other pole = the average of the two neighbours (pixel -1: the right one)
    ------------------------------------------------------------------------
    p_polar : process(clk)
        variable vu   : unsigned(18 downto 0);
        variable vmag : unsigned(17 downto 0);
        variable q    : std_logic_vector(2 downto 0);
        variable vlx  : unsigned(19 downto 0);
        variable vtx  : unsigned(18 downto 0);
        variable vdt  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- S10
            vu := s9_lh - s9_ll;
            if qsr(9)(2) = '1' or vu(18) = '1' then s10_ua <= to_unsigned(262143, 18);
            else s10_ua <= vu(17 downto 0); end if;
            -- S11
            s11_ia <= s10_ua(17 downto 10);
            s11_tu <= s10_ua(9 downto 2);
            -- S12, S13
            s12_tu <= s11_tu;
            s13_tu <= s12_tu;
            s13_g <= ug_q; s13_gs <= ugs_q; s13_a <= ua_q; s13_as <= uas_q;
            -- S14: interpolation products (slopes are negative)
            s14_gp <= resize(bmul(signed(s13_gs(10 downto 0)), signed('0' & s13_tu(7 downto 3))), 22);
            s14_ap <= resize(bmul(signed(s13_as(10 downto 0)), signed('0' & s13_tu(7 downto 2))), 22);
            s14_g  <= unsigned(s13_g);
            s14_a  <= unsigned(s13_a);
            -- S15
            s15_g <= resize(unsigned(signed(resize(s14_g, 17)) + resize(shift_right(s14_gp, 5), 17)), 14);
            s15_a <= resize(unsigned(signed(resize(s14_a, 17)) + resize(shift_right(s14_ap, 6), 17)), 16);
            -- S16 (log2 hi, the flags and log2 rho from the delay line)
            s16_l <= unsigned(dll_q(18 downto 0)) + resize(s15_g, 19);
            q := dll_q(21 downto 19);                  -- sw, x < 0, y < 0
            if q(2) = '1' then vmag := to_unsigned(65536, 18) - resize(s15_a, 18);
            else               vmag := resize(s15_a, 18); end if;
            if    q(1) = '0' and q(0) = '0' then s16_th <= vmag;
            elsif q(1) = '1' and q(0) = '0' then s16_th <= to_unsigned(131072, 18) - vmag;
            elsif q(1) = '1' and q(0) = '1' then s16_th <= to_unsigned(131072, 18) + vmag;
            else                                 s16_th <= to_unsigned(0, 18) - vmag; end if;
            s16_pf <= dll_q(23 downto 22);
            s16_lr <= dll_q(31 downto 24);
            -- S17: this pixel ; the S18 register keeps the pixel before (the next one's left neighbour)
            s17_l <= s16_l; s17_th <= s16_th; s17_pf <= s16_pf; s17_lr <= s16_lr;
            s18_l <= s17_l; s18_th <= s17_th;
            -- S18: own pole x 2 ; other pole = left + right (Q15 / 2^19 per turn, shortest arc)
            if s17_pf(0) = '1' then
                vlx := shift_left(resize(s16_l, 20), 1);
                vtx := s16_th & '0';
            else
                vlx := resize(s16_l, 20) + resize(s18_l, 20);
                vdt := s16_th - s18_th;
                vtx := (s18_th & '0') + unsigned(resize(signed(vdt), 19));
            end if;
            if s17_pf(1) = '0' then
                s18_l1 <= s17_l & '0'; s18_l2 <= vlx;
                s18_t1 <= s17_th & '0'; s18_t2 <= vtx;
            else
                s18_l1 <= vlx; s18_l2 <= s17_l & '0';
                s18_t1 <= vtx; s18_t2 <= s17_th & '0';
            end if;
            s18_lr <= s17_lr;
        end if;
    end process p_polar;

    ------------------------------------------------------------------------
    -- S19..S26: sigma, tau, the gradient, the two phases (N bands per 2 pi, mod one cycle, Q16)
    ------------------------------------------------------------------------
    p_phase : process(clk)
        variable v_tb  : unsigned(20 downto 0);
        variable v_s18 : unsigned(17 downto 0);
        variable v_sg  : unsigned(18 downto 0);
        variable vlr   : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            -- S19: sigma = (th1 -+ th2) / 2 ; (L1 + L2) / 2 ; (L1 - L2) / 2
            v_sg := f_addsub(s18_t1, s18_t2, not s_like);
            s19_sig <= v_sg(18 downto 1);
            v_tb := resize(s18_l1, 21) + resize(s18_l2, 21);
            s19_sl  <= v_tb(20 downto 1);
            v_tb := unsigned(signed(resize(s18_l1, 21)) - signed(resize(s18_l2, 21)));
            s19_dl  <= signed(v_tb(20) & v_tb(20 downto 1));
            -- the gradient constant (+ log2 rho, Like: msb + 4-bit mantissa)
            vlr := signed(resize(unsigned(s18_lr(7 downto 4)) & "00000000000000", 21))
                   + signed(resize(unsigned(s18_lr(3 downto 0)) & "0000000000", 21));
            if s_like = '1' then s19_cg <= resize(s_Cg, 21) + vlr;
            else                 s19_cg <= resize(s_Cg, 21); end if;
            -- S20: tau (Opposite L1 - L2, Like L1 + L2 - 2 log2 a) ; log2 |grad| ; sigma x N, first half
            if s_like = '1' then s20_tau <= signed(resize(s19_sl, 21)) - signed(resize(s_cl, 21));
            else                 s20_tau <= s19_dl; end if;
            s20_lgw <= s19_cg - signed(resize(s19_sl, 21));
            s20_q0 <= nmul(s19_sig, s_nj, s_nk, s_n2);
            -- S21: the sigma phase ; tau x 4 ln2 / 2pi in two halves (CSD 2 - 1/4 + 1/64 - 1/2048) ;
            --      the gradient in Q6
            s21_ps <= s20_q0(17 downto 2);
            s21_x1 <= shift_left(resize(s20_tau, 23), 1) - resize(shift_right(s20_tau, 2), 23);
            s21_x2 <= resize(shift_right(s20_tau, 6), 23) - resize(shift_right(s20_tau, 11), 23);
            s21_lw <= resize(shift_right(s20_lgw, 8), 14);
            -- S22
            sps(22) <= s21_ps + s_offs;
            v_tb := unsigned(resize(s21_x1 + s21_x2, 21));
            s22_t1 <= v_tb(17 downto 0);
            -- S23: tau x N, first half
            s23_q0 <= nmul(s22_t1, s_nj, s_nk, s_n2);
            -- S24
            s24_pt <= s23_q0(17 downto 2);
            -- S25
            s25_pt <= s24_pt + s_offt;
            for i in 23 to 25 loop sps(i) <= sps(i - 1); end loop;
            -- S26
            s26_phi(0) <= sps(25);
            s26_phi(1) <= s25_pt;
        end if;
    end process p_phase;

    ------------------------------------------------------------------------
    -- the anti-aliasing width (hypnos):  log2 w (Q6) = log2 |grad| ; above the knee it is widened so
    -- the filter reaches a full cycle (flat grey) before the lines alias.  Bold: knee at 1/4 cycle per
    -- px, flat at 1/2.  Fine: knee at 1/12 (a 2-px line), flat near 1/5.  Rounded UP to a half octave
    -- so the divide is a shift (edges 1 .. 1.4 px).
    ------------------------------------------------------------------------
    p_grad : process(clk)
        variable v_lw  : signed(13 downto 0);
        variable v_t   : signed(11 downto 0);
        variable v_v   : signed(12 downto 0);
        variable v_rr  : signed(13 downto 0);
        variable v_sh  : integer range -8 to 15;
        variable v_w   : unsigned(15 downto 0);
        variable v_m   : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- S24
            v_lw := s21_lw;
            if v_lw > 255 then v_lw := to_signed(255, 14);
            elsif v_lw < -1023 then v_lw := to_signed(-1023, 14); end if;
            s22_lg <= v_lw(11 downto 0);
            -- S23
            if s_fine = '1' then
                v_t := s22_lg + 229;
                s23_tk <= s22_lg + 458;
            else
                v_t := s22_lg + 128;
                s23_tk <= to_signed(128, 12);
            end if;
            if v_t > 0 then s23_fl <= '1'; else s23_fl <= '0'; end if;
            s23_lg <= s22_lg;
            -- S24
            if s23_fl = '1' then s24_v <= shift_left(resize(s23_lg, 13), 1) + resize(s23_tk, 13);
            else                 s24_v <= resize(s23_lg, 13); end if;
            -- S25: quantise up to a half octave ; reciprocal 2^-log2 w = 2^ri * {1, sqrt 2}
            v_v := s24_v;
            if v_v < -1023 then v_v := to_signed(-1023, 13); end if;
            v_v := v_v + 31;
            v_v(4 downto 0) := (others => '0');
            if v_v >= 0 then
                s25_sat <= '1'; s25_ri <= (others => '0'); s25_rh <= '0';
            else
                s25_sat <= '0';
                v_rr := -resize(v_v, 14);
                s25_ri <= unsigned(v_rr(9 downto 6));
                s25_rh <= v_rr(5);
            end if;
            -- S26: half the filter width, w / 2 (Q16): w = {64, 91} << (10 - ri - rh)
            if s25_sat = '1' then
                v_w := (others => '1');
            else
                v_sh := 10 - to_integer(s25_ri);
                if s25_rh = '1' then v_m := to_unsigned(91, 7); v_sh := v_sh - 1;
                else                 v_m := to_unsigned(64, 7); end if;
                if v_sh >= 0 then v_w := resize(shift_left(resize(v_m, 23), v_sh), 16);
                else              v_w := resize(shift_right(resize(v_m, 16), -v_sh), 16); end if;
                if v_w = 0 then v_w := to_unsigned(1, 16); end if;
            end if;
            s26_wh <= v_w(15 downto 1);
            s26_ri <= s25_ri; s26_rh <= s25_rh;
            s27_ri <= s26_ri; s27_rh <= s26_rh;
            s28_ri <= s27_ri; s28_rh <= s27_rh;
            s29_ri <= s28_ri; s29_rh <= s28_rh;
            s30_rh <= s29_rh;
        end if;
    end process p_grad;

    ------------------------------------------------------------------------
    -- S27..S31: one-pixel box filter of the band square wave (hypnos).
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
                -- S27
                s27_lo(c) <= s26_phi(c) - resize(s26_wh, 16);
                s27_hi(c) <= s26_phi(c) + resize(s26_wh, 16);
                -- S28
                if s27_hi(c) < s27_lo(c) then s28_k(c) <= '1'; else s28_k(c) <= '0'; end if;
                if s27_hi(c) >= s27_lo(c) and s27_hi(c) < s_D16 then s28_full(c) <= '1'; else s28_full(c) <= '0'; end if;
                if s27_hi(c) < s_D16 then s28_mh(c) <= s27_hi(c); else s28_mh(c) <= s_D16; end if;
                if s27_lo(c) < s_D16 then s28_ml(c) <= s27_lo(c); else s28_ml(c) <= s_D16; end if;
                -- S29
                vn := signed(resize(s28_mh(c), 18)) - signed(resize(s28_ml(c), 18));
                if s28_k(c) = '1' then vn := vn + signed(resize(s_D16, 18)); end if;
                if vn < 0 then vn := (others => '0'); end if;
                s29_n(c) <= unsigned(vn(15 downto 0));
                s29_full(c) <= s28_full(c);
                -- S30: (N << ri) >> 7
                vs := shift_left(resize(s29_n(c), 25), to_integer(s29_ri));
                s30_nsh(c) <= vs(16 downto 7);
                s30_full(c) <= s29_full(c);
                -- S31: / 2, or * sqrt2 / 2 on a half-octave step
                if s30_rh = '1' then
                    s31_cp(c) <= resize(shift_right(s30_nsh(c), 1) + shift_right(s30_nsh(c), 3)
                                        + shift_right(s30_nsh(c), 4) + shift_right(s30_nsh(c), 6), 11);
                else
                    s31_cp(c) <= resize(shift_right(s30_nsh(c), 1), 11);
                end if;
                s31_full(c) <= s30_full(c);
                -- S32: coverage 0..256
                vc := resize(s31_cp(c), 10);
                if vc > 256 or s31_full(c) = '1' then vc := to_unsigned(256, 10); end if;
                s32_cov(c) <= resize(vc, 9);
            end loop;
        end if;
    end process p_cov;

    ------------------------------------------------------------------------
    -- S33..S39: weave (Lines: sigma ; Checker Bold: XOR = a + b - 2ab ; Checker Fine: OR = a + b - ab),
    -- luma 64 .. 940, then the output conditioning: [1 2 1]/4, luma 13/16 toward black (white 775),
    -- blanking gate.  U = V = 512 always.  The filter's neighbours at the picture edges are real
    -- pixels: the one left of the line (dx = -cx - 1) and the last one held.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_c  : signed(11 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_s  : unsigned(18 downto 0);
        variable v_f  : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S33
            s33_pp <= s32_cov(0)(8 downto 4) * s32_cov(1)(8 downto 4);          -- 0 .. 256 (coverages in 16ths)
            s33_ab <= resize(s32_cov(0)(8 downto 4), 6) + resize(s32_cov(1)(8 downto 4), 6);
            s33_a  <= s32_cov(0);
            -- S34
            v_p := s33_pp;
            if s_fine = '0' then v_p := shift_left(v_p, 1); end if;
            v_c := signed(resize(s33_ab & "0000", 12)) - signed(resize(v_p, 12));
            if s_chk = '0' then s34_c <= s33_a;
            elsif v_c > 256 then s34_c <= to_unsigned(256, 9);
            elsif v_c < 0 then s34_c <= (others => '0');
            else s34_c <= unsigned(v_c(8 downto 0)); end if;
            -- S35: y = 64 + c * 876 / 256   (876 = 1024 - 128 - 16 - 4)
            v_s := shift_left(resize(s34_c, 19), 10) - shift_left(resize(s34_c, 19), 7)
                   - shift_left(resize(s34_c, 19), 4) - shift_left(resize(s34_c, 19), 2);
            s35_y <= to_unsigned(64, 10) + v_s(17 downto 8);
            -- S36, S36: [1 2 1] / 4 around the stage-35 pixel
            s36_y <= s35_y;
            s37_y <= s36_y;
            v_f := resize(s35_y, 12) + shift_left(resize(s36_y, 12), 1) + resize(s37_y, 12) + 2;
            s37_f <= v_f(11 downto 2);
            -- S38: luma 13/16 toward black
            if s37_f > 64 then v_d := s37_f - 64; else v_d := (others => '0'); end if;
            s38_y <= s37_f - shift_right(v_d, 3) - shift_right(v_d, 4);
            -- S39: blanking gate on the aligned generated avid
            if sy(38)(3) = '1' then o_y <= s38_y; else o_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(C_MID);
    data_out.v       <= std_logic_vector(C_MID);
    data_out.avid    <= sy(39)(3);
    data_out.hsync_n <= sy(39)(2);
    data_out.vsync_n <= sy(39)(1);
    data_out.field_n <= sy(39)(0);

end architecture dipole;
