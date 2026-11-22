-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- streamline.vhd  (Hardline, v0.1.0) -- potential flow past a cylinder, in black-and-white bands
--
-- Art reference: the streamline photographs of Van Dyke's "An Album of Fluid Motion", Hele-Shaw cells,
-- Prandtl's drawings; with spin, the Magnus effect.  Level bands part round a solid disc, squeeze over
-- its top and bottom, stall at the two stagnation points and close again behind it; dashes cut into
-- them travel along the streamlines.
--
--   W = z + R^2/z - i G log z   (z = x + i y, y DOWN the screen; G = 2 s R)   -- in psi cycles (period P):
--   psi = y/P - y k/P - (2 ln2 s R/P) (log2 r - log2 R) + bend      the streamlines
--   phi = x/4P + x k/4P - drift                                      the dash cuts (the equipotentials of
--                                                                    the flow without spin)
--   k = R^2 / r^2:   y k / P = 2^(log2|y| + 2 (log2 R - log2 r) - log2 P)   (one exp2 table read each)
--
-- computed on the Hypnos polar chassis (log2 r from two integer logs, no CORDIC, no divide):
--   y/P, x/4P            y/P per line on the serial multiplier (|dy| x 1/P), x/4P a per-pixel DDA; 1/P
--                        comes from the same 2^(j/256) table that defines log2 P (Q8), so the near and far
--                        field agree; both start at the band offsets and the drift
--   y k/P, x k/4P        log2|y| (|x|) - 2 log2 r + per-field constant -> exp2 mantissa table -> one shift;
--                        Solid: k is held at 1 inside the disc
--   spin                 (log2 r - log2 R) x a 2-bit mantissa (one add / subtract), one shift
--   anti-aliasing        the map is conformal: |grad psi| = |dpsi/dx + i dphi/dx|, both from the previous
--                        pixel (L1 norm, log2(4/3) off an eighth-octave log), Hypnos's knee and half-octave
--                        width, into Hypnos's one-pixel box filter of the
--                        band square wave; bands finer than about 2 px fade to flat grey (Open: where k/P
--                        passes a cycle per px)
--   Dashes               psi band XOR phi band (Bold: a quarter-period cut; Fine: OR, a flow net)
--   Solid                a white disc with a thin black ring (6 virtual px), its edges anti-aliased
--
--   K1  X        the disc's centre, +-1/2 screen width (detent at the centre)
--   K2  Y        the disc's centre, +-1/2 screen height (detent)
--   K3  Period   the streamline spacing, 300 -> 13 virtual px (exponential, clockwise = finer)
--   K4  Spin     the circulation, bipolar with a centre detent: s = 1.6 t|t| in steps of a quarter
--                octave of G; clockwise lifts the flow (the stagnation points slide down round the disc,
--                leave it at s = 1, loops close round it)
--   K5  Drift    bipolar (Hypnos v1.1.0 Speed): centre +-24 counts = still, up = the dashes run left to
--                right; exponential 2^-12 .. 2^-2 cycle per field; wraps exactly one dash period.  In Lines
--                the streamlines creep instead, so there is always motion
--   K6  Camera   box-filtered luma added to psi: 2 periods at white at 100 %; 0 = the clean figure
--   S7  Glide    On: X, Y, Period, Spin, Drift's speed, Camera and Body ease 1/16 per field, never slower
--                than half a count, landing exactly / Off: no easing (the value latched this field)
--   S8  Weave    Dashes (psi XOR phi) / Lines (psi alone)
--   S9  Body     Solid (a white disc, black ring) / Open (the formula continued inside: the doublet's eye)
--   S10 Weight   Bold (equal bands, quarter-period cuts) / Fine (1/6 lines; Dashes: a flow net)
--   S11 Source   Video / Osc (the shell's slow triangle lattice in the camera's place, at twice its level)
--   P12 BODY     the radius: 20 px (a bead) .. 180 px (50 %) .. 499 px (a great disc), virtual
--
-- Chassis: timing / generated avid / log-polar engine / box-filter AA from hypnos.vhd v1.1.0 by way of
-- hardline multipole v0.2.0; the source field (lattice, 32-px box, vertical IIR) from kinetic riley.
-- Luma only: U = V = 512 on every pixel.  One latency (49) in every mode.  23 EBR.
--
-- Stage numbering: stage N = N registers after s_dx.  o_y is stage 47.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.streamline_pkg.all;

architecture streamline of program_top is
    constant C_PROCESSING_DELAY_CLKS : integer := 49;      -- = processing_delay_clks in the TOML

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_DB    : integer := 14;                 -- X / Y / Spin / Drift detent deadband
    constant C_GL    : integer := 4;                  -- glide one-pole shift (1/16 per field)
    constant C_GMIN  : integer := 32;                 -- ... never slower than half a knob count per field

    constant C_LG0   : integer := 18830;              -- log2(2 ln2 1.6), Q14

    -- TOML defaults (the holds and glides start there)
    constant C_DEF_K1  : integer := 416;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 400;
    constant C_DEF_K4  : integer := 512;
    constant C_DEF_K5  : integer := 680;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;

    constant C_XA   : integer := 5;                   -- log2 hi / lo: stage 8 -> EBR output = stage 14
    constant C_XB   : integer := 4;                   -- spin term + flags: stage 19 -> stage 24
    constant C_XC   : integer := 6;                   -- psi, phi: stage 26 -> stage 33
    constant C_XD   : integer := 20;                  -- disc weights: stage 20 -> stage 41
    constant C_XS   : integer := 46;                  -- syncs: delay 49

    attribute no_rw_check : boolean;

    ----------------------------------------------------------------------
    -- field-latched controls + per-field terms
    ----------------------------------------------------------------------
    signal s_init   : std_logic := '1';               -- first field: latch and snap unconditionally
    -- the seven knobs K1 K2 K3 K4 K5 K6 P12 as two rings (hold, Q6 glide): one datapath serves all
    type t_h7 is array(0 to 6) of unsigned(9 downto 0);
    type t_g7 is array(0 to 6) of unsigned(15 downto 0);
    signal r_g : t_g7 := (to_unsigned(C_DEF_K1 * 64, 16), to_unsigned(C_DEF_K2 * 64, 16), to_unsigned(C_DEF_K3 * 64, 16),
                          to_unsigned(C_DEF_K4 * 64, 16), to_unsigned(C_DEF_K5 * 64, 16), to_unsigned(C_DEF_K6 * 64, 16),
                          to_unsigned(C_DEF_P12 * 64, 16));
    signal s_ri     : unsigned(2 downto 0) := (others => '0');
    signal s_rp     : unsigned(1 downto 0) := (others => '0');
    signal s_rh     : unsigned(9 downto 0) := (others => '0');
    signal s_rt     : unsigned(15 downto 0) := (others => '0');
    signal s_kd     : signed(16 downto 0) := (others => '0');
    signal s_glide  : std_logic := '1';               -- S7
    signal s_lines  : std_logic := '0';               -- S8  0 Dashes / 1 Lines
    signal s_solid  : std_logic := '1';               -- S9  0 Solid / 1 Open (inverted here)
    signal s_fine   : std_logic := '0';               -- S10 0 Bold / 1 Fine
    signal s_src    : std_logic := '0';               -- S11 0 Video / 1 Osc

    signal s_cx, s_cy : signed(13 downto 0) := (others => '0');
    -- PERIOD
    signal s_v9     : unsigned(19 downto 0) := (others => '0');
    signal s_lp     : unsigned(11 downto 0) := to_unsigned(1657, 12);  -- log2 P (x pixels), Q8
    signal s_ex     : unsigned(3 downto 0) := to_unsigned(7, 4);
    signal s_isc    : unsigned(3 downto 0) := (others => '0');          -- 1/P: serial shift count
    signal s_nlp14, s_nlp14x, s_kbt : signed(20 downto 0) := (others => '0');  -- -log2 P (- 2), -log2 P / 2
    signal s_inv    : unsigned(29 downto 0) := (others => '0');          -- 1/P, Q30
    signal s_ninv   : signed(31 downto 0) := (others => '0');
    signal s_u0     : unsigned(19 downto 0) := (others => '0');          -- x/4P at pixel -2 + offset, Q20
    -- BODY
    signal s_lrhi   : std_logic := '0';
    signal s_lrv    : unsigned(17 downto 0) := to_unsigned(122748, 18);
    signal s_LR     : unsigned(17 downto 0) := to_unsigned(113163, 18); -- log2 R (x pixels), Q14
    signal s_lrp    : signed(10 downto 0) := (others => '0');            -- log2 R - log2 P, Q4
    signal s_lkd    : unsigned(17 downto 0) := (others => '0');
    signal s_ed     : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_f2     : unsigned(1 downto 0) := (others => '0');
    -- SPIN
    signal s_sa     : unsigned(9 downto 0) := (others => '0');
    signal s_sneg, s_sz : std_logic := '0';
    signal s_lt     : signed(8 downto 0) := (others => '0');             -- log2 |t|, Q4
    signal s_lga, s_lg : signed(10 downto 0) := (others => '0');         -- log2 g, Q4
    signal s_sM     : unsigned(2 downto 0) := (others => '0');            -- 4 .. 7 (0 = off)
    signal s_rr     : unsigned(3 downto 0) := (others => '0');            -- 7 - exponent (0 .. 12)
    -- camera gain, duties, offsets
    signal s_g1, s_g2 : unsigned(16 downto 0) := (others => '0');
    signal s_gain   : unsigned(6 downto 0) := to_unsigned(74, 7);
    signal s_Dps    : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal s_Dph    : unsigned(15 downto 0) := to_unsigned(16384, 16);
    signal s_offps  : unsigned(15 downto 0) := to_unsigned(16384, 16);
    signal s_offph  : unsigned(15 downto 0) := to_unsigned(8192, 16);
    -- DRIFT: decode, serial shifter, the phase accumulator (Q20 cycles)
    signal s_vneg, s_vzero : std_logic := '0';
    signal s_av, s_u, s_ls : unsigned(9 downto 0) := (others => '0');
    signal s_spd    : unsigned(19 downto 0) := (others => '0');
    signal s_ssh    : unsigned(3 downto 0) := (others => '0');
    signal s_anim   : unsigned(19 downto 0) := (others => '0');
    -- per-mode constants (timing ID)
    signal s_ls8    : unsigned(8 downto 0) := (others => '0');
    signal s_ls14   : unsigned(14 downto 0) := (others => '0');
    signal s_rw     : unsigned(7 downto 0) := to_unsigned(96, 8);

    -- vblank sequencer + serial multiplier (loaded by the sequencer and the line start; the accumulator
    -- may start at an offset)
    signal s_seq   : unsigned(7 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse, s_vs_d : std_logic := '0';
    signal s_saw_avid   : std_logic := '0';
    signal sm_a    : signed(31 downto 0) := (others => '0');
    signal sm_b    : unsigned(15 downto 0) := (others => '0');
    signal sm_acc  : signed(31 downto 0) := (others => '0');
    signal sm_n    : unsigned(4 downto 0) := (others => '0');
    signal t2_a    : unsigned(7 downto 0) := (others => '0');
    signal t2_q, t2_r : std_logic_vector(15 downto 0) := (others => '0');

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
    signal s_lc       : unsigned(4 downto 0) := (others => '1');   -- clocks since the line start (saturating)
    signal s_uu       : unsigned(19 downto 0) := (others => '0');  -- x/4P (Q20): stage 24
    signal s_vv       : unsigned(17 downto 0) := (others => '0');  -- y/P (Q16, mod 4): stage 24

    ----------------------------------------------------------------------
    -- EBR delay lines (one write, one registered read each)
    ----------------------------------------------------------------------
    type t_dl48 is array(0 to 255) of std_logic_vector(47 downto 0);
    type t_dl32 is array(0 to 255) of std_logic_vector(31 downto 0);
    type t_dl16 is array(0 to 255) of std_logic_vector(15 downto 0);
    signal dla  : t_dl48 := (others => (others => '0'));      -- log2 hi, log2 lo, flags (stage 8)
    signal dlb  : t_dl32 := (others => (others => '0'));      -- spin term, flags, kbig (stage 19)
    signal dlc  : t_dl48 := (others => (others => '0'));      -- psi, phi (stage 26)
    signal dld  : t_dl16 := (others => (others => '0'));      -- disc weights (stage 20)
    signal dls  : t_dl16 := (others => (others => '0'));      -- generated avid + syncs
    attribute no_rw_check of dla : signal is true;
    attribute no_rw_check of dlb : signal is true;
    attribute no_rw_check of dlc : signal is true;
    attribute no_rw_check of dld : signal is true;
    attribute no_rw_check of dls : signal is true;
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal dla_q, dlc_q : std_logic_vector(47 downto 0) := (others => '0');
    signal dla_d, dlc_d : std_logic_vector(47 downto 0);
    signal dlb_q : std_logic_vector(31 downto 0) := (others => '0');
    signal dlb_d : std_logic_vector(31 downto 0);
    signal dld_q : std_logic_vector(15 downto 0) := (others => '0');
    signal dld_d : std_logic_vector(15 downto 0);
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
    -- gain table: (level - 16) * gain >> 5 (Osc: level * 2 gain >> 5), refilled every field
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
    signal s_cv    : unsigned(18 downto 0) := (others => '0');          -- the bend, Q16 psi cycles
    signal s_cn    : unsigned(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    -- S1..S8: |dx|,|dy| -> hi/lo -> log2 hi, log2 lo (Q14)
    signal s1_a, s1_b : unsigned(12 downto 0) := (others => '0');
    signal s1_dxs, s1_dys : std_logic := '0';
    signal s2_hi, s2_lo : unsigned(12 downto 0) := (others => '0');
    signal s2_sw, s2_dxs, s2_dys, s2_lz, s2_xz, s2_yz : std_logic := '0';
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
    type t_q is array(3 to 8) of std_logic_vector(5 downto 0);    -- sw, dxs, dys, lz, xz, yz
    signal qsr : t_q := (others => (others => '0'));

    -- S9..S15: u = log2(hi/lo) -> log2 r ; log2|y| - log2 P, log2|x| - log2 4P
    signal s9_ua  : unsigned(17 downto 0) := (others => '0');
    signal s10_ia : unsigned(7 downto 0) := (others => '0');
    signal s10_tu, s11_tu, s12_tu : unsigned(4 downto 0) := (others => '0');
    signal ug_q, ugs_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s12_g, s12_gs : std_logic_vector(15 downto 0) := (others => '0');
    signal s13_gp : signed(21 downto 0) := (others => '0');
    signal s13_g  : unsigned(15 downto 0) := (others => '0');
    signal s14_g  : unsigned(13 downto 0) := (others => '0');
    signal s15_lr : unsigned(17 downto 0) := (others => '0');    -- log2 r (x pixels), Q14
    signal s15_ey, s15_ex, s16_ey, s16_ex : signed(20 downto 0) := (others => '0');
    signal s15_f  : std_logic_vector(3 downto 0) := "0000";      -- xs, ys, xz, yz
    type t_f is array(16 to 19) of std_logic_vector(3 downto 0);
    signal fsr : t_f := (others => (others => '0'));

    -- S16..S22: dL ; the exp2 exponents (k held at 1 inside a Solid disc) ; the spin term ; the disc
    signal s16_dl : signed(19 downto 0) := (others => '0');
    signal s17_ey, s17_ex : signed(20 downto 0) := (others => '0');
    signal s17_kb, s18_kb, s19_kb : std_logic := '0';
    signal s17_sp  : signed(20 downto 0) := (others => '0');
    signal s18_sp  : signed(29 downto 0) := (others => '0');
    signal s19_sp  : unsigned(17 downto 0) := (others => '0');
    signal s17_dsh : signed(19 downto 0) := (others => '0');
    signal s18_ds  : signed(8 downto 0) := (others => '0');
    signal s19_dp  : signed(11 downto 0) := (others => '0');
    signal s20_t1, s20_t2 : unsigned(4 downto 0) := (others => '0');
    signal ksr     : std_logic_vector(24 to 31) := (others => '0');  -- kbig after the B line

    -- S18..S24: the two exp2 chains: 0 = y k / P (18 bits, mod 4 cycles), 1 = x k / 4P (16 bits)
    type t_s6x2  is array(0 to 1) of signed(5 downto 0);
    type t_u21x2 is array(0 to 1) of unsigned(20 downto 0);
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
    signal s18_xa  : t_u8x2 := (others => (others => '0'));
    signal s18_t, s19_t, s20_t : t_u6x2 := (others => (others => '0'));
    signal s18_k, s19_k, s20_k, s21_k, s22_k : t_s6x2 := (others => (others => '0'));
    signal exm_q, exs_q : t_v16x2 := (others => (others => '0'));
    signal s20_m, s21_m, s22_mt : t_u16x2 := (others => (others => '0'));
    signal s20_s   : t_u8x2 := (others => (others => '0'));
    signal s21_ep  : t_u14x2 := (others => (others => '0'));
    signal s23_y1  : t_u21x2 := (others => (others => '0'));
    signal s23_f   : t_u2x2 := (others => (others => '0'));
    signal s24_my  : unsigned(17 downto 0) := (others => '0');
    signal s24_mx  : unsigned(15 downto 0) := (others => '0');

    -- S25..S33: psi, phi, their differences, the gradient, the anti-aliasing width
    signal s25_a1, s25_a2 : unsigned(17 downto 0) := (others => '0');
    signal s25_phi, s26_phi, s27_fp : unsigned(15 downto 0) := (others => '0');
    signal s26_psi, s27_pp : unsigned(17 downto 0) := (others => '0');
    signal s27_a, s27_b : unsigned(17 downto 0) := (others => '0');
    signal s28_g   : unsigned(18 downto 0) := (others => '0');
    signal s29_p   : unsigned(4 downto 0) := (others => '0');
    signal s29_m   : unsigned(2 downto 0) := (others => '0');
    signal s29_nz  : std_logic := '0';
    signal s30_h   : signed(8 downto 0) := (others => '0');
    signal s31_c0, s31_c1 : signed(7 downto 0) := (others => '0');
    signal s31_fl  : std_logic := '0';
    signal s32_ri, s33_ri, s34_ri, s35_ri, s36_ri : unsigned(3 downto 0) := (others => '0');
    signal s32_rh, s33_rh, s34_rh, s35_rh, s36_rh, s37_rh : std_logic := '0';
    signal s32_sat, s33_sat, s34_sat, s35_sat, s36_sat : std_logic := '0';
    signal s33_wh, s33_whf : unsigned(14 downto 0) := (others => '1');

    -- S34..S39: one-pixel box filter -> coverage 0..256 (0 = psi, 1 = phi)
    signal s34_lo, s34_hi, s35_mh, s35_ml, s36_n : t_u16x2 := (others => (others => '0'));
    signal s35_k, s35_full, s36_full, s37_full, s38_full : t_bx2 := (others => '0');
    signal s37_nsh : t_u10x2 := (others => (others => '0'));
    signal s38_cp  : t_u11x2 := (others => (others => '0'));
    signal s39_cov : t_u9x2 := (others => (others => '0'));

    -- S40..S47: weave, disc, luma, output conditioning
    signal s40_d   : signed(9 downto 0) := (others => '0');
    signal s40_a   : unsigned(8 downto 0) := (others => '0');
    signal s40_ge  : std_logic := '0';
    signal s40_b   : unsigned(8 downto 0) := (others => '0');
    signal s41_c, s42_c : unsigned(8 downto 0) := (others => '0');
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

    -- Exact signed product a * m, radix-4 Booth on m (hypnos), one stage
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

    -- a x m for m = 4 .. 7 on ONE carry chain: 4a, 4a + a, 4a + 2a, 8a - a
    function f_mul47(a : signed; m : unsigned(1 downto 0)) return signed is
        variable v_a  : unsigned(a'length + 2 downto 0);
        variable v_b  : unsigned(a'length + 2 downto 0);
        variable v_r  : unsigned(a'length + 2 downto 0);
    begin
        if m = "11" then v_a := unsigned(shift_left(resize(a, a'length + 3), 3));
        else             v_a := unsigned(shift_left(resize(a, a'length + 3), 2)); end if;
        case m is
            when "01"   => v_b := unsigned(resize(a, a'length + 3));
            when "10"   => v_b := unsigned(shift_left(resize(a, a'length + 3), 1));
            when "11"   => v_b := unsigned(resize(a, a'length + 3));
            when others => v_b := (others => '0');
        end case;
        v_r := f_addsub(v_a, v_b, m(1) and m(0));
        return signed(v_r);
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
            -- pitch in x-pixel units (Q8); the Osc lattice's steps; log2 of the x step; the ring width
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
                when "10"   => s_oxs <= to_unsigned(204, 9); s_ls8 <= to_unsigned(256, 9);
                               s_ls14 <= to_unsigned(16384, 15); s_rw <= to_unsigned(48, 8);
                when "01"   => s_oxs <= to_unsigned(153, 9); s_ls8 <= to_unsigned(150, 9);
                               s_ls14 <= to_unsigned(9585, 15); s_rw <= to_unsigned(64, 8);
                when others => s_oxs <= to_unsigned(102, 9); s_ls8 <= to_unsigned(0, 9);
                               s_ls14 <= to_unsigned(0, 15); s_rw <= to_unsigned(96, 8);
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
        variable v_u11  : unsigned(10 downto 0);
        variable v_m7   : unsigned(6 downto 0);
        variable v_ab   : unsigned(15 downto 0);
        variable v_p    : integer range 0 to 9;
        variable v_n9   : unsigned(9 downto 0);
        variable v_eg   : signed(6 downto 0);
        variable v_c2   : signed(14 downto 0);
        variable v_d16  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            s_vs_d       <= s_vs_pulse;
            -- the serial multiplier: sm_acc += sm_a while bits of sm_b remain (16 clocks after a load)
            if sm_n /= 0 then
                if sm_b(0) = '1' then sm_acc <= sm_acc + sm_a; end if;
                sm_a <= shift_left(sm_a, 1);
                sm_b <= shift_right(sm_b, 1);
                sm_n <= sm_n - 1;
            end if;
            if data_in.avid = '1' then s_saw_avid <= '1'; end if;

            -- DRIFT mantissa << (octave + 2), one bit per clock (Hypnos); loaded at sequencer state 225
            -- (a negative speed loads the one's complement and shifts in ones: -(m << k) - 1)
            if s_ssh /= 0 then
                s_spd <= s_spd(18 downto 0) & s_vneg;
                s_ssh <= s_ssh - 1;
            end if;
            if s_seq = 225 and data_in.avid = '0' then
                if s_vzero = '1' then s_spd <= (others => '0'); s_ssh <= (others => '0');
                else
                    v_m7 := '1' & s_ls(5 downto 0);                  -- 64 + f
                    if s_vneg = '1' then v_m7 := not v_m7; end if;
                    s_spd <= (19 downto 9 => s_vneg) & v_m7 & s_vneg & s_vneg;   -- (64 + f) << 2, signed
                    s_ssh <= s_ls(9 downto 6);
                end if;
            end if;
            -- y/P for each line on the serial multiplier, started 2 clocks into the line (|dy| is then at
            -- stage 1): |dy| x (1/P, or -1/P - 1 above the disc), the accumulator starting at the psi offset;
            -- read 23 clocks into the line (stage 24 of pixel -2)
            if s_lc = 1 then
                if s1_dys = '1' then sm_a <= s_ninv; else sm_a <= signed(resize(s_inv, 32)); end if;
                sm_b <= resize(s1_b, 16); sm_acc <= signed(shift_left(resize(s_offps, 32), 14)); sm_n <= to_unsigned(16, 5);
            end if;
            -- 1/P: the table value shifted left one bit per clock (loaded at state 223)
            if s_isc /= 0 then
                s_inv <= s_inv(28 downto 0) & '0';
                s_isc <= s_isc - 1;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_saw_avid = '1' then
                s_vs_pulse <= '1';
                s_saw_avid <= '0';
                s_glide <= registers_in(6)(0);
                s_lines <= registers_in(6)(1);
                s_solid <= not registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_Wl    <= s_W;
                s_seq   <= to_unsigned(250, 8);
                s_ri    <= (others => '0');
                s_rp    <= (others => '0');

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                -- KNOB RING (states 249 .. 229, three per knob): +-1-count hold against pot jitter (about the
                -- glided value, which lands exactly on the held target), then
                -- the glide: 1/16 of the gap per field, never slower than half a count, landing exactly.
                -- Glide off (or the first field): the held value itself, the same field.
                if s_seq <= 249 and s_seq >= 229 then
                    case s_rp is
                        when "00" =>
                            case to_integer(s_ri) is
                                when 0      => v_raw := unsigned(registers_in(0));
                                when 1      => v_raw := unsigned(registers_in(1));
                                when 2      => v_raw := unsigned(registers_in(2));
                                when 3      => v_raw := unsigned(registers_in(3));
                                when 4      => v_raw := unsigned(registers_in(4));
                                when 5      => v_raw := unsigned(registers_in(5));
                                when others => v_raw := unsigned(registers_in(7));
                            end case;
                            v_e := signed('0' & v_raw) - signed('0' & r_g(0)(15 downto 6));
                            if s_init = '1' or (v_e /= 0 and v_e /= 1 and v_e /= -1) then s_rh <= v_raw; else s_rh <= r_g(0)(15 downto 6); end if;
                            s_rp <= "01";
                        when "01" =>
                            -- X, Y, Spin and Drift: a detent at the centre
                            if (s_ri(2 downto 1) = "00" or s_ri = 3 or s_ri = 4) and s_rh > 512 - C_DB and s_rh < 512 + C_DB then
                                v_t := to_unsigned(32768, 16);
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
                            for i in 0 to 5 loop
                                r_g(i) <= r_g(i + 1);
                            end loop;
                            r_g(6) <= v_gn;
                            s_ri <= s_ri + 1;
                            s_rp <= "00";
                    end case;
                end if;

                case to_integer(s_seq) is
                    -- the disc: cx = W/2 + (K1 - 512) Wv / 1024 (serial) ; DRIFT (K5, Hypnos v1.1.0 shape):
                    -- bipolar, |v| < 24 counts = still; exponential, 2^-12 .. 2^-2 cycle per field
                    -- (piecewise-linear mantissa 1 + f/64, octave = ls / 64)
                    when 228 =>
                        sm_a <= resize(signed((not r_g(0)(15)) & r_g(0)(14 downto 3)), 32);   -- (g - 32768) >> 3
                        case s_mcls is
                            when "10"   => sm_b <= to_unsigned(960, 16);
                            when "01"   => sm_b <= to_unsigned(1280, 16);
                            when others => sm_b <= to_unsigned(1920, 16);
                        end case;
                        sm_acc <= (others => '0'); sm_n <= to_unsigned(16, 5);
                        s_v9 <= shift_left(resize(r_g(2), 20), 3) + resize(r_g(2), 20);   -- 9 g3 (20 bits: 9 x 65472)
                        s_vneg <= not r_g(4)(15);
                        if r_g(4)(15) = '1' then s_av <= '0' & r_g(4)(14 downto 6);
                        else                     s_av <= '0' & not r_g(4)(14 downto 6); end if;
                    -- PERIOD: log2 P = 2107 - 9 g3 / 512 (Q8, virtual px: 300 -> 13.3) - log2(x step)
                    when 227 =>
                        s_lp <= to_unsigned(2107, 12) - resize(s_v9(19 downto 9), 12) - resize(s_ls8, 12);
                        v_u11 := resize(s_av, 11) - 24;              -- u = |v| - 24; the borrow is the dead zone
                        s_vzero <= v_u11(10);
                        s_u <= v_u11(9 downto 0);
                    when 226 =>
                        s_nlp14 <= to_signed(0, 21) - signed(resize(s_lp & "000000", 21));
                        t2_a <= (not s_lp(7 downto 0)) + 1;                                 -- (-j) & 255
                        if s_lp(7 downto 0) = 0 then s_ex <= s_lp(11 downto 8);
                        else s_ex <= s_lp(11 downto 8) + 1; end if;
                        s_ls <= s_u + shift_right(s_u, 2) + shift_right(s_u, 4);
                    -- (225: the drift shifter loads, above)
                    when 225 =>
                        s_nlp14x <= s_nlp14 - 32768;
                        s_kbt    <= shift_right(s_nlp14, 1);
                    -- 1/P = 2^((-j & 255) / 256) / 2^ex  (Q30): the table value, then 15 - ex serial shifts
                    when 223 =>
                        s_inv <= resize(unsigned(t2_r), 30);
                        s_isc <= to_unsigned(15, 4) - s_ex;
                    -- the phase accumulator (one cycle = 2^20, wraps forever without a seam) ; duties ;
                    -- offsets centre each ink band on the integer levels (psi = 0: the body and the axis).
                    -- Dashes: the drift moves phi (the dashes run along the streamlines); Lines: psi creeps.
                    when 212 =>
                        s_anim <= s_anim + s_spd;
                        if s_fine = '1' then s_Dps <= to_unsigned(10923, 16); s_Dph <= to_unsigned(2731, 16);
                        else                 s_Dps <= to_unsigned(32768, 16); s_Dph <= to_unsigned(16384, 16); end if;
                    when 211 =>
                        if s_lines = '1' then v_d16 := s_Dps; else v_d16 := s_Dph; end if;
                        v_d16 := f_addsub(shift_right(v_d16, 1), s_anim(19 downto 4), '1');
                        if s_lines = '1' then s_offps <= v_d16; s_offph <= shift_right(s_Dph, 1);
                        else                  s_offps <= shift_right(s_Dps, 1); s_offph <= v_d16; end if;
                    when 209 =>
                        s_cx <= signed(resize(s_W(11 downto 1), 14)) + resize(shift_right(sm_acc, 13), 14);
                        sm_a <= resize(signed((not r_g(1)(15)) & r_g(1)(14 downto 3)), 32); sm_b <= resize(s_H, 16);
                        sm_acc <= (others => '0'); sm_n <= to_unsigned(16, 5);
                    when 208 =>
                        s_ninv <= not signed(resize(s_inv, 32));              -- -inv - 1 (one's complement)
                    -- the DDA start: x/4P at pixel -2 (+ the phi offset; the accumulator starts at it)
                    when 190 =>
                        s_cy <= signed(resize(s_H(11 downto 1), 14)) + resize(shift_right(sm_acc, 13), 14);
                        -- (cx + 2) x (-1/P - 1), or |cx + 2| x 1/P when the centre is off the left edge (SD)
                        v_c2 := resize(s_cx, 15) + 2;
                        if v_c2(14) = '1' then sm_a <= signed(resize(s_inv, 32)); v_c2 := to_signed(0, 15) - v_c2;
                        else sm_a <= s_ninv; end if;
                        sm_b <= unsigned(resize(v_c2, 16)); sm_acc <= signed(shift_left(resize(s_offph, 32), 16)); sm_n <= to_unsigned(16, 5);
                    when 171 =>
                        s_u0 <= unsigned(sm_acc(31 downto 12));
                    -- BODY: log2 R = 4.32 + 3.17 t (t < 1/2) or 7.49 + 1.47 (t - 1/2) octaves (virtual px)
                    when 133 =>
                        s_lrhi <= r_g(6)(15);
                        if r_g(6)(15) = '0' then sm_a <= signed(resize(r_g(6), 32)); sm_b <= to_unsigned(3246, 16);
                        else sm_a <= signed(resize(r_g(6)(14 downto 0), 32)); sm_b <= to_unsigned(1509, 16); end if;
                        sm_acc <= (others => '0'); sm_n <= to_unsigned(16, 5);
                    when 114 =>
                        if s_lrhi = '0' then s_lrv <= to_unsigned(70810, 18) + unsigned(sm_acc(28 downto 11));
                        else s_lrv <= to_unsigned(122748, 18) + unsigned(sm_acc(28 downto 11)); end if;
                    when 113 =>
                        s_LR <= s_lrv - resize(s_ls14, 18);
                    when 112 =>
                        s_lrp <= signed(resize(s_LR(17 downto 10), 11)) + s_nlp14(20 downto 10);
                        s_lkd <= s_LR - to_unsigned(8663, 18);
                        -- SPIN: |t| (Q6 knob) >> 5, its sign
                        if r_g(3)(15) = '1' then v_ab := r_g(3) - to_unsigned(32768, 16); s_sneg <= '0';
                        else v_ab := to_unsigned(32768, 16) - r_g(3); s_sneg <= '1'; end if;
                        if v_ab(15 downto 5) > 1023 then s_sa <= (others => '1'); else s_sa <= v_ab(14 downto 5); end if;
                    -- disc distance scale (ln2 R ~ (4 + f2)/4 2^ed px per octave); log2 |t| (linear mantissa)
                    when 111 =>
                        s_ed <= s_lkd(17 downto 14);
                        s_f2 <= s_lkd(13 downto 12);
                        v_p := 0;
                        for i in 0 to 9 loop
                            if s_sa(i) = '1' then v_p := i; end if;
                        end loop;
                        if s_sa = 0 then s_sz <= '1'; else s_sz <= '0'; end if;
                        v_n9 := shift_left(s_sa, 9 - v_p);
                        s_lt <= to_signed((v_p - 10) * 16, 9) + signed(resize(v_n9(8 downto 5), 9));
                    -- log2 g = log2(2 ln2 1.6) + 2 log2|t| + log2 R - log2 P (in 16ths of an octave)
                    when 110 =>
                        s_lga <= to_signed(C_LG0 / 1024, 11) + shift_left(resize(s_lt, 11), 1);
                    when 109 =>
                        s_lg <= s_lga + s_lrp;
                    -- g = (4 + b)/4 2^e: the psi term is (dL (4 + b)) << e, kept as << 7 then >> (7 - e)
                    when 108 =>
                        v_eg := resize(shift_right(s_lg, 4), 7);
                        if s_sz = '1' or v_eg < -5 then
                            s_sM <= (others => '0'); s_rr <= (others => '0'); s_sneg <= '1';   -- the term exactly 0
                        else
                            s_sM <= '1' & unsigned(s_lg(3 downto 2));
                            if v_eg > 7 then s_rr <= (others => '0');
                            else v_eg := to_signed(7, 7) - v_eg; s_rr <= unsigned(v_eg(3 downto 0)); end if;
                        end if;
                    -- CAMERA gain = K6 75 / 1024 (0 .. 74); the table refill starts at state 98
                    when 100 =>
                        s_g1 <= shift_left(resize(r_g(5)(15 downto 6), 17), 6) + shift_left(resize(r_g(5)(15 downto 6), 17), 3);
                        s_g2 <= shift_left(resize(r_g(5)(15 downto 6), 17), 1) + resize(r_g(5)(15 downto 6), 17);
                    when 99 =>
                        v_u11 := resize(shift_right(s_g1 + s_g2, 10), 11);
                        s_gain <= v_u11(6 downto 0);
                    when 97 => s_init <= '0';
                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;


    ------------------------------------------------------------------------
    -- coordinate accumulators on the GENERATED avid (hypnos): dx in real pixels (it starts at pixel -2
    -- so pixel -1 has a real left neighbour for the gradient, and runs on through the blanking), dy in
    -- x-pixel units.  The DDA x/4P runs 24 clocks behind (stage 24); y/P is loaded per line from the
    -- serial multiplier at the same point.
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            if g_go = '1' then
                if s_newframe = '1' then
                    v_dy0 := shift_left(resize(-s_cy, 23), 8);
                    v_dy0(7) := '1';                                   -- +0.5
                    s_dyq <= v_dy0;
                    s_newframe <= '0';
                else
                    s_dyq <= s_dyq + signed(resize(s_ystp, 23));
                end if;
                s_dx <= not resize(s_cx, 15) - 1;                     -- -cx - 2
            else
                s_dx <= s_dx + 1;
            end if;
            if s_vs_pulse = '1' then s_newframe <= '1'; end if;

            if g_go = '1' then s_lc <= (others => '0');
            elsif s_lc /= 31 then s_lc <= s_lc + 1; end if;
            if s_lc = 23 then
                s_uu <= s_u0;
                s_vv <= unsigned(sm_acc(31 downto 14));
            else
                s_uu <= s_uu + resize(s_inv(29 downto 12), 20);
            end if;
        end if;
    end process p_acc;
    s_dyc <= s_dyq(22 downto 8);

    ------------------------------------------------------------------------
    -- EBR delay lines.  Write address = free-running counter; each read runs X behind it (never the
    -- written address).
    ------------------------------------------------------------------------
    dla_d <= "000000" & qsr(8) & std_logic_vector(s8_ll) & std_logic_vector(s8_lh);
    dlb_d <= "000000000" & s19_kb & fsr(19) & std_logic_vector(s19_sp);
    dlc_d <= "00000000000000" & std_logic_vector(s26_phi) & std_logic_vector(s26_psi);
    dld_d <= "000000" & std_logic_vector(s20_t2) & std_logic_vector(s20_t1);
    dls_d <= x"000" & g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_wa <= s_wa + 1;
            dla(to_integer(s_wa)) <= dla_d;
            dla_q <= dla(to_integer(s_wa - C_XA));
            dlb(to_integer(s_wa)) <= dlb_d;
            dlb_q <= dlb(to_integer(s_wa - C_XB));
            dlc(to_integer(s_wa)) <= dlc_d;
            dlc_q <= dlc(to_integer(s_wa - C_XC));
            dld(to_integer(s_wa)) <= dld_d;
            dld_q <= dld(to_integer(s_wa - C_XD));
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
            exm_q(0) <= C_EXM(to_integer(s18_xa(0)));
            exs_q(0) <= C_EXS(to_integer(s18_xa(0)));
            exm_q(1) <= C_EXM(to_integer(s18_xa(1)));
            t2_q  <= C_T2(to_integer(t2_a));
            t2_r  <= t2_q;
        end if;
    end process p_rom;

    ------------------------------------------------------------------------
    -- [D] SOURCE FIELD (kinetic riley): luma or the Osc lattice -> x K6 (a table refilled every field:
    -- (level - 16) * gain >> 5, Osc level * 2 gain >> 5) -> 32-px box, edge-replicated and reloaded at
    -- every line start -> vertical 1/16 IIR on every 8th column -> linear interpolation back to pixel
    -- rate.  The result s_cv is the bend in Q16 psi cycles (white at 100 %: 2 periods; Osc up to 4); it
    -- is tapped unaligned (the camera is never shown).
    ------------------------------------------------------------------------
    bx_d  <= "000000" & std_logic_vector(s_sl);
    wt_d  <= x"03FF" when wt_acc(15) = '1' else "000000" & std_logic_vector(wt_acc(14 downto 5));
    wt_ra <= s_osum when s_src = '1' else unsigned(data_in.y(9 downto 2));
    p_src : process(clk)
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(9 downto 0);
        variable v_d   : signed(17 downto 0);
        variable v_a   : signed(17 downto 0);
        variable v_x, v_y : unsigned(6 downto 0);
        variable v_g   : unsigned(7 downto 0);
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
            -- (max(i - 16, 0) * gain) >> 5, by accumulation; Osc: (i * 2 gain) >> 5; clamped at 1023
            if s_src = '1' then v_g := s_gain & '0'; else v_g := '0' & s_gain; end if;
            if s_seq = 98 then
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

            -- vertical IIR on every 8th column: q += (box - q + 8) >> 4
            s_eb <= s_bsum; s_eq <= unsigned(lb_q);
            if s_bv = '1' and s_bcol(2 downto 0) = "111" then s_ev <= '1'; else s_ev <= '0'; end if;
            s_ecol <= s_bcol;
            v_d := signed(resize(s_eb, 18)) - signed(resize(s_eq, 18)) + 8;
            v_a := signed(resize(s_eq, 18)) + shift_right(v_d, 4);
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
            s1_a <= va(12 downto 0);          -- the disc stays within the picture's reach: no clamp
            s1_b <= vb(12 downto 0);
            s1_dxs <= s_dx(14);
            s1_dys <= s_dyc(14);
            -- S2
            if s1_a >= s1_b then s2_hi <= s1_a; s2_lo <= s1_b; s2_sw <= '0';
            else                 s2_hi <= s1_b; s2_lo <= s1_a; s2_sw <= '1'; end if;
            if s1_a = 0 or s1_b = 0 then s2_lz <= '1'; else s2_lz <= '0'; end if;
            if s1_a = 0 then s2_xz <= '1'; else s2_xz <= '0'; end if;
            if s1_b = 0 then s2_yz <= '1'; else s2_yz <= '0'; end if;
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

            qsr(3) <= s2_sw & s2_dxs & s2_dys & s2_lz & s2_xz & s2_yz;
            for i in 4 to 8 loop qsr(i) <= qsr(i - 1); end loop;
        end if;
    end process p_logs;



    ------------------------------------------------------------------------
    -- S9..S15: u = log2(hi/lo) ; log2 r = log2 hi + 0.5 log2(1 + 4^-u) (EBR table, interpolated)
    -- (hypnos) ; log2|y| - log2 P and log2|x| - log2 4P from the delay line (the swap picks which is which)
    ------------------------------------------------------------------------
    p_polar : process(clk)
        variable vu   : unsigned(17 downto 0);
        variable q    : std_logic_vector(5 downto 0);
        variable vlh, vll, vly, vlx : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- S9
            vu := s8_lh - s8_ll;
            if qsr(8)(2) = '1' then vu := to_unsigned(16 * 16384 - 1, 18); end if;
            s9_ua <= vu;
            -- S10
            s10_ia <= s9_ua(17 downto 10);
            s10_tu <= s9_ua(9 downto 5);
            -- S11, S12
            s11_tu <= s10_tu;
            s12_tu <= s11_tu;
            s12_g <= ug_q; s12_gs <= ugs_q;
            -- S13: interpolation product (the slope is negative)
            s13_gp <= resize(bmul(signed(s12_gs(10 downto 0)), signed('0' & s12_tu)), 22);
            s13_g  <= unsigned(s12_g);
            -- S14
            s14_g <= resize(unsigned(signed(resize(s13_g, 17)) + resize(shift_right(s13_gp, 5), 17)), 14);
            -- S15
            vlh := unsigned(dla_q(17 downto 0));
            vll := unsigned(dla_q(35 downto 18));
            q := dla_q(41 downto 36);                 -- sw, dxs, dys, lz, xz, yz
            s15_lr <= vlh + resize(s14_g, 18);
            if q(5) = '1' then vlx := vll; vly := vlh; else vlx := vlh; vly := vll; end if;
            s15_ey <= signed(resize(vly, 21)) + s_nlp14;
            s15_ex <= signed(resize(vlx, 21)) + s_nlp14x;
            s15_f <= q(4) & q(3) & q(1) & q(0);       -- xs, ys, xz, yz
        end if;
    end process p_polar;

    ------------------------------------------------------------------------
    -- S16..S20: dL = log2 r - log2 R ; the exp2 exponents E = log2|y| (|x|) - log2 P (4P) - 2 dL, with
    -- k held at 1 inside a Solid disc ; kbig (Open: k/P past a cycle per px) ; the spin term (dL M) << e
    -- (one's complement for s > 0) ; the disc distance d = ln2 R dL (Q4 px) and its two blend weights
    ------------------------------------------------------------------------
    p_field : process(clk)
        variable v_dl  : signed(19 downto 0);
        variable v_s   : signed(29 downto 0);
        variable v_t   : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S16
            s16_dl <= signed(resize(s15_lr, 20)) - signed(resize(s_LR, 20));
            s16_ey <= s15_ey; s16_ex <= s15_ex;
            fsr(16) <= s15_f;
            -- S17
            if s_solid = '1' and s16_dl(19) = '1' then v_dl := (others => '0'); else v_dl := s16_dl; end if;
            s17_ey <= s16_ey - shift_left(resize(v_dl, 21), 1);
            s17_ex <= s16_ex - shift_left(resize(v_dl, 21), 1);
            if s_solid = '0' and s16_dl(19 downto 8) <= s_kbt(20 downto 8) then s17_kb <= '1'; else s17_kb <= '0'; end if;
            if s_sM(2) = '1' then s17_sp <= f_mul47(s16_dl(19 downto 2), s_sM(1 downto 0)); else s17_sp <= (others => '0'); end if;
            case to_integer(s_ed) is                                          -- dL >> (10 - ed)
                when 0 | 1 | 2 => s17_dsh <= shift_right(s16_dl, 8);
                when 3      => s17_dsh <= shift_right(s16_dl, 7);
                when 4      => s17_dsh <= shift_right(s16_dl, 6);
                when 5      => s17_dsh <= shift_right(s16_dl, 5);
                when 6      => s17_dsh <= shift_right(s16_dl, 4);
                when 7      => s17_dsh <= shift_right(s16_dl, 3);
                when others => s17_dsh <= shift_right(s16_dl, 2);
            end case;
            fsr(17) <= fsr(16);
            -- S18: spin, coarse shift (right by 4 (rr / 4), after << 7) ; disc (4 + f2) dL
            v_s := shift_left(resize(s17_sp, 30), 9);
            s18_sp <= shift_right(v_s, to_integer(s_rr(3 downto 2) & "00"));
            s18_kb <= s17_kb;
            if s17_dsh > 127 then s18_ds <= to_signed(127, 9);
            elsif s17_dsh < -16 then s18_ds <= to_signed(-16, 9);
            else s18_ds <= s17_dsh(8 downto 0); end if;
            fsr(18) <= fsr(17);
            -- S19: spin, fine shift and the sign ; disc distance
            v_s := shift_right(s18_sp, to_integer(s_rr(1 downto 0)));
            if s_sneg = '1' then s19_sp <= unsigned(v_s(17 downto 0));
            else                 s19_sp <= unsigned(not v_s(17 downto 0)); end if;
            s19_kb <= s18_kb;
            s19_dp <= f_mul47(s18_ds, s_f2);
            fsr(19) <= fsr(18);
            -- S20: d = s19_dp / 4 (Q4 px); white weight (disc edge at d = 0) and figure weight (the ring's
            -- outer edge at d = rw), in 16ths
            v_t := to_signed(8, 10) - resize(shift_right(s19_dp, 2), 10);
            if v_t < 0 then s20_t1 <= (others => '0'); elsif v_t > 16 then s20_t1 <= to_unsigned(16, 5);
            else s20_t1 <= unsigned(v_t(4 downto 0)); end if;
            v_t := resize(shift_right(s19_dp, 2), 10) - signed(resize(s_rw, 10)) + 8;
            if v_t < 0 then s20_t2 <= (others => '0'); elsif v_t > 16 then s20_t2 <= to_unsigned(16, 5);
            else s20_t2 <= unsigned(v_t(4 downto 0)); end if;
        end if;
    end process p_field;

    ------------------------------------------------------------------------
    -- S18..S24: the two exp2 chains (multipole's phase chain): 2^E = mantissa (EBR PWL) << integer part,
    -- as a cycle count Q16: y k / P mod 4 cycles (18 bits), x k / 4P mod 1 (16 bits, table steps only)
    ------------------------------------------------------------------------
    p_exp : process(clk)
        variable v_e   : signed(20 downto 0);
        variable v_k   : signed(8 downto 0);
        variable v_sh  : unsigned(5 downto 0);
        variable v_w48 : unsigned(47 downto 0);
        variable v_w24 : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            for c in 0 to 1 loop
                -- S18
                if c = 0 then v_e := s17_ey; else v_e := s17_ex; end if;
                s18_xa(c) <= unsigned(v_e(13 downto 6));
                s18_t(c)  <= unsigned(v_e(5 downto 0));
                v_k := resize(shift_right(v_e, 14), 9) + 2;
                if v_k < -16 then v_k := to_signed(-16, 9);
                elsif v_k > 16 then v_k := to_signed(16, 9); end if;
                s18_k(c) <= v_k(5 downto 0);
                -- S19 (EBR read)
                s19_t(c) <= s18_t(c); s19_k(c) <= s18_k(c);
                -- S20 (fabric register)
                s20_m(c) <= unsigned(exm_q(c));
                if c = 0 then s20_s(c) <= unsigned(exs_q(c)(7 downto 0)); end if;
                s20_t(c) <= s19_t(c); s20_k(c) <= s19_k(c);
                -- S21 (the x chain, the dash cuts only, runs without interpolation)
                if c = 0 then s21_ep(c) <= resize(s20_s(c) * s20_t(c)(5 downto 2), 14); end if;
                s21_m(c)  <= s20_m(c); s21_k(c) <= s20_k(c);
                -- S22: the mantissa, Q14
                if c = 0 then s22_mt(c) <= s21_m(c) + resize(shift_right(s21_ep(c), 4), 16);
                else          s22_mt(c) <= s21_m(c); end if;
                s22_k(c)  <= s21_k(c);
                -- S23, S24: (mantissa << (k + 16)) bits 33 .. 16, in two steps
                v_sh := unsigned(resize(s22_k(c), 6) + 16);
                v_w48 := shift_left(resize(s22_mt(c), 48), to_integer(v_sh(5 downto 2) & "00"));
                s23_y1(c) <= v_w48(33 downto 13);
                s23_f(c)  <= v_sh(1 downto 0);
            end loop;
            -- S24
            v_w24 := shift_left(resize(s23_y1(0), 24), to_integer(s23_f(0)));
            s24_my <= v_w24(20 downto 3);
            v_w24 := shift_left(resize(s23_y1(1), 24), to_integer(s23_f(1)));
            s24_mx <= v_w24(18 downto 3);
        end if;
    end process p_exp;

    ------------------------------------------------------------------------
    -- S25..S33: psi (18 bits, mod 4 cycles) and phi (16 bits) ; their horizontal differences (the
    -- gradient of an analytic W: |W'| = |dpsi/dx + i dphi/dx|, phi scaled x4 to psi cycles) ; the L1
    -- norm ; H8 = log2 of it in eighths of an octave (3-bit linear mantissa, less log2(4/3) ~ 3/8) ;
    -- the Weight knee (Bold: knee at 1/4 cycle per px, flat at 1/2; Fine: knee at 1/12, a 2-px line),
    -- rounded UP to a half octave so the divide is a shift (hypnos)
    ------------------------------------------------------------------------
    p_grad : process(clk)
        variable v_yk  : unsigned(17 downto 0);
        variable v_xk  : unsigned(15 downto 0);
        variable v_d18 : unsigned(17 downto 0);
        variable v_d16 : unsigned(15 downto 0);
        variable v_p   : integer range 0 to 18;
        variable v_n   : unsigned(18 downto 0);
        variable v_v   : signed(7 downto 0);
        variable v_sh  : integer range -8 to 15;
        variable v_w   : unsigned(15 downto 0);
        variable v_m   : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- S25 (the DDA, y/P and the B line are stage-24 values; B = spin, xs ys xz yz, kbig)
            -- psi = y/P - y k/P: y > 0 subtracts (one's complement); phi = x/4P + x k/4P
            if dlb_q(18) = '1' then v_yk := (others => '0');
            elsif dlb_q(20) = '0' then v_yk := not s24_my;
            else v_yk := s24_my; end if;
            if dlb_q(19) = '1' then v_xk := (others => '0');
            elsif dlb_q(21) = '1' then v_xk := not s24_mx;
            else v_xk := s24_mx; end if;
            s25_a1  <= s_vv + v_yk;
            s25_a2  <= unsigned(dlb_q(17 downto 0)) + s_cv(17 downto 0);
            s25_phi <= s_uu(19 downto 4) + v_xk;
            ksr(24) <= dlb_q(22);
            for i in 25 to 31 loop ksr(i) <= ksr(i - 1); end loop;
            -- S26
            s26_psi <= s25_a1 + s25_a2;
            s26_phi <= s25_phi;
            -- S27: differences from the previous pixel (s27_pp / s27_fp hold it), magnitudes (one's complement)
            s27_pp <= s26_psi; s27_fp <= s26_phi;
            v_d18 := s26_psi - s27_pp;
            if v_d18(17) = '1' then s27_a <= not v_d18; else s27_a <= v_d18; end if;
            v_d16 := s26_phi - s27_fp;
            if v_d16(15) = '1' then s27_b <= shift_left(resize(not v_d16, 18), 2);
            else s27_b <= shift_left(resize(v_d16, 18), 2); end if;
            -- S28: L1 norm
            s28_g <= resize(s27_a, 19) + resize(s27_b, 19);
            -- S29: leading one, the 3 bits below it
            v_p := 0;
            for i in 0 to 18 loop
                if s28_g(i) = '1' then v_p := i; end if;
            end loop;
            v_n := shift_left(s28_g, 18 - v_p);
            s29_p <= to_unsigned(v_p, 5);
            s29_m <= v_n(17 downto 15);
            if s28_g = 0 then s29_nz <= '0'; else s29_nz <= '1'; end if;
            -- S30: H8 = 8 (p - 16) + m - 3
            if s29_nz = '0' then s30_h <= to_signed(-255, 9);
            else s30_h <= shift_left(signed(resize(s29_p, 9)), 3) + signed(resize(s29_m, 9)) - 131; end if;
            -- S31: the two candidates, already rounded up to a half octave: (H8 + 3) / 4 and (2 H8 + K + 3) / 4
            -- (Bold K = 16, knee above H8 = -16; Fine K = 29, knee above H8 = -29)
            v_v := resize(shift_right(s30_h + 3, 2), 8);
            if s30_h < -127 then v_v := to_signed(-32, 8); end if;
            s31_c0 <= v_v;
            if s_fine = '1' then
                s31_c1 <= resize(shift_right(shift_left(resize(s30_h, 10), 1) + 32, 2), 8);
                if s30_h > -29 then s31_fl <= '1'; else s31_fl <= '0'; end if;
            else
                s31_c1 <= resize(shift_right(shift_left(resize(s30_h, 10), 1) + 19, 2), 8);
                if s30_h > -16 then s31_fl <= '1'; else s31_fl <= '0'; end if;
            end if;
            -- S32: select ; flat at >= 0 (or kbig) ; the reciprocal 2^-log2 w = 2^ri * {1, sqrt 2}
            if s31_fl = '1' then v_v := s31_c1; else v_v := s31_c0; end if;
            if v_v >= 0 or ksr(31) = '1' then
                s32_sat <= '1'; s32_ri <= (others => '0'); s32_rh <= '0';
            else
                s32_sat <= '0';
                v_v := to_signed(0, 8) - v_v;
                if v_v > 31 then v_v := to_signed(31, 8); end if;
                s32_ri <= unsigned(v_v(4 downto 1));
                s32_rh <= v_v(0);
            end if;
            -- S33: half the filter width, w / 2 (Q16): w = {64, 91} << (10 - ri - rh); the dash chain
            -- runs at a quarter of it (phi cycles are four psi cycles long) unless flat
            if s32_sat = '1' then
                v_w := (others => '1');
            else
                v_sh := 10 - to_integer(s32_ri);
                if s32_rh = '1' then v_m := to_unsigned(91, 7); v_sh := v_sh - 1;
                else                 v_m := to_unsigned(64, 7); end if;
                if v_sh >= 0 then v_w := resize(shift_left(resize(v_m, 23), v_sh), 16);
                else              v_w := resize(shift_right(resize(v_m, 16), -v_sh), 16); end if;
                if v_w = 0 then v_w := to_unsigned(1, 16); end if;
            end if;
            s33_wh <= v_w(15 downto 1);
            if s32_sat = '1' then s33_whf <= v_w(15 downto 1); else s33_whf <= resize(v_w(15 downto 3), 15); end if;
            s33_ri <= s32_ri; s33_rh <= s32_rh; s33_sat <= s32_sat;
            s34_ri <= s33_ri; s34_rh <= s33_rh; s34_sat <= s33_sat;
            s35_ri <= s34_ri; s35_rh <= s34_rh; s35_sat <= s34_sat;
            s36_ri <= s35_ri; s36_rh <= s35_rh; s36_sat <= s35_sat;
            s37_rh <= s36_rh;
        end if;
    end process p_grad;

    ------------------------------------------------------------------------
    -- S34..S39: one-pixel box filter of the band square wave (hypnos), psi (0) and phi (1); the
    -- phases come out of the C line as stage-33 values.
    --   ink = frac in [0, D) ; P(x) = floor(x) D + min(frac x, D) ; coverage = (P(ph + w/2) - P(ph - w/2)) / w
    ------------------------------------------------------------------------
    p_cov : process(clk)
        variable vn  : signed(17 downto 0);
        variable vs  : unsigned(24 downto 0);
        variable vc  : unsigned(9 downto 0);
        variable vph : unsigned(15 downto 0);
        variable vwh : unsigned(14 downto 0);
        variable vD  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            for c in 0 to 1 loop
                if c = 0 then vD := s_Dps; else vD := s_Dph; end if;
                -- S34
                if c = 0 then vph := unsigned(dlc_q(15 downto 0)); vwh := s33_wh;
                else          vph := unsigned(dlc_q(33 downto 18)); vwh := s33_whf; end if;
                s34_lo(c) <= vph - resize(vwh, 16);
                s34_hi(c) <= vph + resize(vwh, 16);
                -- S35
                if s34_hi(c) < s34_lo(c) then s35_k(c) <= '1'; else s35_k(c) <= '0'; end if;
                if s34_hi(c) >= s34_lo(c) and s34_hi(c) < vD then s35_full(c) <= '1'; else s35_full(c) <= '0'; end if;
                if s34_hi(c) < vD then s35_mh(c) <= s34_hi(c); else s35_mh(c) <= vD; end if;
                if s34_lo(c) < vD then s35_ml(c) <= s34_lo(c); else s35_ml(c) <= vD; end if;
                -- S36
                vn := signed(resize(s35_mh(c), 18)) - signed(resize(s35_ml(c), 18));
                if s35_k(c) = '1' then vn := vn + signed(resize(vD, 18)); end if;
                if vn < 0 then vn := (others => '0'); end if;
                s36_n(c) <= unsigned(vn(15 downto 0));
                s36_full(c) <= s35_full(c);
                -- S37: (N << ri) >> 7 (phi, unless flat: >> 5)
                vs := shift_left(resize(s36_n(c), 25), to_integer(s36_ri));
                if c = 1 and s36_sat = '0' then s37_nsh(c) <= vs(14 downto 5);
                else s37_nsh(c) <= vs(16 downto 7); end if;
                s37_full(c) <= s36_full(c);
                -- S38: / 2, or * (about) sqrt2 / 2 on a half-octave step
                if s37_rh = '1' then
                    s38_cp(c) <= resize(shift_right(s37_nsh(c), 1) + shift_right(s37_nsh(c), 2), 11);
                else
                    s38_cp(c) <= resize(shift_right(s37_nsh(c), 1), 11);
                end if;
                s38_full(c) <= s37_full(c);
                -- S39: coverage 0..256
                vc := resize(s38_cp(c), 10);
                if vc > 256 or s38_full(c) = '1' then vc := to_unsigned(256, 10); end if;
                s39_cov(c) <= resize(vc, 9);
            end loop;
        end if;
    end process p_cov;

    ------------------------------------------------------------------------
    -- S40..S47: weave (Lines: psi ; Dashes Bold: XOR = |a - b| ; Dashes Fine: OR = max(a, b)), the
    -- Solid disc (white inside d = 0, black ring to d = rw, the figure beyond: max(min(c, figure
    -- weight), white weight)), luma 64 .. 940, then the output conditioning: [1 2 1]/4, luma 13/16
    -- toward black (white 775), blanking gate.  U = V = 512 always.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_s  : unsigned(18 downto 0);
        variable v_f  : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
        variable v_c  : unsigned(8 downto 0);
        variable v_w  : unsigned(8 downto 0);
        variable v_n10 : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S40
            s40_d  <= signed(resize(s39_cov(0), 10)) - signed(resize(s39_cov(1), 10));
            if s39_cov(0) >= s39_cov(1) then s40_ge <= '1'; else s40_ge <= '0'; end if;
            s40_a  <= s39_cov(0);
            s40_b  <= s39_cov(1);
            -- S41
            if s_lines = '1' then s41_c <= s40_a;
            elsif s_fine = '1' then
                if s40_ge = '1' then s41_c <= s40_a; else s41_c <= s40_b; end if;
            elsif s40_d(9) = '1' then v_n10 := to_signed(0, 10) - s40_d; s41_c <= unsigned(v_n10(8 downto 0));
            else s41_c <= unsigned(s40_d(8 downto 0)); end if;
            -- S42: the disc (weights from the D line, stage 41)
            v_c := s41_c;
            v_w := shift_left(resize(unsigned(dld_q(9 downto 5)), 9), 4);
            if v_w < v_c then v_c := v_w; end if;
            v_w := shift_left(resize(unsigned(dld_q(4 downto 0)), 9), 4);
            if v_w > v_c then v_c := v_w; end if;
            if s_solid = '1' then s42_c <= v_c; else s42_c <= s41_c; end if;
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

end architecture streamline;
