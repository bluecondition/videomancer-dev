-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- kinegram.vhd  (Hardline, v0.2.0 -- a barrier-grid animation that plays itself)
--
-- v0.2.0 (2026-11-16): P12 is SPEED (Ron: the set may move by itself; hypnos's Speed is
-- the model).  The barrier sheet travels continuously, so the N frames play forward (up)
-- or backward (down); centre +-24 counts = stopped, landing on the nearest clean frame.
-- 1/2048 .. 1/2 sliver per field, exponential; the position wraps modulo one barrier
-- period (N slivers).  The v0.1.0 lean (the sheet turning near the ends of the slide)
-- is gone: with a speed the ends are the fastest animation.
--
-- Scanimation / kinegram: a picture made of N interleaved slivers of N animation
-- frames, seen through a sheet of black bars with clear slits.  Moving the sheet
-- one sliver steps the visible frame.
--
--   lattice   vertical slivers of width w (K5 Pitch), anchored on the screen centre;
--             sliver index k = floor(x / w) mod N  (K4 Frames, N = 2 .. 8)
--   figure    Rings:  frac(r / Period - k / N) < 1/2        (K3 Period)
--             Spokes: frac(arms theta / 2 pi - k / N) < 1/2 (K3 = arm count 1 .. 16)
--             centre on K1 / K2 (+-2 screens)
--   barrier   period N w, slit one sliver wide (S10 Fine: half a sliver), positioned by
--             the P12 SPEED accumulator plus the camera: box-filtered luma * K6 shifts
--             the barrier by up to one period (S11 Osc: a slow triangle lattice at 4x)
--   output    figure AND slit (S9 Inverse: figure AND NOT slit), luma only
--
-- The figure comes from the log-domain polar engine of hypnos v1.0.4 (log2 hi / lo
-- -> log2 r and theta from interpolated EBR tables, exp2 mantissa x frequency
-- mantissa, analytic-gradient one-pixel box filter), with the shapes, warp, spiral
-- and palettes removed.  The lattice and the barrier are two free-running DDAs in
-- REAL pixels (Q.10), so the slit edges are a one-pixel box filter with no divide;
-- the camera shift is a table (L * gain mod P) refilled every field, so the pixel
-- path has one small multiplier (figure coverage x slit coverage).
--
-- Stage map (stage 0 = s_dx; pixel i is at stage S at clock T0 + i + S):
--    1 ..  8  |dx| |dy| -> hi lo -> log2 (Q14)
--    9 .. 15  u = log2(hi / lo) -> log2 r, theta
--   16        log2 r in 1080-line units, arms * theta, angular gradient
--   17 .. 24  exp2 -> ring phase;  21 .. 25  filter width
--   20 .. 25  barrier DDA - camera shift -> slit coverage (7 bits), sliver choice
--   22 .. 25  lattice DDA -> k / N of a cycle, subtracted at 26
--   27 .. 32  one-pixel box filter of the band -> coverage 0 .. 256
--   33 .. 34  min(coverage, slit) -> luma 64 .. 776 (the 13/16 conditioning is folded
--             into the scale: 64 + 712 c)
--   35 .. 38  luma [1 2 1]/4, blanking gate.  Latency 39 clocks in every mode.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.kinegram_pkg.all;

architecture kinegram of program_top is
    constant C_PROCESSING_DELAY_CLKS : integer := 39;    -- = processing_delay_clks in the TOML

    constant C_DB  : integer := 14;                 -- centre-knob detent deadband
    constant C_GL  : integer := 4;                  -- centre glide one-pole shift (fields)

    -- TOML defaults: the knob glides start there
    constant C_DEF_K3  : integer := 636;
    constant C_DEF_K5  : integer := 341;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 862;

    -- An EBR delay line written from stage W and read (straight from the EBR
    -- output register) by the logic building stage C runs its read address
    -- X = C - 2 - W behind the write address.
    constant C_XH : integer := 15 - 2 - 8;     -- log2 hi + flags     8 -> 15
    constant C_XS : integer := 25 - 2 - 16;    -- spoke phase        16 -> 25
    constant C_XB : integer := 21 - 2 - 16;    -- angular gradient   16 -> 21
    constant C_XE : integer := 33 - 2 - 25;    -- slit coverage      25 -> 33
    constant C_XA : integer := 33;             -- syncs: dls_q = the generated avid of stage 33

    attribute no_rw_check : boolean;

    ----------------------------------------------------------------------
    -- frame-latched controls + per-field terms
    ----------------------------------------------------------------------
    signal s_kab    : unsigned(9 downto 0) := (others => '0');       -- |K - 512| past the detent
    signal s_kng : std_logic := '0';
    signal s_glide  : std_logic := '1';
    signal s_spk, s_inv, s_fine, s_src : std_logic := '0';
    signal s_nz     : unsigned(2 downto 0) := to_unsigned(2, 3);     -- frames - 2
    signal s_az     : unsigned(3 downto 0) := to_unsigned(9, 4);     -- arms - 1

    -- knob channels, Q6: K1 X, K2 Y, K3 Period, K5 Pitch, K6 Camera, P12 Slide
    signal s_a1 : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_a2 : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_a3 : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);
    signal s_a5 : unsigned(15 downto 0) := to_unsigned(C_DEF_K5 * 64, 16);
    signal s_a6 : unsigned(15 downto 0) := to_unsigned(C_DEF_K6 * 64, 16);
    signal s_ap : unsigned(15 downto 0) := to_unsigned(C_DEF_P12 * 64, 16);
    signal s_gdv : signed(17 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := "000";

    signal s_cx, s_cy   : signed(13 downto 0) := (others => '0');
    signal s_cxs, s_cys : signed(21 downto 0) := (others => '0');   -- centre glide accum Q8
    signal s_tg, s_gd, s_gdd : signed(21 downto 0) := (others => '0');
    signal s_gs, s_gm : std_logic := '0';

    signal s_lfs   : signed(17 downto 0) := to_signed(-29995, 18);  -- log2 cycles / virtual px, Q12
    signal s_lf    : signed(11 downto 0) := to_signed(-469, 12);    -- the same, Q6
    signal s_m     : unsigned(8 downto 0) := to_unsigned(256, 9);   -- freq mantissa Q8
    signal s_kc    : integer range -16 to 0 := -11;                 -- Li - 4 (phase shift base)
    signal s_a0    : signed(11 downto 0) := (others => '0');        -- log2 radial gradient per pixel, Q6
    signal s_arms  : signed(5 downto 0) := to_signed(10, 6);
    signal s_larm  : signed(11 downto 0) := (others => '0');        -- log2 angular gradient base, Q6

    -- pitch: w = 2^lw (real pixels, Q10), from a second copy of the exp2 mantissa table
    signal s_lw    : unsigned(16 downto 0) := (others => '0');
    signal sq_a    : unsigned(7 downto 0) := (others => '0');
    signal sq_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal s_wp     : unsigned(15 downto 0) := to_unsigned(5461, 16);    -- sliver width, px Q10
    signal s_P     : unsigned(18 downto 0) := to_unsigned(21844, 19);   -- barrier period = N w
    signal s_sw    : unsigned(15 downto 0) := to_unsigned(5461, 16);    -- slit width
    signal s_cN    : unsigned(15 downto 0) := to_unsigned(16384, 16);   -- 1/N of a cycle
    signal s_nm1   : unsigned(2 downto 0) := to_unsigned(3, 3);         -- N - 1

    -- slide and lean
    -- SPEED (P12): the barrier travels s_spd slivers per field; s_an = its position,
    -- bit 23 carry / borrow, 22 .. 20 whole slivers (mod N), 19 .. 0 the fraction
    signal s_neg, s_stop : std_logic := '0';
    signal s_uu    : unsigned(8 downto 0) := (others => '0');           -- counts past the deadband
    signal s_L     : unsigned(9 downto 0) := (others => '0');           -- log2 speed + 11, Q6 octaves
    signal s_spd   : unsigned(20 downto 0) := (others => '0');          -- slivers per field, Q20
    signal s_ssh   : unsigned(3 downto 0) := (others => '0');
    signal s_an    : unsigned(23 downto 0) := (others => '0');
    signal s_spx   : unsigned(18 downto 0) := (others => '0');          -- W/2 mod P, then the slide, px Q10
    signal s_G     : unsigned(17 downto 0) := (others => '0');          -- camera shift per luma count, px Q16
    -- line seed (px Q16, mod P) and its small modular unit
    signal s_ls    : signed(26 downto 0) := (others => '0');
    signal mu_op, mu_x : std_logic_vector(1 downto 0) := "00";          -- 01 add, 10 subtract, 11 wrap
    signal mu_y    : unsigned(26 downto 0) := (others => '0');
    signal mu_sel  : std_logic := '0';                                  -- 0 G, 1 s_spx
    -- lattice at pixel 0's left edge
    signal z_ld    : std_logic := '0';
    signal s_k0    : unsigned(2 downto 0) := (others => '0');
    signal s_lp0   : unsigned(15 downto 0) := (others => '0');
    -- camera table fill
    signal tf_i    : unsigned(10 downto 0) := (others => '0');
    signal tf_ph   : std_logic := '0';
    signal tb_we   : std_logic := '0';
    signal tb_wa   : unsigned(9 downto 0) := (others => '0');
    signal tb_wd   : std_logic_vector(15 downto 0) := (others => '0');

    -- vblank sequencer + shared serial multiplier: (a * b) >> 16, 16 clocks
    signal s_seq   : unsigned(5 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_saw_avid   : std_logic := '0';
    signal sm_go   : std_logic := '0';
    signal sm_a    : unsigned(19 downto 0) := (others => '0');
    signal sm_b, sm_sh : unsigned(15 downto 0) := (others => '0');
    signal sm_acc  : unsigned(20 downto 0) := (others => '0');
    signal sm_n    : unsigned(4 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- raster measurement + generated avid
    ----------------------------------------------------------------------
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Wl    : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_ystp  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_mcls  : std_logic_vector(1 downto 0) := "00";
    signal s_lacc  : unsigned(19 downto 0) := (others => '0');
    signal s_ls14  : unsigned(14 downto 0) := (others => '0');      -- log2(x step), Q14
    signal s_ls6   : unsigned(6 downto 0) := (others => '0');       -- the same, Q6
    signal s_avid_q, s_avid_r : std_logic := '0';
    signal s_hs_q, s_hs_r     : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_rsr   : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc    : unsigned(3 downto 0) := "1000";
    signal g_cnt   : unsigned(11 downto 0) := (others => '0');

    -- coordinate accumulators
    signal s_dx, s_dyc : signed(14 downto 0) := (others => '0');
    signal s_dyb      : signed(22 downto 0) := (others => '0');
    signal s_gav_p    : std_logic := '0';
    signal s_newframe : std_logic := '1';
    signal s_pc       : unsigned(4 downto 0) := (others => '1');    -- clocks since T0, saturating

    ----------------------------------------------------------------------
    -- EBR delay lines and tables
    ----------------------------------------------------------------------
    type t_dl32 is array(0 to 255) of std_logic_vector(31 downto 0);
    type t_dl16 is array(0 to 255) of std_logic_vector(15 downto 0);
    signal dlh  : t_dl32;      -- log2 hi + quadrant flags
    signal dlc  : t_dl16;      -- spoke phase
    signal dld  : t_dl16;      -- angular-gradient log
    signal dle  : t_dl16;      -- slit coverage
    signal dls  : t_dl16;      -- syncs
    attribute no_rw_check of dlh : signal is true;
    attribute no_rw_check of dlc : signal is true;
    attribute no_rw_check of dld : signal is true;
    attribute no_rw_check of dle : signal is true;
    attribute no_rw_check of dls : signal is true;
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal dlh_q : std_logic_vector(31 downto 0) := (others => '0');
    signal dlc_q, dld_q, dle_q, dls_q : std_logic_vector(15 downto 0) := (others => '0');
    type t_sy is array(1 to 5) of std_logic_vector(3 downto 0);
    signal s_sy : t_sy := (others => "0111");

    type t_tb is array(0 to 1023) of std_logic_vector(15 downto 0);
    signal tb_ram : t_tb := (others => (others => '0'));       -- camera shift mod P, px Q7
    attribute no_rw_check of tb_ram : signal is true;
    signal tb_q : std_logic_vector(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- source field
    ----------------------------------------------------------------------
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_sl, s_sfirst : unsigned(9 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal bx_ram : t_dl16;
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(14 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb, s_eq : unsigned(9 downto 0) := (others => '0');
    type t_lb is array(0 to 511) of std_logic_vector(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1 : std_logic_vector(15 downto 0) := (others => '0');
    signal lb_wd : unsigned(9 downto 0) := (others => '0');
    signal lb_wa : unsigned(8 downto 0) := (others => '0');
    signal lb_we, s_ldq, s_lfirst : std_logic := '0';
    signal s_la, s_lb : unsigned(9 downto 0) := (others => '0');
    signal s_Lq  : unsigned(11 downto 0) := (others => '0');
    signal s_Ldd : signed(11 downto 0) := (others => '0');
    signal s_lj  : unsigned(1 downto 0) := "00";

    ----------------------------------------------------------------------
    -- pixel pipeline (stage N = N registers after s_dx)
    ----------------------------------------------------------------------
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

    signal s9_ua  : unsigned(17 downto 0) := (others => '0');
    signal s10_ia : unsigned(7 downto 0) := (others => '0');
    signal s10_tu, s11_tu, s12_tu : unsigned(6 downto 0) := (others => '0');
    signal ug_q, ugs_q, ua_q, uas_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s12_g, s12_gs, s12_a, s12_as : std_logic_vector(15 downto 0) := (others => '0');
    signal s13_gp, s13_ap : signed(21 downto 0) := (others => '0');
    signal s13_g, s13_a : unsigned(15 downto 0) := (others => '0');
    signal s14_g   : unsigned(13 downto 0) := (others => '0');
    signal s14_a   : unsigned(15 downto 0) := (others => '0');
    signal s15_lr  : unsigned(17 downto 0) := (others => '0');   -- log2 r (pixels), Q14
    signal s15_th  : unsigned(17 downto 0) := (others => '0');   -- polar angle, 2^18 / turn

    signal s16_e   : unsigned(18 downto 0) := (others => '0');   -- log2 r (1080-line units), Q14
    signal s16_sp  : unsigned(15 downto 0) := (others => '0');   -- arms * theta, Q16 cycles
    signal s16_b   : signed(11 downto 0) := (others => '0');     -- log2 angular gradient, Q6
    signal s17_xa  : unsigned(7 downto 0) := (others => '0');
    signal s17_t, s18_t, s19_t : unsigned(5 downto 0) := (others => '0');
    signal s17_k, s18_k, s19_k, s20_k, s21_k, s22_k, s23_k : integer range -17 to 7 := 0;
    signal exm_q, exs_q, s19_m, s19_s : std_logic_vector(15 downto 0) := (others => '0');
    signal s20_ep  : unsigned(13 downto 0) := (others => '0');
    signal s20_m   : unsigned(15 downto 0) := (others => '0');
    signal s21_mt  : unsigned(15 downto 0) := (others => '0');
    signal s22_ph, s22_pl : unsigned(16 downto 0) := (others => '0');
    signal s23_pr  : unsigned(24 downto 0) := (others => '0');
    signal s24_phr : unsigned(15 downto 0) := (others => '0');

    signal s21_lg, s22_lg : signed(11 downto 0) := (others => '0');
    signal s22_lwp : std_logic := '0';
    signal s23_lw  : signed(11 downto 0) := (others => '0');
    signal s24_h, s24_sat : std_logic := '0';
    signal s24_iw  : integer range -16 to 0 := -16;
    signal s24_ri, s25_ri, s26_ri, s27_ri, s28_ri, s29_ri : integer range 0 to 15 := 0;
    signal s24_rh, s25_rh, s26_rh, s27_rh, s28_rh, s29_rh, s30_rh : std_logic := '0';
    signal s25_w   : unsigned(15 downto 0) := (others => '1');
    signal s25_phq : unsigned(15 downto 0) := (others => '0');
    signal s25_ko  : unsigned(15 downto 0) := (others => '0');
    signal s26_phi : unsigned(15 downto 0) := (others => '0');
    signal s26_wh  : unsigned(14 downto 0) := (others => '0');

    -- lattice (stage 23 / 24) and barrier (stage 21 .. 25)
    signal s_lpos  : unsigned(15 downto 0) := (others => '0');
    signal s_kcn   : unsigned(2 downto 0) := (others => '0');
    signal s_kox, s_koy : unsigned(15 downto 0) := (others => '0');
    signal s_spos  : unsigned(18 downto 0) := (others => '0');
    signal s20_t   : unsigned(15 downto 0) := (others => '0');
    signal s21_er  : signed(19 downto 0) := (others => '0');
    signal s22_e   : signed(19 downto 0) := (others => '0');
    signal s23_g   : signed(19 downto 0) := (others => '0');
    signal s23_fe, s24_fr : std_logic := '0';
    signal s23_c1, s24_c1, s24_c2 : unsigned(6 downto 0) := (others => '0');
    signal s25_sl  : unsigned(6 downto 0) := (others => '0');

    signal s27_lo, s27_hi : unsigned(15 downto 0) := (others => '0');
    signal s28_k   : std_logic := '0';
    signal s28_full, s29_full, s30_full, s31_full : std_logic := '0';
    signal s28_mh, s28_ml : unsigned(15 downto 0) := (others => '0');
    signal s29_n   : unsigned(15 downto 0) := (others => '0');
    signal s30_nsh : unsigned(9 downto 0) := (others => '0');
    signal s31_cp  : unsigned(10 downto 0) := (others => '0');
    signal s32_cov : unsigned(8 downto 0) := (others => '0');
    signal s33_c   : unsigned(8 downto 0) := (others => '0');
    signal s34_y, s35_y, s36_yo, s36_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s36_ya  : unsigned(11 downto 0) := (others => '0');
    signal s37_y   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal o_y     : unsigned(9 downto 0) := to_unsigned(64, 10);

    constant C_D16 : unsigned(15 downto 0) := to_unsigned(32768, 16);   -- equal bands

    function f_sat(x : unsigned; w : integer) return unsigned is
    begin
        if x > to_unsigned(2**w - 1, x'length) then
            return to_unsigned(2**w - 1, w);
        end if;
        return resize(x, w);
    end function;

    -- a + b or a - b on ONE carry chain (the low column makes the carry-in)
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;

    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- Exact signed product a * m, radix-4 Booth on m (hypnos v1.0.2).
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

    function bmul_u(a : unsigned; m : unsigned) return unsigned is
        constant NP : natural := a'length + m'length;
        variable vp : signed(NP + 1 downto 0);
    begin
        vp := bmul(signed('0' & a), signed('0' & m));
        return unsigned(vp(NP - 1 downto 0));
    end function;

begin

    ------------------------------------------------------------------------
    -- raster measurement + GENERATED AVID (hypnos; cubist v0.3.3 parity lock):
    -- start 2 or 3 clocks after the source's first avid rise, whichever keeps
    -- the hsync -> start parity at its usual value, and run a field-latched width.
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;

            -- per-mode steps (the timing ID is static while a mode runs): s_ystp =
            -- the field-line pitch in x-pixel units (Q.8), the mode class
            -- (00 1080 / 01 720p / 10 SD), the Osc line step and the lean class
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_ystp <= to_unsigned(569, 10); s_mcls <= "10"; s_oys <= to_unsigned(536, 10);  -- 480i
                when 4          => s_ystp <= to_unsigned(288, 10); s_mcls <= "10"; s_oys <= to_unsigned(272, 10);  -- 480p
                when 8          => s_ystp <= to_unsigned(480, 10); s_mcls <= "10"; s_oys <= to_unsigned(452, 10);  -- 576i
                when 12         => s_ystp <= to_unsigned(240, 10); s_mcls <= "10"; s_oys <= to_unsigned(226, 10);  -- 576p
                when 5 | 6 | 14 => s_ystp <= to_unsigned(256, 10); s_mcls <= "01"; s_oys <= to_unsigned(181, 10);  -- 720p
                when 1 | 2 | 10 => s_ystp <= to_unsigned(512, 10); s_mcls <= "00"; s_oys <= to_unsigned(242, 10);  -- 1080i
                when others     => s_ystp <= to_unsigned(256, 10); s_mcls <= "00"; s_oys <= to_unsigned(121, 10);  -- 1080p
            end case;
            -- log2(x step): SD 1 octave, 720p log2(1.5)
            case s_mcls is
                when "10"   => s_ls14 <= to_unsigned(16384, 15); s_ls6 <= to_unsigned(64, 7); s_oxs <= to_unsigned(204, 9);
                when "01"   => s_ls14 <= to_unsigned(9585, 15);  s_ls6 <= to_unsigned(37, 7); s_oxs <= to_unsigned(153, 9);
                when others => s_ls14 <= (others => '0');        s_ls6 <= (others => '0');    s_oxs <= to_unsigned(102, 9);
            end case;

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then                 -- avid just fell
                -- hysteresis: analog line lengths wander +/-1 px
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
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                if g_cnt + 1 >= s_Wl then g_avid <= '0'; end if;
                g_cnt <= g_cnt + 1;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-field control latch + vblank sequencer.  The frame tick is the first
    -- vsync edge after active video (analog vsync serrates).  One small op per
    -- state (vblank cones are timed at the pixel clock); products go through
    -- the serial multiplier and the sequencer waits for it.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_adv  : boolean;
        variable v_gc, v_gdl : signed(21 downto 0);
        variable v_pr   : signed(21 downto 0);
        variable v_wh   : unsigned(11 downto 0);
        variable v_ak, v_am : unsigned(15 downto 0);
        variable v_rm   : unsigned(9 downto 0);
        variable v_k    : signed(10 downto 0);
        variable v_k7   : unsigned(12 downto 0);
        variable v_z7   : unsigned(2 downto 0);
        variable v_f7   : unsigned(9 downto 0);
        variable v_k3   : unsigned(9 downto 0);
        variable v_sh   : integer;
        variable v_av9  : unsigned(8 downto 0);
        variable v_sub  : std_logic;
        variable v_r    : unsigned(26 downto 0);
        variable v_ga   : unsigned(17 downto 0);
        variable v_sb   : unsigned(20 downto 0);
        variable v_lw   : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            sm_go <= '0';
            tb_we <= '0';
            mu_op <= "00";
            z_ld  <= '0';
            sq_q  <= C_EXM(to_integer(sq_a));
            if data_in.avid = '1' then s_saw_avid <= '1'; end if;

            -- the modular unit on the line seed (px Q16, held in [0, P)): one adder, two
            -- clocks per op (the operand is registered first), ops may follow each other
            -- clock by clock.   01 add Y, 10 subtract Y, 11 wrap once (+P if negative,
            -- else -P if that fits)
            mu_x <= mu_op;
            if mu_op = "11" then
                mu_y <= resize(s_P & "000000", 27);
            else
                if mu_sel = '1' then mu_y <= resize(s_spx & "000000", 27);
                else                 mu_y <= resize(s_G, 27); end if;
            end if;
            if mu_x = "11" then v_sub := not s_ls(26); else v_sub := mu_x(1); end if;
            v_r := f_addsub(unsigned(s_ls), mu_y, v_sub);
            if mu_x = "01" or mu_x = "10" or (mu_x = "11" and (s_ls(26) = '1' or v_r(26) = '0')) then
                s_ls <= signed(v_r);
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_saw_avid = '1' then
                s_vs_pulse <= '1';
                s_saw_avid <= '0';

                s_glide <= registers_in(6)(0);            -- S7 Glide
                s_spk   <= registers_in(6)(1);            -- S8 Figure: Rings / Spokes
                s_inv   <= registers_in(6)(2);            -- S9 Barrier: Slits / Inverse
                s_fine  <= registers_in(6)(3);            -- S10 Weight: Bold / Fine
                s_src   <= registers_in(6)(4);            -- S11 Source: Video / Osc
                s_Wl    <= s_W;

                -- K4 Frames: 7 steps, hysteresis (8 counts) with the two end exceptions
                v_k7 := resize(unsigned(registers_in(3)) & "000", 13) - resize(unsigned(registers_in(3)), 13);
                v_z7 := v_k7(12 downto 10);
                v_f7 := v_k7(9 downto 0);
                if (v_f7 >= 56 and v_f7 < 968) or (v_z7 = 0 and v_f7 < 56) or (v_z7 = 6 and v_f7 >= 968) then
                    s_nz <= v_z7;
                end if;
                -- K3 arms (Spokes): 16 steps, the same hysteresis
                v_k3 := unsigned(registers_in(2));
                if (v_k3(5 downto 3) /= "000" and v_k3(5 downto 3) /= "111")
                   or (v_k3(9 downto 6) = "0000" and v_k3(5 downto 3) = "000")
                   or (v_k3(9 downto 6) = "1111" and v_k3(5 downto 3) = "111") then
                    s_az <= v_k3(9 downto 6);
                end if;

                z_gp <= "00"; z_gc <= "000";
                s_seq <= to_unsigned(1, 6);

            elsif s_seq /= 0 and data_in.avid = '0' and sm_go = '0' and sm_n = 0 then
                v_adv := true;
                if s_seq(3) = '1' then v_gc := s_cys; v_wh := s_H; else v_gc := s_cxs; v_wh := s_W; end if;
                if s_seq(2) = '1' then v_ak := s_a2; else v_ak := s_a1; end if;
                v_pr := signed(shift_left(resize(sm_acc, 22), 2));
                case z_gc is
                    when "000"  => v_am := s_a1; v_rm := unsigned(registers_in(0));
                    when "001"  => v_am := s_a2; v_rm := unsigned(registers_in(1));
                    when "010"  => v_am := s_a3; v_rm := unsigned(registers_in(2));
                    when "011"  => v_am := s_a5; v_rm := unsigned(registers_in(4));
                    when "100"  => v_am := s_a6; v_rm := unsigned(registers_in(5));
                    when others => v_am := s_ap; v_rm := unsigned(registers_in(7));
                end case;
                case to_integer(s_seq) is

                    -- knob channels (K1, K2, K3, K5, K6, P12), Q6, one shared datapath, three
                    -- clocks each.  A gap of one count or less is held (pot jitter).  With Glide
                    -- on K3, K5 and K6 ease 1/16 of the gap per field and P12 Speed 1/8 (the
                    -- speed eases, never the position); K1 / K2 pass (the centre has its own
                    -- glide below).  Glide off: the knob itself.
                    when 1 =>
                        v_adv := false;
                        case z_gp is
                            when "00" =>
                                s_gdv <= signed(resize(v_rm & "000000", 18)) - signed(resize(v_am, 18));
                                z_gp <= "01";
                            when "01" =>
                                if s_gdv > 64 or s_gdv < -64 then
                                    if s_glide = '1' and z_gc = "101" then s_gdv <= shift_right(s_gdv, 3);
                                    elsif s_glide = '1' and z_gc(2 downto 1) /= "00" then s_gdv <= shift_right(s_gdv, 4); end if;
                                else
                                    s_gdv <= (others => '0');
                                end if;
                                z_gp <= "10";
                            when others =>
                                v_ga := unsigned(signed(resize(v_am, 18)) + s_gdv);
                                case z_gc is
                                    when "000"  => s_a1 <= v_ga(15 downto 0);
                                    when "001"  => s_a2 <= v_ga(15 downto 0);
                                    when "010"  => s_a3 <= v_ga(15 downto 0);
                                    when "011"  => s_a5 <= v_ga(15 downto 0);
                                    when "100"  => s_a6 <= v_ga(15 downto 0);
                                    when others => s_ap <= v_ga(15 downto 0);
                                end case;
                                z_gp <= "00";
                                z_gc <= z_gc + 1;
                                if z_gc = "101" then v_adv := true; end if;
                        end case;

                    -- centre: target = W/2 +- |K - 512| * W / 256 (serial multiply; detent
                    -- deadband), then glide: ease, never slower than 1 px / field, land exactly.
                    -- x in states 2 .. 7, y in 4, 8 .. 12 on the same datapath.
                    when 2 | 4 =>
                        v_k := signed(resize(v_ak(15 downto 6), 11)) - 512;
                        if abs(v_k) < C_DB then s_kab <= (others => '0');
                        else                    s_kab <= unsigned(resize(abs(v_k), 10)); end if;
                        s_kng <= v_k(10);
                        if s_seq(2) = '1' then                               -- state 4: the x target
                            if s_kng = '1' then s_tg <= shift_left(resize(signed('0' & v_wh(11 downto 1)), 22), 8) - v_pr;
                            else                s_tg <= shift_left(resize(signed('0' & v_wh(11 downto 1)), 22), 8) + v_pr; end if;
                        end if;
                    when 3 | 8 =>
                        sm_a <= resize(v_wh & "00000000", 20);
                        sm_b <= resize(s_kab & "000000", 16);
                        sm_go <= '1';
                    when 9 =>
                        if s_kng = '1' then s_tg <= shift_left(resize(signed('0' & v_wh(11 downto 1)), 22), 8) - v_pr;
                        else                s_tg <= shift_left(resize(signed('0' & v_wh(11 downto 1)), 22), 8) + v_pr; end if;
                    when 5 | 10 => s_gd <= s_tg - v_gc;
                    when 6 | 11 =>
                        if s_glide = '0' or (s_gd <= 256 and s_gd >= -256) then s_gs <= '1'; else s_gs <= '0'; end if;
                        if s_gd < 4096 and s_gd > -4096 then s_gm <= '1'; else s_gm <= '0'; end if;
                        s_gdd <= shift_right(s_gd, C_GL);
                    when 7 | 12 =>
                        if    s_gs = '1' then v_gdl := s_gd;
                        elsif s_gm = '1' then
                            if s_gd(21) = '0' then v_gdl := to_signed(256, 22); else v_gdl := to_signed(-256, 22); end if;
                        else  v_gdl := s_gdd; end if;
                        if s_seq(3) = '1' then s_cys <= v_gc + v_gdl;
                        else                   s_cxs <= v_gc + v_gdl; end if;

                    -- PERIOD: log2 cycles / px = -9.807 + K3 * 4 / 1024 octaves (Q12)
                    when 13 => s_lfs <= to_signed(-40171, 18) + signed(resize(shift_right(s_a3, 2), 18));
                    -- PITCH: log2 w (px, Q14) = 2 + K5 * 3 / 1024 octaves - log2(x step)
                    when 14 =>
                        s_lf <= resize(shift_right(s_lfs, 6), 12);
                        s_arms <= signed(resize(s_az, 6)) + 1;
                        s_cN  <= C_KN(to_integer(s_nz));
                        s_nm1 <= s_nz + 1;
                        v_lw := resize(s_a5, 17) - resize(shift_right(s_a5, 2), 17);
                        s_lw <= v_lw + to_unsigned(32768, 17) - resize(s_ls14, 17);
                    when 15 =>
                        sq_a <= unsigned(s_lf(5 downto 0)) & "00";
                        v_sh := to_integer(shift_right(s_lf, 6)) - 4;
                        if v_sh < -16 then v_sh := -16; elsif v_sh > 0 then v_sh := 0; end if;
                        s_kc <= v_sh;
                        s_larm <= resize(C_LARM(to_integer(unsigned(s_arms(4 downto 0)))), 12);
                    when 16 =>
                        sq_a <= s_lw(13 downto 6);
                        s_a0 <= s_lf + signed(resize(s_ls6, 12));
                    when 17 =>                                    -- frequency mantissa, Q8
                        s_m <= resize(shift_right(unsigned(sq_q) + 32, 6), 9);
                    when 18 =>                                    -- w = mantissa << octave
                        case to_integer(s_lw(16 downto 14)) is
                            when 0      => s_wp <= resize(shift_right(unsigned(sq_q), 4), 16);
                            when 1      => s_wp <= resize(shift_right(unsigned(sq_q), 3), 16);
                            when 2      => s_wp <= resize(shift_right(unsigned(sq_q), 2), 16);
                            when 3      => s_wp <= resize(shift_right(unsigned(sq_q), 1), 16);
                            when others => s_wp <= unsigned(sq_q);
                        end case;
                    when 19 =>
                        sm_a <= s_wp & "0000";
                        sm_b <= resize(resize(s_nz, 4) + 2, 4) & "000000000000";
                        sm_go <= '1';
                    when 20 =>
                        s_P <= sm_acc(18 downto 0);
                        if s_fine = '1' then s_sw <= shift_right(s_wp, 1); else s_sw <= s_wp; end if;

                    -- SPEED (hypnos): +-24 counts at the centre = stopped; outside it the speed
                    -- is 2^(L/64 - 11) slivers per field, L = 1.3125 x the counts past the
                    -- deadband (0 .. 640): 1/2048 .. 1/2 sliver per field, up = forward.
                    -- Lattice seed: both pixel DDAs are started from zero (as if a line began)
                    -- and run W/2 clocks; what they reach at +W/2 is mirrored into pixel 0.
                    -- (|P12 - 512| in counts as a one's complement: 511 - P12 below the centre)
                    when 21 =>
                        s_neg <= not s_ap(15);
                        if s_ap(15) = '1' then v_av9 := s_ap(14 downto 6); else v_av9 := not s_ap(14 downto 6); end if;
                        if v_av9 < 24 then s_stop <= '1'; s_uu <= (others => '0');
                        else s_stop <= '0'; s_uu <= v_av9 - 24; end if;
                        s_ls <= (others => '0'); s_lp0 <= (others => '0'); s_k0 <= (others => '0');
                        z_ld <= '1';
                        tf_i <= to_unsigned(2027, 11);            -- -21: the barrier DDA loads at s_pc = 19
                    when 22 =>
                        v_adv := false;
                        tf_i <= tf_i + 1;
                        if tf_i = resize(s_Wl(10 downto 1), 11) then
                            v_adv := true;
                            s_spx <= s_spos;                                  -- W/2 mod P
                        end if;
                    when 23 =>
                        s_L <= resize(s_uu, 10) + resize(shift_right(s_uu, 2), 10) + resize(shift_right(s_uu, 4), 10);
                    when 24 =>
                        sq_a <= s_L(5 downto 0) & "00";                       -- speed mantissa
                        s_lp0 <= s_lpos; s_k0 <= s_kcn;                       -- two clocks after the barrier's
                        -- CAMERA gain: px (Q16) per luma count = P * K6 / 910
                        sm_a <= s_P & '0'; sm_b <= s_a6; sm_go <= '1';
                    when 25 =>
                        if s_lp0 /= 0 then
                            s_lp0 <= s_wp - s_lp0;
                            s_k0  <= s_nm1 - s_k0;
                        elsif s_k0 /= 0 then
                            s_k0  <= s_nm1 + 1 - s_k0;
                        end if;
                        s_G <= resize(shift_right(sm_acc, 5), 18) + resize(shift_right(sm_acc, 8), 18);
                    when 26 =>
                        if s_src = '1' then s_G <= shift_left(s_G, 2); end if;           -- Osc: 4x
                        tf_i <= (others => '1'); tf_ph <= '0';
                        mu_sel <= '0';
                        s_spd <= resize(shift_right(unsigned(sq_q), 5), 21);             -- 512 .. 1020
                        s_ssh <= s_L(9 downto 6);

                    -- camera table: t(i) = i * G mod P, two clocks per entry on the unit: an add
                    -- and a wrap are issued in turn and land three clocks later, so the value
                    -- written on the add's clock is the wrapped sum of the entry before
                    when 27 =>
                        v_adv := false;
                        tf_ph <= not tf_ph;
                        if tf_ph = '0' then
                            if tf_i(10) = '0' then
                                tb_we <= '1'; tb_wa <= tf_i(9 downto 0);
                                tb_wd <= std_logic_vector(s_ls(24 downto 9));
                            end if;
                            tf_i <= tf_i + 1;
                            if tf_i /= 1023 then mu_op <= "01"; end if;
                        elsif tf_i = 1024 then
                            v_adv := true;
                        else
                            mu_op <= "11";
                        end if;

                    -- speed = mantissa << octave, one bit per clock
                    when 28 =>
                        if s_ssh /= 0 then
                            v_adv := false;
                            s_spd <= shift_left(s_spd, 1);
                            s_ssh <= s_ssh - 1;
                        end if;
                    -- POSITION, once per field: +- the speed; stopped, it lands on the nearest
                    -- whole sliver (a clean frame) at 1/32 sliver per field
                    when 29 =>
                        if s_stop = '1' and s_an(19 downto 15) = "00000" then
                            s_an(19 downto 0) <= (others => '0');
                        elsif s_stop = '1' and s_an(19 downto 15) = "11111" then
                            s_an(19 downto 0) <= (others => '0');
                            s_an(23 downto 20) <= s_an(23 downto 20) + 1;
                        else
                            if s_stop = '1' then v_sb := to_unsigned(32768, 21); v_sub := not s_an(19);
                            else                 v_sb := s_spd;                  v_sub := s_neg; end if;
                            s_an <= f_addsub('0' & s_an(22 downto 0), resize(v_sb, 24), v_sub);
                        end if;
                    -- wrap the whole slivers into 0 .. N - 1 (a carry or borrow: + N mod 8;
                    -- repeats while they are N or more, as after a Frames change)
                    when 30 =>
                        if s_an(23) = '1' then
                            v_adv := false;
                            s_an(23) <= '0';
                            s_an(22 downto 20) <= s_an(22 downto 20) + s_nm1 + 1;
                        elsif s_an(22 downto 20) > s_nm1 then
                            v_adv := false;
                            s_an(22 downto 20) <= s_an(22 downto 20) - s_nm1 - 1;
                        end if;

                    -- line seed: 1 px - (W/2 mod P) - slide  (mod P), slide = w x position
                    when 31 => s_ls <= to_signed(65536, 27); mu_sel <= '1';
                    when 32 =>
                        mu_op <= "10";
                        sm_a <= resize(s_wp & "000", 20); sm_b <= s_an(22 downto 7); sm_go <= '1';
                    when 33 =>
                        mu_op <= "11";
                        s_spx <= sm_acc(18 downto 0);
                    when 34 => mu_op <= "10";
                    when 35 =>
                        mu_op <= "11";
                        s_cx <= resize(shift_right(s_cxs, 8), 14);
                        s_cy <= resize(shift_right(s_cys, 8), 14);
                    when others => null;
                end case;
                if v_adv then
                    if s_seq = 35 then s_seq <= (others => '0'); else s_seq <= s_seq + 1; end if;
                end if;
            end if;
        end if;
    end process p_frame;

    -- serial multiplier: sm_acc = (sm_a * sm_b) >> 16, sixteen clocks, LSB first
    p_smul : process(clk)
        variable v_s : unsigned(21 downto 0);
    begin
        if rising_edge(clk) then
            if sm_go = '1' then
                sm_sh  <= sm_b;
                sm_acc <= (others => '0');
                sm_n   <= to_unsigned(16, 5);
            elsif sm_n /= 0 then
                if sm_sh(0) = '1' then v_s := resize(sm_acc, 22) + resize(sm_a, 22);
                else                   v_s := resize(sm_acc, 22); end if;
                sm_acc <= v_s(21 downto 1);
                sm_sh  <= shift_right(sm_sh, 1);
                sm_n   <= sm_n - 1;
            end if;
        end if;
    end process p_smul;

    ------------------------------------------------------------------------
    -- coordinate accumulators on the GENERATED avid (hypnos): dx loads -cx at
    -- each line start and steps per pixel; dy seeds at the first line of the
    -- field and steps per line by s_ystp (Q.8 x-pixel units).  s_pc counts the
    -- clocks from T0 (the DDAs of the later stages load from it).
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            s_gav_p <= g_avid;
            if s_pc /= 31 then s_pc <= s_pc + 1; end if;
            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif g_avid = '1' and s_gav_p = '0' then
                if s_newframe = '1' then
                    v_dy0 := shift_left(resize(-s_cy, 23), 8);
                    v_dy0(7) := '1';                                   -- +0.5
                    s_dyc <= v_dy0(22 downto 8);
                    s_dyb <= v_dy0 + signed(resize(s_ystp, 23));
                    s_newframe <= '0';
                else
                    s_dyc <= s_dyb(22 downto 8);
                    s_dyb <= s_dyb + signed(resize(s_ystp, 23));
                end if;
                s_dx <= resize(-s_cx, 15);
                s_pc <= (others => '0');
            elsif g_avid = '1' then
                s_dx <= s_dx + 1;
            end if;
            if z_ld = '1' then s_pc <= (others => '0'); end if;      -- the vblank lattice-seed run
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- EBR delay lines.  Write address = free-running counter; each read runs
    -- X behind it (never the address being written), then a fabric register.
    ------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_wa <= s_wa + 1;
            dlh(to_integer(s_wa)) <= "0000000000" & qsr(8) & std_logic_vector(s8_lh);
            dlh_q  <= dlh(to_integer(s_wa - C_XH));
            dlc(to_integer(s_wa)) <= std_logic_vector(s16_sp);
            dlc_q  <= dlc(to_integer(s_wa - C_XS));
            dld(to_integer(s_wa)) <= "0000" & std_logic_vector(s16_b);
            dld_q  <= dld(to_integer(s_wa - C_XB));
            dle(to_integer(s_wa)) <= "000000000" & std_logic_vector(s25_sl);
            dle_q  <= dle(to_integer(s_wa - C_XE));
            dls(to_integer(s_wa)) <= "000000000000" & g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            dls_q  <= dls(to_integer(s_wa - C_XA));
            s_sy(1) <= dls_q(3 downto 0);
            for i in 2 to 5 loop s_sy(i) <= s_sy(i - 1); end loop;
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
            exm_q <= C_EXM(to_integer(s17_xa));
            exs_q <= C_EXS(to_integer(s17_xa));
        end if;
    end process p_rom;

    -- camera table: written in the vblank sequence only, read by the pixel path
    p_tbram : process(clk)
    begin
        if rising_edge(clk) then
            tb_q <= tb_ram(to_integer(s_Lq(11 downto 2)));
            if tb_we = '1' then tb_ram(to_integer(tb_wa)) <= tb_wd; end if;
        end if;
    end process p_tbram;

    -- box delay: read 32 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= "000000" & std_logic_vector(s_sl); end if;
        end if;
    end process p_bxram;

    -- luma line store (quarter horizontal resolution): the IIR reads its own
    -- column group one address ahead of its write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 2)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= "000000" & std_logic_vector(lb_wd); end if;
        end if;
    end process p_lbram;

    ------------------------------------------------------------------------
    -- SOURCE FIELD (riley's, at 10 bits and quarter resolution): luma (or the
    -- Osc lattice) -> 32-px box, edge-replicated and reloaded at every line
    -- start -> vertical 1/8 IIR in the line store -> linear interpolation
    -- between column groups.  It is not pipeline-aligned: the interpolator
    -- runs 17 source columns ahead of the pixel at stage 18, which centres the
    -- box on the pixel to within a few columns.  The camera is never shown.
    ------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l   : unsigned(9 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(9 downto 0);
        variable v_d   : signed(11 downto 0);
        variable v_a   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_vs_pulse = '1' then
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);
                s_ot <= s_ot + 1;
            end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: luma above black (10 bits), or the lattice
            v_l := unsigned(data_in.y);
            if s_src = '1' then s_sl <= s_osum & "00";
            elsif v_l > 64 then s_sl <= v_l - 64;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 samples; the samples before the line start count as the first one
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(225, 8); end if;
            if s_bcnt < 32 then v_tap := s_sfirst; else v_tap := unsigned(bx_q(9 downto 0)); end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                s_bsum <= shift_left(resize(s_sl, 15), 5);
                s_bcnt <= to_unsigned(1, 6);
                s_lfirst <= '1';
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 32 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on the last column of each group of four: q += (box - q + 4) >> 3
            s_eb <= s_bsum(14 downto 5); s_eq <= unsigned(lb_q1(9 downto 0));
            if s_bv = '1' and s_bcol(1 downto 0) = "11" then s_ev <= '1'; else s_ev <= '0'; end if;
            s_ecol <= s_bcol;
            v_d := signed(resize(s_eb, 12)) - signed(resize(s_eq, 12)) + 4;
            v_a := signed(resize(s_eq, 12)) + shift_right(v_d, 3);
            lb_wd <= unsigned(v_a(9 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 2);

            -- interpolation: 4 Lq runs from the previous group's value to this one's
            if lb_we = '1' then
                if s_lfirst = '1' then s_la <= lb_wd; else s_la <= s_lb; end if;
                s_lb <= lb_wd;
                s_lfirst <= '0';
            end if;
            s_ldq <= lb_we;
            if s_ldq = '1' then
                s_Lq  <= s_la & "00";
                s_Ldd <= signed(resize(s_lb, 12)) - signed(resize(s_la, 12));
                s_lj  <= "11";
            elsif s_lj /= 0 then
                s_Lq <= unsigned(signed(s_Lq) + s_Ldd);
                s_lj <= s_lj - 1;
            end if;
        end if;
    end process p_src;

    ------------------------------------------------------------------------
    -- S1..S8: |dx|,|dy| (13-bit), hi = max, lo = min, and their log2 (Q14):
    -- priority-encode, normalise, 256-segment PWL from EBR.  (hypnos)
    ------------------------------------------------------------------------
    p_logs : process(clk)
        variable va, vb : unsigned(14 downto 0);
        variable vh, vl : unsigned(12 downto 0);
        variable mh, ml : integer range 0 to 12;
    begin
        if rising_edge(clk) then
            -- S1
            va := unsigned(abs(s_dx));
            vb := unsigned(abs(s_dyc));
            s1_a <= f_sat(va, 13);
            s1_b <= f_sat(vb, 13);
            s1_dxs <= s_dx(14);
            s1_dys <= s_dyc(14);
            -- S2
            if s1_a >= s1_b then s2_hi <= s1_a; s2_lo <= s1_b; s2_sw <= '0';
            else                 s2_hi <= s1_b; s2_lo <= s1_a; s2_sw <= '1'; end if;
            if s1_a = 0 or s1_b = 0 then s2_lz <= '1'; else s2_lz <= '0'; end if;
            s2_dxs <= s1_dxs;
            s2_dys <= s1_dys;
            -- S3: msb (hi >= 1 always: the exact centre reads as 1)
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
            -- S8: log2 = msb + PWL(mantissa), Q14
            s8_lh <= shift_left(resize(s7_mh, 18), 14) + resize(s7_lvh, 18) + resize(shift_right(s7_lph, 4), 18);
            s8_ll <= shift_left(resize(s7_ml, 18), 14) + resize(s7_lvl, 18) + resize(shift_right(s7_lpl, 4), 18);

            qsr(3) <= s2_sw & s2_dxs & s2_dys & s2_lz;
            for i in 4 to 8 loop qsr(i) <= qsr(i - 1); end loop;
        end if;
    end process p_logs;

    ------------------------------------------------------------------------
    -- S9..S15: u = log2(hi/lo) >= 0 ;  log2 r = log2 hi + 0.5 log2(1 + 4^-u)
    -- and the octant angle atan(2^-u), both from interpolated EBR tables.
    -- S16: log2 r in 1080-line units, the spoke phase arms * theta (whole
    -- cycles per turn: seamless), the angular gradient log2(arms / (2 pi r)).
    ------------------------------------------------------------------------
    p_polar : process(clk)
        variable vu   : unsigned(17 downto 0);
        variable vmag : unsigned(17 downto 0);
        variable q    : std_logic_vector(3 downto 0);
        variable vlh  : unsigned(17 downto 0);
        variable vsp  : signed(24 downto 0);
    begin
        if rising_edge(clk) then
            -- S9
            vu := s8_lh - s8_ll;
            if qsr(8)(0) = '1' then vu := to_unsigned(16 * 16384 - 1, 18); end if;
            s9_ua <= vu;
            -- S10: 1/16-octave segment + fraction
            s10_ia <= s9_ua(17 downto 10);
            s10_tu <= s9_ua(9 downto 3);
            -- S11, S12 (EBR read, fabric register)
            s11_tu <= s10_tu;
            s12_tu <= s11_tu;
            s12_g <= ug_q; s12_gs <= ugs_q; s12_a <= ua_q; s12_as <= uas_q;
            -- S13: interpolation products (slopes are negative)
            s13_gp <= resize(bmul(signed(s12_gs(10 downto 0)), signed(resize(s12_tu, 8))), 22);
            s13_ap <= resize(bmul(signed(s12_as(10 downto 0)), signed(resize(s12_tu, 8))), 22);
            s13_g  <= unsigned(s12_g);
            s13_a  <= unsigned(s12_a);
            -- S14
            s14_g <= resize(unsigned(signed(resize(s13_g, 17)) + resize(shift_right(s13_gp, 7), 17)), 14);
            s14_a <= resize(unsigned(signed(resize(s13_a, 17)) + resize(shift_right(s13_ap, 7), 17)), 16);
            -- S15: log2 r ; full polar angle from the octant angle + swap + signs
            vlh := unsigned(dlh_q(17 downto 0));
            s15_lr <= vlh + resize(s14_g, 18);
            q := dlh_q(21 downto 18);
            if q(3) = '1' then vmag := to_unsigned(65536, 18) - resize(s14_a, 18);
            else               vmag := resize(s14_a, 18); end if;
            if    q(2) = '0' and q(1) = '0' then s15_th <= vmag;
            elsif q(2) = '1' and q(1) = '0' then s15_th <= to_unsigned(131072, 18) - vmag;
            elsif q(2) = '1' and q(1) = '1' then s15_th <= to_unsigned(131072, 18) + vmag;
            else                                 s15_th <= to_unsigned(0, 18) - vmag; end if;
            -- S16
            s16_e <= resize(s15_lr, 19) + resize(s_ls14, 19);
            vsp := bmul(signed(resize(s15_th, 19)), s_arms);
            s16_sp <= unsigned(vsp(17 downto 2));
            s16_b <= s_larm - signed(resize(shift_right(s15_lr, 8), 12));
        end if;
    end process p_polar;

    ------------------------------------------------------------------------
    -- S17..S24: r = 2^e (mantissa from EBR PWL) ; phase = r * f as ONE
    -- multiply (mantissa x freq mantissa) and ONE shift by (Ie + Li - 4).
    ------------------------------------------------------------------------
    p_exp : process(clk)
        variable vk : integer;
        variable vw : unsigned(41 downto 0);
    begin
        if rising_edge(clk) then
            -- S17
            s17_xa <= s16_e(13 downto 6);
            s17_t  <= s16_e(5 downto 0);
            vk := to_integer(s16_e(18 downto 14)) + s_kc;
            if vk < -17 then vk := -17; elsif vk > 7 then vk := 7; end if;
            s17_k <= vk;
            -- S18, S19 (EBR read, fabric register)
            s18_t <= s17_t; s18_k <= s17_k;
            s19_t <= s18_t; s19_k <= s18_k;
            s19_m <= exm_q; s19_s <= exs_q;
            -- S20
            s20_ep <= resize(unsigned(s19_s(7 downto 0)) * s19_t, 14);
            s20_m  <= unsigned(s19_m);
            s20_k  <= s19_k;
            -- S21
            s21_mt <= s20_m + resize(shift_right(s20_ep, 6), 16);
            s21_k  <= s20_k;
            -- S22: mantissa (Q14) x freq mantissa (Q8), split on the wide operand
            s22_ph <= bmul_u(s21_mt(15 downto 8), s_m);
            s22_pl <= bmul_u(s21_mt(7 downto 0), s_m);
            s22_k  <= s21_k;
            -- S23
            s23_pr <= shift_left(resize(s22_ph, 25), 8) + resize(s22_pl, 25);
            s23_k  <= s22_k;
            -- S24: phase (Q16 fraction of a cycle) = product * 2^k
            vw := shift_left(resize(s23_pr, 42), s23_k + 17);
            if s23_k <= -17 then
                s24_phr <= (others => '0');
            else
                s24_phr <= vw(34 downto 19);
            end if;
        end if;
    end process p_exp;

    ------------------------------------------------------------------------
    -- S21..S26: filter width w = the phase gradient in cycles per pixel
    -- (Rings: per field; Spokes: the angular gradient), widened above 1/4
    -- cycle per pixel so it reaches a full cycle (flat mid-grey) at Nyquist,
    -- rounded UP to a half octave so the divide is a shift.  (hypnos)
    ------------------------------------------------------------------------
    p_grad : process(clk)
        variable vlg : signed(11 downto 0);
        variable vlw : signed(11 downto 0);
        variable viw : integer;
        variable vsh : integer;
        variable vw  : unsigned(15 downto 0);
        variable vm  : unsigned(6 downto 0);
        variable vrr : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- S21
            if s_spk = '1' then vlg := signed(dld_q(11 downto 0)); else vlg := s_a0; end if;
            if vlg > 255 then vlg := to_signed(255, 12); end if;
            s21_lg <= vlg;
            -- S22  (2 lg + 128 is formed with a shift, never lg + (lg + 128): router1)
            s22_lg <= s21_lg;
            vlw := s21_lg + 128;
            if vlw > 0 then s22_lwp <= '1'; else s22_lwp <= '0'; end if;
            -- S23
            if s22_lwp = '1' then s23_lw <= shift_left(s22_lg, 1) + 128; else s23_lw <= s22_lg; end if;
            -- S24: clamp to 16 octaves ; quantise UP to a half octave
            vlw := s23_lw;
            if vlw < -1023 then vlw := to_signed(-1023, 12); end if;
            vlw := vlw + 31;
            vlw(4 downto 0) := (others => '0');
            if vlw >= 0 then s24_sat <= '1'; else s24_sat <= '0'; end if;
            s24_h <= vlw(5);
            viw := to_integer(shift_right(vlw, 6));
            if viw > 0 then viw := 0; end if;
            s24_iw <= viw;
            if vlw >= 0 then vrr := (others => '0'); else vrr := -resize(vlw, 13); end if;
            s24_ri <= to_integer(unsigned(vrr(9 downto 6)));
            s24_rh <= vrr(5);
            -- S25: filter width w (Q16)
            if s24_sat = '1' then
                vw := (others => '1');
            else
                vsh := s24_iw + 10;                       -- w (Q16) = {64, 91} * 2^(Iw+10)
                if s24_h = '1' then vm := to_unsigned(91, 7); else vm := to_unsigned(64, 7); end if;
                if vsh >= 0 then vw := resize(shift_left(resize(vm, 16), vsh), 16);
                else             vw := resize(shift_right(resize(vm, 16), -vsh), 16); end if;
                if vw = 0 then vw := to_unsigned(1, 16); end if;
            end if;
            s25_w  <= vw;
            s25_ri <= s24_ri; s25_rh <= s24_rh;
            -- S25: the figure's phase, and the frame offset of the sliver this pixel shows
            if s_spk = '1' then s25_phq <= unsigned(dlc_q); else s25_phq <= s24_phr; end if;
            if s24_fr = '1' then s25_ko <= s_kox; else s25_ko <= s_koy; end if;
            -- S26
            s26_phi <= s25_phq - s25_ko;
            s26_wh  <= s25_w(15 downto 1);
            s26_ri  <= s25_ri; s26_rh <= s25_rh;
        end if;
    end process p_grad;

    ------------------------------------------------------------------------
    -- LATTICE and BARRIER: two DDAs in real pixels (Q.10), loaded from the
    -- per-field / per-line seeds at fixed clocks after T0 and free-running.
    --   lattice (stage 23): position inside the sliver at the pixel's LEFT edge,
    --     sliver index k, k / N of a cycle.  s_kox = the next pixel's (= this
    --     pixel's right edge), s_koy = this pixel's at stage 24.
    --   barrier (stage 21): e = position of the pixel's right edge in the
    --     barrier period, minus the camera shift.  Slit = [0, s): coverage =
    --     clamp(e, 0, 1) - clamp(e - s, 0, 1), a one-pixel box filter.
    --   A partly covered pixel shows the sliver its lit part lies in: the
    --   right-edge sliver on the slit's rising edge, else the left-edge one
    --   (Inverse: the right-edge one on the falling edge).
    ------------------------------------------------------------------------
    p_bar : process(clk)
        variable v_n  : unsigned(16 downto 0);
        variable v_m  : signed(17 downto 0);
        variable v_sn : unsigned(19 downto 0);
        variable v_sm : signed(20 downto 0);
        variable v_c  : unsigned(6 downto 0);
        variable v_k1, v_k2 : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- barrier DDA, stage 20
            v_sn := resize(s_spos, 20) + 1024;
            v_sm := signed(resize(v_sn, 21)) - signed(resize(s_P, 21));
            if s_pc = 19 then s_spos <= unsigned(s_ls(24 downto 6));
            elsif v_sm(20) = '0' then s_spos <= unsigned(v_sm(18 downto 0));
            else s_spos <= v_sn(18 downto 0); end if;
            -- camera shift (EBR read at 19, fabric register at 20)
            s20_t <= unsigned(tb_q);
            -- S21
            s21_er <= signed(resize(s_spos, 20)) - signed(resize(s20_t & "000", 20));
            -- S22: wrap into [0, P)
            if s21_er(19) = '1' then s22_e <= s21_er + signed(resize(s_P, 20)); else s22_e <= s21_er; end if;
            -- S23: rising clamp (Q6) ; the same position minus the slit width
            if s22_e(19 downto 10) /= 0 then s23_c1 <= to_unsigned(64, 7); s23_fe <= '0';
            else s23_c1 <= '0' & unsigned(s22_e(9 downto 4)); s23_fe <= '1'; end if;
            s23_g <= s22_e - signed(resize(s_sw, 20));
            -- S24: falling clamp and the sliver choice
            s24_c1 <= s23_c1;
            if s23_g(19) = '1' then s24_c2 <= (others => '0');
            elsif s23_g(18 downto 10) /= 0 then s24_c2 <= to_unsigned(64, 7);
            else s24_c2 <= '0' & unsigned(s23_g(9 downto 4)); end if;
            if s_inv = '1' then
                if s23_g(19 downto 10) = 0 then s24_fr <= '1'; else s24_fr <= '0'; end if;
            else
                s24_fr <= s23_fe;
            end if;
            -- S25: slit coverage 0 .. 64
            v_c := s24_c1 - s24_c2;
            if s_inv = '1' then s25_sl <= to_unsigned(64, 7) - v_c; else s25_sl <= v_c; end if;

            -- lattice DDA, stage 22 ; k / N of a cycle at 23 (next pixel) and 24 (this pixel)
            v_n := resize(s_lpos, 17) + 1024;
            v_m := signed(resize(v_n, 18)) - signed(resize(s_wp, 18));
            if s_pc = 21 then
                s_lpos <= s_lp0; s_kcn <= s_k0;
            elsif v_m(17) = '0' then
                s_lpos <= unsigned(v_m(15 downto 0));
                if s_kcn = s_nm1 then s_kcn <= (others => '0'); else s_kcn <= s_kcn + 1; end if;
            else
                s_lpos <= v_n(15 downto 0);
            end if;
            if s_kcn(0) = '1' then v_k1 := s_cN; else v_k1 := (others => '0'); end if;
            if s_kcn(1) = '1' then v_k1 := v_k1 + shift_left(s_cN, 1); end if;
            if s_kcn(2) = '1' then v_k2 := shift_left(s_cN, 2); else v_k2 := (others => '0'); end if;
            s_kox <= v_k1 + v_k2;
            s_koy <= s_kox;
        end if;
    end process p_bar;

    ------------------------------------------------------------------------
    -- S27..S32: one-pixel box filter of the band square-wave (hypnos).
    --   band = frac in [0, 1/2) ; P(x) = floor(x)/2 + min(frac x, 1/2)
    --   coverage = (P(phi + w/2) - P(phi - w/2)) / w   (divide = * 2^-log2 w)
    ------------------------------------------------------------------------
    p_cov : process(clk)
        variable vn  : signed(17 downto 0);
        variable vs  : unsigned(24 downto 0);
        variable vc  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S27
            s27_lo <= s26_phi - resize(s26_wh, 16);
            s27_hi <= s26_phi + resize(s26_wh, 16);
            s27_ri <= s26_ri; s27_rh <= s26_rh;
            -- S28
            if s27_hi < s27_lo then s28_k <= '1'; else s28_k <= '0'; end if;
            if s27_hi >= s27_lo and s27_hi < C_D16 then s28_full <= '1'; else s28_full <= '0'; end if;
            if s27_hi < C_D16 then s28_mh <= s27_hi; else s28_mh <= C_D16; end if;
            if s27_lo < C_D16 then s28_ml <= s27_lo; else s28_ml <= C_D16; end if;
            s28_ri <= s27_ri; s28_rh <= s27_rh;
            -- S29
            vn := signed(resize(s28_mh, 18)) - signed(resize(s28_ml, 18));
            if s28_k = '1' then vn := vn + signed(resize(C_D16, 18)); end if;
            if vn < 0 then vn := (others => '0'); end if;
            s29_n   <= unsigned(vn(15 downto 0));
            s29_ri  <= s28_ri; s29_rh <= s28_rh;
            s29_full <= s28_full;
            -- S30: (N << Ri) >> 7
            vs := shift_left(resize(s29_n, 25), s29_ri);
            s30_nsh <= vs(16 downto 7);
            s30_rh  <= s29_rh;
            s30_full <= s29_full;
            -- S31: / 2, or * sqrt2 / 2 on a half-octave step
            if s30_rh = '1' then
                s31_cp <= resize(shift_right(s30_nsh, 1) + shift_right(s30_nsh, 3)
                                 + shift_right(s30_nsh, 4) + shift_right(s30_nsh, 6), 11);
            else
                s31_cp <= resize(shift_right(s30_nsh, 1), 11);
            end if;
            s31_full <= s30_full;
            -- S32: coverage 0..256
            vc := resize(s31_cp, 10);
            if vc > 256 or s31_full = '1' then vc := to_unsigned(256, 10); end if;
            s32_cov <= resize(vc, 9);
        end if;
    end process p_cov;

    ------------------------------------------------------------------------
    -- S33..S38: figure AND slit -> luma, [1 2 1]/4, blanking gate.
    --   y = 64 + 712 * c  (c = coverage product, 2^14 = 1): white 776.  This is
    --   the set's conditioning y - (d >> 3) - (d >> 4) on a 940 white, folded
    --   into the scale (the filter is linear, so the order does not matter).
    --   Outside the generated avid the filter sees black.  Chroma is constant.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_y : unsigned(11 downto 0);
        variable v_s : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- S33: figure AND slit = the smaller coverage (0 .. 256)
            v_s := unsigned(dle_q(6 downto 0)) & "00";
            if s32_cov < v_s then s33_c <= s32_cov; else s33_c <= v_s; end if;
            -- S34: 64 + 712 c / 256
            if dls_q(3) = '1' then
                s34_y <= to_unsigned(64, 10) + resize(s33_c & '0', 10) + resize(shift_right(s33_c, 1), 10)
                         + resize(shift_right(s33_c, 2), 10) + resize(shift_right(s33_c, 5), 10);
            else
                s34_y <= to_unsigned(64, 10);
            end if;
            -- S35, S36: taps y(n + 1), y(n), y(n - 1)
            s35_y  <= s34_y;
            s36_yo <= s35_y;
            s36_ya <= resize(s34_y, 12) + resize(s36_yo, 12) + 2;
            s36_yb <= s35_y;
            -- S37
            v_y := s36_ya + shift_left(resize(s36_yb, 12), 1);
            s37_y <= v_y(11 downto 2);
            -- S38: blanking gate on the generated avid of stage 37
            if s_sy(4)(3) = '1' then o_y <= s37_y; else o_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.avid    <= s_sy(5)(3);
    data_out.hsync_n <= s_sy(5)(2);
    data_out.vsync_n <= s_sy(5)(1);
    data_out.field_n <= s_sy(5)(0);

end architecture kinegram;
