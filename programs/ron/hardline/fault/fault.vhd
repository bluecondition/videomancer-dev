-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- fault.vhd  (FAULT v0.1.0, Hardline set, 2026-11-15)
--
-- A perfect ring target cut into strips, the strips slid along the cut
-- (Marina Apollonio's "Dinamica circolare", Soto's dislocations).  Black and
-- white only: U = V = 512 on every pixel.  Nothing moves by itself.
--
--   strips    the screen is cut through its centre into parallel strips
--             (height K4 Slices, direction K5 Angle).  Two accumulators hold
--             each pixel's distance into its strip, across the cut (v) and
--             along it (u), in pixels, and wrap at the strip height; a 3-bit
--             class counts the wraps.
--   shift     the class picks one of 8 shift vectors (a block-RAM table the
--             sequencer refills each field): S9 Alternate +-slip by parity,
--             Staircase an 8-step sawtooth (k - 3.5) slip.  S8 Tiles adds the
--             u class turned a quarter (columns slide across the cut too).
--             The shift is added to (dx, dy) BEFORE the distance is taken.
--   rings     the Hypnos log-domain polar engine: hi, lo = max, min(|dx|,
--             |dy|) -> log2 -> log2 r = log2 hi + 0.5 log2(1 + 4^-u) ->
--             phase = 2^(log2 r + log2 f), one table and one shift.
--   camera    box-filtered luma (64-px box, vertical 1/32 one-pole in a
--             half-.. quarter-resolution line store) x K6 is added to the
--             ring phase (2.56 cycles at white, 100 %).  S11 Osc substitutes
--             a slow triangle lattice at the same point.
--   edges     the one-pixel box filter of the band square-wave from the
--             analytic phase gradient (ring frequency, combined with a
--             half-octave code of the camera field's own gradient); rings
--             finer than about 3 px fade to the duty grey.
--   kerf      an anti-aliased black line about 1.75 px wide on every cut
--             (it opens with the first 2 px of slip), so a cut is clean at
--             any angle.
--   output    luma [1 2 1] / 4, 13/16 toward black (white about 775),
--             blanking gate.  One latency (40 clocks) in every mode.
--
-- Pipeline stage Pk: pixel i is at Pk in the i-th clock in which gd(k + 1) is
-- high (gd = the generated avid, delayed).  The integer model
-- (.hardline_work/rb_fault/imodel.py) mirrors every stage.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.fault_pkg.all;

architecture fault of program_top is
    constant C_LAT : integer := 40;                   -- = processing_delay_clks

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_DB  : integer := 14;                   -- centre-knob detent deadband
    constant C_GL  : integer := 4;                    -- glide one-pole shift (1/16 per field)

    -- TOML defaults: the held knobs and the glides start there
    constant C_DEF_K3  : integer := 614;
    constant C_DEF_K4  : integer := 526;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 618;

    -- EBR delay lines: written from stage W, read by the logic building stage
    -- C, the read address runs X = C - 2 - W behind the write address
    -- (one less here: each read is re-registered in fabric before the adder)
    constant C_XH : integer := 19 - 3 - 13;           -- log2 hi + E0
    constant C_XK : integer := 33 - 3 - 4;            -- kerf coverage

    attribute no_rw_check : boolean;

    type t_ram16 is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_ram32 is array (0 to 255) of std_logic_vector(31 downto 0);
    type t_ram8  is array (0 to 255) of unsigned(7 downto 0);
    type t_lb    is array (0 to 511) of std_logic_vector(15 downto 0);

    ----------------------------------------------------------------------
    -- raster measurement, mode constants, generated avid
    ----------------------------------------------------------------------
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Wl    : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_ystp  : unsigned(9 downto 0) := to_unsigned(256, 10);   -- field-line pitch in x pixels, Q8
    signal s_mcls  : std_logic_vector(1 downto 0) := "00";           -- 00 1080 / 01 720p / 10 SD
    signal s_lacc  : unsigned(19 downto 0) := (others => '0');
    signal s_ls14  : unsigned(14 downto 0) := (others => '0');       -- log2(x step), Q14
    signal s_oxs   : unsigned(8 downto 0) := to_unsigned(102, 9);    -- Osc steps
    signal s_oys   : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_n64   : std_logic := '1';                               -- 64-px box (1080), else 32
    signal s_avid_q, s_avid_r : std_logic := '0';
    signal s_hs_q, s_hs_r     : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_rsr   : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc    : unsigned(3 downto 0) := "1000";
    signal g_cnt   : unsigned(11 downto 0) := (others => '0');
    signal gd      : std_logic_vector(1 to C_LAT) := (others => '0');   -- g_avid delayed k clocks

    ----------------------------------------------------------------------
    -- held controls, glides, per-field constants
    ----------------------------------------------------------------------
    signal s_k1, s_k2, s_k5 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k3  : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal s_k4  : unsigned(9 downto 0) := to_unsigned(C_DEF_K4, 10);
    signal s_k6  : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal s_k12 : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal s_glide : std_logic := '1';
    signal s_tiles, s_stair, s_fine, s_osc : std_logic := '0';

    signal s_g3 : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);   -- glided knobs, Q6
    signal s_g4 : unsigned(15 downto 0) := to_unsigned(C_DEF_K4 * 64, 16);
    signal s_g5 : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_g6 : unsigned(15 downto 0) := to_unsigned(C_DEF_K6 * 64, 16);
    signal s_gm : unsigned(15 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_kd : signed(16 downto 0) := (others => '0');
    signal s_kn : unsigned(1 downto 0) := "00";

    signal s_cxs, s_cys, s_axs, s_ays : signed(21 downto 0) := (others => '0');   -- centre / slip glides, Q8 px
    signal s_gd, s_gdd : signed(21 downto 0) := (others => '0');
    signal s_gs, s_gmm : std_logic := '0';
    signal s_gch : unsigned(1 downto 0) := "00";

    signal s_cx, s_cy : signed(13 downto 0) := (others => '0');
    signal s_ax, s_ay : signed(11 downto 0) := (others => '0');

    signal s_t   : unsigned(16 downto 0) := (others => '0');
    signal c_E0  : signed(19 downto 0) := to_signed(-95000, 20);   -- log2 ring cycles / px, Q14
    signal c_hw0 : signed(5 downto 0) := to_signed(-11, 6);        -- the same in half octaves, rounded up
    signal s_pe  : unsigned(17 downto 0) := (others => '0');
    signal s_ip  : unsigned(3 downto 0) := (others => '0');
    signal s_pm  : unsigned(15 downto 0) := to_unsigned(16384, 16);
    signal s_lsz : signed(19 downto 0) := (others => '0');
    signal s_is  : unsigned(3 downto 0) := (others => '0');
    signal s_shr : unsigned(21 downto 0) := (others => '0');
    signal s_shn : unsigned(3 downto 0) := (others => '0');
    signal c_S   : signed(21 downto 0) := to_signed(108 * 2048, 22);   -- strip height, Q11 px
    signal s_ai  : unsigned(9 downto 0) := (others => '0');
    signal s_aneg : std_logic := '0';
    signal s_xad : unsigned(7 downto 0) := (others => '0');
    signal s_xsel : std_logic := '0';
    signal s_sn, s_cs : unsigned(14 downto 0) := (others => '0');      -- |sin|, cos of the cut angle, Q14
    signal s_sns : signed(16 downto 0) := (others => '0');
    signal c_nx  : signed(12 downto 0) := (others => '0');             -- v step per pixel, Q11
    signal c_tx  : unsigned(11 downto 0) := to_unsigned(2048, 12);     -- u step per pixel
    signal c_dvl : unsigned(12 downto 0) := to_unsigned(2048, 13);     -- v step per line
    signal c_dul : signed(13 downto 0) := (others => '0');             -- u step per line
    signal s_R   : signed(22 downto 0) := (others => '0');             -- the +-S scratch
    signal s_rc  : unsigned(2 downto 0) := "100";
    signal s_wlv, s_wlu : std_logic := '0';
    signal bv_i1, bu_i1 : signed(21 downto 0) := (others => '0');
    signal s_sneg : std_logic := '0';
    signal s_sq  : unsigned(9 downto 0) := (others => '0');
    signal s_sp  : unsigned(11 downto 0) := (others => '0');
    signal s_tc  : unsigned(2 downto 0) := "000";
    signal s_ty  : std_logic := '0';
    signal tw_we : std_logic := '0';
    signal tw_a  : unsigned(2 downto 0) := "000";
    signal tw_dx, tw_dy : std_logic_vector(15 downto 0) := (others => '0');
    signal c_kc0 : signed(5 downto 0) := to_signed(-16, 6);
    signal c_G   : unsigned(8 downto 0) := (others => '0');            -- camera gain: phase (Q16) = G x level (Q4) / 8
    signal c_camon : std_logic := '0';
    signal s_nrm : unsigned(9 downto 0) := (others => '0');
    signal s_nn  : unsigned(3 downto 0) := (others => '0');
    signal c_hb0 : signed(6 downto 0) := to_signed(-41, 7);            -- camera gradient code 0, half octaves
    signal c_D16 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal wt_we : std_logic := '0';
    signal wt_i  : unsigned(8 downto 0) := (others => '0');
    signal wt_wa : unsigned(7 downto 0) := (others => '0');
    signal wt_acc : unsigned(15 downto 0) := (others => '0');

    signal tab_a : unsigned(8 downto 0) := (others => '0');
    signal tab_q : std_logic_vector(15 downto 0) := (others => '0');
    signal tab_r : unsigned(15 downto 0) := (others => '0');

    signal s_pc  : unsigned(6 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_saw_avid   : std_logic := '0';
    signal s_done       : std_logic := '0';
    -- serial shift-add multiplier (signed A x unsigned B), 12 clocks; neg: -A; keep: add to the last product
    signal sm_go, sm_neg, sm_keep, sm_s : std_logic := '0';
    signal sm_ai  : signed(16 downto 0) := (others => '0');
    signal sm_bi  : unsigned(11 downto 0) := (others => '0');
    signal sm_a   : signed(28 downto 0) := (others => '0');
    signal sm_b   : unsigned(11 downto 0) := (others => '0');
    signal sm_acc : signed(29 downto 0) := (others => '0');
    signal sm_n   : unsigned(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- strips: beam and line accumulators, shift tables, kerf
    ----------------------------------------------------------------------
    signal bv, bu, lv, lu : signed(21 downto 0) := (others => '0');
    signal bvc, buc, lvc, luc : unsigned(2 downto 0) := "100";
    signal s_lpb : std_logic := '0';
    signal txk_ram, tyk_ram, txj_ram, tyj_ram : t_ram16 := (others => (others => '0'));
    attribute no_rw_check of txk_ram : signal is true;
    attribute no_rw_check of tyk_ram : signal is true;
    attribute no_rw_check of txj_ram : signal is true;
    attribute no_rw_check of tyj_ram : signal is true;
    signal txk_q, tyk_q, txj_q, tyj_q : std_logic_vector(15 downto 0) := (others => '0');
    signal p2_txk, p2_tyk, p2_txj, p2_tyj : signed(14 downto 0) := (others => '0');
    signal p3_sx, p3_sy : signed(15 downto 0) := (others => '0');
    signal p1_dav, p1_dbv, p1_dau, p1_dbu : unsigned(5 downto 0) := (others => '0');
    signal p2_dv, p2_du, p3_d : unsigned(5 downto 0) := (others => '0');
    signal p4_kc : unsigned(4 downto 0) := (others => '0');

    -- coordinate accumulators (P3)
    signal s_dx  : signed(14 downto 0) := (others => '0');
    signal s_dyc : signed(14 downto 0) := (others => '0');
    signal s_dyb : signed(22 downto 0) := (others => '0');
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- EBR delay lines
    ----------------------------------------------------------------------
    signal dlh : t_ram32 := (others => (others => '0'));   -- log2 hi + E0
    signal dlk : t_ram16 := (others => (others => '0'));   -- kerf coverage
    signal dls : t_ram16 := (others => (others => '0'));   -- syncs
    attribute no_rw_check of dlh : signal is true;
    attribute no_rw_check of dlk : signal is true;
    attribute no_rw_check of dls : signal is true;
    signal s_wa  : unsigned(7 downto 0) := (others => '0');
    signal dlh_q : std_logic_vector(31 downto 0) := (others => '0');
    signal dlk_q : std_logic_vector(15 downto 0) := (others => '0');
    signal dlh_r : signed(19 downto 0) := (others => '0');
    signal dlk_r : unsigned(4 downto 0) := (others => '0');
    signal dls_q : std_logic_vector(15 downto 0) := x"0007";
    signal dls_q2 : std_logic_vector(2 downto 0) := "111";

    ----------------------------------------------------------------------
    -- polar engine
    ----------------------------------------------------------------------
    signal p4_dx, p4_dy : signed(15 downto 0) := (others => '0');
    signal p5_a, p5_b : unsigned(13 downto 0) := (others => '0');
    signal p6_hi, p6_lo : unsigned(13 downto 0) := (others => '0');
    signal p7_hi, p7_lo : unsigned(13 downto 0) := (others => '0');
    signal p7_mh, p7_ml : unsigned(3 downto 0) := (others => '0');
    signal p8_ih, p8_il : unsigned(7 downto 0) := (others => '0');
    signal p8_th, p8_tl, p9_th, p9_tl, p10_th, p10_tl : unsigned(4 downto 0) := (others => '0');
    signal p8_mh, p8_ml, p9_mh, p9_ml, p10_mh, p10_ml, p11_mh, p11_ml : unsigned(3 downto 0) := (others => '0');
    signal lvh_q, lsh_q, lvl_q, lsl_q : std_logic_vector(15 downto 0) := (others => '0');
    signal p10_lvh, p10_lsh, p10_lvl, p10_lsl : std_logic_vector(15 downto 0) := (others => '0');
    signal p11_lph, p11_lpl : unsigned(11 downto 0) := (others => '0');
    signal p11_lvh, p11_lvl : unsigned(13 downto 0) := (others => '0');
    signal p12_lh, p12_ll : unsigned(17 downto 0) := (others => '0');
    signal lzsr : std_logic_vector(6 to 12) := (others => '0');
    signal p13_u   : unsigned(17 downto 0) := (others => '0');
    signal p13_lhe : signed(19 downto 0) := (others => '0');
    signal p14_ia  : unsigned(7 downto 0) := (others => '0');
    signal p14_tu, p15_tu, p16_tu : unsigned(6 downto 0) := (others => '0');
    signal ug_q, ugs_q, p16_g, p16_gs : std_logic_vector(15 downto 0) := (others => '0');
    signal p17_gp : signed(18 downto 0) := (others => '0');
    signal p17_g  : unsigned(15 downto 0) := (others => '0');
    signal p18_g  : unsigned(13 downto 0) := (others => '0');
    signal p19_e  : signed(19 downto 0) := (others => '0');
    signal p20_xa : unsigned(7 downto 0) := (others => '0');
    signal p20_t, p21_t, p22_t : unsigned(5 downto 0) := (others => '0');
    signal p20_sa, p21_sa, p22_sa, p23_sa, p24_sa : unsigned(4 downto 0) := (others => '0');
    signal p20_z, p21_z, p22_z, p23_z, p24_z : std_logic := '0';
    signal exm_q, exs_q, p22_m, p22_s : std_logic_vector(15 downto 0) := (others => '0');
    signal p23_ep : unsigned(13 downto 0) := (others => '0');
    signal p23_m  : unsigned(15 downto 0) := (others => '0');
    signal p24_mt : unsigned(15 downto 0) := (others => '0');
    signal p25_phr : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- source field (camera luma or the Osc lattice)
    ----------------------------------------------------------------------
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot   : unsigned(7 downto 0) := (others => '0');
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal bx_ram : t_ram8 := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_r : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_bsum : unsigned(13 downto 0) := (others => '0');
    signal w_box12 : unsigned(11 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal lb_we  : std_logic := '0';
    signal lb_wa  : unsigned(8 downto 0) := (others => '0');
    signal lb_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal e_pc   : unsigned(10 downto 0) := (others => '0');     -- index of the pixel at P18
    signal e_f1, e_f2, p21_f : unsigned(1 downto 0) := "00";
    signal e_st   : std_logic_vector(0 to 5) := (others => '0');
    signal e_idx, e_idxw : unsigned(8 downto 0) := (others => '0');
    signal e_first : std_logic := '1';                              -- the first line of a field
    signal e_cq, e_pq, e_qo, e_bxr, e_qn, e_qnp : unsigned(11 downto 0) := (others => '0');
    signal e_cc, e_pcd, e_code : unsigned(3 downto 0) := (others => '0');
    signal e_dqs  : signed(7 downto 0) := (others => '0');
    signal e_dvm, e_dhm : unsigned(7 downto 0) := (others => '0');
    signal e_v    : unsigned(8 downto 0) := (others => '0');
    signal p21_diff : signed(12 downto 0) := (others => '0');
    signal p21_prev : unsigned(11 downto 0) := (others => '0');
    signal p22_L  : unsigned(13 downto 0) := (others => '0');
    signal wt_ram : t_ram16 := (others => (others => '0'));          -- camera phase by whole level
    attribute no_rw_check of wt_ram : signal is true;
    signal wt_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal p24_wt : unsigned(15 downto 0) := (others => '0');
    signal p23_pa, p23_pb, p24_pp : unsigned(12 downto 0) := (others => '0');
    signal p25_camd : unsigned(15 downto 0) := to_unsigned(16384, 16);

    ----------------------------------------------------------------------
    -- gradient -> filter width, coverage, output
    ----------------------------------------------------------------------
    signal g0_hb  : signed(6 downto 0) := (others => '0');
    signal g0_v   : std_logic := '0';
    signal g1_hw  : unsigned(4 downto 0) := (others => '0');
    signal g2_wh  : unsigned(14 downto 0) := (others => '1');
    signal g2_ri, p26_ri, p27_ri, p28_ri, p29_ri : integer range 0 to 15 := 0;
    signal g2_rh, p26_rh, p27_rh, p28_rh, p29_rh, p30_rh : std_logic := '0';
    signal p26_phi : unsigned(15 downto 0) := (others => '0');
    signal p26_wh  : unsigned(14 downto 0) := (others => '0');
    signal p27_lo, p27_hi : unsigned(15 downto 0) := (others => '0');
    signal p28_k   : std_logic := '0';
    signal p28_full, p29_full, p30_full, p31_full : std_logic := '0';
    signal p28_mh, p28_ml : unsigned(15 downto 0) := (others => '0');
    signal p29_n   : unsigned(15 downto 0) := (others => '0');
    signal p30_nsh : unsigned(9 downto 0) := (others => '0');
    signal p31_cp  : unsigned(10 downto 0) := (others => '0');
    signal p32_cov : unsigned(8 downto 0) := (others => '0');
    signal p33_fa  : unsigned(9 downto 0) := (others => '0');
    signal p33_fb  : unsigned(7 downto 0) := (others => '0');
    signal p33_fk  : unsigned(9 downto 0) := (others => '0');
    signal p34_y, p35_y, p35_z2, p36_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p36_ya  : unsigned(11 downto 0) := (others => '0');
    signal p37_yf, p38_ys : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal o_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- the luma ceiling under kerf coverage k / 16: f(256 - 16 k) = 64 + 54 m + m/2 + m/4, m = 16 - k
    type t_fk is array (0 to 31) of unsigned(9 downto 0);
    function f_fk return t_fk is
        variable r : t_fk;
        variable m : integer;
    begin
        for k in 0 to 31 loop
            if k < 16 then m := 16 - k; else m := 0; end if;
            r(k) := to_unsigned(64 + 54 * m + m / 2 + m / 4, 10);
        end loop;
        return r;
    end function;
    constant C_FK : t_fk := f_fk;

    -- filter width by half-octave index hw = idx - 32 (w = 2^(hw / 2) cycles per pixel, Q16):
    -- w / 2 (15 bits) & reciprocal shift ri (4 bits) & half-octave flag rh.  (Hypnos's S33 / S34.)
    type t_wt is array (0 to 31) of std_logic_vector(19 downto 0);
    function f_wt return t_wt is
        variable r : t_wt;
        variable hw, iw, hb, sh, w, vrr : integer;
    begin
        for i in 0 to 31 loop
            hw := i - 32;
            hb := i mod 2;
            iw := (hw - hb) / 2;
            if hb = 1 then w := 91; else w := 64; end if;
            sh := iw + 10;
            if sh >= 0 then w := w * (2 ** sh); else w := w / (2 ** (-sh)); end if;
            if w = 0 then w := 1; end if;
            vrr := -hw;
            r(i)(19 downto 5) := std_logic_vector(to_unsigned(w / 2, 15));
            r(i)(4 downto 1)  := std_logic_vector(to_unsigned((vrr / 2) mod 16, 4));
            if vrr mod 2 = 1 then r(i)(0) := '1'; else r(i)(0) := '0'; end if;
        end loop;
        return r;
    end function;
    constant C_WT : t_wt := f_wt;

    -- a + b or a - b on ONE carry chain (the low column makes the carry-in)
    function f_addsub(a, b : signed; sub : std_logic) return signed is
        variable v_a, v_b, v_r : signed(a'length downto 0);
    begin
        v_a(a'length downto 1) := a; v_a(0) := '1';
        if sub = '1' then v_b(a'length downto 1) := not b; v_b(0) := '1';
        else              v_b(a'length downto 1) := b;     v_b(0) := '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;

    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- a knob follows its register only past a +-1 count change (pot jitter is held, never eased)
    function f_hold(r : std_logic_vector(9 downto 0); h : unsigned(9 downto 0)) return unsigned is
        variable d : signed(10 downto 0);
    begin
        d := signed(resize(unsigned(r), 11)) - signed(resize(h, 11));
        if d > 1 or d < -1 then return unsigned(r); end if;
        return h;
    end function;

    -- Exact signed product a * m, radix-4 Booth on m (from Hypnos; checked there exhaustively)
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
        variable va : signed(a'length downto 0);
        variable vm : signed(m'length downto 0);
        variable vp : signed(NP + 1 downto 0);
    begin
        va := signed(resize(a, a'length + 1));
        vm := signed(resize(m, m'length + 1));
        vp := bmul(va, vm);
        return unsigned(vp(NP - 1 downto 0));
    end function;

begin

    ------------------------------------------------------------------------
    -- raster measurement + mode constants + GENERATED AVID (Hypnos: start 2
    -- or 3 clocks after the source's first avid rise, whichever keeps the
    -- hsync -> start parity at the source's usual value; fixed width).
    ------------------------------------------------------------------------
    p_measure : process(clk)
        variable v_wd : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;

            -- per-mode steps from the timing ID (static while a mode runs):
            -- s_ystp = field-line pitch in x pixels, s_oys = Osc step per line
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_ystp <= to_unsigned(569, 10); s_mcls <= "10"; s_oys <= to_unsigned(537, 10);  -- 480i
                when 4          => s_ystp <= to_unsigned(288, 10); s_mcls <= "10"; s_oys <= to_unsigned(272, 10);  -- 480p
                when 8          => s_ystp <= to_unsigned(480, 10); s_mcls <= "10"; s_oys <= to_unsigned(453, 10);  -- 576i
                when 12         => s_ystp <= to_unsigned(240, 10); s_mcls <= "10"; s_oys <= to_unsigned(226, 10);  -- 576p
                when 5 | 6 | 14 => s_ystp <= to_unsigned(256, 10); s_mcls <= "01"; s_oys <= to_unsigned(181, 10);  -- 720p
                when 1 | 2 | 10 => s_ystp <= to_unsigned(512, 10); s_mcls <= "00"; s_oys <= to_unsigned(242, 10);  -- 1080i
                when others     => s_ystp <= to_unsigned(256, 10); s_mcls <= "00"; s_oys <= to_unsigned(121, 10);  -- 1080p
            end case;
            -- log2 of the x step in virtual px: SD 1 octave, 720p log2(1.5)
            case s_mcls is
                when "10"   => s_ls14 <= to_unsigned(16384, 15); s_oxs <= to_unsigned(204, 9); s_n64 <= '0';
                when "01"   => s_ls14 <= to_unsigned(9585, 15);  s_oxs <= to_unsigned(153, 9); s_n64 <= '0';
                when others => s_ls14 <= to_unsigned(0, 15);     s_oxs <= to_unsigned(102, 9); s_n64 <= '1';
            end case;

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then                 -- avid just fell
                -- hysteresis: analog line lengths wander +-1 px (keep the width within +-2)
                v_wd := signed(resize(s_xcnt, 13)) - signed(resize(s_W, 13));
                if s_xcnt(11 downto 5) /= 0 and not ((v_wd(12 downto 2) = 0 and v_wd(1 downto 0) /= "11") or v_wd(12 downto 1) = -1) then
                    s_W <= s_xcnt;
                end if;
                s_xcnt <= (others => '0');
                s_lacc <= s_lacc + s_ystp;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_saw_avid = '1' then
                -- field height in x-pixel units (1080p: the line count)
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
                g_cnt  <= s_Wl;
            elsif g_avid = '1' then
                if g_cnt(11 downto 1) = 0 then g_avid <= '0'; end if;      -- high for s_Wl clocks
                g_cnt <= g_cnt - 1;
            end if;

            gd(1) <= g_avid;
            for i in 2 to C_LAT loop
                gd(i) <= gd(i - 1);
            end loop;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-field control latch + sequencer.  The field tick is the first vsync
    -- edge after active video (analog vsync serrates).  One small operation
    -- per state; every product goes through the one serial multiplier (the
    -- sequencer waits while it runs); a table value is taken three states
    -- after its address, which nothing changes in between.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_k    : signed(10 downto 0);
        variable v_gc   : signed(21 downto 0);
        variable v_gdl  : signed(21 downto 0);
        variable v_gt   : signed(21 downto 0);
        variable v_T    : signed(22 downto 0);
        variable v_m    : unsigned(9 downto 0);
        variable v_g    : unsigned(16 downto 0);
        variable v_gk   : unsigned(15 downto 0);
        variable v_g3   : unsigned(17 downto 0);
        variable v_e0   : signed(19 downto 0);
        variable v_kp   : signed(19 downto 0);
        variable v_h    : signed(11 downto 0);
        variable v_az, v_a1 : std_logic;
        variable v_hb   : integer range -64 to 63;
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            sm_go  <= '0';
            s_done <= '0';
            tw_we  <= '0';
            wt_we  <= '0';
            s_wlv  <= '0';
            s_wlu  <= '0';
            tab_r  <= unsigned(tab_q);
            if data_in.avid = '1' then s_saw_avid <= '1'; end if;

            -- the +-S step of the scratch register: toward [0, S)
            if s_R(22) = '1' then v_T := s_R + resize(c_S, 23);
            else                  v_T := s_R - resize(c_S, 23); end if;

            case s_gch is
                when "00"   => v_gc := s_cxs;
                when "01"   => v_gc := s_cys;
                when "10"   => v_gc := s_axs;
                when others => v_gc := s_ays;
            end case;
            -- glide target: the product, Q8 px (centre: W (128 + k) ; slip: >> 7)
            if s_gch(1) = '0' then v_gt := sm_acc(21 downto 0); else v_gt := sm_acc(28 downto 7); end if;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_saw_avid = '1' then
                s_saw_avid <= '0';
                s_k1  <= f_hold(registers_in(0), s_k1);
                s_k2  <= f_hold(registers_in(1), s_k2);
                s_k3  <= f_hold(registers_in(2), s_k3);
                s_k4  <= f_hold(registers_in(3), s_k4);
                s_k5  <= f_hold(registers_in(4), s_k5);
                s_k6  <= f_hold(registers_in(5), s_k6);
                s_k12 <= f_hold(registers_in(7), s_k12);
                s_glide <= registers_in(6)(0);            -- S7 Glide
                s_tiles <= registers_in(6)(1);            -- S8 Cut: Strips / Tiles
                s_stair <= registers_in(6)(2);            -- S9 Order: Alternate / Staircase
                s_fine  <= registers_in(6)(3);            -- S10 Weight: Bold / Fine
                s_osc   <= registers_in(6)(4);            -- S11 Source: Video / Osc
                s_Wl    <= s_W;
                s_kn    <= "00";
                s_pc    <= to_unsigned(1, 7);

            elsif s_pc /= 0 and data_in.avid = '0' and sm_go = '0' and sm_n = 0 then
                s_pc <= s_pc + 1;
                sm_neg  <= '0';
                sm_keep <= '0';
                case to_integer(s_pc) is

                    -- CENTRE: W (128 + (K1 - 512)) / 256 = W/2 + (K1 - 512) W / 256, Q8 px
                    when 1 | 5 =>
                        if s_pc = 1 then v_k := signed(resize(s_k1, 11)) - 512; sm_bi <= s_Wl; s_gch <= "00";
                        else             v_k := signed(resize(s_k2, 11)) - 512; sm_bi <= s_H;  s_gch <= "01"; end if;
                        if v_k < C_DB and v_k > -C_DB then v_k := (others => '0'); end if;
                        sm_ai <= resize(v_k + 128, 17); sm_go <= '1';
                    -- the glide (four channels share it): ease 1/16 of the gap, never slower
                    -- than 1 px per field, land exactly; S7 off = snap (c + (tg - c) = tg)
                    when 2 | 6 | 59 | 63 =>
                        s_gd <= v_gt - v_gc;
                    when 3 | 7 | 60 | 64 =>
                        -- within 1 px: land ; within 16 px: 1 px per field
                        if s_glide = '0' or s_gd(21 downto 8) = 0 or s_gd(21 downto 8) = -1 then s_gs <= '1'; else s_gs <= '0'; end if;
                        if s_gd(21 downto 12) = 0 or s_gd(21 downto 12) = -1 then s_gmm <= '1'; else s_gmm <= '0'; end if;
                        s_gdd <= shift_right(s_gd, C_GL);
                    when 4 | 8 | 61 | 65 =>
                        if    s_gs = '1' then v_gdl := s_gd;
                        elsif s_gmm = '1' then
                            if s_gd(21) = '0' then v_gdl := to_signed(256, 22); else v_gdl := to_signed(-256, 22); end if;
                        else  v_gdl := s_gdd; end if;
                        case s_gch is
                            when "00"   => s_cxs <= v_gc + v_gdl;
                            when "01"   => s_cys <= v_gc + v_gdl;
                            when "10"   => s_axs <= v_gc + v_gdl;
                            when others => s_ays <= v_gc + v_gdl;
                        end case;

                    -- KNOB GLIDES (K3 Period, K4 Slices, K5 Angle, K6 Camera), Q6: 1/16 of the gap
                    -- per field; S7 off = the held knob itself
                    when 9 =>
                        case s_kn is
                            when "00"   => s_gm <= s_g3; s_gr <= s_k3;
                            when "01"   => s_gm <= s_g4; s_gr <= s_k4;
                            when "10"   => s_gm <= s_g5; s_gr <= s_k5;
                            when others => s_gm <= s_g6; s_gr <= s_k6;
                        end case;
                    when 10 =>
                        s_kd <= signed(shift_left(resize(s_gr, 17), 6)) - signed(resize(s_gm, 17));
                    when 11 =>
                        if s_glide = '1' then v_g := unsigned(signed(resize(s_gm, 17)) + shift_right(s_kd, C_GL));
                        else                  v_g := shift_left(resize(s_gr, 17), 6); end if;
                        case s_kn is
                            when "00"   => s_g3 <= v_g(15 downto 0);
                            when "01"   => s_g4 <= v_g(15 downto 0);
                            when "10"   => s_g5 <= v_g(15 downto 0);
                            when others => s_g6 <= v_g(15 downto 0);
                        end case;
                        if s_kn /= 3 then
                            s_kn <= s_kn + 1;
                            s_pc <= to_unsigned(9, 7);
                        end if;

                    -- PERIOD: log2 cycles / virtual px = -8.807 + 5 K3 / 1024 (448 .. 14 px), Q14, plus
                    -- log2 of the x step (per real pixel).  SLICES: log2 strip height = 9.322 - 5 K4 /
                    -- 1024 (640 .. 20 virtual px), in real pixels
                    when 12 | 16 =>
                        if s_pc = 12 then v_gk := s_g3; else v_gk := s_g4; end if;
                        s_t <= resize(v_gk, 17) + resize(shift_right(v_gk, 2), 17);
                    when 13 =>
                        case s_mcls is
                            when "10"   => v_kp := to_signed(-144296 + 16384, 20);
                            when "01"   => v_kp := to_signed(-144296 + 9585, 20);
                            when others => v_kp := to_signed(-144296, 20);
                        end case;
                        c_E0 <= v_kp + signed(resize(s_t, 20));
                    when 14 =>
                        v_e0 := -c_E0;
                        s_pe <= unsigned(v_e0(17 downto 0));
                        v_h := c_E0(19 downto 8) + 31;
                        c_hw0 <= v_h(10 downto 5);
                    when 15 =>
                        s_ip   <= s_pe(17 downto 14);
                        s_xad  <= s_pe(13 downto 6);                         -- period mantissa (the pixel path's
                        s_xsel <= '1';                                       -- 2^x table, idle in vblank)
                    when 17 =>
                        case s_mcls is
                            when "10"   => v_kp := to_signed(152731 - 16384, 20);
                            when "01"   => v_kp := to_signed(152731 - 9585, 20);
                            when others => v_kp := to_signed(152731, 20);
                        end case;
                        s_lsz <= v_kp - signed(resize(s_t, 20));
                    -- ANGLE: +-90 degrees about horizontal, 1024 steps (K5 - 512, +-6 dead centre)
                    when 18 =>
                        s_aneg <= not s_g5(15);
                        if s_g5(15) = '1' then v_m := resize(s_g5(14 downto 6), 10); else v_m := resize(not s_g5(14 downto 6), 10); end if;
                        if v_m < 6 then s_ai <= (others => '0'); else s_ai <= v_m - 5; end if;
                    when 19 =>
                        s_pm  <= unsigned(p22_m);
                        s_is  <= unsigned(s_lsz(17 downto 14));
                        s_xad <= unsigned(s_lsz(13 downto 6));               -- strip mantissa
                    when 20 => tab_a <= s_ai(8 downto 0);                    -- sin
                    when 23 =>
                        s_shr <= resize(unsigned(p22_m), 22);                -- strip mantissa, Q14
                        s_shn <= s_is - 3;
                        s_xsel <= '0';
                        s_sn  <= tab_r(14 downto 0);
                        tab_a <= not s_ai(8 downto 0);                       -- cos = sin(90 - a), one step short
                    when 24 =>                                           -- S = mantissa << (int - 3), Q11 px
                        if s_shn /= 0 then
                            s_shr <= shift_left(s_shr, 1);
                            s_shn <= s_shn - 1;
                            s_pc  <= s_pc;
                        else
                            c_S <= signed(s_shr);
                        end if;
                    when 26 => s_cs <= tab_r(14 downto 0);
                    when 34 =>
                        if s_aneg = '1' then
                            s_sns <= -signed(resize(s_sn, 17));
                            c_nx  <= signed(resize(s_sn(14 downto 3), 13));
                        else
                            s_sns <= signed(resize(s_sn, 17));
                            c_nx  <= -signed(resize(s_sn(14 downto 3), 13));
                        end if;
                        c_tx <= s_cs(14 downto 3);
                    -- line steps
                    when 35 => sm_ai <= signed(resize(s_cs, 17)); sm_bi <= resize(s_ystp, 12); sm_go <= '1';
                    when 36 =>
                        c_dvl <= unsigned(sm_acc(23 downto 11));
                        sm_ai <= s_sns; sm_bi <= resize(s_ystp, 12); sm_go <= '1';
                    -- the anchor: a cut through the screen centre.  v0 = (W sin - H cos) / 2,
                    -- u0 = -(W cos + H sin) / 2 at the first pixel, then into [0, S) counting strips
                    when 37 =>
                        c_dul <= sm_acc(24 downto 11);
                        sm_ai <= s_sns; sm_bi <= s_Wl; sm_go <= '1';
                    when 38 => sm_ai <= signed(resize(s_cs, 17)); sm_bi <= s_H; sm_neg <= '1'; sm_keep <= '1'; sm_go <= '1';
                    when 39 | 43 =>
                        s_R  <= sm_acc(26 downto 4);
                        s_rc <= "100";
                    when 40 | 44 =>
                        if s_R(22) = '1' then
                            s_R <= v_T; s_rc <= s_rc - 1; s_pc <= s_pc;
                        elsif v_T(22) = '0' then
                            s_R <= v_T; s_rc <= s_rc + 1; s_pc <= s_pc;
                        elsif s_pc = 40 then
                            s_wlv <= '1';
                        else
                            s_wlu <= '1';
                        end if;
                    when 41 => sm_ai <= signed(resize(s_cs, 17)); sm_bi <= s_Wl; sm_neg <= '1'; sm_go <= '1';
                    when 42 => sm_ai <= s_sns; sm_bi <= s_H; sm_neg <= '1'; sm_keep <= '1'; sm_go <= '1';
                    -- the wrapped steps: step - S for a positive step, step + S for a negative one
                    when 45 => s_R <= resize(c_nx, 23);
                    when 46 | 48 => s_R <= v_T;
                    when 47 => bv_i1 <= s_R(21 downto 0); s_R <= signed(resize(c_tx, 23));
                    when 49 => bu_i1 <= s_R(21 downto 0);

                    -- SLIP (P12, bipolar, +-12 dead centre): +-3.9 ring periods, in pixels along the cut
                    when 54 =>
                        s_sneg <= not s_k12(9);
                        if s_k12(9) = '1' then v_m := resize(s_k12(8 downto 0), 10); else v_m := resize(not s_k12(8 downto 0), 10); end if;
                        if v_m <= 12 then s_sq <= (others => '0');
                        elsif s_stair = '1' then s_sq <= shift_right(v_m - 12, 1);     -- Staircase: half a slip per strip
                        else s_sq <= v_m - 12; end if;
                    when 55 => sm_ai <= signed(resize(s_pm, 17)); sm_bi <= resize(s_sq, 12); sm_go <= '1';
                    when 56 =>
                        s_shr <= resize(unsigned(sm_acc(25 downto 12)), 22);
                        s_shn <= 8 - s_ip;
                    when 57 =>
                        if s_shn /= 0 then
                            s_shr <= shift_right(s_shr, 1);
                            s_shn <= s_shn - 1;
                            s_pc  <= s_pc;
                        else
                            s_sp <= s_shr(11 downto 0);                  -- |slip| in px, Q1
                        end if;
                    when 58 => sm_ai <= signed(resize(s_cs, 17)); sm_bi <= s_sp; sm_neg <= s_sneg; sm_go <= '1'; s_gch <= "10";
                    when 62 => sm_ai <= s_sns; sm_bi <= s_sp; sm_neg <= s_sneg; sm_go <= '1'; s_gch <= "11";
                    when 66 =>
                        s_ax <= s_axs(19 downto 8);
                        s_ay <= s_ays(19 downto 8);
                        s_tc <= "000";
                        s_ty <= '0';
                    -- shift table, 8 classes x (x, y): Alternate -A, +A, ... = 2 (c mod 2) A - A ;
                    -- Staircase (c - 4) A = c A - 4 A
                    when 67 | 68 =>
                        if s_ty = '0' then sm_ai <= resize(s_ax, 17); else sm_ai <= resize(s_ay, 17); end if;
                        sm_bi <= (others => '0');
                        if s_pc = 67 then
                            if s_stair = '1' then sm_bi(2 downto 0) <= s_tc; else sm_bi(1) <= s_tc(0); end if;
                        else
                            if s_stair = '1' then sm_bi(2) <= '1'; else sm_bi(0) <= '1'; end if;
                            sm_neg <= '1'; sm_keep <= '1';
                        end if;
                        sm_go <= '1';
                    when 69 =>
                        if s_ty = '0' then
                            tw_dx <= std_logic_vector(sm_acc(15 downto 0));
                            s_ty  <= '1';
                            s_pc  <= to_unsigned(67, 7);
                        else
                            tw_dy <= std_logic_vector(sm_acc(15 downto 0));
                            tw_we <= '1';
                            tw_a  <= s_tc;
                            s_ty  <= '0';
                            s_tc  <= s_tc + 1;
                            if s_tc /= 7 then s_pc <= to_unsigned(67, 7); end if;
                        end if;
                    -- the kerf opens with the first 2 px of slip
                    when 70 =>
                        if s_ax = 0 and s_ay = 0 then v_az := '1'; else v_az := '0'; end if;
                        if (s_ax(11 downto 1) = 0 or s_ax = -1) and (s_ay(11 downto 1) = 0 or s_ay = -1) then v_a1 := '1'; else v_a1 := '0'; end if;
                        if    v_az = '1' then c_kc0 <= to_signed(-16, 6);
                        elsif v_a1 = '1' then c_kc0 <= to_signed(3, 6);
                        else                  c_kc0 <= to_signed(22, 6); end if;
                    -- CAMERA gain G = 3 K6 / 8 (2.56 cycles at white), and its log2 in half octaves
                    when 71 =>
                        v_g3 := resize(s_g6, 18) + shift_left(resize(s_g6, 18), 1);
                        c_G <= v_g3(17 downto 9);
                        s_nrm <= s_g6(15 downto 6);
                        s_nn  <= to_unsigned(9, 4);
                        if s_g6(15 downto 6) = 0 then c_camon <= '0'; else c_camon <= '1'; end if;
                    when 72 =>
                        if s_nrm(9) = '0' and s_nn /= 0 then
                            s_nrm <= shift_left(s_nrm, 1);
                            s_nn  <= s_nn - 1;
                            s_pc  <= s_pc;
                        end if;
                    when 73 =>
                        -- ceil((64 n + lut(mantissa) - 1339) / 32) = 2 n - 41 (+ 1 from mantissa 1.375)
                        v_hb := 2 * to_integer(s_nn) - 41;
                        if s_nrm(8 downto 6) >= 3 then v_hb := v_hb + 1; end if;
                        c_hb0 <= to_signed(v_hb, 7);
                        wt_i <= (others => '0');
                        if s_fine = '1' then wt_acc <= to_unsigned(5461, 16); c_D16 <= to_unsigned(10923, 16);
                        else                 wt_acc <= to_unsigned(16384, 16); c_D16 <= to_unsigned(32768, 16); end if;
                    -- camera phase table: half the duty + G x level / 8, per whole level
                    when 74 =>
                        if wt_i(8) = '0' then
                            wt_we  <= '1';
                            wt_wa  <= wt_i(7 downto 0);
                            tw_dx  <= std_logic_vector(wt_acc);          -- (the shift table's data register, idle here)
                            wt_acc <= wt_acc + shift_left(resize(c_G, 16), 1);
                            wt_i   <= wt_i + 1;
                            s_pc   <= s_pc;
                        end if;
                    when 75 =>
                        s_cx <= s_cxs(21 downto 8);
                        s_cy <= s_cys(21 downto 8);
                        s_done <= '1';
                        s_pc   <= (others => '0');

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    p_smul : process(clk)
    begin
        if rising_edge(clk) then
            if sm_go = '1' then
                sm_a <= resize(sm_ai, 29);
                sm_s <= sm_neg;
                sm_b <= sm_bi;
                if sm_keep = '0' then sm_acc <= (others => '0'); end if;
                sm_n <= to_unsigned(12, 4);
            elsif sm_n /= 0 then
                if sm_b(0) = '1' then sm_acc <= f_addsub(sm_acc, resize(sm_a, 30), sm_s); end if;
                sm_a <= shift_left(sm_a, 1);
                sm_b <= shift_right(sm_b, 1);
                sm_n <= sm_n - 1;
            end if;
        end if;
    end process p_smul;

    ------------------------------------------------------------------------
    -- STRIPS.  P0: the beam accumulators (distance into the strip, Q11 px)
    -- step per pixel and wrap at the strip height, counting the class; the
    -- line registers step once per line at the end of the generated avid.
    -- P1 .. P3: class -> shift table (block RAM, fabric register, the Tiles
    -- sum).  P1 .. P4: distance to the nearest cut -> kerf coverage.
    ------------------------------------------------------------------------
    p_strip : process(clk)
        variable v_a0, v_a1 : signed(21 downto 0);
        variable v_wr, v_la : std_logic;
        variable v_e  : signed(7 downto 0);
        variable v_dd : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            -- v: across the cut
            v_a0 := bv + resize(c_nx, 22);
            v_a1 := bv + bv_i1;
            if c_nx(12) = '1' then v_wr := v_a0(21); else v_wr := not v_a1(21); end if;
            if g_avid = '1' and gd(1) = '0' then
                bv <= lv; bvc <= lvc;
            elsif g_avid = '1' then
                if v_wr = '1' then
                    bv <= v_a1;
                    if c_nx(12) = '1' then bvc <= bvc - 1; else bvc <= bvc + 1; end if;
                else
                    bv <= v_a0;
                end if;
            end if;
            -- u: along the cut (its step per pixel is never negative)
            v_a0 := bu + signed(resize(c_tx, 22));
            v_a1 := bu + bu_i1;
            if g_avid = '1' and gd(1) = '0' then
                bu <= lu; buc <= luc;
            elsif g_avid = '1' then
                if v_a1(21) = '0' then bu <= v_a1; buc <= buc + 1;
                else                   bu <= v_a0; end if;
            end if;
            -- line registers (the sequencer writes the first line's values in vblank): at the end of
            -- each line add the line step, then in the next clock wrap into [0, S)
            if g_avid = '0' and gd(1) = '1' then v_la := '1'; else v_la := '0'; end if;
            s_lpb <= v_la;
            if v_la = '1' then v_a0 := signed(resize(c_dvl, 22)); else v_a0 := c_S; end if;
            v_a1 := f_addsub(lv, v_a0, s_lpb);
            if s_wlv = '1' then
                lv <= s_R(21 downto 0); lvc <= s_rc;
            elsif v_la = '1' then
                lv <= v_a1;
            elsif s_lpb = '1' and v_a1(21) = '0' then
                lv <= v_a1; lvc <= lvc + 1;
            end if;
            if v_la = '1' then v_a0 := resize(c_dul, 22); else v_a0 := c_S; end if;
            v_a1 := f_addsub(lu, v_a0, s_lpb and not lu(21));
            if s_wlu = '1' then
                lu <= s_R(21 downto 0); luc <= s_rc;
            elsif v_la = '1' then
                lu <= v_a1;
            elsif s_lpb = '1' and lu(21) = '1' then
                lu <= v_a1; luc <= luc - 1;
            elsif s_lpb = '1' and v_a1(21) = '0' then
                lu <= v_a1; luc <= luc + 1;
            end if;

            -- shift tables (one write each, registered read, then a fabric register)
            if tw_we = '1' then txk_ram(to_integer(tw_a)) <= tw_dx; end if;
            if tw_we = '1' then txj_ram(to_integer(tw_a)) <= tw_dx; end if;
            if tw_we = '1' then tyk_ram(to_integer(tw_a)) <= tw_dy; end if;
            if tw_we = '1' then tyj_ram(to_integer(tw_a)) <= tw_dy; end if;
            txk_q <= txk_ram(to_integer(bvc));
            tyk_q <= tyk_ram(to_integer(bvc));
            txj_q <= txj_ram(to_integer(buc));
            tyj_q <= tyj_ram(to_integer(buc));
            p2_txk <= signed(txk_q(14 downto 0)); p2_tyk <= signed(tyk_q(14 downto 0));
            p2_txj <= signed(txj_q(14 downto 0)); p2_tyj <= signed(tyj_q(14 downto 0));
            -- P3: rows slide along the cut; Tiles: columns slide across it (the u class turned a quarter)
            if s_tiles = '1' then
                p3_sx <= resize(p2_txk, 16) - resize(p2_tyj, 16);
                p3_sy <= resize(p2_tyk, 16) + resize(p2_txj, 16);
            else
                p3_sx <= resize(p2_txk, 16);
                p3_sy <= resize(p2_tyk, 16);
            end if;

            -- kerf: distance to the nearest cut, Q4 px, held at 63 past 4 px
            v_dd := unsigned(bv(21 downto 7));
            if v_dd(14 downto 6) /= 0 then p1_dav <= (others => '1'); else p1_dav <= v_dd(5 downto 0); end if;
            v_dd := unsigned(c_S(21 downto 7)) - unsigned(bv(21 downto 7));
            if v_dd(14 downto 6) /= 0 then p1_dbv <= (others => '1'); else p1_dbv <= v_dd(5 downto 0); end if;
            v_dd := unsigned(bu(21 downto 7));
            if v_dd(14 downto 6) /= 0 then p1_dau <= (others => '1'); else p1_dau <= v_dd(5 downto 0); end if;
            v_dd := unsigned(c_S(21 downto 7)) - unsigned(bu(21 downto 7));
            if v_dd(14 downto 6) /= 0 then p1_dbu <= (others => '1'); else p1_dbu <= v_dd(5 downto 0); end if;
            if p1_dav < p1_dbv then p2_dv <= p1_dav; else p2_dv <= p1_dbv; end if;
            if s_tiles = '0' then p2_du <= (others => '1');
            elsif p1_dau < p1_dbu then p2_du <= p1_dau;
            else p2_du <= p1_dbu; end if;
            if p2_dv < p2_du then p3_d <= p2_dv; else p3_d <= p2_du; end if;
            -- P4: coverage 0 .. 16 = clamp(kc0 - d)
            v_e := resize(c_kc0, 8) - signed(resize(p3_d, 8));
            if v_e(7) = '1' then p4_kc <= (others => '0');
            elsif v_e > 16 then p4_kc <= to_unsigned(16, 5);
            else p4_kc <= unsigned(v_e(4 downto 0)); end if;
        end if;
    end process p_strip;

    ------------------------------------------------------------------------
    -- P3: coordinate accumulators.  dx loads -cx at each line start and steps
    -- per pixel; dy seeds at the first line after the sequencer and steps per
    -- line by s_ystp (Q8 x-pixel units, +0.5 seed).  (Hypnos)
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            if s_done = '1' then
                s_newframe <= '1';
            elsif gd(3) = '1' and gd(4) = '0' then
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
            elsif gd(3) = '1' then
                s_dx <= s_dx + 1;
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- EBR delay lines (write address = free-running counter; each read runs
    -- X behind it) and the table ROMs: every address is a register, every
    -- read is registered and then re-registered in fabric before any logic.
    ------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_wa <= s_wa + 1;
            dlh(to_integer(s_wa)) <= std_logic_vector(resize(p13_lhe, 32));
            dlh_q <= dlh(to_integer(s_wa - C_XH));
            dlh_r <= signed(dlh_q(19 downto 0));
            dlk(to_integer(s_wa)) <= "00000000000" & std_logic_vector(p4_kc);
            dlk_q <= dlk(to_integer(s_wa - C_XK));
            dlk_r <= unsigned(dlk_q(4 downto 0));
            dls(to_integer(s_wa)) <= "0000000000000" & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            dls_q <= dls(to_integer(s_wa - (C_LAT - 2)));
            dls_q2 <= dls_q(2 downto 0);
        end if;
    end process p_dl;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lvh_q <= C_FT_LOGV(to_integer(p8_ih));
            lsh_q <= C_FT_LOGS(to_integer(p8_ih));
            lvl_q <= C_FT_LOGV(to_integer(p8_il));
            lsl_q <= C_FT_LOGS(to_integer(p8_il));
            ug_q  <= C_FT_UG(to_integer(p14_ia));
            ugs_q <= C_FT_UGS(to_integer(p14_ia));
            exm_q <= C_FT_EXM(to_integer(p20_xa));
            exs_q <= C_FT_EXS(to_integer(p20_xa));
            tab_q <= C_FT_TAB(to_integer(tab_a));
        end if;
    end process p_rom;

    ------------------------------------------------------------------------
    -- P4 .. P19: shifted coordinates -> hi / lo -> log2 (priority encode,
    -- normalise, 256-segment PWL) -> u = log2(hi / lo) -> 0.5 log2(1 + 4^-u)
    -- -> log2 r + E0 = log2 of the phase in cycles, Q14.  (Hypnos, 14 bits.)
    ------------------------------------------------------------------------
    p_polar : process(clk)
        variable va, vb : unsigned(15 downto 0);
        variable vh, vl : unsigned(13 downto 0);
        variable mh, ml : integer range 0 to 13;
        variable vu     : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- P4: the strip's shift, added before the distance is taken
            p4_dx <= resize(s_dx, 16) + p3_sx;
            p4_dy <= resize(s_dyc, 16) + p3_sy;
            -- P5
            va := unsigned(abs(p4_dx));
            vb := unsigned(abs(p4_dy));
            if va(15 downto 14) /= "00" then p5_a <= (others => '1'); else p5_a <= va(13 downto 0); end if;
            if vb(15 downto 14) /= "00" then p5_b <= (others => '1'); else p5_b <= vb(13 downto 0); end if;
            -- P6
            if p5_a >= p5_b then p6_hi <= p5_a; p6_lo <= p5_b;
            else                 p6_hi <= p5_b; p6_lo <= p5_a; end if;
            if p5_a = 0 or p5_b = 0 then lzsr(6) <= '1'; else lzsr(6) <= '0'; end if;
            for i in 7 to 12 loop lzsr(i) <= lzsr(i - 1); end loop;
            -- P7: msb (the exact centre reads as 1)
            vh := p6_hi; if vh = 0 then vh := to_unsigned(1, 14); end if;
            vl := p6_lo; if vl = 0 then vl := to_unsigned(1, 14); end if;
            mh := 0; ml := 0;
            for i in 0 to 13 loop
                if vh(i) = '1' then mh := i; end if;
                if vl(i) = '1' then ml := i; end if;
            end loop;
            p7_hi <= vh; p7_lo <= vl;
            p7_mh <= to_unsigned(mh, 4); p7_ml <= to_unsigned(ml, 4);
            -- P8: normalise -> 8-bit segment + 5-bit fraction
            vh := shift_left(p7_hi, 13 - to_integer(p7_mh));
            vl := shift_left(p7_lo, 13 - to_integer(p7_ml));
            p8_ih <= vh(12 downto 5); p8_th <= vh(4 downto 0);
            p8_il <= vl(12 downto 5); p8_tl <= vl(4 downto 0);
            p8_mh <= p7_mh; p8_ml <= p7_ml;
            -- P9, P10 (EBR read, fabric register)
            p9_th <= p8_th; p9_tl <= p8_tl; p9_mh <= p8_mh; p9_ml <= p8_ml;
            p10_th <= p9_th; p10_tl <= p9_tl; p10_mh <= p9_mh; p10_ml <= p9_ml;
            p10_lvh <= lvh_q; p10_lsh <= lsh_q; p10_lvl <= lvl_q; p10_lsl <= lsl_q;
            -- P11
            p11_lph <= unsigned(p10_lsh(6 downto 0)) * p10_th;
            p11_lpl <= unsigned(p10_lsl(6 downto 0)) * p10_tl;
            p11_lvh <= unsigned(p10_lvh(13 downto 0));
            p11_lvl <= unsigned(p10_lvl(13 downto 0));
            p11_mh <= p10_mh; p11_ml <= p10_ml;
            -- P12: log2 = msb + PWL(mantissa), Q14
            p12_lh <= shift_left(resize(p11_mh, 18), 14) + resize(p11_lvh, 18) + resize(shift_right(p11_lph, 5), 18);
            p12_ll <= shift_left(resize(p11_ml, 18), 14) + resize(p11_lvl, 18) + resize(shift_right(p11_lpl, 5), 18);
            -- P13: u ; log2 hi + E0 rides the delay line
            vu := p12_lh - p12_ll;
            if lzsr(12) = '1' then vu := to_unsigned(16 * 16384 - 1, 18); end if;
            p13_u   <= vu;
            p13_lhe <= signed(resize(p12_lh, 20)) + c_E0;
            -- P14: 1/16-octave segment + 7-bit fraction
            p14_ia <= p13_u(17 downto 10);
            p14_tu <= p13_u(9 downto 3);
            -- P15, P16 (EBR read, fabric register)
            p15_tu <= p14_tu;
            p16_tu <= p15_tu;
            p16_g <= ug_q; p16_gs <= ugs_q;
            -- P17: interpolation product (the slope is negative)
            p17_gp <= bmul(signed(p16_gs(10 downto 0)), signed(resize(p16_tu, 8)));
            p17_g  <= unsigned(p16_g);
            -- P18
            p18_g <= resize(unsigned(signed(resize(p17_g, 17)) + resize(shift_right(p17_gp, 7), 17)), 14);
            -- P19: e = log2 r + E0
            p19_e <= dlh_r + signed(resize(p18_g, 20));
        end if;
    end process p_polar;

    ------------------------------------------------------------------------
    -- P20 .. P25: phase = 2^e.  Mantissa from the EBR PWL table, then ONE
    -- left shift by (int e + 9) and a 16-bit window = the phase mod 1 cycle.
    ------------------------------------------------------------------------
    p_exp : process(clk)
        variable vi : signed(5 downto 0);
        variable vs : signed(6 downto 0);
        variable vw : unsigned(35 downto 0);
    begin
        if rising_edge(clk) then
            -- P20
            if s_xsel = '1' then p20_xa <= s_xad; else p20_xa <= unsigned(p19_e(13 downto 6)); end if;
            p20_t  <= unsigned(p19_e(5 downto 0));
            vi := p19_e(19 downto 14);
            vs := resize(vi, 7) + 9;
            if vs(6) = '1' then p20_z <= '1'; else p20_z <= '0'; end if;
            if vs > 19 then p20_sa <= to_unsigned(19, 5); else p20_sa <= unsigned(vs(4 downto 0)); end if;
            -- P21, P22 (EBR read, fabric register)
            p21_t <= p20_t; p21_sa <= p20_sa; p21_z <= p20_z;
            p22_t <= p21_t; p22_sa <= p21_sa; p22_z <= p21_z;
            p22_m <= exm_q; p22_s <= exs_q;
            -- P23
            p23_ep <= unsigned(p22_s(7 downto 0)) * p22_t;
            p23_m  <= unsigned(p22_m);
            p23_sa <= p22_sa; p23_z <= p22_z;
            -- P24
            p24_mt <= p23_m + resize(shift_right(p23_ep, 6), 16);
            p24_sa <= p23_sa; p24_z <= p23_z;
            -- P25
            vw := shift_left(resize(p24_mt, 36), to_integer(p24_sa));
            if p24_z = '1' then p25_phr <= (others => '0');
            else                p25_phr <= vw(22 downto 7); end if;
        end if;
    end process p_exp;

    ------------------------------------------------------------------------
    -- SOURCE FIELD.  Luma (or the Osc lattice) -> 64-px box (32 px outside
    -- 1080; edge-replicated and reloaded at every line start) -> vertical
    -- 1/32 one-pole in a line store with one entry per 4 pixels (Q8.4 level
    -- + a 4-bit half-octave code of the field's own gradient).  The pixel
    -- path reads the same entry the filter is about to update (the line
    -- above), interpolates across the 4 pixels and multiplies by the gain.
    ------------------------------------------------------------------------
    w_box12 <= s_bsum(13 downto 2) when s_n64 = '1' else s_bsum(12 downto 1);

    p_src : process(clk)
        variable v_l   : unsigned(7 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_dq, v_dh : signed(12 downto 0);
        variable v_qn  : signed(12 downto 0);
        variable v_g1, v_g0, v_ls : signed(14 downto 0);
        variable v_m   : integer range 0 to 7;
        variable v_c   : unsigned(3 downto 0);
        variable v_cm  : unsigned(3 downto 0);
        variable v_pa, v_pb : unsigned(12 downto 0);
        variable v_wt  : std_logic_vector(19 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting (the heirloom shell's)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avid_q = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_done = '1' then
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);
                s_ot <= s_ot + 1;
            end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s;

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_osc = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;

            -- box: sum of the last N samples; the samples before the line start count as the first one
            bx_q <= bx_ram(to_integer(bx_rp));
            bx_r <= bx_q;
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
            if s_sv = '1' then
                bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else
                bx_wp <= (others => '0');
                if s_n64 = '1' then bx_rp <= to_unsigned(194, 8); else bx_rp <= to_unsigned(226, 8); end if;
            end if;
            if (s_n64 = '1' and s_bcnt < 64) or (s_n64 = '0' and s_bcnt < 32) then v_tap := s_sfirst; else v_tap := bx_r; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                if s_n64 = '1' then s_bsum <= shift_left(resize(s_sl, 14), 6);
                else                s_bsum <= shift_left(resize(s_sl, 14), 5); end if;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 64 then s_bcnt <= s_bcnt + 1; end if;
            end if;

            -- entry clock: e_pc = index of the pixel at P18; one entry per 4 pixels
            if gd(19) = '1' then e_pc <= e_pc + 1; else e_pc <= (others => '0'); end if;
            e_f1 <= e_pc(1 downto 0); e_f2 <= e_f1;
            if gd(19) = '1' and e_pc(1 downto 0) = "00" then e_st(0) <= '1'; else e_st(0) <= '0'; end if;
            for i in 1 to 5 loop e_st(i) <= e_st(i - 1); end loop;
            lb_q <= lb_ram(to_integer(e_pc(10 downto 2)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
            if s_done = '1' then e_first <= '1';
            elsif gd(26) = '0' and gd(27) = '1' then e_first <= '0'; end if;

            -- I0 (the pixel path's P20): this entry and the one before it
            if e_st(0) = '1' then
                if e_first = '1' then
                    v_qn := signed(resize(w_box12, 13)); v_c := "0000";
                else
                    v_qn := signed(resize(unsigned(lb_q(15 downto 4)), 13)); v_c := unsigned(lb_q(3 downto 0));
                end if;
                e_idx <= e_pc(10 downto 2);
                if e_pc(10 downto 2) = 0 then
                    e_pq <= unsigned(v_qn(11 downto 0)); e_pcd <= v_c;
                else
                    e_pq <= e_cq; e_pcd <= e_cc;
                end if;
                e_cq <= unsigned(v_qn(11 downto 0)); e_cc <= v_c;
                e_qo  <= unsigned(lb_q(15 downto 4));
                e_bxr <= w_box12;
            end if;
            -- I1: the one-pole (the first line of a field loads the box itself)
            v_dq := signed(resize(e_bxr, 13)) - signed(resize(e_qo, 13));
            if e_first = '1' then
                e_qn <= e_bxr; e_dqs <= (others => '0');
            else
                v_qn := signed(resize(e_qo, 13)) + resize(shift_right(v_dq, 5), 13);
                e_qn <= unsigned(v_qn(11 downto 0));
                e_dqs <= resize(shift_right(v_dq, 5), 8);
            end if;
            -- I2: |vertical step|, |horizontal step| (per entry = per 4 px)
            e_dvm <= unsigned(abs(e_dqs));
            v_dh := signed(resize(e_qn, 13)) - signed(resize(e_qnp, 13));
            if v_dh(12) = '1' then v_dh := -v_dh; end if;
            if e_idx = 0 then e_dhm <= (others => '0');
            elsif v_dh(12 downto 8) /= 0 then e_dhm <= (others => '1');
            else e_dhm <= unsigned(v_dh(7 downto 0)); end if;
            if e_st(2) = '1' then e_idxw <= e_idx; end if;
            if e_st(4) = '1' then e_qnp <= e_qn; end if;
            -- I3: gradient, Q4 levels per px ; I4: its half-octave code 2 msb + next bit
            e_v <= resize(e_dvm, 9) + resize(shift_right(e_dhm, 2), 9);
            v_m := 0;
            for i in 0 to 7 loop
                if e_v(i) = '1' then v_m := i; end if;
            end loop;
            if e_v(8) = '1' then e_code <= "1111";
            elsif e_v < 2 then e_code <= "0000";
            else
                v_c := to_unsigned(2 * v_m, 4);
                if e_v(v_m - 1) = '1' then v_c := v_c + 1; end if;
                e_code <= v_c;
            end if;
            lb_we <= e_st(5);
            lb_wa <= e_idxw;
            lb_wd <= std_logic_vector(e_qn) & std_logic_vector(e_code);

            -- pixel path.  P21: step across the entry ; P22: level, Q8.6
            p21_diff <= signed(resize(e_cq, 13)) - signed(resize(e_pq, 13));
            p21_prev <= e_pq;
            p21_f    <= e_f2;
            if p21_f(1) = '1' then v_g1 := shift_left(resize(p21_diff, 15), 1); else v_g1 := (others => '0'); end if;
            if p21_f(0) = '1' then v_g0 := resize(p21_diff, 15); else v_g0 := (others => '0'); end if;
            v_ls := signed(shift_left(resize(p21_prev, 15), 2)) + v_g1 + v_g0;
            p22_L <= unsigned(v_ls(13 downto 0));                 -- a slice: a narrowing resize would fold it
            -- P23 .. P25: phase = table(whole level) + G x (level fraction) / 8  (the table holds half
            -- the duty + G x level / 8, so the band is centred on a whole phase)
            wt_q <= wt_ram(to_integer(p22_L(13 downto 6)));
            if wt_we = '1' then wt_ram(to_integer(wt_wa)) <= tw_dx; end if;
            if p22_L(5) = '1' then v_pa := shift_left(resize(c_G, 13), 3); else v_pa := (others => '0'); end if;
            if p22_L(4) = '1' then v_pb := shift_left(resize(c_G, 13), 2); else v_pb := (others => '0'); end if;
            p23_pa <= v_pa + v_pb;
            if p22_L(3) = '1' then v_pa := shift_left(resize(c_G, 13), 1); else v_pa := (others => '0'); end if;
            if p22_L(2) = '1' then v_pb := resize(c_G, 13); else v_pb := (others => '0'); end if;
            p23_pb <= v_pa + v_pb;
            p24_wt <= unsigned(wt_q);
            p24_pp <= p23_pa + p23_pb;
            p25_camd <= p24_wt + resize(shift_right(p24_pp, 3), 16);

            -- filter width, in half octaves: the ring gradient, or the camera field's own where it is
            -- steeper (the smaller code of two neighbours: a spike is not a slope).  hw = -32 .. -1.
            if e_cc < e_pcd then v_cm := e_cc; else v_cm := e_pcd; end if;
            g0_hb <= signed(resize(v_cm, 7)) + c_hb0;
            if v_cm > 2 and c_camon = '1' then g0_v <= '1'; else g0_v <= '0'; end if;
            if g0_v = '1' and g0_hb > resize(c_hw0, 7) then g1_hw <= unsigned(g0_hb(4 downto 0)); else g1_hw <= unsigned(c_hw0(4 downto 0)); end if;
            v_wt := C_WT(to_integer(g1_hw));
            g2_wh <= unsigned(v_wt(19 downto 5));
            g2_ri <= to_integer(unsigned(v_wt(4 downto 1)));
            g2_rh <= v_wt(0);
        end if;
    end process p_src;

    ------------------------------------------------------------------------
    -- P26 .. P32: one-pixel box filter of the band square-wave (Hypnos).
    --   band = frac in [0, D) ; P(x) = floor(x) D + min(frac x, D)
    --   coverage = (P(phi + w/2) - P(phi - w/2)) / w
    ------------------------------------------------------------------------
    p_cov : process(clk)
        variable vn  : signed(17 downto 0);
        variable vs  : unsigned(24 downto 0);
        variable vc  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- P26: ring phase + camera phase (+ D / 2)
            p26_phi <= p25_phr + p25_camd;
            p26_wh  <= g2_wh;
            p26_ri  <= g2_ri; p26_rh <= g2_rh;
            -- P27
            p27_lo <= p26_phi - resize(p26_wh, 16);
            p27_hi <= p26_phi + resize(p26_wh, 16);
            p27_ri <= p26_ri; p27_rh <= p26_rh;
            -- P28
            if p27_hi < p27_lo then p28_k <= '1'; else p28_k <= '0'; end if;
            if p27_hi >= p27_lo and p27_hi < c_D16 then p28_full <= '1'; else p28_full <= '0'; end if;
            if p27_hi < c_D16 then p28_mh <= p27_hi; else p28_mh <= c_D16; end if;
            if p27_lo < c_D16 then p28_ml <= p27_lo; else p28_ml <= c_D16; end if;
            p28_ri <= p27_ri; p28_rh <= p27_rh;
            -- P29
            vn := signed(resize(p28_mh, 18)) - signed(resize(p28_ml, 18));
            if p28_k = '1' then vn := vn + signed(resize(c_D16, 18)); end if;
            if vn < 0 then vn := (others => '0'); end if;
            p29_n    <= unsigned(vn(15 downto 0));
            p29_ri   <= p28_ri; p29_rh <= p28_rh;
            p29_full <= p28_full;
            -- P30: (N << Ri) >> 7
            vs := shift_left(resize(p29_n, 25), p29_ri);
            p30_nsh  <= vs(16 downto 7);
            p30_rh   <= p29_rh;
            p30_full <= p29_full;
            -- P31: / 2, or * sqrt2 / 2 on a half-octave step
            if p30_rh = '1' then
                p31_cp <= resize(shift_right(p30_nsh, 1) + shift_right(p30_nsh, 3)
                                 + shift_right(p30_nsh, 4) + shift_right(p30_nsh, 6), 11);
            else
                p31_cp <= resize(shift_right(p30_nsh, 1), 11);
            end if;
            p31_full <= p30_full;
            -- P32: coverage 0 .. 256
            vc := resize(p31_cp, 10);
            if vc > 256 or p31_full = '1' then vc := to_unsigned(256, 10); end if;
            p32_cov <= resize(vc, 9);
        end if;
    end process p_cov;

    ------------------------------------------------------------------------
    -- P33 .. P39: luma = 64 + 3.42 coverage (white 940), cut to the kerf's
    -- level; [1 2 1] / 4; 13/16 toward black (white about 775); blanking
    -- gate.  U = V = 512 always.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_s : unsigned(10 downto 0);
        variable v_y : unsigned(11 downto 0);
        variable v_d : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- P33: f(cov) = 64 + 3 c + c/4 + c/8 + c/32 + c/64 in two parts, and the kerf's ceiling
            -- f(256 - 16 k): the luma is the smaller of the two (f rises with coverage)
            p33_fa <= resize(shift_left(resize(p32_cov, 11), 1) + resize(p32_cov, 11), 10);
            p33_fb <= resize(to_unsigned(64, 8) + resize(shift_right(p32_cov, 2), 8) + resize(shift_right(p32_cov, 3), 8)
                             + resize(shift_right(p32_cov, 5), 8) + resize(shift_right(p32_cov, 6), 8), 8);
            p33_fk <= C_FK(to_integer(dlk_r));
            -- P34: black outside the line (the [1 2 1] sees clean edges)
            v_s := resize(p33_fa, 11) + resize(p33_fb, 11);
            if gd(34) = '0' then p34_y <= to_unsigned(64, 10);
            elsif v_s < resize(p33_fk, 11) then p34_y <= v_s(9 downto 0);
            else p34_y <= p33_fk; end if;
            -- P35, P36: taps y(n), y(n - 1), y(n - 2); the centre pixel n - 1 is at P36
            p35_y  <= p34_y;
            p35_z2 <= p35_y;
            p36_yb <= p35_y;
            p36_ya <= resize(p34_y, 12) + resize(p35_z2, 12) + 2;
            -- P37
            v_y := p36_ya + shift_left(resize(p36_yb, 12), 1);
            p37_yf <= v_y(11 downto 2);
            -- P38: 13/16 toward black
            if p37_yf > 64 then v_d := p37_yf - 64; else v_d := (others => '0'); end if;
            p38_ys <= p37_yf - shift_right(v_d, 3) - shift_right(v_d, 4);
            -- P39: blanking gate
            if gd(39) = '1' then o_y <= p38_ys; else o_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(C_MID);
    data_out.v       <= std_logic_vector(C_MID);
    data_out.avid    <= gd(C_LAT);
    data_out.hsync_n <= dls_q2(2);
    data_out.vsync_n <= dls_q2(1);
    data_out.field_n <= dls_q2(0);

end architecture fault;
