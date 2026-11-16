-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Flute v0.1.0 -- Hardline.  Reeded (fluted) architectural glass in front of a ringed or striped scene;
-- lenticular sheets.  Black and white, U = V = 512 on every pixel.
--
-- The screen is cut into parallel flutes (width K4, direction K5).  Each flute is a cylinder lens: with t the
-- offset from the flute's centre line along the flute normal n, the scene is sampled at
--
--     p' = p + (Power - 1) t n          Power 1 = flat glass, > 1 = squeezed copies, 0 = one tone across
--                                        the flute, < 0 = flipped copies
--
-- and the scene is concentric rings about X / Y (period K3) or 45-degree stripes through X / Y.  A thin
-- anti-aliased dark groove sits on every flute boundary and hides the jump.
--
--   K1 X, K2 Y   the scene's centre behind the glass (+-2 screens)        K3 Period   1024 .. 32 virtual px
--   K4 Flutes    flute width 16 .. 430 virtual px (exponential)           K5 Angle    -90 .. +90 deg, centre = vertical
--   K6 Camera    luma -> scene phase (2 periods at white at 100 %)        S7 Glide    Off = no easing at all
--   S8 Scene     Rings / Stripes                                          S9 Shade    Flat / Shade (glass rods)
--   S10 Weight   Bold / Fine (Shade on: 4 steps / smooth)                 S11 Source  Video / Osc
--   P12 POWER    bipolar: centre = flat glass, up to x4.9, down through 0 to x-3.9
--
-- Engine (the integer model .hardline_work/rb_flute/imodel.py mirrors it):
--   per field  a small microcoded engine started at the vblank zone (flute.py holds the microprogram and its
--              emulator): 37-bit accumulator, 64 x 32-bit register file in block RAM, one adder, a serial
--              18 x 18 multiplier and a serial right shift.  It does the knob holds and glides, the sine / exp
--              lookups with interpolation, three log2s, the flute phase at the top-left pixel (a remainder)
--              and every DDA constant.  Everything is in x-pixel units; the timing ID gives the line pitch
--              and log2 of the x step, so SD, 720p and 1080p show the same picture.
--   per line   three adds (flute offset with wrap, scene normal coordinate, scene tangent coordinate).
--   per pixel  flute offset u (wraps by the flute width), scene coordinates (q, v) in the FLUTE frame as DDAs
--              (q jumps by (Power - 1) W at each wrap), then the Hypnos log-domain polar engine:
--              log2 hi, log2 lo -> u = log2(hi / lo) -> log2 r = log2 hi + g(u) -> exp2 -> ONE shift = phase.
--              The frequency's fractional octaves ride in the exponent, so there is no phase multiplier.
--              Stripes run the same path with q = the stripe coordinate and v = 0.
--   AA         Hypnos's one-pixel box filter of the band square wave.  The phase gradient is analytic and
--              includes the lens: tangent term Lf + log2|v| - log2 r, normal term Lf + log2|Power| + log2|q| -
--              log2 r (both fall out of the engine as -g and -g - u), combined as max + 1/2 log2(1 + 2^-2|a-b|).
--   tone       coverage x flute factor (groove coverage / 4 anti-aliased steps / smooth triangle) -> luma
--              -> [1 2 1] / 4 -> 13/16 toward black -> blanking gate.  White lands at about 775.
--
-- Stage numbering: a "stage S" register holds pixel n during cycle T + S + n.  Stage 0 = the flute offset
-- accumulator; the output register is stage 43; data_in -> data_out latency 44 in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.flute_pkg.all;

architecture flute of program_top is
    constant C_PROCESSING_DELAY_CLKS : natural := 44;      -- = processing_delay_clks in the TOML

    -- delay-line read offsets: a line written from stage W whose EBR output register is stage Q runs its
    -- read address Q - 1 - W behind the write address
    constant C_XH  : integer := 3;       -- log2 hi + flags: written from stage 10, EBR register = stage 14
    constant C_XS  : integer := 22;      -- flute factor:    written from stage 8,  EBR register = stage 31
    constant C_XY  : integer := 36;      -- avid + syncs:    written from stage -1, EBR register = stage 36

    attribute no_rw_check : boolean;

    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- Exact signed product a * m, radix-4 Booth on m (from Hypnos)
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

    ----------------------------------------------------------------------
    -- timing, zone, raster, generated avid
    ----------------------------------------------------------------------
    signal s_ystp  : unsigned(9 downto 0) := to_unsigned(256, 10);      -- field-line pitch in x pixels, Q8
    signal s_mcls  : std_logic_vector(1 downto 0) := "00";              -- 00 1080 / 01 720p / 10 SD
    signal s_ls14  : unsigned(14 downto 0) := (others => '0');          -- log2(x step in virtual px), Q14
    signal s_ls12  : unsigned(12 downto 0) := (others => '0');
    signal s_ls6   : unsigned(6 downto 0) := (others => '0');
    signal s_gh    : unsigned(5 downto 0) := to_unsigned(32, 6);        -- groove half-width, Q4 px
    signal s_oxs   : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys   : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_idle  : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc    : unsigned(10 downto 0) := (others => '0');
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_wv    : std_logic := '0';
    signal s_xl    : unsigned(11 downto 0) := (others => '0');
    signal s_xd    : signed(12 downto 0) := (others => '0');
    signal s_xs1, s_xs2, s_xok : std_logic := '0';
    signal s_Wl    : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_lacc  : unsigned(19 downto 0) := (others => '0');
    signal s_avid_q, s_avid_r : std_logic := '0';
    signal s_hs_q, s_hs_r     : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_rsr   : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc    : unsigned(3 downto 0) := "1000";
    signal g_cnt   : unsigned(11 downto 0) := (others => '0');
    signal s_gav_p : std_logic := '0';
    signal s_st0   : std_logic := '0';                                  -- line start, high in cycle T

    ----------------------------------------------------------------------
    -- the per-field engine
    ----------------------------------------------------------------------
    constant OP_LD   : std_logic_vector(3 downto 0) := x"0";
    constant OP_ADD  : std_logic_vector(3 downto 0) := x"1";
    constant OP_SUB  : std_logic_vector(3 downto 0) := x"2";
    constant OP_MUL  : std_logic_vector(3 downto 0) := x"3";
    constant OP_LDI  : std_logic_vector(3 downto 0) := x"4";
    constant OP_ADDI : std_logic_vector(3 downto 0) := x"5";
    constant OP_SUBI : std_logic_vector(3 downto 0) := x"6";
    constant OP_MULI : std_logic_vector(3 downto 0) := x"7";
    constant OP_ST   : std_logic_vector(3 downto 0) := x"8";
    constant OP_IN   : std_logic_vector(3 downto 0) := x"9";
    constant OP_OUT  : std_logic_vector(3 downto 0) := x"A";
    constant OP_SHR  : std_logic_vector(3 downto 0) := x"B";
    constant OP_SKIP : std_logic_vector(3 downto 0) := x"C";
    constant OP_JMP  : std_logic_vector(3 downto 0) := x"D";
    constant OP_CALL : std_logic_vector(3 downto 0) := x"E";
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";      -- 0 return, 1 halt, 2 normalise

    signal rf_ram : t_c_fl_rfi := C_FL_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc, e_lr : unsigned(9 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";      -- the microprogram ROM's output register
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";      -- its fabric copy (from phase 1 on)
    signal x_mul, x_shr, x_nrm, x_halt, x_out, x_st : std_logic := '0';   -- decoded at phase 1
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';              -- the adder's controls, as registers
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal w_in  : unsigned(15 downto 0);
    signal ft_adr : unsigned(8 downto 0) := (others => '0');
    signal ft_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal s_glide : std_logic := '1';
    signal c_stripes, s_src : std_logic := '0';

    -- per-field constants (the engine's outputs)
    signal s_wq4   : unsigned(12 downto 0) := to_unsigned(2048, 13);    -- flute width, Q4 px
    signal s_sy    : signed(16 downto 0) := (others => '0');            -- flute-normal step per line, Q14
    signal s_G     : unsigned(10 downto 0) := (others => '0');
    signal c_uinc  : unsigned(14 downto 0) := to_unsigned(16384, 15);   -- Q14
    signal c_uincw : signed(23 downto 0) := (others => '0');            -- uinc - W
    signal c_qincw : signed(27 downto 0) := (others => '0');
    signal c_qinc  : signed(17 downto 0) := (others => '0');
    signal c_qline : signed(19 downto 0) := (others => '0');
    signal c_qjump : signed(27 downto 0) := (others => '0');
    signal c_vinc  : signed(15 downto 0) := (others => '0');
    signal c_vline : signed(16 downto 0) := (others => '0');
    signal c_eoff  : signed(17 downto 0) := (others => '0');
    signal c_kc    : signed(4 downto 0) := (others => '0');
    signal c_lfp   : signed(11 downto 0) := (others => '0');
    signal c_lp    : signed(11 downto 0) := (others => '0');
    signal c_as, c_bs : signed(11 downto 0) := (others => '0');
    signal c_d16   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_shm   : std_logic_vector(1 downto 0) := "00";              -- 00 flat, 01 four steps, 10 smooth
    signal c_t3    : unsigned(11 downto 0) := (others => '0');
    signal c_r7    : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal c_nsh   : unsigned(2 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- line engine and beam accumulators
    ----------------------------------------------------------------------
    signal l_st : unsigned(1 downto 0) := (others => '0');
    signal l_u, l_un, l_ua, l_ub : signed(23 downto 0) := (others => '0');
    signal l_q, l_v : signed(27 downto 0) := (others => '0');
    signal b_u  : signed(23 downto 0) := (others => '0');
    signal b_wf : std_logic := '0';
    signal b_q, b_v : signed(27 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- RAMs
    ----------------------------------------------------------------------
    type t_dl32 is array(0 to 255) of std_logic_vector(31 downto 0);
    type t_dl16 is array(0 to 255) of std_logic_vector(15 downto 0);
    type t_dl4  is array(0 to 255) of std_logic_vector(3 downto 0);
    signal dlh : t_dl32;
    signal dsh : t_dl16;
    signal dls : t_dl4;
    attribute no_rw_check of dlh : signal is true;
    attribute no_rw_check of dsh : signal is true;
    attribute no_rw_check of dls : signal is true;
    signal s_wa  : unsigned(7 downto 0) := (others => '0');
    signal dlh_q : std_logic_vector(31 downto 0) := (others => '0');
    signal dsh_q : std_logic_vector(15 downto 0) := (others => '0');
    signal dls_q : std_logic_vector(3 downto 0) := "0110";

    type t_ct is array(0 to 255) of unsigned(15 downto 0);
    signal ct_ram : t_ct := (others => (others => '0'));
    attribute no_rw_check of ct_ram : signal is true;
    signal ct_q, ct_wd, ct_acc : unsigned(15 downto 0) := (others => '0');
    signal ct_wa, ct_i : unsigned(7 downto 0) := (others => '0');
    signal ct_we, ct_run : std_logic := '0';

    -- source field
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal s_col : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal s_bv, s_ev, s_evv : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb8, s_eq : unsigned(7 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(7 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(7 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal s21_q, s21_qp, s22_l : unsigned(7 downto 0) := (others => '0');
    signal s24_cam : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    -- flute factor (stages 1 .. 8)
    signal s1_na, s1_wa : unsigned(12 downto 0) := (others => '0');
    signal s2_d  : unsigned(12 downto 0) := (others => '0');
    signal s3_e0, s3_e1, s3_e2, s3_e3 : signed(14 downto 0) := (others => '0');
    signal s3_dn : unsigned(11 downto 0) := (others => '0');
    signal s4_c0, s4_c1, s4_c2, s4_c3 : unsigned(4 downto 0) := (others => '0');
    signal s4_p  : unsigned(14 downto 0) := (others => '0');
    signal s5_sum : unsigned(5 downto 0) := (others => '0');
    signal s5_c0, s6_c0, s7_c0 : unsigned(4 downto 0) := (others => '0');
    signal s5_p, s6_p, s7_p : unsigned(6 downto 0) := (others => '0');
    signal s6_a  : unsigned(12 downto 0) := (others => '0');
    signal s6_b  : unsigned(8 downto 0) := (others => '0');
    signal s7_t  : unsigned(12 downto 0) := (others => '0');
    signal s8_sh : unsigned(6 downto 0) := (others => '0');
    signal s32_sh : unsigned(6 downto 0) := (others => '0');

    -- 2 .. 9: |q|, |v| -> hi / lo -> log2 (Q14)
    signal s2_a, s2_b : unsigned(15 downto 0) := (others => '0');
    signal s3_hi, s3_lo : unsigned(15 downto 0) := (others => '0');
    signal s3_sw, s3_lz : std_logic := '0';
    signal s4_hi, s4_lo : unsigned(15 downto 0) := (others => '0');
    signal s4_mh, s4_ml : unsigned(3 downto 0) := (others => '0');
    signal s5_ih, s5_il : unsigned(7 downto 0) := (others => '0');
    signal s5_th, s5_tl, s6_th, s6_tl, s7_th, s7_tl : unsigned(6 downto 0) := (others => '0');
    signal s5_mh, s5_ml, s6_mh, s6_ml, s7_mh, s7_ml, s8_mh, s8_ml : unsigned(3 downto 0) := (others => '0');
    signal lvh_q, lsh_q, lvl_q, lsl_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s7_lvh, s7_lsh, s7_lvl, s7_lsl : std_logic_vector(15 downto 0) := (others => '0');
    signal s8_lph, s8_lpl : unsigned(13 downto 0) := (others => '0');
    signal s8_lvh, s8_lvl : unsigned(13 downto 0) := (others => '0');
    signal s9_lh, s9_ll : unsigned(17 downto 0) := (others => '0');
    type t_fl is array(4 to 9) of std_logic_vector(1 downto 0);          -- sw, lz
    signal fsr : t_fl := (others => (others => '0'));

    -- 10 .. 17: u = log2(hi / lo) -> g(u) -> log2 r -> exponent, gradient terms
    signal s10_ua : unsigned(17 downto 0) := (others => '0');
    signal s10_lh : unsigned(17 downto 0) := (others => '0');
    signal s10_fl : std_logic_vector(1 downto 0) := "00";
    signal s11_ia : unsigned(7 downto 0) := (others => '0');
    signal s11_tu, s12_tu, s13_tu : unsigned(6 downto 0) := (others => '0');
    signal ug_q, ugs_q, s13_g, s13_gs : std_logic_vector(15 downto 0) := (others => '0');
    signal s14_gp : signed(18 downto 0) := (others => '0');
    signal s14_g  : unsigned(15 downto 0) := (others => '0');
    signal s15_g  : unsigned(13 downto 0) := (others => '0');
    signal s15_d  : std_logic_vector(31 downto 0) := (others => '0');
    signal s16_lr : unsigned(17 downto 0) := (others => '0');
    signal s16_base, s17_base, s17_tlo : signed(11 downto 0) := (others => '0');
    signal s16_uq : unsigned(8 downto 0) := (others => '0');
    signal s16_sw, s17_sw : std_logic := '0';
    signal s17_e  : signed(19 downto 0) := (others => '0');

    -- 18 .. 26: exp2 mantissa, shift = phase; gradient combine -> filter width
    signal s18_xa : unsigned(7 downto 0) := (others => '0');
    signal s18_t, s19_t, s20_t : unsigned(5 downto 0) := (others => '0');
    signal s18_sh, s19_sh, s20_sh, s21_sh, s22_sh : unsigned(4 downto 0) := (others => '0');   -- k + 17
    signal s18_av, s18_bq : signed(11 downto 0) := (others => '0');
    signal exm_q, exs_q, s20_m, s20_s : std_logic_vector(15 downto 0) := (others => '0');
    signal s19_dl : signed(12 downto 0) := (others => '0');
    signal s19_mx, s20_mx, s21_mx : signed(11 downto 0) := (others => '0');
    signal s20_ad : unsigned(7 downto 0) := (others => '0');
    signal s21_h  : unsigned(5 downto 0) := (others => '0');
    signal s21_ep : unsigned(13 downto 0) := (others => '0');
    signal s21_m  : unsigned(15 downto 0) := (others => '0');
    signal s22_mt : unsigned(15 downto 0) := (others => '0');
    signal s22_lg, s23_lg, s24_lw : signed(11 downto 0) := (others => '0');
    signal s23_lwp : std_logic := '0';
    signal s23_ps : unsigned(27 downto 0) := (others => '0');
    signal s23_f  : unsigned(2 downto 0) := (others => '0');
    signal s24_phr : unsigned(20 downto 0) := (others => '0');
    signal s25_h, s25_sat : std_logic := '0';
    signal s25_iw : integer range -16 to 0 := -16;
    signal s25_ri, s26_ri, s27_ri, s28_ri, s29_ri : integer range 0 to 15 := 0;
    signal s25_rh, s26_rh, s27_rh, s28_rh, s29_rh, s30_rh : std_logic := '0';
    signal s25_q, s26_phi : unsigned(15 downto 0) := (others => '0');
    signal s26_w  : unsigned(15 downto 0) := (others => '1');

    -- 27 .. 32: one-pixel box filter -> coverage 0 .. 256
    signal s27_lo, s27_hi : unsigned(15 downto 0) := (others => '0');
    signal s28_k : std_logic := '0';
    signal s28_full, s29_full, s30_full, s31_full : std_logic := '0';
    signal s28_mh, s28_ml : unsigned(15 downto 0) := (others => '0');
    signal s29_n  : unsigned(15 downto 0) := (others => '0');
    signal s30_nsh : unsigned(9 downto 0) := (others => '0');
    signal s31_cp : unsigned(10 downto 0) := (others => '0');
    signal s32_cov : unsigned(8 downto 0) := (others => '0');

    -- 33 .. 43: tone, output conditioning
    signal s33_a  : unsigned(15 downto 0) := (others => '0');
    signal s34_a9 : unsigned(8 downto 0) := (others => '0');
    signal s35_p1 : unsigned(18 downto 0) := (others => '0');
    signal s35_p2 : unsigned(13 downto 0) := (others => '0');
    signal s36_y, s37_y, s38_y, s38_f, s39_y, s40_y, s41_y, s42_y, o_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_sy is array(37 to 43) of std_logic_vector(3 downto 0);        -- avid, hsync, vsync, field
    signal ysr : t_sy := (others => "0110");

begin

    ------------------------------------------------------------------------
    -- ROMs (block RAM): every address is a register, every read is registered
    ------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lvh_q <= C_FL_LOGV(to_integer(s5_ih));
            lsh_q <= C_FL_LOGS(to_integer(s5_ih));
            lvl_q <= C_FL_LOGV(to_integer(s5_il));
            lsl_q <= C_FL_LOGS(to_integer(s5_il));
            ug_q  <= C_FL_UG(to_integer(s11_ia));
            ugs_q <= C_FL_UGS(to_integer(s11_ia));
            exm_q <= C_FL_EXM(to_integer(s18_xa));
            exs_q <= C_FL_EXS(to_integer(s18_xa));
        end if;
    end process p_rom;

    ------------------------------------------------------------------------
    -- TIMING: per-mode steps from the timing ID, the vblank zone (no avid for
    -- 1024 clocks), raster measurement, and the GENERATED AVID (parity-locked
    -- start 2 or 3 clocks after the source's avid rise, field-latched width).
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;
            s_xs1    <= '0';

            -- s_ystp = field-line pitch in x-pixel units (Q8); s_oys = the Osc lattice's line step
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_ystp <= to_unsigned(569, 10); s_mcls <= "10"; s_oys <= to_unsigned(537, 10);  -- 480i
                when 4          => s_ystp <= to_unsigned(288, 10); s_mcls <= "10"; s_oys <= to_unsigned(272, 10);  -- 480p
                when 8          => s_ystp <= to_unsigned(480, 10); s_mcls <= "10"; s_oys <= to_unsigned(453, 10);  -- 576i
                when 12         => s_ystp <= to_unsigned(240, 10); s_mcls <= "10"; s_oys <= to_unsigned(226, 10);  -- 576p
                when 5 | 6 | 14 => s_ystp <= to_unsigned(256, 10); s_mcls <= "01"; s_oys <= to_unsigned(181, 10);  -- 720p
                when 1 | 2 | 10 => s_ystp <= to_unsigned(512, 10); s_mcls <= "00"; s_oys <= to_unsigned(242, 10);  -- 1080i
                when others     => s_ystp <= to_unsigned(256, 10); s_mcls <= "00"; s_oys <= to_unsigned(121, 10);  -- 1080p
            end case;
            -- log2(x step): SD 1 octave, 720p log2(1.5); the groove is 4 virtual px wide in every mode
            case s_mcls is
                when "10"   => s_ls14 <= to_unsigned(16384, 15); s_ls12 <= to_unsigned(4096, 13); s_ls6 <= to_unsigned(64, 7);
                               s_gh <= to_unsigned(16, 6); s_oxs <= to_unsigned(204, 9);
                when "01"   => s_ls14 <= to_unsigned(9585, 15);  s_ls12 <= to_unsigned(2396, 13); s_ls6 <= to_unsigned(37, 7);
                               s_gh <= to_unsigned(21, 6); s_oxs <= to_unsigned(153, 9);
                when others => s_ls14 <= to_unsigned(0, 15);     s_ls12 <= to_unsigned(0, 13);    s_ls6 <= to_unsigned(0, 7);
                               s_gh <= to_unsigned(32, 6); s_oxs <= to_unsigned(102, 9);
            end case;

            -- the vblank zone
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

            -- width (hysteresis: analog line lengths wander +-1 px; the first one latches) and height
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                s_xl <= s_xcnt; s_xs1 <= '1';
                s_xcnt <= (others => '0');
                s_lacc <= s_lacc + s_ystp;
            end if;
            -- (the comparison takes two clocks of the blanking: a line length is only ever tested here)
            s_xs2 <= s_xs1;
            s_xd  <= signed('0' & s_xl) - signed('0' & s_W);
            if s_xl > 16 then s_xok <= '1'; else s_xok <= '0'; end if;
            if s_xs2 = '1' and s_xok = '1' and (s_wv = '0' or s_xd > 2 or s_xd < -2) then
                s_W <= s_xl;
                s_wv <= '1';
            end if;
            if s_zgo = '1' then
                if s_lacc(19 downto 8) > 8 then
                    s_H <= s_lacc(19 downto 8);
                end if;
                s_lacc <= (others => '0');
                s_Wl <= s_W;
            end if;

            -- generated avid
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
    -- THE PER-FIELD ENGINE.  Started at every zone start from address 0 (so a
    -- stalled or garbled run cannot outlive a field); three clocks per micro-op
    -- (register-file read, the ROM word copied to fabric / operand, decode and
    -- next address / execute), serial multiply
    -- (18 steps) and shift.  ACC = 37 bits; MUL = ACC(17:0) x operand(17:0),
    -- both signed, by add-and-shift-right with the last step subtracting.
    ------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(s_Wl, 16)            when x"8",
        resize(s_H, 16)             when x"9",
        resize(s_ystp, 16)          when x"A",
        unsigned(ft_q)              when x"B",
        resize(e_cnt, 16)           when x"C",
        resize(s_ls14, 16)          when x"D",
        resize(s_ls12, 16)          when x"E",
        resize(s_ls6, 16)           when x"F",
        resize(unsigned(registers_in(to_integer(unsigned(e_irf(2 downto 0))))), 16) when others;

    e_out <= '1' when e_run = '1' and e_ph = "10" and x_out = '1' else '0';

    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_gate, v_sub, v_shift, v_last, v_cond, v_fetch, v_nz : std_logic;
        variable v_bx  : signed(36 downto 0);
        variable v_t   : signed(37 downto 0);
        variable v_sum : signed(36 downto 0);
    begin
        if rising_edge(clk) then
            v_op := e_irf(15 downto 12);
            v_arg := e_irf(11 downto 0);
            v_fetch := '0';
            ft_q <= C_FL_FT(to_integer(ft_adr));
            if e_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(e_ir(5 downto 0))));
            end if;
            if e_run = '1' and e_ph = "10" and x_st = '1' then
                rf_ram(to_integer(unsigned(e_irf(5 downto 0)))) <= std_logic_vector(e_acc(31 downto 0));
            end if;

            if s_zgo = '1' then
                e_run <= '1'; e_pc <= (others => '0'); e_ph <= "11";
                s_glide   <= registers_in(6)(0);
                c_stripes <= registers_in(6)(1);
                s_src     <= registers_in(6)(4);
                -- S9 Shade / S10 Weight: flat bold, flat fine (1/6 duty), 4 steps, smooth
                if registers_in(6)(2) = '0' then c_shm <= "00";
                elsif registers_in(6)(3) = '0' then c_shm <= "01";
                else c_shm <= "10"; end if;
                if registers_in(6)(3) = '1' and registers_in(6)(2) = '0' then c_d16 <= to_unsigned(10923, 16);
                else c_d16 <= to_unsigned(32768, 16); end if;
            elsif e_run = '1' then
                case e_ph is
                    when "11" =>                          -- fetch (first micro-op)
                        v_fetch := '1';
                        e_ph <= "00";
                    when "00" =>                          -- register-file read
                        e_irf <= e_ir;
                        e_ph <= "01";
                    when "01" =>                          -- operand, accumulator clear, next address
                        case v_op is
                            when OP_LD | OP_ADD | OP_SUB => e_b <= resize(signed(rf_q), 37);
                            when OP_LDI | OP_ADDI | OP_SUBI => e_b <= resize(signed(v_arg), 37);
                            when OP_IN   => e_b <= signed(resize(w_in, 37));
                            when OP_MUL  => e_b <= shift_left(resize(signed(rf_q(17 downto 0)), 37), 18);
                            when OP_MULI => e_b <= shift_left(resize(signed(v_arg), 37), 18);
                            when others  => null;
                        end case;
                        if v_op = OP_LD or v_op = OP_LDI or v_op = OP_IN then
                            e_acc <= (others => '0');
                        elsif v_op = OP_MUL or v_op = OP_MULI then
                            e_acc(36 downto 18) <= (others => '0');
                        end if;
                        if v_op = OP_MUL or v_op = OP_MULI then e_cnt <= to_unsigned(17, 5);
                        elsif v_op = OP_SHR then e_cnt <= unsigned(v_arg(4 downto 0)) - 1;
                        elsif v_op = OP_SPC then e_cnt <= (others => '0'); end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := e_acc(36);
                            when "001"  => v_cond := s_glide;
                            when "010"  => v_cond := c_stripes;
                            when others => v_cond := s_src;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then e_pc <= e_pc + 2;
                        elsif v_op = OP_JMP then e_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then e_pc <= unsigned(v_arg(9 downto 0)); e_lr <= e_pc + 1;
                        elsif v_op = OP_SPC and v_arg(1 downto 0) = "00" then e_pc <= e_lr;
                        else e_pc <= e_pc + 1; end if;
                        x_mul <= '0'; x_shr <= '0'; x_nrm <= '0'; x_halt <= '0'; x_out <= '0'; x_st <= '0';
                        x_gate <= '0'; x_subr <= '0'; x_shift <= '0'; x_cz <= '0';
                        case v_op is
                            when OP_LD | OP_ADD | OP_LDI | OP_ADDI | OP_IN => x_gate <= '1';
                            when OP_SUB | OP_SUBI => x_gate <= '1'; x_subr <= '1';
                            when OP_MUL | OP_MULI => x_mul <= '1'; x_shift <= '1'; x_gate <= e_acc(0);
                            when OP_SHR  =>
                                x_shr <= '1'; x_shift <= '1';
                                if v_arg(4 downto 0) = "00001" then x_cz <= '1'; end if;
                            when OP_ST   => x_st <= '1';
                            when OP_OUT  => x_out <= '1';
                            when OP_SPC  => x_nrm <= v_arg(1); x_halt <= v_arg(0) and not v_arg(1);
                            when others  => null;
                        end case;
                        e_ph <= "10";
                    when others =>                        -- execute (the adder sees registers only)
                        v_gate := x_gate; v_sub := x_subr; v_shift := x_shift; v_last := '1';
                        if e_acc(36 downto 5) = 0 then v_nz := '0'; else v_nz := '1'; end if;
                        if v_gate = '0' then v_bx := (others => '0');
                        elsif v_sub = '1' then v_bx := not e_b;
                        else v_bx := e_b; end if;
                        v_t := (e_acc & '1') + (v_bx & (v_sub and v_gate));
                        v_sum := v_t(37 downto 1);
                        if x_mul = '1' or x_shr = '1' then
                            if x_cz = '0' then v_last := '0'; end if;
                            e_cnt <= e_cnt - 1;
                            if e_cnt = 1 then x_cz <= '1'; x_subr <= x_mul; end if;   -- the next step is the last
                            if x_mul = '1' then x_gate <= v_sum(1); end if;           -- the next multiplier bit
                        elsif x_nrm = '1' and v_nz = '1' then          -- normalise: shift right until < 32
                            v_shift := '1'; v_last := '0';
                            e_cnt <= e_cnt + 1;
                        end if;
                        if x_halt = '1' then e_run <= '0'; end if;
                        if v_shift = '1' then e_acc <= v_sum(36) & v_sum(36 downto 1);
                        else e_acc <= v_sum; end if;
                        if v_last = '1' then
                            v_fetch := '1';
                            e_ph <= "00";
                        end if;
                end case;
            end if;
            if v_fetch = '1' then
                e_ir <= C_FL_UC(to_integer(e_pc));
            end if;

            -- the engine's output registers
            if e_out = '1' then
                case to_integer(unsigned(e_irf(4 downto 0))) is
                    when 0  => ft_adr  <= unsigned(e_acc(8 downto 0));
                    when 1  => c_eoff  <= e_acc(17 downto 0);
                    when 2  => c_kc    <= e_acc(4 downto 0);
                    when 3  => c_lfp   <= e_acc(11 downto 0);
                    when 4  => c_lp    <= e_acc(11 downto 0);
                    when 5  => c_as    <= e_acc(11 downto 0);
                    when 6  => c_bs    <= e_acc(11 downto 0);
                    when 7  => s_wq4   <= unsigned(e_acc(12 downto 0));
                    when 8  => c_t3    <= unsigned(e_acc(11 downto 0));
                    when 9  => c_r7    <= unsigned(e_acc(6 downto 0));
                    when 10 => c_nsh   <= unsigned(e_acc(2 downto 0));
                    when 11 => c_uinc  <= unsigned(e_acc(14 downto 0));
                    when 12 => c_uincw <= e_acc(23 downto 0);
                    when 13 => c_qinc  <= e_acc(17 downto 0);
                    when 14 => c_qincw <= e_acc(27 downto 0);
                    when 15 => c_vinc  <= e_acc(15 downto 0);
                    when 16 => s_sy    <= e_acc(16 downto 0);
                    when 17 => c_qline <= e_acc(19 downto 0);
                    when 18 => c_vline <= e_acc(16 downto 0);
                    when 19 => c_qjump <= e_acc(27 downto 0);
                    when 23 => s_G     <= unsigned(e_acc(10 downto 0));
                    when others => null;             -- 20 .. 22: the line engine's start values
                end case;
            end if;
        end if;
    end process p_eng;

    ------------------------------------------------------------------------
    -- LINE ENGINE: at the end of each line, the next line's start values.
    --   u (flute offset - W / 2, in [-W, 0)) steps by the normal step and wraps
    --   q steps by its line step, and by -+ the flute jump when u wraps
    --   v steps by its line step
    -- The per-field engine writes the first line's values (outputs 20 .. 22).
    ------------------------------------------------------------------------
    p_line : process(clk)
        variable v_w14 : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            v_w14 := shift_left(resize(signed('0' & s_wq4), 24), 10);
            if e_out = '1' and e_irf(4 downto 2) = "101" then
                case e_irf(1 downto 0) is
                    when "00"   => l_u <= e_acc(23 downto 0);
                    when "01"   => l_q <= e_acc(27 downto 0);
                    when "10"   => l_v <= e_acc(27 downto 0);
                    when others => null;
                end case;
                l_st <= "00";
            elsif s_gav_p = '1' and g_avid = '0' then
                l_st <= "01";
            else
                case l_st is
                    when "01" =>
                        l_un <= l_u + resize(s_sy, 24);
                        l_st <= "10";
                    when "10" =>
                        l_ua <= l_un - v_w14;
                        l_ub <= l_un + v_w14;
                        l_q  <= l_q + resize(c_qline, 28);
                        l_v  <= l_v + resize(c_vline, 28);
                        l_st <= "11";
                    when "11" =>
                        if l_un(23) = '0' then
                            l_u <= l_ua; l_q <= l_q - c_qjump;
                        elsif l_ub(23) = '1' then
                            l_u <= l_ub; l_q <= l_q + c_qjump;
                        else
                            l_u <= l_un;
                        end if;
                        l_st <= "00";
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ------------------------------------------------------------------------
    -- BEAM: stage 0 the flute offset (its wrap flag tells stage 1 that this
    -- pixel began a new flute), stage 1 the scene coordinates.
    ------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_start : std_logic;
        variable v_u1 : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            s_gav_p <= g_avid;
            v_start := g_avid and not s_gav_p;
            s_st0 <= v_start;
            -- stage 0
            v_u1 := b_u + signed(resize(c_uinc, 24));
            if v_start = '1' then
                b_u <= l_u; b_wf <= '0';
            elsif v_u1(23) = '0' then
                b_u <= b_u + c_uincw; b_wf <= '1';
            else
                b_u <= v_u1; b_wf <= '0';
            end if;
            -- stage 1
            if s_st0 = '1' then
                b_q <= l_q; b_v <= l_v;
            else
                if b_wf = '1' then b_q <= b_q + c_qincw; else b_q <= b_q + resize(c_qinc, 28); end if;
                b_v <= b_v + resize(c_vinc, 28);
            end if;
        end if;
    end process p_beam;

    ------------------------------------------------------------------------
    -- delay lines (block RAM): write address = a free-running counter, each
    -- read runs a fixed distance behind it
    ------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_wa <= s_wa + 1;
            dlh(to_integer(s_wa)) <= "00" & s10_fl & std_logic_vector(s10_ua(17 downto 8)) & std_logic_vector(s10_lh);
            dlh_q <= dlh(to_integer(s_wa - C_XH));
            dsh(to_integer(s_wa)) <= "000000000" & std_logic_vector(s8_sh);
            dsh_q <= dsh(to_integer(s_wa - C_XS));
            dls(to_integer(s_wa)) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            dls_q <= dls(to_integer(s_wa - C_XY));
        end if;
    end process p_dl;

    ------------------------------------------------------------------------
    -- FLUTE FACTOR (stages 1 .. 8): d = distance from the nearest groove (Q4 px)
    --   Flat      the groove: clamp(d - half-width + 1/2), one pixel of ramp
    --   4 steps   three such ramps at d = W/8, W/4, 3W/8 (black, dark, light, white)
    --   smooth    d / (W / 2): a triangle across the flute (reciprocal mantissa from the engine)
    -- The factor is 0 .. 64.
    ------------------------------------------------------------------------
    p_shade : process(clk)
        function f_e(d : unsigned(12 downto 0); t : unsigned) return signed is
        begin
            return signed(resize(d, 15)) - signed(resize(t, 15)) + 8;
        end function;
        function f_cl(e : signed(14 downto 0)) return unsigned is
        begin
            if e(14) = '1' then return to_unsigned(0, 5);
            elsif e > 16 then return to_unsigned(16, 5);
            else return unsigned(e(4 downto 0)); end if;
        end function;
        variable v_p : unsigned(7 downto 0);
        variable v_wa : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 1
            s1_na <= unsigned(not std_logic_vector(b_u(22 downto 10)));
            v_wa := signed('0' & s_wq4) + resize(b_u(23 downto 10), 14);        -- 0 .. W - 1: slice, not resize
            s1_wa <= unsigned(v_wa(12 downto 0));
            -- 2
            if s1_na < s1_wa then s2_d <= s1_na; else s2_d <= s1_wa; end if;
            -- 3: d - threshold + 1/2 px
            s3_e0 <= f_e(s2_d, s_gh);
            s3_e1 <= f_e(s2_d, s_wq4(12 downto 3));
            s3_e2 <= f_e(s2_d, s_wq4(12 downto 2));
            s3_e3 <= f_e(s2_d, c_t3);
            s3_dn <= resize(shift_left(s2_d, to_integer(c_nsh)), 12);
            -- 4: the ramps, 0 .. 16
            s4_c0 <= f_cl(s3_e0); s4_c1 <= f_cl(s3_e1); s4_c2 <= f_cl(s3_e2); s4_c3 <= f_cl(s3_e3);
            s4_p  <= s3_dn(11 downto 4) * c_r7;
            -- 5
            s5_sum <= resize(s4_c1, 6) + resize(s4_c2, 6) + resize(s4_c3, 6);
            s5_c0  <= s4_c0;
            v_p := s4_p(14 downto 7);
            if v_p > 64 then s5_p <= to_unsigned(64, 7); else s5_p <= v_p(6 downto 0); end if;
            -- 6: sum * 85 + 32 in two halves
            s6_a <= shift_left(resize(s5_sum, 13), 6) + shift_left(resize(s5_sum, 13), 4);
            s6_b <= shift_left(resize(s5_sum, 9), 2) + resize(s5_sum, 9) + 32;
            s6_c0 <= s5_c0; s6_p <= s5_p;
            -- 7
            s7_t  <= s6_a + resize(s6_b, 13);
            s7_c0 <= s6_c0; s7_p <= s6_p;
            -- 8
            case c_shm is
                when "00"   => s8_sh <= s7_c0 & "00";
                when "01"   => s8_sh <= s7_t(12 downto 6);
                when others => s8_sh <= s7_p;
            end case;
        end if;
    end process p_shade;

    ------------------------------------------------------------------------
    -- 2 .. 9: |q|, |v| (13.3 bits), hi = max, lo = min, and their log2 (Q14):
    -- priority-encode, normalise, 256-segment PWL from block RAM.
    ------------------------------------------------------------------------
    p_logs : process(clk)
        variable va : unsigned(27 downto 0);
        variable vh, vl : unsigned(15 downto 0);
        variable mh, ml : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            -- 2
            va := unsigned(abs(b_q));
            if va(27) = '1' then s2_a <= (others => '1'); else s2_a <= va(26 downto 11); end if;
            va := unsigned(abs(b_v));
            if c_stripes = '1' then s2_b <= (others => '0');
            elsif va(27) = '1' then s2_b <= (others => '1');
            else s2_b <= va(26 downto 11); end if;
            -- 3
            if s2_a >= s2_b then s3_hi <= s2_a; s3_lo <= s2_b; s3_sw <= '0';
            else                 s3_hi <= s2_b; s3_lo <= s2_a; s3_sw <= '1'; end if;
            if s2_a = 0 or s2_b = 0 then s3_lz <= '1'; else s3_lz <= '0'; end if;
            -- 4: msb (0 reads as 1)
            vh := s3_hi; if vh = 0 then vh := to_unsigned(1, 16); end if;
            vl := s3_lo; if vl = 0 then vl := to_unsigned(1, 16); end if;
            mh := 0; ml := 0;
            for i in 0 to 15 loop
                if vh(i) = '1' then mh := i; end if;
                if vl(i) = '1' then ml := i; end if;
            end loop;
            s4_hi <= vh; s4_lo <= vl;
            s4_mh <= to_unsigned(mh, 4); s4_ml <= to_unsigned(ml, 4);
            fsr(4) <= s3_sw & s3_lz;
            for i in 5 to 9 loop fsr(i) <= fsr(i - 1); end loop;
            -- 5: normalise -> 8-bit segment index + 7-bit fraction
            vh := shift_left(s4_hi, 15 - to_integer(s4_mh));
            vl := shift_left(s4_lo, 15 - to_integer(s4_ml));
            s5_ih <= vh(14 downto 7); s5_th <= vh(6 downto 0);
            s5_il <= vl(14 downto 7); s5_tl <= vl(6 downto 0);
            s5_mh <= s4_mh; s5_ml <= s4_ml;
            -- 6, 7 (block-RAM read, fabric register)
            s6_th <= s5_th; s6_tl <= s5_tl; s6_mh <= s5_mh; s6_ml <= s5_ml;
            s7_th <= s6_th; s7_tl <= s6_tl; s7_mh <= s6_mh; s7_ml <= s6_ml;
            s7_lvh <= lvh_q; s7_lsh <= lsh_q; s7_lvl <= lvl_q; s7_lsl <= lsl_q;
            -- 8
            s8_lph <= unsigned(s7_lsh(6 downto 0)) * s7_th;
            s8_lpl <= unsigned(s7_lsl(6 downto 0)) * s7_tl;
            s8_lvh <= unsigned(s7_lvh(13 downto 0));
            s8_lvl <= unsigned(s7_lvl(13 downto 0));
            s8_mh <= s7_mh; s8_ml <= s7_ml;
            -- 9: log2 = msb + PWL(mantissa), Q14
            s9_lh <= shift_left(resize(s8_mh, 18), 14) + resize(s8_lvh, 18) + resize(s8_lph(13 downto 7), 18);
            s9_ll <= shift_left(resize(s8_ml, 18), 14) + resize(s8_lvl, 18) + resize(s8_lpl(13 downto 7), 18);
        end if;
    end process p_logs;

    ------------------------------------------------------------------------
    -- 10 .. 17: u = log2(hi / lo) >= 0;  log2 r = log2 hi + 1/2 log2(1 + 4^-u)
    -- (block-RAM table, 1/16-octave steps, interpolated);  exponent e = log2 r
    -- + the per-field offset;  the two gradient terms.
    ------------------------------------------------------------------------
    p_polar : process(clk)
        variable vu : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- 10
            vu := s9_lh - s9_ll;
            if fsr(9)(0) = '1' then vu := to_unsigned(262143, 18); end if;      -- lo = 0: the table end (g = 0)
            s10_ua <= vu;
            s10_lh <= s9_lh;
            s10_fl <= fsr(9);
            -- 11: 1/16-octave segment + 7-bit fraction
            s11_ia <= s10_ua(17 downto 10);
            s11_tu <= s10_ua(9 downto 3);
            -- 12, 13 (block-RAM read, fabric register)
            s12_tu <= s11_tu;
            s13_tu <= s12_tu;
            s13_g <= ug_q; s13_gs <= ugs_q;
            -- 14: interpolation product (the slope is negative)
            s14_gp <= bmul(signed(s13_gs(10 downto 0)), signed(resize(s13_tu, 8)));
            s14_g  <= unsigned(s13_g);
            -- 15
            s15_g <= resize(unsigned(signed(resize(s14_g, 17)) + resize(shift_right(s14_gp, 7), 17)), 14);
            s15_d <= dlh_q;
            -- 16: log2 r; gradient base Lf - g; u in Q6 (held to 8 octaves)
            s16_lr <= unsigned(s15_d(17 downto 0)) + resize(s15_g, 18);
            s16_base <= c_lfp - signed(resize(s15_g(13 downto 8), 12));
            if s15_d(27) = '1' then s16_uq <= (others => '1'); else s16_uq <= unsigned(s15_d(26 downto 18)); end if;
            s16_sw <= s15_d(29);
            -- 17
            s17_e <= signed(resize(s16_lr, 20)) + resize(c_eoff, 20);
            s17_tlo <= s16_base - signed(resize(s16_uq, 12));
            s17_base <= s16_base;
            s17_sw <= s16_sw;
        end if;
    end process p_polar;

    ------------------------------------------------------------------------
    -- 18 .. 26: r * f = 2^e: the mantissa from a block-RAM PWL, then one shift
    -- (in two stages) gives the phase.  Gradient: a = tangent term, b = normal
    -- term (x |Power|), Lg = max + 1/2 log2(1 + 2^-2|a-b|); the filter width
    -- w = 2^Lg cycles per pixel, widened above 1/4 cycle per pixel so it
    -- reaches a full cycle (the duty grey) at the Nyquist limit.
    ------------------------------------------------------------------------
    p_exp : process(clk)
        variable vk  : signed(6 downto 0);
        variable vb  : signed(11 downto 0);
        variable vlg : signed(11 downto 0);
        variable vlw : signed(11 downto 0);
        variable viw : integer;
        variable vsh : integer;
        variable vw  : unsigned(15 downto 0);
        variable vm  : unsigned(6 downto 0);
        variable vrr : signed(12 downto 0);
        variable vc  : unsigned(39 downto 0);
        variable vf  : unsigned(34 downto 0);
        variable vad : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 18
            s18_xa <= unsigned(s17_e(13 downto 6));
            s18_t  <= unsigned(s17_e(5 downto 0));
            vk := resize(s17_e(19 downto 14), 7) + resize(c_kc, 7);
            if vk < -17 then vk := to_signed(-17, 7); elsif vk > 12 then vk := to_signed(12, 7); end if;
            vk := vk + 17;
            s18_sh <= unsigned(vk(4 downto 0));
            if c_stripes = '1' then
                s18_av <= c_as; s18_bq <= c_bs;
            elsif s17_sw = '1' then
                s18_av <= s17_base; s18_bq <= s17_tlo + c_lp;
            else
                s18_av <= s17_tlo;  s18_bq <= s17_base + c_lp;
            end if;
            -- 19, 20 (block-RAM read, fabric register)
            s19_t <= s18_t; s19_sh <= s18_sh;
            s20_t <= s19_t; s20_sh <= s19_sh;
            s20_m <= exm_q; s20_s <= exs_q;
            s19_dl <= resize(s18_av, 13) - resize(s18_bq, 13);
            if s18_bq > s18_av then s19_mx <= s18_bq; else s19_mx <= s18_av; end if;
            -- (a slice, not resize: a signed narrowing resize keeps the sign bit and folds 128 .. 255)
            vad := abs(s19_dl);
            if vad > 255 then s20_ad <= (others => '1');
            else s20_ad <= unsigned(vad(7 downto 0)); end if;
            s20_mx <= s19_mx;
            -- 21
            s21_ep <= unsigned(s20_s(7 downto 0)) * s20_t;
            s21_m  <= unsigned(s20_m);
            s21_sh <= s20_sh;
            s21_h  <= C_FL_HT(to_integer(s20_ad(7 downto 2)))(5 downto 0);
            s21_mx <= s20_mx;
            -- 22
            s22_mt <= s21_m + resize(s21_ep(13 downto 6), 16);
            s22_sh <= s21_sh;
            vlg := s21_mx + signed(resize(s21_h, 12));
            if vlg > 255 then vlg := to_signed(255, 12); end if;
            s22_lg <= vlg;
            -- 23: coarse shift (0 / 8 / 16 / 24);  lg + 128 > 0 ?
            vc := shift_left(resize(s22_mt, 40), 8 * to_integer(s22_sh(4 downto 3)));
            s23_ps <= vc(37 downto 10);
            s23_f  <= s22_sh(2 downto 0);
            s23_lg <= s22_lg;
            vlw := s22_lg + 128;
            if vlw > 0 then s23_lwp <= '1'; else s23_lwp <= '0'; end if;
            -- 24: fine shift -> phase (Q18, 3 integer bits) = mantissa * 2^k
            vf := shift_left(resize(s23_ps, 35), to_integer(s23_f));
            s24_phr <= vf(27 downto 7);
            -- (2 lg + 128 as a shift: lg + (lg + 128) would put one net on both carry inputs)
            if s23_lwp = '1' then s24_lw <= shift_left(s23_lg, 1) + 128; else s24_lw <= s23_lg; end if;
            -- 25: half-octave quantise (UP) -> w = {1, sqrt 2} * 2^Iw and its reciprocal;
            --     phase + camera + half the duty (the band is centred on whole cycles), Q16
            vlw := s24_lw;
            if vlw < -1023 then vlw := to_signed(-1023, 12); end if;
            vlw := vlw + 31;
            vlw(4 downto 0) := (others => '0');
            if vlw >= 0 then s25_sat <= '1'; else s25_sat <= '0'; end if;
            s25_h <= vlw(5);
            viw := to_integer(shift_right(vlw, 6));
            if viw > 0 then viw := 0; end if;
            s25_iw <= viw;
            if vlw >= 0 then vrr := (others => '0'); else vrr := -resize(vlw, 13); end if;
            s25_ri <= to_integer(unsigned(vrr(9 downto 6)));
            s25_rh <= vrr(5);
            s25_q  <= s24_phr(17 downto 2) + s24_cam + ('0' & c_d16(15 downto 1));
            -- 26: filter width w (Q16)
            if s25_sat = '1' then
                vw := (others => '1');
            else
                vsh := s25_iw + 10;
                if s25_h = '1' then vm := to_unsigned(91, 7); else vm := to_unsigned(64, 7); end if;
                if vsh >= 0 then vw := resize(shift_left(resize(vm, 16), vsh), 16);
                else             vw := resize(shift_right(resize(vm, 16), -vsh), 16); end if;
                if vw = 0 then vw := to_unsigned(1, 16); end if;
            end if;
            s26_w   <= vw;
            s26_ri  <= s25_ri; s26_rh <= s25_rh;
            s26_phi <= s25_q;
        end if;
    end process p_exp;

    ------------------------------------------------------------------------
    -- 27 .. 32: one-pixel box filter of the band square wave.
    --   band = frac in [0, D);  P(x) = floor(x) D + min(frac x, D)
    --   coverage = (P(phi + w/2) - P(phi - w/2)) / w   (divide = * 2^-log2 w)
    ------------------------------------------------------------------------
    p_cov : process(clk)
        variable vn  : signed(17 downto 0);
        variable vs  : unsigned(24 downto 0);
        variable vc  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 27
            s27_lo <= s26_phi - resize(s26_w(15 downto 1), 16);
            s27_hi <= s26_phi + resize(s26_w(15 downto 1), 16);
            s27_ri <= s26_ri; s27_rh <= s26_rh;
            -- 28
            if s27_hi < s27_lo then s28_k <= '1'; else s28_k <= '0'; end if;
            if s27_hi >= s27_lo and s27_hi < c_d16 then s28_full <= '1'; else s28_full <= '0'; end if;
            if s27_hi < c_d16 then s28_mh <= s27_hi; else s28_mh <= c_d16; end if;
            if s27_lo < c_d16 then s28_ml <= s27_lo; else s28_ml <= c_d16; end if;
            s28_ri <= s27_ri; s28_rh <= s27_rh;
            -- 29
            vn := signed(resize(s28_mh, 18)) - signed(resize(s28_ml, 18));
            if s28_k = '1' then vn := vn + signed(resize(c_d16, 18)); end if;
            if vn < 0 then vn := (others => '0'); end if;
            s29_n <= unsigned(vn(15 downto 0));
            s29_ri <= s28_ri; s29_rh <= s28_rh; s29_full <= s28_full;
            -- 30: (N << Ri) >> 7
            vs := shift_left(resize(s29_n, 25), s29_ri);
            s30_nsh <= vs(16 downto 7);
            s30_rh <= s29_rh; s30_full <= s29_full;
            -- 31: / 2, or * sqrt 2 / 2 on a half-octave step
            if s30_rh = '1' then
                s31_cp <= resize(shift_right(s30_nsh, 1), 11) + resize(shift_right(s30_nsh, 3), 11)
                          + resize(shift_right(s30_nsh, 4), 11) + resize(shift_right(s30_nsh, 6), 11);
            else
                s31_cp <= resize(shift_right(s30_nsh, 1), 11);
            end if;
            s31_full <= s30_full;
            -- 32: coverage 0 .. 256
            if s31_cp > 256 or s31_full = '1' then s32_cov <= to_unsigned(256, 9);
            else s32_cov <= s31_cp(8 downto 0); end if;
            s32_sh <= unsigned(dsh_q(6 downto 0));
        end if;
    end process p_cov;

    ------------------------------------------------------------------------
    -- SOURCE FIELD (the Kinetic shell's): luma (or the Osc lattice) -> 32-px
    -- box, edge-replicated and reloaded at every line start -> vertical 1/8 IIR
    -- in the line store (half horizontal resolution).  The pixel path taps the
    -- IIR's read (stage 21), interpolates (22) and looks the phase offset up in
    -- a table refilled every field: offset = level * gain (stage 24).
    ------------------------------------------------------------------------
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    p_ctram : process(clk)
    begin
        if rising_edge(clk) then
            ct_q <= ct_ram(to_integer(s22_l));
            if ct_we = '1' then ct_ram(to_integer(ct_wa)) <= ct_wd; end if;
        end if;
    end process p_ctram;

    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d : signed(9 downto 0);
        variable v_a : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting (the one thing that moves by itself)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avid_q = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 samples; the samples before the line start count as the first one
            -- (reloaded on every line, so a glitched sample cannot leave an offset)
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(225, 8); end if;
            if s_bcnt < 32 then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                s_bsum <= shift_left(resize(s_sl, 13), 5);
                s_bcnt <= to_unsigned(1, 6);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 32 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on the odd columns: q += (box - q + 4) >> 3
            s_eb8 <= s_bsum(12 downto 5); s_eq <= lb_q1;
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb8, 10)) - signed(resize(s_eq, 10)) + 4;
            v_a := unsigned(signed(resize(s_eq, 9)) + resize(shift_right(v_d, 3), 9));
            lb_wd <= v_a(7 downto 0);
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- pixel path: the camera is never shown, so the field need not be aligned to the picture: the
            -- pixel path taps the IIR's own read of the store (the line above, 22 columns ahead of the
            -- pixel at stage 21) and holds the last column to the end of the line; two-tap mean (the store
            -- is half resolution)
            s_evv <= s_bv;
            if s_evv = '1' then s21_q <= s_eq; end if;
            s21_qp <= s21_q;
            v_a := resize(s21_q, 9) + resize(s21_qp, 9) + 1;
            s22_l <= v_a(8 downto 1);
            s24_cam <= ct_q;

            -- phase-offset table fill (256 clocks from the engine's gain output): T(l) = l * gain (mod one cycle)
            ct_we <= '0';
            if e_out = '1' and e_irf(4 downto 0) = "10111" then
                ct_run <= '1'; ct_i <= (others => '0'); ct_acc <= (others => '0');
            elsif ct_run = '1' then
                ct_we <= '1'; ct_wa <= ct_i; ct_wd <= ct_acc;
                ct_acc <= ct_acc + resize(s_G, 16); ct_i <= ct_i + 1;
                if ct_i = 255 then ct_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ------------------------------------------------------------------------
    -- 33 .. 43: tone = coverage x flute factor -> luma 64 .. 940 -> [1 2 1] / 4
    -- (picture edges replicated) -> 13/16 toward black (white 775) -> blanking
    -- gate -> two pad stages (latency 44).  U = V = 512 always.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_a : unsigned(15 downto 0);
        variable v_l, v_r : unsigned(9 downto 0);
        variable v_f : unsigned(11 downto 0);
        variable v_d : unsigned(9 downto 0);
        variable v_y : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            -- 33
            s33_a <= s32_cov * s32_sh;
            -- 34
            v_a := s33_a + 32;
            s34_a9 <= v_a(14 downto 6);
            -- 35, 36: * 876 / 256 = (1024 - 128 - 16 - 4) / 256
            s35_p1 <= shift_left(resize(s34_a9, 19), 10) - shift_left(resize(s34_a9, 19), 7);
            s35_p2 <= shift_left(resize(s34_a9, 14), 4) + shift_left(resize(s34_a9, 14), 2);
            v_y := s35_p1 - resize(s35_p2, 19);
            s36_y <= to_unsigned(64, 10) + v_y(17 downto 8);
            -- 37, 38: [1 2 1] / 4 around the stage-37 pixel
            s37_y <= s36_y; s38_y <= s37_y;
            ysr(37) <= dls_q;
            for i in 38 to 43 loop ysr(i) <= ysr(i - 1); end loop;
            if dls_q(3) = '1' then v_r := s36_y; else v_r := s37_y; end if;
            if ysr(38)(3) = '1' then v_l := s38_y; else v_l := s37_y; end if;
            v_f := resize(v_l, 12) + shift_left(resize(s37_y, 12), 1) + resize(v_r, 12) + 2;
            s38_f <= v_f(11 downto 2);
            -- 39: 13/16 toward black
            if s38_f > 64 then v_d := s38_f - 64; else v_d := (others => '0'); end if;
            s39_y <= s38_f - shift_right(v_d, 3) - shift_right(v_d, 4);
            -- 40: blanking gate
            if ysr(39)(3) = '1' then s40_y <= s39_y; else s40_y <= to_unsigned(64, 10); end if;
            -- 41 .. 43
            s41_y <= s40_y; s42_y <= s41_y; o_y <= s42_y;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.avid    <= ysr(43)(3);
    data_out.hsync_n <= ysr(43)(2);
    data_out.vsync_n <= ysr(43)(1);
    data_out.field_n <= ysr(43)(0);

end architecture flute;
