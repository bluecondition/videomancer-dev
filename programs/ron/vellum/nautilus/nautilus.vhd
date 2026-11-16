-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Nautilus v0.1.0 -- Vellum.  The golden rectangle's whirling squares with their quarter arcs (the
-- Fibonacci spiral), Le Corbusier's Modulor, nautilus diagrams: exponential growth drawn with a ruler and
-- a compass.  A rho:1 rectangle is cut into a piece and a smaller rho:1 rectangle, again and again, turning
-- 90 degrees each time; every piece holds a quarter arc (a quarter ellipse for rho /= phi), and the arcs
-- join into one spiral.  The figure is infinite and self-similar about its pole (the eye) under a zoom of
-- rho with a quarter turn, so it zooms forever.
--
--   Label L has the cut threshold T(L) = TW rho^(lam - L): the distance from the eye to the line cutting
--   piece L off rectangle R_L, in direction W, N, E, S, ... by L mod 4.  A window of 20 entries k holds the
--   labels floor(lam) + 1 + k (entry 0 always in [TW / rho, TW)).  A pixel's piece is the first entry whose
--   cut it lies beyond: the W / E entries depend on x only (a per-field column map), the N / S entries on y
--   only (scanned once per line).  The piece's record (block RAM) gives its sides, so the edges, the arc
--   (an 8-step CORDIC on the piece's coordinates, the V axis stretched by rho - 1/rho so every piece is a
--   square), the arc's echoes (the camera), the chord and the diagonal are distances in AA pixels.
--
--   K1  X       the eye, horizontally (50 % = the golden point of the centred rectangle)
--   K2  Y       the eye, vertically
--   K3  Scale   zoom offset about the eye, +-1.5 levels (exponential), glided
--   K4  Detail  1 arc (Grey: black keylines) / 2 + light edges / 3 + chords / 4 + diagonals (hysteresis)
--   K5  Drift   endless zoom into (or out of) the eye: Hypnos SPEED, 2^-12 .. 2^-3 levels per field
--   K6  Camera  the luma sets the weight of the arc's concentric echoes (an engraving), 0 = none
--   S7  Glide   Off / On
--   S8  Hand    Clockwise / Counter-clockwise
--   S9  Tone    Grey (four greys by direction, edges, white arc) / Ink (white lines; camera XOR)
--   S10 Weight  Bold / Fine (Grey Fine: a faint wash with outlines)
--   S11 Source  Video / Osc
--   P12 Ratio   rho: sqrt 2 (A paper) .. phi (50 %) .. 2
--
-- Chassis (Kinetic riley, without its wave engine, palettes and chroma paths):
--   [A] TIMING                 vblank zone, width, generated avid, sync delay, the timing ID's mode
--   [B] CONTROLS               switches and the Detail steps latched per field; holds and glides run in [C]
--   [C] WHIRL ENGINE           the per-field microcoded engine (after Hardline flute: 37-bit accumulator,
--                              64 x 32 register file, serial multiply and shift; nautilus.py holds the program,
--                              its assembler and emulator), the column map, the line engine, the beam, the
--                              record RAMs, the piece's distances, the CORDIC
--   [D] SOURCE FIELD           Osc lattice, 32-px box, vertical IIR line store, weight table
--   [E] COVERAGE + TONE        coverages, films, opaque edge lines, the arc, Ink XOR, fade
--   [F] OUTPUT CONDITIONING    luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixels 0 .. W + 7 are rendered; output column j is luma [1 2 1] over
-- pixels j + 3 .. j + 5, and the generated avid lands 4 clocks after the delayed source avid, so the
-- picture centre sits on the screen centre.  s_gav_sr(S - 2) flags pixels 0 .. W - 1 at stage S.  One
-- latency (48) in every mode.  U = V = 512 on every pixel.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.nautilus_pkg.all;
use work.all;

architecture nautilus of program_top is
    constant C_LAT : integer := 48;   -- = processing_delay_clks
    constant N_ENT : integer := 20;

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
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- the engine's video mode (nautilus.py MODES): 0 1080p, 1 720p, 2 1080i, 3 480p, 4 576p, 5 480i, 6 576i
    function f_mode(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 5 | 6 | 14 => return to_unsigned(1, 3);
            when 1 | 2 | 10 => return to_unsigned(2, 3);
            when 4          => return to_unsigned(3, 3);
            when 12         => return to_unsigned(4, 3);
            when 0          => return to_unsigned(5, 3);
            when 8          => return to_unsigned(6, 3);
            when others     => return to_unsigned(0, 3);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    function f_clamp16(v : integer) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 5); elsif v > 16 then return to_unsigned(16, 5);
        else return to_unsigned(v, 5); end if;
    end function;
    -- distance (K units) x 1/K8 = (d>>1) + (d>>3) - (d>>6) - (d>>9)
    function f_c607a(d : signed) return signed is
    begin
        return shift_right(d, 1) + shift_right(d, 3);
    end function;
    function f_c607b(d : signed) return signed is
    begin
        return shift_right(d, 6) + shift_right(d, 9);
    end function;
    -- a + b or a - b on ONE carry chain (the low column makes the carry-in)
    function f_as(a, b : signed; sub : std_logic) return signed is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & unsigned(a) & '1';
        if sub = '1' then v_b := '0' & (not unsigned(b)) & '1'; else v_b := '0' & unsigned(b) & '0'; end if;
        v_r := v_a + v_b;
        return signed(v_r(a'length downto 1));
    end function;
    -- the arc's coverage of one side: clamp(2 d + 8, 0, 16), d in Q3 AA units
    function f_cova(d : signed(10 downto 0)) return unsigned is
    begin
        if d <= -4 then return to_unsigned(0, 5);
        elsif d >= 4 then return to_unsigned(16, 5);
        else return unsigned(d(3 downto 0) & '0') + 8; end if;
    end function;
    -- Ink: 64 + (x 876) >> 4
    function f_ink(x : unsigned(4 downto 0)) return unsigned is
        type t_ink is array (0 to 16) of integer;
        constant C : t_ink := (64, 118, 173, 228, 283, 337, 392, 447, 502, 556, 611, 666, 721, 775, 830, 885, 940);
    begin
        if x > 16 then return to_unsigned(940, 10); end if;
        return to_unsigned(C(to_integer(x)), 10);
    end function;
    -- the record port's saturation at 65535
    function f_sat16(v : signed(36 downto 0)) return unsigned is
    begin
        if v(36 downto 16) /= 0 then return x"FFFF"; else return unsigned(v(15 downto 0)); end if;
    end function;

    attribute no_rw_check : boolean;

    -- the engine's opcodes (nautilus.py OPC)
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
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";      -- 0 return, 1 halt

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_moder, s_mode : unsigned(2 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_oxs  : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys  : unsigned(9 downto 0) := to_unsigned(121, 10);

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    -- the syncs' C_LAT delay: a block-RAM ring (write at sy_wp, read C_LAT - 2 behind, + RAM and fabric registers)
    type t_sy is array (0 to 255) of std_logic_vector(1 downto 0);
    signal sy_ram : t_sy := (others => "11");
    attribute no_rw_check of sy_ram : signal is true;
    signal sy_wp : unsigned(7 downto 0) := to_unsigned(C_LAT - 2, 8);
    signal sy_rp : unsigned(7 downto 0) := (others => '0');
    signal sy_q, sy_f : std_logic_vector(1 downto 0) := "11";
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k): pixels 0 .. W - 1 at stage k + 2
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 15) := (others => '0');  -- s_st(k) = g_start delayed k + 1

    -- [B] switches and Detail (latched at the zone start)
    signal s_glide, s_hand, s_ink, s_fine, s_src : std_logic := '0';
    signal s_det : unsigned(1 downto 0) := to_unsigned(1, 2);

    -- [C] the engine
    signal rf_ram : t_c_nt_rfi := C_NT_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc, e_lr : unsigned(9 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";
    signal x_mul, x_shr, x_halt, x_out, x_st : std_logic := '0';
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal w_in  : signed(17 downto 0);
    signal ft_adr : unsigned(8 downto 0) := (others => '0');
    signal ft_q  : unsigned(15 downto 0) := (others => '0');
    signal z_xl  : signed(22 downto 0) := (others => '0');
    signal z_ahi : unsigned(1 downto 0) := (others => '0');
    signal z_akh : unsigned(3 downto 0) := (others => '0');
    signal e_done : std_logic := '0';
    signal e_wd   : unsigned(1 downto 0) := (others => '0');

    -- [C] per-field constants (the engine's outputs)
    signal c_rot  : unsigned(1 downto 0) := (others => '0');
    signal c_Xb0, c_Xbs0, c_Yb0, c_Ybs0 : signed(24 downto 0) := (others => '0');   -- Q12 AA units
    signal c_xs, c_xss, c_ys, c_yss : signed(13 downto 0) := (others => '0');
    signal c_we, c_wa, c_wc, c_wh : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal c_cw   : unsigned(6 downto 0) := (others => '0');
    signal c_gst  : unsigned(6 downto 0) := (others => '0');
    signal c_ink, c_fine : std_logic := '0';
    signal c_det  : unsigned(1 downto 0) := to_unsigned(1, 2);
    signal c_g    : unsigned(9 downto 0) := to_unsigned(432, 10);
    signal c_le   : unsigned(9 downto 0) := to_unsigned(607, 10);
    signal c_b0, c_b1, c_b2, c_b3 : unsigned(9 downto 0) := (others => '0');

    -- [C] record RAMs (written by the engine in the zone; read by the pixel path, the column map and the
    -- line engine)
    type t_rec is array (0 to 255) of unsigned(15 downto 0);
    signal rec0, rec1, rec2, rec3, rec4, rec5, rec7 : t_rec := (others => (others => '0'));
    attribute no_rw_check of rec0 : signal is true;
    attribute no_rw_check of rec1 : signal is true;
    attribute no_rw_check of rec2 : signal is true;
    attribute no_rw_check of rec3 : signal is true;
    attribute no_rw_check of rec4 : signal is true;
    attribute no_rw_check of rec5 : signal is true;
    attribute no_rw_check of rec7 : signal is true;
    signal rw_a  : unsigned(4 downto 0) := (others => '0');
    signal rw_we0, rw_we1, rw_we2, rw_we3, rw_we4, rw_we5, rw_we7 : std_logic;
    signal rw_d0, rw_dsat, rw_d5, rw_d7 : unsigned(15 downto 0);
    signal r0_a  : unsigned(4 downto 0) := (others => '0');
    signal rq0, rq1, rq2, rq3, rq4, rq5, rq7 : unsigned(15 downto 0) := (others => '0');
    signal rf0, rf1, rf2, rf3, rf4, rf5, rf7 : unsigned(15 downto 0) := (others => '0');
    type t_k is array (6 to 16) of unsigned(4 downto 0);
    signal sd_k : t_k := (others => (others => '0'));

    -- [C] the column map: the first W / E entry hit in each column (filled after the engine, in the zone)
    type t_xm is array (0 to 2047) of unsigned(4 downto 0);
    signal xm_ram : t_xm := (others => to_unsigned(20, 5));
    attribute no_rw_check of xm_ram : signal is true;
    signal xm_q, s3_kx : unsigned(4 downto 0) := to_unsigned(20, 5);
    signal xm_we : std_logic := '0';
    signal xm_wa : unsigned(10 downto 0) := (others => '0');
    signal xm_wd : unsigned(4 downto 0) := (others => '0');
    signal fl_run, fl_iss : std_logic := '0';
    signal fl_n, fl_end : unsigned(11 downto 0) := (others => '0');
    signal fl_j : unsigned(2 downto 0) := (others => '0');
    signal fl_x : signed(24 downto 0) := (others => '0');
    signal fl_a : unsigned(4 downto 0) := (others => '0');
    -- the scan pipeline shared by the column map and the line engine: issue (entry, projection, last)
    signal q_v1, q_v2, q_v3, q_l1, q_l2, q_l3, q_f1, q_f2, q_f3 : std_logic := '0';
    signal q_g1, q_g2, q_g3 : std_logic := '0';
    signal q_k2, q_k3 : unsigned(4 downto 0) := (others => '0');
    signal q_p1, q_p2, q_p3 : signed(15 downto 0) := (others => '0');
    signal q_n1, q_n2, q_n3 : unsigned(10 downto 0) := (others => '0');
    signal q_best, q_bestl : unsigned(4 downto 0) := to_unsigned(20, 5);
    signal q_kick : std_logic := '0';

    -- [C] line engine and beam
    signal n_Yb, n_Ybs : signed(24 downto 0) := (others => '0');
    signal ls_run : std_logic := '0';
    signal ls_j : unsigned(2 downto 0) := (others => '0');
    signal ls_trig : std_logic := '0';
    signal l_kns, n_kns : unsigned(4 downto 0) := to_unsigned(20, 5);
    signal b_Xb, b_Xbs : signed(24 downto 0) := (others => '0');
    signal l_y, l_yn, l_ys, l_ysn : signed(15 downto 0) := (others => '0');
    signal px_n : unsigned(10 downto 0) := (others => '0');

    -- [C] pixel path
    type t_s16x4 is array (0 to 3) of signed(15 downto 0);
    signal s2_x, s2_xs : signed(15 downto 0) := (others => '0');
    type t_x is array (3 to 5) of signed(15 downto 0);
    signal sd_x, sd_xs : t_x := (others => (others => '0'));
    signal s6_x, s6_xn, s6_xs, s6_xsn : signed(15 downto 0) := (others => '0');
    signal s4_k, s5_k : unsigned(4 downto 0) := to_unsigned(20, 5);
    signal s6_s1, s7_s1 : unsigned(1 downto 0) := (others => '0');
    signal s7_p1, s7_p2, s7_p2s : signed(15 downto 0) := (others => '0');
    signal s8_u   : signed(16 downto 0) := (others => '0');
    signal s8_v1, s8_v3, s8_v1s : signed(17 downto 0) := (others => '0');
    signal s8_a   : unsigned(17 downto 0) := (others => '0');
    signal s9_u4  : signed(18 downto 0) := (others => '0');
    signal s9_dg  : signed(18 downto 0) := (others => '0');
    signal s9_cs  : signed(18 downto 0) := (others => '0');
    signal s9_e1  : signed(17 downto 0) := (others => '0');
    signal s9_v3  : signed(17 downto 0) := (others => '0');
    signal s9_a   : unsigned(17 downto 0) := (others => '0');
    signal s10_e2 : signed(18 downto 0) := (others => '0');
    signal s10_e1 : signed(17 downto 0) := (others => '0');
    signal s10_dh : signed(19 downto 0) := (others => '0');
    signal s10_ag, s11_g1, s11_g2, s12_agc : unsigned(9 downto 0) := (others => '0');
    signal s11_m  : signed(18 downto 0) := (others => '0');
    signal s12_te : signed(19 downto 0) := (others => '0');
    signal s11_ad : unsigned(9 downto 0) := (others => '0');
    signal s12_h1, s12_h2 : unsigned(9 downto 0) := (others => '0');
    signal s13_adc : unsigned(9 downto 0) := (others => '0');
    type t_s21x9 is array (0 to 8) of signed(20 downto 0);
    signal c1x, c1y : t_s21x9 := (others => (others => '0'));
    signal c1x8 : signed(20 downto 0) := (others => '0');
    type t_fd is array (19 to 34) of unsigned(4 downto 0);
    signal sd_fd : t_fd := (others => (others => '0'));
    signal sd_na : std_logic_vector(19 to 22) := (others => '0');
    type t_s1d is array (8 to 24) of unsigned(1 downto 0);
    signal sd_s1 : t_s1d := (others => "00");
    -- distances to coverage
    signal s19_dak : signed(19 downto 0) := (others => '0');
    signal s20_da : signed(10 downto 0) := (others => '0');
    signal s20_q  : signed(19 downto 0) := (others => '0');
    signal s21_pa, s21_qa : signed(10 downto 0) := (others => '0');
    signal s21_rz : std_logic := '0';
    signal s21_dk : unsigned(7 downto 0) := (others => '0');
    signal s22_da : signed(10 downto 0) := (others => '0');
    signal s22_dr : unsigned(7 downto 0) := (others => '0');
    signal s22_rz : std_logic := '0';
    type t_cov is array (natural range <>) of unsigned(4 downto 0);
    signal sd_ce : t_cov(13 to 14) := (others => (others => '0'));
    signal sd_ch : t_cov(14 to 14) := (others => (others => '0'));
    signal sd_cg : t_cov(13 to 14) := (others => (others => '0'));
    signal ce_d  : t_cov(24 to 27) := (others => (others => '0'));
    -- the coverage ring: {edge, diagonal, chord} written at stage 14, read 7 behind, + RAM and fabric registers
    type t_cv is array (0 to 255) of unsigned(14 downto 0);
    signal cv_ram : t_cv := (others => (others => '0'));
    attribute no_rw_check of cv_ram : signal is true;
    signal cv_wp : unsigned(7 downto 0) := to_unsigned(8, 8);
    signal cv_rp : unsigned(7 downto 0) := (others => '0');
    signal cv_q, cv_f : unsigned(14 downto 0) := (others => '0');
    signal ce23, cg23, ch23 : unsigned(4 downto 0);
    signal s23_ca, s23_cr : unsigned(4 downto 0) := (others => '0');
    signal s23_d, s23_dw : signed(10 downto 0) := (others => '0');
    signal s23_rd : signed(8 downto 0) := (others => '0');
    signal s23_w4 : unsigned(4 downto 0) := (others => '0');
    signal s23_na, s23_rz : std_logic := '0';
    -- tone
    signal s24_t0 : unsigned(9 downto 0) := (others => '0');
    signal s24_f1, s24_f2 : signed(11 downto 0) := (others => '0');
    signal s24_m1, s24_m2, s25_fig : unsigned(4 downto 0) := (others => '0');
    type t_ca is array (24 to 31) of unsigned(4 downto 0);
    signal sd_ca : t_ca := (others => (others => '0'));
    signal s24_cr, s25_cr : unsigned(4 downto 0) := (others => '0');
    signal s25_t  : signed(12 downto 0) := (others => '0');
    signal s26_t  : unsigned(9 downto 0) := (others => '0');
    signal s26_x  : unsigned(4 downto 0) := (others => '0');
    signal s27_t, s28_t, s29_t, s30_t, s31_t, s32_t : unsigned(9 downto 0) := (others => '0');
    signal s27_d, s30_d : signed(10 downto 0) := (others => '0');
    signal s28_p, s31_p : signed(15 downto 0) := (others => '0');
    signal s33_d  : signed(10 downto 0) := (others => '0');
    signal s34_p  : signed(15 downto 0) := (others => '0');
    signal s35_y, s36_y, s37_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
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
    signal s_bv, s_ev : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb8, s_eq : unsigned(7 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(7 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(7 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');
    signal s8_q, s8_qp, s9_l : unsigned(7 downto 0) := (others => '0');
    type t_wt is array (0 to 255) of unsigned(7 downto 0);
    signal wt_ram : t_wt := (others => (others => '0'));
    attribute no_rw_check of wt_ram : signal is true;
    signal wt_q, wt_wd, wt_wa, wt_i : unsigned(7 downto 0) := (others => '0');
    signal wt_we, wt_run, wt_go : std_logic := '0';
    signal wt_acc : unsigned(15 downto 0) := (others => '0');
    signal s22_wr : unsigned(5 downto 0) := (others => '0');

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yd : t_u10(1 to 5) := (others => to_unsigned(64, 10));
    signal s38_ya : unsigned(11 downto 0) := (others => '0');
    signal s38_yb : unsigned(9 downto 0) := (others => '0');
    signal s39_y, s40_y, s41_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_ft : process(clk)
    begin
        if rising_edge(clk) then
            ft_q <= C_NT_FT(to_integer(ft_adr));
        end if;
    end process p_ft;

    -- the record port: R1 R2 R4 saturate, R0 takes A >> 16 in its top bits, R7 packs fade / no-arc / AK >> 16
    rw_dsat <= f_sat16(e_acc);
    rw_d0   <= z_ahi & unsigned(e_acc(13 downto 0));
    rw_d5   <= x"FFFF" when e_acc(36 downto 19) /= 0 else unsigned(e_acc(15 downto 0));
    rw_d7   <= "0000000" & to_unsigned(0, 5) & z_akh when e_acc(36) = '1' or rw_a = N_ENT else
               "0000000" & to_unsigned(16, 5) & z_akh when e_acc(35 downto 4) /= 0 else
               "0000000" & unsigned(e_acc(4 downto 0)) & z_akh;
    rw_we0 <= '1' when e_out = '1' and e_irf(4 downto 0) = "00010" else '0';
    rw_we1 <= '1' when e_out = '1' and e_irf(4 downto 0) = "00011" else '0';
    rw_we2 <= '1' when e_out = '1' and e_irf(4 downto 0) = "00100" else '0';
    rw_we3 <= '1' when e_out = '1' and e_irf(4 downto 0) = "00101" else '0';
    rw_we4 <= '1' when e_out = '1' and e_irf(4 downto 0) = "00110" else '0';
    rw_we5 <= '1' when e_out = '1' and e_irf(4 downto 0) = "00111" else '0';
    rw_we7 <= '1' when e_out = '1' and e_irf(4 downto 0) = "01000" else '0';

    -- rec0 is read by the pixel path (stage 5), the column map and the line engine (zone / horizontal blank)
    r0_a <= fl_a when q_v1 = '1' else s5_k;
    p_rec0 : process(clk) begin if rising_edge(clk) then
        rq0 <= rec0(to_integer(r0_a)); if rw_we0 = '1' then rec0(to_integer(rw_a)) <= rw_d0; end if; end if; end process;
    p_rec1 : process(clk) begin if rising_edge(clk) then
        rq1 <= rec1(to_integer(s5_k)); if rw_we1 = '1' then rec1(to_integer(rw_a)) <= rw_dsat; end if; end if; end process;
    p_rec2 : process(clk) begin if rising_edge(clk) then
        rq2 <= rec2(to_integer(s5_k)); if rw_we2 = '1' then rec2(to_integer(rw_a)) <= rw_dsat; end if; end if; end process;
    p_rec3 : process(clk) begin if rising_edge(clk) then
        rq3 <= rec3(to_integer(s5_k)); if rw_we3 = '1' then rec3(to_integer(rw_a)) <= unsigned(e_acc(15 downto 0)); end if; end if; end process;
    p_rec4 : process(clk) begin if rising_edge(clk) then
        rq4 <= rec4(to_integer(s5_k)); if rw_we4 = '1' then rec4(to_integer(rw_a)) <= rw_dsat; end if; end if; end process;
    p_rec5 : process(clk) begin if rising_edge(clk) then
        rq5 <= rec5(to_integer(sd_k(16))); if rw_we5 = '1' then rec5(to_integer(rw_a)) <= rw_d5; end if; end if; end process;
    p_rec7 : process(clk) begin if rising_edge(clk) then
        rq7 <= rec7(to_integer(sd_k(16))); if rw_we7 = '1' then rec7(to_integer(rw_a)) <= rw_d7; end if; end if; end process;

    -- the coverage ring (stage 14 -> 23)
    p_cv : process(clk)
    begin
        if rising_edge(clk) then
            cv_q <= cv_ram(to_integer(cv_rp));
            cv_ram(to_integer(cv_wp)) <= sd_ce(14) & sd_cg(14) & sd_ch(14);
            cv_f <= cv_q;
            cv_wp <= cv_wp + 1; cv_rp <= cv_rp + 1;
        end if;
    end process p_cv;
    ce23 <= cv_f(14 downto 10); cg23 <= cv_f(9 downto 5); ch23 <= cv_f(4 downto 0);

    -- the syncs' delay ring
    p_sy : process(clk)
    begin
        if rising_edge(clk) then
            sy_q <= sy_ram(to_integer(sy_rp));
            sy_ram(to_integer(sy_wp)) <= data_in.vsync_n & data_in.hsync_n;
            sy_f <= sy_q;
            sy_wp <= sy_wp + 1; sy_rp <= sy_rp + 1;
        end if;
    end process p_sy;

    -- the column map
    p_xm : process(clk)
    begin
        if rising_edge(clk) then
            xm_q <= xm_ram(to_integer(px_n));
            if xm_we = '1' then xm_ram(to_integer(xm_wa)) <= xm_wd; end if;
        end if;
    end process p_xm;

    -- weight table: written in the zone, read by the pixel path
    p_wtram : process(clk)
    begin
        if rising_edge(clk) then
            wt_q <= wt_ram(to_integer(s9_l));
            if wt_we = '1' then wt_ram(to_integer(wt_wa)) <= wt_wd; end if;
        end if;
    end process p_wtram;

    -- box delay: read 32 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    -- luma line store (half horizontal resolution): port 1 = the IIR's own read, port 2 = the pixel path
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 1)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] the per-field latches
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k  : unsigned(9 downto 0);
        variable v_c  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_moder <= f_mode(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

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
                s_glide <= registers_in(6)(0);
                s_hand  <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_mode  <= s_moder;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- Detail: 4 steps, leave a step only 8 counts past its edge (no end dead zones)
                v_k := unsigned(registers_in(3));
                v_c := signed(resize(v_k, 12)) - signed(resize(s_det & "10000000", 12));
                if v_c > 136 or v_c < -136 then s_det <= v_k(9 downto 8); end if;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is                -- the Osc's y step, (ys x 121) >> 8
                when 0          => s_oys <= to_unsigned(537, 10);
                when 4          => s_oys <= to_unsigned(272, 10);
                when 8          => s_oys <= to_unsigned(453, 10);
                when 12         => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10);
                when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others     => s_oys <= to_unsigned(121, 10);
            end case;

            -- sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the core
            -- on every input, so it only gets a register)
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 14);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width)
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
    -- [C] THE PER-FIELD ENGINE (after Hardline flute).  Started at every zone
    -- start from address 0; three clocks per micro-op (register-file read, the
    -- ROM word copied to fabric / operand, decode and next address / execute),
    -- serial multiply (18 steps, ACC(17:0) x operand(17:0), both signed) and a
    -- right shift of four bits a clock while four or more remain.
    ---------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        signed(resize(s_mode, 18))                        when x"8",
        signed(resize(ft_q, 18))                          when x"9",
        resize(z_xl(22 downto 16), 18)                    when x"A",
        signed(resize(unsigned(z_xl(15 downto 8)), 18))   when x"B",
        signed(resize(unsigned(z_xl(7 downto 0)), 18))    when x"C",
        signed(resize(unsigned(registers_in(to_integer(unsigned(e_irf(2 downto 0))))), 18)) when others;

    e_out <= '1' when e_run = '1' and e_ph = "10" and x_out = '1' else '0';

    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_gate, v_sub, v_shift, v_last, v_cond, v_fetch : std_logic;
        variable v_bx  : signed(36 downto 0);
        variable v_t   : signed(37 downto 0);
        variable v_sum : signed(36 downto 0);
    begin
        if rising_edge(clk) then
            v_op := e_irf(15 downto 12);
            v_arg := e_irf(11 downto 0);
            v_fetch := '0';
            e_done <= '0';
            if e_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(e_ir(5 downto 0))));
            end if;
            if e_run = '1' and e_ph = "10" and x_st = '1' then
                rf_ram(to_integer(unsigned(e_irf(5 downto 0)))) <= std_logic_vector(e_acc(31 downto 0));
            end if;

            -- start at the zone start; a run still going there (only in a simulator's short vertical blank)
            -- finishes first, and a watchdog restarts one that has not finished for three fields
            if s_zgo = '1' and (e_run = '0' or e_wd = 3) then
                e_run <= '1'; e_pc <= (others => '0'); e_ph <= "11"; e_wd <= (others => '0');
            elsif e_run = '1' then
                if s_zgo = '1' then e_wd <= e_wd + 1; end if;
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
                            when OP_IN   => e_b <= resize(w_in, 37);
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
                        elsif v_op = OP_SHR then e_cnt <= unsigned(v_arg(4 downto 0));
                        else e_cnt <= (others => '0'); end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := e_acc(36);
                            when "001"  => v_cond := s_glide;
                            when "010"  => v_cond := s_hand;
                            when "011"  => v_cond := s_ink;
                            when others => v_cond := s_fine;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then e_pc <= e_pc + 2;
                        elsif v_op = OP_JMP then e_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then e_pc <= unsigned(v_arg(9 downto 0)); e_lr <= e_pc + 1;
                        elsif v_op = OP_SPC and v_arg(0) = '0' then e_pc <= e_lr;
                        else e_pc <= e_pc + 1; end if;
                        x_mul <= '0'; x_shr <= '0'; x_halt <= '0'; x_out <= '0'; x_st <= '0';
                        x_gate <= '0'; x_subr <= '0'; x_shift <= '0'; x_cz <= '0';
                        case v_op is
                            when OP_LD | OP_ADD | OP_LDI | OP_ADDI | OP_IN => x_gate <= '1';
                            when OP_SUB | OP_SUBI => x_gate <= '1'; x_subr <= '1';
                            when OP_MUL | OP_MULI => x_mul <= '1'; x_shift <= '1'; x_gate <= e_acc(0);
                            when OP_SHR  => x_shr <= '1';
                            when OP_ST   => x_st <= '1';
                            when OP_OUT  => x_out <= '1';
                            when OP_SPC  => x_halt <= v_arg(0);
                            when others  => null;
                        end case;
                        e_ph <= "10";
                    when others =>                        -- execute (the adder sees registers only)
                        v_gate := x_gate; v_sub := x_subr; v_shift := x_shift; v_last := '1';
                        if v_gate = '0' then v_bx := (others => '0');
                        elsif v_sub = '1' then v_bx := not e_b;
                        else v_bx := e_b; end if;
                        v_t := (e_acc & '1') + (v_bx & (v_sub and v_gate));
                        v_sum := v_t(37 downto 1);
                        if x_mul = '1' then
                            if x_cz = '0' then v_last := '0'; end if;
                            e_cnt <= e_cnt - 1;
                            if e_cnt = 1 then x_cz <= '1'; x_subr <= '1'; end if;   -- the next step is the last
                            x_gate <= v_sum(1);                                     -- the next multiplier bit
                        end if;
                        if x_halt = '1' then e_run <= '0'; end if;
                        if x_shr = '1' then
                            if e_cnt >= 4 then
                                e_acc <= shift_right(e_acc, 4); e_cnt <= e_cnt - 4;
                                if e_cnt /= 4 then v_last := '0'; end if;
                            else
                                e_acc <= shift_right(e_acc, 1); e_cnt <= e_cnt - 1;
                                if e_cnt /= 1 then v_last := '0'; end if;
                            end if;
                        elsif v_shift = '1' then e_acc <= v_sum(36) & v_sum(36 downto 1);
                        else e_acc <= v_sum; end if;
                        if v_last = '1' then
                            v_fetch := '1';
                            e_ph <= "00";
                        end if;
                end case;
            end if;
            if v_fetch = '1' then
                e_ir <= C_NT_UC(to_integer(e_pc));
            end if;

            -- the engine's output registers
            if e_out = '1' then
                case to_integer(unsigned(e_irf(4 downto 0))) is
                    when 0  => ft_adr <= unsigned(e_acc(8 downto 0));
                    when 1  => rw_a   <= unsigned(e_acc(4 downto 0));
                    when 5  => z_ahi  <= unsigned(e_acc(17 downto 16));
                    when 7  => if e_acc(36 downto 19) /= 0 then z_akh <= "1111";
                               else z_akh <= '0' & unsigned(e_acc(18 downto 16)); end if;
                    when 9  => c_Xb0  <= e_acc(28 downto 4);
                    when 10 => c_Xbs0 <= e_acc(28 downto 4);
                    when 11 => c_Yb0  <= e_acc(28 downto 4);
                    when 12 => c_Ybs0 <= e_acc(28 downto 4);
                    when 13 => c_xs   <= e_acc(17 downto 4);
                    when 14 => c_xss  <= e_acc(17 downto 4);
                    when 15 => c_ys   <= e_acc(17 downto 4);
                    when 16 => c_yss  <= e_acc(17 downto 4);
                    when 17 => c_we   <= unsigned(e_acc(6 downto 0));
                    when 18 => c_wa   <= unsigned(e_acc(6 downto 0));
                    when 19 => c_wc   <= unsigned(e_acc(6 downto 0));
                    when 20 => c_wh   <= unsigned(e_acc(6 downto 0));
                    when 21 => c_cw   <= unsigned(e_acc(6 downto 0));
                    when 22 => c_gst  <= unsigned(e_acc(6 downto 0));
                    when 23 => c_rot  <= unsigned(e_acc(1 downto 0));
                    when 24 => e_done <= '1';
                    when 25 => z_xl   <= e_acc(22 downto 0);
                    when others => null;
                end case;
            end if;
            -- the tones (constants by Tone / Weight / Detail, latched at the zone start)
            if s_zgo = '1' then
                c_ink <= s_ink; c_fine <= s_fine; c_det <= s_det;
                if s_ink = '1' then
                    c_b0 <= to_unsigned(64, 10); c_b1 <= to_unsigned(64, 10);
                    c_b2 <= to_unsigned(64, 10); c_b3 <= to_unsigned(64, 10); c_g <= to_unsigned(300, 10);
                elsif s_fine = '1' then
                    c_b0 <= to_unsigned(228, 10); c_b1 <= to_unsigned(174, 10);
                    c_b2 <= to_unsigned(119, 10); c_b3 <= to_unsigned(64, 10); c_g <= to_unsigned(200, 10);
                else
                    c_b0 <= to_unsigned(721, 10); c_b1 <= to_unsigned(502, 10);
                    c_b2 <= to_unsigned(283, 10); c_b3 <= to_unsigned(64, 10); c_g <= to_unsigned(432, 10);
                end if;
                if s_det = 0 then c_le <= to_unsigned(64, 10);
                elsif s_fine = '1' then c_le <= to_unsigned(502, 10);
                else c_le <= to_unsigned(607, 10); end if;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [C] THE COLUMN MAP and THE LINE ENGINE.  One scan pipeline: an issue
    -- (entry k, its projection, the column) reads rec0 (T_k), two clocks later
    -- the compare; the first hit of each group of five is the result.
    --   column map (after the engine, in the zone): for columns 0 .. W + 7 the
    --     W entries (x < 0, projection -X) or the E entries (projection X)
    --   line engine: at the zone (line 0) and in each line's horizontal blank,
    --     the N / S entries for the next line's Y
    ---------------------------------------------------------------------------
    p_scan : process(clk)
        variable v_c : unsigned(1 downto 0);
        variable v_y : signed(15 downto 0);
        variable v_b : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            q_kick <= '0';
            xm_we <= '0';
            wt_go <= '0';
            -- issue: the line engine first (it must finish within a horizontal blank), then the column map
            fl_iss <= '0';
            if e_done = '1' then
                fl_run <= '1'; fl_n <= (others => '0'); fl_j <= (others => '0'); fl_x <= c_Xb0;
                fl_end <= s_w + 8;
                wt_go <= '1';
            end if;
            if ls_run = '1' then
                v_y := n_Yb(24 downto 9);
                if v_y(15) = '1' then v_c := 1 - c_rot; else v_c := 3 - c_rot; end if;
                fl_a <= ls_j & v_c;
                q_v1 <= '1'; q_f1 <= '0';
                if ls_j = 0 then q_g1 <= '1'; else q_g1 <= '0'; end if;
                if v_y(15) = '1' then q_p1 <= -v_y; else q_p1 <= v_y; end if;
                if ls_j = 4 then q_l1 <= '1'; ls_run <= '0'; ls_j <= (others => '0');
                else q_l1 <= '0'; ls_j <= ls_j + 1; end if;
            elsif fl_run = '1' and e_done = '0' and g_start = '0' and g_avid = '0' and s_gav_sr(10) = '0'
                  and s_gav_sr(11) = '0' then          -- (pauses while the pixel path reads rec0)
                fl_iss <= '1';
                if fl_x(24) = '1' then v_c := 0 - c_rot; else v_c := 2 - c_rot; end if;
                fl_a <= fl_j & v_c;
                q_v1 <= '1'; q_f1 <= '1';
                if fl_j = 0 then q_g1 <= '1'; else q_g1 <= '0'; end if;
                if fl_x(24) = '1' then q_p1 <= -fl_x(24 downto 9); else q_p1 <= fl_x(24 downto 9); end if;
                q_n1 <= fl_n(10 downto 0);
                if fl_j = 4 then
                    q_l1 <= '1'; fl_j <= (others => '0');
                    fl_x <= fl_x + resize(c_xs, 25);
                    fl_n <= fl_n + 1;
                    if fl_n + 1 = fl_end then fl_run <= '0'; q_kick <= '1'; end if;
                else
                    q_l1 <= '0'; fl_j <= fl_j + 1;
                end if;
            else
                q_v1 <= '0'; q_l1 <= '0';
            end if;
            if ls_trig = '1' then ls_run <= '1'; ls_j <= (others => '0'); end if;
            -- (fl_a is the read address the clock after the issue; q_k* carries it)
            -- 2 (rq0), 3 (the compare against rf0's T)
            q_v2 <= q_v1; q_l2 <= q_l1; q_f2 <= q_f1; q_g2 <= q_g1; q_k2 <= fl_a; q_p2 <= q_p1; q_n2 <= q_n1;
            q_v3 <= q_v2; q_l3 <= q_l2; q_f3 <= q_f2; q_g3 <= q_g2; q_k3 <= q_k2; q_p3 <= q_p2; q_n3 <= q_n2;
            -- 3: the compare (projection > T_k, both Q3); the first hit of the group of five
            -- (one accumulator per stream: line-engine issues may fall between a column's five)
            if q_v3 = '1' then
                if q_g3 = '1' then v_b := to_unsigned(20, 5);
                elsif q_f3 = '1' then v_b := q_best;
                else v_b := q_bestl; end if;
                if v_b = 20 and q_p3 > signed(resize(rf0(13 downto 0), 16)) then v_b := q_k3; end if;
                if q_f3 = '1' then q_best <= v_b; else q_bestl <= v_b; end if;
                if q_l3 = '1' then
                    if q_f3 = '1' then xm_we <= '1'; xm_wa <= q_n3; xm_wd <= v_b;
                    else n_kns <= v_b; end if;
                end if;
            end if;
        end if;
    end process p_scan;

    ---------------------------------------------------------------------------
    -- [C] line engine (Y of the next line, loaded at each line start) and the
    -- beam (X, stage 1).  Lines advance on active lines only.
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            ls_trig <= '0';
            -- line 0: at the zone start, and again after this field's column map if that is still in the zone
            if s_zgo = '1' or (q_kick = '1' and s_zone = '1') then
                n_Yb <= c_Yb0; n_Ybs <= c_Ybs0; ls_trig <= '1';
            elsif g_start = '1' then
                n_Yb <= n_Yb + resize(c_ys, 25); n_Ybs <= n_Ybs + resize(c_yss, 25);
                l_kns <= n_kns;
            end if;
            -- the next line's scan, once this line's last rendered pixel has read its record
            if s_gav_sr(11) = '0' and s_gav_sr(12) = '1' then ls_trig <= '1'; end if;
            if g_start = '1' then
                b_Xb <= c_Xb0; b_Xbs <= c_Xbs0; px_n <= (others => '0');
                l_y  <= n_Yb(24 downto 9);  l_yn  <= -n_Yb(24 downto 9);
                l_ys <= n_Ybs(24 downto 9); l_ysn <= -n_Ybs(24 downto 9);
            else
                b_Xb <= b_Xb + resize(c_xs, 25); b_Xbs <= b_Xbs + resize(c_xss, 25);
                px_n <= px_n + 1;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [C] pixel path: the piece (stages 2 .. 7), its distances (8 .. 11) and
    -- the CORDIC (10 .. 17)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s0 : unsigned(1 downto 0);
        variable v_m  : signed(18 downto 0);
        variable v_x, v_y : signed(20 downto 0);
        variable v_a, v_as : t_s16x4;
        variable v_ab19 : signed(18 downto 0);
        variable v_ab20 : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: X and the stretched X (Q3 units); the projections are A_0 = -X (W), A_1 = -Y (N),
            -- A_2 = X (E), A_3 = Y (S), Y and its stretched copy from the line registers.  The column map's
            -- read (address = the pixel, stage 1) arrives here.
            s2_x <= b_Xb(24 downto 9); s2_xs <= b_Xbs(24 downto 9);
            sd_x(3) <= s2_x; sd_xs(3) <= s2_xs;
            for i in 4 to 5 loop sd_x(i) <= sd_x(i - 1); sd_xs(i) <= sd_xs(i - 1); end loop;
            s6_x <= sd_x(5); s6_xn <= -sd_x(5); s6_xs <= sd_xs(5); s6_xsn <= -sd_xs(5);
            -- 3: the map's fabric copy; 4: the piece = the first of the column's and the line's entries
            s3_kx <= xm_q;
            if s3_kx < l_kns then s4_k <= s3_kx; else s4_k <= l_kns; end if;
            s5_k <= s4_k;
            -- 6: the record RAMs' own registers; the piece's direction s1 = (k + rot) mod 4
            s6_s1 <= s5_k(1 downto 0) + c_rot;
            sd_k(6) <= s5_k; for i in 7 to 16 loop sd_k(i) <= sd_k(i - 1); end loop;
            -- 7: fabric copies; P1 = A_s1, P2 = A_s1-1 (and stretched)
            rf0 <= rq0; rf1 <= rq1; rf2 <= rq2; rf3 <= rq3; rf4 <= rq4;
            rf5 <= rq5; rf7 <= rq7;                -- (stage 18: read at the delayed address)
            v_s0 := s6_s1 - 1;
            v_a(0) := s6_xn; v_a(1) := l_yn; v_a(2) := s6_x; v_a(3) := l_y;
            v_as(0) := s6_xsn; v_as(1) := l_ysn; v_as(2) := s6_xs; v_as(3) := l_ys;
            s7_p1  <= v_a(to_integer(s6_s1));
            s7_p2  <= v_a(to_integer(v_s0));
            s7_p2s <= v_as(to_integer(v_s0));
            s7_s1 <= s6_s1;
            -- 8: the distances to the piece's sides (Q3 units), its stretched V
            s8_u   <= resize(s7_p1, 17) - signed(resize(rf0(13 downto 0), 17));
            s8_v1  <= signed(resize(rf1, 18)) - resize(s7_p2, 18);
            s8_v3  <= signed(resize(rf2, 18)) + resize(s7_p2, 18);
            s8_v1s <= signed(resize(rf4, 18)) - resize(s7_p2s, 18);
            s8_a   <= rf0(15 downto 14) & rf3;
            sd_s1(8) <= s7_s1; for i in 9 to 24 loop sd_s1(i) <= sd_s1(i - 1); end loop;
            -- 9: the outer side, the diagonal, the chord's sum; the CORDIC inputs
            s9_u4 <= signed(resize(s8_a, 19)) - resize(s8_u, 19);
            s9_dg <= resize(s8_u, 19) - resize(s8_v1s, 19);
            s9_cs <= resize(s8_u, 19) + resize(s8_v1s, 19);
            if s8_u < resize(s8_v1, 17) then s9_e1 <= resize(s8_u, 18); else s9_e1 <= s8_v1; end if;
            s9_v3 <= s8_v3; s9_a <= s8_a;
            if s8_u(16) = '1' then c1x(0) <= (others => '0');
            else c1x(0) <= signed(resize(unsigned(s8_u(15 downto 0)) & '0', 21)); end if;
            if s8_v1s(17) = '1' then c1y(0) <= (others => '0');
            else c1y(0) <= signed(resize(unsigned(s8_v1s(16 downto 0)) & '0', 21)); end if;
            -- 10: the second min, the chord, |diagonal|
            if resize(s9_v3, 19) < s9_u4 then s10_e2 <= resize(s9_v3, 19); else s10_e2 <= s9_u4; end if;
            s10_e1 <= s9_e1;
            s10_dh <= resize(s9_cs, 20) - signed(resize(s9_a, 20));
            if abs(s9_dg) > 1023 then s10_ag <= to_unsigned(1023, 10);
            else v_ab19 := abs(s9_dg); s10_ag <= unsigned(v_ab19(9 downto 0)); end if;
            -- 11, 12: |diagonal| x 1/sqrt 2 = (d>>1) + (d>>3) + (d>>4) + (d>>6) + (d>>8)
            s11_g1 <= shift_right(s10_ag, 1) + shift_right(s10_ag, 3) + shift_right(s10_ag, 4);
            s11_g2 <= shift_right(s10_ag, 6) + shift_right(s10_ag, 8);
            s12_agc <= s11_g1 + s11_g2;
            -- 11: the edge distance, |chord|
            if resize(s10_e1, 19) < s10_e2 then s11_m <= resize(s10_e1, 19); else s11_m <= s10_e2; end if;
            if abs(s10_dh) > 1023 then s11_ad <= to_unsigned(1023, 10);
            else v_ab20 := abs(s10_dh); s11_ad <= unsigned(v_ab20(9 downto 0)); end if;
            -- 12, 13: |chord| x 1/sqrt 2
            s12_h1 <= shift_right(s11_ad, 1) + shift_right(s11_ad, 3) + shift_right(s11_ad, 4);
            s12_h2 <= shift_right(s11_ad, 6) + shift_right(s11_ad, 8);
            s13_adc <= s12_h1 + s12_h2;
            -- the CORDIC: iterations at stages 10 .. 17
            for i in 0 to 7 loop
                v_x := shift_right(c1y(i), i); v_y := shift_right(c1x(i), i);
                c1x(i + 1) <= f_as(c1x(i), v_x, c1y(i)(20));
                c1y(i + 1) <= f_as(c1y(i), v_y, not c1y(i)(20));
            end loop;
            c1x8 <= c1x(8);
            -- the late record word (stage 18): AK, no-arc flag, fade riding along
            sd_fd(19) <= unsigned(rf7(8 downto 4)); for i in 20 to 34 loop sd_fd(i) <= sd_fd(i - 1); end loop;
            sd_na(19) <= rf7(3); for i in 20 to 22 loop sd_na(i) <= sd_na(i - 1); end loop;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 32-px box, edge-replicated
    -- and reloaded at every line start -> vertical 1/8 IIR in the line store
    -- (half horizontal resolution).  The pixel path reads the store,
    -- interpolates and looks the ring weight up: stages 8 .. 11.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d : signed(9 downto 0);
        variable v_a : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart in the
            -- zone from this field's drift (the dazzle fix: line 0 no longer carries the last field's x)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);
            end if;
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

            -- pixel path: 8 fabric copies of this and the last read; 9 the mean; 10 the weight table;
            -- 11 .. 22 the ring half-width riding along
            if s_st(14) = '1' or s_st(15) = '1' then lb_rc <= (others => '0'); else lb_rc <= lb_rc + 1; end if;
            s8_q <= lb_q2; s8_qp <= s8_q;
            v_a := resize(s8_q, 9) + resize(s8_qp, 9) + 1;
            s9_l <= v_a(8 downto 1);
            s22_wr <= wt_q(5 downto 0);

            -- weight table fill: ring half-width (Q3 units) = min(35, (gain x (L - 40)) >> 8)
            wt_we <= '0';
            if wt_go = '1' then
                wt_run <= '1'; wt_i <= (others => '0'); wt_acc <= (others => '0');
            elsif wt_run = '1' then
                wt_we <= '1'; wt_wa <= wt_i;
                if wt_acc(15 downto 8) > 35 then wt_wd <= to_unsigned(35, 8);
                else wt_wd <= wt_acc(15 downto 8); end if;
                if wt_i >= 40 then wt_acc <= wt_acc + c_gst; end if;
                wt_i <= wt_i + 1;
                if wt_i = 255 then wt_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] COVERAGE + TONE (stages 12 .. 37)
    ---------------------------------------------------------------------------
    p_tone : process(clk)
        variable v_e   : signed(12 downto 0);
        variable v_c   : integer range -8192 to 8191;
        variable v_q   : signed(19 downto 0);
        variable v_dk  : signed(8 downto 0);
        variable v_t   : signed(13 downto 0);
        variable v_f   : signed(12 downto 0);
        variable v_cs  : unsigned(5 downto 0);
        variable v_p   : signed(15 downto 0);
        variable v_ak  : unsigned(18 downto 0);
        variable v_g1, v_g2 : signed(15 downto 0);
        variable v_c1, v_c2 : unsigned(4 downto 0);
        variable v_ab9 : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            -- 12, 13: the edge coverage, clamp(2 (we / 2 + 4 - e), 0, 16)
            s12_te <= resize(signed('0' & shift_right(c_we, 1)), 20) + 4 - resize(s11_m, 20);
            if s12_te <= 0 then sd_ce(13) <= (others => '0');
            elsif s12_te >= 8 then sd_ce(13) <= to_unsigned(16, 5);
            else sd_ce(13) <= unsigned(s12_te(3 downto 0)) & '0'; end if;
            sd_ce(14) <= sd_ce(13);
            -- 13: the diagonal coverage
            v_e := resize(signed('0' & shift_right(c_wc, 1)), 13) + 4 - signed(resize(s12_agc, 13));
            v_c := 2 * to_integer(v_e);
            sd_cg(13) <= f_clamp16(v_c);
            sd_cg(14) <= sd_cg(13);
            -- 14: the chord coverage
            v_e := resize(signed('0' & shift_right(c_wh, 1)), 13) + 4 - signed(resize(s13_adc, 13));
            v_c := 2 * to_integer(v_e);
            sd_ch(14) <= f_clamp16(v_c);
            ce_d(24) <= ce23; for i in 25 to 27 loop ce_d(i) <= ce_d(i - 1); end loop;

            -- 19: the arc distance in K units (Q3)
            v_ak := unsigned(rf7(2 downto 0)) & rf5;
            s19_dak <= signed(resize(v_ak, 20)) - resize(shift_right(c1x8, 1), 20);
            -- 20: clamp, the ring phase
            if s19_dak > 1023 then s20_da <= to_signed(1023, 11); elsif s19_dak < -1024 then s20_da <= to_signed(-1024, 11);
            else s20_da <= resize(s19_dak, 11); end if;
            s20_q <= s19_dak - signed(resize(c_cw, 20));
            -- 21: x 1/K8 (first half); ring index and distance to the ring centre line
            s21_pa <= f_c607a(s20_da); s21_qa <= f_c607b(s20_da);
            v_q := shift_right(s20_q, 8);
            if v_q = 0 or v_q = -1 or sd_na(20) = '1' then s21_rz <= '1'; else s21_rz <= '0'; end if;
            v_dk := signed(resize(unsigned(s20_q(7 downto 0)), 9)) - 128;
            v_ab9 := abs(v_dk); s21_dk <= unsigned(v_ab9(7 downto 0));   -- (|-128| = 128: slice, never a signed resize)
            -- 22
            s22_da <= s21_pa - s21_qa;
            s22_dr <= shift_right(s21_dk, 1) + shift_right(s21_dk, 3) - shift_right(s21_dk, 6) - shift_right(s21_dk, 9);
            s22_rz <= s21_rz;
            -- 23, 24: arc and ring coverages.  clamp(2 min(d, wa - d) + 8) = min(clamp(2 d + 8), clamp(2 (wa - d)
            -- + 8)); rings min(clamp(2 (wr + 4 - dr)), 4 wr).  s23_ca / s23_cr hold the pixel at stage 24, and the
            -- tone section from here on runs one stage behind its numbering.
            s23_d <= s22_da; s23_dw <= resize(signed('0' & c_wa), 11) - s22_da; s23_na <= sd_na(22);
            s23_rd <= signed(resize(s22_wr, 9)) + 4 - signed(resize(s22_dr, 9));
            if s22_wr >= 4 then s23_w4 <= to_unsigned(16, 5); else s23_w4 <= s22_wr(2 downto 0) & "00"; end if;
            s23_rz <= s22_rz;
            v_c1 := f_cova(s23_d); v_c2 := f_cova(s23_dw);
            if s23_na = '1' then s23_ca <= (others => '0');
            elsif v_c1 < v_c2 then s23_ca <= v_c1; else s23_ca <= v_c2; end if;
            if s23_rd <= 0 then v_c1 := (others => '0');
            elsif s23_rd >= 8 then v_c1 := to_unsigned(16, 5);
            else v_c1 := unsigned(s23_rd(3 downto 0)) & '0'; end if;
            if s23_rz = '1' then s23_cr <= (others => '0');
            elsif v_c1 < s23_w4 then s23_cr <= v_c1; else s23_cr <= s23_w4; end if;

            -- 24: the tint; Grey films (rings: toward black on the light tints, white on the dark; chords and
            -- diagonals: light films), Detail gates; Ink: the union of the gated lines
            case to_integer(sd_s1(24)) is
                when 0 => s24_t0 <= c_b0; when 1 => s24_t0 <= c_b1; when 2 => s24_t0 <= c_b2; when others => s24_t0 <= c_b3;
            end case;
            if c_fine = '1' or sd_s1(24)(1) = '1' then
                v_f := signed(resize(s23_cr & "0000", 13)) + signed(resize(s23_cr & "0", 13));
            else
                v_f := -signed(resize(s23_cr & "0000", 13)) - signed(resize(s23_cr & "0", 13));
            end if;
            s24_f1 <= resize(v_f, 12);
            v_cs := (others => '0');
            if c_det(1) = '1' then v_cs := v_cs + resize(ch23, 6); end if;
            if c_det = 3 then v_cs := v_cs + resize(cg23, 6); end if;
            if c_fine = '1' then s24_f2 <= signed(resize(v_cs & "0000", 12)) + signed(resize(v_cs & "00", 12)) + signed(resize(v_cs & "0", 12));
            else s24_f2 <= signed(resize(v_cs & "0000", 12)); end if;
            if c_det /= 0 and ce23 > s23_ca then s24_m1 <= ce23; else s24_m1 <= s23_ca; end if;
            if c_det(1) = '0' then s24_m2 <= (others => '0');
            elsif c_det = 3 and cg23 > ch23 then s24_m2 <= cg23;
            else s24_m2 <= ch23; end if;
            sd_ca(24) <= s23_ca; for i in 25 to 31 loop sd_ca(i) <= sd_ca(i - 1); end loop;
            s24_cr <= s23_cr;
            -- 25: Grey t = t0 + films; Ink figure
            s25_t <= signed(resize(s24_t0, 13)) + resize(s24_f1, 13) + resize(s24_f2, 13);
            if s24_m1 > s24_m2 then s25_fig <= s24_m1; else s25_fig <= s24_m2; end if;
            s25_cr <= s24_cr;
            -- 26: clamp (Grey); Ink: figure XOR rings, anti-aliased as a triangle of the summed coverage
            if s25_t < 64 then s26_t <= to_unsigned(64, 10);
            elsif s25_t > 940 then s26_t <= to_unsigned(940, 10);
            else s26_t <= unsigned(s25_t(9 downto 0)); end if;
            v_cs := resize(s25_fig, 6) + resize(s25_cr, 6);
            if v_cs > 16 then s26_x <= resize(32 - v_cs, 5); else s26_x <= v_cs(4 downto 0); end if;
            -- 27 .. 29: the opaque edge line (Grey), t + ((Le - t) cov) >> 4; Ink tone
            if c_ink = '1' then
                s27_t <= f_ink(s26_x);
                s27_d <= (others => '0');
            else
                s27_t <= s26_t;
                s27_d <= signed(resize(c_le, 11)) - signed(resize(s26_t, 11));
            end if;
            if ce_d(27)(4) = '1' then
                s28_p <= shift_left(resize(s27_d, 16), 4);
            else
                v_g1 := (others => '0');
                if ce_d(27)(3) = '1' then v_g1 := shift_left(resize(s27_d, 16), 3); end if;
                if ce_d(27)(2) = '1' then v_g1 := v_g1 + shift_left(resize(s27_d, 16), 2); end if;
                v_g2 := (others => '0');
                if ce_d(27)(1) = '1' then v_g2 := shift_left(resize(s27_d, 16), 1); end if;
                if ce_d(27)(0) = '1' then v_g2 := v_g2 + resize(s27_d, 16); end if;
                s28_p <= v_g1 + v_g2;
            end if;
            s28_t <= s27_t;
            v_p := signed(resize(s28_t, 16)) + shift_right(s28_p, 4);
            s29_t <= unsigned(v_p(9 downto 0));
            -- 30 .. 32: the arc over white (Grey)
            if c_ink = '1' then s30_d <= (others => '0');
            else s30_d <= to_signed(940, 11) - signed(resize(s29_t, 11)); end if;
            s30_t <= s29_t;
            if sd_ca(30)(4) = '1' then
                s31_p <= shift_left(resize(s30_d, 16), 4);
            else
                v_g1 := (others => '0');
                if sd_ca(30)(3) = '1' then v_g1 := shift_left(resize(s30_d, 16), 3); end if;
                if sd_ca(30)(2) = '1' then v_g1 := v_g1 + shift_left(resize(s30_d, 16), 2); end if;
                v_g2 := (others => '0');
                if sd_ca(30)(1) = '1' then v_g2 := shift_left(resize(s30_d, 16), 1); end if;
                if sd_ca(30)(0) = '1' then v_g2 := v_g2 + resize(s30_d, 16); end if;
                s31_p <= v_g1 + v_g2;
            end if;
            s31_t <= s30_t;
            v_p := signed(resize(s31_t, 16)) + shift_right(s31_p, 4);
            s32_t <= unsigned(v_p(9 downto 0));
            -- 33 .. 35: fade toward the core grey, G + ((t - G) fade) >> 4
            s33_d <= signed(resize(s32_t, 11)) - signed(resize(c_g, 11));
            if sd_fd(34)(4) = '1' then
                s34_p <= shift_left(resize(s33_d, 16), 4);
            else
                v_g1 := (others => '0');
                if sd_fd(34)(3) = '1' then v_g1 := shift_left(resize(s33_d, 16), 3); end if;
                if sd_fd(34)(2) = '1' then v_g1 := v_g1 + shift_left(resize(s33_d, 16), 2); end if;
                v_g2 := (others => '0');
                if sd_fd(34)(1) = '1' then v_g2 := shift_left(resize(s33_d, 16), 1); end if;
                if sd_fd(34)(0) = '1' then v_g2 := v_g2 + resize(s33_d, 16); end if;
                s34_p <= v_g1 + v_g2;
            end if;
            v_p := signed(resize(c_g, 16)) + shift_right(s34_p, 4);
            s35_y <= unsigned(v_p(9 downto 0));
            -- 37: the ground (black) outside pixels 0 .. W + 7
            if s_gav_sr(34) = '1' or s_gav_sr(42) = '1' then s37_y <= s35_y; else s37_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_tone;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 38 .. 41): luma [1 2 1]/4 over pixels
    -- j + 3 .. j + 5, 13/16 toward black, blanking gate; U = V = 512
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd(1) <= s37_y;
            for i in 2 to 5 loop s_yd(i) <= s_yd(i - 1); end loop;
            s38_ya <= resize(s_yd(3), 12) + resize(s_yd(5), 12) + 2;
            s38_yb <= s_yd(4);
            v_y := s38_ya + shift_left(resize(s38_yb, 12), 1);
            s39_y <= v_y(11 downto 2);
            if s39_y > 64 then v_dl := s39_y - 64; else v_dl := (others => '0'); end if;
            s40_y <= s39_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then s41_y <= s40_y; else s41_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s41_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= sy_f(0);
    data_out.vsync_n <= sy_f(1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture nautilus;
