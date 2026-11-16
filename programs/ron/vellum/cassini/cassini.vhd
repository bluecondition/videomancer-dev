-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cassini v0.1.0 -- Vellum.  Cassini ovals and the lemniscate of Bernoulli: bands of constant r1 r2 about two
-- foci (small rings round each focus, the figure-eight, peanuts, ovals and near-circles far out), streaming out
-- of the foci; or two translucent log-spaced targets, one round each focus, moving at their own rates (their
-- XOR is the curvilinear checkerboard whose diagonals are the Cassini ovals and the Apollonian circles).  After
-- the field diagrams and bipolar-coordinate plates of old physics books.  Black, white and greys only.
--
--   Cassini   phi = (B/2)(log2 R1 + log2 R2) - (B/2)(36.28 + la/64) - 4 drift + camera     (bands, mod 4)
--             Grey: a four-step staircase by band (black, 1/3, 2/3, white); Ink: black / white bands.  One
--             coverage unit per pixel blends across the nearest band edge.  Fine: lines on the band edges
--   Targets   phi1 = (B/2)(log2 R1 - 18.14) - 4 drift + camera,  phi2 = (B/2)(log2 R2 - 18.14) + 3 drift + camera
--             Grey: target 1 x 1/3 + target 2 x 2/3;  Ink: XOR (the bipolar checkerboard)
--   R1, R2 = squared distances to the foci (h = half virtual px)
--
--   K1 X / K2 Y  the midpoint of the foci, +-1 screen      K3 Scale   bands per octave
--   K4 Angle     the axis, -90 .. +90 degrees, detent      K5 Drift   bipolar speed (Hypnos), still at centre
--   K6 Camera    luma added to the band phase              S7 Glide   S8 Family   S9 Tone   S10 Weight   S11 Source
--   P12 Spread   the foci separation: one point .. 536 virtual px (default) .. near the screen edges
--
-- Sections (the integer model .vellum_work/rb_cassini/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] SWITCHES                         latched at the zone start
--   [C] PER-FIELD ENGINE                 microcoded (cassini.py PROGRAM: knob holds + glides, Drift, scale,
--                                        angle, spread, DDA constants); table fills (T, E, gain); per-line DDAs
--   [D] SOURCE FIELD                     Osc lattice, 8-px means, gain table, one-poles, a 1/8-resolution store
--   [E] GEOMETRY                         R1 R2 DDAs, normalise, T RAM + interpolation, phases, AA logs
--   [F] COVERAGE AND TONE                one coverage unit, tone, output conditioning
--
-- Stage numbering: stage 1 is g_start's own clock; a register "at stage S" holds pixel n at T + S + n.  Pixels
-- 0 .. W + 2 are rendered (the beam keeps stepping), pixel i sits at x = left - 2 xs + i xs (h); the luma
-- filter's centre (stage 26) is output column i - 2.  One latency (C_LAT) in every mode.  U = V = 512 always.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cassini_pkg.all;
use work.all;

architecture cassini of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks

    -- TOML defaults: the holds and glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
    constant C_DEF_K4  : integer := 512;
    constant C_DEF_K5  : integer := 660;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps
    ---------------------------------------------------------------------------
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
    -- the Osc lattice's step per field line, (ys 121) >> 8
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(537, 10);
            when 4          => return to_unsigned(272, 10);
            when 8          => return to_unsigned(453, 10);
            when 12         => return to_unsigned(226, 10);
            when 5 | 6 | 14 => return to_unsigned(181, 10);
            when 1 | 2 | 10 => return to_unsigned(242, 10);
            when others     => return to_unsigned(121, 10);
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
    function f_il(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others             => return '0';
        end case;
    end function;
    -- 64 log2 of the pixel footprint max(xs, line height) in h, by timing ID
    function f_lpix(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(202, 8);   -- 480i: 8.875 h
            when 4          => return to_unsigned(139, 8);   -- 480p: 4.5
            when 8          => return to_unsigned(186, 8);   -- 576i: 7.5
            when 12         => return to_unsigned(128, 8);   -- 576p: 4 (xs)
            when 5 | 6 | 14 => return to_unsigned(101, 8);   -- 720p: 3
            when 1 | 2 | 10 => return to_unsigned(128, 8);   -- 1080i: 4
            when others     => return to_unsigned(64, 8);    -- 1080p: 2
        end case;
    end function;
    -- the Fine line's half width, Q8 footprints: 2 virtual px, at least 3/4 of a footprint
    function f_hf(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8      => return to_unsigned(192, 10);
            when 4          => return to_unsigned(227, 10);
            when 12         => return to_unsigned(256, 10);
            when 5 | 6 | 14 => return to_unsigned(341, 10);
            when 1 | 2 | 10 => return to_unsigned(256, 10);
            when others     => return to_unsigned(512, 10);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- leading one of an 8-bit group (0 when none)
    function f_lead8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return to_unsigned(7, 3);
        elsif v(6) = '1' then return to_unsigned(6, 3);
        elsif v(5) = '1' then return to_unsigned(5, 3);
        elsif v(4) = '1' then return to_unsigned(4, 3);
        elsif v(3) = '1' then return to_unsigned(3, 3);
        elsif v(2) = '1' then return to_unsigned(2, 3);
        elsif v(1) = '1' then return to_unsigned(1, 3);
        else return to_unsigned(0, 3); end if;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ilr, s_il : std_logic := '0';
    signal s_lpr, c_lpix : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal c_s2   : unsigned(7 downto 0) := to_unsigned(32, 8);       -- h per field line, Q4
    signal c_left : unsigned(8 downto 0) := (others => '0');           -- h left of column 0 (SD: 480)
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal c_nq   : unsigned(7 downto 0) := to_unsigned(239, 8);      -- the camera store's last column, W/8 - 1

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 7) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] switches (latched at the zone start)
    signal s_glide : std_logic := '1';
    signal s_tgt, s_ink, s_fine, s_src : std_logic := '0';

    -- [C] THE PER-FIELD ENGINE (after Hardline flute): 37-bit accumulator, a 64 x 36-bit register file in block
    -- RAM (power-up values: the knob holds and glides at the TOML defaults), one adder, a serial 18 x 18 signed
    -- multiplier and a serial right shift; micro-op = opcode (15..12) & operand (11..0)
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
    signal rf_ram : t_c_cs_rfi := C_CS_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(35 downto 0) := (others => '0');
    signal u_run : std_logic := '0';
    signal u_ph  : unsigned(1 downto 0) := "11";
    signal u_pc, u_lr : unsigned(8 downto 0) := (others => '0');
    signal u_ir  : std_logic_vector(15 downto 0) := x"F001";      -- the microprogram ROM's output register
    signal u_irf : std_logic_vector(15 downto 0) := x"F001";      -- its fabric copy (from phase 1 on)
    signal u_mul, u_shr, u_halt, u_xo, u_xst : std_logic := '0';  -- decoded at phase 1
    signal u_gate, u_subr, u_shift, u_cz : std_logic := '0';      -- the adder's controls, as registers
    signal u_acc, u_b : signed(36 downto 0) := (others => '0');
    signal u_cnt : unsigned(4 downto 0) := (others => '0');
    signal u_out : std_logic;
    signal u_tgo : std_logic := '0';                               -- the E step is out: the T fill starts
    signal w_in  : unsigned(15 downto 0);
    signal ft_adr : unsigned(9 downto 0) := (others => '0');
    signal ft_q  : std_logic_vector(15 downto 0) := (others => '0');

    -- [C] per-field constants (the engine's outputs)
    signal c_d16  : unsigned(15 downto 0) := (others => '0');
    signal c_Est  : unsigned(16 downto 0) := to_unsigned(16384, 17);
    signal c_A2   : unsigned(21 downto 0) := (others => '0');
    signal c_Iy   : unsigned(16 downto 0) := (others => '0');
    signal c_Ldx  : unsigned(21 downto 0) := (others => '0');
    signal c_dLy  : signed(24 downto 0) := (others => '0');
    signal c_D0   : signed(18 downto 0) := (others => '0');
    signal c_I    : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal c_KB   : signed(11 downto 0) := (others => '0');
    signal c_gst  : unsigned(14 downto 0) := (others => '0');
    signal c_Hf   : unsigned(9 downto 0) := to_unsigned(512, 10);     -- the Fine half width, Q8 px

    -- [C] T table (two copies, one read port each): word = value (31..16) & slope (15..0)
    type t_tram is array (0 to 255) of unsigned(31 downto 0);
    signal t1_ram, t2_ram : t_tram := (others => (others => '0'));
    attribute no_rw_check of t1_ram : signal is true;
    attribute no_rw_check of t2_ram : signal is true;
    signal t1_q, t2_q, t_wd : unsigned(31 downto 0) := (others => '0');
    signal t1_a, t2_a, t_wa : unsigned(7 downto 0) := (others => '0');
    signal t_we : std_logic := '0';
    -- T fill engine (radix-4 serial multiplier, 10 clocks per entry)
    signal f_run, f_pre : std_logic := '0';
    signal f_i    : unsigned(8 downto 0) := (others => '0');
    signal f_k    : unsigned(3 downto 0) := (others => '0');
    signal f_ra   : unsigned(7 downto 0) := (others => '0');
    signal f_rq, f_rf : unsigned(15 downto 0) := (others => '0');
    signal f_src, f_s3 : unsigned(18 downto 0) := (others => '0');
    signal f_acc  : unsigned(32 downto 0) := (others => '0');
    signal f_mm   : unsigned(15 downto 0) := (others => '0');
    signal f_T, f_Tp : unsigned(17 downto 0) := (others => '0');
    -- E tables (copy 2 = copy 1 + 4 drift: the second target)
    type t_eram is array (0 to 255) of unsigned(15 downto 0);
    signal e1_ram, e2_ram : t_eram := (others => (others => '0'));
    attribute no_rw_check of e1_ram : signal is true;
    attribute no_rw_check of e2_ram : signal is true;
    signal e1_q, e2_q, e_wd1, e_wd2, e_acc : unsigned(15 downto 0) := (others => '0');
    signal e1_a, e2_a, e_wa : unsigned(7 downto 0) := (others => '0');
    signal e_we, e_run : std_logic := '0';

    -- [C] per-line DDAs (the beam loads them at g_start; they advance on the next clock)
    signal e_zy   : unsigned(34 downto 0) := (others => '0');
    signal e_dy   : signed(25 downto 0) := (others => '0');
    signal e_ly   : signed(34 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd, s_fl, s_seen : std_logic := '0';
    signal s_k8  : unsigned(2 downto 0) := (others => '0');
    signal s_acc : unsigned(10 downto 0) := (others => '0');
    signal s_avg : unsigned(7 downto 0) := (others => '0');
    signal s_p   : std_logic_vector(0 to 6) := (others => '0');       -- the 8-px mean's pipeline (one pulse per mean)
    signal s_f0  : std_logic_vector(0 to 6) := (others => '0');       -- ... the line's first mean
    type t_gt is array (0 to 255) of unsigned(15 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt_ram : signal is true;
    signal gt_q, s_tv, gt_wd : unsigned(15 downto 0) := (others => '0');
    signal gt_wa, gt_i : unsigned(7 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_acc : unsigned(22 downto 0) := (others => '0');
    signal s_h   : unsigned(19 downto 0) := (others => '0');
    signal s_cc, s_cw : unsigned(7 downto 0) := (others => '0');
    signal s_eq, lq_wd : unsigned(15 downto 0) := (others => '0');
    type t_lq is array (0 to 255) of unsigned(15 downto 0);
    signal lq_ram : t_lq := (others => (others => '0'));
    attribute no_rw_check of lq_ram : signal is true;
    signal lq_q0, lq_q1, lq_q2 : unsigned(15 downto 0) := (others => '0');
    signal lq_ra0, lq_ra1 : unsigned(7 downto 0) := (others => '0');
    signal lq_we : std_logic := '0';
    signal x_pc  : unsigned(10 downto 0) := (others => '0');
    signal x_run : std_logic := '0';
    signal x_f1, x_f2, x_f3, x_f4 : unsigned(2 downto 0) := (others => '0');
    signal x_q0, x_q1 : unsigned(15 downto 0) := (others => '0');
    signal x_dq  : signed(16 downto 0) := (others => '0');
    signal x_qb  : unsigned(15 downto 0) := (others => '0');
    signal x11_cam : unsigned(15 downto 0) := (others => '0');

    -- [E] geometry
    signal b_za   : signed(27 downto 0) := (others => '0');
    signal b_d    : signed(18 downto 0) := (others => '0');
    signal b_l    : signed(34 downto 0) := (others => '0');
    type t_u27x3 is array (0 to 2) of unsigned(26 downto 0);
    type t_u3x3 is array (0 to 2) of unsigned(2 downto 0);
    type t_u5x3 is array (0 to 2) of unsigned(4 downto 0);
    type t_u16x3 is array (0 to 2) of unsigned(15 downto 0);
    type t_u32x3 is array (0 to 2) of unsigned(31 downto 0);
    signal p3_r   : t_u27x3 := (others => (others => '0'));
    signal p4_c   : t_u32x3 := (others => (others => '0'));
    signal p4_pos : t_u3x3 := (others => (others => '0'));
    signal p4_e, p5_e : t_u5x3 := (others => (others => '0'));
    signal p5_m   : t_u16x3 := (others => (others => '0'));
    signal p6_fr1, p6_fr2, p7_fr1, p7_fr2, p8_fr1, p8_fr2 : unsigned(3 downto 0) := (others => '0');
    signal p6_l1, p6_l2, p6_lz : unsigned(10 downto 0) := (others => '0');
    signal p7_l1, p7_l2 : unsigned(10 downto 0) := (others => '0');
    signal p7_ls, p7_lz : unsigned(11 downto 0) := (others => '0');
    signal p8_lg, p8_g1, p8_g2 : signed(11 downto 0) := (others => '0');
    signal p8_v1, p8_v2, p8_e1, p8_e2, p9_v1, p9_v2, p9_e1, p9_e2, p10_e1, p10_e2 : unsigned(15 downto 0)
        := (others => '0');
    signal p8_s1, p8_s2 : unsigned(10 downto 0) := (others => '0');
    signal p9_s1, p9_s2 : unsigned(14 downto 0) := (others => '0');
    signal p10_t1, p10_t2 : unsigned(16 downto 0) := (others => '0');
    signal p11_pa, p11_pb, p12_pa, p12_pb : unsigned(15 downto 0) := (others => '0');

    -- [F] coverage and tone: two films' front ends (0 = A, 1 = B), ONE coverage unit, tone
    type t_s13x2 is array (0 to 1) of signed(12 downto 0);
    type t_u4x2 is array (0 to 1) of unsigned(3 downto 0);
    type t_u3x2 is array (0 to 1) of unsigned(2 downto 0);
    type t_u14x2 is array (0 to 1) of unsigned(13 downto 0);
    type t_s14x2 is array (0 to 1) of signed(13 downto 0);
    signal p9_lw  : t_s13x2 := (others => (others => '0'));
    type t_na is array (10 to 14) of t_u4x2;
    signal p_n    : t_na := (others => (others => (others => '0')));    -- the window's whole octaves (2 .. 13)
    type t_ha is array (10 to 14) of std_logic_vector(0 to 1);
    signal p_hf   : t_ha := (others => "00");                          -- and its half octave
    type t_fa is array (10 to 14) of t_u3x2;
    signal p_f    : t_fa := (others => (others => (others => '0')));    -- the contrast in quarters
    type t_fy is array (15 to 23) of unsigned(2 downto 0);
    signal p_fy   : t_fy := (others => (others => '0'));               -- the picture's contrast
    type t_kh is array (13 to 21) of unsigned(1 downto 0);
    signal p_kh   : t_kh := (others => (others => '0'));               -- Cassini: the band above the nearest edge
    signal p13_pp : t_u14x2 := (others => (others => '0'));
    signal p14_u  : t_s14x2 := (others => (others => '0'));
    signal p15_u  : signed(13 downto 0) := (others => '0');
    signal p15_n  : unsigned(3 downto 0) := (others => '0');
    signal p15_hf, p16_hf, p15_ov, p15_b, p16_b, p17_b : std_logic := '0';
    signal p15_sa, p15_sb, p16_sa, p16_sb, p17_sa, p17_sb : std_logic := '0';   -- the films' solid states
    signal p16_v  : signed(10 downto 0) := (others => '0');
    signal p17_v  : signed(11 downto 0) := (others => '0');
    signal p18_c, p18_a, p18_bb, p19_c, p19_a, p19_bb : unsigned(8 downto 0) := (others => '0');
    signal p19_z  : unsigned(8 downto 0) := (others => '0');
    signal p20_a, p20_b, p20_z1, p20_z2 : unsigned(15 downto 0) := (others => '0');
    signal p21_s  : signed(18 downto 0) := (others => '0');
    signal p22_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p23_d  : signed(10 downto 0) := (others => '0');
    signal s_y24, s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y26, s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            f_rq   <= C_CS_SRC(to_integer(f_ra));
            f_rf   <= f_rq;
        end if;
    end process p_rom;
    p_t1ram : process(clk)
    begin
        if rising_edge(clk) then
            t1_q <= t1_ram(to_integer(t1_a));
            if t_we = '1' then t1_ram(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_t1ram;
    p_t2ram : process(clk)
    begin
        if rising_edge(clk) then
            t2_q <= t2_ram(to_integer(t2_a));
            if t_we = '1' then t2_ram(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_t2ram;
    p_e1ram : process(clk)
    begin
        if rising_edge(clk) then
            e1_q <= e1_ram(to_integer(e1_a));
            if e_we = '1' then e1_ram(to_integer(e_wa)) <= e_wd1; end if;
        end if;
    end process p_e1ram;
    p_e2ram : process(clk)
    begin
        if rising_edge(clk) then
            e2_q <= e2_ram(to_integer(e2_a));
            if e_we = '1' then e2_ram(to_integer(e_wa)) <= e_wd2; end if;
        end if;
    end process p_e2ram;
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(s_avg));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
        end if;
    end process p_gtram;
    -- the camera store (1/8 horizontal resolution): port 0 = the one-pole's own read, ports 1, 2 = the pixel
    -- path's two neighbouring samples
    p_lqram : process(clk)
    begin
        if rising_edge(clk) then
            lq_q0 <= lq_ram(to_integer(s_cc));
            lq_q1 <= lq_ram(to_integer(lq_ra0));
            lq_q2 <= lq_ram(to_integer(lq_ra1));
            if lq_we = '1' then lq_ram(to_integer(s_cw)) <= lq_wd; end if;
        end if;
    end process p_lqram;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] SWITCHES
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_ilr <= f_il(s_tid);
            s_lpr <= f_lpix(s_tid);

            -- [A] vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- [A] width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_tgt   <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_il    <= s_ilr;
                c_lpix  <= s_lpr;
                c_s2    <= resize(shift_right(s_ys, 3), 8);
                c_Hf    <= f_hf(s_tid);
                s_oys   <= f_oys(s_tid);
                if s_xsr = 4 then c_left <= to_unsigned(480, 9); else c_left <= (others => '0'); end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); c_I <= to_unsigned(32, 6);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_I <= to_unsigned(18, 6);
                    when others => s_oxs <= to_unsigned(102, 9); c_I <= to_unsigned(8, 6);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            c_nq <= resize(shift_right(s_w, 3), 8) - 1;

            -- [A] sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the
            -- core on every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 6);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] THE PER-FIELD ENGINE.  Started at every zone start from address 0 (a
    -- stalled or garbled run cannot outlive a field); three clocks per micro-op
    -- (register-file read, operand / decode / next address, execute), serial
    -- multiply (18 steps) and shift.  MUL = ACC(17:0) x operand(17:0), both
    -- signed, by add-and-shift-right with the last step subtracting.  Its
    -- outputs load the per-field constants, the per-line DDAs' line-0 values and
    -- start the table fills (T on the E step, gain on its step, E on its start).
    ---------------------------------------------------------------------------
    with u_irf(3 downto 0) select w_in <=
        resize(s_xs, 16)            when x"8",
        resize(c_s2, 16)            when x"9",
        resize(c_left, 16)          when x"A",
        unsigned(ft_q)              when x"B",
        resize(c_lpix, 16)          when x"D",
        resize(unsigned(registers_in(to_integer(unsigned(u_irf(2 downto 0))))), 16) when others;

    u_out <= '1' when u_run = '1' and u_ph = "10" and u_xo = '1' else '0';

    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_gate, v_sub, v_shift, v_last, v_cond, v_fetch : std_logic;
        variable v_bx  : signed(36 downto 0);
        variable v_t   : signed(37 downto 0);
        variable v_sum : signed(36 downto 0);
    begin
        if rising_edge(clk) then
            v_op := u_irf(15 downto 12);
            v_arg := u_irf(11 downto 0);
            v_fetch := '0';
            ft_q <= C_CS_FT(to_integer(ft_adr));
            if u_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(u_ir(5 downto 0))));
            end if;
            if u_run = '1' and u_ph = "10" and u_xst = '1' then
                rf_ram(to_integer(unsigned(u_irf(5 downto 0)))) <= std_logic_vector(u_acc(35 downto 0));
            end if;

            if s_zgo = '1' then
                u_run <= '1'; u_pc <= (others => '0'); u_ph <= "11";
            elsif u_run = '1' then
                case u_ph is
                    when "11" =>                          -- fetch (first micro-op)
                        v_fetch := '1';
                        u_ph <= "00";
                    when "00" =>                          -- register-file read
                        u_irf <= u_ir;
                        u_ph <= "01";
                    when "01" =>                          -- operand, accumulator clear, next address
                        case v_op is
                            when OP_LD | OP_ADD | OP_SUB => u_b <= resize(signed(rf_q), 37);
                            when OP_LDI | OP_ADDI | OP_SUBI => u_b <= resize(signed(v_arg), 37);
                            when OP_IN   => u_b <= signed(resize(w_in, 37));
                            when OP_MUL  => u_b <= shift_left(resize(signed(rf_q(17 downto 0)), 37), 18);
                            when OP_MULI => u_b <= shift_left(resize(signed(v_arg), 37), 18);
                            when others  => null;
                        end case;
                        if v_op = OP_LD or v_op = OP_LDI or v_op = OP_IN then
                            u_acc <= (others => '0');
                        elsif v_op = OP_MUL or v_op = OP_MULI then
                            u_acc(36 downto 18) <= (others => '0');
                        end if;
                        if v_op = OP_MUL or v_op = OP_MULI then u_cnt <= to_unsigned(17, 5);
                        elsif v_op = OP_SHR then u_cnt <= unsigned(v_arg(4 downto 0)) - 1; end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := u_acc(36);
                            when "001"  => v_cond := s_glide;
                            when "010"  => v_cond := s_tgt;
                            when others => v_cond := s_src;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then u_pc <= u_pc + 2;
                        elsif v_op = OP_JMP then u_pc <= unsigned(v_arg(8 downto 0));
                        elsif v_op = OP_CALL then u_pc <= unsigned(v_arg(8 downto 0)); u_lr <= u_pc + 1;
                        elsif v_op = OP_SPC and v_arg(0) = '0' then u_pc <= u_lr;
                        else u_pc <= u_pc + 1; end if;
                        u_mul <= '0'; u_shr <= '0'; u_halt <= '0'; u_xo <= '0'; u_xst <= '0';
                        u_gate <= '0'; u_subr <= '0'; u_shift <= '0'; u_cz <= '0';
                        case v_op is
                            when OP_LD | OP_ADD | OP_LDI | OP_ADDI | OP_IN => u_gate <= '1';
                            when OP_SUB | OP_SUBI => u_gate <= '1'; u_subr <= '1';
                            when OP_MUL | OP_MULI => u_mul <= '1'; u_shift <= '1'; u_gate <= u_acc(0);
                            when OP_SHR  =>
                                u_shr <= '1'; u_shift <= '1';
                                if v_arg(4 downto 0) = "00001" then u_cz <= '1'; end if;
                            when OP_ST   => u_xst <= '1';
                            when OP_OUT  => u_xo <= '1';
                            when OP_SPC  => u_halt <= v_arg(0);
                            when others  => null;
                        end case;
                        u_ph <= "10";
                    when others =>                        -- execute (the adder sees registers only)
                        v_gate := u_gate; v_sub := u_subr; v_shift := u_shift; v_last := '1';
                        if v_gate = '0' then v_bx := (others => '0');
                        elsif v_sub = '1' then v_bx := not u_b;
                        else v_bx := u_b; end if;
                        v_t := (u_acc & '1') + (v_bx & (v_sub and v_gate));
                        v_sum := v_t(37 downto 1);
                        if u_mul = '1' or u_shr = '1' then
                            if u_cz = '0' then v_last := '0'; end if;
                            u_cnt <= u_cnt - 1;
                            if u_cnt = 1 then u_cz <= '1'; u_subr <= u_mul; end if;   -- the next step is the last
                            if u_mul = '1' then u_gate <= v_sum(1); end if;           -- the next multiplier bit
                        end if;
                        if u_halt = '1' then u_run <= '0'; end if;
                        if v_shift = '1' then u_acc <= v_sum(36) & v_sum(36 downto 1);
                        else u_acc <= v_sum; end if;
                        if v_last = '1' then
                            v_fetch := '1';
                            u_ph <= "00";
                        end if;
                end case;
            end if;
            if v_fetch = '1' then
                u_ir <= C_CS_UC(to_integer(u_pc));
            end if;

            -- the engine's output registers (the zone has no active lines: the per-line DDAs advance only
            -- at line starts)
            u_tgo <= '0';
            if u_out = '1' then
                case to_integer(unsigned(u_irf(3 downto 0))) is
                    when 0  => ft_adr <= unsigned(u_acc(9 downto 0));
                    when 1  => c_Est  <= unsigned(u_acc(16 downto 0)); u_tgo <= '1';
                    when 2  => c_KB   <= u_acc(11 downto 0);
                    when 4  => c_A2   <= unsigned(u_acc(21 downto 0));
                    when 7  => c_Iy   <= unsigned(u_acc(16 downto 0));
                    when 8  => c_D0   <= u_acc(18 downto 0);
                    when 9  => c_Ldx  <= unsigned(u_acc(21 downto 0));
                    when 11 => c_dLy  <= u_acc(24 downto 0);
                    when 13 => c_d16  <= unsigned(u_acc(15 downto 0));
                    when others => null;          -- 3 gain step, 5 6 10 the DDAs, 12 the E start: below
                end case;
            end if;

            -- [C] per-line DDAs (exact): the beam takes the line's start values at g_start, they advance on the
            -- next clock.  Zy = (Zx0 + A2) 256 + dy^2 (Q8 h^2), Ly = Lx0 + 2 a sin dy (Q8)
            if u_out = '1' and u_irf(3 downto 0) = x"5" then
                e_zy <= unsigned(u_acc(34 downto 0));
            elsif s_st(0) = '1' then
                e_zy <= unsigned(signed(e_zy) + resize(e_dy, 35));
            end if;
            if u_out = '1' and u_irf(3 downto 0) = x"6" then
                e_dy <= u_acc(25 downto 0);
            elsif s_st(0) = '1' then
                e_dy <= e_dy + signed(resize(c_Iy, 26));
            end if;
            if u_out = '1' and u_irf(3 downto 0) = x"A" then
                e_ly <= u_acc(34 downto 0);
            elsif s_st(0) = '1' then
                e_ly <= e_ly + resize(c_dLy, 35);
            end if;

            -- [C] E tables fill (64 clocks from the E start): E(n) = E0 + n (B/2) - 4 drift; copy 2 + 4 drift
            -- (Targets)
            e_we <= '0';
            if u_out = '1' and u_irf(3 downto 0) = x"C" then
                e_acc <= unsigned(u_acc(15 downto 0));
                e_run <= '1'; e_wa <= (others => '0');
            elsif e_run = '1' then
                e_we <= '1'; e_wd1 <= e_acc;
                if s_tgt = '1' then e_wd2 <= e_acc + (c_d16(13 downto 0) & "00"); else e_wd2 <= e_acc; end if;
                e_acc <= e_acc + c_Est(15 downto 0);
                if e_we = '1' then e_wa <= e_wa + 1; end if;
                if e_wa = 63 and e_we = '1' then e_run <= '0'; e_we <= '0'; end if;
            end if;

            -- [D] gain table fill (256 clocks from its step): T(l) = (l gstep) >> 7
            gt_we <= '0';
            if u_out = '1' and u_irf(3 downto 0) = x"3" then
                c_gst <= unsigned(u_acc(14 downto 0));
                gt_run <= '1'; gt_i <= (others => '0'); gt_acc <= (others => '0');
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_i; gt_wd <= resize(shift_right(gt_acc, 7), 16);
                gt_acc <= gt_acc + resize(c_gst, 23); gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [C] T TABLE FILL: entry i = (src(i) x M) >> 15, written as (T(i), T(i+1) - T(i)); src(256) = 65536.
    -- Radix-4 serial multiplier, MSB first, 10 clocks per entry; zc 260 .. about 2840.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_p : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            t_we <= '0';
            if u_tgo = '1' then
                f_pre <= '1'; f_run <= '0'; f_k <= (others => '0'); f_i <= (others => '0');
                f_ra <= (others => '0');
            elsif f_pre = '1' then                 -- wait for the first source word
                f_k <= f_k + 1;
                if f_k = 3 then
                    f_pre <= '0'; f_run <= '1'; f_k <= (others => '0');
                    f_src <= resize(f_rf, 19);
                end if;
            elsif f_run = '1' then
                f_k <= f_k + 1;
                case to_integer(f_k) is
                    when 0 =>
                        f_s3 <= f_src + shift_left(f_src, 1);
                        f_acc <= (others => '0');
                        f_mm <= c_Est(16 downto 1);
                        f_ra <= f_i(7 downto 0) + 1;                  -- prefetch the next source word
                        if f_i > 1 then                               -- f_T = T(i - 1), f_Tp = T(i - 2): entry i - 2
                            t_we <= '1'; t_wa <= resize(f_i - 2, 8);
                            t_wd <= f_Tp(15 downto 0) & resize(f_T - f_Tp, 16);
                        end if;
                        f_Tp <= f_T;
                        if f_i = 257 then f_run <= '0'; end if;
                    when 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 =>
                        case to_integer(f_mm(15 downto 14)) is
                            when 0      => v_p := (others => '0');
                            when 1      => v_p := f_src;
                            when 2      => v_p := shift_left(f_src, 1);
                            when others => v_p := f_s3;
                        end case;
                        f_acc <= shift_left(f_acc, 2) + resize(v_p, 33);
                        f_mm <= f_mm(13 downto 0) & "00";
                    when others =>                                    -- 9: the entry's value; the next source word
                        f_T <= f_acc(32 downto 15);
                        if f_i = 255 then f_src <= to_unsigned(65536, 19);
                        else f_src <= resize(f_rf, 19); end if;
                        f_i <= f_i + 1;
                        f_k <= (others => '0');
                end case;
            end if;
        end if;
    end process p_fill;

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
    -- [E] GEOMETRY: beam DDAs (stage 2), R1 R2 Zc (3), normalise (4 .. 5), T RAM,
    -- E RAMs and AA logs (6 .. 8), the widths (9), interpolation (9 .. 10),
    -- phases (11 .. 12)
    ---------------------------------------------------------------------------
    p_geo : process(clk)
        variable v_l   : signed(26 downto 0);
        variable v_r   : signed(28 downto 0);
        variable v_32  : unsigned(31 downto 0);
        variable v_g   : unsigned(1 downto 0);
        variable v_sh  : unsigned(2 downto 0);
        variable v_lg  : signed(13 downto 0);
        variable v_sum : unsigned(16 downto 0);
        variable v_nz  : std_logic_vector(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: beam
            if g_start = '1' then
                b_za <= signed(resize(e_zy(34 downto 8), 28)); b_d <= c_D0; b_l <= e_ly;
            else
                b_za <= b_za + resize(b_d, 28);
                b_d  <= b_d + signed(resize(c_I, 19));
                b_l  <= b_l + signed(resize(c_Ldx, 35));
            end if;
            -- 3: R1 = ZA - Lx, R2 = ZA + Lx (negative -> 1), Zc = |ZA - A2| = |z|^2 (one's complement)
            v_l := resize(shift_right(b_l, 8), 27);
            v_r := resize(b_za, 29) - resize(v_l, 29);
            if v_r(28) = '1' then p3_r(0) <= to_unsigned(1, 27); else p3_r(0) <= unsigned(v_r(26 downto 0)); end if;
            v_r := resize(b_za, 29) + resize(v_l, 29);
            if v_r(28) = '1' then p3_r(1) <= to_unsigned(1, 27); else p3_r(1) <= unsigned(v_r(26 downto 0)); end if;
            v_r := resize(b_za, 29) - signed(resize(c_A2, 29));
            if v_r(28) = '1' then p3_r(2) <= not unsigned(v_r(26 downto 0)); else p3_r(2) <= unsigned(v_r(26 downto 0)); end if;
            -- 4: leading byte and the coarse shift; 5: the leading one at bit 31, the 16 bits below it
            for j in 0 to 2 loop
                v_32 := "00000" & p3_r(j);
                for g in 0 to 3 loop
                    if v_32(8 * g + 7 downto 8 * g) /= 0 then v_nz(g) := '1'; else v_nz(g) := '0'; end if;
                end loop;
                if v_nz(3) = '1' then v_g := "11";
                elsif v_nz(2) = '1' then v_g := "10";
                elsif v_nz(1) = '1' then v_g := "01";
                else v_g := "00"; end if;
                p4_pos(j) <= f_lead8(v_32(8 * to_integer(v_g) + 7 downto 8 * to_integer(v_g)));
                p4_e(j) <= v_g & f_lead8(v_32(8 * to_integer(v_g) + 7 downto 8 * to_integer(v_g)));
                case v_g is
                    when "11"   => p4_c(j) <= v_32;
                    when "10"   => p4_c(j) <= v_32(23 downto 0) & x"00";
                    when "01"   => p4_c(j) <= v_32(15 downto 0) & x"0000";
                    when others => p4_c(j) <= v_32(7 downto 0) & x"000000";
                end case;
                v_sh := 7 - p4_pos(j);
                v_32 := shift_left(p4_c(j), to_integer(v_sh));
                p5_m(j) <= v_32(30 downto 15);
                p5_e(j) <= p4_e(j);
            end loop;
            -- 6: T RAM / E RAM addresses, the interpolation fraction (4 bits), the AA logs (Q6)
            t1_a <= p5_m(0)(15 downto 8); t2_a <= p5_m(1)(15 downto 8);
            p6_fr1 <= p5_m(0)(7 downto 4); p6_fr2 <= p5_m(1)(7 downto 4);
            if s_tgt = '1' then e1_a <= resize(p5_e(0), 8); else e1_a <= resize(p5_e(0), 8) + resize(p5_e(1), 8); end if;
            e2_a <= resize(p5_e(1), 8);
            p6_l1 <= p5_e(0) & C_CS_LM(to_integer(p5_m(0)(15 downto 10)));
            p6_l2 <= p5_e(1) & C_CS_LM(to_integer(p5_m(1)(15 downto 10)));
            p6_lz <= p5_e(2) & C_CS_LM(to_integer(p5_m(2)(15 downto 10)));
            -- 7: the RAMs' own registers; the AA log sum
            p7_fr1 <= p6_fr1; p7_fr2 <= p6_fr2;
            p7_ls <= resize(p6_l1, 12) + resize(p6_l2, 12);
            p7_lz <= resize(p6_lz, 12);
            p7_l1 <= p6_l1; p7_l2 <= p6_l2;
            -- 8: fabric copies; the gradients, log2 periods per pixel (Q6): Cassini KB + (lZ - lR1 - lR2) / 2,
            -- Targets KB - 64 - lR / 2
            p8_v1 <= t1_q(31 downto 16); p8_s1 <= t1_q(10 downto 0);
            p8_v2 <= t2_q(31 downto 16); p8_s2 <= t2_q(10 downto 0);
            p8_e1 <= e1_q; p8_e2 <= e2_q;
            p8_fr1 <= p7_fr1; p8_fr2 <= p7_fr2;
            v_lg := signed(resize(p7_lz, 14)) - signed(resize(p7_ls, 14));
            p8_lg <= c_KB + resize(shift_right(v_lg, 1), 12);
            p8_g1 <= c_KB - 64 - signed(resize(shift_right(p7_l1, 1), 12));
            p8_g2 <= c_KB - 64 - signed(resize(shift_right(p7_l2, 1), 12));
            -- 9: slope x fraction (4 bits); the widths (Cassini: the band edges; Targets: each film, Bold films
            -- have half the band frequency)
            p9_s1 <= resize(p8_s1 * p8_fr1, 15);
            p9_s2 <= resize(p8_s2 * p8_fr2, 15);
            p9_v1 <= p8_v1; p9_v2 <= p8_v2; p9_e1 <= p8_e1; p9_e2 <= p8_e2;
            if s_tgt = '1' then
                if s_fine = '1' then p9_lw(0) <= resize(p8_g1, 13); p9_lw(1) <= resize(p8_g2, 13);
                else p9_lw(0) <= resize(p8_g1, 13) - 64; p9_lw(1) <= resize(p8_g2, 13) - 64; end if;
            else
                p9_lw(0) <= resize(p8_lg, 13); p9_lw(1) <= resize(p8_lg, 13);
            end if;
            -- 10: the interpolated table values
            p10_t1 <= resize(p9_v1, 17) + resize(shift_right(p9_s1, 4), 17);
            p10_t2 <= resize(p9_v2, 17) + resize(shift_right(p9_s2, 4), 17);
            p10_e1 <= p9_e1; p10_e2 <= p9_e2;
            -- 11: the phases (mod 4 bands): Cassini E(e1 + e2) + T1 + T2; Targets E1(e1) + T1, E2(e2) + T2
            if s_tgt = '1' then
                p11_pa <= p10_e1 + p10_t1(15 downto 0);
                p11_pb <= p10_e2 + p10_t2(15 downto 0);
            else
                v_sum := resize(p10_e1, 17) + p10_t1 + p10_t2;
                p11_pa <= v_sum(15 downto 0);
                p11_pb <= v_sum(15 downto 0);
            end if;
            -- 12: + camera
            p12_pa <= p11_pa + x11_cam;
            p12_pb <= p11_pb + x11_cam;
        end if;
    end process p_geo;

    ---------------------------------------------------------------------------
    -- [F] COVERAGE AND TONE (stages 10 .. 28): ONE coverage unit, 1/2 + d / w
    -- (d = the distance to the edge, w = the width rounded up to a half
    -- octave, at most 1/4 period).  Cassini: the nearest band edge, the tone
    -- stepping across it (grey staircase, ink, or the line on it); Targets:
    -- the film whose edge is within reach, the other film solid.  Past 1/4
    -- period per pixel the picture fades to mid grey in four steps.  Tone,
    -- [1 2 1], 13/16, gate
    ---------------------------------------------------------------------------
    p_cov : process(clk)
        variable v_nh  : signed(12 downto 0);
        variable v_f   : signed(12 downto 0);
        variable v_ph  : unsigned(15 downto 0);
        variable v_a   : unsigned(12 downto 0);
        variable v_ov  : std_logic_vector(0 to 1);
        variable v_w   : signed(27 downto 0);
        variable v_c   : signed(12 downto 0);
        variable v_t   : unsigned(9 downto 0);
        variable v_s   : signed(18 downto 0);
        variable v_zz  : signed(18 downto 0);
        variable v_dy  : signed(10 downto 0);
        variable v_d11 : signed(10 downto 0);
        variable v_d12 : signed(11 downto 0);
        variable v_dly : unsigned(9 downto 0);
        variable v_y   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            for j in 0 to 1 loop
                -- 10: the window (half octaves, 4 .. 27: at most 1/4 period per pixel) and the contrast
                -- (quarters: full below 1/4 period per pixel, fading to flat at 1)
                v_nh := shift_right(-p9_lw(j), 5);
                if v_nh < 4 then v_nh := to_signed(4, 13); elsif v_nh > 27 then v_nh := to_signed(27, 13); end if;
                p_n(10)(j) <= unsigned(v_nh(4 downto 1)); p_hf(10)(j) <= v_nh(0);
                v_f := shift_right(-p9_lw(j) - 32, 5);
                if v_f < 0 then p_f(10)(j) <= (others => '0');
                elsif v_f > 4 then p_f(10)(j) <= to_unsigned(4, 3);
                else p_f(10)(j) <= unsigned(v_f(2 downto 0)); end if;
                -- 13: Cassini: frac phi; Targets: the film's phase moved so its band (Bold: the upper half of the
                -- period of phi / 2) or line (Fine: on the integers) is centred on 1/2
                if j = 0 then v_ph := p12_pa; else v_ph := p12_pb; end if;
                if s_tgt = '0' then
                    p13_pp(j) <= v_ph(13 downto 0);
                elsif s_fine = '1' then
                    p13_pp(j) <= v_ph(13 downto 0) + 8192;
                else
                    p13_pp(j) <= v_ph(14 downto 1) + 12288;
                end if;
                -- 14: the unit's distance (Q14 periods).  Bold: signed (Cassini: to the nearest integer, positive
                -- above it; Targets: into the band); Fine: to the line (one's complement in the upper half)
                if s_tgt = '0' and s_fine = '0' then
                    p14_u(j) <= signed(p13_pp(j));
                else
                    if s_tgt = '0' then
                        if p13_pp(j)(13) = '1' then v_a := not p13_pp(j)(12 downto 0); else v_a := p13_pp(j)(12 downto 0); end if;
                    else
                        if p13_pp(j)(13) = '1' then v_a := p13_pp(j)(12 downto 0); else v_a := not p13_pp(j)(12 downto 0); end if;
                    end if;
                    if s_fine = '1' then p14_u(j) <= signed(resize(v_a, 14));
                    else p14_u(j) <= 4096 - signed(resize(v_a, 14)); end if;
                end if;
            end loop;
            for k in 11 to 14 loop p_n(k) <= p_n(k - 1); p_hf(k) <= p_hf(k - 1); p_f(k) <= p_f(k - 1); end loop;
            -- Cassini: the band above the nearest edge, round(phi) mod 4
            p_kh(13) <= p12_pa(15 downto 14) + ("0" & p12_pa(13));
            for k in 14 to 21 loop p_kh(k) <= p_kh(k - 1); end loop;
            -- 15: each film's edge outside its ramp (|u << n| past 2^16)?  The unit takes film A unless only
            -- film B's edge is within reach; the other film counts as solid (Fine: off the line)
            for j in 0 to 1 loop
                v_ov(j) := '0';
                for i in 0 to 12 loop
                    if i + to_integer(p_n(14)(j)) >= 16 and (p14_u(j)(i) xor p14_u(j)(13)) = '1' then v_ov(j) := '1'; end if;
                end loop;
            end loop;
            if s_tgt = '1' and v_ov(0) = '1' and v_ov(1) = '0' then
                p15_b <= '1'; p15_u <= p14_u(1); p15_n <= p_n(14)(1); p15_hf <= p_hf(14)(1); p15_ov <= v_ov(1);
            else
                p15_b <= '0'; p15_u <= p14_u(0); p15_n <= p_n(14)(0); p15_hf <= p_hf(14)(0); p15_ov <= v_ov(0);
            end if;
            p15_sa <= not s_fine and not p14_u(0)(13);
            p15_sb <= not s_fine and not p14_u(1)(13);
            if s_tgt = '1' and p_f(14)(1) < p_f(14)(0) then p_fy(15) <= p_f(14)(1); else p_fy(15) <= p_f(14)(0); end if;
            for k in 16 to 23 loop p_fy(k) <= p_fy(k - 1); end loop;
            -- 16: v = (u << n) >> 6 (Q8 px), +-1024 outside the ramp
            v_w := shift_left(resize(p15_u, 28), to_integer(p15_n));
            if p15_ov = '1' then
                if p15_u(13) = '1' then p16_v <= to_signed(-1024, 11); else p16_v <= to_signed(1023, 11); end if;
            else
                p16_v <= v_w(16 downto 6);
            end if;
            p16_hf <= p15_hf; p16_b <= p15_b; p16_sa <= p15_sa; p16_sb <= p15_sb;
            -- 17: x 1.5 for a half octave
            if p16_hf = '1' then p17_v <= resize(p16_v, 12) + resize(shift_right(p16_v, 1), 12);
            else p17_v <= resize(p16_v, 12); end if;
            p17_b <= p16_b; p17_sa <= p16_sa; p17_sb <= p16_sb;
            -- 18: the coverage, 0 .. 256: Bold 128 + v, Fine 128 + half width - v; the two films' coverages
            if s_fine = '1' then v_c := 128 + signed(resize(c_Hf, 13)) - resize(p17_v, 13);
            else v_c := 128 + resize(p17_v, 13); end if;
            if v_c < 0 then v_c := (others => '0'); elsif v_c > 256 then v_c := to_signed(256, 13); end if;
            p18_c <= unsigned(v_c(8 downto 0));
            if p17_b = '1' then
                if p17_sa = '1' then p18_a <= to_unsigned(256, 9); else p18_a <= (others => '0'); end if;
                p18_bb <= unsigned(v_c(8 downto 0));
            else
                p18_a <= unsigned(v_c(8 downto 0));
                if p17_sb = '1' then p18_bb <= to_unsigned(256, 9); else p18_bb <= (others => '0'); end if;
            end if;
            -- 19: Targets Ink: the XOR, a triangle of the summed coverage
            v_t := resize(p18_a, 10) + resize(p18_bb, 10);
            if s_tgt = '1' then
                if v_t > 256 then p19_z <= resize(512 - v_t, 9); else p19_z <= v_t(8 downto 0); end if;
            else
                p19_z <= p18_c;
            end if;
            p19_c <= p18_c; p19_a <= p18_a; p19_bb <= p18_bb;
            -- 20: the products (all / 4): 73 x (Cassini cov / Targets cA), 73 x cB, 7 z and 5 z (876 z / 4 =
            -- 32 (7 z) - 5 z at 21, z = the Ink or Fine coverage)
            if s_tgt = '1' then
                p20_a <= shift_left(resize(p19_a, 16), 6) + shift_left(resize(p19_a, 16), 3) + resize(p19_a, 16);
            else
                p20_a <= shift_left(resize(p19_c, 16), 6) + shift_left(resize(p19_c, 16), 3) + resize(p19_c, 16);
            end if;
            p20_b <= shift_left(resize(p19_bb, 16), 6) + shift_left(resize(p19_bb, 16), 3) + resize(p19_bb, 16);
            p20_z1 <= shift_left(resize(p19_z, 16), 3) - resize(p19_z, 16);
            p20_z2 <= shift_left(resize(p19_z, 16), 2) + resize(p19_z, 16);
            -- 21: the tone x 64.  Cassini Bold: T(lo) 64 + (T(hi) - T(lo)) cov / 4 (Grey T = 0, 292, 584, 876;
            -- Ink 0, 876, 0, 876); Fine: the line (Grey: white on even integers, dark grey on odd); Targets:
            -- Grey (292 cA + 584 cB) / 4, Ink 876 xor / 4
            v_zz := shift_left(resize(signed('0' & p20_z1), 19), 5) - resize(signed('0' & p20_z2), 19);
            if s_tgt = '1' then
                if s_ink = '1' then v_s := v_zz;
                else v_s := resize(signed('0' & p20_a), 19) + shift_left(resize(signed('0' & p20_b), 19), 1); end if;
            elsif s_fine = '1' then
                if s_ink = '1' or p_kh(20)(0) = '0' then v_s := v_zz;
                else v_s := resize(signed('0' & p20_a), 19); end if;
            elsif s_ink = '1' then
                if p_kh(20)(0) = '1' then v_s := v_zz;
                else v_s := to_signed(56064, 19) - v_zz; end if;
            else
                case to_integer(p_kh(20)) is
                    when 0      => v_s := to_signed(56064, 19) - v_zz;
                    when 1      => v_s := resize(signed('0' & p20_a), 19);
                    when 2      => v_s := to_signed(18688, 19) + resize(signed('0' & p20_a), 19);
                    when others => v_s := to_signed(37376, 19) + resize(signed('0' & p20_a), 19);
                end case;
            end if;
            p21_s <= v_s;
            -- 22: y; 23 .. 24: past 1/4 period per pixel the picture fades to flat mid grey, 502 + (y - 502) f / 4
            p22_y <= resize(unsigned(shift_right(p21_s, 6)) + 64, 10);
            p23_d <= signed(resize(p22_y, 11)) - 502;
            case to_integer(p_fy(23)) is
                when 0      => v_dy := (others => '0');
                when 1      => v_dy := shift_right(p23_d, 2);
                when 2      => v_dy := shift_right(p23_d, 1);
                when 3      => v_d12 := resize(p23_d, 12) + shift_left(resize(p23_d, 12), 1);   -- 3 d: 12 bits
                               v_d12 := shift_right(v_d12, 2);
                               v_dy := v_d12(10 downto 0);
                when others => v_dy := p23_d;
            end case;
            v_d11 := v_dy + 502;
            s_y24 <= unsigned(v_d11(9 downto 0));                 -- (a slice: resize would keep the sign bit)
            -- 25 .. 28: luma [1 2 1] / 4 (centre at 26), 13/16 toward black, blanking gate
            s_yd1 <= s_y24; s_yd2 <= s_yd1;
            s_ya <= resize(s_y24, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y26 <= v_y(11 downto 2);
            if s_y26 > 64 then v_dly := s_y26 - 64; else v_dly := (others => '0'); end if;
            s_y27 <= s_y26 - shift_right(v_dly, 3) - shift_right(v_dly, 4);
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y28 <= s_y27;
            else
                s_y28 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_cov;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 8-pixel means -> gain table
    -- (K6, Q12 bands) -> horizontal one-pole (1/4 per mean, reloaded from the
    -- line's first mean) -> vertical one-pole (1/32 per line; 1/64 at 1080p,
    -- 1/16 at 480i / 576i) in a store at 1/8 horizontal resolution.  The pixel
    -- path reads two neighbouring samples behind the write, interpolates, and
    -- adds the result to the band phase(s) at stage 12.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l  : unsigned(7 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_a  : unsigned(10 downto 0);
        variable v_d  : signed(20 downto 0);
        variable v_h  : signed(20 downto 0);
        variable v_q  : signed(16 downto 0);
        variable v_e  : signed(16 downto 0);
        variable v_t  : signed(19 downto 0);
        variable v_u  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then   -- both phases restart from this field's drift
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
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

            -- 8-pixel means: one pulse (s_p(0)) per 8 samples, s_f0 marks the line's first
            s_p(0) <= '0'; s_f0(0) <= '0';
            if s_sv = '1' then
                if s_svd = '0' then s_k8 <= to_unsigned(1, 3); v_a := resize(s_sl, 11);
                else s_k8 <= s_k8 + 1; v_a := s_acc + resize(s_sl, 11); end if;
                if s_svd = '1' and s_k8 = 7 then
                    s_avg <= v_a(10 downto 3); s_p(0) <= '1'; s_acc <= (others => '0');
                else
                    s_acc <= v_a;
                end if;
            end if;
            if s_sv = '1' and s_svd = '0' then s_seen <= '1'; elsif s_p(0) = '1' then s_seen <= '0'; end if;
            if s_sv = '1' and s_svd = '1' and s_k8 = 7 then s_f0(0) <= s_seen; end if;
            s_p(1 to 6) <= s_p(0 to 5); s_f0(1 to 6) <= s_f0(0 to 5);
            -- (1) gain table register; (2) its fabric copy
            s_tv <= gt_q;
            -- (2): the horizontal one-pole (Q4) and the store read of this column
            if s_p(2) = '1' then
                v_d := signed(resize(s_tv & "0000", 21)) - signed(resize(s_h, 21));
                if s_f0(2) = '1' then s_h <= resize(s_tv & "0000", 20);
                else v_h := signed(resize(s_h, 21)) + shift_right(v_d, 2); s_h <= unsigned(v_h(19 downto 0)); end if;
            end if;
            -- the store column (the read at (2) uses s_cc, the write at (5) s_cw)
            if s_sv = '1' and s_svd = '0' then s_cc <= (others => '0');
            elsif s_p(3) = '1' then s_cc <= s_cc + 1; end if;
            if s_p(3) = '1' then s_cw <= s_cc; end if;
            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1';
            elsif s_sv = '0' and s_svd = '1' then
                s_fl <= '0';
            end if;
            -- (4): q (the read at (2), registered at (3)); (5): q + (h - q) >> 6 (5 at 720p / 1080i, 4 at 480i / 576i)
            if s_p(4) = '1' then s_eq <= lq_q0; end if;
            lq_we <= '0';
            if s_p(5) = '1' then
                v_e := signed(resize(s_h(19 downto 4), 17)) - signed(resize(s_eq, 17));
                if s_fl = '1' then v_q := signed(resize(s_h(19 downto 4), 17));
                elsif s_xs = 2 and s_il = '0' then v_q := signed(resize(s_eq, 17)) + shift_right(v_e, 6);
                elsif (s_xs = 2) = (s_il = '1') then v_q := signed(resize(s_eq, 17)) + shift_right(v_e, 5);
                else v_q := signed(resize(s_eq, 17)) + shift_right(v_e, 4); end if;
                lq_wd <= unsigned(v_q(15 downto 0)); lq_we <= '1';
            end if;

            -- pixel path: coarse position (from stage 3), the two neighbouring samples (clamped at the last
            -- column), their interpolation by the 1/8 fraction, the camera phase (Q14 bands, mod 4) at stage 11
            if g_start = '1' then x_run <= '0';
            elsif s_st(1) = '1' then x_run <= '1'; end if;
            if x_run = '0' then x_pc <= (others => '0');
            elsif x_pc /= 2047 then x_pc <= x_pc + 1; end if;
            -- (the store is read 34 pixels behind the beam: always after this line's write of both samples)
            v_u := signed('0' & x_pc) - 34;
            if v_u < 0 then
                lq_ra0 <= (others => '0'); lq_ra1 <= (others => '0'); x_f1 <= (others => '0');
            elsif unsigned(v_u(10 downto 3)) >= c_nq then
                lq_ra0 <= c_nq; lq_ra1 <= c_nq; x_f1 <= (others => '0');
            else
                lq_ra0 <= unsigned(v_u(10 downto 3)); lq_ra1 <= unsigned(v_u(10 downto 3)) + 1;
                x_f1 <= unsigned(v_u(2 downto 0));
            end if;
            x_f2 <= x_f1;
            x_q0 <= lq_q1; x_q1 <= lq_q2; x_f3 <= x_f2;
            x_dq <= signed(resize(x_q1, 17)) - signed(resize(x_q0, 17));
            x_qb <= x_q0; x_f4 <= x_f3;
            v_t := (others => '0');
            if x_f4(2) = '1' then v_t := v_t + shift_left(resize(x_dq, 20), 2); end if;
            if x_f4(1) = '1' then v_t := v_t + shift_left(resize(x_dq, 20), 1); end if;
            if x_f4(0) = '1' then v_t := v_t + resize(x_dq, 20); end if;
            v_t := signed(resize(x_qb, 20)) + shift_right(v_t, 3);
            x11_cam <= unsigned(v_t(13 downto 0)) & "00";
        end if;
    end process p_src;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cassini;
