-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Trine v0.1.0 -- Vellum.  Islamic triangular tilings, Vasarely's "Tridim", the Sierpinski triangle of a
-- mathematics plate.  Luma only (U = V = 512 on every pixel).
--
-- Form = Nest: the screen tiled by a triangular lattice; every cell a nest of concentric triangles (bands of
--   3 x the smallest barycentric coordinate), up- and down-cells offset in tone so their nests interlock.
--   P12 turns the nests over: each ring is the cell's triangle cut by the turned (inverted) triangle scaled
--   by u = 2^(1 - 2 P12), so the rings go triangle -> hexagon -> turned triangle, and the corners they leave
--   are ground.
--     lambda = the cell's barycentrics from the lattice front (the Gasket's, one level);
--     N = min(3 d min lambda, d (1 + 1/u) - 3 d max lambda / u)
--     phase = N + offset + camera - drift.  The cell across the nearest edge shows its outermost band there.
-- Form = Gasket: the Sierpinski triangle (lattice coordinates a, b; the first carry of a + b marks the hole
--   level), each hole truncated the same way by P12 (triangle -> hexagon -> turned triangle), the camera
--   shrinking the holes (a halftone), an endless zoom about the apex (x2 per octave, the next level opening).
--
--   K1  X        lattice pan / the apex, +-700 virtual px                (glided)
--   K2  Y        +-540 virtual px                                        (glided)
--   K3  Scale    cell side 125 .. 997 virtual px / gasket height 288 .. 2306, exponential (glided)
--   K4  Depth    1 .. 12 bands per cell / levels drawn (stepped, hysteresis)
--   K5  Drift    bipolar speed (Hypnos), still at the centre; bands stream into the cells / the gasket zooms
--   K6  Camera   luma added to the nest phase / shrinking the holes      (glided)
--   S7  Glide    Off / On      S8 Form   Nest / Gasket       S9 Tone   Grey / Ink
--   S10 Weight   Bold / Fine   S11 Source Video / Osc
--   P12 Turn     nests and holes: triangle -> hexagon -> turned triangle (glided)
--
-- Sections (the integer model .vellum_work/rb_trine/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width / height, generated avid
--   [B] CONTROLS                         switches, Depth hysteresis (the holds and glides run in [C])
--   [C] FIELD ENGINE                     Hardline flute's microcoded engine (trine.py): holds, glides, drift,
--                                        every per-field constant; then the gasket's level table in fabric
--   [L] LINE ENGINE                      the lattice row coordinate of the next line
--   [D] SOURCE FIELD                     Osc lattice, gain table, 32/64-px box, vertical one-pole (dazzle)
--   [U] LATTICE FRONT (both Forms)       stages U1 .. U8 (U0 = T + 8; U3 is folded into U4): a, b, the carries, the barycentric
--                                        triple, its min and max, the Nest's scaling / the Gasket's thresholds
--   [N] NEST tail                        U9 .. U15: the turned family, phase, bands, the nearest edge
--   [G] GASKET tail                      U9 .. U15: the hole + camera, the frame, the nearest edge
--   [E] COVERAGE AND BLEND               one 3-step divider (9 levels), blend, the mid-grey fade
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  Both tails hand (d, w, tone code) to the shared back end at T + 22;
-- the faded luma is at T + 35 + i for DDA pixel i, the output register at T + 41 + j for column j (the [1 2 1]
-- centred on DDA pixel j + 1).  One latency (40) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.trine_pkg.all;
use work.all;

architecture trine of program_top is
    constant C_LAT : integer := 40;   -- = processing_delay_clks


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
    -- 2^20 / ys (field lines per virtual line, Q12 of 256)
    function f_invys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(921, 13);
            when 4          => return to_unsigned(1820, 13);
            when 8          => return to_unsigned(1092, 13);
            when 12         => return to_unsigned(2185, 13);
            when 5 | 6 | 14 => return to_unsigned(2731, 13);
            when 1 | 2 | 10 => return to_unsigned(2048, 13);
            when others     => return to_unsigned(4096, 13);
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
    function f_il(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others             => return '0';
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
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
    function f_as(a, b : signed; sub : std_logic) return signed is
    begin
        return signed(f_addsub(unsigned(a), unsigned(b), sub));
    end function;
    -- the Nest's band tone (Grey: 4 steps, Ink: black / white by parity)
    function f_tone(n : unsigned(1 downto 0); ink : std_logic) return unsigned is
    begin
        if ink = '1' then
            if n(0) = '1' then return to_unsigned(940, 10); else return to_unsigned(64, 10); end if;
        end if;
        case to_integer(n) is
            when 0      => return to_unsigned(64, 10);
            when 1      => return to_unsigned(356, 10);
            when 2      => return to_unsigned(648, 10);
            when others => return to_unsigned(940, 10);
        end case;
    end function;
    -- Fine line tone of a band boundary
    function f_ltone(n : unsigned(1 downto 0); ink : std_logic) return unsigned is
    begin
        if ink = '1' then return to_unsigned(940, 10); end if;
        case to_integer(n) is
            when 0      => return to_unsigned(414, 10);
            when 1      => return to_unsigned(589, 10);
            when 2      => return to_unsigned(764, 10);
            when others => return to_unsigned(939, 10);
        end case;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width / height
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ilr, s_il : std_logic := '0';
    signal s_ivr, s_iv : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_lcnt : unsigned(10 downto 0) := (others => '0');
    signal s_h    : unsigned(10 downto 0) := to_unsigned(1080, 11);

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- sync delay: a block-RAM ring, read C_LAT - 2 behind the write (+ the read register = C_LAT - 1, + one
    -- fabric register = C_LAT)
    type t_dl is array (0 to 255) of std_logic_vector(3 downto 0);
    signal dl_ram : t_dl := (others => "0011");
    attribute no_rw_check of dl_ram : signal is true;
    signal dl_wa : unsigned(7 downto 0) := (others => '0');
    signal dl_q, dl_q2 : std_logic_vector(3 downto 0) := "0011";
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 12) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] switches, Depth hysteresis
    signal s_glide : std_logic := '1';
    signal s_gas, s_ink, s_fine, s_src : std_logic := '0';
    signal s_zn  : unsigned(3 downto 0) := to_unsigned(3, 4);          -- Depth - 1
    signal z_v12 : unsigned(13 downto 0) := to_unsigned(3576, 14);
    signal z_up, z_dn : std_logic := '0';
    signal c_lvm : std_logic_vector(12 downto 0) := (others => '0');   -- gasket levels drawn (bit L - 1)
    signal c_D3  : unsigned(5 downto 0) := to_unsigned(12, 6);

    -- [C] the per-field engine (Hardline flute's): 37-bit accumulator, 128 x 32 register file, serial
    -- multiplier and shift, microprogram and table in block RAM
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
    signal rf_ram : t_c_tr_rfi := C_TR_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run, e_rund : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc, e_lr : unsigned(9 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";
    signal x_mul, x_shr, x_nrm, x_halt, x_out, x_st : std_logic := '0';
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal w_in  : unsigned(15 downto 0);
    signal ft_adr : unsigned(8 downto 0) := (others => '0');
    signal ft_q  : std_logic_vector(15 downto 0) := (others => '0');
    -- the engine's outputs (per-field constants)
    signal c_D3i : unsigned(14 downto 0) := (others => '0');
    signal c_cz : unsigned(20 downto 0) := to_unsigned(1048576, 21);   -- Nest: d (1 + 1/u); Gasket: 1/z
    signal c_ga, c_gb : unsigned(12 downto 0) := (others => '0');     -- the edge gradients (Nest L1; Gasket w >> gsh)
    signal c_L1ai, c_L1bi : unsigned(12 downto 0) := (others => '0');
    signal c_poup, c_podn : unsigned(13 downto 0) := (others => '0');
    signal c_du, c_dS : unsigned(15 downto 0) := (others => '0');
    signal c_u0, c_S0 : signed(25 downto 0) := (others => '0');
    signal c_gsh : unsigned(2 downto 0) := (others => '0');
    signal c_g6  : unsigned(12 downto 0) := (others => '0');            -- camera gain (K6, glided, Q3)
    type t_tb is array (0 to 255) of unsigned(15 downto 0);
    signal tb1_ram, tb2_ram, tb3_ram : t_tb := (others => (others => '0'));
    attribute no_rw_check of tb1_ram : signal is true;
    attribute no_rw_check of tb2_ram : signal is true;
    attribute no_rw_check of tb3_ram : signal is true;
    signal tb_we1, tb_we2, tb_we3 : std_logic := '0';
    signal tb_wa, tb_ra, tb_rh : unsigned(7 downto 0) := (others => '0');
    signal tb_wd, tb_q1, tb_q2, tb_q3 : unsigned(15 downto 0) := (others => '0');

    -- [L] line engine (per line: S and 3 (1/z - S))
    signal l_kick, e_kr : std_logic := '0';
    signal e_lc  : unsigned(4 downto 0) := to_unsigned(31, 5);
    signal e_S   : signed(25 downto 0) := (others => '0');
    signal e_f   : signed(26 downto 0) := (others => '0');
    signal n_F3  : signed(23 downto 0) := (others => '0');

    -- [U] the lattice front (both Forms), stages U1 .. U8 (U0 = T + 8, U1, U2, then U4 .. U8 = T + 11 .. T + 15; U3 is folded into U4)
    signal gb_u  : signed(25 downto 0) := (others => '0');
    signal gb_S  : signed(25 downto 0) := (others => '0');
    signal gb_F3 : signed(23 downto 0) := (others => '0');
    signal u1_a, u1_b, u1_S : signed(21 downto 0) := (others => '0');
    signal u2_a, u2_b, u2_S : unsigned(19 downto 0) := (others => '0');
    signal u2_po, u2_oh : std_logic_vector(12 downto 0) := (others => '0');
    signal u2_dn : std_logic := '0';
    signal u2_mab : signed(21 downto 0) := (others => '0');
    signal u4_fd : signed(23 downto 0) := (others => '0');
    signal u4_dn, u5_dn, u6_dn, u7_dn, u8_dn : std_logic := '0';
    signal u4_jf : std_logic := '0';
    signal u4_t0, u4_t1, u4_t2, u5_mn, u5_mx, u5_t2, u6_mn, u6_mx : unsigned(20 downto 0) := (others => '0');
    signal u6_kn2, u6_kx2, u7_kn2, u7_kx2, u8_kn2, u8_kx2 : std_logic := '0';
    signal u6_T1, u6_T2 : unsigned(15 downto 0) := (others => '0');
    signal u7_pm  : unsigned(20 downto 0) := (others => '0');       -- Nest: (min >> 6) 3d
    signal u7_plo, u7_phi : unsigned(17 downto 0) := (others => '0');  -- Nest: (max >> 9) x (3d/u >> 3), two halves
    signal u7_p3, u7_x3 : unsigned(22 downto 0) := (others => '0'); -- Gasket: 3 min, 3 max
    signal u7_c1, u7_c2 : signed(22 downto 0) := (others => '0');   -- Gasket: delta - T1, delta + T2
    signal u8_lmin : unsigned(18 downto 0) := (others => '0');
    signal u8_lmx  : unsigned(19 downto 0) := (others => '0');
    signal u8_m1, u8_m2 : signed(23 downto 0) := (others => '0');
    type t_u4a is array (natural range <>) of unsigned(3 downto 0);
    signal g_L   : t_u4a(4 to 9) := (others => (others => '0'));
    type t_s13a is array (natural range <>) of signed(12 downto 0);
    signal g_fd  : t_s13a(5 to 12) := (others => (others => '0'));
    signal g_jf  : std_logic_vector(5 to 11) := (others => '0');
    signal g_hole : std_logic_vector(4 to 12) := (others => '0');

    -- [N] Nest tail, U9 .. U14
    signal n9_phi : signed(19 downto 0) := (others => '0');
    signal n9_lmin : unsigned(18 downto 0) := (others => '0');
    signal n9_DL, n10_DL : unsigned(13 downto 0) := (others => '0');
    signal n9_nn, n10_nn, n11_nn, n12_nn, n13_nn : unsigned(1 downto 0) := (others => '0');
    signal n10_ls, n11_ls : unsigned(12 downto 0) := (others => '0');
    signal n9_kn2, n9_kx2, n10_k2 : std_logic := '0';
    signal n10_N : signed(19 downto 0) := (others => '0');
    signal n11_N, n12_N : signed(13 downto 0) := (others => '0');
    signal n10_inv, n11_inv, n12_inv, n13_inv : std_logic := '0';
    signal n11_phi : unsigned(13 downto 0) := (others => '0');
    signal n11_g1 : unsigned(12 downto 0) := (others => '0');
    signal n12_nb, n13_nb : unsigned(1 downto 0) := (others => '0');
    signal n12_db : unsigned(11 downto 0) := (others => '0');
    signal n12_hi, n13_hi : std_logic := '0';
    signal n12_w, n13_w : unsigned(12 downto 0) := (others => '0');
    signal n12_aN : unsigned(12 downto 0) := (others => '0');
    signal n13_hw : unsigned(10 downto 0) := (others => '0');
    signal n13_dist : signed(13 downto 0) := (others => '0');
    signal n13_sb : std_logic := '0';
    signal n14_d : signed(12 downto 0) := (others => '0');
    signal n14_w : unsigned(12 downto 0) := (others => '0');
    signal n14_code : unsigned(10 downto 0) := (others => '0');    -- f4, nn, inv, sb, hi, nb

    -- [G] Gasket tail, U9 .. U14
    signal g9_dh : signed(23 downto 0) := (others => '0');
    signal g9_j2, g10_j2, g11_j2 : std_logic := '0';
    signal g9_er, g10_er : unsigned(14 downto 0) := (others => '0');
    signal g10_we, g11_wc : unsigned(11 downto 0) := (others => '0');
    signal g10_dh, g12_dh : signed(14 downto 0) := (others => '0');
    signal g11_dh : signed(15 downto 0) := (others => '0');
    signal g12_ht, g13_ht : unsigned(9 downto 0) := (others => '0');
    signal g12_uh, g13_uh : std_logic := '0';
    signal g12_adh, g12_afd : unsigned(14 downto 0) := (others => '0');
    signal g12_wh, g12_wf, g13_wh : unsigned(12 downto 0) := (others => '0');
    signal g13_dW : signed(15 downto 0) := (others => '0');
    signal g14_d : signed(12 downto 0) := (others => '0');
    signal g14_w : unsigned(12 downto 0) := (others => '0');
    signal g14_code : unsigned(10 downto 0) := (others => '0');    -- uh, HT

    -- [D] source field (dazzle's)
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_v1, s_v2, s_v2d, s_v3, s_v4, s_ld, s_fl, s_seen, s_efl, s_ffl : std_logic := '0';
    type t_gt is array (0 to 255) of unsigned(15 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt_ram : signal is true;
    signal gt_q, s_tv, s_tvd, s_sfirst, gt_wd : unsigned(15 downto 0) := (others => '0');
    signal gt_wa, gt_i : unsigned(7 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_acc : unsigned(21 downto 0) := (others => '0');
    signal s_col, s_c2, s_bcol, s_ecol, s_fcol : unsigned(10 downto 0) := (others => '0');
    signal bx_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_f : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_b64 : std_logic := '0';
    signal s_df   : signed(16 downto 0) := (others => '0');
    signal s_bsum : unsigned(21 downto 0) := (others => '0');
    signal s_ev, s_fv : std_logic := '0';
    signal s_eb, s_eq, s_fq : unsigned(15 downto 0) := (others => '0');
    signal s_dd : signed(16 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(15 downto 0) := (others => '0');
    type t_ly is array (0 to 1023) of signed(7 downto 0);
    signal ly_ram : t_ly := (others => (others => '0'));
    attribute no_rw_check of ly_ram : signal is true;
    signal ly_q2, ly_wd, x_a0 : signed(7 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');
    signal x_q0, x_q1, x_q2 : unsigned(15 downto 0) := (others => '0');
    -- camera values of DDA pixel i, valid at T + 15 + i: Lp (bands Q12), Lx (per pixel), Ly (per line)
    signal cm_p : unsigned(14 downto 0) := (others => '0');
    signal cm_x : signed(11 downto 0) := (others => '0');
    signal cm_y : signed(9 downto 0) := (others => '0');
    signal cm_s : unsigned(12 downto 0) := (others => '0');            -- |Lx| + |Ly|, valid at T + 16 + i

    -- [E] coverage and blend
    signal bk_d : signed(12 downto 0) := (others => '0');
    signal bk_w : unsigned(12 downto 0) := (others => '0');
    type t_u10b is array (natural range <>) of unsigned(9 downto 0);
    type t_u11a is array (natural range <>) of unsigned(10 downto 0);
    signal v_code : t_u11a(21 to 28) := (others => (others => '0'));
    type t_u3a is array (natural range <>) of unsigned(2 downto 0);
    signal v_f4 : t_u3a(30 to 33) := (others => (others => '0'));
    signal x14_d : signed(12 downto 0) := (others => '0');
    signal x14_w, x15_w, x14_wh : unsigned(12 downto 0) := (others => '0');
    signal x15_e : signed(14 downto 0) := (others => '0');
    type t_dv13 is array (0 to 3) of unsigned(12 downto 0);
    signal dv_r, dv_w : t_dv13 := (others => (others => '0'));
    type t_dv3 is array (0 to 3) of unsigned(2 downto 0);
    signal dv_q : t_dv3 := (others => (others => '0'));
    signal dv_z, dv_f : std_logic_vector(0 to 3) := (others => '0');
    signal x29_cov : unsigned(3 downto 0) := (others => '0');
    signal b30_df : signed(10 downto 0) := (others => '0');
    signal b30_cov : unsigned(3 downto 0) := (others => '0');
    signal b30_far, b31_far, b32_far : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal b31_pa, b31_pb : signed(15 downto 0) := (others => '0');
    signal b32_s : signed(15 downto 0) := (others => '0');
    signal b33_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal f34_t : unsigned(11 downto 0) := (others => '0');
    signal f34_c : unsigned(11 downto 0) := (others => '0');
    signal s_y35 : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [F] output conditioning
    signal s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y37, s_y38, s_y39 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    -- gasket tables (T1, T2: read at G5 by L*; the hole tone: read at G13), written after the engine halts
    p_tb1 : process(clk)
    begin
        if rising_edge(clk) then
            tb_q1 <= tb1_ram(to_integer(tb_ra));
            if tb_we1 = '1' then tb1_ram(to_integer(tb_wa)) <= tb_wd; end if;
        end if;
    end process p_tb1;
    p_tb2 : process(clk)
    begin
        if rising_edge(clk) then
            tb_q2 <= tb2_ram(to_integer(tb_ra));
            if tb_we2 = '1' then tb2_ram(to_integer(tb_wa)) <= tb_wd; end if;
        end if;
    end process p_tb2;
    p_tb3 : process(clk)
    begin
        if rising_edge(clk) then
            tb_q3 <= tb3_ram(to_integer(tb_rh));
            if tb_we3 = '1' then tb3_ram(to_integer(tb_wa)) <= tb_wd; end if;
        end if;
    end process p_tb3;

    -- sync delay ring: written every clock, read C_LAT - 2 behind
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            dl_wa <= dl_wa + 1;
            dl_ram(to_integer(dl_wa)) <= '0' & g_avid & data_in.vsync_n & data_in.hsync_n;
            dl_q <= dl_ram(to_integer(dl_wa - (C_LAT - 2)));
            dl_q2 <= dl_q;
        end if;
    end process p_dl;

    -- gain table: written in the zone, read by the source path
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(s_sl));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
        end if;
    end process p_gtram;

    -- box delay: read 30 (or 62) samples behind the write (+ two registers = 32 or 64)
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_v2 = '1' then bx_ram(to_integer(bx_wp)) <= s_tv; end if;
        end if;
    end process p_bxram;

    -- source line store (half horizontal resolution): port 1 = the one-pole's own read, port 2 = the pixel path
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_c2(10 downto 1)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;
    p_lyram : process(clk)
    begin
        if rising_edge(clk) then
            ly_q2 <= ly_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then ly_ram(to_integer(lb_wa)) <= ly_wd; end if;
        end if;
    end process p_lyram;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] switches, Depth + the gasket table fill
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k   : unsigned(9 downto 0);
        variable v_lo, v_hi : unsigned(13 downto 0);
        variable v_r   : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_ilr <= f_il(s_tid);
            s_ivr <= f_invys(s_tid);

            -- [A] vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- [A] width (avid pixels per line, max over the field) and height (active lines per field)
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            if s_zgo = '1' then
                s_lcnt <= (others => '0');
            elsif data_in.avid = '1' and s_avq = '0' and s_lcnt /= 2047 then
                s_lcnt <= s_lcnt + 1;
            end if;

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_h <= s_lcnt;
                s_glide <= registers_in(6)(0);
                s_gas   <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_il    <= s_ilr;
                s_iv    <= s_ivr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- Depth: 12 steps with hysteresis (enter a step only 8 counts past its edge; the two ends
                -- are reached because the band lies on the entry side)
                if z_up = '1' or z_dn = '1' then s_zn <= z_v12(13 downto 10); end if;
            end if;
            v_k := unsigned(registers_in(3));
            z_v12 <= (v_k & "000") + (resize(v_k, 14) sll 2);
            v_lo := resize(s_zn & "0000000000", 14);
            v_hi := v_lo + 1120;
            if z_v12 >= v_hi then z_up <= '1'; else z_up <= '0'; end if;
            if z_v12 + 96 < v_lo then z_dn <= '1'; else z_dn <= '0'; end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is              -- the Osc lattice's line step, ys * 121 / 256
                when 0          => s_oys <= to_unsigned(537, 10);
                when 4          => s_oys <= to_unsigned(272, 10);
                when 8          => s_oys <= to_unsigned(453, 10);
                when 12         => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10);
                when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others     => s_oys <= to_unsigned(121, 10);
            end case;
            for L in 1 to 13 loop
                if L <= to_integer(s_zn) + 2 then c_lvm(L - 1) <= '1'; else c_lvm(L - 1) <= '0'; end if;
            end loop;
            c_D3 <= resize(s_zn & '0', 6) + resize(s_zn, 6) + 3;

            -- line 0 once the engine has halted
            e_rund <= e_run;
            l_kick <= e_rund and not e_run;

            -- (the syncs and the generated avid are delayed in the dl_ram ring, p_dl)
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 11);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] THE PER-FIELD ENGINE (Hardline flute's).  Started at every zone start
    -- from address 0; three clocks per micro-op (register-file read, the ROM word
    -- copied to fabric / operand, decode and next address / execute), serial
    -- multiply (18 steps) and shift.  ACC = 37 bits; MUL = ACC(17:0) x operand
    -- (17:0), both signed, by add-and-shift-right with the last step subtracting.
    -- trine.py holds the microprogram, its assembler and its emulator.
    ---------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(s_w, 16)             when x"8",
        resize(s_h, 16)             when x"9",
        resize(s_ys, 16)            when x"A",
        unsigned(ft_q)              when x"B",
        resize(e_cnt, 16)           when x"C",
        resize(s_xs, 16)            when x"D",
        resize(s_iv, 16)            when x"E",
        resize(s_zn, 16)            when x"F",
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
            ft_q <= C_TR_FT(to_integer(ft_adr));
            if e_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(e_ir(6 downto 0))));
            end if;
            if e_run = '1' and e_ph = "10" and x_st = '1' then
                rf_ram(to_integer(unsigned(e_irf(6 downto 0)))) <= std_logic_vector(e_acc(31 downto 0));
            end if;

            if s_zgo = '1' then
                e_run <= '1'; e_pc <= (others => '0'); e_ph <= "11";
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
                            when "010"  => v_cond := s_ink;
                            when "011"  => v_cond := s_src;
                            when others => v_cond := s_gas;
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
                e_ir <= C_TR_UC(to_integer(e_pc));
            end if;

            -- the engine's output registers; 25 .. 27 write the level table (T1, T2, HT) at TA
            tb_wd <= unsigned(e_acc(15 downto 0));
            if e_out = '1' and unsigned(e_irf(4 downto 0)) = 25 then tb_we1 <= '1'; else tb_we1 <= '0'; end if;
            if e_out = '1' and unsigned(e_irf(4 downto 0)) = 26 then tb_we2 <= '1'; else tb_we2 <= '0'; end if;
            if e_out = '1' and unsigned(e_irf(4 downto 0)) = 27 then tb_we3 <= '1'; else tb_we3 <= '0'; end if;
            if e_out = '1' then
                case to_integer(unsigned(e_irf(4 downto 0))) is
                    when 0  => ft_adr  <= unsigned(e_acc(8 downto 0));
                    when 1  => c_D3i   <= unsigned(e_acc(14 downto 0));
                    when 8 | 19 => c_cz <= unsigned(e_acc(20 downto 0));
                    when 9 | 20 => c_ga <= unsigned(e_acc(12 downto 0));
                    when 10 | 21 => c_gb <= unsigned(e_acc(12 downto 0));
                    when 11 => c_L1ai  <= unsigned(e_acc(12 downto 0));
                    when 12 => c_L1bi  <= unsigned(e_acc(12 downto 0));
                    when 13 => c_poup  <= unsigned(e_acc(13 downto 0));
                    when 14 => c_podn  <= unsigned(e_acc(13 downto 0));
                    when 15 => c_du    <= unsigned(e_acc(15 downto 0));
                    when 16 => c_dS    <= unsigned(e_acc(15 downto 0));
                    when 17 => c_u0    <= e_acc(25 downto 0);
                    when 18 => c_S0    <= e_acc(25 downto 0);
                    when 22 => c_gsh   <= unsigned(e_acc(2 downto 0));
                    when 23 => c_g6    <= unsigned(e_acc(12 downto 0));
                    when 24 => tb_wa   <= resize(unsigned(e_acc(3 downto 0)), 8);
                    when others => null;
                end case;
            end if;
        end if;
    end process p_eng;

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
    -- [D] SOURCE FIELD (Hardline dazzle's): luma (or the Osc lattice) -> gain
    -- table (K6, Q13 bands) -> box of 32 px (64 on the 1920-wide rasters) ->
    -- vertical one-pole in the line store at half horizontal resolution; the step
    -- per line is stored beside it.  The pixel path reads the store from T + 8:
    -- Lp / Lx / Ly of DDA pixel i are valid at T + 15 + i.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(15 downto 0);
        variable v_dl : signed(16 downto 0);
        variable v_n : signed(17 downto 0);
        variable v_qs : unsigned(16 downto 0);
        variable v_qd : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then   -- both phases restart from this field's drift
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
            s_sv <= data_in.avid;
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_tv <= gt_q;
            if s_v2 = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 (64) samples; the samples before the line start count as the first one
            if s_v2 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            elsif s_b64 = '1' then bx_wp <= (others => '0'); bx_rp <= to_unsigned(194, 8);
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(226, 8); end if;
            bx_f <= bx_q;
            if s_bcnt(6) = '0' and (s_b64 = '1' or s_bcnt(5) = '0') then v_tap := s_sfirst; else v_tap := bx_f; end if;
            s_df <= signed(resize(s_tv, 17)) - signed(resize(v_tap, 17));
            s_tvd <= s_tv; s_v3 <= s_v2; s_c2 <= s_col;
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_sfirst <= s_tv;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_v2 = '1' then
                if s_bcnt(6) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_ld = '1' then
                if s_b64 = '1' then s_bsum <= shift_left(resize(s_tvd, 22), 6);
                else s_bsum <= shift_left(resize(s_tvd, 22), 5); end if;
            elsif s_v3 = '1' then
                s_bsum <= unsigned(signed(s_bsum) + resize(s_df, 22));
            end if;
            s_v4 <= s_v3; s_bcol <= s_c2;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif s_ld = '1' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- vertical one-pole on the odd columns: q += (box - q) >> 5 (6 at 1080p, 4 at 480i / 576i)
            if s_b64 = '1' then s_eb <= s_bsum(21 downto 6); else s_eb <= s_bsum(20 downto 5); end if;
            s_eq <= lb_q1;
            s_ev <= s_v4 and s_bcol(0); s_ecol <= s_bcol; s_efl <= s_fl;
            s_dd <= signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            if s_efl = '1' then s_fq <= s_eb; else s_fq <= s_eq; end if;
            s_fv <= s_ev; s_fcol <= s_ecol; s_ffl <= s_efl;
            if s_ffl = '1' then v_dl := (others => '0');
            elsif s_b64 = '1' and s_il = '0' then v_dl := shift_right(s_dd, 6);
            elsif s_b64 = s_il then v_dl := shift_right(s_dd, 5);
            else v_dl := shift_right(s_dd, 4); end if;
            v_n := signed(resize(s_fq, 18)) + resize(v_dl, 18);
            lb_wd <= unsigned(v_n(15 downto 0));
            if v_dl(16 downto 10) /= "0000000" and v_dl(16 downto 10) /= "1111111" then
                if v_dl(16) = '1' then ly_wd <= to_signed(-128, 8); else ly_wd <= to_signed(127, 8); end if;
            else
                ly_wd <= v_dl(10 downto 3);
            end if;
            lb_we <= s_fv; lb_wa <= s_fcol(10 downto 1);

            -- gain table fill (zone, after the glides): T(i) = (i * K6) >> 6 (Osc: >> 5) by accumulation
            gt_we <= '0';
            if e_rund = '1' and e_run = '0' then
                gt_run <= '1'; gt_i <= (others => '0'); gt_acc <= (others => '0');
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_i;
                if s_src = '1' then gt_wd <= gt_acc(20 downto 5); else gt_wd <= gt_acc(21 downto 6); end if;
                gt_acc <= gt_acc + resize(c_g6, 22); gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;

            -- pixel side: the store's column counter (0 from T + 8), the reads, the interpolated phase
            if s_st(8) = '1' or s_st(9) = '1' or s_st(10) = '1' or s_st(11) = '1' or s_st(12) = '1' then
                lb_rc <= (others => '0');
            else
                lb_rc <= lb_rc + 1;
            end if;
            x_q0 <= lb_q2; x_q1 <= x_q0; x_q2 <= x_q1;
            x_a0 <= ly_q2;
            v_qs := resize(x_q0, 17) + resize(x_q1, 17);
            cm_p <= v_qs(16 downto 2);
            v_qd := signed(resize(x_q0, 17)) - signed(resize(x_q2, 17));
            cm_x <= v_qd(13 downto 2);
            cm_y <= resize(x_a0 & "00", 10);
            cm_s <= resize(unsigned(abs(resize(cm_x, 13))), 13) + resize(unsigned(abs(resize(cm_y, 11))), 13);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [L] LINE ENGINE: the lattice row coordinate S of the next line (and the
    -- Gasket's frame base distance 3 (1/z - S)); loaded by the kick after the
    -- field engine, stepped at line step 12 (after the beam has loaded it).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_f3 : signed(28 downto 0);
        variable v_s28 : unsigned(27 downto 0);
    begin
        if rising_edge(clk) then
            if l_kick = '1' then
                e_S <= c_S0; e_lc <= (others => '0'); e_kr <= '1';
            elsif g_start = '1' then
                e_lc <= (others => '0'); e_kr <= '0';
            elsif e_lc /= 31 then
                e_lc <= e_lc + 1;
            end if;
            case to_integer(e_lc) is
                when 12 =>
                    if e_kr = '0' then e_S <= e_S + signed(resize(c_dS, 26)); end if;
                when 13 =>
                    e_f <= resize(signed('0' & c_cz), 27) - resize(e_S, 27);
                when 14 =>
                    v_s28 := resize(unsigned(e_f(25 downto 0)) & '0', 28) + resize(unsigned(e_f), 28);   -- 3 e_f (see U4)
                    v_f3 := signed(e_f(26) & v_s28);
                    if v_f3 > 8388607 then n_F3 <= to_signed(8388607, 24);
                    elsif v_f3 < -8388608 then n_F3 <= to_signed(-8388608, 24);
                    else n_F3 <= v_f3(23 downto 0); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- the lattice BEAM: u = b - a along the line, S = a + b per line.  Pixel 0 at T + 8.
    ---------------------------------------------------------------------------
    p_gbeam : process(clk)
    begin
        if rising_edge(clk) then
            if s_st(6) = '1' then
                gb_u <= c_u0; gb_S <= e_S; gb_F3 <= n_F3;
            else
                gb_u <= gb_u + signed(resize(c_du, 26));
            end if;
        end if;
    end process p_gbeam;

    ---------------------------------------------------------------------------
    -- [U] THE LATTICE FRONT (both Forms), U1 .. U8.  Lattice coordinates a, b in
    -- units of the lattice triangle's height (Q20): Nest = one cell, Gasket = the
    -- zoomed frame triangle.  The carries of a + b mark the triangles: Nest, the
    -- carry into bit 20 = a down-cell; Gasket, the coarsest carry among the drawn
    -- levels = the hole level L*.  The triple = the cell's / hole's barycentrics.
    ---------------------------------------------------------------------------
    p_front : process(clk)
        variable v_al, v_be, v_sg : unsigned(19 downto 0);
        variable v_dn : std_logic;
        variable v_f3 : signed(23 downto 0);
        variable v_s23 : unsigned(22 downto 0);
        variable v27, w27 : signed(26 downto 0);
        variable v_cv : std_logic_vector(20 downto 0);
        variable v_po, v_oh : std_logic_vector(12 downto 0);
        variable v_mk : std_logic_vector(19 downto 0);
        variable v_dl : unsigned(20 downto 0);
        variable v_L  : unsigned(3 downto 0);
        variable v_x  : signed(23 downto 0);
        variable v32  : unsigned(31 downto 0);
    begin
        if rising_edge(clk) then
            -- U1: a = floor((S - u) / 2), b = ceil((S + u) / 2), so a + b = S exactly (Gasket: saturated at
            --     +-2 heights; Nest: modulo 2 cells, only the fraction and the carry into bit 20 matter)
            v27 := resize(gb_S, 27) - resize(gb_u, 27);
            w27 := resize(gb_S, 27) + resize(gb_u, 27) + 1;
            if s_gas = '1' and v27(26 downto 22) /= "00000" and v27(26 downto 22) /= "11111" then
                if v27(26) = '1' then u1_a <= to_signed(-2097152, 22); else u1_a <= to_signed(2097151, 22); end if;
            else u1_a <= v27(22 downto 1); end if;
            if s_gas = '1' and w27(26 downto 22) /= "00000" and w27(26 downto 22) /= "11111" then
                if w27(26) = '1' then u1_b <= to_signed(-2097152, 22); else u1_b <= to_signed(2097151, 22); end if;
            else u1_b <= w27(22 downto 1); end if;
            if s_gas = '1' and gb_S(25 downto 21) /= "00000" and gb_S(25 downto 21) /= "11111" then
                if gb_S(25) = '1' then u1_S <= to_signed(-2097152, 22); else u1_S <= to_signed(2097151, 22); end if;
            else u1_S <= gb_S(21 downto 0); end if;
            -- U2: the carries (Gasket: the drawn levels L at bit 20 - L, prefix OR and one-hot; Nest: bit 20);
            --     3a, 3b for the Gasket's frame
            v_cv := std_logic_vector(u1_S(20 downto 0) xor u1_a(20 downto 0) xor u1_b(20 downto 0));
            v_po(0) := v_cv(19) and c_lvm(0); v_oh(0) := v_po(0);
            for k in 1 to 12 loop
                v_po(k) := v_po(k - 1) or (v_cv(19 - k) and c_lvm(k));
                v_oh(k) := v_cv(19 - k) and c_lvm(k) and not v_po(k - 1);
            end loop;
            u2_po <= v_po; u2_oh <= v_oh; u2_dn <= v_cv(20);
            if u1_b < u1_a then u2_mab <= u1_b; else u2_mab <= u1_a; end if;
            u2_a <= unsigned(u1_a(19 downto 0)); u2_b <= unsigned(u1_b(19 downto 0)); u2_S <= unsigned(u1_S(19 downto 0));
            -- U3 + U4: the bits below the level (Gasket: L*; Nest: the whole cell), delta, L*; the triple
            --      (down: delta - a, delta - b, s;  up: a, b, delta - s); the frame distance min(3a, 3b, 3 (1/z - S))
            if s_gas = '1' then
                v_mk := (others => u2_po(12));
                v_mk(19) := '0';
                for q in 6 to 18 loop v_mk(q) := u2_po(18 - q); end loop;
                v_dl := (others => '0');
                for L in 1 to 13 loop v_dl(20 - L) := u2_oh(L - 1); end loop;
                v_dn := u2_po(12);
            else
                v_mk := (others => '1');
                v_dl := to_unsigned(1048576, 21);
                v_dn := u2_dn;
            end if;
            v_al := u2_a and unsigned(v_mk); v_be := u2_b and unsigned(v_mk); v_sg := u2_S and unsigned(v_mk);
            if v_dn = '1' then
                u4_t0 <= v_dl - resize(v_al, 21); u4_t1 <= v_dl - resize(v_be, 21); u4_t2 <= resize(v_sg, 21);
            else
                u4_t0 <= resize(v_al, 21); u4_t1 <= resize(v_be, 21); u4_t2 <= v_dl - resize(v_sg, 21);
            end if;
            u4_dn <= v_dn;
            v_L := (others => '0');
            for L in 1 to 13 loop
                if u2_oh(L - 1) = '1' then v_L := v_L or to_unsigned(L, 4); end if;
            end loop;
            g_L(4) <= v_L; g_hole(4) <= u2_po(12); tb_ra <= resize(v_L, 8);
            -- 3 x as sign & ((x low bits & '0') + x unsigned): no operand pair shares a net (a sign-extended
            -- x + 2x puts the sign bit on both inputs of one LUT, which hangs router1)
            v_s23 := resize(unsigned(u2_mab(20 downto 0)) & '0', 23) + resize(unsigned(u2_mab), 23);
            v_f3 := signed(u2_mab(21) & v_s23);
            if gb_F3 < v_f3 then u4_fd <= gb_F3; u4_jf <= '1'; else u4_fd <= v_f3; u4_jf <= '0'; end if;
            -- U5, U6: min and max of the triple (index 2 = the horizontal edge); the frame scaled (>> gsh)
            if u4_t1 < u4_t0 then u5_mn <= u4_t1; else u5_mn <= u4_t0; end if;
            if u4_t1 > u4_t0 then u5_mx <= u4_t1; else u5_mx <= u4_t0; end if;
            u5_t2 <= u4_t2; u5_dn <= u4_dn;
            v_x := shift_right(u4_fd, to_integer(c_gsh));
            if v_x > 4095 then g_fd(5) <= to_signed(4095, 13);
            elsif v_x < -4096 then g_fd(5) <= to_signed(-4096, 13);
            else g_fd(5) <= v_x(12 downto 0); end if;
            g_jf(5) <= u4_jf;
            if u5_t2 < u5_mn then u6_mn <= u5_t2; u6_kn2 <= '1'; else u6_mn <= u5_mn; u6_kn2 <= '0'; end if;
            if u5_t2 > u5_mx then u6_mx <= u5_t2; u6_kx2 <= '1'; else u6_mx <= u5_mx; u6_kx2 <= '0'; end if;
            u6_dn <= u5_dn;
            u6_T1 <= tb_q1; u6_T2 <= tb_q2;
            -- U7: Nest: 3d min (Q14 x 6 bits), max x 3d/u (two partial products); Gasket: 3 min, 3 max, delta -/+ T
            u7_pm  <= resize(u6_mn(20 downto 6) * c_D3, 21);
            u7_plo <= resize(u6_mx(20 downto 9) * c_D3i(8 downto 3), 18);
            u7_phi <= resize(u6_mx(20 downto 9) * c_D3i(14 downto 9), 18);
            u7_p3  <= resize(u6_mn & '0', 23) + resize(u6_mn, 23);
            u7_x3  <= resize(u6_mx & '0', 23) + resize(u6_mx, 23);
            u7_c1 <= signed(resize(u6_T1 & "0000", 23));           -- delta - T1 (the table holds it / 16)
            u7_c2 <= signed(resize(u6_T2 & "00000", 23));          -- delta + T2 (/ 32)
            u7_kn2 <= u6_kn2; u7_kx2 <= u6_kx2; u7_dn <= u6_dn;
            -- U8: Nest: 3d min lambda (Q12 bands) and 3d max lambda / u; Gasket: m1 = delta - T1 - 3 min,
            --     m2 = 3 max - delta - T2
            u8_lmin <= resize(u7_pm(20 downto 2), 19);
            v32 := shift_left(resize(u7_phi, 32), 6) + resize(u7_plo, 32);
            u8_lmx <= v32(23 downto 4);
            u8_m1 <= resize(u7_c1, 24) - signed(resize(u7_p3, 24));
            u8_m2 <= signed(resize(u7_x3, 24)) - resize(u7_c2, 24);
            u8_kn2 <= u7_kn2; u8_kx2 <= u7_kx2; u8_dn <= u7_dn;
            -- pipes
            for k in 5 to 9 loop g_L(k) <= g_L(k - 1); end loop;
            for k in 5 to 12 loop g_hole(k) <= g_hole(k - 1); end loop;
            for k in 6 to 12 loop g_fd(k) <= g_fd(k - 1); end loop;
            for k in 6 to 11 loop g_jf(k) <= g_jf(k - 1); end loop;
        end if;
    end process p_front;

    ---------------------------------------------------------------------------
    -- [N] NEST tail, U9 .. U15
    --   N = min(3d min lambda, d (1 + 1/u) - 3d max lambda / u): the turned nest's phase, 0 on its outline
    --   phase = N + offset(up / down) + camera - drift (mod 4 bands)
    ---------------------------------------------------------------------------
    p_ntail : process(clk)
        variable v_s  : unsigned(13 downto 0);
        variable v_dc : signed(19 downto 0);
        variable v_g  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            -- U9: the turned family's phase; offsets + camera (Lp at T + 15); |Lx| + |Ly|
            n9_phi <= signed(resize(c_cz(17 downto 0), 20)) - signed(resize(u8_lmx, 20));
            n9_lmin <= u8_lmin;
            if u8_dn = '0' then
                n9_DL <= c_poup + cm_p(13 downto 0); v_s := c_podn + cm_p(13 downto 0);
            else
                n9_DL <= c_podn + cm_p(13 downto 0); v_s := c_poup + cm_p(13 downto 0);
            end if;
            n9_nn <= v_s(13 downto 12);
            n9_kn2 <= u8_kn2; n9_kx2 <= u8_kx2;
            -- U10: the nest phase N (the nearer family)
            if n9_phi < signed(resize(n9_lmin, 20)) then
                n10_N <= n9_phi; n10_inv <= '1'; n10_k2 <= n9_kx2;
            else
                n10_N <= signed(resize(n9_lmin, 20)); n10_inv <= '0'; n10_k2 <= n9_kn2;
            end if;
            n10_DL <= n9_DL; n10_nn <= n9_nn; n10_ls <= cm_s;
            -- U11: phase; the base gradient (L1, bands per pixel) of the active edge
            n11_phi <= unsigned(n10_N(13 downto 0)) + n10_DL;
            if n10_inv = '1' then
                if n10_k2 = '1' then n11_g1 <= c_L1bi; else n11_g1 <= c_L1ai; end if;
            else
                if n10_k2 = '1' then n11_g1 <= c_gb; else n11_g1 <= c_ga; end if;
            end if;
            if n10_N > 8191 then n11_N <= to_signed(8191, 14);
            elsif n10_N < -8191 then n11_N <= to_signed(-8191, 14);
            else n11_N <= n10_N(13 downto 0); end if;
            n11_inv <= n10_inv; n11_nn <= n10_nn; n11_ls <= n10_ls;
            -- U12: band, distance to the nearest band boundary, |grad| with the camera's slopes
            n12_nb <= n11_phi(13 downto 12); n12_hi <= n11_phi(11);
            if n11_phi(11) = '1' then n12_db <= (not n11_phi(11 downto 0)) + 1; else n12_db <= n11_phi(11 downto 0); end if;
            v_s := resize(n11_g1, 14) + resize(n11_ls, 14);
            if v_s(13) = '1' then n12_w <= (others => '1'); else n12_w <= v_s(12 downto 0); end if;
            n12_N <= n11_N; n12_inv <= n11_inv; n12_nn <= n11_nn;
            if n11_N(13) = '1' then n12_aN <= unsigned(-n11_N(12 downto 0)); else n12_aN <= unsigned(n11_N(12 downto 0)); end if;
            -- U13: the nearest edge (band boundary or region) and its signed distance (Fine: the magnitude);
            --      the Fine half-width (1/12 band, at least one pixel)
            if signed(resize(n12_db, 14)) <= n12_N then
                n13_sb <= '1'; n13_dist <= signed(resize(n12_db, 14));
            else
                n13_sb <= '0';
                if s_fine = '1' then n13_dist <= signed(resize(n12_aN, 14)); else n13_dist <= n12_N; end if;
            end if;
            if n12_w(12 downto 11) /= "00" then n13_hw <= to_unsigned(2047, 11);
            elsif n12_w(10 downto 0) < 341 then n13_hw <= to_unsigned(341, 11);
            else n13_hw <= n12_w(10 downto 0); end if;
            n13_nb <= n12_nb; n13_hi <= n12_hi; n13_w <= n12_w; n13_inv <= n12_inv; n13_nn <= n12_nn;
            -- U14: hand-over values
            if s_fine = '0' then v_dc := resize(n13_dist, 20);
            else v_dc := signed(resize(n13_hw, 20)) - resize(n13_dist, 20); end if;
            if v_dc > 4095 then n14_d <= to_signed(4095, 13);
            elsif v_dc < -4096 then n14_d <= to_signed(-4096, 13);
            else n14_d <= v_dc(12 downto 0); end if;
            n14_w <= n13_w;
            n14_code(1 downto 0) <= n13_nb; n14_code(2) <= n13_hi; n14_code(3) <= n13_sb; n14_code(4) <= n13_inv;
            n14_code(6 downto 5) <= n13_nn;
            if n13_w < 1365 then n14_code(10 downto 7) <= "0000";
            elsif n13_w >= 1365 + 1024 then n14_code(10 downto 7) <= "0100";
            else
                v_g := n13_w - 1365;
                n14_code(10 downto 7) <= "00" & v_g(9 downto 8);
            end if;
        end if;
    end process p_ntail;

    ---------------------------------------------------------------------------
    -- [G] GASKET tail, U9 .. U15
    --   dh = max(delta - T1 - 3 min, 3 max - delta - T2) + camera: the hole (a down-triangle cut by the turned
    --   one), <= 0 inside; the white's distance is the nearer of dh and the frame's
    ---------------------------------------------------------------------------
    p_gtail : process(clk)
        variable v_w  : unsigned(12 downto 0);
        variable v_ad : unsigned(14 downto 0);
        variable v_dd : signed(15 downto 0);
        variable v_x  : signed(23 downto 0);
        variable v_l  : unsigned(11 downto 0);
        variable v_a  : unsigned(15 downto 0);
        variable v_ds : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            -- U9: dh = max(m1, m2) (<= 0 inside the hole); the camera's erosion (Video min(2 Lp, 16383), Osc Lp / 2)
            if u8_m1 >= u8_m2 then g9_dh <= u8_m1; g9_j2 <= u8_kn2; else g9_dh <= u8_m2; g9_j2 <= u8_kx2; end if;
            if s_src = '1' then g9_er <= '0' & cm_p(14 downto 1);
            elsif cm_p(14 downto 13) /= "00" then g9_er <= to_unsigned(16383, 15);   -- Video: 2 Lp, capped
            else g9_er <= cm_p(13 downto 0) & '0'; end if;
            -- U10: >> gsh, saturated; the hole tone's table address (data at U11, copied at U12)
            tb_rh <= resize(g_L(9), 8);
            v_x := shift_right(g9_dh, to_integer(c_gsh));
            if v_x > 16383 then g10_dh <= to_signed(16383, 15);
            elsif v_x < -16384 then g10_dh <= to_signed(-16384, 15);
            else g10_dh <= v_x(14 downto 0); end if;
            g10_er <= g9_er; g10_j2 <= g9_j2;
            if s_src = '1' then g10_we <= resize(cm_s(12 downto 1), 12); else g10_we <= resize(cm_s, 12); end if;
            -- U11: + the erosion (bright areas shrink the holes: fine levels close first)
            g11_dh <= resize(g10_dh, 16) + signed(resize(g10_er, 16));
            g11_wc <= g10_we; g11_j2 <= g10_j2;
            -- U12: which edge is nearer; the widths; the hole tone (read at L*)
            g12_ht <= tb_q3(9 downto 0);
            if g_hole(11) = '1' and g_fd(11)(12) = '0' and g11_dh < resize(g_fd(11), 16) then g12_uh <= '1'; else g12_uh <= '0'; end if;
            if g11_dh(15) = '1' then v_a := unsigned(-g11_dh); else v_a := unsigned(g11_dh); end if;
            if v_a(15) = '1' then g12_adh <= (others => '1'); else g12_adh <= v_a(14 downto 0); end if;
            if g_fd(11)(12) = '1' then g12_afd <= resize(unsigned(-resize(g_fd(11), 14)), 15); else g12_afd <= resize(unsigned(g_fd(11)), 15); end if;
            if g11_j2 = '1' then g12_wh <= resize(c_gb(10 downto 0), 13) + resize(g11_wc, 13);
            else g12_wh <= resize(c_ga(10 downto 0), 13) + resize(g11_wc, 13); end if;
            if g_jf(11) = '1' then g12_wf <= resize(c_gb(10 downto 0), 13); else g12_wf <= resize(c_ga(10 downto 0), 13); end if;
            if g11_dh > 16383 then g12_dh <= to_signed(16383, 15); else g12_dh <= g11_dh(14 downto 0); end if;
            -- U13: the white's signed distance to the nearest edge (hole or frame) and its width (Fine: 3-px lines
            --      along every edge)
            if g12_uh = '1' then v_w := g12_wh; v_ad := g12_adh; v_dd := resize(g12_dh, 16);
            else v_w := g12_wf; v_ad := g12_afd; v_dd := resize(g_fd(12), 16); end if;
            if s_fine = '1' then g13_dW <= signed(resize(v_w + shift_right(v_w, 1), 16)) - signed(resize(v_ad, 16));
            else g13_dW <= v_dd; end if;
            g13_wh <= v_w; g13_uh <= g12_uh; g13_ht <= g12_ht;
            -- U14: hand-over
            v_ds := g13_dW;
            if v_ds > 4095 then g14_d <= to_signed(4095, 13);
            elsif v_ds < -4096 then g14_d <= to_signed(-4096, 13);
            else g14_d <= v_ds(12 downto 0); end if;
            g14_w <= g13_wh; g14_code <= g13_uh & g13_ht;
        end if;
    end process p_gtail;

    ---------------------------------------------------------------------------
    -- [E] COVERAGE AND BLEND (shared): at T + 21 the mode's values; coverage =
    -- clamp(16 (d + w/2 + w/32) / w) by 4 restoring steps (T + 29); blend far ->
    -- own (T + 33); the Nest's fade to mid-grey where bands outrun the raster (T + 35)
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_c  : unsigned(10 downto 0);
        variable v_own, v_far : unsigned(9 downto 0);
        variable v_f  : unsigned(2 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_g  : signed(15 downto 0);
        variable v_s  : signed(15 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 21: the mode's hand-over
            if s_gas = '1' then
                bk_d <= g14_d; bk_w <= g14_w; v_code(21) <= g14_code;
            else
                bk_d <= n14_d; bk_w <= n14_w; v_code(21) <= n14_code;
            end if;
            for k in 22 to 28 loop v_code(k) <= v_code(k - 1); end loop;
            for k in 31 to 33 loop v_f4(k) <= v_f4(k - 1); end loop;
            -- 22, 23
            x14_d <= bk_d;
            x14_wh <= shift_right(bk_w, 1) + shift_right(bk_w, 5);
            x14_w <= bk_w;
            x15_e <= resize(x14_d, 15) + signed(resize(x14_wh, 15));
            x15_w <= x14_w;
            -- 24 .. 28
            dv_z(0) <= x15_e(14);
            if x15_e >= signed(resize(x15_w, 15)) then dv_f(0) <= '1'; else dv_f(0) <= '0'; end if;
            dv_r(0) <= unsigned(x15_e(12 downto 0));
            dv_w(0) <= x15_w;
            for i in 1 to 3 loop
                v_t := dv_r(i - 1) & '0';
                if v_t >= resize(dv_w(i - 1), 14) then
                    dv_r(i) <= resize(v_t - resize(dv_w(i - 1), 14), 13);
                    dv_q(i) <= dv_q(i - 1)(1 downto 0) & '1';
                else
                    dv_r(i) <= v_t(12 downto 0);
                    dv_q(i) <= dv_q(i - 1)(1 downto 0) & '0';
                end if;
                dv_w(i) <= dv_w(i - 1);
                dv_z(i) <= dv_z(i - 1); dv_f(i) <= dv_f(i - 1);
            end loop;
            -- 29
            if dv_z(3) = '1' then x29_cov <= (others => '0');
            elsif dv_f(3) = '1' then x29_cov <= to_unsigned(8, 4);
            else x29_cov <= '0' & dv_q(3); end if;
            -- 30 .. 33: far + ((own - far) * cov + 8) / 16
            -- the tones from the code: Nest (nb, hi, sb, inv, nn, f4), Gasket (uh, HT)
            v_c := v_code(28);
            if s_gas = '1' then
                v_f := (others => '0');
                if s_fine = '0' then
                    v_own := to_unsigned(940, 10);
                    if v_c(10) = '1' then v_far := to_unsigned(940, 10) - v_c(9 downto 0); else v_far := to_unsigned(64, 10); end if;
                else
                    if v_c(10) = '1' then v_own := to_unsigned(64, 10) + v_c(9 downto 0); else v_own := to_unsigned(940, 10); end if;
                    v_far := to_unsigned(64, 10);
                end if;
            else
                v_f := v_c(9 downto 7);
                if s_fine = '0' then
                    v_own := f_tone(v_c(1 downto 0), s_ink);
                    if v_c(3) = '1' then
                        if v_c(2) = '1' then v_far := f_tone(v_c(1 downto 0) + 1, s_ink); else v_far := f_tone(v_c(1 downto 0) - 1, s_ink); end if;
                    elsif v_c(4) = '1' then v_far := to_unsigned(64, 10);
                    else v_far := f_tone(v_c(6 downto 5), s_ink); end if;
                else
                    if v_c(3) = '1' then
                        if v_c(2) = '1' then v_own := f_ltone(v_c(1 downto 0) + 1, s_ink); else v_own := f_ltone(v_c(1 downto 0), s_ink); end if;
                    else v_own := to_unsigned(940, 10); end if;
                    v_far := to_unsigned(64, 10);
                end if;
            end if;
            b30_df <= signed(resize(v_own, 11)) - signed(resize(v_far, 11));
            b30_cov <= x29_cov; b30_far <= v_far; v_f4(30) <= v_f;
            if b30_cov(3) = '1' then
                b31_pa <= shift_left(resize(b30_df, 16), 3); b31_pb <= (others => '0');
            else
                if b30_cov(2) = '1' then b31_pa <= shift_left(resize(b30_df, 16), 2); else b31_pa <= (others => '0'); end if;
                if b30_cov(1) = '1' then v_g := shift_left(resize(b30_df, 16), 1); else v_g := (others => '0'); end if;
                if b30_cov(0) = '1' then b31_pb <= v_g + resize(b30_df, 16); else b31_pb <= v_g; end if;
            end if;
            b31_far <= b30_far;
            b32_s <= b31_pa + b31_pb + 4;
            b32_far <= b31_far;
            v_s := signed(resize(b32_far, 16)) + shift_right(b32_s, 3);
            b33_y <= unsigned(v_s(9 downto 0));
            -- 34, 35: y (4 - f) / 4 + 502 f / 4
            case to_integer(v_f4(33)) is
                when 0      => f34_t <= resize(b33_y & "00", 12);       f34_c <= to_unsigned(2, 12);
                when 1      => f34_t <= resize(b33_y & '0', 12) + resize(b33_y, 12); f34_c <= to_unsigned(504, 12);
                when 2      => f34_t <= resize(b33_y & '0', 12);        f34_c <= to_unsigned(1006, 12);
                when 3      => f34_t <= resize(b33_y, 12);              f34_c <= to_unsigned(1508, 12);
                when others => f34_t <= (others => '0');                f34_c <= to_unsigned(2010, 12);
            end case;
            v_y := f34_t + f34_c;
            s_y35 <= v_y(11 downto 2);
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: luma [1 2 1]/4, luma 13/16 toward black, blanking
    -- gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd1 <= s_y35; s_yd2 <= s_yd1;
            s_ya <= resize(s_y35, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y37 <= v_y(11 downto 2);
            if s_y37 > 64 then v_dl := s_y37 - 64; else v_dl := (others => '0'); end if;
            s_y38 <= s_y37 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if dl_q(2) = '1' then
                s_y39 <= s_y38;
            else
                s_y39 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y39);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= dl_q2(0);
    data_out.vsync_n <= dl_q2(1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= dl_q2(2);

end architecture trine;
