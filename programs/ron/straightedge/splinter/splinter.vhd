-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Splinter v0.1.0 -- Straightedge.  The pinwheel tiling of John Conway and Charles Radin (1994): a right
-- triangle with legs 1 and 2 splits into five copies of itself, each turned by atan(1/2), so the triangles
-- point in infinitely many directions (the facade of Federation Square, Melbourne).  Luma only (U = V = 512).
--
-- The picture is a per-pixel cascade.  The pixel's coordinates in the top triangle (right angle at the origin,
-- long leg 2 along x, short leg 1 along y; s = x + 2y, t = 2x - y):
--     child 0  t < 0        (u, v) = (2 - s, -t)        mirrored
--     child 1  s < 1        (u, v) = (t, 1 - s)         mirrored
--     child 2  t >= 2       (u, v) = (t - 2, 2 - s)     mirrored
--     child 3  2s + t < 4   (u, v) = (t, s - 1)         direct
--     child 4  else         (u, v) = (2 - t, 2 - s)     direct
-- adds and bit inversions only (midpoint convention), eight levels deep.  Child 3's map fixes the point
-- (1/2, 1/2): the eye of the endless spiral zoom (x sqrt 5 with a turn of atan(1/2) per level; the phase
-- wraps at one level and the levels relabel, so nothing jumps).
--
-- A piece of size level L splits when L passes its threshold (Shatter + a hash of its path).  pi = the
-- first unsplit piece on the pixel's path (levels 3 .. 7): its tone, its kerf.  The camera's element kappa
-- is the piece at level 8 (or 7): a white similar triangle inside it, its size set by the luma held in a
-- 60 x 32 block grid written during the previous field.
--
--   K1  X        the zoom's eye, +-480 virtual px                   (glided)
--   K2  Y        +-270 virtual px                                   (glided)
--   K3  Scale    the great triangles' size, 2.5 .. 3.5 levels over the floor (glided)
--   K4  Gap      the kerf: a hairline .. exploded shards            (glided)
--   K5  Drift    bipolar speed (Hypnos), still at the centre: the endless spiral zoom
--   K6  Camera   the luma sizes a white splinter in every level-8 piece (glided)
--   S7  Glide    Off / On      S8 Lines  Finest / All      S9 Tone   Grey / Ink
--   S10 Weight   Bold / Fine   S11 Source Video / Osc
--   P12 Shatter  how far the triangles have broken (glided)
--
-- Sections (the integer model .straightedge_work/rb_splinter/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width / height, generated avid
--   [B] CONTROLS                         switches latched per field (the holds and glides run in [C])
--   [C] FIELD ENGINE                     Hardline flute's microcoded engine (splinter.py): holds, glides,
--                                        drift, the view, the thresholds, the kerf table
--   [L] LINE ENGINE + BEAM               the top-triangle coordinates of every pixel (two affine DDAs)
--   [D] SOURCE FIELD                     Osc lattice, block box, vertical one-pole, gain, the block grid
--   [S] THE CASCADE                      eight child steps (two stages each), the digits and hashes
--   [P] PI                               the split decisions, pi's capture, its ranks and tone
--   [E] LEAF                             pi's kerf: the coverage
--   [K] KAPPA                            the camera element and the luma of the pixel's block: the splinter
--   [T] TONE                             two blends
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  A register "at stage P" holds DDA pixel i at T + 8 + P + i.  The level-n
-- coordinates are at stage 2n; pi at 16; the coverages at 22; the tone at 27; the output register holds
-- column j at T + 41 + j (the [1 2 1] centred on DDA pixel j + 1).  One latency (40) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.splinter_pkg.all;
use work.all;

architecture splinter of program_top is
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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- the anti-aliasing unit in half virtual px: 1080p 2, 720p / 1080i 3, 480p / 576p 4, 480i / 576i 6
    function f_aa(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8               => return to_unsigned(6, 3);
            when 4 | 12              => return to_unsigned(4, 3);
            when 5 | 6 | 14 | 1 | 2 | 10 => return to_unsigned(3, 3);
            when others              => return to_unsigned(2, 3);
        end case;
    end function;
    -- the grid's vertical one-pole shift: 3 (1080p, 720p), 2 (1080i, 480p, 576p), 1 (480i, 576i)
    function f_kv(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8               => return to_unsigned(1, 2);
            when 4 | 12 | 1 | 2 | 10 => return to_unsigned(2, 2);
            when others              => return to_unsigned(3, 2);
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
    -- the hypotenuse distance: h / sqrt 5 ~ h (1/2 - 1/16 + 1/128)
    function f_hyp(h : unsigned) return unsigned is
        variable v : unsigned(h'length - 1 downto 0);
    begin
        v := shift_right(h, 1) - shift_right(h, 4) + shift_right(h, 7);
        return v;
    end function;
    function f_min(a, b : signed) return signed is
    begin
        if a < b then return a; else return b; end if;
    end function;
    function f_max(a, b : signed) return signed is
    begin
        if a > b then return a; else return b; end if;
    end function;
    -- coverage floor(4 m / pix), 0 .. 4 (four comparisons)
    function f_cov4(m : signed(15 downto 0); pix : unsigned(12 downto 0)) return unsigned is
        variable v_m4, v_m2, v_p1, v_p3 : signed(17 downto 0);
    begin
        v_m4 := m & "00"; v_m2 := resize(m & '0', 18);
        v_p1 := signed(resize(pix, 18));
        v_p3 := signed(resize(pix & '0', 18)) + v_p1;
        if m(15) = '1' then return to_unsigned(0, 3);
        elsif resize(m, 18) >= v_p1 then return to_unsigned(4, 3);
        elsif v_m4 >= v_p3 then return to_unsigned(3, 3);
        elsif v_m2 >= v_p1 then return to_unsigned(2, 3);
        elsif v_m4 >= v_p1 then return to_unsigned(1, 3);
        else return to_unsigned(0, 3); end if;
    end function;
    -- tone codes: 0 .. 4 the Grey tone by child index, 5 black, 6 white
    function f_tone(c : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(c) is
            when 0      => return to_unsigned(449, 10);
            when 1 | 2  => return to_unsigned(625, 10);
            when 3      => return to_unsigned(800, 10);
            when 4      => return to_unsigned(292, 10);
            when 5      => return to_unsigned(64, 10);
            when others => return to_unsigned(940, 10);
        end case;
    end function;
    function f_ltone(c : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(c) is
            when 0      => return to_unsigned(695, 10);
            when 1 | 2  => return to_unsigned(782, 10);
            when 3      => return to_unsigned(870, 10);
            when 4      => return to_unsigned(616, 10);
            when others => return to_unsigned(940, 10);
        end case;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width / height
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr, s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_aar, s_aa : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_kvr, s_kv : unsigned(1 downto 0) := to_unsigned(3, 2);
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
    signal s_st       : std_logic_vector(0 to 7) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] switches
    signal s_glide : std_logic := '1';
    signal s_all, s_ink, s_fine, s_src : std_logic := '0';
    signal s_bank : std_logic := '0';

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
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";      -- 0 return, 1 halt
    signal rf_ram : t_c_sp_rfi := C_SP_RFI;
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
    signal w_in  : unsigned(15 downto 0);
    signal ft_adr : unsigned(8 downto 0) := (others => '0');
    signal ft_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal e_wd  : unsigned(1 downto 0) := (others => '0');
    -- the engine's outputs (per-field constants)
    signal c_dxx, c_dyx, c_dxy, c_dyy : signed(19 downto 0) := (others => '0');
    signal c_phi : unsigned(7 downto 0) := (others => '0');
    signal c_lh  : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal c_at  : signed(11 downto 0) := (others => '0');
    signal c_pk7, c_pk8 : unsigned(10 downto 0) := to_unsigned(64, 11);
    signal c_g6  : unsigned(8 downto 0) := (others => '0');
    signal l_kick : std_logic := '0';
    -- the per-level table (five read copies) and the rotation table
    type t_tb is array (0 to 255) of unsigned(15 downto 0);
    signal tb1_ram, tb2_ram, tb3_ram, tb4_ram : t_tb := (others => (others => '0'));
    attribute no_rw_check of tb1_ram : signal is true;
    attribute no_rw_check of tb2_ram : signal is true;
    attribute no_rw_check of tb3_ram : signal is true;
    attribute no_rw_check of tb4_ram : signal is true;
    signal tb_we : std_logic := '0';
    signal tb_wa : unsigned(7 downto 0) := (others => '0');
    signal tb_wd : unsigned(15 downto 0) := (others => '0');
    signal tb_a1, tb_a2, tb_a3, tb_a4 : unsigned(7 downto 0);
    signal tb_q1, tb_q2, tb_q3, tb_q4 : unsigned(15 downto 0) := (others => '0');

    -- [L] line engine and beam
    signal e_kr  : std_logic := '0';
    signal e_lc  : unsigned(4 downto 0) := to_unsigned(31, 5);
    signal e_lx, e_ly : unsigned(29 downto 0) := (others => '0');
    signal b_x, b_y : unsigned(29 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd, s_fl, s_seen : std_logic := '0';
    signal s_bh  : unsigned(12 downto 0) := (others => '0');
    signal s_bcol : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal s_racc : unsigned(13 downto 0) := (others => '0');
    signal s_row  : unsigned(4 downto 0) := (others => '0');
    signal fl_v, g1_v, g2_v, g3_v, g4_v, g5_v : std_logic := '0';
    signal fl_col, g1_col, g2_col, g3_col, g4_col, g5_col : unsigned(5 downto 0) := (others => '0');
    signal fl_sum : unsigned(12 downto 0) := (others => '0');
    signal g1_mean, g2_mean : unsigned(7 downto 0) := (others => '0');
    signal g1_fl, g2_fl : std_logic := '0';
    signal iq_ram : t_tb := (others => (others => '0'));
    attribute no_rw_check of iq_ram : signal is true;
    signal iq_q, iq_f, iq_wd : unsigned(15 downto 0) := (others => '0');
    signal iq_we : std_logic := '0';
    signal iq_wa : unsigned(7 downto 0) := (others => '0');
    signal g5_v6 : unsigned(5 downto 0) := (others => '0');
    signal g6_p  : unsigned(13 downto 0) := (others => '0');
    type t_cg is array (0 to 4095) of unsigned(3 downto 0);
    signal cg_ram : t_cg := (others => (others => '0'));
    attribute no_rw_check of cg_ram : signal is true;
    signal cg_we : std_logic := '0';
    signal cg_wa, cg_ra : unsigned(11 downto 0) := (others => '0');
    signal cg_wd, cg_q, cg_f : unsigned(3 downto 0) := (others => '0');

    -- [S] the cascade (index = level): coordinates 1.(17 - n) at level n, midpoint convention (a register X
    -- stands for X + 1/2 LSB); only the low 18 - n bits of level n are driven
    type t_co is array (0 to 8) of unsigned(17 downto 0);
    signal cx, cy : t_co := (others => (others => '0'));
    type t_d3 is array (0 to 8) of unsigned(2 downto 0);
    signal dB1, dB2 : t_d3 := (others => "011");        -- the level's digit and its parent's (even stages)
    signal dA1 : t_d3 := (others => "011");             -- the level's digit one stage later
    type t_h4 is array (0 to 8) of unsigned(3 downto 0);
    signal hpA, h2pA, hB, h2B : t_h4 := (others => (others => '0'));
    signal mB, mA : std_logic_vector(0 to 8) := (others => '0');          -- mirrored

    -- [P] pi (index = level)
    type t_u10a is array (3 to 6) of unsigned(9 downto 0);
    type t_s11a is array (3 to 6) of signed(10 downto 0);
    signal q_cl : t_u10a := (others => (others => '0'));
    signal q_e  : t_s11a := (others => (others => '0'));
    signal stB, stA : std_logic_vector(2 to 7) := (others => '0');
    type t_p15 is array (3 to 7) of unsigned(14 downto 0);
    signal pAx, pAy, pBx, pBy : t_p15 := (others => (others => '0'));
    type t_rk is array (0 to 7) of unsigned(8 downto 0);            -- short & long & hyp
    signal rkB, rkD : t_rk := (others => (others => '0'));
    type t_mt is array (2 to 7) of unsigned(6 downto 0);            -- level & mirrored & digit
    signal mtB, mtD : t_mt := (others => (others => '0'));

    -- [E] leaf (stage numbers), 1.14
    signal l16_px, l16_py, l17_px, l17_py, l18_px, l18_py : unsigned(14 downto 0) := (others => '0');
    signal l17_hr, l18_deh : unsigned(14 downto 0) := (others => '0');
    signal tf_gs, tf_gl, tf_gh, tf_pix : unsigned(12 downto 0) := (others => '0');
    signal l19_es, l19_el, l19_eh, l20_m1, l20_eh : signed(15 downto 0) := (others => '0');
    signal l19_pix, l20_pix, l21_pix : unsigned(12 downto 0) := to_unsigned(64, 13);
    signal l21_np : signed(15 downto 0) := (others => '0');
    signal l21_tk : unsigned(14 downto 0) := (others => '0');
    signal l22_c1, l22_c2 : unsigned(2 downto 0) := (others => '0');
    type t_tc is array (17 to 22) of unsigned(3 downto 0);          -- mirrored & digit of pi
    signal l_tc : t_tc := (others => (others => '0'));

    -- [K] kappa, 1.10, and the block of the pixel
    signal k15_useg, k16_useg, k17_useg, k18_useg, k19_useg, k20_useg : std_logic := '0';
    signal k15_x7, k15_y7, k16_x7, k16_y7, k17_x, k17_y : unsigned(10 downto 0) := (others => '0');
    signal k18_hr, k18_mn, k19_kh, k19_mn, k20_dk : unsigned(10 downto 0) := (others => '0');
    signal k_bh : unsigned(11 downto 0) := (others => '0');
    signal k_racc : unsigned(13 downto 0) := (others => '0');
    signal k_row : unsigned(4 downto 0) := (others => '0');
    signal k_seen : std_logic := '0';
    signal s_bst : std_logic_vector(0 to 17) := (others => '0');     -- g_start delayed 9 .. 26
    signal k21_mi : signed(15 downto 0) := (others => '0');
    signal k21_pix : unsigned(12 downto 0) := (others => '0');
    signal k21_nz : std_logic := '0';
    signal k22_ci : unsigned(2 downto 0) := (others => '0');

    -- [T] tone
    signal b23_a : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal b23_c, b23_c2, b24_c, b24_c2, b25_c2, b26_c2 : unsigned(2 downto 0) := (others => '0');
    signal b24_df, b26_df : signed(10 downto 0) := (others => '0');
    signal b24_a, b25_v, b26_a : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal b27_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [F] output conditioning
    signal s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_yf, s_yg, s_yo : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    -- the per-level table: [s][0] = 1.14 LSB per pixel, [s][r] = the kerf of a rank-r edge.  Four copies,
    -- read at stage 16 (data registered at 17, copied to fabric at 18)
    p_tb1 : process(clk) begin if rising_edge(clk) then
        tb_q1 <= tb1_ram(to_integer(tb_a1)); if tb_we = '1' then tb1_ram(to_integer(tb_wa)) <= tb_wd; end if; end if; end process;
    p_tb2 : process(clk) begin if rising_edge(clk) then
        tb_q2 <= tb2_ram(to_integer(tb_a2)); if tb_we = '1' then tb2_ram(to_integer(tb_wa)) <= tb_wd; end if; end if; end process;
    p_tb3 : process(clk) begin if rising_edge(clk) then
        tb_q3 <= tb3_ram(to_integer(tb_a3)); if tb_we = '1' then tb3_ram(to_integer(tb_wa)) <= tb_wd; end if; end if; end process;
    p_tb4 : process(clk) begin if rising_edge(clk) then
        tb_q4 <= tb4_ram(to_integer(tb_a4)); if tb_we = '1' then tb4_ram(to_integer(tb_wa)) <= tb_wd; end if; end if; end process;
    -- the camera grid: 2 banks x 32 rows x 64 columns, 4 bits
    p_cg : process(clk)
    begin
        if rising_edge(clk) then
            cg_q <= cg_ram(to_integer(cg_ra));
            if cg_we = '1' then cg_ram(to_integer(cg_wa)) <= cg_wd; end if;
        end if;
    end process p_cg;

    -- the grid's vertical one-pole state, one word per block column
    p_iq : process(clk)
    begin
        if rising_edge(clk) then
            iq_q <= iq_ram(to_integer(resize(fl_col, 8)));
            if iq_we = '1' then iq_ram(to_integer(iq_wa)) <= iq_wd; end if;
        end if;
    end process p_iq;

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

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] switches
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_aar <= f_aa(s_tid);
            s_kvr <= f_kv(s_tid);

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
                s_all   <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_aa    <= s_aar;
                s_kv    <= s_kvr;
                s_bank  <= not s_bank;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
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

            -- (the syncs and the generated avid are delayed in the dl_ram ring, p_dl)
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 6);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] THE PER-FIELD ENGINE (Hardline flute's).  Started at every zone start
    -- from address 0; three clocks per micro-op (register-file read, the ROM word
    -- copied to fabric / operand, decode and next address / execute), serial
    -- multiply (18 steps) and a right shift of four bits a clock.  ACC = 37 bits;
    -- MUL = ACC(17:0) x operand(17:0), both signed, by add-and-shift-right with the
    -- last step subtracting.  splinter.py holds the microprogram, its assembler
    -- and its emulator.
    ---------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(s_w, 16)             when x"8",
        resize(s_h, 16)             when x"9",
        resize(s_ys, 16)            when x"A",
        unsigned(ft_q)              when x"B",
        resize(s_xs, 16)            when x"D",
        resize(s_aa, 16)            when x"E",
        resize(unsigned(registers_in(to_integer(unsigned(e_irf(2 downto 0))))), 16) when others;

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
            ft_q <= C_SP_FT(to_integer(ft_adr));
            if e_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(e_ir(6 downto 0))));
            end if;
            if e_run = '1' and e_ph = "10" and x_st = '1' then
                rf_ram(to_integer(unsigned(e_irf(6 downto 0)))) <= std_logic_vector(e_acc(31 downto 0));
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
                        elsif v_op = OP_SHR then e_cnt <= unsigned(v_arg(4 downto 0));
                        else e_cnt <= (others => '0'); end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := e_acc(36);
                            when "001"  => v_cond := s_glide;
                            when "010"  => v_cond := s_all;
                            when others => v_cond := s_src;
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
                e_ir <= C_SP_UC(to_integer(e_pc));
            end if;

            -- the engine's output registers
            tb_we <= '0'; l_kick <= '0';
            if e_out = '1' then
                case to_integer(unsigned(e_irf(4 downto 0))) is
                    when 0  => ft_adr <= unsigned(e_acc(8 downto 0));
                    when 3  => c_dxx  <= e_acc(19 downto 0);
                    when 4  => c_dyx  <= e_acc(19 downto 0);
                    when 5  => c_dxy  <= e_acc(19 downto 0);
                    when 6  => c_dyy  <= e_acc(19 downto 0);
                    when 7  => c_phi  <= unsigned(e_acc(7 downto 0));
                    when 8  => c_lh   <= unsigned(e_acc(9 downto 0));
                    when 9  => c_at   <= e_acc(11 downto 0);
                    when 10 => tb_wa  <= unsigned(e_acc(7 downto 0));
                    when 11 => tb_wd  <= unsigned(e_acc(15 downto 0)); tb_we <= '1';
                    when 12 => c_pk7  <= unsigned(e_acc(10 downto 0));
                    when 13 => c_pk8  <= unsigned(e_acc(10 downto 0));
                    when 19 => c_g6   <= unsigned(e_acc(8 downto 0));
                    when 20 => l_kick <= '1';
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
    -- [L] LINE ENGINE + BEAM: the top-triangle coordinates (Q28).  The line start
    -- is loaded by the engine's kick and stepped per line after the beam has taken
    -- it; the beam steps per pixel.  DDA pixel 0 at T + 8.
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            if l_kick = '1' then
                e_lc <= (others => '0'); e_kr <= '1';
            elsif g_start = '1' then
                e_lc <= (others => '0'); e_kr <= '0';
            elsif e_lc /= 31 then
                e_lc <= e_lc + 1;
            end if;
            if e_out = '1' and e_irf(4 downto 0) = "00001" then
                e_lx <= unsigned(e_acc(29 downto 0));               -- the engine's X0
            elsif e_lc = 12 and e_kr = '0' then
                e_lx <= unsigned(signed(e_lx) + resize(c_dxy, 30));
            end if;
            if e_out = '1' and e_irf(4 downto 0) = "00010" then
                e_ly <= unsigned(e_acc(29 downto 0));               -- the engine's Y0
            elsif e_lc = 12 and e_kr = '0' then
                e_ly <= unsigned(signed(e_ly) + resize(c_dyy, 30));
            end if;
            if s_st(6) = '1' then
                b_x <= e_lx; b_y <= e_ly;
            else
                b_x <= unsigned(signed(b_x) + resize(c_dxx, 30));
                b_y <= unsigned(signed(b_y) + resize(c_dyx, 30));
            end if;
        end if;
    end process p_line;
    cx(0) <= b_x(28 downto 11);
    cy(0) <= b_y(28 downto 11);

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> the mean over each 32-px
    -- block column of the line -> a vertical one-pole per block column -> gain
    -- (K6, from luma 0.26) -> the block grid (4 bits), written into the bank the
    -- pixel path is not reading.  The running sums restart at every block.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_col : unsigned(5 downto 0);
        variable v_m : unsigned(14 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_q : unsigned(11 downto 0);
        variable v_g : unsigned(7 downto 0);
        variable v_r : unsigned(14 downto 0);
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
            s_sv <= data_in.avid; s_svd <= s_sv;

            -- rows of 33.75 virtual lines; the first line of a field loads the one-pole
            if s_zone = '1' and s_sv = '0' then
                s_fl <= '1'; s_seen <= '0'; s_racc <= (others => '0'); s_row <= (others => '0');
            elsif s_sv = '1' and s_svd = '0' then
                if s_seen = '1' then
                    s_fl <= '0';
                    if s_racc + resize(s_ys, 14) >= 8640 then
                        s_racc <= s_racc + resize(s_ys, 14) - 8640;
                        if s_row /= 31 then s_row <= s_row + 1; end if;
                    else
                        s_racc <= s_racc + resize(s_ys, 14);
                    end if;
                end if;
                s_seen <= '1';
            end if;

            -- the block column of the sample and the block sums
            fl_v <= '0';
            v_col := s_bh(11 downto 6);
            if s_sv = '1' then
                s_bh <= s_bh + resize(s_xs, 13);
                if s_svd = '0' or v_col /= s_bcol then
                    fl_v <= s_svd; fl_col <= s_bcol; fl_sum <= s_bsum;
                    s_bsum <= resize(s_sl, 13); s_bcol <= v_col;
                else
                    s_bsum <= s_bsum + resize(s_sl, 13);
                end if;
            else
                if s_xs = 4 then s_bh <= to_unsigned(480, 13); else s_bh <= (others => '0'); end if;
                if s_svd = '1' then
                    fl_v <= '1'; fl_col <= s_bcol; fl_sum <= s_bsum;
                end if;
            end if;

            -- g1: the block mean (sum x xs / 64); the one-pole's read is issued from fl_col
            case to_integer(s_xs) is
                when 4      => v_m := resize(fl_sum & "00", 15);
                when 3      => v_m := resize(fl_sum & '0', 15) + resize(fl_sum, 15);
                when others => v_m := resize(fl_sum & '0', 15);
            end case;
            g1_mean <= v_m(13 downto 6); g1_v <= fl_v; g1_col <= fl_col; g1_fl <= s_fl;
            -- g2: the fabric copy of the RAM's read register (which is beside g1)
            g2_mean <= g1_mean; g2_v <= g1_v; g2_col <= g1_col; g2_fl <= g1_fl;
            iq_f <= iq_q;
            -- g3: q += (mean - q) >> kv (the first line loads it); the write
            v_d := signed(resize(g2_mean & "0000", 13)) - signed(resize(iq_f(11 downto 0), 13));
            case to_integer(s_kv) is
                when 1      => v_d := shift_right(v_d, 1);
                when 2      => v_d := shift_right(v_d, 2);
                when others => v_d := shift_right(v_d, 3);
            end case;
            v_d := signed(resize(iq_f(11 downto 0), 13)) + v_d;
            if g2_fl = '1' then v_q := g2_mean & "0000";
            else v_q := unsigned(v_d(11 downto 0)); end if;
            iq_wd <= resize(v_q, 16); iq_wa <= resize(g2_col, 8); iq_we <= g2_v;
            g3_v <= g2_v; g3_col <= g2_col;
            -- g4: (q - 55) >> 2 (0 .. 46)
            if iq_wd(11 downto 4) > 55 then v_g := iq_wd(11 downto 4) - 55; else v_g := (others => '0'); end if;
            g5_v6 <= v_g(7 downto 2); g4_v <= g3_v; g4_col <= g3_col;
            -- g5: x gain (K6 >> 4, 0 .. 63; Osc x3)
            g6_p <= g5_v6 * c_g6(8 downto 1); g5_v <= g4_v; g5_col <= g4_col;
            -- g6: >> 7, capped at 15; the grid write
            v_r := resize(shift_right(g6_p, 7), 15);
            if v_r > 15 then cg_wd <= to_unsigned(15, 4); else cg_wd <= v_r(3 downto 0); end if;
            cg_we <= g5_v; cg_wa <= s_bank & s_row & g5_col;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [S] THE CASCADE.  Step n takes the level-n coordinates (stage 2n, 1.(17 - n))
    -- to level n + 1 (stage 2n + 2).  Midpoint convention: a register X stands for
    -- X + 1/2 LSB, so no point lies on a dividing line and a negation is a bitwise
    -- NOT:
    --   A   S = X + 2Y + 1,  T = 2X - Y,  and the test 2S + T + 1 < 4
    --   B   the child, and its map as selected / inverted bits of S and T; one LSB
    --       is dropped (a point pushed onto the hypotenuse by that, S = 2, is held
    --       on it by the registers' reset)
    -- Beside it: the digits and the hashes of the path and the mirror flag.
    ---------------------------------------------------------------------------
    g_step : for n in 0 to 7 generate
        constant WN : integer := 18 - n;                      -- bits of the level's coordinates
        signal a_s : unsigned(WN downto 0) := (others => '0');
        signal a_t : signed(WN + 1 downto 0) := (others => '0');
        signal a_q : std_logic := '0';
    begin
        p_step : process(clk)
            variable v_sx, v_sy, v_sv : unsigned(WN + 2 downto 0);
            variable v_s : unsigned(WN downto 0);
            variable v_t : signed(WN + 1 downto 0);
            variable v_qa, v_qb, v_qv : signed(WN + 4 downto 0);
            variable v_c : unsigned(2 downto 0);
            variable v_d1, v_d2 : unsigned(3 downto 0);
            variable v_ns, v_nt, v_tt : unsigned(WN - 2 downto 0);
        begin
            if rising_edge(clk) then
                -- A (stage 2n + 1): the + 1 rides the carry-in
                v_sx := "00" & cx(n)(WN - 1 downto 0) & '1';
                v_sy := '0' & cy(n)(WN - 1 downto 0) & "01";
                v_sv := v_sx + v_sy;
                v_s := v_sv(WN + 1 downto 1);
                v_t := signed(resize(cx(n)(WN - 1 downto 0) & '0', WN + 2)) - signed(resize(cy(n)(WN - 1 downto 0), WN + 2));
                v_qa := signed(resize(v_s & "01", WN + 5));
                v_qb := resize(v_t & '1', WN + 5);
                v_qv := v_qa + v_qb;                          -- (2S + T + 1) & '0'
                a_s <= v_s; a_t <= v_t;
                if v_qv(WN + 4) = '1' or v_qv(WN + 3 downto WN + 2) = "00" then a_q <= '1'; else a_q <= '0'; end if;
                dA1(n) <= dB1(n); mA(n) <= mB(n);
                v_d1 := resize(dB1(n), 4); v_d2 := resize(dB2(n), 4);
                hpA(n)  <= shift_left(v_d1, 2) + v_d1 + shift_left(v_d2, 1) + v_d2 + 2;
                h2pA(n) <= shift_left(v_d1, 3) - v_d1 + shift_left(v_d2, 2) + v_d2 + 9;

                -- B (stage 2n + 2)
                if a_t(WN + 1) = '1' then v_c := "000";
                elsif a_s(WN downto WN - 1) = "00" then v_c := "001";
                elsif a_t(WN) = '1' then v_c := "010";
                elsif a_q = '1' then v_c := "011";
                else v_c := "100"; end if;
                v_ns := not a_s(WN - 1 downto 1); v_nt := not unsigned(a_t(WN - 1 downto 1));
                v_tt := unsigned(a_t(WN - 1 downto 1));
                -- u = 2 - s | t | t - 2 | t | 2 - t
                if v_c = "000" and a_s(WN) = '1' then cx(n + 1)(WN - 2 downto 0) <= (others => '0');
                elsif v_c = "000" then cx(n + 1)(WN - 2 downto 0) <= v_ns;
                elsif v_c = "100" then cx(n + 1)(WN - 2 downto 0) <= v_nt;
                else cx(n + 1)(WN - 2 downto 0) <= v_tt; end if;
                -- v = -t | 1 - s | 2 - s | s - 1 | 2 - s
                if (v_c = "010" or v_c = "100") and a_s(WN) = '1' then cy(n + 1)(WN - 2 downto 0) <= (others => '0');
                elsif v_c = "000" then cy(n + 1)(WN - 2 downto 0) <= v_nt;
                elsif v_c = "001" then cy(n + 1)(WN - 2 downto 0) <= '0' & v_ns(WN - 3 downto 0);
                elsif v_c = "011" then cy(n + 1)(WN - 2 downto 0) <= '0' & a_s(WN - 2 downto 1);
                else cy(n + 1)(WN - 2 downto 0) <= v_ns; end if;
                -- the path: digits, hashes, mirror
                dB1(n + 1) <= v_c; dB2(n + 1) <= dA1(n);
                case v_c is                                   -- + 7 c, + 3 c (mod 16)
                    when "000"  => hB(n + 1) <= hpA(n);      h2B(n + 1) <= h2pA(n);
                    when "001"  => hB(n + 1) <= hpA(n) + 7;  h2B(n + 1) <= h2pA(n) + 3;
                    when "010"  => hB(n + 1) <= hpA(n) + 14; h2B(n + 1) <= h2pA(n) + 6;
                    when "011"  => hB(n + 1) <= hpA(n) + 5;  h2B(n + 1) <= h2pA(n) + 9;
                    when others => hB(n + 1) <= hpA(n) + 12; h2B(n + 1) <= h2pA(n) + 12;
                end case;
                if v_c(2) = '0' and v_c(1 downto 0) /= "11" then mB(n + 1) <= not mA(n); else mB(n + 1) <= mA(n); end if;
            end if;
        end process p_step;
    end generate g_step;

    ---------------------------------------------------------------------------
    -- [P] PI.  The level-n piece (n = 3 .. 6) is split when, in 1/256 level,
    --   (6 - n) 256 + phi - 128 + 8 h2 - clip(AT + 20 h, 0, LH) >= 0     (AT, LH: the engine's, less 128)
    -- pi = the first unsplit level (else level 7).  Its coordinates (aligned to
    -- 1.14) are captured while not stopped; its ranks, digit and mirror flag freeze.
    ---------------------------------------------------------------------------
    p_pi : process(clk)
        variable v_t : signed(12 downto 0);
        variable v_a : signed(11 downto 0);
        variable v_rk : unsigned(8 downto 0);
        variable v_new : unsigned(2 downto 0);
        variable v_rs, v_rl, v_rh : unsigned(2 downto 0);
        variable v_lam : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            for n in 3 to 6 loop
                -- stage 2n + 1: the clipped threshold; (6 - n) 256 + phi - 128 + 8 h2
                v_t := resize(c_at, 13) + signed(resize(hB(n) & "0000", 13)) + signed(resize(hB(n) & "00", 13));
                if v_t(12) = '1' then q_cl(n) <= (others => '0');
                elsif v_t > signed(resize(c_lh, 13)) then q_cl(n) <= c_lh;
                else q_cl(n) <= unsigned(v_t(9 downto 0)); end if;
                v_lam := signed(resize(to_unsigned(6 - n, 2) & c_phi, 11)) - 128;
                q_e(n) <= v_lam + signed(resize(h2B(n) & "000", 11));
                -- stage 2n + 2: stopped at this level or above
                v_a := resize(q_e(n), 12) - signed(resize(q_cl(n), 12));
                stB(n) <= stA(n - 1) or v_a(11);
                -- stage 2n + 3
                stA(n) <= stB(n);
            end loop;
            -- pi's coordinates: captured at stage 2n + 1 while not stopped at a level above
            for n in 3 to 7 loop
                if stB(n - 1) = '1' then
                    pAx(n) <= pBx(n - 1 + boolean'pos(n = 3)); pAy(n) <= pBy(n - 1 + boolean'pos(n = 3));
                else
                    pAx(n) <= shift_left(resize(cx(n)(17 - n downto 0), 15), n - 3);
                    pAy(n) <= shift_left(resize(cy(n)(17 - n downto 0), 15), n - 3);
                end if;
                pBx(n) <= pAx(n); pBy(n) <= pAy(n);
            end loop;
            -- the ranks of the piece's edges (short, long, hypotenuse): level n at stage 2n + 2
            for n in 1 to 7 loop
                rkD(n) <= rkB(n - 1);
                v_new := to_unsigned(n, 3);
                v_rs := rkD(n)(8 downto 6); v_rl := rkD(n)(5 downto 3); v_rh := rkD(n)(2 downto 0);
                case dA1(n) is
                    when "000"  => v_rk := v_rh & v_new & v_rs;
                    when "001"  => v_rk := v_new & v_new & v_rl;
                    when "010"  => v_rk := v_new & v_rh & v_rl;
                    when "011"  => v_rk := v_new & v_new & v_new;
                    when others => v_rk := v_new & v_rh & v_new;
                end case;
                if n > 3 and stA(n - 1 + boolean'pos(n < 4) * (4 - n)) = '1' then rkB(n) <= rkD(n); else rkB(n) <= v_rk; end if;
            end loop;
            -- pi's level, mirror flag and digit: level n at stage 2n + 2
            for n in 3 to 7 loop
                mtD(n) <= mtB(n - 1);
                if n > 3 and stA(n - 1) = '1' then mtB(n) <= mtD(n);
                else mtB(n) <= to_unsigned(n, 3) & mA(n) & dA1(n); end if;
            end loop;
        end if;
    end process p_pi;

    ---------------------------------------------------------------------------
    -- [E] LEAF.  pi at stage 16: (px, py) 1.14, its level, its edges' ranks.
    --   17 the table (the kerf of each edge, the pixel), 18 its fabric copy and the hypotenuse distance
    --   19 .. 21 the margin = min(edge distance - kerf), + half a pixel
    --   22 the coverage floor(4 m / pix), 0 .. 4 (Fine: also one line thickness further in)
    ---------------------------------------------------------------------------
    tb_a1 <= '0' & mtB(7)(6 downto 4) & '0' & rkB(7)(8 downto 6);
    tb_a2 <= '0' & mtB(7)(6 downto 4) & '0' & rkB(7)(5 downto 3);
    tb_a3 <= '0' & mtB(7)(6 downto 4) & '0' & rkB(7)(2 downto 0);
    tb_a4 <= '0' & mtB(7)(6 downto 4) & "0000";

    p_leaf : process(clk)
        variable v_h : unsigned(15 downto 0);
        variable v_m : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 16 (the capture of level 7 is at 15; the ranks and the level arrive at 16)
            l16_px <= pAx(7); l16_py <= pAy(7);
            -- 17
            v_h := to_unsigned(32767, 16) - resize(l16_px, 16) - resize(l16_py & '0', 16);
            l17_hr <= v_h(14 downto 0);
            l17_px <= l16_px; l17_py <= l16_py;
            l_tc(17) <= mtB(7)(3 downto 0);
            for k in 18 to 22 loop l_tc(k) <= l_tc(k - 1); end loop;
            -- 18
            tf_gs <= tb_q1(12 downto 0); tf_gl <= tb_q2(12 downto 0); tf_gh <= tb_q3(12 downto 0);
            tf_pix <= tb_q4(12 downto 0);
            l18_deh <= f_hyp(l17_hr);
            l18_px <= l17_px; l18_py <= l17_py;
            -- 19
            l19_es <= signed(resize(l18_px, 16)) - signed(resize(tf_gs, 16));
            l19_el <= signed(resize(l18_py, 16)) - signed(resize(tf_gl, 16));
            l19_eh <= signed(resize(l18_deh, 16)) - signed(resize(tf_gh, 16));
            l19_pix <= tf_pix;
            -- 20
            l20_m1 <= f_min(l19_es, l19_el); l20_eh <= l19_eh; l20_pix <= l19_pix;
            -- 21: the margin (capped) + half a pixel
            v_m := f_min(l20_m1, l20_eh);
            if v_m(15) = '0' and v_m(14 downto 12) /= "000" then v_m := to_signed(4095, 16); end if;
            l21_np <= v_m + signed(resize(l20_pix(12 downto 1), 16));
            l21_pix <= l20_pix;
            case to_integer(s_aa) is                           -- the line thickness: 4, 3, 2 pixels
                when 2      => l21_tk <= resize(l20_pix & "00", 15);
                when 3      => l21_tk <= resize(l20_pix & '0', 15) + resize(l20_pix, 15);
                when others => l21_tk <= resize(l20_pix & '0', 15);
            end case;
            -- 22: the coverage; Fine: also the coverage one line thickness further in
            l22_c1 <= f_cov4(l21_np, l21_pix);
            if s_fine = '1' then l22_c2 <= f_cov4(l21_np - signed(resize(l21_tk, 16)), l21_pix); else l22_c2 <= (others => '0'); end if;
        end if;
    end process p_leaf;

    ---------------------------------------------------------------------------
    -- [K] KAPPA, the camera element: the level-8 piece, or the level-7 piece while
    -- its children would be under 11.8 .. 26 px (phi - 64 - 8 h < 0).  A white
    -- similar triangle inside it, its inset set by the luma held in the pixel's
    -- block of the grid (read from the bank the source field is not writing).
    ---------------------------------------------------------------------------
    cg_ra <= (not s_bank) & k_row & k_bh(11 downto 6);

    p_kap : process(clk)
        variable v_b : signed(9 downto 0);
        variable v_h : unsigned(11 downto 0);
        variable v_ic : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- the pixel's block: the column steps with the beam, 18 stages behind it; rows of 33.75 lines
            s_bst <= s_st(7) & s_bst(0 to 16);
            if s_bst(16) = '1' then
                if s_xs = 4 then k_bh <= to_unsigned(480, 12); else k_bh <= (others => '0'); end if;
            else
                k_bh <= k_bh + resize(s_xs, 12);
            end if;
            if s_zone = '1' then
                k_seen <= '0'; k_racc <= (others => '0'); k_row <= (others => '0');
            elsif g_start = '1' then
                if k_seen = '1' then
                    if k_racc + resize(s_ys, 14) >= 8640 then
                        k_racc <= k_racc + resize(s_ys, 14) - 8640;
                        if k_row /= 31 then k_row <= k_row + 1; end if;
                    else
                        k_racc <= k_racc + resize(s_ys, 14);
                    end if;
                end if;
                k_seen <= '1';
            end if;
            -- 15
            v_b := signed(resize(c_phi, 10)) - 64 - signed(resize(hB(7) & "000", 10));
            k15_useg <= not v_b(9);
            k15_x7 <= cx(7)(10 downto 0); k15_y7 <= cy(7)(10 downto 0);
            -- 16
            k16_useg <= k15_useg; k16_x7 <= k15_x7; k16_y7 <= k15_y7;
            -- 17: the element's coordinates (1.10)
            if k16_useg = '1' then
                k17_x <= cx(8)(9 downto 0) & '0'; k17_y <= cy(8)(9 downto 0) & '0';
            else
                k17_x <= k16_x7; k17_y <= k16_y7;
            end if;
            k17_useg <= k16_useg;
            -- 18, 19, 20: its nearest edge
            v_h := to_unsigned(2047, 12) - resize(k17_x, 12) - resize(k17_y & '0', 12);
            k18_hr <= v_h(10 downto 0);
            if k17_x < k17_y then k18_mn <= k17_x; else k18_mn <= k17_y; end if;
            k19_kh <= f_hyp(k18_hr); k19_mn <= k18_mn;
            if k19_kh < k19_mn then k20_dk <= k19_kh; else k20_dk <= k19_mn; end if;
            k18_useg <= k17_useg; k19_useg <= k18_useg; k20_useg <= k19_useg;
            -- 19 the grid's register, 20 its fabric copy
            cg_f <= cg_q;
            -- 21: the margin to the splinter's edge: inset 392 - 24 luma (1.10)
            v_ic := resize(cg_f & "0000", 9) + resize(cg_f & "000", 9);
            k21_mi <= signed(resize(k20_dk, 16)) + signed(resize(v_ic, 16)) - 392;
            if k20_useg = '1' then k21_pix <= resize(c_pk8, 13); else k21_pix <= resize(c_pk7, 13); end if;
            if cg_f = 0 then k21_nz <= '0'; else k21_nz <= '1'; end if;
            -- 22: the coverage
            if k21_nz = '1' then k22_ci <= f_cov4(k21_mi, k21_pix); else k22_ci <= (others => '0'); end if;
        end if;
    end process p_kap;

    ---------------------------------------------------------------------------
    -- [T] TONE: two blends  x + ((y - x) c + 2) / 4, c = 0 .. 4
    --   Bold: pi's tone -> white (Ink: its inverse) by the splinter; black -> that by the kerf
    --   Fine: black -> the line by the outline; -> white (Ink: the inverse) by the splinter
    ---------------------------------------------------------------------------
    p_tone : process(clk)
        variable v_cs : unsigned(2 downto 0);
        variable v_g : signed(15 downto 0);
        variable v_tone : unsigned(9 downto 0);

        function f_mix(a : unsigned(9 downto 0); df : signed(10 downto 0); c : unsigned(2 downto 0)) return unsigned is
            variable v_p, v_s : signed(13 downto 0);
        begin
            if c(2) = '1' then v_p := shift_left(resize(df, 14), 2);
            else
                if c(1) = '1' then v_p := shift_left(resize(df, 14), 1); else v_p := (others => '0'); end if;
                if c(0) = '1' then v_p := v_p + resize(df, 14); end if;
            end if;
            v_s := signed(resize(a & "10", 14)) + v_p;            -- (4 a + 2 + c df) / 4
            return unsigned(v_s(11 downto 2));
        end function;
    begin
        if rising_edge(clk) then
            -- 23: the splinter clipped to pi; the tone (Fine: the line's tone), the first and second weights
            if k22_ci < l22_c1 then v_cs := k22_ci; else v_cs := l22_c1; end if;
            if s_fine = '1' then
                if s_ink = '1' then b23_a <= to_unsigned(940, 10); else b23_a <= f_ltone(l_tc(22)(2 downto 0)); end if;
                b23_c <= l22_c1 - l22_c2; b23_c2 <= v_cs;
            else
                if s_ink = '0' then b23_a <= f_tone(l_tc(22)(2 downto 0));
                elsif l_tc(22)(3) = '1' then b23_a <= to_unsigned(64, 10);
                else b23_a <= to_unsigned(940, 10); end if;
                b23_c <= v_cs; b23_c2 <= l22_c1;
            end if;
            -- 24, 25: the first blend
            if s_fine = '1' then
                b24_df <= signed(resize(b23_a, 11)) - 64; b24_a <= to_unsigned(64, 10);
            else
                if s_ink = '1' then v_g := to_signed(1004, 16) - signed(resize(b23_a & '0', 16));
                else v_g := to_signed(940, 16) - signed(resize(b23_a, 16)); end if;
                b24_df <= v_g(10 downto 0); b24_a <= b23_a;
            end if;
            b24_c <= b23_c; b24_c2 <= b23_c2;
            b25_v <= f_mix(b24_a, b24_df, b24_c); b25_c2 <= b24_c2;
            -- 26, 27: the second blend
            if s_fine = '1' then
                if s_ink = '1' then v_g := to_signed(1004, 16) - signed(resize(b25_v & '0', 16));
                else v_g := to_signed(940, 16) - signed(resize(b25_v, 16)); end if;
                b26_df <= v_g(10 downto 0); b26_a <= b25_v;
            else
                b26_df <= signed(resize(b25_v, 11)) - 64; b26_a <= to_unsigned(64, 10);
            end if;
            b26_c2 <= b25_c2;
            b27_y <= f_mix(b26_a, b26_df, b26_c2);
        end if;
    end process p_tone;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: luma [1 2 1]/4, luma 13/16 toward black, blanking
    -- gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd1 <= b27_y; s_yd2 <= s_yd1;
            s_ya <= resize(b27_y, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_yf <= v_y(11 downto 2);
            if s_yf > 64 then v_dl := s_yf - 64; else v_dl := (others => '0'); end if;
            s_yg <= s_yf - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if dl_q(2) = '1' then
                s_yo <= s_yg;
            else
                s_yo <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_yo);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= dl_q2(0);
    data_out.vsync_n <= dl_q2(1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= dl_q2(2);

end architecture splinter;
