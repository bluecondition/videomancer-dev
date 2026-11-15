-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Bauhaus v0.1.0 -- Kinetic.
--
-- A grid of shape tiles (Bauhaus geometric posters, generative "shape tile" grids): every cell holds one
-- primitive in two flat palette colours.
--
--   element  a cell of pitch P (cell boundary on the screen centre); state = its generation (Q10)
--   theta    = Spread * index - phase     index: Linear = i + j; Radial = max + min / 2 of (|i|, |j|)
--   state    = 1/2 + Shuffle * wave(theta)             (wave: sine / snap = dwell and quick flips)
--   motif    hash(i, j, floor(state)) picks the primitive, its orientation and the two colours;
--            the figure's size = min(1, 4 * distance of frac(state) from a re-roll): shrink, swap, grow
--   figures  curved: quarter disc, quarter target, half disc, disc / donut / dot / target
--            straight: diagonal half, half rectangle, bar, diamond / square
--            all = one anchor (corner, edge middle, centre of the figure box) + one radius:
--            disc R^2 - r^2 (rings from 2 r^2 - R^2 / 2 and R^2 / 4 - 4 r^2), diamond R - (dx + dy),
--            rectangle min(Rx - dx, Ry - dy); coverage = 1/2 + f * (cell px / 2R), 8 steps (rectangle: 1 + ...)
--   source   luma held per cell (smear along x, IIR down the lines, sampled mid-cell): level 0 = the hash
--            poster, 1 .. 13 = a disc / square growing dot -> quarter -> half -> full on the ground colour
--   colour   a 256-word blend table T(colour a, colour b, coverage) refilled every field
--
--   K1  Scale    cell pitch 38 .. 332 virtual lines (exponential, glided)
--   K2  Shape    figure inset: full-bleed .. 40 % of the cell with margins
--   K3  Mix      curved motifs .. straight motifs
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Spread   phase step per cell 0 .. 1/2 cycle (square law)
--   K6  Colour   the 16 shared palettes
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Gutters  Off / On (ground lines between the cells)
--   S10 Layout   Random / Mirrored (four-fold)
--   S11 Source   Video / Osc
--   P12 SHUFFLE  how many generations the wave spans: 0.44 (breathing only) .. 4.4 (square law, glided)
--
-- Chassis sections lifted from louver.vhd:
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-cell hold
--   [D] wave engine: line / pixel accumulators -> theta -> wave ROM -> state
--   [E] the look: hash, motif decode, distances, coverage, blend table
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 42) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.bauhaus_pkg.all;
use work.all;

architecture bauhaus of program_top is
    constant C_NP       : integer := 35;      -- pixel stages up to the colour register
    constant C_LAT      : integer := 42;      -- = processing_delay_clks = C_NP + 7
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob

    -- [A] virtual-1080 steps from the timing ID: y step per field line (Q8), x step in half virtual px
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
    -- field lines from the top to the screen centre (540 virtual lines)
    function f_half(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(121, 11);
            when 4          => return to_unsigned(240, 11);
            when 8          => return to_unsigned(144, 11);
            when 12         => return to_unsigned(288, 11);
            when 5 | 6 | 14 => return to_unsigned(360, 11);
            when 1 | 2 | 10 => return to_unsigned(270, 11);
            when others     => return to_unsigned(540, 11);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- gated operands: every multiply in this file is a sum of these
    function f_gu(x : unsigned; b : std_logic; w : natural) return unsigned is
        variable r : unsigned(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (registered address and data, then a fabric register)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(9 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_sqx_q, r_sqy_q : unsigned(15 downto 0) := (others => '0');
    signal r_r2_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    -- [C] per-cell luma hold: {held, filter} by cell column
    signal hl : t_r16 := (others => (others => '0'));
    attribute no_rw_check of hl : signal is true;
    signal hl_ra, hl_wa : unsigned(7 downto 0) := (others => '0');
    signal hl_q, hl_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal hl_we : std_logic := '0';
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    signal ur : t_r16 := (others => (others => '0'));          -- in-cell x: stage 3 -> 20
    attribute no_rw_check of ur : signal is true;
    signal ur_q : std_logic_vector(15 downto 0) := (others => '0');
    signal ir : t_r16 := (others => (others => '0'));          -- cell column for the hash: stage 4 -> 14
    attribute no_rw_check of ir : signal is true;
    signal ir_q : std_logic_vector(15 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] blend table, refilled every field: T(a, b, co) = colour a + (colour b - colour a) co / 8
    -- {Y 10, U 8, V 8}; address = a & b & co
    type t_tt is array (0 to 255) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(7 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_run : std_logic := '0';
    signal t_ph, t_co : unsigned(3 downto 0) := (others => '0');
    signal t_a, t_b : unsigned(1 downto 0) := (others => '0');
    signal t_ay : unsigned(9 downto 0) := (others => '0');
    signal t_ey : signed(13 downto 0) := (others => '0');
    signal t_dy : signed(10 downto 0) := (others => '0');
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    type t_s9  is array (1 to 2) of signed(8 downto 0);
    type t_s12 is array (1 to 2) of signed(11 downto 0);
    signal t_ac : t_u8 := (others => (others => '0'));
    signal t_ec : t_s12 := (others => (others => '0'));
    signal t_dc : t_s9 := (others => (others => '0'));
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_r4 := (others => (others => '1'));           -- syncs: C_LAT
    attribute no_rw_check of sr : signal is true;
    signal sr_q, sr_f : std_logic_vector(3 downto 0) := (others => '1');
    signal rc   : unsigned(7 downto 0) := (others => '0');

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(598 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(323 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_gut, s_mir, s_src, s_light : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier (24 iterations), serial shifter, sequencer scratch
    signal f_a, f_acc, z_fa : unsigned(47 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(23 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_en  : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : unsigned(1 downto 0) := (others => '0');
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_first : std_logic := '0';
    signal z_k2  : unsigned(15 downto 0) := (others => '0');

    -- per-field constants
    signal c_ip   : unsigned(22 downto 0) := to_unsigned(1988000, 23);  -- 2^-e, Q23
    signal c_pq   : unsigned(10 downto 0) := to_unsigned(540, 11);      -- pitch, Q2 virtual px
    signal c_du   : unsigned(15 downto 0) := to_unsigned(7766, 16);     -- cells per pixel, Q20
    signal c_dv   : unsigned(17 downto 0) := to_unsigned(7766, 18);     -- cells per field line, Q20
    signal c_x0, c_y0 : signed(26 downto 0) := (others => '0');         -- first pixel / line, cells Q20
    signal c_kq   : unsigned(7 downto 0) := to_unsigned(67, 8);         -- cell size in real px / 2
    signal c_g8   : unsigned(7 downto 0) := to_unsigned(49, 8);         -- gutter half-width, eighths of a px
    signal c_nc   : signed(6 downto 0) := (others => '0');              -- coverage scale exponents (quarter octaves)
    signal c_nl, c_nl1 : signed(6 downto 0) := (others => '0');
    signal c_b    : unsigned(6 downto 0) := to_unsigned(64, 7);         -- figure box / cell, 64 = 1
    signal c_m    : unsigned(9 downto 0) := (others => '0');            -- box edges, cell = 1024
    signal c_hi   : unsigned(9 downto 0) := (others => '1');
    signal c_thr  : unsigned(4 downto 0) := (others => '0');            -- straight motifs, sixteenths
    signal c_s    : unsigned(14 downto 0) := (others => '0');           -- spread per cell, Q16 cycles
    signal c_si0, c_sj0 : signed(23 downto 0) := (others => '0');       -- spread * first cell column / row
    signal c_d    : unsigned(7 downto 0) := to_unsigned(25, 8);         -- shuffle depth, generations / 32
    signal c_ph   : unsigned(25 downto 0) := (others => '0');           -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal c_gy   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_gc   : t_u8 := (others => to_unsigned(128, 8));

    -- [B] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start, g_start_d : std_logic := '0';

    -- [C] source
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_sv  : unsigned(7 downto 0) := (others => '0');
    signal s_sav, s_savd : std_logic := '0';
    signal s_y1  : unsigned(11 downto 0) := (others => '0');
    signal k_ia, k_i : unsigned(7 downto 0) := (others => '0');
    signal k_held, k_filt : unsigned(7 downto 0) := (others => '0');
    signal m_done : std_logic := '0';
    signal m_sr  : std_logic_vector(3 to 7) := (others => '0');
    signal s_cs  : std_logic_vector(3 to 14) := (others => '0');

    -- [D] wave engine: line registers and pixel accumulators
    signal ls_y, ls_yn, l_y : signed(26 downto 0) := (others => '0');
    signal ls_sj, ls_sjn, l_sj : signed(23 downto 0) := (others => '0');
    signal ls_cross, ls_rf, l_rf, l_first : std_logic := '0';
    signal l_jh   : unsigned(6 downto 0) := (others => '0');
    signal l_vp   : unsigned(9 downto 0) := (others => '0');
    signal l_t, l_sja : signed(23 downto 0) := (others => '0');
    signal l_an   : std_logic := '0';
    signal l_dy0, l_dy1, l_dy2 : unsigned(9 downto 0) := (others => '0');
    signal l_byd  : signed(10 downto 0) := (others => '0');
    signal l_gyd  : unsigned(8 downto 0) := (others => '0');
    signal l_t1, l_hp : unsigned(15 downto 0) := (others => '0');
    signal a_x    : signed(26 downto 0) := (others => '0');
    signal a_nw   : std_logic := '0';
    signal q2_si, q3_t, q4_sa : signed(23 downto 0) := (others => '0');
    signal q2_i   : signed(6 downto 0) := (others => '0');
    signal q2_u, q3_up : unsigned(9 downto 0) := (others => '0');
    signal q2_nw, q3_an, q5_ge : std_logic := '0';
    signal q3_ih, q4_ih, q14_ih : unsigned(6 downto 0) := (others => '0');
    signal q5_sa  : unsigned(16 downto 0) := (others => '0');
    signal q6_a, q6_b : unsigned(15 downto 0) := (others => '0');
    signal q7_th  : unsigned(13 downto 0) := (others => '0');
    signal s_neg  : std_logic_vector(8 to 13) := (others => '0');
    signal q10_w  : unsigned(9 downto 0) := (others => '0');
    signal q11_p0, q11_p1, q11_p2, q11_p3 : unsigned(11 downto 0) := (others => '0');
    signal q12_qa, q12_qb : unsigned(13 downto 0) := (others => '0');
    signal q13_aw : unsigned(12 downto 0) := (others => '0');
    signal q14_st : signed(14 downto 0) := (others => '0');

    -- [E] the look
    signal q15_h, q16_h, q17_h, q18_h : unsigned(17 downto 0) := (others => '0');
    signal q15_s, q16_sc : unsigned(7 downto 0) := (others => '0');
    signal q15_l, q16_lp, q17_lp, q18_lp : unsigned(3 downto 0) := (others => '0');
    signal q17_pa, q17_pb, q17_pc : unsigned(9 downto 0) := (others => '0');
    signal q17_pd : unsigned(7 downto 0) := (others => '0');
    signal q18_qa, q18_qb : unsigned(11 downto 0) := (others => '0');
    signal q19_rf : unsigned(10 downto 0) := (others => '0');
    signal q19_typ, q19_axc, q19_ayc, q19_rxs, q19_rys : unsigned(1 downto 0) := (others => '0');
    signal q19_ring, q19_core : std_logic := '0';
    type t_c4 is array (natural range <>) of unsigned(3 downto 0);
    signal s_col  : t_c4(19 to 32) := (others => (others => '0'));    -- c0 & c1
    signal s_typ  : t_c4(20 to 30) := (others => (others => '0'));    -- typ & ring & core
    signal s_zf   : std_logic_vector(20 to 31) := (others => '0');
    signal q20_up, q20_ax : unsigned(9 downto 0) := (others => '0');
    signal q20_ayc : unsigned(1 downto 0) := (others => '0');
    signal q20_rx, q20_ry, q21_rx, q21_ry : unsigned(10 downto 0) := (others => '0');
    signal q21_dx, q21_dy : unsigned(9 downto 0) := (others => '0');
    signal q21_rc : unsigned(7 downto 0) := (others => '0');
    signal q21_bxd : signed(10 downto 0) := (others => '0');
    signal q21_gxd : unsigned(8 downto 0) := (others => '0');
    signal q22_sxy : unsigned(10 downto 0) := (others => '0');
    signal q22_ex, q22_ey, q22_rx : signed(11 downto 0) := (others => '0');
    signal q22_ob : unsigned(6 downto 0) := (others => '0');
    signal q22_g  : unsigned(7 downto 0) := (others => '0');
    signal q23_sqx, q23_sqy, q23_r2, q24_r2 : unsigned(15 downto 0) := (others => '0');
    signal q23_qr : unsigned(5 downto 0) := (others => '0');
    signal q23_fl, q24_fl, q25_fl, q26_fl : signed(11 downto 0) := (others => '0');
    signal q23_b0, q23_b1, q23_b2, q23_b3 : unsigned(8 downto 0) := (others => '0');
    signal q23_g0, q23_g1, q23_g2, q23_g3 : unsigned(9 downto 0) := (others => '0');
    signal q24_bq, q24_br : unsigned(10 downto 0) := (others => '0');
    signal q24_gq, q24_gr : unsigned(11 downto 0) := (others => '0');
    signal q24_rs : unsigned(16 downto 0) := (others => '0');
    signal q24_n  : signed(6 downto 0) := (others => '0');
    signal s_cb   : t_c4(25 to 31) := (others => (others => '0'));
    signal s_cg   : t_c4(25 to 32) := (others => (others => '0'));
    signal q25_fa : signed(17 downto 0) := (others => '0');
    signal q25_fb : signed(18 downto 0) := (others => '0');
    signal q25_fc, q26_fc : signed(19 downto 0) := (others => '0');
    signal q26_m1 : signed(18 downto 0) := (others => '0');
    type t_n6 is array (natural range <>) of unsigned(5 downto 0);
    signal s_n    : t_n6(25 to 30) := (others => (others => '0'));
    signal q27_f  : signed(19 downto 0) := (others => '0');
    signal q28_x, q28_y, q29_t : signed(22 downto 0) := (others => '0');
    signal q30_x  : signed(19 downto 0) := (others => '0');
    signal q31_cov, q32_cov : unsigned(3 downto 0) := (others => '0');
    signal qf_y   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal qf_c   : t_u8 := (others => to_unsigned(128, 8));

    -- [F] output conditioning
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, hold, delay rings, blend table (one write statement each;
    -- none ever reads the address it is writing while the data is used)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q  <= C_BH_EXP2(to_integer(r_ex_a));
            r_w_q   <= C_BH_SIN(to_integer(r_w_a));
            r_sqx_q <= C_BH_SQX(to_integer(q21_dx));
            r_sqy_q <= C_BH_SQY(to_integer(q21_dy));
            r_r2_q  <= C_BH_R2(to_integer(q21_rc));
            r_pl_q  <= C_BH_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_hl : process(clk)
    begin
        if rising_edge(clk) then
            hl_q <= hl(to_integer(hl_ra));
            if hl_we = '1' then hl(to_integer(hl_wa)) <= hl_wd; end if;
        end if;
    end process p_hl;

    p_tt : process(clk)
    begin
        if rising_edge(clk) then
            tt_q <= tt(to_integer(tt_ra));
            if tt_we = '1' then tt(to_integer(tt_wa)) <= tt_wd; end if;
        end if;
    end process p_tt;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            ur(to_integer(rc + 15)) <= "000000" & std_logic_vector(q3_up);
            ur_q <= ur(to_integer(rc));
            ir(to_integer(rc + 8)) <= "000000000" & std_logic_vector(q4_ih);
            ir_q <= ir(to_integer(rc));
            cr(to_integer(rc + 6)) <= std_logic_vector(qf_c(1)) & std_logic_vector(qf_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- [A] Controls, vblank zone, glides, per-field sequencer (one small
    -- operation per zone clock; quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_ip3 : unsigned(24 downto 0);
        variable v_k  : unsigned(10 downto 0);
        variable v_q  : signed(6 downto 0);
        variable v_i  : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- vblank zone: no avid for 1024 clocks
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

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- Rate: bipolar with a +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;     -- |v| - 8
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_snap <= registers_in(6)(0);
                s_rad  <= registers_in(6)(1);
                s_gut  <= registers_in(6)(2);
                s_mir  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(5)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(5)(9 downto 6)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide for Scale and Shuffle (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 1 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" and s_zc(8 downto 5) < 11 and s_zc(8 downto 5) /= 9 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(801, 24);       -- pitch exponent
                    when 1  =>                                                                                -- spread^2
                        z_fa(9 downto 0) <= unsigned(registers_in(4)); z_fb(9 downto 0) <= unsigned(registers_in(4));
                    when 2  => z_fa(22 downto 0) <= c_ip;                 z_fb <= resize(s_ys, 24);           -- cells per line
                    when 3  => z_fa(15 downto 0) <= c_du;                 z_fb <= resize(s_w(11 downto 1) - 4, 24);   -- first pixel
                    when 4  => z_fa(17 downto 0) <= c_dv;                 z_fb <= resize(f_half(s_tid), 24);  -- first line
                    when 5  =>                                                                                -- spread * first column
                        v_i := 0 - c_x0(26 downto 20);
                        z_fa(14 downto 0) <= c_s;                         z_fb <= resize(unsigned(v_i), 24);
                    when 6  =>                                                                                -- spread * first row
                        v_i := 0 - c_y0(26 downto 20);
                        z_fa(14 downto 0) <= c_s;                         z_fb <= resize(unsigned(v_i), 24);
                    when 7  => z_fa(12 downto 0) <= s_gp;                 z_fb <= resize(s_gp, 24);           -- shuffle^2
                    when 8  => z_fa(8 downto 0)  <= s_vm;                 z_fb <= to_unsigned(900, 24);       -- rate exponent
                    when others => z_fa(15 downto 0) <= z_mt;             z_fb <= to_unsigned(1193046, 24);   -- rate (10)
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(46 downto 0) & '0';
                f_b <= '0' & f_b(23 downto 1);
            end if;

            -- serial shifter for the 2^n scalings (no barrels)
            if z_shn /= 0 then
                z_shn <= z_shn - 1;
                case z_sht is
                    when "01"   => c_ip  <= c_ip(21 downto 0) & '0';
                    when others => c_dph <= c_dph(20 downto 0) & '0';
                end case;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- wave phase (last field's step): one adder, add or subtract
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    -- Shape: figure box 64 .. 26 sixty-fourths of the cell; Mix threshold
                    when 21 => z_k2 <= resize(unsigned(registers_in(1)) & "00000", 16) + resize(unsigned(registers_in(1)) & "000", 16);
                    when 22 => z_k2 <= z_k2 - resize(unsigned(registers_in(1)), 16);
                    when 23 => c_b <= to_unsigned(64, 7) - resize(z_k2(15 downto 10), 7);
                    when 24 => c_m <= to_unsigned(512, 10) - shift_left(resize(c_b, 10), 3);
                    when 25 => c_hi <= not c_m;
                    when 26 => z_k2 <= resize(unsigned(registers_in(2)) & "0000", 16) + resize(unsigned(registers_in(2)), 16);
                    when 27 => c_thr <= z_k2(14 downto 10);

                    -- pitch: e = 0.25 + k1 * 3.13 octaves (Q8), P = 32 * 2^e, 2^-e
                    when 60 => z_e <= to_unsigned(64, 11) + resize(f_acc(23 downto 13), 11);
                    when 61 => z_en <= 0 - z_e;                 -- 2048 - e
                    when 62 => r_ex_a <= z_en(7 downto 0); z_sh <= z_en(10 downto 8);
                    when 64 => z_mt <= r_ex_q;
                    when 65 =>
                        c_ip <= resize(z_mt, 23); z_shn <= z_sh; z_sht <= "01";
                        r_ex_a <= z_e(7 downto 0);
                    when 67 => z_mt <= r_ex_q;
                    when 68 =>
                        case z_e(9 downto 8) is
                            when "00"   => c_pq <= resize(z_mt(15 downto 8), 11);
                            when "01"   => c_pq <= resize(z_mt(15 downto 7), 11);
                            when "10"   => c_pq <= resize(z_mt(15 downto 6), 11);
                            when others => c_pq <= z_mt(15 downto 5);
                        end case;
                    when 69 =>
                        case to_integer(s_xs) is
                            when 2      => v_k := shift_right(c_pq, 3);
                            when 4      => v_k := shift_right(c_pq, 4);
                            when others => v_k := shift_right(c_pq, 4) + shift_right(c_pq, 6) + shift_right(c_pq, 8);
                        end case;
                        c_kq <= v_k(7 downto 0);
                        case to_integer(s_xs) is
                            when 2      => v_q := to_signed(4, 7);
                            when 4      => v_q := to_signed(8, 7);
                            when others => v_q := to_signed(6, 7);
                        end case;
                        c_nc <= v_q - signed(resize(z_e(10 downto 6), 7));
                    when 70 =>
                        c_g8 <= to_unsigned(16, 8) + c_kq;
                        c_nl <= c_nc + 32; c_nl1 <= c_nc + 34;
                    -- cells per pixel = 2^-e * xs / 64 (after the shifter has finished)
                    when 80 =>
                        case to_integer(s_xs) is
                            when 2      => c_du <= resize(c_ip(22 downto 8), 16);
                            when 4      => c_du <= c_ip(22 downto 7);
                            when others =>
                                v_ip3 := resize(c_ip & '0', 25) + resize(c_ip, 25);
                                c_du <= v_ip3(24 downto 9);
                        end case;
                    when 92  => c_s <= f_acc(19 downto 5);
                    when 124 => c_dv <= f_acc(33 downto 16);
                    when 156 => c_x0 <= 0 - signed(f_acc(26 downto 0));
                    when 188 => c_y0 <= 0 - signed(f_acc(26 downto 0));
                    when 220 => c_si0 <= 0 - signed(f_acc(23 downto 0));
                    when 252 => c_sj0 <= 0 - signed(f_acc(23 downto 0));
                    when 284 => c_d <= to_unsigned(14, 8) + f_acc(26 downto 19);
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 316 => z_e <= f_acc(18 downto 8);
                    when 317 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 319 => z_mt <= r_ex_q;
                    when 380 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh; z_sht <= "11";
                    when 620 => z_first <= '1';
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            if g_start = '1' then z_first <= '0'; end if;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Blend table fill (zone, zc 28 .. ~260; adders only).  For colours a, b
    -- and co = 0 .. 8:  T = (8 a + (b - a) co) / 8.  Colour a = 0 also loads the
    -- ground registers and the palette's light-ground flag.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            if s_zone = '1' and s_zc = 28 then
                t_run <= '1'; t_ph <= (others => '0'); t_a <= "00"; t_b <= "00";
            elsif t_run = '1' then
                t_ph <= t_ph + 1;
                case to_integer(t_ph) is
                    when 0 => r_pl_a <= "00" & s_pal & t_a;
                    when 1 => r_pl_a <= "00" & s_pal & t_b;
                    when 2 =>
                        t_ay <= unsigned(r_pl_q(31 downto 22));
                        t_ac(1) <= unsigned(r_pl_q(21 downto 14)); t_ac(2) <= unsigned(r_pl_q(11 downto 4));
                        if t_a = 0 then
                            c_gy <= unsigned(r_pl_q(31 downto 22));
                            c_gc(1) <= unsigned(r_pl_q(21 downto 14)); c_gc(2) <= unsigned(r_pl_q(11 downto 4));
                            s_light <= r_pl_q(0);
                        end if;
                    when 3 =>
                        t_dy <= signed(resize(unsigned(r_pl_q(31 downto 22)), 11)) - signed(resize(t_ay, 11));
                        t_dc(1) <= signed(resize(unsigned(r_pl_q(21 downto 14)), 9)) - signed(resize(t_ac(1), 9));
                        t_dc(2) <= signed(resize(unsigned(r_pl_q(11 downto 4)), 9)) - signed(resize(t_ac(2), 9));
                        t_ey <= signed(resize(t_ay & "000", 14));
                        for i in 1 to 2 loop t_ec(i) <= signed(resize(t_ac(i) & "000", 12)); end loop;
                        t_co <= (others => '0');
                    when 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 =>
                        tt_we <= '1'; tt_wa <= t_a & t_b & t_co;
                        tt_wd <= "000000" & std_logic_vector(t_ey(12 downto 3)) & std_logic_vector(t_ec(1)(10 downto 3)) & std_logic_vector(t_ec(2)(10 downto 3));
                        t_ey <= t_ey + resize(t_dy, 14);
                        for i in 1 to 2 loop t_ec(i) <= t_ec(i) + resize(t_dc(i), 12); end loop;
                        t_co <= t_co + 1;
                    when 13 =>
                        t_ph <= (others => '0');
                        t_b <= t_b + 1;
                        if t_b = 3 then
                            t_a <= t_a + 1;
                            if t_a = 3 then t_run <= '0'; end if;
                        end if;
                    when others => t_ph <= (others => '0');
                end case;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [B] Generated avid (parity-locked start 3/4 clocks after the source's
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
            g_start_d <= g_start;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- [C] Source field.  Osc: a two-axis triangle lattice (640 x 540 virtual
    -- px) drifting diagonally.  The luma (or the lattice) is smeared along x
    -- (1/16) and held per cell: one sample per cell per line at mid-cell goes
    -- through a 1/8 filter down the lines in a RAM word per cell column
    -- {held, filter}; a cell row uses the filter value of the line before it
    -- (the field's first line: the smear at the cell start), so every cell is
    -- rigid.  Stage 4: the word of the cell starting here; stage 7: the write.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_x  : unsigned(11 downto 0);
        variable v_d1 : signed(12 downto 0);
        variable v_cs, v_md : std_logic;
        variable v_df : signed(8 downto 0);
        variable v_fn : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_tx := f_tri7(s_ox(15 downto 8)); v_ty := f_tri7(s_oy(15 downto 8));
            v_s := resize(v_tx, 8) + resize(v_ty, 8);
            v_o := v_s - shift_right(v_s, 3);

            -- input sample (8 bits), held through blanking
            s_sav <= data_in.avid; s_savd <= s_sav;
            if data_in.avid = '1' then
                if s_src = '1' then
                    s_sv <= v_o;
                else
                    v_l := unsigned(data_in.y(9 downto 2));
                    if v_l > 16 then s_sv <= v_l - 16; else s_sv <= (others => '0'); end if;
                end if;
            end if;

            -- smear along x (Q4)
            v_x := s_sv & "0000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
            elsif s_sav = '1' then
                v_d1 := signed(resize(v_x, 13)) - signed(resize(s_y1, 13));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 4), 12));
            end if;

            -- per-cell hold.  cs = the stage-2 pixel starts a cell (or the line); md = its mid-cell sample point
            if s_gav_sr(0) = '1' and (q2_nw = '1' or s_gav_sr(1) = '0') then v_cs := '1'; else v_cs := '0'; end if;
            if s_gav_sr(0) = '1' and q2_u(9) = '1' and (m_done = '0' or v_cs = '1') then v_md := '1'; else v_md := '0'; end if;
            if v_md = '1' then m_done <= '1'; elsif v_cs = '1' then m_done <= '0'; end if;
            s_cs(3) <= v_cs;
            for i in 4 to 14 loop s_cs(i) <= s_cs(i - 1); end loop;
            m_sr(3) <= v_md;
            for i in 4 to 7 loop m_sr(i) <= m_sr(i - 1); end loop;
            hl_ra <= '0' & unsigned(q2_i);
            k_ia <= hl_ra;
            if s_cs(4) = '1' then
                k_i <= k_ia;
                k_filt <= unsigned(hl_q(7 downto 0));
                if l_first = '1' then k_held <= s_y1(11 downto 4);
                elsif l_rf = '1' then k_held <= unsigned(hl_q(7 downto 0));
                else k_held <= unsigned(hl_q(15 downto 8)); end if;
            end if;
            v_df := signed(resize(s_y1(11 downto 4), 9)) - signed(resize(k_filt, 9));
            if l_first = '1' then v_fn := s_y1(11 downto 4);
            else v_fn := unsigned(signed(k_filt) + resize(shift_right(v_df, 3), 8)); end if;
            hl_wd <= std_logic_vector(k_held) & std_logic_vector(v_fn);
            hl_wa <= k_i;
            hl_we <= m_sr(7);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Wave engine.  x and y are accumulators in cells (Q20, 0 = the screen
    -- centre): integer part = cell column / row, fraction = the in-cell
    -- coordinate.  Spread * column steps by one adder at each cell crossing.
    -- ls_* hold the NEXT line's values; l_* are this line's (stable).
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_x  : signed(26 downto 0);
        variable v_n  : std_logic;
        variable v_ph : unsigned(15 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_ab : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- line registers
            ls_yn <= ls_y + signed(resize(c_dv, 27));
            if ls_yn(20) /= ls_y(20) then ls_cross <= '1'; else ls_cross <= '0'; end if;
            ls_sjn <= ls_sj + signed(resize(c_s, 24));
            if s_zone = '1' and s_zc = 400 then
                ls_y <= c_y0; ls_sj <= c_sj0; ls_rf <= '1';
            elsif g_start = '1' then
                l_y <= ls_y; l_sj <= ls_sj; l_rf <= ls_rf; l_first <= z_first;
                ls_y <= ls_yn; ls_rf <= ls_cross;
                if ls_cross = '1' then ls_sj <= ls_sjn; end if;
            end if;
            -- this line's row values (settled before the line's first pixel reaches the stages that read them)
            v_n := l_y(26);
            if v_n = '1' and s_mir = '1' then
                l_jh <= unsigned(not l_y(26 downto 20)); l_vp <= unsigned(not l_y(19 downto 10));
            else
                l_jh <= unsigned(l_y(26 downto 20)); l_vp <= unsigned(l_y(19 downto 10));
            end if;
            if v_n = '1' and (s_mir = '1' or s_rad = '1') then
                l_t <= l_sj + signed(resize(c_s, 24)); l_an <= '1';
            else
                l_t <= l_sj; l_an <= '0';
            end if;
            if l_an = '1' then l_sja <= 0 - l_t; else l_sja <= l_t; end if;
            v_d := signed(resize(l_vp, 11)) - signed(resize(c_m, 11));
            v_ab := unsigned(abs(v_d)); l_dy0 <= v_ab(9 downto 0);
            v_d := signed(resize(l_vp, 11)) - 512;
            v_ab := unsigned(abs(v_d)); l_dy1 <= v_ab(9 downto 0);
            v_d := signed(resize(l_vp, 11)) - signed(resize(c_hi, 11));
            v_ab := unsigned(abs(v_d)); l_dy2 <= v_ab(9 downto 0);
            if l_vp(9) = '0' then
                l_byd <= signed(resize(l_vp, 11)) - signed(resize(c_m, 11));
                l_gyd <= l_vp(8 downto 0);
            else
                l_byd <= signed(resize(c_hi, 11)) - signed(resize(l_vp, 11));
                l_gyd <= not l_vp(8 downto 0);
            end if;
            v_ph := c_ph(25 downto 10);
            l_t1 <= unsigned(l_sja(15 downto 0)) - v_ph;
            l_hp <= unsigned(l_sja(16 downto 1)) - v_ph;

            -- 1: the pixel accumulator; a_nw = this pixel starts a new cell
            v_x := a_x + signed(resize(c_du, 27));
            if g_start = '1' then
                a_x <= c_x0; a_nw <= '0';
            elsif g_avid = '1' then
                a_x <= v_x;
                a_nw <= v_x(20) xor a_x(20);
            else
                a_nw <= '0';
            end if;
            -- 2: Spread * column, column, in-cell x
            if g_start_d = '1' then
                q2_si <= c_si0;
            elsif a_nw = '1' then
                q2_si <= q2_si + signed(resize(c_s, 24));
            end if;
            q2_i <= a_x(26 downto 20); q2_u <= unsigned(a_x(19 downto 10)); q2_nw <= a_nw;
        end if;
    end process p_acc;

    p_wave : process(clk)
        variable v_n   : std_logic;
        variable v_pos : unsigned(11 downto 0);
        variable v_p   : unsigned(17 downto 0);
        variable v_th  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: mirror / magnitude selects
            v_n := q2_i(6);
            if v_n = '1' and s_mir = '1' then
                q3_ih <= unsigned(not q2_i); q3_up <= not q2_u;
            else
                q3_ih <= unsigned(q2_i); q3_up <= q2_u;
            end if;
            if v_n = '1' and (s_mir = '1' or s_rad = '1') then
                q3_t <= q2_si + signed(resize(c_s, 24)); q3_an <= '1';
            else
                q3_t <= q2_si; q3_an <= '0';
            end if;
            -- 4: Spread * |column|
            if q3_an = '1' then q4_sa <= 0 - q3_t; else q4_sa <= q3_t; end if;
            q4_ih <= q3_ih;
            -- 5: compare with the row term
            if q4_sa >= l_sja then q5_ge <= '1'; else q5_ge <= '0'; end if;
            q5_sa <= unsigned(q4_sa(16 downto 0));
            -- 6: Linear = column + row; Radial = max + min / 2; the phase rides on the row terms
            if s_rad = '0' then
                q6_a <= q5_sa(15 downto 0) + l_t1; q6_b <= (others => '0');
            elsif q5_ge = '1' then
                q6_a <= q5_sa(15 downto 0); q6_b <= l_hp;
            else
                q6_a <= l_t1; q6_b <= q5_sa(16 downto 1);
            end if;
            -- 7: theta (14 bits per cycle)
            v_th := q6_a + q6_b;
            q7_th <= v_th(15 downto 2);
            -- 8: wave ROM address: quarter-wave fold; Snap = the first sixteenth stretched over the quarter
            v_pos := q7_th(11 downto 0);
            if q7_th(12) = '1' then v_pos := not v_pos; end if;
            if s_snap = '0' then r_w_a <= v_pos(11 downto 2);
            elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
            else r_w_a <= v_pos(9 downto 0); end if;
            s_neg(8) <= q7_th(13);
            for i in 9 to 13 loop s_neg(i) <= s_neg(i - 1); end loop;
            -- 9: ROM; 10: fabric copy
            q10_w <= r_w_q(9 downto 0);
            -- 11 .. 13: Shuffle * wave as gated sums (generations Q10)
            q11_p0 <= f_gu(q10_w, c_d(0), 12) + shift_left(f_gu(q10_w, c_d(1), 12), 1);
            q11_p1 <= f_gu(q10_w, c_d(2), 12) + shift_left(f_gu(q10_w, c_d(3), 12), 1);
            q11_p2 <= f_gu(q10_w, c_d(4), 12) + shift_left(f_gu(q10_w, c_d(5), 12), 1);
            q11_p3 <= f_gu(q10_w, c_d(6), 12) + shift_left(f_gu(q10_w, c_d(7), 12), 1);
            q12_qa <= resize(q11_p0, 14) + shift_left(resize(q11_p1, 14), 2);
            q12_qb <= resize(q11_p2, 14) + shift_left(resize(q11_p3, 14), 2);
            v_p := resize(q12_qa, 18) + shift_left(resize(q12_qb, 18), 4);
            q13_aw <= v_p(17 downto 5);
            -- 14: the state = 1/2 +- Shuffle * wave: generation = bits 13 .. 10, fraction = bits 9 .. 0
            if s_neg(13) = '1' then q14_st <= to_signed(512, 15) - signed(resize(q13_aw, 15));
            else q14_st <= to_signed(512, 15) + signed(resize(q13_aw, 15)); end if;
            q14_ih <= unsigned(ir_q(6 downto 0));
        end if;
    end process p_wave;

    ---------------------------------------------------------------------------
    -- [E] The look
    ---------------------------------------------------------------------------
    p_look : process(clk)
        variable v_x, v_h : unsigned(17 downto 0);
        variable v_fr  : unsigned(8 downto 0);
        variable v_k5  : unsigned(10 downto 0);
        variable v_l   : unsigned(4 downto 0);
        variable v_lp  : unsigned(3 downto 0);
        variable v_cap : unsigned(7 downto 0);
        variable v_p15 : unsigned(14 downto 0);
        variable v_hm  : unsigned(3 downto 0);
        variable v_t, v_o, v_r, v_rr : unsigned(1 downto 0);
        variable v_cs  : unsigned(2 downto 0);
        variable v_st  : std_logic;
        variable v_cax, v_cay, v_eax, v_eay : unsigned(1 downto 0);
        variable v_typ, v_ax, v_ay, v_rx, v_ry, v_c0, v_c1, v_acl : unsigned(1 downto 0);
        variable v_ring, v_core : std_logic;
        variable v_d   : signed(10 downto 0);
        variable v_ab  : unsigned(10 downto 0);
        variable v_rs  : unsigned(10 downto 0);
        variable v_fb  : signed(10 downto 0);
        variable v_g   : unsigned(8 downto 0);
        variable v_bp  : unsigned(14 downto 0);
        variable v_gp  : unsigned(15 downto 0);
        variable v_cg  : signed(11 downto 0);
        variable v_n7  : signed(6 downto 0);
        variable v_m19 : signed(18 downto 0);
        variable v_f20 : signed(19 downto 0);
        variable v_s20 : signed(19 downto 0);
        variable v_cv  : unsigned(3 downto 0);
        variable v_gz  : std_logic;
    begin
        if rising_edge(clk) then
            -- 15: hash round 1 over {generation, row, column}; the figure's size from the state's fraction
            v_x := unsigned(q14_st(13 downto 10)) & l_jh & q14_ih;
            q15_h <= (v_x xor to_unsigned(16#15A5B#, 18)) + shift_left(v_x, 3);
            if q14_st(9) = '1' then v_fr := unsigned(not q14_st(8 downto 0)); else v_fr := unsigned(q14_st(8 downto 0)); end if;
            if v_fr(8 downto 1) > 128 then q15_s <= to_unsigned(128, 8); else q15_s <= v_fr(8 downto 1); end if;
            -- the cell's source level (0 .. 15), latched as the cell start passes
            if s_cs(14) = '1' then
                v_k5 := resize(k_held & "00", 11) + resize(k_held, 11);
                v_l := v_k5(10 downto 6);
                if v_l > 15 then q15_l <= to_unsigned(15, 4); else q15_l <= v_l(3 downto 0); end if;
            end if;
            -- 16 .. 18: hash rounds 2 .. 4; the level by palette polarity (0 = no source) and its size cap
            v_x := q15_h xor shift_right(q15_h, 7);
            q16_h <= v_x + shift_left(v_x, 5);
            v_x := q16_h xor shift_right(q16_h, 9);
            q17_h <= v_x + shift_left(v_x, 2);
            v_x := q17_h xor shift_right(q17_h, 6);
            q18_h <= v_x + shift_left(v_x, 7);
            if q15_l = 0 then
                v_lp := (others => '0');
            else
                if s_light = '1' then v_lp := 14 - q15_l; else v_lp := q15_l; end if;   -- (15 -> 15: clamped below)
                if q15_l > 13 then
                    if s_light = '1' then v_lp := to_unsigned(1, 4); else v_lp := to_unsigned(13, 4); end if;
                end if;
            end if;
            q16_lp <= v_lp; q17_lp <= q16_lp; q18_lp <= q17_lp;
            case to_integer(v_lp) is
                when 1 | 7 | 10 => v_cap := to_unsigned(64, 8);
                when 2 | 9      => v_cap := to_unsigned(96, 8);
                when 4 | 11     => v_cap := to_unsigned(72, 8);
                when 5 | 8 | 12 => v_cap := to_unsigned(80, 8);
                when 6          => v_cap := to_unsigned(92, 8);
                when others     => v_cap := to_unsigned(128, 8);
            end case;
            if q15_s < v_cap then q16_sc <= q15_s; else q16_sc <= v_cap; end if;
            -- 17 .. 19: size * box (gated sums): the figure's full radius, cell = 1024
            q17_pa <= f_gu(q16_sc, c_b(0), 10) + shift_left(f_gu(q16_sc, c_b(1), 10), 1);
            q17_pb <= f_gu(q16_sc, c_b(2), 10) + shift_left(f_gu(q16_sc, c_b(3), 10), 1);
            q17_pc <= f_gu(q16_sc, c_b(4), 10) + shift_left(f_gu(q16_sc, c_b(5), 10), 1);
            q17_pd <= f_gu(q16_sc, c_b(6), 8);
            q18_qa <= resize(q17_pa, 12) + shift_left(resize(q17_pb, 12), 2);
            q18_qb <= resize(q17_pc, 12) + shift_left(resize(q17_pd, 12), 2);
            v_p15 := resize(q18_qa, 15) + shift_left(resize(q18_qb, 15), 4);
            q19_rf <= v_p15(13 downto 3);

            -- 19: motif decode
            v_h := q18_h xor shift_right(q18_h, 8);
            v_hm := v_h(17 downto 14); v_t := v_h(13 downto 12); v_o := v_h(11 downto 10);
            v_cs := v_h(9 downto 7); v_r := v_h(6 downto 5);
            if resize(v_hm, 5) < c_thr then v_st := '1'; else v_st := '0'; end if;
            v_cax := v_o(0) & '0'; v_cay := v_o(1) & '0';                       -- corner o
            if v_o(1) = '0' then v_eax := v_o(0) & '0'; v_eay := "01";         -- edge o
            else v_eax := "01"; v_eay := v_o(0) & '0'; end if;
            if v_r = 3 then v_rr := "00"; else v_rr := v_r; end if;
            if q18_lp = 0 then
                -- the hash poster
                v_ring := '0'; v_core := '0';
                if v_st = '1' then
                    if v_t = 0 or (v_t = 3 and v_o(0) = '0') then v_typ := "01"; else v_typ := "10"; end if;
                else
                    v_typ := "00";
                    if v_t = 1 or (v_t = 3 and v_o(0) = '1') then v_ring := '1'; end if;
                    if v_t = 1 or (v_t = 3 and v_o = 3) then v_core := '1'; end if;
                end if;
                case to_integer(v_t) is
                    when 0      => v_ax := v_cax; v_ay := v_cay;
                    when 1      => if v_st = '1' then v_ax := v_eax; v_ay := v_eay; else v_ax := v_cax; v_ay := v_cay; end if;
                    when 2      => if v_st = '1' then v_ax := "01"; v_ay := "01"; else v_ax := v_eax; v_ay := v_eay; end if;
                    when others => v_ax := "01"; v_ay := "01";
                end case;
                if v_st = '0' then
                    if v_t(1) = '0' then v_rx := "00";
                    elsif v_t = 3 and v_o = 2 then v_rx := "10";
                    else v_rx := "01"; end if;
                    v_ry := v_rx;
                else
                    case to_integer(v_t) is
                        when 0      => v_rx := "00"; v_ry := "00";
                        when 1      => v_rx := "01"; v_ry := "01";
                        when 2      =>
                            if v_o(0) = '0' then v_rx := "01"; v_ry := '1' & (not v_o(1));
                            else v_rx := '1' & (not v_o(1)); v_ry := "01"; end if;
                        when others =>
                            if v_o(0) = '0' then v_rx := "01"; else v_rx := "10"; end if;
                            v_ry := v_rx;
                    end case;
                end if;
                if v_cs(2) = '0' then v_c0 := "00";
                elsif v_cs(1) = '0' then v_c0 := "01";
                else v_c0 := '1' & v_cs(0); end if;
                v_c1 := v_c0 + 1 + v_rr;
            else
                -- the source halftone: a disc or a square on the ground colour, growing with the level
                v_ring := '0'; v_core := '0';
                if v_st = '1' then v_typ := "10"; else v_typ := "00"; end if;
                case to_integer(q18_lp) is
                    when 4 | 5 | 6 => v_acl := "01";
                    when 7 | 8 | 9 => v_acl := "10";
                    when others    => v_acl := "00";
                end case;
                if v_acl = 0 then v_ax := "01"; v_ay := "01";
                elsif v_acl = 1 then v_ax := v_cax; v_ay := v_cay;
                else v_ax := v_eax; v_ay := v_eay; end if;
                if q18_lp < 4 then v_rx := "10"; else v_rx := "00"; end if;
                v_ry := v_rx;
                v_c0 := "00";
                v_c1 := v_rr + 1;
            end if;
            q19_typ <= v_typ; q19_ring <= v_ring; q19_core <= v_core;
            q19_axc <= v_ax; q19_ayc <= v_ay; q19_rxs <= v_rx; q19_rys <= v_ry;
            s_col(19) <= v_c0 & v_c1;
            for i in 20 to 32 loop s_col(i) <= s_col(i - 1); end loop;

            -- 20: radii, the x anchor
            q20_up <= unsigned(ur_q(9 downto 0));
            case q19_rxs is
                when "00"   => v_rs := q19_rf;
                when "01"   => v_rs := shift_right(q19_rf, 1);
                when "10"   => v_rs := shift_right(q19_rf, 2);
                when others => v_rs := shift_right(q19_rf, 3);
            end case;
            q20_rx <= v_rs;
            case q19_rys is
                when "00"   => v_rs := q19_rf;
                when "01"   => v_rs := shift_right(q19_rf, 1);
                when "10"   => v_rs := shift_right(q19_rf, 2);
                when others => v_rs := shift_right(q19_rf, 3);
            end case;
            q20_ry <= v_rs;
            case q19_axc is
                when "00"   => q20_ax <= c_m;
                when "01"   => q20_ax <= to_unsigned(512, 10);
                when others => q20_ax <= c_hi;
            end case;
            q20_ayc <= q19_ayc;
            s_typ(20) <= q19_typ & q19_ring & q19_core;
            for i in 21 to 30 loop s_typ(i) <= s_typ(i - 1); end loop;
            if q19_rf(10 downto 5) = 0 then s_zf(20) <= '1'; else s_zf(20) <= '0'; end if;
            for i in 21 to 31 loop s_zf(i) <= s_zf(i - 1); end loop;

            -- 21: |dx|, |dy| (= the square ROMs' addresses), the disc radius (ROM address), box and gutter x distances
            v_d := signed(resize(q20_up, 11)) - signed(resize(q20_ax, 11));
            v_ab := unsigned(abs(v_d));
            q21_dx <= v_ab(9 downto 0);
            case q20_ayc is
                when "00"   => q21_dy <= l_dy0;
                when "01"   => q21_dy <= l_dy1;
                when others => q21_dy <= l_dy2;
            end case;
            if q20_rx(10) = '1' then q21_rc <= (others => '1'); else q21_rc <= q20_rx(9 downto 2); end if;
            q21_rx <= q20_rx; q21_ry <= q20_ry;
            if q20_up(9) = '0' then
                q21_bxd <= signed(resize(q20_up, 11)) - signed(resize(c_m, 11));
                q21_gxd <= q20_up(8 downto 0);
            else
                q21_bxd <= signed(resize(c_hi, 11)) - signed(resize(q20_up, 11));
                q21_gxd <= not q20_up(8 downto 0);
            end if;

            -- 22: (ROMs) the straight figures' terms; outside-the-box distance; gutter distance
            q22_sxy <= resize(q21_dx, 11) + resize(q21_dy, 11);
            q22_ex <= signed(resize(q21_rx, 12)) - signed(resize(q21_dx, 12));
            q22_ey <= signed(resize(q21_ry, 12)) - signed(resize(q21_dy, 12));
            q22_rx <= signed(resize(q21_rx, 12));
            if q21_bxd < l_byd then v_fb := q21_bxd; else v_fb := l_byd; end if;
            if v_fb(10) = '0' then q22_ob <= (others => '0');
            elsif v_fb < -127 then q22_ob <= (others => '1');
            else v_ab := unsigned(0 - v_fb); q22_ob <= v_ab(6 downto 0); end if;
            if q21_gxd < l_gyd then v_g := q21_gxd; else v_g := l_gyd; end if;
            if v_g(8) = '1' then q22_g <= (others => '1'); else q22_g <= v_g(7 downto 0); end if;

            -- 23: fabric copies of the ROM words; diamond R - (dx + dy) or rectangle min(Rx - dx, Ry - dy)
            q23_sqx <= r_sqx_q; q23_sqy <= r_sqy_q;
            q23_r2 <= unsigned(r_r2_q(31 downto 16)); q23_qr <= unsigned(r_r2_q(5 downto 0));
            if s_typ(22)(2) = '1' then q23_fl <= q22_rx - signed(resize(q22_sxy, 12));
            elsif q22_ex < q22_ey then q23_fl <= q22_ex;
            else q23_fl <= q22_ey; end if;
            q24_fl <= q23_fl; q25_fl <= q24_fl; q26_fl <= q25_fl;
            -- 23 .. 25: box and gutter coverages = distance * cell px (gated sums)
            q23_b0 <= f_gu(q22_ob, c_kq(0), 9) + shift_left(f_gu(q22_ob, c_kq(1), 9), 1);
            q23_b1 <= f_gu(q22_ob, c_kq(2), 9) + shift_left(f_gu(q22_ob, c_kq(3), 9), 1);
            q23_b2 <= f_gu(q22_ob, c_kq(4), 9) + shift_left(f_gu(q22_ob, c_kq(5), 9), 1);
            q23_b3 <= f_gu(q22_ob, c_kq(6), 9) + shift_left(f_gu(q22_ob, c_kq(7), 9), 1);
            q23_g0 <= f_gu(q22_g, c_kq(0), 10) + shift_left(f_gu(q22_g, c_kq(1), 10), 1);
            q23_g1 <= f_gu(q22_g, c_kq(2), 10) + shift_left(f_gu(q22_g, c_kq(3), 10), 1);
            q23_g2 <= f_gu(q22_g, c_kq(4), 10) + shift_left(f_gu(q22_g, c_kq(5), 10), 1);
            q23_g3 <= f_gu(q22_g, c_kq(6), 10) + shift_left(f_gu(q22_g, c_kq(7), 10), 1);
            q24_bq <= resize(q23_b0, 11) + shift_left(resize(q23_b1, 11), 2);
            q24_br <= resize(q23_b2, 11) + shift_left(resize(q23_b3, 11), 2);
            q24_gq <= resize(q23_g0, 12) + shift_left(resize(q23_g1, 12), 2);
            q24_gr <= resize(q23_g2, 12) + shift_left(resize(q23_g3, 12), 2);
            v_bp := resize(q24_bq, 15) + shift_left(resize(q24_br, 15), 4);
            if v_bp(14 downto 9) /= 0 then s_cb(25) <= (others => '0');
            else s_cb(25) <= to_unsigned(8, 4) - resize(v_bp(8 downto 6), 4); end if;
            for i in 26 to 31 loop s_cb(i) <= s_cb(i - 1); end loop;
            v_gp := resize(q24_gq, 16) + shift_left(resize(q24_gr, 16), 4);
            v_cg := signed(resize(v_gp(15 downto 6), 12)) + 4 - signed(resize(c_g8, 12));
            if v_cg < 0 then s_cg(25) <= (others => '0');
            elsif v_cg > 8 then s_cg(25) <= to_unsigned(8, 4);
            else s_cg(25) <= unsigned(v_cg(3 downto 0)); end if;
            for i in 26 to 32 loop s_cg(i) <= s_cg(i - 1); end loop;

            -- 24: r^2; the coverage scale exponent (quarter octaves): cell px / 2R (disc), cell px (/ 1.4: diamond)
            q24_rs <= resize(q23_sqx, 17) + resize(q23_sqy, 17);
            q24_r2 <= q23_r2;
            if s_typ(23)(3 downto 2) = "00" then q24_n <= c_nc + signed(resize(q23_qr, 7));
            elsif s_typ(23)(2) = '1' then q24_n <= c_nl1;
            else q24_n <= c_nl; end if;
            -- 25: the disc's three edge functions
            q25_fa <= signed(resize(q24_r2, 18)) - signed(resize(q24_rs, 18));
            q25_fb <= signed(resize(q24_rs & '0', 19)) - signed(resize(q24_r2(15 downto 1), 19));
            q25_fc <= signed(resize(q24_r2(15 downto 2), 20)) - signed(resize(q24_rs & "00", 20));
            if q24_n(6) = '1' then s_n(25) <= (others => '0'); else s_n(25) <= unsigned(q24_n(5 downto 0)); end if;
            for i in 26 to 30 loop s_n(i) <= s_n(i - 1); end loop;
            -- 26: ring = min(outer, inner)
            if s_typ(25)(1) = '1' and q25_fb < resize(q25_fa, 19) then q26_m1 <= q25_fb;
            else q26_m1 <= resize(q25_fa, 19); end if;
            q26_fc <= q25_fc;
            -- 27: core = max(ring, core); the figure's edge function
            if s_typ(26)(3 downto 2) /= "00" then
                q27_f <= shift_left(resize(q26_fl, 20), 7);
            elsif s_typ(26)(0) = '1' and q26_fc > resize(q26_m1, 20) then
                q27_f <= q26_fc;
            else
                q27_f <= resize(q26_m1, 20);
            end if;
            -- 28, 29: * (8 - n mod 4)
            if s_n(27)(1) = '1' then q28_x <= shift_left(resize(q27_f, 23), 3) - shift_left(resize(q27_f, 23), 1);
            else q28_x <= shift_left(resize(q27_f, 23), 3); end if;
            if s_n(27)(0) = '1' then q28_y <= resize(q27_f, 23); else q28_y <= (others => '0'); end if;
            q29_t <= q28_x - q28_y;
            -- 30, 31: >> (3 + n / 4); coverage in eighths
            case s_n(29)(5 downto 4) is
                when "00"   => q30_x <= q29_t(22 downto 3);
                when "01"   => q30_x <= resize(q29_t(22 downto 7), 20);
                when "10"   => q30_x <= resize(q29_t(22 downto 11), 20);
                when others => q30_x <= resize(q29_t(22 downto 15), 20);
            end case;
            case s_n(30)(3 downto 2) is
                when "00"   => v_s20 := q30_x;
                when "01"   => v_s20 := shift_right(q30_x, 1);
                when "10"   => v_s20 := shift_right(q30_x, 2);
                when others => v_s20 := shift_right(q30_x, 3);
            end case;
            if s_typ(30)(3) = '1' then
                -- a rectangle's ramp lies outside its edge (full at f = 0), so an edge on the box edge leaves no seam
                if v_s20 < -8 then q31_cov <= (others => '0');
                elsif v_s20 >= 0 then q31_cov <= to_unsigned(8, 4);
                else q31_cov <= (not v_s20(3)) & unsigned(v_s20(2 downto 0)); end if;
            elsif v_s20 < -4 then q31_cov <= (others => '0');
            elsif v_s20 > 4 then q31_cov <= to_unsigned(8, 4);
            else q31_cov <= unsigned(v_s20(3 downto 0)) + 4; end if;
            -- 32: clipped to the figure box; nothing while the figure is under 3 % of its size
            if s_zf(31) = '1' then q32_cov <= (others => '0');
            elsif q31_cov < s_cb(31) then q32_cov <= q31_cov;
            else q32_cov <= s_cb(31); end if;
            -- 33: the blend table's address: (cell colour, figure colour, coverage), or in a gutter
            -- (ground, whichever is on top, gutter coverage)
            if s_gut = '1' and s_cg(32)(3) = '0' then v_gz := '1'; else v_gz := '0'; end if;
            if v_gz = '0' then
                tt_ra <= s_col(32) & q32_cov;
            elsif q32_cov >= 4 then
                tt_ra <= "00" & s_col(32)(1 downto 0) & s_cg(32);
            else
                tt_ra <= "00" & s_col(32)(3 downto 2) & s_cg(32);
            end if;
            -- 34: table; 35: the colour (the ground colour outside the generated avid)
            if s_gav_sr(C_NP - 3) = '0' then
                qf_y <= c_gy; qf_c <= c_gc;
            else
                qf_y <= unsigned(tt_q(25 downto 16));
                qf_c(1) <= unsigned(tt_q(15 downto 8)); qf_c(2) <= unsigned(tt_q(7 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(C_NP - 3);
        end if;
    end process p_look;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_bah = "00000000" then
                s_bu <= c_gc(1) & "000"; s_bv <= c_gc(2) & "000";
            else
                s_bu <= s_bu + qf_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + qf_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= qf_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y33 <= s_y32; s_u33 <= s_u32; s_v33 <= s_v32;
            else
                s_y33 <= to_unsigned(64, 10); s_u33 <= to_unsigned(512, 10); s_v33 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y33);
    data_out.u       <= std_logic_vector(s_u33);
    data_out.v       <= std_logic_vector(s_v33);
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture bauhaus;
