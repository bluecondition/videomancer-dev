-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Snakes v0.1.1 -- Kinetic.
--
-- After Akiyoshi Kitaoka, "Rotating Snakes" (2003): discs of concentric rings of segments stepping
-- black -> dark colour -> white -> light colour around each ring.  That luminance order makes a still disc seem
-- to turn; here the rings also really turn.
--
--   element  a ring of a disc; state = its rotation angle (8192 per turn)
--   lattice  cells of pitch P (4096 units); Staggered: rows 7/8 P apart, alternate rows offset P/2
--   polar    cell-local (dx, dy) folded to the first octant; ratio = min * recip(max) (table + one multiply);
--            angle = atan table; radius = max * sec (table + one multiply)
--   disc     hub r < 256, six rings of 256 units to r = 1792, a thin ground line between rings
--   theta    = Spread * disc index + Spread / 2 * ring - phase     (index: Linear i + j; Radial octagonal |i|, |j|)
--   angle    A = polar angle + Depth * wave(theta) +- drive        (+ on even rings, - on odd rings)
--   phase    phi = Segments * A + 1/2 step on odd rings +- source  (8192 = one black/dark/white/light cycle)
--   Order    phi is mirrored on alternate discs (they seem to turn, and do turn, the other way)
--   edges    step edges: coverage 4 .. 8 eighths over a power-of-two ramp chosen per ring from Segments, the
--            pixel size and the ring's radius; ring edges: 0 .. 8 eighths toward the ground
--   out      = ground + (T(step, side, coverage) - ground) * ring coverage / 8;  T is a per-field table
--
--   K1  Scale     disc pitch 180 .. 1018 virtual px (exponential, glided)
--   K2  Segments  3 .. 16 cycles (12 .. 64 segments) per ring
--   K3  Depth     wave swing 0 .. 1/4 turn (square law)
--   K4  Rate      bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Colour    the 16 shared palettes; the four tones are the palette colours shaded to fixed luma bands
--                 [K6 before v0.1.1]
--   K6  Camera    the source's strength: phi offset = luma * 8 * 1.25 k / 256 (0 = none, 20 % = v0.1.0's,
--                 50 % = 2.5 x, 100 % = 5 x; glided; x 1.25 since 2026-11-15).  (Spread, K5 before v0.1.1,
--                 is fixed at 760: phase step per disc 0.28 cycle, half of it per ring: the pinwheels)
--   S7  Wave      Smooth / Snap
--   S8  Field     Linear / Radial
--   S9  Lattice   Staggered / Square
--   S10 Order     Alternate (sequence mirrored on alternate discs) / Same
--   S11 Source    Video / Osc (luma held per disc turns odd and even rings against each other)
--   P12 DRIVE     0 = no crawl .. 100 % every ring crawling 1/4 turn per second, alternate rings opposite
--
-- Chassis lifted from louver.vhd:
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-disc hold
--   [D] wave engine
--   [E] the look: polar coordinates, rings, coverage, palette + AA blend
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 46) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 760 (C_KFIX_5).
-- 2026-11-15: K6 camera top +25 % (Ron)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.snakes_pkg.all;
use work.all;

architecture snakes of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(760, 10));   -- K6 camera pass: fixed knob
    constant C_LAT      : integer := 46;      -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob
    constant C_DEF_K6   : integer := 512;     -- K6 Camera (TOML default; the glide starts there)

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
    -- gated operands: every multiply in this file is a sum of these (one adder per two operand bits per stage)
    function f_gu(x : unsigned; b : std_logic; w : natural) return unsigned is
        variable r : unsigned(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;
    function f_gs(x : signed; b : std_logic; w : natural) return signed is
        variable r : signed(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (one read port each, registered address and data, then a fabric register)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(9 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_rc_a : unsigned(9 downto 0) := (others => '0');
    signal r_rc_q : unsigned(15 downto 0) := (others => '0');
    signal r_at_a : unsigned(9 downto 0) := (others => '0');
    signal r_at_q : unsigned(15 downto 0) := (others => '0');
    signal r_se_a : unsigned(8 downto 0) := (others => '0');
    signal r_se_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    -- [C] source hold: one smeared luma sample per disc column, written on the first line of each cell row
    -- (since the K6 camera pass the word is the sample's phi offset, luma x Camera, 13 bits)
    signal hl : t_r16 := (others => x"0000");
    attribute no_rw_check of hl : signal is true;
    signal hl_ra, hl_wa : unsigned(7 downto 0) := (others => '0');
    signal hl_q, hl_wd  : std_logic_vector(15 downto 0) := x"0000";
    signal hl_we : std_logic := '0';
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    signal ra : t_r16 := (others => (others => '0'));          -- larger |coordinate|, signs: stage 3 -> 13
    attribute no_rw_check of ra : signal is true;
    signal ra_d, ra_q, ra_f : std_logic_vector(15 downto 0) := (others => '0');
    signal rb : t_r16 := (others => (others => '0'));          -- polar angle, disc parity: stage 15 -> 27
    attribute no_rw_check of rb : signal is true;
    signal rb_d, rb_q, rb_f : std_logic_vector(15 downto 0) := (others => '0');
    signal rw : t_r16 := (others => (others => '0'));          -- the disc's wave angle: stage 6 -> 19
    attribute no_rw_check of rw : signal is true;
    signal rw_d, rw_q, rw_f : std_logic_vector(15 downto 0) := (others => '0');
    signal rd : t_r16 := (others => (others => '0'));          -- ring coverage, ramp select: stage 20 -> 33
    attribute no_rw_check of rd : signal is true;
    signal rd_d, rd_q, rd_f : std_logic_vector(15 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] blend table, refilled every field: T(step, side, c) = (c tone(step) + (8 - c) tone(neighbour)) / 8 - ground
    -- {Y 11 bits signed, U 9, V 9 (chroma in 8-bit units)}; address = step & side & (c - 4)
    type t_tt is array (0 to 255) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(7 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_run : std_logic := '0';
    signal t_ph  : unsigned(2 downto 0) := (others => '0');
    signal t_g   : std_logic := '0';
    signal t_ix  : unsigned(2 downto 0) := (others => '0');
    signal t_c   : unsigned(2 downto 0) := (others => '0');
    signal t_iy, t_jy : unsigned(9 downto 0) := (others => '0');
    signal t_ay, t_ny : signed(14 downto 0) := (others => '0');
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    type t_s13 is array (1 to 2) of signed(12 downto 0);
    signal t_ic, t_jc : t_u8 := (others => (others => '0'));
    signal t_ac, t_nc : t_s13 := (others => (others => '0'));
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
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(409 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);   -- K6 Camera
    signal c_kg : unsigned(9 downto 0) := to_unsigned(C_DEF_K6 / 2 + C_DEF_K6 / 8, 10);   -- its per-field value: HALF of 1.25 k
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_sq, s_same, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(3, 4);
    signal s_nq  : unsigned(3 downto 0) := to_unsigned(5, 4);
    signal c_n   : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier (24 iterations), serial shifter, sequencer scratch
    signal f_a, f_acc, z_fa : unsigned(47 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(23 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_nz  : std_logic := '0';
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : unsigned(1 downto 0) := (others => '0');
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_x   : unsigned(19 downto 0) := (others => '0');
    signal z_q   : unsigned(25 downto 0) := (others => '0');
    signal z_k8  : unsigned(3 downto 0) := to_unsigned(7, 4);
    -- lattice origin search: step back from the screen centre to the first pixel / line
    signal z_ps  : unsigned(1 downto 0) := (others => '0');
    signal z_run, z_bw, z_load, z_first : std_logic := '0';
    signal z_n   : unsigned(10 downto 0) := (others => '0');
    signal z_f   : unsigned(23 downto 0) := (others => '0');
    signal z_d   : unsigned(19 downto 0) := (others => '0');
    signal z_i   : unsigned(7 downto 0) := (others => '0');
    signal z_s   : signed(17 downto 0) := (others => '0');

    -- per-field constants
    signal c_ib   : unsigned(16 downto 0) := to_unsigned(46603, 17);   -- 2^17 * 128 / pitch
    signal c_du   : unsigned(19 downto 0) := to_unsigned(46603, 20);   -- u step per pixel (2^24 per cell)
    signal c_dvs, c_dv, c_dm : unsigned(19 downto 0) := to_unsigned(46603, 20);   -- v step per line; pixel size
    signal c_kr   : unsigned(2 downto 0) := (others => '0');           -- ring-edge ramp = 8 << kr units
    signal c_hg   : unsigned(6 downto 0) := to_unsigned(12, 7);        -- half the line between rings
    signal c_ka8, c_ka9, c_ka10 : unsigned(2 downto 0) := (others => '0');   -- step-edge ramp shift by radius band
    signal c_d    : unsigned(7 downto 0) := (others => '0');           -- depth, 255 = 1/4 turn
    signal c_ss   : unsigned(13 downto 0) := (others => '0');          -- spread per disc, 2^15 per cycle
    signal c_ddr  : unsigned(17 downto 0) := (others => '0');          -- drive per field, 2^26 per turn
    signal c_dr   : unsigned(25 downto 0) := (others => '0');
    signal c_drp, c_drn : unsigned(12 downto 0) := (others => '0');
    signal c_ph   : unsigned(25 downto 0) := (others => '0');          -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    signal c_nph  : unsigned(14 downto 0) := (others => '0');
    signal c_ufa, c_ufb : unsigned(23 downto 0) := (others => '0');    -- pixel 0: even rows / odd rows (Staggered)
    signal c_ia, c_ixb  : unsigned(7 downto 0) := (others => '0');
    signal c_sia, c_sib : signed(17 downto 0) := (others => '0');
    signal c_v0f  : unsigned(23 downto 0) := (others => '0');
    signal c_j0   : unsigned(7 downto 0) := (others => '0');
    signal c_sj0  : signed(17 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    -- palette: ground and the four tones (luma 10 bits, chroma 8 bits)
    signal c_gy   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_gc   : t_u8 := (others => to_unsigned(128, 8));

    -- [B] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- [C] source
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_sv  : unsigned(7 downto 0) := (others => '0');
    signal s_sav, s_savd : std_logic := '0';
    signal s_y1 : unsigned(13 downto 0) := (others => '0');
    signal s_ym : unsigned(17 downto 0) := (others => '0');    -- K6 Camera: the smeared sample x the gain
    signal s_u23d : std_logic := '0';

    -- [D] lattice: line registers (the NEXT line's start values), this line's values, pixel accumulators
    signal ls_vf  : unsigned(23 downto 0) := (others => '0');
    signal ls_j   : unsigned(7 downto 0) := (others => '0');
    signal ls_sj  : signed(17 downto 0) := (others => '0');
    signal ls_nx, ls_wr : std_logic := '0';
    signal n_ay   : unsigned(10 downto 0) := (others => '0');
    signal n_sy   : std_logic := '0';
    signal l_ay   : unsigned(10 downto 0) := (others => '0');
    signal l_sy, l_new, l_j0 : std_logic := '0';
    signal l_sj   : signed(17 downto 0) := (others => '0');
    signal l_asj  : unsigned(16 downto 0) := (others => '0');
    signal l_i0   : unsigned(7 downto 0) := (others => '0');
    signal a_uf   : unsigned(23 downto 0) := (others => '0');
    signal a_nw   : std_logic := '0';
    signal a_i    : unsigned(7 downto 0) := (others => '0');
    signal a_si   : signed(17 downto 0) := (others => '0');
    signal s_nw   : std_logic_vector(2 to 29) := (others => '0');
    -- wave
    signal q3_asi : unsigned(16 downto 0) := (others => '0');
    signal q3_lin, q4_lin : unsigned(14 downto 0) := (others => '0');
    signal q4_ge  : std_logic := '0';
    signal q4_asi : unsigned(15 downto 0) := (others => '0');
    signal q5_xa, q5_xb : unsigned(14 downto 0) := (others => '0');
    signal q6_tb  : unsigned(13 downto 0) := (others => '0');
    signal p20_t1, p20_t2, p21_tw : unsigned(13 downto 0) := (others => '0');
    signal s_neg  : std_logic_vector(22 to 27) := (others => '0');
    signal p24_w  : unsigned(9 downto 0) := (others => '0');
    signal p25_p0, p25_p1, p25_p2, p25_p3 : unsigned(11 downto 0) := (others => '0');
    signal p26_qa, p26_qb : unsigned(13 downto 0) := (others => '0');
    signal p27_aw : unsigned(10 downto 0) := (others => '0');
    signal p28_a1, p28_dr, p29_a : unsigned(12 downto 0) := (others => '0');
    signal p30_p01, p30_p23, p30_p4, p31_phi0, p31_st, p32_phi1 : unsigned(12 downto 0) := (others => '0');
    signal s_rodd : std_logic_vector(20 to 30) := (others => '0');
    signal s_par  : std_logic_vector(28 to 32) := (others => '0');
    signal k28    : unsigned(7 downto 0) := (others => '0');
    signal p30_hold : unsigned(12 downto 0) := (others => '0');

    -- [E] the look
    signal p2_ax  : unsigned(10 downto 0) := (others => '0');
    signal p2_sx, p3_sx, p3_swap, p3_par : std_logic := '0';
    signal p3_mx, p3_mn : unsigned(10 downto 0) := (others => '0');
    signal p4_mnn, p5_mnn, p6_mnn : unsigned(10 downto 0) := (others => '0');
    signal p6_rc  : unsigned(11 downto 0) := (others => '0');
    type t_pp is array (0 to 5) of unsigned(12 downto 0);
    signal p7_pp, p14_pp : t_pp := (others => (others => '0'));
    signal p8_sa, p8_sb, p8_sc, p9_sc, p15_sa, p15_sb : unsigned(14 downto 0) := (others => '0');
    signal p15_sc, p16_sc : unsigned(15 downto 0) := (others => '0');
    signal p9_t, p16_t : unsigned(19 downto 0) := (others => '0');
    signal p10_ra : unsigned(10 downto 0) := (others => '0');
    signal p13_ao, p13_se : unsigned(10 downto 0) := (others => '0');
    signal p14_mx : unsigned(10 downto 0) := (others => '0');
    signal p14_aq : unsigned(11 downto 0) := (others => '0');
    signal p14_sx, p14_par, p15_par : std_logic := '0';
    signal p15_th : unsigned(12 downto 0) := (others => '0');
    signal p17_r, p18_r : unsigned(11 downto 0) := (others => '0');
    signal p19_ring : unsigned(3 downto 0) := (others => '0');
    signal p19_e  : unsigned(7 downto 0) := (others => '0');
    signal p19_hub : std_logic := '0';
    signal p20_a, p34_a, p35_a, p36_a, p37_a : unsigned(3 downto 0) := (others => '0');
    signal p20_ka, p34_ka : unsigned(2 downto 0) := (others => '0');
    signal p20_x15, p20_hub, p34_hub : std_logic := '0';
    signal p33_stp, p34_stp : unsigned(1 downto 0) := (others => '0');
    signal p33_hi, p34_hi : std_logic := '0';
    signal p33_d  : unsigned(9 downto 0) := (others => '0');
    signal p34_dd : unsigned(10 downto 0) := (others => '0');
    signal p37_d  : std_logic_vector(28 downto 0) := (others => '0');
    signal p38_y1, p38_y2 : signed(14 downto 0) := (others => '0');
    signal p38_c1, p38_c2 : t_s13 := (others => (others => '0'));
    signal q39_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q39_c  : t_u8 := (others => to_unsigned(128, 8));

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
    -- Block RAM: ROMs, hold, delay rings (one write statement each; none ever
    -- uses data read from the address it is writing)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_SN_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_SN_SIN(to_integer(r_w_a));
            r_rc_q <= C_SN_RCP(to_integer(r_rc_a));
            r_at_q <= C_SN_ATN(to_integer(r_at_a));
            r_se_q <= C_SN_SEC(to_integer(r_se_a));
            r_pl_q <= C_SN_PAL(to_integer(r_pl_a));
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
            ra(to_integer(rc + 8)) <= ra_d;
            ra_q <= ra(to_integer(rc));
            ra_f <= ra_q;
            rb(to_integer(rc + 10)) <= rb_d;
            rb_q <= rb(to_integer(rc));
            rb_f <= rb_q;
            rw(to_integer(rc + 11)) <= rw_d;
            rw_q <= rw(to_integer(rc));
            rw_f <= rw_q;
            rd(to_integer(rc + 11)) <= rd_d;
            rd_q <= rd(to_integer(rc));
            rd_f <= rd_q;
            cr(to_integer(rc + 6)) <= std_logic_vector(q39_c(1)) & std_logic_vector(q39_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;
    ra_d <= "00" & p3_par & p3_swap & p3_sx & std_logic_vector(p3_mx);
    rb_d <= "00" & p15_par & std_logic_vector(p15_th);
    rw_d <= "00" & std_logic_vector(q6_tb);
    rd_d <= "0000000" & p20_hub & p20_x15 & std_logic_vector(p20_ka) & std_logic_vector(p20_a);

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
        variable v_n14 : unsigned(13 downto 0);
        variable v_nd : signed(14 downto 0);
        variable v_f25 : unsigned(24 downto 0);
        variable v_k9 : unsigned(3 downto 0);
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
                s_sq   <= registers_in(6)(2);
                s_same <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(4)(9 downto 6)); end if;
                -- Segments: 3 .. 16 cycles with hysteresis
                v_n14 := shift_left(resize(unsigned(registers_in(1)), 14), 4) - shift_left(resize(unsigned(registers_in(1)), 14), 1);
                v_nd := signed(resize(v_n14, 15)) - signed(resize(s_nq & "1000000000", 15));
                if v_nd > 640 or v_nd < -640 then s_nq <= v_n14(13 downto 10); end if;
            end if;
            c_n <= resize(s_nq, 5) + 3;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide for Scale, Drive and Camera (1/8 of the gap per field, +-2 deadband, Camera +-4):
            -- four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(1) = '1' then s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                        elsif z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif (s_gd > 16 or s_gd < -16) and (z_gc(1) = '0' or s_gd > 32 or s_gd < -32) then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(1) = '1' then s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        elsif z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 2 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- K6 Camera: the glided value, latched once the glides are done (zone clock 24).  2026-11-15 (K6 top
            -- +25 %): HALF of 1.25 k (k / 2 + k / 8, 0 .. 638), the product is sliced one bit lower below
            if s_zone = '1' and s_zc = 24 then c_kg <= resize(s_g6(12 downto 4), 10) + resize(s_g6(12 downto 6), 10); end if;

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k (no zc decode on the
            -- multiplier's path).
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" and s_zc(8 downto 5) /= 1
               and s_zc(8 downto 5) /= 8 and s_zc(8 downto 5) /= 9 and s_zc(8 downto 5) /= 10
               and s_zc(8 downto 5) /= 11 and s_zc(8 downto 5) /= 13 and s_zc(8 downto 5) /= 15 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(5125, 24);      -- pitch exponent
                    when 2  => z_fa(16 downto 0) <= c_ib;                 z_fb <= resize(s_ys, 24);           -- v step, square cell
                    when 3  => z_fa(19 downto 0) <= c_dvs;                z_fb <= to_unsigned(37449, 24);     -- * 8/7
                    when 4  =>                                                                                -- depth^2
                        z_fa(12 downto 3) <= unsigned(registers_in(2)); z_fb(12 downto 3) <= unsigned(registers_in(2));
                    when 5  =>                                                                                -- spread^2
                        z_fa(12 downto 3) <= unsigned(C_KFIX_5); z_fb(12 downto 3) <= unsigned(C_KFIX_5);
                    when 6  => z_fa(12 downto 0) <= s_gp;                 z_fb <= resize(s_gp, 24);           -- drive^2
                    when 7  => z_fa(19 downto 0) <= c_dm;                 z_fb <= resize(c_n, 24);            -- segments * pixel size
                    when 12 => z_fa(8 downto 0)  <= s_vm;                 z_fb <= to_unsigned(900, 24);       -- rate exponent
                    when others => z_fa(15 downto 0) <= z_mt;             z_fb <= to_unsigned(1193046, 24);   -- rate (14)
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
                    when "01"   => c_ib  <= '0' & c_ib(16 downto 1);
                    when others => c_dph <= c_dph(20 downto 0) & '0';
                end case;
            end if;

            -- lattice origin: a disc is centred on the screen centre; step back to the first pixel, then the first line
            z_load <= '0';
            if z_run = '1' then
                v_f25 := ('0' & z_f) - ('0' & resize(z_d, 24));
                z_f <= v_f25(23 downto 0); z_bw <= v_f25(24);
                if z_n = 1 then z_run <= '0'; end if;
                z_n <= z_n - 1;
            else
                z_bw <= '0';
            end if;
            if z_bw = '1' then
                z_i <= z_i - 1;
                z_s <= z_s - signed(resize(c_ss, 18));
            end if;
            if z_run = '0' and z_bw = '0' then
                case z_ps is
                    when "01" =>
                        c_ufa <= z_f; c_ia <= z_i; c_sia <= z_s;
                        z_f <= x"800000"; z_i <= (others => '0'); z_s <= (others => '0');
                        z_d <= c_dv; z_n <= f_half(s_tid); z_run <= '1';
                        z_ps <= "10";
                    when "10" =>
                        c_v0f <= z_f; c_j0 <= z_i; c_sj0 <= z_s;
                        c_ufb <= (not c_ufa(23)) & c_ufa(22 downto 0);
                        if c_ufa(23) = '1' then
                            c_ixb <= c_ia + 1; c_sib <= c_sia + signed(resize(c_ss, 18));
                        else
                            c_ixb <= c_ia; c_sib <= c_sia;
                        end if;
                        z_ps <= "11";
                    when "11" =>
                        z_load <= '1'; z_first <= '1';
                        z_ps <= "00";
                    when others => null;
                end case;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- wave phase and drive angle (last field's steps): one adder each
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                        c_dr <= c_dr + c_ddr;
                    when 21 =>
                        c_nph <= 0 - c_ph(25 downto 11);
                        c_drp <= c_dr(25 downto 13);
                        c_drn <= 0 - c_dr(25 downto 13);
                    -- pitch: e = k1 * 2.5 octaves + 0.49 (Q8); 2^17 * 128 / pitch
                    when 60 => z_e <= f_acc(26 downto 16) + 126;
                    when 61 =>
                        r_ex_a <= 0 - z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                        if z_e(7 downto 0) = 0 then z_nz <= '0'; else z_nz <= '1'; end if;
                    when 63 => z_mt <= r_ex_q;
                    when 64 =>
                        c_ib <= z_mt & '0'; z_sht <= "01";
                        if z_nz = '1' then z_shn <= z_sh + 1; else z_shn <= z_sh; end if;
                    when 80 =>
                        case to_integer(s_xs) is
                            when 4      => c_du <= resize(c_ib & '0', 20);
                            when 3      => c_du <= resize(c_ib, 20) + resize(c_ib(16 downto 1), 20);
                            when others => c_du <= resize(c_ib, 20);
                        end case;
                    when 124 => c_dvs <= f_acc(27 downto 8);
                    when 156 =>
                        if s_sq = '1' then c_dv <= c_dvs; else c_dv <= f_acc(34 downto 15); end if;
                    when 157 =>
                        if c_du > c_dvs then c_dm <= c_du; else c_dm <= c_dvs; end if;
                    -- ring-edge ramp: 8 << kr units, about 0.7 .. 1.4 pixels
                    when 158 => z_x <= resize(c_dm(19 downto 1), 20) + resize(c_dm(19 downto 3), 20);
                    when 159 => z_x <= z_x + resize(c_dm(19 downto 4), 20);
                    when 160 =>
                        if z_x(19) = '1' or z_x(18) = '1' then c_kr <= to_unsigned(4, 3); c_hg <= to_unsigned(72, 7);
                        elsif z_x(17) = '1' then c_kr <= to_unsigned(3, 3); c_hg <= to_unsigned(40, 7);
                        elsif z_x(16) = '1' then c_kr <= to_unsigned(2, 3); c_hg <= to_unsigned(24, 7);
                        elsif z_x(15) = '1' then c_kr <= to_unsigned(1, 3); c_hg <= to_unsigned(16, 7);
                        else c_kr <= to_unsigned(0, 3); c_hg <= to_unsigned(12, 7); end if;
                    when 188 => c_d <= f_acc(25 downto 18);
                    when 220 => c_ss <= f_acc(25 downto 12);
                    when 252 => c_ddr <= f_acc(25 downto 8);
                    -- step-edge ramp shift by radius band, from segments * pixel size * 1.75
                    when 284 => z_q <= resize(f_acc(24 downto 0), 26) + resize(f_acc(24 downto 1), 26);
                    when 285 => z_q <= z_q + resize(f_acc(24 downto 2), 26);
                    when 286 =>
                        if    z_q(25) = '1' or z_q(24) = '1' then z_k8 <= to_unsigned(11, 4);
                        elsif z_q(23) = '1' then z_k8 <= to_unsigned(10, 4);
                        elsif z_q(22) = '1' then z_k8 <= to_unsigned(9, 4);
                        elsif z_q(21) = '1' then z_k8 <= to_unsigned(8, 4);
                        elsif z_q(20) = '1' then z_k8 <= to_unsigned(7, 4);
                        elsif z_q(19) = '1' then z_k8 <= to_unsigned(6, 4);
                        elsif z_q(18) = '1' then z_k8 <= to_unsigned(5, 4);
                        elsif z_q(17) = '1' then z_k8 <= to_unsigned(4, 4);
                        elsif z_q(16) = '1' then z_k8 <= to_unsigned(3, 4);
                        else z_k8 <= to_unsigned(2, 4); end if;
                    when 287 =>
                        if z_k8 > 7 then c_ka8 <= "111"; else c_ka8 <= z_k8(2 downto 0); end if;
                        v_k9 := z_k8 - 1;
                        if v_k9 > 7 then c_ka9 <= "111"; else c_ka9 <= v_k9(2 downto 0); end if;
                        v_k9 := z_k8 - 2;
                        if v_k9 > 7 then c_ka10 <= "111"; else c_ka10 <= v_k9(2 downto 0); end if;
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh; z_sht <= "11";
                    -- lattice origin: pixel 0 of an even row first
                    when 600 =>
                        z_f <= x"800000"; z_i <= (others => '0'); z_s <= (others => '0');
                        z_d <= c_du; z_n <= s_w(11 downto 1); z_run <= '1';
                        z_ps <= "01";
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
    -- Blend table fill (zone, from zc 22; the tones come straight from the
    -- palette ROM; adders only).  For step i, side s
    -- (neighbour j = i - 1 or i + 1) and c = 4 .. 8:
    --   T = (c tone(i) + (8 - c) tone(j) - 8 ground) / 8
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_i, v_j : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            v_i := t_ix(2 downto 1);
            if t_ix(0) = '1' then v_j := v_i + 1; else v_j := v_i - 1; end if;
            if s_zone = '1' and s_zc = 22 then
                r_pl_a <= '0' & s_pal & "000";                       -- the ground
                t_run <= '1'; t_ph <= "000"; t_ix <= (others => '0'); t_g <= '1';
            elsif t_run = '1' then
                case t_ph is
                    when "000" =>
                        r_pl_a <= '0' & s_pal & (resize(v_i, 3) + 1);
                        t_ph <= "001";
                    when "001" =>
                        r_pl_a <= '0' & s_pal & (resize(v_j, 3) + 1);
                        if t_g = '1' then
                            c_gy <= unsigned(r_pl_q(31 downto 22)); c_gc(1) <= unsigned(r_pl_q(21 downto 14)); c_gc(2) <= unsigned(r_pl_q(11 downto 4));
                        end if;
                        t_g <= '0';
                        t_ph <= "010";
                    when "010" =>
                        t_iy <= unsigned(r_pl_q(31 downto 22)); t_ic(1) <= unsigned(r_pl_q(21 downto 14)); t_ic(2) <= unsigned(r_pl_q(11 downto 4));
                        t_ph <= "011";
                    when "011" =>
                        t_jy <= unsigned(r_pl_q(31 downto 22)); t_jc(1) <= unsigned(r_pl_q(21 downto 14)); t_jc(2) <= unsigned(r_pl_q(11 downto 4));
                        t_ph <= "100";
                    when "100" =>
                        t_ay <= signed(resize(t_iy & "00", 15)) + signed(resize(t_jy & "00", 15));
                        t_ny <= signed(resize(t_iy, 15)) - signed(resize(t_jy, 15));
                        for i in 1 to 2 loop
                            t_ac(i) <= signed(resize(t_ic(i) & "00", 13)) + signed(resize(t_jc(i) & "00", 13));
                            t_nc(i) <= signed(resize(t_ic(i), 13)) - signed(resize(t_jc(i), 13));
                        end loop;
                        t_ph <= "101";
                    when "101" =>
                        t_ay <= t_ay - signed(resize(c_gy & "000", 15));
                        for i in 1 to 2 loop
                            t_ac(i) <= t_ac(i) - signed(resize(c_gc(i) & "000", 13));
                        end loop;
                        t_c <= (others => '0');
                        t_ph <= "110";
                    when others =>
                        tt_we <= '1'; tt_wa <= "00" & t_ix & t_c;
                        tt_wd <= "000" & std_logic_vector(t_ay(13 downto 3)) & std_logic_vector(t_ac(1)(11 downto 3)) & std_logic_vector(t_ac(2)(11 downto 3));
                        t_ay <= t_ay + t_ny;
                        for i in 1 to 2 loop t_ac(i) <= t_ac(i) + t_nc(i); end loop;
                        t_c <= t_c + 1;
                        if t_c = 4 then
                            t_ph <= "000";
                            t_ix <= t_ix + 1;
                            if t_ix = 7 then t_run <= '0'; end if;
                        end if;
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
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- [C] Source field.  Osc: a two-axis triangle lattice (640 x 540 virtual
    -- px) drifting diagonally.  The luma (or the lattice) is smeared along x by
    -- a 1/16 filter and sampled once per disc, at the disc's centre column on
    -- the first line of its cell row, into a RAM indexed by disc column.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_x  : unsigned(13 downto 0);
        variable v_d1 : signed(14 downto 0);
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

            -- smear
            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
            elsif s_sav = '1' then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 4), 14));
            end if;

            -- per-disc sample: the pixel where u crosses 1/2, on the first line of a cell row
            if g_avid = '1' then s_u23d <= a_uf(23); else s_u23d <= '0'; end if;
            hl_we <= g_avid and a_uf(23) and (not s_u23d) and l_new;
            hl_wa <= a_i;
            -- K6 Camera: the sample x the gain in its own stage (one pixel behind s_y1), then the phi offset
            -- (Video: luma * 8 * k / 256; Osc twice that, mod one cycle); k = 256 gives v0.1.0's offsets.
            -- 2026-11-15: c_kg = 1.25 k / 2, so one bit lower; the top bit drops out of the 13-bit phase (a cycle)
            s_ym <= s_y1(13 downto 6) * c_kg;
            if s_src = '1' then hl_wd <= "000" & std_logic_vector(s_ym(15 downto 3));
            else hl_wd <= "000" & std_logic_vector(s_ym(16 downto 4)); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Lattice accumulators.  u and v count cells with 24 fraction bits, so
    -- a cell boundary is the carry out; the disc index and Spread * index step
    -- one clock later.  ls_* hold the start values of the NEXT line.
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_c25 : unsigned(24 downto 0);
        variable v_c   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- next line's |dy| (units of 1/4096 pitch; Staggered: the cell is 7/8 pitch tall)
            v_c := signed((not ls_vf(23)) & ls_vf(22 downto 12));
            if s_sq = '0' then v_c := v_c - shift_right(v_c, 3); end if;
            n_sy <= v_c(11);
            if v_c(11) = '1' then n_ay <= unsigned(not v_c(10 downto 0)); else n_ay <= unsigned(v_c(10 downto 0)); end if;
            l_asj <= unsigned(resize(abs(l_sj), 17));

            if z_load = '1' then
                ls_vf <= c_v0f; ls_j <= c_j0; ls_sj <= c_sj0; ls_nx <= '0'; ls_wr <= '0';
            elsif g_start = '1' then
                v_c25 := ('0' & ls_vf) + ('0' & resize(c_dv, 24));
                ls_vf <= v_c25(23 downto 0); ls_nx <= v_c25(24); ls_wr <= v_c25(24);
            elsif ls_wr = '1' then
                ls_j <= ls_j + 1; ls_sj <= ls_sj + signed(resize(c_ss, 18)); ls_wr <= '0';
            end if;

            if g_start = '1' then
                l_new <= ls_nx or z_first; l_sj <= ls_sj; l_j0 <= ls_j(0);
                l_ay <= n_ay; l_sy <= n_sy;
                if s_sq = '0' and ls_j(0) = '1' then
                    a_uf <= c_ufb; a_i <= c_ixb; a_si <= c_sib; l_i0 <= c_ixb;
                else
                    a_uf <= c_ufa; a_i <= c_ia; a_si <= c_sia; l_i0 <= c_ia;
                end if;
                a_nw <= '0';
            else
                if g_avid = '1' then
                    v_c25 := ('0' & a_uf) + ('0' & resize(c_du, 24));
                    a_uf <= v_c25(23 downto 0); a_nw <= v_c25(24);
                else
                    a_nw <= '0';
                end if;
                if a_nw = '1' then
                    a_i <= a_i + 1; a_si <= a_si + signed(resize(c_ss, 18));
                end if;
            end if;
        end if;
    end process p_acc;

    ---------------------------------------------------------------------------
    -- [E] Polar coordinates: octant fold, ratio, angle, radius; rings
    ---------------------------------------------------------------------------
    p_polar : process(clk)
        variable v_sw  : std_logic;
        variable v_mx, v_mn : unsigned(10 downto 0);
        variable v_p23 : unsigned(22 downto 0);
        variable v_p24 : unsigned(23 downto 0);
        variable v_aq  : unsigned(12 downto 0);
        variable v_a14 : unsigned(13 downto 0);
        variable v_ng  : std_logic;
        variable v_g   : unsigned(7 downto 0);
        variable v_e   : signed(9 downto 0);
        variable v_a   : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            s_nw(2) <= a_nw;
            for i in 3 to 29 loop s_nw(i) <= s_nw(i - 1); end loop;

            -- 2: |dx| (symmetric about the cell centre)
            p2_sx <= not a_uf(23);
            if a_uf(23) = '1' then p2_ax <= a_uf(22 downto 12); else p2_ax <= not a_uf(22 downto 12); end if;
            -- 3: larger / smaller, disc parity
            if l_ay > p2_ax then v_sw := '1'; else v_sw := '0'; end if;
            p3_swap <= v_sw; p3_sx <= p2_sx;
            if v_sw = '1' then p3_mx <= l_ay; p3_mn <= p2_ax; else p3_mx <= p2_ax; p3_mn <= l_ay; end if;
            p3_par <= a_i(0) xor l_j0;
            -- 4: normalise the larger to 1024 .. 2047: its low 10 bits address the reciprocal table
            if p3_mx(10) = '1' then
                v_mx := p3_mx; v_mn := p3_mn;
            elsif p3_mx(9) = '1' then
                v_mx := p3_mx(9 downto 0) & '0'; v_mn := p3_mn(9 downto 0) & '0';
            elsif p3_mx(8) = '1' then
                v_mx := p3_mx(8 downto 0) & "00"; v_mn := p3_mn(8 downto 0) & "00";
            elsif p3_mx(7) = '1' then
                v_mx := p3_mx(7 downto 0) & "000"; v_mn := p3_mn(7 downto 0) & "000";
            else
                v_mx := p3_mx(6 downto 0) & "0000"; v_mn := p3_mn(6 downto 0) & "0000";
            end if;
            r_rc_a <= v_mx(9 downto 0); p4_mnn <= v_mn;
            -- 5: ROM; 6: fabric copy
            p5_mnn <= p4_mnn; p6_mnn <= p5_mnn;
            p6_rc <= r_rc_q(11 downto 0);
            -- 7 .. 10: ratio = smaller * reciprocal (gated sums)
            for k in 0 to 5 loop
                p7_pp(k) <= f_gu(p6_mnn, p6_rc(2 * k), 13) + shift_left(f_gu(p6_mnn, p6_rc(2 * k + 1), 13), 1);
            end loop;
            p8_sa <= resize(p7_pp(0), 15) + shift_left(resize(p7_pp(1), 15), 2);
            p8_sb <= resize(p7_pp(2), 15) + shift_left(resize(p7_pp(3), 15), 2);
            p8_sc <= resize(p7_pp(4), 15) + shift_left(resize(p7_pp(5), 15), 2);
            p9_t  <= resize(p8_sa, 20) + shift_left(resize(p8_sb, 20), 4);
            p9_sc <= p8_sc;
            v_p23 := resize(p9_t, 23) + shift_left(resize(p9_sc, 23), 8);
            p10_ra <= v_p23(22 downto 12);
            -- 11: table addresses; 12: ROM; 13: fabric copies
            if p10_ra(10) = '1' then
                r_at_a <= (others => '1'); r_se_a <= (others => '1');
            else
                r_at_a <= p10_ra(9 downto 0); r_se_a <= p10_ra(9 downto 1);
            end if;
            p13_ao <= r_at_q(10 downto 0); p13_se <= r_se_q(10 downto 0);
            -- 14 .. 18: radius = larger * (1 + sec part); the angle unfolded by octant and quadrant
            for k in 0 to 4 loop
                p14_pp(k) <= f_gu(unsigned(ra_f(10 downto 0)), p13_se(2 * k), 13)
                           + shift_left(f_gu(unsigned(ra_f(10 downto 0)), p13_se(2 * k + 1), 13), 1);
            end loop;
            p14_pp(5) <= f_gu(unsigned(ra_f(10 downto 0)), p13_se(10), 13);
            p14_mx <= unsigned(ra_f(10 downto 0));
            if ra_f(12) = '1' then p14_aq <= to_unsigned(2048, 12) - resize(p13_ao, 12); else p14_aq <= resize(p13_ao, 12); end if;
            p14_sx <= ra_f(11); p14_par <= ra_f(13);
            p15_sa <= resize(p14_pp(0), 15) + shift_left(resize(p14_pp(1), 15), 2);
            p15_sb <= resize(p14_pp(2), 15) + shift_left(resize(p14_pp(3), 15), 2);
            p15_sc <= resize(p14_pp(4), 16) + shift_left(resize(p14_pp(5), 16), 2) + shift_left(resize(p14_mx, 16), 4);
            v_ng := p14_sx xor l_sy;
            v_aq := resize(p14_aq, 13);
            if v_ng = '1' then v_aq := not v_aq; end if;
            v_a14 := (p14_sx & "000000000000" & '1') + (v_aq & v_ng);
            p15_th <= v_a14(13 downto 1);
            p15_par <= p14_par;
            p16_t  <= resize(p15_sa, 20) + shift_left(resize(p15_sb, 20), 4);
            p16_sc <= p15_sc;
            v_p24 := resize(p16_t, 24) + shift_left(resize(p16_sc, 24), 8);
            p17_r <= v_p24(23 downto 12);
            p18_r <= p17_r;
            -- 19: ring number, distance to the nearer ring edge (the hub has only its outer edge)
            p19_ring <= p18_r(11 downto 8);
            v_g := p18_r(7 downto 0);
            if p18_r(11 downto 8) = 0 then
                p19_hub <= '1'; p19_e <= not v_g;
            else
                p19_hub <= '0';
                if v_g(7) = '1' then p19_e <= '0' & (not v_g(6 downto 0)); else p19_e <= v_g; end if;
            end if;
            -- 20: ring coverage 0 .. 8; the step-edge ramp of this ring's radius band
            v_e := signed(resize(p19_e, 10)) - signed(resize(c_hg, 10));
            case to_integer(c_kr) is
                when 0      => v_a := v_e;
                when 1      => v_a := shift_right(v_e, 1);
                when 2      => v_a := shift_right(v_e, 2);
                when 3      => v_a := shift_right(v_e, 3);
                when others => v_a := shift_right(v_e, 4);
            end case;
            v_a := v_a + 4;
            if p19_ring >= 7 or v_a < 0 then p20_a <= (others => '0');
            elsif v_a > 8 then p20_a <= to_unsigned(8, 4);
            else p20_a <= unsigned(v_a(3 downto 0)); end if;
            if p19_ring >= 4 then p20_ka <= c_ka10;
            elsif p19_ring >= 2 then p20_ka <= c_ka9;
            else p20_ka <= c_ka8; end if;
            if p19_ring = 3 or p19_ring >= 6 then p20_x15 <= '1'; else p20_x15 <= '0'; end if;
            p20_hub <= p19_hub;
        end if;
    end process p_polar;

    ---------------------------------------------------------------------------
    -- [D] Wave engine: theta per disc and ring -> wave ROM -> Depth * wave;
    -- the ring's angle; the segment phase
    ---------------------------------------------------------------------------
    p_wave : process(clk)
        variable v_a   : unsigned(17 downto 0);
        variable v_pos : unsigned(11 downto 0);
        variable v_p   : unsigned(17 downto 0);
        variable v_w13 : unsigned(12 downto 0);
        variable v_a14 : unsigned(13 downto 0);
        variable v_k   : unsigned(7 downto 0);
        variable v_st  : unsigned(12 downto 0);
        variable v_ph  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: |Spread * i|, the linear sum
            v_a := unsigned(abs(a_si)); q3_asi <= v_a(16 downto 0);
            q3_lin <= unsigned(a_si(14 downto 0)) + unsigned(l_sj(14 downto 0));
            -- 4: compare
            if q3_asi >= l_asj then q4_ge <= '1'; else q4_ge <= '0'; end if;
            q4_asi <= q3_asi(15 downto 0); q4_lin <= q3_lin;
            -- 5: octagonal metric max + min / 2 (Radial) or i + j (Linear), less the phase
            if s_rad = '0' then
                q5_xa <= q4_lin + c_nph; q5_xb <= (others => '0');
            elsif q4_ge = '1' then
                q5_xa <= q4_asi(14 downto 0) + c_nph; q5_xb <= l_asj(15 downto 1);
            else
                q5_xa <= l_asj(14 downto 0) + c_nph; q5_xb <= q4_asi(15 downto 1);
            end if;
            -- 6: the disc's theta (14 bits per cycle)
            v_a(14 downto 0) := q5_xa + q5_xb;
            q6_tb <= v_a(14 downto 1);
            -- 20, 21: + ring * Spread / 2
            p20_t1 <= unsigned(rw_f(13 downto 0)) + f_gu(c_ss(13 downto 2), p19_ring(0), 14);
            p20_t2 <= f_gu(c_ss(13 downto 1), p19_ring(1), 14) + f_gu(c_ss, p19_ring(2), 14);
            p21_tw <= p20_t1 + p20_t2;
            s_rodd(20) <= p19_ring(0);
            for i in 21 to 30 loop s_rodd(i) <= s_rodd(i - 1); end loop;
            -- 22: wave ROM address: quarter-wave fold; Snap = the first sixteenth stretched over the quarter
            v_pos := p21_tw(11 downto 0);
            if p21_tw(12) = '1' then v_pos := not v_pos; end if;
            if s_snap = '0' then r_w_a <= v_pos(11 downto 2);
            elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
            else r_w_a <= v_pos(9 downto 0); end if;
            s_neg(22) <= p21_tw(13);
            for i in 23 to 27 loop s_neg(i) <= s_neg(i - 1); end loop;
            -- 23: ROM; 24: fabric copy
            p24_w <= r_w_q(9 downto 0);
            -- 25 .. 27: Depth * wave as gated sums (255 * 1023 = 1/4 turn = 2048)
            p25_p0 <= f_gu(p24_w, c_d(0), 12) + shift_left(f_gu(p24_w, c_d(1), 12), 1);
            p25_p1 <= f_gu(p24_w, c_d(2), 12) + shift_left(f_gu(p24_w, c_d(3), 12), 1);
            p25_p2 <= f_gu(p24_w, c_d(4), 12) + shift_left(f_gu(p24_w, c_d(5), 12), 1);
            p25_p3 <= f_gu(p24_w, c_d(6), 12) + shift_left(f_gu(p24_w, c_d(7), 12), 1);
            p26_qa <= resize(p25_p0, 14) + shift_left(resize(p25_p1, 14), 2);
            p26_qb <= resize(p25_p2, 14) + shift_left(resize(p25_p3, 14), 2);
            v_p := resize(p26_qa, 18) + shift_left(resize(p26_qb, 18), 4);
            p27_aw <= v_p(17 downto 7);
            -- 28, 29: the ring's angle = polar angle +- swing +- drive (odd rings the other way)
            v_w13 := resize(p27_aw, 13);
            if s_neg(27) = '1' then v_w13 := not v_w13; end if;
            v_a14 := (unsigned(rb_f(12 downto 0)) & '1') + (v_w13 & s_neg(27));
            p28_a1 <= v_a14(13 downto 1);
            if s_rodd(27) = '1' then p28_dr <= c_drn; else p28_dr <= c_drp; end if;
            s_par(28) <= rb_f(13);
            for i in 29 to 32 loop s_par(i) <= s_par(i - 1); end loop;
            p29_a <= p28_a1 + p28_dr;
            -- 30, 31: * Segments (mod one cycle)
            p30_p01 <= f_gu(p29_a, c_n(0), 13) + shift_left(f_gu(p29_a, c_n(1), 13), 1);
            p30_p23 <= f_gu(p29_a, c_n(2), 13) + shift_left(f_gu(p29_a, c_n(3), 13), 1);
            p30_p4  <= f_gu(p29_a, c_n(4), 13);
            p31_phi0 <= p30_p01 + shift_left(p30_p23, 2) + shift_left(p30_p4, 4);
            -- the disc's held source: column counter at 28, RAM at 29, captured at each new column
            v_k := k28;
            if s_gav_sr(25) = '1' and s_gav_sr(26) = '0' then v_k := l_i0;
            elsif s_nw(27) = '1' then v_k := k28 + 1; end if;
            k28 <= v_k; hl_ra <= v_k;
            if s_nw(29) = '1' or (s_gav_sr(27) = '1' and s_gav_sr(28) = '0') then p30_hold <= unsigned(hl_q(12 downto 0)); end if;
            -- 31: odd rings: half a step, source the other way (the held offset is luma x Camera: at 25 %
            -- white ~ 0.9 step, Osc twice that)
            v_st := p30_hold;
            if s_rodd(30) = '1' then p31_st <= to_unsigned(1024, 13) - v_st; else p31_st <= v_st; end if;
            -- 32: the segment phase
            p32_phi1 <= p31_phi0 + p31_st;
            -- 33: Order (mirrored on alternate discs); step, side, distance to the nearer step edge
            v_ph := p32_phi1;
            if s_par(32) = '1' and s_same = '0' then v_ph := not v_ph; end if;
            p33_stp <= v_ph(12 downto 11); p33_hi <= v_ph(10);
            if v_ph(10) = '1' then p33_d <= not v_ph(9 downto 0); else p33_d <= v_ph(9 downto 0); end if;
        end if;
    end process p_wave;

    ---------------------------------------------------------------------------
    -- [E] Coverage and colour
    ---------------------------------------------------------------------------
    p_look : process(clk)
        variable v_ge  : std_logic;
        variable v_lo  : unsigned(1 downto 0);
        variable v_c3  : unsigned(2 downto 0);
        variable v_stp : unsigned(1 downto 0);
        variable v_dy  : signed(10 downto 0);
        variable v_dc  : signed(8 downto 0);
        variable v_sy  : signed(14 downto 0);
        variable v_sc  : signed(12 downto 0);
        variable v_oy  : signed(13 downto 0);
        variable v_oc  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 34: upper half of a radius band: ramp * 2/3
            if rd_f(7) = '1' then p34_dd <= resize(p33_d, 11) + resize(p33_d(9 downto 1), 11); else p34_dd <= resize(p33_d, 11); end if;
            p34_stp <= p33_stp; p34_hi <= p33_hi;
            p34_a <= unsigned(rd_f(3 downto 0)); p34_ka <= unsigned(rd_f(6 downto 4)); p34_hub <= rd_f(8);
            -- 35: step coverage c = 4 + (distance >> ka), 8 from 4 up; the table address
            case to_integer(p34_ka) is
                when 0 => v_lo := p34_dd(1 downto 0); if p34_dd(10 downto 2) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                when 1 => v_lo := p34_dd(2 downto 1); if p34_dd(10 downto 3) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                when 2 => v_lo := p34_dd(3 downto 2); if p34_dd(10 downto 4) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                when 3 => v_lo := p34_dd(4 downto 3); if p34_dd(10 downto 5) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                when 4 => v_lo := p34_dd(5 downto 4); if p34_dd(10 downto 6) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                when 5 => v_lo := p34_dd(6 downto 5); if p34_dd(10 downto 7) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                when 6 => v_lo := p34_dd(7 downto 6); if p34_dd(10 downto 8) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
                when others => v_lo := p34_dd(8 downto 7); if p34_dd(10 downto 9) /= 0 then v_ge := '1'; else v_ge := '0'; end if;
            end case;
            v_stp := p34_stp;
            if p34_hub = '1' then v_c3 := "100"; v_stp := "11";
            elsif v_ge = '1' then v_c3 := "100";
            else v_c3 := '0' & v_lo; end if;
            tt_ra <= "00" & v_stp & p34_hi & v_c3;
            p35_a <= p34_a; p36_a <= p35_a; p37_a <= p36_a;
            -- 36: RAM; 37: fabric copy
            p37_d <= tt_q(28 downto 0);
            -- 38, 39: ground + table word * ring coverage / 8; the ground colour outside the generated avid
            v_dy := signed(p37_d(28 downto 18));
            p38_y1 <= f_gs(v_dy, p37_a(0), 15) + shift_left(f_gs(v_dy, p37_a(1), 15), 1);
            if p37_a(3) = '1' then p38_y2 <= shift_left(resize(v_dy, 15), 3);
            else p38_y2 <= shift_left(f_gs(v_dy, p37_a(2), 15), 2); end if;
            v_sy := p38_y1 + p38_y2;
            v_oy := signed(resize(c_gy, 14)) + resize(shift_right(v_sy, 3), 14);
            if s_gav_sr(36) = '0' then q39_y <= c_gy;
            elsif v_oy < 0 then q39_y <= (others => '0');
            elsif v_oy > 1023 then q39_y <= (others => '1');
            else q39_y <= unsigned(v_oy(9 downto 0)); end if;
            for i in 1 to 2 loop
                if i = 1 then v_dc := signed(p37_d(17 downto 9)); else v_dc := signed(p37_d(8 downto 0)); end if;
                p38_c1(i) <= f_gs(v_dc, p37_a(0), 13) + shift_left(f_gs(v_dc, p37_a(1), 13), 1);
                if p37_a(3) = '1' then p38_c2(i) <= shift_left(resize(v_dc, 13), 3);
                else p38_c2(i) <= shift_left(f_gs(v_dc, p37_a(2), 13), 2); end if;
                v_sc := p38_c1(i) + p38_c2(i);
                v_oc := signed(resize(c_gc(i), 12)) + resize(shift_right(v_sc, 3), 12);
                if s_gav_sr(36) = '0' then q39_c(i) <= c_gc(i);
                elsif v_oc < 0 then q39_c(i) <= (others => '0');
                elsif v_oc > 255 then q39_c(i) <= (others => '1');
                else q39_c(i) <= unsigned(v_oc(7 downto 0)); end if;
            end loop;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(36);
        end if;
    end process p_look;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 47 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 40: running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
            -- back to 8 x ground once 8 blank samples are in, so a glitched sample can't offset the colours for good
            if s_bah = "00000000" then
                s_bu <= c_gc(1) & "000"; s_bv <= c_gc(2) & "000";
            else
                s_bu <= s_bu + q39_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q39_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q39_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            -- 41: filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- 42: luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 43: blanking gate
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

end architecture snakes;
