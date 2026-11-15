-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cogwork v0.1.1 -- Kinetic.
--
-- A wall of meshed cogs on a square lattice turning as one train (clockwork, gear walls).  Alternate gears turn
-- opposite ways and sit half a tooth apart, so neighbours mesh.
--
--   element  a gear (one lattice cell of pitch P; pitch radius P / 2)
--   state    = Depth * wave(theta) + source:  it swings the gear's spoked wheel against its rim and swells its hub
--   theta    = Spread * index - phase     index: Linear = column + row; Radial = octagonal distance from the centre gear
--   polar    cell-local (dx, dy) folded into one octant; ratio = min * recip(max) (table + one multiply);
--            angle = atan table; radius = max * sec (second table + one multiply); all in 1/4096 of the pitch
--   teeth    r < P/2 + a * clip(tooth triangle of N * (angle -+ train))  (trapezoid teeth, a = 0.95 P / N)
--   mesh     the neighbour's teeth inside this cell are this gear's own profile mirrored at the cell edge:
--            r + (P - 2 max) < P/2 - a * tooth   (exact on the edge, so there is no seam)
--   body     hub hole, accent hub (swells with the state), collar, four spokes (bars) with windows, a bevel groove
--            inside the rim; Disc fills the windows with a shade; Line draws every edge as an outline instead
--   edges    one signed distance per pixel (the nearest edge of its radial zone) -> coverage in eighths -> a
--            per-field blend table (9 colour pairs x 9 steps)
--
--   K1  Scale    gear pitch 64 .. 512 virtual lines (exponential, glided)
--   K2  Teeth    6, 8, 10, 12, 16, 20, 24, 32 teeth (hysteresis)
--   K3  Depth    spoke swing 0 .. 1/2 turn and hub swell (square law)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Colour   the 16 shared palettes (ground, A / B = gears by parity, accent = hubs) (was K6; Spread is
--                fixed at 450 = the old default, C_KFIX_5)
--   K6  Camera   camera strength 0 .. 100 % (default 50 % = twice the v0.1.0 gain): state += luma x 0 .. 32
--                (25 % = x8 = v0.1.0), Osc x3.  Applied to the held sample before the hold.
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Body     Spoked / Disc
--   S10 Ink      Fill / Line
--   S11 Source   Video / Osc
--   P12 DRIVE    train speed, 1/256 tooth per field .. 1 tooth per field (exponential, glided): the teeth strobe
--
-- Sections (the Kinetic chassis, lifted from louver):
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-gear hold
--   [D] wave engine: cell indices -> theta -> wave ROM -> state
--   [E] the look: polar engine, gear silhouette, coverage, blend table
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 32) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 450 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cogwork_pkg.all;
use work.all;

architecture cogwork of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(450, 10));   -- K6 camera pass: fixed knob
    constant C_LAT      : integer := 32;      -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob

    -- gear constants, 1/4096 of the pitch
    constant C_RH0 : integer := 205;          -- hub hole radius
    constant C_RH1 : integer := 471;          -- hub (accent) radius at rest
    constant C_RCO : integer := 676;          -- collar radius at rest = the windows' inner edge
    constant C_ZA  : integer := 338;
    constant C_ZB  : integer := 573;
    constant C_WS  : integer := 137;          -- spoke half-width
    constant C_E0  : integer := 2008;         -- pitch radius less half the mesh clearance

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
    -- 256 * log2(virtual lines per field line): the pixel size for the coverage gain
    function f_lpq(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(551, 11);
            when 4          => return to_unsigned(300, 11);
            when 8          => return to_unsigned(488, 11);
            when 12         => return to_unsigned(232, 11);
            when 5 | 6 | 14 => return to_unsigned(150, 11);
            when 1 | 2 | 10 => return to_unsigned(256, 11);
            when others     => return to_unsigned(0, 11);
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
    signal r_w_a  : unsigned(8 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_rc_q : unsigned(15 downto 0) := (others => '0');
    signal r_at_q : unsigned(15 downto 0) := (others => '0');
    signal r_sc_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- [C] source hold: one smeared luma sample per gear column, taken on the first line of each cell row
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal sl : t_r16 := (others => (others => '0'));
    attribute no_rw_check of sl : signal is true;
    signal sl_ra, sl_wa : unsigned(7 downto 0) := (others => '0');
    signal sl_q, sl_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal sl_we : std_logic := '0';
    -- K6 camera gain: glided knob (Q3), per-field gain (K6 / 4, x3 for Osc), the gated-sum product stages
    signal s_g6  : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal c_cg  : unsigned(9 downto 0) := to_unsigned(128, 10);
    type t_u12c is array (0 to 3) of unsigned(11 downto 0);
    signal cm1_p : t_u12c := (others => (others => '0'));
    signal cm2_a, cm2_b : unsigned(13 downto 0) := (others => '0');
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    signal dr : t_r16 := (others => (others => '0'));          -- {par, hy, hx, swap, max}: stage 3 -> 11
    attribute no_rw_check of dr : signal is true;
    signal dr_q : std_logic_vector(15 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] blend table, refilled every field: T(pair, cov) = lo + (hi - lo) cov / 8;  {Y 10, U 8, V 8}
    type t_tt is array (0 to 255) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(7 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_run : std_logic := '0';
    signal t_p, t_k, t_st : unsigned(3 downto 0) := (others => '0');
    signal t_ay : unsigned(12 downto 0) := (others => '0');
    signal t_dy : signed(10 downto 0) := (others => '0');
    signal t_hy : signed(12 downto 0) := (others => '0');
    type t_s11a is array (1 to 2) of signed(10 downto 0);
    signal t_hc : t_s11a := (others => (others => '0'));
    type t_u11a is array (1 to 2) of unsigned(10 downto 0);
    type t_s9a  is array (1 to 2) of signed(8 downto 0);
    signal t_ac : t_u11a := (others => (others => '0'));
    signal t_dc : t_s9a := (others => (others => '0'));
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

    -- knobs: glided (Q3), raw rate, switches, palette, tooth step
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(598 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_disc, s_line, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal s_nix : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier (24 iterations), serial shifter, sequencer scratch
    signal f_a, f_acc, z_fa : unsigned(37 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(20 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_m   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_nz  : std_logic := '0';
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : std_logic := '0';
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_v   : unsigned(22 downto 0) := (others => '0');

    -- per-field constants
    signal c_ip   : unsigned(18 downto 0) := to_unsigned(77672, 19);   -- 2^24 / pitch
    signal c_sx   : unsigned(19 downto 0) := to_unsigned(77672, 20);   -- cell phase step per pixel, Q24
    signal c_sy   : unsigned(20 downto 0) := to_unsigned(77672, 21);   -- per line
    signal c_x0, c_y0 : unsigned(31 downto 0) := (others => '0');      -- {cell index, phase} of pixel 0 / line 0
    signal c_si0, c_sj0 : signed(19 downto 0) := (others => '0');      -- Spread * index there
    signal c_n    : unsigned(2 downto 0) := to_unsigned(2, 3);         -- coverage gain = (m3 ? 1.5 : 1) * 2^-n
    signal c_m3   : std_logic := '0';
    signal c_a7   : unsigned(6 downto 0) := to_unsigned(41, 7);        -- tooth half-depth / 8
    signal c_rn   : unsigned(12 downto 0) := to_unsigned(2731, 13);    -- 32768 / N
    signal c_n1, c_n2 : unsigned(1 downto 0) := "00";                  -- N = 2^(n1 + 2) + (n2 = 0 ? 0 : 2^n2)
    signal c_rin, c_rb0, c_rb1, c_zc, c_zd : signed(12 downto 0) := (others => '0');
    signal c_d    : unsigned(5 downto 0) := (others => '0');           -- depth, Q7 turns
    signal c_s    : unsigned(14 downto 0) := (others => '0');          -- spread per gear, 1/65536 cycle
    signal c_ph   : unsigned(25 downto 0) := (others => '0');          -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    signal c_nph  : unsigned(15 downto 0) := (others => '0');
    signal c_psi  : unsigned(27 downto 0) := (others => '0');          -- train angle, Q28 turns
    signal c_dpsi : unsigned(25 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    signal c_gy : unsigned(9 downto 0) := to_unsigned(64, 10);         -- ground
    signal c_gc : t_u8 := (others => to_unsigned(128, 8));

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

    -- [D] line registers (the NEXT line's start values) and the pixel accumulator (stage 1)
    signal ls_ay, ls_ayn, l_ay : unsigned(31 downto 0) := (others => '0');
    signal ls_sj, l_sj : signed(19 downto 0) := (others => '0');
    signal ls_first, l_first, s_f1 : std_logic := '0';
    signal a_x : unsigned(31 downto 0) := (others => '0');
    -- wave path
    signal q2_si : signed(19 downto 0) := (others => '0');
    signal q2_ixl : std_logic := '0';
    signal q3_asi, q3_asj : unsigned(18 downto 0) := (others => '0');
    signal q3_lin : unsigned(15 downto 0) := (others => '0');
    signal q4_asi, q4_asj : unsigned(16 downto 0) := (others => '0');
    signal q4_lin, q5_xa, q5_xb, q6_th : unsigned(15 downto 0) := (others => '0');
    signal q4_ge : std_logic := '0';
    signal s_neg : std_logic_vector(7 to 12) := (others => '0');
    signal q9_w  : unsigned(9 downto 0) := (others => '0');
    signal q10_p0, q10_p1, q10_p2 : unsigned(11 downto 0) := (others => '0');
    signal q11_qa : unsigned(13 downto 0) := (others => '0');
    signal q11_p2 : unsigned(11 downto 0) := (others => '0');
    signal q12_aw : unsigned(12 downto 0) := (others => '0');
    signal q11_src : unsigned(13 downto 0) := (others => '0');
    signal q12_st : unsigned(14 downto 0) := (others => '0');
    signal q13_stt : signed(15 downto 0) := (others => '0');
    signal q14_hs : signed(9 downto 0) := (others => '0');
    type t_ix is array (2 to 8) of unsigned(4 downto 0);
    signal q_ix : t_ix := (others => (others => '0'));

    -- [E] polar engine
    signal q2_vx, q2_vy, q3_mx, q3_mn, q4_mn, q5_mn : unsigned(10 downto 0) := (others => '0');
    signal q2_hx, q2_hy, q2_par, q3_sw, q3_hx, q3_hy, q3_par : std_logic := '0';
    signal q5_rc : unsigned(11 downto 0) := (others => '0');
    type t_u13a is array (natural range <>) of unsigned(12 downto 0);
    signal q6_p  : t_u13a(0 to 5) := (others => (others => '0'));
    signal q7_a, q7_b, q7_c, q8_c : unsigned(14 downto 0) := (others => '0');
    signal q8_t  : unsigned(18 downto 0) := (others => '0');
    signal q9_rt : unsigned(9 downto 0) := (others => '0');
    signal q11_at : unsigned(10 downto 0) := (others => '0');
    signal q11_sc : unsigned(9 downto 0) := (others => '0');
    signal q11_mx, q12_mx, q13_mx, q14_mx, q15_mx : unsigned(10 downto 0) := (others => '0');
    signal q11_sw, q11_hx, q11_hy : std_logic := '0';
    signal s_par : std_logic_vector(11 to 19) := (others => '0');
    signal q12_p : t_u13a(0 to 4) := (others => (others => '0'));
    signal q13_a, q13_b, q13_c, q14_c : unsigned(14 downto 0) := (others => '0');
    signal q14_t : unsigned(18 downto 0) := (others => '0');
    signal q15_r : unsigned(11 downto 0) := (others => '0');
    signal q12_phi, q13_phg, q14_tau : unsigned(13 downto 0) := (others => '0');
    signal q14_phs : unsigned(11 downto 0) := (others => '0');
    -- gear
    signal q15_xc : signed(7 downto 0) := (others => '0');
    signal s_fl   : std_logic_vector(15 to 18) := (others => '0');
    signal q15_d8 : unsigned(7 downto 0) := (others => '0');
    signal q15_rh1, q15_rco, q15_zb, q15_za : signed(12 downto 0) := (others => '0');
    type t_s10a is array (natural range <>) of signed(9 downto 0);
    signal q16_t  : t_s10a(0 to 3) := (others => (others => '0'));
    signal q17_tt : signed(10 downto 0) := (others => '0');
    type t_u11b is array (natural range <>) of unsigned(10 downto 0);
    signal q16_u  : t_u11b(0 to 3) := (others => (others => '0'));
    signal q17_pu : unsigned(16 downto 0) := (others => '0');
    signal q16_da, q16_db, q16_c1, q16_c2, q16_d1, q16_d2, q16_e, q16_f : signed(12 downto 0) := (others => '0');
    signal q16_za, q16_zb, q16_zc, q16_zd : std_logic := '0';
    signal q17_dr, q18_dr, q17_e, q17_f : signed(12 downto 0) := (others => '0');
    signal q17_zn, q18_zn, q19_zn : unsigned(2 downto 0) := (others => '0');
    signal q18_do, q18_dn, q19_de, q19_dc, q19_dr : signed(12 downto 0) := (others => '0');
    signal q18_dsp : signed(11 downto 0) := (others => '0');
    signal q19_own : std_logic := '0';
    signal q20_d  : signed(10 downto 0) := (others => '0');
    signal q20_pr, q21_pr, q22_pr : unsigned(3 downto 0) := (others => '0');
    signal q21_dm, q22_cc : signed(11 downto 0) := (others => '0');
    signal q25_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q25_c  : t_u8 := (others => to_unsigned(128, 8));

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
    -- Block RAM: ROMs, hold, delay rings, blend table (one write statement
    -- each; a read of the address being written is never used)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_CG_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_CG_SIN(to_integer(r_w_a));
            r_rc_q <= C_CG_RCP(to_integer(q3_mx(10 downto 2)));
            r_at_q <= C_CG_ATN(to_integer(q9_rt));
            r_sc_q <= C_CG_SEC(to_integer(q9_rt(9 downto 1)));
            r_pl_q <= C_CG_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_sl : process(clk)
    begin
        if rising_edge(clk) then
            sl_q <= sl(to_integer(sl_ra));
            if sl_we = '1' then sl(to_integer(sl_wa)) <= sl_wd; end if;
        end if;
    end process p_sl;

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
            dr(to_integer(rc + 6)) <= '0' & q3_par & q3_hy & q3_hx & q3_sw & std_logic_vector(q3_mx);
            dr_q <= dr(to_integer(rc));
            cr(to_integer(rc + 6)) <= std_logic_vector(q25_c(1)) & std_logic_vector(q25_c(2));
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
        variable v_15 : unsigned(14 downto 0);
        variable v_17 : unsigned(16 downto 0);
        variable v_a  : signed(12 downto 0);
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
                s_disc <= registers_in(6)(2);
                s_line <= registers_in(6)(3);
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
                -- Teeth: 8 steps with hysteresis
                v_d := signed(resize(unsigned(registers_in(1)), 11)) - signed(resize(s_nix & "1000000", 11));
                if v_d > 80 or v_d < -80 then s_nix <= unsigned(registers_in(1)(9 downto 7)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- tooth count: N = 2^(n1 + 2) + 2^n2;  half-depth 0.95 / N;  32768 / N;  the radial zones
            case to_integer(s_nix) is
                when 0      => c_a7 <= to_unsigned(81, 7); c_rn <= to_unsigned(5461, 13); c_n1 <= "00"; c_n2 <= "01";
                when 1      => c_a7 <= to_unsigned(61, 7); c_rn <= to_unsigned(4096, 13); c_n1 <= "01"; c_n2 <= "00";
                when 2      => c_a7 <= to_unsigned(49, 7); c_rn <= to_unsigned(3277, 13); c_n1 <= "01"; c_n2 <= "01";
                when 3      => c_a7 <= to_unsigned(41, 7); c_rn <= to_unsigned(2731, 13); c_n1 <= "01"; c_n2 <= "10";
                when 4      => c_a7 <= to_unsigned(30, 7); c_rn <= to_unsigned(2048, 13); c_n1 <= "10"; c_n2 <= "00";
                when 5      => c_a7 <= to_unsigned(24, 7); c_rn <= to_unsigned(1638, 13); c_n1 <= "10"; c_n2 <= "10";
                when 6      => c_a7 <= to_unsigned(20, 7); c_rn <= to_unsigned(1365, 13); c_n1 <= "10"; c_n2 <= "11";
                when others => c_a7 <= to_unsigned(15, 7); c_rn <= to_unsigned(1024, 13); c_n1 <= "11"; c_n2 <= "00";
            end case;
            v_a := signed(resize(c_a7 & "000", 13));
            c_rin <= to_signed(1788, 13) - v_a;
            c_rb0 <= to_signed(1878, 13) - v_a;
            c_rb1 <= to_signed(1948, 13) - v_a;
            c_zc  <= to_signed(1833, 13) - v_a;
            c_zd  <= to_signed(1998, 13) - v_a;
            c_nph <= 0 - c_ph(25 downto 10);

            -- shared glide for Scale and Drive (1/8 of the gap per field, +-2 deadband) and Camera (+-4): four clocks per channel
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
                        elsif (z_gc(1) = '0' and (s_gd > 16 or s_gd < -16)) or s_gd > 32 or s_gd < -32 then
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

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k (no zc decode on the
            -- multiplier's path).
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" and s_zc(8 downto 5) /= 8
               and s_zc(8 downto 5) /= 9 and s_zc(8 downto 5) /= 10 and s_zc(8 downto 5) /= 11
               and s_zc(8 downto 5) /= 13 and s_zc(8 downto 5) /= 15 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(18 downto 0) <= c_ip;                 z_fb <= resize(s_ys, 21);           -- step per line
                    when 1  => z_fa(19 downto 0) <= c_sx;                 z_fb <= resize(s_w(11 downto 1), 21);   -- to the centre column
                    when 2  => z_fa(20 downto 0) <= c_sy;                 z_fb <= resize(f_half(s_tid), 21);  -- to the centre line
                    when 3  =>                                                                                -- depth^2
                        z_fa(12 downto 3) <= unsigned(registers_in(2)); z_fb(12 downto 3) <= unsigned(registers_in(2));
                    when 4  =>                                                                                -- spread^2
                        z_fa(12 downto 3) <= unsigned(C_KFIX_5); z_fb(12 downto 3) <= unsigned(C_KFIX_5);
                    when 5  => z_fa(14 downto 0) <= c_s;                  z_fb <= resize(0 - c_x0(31 downto 24), 21);   -- Spread * first column
                    when 6  => z_fa(14 downto 0) <= c_s;                  z_fb <= resize(0 - c_y0(31 downto 24), 21);   -- Spread * first row
                    when 7  => z_fa(22 downto 0) <= z_v;                  z_fb <= resize(c_rn, 21);           -- drive: teeth -> turns
                    when 12 => z_fa(8 downto 0)  <= s_vm;                 z_fb <= to_unsigned(900, 21);       -- rate exponent
                    when others => z_fa(15 downto 0) <= z_mt;             z_fb <= to_unsigned(1193046, 21);   -- rate (14)
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(36 downto 0) & '0';
                f_b <= '0' & f_b(20 downto 1);
            end if;

            -- serial shifter for the 2^n scalings (no barrels)
            if z_shn /= 0 then
                z_shn <= z_shn - 1;
                if z_sht = '1' then z_v <= z_v(21 downto 0) & '0';
                else c_dph <= c_dph(20 downto 0) & '0'; end if;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- pitch: e = k1 * 3 octaves (Q8) above 64 lines; 2^24 / pitch; the coverage gain
                    when 2 =>
                        v_15 := resize(s_g1, 15) + shift_left(resize(s_g1, 15), 1);
                        z_e <= '0' & v_15(14 downto 5);
                    when 3 =>
                        r_ex_a <= 0 - z_e(7 downto 0); z_sh <= '0' & z_e(9 downto 8);
                        if z_e(7 downto 0) = 0 then z_nz <= '0'; else z_nz <= '1'; end if;
                        z_m <= to_unsigned(768, 11) + f_lpq(s_tid) - z_e;
                    when 5 => z_mt <= r_ex_q;
                    when 6 =>
                        if z_nz = '1' then v_17 := '0' & z_mt; else v_17 := to_unsigned(65536, 17); end if;
                        case to_integer(z_sh) is
                            when 0      => c_ip <= v_17 & "00";
                            when 1      => c_ip <= '0' & v_17 & '0';
                            when others => c_ip <= "00" & v_17;
                        end case;
                        if z_m(7 downto 0) = 0 then
                            c_n <= z_m(10 downto 8); c_m3 <= '0';
                        elsif z_m(7 downto 0) <= 106 then
                            c_n <= z_m(10 downto 8) + 1; c_m3 <= '1';
                        else
                            c_n <= z_m(10 downto 8) + 1; c_m3 <= '0';
                        end if;
                    when 7 =>
                        case to_integer(s_xs) is
                            when 4      => c_sx <= c_ip & '0';
                            when 3      => c_sx <= resize(c_ip, 20) + resize(c_ip(18 downto 1), 20);
                            when others => c_sx <= resize(c_ip, 20);
                        end case;
                    -- drive: 2^(8 p) / 256 teeth per field (Q23)
                    when 10 => r_ex_a <= s_gp(9 downto 2);
                    when 12 => z_v <= resize(r_ex_q, 23); z_shn <= s_gp(12 downto 10); z_sht <= '1';
                    -- wave phase and train angle (last field's steps)
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    when 21 => c_psi <= c_psi + resize(c_dpsi, 28);
                    -- Camera: gain = K6 / 4 (K6 256 -> 64 = the v0.1.0 x8), Osc x3
                    when 24 =>
                        if s_src = '1' then c_cg <= resize(s_g6(12 downto 5), 10) + resize(s_g6(12 downto 4), 10);
                        else c_cg <= resize(s_g6(12 downto 5), 10); end if;
                    when 60  => c_sy <= f_acc(28 downto 8);
                    when 92  => c_x0 <= to_unsigned(8388608, 32) - f_acc(31 downto 0);
                    when 124 => c_y0 <= to_unsigned(8388608, 32) - f_acc(31 downto 0);
                    when 156 => c_d <= f_acc(25 downto 20);                 -- depth = (k3 / 1023)^2 / 2 turn
                    when 188 => c_s <= f_acc(25 downto 11);                 -- spread = (k5 / 1023)^2 / 2 cycle
                    when 220 => c_si0 <= signed(0 - f_acc(19 downto 0));
                    when 252 => c_sj0 <= signed(0 - f_acc(19 downto 0));
                    when 284 => c_dpsi <= f_acc(35 downto 10);
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh; z_sht <= '0';
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Blend table fill (zone, zc 40 .. ~180; adders only).  For each colour
    -- pair (lo, hi) and cov = 0 .. 8:  T = (8 lo + (hi - lo) cov) / 8.  The two
    -- colours come straight from the palette ROM (0 ground, 1 A, 2 B, 3 accent);
    -- a shade is 3/4 of the way from black.
    --   0 ground-accent   1, 2 accent-A / B   3, 4 A / B-window   5, 6 A / B-shade   7, 8 ground-A / B
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_lo, v_hi : unsigned(1 downto 0);
        variable v_shd : std_logic;
        variable v_y  : signed(12 downto 0);
        variable v_c  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            v_shd := '0';
            case to_integer(t_p) is
                when 0      => v_lo := "00"; v_hi := "11";
                when 1      => v_lo := "11"; v_hi := "01";
                when 2      => v_lo := "11"; v_hi := "10";
                when 3      => v_lo := "01"; if s_disc = '1' then v_hi := "01"; v_shd := '1'; else v_hi := "00"; end if;
                when 4      => v_lo := "10"; if s_disc = '1' then v_hi := "10"; v_shd := '1'; else v_hi := "00"; end if;
                when 5      => v_lo := "01"; v_hi := "01"; v_shd := '1';
                when 6      => v_lo := "10"; v_hi := "10"; v_shd := '1';
                when 7      => v_lo := "00"; v_hi := "01";
                when others => v_lo := "00"; v_hi := "10";
            end case;
            if s_zone = '1' and s_zc = 40 then
                t_run <= '1'; t_p <= (others => '0'); t_st <= (others => '0');
            elsif t_run = '1' then
                t_st <= t_st + 1;
                case to_integer(t_st) is
                    when 0 => r_pl_a <= "00" & s_pal & v_lo;
                    when 1 => r_pl_a <= "00" & s_pal & v_hi;
                    when 2 =>
                        t_ay <= unsigned(r_pl_q(31 downto 22)) & "000";
                        t_ac(1) <= unsigned(r_pl_q(21 downto 14)) & "000";
                        t_ac(2) <= unsigned(r_pl_q(11 downto 4)) & "000";
                        if t_p = 0 then
                            c_gy <= unsigned(r_pl_q(31 downto 22));
                            c_gc(1) <= unsigned(r_pl_q(21 downto 14)); c_gc(2) <= unsigned(r_pl_q(11 downto 4));
                        end if;
                    when 3 =>                                   -- hi: a plain fabric copy of the ROM word
                        t_hy <= signed(resize(unsigned(r_pl_q(31 downto 22)), 13));
                        t_hc(1) <= signed(resize(unsigned(r_pl_q(21 downto 14)), 11));
                        t_hc(2) <= signed(resize(unsigned(r_pl_q(11 downto 4)), 11));
                    when 4 =>                                   -- shaded where the pair asks for it
                        if v_shd = '1' then
                            v_y := t_hy - 64;
                            t_hy <= shift_right(shift_left(v_y, 1) + v_y, 2) + 64;
                            for i in 1 to 2 loop
                                v_c := t_hc(i) - 128;
                                t_hc(i) <= shift_right(shift_left(v_c, 1) + v_c, 2) + 128;
                            end loop;
                        end if;
                    when 5 =>
                        t_dy <= t_hy(10 downto 0) - signed(resize(t_ay(12 downto 3), 11));
                        for i in 1 to 2 loop
                            t_dc(i) <= t_hc(i)(8 downto 0) - signed(resize(t_ac(i)(10 downto 3), 9));
                        end loop;
                        t_k <= (others => '0');
                    when others =>
                        tt_we <= '1'; tt_wa <= t_p & t_k;
                        tt_wd <= "000000" & std_logic_vector(t_ay(12 downto 3)) & std_logic_vector(t_ac(1)(10 downto 3)) & std_logic_vector(t_ac(2)(10 downto 3));
                        t_ay <= unsigned(signed(t_ay) + resize(t_dy, 13));
                        for i in 1 to 2 loop t_ac(i) <= unsigned(signed(t_ac(i)) + resize(t_dc(i), 11)); end loop;
                        t_k <= t_k + 1;
                        if t_k = 8 then
                            t_st <= (others => '0');
                            t_p <= t_p + 1;
                            if t_p = 8 then t_run <= '0'; end if;
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
    -- px) drifting diagonally.  The luma (or the lattice) is smeared along x
    -- by a 1/64 filter and held per gear: on the first line of each cell row
    -- the smeared value at the cell's centre column goes into a RAM indexed by
    -- the column, and is read back for the rest of the cell row.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_x  : unsigned(13 downto 0);
        variable v_d1 : signed(14 downto 0);
        variable v_cp : unsigned(17 downto 0);
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

            -- input sample: the source value (8 bits), held through blanking
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
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 6), 14));
            end if;

            -- hold: write as the stage-1 pixel crosses the cell centre on a cell row's first line
            sl_wa <= "000" & a_x(28 downto 24);
            -- K6 camera gain on the smeared sample (3 stages: the written sample is the smear 2 px before the write
            -- point): h = sample x c_cg / 16 = the source term / 2, max 255 x 765 / 16 < 2^14
            for i in 0 to 3 loop
                cm1_p(i) <= f_gu(c_cg, s_y1(6 + 2 * i), 12) + shift_left(f_gu(c_cg, s_y1(7 + 2 * i), 12), 1);
            end loop;
            cm2_a <= resize(cm1_p(0), 14) + shift_left(resize(cm1_p(1), 14), 2);
            cm2_b <= resize(cm1_p(2), 14) + shift_left(resize(cm1_p(3), 14), 2);
            v_cp := resize(cm2_a, 18) + shift_left(resize(cm2_b, 18), 4);
            sl_wd <= "00" & std_logic_vector(v_cp(17 downto 4));
            if l_first = '1' and g_avid = '1' and a_x(23) = '1' and q2_hx = '1' then sl_we <= '1'; else sl_we <= '0'; end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Cell coordinates and the wave engine.  One 32-bit accumulator per
    -- axis = {cell index, phase within the cell}; ls_* hold the NEXT line's
    -- start values.  Spread * index is accumulated at each cell wrap.
    ---------------------------------------------------------------------------
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            ls_ayn <= ls_ay + resize(c_sy, 32);
            if s_zone = '1' and s_zc = 600 then
                ls_ay <= c_y0; ls_sj <= c_sj0; ls_first <= '1';
            elsif g_start = '1' then
                ls_ay <= ls_ayn;
                if ls_ayn(24) /= ls_ay(24) then
                    ls_sj <= ls_sj + signed(resize(c_s, 20));
                    ls_first <= '1';
                else
                    ls_first <= '0';
                end if;
            end if;

            -- stage 1
            s_f1 <= g_start;
            if g_start = '1' then
                a_x <= c_x0;
                l_ay <= ls_ay; l_sj <= ls_sj; l_first <= ls_first;
            elsif g_avid = '1' then
                a_x <= a_x + resize(c_sx, 32);
            end if;
        end if;
    end process p_acc;

    p_wave : process(clk)
        variable v_a   : unsigned(19 downto 0);
        variable v_pos : unsigned(11 downto 0);
        variable v_th  : unsigned(13 downto 0);
        variable v_p   : unsigned(15 downto 0);
        variable v_st  : signed(15 downto 0);
        variable v_h   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: Spread * column of the stage-2 pixel
            if s_f1 = '1' then
                q2_si <= c_si0; q2_ixl <= a_x(24);
            elsif a_x(24) /= q2_ixl then
                q2_si <= q2_si + signed(resize(c_s, 20)); q2_ixl <= a_x(24);
            end if;
            q_ix(2) <= a_x(28 downto 24);
            for i in 3 to 8 loop q_ix(i) <= q_ix(i - 1); end loop;
            -- 3: magnitudes (Radial) and the sum (Linear)
            v_a := unsigned(abs(q2_si)); q3_asi <= v_a(18 downto 0);
            v_a := unsigned(abs(l_sj));  q3_asj <= v_a(18 downto 0);
            q3_lin <= unsigned(q2_si(15 downto 0)) + unsigned(l_sj(15 downto 0));
            -- 4: compare
            if q3_asi >= q3_asj then q4_ge <= '1'; else q4_ge <= '0'; end if;
            q4_asi <= q3_asi(16 downto 0); q4_asj <= q3_asj(16 downto 0); q4_lin <= q3_lin;
            -- 5: octagonal metric max + min / 2 (Radial) or column + row (Linear), less the phase
            if s_rad = '0' then
                q5_xa <= q4_lin + c_nph; q5_xb <= (others => '0');
            elsif q4_ge = '1' then
                q5_xa <= q4_asi(15 downto 0) + c_nph; q5_xb <= q4_asj(16 downto 1);
            else
                q5_xa <= q4_asj(15 downto 0) + c_nph; q5_xb <= q4_asi(16 downto 1);
            end if;
            -- 6: theta (16 bits per cycle; the top 14 are used)
            q6_th <= q5_xa + q5_xb;
            -- 7: wave ROM address: quarter-wave fold; Snap = the first sixteenth stretched over the quarter
            v_th := q6_th(15 downto 2);
            v_pos := v_th(11 downto 0);
            if v_th(12) = '1' then v_pos := not v_pos; end if;
            if s_snap = '0' then r_w_a <= v_pos(11 downto 3);
            elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
            else r_w_a <= v_pos(9 downto 1); end if;
            s_neg(7) <= v_th(13);
            for i in 8 to 12 loop s_neg(i) <= s_neg(i - 1); end loop;
            -- 8: ROM; 9: fabric copy
            q9_w <= r_w_q(9 downto 0);
            -- 10 .. 12: Depth * wave as gated sums (turns Q14)
            q10_p0 <= f_gu(q9_w, c_d(0), 12) + shift_left(f_gu(q9_w, c_d(1), 12), 1);
            q10_p1 <= f_gu(q9_w, c_d(2), 12) + shift_left(f_gu(q9_w, c_d(3), 12), 1);
            q10_p2 <= f_gu(q9_w, c_d(4), 12) + shift_left(f_gu(q9_w, c_d(5), 12), 1);
            q11_qa <= resize(q10_p0, 14) + shift_left(resize(q10_p1, 14), 2);
            q11_p2 <= q10_p2;
            v_p := resize(q11_qa, 16) + shift_left(resize(q11_p2, 16), 4);
            q12_aw <= v_p(15 downto 3);

            -- source hold read: address at 9, RAM at 10, fabric at 11, x2 at 12 (gained before the hold by K6:
            -- Video x0 .. x32, default x16; Osc x3 that)
            sl_ra <= "000" & q_ix(8);
            q11_src <= unsigned(sl_q(13 downto 0));
            q12_st <= q11_src & '0';
            -- 13: the state = +-Depth * wave + source (14 bits per turn)
            v_st := signed(resize(q12_aw, 16));
            if s_neg(12) = '1' then v_st := -v_st; end if;
            q13_stt <= v_st + signed(resize(q12_st, 16));
            -- 14: the hub swell
            v_h := q13_stt(15 downto 4);
            if v_h > 420 then q14_hs <= to_signed(420, 10);
            elsif v_h < -230 then q14_hs <= to_signed(-230, 10);
            else q14_hs <= v_h(9 downto 0); end if;
        end if;
    end process p_wave;

    ---------------------------------------------------------------------------
    -- [E] The look: polar engine, gear profile, zones, coverage, blend table
    ---------------------------------------------------------------------------
    p_look : process(clk)
        variable v_vx, v_vy : unsigned(10 downto 0);
        variable v_23 : unsigned(22 downto 0);
        variable v_q  : unsigned(11 downto 0);
        variable v_odd : std_logic;
        variable v_s1, v_s2 : unsigned(13 downto 0);
        variable v_tf : unsigned(12 downto 0);
        variable v_xq : signed(9 downto 0);
        variable v_df : unsigned(10 downto 0);
        variable v_r9 : unsigned(8 downto 0);
        variable v_r  : signed(12 downto 0);
        variable v_t16 : signed(15 downto 0);
        variable v_u  : unsigned(9 downto 0);
        variable v_c0, v_d0, v_de, v_d : signed(12 downto 0);
        variable v_dm : signed(11 downto 0);
        variable v_cc : signed(11 downto 0);
        variable v_cv : signed(12 downto 0);
        variable v_cov : unsigned(3 downto 0);
        variable v_par : std_logic;
    begin
        if rising_edge(clk) then
            -- 2: fold into the first quadrant (|d| - 1/2 unit on the negative side: symmetric about the centre)
            if a_x(23) = '1' then q2_vx <= a_x(22 downto 12); else q2_vx <= not a_x(22 downto 12); end if;
            if l_ay(23) = '1' then q2_vy <= l_ay(22 downto 12); else q2_vy <= not l_ay(22 downto 12); end if;
            q2_hx <= not a_x(23); q2_hy <= not l_ay(23);
            q2_par <= a_x(24) xor l_ay(24);
            -- 3: and into the first octant
            if q2_vy > q2_vx then
                q3_mx <= q2_vy; q3_mn <= q2_vx; q3_sw <= '1';
            else
                q3_mx <= q2_vx; q3_mn <= q2_vy; q3_sw <= '0';
            end if;
            q3_hx <= q2_hx; q3_hy <= q2_hy; q3_par <= q2_par;
            -- 4: reciprocal ROM; 5: fabric copy
            q4_mn <= q3_mn; q5_mn <= q4_mn;
            q5_rc <= r_rc_q(11 downto 0);
            -- 6 .. 9: ratio = min * recip(max) as gated sums
            for k in 0 to 5 loop
                q6_p(k) <= f_gu(q5_mn, q5_rc(2 * k), 13) + shift_left(f_gu(q5_mn, q5_rc(2 * k + 1), 13), 1);
            end loop;
            q7_a <= resize(q6_p(0), 15) + shift_left(resize(q6_p(1), 15), 2);
            q7_b <= resize(q6_p(2), 15) + shift_left(resize(q6_p(3), 15), 2);
            q7_c <= resize(q6_p(4), 15) + shift_left(resize(q6_p(5), 15), 2);
            q8_t <= resize(q7_a, 19) + shift_left(resize(q7_b, 19), 4);
            q8_c <= q7_c;
            v_23 := resize(q8_t, 23) + shift_left(resize(q8_c, 23), 8);
            if v_23(22 downto 19) /= 0 then q9_rt <= (others => '1'); else q9_rt <= v_23(18 downto 9); end if;
            -- 10: atan and secant ROMs; 11: fabric copies, with the octant data from its delay ring
            q11_at <= r_at_q(10 downto 0);
            q11_sc <= r_sc_q(9 downto 0);
            q11_mx <= unsigned(dr_q(10 downto 0));
            q11_sw <= dr_q(11); q11_hx <= dr_q(12); q11_hy <= dr_q(13);
            s_par(11) <= dr_q(14);
            for i in 12 to 19 loop s_par(i) <= s_par(i - 1); end loop;
            -- 12 .. 15: radius = max * (1 + (sec - 1)) as gated sums
            for k in 0 to 4 loop
                q12_p(k) <= f_gu(q11_mx, q11_sc(2 * k), 13) + shift_left(f_gu(q11_mx, q11_sc(2 * k + 1), 13), 1);
            end loop;
            q12_mx <= q11_mx; q13_mx <= q12_mx; q14_mx <= q13_mx; q15_mx <= q14_mx;
            q13_a <= resize(q12_p(0), 15) + shift_left(resize(q12_p(1), 15), 2);
            q13_b <= resize(q12_p(2), 15) + shift_left(resize(q12_p(3), 15), 2);
            q13_c <= resize(q12_p(4), 15) + shift_left(resize(q12_mx, 15), 3);
            q14_t <= resize(q13_a, 19) + shift_left(resize(q13_b, 19), 4);
            q14_c <= q13_c;
            v_23 := resize(q14_t, 23) + shift_left(resize(q14_c, 23), 8);
            q15_r <= v_23(22 downto 11);
            -- 12: the angle, unfolded by octant (14 bits per turn)
            if q11_sw = '1' then v_q := not ('0' & q11_at); else v_q := '0' & q11_at; end if;
            v_odd := q11_hx xor q11_hy;
            if v_odd = '1' then v_q := not v_q; end if;
            q12_phi <= q11_hy & v_odd & v_q;
            -- 13: in the gear's frame: alternate gears turn opposite ways
            if s_par(12) = '1' then q13_phg <= q12_phi + c_psi(27 downto 14);
            else q13_phg <= q12_phi - c_psi(27 downto 14); end if;
            -- 14: tooth phase = N * angle (+ half a tooth on odd gears); spoke phase = angle - state
            case c_n1 is
                when "00"   => v_s1 := q13_phg(11 downto 0) & "00";
                when "01"   => v_s1 := q13_phg(10 downto 0) & "000";
                when "10"   => v_s1 := q13_phg(9 downto 0) & "0000";
                when others => v_s1 := q13_phg(8 downto 0) & "00000";
            end case;
            case c_n2 is
                when "01"   => v_s2 := q13_phg(12 downto 0) & '0';
                when "10"   => v_s2 := q13_phg(11 downto 0) & "00";
                when "11"   => v_s2 := q13_phg(10 downto 0) & "000";
                when others => v_s2 := (others => '0');
            end case;
            v_s1 := v_s1 + v_s2;
            q14_tau <= (v_s1(13) xor s_par(13)) & v_s1(12 downto 0);
            q14_phs <= q13_phg(11 downto 0) - unsigned(q13_stt(11 downto 0));
            -- 15: tooth: flank coordinate, clipped (trapezoid); spoke: distance in angle from the nearest spoke
            if q14_tau(13) = '1' then v_tf := not q14_tau(12 downto 0); else v_tf := q14_tau(12 downto 0); end if;
            v_xq := to_signed(256, 10) - signed(resize(v_tf(12 downto 4), 10));
            if v_xq > 127 then q15_xc <= to_signed(127, 8); s_fl(15) <= '0';
            elsif v_xq < -128 then q15_xc <= to_signed(-128, 8); s_fl(15) <= '0';
            else q15_xc <= v_xq(7 downto 0); s_fl(15) <= '1'; end if;
            for i in 16 to 18 loop s_fl(i) <= s_fl(i - 1); end loop;
            if q14_phs(11) = '1' then v_df := not q14_phs(10 downto 0); else v_df := q14_phs(10 downto 0); end if;
            if v_df(10) = '1' then q15_d8 <= (others => '1'); else q15_d8 <= v_df(9 downto 2); end if;
            -- 15: the hub's radii (they swell with the state)
            q15_rh1 <= to_signed(C_RH1, 13) + resize(q14_hs, 13);
            q15_rco <= to_signed(C_RCO, 13) + resize(q14_hs, 13);
            q15_zb  <= to_signed(C_ZB, 13) + resize(q14_hs, 13);
            q15_za  <= to_signed(C_ZA, 13) + resize(shift_right(q14_hs, 1), 13);
            -- 16, 17: tooth height T = a * clip (gated sums); spoke offset u = radius * angle
            for k in 0 to 2 loop
                q16_t(k) <= f_gs(q15_xc, c_a7(2 * k), 10) + shift_left(f_gs(q15_xc, c_a7(2 * k + 1), 10), 1);
            end loop;
            q16_t(3) <= f_gs(q15_xc, c_a7(6), 10);
            v_t16 := (resize(q16_t(0), 16) + shift_left(resize(q16_t(1), 16), 2))
                   + shift_left(resize(q16_t(2), 16) + shift_left(resize(q16_t(3), 16), 2), 4);
            q17_tt <= v_t16(14 downto 4);
            if q15_r(11) = '1' then v_r9 := (others => '1'); else v_r9 := q15_r(10 downto 2); end if;
            for k in 0 to 3 loop
                q16_u(k) <= f_gu(v_r9, q15_d8(2 * k), 11) + shift_left(f_gu(v_r9, q15_d8(2 * k + 1), 11), 1);
            end loop;
            q17_pu <= (resize(q16_u(0), 17) + shift_left(resize(q16_u(1), 17), 2))
                    + shift_left(resize(q16_u(2), 17) + shift_left(resize(q16_u(3), 17), 2), 4);
            -- 16: radial distances to every ring edge, zone flags, the tooth-line terms
            v_r := signed(resize(q15_r, 13));
            q16_da <= v_r - C_RH0;
            q16_db <= v_r - q15_rh1;
            q16_c1 <= v_r - q15_rco;
            q16_c2 <= c_rin - v_r;
            q16_d1 <= v_r - c_rb0;
            q16_d2 <= c_rb1 - v_r;
            q16_e  <= to_signed(C_E0, 13) - v_r;
            q16_f  <= signed(resize(q15_mx & '0', 13)) - v_r - (4095 - C_E0);
            if v_r < q15_za then q16_za <= '1'; else q16_za <= '0'; end if;
            if v_r < q15_zb then q16_zb <= '1'; else q16_zb <= '0'; end if;
            if v_r < c_zc then q16_zc <= '1'; else q16_zc <= '0'; end if;
            if v_r < c_zd then q16_zd <= '1'; else q16_zd <= '0'; end if;
            -- 17: the zone and its radial distance
            if q16_c1 < q16_c2 then v_c0 := q16_c1; else v_c0 := q16_c2; end if;
            if q16_d1 < q16_d2 then v_d0 := q16_d1; else v_d0 := q16_d2; end if;
            if q16_za = '1' then    q17_dr <= q16_da; q17_zn <= "000";
            elsif q16_zb = '1' then q17_dr <= q16_db; q17_zn <= "001";
            elsif q16_zc = '1' then q17_dr <= v_c0;   q17_zn <= "010";
            elsif q16_zd = '1' then q17_dr <= v_d0;   q17_zn <= "011";
            else                    q17_dr <= v_d0;   q17_zn <= "100"; end if;
            q17_e <= q16_e; q17_f <= q16_f;
            -- 18: own profile, the neighbour's profile mirrored into this cell, the spoke edge
            q18_do <= q17_e + resize(q17_tt, 13);
            q18_dn <= q17_f - resize(q17_tt, 13);
            v_u := q17_pu(16 downto 7) - resize(q17_pu(16 downto 9), 10);
            if s_disc = '1' and s_line = '1' then q18_dsp <= signed(resize(v_u, 12));
            else q18_dsp <= signed(resize(v_u, 12)) - C_WS; end if;
            q18_dr <= q17_dr; q18_zn <= q17_zn;
            -- 19: tooth zone: whichever gear is nearer (flanks are steep: halve the radial distance there);
            -- window zone: inside the ring and off the spoke
            if q18_do >= q18_dn then v_de := q18_do; q19_own <= '1'; else v_de := q18_dn; q19_own <= '0'; end if;
            if s_fl(18) = '1' then q19_de <= shift_right(v_de, 1); else q19_de <= v_de; end if;
            if (s_disc = '1' and s_line = '1') or resize(q18_dsp, 13) < q18_dr then q19_dc <= resize(q18_dsp, 13);
            else q19_dc <= q18_dr; end if;
            q19_dr <= q18_dr; q19_zn <= q18_zn;
            -- 20: the pixel's one signed distance (clamped) and its colour pair
            v_par := s_par(19);
            case q19_zn is
                when "100"  => v_d := q19_de;
                when "010"  => v_d := q19_dc;
                when others => v_d := q19_dr;
            end case;
            if v_d > 1023 then q20_d <= to_signed(1023, 11);
            elsif v_d < -1024 then q20_d <= to_signed(-1024, 11);
            else q20_d <= v_d(10 downto 0); end if;
            if q19_zn = "100" then
                if (q19_own xor v_par) = '1' then q20_pr <= to_unsigned(7, 4); else q20_pr <= to_unsigned(8, 4); end if;
            elsif s_line = '1' then
                if q19_zn(1) = '0' then q20_pr <= to_unsigned(0, 4);
                elsif v_par = '1' then q20_pr <= to_unsigned(8, 4);
                else q20_pr <= to_unsigned(7, 4); end if;
            else
                case q19_zn is
                    when "000"  => q20_pr <= to_unsigned(0, 4);
                    when "001"  => if v_par = '1' then q20_pr <= to_unsigned(2, 4); else q20_pr <= to_unsigned(1, 4); end if;
                    when "010"  => if v_par = '1' then q20_pr <= to_unsigned(4, 4); else q20_pr <= to_unsigned(3, 4); end if;
                    when others => if v_par = '1' then q20_pr <= to_unsigned(6, 4); else q20_pr <= to_unsigned(5, 4); end if;
                end case;
            end if;
            -- 21: coverage gain mantissa (1 or 1.5)
            if c_m3 = '1' then q21_dm <= resize(q20_d, 12) + resize(shift_right(q20_d, 1), 12);
            else q21_dm <= resize(q20_d, 12); end if;
            q21_pr <= q20_pr;
            -- 22: and its power of two
            case to_integer(c_n) is
                when 0      => q22_cc <= q21_dm;
                when 1      => q22_cc <= shift_right(q21_dm, 1);
                when 2      => q22_cc <= shift_right(q21_dm, 2);
                when 3      => q22_cc <= shift_right(q21_dm, 3);
                when 4      => q22_cc <= shift_right(q21_dm, 4);
                when 5      => q22_cc <= shift_right(q21_dm, 5);
                when others => q22_cc <= shift_right(q21_dm, 6);
            end case;
            q22_pr <= q21_pr;
            -- 23: coverage in eighths: Fill = a step at the edge; Line = a 3.5-px line on the edge
            if s_line = '1' then v_cv := to_signed(18, 13) - resize(abs(q22_cc), 13);
            else v_cv := to_signed(4, 13) + resize(q22_cc, 13); end if;
            if v_cv < 0 then v_cov := (others => '0');
            elsif v_cv > 8 then v_cov := to_unsigned(8, 4);
            else v_cov := unsigned(v_cv(3 downto 0)); end if;
            tt_ra <= q22_pr & v_cov;
            -- 24: blend table; 25: fabric copy; the ground colour outside the generated avid
            if s_gav_sr(22) = '0' then
                q25_y <= c_gy; q25_c <= c_gc;
            else
                q25_y <= unsigned(tt_q(25 downto 16));
                q25_c(1) <= unsigned(tt_q(15 downto 8));
                q25_c(2) <= unsigned(tt_q(7 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(22);
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
            -- running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
            -- back to 8 x ground once 8 blank samples are in, so a glitched sample can't offset the colours for good
            if s_bah = "00000000" then
                s_bu <= c_gc(1) & "000"; s_bv <= c_gc(2) & "000";
            else
                s_bu <= s_bu + q25_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q25_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q25_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            -- filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- blanking gate
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

end architecture cogwork;
