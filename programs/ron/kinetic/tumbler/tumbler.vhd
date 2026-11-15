-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tumbler v0.1.0 -- Kinetic.
--
-- The tumbling-blocks (rhombille) tiling and the reversible Necker cube: every hexagonal cell is a cube of three
-- rhombi in three tones; turn the cube's light and the tones rotate, so the cubes tumble and invert.
--
--   lattice  half column pitch Hc (W = 2 Hc); the row axis is carried in x units (v = m * y, m = Shape), so the
--            three edge families are x = k Hc,  x + v = k W,  v - x = k W  (adders only, no multiplies)
--   cell     lx in [-Hc, Hc), v in [0, 3 Hc):  ea = lx + v - W,  eb = v - lx - W
--            ea < 0 and eb < 0: this row's cube (right face if lx >= 0, else left)
--            exactly one < 0:   the next row's cube, top face;   neither: its left / right face
--   element  a cube; state = its light angle psi (14 bits per turn)
--   theta    = Spread * index - phase;  index: Linear = column + row / 2, Radial = octagonal distance
--   psi      = Depth * wave(theta) + source + face / 3 turn      (wave: sine / snap)
--   colour   psi -> position on the wheel ground -> A -> B -> ground (a per-field 96-step table); Three = snapped
--            to the three pure colours
--   edges    distance to the nearest edge of the pixel's own rhombus (fold of the two forms that bound it);
--            a line (black hairline, or the accent colour) and an inset frame blend toward the line colour,
--            coverage in eighths from the linear forms
--   source   luma smeared along x (1/16), sampled once per cube on the first line of the row above it into a
--            256-entry RAM by half-column index, held for the whole cube
--
--   K1  Scale    column pitch 48 .. 400 virtual px (exponential, glided)
--   K2  Shape    flat tiles .. tall blocks (the lattice slope m = 3 .. 1.125)
--   K3  Inset    a framed panel inside every rhombus, none .. 0.31 Hc
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Spread   phase step per column 0 .. 1/2 cycle (square law)
--   K6  Colour   the 16 shared palettes (ground, A, B = the three face tones; accent = the edge line)
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Edges    Off (black hairline) / On (accent line)
--   S10 Tone     Shaded / Three
--   S11 Source   Video / Osc
--   P12 FLIP     swing of the light angle 0 .. 1.5 turns (square law, glided)
--
-- Sections (the Kinetic chassis, lifted from louver):
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-cube hold
--   [D] lattice accumulators, wave engine
--   [E] the look: faces, edge distances, coverage, colour wheel, blend
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 30) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 520 (C_KFIX_5).
-- 2026-11-15: K6 camera top +25 % (Ron)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tumbler_pkg.all;
use work.all;

architecture tumbler of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(520, 10));   -- K6 camera pass: fixed knob
    constant C_LAT      : integer := 30;      -- = processing_delay_clks
    constant C_NF       : integer := 20;      -- s_gav_sr index read by the final colour stage (23)
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
    function f_gs(x : signed; b : std_logic; w : natural) return signed is
        variable r : signed(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;
    -- 4 << sh: the coverage half-step, folded into the thresholds once per field
    function f_bias(sh : unsigned(1 downto 0)) return unsigned is
    begin
        case to_integer(sh) is
            when 0      => return to_unsigned(4, 13);
            when 1      => return to_unsigned(8, 13);
            when 2      => return to_unsigned(16, 13);
            when others => return to_unsigned(32, 13);
        end case;
    end function;
    -- coverage in eighths: clamp((threshold + bias - d) >> sh, 0, 8)
    function f_cov(thb : unsigned(12 downto 0); d : signed(13 downto 0); sh : unsigned(1 downto 0)) return unsigned is
        variable v : signed(14 downto 0);
        variable r : unsigned(3 downto 0);
    begin
        v := signed(resize(thb, 15)) - resize(d, 15);
        r := "0000";
        if v(14) = '0' then
            case to_integer(sh) is
                when 0 =>
                    if v(13 downto 3) /= 0 then r := "1000"; else r(2 downto 0) := unsigned(v(2 downto 0)); end if;
                when 1 =>
                    if v(13 downto 4) /= 0 then r := "1000"; else r(2 downto 0) := unsigned(v(3 downto 1)); end if;
                when 2 =>
                    if v(13 downto 5) /= 0 then r := "1000"; else r(2 downto 0) := unsigned(v(4 downto 2)); end if;
                when others =>
                    if v(13 downto 6) /= 0 then r := "1000"; else r(2 downto 0) := unsigned(v(5 downto 3)); end if;
            end case;
        end if;
        return r;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (one read port each, registered address and data, then a fabric register)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(9 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_sg_a : unsigned(7 downto 0) := (others => '0');
    signal r_sg_q : unsigned(15 downto 0) := (others => '0');
    signal r_tn_a : unsigned(8 downto 0) := (others => '0');
    signal r_tn_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    -- [C] source hold: one smeared luma sample per cube, by half-column index
    signal sl : t_r16 := (others => (others => '0'));
    attribute no_rw_check of sl : signal is true;
    signal sl_ra, sl_wa : unsigned(7 downto 0) := (others => '0');
    signal sl_q, sl_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal sl_we : std_logic := '0';
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    signal fr : t_r16 := (others => (others => '0'));          -- {face, source}: stage 7 -> 13
    attribute no_rw_check of fr : signal is true;
    signal fr_q : std_logic_vector(15 downto 0) := (others => '0');
    signal wr : t_r16 := (others => (others => '0'));          -- line weight: stage 7 -> 21
    attribute no_rw_check of wr : signal is true;
    signal wr_q : std_logic_vector(15 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] colour wheel, refilled every field: 3 x 32 steps ground -> A -> B -> ground, {Y 10, U 8, V 8}
    type t_tt is array (0 to 127) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(6 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_run, t_ld : std_logic := '0';
    signal t_seg : unsigned(1 downto 0) := (others => '0');
    signal t_i   : unsigned(4 downto 0) := (others => '0');
    signal t_ay, t_sy : signed(15 downto 0) := (others => '0');
    type t_s14 is array (1 to 2) of signed(13 downto 0);
    signal t_ac, t_sc : t_s14 := (others => (others => '0'));
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
    signal s_sd   : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(690 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k2 : unsigned(9 downto 0) := to_unsigned(693, 10);
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_edge, s_three, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    -- K6 Camera: knob (deadband +-4), glided gain k/256 in Q3 (k = 256: the shipped 1x; 512: 2x; 1023: 4x)
    signal s_kr  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_kt  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_kgd : signed(13 downto 0) := (others => '0');
    signal s_kg  : unsigned(12 downto 0) := to_unsigned(4096, 13);
    -- K6 top +25 % (2026-11-15): per-field multiplier gain = 1.25 k / 2 (max 638; the product is taken / 128)
    signal c_kq  : unsigned(9 downto 0) := to_unsigned(320, 10);

    -- field serial multiplier (24 iterations), serial shifter, sequencer scratch
    signal f_a, f_acc, z_fa : unsigned(47 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(23 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : std_logic := '0';
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_v0  : unsigned(18 downto 0) := (others => '0');
    signal z_v   : signed(19 downto 0) := (others => '0');
    signal z_par, z_vrun, z_nrun, z_first : std_logic := '0';
    signal z_sy, z_sx : signed(23 downto 0) := (others => '0');
    signal z_loc, z_t : signed(18 downto 0) := (others => '0');
    signal z_cx  : unsigned(7 downto 0) := (others => '0');

    -- per-field constants
    signal c_pw   : unsigned(19 downto 0) := to_unsigned(25600 * 8, 20);   -- half column pitch Hc, Q11 (c_p = Q8)
    signal c_p    : unsigned(16 downto 0);
    signal c_m8   : unsigned(9 downto 0) := to_unsigned(443, 10);       -- lattice slope m, Q8
    signal c_sig  : unsigned(7 downto 0) := to_unsigned(128, 8);        -- sqrt(1 + m^2), Q6
    signal c_vs   : unsigned(11 downto 0) := to_unsigned(443, 12);      -- v step per line, Q8
    signal c_v3   : unsigned(17 downto 0) := to_unsigned(76800, 18);    -- 3 Hc, Q8
    signal c_vmp  : signed(19 downto 0) := (others => '0');             -- c_vs - c_v3
    signal c_3h   : unsigned(12 downto 0) := to_unsigned(2400, 13);     -- 3 Hc, Q3
    signal c_d    : unsigned(7 downto 0) := (others => '0');            -- depth, Q7 turns
    signal c_s6, c_sh, c_sr : unsigned(16 downto 0) := (others => '0'); -- spread per column, half, per row (Q18)
    signal c_in3  : unsigned(9 downto 0) := (others => '0');            -- inset, Q3 px
    signal c_hw3  : unsigned(6 downto 0) := to_unsigned(8, 7);          -- line half-width, Q3 px
    signal c_shv, c_shs : unsigned(1 downto 0) := "00";
    signal c_inb, c_hwb, c_inSb, c_hwSb : unsigned(12 downto 0) := (others => '0');   -- thresholds + bias
    signal c_xst  : unsigned(9 downto 0) := to_unsigned(256, 10);       -- x step per pixel, Q8
    signal c_xmp, c_x2mp : signed(18 downto 0) := (others => '0');
    signal c_locE, c_locO : unsigned(16 downto 0) := (others => '0');
    signal c_cxE, c_cxO : unsigned(7 downto 0) := (others => '0');
    signal c_sxE, c_sxO : signed(23 downto 0) := (others => '0');
    signal c_wrE, c_wrO : std_logic := '0';
    signal c_ph   : unsigned(25 downto 0) := (others => '0');           -- wave phase, Q26 cycles
    signal c_nph  : unsigned(13 downto 0) := (others => '0');
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    type t_s9  is array (1 to 2) of signed(8 downto 0);
    type t_s11 is array (1 to 2) of signed(10 downto 0);
    -- palette: luma 10 bits, chroma 8 bits (ground, A, B, accent), and the line colour K
    signal c_gy, c_ay, c_by, c_cy, c_ky : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_gc, c_ac, c_bc, c_cc, c_kc : t_u8 := (others => to_unsigned(128, 8));

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
    signal s_ext : unsigned(3 downto 0) := (others => '0');
    signal s_y1  : unsigned(13 downto 0) := (others => '0');

    -- [D] lattice: line registers (the NEXT line's start values) and pixel accumulators (stage 1)
    signal ls_v, ls_n1 : unsigned(17 downto 0) := (others => '0');
    signal ls_n2 : signed(19 downto 0) := (others => '0');
    signal ls_ea, ls_eb, l_ea0, l_eb0 : signed(13 downto 0) := (others => '0');
    signal ls_sy, ls_syn, l_sy, l_syn : signed(23 downto 0) := (others => '0');
    signal ls_par, ls_bf, l_bf, l_first : std_logic := '0';
    signal a_loc  : unsigned(16 downto 0) := (others => '0');
    signal a_sx   : signed(23 downto 0) := (others => '0');
    signal a_cx   : unsigned(7 downto 0) := (others => '0');
    signal a_wrap, a_fp : std_logic := '0';
    signal q2_lx  : signed(12 downto 0) := (others => '0');
    signal q2_ea, q2_eb, q3_fa, q3_fb : signed(13 downto 0) := (others => '0');
    signal q2_sx, q3_sx, q4_sxe, q4_sye : signed(23 downto 0) := (others => '0');
    signal q2_cx, q3_cx : unsigned(7 downto 0) := (others => '0');
    signal q2_fp, q3_x, q3_nxt, q4_x, q5_usev, q6_ge : std_logic := '0';
    signal q3_face, q4_face, q5_face, q6_face, q7_face : unsigned(1 downto 0) := "00";
    signal q3_alx, q4_da, q4_db, q5_ds : unsigned(12 downto 0) := (others => '0');
    signal q4_dh, q5_dh : signed(12 downto 0) := (others => '0');
    signal q5_asx, q5_asy : unsigned(18 downto 0) := (others => '0');
    signal q6_asx, q6_asy : unsigned(14 downto 0) := (others => '0');
    signal q5_lin, q6_lin, q7_xa, q7_xb, q8_th : unsigned(13 downto 0) := (others => '0');
    signal q6_cs, q6_fs, q6_cv, q6_fv : unsigned(3 downto 0) := (others => '0');
    signal q6_src : unsigned(7 downto 0) := (others => '0');
    -- K6 Camera: the held source x the glided gain, three stages of gated sums (8 x 10 bits, stages 7 .. 9)
    -- -> 10 bits (1.0 = 256); the {face, source} ring is written at stage 9 (it was 7)
    signal m7_p0, m7_p1, m7_p2, m7_p3 : unsigned(11 downto 0) := (others => '0');
    signal m8_a, m8_b : unsigned(13 downto 0) := (others => '0');
    signal q9_src : unsigned(9 downto 0) := (others => '0');
    signal q8_face, q9_face : unsigned(1 downto 0) := "00";
    signal q7_wt  : unsigned(4 downto 0) := (others => '0');
    signal s_neg  : std_logic_vector(9 to 14) := (others => '0');
    signal q11_w  : unsigned(9 downto 0) := (others => '0');
    signal q12_p0, q12_p1, q12_p2, q12_p3 : unsigned(11 downto 0) := (others => '0');
    signal q13_qa, q13_qb : unsigned(13 downto 0) := (others => '0');
    signal q13_fs : std_logic_vector(11 downto 0) := (others => '0');
    signal q14_aw : unsigned(14 downto 0) := (others => '0');
    signal q14_st, q15_psi : unsigned(13 downto 0) := (others => '0');

    -- [E] colour
    signal q20_y, q21_y, q22_y : unsigned(9 downto 0) := (others => '0');
    signal q20_c, q21_c, q22_c : t_u8 := (others => (others => '0'));
    signal q21_dy : signed(10 downto 0) := (others => '0');
    signal q21_dc : t_s9 := (others => (others => '0'));
    signal q21_wt : unsigned(4 downto 0) := (others => '0');
    signal q22_w4 : std_logic := '0';
    signal q22_p0y, q22_p1y : signed(12 downto 0) := (others => '0');
    signal q22_p0c, q22_p1c : t_s11 := (others => (others => '0'));
    signal q23_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q23_c  : t_u8 := (others => to_unsigned(128, 8));

    -- [F] output conditioning
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    c_p <= c_pw(19 downto 3);

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, hold, colour wheel, delay rings (one write statement
    -- each; none ever reads the address it is writing)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_TB_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_TB_SIN(to_integer(r_w_a));
            r_sg_q <= C_TB_SIG(to_integer(r_sg_a));
            r_tn_q <= C_TB_TONE(to_integer(r_tn_a));
            r_pl_q <= C_TB_PAL(to_integer(r_pl_a));
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
            fr(to_integer(rc + 2)) <= "0000" & std_logic_vector(q9_face) & std_logic_vector(q9_src);   -- stage 9 -> 13
            fr_q <= fr(to_integer(rc));
            wr(to_integer(rc + 12)) <= "00000000000" & std_logic_vector(q7_wt);
            wr_q <= wr(to_integer(rc));
            cr(to_integer(rc + 6)) <= std_logic_vector(q23_c(1)) & std_logic_vector(q23_c(2));
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
        variable v_t  : unsigned(28 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_q  : unsigned(9 downto 0);
        variable v_h  : unsigned(10 downto 0);
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
                s_snap  <= registers_in(6)(0);
                s_rad   <= registers_in(6)(1);
                s_edge  <= registers_in(6)(2);
                s_three <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_k2    <= unsigned(registers_in(1));
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); s_sd <= '1';
                    when 3      => s_oxs <= to_unsigned(153, 9); s_sd <= '0';
                    when others => s_oxs <= to_unsigned(102, 9); s_sd <= '0';
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(4)(9 downto 6)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide for Scale and Flip (1/8 of the gap per field, +-2 deadband): four clocks per channel
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
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" and s_zc(8 downto 5) /= 10
               and s_zc(8 downto 5) /= 11 and s_zc(8 downto 5) /= 13 and s_zc(8 downto 5) /= 15 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(6270, 24);      -- pitch exponent
                    when 1  => z_fa(11 downto 0) <= s_ys;                 z_fb <= resize(c_m8, 24);           -- v step per line
                    when 2  => z_fa(9 downto 0)  <= c_m8;                 z_fb <= to_unsigned(540, 24);       -- v at the top line
                    when 3  => z_fa(12 downto 0) <= s_gp;                 z_fb <= resize(s_gp, 24);           -- flip^2
                    when 4  =>                                                                                -- spread^2
                        z_fa(12 downto 3) <= unsigned(C_KFIX_5); z_fb(12 downto 3) <= unsigned(C_KFIX_5);
                    when 5  => z_fa(16 downto 0) <= c_p;                  z_fb <= resize(unsigned(registers_in(2)), 24);  -- inset
                    when 6  => z_fa(9 downto 0)  <= c_in3;                z_fb <= resize(c_sig, 24);          -- inset, slanted units
                    when 7  => z_fa(6 downto 0)  <= c_hw3;                z_fb <= resize(c_sig, 24);          -- line, slanted units
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
                if z_sht = '1' then c_pw <= c_pw(18 downto 0) & '0';
                else c_dph <= c_dph(20 downto 0) & '0'; end if;
            end if;

            -- origins: a cube is centred on the screen centre; step back to the first line / pixel
            if z_vrun = '1' then
                if z_v(19) = '1' then
                    z_v   <= z_v + signed(resize(c_v3, 20));
                    z_par <= not z_par;
                    z_sy  <= z_sy - signed(resize(c_sr, 24));
                else
                    z_vrun <= '0';
                end if;
            end if;
            if z_nrun = '1' then
                if z_loc(18) = '1' then
                    z_loc <= z_loc + signed(resize(c_p & '0', 19));
                    z_cx  <= z_cx - 2;
                    z_sx  <= z_sx - signed(resize(c_s6, 24));
                else
                    z_nrun <= '0';
                end if;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- wave phase (last field's step): one adder, add or subtract
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    when 21 =>
                        c_nph <= 0 - c_ph(25 downto 12);
                        c_m8 <= to_unsigned(768, 10) - resize(s_k2(9 downto 1), 10) + resize(s_k2(9 downto 5), 10);
                        r_sg_a <= s_k2(9 downto 2);
                        if s_ys >= 900 then c_shv <= "10"; elsif s_ys >= 384 then c_shv <= "01"; else c_shv <= "00"; end if;
                    -- palette: ground, A, B, accent (chroma kept to 8 bits)
                    when 22 => r_pl_a <= "00" & s_pal & "00";
                    when 24 =>
                        c_sig <= r_sg_q(7 downto 0);
                        if c_shv = "10" or (c_shv = "01" and c_m8 >= 680) then c_shs <= "11";
                        elsif c_shv = "01" or c_m8 >= 680 then c_shs <= "10";
                        else c_shs <= "01"; end if;
                    when 25 =>
                        c_gy <= unsigned(r_pl_q(31 downto 22)); c_gc(1) <= unsigned(r_pl_q(21 downto 14)); c_gc(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "01";
                    when 28 =>
                        c_ay <= unsigned(r_pl_q(31 downto 22)); c_ac(1) <= unsigned(r_pl_q(21 downto 14)); c_ac(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "10";
                    when 31 =>
                        c_by <= unsigned(r_pl_q(31 downto 22)); c_bc(1) <= unsigned(r_pl_q(21 downto 14)); c_bc(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "11";
                    when 34 =>
                        c_cy <= unsigned(r_pl_q(31 downto 22)); c_cc(1) <= unsigned(r_pl_q(21 downto 14)); c_cc(2) <= unsigned(r_pl_q(11 downto 4));
                    -- K6 Camera: knob with a +-4 deadband (ends snap to 0 / 1023), then ease 1/8 of the gap
                    when 40 => s_kr <= unsigned(registers_in(5));
                    when 41 =>
                        v_d := signed(resize(s_kr, 11)) - signed(resize(s_kt, 11));
                        if s_kr < 4 then s_kt <= (others => '0');
                        elsif s_kr > 1019 then s_kt <= (others => '1');
                        elsif v_d > 4 or v_d < -4 then s_kt <= s_kr; end if;
                    when 42 => s_kgd <= signed(resize(s_kt & "000", 14)) - signed(resize(s_kg, 14));
                    when 43 =>
                        if C_SIM_SNAP or (s_kgd < 8 and s_kgd > -8) then s_kg <= s_kt & "000";
                        else s_kg <= unsigned(signed(s_kg) + resize(shift_right(s_kgd, 3), 13)); end if;
                    -- K6 top +25 %: k / 2 + k / 8 (k = s_kg / 8), so source x c_kq / 128 = 1.25 x source x k / 256
                    when 44 => c_kq <= resize(s_kg(12 downto 4), 10) + resize(s_kg(12 downto 6), 10);
                    when 36 =>                       -- the line colour: black hairline, or the accent
                        if s_edge = '1' then c_ky <= c_cy; c_kc <= c_cc;
                        else c_ky <= to_unsigned(64, 10); c_kc <= (others => to_unsigned(128, 8)); end if;

                    -- half column pitch: e = 150 + k1 * 3.06 octaves (Q8), Hc = 16 * 2^e
                    when 60 => z_e <= f_acc(26 downto 16) + 150;
                    when 61 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 63 => z_mt <= r_ex_q;
                    when 64 => c_pw <= "0000" & z_mt; z_shn <= z_sh; z_sht <= '1';
                    when 80 =>
                        c_xst <= s_xs & "0000000";
                        c_v3  <= resize(c_p, 18) + resize(c_p & '0', 18);
                        c_3h  <= resize(c_p(15 downto 5), 13) + resize(c_p(15 downto 5) & '0', 13);
                        if s_edge = '1' then c_hw3 <= resize(c_p(16 downto 9), 7) + 12;
                        else c_hw3 <= to_unsigned(8, 7) + (c_shv & "00"); end if;
                    when 81 =>
                        c_xmp  <= signed(resize(c_xst, 19)) - signed(resize(c_p & '0', 19));
                        c_x2mp <= signed(resize(c_xst & '0', 19)) - signed(resize(c_p & '0', 19));
                        c_hwb  <= resize(c_hw3, 13) + f_bias(c_shv);
                    when 92  => c_vs <= f_acc(19 downto 8);
                    when 100 => c_vmp <= signed(resize(c_vs, 20)) - signed(resize(c_v3, 20));
                    when 124 => z_v0 <= f_acc(18 downto 0);
                    when 156 =>                      -- depth = (p12 / 1023)^2 * 1.5 turns
                        v_t := shift_left(resize(f_acc(25 downto 0), 29), 2) + shift_left(resize(f_acc(25 downto 0), 29), 1);
                        c_d <= v_t(28 downto 21);
                    when 188 => c_s6 <= f_acc(25 downto 9);
                    when 200 =>
                        c_sh <= '0' & c_s6(16 downto 1);
                        if s_rad = '1' then c_sr <= c_s6 - c_s6(16 downto 3);
                        else c_sr <= '0' & c_s6(16 downto 1); end if;
                    when 220 =>                      -- inset = k3 * 0.31 Hc
                        v_q := f_acc(25 downto 16);
                        v_h := resize(v_q(9 downto 1), 11) + resize(v_q(9 downto 3), 11);
                        c_in3 <= v_h(9 downto 0);
                    when 221 => c_inb <= resize(c_in3, 13) + f_bias(c_shv);
                    when 252 => c_inSb <= f_acc(18 downto 6) + f_bias(c_shs);
                    when 284 => c_hwSb <= f_acc(18 downto 6) + f_bias(c_shs);
                    when 300 =>
                        z_v <= 0 - signed(resize(z_v0, 20)); z_par <= '0'; z_sy <= (others => '0'); z_vrun <= '1';
                        if s_sd = '1' then z_loc <= signed(resize(c_p, 19)) - 184320;      -- 720 px
                        else z_loc <= signed(resize(c_p, 19)) - 245760; end if;            -- 960 px
                        z_cx <= (others => '0'); z_sx <= (others => '0'); z_nrun <= '1';
                    when 400 =>
                        z_t <= z_loc - signed(resize(c_p, 19));
                        c_locE <= unsigned(z_loc(16 downto 0)); c_cxE <= z_cx; c_sxE <= z_sx;
                    when 401 =>                      -- odd rows: half a column across
                        if z_t(18) = '0' then
                            c_locO <= unsigned(z_t(16 downto 0)); c_cxO <= z_cx + 1; c_sxO <= z_sx + signed(resize(c_sh, 24));
                        else
                            c_locO <= unsigned(z_loc(16 downto 0)) + c_p; c_cxO <= z_cx - 1; c_sxO <= z_sx - signed(resize(c_sh, 24));
                        end if;
                    when 402 =>
                        if signed(resize(c_locE, 19)) + c_xmp >= 0 then c_wrE <= '1'; else c_wrE <= '0'; end if;
                        if signed(resize(c_locO, 19)) + c_xmp >= 0 then c_wrO <= '1'; else c_wrO <= '0'; end if;
                        z_first <= '1';
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
            if g_start = '1' then z_first <= '0'; end if;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Colour wheel fill (zone, zc 40 .. ~140; adders only): three segments of
    -- 32 steps, ground -> A -> B -> ground
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_fy, v_ty : unsigned(9 downto 0);
        variable v_fc, v_tc : t_u8;
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            case t_seg is
                when "00"   => v_fy := c_gy; v_fc := c_gc; v_ty := c_ay; v_tc := c_ac;
                when "01"   => v_fy := c_ay; v_fc := c_ac; v_ty := c_by; v_tc := c_bc;
                when others => v_fy := c_by; v_fc := c_bc; v_ty := c_gy; v_tc := c_gc;
            end case;
            if s_zone = '1' and s_zc = 40 then
                t_run <= '1'; t_ld <= '1'; t_seg <= "00";
            elsif t_run = '1' then
                if t_ld = '1' then
                    t_ay <= signed(resize(v_fy & "00000", 16));
                    t_sy <= signed(resize(v_ty, 16)) - signed(resize(v_fy, 16));
                    for i in 1 to 2 loop
                        t_ac(i) <= signed(resize(v_fc(i) & "00000", 14));
                        t_sc(i) <= signed(resize(v_tc(i), 14)) - signed(resize(v_fc(i), 14));
                    end loop;
                    t_i <= (others => '0'); t_ld <= '0';
                else
                    tt_we <= '1'; tt_wa <= t_seg & t_i;
                    tt_wd <= "000000" & std_logic_vector(t_ay(14 downto 5)) & std_logic_vector(t_ac(1)(12 downto 5)) & std_logic_vector(t_ac(2)(12 downto 5));
                    t_ay <= t_ay + t_sy;
                    for i in 1 to 2 loop t_ac(i) <= t_ac(i) + t_sc(i); end loop;
                    t_i <= t_i + 1;
                    if t_i = 31 then
                        t_ld <= '1'; t_seg <= t_seg + 1;
                        if t_seg = 2 then t_run <= '0'; end if;
                    end if;
                end if;
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
    -- a 1/16 filter; the lattice stage samples it once per cube.
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

            -- smear (Q6); runs 8 clocks past the line end on the held sample
            if s_sav = '0' and s_savd = '1' then
                s_ext <= to_unsigned(8, 4);
            elsif s_ext /= 0 then
                s_ext <= s_ext - 1;
            end if;
            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
            elsif s_sav = '1' or s_ext /= 0 then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 4), 14));
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Lattice accumulators.  Per line: v (the row coordinate in x units,
    -- wrapping at 3 Hc), the row parity, Spread * row.  Per pixel: the column
    -- offset (wrapping at W), the half-column index, Spread * column.  ls_*
    -- hold the start values of the NEXT line; wrap candidates are pre-registered.
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_w : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            ls_n1  <= resize(ls_v + c_vs, 18);
            ls_n2  <= signed(resize(ls_v, 20)) + c_vmp;
            ls_ea  <= signed(resize(ls_v(17 downto 5), 14)) - signed(resize(c_3h, 14));
            ls_eb  <= signed(resize(ls_v(17 downto 5), 14)) - signed(resize(c_p(15 downto 5), 14));
            ls_syn <= ls_sy + signed(resize(c_sr, 24));
            if s_zone = '1' and s_zc = 400 then
                ls_v <= unsigned(z_v(17 downto 0)); ls_par <= z_par; ls_sy <= z_sy; ls_bf <= '0';
            elsif g_start = '1' then
                if ls_n2(19) = '0' then
                    ls_v <= unsigned(ls_n2(17 downto 0)); ls_par <= not ls_par; ls_sy <= ls_syn; ls_bf <= '1';
                else
                    ls_v <= ls_n1; ls_bf <= '0';
                end if;
            end if;

            if g_start = '1' then
                l_ea0 <= ls_ea; l_eb0 <= ls_eb; l_sy <= ls_sy; l_syn <= ls_syn; l_bf <= ls_bf; l_first <= z_first;
                if ls_par = '1' then
                    a_loc <= c_locO; a_cx <= c_cxO; a_sx <= c_sxO; a_wrap <= c_wrO;
                else
                    a_loc <= c_locE; a_cx <= c_cxE; a_sx <= c_sxE; a_wrap <= c_wrE;
                end if;
                a_fp <= '1';
            elsif g_avid = '1' then
                a_fp <= '0';
                if a_wrap = '1' then
                    v_w := signed(resize(a_loc, 19)) + c_xmp;
                    a_loc <= unsigned(v_w(16 downto 0));
                    a_sx  <= a_sx + signed(resize(c_s6, 24));
                    a_cx  <= a_cx + 2;
                    a_wrap <= '0';
                else
                    a_loc <= resize(a_loc + c_xst, 17);
                    if signed(resize(a_loc, 19)) + c_x2mp >= 0 then a_wrap <= '1'; else a_wrap <= '0'; end if;
                end if;
            end if;
        end if;
    end process p_acc;

    ---------------------------------------------------------------------------
    -- [D] / [E] The pixel pipeline, stages 2 .. 23
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_hc, v_w3, v_t1, v_t2, v_ab, v_r : signed(13 downto 0);
        variable v_x, v_ina, v_inb, v_this, v_we : std_logic;
        variable v_wa  : unsigned(7 downto 0);
        variable v_a20 : signed(19 downto 0);
        variable v_cl, v_cf : unsigned(3 downto 0);
        variable v_fw, v_lw : unsigned(4 downto 0);
        variable v_sel : unsigned(13 downto 0);
        variable v_pos : unsigned(11 downto 0);
        variable v_p   : unsigned(17 downto 0);
        variable v_st  : unsigned(13 downto 0);
        variable v_m18 : unsigned(17 downto 0);
        variable v_al  : signed(15 downto 0);
        variable v_py  : signed(14 downto 0);
        variable v_pc  : signed(12 downto 0);
        variable v_oy  : signed(11 downto 0);
        variable v_oc  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            v_hc := signed(resize(c_p(15 downto 5), 14));
            v_w3 := shift_left(v_hc, 1);

            -- 2: the linear forms (Q3 x units)
            q2_lx <= signed(resize(a_loc(16 downto 5), 13)) - v_hc(12 downto 0);
            q2_ea <= signed(resize(a_loc(16 downto 5), 14)) + l_ea0;
            q2_eb <= l_eb0 - signed(resize(a_loc(16 downto 5), 14));
            q2_sx <= a_sx; q2_cx <= a_cx; q2_fp <= a_fp;

            -- 3: cube and face; the forms folded to the nearest line of their family
            v_x := not q2_lx(12);
            v_ina := q2_ea(13); v_inb := q2_eb(13);
            v_this := v_ina and v_inb;
            q3_x <= v_x; q3_nxt <= not v_this;
            if (v_ina xor v_inb) = '1' then q3_face <= "00";            -- top
            elsif v_this = v_x then q3_face <= "01";                    -- right
            else q3_face <= "10"; end if;                               -- left
            v_t1 := q2_ea + v_hc; v_t2 := v_hc - q2_ea;
            if v_t1(13) = '1' then q3_fa <= q2_ea + v_w3;
            elsif v_t2(13) = '1' then q3_fa <= q2_ea - v_w3;
            else q3_fa <= q2_ea; end if;
            v_t1 := q2_eb + v_hc; v_t2 := v_hc - q2_eb;
            if v_t1(13) = '1' then q3_fb <= q2_eb + v_w3;
            elsif v_t2(13) = '1' then q3_fb <= q2_eb - v_w3;
            else q3_fb <= q2_eb; end if;
            q3_alx <= unsigned(abs(q2_lx));
            q3_sx <= q2_sx; q3_cx <= q2_cx;
            -- the source sample: on a row's first line, the cubes of the NEXT row (the last write of each lands
            -- at the cube's centre column); on the field's first line also this row's cubes
            v_wa := q2_cx; v_we := '0';
            if v_x = '1' then
                if (l_first and q2_fp) = '0' then v_wa := q2_cx + 1; end if;
                v_we := l_first or l_bf;
            elsif q2_fp = '1' then
                v_wa := q2_cx - 1;
                v_we := l_first or l_bf;
            else
                v_we := l_first;
            end if;
            sl_we <= v_we and s_gav_sr(0);
            sl_wa <= v_wa;
            sl_wd <= x"00" & std_logic_vector(s_y1(13 downto 6));

            -- 4: distances to the three line families; the cube's index terms; the hold address
            v_ab := abs(q3_fa); q4_da <= unsigned(v_ab(12 downto 0));
            v_ab := abs(q3_fb); q4_db <= unsigned(v_ab(12 downto 0));
            v_r := v_hc - signed(resize(q3_alx, 14));
            if unsigned(v_hc) < (q3_alx & '0') then q4_dh <= v_r(12 downto 0); else q4_dh <= signed(q3_alx); end if;
            q4_face <= q3_face; q4_x <= q3_x;
            if q3_nxt = '0' then
                q4_sxe <= q3_sx; q4_sye <= l_sy; sl_ra <= q3_cx;
            elsif q3_x = '1' then
                q4_sxe <= q3_sx + signed(resize(c_sh, 24)); q4_sye <= l_syn; sl_ra <= q3_cx + 1;
            else
                q4_sxe <= q3_sx - signed(resize(c_sh, 24)); q4_sye <= l_syn; sl_ra <= q3_cx - 1;
            end if;

            -- 5: the two families that bound this face
            case q4_face is
                when "00"   => if q4_da < q4_db then q5_ds <= q4_da; else q5_ds <= q4_db; end if; q5_usev <= '0';
                when "01"   => q5_ds <= q4_da; q5_usev <= '1';
                when others => q5_ds <= q4_db; q5_usev <= '1';
            end case;
            q5_dh <= q4_dh; q5_face <= q4_face;
            v_a20 := abs(q4_sxe(23 downto 4)); q5_asx <= unsigned(v_a20(18 downto 0));
            v_a20 := abs(q4_sye(23 downto 4)); q5_asy <= unsigned(v_a20(18 downto 0));
            q5_lin <= unsigned(q4_sxe(17 downto 4)) + unsigned(q4_sye(17 downto 4));

            -- 6: coverages in eighths: line and inset frame, slanted and vertical
            q6_cs <= f_cov(c_hwSb, signed(resize(q5_ds, 14)), c_shs);
            q6_fs <= f_cov(c_inSb, signed(resize(q5_ds, 14)), c_shs);
            if q5_usev = '1' then
                q6_cv <= f_cov(c_hwb, resize(q5_dh, 14), c_shv);
                q6_fv <= f_cov(c_inb, resize(q5_dh, 14), c_shv);
            else
                q6_cv <= (others => '0'); q6_fv <= (others => '0');
            end if;
            q6_face <= q5_face;
            q6_src <= unsigned(sl_q(7 downto 0));
            if q5_asx >= q5_asy then q6_ge <= '1'; else q6_ge <= '0'; end if;
            q6_asx <= q5_asx(14 downto 0); q6_asy <= q5_asy(14 downto 0); q6_lin <= q5_lin;

            -- 7: weight toward the line colour (sixteenths): the line, or 6/16 inside the frame
            if q6_cs > q6_cv then v_cl := q6_cs; else v_cl := q6_cv; end if;
            if q6_fs > q6_fv then v_cf := q6_fs; else v_cf := q6_fv; end if;
            v_lw := v_cl & '0';
            v_fw := resize(v_cf(3 downto 1), 5) + resize(v_cf(3 downto 2), 5);
            if v_lw > v_fw then q7_wt <= v_lw; else q7_wt <= v_fw; end if;
            q7_face <= q6_face;
            -- 7 .. 9: K6 Camera: held source x the glided gain (1.25 k / 256 since 2026-11-15, as c_kq / 128)
            m7_p0 <= f_gu(c_kq, q6_src(0), 12) + shift_left(f_gu(c_kq, q6_src(1), 12), 1);
            m7_p1 <= f_gu(c_kq, q6_src(2), 12) + shift_left(f_gu(c_kq, q6_src(3), 12), 1);
            m7_p2 <= f_gu(c_kq, q6_src(4), 12) + shift_left(f_gu(c_kq, q6_src(5), 12), 1);
            m7_p3 <= f_gu(c_kq, q6_src(6), 12) + shift_left(f_gu(c_kq, q6_src(7), 12), 1);
            m8_a <= resize(m7_p0, 14) + shift_left(resize(m7_p1, 14), 2);
            m8_b <= resize(m7_p2, 14) + shift_left(resize(m7_p3, 14), 2);
            q8_face <= q7_face;
            v_m18 := resize(m8_a, 18) + shift_left(resize(m8_b, 18), 4);
            -- up to 255 x 638 / 128 = 1271: kept modulo 1024, exact for psi (x48 and x96 are multiples of 16, and
            -- 1024 x 16 is a whole turn)
            q9_src <= v_m18(16 downto 7);
            q9_face <= q8_face;
            -- index: column + row / 2 (Linear) or octagonal max + min / 2 (Radial), less the phase
            if s_rad = '0' then
                v_sel := q6_lin; q7_xb <= (others => '0');
            elsif q6_ge = '1' then
                v_sel := q6_asx(13 downto 0); q7_xb <= q6_asy(14 downto 1);
            else
                v_sel := q6_asy(13 downto 0); q7_xb <= q6_asx(14 downto 1);
            end if;
            q7_xa <= v_sel + c_nph;
            -- 8: theta (14 bits per cycle)
            q8_th <= q7_xa + q7_xb;
            -- 9: wave ROM address: quarter-wave fold; Snap = the first sixteenth stretched over the quarter
            v_pos := q8_th(11 downto 0);
            if q8_th(12) = '1' then v_pos := not v_pos; end if;
            if s_snap = '0' then r_w_a <= v_pos(11 downto 2);
            elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
            else r_w_a <= v_pos(9 downto 0); end if;
            s_neg(9) <= q8_th(13);
            for i in 10 to 14 loop s_neg(i) <= s_neg(i - 1); end loop;
            -- 10: ROM; 11: fabric copy
            q11_w <= r_w_q(9 downto 0);
            -- 12 .. 14: Depth * wave as gated sums (turns Q14)
            q12_p0 <= f_gu(q11_w, c_d(0), 12) + shift_left(f_gu(q11_w, c_d(1), 12), 1);
            q12_p1 <= f_gu(q11_w, c_d(2), 12) + shift_left(f_gu(q11_w, c_d(3), 12), 1);
            q12_p2 <= f_gu(q11_w, c_d(4), 12) + shift_left(f_gu(q11_w, c_d(5), 12), 1);
            q12_p3 <= f_gu(q11_w, c_d(6), 12) + shift_left(f_gu(q11_w, c_d(7), 12), 1);
            q13_qa <= resize(q12_p0, 14) + shift_left(resize(q12_p1, 14), 2);
            q13_qb <= resize(q12_p2, 14) + shift_left(resize(q12_p3, 14), 2);
            q13_fs <= fr_q(11 downto 0);
            v_p := resize(q13_qa, 18) + shift_left(resize(q13_qb, 18), 4);
            q14_aw <= v_p(17 downto 3);
            -- 14: source term (Video x48: white = 0.64 turn; Osc x96) + the face's third of a turn
            -- (the held source is 10 bits since the K6 camera gain: 256 = the shipped 1x; psi wraps mod a turn)
            v_st := shift_left(resize(unsigned(q13_fs(9 downto 0)), 14), 5) + shift_left(resize(unsigned(q13_fs(9 downto 0)), 14), 4);
            if s_src = '1' then v_st := v_st(12 downto 0) & '0'; end if;
            case q13_fs(11 downto 10) is
                when "01"   => q14_st <= v_st + 5461;
                when "10"   => q14_st <= v_st + 10922;
                when others => q14_st <= v_st;
            end case;
            -- 15: the state: psi = +-Depth * wave + source + face (14 bits per turn)
            v_al := signed(resize(q14_aw, 16));
            if s_neg(14) = '1' then v_al := -v_al; end if;
            v_al := v_al + signed(resize(q14_st, 16));
            q15_psi <= unsigned(v_al(13 downto 0));
            -- 16: tone ROM address; 17: ROM; 18: wheel address; 19: RAM; 20: fabric copy
            r_tn_a <= q15_psi(13 downto 5);
            if s_three = '1' then tt_ra <= r_tn_q(14 downto 8); else tt_ra <= r_tn_q(6 downto 0); end if;
            q20_y <= unsigned(tt_q(25 downto 16));
            q20_c(1) <= unsigned(tt_q(15 downto 8)); q20_c(2) <= unsigned(tt_q(7 downto 0));
            -- 21: line colour - face colour
            q21_dy <= signed(resize(c_ky, 11)) - signed(resize(q20_y, 11));
            q21_y <= q20_y; q21_c <= q20_c;
            q21_wt <= unsigned(wr_q(4 downto 0));
            -- 22: * weight as gated sums
            q22_p0y <= f_gs(q21_dy, q21_wt(0), 13) + shift_left(f_gs(q21_dy, q21_wt(1), 13), 1);
            q22_p1y <= f_gs(q21_dy, q21_wt(2), 13) + shift_left(f_gs(q21_dy, q21_wt(3), 13), 1);
            q22_y <= q21_y; q22_c <= q21_c; q22_w4 <= q21_wt(4);
            -- 23: the blend; the ground colour outside the generated avid; the field's first line is black
            v_py := resize(q22_p0y, 15) + shift_left(resize(q22_p1y, 15), 2);
            v_oy := signed(resize(q22_y, 12)) + resize(shift_right(v_py, 4), 12);
            if s_gav_sr(C_NF) = '0' then q23_y <= c_gy;
            elsif l_first = '1' then q23_y <= to_unsigned(64, 10);
            elsif q22_w4 = '1' then q23_y <= c_ky;
            else q23_y <= unsigned(v_oy(9 downto 0)); end if;
            for i in 1 to 2 loop
                q21_dc(i) <= signed(resize(c_kc(i), 9)) - signed(resize(q20_c(i), 9));
                q22_p0c(i) <= f_gs(q21_dc(i), q21_wt(0), 11) + shift_left(f_gs(q21_dc(i), q21_wt(1), 11), 1);
                q22_p1c(i) <= f_gs(q21_dc(i), q21_wt(2), 11) + shift_left(f_gs(q21_dc(i), q21_wt(3), 11), 1);
                v_pc := resize(q22_p0c(i), 13) + shift_left(resize(q22_p1c(i), 13), 2);
                v_oc := signed(resize(q22_c(i), 10)) + resize(shift_right(v_pc, 4), 10);
                if s_gav_sr(C_NF) = '0' then q23_c(i) <= c_gc(i);
                elsif l_first = '1' then q23_c(i) <= to_unsigned(128, 8);
                elsif q22_w4 = '1' then q23_c(i) <= c_kc(i);
                else q23_c(i) <= unsigned(v_oc(7 downto 0)); end if;
            end loop;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(C_NF);
        end if;
    end process p_pix;

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
                s_bu <= s_bu + q23_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q23_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q23_y;
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

end architecture tumbler;
