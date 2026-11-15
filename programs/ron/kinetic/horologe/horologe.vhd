-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Horologe v0.1.1 -- Kinetic.
--
-- A wall of analog clocks whose hands are choreographed by a travelling wave ("A Million Times", Humans
-- since 1982): lines, waves and standing figures run across the grid.
--
--   element  a clock of pitch P (square grid or brick); state = the angle of hand 1 (Q14 turns, signed)
--   theta    = Spread * index - phase;  index: Linear = column + row, Radial = octagonal distance from the centre
--   hand 1   = Depth * sin(theta) + source      (Snap: ticks between the 8 compass points)
--   hand 2   = 1/2 turn + (1 - Hands / 32) * hand 1        (locked in line .. mirrored .. counter-running x3)
--   hands    capsules from the clock centre: two affine forms each (along, across the hand), Q11 px.  Per clock
--            and line a small serial machine fetches |cos|, |sin| and forms the seeds (18 add-shift steps); per
--            pixel the forms advance by one add.  Round caps: octagonal metric max + 3/8 min past the tip.
--   dial     a face disc and an accent hub from r^2 = x^2 + y^2 (a squares ROM), anti-aliased by a shift
--   colour   top layer over the layer under it, through a per-field table of 9 pre-blended steps per pair
--            (ground, face, hand 2 = B, hand 1 = A, hub = accent); luma 10 bits, chroma 8 until the output filter
--
--   K1  Scale    clock pitch 56 .. 267 virtual px (exponential, glided)
--   K2  Colour   the 16 shared palettes (was K6; Shape is fixed at 450 = the old default, C_KFIX_2)
--   K3  Depth    swing 0 .. 1.25 turns (square law)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Hands    hand 2 against hand 1: locked in line .. mirrored (centre) .. counter-running at up to x3 (glided)
--   K6  Camera   camera strength 0 .. 100 % (default 50 % = twice the v0.1.0 gain): source term = luma x 0 .. 80
--                (Q14 turns per luma step; 25 % = x20 = v0.1.0), Osc x2.  Applied to the held sample as it is read,
--                through a 256-entry gain table refilled every field by repeated addition (no multiplier).
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Dial     Faces / Hands
--   S10 Grid     Square / Brick
--   S11 Source   Video / Osc  (the luma held per clock adds to the angle)
--   P12 SPREAD   phase step per clock 0 .. 1/2 cycle (square law, glided)
--
-- Sections: [A] timing ID -> virtual-1080 steps, vblank zone, glides, per-field sequencer (serial multiplier)
--   [B] generated avid  [C] source field and the per-clock hold  [D] wave engine  [E] the clocks
--   [F] output conditioning.  One latency (C_LAT = 57) in every mode.  Hardware colour convention (u = Cr).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K2 = Colour (was K6); K2 Shape fixed at 450 (C_KFIX_2).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.horologe_pkg.all;
use work.all;

architecture horologe of program_top is
    constant C_KFIX_2 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(450, 10));   -- K6 camera pass: fixed knob
    constant C_LAT      : integer := 57;      -- = processing_delay_clks
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
    -- Osc lattice y step per field line = (y step * 121) / 256
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
    -- coverage 0 .. 8 from a biased r^2 margin: (v >> sr), clamped
    function f_cov(v : signed(17 downto 0); sr : unsigned(3 downto 0)) return unsigned is
        variable w : unsigned(16 downto 0);
    begin
        if v(17) = '1' then return to_unsigned(0, 4); end if;
        w := shift_right(unsigned(v(16 downto 0)), to_integer(sr));
        if w(16 downto 3) /= 0 then return to_unsigned(8, 4); end if;
        return '0' & w(2 downto 0);
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (registered address and data, then a fabric register)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(9 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a : unsigned(8 downto 0) := (others => '0');
    signal r_cs_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_sq_a : unsigned(8 downto 0) := (others => '0');
    signal r_sq_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- [C] source hold: one smeared luma sample per clock, two banks by row parity (the row being drawn reads
    -- its bank; its last line writes the other bank for the row below)
    type t_h8 is array (0 to 511) of std_logic_vector(7 downto 0);
    signal sl : t_h8 := (others => x"00");
    attribute no_rw_check of sl : signal is true;
    signal sl_ra, sl_wa : unsigned(8 downto 0) := (others => '0');
    signal sl_q, sl_wd  : std_logic_vector(7 downto 0) := (others => '0');
    -- K6 camera gain: glided knob (Q3), per-field gain (5/16 K6, x2 for Osc), and the gain table
    -- gt(s) = s x gain / 16, refilled every field by repeated addition (no multiplier)
    signal s_g6  : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal c_cg  : unsigned(9 downto 0) := to_unsigned(160, 10);
    type t_gt is array (0 to 255) of std_logic_vector(15 downto 0);
    signal gt : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt : signal is true;
    signal gt_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal gt_acc : unsigned(17 downto 0) := (others => '0');
    signal gt_we : std_logic := '0';
    signal sl_we : std_logic := '0';
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lxr : t_r16 := (others => (others => '0'));         -- local x: stage 2 -> 17
    attribute no_rw_check of lxr : signal is true;
    signal lxr_q : std_logic_vector(15 downto 0) := (others => '0');
    signal cvr : t_r16 := (others => (others => '0'));         -- face / hub coverage: stage 8 -> 47
    attribute no_rw_check of cvr : signal is true;
    signal cvr_q : std_logic_vector(15 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] blend table, refilled every field: address = under (2) & top (2) & coverage (4);
    -- word = {Y 10, U 8, V 8} = (8 under + coverage (top - under) + 4) / 8
    type t_tt is array (0 to 255) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(7 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_run : std_logic := '0';
    signal t_ph  : unsigned(1 downto 0) := (others => '0');
    signal t_g, t_k : unsigned(3 downto 0) := (others => '0');
    signal t_uy, t_ty : unsigned(9 downto 0) := (others => '0');
    signal t_ay, t_sy : signed(13 downto 0) := (others => '0');
    type t_u8a is array (1 to 2) of unsigned(7 downto 0);
    type t_s12a is array (1 to 2) of signed(11 downto 0);
    signal t_uc, t_tc : t_u8a := (others => (others => '0'));
    signal t_ac, t_sc : t_s12a := (others => (others => '0'));
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
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(499 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gh : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_dial, s_brick, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(1 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

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
    -- Shape (K2 in v0.1.0) is fixed at C_KFIX_2 = 450: hand length and half-width as fractions of the pitch (Q10)
    constant C_LF : unsigned(9 downto 0) := to_unsigned(440 - 450 / 8 - 450 / 32, 10);   -- 370
    constant C_WF : unsigned(9 downto 0) := to_unsigned(18 + 450 / 16, 10);              -- 46
    signal z_p2, z_ta, z_tb : unsigned(17 downto 0) := (others => '0');
    signal z_loc : signed(18 downto 0) := (others => '0');
    signal z_k   : signed(7 downto 0) := (others => '0');
    signal z_sk  : signed(19 downto 0) := (others => '0');
    signal z_nrun, z_wr : std_logic := '0';

    -- per-field constants
    signal c_pw   : unsigned(19 downto 0) := to_unsigned(30720 * 8, 20);   -- pitch, Q11 (c_p = Q8 virtual px)
    signal c_p    : unsigned(16 downto 0);
    signal c_ph2  : unsigned(9 downto 0);                               -- pitch / 2, Q2
    signal c_cls  : unsigned(1 downto 0) := (others => '0');           -- line pitch class
    signal c_L    : unsigned(11 downto 0) := to_unsigned(700, 12);     -- hand length, Q4
    signal c_w    : unsigned(9 downto 0) := to_unsigned(80, 10);       -- hand half-width, Q4
    signal c_rh   : unsigned(9 downto 0) := to_unsigned(120, 10);      -- hub radius, Q4
    signal c_wb   : unsigned(10 downto 0) := to_unsigned(88, 11);      -- half-width + half a coverage ramp, Q4
    signal c_tf, c_th : unsigned(17 downto 0) := (others => '0');      -- r^2 thresholds (biased)
    signal c_srf, c_srh : unsigned(3 downto 0) := to_unsigned(4, 4);   -- their anti-alias shifts
    signal c_kh   : unsigned(6 downto 0) := to_unsigned(64, 7);        -- hands ratio
    signal c_d    : unsigned(7 downto 0) := (others => '0');           -- depth, Q7 turns
    signal c_s    : unsigned(22 downto 0) := (others => '0');          -- spread per clock, Q24 cycles
    signal c_xst  : unsigned(9 downto 0) := to_unsigned(256, 10);      -- x step per pixel, Q8
    signal c_yst  : unsigned(11 downto 0) := to_unsigned(256, 12);     -- y step per line, Q8
    signal c_xmp, c_x2mp, c_ymp : signed(18 downto 0) := (others => '0');
    type t_u17a is array (0 to 1) of unsigned(16 downto 0);
    type t_s8a  is array (0 to 1) of signed(7 downto 0);
    type t_s20b is array (0 to 1) of signed(19 downto 0);
    signal c_xloc : t_u17a := (others => (others => '0'));             -- column layout at pixel 0: plain, brick-shifted
    signal c_xk   : t_s8a := (others => (others => '0'));
    signal c_xsk  : t_s20b := (others => (others => '0'));               -- Spread * column, Q14 cycles
    signal c_wrap0 : std_logic_vector(0 to 1) := "00";
    signal c_ph   : unsigned(25 downto 0) := (others => '0');          -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    -- colours: 0 ground, 1 face, 2 hand 2 (B), 3 hand 1 (A), 4 hub (accent); luma 10 bits, chroma 8 bits
    signal c_cy : t_u10(0 to 4) := (others => to_unsigned(64, 10));
    type t_c8 is array (0 to 4) of unsigned(7 downto 0);
    signal c_cu, c_cv : t_c8 := (others => to_unsigned(128, 8));

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
    signal s_ext : unsigned(5 downto 0) := (others => '0');
    signal s_y1 : unsigned(13 downto 0) := (others => '0');

    -- [D] wave engine: line registers (the NEXT line's start values) and stage-1 accumulators
    signal ls_sy  : signed(19 downto 0) := (others => '0');
    signal ls_loc : unsigned(16 downto 0) := (others => '0');
    signal ls_k   : signed(7 downto 0) := (others => '0');
    signal ls_n1  : unsigned(16 downto 0) := (others => '0');
    signal ls_n2  : signed(18 downto 0) := (others => '0');
    signal ls_ly, l_ly : signed(10 downto 0) := (others => '0');       -- local y of the line, Q2
    signal l_y2   : unsigned(15 downto 0) := (others => '0');
    signal l_last, l_wbank : std_logic := '0';
    signal a_skx, a_sy : signed(19 downto 0) := (others => '0');
    signal a_loc  : unsigned(16 downto 0) := (others => '0');
    signal a_k, a_k0 : signed(7 downto 0) := (others => '0');
    signal a_ky0, a_wrap, a_nw : std_logic := '0';
    signal s_nw   : std_logic_vector(2 to 17) := (others => '0');
    signal q2_ask, q2_asy : unsigned(18 downto 0) := (others => '0');
    signal q2_lin : unsigned(13 downto 0) := (others => '0');
    signal q2_lx  : signed(10 downto 0) := (others => '0');
    signal q3_ge  : std_logic := '0';
    signal q3_ask, q3_asy : unsigned(14 downto 0) := (others => '0');
    signal q3_lin, q3_y, q4_xa, q4_xb, q5_th : unsigned(13 downto 0) := (others => '0');
    signal s_neg  : std_logic_vector(6 to 11) := (others => '0');
    signal q8_w   : unsigned(9 downto 0) := (others => '0');
    signal q9_p0, q9_p1, q9_p2, q9_p3 : unsigned(11 downto 0) := (others => '0');
    signal q10_qa, q10_qb : unsigned(13 downto 0) := (others => '0');
    signal q11_aw  : unsigned(14 downto 0) := (others => '0');
    signal q11_st  : unsigned(15 downto 0) := (others => '0');
    signal q12_al  : signed(16 downto 0) := (others => '0');
    signal q13_a   : signed(13 downto 0) := (others => '0');           -- hand 1, Q11 turns
    signal h14_p0, h14_p1, h14_p2, h14_p3, h15_qa, h15_qb, h16_pr : signed(15 downto 0) := (others => '0');
    signal q14_a1, q15_a1, q16_a1, q17_a1, q17_a2 : unsigned(10 downto 0) := (others => '0');
    signal q17_lx  : signed(10 downto 0) := (others => '0');
    signal k8, k9, k10 : unsigned(7 downto 0) := (others => '0');
    signal b8 : std_logic := '0';

    -- [E] the clock machine (per clock and line) and the pixel forms
    signal m_pc   : unsigned(5 downto 0) := (others => '1');
    signal m_pre, m_go : std_logic := '0';
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal m_v, l_lyn : signed(10 downto 0) := (others => '0');
    signal m_f2   : unsigned(8 downto 0) := (others => '0');
    type t_q2 is array (1 to 2) of std_logic_vector(1 downto 0);
    signal m_q    : t_q2 := (others => "00");
    type t_u9a is array (1 to 2) of unsigned(8 downto 0);
    signal m_c, m_s : t_u9a := (others => (others => '0'));
    type t_s20a is array (1 to 2) of signed(19 downto 0);
    signal r_al, r_ac : t_s20a := (others => (others => '0'));        -- shifting multiplicands: x, then +-y
    type t_u12a is array (1 to 2) of unsigned(11 downto 0);
    signal p_sal, p_sac : t_u12a := (others => (others => '0'));
    type t_s21a is array (1 to 2) of signed(20 downto 0);
    signal g_al, g_ac, p_al, p_ac : t_s21a := (others => (others => '0'));
    signal p_sneg : std_logic_vector(1 to 2) := "00";
    type t_s14a is array (1 to 2) of signed(13 downto 0);
    type t_u13a is array (1 to 2) of unsigned(12 downto 0);
    type t_u14a is array (1 to 2) of unsigned(13 downto 0);
    type t_u4a  is array (1 to 2) of unsigned(3 downto 0);
    signal q43_al, q44_u : t_s14a := (others => (others => '0'));
    signal q43_ac, q44_ac, q45_mx, q45_mn : t_u13a := (others => (others => '0'));
    signal q44_beh : std_logic_vector(1 to 2) := "00";
    signal q46_d  : t_u14a := (others => (others => '0'));
    signal q47_c  : t_u4a := (others => (others => '0'));
    signal q5_x2  : unsigned(15 downto 0) := (others => '0');
    signal q6_r2  : unsigned(16 downto 0) := (others => '0');
    signal q7_vf, q7_vh : signed(17 downto 0) := (others => '0');
    signal q8_cf, q8_ch, q47_cf, q47_ch : unsigned(3 downto 0) := (others => '0');
    signal q50_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q50_c  : t_u8 := (others => to_unsigned(128, 8));

    -- [F] output conditioning
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel.  v0.1.1: only taps 0 .. 8 are flip-flops; taps 40 and 47
    -- ride the cvr delay ring (written as taps 1 and 8, 39 clocks to the fabric copy) and 48 .. 56 follow tap 47
    -- in flip-flops (37 fewer flip-flops, bit-identical)
    type t_bit_shift is array (0 to 8) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');
    signal g40, g47   : std_logic := '0';
    signal g_t        : std_logic_vector(48 to C_LAT - 1) := (others => '0');

begin

    c_p   <= c_pw(19 downto 3);
    c_ph2 <= c_pw(19 downto 10);
    m_go  <= s_nw(17) or m_pre;

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, the hold, delay rings (one write statement each; none
    -- ever reads the address it is writing)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_HG_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_HG_SIN(to_integer(r_w_a));
            r_cs_q <= C_HG_CS(to_integer(r_cs_a));
            r_sq_q <= C_HG_SQ(to_integer(r_sq_a));
            r_pl_q <= C_HG_PAL(to_integer(r_pl_a));
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

    -- K6 gain table: filled in the vblank zone while zc = 768 .. 1023 (address = zc's low byte, gt(n) = n x c_cg / 16
    -- by one add per entry; the sum is held at 0 outside the window).  Read every clock straight from the hold's
    -- read data (sl_q, stage 9): gt_q at stage 10 takes the place of v0.1.0's fabric copy of the hold.  The fill
    -- runs early in the vblank, long before the first line's clocks are read, so a fill-time read is never used.
    gt_we <= '1' when s_zone = '1' and s_zc(10 downto 8) = "011" else '0';
    p_gt : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt(to_integer(unsigned(sl_q)));
            if gt_we = '1' then gt(to_integer(s_zc(7 downto 0))) <= "00" & std_logic_vector(gt_acc(17 downto 4)); end if;
        end if;
    end process p_gt;

    p_gfill : process(clk)
    begin
        if rising_edge(clk) then
            if gt_we = '1' then gt_acc <= gt_acc + resize(c_cg, 18);
            else gt_acc <= (others => '0'); end if;
        end if;
    end process p_gfill;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            lxr(to_integer(rc + 13)) <= "00000" & std_logic_vector(q2_lx);
            lxr_q <= lxr(to_integer(rc));
            cvr(to_integer(rc + 37)) <= "000000" & s_gav_sr(8) & s_gav_sr(1) & std_logic_vector(q8_cf) & std_logic_vector(q8_ch);
            cvr_q <= cvr(to_integer(rc));
            g40 <= cvr_q(8); g47 <= cvr_q(9);
            cr(to_integer(rc + 6)) <= std_logic_vector(q50_c(1)) & std_logic_vector(q50_c(2));
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
        variable v_l  : signed(18 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_e  : unsigned(10 downto 0);
        variable v_w  : unsigned(9 downto 0);
        variable v_wm : unsigned(9 downto 0);
        variable v_r  : unsigned(2 downto 0);
        variable v_y13 : unsigned(12 downto 0);
        variable v_c11 : unsigned(10 downto 0);
        variable v_cg  : unsigned(9 downto 0);
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
                s_dial  <= registers_in(6)(2);
                s_brick <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); s_sd <= '1';
                    when 3      => s_oxs <= to_unsigned(153, 9); s_sd <= '0';
                    when others => s_oxs <= to_unsigned(102, 9); s_sd <= '0';
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(1)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(1)(9 downto 6)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

            -- shared glide for Scale, Spread, Hands (1/8 of the gap per field, +-2 deadband) and Camera (+-4): four clocks each
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "00"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case z_gc is
                            when "00"   => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when "01"   => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                            when "10"   => s_gm <= s_gh; s_gr <= unsigned(registers_in(4));
                            when others => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif (z_gc /= 3 and (s_gd > 16 or s_gd < -16)) or s_gd > 32 or s_gd < -32 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case z_gc is
                            when "00"   => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when "01"   => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when "10"   => s_gh <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 3 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" and s_zc(8 downto 5) /= 6
               and s_zc(8 downto 5) /= 8 and s_zc(8 downto 5) /= 9 and s_zc(8 downto 5) /= 10
               and s_zc(8 downto 5) /= 11 and s_zc(8 downto 5) /= 13 and s_zc(8 downto 5) /= 15 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(4625, 24);      -- pitch exponent
                    when 1  =>                                                                                -- depth^2
                        z_fa(12 downto 3) <= unsigned(registers_in(2)); z_fb(12 downto 3) <= unsigned(registers_in(2));
                    when 2  => z_fa(12 downto 0) <= s_gp;                 z_fb <= resize(s_gp, 24);           -- spread^2
                    when 3  => z_fa(16 downto 0) <= c_p;                  z_fb <= resize(c_p, 24);            -- pitch^2
                    when 4  => z_fa(16 downto 0) <= c_p;                  z_fb <= resize(C_LF, 24);           -- hand length
                    when 5  => z_fa(16 downto 0) <= c_p;                  z_fb <= resize(C_WF, 24);           -- hand half-width
                    when 7  => z_fa(9 downto 0)  <= c_rh;                 z_fb <= resize(c_rh, 24);           -- hub radius^2
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
                if z_sht = '0' then c_pw  <= c_pw(18 downto 0) & '0';
                else                c_dph <= c_dph(20 downto 0) & '0'; end if;
            end if;

            -- grid origins: the centre clock sits on the screen centre; step back to the first line / pixel
            if z_nrun = '1' then
                if z_loc(18) = '1' then
                    z_loc <= z_loc + signed(resize(c_p, 19));
                    z_k   <= z_k - 1;
                    z_sk  <= z_sk - signed(resize(c_s(22 downto 10), 20));
                else
                    z_nrun <= '0';
                end if;
            end if;
            if z_loc + c_xmp >= 0 then z_wr <= '1'; else z_wr <= '0'; end if;

            if s_zone = '1' then
                v_l := signed(resize(c_p(16 downto 1), 19));
                case to_integer(s_zc) is
                    -- wave phase (last field's step): one adder, add or subtract
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    -- palette: ground, A, B, accent (chroma kept to 8 bits), then the face = (7 ground + A) / 8
                    when 22 => r_pl_a <= "00" & s_pal & "00";
                    when 25 =>
                        c_cy(0) <= unsigned(r_pl_q(31 downto 22)); c_cu(0) <= unsigned(r_pl_q(21 downto 14)); c_cv(0) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "01";
                    when 28 =>
                        c_cy(3) <= unsigned(r_pl_q(31 downto 22)); c_cu(3) <= unsigned(r_pl_q(21 downto 14)); c_cv(3) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "10";
                    when 31 =>
                        c_cy(2) <= unsigned(r_pl_q(31 downto 22)); c_cu(2) <= unsigned(r_pl_q(21 downto 14)); c_cv(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "11";
                    when 34 =>
                        c_cy(4) <= unsigned(r_pl_q(31 downto 22)); c_cu(4) <= unsigned(r_pl_q(21 downto 14)); c_cv(4) <= unsigned(r_pl_q(11 downto 4));
                    when 36 =>
                        v_y13 := shift_left(resize(c_cy(0), 13), 3) - resize(c_cy(0), 13) + resize(c_cy(3), 13);
                        c_cy(1) <= v_y13(12 downto 3);
                        v_c11 := shift_left(resize(c_cu(0), 11), 3) - resize(c_cu(0), 11) + resize(c_cu(3), 11);
                        c_cu(1) <= v_c11(10 downto 3);
                        v_c11 := shift_left(resize(c_cv(0), 11), 3) - resize(c_cv(0), 11) + resize(c_cv(3), 11);
                        c_cv(1) <= v_c11(10 downto 3);

                    -- pitch: e = k1 * 2.26 octaves (Q8), P = 56 * 2^e
                    when 60 => z_e <= f_acc(26 downto 16);
                    when 61 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 63 => z_mt <= r_ex_q;
                    when 64 =>
                        c_pw <= resize(z_mt & '0', 20) + resize(z_mt, 20) + resize(z_mt(15 downto 1), 20);
                        z_shn <= z_sh; z_sht <= '0';
                    when 80 =>
                        c_xst <= shift_left(resize(s_xs, 10), 7); c_yst <= s_ys;
                        if s_ys = 256 then c_cls <= "00"; elsif s_ys >= 900 then c_cls <= "10"; else c_cls <= "01"; end if;
                        c_kh <= s_gh(12 downto 6);
                        -- Camera: gain = 5/16 K6 (K6 256 -> 80 = the v0.1.0 x20), Osc x2
                        v_cg := resize(s_g6(12 downto 5), 10) + resize(s_g6(12 downto 7), 10);
                        if s_src = '1' then c_cg <= v_cg(8 downto 0) & '0'; else c_cg <= v_cg; end if;
                    when 81 =>
                        c_xmp  <= signed(resize(c_xst, 19)) - signed(resize(c_p, 19));
                        c_x2mp <= signed(resize(c_xst & '0', 19)) - signed(resize(c_p, 19));
                        c_ymp  <= signed(resize(c_yst, 19)) - signed(resize(c_p, 19));
                    when 83 =>
                        v_e := z_e + 305;
                        c_srf <= to_unsigned(3, 4) + resize(v_e(9 downto 8), 4) + resize(c_cls, 4);
                    when 92 =>                       -- depth = (k3 / 1023)^2 * 1.25 turns
                        v_t := shift_left(resize(f_acc(25 downto 0), 29), 2) + resize(f_acc(25 downto 0), 29);
                        c_d <= v_t(28 downto 21);
                    when 124 => c_s <= f_acc(25 downto 3);
                    when 156 => z_p2 <= f_acc(33 downto 16);
                    when 157 =>                      -- dial: 2 R^2 with R = 0.459 P  (27 / 64 P^2)
                        z_ta <= shift_right(z_p2, 2) + shift_right(z_p2, 3);
                        z_tb <= shift_right(z_p2, 5) + shift_right(z_p2, 6);
                    when 158 => z_ta <= z_ta + z_tb;
                    when 159 => c_tf <= z_ta + shift_left(to_unsigned(4, 18), to_integer(c_srf));
                    when 188 => c_L <= f_acc(25 downto 14);
                    when 220 =>
                        case c_cls is
                            when "00"   => v_wm := to_unsigned(24, 10);
                            when "01"   => v_wm := to_unsigned(36, 10);
                            when others => v_wm := to_unsigned(64, 10);
                        end case;
                        v_w := f_acc(23 downto 14);
                        if v_w < v_wm then c_w <= v_wm; else c_w <= v_w; end if;
                    when 221 =>
                        c_rh <= c_w + shift_right(c_w, 3) + 24;
                        case c_cls is
                            when "00"   => c_wb <= resize(c_w, 11) + 8;
                            when "01"   => c_wb <= resize(c_w, 11) + 16;
                            when others => c_wb <= resize(c_w, 11) + 32;
                        end case;
                    when 222 =>
                        if    c_rh(9 downto 4) >= 24 then v_r := to_unsigned(4, 3);
                        elsif c_rh(9 downto 4) >= 12 then v_r := to_unsigned(3, 3);
                        elsif c_rh(9 downto 4) >= 6  then v_r := to_unsigned(2, 3);
                        elsif c_rh(9 downto 4) >= 3  then v_r := to_unsigned(1, 3);
                        else                              v_r := to_unsigned(0, 3); end if;
                        c_srh <= resize(v_r, 4) + resize(c_cls, 4);
                    when 284 => c_th <= resize(f_acc(19 downto 7), 18) + shift_left(to_unsigned(4, 18), to_integer(c_srh));
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh; z_sht <= '1';
                    -- origins: rows (stored by p_acc at 580), columns, brick-shifted columns
                    when 550 =>
                        z_loc <= v_l - 138240;                             -- 540 lines
                        z_k <= (others => '0'); z_sk <= (others => '0'); z_nrun <= '1';
                    when 580 =>
                        if s_sd = '1' then z_loc <= v_l - 184320;          -- 720 px
                        else z_loc <= v_l - 245760; end if;                -- 960 px
                        z_k <= (others => '0'); z_sk <= (others => '0'); z_nrun <= '1';
                    when 620 =>
                        c_xloc(0) <= unsigned(z_loc(16 downto 0)); c_xk(0) <= z_k; c_xsk(0) <= z_sk; c_wrap0(0) <= z_wr;
                        if s_sd = '1' then z_loc <= to_signed(-184320, 19);
                        else z_loc <= to_signed(-245760, 19); end if;
                        z_k <= (others => '0'); z_sk <= (others => '0'); z_nrun <= '1';
                    when 660 =>
                        c_xloc(1) <= unsigned(z_loc(16 downto 0)); c_xk(1) <= z_k; c_xsk(1) <= z_sk; c_wrap0(1) <= z_wr;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to 8 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            g_t(48) <= g47;
            for i in 49 to C_LAT - 1 loop
                g_t(i) <= g_t(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Blend table fill (zone, from zc 48; adders only).  16 groups (under,
    -- top), 9 steps each: (8 under + k (top - under) + 4) / 8
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_u, v_t : integer range 0 to 4;
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            v_u := to_integer(t_g(3 downto 2));
            v_t := to_integer(t_g(1 downto 0)) + 1;
            if s_zone = '1' and s_zc = 48 then
                t_run <= '1'; t_ph <= "00"; t_g <= (others => '0');
            elsif t_run = '1' then
                case t_ph is
                    when "00" =>
                        t_uy <= c_cy(v_u); t_uc(1) <= c_cu(v_u); t_uc(2) <= c_cv(v_u);
                        t_ty <= c_cy(v_t); t_tc(1) <= c_cu(v_t); t_tc(2) <= c_cv(v_t);
                        t_ph <= "01";
                    when "01" =>
                        t_ay <= signed(resize(t_uy & "100", 14));
                        t_sy <= signed(resize(t_ty, 14)) - signed(resize(t_uy, 14));
                        for i in 1 to 2 loop
                            t_ac(i) <= signed(resize(t_uc(i) & "100", 12));
                            t_sc(i) <= signed(resize(t_tc(i), 12)) - signed(resize(t_uc(i), 12));
                        end loop;
                        t_k <= (others => '0');
                        t_ph <= "10";
                    when others =>
                        tt_we <= '1'; tt_wa <= t_g & t_k;
                        tt_wd <= "000000" & std_logic_vector(t_ay(12 downto 3)) & std_logic_vector(t_ac(1)(10 downto 3)) & std_logic_vector(t_ac(2)(10 downto 3));
                        t_ay <= t_ay + t_sy;
                        for i in 1 to 2 loop t_ac(i) <= t_ac(i) + t_sc(i); end loop;
                        t_k <= t_k + 1;
                        if t_k = 8 then
                            t_ph <= "00";
                            t_g <= t_g + 1;
                            if t_g = 15 then t_run <= '0'; end if;
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
    -- a 1/64 filter; p_wave samples it once per clock into the hold.
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

            -- input sample: source value (8 bits), held through blanking
            s_sav <= data_in.avid; s_savd <= s_sav;
            if data_in.avid = '1' then
                if s_src = '1' then
                    s_sv <= v_o;
                else
                    v_l := unsigned(data_in.y(9 downto 2));
                    if v_l > 16 then s_sv <= v_l - 16; else s_sv <= (others => '0'); end if;
                end if;
            end if;

            -- smear (runs 56 more past the line end)
            if s_sav = '0' and s_savd = '1' then
                s_ext <= to_unsigned(56, 6);
            elsif s_ext /= 0 then
                s_ext <= s_ext - 1;
            end if;
            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
            elsif s_sav = '1' or s_ext /= 0 then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 6), 14));
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Wave engine.  Rows step per line (ls_* hold the NEXT line's start
    -- values), columns per pixel.  Outside the generated avid the stage-1
    -- registers sit on the coming line's first clock, so its angles and seeds
    -- are ready before the line starts.
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_w   : signed(18 downto 0);
        variable v_par : integer range 0 to 1;
    begin
        if rising_edge(clk) then
            ls_n1 <= resize(ls_loc + c_yst, 17);
            ls_n2 <= signed(resize(ls_loc, 19)) + c_ymp;
            ls_ly <= resize(signed(resize(ls_loc(16 downto 6), 12)) - signed(resize(c_ph2, 12)), 11);
            if s_zone = '1' and s_zc = 580 then
                ls_loc <= unsigned(z_loc(16 downto 0)); ls_k <= z_k; ls_sy <= z_sk;
            elsif g_start = '1' then
                if ls_n2(18) = '0' then
                    ls_loc <= unsigned(ls_n2(16 downto 0)); ls_k <= ls_k + 1;
                    ls_sy <= ls_sy + signed(resize(c_s(22 downto 10), 20));
                else
                    ls_loc <= ls_n1;
                end if;
            end if;
            if g_start = '1' then
                l_last <= not ls_n2(18);       -- the next line starts a new row
                l_wbank <= not ls_k(0);
            end if;

            -- stage 1
            if s_brick = '1' and ls_k(0) = '1' then v_par := 1; else v_par := 0; end if;
            if g_avid = '0' then
                a_loc <= c_xloc(v_par); a_k <= c_xk(v_par); a_k0 <= c_xk(v_par); a_skx <= c_xsk(v_par);
                a_wrap <= c_wrap0(v_par); a_nw <= '0';
                a_sy <= ls_sy; a_ky0 <= ls_k(0);
            elsif a_wrap = '1' then
                v_w := signed(resize(a_loc, 19)) + c_xmp;
                a_loc <= unsigned(v_w(16 downto 0));
                a_skx <= a_skx + signed(resize(c_s(22 downto 10), 20));
                a_k   <= a_k + 1;
                a_nw  <= '1'; a_wrap <= '0';
            else
                a_loc <= resize(a_loc + c_xst, 17);
                a_nw  <= '0';
                if signed(resize(a_loc, 19)) + c_x2mp >= 0 then a_wrap <= '1'; else a_wrap <= '0'; end if;
            end if;
        end if;
    end process p_acc;

    p_wave : process(clk)
        variable v_a   : unsigned(19 downto 0);
        variable v_pos : unsigned(11 downto 0);
        variable v_al  : signed(16 downto 0);
        variable v_p   : unsigned(17 downto 0);
        variable v_k   : unsigned(7 downto 0);
        variable v_bk  : std_logic;
        variable v_sel : unsigned(13 downto 0);
        variable v_sn  : signed(16 downto 0);
        variable v_n   : signed(5 downto 0);
        variable v_bo  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_nw(2) <= a_nw;
            for i in 3 to 17 loop s_nw(i) <= s_nw(i - 1); end loop;

            -- 2: magnitudes for the radial metric and the linear sum (14 fraction bits from here on), local x (Q2)
            v_a := unsigned(abs(a_skx)); q2_ask <= v_a(18 downto 0);
            v_a := unsigned(abs(a_sy));  q2_asy <= v_a(18 downto 0);
            q2_lin <= unsigned(a_skx(13 downto 0)) + unsigned(a_sy(13 downto 0));
            q2_lx <= resize(signed(resize(a_loc(16 downto 6), 12)) - signed(resize(c_ph2, 12)), 11);
            -- 3: compare; the phase term
            if q2_ask >= q2_asy then q3_ge <= '1'; else q3_ge <= '0'; end if;
            q3_ask <= q2_ask(14 downto 0); q3_asy <= q2_asy(14 downto 0); q3_lin <= q2_lin;
            q3_y <= 0 - c_ph(25 downto 12);
            -- 4: octagonal metric max + min / 2 (Radial) or column + row (Linear), + the phase term
            if s_rad = '0' then
                v_sel := q3_lin; q4_xb <= (others => '0');
            elsif q3_ge = '1' then
                v_sel := q3_ask(13 downto 0); q4_xb <= q3_asy(14 downto 1);
            else
                v_sel := q3_asy(13 downto 0); q4_xb <= q3_ask(14 downto 1);
            end if;
            q4_xa <= v_sel + q3_y;
            -- 5: theta (14 bits per cycle)
            q5_th <= q4_xa + q4_xb;
            -- 6: wave ROM address: quarter-wave fold
            v_pos := q5_th(11 downto 0);
            if q5_th(12) = '1' then v_pos := not v_pos; end if;
            r_w_a <= v_pos(11 downto 2);
            s_neg(6) <= q5_th(13);
            for i in 7 to 11 loop s_neg(i) <= s_neg(i - 1); end loop;
            -- 7: ROM; 8: fabric copy
            q8_w <= r_w_q(9 downto 0);
            -- 9 .. 11: Depth * wave as gated sums (turns Q14)
            q9_p0 <= f_gu(q8_w, c_d(0), 12) + shift_left(f_gu(q8_w, c_d(1), 12), 1);
            q9_p1 <= f_gu(q8_w, c_d(2), 12) + shift_left(f_gu(q8_w, c_d(3), 12), 1);
            q9_p2 <= f_gu(q8_w, c_d(4), 12) + shift_left(f_gu(q8_w, c_d(5), 12), 1);
            q9_p3 <= f_gu(q8_w, c_d(6), 12) + shift_left(f_gu(q8_w, c_d(7), 12), 1);
            q10_qa <= resize(q9_p0, 14) + shift_left(resize(q9_p1, 14), 2);
            q10_qb <= resize(q9_p2, 14) + shift_left(resize(q9_p3, 14), 2);
            v_p := resize(q10_qa, 18) + shift_left(resize(q10_qb, 18), 4);
            q11_aw <= v_p(17 downto 3);

            -- source hold read: address at 8 (bank = row parity), RAM at 9, gain table (RAM) at 10
            v_k := k8; v_bk := b8;
            if s_gav_sr(5) = '0' then v_k := unsigned(a_k0); v_bk := a_ky0;
            elsif s_nw(7) = '1' then v_k := k8 + 1; end if;
            k8 <= v_k; b8 <= v_bk;
            sl_ra <= v_bk & v_k;
            k9 <= k8; k10 <= k9;
            -- write: at each clock's last pixel on the row's last line, into the other bank for the row below
            -- (brick: the clock below an odd row's boundary is the next column)
            sl_wd <= std_logic_vector(s_y1(13 downto 6));
            if s_brick = '1' and l_wbank = '0' then v_bo := k10 + 1; else v_bo := k10; end if;
            sl_wa <= l_wbank & v_bo;
            if l_last = '1' and s_gav_sr(8) = '1' and (s_nw(9) = '1' or s_gav_sr(7) = '0') then sl_we <= '1'; else sl_we <= '0'; end if;

            -- 11: source term = 4 x the gain table's word (K6: Video x0 .. x80 per luma step, default x40, white =
            -- 0.53 turn; Osc x2): gt = sample x c_cg / 16, max 255 x 638 / 16 < 2^14
            q11_st <= unsigned(gt_q(13 downto 0)) & "00";
            -- 12: hand 1 = +-Depth * wave + source (Q14 turns, signed; 17 bits: up to 3.6 turns never wraps)
            v_al := signed(resize(q11_aw, 17));
            if s_neg(11) = '1' then v_al := -v_al; end if;
            q12_al <= v_al + signed(resize(q11_st, 17));
            -- 13: Snap: tick between the 8 compass points (dwell, a quick move across +-1/64 turn); Q11 turns
            v_sn := q12_al + 256;
            v_n := v_sn(16 downto 11);
            if s_snap = '0' then
                q13_a <= q12_al(16 downto 3);
            elsif v_sn(10 downto 9) /= "00" then
                q13_a <= (v_n + 1) & "00000000";
            else
                q13_a <= v_n & v_sn(8 downto 1);
            end if;
            -- 14 .. 17: hand 2 = 1/2 turn + hand 1 - (Hands * hand 1) / 32 (gated sums, 16 bits modular)
            h14_p0 <= f_gs(q13_a, c_kh(0), 16) + shift_left(f_gs(q13_a, c_kh(1), 16), 1);
            h14_p1 <= f_gs(q13_a, c_kh(2), 16) + shift_left(f_gs(q13_a, c_kh(3), 16), 1);
            h14_p2 <= f_gs(q13_a, c_kh(4), 16) + shift_left(f_gs(q13_a, c_kh(5), 16), 1);
            h14_p3 <= f_gs(q13_a, c_kh(6), 16);
            q14_a1 <= unsigned(q13_a(10 downto 0));
            h15_qa <= h14_p0 + shift_left(h14_p1, 2);
            h15_qb <= h14_p2 + shift_left(h14_p3, 2);
            q15_a1 <= q14_a1;
            h16_pr <= h15_qa + shift_left(h15_qb, 4);
            q16_a1 <= q15_a1;
            q17_a2 <= q16_a1 + 1024 - unsigned(h16_pr(15 downto 5));
            q17_a1 <= q16_a1;
            q17_lx <= signed(lxr_q(10 downto 0));
        end if;
    end process p_wave;

    ---------------------------------------------------------------------------
    -- [E] The clock machine.  At each clock's first pixel (stage 17), and
    -- once before each line for its first clock, 25 clocks of work: fetch
    -- |cos|, |sin| of both hands, then build the seeds of the two forms
    --     along' = |sin| x -+ |cos| y      across' = |cos| x +- |sin| y
    -- by 9 + 9 add-shift steps (LSB first; the multiplicand registers hold
    -- x, then + / - y by the quadrant; the magnitude bit is the adder's
    -- enable).  The true along is along' or its complement by the sine's
    -- sign.  The pixel forms (stage 42) take the seed as the machine ends,
    -- which is when the clock's first pixel arrives, and add the per-pixel
    -- step |sin| dx, |cos| dx on every other pixel.  Clocks are never narrower
    -- than 28 pixels.
    ---------------------------------------------------------------------------
    p_mach : process(clk)
        variable v_c, v_s : unsigned(8 downto 0);
        variable v_h  : integer range 1 to 2;
        variable v_st : unsigned(11 downto 0);
        variable v_m  : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- the coming line's first clock: 48 clocks after a line ends, and in the zone for line 0
            if g_avid = '1' then m_pc <= (others => '0');
            elsif m_pc /= 63 then m_pc <= m_pc + 1; end if;
            if m_pc = 47 or (s_zone = '1' and s_zc = 700) then m_pre <= '1'; else m_pre <= '0'; end if;
            if m_pre = '1' then l_ly <= ls_ly; l_lyn <= -ls_ly; end if;

            case to_integer(m_cnt) is
                when 0 => null;
                when 1 =>
                    r_cs_a <= m_f2;
                    for h in 1 to 2 loop r_al(h) <= resize(m_v, 20); r_ac(h) <= resize(m_v, 20); end loop;
                    m_v <= l_ly;
                    m_cnt <= m_cnt + 1;
                when 2 | 3 =>
                    if m_cnt = 2 then v_h := 1; else v_h := 2; end if;
                    v_c := unsigned(r_cs_q(24 downto 16)); v_s := unsigned(r_cs_q(8 downto 0));
                    if m_q(v_h)(0) = '1' then
                        m_c(v_h) <= v_s; m_s(v_h) <= v_c;
                    else
                        m_c(v_h) <= v_c; m_s(v_h) <= v_s;
                    end if;
                    if m_cnt = 3 then
                        for h in 1 to 2 loop g_al(h) <= (others => '0'); g_ac(h) <= (others => '0'); end loop;
                    end if;
                    m_cnt <= m_cnt + 1;
                when 4 to 12 =>                     -- |sin| x, |cos| x
                    for h in 1 to 2 loop
                        if m_s(h)(0) = '1' then g_al(h) <= g_al(h) + resize(r_al(h), 21); end if;
                        if m_c(h)(0) = '1' then g_ac(h) <= g_ac(h) + resize(r_ac(h), 21); end if;
                        r_al(h) <= shift_left(r_al(h), 1); r_ac(h) <= shift_left(r_ac(h), 1);
                        m_s(h) <= m_s(h)(0) & m_s(h)(8 downto 1);
                        m_c(h) <= m_c(h)(0) & m_c(h)(8 downto 1);
                    end loop;
                    m_cnt <= m_cnt + 1;
                when 13 =>                          -- the forms that take + y (cos and sin of one sign: across)
                    for h in 1 to 2 loop
                        if m_q(h)(0) = '0' then r_ac(h) <= resize(m_v, 20); else r_al(h) <= resize(m_v, 20); end if;
                    end loop;
                    m_v <= l_lyn;
                    m_cnt <= m_cnt + 1;
                when 14 =>                          -- the forms that take - y
                    for h in 1 to 2 loop
                        if m_q(h)(0) = '0' then r_al(h) <= resize(m_v, 20); else r_ac(h) <= resize(m_v, 20); end if;
                    end loop;
                    m_cnt <= m_cnt + 1;
                when 15 to 23 =>                    -- -+ |cos| y, +- |sin| y
                    for h in 1 to 2 loop
                        if m_c(h)(0) = '1' then g_al(h) <= g_al(h) + resize(r_al(h), 21); end if;
                        if m_s(h)(0) = '1' then g_ac(h) <= g_ac(h) + resize(r_ac(h), 21); end if;
                        r_al(h) <= shift_left(r_al(h), 1); r_ac(h) <= shift_left(r_ac(h), 1);
                        m_s(h) <= m_s(h)(0) & m_s(h)(8 downto 1);
                        m_c(h) <= m_c(h)(0) & m_c(h)(8 downto 1);
                    end loop;
                    m_cnt <= m_cnt + 1;
                when others =>
                    m_cnt <= (others => '0');
            end case;
            if m_go = '1' then
                m_v <= q17_lx;
                r_cs_a <= q17_a1(8 downto 0); m_q(1) <= std_logic_vector(q17_a1(10 downto 9));
                m_f2 <= q17_a2(8 downto 0);   m_q(2) <= std_logic_vector(q17_a2(10 downto 9));
                m_cnt <= to_unsigned(1, 5);
            end if;

            -- 42: the pixel forms
            for h in 1 to 2 loop
                if m_cnt = 24 then
                    p_al(h) <= g_al(h); p_ac(h) <= g_ac(h); p_sneg(h) <= m_q(h)(1);
                    for j in 0 to 1 loop
                        if j = 0 then v_m := m_s(h); else v_m := m_c(h); end if;
                        if s_xs = 4 then
                            v_st := shift_left(resize(v_m, 12), 3);
                        elsif s_xs = 3 then
                            v_st := shift_left(resize(v_m, 12), 2) + shift_left(resize(v_m, 12), 1);
                        else
                            v_st := shift_left(resize(v_m, 12), 2);
                        end if;
                        if j = 0 then p_sal(h) <= v_st; else p_sac(h) <= v_st; end if;
                    end loop;
                elsif g40 = '1' then
                    p_al(h) <= p_al(h) + signed(resize(p_sal(h), 21));
                    p_ac(h) <= p_ac(h) + signed(resize(p_sac(h), 21));
                end if;
            end loop;
        end if;
    end process p_mach;

    ---------------------------------------------------------------------------
    -- [E] The look: dial and hub from r^2, capsule distance and coverage of
    -- the two hands, priority blend through the table
    ---------------------------------------------------------------------------
    p_look : process(clk)
        variable v_ax  : unsigned(10 downto 0);
        variable v_f   : signed(20 downto 0);
        variable v_sd  : signed(14 downto 0);
        variable v_cv  : signed(14 downto 0);
        variable v_u   : unsigned(1 downto 0);
        variable v_t   : unsigned(1 downto 0);
        variable v_k   : unsigned(3 downto 0);
        variable v_u1, v_u2, v_u3 : std_logic;
    begin
        if rising_edge(clk) then
            -- 3: squares ROM address: |x| in half px for the pixel, |y| of the coming line in blanking
            if s_gav_sr(0) = '1' then v_ax := unsigned(abs(q2_lx)); else v_ax := unsigned(abs(ls_ly)); end if;
            r_sq_a <= v_ax(9 downto 1);
            -- 4: ROM; 5: fabric copy; the line's y^2 is taken as the line starts
            q5_x2 <= r_sq_q;
            if g_start = '1' then l_y2 <= q5_x2; end if;
            -- 6: r^2;  7: margins to the dial and the hub;  8: coverages
            q6_r2 <= resize(q5_x2, 17) + resize(l_y2, 17);
            q7_vf <= signed(c_tf) - signed(resize(q6_r2, 18));
            q7_vh <= signed(c_th) - signed(resize(q6_r2, 18));
            if s_dial = '1' then q8_cf <= (others => '0'); else q8_cf <= f_cov(q7_vf, c_srf); end if;
            q8_ch <= f_cov(q7_vh, c_srh);

            for h in 1 to 2 loop
                -- 43: along (true sign) and |across|, Q4
                v_f := p_al(h);
                if p_sneg(h) = '1' then v_f := not v_f; end if;
                q43_al(h) <= v_f(20 downto 7);
                v_f := p_ac(h);
                if v_f(20) = '1' then v_f := not v_f; end if;
                q43_ac(h) <= unsigned(v_f(19 downto 7));
                -- 44: distance past the tip
                q44_u(h) <= q43_al(h) - signed(resize(c_L, 14));
                q44_beh(h) <= q43_al(h)(13);
                q44_ac(h) <= q43_ac(h);
                -- 45: behind the centre: nothing; along the hand: |across|; past the tip: octagonal max + 3/8 min
                if q44_beh(h) = '1' then
                    q45_mx(h) <= (others => '1'); q45_mn(h) <= (others => '0');
                elsif q44_u(h)(13) = '1' then
                    q45_mx(h) <= q44_ac(h); q45_mn(h) <= (others => '0');
                elsif unsigned(q44_u(h)(12 downto 0)) >= q44_ac(h) then
                    q45_mx(h) <= unsigned(q44_u(h)(12 downto 0)); q45_mn(h) <= q44_ac(h);
                else
                    q45_mx(h) <= q44_ac(h); q45_mn(h) <= unsigned(q44_u(h)(12 downto 0));
                end if;
                -- 46: distance
                q46_d(h) <= resize(q45_mx(h), 14) + resize(shift_right(q45_mn(h), 2), 14) + resize(shift_right(q45_mn(h), 3), 14);
                -- 47: coverage in eighths over 1 / 2 / 4 virtual px
                v_sd := signed(resize(c_wb, 15)) - signed(resize(q46_d(h), 15));
                case c_cls is
                    when "00"   => v_cv := shift_right(v_sd, 1);
                    when "01"   => v_cv := shift_right(v_sd, 2);
                    when others => v_cv := shift_right(v_sd, 3);
                end case;
                if v_cv < 0 then q47_c(h) <= (others => '0');
                elsif v_cv > 8 then q47_c(h) <= to_unsigned(8, 4);
                else q47_c(h) <= unsigned(v_cv(3 downto 0)); end if;
            end loop;
            q47_cf <= unsigned(cvr_q(7 downto 4)); q47_ch <= unsigned(cvr_q(3 downto 0));

            -- 48: top layer, its coverage, the layer under it (the highest one below that covers at least half)
            if q47_cf(3 downto 2) /= "00" then v_u1 := '1'; else v_u1 := '0'; end if;
            if q47_c(2)(3 downto 2) /= "00" then v_u2 := '1'; else v_u2 := '0'; end if;
            if q47_c(1)(3 downto 2) /= "00" then v_u3 := '1'; else v_u3 := '0'; end if;
            if q47_ch /= 0 then
                v_t := "11"; v_k := q47_ch;
                if v_u3 = '1' then v_u := "11"; elsif v_u2 = '1' then v_u := "10"; elsif v_u1 = '1' then v_u := "01"; else v_u := "00"; end if;
            elsif q47_c(1) /= 0 then
                v_t := "10"; v_k := q47_c(1);
                if v_u2 = '1' then v_u := "10"; elsif v_u1 = '1' then v_u := "01"; else v_u := "00"; end if;
            elsif q47_c(2) /= 0 then
                v_t := "01"; v_k := q47_c(2);
                if v_u1 = '1' then v_u := "01"; else v_u := "00"; end if;
            else
                v_t := "00"; v_k := q47_cf; v_u := "00";
            end if;
            tt_ra <= v_u & v_t & v_k;
            -- 49: table; 50: fabric copy, the ground colour outside the generated avid
            if g47 = '0' then
                q50_y <= c_cy(0); q50_c(1) <= c_cu(0); q50_c(2) <= c_cv(0);
            else
                q50_y <= unsigned(tt_q(25 downto 16));
                q50_c(1) <= unsigned(tt_q(15 downto 8)); q50_c(2) <= unsigned(tt_q(7 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & g47;
        end if;
    end process p_look;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 37 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 30: running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
            -- back to 8 x ground once 8 blank samples are in, so a glitched sample can't offset the colours for good
            if s_bah = "00000000" then
                s_bu <= c_cu(0) & "000"; s_bv <= c_cv(0) & "000";
            else
                s_bu <= s_bu + q50_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q50_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q50_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            -- 31: filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- 32: luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 33: blanking gate
            if g_t(C_LAT - 2) = '1' then
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
    data_out.avid    <= g_t(C_LAT - 1);

end architecture horologe;
