-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Pendula v0.1.1 -- Kinetic.
--
-- A lattice of dots, one per cell, each riding its own small orbit, phased by a travelling wave (pendulum-wave
-- machines, the Tusi-couple illusion, water-wave particle orbits).
--
--   element  the dot of one lattice cell (pitch P, cell 0 centred on the screen centre)
--   state    its displacement on its orbit: an ellipse with semi-axes ax, ay turned by psi,
--              (dx, dy) = a (cos(theta + psi), sin(theta + psi)) + b (cos(psi - theta), sin(psi - theta)),
--              a = (ax + ay) / 2, b = (ax - ay) / 2   (two counter-rotating circles)
--   theta    = phase - Spread * n          n = the cell's index in quarter cells: Linear 4 i + 2 j,
--              Radial 2 max + min of (|2 i|, |2 j|); Snap = dwell at the two ends of the orbit and quick flips
--   psi      Axis Same: 0; Fan: n / 32 turn (the axis turns 1/8 turn per column: the Tusi figure at Phase 50 %)
--   dot      radius^2 = R0^2 + luma * (Rlim^2 - R0^2): the held, smeared luma of the cell (or the Osc lattice)
--              grows the dot from the Size radius to the largest that stays inside its cell
--   edges    coverage from (R^2 - r^2) / (2 R), the division a shift by the leading bit pair of R^2; r^2 from a
--              table of squares; 8 steps
--   colour   a per-field table T(mark coverage, colour mix, dot coverage): ground, accent rest mark under the
--              dot, dot colour A .. B by the wave phase (Fan: by theta + psi); luma 10 bits, chroma 8 bits
--              until the output filter
--
--   K1  Scale    cell pitch 32 .. 256 virtual px (exponential, glided)
--   K2  Colour   the 16 shared palettes (was K6; Shape, the orbit, is fixed at 128 = the old default, a flat
--                ellipse: C_KFIX_2)
--   K3  Depth    orbit size 0 .. 69 % of the half cell (square law)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Size     dot radius 25 % .. 87 % of the room the orbit leaves
--   K6  Camera   camera strength 0 .. 100 %: the held luma (and the Osc lattice) x 0 .. 3.75 with a soft knee at
--                white, before the hold; 20 % = x1 = v0.1.0, 50 % = x2.5, 100 % = x3.75 (2026-11-15: x 1.25 everywhere)
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Lattice  Square / Hex (alternate rows offset half a pitch)
--   S10 Axis     Same / Fan
--   S11 Source   Video / Osc
--   P12 PHASE    phase step per column 0 .. 1/2 cycle (square law, glided)
--
-- The file is in lifted sections (the Kinetic chassis, from louver):
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-cell hold
--   [D] wave engine: lattice accumulators -> cell index -> theta -> orbit angles
--   [E] the look: trig table -> displacement -> r^2 -> coverage -> blend table
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 36) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K2 = Colour (was K6); K2 Shape fixed at 128 (C_KFIX_2).
-- 2026-11-15: K6 camera top +25 % (Ron)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.pendula_pkg.all;
use work.all;

architecture pendula of program_top is
    constant C_KFIX_2 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(128, 10));   -- K6 camera pass: fixed knob
    constant C_LAT      : integer := 36;      -- = processing_delay_clks
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
    signal r_ta, r_tb, r_qx, r_qy : unsigned(8 downto 0) := (others => '0');
    signal r_ta_q, r_tb_q, r_qx_q, r_qy_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- [C] per-cell luma hold: two banks by cell-row parity x 128 columns
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal hl : t_r16 := (others => (others => '0'));
    attribute no_rw_check of hl : signal is true;
    signal h_ra, h_wa : unsigned(7 downto 0) := (others => '0');
    signal h_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal h_wd  : std_logic_vector(7 downto 0) := (others => '0');
    -- K6 camera gain: glided knob (Q3), per-field gain (Q8), the gated-sum product stages
    signal s_g6  : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal c_cg  : unsigned(9 downto 0) := to_unsigned(640, 10);
    type t_u12c is array (0 to 3) of unsigned(11 downto 0);
    signal cm1_p : t_u12c := (others => (others => '0'));
    signal cm2_a, cm2_b : unsigned(13 downto 0) := (others => '0');
    signal cm3_x : unsigned(9 downto 0) := (others => '0');
    signal h_we, h_col, h_fl : std_logic := '0';
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    signal lr : t_r16 := (others => (others => '0'));          -- cell-local x: stage 3 -> 18
    attribute no_rw_check of lr : signal is true;
    signal lr_q : std_logic_vector(15 downto 0) := (others => '0');
    type t_r32 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal rr : t_r32 := (others => (others => '0'));          -- dot radius^2 and colour mix: stage 10 -> 23
    attribute no_rw_check of rr : signal is true;
    signal rr_d, rr_q : std_logic_vector(31 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] blend table, refilled every field: address = mark & mix & dot coverage; {Y 10, U 8, V 8}
    type t_tt is array (0 to 511) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(8 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_st  : unsigned(1 downto 0) := (others => '0');
    signal t_m, t_x, t_c : unsigned(2 downto 0) := (others => '0');
    type t_s14a is array (0 to 2) of signed(13 downto 0);
    type t_s17a is array (0 to 2) of signed(16 downto 0);
    type t_s11a is array (0 to 2) of signed(10 downto 0);
    type t_u10a is array (0 to 2) of unsigned(9 downto 0);
    signal tf_u8, tf_d8, t_stp : t_s14a := (others => (others => '0'));
    signal t_acc : t_s17a := (others => (others => '0'));
    signal c_pg, c_pa : t_u10a := (others => (others => '0'));  -- ground, A (luma 10 bits, chroma 8)
    signal c_dba, c_dcg : t_s11a := (others => (others => '0'));  -- B - A, accent - ground
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
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(650 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(458 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_hex, s_fan, s_src : std_logic := '0';
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
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : unsigned(1 downto 0) := (others => '0');
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_d2, z_t, z_f10 : unsigned(9 downto 0) := (others => '0');
    signal z_pm2, z_r0 : unsigned(7 downto 0) := (others => '0');
    signal z_loc : signed(18 downto 0) := (others => '0');
    signal z_k   : signed(7 downto 0) := (others => '0');
    signal z_nrun, z_first : std_logic := '0';

    -- per-field constants
    signal c_pw   : unsigned(19 downto 0) := to_unsigned(122506, 20);  -- pitch, Q10 virtual px
    signal c_p    : unsigned(15 downto 0);                             -- pitch, Q8
    signal c_hp   : unsigned(14 downto 0);                             -- pitch / 2, Q8
    signal c_ph1  : unsigned(7 downto 0);                              -- pitch / 2 in half px
    signal c_aa   : unsigned(7 downto 0) := (others => '0');           -- orbit semi-axis, half px
    signal c_a, c_bm : unsigned(7 downto 0) := (others => '0');        -- the two circles' radii, half px
    signal c_bneg : std_logic := '0';
    signal c_rl   : unsigned(7 downto 0) := to_unsigned(40, 8);        -- largest dot radius, half px
    signal c_rl2, c_r02, c_dr2 : unsigned(13 downto 0) := to_unsigned(100, 14);   -- px^2
    signal c_rm   : unsigned(5 downto 0) := to_unsigned(12, 6);        -- rest mark radius, half px
    signal c_s    : unsigned(12 downto 0) := (others => '0');          -- phase step per index unit, 2^-16 cycle
    signal c_xst  : unsigned(9 downto 0) := to_unsigned(256, 10);      -- x step per pixel, Q8
    signal c_xmp, c_x2mp, c_ymp : signed(18 downto 0) := (others => '0');
    signal c_x0   : unsigned(15 downto 0) := (others => '0');
    signal c_i0   : signed(7 downto 0) := (others => '0');
    signal c_wrap0 : std_logic := '0';
    signal c_ph   : unsigned(25 downto 0) := (others => '0');          -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
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

    -- [D] wave engine: line registers (the NEXT line's start values), pixel accumulators (stage 1), per-line values
    signal ls_loc : unsigned(16 downto 0) := (others => '0');
    signal ls_k   : signed(7 downto 0) := (others => '0');
    signal ls_n1  : unsigned(16 downto 0) := (others => '0');
    signal ls_n2  : signed(18 downto 0) := (others => '0');
    signal ls_t   : signed(17 downto 0) := (others => '0');
    signal a_loc  : unsigned(16 downto 0) := (others => '0');
    signal a_i    : signed(7 downto 0) := (others => '0');
    signal a_wrap, a_nw, q2_nw : std_logic := '0';
    signal l_j    : signed(7 downto 0) := (others => '0');
    signal l_ly   : signed(9 downto 0) := (others => '0');
    signal l_aly  : unsigned(9 downto 0) := (others => '0');
    signal l_ay   : unsigned(7 downto 0) := (others => '0');
    signal l_hx, l_jp, l_srow, l_tp : std_logic := '0';
    signal q2_t1  : signed(17 downto 0) := (others => '0');
    signal q2_i, q3_ie : signed(7 downto 0) := (others => '0');
    signal q3_lx  : signed(9 downto 0) := (others => '0');
    signal q3_x2, q4_lin, q5_lin, q6_n : signed(8 downto 0) := (others => '0');
    signal q4_ax, q5_ax : unsigned(7 downto 0) := (others => '0');
    signal q5_ge  : std_logic := '0';
    type t_u16a is array (natural range <>) of unsigned(15 downto 0);
    signal q7_p   : t_u16a(0 to 3) := (others => (others => '0'));
    signal q7_p4, q8_a, q8_b, q8_c, q9_s, q9_c, q10_th : unsigned(15 downto 0) := (others => '0');
    type t_u5a is array (natural range <>) of unsigned(4 downto 0);
    signal s_n5   : t_u5a(7 to 10) := (others => (others => '0'));
    signal q11_al, q11_be : unsigned(10 downto 0) := (others => '0');
    signal w_t4   : unsigned(3 downto 0);
    signal q6_lum : unsigned(7 downto 0) := (others => '0');
    signal q7_l0, q7_l1, q7_l2 : unsigned(15 downto 0) := (others => '0');
    signal q8_la  : unsigned(17 downto 0) := (others => '0');
    signal q8_lb  : unsigned(15 downto 0) := (others => '0');
    signal q9_pr  : unsigned(13 downto 0) := (others => '0');
    signal q10_re : unsigned(14 downto 0) := (others => '0');

    -- [E] the look
    signal q12_qa, q13_qa, q12_qb, q13_qb : unsigned(1 downto 0) := "00";
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal q14_t  : t_u8a(0 to 3) := (others => (others => '0'));       -- |cos a|, |sin a|, |cos b|, |sin b|
    signal q14_n, q15_n, q16_n, q17_n : std_logic_vector(0 to 3) := (others => '0');
    type t_u10b is array (0 to 15) of unsigned(9 downto 0);
    signal q15_p  : t_u10b := (others => (others => '0'));
    type t_u12a is array (0 to 3) of unsigned(11 downto 0);
    signal q16_a, q16_b : t_u12a := (others => (others => '0'));
    signal q17_x  : t_u8a(0 to 3) := (others => (others => '0'));
    signal q18_dx, q18_dy, q18_lx : signed(9 downto 0) := (others => '0');
    signal q19_ex, q19_ey : signed(10 downto 0) := (others => '0');
    signal q19_alx, q20_alx : unsigned(9 downto 0) := (others => '0');
    signal q20_ge : std_logic := '0';
    signal q21_m  : unsigned(10 downto 0) := (others => '0');
    signal q22_mk, q23_mk, q24_mk, q25_mk : unsigned(2 downto 0) := (others => '0');
    signal q22_sx, q22_sy : unsigned(15 downto 0) := (others => '0');
    signal q23_r2 : unsigned(16 downto 0) := (others => '0');
    signal q23_re : unsigned(14 downto 0) := (others => '0');
    signal q23_mix, q24_mix, q25_mix : unsigned(2 downto 0) := (others => '0');
    signal q24_d  : signed(17 downto 0) := (others => '0');
    signal q24_k  : unsigned(2 downto 0) := (others => '0');
    signal q25_cov : unsigned(2 downto 0) := (others => '0');
    signal q28_t  : std_logic_vector(25 downto 0) := (others => '0');
    signal q29_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q29_c  : t_u8 := (others => to_unsigned(128, 8));

    -- [F] output conditioning
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    c_p   <= c_pw(17 downto 2);
    c_hp  <= c_pw(17 downto 3);
    c_ph1 <= c_pw(17 downto 10);

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, hold, delay rings (one write statement each; none ever
    -- reads the address it is writing, except the hold on a field's first line,
    -- where that one read is discarded)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_PD_EXP2(to_integer(r_ex_a));
            r_ta_q <= C_PD_TRIG(to_integer(r_ta));
            r_tb_q <= C_PD_TRIG(to_integer(r_tb));
            r_qx_q <= C_PD_SQ(to_integer(r_qx));
            r_qy_q <= C_PD_SQ(to_integer(r_qy));
            r_pl_q <= C_PD_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_hl : process(clk)
    begin
        if rising_edge(clk) then
            h_q <= hl(to_integer(h_ra));
            if h_we = '1' then hl(to_integer(h_wa)) <= x"00" & h_wd; end if;
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
            lr(to_integer(rc + 13)) <= "000000" & std_logic_vector(q3_lx);
            lr_q <= lr(to_integer(rc));
            rr(to_integer(rc + 11)) <= rr_d;
            rr_q <= rr(to_integer(rc));
            cr(to_integer(rc + 6)) <= std_logic_vector(q29_c(1)) & std_logic_vector(q29_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;
    -- the colour mix: a triangle of theta (0 = colour A at theta 0, 7 = colour B half a cycle on)
    rr_d(31 downto 18) <= (others => '0');
    rr_d(17 downto 3)  <= std_logic_vector(q10_re);
    w_t4 <= q10_th(15 downto 12) + s_n5(10)(4 downto 1) when s_fan = '1' else q10_th(15 downto 12);   -- Fan: theta + psi
    rr_d(2 downto 0)   <= not std_logic_vector(w_t4(2 downto 0)) when w_t4(3) = '1' else std_logic_vector(w_t4(2 downto 0));

    ---------------------------------------------------------------------------
    -- [A] Controls, vblank zone, glides, per-field sequencer (one small
    -- operation per zone clock; quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_x8 : unsigned(9 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_v  : unsigned(7 downto 0);
        variable v_r  : unsigned(13 downto 0);
        variable v_k5 : unsigned(9 downto 0);
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
                s_hex  <= registers_in(6)(2);
                s_fan  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
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
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide for Scale and Phase (1/8 of the gap per field, +-2 deadband) and Camera (+-4): four clocks per channel
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
               and s_zc(8 downto 5) /= 9 and s_zc(8 downto 5) /= 10
               and s_zc(8 downto 5) /= 11 and s_zc(8 downto 5) /= 13 and s_zc(8 downto 5) /= 15 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(6150, 24);      -- pitch exponent
                    when 1  =>                                                                                -- depth^2
                        z_fa(9 downto 0) <= unsigned(registers_in(2)); z_fb(9 downto 0) <= unsigned(registers_in(2));
                    when 2  => z_fa(9 downto 0) <= z_d2;                  z_fb(7 downto 0) <= z_pm2;          -- orbit semi-axis
                    when 3  => z_fa(7 downto 0) <= c_aa;                  z_fb(9 downto 0) <= z_t;            -- |b|
                    when 4  => z_fa(7 downto 0) <= c_rl;                  z_fb(7 downto 0) <= c_rl;           -- Rlim^2
                    when 5  => z_fa(7 downto 0) <= c_rl;                  z_fb(9 downto 0) <= z_f10;          -- R0
                    when 6  => z_fa(7 downto 0) <= z_r0;                  z_fb(7 downto 0) <= z_r0;           -- R0^2
                    when 7  => z_fa(12 downto 0) <= s_gp;                 z_fb(12 downto 0) <= s_gp;          -- spread = phase^2
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
                    when "01"   => c_pw  <= c_pw(18 downto 0) & '0';
                    when others => c_dph <= c_dph(20 downto 0) & '0';
                end case;
            end if;

            -- lattice origins: cell 0 is centred on the screen centre; step back to the first pixel / line
            if z_nrun = '1' then
                if z_loc(18) = '1' then
                    z_loc <= z_loc + signed(resize(c_p, 19));
                    z_k   <= z_k - 1;
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
                    -- Camera: gain Q8 = K6 up to 512 (256 = x1 = v0.1.0, 512 = x2), then 256 + K6 / 2 (1023 = x3: x4
                    -- turned nearly every dot of a bright picture into the largest dot)
                    when 21 =>
                        if s_g6(12) = '1' then c_cg <= to_unsigned(256, 10) + resize(s_g6(12 downto 4), 10);
                        else c_cg <= s_g6(12 downto 3); end if;
                    -- 2026-11-15: K6 camera top +25 %: gain x 1.25 at every position (0 .. 958 = x3.74)
                    when 23 =>
                        c_cg <= c_cg + resize(c_cg(9 downto 2), 10);
                    -- palette: ground, A, B, accent (chroma kept to 8 bits); B and accent as differences
                    when 22 => r_pl_a <= "00" & s_pal & "00";
                    when 25 =>
                        c_gy <= unsigned(r_pl_q(31 downto 22)); c_gc(1) <= unsigned(r_pl_q(21 downto 14)); c_gc(2) <= unsigned(r_pl_q(11 downto 4));
                        c_pg(0) <= unsigned(r_pl_q(31 downto 22));
                        c_pg(1) <= "00" & unsigned(r_pl_q(21 downto 14)); c_pg(2) <= "00" & unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "01";
                    when 28 =>
                        c_pa(0) <= unsigned(r_pl_q(31 downto 22));
                        c_pa(1) <= "00" & unsigned(r_pl_q(21 downto 14)); c_pa(2) <= "00" & unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "10";
                    when 31 =>
                        c_dba(0) <= signed(resize(unsigned(r_pl_q(31 downto 22)), 11)) - signed(resize(c_pa(0), 11));
                        c_dba(1) <= signed(resize(unsigned(r_pl_q(21 downto 14)), 11)) - signed(resize(c_pa(1), 11));
                        c_dba(2) <= signed(resize(unsigned(r_pl_q(11 downto 4)), 11)) - signed(resize(c_pa(2), 11));
                        r_pl_a <= "00" & s_pal & "11";
                    when 34 =>
                        c_dcg(0) <= signed(resize(unsigned(r_pl_q(31 downto 22)), 11)) - signed(resize(c_pg(0), 11));
                        c_dcg(1) <= signed(resize(unsigned(r_pl_q(21 downto 14)), 11)) - signed(resize(c_pg(1), 11));
                        c_dcg(2) <= signed(resize(unsigned(r_pl_q(11 downto 4)), 11)) - signed(resize(c_pg(2), 11));

                    -- pitch: e = k1 * 3 octaves (Q8), P = 32 * 2^e
                    when 60 => z_e <= f_acc(26 downto 16);
                    when 61 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 63 => z_mt <= r_ex_q;
                    when 64 => c_pw <= "0000" & z_mt; z_shn <= z_sh; z_sht <= "01";
                    when 80 =>
                        v_x8 := resize(s_xs, 10);
                        c_xst <= shift_left(v_x8, 7);
                        z_pm2 <= c_ph1 - 2;
                        v_s := signed(resize(unsigned(C_KFIX_2), 11)) - 512;
                        if v_s(10) = '1' then v_s := -v_s; end if;       -- |k2 - 512| (0 .. 512: sliced, not resized)
                        z_t <= unsigned(v_s(9 downto 0));
                        if unsigned(C_KFIX_2) > 512 then c_bneg <= '1'; else c_bneg <= '0'; end if;
                        v_k5 := unsigned(registers_in(4));
                        z_f10 <= to_unsigned(256, 10) + shift_right(v_k5, 1) + shift_right(v_k5, 3);
                        c_rm <= resize(c_p(15 downto 12), 6) + 5;
                    when 81 =>
                        c_xmp  <= signed(resize(c_xst, 19)) - signed(resize(c_p, 19));
                        c_x2mp <= signed(resize(c_xst & '0', 19)) - signed(resize(c_p, 19));
                        c_ymp  <= signed(resize(s_ys, 19)) - signed(resize(c_p, 19));
                    when 92  => z_d2 <= f_acc(19 downto 10);
                    when 124 =>                      -- orbit semi-axis = depth^2 * 11/16 of the half cell
                        v_v := f_acc(17 downto 10);
                        c_aa <= v_v - shift_right(v_v, 2) - shift_right(v_v, 4);
                    when 156 => c_bm <= f_acc(17 downto 10);
                    when 157 => c_a <= c_aa - c_bm; c_rl <= z_pm2 - c_aa;
                    when 188 => c_rl2 <= f_acc(15 downto 2);
                    when 220 => z_r0 <= f_acc(17 downto 10);
                    when 252 =>
                        v_r := f_acc(15 downto 2);
                        if v_r < 2 then c_r02 <= to_unsigned(2, 14); else c_r02 <= v_r; end if;
                    when 253 => c_dr2 <= c_rl2 - c_r02;
                    when 284 => c_s <= f_acc(25 downto 13);
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh; z_sht <= "11";
                    -- x origin, then y origin (stored in p_acc at 640)
                    when 550 =>
                        if s_sd = '1' then z_loc <= signed(resize(c_hp, 19)) - 184320;      -- 720 px
                        else z_loc <= signed(resize(c_hp, 19)) - 245760; end if;            -- 960 px
                        z_k <= (others => '0');
                        z_nrun <= '1';
                    when 600 =>
                        c_x0 <= unsigned(z_loc(15 downto 0)); c_i0 <= z_k;
                        if z_loc + c_xmp >= 0 then c_wrap0 <= '1'; else c_wrap0 <= '0'; end if;
                    when 601 =>
                        z_loc <= signed(resize(c_hp, 19)) - 138240;                         -- 540 lines
                        z_k <= (others => '0');
                        z_nrun <= '1';
                    when 640 => z_first <= '1';
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
    -- Blend table fill (zone, from zc 40, about 650 clocks; adders only).  With
    -- weights w = 0 1 2 3 5 6 7 8 eighths for each 3-bit code:
    --   U = ground + (accent - ground) w(mark),  D = A + (B - A) w(mix),
    --   T(mark, mix, cov) = U + (D - U) w(cov)
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            if s_zone = '1' and s_zc = 40 then
                t_st <= "01"; t_m <= "000"; t_x <= "000";
                for i in 0 to 2 loop
                    tf_u8(i) <= signed(resize(c_pg(i) & "000", 14));
                    tf_d8(i) <= signed(resize(c_pa(i) & "000", 14));
                end loop;
            else
                case t_st is
                    when "01" =>
                        for i in 0 to 2 loop
                            t_stp(i) <= tf_d8(i) - tf_u8(i);
                            t_acc(i) <= shift_left(resize(tf_u8(i), 17), 3);
                        end loop;
                        t_c <= "000"; t_st <= "10";
                    when "10" =>
                        tt_we <= '1'; tt_wa <= t_m & t_x & t_c;
                        tt_wd <= "000000" & std_logic_vector(t_acc(0)(15 downto 6)) & std_logic_vector(t_acc(1)(13 downto 6)) & std_logic_vector(t_acc(2)(13 downto 6));
                        for i in 0 to 2 loop
                            if t_c = 3 then t_acc(i) <= t_acc(i) + shift_left(resize(t_stp(i), 17), 1);
                            else t_acc(i) <= t_acc(i) + resize(t_stp(i), 17); end if;
                        end loop;
                        t_c <= t_c + 1;
                        if t_c = 7 then t_st <= "11"; end if;
                    when "11" =>
                        t_st <= "01";
                        if t_x = 7 then
                            t_x <= "000"; t_m <= t_m + 1;
                            for i in 0 to 2 loop
                                tf_d8(i) <= signed(resize(c_pa(i) & "000", 14));
                                if t_m = 3 then tf_u8(i) <= tf_u8(i) + shift_left(resize(c_dcg(i), 14), 1);
                                else tf_u8(i) <= tf_u8(i) + resize(c_dcg(i), 14); end if;
                            end loop;
                            if t_m = 7 then t_st <= "00"; end if;
                        else
                            t_x <= t_x + 1;
                            for i in 0 to 2 loop
                                if t_x = 3 then tf_d8(i) <= tf_d8(i) + shift_left(resize(c_dba(i), 14), 1);
                                else tf_d8(i) <= tf_d8(i) + resize(c_dba(i), 14); end if;
                            end loop;
                        end if;
                    when others => null;
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
    -- a 1/16 filter and held per cell: on the last line of each cell row (and
    -- on the field's first line) the smear is sampled at the centre of every
    -- cell of the row below into the bank of that row's parity, and each pixel
    -- reads its own cell's sample, so every dot is rigid.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_x  : unsigned(13 downto 0);
        variable v_d1 : signed(14 downto 0);
        variable v_cp : unsigned(17 downto 0);
        variable v_kn : unsigned(9 downto 0);
        variable v_av, v_f, v_l2, v_tn, v_fe, v_ev, v_ts : std_logic;
        variable v_ix : signed(7 downto 0);
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

            -- hold writes, from the stage-2 pixel: the target row's cell centres are this lattice's cell
            -- centres (target row not offset) or its cell boundaries (target row offset half a pitch)
            v_av := s_gav_sr(0);
            v_f  := v_av and not s_gav_sr(1);         -- the line's first pixel
            v_l2 := v_av and not g_avid;              -- the line's last pixel
            v_tn := q2_t1(17);                        -- left of the cell centre
            v_ts := s_hex and l_tp;
            v_fe := h_fl and not v_f and not q2_nw;
            v_ix := q2_i;
            if v_ts = '0' then
                v_ev := (not v_tn or v_l2) and not v_fe;      -- at the cell centre, or at the line's end if it is past it
            elsif (v_f = '1' and v_tn = '1') or q2_nw = '1' then
                v_ev := '1'; v_ix := q2_i - 1;
            else
                v_ev := v_l2;
            end if;
            if v_av = '1' then h_fl <= v_fe or v_ev; end if;
            h_we <= v_ev and v_av and l_srow;
            h_wa <= l_tp & unsigned(v_ix(6 downto 0));
            -- K6 camera gain on the smeared sample (4 stages: the written sample is the smear 3 px before the write
            -- point): x = sample x c_cg / 256, then a soft knee: x up to 223 unchanged (above the v0.1.0 white,
            -- 219, so 25 % is the v0.1.0 picture), 1/4 slope above, 255 from x = 348
            for i in 0 to 3 loop
                cm1_p(i) <= f_gu(c_cg, s_y1(6 + 2 * i), 12) + shift_left(f_gu(c_cg, s_y1(7 + 2 * i), 12), 1);
            end loop;
            cm2_a <= resize(cm1_p(0), 14) + shift_left(resize(cm1_p(1), 14), 2);
            cm2_b <= resize(cm1_p(2), 14) + shift_left(resize(cm1_p(3), 14), 2);
            v_cp := resize(cm2_a, 18) + shift_left(resize(cm2_b, 18), 4);
            cm3_x <= v_cp(17 downto 8);
            v_kn := cm3_x - 224;
            if cm3_x < 224 then h_wd <= std_logic_vector(cm3_x(7 downto 0));
            elsif cm3_x >= 348 then h_wd <= (others => '1');
            else h_wd <= std_logic_vector(to_unsigned(224, 8) + resize(v_kn(6 downto 2), 8)); end if;
            -- the read of the address being written (only possible on a field's first line) is discarded
            if h_we = '1' and h_wa = h_ra then h_col <= '1'; else h_col <= '0'; end if;
            if h_col = '0' then q6_lum <= unsigned(h_q(7 downto 0)); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Wave engine.  The lattice: y steps per line (ls_* hold the NEXT
    -- line's start values, wrap candidates pre-registered), x steps per pixel
    -- (the wrap flag of the next step registered a pixel ahead).
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_w  : signed(18 downto 0);
        variable v_ab : signed(9 downto 0);
        variable v_aj : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            ls_n1 <= resize(ls_loc + s_ys, 17);
            ls_n2 <= signed(resize(ls_loc, 19)) + c_ymp;
            ls_t  <= signed(resize(ls_loc, 18)) - signed(resize(c_hp, 18));
            if s_zone = '1' and s_zc = 640 then
                ls_loc <= unsigned(z_loc(16 downto 0)); ls_k <= z_k;
            elsif g_start = '1' then
                if ls_n2(18) = '0' then
                    ls_loc <= unsigned(ls_n2(16 downto 0)); ls_k <= ls_k + 1;
                else
                    ls_loc <= ls_n1;
                end if;
            end if;
            -- this line's row values
            if g_start = '1' then
                l_j  <= ls_k;
                l_ly <= ls_t(16 downto 7);
                v_ab := abs(ls_t(16 downto 7));
                l_aly <= unsigned(v_ab);
                v_aj := abs(ls_k);
                l_ay <= unsigned(v_aj(6 downto 0)) & '0';
                l_hx <= s_hex and ls_k(0);
                l_jp <= ls_k(0);
                l_srow <= z_first or not ls_n2(18);       -- the field's first line, or the last line of a cell row
                l_tp <= ls_k(0) xor not ls_n2(18);        -- the parity of the row the samples are for
            end if;

            -- pixel accumulators (stage 1)
            if g_start = '1' then
                a_loc <= '0' & c_x0; a_i <= c_i0;
                a_wrap <= c_wrap0; a_nw <= '0';
            elsif g_avid = '1' then
                if a_wrap = '1' then
                    v_w := signed(resize(a_loc, 19)) + c_xmp;
                    a_loc <= unsigned(v_w(16 downto 0));
                    a_i   <= a_i + 1;
                    a_nw  <= '1'; a_wrap <= '0';
                else
                    a_loc <= resize(a_loc + c_xst, 17);
                    a_nw  <= '0';
                    if signed(resize(a_loc, 19)) + c_x2mp >= 0 then a_wrap <= '1'; else a_wrap <= '0'; end if;
                end if;
            else
                a_nw <= '0';
            end if;
        end if;
    end process p_acc;

    p_wave : process(clk)
        variable v_t  : signed(17 downto 0);
        variable v_ie : signed(7 downto 0);
        variable v_ab : signed(8 downto 0);
        variable v_n  : unsigned(9 downto 0);
        variable v_un : unsigned(8 downto 0);
        variable v_th : unsigned(15 downto 0);
        variable v_ps, v_t11 : unsigned(10 downto 0);
        variable v_lu : unsigned(5 downto 0);
        variable v_pl : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: x relative to the cell centre (Q8)
            q2_t1 <= signed(resize(a_loc, 18)) - signed(resize(c_hp, 18));
            q2_i  <= a_i;
            q2_nw <= a_nw;
            -- 3: this row's lattice: as it is, or offset half a pitch (Hex, odd rows); cell-local x in half px
            v_t := q2_t1; v_ie := q2_i;
            if l_hx = '1' then
                if q2_t1(17) = '0' then
                    v_t := q2_t1 - signed(resize(c_hp, 18));
                else
                    v_t := q2_t1 + signed(resize(c_hp, 18)); v_ie := q2_i - 1;
                end if;
            end if;
            q3_lx <= v_t(16 downto 7);
            q3_ie <= v_ie;
            q3_x2 <= v_ie & l_hx;                     -- the cell's x position in half cells
            -- 4: |x|, the linear index; the hold's read address
            v_ab := abs(q3_x2);
            q4_ax  <= unsigned(v_ab(7 downto 0));
            q4_lin <= q3_x2 + resize(l_j, 9);
            h_ra   <= l_jp & unsigned(q3_ie(6 downto 0));
            -- 5: compare for the octagonal metric
            if q4_ax >= l_ay then q5_ge <= '1'; else q5_ge <= '0'; end if;
            q5_ax <= q4_ax; q5_lin <= q4_lin;
            -- 6: the index n in quarter cells: Linear 2 (x + j); Radial 2 max + min
            if s_rad = '0' then
                q6_n <= q5_lin(7 downto 0) & '0';
            else
                if q5_ge = '1' then v_n := resize(q5_ax & '0', 10) + resize(l_ay, 10);
                else v_n := resize(l_ay & '0', 10) + resize(q5_ax, 10); end if;
                q6_n <= signed(v_n(8 downto 0));
            end if;
            -- 7 .. 10: theta = phase - Spread * n (16 bits per cycle, modulo), a sum of gated operands
            v_un := unsigned(q6_n);
            for k in 0 to 3 loop
                q7_p(k) <= f_gu(c_s, v_un(2 * k), 16) + shift_left(f_gu(c_s, v_un(2 * k + 1), 16), 1);
            end loop;
            q7_p4 <= f_gu(c_s, v_un(8), 16);
            q8_a <= q7_p(0) + shift_left(q7_p(1), 2);
            q8_b <= q7_p(2) + shift_left(q7_p(3), 2);
            q8_c <= c_ph(25 downto 10) + shift_left(q7_p4, 8);
            q9_s <= q8_a + shift_left(q8_b, 4);
            q9_c <= q8_c;
            q10_th <= q9_c - q9_s;
            s_n5(7) <= v_un(4 downto 0);
            for i in 8 to 10 loop s_n5(i) <= s_n5(i - 1); end loop;
            -- 7 .. 10: the dot's radius^2 = R0^2 + luma * (Rlim^2 - R0^2) / 64
            v_lu := q6_lum(7 downto 2);
            q7_l0 <= f_gu(c_dr2, v_lu(0), 16) + shift_left(f_gu(c_dr2, v_lu(1), 16), 1);
            q7_l1 <= f_gu(c_dr2, v_lu(2), 16) + shift_left(f_gu(c_dr2, v_lu(3), 16), 1);
            q7_l2 <= f_gu(c_dr2, v_lu(4), 16) + shift_left(f_gu(c_dr2, v_lu(5), 16), 1);
            q8_la <= resize(q7_l0, 18) + shift_left(resize(q7_l1, 18), 2);
            q8_lb <= q7_l2;
            v_pl := resize(q8_la, 20) + shift_left(resize(q8_lb, 20), 4);
            q9_pr <= v_pl(19 downto 6);
            q10_re <= resize(c_r02, 15) + resize(q9_pr, 15);
            -- 11: Snap = dwell at theta 0 and 1/2, the last quarter of each half cycle flips; the two circles'
            -- angles (11 bits per turn): theta + psi and psi - theta
            v_th := q10_th;
            if s_snap = '1' then
                if q10_th(14 downto 13) = "11" then v_th := q10_th(15) & q10_th(12 downto 0) & "00";
                else v_th := q10_th(15) & "000000000000000"; end if;
            end if;
            v_t11 := v_th(15 downto 5);
            if s_fan = '1' then v_ps := s_n5(10) & "000000"; else v_ps := (others => '0'); end if;
            q11_al <= v_t11 + v_ps;
            q11_be <= v_ps - v_t11;
        end if;
    end process p_wave;

    ---------------------------------------------------------------------------
    -- [E] The look: displacement, r^2, coverage, rest mark, blend table
    ---------------------------------------------------------------------------
    p_look : process(clk)
        variable v_c, v_s : unsigned(7 downto 0);
        variable v_amp : unsigned(7 downto 0);
        variable v_p  : unsigned(16 downto 0);
        variable v_x, v_y : signed(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_ae : unsigned(10 downto 0);
        variable v_al : signed(9 downto 0);
        variable v_mx, v_mn : unsigned(9 downto 0);
        variable v_vm : signed(12 downto 0);
        variable v_k  : unsigned(2 downto 0);
        variable v_d  : signed(19 downto 0);
        variable v_cv : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- 12: trig addresses; 13: ROM; 14: fabric copies, by quadrant: magnitudes and signs
            r_ta <= q11_al(8 downto 0); q12_qa <= q11_al(10 downto 9); q13_qa <= q12_qa;
            r_tb <= q11_be(8 downto 0); q12_qb <= q11_be(10 downto 9); q13_qb <= q12_qb;
            v_c := r_ta_q(15 downto 8); v_s := r_ta_q(7 downto 0);
            if q13_qa(0) = '1' then q14_t(0) <= v_s; q14_t(1) <= v_c; else q14_t(0) <= v_c; q14_t(1) <= v_s; end if;
            q14_n(0) <= q13_qa(1) xor q13_qa(0); q14_n(1) <= q13_qa(1);
            v_c := r_tb_q(15 downto 8); v_s := r_tb_q(7 downto 0);
            if q13_qb(0) = '1' then q14_t(2) <= v_s; q14_t(3) <= v_c; else q14_t(2) <= v_c; q14_t(3) <= v_s; end if;
            q14_n(2) <= q13_qb(1) xor q13_qb(0) xor c_bneg; q14_n(3) <= q13_qb(1) xor c_bneg;
            -- 15 .. 17: the four products radius * trig as gated sums (half px, rounded)
            for i in 0 to 3 loop
                if i < 2 then v_amp := c_a; else v_amp := c_bm; end if;
                for k in 0 to 3 loop
                    q15_p(4 * i + k) <= f_gu(q14_t(i), v_amp(2 * k), 10) + shift_left(f_gu(q14_t(i), v_amp(2 * k + 1), 10), 1);
                end loop;
                q16_a(i) <= resize(q15_p(4 * i), 12) + shift_left(resize(q15_p(4 * i + 1), 12), 2);
                q16_b(i) <= resize(q15_p(4 * i + 2), 12) + shift_left(resize(q15_p(4 * i + 3), 12), 2);
                v_p := resize(q16_a(i), 17) + shift_left(resize(q16_b(i), 17), 4) + 128;
                q17_x(i) <= v_p(15 downto 8);
            end loop;
            q15_n <= q14_n; q16_n <= q15_n; q17_n <= q16_n;
            -- 18: the displacement; the cell-local x from its delay ring
            v_x := signed(resize(q17_x(0), 10)); if q17_n(0) = '1' then v_x := -v_x; end if;
            v_y := signed(resize(q17_x(2), 10)); if q17_n(2) = '1' then v_y := -v_y; end if;
            q18_dx <= v_x + v_y;
            v_x := signed(resize(q17_x(1), 10)); if q17_n(1) = '1' then v_x := -v_x; end if;
            v_y := signed(resize(q17_x(3), 10)); if q17_n(3) = '1' then v_y := -v_y; end if;
            q18_dy <= v_x + v_y;
            q18_lx <= signed(lr_q(9 downto 0));
            -- 19: pixel relative to the dot centre
            q19_ex <= resize(q18_lx, 11) - resize(q18_dx, 11);
            q19_ey <= resize(l_ly, 11) - resize(q18_dy, 11);
            -- 20: |e| capped at 511 -> the table of squares; 21: ROM; 22: fabric copies
            v_e := abs(q19_ex); v_ae := unsigned(v_e);
            if v_ae(10 downto 9) /= "00" then r_qx <= (others => '1'); else r_qx <= v_ae(8 downto 0); end if;
            v_e := abs(q19_ey); v_ae := unsigned(v_e);
            if v_ae(10 downto 9) /= "00" then r_qy <= (others => '1'); else r_qy <= v_ae(8 downto 0); end if;
            q22_sx <= r_qx_q; q22_sy <= r_qy_q;
            -- 23: r^2; the dot's R^2 and colour mix from their delay ring
            q23_r2 <= resize(q22_sx, 17) + resize(q22_sy, 17);
            q23_re <= unsigned(rr_q(17 downto 3)); q23_mix <= unsigned(rr_q(2 downto 0));
            -- 24: R^2 - r^2; the shift = the leading bit pair of R^2 (2 R ~ 2^(k+2): a ramp of 1 .. 2 px)
            q24_d <= signed(resize(q23_re, 18)) - signed(resize(q23_r2, 18));
            if    q23_re(14 downto 12) /= 0 then v_k := to_unsigned(6, 3);
            elsif q23_re(11 downto 10) /= 0 then v_k := to_unsigned(5, 3);
            elsif q23_re(9 downto 8) /= 0   then v_k := to_unsigned(4, 3);
            elsif q23_re(7 downto 6) /= 0   then v_k := to_unsigned(3, 3);
            elsif q23_re(5 downto 4) /= 0   then v_k := to_unsigned(2, 3);
            elsif q23_re(3 downto 2) /= 0   then v_k := to_unsigned(1, 3);
            else                                 v_k := to_unsigned(0, 3); end if;
            if s_sd = '1' then q24_k <= v_k + 1; else q24_k <= v_k; end if;
            q24_mix <= q23_mix; q25_mix <= q24_mix;
            -- 25: dot coverage, 8 steps
            v_d := shift_left(resize(q24_d, 20), 2);
            v_cv := shift_right(v_d, to_integer(q24_k)) + 8;
            if v_cv < 0 then q25_cov <= "000";
            elsif v_cv > 15 then q25_cov <= "111";
            else q25_cov <= unsigned(v_cv(3 downto 1)); end if;

            -- rest mark: an octagon (max + min / 2) about the cell centre.  19: |x|; 20: compare; 21: metric;
            -- 22: coverage, 8 steps over 2 px (4 px at SD)
            v_al := abs(q18_lx); q19_alx <= unsigned(v_al);
            if q19_alx >= l_aly then q20_ge <= '1'; else q20_ge <= '0'; end if;
            q20_alx <= q19_alx;
            if q20_ge = '1' then v_mx := q20_alx; v_mn := l_aly; else v_mx := l_aly; v_mn := q20_alx; end if;
            q21_m <= resize(v_mx, 11) + resize(shift_right(v_mn, 1), 11);
            v_vm := signed(resize(c_rm, 13)) - signed(resize(q21_m, 13));
            if s_sd = '1' then v_vm := shift_left(v_vm, 1) + 8; else v_vm := shift_left(v_vm, 2) + 8; end if;
            if v_vm < 0 then q22_mk <= "000";
            elsif v_vm > 15 then q22_mk <= "111";
            else q22_mk <= unsigned(v_vm(3 downto 1)); end if;
            q23_mk <= q22_mk; q24_mk <= q23_mk; q25_mk <= q24_mk;

            -- 26: the blend table's address; 27: RAM; 28: fabric copy; 29: the ground colour outside the avid
            tt_ra <= q25_mk & q25_mix & q25_cov;
            q28_t <= tt_q(25 downto 0);
            if s_gav_sr(26) = '0' then
                q29_y <= c_gy; q29_c <= c_gc;
            else
                q29_y <= unsigned(q28_t(25 downto 16));
                q29_c(1) <= unsigned(q28_t(15 downto 8)); q29_c(2) <= unsigned(q28_t(7 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(26);
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
                s_bu <= c_gc(1) & "000"; s_bv <= c_gc(2) & "000";
            else
                s_bu <= s_bu + q29_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q29_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q29_y;
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

end architecture pendula;
