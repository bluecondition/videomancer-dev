-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Louver v0.1.0 -- Kinetic #6 (pilot).
--
-- Rows of flat two-sided slats, each turning about its long axis, lit from one side (kinetic louvre walls,
-- tri-vision billboards, venetian blinds turning in a wave, twisted-ribbon sculpture).
--
--   element  a slat of pitch P; state = its angle alpha about its long axis (14 bits per turn)
--   theta    = Spread * index + Twist * u / P - phase          (u = position along the slat from the centre)
--              index: Linear = slat number; Radial = octagonal distance (|slat number|, |u| / P)
--   alpha    = Depth * wave(theta) + source                    (wave: sine / snap = dwell and quick flips)
--   slab     a slat is a slab of width Blade * P and thickness P/16 + 1.5: projected half-height
--              H = (Blade P / 2) |cos alpha| + (thickness / 2) |sin alpha|,  inside when |t| < H
--              the thin face (accent colour) is the outer thickness * |sin alpha| on the side sign(sin * cos)
--   face     front (cos alpha > 0): colour A, or the picture (Face = Video); back: colour B
--   shade    0.42 + 0.58 |cos(alpha - light)|, glint toward the accent where the face catches the light
--   edges    coverage from H - |t| over a ramp of (pixel size + |dH| per pixel along the slat), 8 steps;
--            a face narrower than 2 .. 4 px fades into its neighbour (no 1-px hard lines)
--   out      = [ground + (accent - ground) * slab coverage / 8 - accent * face weight / 16]   (a per-field table)
--              + face colour * shade * face weight / 16;  luma 10 bits, chroma 8 bits until the output filter
--
--   K1  Scale    slat pitch 16 .. 270 virtual lines (exponential, glided)
--   K2  Blade    blade width 20 % .. 100 % of the pitch
--   K3  Depth    swing 0 .. 1.25 turns (square law)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Colour   the 16 shared palettes (ground = gap, A = front, B = back, accent = glint and thin face)
--                [K6 camera pass: was K6; Spread fixed at 354 = phase step 0.06 cycle per slat]
--   K6  Camera   camera (and Osc) strength: source term = src * 20 g turns/2^14 (Osc x2), g = K6 / 256 (glided):
--                0 = none (= a black input), 1 = the v0.1.0 gain at 25 %, 2 at the 50 % default, 4 at 100 %
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Axis     Across (horizontal slats) / Down (vertical slats)
--   S10 Face     Palette / Video
--   S11 Source   Video / Osc
--   P12 TWIST    0 .. 1/2 wave cycle per pitch length along the slat (square law, glided)
--
-- The file is in lifted sections (the Kinetic chassis):
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-element holds, video delay ring
--   [D] wave engine: line / pixel accumulators -> theta -> wave ROM -> state
--   [E] the look: slat silhouette, coverage, palette + AA blend
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 36) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 354 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.louver_pkg.all;
use work.all;

architecture louver of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(354, 10));   -- K6 camera pass: fixed knob
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
    signal r_w_a  : unsigned(9 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a : unsigned(8 downto 0) := (others => '0');
    signal r_cs_q : unsigned(15 downto 0) := (others => '0');
    signal r_sh_a : unsigned(8 downto 0) := (others => '0');
    signal r_sh_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- [C] source hold, Across: one smeared luma line per slat (every second pixel; written on the slat's
    -- sample row only)
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    attribute no_rw_check of lb : signal is true;
    signal s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    -- source hold, Down: a one-pole filter along y per slat column (Q6)
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal sl : t_r16 := (others => (others => '0'));
    attribute no_rw_check of sl : signal is true;
    signal sl_ra, sl_wa : unsigned(7 downto 0) := (others => '0');
    signal sl_q, sl_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal sl_we : std_logic := '0';
    -- [E] previous line's half-height per slat column (Down: the edge slope for the coverage ramp)
    signal hp : t_r16 := (others => (others => '0'));
    attribute no_rw_check of hp : signal is true;
    signal hp_ra, hp_wa : unsigned(7 downto 0) := (others => '0');
    signal hp_q, hp_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal hp_we : std_logic := '0';
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    type t_r32 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal vr : t_r32 := (others => (others => '0'));          -- video: input register -> the stage-26 mux
    attribute no_rw_check of vr : signal is true;
    signal vr_d, vr_q : std_logic_vector(31 downto 0) := (others => '0');
    signal tr : t_r16 := (others => (others => '0'));          -- slat offset t: stage 2 -> 18
    attribute no_rw_check of tr : signal is true;
    signal tr_q : std_logic_vector(15 downto 0) := (others => '0');
    signal ar : t_r16 := (others => (others => '0'));          -- alpha (top 10 bits): stage 12 -> shade address at 22
    attribute no_rw_check of ar : signal is true;
    signal ar_q : std_logic_vector(15 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] blend table, refilled every field: T(co, ws) = ground + (accent - ground) co / 8 - (accent - black) ws / 16
    -- {Y 11 bits signed, U 10, V 10 (chroma in 8-bit units)}; address = co & ws
    type t_tt is array (0 to 511) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(8 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_run, t_row : std_logic := '0';
    signal t_co  : unsigned(3 downto 0) := (others => '0');
    signal t_ws  : unsigned(4 downto 0) := (others => '0');
    signal t_ry, t_ey : signed(14 downto 0) := (others => '0');
    type t_s14 is array (1 to 2) of signed(13 downto 0);
    signal t_rc, t_ec : t_s14 := (others => (others => '0'));
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
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(625 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(323 * 8, 13);
    signal s_gk : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);   -- K6 camera pass: the Camera glide
    signal c_cam : unsigned(6 downto 0) := to_unsigned(40, 7);         -- camera coefficient 20 g = 5 K6 / 64
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_down, s_face, s_src : std_logic := '0';
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
    signal z_nz  : std_logic := '0';
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : unsigned(1 downto 0) := (others => '0');
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_bl  : unsigned(9 downto 0) := to_unsigned(870, 10);
    signal z_loc : signed(18 downto 0) := (others => '0');
    signal z_k   : signed(7 downto 0) := (others => '0');
    signal z_nrun, z_first, z_lrun : std_logic := '0';
    signal z_n   : unsigned(10 downto 0) := (others => '0');

    -- per-field constants
    signal c_pw   : unsigned(19 downto 0) := to_unsigned(23040 * 8, 20);   -- pitch, Q11 (c_p = Q8 virtual lines)
    signal c_p    : unsigned(16 downto 0);
    signal c_ip   : unsigned(20 downto 0) := to_unsigned(186413, 21);  -- 1 / pitch, Q24
    signal c_hmax : unsigned(11 downto 0) := to_unsigned(600, 12);     -- blade half-width, Q4
    signal c_hlim : unsigned(11 downto 0) := to_unsigned(700, 12);     -- pitch / 2 - 1, Q4
    signal c_phq  : unsigned(11 downto 0);                              -- pitch / 2, Q4
    signal c_thk  : unsigned(8 downto 0) := to_unsigned(100, 9);       -- slab thickness, Q4
    signal c_d    : unsigned(7 downto 0) := (others => '0');           -- depth, Q7 turns
    signal c_s    : unsigned(22 downto 0) := (others => '0');          -- spread per slat, Q24 cycles
    signal z_x    : unsigned(19 downto 0) := (others => '0');          -- spread, twist per virtual px along the slat (scratch)
    signal c_dsu, c_dtw : unsigned(22 downto 0) := (others => '0');   -- their step per pixel (Across) / per line (Down)
    signal c_xst  : unsigned(9 downto 0) := (others => '0');           -- across step per pixel, Q8
    signal c_yst  : unsigned(11 downto 0) := to_unsigned(256, 12);     -- across step per line, Q8
    signal c_xmp, c_x2mp, c_ymp : signed(18 downto 0) := (others => '0');
    signal c_wrap0 : std_logic := '0';
    signal c_base : unsigned(6 downto 0) := to_unsigned(16, 7);        -- across-axis pixel size, Q4
    signal c_ph   : unsigned(25 downto 0) := (others => '0');          -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    type t_s8  is array (1 to 2) of signed(7 downto 0);
    type t_s9  is array (1 to 2) of signed(8 downto 0);
    -- palette: luma 10 bits (ground raw; A, B, accent less black), chroma 8 bits (ground raw; A, B, accent centred)
    signal c_gy   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_ay, c_by, c_cy : unsigned(9 downto 0) := (others => '0');
    signal c_gc   : t_u8 := (others => to_unsigned(128, 8));
    signal c_ac, c_bc, c_cc : t_s8 := (others => (others => '0'));
    signal c_ky   : signed(10 downto 0) := (others => '0');            -- accent - ground
    signal c_kc   : t_s9 := (others => (others => '0'));

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
    signal s_sav, s_savd, s_wen, s_srow : std_logic := '0';
    signal s_ext : unsigned(5 downto 0) := (others => '0');
    signal s_y1 : unsigned(13 downto 0) := (others => '0');
    signal s_wi  : unsigned(10 downto 0) := (others => '0');
    signal v_iy, v_iu, v_iv : unsigned(9 downto 0) := (others => '0');

    -- [D] wave engine: line registers (the NEXT line's start values) and pixel accumulators (stage 1)
    signal ls_su, ls_tw, ls_sk : signed(27 downto 0) := (others => '0');
    signal ls_loc : unsigned(16 downto 0) := (others => '0');
    signal ls_k   : signed(7 downto 0) := (others => '0');
    signal ls_n1  : unsigned(16 downto 0) := (others => '0');
    signal ls_n2  : signed(18 downto 0) := (others => '0');
    signal a_su, a_tw, a_sk : signed(27 downto 0) := (others => '0');
    signal a_loc  : unsigned(16 downto 0) := (others => '0');
    signal a_wrap, a_nw : std_logic := '0';
    signal l_k0   : signed(7 downto 0) := (others => '0');
    signal l_first : std_logic := '0';
    signal s_nw   : std_logic_vector(2 to 18) := (others => '0');
    signal q2_ask, q2_asu : unsigned(18 downto 0) := (others => '0');
    signal q2_atw, q2_lin : unsigned(13 downto 0) := (others => '0');
    signal q2_t   : signed(12 downto 0) := (others => '0');
    signal q3_ge  : std_logic := '0';
    signal q3_ask, q3_asu : unsigned(14 downto 0) := (others => '0');
    signal q3_lin, q3_y, q4_xa, q4_xb, q5_th : unsigned(13 downto 0) := (others => '0');
    signal s_neg  : std_logic_vector(6 to 11) := (others => '0');
    signal q8_w   : unsigned(9 downto 0) := (others => '0');
    signal q9_p0, q9_p1, q9_p2, q9_p3 : unsigned(11 downto 0) := (others => '0');
    signal q10_qa, q10_qb : unsigned(13 downto 0) := (others => '0');
    signal q10_src : unsigned(7 downto 0) := (others => '0');
    signal q10_e   : unsigned(13 downto 0) := (others => '0');
    signal q11_aw  : unsigned(14 downto 0) := (others => '0');
    -- K6 camera pass: source term = src * c_cam (Osc x2) as two gated partial sums (11), their sum (12),
    -- alpha = +-Depth * wave + source at 13 (folded into the |cos| ROM address stage)
    signal q11_pa  : unsigned(11 downto 0) := (others => '0');
    signal q11_pb  : unsigned(10 downto 0) := (others => '0');
    signal q12_st  : unsigned(13 downto 0) := (others => '0');
    signal q12_aw  : signed(15 downto 0) := (others => '0');
    signal q13_al  : unsigned(13 downto 0) := (others => '0');
    signal k8, k9, k10, k17, k18, k19 : unsigned(7 downto 0) := (others => '0');

    -- [E] the look
    signal q13_sg, q14_sg, q15_sig, q16_sig, q17_sig, q18_sig : std_logic := '0';
    signal q15_cos : unsigned(8 downto 0) := (others => '0');
    signal q15_sin : unsigned(4 downto 0) := (others => '0');
    signal q16_h01, q16_h23, q16_h45, q16_h67 : unsigned(13 downto 0) := (others => '0');
    signal q16_h8  : unsigned(11 downto 0) := (others => '0');
    signal q16_t01, q16_t23 : unsigned(10 downto 0) := (others => '0');
    signal q16_t4  : unsigned(8 downto 0) := (others => '0');
    signal q17_q   : unsigned(15 downto 0) := (others => '0');
    signal q17_r   : unsigned(16 downto 0) := (others => '0');
    signal q17_ts, q18_ts : unsigned(7 downto 0) := (others => '0');
    signal q18_hc  : unsigned(11 downto 0) := (others => '0');
    signal q18_t   : signed(12 downto 0) := (others => '0');
    signal q19_h, q20_h, q19_hpv, q19_at : unsigned(11 downto 0) := (others => '0');
    signal q19_stp : signed(14 downto 0) := (others => '0');
    signal q19_cb, q20_cb, q21_cb, q22_cb, q23_cb : unsigned(3 downto 0) := (others => '0');
    signal q19_ct, q20_ct, q21_ct, q22_ct, q23_ct : unsigned(3 downto 0) := (others => '0');
    signal q20_fo, q21_fo, q22_fo, q20_fe, q21_fe, q22_fe : signed(8 downto 0) := (others => '0');   -- saturated
    signal q20_dh : signed(12 downto 0) := (others => '0');
    signal q21_w  : unsigned(12 downto 0) := (others => '0');
    signal q22_lw : unsigned(2 downto 0) := (others => '0');
    signal q23_co, q23_nce, q24_co, q24_bc, q25_co : unsigned(3 downto 0) := (others => '0');
    signal q25_ws : unsigned(4 downto 0) := (others => '0');
    signal q22_aq, q23_aq, q24_aq, q25_aq : std_logic_vector(1 downto 0) := "00";
    signal q24_shade, q25_shade : unsigned(6 downto 0) := (others => '0');
    signal q24_g4 : unsigned(3 downto 0) := (others => '0');
    signal q26_s2 : unsigned(6 downto 0) := (others => '0');
    signal q26_y  : unsigned(9 downto 0) := (others => '0');           -- the face's colour: luma less black, chroma centred
    signal q26_c  : t_s8 := (others => (others => '0'));
    signal q27_y0, q27_y1, q27_y2 : unsigned(11 downto 0) := (others => '0');
    signal q27_y3 : unsigned(9 downto 0) := (others => '0');
    type t_s10 is array (1 to 2) of signed(9 downto 0);
    signal q27_c0, q27_c1 : t_s10 := (others => (others => '0'));
    signal q27_c2 : t_s8 := (others => (others => '0'));
    signal q28_t  : std_logic_vector(30 downto 0) := (others => '0');
    signal q28_py : unsigned(9 downto 0) := (others => '0');
    signal q28_pc : t_s8 := (others => (others => '0'));
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

    c_p <= c_pw(19 downto 3);
    c_phq <= c_pw(19 downto 8);

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, holds, delay rings (one write statement each; none ever
    -- reads the address it is writing)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_LV_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_LV_SIN(to_integer(r_w_a));
            r_cs_q <= C_LV_CS(to_integer(r_cs_a));
            r_sh_q <= C_LV_SHD(to_integer(r_sh_a));
            r_pl_q <= C_LV_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra(10 downto 1)));
            if s_lb_we = '1' then lb(to_integer(s_wi(10 downto 1))) <= std_logic_vector(s_y1(13 downto 6)); end if;
        end if;
    end process p_lb;

    p_sl : process(clk)
    begin
        if rising_edge(clk) then
            sl_q <= sl(to_integer(sl_ra));
            if sl_we = '1' then sl(to_integer(sl_wa)) <= sl_wd; end if;
        end if;
    end process p_sl;

    p_hp : process(clk)
    begin
        if rising_edge(clk) then
            hp_q <= hp(to_integer(hp_ra));
            if hp_we = '1' then hp(to_integer(hp_wa)) <= hp_wd; end if;
        end if;
    end process p_hp;

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
            vr(to_integer(rc + 26)) <= vr_d;
            vr_q <= vr(to_integer(rc));
            tr(to_integer(rc + 14)) <= "000" & std_logic_vector(q2_t);
            tr_q <= tr(to_integer(rc));
            ar(to_integer(rc + 7)) <= "000000" & std_logic_vector(q13_al(13 downto 4));   -- K6 camera pass: stage 13, d 7
            ar_q <= ar(to_integer(rc));
            cr(to_integer(rc + 6)) <= std_logic_vector(q29_c(1)) & std_logic_vector(q29_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;
    vr_d <= "000000" & std_logic_vector(v_iy) & std_logic_vector(v_iu(9 downto 2)) & std_logic_vector(v_iv(9 downto 2));

    ---------------------------------------------------------------------------
    -- [A] Controls, vblank zone, glides, per-field sequencer (one small
    -- operation per zone clock; quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_t  : unsigned(28 downto 0);
        variable v_x8 : unsigned(9 downto 0);
        variable v_l  : signed(18 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
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
                s_down <= registers_in(6)(2);
                s_face <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
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

            -- shared glide for Scale and Twist (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(1) = '1' then s_gm <= s_gk; s_gr <= unsigned(registers_in(5));   -- K6 Camera
                        elsif z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>                           -- deadband +-2 codes (the camera, channel 2: +-4)
                        if C_SIM_SNAP then
                            null;
                        elsif (s_gd > 16 or s_gd < -16) and (z_gc(1) = '0' or s_gd > 32 or s_gd < -32) then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(1) = '1' then s_gk <= unsigned(signed(s_gm) + s_gd(12 downto 0));
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
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" and s_zc(8 downto 5) /= 10
               and s_zc(8 downto 5) /= 11 and s_zc(8 downto 5) /= 13 and s_zc(8 downto 5) /= 15 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(8364, 24);      -- pitch exponent
                    when 1  => z_fa(9 downto 0)  <= unsigned(registers_in(1)); z_fb <= to_unsigned(819, 24);  -- blade
                    when 2  => z_fa(16 downto 0) <= c_p;                  z_fb <= resize(z_bl, 24);           -- blade half-width
                    when 3  =>                                                                                -- depth^2
                        z_fa(12 downto 3) <= unsigned(registers_in(2)); z_fb(12 downto 3) <= unsigned(registers_in(2));
                    when 4  => z_fa(12 downto 0) <= s_gp;                 z_fb <= resize(s_gp, 24);           -- twist^2 (held in c_s)
                    when 7  =>                                                                                -- spread^2
                        z_fa(12 downto 3) <= unsigned(C_KFIX_5); z_fb(12 downto 3) <= unsigned(C_KFIX_5);
                    when 5 | 8 => z_fa(20 downto 0) <= c_ip;              z_fb <= resize(c_s, 24);            -- twist, spread / pitch
                    when 6 | 9 =>                                               -- step along the slat: per line (Down) / per pixel
                        z_fa(19 downto 0) <= z_x;
                        if s_down = '1' then z_fb <= resize(s_ys, 24); else z_fb <= resize(s_xs & "0000000", 24); end if;
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
                    when "10"   => c_ip  <= '0' & c_ip(20 downto 1);
                    when others => c_dph <= c_dph(20 downto 0) & '0';
                end case;
            end if;

            if z_lrun = '1' then
                if z_n = 1 then z_lrun <= '0'; end if;
                z_n <= z_n - 1;
            end if;

            -- the across-axis origin: slat 0 is centred on the screen centre; step back to the first line / pixel
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
                    -- K6 camera pass: camera coefficient 20 g = 5 K6 / 64 (0 .. 79; 20 at 25 %, 40 at 50 %)
                    when 40 =>
                        v_t := resize(s_gk(12 downto 3) & "00", 29) + resize(s_gk(12 downto 3), 29);
                        c_cam <= v_t(12 downto 6);
                    -- palette: ground, A, B, accent (chroma kept to 8 bits)
                    when 22 => r_pl_a <= "00" & s_pal & "00";
                    when 25 =>
                        c_gy <= unsigned(r_pl_q(31 downto 22)); c_gc(1) <= unsigned(r_pl_q(21 downto 14)); c_gc(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "01";
                    when 28 =>
                        c_ay <= unsigned(r_pl_q(31 downto 22)) - 64;
                        c_ac(1) <= signed(unsigned(r_pl_q(21 downto 14)) - 128); c_ac(2) <= signed(unsigned(r_pl_q(11 downto 4)) - 128);
                        r_pl_a <= "00" & s_pal & "10";
                    when 31 =>
                        c_by <= unsigned(r_pl_q(31 downto 22)) - 64;
                        c_bc(1) <= signed(unsigned(r_pl_q(21 downto 14)) - 128); c_bc(2) <= signed(unsigned(r_pl_q(11 downto 4)) - 128);
                        r_pl_a <= "00" & s_pal & "11";
                    when 34 =>
                        c_cy <= unsigned(r_pl_q(31 downto 22)) - 64;
                        c_cc(1) <= signed(unsigned(r_pl_q(21 downto 14)) - 128); c_cc(2) <= signed(unsigned(r_pl_q(11 downto 4)) - 128);
                        c_ky <= signed(resize(unsigned(r_pl_q(31 downto 22)), 11)) - signed(resize(c_gy, 11));
                        c_kc(1) <= signed(resize(unsigned(r_pl_q(21 downto 14)), 9)) - signed(resize(c_gc(1), 9));
                        c_kc(2) <= signed(resize(unsigned(r_pl_q(11 downto 4)), 9)) - signed(resize(c_gc(2), 9));

                    -- pitch: e = k1 * 4.08 octaves (Q8), P = 16 * 2^e, 1 / P
                    when 60 => z_e <= f_acc(26 downto 16);
                    when 61 =>
                        r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                        if z_e(7 downto 0) = 0 then z_nz <= '0'; else z_nz <= '1'; end if;
                    when 63 => z_mt <= r_ex_q;
                    when 64 =>
                        c_pw <= "0000" & z_mt; z_shn <= z_sh; z_sht <= "01";
                        r_ex_a <= 0 - z_e(7 downto 0);
                    when 66 => z_mt <= r_ex_q;
                    when 72 =>
                        c_ip <= z_mt & "00000"; z_sht <= "10";
                        if z_nz = '1' then z_shn <= z_sh + 1; else z_shn <= z_sh; end if;
                    when 80 =>                       -- the across axis is y (Across) or x (Down)
                        v_x8 := resize(s_xs, 10);
                        if s_down = '1' then
                            c_xst <= shift_left(v_x8, 7); c_yst <= (others => '0');
                            c_base <= resize(shift_left(v_x8, 3), 7);
                        else
                            c_xst <= (others => '0'); c_yst <= s_ys;
                            c_base <= s_ys(10 downto 4);
                        end if;
                        c_hlim <= c_p(16 downto 5) - 16;
                        c_thk  <= c_p(16 downto 8) + 24;
                    when 81 =>
                        c_xmp  <= signed(resize(c_xst, 19)) - signed(resize(c_p, 19));
                        c_x2mp <= signed(resize(c_xst & '0', 19)) - signed(resize(c_p, 19));
                        c_ymp  <= signed(resize(c_yst, 19)) - signed(resize(c_p, 19));
                    when 92  => z_bl <= to_unsigned(205, 10) + f_acc(19 downto 10);
                    when 124 => c_hmax <= f_acc(26 downto 15);
                    when 156 =>                      -- depth = (k3 / 1023)^2 * 1.25 turns
                        v_t := shift_left(resize(f_acc(25 downto 0), 29), 2) + resize(f_acc(25 downto 0), 29);
                        c_d <= v_t(28 downto 21);
                    when 188 | 284 => c_s <= f_acc(25 downto 3);
                    when 220 | 316 => z_x <= f_acc(43 downto 24);
                    when 252 => c_dtw <= f_acc(30 downto 8);
                    when 348 => c_dsu <= f_acc(30 downto 8);
                    -- the along-slat start values = the step accumulated over the pixels / lines to the centre (p_acc)
                    when 400 =>
                        z_lrun <= '1';
                        if s_down = '0' then z_n <= s_w(11 downto 1);
                        else z_n <= f_half(s_tid); end if;
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh; z_sht <= "11";
                    -- across-axis origin
                    when 550 =>
                        v_l := signed(resize(c_p(16 downto 1), 19));
                        if s_down = '0' then z_loc <= v_l - 138240;       -- 540 lines
                        elsif s_sd = '1' then z_loc <= v_l - 184320;      -- 720 px
                        else z_loc <= v_l - 245760; end if;               -- 960 px
                        z_k <= (others => '0');
                        z_nrun <= '1';
                    when 620 =>
                        z_first <= '1';
                        if z_loc + c_xmp >= 0 then c_wrap0 <= '1'; else c_wrap0 <= '0'; end if;
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
    -- Blend table fill (zone, zc 40 .. ~205; adders only).  For co = 0 .. 8 and
    -- ws = 0 .. 16:  T = (16 ground + 2 (accent - ground) co - (accent - black) ws) / 16
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            if s_zone = '1' and s_zc = 40 then
                t_run <= '1'; t_row <= '1'; t_co <= (others => '0');
                t_ry <= signed(resize(c_gy & "0000", 15));
                for i in 1 to 2 loop t_rc(i) <= signed(resize(c_gc(i) & "0000", 14)); end loop;
            elsif t_run = '1' then
                if t_row = '1' then
                    t_ey <= t_ry; t_ec <= t_rc; t_ws <= (others => '0'); t_row <= '0';
                else
                    tt_we <= '1'; tt_wa <= t_co & t_ws;
                    tt_wd <= '0' & std_logic_vector(t_ey(14 downto 4)) & std_logic_vector(t_ec(1)(13 downto 4)) & std_logic_vector(t_ec(2)(13 downto 4));
                    t_ey <= t_ey - signed(resize(c_cy, 15));
                    for i in 1 to 2 loop t_ec(i) <= t_ec(i) - resize(c_cc(i), 14); end loop;
                    t_ws <= t_ws + 1;
                    if t_ws = 16 then
                        t_row <= '1';
                        t_ry <= t_ry + shift_left(resize(c_ky, 15), 1);
                        for i in 1 to 2 loop t_rc(i) <= t_rc(i) + shift_left(resize(c_kc(i), 14), 1); end loop;
                        t_co <= t_co + 1;
                        if t_co = 8 then t_run <= '0'; end if;
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
    -- px) drifting diagonally, with its own colours.  The luma (or the lattice)
    -- is smeared along x by a 1/64 filter and held per element: Across, one
    -- smeared line per slat (the row before the slat's first row), read 56 px
    -- ahead to take the filter lag out; Down, one sample per slat per line,
    -- filtered along y (1/16).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_dx : signed(7 downto 0);
        variable v_c  : signed(10 downto 0);
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

            -- input sample: source value (8 bits) and the face colour {y - 64, u, v}; held through blanking
            s_sav <= data_in.avid; s_savd <= s_sav;
            if data_in.avid = '1' then
                if s_src = '1' then
                    s_sv <= v_o;
                    v_iy <= resize(v_o & '0', 10) + resize(v_o, 10);
                    v_dx := signed(resize(v_tx, 8)) - 64;
                    v_c := shift_left(resize(v_dx, 11), 2) + shift_left(resize(v_dx, 11), 1) + 512;
                    v_iu <= unsigned(v_c(9 downto 0));
                    v_dx := signed(resize(v_ty, 8)) - 64;
                    v_c := shift_left(resize(v_dx, 11), 2) + shift_left(resize(v_dx, 11), 1) + 512;
                    v_iv <= unsigned(v_c(9 downto 0));
                else
                    v_l := unsigned(data_in.y(9 downto 2));
                    if v_l > 16 then s_sv <= v_l - 16; else s_sv <= (others => '0'); end if;
                    if unsigned(data_in.y) > 64 then v_iy <= unsigned(data_in.y) - 64; else v_iy <= (others => '0'); end if;
                    v_iu <= unsigned(data_in.u); v_iv <= unsigned(data_in.v);
                end if;
            end if;

            -- smear: the filter after pixel i is written at address i (every second one); 56 more past the line end
            if s_sav = '0' and s_savd = '1' then
                s_ext <= to_unsigned(56, 6);
            elsif s_ext /= 0 then
                s_ext <= s_ext - 1;
            end if;
            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
                s_wi <= (others => '0'); s_wen <= '1';
                s_srow <= z_first or not ls_n2(18);     -- line 0, or the last row before a new slat
            elsif s_sav = '1' or s_ext /= 0 then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 6), 14));
                s_wi <= s_wi + 1; s_wen <= '1';
            else
                s_wen <= '0';
            end if;
        end if;
    end process p_src;
    s_lb_we <= s_wen and s_srow and not s_wi(0);

    ---------------------------------------------------------------------------
    -- [D] Wave engine.  Every quantity is "start value per line + step per
    -- pixel"; the across axis (slat offset loc, slat number, Spread * number)
    -- steps per line in Across and per pixel in Down, the along axis (Spread *
    -- u / P and Twist * u / P, both counting DOWN from + at the left / top)
    -- the other way round, so one set of accumulators serves both.  ls_* hold
    -- the start values of the NEXT line.
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_w   : signed(18 downto 0);
        variable v_b   : signed(27 downto 0);
        variable v_a31 : signed(28 downto 0);
        variable v_sub, v_en : std_logic;
    begin
        if rising_edge(clk) then
            -- line registers: wrap candidates are pre-registered (ls_loc is stable between line starts)
            ls_n1 <= resize(ls_loc + c_yst, 17);
            ls_n2 <= signed(resize(ls_loc, 19)) + c_ymp;
            -- along-slat start values: + step for each pixel / line to the centre (zone), - step per line (Down)
            v_sub := not z_lrun;
            v_en := z_lrun or (g_start and s_down);
            v_b := signed(resize(c_dsu(22 downto 2), 28));
            if v_sub = '1' then v_b := not v_b; end if;
            v_a31 := (ls_su & '1') + (v_b & v_sub);
            if s_zone = '1' and s_zc = 400 then ls_su <= (others => '0');
            elsif v_en = '1' then ls_su <= v_a31(28 downto 1); end if;
            v_b := signed(resize(c_dtw(22 downto 2), 28));
            if v_sub = '1' then v_b := not v_b; end if;
            v_a31 := (ls_tw & '1') + (v_b & v_sub);
            if s_zone = '1' and s_zc = 400 then ls_tw <= (others => '0');
            elsif v_en = '1' then ls_tw <= v_a31(28 downto 1); end if;
            -- Spread * slat number: - S per step back in the zone's origin search, + S at each slat wrap (one adder)
            v_sub := z_nrun;
            v_en := (z_nrun and z_loc(18)) or (g_start and not ls_n2(18));
            v_b := signed(resize(c_s(22 downto 2), 28));
            if v_sub = '1' then v_b := not v_b; end if;
            v_a31 := (ls_sk & '1') + (v_b & v_sub);
            if s_zone = '1' and s_zc = 550 then
                ls_sk <= (others => '0');
            elsif v_en = '1' then
                ls_sk <= v_a31(28 downto 1);
            end if;
            if s_zone = '1' and s_zc = 620 then
                ls_loc <= unsigned(z_loc(16 downto 0)); ls_k <= z_k;
            elsif g_start = '1' then
                if ls_n2(18) = '0' then
                    ls_loc <= unsigned(ls_n2(16 downto 0)); ls_k <= ls_k + 1;
                else
                    ls_loc <= ls_n1;
                end if;
            end if;

            -- pixel accumulators (stage 1); the wrap flag of the next step is registered a pixel ahead
            if g_start = '1' then
                a_su <= ls_su; a_tw <= ls_tw; a_sk <= ls_sk; a_loc <= ls_loc;
                a_wrap <= c_wrap0; a_nw <= '0';
                l_k0 <= ls_k; l_first <= z_first;
            elsif g_avid = '1' then
                if s_down = '0' then
                    a_su <= a_su - signed(resize(c_dsu(22 downto 2), 28));
                    a_tw <= a_tw - signed(resize(c_dtw(22 downto 2), 28));
                end if;
                if a_wrap = '1' then
                    v_w := signed(resize(a_loc, 19)) + c_xmp;
                    a_loc <= unsigned(v_w(16 downto 0));
                    a_sk  <= a_sk + signed(resize(c_s(22 downto 2), 28));
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
        variable v_a   : unsigned(19 downto 0);
        variable v_pos : unsigned(11 downto 0);
        variable v_al  : signed(15 downto 0);
        variable v_p   : unsigned(17 downto 0);
        variable v_x   : unsigned(13 downto 0);
        variable v_d1  : signed(14 downto 0);
        variable v_k   : unsigned(7 downto 0);
        variable v_sel : unsigned(13 downto 0);
        variable v_st  : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            s_nw(2) <= a_nw;
            for i in 3 to 18 loop s_nw(i) <= s_nw(i - 1); end loop;

            -- 2: magnitudes for the radial metric and the linear sum (14 fraction bits from here on), the slat offset t (Q4)
            v_a := unsigned(abs(a_sk(27 downto 8))); q2_ask <= v_a(18 downto 0);
            v_a := unsigned(abs(a_su(27 downto 8))); q2_asu <= v_a(18 downto 0);
            v_a := unsigned(abs(a_tw(27 downto 8))); q2_atw <= v_a(13 downto 0);
            q2_lin <= unsigned(a_sk(21 downto 8)) - unsigned(a_tw(21 downto 8));
            q2_t <= signed(resize(a_loc(16 downto 4), 13)) - signed(resize(c_phq, 13));
            -- 3: compare; the twist / phase term
            if q2_ask >= q2_asu then q3_ge <= '1'; else q3_ge <= '0'; end if;
            q3_ask <= q2_ask(14 downto 0); q3_asu <= q2_asu(14 downto 0); q3_lin <= q2_lin;
            if s_rad = '1' then q3_y <= q2_atw - c_ph(25 downto 12);
            else q3_y <= 0 - c_ph(25 downto 12); end if;
            -- 4: octagonal metric max + min / 2 (Radial) or slat number + twist (Linear), + the phase term
            if s_rad = '0' then
                v_sel := q3_lin; q4_xb <= (others => '0');
            elsif q3_ge = '1' then
                v_sel := q3_ask(13 downto 0); q4_xb <= q3_asu(14 downto 1);
            else
                v_sel := q3_asu(13 downto 0); q4_xb <= q3_ask(14 downto 1);
            end if;
            q4_xa <= v_sel + q3_y;
            -- 5: theta (14 bits per cycle)
            q5_th <= q4_xa + q4_xb;
            -- 6: wave ROM address: quarter-wave fold; Snap = the first sixteenth stretched over the quarter
            v_pos := q5_th(11 downto 0);
            if q5_th(12) = '1' then v_pos := not v_pos; end if;
            if s_snap = '0' then r_w_a <= v_pos(11 downto 2);
            elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
            else r_w_a <= v_pos(9 downto 0); end if;
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

            -- source hold reads: address at 8, RAM at 9, fabric at 10
            if s_gav_sr(6) = '1' then s_lb_ra <= s_lb_ra + 1; else s_lb_ra <= to_unsigned(56, 11); end if;
            v_k := k8;
            if s_gav_sr(5) = '1' and s_gav_sr(6) = '0' then v_k := unsigned(l_k0);
            elsif s_nw(7) = '1' then v_k := k8 + 1; end if;
            k8 <= v_k;
            if s_gav_sr(5) = '1' then sl_ra <= v_k; else sl_ra <= (not v_k(7)) & v_k(6 downto 0); end if;
            k9 <= k8; k10 <= k9;
            q10_e <= unsigned(sl_q(13 downto 0));
            if s_down = '1' then q10_src <= unsigned(sl_q(13 downto 6)); else q10_src <= unsigned(s_lb_q); end if;
            -- Down: at the slat's last pixel, advance its filter with the smeared sample
            v_x := s_y1(13 downto 6) & "000000";
            v_d1 := signed(resize(v_x, 15)) - signed(resize(q10_e, 15));
            sl_wd <= "00" & std_logic_vector(unsigned(signed(q10_e) + resize(shift_right(v_d1, 4), 14)));
            sl_wa <= k10;
            if s_gav_sr(8) = '1' and (s_nw(9) = '1' or s_gav_sr(7) = '0') then sl_we <= '1'; else sl_we <= '0'; end if;

            -- 11: source term src * c_cam (K6 camera pass; c_cam = 20 g, so g = 1 is v0.1.0's Video x20:
            -- white = 0.27 turn) as two gated partial sums
            q11_pa <= f_gu(q10_src, c_cam(0), 12) + shift_left(f_gu(q10_src, c_cam(1), 12), 1)
                    + shift_left(f_gu(q10_src, c_cam(2), 12), 2) + shift_left(f_gu(q10_src, c_cam(3), 12), 3);
            q11_pb <= f_gu(q10_src, c_cam(4), 11) + shift_left(f_gu(q10_src, c_cam(5), 11), 1)
                    + shift_left(f_gu(q10_src, c_cam(6), 11), 2);
            -- 12: their sum (Osc x2), modulo a turn; +-Depth * wave
            v_st := resize(q11_pa, 14) + shift_left(resize(q11_pb, 14), 4);
            if s_src = '1' then q12_st <= v_st(12 downto 0) & '0'; else q12_st <= v_st; end if;
            v_al := signed(resize(q11_aw, 16));
            if s_neg(11) = '1' then v_al := -v_al; end if;
            q12_aw <= v_al;
        end if;
    end process p_wave;

    ---------------------------------------------------------------------------
    -- [E] The look: slat silhouette, coverage, palette + AA blend
    ---------------------------------------------------------------------------
    p_look : process(clk)
        variable v_h   : unsigned(20 downto 0);
        variable v_ts  : unsigned(13 downto 0);
        variable v_hs  : unsigned(12 downto 0);
        variable v_t   : signed(14 downto 0);
        variable v_ab  : signed(12 downto 0);
        variable v_hp  : unsigned(11 downto 0);
        variable v_adh : unsigned(12 downto 0);
        variable v_s   : signed(15 downto 0);
        variable v_co  : signed(15 downto 0);
        variable v_a, v_e : signed(8 downto 0);
        variable v_k   : unsigned(7 downto 0);
        variable v_m0  : unsigned(3 downto 0);
        variable v_l   : signed(4 downto 0);
        variable v_n   : unsigned(4 downto 0);
        variable v_w7  : unsigned(7 downto 0);
        variable v_s11 : unsigned(10 downto 0);
        variable v_py  : unsigned(16 downto 0);
        variable v_pc  : signed(12 downto 0);
        variable v_oy  : signed(11 downto 0);
        variable v_oc  : signed(10 downto 0);
        variable v_al  : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 13: the state: alpha = +-Depth * wave + source (14 bits per turn; K6 camera pass: was stage 12);
            -- |cos|, |sin| address by quarter turn
            v_al := q12_aw + signed(resize(q12_st, 16));
            q13_al <= unsigned(v_al(13 downto 0));
            if v_al(12) = '1' then r_cs_a <= not unsigned(v_al(11 downto 3)); else r_cs_a <= unsigned(v_al(11 downto 3)); end if;
            q13_sg <= not v_al(12);                         -- sin alpha * cos alpha > 0: thin face on the +t side
            q14_sg <= q13_sg;
            -- 14: ROM; 15: fabric copies
            q15_cos <= r_cs_q(15 downto 7); q15_sin <= r_cs_q(6 downto 2);
            q15_sig <= q14_sg; q16_sig <= q15_sig; q17_sig <= q16_sig; q18_sig <= q17_sig;

            -- silhouette ----------------------------------------------------------------------------------
            -- 16 .. 18: blade half-width * |cos|, thickness * |sin| as gated sums
            q16_h01 <= f_gu(c_hmax, q15_cos(0), 14) + shift_left(f_gu(c_hmax, q15_cos(1), 14), 1);
            q16_h23 <= f_gu(c_hmax, q15_cos(2), 14) + shift_left(f_gu(c_hmax, q15_cos(3), 14), 1);
            q16_h45 <= f_gu(c_hmax, q15_cos(4), 14) + shift_left(f_gu(c_hmax, q15_cos(5), 14), 1);
            q16_h67 <= f_gu(c_hmax, q15_cos(6), 14) + shift_left(f_gu(c_hmax, q15_cos(7), 14), 1);
            q16_h8  <= f_gu(c_hmax, q15_cos(8), 12);
            q16_t01 <= f_gu(c_thk, q15_sin(0), 11) + shift_left(f_gu(c_thk, q15_sin(1), 11), 1);
            q16_t23 <= f_gu(c_thk, q15_sin(2), 11) + shift_left(f_gu(c_thk, q15_sin(3), 11), 1);
            q16_t4  <= f_gu(c_thk, q15_sin(4), 9);
            q17_q <= resize(q16_h01, 16) + shift_left(resize(q16_h23, 16), 2);
            q17_r <= resize(q16_h45, 17) + shift_left(resize(q16_h67, 17), 2) + shift_left(resize(q16_h8, 17), 4);
            v_ts := resize(q16_t01, 14) + shift_left(resize(q16_t23, 14), 2) + shift_left(resize(q16_t4, 14), 4);
            q17_ts <= v_ts(13 downto 6);
            v_h := resize(q17_q, 21) + shift_left(resize(q17_r, 21), 4);
            q18_hc <= v_h(20 downto 9);
            q18_ts <= q17_ts;
            q18_t <= signed(tr_q(12 downto 0));
            -- 19: H (never past the pitch: slats do not overlap), the thin face's inner edge, |t|, the fade caps
            v_hs := resize(q18_hc, 13) + q18_ts;
            if v_hs > c_hlim then q19_h <= c_hlim; else q19_h <= v_hs(11 downto 0); end if;
            v_t := resize(q18_t, 15);
            if q18_sig = '0' then v_t := -v_t; end if;
            q19_stp <= v_t + signed(resize(q18_ts & '0', 15));
            v_ab := abs(q18_t);
            q19_at <= unsigned(v_ab(11 downto 0));
            if q18_hc(11 downto 5) /= 0 then q19_cb <= to_unsigned(8, 4); else q19_cb <= '0' & q18_hc(4 downto 2); end if;
            if q18_ts(7 downto 4) /= 0 then q19_ct <= to_unsigned(8, 4); else q19_ct <= '0' & q18_ts(3 downto 1); end if;
            q20_cb <= q19_cb; q21_cb <= q20_cb; q22_cb <= q21_cb; q23_cb <= q22_cb;
            q20_ct <= q19_ct; q21_ct <= q20_ct; q22_ct <= q21_ct; q23_ct <= q22_ct;
            -- previous line's H of this slat column (Down): address at 17, RAM at 18, fabric at 19
            v_k := k17;
            if s_gav_sr(14) = '1' and s_gav_sr(15) = '0' then v_k := unsigned(l_k0);
            elsif s_nw(16) = '1' then v_k := k17 + 1; end if;
            k17 <= v_k;
            if s_gav_sr(14) = '1' then hp_ra <= v_k; else hp_ra <= (not v_k(7)) & v_k(6 downto 0); end if;
            k18 <= k17; k19 <= k18;
            q19_hpv <= unsigned(hp_q(11 downto 0));
            hp_wd <= "0000" & std_logic_vector(q19_h);
            hp_wa <= k19;
            if s_gav_sr(17) = '1' and (s_nw(18) = '1' or s_gav_sr(16) = '0') then hp_we <= '1'; else hp_we <= '0'; end if;
            -- 20: signed distances to the silhouette and to the thin face; H step along the slat
            v_s := resize(signed(resize(q19_h, 14)) - signed(resize(q19_at, 14)), 16);
            if v_s > 255 then q20_fo <= to_signed(255, 9); elsif v_s < -256 then q20_fo <= to_signed(-256, 9); else q20_fo <= v_s(8 downto 0); end if;
            v_s := resize(q19_stp, 16) - signed(resize(q19_h, 16));
            if v_s > 255 then q20_fe <= to_signed(255, 9); elsif v_s < -256 then q20_fe <= to_signed(-256, 9); else q20_fe <= v_s(8 downto 0); end if;
            q20_h  <= q19_h;
            if s_down = '1' then v_hp := q19_hpv; else v_hp := q20_h; end if;
            if (s_down = '1' and l_first = '1') or (s_down = '0' and s_gav_sr(18) = '0') then
                q20_dh <= (others => '0');                                     -- the field's first line / the line's first pixel
            else
                q20_dh <= signed(resize(q19_h, 13)) - signed(resize(v_hp, 13));
            end if;
            -- 21: ramp width = across-axis pixel size + |dH|
            v_adh := unsigned(abs(q20_dh));
            q21_w <= resize(c_base, 13) + v_adh;
            q21_fo <= q20_fo; q21_fe <= q20_fe;
            -- 22: rounded up to a power of two (1 .. 16 px)
            if    q21_w > 128 then q22_lw <= to_unsigned(4, 3);
            elsif q21_w > 64  then q22_lw <= to_unsigned(3, 3);
            elsif q21_w > 32  then q22_lw <= to_unsigned(2, 3);
            elsif q21_w > 16  then q22_lw <= to_unsigned(1, 3);
            else                   q22_lw <= to_unsigned(0, 3); end if;
            q22_fo <= q21_fo; q22_fe <= q21_fe;
            -- 23: coverages in eighths: slab over ground, 8 - thin face
            case to_integer(q22_lw) is
                when 0      => v_a := shift_right(q22_fo, 1); v_e := shift_right(q22_fe, 1);
                when 1      => v_a := shift_right(q22_fo, 2); v_e := shift_right(q22_fe, 2);
                when 2      => v_a := shift_right(q22_fo, 3); v_e := shift_right(q22_fe, 3);
                when 3      => v_a := shift_right(q22_fo, 4); v_e := shift_right(q22_fe, 4);
                when others => v_a := shift_right(q22_fo, 5); v_e := shift_right(q22_fe, 5);
            end case;
            v_a := v_a + 4; v_co := 4 - resize(v_e, 16);
            if v_a < 0 then q23_co <= (others => '0'); elsif v_a > 8 then q23_co <= to_unsigned(8, 4); else q23_co <= unsigned(v_a(3 downto 0)); end if;
            if v_co < 0 then q23_nce <= (others => '0'); elsif v_co > 8 then q23_nce <= to_unsigned(8, 4); else q23_nce <= unsigned(v_co(3 downto 0)); end if;
            -- 24: big-face coverage = min(slab, 8 - thin); a face narrower than 2 .. 4 px fades into the other
            if q23_co < q23_nce then v_m0 := q23_co; else v_m0 := q23_nce; end if;
            v_l := signed(resize(q23_co, 5)) - signed(resize(q23_ct, 5));
            if q23_cb(3) = '0' then
                if v_m0 < q23_cb then q24_bc <= v_m0; else q24_bc <= q23_cb; end if;
            elsif v_l > signed(resize(v_m0, 5)) then
                q24_bc <= unsigned(v_l(3 downto 0));
            else
                q24_bc <= v_m0;
            end if;
            q24_co <= q23_co; q25_co <= q24_co;
            -- 25: the face's weight in sixteenths; the glint hands part of it to the accent
            v_n := 16 - resize(q24_g4, 5);      -- (q24_g4: the shade word's fabric copy)
            if q24_bc(3) = '1' then
                v_w7 := v_n & "000";
            else
                v_w7 := f_gu(v_n, q24_bc(0), 8) + shift_left(f_gu(v_n, q24_bc(1), 8), 1) + shift_left(f_gu(v_n, q24_bc(2), 8), 2);
            end if;
            q25_ws <= v_w7(7 downto 3);

            -- colour: luma 10 bits, chroma 8 bits ------------------------------------------------------------
            -- 22: shade address (alpha from its delay ring); 23: ROM; 24: fabric copies
            r_sh_a <= unsigned(ar_q(8 downto 0));
            q22_aq <= ar_q(9 downto 8); q23_aq <= q22_aq; q24_aq <= q23_aq; q25_aq <= q24_aq;
            q24_shade <= r_sh_q(15 downto 9); q24_g4 <= r_sh_q(8 downto 5);
            q25_shade <= q24_shade;
            -- 26: face weight * shade (/ 128); the blend table's address; the big face's colour: front
            -- (cos alpha > 0) = A or the picture, back = B
            if q25_ws(4) = '1' then
                v_s11 := q25_shade & "0000";
            else
                v_s11 := (f_gu(q25_shade, q25_ws(0), 11) + shift_left(f_gu(q25_shade, q25_ws(1), 11), 1))
                       + (shift_left(f_gu(q25_shade, q25_ws(2), 11), 2) + shift_left(f_gu(q25_shade, q25_ws(3), 11), 3));
            end if;
            q26_s2 <= v_s11(10 downto 4);
            tt_ra <= q25_co & q25_ws;
            if (q25_aq(1) xor q25_aq(0)) = '1' then
                q26_y <= c_by; q26_c <= c_bc;
            elsif s_face = '1' then
                q26_y <= unsigned(vr_q(25 downto 16));
                q26_c(1) <= signed(unsigned(vr_q(15 downto 8)) - 128);
                q26_c(2) <= signed(unsigned(vr_q(7 downto 0)) - 128);
            else
                q26_y <= c_ay; q26_c <= c_ac;
            end if;
            -- 27, 28: face colour * that weight as gated sums (7 bits on luma, 5 on chroma); the table word
            q27_y0 <= f_gu(q26_y, q26_s2(0), 12) + shift_left(f_gu(q26_y, q26_s2(1), 12), 1);
            q27_y1 <= f_gu(q26_y, q26_s2(2), 12) + shift_left(f_gu(q26_y, q26_s2(3), 12), 1);
            q27_y2 <= f_gu(q26_y, q26_s2(4), 12) + shift_left(f_gu(q26_y, q26_s2(5), 12), 1);
            q27_y3 <= f_gu(q26_y, q26_s2(6), 10);
            v_py := (resize(q27_y0, 17) + shift_left(resize(q27_y1, 17), 2)) + (shift_left(resize(q27_y2, 17), 4) + shift_left(resize(q27_y3, 17), 6));
            q28_py <= v_py(16 downto 7);
            q28_t <= tt_q(30 downto 0);
            v_oy := resize(signed(q28_t(30 downto 20)), 12) + signed(resize(q28_py, 12));
            -- 29: ground / accent blend + shaded face; clamp; the ground colour outside the generated avid
            if s_gav_sr(26) = '0' then q29_y <= c_gy;
            elsif v_oy < 0 then q29_y <= (others => '0');
            elsif v_oy > 1023 then q29_y <= (others => '1');
            else q29_y <= unsigned(v_oy(9 downto 0)); end if;
            for i in 1 to 2 loop
                q27_c0(i) <= f_gs(q26_c(i), q26_s2(2), 10) + shift_left(f_gs(q26_c(i), q26_s2(3), 10), 1);
                q27_c1(i) <= f_gs(q26_c(i), q26_s2(4), 10) + shift_left(f_gs(q26_c(i), q26_s2(5), 10), 1);
                q27_c2(i) <= f_gs(q26_c(i), q26_s2(6), 8);
                v_pc := (resize(q27_c0(i), 13) + shift_left(resize(q27_c1(i), 13), 2)) + shift_left(resize(q27_c2(i), 13), 4);
                q28_pc(i) <= v_pc(12 downto 5);
                if i = 1 then v_oc := resize(signed(q28_t(19 downto 10)), 11) + resize(q28_pc(i), 11);
                else v_oc := resize(signed(q28_t(9 downto 0)), 11) + resize(q28_pc(i), 11); end if;
                if s_gav_sr(26) = '0' then q29_c(i) <= c_gc(i);
                elsif v_oc < 0 then q29_c(i) <= (others => '0');
                elsif v_oc > 255 then q29_c(i) <= (others => '1');
                else q29_c(i) <= unsigned(v_oc(7 downto 0)); end if;
            end loop;
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

end architecture louver;
