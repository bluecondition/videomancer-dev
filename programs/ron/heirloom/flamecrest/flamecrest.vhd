-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Flamecrest v0.1.1 -- Heirloom phase 3 (video clips), from clip WIN_20240213_23_38_52 at 3.0 s.
--
-- v0.1.1: K5 Strokes made bold: its travel also loosens the tongue gate (none at 0, the still at 512,
-- ungated at 1023), lets strokes reach into the black ground above 512 and grows the olive shadow; width
-- 0 .. 153 (512) .. 242.
--
-- The still: a mirrored crest of flames hanging from the top of the frame on black.  Pink and white arch tops,
-- cyan and green tongues on the sides, a yellow / orange / red fan converging on a central X, black diamond notches
-- with small islands at x 540 / 1480, the flame body ending in rounded red tongue tips at y ~800 with a soft red
-- glow, hard red strokes with olive shadows, a wavy pink strip along the bottom and horizontal bars at the left edge.
-- In the clip the colouriser offset cycles: the flames fall, the black recedes and the diamonds bloom into nested
-- rings, then black opens inside the top arches (one cycle ~7 s).
--
-- The patch, as modelled (flamecrest.py has every table; C = colouriser units, 8192 per palette cycle):
--   d     = |x - 1010| (Twin: 2 |d - 505|), t = mirror fold of d * Diamonds on a 940 px lattice
--   R     = vertical ramp with a retrace at the bottom (y 990 + ripple)
--   s     = P0(t) - su(t) R  /  Q0(t) - sd(t) (R - Rd)   (a VCA: the horizontal shape deepens down the frame)
--           capped, + the notch profile NP(rd) (hole + island)
--   r     = T(t) + g(|y - 505|) + A2 S(d * Tongues)        (asymmetric triangle + two-slope vertical + tongues)
--   ring  = 0.5 - frac(r * Rings + Etch * luma)
--   pre   = s - 0.05 + 0.24 ring + Etch * luma;  Blaze: (pre - 0.12) G + 0.12 + lift;  floor in the black window
--   index = (pre + offset) mod one cycle -> palette (4 zone rows, centre .. side), black window in the palette
--   strokes = hard red on one ring phase band (jittered, on one side of each tongue), olive shadow 8 virtual px right
--
--   K1  Tongues   tongue density x0.4 .. x3 (512 = the still: 140 px)
--   K2  Diamonds  diamond lattice x0.5 .. x3 (512 = the still: notches at 540 / 1480)
--   K3  Rings     nested-band density x0.33 .. x3 (512 = the still: 100 px)
--   K4  Speed     bipolar colour cycling: flames fall (right) / rise (left); 680 = the clip's rate
--   K5  Strokes   the hard red strokes: none (0) .. the still (512) .. heavy, wide, ungated, into the black (1023)
--   K6  Etch      how far the camera (or the Osc) bends the tongues
--   S7  Rise      the crest flipped: flames rising from the bottom
--   S8  Twin      two crests side by side
--   S9  Frost     U and V exchanged: blue / teal / violet flames
--   S10 Embers    the black ground glows dark red with the hidden rings
--   S11 Source    Video / Osc
--   P12 BLAZE     colouriser gain + lift: a thin dying crest (0) .. the still (420) .. the frame ablaze (1023)
--
-- Shell: generated avid, one latency (C_LAT 28) in every mode, virtual 1080-line coordinates (SD shows the
-- centre 4:3), output conditioning (Y, U and V through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum resync,
-- luma 13/16) before the blanking gate, colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.flamecrest_pkg.all;
use work.all;

architecture flamecrest of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks

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
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    -- triangle 0..254 of an 8-bit phase
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
        variable r : unsigned(7 downto 0);
    begin
        if v(7) = '1' then r := (not v(6 downto 0)) & '0'; else r := v(6 downto 0) & '0'; end if;
        return r;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data; every consumer adds a fabric copy)
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_kt_a  : unsigned(7 downto 0) := (others => '0');
    signal r_kt_q  : std_logic_vector(52 downto 0) := (others => '0');
    signal r_np_a  : unsigned(7 downto 0) := (others => '0');
    signal r_np_q  : signed(13 downto 0) := (others => '0');
    signal r_s_a   : unsigned(7 downto 0) := (others => '0');
    signal r_s_q   : signed(8 downto 0) := (others => '0');
    signal r_rp_a  : unsigned(7 downto 0) := (others => '0');
    signal r_rp_q  : signed(7 downto 0) := (others => '0');
    signal r_pl_a  : unsigned(9 downto 0) := (others => '0');
    signal r_pl_q  : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(420 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_rise, s_twin, s_frost, s_emb, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_neg  : std_logic := '0';
    signal z_v    : unsigned(9 downto 0) := (others => '0');
    signal z_a    : unsigned(17 downto 0) := (others => '0');
    signal c_ks   : unsigned(11 downto 0) := to_unsigned(936, 12);    -- tongue phase per h (Q10 of 256/period)
    signal c_kd   : unsigned(10 downto 0) := to_unsigned(558, 11);    -- lattice units per h (Q10)
    signal c_krp  : unsigned(9 downto 0) := to_unsigned(164, 10);     -- ring cycles per Q2 px (Q16)
    signal c_kw   : unsigned(8 downto 0) := to_unsigned(153, 9);      -- stroke half-width (1024 per ring cycle)
    signal c_eg0  : unsigned(8 downto 0) := to_unsigned(78, 9);       -- etch gain (camera)
    signal c_eg   : unsigned(11 downto 0) := to_unsigned(78, 12);     -- etch gain (x6 in Osc mode)
    signal c_g    : unsigned(7 downto 0) := to_unsigned(64, 8);       -- Blaze gain (Q6)
    signal c_lift : signed(12 downto 0) := (others => '0');           -- Blaze lift (C)
    signal c_spd  : unsigned(7 downto 0) := to_unsigned(20, 8);       -- colour-cycle step per field (C)
    signal c_off  : unsigned(12 downto 0) := (others => '0');         -- colour-cycle offset (C, mod 8192)
    signal c_msq  : unsigned(17 downto 0) := (others => '0');
    -- x-phase DDA start values and steps (per field, by mode)
    signal c_rp0, c_jp0, c_kp0 : unsigned(15 downto 0) := (others => '0');
    signal c_rps, c_jps, c_kps : unsigned(11 downto 0) := (others => '0');
    signal c_xb0  : unsigned(11 downto 0) := (others => '0');
    signal c_shn  : unsigned(3 downto 0) := to_unsigned(8, 4);        -- shadow offset in pixels
    signal c_shb  : unsigned(3 downto 0) := to_unsigned(8, 4);        -- its base by mode
    signal c_sth  : signed(10 downto 0) := to_signed(-40, 11);        -- stroke tongue gate (S > sth)
    signal c_klo  : unsigned(7 downto 0) := to_unsigned(36, 8);       -- stroke palette range: (idx - klo) mod 256 < kwid
    signal c_kwid : unsigned(8 downto 0) := to_unsigned(134, 9);
    signal p21_klit : std_logic := '0';

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(31 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (computes the NEXT line on one serial multiplier: 12 iterations, result at load + 13)
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_v    : unsigned(10 downto 0) := (others => '0');
    signal e_ady  : unsigned(10 downto 0) := (others => '0');
    signal e_amn  : unsigned(10 downto 0) := (others => '0');
    signal e_amx  : unsigned(10 downto 0) := (others => '0');
    signal e_g1   : unsigned(22 downto 0) := (others => '0');
    signal e_fb   : unsigned(7 downto 0) := (others => '0');
    signal e_t1, e_t2 : unsigned(7 downto 0) := (others => '0');
    signal e_sw1  : unsigned(8 downto 0) := (others => '0');
    signal e_par  : std_logic := '0';
    signal m_a    : unsigned(22 downto 0) := (others => '0');
    signal m_b    : unsigned(11 downto 0) := (others => '0');
    signal m_acc  : unsigned(22 downto 0) := (others => '0');
    -- next-line values (loaded into l_* at the line start)
    signal n_rl   : unsigned(11 downto 0) := (others => '0');
    signal n_gq   : unsigned(11 downto 0) := (others => '0');
    signal n_kdy  : unsigned(9 downto 0) := (others => '0');
    signal n_eb4  : signed(13 downto 0) := (others => '0');
    signal n_sw   : unsigned(11 downto 0) := to_unsigned(200, 12);
    signal n_rs   : signed(10 downto 0) := (others => '0');
    signal n_jl, n_jl2 : unsigned(7 downto 0) := (others => '0');
    signal n_par  : std_logic := '0';
    signal l_rl   : unsigned(11 downto 0) := (others => '0');
    signal l_gq   : unsigned(11 downto 0) := (others => '0');
    signal l_kdy  : unsigned(9 downto 0) := (others => '0');
    signal l_eb4  : signed(13 downto 0) := (others => '0');
    signal l_sw   : unsigned(11 downto 0) := to_unsigned(200, 12);
    signal l_rs   : signed(10 downto 0) := (others => '0');
    signal l_jl, l_jl2 : unsigned(7 downto 0) := (others => '0');
    signal l_par  : std_logic := '0';

    -- beam (stage 0)
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_rp, b_jp, b_kp : unsigned(15 downto 0) := (others => '0');

    -- pixel path (signal pN_* = valid at stage N)
    signal p1_ins : std_logic := '0';
    signal p1_xe  : unsigned(11 downto 0) := (others => '0');
    signal p12_jit : unsigned(5 downto 0) := (others => '0');
    signal p2_d   : unsigned(10 downto 0) := (others => '0');
    signal p3_d   : unsigned(10 downto 0) := (others => '0');
    signal p3_rip : signed(7 downto 0) := (others => '0');
    signal p4_mdl : unsigned(16 downto 0) := (others => '0');
    signal p4_mdh : unsigned(15 downto 0) := (others => '0');
    signal p4_msl : unsigned(16 downto 0) := (others => '0');
    signal p4_msh : unsigned(16 downto 0) := (others => '0');
    signal p4_eq  : signed(13 downto 0) := (others => '0');
    signal p5_traw : unsigned(11 downto 0) := (others => '0');
    signal p5_e   : unsigned(9 downto 0) := (others => '0');
    signal p5_ef  : std_logic_vector(1 downto 0) := "00";   -- 00 above (line R), 01 retrace, 10 below the retrace
    signal p5_abv : std_logic := '0';
    signal p6_t   : unsigned(10 downto 0) := (others => '0');
    signal p6_rp  : unsigned(20 downto 0) := (others => '0');
    signal p6_ef  : std_logic_vector(1 downto 0) := "00";
    signal p7_t   : unsigned(9 downto 0) := (others => '0');
    signal p7_lo  : std_logic := '0';
    signal p7_a   : unsigned(9 downto 0) := (others => '0');
    signal p7_s   : signed(8 downto 0) := (others => '0');
    signal p7_r   : unsigned(11 downto 0) := (others => '0');
    signal p8_tq  : unsigned(10 downto 0) := (others => '0');
    signal p8_rdx : unsigned(9 downto 0) := (others => '0');
    signal p8_s   : signed(8 downto 0) := (others => '0');
    signal p8_z   : unsigned(1 downto 0) := (others => '0');
    signal p8_r   : unsigned(11 downto 0) := (others => '0');
    signal p9_r   : signed(13 downto 0) := (others => '0');
    signal p9_rr  : unsigned(11 downto 0) := (others => '0');
    signal p10_up : std_logic := '0';
    signal p10_base : signed(14 downto 0) := (others => '0');
    signal p10_coef : unsigned(13 downto 0) := (others => '0');
    signal p10_op : unsigned(11 downto 0) := (others => '0');
    signal p10_ra, p10_rb : signed(19 downto 0) := (others => '0');
    signal p10_la : unsigned(13 downto 0) := (others => '0');
    signal p10_lb : unsigned(13 downto 0) := (others => '0');
    signal p11_pl, p11_ph : unsigned(19 downto 0) := (others => '0');
    signal p11_base : signed(14 downto 0) := (others => '0');
    signal p11_np : signed(13 downto 0) := (others => '0');
    signal p11_ru : unsigned(15 downto 0) := (others => '0');
    signal p11_lu : unsigned(15 downto 0) := (others => '0');
    signal p11_lp : unsigned(14 downto 0) := (others => '0');   -- luma * etch >> 5 (colour level)
    signal p12_prod : unsigned(13 downto 0) := (others => '0');
    signal p12_base : signed(14 downto 0) := (others => '0');
    signal p12_np : signed(15 downto 0) := (others => '0');
    signal p12_u  : unsigned(15 downto 0) := (others => '0');
    signal p13_sa : signed(15 downto 0) := (others => '0');
    signal p13_np : signed(15 downto 0) := (others => '0');
    signal p13_g  : signed(10 downto 0) := (others => '0');
    signal p13_f  : unsigned(9 downto 0) := (others => '0');
    signal p13_uk : unsigned(9 downto 0) := (others => '0');
    signal p13_m4 : unsigned(3 downto 0) := (others => '0');
    signal p14_s  : signed(15 downto 0) := (others => '0');
    signal p14_ring : signed(11 downto 0) := (others => '0');
    signal p14_du : unsigned(9 downto 0) := (others => '0');
    signal p15_pre : signed(15 downto 0) := (others => '0');
    signal p15_k1 : std_logic := '0';
    signal p16_pd : signed(15 downto 0) := (others => '0');
    signal p17_ga, p17_gb : signed(22 downto 0) := (others => '0');
    signal p17_pd : signed(15 downto 0) := (others => '0');
    signal p_ret  : std_logic_vector(31 downto 0) := (others => '0');    -- in the bottom retrace at stage k
    signal p_sg   : std_logic_vector(31 downto 0) := (others => '0');    -- strokes allowed on this part of the tongue
    signal p18_pre : signed(16 downto 0) := (others => '0');
    signal p19_pre : signed(16 downto 0) := (others => '0');
    signal p20_idx : unsigned(7 downto 0) := (others => '0');
    signal p21_lit : std_logic := '0';
    signal p22_key : std_logic := '0';
    -- stage pipes for flags carried down the path
    type t_z_pipe is array (9 to 21) of unsigned(1 downto 0);
    signal pz : t_z_pipe := (others => (others => '0'));
    signal p_ins : std_logic_vector(31 downto 0) := (others => '0');     -- p_ins(k) = in-strip flag at stage k
    signal p_abv : std_logic_vector(31 downto 0) := (others => '0');     -- above-retrace (strokes allowed) at stage k
    type t_m4_pipe is array (14 to 24) of unsigned(3 downto 0);
    signal pm4 : t_m4_pipe := (others => (others => '0'));
    signal plm3 : unsigned(7 downto 0) := (others => '0');
    type t_r_pipe is array (8 to 9) of unsigned(11 downto 0);
    signal pR : t_r_pipe := (others => (others => '0'));

    signal p_k14  : std_logic_vector(31 downto 0) := (others => '0');    -- stroke band test chain
    signal k_sr   : std_logic_vector(15 downto 0) := (others => '0');    -- final stroke key by pixel (bit 0 = stage 22)
    signal p23_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal p23_lit, p24_lit : std_logic := '0';
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- block-RAM rings: the box's 8-back sample (Y U V) and the syncs (C_LAT back); one write counter
    type t_br is array (0 to 255) of std_logic_vector(31 downto 0);
    signal br : t_br := (others => x"10200800");             -- black: Y 64, U 512, V 512
    type t_sr is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_sr := (others => "1111");
    attribute no_rw_check : boolean;
    attribute no_rw_check of br : signal is true;
    attribute no_rw_check of sr : signal is true;
    signal o_wa, o_ra, o_sa : unsigned(7 downto 0) := (others => '0');
    signal o_rq : std_logic_vector(31 downto 0) := x"10200800";
    signal o_q  : std_logic_vector(31 downto 0) := x"10200800";
    signal o_sq : std_logic_vector(3 downto 0) := "1111";
    signal o_so : std_logic_vector(3 downto 0) := "1111";
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y26, s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_FC_EXP2(to_integer(r_ex_a));
            r_kt_q <= C_FC_KT(to_integer(r_kt_a));
            r_np_q <= C_FC_NP(to_integer(r_np_a));
            r_s_q  <= C_FC_S(to_integer(r_s_a));
            r_rp_q <= C_FC_RIP(to_integer(r_rp_a));
            r_pl_q <= C_FC_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_p  : unsigned(9 downto 0);
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
                s_rise  <= registers_in(6)(0);
                s_twin  <= registers_in(6)(1);
                s_frost <= registers_in(6)(2);
                s_emb   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                -- x-phase DDAs (mod 2^16): start = x0 * k, step = xs * k (k = 41 ripple, 800 / 340 jitter)
                case to_integer(s_xsr) is
                    when 4 =>
                        s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); c_shb <= to_unsigned(4, 4);
                        c_rp0 <= to_unsigned(19680, 16); c_rps <= to_unsigned(164, 12);
                        c_jp0 <= to_unsigned(56320, 16);  c_jps <= to_unsigned(3200, 12);
                        c_kp0 <= to_unsigned(32128, 16);  c_kps <= to_unsigned(1360, 12);
                    when 3 =>
                        s_x0 <= (others => '0'); s_oxs <= to_unsigned(153, 9); c_shb <= to_unsigned(5, 4);
                        c_rp0 <= (others => '0'); c_rps <= to_unsigned(123, 12);
                        c_jp0 <= (others => '0'); c_jps <= to_unsigned(2400, 12);
                        c_kp0 <= (others => '0'); c_kps <= to_unsigned(1020, 12);
                    when others =>
                        s_x0 <= (others => '0'); s_oxs <= to_unsigned(102, 9); c_shb <= to_unsigned(8, 4);
                        c_rp0 <= (others => '0'); c_rps <= to_unsigned(82, 12);
                        c_jp0 <= (others => '0'); c_jps <= to_unsigned(1600, 12);
                        c_kp0 <= (others => '0'); c_kps <= to_unsigned(680, 12);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from load + 17.
            -- The slot number is decoded at 30 + 24 k and the operands pre-muxed at 31 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  =>                                                  -- tongues exponent
                        z_fa <= resize(z_v, 32);
                        if z_neg = '1' then z_fb <= to_unsigned(169, 16); else z_fb <= to_unsigned(203, 16); end if;
                    when 1  => z_fa <= to_unsigned(936, 32);  z_fb <= z_mt;      -- kS
                    when 2  =>                                                  -- diamonds exponent
                        z_fa <= resize(z_v, 32);
                        if z_neg = '1' then z_fb <= to_unsigned(128, 16); else z_fb <= to_unsigned(203, 16); end if;
                    when 3  => z_fa <= to_unsigned(558, 32);  z_fb <= z_mt;      -- kD
                    when 4  => z_fa <= resize(z_v, 32); z_fb <= to_unsigned(203, 16);   -- rings exponent
                    when 5  => z_fa <= to_unsigned(164, 32);  z_fb <= z_mt;      -- krp
                    when 6  =>                                                  -- stroke width, two slopes
                        if s_g5(12 downto 3) > 512 then z_fa <= resize(s_g5(12 downto 3) - 512, 32); z_fb <= to_unsigned(180, 16);
                        else z_fa <= resize(s_g5(12 downto 3), 32); z_fb <= to_unsigned(307, 16); end if;
                    when 7  => z_fa <= resize(s_g6(12 downto 3), 32); z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 8  => z_fa <= resize(s_vm, 32); z_fb <= resize(s_vm, 16);       -- speed^2
                    when 9  => z_fa <= resize(c_msq, 32); z_fb <= to_unsigned(51, 16);  -- x 51
                    when 10 =>                                                  -- blaze exponent
                        z_fa <= resize(z_v, 32);
                        if z_neg = '1' then z_fb <= to_unsigned(156, 16); else z_fb <= to_unsigned(144, 16); end if;
                    when 11 => z_fa <= to_unsigned(64, 32);   z_fb <= z_mt;      -- G
                    when others =>                                              -- blaze lift
                        z_fa <= resize(z_v, 32);
                        if z_neg = '1' then z_fb <= to_unsigned(891, 16); else z_fb <= to_unsigned(870, 16); end if;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20 =>                      -- colour cycle (last field's step)
                        if s_vneg = '1' then c_off <= c_off - resize(c_spd, 13);
                        else c_off <= c_off + resize(c_spd, 13); end if;
                        c_xb0 <= resize(s_x0, 12);
                    when 22 =>                      -- tongues: v = |k1 - 512|
                        v_p := s_g1(12 downto 3);
                        if v_p >= 512 then z_v <= v_p - 512; z_neg <= '0'; else z_v <= 512 - v_p; z_neg <= '1'; end if;
                    -- slot 0 result at 49
                    when 50 =>
                        if z_neg = '1' then z_e <= -signed(resize(f_acc(19 downto 8), 12));
                        else z_e <= signed(resize(f_acc(19 downto 8), 12)); end if;
                    when 51 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 53 =>
                        z_mt <= r_ex_q;
                        v_p := s_g2(12 downto 3);           -- diamonds: v = |k2 - 512|
                        if v_p >= 512 then z_v <= v_p - 512; z_neg <= '0'; else z_v <= 512 - v_p; z_neg <= '1'; end if;
                    -- slot 1 (kS) result at 73; slot 2 loads at 80 (z_v set at 53)
                    when 74 =>
                        case to_integer(z_sh) is            -- 936 * mt >> (15 - sh), sh = -2 .. 1
                            when -2     => c_ks <= f_acc(28 downto 17);
                            when -1     => c_ks <= f_acc(27 downto 16);
                            when 0      => c_ks <= f_acc(26 downto 15);
                            when others => c_ks <= f_acc(25 downto 14);
                        end case;
                    when 98 =>
                        if z_neg = '1' then z_e <= -signed(resize(f_acc(19 downto 8), 12));
                        else z_e <= signed(resize(f_acc(19 downto 8), 12)); end if;
                    when 99  => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 101 =>
                        z_mt <= r_ex_q;
                        v_p := s_g3(12 downto 3);           -- rings: v = |k3 - 512|
                        if v_p >= 512 then z_v <= v_p - 512; z_neg <= '0'; else z_v <= 512 - v_p; z_neg <= '1'; end if;
                    when 122 =>                             -- kD = 558 * mt >> (15 - sh), sh = -1 .. 1
                        case to_integer(z_sh) is
                            when -1     => c_kd <= f_acc(26 downto 16);
                            when 0      => c_kd <= f_acc(25 downto 15);
                            when others => c_kd <= f_acc(24 downto 14);
                        end case;
                    when 146 =>
                        if z_neg = '1' then z_e <= -signed(resize(f_acc(19 downto 8), 12));
                        else z_e <= signed(resize(f_acc(19 downto 8), 12)); end if;
                    when 147 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 149 => z_mt <= r_ex_q;
                    when 170 =>                             -- krp = 164 * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_krp <= f_acc(26 downto 17);
                            when -1     => c_krp <= f_acc(25 downto 16);
                            when 0      => c_krp <= f_acc(24 downto 15);
                            when others => c_krp <= f_acc(23 downto 14);
                        end case;
                    when 194 =>                                    -- k5 307 >> 10, or 153 + (k5 - 512) 180 >> 10
                        if s_g5(12 downto 3) > 512 then c_kw <= to_unsigned(153, 9) + f_acc(18 downto 10);
                        else c_kw <= f_acc(18 downto 10); end if;
                    when 195 =>                                    -- tongue gate 280 - 5 k5 / 8; reach e = (k5 - 512) / 8
                        v_p := s_g5(12 downto 3);
                        c_sth <= to_signed(280, 11) - signed(resize(shift_right(shift_left(resize(v_p, 13), 2) + resize(v_p, 13), 3), 11));
                        if v_p > 512 then
                            c_klo  <= to_unsigned(36, 8) - resize(shift_right(v_p - 512, 3), 8);
                            c_kwid <= to_unsigned(134, 9) + resize(shift_right(v_p - 512, 2) and "1111111110", 9);
                            if s_xs = 2 then c_shn <= c_shb + resize(shift_right(v_p - 512, 6), 4);
                            else c_shn <= c_shb + resize(shift_right(v_p - 512, 7), 4); end if;
                        else
                            c_klo <= to_unsigned(36, 8); c_kwid <= to_unsigned(134, 9); c_shn <= c_shb;
                        end if;
                    when 218 => c_eg0 <= f_acc(19 downto 11);      -- k6^2 >> 11
                    when 219 =>
                        if s_src = '1' then c_eg <= shift_left(resize(c_eg0, 12), 2) + shift_left(resize(c_eg0, 12), 1);
                        else c_eg <= resize(c_eg0, 12); end if;
                    when 242 => c_msq <= f_acc(17 downto 0);       -- m^2
                    when 266 =>
                        c_spd <= f_acc(23 downto 16);              -- m^2 * 51 >> 16
                        v_p := s_gp(12 downto 3);                  -- blaze: v = |p - 420|
                        if v_p >= 420 then z_v <= v_p - 420; z_neg <= '0'; else z_v <= 420 - v_p; z_neg <= '1'; end if;
                    when 290 =>
                        if z_neg = '1' then z_e <= -signed(resize(f_acc(19 downto 8), 12));
                        else z_e <= signed(resize(f_acc(19 downto 8), 12)); end if;
                    when 291 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 293 => z_mt <= r_ex_q;
                    when 314 =>                             -- G = 64 * mt >> (15 - sh), sh = -1 .. 1
                        case to_integer(z_sh) is
                            when -1     => c_g <= f_acc(23 downto 16);
                            when 0      => c_g <= f_acc(22 downto 15);
                            when others => c_g <= f_acc(21 downto 14);
                        end case;
                    when 338 =>                             -- lift = +-(|p - 420| * 870 or 891) >> 8
                        if z_neg = '1' then c_lift <= -signed(resize(f_acc(20 downto 8), 13));
                        else c_lift <= signed(resize(f_acc(20 downto 8), 13)); end if;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
    -- rise, field-latched width)
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer
    -- write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line's values on one serial multiplier (a x b, b <= 12
    -- bits; load at clock L, result in m_acc from L + 13)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d : signed(11 downto 0);
        variable v_v : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_par <= '0';
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 20);
                e_lc <= (others => '0');
                e_par <= not e_par;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(21 downto 0) & '0';
            m_b <= '0' & m_b(11 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_v := e_vy(18 downto 8);
                    if v_v > 1079 then v_v := to_unsigned(1079, 11); end if;
                    if s_rise = '1' then v_v := 1079 - v_v; end if;
                    e_v <= v_v;
                    n_par <= e_par;
                when 2 =>
                    v_d := signed(resize(e_v, 12)) - 505;
                    if v_d < 0 then v_d := -v_d; end if;
                    e_ady <= unsigned(v_d(10 downto 0));
                    n_eb4 <= shift_left(signed(resize(e_v, 14)) - 952, 2);
                    m_a <= resize(e_v + 45, 23); m_b <= to_unsigned(4053, 12); m_acc <= (others => '0');
                when 3 =>
                    if e_ady > 140 then e_amn <= to_unsigned(140, 11); e_amx <= e_ady - 140;
                    else e_amn <= e_ady; e_amx <= (others => '0'); end if;
                when 15 =>                     -- R = min((v + 45) 4053 >> 10, 4095)
                    if m_acc(22) = '1' then n_rl <= (others => '1'); else n_rl <= m_acc(21 downto 10); end if;
                    m_a <= resize(e_amn, 23); m_b <= to_unsigned(2335, 12); m_acc <= (others => '0');
                when 29 =>
                    e_g1 <= m_acc;
                    m_a <= resize(e_amx, 23); m_b <= to_unsigned(1434, 12); m_acc <= (others => '0');
                when 43 =>                     -- g (Q2 px) = (min 2335 + max 1434) >> 10
                    n_gq <= resize(shift_right(e_g1 + m_acc, 10), 12);
                    m_a <= resize(e_ady, 23); m_b <= to_unsigned(1120, 12); m_acc <= (others => '0');
                when 57 =>
                    n_kdy <= m_acc(19 downto 10);
                    m_a <= resize(e_v, 23); m_b <= to_unsigned(386, 12); m_acc <= (others => '0');
                when 71 =>
                    e_t1 <= f_tri8(m_acc(15 downto 8));
                    m_a <= resize(e_v, 23); m_b <= to_unsigned(1236, 12); m_acc <= (others => '0');
                when 85 =>
                    e_t2 <= f_tri8(m_acc(15 downto 8));
                    m_a <= resize(e_v, 23); m_b <= to_unsigned(1725, 12); m_acc <= (others => '0');
                when 99 =>
                    e_fb <= m_acc(15 downto 8);
                    m_a <= resize(e_v, 23); m_b <= to_unsigned(2849, 12); m_acc <= (others => '0');
                when 113 =>
                    n_jl <= f_tri8(m_acc(15 downto 8));
                    m_a <= resize(e_v, 23); m_b <= to_unsigned(1092, 12); m_acc <= (others => '0');
                when 127 =>
                    n_jl2 <= f_tri8(m_acc(15 downto 8));
                    m_a <= resize(f_tri8(e_fb), 23); m_b <= to_unsigned(632, 12); m_acc <= (others => '0');
                when 141 =>                    -- strip ring = 807 - (tri(fb) 632 >> 8)
                    n_rs <= to_signed(807, 11) - signed(resize(m_acc(18 downto 8), 11));
                    m_a <= resize(e_t1, 23); m_b <= to_unsigned(100, 12); m_acc <= (others => '0');
                when 155 =>
                    e_sw1 <= resize(m_acc(15 downto 8), 9);
                    m_a <= resize(e_t2, 23); m_b <= to_unsigned(48, 12); m_acc <= (others => '0');
                when 169 =>                    -- strip edge (h) = 120 + t1 100 >> 8 + t2 48 >> 8
                    n_sw <= to_unsigned(120, 12) + resize(e_sw1, 12) + resize(m_acc(15 downto 8), 12);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): x in h units and the x-phase DDAs; the line's values
    -- load at the line start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_px <= (others => '0');
                b_rp <= c_rp0; b_jp <= c_jp0; b_kp <= c_kp0;
                l_rl <= n_rl; l_gq <= n_gq; l_kdy <= n_kdy; l_eb4 <= n_eb4; l_sw <= n_sw; l_rs <= n_rs;
                l_jl <= n_jl; l_jl2 <= n_jl2; l_par <= n_par;
            else
                if g_avid = '1' then b_x <= b_x + s_xs; end if;
                b_px <= b_px + 1;                      -- these keep stepping past the line end: stages 8 and 12 read
                b_rp <= b_rp + c_rps; b_jp <= b_jp + c_jps; b_kp <= b_kp + c_kps;   -- them 7 / 11 pixels ahead
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : signed(12 downto 0);
        variable v_tm  : unsigned(10 downto 0);
        variable v_t   : unsigned(10 downto 0);
        variable v_a   : unsigned(10 downto 0);
        variable v_sa  : signed(15 downto 0);
        variable v_r   : signed(13 downto 0);
        variable v_g   : signed(10 downto 0);
        variable v_du  : unsigned(9 downto 0);
        variable v_z   : signed(11 downto 0);
        variable v_ph  : signed(16 downto 0);
        variable v_lit : std_logic;
        variable v_lw  : unsigned(19 downto 0);
        variable v_j   : unsigned(8 downto 0);
        variable v_by  : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: hold left of the strip edge; ripple ROM address
            if b_x < l_sw then p1_ins <= '1'; p1_xe <= l_sw; else p1_ins <= '0'; p1_xe <= b_x; end if;
            r_rp_a <= b_rp(15 downto 8);
            -- 2: distance from the axis (h)
            v_d := signed(resize(p1_xe, 13)) - 2020;
            if v_d < 0 then v_d := -v_d; end if;
            p2_d <= unsigned(v_d(10 downto 0));
            -- 3: twin fold; ripple ROM data copy
            if s_twin = '1' then
                v_d := signed(resize(p2_d, 13)) - 1010;
                if v_d < 0 then v_d := -v_d; end if;
                p3_d <= unsigned(v_d(9 downto 0)) & '0';
            else
                p3_d <= p2_d;
            end if;
            p3_rip <= r_rp_q;
            -- 4: lattice and tongue products (operands split hi / lo); retrace offset
            p4_mdl <= resize(p3_d * c_kd(5 downto 0), 17);
            p4_mdh <= resize(p3_d * c_kd(10 downto 6), 16);
            p4_msl <= resize(p3_d * c_ks(5 downto 0), 17);
            p4_msh <= resize(p3_d * c_ks(11 downto 6), 17);
            p4_eq  <= l_eb4 - resize(p3_rip, 14);
            -- 5: sums; tongue ROM address; retrace class
            p5_traw <= resize(shift_right(resize(p4_mdl, 23) + shift_left(resize(p4_mdh, 23), 6), 10), 12);
            r_s_a   <= resize(shift_right(resize(p4_msl, 24) + shift_left(resize(p4_msh, 24), 6), 10), 8);
            if p4_eq < 0 then
                p5_ef <= "00"; p5_e <= (others => '0');
            elsif p4_eq < 512 then
                p5_ef <= "01"; p5_e <= resize(unsigned(to_signed(512, 14) - p4_eq), 10);
            else
                p5_ef <= "10"; p5_e <= (others => '0');
            end if;
            if p4_eq < -40 then p5_abv <= '1'; else p5_abv <= '0'; end if;
            -- 6: lattice fold; retrace ramp product
            v_tm := p5_traw(10 downto 0);
            if v_tm <= 1024 then p6_t <= v_tm;
            else p6_t <= resize(to_unsigned(2048, 12) - resize(v_tm, 12), 11); end if;
            p6_rp <= resize(p5_e & "000", 21);                      -- retrace: R = 8 (512 - eq)
            p6_ef <= p5_ef;
            -- 7: clamp t; triangle operands; tongue ROM data copy; pixel ramp R
            if p6_t > 1023 then v_t := to_unsigned(1023, 11); else v_t := p6_t; end if;
            p7_t <= v_t(9 downto 0);
            if v_t < 512 then p7_lo <= '1'; v_a := 512 - v_t; else p7_lo <= '0'; v_a := v_t - 512; end if;
            p7_a <= v_a(9 downto 0);
            p7_s <= r_s_q;
            case p6_ef is
                when "00"   => p7_r <= l_rl;
                when "01"   =>
                    if p6_rp(12) = '1' then p7_r <= (others => '1'); else p7_r <= p6_rp(11 downto 0); end if;
                when others => p7_r <= (others => '0');
            end case;
            -- 8: T (Q2 px), rd_x (px), zone; knot ROM address
            if p7_lo = '1' then
                p8_tq <= resize(shift_right(shift_left(resize(p7_a, 22), 11) + shift_left(resize(p7_a, 22), 10)
                                            + shift_left(resize(p7_a, 22), 8) + shift_left(resize(p7_a, 22), 6), 10), 11);
            else
                p8_tq <= resize(p7_a, 11);
            end if;
            p8_rdx <= resize(shift_right(shift_left(resize(p7_a, 20), 10) - shift_left(resize(p7_a, 20), 6)
                                         - shift_left(resize(p7_a, 20), 4), 10), 10);
            p8_s <= p7_s;
            -- Bayer offset by (line, pixel) parity; b_px is 7 pixels ahead here (opposite parity)
            if l_par = '0' then
                if b_px(0) = '0' then v_by := to_unsigned(32, 6); else v_by := to_unsigned(0, 6); end if;
            else
                if b_px(0) = '0' then v_by := to_unsigned(16, 6); else v_by := to_unsigned(48, 6); end if;
            end if;
            v_z := signed(resize(p7_t, 12)) - 380 + signed(resize(v_by, 12));
            if v_z < 0 then p8_z <= "00";
            elsif v_z >= 256 then p8_z <= "11";
            else p8_z <= unsigned(v_z(7 downto 6)); end if;
            p8_r <= p7_r;
            r_kt_a <= p7_t(9 downto 2);
            -- 9: knot ROM data (read straight from the EBR register at 10); notch ROM address; ring radius
            if resize(p8_rdx, 11) + resize(l_kdy, 11) > 255 then r_np_a <= (others => '1');
            else r_np_a <= resize(p8_rdx + l_kdy, 8); end if;
            p9_r <= signed(resize(p8_tq, 14)) + signed(resize(l_gq, 14)) + resize(p8_s, 14);
            p9_rr <= p8_r;
            -- 10: VCA branch select; ring and etch partial products
            if p9_rr < 2177 then
                p10_up <= '1';
                p10_base <= resize(signed('0' & r_kt_q(52 downto 40)), 15);
                p10_coef <= unsigned(r_kt_q(25 downto 12));
                p10_op   <= p9_rr;
            else
                p10_up <= '0';
                p10_base <= resize(signed(r_kt_q(39 downto 26)), 15);
                p10_coef <= "00" & unsigned(r_kt_q(11 downto 0));
                p10_op   <= p9_rr - 2177;
            end if;
            p10_ra <= resize(p9_r * signed('0' & c_krp(4 downto 0)), 20);
            p10_rb <= resize(p9_r * signed('0' & c_krp(9 downto 5)), 20);
            p10_la <= resize(plm3 * c_eg(5 downto 0), 14);         -- luma of the pixel 7 to the right
            p10_lb <= resize(plm3 * c_eg(11 downto 6), 14);
            -- 11: VCA partial products; ring / etch sums; notch ROM data copy
            p11_pl <= resize(p10_coef * p10_op(5 downto 0), 20);
            p11_ph <= resize(p10_coef * p10_op(11 downto 6), 20);
            p11_base <= p10_base;
            p11_np <= r_np_q;
            p11_ru <= unsigned(p10_ra(15 downto 0) + shift_left(p10_rb(15 downto 0), 5));
            v_lw := resize(p10_la, 20) + shift_left(resize(p10_lb, 20), 6);
            p11_lu <= v_lw(15 downto 0);
            p11_lp <= v_lw(19 downto 5);
            -- 12: VCA product; phase
            p12_prod <= resize(shift_right(resize(p11_pl, 26) + shift_left(resize(p11_ph, 26), 6), 12), 14);
            p12_base <= p11_base;
            p12_np <= resize(p11_np, 16) + signed(resize(p11_lp, 16));   -- notch + camera colour level
            p12_u <= p11_ru + p11_lu;
            v_j := resize(f_tri8(b_jp(15 downto 8) + l_jl), 9) + resize(f_tri8(b_kp(15 downto 8) - l_jl2), 9);
            p12_jit <= v_j(8 downto 3);                    -- stroke jitter of the pixel 11 ahead
            -- 13: s before the cap; ring operand; stroke phase; ember level
            p13_sa <= resize(p12_base, 16) - signed(resize(p12_prod, 16));
            p13_np <= p12_np;
            p13_g <= to_signed(512, 11) - signed(resize(p12_u(15 downto 6), 11));
            p13_f <= p12_u(15 downto 6);
            p13_uk <= p12_u(15 downto 6) + resize(p12_jit, 10) - 32;
            p13_m4 <= f_tri8(p12_u(15 downto 8))(7 downto 4);
            -- 14: s (capped, + notch); ring term; stroke distance
            if p13_sa > 5149 then v_sa := to_signed(5149, 16); else v_sa := p13_sa; end if;
            p14_s <= v_sa + p13_np;
            if p_ins(13) = '1' then
                p14_ring <= resize(l_rs, 12);
            else
                p14_ring <= resize(shift_right(shift_left(resize(p13_g, 22), 10) + shift_left(resize(p13_g, 22), 8)
                                               + shift_left(resize(p13_g, 22), 7) - shift_left(resize(p13_g, 22), 2), 10), 12);
            end if;
            if p13_uk >= 635 then v_du := p13_uk - 635; else v_du := 635 - p13_uk; end if;
            if v_du > 512 then v_du := (not v_du) + 1; end if;      -- 1024 - du
            p14_du <= v_du;
            -- 15: pre; stroke band
            p15_pre <= p14_s - 293 + resize(p14_ring, 16);
            if p14_du < resize(c_kw, 10) and p_sg(14) = '1' and p_ins(14) = '0' and p_abv(14) = '1' then
                p15_k1 <= '1';
            else
                p15_k1 <= '0';
            end if;
            -- 16 - 18: Blaze (pre - 0.12) G + 0.12 + lift
            p16_pd <= p15_pre - 702;
            p17_pd <= p16_pd;
            p17_ga <= resize(p16_pd * signed('0' & c_g(3 downto 0)), 23);
            p17_gb <= resize(p16_pd * signed('0' & c_g(7 downto 4)), 23);
            if p_ret(17) = '1' then         -- the bottom retrace keeps unit gain (no single-row bands at high Blaze)
                p18_pre <= resize(p17_pd, 17) + 702;
            elsif p_ins(17) = '1' then      -- so do the left-edge bars (lift only)
                p18_pre <= resize(p17_pd, 17) + 702 + resize(c_lift, 17);
            else
                p18_pre <= resize(shift_right(p17_ga + shift_left(p17_gb, 4), 6), 17) + 702 + resize(c_lift, 17);
            end if;
            -- 19: floor
            if p18_pre < -1750 then p19_pre <= to_signed(-1750, 17); else p19_pre <= p18_pre; end if;
            -- 20: palette index
            v_ph := p19_pre + signed(resize(c_off, 17));
            p20_idx <= unsigned(v_ph(12 downto 5));
            -- 21: palette address; lit
            r_pl_a <= pz(20) & p20_idx;
            if p20_idx >= 36 and p20_idx < 170 then p21_lit <= '1'; else p21_lit <= '0'; end if;
            if resize(p20_idx - c_klo, 9) < c_kwid then p21_klit <= '1'; else p21_klit <= '0'; end if;
            -- 22: palette data; final stroke key
            p22_key <= p_k14(21) and p21_klit;
            -- 23: fabric copy
            p23_q <= r_pl_q;
            p23_lit <= p21_lit;
            p24_lit <= p23_lit;
            -- 24: compose (black outside the generated avid)
            k_sr <= k_sr(14 downto 0) & p22_key;
            if s_gav_sr(22) = '0' then
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            elsif k_sr(0) = '1' then
                if s_frost = '1' then
                    s_y24 <= to_unsigned(C_RED_Y, 10); s_u24 <= to_unsigned(C_RED_V, 10); s_v24 <= to_unsigned(C_RED_U, 10);
                else
                    s_y24 <= to_unsigned(C_RED_Y, 10); s_u24 <= to_unsigned(C_RED_U, 10); s_v24 <= to_unsigned(C_RED_V, 10);
                end if;
            elsif k_sr(to_integer(c_shn)) = '1' then
                if s_frost = '1' then
                    s_y24 <= shift_right(unsigned(p23_q(31 downto 22)), 2) + C_OLV_Y;
                    s_u24 <= shift_right(unsigned(p23_q(11 downto 2)), 2) + C_OLV_V;
                    s_v24 <= shift_right(unsigned(p23_q(21 downto 12)), 2) + C_OLV_U;
                else
                    s_y24 <= shift_right(unsigned(p23_q(31 downto 22)), 2) + C_OLV_Y;
                    s_u24 <= shift_right(unsigned(p23_q(21 downto 12)), 2) + C_OLV_U;
                    s_v24 <= shift_right(unsigned(p23_q(11 downto 2)), 2) + C_OLV_V;
                end if;
            elsif s_emb = '1' and p23_lit = '0' and p_ins(23) = '0' then
                s_y24 <= to_unsigned(84, 10) + resize(pm4(23) * to_unsigned(6, 3), 10);
                if s_frost = '1' then
                    s_v24 <= to_unsigned(552, 10) + resize(pm4(23) * to_unsigned(10, 4), 10);
                    s_u24 <= to_unsigned(512, 10) - resize(pm4(23) * to_unsigned(2, 2), 10);
                else
                    s_u24 <= to_unsigned(552, 10) + resize(pm4(23) * to_unsigned(10, 4), 10);
                    s_v24 <= to_unsigned(512, 10) - resize(pm4(23) * to_unsigned(2, 2), 10);
                end if;
            else
                s_y24 <= unsigned(p23_q(31 downto 22));
                if s_frost = '1' then
                    s_u24 <= unsigned(p23_q(11 downto 2)); s_v24 <= unsigned(p23_q(21 downto 12));
                else
                    s_u24 <= unsigned(p23_q(21 downto 12)); s_v24 <= unsigned(p23_q(11 downto 2));
                end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(22);

            -- flag pipes
            p_ins(2) <= p1_ins;
            p_abv(6) <= p5_abv;
            if resize(p8_s, 11) > c_sth then p_sg(9) <= '1'; else p_sg(9) <= '0'; end if;
            for i in 10 to 31 loop p_sg(i) <= p_sg(i - 1); end loop;
            if p5_ef = "01" then p_ret(6) <= '1'; else p_ret(6) <= '0'; end if;
            for i in 7 to 31 loop p_ret(i) <= p_ret(i - 1); end loop;
            for i in 3 to 31 loop p_ins(i) <= p_ins(i - 1); end loop;
            for i in 7 to 31 loop p_abv(i) <= p_abv(i - 1); end loop;
            if p_ins(8) = '1' then pz(9) <= "00"; else pz(9) <= p8_z; end if;
            for i in 10 to 21 loop pz(i) <= pz(i - 1); end loop;
            p_k14(16) <= p15_k1;
            for i in 17 to 31 loop p_k14(i) <= p_k14(i - 1); end loop;
            pm4(14) <= p13_m4;
            for i in 15 to 24 loop pm4(i) <= pm4(i - 1); end loop;
            plm3 <= unsigned(s_lb_q);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the HD HDMI bar fix and the still's softness), luma
    -- 13/16 and the blanking gate.  The running 8-sums snap back to black once
    -- 8 blank samples are in, so a glitched sample can't offset the colours.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- rings: write this clock's compose output and input syncs; the box reads 8 back (o_q), the syncs
            -- C_LAT back (o_so): read address registered, EBR output register, one fabric register
            br(to_integer(o_wa)) <= std_logic_vector(s_y24) & std_logic_vector(s_u24) & std_logic_vector(s_v24) & "00";
            sr(to_integer(o_wa)) <= '1' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            o_wa <= o_wa + 1;
            o_ra <= o_wa - 5;
            o_sa <= o_wa - (C_LAT - 3);
            o_rq <= br(to_integer(o_ra));
            o_sq <= sr(to_integer(o_sa));
            o_q  <= o_rq;
            o_so <= o_sq;
            -- box: stage 25 sums, stage 26 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bay <= s_bay + s_y24 - unsigned(o_q(31 downto 22));
                s_bau <= s_bau + s_u24 - unsigned(o_q(21 downto 12));
                s_bav <= s_bav + s_v24 - unsigned(o_q(11 downto 2));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bay, 14) + s_bayd; s_y26 <= v_sc(13 downto 4);
            v_sc := resize(s_bau, 14) + s_baud; s_u26 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v26 <= v_sc(13 downto 4);
            -- 27: luma 13/16
            if s_y26 > 64 then v_dl := s_y26 - 64; else v_dl := (others => '0'); end if;
            s_y27 <= s_y26 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u27 <= s_u26; s_v27 <= s_v26;
            -- 28: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y28 <= s_y27; s_u28 <= s_u27; s_v28 <= s_v27;
            else
                s_y28 <= to_unsigned(64, 10); s_u28 <= to_unsigned(512, 10); s_v28 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= o_so(0);
    data_out.vsync_n <= o_so(1);
    data_out.field_n <= o_so(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture flamecrest;
