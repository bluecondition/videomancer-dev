-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cateye v0.1.1 -- Heirloom (phase 3, video clips), from the clip WIN_20240102_00_30_49 at 1.0 s.
--
-- v0.1.1: Osc feeds the lattice at 3x the camera's bend gain (per field, capped at 2x the camera's maximum), so the
-- walls and the ridge zigzag and the rings warp at the default Bend; K3 Rings tops out at 2x the rings (was 3x) and
-- above 640 steps through three darker ring palettes, so 1023 is a bold, high-contrast ringed eye.
--
-- The still: a pink-and-yellow chevron eye.  Magenta walls with dashed blue/orange lines bend into a ">" toward
-- the middle of the frame; a pale yellow cream valley holds a bullet-shaped eye (rounded left, pointed right) of
-- red / maroon rings with a cyan-or-orange rim, a kite-shaped pink pupil and a pink-red glow, hugged by a yellow
-- chevron ridge.  Horizontal stripes every 99 lines turn the line bands blue or orange.  In the clip Ron turns a
-- colourizer knob: the blue drains (cream -> yellow, magenta -> red), the eye grows rings and a black rim.
--
-- The patch, as fitted (cateye.py has every constant):
--   u     = x' - T(y) w(x')         a horizontal ramp zoomed by a vertical triangle (multiplier), w piecewise
--                                   linear: max(L, min(M, R)); T measured per row (apex on line 540.5)
--   walls = palettes L / R by the depth from the cream edges (390 / 1062 px), cream = palette D by u, the socket
--           dip deepens with the heat gain
--   eye   = r = max(ellipse(416 x 228), chevron) about (1002, 537) -> phi(r) (pupil, pupil line, moat, rim) ->
--           palette E (rings beyond the line), a soft-keyed glow outside
--   stripes = a 99.2-line square wave choosing the 'blue' or 'red' palette rows
--
--   K1  Chevron   the depth of the V-bend (walls, ridge, the eye's pointed side): straight (0) .. still .. 2.5x
--   K2  Eye       the eye's size: a bead (0) .. the still (512) .. 2.4x
--   K3  Rings     the moat depth: a plain red eye (0) .. the still .. 2x the rings in darker, bolder colours (1023)
--   K4  Speed     bipolar: rings stream out of (+) or into (-) the moat, the stripes roll
--   K5  Stripes   the stripe frequency: 4x coarser (0) .. 99 lines (512) .. 4x finer (1023)
--   K6  Bend      how far the camera (or the Osc) bends the walls and the rings
--   S7  Facing    Right (the still) / Left: the whole patch reversed, the eye looking the other way
--   S8  Stripes   Dashes (the still) / Bars: every other stripe's chroma inverted across the picture
--   S9  Pupil     Pink (the still) / Black: the kite pupil turns black inside its lit line
--   S10 Walls     Magenta (the still) / Black
--   S11 Source    Video / Osc
--   P12 HEAT      the colourizer gain + blue drain: pale dawn (0) .. the still (348) .. blazing yellow and red,
--                 a black-rimmed eye full of rings, dashes spilling across the socket (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide); a per-line engine kicked at each line start
-- (computes the NEXT line: T, the DDA starts and steps, the eye's vertical terms, the halo slope, the slit) on
-- its own serial multiplier; a beam of linear and second-order DDAs (turning at the centre for Mirror); a
-- pixel path (24 stages).  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning
-- (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16 as running 8-sums that resync to neutral in every blanking,
-- luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cateye_pkg.all;
use work.all;

architecture cateye of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks


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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the oscillator)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_pa   : unsigned(10 downto 0) := (others => '0');
    signal r_py, r_pu, r_pv : unsigned(7 downto 0) := (others => '0');
    signal r_sq_a : unsigned(9 downto 0) := (others => '0');
    signal r_sq_q : unsigned(7 downto 0) := (others => '0');
    signal r_ph_a : unsigned(7 downto 0) := (others => '0');
    signal r_ph_q : unsigned(15 downto 0) := (others => '0');
    signal r_t_a  : unsigned(8 downto 0) := (others => '0');
    signal r_t_q  : unsigned(15 downto 0) := (others => '0');

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
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(348 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(683, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg, s_vnegf : std_logic := '0';
    signal s_flip, s_bars, s_slit, s_blk, s_src : std_logic := '0';

    -- per-field constants
    signal c_chev : unsigned(8 downto 0) := to_unsigned(128, 9);     -- Q7
    signal c_q8   : unsigned(9 downto 0) := to_unsigned(256, 10);    -- 1 / eye size, Q8
    signal c_moat : unsigned(7 downto 0) := to_unsigned(32, 8);      -- Q5
    signal c_rb   : unsigned(7 downto 0) := to_unsigned(91, 8);      -- first ring-palette entry
    signal c_mm   : unsigned(11 downto 0) := to_unsigned(1024, 12);  -- stripe frequency multiplier, Q10
    signal c_g6   : unsigned(6 downto 0) := to_unsigned(64, 7);      -- heat gain G, Q6
    signal c_gm6  : unsigned(6 downto 0) := to_unsigned(64, 7);      -- ring gain Gm, Q6
    signal c_kap  : signed(7 downto 0) := (others => '0');            -- blue drain kappa, Q6
    signal c_gmm  : unsigned(7 downto 0) := to_unsigned(32, 8);       -- Gm * moat, Q5
    signal c_fF   : unsigned(19 downto 0) := to_unsigned(169125, 20); -- stripe cycles per virtual line, Q24
    signal c_fs   : unsigned(23 downto 0) := to_unsigned(169125, 24); -- per field line
    signal c_ph0  : unsigned(23 downto 0) := (others => '0');
    signal c_b8   : unsigned(7 downto 0) := to_unsigned(10, 8);       -- bend, Q6
    signal c_fls  : unsigned(9 downto 0) := (others => '0');          -- ring flow step, Q8 px per field
    signal c_rls  : unsigned(17 downto 0) := (others => '0');         -- stripe roll step, Q24 cycles per field
    signal c_fl   : unsigned(12 downto 0) := (others => '0');         -- ring flow phase, Q8 px mod 32
    signal c_rl   : unsigned(23 downto 0) := (others => '0');         -- stripe roll phase
    signal c_kq   : unsigned(11 downto 0) := to_unsigned(1260, 12);  -- ka * 2^20 per h
    signal c_kx   : unsigned(13 downto 0) := (others => '0');         -- kq * xs
    signal c_q0   : unsigned(33 downto 0) := (others => '0');         -- Qx at the line start, Q28
    signal c_k2   : unsigned(16 downto 0) := (others => '0');         -- second difference, Q28
    signal c_d0   : signed(26 downto 0) := (others => '0');           -- first difference at the start
    signal c_kc   : unsigned(15 downto 0) := to_unsigned(14624, 16); -- chevron gauge per h, Q22 (r 2^14)
    signal c_kcxs : signed(27 downto 0) := (others => '0');           -- the beam's chevron step (negative facing left)
    signal c_dth  : unsigned(7 downto 0) := to_unsigned(106, 8);      -- socket depth where the heat bars start (h)
    signal c_kmag : unsigned(5 downto 0) := (others => '0');          -- |kappa|
    signal c_kneg : std_logic := '0';
    signal c_cx0  : signed(27 downto 0) := (others => '0');
    signal c_kb   : unsigned(15 downto 0) := to_unsigned(18432, 16); -- ellipse gauge per line, Q16 (t 2^14)
    signal c_kcy  : unsigned(16 downto 0) := (others => '0');
    signal z_kl0, z_km0 : unsigned(21 downto 0) := (others => '0');
    signal c_kr0  : unsigned(21 downto 0) := (others => '0');
    signal c_kl, c_km, c_kr : unsigned(12 downto 0) := (others => '0');
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);     -- beam start x (h): x0 + 8 xs
    signal c_xp0  : unsigned(11 downto 0) := to_unsigned(16, 12);     -- mirrored start
    signal z_dx0  : unsigned(10 downto 0) := (others => '0');         -- |xp0 - 2004|
    signal z_dxn  : std_logic := '0';                                 -- xp0 < 2004
    signal z_d952 : unsigned(11 downto 0) := (others => '0');         -- |xp0 - 952|
    signal z_d9n  : std_logic := '0';                                 -- xp0 < 952

    -- field serial multiplier (36 x 18, 18 iterations, result at load + 19)
    signal z_k   : unsigned(4 downto 0) := (others => '0');
    signal z_ph  : unsigned(4 downto 0) := (others => '0');
    signal z_run : std_logic := '0';
    signal z_kl  : unsigned(4 downto 0) := (others => '0');          -- copy of z_k for the operand mux
    signal z_ld1, z_st1, z_st2 : std_logic := '0';
    attribute keep : boolean;
    attribute keep of z_k  : signal is true;
    attribute keep of z_kl : signal is true;
    signal z_s   : unsigned(17 downto 0) := (others => '0');
    signal z_a2  : unsigned(9 downto 0) := (others => '0');
    signal z_ap  : unsigned(9 downto 0) := (others => '0');
    signal z_lo2, z_lop, z_gdn : std_logic := '1';
    signal z_b2a : unsigned(8 downto 0) := (others => '0');
    signal z_bpa, z_bpb : unsigned(6 downto 0) := (others => '0');
    signal z_gd  : unsigned(6 downto 0) := (others => '0');
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

    -- line engine
    signal e_lc   : unsigned(8 downto 0) := to_unsigned(511, 9);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');       -- next line, Q8 virtual lines
    signal e_dye  : unsigned(18 downto 0) := (others => '0');       -- |y - 537|, Q8
    signal e_ph   : unsigned(23 downto 0) := (others => '0');
    signal e_tr   : unsigned(7 downto 0) := (others => '0');
    signal e_tlc, e_trc : unsigned(8 downto 0) := (others => '0');
    signal e_t14  : unsigned(17 downto 0) := (others => '0');
    signal e_qy   : unsigned(16 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(17 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_il, n_im, n_ir : signed(12 downto 0) := (others => '0');   -- ascending DDA steps, Q8 h
    signal n_al0, n_am0, n_ar0 : signed(22 downto 0) := (others => '0');
    signal n_qy   : unsigned(16 downto 0) := (others => '0');
    signal n_cy   : unsigned(19 downto 0) := (others => '0');
    signal n_s2   : unsigned(6 downto 0) := to_unsigned(94, 7);
    signal n_blue : std_logic := '0';

    -- per-line values in use (loaded at the line's start)
    signal l_il, l_im, l_ir : signed(12 downto 0) := (others => '0');
    signal l_qy   : unsigned(16 downto 0) := (others => '0');
    signal l_cy   : unsigned(19 downto 0) := (others => '0');
    signal l_s2   : unsigned(6 downto 0) := to_unsigned(94, 7);
    signal l_blue : std_logic := '0';

    -- beam
    signal b_xp   : unsigned(11 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_al, b_am, b_ar : signed(22 downto 0) := (others => '0');
    signal b_qx   : signed(35 downto 0) := (others => '0');
    signal b_dx   : signed(27 downto 0) := (others => '0');
    signal b_cx   : signed(27 downto 0) := (others => '0');

    -- pixel path (qK = stage K after the beam values)
    signal q1_m, q1_al : signed(15 downto 0) := (others => '0');
    signal q1_xp, q2_xp : unsigned(11 downto 0) := (others => '0');
    signal q1_qx  : unsigned(16 downto 0) := (others => '0');
    signal q1_c   : signed(20 downto 0) := (others => '0');
    signal q2_e2  : unsigned(17 downto 0) := (others => '0');
    signal q2_tw  : signed(15 downto 0) := (others => '0');
    signal q2_c   : unsigned(7 downto 0) := (others => '0');
    signal q3_u0, q4_u0, q5_u0 : signed(13 downto 0) := (others => '0');
    signal q3_c, q4_c, q5_c : unsigned(7 downto 0) := (others => '0');
    signal q3_lm  : unsigned(7 downto 0) := (others => '0');
    signal q4_pl, q4_ph : unsigned(12 downto 0) := (others => '0');
    signal q5_re  : unsigned(7 downto 0) := (others => '0');
    signal q5_p   : unsigned(16 downto 0) := (others => '0');
    signal q6_u   : signed(13 downto 0) := (others => '0');
    signal q6_br  : unsigned(7 downto 0) := (others => '0');
    signal q6_r   : unsigned(7 downto 0) := (others => '0');
    signal q7_lp, q7_rp : std_logic := '0';
    signal q7_dl, q7_dr, q7_dip : unsigned(9 downto 0) := (others => '0');
    signal q7_di  : unsigned(7 downto 0) := (others => '0');
    signal q7_rb, q8_rb, q9_rb, q10_rb, q11_rb : unsigned(7 downto 0) := (others => '0');
    signal q9_ph  : unsigned(15 downto 0) := (others => '0');
    type t_u2 is array (natural range <>) of unsigned(1 downto 0);
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    signal q_bf   : t_u2(8 to 13) := (others => "00");      -- background family of stage K
    signal q_bi   : t_u8(8 to 13) := (others => (others => '0'));
    signal q_bd   : t_u10a(8 to 13) := (others => (others => '0'));
    signal q_bdip : std_logic_vector(8 to 13) := (others => '0');
    signal q10_b0, q10_b1 : unsigned(10 downto 0) := (others => '0');
    signal q10_m0, q10_m1 : unsigned(11 downto 0) := (others => '0');
    signal q10_h0, q10_h1 : unsigned(10 downto 0) := (others => '0');
    signal q11_ea, q11_eb : unsigned(9 downto 0) := (others => '0');
    signal q11_hp : unsigned(14 downto 0) := (others => '0');
    signal q12_de : unsigned(10 downto 0) := (others => '0');
    signal q12_al : unsigned(6 downto 0) := (others => '0');
    signal q12_rb : unsigned(7 downto 0) := (others => '0');
    signal q13_in, q13_ho, q13_pb : std_logic := '0';
    signal q13_de, q13_der : unsigned(10 downto 0) := (others => '0');
    signal q13_hi : unsigned(7 downto 0) := (others => '0');
    signal q14_fam, q15_fam, q16_fam : unsigned(1 downto 0) := (others => '0');
    signal q14_idx, q15_idx, q16_idx : unsigned(7 downto 0) := (others => '0');
    signal q14_core, q15_core, q16_core : std_logic := '0';
    signal q14_blk, q15_blk, q16_blk : std_logic := '0';                          -- black-wall candidate
    signal q14_ln, q15_ln, q16_ln : std_logic := '0';                             -- a line band (spared by the drain)
    signal q16_y, q16_u, q16_v : unsigned(7 downto 0) := (others => '0');
    signal q17_y, q17_u, q17_v : unsigned(9 downto 0) := (others => '0');
    signal q17_bp : unsigned(9 downto 0) := (others => '0');
    signal q17_ln : std_logic := '0';
    signal q18_y, q18_u, q18_v : unsigned(9 downto 0) := (others => '0');
    signal q18_kp : unsigned(15 downto 0) := (others => '0');
    signal q19_y, q19_u : unsigned(9 downto 0) := (others => '0');
    signal q19_v  : signed(11 downto 0) := (others => '0');
    signal q20_y, q20_u, q20_v : unsigned(9 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cu, s_cv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_wy : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
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

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            r_py <= C_CE_PY(to_integer(r_pa));
            r_pu <= C_CE_PU(to_integer(r_pa));
            r_pv <= C_CE_PV(to_integer(r_pa));
        end if;
    end process p_pal;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_sq_q <= C_CE_SQ(to_integer(r_sq_a));
            r_ph_q <= C_CE_PHI(to_integer(r_ph_a));
            r_t_q  <= C_CE_T(to_integer(r_t_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_t  : unsigned(19 downto 0);
        variable v_x  : unsigned(19 downto 0);
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
            -- speed: bipolar, +-8 deadband
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
                s_flip <= registers_in(6)(0);
                s_bars <= registers_in(6)(1);
                s_slit <= registers_in(6)(2);
                s_blk  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
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

            -- serial products on a slot counter (from zc 62): slot k loads at phase 1 (operands muxed straight
            -- into the multiplier by the slot number), 18 iterations, stores at phase 21, helpers at phase 22.
            if s_zone = '0' then
                z_run <= '0';
            elsif s_zc = 62 then
                z_run <= '1'; z_ph <= (others => '0'); z_k <= (others => '0'); z_kl <= (others => '0');
            elsif z_run = '1' then
                if z_ph = 23 then
                    z_ph <= (others => '0');
                    if z_k = 25 then z_run <= '0'; end if;
                    z_k <= z_k + 1; z_kl <= z_kl + 1;
                else
                    z_ph <= z_ph + 1;
                end if;
            end if;
            -- strobes a clock ahead (no phase compare on the load / store paths)
            if z_run = '1' and z_ph = 0 then z_ld1 <= '1'; else z_ld1 <= '0'; end if;
            if z_run = '1' and z_ph = 20 then z_st1 <= '1'; else z_st1 <= '0'; end if;
            z_st2 <= z_st1;
            if z_st1 = '1' then
                case to_integer(z_k) is
                    when 0 =>
                        if z_lo2 = '1' then c_q8 <= 608 - resize(m_acc(17 downto 4), 10);
                        else c_q8 <= 256 - resize(m_acc(17 downto 8), 10); end if;
                    when 1 =>
                        if z_lop = '1' then v_t := m_acc(19 downto 0) + 512; c_g6 <= 48 + resize(v_t(16 downto 10), 7);
                        else c_g6 <= 64 + resize(m_acc(16 downto 10), 7); end if;
                    when 2 =>
                        if z_lop = '1' then v_t := m_acc(19 downto 0) + 512; c_kap <= signed(resize(v_t(16 downto 10), 8)) - 29;
                        else c_kap <= signed(resize(m_acc(16 downto 10), 8)); end if;
                    when 3 =>
                        if z_lop = '1' then c_dth <= to_unsigned(120, 8);
                        else c_dth <= 106 - resize(m_acc(16 downto 10) - shift_right(m_acc(16 downto 10), 2), 8); end if;
                    when 4 =>
                        if z_gdn = '1' then c_gm6 <= 64 - resize(m_acc(12 downto 6), 7);
                        else c_gm6 <= 64 + resize(m_acc(12 downto 6), 7); end if;
                    when 5 =>
                        c_gmm <= m_acc(13 downto 6);
                    when 6 =>
                        c_fF  <= m_acc(29 downto 10);
                    when 7 =>
                        c_fs  <= m_acc(31 downto 8);
                    when 8 =>
                        c_ph0 <= c_rl - m_acc(23 downto 0);
                    when 9 =>
                        -- bend gain; the Osc lattice at 3x the camera's (capped at 255)
                        if s_src = '1' then
                            v_t := resize(m_acc(20 downto 13), 20) + shift_left(resize(m_acc(20 downto 13), 20), 1);
                            if v_t > 255 then c_b8 <= (others => '1'); else c_b8 <= v_t(7 downto 0); end if;
                        else
                            c_b8 <= m_acc(20 downto 13);
                        end if;
                    when 10 =>
                        z_s   <= m_acc(17 downto 0);
                    when 11 =>
                        c_fls <= m_acc(25 downto 16);
                    when 12 =>
                        c_rls <= m_acc(17 downto 0); s_vnegf <= s_vneg;
                    when 13 =>
                        c_kq  <= m_acc(17 downto 6);
                    when 14 =>
                        z_s   <= resize(m_acc(22 downto 6), 18);
                    when 15 =>
                        c_q0  <= m_acc(33 downto 0);
                    when 16 =>
                        c_k2  <= m_acc(27 downto 11);
                    when 17 =>
                        c_d0  <= signed(resize(shift_right(c_k2, 1), 27)) - signed(resize(m_acc(30 downto 5), 27));   -- D0 = K2 / 2 - |da|
                    when 18 =>
                        c_kc  <= m_acc(18 downto 3);
                    when 19 =>
                        if z_dxn = '1' then c_cx0 <= -signed(resize(m_acc(26 downto 0), 28));
                        else c_cx0 <= signed(resize(m_acc(26 downto 0), 28)); end if;
                    when 20 =>
                        c_kb  <= m_acc(15 downto 0);
                    when 21 =>
                        z_s   <= m_acc(17 downto 0);
                    when 22 =>
                        c_kcy <= m_acc(26 downto 10);
                    when 23 =>
                        z_kl0 <= m_acc(21 downto 0);
                    when 24 =>
                        z_km0 <= m_acc(21 downto 0);
                    when 25 =>
                        c_kr0 <= m_acc(21 downto 0) + 403866;
                    when others => null;
                end case;
            end if;
            if z_st2 = '1' then
                case to_integer(z_k) is
                    when 1 =>
                        if c_g6 < 64 then z_gd <= resize(64 - c_g6, 7); z_gdn <= '1';
                        else z_gd <= resize(c_g6 - 64, 7); z_gdn <= '0'; end if;
                    when 2 =>
                        if c_kap < 0 then c_kneg <= '1'; v_t := resize(unsigned(-c_kap), 20);
                        else c_kneg <= '0'; v_t := resize(unsigned(c_kap), 20); end if;
                        if v_t > 63 then c_kmag <= to_unsigned(63, 6); else c_kmag <= v_t(5 downto 0); end if;
                    when 13 =>
                        case to_integer(s_xs) is
                            when 4      => c_kx <= resize(shift_left(resize(c_kq, 14), 2), 14);
                            when 3      => c_kx <= resize(shift_left(resize(c_kq, 14), 1) + resize(c_kq, 14), 14);
                            when others => c_kx <= resize(shift_left(resize(c_kq, 14), 1), 14);
                        end case;
                    when 18 =>
                        case to_integer(s_xs) is
                            when 4      => v_x := resize(shift_left(resize(c_kc, 20), 2), 20);
                            when 3      => v_x := resize(shift_left(resize(c_kc, 20), 1) + resize(c_kc, 20), 20);
                            when others => v_x := resize(shift_left(resize(c_kc, 20), 1), 20);
                        end case;
                        if s_flip = '1' then c_kcxs <= -signed(resize(v_x, 28)); else c_kcxs <= signed(resize(v_x, 28)); end if;
                    when others => null;
                end case;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps); beam origin
                    when 20 =>
                        if s_vnegf = '1' then c_fl <= c_fl - resize(c_fls, 13); c_rl <= c_rl + resize(c_rls, 24);
                        else c_fl <= c_fl + resize(c_fls, 13); c_rl <= c_rl - resize(c_rls, 24); end if;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                    when 21 =>
                        if s_flip = '1' then c_xp0 <= 3840 - c_xb0; else c_xp0 <= c_xb0; end if;
                        case to_integer(s_xs) is
                            when 4      => c_kl <= to_unsigned(5014, 13); c_km <= to_unsigned(5137, 13); c_kr <= to_unsigned(1884, 13);
                            when 3      => c_kl <= to_unsigned(3760, 13); c_km <= to_unsigned(3853, 13); c_kr <= to_unsigned(1413, 13);
                            when others => c_kl <= to_unsigned(2507, 13); c_km <= to_unsigned(2569, 13); c_kr <= to_unsigned(942, 13);
                        end case;
                    when 22 =>
                        if c_xp0 < 2004 then z_dx0 <= resize(2004 - c_xp0, 11); z_dxn <= '1';
                        else z_dx0 <= resize(c_xp0 - 2004, 11); z_dxn <= '0'; end if;
                        if c_xp0 < 952 then z_d952 <= 952 - c_xp0; z_d9n <= '1';
                        else z_d952 <= c_xp0 - 952; z_d9n <= '0'; end if;
                    -- control curves: chevron and moat in fabric, the two-slope operands for the products
                    when 23 =>
                        v_k := s_g1(12 downto 3);
                        if v_k <= 512 then c_chev <= resize(v_k(9 downto 2), 9);
                        else c_chev <= 128 + resize(shift_right(resize(v_k - 512, 12) + shift_left(resize(v_k - 512, 12), 1), 3), 9); end if;
                        v_k := s_g3(12 downto 3);
                        if v_k <= 512 then c_moat <= resize(v_k(9 downto 4), 8);
                        else c_moat <= 32 + resize(shift_right(v_k - 512, 4), 8); end if;
                        -- ring palette: the still's red / maroon up to 640, then three darker cycles
                        if v_k <= 640 then c_rb <= to_unsigned(91, 8);
                        elsif v_k <= 768 then c_rb <= to_unsigned(192, 8);
                        elsif v_k <= 896 then c_rb <= to_unsigned(208, 8);
                        else c_rb <= to_unsigned(224, 8); end if;
                    when 24 =>
                        v_k := s_g2(12 downto 3);
                        if v_k <= 512 then z_a2 <= v_k; z_lo2 <= '1'; z_b2a <= to_unsigned(11, 9);
                        else z_a2 <= v_k - 512; z_lo2 <= '0'; z_b2a <= to_unsigned(75, 9); end if;
                        v_k := s_gp(12 downto 3);
                        if v_k <= 348 then z_ap <= v_k; z_lop <= '1'; z_bpa <= to_unsigned(47, 7); z_bpb <= to_unsigned(85, 7);
                        else z_ap <= v_k - 348; z_lop <= '0'; z_bpa <= to_unsigned(71, 7); z_bpb <= to_unsigned(97, 7); end if;
                    when 25 =>
                        v_k := s_g5(12 downto 3);
                        if v_k <= 512 then c_mm <= 256 + resize(shift_right(resize(v_k, 12) + shift_left(resize(v_k, 12), 1), 1), 12);
                        else c_mm <= 1024 + resize(shift_left(resize(v_k - 512, 12), 2) + shift_left(resize(v_k - 512, 12), 1), 12); end if;
                    when others => null;
                end case;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
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
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on its own serial multiplier (18 iterations, slot
    -- every 20 clocks, result read at load + 19):
    --   T = TL, TR (ROM by |y - 540.5|) * chevron;  the three linear DDA steps and starts
    --   t = |y - 537| kb;  Qy = t^2;  Cy = |y - 537| kcy;  halo slope;  slit half-width
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(19 downto 0);
        variable v_n  : signed(19 downto 0);
        variable v_t  : unsigned(17 downto 0);
        variable v_s2 : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1500 then e_zk <= '1'; end if;     -- after the field constants
            if s_zone = '1' and s_zc = 1490 then e_ph <= c_ph0; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(17 downto 1);
            -- field products (vblank zone, the line engine is idle there): slot k's operands at phase 1
            if z_ld1 = '1' then
                m_acc <= (others => '0');
                case to_integer(z_kl) is
                    when 0  => m_a <= resize(z_a2, 36); m_b <= resize(z_b2a, 18);   -- eye size -> 1 / size
                    when 1  => m_a <= resize(z_ap, 36); m_b <= resize(z_bpa, 18);   -- heat gain
                    when 2  => m_a <= resize(z_ap, 36); m_b <= resize(z_bpb, 18);   -- blue drain
                    when 3  => m_a <= resize(z_ap, 36); m_b <= to_unsigned(158, 18);   -- heat bars threshold
                    when 4  => m_a <= resize(z_gd, 36); m_b <= to_unsigned(38, 18);   -- ring gain
                    when 5  => m_a <= resize(c_gm6, 36); m_b <= resize(c_moat, 18);   -- Gm moat
                    when 6  => m_a <= to_unsigned(169125, 36); m_b <= resize(c_mm, 18);   -- F
                    when 7  => m_a <= resize(c_fF, 36); m_b <= resize(s_ys, 18);   -- F per line
                    when 8  => m_a <= resize(c_fF, 36); m_b <= to_unsigned(82, 18);   -- phase at line 0
                    when 9  => m_a <= resize(s_g6(12 downto 3), 36); m_b <= resize(s_g6(12 downto 3), 18);   -- bend^2
                    when 10 => m_a <= resize(s_vm, 36); m_b <= resize(s_vm, 18);   -- speed^2
                    when 11 => m_a <= resize(z_s, 36); m_b <= to_unsigned(265, 18);   -- flow step
                    when 12 => m_a <= resize(c_fls, 36); m_b <= to_unsigned(158, 18);   -- roll step
                    when 13 => m_a <= resize(c_q8, 36); m_b <= to_unsigned(315, 18);   -- kq
                    when 14 => m_a <= resize(z_dx0, 36); m_b <= resize(c_kq, 18);   -- |A|
                    when 15 => m_a <= resize(z_s, 36); m_b <= z_s;   -- A^2
                    when 16 => m_a <= resize(c_kx, 36); m_b <= resize(c_kx, 18);   -- K2
                    when 17 => m_a <= resize(c_kx, 36); m_b <= z_s;   -- da
                    when 18 => m_a <= resize(c_q8, 36); m_b <= to_unsigned(457, 18);   -- kc
                    when 19 => m_a <= resize(z_dx0, 36); m_b <= resize(c_kc, 18);   -- |Cx0|
                    when 20 => m_a <= resize(c_q8, 36); m_b <= to_unsigned(72, 18);   -- kb
                    when 21 => m_a <= resize(c_chev, 36); m_b <= resize(c_q8, 18);   -- chev q
                    when 22 => m_a <= resize(z_s, 36); m_b <= to_unsigned(581, 18);   -- kcy
                    when 23 => m_a <= resize(z_d952, 36); m_b <= to_unsigned(1253, 18);   -- |kL0|
                    when 24 => m_a <= resize(z_d952, 36); m_b <= to_unsigned(1284, 18);   -- |kM0|
                    when others => m_a <= resize(c_xp0, 36); m_b <= to_unsigned(471, 18);   -- kR0
                end case;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    v_d := signed(resize(e_vy, 20)) - 138368;              -- 540.5 * 256
                    v_n := -v_d;                                         -- negate at full width, then slice
                    if v_d < 0 then v_d := v_n; end if;
                    if v_d(18) = '1' then r_t_a <= (others => '1');
                    else r_t_a <= unsigned(v_d(17 downto 9)); end if;
                    v_d := signed(resize(e_vy, 20)) - 137472;              -- 537 * 256
                    v_n := -v_d;
                    if v_d < 0 then e_dye <= unsigned(v_n(18 downto 0)); else e_dye <= unsigned(v_d(18 downto 0)); end if;
                    n_blue <= not e_ph(23);
                when 2 =>
                    null;
                when 4 =>
                    e_tr <= r_t_q(7 downto 0);
                    m_a <= resize(c_chev, 36); m_b <= resize(r_t_q(15 downto 8), 18); m_acc <= (others => '0');
                when 23 =>
                    e_tlc <= m_acc(15 downto 7);
                when 24 =>
                    m_a <= resize(c_chev, 36); m_b <= resize(e_tr, 18); m_acc <= (others => '0');
                when 43 =>
                    e_trc <= m_acc(15 downto 7);
                when 44 =>
                    m_a <= resize(c_kl, 36); m_b <= resize(e_tlc, 18); m_acc <= (others => '0');
                when 63 =>
                    if s_flip = '1' then n_il <= signed(resize(m_acc(23 downto 11), 13));
                    else n_il <= -signed(resize(m_acc(23 downto 11), 13)); end if;
                when 64 =>
                    m_a <= resize(c_km, 36); m_b <= resize(e_trc, 18); m_acc <= (others => '0');
                when 83 =>
                    if s_flip = '1' then n_im <= -signed(resize(m_acc(23 downto 11), 13));
                    else n_im <= signed(resize(m_acc(23 downto 11), 13)); end if;
                when 84 =>
                    m_a <= resize(c_kr, 36); m_b <= resize(e_trc, 18); m_acc <= (others => '0');
                when 103 =>
                    if s_flip = '1' then n_ir <= -signed(resize(m_acc(23 downto 11), 13));
                    else n_ir <= signed(resize(m_acc(23 downto 11), 13)); end if;
                when 104 =>
                    m_a <= resize(z_kl0, 36); m_b <= resize(e_tlc, 18); m_acc <= (others => '0');
                when 123 =>                                              -- L(x0) = 0.306 (952 - x0): negative when mirrored
                    if z_d9n = '1' then n_al0 <= signed(resize(m_acc(32 downto 11), 23));
                    else n_al0 <= -signed(resize(m_acc(32 downto 11), 23)); end if;
                when 124 =>
                    m_a <= resize(z_km0, 36); m_b <= resize(e_trc, 18); m_acc <= (others => '0');
                when 143 =>
                    if z_d9n = '1' then n_am0 <= -signed(resize(m_acc(32 downto 11), 23));
                    else n_am0 <= signed(resize(m_acc(32 downto 11), 23)); end if;
                when 144 =>
                    m_a <= resize(c_kr0, 36); m_b <= resize(e_trc, 18); m_acc <= (others => '0');
                when 163 =>
                    n_ar0 <= signed(resize(m_acc(32 downto 11), 23));
                when 164 =>
                    m_a <= resize(e_dye, 36); m_b <= resize(c_kb, 18); m_acc <= (others => '0');
                when 183 =>
                    e_t14 <= m_acc(33 downto 16);
                when 184 =>
                    if e_t14(17 downto 16) /= "00" then v_t := to_unsigned(65535, 18); else v_t := e_t14; end if;
                    m_a <= resize(v_t, 36); m_b <= v_t; m_acc <= (others => '0');
                    v_s2 := 94 - signed(resize(shift_right(e_t14, 8), 9));
                    if e_t14(17 downto 14) /= "0000" or v_s2 < 46 then n_s2 <= to_unsigned(46, 7);
                    else n_s2 <= unsigned(v_s2(6 downto 0)); end if;
                when 203 =>
                    if m_acc(35 downto 30) /= "000000" then e_qy <= to_unsigned(65536, 17);
                    else e_qy <= resize(m_acc(29 downto 14), 17); end if;
                when 204 =>
                    n_qy <= e_qy;
                    m_a <= resize(e_dye, 36); m_b <= resize(c_kcy, 18); m_acc <= (others => '0');
                when 223 =>
                    n_cy <= m_acc(35 downto 16);
                when 224 =>
                    e_ph <= e_ph + c_fs;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: x' (h; Facing Left runs it down from 3840 - x0) and its DDAs: three linear ramps T L, T M, T R,
    -- the ellipse's (dx ka)^2 by a second-order DDA and the chevron's dx kc
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_xp <= c_xp0;
                b_px <= (others => '0');
                b_al <= n_al0; b_am <= n_am0; b_ar <= n_ar0;
                b_qx <= signed(resize(c_q0, 36));
                b_dx <= resize(c_d0, 28);
                b_cx <= c_cx0;
                l_il <= n_il; l_im <= n_im; l_ir <= n_ir;
                l_qy <= n_qy; l_cy <= n_cy; l_s2 <= n_s2;
                l_blue <= n_blue;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if s_flip = '1' then b_xp <= b_xp - s_xs; else b_xp <= b_xp + s_xs; end if;
                b_al <= b_al + l_il; b_am <= b_am + l_im; b_ar <= b_ar + l_ir;
                b_cx <= b_cx + c_kcxs;
                b_qx <= b_qx + b_dx;
                b_dx <= b_dx + signed(resize(c_k2, 28));
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_c   : signed(20 downto 0);
        variable v_r   : unsigned(7 downto 0);
        variable v_u   : signed(13 downto 0);
        variable v_w   : signed(13 downto 0);
        variable v_gi  : unsigned(15 downto 0);
        variable v_ie  : unsigned(7 downto 0);
        variable v_dp  : unsigned(9 downto 0);
        variable v_hd  : unsigned(9 downto 0);
        variable v_dd  : signed(10 downto 0);
        variable v_bp  : signed(12 downto 0);
        variable v_bk  : std_logic;
        variable v_vv  : signed(11 downto 0);
        variable v_dv  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: ellipse term clamped to [0, 2^30]; chevron term; min of the two right-side ramps
            if b_qx < 0 then q1_qx <= (others => '0');
            elsif b_qx(35 downto 30) /= "000000" then q1_qx <= to_unsigned(65536, 17);
            else q1_qx <= resize(unsigned(b_qx(29 downto 14)), 17); end if;
            q1_c <= resize(shift_right(b_cx, 8), 21) + signed(resize(l_cy, 21));
            if b_am(22 downto 7) < b_ar(22 downto 7) then q1_m <= b_am(22 downto 7); else q1_m <= b_ar(22 downto 7); end if;
            q1_al <= b_al(22 downto 7); q1_xp <= b_xp;
            -- 2: E^2, T w = max(L, min(M, R)), chevron r * 128
            q2_e2 <= resize(q1_qx, 18) + resize(l_qy, 18);
            if q1_al > q1_m then q2_tw <= q1_al; else q2_tw <= q1_m; end if;
            v_c := shift_right(q1_c, 7);
            if v_c < 0 then q2_c <= (others => '0');
            elsif v_c > 255 then q2_c <= (others => '1');
            else q2_c <= unsigned(v_c(7 downto 0)); end if;
            q2_xp <= q1_xp;
            -- 3: sqrt address; u0 = x' - T w; luma; slit distance
            if q2_e2(17 downto 16) /= "00" then r_sq_a <= (others => '1'); else r_sq_a <= q2_e2(15 downto 6); end if;
            q3_u0 <= signed(resize(q2_xp, 14)) - resize(shift_right(q2_tw, 1), 14);
            q3_c <= q2_c;
            q3_lm <= unsigned(s_lb_q);
            -- 4: luma x bend (nibble partials)
            q4_pl <= resize(q3_lm(3 downto 0) * c_b8, 13);
            q4_ph <= resize(q3_lm(7 downto 4) * c_b8, 13);
            q4_u0 <= q3_u0; q4_c <= q3_c;
            -- 5: bend sum, sqrt copy
            q5_p <= resize(q4_pl, 17) + shift_left(resize(q4_ph, 17), 4);
            q5_re <= r_sq_q; q5_u0 <= q4_u0; q5_c <= q4_c;
            -- 6: u = u0 - bend (h); r = max(ellipse, chevron); bend for the eye
            q6_u <= q5_u0 - signed(resize(shift_right(q5_p, 5), 14));
            if q5_p(16) = '1' then q6_br <= (others => '1'); else q6_br <= q5_p(15 downto 8); end if;
            if q5_re > q5_c then q6_r <= q5_re; else q6_r <= q5_c; end if;
            -- 7: wall depths (h, clamped), socket dip = min(rise, fall) >= 0, cream index; bent r -> phi address
            v_u := 780 - q6_u;
            if v_u > 0 then q7_lp <= '1'; else q7_lp <= '0'; end if;
            if v_u > 1023 then q7_dl <= (others => '1'); else q7_dl <= unsigned(v_u(9 downto 0)); end if;
            v_u := q6_u - 2124;
            if v_u > 0 then q7_rp <= '1'; else q7_rp <= '0'; end if;
            if v_u > 1023 then q7_dr <= (others => '1'); else q7_dr <= unsigned(v_u(9 downto 0)); end if;
            v_u := shift_right(q6_u - 1240, 3);
            v_w := 2028 - q6_u;
            if v_w < v_u then v_u := v_w; end if;
            if v_u < 0 then q7_dip <= (others => '0'); else q7_dip <= unsigned(v_u(9 downto 0)); end if;
            v_u := q6_u - 780;
            if v_u < 0 then q7_di <= (others => '0');
            elsif v_u > 2047 then q7_di <= (others => '1');
            else q7_di <= unsigned(v_u(10 downto 3)); end if;
            if q6_r > q6_br then v_r := q6_r - q6_br; else v_r := (others => '0'); end if;
            q7_rb <= v_r; r_ph_a <= v_r;
            -- 8: background family / index / depth (a deep socket turns to the wall palette: the heat bars)
            q_bdip(8) <= '0';
            if q7_lp = '1' then
                q_bf(8) <= "00"; q_bd(8) <= q7_dl;
                if q7_dl(9 downto 2) = "11111111" then q_bi(8) <= (others => '1'); else q_bi(8) <= q7_dl(9 downto 2); end if;
            elsif q7_rp = '1' then
                q_bf(8) <= "01"; q_bd(8) <= q7_dr; q_bi(8) <= q7_dr(9 downto 2);
            elsif q7_dip > resize(c_dth, 10) then
                q_bf(8) <= "00"; q_bd(8) <= q7_dip; q_bdip(8) <= '1';
                v_dp := q7_dip - resize(c_dth, 10) + 30;
                if v_dp > 255 then q_bi(8) <= (others => '1'); else q_bi(8) <= v_dp(7 downto 0); end if;
            else
                q_bf(8) <= "10"; q_bd(8) <= q7_dip; q_bi(8) <= q7_di;
            end if;
            for i in 9 to 13 loop
                q_bf(i) <= q_bf(i - 1); q_bi(i) <= q_bi(i - 1); q_bd(i) <= q_bd(i - 1); q_bdip(i) <= q_bdip(i - 1);
            end loop;
            q8_rb <= q7_rb;
            q9_ph <= r_ph_q; q9_rb <= q8_rb;
            -- 10: eye partial products (G base, Gm moat bump, halo slope)
            q10_b0 <= resize(q9_ph(11 downto 8) * c_g6, 11);
            q10_b1 <= resize(q9_ph(15 downto 12) * c_g6, 11);
            q10_m0 <= resize(q9_ph(3 downto 0) * c_gmm, 12);
            q10_m1 <= resize(q9_ph(7 downto 4) * c_gmm, 12);
            if q9_rb > 122 then v_ie := q9_rb - 122; else v_ie := (others => '0'); end if;
            q10_h0 <= resize(v_ie(3 downto 0) * l_s2, 11);
            q10_h1 <= resize(v_ie(7 downto 4) * l_s2, 11);
            q10_rb <= q9_rb;
            -- 11: eye sums
            v_gi := resize(q10_b0, 16) + shift_left(resize(q10_b1, 16), 4);
            q11_ea <= resize(shift_right(v_gi, 6), 10);
            v_gi := resize(q10_m0, 16) + shift_left(resize(q10_m1, 16), 4);
            q11_eb <= resize(shift_right(v_gi, 5), 10);
            q11_hp <= resize(q10_h0, 15) + shift_left(resize(q10_h1, 15), 4);
            q11_rb <= q10_rb;
            -- 12: d_E; halo alpha
            q12_de <= resize(q11_ea, 11) + resize(q11_eb, 11);
            if q11_rb < 122 then q12_al <= to_unsigned(64, 7);
            elsif q11_hp(14 downto 5) >= 64 then q12_al <= (others => '0');
            else q12_al <= 64 - resize(q11_hp(10 downto 5), 7); end if;
            q12_rb <= q11_rb;
            -- 13: inside the eye?  ring phase; the glow wins where it is deeper than the background (3.75 alpha h)
            if q12_de > 120 or q12_rb < 90 then q13_in <= '1'; else q13_in <= '0'; end if;
            q13_de <= q12_de;
            q13_der <= q12_de - 182 + resize(c_fl(12 downto 8), 11);
            v_hd := shift_left(resize(q12_al, 10), 2) - resize(q12_al(6 downto 2), 10);
            if q12_al /= 0 and v_hd > q_bd(12) then q13_ho <= '1'; else q13_ho <= '0'; end if;
            q13_hi <= 128 + resize(q12_al(6 downto 1), 8);
            if s_slit = '1' and q12_rb < 38 and q12_de < 140 then q13_pb <= '1'; else q13_pb <= '0'; end if;
            -- 14: final family / index -> palette address; compose flags
            if q13_de < 182 then v_ie := q13_de(8 downto 1);
            else v_ie := c_rb + resize(q13_der(4 downto 1), 8); end if;
            q14_core <= q13_pb and q13_in;
            q14_blk <= '0'; q14_ln <= '0';
            r_pa(10) <= l_blue;
            if q13_in = '1' then
                q14_fam <= "11"; q14_idx <= v_ie; r_pa(9 downto 8) <= "11"; r_pa(7 downto 0) <= v_ie;
                if v_ie >= 70 and v_ie <= 90 then q14_ln <= '1'; end if;
            elsif q13_ho = '1' then
                q14_fam <= "11"; q14_idx <= q13_hi; r_pa(9 downto 8) <= "11"; r_pa(7 downto 0) <= q13_hi;
            else
                q14_fam <= q_bf(13); q14_idx <= q_bi(13); r_pa(9 downto 8) <= q_bf(13); r_pa(7 downto 0) <= q_bi(13);
                if q_bf(13) = "00" and q_bdip(13) = '0' and q_bi(13) > 91 then q14_blk <= '1'; end if;
                if q_bf(13) = "01" and q_bi(13) > 54 then q14_blk <= '1'; end if;
                if (q_bf(13) = "00" and q_bi(13) >= 64 and q_bi(13) <= 93) or (q_bf(13) = "01" and q_bi(13) >= 30 and q_bi(13) <= 56) then
                    q14_ln <= '1';
                end if;
            end if;
            -- 15: palette (EBR); 16: fabric copy
            q15_fam <= q14_fam; q15_idx <= q14_idx; q15_core <= q14_core; q15_blk <= q14_blk; q15_ln <= q14_ln;
            q16_y <= r_py; q16_u <= r_pu; q16_v <= r_pv;
            q16_fam <= q15_fam; q16_idx <= q15_idx; q16_core <= q15_core; q16_blk <= q15_blk; q16_ln <= q15_ln;
            -- 17: compose: black walls / black pupil; the palette colour's blue component
            v_bk := q16_core or (s_blk and q16_blk);
            if v_bk = '1' then
                q17_y <= to_unsigned(64, 10); q17_u <= to_unsigned(512, 10); q17_v <= to_unsigned(512, 10);
                q17_bp <= (others => '0');
            else
                q17_y <= q16_y & "00"; q17_u <= q16_u & "00"; q17_v <= q16_v & "00";
                v_dd := signed(resize(q16_v, 11)) - 128;                                   -- (V - 512) / 4
                v_bp := signed(resize(q16_y, 13)) - 16 + shift_left(resize(v_dd, 13), 1) + resize(shift_right(v_dd, 3), 13);
                if v_bp < 0 then q17_bp <= (others => '0');
                elsif v_bp > 1023 then q17_bp <= (others => '1');
                else q17_bp <= unsigned(v_bp(9 downto 0)); end if;
            end if;
            q17_ln <= q16_ln;
            -- 18: drain product |kappa| Bp
            if q17_ln = '1' then q18_kp <= (others => '0'); else q18_kp <= resize(c_kmag * q17_bp, 16); end if;
            q18_y <= q17_y; q18_u <= q17_u; q18_v <= q17_v;
            -- 19: apply: V -+ 3/128 |kappa| Bp
            q19_y <= q18_y;
            v_dv := shift_right(q18_kp, 6) + shift_right(q18_kp, 7);
            if c_kneg = '1' then q19_v <= signed(resize(q18_v, 12)) + signed(resize(v_dv, 12));
            else q19_v <= signed(resize(q18_v, 12)) - signed(resize(v_dv, 12)); end if;
            q19_u <= q18_u;
            -- 20: clamps
            if q19_y > 940 then q20_y <= to_unsigned(940, 10); else q20_y <= q19_y; end if;
            q20_u <= q19_u;
            if q19_v < 16 then q20_v <= to_unsigned(16, 10);
            elsif q19_v > 1008 then q20_v <= to_unsigned(1008, 10);
            else q20_v <= unsigned(q19_v(9 downto 0)); end if;
            -- 21: bars (every other stripe's chroma negated); neutral black outside the generated avid
            if s_gav_sr(19) = '0' then
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            elsif s_bars = '1' and l_blue = '1' then
                s_y21 <= q20_y; s_u21 <= (not q20_u) + 1; s_v21 <= (not q20_v) + 1;
            else
                s_y21 <= q20_y; s_u21 <= q20_u; s_v21 <= q20_v;
            end if;
            s_bh <= s_bh(6 downto 0) & s_gav_sr(19);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent running
    -- 8-sums that snap back to neutral once 8 blank samples are in), luma [1 2 1]
    -- centred on the same sample, luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_yf : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 22: running sums, luma window
            s_cu(0) <= s_u21; s_cv(0) <= s_v21;
            for i in 1 to 7 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            if s_bh = "00000000" then
                s_bu <= to_unsigned(4096, 13); s_bv <= to_unsigned(4096, 13);
            else
                s_bu <= s_bu + s_u21 - s_cu(7);
                s_bv <= s_bv + s_v21 - s_cv(7);
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_wy(0) <= s_y21;
            for i in 1 to 5 loop s_wy(i) <= s_wy(i - 1); end loop;
            -- 23: chroma = (S8(t) + S8(t - 1)) / 16, centred 4 samples back; luma [1 2 1] centred there too
            v_sc := resize(s_bu, 14) + s_bud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bv, 14) + s_bvd; s_v23 <= v_sc(13 downto 4);
            v_sy := resize(s_wy(3), 12) + shift_left(resize(s_wy(4), 12), 1) + resize(s_wy(5), 12);
            s_y23 <= v_sy(11 downto 2);
            -- 24: luma 13/16; blanking gate
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            v_yf := s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y24 <= v_yf; s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cateye;
