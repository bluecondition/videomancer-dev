-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Jukebox v0.6.1 -- Heirloom #16, rebuilt from the photo WIN_20230315_00_28_42.
-- v0.6.0 (phase 2): the grid is fixed at the photo's 9.15 x 2.08 and K1 / K2 size the capsules one
-- by one instead of zooming the screen; K5 detunes the two column kinds; S8 swaps the band waveform.
-- v0.6.1 (phase 2 final, from Ron's first-flash notes; approved 2026-11-03): the thin rainbow lines before
-- every A -> B cut were one bad table entry (fixed in the hook); K1 Lens / Fan both run to L = 1.5 (past 1
-- the step reverses: folded, mirrored
-- columns), K2 Cascade x10 at the top and Alternate x20 / x1.5, K3 Nose to 21x (readout 200 %); the band
-- offset E is interpolated between table positions so magnified columns don't step.
--
-- The photo: a wall of tall rainbow capsules, 9.15 columns by 2.08 rows.  In
-- each column the contour bands run flat across and bend into rounded noses
-- at the column's left side, right after a hard cut; violet/blue cores on the
-- row centres (lines 259 / 779), red/yellow seams between the rows.  The
-- columns alternate in two kinds (red-topped / yellow-topped, half a band
-- apart).
--
-- The patch, as measured (jukebox.py has every constant and table):
--   vertical    dn = |y - row centre| / half-row; V = 0.0078 + 3.625 dn + 1.711 dn^2
--               fold units, with a 0.77 dip on the seam.
--   horizontal  a 2-column oscillator; every column starts with a reset
--               transient.  Its measured waveforms (column tables, 2 kinds x
--               256 positions) feed the three colour channels:
--   G  = 10 + 233 (o + c tri(a Bands (V + etch luma) + E - t))   the rainbow contours
--   R  = r + 36.5 (V + tint) - 0.09 G                            red rises to the seams
--   B  = 184 + bo - 79 bg (V - tint) - 0.086 G                   blue fills the cores
--   then BT.601 to 10-bit YUV (luma pre-scaled 16/13).
--
--   K1  Widths    per-column widths, bipolar (centre = the photo's equal columns): right = Lens (the
--                 centre columns swell, the edge columns squeeze, like a curved jukebox front), left =
--                 Fan (columns widen toward the left and narrow toward the right: a wall in perspective);
--                 the last third of each side folds the wall (the step reverses: mirrored columns)
--   K2  Heights   per-row heights, bipolar (centre = the photo's 2.08 equal rows): right = Cascade (rows
--                 small at the top growing down the screen, up to ~10 rows), left = Alternate (tall
--                 capsule rows between thin strips, x1.5 / x20)
--   K3  Nose      how far the rounded noses reach into each column (0.31x .. 21x; 512 = the photo)
--   K4  Speed     bipolar band flow (centre +-8 still; right = bands stream out of the cores)
--   K5  Detune    band count of the two column kinds against each other, bipolar (centre = the photo):
--                 right = red-topped columns up to 2.8x the bands, yellow-topped down to 0.35x; left = reverse
--   K6  Etch      the picture's luma pushed into the contours (0 .. 6 folds, twice v0.5's top)
--   S7  Mirror    every other column reversed: closed capsules
--   S8  Fold      Tri (the photo's soft triangle-folded bands) / Saw (hard-edged sawtooth bands)
--   S9  Palette   Photo / Swap (red and blue exchanged: red cores, blue seams)
--   S10 Stack     Mirror (the photo) / Saw (rows repeat, cut flat at every seam)
--   S11 Source    Video / Osc
--   P12 BANDS     contour bands per capsule: 0 = one soft gradient, 100 % = fine filigree
--
-- Engine: a vblank-zone sequencer (one serial multiplier, fixed slots, a shared
-- glide) sets the per-field constants; a per-line engine (one serial
-- multiplier, kicked at each line start, computes the NEXT line) sets the
-- vertical field values; the beam runs the 2-column phase.  Column tables and
-- the colour matrix live in block RAM.  Generated avid (parity-locked);
-- output conditioning (chroma 9-tap, luma [1 2 1], luma 13/16) before the
-- blanking gate.  Latency 28 clocks in every mode.  Colours in the hardware
-- convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.jukebox_pkg.all;
use work.all;

architecture jukebox of program_top is
    constant C_LAT  : integer := 28;          -- = processing_delay_clks
    constant C_HC   : integer := 1175078;     -- 2-column phase at the screen centre (Q24)
    -- v0.6: the grid is fixed at the photo's (v0.5 K1 583 -> 9.15 columns, K2 475 -> 2.08 rows)
    constant C_HD   : integer := 39955;       -- 8738 + (96308 (583^2 >> 4)) >> 16: 2-column step at 1920
    constant C_RQ   : integer := 2127;        -- 1024 + (5130 (475^2 >> 4)) >> 16: rows x 1024
    constant C_VPH0 : integer := 8406359;     -- vertical phase of the first line (Q24)

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
    -- v0.6: the vertical step of the photo's 2.08 rows per timing ((2127 ys) 3884 >> 16)
    function f_vstepc(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(143452, 20);
            when 4          => return to_unsigned(72608, 20);
            when 8          => return to_unsigned(121014, 20);
            when 12         => return to_unsigned(60507, 20);
            when 5 | 6 | 14 => return to_unsigned(48405, 20);
            when 1 | 2 | 10 => return to_unsigned(64541, 20);
            when others     => return to_unsigned(32270, 20);
        end case;
    end function;
    -- v0.6 K1 Widths DDA constants: the step profile hstep (1 + L g'(t)), g' = p0 + p1 t + p2 t^2 (Lens: -1, 0, 3;
    -- Fan: -0.2875, 1.45, 0.8625; both integrate to 0 and g'(-1) = 2 / -7/8), t = -1 + x dt, dt = 2 / width.
    -- KD16 = |p1 dt + p2 (dt^2 - 2 dt)| 2^21 (the first difference, negative for both), K28 = 2 p2 dt^2 2^30
    -- (the second difference).  Index = side & width class.
    function f_kd(side : std_logic; c : unsigned(1 downto 0)) return unsigned is
    begin
        case to_integer(c) is
            when 1      => if side = '1' then return to_unsigned(897, 16); else return to_unsigned(19645, 16); end if;
            when 2      => if side = '1' then return to_unsigned(1588, 16); else return to_unsigned(34904, 16); end if;
            when others => if side = '1' then return to_unsigned(599, 16); else return to_unsigned(13100, 16); end if;
        end case;
    end function;
    function f_k2(side : std_logic; c : unsigned(1 downto 0)) return unsigned is
    begin
        case to_integer(c) is
            when 1      => if side = '1' then return to_unsigned(4522, 16); else return to_unsigned(15729, 16); end if;
            when 2      => if side = '1' then return to_unsigned(14292, 16); else return to_unsigned(49710, 16); end if;
            when others => if side = '1' then return to_unsigned(2010, 16); else return to_unsigned(6991, 16); end if;
        end case;
    end function;
    -- v0.6 K2 Cascade: the step decrement per line (Q4) = c 9.8 vstep / lines = (cc KDT) >> 10, per timing
    -- (v0.6.1: 9.8, was 4.75; the step falls from x(1 + 9c) at the top to x(1 - 0.8c) at the bottom)
    function f_kdt(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(370333, 19);
            when 4          => return to_unsigned(94874, 19);
            when 8          => return to_unsigned(263542, 19);
            when 12         => return to_unsigned(65885, 19);
            when 5 | 6 | 14 => return to_unsigned(42167, 19);
            when 1 | 2 | 10 => return to_unsigned(74963, 19);
            when others     => return to_unsigned(18740, 19);
        end case;
    end function;
    -- width class: 0 = 1920, 1 = 1280, 2 = 720
    function f_cls(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(2, 2);
            when 5 | 6 | 14     => return to_unsigned(1, 2);
            when others         => return to_unsigned(0, 2);
        end case;
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0, of the line being drawn
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');


    -- oscillator source (Source = Osc)
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_clsr : unsigned(1 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(8 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_half : unsigned(10 downto 0) := to_unsigned(960, 11);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(326 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(406 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');                 -- shared glide: selected knob
    signal s_gd : signed(14 downto 0) := (others => '0');                   -- and its step
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_vm : unsigned(9 downto 0) := (others => '0');                  -- |speed| - 8
    signal s_vneg, s_vstill : std_logic := '1';
    signal s_mirror, s_fsaw, s_swap, s_saw, s_src : std_logic := '0';
    signal s_cls  : unsigned(1 downto 0) := (others => '0');
    -- v0.6 knob decodes (free-running, from the glided knobs)
    signal s_lq    : unsigned(9 downto 0) := (others => '0');               -- K1 |widths| (0 .. 511, Q9 of L)
    signal s_wside : std_logic := '0';                                      -- 0 = Lens, 1 = Fan
    signal s_v5    : unsigned(8 downto 0) := (others => '0');               -- K5 |detune| (0 .. 504)
    signal s_dneg  : std_logic := '0';
    signal s_v2    : unsigned(8 downto 0) := (others => '0');               -- K2 Alternate (0 .. 504)
    signal s_cc    : unsigned(7 downto 0) := (others => '0');               -- K2 Cascade c (Q8)
    signal s_hcas  : std_logic := '0';                                      -- 1 = Cascade side
    signal s_dkn   : unsigned(9 downto 0) := (others => '0');               -- shared knob decoder: input,
    signal s_dvm   : unsigned(9 downto 0) := (others => '0');               -- |v| - 8,
    signal s_dsg, s_dst : std_logic := '0';                                 -- sign, centre zone

    -- per-field constants
    signal s_psq, s_k6sq : unsigned(19 downto 0) := (others => '0');
    signal s_hstep : unsigned(17 downto 0) := to_unsigned(39955, 18);
    signal s_hp0   : unsigned(23 downto 0) := (others => '0');

    signal s_vstep : unsigned(19 downto 0) := to_unsigned(32270, 20);
    signal s_bands : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_etch  : unsigned(10 downto 0) := (others => '0');
    signal s_be    : unsigned(12 downto 0) := (others => '0');
    signal s_vsq   : unsigned(17 downto 0) := (others => '0');
    signal s_spd   : signed(15 downto 0) := (others => '0');
    signal s_t17   : unsigned(16 downto 0) := (others => '0');
    signal s_t9    : unsigned(8 downto 0) := (others => '0');
    signal s_nose  : unsigned(9 downto 0) := to_unsigned(256, 10);         -- nose stretch (Q8, 1/stretch)
    -- v0.6 per-field constants
    signal s_wfix  : unsigned(14 downto 0) := (others => '0');              -- 23 A (Fan centre fix)
    signal s_lh    : unsigned(16 downto 0) := (others => '0');              -- K1 L x hstep
    signal s_s0    : signed(31 downto 0) := to_signed(39955 * 4096, 32);   -- the first pixel's step (Q12; < 0 past L = 1)
    signal s_d0    : unsigned(23 downto 0) := (others => '0');              -- |first step difference| (Q12)
    signal s_d2    : unsigned(15 downto 0) := (others => '0');              -- second difference (Q12)
    signal s_fa, s_fb : unsigned(9 downto 0) := to_unsigned(256, 10);      -- detune factors (Q8)
    signal s_bna, s_bnb : unsigned(11 downto 0) := to_unsigned(256, 12);    -- Bands of column kind A / B
    signal s_ma    : unsigned(11 downto 0) := to_unsigned(256, 12);         -- row-kind step factors (Q8)
    signal s_mb    : unsigned(12 downto 0) := to_unsigned(256, 13);
    signal s_sb    : unsigned(25 downto 0) := to_unsigned(32270 * 16, 26);  -- row kind B's step (Q4, Alternate)
    signal s_hcasf : std_logic := '0';                                      -- Cascade, field-latched
    signal s_dd    : unsigned(17 downto 0) := (others => '0');              -- Cascade step decrement per line (Q4)

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_ld  : std_logic := '0';                                        -- load a slot next clock (registered decode)
    signal f_lk  : unsigned(5 downto 0) := (others => '0');                 -- and which
    signal f_acc : unsigned(35 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk   : std_logic := '0';
    signal e_ph   : unsigned(28 downto 0) := to_unsigned(C_VPH0, 25) & x"0";    -- 25-bit phase (row pair) + Q4
    signal e_sa   : unsigned(24 downto 0) := to_unsigned(32270 * 16, 25);   -- row kind A's step this line (Q4)
    signal e_par  : std_logic := '0';                                       -- row kind of the computed line
    signal e_dn   : unsigned(11 downto 0) := (others => '0');
    signal e_dn2  : unsigned(11 downto 0) := (others => '0');
    signal e_t1   : unsigned(21 downto 0) := (others => '0');               -- 928 dn
    signal e_v2   : unsigned(20 downto 0) := (others => '0');               -- 438 dn^2
    signal e_w    : unsigned(12 downto 0) := (others => '0');
    signal e_x1   : unsigned(23 downto 0) := (others => '0');
    signal e_wx   : unsigned(12 downto 0) := (others => '0');
    signal e_dp   : unsigned(20 downto 0) := (others => '0');
    signal e_dip  : unsigned(7 downto 0) := (others => '0');
    signal e_dq   : unsigned(15 downto 0) := (others => '0');
    signal e_v0, e_vg, e_vr : unsigned(10 downto 0) := (others => '0');
    signal e_rv1, e_bv1 : signed(11 downto 0) := (others => '0');
    signal e_rv2, e_bv2 : signed(19 downto 0) := (others => '0');
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(11 downto 0) := (others => '0');
    signal m_acc  : unsigned(23 downto 0) := (others => '0');
    signal n_bvga, n_bvgb : unsigned(14 downto 0) := (others => '0');
    signal n_rv, n_bv : signed(9 downto 0) := (others => '0');
    signal l_bvga, l_bvgb : unsigned(14 downto 0) := (others => '0');
    signal l_rv, l_bv : signed(9 downto 0) := (others => '0');

    -- beam
    signal b_hp : unsigned(23 downto 0) := (others => '0');
    signal b_px : unsigned(10 downto 0) := (others => '0');
    signal b_s  : signed(31 downto 0) := to_signed(39955 * 4096, 32);      -- K1 DDA: this pixel's step (Q12)
    signal b_d  : signed(25 downto 0) := (others => '0');                   -- and its difference

    -- pixel path
    signal p1_typ : std_logic := '0';
    signal p1_pos : unsigned(11 downto 0) := (others => '0');               -- column position, 8.4
    signal p2_typ, p2_ge : std_logic := '0';
    signal p2_pos : unsigned(11 downto 0) := (others => '0');
    signal p2_d   : unsigned(9 downto 0) := (others => '0');                -- distance into the tail, 8.2
    signal p3_typ, p3_ge : std_logic := '0';
    signal p3_pos : unsigned(11 downto 0) := (others => '0');
    signal p3_nl, p3_nh : unsigned(14 downto 0) := (others => '0');
    signal p4_fr, p5_fr, p6_fr : unsigned(3 downto 0) := (others => '0');  -- table-address fraction (E interpolation)
    signal p7_ei  : signed(11 downto 0) := (others => '0');
    signal p3_y8  : unsigned(7 downto 0) := (others => '0');
    signal p4_ad  : unsigned(8 downto 0) := (others => '0');
    signal p4_bla, p4_blb : unsigned(16 downto 0) := (others => '0');
    signal p5_bl  : unsigned(12 downto 0) := (others => '0');
    signal p5_bvg : unsigned(14 downto 0) := (others => '0');
    signal t5_0, t5_1, t5_2, t5_3 : std_logic_vector(15 downto 0) := (others => '0');
    signal t6_0, t6_1, t6_2, t6_3 : std_logic_vector(15 downto 0) := (others => '0');
    signal p6_gs  : unsigned(14 downto 0) := (others => '0');
    signal p7_gal, p7_gah : unsigned(18 downto 0) := (others => '0');
    signal p7_bbl, p7_bbh : signed(14 downto 0) := (others => '0');
    signal p7_e   : unsigned(8 downto 0) := (others => '0');
    signal p7_c, p8_c, p9_c : unsigned(7 downto 0) := (others => '0');
    signal p7_o, p8_o, p9_o, p10_o, p11_o : signed(7 downto 0) := (others => '0');
    signal p7_r, p15_r, p7_bo, p15_bo : unsigned(7 downto 0) := (others => '0');   -- 7 -> 15 through r_drg
    signal p8_ga9 : unsigned(8 downto 0) := (others => '0');
    signal p8_e   : unsigned(8 downto 0) := (others => '0');
    signal p8_bb, p15_bb : signed(10 downto 0) := (others => '0');          -- 8 -> 15 through r_brg
    signal p9_tri : unsigned(7 downto 0) := (others => '0');
    signal p10_cl, p10_ch : unsigned(11 downto 0) := (others => '0');
    signal p11_ct : unsigned(8 downto 0) := (others => '0');
    signal p12_g8, p13_g8 : signed(9 downto 0) := (others => '0');
    signal p13_gm : signed(14 downto 0) := (others => '0');
    signal p14_g, p15_g : unsigned(7 downto 0) := (others => '0');
    signal p15_gr, p15_gb : unsigned(12 downto 0) := (others => '0');
    signal p16_r, p16_g, p16_b : unsigned(7 downto 0) := (others => '0');
    signal p17_ar, p17_ag, p17_ab : unsigned(7 downto 0) := (others => '0');
    signal p18_qr, p18_qg, p18_qb : std_logic_vector(31 downto 0) := (others => '0');
    signal p19_qr, p19_qg, p19_qb : std_logic_vector(31 downto 0) := (others => '0');
    signal p20_ya, p20_yb : unsigned(11 downto 0) := (others => '0');
    signal p20_ua, p20_ub, p20_va, p20_vb : signed(11 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 box inputs
    signal s_y22, s_y23, s_y24, s_y25, s_y26, s_y27x, s_yf27 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

    -- v0.6.1 delay rings in block RAM, written every clock at r_wp (FF-to-FF copies cost an LC per bit):
    -- r and bo from stage 7 to 15 (read 6 back, two registers = 8), bb from 8 to 15 (read 5 back), and the
    -- syncs (read 26 back, two registers = 28 = C_LAT).  The read address never equals the write address.
    attribute no_rw_check : boolean;
    type t_ring16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_drg : t_ring16 := (others => (others => '0'));
    attribute no_rw_check of r_drg : signal is true;
    signal r_dq, r_dq2 : std_logic_vector(15 downto 0) := (others => '0');
    type t_ring11 is array (0 to 255) of std_logic_vector(10 downto 0);
    signal r_brg : t_ring11 := (others => (others => '0'));
    attribute no_rw_check of r_brg : signal is true;
    signal r_bq, r_bq2 : std_logic_vector(10 downto 0) := (others => '0');
    type t_ring4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal r_srg : t_ring4 := (others => (others => '1'));
    attribute no_rw_check of r_srg : signal is true;
    signal r_sq, r_sq2 : std_logic_vector(3 downto 0) := (others => '1');
    signal r_wp : unsigned(7 downto 0) := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Delay rings (block RAM)
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_wp <= r_wp + 1;
            r_drg(to_integer(r_wp)) <= std_logic_vector(p7_r) & std_logic_vector(p7_bo);
            r_dq <= r_drg(to_integer(r_wp - 6));
            r_dq2 <= r_dq;
            r_brg(to_integer(r_wp)) <= std_logic_vector(p8_bb);
            r_bq <= r_brg(to_integer(r_wp - 5));
            r_bq2 <= r_bq;
            r_srg(to_integer(r_wp)) <= '1' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            r_sq <= r_srg(to_integer(r_wp - 26));
            r_sq2 <= r_sq;
        end if;
    end process p_ring;
    p15_r  <= unsigned(r_dq2(15 downto 8));
    p15_bo <= unsigned(r_dq2(7 downto 0));
    p15_bb <= signed(r_bq2);

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer.  Every zone-clock step is one
    -- small operation (the zone is quasi-static but timed at the pixel clock).
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : integer range 0 to 64;
        variable v_fh, v_fl : unsigned(9 downto 0);

    begin
        if rising_edge(clk) then
            s_tid  <= registers_in(8)(3 downto 0);
            s_ys   <= f_vstep_y(s_tid);
            s_clsr <= f_cls(s_tid);

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
            elsif s_zc /= 511 then
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
            -- detune factors (Q8): x1 .. x2.84 up and x1 .. x0.36 down, linear in the knob
            v_fh := to_unsigned(256, 10) + resize(s_v5, 10) - resize(s_v5(8 downto 4), 10);
            v_fl := to_unsigned(256, 10) - resize(s_v5(8 downto 2), 10) - resize(s_v5(8 downto 4), 10) - resize(s_v5(8 downto 6), 10);
            if s_dneg = '1' then s_fa <= v_fh; s_fb <= v_fl; else s_fa <= v_fl; s_fb <= v_fh; end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mirror   <= registers_in(6)(0);
                s_fsaw     <= registers_in(6)(1);
                s_swap     <= registers_in(6)(2);
                s_saw      <= registers_in(6)(3);
                s_src      <= registers_in(6)(4);
                s_cls      <= s_clsr;
            end if;
            s_half <= s_w(11 downto 1);
            g_wm1 <= s_w - 1;

            -- shared glide (1/8 of the gap per field, +-2 deadband): knob k on
            -- zone clocks 3k+1 (select, distance), 3k+2 (step), 3k+3 (update)
            -- one bipolar decoder for K1 / K2 / K5 (glided, just after each glide update) and K4 (raw),
            -- three zone clocks per knob: the input (zone clock 4 / 7 / 13 / 20), |v| - 8, its sign and the
            -- +-8 centre zone (+1), the knob's own values (+2), latched for the field
            case to_integer(s_zc) is
                when 4      => s_dkn <= s_g1(12 downto 3);
                when 7      => s_dkn <= s_g2(12 downto 3);
                when 13     => s_dkn <= s_g5(12 downto 3);
                when others => s_dkn <= s_k4;
            end case;
            v_s := signed(resize(s_dkn, 11)) - 512;
            if v_s(10) = '1' then s_dvm <= unsigned(not v_s(9 downto 0)) - 7; else s_dvm <= unsigned(v_s(9 downto 0)) - 8; end if;
            s_dsg <= v_s(10);
            if v_s > -9 and v_s < 9 then s_dst <= '1'; else s_dst <= '0'; end if;
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 6 =>       -- K1 Widths: Lens (right) / Fan (left), L = lq / 512 up to 1.5 on both sides
                        s_wside <= s_dsg;
                        if s_dst = '1' then s_lq <= (others => '0');
                        else s_lq <= s_dvm + resize(s_dvm(9 downto 1), 10) + resize(s_dvm(9 downto 5), 10); end if;
                    when 9 =>       -- K2 Heights: Cascade (right, c = cc / 256) / Alternate (left)
                        if s_dst = '1' then
                            s_hcas <= '0'; s_v2 <= (others => '0'); s_cc <= (others => '0');
                        elsif s_dsg = '1' then
                            s_hcas <= '0'; s_v2 <= s_dvm(8 downto 0); s_cc <= (others => '0');
                        else
                            s_hcas <= '1'; s_v2 <= (others => '0'); s_cc <= s_dvm(8 downto 1);
                        end if;
                    when 15 =>      -- K5 Detune
                        s_dneg <= s_dsg;
                        if s_dst = '1' then s_v5 <= (others => '0'); else s_v5 <= s_dvm(8 downto 0); end if;
                    when 22 =>      -- K4 Speed
                        s_vstill <= s_dst; s_vneg <= s_dsg; s_vm <= s_dvm;
                    when others => null;
                end case;
            end if;

            if s_zone = '1' and s_zc >= 1 and s_zc <= 18 then
                case to_integer(s_zc) is
                    when 1  => s_gm <= s_g1; s_gd <= signed(resize(unsigned(registers_in(0)) & "000", 15)) - signed(resize(s_g1, 15));
                    when 4  => s_gm <= s_g2; s_gd <= signed(resize(unsigned(registers_in(1)) & "000", 15)) - signed(resize(s_g2, 15));
                    when 7  => s_gm <= s_g3; s_gd <= signed(resize(unsigned(registers_in(2)) & "000", 15)) - signed(resize(s_g3, 15));
                    when 10 => s_gm <= s_g5; s_gd <= signed(resize(unsigned(registers_in(4)) & "000", 15)) - signed(resize(s_g5, 15));
                    when 13 => s_gm <= s_g6; s_gd <= signed(resize(unsigned(registers_in(5)) & "000", 15)) - signed(resize(s_g6, 15));
                    when 16 => s_gm <= s_gp; s_gd <= signed(resize(unsigned(registers_in(7)) & "000", 15)) - signed(resize(s_gp, 15));
                    when 2 | 5 | 8 | 11 | 14 | 17 =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                    when 3  => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 6  => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 9  => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 12 => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 15 => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                end case;
            end if;

            -- serial products: one slot every 24 zone clocks (k = 1, 4 .. 55):
            -- load at 8k, 16 iterations, result stable from 8k + 17.  (v0.6: the
            -- column / row counts and the vertical step are per-timing constants;
            -- slots 1 .. 16 now set up the K1 Widths DDA.)  The slot decision is
            -- registered one zone clock ahead (f_ld / f_lk), so only a registered
            -- selector drives the operand mux.
            v_k := to_integer(s_zc(8 downto 3)) + 1;
            f_ld <= '0';
            if s_zc(2 downto 0) = "111" and (v_k = 1 or v_k = 7 or v_k = 10 or v_k = 13 or v_k = 16 or v_k = 19 or
               v_k = 22 or v_k = 25 or v_k = 28 or v_k = 31 or v_k = 34 or v_k = 37 or v_k = 40 or
               v_k = 43 or v_k = 46 or v_k = 49 or v_k = 52 or v_k = 55) then
                f_ld <= '1';
            end if;
            f_lk <= to_unsigned(v_k mod 64, 6);
            if f_ld = '1' then
                f_acc <= (others => '0');
                case to_integer(f_lk) is
                    when 1  => f_a <= resize(s_hstep, 36);             f_b <= resize(s_lq, 16);
                    when 7  => f_a <= resize(s_hstep, 36);             f_b <= resize(s_half, 16);
                    when 10 => f_a <= resize(f_acc(26 downto 15), 36); f_b <= resize(s_lq, 16);   -- half-screen span (slot 7)
                    when 13 => f_a <= resize(s_lh, 36);                f_b <= f_kd(s_wside, s_cls);
                    when 16 => f_a <= resize(s_lh, 36);                f_b <= f_k2(s_wside, s_cls);
                    when 19 => f_a <= resize(s_v2, 36);                f_b <= resize(s_v2, 16);
                    when 22 => f_a <= resize(s_gp(12 downto 3), 36);  f_b <= resize(s_gp(12 downto 3), 16);
                    when 25 => f_a <= resize(s_psq, 36);               f_b <= to_unsigned(1222, 16);
                    when 28 => f_a <= resize(s_g6(12 downto 3), 36);  f_b <= resize(s_g6(12 downto 3), 16);
                    when 31 => f_a <= resize(s_k6sq, 36);              f_b <= to_unsigned(1538, 16);
                    when 34 => f_a <= resize(s_bands, 36);             f_b <= resize(s_etch, 16);
                    when 37 => f_a <= resize(s_vm, 36);                f_b <= resize(s_vm, 16);
                    when 40 => f_a <= resize(s_vsq, 36);               f_b <= to_unsigned(66, 16);
                    when 43 => f_a <= resize(s_bands, 36);             f_b <= resize(s_fa, 16);
                    when 46 => f_a <= resize(s_bands, 36);             f_b <= resize(s_fb, 16);
                    when 49 => f_a <= resize(s_vstep, 36);             f_b <= resize(s_ma, 16);
                    when 52 => f_a <= resize(s_vstep, 36);             f_b <= resize(s_mb, 16);
                    -- 55: (others)
                    when others => f_a <= resize(f_kdt(s_tid), 36);    f_b <= resize(s_cc, 16);
                end case;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            -- stores and shift-adds (one operation per zone clock)
            case to_integer(s_zc) is
                when 1   =>
                    case to_integer(s_cls) is
                        when 1      => s_hstep <= to_unsigned(C_HD + C_HD / 2, 18);
                        when 2      => s_hstep <= to_unsigned(C_HD * 2, 18);
                        when others => s_hstep <= to_unsigned(C_HD, 18);
                    end case;
                    s_vstep <= f_vstepc(s_tid);
                -- K1 Widths DDA: Lh = L hstep; the first pixel's step hstep + Lh g'(-1); its first and second
                -- differences (L = 0 gives hstep and 0, 0: the photo's uniform columns exactly)
                when 26  => s_lh <= f_acc(25 downto 9);
                when 27  =>         -- Lens: hstep + 2 Lh; Fan: hstep - 7/8 Lh (negative past L = 8/7)
                    if s_wside = '1' then s_s0 <= signed(shift_left(resize(s_hstep, 32), 12)) - signed(shift_left(resize(s_lh, 32), 12));
                    else s_s0 <= signed(shift_left(resize(s_hstep, 32), 12)) + signed(shift_left(resize(s_lh, 32), 13)); end if;
                when 28  => if s_wside = '1' then s_s0 <= s_s0 + signed(shift_left(resize(s_lh, 32), 9)); end if;
                when 74  => s_hp0 <= to_unsigned(C_HC, 24) - f_acc(23 downto 0);
                -- K1: A = L x half-screen span = lq hh15 / 512 (1/256 columns); Fan moves the centre column by
                -- -0.725 A, so the start phase takes +23/32 A back (the Lens integrates to 0 at the centre)
                when 98  => s_wfix <= shift_left(resize(f_acc(20 downto 9), 15), 4) + shift_left(resize(f_acc(20 downto 9), 15), 2);
                when 99  => s_wfix <= s_wfix + shift_left(resize(f_acc(20 downto 9), 15), 1) + resize(f_acc(20 downto 9), 15);
                when 100 => if s_wside = '1' then s_hp0 <= s_hp0 + (s_wfix(13 downto 5) & "000000000000000"); end if;
                when 122 => s_d0 <= f_acc(32 downto 9);
                when 146 => s_d2 <= f_acc(33 downto 18);
                -- K2 Heights: row-kind step factors (Q8).  Cascade: 1 + 9c for both (x10 at the end); Alternate:
                -- tall kind 1 + v/256 (x1.5), thin kind 1 + v^2 / 51 (x20 at the end; v^2 from slot 19)
                when 170 =>
                    s_hcasf <= s_hcas;
                    if s_hcas = '1' then
                        s_ma <= to_unsigned(256, 12) + shift_left(resize(s_cc, 12), 3) + resize(s_cc, 12);
                        s_mb <= to_unsigned(256, 13) + shift_left(resize(s_cc, 13), 3) + resize(s_cc, 13);
                    else
                        s_ma <= to_unsigned(256, 12) + resize(s_v2(8 downto 2), 12);
                        s_mb <= to_unsigned(256, 13) + resize(f_acc(17 downto 6), 13) + resize(f_acc(17 downto 8), 13);
                    end if;
                when 194 => s_psq <= f_acc(19 downto 0);
                when 218 => s_bands <= to_unsigned(64, 11) + f_acc(30 downto 20);
                when 242 => s_k6sq <= f_acc(19 downto 0);
                when 266 => s_etch <= f_acc(30 downto 20);
                when 290 => s_be <= f_acc(20 downto 8);
                when 314 => s_vsq <= f_acc(17 downto 0);
                when 338 =>
                    if s_vstill = '1' then
                        s_spd <= (others => '0');
                    elsif s_vneg = '1' then
                        s_spd <= -signed(resize(f_acc(23 downto 10), 16));
                    else
                        s_spd <= signed(resize(f_acc(23 downto 10), 16));
                    end if;
                when 339 => s_t17 <= unsigned(signed(s_t17) + resize(s_spd, 17));
                when 340 => s_t9 <= s_t17(16 downto 8);
                when 341 => s_nose <= C_JB_NOSE(to_integer(s_g3(12 downto 7)));
                -- K2 Heights: row-kind step factors (Alternate LUT, or 1 + 4c for both kinds on the Cascade side)
                when 362 => s_bna <= f_acc(19 downto 8);
                when 386 => s_bnb <= f_acc(19 downto 8);
                -- (410: row kind A's step goes straight into the line engine's e_sa)
                when 434 => s_sb <= f_acc(29 downto 4);
                -- Cascade decrement per line (Q4): 9.8 c vstep / lines, one product with the per-timing constant
                when 458 => s_dd <= f_acc(27 downto 10);
                when others => null;
            end case;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
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
    -- Source side: oscillator, line-buffer write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_oty(9 downto 2); else v_l := unsigned(data_in.y(9 downto 2)); end if;
            if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line's vertical field values: dn (the vertical
    -- triangle, or the saw for Stack), V0 = 2 + 928 dn + 438 dn^2 (Q8 fold
    -- units, dn in Q12), the seam dip, Bands x Vg, the red and blue ramps.
    -- v0.6: the vertical phase steps by the row kind's own step (K2 Heights:
    -- Alternate), and both steps fall by s_dd per line (K2 Heights: Cascade);
    -- Bands x Vg twice, once per column kind (K5 Detune).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_p25 : unsigned(24 downto 0);
        variable v_p2 : unsigned(23 downto 0);
        variable v_a  : unsigned(23 downto 0);
        variable v_x  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 508 then e_zk <= '1'; end if;     -- after the last sequencer store (506)

            -- row kind A's top step from the field sequencer (zone clock 410, no lines run in the zone);
            -- kind B keeps its own constant step (Alternate) or follows A (Cascade, where only A falls)
            if s_zone = '1' and s_zc = 410 then e_sa <= f_acc(28 downto 4); end if;
            if e_zk = '1' then
                e_ph <= to_unsigned(C_VPH0, 25) & x"0";
                e_lc <= (others => '0');
            elsif g_start = '1' then
                if e_par = '1' or s_hcasf = '1' then e_ph <= e_ph + resize(e_sa, 29); else e_ph <= e_ph + resize(s_sb, 29); end if;
                e_sa <= e_sa - resize(s_dd, 25);
                e_lc <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;
            -- the pixel path takes the previous computation at the line start
            if g_start = '1' then
                l_bvga <= n_bvga; l_bvgb <= n_bvgb; l_rv <= n_rv; l_bv <= n_bv;
            end if;

            -- multiplier (iterates every clock it is not loaded)
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(22 downto 0) & '0';
            m_b <= '0' & m_b(11 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_p25 := e_ph(28 downto 4) + to_unsigned(8388608, 25);
                    e_par <= v_p25(24);                         -- row kind: the top capsule takes step A
                    v_p2 := v_p25(23 downto 0);
                    if s_saw = '1' then
                        e_dn <= v_p2(23 downto 12);
                    else
                        if v_p2(23) = '1' then v_a := '0' & v_p2(22 downto 0);
                        else v_a := to_unsigned(8388608, 24) - ('0' & v_p2(22 downto 0)); end if;
                        if v_a(23) = '1' then e_dn <= (others => '1'); else e_dn <= v_a(22 downto 11); end if;
                    end if;
                when 2 =>
                    m_a <= resize(e_dn, 24); m_b <= e_dn; m_acc <= (others => '0');
                when 3 =>
                    e_t1 <= shift_left(resize(e_dn, 22), 10) - shift_left(resize(e_dn, 22), 6);
                    e_w  <= to_unsigned(4096, 13) - resize(e_dn, 13);
                when 4 =>
                    e_t1 <= e_t1 - shift_left(resize(e_dn, 22), 5);
                    e_x1 <= shift_left(resize(e_w, 24), 11) - shift_left(resize(e_w, 24), 5);
                when 5 =>
                    e_x1 <= e_x1 - resize(e_w, 24);
                when 6 =>
                    v_x := e_x1(23 downto 6);
                    if v_x < 4096 then e_wx <= to_unsigned(4096, 13) - v_x(12 downto 0); else e_wx <= (others => '0'); end if;
                when 7 =>
                    e_dp <= shift_left(resize(e_wx, 21), 7) + shift_left(resize(e_wx, 21), 6);
                when 8 =>
                    e_dp <= e_dp + shift_left(resize(e_wx, 21), 2) + resize(e_wx, 21);
                when 9 =>
                    e_dip <= e_dp(19 downto 12);
                when 10 =>
                    e_dq <= shift_left(resize(e_dip, 16), 6) + shift_left(resize(e_dip, 16), 4);
                when 11 =>
                    e_dq <= e_dq + shift_left(resize(e_dip, 16), 3) + shift_left(resize(e_dip, 16), 1);
                when 15 =>
                    e_dn2 <= m_acc(23 downto 12);
                when 16 =>
                    e_v2 <= shift_left(resize(e_dn2, 21), 9) - shift_left(resize(e_dn2, 21), 6);
                when 17 =>
                    e_v2 <= e_v2 - shift_left(resize(e_dn2, 21), 3) - shift_left(resize(e_dn2, 21), 1);
                when 18 =>
                    e_v0 <= to_unsigned(2, 11) + resize(e_t1(21 downto 12), 11) + resize(e_v2(20 downto 12), 11);
                when 19 =>
                    e_vg <= e_v0 - resize(e_dip, 11);
                    e_vr <= e_v0 - resize(e_dq(15 downto 8), 11);
                when 20 =>
                    m_a <= resize(e_vg, 24); m_b <= s_bna; m_acc <= (others => '0');
                    e_rv1 <= signed(resize(e_vr, 12));
                    e_bv1 <= signed(resize(e_v0, 12));
                when 21 =>
                    e_rv2 <= shift_left(resize(e_rv1, 20), 6) + shift_left(resize(e_rv1, 20), 3);
                    e_bv2 <= shift_left(resize(e_bv1, 20), 6) + shift_left(resize(e_bv1, 20), 4);
                when 22 =>
                    e_rv2 <= e_rv2 + resize(e_rv1, 20);
                    e_bv2 <= e_bv2 - resize(e_bv1, 20);
                when 34 =>
                    n_bvga <= m_acc(22 downto 8);
                    n_rv  <= e_rv2(18 downto 9);
                    n_bv  <= e_bv2(17 downto 8);
                when 35 =>
                    m_a <= resize(e_vg, 24); m_b <= s_bnb; m_acc <= (others => '0');
                when 49 =>
                    n_bvgb <= m_acc(22 downto 8);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the 2-column phase
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_hp <= s_hp0;
                b_px <= (others => '0');
                b_s  <= s_s0;
                b_d  <= -signed(resize(s_d0, 26));
            elsif g_avid = '1' then
                b_hp <= b_hp + unsigned(resize(b_s(31 downto 12), 24));    -- (a negative step runs the columns backwards)
                b_px <= b_px + 1;
                -- K1 Widths: the step follows the quadratic profile across the line (second-order DDA)
                b_s  <= b_s + resize(b_d, 32);
                b_d  <= b_d + signed(resize(s_d2, 26));
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Column tables (one registered read each) and colour-matrix ROMs
    ---------------------------------------------------------------------------
    p_tab : process(clk)
    begin
        if rising_edge(clk) then
            t5_0 <= C_JB_T0(to_integer(p4_ad));
            t5_1 <= C_JB_T1(to_integer(p4_ad));
            t5_2 <= C_JB_T2(to_integer(p4_ad));
            t5_3 <= C_JB_T3(to_integer(p4_ad));
            p18_qr <= C_JB_RGB_R(to_integer(p17_ar));
            p18_qg <= C_JB_RGB_G(to_integer(p17_ag));
            p18_qb <= C_JB_RGB_B(to_integer(p17_ab));
        end if;
    end process p_tab;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_pos : unsigned(11 downto 0);
        variable v_n   : unsigned(14 downto 0);
        variable v_pw  : unsigned(10 downto 0);
        variable v_bl  : unsigned(20 downto 0);
        variable v_ga  : unsigned(22 downto 0);
        variable v_bb  : signed(18 downto 0);
        variable v_f   : unsigned(8 downto 0);
        variable v_ct  : unsigned(15 downto 0);
        variable v_g   : signed(11 downto 0);
        variable v_r   : signed(11 downto 0);
        variable v_b   : signed(12 downto 0);
        variable v_y   : unsigned(11 downto 0);
        variable v_u   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: column kind and position, 8.4 (Mirror: B reversed)
            p1_typ <= b_hp(23);
            v_pos := b_hp(22 downto 11);
            if s_mirror = '1' and b_hp(23) = '0' then v_pos := not v_pos; end if;
            p1_pos <= v_pos;

            -- 2: nose: distance into the tail (8.2)
            p2_typ <= p1_typ; p2_pos <= p1_pos;
            if p1_pos(11 downto 4) >= 48 then
                p2_ge <= '1'; v_pos := p1_pos - 768; p2_d <= v_pos(11 downto 2);
            else
                p2_ge <= '0'; p2_d <= (others => '0');
            end if;

            -- 3: tail stretch partial products (d x s, s in Q8); luma
            p3_typ <= p2_typ; p3_pos <= p2_pos; p3_ge <= p2_ge;
            p3_nl <= p2_d * s_nose(4 downto 0);
            p3_nh <= p2_d * s_nose(9 downto 5);
            p3_y8 <= unsigned(s_lb_q);

            -- 4: table address and its fraction (for the E interpolation); etch partial products
            if p3_ge = '1' then
                v_n := resize(p3_nl(14 downto 6), 15) + resize(p3_nh(14 downto 1), 15);    -- (d s) >> 6: 11.4
                v_pw := to_unsigned(48, 11) + v_n(14 downto 4);
                if v_pw > 255 then
                    p4_ad <= p3_typ & x"FF"; p4_fr <= (others => '0');
                else
                    p4_ad <= p3_typ & v_pw(7 downto 0); p4_fr <= v_n(3 downto 0);
                end if;
            else
                p4_ad <= p3_typ & p3_pos(11 downto 4); p4_fr <= p3_pos(3 downto 0);
            end if;
            p4_bla <= s_be * p3_y8(3 downto 0);
            p4_blb <= s_be * p3_y8(7 downto 4);

            -- 5: tables out of block RAM (p_tab); etch sum; this column kind's Bands x Vg (K5 Detune)
            v_bl := resize(p4_bla, 21) + shift_left(resize(p4_blb, 21), 4);
            p5_bl <= v_bl(20 downto 8);
            if p4_ad(8) = '1' then p5_bvg <= l_bvga; else p5_bvg <= l_bvgb; end if;
            p5_fr <= p4_fr;

            -- 6: fabric copies; G-field vertical term
            t6_0 <= t5_0; t6_1 <= t5_1; t6_2 <= t5_2; t6_3 <= t5_3;
            p6_gs <= resize(p5_bvg, 15) + resize(p5_bl, 15);
            p6_fr <= p5_fr;

            -- 7: gain partial products (G: a x gs; B: bg x Bv); E slope x fraction; side values
            p7_gal <= unsigned(t6_1(11 downto 8)) * p6_gs;
            p7_gah <= unsigned(t6_1(15 downto 12)) * p6_gs;
            p7_bbl <= signed('0' & t6_2(3 downto 0)) * l_bv;
            p7_bbh <= signed('0' & t6_2(7 downto 4)) * l_bv;
            p7_e  <= unsigned(t6_0(8 downto 0));
            p7_ei <= signed(t6_0(15 downto 9)) * signed('0' & p6_fr);
            p7_c  <= unsigned(t6_1(7 downto 0));
            p7_o  <= signed(t6_2(15 downto 8));
            p7_r  <= unsigned(t6_3(15 downto 8));
            p7_bo <= unsigned(t6_3(7 downto 0));

            -- 8: products; E interpolated between table positions (mod 2 folds)
            v_ga := resize(p7_gal, 23) + shift_left(resize(p7_gah, 23), 4);
            p8_ga9 <= v_ga(15 downto 7);
            v_bb := resize(p7_bbl, 19) + shift_left(resize(p7_bbh, 19), 4);
            p8_bb <= v_bb(17 downto 7);
            p8_e <= p7_e + unsigned(resize(shift_right(p7_ei, 4), 9)); p8_c <= p7_c; p8_o <= p7_o;

            -- 9: fold phase (mod 2 folds) and the triangle fold
            v_f := p8_ga9 + p8_e - s_t9;
            if s_fsaw = '1' then
                p9_tri <= v_f(8 downto 1);                                  -- S8 Saw: one hard-edged ramp per 2 folds
            elsif v_f(8) = '1' then p9_tri <= not v_f(7 downto 0); else p9_tri <= v_f(7 downto 0); end if;
            p9_c <= p8_c; p9_o <= p8_o;

            -- 10: contrast partial products
            p10_cl <= p9_c(3 downto 0) * p9_tri;
            p10_ch <= p9_c(7 downto 4) * p9_tri;
            p10_o <= p9_o;

            -- 11: contrast
            v_ct := resize(p10_cl, 16) + shift_left(resize(p10_ch, 16), 4);
            p11_ct <= v_ct(15 downto 7);
            p11_o <= p10_o;

            -- 12: G level before the 233/256 scale
            p12_g8 <= resize(p11_o, 10) + signed(resize(p11_ct, 10));
           

            -- 13: g8 * 23
            p13_gm <= shift_left(resize(p12_g8, 15), 4) + shift_left(resize(p12_g8, 15), 3) - resize(p12_g8, 15);
            p13_g8 <= p12_g8;
           

            -- 14: G = 10 + g8 * 233 / 256, clipped
            v_g := to_signed(10, 12) + resize(p13_g8, 12) - resize(shift_right(p13_gm, 8), 12);
            if v_g < 0 then p14_g <= (others => '0');
            elsif v_g > 255 then p14_g <= (others => '1');
            else p14_g <= unsigned(v_g(7 downto 0)); end if;
           

            -- 15: crosstalk terms G * 23, G * 22
            p15_gr <= shift_left(resize(p14_g, 13), 4) + shift_left(resize(p14_g, 13), 3) - resize(p14_g, 13);
            p15_gb <= shift_left(resize(p14_g, 13), 4) + shift_left(resize(p14_g, 13), 2) + shift_left(resize(p14_g, 13), 1);
            p15_g <= p14_g;

            -- 16: red and blue, clipped
            v_r := signed(resize(p15_r, 12)) + resize(l_rv, 12) - signed(resize(p15_gr(12 downto 8), 12));
            if v_r < 0 then p16_r <= (others => '0');
            elsif v_r > 255 then p16_r <= (others => '1');
            else p16_r <= unsigned(v_r(7 downto 0)); end if;
            v_b := to_signed(184, 13) + signed(resize(p15_bo, 13)) - resize(p15_bb, 13) - signed(resize(p15_gb(12 downto 8), 13));
            if v_b < 0 then p16_b <= (others => '0');
            elsif v_b > 255 then p16_b <= (others => '1');
            else p16_b <= unsigned(v_b(7 downto 0)); end if;
            p16_g <= p15_g;

            -- 17: Palette Swap; colour-matrix ROM addresses
            if s_swap = '1' then p17_ar <= p16_b; p17_ab <= p16_r; else p17_ar <= p16_r; p17_ab <= p16_b; end if;
            p17_ag <= p16_g;

            -- 18: ROMs (p_tab); 19: fabric copies
            p19_qr <= p18_qr; p19_qg <= p18_qg; p19_qb <= p18_qb;

            -- 20: partial sums
            p20_ya <= resize(unsigned(p19_qr(31 downto 22)), 12) + resize(unsigned(p19_qg(31 downto 22)), 12);
            p20_yb <= to_unsigned(64, 12) + resize(unsigned(p19_qb(31 downto 22)), 12);
            p20_ua <= resize(signed(p19_qr(21 downto 12)), 12) + resize(signed(p19_qg(21 downto 12)), 12);
            p20_ub <= to_signed(512, 12) + resize(signed(p19_qb(21 downto 12)), 12);
            p20_va <= resize(signed(p19_qr(11 downto 2)), 12) + resize(signed(p19_qg(11 downto 2)), 12);
            p20_vb <= to_signed(512, 12) + resize(signed(p19_qb(11 downto 2)), 12);

            -- 21: Y U V, clamped (drawn luma <= 940 -> ~777 after the 13/16 scale)
            v_y := p20_ya + p20_yb;
            if v_y > 940 then s_y21 <= to_unsigned(940, 10); else s_y21 <= v_y(9 downto 0); end if;
            -- chroma neutral off the generated avid, so the 8-sums in p_out resync in every blanking interval
            v_u := p20_ua + p20_ub;
            if s_gav_sr(19) = '0' then s_u21 <= to_unsigned(512, 10);
            elsif v_u < 0 then s_u21 <= (others => '0'); elsif v_u > 1023 then s_u21 <= (others => '1'); else s_u21 <= unsigned(v_u(9 downto 0)); end if;
            v_u := p20_va + p20_vb;
            if s_gav_sr(19) = '0' then s_v21 <= to_unsigned(512, 10);
            elsif v_u < 0 then s_v21 <= (others => '0'); elsif v_u > 1023 then s_v21 <= (others => '1'); else s_v21 <= unsigned(v_u(9 downto 0)); end if;
            s_bxh <= s_bxh(6 downto 0) & s_gav_sr(19);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent
    -- 8-sums, luma [1 2 1]; centre at stage 26 -> 27), luma 13/16 and the
    -- blanking gate at stage 28
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- the running 8-sums snap back to neutral once 8 blank samples are in, so a glitched sample can't
            -- offset the chroma for good
            s_csu(0) <= s_u21; s_csv(0) <= s_v21;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            if s_bxh = "00000000" then
                s_boxu <= to_unsigned(4096, 14); s_boxv <= to_unsigned(4096, 14);
            else
                s_boxu <= s_boxu + s_u21 - s_csu(7);
                s_boxv <= s_boxv + s_v21 - s_csv(7);
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u27 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v27 <= v_sc(13 downto 4);
            s_y22 <= s_y21; s_y23 <= s_y22; s_y24 <= s_y23; s_y25 <= s_y24;
            s_y26 <= s_y25; s_y27x <= s_y26;
            v_sy := resize(s_y25, 12) + shift_left(resize(s_y26, 12), 1) + resize(s_y27x, 12);
            s_yf27 <= v_sy(11 downto 2);
            if s_yf27 > 64 then v_dl := s_yf27 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(26) = '1' then
                s_y28 <= s_yf27 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u28 <= s_u27; s_v28 <= s_v27;
            else
                s_y28 <= to_unsigned(64, 10); s_u28 <= to_unsigned(512, 10); s_v28 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= r_sq2(0);
    data_out.vsync_n <= r_sq2(1);
    data_out.field_n <= r_sq2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture jukebox;
