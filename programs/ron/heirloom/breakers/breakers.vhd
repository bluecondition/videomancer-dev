-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Breakers v0.1.1 -- Heirloom (video phase), from the clip WIN_20230214_23_42_08 at 0.5 s.
--
-- v0.1.1: K5 Grain's travel made obvious (0 = clean smooth surf stripes, no jitter; 512 = the clip; 1023 = 12x
-- deeper shredding, long smears, tall slabs slipped sideways); the camera bend softened (default Etch 80, was 200)
-- with Osc raised to 10x the camera's gain, so the default Osc flip is unchanged.
--
-- The still: two wavy ribbons (blue core, pink and white edges) on saturated cyan, swinging left and right down
-- the frame; green "surf" (fine dark-teal / green / mint / lime stripes, torn into streaks) eating the ribbons
-- from the left; a flat cyan lagoon over the bottom third with blue dashes near the surf; an eye of ribbon
-- colours swelling on the right edge just above the lagoon; cyan dropout lines and slipped slabs.  In the clip
-- the ribbons roll DOWN one wave per second, and the lagoon edge sinks 136 lines a second and snaps back once a
-- second while a new eye is born at the top right, swells and sinks into it.
--
-- The patch, as fitted (breakers.py has the tables):
--   field  f = (x - X0(y)) / S(y)  band units (band 0 = the first ribbon at f = 0, band 1 at f = 1)
--          X0 = M(y) + A(y) sin(vf (y / 420 + 0.0005 y^2 / 1080) cycles - tau + phase), M, A, S fitted per row
--   clean  one band period of the still (cyan, white, pink, blue core, ...) and open cyan past f = 1.5
--   surf   left of the shore contour: streak noise jitters f and the colour comes from the surf bank (indexed
--          by shore - f); the surf sees the field held at each 4-row noise row's start, noise slabs of 6..23 rows
--   lagoon below yL(tau): cyan, with keyed blue dashes on a 64 px ripple near the shore
--   eye    the open water lifted into ribbon colours on the right edge: lift = ea (1 - t^2) by row, times a
--          ramp from band 1's white edge
--   tears  cyan dropout rows (4 virtual rows) and slipped slabs (4..16 rows, up to +-112 px)
--
--   K1  Ribbons  ribbon spacing x2.8 .. x0.35 (512 = the clip); the first ribbon stays put
--   K2  Swell    wave amplitude 0 (straight ribbons) .. the clip (512) .. 3x
--   K3  Ripple   waves per frame x0.33 .. x3 (512 = the clip, ~2.7 waves)
--   K4  Speed    bipolar: the waves roll down (+) or up (-); 700 = the clip (one wave per second)
--   K5  Grain    surf texture: clean smooth stripes (0) .. the clip (512) .. deep shredding, long smears, slipped slabs (1023)
--   K6  Etch     how far the camera (or the Osc) bends the ribbons
--   S7  Mirror   the right half reflected onto the left: ribbons meeting in the middle
--   S8  Lagoon   On (the clip) / Off: ribbons all the way down, no curl, no lagoon
--   S9  Fold     the colouriser repeats: endless ribbons over the open water (and slipped bands in the lagoon)
--   S10 Palette  Sea (the clip) / Crab (the sibling clip's black, green, orange, red and blue rings)
--   S11 Source   Video / Osc
--   P12 BREAK    calm water (0: clean ribbons, a thin strip of surf) .. the clip (410) .. a storm (surf over
--                all but the last ribbon, tears everywhere)
--
-- Engine: vblank-zone sequencer (one serial multiplier, one serial divider, shared glide); a per-line engine
-- kicked at each line start (computes the NEXT line: the wave, the band spacing and its reciprocal, the
-- field's start value, the curl, the eye's lift, the slab / row / tear random draws); a mirror-aware beam DDA;
-- two streak-noise generators reseeded per line from the slab / row seeds; a 20-stage pixel path.  Virtual
-- 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (U and V through a
-- [1 2 2 2 2 2 2 2 1]/16 box with its history in a block-RAM ring, luma [1 2 1]/4 and 13/16) before the
-- blanking gate.  Colours in the hardware convention.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.breakers_pkg.all;
use work.all;

architecture breakers of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

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
    -- per-mode constants of the line DDAs (ys = virtual rows per line, Q8): theta's second difference
    -- factor 31 ys^2 / 32, M's linear step 106112 ys / 256 and second difference 844 ys^2 / 65536, S's linear
    -- step 6887 ys / 256 and second difference 3 ys^2 / 16; and the Osc lattice's line step 121 ys / 256
    type t_mtab is array (0 to 5) of integer;
    function f_mtab(t : std_logic_vector(3 downto 0)) return t_mtab is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return (1254573, 471701, 16678, 30614, 242820, 537);
            when 4          => return (321408, 238752, 4272, 15495, 62208, 272);
            when 8          => return (892800, 397920, 11868, 25826, 172800, 453);
            when 12         => return (223200, 198960, 2967, 12913, 43200, 226);
            when 5 | 6 | 14 => return (142848, 159168, 1899, 10330, 27648, 181);
            when 1 | 2 | 10 => return (253952, 212224, 3376, 13774, 49152, 242);
            when others     => return (63488, 106112, 844, 6887, 12288, 121);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    function f_xs16(x : unsigned(15 downto 0)) return unsigned is
        variable a, b, c : unsigned(15 downto 0);
    begin
        a := x xor shift_left(x, 7);
        b := a xor shift_right(a, 9);
        c := b xor shift_left(b, 8);
        return c;
    end function;
    function f_xs32(x : unsigned(31 downto 0)) return unsigned is
        variable a, b, c : unsigned(31 downto 0);
    begin
        a := x xor shift_left(x, 13);
        b := a xor shift_right(a, 17);
        c := b xor shift_left(b, 5);
        return c;
    end function;

    -- line buffer: (luma >> 2) - 16 clipped at 0, or the Osc lattice; every other pixel
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_pa  : unsigned(10 downto 0) := (others => '0');
    signal r_pq  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_sn_a, r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_sn_q, r_ex_q : unsigned(15 downto 0) := (others => '0');

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
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(80 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(410 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_lagoff, s_fold, s_pal, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field constants
    signal c_tau  : unsigned(15 downto 0) := to_unsigned(6554, 16);    -- the master phase (Q16 cycles)
    signal c_spd  : unsigned(15 downto 0) := (others => '0');          -- tau step per field
    signal c_phi  : unsigned(15 downto 0) := to_unsigned(42322, 16);   -- wave phase at row 0
    signal c_te   : unsigned(15 downto 0) := (others => '0');          -- eye phase
    signal c_yl   : unsigned(10 downto 0) := to_unsigned(747, 11);     -- lagoon edge (virtual row)
    signal c_ey   : unsigned(10 downto 0) := to_unsigned(640, 11);
    signal c_ry   : unsigned(7 downto 0) := to_unsigned(115, 8);
    signal c_kr   : unsigned(10 downto 0) := to_unsigned(570, 11);     -- 2^16 / ry
    signal c_ea   : unsigned(10 downto 0) := to_unsigned(921, 11);     -- the eye's peak lift (Q4 index)
    signal z_xq   : unsigned(15 downto 0) := (others => '0');
    signal c_vf   : unsigned(15 downto 0) := to_unsigned(4096, 16);    -- waves per frame factor (Q12)
    signal c_sp   : unsigned(15 downto 0) := to_unsigned(4096, 16);    -- ribbon spacing factor (Q12)
    signal c_ig   : unsigned(15 downto 0) := to_unsigned(4096, 16);    -- 1 / grain (Q12)
    signal c_gst  : unsigned(7 downto 0) := to_unsigned(16, 8);        -- noise hold step per pixel (Q4 virtual px)
    signal c_ysg  : unsigned(9 downto 0) := to_unsigned(16, 10);       -- slab step per line (Q4 virtual rows)
    signal c_ak   : unsigned(12 downto 0) := to_unsigned(2442, 13);    -- wave amplitude (Q4 px)
    signal c_ak7  : unsigned(18 downto 0) := (others => '0');          -- its taper per row (Q16)
    signal c_sh4  : signed(12 downto 0) := (others => '0');            -- (Q4 index)
    signal c_a0   : unsigned(5 downto 0) := to_unsigned(10, 6);
    signal c_amax : unsigned(7 downto 0) := to_unsigned(61, 8);
    signal c_gd   : unsigned(15 downto 0) := to_unsigned(4096, 16);    -- Grain's surf depth factor (Q12)
    signal c_a0s  : unsigned(8 downto 0) := to_unsigned(10, 9);        -- surf depth x Grain
    signal c_ams  : unsigned(7 downto 0) := to_unsigned(61, 8);        -- (saturated at 255)
    signal c_rjp  : unsigned(7 downto 0) := (others => '0');           -- Grain > 512: chance a slab slips sideways
    signal c_g5hi : std_logic := '0';                                  -- Grain > 512
    signal c_g5d  : unsigned(9 downto 0) := (others => '0');           -- |Grain - 512| (above) or Grain (below)
    signal c_eg   : unsigned(10 downto 0) := (others => '0');          -- etch gain per luma step (Q15 band)
    signal c_xms  : unsigned(11 downto 0) := (others => '0');          -- first pixel's (mirrored) x (h)
    signal z_m2   : unsigned(17 downto 0) := (others => '0');
    signal z_tr   : unsigned(9 downto 0) := (others => '0');
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    -- field serial divider: 2^23 / d (d 10 bits)
    signal z_dr  : unsigned(10 downto 0) := (others => '0');
    signal z_dq  : unsigned(10 downto 0) := (others => '0');
    signal z_dd  : unsigned(9 downto 0) := (others => '0');
    signal z_dc  : unsigned(4 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (its registers are loaded into the beam / l_* at the next g_start)
    signal e_lc   : unsigned(8 downto 0) := (others => '1');
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_y    : unsigned(10 downto 0) := (others => '0');
    signal e_yq   : unsigned(9 downto 0) := (others => '0');
    signal e_dyl  : signed(11 downto 0) := (others => '0');
    signal e_dye  : unsigned(10 downto 0) := (others => '0');
    signal e_r    : unsigned(31 downto 0) := x"2545F491";
    signal e_slab : signed(11 downto 0) := (others => '0');
    signal e_row  : signed(12 downto 0) := (others => '0');
    signal e_newr, e_news : std_logic := '0';
    signal e_rj   : signed(11 downto 0) := (others => '0');            -- the slab's sideways slip (Q4 index)
    signal e_sseed, e_rseed : unsigned(15 downto 0) := x"0001";
    signal e_rip  : unsigned(15 downto 0) := (others => '0');
    signal e_tph  : unsigned(1 downto 0) := "00";
    signal e_tc   : unsigned(2 downto 0) := (others => '0');
    signal e_tsl  : signed(12 downto 0) := (others => '0');
    signal e_tlen : unsigned(2 downto 0) := (others => '0');
    signal e_drop : std_logic := '0';
    signal e_slip : signed(12 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal e_sc   : unsigned(19 downto 0) := (others => '0');           -- scratch: 639132 + 124 Y, P, then A(y)
    signal e_yq2  : unsigned(19 downto 0) := (others => '0');
    signal e_th   : unsigned(15 downto 0) := (others => '0');
    signal e_q    : unsigned(1 downto 0) := "00";
    signal e_s    : unsigned(14 downto 0) := (others => '0');
    signal e_w4   : signed(13 downto 0) := (others => '0');
    signal e_x04  : signed(15 downto 0) := (others => '0');
    signal e_s4   : unsigned(13 downto 0) := to_unsigned(3521, 14);  -- S4, then Sh2 (the divisor)
    signal e_dr   : unsigned(14 downto 0) := (others => '0');
    signal e_dq   : unsigned(11 downto 0) := (others => '0');
    signal e_g    : unsigned(11 downto 0) := (others => '0');
    signal e_d4   : signed(17 downto 0) := (others => '0');
    signal e_f20  : signed(26 downto 0) := (others => '0');
    signal e_c7   : unsigned(6 downto 0) := (others => '0');
    signal e_t8   : unsigned(7 downto 0) := (others => '0');
    signal e_ein  : std_logic := '0';
    signal e_lift : unsigned(10 downto 0) := (others => '0');
    signal e_step : unsigned(13 downto 0) := (others => '0');
    signal e_lag  : std_logic := '0';
    signal e_fsd  : signed(17 downto 0) := (others => '0');            -- the surf's field: held per noise row (Q4 index)
    signal e_dfs, e_lsd : signed(17 downto 0) := (others => '0');
    signal l_dfs, l_lsd : signed(17 downto 0) := (others => '0');
    signal l_lift : unsigned(10 downto 0) := (others => '0');
    signal l_lag, l_drop : std_logic := '0';
    signal l_rip : unsigned(6 downto 0) := (others => '0');

    -- beam
    signal b_x, b_x1 : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_f   : signed(26 downto 0) := (others => '0');
    signal b_neg : std_logic := '0';
    signal b_stp : unsigned(13 downto 0) := (others => '0');

    -- noise generators: 1 slab streaks, 2 row streaks (the slab generator also clocks the sample-and-hold)
    type t_u16x2 is array (1 to 2) of unsigned(15 downto 0);
    type t_s12x2 is array (1 to 2) of signed(12 downto 0);
    type t_s10x2 is array (1 to 2) of signed(9 downto 0);
    type t_s8x2 is array (1 to 2) of signed(7 downto 0);
    signal n_r   : t_u16x2 := (others => x"0001");
    signal n_cnt : t_s12x2 := (others => (others => '0'));
    signal n_v   : t_s10x2 := (others => (others => '0'));
    signal n_tg  : t_s8x2 := (others => (others => '0'));

    -- pixel path (field values in Q4 index = Q11 band from stage 2)
    signal p2_f, p3_f, p4_f : signed(17 downto 0) := (others => '0');
    signal p2_d8, p2_e8, p3_e8, p4_e8 : unsigned(7 downto 0) := (others => '0');
    signal p2_nf, p2_nr, p3_nr : signed(7 downto 0) := (others => '0');
    signal p2_tri, p3_tri, p4_tri, p5_tri : std_logic := '0';
    signal p3_lm : unsigned(7 downto 0) := (others => '0');
    signal p3_n8, p4_n8, p5_n8, p6_n8, p7_n8, p8_n8 : signed(7 downto 0) := (others => '0');
    signal p4_a8 : unsigned(7 downto 0) := (others => '0');
    signal p3_an : unsigned(9 downto 0) := (others => '0');
    signal p4_off : signed(16 downto 0) := (others => '0');
    signal p5_et : signed(18 downto 0) := (others => '0');
    signal p4_lms : signed(8 downto 0) := (others => '0');
    signal p5_fo, p6_fo, p7_fin, p8_sd : signed(17 downto 0) := (others => '0');
    signal p5_lev : signed(8 downto 0) := (others => '0');
    signal p6_et : signed(15 downto 0) := (others => '0');
    signal p8_j  : signed(10 downto 0) := (others => '0');
    signal p6_dash, p6_dsh2, p7_dash, p7_dsh2, p8_dash, p8_dsh2, p9_dash, p9_dsh2, p10_dash, p10_dsh2 : std_logic := '0';
    signal p8_ic0, p9_is0 : signed(14 downto 0) := (others => '0');
    signal p8_ramp : unsigned(8 downto 0) := (others => '0');
    signal p8_fm, p9_fm : signed(12 downto 0) := (others => '0');
    signal p9_fk : signed(17 downto 0) := (others => '0');
    signal p9_ep : unsigned(15 downto 0) := (others => '0');
    signal p9_eye, p10_eye, p10_surf, p11_surf : std_logic := '0';
    signal p9_ic, p10_ic, p11_ic, p10_is, p11_is : unsigned(8 downto 0) := (others => '0');
    signal p10_fl : signed(13 downto 0) := (others => '0');
    signal p12_ad, p13_ad : unsigned(10 downto 0) := (others => '0');
    signal p16_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma box history in a 256 x 20 block-RAM ring (read 8 samples back), luma [1 2 1]
    type t_bx is array (0 to 255) of std_logic_vector(19 downto 0);
    signal bx : t_bx := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of bx : signal is true;
    signal s_bwp : unsigned(7 downto 0) := (others => '0');
    signal s_bxq : std_logic_vector(19 downto 0) := (others => '0');
    signal s_bxd : std_logic_vector(19 downto 0) := (others => '0');
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yh  : t_u10(1 to 3) := (others => to_unsigned(64, 10));
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

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

    p_bx : process(clk)
    begin
        if rising_edge(clk) then
            s_bxq <= bx(to_integer(s_bwp - 6));
            bx(to_integer(s_bwp)) <= std_logic_vector(s_u17) & std_logic_vector(s_v17);
            s_bwp <= s_bwp + 1;
        end if;
    end process p_bx;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_pq   <= C_BR_PAL(to_integer(r_pa));
            r_sn_q <= C_BR_SIN(to_integer(r_sn_a));
            r_ex_q <= C_BR_EXP2(to_integer(r_ex_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_t  : unsigned(10 downto 0);
        variable v_r  : unsigned(10 downto 0);
        variable v_u  : unsigned(11 downto 0);
        variable v_b  : std_logic;
        variable v_mt : t_mtab;
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            v_mt  := f_mtab(s_tid);
            s_oys <= to_unsigned(v_mt(5), 10);

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
                s_g5 <= unsigned(registers_in(4)) & "000";       -- Grain and Etch: per-field, unglided
                s_g6 <= unsigned(registers_in(5)) & "000";
                s_mir    <= registers_in(6)(0);
                s_lagoff <= registers_in(6)(1);
                s_fold   <= registers_in(6)(2);
                s_pal    <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;

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
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 3 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from load + 17
            -- (stored at load + 18).  The slot is decoded at 30 + 24 k and the operands pre-muxed at 31 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 510 => z_sel <= to_unsigned(20, 5); z_pre <= '1';
                    when 534 => z_sel <= to_unsigned(21, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_vm, 36);              z_fb <= resize(s_vm, 16);           -- speed^2
                    when 1  => z_fa <= resize(z_m2, 36);              z_fb <= to_unsigned(2212, 16);      -- tau step
                    when 2  => z_fa <= resize(c_tau, 36);             z_fb <= to_unsigned(136, 16);       -- lagoon edge
                    when 3  => z_fa <= resize(c_te, 36);              z_fb <= to_unsigned(380, 16);       -- eye row
                    when 4  => z_fa <= resize(c_te, 36);              z_fb <= to_unsigned(3686, 16);      -- eye lift
                    when 5  => z_fa <= resize(c_te, 36);              z_fb <= to_unsigned(260, 16);       -- eye height
                    when 7  =>                                                                            -- |k3 - 512| * 205
                        if s_g3(12 downto 3) >= 512 then z_fa <= resize(s_g3(12 downto 3) - 512, 36);
                        else z_fa <= resize(512 - s_g3(12 downto 3), 36); end if;
                        z_fb <= to_unsigned(205, 16);
                    when 8  =>                                                                            -- swell
                        if s_g2(12 downto 3) > 512 then z_fa <= resize(s_g2(12 downto 3) - 512, 36); z_fb <= to_unsigned(4894, 16);
                        else z_fa <= resize(s_g2(12 downto 3), 36); z_fb <= to_unsigned(2442, 16); end if;
                    when 9  => z_fa <= resize(c_ak, 36);              z_fb <= to_unsigned(2719, 16);      -- swell taper
                    when 10 => z_fa <= resize(s_ys, 36);              z_fb <= c_ig;                       -- slab step
                    when 11 => z_fa <= resize(s_gp(12 downto 3), 36); z_fb <= to_unsigned(1640, 16);      -- shore
                    when 12 => z_fa <= resize(s_gp(12 downto 3), 36); z_fb <= to_unsigned(20, 16);
                    when 13 =>                                                                            -- etch (Osc 3x)
                        z_fa <= resize(s_g6(12 downto 3), 36);
                        if s_src = '1' then z_fb <= to_unsigned(450, 16); else z_fb <= to_unsigned(45, 16); end if;
                    when 14 => z_fa <= resize(s_gp(12 downto 3), 36); z_fb <= to_unsigned(123, 16);
                    when 15 => z_fa <= resize(s_gp(12 downto 3), 36); z_fb <= to_unsigned(850, 16);
                    when 16 =>                                                                            -- Grain's surf depth: 0 .. 1 .. 12x
                        z_fa <= resize(c_g5d, 36);
                        if c_g5hi = '1' then z_fb <= to_unsigned(88, 16); else z_fb <= to_unsigned(8, 16); end if;
                    when 20 => z_fa <= resize(c_a0, 36);              z_fb <= c_gd;                       -- surf depth x Grain
                    when others => z_fa <= resize(c_amax, 36);        z_fb <= c_gd;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- serial divider: 2^16 / ry, started at zc 200, 24 iterations
            if s_zone = '1' and s_zc = 200 then
                z_dd <= resize(c_ry, 10);
                z_dr <= (others => '0'); z_dq <= (others => '0'); z_dc <= to_unsigned(24, 5);
            elsif z_dc /= 0 then
                if z_dc = 17 then v_b := '1'; else v_b := '0'; end if;   -- the numerator 2^16's single 1
                v_r := z_dr(9 downto 0) & v_b;
                if v_r >= ('0' & z_dd) then
                    z_dr <= v_r - ('0' & z_dd); z_dq <= z_dq(9 downto 0) & '1';
                else
                    z_dr <= v_r; z_dq <= z_dq(9 downto 0) & '0';
                end if;
                z_dc <= z_dc - 1;
            end if;

            -- one shared exponent shifter: 2^(e/256) in Q12 from the EXP2 ROM (e = z_sh * 256 + address)
            case to_integer(z_sh) is
                when -3     => z_xq <= shift_right(r_ex_q, 6);
                when -2     => z_xq <= shift_right(r_ex_q, 5);
                when -1     => z_xq <= shift_right(r_ex_q, 4);
                when 0      => z_xq <= shift_right(r_ex_q, 3);
                when 1      => z_xq <= shift_right(r_ex_q, 2);
                when others => z_xq <= shift_right(r_ex_q, 1);
            end case;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20 =>                          -- the master phase moves on (last field's step)
                        if s_vneg = '1' then c_tau <= c_tau - c_spd; else c_tau <= c_tau + c_spd; end if;
                        if s_mir = '1' then c_xms <= to_unsigned(3840, 12) - resize(s_x0, 12);
                        else c_xms <= resize(s_x0, 12); end if;
                    when 21 =>
                        c_phi <= to_unsigned(48876, 16) - c_tau;
                        c_te  <= c_tau + to_unsigned(9830, 16);
                        -- spacing exponent: -(k1 - 512) * 0.75 (Q8 octaves)
                        v_d := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        z_e <= -(shift_right(v_d, 1) + shift_right(v_d, 2));
                    when 22 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= resize(shift_right(z_e, 8), 3);
                    when 25 =>
                        c_sp <= z_xq;
                        -- grain exponent: -(k5 - 512) * 1.5, clamped to -768 .. 767
                        v_d := signed(resize(s_g5(12 downto 3), 11)) - 512;
                        v_e := -(resize(v_d, 12) + resize(shift_right(v_d, 1), 12));
                        if v_e > 767 then v_e := to_signed(767, 12); elsif v_e < -768 then v_e := to_signed(-768, 12); end if;
                        z_e <= v_e(10 downto 0);
                    when 26 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= resize(shift_right(z_e, 8), 3);
                    when 29 => c_ig <= z_xq;
                    when 31 =>                          -- Grain: above / below the clip's 512
                        if s_g5(12 downto 3) > 512 then c_g5hi <= '1'; c_g5d <= s_g5(12 downto 3) - 512;
                        else c_g5hi <= '0'; c_g5d <= s_g5(12 downto 3); end if;
                    when 32 =>                          -- Grain > 512: slabs slip sideways, chance (k5 - 512) / 512
                        if c_g5hi = '1' then c_rjp <= c_g5d(8 downto 1); else c_rjp <= (others => '0'); end if;
                    when 30 =>                          -- hold step (Q4 virtual px) = 8 xs / grain = xs * ig >> 9
                        case to_integer(s_xs) is
                            when 4      => c_gst <= resize(shift_right(c_ig, 7), 8);
                            when 3      => c_gst <= resize(shift_right(resize(c_ig, 18) + shift_left(resize(c_ig, 18), 1), 9), 8);
                            when others => c_gst <= resize(shift_right(c_ig, 8), 8);
                        end case;
                    when 50  => z_m2  <= f_acc(17 downto 0);
                    when 74  => c_spd <= f_acc(31 downto 16);
                    when 98  => c_yl  <= to_unsigned(734, 11) + resize(f_acc(23 downto 16), 11);
                    when 122 => c_ey  <= to_unsigned(545, 11) + resize(f_acc(24 downto 16), 11);
                    when 146 =>                         -- the eye's peak lift: 1.8 te - 0.12 band (Q4 index), 0 .. 1843
                        v_u := f_acc(27 downto 16);
                        if v_u <= 246 then c_ea <= (others => '0');
                        elsif v_u > 2089 then c_ea <= to_unsigned(1843, 11);
                        else c_ea <= resize(v_u - 246, 11); end if;
                    when 170 =>
                        v_t := resize(f_acc(24 downto 16), 11) + 50;
                        if v_t > 140 then c_ry <= to_unsigned(140, 8); else c_ry <= v_t(7 downto 0); end if;
                    when 218 =>                          -- vf exponent = sign(d) (|d| 205 >> 8)
                        if s_g3(12 downto 3) >= 512 then z_e <= signed(resize(f_acc(17 downto 8), 11));
                        else z_e <= -signed(resize(f_acc(17 downto 8), 11)); end if;
                    when 219 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= resize(shift_right(z_e, 8), 3);
                    when 222 => c_vf <= z_xq;
                    when 226 => c_kr <= z_dq(10 downto 0);                       -- 2^16 / ry (divider done at 225)
                    when 242 =>
                        if s_g2(12 downto 3) > 512 then c_ak <= to_unsigned(2442, 13) + f_acc(21 downto 9);
                        else c_ak <= f_acc(21 downto 9); end if;
                    when 266 => c_ak7 <= f_acc(24 downto 6);
                    when 290 => c_ysg <= f_acc(25 downto 16);
                    when 314 => c_sh4 <= resize(shift_right(signed(f_acc(21 downto 0)) - to_signed(964690, 22), 9), 13);
                    when 338 => c_a0   <= to_unsigned(10, 6) + resize(f_acc(14 downto 10), 6);
                    when 362 => c_eg   <= f_acc(18 downto 8);
                    when 386 => c_amax <= to_unsigned(61, 8) + resize(f_acc(16 downto 10), 8);
                    when 410 => z_tr   <= to_unsigned(121, 10) + resize(f_acc(19 downto 10), 10);
                    when 434 =>
                        if c_g5hi = '1' then c_gd <= to_unsigned(4096, 16) + f_acc(15 downto 0);
                        else c_gd <= f_acc(15 downto 0); end if;
                    when 530 => c_a0s  <= f_acc(20 downto 12);
                    when 554 =>
                        if f_acc(23 downto 20) /= "0000" then c_ams <= (others => '1'); else c_ams <= f_acc(19 downto 12); end if;
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
    -- write of (luma >> 2) - 16, every other pixel
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
            s_lb_we <= data_in.avid and not s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_osum - 111);           -- signed about the lattice's mean (bipolar bend)
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17) and one serial divider (23 iterations); the beam and the l_*
    -- registers load its results at the next g_start
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_dr : unsigned(14 downto 0);
        variable v_v  : signed(9 downto 0);
        variable v_w  : signed(12 downto 0);
        variable v_i  : unsigned(7 downto 0);
        variable v_t  : unsigned(15 downto 0);
        variable v_c  : unsigned(11 downto 0);
        variable v_b  : std_logic;
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_slab <= (others => '0'); e_row <= (others => '0');
                e_tph <= "00"; e_rip <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- divider: G = 2^22 / Sh2 (12-bit quotient; the numerator's single 1 enters on the first step)
            if e_lc = 201 then
                e_dr <= (others => '0'); e_dq <= (others => '0');
            elsif e_lc > 201 and e_lc < 225 then
                if e_lc = 202 then v_b := '1'; else v_b := '0'; end if;
                v_dr := e_dr(13 downto 0) & v_b;
                if v_dr >= ('0' & e_s4) then
                    e_dr <= v_dr - ('0' & e_s4); e_dq <= e_dq(10 downto 0) & '1';
                else
                    e_dr <= v_dr; e_dq <= e_dq(10 downto 0) & '0';
                end if;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    e_y <= e_vy(18 downto 8);
                    e_slab <= e_slab - signed(resize(c_ysg, 12));
                    e_row  <= e_row - signed(resize(s_ys, 13));
                    e_drop <= '0'; e_slip <= (others => '0');
                when 2 =>
                    m_a <= resize(e_y, 32); m_b <= to_unsigned(124, 16); m_acc <= (others => '0');
                    if e_y <= 756 then e_yq <= e_y(9 downto 0); else e_yq <= to_unsigned(756, 10); end if;
                    e_dyl <= signed(resize(c_yl, 12)) - signed(resize(e_y, 12));
                    if e_y >= c_ey then e_dye <= e_y - c_ey; else e_dye <= c_ey - e_y; end if;
                    if e_row <= 0 then e_newr <= '1'; else e_newr <= '0'; end if;
                    e_r <= f_xs32(e_r);
                when 3 =>                       -- new slab (on noise-row boundaries)?
                    e_news <= '0';
                    if e_slab <= 0 and e_newr = '1' then
                        e_news <= '1';
                        e_sseed <= e_r(15 downto 1) & '1';
                        e_slab <= e_slab + signed(resize((to_unsigned(6, 6) + resize(e_r(19 downto 16), 6)
                                  + resize(e_r(21 downto 20), 6)) & x"0", 12));
                    end if;
                    e_r <= f_xs32(e_r);
                when 4 =>                       -- the slab's sideways slip (Grain > 512): 4 v, v = -256 .. 255 (Q4 index)
                    if e_news = '1' then
                        if e_r(16 downto 9) < c_rjp then
                            v_v := signed(resize(e_r(8 downto 0), 10)) - 256;
                            e_rj <= shift_left(resize(v_v, 12), 2);
                        else
                            e_rj <= (others => '0');
                        end if;
                    end if;
                    e_r <= f_xs32(e_r);
                when 5 =>                       -- new row (4 virtual rows)?
                    if e_newr = '1' then
                        e_rseed <= e_r(15 downto 1) & '1';
                        e_row <= e_row + 1024;
                        e_rip <= e_rip + to_unsigned(24248, 16);
                    end if;
                    e_r <= f_xs32(e_r);
                when 7 =>                       -- tears on the noise-row grid: a dropout row, then maybe 1..4 slipped rows
                    case e_tph is
                        when "01" =>
                            if e_newr = '1' then
                                if e_tsl /= 0 then e_tph <= "10"; e_tc <= e_tlen; e_slip <= e_tsl;
                                else e_tph <= "00"; end if;
                            else
                                e_drop <= '1';
                            end if;
                        when "10" =>
                            if e_newr = '1' then
                                if e_tc = 1 then e_tph <= "00"; else e_slip <= e_tsl; end if;
                                e_tc <= e_tc - 1;
                            else
                                e_slip <= e_tsl;
                            end if;
                        when others =>
                            if e_newr = '1' and e_r(15 downto 0) < resize(z_tr & "00", 16) then
                                e_tph <= "01"; e_drop <= '1';
                                if e_r(31 downto 24) < 154 then
                                    v_i := e_r(23 downto 16);
                                    v_v := signed(resize(v_i, 10)) - 128;
                                    v_w := shift_left(resize(v_v, 13), 5) - shift_left(resize(v_v, 13), 2);   -- 28 v (no self-add)
                                    e_tsl <= v_w;
                                else
                                    e_tsl <= (others => '0');
                                end if;
                                e_tlen <= to_unsigned(1, 3) + resize(e_r(29 downto 28), 3);
                            end if;
                    end case;
                    e_r <= f_xs32(e_r);
                -- vertical phase: P = Y (639132 + 124 Y) >> 12 (Q16 cycles), theta = vf P + phi
                when 19 => e_sc <= to_unsigned(639132, 20) + resize(m_acc(17 downto 0), 20);
                when 20 =>
                    m_a <= resize(e_sc, 32); m_b <= resize(e_y, 16); m_acc <= (others => '0');
                when 37 => e_sc <= resize(m_acc(29 downto 12), 20);
                when 38 =>
                    m_a <= resize(e_sc, 32); m_b <= c_vf; m_acc <= (others => '0');
                when 55 => e_th <= m_acc(27 downto 12) + c_phi;
                when 56 =>
                    v_i := e_th(13 downto 6);
                    if e_th(14) = '1' then v_i := not v_i; end if;
                    r_sn_a <= v_i; e_q <= e_th(15 downto 14);
                    m_a <= resize(e_yq, 32); m_b <= resize(e_yq, 16); m_acc <= (others => '0');
                when 58 => e_s <= r_sn_q(14 downto 0);
                when 73 => e_yq2 <= m_acc(19 downto 0);
                when 75 =>
                    m_a <= resize(c_ak7, 32); m_b <= resize(e_y, 16); m_acc <= (others => '0');
                when 92 => e_sc <= resize(c_ak - m_acc(28 downto 16), 20);
                when 93 =>
                    m_a <= resize(e_sc, 32); m_b <= resize(e_s, 16); m_acc <= (others => '0');
                when 110 =>
                    if e_q(1) = '1' then e_w4 <= -signed(resize(m_acc(27 downto 15), 14));
                    else e_w4 <= signed(resize(m_acc(27 downto 15), 14)); end if;
                -- ribbon centre X0 = M + wave (Q4 px) and spacing S (Q4 px)
                when 111 =>
                    m_a <= resize(e_yq2, 32); m_b <= to_unsigned(422, 16); m_acc <= (others => '0');
                when 128 => e_x04 <= to_signed(17005, 16) - signed(resize(m_acc(28 downto 16), 16));
                when 129 =>
                    m_a <= resize(e_y, 32); m_b <= to_unsigned(6632, 16); m_acc <= (others => '0');
                when 146 => e_x04 <= e_x04 - signed(resize(m_acc(23 downto 12), 16));
                when 147 =>
                    e_x04 <= e_x04 + resize(e_w4, 16);
                    m_a <= resize(e_yq2, 32); m_b <= to_unsigned(384, 16); m_acc <= (others => '0');
                when 164 => e_s4 <= to_unsigned(7043, 14) + resize(m_acc(28 downto 16), 14);
                when 165 =>
                    m_a <= resize(e_y, 32); m_b <= to_unsigned(6887, 16); m_acc <= (others => '0');
                when 182 => e_s4 <= e_s4 - resize(m_acc(23 downto 12), 14);
                when 183 =>
                    m_a <= resize(e_s4, 32); m_b <= c_sp; m_acc <= (others => '0');
                when 200 => e_s4 <= m_acc(26 downto 13);                   -- now Sh2, the divisor
                -- start value F = G (xms - X0h - slip) (Q20 band)
                when 225 =>
                    e_g <= e_dq;
                    e_d4 <= signed(resize(shift_left(resize(c_xms, 18), 4), 18)) - shift_left(resize(e_x04, 18), 1);
                when 226 => e_d4 <= e_d4 - resize(e_slip, 18);
                when 227 =>                     -- signed multiplicand, sign-extended: the product is exact mod 2^32
                    m_a <= unsigned(resize(e_d4, 32));
                    m_b <= resize(e_g, 16); m_acc <= (others => '0');
                when 244 =>
                    e_f20 <= signed(m_acc(30 downto 4));
                    e_c7 <= to_unsigned(64, 7) - unsigned(e_dyl(6 downto 0));
                -- the curl into the lagoon edge: (64 - d)^2 * 31 for 0 < d < 64
                when 245 =>
                    v_c := shift_left(resize(e_c7, 12), 5) - resize(e_c7, 12);
                    m_a <= resize(v_c, 32); m_b <= resize(e_c7, 16); m_acc <= (others => '0');
                when 262 =>
                    if s_lagoff = '0' and e_dyl > 0 and e_dyl < 64 then
                        e_f20 <= e_f20 + signed(resize(m_acc(17 downto 0), 27));
                    end if;
                    m_a <= resize(e_dye, 32); m_b <= resize(c_kr, 16); m_acc <= (others => '0');
                -- the eye: lift = ea (1 - t^2), t = |y - ey| / ry (Q8)
                when 279 =>
                    v_t := m_acc(23 downto 8);
                    if v_t >= 256 then e_ein <= '0'; e_t8 <= (others => '1');
                    else e_ein <= '1'; e_t8 <= v_t(7 downto 0); end if;
                when 280 =>
                    m_a <= resize(e_t8, 32); m_b <= resize(e_t8, 16); m_acc <= (others => '0');
                when 297 =>
                    m_a <= resize(c_ea, 32); m_b <= resize(m_acc(15 downto 8), 16); m_acc <= (others => '0');
                    case to_integer(s_xs) is
                        when 4      => e_step <= resize(shift_left(resize(e_g, 14), 2), 14);
                        when 3      => e_step <= resize(shift_left(resize(e_g, 14), 1), 14) + resize(e_g, 14);
                        when others => e_step <= resize(shift_left(resize(e_g, 14), 1), 14);
                    end case;
                    if e_y >= c_yl and s_lagoff = '0' then e_lag <= '1'; else e_lag <= '0'; end if;
                    -- the surf's field: held at each noise row's start (Q4 index), with the slab's row jump
                    if e_newr = '1' then e_fsd <= e_f20(26 downto 9) + resize(e_rj, 18); end if;
                when 298 => e_dfs <= e_fsd - e_f20(26 downto 9);
                when 299 => e_lsd <= resize(c_sh4, 18) - e_dfs;
                when 314 =>
                    if e_ein = '1' then e_lift <= c_ea - m_acc(18 downto 8); else e_lift <= (others => '0'); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: field DDA (f, Q20 band; the mirror reverses it left of the centre,
    -- the step sign registered a clock ahead from the next pixel's x)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x   <= resize(s_x0, 12);
                b_x1  <= resize(s_x0, 12) + s_xs;
                b_px  <= (others => '0');
                b_f   <= e_f20;  b_stp <= e_step;
                if s_mir = '1' and s_x0 < 1920 then b_neg <= '1'; else b_neg <= '0'; end if;
                l_dfs <= e_dfs; l_lsd <= e_lsd; l_lift <= e_lift; l_lag <= e_lag; l_drop <= e_drop; l_rip <= e_rip(15 downto 9);
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x1;
                b_x1 <= b_x1 + s_xs;
                if b_neg = '1' then b_f <= b_f - signed(resize(b_stp, 27));
                else b_f <= b_f + signed(resize(b_stp, 27)); end if;
                if s_mir = '1' and b_x1 < 1920 then b_neg <= '1'; else b_neg <= '0'; end if;
            end if;
            s_lb_ra <= b_px(10 downto 1);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Streak noise: xorshift16 per pixel, random holds (Q4 virtual px / grain),
    -- first-order slew; reseeded at each line start from the slab / row seeds,
    -- so every line of a slab (row) replays the same streaks.  The slab
    -- generator's low LFSR bits clock the surf's sample-and-hold (1 px in 4).
    ---------------------------------------------------------------------------
    p_noise : process(clk)
        variable v_rn : unsigned(15 downto 0);
        variable v_c2 : signed(12 downto 0);
        variable v_h  : unsigned(5 downto 0);
        variable v_df : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                n_r(1) <= (e_sseed xor x"3C3C") or x"0001";
                n_r(2) <= (e_rseed xor x"6996") or x"0001";
                for g in 1 to 2 loop n_cnt(g) <= (others => '0'); n_v(g) <= (others => '0'); n_tg(g) <= (others => '0'); end loop;
            elsif g_avid = '1' then
                for g in 1 to 2 loop
                    v_rn := f_xs16(n_r(g));
                    v_c2 := n_cnt(g) - signed(resize(c_gst, 13));
                    v_h := to_unsigned(12, 6) + resize(v_rn(4 downto 0), 6);
                    if v_c2 < 0 then
                        if g = 1 then n_cnt(g) <= v_c2 + signed(resize(v_h & "000000", 13));   -- slab streaks: 48 .. 172 virtual px
                        else n_cnt(g) <= v_c2 + signed(resize(v_h & "0000", 13)); end if;     -- row streaks: 12 .. 43
                        n_tg(g) <= signed(v_rn(15 downto 8));
                    else
                        n_cnt(g) <= v_c2;
                    end if;
                    v_df := resize(n_tg(g) & "00", 11) - resize(n_v(g), 11);
                    n_v(g) <= n_v(g) + resize(shift_right(v_df, 2), 10);
                    n_r(g) <= v_rn;
                end loop;
            end if;
        end if;
    end process p_noise;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n  : signed(10 downto 0);
        variable v_a  : signed(9 downto 0);
        variable v_d  : signed(17 downto 0);
        variable v_ic : unsigned(8 downto 0);
        variable v_r  : unsigned(6 downto 0);
        variable v_e  : signed(18 downto 0);
        variable v_dd : unsigned(7 downto 0);
        variable v_j  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: field in Q4 index units; the surf field's (this line's moved to the noise row's start,
            -- un-etched) distances to the shore; noise; the dash ripple (64 virtual px, phase per noise row)
            p2_f <= b_f(26 downto 9);
            v_d := shift_right(l_lsd - b_f(26 downto 9), 3);
            if v_d < 0 then p2_d8 <= (others => '0'); elsif v_d > 255 then p2_d8 <= (others => '1');
            else p2_d8 <= unsigned(v_d(7 downto 0)); end if;
            v_d := not v_d;                                  -- -d - 1: the distance on the open-water side
            if v_d < 0 then p2_e8 <= (others => '0'); elsif v_d > 255 then p2_e8 <= (others => '1');
            else p2_e8 <= unsigned(v_d(7 downto 0)); end if;
            p2_nf <= n_v(1)(9 downto 2);
            p2_nr <= n_v(2)(9 downto 2);
            v_r := b_x(6 downto 0) + l_rip;
            if v_r(6) = '1' then p2_tri <= not v_r(5); else p2_tri <= v_r(5); end if;
            -- 3: luma; noise sum; noise depth by distance to the shore
            p3_f <= p2_f; p3_e8 <= p2_e8; p3_nr <= p2_nr; p3_tri <= p2_tri;
            p3_lm <= unsigned(s_lb_q);
            v_n := resize(p2_nf, 11) + resize(shift_right(p2_nr, 2), 11);
            if v_n > 127 then p3_n8 <= to_signed(127, 8);
            elsif v_n < -127 then p3_n8 <= to_signed(-127, 8);
            else p3_n8 <= v_n(7 downto 0); end if;
            v_dd := '0' & p2_d8(7 downto 1);                 -- the depth ramp (its base and cap scale with Grain)
            if v_dd > c_ams then p3_an <= resize(c_a0s, 10) + resize(c_ams, 10);
            else p3_an <= resize(c_a0s, 10) + resize(v_dd, 10); end if;
            -- 4: noise offset (<< 5); dash amplitude
            p4_f <= p3_f; p4_e8 <= p3_e8; p4_n8 <= p3_n8; p4_tri <= p3_tri;
            if s_src = '1' then p4_lms <= resize(signed(p3_lm), 9); else p4_lms <= signed('0' & p3_lm); end if;
            p4_off <= signed('0' & p3_an(9 downto 2)) * p3_n8;
            v_a := resize(p3_nr, 10) + resize(shift_right(p3_nr, 2), 10) + resize(shift_right(p3_nr, 4), 10) + 38;
            if v_a < 0 then p4_a8 <= (others => '0'); elsif v_a > 255 then p4_a8 <= (others => '1');
            else p4_a8 <= unsigned(v_a(7 downto 0)); end if;
            -- 5: etch product (luma x gain, Q15 band); jittered field; dash level
            p5_n8 <= p4_n8; p5_tri <= p4_tri;
            p5_et  <= p4_lms * signed('0' & c_eg(10 downto 2));
            p5_fo  <= p4_f + resize(shift_right(p4_off, 5), 18);
            p5_lev <= signed(resize(p4_a8, 9)) - signed(resize(p4_e8, 9));
            -- 6: etch in Q4 index units (minus the Osc lattice mean); dash flags
            p6_n8 <= p5_n8; p6_fo <= p5_fo;
            p6_et <= resize(shift_right(p5_et, 2), 16);
            if p5_tri = '1' and p5_lev > 77 then p6_dash <= '1'; else p6_dash <= '0'; end if;
            if p5_lev > 140 then p6_dsh2 <= '1'; else p6_dsh2 <= '0'; end if;
            -- 7: etched field
            p7_n8 <= p6_n8; p7_dash <= p6_dash; p7_dsh2 <= p6_dsh2;
            p7_fin <= p6_fo + resize(p6_et, 18);
            -- 8: the surf's field; clean index; the eye's ramp (from band 1's white edge, f = 1.3, half a band
            -- per unit) and the field capped at the open water's 1.5
            p8_dash <= p7_dash; p8_dsh2 <= p7_dsh2;
            v_j := shift_left(resize(p7_n8, 11), 1) + resize(p7_n8, 11);      -- the shore's jitter (none in Grain's bottom quarter)
            if s_g5(12 downto 11) = "00" then p8_j <= (others => '0'); else p8_j <= v_j; end if;
            p8_sd  <= l_lsd - p7_fin;                        -- shore - the surf's field
            p8_ic0 <= resize(shift_right(p7_fin, 4), 15) + 128;
            v_d := shift_right(p7_fin - 2662, 2);
            if v_d <= 0 then p8_ramp <= (others => '0'); elsif v_d > 511 then p8_ramp <= (others => '1');
            else p8_ramp <= unsigned(v_d(8 downto 0)); end if;
            if p7_fin > 3072 then p8_fm <= to_signed(3072, 13);
            elsif p7_fin < 0 then p8_fm <= (others => '0');
            else p8_fm <= p7_fin(12 downto 0); end if;
            -- 9: surf index and key operand; clean wraps; the eye's lift
            p9_dash <= p8_dash; p9_dsh2 <= p8_dsh2; p9_fm <= p8_fm;
            p9_is0 <= resize(shift_right(p8_sd, 4), 15) + 64;
            p9_fk  <= p8_sd - resize(p8_j, 18);
            if p8_ic0 < 0 then v_ic := "00" & unsigned(p8_ic0(6 downto 0));
            elsif s_fold = '1' and p8_ic0 >= 320 then v_ic := to_unsigned(192, 9) + resize(unsigned(p8_ic0(6 downto 0)) - 64, 9);
            elsif p8_ic0 > 511 then v_ic := to_unsigned(511, 9);
            else v_ic := unsigned(p8_ic0(8 downto 0)); end if;
            p9_ic <= v_ic;
            p9_ep <= l_lift(10 downto 3) * p8_ramp(8 downto 1);
            if p8_ramp /= 0 and l_lift /= 0 and l_lag = '0' then p9_eye <= '1'; else p9_eye <= '0'; end if;
            -- 10: surf key; surf wraps; the lifted field
            p10_ic <= p9_ic; p10_eye <= p9_eye; p10_dash <= p9_dash; p10_dsh2 <= p9_dsh2;
            if p9_fk > 0 then p10_surf <= '1'; else p10_surf <= '0'; end if;
            if p9_is0 < 0 then p10_is <= (others => '0');
            elsif p9_is0 > 511 then p10_is <= "11" & unsigned(p9_is0(6 downto 0));
            else p10_is <= unsigned(p9_is0(8 downto 0)); end if;
            p10_fl <= resize(p9_fm, 14) + signed(resize(p9_ep(15 downto 4), 14));
            -- 11: the eye (read through the ribbon colours), the lagoon (dashes / cyan, or under Fold the bands
            -- slipped half a band) and the dropout lines
            p11_surf <= p10_surf; p11_is <= p10_is;
            v_ic := p10_ic;
            if p10_eye = '1' and p10_surf = '0' then
                if p10_fl < 3072 then v_ic := unsigned(p10_fl(12 downto 4)) + 128;
                else v_ic := unsigned(p10_fl(12 downto 4)); end if;
            end if;
            if l_lag = '1' and p10_surf = '0' then
                if s_fold = '1' then
                    if p10_ic >= 256 then v_ic := p10_ic - 64; else v_ic := p10_ic + 64; end if;
                elsif p10_dash = '1' then
                    if p10_dsh2 = '1' then v_ic := to_unsigned(261, 9); else v_ic := to_unsigned(271, 9); end if;
                else
                    v_ic := to_unsigned(335, 9);
                end if;
            end if;
            if l_drop = '1' and p10_surf = '0' then v_ic := to_unsigned(333, 9); end if;
            p11_ic <= v_ic;
            -- 12: bank and palette set; 13: address delay (keeps the 20-clock latency on 12 bits)
            if p11_surf = '1' then p12_ad <= s_pal & '1' & p11_is; else p12_ad <= s_pal & '0' & p11_ic; end if;
            p13_ad <= p12_ad;
            -- 14: palette address, 15: palette ROM (EBR), 16: fabric copy
            r_pa <= p13_ad;
            p16_q <= r_pq;
            -- 17: compose; neutral black outside the generated avid
            if s_gav_sr(14) = '0' then
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            else
                s_y17 <= unsigned(p16_q(31 downto 22)); s_u17 <= unsigned(p16_q(21 downto 12)); s_v17 <= unsigned(p16_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums, the history in the block-RAM ring), Y through [1 2 1]/4;
    -- luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to neutral (the off-avid input) once 8 blank samples are in, so a
            -- glitched sample can't offset the colours for good.  18: sums, 19: output
            s_bxd <= s_bxq;                                      -- fabric copy of the sample 8 back
            if s_bah = "00000000" then
                s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bau <= s_bau + s_u17 - unsigned(s_bxd(19 downto 10));
                s_bav <= s_bav + s_v17 - unsigned(s_bxd(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 14) + s_baud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v19 <= v_sc(13 downto 4);
            s_yh(1) <= s_y17;
            for i in 2 to 3 loop s_yh(i) <= s_yh(i - 1); end loop;
            v_y := resize(s_yh(1), 12) + shift_left(resize(s_yh(2), 12), 1) + resize(s_yh(3), 12);
            s_y19 <= v_y(11 downto 2);
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture breakers;
