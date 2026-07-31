-- daedalus.vhd  (v0.4 -- the picture drawn as a labyrinth)
--
-- Forked from VIDEOSKY, which cut the incoming video into a copper line-screen:
-- an accumulator pair P/Q gives a rotatable, scalable coordinate frame, and the
-- rules are the LEVEL SETS of P (`phase < thickness`).  Level sets can never
-- break, which is why VideoSky's Relief can bow them around the image.
--
-- Daedalus keeps the frame and throws away the level set.  A maze is not a
-- level set, it is a TILING -- so we use the part of the accumulator VideoSky
-- discarded:
--
--   * THE LATTICE IS ALREADY THERE.  acc is 24 bits with 8 fractional bits, so
--     acc(15 downto 8) is the phase within one period -- the SUB-CELL
--     coordinate -- and acc(23 downto 16) is the period INDEX -- the CELL id.
--     VideoSky only ever read the phase byte.  Taking both bytes turns the same
--     two adds per pixel into a full cell grid that rotates with Angle and
--     scales with Pitch for no extra cost.
--
--   * ONE CONTINUOUS LINE (the Truchet weave).  Two tiles, each joining all
--     four edge-midpoint ports IN PAIRS -- N<->W + S<->E, or its mirror
--     N<->E + S<->W.  Because every tile uses every port, each port is always
--     live and always has exactly ONE path arriving from each side: degree 2
--     everywhere, no branching possible, so the ink is a set of unbroken lines
--     that snake around each other at right angles and fill the plane with no
--     gaps and no sparse patches.  (v0.1 hashed each cell EDGE for liveness and
--     drew arms to the live ports.  It connected fine, but independent ports
--     mean degree 0..4 -- T-junctions, crosses, dead ends.  That is a network,
--     not a labyrinth.)
--
--     The paths are STAIRCASES, and they have to be.  With ports at the edge
--     midpoints, a single-turn L from N to W must turn either at the cell
--     CENTRE -- where it collides with the S<->E path -- or at the cell CORNER,
--     where the paths of four cells collide.  Physical Truchet tiles dodge this
--     with quarter-circle arcs that bulge toward the corner without touching
--     it; there is no right-angle equivalent of that arc.  So N<->W runs
--     (128,0)->(128,64)->(64,64)->(64,128)->(0,128), hugging the corner
--     diagonal but reaching neither corner nor centre, and S<->E is its 180
--     degree rotation.  Tile 1 is tile 0 MIRRORED IN u, so it costs one
--     subtract and a mux rather than a second set of segment tests.
--
--   * UNDER AND OVER (v0.3).  A third tile joins the ports STRAIGHT THROUGH --
--     N<->S and W<->E -- one bar crossing the other, and the under bar BREAKS
--     either side of the over bar and comes back out the far side.  Which bar
--     is on top alternates checkerboard-fashion with cell parity, so a line
--     running through several crossings goes over, under, over -- a woven
--     basket, the line visibly ducking under itself and re-emerging.  The
--     cross pairs all four ports like every other tile, so the degree-2 /
--     one-continuous-line guarantee survives.  Tangle drives the cross
--     fraction from the hash's next bit-field: at 0 the field is pure ordered
--     runs, at max nearly half the cells are crossings.
--
--   * MEANDER CELLS (v0.4).  The cells are 4x WIDER than they are tall: Q (the
--     column axis) steps at a quarter of P's rate, so every tile stroke that
--     runs along the wide axis is four times longer in pixels than a vertical
--     jog.  The same two staircases and the crossing then render as LONG
--     HORIZONTAL RUNS joined by short 90-degree drops -- a greek-key meander
--     locked to a grid, not a diagonal-reading texture.  Two consequences are
--     handled explicitly: line thickness is measured in sub-cell units, so
--     vertical strokes take a QUARTER width ((w+3)/4) to come out the same
--     thickness in pixels as horizontal ones; and Relief's displacement gets
--     the same /4 on the Q axis so the warp stays isotropic on screen.
--
--   * FRACTAL OCTAVES (v0.3, dialled back in v0.4).  An octave of the lattice is a BIT-SHIFT of the
--     same warped coordinate: shift the 16-bit integer part left by k and the
--     top byte is the cell id of a lattice 2^k times finer, the bottom byte its
--     sub-cell coordinate, already rescaled.  So up to four nested labyrinths
--     share one accumulator, one warp and one Relief, and each octave pays only
--     its own hash, mirror and eight segment compares.  The octaves are
--     OR-composited: fine ink inside coarse ink is invisible, so the finer
--     weave automatically fills the GAPS between the big lines.  Octave k
--     fades in (a per-frame width ramp, no pop) only while its cells are still
--     >= ~11 px, so at fine pitch there is one labyrinth, and as Pitch opens
--     out toward big spread-apart lines, successively finer labyrinths appear
--     in the space between them -- each with proportionally finer line weight.
--
--   * RELIEF WARPS THE DOMAIN, NOT THE PATTERN (P12, the headline).  VideoSky
--     added luma to the phase byte at the end.  Do that here and the tiles
--     displace independently and tear at every edge.  Instead the luma is added
--     to the FULL 16-bit integer part BEFORE it is split into cell id and
--     sub-cell, so the cell boundaries bow along with the strokes and
--     neighbours never disagree about where their shared edge is.  Where luma
--     is smooth, the labyrinth drapes; where luma FOLDS the coordinate you get
--     a mirrored crease (still continuous); where luma is DISCONTINUOUS it
--     tears, and no amount of arithmetic fixes that without a line buffer.
--     Top of the slider folds everywhere at once -- the climax.  The warp sits
--     BEFORE the octave split, so all four octaves drape as one surface.
--
-- Renderer: streaming, 10 stages, no BRAM, no line buffer.  TWO pixel-rate
-- multiplies (contrast, relief) -- one fewer than VideoSky, since the maze
-- needs no sine tap and one `rel` drives both axes.  The per-frame angle
-- resolve shares ONE registered 9x8 multiplier, split-operand across vblank
-- cycles, so no wide combinational product lands on a vsync-latched register
-- (the ziffern trap: those paths are still timed).
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture daedalus of program_top is

    constant C_LATENCY : integer := 10;
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Solid ink above this tone.  The blackwork masses (S7); K1 Ink slides the
    -- whole ramp into and out of it, so it needs no knob of its own.
    constant C_BLACKOUT : unsigned(7 downto 0) := to_unsigned(224, 8);

    -- Parallel staircase segments sit 64 apart in sub-cell units, so the arm
    -- half-width has to stay under 32 or neighbouring runs merge into mush.
    constant C_WMAX : integer := 26;

    -- Nested lattice octaves.  Octave k is 2^k times finer than the base.
    -- Three, not four: the 16 px octave was the densest and its crossing gaps
    -- were at the aliasing floor (the v0.4 dial-back).
    constant C_NOCT : integer := 3;

    -- Octave k is fully present while step*2^k is below ~4500 (cells >= ~15 px)
    -- and fades to nothing as step*2^k reaches 6144 (cells ~11 px): the width
    -- cap ramps (6144 - step<<k)/64, clamped to C_WMAX.
    constant C_OCT_LIM : integer := 6144;

    -- One seed per octave, so the nested tile fields are uncorrelated.
    type t_seeds is array(0 to C_NOCT - 1) of unsigned(15 downto 0);
    constant C_SEEDS : t_seeds := (x"9E37", x"7C15", x"3A9D");

    ----------------------------------------------------------------------
    -- quarter-wave folded sine, 64-entry logic ROM (intaglio lineage).
    -- Used ONLY by the vblank angle resolve -- the pixel path has no sine tap.
    ----------------------------------------------------------------------
    type t_qsin is array(0 to 63) of signed(9 downto 0);
    constant C_QSIN : t_qsin := (
        to_signed(  0, 10), to_signed( 13, 10), to_signed( 25, 10), to_signed( 38, 10), to_signed( 50, 10), to_signed( 63, 10), to_signed( 75, 10), to_signed( 87, 10),
        to_signed(100, 10), to_signed(112, 10), to_signed(124, 10), to_signed(136, 10), to_signed(148, 10), to_signed(160, 10), to_signed(172, 10), to_signed(184, 10),
        to_signed(196, 10), to_signed(207, 10), to_signed(218, 10), to_signed(230, 10), to_signed(241, 10), to_signed(252, 10), to_signed(263, 10), to_signed(273, 10),
        to_signed(284, 10), to_signed(294, 10), to_signed(304, 10), to_signed(314, 10), to_signed(324, 10), to_signed(334, 10), to_signed(343, 10), to_signed(352, 10),
        to_signed(361, 10), to_signed(370, 10), to_signed(379, 10), to_signed(387, 10), to_signed(395, 10), to_signed(403, 10), to_signed(410, 10), to_signed(418, 10),
        to_signed(425, 10), to_signed(432, 10), to_signed(438, 10), to_signed(445, 10), to_signed(451, 10), to_signed(456, 10), to_signed(462, 10), to_signed(467, 10),
        to_signed(472, 10), to_signed(477, 10), to_signed(481, 10), to_signed(485, 10), to_signed(489, 10), to_signed(492, 10), to_signed(496, 10), to_signed(499, 10),
        to_signed(501, 10), to_signed(503, 10), to_signed(505, 10), to_signed(507, 10), to_signed(509, 10), to_signed(510, 10), to_signed(510, 10), to_signed(511, 10));

    function f_qsin(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then
            v_i := a(7 downto 2);
        else
            v_i := not a(7 downto 2);
        end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then
            return -v_t;
        else
            return v_t;
        end if;
    end function;

    ----------------------------------------------------------------------
    -- Cell hash -> the tile bit.  Split across two pipeline stages so the mix
    -- chain never lands on one critical path.  Every step is a bijection on
    -- 16 bits (odd multiplies, and xor-with-own-high-bits), so distinct cells
    -- never collide and the TOP bits are the well-mixed ones -- which is what
    -- the Tangle threshold reads.  Shift-and-ADD, not shift-and-xor: xor alone
    -- is linear over GF(2) and leaves visible lattice structure (the starfield
    -- hash-bit-bias lesson); the carries are the nonlinearity.
    ----------------------------------------------------------------------
    function f_mix_a(c1, c2 : unsigned(7 downto 0); seed : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := (c1 & c2) xor seed;
        v := v + shift_left(v, 3);          -- *9
        v := v xor shift_right(v, 7);
        return v;
    end function;

    function f_mix_b(a : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := a + shift_left(a, 5);          -- *33
        v := v xor shift_right(v, 9);
        v := v + shift_left(v, 2);          -- *5
        return v;
    end function;

    -- Clamp into the legal 10-bit luma swing.  Returns UNSIGNED on purpose:
    -- unsigned(resize(v,10)) of a signed value saturates at the SIGNED range,
    -- so anything >= 512 wraps to black (the penrose lesson).
    function f_cy(v : signed) return unsigned is
    begin
        if    v < 16   then return to_unsigned(16, 10);
        elsif v > 1008 then return to_unsigned(1008, 10);
        else                return resize(unsigned(v(9 downto 0)), 10);
        end if;
    end function;

    -- Clamp a tone into 0..255.
    function f_ct(v : signed) return unsigned is
    begin
        if    v < 0   then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else               return resize(unsigned(v(7 downto 0)), 8);
        end if;
    end function;

    -- Clamp an arm half-width into 0..C_WMAX.
    function f_cw(v : signed) return unsigned is
    begin
        if    v < 0      then return to_unsigned(0, 5);
        elsif v > C_WMAX then return to_unsigned(C_WMAX, 5);
        else                  return resize(unsigned(v(4 downto 0)), 5);
        end if;
    end function;

    -- |a - b| on 8-bit sub-cell coordinates.
    function f_ad(a : unsigned(7 downto 0); b : integer) return unsigned is
        variable v_b : unsigned(7 downto 0);
    begin
        v_b := to_unsigned(b, 8);
        if a >= v_b then return a - v_b; else return v_b - a; end if;
    end function;

    ----------------------------------------------------------------------
    -- per-octave signal bundles
    ----------------------------------------------------------------------
    type t_u8a  is array(0 to C_NOCT - 1) of unsigned(7 downto 0);
    type t_u16a is array(0 to C_NOCT - 1) of unsigned(15 downto 0);
    type t_u5a  is array(0 to C_NOCT - 1) of unsigned(4 downto 0);

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_k1_ink   : unsigned(9 downto 0) := to_unsigned(512, 10);   -- K1 Ink
    signal s_k2_con   : unsigned(9 downto 0) := to_unsigned(256, 10);   -- K2 Contrast
    signal s_k3_pit   : unsigned(9 downto 0) := to_unsigned(400, 10);   -- K3 Pitch
    signal s_k4_ang   : unsigned(9 downto 0) := (others => '0');        -- K4 Angle
    signal s_k5_tan   : unsigned(9 downto 0) := to_unsigned(1023, 10);  -- K5 Tangle
    signal s_scheme   : unsigned(9 downto 0) := (others => '0');        -- K6 Scheme
    signal s_p12      : unsigned(9 downto 0) := (others => '0');        -- P12 Relief
    signal s_black    : std_logic := '0';                               -- S7  Blackout
    signal s_invert   : std_logic := '0';                               -- S8  Invert
    signal s_tint     : std_logic := '0';                               -- S9  Tint
    signal s_wmod     : std_logic := '0';                               -- S10 Width
    signal s_drift    : std_logic := '0';                               -- S11 Drift

    ----------------------------------------------------------------------
    -- per-frame resolved terms
    ----------------------------------------------------------------------
    signal s_cos8, s_sin8 : signed(7 downto 0) := (others => '0');
    signal s_step   : unsigned(14 downto 0) := to_unsigned(6912, 15);
    signal s_dpx, s_dpy : signed(17 downto 0) := (others => '0');
    signal s_dqx, s_dqy : signed(17 downto 0) := (others => '0');
    signal s_bias   : signed(9 downto 0)  := (others => '0');   -- Ink, +/-256
    signal s_gain   : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s_depth6 : unsigned(5 downto 0) := (others => '0');
    signal s_tang6  : unsigned(5 downto 0) := to_unsigned(31, 6);
    signal s_crs6   : unsigned(5 downto 0) := to_unsigned(15, 6);
    signal s_fixedw : unsigned(4 downto 0) := to_unsigned(9, 5);

    -- Per-octave width caps: the fade-in ramp.  Octave 0 is always fully
    -- present; only 1..C_NOCT-1 are written by the sequencer.
    signal s_wcap : t_u5a := (others => to_unsigned(C_WMAX, 5));

    signal s_ink_y, s_ink_u, s_ink_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_pap_y, s_pap_u, s_pap_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- vblank sequencer: one shared, fully registered 9x8 multiplier
    ----------------------------------------------------------------------
    signal s_seq   : unsigned(4 downto 0) := (others => '0');
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');
    signal s_qs_r  : signed(9 downto 0)  := (others => '0');
    signal s_ma    : signed(8 downto 0) := (others => '0');
    signal s_mb    : signed(7 downto 0) := (others => '0');
    signal s_mp    : signed(16 downto 0) := (others => '0');
    signal s_pah_c, s_pal_c : signed(16 downto 0) := (others => '0');
    signal s_pah_s, s_pal_s : signed(16 downto 0) := (others => '0');
    signal s_prod_c, s_prod_s : signed(24 downto 0) := (others => '0');

    signal s_prev_vsync_n : std_logic := '1';
    signal s_vs_pulse : std_logic := '0';
    signal s_fpar  : std_logic := '0';
    signal s_ilace : std_logic := '0';
    signal s_dracc : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- screen-coordinate accumulators.  24 bits with 8 fractional bits.  The
    -- integer part acc(23 downto 8) splits into CELL = acc(23 downto 16) and
    -- SUB-CELL = acc(15 downto 8).  A 24-bit wrap is a multiple of 2^16, so
    -- both bytes are continuous across the wrap and the width is free.
    ----------------------------------------------------------------------
    signal s_accp, s_accq : signed(23 downto 0) := (others => '0');
    signal s_linp, s_linq : signed(23 downto 0) := (others => '0');
    signal s_avid_p : std_logic := '0';

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    signal r1_ydiff : signed(10 downto 0) := (others => '0');
    signal r1_gain  : signed(7 downto 0)  := (others => '0');
    signal r1_pi, r1_qi : unsigned(15 downto 0) := (others => '0');

    signal r2_yp    : signed(18 downto 0) := (others => '0');
    signal r2_pi, r2_qi : unsigned(15 downto 0) := (others => '0');

    signal r3_y8    : unsigned(7 downto 0) := (others => '0');
    signal r3_pi, r3_qi : unsigned(15 downto 0) := (others => '0');

    signal r4_rel   : signed(11 downto 0) := (others => '0');
    signal r4_t8    : signed(9 downto 0)  := (others => '0');
    signal r4_pi, r4_qi : unsigned(15 downto 0) := (others => '0');

    signal r5_cx, r5_cy : t_u8a := (others => (others => '0'));
    signal r5_u,  r5_v  : t_u8a := (others => (others => '0'));
    signal r5_t         : unsigned(7 downto 0) := (others => '0');

    signal r6_h       : t_u16a := (others => (others => '0'));
    signal r6_u, r6_v : t_u8a := (others => (others => '0'));
    signal r6_par     : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r6_t       : unsigned(7 downto 0) := (others => '0');

    signal r7_tile    : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r7_cross   : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r7_par     : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r7_u, r7_v : t_u8a := (others => (others => '0'));
    signal r7_w       : t_u5a := (others => (others => '0'));
    signal r7_solid   : std_logic := '0';

    signal r8_uu, r8_v : t_u8a := (others => (others => '0'));
    signal r8_a64, r8_a128, r8_a192 : t_u8a := (others => (others => '0'));
    signal r8_b64, r8_b128, r8_b192 : t_u8a := (others => (others => '0'));
    signal r8_w, r8_wv : t_u5a := (others => (others => '0'));
    signal r8_gap, r8_gapv : t_u8a := (others => (others => '0'));
    signal r8_cross    : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r8_par      : std_logic_vector(0 to C_NOCT - 1) := (others => '0');
    signal r8_solid    : std_logic := '0';

    signal r9_ink : std_logic := '0';

    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- chroma delay to the colour resolve
    type t_c is array(0 to C_LATENCY - 2) of unsigned(9 downto 0);
    signal s_ud, s_vd : t_c := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared qsin port: the sequencer takes the cos and sin taps two cycles
    -- after presenting each angle.  Address and ROM output both registered.
    ------------------------------------------------------------------------
    s_qs_a <= s_k4_ang + to_unsigned(256, 10) when s_seq = 31 else s_k4_ang;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_t  : signed(11 downto 0);
        variable v_sk : unsigned(17 downto 0);
        variable v_c  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);
            s_mp    <= s_ma * s_mb;

            s_vs_pulse <= '0';

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_vs_pulse <= '1';

                s_k1_ink <= unsigned(registers_in(0));
                s_k2_con <= unsigned(registers_in(1));
                s_k3_pit <= unsigned(registers_in(2));
                s_k4_ang <= unsigned(registers_in(3));
                s_k5_tan <= unsigned(registers_in(4));
                s_scheme <= unsigned(registers_in(5));
                s_p12    <= unsigned(registers_in(7));
                s_black  <= registers_in(6)(0);      -- S7
                s_invert <= registers_in(6)(1);      -- S8
                s_tint   <= registers_in(6)(2);      -- S9
                s_wmod   <= registers_in(6)(3);      -- S10
                s_drift  <= registers_in(6)(4);      -- S11

                -- interlace detect: did the field parity toggle since last vsync?
                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then
                    s_ilace <= '1';
                else
                    s_ilace <= '0';
                end if;

                if registers_in(6)(4) = '1' then
                    s_dracc <= s_dracc + 2;
                end if;

                s_seq <= to_unsigned(31, 5);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    -- 31/30 present the cos and sin angles into the qsin port.
                    when 30 =>
                        -- Pitch is linear in FREQUENCY: 512 (cell ~128 px) up to
                        -- 16880 (cell ~4 px).  Phase unit 256 = one cell,
                        -- 8 fractional bits.
                        s_step <= to_unsigned(512, 15)
                                  + shift_left(resize(s_k3_pit, 15), 4);

                    when 29 => s_cos8 <= resize(shift_right(s_qs_r, 2), 8);
                    when 28 => s_sin8 <= resize(shift_right(s_qs_r, 2), 8);

                    -- step*cos and step*sin, split-operand through the one
                    -- registered multiplier: step = sh*256 + sl.
                    when 27 => s_ma <= signed('0' & s_step(14 downto 8) & '0');
                               s_mb <= s_cos8;
                    when 26 => s_ma <= signed('0' & s_step(7 downto 0));
                               s_mb <= s_cos8;
                    when 25 => s_pah_c <= s_mp;
                    when 24 => s_pal_c <= s_mp;
                    when 23 => s_ma <= signed('0' & s_step(14 downto 8) & '0');
                               s_mb <= s_sin8;
                    when 22 => s_ma <= signed('0' & s_step(7 downto 0));
                               s_mb <= s_sin8;
                    when 21 => s_pah_s <= s_mp;
                    when 20 => s_pal_s <= s_mp;

                    -- sh was pre-shifted left by 1 above, so its partial lands
                    -- at <<7 not <<8; the two halves then recombine.
                    when 19 =>
                        s_prod_c <= shift_left(resize(s_pah_c, 25), 7)
                                    + resize(s_pal_c, 25);
                        s_prod_s <= shift_left(resize(s_pah_s, 25), 7)
                                    + resize(s_pal_s, 25);

                    -- Angle 0 must give an upright lattice, so cos drives the
                    -- per-LINE step and sin the per-pixel one.
                    when 18 =>
                        s_dpy <= resize(shift_right(s_prod_c, 7), 18);
                        s_dpx <= resize(shift_right(s_prod_s, 7), 18);

                    -- Q is P rotated 90 degrees, at a QUARTER of the rate: the
                    -- cells come out 4x wider than tall, which is what turns
                    -- the staircases into long runs with short jogs.
                    when 17 =>
                        s_dqx <= -resize(shift_right(s_dpy, 2), 18);
                        s_dqy <=  resize(shift_right(s_dpx, 2), 18);

                    when 16 =>      -- Ink: a bias on the whole tone ramp
                        s_bias <= signed(resize(shift_right(s_k1_ink, 1), 10))
                                  - to_signed(256, 10);

                    when 15 =>      -- Contrast: 1.0x .. 4.9x about mid-grey
                        s_gain <= to_unsigned(16, 7) + resize(s_k2_con(9 downto 4), 7);

                    when 14 => s_depth6 <= s_p12(9 downto 4);

                    -- Tangle: the tile-mirror probability, 0..31 out of 64.
                    -- At 0 every cell takes tile 0 and the staircases chain into
                    -- long ordered diagonal runs; at max it is a coin flip per
                    -- cell and the weave is maximally knotted.  Past 1/2 the
                    -- pattern is just the mirror of below it, so the knob stops
                    -- there and every position does something.
                    when 13 =>
                        s_tang6 <= resize(s_k5_tan(9 downto 5), 6);
                        -- crossings at HALF the mirror rate (max ~24% of
                        -- cells) -- the v0.4 density dial-back
                        s_crs6  <= resize(s_k5_tan(9 downto 6), 6);

                    when 12 =>      -- constant line weight when Width is Fixed
                        v_t := to_signed(9, 12) + resize(shift_right(s_bias, 5), 12);
                        s_fixedw <= f_cw(v_t);

                    when 11 =>      -- print scheme (K6)
                        if s_scheme < 102 then                  -- Parchment
                            s_pap_y <= to_unsigned(740, 10);
                            s_pap_u <= to_unsigned(470, 10); s_pap_v <= to_unsigned(540, 10);
                            s_ink_y <= to_unsigned(30, 10);
                            s_ink_u <= to_unsigned(495, 10); s_ink_v <= to_unsigned(522, 10);
                        elsif s_scheme < 307 then               -- Gallery
                            s_pap_y <= to_unsigned(830, 10);
                            s_pap_u <= C_CHROMA_MID;         s_pap_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(40, 10);
                            s_ink_u <= C_CHROMA_MID;         s_ink_v <= C_CHROMA_MID;
                        elsif s_scheme < 511 then               -- Night
                            s_pap_y <= to_unsigned(120, 10);
                            s_pap_u <= C_CHROMA_MID;         s_pap_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(750, 10);
                            s_ink_u <= C_CHROMA_MID;         s_ink_v <= C_CHROMA_MID;
                        elsif s_scheme < 716 then               -- Blueprint
                            s_pap_y <= to_unsigned(300, 10);
                            s_pap_u <= to_unsigned(700, 10); s_pap_v <= to_unsigned(450, 10);
                            s_ink_y <= to_unsigned(850, 10);
                            s_ink_u <= to_unsigned(500, 10); s_ink_v <= to_unsigned(506, 10);
                        elsif s_scheme < 920 then               -- Cyanotype
                            s_pap_y <= to_unsigned(200, 10);
                            s_pap_u <= to_unsigned(640, 10); s_pap_v <= to_unsigned(470, 10);
                            s_ink_y <= to_unsigned(880, 10);
                            s_ink_u <= to_unsigned(520, 10); s_ink_v <= to_unsigned(505, 10);
                        else                                    -- Copper
                            s_pap_y <= to_unsigned(800, 10);
                            s_pap_u <= to_unsigned(480, 10); s_pap_v <= to_unsigned(530, 10);
                            s_ink_y <= to_unsigned(300, 10);
                            s_ink_u <= to_unsigned(430, 10); s_ink_v <= to_unsigned(600, 10);
                        end if;

                    -- Octave fade ramps.  All three subtracts are shallow cones
                    -- off the registered s_step, but they are still STA-timed,
                    -- so they get their own state rather than piggybacking.
                    when 10 =>
                        for k in 1 to C_NOCT - 1 loop
                            v_sk := shift_left(resize(s_step, 18), k);
                            if v_sk >= to_unsigned(C_OCT_LIM, 18) then
                                s_wcap(k) <= (others => '0');
                            else
                                v_c := resize(shift_right(to_unsigned(C_OCT_LIM, 18) - v_sk, 6), 12);
                                if v_c > C_WMAX then
                                    s_wcap(k) <= to_unsigned(C_WMAX, 5);
                                else
                                    s_wcap(k) <= resize(v_c, 5);
                                end if;
                            end if;
                        end loop;

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- Screen accumulators.  Origin at the top-left active pixel; Drift walks
    -- the frame origin so the lattice crawls.  On an interlaced source each
    -- field starts half a line step down so the two fields interleave.
    -- Advance on ACTIVE lines only -- a blanking-hsync advance makes the
    -- phase field-dependent and the whole lattice twitters at 30 Hz.
    ------------------------------------------------------------------------
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            if s_vs_pulse = '1' then
                if s_ilace = '1' and data_in.field_n = '1' then
                    s_linp <= shift_left(resize(signed('0' & s_dracc), 24), 8)
                              + shift_right(resize(s_dpy, 24), 1);
                    s_linq <= shift_right(resize(s_dqy, 24), 1);
                else
                    s_linp <= shift_left(resize(signed('0' & s_dracc), 24), 8);
                    s_linq <= (others => '0');
                end if;
            elsif data_in.avid = '1' and s_avid_p = '0' then
                s_accp <= s_linp;
                s_accq <= s_linq;
                s_linp <= s_linp + resize(s_dpy, 24);
                s_linq <= s_linq + resize(s_dqy, 24);
            elsif data_in.avid = '1' then
                s_accp <= s_accp + resize(s_dpx, 24);
                s_accq <= s_accq + resize(s_dqx, 24);
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- R1: capture.  The contrast multiply's operands are registered here.
    -- The accumulators' INTEGER parts come across whole -- cell id and
    -- sub-cell both, since Relief has to displace them together.
    ------------------------------------------------------------------------
    p_r1 : process(clk)
    begin
        if rising_edge(clk) then
            r1_ydiff <= signed('0' & unsigned(data_in.y)) - to_signed(512, 11);
            r1_gain  <= signed('0' & s_gain);
            r1_pi    <= unsigned(s_accp(23 downto 8));
            r1_qi    <= unsigned(s_accq(23 downto 8));
            s_ud(0)  <= unsigned(data_in.u);
            s_vd(0)  <= unsigned(data_in.v);
            for i in 1 to C_LATENCY - 2 loop
                s_ud(i) <= s_ud(i - 1);
                s_vd(i) <= s_vd(i - 1);
            end loop;
        end if;
    end process p_r1;

    ------------------------------------------------------------------------
    -- R2: contrast multiply.
    ------------------------------------------------------------------------
    p_r2 : process(clk)
    begin
        if rising_edge(clk) then
            r2_yp <= r1_ydiff * r1_gain;
            r2_pi <= r1_pi;
            r2_qi <= r1_qi;
        end if;
    end process p_r2;

    ------------------------------------------------------------------------
    -- R3: clamp the stretched luma back into the legal swing and take the
    -- top 8 bits as the working tone.
    ------------------------------------------------------------------------
    p_r3 : process(clk)
        variable v_y : signed(13 downto 0);
        variable v_c : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_y := to_signed(512, 14) + resize(shift_right(r2_yp, 4), 14);
            v_c := f_cy(v_y);
            r3_y8 <= v_c(9 downto 2);
            r3_pi <= r2_pi;
            r3_qi <= r2_qi;
        end if;
    end process p_r3;

    ------------------------------------------------------------------------
    -- R4: the one shaping multiply.
    --   rel = luma * Relief -- up to ~4 cells of displacement
    -- and the raw ink tone: 255 = black = full ink.
    ------------------------------------------------------------------------
    p_r4 : process(clk)
    begin
        if rising_edge(clk) then
            r4_rel <= resize(shift_right(signed('0' & r3_y8) * signed('0' & s_depth6), 4), 12);
            r4_t8  <= to_signed(255, 10) - signed(resize(r3_y8, 10));
            r4_pi  <= r3_pi;
            r4_qi  <= r3_qi;
        end if;
    end process p_r4;

    ------------------------------------------------------------------------
    -- R5: THE WARP, then the octave split.  rel lands on the full 16-bit
    -- integer coordinate BEFORE any splitting, so the cell boundaries of
    -- EVERY octave bow together and adjacent cells always agree on where
    -- their shared edge is.  Then octave k is a bit-shift of the warped
    -- coordinate: shift left by k and the top byte is the k-th octave's cell
    -- id, the bottom byte its sub-cell coordinate, already rescaled to
    -- 0..255.  Both adds wrap mod 2^16, which is exactly right: the cell id
    -- is only ever used as hash input.
    ------------------------------------------------------------------------
    p_r5 : process(clk)
        variable v_wp, v_wq : unsigned(15 downto 0);
        variable v_sp, v_sq : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- Q's displacement is /4 to match its /4 step: the warp stays
            -- isotropic in PIXELS even though the cells are anisotropic.
            v_wp := r4_pi + unsigned(resize(r4_rel, 16));
            v_wq := r4_qi + unsigned(resize(shift_right(r4_rel, 2), 16));
            for k in 0 to C_NOCT - 1 loop
                v_sp := shift_left(v_wp, k);
                v_sq := shift_left(v_wq, k);
                r5_cx(k) <= v_sp(15 downto 8);
                r5_u(k)  <= v_sp(7 downto 0);
                r5_cy(k) <= v_sq(15 downto 8);
                r5_v(k)  <= v_sq(7 downto 0);
            end loop;
            r5_t <= f_ct(r4_t8 + resize(s_bias, 10));
        end if;
    end process p_r5;

    ------------------------------------------------------------------------
    -- R6: hash round A on each octave's cell id, each with its own seed so
    -- the nested tile fields are uncorrelated.  The parity bit is the
    -- checkerboard that decides which bar of a crossing goes over.
    ------------------------------------------------------------------------
    p_r6 : process(clk)
    begin
        if rising_edge(clk) then
            for k in 0 to C_NOCT - 1 loop
                r6_h(k)   <= f_mix_a(r5_cx(k), r5_cy(k), C_SEEDS(k));
                r6_par(k) <= r5_cx(k)(0) xor r5_cy(k)(0);
                r6_u(k)   <= r5_u(k);
                r6_v(k)   <= r5_v(k);
            end loop;
            r6_t <= r5_t;
        end if;
    end process p_r6;

    ------------------------------------------------------------------------
    -- R7: hash round B -> each octave's tile bit, and settle the per-octave
    -- line weights.  The base weight (Fixed, or tone-modulated so the
    -- shadows fatten the line) is shifted up one per octave -- in sub-cell
    -- units, which HALVES in pixels per octave, so the pixel weight would
    -- stay constant -- then clamped by the octave's fade cap.  The cap does
    -- double duty: it ramps a newly-enabled octave in smoothly, and it holds
    -- every octave under C_WMAX so parallel runs never merge.  A capped-out
    -- octave has width 0 and the segment tests below can never fire.
    ------------------------------------------------------------------------
    p_r7 : process(clk)
        variable v_h  : unsigned(15 downto 0);
        variable v_tw : unsigned(4 downto 0);
        variable v_wb : unsigned(4 downto 0);
        variable v_ws : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if s_wmod = '0' then
                v_wb := s_fixedw;
            else
                v_tw := r6_t(7 downto 3);
                if v_tw > C_WMAX then
                    v_wb := to_unsigned(C_WMAX, 5);
                else
                    v_wb := v_tw;
                end if;
            end if;

            for k in 0 to C_NOCT - 1 loop
                v_h := f_mix_b(r6_h(k));
                if v_h(15 downto 10) < s_tang6 then
                    r7_tile(k) <= '1';
                else
                    r7_tile(k) <= '0';
                end if;

                -- The next bit-field of the same hash, at half the mirror
                -- rate: Tangle raises mirrors and crossings together, so 0
                -- stays pure ordered runs and max is a knotted basket.
                if v_h(9 downto 4) < s_crs6 then
                    r7_cross(k) <= '1';
                else
                    r7_cross(k) <= '0';
                end if;

                v_ws := shift_left(resize(v_wb, 8), k);
                if v_ws > resize(s_wcap(k), 8) then
                    r7_w(k) <= s_wcap(k);
                else
                    r7_w(k) <= resize(v_ws, 5);
                end if;

                r7_par(k) <= r6_par(k);
                r7_u(k)   <= r6_u(k);
                r7_v(k)   <= r6_v(k);
            end loop;

            -- Blackout: the shadows go to solid ink, no maze.  This is what
            -- makes it read as blackwork rather than as a texture filter.
            if r6_t > C_BLACKOUT and s_black = '1' then
                r7_solid <= '1';
            else
                r7_solid <= '0';
            end if;
        end if;
    end process p_r7;

    ------------------------------------------------------------------------
    -- R8: mirror and measure, per octave.  Tile 1 is tile 0 reflected in u,
    -- so ONE subtract buys the second tile and the segment tests below never
    -- need to know which tile they are drawing.
    ------------------------------------------------------------------------
    p_r8 : process(clk)
        variable v_uu : unsigned(7 downto 0);
        variable v_wv : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            for k in 0 to C_NOCT - 1 loop
                if r7_tile(k) = '1' then
                    v_uu := to_unsigned(255, 8) - r7_u(k);
                else
                    v_uu := r7_u(k);
                end if;

                -- Vertical strokes take a quarter width (rounded up): their
                -- thickness is measured across the 4x-wide axis, so this is
                -- what makes them come out the SAME thickness in pixels as
                -- the horizontal runs.
                v_wv := resize(shift_right(resize(r7_w(k), 8) + 3, 2), 5);

                r8_uu(k)   <= v_uu;
                r8_v(k)    <= r7_v(k);
                r8_a64(k)  <= f_ad(v_uu,     64);
                r8_a128(k) <= f_ad(v_uu,    128);
                r8_a192(k) <= f_ad(v_uu,    192);
                r8_b64(k)  <= f_ad(r7_v(k),  64);
                r8_b128(k) <= f_ad(r7_v(k), 128);
                r8_b192(k) <= f_ad(r7_v(k), 192);
                r8_w(k)    <= r7_w(k);
                r8_wv(k)   <= v_wv;

                -- The under bar stops this far from the over bar's centre
                -- line, per axis: one full width of the over bar clear each
                -- side, plus a hair.
                r8_gap(k)   <= shift_left(resize(r7_w(k), 8), 1) + 4;
                r8_gapv(k)  <= shift_left(resize(v_wv, 8), 1) + 2;
                r8_cross(k) <= r7_cross(k);
                r8_par(k)   <= r7_par(k);
            end loop;
            r8_solid <= r7_solid;
        end if;
    end process p_r8;

    ------------------------------------------------------------------------
    -- R9: the rasteriser -- two staircases, eight axis-aligned segments per
    -- octave, pure compares.  No multiply, no ROM.
    --
    --   N<->W : (128,0)->(128,64)->(64,64)->(64,128)->(0,128)
    --   S<->E : (128,255)->(128,192)->(192,192)->(192,128)->(255,128)
    --
    -- The two never touch: one hugs the NW corner diagonal, the other the SE.
    -- Every port sits at an edge midpoint and every tile uses all four, so the
    -- path always continues into the neighbour -- degree 2, no branches, no
    -- ends.  The octaves OR together: fine ink inside coarse ink is invisible,
    -- so the finer weave fills the GAPS between the big lines.
    ------------------------------------------------------------------------
    p_r9 : process(clk)
        variable v_ink : std_logic;
        variable v_w8  : unsigned(7 downto 0);
        variable v_v8  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            v_ink := '0';

            for k in 0 to C_NOCT - 1 loop
                -- a-tests bound strokes that run ALONG the wide axis (the
                -- long horizontal runs): full width.  b-tests bound the
                -- vertical jogs: quarter width, so the pixels match.
                v_w8 := resize(r8_w(k), 8);
                v_v8 := resize(r8_wv(k), 8);

                if r8_cross(k) = '1' then
                    -- Crossing: N<->S and W<->E straight through.  The over
                    -- bar is unbroken; the under bar keeps clear of it by
                    -- gap and comes back out the far side.  Parity picks
                    -- the over bar, checkerboard-fashion, so consecutive
                    -- crossings alternate over, under, over.
                    if r8_par(k) = '1' then
                        if r8_a128(k) < v_w8 then v_ink := '1'; end if;
                        if r8_b128(k) < v_v8 and r8_a128(k) >= r8_gap(k) then v_ink := '1'; end if;
                    else
                        if r8_b128(k) < v_v8 then v_ink := '1'; end if;
                        if r8_a128(k) < v_w8 and r8_b128(k) >= r8_gapv(k) then v_ink := '1'; end if;
                    end if;
                else
                    -- N <-> W
                    if r8_a128(k) < v_w8 and r8_v(k) <= 64 then v_ink := '1'; end if;
                    if r8_b64(k)  < v_v8 and r8_uu(k) >= 64 and r8_uu(k) <= 128 then v_ink := '1'; end if;
                    if r8_a64(k)  < v_w8 and r8_v(k)  >= 64 and r8_v(k)  <= 128 then v_ink := '1'; end if;
                    if r8_b128(k) < v_v8 and r8_uu(k) <= 64 then v_ink := '1'; end if;

                    -- S <-> E
                    if r8_a128(k) < v_w8 and r8_v(k) >= 192 then v_ink := '1'; end if;
                    if r8_b192(k) < v_v8 and r8_uu(k) >= 128 and r8_uu(k) <= 192 then v_ink := '1'; end if;
                    if r8_a192(k) < v_w8 and r8_v(k)  >= 128 and r8_v(k)  <= 192 then v_ink := '1'; end if;
                    if r8_b128(k) < v_v8 and r8_uu(k) >= 192 then v_ink := '1'; end if;
                end if;
            end loop;

            r9_ink <= v_ink or r8_solid;
        end if;
    end process p_r9;

    ------------------------------------------------------------------------
    -- R10: ink or paper.  Tint hands the ink the source's own chroma, so the
    -- labyrinth prints in the colour of whatever it is drawn over.
    ------------------------------------------------------------------------
    p_r10 : process(clk)
    begin
        if rising_edge(clk) then
            if (r9_ink xor s_invert) = '1' then
                s_out_y <= s_ink_y;
                if s_tint = '1' then
                    s_out_u <= s_ud(C_LATENCY - 2);
                    s_out_v <= s_vd(C_LATENCY - 2);
                else
                    s_out_u <= s_ink_u;
                    s_out_v <= s_ink_v;
                end if;
            else
                s_out_y <= s_pap_y;
                s_out_u <= s_pap_u;
                s_out_v <= s_pap_v;
            end if;
        end if;
    end process p_r10;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture daedalus;
