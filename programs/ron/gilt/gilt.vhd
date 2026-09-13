-- gilt.vhd
--
-- GILT -- live video re-rendered as a Gustav Klimt "Golden Phase" painting.
--
-- Flesh stays painted and human; everything else is GILDED -- flattened into
-- a shimmering field of gold leaf, mosaic tesserae and hand-placed ornament.
-- The signature is the hard COLLISION between the two worlds (a cheek ends,
-- a gold mosaic begins) and ornament that is placed, never tiled.
--
--   v0.7 OFF-GRID: the ornament no longer lives on a lattice. A blurred
--   content field (luma + both chroma axes, ~8px x ~4 lines) is built from
--   the incoming video and everything structural is driven from it: it
--   DRAGS the cell field sideways (luma) and vertically (chroma), sets cell
--   SIZE, sets ornament DENSITY, joins the cell IDENTITY hash so motif
--   boundaries fall on image contours, shifts the leaf SHADE, and picks the
--   jewel HUE from the picture's own colour. P6 = CHAOS (was Drift).
--   v0.8 NO REPEATS: cell seams deleted (they drew the lattice); bars get a
--   per-cell axis, pitch and phase (no more checkerboard of identical
--   vertical-line squares); discs become organic blobs (per-cell shape term
--   + ellipse squash + an 8px wobble riding the warped lattice); per-cell
--   half-size, ring pitch, diagonal checker and dark/light inversion so no
--   two cells render the same glyph. S7 = FLOW (was Mosaic): swaps which
--   axis luma and chroma drag.
--
--   v0.9 WORLEY: the square quadtree is gone. Cells are the nearest of
--   four jittered lattice points (own cell + the neighbour on the pixel's
--   side, each axis), octagonal metric, six parallel compares -> irregular
--   polygons with no axis-aligned edge anywhere; the material joins the
--   point hash so the tessellation itself re-rolls at image contours. The
--   Luma overlay now SHARES the cells/geometry (own hash salt only), which
--   paid for the engine. P4 = lattice pitch 16/32/64. C_LATENCY 13 -> 17.
--
--   Signal flow (streaming, no frame buffer, C_LATENCY = 17):
--     E1  input latch + position + line-buffer read (contour)
--     E2  flesh chroma distance | region mood/shear hash | v/h edge
--     E3  QUADTREE cell-size resolve (3 coarse-block hashes) + shear
--     E4  normalized in-cell coords + cell-hash A + cell-edge tessera + grain hash A
--     E5  cell-hash B -> motif id/accent | dx,dy -> |dx|,|dy| | grain hash B
--     E6  squared-distance / max | ground ramp read + gild key
--     E7  motif band compares -> layer select
--     E8  compose: ground (+grain +tessera +meadow) with ornament on top
--     E9  contour ink + halo overlay
--     E10 flesh restore (hard-edged warmed original)
--     E11..E13  GILD dry/wet mix (P12 staged), 3 inline multiplies
--
--   Ornament is "not a grid" structurally: a coarse region hash sets each
--   broad area's MOOD (which motifs it favours), whether it SUBDIVIDES to
--   finer cells, and a per-region SHEAR that offsets the cell lattice so
--   neighbouring regions never line up. Per-cell hashes then pick motif,
--   rotation, accent and phase, biased so plain gold is the common outcome.
--
--   Multiplies: 3 (final dry/wet lerp only). Everything else is shift/add,
--   squared-distance LUT, and a 32-entry gold-ramp logic ROM. EBR: 1 line
--   buffer (contour). See SUMMARY.md for the intent-vs-timing trades.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture gilt of program_top is

    constant C_LATENCY : integer := 17;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_cu10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    -- |signed| clamped into 0..255 (u8)
    function f_abs8(v : signed) return unsigned is
        variable t : signed(v'length downto 0);
    begin
        t := resize(v, v'length + 1);
        if t < 0 then
            t := -t;
        end if;
        if t > 255 then
            t := to_signed(255, t'length);
        end if;
        return resize(unsigned(t), 8);
    end function;

    -- |signed| clamped to 0..31 (squared-distance LUT domain)
    function f_abs31(v : signed) return unsigned is
        variable t : signed(v'length downto 0);
    begin
        t := resize(v, v'length + 1);
        if t < 0 then
            t := -t;
        end if;
        if t > 31 then
            t := to_signed(31, t'length);
        end if;
        return resize(unsigned(t), 5);
    end function;

    -- squared-distance LUT, 32 entries (logic ROM)
    type t_sq is array (0 to 31) of unsigned(9 downto 0);
    constant C_SQ : t_sq := (
        to_unsigned(  0, 10), to_unsigned(  1, 10), to_unsigned(  4, 10), to_unsigned(  9, 10),
        to_unsigned( 16, 10), to_unsigned( 25, 10), to_unsigned( 36, 10), to_unsigned( 49, 10),
        to_unsigned( 64, 10), to_unsigned( 81, 10), to_unsigned(100, 10), to_unsigned(121, 10),
        to_unsigned(144, 10), to_unsigned(169, 10), to_unsigned(196, 10), to_unsigned(225, 10),
        to_unsigned(256, 10), to_unsigned(289, 10), to_unsigned(324, 10), to_unsigned(361, 10),
        to_unsigned(400, 10), to_unsigned(441, 10), to_unsigned(484, 10), to_unsigned(529, 10),
        to_unsigned(576, 10), to_unsigned(625, 10), to_unsigned(676, 10), to_unsigned(729, 10),
        to_unsigned(784, 10), to_unsigned(841, 10), to_unsigned(900, 10), to_unsigned(961, 10));

    function f_sq(v : unsigned(4 downto 0)) return unsigned is
    begin
        return C_SQ(to_integer(v));
    end function;

    -- value-noise hash, split across two stages (inferno/turpentine lesson)
    function f_hash_a(cx, cy : unsigned(7 downto 0);
                      seed   : unsigned(15 downto 0)) return unsigned is
        variable h16 : unsigned(15 downto 0);
        variable h   : unsigned(11 downto 0);
    begin
        h16 := (cx & cy) xor seed;
        h16 := h16 xor (x"0" & h16(15 downto 4));
        h   := h16(11 downto 0);
        return h + rotate_left(h, 5);
    end function;

    -- one-pole IIR on an 8-bit content-field value. The content field is
    -- never drawn -- it only tells the ornament engine where the picture is.
    function f_iir(cur, tgt : unsigned(7 downto 0); sh : integer)
        return unsigned is
        variable d, r : signed(9 downto 0);
    begin
        d := resize(signed('0' & tgt), 10) - resize(signed('0' & cur), 10);
        r := resize(signed('0' & cur), 10) + shift_right(d, sh);
        return unsigned(r(7 downto 0));     -- always in 0..255: slice, don't resize
    end function;

    -- CHAOS scaler: eight monotonic steps from nothing to full strength,
    -- all shift-adds (no multiplier). Scales the content-field drives.
    function f_wscale(d : signed(10 downto 0); lvl : unsigned(2 downto 0))
        return signed is
        variable r : signed(10 downto 0);
    begin
        case to_integer(lvl) is
            when 0      => r := (others => '0');
            when 1      => r := shift_right(d, 3);
            when 2      => r := shift_right(d, 2);
            when 3      => r := shift_right(d, 2) + shift_right(d, 3);
            when 4      => r := shift_right(d, 1);
            when 5      => r := shift_right(d, 1) + shift_right(d, 3);
            when 6      => r := shift_right(d, 1) + shift_right(d, 2);
            when others => r := d;
        end case;
        return r;
    end function;

    -- |signed 8| -> 7 bits (inputs never exceed +/-127)
    function f_abs7(v : signed(7 downto 0)) return unsigned is
        variable t : signed(8 downto 0);
    begin
        t := resize(v, 9);
        if t < 0 then
            t := -t;
        end if;
        return unsigned(t(6 downto 0));
    end function;

    -- |d| >> (size + squash): 0/1/2 shift, three-way mux, no adder
    function f_ashr(a : unsigned(7 downto 0); s1, s2 : std_logic)
        return unsigned is
    begin
        if s1 = '1' and s2 = '1' then
            return resize(a(7 downto 2), 7);
        elsif s1 = '1' or s2 = '1' then
            return resize(a(7 downto 1), 7);
        else
            return resize(a, 7);
        end if;
    end function;

    function f_hash_b(hin : unsigned(11 downto 0)) return unsigned is
        variable h : unsigned(11 downto 0);
    begin
        h := hin xor rotate_left(hin, 9);
        h := h + rotate_left(h, 3);
        h := h xor ("00000" & h(11 downto 5));
        return h;
    end function;

    ----------------------------------------------------------------------
    -- gold ground ramp (32 entries, dark->light), interpolated from the
    -- nine Klimt stops (ink brown-black .. bone highlight)
    ----------------------------------------------------------------------
    type t_ramp is array (0 to 31) of unsigned(9 downto 0);
    constant C_RAMP_Y : t_ramp := (
        to_unsigned(  40,10), to_unsigned(  94,10), to_unsigned( 135,10),
        to_unsigned( 184,10), to_unsigned( 228,10), to_unsigned( 273,10),
        to_unsigned( 318,10), to_unsigned( 361,10), to_unsigned( 402,10),
        to_unsigned( 440,10), to_unsigned( 476,10), to_unsigned( 511,10),
        to_unsigned( 546,10), to_unsigned( 579,10), to_unsigned( 612,10),
        to_unsigned( 644,10), to_unsigned( 676,10), to_unsigned( 703,10),
        to_unsigned( 729,10), to_unsigned( 755,10), to_unsigned( 780,10),
        to_unsigned( 804,10), to_unsigned( 825,10), to_unsigned( 845,10),
        to_unsigned( 865,10), to_unsigned( 885,10), to_unsigned( 903,10),
        to_unsigned( 913,10), to_unsigned( 924,10), to_unsigned( 934,10),
        to_unsigned( 945,10), to_unsigned( 955,10));
    constant C_RAMP_U : t_ramp := (
        to_unsigned( 505,10), to_unsigned( 471,10), to_unsigned( 448,10),
        to_unsigned( 426,10), to_unsigned( 405,10), to_unsigned( 385,10),
        to_unsigned( 366,10), to_unsigned( 347,10), to_unsigned( 329,10),
        to_unsigned( 317,10), to_unsigned( 306,10), to_unsigned( 294,10),
        to_unsigned( 284,10), to_unsigned( 281,10), to_unsigned( 277,10),
        to_unsigned( 274,10), to_unsigned( 270,10), to_unsigned( 276,10),
        to_unsigned( 282,10), to_unsigned( 289,10), to_unsigned( 295,10),
        to_unsigned( 304,10), to_unsigned( 320,10), to_unsigned( 336,10),
        to_unsigned( 352,10), to_unsigned( 368,10), to_unsigned( 383,10),
        to_unsigned( 396,10), to_unsigned( 408,10), to_unsigned( 421,10),
        to_unsigned( 433,10), to_unsigned( 445,10));
    constant C_RAMP_V : t_ramp := (
        to_unsigned( 520,10), to_unsigned( 550,10), to_unsigned( 570,10),
        to_unsigned( 587,10), to_unsigned( 602,10), to_unsigned( 615,10),
        to_unsigned( 626,10), to_unsigned( 636,10), to_unsigned( 645,10),
        to_unsigned( 649,10), to_unsigned( 653,10), to_unsigned( 657,10),
        to_unsigned( 660,10), to_unsigned( 659,10), to_unsigned( 657,10),
        to_unsigned( 656,10), to_unsigned( 655,10), to_unsigned( 650,10),
        to_unsigned( 644,10), to_unsigned( 638,10), to_unsigned( 632,10),
        to_unsigned( 626,10), to_unsigned( 617,10), to_unsigned( 609,10),
        to_unsigned( 600,10), to_unsigned( 591,10), to_unsigned( 583,10),
        to_unsigned( 577,10), to_unsigned( 571,10), to_unsigned( 564,10),
        to_unsigned( 558,10), to_unsigned( 552,10));

    -- jewel accents: 8 VIVID collage hues (crimson, scarlet, lapis,
    -- turquoise, viridian, emerald, violet, magenta) : Y,U,V
    type t_jewel is array (0 to 7) of unsigned(9 downto 0);
    constant C_JW_Y : t_jewel := (
        to_unsigned(308,10), to_unsigned(431,10), to_unsigned(246,10), to_unsigned(450,10),
        to_unsigned(345,10), to_unsigned(454,10), to_unsigned(323,10), to_unsigned(374,10));
    constant C_JW_U : t_jewel := (
        to_unsigned(440,10), to_unsigned(315,10), to_unsigned(712,10), to_unsigned(620,10),
        to_unsigned(476,10), to_unsigned(460,10), to_unsigned(691,10), to_unsigned(617,10));
    constant C_JW_V : t_jewel := (
        to_unsigned(863,10), to_unsigned(816,10), to_unsigned(423,10), to_unsigned(249,10),
        to_unsigned(339,10), to_unsigned(275,10), to_unsigned(596,10), to_unsigned(759,10));

    -- ornament ink (warm near-black brown) and bone/white highlight,
    -- used for contour and as fallbacks.
    constant C_INK_Y  : unsigned(9 downto 0) := to_unsigned( 30, 10);
    constant C_INK_U  : unsigned(9 downto 0) := to_unsigned(506, 10);
    constant C_INK_V  : unsigned(9 downto 0) := to_unsigned(520, 10);
    constant C_BONE_Y : unsigned(9 downto 0) := to_unsigned(931, 10);
    constant C_BONE_U : unsigned(9 downto 0) := to_unsigned(431, 10);
    constant C_BONE_V : unsigned(9 downto 0) := to_unsigned(546, 10);

    -- per-cell DARK ornament palette (warm blacks / umbers / bronzes) so
    -- glyph ink varies cell to cell instead of one flat black.
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_DK_Y : t_pal := (
        to_unsigned( 30,10), to_unsigned(120,10), to_unsigned(175,10), to_unsigned( 24,10),
        to_unsigned( 82,10), to_unsigned(112,10), to_unsigned( 40,10), to_unsigned(150,10));
    constant C_DK_U : t_pal := (
        to_unsigned(505,10), to_unsigned(455,10), to_unsigned(425,10), to_unsigned(507,10),
        to_unsigned(500,10), to_unsigned(470,10), to_unsigned(500,10), to_unsigned(438,10));
    constant C_DK_V : t_pal := (
        to_unsigned(520,10), to_unsigned(565,10), to_unsigned(600,10), to_unsigned(517,10),
        to_unsigned(500,10), to_unsigned(598,10), to_unsigned(524,10), to_unsigned(592,10));
    -- per-cell LIGHT ornament palette (bone / pale gold / bright brass /
    -- ivory / amber) so gold glyphs shimmer through the metal range.
    constant C_LT_Y : t_pal := (
        to_unsigned(950,10), to_unsigned(830,10), to_unsigned(700,10), to_unsigned(923,10),
        to_unsigned(860,10), to_unsigned(720,10), to_unsigned(960,10), to_unsigned(770,10));
    constant C_LT_U : t_pal := (
        to_unsigned(440,10), to_unsigned(340,10), to_unsigned(285,10), to_unsigned(462,10),
        to_unsigned(360,10), to_unsigned(300,10), to_unsigned(470,10), to_unsigned(320,10));
    constant C_LT_V : t_pal := (
        to_unsigned(552,10), to_unsigned(610,10), to_unsigned(650,10), to_unsigned(537,10),
        to_unsigned(595,10), to_unsigned(640,10), to_unsigned(545,10), to_unsigned(635,10));

    -- flesh key centre (warm porcelain skin cluster) and luma gate
    constant C_SK_U   : signed(11 downto 0) := to_signed(470, 12);
    constant C_SK_V   : signed(11 downto 0) := to_signed(575, 12);
    constant C_SK_YLO : unsigned(9 downto 0) := to_unsigned(176, 10);
    constant C_SK_YHI : unsigned(9 downto 0) := to_unsigned(956, 10);

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame derived terms (vblank sequencer)
    ----------------------------------------------------------------------
    signal s_p1  : unsigned(9 downto 0) := (others => '0');
    signal s_p2  : unsigned(9 downto 0) := (others => '0');
    signal s_p3  : unsigned(9 downto 0) := (others => '0');
    signal s_p4  : unsigned(9 downto 0) := (others => '0');
    signal s_p5  : unsigned(9 downto 0) := (others => '0');
    signal s_p6  : unsigned(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_flow   : std_logic := '0';   -- S7 Flow: swap which axis luma/chroma drag
    signal s_outln  : std_logic := '1';   -- S8
    signal s_luma   : std_logic := '0';   -- S9 (Luma figure/ground mode)
    signal s_kiss   : std_logic := '0';   -- S10 (0=Adele,1=Kiss)
    signal s_halo   : std_logic := '1';   -- S11
    signal s_meadow : std_logic := '0';   -- derived: auto-on in Kiss bank

    -- Luma mode black level (P1): tiles darker than this stay bare umber
    -- ground; brighter tiles climb the object ladder. Higher P1 = sparser
    -- objects (flesh-restore is off in Luma mode, so P1 is free).
    signal s_luma_thr : unsigned(9 downto 0) := to_unsigned(281, 10);

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';
    signal s_seq          : unsigned(3 downto 0) := (others => '0');

    -- derived per-frame
    signal s_flesh_w  : unsigned(9 downto 0) := to_unsigned(200, 10);
    signal s_pat_bias : signed(5 downto 0)   := (others => '0');
    signal s_pat_u    : signed(6 downto 0)   := (others => '0');
    signal s_pat_v    : signed(6 downto 0)   := (others => '0');
    signal s_k        : integer range 4 to 6 := 5;                  -- Worley pitch 2^k
    signal s_covthr   : unsigned(7 downto 0) := to_unsigned(150, 8);
    signal s_jewthr   : unsigned(7 downto 0) := to_unsigned(60, 8);
    signal s_jsat     : integer range 0 to 3 := 2;
    -- CHAOS (P6): how hard the incoming picture drags the ornament field
    -- off the lattice, and how hard the picture's own material drives cell
    -- size / density / shade / hue.
    signal s_warp_lvl : unsigned(2 downto 0) := "101";
    signal s_mat_lvl  : unsigned(2 downto 0) := "101";
    signal s_mix10    : unsigned(9 downto 0) := (others => '0');
    signal s_goldceil : unsigned(9 downto 0) := (others => '1');
    signal s_orn_on   : std_logic := '1';
    signal s_ctr_on   : std_logic := '1';
    signal s_flat     : unsigned(3 downto 0) := (others => '0');
    signal s_jew_on   : std_logic := '1';
    signal s_halo_amt : unsigned(3 downto 0) := (others => '0');
    constant C_EDGETHR : unsigned(9 downto 0) := to_unsigned(56, 10);

    ----------------------------------------------------------------------
    -- OVERLAY (Luma-mod second gilt world): since v0.9 it SHARES the
    -- Worley cells and geometry with the base; it differs only by its own
    -- identity-hash salt (motif bank, shade, hue), mood, pale ground and
    -- the luma mask that reveals it.
    ----------------------------------------------------------------------
    signal omood9, omood10 : unsigned(1 downto 0) := (others => '0');
    signal ochb9, ochb10 : unsigned(11 downto 0) := (others => '0');
    signal oosel11 : unsigned(1 downto 0) := (others => '0');
    signal ocvar11, ojidx11 : unsigned(2 downto 0) := (others => '0');
    signal og_y11, og_u11, og_v11 : unsigned(9 downto 0) := (others => '0');
    signal omask11 : std_logic := '0';

    ----------------------------------------------------------------------
    -- position / line bookkeeping
    ----------------------------------------------------------------------
    signal s_x_count : unsigned(10 downto 0) := (others => '0');
    signal s_aline   : unsigned(10 downto 0) := (others => '0');
    signal s_seen    : std_logic := '0';
    -- registered "meadow row" flag: the s_aline > 420 compare is a 10-bit
    -- carry chain that was on E8's critical path when evaluated per pixel
    signal s_mrow    : std_logic := '0';

    ----------------------------------------------------------------------
    -- line buffer (prev-line luma8) for contour
    ----------------------------------------------------------------------
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    signal lb_q   : std_logic_vector(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- per-tile luma RAM (Luma-mod mask): one averaged luma per 16x16 tile,
    -- kept as a 4-STRIPE RING (64 lines) so an overlay cell of any size
    -- (8..64px) can sample the tile at its own ORIGIN -> whole-cell
    -- inclusion, quilt-style. Address = {stripe(1:0), tile column(6:0)}.
    ----------------------------------------------------------------------
    type t_lram is array (0 to 1023) of unsigned(9 downto 0);
    signal lram    : t_lram := (others => (others => '0'));
    signal lram_ra : unsigned(9 downto 0) := (others => '0');
    signal lram_wa : unsigned(9 downto 0) := (others => '0');
    signal lram_wd : unsigned(9 downto 0) := (others => '0');
    signal lram_we : std_logic := '0';
    signal lram_q  : unsigned(9 downto 0) := (others => '0');
    signal s_lacc  : unsigned(13 downto 0) := (others => '0');
    signal s_lgood : std_logic := '0';

    ----------------------------------------------------------------------
    -- CONTENT FIELD (v0.7) -- a blurred picture of the incoming video:
    -- ~8 px horizontally (one-pole IIR along the line) and ~4 lines
    -- vertically (a one-line buffer feeding a second IIR), on luma AND on
    -- both chroma axes. Nothing here is ever drawn. It exists so that the
    -- ornament engine has something to follow that is not a coordinate:
    -- the cell lattice is dragged by it, and cell size, cell identity,
    -- ornament density, leaf shade and jewel hue all key off it. This is
    -- what takes the engine off the grid.
    --
    -- vram is a single-bank line buffer at 4 px decimation whose read
    -- address runs one group AHEAD of the write address -- iCE40 EBR
    -- returns UNDEFINED on a same-address read-during-write, so the two
    -- must never touch the same word.
    ----------------------------------------------------------------------
    type t_vram is array (0 to 511) of std_logic_vector(23 downto 0);
    signal vram    : t_vram := (others => (others => '0'));
    signal vram_q  : std_logic_vector(23 downto 0) := (others => '0');
    signal vram_ra : unsigned(8 downto 0) := (others => '0');
    signal vram_wa : unsigned(8 downto 0) := (others => '0');
    signal vram_wd : std_logic_vector(23 downto 0) := (others => '0');
    signal vram_we : std_logic := '0';
    signal vw_a1, vw_a2 : unsigned(8 downto 0) := (others => '0');
    signal vw_e1, vw_e2 : std_logic := '0';

    signal s_ylp, s_ulp, s_vlp : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_ysm, s_usm, s_vsm : unsigned(7 downto 0) := to_unsigned(128, 8);

    signal fa_du, fa_dv : signed(8 downto 0)   := (others => '0');
    signal fa_ysm       : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal fb_sat       : unsigned(9 downto 0) := (others => '0');
    signal fb_lumd      : signed(8 downto 0)   := (others => '0');
    signal fb_wdy       : signed(8 downto 0)   := (others => '0');
    signal fb_mat       : unsigned(5 downto 0) := (others => '0');
    signal fb_jhue      : unsigned(2 downto 0) := (others => '0');
    signal fb_sathi     : std_logic := '0';
    signal fc_ad        : signed(10 downto 0)  := (others => '0');
    signal fc_sat       : unsigned(7 downto 0) := (others => '0');

    -- what the rest of the engine actually consumes (free-running: a few
    -- pixels of skew, invisible under the 8 px blur that produced them)
    signal s_warpx, s_warpy : signed(10 downto 0)  := (others => '0');
    signal s_mat       : unsigned(5 downto 0) := (others => '0');
    signal s_jhue      : unsigned(2 downto 0) := (others => '0');
    signal s_sathi     : std_logic := '0';
    signal s_covthr_px : unsigned(7 downto 0) := to_unsigned(150, 8);

    ----------------------------------------------------------------------
    -- pixel pipeline registers
    ----------------------------------------------------------------------
    signal iy1, iu1, iv1 : unsigned(9 downto 0) := (others => '0');
    signal px1 : unsigned(10 downto 0) := (others => '0');
    signal luma8_1 : unsigned(7 downto 0) := (others => '0');
    signal ph8_1   : unsigned(7 downto 0) := (others => '0');

    signal iy2, iy3, iy4, iy5, iy6, iy7, iy8, iy9, iy10
        : unsigned(9 downto 0) := (others => '0');

    -- flesh/edge carries (fdist,fgate to age 13; edge to age 12)
    signal fdist2, fdist3, fdist4, fdist5, fdist6, fdist7, fdist8, fdist9,
           fdist10, fdist11, fdist12, fdist13
        : unsigned(10 downto 0) := (others => '0');
    signal fgate2, fgate3, fgate4, fgate5, fgate6, fgate7, fgate8, fgate9,
           fgate10, fgate11, fgate12, fgate13
        : std_logic := '0';
    signal edge2, edge3, edge4, edge5, edge6, edge7, edge8, edge9, edge10,
           edge11, edge12
        : unsigned(9 downto 0) := (others => '0');

    signal px2, px3 : unsigned(10 downto 0) := (others => '0');

    -- mood (plain / busy / tall bars) per 4x4 group of Worley cells
    signal rmood9, rmood10 : unsigned(1 downto 0) := (others => '0');
    signal mha8 : unsigned(11 downto 0) := (others => '0');

    -- E2 content-warped position
    signal sxw2, syw2 : unsigned(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- WORLEY cell engine (E3..E8): nearest of four jittered lattice points
    -- (own cell + the neighbour on the pixel's side, each axis). Cells are
    -- irregular polygons; there is no axis-aligned edge anywhere.
    ----------------------------------------------------------------------
    type t_a7 is array (0 to 3) of unsigned(6 downto 0);
    type t_a8 is array (0 to 3) of unsigned(7 downto 0);
    type t_a1 is array (0 to 3) of std_logic;
    signal sxr3, syr3 : unsigned(10 downto 0) := (others => '0');
    signal cx0_3, cx1_3, cy0_3, cy1_3 : unsigned(7 downto 0) := (others => '0');
    signal fx3, fy3 : unsigned(5 downto 0) := (others => '0');
    signal h00_4, h10_4, h01_4, h11_4 : unsigned(11 downto 0) := (others => '0');
    signal fx4, fy4, fxn4, fyn4 : signed(7 downto 0) := (others => '0');
    signal cx0_4, cy0_4, cx0_5, cy0_5, cx0_6, cy0_6, cx0_7, cy0_7, cx1_7, cy1_7
        : unsigned(7 downto 0) := (others => '0');
    signal nsx4, nsy4, nsx5, nsy5, nsx6, nsy6 : std_logic := '0';
    signal cadx5, cady5, cadx6, cady6, cadx7, cady7 : t_a7 := (others => (others => '0'));
    signal csgx5, csgy5, csgx6, csgy6, csgx7, csgy7 : t_a1 := (others => '0');
    signal cd6 : t_a8 := (others => (others => '0'));
    signal c01_7, c02_7, c03_7, c12_7, c13_7, c23_7 : std_logic := '0';
    signal wadx8, wady8 : unsigned(6 downto 0) := (others => '0');
    signal wsgx8, wsgy8 : std_logic := '0';
    signal wcx8, wcy8 : unsigned(7 downto 0) := (others => '0');
    signal cha8, gha8, bha8 : unsigned(11 downto 0) := (others => '0');

    -- E9 : cell-hash B + per-cell glyph parameters + grain/wobble
    signal chb9     : unsigned(11 downto 0) := (others => '0');
    signal adx9, ady9 : unsigned(6 downto 0) := (others => '0');
    signal lxn9, lyn9 : unsigned(5 downto 0) := (others => '0');
    signal grain9   : signed(6 downto 0) := (others => '0');
    signal ridx9    : unsigned(4 downto 0) := (others => '0');   -- ramp index
    signal inv9, inv10  : std_logic := '0';                      -- dark<->light swap
    signal flp9         : std_logic := '0';                      -- bars horizontal
    signal wob9         : signed(8 downto 0) := (others => '0'); -- blob wobble + grain
    signal barin10      : std_logic := '0';                      -- inside a bar stripe
    signal dsum10, ddif10 : unsigned(6 downto 0) := (others => '0');

    -- E6 : squared distance / max (geometry split, part 2) + ground
    signal chb10     : unsigned(11 downto 0) := (others => '0');
    signal adx10, ady10 : unsigned(6 downto 0) := (others => '0');
    signal mmax10    : unsigned(6 downto 0) := (others => '0');
    signal r2_10     : unsigned(11 downto 0) := (others => '0');
    signal lxn10, lyn10 : unsigned(5 downto 0) := (others => '0');
    signal grain10   : signed(6 downto 0) := (others => '0');
    signal gyr10, gur10, gvr10 : unsigned(9 downto 0) := (others => '0'); -- raw ramp
    signal gild10    : std_logic := '0';

    -- E7 : motif select + eval
    signal msel11   : unsigned(1 downto 0) := (others => '0');
    signal jidx11   : unsigned(2 downto 0) := (others => '0');   -- jewel hue
    signal cvar11   : unsigned(2 downto 0) := (others => '0');   -- gold-shade
    signal gy11, gu11, gv11 : unsigned(9 downto 0) := (others => '0');
    signal gild11   : std_logic := '0';

    -- E8 compose, E9 contour/halo, E10 flesh restore
    signal wy12, wu12, wv12 : unsigned(9 downto 0) := (others => '0');
    signal wy13, wu13, wv13 : unsigned(9 downto 0) := (others => '0');
    signal wy14, wu14, wv14 : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- dry delay (original video) + sync delay + inline mix
    ----------------------------------------------------------------------
    type t_dsr is array (0 to 13) of std_logic_vector(9 downto 0);
    signal dy_sr, du_sr, dv_sr : t_dsr := (others => (others => '0'));

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

    signal m1_ay, m1_au, m1_av : unsigned(9 downto 0) := (others => '0');
    signal m1_dy, m1_du, m1_dv : signed(10 downto 0) := (others => '0');
    -- The mix coefficient feeds all THREE multipliers. As one shared net it
    -- was the HD HDMI critical path: 7.1 ns of pure routing on a single hop
    -- across the die, and the seed spread ran 68-84 MHz on placement luck
    -- alone. Kept as three copies so the placer can sit each one beside its
    -- own multiplier; "keep" stops yosys merging them back into one.
    signal m1_t8y, m1_t8u, m1_t8v : unsigned(7 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of m1_t8y : signal is true;
    attribute keep of m1_t8u : signal is true;
    attribute keep of m1_t8v : signal is true;
    signal m2_ay, m2_au, m2_av : unsigned(9 downto 0) := (others => '0');
    signal m2_py, m2_pu, m2_pv : signed(19 downto 0) := (others => '0');
    signal m3_y, m3_u, m3_v    : unsigned(9 downto 0) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_w  : unsigned(11 downto 0);
        variable v_sc : unsigned(9 downto 0);
        variable v_fl : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_p1  <= unsigned(registers_in(0));
                s_p2  <= unsigned(registers_in(1));
                s_p3  <= unsigned(registers_in(2));
                s_p4  <= unsigned(registers_in(3));
                s_p5  <= unsigned(registers_in(4));
                s_p6  <= unsigned(registers_in(5));
                s_flow   <= registers_in(6)(0);
                s_outln  <= registers_in(6)(1);
                s_luma   <= registers_in(6)(2);      -- S9 = Luma mode
                s_kiss   <= registers_in(6)(3);
                s_halo   <= registers_in(6)(4);
                s_meadow <= registers_in(6)(3);      -- meadow auto-on in Kiss
                s_p12 <= unsigned(registers_in(7));

                s_seq <= to_unsigned(11, 4);
            elsif s_seq /= 0 and data_in.avid = '0' then
                case to_integer(s_seq) is
                    when 11 =>
                        v_w := resize(s_p1(9 downto 1), 12)
                               + resize(s_p1(9 downto 3), 12);
                        if s_p12 > 870 then
                            v_w := resize(v_w(11 downto 1), 12)
                                   + resize(v_w(11 downto 2), 12);
                        end if;
                        if v_w > 511 then
                            s_flesh_w <= to_unsigned(511, 10);
                        else
                            s_flesh_w <= resize(v_w(9 downto 0), 10);
                        end if;
                        -- Luma-mod mask threshold from P1 (96..607): input
                        -- brighter than this reveals the overlay field;
                        -- higher P1 = smaller revealed area.
                        s_luma_thr <= to_unsigned(96, 10)
                                      + resize(s_p1(9 downto 1), 10);
                    when 10 =>
                        s_pat_bias <= resize(signed('0' & s_p2(9 downto 7)), 6)
                                      - to_signed(3, 6);
                        s_pat_v <= to_signed(24, 7)
                                   - resize(signed('0' & s_p2(9 downto 5)), 7);
                        s_pat_u <= resize(signed('0' & s_p2(9 downto 5)), 7)
                                   - to_signed(24, 7);
                    when 9 =>
                        -- SCALE: quadtree subdivide probability. High P4 =
                        -- fewer subdivisions = bold poster blocks; low P4 =
                        -- everything resolves fine. Multi-scale mix at any
                        -- setting (some cells stay coarse, neighbours split).
                        -- SCALE: Worley lattice pitch 16 / 32 / 64 px
                        if s_p4 < 256 then
                            s_k <= 4;
                        elsif s_p4 < 768 then
                            s_k <= 5;
                        else
                            s_k <= 6;
                        end if;
                    when 8 =>
                        v_sc := to_unsigned(230, 10)
                                - resize(s_p3(9 downto 2), 10);
                        if s_orn_on = '0' then
                            s_covthr <= to_unsigned(250, 8);
                        elsif v_sc < 40 then
                            s_covthr <= to_unsigned(40, 8);
                        else
                            s_covthr <= resize(v_sc(7 downto 0), 8);
                        end if;
                    when 7 =>
                        -- JEWEL: fraction of ornament cells that become a
                        -- SOLID bright-colour cell (crimson/lapis/violet...).
                        -- P5>>2 => up to ~60% of ornamented cells at max.
                        if s_jew_on = '0' or s_p5 < 24 then
                            s_jewthr <= to_unsigned(0, 8);
                        else
                            s_jewthr <= resize(s_p5(9 downto 2), 8);
                        end if;
                        -- vivid by default: full saturation from mid-P5 up.
                        if s_p5 < 96 then
                            s_jsat <= 2;                    -- washed (low)
                        elsif s_p5 < 288 then
                            s_jsat <= 1;                    -- half
                        else
                            s_jsat <= 0;                    -- full vivid
                        end if;
                    when 6 =>
                        -- CHAOS (P6): how hard the incoming picture drags the
                        -- ornament off the lattice. At 0 the field is the old
                        -- ruled quadtree; at full it is dragged bodily by luma
                        -- (sideways) and chroma (up and down). The MATERIAL
                        -- coupling never falls to zero -- even at P6 = 0 the
                        -- cell size, density, leaf shade and jewel hue still
                        -- follow the picture, so nothing here is ever purely
                        -- mathematical.
                        s_warp_lvl <= s_p6(9 downto 7);
                        s_mat_lvl  <= to_unsigned(3, 3) + s_p6(9 downto 8);
                    when 5 =>
                        v_fl := resize(s_p12, 12) + resize(s_p12(9 downto 1), 12);
                        if v_fl > 1023 then
                            s_mix10 <= (others => '1');
                        else
                            s_mix10 <= resize(v_fl(9 downto 0), 10);
                        end if;
                    when 4 =>
                        if s_p12 < 154 then
                            s_goldceil <= (others => '0');
                        else
                            v_fl := resize(s_p12 - 154, 12);
                            v_fl := v_fl + resize(v_fl(11 downto 1), 12)
                                    + resize(v_fl(11 downto 1), 12);
                            if v_fl > 1023 then
                                s_goldceil <= (others => '1');
                            else
                                s_goldceil <= resize(v_fl(9 downto 0), 10);
                            end if;
                        end if;
                    when 3 =>
                        if s_p12 > 358 then
                            s_ctr_on <= '1';
                            s_flat   <= resize(shift_right(s_p12 - 358, 6), 4);
                        else
                            s_ctr_on <= '0';
                            s_flat   <= (others => '0');
                        end if;
                    when 2 =>
                        if s_p12 > 614 then
                            s_orn_on <= '1';
                        else
                            s_orn_on <= '0';
                        end if;
                    when 1 =>
                        -- colour arrives WITH the ornament bloom (not only at
                        -- the very top), then halo in the final stretch.
                        if s_p12 > 560 then
                            s_jew_on <= '1';
                        else
                            s_jew_on <= '0';
                        end if;
                        if s_p12 > 870 then
                            s_halo_amt <= resize(shift_right(s_p12 - 870, 4), 4);
                        else
                            s_halo_amt <= (others => '0');
                        end if;
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- line buffer: prev-line luma8 at current x (read before write)
    ------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            lb_q <= lb_ram(to_integer(s_x_count));
            if data_in.avid = '1' then
                lb_ram(to_integer(s_x_count))
                    <= std_logic_vector(unsigned(data_in.y(9 downto 2)));
            end if;
        end if;
    end process p_lb;

    ------------------------------------------------------------------------
    -- CONTENT FIELD: blur the incoming picture, then turn it into the
    -- structural drives the ornament engine follows.
    ------------------------------------------------------------------------
    p_field : process(clk)
        variable v_y, v_u, v_v    : unsigned(7 downto 0);
        variable v_py, v_pu, v_pv : unsigned(7 downto 0);
        variable v_adu, v_adv     : unsigned(9 downto 0);
        variable v_sec, v_jh      : unsigned(2 downto 0);
        variable v_wy             : signed(11 downto 0);
        variable v_st, v_ct       : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            ------------------------------------------------------------
            -- F1: horizontal IIR (~8 px), reloaded at each line start
            ------------------------------------------------------------
            v_y := unsigned(data_in.y(9 downto 2));
            v_u := unsigned(data_in.v(9 downto 2));   -- HW U<->V swap
            v_v := unsigned(data_in.u(9 downto 2));
            if s_x_count = 0 then
                s_ylp <= v_y;  s_ulp <= v_u;  s_vlp <= v_v;
            else
                s_ylp <= f_iir(s_ylp, v_y, 3);
                s_ulp <= f_iir(s_ulp, v_u, 3);
                s_vlp <= f_iir(s_vlp, v_v, 3);
            end if;

            -- read the group we are about to update; the write of the same
            -- group lands two cycles later, by which time the read address
            -- has moved on to the next group (never the same word).
            vram_ra <= s_x_count(10 downto 2);
            vw_a1   <= s_x_count(10 downto 2);
            if data_in.avid = '1' and s_x_count(1 downto 0) = "11" then
                vw_e1 <= '1';
            else
                vw_e1 <= '0';
            end if;
            vw_a2 <= vw_a1;
            vw_e2 <= vw_e1;

            ------------------------------------------------------------
            -- F2: vertical IIR (~4 lines) through the line buffer
            ------------------------------------------------------------
            v_py := unsigned(vram_q(23 downto 16));
            v_pu := unsigned(vram_q(15 downto  8));
            v_pv := unsigned(vram_q( 7 downto  0));
            if s_aline < 2 then          -- seed the field at the frame top
                s_ysm <= s_ylp;  s_usm <= s_ulp;  s_vsm <= s_vlp;
            else
                s_ysm <= f_iir(v_py, s_ylp, 2);
                s_usm <= f_iir(v_pu, s_ulp, 2);
                s_vsm <= f_iir(v_pv, s_vlp, 2);
            end if;
            vram_wd <= std_logic_vector(s_ysm) & std_logic_vector(s_usm)
                       & std_logic_vector(s_vsm);
            vram_wa <= vw_a2;
            vram_we <= vw_e2 and data_in.avid;

            ------------------------------------------------------------
            -- F3: chroma offsets about neutral
            ------------------------------------------------------------
            fa_du  <= signed('0' & s_usm) - to_signed(128, 9);
            fa_dv  <= signed('0' & s_vsm) - to_signed(128, 9);
            fa_ysm <= s_ysm;

            ------------------------------------------------------------
            -- F4: what the picture MEANS -- saturation, hue sector,
            -- material id, and the two lattice drives.
            ------------------------------------------------------------
            v_adu := resize(f_abs8(fa_du), 10);
            v_adv := resize(f_abs8(fa_dv), 10);
            fb_sat  <= v_adu + v_adv;
            fb_lumd <= signed('0' & fa_ysm) - to_signed(128, 9);

            -- vertical drive from the warm/cool chroma axis, with a luma
            -- term folded in so a grey picture still bends vertically
            v_wy := shift_left(resize(fa_dv - fa_du, 12), 1)
                    + shift_right(resize(signed('0' & fa_ysm) - to_signed(128, 9), 12), 1);
            if v_wy > 127 then
                fb_wdy <= to_signed(127, 9);
            elsif v_wy < -127 then
                fb_wdy <= to_signed(-127, 9);
            else
                fb_wdy <= resize(v_wy, 9);
            end if;

            -- nearest jewel to the picture's own hue (8 sectors)
            if fa_du < 0 then
                if fa_dv >= 0 then
                    if v_adv >= v_adu then v_jh := "000";   -- crimson
                    else                   v_jh := "001";   -- scarlet
                    end if;
                    v_sec := "100";
                else
                    if v_adu >= v_adv then v_jh := "100";   -- viridian
                    else                   v_jh := "101";   -- emerald
                    end if;
                    v_sec := "101";
                end if;
            else
                if fa_dv < 0 then
                    if v_adv >= v_adu then v_jh := "011";   -- turquoise
                    else                   v_jh := "010";   -- lapis
                    end if;
                    v_sec := "110";
                else
                    if v_adu >= v_adv then v_jh := "110";   -- violet
                    else                   v_jh := "111";   -- magenta
                    end if;
                    v_sec := "111";
                end if;
            end if;
            if (v_adu + v_adv) < 14 then
                v_sec    := "000";                  -- grey: no hue of its own
                fb_sathi <= '0';
            else
                fb_sathi <= '1';
            end if;
            fb_jhue <= v_jh;
            -- MATERIAL: 8 tonal bands x 5 chroma sectors. Two cells in the
            -- same place but on different material are different cells.
            fb_mat  <= fa_ysm(7 downto 5) & v_sec;

            ------------------------------------------------------------
            -- F5: scale the drives by CHAOS
            ------------------------------------------------------------
            s_warpx <= f_wscale(resize(fb_lumd, 11), s_warp_lvl);
            s_warpy <= f_wscale(resize(fb_wdy,  11), s_warp_lvl);
            fc_ad   <= f_wscale(resize(fb_lumd, 11), s_mat_lvl);
            fc_sat  <= fb_sat(9 downto 2);
            s_mat   <= fb_mat;
            s_jhue  <= fb_jhue;
            s_sathi <= fb_sathi;

            ------------------------------------------------------------
            -- F6: the picture sets cell SIZE and ornament DENSITY --
            -- bright, saturated ground resolves into fine busy cells,
            -- dark flat ground stays big and plain.
            ------------------------------------------------------------
            if s_orn_on = '0' then
                s_covthr_px <= s_covthr;      -- P12 below the bloom: stay plain
            else
                v_ct := resize(signed('0' & s_covthr), 12)
                        - resize(shift_right(fc_ad, 1), 12)
                        - resize(signed('0' & fc_sat), 12);
                if v_ct < 24 then
                    s_covthr_px <= to_unsigned(24, 8);
                elsif v_ct > 250 then
                    s_covthr_px <= to_unsigned(250, 8);
                else
                    s_covthr_px <= unsigned(v_ct(7 downto 0));
                end if;
            end if;
        end if;
    end process p_field;

    p_vram : process(clk)
    begin
        if rising_edge(clk) then
            if vram_we = '1' then
                vram(to_integer(vram_wa)) <= vram_wd;
            end if;
            vram_q <= vram(to_integer(vram_ra));
        end if;
    end process p_vram;

    ------------------------------------------------------------------------
    -- main pixel pipeline
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_du, v_dv   : signed(11 downto 0);
        variable v_eh, v_ev   : signed(8 downto 0);
        variable v_rx, v_ry   : unsigned(7 downto 0);
        variable v_cx, v_cy   : unsigned(7 downto 0);
        variable v_sx         : unsigned(10 downto 0);
        variable v_sy         : unsigned(10 downto 0);
        variable v_sh         : integer range 3 to 6;
        variable v_hs         : unsigned(11 downto 0);
        variable v_s6, v_s5, v_s4 : std_logic;
        variable v_lx, v_ly   : unsigned(5 downto 0);
        variable v_mask       : unsigned(6 downto 0);
        variable v_hy         : unsigned(9 downto 0);
        variable v_dx, v_dy   : signed(7 downto 0);
        variable v_h          : unsigned(11 downto 0);
        variable v_gyidx      : signed(12 downto 0);
        variable v_idx        : integer range 0 to 31;
        variable v_cov, v_acc : unsigned(7 downto 0);
        variable v_mtype      : unsigned(2 downto 0);
        variable v_jidx       : unsigned(2 downto 0);
        variable v_ring       : unsigned(2 downto 0);
        variable v_sub        : unsigned(1 downto 0);
        variable v_sel        : unsigned(1 downto 0);
        variable v_plain      : std_logic;
        variable v_colcell    : std_logic;
        variable v_pillar     : std_logic;
        variable v_cvar       : unsigned(2 downto 0);
        variable v_ocvar      : unsigned(2 downto 0);
        variable v_plainthr   : unsigned(8 downto 0);
        variable v_tri        : signed(8 downto 0);
        variable v_by, v_bu, v_bv    : unsigned(9 downto 0);
        variable v_ju, v_jv   : unsigned(9 downto 0);
        variable v_du2, v_dv2 : signed(11 downto 0);
        variable v_delta      : signed(6 downto 0);
        variable v_oy, v_ou, v_ov : unsigned(9 downto 0);
        variable v_gr2        : signed(13 downto 0);
        variable v_gmm        : signed(8 downto 0);
        variable v_rph        : unsigned(6 downto 0);
        variable v_ax, v_ay   : unsigned(7 downto 0);
        variable v_b          : unsigned(11 downto 0);
        variable v_grain      : signed(6 downto 0);
        variable v_abk        : unsigned(8 downto 0);
        variable v_barin      : std_logic;
        variable v_ringv      : unsigned(2 downto 0);
        variable v_cx0, v_cy0 : unsigned(7 downto 0);
        variable v_fx, v_fy   : unsigned(5 downto 0);
        variable v_mx8, v_my8 : unsigned(7 downto 0);
        variable v_dd         : signed(7 downto 0);
        variable v_mx, v_sm   : unsigned(7 downto 0);
        variable v_wi         : unsigned(1 downto 0);
        variable v_wcx, v_wcy : unsigned(7 downto 0);
        variable v_size, v_sqx, v_sqy : std_logic;
        variable v_l          : signed(8 downto 0);
        variable v_row        : signed(11 downto 0);
        variable v_col        : unsigned(10 downto 0);
        variable v_osx        : unsigned(10 downto 0);
        variable v_osy        : unsigned(10 downto 0);
        variable v_osh        : integer range 3 to 6;
        variable v_oxo        : unsigned(10 downto 0);
        variable v_oyo        : unsigned(10 downto 0);
        variable v_osel       : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            if s_aline > 420 then
                s_mrow <= '1';
            else
                s_mrow <= '0';
            end if;

            --------------------------------------------------------------
            -- E1: input latch + position + dry SR + prev-pixel luma
            --------------------------------------------------------------
            -- HW chroma is carried U<->V swapped vs standard BT.601 (see
            -- cga / mondrian / phosphor). Swap once here so ALL internal
            -- logic (gold ramp, flesh key, jewels) is authored in plain
            -- standard convention (internal u = Cb, v = Cr); swap back at
            -- the output. Passthrough stays identity.
            iy1 <= unsigned(data_in.y);
            iu1 <= unsigned(data_in.v);
            iv1 <= unsigned(data_in.u);
            px1 <= s_x_count;
            luma8_1 <= unsigned(data_in.y(9 downto 2));
            ph8_1   <= luma8_1;

            if data_in.avid = '1' then
                s_x_count <= s_x_count + 1;
                s_seen    <= '1';
            end if;

            dy_sr(0) <= data_in.y;
            du_sr(0) <= data_in.v;   -- swapped to internal (Cb) convention
            dv_sr(0) <= data_in.u;   -- swapped to internal (Cr) convention
            for i in 1 to 13 loop
                dy_sr(i) <= dy_sr(i - 1);
                du_sr(i) <= du_sr(i - 1);
                dv_sr(i) <= dv_sr(i - 1);
            end loop;

            iy2 <= iy1; iy3 <= iy2; iy4 <= iy3; iy5 <= iy4; iy6 <= iy5;
            iy7 <= iy6; iy8 <= iy7; iy9 <= iy8; iy10 <= iy9;

            --------------------------------------------------------------
            -- E2: flesh chroma distance | edge | region-hash A
            --------------------------------------------------------------
            v_du := signed('0' & iu1) - C_SK_U;
            v_dv := signed('0' & iv1) - C_SK_V;
            fdist2 <= resize(f_abs8(v_du), 11)
                      + resize(f_abs8(v_dv), 11)
                      + resize(f_abs8(v_dv(11 downto 1)), 11);
            if iy1 > C_SK_YLO and iy1 < C_SK_YHI then
                fgate2 <= '1';
            else
                fgate2 <= '0';
            end if;

            v_eh := signed('0' & luma8_1) - signed('0' & ph8_1);
            v_ev := signed('0' & luma8_1) - signed('0' & unsigned(lb_q));
            edge2 <= resize(f_abs8(v_eh), 10) + resize(f_abs8(v_ev), 10);

            px2  <= px1;

            -- CONTENT WARP: the picture itself drags the ornament lattice.
            -- Luma pulls it sideways, chroma pulls it up and down, so cell
            -- rows sag over a face and pile up along a colour edge. Every
            -- downstream coordinate (block hashes, cell coords, tessera
            -- seams, the tall bars keyed to absolute x) rides this, so the
            -- whole field bends together instead of being ruled.
            -- S7 FLOW swaps which axis luma and chroma drag; the overlay
            -- always takes the opposite pairing so the two worlds never flow
            -- the same way.
            if s_flow = '0' then
                sxw2 <= px1     + unsigned(s_warpx);
                syw2 <= s_aline + unsigned(s_warpy);
            else
                sxw2 <= px1     + unsigned(s_warpy);
                syw2 <= s_aline + unsigned(s_warpx);
            end if;

            fdist3 <= fdist2; fgate3 <= fgate2; edge3 <= edge2;

            --------------------------------------------------------------
            -- E3 (Worley 1): lattice cell + in-cell fraction, normalized to
            -- a 0..63 space whatever the pitch (P4: 16 / 32 / 64 px).
            --------------------------------------------------------------
            sxr3 <= sxw2;
            syr3 <= syw2;
            case s_k is
                when 4 =>
                    v_cx0 := resize(shift_right(sxw2, 4), 8);
                    v_cy0 := resize(shift_right(syw2, 4), 8);
                    v_fx  := sxw2(3 downto 0) & "00";
                    v_fy  := syw2(3 downto 0) & "00";
                when 5 =>
                    v_cx0 := resize(shift_right(sxw2, 5), 8);
                    v_cy0 := resize(shift_right(syw2, 5), 8);
                    v_fx  := sxw2(4 downto 0) & '0';
                    v_fy  := syw2(4 downto 0) & '0';
                when others =>
                    v_cx0 := resize(shift_right(sxw2, 6), 8);
                    v_cy0 := resize(shift_right(syw2, 6), 8);
                    v_fx  := sxw2(5 downto 0);
                    v_fy  := syw2(5 downto 0);
            end case;
            cx0_3 <= v_cx0;  cy0_3 <= v_cy0;
            fx3   <= v_fx;   fy3   <= v_fy;
            -- the neighbour worth testing is the one on the pixel's side
            if v_fx(5) = '1' then cx1_3 <= v_cx0 + 1; else cx1_3 <= v_cx0 - 1; end if;
            if v_fy(5) = '1' then cy1_3 <= v_cy0 + 1; else cy1_3 <= v_cy0 - 1; end if;
            px3    <= px2;
            fdist4 <= fdist3; fgate4 <= fgate3; edge4 <= edge3;

            --------------------------------------------------------------
            -- E4 (Worley 2): the four candidate POINTS, one hash per cell.
            -- The MATERIAL joins the hash input, so the point set -- and
            -- with it the whole tessellation -- re-rolls where the picture
            -- changes: the image fractures the tiling along its own edges.
            --------------------------------------------------------------
            v_mx8 := "00" & s_mat;
            v_my8 := s_mat & "00";
            h00_4 <= f_hash_a(cx0_3 xor v_mx8, cy0_3 xor v_my8, x"B7C3");
            h10_4 <= f_hash_a(cx1_3 xor v_mx8, cy0_3 xor v_my8, x"B7C3");
            h01_4 <= f_hash_a(cx0_3 xor v_mx8, cy1_3 xor v_my8, x"B7C3");
            h11_4 <= f_hash_a(cx1_3 xor v_mx8, cy1_3 xor v_my8, x"B7C3");
            -- fraction pre-offset by the neighbour's origin so E5 is one
            -- subtract + one abs per candidate
            fx4 <= signed(resize(fx3, 8));
            fy4 <= signed(resize(fy3, 8));
            if fx3(5) = '1' then
                fxn4 <= signed(resize(fx3, 8)) - to_signed(64, 8);
            else
                fxn4 <= signed(resize(fx3, 8)) + to_signed(64, 8);
            end if;
            if fy3(5) = '1' then
                fyn4 <= signed(resize(fy3, 8)) - to_signed(64, 8);
            else
                fyn4 <= signed(resize(fy3, 8)) + to_signed(64, 8);
            end if;
            cx0_4 <= cx0_3;  cy0_4 <= cy0_3;
            nsx4  <= fx3(5); nsy4  <= fy3(5);

            -- per-tile luma sampler: average the TOP line of every 16x16
            -- screen tile into the 8-stripe ring (address {aline(6:4), col}).
            -- This is the Luma-mod whole-cell mask source.
            lram_we <= '0';
            if s_avid_sr(2) = '1' and s_aline(3 downto 0) = 0 then
                if px3(3 downto 0) = 0 then
                    s_lacc  <= resize(iy3, 14);
                    s_lgood <= '1';
                elsif s_lgood = '1' then
                    s_lacc <= s_lacc + resize(iy3, 14);
                end if;
                if px3(3 downto 0) = 15 and s_lgood = '1' then
                    lram_wa <= s_aline(6 downto 4) & px3(10 downto 4);
                    lram_wd <= resize(shift_right(s_lacc + resize(iy3, 14), 4), 10);
                    lram_we <= '1';
                end if;
            end if;

            fdist5 <= fdist4; fgate5 <= fgate4; edge5 <= edge4;

            --------------------------------------------------------------
            -- E5 (Worley 3): offset to each candidate point -> |dx|,|dy|
            -- and sign (one subtract + one abs per axis per candidate)
            --------------------------------------------------------------
            v_dd := fx4  - signed(resize(h00_4(5 downto 0), 8));  cadx5(0) <= f_abs7(v_dd); csgx5(0) <= v_dd(7);
            v_dd := fy4  - signed(resize(h00_4(11 downto 6), 8)); cady5(0) <= f_abs7(v_dd); csgy5(0) <= v_dd(7);
            v_dd := fxn4 - signed(resize(h10_4(5 downto 0), 8));  cadx5(1) <= f_abs7(v_dd); csgx5(1) <= v_dd(7);
            v_dd := fy4  - signed(resize(h10_4(11 downto 6), 8)); cady5(1) <= f_abs7(v_dd); csgy5(1) <= v_dd(7);
            v_dd := fx4  - signed(resize(h01_4(5 downto 0), 8));  cadx5(2) <= f_abs7(v_dd); csgx5(2) <= v_dd(7);
            v_dd := fyn4 - signed(resize(h01_4(11 downto 6), 8)); cady5(2) <= f_abs7(v_dd); csgy5(2) <= v_dd(7);
            v_dd := fxn4 - signed(resize(h11_4(5 downto 0), 8));  cadx5(3) <= f_abs7(v_dd); csgx5(3) <= v_dd(7);
            v_dd := fyn4 - signed(resize(h11_4(11 downto 6), 8)); cady5(3) <= f_abs7(v_dd); csgy5(3) <= v_dd(7);
            cx0_5 <= cx0_4;  cy0_5 <= cy0_4;  nsx5 <= nsx4;  nsy5 <= nsy4;
            fdist6 <= fdist5; fgate6 <= fgate5; edge6 <= edge5;

            --------------------------------------------------------------
            -- E6 (Worley 4): octagonal distance per candidate --
            -- max(|dx|,|dy|) + (|dx|+|dy|)/2. No squares needed to pick the
            -- nearest point, and the cells come out as irregular polygons.
            --------------------------------------------------------------
            for i in 0 to 3 loop
                if cadx5(i) > cady5(i) then
                    v_mx := resize(cadx5(i), 8);
                else
                    v_mx := resize(cady5(i), 8);
                end if;
                v_sm := resize(cadx5(i), 8) + resize(cady5(i), 8);
                cd6(i)    <= v_mx + ('0' & v_sm(7 downto 1));
                cadx6(i)  <= cadx5(i);  cady6(i) <= cady5(i);
                csgx6(i)  <= csgx5(i);  csgy6(i) <= csgy5(i);
            end loop;
            cx0_6 <= cx0_5;  cy0_6 <= cy0_5;  nsx6 <= nsx5;  nsy6 <= nsy5;
            fdist7 <= fdist6; fgate7 <= fgate6; edge7 <= edge6;

            --------------------------------------------------------------
            -- E7 (Worley 5): all six pairwise compares in parallel (the
            -- shallow-tree argmin), plus the neighbour indices for E8.
            --------------------------------------------------------------
            if cd6(0) <= cd6(1) then c01_7 <= '1'; else c01_7 <= '0'; end if;
            if cd6(0) <= cd6(2) then c02_7 <= '1'; else c02_7 <= '0'; end if;
            if cd6(0) <= cd6(3) then c03_7 <= '1'; else c03_7 <= '0'; end if;
            if cd6(1) <= cd6(2) then c12_7 <= '1'; else c12_7 <= '0'; end if;
            if cd6(1) <= cd6(3) then c13_7 <= '1'; else c13_7 <= '0'; end if;
            if cd6(2) <= cd6(3) then c23_7 <= '1'; else c23_7 <= '0'; end if;
            for i in 0 to 3 loop
                cadx7(i) <= cadx6(i);  cady7(i) <= cady6(i);
                csgx7(i) <= csgx6(i);  csgy7(i) <= csgy6(i);
            end loop;
            cx0_7 <= cx0_6;  cy0_7 <= cy0_6;
            if nsx6 = '1' then cx1_7 <= cx0_6 + 1; else cx1_7 <= cx0_6 - 1; end if;
            if nsy6 = '1' then cy1_7 <= cy0_6 + 1; else cy1_7 <= cy0_6 - 1; end if;
            fdist8 <= fdist7; fgate8 <= fgate7; edge8 <= edge7;

            --------------------------------------------------------------
            -- E8 (Worley 6): winner select + the cell IDENTITY hash of the
            -- winning cell (material joins it, as before). Grain and blob
            -- hashes A are struck here from the free-running warped
            -- position (a few px of skew is invisible in noise).
            --------------------------------------------------------------
            if c01_7 = '1' and c02_7 = '1' and c03_7 = '1' then
                v_wi := "00";
            elsif c01_7 = '0' and c12_7 = '1' and c13_7 = '1' then
                v_wi := "01";
            elsif c02_7 = '0' and c12_7 = '0' and c23_7 = '1' then
                v_wi := "10";
            else
                v_wi := "11";
            end if;
            wadx8 <= cadx7(to_integer(v_wi));
            wady8 <= cady7(to_integer(v_wi));
            wsgx8 <= csgx7(to_integer(v_wi));
            wsgy8 <= csgy7(to_integer(v_wi));
            if v_wi(0) = '1' then v_wcx := cx1_7; else v_wcx := cx0_7; end if;
            if v_wi(1) = '1' then v_wcy := cy1_7; else v_wcy := cy0_7; end if;
            wcx8 <= v_wcx;  wcy8 <= v_wcy;
            v_mx8 := "00" & s_mat;
            v_my8 := s_mat & "00";
            cha8 <= f_hash_a(v_wcx xor v_mx8, v_wcy xor v_my8, x"C731");
            -- MOOD (plain / busy / tall bars) keyed to 4x4 groups of Worley
            -- cells, so a mood area is a union of whole polygons -- no
            -- straight edge anywhere. This replaces the 128px SCREEN-block
            -- mood hash that had drawn a grid of large squares on flat
            -- backgrounds since v0.1.
            mha8 <= f_hash_a("00" & v_wcx(7 downto 2), "00" & v_wcy(7 downto 2), x"6A1D");
            gha8 <= f_hash_a(sxw2(9 downto 2), syw2(9 downto 2), x"9E37");
            bha8 <= f_hash_a(sxw2(10 downto 3), syw2(10 downto 3), x"5B3D");
            fdist9 <= fdist8; fgate9 <= fgate8; edge9 <= edge8;

            --------------------------------------------------------------
            -- E9: cell-hash B (base + overlay salts) | per-cell size /
            -- squash / flip / inversion from the identity hash | signed
            -- in-cell position for checker & speckle bits | grain + blob
            -- wobble | ramp index | whole-cell luma tile address
            --------------------------------------------------------------
            chb9  <= f_hash_b(cha8);
            ochb9 <= f_hash_b(cha8 xor x"8A7");
            -- glyph size: half in a hash-chosen half of the cells, but never
            -- in the bright parts of the picture (bright = bold)
            v_size := (cha8(0) xor cha8(7)) and not s_ysm(7);
            v_sqx  := cha8(1) xor cha8(8);
            v_sqy  := (cha8(2) xor cha8(9)) and not v_sqx;
            adx9 <= f_ashr('0' & wadx8, v_size, v_sqx);
            ady9 <= f_ashr('0' & wady8, v_size, v_sqy);
            flp9 <= cha8(3) xor cha8(10);
            inv9 <= cha8(4) xor cha8(11);
            if wsgx8 = '1' then
                v_l := to_signed(32, 9) - signed(resize(wadx8, 9));
            else
                v_l := to_signed(32, 9) + signed(resize(wadx8, 9));
            end if;
            lxn9 <= unsigned(v_l(5 downto 0));
            if wsgy8 = '1' then
                v_l := to_signed(32, 9) - signed(resize(wady8, 9));
            else
                v_l := to_signed(32, 9) + signed(resize(wady8, 9));
            end if;
            lyn9 <= unsigned(v_l(5 downto 0));

            -- gold-leaf grain: fine sparkle, stronger bright glints and
            -- occasional dark flecks so the leaf glints like metal.
            v_h := f_hash_b(gha8);
            v_grain := resize(signed('0' & v_h(4 downto 0)) - 16, 7);
            if v_h(11 downto 8) = "1111" then
                v_grain := to_signed(52, 7);         -- bright specular glint
            elsif v_h(11 downto 8) = "0000" then
                v_grain := to_signed(-26, 7);        -- dark fleck
            end if;
            grain9 <= v_grain;
            -- blob wobble (+/-64 on r^2, 8px-coherent) folded together with
            -- the per-pixel grain dither into ONE operand for E10's r^2 sum
            v_b  := f_hash_b(bha8);
            wob9 <= (resize(signed('0' & v_b(6 downto 0)), 9) - to_signed(64, 9))
                    + shift_left(resize(v_grain, 9), 1);

            -- gold-ramp index (arithmetic here; ROM read at E10).
            v_gyidx := resize(signed('0' & iy9(9 downto 5)), 13)
                       + resize(s_pat_bias, 13);
            if v_gyidx < 0 then
                ridx9 <= to_unsigned(0, 5);
            elsif v_gyidx > 31 then
                ridx9 <= to_unsigned(31, 5);
            else
                ridx9 <= unsigned(std_logic_vector(v_gyidx(4 downto 0)));
            end if;

            -- whole-cell mask sample: the 16px tile half a pitch ABOVE the
            -- winning cell's top edge, at its centre column. That row is
            -- always already sampled (the lower neighbour is only chosen
            -- from the lower half of a cell) and always inside the 128-line
            -- ring (worst case 2 pitches back at pitch 64).
            case s_k is
                when 4 =>
                    v_row := shift_left(resize(signed('0' & wcy8), 12), 4) - to_signed(8, 12);
                    v_col := shift_left(resize(wcx8, 11), 4) + to_unsigned(8, 11);
                when 5 =>
                    v_row := shift_left(resize(signed('0' & wcy8), 12), 5) - to_signed(16, 12);
                    v_col := shift_left(resize(wcx8, 11), 5) + to_unsigned(16, 11);
                when others =>
                    v_row := shift_left(resize(signed('0' & wcy8), 12), 6) - to_signed(32, 12);
                    v_col := shift_left(resize(wcx8, 11), 6) + to_unsigned(32, 11);
            end case;
            if v_row < 0 then
                v_row := (others => '0');
            end if;
            lram_ra <= unsigned(v_row(6 downto 4)) & v_col(10 downto 4);

            rmood9 <= mha8(1 downto 0) xor s_mat(1 downto 0);
            omood9 <= mha8(3 downto 2) xor s_mat(4 downto 3);
            fdist10 <= fdist9; fgate10 <= fgate9; edge10 <= edge9;

            --------------------------------------------------------------
            -- E6: squared-distance / max (geometry part 2) + ground ramp
            --------------------------------------------------------------
            -- grain-dithered ("hand-cut") geometry, folded into THIS stage
            -- so E7's compare/select chain reads registered values only
            -- (putting the dither adders inside E7 cost ~15 MHz).
            if adx9 > ady9 then
                v_gmm := signed(resize(adx9, 9));
            else
                v_gmm := signed(resize(ady9, 9));
            end if;
            v_gmm := v_gmm + resize(shift_right(wob9, 4), 9);
            if v_gmm < 0 then
                mmax10 <= (others => '0');
            else
                mmax10 <= unsigned(v_gmm(6 downto 0));
            end if;
            -- ORGANIC BLOB METRIC: r^2 (on the per-cell squashed |dx|,|dy|)
            -- plus the 8px wobble+grain operand. Same three-operand shape as
            -- v0.7's sum, which closed. No two blobs share an outline.
            v_gr2 := signed(resize(f_sq(f_abs31(signed('0' & adx9))), 14))
                     + signed(resize(f_sq(f_abs31(signed('0' & ady9))), 14))
                     + resize(wob9, 14);
            if v_gr2 < 0 then
                r2_10 <= (others => '0');
            else
                r2_10 <= unsigned(v_gr2(11 downto 0));
            end if;
            -- BARS: per-cell orientation (x or y key), per-cell phase and
            -- pitch (8 / 16 / 32 / thin-16), fuzzy width. Decided HERE so E7
            -- reads one registered bit. This is what breaks the checkerboard
            -- of identical vertical-line squares.
            if flp9 = '1' then
                v_abk := resize(lyn9, 9);
            else
                v_abk := resize(lxn9, 9);
            end if;
            v_abk := v_abk + (chb9(11 downto 7) & '0');
            case chb9(6 downto 5) is
                when "00" =>
                    if v_abk(2 downto 0) < 4 + ("00" & grain9(0)) then v_barin := '1'; else v_barin := '0'; end if;
                when "01" =>
                    if v_abk(3 downto 0) < 10 + ("000" & grain9(0)) then v_barin := '1'; else v_barin := '0'; end if;
                when "10" =>
                    if v_abk(4 downto 0) < 20 + ("0000" & grain9(0)) then v_barin := '1'; else v_barin := '0'; end if;
                when others =>
                    if v_abk(3 downto 0) < 5 + ("000" & grain9(0)) then v_barin := '1'; else v_barin := '0'; end if;
            end case;
            barin10 <= v_barin;
            -- diagonal checker axes
            dsum10 <= resize(lxn9, 7) + resize(lyn9, 7);
            ddif10 <= resize(lxn9, 7) - resize(lyn9, 7) + to_unsigned(64, 7);
            inv10  <= inv9;
            chb10 <= chb9; rmood10 <= rmood9;
            lxn10 <= lxn9; lyn10 <= lyn9;
            adx10 <= adx9; ady10 <= ady9;
            grain10 <= grain9;

            ochb10 <= ochb9;  omood10 <= omood9;

            v_idx := to_integer(ridx9);
            gyr10 <= C_RAMP_Y(v_idx);
            gur10 <= C_RAMP_U(v_idx);
            gvr10 <= C_RAMP_V(v_idx);
            -- gold creeps from the shadows with GILD
            if iy10 < s_goldceil then
                gild10 <= '1';
            else
                gild10 <= '0';
            end if;

            fdist11 <= fdist10; fgate11 <= fgate10; edge11 <= edge10;

            --------------------------------------------------------------
            -- E7: motif select + evaluate -> layer select (plain-biased)
            --------------------------------------------------------------
            -- hash fields (decorrelated): motif, plain-cov, colour-cell,
            -- jewel hue, gold-shade.
            v_cov  := chb10(7 downto 0);
            v_acc  := chb10(11 downto 4);
            -- jewel cells take their hue from the PICTURE wherever the
            -- picture has a hue of its own; only grey ground falls back to
            -- the cell hash. Leaf shade follows the tonal band the same way.
            if s_sathi = '1' then
                v_jidx := s_jhue;
            else
                v_jidx := chb10(2 downto 0);
            end if;
            v_cvar := chb10(6 downto 4) xor s_mat(5 downto 3);

            -- hand-cut geometry: r2_10 / mmax10 arrive pre-dithered from E6;
            -- add only a per-cell ring PHASE (so no two cells' rings align)
            -- and a fuzzy bar width here.
            v_rph := resize(mmax10, 7) + resize(chb10(1 downto 0) & '0', 7);
            -- per-cell ring pitch (wide or tight)
            if chb10(3) = '1' then
                v_ringv := v_rph(4 downto 2);
            else
                v_ringv := v_rph(5 downto 3);
            end if;

            -- coverage is content-driven (s_covthr_px): bright, saturated
            -- ground blooms into busy ornament, dark flat ground stays plain
            v_plainthr := resize(s_covthr_px, 9);
            if rmood10 = "00" then
                v_plainthr := resize(s_covthr_px, 9) + 56;
            elsif rmood10 = "11" then
                v_plainthr := resize(s_covthr_px, 9) - 24;
            end if;
            v_plain := '0';
            if ('0' & v_cov) < v_plainthr then
                v_plain := '1';
            end if;

            -- colour-cell: a fraction (P5) of ornament cells become a SOLID
            -- bright jewel cell (crimson/lapis/violet...) among the gold.
            v_colcell := '0';
            if v_acc < s_jewthr then
                v_colcell := '1';
            end if;

            -- pillar region: some whole regions (Kiss especially) render as
            -- bold TALL vertical bars keyed to absolute x, so they run
            -- continuously down across cells -- the Kiss-robe rectangles.
            v_pillar := '0';
            if rmood10 = "11" or (s_kiss = '1' and rmood10 = "10") then
                v_pillar := '1';
            end if;

            -- per-cell motif choice (varies cell to cell)
            if s_kiss = '0' then
                case chb10(10 downto 8) is
                    when "000" => v_mtype := "011";   -- concentric
                    when "001" => v_mtype := "101";   -- almond eye
                    when "010" => v_mtype := "110";   -- squared spiral
                    when "011" => v_mtype := "100";   -- disc/rings
                    when "100" => v_mtype := "001";   -- checker (rarer)
                    when "101" => v_mtype := "011";   -- concentric
                    when "110" => v_mtype := "101";   -- almond eye
                    when others => v_mtype := "111";  -- speckle
                end case;
            else
                case chb10(10 downto 8) is
                    when "000" => v_mtype := "010";   -- bars
                    when "001" => v_mtype := "100";   -- disc/rings
                    when "010" => v_mtype := "010";   -- bars
                    when "011" => v_mtype := "100";   -- disc/rings
                    when "100" => v_mtype := "110";   -- squared spiral
                    when "101" => v_mtype := "100";   -- disc/rings
                    when "110" => v_mtype := "011";   -- concentric
                    when others => v_mtype := "111";  -- speckle
                end case;
            end if;
            if s_meadow = '1' and s_mrow = '1' then
                v_mtype := "100";                                  -- rosettes
            end if;

            -- normalized cell space: centre ~32 (+/- per-cell jitter).
            -- Priority: pillar bars > plain gap > colour cell > gold motif.
            v_sel := "00";
            if v_pillar = '1' then
                -- tall vertical bars: ~11px bar + gold gap, fuzzy-edged,
                -- some bars pale or coloured (jewel hue from the column).
                if barin10 = '1' then
                    case chb10(11 downto 9) is
                        when "100"  => v_sel := "10";              -- pale bar
                        when "101"  => v_sel := "11";              -- colour bar
                                       v_jidx := chb10(5 downto 3);
                        when others => v_sel := "01";              -- dark bar
                    end case;
                else
                    v_sel := "00";
                end if;
            elsif v_plain = '1' then
                v_sel := "00";
            elsif v_colcell = '1' then
                -- solid bright colour cell: a filled square or disc, framed
                if chb10(9) = '1' then                              -- disc
                    if r2_10 < 400 then
                        v_sel := "11";
                    elsif r2_10 < 560 then
                        v_sel := "01";
                    else
                        v_sel := "00";
                    end if;
                else                                               -- square
                    if mmax10 <= 24 then
                        v_sel := "11";
                    elsif mmax10 <= 28 then
                        v_sel := "01";
                    else
                        v_sel := "00";
                    end if;
                end if;
            else
                case v_mtype is
                    when "001" =>   -- checker: gold / ink / bone (2x2),
                                    -- axis-aligned or DIAGONAL per cell
                        if chb10(11) = '1' then
                            v_sub(1) := dsum10(5) xor ddif10(5);
                            v_sub(0) := dsum10(4) xor ddif10(4);
                        else
                            v_sub(1) := lxn10(5) xor lyn10(5);
                            v_sub(0) := lxn10(4) xor lyn10(4);
                        end if;
                        case v_sub is
                            when "00"   => v_sel := "00";
                            when "11"   => v_sel := "01";
                            when "10"   => v_sel := "10";
                            when others => v_sel := "00";
                        end case;
                    when "010" =>   -- bars (per-cell axis / pitch / phase)
                        if barin10 = '1' then
                            v_sel := "01";
                        else
                            v_sel := "00";
                        end if;
                    when "011" =>   -- concentric squares (per-cell phase+pitch)
                        v_ring := v_ringv;
                        case v_ring(1 downto 0) is
                            when "00"   => v_sel := "01";
                            when "01"   => v_sel := "00";
                            when "10"   => v_sel := "10";
                            when others => v_sel := "00";
                        end case;
                    when "100" =>   -- discs / rings (dark centre, pale ring)
                        if r2_10 < 64 then
                            v_sel := "01";
                        elsif r2_10 < 256 then
                            v_sel := "10";
                        elsif r2_10 < 576 then
                            v_sel := "00";
                        else
                            v_sel := "00";
                        end if;
                    when "101" =>   -- almond eye: upward triangle + oval
                        v_tri := resize(to_signed(32, 9) - signed('0' & ady10), 9);
                        if v_tri >= signed('0' & adx10) then
                            if r2_10 < 100 then
                                v_sel := "01";                     -- iris
                            else
                                v_sel := "10";                     -- pale sclera
                            end if;
                        else
                            v_sel := "00";
                        end if;
                    when "110" =>   -- squared spiral (per-cell phase+pitch)
                        v_ring := v_ringv;
                        if lxn10(3) = '1' and v_ring(0) = '1' then
                            v_sel := "00";
                        elsif v_ring(0) = '0' then
                            v_sel := "01";
                        else
                            v_sel := "00";
                        end if;
                    when others =>  -- dense speckle of small gold flakes
                        if chb10(9) = '1' and lxn10(2) = '1' and lyn10(2) = '1' then
                            v_sel := "10";
                        else
                            v_sel := "00";
                        end if;
                end case;
            end if;

            -- per-cell dark<->light INVERSION: the same motif reads as ink-
            -- on-gold in one cell and gold-on-ink in the next
            if inv10 = '1' then
                case v_sel is
                    when "01"   => v_sel := "10";
                    when "10"   => v_sel := "01";
                    when others => null;
                end case;
            end if;
            msel11 <= v_sel;
            jidx11 <= v_jidx;
            cvar11 <= v_cvar;
            -- apply patina warmth/cool to the raw ramp chroma (index-shift
            -- part of patina was folded into ridx9 at E5). The leaf GRAIN
            -- is folded into gy11 HERE -- a per-pixel grain adder inside
            -- E8's compose mux chain was on the critical path.
            gy11 <= f_cu10(resize(signed('0' & gyr10), 13) + resize(grain10, 13));
            gu11 <= f_cu10(resize(signed('0' & gur10), 12) + resize(s_pat_u, 12));
            gv11 <= f_cu10(resize(signed('0' & gvr10), 12) + resize(s_pat_v, 12));
            gild11 <= gild10;

            -- OVERLAY motif select: its own bank -- bold bars, checker,
            -- concentric squares, big rings -- busier than the base (it is
            -- a reveal) and dark-on-pale, the inverse of the base emphasis,
            -- so the revealed patch reads as a DIFFERENT gilt surface.
            v_osel := "00";
            v_rph  := resize(mmax10, 7) + resize(ochb10(1 downto 0) & '0', 7);
            if omood10 = "11" or omood10 = "10" then
                -- bold tall bars keyed to absolute x (run across cells)
                if barin10 = '1' then
                    v_osel := "01";
                end if;
            elsif ochb10(7 downto 4) < "0011" then
                v_osel := "00";                          -- breathing room
            else
                case ochb10(10 downto 8) is
                    when "000" | "011" =>                -- concentric squares
                        case v_rph(5 downto 3) is
                            when "000" | "011" | "110" => v_osel := "01";
                            when others => v_osel := "00";
                        end case;
                    when "001" =>                        -- big disc + outer ring
                        if r2_10 < 256 then
                            v_osel := "01";
                        elsif r2_10 < 484 then
                            v_osel := "00";
                        elsif r2_10 < 676 then
                            v_osel := "01";
                        else
                            v_osel := "00";
                        end if;
                    when "010" =>                        -- checker
                        if (lxn10(4) xor lyn10(4)) = '1' then
                            v_osel := "01";
                        end if;
                    when "100" =>                        -- solid jewel cell
                        if mmax10 <= 26 then
                            if s_jew_on = '1' then
                                v_osel := "11";
                            else
                                v_osel := "01";
                            end if;
                        end if;
                    when "101" =>                        -- short bars
                        if barin10 = '1' then
                            v_osel := "01";
                        end if;
                    when others =>                       -- speckle flakes
                        if ochb10(3) = '1' and lxn10(2) = '1' and lyn10(2) = '1' then
                            v_osel := "01";
                        end if;
                end case;
            end if;
            v_ocvar := ochb10(6 downto 4) xor s_mat(5 downto 3);
            oosel11 <= v_osel;
            ocvar11 <= v_ocvar;
            if s_sathi = '1' then
                ojidx11 <= s_jhue;
            else
                ojidx11 <= ochb10(2 downto 0);
            end if;
            -- pre-grained overlay GROUND: pale gold, per-cell shade (C_LT)
            og_y11 <= f_cu10(resize(signed('0' & C_LT_Y(to_integer(v_ocvar))), 13)
                            + resize(grain10, 13));
            og_u11 <= C_LT_U(to_integer(v_ocvar));
            og_v11 <= C_LT_V(to_integer(v_ocvar));
            -- luma MASK: S8 picks the edge -- On = clean per-pixel cut,
            -- Off = whole cells (quilt-style full glyphs)
            if s_outln = '1' then
                if iy10 >= s_luma_thr then
                    omask11 <= '1';
                else
                    omask11 <= '0';
                end if;
            else
                -- whole cells: the luma tile just above this cell's top edge
                -- (always already sampled, always inside the 128-line ring)
                if lram_q >= s_luma_thr then
                    omask11 <= '1';
                else
                    omask11 <= '0';
                end if;
            end if;

            fdist12 <= fdist11; fgate12 <= fgate11; edge12 <= edge11;

            --------------------------------------------------------------
            -- E8: compose ground (+grain +tessera +meadow) with ornament
            --------------------------------------------------------------
            v_by := gy11; v_bu := gu11; v_bv := gv11;   -- gy11 arrives pre-grained

            if s_meadow = '1' and s_mrow = '1' then
                v_by := ('0' & v_by(9 downto 1)) + to_unsigned(120, 10);
                v_bu := v_bu - resize(v_bu(9 downto 4), 10);
                v_bv := v_bv - resize(v_bv(9 downto 4), 10);
            end if;

            if gild11 = '0' then
                v_by := unsigned(dy_sr(10));
                v_bu := unsigned(du_sr(10));
                v_bv := unsigned(dv_sr(10));
                for k in 0 to 3 loop
                    if s_flat(k) = '1' then
                        v_bu := ('0' & v_bu(9 downto 1)) + to_unsigned(256, 10);
                        v_bv := ('0' & v_bv(9 downto 1)) + to_unsigned(256, 10);
                    end if;
                end loop;
            end if;

            if gild11 = '1' then
                case msel11 is
                    when "01" =>    -- dark ornament (per-cell warm-black shade)
                        wy12 <= C_DK_Y(to_integer(cvar11));
                        wu12 <= C_DK_U(to_integer(cvar11));
                        wv12 <= C_DK_V(to_integer(cvar11));
                    when "10" =>    -- light ornament (per-cell gold/bone shade)
                        wy12 <= C_LT_Y(to_integer(cvar11));
                        wu12 <= C_LT_U(to_integer(cvar11));
                        wv12 <= C_LT_V(to_integer(cvar11));
                    when "11" =>    -- solid jewel colour (muted a touch)
                        v_ju := C_JW_U(to_integer(jidx11));
                        v_jv := C_JW_V(to_integer(jidx11));
                        v_du2 := resize(signed('0' & v_ju), 12) - to_signed(512, 12);
                        v_dv2 := resize(signed('0' & v_jv), 12) - to_signed(512, 12);
                        -- knock the vivid down to 3/4 amplitude so the colour
                        -- sits with the gold instead of shouting over it;
                        -- P5 (s_jsat) desaturates further from there.
                        v_du2 := v_du2 - shift_right(v_du2, 2);
                        v_dv2 := v_dv2 - shift_right(v_dv2, 2);
                        v_du2 := shift_right(v_du2, s_jsat);
                        v_dv2 := shift_right(v_dv2, s_jsat);
                        wy12 <= C_JW_Y(to_integer(jidx11));
                        wu12 <= f_cu10(to_signed(512, 12) + v_du2);
                        wv12 <= f_cu10(to_signed(512, 12) + v_dv2);
                    when others =>
                        wy12 <= v_by; wu12 <= v_bu; wv12 <= v_bv;
                end case;
            else
                wy12 <= v_by; wu12 <= v_bu; wv12 <= v_bv;
            end if;

            -- LUMA MOD overlay: where the mask is on (per-pixel cut or
            -- whole cells, S8), a SECOND gilt world paints over the base --
            -- pale-gold per-cell ground with dark ornament, its own cells,
            -- seams and jewels. The base render underneath stays untouched.
            if s_luma = '1' and omask11 = '1' then
                case oosel11 is
                    when "01" =>                 -- dark ornament, per-cell shade
                        wy12 <= C_DK_Y(to_integer(ocvar11));
                        wu12 <= C_DK_U(to_integer(ocvar11));
                        wv12 <= C_DK_V(to_integer(ocvar11));
                    when "11" =>                 -- jewel cell (muted like base)
                        v_ju := C_JW_U(to_integer(ojidx11));
                        v_jv := C_JW_V(to_integer(ojidx11));
                        v_du2 := resize(signed('0' & v_ju), 12) - to_signed(512, 12);
                        v_dv2 := resize(signed('0' & v_jv), 12) - to_signed(512, 12);
                        v_du2 := v_du2 - shift_right(v_du2, 2);
                        v_dv2 := v_dv2 - shift_right(v_dv2, 2);
                        v_du2 := shift_right(v_du2, s_jsat);
                        v_dv2 := shift_right(v_dv2, s_jsat);
                        wy12 <= C_JW_Y(to_integer(ojidx11));
                        wu12 <= f_cu10(to_signed(512, 12) + v_du2);
                        wv12 <= f_cu10(to_signed(512, 12) + v_dv2);
                    when others =>               -- pale-gold overlay ground
                        wy12 <= og_y11; wu12 <= og_u11; wv12 <= og_v11;
                end case;
            end if;

            fdist13 <= fdist12; fgate13 <= fgate12;

            --------------------------------------------------------------
            -- E9: contour ink + halo
            --------------------------------------------------------------
            -- (both gated off in Luma mode: S8 is repurposed there as the
            -- overlay edge switch, and the bright flesh region is owned by
            -- the overlay layer)
            if s_luma = '0' and s_outln = '1' and s_ctr_on = '1'
               and edge12 > C_EDGETHR then
                wy13 <= '0' & C_INK_Y(9 downto 1);
                wu13 <= C_INK_U;
                wv13 <= C_INK_V;
            elsif s_luma = '0' and s_halo = '1' and s_halo_amt /= 0
                  and fgate12 = '1'
                  and fdist12 > resize(s_flesh_w, 11)
                  and fdist12 < (resize(s_flesh_w, 11) + 48) then
                v_delta := resize(signed('0' & s_halo_amt), 7) + to_signed(16, 7);
                v_hy := f_cu10(resize(signed('0' & wy12), 13)
                               + shift_left(resize(v_delta, 13), 2));
                wy13 <= v_hy;
                wu13 <= wu12 - resize(wu12(9 downto 4), 10);
                wv13 <= f_cu10(resize(signed('0' & wv12), 12)
                              + resize(signed('0' & wv12(9 downto 4)), 12));
            else
                wy13 <= wy12; wu13 <= wu12; wv13 <= wv12;
            end if;

            --------------------------------------------------------------
            -- E10: flesh restore (hard-edged warmed original)
            --------------------------------------------------------------
            v_oy := unsigned(dy_sr(12));
            v_ou := unsigned(du_sr(12));
            v_ov := unsigned(dv_sr(12));
            -- Luma mode is fully synthetic -> no flesh-restore (the raw video
            -- must not show through); otherwise restore warmed skin.
            if s_luma = '0' and fgate13 = '1' and fdist13 < resize(s_flesh_w, 11) then
                wy14 <= v_oy;
                wu14 <= v_ou - resize(v_ou(9 downto 5), 10);
                wv14 <= f_cu10(resize(signed('0' & v_ov), 12)
                               + resize(signed('0' & v_ov(9 downto 5)), 12));
            else
                wy14 <= wy13; wu14 <= wu13; wv14 <= wv13;
            end if;

            --------------------------------------------------------------
            -- line / field bookkeeping
            --------------------------------------------------------------
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_x_count <= (others => '0');
                s_lgood   <= '0';
                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 2000 then
                        s_aline <= s_aline + 1;
                    end if;
                else
                    s_aline <= (others => '0');
                end if;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_aline <= (others => '0');
            end if;

        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- per-tile luma RAM (Luma object mode): one averaged luma per 16px brick
    ------------------------------------------------------------------------
    p_lram : process(clk)
    begin
        if rising_edge(clk) then
            if lram_we = '1' then
                lram(to_integer(lram_wa)) <= lram_wd;
            end if;
            lram_q <= lram(to_integer(lram_ra));
        end if;
    end process p_lram;

    ------------------------------------------------------------------------
    -- GILD dry/wet mix, inline: result = a + (t8 * (b - a)) >> 8.
    -- wet_final (wy14) is 14 regs from data_in; dry tap dy_sr(13) matches.
    ------------------------------------------------------------------------
    p_mix : process(clk)
        variable v_s : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            m1_ay <= unsigned(dy_sr(13));
            m1_au <= unsigned(du_sr(13));
            m1_av <= unsigned(dv_sr(13));
            m1_dy <= signed(resize(wy14, 11)) - signed(resize(unsigned(dy_sr(13)), 11));
            m1_du <= signed(resize(wu14, 11)) - signed(resize(unsigned(du_sr(13)), 11));
            m1_dv <= signed(resize(wv14, 11)) - signed(resize(unsigned(dv_sr(13)), 11));
            m1_t8y <= s_mix10(9 downto 2);
            m1_t8u <= s_mix10(9 downto 2);
            m1_t8v <= s_mix10(9 downto 2);

            m2_py <= signed('0' & m1_t8y) * m1_dy;
            m2_pu <= signed('0' & m1_t8u) * m1_du;
            m2_pv <= signed('0' & m1_t8v) * m1_dv;
            m2_ay <= m1_ay; m2_au <= m1_au; m2_av <= m1_av;

            v_s := signed(resize(m2_ay, 13)) + resize(m2_py(19 downto 8), 13);
            m3_y <= f_cu10(v_s);
            v_s := signed(resize(m2_au, 13)) + resize(m2_pu(19 downto 8), 13);
            m3_u <= f_cu10(v_s);
            v_s := signed(resize(m2_av, 13)) + resize(m2_pv(19 downto 8), 13);
            m3_v <= f_cu10(v_s);
        end if;
    end process p_mix;

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

    data_out.y       <= std_logic_vector(m3_y);
    data_out.u       <= std_logic_vector(m3_v);   -- swap internal back to HW
    data_out.v       <= std_logic_vector(m3_u);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture gilt;
