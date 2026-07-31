-- gilt.vhd
--
-- GILT -- live video re-rendered as a Gustav Klimt "Golden Phase" painting.
--
-- Flesh stays painted and human; everything else is GILDED -- flattened into
-- a shimmering field of gold leaf, mosaic tesserae and hand-placed ornament.
-- The signature is the hard COLLISION between the two worlds (a cheek ends,
-- a gold mosaic begins) and ornament that is placed, never tiled.
--
--   Signal flow (streaming, no frame buffer, C_LATENCY = 13):
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

    constant C_LATENCY : integer := 13;

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
    signal s_mosaic : std_logic := '1';   -- S7
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
    signal s_subthr   : unsigned(7 downto 0) := to_unsigned(116, 8); -- quadtree
    signal s_covthr   : unsigned(7 downto 0) := to_unsigned(150, 8);
    signal s_jewthr   : unsigned(7 downto 0) := to_unsigned(60, 8);
    signal s_jsat     : integer range 0 to 3 := 2;
    signal s_drift    : unsigned(7 downto 0) := (others => '0');
    signal s_drift_on : std_logic := '0';
    signal s_mix10    : unsigned(9 downto 0) := (others => '0');
    signal s_goldceil : unsigned(9 downto 0) := (others => '1');
    signal s_orn_on   : std_logic := '1';
    signal s_ctr_on   : std_logic := '1';
    signal s_flat     : unsigned(3 downto 0) := (others => '0');
    signal s_jew_on   : std_logic := '1';
    signal s_halo_amt : unsigned(3 downto 0) := (others => '0');
    constant C_EDGETHR : unsigned(9 downto 0) := to_unsigned(56, 10);

    ----------------------------------------------------------------------
    -- OVERLAY (Luma-mod second gilt field): its own quadtree / motifs,
    -- revealed only through the luma mask (S8: clean cut / whole cells).
    ----------------------------------------------------------------------
    signal omood2  : unsigned(1 downto 0) := (others => '0');
    signal oshear2, ovsh2 : signed(5 downto 0) := (others => '0');
    signal osh3    : integer range 3 to 6 := 5;
    signal osxr3   : unsigned(10 downto 0) := (others => '0');
    signal osyr3   : unsigned(9 downto 0)  := (others => '0');
    signal omood3, omood4, omood5, omood6 : unsigned(1 downto 0) := (others => '0');
    signal olxn4, olyn4, olxn5, olyn5, olxn6, olyn6
        : unsigned(5 downto 0) := (others => '0');
    signal ocha4   : unsigned(11 downto 0) := (others => '0');
    signal oseam4, oseam5, oseam6, oseam7 : std_logic := '0';
    signal oabx4, oabx5, oabx6 : unsigned(8 downto 0) := (others => '0');
    signal ochb5, ochb6 : unsigned(11 downto 0) := (others => '0');
    signal oadx5, oady5 : unsigned(6 downto 0) := (others => '0');
    signal omax6   : unsigned(6 downto 0) := (others => '0');
    signal or2_6   : unsigned(11 downto 0) := (others => '0');
    signal obit6   : std_logic := '0';
    signal oosel7  : unsigned(1 downto 0) := (others => '0');
    signal ocvar7, ojidx7 : unsigned(2 downto 0) := (others => '0');
    signal og_y7, og_u7, og_v7 : unsigned(9 downto 0) := (others => '0');
    signal omask7  : std_logic := '0';

    signal s_drift_acc : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- position / line bookkeeping
    ----------------------------------------------------------------------
    signal s_x_count : unsigned(10 downto 0) := (others => '0');
    signal s_aline   : unsigned(9 downto 0)  := (others => '0');
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
    type t_lram is array (0 to 511) of unsigned(9 downto 0);
    signal lram    : t_lram := (others => (others => '0'));
    signal lram_ra : unsigned(8 downto 0) := (others => '0');
    signal lram_wa : unsigned(8 downto 0) := (others => '0');
    signal lram_wd : unsigned(9 downto 0) := (others => '0');
    signal lram_we : std_logic := '0';
    signal lram_q  : unsigned(9 downto 0) := (others => '0');
    signal s_lacc  : unsigned(13 downto 0) := (others => '0');
    signal s_lgood : std_logic := '0';

    ----------------------------------------------------------------------
    -- pixel pipeline registers
    ----------------------------------------------------------------------
    signal iy1, iu1, iv1 : unsigned(9 downto 0) := (others => '0');
    signal px1 : unsigned(10 downto 0) := (others => '0');
    signal luma8_1 : unsigned(7 downto 0) := (others => '0');
    signal ph8_1   : unsigned(7 downto 0) := (others => '0');

    signal iy2, iy3, iy4, iy5, iy6 : unsigned(9 downto 0) := (others => '0');

    -- flesh/edge carries (fdist,fgate to age 9; edge to age 8)
    signal fdist2, fdist3, fdist4, fdist5, fdist6, fdist7, fdist8, fdist9
        : unsigned(10 downto 0) := (others => '0');
    signal fgate2, fgate3, fgate4, fgate5, fgate6, fgate7, fgate8, fgate9
        : std_logic := '0';
    signal edge2, edge3, edge4, edge5, edge6, edge7, edge8
        : unsigned(9 downto 0) := (others => '0');

    signal px2, px3 : unsigned(10 downto 0) := (others => '0');

    -- E2 coarse region (single-round hash): motif mood + lattice shear
    signal rmood2 : unsigned(1 downto 0) := (others => '0');
    signal rshear2, rvsh2 : signed(5 downto 0) := (others => '0');

    -- E3 quadtree resolve: per-pixel cell shift (3..6 => 8..64px) + coords
    signal sh3    : integer range 3 to 6 := 5;
    signal sxr3   : unsigned(10 downto 0) := (others => '0');
    signal syr3   : unsigned(9 downto 0)  := (others => '0');
    signal rmood3 : unsigned(1 downto 0)  := (others => '0');

    -- E4 : normalized cell coords (each cell mapped to 0..63) + cell-hash A
    signal lxn4, lyn4 : unsigned(5 downto 0) := (others => '0');
    signal cha4     : unsigned(11 downto 0) := (others => '0');
    signal rmood4   : unsigned(1 downto 0) := (others => '0');
    signal seam4    : std_logic := '0';
    signal gha4     : unsigned(11 downto 0) := (others => '0');   -- grain hash A
    signal abx4     : unsigned(8 downto 0) := (others => '0');    -- abs sheared x

    -- E5 : cell-hash B + abs offsets (geometry split, part 1)
    signal chb5     : unsigned(11 downto 0) := (others => '0');
    signal adx5, ady5 : unsigned(6 downto 0) := (others => '0');
    signal rmood5   : unsigned(1 downto 0) := (others => '0');
    signal lxn5, lyn5 : unsigned(5 downto 0) := (others => '0');
    signal seam5    : std_logic := '0';
    signal grain5   : signed(6 downto 0) := (others => '0');
    signal ridx5    : unsigned(4 downto 0) := (others => '0');   -- ramp index
    signal abx5     : unsigned(8 downto 0) := (others => '0');

    -- E6 : squared distance / max (geometry split, part 2) + ground
    signal chb6     : unsigned(11 downto 0) := (others => '0');
    signal adx6, ady6 : unsigned(6 downto 0) := (others => '0');
    signal mmax6    : unsigned(6 downto 0) := (others => '0');
    signal r2_6     : unsigned(11 downto 0) := (others => '0');
    signal rmood6   : unsigned(1 downto 0) := (others => '0');
    signal lxn6, lyn6 : unsigned(5 downto 0) := (others => '0');
    signal seam6    : std_logic := '0';
    signal grain6   : signed(6 downto 0) := (others => '0');
    signal gyr6, gur6, gvr6 : unsigned(9 downto 0) := (others => '0'); -- raw ramp
    signal gild6    : std_logic := '0';
    signal abx6     : unsigned(8 downto 0) := (others => '0');

    -- E7 : motif select + eval
    signal msel7   : unsigned(1 downto 0) := (others => '0');
    signal jidx7   : unsigned(2 downto 0) := (others => '0');   -- jewel hue
    signal cvar7   : unsigned(2 downto 0) := (others => '0');   -- gold-shade
    signal gy7, gu7, gv7 : unsigned(9 downto 0) := (others => '0');
    signal seam7   : std_logic := '0';
    signal gild7   : std_logic := '0';

    -- E8 compose, E9 contour/halo, E10 flesh restore
    signal wy8, wu8, wv8 : unsigned(9 downto 0) := (others => '0');
    signal wy9, wu9, wv9 : unsigned(9 downto 0) := (others => '0');
    signal wy10, wu10, wv10 : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- dry delay (original video) + sync delay + inline mix
    ----------------------------------------------------------------------
    type t_dsr is array (0 to 9) of std_logic_vector(9 downto 0);
    signal dy_sr, du_sr, dv_sr : t_dsr := (others => (others => '0'));

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

    signal m1_ay, m1_au, m1_av : unsigned(9 downto 0) := (others => '0');
    signal m1_dy, m1_du, m1_dv : signed(10 downto 0) := (others => '0');
    signal m1_t8               : unsigned(7 downto 0) := (others => '0');
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
                s_mosaic <= registers_in(6)(0);
                s_outln  <= registers_in(6)(1);
                s_luma   <= registers_in(6)(2);      -- S9 = Luma mode
                s_kiss   <= registers_in(6)(3);
                s_halo   <= registers_in(6)(4);
                s_meadow <= registers_in(6)(3);      -- meadow auto-on in Kiss
                s_p12 <= unsigned(registers_in(7));

                if unsigned(registers_in(5)) > 8 then
                    s_drift_acc <= s_drift_acc
                        + resize(unsigned(registers_in(5)(9 downto 6)) + 1, 8);
                    s_drift_on  <= '1';
                else
                    s_drift_on  <= '0';
                end if;

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
                        s_subthr <= to_unsigned(40, 8)
                                    + resize(shift_right(1023 - s_p4, 3), 8);
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
                        if s_drift_on = '1' then
                            s_drift <= s_drift_acc;
                        else
                            s_drift <= (others => '0');
                        end if;
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
    -- main pixel pipeline
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_du, v_dv   : signed(11 downto 0);
        variable v_eh, v_ev   : signed(8 downto 0);
        variable v_rx, v_ry   : unsigned(7 downto 0);
        variable v_cx, v_cy   : unsigned(7 downto 0);
        variable v_sx         : unsigned(10 downto 0);
        variable v_sy         : unsigned(9 downto 0);
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
        variable v_bw         : unsigned(3 downto 0);
        variable v_osx        : unsigned(10 downto 0);
        variable v_osy        : unsigned(9 downto 0);
        variable v_osh        : integer range 3 to 6;
        variable v_oxo        : unsigned(10 downto 0);
        variable v_oyo        : unsigned(9 downto 0);
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
            for i in 1 to 9 loop
                dy_sr(i) <= dy_sr(i - 1);
                du_sr(i) <= du_sr(i - 1);
                dv_sr(i) <= dv_sr(i - 1);
            end loop;

            iy2 <= iy1; iy3 <= iy2; iy4 <= iy3; iy5 <= iy4; iy6 <= iy5;

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

            -- coarse 128px region (single-round hash): motif mood + a
            -- per-region shear of the cell lattice so neighbouring regions
            -- never line up.
            v_rx := resize(px1(10 downto 7), 8);
            v_ry := resize(s_aline(9 downto 7), 8);
            v_h  := f_hash_a(v_rx, v_ry, x"6A1D");
            rmood2  <= v_h(1 downto 0);
            rshear2 <= resize(signed('0' & v_h(6 downto 3)), 6) - 8;
            rvsh2   <= resize(signed('0' & v_h(10 downto 7)), 6) - 8;
            -- OVERLAY field: an independent coarse region (own salt) with
            -- its own mood + shear, so the second gilt world never aligns
            -- with the base ornament.
            v_hs := f_hash_a(v_rx, v_ry, x"3B7F");
            omood2  <= v_hs(1 downto 0);
            oshear2 <= resize(signed('0' & v_hs(6 downto 3)), 6) - 8;
            ovsh2   <= resize(signed('0' & v_hs(10 downto 7)), 6) - 8;
            px2  <= px1;

            fdist3 <= fdist2; fgate3 <= fgate2; edge3 <= edge2;

            --------------------------------------------------------------
            -- E3: QUADTREE cell-size resolve. Apply the region shear, then
            -- test three independent coarse-block hashes (64/32/16 px): a
            -- block that "subdivides" resolves finer, its neighbour may not.
            -- Result: big plain cells sit beside clusters of tiny ones and
            -- cell edges never fall on a single grid -- NOT a lattice.
            --------------------------------------------------------------
            v_sx := px2 + unsigned(resize(rshear2, 11));
            v_sy := s_aline + unsigned(resize(rvsh2, 10));
            v_hs := f_hash_a(resize(shift_right(v_sx, 6), 8),
                             resize(shift_right(v_sy, 6), 8), x"51A3");
            if v_hs(7 downto 0) < s_subthr then v_s6 := '1'; else v_s6 := '0'; end if;
            v_hs := f_hash_a(resize(shift_right(v_sx, 5), 8),
                             resize(shift_right(v_sy, 5), 8), x"7C2B");
            if v_hs(7 downto 0) < s_subthr then v_s5 := '1'; else v_s5 := '0'; end if;
            v_hs := f_hash_a(resize(shift_right(v_sx, 4), 8),
                             resize(shift_right(v_sy, 4), 8), x"A93D");
            if v_hs(7 downto 0) < s_subthr then v_s4 := '1'; else v_s4 := '0'; end if;
            if v_s6 = '0' then
                v_sh := 6;
            elsif v_s5 = '0' then
                v_sh := 5;
            elsif v_s4 = '0' then
                v_sh := 4;
            else
                v_sh := 3;
            end if;
            sh3     <= v_sh;
            sxr3    <= v_sx;
            syr3    <= v_sy;
            rmood3  <= rmood2;
            px3     <= px2;

            -- OVERLAY quadtree resolve (own salts): a full second multi-
            -- scale irregular cell field, revealed through the luma mask.
            v_osx := px2 + unsigned(resize(oshear2, 11));
            v_osy := s_aline + unsigned(resize(ovsh2, 10));
            v_hs := f_hash_a(resize(shift_right(v_osx, 6), 8),
                             resize(shift_right(v_osy, 6), 8), x"D2E9");
            if v_hs(7 downto 0) < s_subthr then v_s6 := '1'; else v_s6 := '0'; end if;
            v_hs := f_hash_a(resize(shift_right(v_osx, 5), 8),
                             resize(shift_right(v_osy, 5), 8), x"1F8B");
            if v_hs(7 downto 0) < s_subthr then v_s5 := '1'; else v_s5 := '0'; end if;
            v_hs := f_hash_a(resize(shift_right(v_osx, 4), 8),
                             resize(shift_right(v_osy, 4), 8), x"C45D");
            if v_hs(7 downto 0) < s_subthr then v_s4 := '1'; else v_s4 := '0'; end if;
            if v_s6 = '0' then
                v_osh := 6;
            elsif v_s5 = '0' then
                v_osh := 5;
            elsif v_s4 = '0' then
                v_osh := 4;
            else
                v_osh := 3;
            end if;
            osh3   <= v_osh;
            osxr3  <= v_osx;
            osyr3  <= v_osy;
            omood3 <= omood2;

            fdist4 <= fdist3; fgate4 <= fgate3; edge4 <= edge3;

            --------------------------------------------------------------
            -- E4: in-cell coords normalized to 0..63 (motifs become size-
            -- independent) + cell-hash A + tessera crackle on cell edges
            --------------------------------------------------------------
            case sh3 is
                when 3 => v_mask := to_unsigned(7, 7);
                when 4 => v_mask := to_unsigned(15, 7);
                when 5 => v_mask := to_unsigned(31, 7);
                when others => v_mask := to_unsigned(63, 7);
            end case;
            v_lx := resize(sxr3 and resize(v_mask, 11), 6);
            v_ly := resize(syr3 and resize(v_mask, 10), 6);
            lxn4 <= shift_left(v_lx, 6 - sh3);
            lyn4 <= shift_left(v_ly, 6 - sh3);
            v_cx := resize(shift_right(sxr3, sh3), 8);
            v_cy := resize(shift_right(syr3, sh3), 8);
            cha4 <= f_hash_a(v_cx, v_cy, x"C731");
            rmood4 <= rmood3;

            -- grain hash A (split; hash B completes at E5)
            v_h  := f_hash_a(px3(9 downto 2) + s_drift, s_aline(9 downto 2), x"9E37");
            gha4 <= v_h;

            -- 1px dark seam on the top/left edge of every (irregular) cell,
            -- WOBBLED +/-1px by the grain hash so the crack wanders like
            -- hand-fractured leaf instead of a ruled line.
            if (v_lx = 0 and v_h(0) = '0') or (v_lx = 1 and v_h(0) = '1')
               or (v_ly = 0 and v_h(1) = '0') or (v_ly = 1 and v_h(1) = '1') then
                seam4 <= '1';
            else
                seam4 <= '0';
            end if;

            -- per-tile luma sampler: average the TOP line of every 16x16
            -- screen tile into the 4-stripe ring (address {aline(5:4), col}).
            -- This is the Luma-mod mask source.
            lram_we <= '0';
            if s_avid_sr(2) = '1' and s_aline(3 downto 0) = 0 then
                if px3(3 downto 0) = 0 then
                    s_lacc  <= resize(iy3, 14);
                    s_lgood <= '1';
                elsif s_lgood = '1' then
                    s_lacc <= s_lacc + resize(iy3, 14);
                end if;
                if px3(3 downto 0) = 15 and s_lgood = '1' then
                    lram_wa <= s_aline(5 downto 4) & px3(10 downto 4);
                    lram_wd <= resize(shift_right(s_lacc + resize(iy3, 14), 4), 10);
                    lram_we <= '1';
                end if;
            end if;

            -- OVERLAY in-cell coords / cell hash / crackle seam (mirrors
            -- the base engine with its own salts and wobble bits)
            case osh3 is
                when 3 => v_mask := to_unsigned(7, 7);
                when 4 => v_mask := to_unsigned(15, 7);
                when 5 => v_mask := to_unsigned(31, 7);
                when others => v_mask := to_unsigned(63, 7);
            end case;
            v_lx := resize(osxr3 and resize(v_mask, 11), 6);
            v_ly := resize(osyr3 and resize(v_mask, 10), 6);
            olxn4 <= shift_left(v_lx, 6 - osh3);
            olyn4 <= shift_left(v_ly, 6 - osh3);
            v_cx := resize(shift_right(osxr3, osh3), 8);
            v_cy := resize(shift_right(osyr3, osh3), 8);
            ocha4 <= f_hash_a(v_cx, v_cy, x"58A7");
            if (v_lx = 0 and v_h(2) = '0') or (v_lx = 1 and v_h(2) = '1')
               or (v_ly = 0 and v_h(3) = '0') or (v_ly = 1 and v_h(3) = '1') then
                oseam4 <= '1';
            else
                oseam4 <= '0';
            end if;
            oabx4  <= osxr3(8 downto 0);
            omood4 <= omood3;
            -- mask-B sample address: the tile holding THIS CELL'S ORIGIN,
            -- so a cell of any size is included whole (quilt-style); the
            -- 4-stripe ring exactly covers the tallest (64px) cell.
            case osh3 is
                when 3 =>
                    v_oxo := osxr3(10 downto 3) & "000";
                    v_oyo := osyr3(9 downto 3) & "000";
                when 4 =>
                    v_oxo := osxr3(10 downto 4) & "0000";
                    v_oyo := osyr3(9 downto 4) & "0000";
                when 5 =>
                    v_oxo := osxr3(10 downto 5) & "00000";
                    v_oyo := osyr3(9 downto 5) & "00000";
                when others =>
                    v_oxo := osxr3(10 downto 6) & "000000";
                    v_oyo := osyr3(9 downto 6) & "000000";
            end case;
            lram_ra <= v_oyo(5 downto 4) & v_oxo(10 downto 4);
            -- absolute sheared x (low bits) -> tall vertical bars that run
            -- continuously across stacked cells (the Kiss robe rectangles).
            abx4 <= sxr3(8 downto 0);

            fdist5 <= fdist4; fgate5 <= fgate4; edge5 <= edge4;

            --------------------------------------------------------------
            -- E5: cell-hash B -> motif fields | dx,dy -> |dx|,|dy|
            --       (geometry split part 1) | grain hash B
            --------------------------------------------------------------
            chb5 <= f_hash_b(cha4);
            -- per-cell centre jitter (+/-4 in normalized space): each motif
            -- sits a little off-centre, like hand-placed ornament, so no
            -- two cells' glyphs line up on one axis.
            v_dx := resize(signed('0' & lxn4), 8) - to_signed(28, 8)
                    - resize(signed('0' & cha4(2 downto 0)), 8);
            v_dy := resize(signed('0' & lyn4), 8) - to_signed(28, 8)
                    - resize(signed('0' & cha4(5 downto 3)), 8);
            adx5 <= resize(f_abs8(v_dx), 7);
            ady5 <= resize(f_abs8(v_dy), 7);
            rmood5 <= rmood4;
            lxn5 <= lxn4; lyn5 <= lyn4; seam5 <= seam4; abx5 <= abx4;

            -- gold-leaf grain: fine sparkle, plus stronger bright glints and
            -- occasional dark flecks so the leaf glints like metal.
            v_h := f_hash_b(gha4);
            grain5 <= resize(signed('0' & v_h(4 downto 0)) - 16, 7);
            if v_h(11 downto 8) = "1111" then
                grain5 <= to_signed(52, 7);          -- bright specular glint
            elsif v_h(11 downto 8) = "0000" then
                grain5 <= to_signed(-26, 7);         -- dark fleck
            end if;

            -- gold-ramp index (arithmetic here; ROM read at E6).
            v_gyidx := resize(signed('0' & iy5(9 downto 5)), 13)
                       + resize(s_pat_bias, 13);
            if v_gyidx < 0 then
                ridx5 <= to_unsigned(0, 5);
            elsif v_gyidx > 31 then
                ridx5 <= to_unsigned(31, 5);
            else
                ridx5 <= unsigned(std_logic_vector(v_gyidx(4 downto 0)));
            end if;

            -- OVERLAY: cell-hash B, jittered centre offsets, |dx|,|dy|
            ochb5 <= f_hash_b(ocha4);
            v_dx := resize(signed('0' & olxn4), 8) - to_signed(28, 8)
                    - resize(signed('0' & ocha4(2 downto 0)), 8);
            v_dy := resize(signed('0' & olyn4), 8) - to_signed(28, 8)
                    - resize(signed('0' & ocha4(5 downto 3)), 8);
            oadx5 <= resize(f_abs8(v_dx), 7);
            oady5 <= resize(f_abs8(v_dy), 7);
            olxn5 <= olxn4; olyn5 <= olyn4;
            oseam5 <= oseam4; oabx5 <= oabx4; omood5 <= omood4;

            fdist6 <= fdist5; fgate6 <= fgate5; edge6 <= edge5;

            --------------------------------------------------------------
            -- E6: squared-distance / max (geometry part 2) + ground ramp
            --------------------------------------------------------------
            -- grain-dithered ("hand-cut") geometry, folded into THIS stage
            -- so E7's compare/select chain reads registered values only
            -- (putting the dither adders inside E7 cost ~15 MHz).
            if adx5 > ady5 then
                v_gmm := signed(resize(adx5, 9));
            else
                v_gmm := signed(resize(ady5, 9));
            end if;
            v_gmm := v_gmm + resize(shift_right(grain5, 2), 9);
            if v_gmm < 0 then
                mmax6 <= (others => '0');
            else
                mmax6 <= unsigned(v_gmm(6 downto 0));
            end if;
            v_gr2 := signed(resize(f_sq(f_abs31(signed('0' & adx5))), 14))
                     + signed(resize(f_sq(f_abs31(signed('0' & ady5))), 14))
                     + shift_left(resize(grain5, 14), 1);
            if v_gr2 < 0 then
                r2_6 <= (others => '0');
            else
                r2_6 <= unsigned(v_gr2(11 downto 0));
            end if;
            chb6 <= chb5; rmood6 <= rmood5;
            lxn6 <= lxn5; lyn6 <= lyn5;
            adx6 <= adx5; ady6 <= ady5;
            seam6 <= seam5; grain6 <= grain5; abx6 <= abx5;

            -- OVERLAY geometry (pre-dithered like the base) + carries +
            -- the per-cell mask bit (origin-tile luma vs P1 threshold)
            if oadx5 > oady5 then
                v_gmm := signed(resize(oadx5, 9));
            else
                v_gmm := signed(resize(oady5, 9));
            end if;
            v_gmm := v_gmm + resize(shift_right(grain5, 2), 9);
            if v_gmm < 0 then
                omax6 <= (others => '0');
            else
                omax6 <= unsigned(v_gmm(6 downto 0));
            end if;
            v_gr2 := signed(resize(f_sq(f_abs31(signed('0' & oadx5))), 14))
                     + signed(resize(f_sq(f_abs31(signed('0' & oady5))), 14))
                     + shift_left(resize(grain5, 14), 1);
            if v_gr2 < 0 then
                or2_6 <= (others => '0');
            else
                or2_6 <= unsigned(v_gr2(11 downto 0));
            end if;
            ochb6 <= ochb5;
            olxn6 <= olxn5; olyn6 <= olyn5;
            oseam6 <= oseam5; oabx6 <= oabx5; omood6 <= omood5;
            if lram_q >= s_luma_thr then
                obit6 <= '1';
            else
                obit6 <= '0';
            end if;

            v_idx := to_integer(ridx5);
            gyr6 <= C_RAMP_Y(v_idx);
            gur6 <= C_RAMP_U(v_idx);
            gvr6 <= C_RAMP_V(v_idx);
            -- gold creeps from the shadows with GILD
            if iy6 < s_goldceil then
                gild6 <= '1';
            else
                gild6 <= '0';
            end if;

            fdist7 <= fdist6; fgate7 <= fgate6; edge7 <= edge6;

            --------------------------------------------------------------
            -- E7: motif select + evaluate -> layer select (plain-biased)
            --------------------------------------------------------------
            -- hash fields (decorrelated): motif, plain-cov, colour-cell,
            -- jewel hue, gold-shade.
            v_cov  := chb6(7 downto 0);
            v_acc  := chb6(11 downto 4);
            v_jidx := chb6(2 downto 0);
            v_cvar := chb6(6 downto 4);

            -- hand-cut geometry: r2_6 / mmax6 arrive pre-dithered from E6;
            -- add only a per-cell ring PHASE (so no two cells' rings align)
            -- and a fuzzy bar width here.
            v_rph := resize(mmax6, 7) + resize(chb6(1 downto 0) & '0', 7);
            if grain6(0) = '1' then
                v_bw := to_unsigned(11, 4);
            else
                v_bw := to_unsigned(10, 4);
            end if;

            v_plainthr := resize(s_covthr, 9);
            if rmood6 = "00" then
                v_plainthr := resize(s_covthr, 9) + 56;
            elsif rmood6 = "11" then
                v_plainthr := resize(s_covthr, 9) - 24;
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
            if rmood6 = "11" or (s_kiss = '1' and rmood6 = "10") then
                v_pillar := '1';
            end if;

            -- per-cell motif choice (varies cell to cell)
            if s_kiss = '0' then
                case chb6(10 downto 8) is
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
                case chb6(10 downto 8) is
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
                if abx6(3 downto 0) < v_bw then
                    case abx6(8 downto 6) is
                        when "100"  => v_sel := "10";              -- pale bar
                        when "101"  => v_sel := "11";              -- colour bar
                                       v_jidx := abx6(6 downto 4);
                        when others => v_sel := "01";              -- dark bar
                    end case;
                else
                    v_sel := "00";
                end if;
            elsif v_plain = '1' then
                v_sel := "00";
            elsif v_colcell = '1' then
                -- solid bright colour cell: a filled square or disc, framed
                if chb6(9) = '1' then                              -- disc
                    if r2_6 < 400 then
                        v_sel := "11";
                    elsif r2_6 < 560 then
                        v_sel := "01";
                    else
                        v_sel := "00";
                    end if;
                else                                               -- square
                    if mmax6 <= 24 then
                        v_sel := "11";
                    elsif mmax6 <= 28 then
                        v_sel := "01";
                    else
                        v_sel := "00";
                    end if;
                end if;
            else
                case v_mtype is
                    when "001" =>   -- checker: gold / ink / bone (2x2)
                        v_sub(1) := lxn6(5) xor lyn6(5);
                        v_sub(0) := lxn6(4) xor lyn6(4);
                        case v_sub is
                            when "00"   => v_sel := "00";
                            when "11"   => v_sel := "01";
                            when "10"   => v_sel := "10";
                            when others => v_sel := "00";
                        end case;
                    when "010" =>   -- short vertical bars (keyed to abs x)
                        if abx6(3 downto 0) < v_bw - 1 then
                            v_sel := "01";
                        else
                            v_sel := "00";
                        end if;
                    when "011" =>   -- concentric squares (per-cell phase)
                        v_ring := v_rph(5 downto 3);
                        case v_ring(1 downto 0) is
                            when "00"   => v_sel := "01";
                            when "01"   => v_sel := "00";
                            when "10"   => v_sel := "10";
                            when others => v_sel := "00";
                        end case;
                    when "100" =>   -- discs / rings (dark centre, pale ring)
                        if r2_6 < 64 then
                            v_sel := "01";
                        elsif r2_6 < 256 then
                            v_sel := "10";
                        elsif r2_6 < 576 then
                            v_sel := "00";
                        else
                            v_sel := "00";
                        end if;
                    when "101" =>   -- almond eye: upward triangle + oval
                        v_tri := resize(to_signed(32, 9) - signed('0' & ady6), 9);
                        if v_tri >= signed('0' & adx6) then
                            if r2_6 < 100 then
                                v_sel := "01";                     -- iris
                            else
                                v_sel := "10";                     -- pale sclera
                            end if;
                        else
                            v_sel := "00";
                        end if;
                    when "110" =>   -- squared spiral (per-cell phase)
                        v_ring := v_rph(5 downto 3);
                        if lxn6(3) = '1' and v_ring(0) = '1' then
                            v_sel := "00";
                        elsif v_ring(0) = '0' then
                            v_sel := "01";
                        else
                            v_sel := "00";
                        end if;
                    when others =>  -- dense speckle of small gold flakes
                        if chb6(9) = '1' and lxn6(2) = '1' and lyn6(2) = '1' then
                            v_sel := "10";
                        else
                            v_sel := "00";
                        end if;
                end case;
            end if;

            msel7 <= v_sel;
            jidx7 <= v_jidx;
            cvar7 <= v_cvar;
            -- apply patina warmth/cool to the raw ramp chroma (index-shift
            -- part of patina was folded into ridx5 at E5). The leaf GRAIN
            -- is folded into gy7 HERE -- a per-pixel grain adder inside
            -- E8's compose mux chain was on the critical path.
            gy7 <= f_cu10(resize(signed('0' & gyr6), 13) + resize(grain6, 13));
            gu7 <= f_cu10(resize(signed('0' & gur6), 12) + resize(s_pat_u, 12));
            gv7 <= f_cu10(resize(signed('0' & gvr6), 12) + resize(s_pat_v, 12));
            seam7 <= seam6; gild7 <= gild6;

            -- OVERLAY motif select: its own bank -- bold bars, checker,
            -- concentric squares, big rings -- busier than the base (it is
            -- a reveal) and dark-on-pale, the inverse of the base emphasis,
            -- so the revealed patch reads as a DIFFERENT gilt surface.
            v_osel := "00";
            v_rph  := resize(omax6, 7) + resize(ochb6(1 downto 0) & '0', 7);
            if omood6 = "11" or omood6 = "10" then
                -- bold tall bars keyed to absolute x (run across cells)
                if oabx6(3 downto 0) < v_bw then
                    v_osel := "01";
                end if;
            elsif ochb6(7 downto 4) < "0011" then
                v_osel := "00";                          -- breathing room
            else
                case ochb6(10 downto 8) is
                    when "000" | "011" =>                -- concentric squares
                        case v_rph(5 downto 3) is
                            when "000" | "011" | "110" => v_osel := "01";
                            when others => v_osel := "00";
                        end case;
                    when "001" =>                        -- big disc + outer ring
                        if or2_6 < 256 then
                            v_osel := "01";
                        elsif or2_6 < 484 then
                            v_osel := "00";
                        elsif or2_6 < 676 then
                            v_osel := "01";
                        else
                            v_osel := "00";
                        end if;
                    when "010" =>                        -- checker
                        if (olxn6(4) xor olyn6(4)) = '1' then
                            v_osel := "01";
                        end if;
                    when "100" =>                        -- solid jewel cell
                        if omax6 <= 26 then
                            if s_jew_on = '1' then
                                v_osel := "11";
                            else
                                v_osel := "01";
                            end if;
                        end if;
                    when "101" =>                        -- short bars
                        if oabx6(3 downto 0) < v_bw - 1 then
                            v_osel := "01";
                        end if;
                    when others =>                       -- speckle flakes
                        if ochb6(3) = '1' and olxn6(2) = '1' and olyn6(2) = '1' then
                            v_osel := "01";
                        end if;
                end case;
            end if;
            oosel7 <= v_osel;
            ocvar7 <= ochb6(6 downto 4);
            ojidx7 <= ochb6(2 downto 0);
            oseam7 <= oseam6;
            -- pre-grained overlay GROUND: pale gold, per-cell shade (C_LT)
            og_y7 <= f_cu10(resize(signed('0' & C_LT_Y(to_integer(ochb6(6 downto 4)))), 13)
                            + resize(grain6, 13));
            og_u7 <= C_LT_U(to_integer(ochb6(6 downto 4)));
            og_v7 <= C_LT_V(to_integer(ochb6(6 downto 4)));
            -- luma MASK: S8 picks the edge -- On = clean per-pixel cut,
            -- Off = whole cells (quilt-style full glyphs)
            if s_outln = '1' then
                if iy6 >= s_luma_thr then
                    omask7 <= '1';
                else
                    omask7 <= '0';
                end if;
            else
                omask7 <= obit6;
            end if;

            fdist8 <= fdist7; fgate8 <= fgate7; edge8 <= edge7;

            --------------------------------------------------------------
            -- E8: compose ground (+grain +tessera +meadow) with ornament
            --------------------------------------------------------------
            v_by := gy7; v_bu := gu7; v_bv := gv7;   -- gy7 arrives pre-grained

            if s_meadow = '1' and s_mrow = '1' then
                v_by := ('0' & v_by(9 downto 1)) + to_unsigned(120, 10);
                v_bu := v_bu - resize(v_bu(9 downto 4), 10);
                v_bv := v_bv - resize(v_bv(9 downto 4), 10);
            end if;

            if s_mosaic = '1' and seam7 = '1' then
                v_by := '0' & v_by(9 downto 1);
                v_bu := ('0' & v_bu(9 downto 1)) + to_unsigned(256, 10);
                v_bv := ('0' & v_bv(9 downto 1)) + to_unsigned(260, 10);
            end if;

            if gild7 = '0' then
                v_by := unsigned(dy_sr(6));
                v_bu := unsigned(du_sr(6));
                v_bv := unsigned(dv_sr(6));
                for k in 0 to 3 loop
                    if s_flat(k) = '1' then
                        v_bu := ('0' & v_bu(9 downto 1)) + to_unsigned(256, 10);
                        v_bv := ('0' & v_bv(9 downto 1)) + to_unsigned(256, 10);
                    end if;
                end loop;
            end if;

            if gild7 = '1' then
                case msel7 is
                    when "01" =>    -- dark ornament (per-cell warm-black shade)
                        wy8 <= C_DK_Y(to_integer(cvar7));
                        wu8 <= C_DK_U(to_integer(cvar7));
                        wv8 <= C_DK_V(to_integer(cvar7));
                    when "10" =>    -- light ornament (per-cell gold/bone shade)
                        wy8 <= C_LT_Y(to_integer(cvar7));
                        wu8 <= C_LT_U(to_integer(cvar7));
                        wv8 <= C_LT_V(to_integer(cvar7));
                    when "11" =>    -- solid jewel colour (muted a touch)
                        v_ju := C_JW_U(to_integer(jidx7));
                        v_jv := C_JW_V(to_integer(jidx7));
                        v_du2 := resize(signed('0' & v_ju), 12) - to_signed(512, 12);
                        v_dv2 := resize(signed('0' & v_jv), 12) - to_signed(512, 12);
                        -- knock the vivid down to 3/4 amplitude so the colour
                        -- sits with the gold instead of shouting over it;
                        -- P5 (s_jsat) desaturates further from there.
                        v_du2 := v_du2 - shift_right(v_du2, 2);
                        v_dv2 := v_dv2 - shift_right(v_dv2, 2);
                        v_du2 := shift_right(v_du2, s_jsat);
                        v_dv2 := shift_right(v_dv2, s_jsat);
                        wy8 <= C_JW_Y(to_integer(jidx7));
                        wu8 <= f_cu10(to_signed(512, 12) + v_du2);
                        wv8 <= f_cu10(to_signed(512, 12) + v_dv2);
                    when others =>
                        wy8 <= v_by; wu8 <= v_bu; wv8 <= v_bv;
                end case;
            else
                wy8 <= v_by; wu8 <= v_bu; wv8 <= v_bv;
            end if;

            -- LUMA MOD overlay: where the mask is on (per-pixel cut or
            -- whole cells, S8), a SECOND gilt world paints over the base --
            -- pale-gold per-cell ground with dark ornament, its own cells,
            -- seams and jewels. The base render underneath stays untouched.
            if s_luma = '1' and omask7 = '1' then
                case oosel7 is
                    when "01" =>                 -- dark ornament, per-cell shade
                        wy8 <= C_DK_Y(to_integer(ocvar7));
                        wu8 <= C_DK_U(to_integer(ocvar7));
                        wv8 <= C_DK_V(to_integer(ocvar7));
                    when "11" =>                 -- jewel cell (muted like base)
                        v_ju := C_JW_U(to_integer(ojidx7));
                        v_jv := C_JW_V(to_integer(ojidx7));
                        v_du2 := resize(signed('0' & v_ju), 12) - to_signed(512, 12);
                        v_dv2 := resize(signed('0' & v_jv), 12) - to_signed(512, 12);
                        v_du2 := v_du2 - shift_right(v_du2, 2);
                        v_dv2 := v_dv2 - shift_right(v_dv2, 2);
                        v_du2 := shift_right(v_du2, s_jsat);
                        v_dv2 := shift_right(v_dv2, s_jsat);
                        wy8 <= C_JW_Y(to_integer(ojidx7));
                        wu8 <= f_cu10(to_signed(512, 12) + v_du2);
                        wv8 <= f_cu10(to_signed(512, 12) + v_dv2);
                    when others =>               -- pale-gold overlay ground
                        if s_mosaic = '1' and oseam7 = '1' then
                            wy8 <= '0' & og_y7(9 downto 1);
                            wu8 <= ('0' & og_u7(9 downto 1)) + to_unsigned(256, 10);
                            wv8 <= ('0' & og_v7(9 downto 1)) + to_unsigned(260, 10);
                        else
                            wy8 <= og_y7; wu8 <= og_u7; wv8 <= og_v7;
                        end if;
                end case;
            end if;

            fdist9 <= fdist8; fgate9 <= fgate8;

            --------------------------------------------------------------
            -- E9: contour ink + halo
            --------------------------------------------------------------
            -- (both gated off in Luma mode: S8 is repurposed there as the
            -- overlay edge switch, and the bright flesh region is owned by
            -- the overlay layer)
            if s_luma = '0' and s_outln = '1' and s_ctr_on = '1'
               and edge8 > C_EDGETHR then
                wy9 <= '0' & C_INK_Y(9 downto 1);
                wu9 <= C_INK_U;
                wv9 <= C_INK_V;
            elsif s_luma = '0' and s_halo = '1' and s_halo_amt /= 0
                  and fgate8 = '1'
                  and fdist8 > resize(s_flesh_w, 11)
                  and fdist8 < (resize(s_flesh_w, 11) + 48) then
                v_delta := resize(signed('0' & s_halo_amt), 7) + to_signed(16, 7);
                v_hy := f_cu10(resize(signed('0' & wy8), 13)
                               + shift_left(resize(v_delta, 13), 2));
                wy9 <= v_hy;
                wu9 <= wu8 - resize(wu8(9 downto 4), 10);
                wv9 <= f_cu10(resize(signed('0' & wv8), 12)
                              + resize(signed('0' & wv8(9 downto 4)), 12));
            else
                wy9 <= wy8; wu9 <= wu8; wv9 <= wv8;
            end if;

            --------------------------------------------------------------
            -- E10: flesh restore (hard-edged warmed original)
            --------------------------------------------------------------
            v_oy := unsigned(dy_sr(8));
            v_ou := unsigned(du_sr(8));
            v_ov := unsigned(dv_sr(8));
            -- Luma mode is fully synthetic -> no flesh-restore (the raw video
            -- must not show through); otherwise restore warmed skin.
            if s_luma = '0' and fgate9 = '1' and fdist9 < resize(s_flesh_w, 11) then
                wy10 <= v_oy;
                wu10 <= v_ou - resize(v_ou(9 downto 5), 10);
                wv10 <= f_cu10(resize(signed('0' & v_ov), 12)
                               + resize(signed('0' & v_ov(9 downto 5)), 12));
            else
                wy10 <= wy9; wu10 <= wu9; wv10 <= wv9;
            end if;

            --------------------------------------------------------------
            -- line / field bookkeeping
            --------------------------------------------------------------
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_x_count <= (others => '0');
                s_lgood   <= '0';
                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 1000 then
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
    -- wet_final (wy10) is 10 regs from data_in; dry tap dy_sr(9) matches.
    ------------------------------------------------------------------------
    p_mix : process(clk)
        variable v_s : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            m1_ay <= unsigned(dy_sr(9));
            m1_au <= unsigned(du_sr(9));
            m1_av <= unsigned(dv_sr(9));
            m1_dy <= signed(resize(wy10, 11)) - signed(resize(unsigned(dy_sr(9)), 11));
            m1_du <= signed(resize(wu10, 11)) - signed(resize(unsigned(du_sr(9)), 11));
            m1_dv <= signed(resize(wv10, 11)) - signed(resize(unsigned(dv_sr(9)), 11));
            m1_t8 <= s_mix10(9 downto 2);

            m2_py <= signed('0' & m1_t8) * m1_dy;
            m2_pu <= signed('0' & m1_t8) * m1_du;
            m2_pv <= signed('0' & m1_t8) * m1_dv;
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
