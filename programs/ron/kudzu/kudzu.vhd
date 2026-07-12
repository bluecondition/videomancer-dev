-- kudzu.vhd
--
-- KUDZU -- luma as life.
--
-- Third of the "luma as X" family (mercurial = material, redshift =
-- mass-energy, kudzu = LIFE). The picture is a habitat: brightness is
-- fertile ground, and a living organism colonizes it. Unlike its siblings'
-- pure IIR fields, growth here is a CONTAGION -- vitality can only appear
-- where the ground is fertile AND life is adjacent (a living neighbor
-- cell, a per-column vertical "lighthouse" influence so vines climb, the
-- ground row, a landed spore seed, or spontaneously where the ground is
-- extremely fertile). The canopy sways in the wind (a coverage-weighted
-- warp: only the overgrowth moves, bare video is rock-still), matures into
-- blooms, and drops leaves that pile up as litter.
--
-- P12 "Season" (headline):
--   0..25%   WINTER  dormant: growth dies back, video graded cold/desat
--   ~31%     SPRING  vigorous growth, fresh palette, rising spores that
--                    SEED new colonies where they respawn (particle ->
--                    field coupling)
--   ~56%     SUMMER  full canopy, deep palette, blooms on mature growth
--   ~81%     AUTUMN  palette turns amber, growth stops, leaves fall and
--                    accumulate as litter; slider end = first frost
--
-- Controls:
--   K1 Vigor      growth rate + spread aggressiveness + climb reach
--   K2 Wind       sway amplitude/lean + particle drift (center = calm)
--   K3 Foliage    leaf lattice scale + edge raggedness
--   K4 Species    leaf hue family (green -> teal -> violet ...)
--   K5 Spores     particle density
--   K6 Fertility  luma threshold for fertile ground
--   S7 Polarity   Bright Soil / Dark Soil (fertility inverts)
--   S8 Blooms     mature-growth blossom dots on/off
--   S9 Spores     particles + litter on/off
--   S10 Evergreen no seasonal die-back (video-change die-back stays)
--   S11 Timelapse 2x growth/decay/particle dynamics
--
-- Architecture: single streaming pipeline S0..S14 (C_LATENCY = 15), no
-- frame buffer. Redshift's chassis: ONE single-bank full-color warp line
-- buffer (11 EBR), the coarse cell grid ({vitality u8, maturity u8} in one
-- 2048x16 RAM, per-column fertility IIR, one cell row updated per hblank,
-- rolling corner prefetch + dual bilinear upsample), the per-8px-column
-- particle engine, and the single-add vblank sequencer.
--
-- New machinery (this program):
--   * GROWTH UPDATER: 6-deep, 1 cell/clk. Cells read their E/W neighbours
--     through a 3-tap shift of the row read stream (Jacobi, old values) --
--     growth spreads sideways one cell at a time, so colonies have moving
--     frontiers instead of appearing everywhere at once.
--   * VERTICAL LIGHTHOUSE (vert RAM, 256x16 {val u8, row u8}): each column
--     remembers its strongest growth site; influence decays with row
--     DISTANCE from it (shift-only), so vines climb up/down from the site
--     across fields without a second row read port.
--   * PARTICLE->FIELD SEEDING: in spring, a respawning spore occasionally
--     writes a lighthouse entry {200, random row} -- spores visibly plant
--     new colonies (redshift's particles were display-only).
--   * COVERAGE-WEIGHTED SWAY: warp dx = qsin(line phase) * (coverage x
--     wind) -- the canopy waves, the uncovered picture does not.
--
-- EBR: warp 11 + grid 8 + vacc 1 + vert 1 + particles 2 = 23 of 32.
-- Multiplies (8): 6 bilinear + amp (coverage x wind) + sway (qsin x amp).
-- The winter grade is shift-only (no grade multiplies).
--
-- Timing discipline (mercurial/redshift): BRAM reads land in plain FFs
-- before use; the qsin ROM output is registered before any arithmetic;
-- every multiply is alone in its stage with registered inputs; lanes are
-- fully assembled one stage before the priority mux; the vblank sequencer
-- computes all per-frame terms in single-add steps.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture kudzu of program_top is

    constant C_LATENCY : integer := 15;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_cu8(v : unsigned) return unsigned is
    begin
        if v > 255 then
            return to_unsigned(255, 8);
        else
            return resize(v, 8);
        end if;
    end function;

    function f_cs8(v : signed) return signed is
    begin
        if v > 127 then
            return to_signed(127, 8);
        elsif v < -127 then
            return to_signed(-127, 8);
        else
            return resize(v, 8);
        end if;
    end function;

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

    -- quarter-wave folded sine, 64-entry logic ROM (fireworks/mercurial):
    -- the SDK's 1024-entry LUT infers as 6 EBRs; this is 0.
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
            v_i := not a(7 downto 2);      -- 63 - i
        end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then
            return -v_t;
        else
            return v_t;
        end if;
    end function;

    -- value-noise hash, split across two pipeline stages (inferno lesson:
    -- 1-add hashes show diagonal weave)
    function f_hash_a(cx, cy : unsigned(7 downto 0);
                      seed   : unsigned(15 downto 0)) return unsigned is
        variable h16 : unsigned(15 downto 0);
        variable h   : unsigned(11 downto 0);
    begin
        h16 := ((cx & cy)) xor seed;
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
        return h(9 downto 2);
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms (vblank sequencer)
    ----------------------------------------------------------------------
    signal s_p12      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k1       : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal s_wind     : signed(9 downto 0) := (others => '0');    -- K2 - 512
    signal s_k4       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k5       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k6       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_pol      : std_logic := '0';                         -- S7
    signal s_bloom_on : std_logic := '1';                         -- S8
    signal s_spore_on : std_logic := '1';                         -- S9
    signal s_evgr     : std_logic := '0';                         -- S10
    signal s_tl       : std_logic := '0';                         -- S11

    -- season zone terms (built one add per sequencer step)
    signal s_dspr9    : unsigned(9 downto 0) := (others => '0');
    signal s_dsum9    : unsigned(9 downto 0) := (others => '0');
    signal s_daut9    : unsigned(9 downto 0) := (others => '0');
    signal s_w9       : unsigned(9 downto 0) := (others => '0');
    signal s_late9    : unsigned(9 downto 0) := (others => '0');
    signal s_spr8     : unsigned(7 downto 0) := (others => '0');
    signal s_sum8     : unsigned(7 downto 0) := (others => '0');
    signal s_aut8     : unsigned(7 downto 0) := (others => '0');
    signal s_w8       : unsigned(7 downto 0) := (others => '0');
    signal s_late8    : unsigned(7 downto 0) := (others => '0');

    -- growth parameters
    signal s_growpre  : unsigned(8 downto 0) := (others => '0');
    signal s_grow8    : unsigned(7 downto 0) := (others => '0');
    signal s_dec8     : unsigned(7 downto 0) := (others => '0');
    signal s_die8     : unsigned(7 downto 0) := to_unsigned(10, 8);
    signal s_mat8     : unsigned(7 downto 0) := (others => '0');
    signal s_fthr8    : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_sprthr8  : unsigned(7 downto 0) := to_unsigned(80, 8);
    signal s_spon8    : unsigned(7 downto 0) := to_unsigned(192, 8);
    signal s_vish     : natural range 2 to 4 := 3;
    signal s_gsh      : natural range 2 to 5 := 4;

    -- render parameters
    signal s_d8       : unsigned(7 downto 0) := (others => '0');  -- winter
    signal s_lfy10    : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal s_bloomthr : unsigned(7 downto 0) := (others => '1');
    signal s_lthr10   : unsigned(9 downto 0) := to_unsigned(204, 10);
    signal s_lthrs10  : unsigned(9 downto 0) := to_unsigned(156, 10);
    signal s_thl      : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_thl2     : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_tha      : unsigned(9 downto 0) := to_unsigned(340, 10);
    signal s_lf_u     : unsigned(9 downto 0) := to_unsigned(430, 10);
    signal s_lf_v     : unsigned(9 downto 0) := to_unsigned(420, 10);
    signal s_lf2_u    : unsigned(9 downto 0) := to_unsigned(440, 10);
    signal s_lf2_v    : unsigned(9 downto 0) := to_unsigned(430, 10);
    signal s_ac_u     : unsigned(9 downto 0) := to_unsigned(450, 10);
    signal s_ac_v     : unsigned(9 downto 0) := to_unsigned(620, 10);
    constant C_ACY    : unsigned(9 downto 0) := to_unsigned(700, 10);

    -- particle per-frame terms
    signal s_pgate8   : unsigned(7 downto 0) := (others => '0');
    signal s_pfall    : std_logic := '0';
    signal s_pspd4    : unsigned(3 downto 0) := "0011";
    signal s_melt     : std_logic := '0';
    signal s_seed8    : unsigned(7 downto 0) := (others => '0');
    signal s_pt_y     : unsigned(9 downto 0) := to_unsigned(820, 10);
    signal s_pt_u     : unsigned(9 downto 0) := to_unsigned(490, 10);
    signal s_pt_v     : unsigned(9 downto 0) := to_unsigned(470, 10);
    signal s_wacc     : unsigned(9 downto 0) := (others => '0');
    constant C_LIT_Y  : unsigned(9 downto 0) := to_unsigned(430, 10);
    constant C_LIT_U  : unsigned(9 downto 0) := to_unsigned(480, 10);
    constant C_LIT_V  : unsigned(9 downto 0) := to_unsigned(570, 10);

    -- foliage decode (K3)
    signal s_fsh      : natural range 3 to 6 := 4;                -- lattice
    signal s_rough_sh : natural range 1 to 4 := 2;                -- jitter

    -- wind terms
    signal s_wind8    : unsigned(7 downto 0) := (others => '0');  -- |K2-512|
    signal s_swph     : unsigned(9 downto 0) := (others => '0');  -- sway roll
    signal s_wz       : unsigned(1 downto 0) := (others => '0');  -- rustle

    signal s_framectr : unsigned(2 downto 0) := (others => '0');

    signal s_seq      : unsigned(3 downto 0) := (others => '0');
    signal s_qs_a     : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar    : unsigned(9 downto 0) := (others => '0');  -- registered angle
    signal s_qs_r     : signed(9 downto 0) := (others => '0');    -- registered ROM out

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';

    ----------------------------------------------------------------------
    -- valid chain + coordinates
    ----------------------------------------------------------------------
    signal av : std_logic_vector(1 to 14) := (others => '0');

    signal s_x_count : unsigned(10 downto 0) := (others => '0');  -- input side
    signal wx        : unsigned(10 downto 0) := (others => '0');  -- write (S1)
    signal x_s2      : unsigned(10 downto 0) := (others => '0');
    signal x_s3      : unsigned(10 downto 0) := (others => '0');
    signal x_s7      : unsigned(10 downto 0) := (others => '0');
    signal rd_x      : unsigned(10 downto 0) := (others => '0');  -- read (S8)
    signal s_aline   : unsigned(9 downto 0) := (others => '0');
    signal s_seen    : std_logic := '0';
    signal s_lwidth  : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_wmax    : unsigned(10 downto 0) := to_unsigned(1919, 11);

    -- per-line terms (hblank lstep sequencer: rustle jitter)
    signal s_lstep    : unsigned(1 downto 0) := (others => '0');
    signal s_lh_a     : unsigned(11 downto 0) := (others => '0');
    signal s_ljit10   : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S0..S7: input reg, bilinear, sway amp/phase, dx
    ----------------------------------------------------------------------
    signal r0_y, r0_u, r0_v : unsigned(9 downto 0) := (others => '0');

    signal amp5   : unsigned(7 downto 0) := (others => '0');      -- coverage*wind
    signal ph3    : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- GROWTH GRID
    --
    -- Redshift/mercurial coarse-grid machinery: cells 2^xsh px x 2^ysh
    -- field lines from the MEASURED line width (1080i: 32x16 -> 60 cols,
    -- stride 60; 720p: 40 cols; SD: 16x8 -> 45 cols, stride 48; max
    -- address 2039 < 2048). Per-column vacc accumulates the area-averaged
    -- luma (fertility; polarity applied here). The GROWTH updater walks
    -- one cell row per hblank -- see p_upd. Display side: rolling corner
    -- prefetch + hblank prefetch of cols 0..1, then dual bilinear
    -- ({vitality, maturity} both unsigned).
    ----------------------------------------------------------------------
    signal s_xsh    : natural range 4 to 5 := 5;
    signal s_ysh    : natural range 3 to 5 := 4;
    signal s_cols   : unsigned(5 downto 0) := to_unsigned(60, 6);
    signal s_stride : unsigned(5 downto 0) := to_unsigned(60, 6);
    -- per-frame address constants (v0.2 timing: the inline s_cols-1 /
    -- v_col+2>=s_cols arithmetic in the prefetch address cone was the
    -- routed critical path -- everything is a registered constant now)
    signal s_cm1_11 : unsigned(10 downto 0) := to_unsigned(59, 11);
    signal s_cm2_8  : unsigned(7 downto 0) := to_unsigned(58, 8);
    signal s_st_cm1 : unsigned(10 downto 0) := to_unsigned(119, 11);
    signal s_st_p1  : unsigned(10 downto 0) := to_unsigned(61, 11);
    signal s_st_p2  : unsigned(10 downto 0) := to_unsigned(62, 11);
    -- per-line prefetch address registers
    signal s_rb_last, s_rb1_last : unsigned(10 downto 0) := (others => '0');
    signal s_rb_p1, s_rb1_p1     : unsigned(10 downto 0) := (others => '0');
    signal pf_a, pf_a1           : unsigned(10 downto 0) := (others => '0');

    -- display-side cell tracking
    signal fx5, fy5 : unsigned(4 downto 0) := (others => '0');
    signal dsp_rb   : unsigned(10 downto 0) := (others => '0');
    signal dsp_rb1  : unsigned(10 downto 0) := (others => '0');
    signal pf_seq   : unsigned(3 downto 0) := (others => '0');
    signal grid_raddr : unsigned(10 downto 0) := (others => '0');

    -- corner registers {v u8, a u8} + span-stable diffs
    signal c00, c01, c10, c11 : std_logic_vector(15 downto 0) := (others => '0');
    signal p_top, p_bot       : std_logic_vector(15 downto 0) := (others => '0');
    signal d0v, d1v, d0a, d1a : signed(8 downto 0) := (others => '0');

    -- dual bilinear pipeline (S1..S4)
    signal prtv, prbv, prta, prba : signed(14 downto 0) := (others => '0');
    signal c00v_1, c01v_1 : unsigned(7 downto 0) := (others => '0');
    signal c00a_1, c01a_1 : unsigned(7 downto 0) := (others => '0');
    signal rt2v : signed(8 downto 0) := (others => '0');
    signal dv2v : signed(9 downto 0) := (others => '0');
    signal rt2a : signed(8 downto 0) := (others => '0');
    signal dv2a : signed(9 downto 0) := (others => '0');
    signal pv3v, pv3a : signed(15 downto 0) := (others => '0');
    signal rt3v, rt3a : signed(8 downto 0) := (others => '0');
    signal v_bil4  : unsigned(7 downto 0) := (others => '0');
    signal a_bil4  : unsigned(7 downto 0) := (others => '0');

    -- per-column fertility accumulator
    signal acc_y       : unsigned(12 downto 0) := (others => '0');
    signal acc_pre_col : unsigned(7 downto 0) := (others => '0');
    signal vq_disp     : unsigned(11 downto 0) := (others => '0');
    signal vacc_we     : std_logic := '0';
    signal vacc_waddr  : unsigned(7 downto 0) := (others => '0');
    signal vacc_wdata  : unsigned(15 downto 0) := (others => '0');

    -- growth updater (one cell row per hblank, 1 cell/clk, 8-deep --
    -- the one-stage grow/decay decision was the v0.1 routed critical
    -- path at 46 MHz; each arithmetic step now has its own stage)
    signal u_go, u_wait, u_active : std_logic := '0';
    signal u_rb     : unsigned(10 downto 0) := (others => '0');
    signal u_rbl    : unsigned(10 downto 0) := (others => '0');
    signal u_rowi   : unsigned(7 downto 0) := (others => '0');
    signal u_ground : std_logic := '0';
    signal u_cx     : unsigned(5 downto 0) := (others => '0');
    signal u_v      : std_logic_vector(1 to 7) := (others => '0');
    signal u_a0     : unsigned(10 downto 0) := (others => '0');
    signal u_vca    : unsigned(7 downto 0) := (others => '0');
    signal u_c0, u_c1, u_c2 : unsigned(5 downto 0) := (others => '0');
    -- U1 land
    signal u_sv2    : unsigned(7 downto 0) := (others => '0');
    signal u_sa2    : unsigned(7 downto 0) := (others => '0');
    signal u_t2     : unsigned(7 downto 0) := (others => '0');
    signal u_vv2    : unsigned(7 downto 0) := (others => '0');
    signal u_vr2    : unsigned(7 downto 0) := (others => '0');
    -- U2 shift (3-tap neighbourhood) + row abs-distance
    signal n_c, n_w : unsigned(7 downto 0) := (others => '0');
    signal a_c      : unsigned(7 downto 0) := (others => '0');
    signal t_c      : unsigned(7 downto 0) := (others => '0');
    signal vv_c     : unsigned(7 downto 0) := (others => '0');
    signal u_dd3    : unsigned(7 downto 0) := (others => '0');
    signal u_dz3    : std_logic := '0';
    signal u_cc     : unsigned(5 downto 0) := (others => '0');
    -- U3 intermediates
    signal u_e4     : unsigned(7 downto 0) := (others => '0');
    signal u_nb4    : unsigned(7 downto 0) := (others => '0');
    signal u_if4    : unsigned(7 downto 0) := (others => '0');
    signal u_dz4    : std_logic := '0';
    signal u_vc4    : unsigned(7 downto 0) := (others => '0');
    signal u_ac4    : unsigned(7 downto 0) := (others => '0');
    signal u_cc4    : unsigned(5 downto 0) := (others => '0');
    -- U4 booleans + amounts
    signal u_can5   : std_logic := '0';
    signal u_g5     : unsigned(7 downto 0) := (others => '0');
    signal u_dec5   : unsigned(8 downto 0) := (others => '0');
    signal u_vc5    : unsigned(7 downto 0) := (others => '0');
    signal u_ac5    : unsigned(7 downto 0) := (others => '0');
    signal u_cc5    : unsigned(5 downto 0) := (others => '0');
    signal u_if5    : unsigned(7 downto 0) := (others => '0');
    signal u_dz5    : std_logic := '0';
    -- U5 new vitality
    signal u_vn6    : unsigned(7 downto 0) := (others => '0');
    signal u_ac6    : unsigned(7 downto 0) := (others => '0');
    signal u_cc6    : unsigned(5 downto 0) := (others => '0');
    signal u_if6    : unsigned(7 downto 0) := (others => '0');
    signal u_dz6    : std_logic := '0';
    -- U6 maturity -> write regs
    signal u_wv5    : unsigned(7 downto 0) := (others => '0');
    signal u_wa5    : unsigned(7 downto 0) := (others => '0');
    signal u_waddr5 : unsigned(10 downto 0) := (others => '0');
    signal u_we5    : std_logic := '0';
    signal vt_we5   : std_logic := '0';
    signal vt_wa5   : unsigned(7 downto 0) := (others => '0');
    signal vt_wd5   : std_logic_vector(15 downto 0) := (others => '0');
    signal s_pend_row : std_logic := '0';
    signal s_cellrow  : unsigned(7 downto 0) := (others => '0');
    signal s_ground_rb : unsigned(10 downto 0) := (others => '1');

    -- memories
    type t_grid is array (0 to 2047) of std_logic_vector(15 downto 0);
    type t_v256 is array (0 to 255)  of std_logic_vector(15 downto 0);
    signal grid_ram : t_grid := (others => (others => '0'));
    signal vacc     : t_v256 := (others => (others => '0'));
    signal vert_ram : t_v256 := (others => (others => '0'));
    signal grid_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal vacc_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal vert_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal grid_addr_eff : unsigned(10 downto 0) := (others => '0');
    signal vacc_raddr    : unsigned(7 downto 0) := (others => '0');
    signal vert_waddr    : unsigned(7 downto 0) := (others => '0');
    signal vert_wdata    : std_logic_vector(15 downto 0) := (others => '0');
    signal vert_we       : std_logic := '0';

    ----------------------------------------------------------------------
    -- PARTICLES (spores / falling leaves) -- redshift engine
    ----------------------------------------------------------------------
    type t_p256 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal part_ram : t_p256 := (others => (others => '0'));
    signal surf_ram : t_p256 := (others => "1111111111" & "000000");
    signal part_q, surf_q : std_logic_vector(15 downto 0) := (others => '0');
    signal part_raddr, surf_raddr : unsigned(7 downto 0) := (others => '0');
    signal prf_col : unsigned(7 downto 0) := (others => '0');

    -- render-side column registers (current + prefetched next)
    signal pc_pos : unsigned(11 downto 0) := (others => '0');
    signal pn_pos : unsigned(11 downto 0) := (others => '0');
    signal sc_bs, sn_bs : unsigned(9 downto 0) := (others => '1');
    signal sc_dp, sn_dp : unsigned(5 downto 0) := (others => '0');
    signal pc_h8, pn_h8 : unsigned(7 downto 0) := (others => '0');
    -- off-screen particles (parked at the 1023 sentinel in dark columns)
    -- must not draw: the 2-row hit window wraps onto lines 0..1
    signal pc_vis, pn_vis : std_logic := '0';
    signal p_ha : unsigned(11 downto 0) := (others => '0');
    signal r0_ph3 : unsigned(2 downto 0) := (others => '0');

    -- hit-flag pipes (S1 -> S13)
    signal fl_sr  : std_logic_vector(2 to 13) := (others => '0');
    signal cap_sr : std_logic_vector(2 to 13) := (others => '0');

    -- raster bright-surface detect write
    signal sw_we   : std_logic := '0';
    signal sw_addr : unsigned(7 downto 0) := (others => '0');
    signal sw_data : std_logic_vector(15 downto 0) := (others => '0');

    -- vblank particle updater FSM (redshift: LAND -> CALC -> DECIDE)
    type t_pu is (PU_IDLE, PU_REQ, PU_W1, PU_W2, PU_LAND, PU_CALC,
                  PU_CALC2, PU_DECIDE);
    signal pu_state : t_pu := PU_IDLE;
    signal pu_arm   : std_logic := '0';
    signal pu_cnt   : unsigned(7 downto 0) := (others => '0');
    signal pl_pos   : unsigned(11 downto 0) := (others => '0');
    signal pl_spd   : unsigned(3 downto 0) := (others => '0');
    signal pl_bs    : unsigned(9 downto 0) := (others => '0');
    signal pl_dp    : unsigned(5 downto 0) := (others => '0');
    signal pl_np    : unsigned(11 downto 0) := (others => '0');
    signal pl_top   : unsigned(9 downto 0) := (others => '0');
    signal pl_rs    : unsigned(11 downto 0) := (others => '0');
    signal pl_rspd  : unsigned(3 downto 0) := (others => '0');
    -- PU_CALC2 registered range flags (the fused compare chain into the
    -- seed-write enable was the v0.2 routed critical path)
    signal pl_rsp    : std_logic := '0';
    signal pl_land   : std_logic := '0';
    signal pl_fell   : std_logic := '0';
    signal pl_seedok : std_logic := '0';
    signal pu_busy  : std_logic := '0';
    signal pu_we, pu_swe : std_logic := '0';
    signal pu_waddr : unsigned(7 downto 0) := (others => '0');
    signal pu_wdata, pu_swdata : std_logic_vector(15 downto 0) := (others => '0');
    signal pu_vwe   : std_logic := '0';                    -- lighthouse seed
    signal pu_vaddr : unsigned(7 downto 0) := (others => '0');
    signal pu_vdata : std_logic_vector(15 downto 0) := (others => '0');
    signal pu_prev_vsync_n : std_logic := '1';
    signal s_nrows  : unsigned(9 downto 0) := to_unsigned(540, 10);
    signal s_lfsr   : unsigned(15 downto 0) := x"ACE1";

    signal surf_waddr : unsigned(7 downto 0) := (others => '0');
    signal surf_wdata : std_logic_vector(15 downto 0) := (others => '0');
    signal surf_we    : std_logic := '0';

    signal hp6    : signed(18 downto 0) := (others => '0');       -- sway mult
    signal dx7    : signed(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S8..S10: warp addresses, BRAM reads, land
    ----------------------------------------------------------------------
    signal q_addr  : unsigned(10 downto 0) := (others => '0');
    signal u_addr  : unsigned(9 downto 0) := (others => '0');
    signal v_addr  : unsigned(9 downto 0) := (others => '0');

    type t_lb_y  is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_uv is array (0 to 1023) of std_logic_vector(9 downto 0);
    signal lbY : t_lb_y  := (others => (others => '0'));
    signal lbU : t_lb_uv := (others => (others => '0'));
    signal lbV : t_lb_uv := (others => (others => '0'));
    signal by_q, bu_q, bv_q : std_logic_vector(9 downto 0) := (others => '0');

    signal wy10, wu10, wv10 : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- leaf lattice hash tracks (S7..S10) + coverage delay track
    ----------------------------------------------------------------------
    signal fr1_8, fr2_8 : unsigned(7 downto 0) := (others => '0');
    signal fh8, fh8b : unsigned(11 downto 0) := (others => '0');
    signal jb9, jb9b : unsigned(7 downto 0) := (others => '0');
    signal jit10   : unsigned(8 downto 0) := (others => '0');
    signal jit2_10 : unsigned(7 downto 0) := (others => '0');
    signal jit2_11, jit2_12 : unsigned(7 downto 0) := (others => '0');

    -- bloom dot sub-cell bits (middle-half band of both diagonal coords)
    signal dot7, dot8, dot9, dot10 : std_logic := '0';

    -- v/a bilinear delayed to the gate stage
    signal m_d5, m_d6, m_d7, m_d8, m_d9, m_d10 : unsigned(7 downto 0) := (others => '0');
    signal a_d5, a_d6, a_d7, a_d8, a_d9, a_d10 : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S11..S14: grade + gates, re-register, lanes, priority mux
    ----------------------------------------------------------------------
    signal y11, u11, v11 : unsigned(9 downto 0) := (others => '0');
    signal y12, u12, v12 : unsigned(9 downto 0) := (others => '0');
    signal lf_h11, lf_s11, bl_11, mat_11 : std_logic := '0';
    signal lf_h12, lf_s12, bl_12, mat_12 : std_logic := '0';

    -- S13 lanes (fully assembled one stage before the mux)
    signal sel13 : unsigned(2 downto 0) := (others => '0');
    signal ln_vy, ln_vu, ln_vv : unsigned(9 downto 0) := (others => '0');
    signal ln_ly, ln_lu, ln_lv : unsigned(9 downto 0) := (others => '0');
    signal ln_hy, ln_hu, ln_hv : unsigned(9 downto 0) := (others => '0');

    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared quarter-wave sine: the vblank sequencer (palette hue taps)
    -- borrows the per-pixel sway tap's angle port. Both the angle and the
    -- ROM output are registered (mercurial: that path was the routed HD
    -- critical path twice). Consumers see qsin two clocks late; the
    -- sequencer captures two-behind.
    ------------------------------------------------------------------------
    s_qs_a <= s_thl  + to_unsigned(256, 10) when s_seq = 11 else
              s_thl                         when s_seq = 10 else
              s_thl2 + to_unsigned(256, 10) when s_seq = 9  else
              s_thl2                        when s_seq = 8  else
              s_tha  + to_unsigned(256, 10) when s_seq = 7  else
              s_tha                         when s_seq = 6  else
              ph3;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (one add per step)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_d   : unsigned(9 downto 0);
        variable v_gp  : unsigned(8 downto 0);
        variable v_g8  : unsigned(7 downto 0);
        variable v_pz  : unsigned(7 downto 0);
        variable v_pw  : unsigned(7 downto 0);
        variable v_dc  : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_p12  <= unsigned(registers_in(7));
                s_k1   <= unsigned(registers_in(0));
                s_wind <= signed(unsigned(registers_in(1)) - 512);
                s_k4   <= unsigned(registers_in(3));
                s_k5   <= unsigned(registers_in(4));
                s_k6   <= unsigned(registers_in(5));
                s_pol      <= registers_in(6)(0);   -- switch 7
                s_bloom_on <= registers_in(6)(1);   -- switch 8
                s_spore_on <= registers_in(6)(2);   -- switch 9
                s_evgr     <= registers_in(6)(3);   -- switch 10
                s_tl       <= registers_in(6)(4);   -- switch 11

                -- foliage decode: lattice cell 8..64 px diagonal, jitter
                -- roughness inverse
                s_fsh      <= 3 + to_integer(unsigned(registers_in(2)(9 downto 8)));
                s_rough_sh <= 4 - to_integer(unsigned(registers_in(2)(9 downto 8)));

                -- grid geometry from the measured line width (mercurial),
                -- with the derived address constants latched alongside
                if s_lwidth >= 1600 then
                    s_xsh <= 5; s_ysh <= 4;
                    s_cols <= to_unsigned(60, 6); s_stride <= to_unsigned(60, 6);
                    s_cm1_11 <= to_unsigned(59, 11);
                    s_cm2_8  <= to_unsigned(58, 8);
                    s_st_cm1 <= to_unsigned(119, 11);
                    s_st_p1  <= to_unsigned(61, 11);
                    s_st_p2  <= to_unsigned(62, 11);
                elsif s_lwidth >= 1000 then
                    s_xsh <= 5; s_ysh <= 5;
                    s_cols <= to_unsigned(40, 6); s_stride <= to_unsigned(60, 6);
                    s_cm1_11 <= to_unsigned(39, 11);
                    s_cm2_8  <= to_unsigned(38, 8);
                    s_st_cm1 <= to_unsigned(99, 11);
                    s_st_p1  <= to_unsigned(61, 11);
                    s_st_p2  <= to_unsigned(62, 11);
                else
                    s_xsh <= 4; s_ysh <= 3;
                    s_cols <= to_unsigned(45, 6); s_stride <= to_unsigned(48, 6);
                    s_cm1_11 <= to_unsigned(44, 11);
                    s_cm2_8  <= to_unsigned(43, 8);
                    s_st_cm1 <= to_unsigned(92, 11);
                    s_st_p1  <= to_unsigned(49, 11);
                    s_st_p2  <= to_unsigned(50, 11);
                end if;

                s_framectr <= s_framectr + 1;

                -- wind terms: |wind| amplitude, lean, sway phase roll
                if s_wind < 0 then
                    s_wind8 <= unsigned(resize(shift_right(-s_wind, 1), 8));
                else
                    s_wind8 <= unsigned(resize(shift_right(s_wind, 1), 8));
                end if;

                s_seq <= "1110";        -- 14-step capture chain
            elsif s_seq /= 0 and data_in.avid = '0' then
                case to_integer(s_seq) is
                    when 14 =>
                        -- season zone diffs (parallel abs-subs)
                        if s_p12 >= 320 then
                            s_dspr9 <= s_p12 - 320;
                        else
                            s_dspr9 <= 320 - s_p12;
                        end if;
                        if s_p12 >= 576 then
                            s_dsum9 <= s_p12 - 576;
                        else
                            s_dsum9 <= 576 - s_p12;
                        end if;
                        if s_p12 >= 832 then
                            s_daut9 <= s_p12 - 832;
                        else
                            s_daut9 <= 832 - s_p12;
                        end if;
                        if s_p12 < 256 then
                            s_w9 <= to_unsigned(256, 10) - s_p12;
                        else
                            s_w9 <= (others => '0');
                        end if;
                        if s_p12 > 896 then
                            s_late9 <= s_p12 - 896;
                        else
                            s_late9 <= (others => '0');
                        end if;
                        -- sway phase advance (always breathing; timelapse 2x)
                        if s_tl = '1' then
                            s_swph <= s_swph + 12 + resize(s_wind8(7 downto 4), 10);
                        else
                            s_swph <= s_swph + 6 + resize(s_wind8(7 downto 5), 10);
                        end if;
                        s_wacc <= s_wacc + unsigned(resize(s_wind(9 downto 5), 10));
                        s_wz   <= s_wind8(7 downto 6);
                    when 13 =>
                        -- season window strengths
                        if s_dspr9 >= 192 then
                            s_spr8 <= (others => '0');
                        else
                            s_spr8 <= resize(to_unsigned(192, 10) - s_dspr9, 8);
                        end if;
                        if s_dsum9 >= 224 then
                            s_sum8 <= (others => '0');
                        else
                            s_sum8 <= resize(to_unsigned(224, 10) - s_dsum9, 8);
                        end if;
                        if s_daut9 >= 224 then
                            s_aut8 <= (others => '0');
                        else
                            s_aut8 <= resize(to_unsigned(224, 10) - s_daut9, 8);
                        end if;
                        if s_w9 > 255 then
                            s_w8 <= (others => '1');
                        else
                            s_w8 <= resize(s_w9, 8);
                        end if;
                        s_late8 <= resize(shift_right(s_late9, 2), 8);
                    when 12 =>
                        -- leaf hue (K4 species, autumn pulls toward amber)
                        s_thl <= to_unsigned(384, 10)
                                 + resize(s_k4(9 downto 1), 10)
                                 - resize(s_aut8, 10);
                        s_growpre <= resize(s_spr8, 9)
                                     + resize(shift_right(s_sum8, 1), 9);
                        -- seasonal decay: winter + late autumn + slow autumn
                        if s_evgr = '1' then
                            v_dc := (others => '0');
                        else
                            v_dc := resize(shift_right(s_w8, 4), 9)
                                    + resize(shift_right(s_late8, 2), 9)
                                    + resize(shift_right(s_aut8, 6), 9);
                        end if;
                        if s_tl = '1' then
                            v_dc := shift_left(v_dc, 1);
                        end if;
                        if v_dc > 255 then
                            s_dec8 <= (others => '1');
                        else
                            s_dec8 <= resize(v_dc, 8);
                        end if;
                        s_mat8 <= resize(shift_right(s_sum8, 4), 8)
                                  + resize(shift_right(s_spr8, 6), 8);
                        s_d8   <= s_w8;
                        if s_p12 > 640 then
                            s_pfall <= '1';
                        else
                            s_pfall <= '0';
                        end if;
                        s_gsh  <= 5 - to_integer(s_k1(9 downto 8));
                        case s_k1(9 downto 8) is
                            when "00"   => s_vish <= 4;
                            when "01"   => s_vish <= 3;
                            when others => s_vish <= 2;
                        end case;
                    when 11 =>
                        -- (theta_l presented this step for lf_u)
                        s_thl2 <= s_thl + 40;
                        s_tha  <= s_thl - 300;
                        v_gp := shift_right(s_growpre, s_gsh);
                        if v_gp = 0 and s_growpre /= 0 then
                            v_g8 := to_unsigned(1, 8);
                        elsif v_gp > 255 then
                            v_g8 := (others => '1');
                        else
                            v_g8 := resize(v_gp, 8);
                        end if;
                        if s_tl = '1' and v_g8 < 128 then
                            s_grow8 <= shift_left(v_g8, 1);
                        else
                            s_grow8 <= v_g8;
                        end if;
                        if s_tl = '1' then
                            s_die8 <= to_unsigned(20, 8);
                        else
                            s_die8 <= to_unsigned(10, 8);
                        end if;
                        s_fthr8   <= resize(s_k6(9 downto 2), 8);
                        s_sprthr8 <= to_unsigned(96, 8)
                                     - resize(s_k1(9 downto 4), 8);
                        s_spon8   <= to_unsigned(224, 8)
                                     - resize(s_k1(9 downto 3), 8);
                    when 10 =>
                        -- particle window: spores (spring) / leaves (autumn)
                        v_pw := s_spr8;
                        if s_aut8 > v_pw then
                            v_pw := s_aut8;
                        end if;
                        if shift_right(s_sum8, 2) > v_pw then
                            v_pw := shift_right(s_sum8, 2);
                        end if;
                        if v_pw(7) = '1' then
                            v_pz := (others => '1');
                        else
                            v_pz := v_pw(6 downto 0) & '0';
                        end if;
                        if s_k5(9 downto 2) < v_pz then
                            s_pgate8 <= s_k5(9 downto 2);
                        else
                            s_pgate8 <= v_pz;
                        end if;
                        if s_pfall = '1' then
                            s_pspd4 <= "0010";
                        else
                            s_pspd4 <= "0011";
                        end if;
                        if s_tl = '1' then
                            s_pspd4 <= "0101";
                        end if;
                        s_seed8 <= resize(shift_right(s_spr8, 2), 8);
                        if s_aut8 = 0 and s_framectr(1 downto 0) = "00" then
                            s_melt <= '1';
                        else
                            s_melt <= '0';
                        end if;
                        s_lfy10 <= to_unsigned(260, 10)
                                   + resize(shift_right(s_spr8, 2), 10)
                                   + resize(shift_right(s_aut8, 3), 10);
                        s_bloomthr <= to_unsigned(255, 8)
                                      - resize(shift_right(s_sum8, 1), 8);
                        -- leaf threshold compensates the K3 jitter mean so
                        -- the canopy onset coverage stays put across
                        -- roughness settings
                        s_lthr10 <= to_unsigned(164, 10)
                                    + shift_right(to_unsigned(160, 10),
                                                  s_rough_sh);
                    when 3 =>
                        s_lthrs10 <= s_lthr10 - 48;
                    when 9 =>
                        -- capture two-behind: qsin(theta_l + 256)
                        s_lf_u <= f_cu10(to_signed(512, 11)
                                         + resize(shift_right(s_qs_r, 2), 11));
                        -- particle palette by season
                        if s_pfall = '1' then
                            s_pt_y <= to_unsigned(620, 10);
                            s_pt_u <= to_unsigned(470, 10);
                            s_pt_v <= to_unsigned(590, 10);
                        else
                            s_pt_y <= to_unsigned(820, 10);
                            s_pt_u <= to_unsigned(490, 10);
                            s_pt_v <= to_unsigned(470, 10);
                        end if;
                    when 8 =>
                        s_lf_v <= f_cu10(to_signed(512, 11)
                                         + resize(shift_right(s_qs_r, 2), 11));
                    when 7 =>
                        s_lf2_u <= f_cu10(to_signed(512, 11)
                                          + resize(shift_right(s_qs_r, 2), 11));
                    when 6 =>
                        s_lf2_v <= f_cu10(to_signed(512, 11)
                                          + resize(shift_right(s_qs_r, 2), 11));
                    when 5 =>
                        s_ac_u <= f_cu10(to_signed(512, 11)
                                         + resize(shift_right(s_qs_r, 2), 11));
                    when 4 =>
                        s_ac_v <= f_cu10(to_signed(512, 11)
                                         + resize(shift_right(s_qs_r, 2), 11));
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- main pixel pipeline (S0..S14)
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_q   : signed(12 downto 0);
        variable v_qa  : unsigned(10 downto 0);
        variable v_y   : signed(11 downto 0);
        variable v_u   : signed(11 downto 0);
        variable v_v   : signed(11 downto 0);
        variable v_pos : unsigned(4 downto 0);
        variable v_end : boolean;
        variable v_col : unsigned(7 downto 0);
        variable v_l8  : unsigned(7 downto 0);
        variable v_sum : unsigned(12 downto 0);
        variable v_avg : unsigned(7 downto 0);
        variable v_rtv : signed(8 downto 0);
        variable v_rbv : signed(8 downto 0);
        variable v_rta : signed(8 downto 0);
        variable v_rba : signed(8 downto 0);
        variable v_vv  : signed(9 downto 0);
        variable v_av  : signed(9 downto 0);
        variable v_rowend : boolean;
        variable v_nal : unsigned(9 downto 0);
        variable v_pd  : unsigned(9 downto 0);
        variable v_fx3 : unsigned(2 downto 0);
        variable v_sd  : unsigned(10 downto 0);
        variable v_ly  : signed(11 downto 0);
        variable v_cj  : unsigned(9 downto 0);
        variable v_d1  : std_logic;
        variable v_d2  : std_logic;
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            av(1) <= data_in.avid;
            for i in 2 to 14 loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            -- S0: input register + grid display tracking
            sw_we <= '0';
            if data_in.avid = '1' then
                r0_y <= unsigned(data_in.y);
                r0_u <= unsigned(data_in.u);
                r0_v <= unsigned(data_in.v);
                r0_ph3 <= s_x_count(2 downto 0);
                s_x_count <= s_x_count + 1;

                -- particle column prefetch (8-px spans): shift at phase 0,
                -- fetch column c+1 mid-span, hash in two stages
                case s_x_count(2 downto 0) is
                    when "000" =>
                        pc_pos <= pn_pos;
                        sc_bs  <= sn_bs;
                        sc_dp  <= sn_dp;
                        pc_h8  <= pn_h8;
                        pc_vis <= pn_vis;
                    when "001" =>
                        prf_col <= s_x_count(10 downto 3) + 1;
                    when "010" =>
                        p_ha <= f_hash_a(prf_col, x"6B", x"9D2C");
                    when "011" =>
                        pn_pos <= unsigned(part_q(15 downto 4));
                        sn_bs  <= unsigned(surf_q(15 downto 6));
                        sn_dp  <= unsigned(surf_q(5 downto 0));
                    when "100" =>
                        pn_h8 <= f_hash_b(p_ha);
                        if pn_pos(11 downto 2) < s_nrows then
                            pn_vis <= '1';
                        else
                            pn_vis <= '0';
                        end if;
                    when "101" =>
                        -- raster bright-surface detect on this column's
                        -- lane pixel (r0_y is x%8 = 4): min-write
                        if r0_y(9 downto 2) > 168
                           and s_aline < sc_bs then
                            sw_we   <= '1';
                            sw_addr <= s_x_count(10 downto 3);
                            sw_data <= std_logic_vector(s_aline)
                                       & std_logic_vector(sc_dp);
                        end if;
                    when others =>
                        null;
                end case;

                -- span position within the current cell
                if s_xsh = 4 then
                    v_pos := '0' & s_x_count(3 downto 0);
                else
                    v_pos := s_x_count(4 downto 0);
                end if;
                v_end := (s_xsh = 5 and v_pos = 31)
                         or (s_xsh = 4 and v_pos = 15);
                v_col := resize(shift_right(s_x_count, s_xsh), 8);

                -- bilinear x-fraction for THIS pixel
                if s_xsh = 4 then
                    fx5 <= s_x_count(3 downto 0) & '0';
                else
                    fx5 <= s_x_count(4 downto 0);
                end if;

                -- per-column fertility accumulator + IIR (polarity applied
                -- here: Dark Soil = darkness is fertile)
                if s_pol = '1' then
                    v_l8 := not r0_y(9 downto 2);
                else
                    v_l8 := r0_y(9 downto 2);
                end if;
                if v_pos = 0 then
                    v_sum := resize(v_l8, 13);
                else
                    v_sum := acc_y + resize(v_l8, 13);
                end if;
                acc_y <= v_sum;
                if v_pos = 12 then
                    acc_pre_col <= v_col;      -- vacc read for the IIR
                end if;
                vacc_we <= '0';
                if v_end then
                    v_avg := resize(shift_right(v_sum, s_xsh), 8);
                    vacc_waddr <= v_col;
                    if fy5 = 0 then            -- first line of the cell row
                        vacc_wdata <= resize(v_avg & "0000", 16);
                    else
                        vacc_wdata <= resize(
                            unsigned(signed(resize(vq_disp, 14))
                                     + shift_right(signed(resize(v_avg & "0000", 14))
                                                   - signed(resize(vq_disp, 14)), 2)),
                            16);
                    end if;
                    vacc_we <= '1';
                end if;
                if v_pos = 14 then
                    vq_disp <= unsigned(vacc_q(11 downto 0));
                end if;

                -- rolling corner prefetch: fetch column col+2's two rows
                -- mid-span, shift corners at span start -- but NOT at
                -- column 0, whose corners the hblank prefetch just loaded
                if v_pos = 0 and v_col /= 0 then
                    c00 <= c10;  c01 <= c11;
                    c10 <= p_top;  c11 <= p_bot;
                    d0v <= signed(resize(unsigned(p_top(15 downto 8)), 9))
                           - signed(resize(unsigned(c10(15 downto 8)), 9));
                    d1v <= signed(resize(unsigned(p_bot(15 downto 8)), 9))
                           - signed(resize(unsigned(c11(15 downto 8)), 9));
                    d0a <= signed(resize(unsigned(p_top(7 downto 0)), 9))
                           - signed(resize(unsigned(c10(7 downto 0)), 9));
                    d1a <= signed(resize(unsigned(p_bot(7 downto 0)), 9))
                           - signed(resize(unsigned(c11(7 downto 0)), 9));
                    -- running prefetch addresses (dsp_rb* + col + 2)
                    pf_a  <= pf_a + 1;
                    pf_a1 <= pf_a1 + 1;
                elsif v_pos = 4 and v_col /= 0 then
                    -- span 0's rolling prefetch is skipped: the hblank
                    -- prefetch already loaded col 2's corners, and the
                    -- growth updater may still own the grid read port
                    -- this early in the line
                    if v_col >= s_cm2_8 then
                        grid_raddr <= s_rb_last;
                    else
                        grid_raddr <= pf_a;
                    end if;
                elsif v_pos = 6 and v_col /= 0 then
                    p_top <= grid_q;
                elsif v_pos = 8 and v_col /= 0 then
                    if v_col >= s_cm2_8 then
                        grid_raddr <= s_rb1_last;
                    else
                        grid_raddr <= pf_a1;
                    end if;
                elsif v_pos = 10 and v_col /= 0 then
                    p_bot <= grid_q;
                end if;
            end if;

            -- hblank prefetch of the coming line's first two columns, the
            -- particle column 0, AND column 2's rolling corners (p_top /
            -- p_bot) -- so span 0 needs no rolling prefetch and the first
            -- display read of the line lands well clear of the growth
            -- updater's walk (see p_upd)
            if pf_seq /= 0 then
                case pf_seq is
                    when "0001" => grid_raddr <= resize(dsp_rb, 11);
                                   prf_col <= (others => '0');
                    when "0010" => grid_raddr <= resize(dsp_rb1, 11);
                                   p_ha <= f_hash_a(x"00", x"6B", x"9D2C");
                    when "0011" => c00 <= grid_q;
                                   grid_raddr <= s_rb_p1;
                                   pn_pos <= unsigned(part_q(15 downto 4));
                                   sn_bs  <= unsigned(surf_q(15 downto 6));
                                   sn_dp  <= unsigned(surf_q(5 downto 0));
                    when "0100" => c01 <= grid_q;
                                   grid_raddr <= s_rb1_p1;
                                   pn_h8 <= f_hash_b(p_ha);
                                   if pn_pos(11 downto 2) < s_nrows then
                                       pn_vis <= '1';
                                   else
                                       pn_vis <= '0';
                                   end if;
                    when "0101" => c10 <= grid_q;
                                   grid_raddr <= pf_a;
                    when "0110" => c11 <= grid_q;
                                   grid_raddr <= pf_a1;
                    when "0111" =>
                        p_top <= grid_q;
                        d0v <= signed(resize(unsigned(c10(15 downto 8)), 9))
                               - signed(resize(unsigned(c00(15 downto 8)), 9));
                        d1v <= signed(resize(unsigned(c11(15 downto 8)), 9))
                               - signed(resize(unsigned(c01(15 downto 8)), 9));
                        d0a <= signed(resize(unsigned(c10(7 downto 0)), 9))
                               - signed(resize(unsigned(c00(7 downto 0)), 9));
                        d1a <= signed(resize(unsigned(c11(7 downto 0)), 9))
                               - signed(resize(unsigned(c01(7 downto 0)), 9));
                    when others =>
                        p_bot <= grid_q;
                end case;
                if pf_seq = "1000" then
                    pf_seq <= (others => '0');
                else
                    pf_seq <= pf_seq + 1;
                end if;
            end if;

            -- S1: the four bilinear horizontal mults (d0/d1 are span-
            -- stable; the base corners are pipelined alongside the
            -- products); warp buffer write rides this stage (see p_lb*)
            if av(1) = '1' then
                prtv <= d0v * signed('0' & fx5);
                prbv <= d1v * signed('0' & fx5);
                prta <= d0a * signed('0' & fx5);
                prba <= d1a * signed('0' & fx5);
                c00v_1 <= unsigned(c00(15 downto 8));
                c01v_1 <= unsigned(c01(15 downto 8));
                c00a_1 <= unsigned(c00(7 downto 0));
                c01a_1 <= unsigned(c01(7 downto 0));
                wx   <= wx + 1;

                -- particle hit tests (1-bit flags piped to the S13 lane
                -- select). Flake x wanders as it moves (pos into the
                -- jitter) and leans with the wind accumulator.
                v_pd  := s_aline - pc_pos(11 downto 2);
                v_fx3 := pc_h8(4 downto 2) + s_wacc(9 downto 7)
                         + pc_pos(4 downto 2);
                if s_spore_on = '1' and pc_vis = '1'
                   and pc_h8 < s_pgate8
                   and v_pd(9 downto 1) = 0
                   and (r0_ph3 = v_fx3 or r0_ph3 = v_fx3 + 1) then
                    fl_sr(2) <= '1';
                else
                    fl_sr(2) <= '0';
                end if;
                -- leaf-litter band [bsurf - depth, bsurf)
                if sc_dp /= 0
                   and s_aline < sc_bs
                   and s_aline + resize(sc_dp, 10) >= sc_bs then
                    cap_sr(2) <= '1';
                else
                    cap_sr(2) <= '0';
                end if;
            end if;
            fl_sr(3 to 13)  <= fl_sr(2 to 12);
            cap_sr(3 to 13) <= cap_sr(2 to 12);

            -- S2: bilinear horizontal lerp sums + vertical diffs
            if av(2) = '1' then
                v_rtv := signed(resize(c00v_1, 9)) + resize(shift_right(prtv, 5), 9);
                v_rbv := signed(resize(c01v_1, 9)) + resize(shift_right(prbv, 5), 9);
                rt2v  <= v_rtv;
                dv2v  <= resize(v_rbv, 10) - resize(v_rtv, 10);
                v_rta := signed(resize(c00a_1, 9)) + resize(shift_right(prta, 5), 9);
                v_rba := signed(resize(c01a_1, 9)) + resize(shift_right(prba, 5), 9);
                rt2a  <= v_rta;
                dv2a  <= resize(v_rba, 10) - resize(v_rta, 10);
                x_s2 <= x_s2 + 1;
            end if;

            -- S3: bilinear vertical mults, alone in their stage; sway
            -- phase assembled for this pixel (consumed via the shared
            -- qsin port: angle registers at S4, ROM output at S5)
            if av(3) = '1' then
                pv3v <= dv2v * signed('0' & fy5);
                pv3a <= dv2a * signed('0' & fy5);
                rt3v <= rt2v;
                rt3a <= rt2a;
                ph3  <= shift_left(resize(s_aline, 10), 2)
                        + s_swph + s_ljit10
                        + resize(x_s3(10 downto 6), 10);
                x_s3 <= x_s3 + 1;
            end if;

            -- S4: assemble the coverage/maturity bilinears (+ warm-up
            -- guard for the first lines/columns before the prefetch has
            -- primed)
            if av(4) = '1' then
                v_vv := resize(rt3v, 10) + resize(shift_right(pv3v, 5), 10);
                v_av := resize(rt3a, 10) + resize(shift_right(pv3a, 5), 10);
                if s_aline < 4 or x_s2 < 8 then
                    v_bil4 <= (others => '0');
                    a_bil4 <= (others => '0');
                else
                    if v_vv < 0 then
                        v_bil4 <= (others => '0');
                    elsif v_vv > 255 then
                        v_bil4 <= (others => '1');
                    else
                        v_bil4 <= resize(unsigned(v_vv), 8);
                    end if;
                    if v_av < 0 then
                        a_bil4 <= (others => '0');
                    elsif v_av > 255 then
                        a_bil4 <= (others => '1');
                    else
                        a_bil4 <= resize(unsigned(v_av), 8);
                    end if;
                end if;
            end if;

            -- S5: sway amplitude mult, alone in its stage: coverage x wind
            -- (this is what makes ONLY the overgrowth move)
            if av(5) = '1' then
                amp5 <= resize(shift_right(v_bil4 * s_wind8, 8), 8);
                m_d5 <= v_bil4;
                a_d5 <= a_bil4;
            end if;

            -- S6: sway mult, alone in its stage: qsin(phase) x amplitude
            if av(6) = '1' then
                hp6 <= s_qs_r * signed('0' & amp5);
                m_d6 <= m_d5;
                a_d6 <= a_d5;
            end if;

            -- S7: total displacement clamped s8 (amp is already coverage-
            -- weighted, so bare video never moves); leaf lattice coords +
            -- bloom-dot sub-cell band bits (middle half of both diagonal
            -- coords -> a diamond dot, size tracks the K3 lattice)
            if av(7) = '1' then
                dx7 <= f_cs8(resize(shift_right(hp6, 12), 13));
                v_sd := x_s7 + resize(s_aline, 11);
                fr1_8 <= resize(shift_right(v_sd, s_fsh), 8);
                case s_fsh is
                    when 3      => v_d1 := v_sd(2) xor v_sd(1);
                    when 4      => v_d1 := v_sd(3) xor v_sd(2);
                    when 5      => v_d1 := v_sd(4) xor v_sd(3);
                    when others => v_d1 := v_sd(5) xor v_sd(4);
                end case;
                v_sd := x_s7 - resize(s_aline, 11);
                fr2_8 <= resize(shift_right(v_sd, s_fsh), 8);
                case s_fsh is
                    when 3      => v_d2 := v_sd(2) xor v_sd(1);
                    when 4      => v_d2 := v_sd(3) xor v_sd(2);
                    when 5      => v_d2 := v_sd(4) xor v_sd(3);
                    when others => v_d2 := v_sd(5) xor v_sd(4);
                end case;
                dot7 <= v_d1 and v_d2;
                x_s7 <= x_s7 + 1;
                m_d7 <= m_d6;
                a_d7 <= a_d6;
            end if;

            -- S8: warp read addresses (clamped to the measured line
            -- width); leaf hash stage A (two independent lattice hashes:
            -- edge jitter + vein/bloom)
            if av(8) = '1' then
                v_q := signed(resize(rd_x, 13)) + resize(dx7, 13);
                if v_q < 0 then
                    v_qa := (others => '0');
                elsif v_q > signed(resize(s_wmax, 13)) then
                    v_qa := s_wmax;
                else
                    v_qa := unsigned(v_q(10 downto 0));
                end if;
                q_addr <= v_qa;
                u_addr <= v_qa(10 downto 1);
                v_addr <= v_qa(10 downto 1);
                rd_x <= rd_x + 1;

                fh8  <= f_hash_a(fr1_8, fr2_8, x"51F0");
                fh8b <= f_hash_a(fr2_8, fr1_8, x"C3A5");
                dot8 <= dot7;
                m_d8 <= m_d7;
                a_d8 <= a_d7;
            end if;

            -- S9: BRAM reads happen this cycle (see p_lb*); hash stage B
            if av(9) = '1' then
                jb9  <= f_hash_b(fh8);
                jb9b <= f_hash_b(fh8b);
                dot9 <= dot8;
                m_d9 <= m_d8;
                a_d9 <= a_d8;
            end if;

            -- S10: land the warp reads in plain FFs; scale the edge jitter
            if av(10) = '1' then
                wy10 <= unsigned(by_q);
                wu10 <= unsigned(bu_q);
                wv10 <= unsigned(bv_q);
                jit10 <= resize(shift_right(jb9, s_rough_sh), 9)
                         + resize(shift_right(jb9, s_rough_sh + 2), 9);
                jit2_10 <= jb9b;
                dot10 <= dot9;
                m_d10 <= m_d9;
                a_d10 <= a_d9;
            end if;

            -- S11: winter grade (shift-only: darken + desat toward gray +
            -- blue bias) and the leaf gates on the delayed coverage
            if av(11) = '1' then
                v_y := signed(resize(wy10, 12));
                v_u := signed(resize(wu10, 12));
                v_v := signed(resize(wv10, 12));
                case s_d8(7 downto 6) is
                    when "01" =>
                        v_y := v_y - signed(resize(wy10(9 downto 5), 12));
                        v_u := v_u + shift_right(to_signed(512, 12) - v_u, 3);
                        v_v := v_v + shift_right(to_signed(512, 12) - v_v, 3);
                    when "10" =>
                        v_y := v_y - signed(resize(wy10(9 downto 4), 12));
                        v_u := v_u + shift_right(to_signed(512, 12) - v_u, 2);
                        v_v := v_v + shift_right(to_signed(512, 12) - v_v, 2);
                    when "11" =>
                        v_y := v_y - signed(resize(wy10(9 downto 4), 12))
                                   - signed(resize(wy10(9 downto 5), 12));
                        v_u := v_u + shift_right(to_signed(512, 12) - v_u, 1);
                        v_v := v_v + shift_right(to_signed(512, 12) - v_v, 1);
                    when others =>
                        null;
                end case;
                if s_d8(7 downto 6) /= "00" then
                    v_u := v_u + signed(resize(s_d8(7 downto 4), 12));
                    v_v := v_v - signed(resize(s_d8(7 downto 4), 12));
                end if;
                y11 <= f_cu10(v_y);
                u11 <= f_cu10(v_u);
                v11 <= f_cu10(v_v);

                -- leaf gates: coverage + lattice jitter vs threshold
                v_cj := resize(m_d10, 10) + resize(jit10, 10);
                if v_cj > s_lthr10 then
                    lf_h11 <= '1';
                else
                    lf_h11 <= '0';
                end if;
                if v_cj > s_lthrs10 then
                    lf_s11 <= '1';
                else
                    lf_s11 <= '0';
                end if;
                -- bloom: mature growth + sparse lattice cells + sub-cell
                -- dot band
                if s_bloom_on = '1' and a_d10 > s_bloomthr
                   and jit2_10 > 232 and dot10 = '1' then
                    bl_11 <= '1';
                else
                    bl_11 <= '0';
                end if;
                if a_d10 > 128 then
                    mat_11 <= '1';
                else
                    mat_11 <= '0';
                end if;
                jit2_11 <= jit2_10;
            end if;

            -- S12: re-register (gates land at the S13 lane assembly in
            -- step with the graded video -- redshift lesson)
            if av(12) = '1' then
                y12 <= y11;
                u12 <= u11;
                v12 <= v11;
                lf_h12 <= lf_h11;
                lf_s12 <= lf_s11;
                bl_12  <= bl_11;
                mat_12 <= mat_11;
                jit2_12 <= jit2_11;
            end if;

            -- S13: lanes fully assembled one stage before the mux
            if av(13) = '1' then
                -- graded video lane
                ln_vy <= y12;
                ln_vu <= u12;
                ln_vv <= v12;
                -- leaf lane: season base + video luma showing through +
                -- vein darkening from the second lattice hash
                v_ly := signed(resize(s_lfy10, 12))
                        + signed(resize(y12(9 downto 2), 12))
                        - signed(resize(jit2_12(7 downto 3), 12));
                if mat_12 = '1' then
                    v_ly := v_ly - 24;      -- old growth reads darker
                end if;
                ln_ly <= f_cu10(v_ly);
                if mat_12 = '1' then
                    ln_lu <= s_lf2_u;
                    ln_lv <= s_lf2_v;
                else
                    ln_lu <= s_lf_u;
                    ln_lv <= s_lf_v;
                end if;
                -- half-blend soft-edge lane
                ln_hy <= ('0' & y12(9 downto 1)) + ('0' & s_lfy10(9 downto 1))
                         + resize(y12(9 downto 3), 10);
                ln_hu <= ('0' & u12(9 downto 1)) + ('0' & s_lf_u(9 downto 1));
                ln_hv <= ('0' & v12(9 downto 1)) + ('0' & s_lf_v(9 downto 1));

                -- priority select: particle > litter > bloom > leaf >
                -- soft edge > video
                if fl_sr(13) = '1' then
                    sel13 <= "101";
                elsif cap_sr(13) = '1' and s_spore_on = '1' then
                    sel13 <= "100";
                elsif bl_12 = '1' and lf_h12 = '1' then
                    sel13 <= "011";
                elsif lf_h12 = '1' then
                    sel13 <= "010";
                elsif lf_s12 = '1' then
                    sel13 <= "001";
                else
                    sel13 <= "000";
                end if;
            end if;

            -- S14: priority mux -> output registers
            if av(14) = '1' then
                case sel13 is
                    when "101" =>                       -- spore / leaf
                        s_out_y <= s_pt_y;
                        s_out_u <= s_pt_u;
                        s_out_v <= s_pt_v;
                    when "100" =>                       -- leaf litter
                        s_out_y <= C_LIT_Y;
                        s_out_u <= C_LIT_U;
                        s_out_v <= C_LIT_V;
                    when "011" =>                       -- bloom
                        s_out_y <= C_ACY;
                        s_out_u <= s_ac_u;
                        s_out_v <= s_ac_v;
                    when "010" =>                       -- leaf canopy
                        s_out_y <= ln_ly;
                        s_out_u <= ln_lu;
                        s_out_v <= ln_lv;
                    when "001" =>                       -- canopy soft edge
                        s_out_y <= ln_hy;
                        s_out_u <= ln_hu;
                        s_out_v <= ln_hv;
                    when others =>                      -- graded video
                        s_out_y <= ln_vy;
                        s_out_u <= ln_vu;
                        s_out_v <= ln_vv;
                end case;
            end if;

            -- line bookkeeping: width measure, counter resets, aline,
            -- per-line rustle jitter (2-step lstep sequencer)
            if s_lstep = 1 then
                s_lh_a  <= f_hash_a(resize(s_aline, 8),
                                    resize(s_swph(9 downto 2), 8), x"B33F");
                s_lstep <= "10";
            elsif s_lstep = 2 then
                -- wind zone scales the per-line rustle (calm = smooth)
                if s_wz = 0 then
                    s_ljit10 <= (others => '0');
                else
                    s_ljit10 <= shift_left(
                        resize(f_hash_b(s_lh_a), 10),
                        to_integer(s_wz) - 1);
                end if;
                s_lstep <= "00";
            end if;

            u_go <= '0';
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_x_count > 0 then
                    s_lwidth <= s_x_count;
                    s_wmax   <= s_x_count - 1;
                end if;
                s_x_count <= (others => '0');
                wx    <= (others => '0');
                x_s2  <= (others => '0');
                x_s3  <= (others => '0');
                x_s7  <= (others => '0');
                rd_x  <= (others => '0');
                s_lstep <= "01";

                v_rowend := (s_ysh = 3 and s_aline(2 downto 0) = "111")
                         or (s_ysh = 4 and s_aline(3 downto 0) = "1111")
                         or (s_ysh = 5 and s_aline(4 downto 0) = "11111");

                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 1000 then
                        s_aline <= s_aline + 1;
                    end if;
                    v_nal := s_aline + 1;
                    if v_rowend then
                        -- this cell row is complete: update it, then
                        -- advance the display row bases (all the derived
                        -- prefetch addresses are single adds of registered
                        -- values: new dsp_rb = old dsp_rb1)
                        u_go       <= '1';
                        u_rb       <= dsp_rb;
                        u_rowi     <= s_cellrow;
                        s_cellrow  <= s_cellrow + 1;
                        s_pend_row <= '0';
                        dsp_rb     <= dsp_rb + resize(s_stride, 11);
                        s_rb_last  <= dsp_rb1 + s_cm1_11;
                        s_rb_p1    <= dsp_rb1 + 1;
                        pf_a       <= dsp_rb1 + 2;
                        if dsp_rb1 = s_ground_rb then
                            -- new top row IS the ground row: replicate it
                            -- vertically (advancing dsp_rb1 past the grid
                            -- would wrap the 11-bit address into row 0 --
                            -- phantom canopy on the bottom row at 1080i)
                            s_rb1_last <= dsp_rb1 + s_cm1_11;
                            s_rb1_p1   <= dsp_rb1 + 1;
                            pf_a1      <= dsp_rb1 + 2;
                        else
                            dsp_rb1    <= dsp_rb1 + resize(s_stride, 11);
                            s_rb1_last <= dsp_rb1 + s_st_cm1;
                            s_rb1_p1   <= dsp_rb1 + s_st_p1;
                            pf_a1      <= dsp_rb1 + s_st_p2;
                        end if;
                    else
                        s_pend_row <= '1';
                        pf_a  <= dsp_rb + 2;
                        pf_a1 <= dsp_rb1 + 2;
                    end if;
                else
                    -- vblank: fire the final partial cell row once, then
                    -- reset the field-vertical state
                    if s_pend_row = '1' then
                        u_go       <= '1';
                        u_rb       <= dsp_rb;
                        u_rowi     <= s_cellrow;
                        s_pend_row <= '0';
                        s_ground_rb <= dsp_rb;
                    else
                        if dsp_rb >= resize(s_stride, 11) then
                            s_ground_rb <= dsp_rb - resize(s_stride, 11);
                        end if;
                    end if;
                    if s_aline /= 0 then
                        s_nrows <= s_aline;     -- measured field height
                    end if;
                    s_aline <= (others => '0');
                    v_nal   := (others => '0');
                    s_cellrow <= (others => '0');
                    dsp_rb  <= (others => '0');
                    dsp_rb1 <= resize(s_stride, 11);
                    s_rb_last  <= s_cm1_11;
                    s_rb1_last <= s_st_cm1;
                    s_rb_p1    <= to_unsigned(1, 11);
                    s_rb1_p1   <= s_st_p1;
                    pf_a       <= to_unsigned(2, 11);
                    pf_a1      <= s_st_p2;
                end if;

                case s_ysh is
                    when 3      => fy5 <= v_nal(2 downto 0) & "00";
                    when 4      => fy5 <= v_nal(3 downto 0) & '0';
                    when others => fy5 <= v_nal(4 downto 0);
                end case;

                pf_seq <= "0001";  -- prefetch the coming line's cols 0..2
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- growth updater: one cell row per hblank, 1 cell/clk, 8-deep. Starts
    -- 8 clocks after u_go so the hblank corner prefetch owns the grid read
    -- port first (worst 8 + 61 + 9 clocks vs the 138-clock minimum
    -- hblank). All shifts, zero multiplies. These paths are STA-timed at
    -- the full pixel clock (ziffern/mercurial lesson) -- the v0.1 single-
    -- stage decision routed at 46 MHz, so every arithmetic step gets its
    -- own stage:
    --
    --   U0 issue     grid/vacc/vert read addresses
    --   U1 land      BRAM reads into plain FFs
    --   U2 shift     3-tap E/C/W neighbourhood + lighthouse row distance
    --   U3 inter     fertility excess, neighbour max, lighthouse influence
    --   U4 decide    adjacency booleans + grow/decay amounts
    --   U5 apply     new vitality (one add chain, clamped)
    --   U6 write     maturity + packed {v,a} + lighthouse capture
    --
    -- Growth rule per cell (Jacobi, previous-field neighbour values):
    --   fertile (luma excess e > 8) AND adjacent to life (alive already,
    --   E/W neighbour above spread threshold, column lighthouse influence,
    --   ground row, or e above the spontaneous threshold) -> v += grow;
    --   seasonal decay + infertile die-back always subtract. Maturity
    --   accumulates while v is high (drives the bloom/two-tone palette).
    ------------------------------------------------------------------------
    p_upd : process(clk)
        variable v_e   : signed(8 downto 0);
        variable v_nb  : unsigned(7 downto 0);
        variable v_dd  : unsigned(7 downto 0);
        variable v_if  : signed(10 downto 0);
        variable v_src : unsigned(7 downto 0);
        variable v_dec : unsigned(8 downto 0);
        variable v_vn  : signed(9 downto 0);
        variable v_an  : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- delayed issue loop
            if u_go = '1' then
                u_wait <= '1';
                u_ground <= '0';
                if u_rb = s_ground_rb then
                    u_ground <= '1';
                end if;
                u_rbl <= u_rb + s_cm1_11;      -- clamped (last-col) address
            elsif u_wait = '1' and pf_seq = 0 then
                u_wait   <= '0';
                u_active <= '1';
                u_cx     <= (others => '0');
            elsif u_active = '1' then
                if u_cx = s_cols then
                    u_active <= '0';
                end if;
                u_cx <= u_cx + 1;
            end if;
            if u_active = '1' then
                if u_cx >= s_cols then
                    u_a0  <= u_rbl;
                    u_vca <= resize(s_cm1_11, 8);
                else
                    u_a0  <= resize(u_rb, 11) + resize(u_cx, 11);
                    u_vca <= resize(u_cx, 8);
                end if;
                u_c0 <= u_cx;
            end if;
            u_v(1) <= u_active;
            for i in 2 to 7 loop
                u_v(i) <= u_v(i - 1);
            end loop;
            u_c1 <= u_c0;

            -- U1: land the BRAM reads in plain FFs
            if u_v(2) = '1' then
                u_sv2 <= unsigned(grid_q(15 downto 8));
                u_sa2 <= unsigned(grid_q(7 downto 0));
                u_t2  <= unsigned(vacc_q(11 downto 4));
                u_vv2 <= unsigned(vert_q(15 downto 8));
                u_vr2 <= unsigned(vert_q(7 downto 0));
                u_c2  <= u_c1;
            end if;

            -- U2: 3-tap neighbourhood shift (E = u_sv2 one cell ahead,
            -- C = n_c, W = n_w) + per-cell context regs + row distance
            -- to the column lighthouse (hoisted out of U3)
            if u_v(3) = '1' then
                n_w  <= n_c;
                n_c  <= u_sv2;
                a_c  <= u_sa2;
                t_c  <= u_t2;
                vv_c <= u_vv2;
                if u_rowi >= u_vr2 then
                    v_dd := u_rowi - u_vr2;
                else
                    v_dd := u_vr2 - u_rowi;
                end if;
                u_dd3 <= v_dd;
                if v_dd = 0 then
                    u_dz3 <= '1';
                else
                    u_dz3 <= '0';
                end if;
                u_cc <= u_c2;
            end if;

            -- U3: intermediates for the compute cell (all against the OLD
            -- regs: E arrives in u_sv2 this clock)
            if u_v(4) = '1' then
                -- fertility excess
                v_e := signed(resize(t_c, 9)) - signed(resize(s_fthr8, 9));
                if v_e < 0 then
                    u_e4 <= (others => '0');
                else
                    u_e4 <= resize(unsigned(v_e), 8);
                end if;
                -- E/W neighbour max (edge guard: col 0 has no west; the
                -- east read is address-clamped so the last col replicates)
                if u_cc = 0 then
                    v_nb := u_sv2;
                else
                    v_nb := n_w;
                    if u_sv2 > v_nb then
                        v_nb := u_sv2;
                    end if;
                end if;
                u_nb4 <= v_nb;
                -- column lighthouse influence: value decays with row
                -- distance (shift-only)
                v_if := signed(resize(vv_c, 11))
                        - signed(resize(shift_left(resize(u_dd3, 10), s_vish), 11));
                if v_if < 0 then
                    u_if4 <= (others => '0');
                else
                    u_if4 <= resize(unsigned(v_if), 8);
                end if;
                u_dz4 <= u_dz3;
                u_vc4 <= n_c;
                u_ac4 <= a_c;
                u_cc4 <= u_cc;
            end if;

            -- U4: adjacency/fertility booleans + amounts, nothing chained
            -- past one compare
            if u_v(5) = '1' then
                v_src := u_nb4;
                if u_if4 > v_src then
                    v_src := u_if4;
                end if;
                if u_ground = '1' then
                    v_src := (others => '1');
                end if;
                if (u_e4 > 8)
                   and (u_vc4 >= 24 or v_src >= s_sprthr8
                        or u_e4 >= s_spon8) then
                    u_can5 <= '1';
                else
                    u_can5 <= '0';
                end if;
                if u_e4(7 downto 1) > resize(s_grow8, 7) then
                    u_g5 <= s_grow8;
                else
                    u_g5 <= '0' & u_e4(7 downto 1);
                end if;
                v_dec := resize(s_dec8, 9);
                if u_e4 <= 8 then
                    v_dec := v_dec + resize(s_die8, 9);
                end if;
                u_dec5 <= v_dec;
                u_vc5 <= u_vc4;
                u_ac5 <= u_ac4;
                u_cc5 <= u_cc4;
                u_if5 <= u_if4;
                u_dz5 <= u_dz4;
            end if;

            -- U5: new vitality = vc + (can ? grow : 0) - dec, clamped
            if u_v(6) = '1' then
                v_vn := signed(resize(u_vc5, 10));
                if u_can5 = '1' then
                    v_vn := v_vn + signed(resize(u_g5, 10));
                end if;
                v_vn := v_vn - signed(resize(u_dec5, 10));
                if v_vn < 0 then
                    v_vn := (others => '0');
                elsif v_vn > 255 then
                    v_vn := to_signed(255, 10);
                end if;
                u_vn6 <= resize(unsigned(v_vn), 8);
                u_ac6 <= u_ac5;
                u_cc6 <= u_cc5;
                u_if6 <= u_if5;
                u_dz6 <= u_dz5;
            end if;

            -- U6: maturity + write registers + lighthouse capture
            u_we5  <= '0';
            vt_we5 <= '0';
            if u_v(7) = '1' then
                -- maturity accumulates under a dense canopy
                if u_vn6 >= 176 then
                    v_an := resize(u_ac6, 9) + resize(s_mat8, 9);
                    if v_an > 255 then
                        v_an := to_unsigned(255, 9);
                    end if;
                elsif u_vn6 < 96 then
                    if u_ac6 >= 2 then
                        v_an := resize(u_ac6 - 2, 9);
                    else
                        v_an := (others => '0');
                    end if;
                else
                    v_an := resize(u_ac6, 9);
                end if;

                u_wv5    <= u_vn6;
                u_wa5    <= resize(v_an, 8);
                u_waddr5 <= resize(u_rb, 11) + resize(u_cc6, 11);
                if u_cc6 < s_cols then
                    u_we5 <= '1';
                end if;

                -- lighthouse capture: the strongest growth site in the
                -- column wins; the stored row refreshes itself
                if u_cc6 < s_cols
                   and (u_dz6 = '1' or u_vn6 > u_if6) then
                    vt_we5 <= '1';
                    vt_wa5 <= resize(u_cc6, 8);
                    vt_wd5 <= std_logic_vector(u_vn6)
                              & std_logic_vector(u_rowi);
                end if;
            end if;
        end if;
    end process p_upd;

    ------------------------------------------------------------------------
    -- grid memories. The grid read port is time-muxed: the updater owns it
    -- during its u_v(1) window (blanking only, after the prefetch), the
    -- display prefetch otherwise -- port-exclusive by construction, no
    -- arbiter (mercurial vacc pattern). One write port each. The vert
    -- (lighthouse) write port is shared with the vblank spore seeder
    -- (time-exclusive: the seeder only runs after the row updater idles).
    ------------------------------------------------------------------------
    grid_addr_eff <= u_a0  when u_v(1) = '1' else grid_raddr;
    vacc_raddr    <= u_vca when u_v(1) = '1' else acc_pre_col;

    vert_waddr <= vt_wa5 when vt_we5 = '1' else pu_vaddr;
    vert_wdata <= vt_wd5 when vt_we5 = '1' else pu_vdata;
    vert_we    <= vt_we5 or pu_vwe;

    p_grid : process(clk)
    begin
        if rising_edge(clk) then
            grid_q <= grid_ram(to_integer(grid_addr_eff));
            if u_we5 = '1' then
                grid_ram(to_integer(u_waddr5)) <=
                    std_logic_vector(u_wv5) & std_logic_vector(u_wa5);
            end if;
        end if;
    end process p_grid;

    p_vacc : process(clk)
    begin
        if rising_edge(clk) then
            vacc_q <= vacc(to_integer(vacc_raddr));
            if vacc_we = '1' then
                vacc(to_integer(vacc_waddr)) <= std_logic_vector(vacc_wdata);
            end if;
        end if;
    end process p_vacc;

    p_vert : process(clk)
    begin
        if rising_edge(clk) then
            vert_q <= vert_ram(to_integer(u_vca));
            if vert_we = '1' then
                vert_ram(to_integer(vert_waddr)) <= vert_wdata;
            end if;
        end if;
    end process p_vert;

    ------------------------------------------------------------------------
    -- particle updater: matrix_rain's column walk, once per field during
    -- vblank (armed at vsync, started once the row updater is idle so the
    -- lighthouse write port is free). Spring: spores rise from the bright
    -- surface; a respawning spore occasionally SEEDS the growth field by
    -- writing a lighthouse entry (particle -> field coupling). Autumn:
    -- leaves fall and accumulate litter depth on the bright surface
    -- (decays when the season turns).
    ------------------------------------------------------------------------
    p_part : process(clk)
        variable v_spd  : unsigned(3 downto 0);
        variable v_bs   : unsigned(9 downto 0);
        variable v_dp   : unsigned(5 downto 0);
        variable v_np   : unsigned(11 downto 0);
        variable v_fb   : std_logic;
        variable v_rsp  : boolean;
    begin
        if rising_edge(clk) then
            -- free-running 16-bit Fibonacci LFSR (respawn randomness)
            v_fb := s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10);
            s_lfsr <= s_lfsr(14 downto 0) & v_fb;

            pu_prev_vsync_n <= data_in.vsync_n;
            pu_we  <= '0';
            pu_swe <= '0';
            pu_vwe <= '0';
            if data_in.vsync_n = '0' and pu_prev_vsync_n = '1' then
                pu_arm <= '1';
            end if;
            case pu_state is
                when PU_IDLE =>
                    if pu_arm = '1' and u_active = '0' and u_wait = '0'
                       and u_go = '0' then
                        pu_arm   <= '0';
                        pu_cnt   <= (others => '0');
                        pu_busy  <= '1';
                        pu_state <= PU_REQ;
                    end if;
                when PU_REQ =>
                    pu_state <= PU_W1;      -- read address presented (mux)
                when PU_W1 =>
                    pu_state <= PU_W2;      -- BRAM read edge
                when PU_W2 =>
                    pu_state <= PU_LAND;    -- data valid
                when PU_LAND =>
                    -- land the BRAM reads in plain FFs before ANY use
                    pl_pos <= unsigned(part_q(15 downto 4));
                    pl_spd <= unsigned(part_q(3 downto 0));
                    pl_bs  <= unsigned(surf_q(15 downto 6));
                    pl_dp  <= unsigned(surf_q(5 downto 0));
                    pu_state <= PU_CALC;
                when PU_CALC =>
                    -- advance, litter-band top, respawn snapshots
                    if s_pfall = '1' then
                        pl_np <= pl_pos + resize(s_pspd4, 12)
                                 + resize(pl_spd(1 downto 0), 12);
                    else
                        pl_np <= pl_pos - resize(s_pspd4, 12)
                                 - resize(pl_spd(1 downto 0), 12);
                    end if;
                    if pl_bs > pl_dp then
                        pl_top <= pl_bs - pl_dp;
                    else
                        pl_top <= (others => '0');
                    end if;
                    pl_rs   <= (to_unsigned(960, 10)
                                + resize(unsigned(s_lfsr(5 downto 0)), 10))
                               & "00";
                    pl_rspd <= unsigned(s_lfsr(3 downto 0));
                    pu_state <= PU_CALC2;
                when PU_CALC2 =>
                    -- register every range compare as a 1-bit flag so
                    -- PU_DECIDE is pure muxing (the fused chain was the
                    -- v0.2 critical path)
                    if pl_np(11 downto 2) < 4
                       or pl_np(11 downto 2) > 1000 then
                        pl_rsp <= '1';
                    else
                        pl_rsp <= '0';
                    end if;
                    if pl_np(11 downto 2) >= pl_top
                       and pl_np(11 downto 2) <= pl_bs + 4
                       and pl_bs < s_nrows then
                        pl_land <= '1';
                    else
                        pl_land <= '0';
                    end if;
                    if pl_np(11 downto 2) > s_nrows + 16
                       and pl_np(11 downto 2) < 900 then
                        pl_fell <= '1';
                    else
                        pl_fell <= '0';
                    end if;
                    if s_lfsr(15 downto 8) < s_seed8 then
                        pl_seedok <= '1';
                    else
                        pl_seedok <= '0';
                    end if;
                    pu_state <= PU_DECIDE;
                when PU_DECIDE =>
                    v_np  := pl_np;
                    v_spd := pl_spd;
                    v_bs  := pl_bs;
                    v_dp  := pl_dp;
                    v_rsp := false;
                    if s_pfall = '0' then
                        -- spore: exits top (or wrapped) -> respawn at the
                        -- bright surface
                        if pl_rsp = '1' then
                            v_np  := pl_bs & "00";
                            v_spd := pl_rspd;
                            v_rsp := true;
                        end if;
                    else
                        -- leaf: lands on the litter band, accumulates;
                        -- fell past everything -> respawn above the frame
                        if pl_land = '1' then
                            if v_dp < 48 then
                                v_dp := v_dp + 1;
                            end if;
                            v_np  := pl_rs;
                            v_spd := pl_rspd;
                        elsif pl_fell = '1' then
                            v_np  := pl_rs;
                            v_spd := pl_rspd;
                        end if;
                    end if;

                    -- litter decay + surface drift-down (re-adapt)
                    if s_melt = '1' and v_dp /= 0 then
                        v_dp := v_dp - 1;
                    end if;
                    if v_bs /= "1111111111" then
                        v_bs := v_bs + 1;
                    end if;

                    -- spring seeding: a respawning spore occasionally
                    -- plants a colony -- one lighthouse write {200, random
                    -- row} in this particle's cell column
                    if v_rsp and s_spore_on = '1'
                       and pl_seedok = '1' then
                        pu_vwe <= '1';
                        if s_xsh = 5 then
                            pu_vaddr <= "00" & pu_cnt(7 downto 2);
                        else
                            pu_vaddr <= '0' & pu_cnt(7 downto 1);
                        end if;
                        pu_vdata <= std_logic_vector(to_unsigned(200, 8))
                                    & "000" & std_logic_vector(s_lfsr(4 downto 0));
                    end if;

                    pu_waddr  <= pu_cnt;
                    pu_wdata  <= std_logic_vector(v_np) & std_logic_vector(v_spd);
                    pu_swdata <= std_logic_vector(v_bs) & std_logic_vector(v_dp);
                    pu_we     <= '1';
                    pu_swe    <= '1';
                    if pu_cnt = 255 then
                        pu_busy  <= '0';
                        pu_state <= PU_IDLE;
                    else
                        pu_cnt   <= pu_cnt + 1;
                        pu_state <= PU_REQ;
                    end if;
            end case;
        end if;
    end process p_part;

    ------------------------------------------------------------------------
    -- particle memories: read ports muxed updater (vblank) / render
    -- prefetch (active); surf write port muxed updater / raster detect --
    -- all pre-muxed single ports, port-exclusive by construction.
    ------------------------------------------------------------------------
    part_raddr <= pu_cnt when pu_busy = '1' else prf_col;
    surf_raddr <= pu_cnt when pu_busy = '1' else prf_col;
    surf_waddr <= pu_waddr when pu_swe = '1' else sw_addr;
    surf_wdata <= pu_swdata when pu_swe = '1' else sw_data;
    surf_we    <= pu_swe or sw_we;

    p_partram : process(clk)
    begin
        if rising_edge(clk) then
            part_q <= part_ram(to_integer(part_raddr));
            if pu_we = '1' then
                part_ram(to_integer(pu_waddr)) <= pu_wdata;
            end if;
        end if;
    end process p_partram;

    p_surfram : process(clk)
    begin
        if rising_edge(clk) then
            surf_q <= surf_ram(to_integer(surf_raddr));
            if surf_we = '1' then
                surf_ram(to_integer(surf_waddr)) <= surf_wdata;
            end if;
        end if;
    end process p_surfram;

    ------------------------------------------------------------------------
    -- warp line buffers: SINGLE BANK, canonical 1W1R (redshift). The write
    -- lands the S0-registered pixel at wx while the read address
    -- (registered at S8) is 7 stages behind, so dx <= +7 reads the current
    -- line and dx >= +8 reads the not-yet-overwritten previous line -- a
    -- one-line vertical seam at the sign change, invisible in practice.
    ------------------------------------------------------------------------
    p_lbY : process(clk)
    begin
        if rising_edge(clk) then
            by_q <= lbY(to_integer(q_addr));
            if av(1) = '1' then
                lbY(to_integer(wx)) <= std_logic_vector(r0_y);
            end if;
        end if;
    end process p_lbY;

    p_lbU : process(clk)
    begin
        if rising_edge(clk) then
            bu_q <= lbU(to_integer(u_addr));
            if av(1) = '1' then
                lbU(to_integer(wx(10 downto 1))) <= std_logic_vector(r0_u);
            end if;
        end if;
    end process p_lbU;

    p_lbV : process(clk)
    begin
        if rising_edge(clk) then
            bv_q <= lbV(to_integer(v_addr));
            if av(1) = '1' then
                lbV(to_integer(wx(10 downto 1))) <= std_logic_vector(r0_v);
            end if;
        end if;
    end process p_lbV;

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

end architecture kudzu;
