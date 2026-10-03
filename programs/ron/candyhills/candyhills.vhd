-- candyhills.vhd  (v1.0 — "Candy Hills")
--
-- A real-time, side-scrolling 2D generative landscape: large, long, sweeping
-- rolling candy hills filled with bold 45-degree stripes, dotted with round
-- bushes, under a graded pastel sky, with sunlit crests fading to shaded
-- valleys.  The view scrolls continuously and never repeats.
--
-- Streaming program (no frame buffer): the input SYNC drives the raster; the
-- input picture is used only by S10 Video Sky (hence program_type processing).
--
-- HILLS
--   Each of 3 parallax layers has a crest = baseline - amplitude*(sinA + sinB/2)
--   of a VERY LONG primary sine (~5000/6800/9500 px = several screens) plus a
--   shorter secondary sine, so the ridge sometimes climbs or falls across
--   multiple screens.  Amplitude (P12) is a SMOOTH, glided per-frame gain, and
--   the crest is NOT clamped: at high amplitude it simply runs off the top or
--   bottom of frame (full hill / full sky).  Waves are evaluated multiply-free
--   with per-pixel phase accumulators; back layers scroll slower (parallax).
--   SCROLL: K1 sets 1..95 sub-steps of 1/8 base speed per frame (continuous
--   1/8x..~12x); each sub-step moves every layer's world-x by an exact 1/128-px
--   constant, and the crest phase advances one pixel's worth each time world-x
--   crosses a whole pixel — hills, crest line, stripes and bushes share ONE
--   integer world grid and move as a rigid picture.  S11 reverses it.
--
-- STRIPES
--   45-degree diagonals, ENTIRELY WORLD-ANCHORED: crest, stripes and fan
--   all scroll at identical speed (nothing is screen-anchored, so nothing can
--   slip, shear or reset).  The x gradient (world-x * F/16) equals the y
--   gradient (per-layer per-line accumulator), so K5 changes width (64
--   near-continuous steps, 1024..66 px) while the angle stays 45.  CREST FAN
--   (K2) warps world-x by the crest sine's own COSINE — its derivative
--   tracks hill height, so stripes widen over crests and tighten in valleys
--   (up to ~28..82 deg at max, any width), glued to the terrain like Tiny
--   Wings wedges.  A gentle sine curve of py bends them.
--
-- SHADING
--   Crest-lit gradient that FOLLOWS THE CURVE OF THE CREST: the top ~15% of
--   frame height below the local crest is fully lit (+120 luma, peaks and
--   valleys alike); below the band the lift fades out over 120 px while a
--   multiplicative shade ramps to 37.5% brightness at 640 px past the band.
--
-- SKY
--   Vertical luma gradient, deeper at the top, paling toward the horizon;
--   K3 sweeps its hue through 32 pastel steps (rose .. warm cream).
--
-- Fixed colours stored U/V (Cb/Cr) SWAPPED for the Videomancer HW convention
-- (stored U = Cr, V = Cb), matching the mondrian / glitchscape palettes.
--
-- Controls:
--   K1 Scroll Speed (continuous)  K2 Fan Amount   K3 Sky Tint
--   K4 Palette                    K5 Stripe Width K6 Bush Density (0 = none)
--   S7 3rd Layer  S8 Distance Haze  S9 Crest Line  S10 Video Sky  S11 Reverse
--   P12 Hill Amplitude (headline: flat plains -> ridges running off-screen)
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture candyhills of program_top is

    constant LATENCY : natural := 14;         -- datapath stages s1..s11 (+ s5a, s6b, s7b)
    constant C_BLK  : unsigned(9 downto 0) := to_unsigned(64, 10);

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- Stripe soft-edge width in PIXELS (2^SOFTW px).  The compare threshold is
    -- thr = 2^SOFTW * gx(row) so the blend zone is a CONSTANT pixel width at
    -- any stripe width/fan — a fixed phase threshold ballooned to 100+ px of
    -- washed-out mid-colour on the fanned-wide crest stripes (the hills read
    -- brighter at the top, darker at the bottom).
    constant SOFTW  : natural := 3;
    -- STRIPES: 45-degree diagonals, ENTIRELY WORLD-ANCHORED so the crest, the
    -- stripes and the fan all scroll at IDENTICAL speed (any screen-space
    -- term makes some rows slip — every earlier fan artifact was that leak).
    -- gx from the world-x base (x'*F/16, mod-1024 seamless across the world
    -- wrap); gy from a per-layer per-LINE accumulator (ydacc slope = F<<L).
    -- CREST FAN (K2): x' = x + cosA(world-x)*fan — the crest sine's own
    -- cosine (one shared LUT on the winner's angle) — so dx'/dx = 1 - k*sinA
    -- tracks hill height: stripes WIDEN over tall hills and tighten in
    -- valleys, glued to the hill like the Tiny Wings wedges, and the effect
    -- is relative to the stripe width (k <= 0.86 fg / 0.63 mid / 0.45 bg at
    -- max, so gx never flips): no clamp, no reset, no slip.  A sine curve of py
    -- bends the stripes.  CREST-LIT SHADING: +120 luma in the lit band,
    -- fading to 37.5% brightness below (see SHADING above).
    constant FANWSH  : natural := 6;          -- fan x-warp shift (cos*fan_eff >> FANWSH px)
    constant FANPOST : natural := 11;         -- ydacc fraction bits (phase = acc >> FANPOST)
    constant CURVESH : natural := 3;          -- stripe sine-curve amplitude shift
    constant THICK  : integer := 5;           -- crest-line thickness (px, fixed)

    --------------------------------------------------------------------------
    -- Per-layer constants.  Index 0 = foreground, 1 = mid, 2 = background.
    --------------------------------------------------------------------------
    type t_l3int is array(0 to 2) of integer;

    -- Phase step per pixel for the two crest sines (= 2^20 / wavelength_px).
    -- VERY LONG primary wavelengths (~5000/6800/9500 px = several screens).
    constant STEP_A   : t_l3int := (210, 154, 110);      -- fg/mid/bg primary sine
    constant STEP_B   : t_l3int := (699, 524, 374);      -- fg/mid/bg secondary sine
    -- Scroll SUB-STEP per layer (K1 runs 1..95 of them per frame; 8 = the
    -- base speed of 2 / 1 / 0.375 px per frame for fg / mid / bg).
    --   world-x : VEL16 in 1/128 px (scrollPx has 7 fraction bits)
    --   crest   : +-STEP whenever world-x crosses a WHOLE pixel, so the crest
    --             phase is always exactly floor(world-x) * STEP.
    -- The crest therefore sits on the SAME integer world grid as the stripes
    -- and bushes: the whole scene moves as one rigid picture in 1-px hops.
    -- (A sub-pixel crest under integer-gridded bushes made every bush on a
    -- slope shiver ~1 px against the crest line at fractional speeds.)
    constant VEL16    : t_l3int := (32, 16, 6);
    -- Haze shift toward HAZE colour per layer (0 = none).  bg most hazed.
    constant HZ       : t_l3int := (0, 2, 1);

    -- Stripe frequency F (K5) has 4 FRACTION bits: gx = F/16 phase units per
    -- px, width = 16384/F px.  Seamless across the 16384-px world wrap for ANY
    -- integer F (16384*F/16 = 1024*F = 0 mod 1024).  K5 maps exponentially:
    -- idx = 63 - K5[9:4], F = (16 + idx[3:0]) << idx[5:4] = 16..248 ->
    -- 1024..66 px in 64 steps of ~3-6% (was 8 fixed steps).

    --------------------------------------------------------------------------
    -- Candy stripe palettes (8), stored U = Cr, V = Cb.  Selected by K4.
    --   0 Green/Magenta  1 Mint/Coral  2 Sunset  3 Mono
    --   4 Bubblegum (pink/cyan)  5 Lemon/Berry  6 Aqua/Tangerine  7 Lavender/Lime
    --------------------------------------------------------------------------
    type t_pal8 is array(0 to 7) of integer;
    constant P_AY : t_pal8 := (692, 797, 672, 740, 725, 863, 628, 677);  -- colour A luma
    constant P_AU : t_pal8 := (361, 372, 732, 512, 722, 596, 235, 542);  -- colour A Cr
    constant P_AV : t_pal8 := (280, 447, 268, 512, 554, 229, 586, 672);  -- colour A Cb
    constant P_BY : t_pal8 := (486, 631, 486, 360, 765, 324, 680, 787);  -- colour B luma
    constant P_BU : t_pal8 := (836, 775, 737, 512, 309, 709, 754, 464);  -- colour B Cr
    constant P_BV : t_pal8 := (554, 405, 645, 512, 600, 578, 241, 226);  -- colour B Cb

    -- BUSHES: light-green circles outlined in dark green, growing straight out
    -- of a crest (fg/mid/bg) at 100/50/25 % size, in RANDOM GROUPS of 3 or 6
    -- (K6 density: d of every 16 eight-slot regions hold a group; 0 = none).
    -- The dome is a TRUE circle test dx^2 + dy^2 <= R^2 against a per-slot ANCHOR
    -- (the crest height under the dome centre, held constant across the slot), so
    -- the circle never leans with the crest slope; it is CLIPPED per column at the
    -- crest (draw only above it), and carries an EVEN radial outline in the
    -- annulus (R-T)^2 .. R^2.  Fixed grassy greens (U = Cr, V = Cb swapped); fill
    -- fill light green, outline dark.  Per layer L: slot = 128>>L px
    -- (log 7-L), radius 48>>L, centre 64>>L.  No per-frame state.
    constant BSLOTLOG : natural := 7;    -- fg slot = 128 px; layer L = 128>>L
    constant BR       : integer := 48;   -- fg bush radius; layer L = 48>>L
    constant BFILL_Y  : integer := 735;  constant BFILL_U : integer := 356;  constant BFILL_V : integer := 292;
    constant BLINE_Y  : integer := 250;  constant BLINE_U : integer := 360;  constant BLINE_V : integer := 296;
    -- Per-layer outer radius^2 and inner (fill) radius^2 = (R - T)^2, T = outline
    -- thickness (5/3/2 px for fg/mid/bg).  fg 48/43, mid 24/21, bg 12/10.
    type t_int3 is array(0 to 2) of integer;
    constant BR2   : t_int3 := (2304, 576, 144);   -- 48^2 / 24^2 / 12^2
    constant BRIN2 : t_int3 := (1849, 441, 100);   -- 43^2 / 21^2 / 10^2

    constant HAZE_Y  : integer := 820;   -- pale desaturated distance tint
    constant HAZE_U  : integer := 500;
    constant HAZE_V  : integer := 500;
    constant SKY_YT  : integer := 744;   -- sky luma at the TOP of frame
                                         -- (+~120..145 by the horizon/bottom)
    -- SKY TINT (K3): a 32-step sweep through pastel sky hues — rose ->
    -- lavender -> periwinkle -> sky blue -> mint -> lime-cream -> warm cream
    -- (100% = the warm cream of the old +-64 tint axis).  Stored U = Cr,
    -- V = Cb (HW swap convention).
    type t_sky32 is array(0 to 31) of integer range 0 to 1023;
    constant SKYT_U : t_sky32 := (577, 571, 565, 560, 554, 548, 539, 529, 520, 510, 500, 485, 466, 448, 430, 411,
                                  408, 421, 433, 445, 458, 467, 470, 473, 476, 479, 484, 493, 502, 511, 520, 529);
    constant SKYT_V : t_sky32 := (527, 535, 542, 550, 558, 566, 570, 574, 578, 582, 586, 582, 576, 569, 562, 555,
                                  546, 533, 521, 509, 496, 484, 474, 463, 452, 441, 432, 430, 429, 428, 426, 425);

    -- Group-placement S-box (PRESENT cipher, a 4-bit permutation): each run of
    -- 16 regions holds every value once, so K6 density d lights exactly d/16
    -- of them — uniform even for the fg layer's 16 regions per world wrap.
    type t_sbox is array(0 to 15) of integer range 0 to 15;
    constant SBOX : t_sbox := (12, 5, 6, 11, 9, 0, 10, 13, 3, 14, 15, 8, 4, 7, 1, 2);
    constant BXOR : t_l3int := (0, 9, 6);   -- per-layer decorrelation

    function clamp10(v : signed) return unsigned is
    begin
        if v < 0 then       return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else                return unsigned(resize(v, 10));
        end if;
    end function;

    -- Triangle wave of a 10-bit phase, signed range -512..510 (~ sine scale).
    function tri10(p : unsigned(9 downto 0)) return signed is
        variable r : unsigned(8 downto 0);
    begin
        r := p(8 downto 0);
        if p(9) = '1' then r := to_unsigned(511, 9) - r; end if;  -- fold to 0..511..0
        return resize(signed(resize(shift_left(resize(r, 11), 1), 11)) - 512, 10);
    end function;

    --------------------------------------------------------------------------
    -- Array types.
    --------------------------------------------------------------------------
    type t_l3u10  is array(0 to 2) of unsigned(9 downto 0);
    type t_l3u11  is array(0 to 2) of unsigned(10 downto 0);
    type t_l3u14  is array(0 to 2) of unsigned(13 downto 0);
    type t_l3u20  is array(0 to 2) of unsigned(19 downto 0);
    type t_l3s10  is array(0 to 2) of signed(9 downto 0);
    type t_l3s12  is array(0 to 2) of signed(11 downto 0);
    type t_l3s13  is array(0 to 2) of signed(12 downto 0);
    type t_l3s14  is array(0 to 2) of signed(13 downto 0);
    type t_l3slv  is array(0 to 2) of std_logic_vector(9 downto 0);
    type t_l3u9   is array(0 to 2) of unsigned(8 downto 0);
    type t_l3u6   is array(0 to 2) of unsigned(5 downto 0);
    type t_l3u13  is array(0 to 2) of unsigned(12 downto 0);
    type t_bcode  is array(0 to 2) of std_logic_vector(1 downto 0);

    --------------------------------------------------------------------------
    -- Raster position / animation.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    -- Virtual x: each scan runs a PRE-ROLL x = -64..-1 before the active line
    -- and a POST-ROLL 64 px past its right edge (in hblank; 128 of SD's 138
    -- blank clocks), so every bush slot centre touching the screen is
    -- evaluated on every line — see the bush anchor store.
    constant ROLL    : integer := 64;       -- = fg half-slot (largest centre offset)
    signal xv        : signed(12 downto 0) := to_signed(-ROLL, 13);
    signal post_cnt  : unsigned(6 downto 0) := (others => '0');
    signal prev_avid : std_logic := '0';
    signal scan_par  : std_logic := '0';    -- anchor-store bank written by this scan
    signal py        : unsigned(10 downto 0) := (others => '0');
    signal frame_act : std_logic := '0';

    signal act_h     : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal max_py    : unsigned(10 downto 0) := (others => '0');

    -- Per-pixel crest phase accumulators (reset to per-frame offset at line start).
    type t_l3u21s is array(0 to 2) of unsigned(20 downto 0);
    signal phA       : t_l3u20 := (others => (others => '0'));
    signal phB       : t_l3u20 := (others => (others => '0'));
    signal offA      : t_l3u20 := (others => (others => '0'));  -- = floor(world-x)*STEP_A
    signal offB      : t_l3u20 := (others => (others => '0'));
    signal scrollPx  : t_l3u21s := (others => (others => '0')); -- 1/128-px units
    signal sub_cnt   : unsigned(6 downto 0) := (others => '0'); -- scroll sub-steps left

    --------------------------------------------------------------------------
    -- Per-frame latched parameters / derived values.
    --------------------------------------------------------------------------
    signal s_spdm    : unsigned(6 downto 0) := to_unsigned(16, 7);  -- K1 sub-steps 1..95
    signal amp_g     : unsigned(9 downto 0) := to_unsigned(512, 10); -- P12 glided (10b)
    signal s_gainA   : unsigned(6 downto 0) := to_unsigned(70, 7);   -- P12 amplitude (0..127)
    signal s_F       : unsigned(8 downto 0) := to_unsigned(32, 9);   -- K5 stripe freq (x16)
    signal k5h       : unsigned(9 downto 0) := to_unsigned(760, 10); -- K5 with a +-12 deadband
    signal k3h       : unsigned(9 downto 0) := to_unsigned(1023, 10); -- K3 with a +-12 deadband
    signal s_fane    : unsigned(6 downto 0) := to_unsigned(44, 7);   -- K2 fan x1.375 (0..86)
    signal s_bden    : unsigned(3 downto 0) := to_unsigned(4, 4);   -- K6 bush density (0..15)
    signal s_palsel  : integer range 0 to 7 := 0;     -- K4 palette (8)

    signal s_lay3    : std_logic := '1';   -- S7
    signal s_haze    : std_logic := '1';   -- S8
    signal s_creston : std_logic := '1';   -- S9 (default on)
    signal s_vidsky  : std_logic := '0';   -- S10 (replace sky with input video)
    signal s_rev     : std_logic := '0';   -- S11 reverse scroll

    -- Per-layer baselines near vertical centre (parallax-offset).
    signal base_y    : t_l3u11 := (others => to_unsigned(360, 11));
    -- Per-layer 45-degree diagonal phase accumulator: slope per line =
    -- freq << layer (in 1/2^FANPOST phase units) — the same base gradient as
    -- the stripes' x term, which is what keeps them at 45 degrees.
    type t_l3u21 is array(0 to 2) of unsigned(20 downto 0);
    signal ydacc     : t_l3u21 := (others => (others => '0'));

    -- Crest-lit shading: lit-band depth (~15% of act_h), latched per frame.
    signal s_litd : unsigned(10 downto 0) := to_unsigned(112, 11);

    -- Selected base palette colours, registered every clock (pipelines the 8:1
    -- palette mux out of the per-frame haze path so it does not gate timing).
    signal sel_ay, sel_au, sel_av : integer range 0 to 1023 := 512;
    signal sel_by, sel_bu, sel_bv : integer range 0 to 1023 := 512;
    -- Crest base colour = the DARKER stripe colour, darkened ~0.7x (its hue kept).
    -- Resolved every clock so the compare+divide stays OFF the per-frame gC path.
    signal sel_cy, sel_cu, sel_cv : integer range 0 to 1023 := 256;
    -- Mid-band colour = average of A and B, also resolved every clock.
    signal sel_my, sel_mu, sel_mv : integer range 0 to 1023 := 512;

    -- HAZE SEQUENCER: the 48 per-layer colour channels (gA/gB/gM/gC x y/u/v x
    -- 3 layers) are written one per clock in vblank by ONE shared lerp
    -- c + (h - c) >> sh, 3-stage pipelined — replaces 24 parallel lerp chains
    -- (~500 LC of adders that only change once a frame).  idx = layer(5:4) &
    -- channel(3:0); channel 12..15 and layer 3 are no-ops.  Starts 4 clocks
    -- after vsync so the palette/haze latched at vsync has settled into sel_*.
    signal hz_go  : std_logic_vector(3 downto 0) := (others => '0');
    signal hz_i   : unsigned(6 downto 0) := (others => '1');   -- bit 6 = idle
    signal hz1_v, hz2_v : std_logic := '0';
    signal hz1_l, hz2_l : unsigned(1 downto 0) := (others => '0');
    signal hz1_k, hz2_k : unsigned(3 downto 0) := (others => '0');
    signal hz1_c, hz2_c : signed(10 downto 0) := (others => '0');
    signal hz1_hy       : std_logic := '0';                          -- channel is luma (HAZE_Y)
    signal hz1_sh       : integer range 0 to 2 := 0;
    signal hz2_d        : signed(10 downto 0) := (others => '0');

    -- Per-frame hazed stripe colours per layer.
    signal gA_y, gA_u, gA_v : t_l3u10 := (others => C_MID);
    signal gB_y, gB_u, gB_v : t_l3u10 := (others => C_MID);
    signal gM_y, gM_u, gM_v : t_l3u10 := (others => C_MID);
    signal gC_y, gC_u, gC_v : t_l3u10 := (others => C_MID);
    signal sky_u, sky_v : unsigned(9 downto 0) := C_MID;
    -- Sky gradient: luma per LINE (py changes only at hsync, and the 13-clock
    -- pipeline never straddles a line in the active region).
    signal s_skysh : integer range 0 to 2 := 0;   -- per-frame py scaling (by act_h)
    signal sky_ly  : unsigned(9 downto 0) := to_unsigned(SKY_YT, 10);

    --------------------------------------------------------------------------
    -- Crest sine LUTs (primary + secondary per layer).
    --------------------------------------------------------------------------
    signal angA, angB : t_l3slv;
    signal sinA, sinB : t_l3s10;
    -- Crest-fan cosine: ONE shared LUT fed by the WINNER's primary angle
    -- (carried down the pipe and muxed at s5b).  Tapping cos_out on all three
    -- crest LUTs costs 3 full cosine tables (+9 EBR, overflows the part);
    -- one shared instance is +3.
    signal fan_cos    : signed(9 downto 0);

    -- Stripe sine-curve LUT (bends the stripes like the crest).
    signal curve_ang : std_logic_vector(9 downto 0) := (others => '0');
    signal curve_sin : signed(9 downto 0);

    -- Datapath-local copies of per-frame params, re-registered every clock so
    -- placement keeps them next to the multipliers (avoids long routes from the
    -- config/SPI region into the pixel pipeline).
    signal d_gainA : unsigned(6 downto 0) := to_unsigned(70, 7);
    signal d_F     : unsigned(8 downto 0) := to_unsigned(32, 9);
    signal d_fane  : unsigned(6 downto 0) := to_unsigned(44, 7);

    --------------------------------------------------------------------------
    -- Datapath stage registers.
    --------------------------------------------------------------------------
    -- s1: capture position, both crest sines, stripe-x, texture
    signal r1_par, r2_par, r3_par, r4_par : std_logic := '0';   -- scan bank, carried
    signal r1_py   : unsigned(10 downto 0) := (others => '0');
    signal r1_sinA, r1_sinB : t_l3s10 := (others => (others => '0'));
    signal r1_ang  : t_l3slv := (others => (others => '0'));   -- crest-fan angle carry
    signal r1_sx   : t_l3u14 := (others => (others => '0'));
    -- s2: combined displacement = sinA + sinB/2
    signal r2_disp : t_l3s12 := (others => (others => '0'));
    signal r2_py   : unsigned(10 downto 0) := (others => '0');
    signal r2_sx   : t_l3u14 := (others => (others => '0'));
    signal r2_ang  : t_l3slv := (others => (others => '0'));
    -- s3: amplitude-scaled displacement (smooth gain multiply)
    signal r3_scl  : t_l3s13 := (others => (others => '0'));
    signal r3_py   : unsigned(10 downto 0) := (others => '0');
    signal r3_sx   : t_l3u14 := (others => (others => '0'));
    signal r3_ang  : t_l3slv := (others => (others => '0'));
    -- s4: crest = baseline - scaled displacement (signed, may run off-screen)
    signal r4_crest : t_l3s13 := (others => (others => '0'));
    signal r4_py   : unsigned(10 downto 0) := (others => '0');
    signal r4_sx   : t_l3u14 := (others => (others => '0'));
    signal r4_ang  : t_l3slv := (others => (others => '0'));
    -- s5: frontmost covering layer / sky / crest-line / winner's world-x
    -- s5a: the three crest subtracts (carry chains), split off from the priority
    --      select so the compares route alone.  dd(l) = py - crest(l); sign = the
    --      "py below crest l" test; the winner's dd is the crest-line depth.
    signal r5a_dd  : t_l3s14 := (others => (others => '0'));   -- 14b: py-crest, no overflow
    signal r5a_sx  : t_l3u14 := (others => (others => '0'));
    signal r5a_py  : unsigned(10 downto 0) := (others => '0');
    signal r5a_ang : t_l3slv := (others => (others => '0'));
    type t_l3u4 is array(0 to 2) of unsigned(3 downto 0);
    signal r5a_bh  : t_l3u4 := (others => (others => '0'));   -- bush region S-box (density)
    signal r5a_bl  : std_logic_vector(2 downto 0) := "000";   -- bush group length 6 (else 3)

    signal r5_layer : integer range 0 to 2 := 0;
    signal r5_sky  : std_logic := '0';
    signal r5_cl   : std_logic := '0';
    signal r5_sx   : unsigned(13 downto 0) := (others => '0');  -- world-x of winner
    signal r5_py   : unsigned(10 downto 0) := (others => '0');
    signal r5_fang : std_logic_vector(9 downto 0) := (others => '0');  -- winner's crest angle
    signal r5_glow : unsigned(6 downto 0) := (others => '0');   -- crest lift (120 in lit band -> 0)
    signal r5_shade : unsigned(5 downto 0) := (others => '0');  -- shade 0..40 (-> 37.5% bright)
    -- s6: base frequency (freq<<layer) + the crest-fan multiply.
    signal r6_layer : integer range 0 to 2 := 0;
    signal r6_sky  : std_logic := '0';
    signal r6_cl   : std_logic := '0';
    signal r6_freqeff : unsigned(10 downto 0) := (others => '0');  -- F << layer
    signal r6_cos  : signed(9 downto 0) := (others => '0');       -- fan_cos registered (BRAM out)
    signal r6_glow : unsigned(6 downto 0) := (others => '0');
    signal r6_shade : unsigned(5 downto 0) := (others => '0');
    signal r6_sx   : unsigned(13 downto 0) := (others => '0');
    signal r6_py   : unsigned(10 downto 0) := (others => '0');
    -- s6b: base multiply sx(9:0)*freq_eff + fan multiply cos*gain (small).
    --      Only phase(9:0) survives downstream, so both are mod-1024.
    signal r6b_sx   : unsigned(13 downto 0) := (others => '0');
    signal r6b_feff : unsigned(10 downto 0) := (others => '0');
    signal r6b_fan : signed(17 downto 0) := (others => '0');
    signal r6b_glow : unsigned(6 downto 0) := (others => '0');
    signal r6b_shade : unsigned(5 downto 0) := (others => '0');
    signal r6b_layer : integer range 0 to 2 := 0;
    signal r6b_sky : std_logic := '0';
    signal r6b_cl  : std_logic := '0';
    signal r6b_py  : unsigned(10 downto 0) := (others => '0');
    -- s7: stripe phase = base + fanraw>>FANPOST
    signal r7_layer : integer range 0 to 2 := 0;
    signal r7_sky  : std_logic := '0';
    signal r7_cl   : std_logic := '0';
    signal r7_sxw  : unsigned(13 downto 0) := (others => '0');   -- fan-warped world-x
    signal r7_feff : unsigned(10 downto 0) := (others => '0');
    -- s7b: stripe base product sx'*Feff split into operand halves (mod 2^14)
    --      + the y terms (diagonal + curve) pre-summed.
    signal r7b_plo : unsigned(13 downto 0) := (others => '0');
    signal r7b_phi : unsigned(7 downto 0) := (others => '0');
    signal r7b_yc  : unsigned(9 downto 0) := (others => '0');
    signal r7b_layer : integer range 0 to 2 := 0;
    signal r7b_sky, r7b_cl : std_logic := '0';
    signal r7b_glow : unsigned(6 downto 0) := (others => '0');
    signal r7b_shade : unsigned(5 downto 0) := (others => '0');
    signal r7b_bcode : t_bcode := (others => "00");
    signal r7_py   : unsigned(10 downto 0) := (others => '0');
    signal r7_glow : unsigned(6 downto 0) := (others => '0');
    signal r7_shade : unsigned(5 downto 0) := (others => '0');
    -- s8: stripe angle = phase + diagonal + sine curve(py)
    signal r8_layer : integer range 0 to 2 := 0;
    signal r8_sky  : std_logic := '0';
    signal r8_cl   : std_logic := '0';
    signal r8_ang  : std_logic_vector(9 downto 0) := (others => '0');
    signal r8_thr  : unsigned(7 downto 0) := to_unsigned(16, 8);  -- soft-edge threshold (2^SOFTW px * gx)
    signal r8_glow : unsigned(6 downto 0) := (others => '0');
    signal r8_shade : unsigned(5 downto 0) := (others => '0');
    -- s9: stripe wave sampled
    signal r9_layer : integer range 0 to 2 := 0;
    signal r9_sky  : std_logic := '0';
    signal r9_cl   : std_logic := '0';
    signal r9_ssin : signed(9 downto 0) := (others => '0');
    signal r9_thr  : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal r9_glow : unsigned(6 downto 0) := (others => '0');
    signal r9_shade : unsigned(5 downto 0) := (others => '0');
    -- s10: assembled base colour (+ sky flag for the video-sky option)
    signal r10_y, r10_u, r10_v : unsigned(9 downto 0) := C_MID;
    signal r10_sky : std_logic := '0';
    signal r10_shade : unsigned(5 downto 0) := (others => '0');

    -- BUSH sub-pipeline (parallel to the colour path; overrides colour at s10).
    --   LINE-LOCAL state only (the per-slot crest anchor is re-derived every
    --   scanline from the crest itself — no per-frame memory), so bushes scroll
    --   cleanly off both edges with no popping.  s4 latches the per-slot anchor;
    --   s5b captures all-layer world-x / (py-crest); s6 does per-layer geometry
    --   (squared radial distance from the dome centre + group placement); s6b
    --   resolves a per-layer 2-bit code (00 none / 01 fill / 10 outline);
    --   s7..s9 carry; s10 picks the frontmost bush in front of the winning hill.
    -- ANCHOR STORE (one 256x16 EBR per layer): each scan writes the crest at
    -- every slot CENTRE (world slot index), the next scan reads it back, so
    -- every dome sits on the exact crest under its centre — no extrapolation
    -- (the old left-edge slope guess drifted bushes up/down near screen-left).
    -- The crest is a function of world-x only, so the previous line's value
    -- is exact.  Banked by scan parity: write bank = this scan, read bank =
    -- the previous scan -> never a same-address read-during-write.
    type t_anc_mem is array(0 to 255) of std_logic_vector(15 downto 0);
    type t_l3a8 is array(0 to 2) of unsigned(7 downto 0);
    type t_l3v16 is array(0 to 2) of std_logic_vector(15 downto 0);
    signal anc_we    : std_logic_vector(2 downto 0);
    signal anc_waddr : t_l3a8;
    signal anc_raddr : t_l3a8;
    signal anc_rdata : t_l3v16;
    signal r5_anc    : t_l3s14 := (others => (others => '0'));   -- registered anchor (s5)
    signal r5_sxL   : t_l3u14 := (others => (others => '0'));    -- world-x per layer
    signal r5_ddL   : t_l3s14 := (others => (others => '0'));    -- py - crest per layer
    signal r5_bpl   : std_logic_vector(2 downto 0) := "000";     -- slot is in a placed group
    signal r6_adx   : t_l3u6 := (others => (others => '0'));     -- |dx| per layer
    signal r6_dyi   : t_l3u6 := (others => (others => '0'));     -- dy = crest-py per layer
    signal r6_bok   : std_logic_vector(2 downto 0) := "000";     -- placed + above crest
    signal r6b_d2   : t_l3u13 := (others => (others => '0'));    -- dx^2 + dy^2 (own stage)
    signal r6b_bok  : std_logic_vector(2 downto 0) := "000";
    signal r7_bcode, r8_bcode, r9_bcode : t_bcode := (others => "00");

    --------------------------------------------------------------------------
    -- Sync alignment pipe + output.
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);
    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Crest sine LUTs.
    --------------------------------------------------------------------------
    gen_crest_lut : for l in 0 to 2 generate
        angA(l) <= std_logic_vector(phA(l)(19 downto 10));
        angB(l) <= std_logic_vector(phB(l)(19 downto 10));
        uA : entity work.sin_cos_full_lut_10x10
            port map (angle_in => angA(l), sin_out => sinA(l), cos_out => open);
        uB : entity work.sin_cos_full_lut_10x10
            port map (angle_in => angB(l), sin_out => sinB(l), cos_out => open);
    end generate;

    -- Stripe curve: sine of py (wavelength ~512 px) -> gentle sine bend.
    curve_ang <= std_logic_vector(resize(shift_left(r7_py, 1), 10));
    u_curve : entity work.sin_cos_full_lut_10x10
        port map (angle_in => curve_ang, sin_out => curve_sin, cos_out => open);

    -- Crest-fan cosine: shared single LUT on the winner's primary angle
    -- (r5_fang, registered at s5b; output registered into r6_cos at s6 —
    -- BRAM read port registered before the s6b multiply).
    u_fancos : entity work.sin_cos_full_lut_10x10
        port map (angle_in => r5_fang, sin_out => open, cos_out => fan_cos);

    --------------------------------------------------------------------------
    -- Bush anchor store: per layer, write at s4 (slot centre), read addressed
    -- from s3 so the data lines up with s4 and is re-registered into r5_anc.
    --------------------------------------------------------------------------
    gen_anc : for L in 0 to 2 generate
        signal mem : t_anc_mem := (others => (others => '0'));
    begin
        anc_we(L)    <= '1' when r4_sx(L)(BSLOTLOG - 1 - L downto 0)
                                 = to_unsigned(2 ** (BSLOTLOG - 1 - L), BSLOTLOG - L)
                        else '0';
        anc_waddr(L) <= r4_par & r4_sx(L)(BSLOTLOG + 6 - L downto BSLOTLOG - L);
        anc_raddr(L) <= (not r3_par) & r3_sx(L)(BSLOTLOG + 6 - L downto BSLOTLOG - L);
        p_anc : process(clk)
        begin
            if rising_edge(clk) then
                if anc_we(L) = '1' then
                    mem(to_integer(anc_waddr(L))) <= std_logic_vector(resize(r4_crest(L), 16));
                end if;
                anc_rdata(L) <= mem(to_integer(anc_raddr(L)));
            end if;
        end process;
    end generate;

    --------------------------------------------------------------------------
    -- Raster position, phase accumulators, per-frame parameter latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_kd  : signed(10 downto 0);
        variable v_ki  : unsigned(5 downto 0);
        variable v_fn  : unsigned(5 downto 0);
        variable v_ns  : unsigned(20 downto 0);
        variable v_sa, v_sb : unsigned(19 downto 0);
        variable v_kv  : unsigned(20 downto 0);
        variable v_gd  : signed(10 downto 0);
        variable v_g   : unsigned(10 downto 0);
        variable v_hc : integer range 0 to 1023;
        variable v_hr : unsigned(9 downto 0);
        variable v_hd : signed(10 downto 0);
        variable v_step : std_logic;
        variable v_k3d : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- Register the selected base palette colours every clock (the 8:1 mux
            -- is then off the per-frame haze path).
            sel_ay <= P_AY(s_palsel); sel_au <= P_AU(s_palsel); sel_av <= P_AV(s_palsel);
            sel_by <= P_BY(s_palsel); sel_bu <= P_BU(s_palsel); sel_bv <= P_BV(s_palsel);
            -- Crest base = darker of A/B (lower luma), luma *~0.7, hue kept.
            if sel_ay <= sel_by then
                sel_cy <= sel_ay - sel_ay / 4 - sel_ay / 8; sel_cu <= sel_au; sel_cv <= sel_av;
            else
                sel_cy <= sel_by - sel_by / 4 - sel_by / 8; sel_cu <= sel_bu; sel_cv <= sel_bv;
            end if;

            sel_my <= (sel_ay + sel_by) / 2; sel_mu <= (sel_au + sel_bu) / 2; sel_mv <= (sel_av + sel_bv) / 2;

            -- HAZE SEQUENCER (see signal comment).  s1: operand select.
            hz_go <= hz_go(2 downto 0) & '0';
            if hz_go(3) = '1' then
                hz_i <= (others => '0');
            elsif hz_i(6) = '0' then
                hz_i <= hz_i + 1;
            end if;
            case to_integer(hz_i(3 downto 0)) is
                when 0  => v_hc := sel_ay;  when 1  => v_hc := sel_au;  when 2  => v_hc := sel_av;
                when 3  => v_hc := sel_by;  when 4  => v_hc := sel_bu;  when 5  => v_hc := sel_bv;
                when 6  => v_hc := sel_my;  when 7  => v_hc := sel_mu;  when 8  => v_hc := sel_mv;
                when 9  => v_hc := sel_cy;  when 10 => v_hc := sel_cu;  when others => v_hc := sel_cv;
            end case;
            hz1_c <= to_signed(v_hc, 11);
            case to_integer(hz_i(3 downto 0)) is
                when 0 | 3 | 6 | 9 => hz1_hy <= '1';
                when others        => hz1_hy <= '0';
            end case;
            if s_haze = '1' and hz_i(5 downto 4) = "01" then    hz1_sh <= HZ(1);
            elsif s_haze = '1' and hz_i(5 downto 4) = "10" then hz1_sh <= HZ(2);
            else                                                 hz1_sh <= 0; end if;
            hz1_v <= not hz_i(6); hz1_l <= hz_i(5 downto 4); hz1_k <= hz_i(3 downto 0);
            -- s2: d = (h - c) >> sh  (sh = 0 -> no haze -> d = 0)
            if hz1_hy = '1' then v_hd := to_signed(HAZE_Y, 11) - hz1_c;
            else                 v_hd := to_signed(HAZE_U, 11) - hz1_c; end if;   -- HAZE_U = HAZE_V
            case hz1_sh is
                when 1      => hz2_d <= shift_right(v_hd, 1);
                when 2      => hz2_d <= shift_right(v_hd, 2);
                when others => hz2_d <= (others => '0');
            end case;
            hz2_c <= hz1_c; hz2_v <= hz1_v; hz2_l <= hz1_l; hz2_k <= hz1_k;
            -- s3: write c + d into the target channel register.
            v_hr := unsigned(resize(hz2_c + hz2_d, 10));
            if hz2_v = '1' and hz2_l /= "11" then
                for l in 0 to 2 loop
                    if to_integer(hz2_l) = l then
                        case to_integer(hz2_k) is
                            when 0  => gA_y(l) <= v_hr;  when 1  => gA_u(l) <= v_hr;  when 2  => gA_v(l) <= v_hr;
                            when 3  => gB_y(l) <= v_hr;  when 4  => gB_u(l) <= v_hr;  when 5  => gB_v(l) <= v_hr;
                            when 6  => gM_y(l) <= v_hr;  when 7  => gM_u(l) <= v_hr;  when 8  => gM_v(l) <= v_hr;
                            when 9  => gC_y(l) <= v_hr;  when 10 => gC_u(l) <= v_hr;  when 11 => gC_v(l) <= v_hr;
                            when others => null;
                        end case;
                    end if;
                end loop;
            end if;

            -- X counter + crest phase accumulators.  A scan = pre-roll (x =
            -- -ROLL..-1, right after the previous line's post-roll; holds at
            -- x = 0 until avid) + active line + post-roll (ROLL px past the
            -- right edge, started by avid falling).  offA/offB are the phase at
            -- x = -ROLL (a constant world offset — crest, stripes and bushes
            -- all key off the same virtual x, so they stay locked).  Lines
            -- before the frame's first active line re-seed on hsync so line 0
            -- picks up this frame's scroll.
            prev_avid <= data_in.avid;
            v_step := '0';
            if prev_avid = '1' and data_in.avid = '0' and xv >= 128 then
                post_cnt <= to_unsigned(ROLL, 7);             -- line end: post-roll
                v_step := '1';
            elsif post_cnt /= 0 then
                post_cnt <= post_cnt - 1;
                if post_cnt = 1 then                          -- post-roll done: next scan
                    xv <= to_signed(-ROLL, 13);
                    scan_par <= not scan_par;
                    for l in 0 to 2 loop
                        phA(l) <= offA(l);
                        phB(l) <= offB(l);
                    end loop;
                else
                    v_step := '1';
                end if;
            elsif v_h_edge = '1' and frame_act = '0' then     -- pre-frame re-seed
                xv <= to_signed(-ROLL, 13);
                for l in 0 to 2 loop
                    phA(l) <= offA(l);
                    phB(l) <= offB(l);
                end loop;
            elsif data_in.avid = '1' or xv < 0 then           -- active, or pre-roll
                v_step := '1';
            end if;
            if v_step = '1' then
                xv <= xv + 1;
                for l in 0 to 2 loop
                    phA(l) <= phA(l) + to_unsigned(STEP_A(l), 20);
                    phB(l) <= phB(l) + to_unsigned(STEP_B(l), 20);
                end loop;
            end if;

            -- SCROLL: sub_cnt sub-steps (loaded at vsync, <= 64 clocks, done
            -- long before the first active line).  S11 reverses by adding the
            -- two's-complement constant — one adder either way.  A sub-step is
            -- < 1 px, so world-x crosses at most one whole pixel per sub-step:
            -- exactly when its integer LSB (bit 7) flips — then the crest phase
            -- moves one pixel's worth (+-STEP), keeping it = floor(world-x)*STEP.
            if sub_cnt /= 0 then
                sub_cnt <= sub_cnt - 1;
                for l in 0 to 2 loop
                    if s_rev = '1' then v_kv := to_unsigned(2 ** 21 - VEL16(l), 21);
                    else                v_kv := to_unsigned(VEL16(l), 21); end if;
                    v_ns := scrollPx(l) + v_kv;
                    scrollPx(l) <= v_ns;
                    if s_rev = '1' then
                        v_sa := to_unsigned(2 ** 20 - STEP_A(l), 20);
                        v_sb := to_unsigned(2 ** 20 - STEP_B(l), 20);
                    else
                        v_sa := to_unsigned(STEP_A(l), 20);
                        v_sb := to_unsigned(STEP_B(l), 20);
                    end if;
                    if v_ns(7) /= scrollPx(l)(7) then
                        offA(l) <= offA(l) + v_sa;
                        offB(l) <= offB(l) + v_sb;
                    end if;
                end loop;
            end if;

            -- Sky gradient luma for the current line: py scaled so the frame
            -- height spans ~+120..145 luma at any resolution.
            case s_skysh is
                when 0      => v_g := shift_right(resize(py, 11), 3);                    -- >= 896 lines
                when 1      => v_g := shift_right(resize(py, 11), 3) + shift_right(py, 4); -- >= 640
                when others => v_g := shift_right(resize(py, 11), 2);                    -- SD
            end case;
            if v_g > 200 then v_g := to_unsigned(200, 11); end if;
            sky_ly <= to_unsigned(SKY_YT, 10) + v_g(9 downto 0);

            -- Y counter + per-frame work.  Act only on the FIRST vsync edge of
            -- a field (frame_act = saw active video): analog vsync serrates, and
            -- a repeat edge would re-scroll and latch act_h = ~0.
            if v_v_edge = '1' and frame_act = '1' then
                py        <= (others => '0');
                frame_act <= '0';
                act_h  <= py;                          -- py at vsync = active height
                max_py <= (others => '0');

                -- Advance scroll: run this frame's sub-steps; re-haze the palette.
                sub_cnt <= s_spdm;
                hz_go(0) <= '1';

                ---------------------------------------------------------------
                -- Parameter latch.
                ---------------------------------------------------------------
                -- K1: k + k/2 + 1 = 1..95 sub-steps (1/8x..~12x base speed).
                s_spdm  <= resize(unsigned(registers_in(0)(9 downto 4)), 7)
                         + resize(unsigned(registers_in(0)(9 downto 5)), 7) + 1;
                -- P12 amplitude glide: ease by diff/8 per frame (no snap) —
                -- smooths slider moves and hides pot ADC noise.  Round the step
                -- toward ZERO (+7 on negative diffs) for a SYMMETRIC +-7 deadband:
                -- a floored shift moved on any 1-count dip but needed +8 to come
                -- back, so noise random-walked the gain across steps (crests
                -- and bushes on tall peaks bounced a few px at random).
                v_gd    := signed(resize(unsigned(registers_in(7)(9 downto 0)), 11))
                         - signed(resize(amp_g, 11));
                if v_gd(10) = '1' then v_gd := v_gd + 7; end if;
                amp_g   <=unsigned(resize(signed(resize(amp_g, 11)) + shift_right(v_gd, 3), 10));
                s_gainA <= amp_g(9 downto 3);                                   -- 0..127 (7b: ~6 px/step at the tallest peaks)
                -- K3 sky tint: +-12 deadband so pot noise can't flicker the sky
                -- between adjacent hue steps.
                v_k3d   := signed(resize(unsigned(registers_in(2)(9 downto 0)), 11))
                         - signed(resize(k3h, 11));
                if v_k3d > 11 or v_k3d < -11 then k3h <= unsigned(registers_in(2)(9 downto 0)); end if;
                s_palsel <= to_integer(unsigned(registers_in(3)(9 downto 7)));  -- K4 (8 palettes)
                -- K5: +-12 deadband (pot noise must not jitter F: a 1-step change
                -- slides the world-anchored stripes by sx*dF/16), then exponential.
                v_kd    := signed(resize(unsigned(registers_in(4)(9 downto 0)), 11))
                         - signed(resize(k5h, 11));
                if v_kd > 11 or v_kd < -11 then k5h <= unsigned(registers_in(4)(9 downto 0)); end if;
                v_ki    := not k5h(9 downto 4);                                 -- 63 - K5[9:4]
                s_F     <= shift_left(resize(to_unsigned(16, 5) + v_ki(3 downto 0), 9),
                                      to_integer(v_ki(5 downto 4)));            -- 16..248

                -- Crest-lit band depth ~ 15% of frame height.
                s_litd <= shift_right(act_h, 3) + shift_right(act_h, 5);
                if act_h >= 896 then    s_skysh <= 0;
                elsif act_h >= 640 then s_skysh <= 1;
                else                    s_skysh <= 2; end if;
                s_bden <= unsigned(registers_in(5)(9 downto 6));                -- K6 0..15

                s_lay3    <= registers_in(6)(0);   -- S7
                s_haze    <= registers_in(6)(1);   -- S8
                s_creston <= registers_in(6)(2);   -- S9
                s_vidsky  <= registers_in(6)(3);   -- S10 video sky
                s_rev     <= registers_in(6)(4);   -- S11 reverse

                v_fn    := unsigned(registers_in(1)(9 downto 4));   -- K2 fan amount 0..63
                s_fane  <= resize(v_fn, 7) + resize(v_fn(5 downto 2), 7) + resize(v_fn(5 downto 3), 7);

                ---------------------------------------------------------------
                -- Per-layer baselines near vertical centre (parallax-offset).
                ---------------------------------------------------------------
                base_y(0) <= shift_right(act_h, 1) + shift_right(act_h, 5);  -- ~0.531 fg
                base_y(1) <= shift_right(act_h, 1);                          -- 0.5   mid
                base_y(2) <= shift_right(act_h, 1) - shift_right(act_h, 4);  -- ~0.437 bg

                sky_u <= to_unsigned(SKYT_U(to_integer(k3h(9 downto 5))), 10);
                sky_v <= to_unsigned(SKYT_V(to_integer(k3h(9 downto 5))), 10);

            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                py        <= (others => '0');
                ydacc     <= (others => (others => '0'));
            elsif v_h_edge = '1' then
                py <= py + 1;
                -- 45-degree diagonal accumulators: one add per LINE per layer;
                -- slope = (F/16) << layer, exactly the stripes' base x gradient
                -- (in 1/2^FANPOST units) -> constant 45 degrees.
                -- 21 bits = 10-bit phase + FANPOST fraction; wraps mod-1024.
                for l in 0 to 2 loop
                    ydacc(l) <= ydacc(l) + shift_left(resize(s_F, 21), FANPOST - 4 + l);
                end loop;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Main datapath.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_dr   : signed(8 downto 0);     -- displacement >> 2
        variable v_win  : integer range 0 to 2;
        variable v_sky  : std_logic;
        variable v_pys  : signed(12 downto 0);
        variable v_dd   : signed(13 downto 0);    -- depth (crest-line test)
        variable v_a14  : unsigned(13 downto 0);    -- x'*Feff + (diagonal + curve)<<4
        variable v_thu  : unsigned(13 downto 0);    -- soft-edge threshold raw
        variable v_e    : unsigned(13 downto 0);    -- depth past the lit band
        variable v_shp  : unsigned(15 downto 0);    -- y * shade product
        variable v_y    : signed(11 downto 0);
        variable v_dx   : signed(8 downto 0);       -- bush: px within slot - centre
        variable v_axu  : unsigned(8 downto 0);     -- bush: |dx| (unclamped)
        variable v_adx  : integer range 0 to 63;    -- bush: |dx| (clamped)
        variable v_dya  : signed(14 downto 0);      -- bush: anchor - py (raw)
        variable v_dyu  : unsigned(14 downto 0);    -- bush: |anchor - py| (unclamped)
        variable v_dyi  : integer range 0 to 63;    -- bush: |anchor - py| (clamped)
        variable v_dyy  : signed(14 downto 0);      -- bush: crest - py (per-column clip)
        variable v_rg   : unsigned(5 downto 0);     -- bush: 8-slot region index
        variable v_ix   : unsigned(3 downto 0);     -- bush: S-box input
        variable v_len  : integer range 0 to 7;     -- bush: group length (3 or 6)
        variable v_sis  : unsigned(2 downto 0);     -- bush: slot within region
        variable v_bsel : std_logic_vector(1 downto 0);  -- bush: winning code @ s10
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -- local copies of per-frame params (short routes to the multipliers)
            d_gainA <= s_gainA; d_F <= s_F; d_fane <= s_fane;

            -----------------------------------------------------------------
            -- s1: capture both crest sines (+ the primary's cosine, for the
            --     crest fan), stripe-x (world x).
            -----------------------------------------------------------------
            r1_par <= scan_par; r1_py <= py;
            for l in 0 to 2 loop
                r1_sinA(l) <= sinA(l);
                r1_sinB(l) <= sinB(l);
                r1_ang(l)  <= angA(l);
                r1_sx(l)   <= unsigned(resize(xv, 14)) + scrollPx(l)(20 downto 7);
            end loop;

            -----------------------------------------------------------------
            -- s2: combined crest displacement = primary + secondary/2.
            -----------------------------------------------------------------
            for l in 0 to 2 loop
                r2_disp(l) <= resize(r1_sinA(l), 12) + resize(shift_right(r1_sinB(l), 1), 12);
            end loop;
            r2_par <= r1_par; r2_py <= r1_py; r2_sx <= r1_sx; r2_ang <= r1_ang;

            -----------------------------------------------------------------
            -- s3: SMOOTH amplitude = displacement * gain (one small multiply).
            -----------------------------------------------------------------
            for l in 0 to 2 loop
                v_dr := resize(shift_right(r2_disp(l), 2), 9);          -- +/-191
                r3_scl(l) <= resize(shift_right(v_dr * signed('0' & d_gainA), 5), 13);
            end loop;
            r3_par <= r2_par; r3_py <= r2_py; r3_sx <= r2_sx; r3_ang <= r2_ang;

            -----------------------------------------------------------------
            -- s4: crest = baseline - scaled.  NOT clamped: large amplitude runs
            --     the ridge off the top/bottom of frame instead of flattening.
            -----------------------------------------------------------------
            for l in 0 to 2 loop
                r4_crest(l) <= signed(resize(base_y(l), 13)) - r3_scl(l);
            end loop;
            r4_par <= r3_par; r4_py <= r3_py; r4_sx <= r3_sx; r4_ang <= r3_ang;

            -----------------------------------------------------------------
            -- BUSH anchor: register the anchor-store read (crest under this
            -- slot's centre, from the previous scan) before any arithmetic.
            -----------------------------------------------------------------
            for L in 0 to 2 loop
                r5_anc(L) <= resize(signed(anc_rdata(L)(12 downto 0)), 14);
            end loop;

            -----------------------------------------------------------------
            -- s5a: the three crest subtracts, parallel (carry chains only).
            -----------------------------------------------------------------
            v_pys := signed(resize(r4_py, 13));
            for l in 0 to 2 loop
                r5a_dd(l) <= resize(v_pys, 14) - resize(r4_crest(l), 14);
            end loop;
            r5a_sx <= r4_sx; r5a_py <= r4_py; r5a_ang <= r4_ang;
            -- BUSH group hash (per layer): region = 8 slots.  S-box of the low
            -- 4 region bits xor the high bits (+ a per-layer constant): a
            -- permutation within every 16 regions -> density d = exactly d/16.
            for L in 0 to 2 loop
                v_rg := resize(r4_sx(L)(13 downto BSLOTLOG + 3 - L), 6);
                v_ix := v_rg(3 downto 0) xor ("00" & v_rg(5 downto 4))
                        xor to_unsigned(BXOR(L), 4);
                r5a_bh(L) <= to_unsigned(SBOX(to_integer(v_ix)), 4);
                if SBOX(to_integer(v_ix xor "1010")) >= 8 then r5a_bl(L) <= '1';
                else                                           r5a_bl(L) <= '0'; end if;
            end loop;

            -----------------------------------------------------------------
            -- s5b: pick frontmost covering layer (py at/below crest l -> dd(l)>=0,
            --     i.e. sign bit 0); sky otherwise; keep the winner's WORLD-x so the
            --     stripes scroll with that hill.  Cheap muxing, no carry chain.
            -----------------------------------------------------------------
            if r5a_dd(0)(13) = '0' then
                v_win := 0; v_sky := '0';
            elsif s_lay3 = '1' and r5a_dd(1)(13) = '0' then
                v_win := 1; v_sky := '0';
            elsif r5a_dd(2)(13) = '0' then
                v_win := 2; v_sky := '0';
            else
                v_win := 0; v_sky := '1';
            end if;
            v_dd := r5a_dd(v_win);                                 -- depth below crest
            r5_layer <= v_win;
            r5_sky   <= v_sky;
            if v_dd < to_signed(THICK, 14) then r5_cl <= '1'; else r5_cl <= '0'; end if;
            r5_sx  <= r5a_sx(v_win);
            r5_py  <= r5a_py;
            r5_fang <= r5a_ang(v_win);
            -- CREST-LIT SHADING (follows the crest curve, peaks and valleys
            -- alike): the top ~15% of frame height below the crest is fully
            -- lit (+120 luma); past the band the lift fades out over 120 px
            -- while a multiplicative shade ramps to 40/64 (37.5% brightness)
            -- at 640 px past the band.
            if v_sky = '1' then
                r5_glow  <= (others => '0');
                r5_shade <= (others => '0');
            elsif v_dd < signed(resize(s_litd, 14)) then
                r5_glow  <= to_unsigned(120, 7);
                r5_shade <= (others => '0');
            else
                v_e := unsigned(v_dd) - resize(s_litd, 14);
                if v_e >= to_unsigned(120, 14) then r5_glow <= (others => '0');
                else r5_glow <= to_unsigned(120, 7) - v_e(6 downto 0); end if;
                if v_e >= to_unsigned(640, 14) then r5_shade <= to_unsigned(40, 6);
                else r5_shade <= v_e(9 downto 4); end if;
            end if;
            -- BUSH: capture ALL layers' world-x and (py - crest) so each layer's
            -- bushes ride its own crest (dy = crest - py = -ddL, per column).
            r5_sxL <= r5a_sx;
            r5_ddL <= r5a_dd;
            -- BUSH placement: region lit by density, first 3 or 6 slots of it;
            -- the mid layer's bushes vanish with the layer (S7 off).
            for L in 0 to 2 loop
                v_sis := r5a_sx(L)(BSLOTLOG + 2 - L downto BSLOTLOG - L);
                if r5a_bl(L) = '1' then v_len := 6; else v_len := 3; end if;
                if r5a_bh(L) < s_bden and to_integer(v_sis) < v_len
                   and (L /= 1 or s_lay3 = '1') then
                    r5_bpl(L) <= '1';
                else
                    r5_bpl(L) <= '0';
                end if;
            end loop;

            -----------------------------------------------------------------
            -- s6: CREST FAN operand.  Feff = F<<layer (per-layer width).
            --     Register the shared fan-cosine LUT output (addressed by the
            --     winner's angle from s5b).  The fan WARPS world-x before the
            --     stripe multiply: x' = x + cos(winner crest angle)*fan, whose
            --     derivative 1 - k*sinA follows hill height — stripes widen over
            --     tall hills, tighten in valleys, and (being a warp) the effect
            --     scales with stripe width automatically: at K2 max the fg gx
            --     swings +-86% (~28..82 deg) at ANY width.  Still f(world-x),
            --     so the stripes stay glued to the terrain.
            -----------------------------------------------------------------
            r6_freqeff <= shift_left(resize(d_F, 11), r5_layer);
            r6_cos  <= fan_cos;
            r6_glow <= r5_glow; r6_shade <= r5_shade;
            r6_sx <= r5_sx; r6_py <= r5_py;
            r6_layer <= r5_layer; r6_sky <= r5_sky; r6_cl <= r5_cl;

            -----------------------------------------------------------------
            -- s6 (bush geometry, per layer L): slot = 128>>L, centre 64>>L, radius
            --   48>>L.  dy is measured from the per-slot ANCHOR (the exact crest
            --   under the dome centre, from the anchor store) -> a TRUE circle at
            --   any crest slope, right up to both screen edges;
            --   the per-column crest test below only CLIPS it at the crest.
            --   Placement (groups, density, S7) was resolved at s5b.
            -----------------------------------------------------------------
            for L in 0 to 2 loop
                -- dx = distance from the slot centre (centre = slot/2).
                v_dx := signed('0' & r5_sxL(L)(BSLOTLOG - 1 - L downto 0))
                        - to_signed(2 ** (BSLOTLOG - 1 - L), 9);
                if v_dx(8) = '1' then v_axu := unsigned(-v_dx);
                else                  v_axu := unsigned(v_dx); end if;
                if v_axu > 63 then v_adx := 63;                              -- incl. dx = -64
                else               v_adx := to_integer(v_axu(5 downto 0)); end if;
                r6_adx(L) <= to_unsigned(v_adx, 6);
                -- dy = anchor - py (signed: the circle continues BELOW the anchor
                -- until the per-column crest clip, so downslopes show a part-buried
                -- round rim instead of a sheared base).
                v_dya := resize(r5_anc(L), 15) - signed(resize(r5_py, 15));
                if v_dya(14) = '1' then v_dyu := unsigned(-v_dya);
                else                    v_dyu := unsigned(v_dya); end if;
                if v_dyu > 63 then v_dyi := 63;
                else               v_dyi := to_integer(v_dyu(5 downto 0)); end if;
                r6_dyi(L) <= to_unsigned(v_dyi, 6);
                -- per-column crest clip: draw only strictly above the crest.
                v_dyy := resize(-r5_ddL(L), 15);
                -- draw only above the crest (dy>=1) and where placed (stateless).
                if v_dyy >= to_signed(1, 15) then r6_bok(L) <= r5_bpl(L);
                else                              r6_bok(L) <= '0'; end if;
            end loop;

            -----------------------------------------------------------------
            -- s6b: fan gain multiply (cos * fan_eff).
            -----------------------------------------------------------------
            r6b_fan  <= r6_cos * signed('0' & d_fane);
            r6b_sx   <= r6_sx; r6b_feff <= r6_freqeff;
            r6b_glow <= r6_glow; r6b_shade <= r6_shade;
            r6b_py <= r6_py;
            r6b_layer <= r6_layer; r6b_sky <= r6_sky; r6b_cl <= r6_cl;
            -- s6b (bush): squared radial distance dx^2+dy^2 in its OWN stage (the
            --   multiplies are the tallest logic) -> compared in s7.
            for L in 0 to 2 loop
                r6b_d2(L)  <= to_unsigned(to_integer(r6_adx(L)) * to_integer(r6_adx(L))
                                        + to_integer(r6_dyi(L)) * to_integer(r6_dyi(L)), 13);
                r6b_bok(L) <= r6_bok(L);
            end loop;

            -----------------------------------------------------------------
            -- s7: fan-warped world-x  x' = x + cos*fan_eff >> FANWSH (mod 2^14).
            -----------------------------------------------------------------
            r7_sxw  <= r6b_sx + unsigned(resize(shift_right(r6b_fan, FANWSH), 14));
            r7_feff <= r6b_feff;
            r7_py <= r6b_py; r7_glow <= r6b_glow; r7_shade <= r6b_shade;
            r7_layer <= r6b_layer; r7_sky <= r6b_sky; r7_cl <= r6b_cl;
            -- s7 (bush code): inside inner radius -> fill; annulus to outer -> even
            --   dark outline; else none.
            for L in 0 to 2 loop
                if r6b_bok(L) = '1' and to_integer(r6b_d2(L)) <= BR2(L) then
                    if to_integer(r6b_d2(L)) <= BRIN2(L) then r7_bcode(L) <= "01";
                    else                                      r7_bcode(L) <= "10"; end if;
                else
                    r7_bcode(L) <= "00";
                end if;
            end loop;

            -----------------------------------------------------------------
            -- s7b: stripe base product x'*Feff mod 2^14 (only bits 13:4 = the
            --      10-bit phase survive), operands split lo/hi to shorten the
            --      carry chains; y terms (45-degree diagonal + sine curve)
            --      pre-summed so s8 is one 3-operand add.
            -----------------------------------------------------------------
            r7b_plo <= resize(r7_sxw * r7_feff(5 downto 0), 14);
            r7b_phi <= resize(r7_sxw(7 downto 0) * r7_feff(10 downto 6), 8);
            r7b_yc  <= ydacc(r7_layer)(20 downto 11)
                     + unsigned(resize(shift_right(curve_sin, CURVESH), 10));
            r7b_glow <= r7_glow; r7b_shade <= r7_shade;
            r7b_layer <= r7_layer; r7b_sky <= r7_sky; r7b_cl <= r7_cl;
            r7b_bcode <= r7_bcode;

            -----------------------------------------------------------------
            -- s8: stripe angle = (x'*Feff >> 4) + diagonal + curve  (mod 1024).
            -----------------------------------------------------------------
            v_a14 := r7b_plo + (r7b_phi & "000000") + (r7b_yc & "0000");
            r8_ang <= std_logic_vector(v_a14(13 downto 4));
            -- soft-edge threshold = 2^SOFTW px * gx (= F/16 << layer), clamped
            -- 4..128 — constant blend width in pixels at any K5/layer.
            v_thu := shift_right(shift_left(resize(d_F, 14), r7b_layer), 4 - SOFTW);
            if    v_thu > to_unsigned(128, 14) then r8_thr <= to_unsigned(128, 8);
            elsif v_thu < to_unsigned(4, 14)   then r8_thr <= to_unsigned(4, 8);
            else                                    r8_thr <= resize(v_thu, 8);
            end if;
            r8_glow <= r7b_glow; r8_shade <= r7b_shade;
            r8_layer <= r7b_layer; r8_sky <= r7b_sky; r8_cl <= r7b_cl;
            r8_bcode <= r7b_bcode;

            -----------------------------------------------------------------
            -- s9: stripe wave = triangle of the angle (sign picks colour A/B).
            -----------------------------------------------------------------
            r9_ssin <= tri10(unsigned(r8_ang));
            r9_thr  <= r8_thr;
            r9_glow <= r8_glow; r9_shade <= r8_shade;
            r9_layer <= r8_layer; r9_sky <= r8_sky; r9_cl <= r8_cl;
            r9_bcode <= r8_bcode;

            -----------------------------------------------------------------
            -- s10: assemble base colour.  A bush shows only when it is in front of
            --      the winning hill: layer L's bush needs the hill winner to be
            --      farther (sky, or a higher layer index).  Pick the frontmost such
            --      bush; sky flag cleared so it stays opaque under video-sky.
            -----------------------------------------------------------------
            if (r9_sky = '1' or r9_layer >= 1) and r9_bcode(0) /= "00" then
                v_bsel := r9_bcode(0);
            elsif (r9_sky = '1' or r9_layer >= 2) and r9_bcode(1) /= "00" then
                v_bsel := r9_bcode(1);
            elsif r9_sky = '1' and r9_bcode(2) /= "00" then
                v_bsel := r9_bcode(2);
            else
                v_bsel := "00";
            end if;
            r10_shade <= (others => '0');       -- only stripe pixels shade
            if pipe(LATENCY - 2).avid = '0' then
                -- BLANKING GATE: neutral black outside active video (aligned one
                -- stage before the output register; shade 0 passes y=64 through).
                r10_y <= C_BLK; r10_u <= C_MID; r10_v <= C_MID; r10_sky <= '0';
            elsif v_bsel = "10" then                                    -- bush outline
                r10_y <= to_unsigned(BLINE_Y, 10); r10_u <= to_unsigned(BLINE_U, 10);
                r10_v <= to_unsigned(BLINE_V, 10); r10_sky <= '0';
            elsif v_bsel = "01" then                                 -- bush fill
                r10_y <= to_unsigned(BFILL_Y, 10); r10_u <= to_unsigned(BFILL_U, 10);
                r10_v <= to_unsigned(BFILL_V, 10); r10_sky <= '0';
            elsif r9_sky = '1' then
                r10_y <= sky_ly; r10_u <= sky_u; r10_v <= sky_v; r10_sky <= '1';
            elsif r9_cl = '1' and s_creston = '1' then
                r10_y <= gC_y(r9_layer); r10_u <= gC_u(r9_layer); r10_v <= gC_v(r9_layer); r10_sky <= '0';
            elsif r9_ssin > signed(resize(r9_thr, 10)) then
                r10_y <= gB_y(r9_layer) + resize(r9_glow, 10);
                r10_u <= gB_u(r9_layer); r10_v <= gB_v(r9_layer); r10_sky <= '0';
                r10_shade <= r9_shade;
            elsif r9_ssin < -signed(resize(r9_thr, 10)) then
                r10_y <= gA_y(r9_layer) + resize(r9_glow, 10);
                r10_u <= gA_u(r9_layer); r10_v <= gA_v(r9_layer); r10_sky <= '0';
                r10_shade <= r9_shade;
            else
                r10_y <= gM_y(r9_layer) + resize(r9_glow, 10);
                r10_u <= gM_u(r9_layer); r10_v <= gM_v(r9_layer); r10_sky <= '0';
                r10_shade <= r9_shade;
            end if;

            -----------------------------------------------------------------
            -- s11: crest-lit shading: y' = y - y*shade/64 (shade <= 40 ->
            --      floor at 37.5% brightness; no underflow, no clamp needed).
            --      In Video Sky mode, pass the input through on the sky.
            -----------------------------------------------------------------
            if r10_sky = '1' and s_vidsky = '1' then
                s_io.y <= pipe(LATENCY - 1).y;
                s_io.u <= pipe(LATENCY - 1).u;
                s_io.v <= pipe(LATENCY - 1).v;
            else
                v_shp := r10_y * r10_shade;
                s_io.y <= std_logic_vector(r10_y - resize(v_shp(15 downto 6), 10));
                s_io.u <= std_logic_vector(r10_u);
                s_io.v <= std_logic_vector(r10_v);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture candyhills;
