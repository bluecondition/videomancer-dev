-- candyhills.vhd  (v0.5 — "Candy Hills")
--
-- A real-time, side-scrolling 2D generative landscape: large, long, sweeping
-- rolling candy hills filled with bold angled stripes that fan wider toward the
-- crest, under a pastel sky, with a soft cloudy paper-grain wash over the frame.
-- The view scrolls continuously leftward and never repeats.
--
-- Streaming synthesis program (no frame buffer): only the input SYNC is used.
--
-- HILLS
--   Each of 3 parallax layers has a crest = baseline - amplitude*(sinA + sinB/2)
--   of a VERY LONG primary sine (~5000/6800/9500 px = several screens) plus a
--   shorter secondary sine, so the ridge sometimes climbs or falls across
--   multiple screens.  Amplitude is a SMOOTH per-frame gain (one small multiply
--   per layer), and the crest is NOT clamped to a band: at high amplitude it
--   simply runs off the top or bottom of frame (full hill / full sky) instead of
--   flattening.  Waves are evaluated multiply-free with per-pixel phase
--   accumulators; back layers scroll fractionally slower (parallax).
--
-- STRIPES
--   Colour comes from a stripe wave whose phase = world-x (scrolls), plus a
--   straight diagonal lean (py>>TILT) and a perspective FAN.  The fan = (px from
--   screen centre)*depth*FanAmount, done in screen space so it is bounded and
--   seamless: it raises the local stripe frequency with depth, so bands are
--   narrow at a hill's bottom and widen / bow toward the crest.  Stripe WIDTH is
--   a continuous frequency (one small multiply) with a large range.
--
-- Fixed colours stored U/V (Cb/Cr) SWAPPED for the Videomancer HW convention
-- (stored U = Cr, V = Cb), matching the mondrian / glitchscape palettes.
--
-- Controls:
--   K1 Scroll Speed     K2 Hill Amplitude (smooth)   K3 Sky Tint
--   K4 Palette          K5 Stripe Width (wide range) K6 Texture Amount
--   S7 3rd Layer  S8 Distance Haze  S9 Crest Line(def off)  S10 Texture  S11 Freeze
--   P12 Fan Amount
--
-- HOOKS for later (not built): bushes / props between colour assembly and the
-- texture wash for the foreground layer near the crest.
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

    constant LATENCY : natural := 13;         -- datapath stages s1..s11 (+ s5a, s6b)

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant SOFT   : integer := 120;         -- soft-edge half-band of stripe wave
    -- Stripes are attached to the TERRAIN (phase = world-x * freq), so they move
    -- with each hill at that layer's scroll rate.  Per-layer width is 100/50/25 %
    -- (freq << layer).  PERSPECTIVE FAN (Fan Amount, P12): the local frequency
    -- rises toward the bottom of the frame (fanadd grows with py) so stripes are
    -- narrow at the bottom and fan wider toward the crest.  The fan divide is
    -- SPLIT: only FANPRE bits dropped BEFORE the sx multiply (so the frequency
    -- stays fractional and the width changes SMOOTHLY per row, no stair-steps),
    -- FANPOST bits after; FANPRE+FANPOST sets the strength.
    -- A gentle sine curve (tilt + curve of py) bends the stripes.
    constant FANPRE  : natural := 4;          -- shift on (py-PYREF)*fan before sx*
    constant FANPOST : natural := 11;         -- shift after the sx multiply
    constant TILTF  : natural := 2;           -- fixed diagonal lean: + py >> TILTF
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
    -- Per-layer scroll velocity in 1/16-px per frame (fractional; bg creeps
    -- slower than fg).  K1 scales all by 2^spd.
    constant VEL16    : t_l3int := (32, 16, 6);
    -- Per-frame crest phase scroll = VEL16 * STEP / 16.
    constant SCROLL_A : t_l3int := (420, 154, 41);
    constant SCROLL_B : t_l3int := (1398, 524, 140);
    -- Haze shift toward HAZE colour per layer (0 = none).  bg most hazed.
    constant HZ       : t_l3int := (0, 2, 1);

    -- Stripe-width table (geometric so the K5 knob spreads widths evenly across
    -- its whole travel).  Index 0..7 from K5[9:7]; low K5 = narrow, high = wide.
    type t_freq8 is array(0 to 7) of integer;
    constant FREQTBL : t_freq8 := (16, 12, 9, 7, 5, 4, 3, 2);

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

    constant CREST_Y : integer := 130;   -- (legacy) old fixed crest line — now derived
    constant CREST_U : integer := 492;
    constant CREST_V : integer := 488;

    -- BUSHES: light-green semicircles outlined in dark green, growing straight out
    -- of a crest (fg/mid/bg) at 100/50/25 % size, in sparse RANDOM GROUPS of 3 or 6,
    -- preferentially on FLAT parts of the crest.  The base follows the crest
    -- PER-COLUMN so the bush is flush with the hill (no sky between them); the dome
    -- is a true circle test dx^2 + dy^2 <= R^2 with an EVEN radial outline in the
    -- annulus (R-T)^2 .. R^2.  Fixed grassy greens (U = Cr, V = Cb swapped); fill
    -- textured (watercolour wash), outline dark.  Per layer L: slot = 128>>L px
    -- (log 7-L), radius 48>>L, centre 64>>L.  No LUT / no per-frame state.
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
    constant SKY_Y0  : integer := 840;   -- base pastel mint sky
    constant SKY_U0  : integer := 466;
    constant SKY_V0  : integer := 488;

    function clamp10(v : signed) return unsigned is
    begin
        if v < 0 then       return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else                return unsigned(resize(v, 10));
        end if;
    end function;

    -- Haze one channel toward h by 1/2^sh (sh in {0,1,2}); sh=0 = no haze.
    function haze1(c, h, sh : integer) return integer is
    begin
        case sh is
            when 1      => return c + (h - c) / 2;
            when 2      => return c + (h - c) / 4;
            when others => return c;
        end case;
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
    type t_l3u18  is array(0 to 2) of unsigned(17 downto 0);
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

    signal px        : unsigned(11 downto 0) := (others => '0');
    signal py        : unsigned(10 downto 0) := (others => '0');
    signal frame_act : std_logic := '0';
    signal frame_cnt : unsigned(9 downto 0) := (others => '0');

    signal act_h     : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal act_cx    : unsigned(11 downto 0) := to_unsigned(640, 12);  -- active width/2
    signal max_px    : unsigned(11 downto 0) := (others => '0');
    signal max_py    : unsigned(10 downto 0) := (others => '0');

    signal lfsr      : unsigned(15 downto 0) := x"ACE1";

    -- Per-pixel crest phase accumulators (reset to per-frame offset at line start).
    signal phA       : t_l3u20 := (others => (others => '0'));
    signal phB       : t_l3u20 := (others => (others => '0'));
    signal offA      : t_l3u20 := (others => (others => '0'));
    signal offB      : t_l3u20 := (others => (others => '0'));
    signal scrollPx  : t_l3u18 := (others => (others => '0'));  -- 1/16-px units

    --------------------------------------------------------------------------
    -- Per-frame latched parameters / derived values.
    --------------------------------------------------------------------------
    signal s_spd     : integer range 0 to 3 := 1;     -- K1 scroll speed
    signal s_gainA   : unsigned(5 downto 0) := to_unsigned(35, 6);   -- K2 amplitude (0..63)
    signal s_freq    : unsigned(5 downto 0) := to_unsigned(3, 6);    -- K5 stripe freq (1..32)
    signal s_fan     : unsigned(5 downto 0) := to_unsigned(32, 6);  -- P12 fan amount (0..63)
    signal s_palsel  : integer range 0 to 7 := 0;     -- K4 palette (8)
    signal s_texsh   : integer range 1 to 4 := 3;    -- K6 texture amount (cloud >> texsh)

    signal s_lay3    : std_logic := '1';   -- S7
    signal s_haze    : std_logic := '1';   -- S8
    signal s_creston : std_logic := '0';   -- S9 (default off)
    signal s_vidsky  : std_logic := '0';   -- S10 (replace sky with input video)
    signal s_freeze  : std_logic := '0';   -- S11

    -- Per-layer baselines near vertical centre (parallax-offset).
    signal base_y    : t_l3u11 := (others => to_unsigned(360, 11));
    signal s_pyref   : unsigned(10 downto 0) := to_unsigned(540, 11);  -- fan crossover row (~0.75*act_h)

    -- Selected base palette colours, registered every clock (pipelines the 8:1
    -- palette mux out of the per-frame haze path so it does not gate timing).
    signal sel_ay, sel_au, sel_av : integer range 0 to 1023 := 512;
    signal sel_by, sel_bu, sel_bv : integer range 0 to 1023 := 512;
    -- Crest base colour = the DARKER stripe colour, darkened ~0.7x (its hue kept).
    -- Resolved every clock so the compare+divide stays OFF the per-frame gC path.
    signal sel_cy, sel_cu, sel_cv : integer range 0 to 1023 := 256;

    -- Per-frame hazed stripe colours per layer.
    signal gA_y, gA_u, gA_v : t_l3u10 := (others => C_MID);
    signal gB_y, gB_u, gB_v : t_l3u10 := (others => C_MID);
    signal gM_y, gM_u, gM_v : t_l3u10 := (others => C_MID);
    signal gC_y, gC_u, gC_v : t_l3u10 := (others => C_MID);
    signal sky_y, sky_u, sky_v : unsigned(9 downto 0) := C_MID;

    --------------------------------------------------------------------------
    -- Crest sine LUTs (primary + secondary per layer).
    --------------------------------------------------------------------------
    signal angA, angB : t_l3slv;
    signal sinA, sinB : t_l3s10;

    -- Stripe sine-curve LUT (bends the stripes like the crest).
    signal curve_ang : std_logic_vector(9 downto 0) := (others => '0');
    signal curve_sin : signed(9 downto 0);

    -- Datapath-local copies of per-frame params, re-registered every clock so
    -- placement keeps them next to the multipliers (avoids long routes from the
    -- config/SPI region into the pixel pipeline).
    signal d_gainA : unsigned(5 downto 0) := to_unsigned(35, 6);
    signal d_freq  : unsigned(5 downto 0) := to_unsigned(3, 6);
    signal d_fan   : unsigned(5 downto 0) := to_unsigned(32, 6);

    --------------------------------------------------------------------------
    -- Datapath stage registers.
    --------------------------------------------------------------------------
    -- s1: capture position, both crest sines, stripe-x, texture
    signal r1_px   : unsigned(11 downto 0) := (others => '0');
    signal r1_py   : unsigned(10 downto 0) := (others => '0');
    signal r1_sinA, r1_sinB : t_l3s10 := (others => (others => '0'));
    signal r1_sx   : t_l3u14 := (others => (others => '0'));
    signal r1_cloud : signed(8 downto 0) := (others => '0');   -- raw watercolour cloud
    -- s2: combined displacement = sinA + sinB/2
    signal r2_disp : t_l3s12 := (others => (others => '0'));
    signal r2_px   : unsigned(11 downto 0) := (others => '0');
    signal r2_py   : unsigned(10 downto 0) := (others => '0');
    signal r2_sx   : t_l3u14 := (others => (others => '0'));
    signal r2_tex  : signed(8 downto 0) := (others => '0');
    -- s3: amplitude-scaled displacement (smooth gain multiply)
    signal r3_scl  : t_l3s13 := (others => (others => '0'));
    signal r3_px   : unsigned(11 downto 0) := (others => '0');
    signal r3_py   : unsigned(10 downto 0) := (others => '0');
    signal r3_sx   : t_l3u14 := (others => (others => '0'));
    signal r3_tex  : signed(8 downto 0) := (others => '0');
    -- s4: crest = baseline - scaled displacement (signed, may run off-screen)
    signal r4_crest : t_l3s13 := (others => (others => '0'));
    signal r4_px   : unsigned(11 downto 0) := (others => '0');
    signal r4_py   : unsigned(10 downto 0) := (others => '0');
    signal r4_sx   : t_l3u14 := (others => (others => '0'));
    signal r4_tex  : signed(8 downto 0) := (others => '0');
    -- s5: frontmost covering layer / sky / crest-line / winner's world-x
    -- s5a: the three crest subtracts (carry chains), split off from the priority
    --      select so the compares route alone.  dd(l) = py - crest(l); sign = the
    --      "py below crest l" test; the winner's dd is the crest-line depth.
    signal r5a_dd  : t_l3s14 := (others => (others => '0'));   -- 14b: py-crest, no overflow
    signal r5a_sx  : t_l3u14 := (others => (others => '0'));
    signal r5a_py  : unsigned(10 downto 0) := (others => '0');
    signal r5a_tex : signed(8 downto 0) := (others => '0');

    signal r5_layer : integer range 0 to 2 := 0;
    signal r5_sky  : std_logic := '0';
    signal r5_cl   : std_logic := '0';
    signal r5_sx   : unsigned(13 downto 0) := (others => '0');  -- world-x of winner
    signal r5_py   : unsigned(10 downto 0) := (others => '0');
    signal r5_pyc  : signed(11 downto 0) := (others => '0');    -- py - PYREF (fan)
    signal r5_tex  : signed(8 downto 0) := (others => '0');
    -- s6: base frequency (freq<<layer) + fan add ((py-PYREF)*fan>>FANPRE), split so
    --     the phase needs only small multiplies.
    signal r6_layer : integer range 0 to 2 := 0;
    signal r6_sky  : std_logic := '0';
    signal r6_cl   : std_logic := '0';
    signal r6_freqeff : unsigned(7 downto 0) := (others => '0');   -- freq << layer
    signal r6_fanadd  : signed(13 downto 0) := (others => '0');    -- (py-PYREF)*fan >> FANPRE (fractional)
    signal r6_sx   : unsigned(13 downto 0) := (others => '0');
    signal r6_py   : unsigned(10 downto 0) := (others => '0');
    signal r6_tex  : signed(8 downto 0) := (others => '0');
    -- s6b: raw multiplies.  base = sx(9:0)*freq_eff (small).  The fan multiply
    --      sx*fanadd (15x14) is the HD critical path, so it is SPLIT into two
    --      half-width partial products against fanadd's high/low 7-bit halves —
    --      each a short 15x7/15x8 multiply — then summed in s7 (which had slack).
    --      This shortens the carry chain that was limiting Fmax.  Only phase(9:0)
    --      survives downstream, so the fan phase is mod-1024.
    signal r6b_base : unsigned(17 downto 0) := (others => '0');
    signal r6b_phi : signed(21 downto 0) := (others => '0');   -- sx * fanadd(13:7)
    signal r6b_plo : signed(22 downto 0) := (others => '0');   -- sx * fanadd(6:0)
    signal r6b_layer : integer range 0 to 2 := 0;
    signal r6b_sky : std_logic := '0';
    signal r6b_cl  : std_logic := '0';
    signal r6b_py  : unsigned(10 downto 0) := (others => '0');
    signal r6b_tex : signed(8 downto 0) := (others => '0');
    -- s7: stripe phase = base + fanraw>>FANPOST
    signal r7_layer : integer range 0 to 2 := 0;
    signal r7_sky  : std_logic := '0';
    signal r7_cl   : std_logic := '0';
    signal r7_phase : unsigned(13 downto 0) := (others => '0');
    signal r7_py   : unsigned(10 downto 0) := (others => '0');
    signal r7_tex  : signed(8 downto 0) := (others => '0');
    -- s8: stripe angle = phase + tilt(py) + sine curve(py)
    signal r8_layer : integer range 0 to 2 := 0;
    signal r8_sky  : std_logic := '0';
    signal r8_cl   : std_logic := '0';
    signal r8_ang  : std_logic_vector(9 downto 0) := (others => '0');
    signal r8_tex  : signed(8 downto 0) := (others => '0');
    -- s9: stripe wave sampled
    signal r9_layer : integer range 0 to 2 := 0;
    signal r9_sky  : std_logic := '0';
    signal r9_cl   : std_logic := '0';
    signal r9_ssin : signed(9 downto 0) := (others => '0');
    signal r9_tex  : signed(8 downto 0) := (others => '0');
    -- s10: assembled base colour (+ sky flag for the video-sky option)
    signal r10_y, r10_u, r10_v : unsigned(9 downto 0) := C_MID;
    signal r10_sky : std_logic := '0';
    signal r10_tex : signed(8 downto 0) := (others => '0');

    -- BUSH sub-pipeline (parallel to the colour path; overrides colour at s10).
    --   Fully STATELESS (no held anchor / no per-slot memory) so bushes scroll
    --   cleanly off both edges with no popping.  s5b captures all-layer world-x /
    --   (py-crest); s6 does per-layer geometry (squared radial distance from the
    --   dome centre + group placement); s6b resolves a per-layer 2-bit code
    --   (00 none / 01 fill / 10 outline); s7..s9 carry; s10 picks the frontmost
    --   bush in front of the winning hill.
    signal r5_sxL   : t_l3u14 := (others => (others => '0'));    -- world-x per layer
    signal r5_ddL   : t_l3s14 := (others => (others => '0'));    -- py - crest per layer
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

    --------------------------------------------------------------------------
    -- Raster position, phase accumulators, per-frame parameter latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_tex : integer range 0 to 3;
        variable v_kw  : integer range 0 to 31;
        variable v_ay, v_au, v_av : integer;
        variable v_by, v_bu, v_bv : integer;
        variable v_my, v_mu, v_mv : integer;
        variable v_sh : integer range 0 to 2;
        variable v_tint : integer;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

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

            -- X counter + crest phase accumulators.
            if v_h_edge = '1' then
                if px > max_px then max_px <= px; end if;
                px <= (others => '0');
                for l in 0 to 2 loop
                    phA(l) <= offA(l);
                    phB(l) <= offB(l);
                end loop;
            elsif data_in.avid = '1' then
                px <= px + 1;
                for l in 0 to 2 loop
                    phA(l) <= phA(l) + to_unsigned(STEP_A(l), 20);
                    phB(l) <= phB(l) + to_unsigned(STEP_B(l), 20);
                end loop;
            end if;

            -- Y counter + per-frame work.
            if v_v_edge = '1' then
                py        <= (others => '0');
                frame_act <= '0';
                frame_cnt <= frame_cnt + 1;
                act_h  <= py;                          -- py at vsync = active height
                act_cx <= '0' & max_px(11 downto 1);   -- active width / 2 (fan centre)
                max_px <= (others => '0');
                max_py <= (others => '0');

                ---------------------------------------------------------------
                -- Advance scroll (unless frozen).
                ---------------------------------------------------------------
                if s_freeze = '0' then
                    for l in 0 to 2 loop
                        offA(l) <= offA(l)
                            + resize(shift_left(to_unsigned(SCROLL_A(l), 24), s_spd), 20);
                        offB(l) <= offB(l)
                            + resize(shift_left(to_unsigned(SCROLL_B(l), 24), s_spd), 20);
                        scrollPx(l) <= scrollPx(l)
                            + resize(shift_left(to_unsigned(VEL16(l), 18), s_spd), 18);
                    end loop;
                end if;

                ---------------------------------------------------------------
                -- Parameter latch.
                ---------------------------------------------------------------
                s_spd   <= to_integer(unsigned(registers_in(0)(9 downto 8)));   -- K1
                s_gainA <= unsigned(registers_in(1)(9 downto 4));               -- K2 0..63 (smooth)
                v_tint  := (to_integer(unsigned(registers_in(2)(9 downto 0))) - 512) / 8;  -- K3 sky tint
                s_palsel <= to_integer(unsigned(registers_in(3)(9 downto 7)));  -- K4 (8 palettes)
                v_kw    := to_integer(unsigned(registers_in(4)(9 downto 7)));   -- K5 0..7
                s_freq  <= to_unsigned(FREQTBL(v_kw), 6);                       -- geometric width, full-knob range
                v_tex   := to_integer(unsigned(registers_in(5)(9 downto 8)));   -- K6
                s_texsh <= 4 - v_tex;                                          -- 4(subtle)..1(strong)

                s_lay3    <= registers_in(6)(0);   -- S7
                s_haze    <= registers_in(6)(1);   -- S8
                s_creston <= registers_in(6)(2);   -- S9
                s_vidsky  <= registers_in(6)(3);   -- S10 video sky
                s_freeze  <= registers_in(6)(4);   -- S11

                s_fan   <= unsigned(registers_in(7)(9 downto 4));   -- P12 fan amount 0..63

                ---------------------------------------------------------------
                -- Per-layer baselines near vertical centre (parallax-offset).
                ---------------------------------------------------------------
                base_y(0) <= shift_right(act_h, 1) + shift_right(act_h, 5);  -- ~0.531 fg
                base_y(1) <= shift_right(act_h, 1);                          -- 0.5   mid
                base_y(2) <= shift_right(act_h, 1) - shift_right(act_h, 4);  -- ~0.437 bg
                s_pyref   <= act_h - shift_right(act_h, 2);                  -- ~0.75*act_h fan crossover

                ---------------------------------------------------------------
                -- Hazed stripe colours per layer for the selected palette.
                ---------------------------------------------------------------
                v_ay := sel_ay; v_au := sel_au; v_av := sel_av;
                v_by := sel_by; v_bu := sel_bu; v_bv := sel_bv;
                v_my := (v_ay + v_by) / 2; v_mu := (v_au + v_bu) / 2; v_mv := (v_av + v_bv) / 2;
                for l in 0 to 2 loop
                    if s_haze = '1' then v_sh := HZ(l); else v_sh := 0; end if;
                    gA_y(l) <= to_unsigned(haze1(v_ay, HAZE_Y, v_sh), 10);
                    gA_u(l) <= to_unsigned(haze1(v_au, HAZE_U, v_sh), 10);
                    gA_v(l) <= to_unsigned(haze1(v_av, HAZE_V, v_sh), 10);
                    gB_y(l) <= to_unsigned(haze1(v_by, HAZE_Y, v_sh), 10);
                    gB_u(l) <= to_unsigned(haze1(v_bu, HAZE_U, v_sh), 10);
                    gB_v(l) <= to_unsigned(haze1(v_bv, HAZE_V, v_sh), 10);
                    gM_y(l) <= to_unsigned(haze1(v_my, HAZE_Y, v_sh), 10);
                    gM_u(l) <= to_unsigned(haze1(v_mu, HAZE_U, v_sh), 10);
                    gM_v(l) <= to_unsigned(haze1(v_mv, HAZE_V, v_sh), 10);
                    gC_y(l) <= to_unsigned(haze1(sel_cy, HAZE_Y, v_sh), 10);
                    gC_u(l) <= to_unsigned(haze1(sel_cu, HAZE_U, v_sh), 10);
                    gC_v(l) <= to_unsigned(haze1(sel_cv, HAZE_V, v_sh), 10);
                end loop;

                sky_y <= to_unsigned(SKY_Y0, 10);
                sky_u <= clamp10(to_signed(SKY_U0 + v_tint, 12));
                sky_v <= clamp10(to_signed(SKY_V0 - v_tint, 12));

            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                py        <= (others => '0');
            elsif v_h_edge = '1' then
                py <= py + 1;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Main datapath.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_a, v_b : unsigned(7 downto 0);
        variable v_cloud : signed(8 downto 0);
        variable v_grain : signed(8 downto 0);
        variable v_dr   : signed(8 downto 0);     -- displacement >> 2
        variable v_win  : integer range 0 to 2;
        variable v_sky  : std_logic;
        variable v_pys  : signed(12 downto 0);
        variable v_dd   : signed(13 downto 0);    -- depth (crest-line test)
        variable v_ang  : signed(12 downto 0);      -- phase + tilt + curve
        variable v_y    : signed(11 downto 0);
        variable v_dx   : signed(8 downto 0);       -- bush: px within slot - centre
        variable v_adx  : integer range 0 to 63;    -- bush: |dx| (clamped)
        variable v_dyi  : integer range 0 to 63;    -- bush: dy = crest - py (clamped)
        variable v_dyy  : signed(14 downto 0);      -- bush: crest - py (raw)
        variable v_sl   : unsigned(8 downto 0);     -- bush: slot index
        variable v_gh   : unsigned(5 downto 0);     -- bush: group hash
        variable v_len  : integer range 0 to 7;     -- bush: group length (3 or 6)
        variable v_place: std_logic;                -- bush: group placement
        variable v_bsel : std_logic_vector(1 downto 0);  -- bush: winning code @ s10
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -- local copies of per-frame params (short routes to the multipliers)
            d_gainA <= s_gainA; d_freq <= s_freq; d_fan <= s_fan;

            -----------------------------------------------------------------
            -- s1: capture both crest sines, stripe-x (world x), texture.
            -----------------------------------------------------------------
            r1_px <= px; r1_py <= py;
            for l in 0 to 2 loop
                r1_sinA(l) <= sinA(l);
                r1_sinB(l) <= sinB(l);
                r1_sx(l)   <= resize(px, 14) + scrollPx(l)(17 downto 4);
            end loop;

            -- Watercolour cloud: sum of two slow folded-triangle "octaves" of
            -- px/py (drifting with the frame) -> a soft mottled low-frequency
            -- gradient, plus a tiny bit of grain for tooth.  Always on; K6 (via
            -- s_texsh) scales the amount.
            v_a := resize(px(9 downto 4), 8) + resize(py(9 downto 5), 8)
                 + resize(frame_cnt(7 downto 1), 8);          -- large slow blotches
            if v_a(7) = '1' then v_a := not v_a; end if;      -- fold 0..127
            v_b := resize(px(8 downto 3), 8) - resize(py(8 downto 4), 8)
                 + resize(frame_cnt(6 downto 0), 8);          -- medium blotches
            if v_b(7) = '1' then v_b := not v_b; end if;
            -- two octaves centred, smooth: ~ +/-96
            v_cloud := resize(signed('0' & v_a(6 downto 1)) - 32, 9)
                     + resize(signed('0' & v_b(6 downto 2)) - 16, 9);
            r1_cloud <= v_cloud;   -- shift + grain applied next stage (s2)

            -----------------------------------------------------------------
            -- s2: combined crest displacement = primary + secondary/2.
            -----------------------------------------------------------------
            for l in 0 to 2 loop
                r2_disp(l) <= resize(r1_sinA(l), 12) + resize(shift_right(r1_sinB(l), 1), 12);
            end loop;
            -- finish the texture: K6 amount shift + a little grain (lfsr is free-run)
            v_grain := signed(resize(lfsr(2 downto 0), 9)) - 3;
            r2_tex <= resize(shift_right(r1_cloud, s_texsh), 9) + v_grain;
            r2_px <= r1_px; r2_py <= r1_py; r2_sx <= r1_sx;

            -----------------------------------------------------------------
            -- s3: SMOOTH amplitude = displacement * gain (one small multiply).
            -----------------------------------------------------------------
            for l in 0 to 2 loop
                v_dr := resize(shift_right(r2_disp(l), 2), 9);          -- +/-191
                r3_scl(l) <= resize(shift_right(v_dr * signed('0' & d_gainA), 4), 13);
            end loop;
            r3_px <= r2_px; r3_py <= r2_py; r3_sx <= r2_sx; r3_tex <= r2_tex;

            -----------------------------------------------------------------
            -- s4: crest = baseline - scaled.  NOT clamped: large amplitude runs
            --     the ridge off the top/bottom of frame instead of flattening.
            -----------------------------------------------------------------
            for l in 0 to 2 loop
                r4_crest(l) <= signed(resize(base_y(l), 13)) - r3_scl(l);
            end loop;
            r4_px <= r3_px; r4_py <= r3_py; r4_sx <= r3_sx; r4_tex <= r3_tex;

            -----------------------------------------------------------------
            -- s5a: the three crest subtracts, parallel (carry chains only).
            -----------------------------------------------------------------
            v_pys := signed(resize(r4_py, 13));
            for l in 0 to 2 loop
                r5a_dd(l) <= resize(v_pys, 14) - resize(r4_crest(l), 14);
            end loop;
            r5a_sx <= r4_sx; r5a_py <= r4_py; r5a_tex <= r4_tex;

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
            v_dd := r5a_dd(v_win);                                 -- depth (crest line only)
            r5_layer <= v_win;
            r5_sky   <= v_sky;
            if v_dd < to_signed(THICK, 14) then r5_cl <= '1'; else r5_cl <= '0'; end if;
            r5_sx  <= r5a_sx(v_win);
            r5_py  <= r5a_py;
            r5_pyc <= signed(resize(r5a_py, 12)) - signed(resize(s_pyref, 12));
            r5_tex <= r5a_tex;
            -- BUSH: capture ALL layers' world-x and (py - crest) so each layer's
            -- bushes ride its own crest (dy = crest - py = -ddL, per column).
            r5_sxL <= r5a_sx;
            r5_ddL <= r5a_dd;

            -----------------------------------------------------------------
            -- s6: PERSPECTIVE FAN operands.  freq_eff = freq<<layer (per-layer
            --     width).  fanadd is CENTRED on PYREF (~0.75*act_h): negative
            --     above it (toward the crest -> lower freq -> WIDER) and positive
            --     below (toward the bottom -> higher freq -> narrower).  Fan
            --     Amount (P12) scales it gently.  Clamped so total freq stays > 0.
            --     Per-scanline py is constant -> no shear; fanadd integer -> seamless.
            -----------------------------------------------------------------
            r6_freqeff <= resize(shift_left(d_freq, r5_layer), 8);
            r6_fanadd  <= resize(shift_right(r5_pyc * signed('0' & d_fan), FANPRE), 14);
            r6_sx <= r5_sx; r6_py <= r5_py;
            r6_layer <= r5_layer; r6_sky <= r5_sky; r6_cl <= r5_cl; r6_tex <= r5_tex;

            -----------------------------------------------------------------
            -- s6 (bush geometry, per layer L): slot = 128>>L, centre 64>>L, radius
            --   48>>L.  CENTRE ANCHOR: sample the crest at the slot's left edge,
            --   extrapolate half a slot forward using the last slot's slope
            --   (delta/2) to estimate the crest UNDER THE CENTRE, hold it across
            --   the slot -> the flat base sits on the crest and the dome is round
            --   at any amplitude.  Placement: sparse random groups of 3 or 6 slots.
            -----------------------------------------------------------------
            for L in 0 to 2 loop
                v_sl := resize(r5_sxL(L)(13 downto BSLOTLOG - L), 9);        -- slot index
                -- dx = distance from the slot centre (centre = slot/2).
                v_dx := signed('0' & r5_sxL(L)(BSLOTLOG - 1 - L downto 0))
                        - to_signed(2 ** (BSLOTLOG - 1 - L), 9);
                if v_dx(8) = '1' then v_adx := to_integer(unsigned(-v_dx));
                else                  v_adx := to_integer(unsigned(v_dx(5 downto 0))); end if;
                if v_adx > 63 then v_adx := 63; end if;
                r6_adx(L) <= to_unsigned(v_adx, 6);
                -- dy = crest - py = -ddL (per column -> base flush with the crest).
                v_dyy := resize(-r5_ddL(L), 15);
                if    v_dyy < to_signed(1, 15)  then v_dyi := 0;             -- on/below crest
                elsif v_dyy > to_signed(63, 15) then v_dyi := 63;
                else                                 v_dyi := to_integer(v_dyy(5 downto 0)); end if;
                r6_dyi(L) <= to_unsigned(v_dyi, 6);
                -- placement: sparse random groups of 3 or 6 within 8-slot regions.
                v_gh  := v_sl(8 downto 3) xor ('0' & v_sl(8 downto 4)) xor ("000" & v_sl(8 downto 6));
                if v_gh(2) = '1' then v_len := 6; else v_len := 3; end if;
                if v_gh(1 downto 0) = "10" and to_integer(v_sl(2 downto 0)) < v_len then
                    v_place := '1';
                else
                    v_place := '0';
                end if;
                -- draw only above the crest (dy>=1) and where placed (stateless).
                if v_dyy >= to_signed(1, 15) then r6_bok(L) <= v_place;
                else                              r6_bok(L) <= '0'; end if;
            end loop;

            -----------------------------------------------------------------
            -- s6b: base multiply + the two fan partial products (short carry
            --      chains).  fanadd = fanadd(13:7)*128 + fanadd(6:0), so
            --      sx*fanadd = phi*128 + plo (summed in s7).
            -----------------------------------------------------------------
            r6b_base <= r6_sx(9 downto 0) * r6_freqeff;               -- seamless base
            r6b_phi  <= signed(resize(r6_sx, 15)) * r6_fanadd(13 downto 7);
            r6b_plo  <= signed(resize(r6_sx, 15)) * signed(resize(unsigned(r6_fanadd(6 downto 0)), 8));
            r6b_py <= r6_py;
            r6b_layer <= r6_layer; r6b_sky <= r6_sky; r6b_cl <= r6_cl; r6b_tex <= r6_tex;
            -- s6b (bush): squared radial distance dx^2+dy^2 in its OWN stage (the
            --   multiplies are the tallest logic) -> compared in s7.
            for L in 0 to 2 loop
                r6b_d2(L)  <= to_unsigned(to_integer(r6_adx(L)) * to_integer(r6_adx(L))
                                        + to_integer(r6_dyi(L)) * to_integer(r6_dyi(L)), 13);
                r6b_bok(L) <= r6_bok(L);
            end loop;

            -----------------------------------------------------------------
            -- s7: fanraw = phi*128 + plo ; phase = base + fanraw>>FANPOST
            --     (fan signed: - widens, + narrows).
            -----------------------------------------------------------------
            r7_phase <= unsigned(resize(signed(resize(r6b_base, 22))
                        + resize(shift_right(resize(shift_left(resize(r6b_phi, 30), 7), 30)
                                             + resize(r6b_plo, 30), FANPOST), 22), 14));
            r7_py <= r6b_py;
            r7_layer <= r6b_layer; r7_sky <= r6b_sky; r7_cl <= r6b_cl; r7_tex <= r6b_tex;
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
            -- s8: stripe angle = phase + diagonal tilt(py>>TILTF) + sine curve.
            -----------------------------------------------------------------
            v_ang := signed(resize(r7_phase(9 downto 0), 13))
                   + resize(shift_right(signed(resize(r7_py, 12)), TILTF), 13)
                   + resize(shift_right(curve_sin, CURVESH), 13);
            r8_ang <= std_logic_vector(v_ang(9 downto 0));
            r8_layer <= r7_layer; r8_sky <= r7_sky; r8_cl <= r7_cl; r8_tex <= r7_tex;
            r8_bcode <= r7_bcode;

            -----------------------------------------------------------------
            -- s9: stripe wave = triangle of the angle (sign picks colour A/B).
            -----------------------------------------------------------------
            r9_ssin <= tri10(unsigned(r8_ang));
            r9_layer <= r8_layer; r9_sky <= r8_sky; r9_cl <= r8_cl; r9_tex <= r8_tex;
            r9_bcode <= r8_bcode;

            -----------------------------------------------------------------
            -- s10: assemble base colour.  A bush shows only when it is in front of
            --      the winning hill: layer L's bush needs the hill winner to be
            --      farther (sky, or a higher layer index).  Pick the frontmost such
            --      bush; sky flag cleared so it stays opaque (video-sky) + textured.
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
            if v_bsel = "10" then                                    -- bush outline
                r10_y <= to_unsigned(BLINE_Y, 10); r10_u <= to_unsigned(BLINE_U, 10);
                r10_v <= to_unsigned(BLINE_V, 10); r10_sky <= '0';
            elsif v_bsel = "01" then                                 -- bush fill (textured)
                r10_y <= to_unsigned(BFILL_Y, 10); r10_u <= to_unsigned(BFILL_U, 10);
                r10_v <= to_unsigned(BFILL_V, 10); r10_sky <= '0';
            elsif r9_sky = '1' then
                r10_y <= sky_y; r10_u <= sky_u; r10_v <= sky_v; r10_sky <= '1';
            elsif r9_cl = '1' and s_creston = '1' then
                r10_y <= gC_y(r9_layer); r10_u <= gC_u(r9_layer); r10_v <= gC_v(r9_layer); r10_sky <= '0';
            elsif r9_ssin > to_signed(SOFT, 10) then
                r10_y <= gB_y(r9_layer); r10_u <= gB_u(r9_layer); r10_v <= gB_v(r9_layer); r10_sky <= '0';
            elsif r9_ssin < to_signed(-SOFT, 10) then
                r10_y <= gA_y(r9_layer); r10_u <= gA_u(r9_layer); r10_v <= gA_v(r9_layer); r10_sky <= '0';
            else
                r10_y <= gM_y(r9_layer); r10_u <= gM_u(r9_layer); r10_v <= gM_v(r9_layer); r10_sky <= '0';
            end if;
            r10_tex <= r9_tex;

            -----------------------------------------------------------------
            -- s11: always-on watercolour wash over luma; OR, in Video Sky mode,
            --      pass the incoming video through wherever the sky shows.
            -----------------------------------------------------------------
            if r10_sky = '1' and s_vidsky = '1' then
                s_io.y <= pipe(LATENCY - 1).y;
                s_io.u <= pipe(LATENCY - 1).u;
                s_io.v <= pipe(LATENCY - 1).v;
            else
                s_io.y <= std_logic_vector(clamp10(signed(resize(r10_y, 12)) + resize(r10_tex, 12)));
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
