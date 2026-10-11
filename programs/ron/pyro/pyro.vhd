-- pyro.vhd
--
-- PYRO: a performable night-sky pyrotechnics show (fork of fireworks v1.3).
--
-- Four launch slots fire rockets from staggered pads; each rocket ascends
-- on an eased (decelerating) trajectory with a glowing trail, then bursts
-- at its apogee into one of five shell types:
--   Peony  -- filled sphere of sparks (speed-randomized radii)
--   Chrys  -- chrysanthemum: tight trailing shell of forced-head comets
--   Ring   -- thin expanding circle
--   Willow -- slow golden-hour sphere with doubled gravity droop
--   Palm   -- a few thick comet arms (forced spark heads)
-- On top of the fireworks chassis: night-sky gradient + hashed starfield
-- + detonation sky-flash, water reflection below a 78% horizon (per-line
-- shimmer wobble), BLAST on the slider, FINALE barrage mode,
-- and FREEZE + time scrub (memoryless replot = age is scrubbable, so
-- bursts collapse back into rising shells).
--
-- v0.10 (2026-10-11): SHARP HEADS at 1080p -- every burning spark (plus
-- rockets and burst cores) also draws a crisp native-resolution head (3x3
-- plus, bold = 5x5 diamond) over the soft canvas glow, positioned to a
-- quarter cell (1.5 px). The engine writes a per-frame head table (2 EBR);
-- a per-line renderer stamps the next line's heads into a two-bank line
-- buffer (1 EBR each; the displayed bank is cleared behind the beam).
-- EBR came from: canvas 180 -> 172 rows (2), physics ROM -> computed on the
-- shared multiplier (1), nebula tile at 16 px texels (1).
-- v0.9 (2026-10-10): ONE colour per shell for its whole life (no two-tone
-- split, no white-hot start, no ember recolour); a shell's colour region
-- HOLDS through idle + the next ascent (embers no longer recolour when the
-- slot relaunches), rockets get a white launch column, and overlaps split
-- at "deeper inside whose ball" instead of lowest-slot-wins.
-- v0.8 (2026-10-10): P12 = BLAST (live burst scale, glided; top eighth
-- adds bold heads + full flash) -- swirl dropped (fireworks don't swirl);
-- K2 = spark count only (the slider owns size: no knob-times-slider).
-- v0.7 (2026-10-10): hand-fire removed; FREEZE scrub is
-- relative to the slider position at the freeze edge; K1 minimum = the
-- slowest rate (no silence); palette Indigo/Violet -> Cyan/Purple (the
-- wheel had a 120 deg cyan hole and a 78 deg magenta hole); K4 reads a
-- labelled-default INDEX (firmware loads 0..4) as that bank.
--
-- v0.6 (2026-10-10): decay sweep writes back at the 2-clock BRAM tap (the
-- 1-clock tap slid the canvas one cell right per frame = the "trails always
-- go right" HW report) and sweeps only the displayed rows (NTSC never
-- finished a full sweep, starving the engine); saw-active per-field tick
-- (serrated analog vsync); generated avid + avid blanking gate, black 64;
-- slot lifecycle serialized into the engine FSM (one copy, was 4 in the
-- vsync cone); no_rw_check canvas; one-adder plus-head addressing.
--
-- LC/congestion cuts (85% ceiling, all pre-planned): crackle micro-bursts,
-- flat-ring orientation, explicit full-throw flick detect (the release
-- detector covers the gesture), negative gravity.
--
-- Rendering reuses the spiroscope phosphor engine verbatim: a 320x180 x 2-bit
-- canvas in BRAM (29 EBRs), displayed 4x4 through a bilinear upscaler so
-- sparks glow, with a dithered decay sweep (slider) providing trails and
-- fading embers for free.
--
-- Particle physics is table-driven and memoryless: nothing is stored per
-- particle. Each frame every particle's position is recomputed from its
-- shell's age via two 128-entry ROM curves (one extra EBR):
--   ease[t]  = 1-(1-t)^2   (decelerating expansion / rocket ascent)
--   droop[t] = t^2         (cumulative gravity fall)
-- position = center + r(t)*(cos,sin)(theta_i) + (0, droop(t)*gravity), where
-- theta_i and the per-particle speed come from a hash of (shell seed, i).
-- The previous frame's position is recomputed the same way (age-1) and a
-- line-walker connects the two, so fast sparks draw solid streaks. All
-- products go through ONE shared 8x10 multiplier (operand@k -> product@k+2).
--
-- COLOR: the 2-bit canvas stores only brightness, so chroma is attributed at
-- readout: each pixel takes the ONE colour of the shell whose ball it sits
-- deepest inside (octagonal distance minus radius, 4-way argmin) -- zero
-- extra BRAM. A shell's colour region holds from its burst until the slot
-- bursts again, so embers keep their colour through the relaunch; rising
-- rockets draw white via a per-slot launch-column test.
--
-- Control map (PYRO):
--   K1 (reg0) LAUNCH  rate (min = one shell per slot every ~9 s)
--   K2 (reg1) SPARKS  sparks per shell (10..60)
--   K3 (reg2) GRAVITY heavy fast-falling .. floaty near-zero-g
--   K4 (reg3) HUE     Gold/Primaries/Pastel/Red-Wht-Blue/Random banks
--   K5 (reg4) TAIL    trail persistence (decay threshold)
--   K6 (reg5) SPARK   0..3: off / shimmer / heavy shimmer / +crackle strobe
--   S7 (reg6.0) TYPE   bank A peony/chrys vs bank B willow/palm
--   S8 (reg6.1) RING   mixes in ring shells
--   S9 (reg6.2) WATER  reflection below the horizon
--   S10(reg6.3) FINALE barrage choreography (4x rate, tight salvos)
--   S11(reg6.4) FREEZE physics halt (slider becomes time scrub)
--   P12(reg7)   BLAST  live burst scale (8..102 cells), every shell at
--                      once; top eighth = bold spark heads + full sky
--                      flash (the climax); in FREEZE: time scrub
--
-- Inherited fireworks passes (two-tone shells removed in v0.9): bright
-- persistent core, crisp-spark decay (bright
-- cells collapse every sweep; only val-1 embers linger on the dithered
-- path), cell/phase COUNTER rasterization (4-px cells on <=1280 rasters,
-- 6-px on 1080-class = the FULL 320x180 canvas), and RADIUS-WEIGHTED color
-- attribution (raw Voronoi carved overlapping shells into color patches).
--
-- Readout pipeline (C_LATENCY = 6):
--   end t   : stage2  cell coords + blend weights (from phase counters)
--   end t+1 : stage3  prefetch reads; per-slot |dx|,|dy| -> max/min
--   end t+2 : stage4  h-blend; raw octagonal distance + zone flag
--   end t+3 : stage5  v-blend -> t (0..192); dist - 2*radius, clamped
--   end t+4 : stage6  argmin -> owning slot + zone
--   end t+5 : stage7  luma = t*4, palette chroma
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture pyro of program_top is

    constant C_CW      : integer := 320;   -- canvas width  (cells)
    constant C_CH      : integer := 172;   -- canvas height (cells)
    -- screen cell rows (1080p = 180): rows 172..179 have no canvas behind
    -- them -- with WATER on they show the reflection anyway (it mirrors rows
    -- above the horizon); with it off they show sky without fireworks.
    -- The 2 EBRs this frees feed the sharp-head overlay.
    constant C_SH      : integer := 180;
    constant C_USHIFT  : integer := 2;     -- 4 screen px per cell
    constant C_VW      : integer := 1280;  -- canvas viewport width  (px)
    constant C_VH      : integer := 720;   -- canvas viewport height (px)
    constant C_LATENCY : integer := 7;     -- input->output register stages
    constant C_CELLS   : integer := C_CW * C_CH;  -- 55040 (27 EBRs at 2 bit)

    constant C_GROUND  : integer := 168;   -- launch pad row (cells)

    -- canvas address = cy*320 + cx (320 = 256+64 -> two shifts + an add)
    function f_addr(cy : unsigned(7 downto 0); cx : unsigned(8 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := resize(cy, 16);
        return shift_left(v, 8) + shift_left(v, 6) + resize(cx, 16);
    end function;

    -- quarter-cell coordinate -> 1080p pixel (6 px per cell = x1.5). No
    -- clamp: x stays in range (canvas range check), and a slightly negative
    -- y wraps to 2045+, which never matches a line
    function f_px(v : signed(12 downto 0)) return unsigned is
        variable t : signed(12 downto 0);
    begin
        t := v + shift_right(v, 1);
        return unsigned(t(10 downto 0));
    end function;

    -- per-particle hash: theta = h(9:0) walks the circle in ~6.3deg steps,
    -- upper bits give a per-seed speed scramble
    function f_hash(seed : unsigned(15 downto 0); i : unsigned(5 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := seed xor shift_left(resize(i, 16), 10)
                  xor shift_left(resize(i, 16), 4)
                  xor shift_left(resize(i, 16), 1);
        return v;
    end function;

    -- bit-reverse decorrelates consecutive particles' speeds (else the
    -- angle-sequential fill order turns speed into a visible spiral)
    function f_rev6(i : unsigned(5 downto 0)) return unsigned is
        variable v : unsigned(5 downto 0);
    begin
        for k in 0 to 5 loop
            v(k) := i(5 - k);
        end loop;
        return v;
    end function;

    -- sub-cell blend weight (x/8) for each phase of each cell divisor
    function f_wgt6(ph : unsigned(2 downto 0)) return unsigned is
    begin
        case ph is
            when "000"  => return "000";   -- 0
            when "001"  => return "001";   -- 1.33 -> 1
            when "010"  => return "011";   -- 2.67 -> 3
            when "011"  => return "100";   -- 4
            when "100"  => return "101";   -- 5.33 -> 5
            when others => return "111";   -- 6.67 -> 7
        end case;
    end function;
    function f_wgt3(ph : unsigned(2 downto 0)) return unsigned is
    begin
        case ph is
            when "000"  => return "000";
            when "001"  => return "011";   -- 2.67 -> 3
            when others => return "101";   -- 5.33 -> 5
        end case;
    end function;

    -- ==== physics curves: ease (expansion) + droop (gravity) ====
    --   ease[i]  = 126 - floor((127-i)^2 / 128)   (decelerating rise)
    --   droop[i] = floor(i^2 / 128)               (accelerating fall)
    -- both 0..126, computed per shell pass on the shared multiplier
    -- (EST_T*) -- this was a 128x16 ROM; its EBR went to the head overlay.
    -- Values are exactly the old table (consumers shift products >>7).
    signal s_tic, s_tip : unsigned(6 downto 0) := (others => '0'); -- index now / prev
    signal s_eC, s_eP   : unsigned(6 downto 0) := (others => '0'); -- ease
    signal s_dC, s_dP   : unsigned(6 downto 0) := (others => '0'); -- droop

    -- ==== Timing / pixel counters (c64 pattern) ====
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_x_count      : unsigned(11 downto 0) := (others => '0');
    signal s_y_count      : unsigned(11 downto 0) := (others => '0');
    signal s_line_width   : unsigned(11 downto 0) := to_unsigned(1280, 12);
    signal s_frame_height : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_line_active  : std_logic := '0';
    signal s_frame_cnt    : unsigned(7 downto 0) := (others => '0');
    -- once-per-field tick: analog vsync serrates into several edges per
    -- field, so only the first edge after active video counts (every
    -- per-frame increment -- ages, frame counter, sky flash -- uses this)
    signal s_saw   : std_logic := '0';
    signal s_ftick : std_logic := '0';

    -- Cell/phase COUNTERS replace all divide/shift rasterization: cells are
    -- 4 px (rasters <= 1280 wide) or 6 px (1080-class: 1920/6 = 320 cells,
    -- 540 field-lines/3 = 180 rows -- the FULL canvas, so 1080i renders at
    -- 6x6 screen px instead of v1.2's blocky 8x8). No division anywhere:
    -- the phase wraps and the cell increments.
    signal s_pfx  : unsigned(2 downto 0) := (others => '0');   -- x sub-phase
    signal s_pcx  : unsigned(8 downto 0) := (others => '0');   -- cell column
    -- prefetch column min(pcx+2, 319) as its own lockstep counter: stage 2
    -- then adds two registers instead of clamp -> +2 -> clamp -> add
    signal s_pcx2 : unsigned(8 downto 0) := to_unsigned(2, 9);
    -- next cell min(pcx+1, 319) for the time-multiplexed attribution
    signal s_pcx1 : unsigned(8 downto 0) := to_unsigned(1, 9);
    signal s_pfy  : unsigned(2 downto 0) := (others => '0');   -- y sub-phase
    signal s_pcy  : unsigned(7 downto 0) := (others => '0');   -- cell row
    signal s_xdiv6   : std_logic := '0';                       -- 6-px cells
    signal s_ydivsel : unsigned(1 downto 0) := "00";           -- 0:/4 1:/3 2:/6
    signal s_short   : std_logic := '0';
    -- measured usable cells (latched from the counters at line/frame end)
    signal s_cwm  : unsigned(8 downto 0)  := to_unsigned(320, 9);
    signal s_chm  : unsigned(7 downto 0)  := to_unsigned(180, 8);
    signal s_cw2  : unsigned(9 downto 0)  := to_unsigned(320, 10);
    signal s_ch2  : unsigned(9 downto 0)  := to_unsigned(180, 10);

    -- ==== Canvas BRAM: 57600 x 2-bit, addr = cy*320 + cx (1W1R) ====
    type t_canvas is array(0 to C_CELLS - 1) of unsigned(1 downto 0);
    signal s_canvas   : t_canvas := (others => (others => '0'));
    signal s_cv_we    : std_logic := '0';
    signal s_cv_waddr : unsigned(15 downto 0) := (others => '0');
    signal s_cv_wdata : unsigned(1 downto 0)  := (others => '0');
    signal s_cv_raddr : unsigned(15 downto 0) := (others => '0');
    signal s_cv_rdata : unsigned(1 downto 0)  := (others => '0');
    -- collision-free enough to skip yosys's read-before-write emulation:
    -- the sweep writes cell k while reading k+2, and a pen write landing on
    -- a display prefetch only blurs one cell for one frame
    attribute no_rw_check : boolean;
    attribute no_rw_check of s_canvas : signal is true;

    -- ==== Readout stage 2 (registered from raw counters) ====
    signal s_insq2 : std_logic := '0';
    signal s_cellx2 : unsigned(8 downto 0) := (others => '0');
    signal s_celly2 : unsigned(7 downto 0) := (others => '0');
    signal s_fx2, s_fy2 : unsigned(2 downto 0) := (others => '0');
    signal s_ftop2, s_fbot2 : std_logic := '0';
    signal s_addr_top2, s_addr_bot2 : unsigned(15 downto 0) := (others => '0');

    -- ==== Nebula cloud layer (starfield technique, 1 EBR) ====
    -- 32x32 4-bit seamless value-noise tile, 16x16-px texels (512x512 px
    -- wrap; v0.9 had 64x32 at 8x16 px in 2 EBR -- the clouds are soft
    -- either way, the EBR went to the head overlay), read per pixel and
    -- added to the sky gradient. Generated at elaboration: 2-octave
    -- bilinear noise over a wrapping 4x4 lattice.
    function f_nebh(xi, yi : integer) return integer is
        variable t : integer;
    begin
        t := (xi * 73 + yi * 199 + 55) mod 256;
        t := (t * 137 + 91) mod 251;
        t := (t * 149 + 33) mod 256;
        return t;                              -- 0..255
    end function;
    function f_nebbi(x, y, lg : integer) return integer is
        -- bilinear value noise; lattice cells are 2**lg texels, lattice
        -- wraps mod (32/2**lg) in x and y (seamless)
        variable cx, cy, fx, fy, s : integer;
        variable h00, h10, h01, h11, top, bot : integer;
    begin
        s   := 2 ** lg;
        cx  := x / s;  cy := y / s;
        fx  := x mod s;  fy := y mod s;
        h00 := f_nebh(cx mod (32 / s),       cy mod (32 / s));
        h10 := f_nebh((cx + 1) mod (32 / s), cy mod (32 / s));
        h01 := f_nebh(cx mod (32 / s),       (cy + 1) mod (32 / s));
        h11 := f_nebh((cx + 1) mod (32 / s), (cy + 1) mod (32 / s));
        top := h00 * (s - fx) + h10 * fx;
        bot := h01 * (s - fx) + h11 * fx;
        return (top * (s - fy) + bot * fy) / (s * s);   -- 0..255
    end function;
    type t_neb is array(0 to 1023) of unsigned(3 downto 0);
    function f_neb_init return t_neb is
        variable v : t_neb;
        variable n : integer;
    begin
        for y in 0 to 31 loop
            for x in 0 to 31 loop
                n := (3 * f_nebbi(x, y, 3) + f_nebbi(x * 2, y * 2, 3)) / 4;
                v(y * 32 + x) := to_unsigned(n / 16, 4);
            end loop;
        end loop;
        return v;
    end function;
    signal s_neb   : t_neb := f_neb_init;
    signal s_neb_q : unsigned(3 downto 0) := (others => '0');

    -- ==== Night sky: gradient + hashed static starfield + sky flash ====
    -- star flag decided at stage 2 (one crisp dot per lit cell), piped to 6
    signal s_bgy   : unsigned(7 downto 0) := to_unsigned(40, 8);
    -- line-rate composites: gradient + sky-flash (sky) and water base +
    -- half flash -- keeps the per-pixel color stage to a single add
    signal s_bgf   : unsigned(9 downto 0) := to_unsigned(40, 10);
    signal s_wf    : unsigned(9 downto 0) := to_unsigned(48, 10);
    signal s_star2, s_star3, s_star4, s_star5, s_star6 : std_logic := '0';
    -- dim/twinkle flag rides alongside
    signal s_stwk2, s_stwk3, s_stwk4, s_stwk5, s_stwk6 : std_logic := '0';
    -- ambient flash on any detonation (owned by p_engine, read by p_color)
    signal s_flash : unsigned(1 downto 0) := "00";

    -- ==== Water reflection (S9): mirror readout below a ~78% horizon ====
    -- all line-rate registers (computed in p_count off the pixel path):
    -- the readout row is reflected about the horizon and nudged sideways by
    -- a per-line shimmer offset; stage 6 attenuates + blue-shifts the mirror
    signal s_horiz : unsigned(7 downto 0) := to_unsigned(140, 8);
    signal s_h2m1  : unsigned(8 downto 0) := to_unsigned(279, 9);  -- 2*horiz-1
    signal s_ry    : unsigned(7 downto 0) := (others => '0');      -- readout row
    signal s_mirr  : std_logic := '0';
    signal s_wob   : signed(2 downto 0) := (others => '0');        -- -1..+1
    signal s_mir2, s_mir3, s_mir4, s_mir5, s_mir6 : std_logic := '0';
    -- line-rate row bases: ry*320 (+wobble) for the readout row and its
    -- blend neighbor -- stage 2 then needs ONE 16-bit add per address
    -- instead of clamp -> wobble -> 3-term f_addr (which missed HD timing)
    signal s_ryc   : unsigned(7 downto 0)  := (others => '0');
    signal s_rowin : std_logic := '1';     -- current row on the screen grid
    -- readout row has canvas behind it (rows >= C_CH: sky only)
    signal s_cvrow : std_logic := '1';
    signal s_cvok2, s_cvok3, s_cvok4, s_cvok5, s_cvok6, s_cvok7 : std_logic := '0';
    signal s_rowb  : unsigned(15 downto 0) := (others => '0');
    signal s_rowb1 : unsigned(15 downto 0) := to_unsigned(320, 16);

    -- ==== Prefetch tags + capture/blend registers ====
    -- (tags 4..7 delay the span-boundary commit by 4 clocks in wide mode:
    --  8-px spans otherwise rotate the blend registers mid-span = seams)
    signal s_tag1, s_tag2, s_tag3 : std_logic_vector(1 downto 0) := "00";
    signal s_tag4, s_tag5 : std_logic_vector(1 downto 0) := "00";
    signal s_TRc, s_BRc : unsigned(1 downto 0) := (others => '0');
    signal s_TLb, s_TRb, s_BLb, s_BRb : unsigned(1 downto 0) := (others => '0');

    -- ==== Blend pipeline (1/8 sub-cell precision covers both cell sizes:
    -- 4-px mode stores subpixel*2, so weights x/8 reproduce the old x/4) ====
    signal s_fx3 : unsigned(2 downto 0) := (others => '0');
    signal s_fy3, s_fy4 : unsigned(2 downto 0) := (others => '0');
    signal s_insq3, s_insq4, s_insq5, s_insq6 : std_logic := '0';
    signal s_h0, s_h1 : unsigned(4 downto 0) := (others => '0');
    signal s_t5 : unsigned(7 downto 0) := (others => '0');

    -- ==== Decay sweep (spiroscope, verbatim) ====
    signal s_decay_active : std_logic := '0';
    signal s_decay_done   : std_logic := '0';
    signal s_decay_rd     : std_logic := '0';
    signal s_decay_k      : unsigned(15 downto 0) := (others => '0');
    signal s_decay_k_d1   : unsigned(15 downto 0) := (others => '0');
    -- read data lands TWO clocks after issue (raddr reg + BRAM output reg);
    -- writing back at the 1-clock tap stored cell k-1's decayed value into
    -- cell k = the whole canvas slid one cell right per frame
    signal s_decay_rd2    : std_logic := '0';
    signal s_decay_k_d2   : unsigned(15 downto 0) := (others => '0');
    -- last cell swept: the displayed rows (+1 for the blend neighbour) only.
    -- An NTSC field has ~47k blanking clocks, fewer than the 57,600 cells,
    -- so a full sweep never finished there and the engine never ran.
    signal s_sw_last      : unsigned(15 downto 0) := to_unsigned(C_CELLS - 1, 16);
    signal s_decay_thr    : unsigned(9 downto 0)  := to_unsigned(108, 10);
    -- per-frame random dither offset: the old hash+framecount pattern
    -- TRANSLATED across the screen each frame, reading as a rightward
    -- scroll/smear of fading content
    signal s_dith         : unsigned(9 downto 0)  := (others => '0');

    -- ==== Pen write request (driven by the walker) ====
    signal s_pen_we    : std_logic := '0';
    signal s_pen_waddr : unsigned(15 downto 0) := (others => '0');
    signal s_pen_wdata : unsigned(1 downto 0)  := (others => '1');

    -- ==== Shell slots (4 rockets/shells) ====
    -- state: 00 idle (embers fading / waiting), 01 launch, 10 burst
    type t_sl_st  is array(0 to 3) of unsigned(1 downto 0);
    type t_sl_u16 is array(0 to 3) of unsigned(15 downto 0);
    type t_sl_u10 is array(0 to 3) of unsigned(9 downto 0);
    type t_sl_u9  is array(0 to 3) of unsigned(8 downto 0);
    type t_sl_u8  is array(0 to 3) of unsigned(7 downto 0);
    type t_sl_u7  is array(0 to 3) of unsigned(6 downto 0);
    type t_sl_u6  is array(0 to 3) of unsigned(5 downto 0);
    type t_sl_u3  is array(0 to 3) of unsigned(2 downto 0);
    -- power-on: slot 0 already bursting, slot 1 mid-ascent -- the show is
    -- visible the moment the program loads (also gives the sim instant frames)
    signal s_sl_state : t_sl_st  := ("10", "01", "00", "00");
    signal s_sl_age   : t_sl_u10 := (to_unsigned(16, 10), to_unsigned(24, 10),
                                     (others => '0'), (others => '0'));
    signal s_sl_delay : t_sl_u10 := (to_unsigned(200, 10), to_unsigned(200, 10),
                                     to_unsigned(20, 10), to_unsigned(70, 10));
    signal s_sl_seed  : t_sl_u16 := (x"ACE1", x"5EED", x"C0DE", x"BEA7");
    signal s_sl_cx    : t_sl_u9  := (to_unsigned(40, 9),  to_unsigned(120, 9),
                                     to_unsigned(200, 9), to_unsigned(280, 9));
    signal s_sl_cy    : t_sl_u8  := (to_unsigned(60, 8), to_unsigned(C_GROUND, 8),
                                     to_unsigned(C_GROUND, 8), to_unsigned(C_GROUND, 8));
    signal s_sl_apy   : t_sl_u8  := (others => to_unsigned(60, 8));
    signal s_sl_typ   : t_sl_u3  := (others => "000");
    -- the shell's one colour (palette index; pastel flag in s_sl_pas)
    signal s_sl_col   : t_sl_u3  := ("000", "011", "001", "001");
    -- current color-split radius (half the shell's expansion radius)
    signal s_sl_rsp   : t_sl_u7  := (others => to_unsigned(12, 7));
    signal s_sl_n     : t_sl_u6  := (others => to_unsigned(24, 6));
    signal s_sl_ever  : std_logic_vector(3 downto 0) := "0011";
    -- free-running shared drift pacing (frame-counter decodes, one per
    -- pace, read by the lifecycle for every slot)
    signal s_pc_sink : std_logic := '0';
    signal s_pc_mom8 : std_logic := '0';
    signal s_pc_mom16: std_logic := '0';

    -- raster-scaled layout (recomputed each vsync from the measured raster,
    -- so the show fills the screen on SD rasters too, not just 1280x720)
    signal s_ground : unsigned(7 downto 0) := to_unsigned(C_GROUND, 8);
    signal s_apbase : unsigned(7 downto 0) := to_unsigned(33, 8);    -- apogee band top
    signal s_cwu    : unsigned(8 downto 0) := to_unsigned(320, 9);   -- usable cells
    signal s_cwm32  : signed(10 downto 0)  := to_signed(288, 11);    -- cwu - 32
    signal s_pad    : t_sl_u9 := (to_unsigned(40, 9),  to_unsigned(120, 9),
                                  to_unsigned(200, 9), to_unsigned(280, 9));

    -- shell types (chrys = peony with forced heads + tight trailing spread;
    -- comet = a few max-speed straight streaks with bright heads and half
    -- gravity)
    constant C_TY_PEONY   : unsigned(2 downto 0) := "000";
    constant C_TY_RING    : unsigned(2 downto 0) := "001";
    constant C_TY_WILLOW  : unsigned(2 downto 0) := "010";
    constant C_TY_PALM    : unsigned(2 downto 0) := "011";
    constant C_TY_CHRYS   : unsigned(2 downto 0) := "100";
    constant C_TY_COMET   : unsigned(2 downto 0) := "101";
    -- per-shell pastel flag (K4 pastel bank -> desaturated palette half)
    signal s_sl_pas  : std_logic_vector(3 downto 0) := (others => '0');

    -- ==== Per-frame latched controls ====
    signal s_sizeK   : unsigned(6 downto 0) := to_unsigned(83, 7);   -- 8..102 (P12)
    signal s_gravK   : unsigned(5 downto 0) := to_unsigned(23, 6);   -- 0..63
    signal s_rateD   : unsigned(9 downto 0) := to_unsigned(180, 10);
    signal s_dens    : unsigned(5 downto 0) := to_unsigned(40, 6);
    signal s_hueSel  : unsigned(2 downto 0) := "000";        -- K4 palette bank
    signal s_sparkK  : unsigned(1 downto 0) := "01";         -- K6 level 0..3
    signal s_twk_thr : unsigned(2 downto 0) := "110";        -- shimmer duty
    signal s_bankB     : std_logic := '0';                   -- S7 willow/palm
    signal s_ringOn    : std_logic := '0';                   -- S8
    signal s_sw_water  : std_logic := '1';                   -- S9
    signal s_sw_finale : std_logic := '0';                   -- S10
    signal s_freeze    : std_logic := '0';                   -- S11

    -- ==== P12: BLAST (outside FREEZE) / time scrub (inside FREEZE) ====
    signal s_sldr    : unsigned(9 downto 0) := (others => '0');  -- raw slider
    -- blast amount, glided (ease by diff/8 per frame: no ADC jitter on
    -- live shells, a smooth swell when the slider moves); 13 bits =
    -- slider*8, power-on at the TOML default (800)
    signal s_bla     : unsigned(12 downto 0) := to_unsigned(6400, 13);
    -- climax zone (top eighth): share of sparks given bold plus heads
    -- (0..15 sixteenths) and the full-strength sky flash
    signal s_hboost  : unsigned(3 downto 0) := (others => '0');
    signal s_bltop   : std_logic := '0';
    -- slider position when FREEZE engaged: scrub = (slider - this)/4, so
    -- flipping FREEZE holds the exact frame wherever the slider rests
    signal s_scr_lat : unsigned(9 downto 0)  := (others => '0');

    -- ==== Free-running LFSR (seeds) ====
    signal s_lfsr : unsigned(15 downto 0) := x"BEEF";

    -- ==== Particle engine FSM ====
    constant EST_IDLE : integer := 0;
    constant EST_SLOT : integer := 1;
    constant EST_R0 : integer := 2;   -- rocket: R0..R6 = 2..8
    constant EST_P0 : integer := 9;   -- burst prep: P0..P7 = 9..16
    constant EST_PC : integer := 17;  -- persistent core flash issue
    constant EST_Q0 : integer := 18;  -- particle: Q0..Q12 = 18..30
    constant EST_AGE  : integer := 31;  -- effective-timeline compute
    constant EST_AGE2 : integer := 32;  -- clamp/split + phase dispatch
    -- per-frame slot lifecycle, ONE slot at a time ahead of its replot (was
    -- four unrolled copies in the vsync cone: ~600 LUT+carry and the
    -- recurring critical path of v0.3-v0.5)
    constant EST_LIFE  : integer := 33;  -- hoist slot regs + new seed/spawn x
    constant EST_LIFE1 : integer := 34;  -- age/spawn/transition (+ hand-fire)
    constant EST_LIFE2 : integer := 35;  -- idle -> next slot, else replot
    constant EST_LIFED : integer := 36;  -- decide: registered action flags
    constant EST_T0    : integer := 37;  -- physics curves: T0..T6 = 37..43

    signal s_est : integer range 0 to 43 := EST_IDLE;

    -- lifecycle hoists: the current slot's fields as plain registers, so
    -- the update logic never reaches through the 4-way slot-array muxes
    signal s_lc_st   : unsigned(1 downto 0)  := "00";
    signal s_lc_age  : unsigned(9 downto 0)  := (others => '0');
    signal s_lc_exp  : std_logic := '0';                       -- age >= delay
    signal s_lc_seed : unsigned(15 downto 0) := (others => '0');
    signal s_lc_ns   : unsigned(15 downto 0) := (others => '0'); -- respawn seed
    signal s_lc_bx   : signed(10 downto 0)   := (others => '0'); -- unclamped x
    signal s_lc_cx   : unsigned(8 downto 0)  := (others => '0');
    signal s_lc_cy   : unsigned(7 downto 0)  := (others => '0');
    signal s_lc_apy  : unsigned(7 downto 0)  := (others => '0');
    -- a detonation happened this frame: the sky flash reloads at the tick
    signal s_boom    : std_logic := '0';
    -- decide -> apply: one-cycle action flags + one-hot slot, so every slot
    -- field's write enable is flag AND one-hot (2 register inputs). The
    -- first v0.6 build drove ~300 slot FFs' CEN from the spawn decision
    -- through 5 LUT levels: 12.7 ns of routing, HD 63-67 MHz.
    signal s_lc_cxs  : unsigned(8 downto 0)  := (others => '0'); -- clamped x
    signal s_lc_oh   : std_logic_vector(3 downto 0) := (others => '0');
    signal s_lc_sp, s_lc_inc, s_lc_boom, s_lc_die : std_logic := '0';
    signal s_lc_sink, s_lc_mvr, s_lc_mvl : std_logic := '0';

    -- ==== FREEZE + scrub: positions are pure functions of age, so the
    -- slider can offset a unified timeline (launch 0..63, burst 64..191)
    -- and bursts genuinely collapse back into rising shells ====
    signal s_scr_r  : signed(9 downto 0) := (others => '0');  -- scrub
    signal s_scr_b  : signed(9 downto 0) := to_signed(64, 10);-- scrub + 64
    signal s_tl     : signed(9 downto 0) := (others => '0');  -- raw timeline
    signal s_cage   : unsigned(6 downto 0) := (others => '0');-- phase-local age
    signal s_cphase : std_logic := '0';                       -- 0 rocket 1 burst
    signal s_ctrack : std_logic := '0';  -- really ascending: track cy write
    signal s_go  : std_logic := '0';
    signal s_ei  : integer range 0 to 4 := 0;
    signal s_pi  : unsigned(5 downto 0) := (others => '0');

    signal s_h      : unsigned(15 downto 0) := (others => '0');
    signal s_ang8   : unsigned(7 downto 0)  := (others => '0');
    signal s_vfrac  : unsigned(7 downto 0)  := (others => '0');
    signal s_hsc    : unsigned(7 downto 0)  := (others => '0');
    signal s_sinP   : signed(9 downto 0)    := (others => '0');
    signal s_cosP   : signed(9 downto 0)    := (others => '0');
    signal s_easeP  : unsigned(7 downto 0)  := (others => '0');
    signal s_droopC : unsigned(7 downto 0)  := (others => '0');
    signal s_droopP : unsigned(7 downto 0)  := (others => '0');
    signal s_rbaseC : unsigned(6 downto 0)  := (others => '0');
    signal s_rbaseP : unsigned(6 downto 0)  := (others => '0');
    signal s_dreffC : unsigned(6 downto 0)  := (others => '0');
    signal s_dreffP : unsigned(6 downto 0)  := (others => '0');
    signal s_rc     : unsigned(6 downto 0)  := (others => '0');
    signal s_riseP  : unsigned(6 downto 0)  := (others => '0');
    signal s_xc, s_yc, s_xp, s_yp : signed(10 downto 0) := (others => '0');
    signal s_vis    : std_logic := '0';
    signal s_ok     : std_logic := '0';
    signal s_lastp  : std_logic := '0';
    signal s_val    : unsigned(1 downto 0) := "11";
    signal s_head   : std_logic := '0';
    -- age-derived per-shell flags, registered once per slot pass (P0) so the
    -- per-spark states never reach through the slot-array age mux
    signal s_val_pre  : unsigned(1 downto 0) := "11";
    signal s_head_pre : std_logic := '0';
    signal s_twk_en   : std_logic := '0';
    signal s_crkl_en  : std_logic := '0';
    signal s_apx      : std_logic := '0';   -- rocket near apogee: bloom head
    signal s_core_on  : std_logic := '0';
    signal s_core_val : unsigned(1 downto 0) := "11";
    -- slot-stable copies latched at P0/R0: the inner spark loop reads these
    -- plain registers instead of 4-way slot-array muxes (routing pressure)
    signal s_ccx   : unsigned(8 downto 0)  := (others => '0');
    signal s_ccy   : unsigned(7 downto 0)  := (others => '0');
    signal s_ctyp  : unsigned(2 downto 0)  := (others => '0');
    signal s_cseed : unsigned(15 downto 0) := (others => '0');
    signal s_cn    : unsigned(5 downto 0)  := (others => '0');

    -- shared sin/cos (combinational quarter-wave fold) + shared multiplier
    -- (the SDK's 1024-entry sin_cos_full_lut_10x10 gets inferred as 6 EBRs
    --  here, which overflows the device next to the 29-EBR canvas; we only
    --  use 256 distinct angles, so a 64-entry logic ROM is exact anyway)
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

    -- quarter-wave fold: sin(a) from quadrant a(9:8) and index a(7:2)
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

    signal s_lut_angle : std_logic_vector(9 downto 0) := (others => '0');
    signal s_lut_sin   : signed(9 downto 0);
    signal s_mul_a     : signed(7 downto 0)  := (others => '0');
    signal s_mul_b     : signed(9 downto 0)  := (others => '0');
    signal s_mul_p     : signed(17 downto 0) := (others => '0');

    -- ==== Plot request (engine -> walker) ====
    signal s_pl_stb  : std_logic := '0';
    signal s_pl_fx   : unsigned(8 downto 0) := (others => '0');
    signal s_pl_fy   : unsigned(7 downto 0) := (others => '0');
    signal s_pl_tx   : unsigned(8 downto 0) := (others => '0');
    signal s_pl_ty   : unsigned(7 downto 0) := (others => '0');
    signal s_pl_val  : unsigned(1 downto 0) := "11";
    signal s_pl_head : std_logic := '0';

    -- ==== Pen writer: line-walker + spark heads ====
    signal s_w_active : std_logic := '0';
    signal s_w_step   : integer range 0 to 6 := 0;
    signal s_w_cnt    : unsigned(4 downto 0) := (others => '0');
    signal s_wk_x  : unsigned(8 downto 0) := (others => '0');
    signal s_wk_y  : unsigned(7 downto 0) := (others => '0');
    signal s_wk_tx : unsigned(8 downto 0) := (others => '0');
    signal s_wk_ty : unsigned(7 downto 0) := (others => '0');
    signal s_w_val  : unsigned(1 downto 0) := "11";
    signal s_w_head : std_logic := '0';
    -- head cell address, kept for the plus-shaped head: its four arms are
    -- this +-1 / +-320 (one adder instead of four more f_addr copies)
    signal s_wk_a   : unsigned(15 downto 0) := (others => '0');

    -- ==== Palette (U/V swapped for HW: stored U = Cr, stored V = Cb) ====
    -- Full hue wheel: 0 Red, 1 Orange, 2 Yellow, 3 Green, 4 Blue, 5 Cyan,
    -- 6 Purple, 7 White. R/O/Y/G/B are the scope-verified set; Cyan (180
    -- deg) and Purple (295 deg) replace Indigo/Violet (7.6 deg apart, while
    -- cyan and magenta were 120/78 deg holes), computed by the same
    -- sRGB -> BT.601-limited -> U/V-swap recipe (reproduces the verified
    -- rows within 2 codes);
    -- entries 8..15 are the pastel bank (chroma halved toward 512) so the
    -- pastel flag is just index bit 3 -- no extra mux depth in stage 6
    type t_pal16 is array(0 to 15) of unsigned(9 downto 0);
    constant C_PAL_U : t_pal16 := (
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10), to_unsigned(136, 10),
        to_unsigned(440, 10), to_unsigned( 64, 10), to_unsigned(852, 10), to_unsigned(512, 10),
        to_unsigned(736, 10), to_unsigned(642, 10), to_unsigned(548, 10), to_unsigned(324, 10),
        to_unsigned(476, 10), to_unsigned(288, 10), to_unsigned(682, 10), to_unsigned(512, 10));
    constant C_PAL_V : t_pal16 := (
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10), to_unsigned(216, 10),
        to_unsigned(960, 10), to_unsigned(663, 10), to_unsigned(821, 10), to_unsigned(512, 10),
        to_unsigned(436, 10), to_unsigned(362, 10), to_unsigned(288, 10), to_unsigned(364, 10),
        to_unsigned(736, 10), to_unsigned(587, 10), to_unsigned(666, 10), to_unsigned(512, 10));
    -- per-color LUMA CEILING (the palette's verified Y column, slightly
    -- lifted; pastels halfway to white): every hue is distinguished as
    -- much by brightness as by chroma -- red at a forced-bright 768 reads
    -- PINK, orange reads cream. Capping spark luma per color is what
    -- makes the full rainbow actually visible on screen.
    constant C_PAL_YC : t_pal16 := (
        to_unsigned(400, 10), to_unsigned(620, 10), to_unsigned(840, 10), to_unsigned(640, 10),
        to_unsigned(320, 10), to_unsigned(700, 10), to_unsigned(470, 10), to_unsigned(768, 10),
        to_unsigned(584, 10), to_unsigned(696, 10), to_unsigned(800, 10), to_unsigned(704, 10),
        to_unsigned(544, 10), to_unsigned(734, 10), to_unsigned(619, 10), to_unsigned(768, 10));
    -- ==== Nearest-shell color attribution ====
    type t_at_u9  is array(0 to 3) of unsigned(8 downto 0);
    type t_at_u8  is array(0 to 4) of unsigned(7 downto 0);
    signal s_acx : t_at_u9 := (others => to_unsigned(160, 9));
    signal s_acy : t_at_u8 := (others => to_unsigned(90, 8));
    signal s_aen : std_logic_vector(3 downto 0) := "0000";
    -- white launch column per slot (ascent only): rocket x, head row
    signal s_rkon : std_logic_vector(3 downto 0) := (others => '0');
    signal s_rkx  : t_at_u9 := (others => (others => '0'));
    signal s_rky  : t_at_u8 := (others => (others => '0'));
    signal s_rk6  : std_logic := '0';
    -- 3-bit palette indices (chroma resolved in stage 6 via the constant
    -- palette, spiroscope-style: 20x 10-bit color regs was ~250 LC)
    type t_at_u3 is array(0 to 3) of unsigned(2 downto 0);
    signal s_aci   : t_at_u3 := (others => "001");
    signal s_apas  : std_logic_vector(3 downto 0) := (others => '0');
    type t_at_u7 is array(0 to 3) of unsigned(6 downto 0);
    signal s_arsp : t_at_u7 := (others => (others => '0'));
    -- stage 3: per-slot max/min of |dx|,|dy|


    -- stage 4: raw octagonal distance + inner/outer zone flag
    type t_at_u10a is array(0 to 3) of unsigned(9 downto 0);

    -- stage 5: RADIUS-WEIGHTED distance (raw - 2*split_radius, clamped to
    -- 8 bits -- beyond 255 cells ownership is arbitrary, and the narrower
    -- compares keep the stage-6 argmin off the critical path):
    -- an active shell owns its own ball of sparks, so overlapping shells no
    -- longer carve each other into Voronoi color patches
    type t_at_u8a is array(0 to 3) of unsigned(7 downto 0);
    -- TIME-MULTIPLEXED attribution: the distance inputs change only once
    -- per cell (4 or 6 px), so ONE pipeline walks the 4 slots on the cell's
    -- sub-phases (pfx 0..3) for the NEXT cell, writing result bank s_ar;
    -- when a new cell's first pixel reaches stage 5 the bank commits to
    -- s_ac, which the stage-6 argmin reads. Lossless vs 4 parallel copies
    -- (was ~480 LUT / 230 carry of the chip).
    signal s_tk2, s_tk3, s_tk4 : unsigned(1 downto 0) := (others => '0');
    signal s_tv2, s_tv3, s_tv4 : std_logic := '0';          -- phase has a slot
    signal s_tcx2 : unsigned(8 downto 0) := (others => '0'); -- next cell x
    signal s_tcy2 : unsigned(7 downto 0) := (others => '0'); -- readout row
    signal s_tax2 : unsigned(8 downto 0) := (others => '0');
    signal s_tay2 : unsigned(7 downto 0) := (others => '0');
    signal s_trs2, s_trs3, s_trs4 : unsigned(6 downto 0) := (others => '0');
    signal s_ten2, s_ten3, s_ten4 : std_logic := '0';
    signal s_trk2 : std_logic := '0';
    signal s_trx2 : unsigned(8 downto 0) := (others => '0');
    signal s_try2 : unsigned(7 downto 0) := (others => '0');
    signal s_trkh3, s_trkh4 : std_logic := '0';
    signal s_nc2, s_nc3, s_nc4 : std_logic := '0';           -- new-cell pixel
    signal s_tmax3, s_tmin3 : unsigned(8 downto 0) := (others => '0');
    signal s_td4  : unsigned(9 downto 0) := (others => '0');
    signal s_ar, s_ac : t_at_u8a := (others => (others => '1'));
    signal s_arrk, s_acrk : std_logic_vector(3 downto 0) := (others => '0');
    -- stage 6: final color index + far flag
    signal s_cidx6 : unsigned(2 downto 0) := "000";
    signal s_t6    : unsigned(7 downto 0) := (others => '0');
    -- stage 6a->6b pipe: the argmin->pastel->palette->luma-cap cone was
    -- one combinational stage too deep for hd timing, so the owning color
    -- index resolves a stage ahead of the cap/branch muxes
    signal s_ci7   : unsigned(3 downto 0) := (others => '0');
    signal s_t7    : unsigned(7 downto 0) := (others => '0');
    signal s_star7, s_stwk7, s_mir7, s_insq7 : std_logic := '0';

    -- ==== Sharp-head overlay (1080p) ====
    -- Each frame the engine writes one entry per bright head: [30] bold,
    -- [26:16] y px, [10:0] x px (1080p pixels from quarter-cell positions).
    -- A per-line renderer stamps the NEXT line's heads into one bank of a
    -- two-bank line buffer while the other bank is displayed (and cleared
    -- one pixel behind the beam). Head x/y come from the same counters as
    -- the canvas readout (x_count = 6*cell + phase), so a head lands on its
    -- own glow by construction.
    type t_ht is array(0 to 255) of std_logic_vector(31 downto 0);
    signal s_ht    : t_ht := (others => (others => '0'));
    attribute no_rw_check of s_ht : signal is true;
    signal s_ht_we : std_logic := '0';
    signal s_ht_wa : unsigned(7 downto 0) := (others => '0');
    signal s_ht_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_ht_ra : unsigned(7 downto 0) := (others => '0');
    signal s_ht_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_hq2   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_hcnt  : unsigned(8 downto 0) := (others => '0');  -- heads this frame
    signal s_hn    : unsigned(8 downto 0) := (others => '0');  -- count for renderer
    signal s_hrdy  : std_logic := '0';                         -- table is final
    signal s_hd_on : std_logic := '0';                         -- 1080p raster
    -- engine side: radius fraction, rf*cos/sin, droop and centre in quarter
    -- cells, the head's quarter-cell position and its pixel coordinates
    signal s_rf    : unsigned(1 downto 0) := (others => '0');
    signal s_rfx, s_rfy : signed(11 downto 0) := (others => '0');
    signal s_dfC4  : unsigned(9 downto 0) := (others => '0');
    signal s_cyd4  : unsigned(11 downto 0) := (others => '0');
    signal s_x4, s_y4 : signed(12 downto 0) := (others => '0');
    signal s_hx, s_hy : unsigned(10 downto 0) := (others => '0');
    -- renderer: 1 issue, 2 read, 3 copy, 4 row test, 5 span, 6 stamp
    signal s_rs    : integer range 0 to 6 := 0;
    signal s_ri    : unsigned(8 downto 0) := (others => '0');
    signal s_rT    : unsigned(10 downto 0) := (others => '0');
    signal s_rhit  : std_logic := '0';
    signal s_rw    : unsigned(1 downto 0) := (others => '0');
    signal s_rx    : unsigned(10 downto 0) := (others => '0');
    signal s_rn    : unsigned(2 downto 0) := (others => '0');
    signal s_rwe   : std_logic := '0';
    signal s_rwa   : unsigned(10 downto 0) := (others => '0');
    signal s_rwb   : std_logic := '0';
    -- line buffer banks (one EBR each, 1 bit per 1080p pixel)
    type t_lb is array(0 to 2047) of std_logic;
    signal s_lb0, s_lb1 : t_lb := (others => '0');
    attribute no_rw_check of s_lb0 : signal is true;
    attribute no_rw_check of s_lb1 : signal is true;
    signal s_lb0_we, s_lb1_we : std_logic := '0';
    signal s_lb0_wa, s_lb1_wa : unsigned(10 downto 0) := (others => '0');
    signal s_lb0_wd, s_lb1_wd : std_logic := '0';
    signal s_lb_ra  : unsigned(10 downto 0) := (others => '0');
    signal s_lb0_q, s_lb1_q : std_logic := '0';
    signal s_lb_act2, s_dbank2, s_dbank3 : std_logic := '0';
    signal s_lbc_a  : unsigned(10 downto 0) := (others => '0');
    signal s_lbc_e, s_lbc_b : std_logic := '0';
    signal s_hl4, s_hl5, s_hl6, s_hl7 : std_logic := '0';

    -- ==== Colour / output (stage 6) ====
    signal s_d_y   : unsigned(9 downto 0) := (others => '0');
    signal s_d_u   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_d_v   : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ==== sync delay SR (output is fully synthesized: no video delay) ====
    -- s_avid_sr carries the SOURCE avid (only its first taps are read, as
    -- the decay sweep's guard band); the output avid is generated
    type t_bit_sr  is array(0 to C_LATENCY - 1) of std_logic;
    signal s_hsync_sr, s_vsync_sr, s_field_sr, s_avid_sr : t_bit_sr := (others => '0');
    signal s_gav_sr : t_bit_sr := (others => '0');

    -- ==== generated avid (aurora/cubist HW-confirmed recipe) ====
    -- The encoders get no data-enable, so they pair Cb/Cr by counting from
    -- hsync, while the core's 444->422 packer restarts on Cb at every avid
    -- rise: a source avid that lands a clock off its usual hsync distance
    -- swaps U/V for that line (thin wrong-coloured lines; neutral immune --
    -- and pyro's sky is not neutral). Start 3 or 4 clocks after the
    -- source's first avid rise, whichever keeps the hsync->start PARITY at
    -- the voted usual value; end after the field-latched line width.
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1279, 12);

begin

    -- ====================================================================
    -- stage 1: pixel/line counters + per-frame geometry
    -- ====================================================================
    p_count : process(clk)
        variable v_ryc : unsigned(7 downto 0);
        variable v_ry1 : unsigned(7 downto 0);
        variable v_rb  : unsigned(15 downto 0);
        variable v_rb1 : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            if data_in.avid = '1' then
                s_x_count     <= s_x_count + 1;
                s_line_active <= '1';
                -- cell/phase advance (saturates at the canvas edge)
                if s_pcx < to_unsigned(C_CW, 9) then
                    if (s_xdiv6 = '1' and s_pfx = 5)
                       or (s_xdiv6 = '0' and s_pfx = 3) then
                        s_pfx <= (others => '0');
                        s_pcx <= s_pcx + 1;
                        if s_pcx2 < to_unsigned(C_CW - 1, 9) then
                            s_pcx2 <= s_pcx2 + 1;
                        end if;
                        if s_pcx1 < to_unsigned(C_CW - 1, 9) then
                            s_pcx1 <= s_pcx1 + 1;
                        end if;
                    else
                        s_pfx <= s_pfx + 1;
                    end if;
                end if;
            end if;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_x_count > 0 then s_line_width <= s_x_count; end if;
                s_x_count <= (others => '0');
                if s_pcx > 0 then s_cwm <= s_pcx; end if;
                s_pfx <= (others => '0');
                s_pcx <= (others => '0');
                s_pcx2 <= to_unsigned(2, 9);
                s_pcx1 <= to_unsigned(1, 9);
                if s_line_active = '1' then
                    s_y_count     <= s_y_count + 1;
                    s_line_active <= '0';
                    if s_pcy < to_unsigned(C_SH, 8) then
                        if (s_ydivsel = "01" and s_pfy = 2)
                           or (s_ydivsel = "00" and s_pfy = 3)
                           or (s_ydivsel = "10" and s_pfy = 5) then
                            s_pfy <= (others => '0');
                            s_pcy <= s_pcy + 1;
                        else
                            s_pfy <= s_pfy + 1;
                        end if;
                    end if;
                end if;
            end if;

            -- counter resets are idempotent, so every vsync edge may do them
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_y_count > 0 then s_frame_height <= s_y_count; end if;
                s_y_count     <= (others => '0');
                s_x_count     <= (others => '0');
                if s_pcy > 0 then s_chm <= s_pcy; end if;
                s_pfy <= (others => '0');
                s_pcy <= (others => '0');
                s_line_active <= '0';
            end if;
            -- per-field tick: first vsync edge after active video only
            s_ftick <= '0';
            if data_in.avid = '1' then
                s_saw <= '1';
            elsif data_in.vsync_n = '0' and s_prev_vsync_n = '1' and s_saw = '1' then
                s_saw       <= '0';
                s_ftick     <= '1';
                s_frame_cnt <= s_frame_cnt + 1;
            end if;

            -- divisor selects + clamped cell dims (settled registers; the
            -- raster is stable between vsyncs so a few clocks' lag is free)
            if s_line_width > to_unsigned(1280, 12) then
                s_xdiv6 <= '1';
                if s_frame_height < to_unsigned(800, 12) then
                    s_ydivsel <= "01";      -- 1080i field: /3 (6 screen lines)
                else
                    s_ydivsel <= "10";      -- 1080p: /6
                end if;
            else
                s_xdiv6   <= '0';
                s_ydivsel <= "00";          -- /4
            end if;
            if s_line_width > to_unsigned(1280, 12)
               and s_frame_height < to_unsigned(800, 12) then
                s_short <= '1';
            else
                s_short <= '0';
            end if;
            if s_cwm > to_unsigned(C_CW, 9) then
                s_cw2 <= to_unsigned(C_CW, 10);
            else
                s_cw2 <= resize(s_cwm, 10);
            end if;
            -- night-sky gradient: dark zenith lifting toward the horizon
            -- (line-rate value, sampled at stage 6 -- 1-line skew invisible)
            s_bgy <= to_unsigned(40, 8) + resize(s_pcy(7 downto 2), 8);
            -- pre-composited backgrounds (line-rate adds, not pixel-rate)
            s_bgf <= resize(s_bgy, 10) + resize(s_flash & "000000", 10);
            s_wf  <= to_unsigned(48, 10) + resize(s_flash & "00000", 10);

            -- water horizon at ~78% of the usable cell height (1/2+1/4+1/32)
            s_horiz <= resize(shift_right(s_ch2, 1) + shift_right(s_ch2, 2)
                              + shift_right(s_ch2, 5), 8);
            s_h2m1  <= (s_horiz & '0') - 1;
            -- mirrored readout row + per-line shimmer wobble (all line-rate;
            -- registers_in(6)(2) = S9 Water)
            if registers_in(6)(2) = '1' and s_pcy >= s_horiz then
                s_mirr <= '1';
                if resize(s_pcy, 9) <= s_h2m1 then
                    s_ry <= resize(s_h2m1 - resize(s_pcy, 9), 8);
                else
                    s_ry <= (others => '0');
                end if;
                -- shimmer: hash of (row, slow frame bits) -> -1 / 0 / +1
                case (s_pcy(1 downto 0) xor s_pcy(3 downto 2))
                     xor s_frame_cnt(4 downto 3) is
                    when "01"   => s_wob <= to_signed(1, 3);
                    when "11"   => s_wob <= to_signed(-1, 3);
                    when others => s_wob <= (others => '0');
                end case;
            else
                s_mirr <= '0';
                s_ry   <= s_pcy;
                s_wob  <= (others => '0');
            end if;
            -- clamped row + row-base addresses (line-rate; wobble folded in
            -- so the pixel path never sees it -- a wrapped edge read on a
            -- wobbled mirror line lands one row over, invisibly)
            if s_ry < to_unsigned(C_CH, 8) then
                v_ryc := s_ry;
            else
                v_ryc := to_unsigned(C_CH - 1, 8);
            end if;
            if s_mirr = '1' then
                if v_ryc = 0 then v_ry1 := v_ryc;
                else v_ry1 := v_ryc - 1; end if;
            else
                if v_ryc = to_unsigned(C_CH - 1, 8) then v_ry1 := v_ryc;
                else v_ry1 := v_ryc + 1; end if;
            end if;
            v_rb  := shift_left(resize(v_ryc, 16), 8) + shift_left(resize(v_ryc, 16), 6);
            v_rb1 := shift_left(resize(v_ry1, 16), 8) + shift_left(resize(v_ry1, 16), 6);
            s_ryc   <= v_ryc;
            if s_pcy < to_unsigned(C_SH, 8) then
                s_rowin <= '1';
            else
                s_rowin <= '0';
            end if;
            if s_ry < to_unsigned(C_CH, 8) then
                s_cvrow <= '1';
            else
                s_cvrow <= '0';
            end if;
            s_rowb  <= unsigned(signed(v_rb)  + resize(s_wob, 16));
            s_rowb1 <= unsigned(signed(v_rb1) + resize(s_wob, 16));
            if s_chm > to_unsigned(C_SH, 8) then
                s_ch2 <= to_unsigned(C_SH, 10);
            elsif s_chm < to_unsigned(60, 8) then
                s_ch2 <= to_unsigned(60, 10);
            else
                s_ch2 <= resize(s_chm, 10);
            end if;
        end if;
    end process p_count;

    -- ====================================================================
    -- stage 2: geometry -> cell coords + sub-position + prefetch addresses
    -- ====================================================================
    p_addr : process(clk)
        variable v_ccx      : unsigned(8 downto 0);
        variable v_ccx1     : unsigned(8 downto 0);
        variable v_fx, v_fy : unsigned(2 downto 0);
        variable v_ph0, v_ph1 : boolean;
        variable v_in       : boolean;
        variable v_sa       : unsigned(15 downto 0);
        variable v_sh       : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_in := (s_pcx < to_unsigned(C_CW, 9)) and (s_rowin = '1');
            -- x clamp (counter saturates AT the canvas edge); the row-side
            -- clamp/compare live in line-rate registers (s_ryc / s_rowin)
            if s_pcx < to_unsigned(C_CW, 9) then
                v_ccx := s_pcx;
            else
                v_ccx := to_unsigned(C_CW - 1, 9);
            end if;
            if s_xdiv6 = '1' then
                v_fx := f_wgt6(s_pfx);
            else
                v_fx := s_pfx(1 downto 0) & '0';
            end if;
            case s_ydivsel is
                when "01"   => v_fy := f_wgt3(s_pfy);
                when "10"   => v_fy := f_wgt6(s_pfy);
                when others => v_fy := s_pfy(1 downto 0) & '0';
            end case;
            -- mirrored rows blend toward the row ABOVE with inverted weight
            -- (not v_fy = 7 - v_fy exactly, for both weight tables)
            if s_mirr = '1' then
                v_fy := not v_fy;
            end if;
            v_ph0 := s_pfx = 0;
            v_ph1 := s_pfx = 1;
            -- prefetch target: span S+1's RIGHT neighbour = cell S+2
            -- (= min(pcx+2, 319), kept as a lockstep counter in p_count)
            v_ccx1 := s_pcx2;

            s_cellx2 <= v_ccx;
            s_celly2 <= s_ryc;
            s_fx2 <= v_fx;
            s_fy2 <= v_fy;
            s_addr_top2 <= s_rowb  + resize(v_ccx1, 16);
            s_addr_bot2 <= s_rowb1 + resize(v_ccx1, 16);
            if v_in and v_ph0 then s_ftop2 <= '1'; else s_ftop2 <= '0'; end if;
            if v_in and v_ph1 then s_fbot2 <= '1'; else s_fbot2 <= '0'; end if;
            if v_in and v_ccx >= 2 then s_insq2 <= '1'; else s_insq2 <= '0'; end if;
            s_mir2 <= s_mirr;
            s_cvok2 <= s_cvrow;
            -- attribution: this phase's slot (pfx 0..3) for the next cell
            s_tk2  <= s_pfx(1 downto 0);
            if s_pfx(2) = '0' then
                s_tv2 <= '1';
            else
                s_tv2 <= '0';
            end if;
            s_tcx2 <= s_pcx1;
            s_tcy2 <= s_ryc;
            s_tax2 <= s_acx(to_integer(s_pfx(1 downto 0)));
            s_tay2 <= s_acy(to_integer(s_pfx(1 downto 0)));
            s_trs2 <= s_arsp(to_integer(s_pfx(1 downto 0)));
            s_ten2 <= s_aen(to_integer(s_pfx(1 downto 0)));
            s_trk2 <= s_rkon(to_integer(s_pfx(1 downto 0)));
            s_trx2 <= s_rkx(to_integer(s_pfx(1 downto 0)));
            s_try2 <= s_rky(to_integer(s_pfx(1 downto 0)));
            if s_pfx = 0 then
                s_nc2 <= '1';
            else
                s_nc2 <= '0';
            end if;
            -- head line buffer: read this pixel; bank = line parity
            s_lb_ra   <= s_x_count(10 downto 0);
            s_lb_act2 <= s_avid_sr(0);
            s_dbank2  <= s_y_count(0);

            -- static starfield: per-cell hash of the row base and column --
            -- XOR-fold only (a second 16-bit ADD here was carry-chain depth
            -- on the congested stage-2 pixel path); 320*cy still spreads y
            -- across the x bits, one crisp dot per lit cell (~0.4%).
            -- The rotate-by-3 term folds ccx(1:0) into the COMPARED bits:
            -- without it the low column bits were don't-cares and every
            -- star came as a 4-cell horizontal RUN (dim flashing dashes)
            v_sa := s_rowb xor resize(v_ccx, 16);
            v_sh := v_sa(9 downto 0) xor v_sa(15 downto 6)
                    xor (v_sa(2 downto 0) & v_sa(9 downto 3));
            if v_sh < to_unsigned(4, 10)
               and s_pfx = 0 and s_pfy = 0 and v_in then
                s_star2 <= '1';
            else
                s_star2 <= '0';
            end if;
            -- slow twinkle: a quarter of the stars runs dim, selected from
            -- row+column-MIXED bits. v_sa(1:0) is just the column residue
            -- (row base is a multiple of 4), so the old selector dimmed
            -- column classes in left-to-right order as the frame counter
            -- advanced -- the whole starfield appeared to march rightward.
            if (v_sa(7 downto 6) xor s_frame_cnt(6 downto 5)) = "11" then
                s_stwk2 <= '1';
            else
                s_stwk2 <= '0';
            end if;
        end if;
    end process p_addr;

    -- ====================================================================
    -- stage 3: canvas port arbiter -- decay sweep (blanking) / prefetch
    -- ====================================================================
    p_canvas_ctrl : process(clk)
        variable v_sl   : unsigned(9 downto 0);
        variable v_new  : unsigned(1 downto 0);
        variable v_hash : unsigned(9 downto 0);
        variable v_rd   : boolean;
    begin
        if rising_edge(clk) then
            s_cv_we <= '0';
            s_decay_rd2  <= s_decay_rd;
            s_decay_k_d2 <= s_decay_k_d1;
            -- sweep end = last displayed row + 1 (cy*320 + 319); full
            -- canvas on 180-row rasters, ~19.5k cells on an NTSC field
            if s_ch2 >= to_unsigned(C_CH - 1, 10) then
                s_sw_last <= to_unsigned(C_CELLS - 1, 16);
            else
                s_sw_last <= shift_left(resize(s_ch2, 16), 8)
                             + shift_left(resize(s_ch2, 16), 6)
                             + to_unsigned(C_CW - 1, 16);
            end if;

            if s_ftick = '1' then
                -- Bright cells (2/3) always step down every sweep, so sparks
                -- that stop being replotted collapse in ~2 frames (crisp);
                -- only val-1 cells (last embers) linger on the slow dithered
                -- path. K5 = TAIL persistence: thr 326 (K5=0, ~3 frames,
                -- clean dots) .. 8 (K5 max, ~2 s willow streaks).
                v_sl := to_unsigned(1023, 10) - unsigned(registers_in(4));
                s_decay_thr <= to_unsigned(8, 10)
                               + resize(shift_right(v_sl, 2), 10)
                               + resize(shift_right(v_sl, 4), 10);
                -- fresh random dither offset per frame: XOR, not +frame --
                -- the old hash+frame pattern translated across the screen
                -- (fading content appeared to scroll sideways)
                s_dith <= s_lfsr(9 downto 0);
                s_decay_active <= '1';
                s_decay_done   <= '0';
                s_decay_k      <= (others => '0');
            end if;

            -- ---- read port ----
            v_rd := (s_decay_active = '1') and (s_decay_done = '0')
                    and (data_in.avid = '0') and (s_avid_sr(0) = '0') and (s_avid_sr(1) = '0');
            if v_rd then
                s_cv_raddr   <= s_decay_k;
                s_decay_k_d1 <= s_decay_k;
                if s_decay_k >= s_sw_last then
                    s_decay_done <= '1';
                else
                    s_decay_k <= s_decay_k + 1;
                end if;
                s_decay_rd <= '1';
                s_tag1 <= "00";
            else
                s_decay_rd <= '0';
                if s_ftop2 = '1' then
                    s_cv_raddr <= s_addr_top2;
                    s_tag1 <= "01";
                elsif s_fbot2 = '1' then
                    s_cv_raddr <= s_addr_bot2;
                    s_tag1 <= "10";
                else
                    s_cv_raddr <= s_addr_top2;
                    s_tag1 <= "00";
                end if;
            end if;

            -- ---- write port (2-clock tap: rdata belongs to k_d2) ----
            if s_decay_rd2 = '1' then
                if s_cv_rdata >= 2 then
                    v_new := s_cv_rdata - 1;
                elsif s_cv_rdata = 1 then
                    v_hash := (s_decay_k_d2(9 downto 0) xor s_decay_k_d2(15 downto 6))
                              xor s_dith;
                    if v_hash < s_decay_thr then
                        v_new := (others => '0');
                    else
                        v_new := "01";
                    end if;
                else
                    v_new := (others => '0');
                end if;
                s_cv_we    <= '1';
                s_cv_waddr <= s_decay_k_d2;
                s_cv_wdata <= v_new;
                -- (a tail write landing on the tick must not cancel the
                -- new sweep the tick just started)
                if s_decay_done = '1' and s_ftick = '0' then
                    s_decay_active <= '0';
                end if;
            elsif s_decay_active = '0' and s_pen_we = '1' then
                s_cv_we    <= '1';
                s_cv_waddr <= s_pen_waddr;
                s_cv_wdata <= s_pen_wdata;
            end if;
        end if;
    end process p_canvas_ctrl;

    -- ==== canvas BRAM (1W1R, c64 p_buf pattern) ====
    p_canvas : process(clk)
    begin
        if rising_edge(clk) then
            if s_cv_we = '1' then
                s_canvas(to_integer(s_cv_waddr)) <= s_cv_wdata;
            end if;
            s_cv_rdata <= s_canvas(to_integer(s_cv_raddr));
        end if;
    end process p_canvas;

    -- ==== nebula ROM (2 EBR): free-running per-pixel read; the 1-2 px
    -- lag against the color pipeline is a fixed shift of a wrapping
    -- texture (starfield trick) ====
    p_neb : process(clk)
    begin
        if rising_edge(clk) then
            s_neb_q <= s_neb(to_integer(s_y_count(8 downto 4)
                                        & s_x_count(8 downto 4)));
        end if;
    end process p_neb;

    -- ====================================================================
    -- Prefetch captures + span-boundary commit (tag pipeline).
    -- ====================================================================
    p_fetch : process(clk)
    begin
        if rising_edge(clk) then
            s_tag2 <= s_tag1;
            s_tag3 <= s_tag2;
            s_tag4 <= s_tag3;
            s_tag5 <= s_tag4;
            if s_tag2 = "01" then
                s_TRc <= s_cv_rdata;
            elsif s_tag2 = "10" then
                s_BRc <= s_cv_rdata;
            end if;
            -- commit at the span boundary: 6-px spans need 2 more clocks or
            -- the rotate lands mid-span (visible seam every cell)
            if (s_xdiv6 = '0' and s_tag3 = "10")
               or (s_xdiv6 = '1' and s_tag5 = "10") then
                s_TLb <= s_TRb;  s_BLb <= s_BRb;
                s_TRb <= s_TRc;  s_BRb <= s_BRc;
            end if;
        end if;
    end process p_fetch;

    -- ====================================================================
    -- stages 3-5: bilinear blend + nearest-shell attribution
    -- ====================================================================
    p_blend : process(clk)
        variable v_wxl : unsigned(3 downto 0);
        variable v_wyl : unsigned(3 downto 0);
        variable v_dx  : signed(9 downto 0);
        variable v_dy  : signed(8 downto 0);
        variable v_ax  : unsigned(8 downto 0);
        variable v_ay  : unsigned(8 downto 0);
        variable v_es  : signed(10 downto 0);
        variable v_rx  : signed(9 downto 0);
        variable v_wad, v_wbd : unsigned(7 downto 0);
        variable v_wai, v_wbi : std_logic;
    begin
        if rising_edge(clk) then
            -- pipes
            s_fx3 <= s_fx2;
            s_fy3 <= s_fy2;   s_fy4 <= s_fy3;
            s_insq3 <= s_insq2;  s_insq4 <= s_insq3;  s_insq5 <= s_insq4;
            s_insq6 <= s_insq5;
            s_star3 <= s_star2;  s_star4 <= s_star3;  s_star5 <= s_star4;
            s_star6 <= s_star5;
            s_stwk3 <= s_stwk2;  s_stwk4 <= s_stwk3;  s_stwk5 <= s_stwk4;
            s_stwk6 <= s_stwk5;
            s_mir3 <= s_mir2;    s_mir4 <= s_mir3;    s_mir5 <= s_mir4;
            s_mir6 <= s_mir5;
            s_cvok3 <= s_cvok2;  s_cvok4 <= s_cvok3;  s_cvok5 <= s_cvok4;
            s_cvok6 <= s_cvok5;
            s_dbank3 <= s_dbank2;
            if s_hd_on = '1' then
                if s_dbank3 = '0' then
                    s_hl4 <= s_lb0_q;
                else
                    s_hl4 <= s_lb1_q;
                end if;
            else
                s_hl4 <= '0';
            end if;
            s_hl5 <= s_hl4;
            s_hl6 <= s_hl5;
            s_t6 <= s_t5;

            -- stage 3->4: horizontal lerp (blend regs are span-stable)
            v_wxl := "1000" - resize(s_fx3, 4);
            s_h0 <= resize(s_TLb * v_wxl, 5) + resize(s_TRb * s_fx3, 5);
            s_h1 <= resize(s_BLb * v_wxl, 5) + resize(s_BRb * s_fx3, 5);

            -- stage 4->5: vertical lerp -> t (0..192)
            v_wyl := "1000" - resize(s_fy4, 4);
            s_t5 <= resize(s_h0 * v_wyl, 8) + resize(s_h1 * s_fy4, 8);

            -- stage 3 (time-multiplexed, one slot per clock): |dx|,|dy| ->
            -- max/min; plus that slot's white launch column test
            s_tk3 <= s_tk2;  s_tv3 <= s_tv2;  s_trs3 <= s_trs2;
            s_ten3 <= s_ten2;  s_nc3 <= s_nc2;
            v_rx := signed(resize(s_tcx2, 10)) - signed(resize(s_trx2, 10))
                    + to_signed(2, 10);
            if s_trk2 = '1' and v_rx >= 0 and v_rx <= to_signed(4, 10)
               and resize(s_tcy2, 9) + 1 >= resize(s_try2, 9) then
                s_trkh3 <= '1';
            else
                s_trkh3 <= '0';
            end if;
            v_dx := signed(resize(s_tcx2, 10)) - signed(resize(s_tax2, 10));
            v_dy := signed(resize(s_tcy2, 9))  - signed(resize(s_tay2, 9));
            if v_dx >= 0 then v_ax := resize(unsigned(v_dx), 9);
            else              v_ax := resize(unsigned(-v_dx), 9); end if;
            if v_dy >= 0 then v_ay := resize(unsigned(v_dy(8 downto 0)), 9);
            else              v_ay := resize(unsigned(-v_dy), 9); end if;
            if v_ax >= v_ay then
                s_tmax3 <= v_ax;  s_tmin3 <= v_ay;
            else
                s_tmax3 <= v_ay;  s_tmin3 <= v_ax;
            end if;

            -- stage 4: raw octagonal distance (max + min/2)
            s_td4 <= resize(s_tmax3, 10) + resize(s_tmin3(8 downto 1), 10);
            s_tk4 <= s_tk3;  s_tv4 <= s_tv3;  s_trs4 <= s_trs3;
            s_ten4 <= s_ten3;  s_trkh4 <= s_trkh3;  s_nc4 <= s_nc3;

            -- stage 5: commit the finished cell's bank as a new cell's first
            -- pixel arrives (that pixel's stage-6 argmin is the first to read
            -- it), then write this slot's result for the cell after.
            -- Weighted distance = distance minus the shell's radius (2x split
            -- radius), offset +128, clamped to 8 bits: inside its own ball a
            -- shell scores below 128, and where two balls overlap the one the
            -- pixel is DEEPER inside wins.
            if s_nc4 = '1' then
                s_ac   <= s_ar;
                s_acrk <= s_arrk;
            end if;
            if s_tv4 = '1' then
                v_es := signed(resize(s_td4, 11))
                        - signed(resize(s_trs4 & '0', 11))
                        + to_signed(128, 11);
                if s_ten4 = '0' then
                    s_ar(to_integer(s_tk4)) <= (others => '1');
                elsif v_es < 0 then
                    s_ar(to_integer(s_tk4)) <= (others => '0');
                elsif v_es > to_signed(255, 11) then
                    s_ar(to_integer(s_tk4)) <= (others => '1');
                else
                    s_ar(to_integer(s_tk4)) <= unsigned(v_es(7 downto 0));
                end if;
                s_arrk(to_integer(s_tk4)) <= s_trkh4;
            end if;

            -- stage 6: 4-way argmin over the committed cell's distances
            if s_ac(0) <= s_ac(1) then
                v_wad := s_ac(0);  v_wai := '0';
            else
                v_wad := s_ac(1);  v_wai := '1';
            end if;
            if s_ac(2) <= s_ac(3) then
                v_wbd := s_ac(2);  v_wbi := '0';
            else
                v_wbd := s_ac(3);  v_wbi := '1';
            end if;
            if v_wbd < v_wad then
                s_cidx6 <= "01" & v_wbi;
            else
                s_cidx6 <= "00" & v_wai;
            end if;
            if s_acrk /= "0000" then
                s_rk6 <= '1';
            else
                s_rk6 <= '0';
            end if;
        end if;
    end process p_blend;

    -- ====================================================================
    -- Attribution registers: latched from slot state once per frame
    -- ====================================================================
    p_attr : process(clk)
    begin
        if rising_edge(clk) then
            if s_ftick = '1' then
                for k in 0 to 3 loop
                    s_aen(k) <= s_sl_ever(k);
                    -- A shell owns its colour region only while bursting
                    -- (centre and radius follow the drift and expansion).
                    -- Through idle AND the slot's next ascent the region
                    -- HOLDS, so the old burst's fading embers keep their
                    -- colour instead of being re-claimed by the nearest
                    -- other shell when the slot relaunches. Look-ahead: an
                    -- ascent at age 63 bursts this frame, at its apogee.
                    if s_sl_state(k) = "10" then
                        s_acx(k)  <= s_sl_cx(k);
                        s_acy(k)  <= s_sl_cy(k);
                        s_arsp(k) <= s_sl_rsp(k);
                        s_aci(k)  <= s_sl_col(k);
                        s_apas(k) <= s_sl_pas(k);
                    elsif s_sl_state(k) = "01"
                          and s_sl_age(k)(5 downto 0) = "111111" then
                        s_acx(k)  <= s_sl_cx(k);
                        s_acy(k)  <= s_sl_apy(k);
                        s_arsp(k) <= to_unsigned(2, 7);
                        s_aci(k)  <= s_sl_col(k);
                        s_apas(k) <= s_sl_pas(k);
                    end if;
                    -- the rocket itself: a white column under its head
                    -- (independent of the held colour region)
                    if s_sl_state(k) = "01" then
                        s_rkon(k) <= '1';
                    else
                        s_rkon(k) <= '0';
                    end if;
                    s_rkx(k) <= s_sl_cx(k);
                    s_rky(k) <= s_sl_cy(k);
                end loop;
            end if;
        end if;
    end process p_attr;

    -- ====================================================================
    -- Shared sin/cos (combinational) + shared multiplier
    -- ====================================================================
    -- ONE folded sine tree; cos is read by re-aiming the angle +256 for a
    -- cycle (the Q schedule already has the state for it)
    s_lut_sin <= f_qsin(unsigned(s_lut_angle));

    p_mul : process(clk)
    begin
        if rising_edge(clk) then
            s_mul_p <= s_mul_a * s_mul_b;       -- signed(7:0) * signed(9:0)
        end if;
    end process p_mul;

    -- free-running seed source
    p_lfsr : process(clk)
    begin
        if rising_edge(clk) then
            s_lfsr <= s_lfsr(14 downto 0)
                      & (s_lfsr(15) xor s_lfsr(14) xor s_lfsr(12) xor s_lfsr(3));
        end if;
    end process p_lfsr;

    -- ====================================================================
    -- Particle engine: slot lifecycle + memoryless per-frame replot.
    -- Operands set in state k are seen by p_mul during k+1, so the product
    -- is readable in state k+2 (spiroscope schedule).
    -- ====================================================================
    p_engine : process(clk)
        variable v_rv   : unsigned(5 downto 0);
        variable v_vf   : unsigned(7 downto 0);
        variable v_d7   : unsigned(6 downto 0);
        variable v_idx  : unsigned(6 downto 0);
        variable v_seed : unsigned(15 downto 0);
        variable v_t3   : unsigned(2 downto 0);
        variable v_n7   : unsigned(6 downto 0);
        variable v_hue  : unsigned(9 downto 0);
        variable v_vis  : std_logic;
        variable v_sp   : boolean;
        variable v_tls  : signed(9 downto 0);
        variable v_scr  : signed(9 downto 0);
        variable v_c1   : unsigned(2 downto 0);
        variable v_pas  : std_logic;
        variable v_n6   : unsigned(5 downto 0);
        variable v_apy  : unsigned(7 downto 0);
        variable v_dly  : unsigned(9 downto 0);
        variable v_bl   : unsigned(9 downto 0);
        variable v_d9   : unsigned(8 downto 0);
        variable v_gd   : signed(13 downto 0);
        variable v_gs   : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            s_pl_stb <= '0';
            -- head pixel coordinates, one shared free-running converter
            -- (s_x4/s_y4 settle >= 2 states before any head is issued)
            s_hx <= f_px(s_x4);
            s_hy <= f_px(s_y4);
            -- lifecycle action flags are one-cycle pulses: set in
            -- EST_LIFED, applied (below the FSM case) during EST_LIFE1
            s_lc_sp   <= '0';  s_lc_inc <= '0';  s_lc_boom <= '0';
            s_lc_die  <= '0';  s_lc_sink <= '0'; s_lc_mvr <= '0';
            s_lc_mvl  <= '0';
            s_ht_we   <= '0';

            -- free-running layout regs off the settled cell dims (see p_count)
            s_cwu    <= s_cw2(8 downto 0);
            -- launch pads sit on the waterline when the reflection is on
            if s_sw_water = '1' then
                s_ground <= s_horiz - 2;
            else
                if s_ch2 > to_unsigned(C_CH, 10) then
                    s_ground <= to_unsigned(C_CH - 4, 8);
                else
                    s_ground <= resize(s_ch2 - 4, 8);
                end if;
            end if;
            s_apbase <= resize(shift_right(s_ch2, 2), 8) - 12;

            -- slider into a plain register (never SPI carry chains in a cone)
            s_sldr <= unsigned(registers_in(7));
            -- pre-registered spawn clamp bound (the spawn-x adder+clamp
            -- chain is the hd_hdmi critical path -- keep it shallow)
            s_cwm32 <= signed(resize(s_cwu, 11)) - to_signed(32, 11);
            -- drift pacing, decoded free-running (the frame counter only
            -- changes at the tick)
            if (s_gravK /= 0 and s_frame_cnt(2 downto 0) = "000")
               or (s_gravK(5) = '1' and s_frame_cnt(2 downto 0) = "100") then
                s_pc_sink <= '1';
            else
                s_pc_sink <= '0';
            end if;
            if s_frame_cnt(2 downto 0) = "010" then
                s_pc_mom8 <= '1';
            else
                s_pc_mom8 <= '0';
            end if;
            if s_frame_cnt(3 downto 0) = "0110" then
                s_pc_mom16 <= '1';
            else
                s_pc_mom16 <= '0';
            end if;
            -- pads at 1/8, 3/8, 5/8, 7/8 of the usable width
            s_pad(0) <= resize(shift_right(s_cw2, 3), 9);
            s_pad(1) <= resize(shift_right(s_cw2, 2), 9)
                        + resize(shift_right(s_cw2, 3), 9);
            s_pad(2) <= resize(shift_right(s_cw2, 1), 9)
                        + resize(shift_right(s_cw2, 3), 9);
            s_pad(3) <= resize(s_cw2, 9) - resize(shift_right(s_cw2, 3), 9);

            -- ---------------- work FSM ----------------
            case s_est is

                when EST_IDLE =>
                    if s_go = '1' then
                        s_go  <= '0';
                        s_ei  <= 0;
                        s_est <= EST_SLOT;
                    end if;

                when EST_SLOT =>
                    -- pen writes are dropped while the decay sweep owns the
                    -- write port: hold off dispatch until it finishes
                    if s_decay_active = '0' then
                        if s_ei = 4 then
                            s_est <= EST_IDLE;
                            -- head table final for this frame
                            s_hn   <= s_hcnt;
                            s_hrdy <= '1';
                        else
                            s_est <= EST_LIFE;
                        end if;
                    end if;

                -- ============ per-frame lifecycle of slot s_ei ============
                -- hoist (slot -> plain regs) -> decide (registered 1-bit
                -- action flags + one-hot slot) -> apply (below the case:
                -- shared data, per-slot CEN = flag AND one-hot)
                when EST_LIFE =>
                    -- pre-add the respawn x (pad + seed jitter); EST_LIFED clamps
                    s_lc_st   <= s_sl_state(s_ei);
                    s_lc_age  <= s_sl_age(s_ei);
                    if s_sl_age(s_ei) >= s_sl_delay(s_ei) then
                        s_lc_exp <= '1';
                    else
                        s_lc_exp <= '0';
                    end if;
                    s_lc_seed <= s_sl_seed(s_ei);
                    s_lc_cx   <= s_sl_cx(s_ei);
                    s_lc_cy   <= s_sl_cy(s_ei);
                    s_lc_apy  <= s_sl_apy(s_ei);
                    v_seed := s_lfsr xor (x"1D" & to_unsigned(s_ei mod 4, 2)
                                          & s_frame_cnt(5 downto 0));
                    s_lc_ns <= v_seed;
                    s_lc_bx <= signed(resize(s_pad(s_ei), 11))
                               + resize(signed('0' & v_seed(5 downto 0)), 11)
                               - to_signed(32, 11);
                    s_est <= EST_LIFED;

                when EST_LIFED =>
                    -- 32-cell margin: near-edge shells lost half their
                    -- sparks off-canvas ("cut off" look)
                    if s_lc_bx < to_signed(32, 11) then
                        s_lc_cxs <= to_unsigned(32, 9);
                    elsif s_lc_bx > s_cwm32 then
                        s_lc_cxs <= unsigned(s_cwm32(8 downto 0));
                    else
                        s_lc_cxs <= unsigned(s_lc_bx(8 downto 0));
                    end if;
                    for k in 0 to 3 loop
                        if k = s_ei then
                            s_lc_oh(k) <= '1';
                        else
                            s_lc_oh(k) <= '0';
                        end if;
                    end loop;
                    -- age / spawn / transition (halted by FREEZE; the replot
                    -- keeps running from the frozen ages)
                    if s_freeze = '0' then
                        case s_lc_st is
                            when "00" =>   -- idle: wait out the relaunch delay
                                if s_lc_exp = '1' then
                                    s_lc_sp <= '1';
                                else
                                    s_lc_inc <= '1';
                                end if;
                            when "01" =>   -- launch: 64-frame ascent
                                if s_lc_age = 63 then
                                    s_lc_boom <= '1';
                                else
                                    s_lc_inc <= '1';
                                end if;
                            when "10" =>   -- burst: 128-frame bloom and fade
                                if s_lc_age = 127 then
                                    s_lc_die <= '1';
                                else
                                    s_lc_inc <= '1';
                                    -- the whole cloud DRIFTS as it fades:
                                    -- paced integer steps of the base center
                                    -- (sparks, core and attribution follow;
                                    -- FREEZE halts it). Sink: 1 cell / 8
                                    -- frames, doubled on the heavy half of
                                    -- GRAVITY; none at 0
                                    if s_pc_sink = '1'
                                       and s_lc_cy < to_unsigned(C_CH - 4, 8) then
                                        s_lc_sink <= '1';
                                    end if;
                                    -- sideways momentum: seed-picked
                                    -- direction, 1 cell / 8 or / 16 frames
                                    if (s_lc_seed(2) = '1' and s_pc_mom8 = '1')
                                       or (s_lc_seed(2) = '0'
                                           and s_pc_mom16 = '1') then
                                        if s_lc_seed(1) = '1' then
                                            if signed(resize(s_lc_cx, 11)) < s_cwm32 then
                                                s_lc_mvr <= '1';
                                            end if;
                                        else
                                            if s_lc_cx > to_unsigned(32, 9) then
                                                s_lc_mvl <= '1';
                                            end if;
                                        end if;
                                    end if;
                                end if;
                            when others => -- unreachable: recover to idle
                                s_lc_die <= '1';
                        end case;
                    end if;
                    s_est <= EST_LIFE1;

                when EST_LIFE1 =>
                    -- the decided actions apply this cycle (below the case)
                    s_est <= EST_LIFE2;

                when EST_LIFE2 =>
                    -- idle slots (embers fading) skip the replot
                    if s_sl_state(s_ei) = "00" then
                        if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                        s_est <= EST_SLOT;
                    else
                        s_est <= EST_AGE;
                    end if;

                when EST_AGE =>
                    -- unified timeline: launch 0..63, burst 64..191, offset
                    -- by the scrub (pre-latched; +64 folded into s_scr_b)
                    if s_sl_state(s_ei) = "10" then
                        s_tl <= signed(resize(s_sl_age(s_ei)(7 downto 0), 10))
                                + s_scr_b;
                    else
                        s_tl <= signed(resize(s_sl_age(s_ei)(7 downto 0), 10))
                                + s_scr_r;
                    end if;
                    if s_freeze = '0' and s_sl_state(s_ei) = "01" then
                        s_ctrack <= '1';
                    else
                        s_ctrack <= '0';
                    end if;
                    s_est <= EST_AGE2;
                when EST_AGE2 =>
                    -- clamp to 0..191 and split into phase + phase-local age
                    -- (low-bit slices, NOT signed resize -- signed resize
                    -- keeps the sign bit and drops bit 6 of ages >= 64)
                    v_tls := s_tl - to_signed(64, 10);
                    if s_tl >= to_signed(64, 10) then
                        s_cphase <= '1';
                        if s_tl >= to_signed(192, 10) then
                            s_cage <= to_unsigned(127, 7);
                        else
                            s_cage <= unsigned(v_tls(6 downto 0));
                        end if;
                        s_est <= EST_T0;
                    else
                        s_cphase <= '0';
                        if s_tl < to_signed(0, 10) then
                            s_cage <= (others => '0');
                        else
                            s_cage <= unsigned(s_tl(6 downto 0));
                        end if;
                        s_est <= EST_T0;
                    end if;

                -- ============ physics curves for this pass (was a ROM) ============
                -- curve index now / last frame: burst = cage, rocket = 2*cage
                -- (64-frame ascent over the 128-entry curve)
                when EST_T0 =>
                    if s_cphase = '1' then
                        s_tic <= s_cage;
                        if s_cage = 0 then
                            s_tip <= (others => '0');
                        else
                            s_tip <= s_cage - 1;
                        end if;
                    else
                        s_tic <= s_cage(5 downto 0) & '0';
                        if s_cage = 0 then
                            s_tip <= (others => '0');
                        else
                            s_tip <= (s_cage(5 downto 0) - 1) & '0';
                        end if;
                    end if;
                    s_est <= EST_T0 + 1;
                when EST_T0 + 1 =>   -- (127 - i) is just NOT i on 7 bits
                    s_mul_a <= signed('0' & not s_tic);
                    s_mul_b <= signed(resize(not s_tic, 10));
                    s_est <= EST_T0 + 2;
                when EST_T0 + 2 =>
                    s_mul_a <= signed('0' & s_tic);
                    s_mul_b <= signed(resize(s_tic, 10));
                    s_est <= EST_T0 + 3;
                when EST_T0 + 3 =>
                    s_eC <= to_unsigned(126, 7) - unsigned(s_mul_p(13 downto 7));
                    s_mul_a <= signed('0' & not s_tip);
                    s_mul_b <= signed(resize(not s_tip, 10));
                    s_est <= EST_T0 + 4;
                when EST_T0 + 4 =>
                    s_dC <= unsigned(s_mul_p(13 downto 7));
                    s_mul_a <= signed('0' & s_tip);
                    s_mul_b <= signed(resize(s_tip, 10));
                    s_est <= EST_T0 + 5;
                when EST_T0 + 5 =>
                    s_eP <= to_unsigned(126, 7) - unsigned(s_mul_p(13 downto 7));
                    s_est <= EST_T0 + 6;
                when EST_T0 + 6 =>
                    s_dP <= unsigned(s_mul_p(13 downto 7));
                    if s_cphase = '1' then
                        s_est <= EST_P0;
                    else
                        s_est <= EST_R0;
                    end if;

                -- ============ ROCKET (launch phase, one streak/frame) ============
                when EST_R0 =>
                    -- ascent height, registered so the multiplier operand mux
                    -- reads a plain register (not slot-array mux + subtract)
                    s_hsc <= s_ground - s_sl_apy(s_ei);
                    s_ccx <= s_sl_cx(s_ei);
                    -- apex flash bloom: bright plus-head for the last frames
                    -- of the ascent (registered here, used in the issue state)
                    if s_cage >= to_unsigned(60, 7) then
                        s_apx <= '1';
                    else
                        s_apx <= '0';
                    end if;
                    s_est <= EST_R0 + 1;
                when EST_R0 + 1 =>
                    -- sway: one sine cycle over the 64-frame ascent
                    s_lut_angle <= std_logic_vector(s_cage(5 downto 0)) & "0000";
                    s_est <= EST_R0 + 2;
                when EST_R0 + 2 =>
                    s_sinP <= s_lut_sin;
                    s_mul_a <= signed('0' & s_eC);                        -- ease[idx]
                    s_mul_b <= signed(resize(s_hsc, 10));
                    s_est <= EST_R0 + 3;
                when EST_R0 + 3 =>
                    s_mul_a <= signed('0' & s_eP);                        -- ease[idx-1]
                    s_mul_b <= signed(resize(s_hsc, 10));
                    s_est <= EST_R0 + 4;
                when EST_R0 + 4 =>
                    -- p = ease[idx]*height -> current rocket y (rise can reach
                    -- 138, so take 8 product bits, not 7)
                    s_yc <= signed(resize(s_ground, 11)) - resize(signed('0' & s_mul_p(14 downto 7)), 11);
                    s_xc <= signed(resize(s_ccx, 11))
                            + resize(shift_right(s_sinP, 8), 11);
                    s_y4 <= signed(shift_left(resize(s_ground, 13), 2))
                            - signed(resize(unsigned(s_mul_p(14 downto 5)), 13));
                    s_x4 <= signed(shift_left(resize(s_ccx, 13), 2))
                            + resize(shift_right(s_sinP, 6), 13);
                    s_est <= EST_R0 + 5;
                when EST_R0 + 5 =>
                    -- p = ease[idx-1]*height -> previous rocket y
                    s_yp <= signed(resize(s_ground, 11)) - resize(signed('0' & s_mul_p(14 downto 7)), 11);
                    s_xp <= s_xc;
                    s_est <= EST_R0 + 6;
                when EST_R0 + 6 =>
                    -- (issue gates check s_pl_stb too: s_w_active lags an
                    -- accepted strobe by one clock, so stb='1' means busy)
                    if s_w_active = '0' and s_pl_stb = '0' then
                        s_pl_fx  <= unsigned(s_xp(8 downto 0));
                        s_pl_fy  <= unsigned(s_yp(7 downto 0));
                        s_pl_tx  <= unsigned(s_xc(8 downto 0));
                        s_pl_ty  <= unsigned(s_yc(7 downto 0));
                        s_pl_val <= "11";
                        s_pl_head <= s_apx;
                        s_pl_stb <= '1';
                        if s_hcnt(8) = '0' then
                            s_ht_we <= '1';
                            s_ht_wa <= s_hcnt(7 downto 0);
                            s_ht_wd <= '0' & s_apx & "000" & std_logic_vector(s_hy)
                                       & "00000" & std_logic_vector(s_hx);
                            s_hcnt  <= s_hcnt + 1;
                        end if;
                        -- track attribution center up the ascent (only for a
                        -- genuinely ascending slot -- never during scrub)
                        if s_ctrack = '1' then
                            s_sl_cy(s_ei) <= unsigned(s_yc(7 downto 0));
                        end if;
                        if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                        s_est <= EST_SLOT;
                    end if;

                -- ============ BURST prep: shared radius/droop for the shell ============
                when EST_P0 =>
                    s_ccx   <= s_sl_cx(s_ei);
                    s_ccy   <= s_sl_cy(s_ei);
                    s_ctyp  <= s_sl_typ(s_ei);
                    s_cseed <= s_sl_seed(s_ei);
                    s_cn    <= s_sl_n(s_ei);
                    v_idx := s_cage;
                    if v_idx >= 112 then
                        s_val_pre <= "01";
                    elsif v_idx >= 88 then
                        s_val_pre <= "10";
                    else
                        s_val_pre <= "11";
                    end if;
                    if s_sl_typ(s_ei) = C_TY_PALM or s_sl_typ(s_ei) = C_TY_CHRYS
                       or s_sl_typ(s_ei) = C_TY_COMET or v_idx < 6 then
                        s_head_pre <= '1';
                    else
                        s_head_pre <= '0';
                    end if;
                    -- K6 SPARK: shimmer from level 1, crackle strobe at level 3
                    if s_sparkK /= "00" and v_idx >= 80 then
                        s_twk_en <= '1';
                    else
                        s_twk_en <= '0';
                    end if;
                    if s_sparkK = "11" and v_idx >= 32 then
                        s_crkl_en <= '1';
                    else
                        s_crkl_en <= '0';
                    end if;
                    -- bright persistent core at the shell center
                    if v_idx < 80 then
                        s_core_on <= '1';  s_core_val <= "11";
                    elsif v_idx < 112 then
                        s_core_on <= '1';  s_core_val <= "10";
                    else
                        s_core_on <= '0';
                    end if;
                    s_est <= EST_P0 + 1;
                when EST_P0 + 1 =>
                    s_est <= EST_P0 + 2;
                when EST_P0 + 2 =>
                    s_droopC <= resize(s_dC, 8);
                    s_mul_a <= signed('0' & s_eC);                         -- ease[idx]
                    s_mul_b <= signed(resize(s_sizeK, 10));
                    s_est <= EST_P0 + 3;
                when EST_P0 + 3 =>
                    s_easeP  <= resize(s_eP, 8);
                    s_droopP <= resize(s_dP, 8);
                    s_mul_a <= signed(resize(s_droopC, 8));
                    s_mul_b <= signed(resize(s_gravK, 10));
                    s_est <= EST_P0 + 4;
                when EST_P0 + 4 =>
                    s_rbaseC <= unsigned(s_mul_p(13 downto 7));           -- ease*size
                    -- color-split radius = half the current shell radius
                    s_sl_rsp(s_ei) <= resize(unsigned(s_mul_p(13 downto 8)), 7);
                    s_mul_a <= signed(resize(s_easeP, 8));
                    s_mul_b <= signed(resize(s_sizeK, 10));
                    s_est <= EST_P0 + 5;
                when EST_P0 + 5 =>
                    v_d7 := unsigned(s_mul_p(13 downto 7));               -- droop*grav
                    if s_ctyp = C_TY_WILLOW then
                        s_dreffC <= v_d7(5 downto 0) & '0';               -- x2
                    elsif s_ctyp = C_TY_PALM then
                        s_dreffC <= v_d7 + resize(v_d7(6 downto 1), 7);   -- x1.5
                    elsif s_ctyp = C_TY_COMET then
                        s_dreffC <= '0' & v_d7(6 downto 1);               -- x0.5
                    else
                        s_dreffC <= v_d7;
                    end if;
                    -- the same droop in quarter cells for the sharp head
                    v_d9 := unsigned(s_mul_p(13 downto 5));
                    if s_ctyp = C_TY_WILLOW then
                        s_dfC4 <= v_d9 & '0';
                    elsif s_ctyp = C_TY_PALM then
                        s_dfC4 <= resize(v_d9, 10) + resize(v_d9(8 downto 1), 10);
                    elsif s_ctyp = C_TY_COMET then
                        s_dfC4 <= resize(v_d9(8 downto 1), 10);
                    else
                        s_dfC4 <= resize(v_d9, 10);
                    end if;
                    s_mul_a <= signed(resize(s_droopP, 8));
                    s_mul_b <= signed(resize(s_gravK, 10));
                    s_est <= EST_P0 + 6;
                when EST_P0 + 6 =>
                    s_rbaseP <= unsigned(s_mul_p(13 downto 7));
                    s_cyd4   <= shift_left(resize(s_ccy, 12), 2) + resize(s_dfC4, 12);
                    s_est <= EST_P0 + 7;
                when EST_P0 + 7 =>
                    v_d7 := unsigned(s_mul_p(13 downto 7));
                    if s_ctyp = C_TY_WILLOW then
                        s_dreffP <= v_d7(5 downto 0) & '0';
                    elsif s_ctyp = C_TY_PALM then
                        s_dreffP <= v_d7 + resize(v_d7(6 downto 1), 7);
                    elsif s_ctyp = C_TY_COMET then
                        s_dreffP <= '0' & v_d7(6 downto 1);
                    else
                        s_dreffP <= v_d7;
                    end if;
                    s_pi  <= (others => '0');
                    -- core head: the shell centre
                    s_x4 <= signed(shift_left(resize(s_ccx, 13), 2));
                    s_y4 <= signed(shift_left(resize(s_ccy, 13), 2));
                    s_est <= EST_PC;

                -- ============ persistent core: bright plus at the center ============
                when EST_PC =>
                    if s_core_on = '0' then
                        s_est <= EST_Q0;
                    elsif s_w_active = '0' and s_pl_stb = '0' then
                        s_pl_fx  <= s_ccx;
                        s_pl_fy  <= s_ccy;
                        s_pl_tx  <= s_ccx;
                        s_pl_ty  <= s_ccy;
                        s_pl_val <= s_core_val;
                        s_pl_head <= '1';
                        s_pl_stb <= '1';
                        if s_core_val = "11" and s_hcnt(8) = '0' then
                            s_ht_we <= '1';
                            s_ht_wa <= s_hcnt(7 downto 0);
                            s_ht_wd <= "01000" & std_logic_vector(s_hy)
                                       & "00000" & std_logic_vector(s_hx);
                            s_hcnt  <= s_hcnt + 1;
                        end if;
                        s_est <= EST_Q0;
                    end if;

                -- ============ one spark: recompute now and last frame, walk ============
                -- Q0 does ONLY the hash: hash -> angle/vfrac/table math chained
                -- in one state was the 57 MHz post-route critical path.
                when EST_Q0 =>
                    s_h <= f_hash(s_cseed, s_pi);
                    -- pre-register the loop-exit compare (slot-array mux +
                    -- subtract is too deep for the issue state's enable cone)
                    if s_pi = s_cn - 1 then
                        s_lastp <= '1';
                    else
                        s_lastp <= '0';
                    end if;
                    s_est <= EST_Q0 + 1;
                when EST_Q0 + 1 =>
                    -- register vfrac; feeding the type-case adder straight into
                    -- the multiplier operand mux was the next critical path.
                    -- ANGLE: bit-REVERSED index in the top bits -- i0 flips
                    -- sides every spark, i1 quadrants, so any spark count
                    -- covers the full circle (the raw hash put i(5:4) in the
                    -- top angle bits: below 48 sparks bursts drew 1/4..3/4
                    -- ARCS). Speed then uses the PLAIN index so angle/speed
                    -- stay decorrelated (rev-for-both = visible spiral).
                    s_ang8 <= (f_rev6(s_pi) xor s_cseed(15 downto 10))
                              & s_h(3 downto 2);
                    v_rv := s_pi xor s_cseed(14 downto 9);
                    case s_ctyp is
                        when C_TY_RING =>
                            v_vf := to_unsigned(120, 8) + resize(s_h(2 downto 0), 8);
                        when C_TY_WILLOW =>
                            v_vf := to_unsigned(72, 8) + resize(v_rv(5 downto 1), 8);
                        when C_TY_PALM =>
                            v_vf := to_unsigned(112, 8) + resize(s_h(12 downto 10), 8);
                        when C_TY_CHRYS =>   -- tight trailing shell of comets
                            v_vf := to_unsigned(88, 8) + resize(v_rv(4 downto 0), 8);
                        when C_TY_COMET =>   -- max-speed straight streaks
                            v_vf := to_unsigned(122, 8) + resize(s_h(2 downto 1), 8);
                        when others =>       -- peony: filled sphere
                            v_vf := to_unsigned(64, 8) + resize(v_rv, 8);
                    end case;
                    s_vfrac <= v_vf;
                    s_est <= EST_Q0 + 2;
                when EST_Q0 + 2 =>
                    -- aim the shared tree at the spark angle (registered at
                    -- Q0+1) -> sin next state
                    s_lut_angle <= std_logic_vector(s_ang8) & "00";
                    s_mul_a <= signed(resize(s_rbaseC, 8));
                    s_mul_b <= signed(resize(s_vfrac, 10));
                    s_est <= EST_Q0 + 3;
                when EST_Q0 + 3 =>
                    s_sinP <= s_lut_sin;
                    -- re-aim the shared tree at angle+256 -> cos next state
                    s_lut_angle <= std_logic_vector(s_ang8 + to_unsigned(64, 8)) & "00";
                    s_mul_a <= signed(resize(s_rbaseP, 8));
                    s_mul_b <= signed(resize(s_vfrac, 10));
                    s_est <= EST_Q0 + 4;
                when EST_Q0 + 4 =>
                    s_cosP <= s_lut_sin;                      -- sin(a+256) = cos(a)
                    s_rc <= unsigned(s_mul_p(13 downto 7));               -- r now
                    s_rf <= unsigned(s_mul_p(6 downto 5));                -- + r/4 bits
                    s_est <= EST_Q0 + 5;
                when EST_Q0 + 5 =>
                    s_riseP <= unsigned(s_mul_p(13 downto 7));            -- r last frame
                    s_mul_a <= signed(resize(s_rc, 8));
                    s_mul_b <= s_cosP;
                    -- radius fraction x (cos, sin) for the head's sub-cell
                    -- position (rf = 0..3 quarter cells: shift-adds)
                    case s_rf is
                        when "00" =>
                            s_rfx <= (others => '0');
                            s_rfy <= (others => '0');
                        when "01" =>
                            s_rfx <= resize(s_cosP, 12);
                            s_rfy <= resize(s_sinP, 12);
                        when "10" =>
                            s_rfx <= shift_left(resize(s_cosP, 12), 1);
                            s_rfy <= shift_left(resize(s_sinP, 12), 1);
                        when others =>
                            s_rfx <= resize(s_cosP, 12) + shift_left(resize(s_cosP, 12), 1);
                            s_rfy <= resize(s_sinP, 12) + shift_left(resize(s_sinP, 12), 1);
                    end case;
                    s_est <= EST_Q0 + 6;
                when EST_Q0 + 6 =>
                    s_mul_a <= signed(resize(s_rc, 8));
                    s_mul_b <= s_sinP;
                    s_est <= EST_Q0 + 7;
                when EST_Q0 + 7 =>
                    s_xc <= signed(resize(s_ccx, 11))
                            + resize(shift_right(s_mul_p, 9), 11);        -- rc*cos
                    s_x4 <= signed(shift_left(resize(s_ccx, 13), 2))
                            + resize(shift_right(s_mul_p, 7), 13)
                            + resize(shift_right(s_rfx, 9), 13);
                    s_mul_a <= signed(resize(s_riseP, 8));                -- (r prev)
                    s_mul_b <= s_cosP;
                    s_est <= EST_Q0 + 8;
                when EST_Q0 + 8 =>
                    s_yc <= signed(resize(s_ccy, 11))
                            + resize(shift_right(s_mul_p, 9), 11)         -- rc*sin
                            + signed(resize(s_dreffC, 11));
                    s_y4 <= signed(resize(s_cyd4, 13))
                            + resize(shift_right(s_mul_p, 7), 13)
                            + resize(shift_right(s_rfy, 9), 13);
                    s_mul_a <= signed(resize(s_riseP, 8));
                    s_mul_b <= s_sinP;
                    s_est <= EST_Q0 + 9;
                when EST_Q0 + 9 =>
                    s_xp <= signed(resize(s_ccx, 11))
                            + resize(shift_right(s_mul_p, 9), 11);        -- rp*cos
                    s_est <= EST_Q0 + 10;
                when EST_Q0 + 10 =>
                    s_yp <= signed(resize(s_ccy, 11))
                            + resize(shift_right(s_mul_p, 9), 11)         -- rp*sin
                            + signed(resize(s_dreffP, 11));
                    -- volumetric tie: slow inner sparks stay bright a dim-tier
                    -- longer (center of the sphere reads denser/hotter)
                    if s_val_pre = "10" and s_vfrac < to_unsigned(96, 8) then
                        s_val <= "11";
                    else
                        s_val <= s_val_pre;
                    end if;
                    -- BLAST climax zone: a rising share of sparks gets the
                    -- bold plus head (index-cycled hash bits = even spread)
                    if s_head_pre = '1' or s_h(9 downto 6) < s_hboost then
                        s_head <= '1';
                    else
                        s_head <= '0';
                    end if;
                    v_vis := '1';
                    if s_crkl_en = '1'
                       and (s_h(1 downto 0) xor s_frame_cnt(1 downto 0)) >= 2 then
                        v_vis := '0';    -- K6 max: 50% duty crackle strobe
                    end if;
                    if s_twk_en = '1'
                       and (s_h(2 downto 0) xor s_frame_cnt(2 downto 0)) >= s_twk_thr then
                        v_vis := '0';    -- late-life shimmer (duty from K6)
                    end if;
                    s_vis <= v_vis;
                    s_est <= EST_Q0 + 11;
                when EST_Q0 + 11 =>
                    -- register the 8-comparator range check: computed inside
                    -- the issue state it gated the plot registers' CEN cone
                    if (s_xc > 0) and (s_xc < 319) and (s_yc >= 0) and (s_yc <= C_CH - 3)
                       and (s_xp > 0) and (s_xp < 319) and (s_yp >= 0) and (s_yp <= C_CH - 3) then
                        s_ok <= '1';
                    else
                        s_ok <= '0';
                    end if;
                    s_est <= EST_Q0 + 12;
                when EST_Q0 + 12 =>
                    if s_ok = '0' or s_vis = '0' then
                        -- spark off-screen or gated dark this frame
                        if s_lastp = '1' then
                            if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                            s_est <= EST_SLOT;
                        else
                            s_pi  <= s_pi + 1;
                            s_est <= EST_Q0;
                        end if;
                    elsif s_w_active = '0' and s_pl_stb = '0' then
                        s_pl_fx  <= unsigned(s_xp(8 downto 0));
                        s_pl_fy  <= unsigned(s_yp(7 downto 0));
                        s_pl_tx  <= unsigned(s_xc(8 downto 0));
                        s_pl_ty  <= unsigned(s_yc(7 downto 0));
                        s_pl_val <= s_val;
                        s_pl_head <= s_head;
                        s_pl_stb <= '1';
                        -- sharp head while the spark still burns bright
                        if s_val = "11" and s_hcnt(8) = '0' then
                            s_ht_we <= '1';
                            s_ht_wa <= s_hcnt(7 downto 0);
                            s_ht_wd <= '0' & s_head & "000" & std_logic_vector(s_hy)
                                       & "00000" & std_logic_vector(s_hx);
                            s_hcnt  <= s_hcnt + 1;
                        end if;
                        if s_lastp = '1' then
                            if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                            s_est <= EST_SLOT;
                        else
                            s_pi  <= s_pi + 1;
                            s_est <= EST_Q0;
                        end if;
                    end if;

                when others =>
                    s_est <= EST_IDLE;
            end case;

            -- ---------------- lifecycle apply (EST_LIFE1 cycle) ----------------
            -- spawn data is computed ONCE from the registered respawn seed;
            -- each slot only muxes it in under flag AND one-hot
            v_seed := s_lc_ns;
            -- type: S7 bank pick; S8 mixes in ring shells; comets salt both
            -- banks (~1 in 4)
            if s_ringOn = '1' and v_seed(2 downto 1) = "11" then
                v_t3 := C_TY_RING;
            elsif v_seed(8 downto 7) = "11" then
                v_t3 := C_TY_COMET;
            elsif s_bankB = '0' then
                if v_seed(6) = '0' then v_t3 := C_TY_PEONY;
                else v_t3 := C_TY_CHRYS; end if;
            else
                if v_seed(6) = '0' then v_t3 := C_TY_WILLOW;
                else v_t3 := C_TY_PALM; end if;
            end if;
            -- K4 HUE bank picks the shell's ONE colour
            case s_hueSel is
                when "000" =>   -- Warm: white / yellow (gold) / orange
                    case v_seed(12 downto 11) is
                        when "00"   => v_c1 := "111";
                        when "10"   => v_c1 := "001";
                        when others => v_c1 := "010";
                    end case;
                when "001" =>   -- Primaries: red / green / blue
                    case v_seed(12 downto 11) is
                        when "01"   => v_c1 := "011";
                        when "10"   => v_c1 := "100";
                        when others => v_c1 := "000";
                    end case;
                when "011" =>   -- Red-White-Blue
                    case v_seed(12 downto 11) is
                        when "00"   => v_c1 := "000";
                        when "01"   => v_c1 := "111";
                        when others => v_c1 := "100";
                    end case;
                when others =>  -- Pastel / Rainbow: any of the 8
                    v_c1 := v_seed(13 downto 11);
            end case;
            -- pastel bank is all-pastel; the Random bank salts in pastel
            -- shells (~1 in 4) for more color variety
            if s_hueSel = "010"
               or (s_hueSel = "100" and v_seed(10 downto 9) = "11") then
                v_pas := '1';
            else
                v_pas := '0';
            end if;
            -- spark count (palm/comet: few thick arms)
            if v_t3 = C_TY_PALM then
                v_n6 := to_unsigned(10, 6);
            elsif v_t3 = C_TY_COMET then
                v_n6 := to_unsigned(14, 6);
            else
                v_n7 := to_unsigned(10, 7) + resize(s_dens, 7);
                if v_n7 > 60 then v_n7 := to_unsigned(60, 7); end if;
                v_n6 := v_n7(5 downto 0);
            end if;
            v_apy := s_apbase + resize(v_seed(10 downto 6), 8);
            -- relaunch delay (FINALE: tight jitter -> overlapping salvos)
            if s_sw_finale = '1' then
                v_dly := s_rateD + resize(s_lc_seed(2 downto 0), 10);
            else
                v_dly := s_rateD + resize(s_lc_seed(4 downto 0), 10);
            end if;
            for k in 0 to 3 loop
                if s_lc_oh(k) = '1' then
                    if s_lc_sp = '1' then
                        s_sl_seed(k)  <= v_seed;
                        s_sl_state(k) <= "01";
                        s_sl_age(k)   <= (others => '0');
                        s_sl_ever(k)  <= '1';
                        s_sl_cx(k)    <= s_lc_cxs;
                        s_sl_cy(k)    <= s_ground;
                        s_sl_apy(k)   <= v_apy;
                        s_sl_typ(k)   <= v_t3;
                        s_sl_col(k)   <= v_c1;
                        s_sl_pas(k)   <= v_pas;
                        s_sl_rsp(k)   <= to_unsigned(2, 7);
                        s_sl_n(k)     <= v_n6;
                    end if;
                    if s_lc_inc = '1' then
                        s_sl_age(k) <= s_lc_age + 1;
                    end if;
                    if s_lc_boom = '1' then
                        s_sl_state(k) <= "10";
                        s_sl_age(k)   <= (others => '0');
                        s_sl_cy(k)    <= s_lc_apy;
                    end if;
                    if s_lc_die = '1' then
                        s_sl_state(k) <= "00";
                        s_sl_age(k)   <= (others => '0');
                        s_sl_delay(k) <= v_dly;
                    end if;
                    if s_lc_sink = '1' then
                        s_sl_cy(k) <= s_lc_cy + 1;
                    end if;
                    if s_lc_mvr = '1' then
                        s_sl_cx(k) <= s_lc_cx + 1;
                    end if;
                    if s_lc_mvl = '1' then
                        s_sl_cx(k) <= s_lc_cx - 1;
                    end if;
                end if;
            end loop;
            if s_lc_boom = '1' then
                s_boom <= '1';
            end if;

            -- ------------- per-frame tick: latch controls, restart replot -------------
            -- (the slot lifecycle itself runs per slot in EST_LIFE*)
            if s_ftick = '1' then
                -- P12 BLAST (glided, see below): live scale of every burst,
                -- 8..102 cells (slider 800 = the old K2-max 83); shells
                -- scale down when the cell world is short (1080i fields:
                -- 6x6 cells -> only ~180/3 rows usable)
                v_bl := s_bla(12 downto 3);
                if s_short = '1' then
                    s_sizeK <= to_unsigned(4, 7) + resize(v_bl(9 downto 5), 7)
                               + resize(v_bl(9 downto 6), 7);
                else
                    s_sizeK <= to_unsigned(8, 7) + resize(v_bl(9 downto 4), 7)
                               + resize(v_bl(9 downto 5), 7);
                end if;
                -- climax zone = top eighth of the travel (896..1023)
                if v_bl(9 downto 7) = "111" then
                    s_hboost <= v_bl(6 downto 3);
                    s_bltop  <= '1';
                else
                    s_hboost <= (others => '0');
                    s_bltop  <= '0';
                end if;
                -- K3 GRAVITY (unsigned: the negative "embers rise" zone was
                -- cut for LC/congestion -- see the cut list)
                s_gravK <= unsigned(registers_in(2)(9 downto 4));
                -- K1 LAUNCH: rate + full-CCW manual-only silence;
                -- FINALE quarters the relaunch delay (4x barrage rate)
                if s_sw_finale = '1' then
                    s_rateD <= to_unsigned(12, 10)
                               + resize(shift_right(to_unsigned(1023, 10)
                                                    - unsigned(registers_in(0)), 3), 10);
                else
                    s_rateD <= to_unsigned(24, 10)
                               + resize(shift_right(to_unsigned(1023, 10)
                                                    - unsigned(registers_in(0)), 1), 10);
                end if;
                -- K2 SPARKS: sparks per shell (the slider owns the size)
                s_dens  <= unsigned(registers_in(1)(9 downto 4));
                -- K4 HUE: 5 palette-bank zones
                v_hue := unsigned(registers_in(3));
                -- at program load the firmware writes the labelled default's
                -- INDEX (0..4), not a knob position: read small values as it
                if v_hue < 5 then  s_hueSel <= v_hue(2 downto 0);
                elsif v_hue < 205 then s_hueSel <= "000";   -- Gold
                elsif v_hue < 410 then s_hueSel <= "001";   -- Primaries
                elsif v_hue < 615 then s_hueSel <= "010";   -- Pastel
                elsif v_hue < 820 then s_hueSel <= "011";   -- Red-Wht-Blue
                else                   s_hueSel <= "100";   -- Random
                end if;
                -- K6 SPARK: 4 levels (off / shimmer / heavy shimmer / crackle)
                s_sparkK <= unsigned(registers_in(5)(9 downto 8));
                case registers_in(5)(9 downto 8) is
                    when "01"   => s_twk_thr <= "110";      -- 25% dark
                    when "10"   => s_twk_thr <= "101";      -- 37% dark
                    when others => s_twk_thr <= "100";      -- 50% dark
                end case;
                s_bankB     <= registers_in(6)(0);
                s_ringOn    <= registers_in(6)(1);
                s_sw_water  <= registers_in(6)(2);
                s_sw_finale <= registers_in(6)(3);
                s_freeze    <= registers_in(6)(4);

                -- P12. Outside FREEZE: BLAST amount, glided (diff/8 per
                -- frame) and held while frozen. Inside FREEZE: timeline scrub
                -- RELATIVE to where the slider sat at the freeze edge (+-255
                -- frames), so flipping FREEZE holds the exact frame; +64 is
                -- pre-folded for burst slots
                if s_freeze = '0' then
                    v_gd := signed(resize(s_sldr & "000", 14))
                            - signed(resize(s_bla, 14));
                    -- (slice, not resize: a signed narrowing would drop
                    -- bit 12 and fold the top half of the slider)
                    v_gs  := signed(resize(s_bla, 14)) + shift_right(v_gd, 3);
                    s_bla <= unsigned(v_gs(12 downto 0));
                    s_scr_lat <= s_sldr;
                end if;
                if s_freeze = '1' then
                    v_scr := resize(shift_right(signed(resize(s_sldr, 11))
                                                - signed(resize(s_scr_lat, 11)), 2), 10);
                else
                    v_scr := (others => '0');
                end if;
                s_scr_r <= v_scr;
                s_scr_b <= v_scr + to_signed(64, 10);

                -- ambient sky flash: reload on any detonation of the last
                -- frame, fade per frame (FINALE or the BLAST climax zone:
                -- full-strength reload; FINALE: half-rate fade)
                s_boom <= '0';
                if s_boom = '1' then
                    if s_sw_finale = '1' or s_bltop = '1' then
                        s_flash <= "11";
                    else
                        s_flash <= "10";
                    end if;
                elsif s_flash /= 0
                      and (s_sw_finale = '0' or s_frame_cnt(0) = '0') then
                    s_flash <= s_flash - 1;
                end if;

                -- restart the replot pass for the new frame (new head table)
                s_go   <= '1';
                s_est  <= EST_IDLE;
                s_hcnt <= (others => '0');
                s_hrdy <= '0';
            end if;
        end if;
    end process p_engine;

    -- ====================================================================
    -- Sharp heads: head table (engine writes, renderer reads), per-line
    -- renderer, two-bank line buffer
    -- ====================================================================
    p_ht : process(clk)
    begin
        if rising_edge(clk) then
            if s_ht_we = '1' then
                s_ht(to_integer(s_ht_wa)) <= s_ht_wd;
            end if;
            s_ht_q <= s_ht(to_integer(s_ht_ra));
        end if;
    end process p_ht;

    -- Renderer: at each active line's start, stamp line y+1's heads into
    -- bank (y+1)&1 (the bank NOT being displayed); during the blanking
    -- before the frame's first line, stamp line 0. Each table entry costs
    -- 5 clocks + its span (<= 5 px), ~1.3k clocks for 250 heads of the
    -- 2200-clock 1080p line. A new line start aborts an unfinished pass.
    p_head : process(clk)
        variable v_dy : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_xdiv6 = '1' and s_ydivsel = "10" then
                s_hd_on <= '1';
            else
                s_hd_on <= '0';
            end if;
            s_rwe <= '0';
            if data_in.avid = '1' and s_avid_sr(0) = '0' then
                s_ri <= (others => '0');
                s_rT <= s_y_count(10 downto 0) + 1;
                if s_hd_on = '1' and s_hrdy = '1' then
                    s_rs <= 1;
                else
                    s_rs <= 0;
                end if;
            elsif data_in.hsync_n = '0' and s_prev_hsync_n = '1' and s_saw = '0' then
                s_ri <= (others => '0');
                s_rT <= (others => '0');
                if s_hd_on = '1' and s_hrdy = '1' then
                    s_rs <= 1;
                else
                    s_rs <= 0;
                end if;
            else
                case s_rs is
                    when 1 =>                       -- issue a table read
                        if s_ri = s_hn then
                            s_rs <= 0;
                        else
                            s_ht_ra <= s_ri(7 downto 0);
                            s_rs <= 2;
                        end if;
                    when 2 =>                       -- EBR read in flight
                        s_rs <= 3;
                    when 3 =>                       -- fabric copy of the entry
                        s_hq2 <= s_ht_q;
                        s_rs <= 4;
                    when 4 =>                       -- does this head cover line T?
                        v_dy := signed('0' & s_rT) - signed('0' & s_hq2(26 downto 16));
                        s_rhit <= '0';
                        if v_dy = 0 then
                            s_rhit <= '1';
                            if s_hq2(30) = '1' then s_rw <= "10"; else s_rw <= "01"; end if;
                        elsif v_dy = 1 or v_dy = -1 then
                            s_rhit <= '1';
                            if s_hq2(30) = '1' then s_rw <= "01"; else s_rw <= "00"; end if;
                        elsif (v_dy = 2 or v_dy = -2) and s_hq2(30) = '1' then
                            s_rhit <= '1';
                            s_rw <= "00";
                        end if;
                        s_rs <= 5;
                    when 5 =>                       -- span start, or next entry
                        if s_rhit = '1' then
                            s_rx <= unsigned(s_hq2(10 downto 0)) - resize(s_rw, 11);
                            s_rn <= resize(s_rw & '0', 3);
                            s_rs <= 6;
                        else
                            s_ri <= s_ri + 1;
                            s_rs <= 1;
                        end if;
                    when 6 =>                       -- stamp one pixel per clock
                        s_rwe <= '1';
                        s_rwa <= s_rx;
                        s_rwb <= s_rT(0);
                        if s_rn = 0 then
                            s_ri <= s_ri + 1;
                            s_rs <= 1;
                        else
                            s_rx <= s_rx + 1;
                            s_rn <= s_rn - 1;
                        end if;
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_head;

    -- line-buffer write ports: the displayed bank is cleared one pixel
    -- behind the beam (never the address being read); the other bank
    -- takes the renderer's stamps
    p_lbw : process(clk)
    begin
        if rising_edge(clk) then
            s_lbc_a <= s_lb_ra;
            s_lbc_e <= s_lb_act2;
            s_lbc_b <= s_dbank2;
            if s_lbc_e = '1' and s_lbc_b = '0' then
                s_lb0_we <= '1';  s_lb0_wa <= s_lbc_a;  s_lb0_wd <= '0';
            elsif s_rwe = '1' and s_rwb = '0' then
                s_lb0_we <= '1';  s_lb0_wa <= s_rwa;    s_lb0_wd <= '1';
            else
                s_lb0_we <= '0';
            end if;
            if s_lbc_e = '1' and s_lbc_b = '1' then
                s_lb1_we <= '1';  s_lb1_wa <= s_lbc_a;  s_lb1_wd <= '0';
            elsif s_rwe = '1' and s_rwb = '1' then
                s_lb1_we <= '1';  s_lb1_wa <= s_rwa;    s_lb1_wd <= '1';
            else
                s_lb1_we <= '0';
            end if;
        end if;
    end process p_lbw;

    p_lb0 : process(clk)
    begin
        if rising_edge(clk) then
            if s_lb0_we = '1' then
                s_lb0(to_integer(s_lb0_wa)) <= s_lb0_wd;
            end if;
            s_lb0_q <= s_lb0(to_integer(s_lb_ra));
        end if;
    end process p_lb0;

    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            if s_lb1_we = '1' then
                s_lb1(to_integer(s_lb1_wa)) <= s_lb1_wd;
            end if;
            s_lb1_q <= s_lb1(to_integer(s_lb_ra));
        end if;
    end process p_lb1;

    -- ====================================================================
    -- Pen writer: 8-connected line-walker with optional plus-shaped head.
    -- Walks from 'from' (exclusive) to 'to' (inclusive); from == to plots
    -- a single dot. Step cap keeps a bad request from hogging the frame.
    -- ====================================================================
    p_writer : process(clk)
        variable v_wake : unsigned(1 downto 0);
        variable v_off  : unsigned(15 downto 0);
        variable v_ha   : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s_pen_we <= '0';
            -- wake value: one brightness step behind the head, so every
            -- particle leaves a dimmer trail along its own motion vector
            if s_w_val = "11" then
                v_wake := "10";
            elsif s_w_val = "10" then
                v_wake := "01";
            else
                v_wake := s_w_val;
            end if;
            -- plus-head arm address: head cell -1 / +1 / -320 / +320
            case s_w_step is
                when 2      => v_off := x"FFFF";
                when 3      => v_off := x"0001";
                when 4      => v_off := to_unsigned(65536 - C_CW, 16);
                when others => v_off := to_unsigned(C_CW, 16);
            end case;
            v_ha := s_wk_a + v_off;
            if s_w_active = '0' then
                if s_pl_stb = '1' then
                    s_wk_x   <= s_pl_fx;
                    s_wk_y   <= s_pl_fy;
                    s_wk_tx  <= s_pl_tx;
                    s_wk_ty  <= s_pl_ty;
                    s_w_val  <= s_pl_val;
                    s_w_head <= s_pl_head;
                    s_w_cnt  <= (others => '0');
                    s_w_step <= 0;
                    s_w_active <= '1';
                end if;
            else
                case s_w_step is
                    when 0 =>      -- move one cell toward the target
                        if s_wk_x < s_wk_tx then s_wk_x <= s_wk_x + 1;
                        elsif s_wk_x > s_wk_tx then s_wk_x <= s_wk_x - 1; end if;
                        if s_wk_y < s_wk_ty then s_wk_y <= s_wk_y + 1;
                        elsif s_wk_y > s_wk_ty then s_wk_y <= s_wk_y - 1; end if;
                        s_w_step <= 1;
                    when 1 =>      -- write the cell (head bright, wake dim)
                        s_pen_we    <= '1';
                        s_pen_waddr <= f_addr(s_wk_y, s_wk_x);
                        s_wk_a      <= f_addr(s_wk_y, s_wk_x);
                        if (s_wk_x = s_wk_tx and s_wk_y = s_wk_ty)
                           or s_w_cnt = to_unsigned(24, 5) then
                            s_pen_wdata <= s_w_val;
                            if s_w_head = '1' then
                                s_w_step <= 2;
                            else
                                s_w_active <= '0';
                            end if;
                        else
                            s_pen_wdata <= v_wake;
                            s_w_cnt  <= s_w_cnt + 1;
                            s_w_step <= 0;
                        end if;
                    when 2 =>      -- plus-shaped head around the target
                        if s_wk_x > 0 then
                            s_pen_we    <= '1';
                            s_pen_wdata <= s_w_val;
                            s_pen_waddr <= v_ha;
                        end if;
                        s_w_step <= 3;
                    when 3 =>
                        if s_wk_x < to_unsigned(C_CW - 1, 9) then
                            s_pen_we    <= '1';
                            s_pen_wdata <= s_w_val;
                            s_pen_waddr <= v_ha;
                        end if;
                        s_w_step <= 4;
                    when 4 =>
                        if s_wk_y > 0 then
                            s_pen_we    <= '1';
                            s_pen_wdata <= s_w_val;
                            s_pen_waddr <= v_ha;
                        end if;
                        s_w_step <= 5;
                    when others =>
                        if s_wk_y < to_unsigned(C_CH - 1, 8) then
                            s_pen_we    <= '1';
                            s_pen_wdata <= s_w_val;
                            s_pen_waddr <= v_ha;
                        end if;
                        s_w_active <= '0';
                        s_w_step   <= 0;
                end case;
            end if;
        end if;
    end process p_writer;

    -- ====================================================================
    -- stage 6: t -> luma (t*21, max 1008); nearest-shell chroma
    -- ====================================================================
    p_color : process(clk)
        variable v_t  : unsigned(7 downto 0);
        variable v_y  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 6a: resolve the owning color index (argmin -> pastel
            -- bit; rockets' launch column is white) and pipe everything
            -- else one stage
            if s_rk6 = '1' then
                s_ci7 <= "0111";
            else
                s_ci7 <= s_apas(to_integer(s_cidx6)) & s_aci(to_integer(s_cidx6));
            end if;
            s_t7    <= s_t6;
            s_star7 <= s_star6;
            s_stwk7 <= s_stwk6;
            s_mir7  <= s_mir6;
            s_insq7 <= s_insq6;
            s_cvok7 <= s_cvok6;
            s_hl7   <= s_hl6;

            -- stage 6b: luma = t*4 (t <= 192 -> max 768), per-color cap,
            -- mirror attenuation, background branches
            v_t := s_t7;
            v_y := resize(v_t & "00", 11);
            if s_hl7 = '1' and s_insq7 = '1' and s_mir7 = '0' then
                -- sharp head: a hot core in the shell's colour at near-full
                -- luma over the soft glow
                s_d_y <= to_unsigned(768, 10);
                s_d_u <= C_PAL_U(to_integer(s_ci7));
                s_d_v <= C_PAL_V(to_integer(s_ci7));
            elsif s_insq7 = '1' and s_cvok7 = '1' and v_t /= 0 then
                -- per-color luma ceiling keeps hues saturated at full glow
                if v_y > resize(C_PAL_YC(to_integer(s_ci7)), 11) then
                    v_y := resize(C_PAL_YC(to_integer(s_ci7)), 11);
                end if;
                -- mirrored copies dim to 5/8 and shift blue
                if s_mir7 = '1' then
                    s_d_y <= resize(v_y(9 downto 1), 10)
                             + resize(v_y(9 downto 3), 10);
                else
                    s_d_y <= v_y(9 downto 0);
                end if;
                s_d_u <= C_PAL_U(to_integer(s_ci7));
                if s_mir7 = '1' then
                    s_d_v <= C_PAL_V(to_integer(s_ci7)) + to_unsigned(32, 10);
                else
                    s_d_v <= C_PAL_V(to_integer(s_ci7));
                end if;
            elsif s_insq7 = '1' and s_star7 = '1' then
                -- static star: crisp dot, cool white, slow twinkle;
                -- reflected stars shimmer at reduced brightness
                if s_stwk7 = '1' then
                    s_d_y <= to_unsigned(200, 10);
                elsif s_mir7 = '1' then
                    s_d_y <= to_unsigned(180, 10);
                else
                    s_d_y <= to_unsigned(300, 10);
                end if;
                s_d_u <= to_unsigned(504, 10);      -- stored U = Cr: cool
                s_d_v <= to_unsigned(524, 10);      -- stored V = Cb: bluish
            elsif s_insq7 = '1' and s_mir7 = '1' then
                -- open water: flat, dark, deep blue; half the sky flash
                s_d_y <= s_wf;
                s_d_u <= to_unsigned(494, 10);
                s_d_v <= to_unsigned(548, 10);
            elsif s_insq7 = '1' then
                -- night sky: blue gradient + nebula clouds + ambient flash
                -- (gradient+flash pre-composited at line rate: one add here)
                s_d_y <= s_bgf + resize(s_neb_q & "00", 10);
                s_d_u <= to_unsigned(500, 10);
                s_d_v <= to_unsigned(534, 10);
            else
                s_d_y <= to_unsigned(64, 10);
                s_d_u <= to_unsigned(512, 10);
                s_d_v <= to_unsigned(512, 10);
            end if;
            -- blanking gate: neutral wherever the (generated) output avid
            -- is low -- the canvas window alone stayed open from avid fall
            -- to hsync on rasters that don't end exactly on cell 320 (SD,
            -- or an analog line one pixel short). Top priority so it maps
            -- onto the output FFs' sync set/reset, not the colour cone.
            if s_gav_sr(C_LATENCY - 2) = '0' then
                s_d_y <= to_unsigned(64, 10);
                s_d_u <= to_unsigned(512, 10);
                s_d_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_color;

    -- ====================================================================
    -- sync delay (match pipeline depth; output is fully synthesized)
    -- ====================================================================
    p_sr : process(clk)
    begin
        if rising_edge(clk) then
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            s_gav_sr(0)   <= g_avid;
            for i in 1 to C_LATENCY - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
                s_gav_sr(i)   <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_sr;

    -- ====================================================================
    -- Generated avid (see declarations; aurora p_gavid verbatim). g_ph is
    -- the hsync->avid-rise distance parity of this line, g_par the voted
    -- usual parity; an off-parity line starts one clock later, so the
    -- generated start always sits at the same hsync parity. It ends after
    -- the width latched per field, or at hsync at the latest. The window
    -- trails the content by 3-4 px (invisible: a 1080p line keeps 1916+).
    -- ====================================================================
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;    -- hsync_n falling
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;       -- avid rising
            if s_ftick = '1' then
                g_wm1 <= s_line_width - 1;
            end if;

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

            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
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

    data_out.hsync_n <= s_hsync_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_LATENCY - 1);
    data_out.avid    <= s_gav_sr(C_LATENCY - 1);

    data_out.y <= std_logic_vector(s_d_y);
    data_out.u <= std_logic_vector(s_d_u);
    data_out.v <= std_logic_vector(s_d_v);

end architecture pyro;
