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
-- shimmer wobble), hand-fired shells on the slider, FINALE barrage mode,
-- and FREEZE + time scrub (memoryless replot = age is scrubbable, so
-- bursts collapse back into rising shells).
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
-- readout: each pixel takes the palette color of the NEAREST shell center
-- (octagonal distance, 5 candidates, pipelined argmin) -- zero extra BRAM.
-- Rockets show gold during ascent; the slot switches to its shell color at
-- burst. Dead shells keep claiming their embers until the slot relaunches.
--
-- Control map (PYRO):
--   K1 (reg0) LAUNCH  rate; <32 = manual-only silence
--   K2 (reg1) SIZE    burst radius + sparks per shell
--   K3 (reg2) GRAVITY heavy fast-falling .. floaty near-zero-g
--   K4 (reg3) HUE     Gold/Primaries/Pastel/Red-Wht-Blue/Random banks
--   K5 (reg4) TAIL    trail persistence (decay threshold)
--   K6 (reg5) SPARK   0..3: off / shimmer / heavy shimmer / +crackle strobe
--   S7 (reg6.0) TYPE   bank A peony/chrys vs bank B willow/palm
--   S8 (reg6.1) RING   mixes in ring shells
--   S9 (reg6.2) WATER  reflection below the horizon
--   S10(reg6.3) FINALE barrage choreography (4x rate, tight salvos)
--   S11(reg6.4) FREEZE physics halt (slider becomes time scrub)
--   P12(reg7)   FIRE   hand-fly a shell / detonate on release / scrub
--
-- Inherited fireworks passes: two-tone shells (inner/outer color split at
-- half the shell radius), bright persistent core, crisp-spark decay (bright
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
    constant C_CH      : integer := 180;   -- canvas height (cells)
    constant C_USHIFT  : integer := 2;     -- 4 screen px per cell
    constant C_VW      : integer := 1280;  -- canvas viewport width  (px)
    constant C_VH      : integer := 720;   -- canvas viewport height (px)
    constant C_LATENCY : integer := 6;     -- input->output register stages
    constant C_CELLS   : integer := C_CW * C_CH;  -- 57600 (29 EBRs at 2 bit)

    constant C_GROUND  : integer := 176;   -- launch pad row (cells)

    -- canvas address = cy*320 + cx (320 = 256+64 -> two shifts + an add)
    function f_addr(cy : unsigned(7 downto 0); cx : unsigned(8 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := resize(cy, 16);
        return shift_left(v, 8) + shift_left(v, 6) + resize(cx, 16);
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

    -- ==== physics ROM: ease (expansion) + droop (gravity), 128 x 16 ====
    -- Curves are stored HALVED (0..126) so every product fits the shared
    -- 8x10 multiplier (spiroscope-size; the first cut's 10x11 missed timing);
    -- consumers shift products >>7 instead of >>8.
    type t_tbl is array(0 to 127) of unsigned(15 downto 0);
    function f_tbl_init return t_tbl is
        variable v    : t_tbl;
        variable e, d : integer;
        variable s    : integer;
    begin
        for i in 0 to 127 loop
            s := 127 - i;
            e := 127 - ((s * s) / 128) - 1;    -- 0 .. 126, decelerating rise
            if e < 0 then e := 0; end if;
            d := (i * i) / 128;                -- 0 .. 126, accelerating fall
            v(i) := to_unsigned(d, 8) & to_unsigned(e, 8);
        end loop;
        return v;
    end function;
    signal s_tbl      : t_tbl := f_tbl_init;
    signal s_tbl_addr : unsigned(6 downto 0) := (others => '0');
    signal s_tbl_q    : unsigned(15 downto 0) := (others => '0');

    -- ==== Timing / pixel counters (c64 pattern) ====
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_x_count      : unsigned(11 downto 0) := (others => '0');
    signal s_y_count      : unsigned(11 downto 0) := (others => '0');
    signal s_line_width   : unsigned(11 downto 0) := to_unsigned(1280, 12);
    signal s_frame_height : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_line_active  : std_logic := '0';
    signal s_frame_cnt    : unsigned(7 downto 0) := (others => '0');

    -- Cell/phase COUNTERS replace all divide/shift rasterization: cells are
    -- 4 px (rasters <= 1280 wide) or 6 px (1080-class: 1920/6 = 320 cells,
    -- 540 field-lines/3 = 180 rows -- the FULL canvas, so 1080i renders at
    -- 6x6 screen px instead of v1.2's blocky 8x8). No division anywhere:
    -- the phase wraps and the cell increments.
    signal s_pfx  : unsigned(2 downto 0) := (others => '0');   -- x sub-phase
    signal s_pcx  : unsigned(8 downto 0) := (others => '0');   -- cell column
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

    -- ==== Readout stage 2 (registered from raw counters) ====
    signal s_insq2 : std_logic := '0';
    signal s_cellx2 : unsigned(8 downto 0) := (others => '0');
    signal s_celly2 : unsigned(7 downto 0) := (others => '0');
    signal s_fx2, s_fy2 : unsigned(2 downto 0) := (others => '0');
    signal s_ftop2, s_fbot2 : std_logic := '0';
    signal s_addr_top2, s_addr_bot2 : unsigned(15 downto 0) := (others => '0');

    -- ==== Nebula cloud layer (starfield technique, scaled to 2 EBR) ====
    -- 64x32 4-bit seamless value-noise tile, 8x16-px texels (512x512 px
    -- wrap), read per pixel and added to the sky gradient. Generated at
    -- elaboration: 2-octave bilinear noise over a wrapping 8x4 lattice.
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
        -- wraps mod (64/2**lg) in x and mod (32/2**lg) in y (seamless)
        variable cx, cy, fx, fy, s : integer;
        variable h00, h10, h01, h11, top, bot : integer;
    begin
        s   := 2 ** lg;
        cx  := x / s;  cy := y / s;
        fx  := x mod s;  fy := y mod s;
        h00 := f_nebh(cx mod (64 / s),       cy mod (32 / s));
        h10 := f_nebh((cx + 1) mod (64 / s), cy mod (32 / s));
        h01 := f_nebh(cx mod (64 / s),       (cy + 1) mod (32 / s));
        h11 := f_nebh((cx + 1) mod (64 / s), (cy + 1) mod (32 / s));
        top := h00 * (s - fx) + h10 * fx;
        bot := h01 * (s - fx) + h11 * fx;
        return (top * (s - fy) + bot * fy) / (s * s);   -- 0..255
    end function;
    type t_neb is array(0 to 2047) of unsigned(3 downto 0);
    function f_neb_init return t_neb is
        variable v : t_neb;
        variable n : integer;
    begin
        for y in 0 to 31 loop
            for x in 0 to 63 loop
                n := (3 * f_nebbi(x, y, 3) + f_nebbi(x * 2, y * 2, 3)) / 4;
                v(y * 64 + x) := to_unsigned(n / 16, 4);
            end loop;
        end loop;
        return v;
    end function;
    signal s_neb   : t_neb := f_neb_init;
    signal s_neb_q : unsigned(3 downto 0) := (others => '0');

    -- ==== Night sky: gradient + hashed static starfield + sky flash ====
    -- star flag decided at stage 2 (one crisp dot per lit cell), piped to 6
    signal s_bgy   : unsigned(7 downto 0) := to_unsigned(40, 8);
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
    signal s_rowin : std_logic := '1';     -- current row inside the canvas
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
    -- col = inner zone / col2 = outer zone (two-tone radial attribution)
    signal s_sl_col   : t_sl_u3  := ("000", "011", "001", "001");
    signal s_sl_col2  : t_sl_u3  := ("010", "000", "001", "001");
    -- current color-split radius (half the shell's expansion radius)
    signal s_sl_rsp   : t_sl_u7  := (others => to_unsigned(12, 7));
    signal s_sl_n     : t_sl_u6  := (others => to_unsigned(24, 6));
    signal s_sl_ever  : std_logic_vector(3 downto 0) := "0011";

    -- raster-scaled layout (recomputed each vsync from the measured raster,
    -- so the show fills the screen on SD rasters too, not just 1280x720)
    signal s_ground : unsigned(7 downto 0) := to_unsigned(176, 8);
    signal s_apbase : unsigned(7 downto 0) := to_unsigned(33, 8);    -- apogee band top
    signal s_cwu    : unsigned(8 downto 0) := to_unsigned(320, 9);   -- usable cells
    signal s_cwm32  : signed(10 downto 0)  := to_signed(288, 11);    -- cwu - 32
    signal s_pad    : t_sl_u9 := (to_unsigned(40, 9),  to_unsigned(120, 9),
                                  to_unsigned(200, 9), to_unsigned(280, 9));

    -- shell types (chrys = peony with forced heads + tight trailing spread;
    -- comet = a few max-speed straight streaks with bright heads and half
    -- gravity -- the two-tone split recolors the trail behind the head)
    constant C_TY_PEONY   : unsigned(2 downto 0) := "000";
    constant C_TY_RING    : unsigned(2 downto 0) := "001";
    constant C_TY_WILLOW  : unsigned(2 downto 0) := "010";
    constant C_TY_PALM    : unsigned(2 downto 0) := "011";
    constant C_TY_CHRYS   : unsigned(2 downto 0) := "100";
    constant C_TY_COMET   : unsigned(2 downto 0) := "101";
    -- per-shell pastel flag (K4 pastel bank -> desaturated palette half)
    signal s_sl_pas  : std_logic_vector(3 downto 0) := (others => '0');

    -- ==== Per-frame latched controls ====
    signal s_sizeK   : unsigned(6 downto 0) := to_unsigned(52, 7);   -- 20..83
    signal s_gravK   : unsigned(5 downto 0) := to_unsigned(23, 6);   -- 0..63
    signal s_rateD   : unsigned(9 downto 0) := to_unsigned(180, 10);
    signal s_silent  : std_logic := '0';                     -- K1 full CCW
    signal s_dens    : unsigned(5 downto 0) := to_unsigned(40, 6);
    signal s_hueSel  : unsigned(2 downto 0) := "000";        -- K4 palette bank
    signal s_sparkK  : unsigned(1 downto 0) := "01";         -- K6 level 0..3
    signal s_twk_thr : unsigned(2 downto 0) := "110";        -- shimmer duty
    signal s_bankB     : std_logic := '0';                   -- S7 willow/palm
    signal s_ringOn    : std_logic := '0';                   -- S8
    signal s_sw_water  : std_logic := '1';                   -- S9
    signal s_sw_finale : std_logic := '0';                   -- S10
    signal s_freeze    : std_logic := '0';                   -- S11

    -- ==== P12 hand-fire: fly a shell by hand, detonate on release ====
    -- 00 idle (armed), 01 flying (slot 3 hand-held), 10 wait re-arm
    signal s_hf_state : unsigned(1 downto 0) := "00";
    signal s_hf_act   : std_logic := '0';
    signal s_hf_cy    : unsigned(7 downto 0) := to_unsigned(176, 8);
    signal s_hf_cyp   : unsigned(7 downto 0) := to_unsigned(176, 8);
    signal s_hf_max   : unsigned(9 downto 0) := (others => '0');
    -- free-running slider decode: the raw SPI register's compare carry
    -- chains were the routed critical path when read inside the vsync
    -- cone -- decode to plain 1-bit flags a clock ahead instead
    signal s_sldr    : unsigned(9 downto 0) := (others => '0');
    signal s_hf_alt  : unsigned(7 downto 0) := to_unsigned(176, 8);
    signal s_sf_go   : std_logic := '0';   -- slider >= 96
    signal s_sf_960  : std_logic := '0';   -- slider >= 960: full throw
    signal s_sf_64   : std_logic := '0';   -- slider < 64 (re-arm)
    signal s_sf_8    : std_logic := '0';   -- slider < 8 (blackout zone)
    signal s_sf_rel  : std_logic := '0';   -- slider dropped 48 below peak
    signal s_sf_gtm  : std_logic := '0';   -- slider above recorded peak
    -- LAUNCH at zero + slider at bottom = fade to black (P12 convention)
    signal s_blk      : std_logic := '0';

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

    signal s_est : integer range 0 to 32 := EST_IDLE;

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

    -- ==== Palette (U/V swapped for HW: stored U = Cr, stored V = Cb) ====
    -- 0 Red, 1 Gold, 2 Yellow, 3 Green, 4 Blue, 5 Indigo, 6 Violet, 7 White;
    -- entries 8..15 are the pastel bank (chroma halved toward 512) so the
    -- pastel flag is just index bit 3 -- no extra mux depth in stage 6
    type t_pal16 is array(0 to 15) of unsigned(9 downto 0);
    constant C_PAL_U : t_pal16 := (
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10), to_unsigned(136, 10),
        to_unsigned(440, 10), to_unsigned(608, 10), to_unsigned(756, 10), to_unsigned(512, 10),
        to_unsigned(736, 10), to_unsigned(642, 10), to_unsigned(548, 10), to_unsigned(324, 10),
        to_unsigned(476, 10), to_unsigned(560, 10), to_unsigned(634, 10), to_unsigned(512, 10));
    constant C_PAL_V : t_pal16 := (
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10), to_unsigned(216, 10),
        to_unsigned(960, 10), to_unsigned(696, 10), to_unsigned(852, 10), to_unsigned(512, 10),
        to_unsigned(436, 10), to_unsigned(362, 10), to_unsigned(288, 10), to_unsigned(364, 10),
        to_unsigned(736, 10), to_unsigned(604, 10), to_unsigned(682, 10), to_unsigned(512, 10));
    -- ember cool-down: warm hues cool within family (white/yellow->gold,
    -- gold->red); cool hues KEEP their identity (green/indigo/violet hold,
    -- blue deepens to indigo) -- cooling everything toward red/gold read
    -- as a single color wash over the whole show
    type t_map8 is array(0 to 7) of unsigned(2 downto 0);
    constant C_EMBER : t_map8 := ("000", "000", "001", "011", "101", "101", "110", "001");

    -- ==== Nearest-shell color attribution ====
    type t_at_u9  is array(0 to 3) of unsigned(8 downto 0);
    type t_at_u8  is array(0 to 4) of unsigned(7 downto 0);
    signal s_acx : t_at_u9 := (others => to_unsigned(160, 9));
    signal s_acy : t_at_u8 := (others => to_unsigned(90, 8));
    signal s_aen : std_logic_vector(3 downto 0) := "0000";
    -- 3-bit palette indices (chroma resolved in stage 6 via the constant
    -- palette, spiroscope-style: 20x 10-bit color regs was ~250 LC)
    type t_at_u3 is array(0 to 3) of unsigned(2 downto 0);
    signal s_aci   : t_at_u3 := (others => "001");
    signal s_aco   : t_at_u3 := (others => "001");
    signal s_apas  : std_logic_vector(3 downto 0) := (others => '0');
    type t_at_u7 is array(0 to 3) of unsigned(6 downto 0);
    signal s_arsp : t_at_u7 := (others => (others => '0'));
    -- stage 3: per-slot max/min of |dx|,|dy|
    signal s_amax : t_at_u9 := (others => (others => '0'));
    signal s_amin : t_at_u9 := (others => (others => '0'));
    -- stage 4: raw octagonal distance + inner/outer zone flag
    type t_at_u10a is array(0 to 3) of unsigned(9 downto 0);
    signal s_draw4 : t_at_u10a := (others => (others => '1'));
    signal s_far4  : std_logic_vector(3 downto 0) := (others => '0');
    -- stage 5: RADIUS-WEIGHTED distance (raw - 2*split_radius, clamped to
    -- 8 bits -- beyond 255 cells ownership is arbitrary, and the narrower
    -- compares keep the stage-6 argmin off the critical path):
    -- an active shell owns its own ball of sparks, so overlapping shells no
    -- longer carve each other into Voronoi color patches
    type t_at_u8a is array(0 to 3) of unsigned(7 downto 0);
    signal s_deff  : t_at_u8a := (others => (others => '1'));
    signal s_far5v : std_logic_vector(3 downto 0) := (others => '0');
    -- stage 6: final color index + far flag
    signal s_cidx6 : unsigned(2 downto 0) := "000";
    signal s_far6  : std_logic := '0';
    signal s_t6    : unsigned(7 downto 0) := (others => '0');

    -- ==== Colour / output (stage 6) ====
    signal s_d_y   : unsigned(9 downto 0) := (others => '0');
    signal s_d_u   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_d_v   : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ==== sync delay SR (output is fully synthesized: no video delay) ====
    type t_bit_sr  is array(0 to C_LATENCY - 1) of std_logic;
    signal s_hsync_sr, s_vsync_sr, s_field_sr, s_avid_sr : t_bit_sr := (others => '0');

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
                if s_line_active = '1' then
                    s_y_count     <= s_y_count + 1;
                    s_line_active <= '0';
                    if s_pcy < to_unsigned(C_CH, 8) then
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

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_y_count > 0 then s_frame_height <= s_y_count; end if;
                s_y_count     <= (others => '0');
                s_x_count     <= (others => '0');
                if s_pcy > 0 then s_chm <= s_pcy; end if;
                s_pfy <= (others => '0');
                s_pcy <= (others => '0');
                s_line_active <= '0';
                s_frame_cnt   <= s_frame_cnt + 1;
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
            if s_pcy < to_unsigned(C_CH, 8) then
                s_rowin <= '1';
            else
                s_rowin <= '0';
            end if;
            s_rowb  <= unsigned(signed(v_rb)  + resize(s_wob, 16));
            s_rowb1 <= unsigned(signed(v_rb1) + resize(s_wob, 16));
            if s_chm > to_unsigned(C_CH, 8) then
                s_ch2 <= to_unsigned(C_CH, 10);
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
            if v_ccx >= to_unsigned(C_CW - 2, 9) then v_ccx1 := to_unsigned(C_CW - 1, 9);
            else v_ccx1 := v_ccx + 2; end if;

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

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
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
                if s_decay_k = to_unsigned(C_CELLS - 1, 16) then
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

            -- ---- write port ----
            if s_decay_rd = '1' then
                if s_cv_rdata >= 2 then
                    v_new := s_cv_rdata - 1;
                elsif s_cv_rdata = 1 then
                    v_hash := (s_decay_k_d1(9 downto 0) xor s_decay_k_d1(15 downto 6))
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
                s_cv_waddr <= s_decay_k_d1;
                s_cv_wdata <= v_new;
                if s_decay_done = '1' then
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

    -- ==== physics ROM (1 EBR, read by the engine only) ====
    p_tbl : process(clk)
    begin
        if rising_edge(clk) then
            s_tbl_q <= s_tbl(to_integer(s_tbl_addr));
        end if;
    end process p_tbl;

    -- ==== nebula ROM (2 EBR): free-running per-pixel read; the 1-2 px
    -- lag against the color pipeline is a fixed shift of a wrapping
    -- texture (starfield trick) ====
    p_neb : process(clk)
    begin
        if rising_edge(clk) then
            s_neb_q <= s_neb(to_integer(s_y_count(8 downto 4)
                                        & s_x_count(8 downto 3)));
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
        variable v_d0  : unsigned(9 downto 0);
        variable v_es  : signed(10 downto 0);
        variable v_wad, v_wbd : unsigned(7 downto 0);
        variable v_wai, v_wbi : std_logic;
        variable v_waf, v_wbf : std_logic;
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
            s_t6 <= s_t5;

            -- stage 3->4: horizontal lerp (blend regs are span-stable)
            v_wxl := "1000" - resize(s_fx3, 4);
            s_h0 <= resize(s_TLb * v_wxl, 5) + resize(s_TRb * s_fx3, 5);
            s_h1 <= resize(s_BLb * v_wxl, 5) + resize(s_BRb * s_fx3, 5);

            -- stage 4->5: vertical lerp -> t (0..192)
            v_wyl := "1000" - resize(s_fy4, 4);
            s_t5 <= resize(s_h0 * v_wyl, 8) + resize(s_h1 * s_fy4, 8);

            -- stage 3: per-slot |dx|,|dy| -> registered max/min
            for k in 0 to 3 loop
                v_dx := signed(resize(s_cellx2, 10)) - signed(resize(s_acx(k), 10));
                v_dy := signed(resize(s_celly2, 9))  - signed(resize(s_acy(k), 9));
                if v_dx >= 0 then v_ax := resize(unsigned(v_dx), 9);
                else              v_ax := resize(unsigned(-v_dx), 9); end if;
                if v_dy >= 0 then v_ay := resize(unsigned(v_dy(8 downto 0)), 9);
                else              v_ay := resize(unsigned(-v_dy), 9); end if;
                if v_ax >= v_ay then
                    s_amax(k) <= v_ax;  s_amin(k) <= v_ay;
                else
                    s_amax(k) <= v_ay;  s_amin(k) <= v_ax;
                end if;
            end loop;

            -- stage 4: raw octagonal distance (max + min/2) + zone flag
            for k in 0 to 3 loop
                v_d0 := resize(s_amax(k), 10) + resize(s_amin(k)(8 downto 1), 10);
                s_draw4(k) <= v_d0;
                if v_d0 > resize(s_arsp(k), 10) then
                    s_far4(k) <= '1';
                else
                    s_far4(k) <= '0';
                end if;
            end loop;

            -- stage 5: subtract 2x split radius (= the shell's current
            -- expansion radius) and clamp: pixels inside a shell's ball score
            -- ~0 for that shell regardless of other centers
            for k in 0 to 3 loop
                v_es := signed(resize(s_draw4(k), 11))
                        - signed(resize(s_arsp(k) & '0', 11));
                if s_aen(k) = '0' then
                    s_deff(k) <= (others => '1');
                elsif v_es < 0 then
                    s_deff(k) <= (others => '0');
                elsif v_es > to_signed(255, 11) then
                    s_deff(k) <= (others => '1');
                else
                    s_deff(k) <= unsigned(v_es(7 downto 0));
                end if;
                s_far5v(k) <= s_far4(k);
            end loop;

            -- stage 6: 4-way argmin over the weighted distances
            if s_deff(0) <= s_deff(1) then
                v_wad := s_deff(0);  v_wai := '0';  v_waf := s_far5v(0);
            else
                v_wad := s_deff(1);  v_wai := '1';  v_waf := s_far5v(1);
            end if;
            if s_deff(2) <= s_deff(3) then
                v_wbd := s_deff(2);  v_wbi := '0';  v_wbf := s_far5v(2);
            else
                v_wbd := s_deff(3);  v_wbi := '1';  v_wbf := s_far5v(3);
            end if;
            if v_wbd < v_wad then
                s_cidx6 <= "01" & v_wbi;  s_far6 <= v_wbf;
            else
                s_cidx6 <= "00" & v_wai;  s_far6 <= v_waf;
            end if;
        end if;
    end process p_blend;

    -- ====================================================================
    -- Attribution registers: latched from slot state once per frame
    -- ====================================================================
    p_attr : process(clk)
        variable v_tl : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                for k in 0 to 3 loop
                    s_aen(k)  <= s_sl_ever(k);
                    s_arsp(k) <= s_sl_rsp(k);
                    s_apas(k) <= s_sl_pas(k);
                    -- attribution follows the center (which now drifts per
                    -- frame in the lifecycle); idle slots hold so embers
                    -- keep their last position and hue
                    if s_sl_state(k) /= "00" then
                        s_acx(k) <= s_sl_cx(k);
                        s_acy(k) <= s_sl_cy(k);
                    end if;
                    -- color by EFFECTIVE timeline (scrub-aware, like the
                    -- replot): white-hot bloom -> shell two-tone -> deep
                    -- embers. During ASCENT the previous shell's colors are
                    -- HELD, not forced gold: the old burst's embers are
                    -- still fading on the canvas, and recoloring them under
                    -- a fresh rocket read as a color wash over the show.
                    -- Idle slots hold too (dead shells keep their embers).
                    if s_sl_state(k) /= "00" then
                        if s_sl_state(k) = "10" then
                            v_tl := signed(resize(s_sl_age(k)(7 downto 0), 10))
                                    + s_scr_b;
                        else
                            v_tl := signed(resize(s_sl_age(k)(7 downto 0), 10))
                                    + s_scr_r;
                        end if;
                        if v_tl < to_signed(64, 10) then
                            null;
                        elsif v_tl < to_signed(70, 10) then
                            s_aci(k) <= "111";
                            s_aco(k) <= "111";
                        elsif v_tl < to_signed(152, 10) then
                            s_aci(k) <= s_sl_col(k);
                            s_aco(k) <= s_sl_col2(k);
                        else
                            s_aci(k) <= C_EMBER(to_integer(s_sl_col(k)));
                            s_aco(k) <= C_EMBER(to_integer(s_sl_col2(k)));
                        end if;
                    end if;
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
        variable v_bxs  : signed(10 downto 0);
        variable v_t3   : unsigned(2 downto 0);
        variable v_n7   : unsigned(6 downto 0);
        variable v_hue  : unsigned(9 downto 0);
        variable v_vis  : std_logic;
        variable v_boom : std_logic;
        variable v_hf_go : std_logic;
        variable v_sp   : boolean;
        variable v_tls  : signed(9 downto 0);
        variable v_scr  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            s_pl_stb <= '0';

            -- free-running layout regs off the settled cell dims (see p_count)
            s_cwu    <= s_cw2(8 downto 0);
            -- launch pads sit on the waterline when the reflection is on
            if s_sw_water = '1' then
                s_ground <= s_horiz - 2;
            else
                s_ground <= resize(s_ch2 - 4, 8);
            end if;
            s_apbase <= resize(shift_right(s_ch2, 2), 8) - 12;

            -- free-running slider decode (plain registers; the vsync-edge
            -- FSM reads 1-bit flags, never SPI-register carry chains)
            s_sldr <= unsigned(registers_in(7));
            if s_sldr >= to_unsigned(96, 10)  then s_sf_go  <= '1'; else s_sf_go  <= '0'; end if;
            if s_sldr >= to_unsigned(960, 10) then s_sf_960 <= '1'; else s_sf_960 <= '0'; end if;
            if s_sldr <  to_unsigned(64, 10)  then s_sf_64  <= '1'; else s_sf_64  <= '0'; end if;
            if s_sldr <  to_unsigned(8, 10)   then s_sf_8   <= '1'; else s_sf_8   <= '0'; end if;
            if resize(s_sldr, 11) + to_unsigned(48, 11)
               < resize(s_hf_max, 11)         then s_sf_rel <= '1'; else s_sf_rel <= '0'; end if;
            if s_sldr > s_hf_max              then s_sf_gtm <= '1'; else s_sf_gtm <= '0'; end if;
            -- pre-registered spawn clamp bound (the spawn-x adder+clamp
            -- chain is the hd_hdmi critical path -- keep it shallow)
            s_cwm32 <= signed(resize(s_cwu, 11)) - to_signed(32, 11);
            -- altitude map: 0..158 cells of rise, clamped at the apogee band
            if resize(s_sldr(9 downto 3), 8) + resize(s_sldr(9 downto 5), 8)
               >= s_ground - s_apbase then
                s_hf_alt <= s_apbase;
            else
                s_hf_alt <= s_ground - (resize(s_sldr(9 downto 3), 8)
                                        + resize(s_sldr(9 downto 5), 8));
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
                        elsif s_sl_state(s_ei) = "00" then
                            if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                        else
                            s_est <= EST_AGE;
                        end if;
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
                        s_est <= EST_P0;
                    else
                        s_cphase <= '0';
                        if s_tl < to_signed(0, 10) then
                            s_cage <= (others => '0');
                        else
                            s_cage <= unsigned(s_tl(6 downto 0));
                        end if;
                        s_est <= EST_R0;
                    end if;

                -- ============ ROCKET (launch phase, one streak/frame) ============
                when EST_R0 =>
                    s_tbl_addr <= s_cage(5 downto 0) & '0';
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
                    if s_cage = 0 then
                        s_tbl_addr <= (others => '0');
                    else
                        s_tbl_addr <= (s_cage(5 downto 0) - 1) & '0';
                    end if;
                    -- sway: one sine cycle over the 64-frame ascent
                    s_lut_angle <= std_logic_vector(s_cage(5 downto 0)) & "0000";
                    s_est <= EST_R0 + 2;
                when EST_R0 + 2 =>
                    s_sinP <= s_lut_sin;
                    s_mul_a <= signed(resize(s_tbl_q(7 downto 0), 8));   -- ease[idx]
                    s_mul_b <= signed(resize(s_hsc, 10));
                    s_est <= EST_R0 + 3;
                when EST_R0 + 3 =>
                    s_mul_a <= signed(resize(s_tbl_q(7 downto 0), 8));   -- ease[idx-1]
                    s_mul_b <= signed(resize(s_hsc, 10));
                    s_est <= EST_R0 + 4;
                when EST_R0 + 4 =>
                    -- p = ease[idx]*height -> current rocket y (rise can reach
                    -- 138, so take 8 product bits, not 7); a hand-held shell
                    -- takes its altitude straight from the slider instead
                    if s_hf_act = '1' and s_ei = 3 then
                        s_yc <= signed(resize(s_hf_cy, 11));
                        s_xc <= signed(resize(s_ccx, 11));      -- no sway
                    else
                        s_yc <= signed(resize(s_ground, 11)) - resize(signed('0' & s_mul_p(14 downto 7)), 11);
                        s_xc <= signed(resize(s_ccx, 11))
                                + resize(shift_right(s_sinP, 8), 11);
                    end if;
                    s_est <= EST_R0 + 5;
                when EST_R0 + 5 =>
                    -- p = ease[idx-1]*height -> previous rocket y
                    if s_hf_act = '1' and s_ei = 3 then
                        s_yp <= signed(resize(s_hf_cyp, 11));
                    else
                        s_yp <= signed(resize(s_ground, 11)) - resize(signed('0' & s_mul_p(14 downto 7)), 11);
                    end if;
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
                    s_tbl_addr <= s_cage;
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
                    if s_cage = 0 then
                        s_tbl_addr <= (others => '0');
                    else
                        s_tbl_addr <= s_cage - 1;
                    end if;
                    s_est <= EST_P0 + 2;
                when EST_P0 + 2 =>
                    s_droopC <= s_tbl_q(15 downto 8);
                    s_mul_a <= signed(resize(s_tbl_q(7 downto 0), 8));    -- ease[idx]
                    s_mul_b <= signed(resize(s_sizeK, 10));
                    s_est <= EST_P0 + 3;
                when EST_P0 + 3 =>
                    s_easeP  <= s_tbl_q(7 downto 0);
                    s_droopP <= s_tbl_q(15 downto 8);
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
                    s_mul_a <= signed(resize(s_droopP, 8));
                    s_mul_b <= signed(resize(s_gravK, 10));
                    s_est <= EST_P0 + 6;
                when EST_P0 + 6 =>
                    s_rbaseP <= unsigned(s_mul_p(13 downto 7));
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
                    s_est <= EST_Q0 + 5;
                when EST_Q0 + 5 =>
                    s_riseP <= unsigned(s_mul_p(13 downto 7));            -- r last frame
                    s_mul_a <= signed(resize(s_rc, 8));
                    s_mul_b <= s_cosP;
                    s_est <= EST_Q0 + 6;
                when EST_Q0 + 6 =>
                    s_mul_a <= signed(resize(s_rc, 8));
                    s_mul_b <= s_sinP;
                    s_est <= EST_Q0 + 7;
                when EST_Q0 + 7 =>
                    s_xc <= signed(resize(s_ccx, 11))
                            + resize(shift_right(s_mul_p, 9), 11);        -- rc*cos
                    s_mul_a <= signed(resize(s_riseP, 8));                -- (r prev)
                    s_mul_b <= s_cosP;
                    s_est <= EST_Q0 + 8;
                when EST_Q0 + 8 =>
                    s_yc <= signed(resize(s_ccy, 11))
                            + resize(shift_right(s_mul_p, 9), 11)         -- rc*sin
                            + signed(resize(s_dreffC, 11));
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
                    s_head <= s_head_pre;
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
                    if (s_xc > 0) and (s_xc < 319) and (s_yc >= 0) and (s_yc <= 177)
                       and (s_xp > 0) and (s_xp < 319) and (s_yp >= 0) and (s_yp <= 177) then
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

            -- ---------------- per-frame lifecycle (vsync edge wins) ----------------
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                -- latch controls; shells scale down when the cell world is
                -- short (1080i fields: 6x6 cells -> only ~180/3 rows usable)
                if s_short = '1' then
                    s_sizeK <= to_unsigned(12, 7)
                               + resize(unsigned(registers_in(1)(9 downto 5)), 7);
                else
                    s_sizeK <= to_unsigned(20, 7) + resize(unsigned(registers_in(1)(9 downto 4)), 7);
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
                if unsigned(registers_in(0)) < to_unsigned(32, 10) then
                    s_silent <= '1';
                else
                    s_silent <= '0';
                end if;
                -- K2 SIZE also drives sparks per shell
                s_dens  <= unsigned(registers_in(1)(9 downto 4));
                -- K4 HUE: 5 palette-bank zones
                v_hue := unsigned(registers_in(3));
                if    v_hue < 205 then s_hueSel <= "000";   -- Gold
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

                -- P12: blackout when fully idle (flags pre-decoded above)
                if s_silent = '1' and s_sf_8 = '1' then
                    s_blk <= '1';
                else
                    s_blk <= '0';
                end if;
                -- FREEZE turns the slider into a timeline scrub: center =
                -- now, ends = -+128 frames; +64 pre-folded for burst slots
                if s_freeze = '1' then
                    v_scr := resize(shift_right(signed('0' & s_sldr)
                                                - to_signed(512, 11), 2), 10);
                else
                    v_scr := (others => '0');
                end if;
                s_scr_r <= v_scr;
                s_scr_b <= v_scr + to_signed(64, 10);
                -- hand-fire trigger: armed and pushed up from the bottom
                v_hf_go := '0';
                if s_freeze = '0' and s_hf_state = "00" and s_sf_go = '1' then
                    v_hf_go := '1';
                end if;

                -- age / spawn / transition each slot (halted by FREEZE;
                -- the per-frame replot keeps running from the frozen ages)
                v_boom := '0';
                if s_freeze = '0' then
                for k in 0 to 3 loop
                    case s_sl_state(k) is
                        when "00" =>   -- idle: wait out the relaunch delay
                            -- slot 3 doubles as the hand-fire shell: manual
                            -- pushes spawn it instantly (even in silence);
                            -- auto relaunch skips it while the hand owns it
                            v_sp := s_silent = '0'
                                    and s_sl_age(k) >= s_sl_delay(k);
                            if k = 3 then
                                v_sp := (v_sp and s_hf_state = "00"
                                         and v_hf_go = '0')
                                        or v_hf_go = '1';
                            end if;
                            if v_sp then
                                v_seed := s_lfsr xor (x"1D" & to_unsigned(k, 2) & s_frame_cnt(5 downto 0));
                                s_sl_seed(k)  <= v_seed;
                                s_sl_state(k) <= "01";
                                s_sl_age(k)   <= (others => '0');
                                s_sl_ever(k)  <= '1';
                                v_bxs := signed(resize(s_pad(k), 11))
                                         + resize(signed('0' & v_seed(5 downto 0)), 11)
                                         - to_signed(32, 11);
                                -- 32-cell margin: near-edge shells lost half
                                -- their sparks off-canvas ("cut off" look)
                                if v_bxs < to_signed(32, 11) then
                                    v_bxs := to_signed(32, 11);
                                elsif v_bxs > s_cwm32 then
                                    v_bxs := s_cwm32;
                                end if;
                                s_sl_cx(k)    <= unsigned(v_bxs(8 downto 0));
                                s_sl_cy(k)    <= s_ground;
                                s_sl_apy(k)   <= s_apbase + resize(v_seed(10 downto 6), 8);
                                -- type: S7 bank pick; S8 mixes in ring
                                -- shells; comets salt both banks (~1 in 4)
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
                                s_sl_typ(k)  <= v_t3;
                                -- K4 HUE bank picks the color pair
                                case s_hueSel is
                                    when "000" =>   -- Gold: white into gold/yellow
                                        s_sl_col(k) <= "111";
                                        if v_seed(11) = '1' then
                                            s_sl_col2(k) <= "010";
                                        else
                                            s_sl_col2(k) <= "001";
                                        end if;
                                    when "001" =>   -- Primaries: white into R/G/B
                                        s_sl_col(k) <= "111";
                                        case v_seed(12 downto 11) is
                                            when "00"   => s_sl_col2(k) <= "000";
                                            when "01"   => s_sl_col2(k) <= "011";
                                            when "10"   => s_sl_col2(k) <= "100";
                                            when others => s_sl_col2(k) <= "000";
                                        end case;
                                    when "011" =>   -- Red-White-Blue solids
                                        case v_seed(12 downto 11) is
                                            when "00"   => s_sl_col(k) <= "000";
                                                           s_sl_col2(k) <= "000";
                                            when "01"   => s_sl_col(k) <= "111";
                                                           s_sl_col2(k) <= "111";
                                            when others => s_sl_col(k) <= "100";
                                                           s_sl_col2(k) <= "100";
                                        end case;
                                    when others =>  -- Pastel / Random: any pair
                                        s_sl_col(k)  <= v_seed(13 downto 11);
                                        s_sl_col2(k) <= v_seed(13 downto 11)
                                                        + to_unsigned(2, 3)
                                                        + resize(v_seed(15 downto 14), 3);
                                end case;
                                -- pastel bank is all-pastel; the Random
                                -- bank salts in pastel shells (~1 in 4)
                                -- for more color variety
                                if s_hueSel = "010"
                                   or (s_hueSel = "100"
                                       and v_seed(10 downto 9) = "11") then
                                    s_sl_pas(k) <= '1';
                                else
                                    s_sl_pas(k) <= '0';
                                end if;
                                s_sl_rsp(k) <= to_unsigned(2, 7);
                                -- spark count (palm/comet: few thick arms)
                                if v_t3 = C_TY_PALM then
                                    s_sl_n(k) <= to_unsigned(10, 6);
                                elsif v_t3 = C_TY_COMET then
                                    s_sl_n(k) <= to_unsigned(14, 6);
                                else
                                    v_n7 := to_unsigned(10, 7) + resize(s_dens, 7);
                                    if v_n7 > 60 then v_n7 := to_unsigned(60, 7); end if;
                                    s_sl_n(k) <= v_n7(5 downto 0);
                                end if;
                            else
                                s_sl_age(k) <= s_sl_age(k) + 1;
                            end if;
                        when "01" =>   -- launch: 64-frame ascent
                            if k = 3 and s_hf_act = '1' then
                                null;  -- hand-held: age frozen, altitude
                                       -- comes from the slider override
                            elsif s_sl_age(k) = 63 then
                                s_sl_state(k) <= "10";
                                s_sl_age(k)   <= (others => '0');
                                s_sl_cy(k)    <= s_sl_apy(k);
                                v_boom := '1';
                            else
                                s_sl_age(k) <= s_sl_age(k) + 1;
                            end if;
                        when "10" =>   -- burst: 128-frame bloom and fade
                            if s_sl_age(k) = 127 then
                                s_sl_state(k) <= "00";
                                s_sl_age(k)   <= (others => '0');
                                -- FINALE: tight jitter -> overlapping salvos
                                if s_sw_finale = '1' then
                                    s_sl_delay(k) <= s_rateD
                                        + resize(s_sl_seed(k)(2 downto 0), 10);
                                else
                                    s_sl_delay(k) <= s_rateD
                                        + resize(s_sl_seed(k)(4 downto 0), 10);
                                end if;
                            else
                                s_sl_age(k) <= s_sl_age(k) + 1;
                                -- the whole cloud DRIFTS as it fades: paced
                                -- integer steps of the base center (sparks,
                                -- core and attribution all follow for free;
                                -- FREEZE halts it, scrub collapses the
                                -- pattern around the drifted position).
                                -- Sink: 1 cell / 8 frames, doubled on the
                                -- heavy half of the GRAVITY knob; none at 0
                                if (s_gravK /= 0
                                    and s_sl_age(k)(2 downto 0) = "000")
                                   or (s_gravK(5) = '1'
                                       and s_sl_age(k)(2 downto 0) = "100") then
                                    if s_sl_cy(k) < to_unsigned(176, 8) then
                                        s_sl_cy(k) <= s_sl_cy(k) + 1;
                                    end if;
                                end if;
                                -- sideways momentum: seed-picked direction,
                                -- 1 cell / 8 or / 16 frames
                                if (s_sl_seed(k)(2) = '1'
                                    and s_sl_age(k)(2 downto 0) = "010")
                                   or (s_sl_seed(k)(2) = '0'
                                       and s_sl_age(k)(3 downto 0) = "0110") then
                                    if s_sl_seed(k)(1) = '1' then
                                        if signed(resize(s_sl_cx(k), 11)) < s_cwm32 then
                                            s_sl_cx(k) <= s_sl_cx(k) + 1;
                                        end if;
                                    else
                                        if s_sl_cx(k) > to_unsigned(32, 9) then
                                            s_sl_cx(k) <= s_sl_cx(k) - 1;
                                        end if;
                                    end if;
                                end if;
                            end if;
                        when others => -- unreachable: recover to idle
                            s_sl_state(k) <= "00";
                            s_sl_age(k)   <= (others => '0');
                    end case;
                end loop;
                end if;

                -- ==== hand-fire FSM (after the loop so a detonation wins
                -- over the launch branch's writes to slot 3) ====
                if s_freeze = '0' then
                    -- altitude tracks the slider every frame (pre-mapped)
                    s_hf_cyp <= s_hf_cy;
                    s_hf_cy  <= s_hf_alt;
                    case s_hf_state is
                        when "00" =>            -- armed
                            if v_hf_go = '1' then
                                s_hf_state <= "01";
                                s_hf_act   <= '1';
                                s_hf_max   <= s_sldr;
                                s_hf_cy    <= s_ground;
                                s_hf_cyp   <= s_ground;
                            end if;
                        when "01" =>            -- flying by hand
                            if s_sf_gtm = '1' then
                                s_hf_max <= s_sldr;
                            end if;
                            -- FULL THROW (slider reaches the top): hand the
                            -- shell off to a normal flight -- age jumps to
                            -- the late-ascent point matching the altitude,
                            -- so it keeps rising and bursts on the standard
                            -- timer. Reload: back to 0, then push up again.
                            if s_sf_960 = '1' then
                                s_sl_age(3) <= to_unsigned(52, 10);
                                s_sl_apy(3) <= s_hf_cy;
                                s_hf_act    <= '0';
                                s_hf_state  <= "10";
                            -- otherwise detonate on release/reverse: a
                            -- 48-count drop from the peak bursts right there
                            elsif s_sf_rel = '1' then
                                s_sl_state(3) <= "10";
                                s_sl_age(3)   <= (others => '0');
                                s_sl_cy(3)    <= s_hf_cy;
                                s_sl_apy(3)   <= s_hf_cy;
                                s_sl_rsp(3)   <= to_unsigned(2, 7);
                                s_hf_act   <= '0';
                                s_hf_state <= "10";
                                v_boom := '1';
                            end if;
                        when others =>          -- wait for re-arm at bottom
                            if s_sf_64 = '1' then
                                s_hf_state <= "00";
                            end if;
                    end case;
                elsif s_hf_act = '1' then
                    -- freezing mid-flight captures the shell as a burst at
                    -- the hand altitude (and hands the slider to the scrub)
                    s_sl_state(3) <= "10";
                    s_sl_age(3)   <= (others => '0');
                    s_sl_cy(3)    <= s_hf_cy;
                    s_sl_apy(3)   <= s_hf_cy;
                    s_sl_rsp(3)   <= to_unsigned(2, 7);
                    s_hf_act   <= '0';
                    s_hf_state <= "10";
                    v_boom := '1';
                end if;

                -- ambient sky flash: reload on any detonation, fade per
                -- frame (FINALE: brighter reload, half-rate fade)
                if v_boom = '1' then
                    if s_sw_finale = '1' then
                        s_flash <= "11";
                    else
                        s_flash <= "10";
                    end if;
                elsif s_flash /= 0
                      and (s_sw_finale = '0' or s_frame_cnt(0) = '0') then
                    s_flash <= s_flash - 1;
                end if;

                -- restart the replot pass for the new frame
                s_go  <= '1';
                s_est <= EST_IDLE;
            end if;
        end if;
    end process p_engine;

    -- ====================================================================
    -- Pen writer: 8-connected line-walker with optional plus-shaped head.
    -- Walks from 'from' (exclusive) to 'to' (inclusive); from == to plots
    -- a single dot. Step cap keeps a bad request from hogging the frame.
    -- ====================================================================
    p_writer : process(clk)
        variable v_wake : unsigned(1 downto 0);
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
                            s_pen_waddr <= f_addr(s_wk_y, s_wk_x - 1);
                        end if;
                        s_w_step <= 3;
                    when 3 =>
                        if s_wk_x < to_unsigned(C_CW - 1, 9) then
                            s_pen_we    <= '1';
                            s_pen_wdata <= s_w_val;
                            s_pen_waddr <= f_addr(s_wk_y, s_wk_x + 1);
                        end if;
                        s_w_step <= 4;
                    when 4 =>
                        if s_wk_y > 0 then
                            s_pen_we    <= '1';
                            s_pen_wdata <= s_w_val;
                            s_pen_waddr <= f_addr(s_wk_y - 1, s_wk_x);
                        end if;
                        s_w_step <= 5;
                    when others =>
                        if s_wk_y < to_unsigned(C_CH - 1, 8) then
                            s_pen_we    <= '1';
                            s_pen_wdata <= s_w_val;
                            s_pen_waddr <= f_addr(s_wk_y + 1, s_wk_x);
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
        variable v_ci : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            v_t := s_t6;
            -- luma = t*4 (t <= 192 -> max 768: full-scale Y clips all RGB
            -- channels on real displays and washes saturated chroma to white)
            v_y := resize(v_t & "00", 11);
            if s_insq6 = '1' and v_t /= 0 then
                -- drawn spark; mirrored copies dim to 5/8 and shift blue
                if s_mir6 = '1' then
                    s_d_y <= resize(v_y(9 downto 1), 10)
                             + resize(v_y(9 downto 3), 10);
                else
                    s_d_y <= v_y(9 downto 0);
                end if;
                -- index bit 3 = the owning shell's pastel flag
                if s_far6 = '1' then
                    v_ci := s_apas(to_integer(s_cidx6)) & s_aco(to_integer(s_cidx6));
                else
                    v_ci := s_apas(to_integer(s_cidx6)) & s_aci(to_integer(s_cidx6));
                end if;
                s_d_u <= C_PAL_U(to_integer(v_ci));
                if s_mir6 = '1' then
                    s_d_v <= C_PAL_V(to_integer(v_ci)) + to_unsigned(32, 10);
                else
                    s_d_v <= C_PAL_V(to_integer(v_ci));
                end if;
            elsif s_insq6 = '1' and s_star6 = '1' and s_blk = '0' then
                -- static star: crisp dot, cool white, slow twinkle;
                -- reflected stars shimmer at reduced brightness
                if s_stwk6 = '1' then
                    s_d_y <= to_unsigned(200, 10);
                elsif s_mir6 = '1' then
                    s_d_y <= to_unsigned(180, 10);
                else
                    s_d_y <= to_unsigned(300, 10);
                end if;
                s_d_u <= to_unsigned(504, 10);      -- stored U = Cr: cool
                s_d_v <= to_unsigned(524, 10);      -- stored V = Cb: bluish
            elsif s_insq6 = '1' and s_mir6 = '1' and s_blk = '0' then
                -- open water: flat, dark, deep blue; half the sky flash
                s_d_y <= to_unsigned(48, 10)
                         + resize(s_flash & "00000", 10);
                s_d_u <= to_unsigned(494, 10);
                s_d_v <= to_unsigned(548, 10);
            elsif s_insq6 = '1' and s_blk = '0' then
                -- night sky: blue gradient + nebula clouds (moonlit: a
                -- touch brighter and grayer where the cloud is dense)
                -- + ambient detonation flash
                s_d_y <= resize(s_bgy, 10)
                         + resize(s_neb_q & "00", 10)
                         + resize(s_flash & "000000", 10);
                s_d_u <= to_unsigned(498, 10) + resize(s_neb_q(3 downto 2), 10);
                s_d_v <= to_unsigned(536, 10) - resize(s_neb_q(3 downto 2), 10);
            else
                s_d_y <= (others => '0');
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
            for i in 1 to C_LATENCY - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_sr;

    data_out.hsync_n <= s_hsync_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);

    data_out.y <= std_logic_vector(s_d_y);
    data_out.u <= std_logic_vector(s_d_u);
    data_out.v <= std_logic_vector(s_d_v);

end architecture pyro;
