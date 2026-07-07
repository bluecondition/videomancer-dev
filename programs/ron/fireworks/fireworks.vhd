-- fireworks.vhd
--
-- Fireworks: a night-sky pyrotechnics show.
--
-- Four launch slots fire rockets from staggered pads at the bottom of the
-- screen; each rocket ascends on an eased (decelerating) trajectory with a
-- glowing trail, then bursts at its apogee into one of five shell types:
--   Peony   -- filled sphere of sparks (speed-randomized radii)
--   Ring    -- thin expanding circle (uniform speed)
--   Willow  -- slow golden-hour sphere with doubled gravity droop
--   Palm    -- a few thick comet arms (forced spark heads)
--   Crackle -- strobing/flickering sparks (50% duty hash gate)
-- (v1.3b: the ground fountain was cut -- its FSM block + 5th attribution
-- candidate were the LC/congestion margin HD timing needed; S10 = Smoke.)
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
-- Control map:
--   K1 (reg0) Launch Rate            S7 (reg6.0) Twinkle (late-life shimmer)
--   K2 (reg1) Shell Size             S8 (reg6.1) Big Sparks (plus-shaped)
--   K3 (reg2) Shell Type (6 zones)   S9 (reg6.2) Pistil (inner half-r shell)
--   K4 (reg3) Gravity                S10(reg6.3) Smoke On/Off
--   K5 (reg4) Color (8 zones)        S11(reg6.4) Background Black/Video
--   K6 (reg5) Smoke (ghost hang)     Slider(reg7) Density (+auto-pistil)
--
-- v1.1: two-tone shells (inner/outer color split at half the current shell
-- radius -- trails hand off color as sparks fly outward), bright persistent
-- core plus at the center, SMOKE phase after each burst, 32-cell edge margin.
-- v1.2: crisp sparks (bright cells collapse every sweep; only val-1 smoke
-- lingers on the dithered path), isotropic per-frame dither (hash+framecount
-- TRANSLATED across the screen = phantom sideways scroll).
-- v1.3: cell/phase COUNTER rasterization -- 4-px cells (<=1280-wide rasters)
-- or 6-px cells (1080-class, FULL 320x180 canvas; v1.2's 8x8 was too coarse);
-- RADIUS-WEIGHTED color attribution (raw Voronoi carved overlapping shells
-- into color patches); P12 = WIND, bipolar, default calm (bursts+smoke drift,
-- rockets stay clean); K6 = Intensity (sparks + smoke hang + auto-pistil).
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

architecture fireworks of program_top is

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
    signal s_pad    : t_sl_u9 := (to_unsigned(40, 9),  to_unsigned(120, 9),
                                  to_unsigned(200, 9), to_unsigned(280, 9));

    -- shell types
    constant C_TY_PEONY   : unsigned(2 downto 0) := "000";
    constant C_TY_RING    : unsigned(2 downto 0) := "001";
    constant C_TY_WILLOW  : unsigned(2 downto 0) := "010";
    constant C_TY_PALM    : unsigned(2 downto 0) := "011";
    constant C_TY_CRACKLE : unsigned(2 downto 0) := "100";

    -- ==== Per-frame latched controls ====
    signal s_sizeK   : unsigned(6 downto 0) := to_unsigned(52, 7);   -- 20..83
    signal s_gravK   : unsigned(5 downto 0) := to_unsigned(23, 6);   -- 0..63
    signal s_rateD   : unsigned(9 downto 0) := to_unsigned(180, 10);
    signal s_typeSel : unsigned(2 downto 0) := "000";
    signal s_colSel  : unsigned(2 downto 0) := "000";
    signal s_dens    : unsigned(5 downto 0) := to_unsigned(40, 6);
    signal s_sw_twinkle  : std_logic := '0';
    signal s_sw_sparks   : std_logic := '0';
    signal s_sw_pistil   : std_logic := '0';
    signal s_sw_smoke    : std_logic := '1';
    signal s_bg_video    : std_logic := '0';

    -- ==== Wind (P12, bipolar around center; default = none) ====
    signal s_wmag : unsigned(3 downto 0) := (others => '0');   -- 0..8
    signal s_wpos : std_logic := '1';                          -- '1' = rightward
    signal s_wacc : unsigned(5 downto 0) := (others => '0');

    -- ==== Free-running LFSR (seeds) ====
    signal s_lfsr : unsigned(15 downto 0) := x"BEEF";

    -- ==== Particle engine FSM ====
    constant EST_IDLE : integer := 0;
    constant EST_SLOT : integer := 1;
    constant EST_R0 : integer := 2;   -- rocket: R0..R6 = 2..8
    constant EST_P0 : integer := 9;   -- burst prep: P0..P7 = 9..16
    constant EST_PC : integer := 17;  -- persistent core flash issue
    constant EST_Q0 : integer := 18;  -- particle: Q0..Q13 = 18..31
    constant EST_S0 : integer := 32;  -- smoke: S0..S4 = 32..36

    signal s_est : integer range 0 to 36 := EST_IDLE;
    signal s_go  : std_logic := '0';
    signal s_ei  : integer range 0 to 4 := 0;
    signal s_pi  : unsigned(5 downto 0) := (others => '0');

    signal s_h      : unsigned(15 downto 0) := (others => '0');
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
    signal s_fage   : unsigned(6 downto 0)  := (others => '0');
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
    signal s_pist_en  : std_logic := '0';
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
    -- 0 Red, 1 Gold, 2 Yellow, 3 Green, 4 Blue, 5 Indigo, 6 Violet, 7 White
    type t_pal8 is array(0 to 7) of unsigned(9 downto 0);
    constant C_PAL_U : t_pal8 := (
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10), to_unsigned(136, 10),
        to_unsigned(440, 10), to_unsigned(608, 10), to_unsigned(756, 10), to_unsigned(512, 10));
    constant C_PAL_V : t_pal8 := (
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10), to_unsigned(216, 10),
        to_unsigned(960, 10), to_unsigned(696, 10), to_unsigned(852, 10), to_unsigned(512, 10));
    -- K5 fixed zones 1..7 -> palette index (Mix handled separately)
    type t_map8 is array(0 to 7) of unsigned(2 downto 0);
    constant C_COLMAP : t_map8 := ("000", "000", "001", "010", "011", "100", "110", "111");

    -- ==== Nearest-shell color attribution ====
    type t_at_u9  is array(0 to 3) of unsigned(8 downto 0);
    type t_at_u8  is array(0 to 4) of unsigned(7 downto 0);
    signal s_acx : t_at_u9 := (others => to_unsigned(160, 9));
    signal s_acy : t_at_u8 := (others => to_unsigned(90, 8));
    signal s_aen : std_logic_vector(3 downto 0) := "0000";
    -- 3-bit palette indices + gray flag (chroma resolved in stage 6 via the
    -- constant palette, spiroscope-style: 20x 10-bit color regs was ~250 LC)
    type t_at_u3 is array(0 to 3) of unsigned(2 downto 0);
    signal s_aci   : t_at_u3 := (others => "001");
    signal s_aco   : t_at_u3 := (others => "001");
    signal s_agray : std_logic_vector(3 downto 0) := (others => '0');
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
    signal s_drawn : std_logic := '0';

    -- ==== sync / video delay SR ====
    type t_data_sr is array(0 to C_LATENCY - 1) of std_logic_vector(9 downto 0);
    type t_bit_sr  is array(0 to C_LATENCY - 1) of std_logic;
    signal s_y_sr, s_u_sr, s_v_sr : t_data_sr := (others => (others => '0'));
    signal s_hsync_sr, s_vsync_sr, s_field_sr, s_avid_sr : t_bit_sr := (others => '0');

begin

    -- ====================================================================
    -- stage 1: pixel/line counters + per-frame geometry
    -- ====================================================================
    p_count : process(clk)
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
                s_bg_video    <= registers_in(6)(4);
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
        variable v_ccy      : unsigned(7 downto 0);
        variable v_ccx1     : unsigned(8 downto 0);
        variable v_ccy1     : unsigned(7 downto 0);
        variable v_fx, v_fy : unsigned(2 downto 0);
        variable v_ph0, v_ph1 : boolean;
        variable v_in       : boolean;
    begin
        if rising_edge(clk) then
            v_in := (s_pcx < to_unsigned(C_CW, 9)) and (s_pcy < to_unsigned(C_CH, 8));
            -- the counters saturate AT the canvas edge (320/180) so v_in can
            -- gate; clamp before address formation or the arbiter's idle-slot
            -- read lands out of bounds (sim bound-check, exact-fit rasters)
            if s_pcx < to_unsigned(C_CW, 9) then
                v_ccx := s_pcx;
            else
                v_ccx := to_unsigned(C_CW - 1, 9);
            end if;
            if s_pcy < to_unsigned(C_CH, 8) then
                v_ccy := s_pcy;
            else
                v_ccy := to_unsigned(C_CH - 1, 8);
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
            v_ph0 := s_pfx = 0;
            v_ph1 := s_pfx = 1;
            -- prefetch target: span S+1's RIGHT neighbour = cell S+2
            if v_ccx >= to_unsigned(C_CW - 2, 9) then v_ccx1 := to_unsigned(C_CW - 1, 9);
            else v_ccx1 := v_ccx + 2; end if;
            if v_ccy = to_unsigned(C_CH - 1, 8) then v_ccy1 := v_ccy;
            else v_ccy1 := v_ccy + 1; end if;

            s_cellx2 <= v_ccx;
            s_celly2 <= v_ccy;
            s_fx2 <= v_fx;
            s_fy2 <= v_fy;
            s_addr_top2 <= f_addr(v_ccy,  v_ccx1);
            s_addr_bot2 <= f_addr(v_ccy1, v_ccx1);
            if v_in and v_ph0 then s_ftop2 <= '1'; else s_ftop2 <= '0'; end if;
            if v_in and v_ph1 then s_fbot2 <= '1'; else s_fbot2 <= '0'; end if;
            if v_in and v_ccx >= 2 then s_insq2 <= '1'; else s_insq2 <= '0'; end if;
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
                -- only val-1 cells (smoke, last embers) linger on the slow
                -- dithered path. K6 = Smoke hang time: thr 326 (K6=0, ~3
                -- frames) .. 8 (K6 max, ~2 s).
                v_sl := to_unsigned(1023, 10) - unsigned(registers_in(5));
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
    begin
        if rising_edge(clk) then
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                for k in 0 to 3 loop
                    s_acx(k)  <= s_sl_cx(k);
                    s_acy(k)  <= s_sl_cy(k);
                    s_aen(k)  <= s_sl_ever(k);
                    s_arsp(k) <= s_sl_rsp(k);
                    case s_sl_state(k) is
                        when "01" =>          -- ascending rocket: gold
                            s_aci(k)   <= "001";
                            s_aco(k)   <= "001";
                            s_agray(k) <= '0';
                        when "10" =>          -- burst: inner col / outer col2
                            s_aci(k)   <= s_sl_col(k);
                            s_aco(k)   <= s_sl_col2(k);
                            s_agray(k) <= '0';
                        when others =>        -- smoke / idle: neutral gray
                            s_agray(k) <= '1';
                    end case;
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
        variable v_ang  : signed(9 downto 0);
        variable v_ang8 : unsigned(7 downto 0);
        variable v_seed : unsigned(15 downto 0);
        variable v_bxs  : signed(10 downto 0);
        variable v_wnd  : signed(10 downto 0);
        variable v_wst  : std_logic;
        variable v_t3   : unsigned(2 downto 0);
        variable v_n7   : unsigned(6 downto 0);
        variable v_k3   : unsigned(9 downto 0);
        variable v_mx   : unsigned(9 downto 0);
        variable v_my   : unsigned(9 downto 0);
        variable v_vis  : std_logic;
    begin
        if rising_edge(clk) then
            s_pl_stb <= '0';

            -- free-running layout regs off the settled cell dims (see p_count)
            s_cwu    <= s_cw2(8 downto 0);
            s_ground <= resize(s_ch2 - 4, 8);
            s_apbase <= resize(shift_right(s_ch2, 2), 8) - 12;
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
                        elsif s_sl_state(s_ei) = "01" then
                            s_est <= EST_R0;
                        elsif s_sl_state(s_ei) = "10" then
                            s_est <= EST_P0;
                        elsif s_sl_state(s_ei) = "11" then
                            s_est <= EST_S0;
                        else
                            if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                        end if;
                    end if;

                -- ============ ROCKET (launch phase, one streak/frame) ============
                when EST_R0 =>
                    s_tbl_addr <= s_sl_age(s_ei)(5 downto 0) & '0';
                    -- ascent height, registered so the multiplier operand mux
                    -- reads a plain register (not slot-array mux + subtract)
                    s_hsc <= s_ground - s_sl_apy(s_ei);
                    s_ccx <= s_sl_cx(s_ei);
                    s_est <= EST_R0 + 1;
                when EST_R0 + 1 =>
                    if s_sl_age(s_ei)(5 downto 0) = 0 then
                        s_tbl_addr <= (others => '0');
                    else
                        s_tbl_addr <= (s_sl_age(s_ei)(5 downto 0) - 1) & '0';
                    end if;
                    -- sway: one sine cycle over the 64-frame ascent
                    s_lut_angle <= std_logic_vector(s_sl_age(s_ei)(5 downto 0)) & "0000";
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
                    -- 138, so take 8 product bits, not 7)
                    s_yc <= signed(resize(s_ground, 11)) - resize(signed('0' & s_mul_p(14 downto 7)), 11);
                    s_xc <= signed(resize(s_ccx, 11))
                            + resize(shift_right(s_sinP, 8), 11);
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
                        s_pl_head <= s_sw_sparks;
                        s_pl_stb <= '1';
                        -- track attribution center up the ascent
                        s_sl_cy(s_ei) <= unsigned(s_yc(7 downto 0));
                        if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                        s_est <= EST_SLOT;
                    end if;

                -- ============ BURST prep: shared radius/droop for the shell ============
                when EST_P0 =>
                    s_tbl_addr <= s_sl_age(s_ei)(6 downto 0);
                    s_ccx   <= s_sl_cx(s_ei);
                    s_ccy   <= s_sl_cy(s_ei);
                    s_ctyp  <= s_sl_typ(s_ei);
                    s_cseed <= s_sl_seed(s_ei);
                    s_cn    <= s_sl_n(s_ei);
                    v_idx := s_sl_age(s_ei)(6 downto 0);
                    if v_idx >= 112 then
                        s_val_pre <= "01";
                    elsif v_idx >= 88 then
                        s_val_pre <= "10";
                    else
                        s_val_pre <= "11";
                    end if;
                    if s_sw_sparks = '1' or s_sl_typ(s_ei) = C_TY_PALM or v_idx < 6 then
                        s_head_pre <= '1';
                    else
                        s_head_pre <= '0';
                    end if;
                    if s_sw_twinkle = '1' and v_idx >= 80 then
                        s_twk_en <= '1';
                    else
                        s_twk_en <= '0';
                    end if;
                    if s_sw_pistil = '1' and v_idx >= 24 then
                        s_pist_en <= '1';
                    else
                        s_pist_en <= '0';
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
                    if s_sl_age(s_ei)(6 downto 0) = 0 then
                        s_tbl_addr <= (others => '0');
                    else
                        s_tbl_addr <= s_sl_age(s_ei)(6 downto 0) - 1;
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
                    -- the multiplier operand mux was the next critical path
                    s_lut_angle <= std_logic_vector(s_h(9 downto 2)) & "00";
                    v_rv := f_rev6(s_pi) xor s_cseed(14 downto 9);
                    case s_ctyp is
                        when C_TY_RING =>
                            v_vf := to_unsigned(120, 8) + resize(s_h(2 downto 0), 8);
                        when C_TY_WILLOW =>
                            v_vf := to_unsigned(72, 8) + resize(v_rv(5 downto 1), 8);
                        when C_TY_PALM =>
                            v_vf := to_unsigned(112, 8) + resize(s_h(12 downto 10), 8);
                        when others =>   -- peony / crackle: filled sphere
                            v_vf := to_unsigned(64, 8) + resize(v_rv, 8);
                    end case;
                    s_vfrac <= v_vf;
                    s_est <= EST_Q0 + 2;
                when EST_Q0 + 2 =>
                    s_sinP <= s_lut_sin;
                    -- re-aim the shared tree at angle+256 -> cos next state
                    v_ang8 := unsigned(s_h(9 downto 2)) + to_unsigned(64, 8);
                    s_lut_angle <= std_logic_vector(v_ang8) & "00";
                    s_mul_a <= signed(resize(s_rbaseC, 8));
                    s_mul_b <= signed(resize(s_vfrac, 10));
                    s_est <= EST_Q0 + 3;
                when EST_Q0 + 3 =>
                    s_cosP <= s_lut_sin;                      -- sin(a+256) = cos(a)
                    s_mul_a <= signed(resize(s_rbaseP, 8));
                    s_mul_b <= signed(resize(s_vfrac, 10));
                    s_est <= EST_Q0 + 4;
                when EST_Q0 + 4 =>
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
                    s_val  <= s_val_pre;
                    s_head <= s_head_pre;
                    v_vis := '1';
                    if s_ctyp = C_TY_CRACKLE
                       and (s_h(1 downto 0) xor s_frame_cnt(1 downto 0)) >= 2 then
                        v_vis := '0';    -- 50% duty strobe
                    end if;
                    if s_twk_en = '1'
                       and (s_h(2 downto 0) xor s_frame_cnt(2 downto 0)) >= 5 then
                        v_vis := '0';    -- late-life shimmer
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
                        if s_pist_en = '1' and s_pi(0) = '0' then
                            s_est <= EST_Q0 + 13;
                        elsif s_lastp = '1' then
                            if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                            s_est <= EST_SLOT;
                        else
                            s_pi  <= s_pi + 1;
                            s_est <= EST_Q0;
                        end if;
                    end if;
                when EST_Q0 + 13 =>
                    -- pistil: midpoint trick = exact half-radius spark, no mult
                    if s_w_active = '0' and s_pl_stb = '0' then
                        v_mx := resize(unsigned(s_xp(8 downto 0)), 10) + resize(s_ccx, 10);
                        v_my := resize(unsigned(s_yp(7 downto 0)), 10) + resize(s_ccy, 10);
                        s_pl_fx <= v_mx(9 downto 1);
                        s_pl_fy <= v_my(8 downto 1);
                        v_mx := resize(unsigned(s_xc(8 downto 0)), 10) + resize(s_ccx, 10);
                        v_my := resize(unsigned(s_yc(7 downto 0)), 10) + resize(s_ccy, 10);
                        s_pl_tx <= v_mx(9 downto 1);
                        s_pl_ty <= v_my(8 downto 1);
                        s_pl_val <= "10";
                        s_pl_head <= '0';
                        s_pl_stb <= '1';
                        if s_lastp = '1' then
                            if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                            s_est <= EST_SLOT;
                        else
                            s_pi  <= s_pi + 1;
                            s_est <= EST_Q0;
                        end if;
                    end if;

                -- ============ SMOKE: drifting gray ghost after the shell ============
                -- 12 flat-cloud dots around the drifting center; each dot
                -- thins out as the cloud ages (dark gray via neutral-chroma
                -- attribution + val "01" luma). One registered step per state.
                when EST_S0 =>
                    s_ccx   <= s_sl_cx(s_ei);
                    s_ccy   <= s_sl_cy(s_ei);
                    s_cseed <= s_sl_seed(s_ei);
                    s_fage  <= s_sl_age(s_ei)(6 downto 0);
                    s_pi    <= (others => '0');
                    s_est   <= EST_S0 + 1;
                when EST_S0 + 1 =>
                    s_h   <= f_hash(s_cseed xor x"5A5A", s_pi);
                    s_est <= EST_S0 + 2;
                when EST_S0 + 2 =>
                    -- flat cloud: x +/-15, y +/-4 around the center
                    s_xc <= signed(resize(s_ccx, 11))
                            + resize(signed('0' & s_h(4 downto 0)), 11)
                            - to_signed(16, 11);
                    s_yc <= signed(resize(s_ccy, 11))
                            + resize(signed('0' & s_h(8 downto 6)), 11)
                            - to_signed(4, 11);
                    -- dissolve: dots drop out one by one as the cloud ages,
                    -- survivors shimmer
                    v_vis := '1';
                    if resize(s_h(15 downto 12), 5) < resize(s_fage(6 downto 3), 5) then
                        v_vis := '0';
                    end if;
                    -- 25% duty: near-continuous replot drew solid sideways
                    -- streaks as the cloud drifted; sparse flicker reads as puffs
                    if (s_h(2 downto 0) xor s_frame_cnt(2 downto 0)) >= 2 then
                        v_vis := '0';
                    end if;
                    s_vis <= v_vis;
                    s_est <= EST_S0 + 3;
                when EST_S0 + 3 =>
                    if (s_xc > 0) and (s_xc < 319) and (s_yc >= 0) and (s_yc <= 177) then
                        s_ok <= '1';
                    else
                        s_ok <= '0';
                    end if;
                    s_est <= EST_S0 + 4;
                when EST_S0 + 4 =>
                    if s_ok = '0' or s_vis = '0' then
                        if s_pi = 15 then
                            if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                            s_est <= EST_SLOT;
                        else
                            s_pi  <= s_pi + 1;
                            s_est <= EST_S0 + 1;
                        end if;
                    elsif s_w_active = '0' and s_pl_stb = '0' then
                        s_pl_fx  <= unsigned(s_xc(8 downto 0));
                        s_pl_fy  <= unsigned(s_yc(7 downto 0));
                        s_pl_tx  <= unsigned(s_xc(8 downto 0));
                        s_pl_ty  <= unsigned(s_yc(7 downto 0));
                        s_pl_val <= "01";
                        s_pl_head <= '0';
                        s_pl_stb <= '1';
                        if s_pi = 15 then
                            if s_ei = 3 then s_ei <= 4; else s_ei <= s_ei + 1; end if;
                            s_est <= EST_SLOT;
                        else
                            s_pi  <= s_pi + 1;
                            s_est <= EST_S0 + 1;
                        end if;
                    end if;

                when others =>
                    s_est <= EST_IDLE;
            end case;

            -- ---------------- per-frame lifecycle (vsync edge wins) ----------------
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                -- latch controls; shells scale down when the cell world is
                -- short (1080i fields: 8x8 cells -> only ~67 cells high)
                if s_short = '1' then
                    s_sizeK <= to_unsigned(12, 7)
                               + resize(unsigned(registers_in(1)(9 downto 5)), 7);
                else
                    s_sizeK <= to_unsigned(20, 7) + resize(unsigned(registers_in(1)(9 downto 4)), 7);
                end if;
                s_gravK <= unsigned(registers_in(3)(9 downto 4));
                s_rateD <= to_unsigned(24, 10)
                           + resize(shift_right(to_unsigned(1023, 10)
                                                - unsigned(registers_in(0)), 1), 10);
                -- K6 = Intensity: sparks per shell (+ smoke hang, pistil)
                s_dens  <= unsigned(registers_in(5)(9 downto 4));
                v_k3 := unsigned(registers_in(2));
                if    v_k3 < 171 then s_typeSel <= "000";   -- Mix
                elsif v_k3 < 342 then s_typeSel <= "001";   -- Peony
                elsif v_k3 < 512 then s_typeSel <= "010";   -- Ring
                elsif v_k3 < 683 then s_typeSel <= "011";   -- Willow
                elsif v_k3 < 853 then s_typeSel <= "100";   -- Palm
                else                  s_typeSel <= "101";   -- Crackle
                end if;
                s_colSel <= unsigned(registers_in(4)(9 downto 7));
                s_sw_twinkle  <= registers_in(6)(0);
                s_sw_sparks   <= registers_in(6)(1);
                if registers_in(6)(2) = '1'
                   or unsigned(registers_in(5)) >= to_unsigned(640, 10) then
                    s_sw_pistil <= '1';
                else
                    s_sw_pistil <= '0';
                end if;
                -- P12 = Wind: bipolar around center with a dead zone
                v_wnd := signed('0' & registers_in(7)) - to_signed(512, 11);
                if v_wnd < 0 then
                    s_wpos <= '0';
                    v_wnd  := -v_wnd;
                else
                    s_wpos <= '1';
                end if;
                if v_wnd < to_signed(32, 11) then
                    s_wmag <= (others => '0');
                else
                    s_wmag <= unsigned(v_wnd(9 downto 6));
                end if;
                s_sw_smoke    <= registers_in(6)(3);


                -- wind: fractional accumulator -> at most one 1-cell step
                -- per carry; bursts and smoke drift, rockets stay clean
                v_wst := '0';
                if s_wmag /= 0 then
                    if resize(s_wacc, 7) + resize(s_wmag, 7) >= to_unsigned(32, 7) then
                        s_wacc <= resize(resize(s_wacc, 7) + resize(s_wmag, 7)
                                         - to_unsigned(32, 7), 6);
                        v_wst := '1';
                    else
                        s_wacc <= s_wacc + s_wmag;
                    end if;
                end if;

                -- age / spawn / transition each slot
                for k in 0 to 3 loop
                    case s_sl_state(k) is
                        when "00" =>   -- idle: wait out the relaunch delay
                            if s_sl_age(k) >= s_sl_delay(k) then
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
                                elsif v_bxs > signed(resize(s_cwu - 32, 11)) then
                                    v_bxs := signed(resize(s_cwu - 32, 11));
                                end if;
                                s_sl_cx(k)    <= unsigned(v_bxs(8 downto 0));
                                s_sl_cy(k)    <= s_ground;
                                s_sl_apy(k)   <= s_apbase + resize(v_seed(10 downto 6), 8);
                                -- type: knob zone, or seed-picked for Mix
                                if s_typeSel = "000" then
                                    v_t3 := v_seed(8 downto 6);
                                    if v_t3 > 4 then v_t3 := v_t3 - 5; end if;
                                else
                                    v_t3 := s_typeSel - 1;
                                end if;
                                s_sl_typ(k) <= v_t3;
                                -- color: Mix -> random inner/outer pair;
                                -- pinned -> white-hot core into chosen color
                                if s_colSel = "000" then
                                    s_sl_col(k)  <= v_seed(13 downto 11);
                                    s_sl_col2(k) <= v_seed(13 downto 11)
                                                    + to_unsigned(2, 3)
                                                    + resize(v_seed(15 downto 14), 3);
                                else
                                    s_sl_col(k)  <= "111";
                                    s_sl_col2(k) <= C_COLMAP(to_integer(s_colSel));
                                end if;
                                s_sl_rsp(k) <= to_unsigned(2, 7);
                                -- spark count (palm: few thick arms)
                                if v_t3 = C_TY_PALM then
                                    s_sl_n(k) <= to_unsigned(10, 6);
                                else
                                    v_n7 := to_unsigned(10, 7) + resize(s_dens, 7);
                                    if v_n7 > 60 then v_n7 := to_unsigned(60, 7); end if;
                                    s_sl_n(k) <= v_n7(5 downto 0);
                                end if;
                            else
                                s_sl_age(k) <= s_sl_age(k) + 1;
                            end if;
                        when "01" =>   -- launch: 64-frame ascent
                            if s_sl_age(k) = 63 then
                                s_sl_state(k) <= "10";
                                s_sl_age(k)   <= (others => '0');
                                s_sl_cy(k)    <= s_sl_apy(k);
                            else
                                s_sl_age(k) <= s_sl_age(k) + 1;
                            end if;
                        when "10" =>   -- burst: 128-frame bloom and fade
                            if s_sl_age(k) = 127 then
                                if s_sw_smoke = '1' then
                                    s_sl_state(k) <= "11";
                                else
                                    s_sl_state(k) <= "00";
                                    s_sl_delay(k) <= s_rateD
                                        + resize(s_sl_seed(k)(4 downto 0), 10);
                                end if;
                                s_sl_age(k)   <= (others => '0');
                            else
                                s_sl_age(k) <= s_sl_age(k) + 1;
                            end if;
                        when others => -- "11" smoke: rise; sideways only by wind
                            if s_sl_age(k) = 95 then
                                s_sl_state(k) <= "00";
                                s_sl_age(k)   <= (others => '0');
                                s_sl_delay(k) <= s_rateD + resize(s_sl_seed(k)(4 downto 0), 10);
                            else
                                s_sl_age(k) <= s_sl_age(k) + 1;
                                if s_sl_age(k)(2 downto 0) = "000"
                                   and s_sl_cy(k) > to_unsigned(10, 8) then
                                    s_sl_cy(k) <= s_sl_cy(k) - 1;
                                end if;
                            end if;
                    end case;
                    -- wind drift (bursts + smoke; not idle pads, not rockets)
                    if v_wst = '1'
                       and (s_sl_state(k) = "10" or s_sl_state(k) = "11") then
                        if s_wpos = '1' then
                            if s_sl_cx(k) < resize(s_cwu - 16, 9) then
                                s_sl_cx(k) <= s_sl_cx(k) + 1;
                            end if;
                        else
                            if s_sl_cx(k) > to_unsigned(16, 9) then
                                s_sl_cx(k) <= s_sl_cx(k) - 1;
                            end if;
                        end if;
                    end if;
                end loop;

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
    begin
        if rising_edge(clk) then
            s_pen_we <= '0';
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
                    when 1 =>      -- write the cell
                        s_pen_we    <= '1';
                        s_pen_wdata <= s_w_val;
                        s_pen_waddr <= f_addr(s_wk_y, s_wk_x);
                        if (s_wk_x = s_wk_tx and s_wk_y = s_wk_ty)
                           or s_w_cnt = to_unsigned(24, 5) then
                            if s_w_head = '1' then
                                s_w_step <= 2;
                            else
                                s_w_active <= '0';
                            end if;
                        else
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
        variable v_ci : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            v_t := s_t6;
            -- luma = t*4 (t <= 192 -> max 768: full-scale Y clips all RGB
            -- channels on real displays and washes saturated chroma to white)
            v_y := resize(v_t & "00", 11);
            if s_insq6 = '1' and v_t /= 0 then
                s_d_y <= v_y(9 downto 0);
                if s_far6 = '1' then
                    v_ci := s_aco(to_integer(s_cidx6));
                else
                    v_ci := s_aci(to_integer(s_cidx6));
                end if;
                if s_agray(to_integer(s_cidx6)) = '1' then
                    s_d_u <= to_unsigned(512, 10);
                    s_d_v <= to_unsigned(512, 10);
                else
                    s_d_u <= C_PAL_U(to_integer(v_ci));
                    s_d_v <= C_PAL_V(to_integer(v_ci));
                end if;
                if v_t >= 32 then s_drawn <= '1'; else s_drawn <= '0'; end if;
            else
                s_d_y   <= (others => '0');
                s_d_u   <= to_unsigned(512, 10);
                s_d_v   <= to_unsigned(512, 10);
                s_drawn <= '0';
            end if;
        end if;
    end process p_color;

    -- ====================================================================
    -- sync / video delay (match pipeline depth)
    -- ====================================================================
    p_sr : process(clk)
    begin
        if rising_edge(clk) then
            s_y_sr(0)     <= data_in.y;
            s_u_sr(0)     <= data_in.u;
            s_v_sr(0)     <= data_in.v;
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LATENCY - 1 loop
                s_y_sr(i)     <= s_y_sr(i - 1);
                s_u_sr(i)     <= s_u_sr(i - 1);
                s_v_sr(i)     <= s_v_sr(i - 1);
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_sr;

    -- ====================================================================
    -- Output: drawn pixels win; empty pixels show video (S11) or black
    -- ====================================================================
    data_out.hsync_n <= s_hsync_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);

    data_out.y <= s_y_sr(C_LATENCY - 1) when (s_drawn = '0' and s_bg_video = '1')
                  else std_logic_vector(s_d_y);
    data_out.u <= s_u_sr(C_LATENCY - 1) when (s_drawn = '0' and s_bg_video = '1')
                  else std_logic_vector(s_d_u);
    data_out.v <= s_v_sr(C_LATENCY - 1) when (s_drawn = '0' and s_bg_video = '1')
                  else std_logic_vector(s_d_v);

end architecture fireworks;
