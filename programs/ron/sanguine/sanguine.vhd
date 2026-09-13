-- sanguine.vhd
-- Copyright (C) 2026  bluecondition
-- SPDX-License-Identifier: GPL-3.0-only
--
-- SANGUINE - realistic blood dripping from the top edge of the frame.
--
-- The frame is divided into vertical drip lanes (16 px pitch in HD, 8 px in
-- SD).  Each lane runs an independent drip lifecycle in a per-frame vblank
-- walk: a bead swells at the top edge (surface tension), breaks, and falls
-- with stuttering gravity (creep / pause / surge), leaving a tapering wet
-- trail that darkens with age and can dry into a stain.  Rivulet paths
-- meander via per-lane-phased triangle waves of y ("grooves" in an invisible
-- surface: repeat drips retrace the same path, like real runs on glass).
--
-- Rendering is raster-order with NO frame buffer: a sliding 3-lane window
-- (prev / cur / next, prefetched during the scan) lets wandering paths cross
-- lane borders and visually merge with neighbours.
--
-- Colour path is parametric (K3 hue rotates around arterial red, K4 depth
-- shapes luma+saturation together) so other liquids are a later fork, not a
-- rewrite.  Compositing: over black or over the incoming video (S7); blood
-- is OPAQUE in both modes and the wet trail fades top-down until it is gone.
-- With S8 CLING, the input luma at the drip head acts
-- as a virtual surface: bright content slows and pools the drip.
--
-- Panel mapping note (brief asked for 8 knobs; hardware has 6):
--   WANDER rides K2 VISCOSITY (thick = meander + pauses, thin = straight),
--   GLINT lives on S11 (Matte/Gloss).
--
-- =====================================================================
-- Register / front-panel map (registers_in indices)
--   0x00 K1  Flow        emission rate / density
--   0x01 K2  Viscosity   thick+wandering+pausing <-> thin+straight+fast
--   0x02 K3  Hue         arterial red at centre, orange-red..violet-red
--   0x03 K4  Depth       bright fresh crimson <-> dark venous (luma+sat)
--   0x04 K5  Trail       wet trail length behind the head
--   0x05 K6  Coagulate   fade quickly <-> dry into permanent stains
--   0x06b0 S7  Source    Black / Video background
--   0x06b1 S8  Cling     drips ignore video / ride its luma contours
--   0x06b2 S9  Age       fresh start / pre-aged stains
--   0x06b3 S10 Purge     any flip clears stains + finished trails
--   0x06b4 S11 Sheen     Matte / Gloss (specular glints + wet sheen)
--   0x07 P12 Hemorrhage  surge gesture (glide-smoothed flow/speed boost)
-- =====================================================================
--
-- Colour: BT.601 YUV stored U/V-SWAPPED for the Videomancer hardware
-- convention (stored U = Cr, stored V = Cb), matching matrix_rain/ziffern.
-- Arterial crimson ~ Y=222 Cr=830 Cb=424 (10-bit).  In GHDL sim (standard
-- BT.601) hues therefore read wrong - judge structure in sim, not hue.
--
-- =====================================================================
-- Pipeline (one pixel/clock; 8 register stages input -> output):
--   S0 p_acc     pixel coords + lane/offset split
--   -- p_window  sliding 3-lane state window (shift at lane crossing,
--                prefetch next lane via the shared BRAM read port)
--   S1 p_s1      per-candidate snapshot: tri-wave raw values, cx, pos,
--                radius/width, age band (window regs -> per-pixel regs)
--   S1.5 p_wander  registered wander shift+sum and raw x/y differences
--   S2 p_geom    dx / dy-to-head / trail extent, clamps, taper width
--   S3 p_test    squares LUT, head/glint/trail/sheen/stain region tests
--   S4 p_prio    combine 3 candidates -> winning region + freshness
--   S5 p_pal     palette select (4-step fresh->dried) + sheen luma
--   S6 p_comp    composite vs delayed video / black, blanking gate
--   -- p_io      output registers
-- Sync passthrough C_SYNCD = 6 consumed at S6.
-- Latency is mode-independent (single path, fixed taps).
-- =====================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture sanguine of program_top is

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_SYNCD : integer := 6;

    -- lifecycle phases
    constant C_PH_IDLE : unsigned(1 downto 0) := "00";
    constant C_PH_BEAD : unsigned(1 downto 0) := "01";
    constant C_PH_FALL : unsigned(1 downto 0) := "10";
    constant C_PH_DONE : unsigned(1 downto 0) := "11";

    -- region codes (priority order)
    constant C_RG_NONE  : unsigned(2 downto 0) := "000";
    constant C_RG_STAIN : unsigned(2 downto 0) := "001";
    constant C_RG_TRAIL : unsigned(2 downto 0) := "010";
    constant C_RG_SHEEN : unsigned(2 downto 0) := "011";
    constant C_RG_HEAD  : unsigned(2 downto 0) := "100";
    constant C_RG_GLINT : unsigned(2 downto 0) := "101";

    -- squares LUTs for the head blob test.  Bead shape is baked into the
    -- vertical table: round = y^2, taller = (0.75y)^2, wider = (1.5y)^2
    -- (separate 1D constants - see the GHDL array-of-arrays crash note).
    type t_sq is array (0 to 15) of unsigned(7 downto 0);
    constant C_SQ : t_sq := (
        to_unsigned(  0,8), to_unsigned(  1,8), to_unsigned(  4,8), to_unsigned(  9,8),
        to_unsigned( 16,8), to_unsigned( 25,8), to_unsigned( 36,8), to_unsigned( 49,8),
        to_unsigned( 64,8), to_unsigned( 81,8), to_unsigned(100,8), to_unsigned(121,8),
        to_unsigned(144,8), to_unsigned(169,8), to_unsigned(196,8), to_unsigned(225,8));
    constant C_SQT : t_sq := (
        to_unsigned(  0,8), to_unsigned(  0,8), to_unsigned(  2,8), to_unsigned(  5,8),
        to_unsigned(  9,8), to_unsigned( 14,8), to_unsigned( 20,8), to_unsigned( 27,8),
        to_unsigned( 36,8), to_unsigned( 45,8), to_unsigned( 56,8), to_unsigned( 68,8),
        to_unsigned( 81,8), to_unsigned( 95,8), to_unsigned(110,8), to_unsigned(126,8));
    constant C_SQW : t_sq := (
        to_unsigned(  0,8), to_unsigned(  2,8), to_unsigned(  9,8), to_unsigned( 20,8),
        to_unsigned( 36,8), to_unsigned( 56,8), to_unsigned( 81,8), to_unsigned(110,8),
        to_unsigned(144,8), to_unsigned(182,8), to_unsigned(225,8), to_unsigned(255,8),
        to_unsigned(255,8), to_unsigned(255,8), to_unsigned(255,8), to_unsigned(255,8));

    -- Hue table: chroma offsets from 512, full saturation, K3 band 0..15.
    -- Band 8 = arterial crimson; 0 = orange-red; 15 = violet-red.
    -- (Cr = stored U, Cb = stored V - hardware U/V-swap convention.)
    type t_hue is array (0 to 15) of integer;
    constant C_HCR : t_hue := (312,316,318,320,322,322,322,320,
                               318,314,308,300,290,278,264,248);
    constant C_HCB : t_hue := (-200,-184,-168,-152,-136,-120,-108,-96,
                               -88,-72,-48,-24,0,28,56,84);

    --------------------------------------------------------------------------
    function mix16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x;
        v := v xor shift_left(v, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        v := v xor shift_right(v, 5);
        return v;
    end function;

    -- triangle wave, period 256: t -> -64..64
    function tri8(t : unsigned(7 downto 0)) return signed is
        variable v : signed(8 downto 0);
    begin
        v := signed('0' & t) - to_signed(128, 9);
        v := abs(v) - to_signed(64, 9);
        return resize(v, 8);
    end function;

    -- triangle wave, period 128: t -> -32..32
    function tri7(t : unsigned(6 downto 0)) return signed is
        variable v : signed(7 downto 0);
    begin
        v := signed('0' & t) - to_signed(64, 8);
        v := abs(v) - to_signed(32, 8);
        return resize(v, 8);
    end function;

    function u_sat10(v : integer) return unsigned is
        variable r : integer;
    begin
        r := v;
        if r < 0 then r := 0; elsif r > 1023 then r := 1023; end if;
        return to_unsigned(r, 10);
    end function;

    --------------------------------------------------------------------------
    -- Controls
    --------------------------------------------------------------------------
    signal k_flow, k_visc, k_hue, k_depth, k_trail, k_coag, k_slider
        : unsigned(9 downto 0);
    signal sw_video, sw_cling, sw_aged, sw_purge, sw_gloss : std_logic;

    --------------------------------------------------------------------------
    -- Timing / resolution (matrix_rain-proven infrastructure)
    --------------------------------------------------------------------------
    signal s_timing  : t_video_timing_port;
    signal s_h_count : unsigned(11 downto 0);
    signal s_v_count : unsigned(11 downto 0);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_vsync_prev  : std_logic := '1';
    signal s_vsync_pulse : std_logic := '0';
    signal s_firstline   : std_logic := '1';
    signal s_x : unsigned(11 downto 0) := (others => '0');
    signal s_y : unsigned(11 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-frame parameters (vblank sequencer, power-of-2 slots)
    --------------------------------------------------------------------------
    signal s_seq     : unsigned(2 downto 0) := "111";
    signal s_frame   : unsigned(15 downto 0) := (others => '0');
    signal s_hem     : unsigned(9 downto 0) := (others => '0');  -- hemorrhage glide
    signal s_purge   : std_logic := '0';
    signal s_s10_prev : std_logic := '0';
    signal s_go      : std_logic := '0';   -- start the lane walk

    signal s_lshift  : integer range 3 to 4 := 4;
    signal s_small   : std_logic := '0';
    signal s_nlanes  : unsigned(7 downto 0) := to_unsigned(120, 8);
    signal s_nrows   : unsigned(11 downto 0) := to_unsigned(1080, 12);

    -- colour path
    signal s_dcr, s_dcb : signed(10 downto 0) := (others => '0');
    signal s_y0      : unsigned(9 downto 0) := to_unsigned(280, 10);
    signal pal_y0, pal_u0, pal_v0 : unsigned(9 downto 0) := (others => '0');
    signal pal_y1, pal_u1, pal_v1 : unsigned(9 downto 0) := (others => '0');
    signal pal_y2, pal_u2, pal_v2 : unsigned(9 downto 0) := (others => '0');
    signal pal_y3, pal_u3, pal_v3 : unsigned(9 downto 0) := (others => '0');
    signal head_y, head_u, head_v : unsigned(9 downto 0) := (others => '0');
    signal glint_y, glint_u, glint_v : unsigned(9 downto 0) := (others => '0');
    signal s_sheen_add : unsigned(7 downto 0) := (others => '0');

    -- physics constants
    signal c_spawn   : unsigned(15 downto 0) := (others => '0');
    signal c_beadthr : unsigned(7 downto 0) := to_unsigned(120, 8);
    signal c_beadrt  : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal c_grav    : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal c_velini  : unsigned(7 downto 0) := to_unsigned(8, 8);
    signal c_pp      : unsigned(7 downto 0) := to_unsigned(40, 8);
    signal c_dry     : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal c_tlen    : unsigned(11 downto 0) := to_unsigned(1000, 12);
    signal c_wa      : unsigned(1 downto 0) := "10";
    signal c_k6band  : unsigned(1 downto 0) := "10";
    -- per-bead release thresholds (0.5x/0.75x/1x/1.5x of c_beadthr) and the
    -- gush launch velocity, precomputed per frame
    signal c_bt0, c_bt1, c_bt2, c_bt3 : unsigned(8 downto 0) := (others => '0');
    signal c_gvel : unsigned(7 downto 0) := (others => '0');
    constant C_CLTH  : unsigned(9 downto 0) := to_unsigned(576, 10);

    --------------------------------------------------------------------------
    -- Per-lane state BRAMs (128 x 16 each, 1W1R)
    --   A: head position, signed Q12.4 rows
    --   B: [15:8] mass, [7:0] velocity Q4.4 rows/frame
    --   C: [15:14] phase, [13:6] age, [5:0] per-spawn seed
    --   D: [15:8] stain depth (rows/8), [7:0] stain strength
    --   E: [10:0] wet-top (rows; trail fades top-down to here),
    --      [12:11] drift (1S = diagonal jog, sign S), [14:13] shape
    --------------------------------------------------------------------------
    type t_lram is array (0 to 127) of std_logic_vector(15 downto 0);
    signal mem_a : t_lram := (others => (others => '0'));
    signal mem_b : t_lram := (others => (others => '0'));
    signal mem_c : t_lram := (others => (others => '0'));
    signal mem_d : t_lram := (others => (others => '0'));
    signal mem_e : t_lram := (others => (others => '0'));
    attribute ram_style : string;
    attribute ram_style of mem_a : signal is "block";
    attribute ram_style of mem_b : signal is "block";
    attribute ram_style of mem_c : signal is "block";
    attribute ram_style of mem_d : signal is "block";
    attribute ram_style of mem_e : signal is "block";

    signal s_ln_ra : unsigned(6 downto 0) := (others => '0');
    signal qa, qb, qc, qd, qe : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ln_we : std_logic := '0';
    signal s_ln_wa : unsigned(6 downto 0) := (others => '0');
    signal s_we_d : std_logic_vector(15 downto 0) := (others => '0');
    signal s_wa_d, s_wb_d, s_wc_d, s_wd_d
        : std_logic_vector(15 downto 0) := (others => '0');

    -- cling surface luma (write: render scan; read: vblank walk)
    type t_clram is array (0 to 127) of std_logic_vector(9 downto 0);
    signal mem_cl : t_clram := (others => (others => '0'));
    attribute ram_style of mem_cl : signal is "block";
    signal s_cl_ra : unsigned(6 downto 0) := (others => '0');
    signal q_cl    : std_logic_vector(9 downto 0) := (others => '0');
    signal s_cl_we : std_logic := '0';
    signal s_cl_wa : unsigned(6 downto 0) := (others => '0');
    signal s_cl_wd : std_logic_vector(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Lane walk FSM
    --------------------------------------------------------------------------
    type t_upd is (U_IDLE, U_REQ, U_W1, U_W2, U_LATCH, U_CALC, U_CALC2, U_WRITE);
    signal s_ustate : t_upd := U_IDLE;
    signal s_ucnt   : unsigned(7 downto 0) := (others => '0');
    signal s_busy   : std_logic := '0';
    signal s_upd_ra : unsigned(6 downto 0) := (others => '0');
    signal s_sp_h   : unsigned(15 downto 0) := (others => '0');
    signal s_pa_h   : unsigned(15 downto 0) := (others => '0');
    signal s_lfsr   : std_logic_vector(15 downto 0) := x"B10D";
    -- walk working registers (state is latched, computed, then packed over
    -- three cycles so no single cycle sees BRAM-out -> physics -> writeback)
    signal u_pos  : signed(15 downto 0) := (others => '0');
    signal u_vel, u_mass, u_age, u_std, u_sts : unsigned(7 downto 0) := (others => '0');
    signal u_phz  : unsigned(1 downto 0) := "00";
    signal u_tmr  : unsigned(5 downto 0) := (others => '0');
    signal u_wett : unsigned(10 downto 0) := (others => '0');
    signal u_drift, u_shp : unsigned(1 downto 0) := "00";
    signal u_cl_hit : std_logic := '0';
    signal u_dk_hit : std_logic := '0';   -- this lane decays its stain this frame
    signal u_dep_hit : std_logic := '0';  -- this lane deposits trail mass this frame
    signal u_drag_hit : std_logic := '0'; -- vel exceeds the mass-coupled cap
    signal u_grv : unsigned(7 downto 0) := (others => '0'); -- gravity + per-bead jitter
    signal u_rel_hit : std_logic := '0';  -- bead mass reached its release threshold
    signal u_runout  : std_logic := '0';  -- bead is (nearly) out of blood
    signal u_lvel : unsigned(7 downto 0) := (others => '0'); -- per-bead launch speed
    signal u_pa_hit : std_logic := '0';   -- stutter pause fires this frame
    signal u_vcap   : std_logic := '0';   -- velocity below the hard cap

    --------------------------------------------------------------------------
    -- Sliding 3-lane render window (0 = prev, 1 = cur, 2 = next)
    --------------------------------------------------------------------------
    type t_lst is record
        pos  : signed(15 downto 0);
        vel  : unsigned(7 downto 0);
        mass : unsigned(7 downto 0);
        ph   : unsigned(1 downto 0);
        age  : unsigned(7 downto 0);
        std  : unsigned(7 downto 0);
        sts  : unsigned(7 downto 0);
        ph1  : unsigned(7 downto 0);
        ph2  : unsigned(7 downto 0);
        pres : unsigned(7 downto 0);
        cx   : unsigned(11 downto 0);
        wett : unsigned(10 downto 0);
        drift : unsigned(1 downto 0);
        shp   : unsigned(1 downto 0);
        joff : signed(4 downto 0);   -- jog offset for the CURRENT line
        jact : std_logic;            -- on the diagonal segment right now
        vld  : std_logic;
    end record;
    type t_lwin is array (0 to 2) of t_lst;
    signal w : t_lwin := (others => (
        pos => (others => '0'), vel => (others => '0'), mass => (others => '0'),
        ph => "00", age => (others => '0'), std => (others => '0'),
        sts => (others => '0'), ph1 => (others => '0'), ph2 => (others => '0'),
        pres => (others => '0'), cx => (others => '0'),
        wett => (others => '0'), drift => "00", shp => "00",
        joff => (others => '0'), jact => '0', vld => '0'));

    signal s_pf_cnt  : unsigned(2 downto 0) := "111";  -- line-start prefetch
    signal s_pf_lane : unsigned(7 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    -- S0
    signal s0_x, s0_y : unsigned(11 downto 0) := (others => '0');
    signal s0_off  : unsigned(3 downto 0) := (others => '0');
    signal s0_lane : unsigned(6 downto 0) := (others => '0');
    signal s0_render : std_logic := '0';

    -- S1: per-candidate snapshot
    type t_s8a  is array (0 to 2) of signed(7 downto 0);
    type t_s6a  is array (0 to 2) of signed(5 downto 0);
    type t_u12a is array (0 to 2) of unsigned(11 downto 0);
    type t_s12a is array (0 to 2) of signed(11 downto 0);
    type t_u2a  is array (0 to 2) of unsigned(1 downto 0);
    type t_u3a  is array (0 to 2) of unsigned(2 downto 0);
    type t_u4a  is array (0 to 2) of unsigned(3 downto 0);
    type t_u8a  is array (0 to 2) of unsigned(7 downto 0);
    type t_sla  is array (0 to 2) of std_logic;
    signal s1_x, s1_y : unsigned(11 downto 0) := (others => '0');
    signal s1_lane : unsigned(6 downto 0) := (others => '0');
    signal s1_render : std_logic := '0';
    signal s1_tri1, s1_tri2 : t_s8a := (others => (others => '0'));
    signal s1_cx   : t_u12a := (others => (others => '0'));
    signal s1_pos  : t_s12a := (others => (others => '0'));
    signal s1_ph   : t_u2a := (others => "00");
    signal s1_r    : t_u4a := (others => "0010");
    signal s1_fast : t_sla := (others => '0');
    signal s1_agep : t_u2a := (others => "00");
    signal s1_doff : t_s6a := (others => (others => '0'));  -- diagonal jog
    signal s1_drf  : t_sla := (others => '0');
    signal s1_shp  : t_u2a := (others => "00");
    signal s1_wetok : t_sla := (others => '0');   -- y below the fade edge
    signal s1_std  : t_u8a := (others => (others => '0'));
    signal s1_sts  : t_u8a := (others => (others => '0'));
    signal s1_vld  : t_sla := (others => '0');

    -- S1.5: registered wander + raw difference terms (splits the old S2
    -- cone: variable shift + two 13-bit subtracts were the critical path)
    type t_s13a is array (0 to 2) of signed(12 downto 0);
    type t_s9a  is array (0 to 2) of signed(8 downto 0);
    signal s15_x, s15_y : unsigned(11 downto 0) := (others => '0');
    signal s15_lane : unsigned(6 downto 0) := (others => '0');
    signal s15_render : std_logic := '0';
    signal s15_wan  : t_s9a := (others => (others => '0'));
    signal s15_dxb  : t_s13a := (others => (others => '0'));
    signal s15_dyhb : t_s13a := (others => (others => '0'));
    signal s15_ph   : t_u2a := (others => "00");
    signal s15_r    : t_u4a := (others => "0010");
    signal s15_fast : t_sla := (others => '0');
    signal s15_agep : t_u2a := (others => "00");
    signal s15_drf  : t_sla := (others => '0');
    signal s15_shp  : t_u2a := (others => "00");
    signal s15_wetok : t_sla := (others => '0');
    signal s15_std  : t_u8a := (others => (others => '0'));
    signal s15_sts  : t_u8a := (others => (others => '0'));
    signal s15_vld  : t_sla := (others => '0');

    -- S2: geometry
    signal s2_x, s2_y : unsigned(11 downto 0) := (others => '0');
    signal s2_lane : unsigned(6 downto 0) := (others => '0');
    signal s2_render : std_logic := '0';
    signal s2_dx  : t_s6a := (others => (others => '0'));
    signal s2_dye : t_s6a := (others => (others => '0'));  -- shaped head dy
    signal s2_tup : t_u12a := (others => (others => '0'));
    signal s2_tupok : t_sla := (others => '0');
    signal s2_tw  : t_u4a := (others => "0001");
    signal s2_ph  : t_u2a := (others => "00");
    signal s2_r   : t_u4a := (others => "0010");
    signal s2_fast : t_sla := (others => '0');
    signal s2_agep : t_u2a := (others => "00");
    signal s2_drf  : t_sla := (others => '0');
    signal s2_shp  : t_u2a := (others => "00");
    signal s2_wetok : t_sla := (others => '0');
    signal s2_std : t_u8a := (others => (others => '0'));
    signal s2_sts : t_u8a := (others => (others => '0'));
    signal s2_vld : t_sla := (others => '0');
    signal s2_dith : std_logic := '0';

    -- S3: per-candidate region flags + freshness (priority encoding is S4's)
    signal s3_head, s3_glint, s3_trail, s3_sheen, s3_stain
        : t_sla := (others => '0');
    signal s3_vld  : t_sla := (others => '0');
    signal s3_fidx : t_u2a := (others => "00");
    signal s3_drf  : t_sla := (others => '0');
    signal s3_sts  : t_u8a := (others => (others => '0'));
    signal s3_render : std_logic := '0';

    -- S4: winner
    signal s4_code : unsigned(2 downto 0) := "000";
    signal s4_fidx : unsigned(1 downto 0) := "00";
    signal s4_sts  : unsigned(7 downto 0) := (others => '0');
    signal s4_drf  : std_logic := '0';
    signal s4_render : std_logic := '0';

    -- S5: palette
    signal s5_py, s5_pu, s5_pv : unsigned(9 downto 0) := (others => '0');
    signal s5_code : unsigned(2 downto 0) := "000";
    signal s5_render : std_logic := '0';

    -- S6: final colour
    signal s6_y, s6_u, s6_v : unsigned(9 downto 0) := (others => '0');
    signal s6_sync : std_logic_vector(4 downto 0) := (others => '0');

    -- sync + video delay lines
    type t_syncp is array (0 to C_SYNCD) of std_logic_vector(4 downto 0);
    signal s_syncp : t_syncp := (others => (others => '0'));
    type t_vd is array (0 to 6) of unsigned(9 downto 0);
    signal vd_y : t_vd := (others => (others => '0'));
    signal vd_u : t_vd := (others => C_MID);
    signal vd_v : t_vd := (others => C_MID);

    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Control reads
    --------------------------------------------------------------------------
    k_flow   <= unsigned(registers_in(0));
    k_visc   <= unsigned(registers_in(1));
    k_hue    <= unsigned(registers_in(2));
    k_depth  <= unsigned(registers_in(3));
    k_trail  <= unsigned(registers_in(4));
    k_coag   <= unsigned(registers_in(5));
    sw_video <= registers_in(6)(0);
    sw_cling <= registers_in(6)(1);
    sw_aged  <= registers_in(6)(2);
    sw_purge <= registers_in(6)(3);
    sw_gloss <= registers_in(6)(4);
    k_slider <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Timing infrastructure
    --------------------------------------------------------------------------
    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    pixel_counter_inst : entity work.pixel_counter
        port map (clk => clk, timing => s_timing,
                  h_count => s_h_count, v_count => s_v_count);

    p_measure_resolution : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then s_measured_h <= s_h_pixel_counter; end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;
            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then s_measured_v <= s_v_line_counter; end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    p_vsync_edge : process(clk)
    begin
        if rising_edge(clk) then
            s_vsync_prev <= s_timing.vsync_n;
            if s_vsync_prev = '0' and s_timing.vsync_n = '1' then
                s_vsync_pulse <= '1';
            else
                s_vsync_pulse <= '0';
            end if;
        end if;
    end process;

    p_firstline : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_firstline <= '1';
            elsif s_timing.avid_start = '1' then
                s_firstline <= '0';
            end if;
        end if;
    end process;

    p_lfsr : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            v_fb := s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10);
            s_lfsr <= s_lfsr(14 downto 0) & v_fb;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame vblank sequencer: hemorrhage glide, palette, physics consts.
    -- Each slot is shallow shift/add logic; the lane walk starts at slot 6.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_diff : signed(11 downto 0);
        variable v_h    : signed(11 downto 0);
        variable v_dcr, v_dcb : signed(10 downto 0);
        variable v_sp  : unsigned(19 downto 0);
        variable v_i   : integer;
    begin
        if rising_edge(clk) then
            s_go <= '0';
            if s_vsync_pulse = '1' then
                s_seq   <= "000";
                s_frame <= s_frame + 1;
                -- purge on ANY flip of S10 (momentary-style)
                s_s10_prev <= sw_purge;
                s_purge <= sw_purge xor s_s10_prev;
                -- hemorrhage glide: fast attack, slow drain (12-bit math;
                -- narrowing resize would wrap values > 511)
                v_diff := signed(resize(k_slider, 12)) - signed(resize(s_hem, 12));
                if v_diff > 0 then
                    v_h := signed(resize(s_hem, 12)) + shift_right(v_diff, 2);
                else
                    v_h := signed(resize(s_hem, 12)) + shift_right(v_diff, 4);
                end if;
                if v_h < 0 then v_h := (others => '0');
                elsif v_h > 1023 then v_h := to_signed(1023, 12);
                end if;
                s_hem <= unsigned(v_h(9 downto 0));
            elsif s_seq /= "111" then
                s_seq <= s_seq + 1;
                case to_integer(s_seq) is
                    when 0 =>
                        -- geometry
                        if s_measured_h < 1024 then
                            s_lshift <= 3; s_small <= '1';
                            s_nlanes <= resize(shift_right(s_measured_h, 3), 8);
                        else
                            s_lshift <= 4; s_small <= '0';
                            s_nlanes <= resize(shift_right(s_measured_h, 4), 8);
                        end if;
                        s_nrows <= s_measured_v;
                    when 1 =>
                        -- hue lookup + depth saturation shaping
                        v_i := to_integer(k_hue(9 downto 6));
                        v_dcr := to_signed(C_HCR(v_i), 11);
                        v_dcb := to_signed(C_HCB(v_i), 11);
                        case to_integer(k_depth(9 downto 8)) is
                            when 0 => null;
                            when 1 => v_dcr := v_dcr - shift_right(v_dcr, 3);
                                      v_dcb := v_dcb - shift_right(v_dcb, 3);
                            when 2 => v_dcr := v_dcr - shift_right(v_dcr, 2);
                                      v_dcb := v_dcb - shift_right(v_dcb, 2);
                            when others =>
                                      v_dcr := v_dcr - shift_right(v_dcr, 2)
                                                     - shift_right(v_dcr, 3);
                                      v_dcb := v_dcb - shift_right(v_dcb, 2)
                                                     - shift_right(v_dcb, 3);
                        end case;
                        s_dcr <= v_dcr;
                        s_dcb <= v_dcb;
                        s_y0  <= u_sat10(315 - to_integer(k_depth(9 downto 3))
                                             - to_integer(k_depth(9 downto 4)));
                    when 2 =>
                        -- fresh + head colours
                        pal_y0 <= s_y0;
                        pal_u0 <= u_sat10(512 + to_integer(s_dcr));
                        pal_v0 <= u_sat10(512 + to_integer(s_dcb));
                        -- head close to the trail colour (same liquid);
                        -- the glint provides the pop
                        head_y <= u_sat10(to_integer(s_y0) + 90);
                        head_u <= u_sat10(512 + to_integer(s_dcr)
                                              - to_integer(shift_right(s_dcr, 3)));
                        head_v <= u_sat10(512 + to_integer(s_dcb));
                        -- subtle pink-white spec, not a stark white dot
                        glint_y <= to_unsigned(870, 10);
                        glint_u <= to_unsigned(590, 10);
                        glint_v <= to_unsigned(470, 10);
                        if sw_gloss = '1' then s_sheen_add <= to_unsigned(96, 8);
                        else                   s_sheen_add <= (others => '0');
                        end if;
                    when 3 =>
                        -- ageing palette: darker, browner (Cb drifts up)
                        pal_y1 <= s_y0 - shift_right(s_y0, 3);
                        pal_u1 <= u_sat10(512 + to_integer(s_dcr)
                                              - to_integer(shift_right(s_dcr, 3)));
                        pal_v1 <= u_sat10(512 + to_integer(s_dcb) + 8);
                        pal_y2 <= u_sat10(to_integer(shift_right(s_y0, 1)) + 20);
                        pal_u2 <= u_sat10(512 + to_integer(shift_right(s_dcr, 1))
                                              + to_integer(shift_right(s_dcr, 3)));
                        pal_v2 <= u_sat10(512 + to_integer(shift_right(s_dcb, 1)) + 12);
                        pal_y3 <= to_unsigned(78, 10);
                        pal_u3 <= u_sat10(512 + to_integer(shift_right(s_dcr, 2)) + 8);
                        pal_v3 <= u_sat10(512 + to_integer(shift_right(s_dcb, 2)) - 8);
                    when 4 =>
                        -- physics constants (hemorrhage folded in)
                        v_sp := shift_left(to_unsigned(4, 20), to_integer(k_flow(9 downto 6)))
                                + shift_left(resize(s_hem, 20), 5);
                        if v_sp > 65535 then c_spawn <= (others => '1');
                        else                 c_spawn <= resize(v_sp, 16);
                        end if;
                        -- VISCOSITY carries double weight: threshold, speed,
                        -- gravity and pauses all sweep their full ranges
                        c_beadthr <= to_unsigned(240, 8)
                                     - resize(k_visc(9 downto 3), 8)
                                     - resize(k_visc(9 downto 4), 8)
                                     - resize(k_visc(9 downto 5), 8);
                        c_beadrt  <= resize(to_unsigned(1, 4) + k_flow(9 downto 8)
                                            + s_hem(9 downto 8), 4);
                        if s_small = '1' then
                            c_grav <= resize(to_unsigned(1, 5)
                                      + resize(k_visc(9 downto 7), 5)
                                      + resize(s_hem(9 downto 8), 5), 5);
                        else
                            c_grav <= resize(to_unsigned(1, 5)
                                      + resize(k_visc(9 downto 6), 5)
                                      + resize(s_hem(9 downto 7), 5), 5);
                        end if;
                        c_velini  <= resize(to_unsigned(2, 8) + k_visc(9 downto 5), 8);
                        -- pauses: thick pauses a lot, hemorrhage suppresses them
                        v_i := 8 + to_integer(not k_visc(9 downto 2))
                               - to_integer(s_hem(9 downto 4));
                        if v_i > 255 then v_i := 255;
                        elsif v_i < 0 then v_i := 0;
                        end if;
                        c_pp <= to_unsigned(v_i, 8);
                        -- darkening is slow everywhere; K6 mostly picks the
                        -- BEHAVIOUR (bright-red / distance / age+distance)
                        c_dry <= resize(to_unsigned(1, 4)
                                 + resize(k_coag(9 downto 7), 4), 4);
                        c_tlen <= to_unsigned(8, 12)
                                  + shift_left(resize(k_trail, 12), 1);
                        c_wa   <= "11" - k_visc(9 downto 8);
                        c_k6band <= k_coag(9 downto 8);
                    when 5 =>
                        c_bt0 <= '0' & shift_right(c_beadthr, 1);
                        c_bt1 <= resize(c_beadthr, 9)
                                 - resize(shift_right(c_beadthr, 2), 9);
                        c_bt2 <= resize(c_beadthr, 9);
                        -- clamp below the 250 mass cap or 1.5x beads at the
                        -- thick end would never release
                        if resize(c_beadthr, 9) + resize(shift_right(c_beadthr, 1), 9)
                           > 245 then
                            c_bt3 <= to_unsigned(245, 9);
                        else
                            c_bt3 <= resize(c_beadthr, 9)
                                     + resize(shift_right(c_beadthr, 1), 9);
                        end if;
                        c_gvel <= c_velini + resize(s_hem(9 downto 5), 8);
                    when 6 =>
                        s_go <= '1';
                    when others => null;
                end case;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Lane walk FSM: one lifecycle step per lane per frame, entirely in
    -- vblank.  Read latency to qa..qd is 3 cycles behind s_upd_ra.
    --------------------------------------------------------------------------
    p_walk : process(clk)
        variable v_pos  : signed(15 downto 0);
        variable v_pi   : signed(11 downto 0);
        variable v_vel  : unsigned(7 downto 0);
        variable v_mass : unsigned(7 downto 0);
        variable v_phz  : unsigned(1 downto 0);
        variable v_age  : unsigned(7 downto 0);
        variable v_tmr  : unsigned(5 downto 0);
        variable v_std  : unsigned(7 downto 0);
        variable v_sts  : unsigned(7 downto 0);
        variable v_vi   : integer;
        variable v_lim  : signed(12 downto 0);
        variable v_wett : unsigned(10 downto 0);
        variable v_drift, v_shp : unsigned(1 downto 0);
        variable v_thr  : unsigned(8 downto 0);
        variable v_vmax : unsigned(8 downto 0);
        variable v_er   : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            s_ln_we <= '0';
            case s_ustate is
                when U_IDLE =>
                    if s_go = '1' then
                        s_ucnt   <= (others => '0');
                        s_busy   <= '1';
                        s_ustate <= U_REQ;
                    end if;
                when U_REQ =>
                    s_upd_ra <= s_ucnt(6 downto 0);
                    s_ustate <= U_W1;
                when U_W1 =>
                    s_sp_h <= mix16(resize(s_ucnt, 16)
                              xor (s_frame(9 downto 0) & "000000") xor x"5DC1");
                    if s_frame(3 downto 0) = s_ucnt(3 downto 0) then
                        u_dk_hit <= '1';
                    else
                        u_dk_hit <= '0';
                    end if;
                    if s_frame(1 downto 0) = s_ucnt(1 downto 0) then
                        u_dep_hit <= '1';
                    else
                        u_dep_hit <= '0';
                    end if;
                    s_ustate <= U_W2;
                when U_W2 =>
                    s_pa_h <= mix16((s_frame(9 downto 2) & s_ucnt) xor x"0B3F");
                    s_ustate <= U_LATCH;
                when U_LATCH =>
                    -- BRAM outputs -> working registers (no logic this cycle)
                    u_pos  <= signed(qa);
                    u_mass <= unsigned(qb(15 downto 8));
                    u_vel  <= unsigned(qb(7 downto 0));
                    u_phz  <= unsigned(qc(15 downto 14));
                    u_age  <= unsigned(qc(13 downto 6));
                    u_tmr  <= unsigned(qc(5 downto 0));
                    u_std  <= unsigned(qd(15 downto 8));
                    u_sts  <= unsigned(qd(7 downto 0));
                    u_wett <= unsigned(qe(10 downto 0));
                    u_drift <= unsigned(qe(12 downto 11));
                    u_shp  <= unsigned(qe(14 downto 13));
                    -- drag test against last frame's velocity/mass (a frame
                    -- of lag is invisible; keeps the CALC cone shallow)
                    if resize(unsigned(qb(7 downto 0)), 9)
                       > to_unsigned(24, 9) + resize(unsigned(qb(15 downto 8)), 9) then
                        u_drag_hit <= '1';
                    else
                        u_drag_hit <= '0';
                    end if;
                    u_grv <= resize(c_grav, 8)
                             + resize(unsigned(qc(1 downto 0)), 8);
                    -- release / runout tests against last frame's mass
                    case qc(3 downto 2) is
                        when "00" => v_thr := c_bt0;
                        when "01" => v_thr := c_bt1;
                        when "10" => v_thr := c_bt2;
                        when others => v_thr := c_bt3;
                    end case;
                    if resize(unsigned(qb(15 downto 8)), 9) >= v_thr then
                        u_rel_hit <= '1';
                    else
                        u_rel_hit <= '0';
                    end if;
                    if unsigned(qb(15 downto 8)) <= 9 then
                        u_runout <= '1';
                    else
                        u_runout <= '0';
                    end if;
                    u_lvel <= shift_right(c_velini, 1)
                              + shift_left(resize(unsigned(qc(4 downto 2)), 8), 1);
                    if s_pa_h(7 downto 0) < c_pp then u_pa_hit <= '1';
                    else                              u_pa_hit <= '0';
                    end if;
                    if unsigned(qb(7 downto 0)) < 240 then u_vcap <= '1';
                    else                                   u_vcap <= '0';
                    end if;
                    if sw_cling = '1' and sw_video = '1'
                       and unsigned(q_cl) > C_CLTH then
                        u_cl_hit <= '1';
                    else
                        u_cl_hit <= '0';
                    end if;
                    s_ustate <= U_CALC;
                when U_CALC =>
                    -- velocity / mass / phase transitions only (position and
                    -- the slower bookkeeping run in U_CALC2 so neither cycle
                    -- carries the whole physics cone)
                    v_mass := u_mass;
                    v_vel  := u_vel;
                    v_phz  := u_phz;
                    v_tmr  := u_tmr;
                    v_wett := u_wett;
                    v_drift := u_drift;
                    v_shp  := u_shp;

                    case v_phz is
                        when C_PH_IDLE =>
                            u_pos <= to_signed(-64, 16);   -- park 4 rows up
                            if s_sp_h < c_spawn then
                                -- per-bead draws: seed, diagonal jog (25%),
                                -- shape; trail fade edge resets to the top
                                v_tmr := unsigned(s_lfsr(5 downto 0));
                                if s_lfsr(7 downto 6) = "11" then
                                    if s_lfsr(8) = '1' then v_drift := "11";
                                    else                    v_drift := "10";
                                    end if;
                                else
                                    v_drift := "00";
                                end if;
                                v_shp  := unsigned(s_lfsr(10 downto 9));
                                v_wett := (others => '0');
                                if s_hem > 512 then
                                    -- gushing wound: skip surface tension,
                                    -- burst straight into the fall
                                    v_phz  := C_PH_FALL;
                                    v_mass := to_unsigned(96, 8)
                                              + resize(v_tmr(3 downto 0), 8);
                                    v_vel  := c_gvel;
                                else
                                    v_phz  := C_PH_BEAD;
                                    v_mass := to_unsigned(4, 8);
                                    v_vel  := (others => '0');
                                end if;
                            end if;
                        when C_PH_BEAD =>
                            -- swell; tmr(5) beads fill at quarter rate (the
                            -- long, patient drips).  Release threshold and
                            -- launch speed were resolved at U_LATCH.
                            if v_mass < 250 and
                               (v_tmr(5) = '0' or s_frame(1 downto 0) = "00") then
                                v_mass := v_mass + resize(c_beadrt, 8);
                            end if;
                            if u_rel_hit = '1' then
                                v_phz := C_PH_FALL;
                                v_vel := u_lvel;
                            end if;
                        when C_PH_FALL =>
                            -- velocity: flat priority select of single-op
                            -- results (all selectors precomputed at U_LATCH);
                            -- pause/drag dominate gravity in any frame they
                            -- fire, so the serial compose is unnecessary
                            if u_pa_hit = '1' then
                                -- viscous stutter: creep / pause / surge
                                v_vel := shift_right(u_vel, 1);
                            elsif u_drag_hit = '1' then
                                -- drag: speed bleeds down as the bead empties
                                v_vel := u_vel - shift_right(u_vel, 2);
                            elsif u_cl_hit = '1' then
                                -- cling: bright video slows the drip
                                v_vel := shift_right(u_vel, 1);
                            elsif u_vcap = '1' then
                                v_vel := u_vel + u_grv;
                            end if;
                            -- cling pools mass regardless of the stutter
                            if u_cl_hit = '1' and v_mass < 250 then
                                v_mass := v_mass + 2;
                            end if;
                            -- deposit mass into the trail; when the bead runs
                            -- dry the drip ends mid-fall (test resolved at
                            -- U_LATCH from last frame's mass)
                            if u_dep_hit = '1' and v_mass > 0 then
                                v_mass := v_mass - 1;
                            end if;
                            if u_runout = '1' then
                                v_phz := C_PH_DONE;
                            end if;
                        when others => null;
                    end case;

                    u_mass <= v_mass;
                    u_vel  <= v_vel;
                    u_phz  <= v_phz;
                    u_tmr  <= v_tmr;
                    u_wett <= v_wett;
                    u_drift <= v_drift;
                    u_shp  <= v_shp;
                    s_ustate <= U_CALC2;

                when U_CALC2 =>
                    -- position integrate + stain / ageing / clear checks
                    v_pos := u_pos;
                    v_phz := u_phz;
                    v_age := u_age;
                    v_std := u_std;
                    v_sts := u_sts;
                    v_pi  := v_pos(15 downto 4);

                    case v_phz is
                        when C_PH_BEAD =>
                            -- bulge protrudes until it hangs at the edge
                            if v_pos < 0 then
                                v_pos := v_pos + to_signed(4, 16);
                            end if;
                        when C_PH_FALL =>
                            v_pos := v_pos + signed(resize(u_vel, 16));
                            -- paint the permanent record while passing:
                            -- depth (rows/8) + the bead's width
                            if v_pi > 0 then
                                v_vi := to_integer(unsigned(v_pi(10 downto 3)));
                                if v_vi > 255 then v_vi := 255; end if;
                                if v_vi > to_integer(v_std) then
                                    v_std := to_unsigned(v_vi, 8);
                                end if;
                                if s_small = '1' then
                                    v_vi := 2 + to_integer(u_mass(7 downto 6));
                                else
                                    v_vi := 3 + to_integer(u_mass(7 downto 5));
                                end if;
                                if v_vi > to_integer(v_sts) then
                                    v_sts := to_unsigned(v_vi, 8);
                                end if;
                            end if;
                            v_lim := signed(resize(s_nrows, 13)) + 12;
                            if resize(v_pos(15 downto 4), 13) > v_lim then
                                v_phz := C_PH_DONE;
                            end if;
                        when C_PH_DONE =>
                            -- drip over: the painted record stays; respawn
                            -- immediately so more blood falls on top
                            v_phz := C_PH_IDLE;
                            v_age := (others => '0');
                        when others => null;
                    end case;

                    u_pos <= v_pos;
                    u_phz <= v_phz;
                    u_age <= v_age;
                    u_std <= v_std;
                    u_sts <= v_sts;
                    s_ustate <= U_WRITE;

                when U_WRITE =>
                    v_phz := u_phz;
                    v_age := u_age;
                    v_std := u_std;
                    v_sts := u_sts;
                    v_wett := u_wett;

                    -- the painted record is permanent: no decay, no erosion.
                    -- purge clears the canvas, keeping live drips
                    if s_purge = '1' then
                        v_std := (others => '0');
                        v_sts := (others => '0');
                        if v_phz = C_PH_DONE then
                            v_phz := C_PH_IDLE;
                            v_age := (others => '0');
                        end if;
                    end if;

                    s_wa_d <= std_logic_vector(u_pos);
                    s_wb_d <= std_logic_vector(u_mass) & std_logic_vector(u_vel);
                    s_wc_d <= std_logic_vector(v_phz) & std_logic_vector(v_age)
                              & std_logic_vector(u_tmr);
                    s_wd_d <= std_logic_vector(v_std) & std_logic_vector(v_sts);
                    s_we_d <= '0' & std_logic_vector(u_shp)
                              & std_logic_vector(u_drift)
                              & std_logic_vector(v_wett);
                    s_ln_wa <= s_ucnt(6 downto 0);
                    s_ln_we <= '1';

                    if s_ucnt + 1 >= s_nlanes then
                        s_busy   <= '0';
                        s_ustate <= U_IDLE;
                    else
                        s_ucnt   <= s_ucnt + 1;
                        s_ustate <= U_REQ;
                    end if;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- BRAMs (1W1R each; read address shared between walk and prefetch)
    --------------------------------------------------------------------------
    p_ram_a : process(clk)
    begin
        if rising_edge(clk) then
            if s_ln_we = '1' then mem_a(to_integer(s_ln_wa)) <= s_wa_d; end if;
            qa <= mem_a(to_integer(s_ln_ra));
        end if;
    end process;

    p_ram_b : process(clk)
    begin
        if rising_edge(clk) then
            if s_ln_we = '1' then mem_b(to_integer(s_ln_wa)) <= s_wb_d; end if;
            qb <= mem_b(to_integer(s_ln_ra));
        end if;
    end process;

    p_ram_c : process(clk)
    begin
        if rising_edge(clk) then
            if s_ln_we = '1' then mem_c(to_integer(s_ln_wa)) <= s_wc_d; end if;
            qc <= mem_c(to_integer(s_ln_ra));
        end if;
    end process;

    p_ram_d : process(clk)
    begin
        if rising_edge(clk) then
            if s_ln_we = '1' then mem_d(to_integer(s_ln_wa)) <= s_wd_d; end if;
            qd <= mem_d(to_integer(s_ln_ra));
        end if;
    end process;

    p_ram_e : process(clk)
    begin
        if rising_edge(clk) then
            if s_ln_we = '1' then mem_e(to_integer(s_ln_wa)) <= s_we_d; end if;
            qe <= mem_e(to_integer(s_ln_ra));
        end if;
    end process;

    p_ram_cl : process(clk)
    begin
        if rising_edge(clk) then
            if s_cl_we = '1' then mem_cl(to_integer(s_cl_wa)) <= s_cl_wd; end if;
            q_cl <= mem_cl(to_integer(s_cl_ra));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sync + video delay lines
    --------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_syncp(0) <= s_v_count(0) & data_in.field_n & data_in.avid &
                          data_in.vsync_n & data_in.hsync_n;
            for k in 1 to C_SYNCD loop
                s_syncp(k) <= s_syncp(k - 1);
            end loop;
            vd_y(0) <= unsigned(data_in.y);
            vd_u(0) <= unsigned(data_in.u);
            vd_v(0) <= unsigned(data_in.v);
            for k in 1 to 6 loop
                vd_y(k) <= vd_y(k - 1);
                vd_u(k) <= vd_u(k - 1);
                vd_v(k) <= vd_v(k - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S0: pixel coordinates + lane/offset split
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable cur_x, cur_y : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.avid_start = '1' then
                cur_x := (others => '0');
                if s_firstline = '1' then cur_y := (others => '0');
                else                      cur_y := s_y + 1; end if;
                s_x <= to_unsigned(1, 12);
                s_y <= cur_y;
            elsif s_timing.avid = '1' then
                cur_x := s_x; cur_y := s_y;
                s_x <= s_x + 1;
            else
                cur_x := s_x; cur_y := s_y;
            end if;

            s0_x <= cur_x;
            s0_y <= cur_y;
            if s_lshift = 3 then
                s0_lane <= resize(cur_x(11 downto 3), 7);
                s0_off  <= '0' & cur_x(2 downto 0);
            else
                s0_lane <= resize(cur_x(11 downto 4), 7);
                s0_off  <= cur_x(3 downto 0);
            end if;
            s0_render <= s_timing.avid or s_timing.avid_start;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sliding window: line-start prefetch (lanes 0,1), crossing shift +
    -- next-lane prefetch during the scan.  Owns the shared BRAM read addr
    -- (the walk FSM takes it while s_busy, which only happens in vblank).
    --------------------------------------------------------------------------
    p_window : process(clk)
        variable v_l    : unsigned(7 downto 0);
        variable v_h    : unsigned(15 downto 0);
        variable v_p    : unsigned(15 downto 0);
        variable v_new  : t_lst;
        variable v_dj   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            if s_busy = '1' then
                s_ln_ra <= s_upd_ra;
                s_cl_ra <= s_upd_ra;
            else
                -- line-start prefetch sequencer
                if s_timing.hsync_start = '1' then
                    s_pf_cnt <= "000";
                elsif s_pf_cnt /= "111" then
                    s_pf_cnt <= s_pf_cnt + 1;
                end if;

                case to_integer(s_pf_cnt) is
                    when 0 => s_ln_ra <= (others => '0');
                    when 1 => s_ln_ra <= to_unsigned(1, 7);
                    when others => null;
                end case;

                if s0_render = '1' and s0_off = 0 and s0_lane /= 0 then
                    s_ln_ra <= s0_lane + 1;
                end if;
            end if;

            -- latch helper: build a window slot from the BRAM outputs
            if s_busy = '0' then
                if (s_pf_cnt = 2) or (s_pf_cnt = 3)
                   or (s0_render = '1' and s0_off = 3 and s0_lane /= 0) then
                    if s_pf_cnt = 2 then
                        v_l := (others => '0');
                    elsif s_pf_cnt = 3 then
                        v_l := to_unsigned(1, 8);
                    else
                        v_l := resize(s0_lane, 8) + 1;
                    end if;
                    v_h := mix16(resize(v_l, 16) xor x"C0DE");
                    v_p := mix16(resize(v_l, 16) xor x"55AA");
                    v_new.pos  := signed(qa);
                    v_new.mass := unsigned(qb(15 downto 8));
                    v_new.vel  := unsigned(qb(7 downto 0));
                    v_new.ph   := unsigned(qc(15 downto 14));
                    v_new.age  := unsigned(qc(13 downto 6));
                    v_new.std  := unsigned(qd(15 downto 8));
                    v_new.sts  := unsigned(qd(7 downto 0));
                    v_new.wett := unsigned(qe(10 downto 0));
                    v_new.drift := unsigned(qe(12 downto 11));
                    v_new.shp  := unsigned(qe(14 downto 13));
                    v_new.ph1  := v_h(7 downto 0);
                    v_new.ph2  := v_h(15 downto 8);
                    -- diagonal jog is LANE-static (hash), not per-spawn, so
                    -- the painted record lies exactly on every drip's path
                    if v_p(5 downto 4) = "11" then
                        if v_p(3) = '1' then v_new.drift := "11";
                        else                 v_new.drift := "10";
                        end if;
                    else
                        v_new.drift := "00";
                    end if;
                    v_new.joff := (others => '0');   -- computed 1-2 cycles later
                    v_new.jact := '0';
                    if v_p(7) = '1' then
                        v_new.pres := '0' & v_p(6 downto 0);
                    else
                        v_new.pres := (others => '0');
                    end if;
                    if s_lshift = 3 then
                        v_new.cx := shift_left(resize(v_l, 12), 3) + 4;
                    else
                        v_new.cx := shift_left(resize(v_l, 12), 4) + 8;
                    end if;
                    if resize(v_l, 8) < s_nlanes then v_new.vld := '1';
                    else                              v_new.vld := '0';
                    end if;

                    if s_pf_cnt = 2 then
                        w(1) <= v_new;
                        w(0).vld <= '0';
                    else
                        w(2) <= v_new;
                    end if;
                end if;

                -- diagonal-jog offset: depends only on (line y, slot hash),
                -- so it is computed once per slot event - two cycles after
                -- the field latch, from registered record fields - and rides
                -- along when the window shifts.  Zero per-pixel logic.
                if (s_pf_cnt = 4)
                   or (s0_render = '1' and s0_off = 5 and s0_lane /= 0) then
                    v_dj := signed(resize(s0_y, 13))
                            - signed(resize(w(2).ph2 & "00", 13));
                    if w(2).drift(1) = '0' or v_dj <= 0 then
                        w(2).joff <= (others => '0');
                        w(2).jact <= '0';
                    else
                        v_dj := shift_right(v_dj, 2);
                        if v_dj >= 8 then
                            v_dj := to_signed(8, 13);
                            w(2).jact <= '0';
                        else
                            w(2).jact <= '1';
                        end if;
                        if w(2).drift(0) = '1' then
                            w(2).joff <= resize(v_dj, 5);
                        else
                            w(2).joff <= -resize(v_dj, 5);
                        end if;
                    end if;
                end if;
                if s_pf_cnt = 5 then
                    v_dj := signed(resize(s0_y, 13))
                            - signed(resize(w(1).ph2 & "00", 13));
                    if w(1).drift(1) = '0' or v_dj <= 0 then
                        w(1).joff <= (others => '0');
                        w(1).jact <= '0';
                    else
                        v_dj := shift_right(v_dj, 2);
                        if v_dj >= 8 then
                            v_dj := to_signed(8, 13);
                            w(1).jact <= '0';
                        else
                            w(1).jact <= '1';
                        end if;
                        if w(1).drift(0) = '1' then
                            w(1).joff <= resize(v_dj, 5);
                        else
                            w(1).joff <= -resize(v_dj, 5);
                        end if;
                    end if;
                end if;

                -- crossing shift: as the s0 pixel enters a new lane, the
                -- window re-centres (S1 consumers see it one stage later)
                if s0_render = '1' and s0_off = 0 and s0_lane /= 0 then
                    w(0) <= w(1);
                    w(1) <= w(2);
                end if;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S1: per-pixel snapshot of the 3 candidates (window regs are shared
    -- state; freeze everything this pixel needs into pipeline regs).
    --------------------------------------------------------------------------
    p_s1 : process(clk)
        variable v_t1 : unsigned(7 downto 0);
        variable v_t2 : unsigned(6 downto 0);
        variable v_r, v_wt : integer range 0 to 31;
    begin
        if rising_edge(clk) then
            s1_x <= s0_x; s1_y <= s0_y;
            s1_lane <= s0_lane;
            s1_render <= s0_render;
            for c in 0 to 2 loop
                v_t1 := resize(s0_y, 8) + w(c).ph1;
                v_t2 := resize(s0_y, 7) + w(c).ph2(6 downto 0);
                s1_tri1(c) <= tri8(v_t1);
                s1_tri2(c) <= tri7(v_t2);
                s1_cx(c)  <= w(c).cx;
                s1_pos(c) <= w(c).pos(15 downto 4);
                s1_ph(c)  <= w(c).ph;
                -- size follows mass; the wider VISCOSITY threshold range now
                -- carries the size spread (big blobs only at thick settings).
                -- Trail width is derived from r in S2 so drop and trail meet
                -- at exactly the same width.
                if s_small = '1' then
                    v_r := 2 + to_integer(w(c).mass(7 downto 6));
                else
                    v_r := 3 + to_integer(w(c).mass(7 downto 5));
                end if;
                s1_r(c)  <= to_unsigned(v_r, 4);

                -- diagonal jog: precomputed per slot in p_window
                s1_doff(c) <= resize(w(c).joff, 6);
                s1_drf(c)  <= w(c).jact;
                s1_shp(c)  <= w(c).shp;

                -- trail fade disabled: blood stays put until PURGE
                s1_wetok(c) <= '1';
                if w(c).vel >= 64 then s1_fast(c) <= '1';
                else                   s1_fast(c) <= '0';
                end if;
                if w(c).age(7 downto 6) = "11" then s1_agep(c) <= "11";
                else s1_agep(c) <= w(c).age(7 downto 6);
                end if;
                if sw_aged = '1' and w(c).pres /= 0 then
                    if w(c).pres > w(c).std then s1_std(c) <= w(c).pres;
                    else                         s1_std(c) <= w(c).std;
                    end if;
                    if w(c).sts < 4 then s1_sts(c) <= to_unsigned(4, 8);
                    else                 s1_sts(c) <= w(c).sts;
                    end if;
                else
                    s1_std(c) <= w(c).std;
                    s1_sts(c) <= w(c).sts;
                end if;
                s1_vld(c) <= w(c).vld;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S1.5: wander shift+sum and raw x/y differences (registered so S2 is
    -- only one subtract + clamps)
    --------------------------------------------------------------------------
    p_wander : process(clk)
        variable v_sh : integer range 3 to 8;
        variable v_w  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            s15_x <= s1_x; s15_y <= s1_y;
            s15_lane <= s1_lane;
            s15_render <= s1_render;
            for c in 0 to 2 loop
                v_sh := 6 - to_integer(c_wa);
                if s_small = '1' then v_sh := v_sh + 1; end if;
                -- jogging beads wander less (clean diagonal, and keeps the
                -- combined offset inside the 3-candidate window)
                if s1_doff(c) /= 0 then v_sh := v_sh + 1; end if;
                v_w := resize(shift_right(s1_tri1(c), v_sh), 10)
                     + resize(shift_right(s1_tri2(c), v_sh), 10)
                     + resize(s1_doff(c), 10);
                s15_wan(c) <= resize(v_w, 9);
                s15_dxb(c) <= signed(resize(s1_x, 13))
                              - signed(resize(s1_cx(c), 13));
                s15_dyhb(c) <= signed(resize(s1_y, 13))
                               - resize(s1_pos(c), 13);
                s15_ph(c)   <= s1_ph(c);
                s15_r(c)    <= s1_r(c);
                s15_fast(c) <= s1_fast(c);
                s15_agep(c) <= s1_agep(c);
                s15_std(c)  <= s1_std(c);
                s15_sts(c)  <= s1_sts(c);
                s15_vld(c)  <= s1_vld(c);
                s15_drf(c)  <= s1_drf(c);
                s15_shp(c)  <= s1_shp(c);
                s15_wetok(c) <= s1_wetok(c);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S2: final geometry per candidate
    --------------------------------------------------------------------------
    p_geom : process(clk)
        variable v_dx   : signed(12 downto 0);
        variable v_dyh  : signed(12 downto 0);
        variable v_tup  : signed(12 downto 0);
        variable v_tw   : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            s2_x <= s15_x; s2_y <= s15_y;
            s2_lane <= s15_lane;
            s2_render <= s15_render;
            s2_dith <= s15_x(0) xor s15_y(0);
            for c in 0 to 2 loop
                v_dx := s15_dxb(c) - resize(s15_wan(c), 13);
                if v_dx > 31 then v_dx := to_signed(31, 13);
                elsif v_dx < -31 then v_dx := to_signed(-31, 13);
                end if;
                s2_dx(c) <= resize(v_dx, 6);

                v_dyh := s15_dyhb(c);
                if v_dyh > 31 then v_dyh := to_signed(31, 13);
                elsif v_dyh < -31 then v_dyh := to_signed(-31, 13);
                end if;
                -- head-blob shaping: elongate when falling fast, droop below
                if s15_fast(c) = '1' and s15_ph(c) = C_PH_FALL then
                    v_dyh := shift_right(v_dyh, 1);
                end if;
                if v_dyh > 0 then
                    v_dyh := v_dyh - shift_right(v_dyh, 2);
                end if;
                s2_dye(c) <= resize(v_dyh, 6);

                v_tup := -s15_dyhb(c);
                if v_tup >= 0 then
                    s2_tupok(c) <= '1';
                    s2_tup(c) <= unsigned(v_tup(11 downto 0));
                else
                    s2_tupok(c) <= '0';
                    s2_tup(c) <= (others => '0');
                end if;

                -- trail width: EXACTLY the bead radius at the contact point,
                -- widening slowly with height above the head (the bead was
                -- fatter when it deposited there).  Drop is never wider than
                -- the trail touching it.
                v_tw := to_integer(s15_r(c));
                if v_tup >= 0 then
                    if v_tup(11 downto 8) /= "0000" then
                        v_tw := v_tw + 2;
                    elsif v_tup(7) = '1' then
                        v_tw := v_tw + 1;
                    end if;
                end if;
                if v_tw > 11 then v_tw := 11; end if;
                s2_tw(c) <= to_unsigned(v_tw, 4);

                s2_ph(c)   <= s15_ph(c);
                s2_r(c)    <= s15_r(c);
                s2_fast(c) <= s15_fast(c);
                s2_agep(c) <= s15_agep(c);
                s2_std(c)  <= s15_std(c);
                s2_sts(c)  <= s15_sts(c);
                s2_vld(c)  <= s15_vld(c);
                s2_drf(c)  <= s15_drf(c);
                s2_shp(c)  <= s15_shp(c);
                s2_wetok(c) <= s15_wetok(c);
            end loop;
        end if;
    end process;

    -- cling sample: when the beam sits exactly on the cur lane's path centre
    -- at the head row, capture the underlying video luma for the walk FSM.
    p_cling : process(clk)
    begin
        if rising_edge(clk) then
            s_cl_we <= '0';
            if s2_render = '1' and s2_vld(1) = '1'
               and s2_dx(1) = 0 and s2_dye(1) = 0 then
                s_cl_we <= '1';
                s_cl_wa <= s2_lane;
                s_cl_wd <= std_logic_vector(vd_y(3));
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S3: region tests per candidate
    --------------------------------------------------------------------------
    p_test : process(clk)
        variable v_adx  : unsigned(4 downto 0);
        variable v_ady  : unsigned(4 downto 0);
        variable v_r    : unsigned(3 downto 0);
        variable v_head, v_trail, v_sheen, v_glint, v_stain : std_logic;
        variable v_f    : integer range 0 to 7;
        variable v_sw   : unsigned(3 downto 0);
        variable v_gl   : signed(5 downto 0);
        variable v_glw  : signed(5 downto 0);
        variable v_sqy  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s3_render <= s2_render;
            for c in 0 to 2 loop
                v_adx := resize(unsigned(abs(s2_dx(c))), 5);
                v_ady := resize(unsigned(abs(s2_dye(c))), 5);
                v_r   := s2_r(c);

                -- per-bead shape via pre-scaled vertical squares table
                case s2_shp(c) is
                    when "10"   => v_sqy := C_SQT(to_integer(v_ady(3 downto 0)));
                    when "11"   => v_sqy := C_SQW(to_integer(v_ady(3 downto 0)));
                    when others => v_sqy := C_SQ(to_integer(v_ady(3 downto 0)));
                end case;

                v_head := '0';
                if (s2_ph(c) = C_PH_BEAD or s2_ph(c) = C_PH_FALL)
                   and v_adx <= 15 and v_ady <= 15 then
                    -- 9-bit sum: two 8-bit squares can pass 255
                    if resize(C_SQ(to_integer(v_adx(3 downto 0))), 9)
                       + resize(v_sqy, 9)
                       <= resize(C_SQ(to_integer(v_r)), 9) then
                        v_head := '1';
                    end if;
                end if;

                -- small specular highlight, upper-left of centre
                v_glint := '0';
                v_gl  := s2_dye(c) + signed('0' & resize(shift_right(v_r, 1), 5));
                v_glw := signed('0' & resize(shift_right(v_r, 3), 5))
                         + to_signed(1, 6);
                if sw_gloss = '1'
                   and s2_dx(c) >= -v_glw and s2_dx(c) <= 0
                   and v_gl >= -1 and v_gl <= 0 then
                    v_glint := '1';
                end if;

                -- trail band behind (above) the head, dithered edge; only
                -- below the top-down fade edge
                v_trail := '0';
                if s2_ph(c) /= C_PH_IDLE and s2_tupok(c) = '1'
                   and s2_tup(c) <= c_tlen and s2_wetok(c) = '1' then
                    -- solid through the full bead radius, dither just outside
                    if v_adx <= resize(s2_tw(c), 5)
                       or (v_adx = resize(s2_tw(c), 5) + 1 and s2_dith = '1') then
                        v_trail := '1';
                    end if;
                end if;

                -- colour changes disabled: everything is bright fresh red
                v_f := 0;

                -- wet sheen stripe widens with the trail
                v_sheen := '0';
                if v_adx <= resize(shift_right(s2_tw(c), 2), 5) + 1
                   and sw_gloss = '1' then
                    v_sheen := '1';
                end if;

                -- painted record: every drip's full path stays as solid
                -- bright red at its bead's width (sts stores the width)
                v_stain := '0';
                v_sw := s2_sts(c)(3 downto 0);
                if v_sw > 11 then v_sw := to_unsigned(11, 4); end if;
                if s2_std(c) /= 0 and s2_sts(c) /= 0
                   and s2_y < shift_left(resize(s2_std(c), 12), 3) then
                    if v_adx <= resize(v_sw, 5)
                       or (v_adx = resize(v_sw, 5) + 1 and s2_dith = '1') then
                        v_stain := '1';
                    end if;
                end if;

                s3_head(c)  <= v_head;
                s3_glint(c) <= v_glint;
                s3_trail(c) <= v_trail;
                s3_sheen(c) <= v_sheen;
                s3_stain(c) <= v_stain;
                s3_vld(c)   <= s2_vld(c);
                s3_drf(c)   <= s2_drf(c);

                s3_fidx(c) <= to_unsigned(v_f, 2);
                s3_sts(c)  <= s2_sts(c);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S4: per-candidate priority encode, then combine (cur wins ties,
    -- then next, then prev)
    --------------------------------------------------------------------------
    p_prio : process(clk)
        variable v_hd, v_gl, v_tr, v_sh, v_st : std_logic_vector(0 to 2);
        variable v_glw, v_hdw, v_shw, v_trw, v_stw : std_logic;
    begin
        if rising_edge(clk) then
            -- validity-qualified flags per candidate (flat, no encoding)
            for c in 0 to 2 loop
                v_hd(c) := s3_head(c)  and s3_vld(c);
                v_gl(c) := s3_glint(c) and s3_head(c) and s3_vld(c);
                v_tr(c) := s3_trail(c) and s3_vld(c);
                v_sh(c) := s3_sheen(c) and s3_trail(c) and s3_vld(c);
                v_st(c) := s3_stain(c) and s3_vld(c);
            end loop;
            v_glw := v_gl(0) or v_gl(1) or v_gl(2);
            v_hdw := v_hd(0) or v_hd(1) or v_hd(2);
            v_shw := v_sh(0) or v_sh(1) or v_sh(2);
            v_trw := v_tr(0) or v_tr(1) or v_tr(2);
            v_stw := v_st(0) or v_st(1) or v_st(2);

            if    v_glw = '1' then s4_code <= C_RG_GLINT;
            elsif v_hdw = '1' then s4_code <= C_RG_HEAD;
            elsif v_shw = '1' then s4_code <= C_RG_SHEEN;
            elsif v_trw = '1' then s4_code <= C_RG_TRAIL;
            elsif v_stw = '1' then s4_code <= C_RG_STAIN;
            else                   s4_code <= C_RG_NONE;
            end if;

            -- params select independently of the region priority: fidx from
            -- the trail-owning candidate (cur first), sts from the
            -- stain-owning one, drift flag from the sheen-owning one.  Head
            -- and glint use fixed palettes so they never read these.
            if    v_tr(1) = '1' then s4_fidx <= s3_fidx(1);
            elsif v_tr(2) = '1' then s4_fidx <= s3_fidx(2);
            else                     s4_fidx <= s3_fidx(0);
            end if;
            if    v_st(1) = '1' then s4_sts <= s3_sts(1);
            elsif v_st(2) = '1' then s4_sts <= s3_sts(2);
            else                     s4_sts <= s3_sts(0);
            end if;
            if    v_sh(1) = '1' then s4_drf <= s3_drf(1);
            elsif v_sh(2) = '1' then s4_drf <= s3_drf(2);
            else                     s4_drf <= s3_drf(0);
            end if;
            s4_render <= s3_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S5: palette select (4-step fresh -> dried) + sheen luma
    --------------------------------------------------------------------------
    p_pal : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_ys : unsigned(10 downto 0);
        variable v_us, v_vs : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            case s4_code is
                when C_RG_GLINT =>
                    v_y := glint_y; v_u := glint_u; v_v := glint_v;
                when C_RG_HEAD =>
                    v_y := head_y; v_u := head_u; v_v := head_v;
                when others =>
                    -- one solid bright red everywhere (trail, sheen base,
                    -- painted record) - no darkening, no fades
                    v_y := pal_y0; v_u := pal_u0; v_v := pal_v0;
                    if s4_code = C_RG_SHEEN then
                        -- wet highlight: luma up (extra on diagonal jogs)
                        -- and chroma pulled toward white
                        v_ys := resize(v_y, 11) + resize(s_sheen_add, 11);
                        if s4_drf = '1' then
                            v_ys := v_ys + 96;
                        end if;
                        if v_ys > 1023 then v_y := (others => '1');
                        else               v_y := v_ys(9 downto 0);
                        end if;
                        v_us := signed(resize(v_u, 12));
                        v_us := v_us - shift_right(v_us - to_signed(512, 12), 2);
                        v_u  := unsigned(v_us(9 downto 0));
                        v_vs := signed(resize(v_v, 12));
                        v_vs := v_vs - shift_right(v_vs - to_signed(512, 12), 2);
                        v_v  := unsigned(v_vs(9 downto 0));
                    end if;
            end case;
            s5_py <= v_y; s5_pu <= v_u; s5_pv <= v_v;
            s5_code <= s4_code;
            s5_render <= s4_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S6: composite vs delayed video / black + blanking gate
    --------------------------------------------------------------------------
    p_comp : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_avid : std_logic;
    begin
        if rising_edge(clk) then
            v_avid := s_syncp(C_SYNCD)(2);

            -- blood is OPAQUE in both modes: the video stays untouched
            -- except where blood covers it
            if s5_code = C_RG_NONE then
                if sw_video = '1' then
                    v_y := vd_y(6); v_u := vd_u(6); v_v := vd_v(6);
                else
                    v_y := (others => '0'); v_u := C_MID; v_v := C_MID;
                end if;
            else
                v_y := s5_py; v_u := s5_pu; v_v := s5_pv;
            end if;

            -- blanking gate: neutral outside active video
            if v_avid = '0' or s5_render = '0' then
                v_y := (others => '0'); v_u := C_MID; v_v := C_MID;
            end if;

            s6_y <= v_y; s6_u <= v_u; s6_v <= v_v;
            s6_sync <= s_syncp(C_SYNCD);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output registers
    --------------------------------------------------------------------------
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io.y       <= std_logic_vector(s6_y);
            s_io.u       <= std_logic_vector(s6_u);
            s_io.v       <= std_logic_vector(s6_v);
            s_io.hsync_n <= s6_sync(0);
            s_io.vsync_n <= s6_sync(1);
            s_io.avid    <= s6_sync(2);
            s_io.field_n <= s6_sync(3);
        end if;
    end process;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture sanguine;
