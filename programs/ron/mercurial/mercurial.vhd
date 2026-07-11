-- mercurial.vhd
--
-- MERCURIAL -- luma as material.
--
-- The incoming picture's luma channel is treated as the height field of a
-- deformable metal surface, and the image is re-rendered as light striking
-- that surface. The original video is not displayed; it is inhabited.
--
-- v0.3 -- Chrome Engine + Strata Replacer + Rubber Engine:
--   Height     : FULL 10-BIT luma, horizontally [1 2 1]-blurred on the
--                line-buffer write path (8-bit height renders quantization
--                contours as dotted rings once normalized).
--   Normals    : 3x3 Sobel (2 chained 2048x10 line buffers), then
--                BLOCK-FLOAT NORMALIZED across two stages (S9 encode, S10
--                shift+clamp -- the single-stage version was the HD
--                critical path): one shared shift x1..x16 scales both
--                components, so direction is exact and magnitude is
--                log-compressed. Floor at mag 24 keeps noise flat.
--   Environment: CONTINUOUS Luster-steepened sky/ground ramp on the
--                vertical tilt + horizon glint band (gated to non-flat,
--                sideways-tilting surfaces) + slow-drifting sine sheen.
--   Specular   : alignment = clamp(K - |g - light|_L1), spec = align^2>>8;
--                Polish opens the window, top zone = hard-knee highlight.
--                Full-length light vector (matches normalized magnitudes).
--   Iridescence: (S8) chroma = the rotated, film-scaled normalized
--                gradient itself, gated by magnitude -- the UV plane is an
--                angle space, so hue = slope direction. No atan2, no LUT.
--   STRATA     : (S9) the luma range is cut into 4 bands by 3 slowly
--                drifting thresholds (Drift, K5); each band is REPLACED by
--                a different material, soft-keyed over 32 luma:
--                  shadows   -> interference fringes (tri-wave contours
--                               that follow the height field)
--                  low-mids  -> brushed metal (ridged value noise streaks,
--                               crawling slowly, anisotropic sheen)
--                  high-mids -> the chrome surface
--                  highlights-> dry video, passed through untouched
--                Off = chrome everywhere (v0.1 behavior).
--   Orbit      : (S11) the light circles the surface on its own.
--   Mercury    : (P12, headline) master morph dry video -> full material.
--
--   RUBBER     : (S10, K6 Elasticity, S7 stiffness invert) -- a coarse
--                grid of damped springs (one per 16x8..32x32 px cell)
--                chases the per-cell area-averaged luma at field rate;
--                the ringing RESIDUAL is bilinearly upsampled and added
--                to the height field ahead of the gradient window, so
--                motion sends a shudder through the chrome itself. See
--                the RUBBER ENGINE block comment for the full scheme.
--
-- Streaming, 1 px/clk, no clock division, C_LATENCY = 20. EBR: 2x line
-- buffer (2048x10 = 5 each) + dry ring (32x30 = 2) + spring state
-- (2048x20 = 10) + residual (2048x8 = 4) + column IIR (256x16 = 1)
-- -> 27 of 32.
--
-- Timing discipline (HD misses without all of these): BRAM reads land in
-- plain FFs before use; the qsin ROM mux output is registered before any
-- arithmetic; every multiply is alone in its stage with registered inputs;
-- the block-float shift select is REGISTERED a stage ahead of the barrel
-- shift; material lanes are fully assembled one stage before the A/B mux.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture mercurial of program_top is

    constant C_LATENCY : integer := 20;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_c255(v : unsigned) return unsigned is
    begin
        if v > 255 then
            return to_unsigned(255, 8);
        else
            return resize(v, 8);
        end if;
    end function;

    function f_absu(v : signed) return unsigned is
    begin
        if v < 0 then
            return unsigned(resize(-v, v'length));
        else
            return unsigned(v);
        end if;
    end function;

    function f_cs10(v : signed) return signed is
    begin
        if v > 511 then
            return to_signed(511, 10);
        elsif v < -511 then
            return to_signed(-511, 10);
        else
            return resize(v, 10);
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

    -- quarter-wave folded sine, 64-entry logic ROM (fireworks): the SDK's
    -- 1024-entry sin_cos_full_lut_10x10 infers as 6 EBRs here; this is 0.
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

    -- value-noise hash, split across two pipeline stages (inferno: 1-add
    -- hashes show diagonal weave)
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

    -- triangle fold of a 10-bit phase: distance from 512, clamped to u9
    function f_tri(v : unsigned(9 downto 0)) return unsigned is
        variable v_d : unsigned(9 downto 0);
    begin
        if v(9) = '1' then
            v_d := v - 512;
        else
            v_d := 512 - resize(v, 10);
        end if;
        if v_d > 511 then
            return to_unsigned(511, 9);
        else
            return resize(v_d, 9);
        end if;
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls (all registered at the vsync edge)
    ----------------------------------------------------------------------
    signal s_p12       : unsigned(9 downto 0) := (others => '0');  -- Mercury
    signal s_kwin      : unsigned(9 downto 0) := to_unsigned(192, 10);
    signal s_rot3      : unsigned(2 downto 0) := (others => '0');
    signal s_fsh       : natural range 2 to 5 := 4;
    signal s_irid_on   : std_logic := '0';
    signal s_orbit_on  : std_logic := '0';
    signal s_strata_on : std_logic := '0';
    signal s_lus       : natural range 0 to 3 := 1;
    signal s_az_eff    : unsigned(9 downto 0) := (others => '0');
    signal s_orbit_acc : unsigned(9 downto 0) := (others => '0');
    signal s_phase     : unsigned(9 downto 0) := (others => '0');

    -- strata thresholds: 3 drifting cut points, pre-registered as +/-16
    -- soft-key windows (compares must see registered bounds)
    signal ph1, ph2, ph3 : unsigned(9 downto 0) := (others => '0');
    signal s_amp : natural range 4 to 7 := 7;
    signal t1m, t1p : unsigned(7 downto 0) := to_unsigned( 40, 8);
    signal t2m, t2p : unsigned(7 downto 0) := to_unsigned(112, 8);
    signal t3m, t3p : unsigned(7 downto 0) := to_unsigned(184, 8);

    -- light vector, captured from the shared f_qsin during vblank
    signal s_lx, s_ly  : signed(9 downto 0) := (others => '0');
    signal s_seq       : unsigned(3 downto 0) := (others => '0');

    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');  -- registered angle
    signal s_qs_r  : signed(9 downto 0) := (others => '0');   -- registered ROM out

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';

    ----------------------------------------------------------------------
    -- valid chain + guards + coordinates
    ----------------------------------------------------------------------
    signal av : std_logic_vector(1 to 19) := (others => '0');

    signal cbx     : unsigned(10 downto 0) := (others => '0');
    signal s_aline : unsigned(9 downto 0) := (others => '0');
    signal s_seen  : std_logic := '0';

    ----------------------------------------------------------------------
    -- RUBBER ENGINE (v0.3)
    --
    -- A coarse grid of damped springs chases the per-cell area-averaged
    -- luma at field rate; the RESIDUAL (pos - target) is bilinearly
    -- upsampled and added to the full-res luma BEFORE the gradient window,
    -- so motion sends a low-frequency shudder through the chrome itself.
    --
    -- Geometry (from the MEASURED line width, spiroscope pattern -- also
    -- keeps decimated GHDL sims correct): cells 2^xsh px x 2^ysh field
    -- lines; both interlaced fields drive the same cells (field-agnostic).
    --   width >= 1600 (1080i): xsh 5, ysh 4, cols 60, stride 60
    --   width >= 1000 (720p) : xsh 5, ysh 5, cols 40, stride 60
    --   else (SD)            : xsh 4, ysh 3, cols 45, stride 48
    -- Max address 33*60+59 = 2039 < 2048; row_base is maintained
    -- incrementally (+stride at cell-row end) -- zero address multiplies.
    --
    -- Memories: state_ram 2048x20 {pos u10 8.2, vel s10} (10 EBR, R/W by
    -- the updater only); res_ram 2048x8 residual s8 (4 EBR, W updater /
    -- R display prefetch -- port-exclusive by construction, no arbiter);
    -- vacc 256x16 per-column target IIR 8.4 (1 EBR).
    --
    -- Updater: ONE CELL ROW PER HBLANK (armed at the hsync closing a cell
    -- row's last active line; the final partial row fires at vblank
    -- start). 7-deep pipeline, 1 cell/clk: worst 60+7 clocks vs 138-clock
    -- minimum hblank. All spring math is shifts:
    --   vel' = clamp(vel + (T-pos)>>>ksh - vel>>>dsh +/- 1 LFSR dither)
    --   pos' = clamp(pos + vel'>>>1),  res = clamp_s8((pos'-T)>>2)
    -- ksh = 6 - T(9:8) bucket (bright cells stiff/fast, dark heavy/slow;
    -- S7 inverts the bucket), dsh = 2 + Elasticity zone (high = ringy).
    ----------------------------------------------------------------------
    signal s_x_count    : unsigned(11 downto 0) := (others => '0');
    signal s_xsh        : natural range 4 to 5 := 5;
    signal s_ysh        : natural range 3 to 5 := 4;
    signal s_cols       : unsigned(5 downto 0) := to_unsigned(60, 6);
    signal s_stride     : unsigned(5 downto 0) := to_unsigned(60, 6);
    signal s_wsh        : natural range 0 to 3 := 2;   -- wobble depth (P6)
    signal s_p6z        : unsigned(1 downto 0) := "01";
    signal s_inv_stiff  : std_logic := '0';            -- switch 7
    signal s_rub_on     : std_logic := '0';             -- switch 10
    signal s_lwidth     : unsigned(11 downto 0) := (others => '0');

    -- display-side cell tracking
    signal fx5, fy5     : unsigned(4 downto 0) := (others => '0');
    signal dsp_rb       : unsigned(10 downto 0) := (others => '0');  -- row base
    signal dsp_rb1      : unsigned(10 downto 0) := (others => '0');  -- next row

    -- residual corner registers + prefetch
    signal c00, c01, c10, c11 : signed(7 downto 0) := (others => '0');
    signal d0, d1       : signed(8 downto 0) := (others => '0');
    signal p_top, p_bot : signed(7 downto 0) := (others => '0');
    signal pf_seq       : unsigned(2 downto 0) := (others => '0');   -- hblank prefetch
    signal res_raddr    : unsigned(10 downto 0) := (others => '0');

    -- residual lerp pipeline (S1..S3) + wobble. The base corners are
    -- PIPELINED alongside the products: the corner registers shift at
    -- span boundaries while S2 still holds the previous span's product,
    -- so consuming c00/c01 directly at S2 glitches every boundary.
    signal prt, prb     : signed(14 downto 0) := (others => '0');
    signal c00_1, c01_1 : signed(7 downto 0) := (others => '0');
    signal rt2          : signed(8 downto 0) := (others => '0');
    signal dv2          : signed(9 downto 0) := (others => '0');
    signal rt3          : signed(8 downto 0) := (others => '0');
    signal pv3          : signed(15 downto 0) := (others => '0');

    -- per-column target accumulator
    signal acc_y        : unsigned(12 downto 0) := (others => '0');
    signal acc_pre_col  : unsigned(7 downto 0) := (others => '0');
    signal vq_disp      : unsigned(11 downto 0) := (others => '0');
    signal vacc_we      : std_logic := '0';
    signal vacc_waddr   : unsigned(7 downto 0) := (others => '0');
    signal vacc_wdata   : unsigned(15 downto 0) := (others => '0');

    -- updater
    signal u_go         : std_logic := '0';
    signal u_rb         : unsigned(10 downto 0) := (others => '0');
    signal u_active     : std_logic := '0';
    signal u_cx         : unsigned(5 downto 0) := (others => '0');
    signal u_vca        : unsigned(7 downto 0) := (others => '0');
    signal u_v          : std_logic_vector(1 to 7) := (others => '0');
    signal u_a0, u_a0d, u_a1, u_a2, u_a3, u_a4, u_a5
        : unsigned(10 downto 0) := (others => '0');
    signal u_pos1, u_t1 : unsigned(9 downto 0) := (others => '0');
    signal u_vel1       : signed(9 downto 0) := (others => '0');
    signal u_ek2        : signed(10 downto 0) := (others => '0');
    signal u_vd2        : signed(9 downto 0) := (others => '0');
    signal u_pos2, u_t2 : unsigned(9 downto 0) := (others => '0');
    signal u_vel3       : signed(9 downto 0) := (others => '0');
    signal u_pos3, u_t3 : unsigned(9 downto 0) := (others => '0');
    signal u_pos4       : unsigned(9 downto 0) := (others => '0');
    signal u_vel4       : signed(9 downto 0) := (others => '0');
    signal u_t4         : unsigned(9 downto 0) := (others => '0');
    signal u_res5       : signed(7 downto 0) := (others => '0');
    signal s_pend_row   : std_logic := '0';
    signal s_lfsr       : unsigned(15 downto 0) := x"ACE1";

    -- memories
    type t_state is array (0 to 2047) of std_logic_vector(19 downto 0);
    type t_res   is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_vacc  is array (0 to 255)  of std_logic_vector(15 downto 0);
    signal state_ram : t_state := (others => (others => '0'));
    signal res_ram   : t_res   := (others => (others => '0'));
    signal vacc      : t_vacc  := (others => (others => '0'));
    signal state_q   : std_logic_vector(19 downto 0) := (others => '0');
    signal res_q     : std_logic_vector(7 downto 0) := (others => '0');
    signal vacc_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal vacc_raddr : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S0..S4: input register, rubber-residual stubs, height + LBs
    ----------------------------------------------------------------------
    signal r0_y  : unsigned(9 downto 0) := (others => '0');
    signal yd_1, yd_2, yd_3 : unsigned(9 downto 0) := (others => '0');
    signal yh_d1, yh_d2     : unsigned(9 downto 0) := (others => '0');
    signal h4    : unsigned(9 downto 0) := (others => '0');

    type t_lb10 is array (0 to 2047) of std_logic_vector(9 downto 0);
    signal lb1, lb2 : t_lb10 := (others => (others => '0'));
    signal q1, q2   : std_logic_vector(9 downto 0) := (others => '0');
    signal lrx, lwx : unsigned(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S5..S8: window rows -> Sobel gradients + magnitude; center tap
    ----------------------------------------------------------------------
    signal qr1, qr2 : unsigned(9 downto 0) := (others => '0');
    signal hd5      : unsigned(9 downto 0) := (others => '0');

    signal ss, ss1, ss2  : unsigned(11 downto 0) := (others => '0');
    signal dyv, dy1, dy2 : signed(10 downto 0) := (others => '0');
    signal hc6, hc7, hc8, hc9, hc10, hc11, hc12, hc13
        : unsigned(7 downto 0) := (others => '0');

    signal gx, gy   : signed(13 downto 0) := (others => '0');
    signal gx8, gy8 : signed(13 downto 0) := (others => '0');
    signal mag14    : unsigned(13 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S9..S10: block-float normalization, split (S9 = registered shift
    -- select + flat flag, S10 = barrel shift + clamp) -- the fused version
    -- was the HD critical path
    ----------------------------------------------------------------------
    signal gx9, gy9 : signed(13 downto 0) := (others => '0');
    signal n9       : natural range 0 to 4 := 0;
    signal flat9    : std_logic := '1';
    signal gxs, gys : signed(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S11: light alignment, gate, rotation; brushed hash A
    ----------------------------------------------------------------------
    signal ax, ay       : unsigned(9 downto 0) := (others => '0');
    signal g_gate       : unsigned(7 downto 0) := (others => '0');
    signal gxr11, gyr11 : signed(10 downto 0) := (others => '0');
    signal gys11        : signed(9 downto 0) := (others => '0');
    signal bh0          : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S12: specular window, film chroma; brushed hash B
    ----------------------------------------------------------------------
    signal align8     : unsigned(7 downto 0) := (others => '0');
    signal cu12, cv12 : signed(7 downto 0) := (others => '0');
    signal g12        : unsigned(7 downto 0) := (others => '0');
    signal gys12      : signed(9 downto 0) := (others => '0');
    signal bn0        : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S13: the multiplies (spec square, chroma gates, brushed lerp) + env
    -- ramp; interference phase; strata band select
    ----------------------------------------------------------------------
    signal spec_p   : unsigned(15 downto 0) := (others => '0');
    signal cu2, cv2 : signed(16 downto 0) := (others => '0');
    signal env_y8   : unsigned(7 downto 0) := (others => '0');
    signal bl13     : unsigned(7 downto 0) := (others => '0');
    signal iph13    : unsigned(9 downto 0) := (others => '0');
    signal al13     : unsigned(7 downto 0) := (others => '0');   -- align8 delayed
    signal bidx13   : unsigned(1 downto 0) := (others => '0');
    signal f8_13    : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S14: material lanes fully assembled (u10 Y/U/V each)
    ----------------------------------------------------------------------
    signal ch_y, ch_u, ch_v : unsigned(9 downto 0) := (others => '0');
    signal br_y             : unsigned(9 downto 0) := (others => '0');
    signal if_y, if_u, if_v : unsigned(9 downto 0) := (others => '0');
    signal bidx14           : unsigned(1 downto 0) := (others => '0');
    signal f8_14            : unsigned(7 downto 0) := (others => '0');

    constant C_BR_U : unsigned(9 downto 0) := to_unsigned(472, 10); -- warm steel
    constant C_BR_V : unsigned(9 downto 0) := to_unsigned(544, 10);

    ----------------------------------------------------------------------
    -- S15..S19: A/B lane mux, blend multiplies, strata sum + Mercury diff,
    -- Mercury multiplies, recombine/out
    ----------------------------------------------------------------------
    signal a_y, a_u, a_v : unsigned(9 downto 0) := (others => '0');
    signal b_y, b_u, b_v : unsigned(9 downto 0) := (others => '0');
    signal f8_15         : unsigned(7 downto 0) := (others => '0');

    signal bdy, bdu, bdv    : signed(18 downto 0) := (others => '0');
    signal admy, admu, admv : signed(10 downto 0) := (others => '0');

    signal dmy, dmu, dmv : signed(10 downto 0) := (others => '0');
    signal pmy, pmu, pmv : signed(19 downto 0) := (others => '0');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- dry video delay rings (edgelab): vr_q = data_in delayed 15 (S15 dry
    -- lane), +1 reg = 16 (S16 pre-diff); a SECOND ring copy provides the
    -- delay-19 tap for S19 (2 EBR bought back ~90 FFs in the LC squeeze)
    ----------------------------------------------------------------------
    type t_ring is array (0 to 31) of std_logic_vector(29 downto 0);
    signal vring   : t_ring := (others => (others => '0'));
    signal vring_b : t_ring := (others => (others => '0'));
    signal vr_wp   : unsigned(4 downto 0) := (others => '0');
    signal vr_q    : std_logic_vector(29 downto 0) := (others => '0');
    signal vr_qd   : std_logic_vector(29 downto 0) := (others => '0');
    signal vr_q19  : std_logic_vector(29 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared quarter-wave sine: the vblank sequencer (light vector + 3
    -- strata thresholds) borrows the per-pixel sheen tap's angle port.
    -- BOTH the angle (s_qs_ar) and the ROM output (s_qs_r) are registered:
    -- select-mux -> ROM -> capture in one cycle was the routed critical
    -- path twice over (those paths are STA-timed at full clock even though
    -- the captures only fire in vblank). Consumers see qsin two clocks
    -- late; the sequencer captures two-behind and the sheen picks up a
    -- 2-pixel offset (invisible in a +/-16 wash).
    ------------------------------------------------------------------------
    s_qs_a <= s_az_eff                        when s_seq = 8 else
              s_az_eff + to_unsigned(256, 10) when s_seq = 7 else
              ph1                             when s_seq = 6 else
              ph2                             when s_seq = 5 else
              ph3                             when s_seq = 4 else
              (others => '0');

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_r  : unsigned(9 downto 0);
        variable v_t  : signed(11 downto 0);
        variable v_c  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_p12      <= unsigned(registers_in(7));
                s_kwin     <= to_unsigned(128, 10)
                              + resize(unsigned(registers_in(1)(9 downto 2)), 10);
                s_rot3     <= unsigned(registers_in(3)(9 downto 7));
                s_fsh      <= 5 - to_integer(unsigned(registers_in(3)(6 downto 5)));
                s_irid_on  <= registers_in(6)(1);   -- switch 8
                s_strata_on <= registers_in(6)(2);  -- switch 9
                s_orbit_on <= registers_in(6)(4);   -- switch 11
                s_lus <= 3 - to_integer(unsigned(registers_in(2)(9 downto 8)));
                s_amp <= 7 - to_integer(unsigned(registers_in(4)(9 downto 8)));

                -- rubber controls + measured grid geometry
                s_inv_stiff <= registers_in(6)(0);  -- switch 7
                s_rub_on    <= registers_in(6)(3);  -- switch 10
                s_p6z <= unsigned(registers_in(5)(9 downto 8));
                -- wobble depth floor of >>1: full-depth (<<2 of a railed
                -- +/-127 residual) slams the whole field to black/white
                if registers_in(5)(9 downto 8) = "11" then
                    s_wsh <= 1;
                else
                    s_wsh <= 3 - to_integer(unsigned(registers_in(5)(9 downto 8)));
                end if;
                if s_lwidth >= 1600 then
                    s_xsh <= 5; s_ysh <= 4;
                    s_cols <= to_unsigned(60, 6); s_stride <= to_unsigned(60, 6);
                elsif s_lwidth >= 1000 then
                    s_xsh <= 5; s_ysh <= 5;
                    s_cols <= to_unsigned(40, 6); s_stride <= to_unsigned(60, 6);
                else
                    s_xsh <= 4; s_ysh <= 3;
                    s_cols <= to_unsigned(45, 6); s_stride <= to_unsigned(48, 6);
                end if;

                if s_orbit_on = '1' then
                    s_orbit_acc <= s_orbit_acc
                                   + resize(unsigned(registers_in(0)(9 downto 5)), 10)
                                   + 1;
                    s_az_eff <= s_orbit_acc;
                else
                    s_az_eff <= unsigned(registers_in(0));
                end if;

                -- strata drift: 3 phase accumulators at 1 : 1.5 : 1.25
                -- rate ratios, Drift knob sets rate (0 = frozen) and depth
                v_r := shift_right(resize(unsigned(registers_in(4)(9 downto 6)), 10), 1);
                ph1 <= ph1 + v_r;
                ph2 <= ph2 + v_r + shift_right(v_r, 1);
                ph3 <= ph3 + v_r + shift_right(v_r, 2);

                s_phase <= s_phase + 3;
                s_seq   <= "1000";        -- 8-step capture chain
            elsif s_seq /= 0 and data_in.avid = '0' then
                -- capture TWO-BEHIND the angle mux: s_qs_r holds qsin of
                -- the angle presented two steps earlier
                if s_seq = 6 then
                    s_ly <= s_qs_r;       -- sin(az)
                elsif s_seq = 5 then
                    s_lx <= s_qs_r;       -- cos(az)
                elsif s_seq <= 4 and s_seq >= 2 then
                    -- drifting threshold: center + qsin >>> amp, then the
                    -- +/-16 soft-key bounds, all registered here
                    if s_seq = 4 then
                        v_c := to_signed(56, 12);
                    elsif s_seq = 3 then
                        v_c := to_signed(128, 12);
                    else
                        v_c := to_signed(200, 12);
                    end if;
                    v_t := v_c + resize(shift_right(s_qs_r, s_amp), 12);
                    -- clamp center into 24..231 so the +/-16 window stays
                    -- inside u8
                    if v_t < 24 then
                        v_t := to_signed(24, 12);
                    elsif v_t > 231 then
                        v_t := to_signed(231, 12);
                    end if;
                    if s_seq = 4 then
                        t1m <= resize(unsigned(v_t - 16), 8);
                        t1p <= resize(unsigned(v_t + 16), 8);
                    elsif s_seq = 3 then
                        t2m <= resize(unsigned(v_t - 16), 8);
                        t2p <= resize(unsigned(v_t + 16), 8);
                    else
                        t3m <= resize(unsigned(v_t - 16), 8);
                        t3p <= resize(unsigned(v_t + 16), 8);
                    end if;
                end if;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- main pixel pipeline
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n    : natural range 0 to 4;
        variable v_es   : signed(11 downto 0);
        variable v_al   : signed(11 downto 0);
        variable v_gxr  : signed(10 downto 0);
        variable v_gyr  : signed(10 downto 0);
        variable v_sp8  : unsigned(7 downto 0);
        variable v_cy   : unsigned(8 downto 0);
        variable v_c3   : signed(7 downto 0);
        variable v_ri   : unsigned(7 downto 0);
        variable v_r9   : unsigned(8 downto 0);
        variable v_tr   : unsigned(8 downto 0);
        variable v_bi   : unsigned(1 downto 0);
        variable v_pos  : unsigned(4 downto 0);
        variable v_end  : boolean;
        variable v_col  : unsigned(7 downto 0);
        variable v_sum  : unsigned(12 downto 0);
        variable v_avg  : unsigned(7 downto 0);
        variable v_rt   : signed(8 downto 0);
        variable v_rb   : signed(8 downto 0);
        variable v_rv   : signed(9 downto 0);
        variable v_wob  : signed(11 downto 0);
        variable v_rowend : boolean;
        variable v_nal  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            av(1) <= data_in.avid;
            for i in 2 to 19 loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            -- S0: input register + rubber-grid display tracking
            if data_in.avid = '1' then
                r0_y <= unsigned(data_in.y);
                s_x_count <= s_x_count + 1;

                -- span position within the current cell (variable cell
                -- width folded into one 5-bit position)
                if s_xsh = 4 then
                    v_pos := '0' & s_x_count(3 downto 0);
                else
                    v_pos := s_x_count(4 downto 0);
                end if;
                v_end := (s_xsh = 5 and v_pos = 31)
                         or (s_xsh = 4 and v_pos = 15);
                v_col := resize(shift_right(s_x_count, s_xsh), 8);

                -- residual lerp fraction for THIS pixel
                if s_xsh = 4 then
                    fx5 <= s_x_count(3 downto 0) & '0';
                else
                    fx5 <= s_x_count(4 downto 0);
                end if;

                -- per-column target accumulator (c64 pattern) + IIR
                if v_pos = 0 then
                    v_sum := resize(r0_y(9 downto 2), 13);
                else
                    v_sum := acc_y + resize(r0_y(9 downto 2), 13);
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

                -- rolling residual corner prefetch: fetch column col+2's
                -- two rows mid-span, shift corners at span start -- but
                -- NOT at column 0, whose corners the hblank prefetch just
                -- loaded (shifting there cascades stale data line-wide)
                if v_pos = 0 and v_col /= 0 then
                    c00 <= c10;  c01 <= c11;
                    c10 <= p_top;  c11 <= p_bot;
                    d0  <= resize(p_top, 9) - resize(c10, 9);
                    d1  <= resize(p_bot, 9) - resize(c11, 9);
                elsif v_pos = 4 then
                    if v_col + 2 >= resize(s_cols, 8) then
                        res_raddr <= resize(dsp_rb, 11)
                                     + resize(s_cols - 1, 11);
                    else
                        res_raddr <= resize(dsp_rb, 11) + resize(v_col, 11) + 2;
                    end if;
                elsif v_pos = 6 then
                    p_top <= signed(res_q);
                elsif v_pos = 8 then
                    if v_col + 2 >= resize(s_cols, 8) then
                        res_raddr <= resize(dsp_rb1, 11)
                                     + resize(s_cols - 1, 11);
                    else
                        res_raddr <= resize(dsp_rb1, 11) + resize(v_col, 11) + 2;
                    end if;
                elsif v_pos = 10 then
                    p_bot <= signed(res_q);
                end if;
            end if;

            -- hblank prefetch of the coming line's first two columns
            if pf_seq /= 0 then
                case pf_seq is
                    when "001" => res_raddr <= resize(dsp_rb, 11);
                    when "010" => res_raddr <= resize(dsp_rb1, 11);
                    when "011" => c00 <= signed(res_q);
                                  res_raddr <= resize(dsp_rb, 11) + 1;
                    when "100" => c01 <= signed(res_q);
                                  res_raddr <= resize(dsp_rb1, 11) + 1;
                    when "101" => c10 <= signed(res_q);
                    when "110" => c11 <= signed(res_q);
                    when others => d0 <= resize(c10, 9) - resize(c00, 9);
                                   d1 <= resize(c11, 9) - resize(c01, 9);
                end case;
                if pf_seq = "111" then
                    pf_seq <= (others => '0');
                else
                    pf_seq <= pf_seq + 1;
                end if;
            end if;

            -- S1: residual horizontal-lerp multiplies (alone in stage;
            -- d0/d1 are stable across the span)
            if av(1) = '1' then
                yd_1  <= r0_y;
                prt   <= d0 * signed('0' & fx5);
                prb   <= d1 * signed('0' & fx5);
                c00_1 <= c00;
                c01_1 <= c01;
            end if;

            -- S2: horizontal lerp sums + vertical diff
            if av(2) = '1' then
                yd_2 <= yd_1;
                v_rt := resize(c00_1, 9) + resize(shift_right(prt, 5), 9);
                v_rb := resize(c01_1, 9) + resize(shift_right(prb, 5), 9);
                rt2  <= v_rt;
                dv2  <= resize(v_rb, 10) - resize(v_rt, 10);
            end if;

            -- S3: vertical-lerp multiply (alone in stage)
            if av(3) = '1' then
                yd_3 <= yd_2;
                pv3  <= dv2 * signed('0' & fy5);
                rt3  <= rt2;
            end if;

            -- S4: height field = horizontally [1 2 1]-blurred 10-bit luma
            -- + the bilinear spring wobble (Elasticity depth = shift);
            -- line-buffer reads presented this cycle
            if av(4) = '1' then
                v_rv := resize(rt3, 10) + resize(shift_right(pv3, 5), 10);
                if s_rub_on = '1' then
                    v_wob := shift_left(resize(shift_right(v_rv, s_wsh), 12), 2);
                else
                    v_wob := (others => '0');
                end if;
                h4 <= f_cu10(signed(resize(shift_right(resize(yd_3, 12)
                             + shift_left(resize(yh_d1, 12), 1)
                             + resize(yh_d2, 12), 2), 12)) + v_wob);
                yh_d1 <= yd_3;
                yh_d2 <= yh_d1;
                lrx   <= lrx + 1;
            end if;

            -- S5: land the BRAM reads in plain FFs
            if av(5) = '1' then
                qr1 <= unsigned(q1);
                qr2 <= unsigned(q2);
                hd5 <= h4;
                lwx <= lwx + 1;
            end if;

            -- S6: Sobel column sums + histories; center tap (mid row)
            if av(6) = '1' then
                ss  <= resize(qr2, 12) + shift_left(resize(qr1, 12), 1)
                       + resize(hd5, 12);
                dyv <= signed(resize(qr2, 11)) - signed(resize(hd5, 11));
                ss1 <= ss;   ss2 <= ss1;
                dy1 <= dyv;  dy2 <= dy1;
                hc6 <= qr1(9 downto 2);
            end if;

            -- S7: gradients (gy > 0 = surface brighter above)
            if av(7) = '1' then
                gx  <= signed(resize(ss, 14)) - signed(resize(ss2, 14));
                gy  <= resize(dyv, 14) + shift_left(resize(dy1, 14), 1)
                       + resize(dy2, 14);
                hc7 <= hc6;
                cbx <= cbx + 1;
            end if;

            -- S8: register raw gradients + magnitude; warm-up guards
            if av(8) = '1' then
                if s_aline < 4 or cbx < 8 then
                    gx8   <= (others => '0');
                    gy8   <= (others => '0');
                    mag14 <= (others => '0');
                else
                    gx8   <= gx;
                    gy8   <= gy;
                    mag14 <= resize(f_absu(gx), 14) + resize(f_absu(gy), 14);
                end if;
                hc8 <= hc7;
            end if;

            -- S9: block-float shift select, REGISTERED (feeding the barrel
            -- shift directly from the priority encode was the HD critical
            -- path); floor at mag 24
            if av(9) = '1' then
                if mag14(13 downto 9) /= "00000" then
                    v_n := 0;
                elsif mag14(8) = '1' then
                    v_n := 1;
                elsif mag14(7) = '1' then
                    v_n := 2;
                elsif mag14(6) = '1' then
                    v_n := 3;
                else
                    v_n := 4;
                end if;
                n9 <= v_n;
                if mag14 < 24 then
                    flat9 <= '1';
                else
                    flat9 <= '0';
                end if;
                gx9 <= gx8;
                gy9 <= gy8;
                hc9 <= hc8;
            end if;

            -- S10: barrel shift + clamp (direction exact, magnitude
            -- log-compressed)
            if av(10) = '1' then
                if flat9 = '1' then
                    gxs <= (others => '0');
                    gys <= (others => '0');
                else
                    gxs <= f_cs10(shift_left(resize(gx9, 19), n9));
                    gys <= f_cs10(shift_left(resize(gy9, 19), n9));
                end if;
                hc10 <= hc9;
            end if;

            -- S11: light alignment distances, edge gate, iridescent
            -- rotate, sheen angle; brushed-metal hash stage A (16px x 2ln
            -- cells, slow crawl via the phase accumulator)
            if av(11) = '1' then
                ax <= resize(f_absu(resize(gxs, 11) - resize(s_lx, 11)), 10);
                ay <= resize(f_absu(resize(gys, 11) - resize(s_ly, 11)), 10);
                g_gate <= f_c255(shift_right(resize(f_absu(gxs), 11)
                                             + resize(f_absu(gys), 11), 1));

                case s_rot3(2 downto 1) is
                    when "00"   => v_gxr := resize(gxs, 11);
                                   v_gyr := resize(gys, 11);
                    when "01"   => v_gxr := -resize(gys, 11);
                                   v_gyr := resize(gxs, 11);
                    when "10"   => v_gxr := -resize(gxs, 11);
                                   v_gyr := -resize(gys, 11);
                    when others => v_gxr := resize(gys, 11);
                                   v_gyr := -resize(gxs, 11);
                end case;
                -- (the 45-degree half-rotation and the sheen were cut in
                -- the v0.3 LC squeeze: quadrant hue rotation only)
                gxr11 <= v_gxr;
                gyr11 <= v_gyr;

                gys11 <= gys;
                hc11  <= hc10;

                -- 32px x 1-line cells: hairline horizontal streaks,
                -- unfiltered (the x-lerp was cut in the LC squeeze;
                -- segmented streaks still read as brushed)
                bh0 <= f_hash_a(resize(shift_right(cbx, 5), 8)
                                + resize(shift_right(s_phase, 3), 8),
                                resize(s_aline, 8), x"517C");
            end if;

            -- S12: specular alignment window; REGISTERED sheen; film
            -- shift + clamp; brushed hash stage B
            if av(12) = '1' then
                v_al := signed(resize(s_kwin, 12)) - signed(resize(ax, 12))
                        - signed(resize(ay, 12));
                if v_al < 0 then
                    align8 <= (others => '0');
                elsif v_al > 255 then
                    align8 <= (others => '1');
                else
                    align8 <= resize(unsigned(v_al), 8);
                end if;

                if s_irid_on = '1' then
                    cu12 <= f_cs8(shift_right(gxr11, s_fsh));
                    cv12 <= f_cs8(shift_right(gyr11, s_fsh));
                else
                    cu12 <= to_signed(8, 8);    -- cool-steel tint
                    cv12 <= to_signed(-4, 8);
                end if;
                g12   <= g_gate;
                gys12 <= gys11;
                hc12  <= hc11;

                bn0 <= f_hash_b(bh0);
            end if;

            -- S13: the multiplies (spec square, chroma gates, brushed
            -- x-lerp) with registered inputs; continuous environment ramp
            -- + gated horizon glint + sheen; interference phase; strata
            -- band select on the height center tap
            if av(13) = '1' then
                spec_p <= align8 * align8;
                cu2    <= cu12 * signed('0' & g12);
                cv2    <= cv12 * signed('0' & g12);

                v_es := to_signed(128, 12)
                        - resize(shift_right(gys12, s_lus), 12);
                if g12 /= 0 and f_absu(gys12) < 64 then
                    v_es := v_es + signed(resize(shift_left(
                                to_unsigned(64, 8) - resize(f_absu(gys12), 8),
                                1), 12));
                end if;
                if v_es < 0 then
                    env_y8 <= (others => '0');
                elsif v_es > 255 then
                    env_y8 <= (others => '1');
                else
                    env_y8 <= resize(unsigned(v_es), 8);
                end if;

                bl13 <= bn0;

                iph13 <= resize(shift_left(resize(cbx, 13), 3), 10)
                         + resize(shift_left(resize(s_aline, 14), 4), 10)
                         + shift_left(resize(hc12, 10), 3)
                         + shift_left(s_phase, 1);
                al13 <= align8;
                hc13 <= hc12;

                -- strata: band index + soft fraction (bounds pre-registered)
                if s_strata_on = '0' then
                    bidx13 <= "10";           -- chrome everywhere
                    f8_13  <= (others => '0');
                elsif hc12 < t1m then
                    bidx13 <= "00"; f8_13 <= (others => '0');
                elsif hc12 < t1p then
                    bidx13 <= "00";
                    f8_13  <= resize(shift_left(resize(hc12 - t1m, 8), 3), 8);
                elsif hc12 < t2m then
                    bidx13 <= "01"; f8_13 <= (others => '0');
                elsif hc12 < t2p then
                    bidx13 <= "01";
                    f8_13  <= resize(shift_left(resize(hc12 - t2m, 8), 3), 8);
                elsif hc12 < t3m then
                    bidx13 <= "10"; f8_13 <= (others => '0');
                elsif hc12 < t3p then
                    bidx13 <= "10";
                    f8_13  <= resize(shift_left(resize(hc12 - t3m, 8), 3), 8);
                else
                    bidx13 <= "11"; f8_13 <= (others => '0');
                end if;
            end if;

            -- S14: assemble ALL material lanes as u10 Y/U/V (one stage
            -- ahead of the A/B mux so the mux stage stays shallow)
            if av(14) = '1' then
                -- chrome: specular + metal base env*3/4 + height/4 (the
                -- sharp knee and highlight desat were cut in the LC squeeze)
                v_sp8 := spec_p(15 downto 8);
                v_cy := resize((env_y8 - shift_right(env_y8, 2))
                               + shift_right(hc13, 2), 9)
                        + resize(v_sp8, 9);
                if v_cy > 255 then
                    ch_y <= to_unsigned(1023, 10);
                else
                    ch_y <= v_cy(7 downto 0) & "00";
                end if;
                v_c3 := resize(cu2(15 downto 8), 8);
                ch_u <= resize(unsigned(to_signed(512, 11)
                               + shift_left(resize(v_c3, 11), 2)), 10);
                v_c3 := resize(cv2(15 downto 8), 8);
                ch_v <= resize(unsigned(to_signed(512, 11)
                               + shift_left(resize(v_c3, 11), 2)), 10);

                -- brushed metal: ridged fold of the value noise
                -- + anisotropic sheen from the (linear) light alignment
                if bl13 >= 128 then
                    v_r9 := shift_left(resize(bl13 - 128, 9), 1);
                else
                    v_r9 := shift_left(resize(128 - bl13, 9), 1);
                end if;
                if v_r9 > 255 then
                    v_r9 := to_unsigned(255, 9);
                end if;
                v_ri := to_unsigned(255, 8) - resize(v_r9, 8);  -- bright ridges
                br_y <= (resize(to_unsigned(64, 8)
                                + shift_right(v_ri, 1)
                                + resize(shift_right(al13, 2), 8), 8)) & "00";

                -- interference: tri-wave fringes following the height
                v_tr := f_tri(iph13);
                if_y <= (to_unsigned(80, 8)
                         + resize(shift_right(to_unsigned(511, 9) - v_tr, 2), 8))
                        & "00";
                v_tr := f_tri(iph13 + to_unsigned(341, 10));
                if_u <= to_unsigned(448, 10) + resize(shift_right(v_tr, 2), 10);
                v_tr := f_tri(iph13 + to_unsigned(682, 10));
                if_v <= to_unsigned(448, 10) + resize(shift_right(v_tr, 2), 10);

                bidx14 <= bidx13;
                f8_14  <= f8_13;
            end if;

            -- S15: A/B lane mux (pure muxes; lanes were assembled at S14)
            if av(15) = '1' then
                case bidx14 is
                    when "00" => a_y <= if_y; a_u <= if_u; a_v <= if_v;
                    when "01" => a_y <= br_y;
                                 a_u <= C_BR_U; a_v <= C_BR_V;
                    when "10" => a_y <= ch_y; a_u <= ch_u; a_v <= ch_v;
                    when others => a_y <= unsigned(vr_q(29 downto 20));
                                   a_u <= unsigned(vr_q(19 downto 10));
                                   a_v <= unsigned(vr_q(9 downto 0));
                end case;
                if bidx14 = "11" then
                    v_bi := "11";
                else
                    v_bi := bidx14 + 1;
                end if;
                case v_bi is
                    when "00" => b_y <= if_y; b_u <= if_u; b_v <= if_v;
                    when "01" => b_y <= br_y;
                                 b_u <= C_BR_U; b_v <= C_BR_V;
                    when "10" => b_y <= ch_y; b_u <= ch_u; b_v <= ch_v;
                    when others => b_y <= unsigned(vr_q(29 downto 20));
                                   b_u <= unsigned(vr_q(19 downto 10));
                                   b_v <= unsigned(vr_q(9 downto 0));
                end case;
                f8_15 <= f8_14;
            end if;

            -- S16: soft-key blend multiplies, alone in their stage; the
            -- (A - dry) Mercury pre-difference runs in parallel, so S17 is
            -- a single add: strata - dry = (A - dry) + (B-A)*f8>>8, and
            -- since f8 <= 248 the blend can never leave [A, B] (no clamp)
            if av(16) = '1' then
                -- f8 top 7 bits (LC squeeze): blend recombine shifts >>7
                bdy <= (signed(resize(b_y, 11)) - signed(resize(a_y, 11)))
                       * signed('0' & f8_15(7 downto 1));
                bdu <= (signed(resize(b_u, 11)) - signed(resize(a_u, 11)))
                       * signed('0' & f8_15(7 downto 1));
                bdv <= (signed(resize(b_v, 11)) - signed(resize(a_v, 11)))
                       * signed('0' & f8_15(7 downto 1));
                admy <= signed(resize(a_y, 11))
                        - signed(resize(unsigned(vr_qd(29 downto 20)), 11));
                admu <= signed(resize(a_u, 11))
                        - signed(resize(unsigned(vr_qd(19 downto 10)), 11));
                admv <= signed(resize(a_v, 11))
                        - signed(resize(unsigned(vr_qd(9 downto 0)), 11));
            end if;

            -- S17: Mercury morph difference (one add of registered values)
            if av(17) = '1' then
                dmy <= admy + resize(shift_right(bdy, 7), 11);
                dmu <= admu + resize(shift_right(bdu, 7), 11);
                dmv <= admv + resize(shift_right(bdv, 7), 11);
            end if;

            -- S18: the Mercury multiplies, alone in their stage (p12 top
            -- 8 bits, LC squeeze: 256 morph steps)
            if av(18) = '1' then
                pmy <= dmy * signed('0' & s_p12(9 downto 2));
                pmu <= dmu * signed('0' & s_p12(9 downto 2));
                pmv <= dmv * signed('0' & s_p12(9 downto 2));
            end if;

            -- S19: recombine with the dry signal + clamp; output register
            if av(19) = '1' then
                s_out_y <= f_cu10(signed(resize(unsigned(vr_q19(29 downto 20)), 12))
                                  + resize(shift_right(pmy, 8), 12));
                s_out_u <= f_cu10(signed(resize(unsigned(vr_q19(19 downto 10)), 12))
                                  + resize(shift_right(pmu, 8), 12));
                s_out_v <= f_cu10(signed(resize(unsigned(vr_q19(9 downto 0)), 12))
                                  + resize(shift_right(pmv, 8), 12));
            end if;

            -- line bookkeeping (phosphor seen mechanism) + rubber row
            -- bases, updater arming, hblank prefetch kick, line fraction
            u_go <= '0';
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                lrx <= (others => '0');
                lwx <= (others => '0');
                cbx <= (others => '0');
                if s_x_count > 0 then
                    s_lwidth <= s_x_count;
                end if;
                s_x_count <= (others => '0');

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
                        -- this cell row is complete: spring-step it, then
                        -- advance the display row bases
                        u_go       <= '1';
                        u_rb       <= dsp_rb;
                        s_pend_row <= '0';
                        dsp_rb     <= dsp_rb + resize(s_stride, 11);
                        dsp_rb1    <= dsp_rb1 + resize(s_stride, 11);
                    else
                        s_pend_row <= '1';
                    end if;
                else
                    -- vblank: fire the final partial cell row once, then
                    -- reset the field-vertical state
                    if s_pend_row = '1' then
                        u_go       <= '1';
                        u_rb       <= dsp_rb;
                        s_pend_row <= '0';
                    end if;
                    s_aline <= (others => '0');
                    v_nal   := (others => '0');
                    dsp_rb  <= (others => '0');
                    dsp_rb1 <= resize(s_stride, 11);
                end if;

                case s_ysh is
                    when 3      => fy5 <= v_nal(2 downto 0) & "00";
                    when 4      => fy5 <= v_nal(3 downto 0) & '0';
                    when others => fy5 <= v_nal(4 downto 0);
                end case;

                pf_seq <= "001";   -- prefetch the coming line's cols 0..1
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- rubber spring updater: one cell row per hblank, 7-deep pipeline,
    -- 1 cell/clock (worst 60+7 clocks vs 138-clock minimum hblank). All
    -- shifts, zero multiplies; runs even when Rubber is bypassed so the
    -- springs keep tracking for an instant re-engage.
    ------------------------------------------------------------------------
    p_upd : process(clk)
        variable v_e   : signed(10 downto 0);
        variable v_b   : unsigned(1 downto 0);
        variable v_s   : signed(11 downto 0);
        variable v_p   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- free-running 16-bit Galois LFSR (spring dither)
            if s_lfsr(0) = '1' then
                s_lfsr <= ('0' & s_lfsr(15 downto 1)) xor x"B400";
            else
                s_lfsr <= '0' & s_lfsr(15 downto 1);
            end if;

            -- issue loop
            if u_go = '1' then
                u_active <= '1';
                u_cx     <= (others => '0');
            elsif u_active = '1' then
                if u_cx = s_cols - 1 then
                    u_active <= '0';
                end if;
                u_cx <= u_cx + 1;
            end if;
            if u_active = '1' then
                u_a0  <= resize(u_rb, 11) + resize(u_cx, 11);
                u_vca <= resize(u_cx, 8);
            end if;
            u_v(1) <= u_active;
            for i in 2 to 7 loop
                u_v(i) <= u_v(i - 1);
            end loop;
            u_a0d <= u_a0;

            -- U1 (v(2)): land the BRAM reads in plain FFs
            if u_v(2) = '1' then
                u_pos1 <= unsigned(state_q(19 downto 10));
                u_vel1 <= signed(state_q(9 downto 0));
                u_t1   <= unsigned(vacc_q(11 downto 2));   -- 8.2, matches pos
                u_a1   <= u_a0d;
            end if;

            -- U2 (v(3)): error + luma-dependent stiffness shift, damping
            if u_v(3) = '1' then
                v_e := signed(resize(u_t1, 11)) - signed(resize(u_pos1, 11));
                v_b := u_t1(9 downto 8);
                if s_inv_stiff = '1' then
                    v_b := not v_b;
                end if;
                u_ek2  <= shift_right(v_e, 6 - to_integer(v_b));  -- ksh 3..6
                u_vd2  <= u_vel1
                          - resize(shift_right(u_vel1,
                                   2 + to_integer(s_p6z)), 10);   -- dsh 2..5
                u_pos2 <= u_pos1;
                u_t2   <= u_t1;
                u_a2   <= u_a1;
            end if;

            -- U3 (v(4)): velocity update + clamp + LFSR dither (breaks
            -- limit cycles into slow breathing)
            if u_v(4) = '1' then
                v_s := resize(u_vd2, 12) + resize(u_ek2, 12);
                if s_lfsr(0) = '1' then
                    v_s := v_s + 1;
                else
                    v_s := v_s - 1;
                end if;
                u_vel3 <= f_cs10(v_s);
                u_pos3 <= u_pos2;
                u_t3   <= u_t2;
                u_a3   <= u_a2;
            end if;

            -- U4 (v(5)): position update + clamp
            if u_v(5) = '1' then
                v_p := signed(resize(u_pos3, 12))
                       + resize(shift_right(u_vel3, 1), 12);
                u_pos4 <= f_cu10(v_p);
                u_vel4 <= u_vel3;
                u_t4   <= u_t3;
                u_a4   <= u_a3;
            end if;

            -- U5 (v(6)): residual = (pos - target) in luma8, clamped
            if u_v(6) = '1' then
                u_res5 <= f_cs8(shift_right(signed(resize(u_pos4, 11))
                                            - signed(resize(u_t4, 11)), 2));
                u_a5   <= u_a4;
            end if;
        end if;
    end process p_upd;

    ------------------------------------------------------------------------
    -- rubber memories. state_ram R/W is updater-only; res_ram W updater /
    -- R display; vacc R is time-muxed (updater in hblank, accumulator IIR
    -- during active) and W is accumulator-only -- no port ever contends.
    ------------------------------------------------------------------------
    -- the updater's vacc address must be the REGISTERED copy (u_vca) so it
    -- pairs with the same cell as the registered state address u_a0
    vacc_raddr <= u_vca when u_v(1) = '1' else acc_pre_col;

    p_state : process(clk)
    begin
        if rising_edge(clk) then
            state_q <= state_ram(to_integer(u_a0));
            if u_v(6) = '1' then
                state_ram(to_integer(u_a4)) <=
                    std_logic_vector(u_pos4) & std_logic_vector(u_vel4);
            end if;
        end if;
    end process p_state;

    p_res : process(clk)
    begin
        if rising_edge(clk) then
            res_q <= res_ram(to_integer(res_raddr));
            if u_v(7) = '1' then
                res_ram(to_integer(u_a5)) <= std_logic_vector(u_res5);
            end if;
        end if;
    end process p_res;

    p_vacc : process(clk)
    begin
        if rising_edge(clk) then
            vacc_q <= vacc(to_integer(vacc_raddr));
            if vacc_we = '1' then
                vacc(to_integer(vacc_waddr)) <= std_logic_vector(vacc_wdata);
            end if;
        end if;
    end process p_vacc;

    ------------------------------------------------------------------------
    -- height line buffers (canonical 1W1R; read leads write by one column)
    ------------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            q1 <= lb1(to_integer(lrx));
            if av(5) = '1' then
                lb1(to_integer(lwx)) <= std_logic_vector(h4);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            q2 <= lb2(to_integer(lrx));
            if av(5) = '1' then
                lb2(to_integer(lwx)) <= q1;
            end if;
        end if;
    end process p_lb2;

    ------------------------------------------------------------------------
    -- dry video delay ring
    ------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            vring(to_integer(vr_wp)) <= data_in.y & data_in.u & data_in.v;
            vr_q  <= vring(to_integer(vr_wp - 14));
            vr_qd <= vr_q;
            vr_wp <= vr_wp + 1;
        end if;
    end process p_ring;

    p_ring_b : process(clk)
    begin
        if rising_edge(clk) then
            vring_b(to_integer(vr_wp)) <= data_in.y & data_in.u & data_in.v;
            vr_q19 <= vring_b(to_integer(vr_wp - 18));
        end if;
    end process p_ring_b;

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

end architecture mercurial;
