-- turpentine.vhd
--
-- TURPENTINE -- chroma as wet paint.
--
-- Redshift treated luma as mass-energy; Mercurial treated luma as material.
-- Turpentine exploits what chroma actually IS: a 2D vector per pixel.
-- (U-512, V-512) has a magnitude (saturation) and a direction (hue), so the
-- picture carries its own force field. Here that field is read as PIGMENT:
--   saturation = how much wet paint is loaded at that pixel (gray = bare
--                paper, it never moves)
--   hue        = the direction the paint runs when solvent hits it
--
-- P12 "Solvent" (headline, 0 = exact dry video by construction):
--   0..25%   VARNISH   the picture becomes a painting: brush grain rises,
--                      strokes oriented along each region's hue direction,
--                      canvas tooth in unpainted (low-sat) areas
--   25..70%  WASH      solvent soaks in: paint runs -- a per-pixel
--                      hue-directed recursive smear, wetness ~ saturation;
--                      the grain progressively dissolves
--   70..100% FLOOD     wetness floor rises (even weak colors run), flow and
--                      turbulence boost, drips detach from saturated
--                      surfaces and streak down carrying their color
--
-- The core engine is shearline's recursive line feedback (redraw the
-- previous output line, blend live video back in), upgraded from a global
-- per-line shift + global decay to PER-PIXEL terms derived from the pixel's
-- own chroma vector:
--   dx = horizontal component of the compass-rotated chroma vector
--   t  = wetness = f(saturation), clamped by the Flow ceiling
-- and one crucial twist: t is computed from max(live sat, sat of the
-- ARRIVING paint read from the previous line). The paint carries its own
-- solvent -- a saturated region invades the dry paper below it, and the
-- (1-t) live inject gradually dilutes the carried pigment until the run
-- dries out on its own. Bounded lerp = unconditionally stable.
--
-- Controls:
--   K1 Compass    rotates the hue -> run-direction map (S11 slowly orbits)
--   K2 Brush      grain stripe pitch (top zone 0 = grain off)
--   K3 Flow       trail persistence ceiling (how far paint runs)
--   K4 Turbulence per-line jitter of the run direction (wiggly wet edges)
--   K5 Pigment    output chroma boost (shift-add knees, no multiplies)
--   K6 Absorb     paper absorbency: saturation threshold to start running
--   S7 Medium     Paint (colors run) / Ink (solvent runs, pigment pinned)
--   S8 Grain      brush strokes + canvas tooth on/off
--   S9 Drips      drip particles on/off
--   S10 Residue   runs Thin (lighten) / Stain (darken) as they travel
--   S11 Creep     compass auto-rotation (keeps static input alive)
--
-- Architecture: single streaming pipeline E1..E15 (C_LATENCY = 15).
--   * Feedback store: shearline's inlined dual-bank ping-pong line buffers
--     (Y full 2048, U/V 2:1 decimated; 22 EBR), read parity = live, write
--     parity carried per-pixel down the pipe (trailing writes finish into
--     the old bank -- no boundary handling).
--   * Chroma math: 4 s8 x s8 rotate products (per-frame cos/sin from the
--     64-entry quarter-wave qsin logic ROM, angle and output registered --
--     mercurial), sat = max + min/2, dx = rx * gain (1 mult) clamped +/-15.
--   * Wetness: 1 u8 x u8 mult after the max(live, carried) combine; the
--     t*(prev-live) lerp is 3 s11 x s11 mults. 8 per-pixel multiplies total;
--     pigment boost and grain are shift-only.
--   * Grain: flow-octant-oriented triangle stripes (w = x, y, x+y or x-y by
--     octant -- strokes run ALONG the flow, zero multiplies) on painted
--     areas; two-stage hash canvas tooth on bare paper. Display-only, never
--     written back into the feedback store.
--   * Drips: redshift's per-8px-column particle machinery. surf_ram
--     {row 10, u3, v3} = topmost saturated row per column + its quantized
--     chroma (raster min-write + 1-row/field drift re-adapt); part_ram
--     {pos 10, u3, v3} drips fall 2..5 rows/field and respawn AT the
--     surface with its color, density-gated by the flood zone. Vblank
--     column-walk FSM, BRAM reads landed in FFs before use (redshift
--     lesson: those paths are STA-timed at full clock).
--
-- Timing discipline (mercurial/redshift): every multiply alone in its
-- stage with registered inputs; BRAM reads land in plain FFs; the qsin
-- angle and output are registered; the vblank sequencer builds all
-- per-frame terms one add per step; lanes assembled one stage before the
-- output mux. EBR: feedback 22 + drips 2 = 24 of 32.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture turpentine of program_top is

    constant C_LATENCY : integer := 15;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
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

    function f_cu8(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 8);
        elsif v > 255 then
            return to_unsigned(255, 8);
        else
            return resize(unsigned(v), 8);
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

    -- |s8| -> u7, safe at -128 (clamps to 127)
    function f_abs7(v : signed(7 downto 0)) return unsigned is
        variable v9 : signed(8 downto 0);
    begin
        v9 := resize(v, 9);
        if v9 < 0 then
            v9 := -v9;
        end if;
        if v9 > 127 then
            v9 := to_signed(127, 9);
        end if;
        return resize(unsigned(v9(6 downto 0)), 7);
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

    -- value-noise hash, split across two pipeline stages (inferno lesson)
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
    signal s_p12      : unsigned(9 downto 0) := (others => '0');
    signal s_k2       : unsigned(9 downto 0) := (others => '0');
    signal s_k3       : unsigned(9 downto 0) := (others => '0');
    signal s_k4       : unsigned(9 downto 0) := (others => '0');
    signal s_k5       : unsigned(9 downto 0) := (others => '0');
    signal s_k6       : unsigned(9 downto 0) := (others => '0');
    signal s_ink      : std_logic := '0';                          -- S7
    signal s_grain_sw : std_logic := '1';                          -- S8
    signal s_drips_sw : std_logic := '1';                          -- S9
    signal s_stain    : std_logic := '1';                          -- S10
    signal s_creep_on : std_logic := '0';                          -- S11

    signal s_ang       : unsigned(9 downto 0) := (others => '0');
    signal s_creep_acc : unsigned(9 downto 0) := (others => '0');
    signal s_cos8      : signed(7 downto 0) := to_signed(127, 8);
    signal s_sin8      : signed(7 downto 0) := (others => '0');

    -- solvent zone envelopes (built one add per sequencer step)
    signal s_wash9   : unsigned(9 downto 0) := (others => '0');   -- 0..512
    signal s_flood9  : unsigned(9 downto 0) := (others => '0');   -- 0..319
    signal s_flood8  : unsigned(7 downto 0) := (others => '0');
    signal s_gv9     : unsigned(9 downto 0) := (others => '0');   -- grain rise
    signal s_dis9    : unsigned(9 downto 0) := (others => '0');   -- grain fall
    signal s_grain8  : unsigned(7 downto 0) := (others => '0');   -- envelope
    signal s_gwet8   : unsigned(7 downto 0) := (others => '0');   -- wet gain
    signal s_wfloor8 : unsigned(7 downto 0) := (others => '0');   -- flood floor
    signal s_gdx8    : unsigned(7 downto 0) := (others => '0');   -- run gain
    signal s_tmax10  : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_thr8    : unsigned(7 downto 0) := to_unsigned(12, 8);
    signal s_cpnt10  : signed(9 downto 0) := to_signed(20, 10);   -- 8 + thr
    signal s_cink10  : signed(9 downto 0) := to_signed(170, 10);  -- 182 - thr
    signal s_grain_en : std_logic := '0';
    signal s_grsh    : natural range 0 to 3 := 0;                 -- stroke amp
    signal s_psh     : natural range 1 to 3 := 1;                 -- stroke pitch
    signal s_gdrip8  : unsigned(7 downto 0) := (others => '0');
    signal s_drip_en : std_logic := '0';
    signal s_jamt    : natural range 0 to 4 := 0;                 -- turbulence
    signal s_pigsel  : unsigned(1 downto 0) := (others => '0');

    signal s_seq   : unsigned(3 downto 0) := (others => '0');
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');     -- reg angle
    signal s_qs_r  : signed(9 downto 0) := (others => '0');       -- reg ROM out

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';

    ----------------------------------------------------------------------
    -- position / line bookkeeping
    ----------------------------------------------------------------------
    signal s_x_count : unsigned(10 downto 0) := (others => '0');
    signal s_aline   : unsigned(9 downto 0) := (others => '0');
    signal s_seen    : std_logic := '0';
    signal s_lwidth  : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_wmax    : unsigned(10 downto 0) := to_unsigned(1919, 11);
    signal s_wedge   : unsigned(10 downto 0) := to_unsigned(1880, 11);
    signal s_nrows   : unsigned(9 downto 0) := to_unsigned(540, 10);
    signal s_par     : std_logic := '0';
    signal s_ln0     : std_logic := '1';                          -- first line

    -- per-line turbulence (2-step hblank sequencer)
    signal s_lstep  : unsigned(1 downto 0) := (others => '0');
    signal s_lrand5 : unsigned(4 downto 0) := (others => '0');
    signal s_tj5    : signed(4 downto 0) := (others => '0');

    signal s_lfsr : unsigned(15 downto 0) := x"C0FE";

    ----------------------------------------------------------------------
    -- E1..E5: input regs, rotate mults, sat, dx, grain track
    ----------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal px1 : unsigned(10 downto 0) := (others => '0');

    signal p_vc2, p_us2, p_uc2, p_vs2 : signed(15 downto 0) := (others => '0');
    signal au2, av2 : unsigned(6 downto 0) := (others => '0');
    signal u3_2, v3_2 : unsigned(2 downto 0) := (others => '0');
    signal px2 : unsigned(10 downto 0) := (others => '0');

    signal rx3, ry3 : signed(7 downto 0) := (others => '0');
    signal ax3, ay3 : unsigned(6 downto 0) := (others => '0');
    signal sat3     : unsigned(7 downto 0) := (others => '0');
    signal u3_3, v3_3 : unsigned(2 downto 0) := (others => '0');
    signal px3 : unsigned(10 downto 0) := (others => '0');

    signal dxp4   : signed(14 downto 0) := (others => '0');       -- M5 rx*gain
    signal wsl4   : unsigned(7 downto 0) := (others => '0');
    signal o4     : unsigned(1 downto 0) := (others => '0');      -- flow octant
    signal wpm4, wpp4 : unsigned(11 downto 0) := (others => '0');
    signal satlo4 : std_logic := '0';
    signal ch_a4  : unsigned(11 downto 0) := (others => '0');
    signal px4    : unsigned(10 downto 0) := (others => '0');

    signal rd5y   : unsigned(10 downto 0) := (others => '0');     -- read addr
    signal bg5    : std_logic := '0';
    signal wsl5   : unsigned(7 downto 0) := (others => '0');      -- thresholded
    signal w3_5   : unsigned(2 downto 0) := (others => '0');
    signal satlo5 : std_logic := '0';
    signal ch8_5  : unsigned(7 downto 0) := (others => '0');
    signal px5    : unsigned(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- feedback line buffers: inlined dual-bank ping-pong (shearline)
    ----------------------------------------------------------------------
    type t_lb_y  is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_uv is array (0 to 1023) of std_logic_vector(9 downto 0);
    signal lbY0, lbY1 : t_lb_y  := (others => (others => '0'));
    signal lbU0, lbU1 : t_lb_uv := (others => (others => '0'));
    signal lbV0, lbV1 : t_lb_uv := (others => (others => '0'));
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdV0, s_rdV1 : std_logic_vector(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- E6..E12: capture, prep, wetness, lerp, result
    ----------------------------------------------------------------------
    signal a_rbank6 : std_logic := '0';
    signal gr6      : signed(5 downto 0) := (others => '0');

    -- context pipes E6 -> E11 (write side) / E9 (t gates)
    signal parp  : std_logic_vector(6 to 11) := (others => '0');
    signal wep   : std_logic_vector(6 to 11) := (others => '0');
    type t_addr_p is array (6 to 11) of unsigned(10 downto 0);
    signal addrp : t_addr_p := (others => (others => '0'));
    signal bgp   : std_logic_vector(6 to 9) := (others => '0');
    signal ln0p  : std_logic_vector(6 to 9) := (others => '0');

    type t_u10_p is array (6 to 11) of unsigned(9 downto 0);
    signal lvY, lvU, lvV : t_u10_p := (others => (others => '0'));

    signal wsl6, wsl7 : unsigned(7 downto 0) := (others => '0');

    signal dY7, dU7, dV7 : signed(10 downto 0) := (others => '0');
    signal ap_u7, ap_v7  : unsigned(7 downto 0) := (others => '0');  -- 1.5*|c|

    signal dY8, dU8, dV8 : signed(10 downto 0) := (others => '0');
    signal wet8          : unsigned(7 downto 0) := (others => '0');

    signal dY9, dU9, dV9 : signed(10 downto 0) := (others => '0');
    signal tp9           : unsigned(15 downto 0) := (others => '0');

    signal dY10, dU10, dV10 : signed(10 downto 0) := (others => '0');
    signal t10    : unsigned(9 downto 0) := (others => '0');
    signal bias10 : signed(2 downto 0) := (others => '0');

    signal prY11, prU11, prV11 : signed(19 downto 0) := (others => '0');
    signal lvYb11 : signed(11 downto 0) := (others => '0');  -- live Y + bias

    -- combinational blend result (bank write-back + display)
    signal s_resY, s_resU, s_resV : std_logic_vector(9 downto 0);

    signal wet12_y, wet12_u, wet12_v : unsigned(9 downto 0) := (others => '0');

    -- grain pipe E6 -> E12
    signal grp : signed(5 downto 0) := (others => '0');
    signal gr7, gr8, gr9, gr10, gr11, gr12 : signed(5 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- E13..E15: grain apply, pigment boost, lanes, output mux
    ----------------------------------------------------------------------
    signal y13, u13, v13 : unsigned(9 downto 0) := (others => '0');

    signal ln_vy, ln_vu, ln_vv : unsigned(9 downto 0) := (others => '0');
    signal ln_ty : unsigned(9 downto 0) := (others => '0');
    signal ln_du, ln_dv : unsigned(9 downto 0) := (others => '0');
    signal sel14 : unsigned(1 downto 0) := (others => '0');

    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- DRIPS: per-8px-column particle machinery (redshift precipitation)
    ----------------------------------------------------------------------
    type t_p256 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal part_ram : t_p256 := (others => "1111111111" & "000000");
    signal surf_ram : t_p256 := (others => "1111111111" & "000000");
    signal part_q, surf_q : std_logic_vector(15 downto 0) := (others => '0');
    signal part_raddr, surf_raddr : unsigned(7 downto 0) := (others => '0');
    signal prf_col : unsigned(7 downto 0) := (others => '0');
    signal pf_seq  : unsigned(1 downto 0) := (others => '0');

    -- render-side column registers (current + prefetched next)
    signal pc_pos : unsigned(9 downto 0) := (others => '1');
    signal pn_pos : unsigned(9 downto 0) := (others => '1');
    signal pc_u3, pn_u3 : unsigned(2 downto 0) := (others => '0');
    signal pc_v3, pn_v3 : unsigned(2 downto 0) := (others => '0');
    signal pc_act : std_logic := '0';
    signal sc_bs, sn_bs : unsigned(9 downto 0) := (others => '1');
    signal pc_h8, pn_h8 : unsigned(7 downto 0) := (others => '0');
    signal p_ha : unsigned(11 downto 0) := (others => '0');

    -- hit-flag + color pipes (E2 -> E13, consumed at the E14 lane select)
    signal fl_t, fl_h : std_logic_vector(2 to 13) := (others => '0');
    type t_u3_p is array (2 to 13) of unsigned(2 downto 0);
    signal cu3p, cv3p : t_u3_p := (others => (others => '0'));

    -- raster saturated-surface detect write
    signal sw_we   : std_logic := '0';
    signal sw_addr : unsigned(7 downto 0) := (others => '0');
    signal sw_data : std_logic_vector(15 downto 0) := (others => '0');

    -- vblank updater FSM (redshift: land BRAM reads first, decide later)
    type t_pu is (PU_IDLE, PU_REQ, PU_W1, PU_W2, PU_LAND, PU_CALC, PU_DECIDE);
    signal pu_state : t_pu := PU_IDLE;
    signal pu_cnt   : unsigned(7 downto 0) := (others => '0');
    signal pl_pos   : unsigned(9 downto 0) := (others => '0');
    signal pl_u3, pl_v3 : unsigned(2 downto 0) := (others => '0');
    signal ps_bs    : unsigned(9 downto 0) := (others => '0');
    signal ps_u3, ps_v3 : unsigned(2 downto 0) := (others => '0');
    signal pl_np    : unsigned(9 downto 0) := (others => '0');
    signal pl_end   : unsigned(9 downto 0) := (others => '0');
    signal pl_rgate : std_logic := '0';
    signal pu_busy  : std_logic := '0';
    signal pu_we, pu_swe : std_logic := '0';
    signal pu_waddr : unsigned(7 downto 0) := (others => '0');
    signal pu_wdata, pu_swdata : std_logic_vector(15 downto 0) := (others => '0');
    signal pu_prev_vsync_n : std_logic := '1';

    signal surf_waddr : unsigned(7 downto 0) := (others => '0');
    signal surf_wdata : std_logic_vector(15 downto 0) := (others => '0');
    signal surf_we    : std_logic := '0';

    ----------------------------------------------------------------------
    -- dry / sync delay shift registers
    ----------------------------------------------------------------------
    type t_data_sr is array (0 to 4) of std_logic_vector(9 downto 0);
    signal s_y_sr, s_u_sr, s_v_sr : t_data_sr := (others => (others => '0'));

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared quarter-wave sine: only the sequencer uses it, but the angle
    -- and ROM output are still registered (mercurial: STA-timed at full
    -- clock even though captures fire only in vblank). Captures two-behind.
    ------------------------------------------------------------------------
    s_qs_a <= s_ang                          when s_seq = 10 else
              s_ang + to_unsigned(256, 10);

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (one add per step)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_t : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_p12 <= unsigned(registers_in(7));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_ink      <= registers_in(6)(0);   -- switch 7
                s_grain_sw <= registers_in(6)(1);   -- switch 8
                s_drips_sw <= registers_in(6)(2);   -- switch 9
                s_stain    <= registers_in(6)(3);   -- switch 10
                s_creep_on <= registers_in(6)(4);   -- switch 11

                s_creep_acc <= s_creep_acc + 1;
                if registers_in(6)(4) = '1' then
                    s_ang <= unsigned(registers_in(0)) + s_creep_acc;
                else
                    s_ang <= unsigned(registers_in(0));
                end if;

                s_seq <= to_unsigned(10, 4);
            elsif s_seq /= 0 and data_in.avid = '0' then
                case to_integer(s_seq) is
                    when 10 =>
                        -- solvent zone splits (qsin presented: ang)
                        if s_p12 > 288 then
                            if s_p12 > 799 then
                                s_wash9 <= to_unsigned(511, 10);
                            else
                                s_wash9 <= resize(s_p12 - 288, 10);
                            end if;
                        else
                            s_wash9 <= (others => '0');
                        end if;
                        if s_p12 > 704 then
                            s_flood9 <= resize(s_p12 - 704, 10);
                        else
                            s_flood9 <= (others => '0');
                        end if;
                        if s_p12 < 256 then
                            s_gv9 <= s_p12;
                        else
                            s_gv9 <= to_unsigned(256, 10);
                        end if;
                        if s_p12 > 640 then
                            s_dis9 <= resize(s_p12 - 640, 10);
                        else
                            s_dis9 <= (others => '0');
                        end if;
                    when 9 =>
                        -- (qsin presented: ang + 256)
                        if s_flood9 > 255 then
                            s_flood8 <= (others => '1');
                        else
                            s_flood8 <= resize(s_flood9, 8);
                        end if;
                        if s_gv9 > s_dis9 then
                            if s_gv9 - s_dis9 > 255 then
                                s_grain8 <= (others => '1');
                            else
                                s_grain8 <= resize(s_gv9 - s_dis9, 8);
                            end if;
                        else
                            s_grain8 <= (others => '0');
                        end if;
                        if s_wash9(9 downto 1) > 255 then
                            s_gwet8 <= (others => '1');
                        else
                            s_gwet8 <= resize(s_wash9(9 downto 1), 8);
                        end if;
                    when 8 =>
                        -- capture two-behind: s_qs_r = qsin(ang)
                        s_sin8 <= resize(shift_right(s_qs_r, 2), 8);
                        s_wfloor8 <= "000" & s_flood8(7 downto 3);
                        s_gdx8 <= resize(s_wash9(8 downto 4), 8)
                                  + resize(s_flood8(7 downto 4), 8);
                    when 7 =>
                        -- capture: s_qs_r = qsin(ang + 256) = cos(ang)
                        s_cos8 <= resize(shift_right(s_qs_r, 2), 8);
                        v_t := to_unsigned(640, 11) + resize(s_k3(9 downto 1), 11);
                        if v_t > 1008 then
                            s_tmax10 <= to_unsigned(1008, 10);
                        else
                            s_tmax10 <= resize(v_t, 10);
                        end if;
                        s_thr8 <= to_unsigned(12, 8) + resize(s_k6(9 downto 3), 8);
                    when 6 =>
                        -- wetness constants with the Absorb threshold and
                        -- the carried-paint handicap folded in (the E8
                        -- serial chain was the v0.1 routed critical path:
                        -- max(a,b)-thr = max(a-thr, b-thr))
                        s_cpnt10 <= to_signed(8, 10)
                                    + signed(resize(s_thr8, 10));
                        s_cink10 <= to_signed(182, 10)
                                    - signed(resize(s_thr8, 10));
                        if s_grain8 < 32 or s_grain_sw = '0'
                           or s_k2(9 downto 8) = "00" then
                            s_grain_en <= '0';
                        else
                            s_grain_en <= '1';
                        end if;
                        s_grsh <= to_integer(s_grain8(7 downto 6));
                        if s_k2(9 downto 8) = "00" then
                            s_psh <= 1;
                        else
                            s_psh <= to_integer(s_k2(9 downto 8));
                        end if;
                    when 5 =>
                        if s_drips_sw = '1' then
                            s_gdrip8 <= s_flood8;
                        else
                            s_gdrip8 <= (others => '0');
                        end if;
                        if s_drips_sw = '1' and s_flood8 /= 0 then
                            s_drip_en <= '1';
                        else
                            s_drip_en <= '0';
                        end if;
                    when 4 =>
                        if s_k4(9 downto 8) = "00" then
                            s_jamt <= 0;
                        elsif s_flood8(7) = '1' and s_k4(9 downto 8) = "11" then
                            s_jamt <= 4;
                        elsif s_flood8(7) = '1' then
                            s_jamt <= to_integer(s_k4(9 downto 8)) + 1;
                        else
                            s_jamt <= to_integer(s_k4(9 downto 8));
                        end if;
                    when 3 =>
                        s_pigsel <= s_k5(9 downto 8);
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- main pixel pipeline E1..E15
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_cu8, v_cv8 : signed(7 downto 0);
        variable v_rx8, v_ry8 : signed(7 downto 0);
        variable v_ws   : signed(10 downto 0);
        variable v_w12  : unsigned(11 downto 0);
        variable v_t2   : unsigned(1 downto 0);
        variable v_gr   : signed(5 downto 0);
        variable v_dx   : signed(6 downto 0);
        variable v_rd   : signed(12 downto 0);
        variable v_pvY, v_pvU, v_pvV : unsigned(9 downto 0);
        variable v_pu8, v_pv8 : signed(7 downto 0);
        variable v_ab7  : unsigned(6 downto 0);
        variable v_sp   : unsigned(7 downto 0);
        variable v_spe  : signed(9 downto 0);
        variable v_t12  : unsigned(11 downto 0);
        variable v_tt   : unsigned(9 downto 0);
        variable v_sumY, v_sumU, v_sumV : signed(12 downto 0);
        variable v_cn   : signed(5 downto 0);
        variable v_cur, v_cvr : signed(9 downto 0);
        variable v_ext  : signed(9 downto 0);
        variable v_pd   : unsigned(9 downto 0);
        variable v_xr   : unsigned(2 downto 0);
        variable v_hit  : std_logic;
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            --------------------------------------------------------------
            -- E1: input registers, position, drip column prefetch
            --------------------------------------------------------------
            s_in_y <= unsigned(data_in.y);
            s_in_u <= unsigned(data_in.u);
            s_in_v <= unsigned(data_in.v);
            px1    <= s_x_count;

            if data_in.avid = '1' then
                s_x_count <= s_x_count + 1;
                s_seen    <= '1';

                -- drip column prefetch (8-px spans, redshift pattern)
                case s_x_count(2 downto 0) is
                    when "000" =>
                        pc_pos <= pn_pos;
                        pc_u3  <= pn_u3;
                        pc_v3  <= pn_v3;
                        pc_h8  <= pn_h8;
                        sc_bs  <= sn_bs;
                        if pn_pos < 1000 then
                            pc_act <= '1';
                        else
                            pc_act <= '0';
                        end if;
                    when "001" =>
                        prf_col <= s_x_count(10 downto 3) + 1;
                    when "010" =>
                        p_ha <= f_hash_a(prf_col, x"3A", x"71C5");
                    when "011" =>
                        pn_pos <= unsigned(part_q(15 downto 6));
                        pn_u3  <= unsigned(part_q(5 downto 3));
                        pn_v3  <= unsigned(part_q(2 downto 0));
                        sn_bs  <= unsigned(surf_q(15 downto 6));
                    when "100" =>
                        pn_h8 <= f_hash_b(p_ha);
                    when others =>
                        null;
                end case;
            end if;

            -- hblank prefetch of the coming line's column 0
            if pf_seq /= 0 then
                case pf_seq is
                    when "01" =>
                        prf_col <= (others => '0');
                        pf_seq  <= "10";
                    when "10" =>
                        p_ha   <= f_hash_a(x"00", x"3A", x"71C5");
                        pf_seq <= "11";
                    when others =>
                        pn_pos <= unsigned(part_q(15 downto 6));
                        pn_u3  <= unsigned(part_q(5 downto 3));
                        pn_v3  <= unsigned(part_q(2 downto 0));
                        sn_bs  <= unsigned(surf_q(15 downto 6));
                        pn_h8  <= f_hash_b(p_ha);
                        pf_seq <= "00";
                end case;
            end if;

            -- dry data shift registers (taps at index 4 = E6 capture)
            s_y_sr(0) <= data_in.y;
            s_u_sr(0) <= data_in.u;
            s_v_sr(0) <= data_in.v;
            for i in 1 to 4 loop
                s_y_sr(i) <= s_y_sr(i - 1);
                s_u_sr(i) <= s_u_sr(i - 1);
                s_v_sr(i) <= s_v_sr(i - 1);
            end loop;

            --------------------------------------------------------------
            -- E2: the four rotate products (s8 x s8, per-frame cos/sin),
            -- chroma magnitudes, drip hit tests
            --------------------------------------------------------------
            v_cu8 := signed((not s_in_u(9)) & s_in_u(8 downto 2));  -- (u-512)/4
            v_cv8 := signed((not s_in_v(9)) & s_in_v(8 downto 2));

            p_vc2 <= v_cv8 * s_cos8;
            p_us2 <= v_cu8 * s_sin8;
            p_uc2 <= v_cu8 * s_cos8;
            p_vs2 <= v_cv8 * s_sin8;

            au2 <= f_abs7(v_cu8);
            av2 <= f_abs7(v_cv8);

            u3_2 <= s_in_u(9 downto 7);
            v3_2 <= s_in_v(9 downto 7);
            px2  <= px1;

            -- drip hit test: head 0..3 rows behind pos, tail 0..15; the
            -- drip's x inside its 8-px column comes from the column hash
            v_pd  := pc_pos - s_aline;
            v_xr  := px1(2 downto 0) - pc_h8(2 downto 0);
            v_hit := '0';
            if s_drip_en = '1' and pc_act = '1'
               and v_xr(2 downto 1) = "00" then
                v_hit := '1';
            end if;
            if v_hit = '1' and v_pd(9 downto 4) = "000000" then
                fl_t(2) <= '1';
            else
                fl_t(2) <= '0';
            end if;
            if v_hit = '1' and v_pd(9 downto 2) = "00000000" then
                fl_h(2) <= '1';
            else
                fl_h(2) <= '0';
            end if;
            cu3p(2) <= pc_u3;
            cv3p(2) <= pc_v3;

            -- flag / color pipes
            fl_t(3 to 13) <= fl_t(2 to 12);
            fl_h(3 to 13) <= fl_h(2 to 12);
            for i in 3 to 13 loop
                cu3p(i) <= cu3p(i - 1);
                cv3p(i) <= cv3p(i - 1);
            end loop;

            --------------------------------------------------------------
            -- E3: rotated flow vector (+ magnitudes) + saturation
            --------------------------------------------------------------
            v_rx8 := f_cs8(resize(shift_right(p_vc2 - p_us2, 7), 10));
            v_ry8 := f_cs8(resize(shift_right(p_uc2 + p_vs2, 7), 10));
            rx3 <= v_rx8;
            ry3 <= v_ry8;
            ax3 <= f_abs7(v_rx8);
            ay3 <= f_abs7(v_ry8);
            if au2 > av2 then
                sat3 <= resize(au2, 8) + resize(av2(6 downto 1), 8);
            else
                sat3 <= resize(av2, 8) + resize(au2(6 downto 1), 8);
            end if;
            u3_3 <= u3_2;
            v3_3 <= v3_2;
            px3  <= px2;

            --------------------------------------------------------------
            -- E4: dx multiply (alone), live wet source, grain stroke
            -- track, canvas hash A, raster surface detect
            --------------------------------------------------------------
            -- run gain fits u6 (max 46): a s8 x s7 multiply, not s8 x s9
            dxp4 <= rx3 * signed('0' & s_gdx8(5 downto 0));

            -- live wetness source: saturation (Ink inverts) + flood floor
            -- + downhill affinity (flow vectors pointing down run wetter)
            if s_ink = '1' then
                v_ws := to_signed(190, 11) - signed(resize(sat3, 11));
            else
                v_ws := signed(resize(sat3, 11));
            end if;
            v_ws := v_ws + signed(resize(s_wfloor8, 11))
                    + resize(shift_right(ry3, 2), 11);
            wsl4 <= f_cu8(resize(v_ws, 9));

            -- brush stroke octant (strokes run ALONG the flow: w constant
            -- along the stroke direction) -- code only, w muxed at E5
            if ax3 > shift_left(resize(ay3, 8), 1) then
                o4 <= "00";                                    -- horizontal
            elsif ay3 > shift_left(resize(ax3, 8), 1) then
                o4 <= "01";                                    -- vertical
            elsif rx3(7) = ry3(7) then
                o4 <= "10";                                    -- diag x-y
            else
                o4 <= "11";                                    -- diag x+y
            end if;
            wpm4 <= resize(px3, 12) - resize(s_aline, 12);
            wpp4 <= resize(px3, 12) + resize(s_aline, 12);
            if sat3 < 48 then
                satlo4 <= '1';
            else
                satlo4 <= '0';
            end if;
            ch_a4 <= f_hash_a(px3(9 downto 2), s_aline(9 downto 2), x"5EED");
            px4   <= px3;

            -- raster saturated-surface detect (lane pixel x%8 = 4):
            -- min-write {row, u3, v3}; the updater drifts the row +1 per
            -- field so it re-adapts when the picture moves
            sw_we <= '0';
            if s_avid_sr(2) = '1'
               and px3(2 downto 0) = "100" and sat3 > 100
               and s_aline < sc_bs then
                sw_we   <= '1';
                sw_addr <= px3(10 downto 3);
                sw_data <= std_logic_vector(s_aline)
                           & std_logic_vector(u3_3)
                           & std_logic_vector(v3_3);
            end if;

            --------------------------------------------------------------
            -- E5: REGISTER the read address (a combinational adder
            -- feeding 22 EBR address pins was routing-hostile); the
            -- serial add/clamp/subtract/compare chain here was the next
            -- routed critical path, so: no dx clamp (natural range +/-38
            -- px/line), left-edge test = the subtract's sign bit (free),
            -- right-edge test = px4 vs a per-line margin, in PARALLEL
            -- with the subtract. Out-of-line reads read dry (t = 0).
            -- Live wet source pre-thresholded here (E8 restage); stroke
            -- w muxed by octant + pitch shift.
            --------------------------------------------------------------
            v_dx := resize(shift_right(dxp4, 8), 7) + resize(s_tj5, 7);
            v_rd := signed(resize(px4, 13)) - resize(v_dx, 13);
            rd5y <= unsigned(v_rd(10 downto 0));
            if v_rd(12) = '1' or px4 > s_wedge then
                bg5 <= '1';
            else
                bg5 <= '0';
            end if;

            wsl5 <= f_cu8(signed(resize(wsl4, 9))
                          - signed(resize(s_thr8, 9)));

            case o4 is
                when "00"   => v_w12 := resize(s_aline, 12);
                when "01"   => v_w12 := resize(px4, 12);
                when "10"   => v_w12 := wpm4;
                when others => v_w12 := wpp4;
            end case;
            v_w12 := shift_right(v_w12, s_psh);
            w3_5  <= v_w12(2 downto 0);

            satlo5 <= satlo4;
            ch8_5  <= f_hash_b(ch_a4);
            px5    <= px4;

            --------------------------------------------------------------
            -- E6: capture context aligned with the bank read data; grain
            -- final select (strokes on paint, canvas tooth on bare paper)
            --------------------------------------------------------------
            a_rbank6 <= not s_par;
            parp(6)  <= s_par;
            wep(6)   <= s_avid_sr(4);
            addrp(6) <= px5;
            bgp(6)   <= bg5;
            if s_aline = 0 then
                ln0p(6) <= '1';
            else
                ln0p(6) <= '0';
            end if;
            lvY(6) <= unsigned(s_y_sr(4));
            lvU(6) <= unsigned(s_u_sr(4));
            lvV(6) <= unsigned(s_v_sr(4));
            wsl6   <= wsl5;

            v_t2 := w3_5(1 downto 0)
                    xor (w3_5(2) & w3_5(2));                   -- 0..3 triangle
            v_gr := resize(signed('0' & v_t2 & '0'), 6) - 3;   -- -3,-1,1,3
            if s_grain_en = '0' then
                gr6 <= (others => '0');
            elsif satlo5 = '1' then
                v_cn := resize(signed('0' & ch8_5(7 downto 4)), 6) - 8;
                if s_grsh >= 2 then
                    gr6 <= v_cn;                               -- +/-8 tooth
                else
                    gr6 <= shift_right(v_cn, 1);               -- +/-4
                end if;
            else
                gr6 <= shift_left(v_gr, s_grsh);
            end if;

            --------------------------------------------------------------
            -- E7: select previous-line bank, diff = prev - live, carried
            -- pigment magnitudes
            --------------------------------------------------------------
            if a_rbank6 = '1' then
                v_pvY := unsigned(s_rdY1);
                v_pvU := unsigned(s_rdU1);
                v_pvV := unsigned(s_rdV1);
            else
                v_pvY := unsigned(s_rdY0);
                v_pvU := unsigned(s_rdU0);
                v_pvV := unsigned(s_rdV0);
            end if;
            dY7 <= signed(resize(v_pvY, 11)) - signed(resize(lvY(6), 11));
            dU7 <= signed(resize(v_pvU, 11)) - signed(resize(lvU(6), 11));
            dV7 <= signed(resize(v_pvV, 11)) - signed(resize(lvV(6), 11));

            -- carried pigment magnitude ~ 1.5 * max(|cu|,|cv|) (abs + one
            -- shift-add; the exact max+min/2 chain was the routed
            -- critical path)
            v_pu8 := signed((not v_pvU(9)) & v_pvU(8 downto 2));
            v_pv8 := signed((not v_pvV(9)) & v_pvV(8 downto 2));
            v_ab7 := f_abs7(v_pu8);
            ap_u7 <= resize(v_ab7, 8) + resize(v_ab7(6 downto 1), 8);
            v_ab7 := f_abs7(v_pv8);
            ap_v7 <= resize(v_ab7, 8) + resize(v_ab7(6 downto 1), 8);

            wsl7 <= wsl6;

            --------------------------------------------------------------
            -- E8: the paint carries its own solvent -- wetness is the max
            -- of the (pre-thresholded) live source and the ARRIVING
            -- paint's saturation, with the Absorb threshold and dry-out
            -- handicap folded into per-frame constants
            --------------------------------------------------------------
            if ap_u7 > ap_v7 then
                v_sp := ap_u7;
            else
                v_sp := ap_v7;
            end if;
            if s_ink = '1' then
                v_spe := s_cink10 - signed(resize(v_sp, 10));
            else
                v_spe := signed(resize(v_sp, 10)) - s_cpnt10;
            end if;
            if v_spe < 0 then
                v_spe := (others => '0');
            end if;
            if unsigned(v_spe(7 downto 0)) > wsl7 then
                wet8 <= unsigned(v_spe(7 downto 0));
            else
                wet8 <= wsl7;
            end if;

            dY8 <= dY7;  dU8 <= dU7;  dV8 <= dV7;

            --------------------------------------------------------------
            -- E9: wetness gain multiply (alone in its stage)
            --------------------------------------------------------------
            tp9 <= wet8 * s_gwet8;
            dY9 <= dY8;  dU9 <= dU8;  dV9 <= dV8;

            --------------------------------------------------------------
            -- E10: t clamp to the Flow ceiling; dry gates (background
            -- read / first line of field); Residue write-back bias
            --------------------------------------------------------------
            v_t12 := tp9(15 downto 4);
            if v_t12 > resize(s_tmax10, 12) then
                v_tt := s_tmax10;
            else
                v_tt := resize(v_t12, 10);
            end if;
            if bgp(9) = '1' or ln0p(9) = '1' then
                v_tt := (others => '0');
            end if;
            t10 <= v_tt;
            if s_stain = '1' then
                bias10 <= -signed('0' & v_tt(9 downto 8));
            else
                bias10 <= signed('0' & v_tt(9 downto 8));
            end if;
            dY10 <= dY9;  dU10 <= dU9;  dV10 <= dV9;

            --------------------------------------------------------------
            -- E11: the recursive lerp multiplies, alone in their stage,
            -- t cut to its top 8 bits (mercurial LC squeeze: same curve,
            -- smaller multiplier, less routing pressure); the Residue
            -- bias pre-folds into the live Y term so the write-back sum
            -- stays two-term (shearline shape)
            --------------------------------------------------------------
            prY11 <= signed('0' & t10(9 downto 2)) * dY10;
            prU11 <= signed('0' & t10(9 downto 2)) * dU10;
            prV11 <= signed('0' & t10(9 downto 2)) * dV10;
            lvYb11 <= signed(resize(lvY(10), 12)) + resize(bias10, 12);

            --------------------------------------------------------------
            -- E12: register the blend result for the display path (the
            -- bank write-back happens in p_lb* off the same s_res_*)
            --------------------------------------------------------------
            wet12_y <= unsigned(s_resY);
            wet12_u <= unsigned(s_resU);
            wet12_v <= unsigned(s_resV);

            -- context / live / grain pipes
            for i in 7 to 11 loop
                parp(i)  <= parp(i - 1);
                wep(i)   <= wep(i - 1);
                addrp(i) <= addrp(i - 1);
                lvY(i)   <= lvY(i - 1);
                lvU(i)   <= lvU(i - 1);
                lvV(i)   <= lvV(i - 1);
            end loop;
            bgp(7 to 9)  <= bgp(6 to 8);
            ln0p(7 to 9) <= ln0p(6 to 8);
            gr7  <= gr6;   gr8  <= gr7;   gr9  <= gr8;
            gr10 <= gr9;   gr11 <= gr10;  gr12 <= gr11;

            --------------------------------------------------------------
            -- E13: grain apply (display only, never fed back) + pigment
            -- chroma boost (shift-add knees, lagoon lesson: no knob mult)
            --------------------------------------------------------------
            y13 <= f_cu10(signed(resize(wet12_y, 12))
                          + resize(gr12, 12));

            v_cur := signed((not wet12_u(9)) & wet12_u(8 downto 0));
            v_cvr := signed((not wet12_v(9)) & wet12_v(8 downto 0));
            case s_pigsel is
                when "00"   => v_ext := (others => '0');
                when "01"   => v_ext := shift_right(v_cur, 2);
                when "10"   => v_ext := shift_right(v_cur, 1);
                when others => v_ext := v_cur;
            end case;
            u13 <= f_cu10(signed(resize(wet12_u, 12)) + resize(v_ext, 12));
            case s_pigsel is
                when "00"   => v_ext := (others => '0');
                when "01"   => v_ext := shift_right(v_cvr, 2);
                when "10"   => v_ext := shift_right(v_cvr, 1);
                when others => v_ext := v_cvr;
            end case;
            v13 <= f_cu10(signed(resize(wet12_v, 12)) + resize(v_ext, 12));

            --------------------------------------------------------------
            -- E14: lanes fully assembled one stage before the mux; drip
            -- color reconstructed from the carried quantized chroma
            --------------------------------------------------------------
            ln_vy <= y13;
            ln_vu <= u13;
            ln_vv <= v13;
            ln_ty <= ('0' & y13(9 downto 1)) + to_unsigned(256, 10);
            ln_du <= (cu3p(13) & "0000000") + to_unsigned(64, 10);
            ln_dv <= (cv3p(13) & "0000000") + to_unsigned(64, 10);
            if fl_h(13) = '1' then
                sel14 <= "10";
            elsif fl_t(13) = '1' then
                sel14 <= "01";
            else
                sel14 <= "00";
            end if;

            --------------------------------------------------------------
            -- E15: output mux
            --------------------------------------------------------------
            case sel14 is
                when "10" =>                        -- drip head
                    s_out_y <= to_unsigned(720, 10);
                    s_out_u <= ln_du;
                    s_out_v <= ln_dv;
                when "01" =>                        -- drip tail
                    s_out_y <= ln_ty;
                    s_out_u <= ln_du;
                    s_out_v <= ln_dv;
                when others =>
                    s_out_y <= ln_vy;
                    s_out_u <= ln_vu;
                    s_out_v <= ln_vv;
            end case;

            --------------------------------------------------------------
            -- line / frame bookkeeping
            --------------------------------------------------------------
            if s_lstep = 1 then
                s_lrand5 <= s_lfsr(4 downto 0);
                s_lstep  <= "10";
            elsif s_lstep = 2 then
                if s_jamt = 0 then
                    s_tj5 <= (others => '0');
                else
                    s_tj5 <= resize(shift_right(
                        signed('0' & s_lrand5) - 16, 4 - s_jamt), 5);
                end if;
                s_lstep <= "00";
            end if;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_x_count > 0 then
                    s_lwidth <= s_x_count;
                    s_wmax   <= s_x_count - 1;
                    s_wedge  <= s_x_count - 40;
                end if;
                s_x_count <= (others => '0');
                s_par     <= not s_par;
                s_lstep   <= "01";
                pf_seq    <= "01";

                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 1000 then
                        s_aline <= s_aline + 1;
                    end if;
                else
                    if s_aline /= 0 then
                        s_nrows <= s_aline;        -- measured field height
                    end if;
                    s_aline <= (others => '0');
                end if;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_par   <= '0';
                s_aline <= (others => '0');
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- blend result (combinational off the E11 regs): the recursive lerp
    -- result = clamp( live + t*(prev-live) >> 10 + residue bias ). Written
    -- back to the banks AND registered for the display path -- what feeds
    -- back is exactly what is shown (before grain/pigment/drips).
    ------------------------------------------------------------------------
    p_result : process(prY11, prU11, prV11, lvYb11, lvU, lvV)
        variable v_sumY, v_sumU, v_sumV : signed(12 downto 0);
    begin
        v_sumY := resize(lvYb11, 13)
                  + resize(prY11(19 downto 8), 13);
        v_sumU := signed(resize(lvU(11), 13))
                  + resize(prU11(19 downto 8), 13);
        v_sumV := signed(resize(lvV(11), 13))
                  + resize(prV11(19 downto 8), 13);

        if v_sumY < 0 then
            s_resY <= (others => '0');
        elsif v_sumY > 1023 then
            s_resY <= std_logic_vector(to_unsigned(1023, 10));
        else
            s_resY <= std_logic_vector(v_sumY(9 downto 0));
        end if;
        if v_sumU < 0 then
            s_resU <= (others => '0');
        elsif v_sumU > 1023 then
            s_resU <= std_logic_vector(to_unsigned(1023, 10));
        else
            s_resU <= std_logic_vector(v_sumU(9 downto 0));
        end if;
        if v_sumV < 0 then
            s_resV <= (others => '0');
        elsif v_sumV > 1023 then
            s_resV <= std_logic_vector(to_unsigned(1023, 10));
        else
            s_resV <= std_logic_vector(v_sumV(9 downto 0));
        end if;
    end process p_result;

    ------------------------------------------------------------------------
    -- feedback banks: one process per bank, canonical 1W1R with a single
    -- pre-decided write (yosys sees two write ports as FFs -- shearline).
    -- Reads use the live parity's opposite bank via the shared address;
    -- writes land the carried parity (trailing writes finish into the old
    -- bank, sidestepping the bank-flip race).
    ------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(rd5y));
            if wep(11) = '1' and parp(11) = '0' then
                lbY0(to_integer(addrp(11))) <= s_resY;
            end if;
        end if;
    end process p_lbY0;

    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(rd5y));
            if wep(11) = '1' and parp(11) = '1' then
                lbY1(to_integer(addrp(11))) <= s_resY;
            end if;
        end if;
    end process p_lbY1;

    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(rd5y(10 downto 1)));
            if wep(11) = '1' and parp(11) = '0' then
                lbU0(to_integer(addrp(11)(10 downto 1))) <= s_resU;
            end if;
        end if;
    end process p_lbU0;

    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(rd5y(10 downto 1)));
            if wep(11) = '1' and parp(11) = '1' then
                lbU1(to_integer(addrp(11)(10 downto 1))) <= s_resU;
            end if;
        end if;
    end process p_lbU1;

    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(rd5y(10 downto 1)));
            if wep(11) = '1' and parp(11) = '0' then
                lbV0(to_integer(addrp(11)(10 downto 1))) <= s_resV;
            end if;
        end if;
    end process p_lbV0;

    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(rd5y(10 downto 1)));
            if wep(11) = '1' and parp(11) = '1' then
                lbV1(to_integer(addrp(11)(10 downto 1))) <= s_resV;
            end if;
        end if;
    end process p_lbV1;

    ------------------------------------------------------------------------
    -- drip updater: vblank column walk, one column per 7-state pass.
    -- Land the BRAM reads in plain FFs, THEN decide (redshift lesson:
    -- the single-state USE version was the routed critical path).
    ------------------------------------------------------------------------
    p_part : process(clk)
        variable v_fb : std_logic;
        variable v_np : unsigned(9 downto 0);
        variable v_u3, v_v3 : unsigned(2 downto 0);
        variable v_bs : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- free-running 16-bit Fibonacci LFSR
            v_fb := s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10);
            s_lfsr <= s_lfsr(14 downto 0) & v_fb;

            pu_prev_vsync_n <= data_in.vsync_n;
            pu_we  <= '0';
            pu_swe <= '0';
            case pu_state is
                when PU_IDLE =>
                    if data_in.vsync_n = '0' and pu_prev_vsync_n = '1' then
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
                    pl_pos <= unsigned(part_q(15 downto 6));
                    pl_u3  <= unsigned(part_q(5 downto 3));
                    pl_v3  <= unsigned(part_q(2 downto 0));
                    ps_bs  <= unsigned(surf_q(15 downto 6));
                    ps_u3  <= unsigned(surf_q(5 downto 3));
                    ps_v3  <= unsigned(surf_q(2 downto 0));
                    pu_state <= PU_CALC;
                when PU_CALC =>
                    -- fall 2..5 rows/field; run ends 176 rows below the
                    -- surface (or at the bottom); respawn density gate
                    pl_np <= pl_pos + 2
                             + resize(unsigned(s_lfsr(1 downto 0)), 10);
                    if ps_bs > 830 then
                        pl_end <= to_unsigned(1006, 10);
                    else
                        pl_end <= ps_bs + 176;
                    end if;
                    if unsigned(s_lfsr(9 downto 2)) < s_gdrip8 then
                        pl_rgate <= '1';
                    else
                        pl_rgate <= '0';
                    end if;
                    pu_state <= PU_DECIDE;
                when PU_DECIDE =>
                    v_np := pl_np;
                    v_u3 := pl_u3;
                    v_v3 := pl_v3;
                    if pl_pos >= 1000 or pl_np > pl_end
                       or pl_np >= s_nrows then
                        if pl_rgate = '1' and ps_bs < s_nrows then
                            v_np := ps_bs;      -- respawn AT the surface,
                            v_u3 := ps_u3;      -- carrying its pigment
                            v_v3 := ps_v3;
                        else
                            v_np := (others => '1');    -- park
                        end if;
                    end if;
                    -- surface drift-down (re-adapt to a moving picture)
                    if ps_bs /= "1111111111" then
                        v_bs := ps_bs + 1;
                    else
                        v_bs := ps_bs;
                    end if;

                    pu_waddr  <= pu_cnt;
                    pu_wdata  <= std_logic_vector(v_np)
                                 & std_logic_vector(v_u3)
                                 & std_logic_vector(v_v3);
                    pu_swdata <= std_logic_vector(v_bs)
                                 & std_logic_vector(ps_u3)
                                 & std_logic_vector(ps_v3);
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
    -- drip memories: read ports muxed updater (vblank) / render prefetch
    -- (active); surf write port muxed updater / raster detect -- all
    -- pre-muxed single ports, port-exclusive by construction (redshift)
    ------------------------------------------------------------------------
    part_raddr <= pu_cnt when pu_busy = '1' else prf_col;
    surf_raddr <= pu_cnt when pu_busy = '1' else prf_col;
    surf_waddr <= pu_waddr  when pu_swe = '1' else sw_addr;
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

end architecture turpentine;
