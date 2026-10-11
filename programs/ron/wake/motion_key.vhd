-- motion_key.vhd - motion matte engine: motion_key_pkg + entity motion_key
-- Copyright (C) 2026 bluecondition
-- License: GNU General Public License v3.0
--
-- One self-contained file: drop it into a program folder and use
-- work.motion_key_pkg.all. Package constants mirror
-- programs/ron/wake/model/wake_model.py (the golden model); change both together.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package motion_key_pkg is

    -- key/matte are valid this many clocks after the render sync input
    constant C_MK_RENDER_LATENCY : integer := 7;

    -- analysis-side controls (sample them straight from registers_in)
    type t_mk_analysis_params is record
        wake        : unsigned(9 downto 0);  -- matte release time, exponential taper
        floor       : unsigned(9 downto 0);  -- noise coring on the cell difference
        chroma      : unsigned(9 downto 0);  -- chroma weight, 512 = luma and chroma equal
        mode        : std_logic;             -- 0 Motion, 1 Presence (Phase 4)
        plate       : std_logic;             -- 0 Live, 1 Hold (Phase 4)
        steady      : std_logic;             -- pan/fade compensation + cut guard
        both_fields : std_logic;             -- interlace: 1 = compare every field with the
                                             -- previous one; 0 = top fields only (default)
    end record;

    -- render-side controls
    type t_mk_render_params is record
        sens     : unsigned(9 downto 0);     -- STIR: 0 = nothing keyed .. 1023 = faintest motion
        softness : unsigned(9 downto 0);     -- <128 = nearest cell + hard key, then softer
        detail   : unsigned(9 downto 0);     -- edge gate: 0 = off .. 1023 = edges/texture only
        invert   : std_logic;                -- key = 1023 - key
        follow   : std_logic;                -- follow-spot instead of matte (Phase 4)
    end record;

    -- per-update statistics, latched at the end of each update
    type t_mk_stats is record
        energy : unsigned(21 downto 0);      -- sum of raw cell differences d (1/4 LSB)
        valid  : std_logic;                  -- engine past warm-up, matte meaningful
        cut    : std_logic;                  -- cut guard tripped: key muted until re-seeded
        width  : unsigned(11 downto 0);      -- measured active width (pixels)
        height : unsigned(10 downto 0);      -- measured active lines per update
    end record;

    type t_int16 is array (0 to 15) of integer;

    -- round(16 * log2(update rate)) by timing ID (fields when interlaced)
    constant C_RATE_OFF16 : t_int16 := (
        94, 90, 94, 73, 94, 90, 94, 79, 90, 73, 95, 74, 90, 78, 95, 94);
    -- release mantissa round(32 * 2^(-i/16)), threshold mantissa round(32 * 2^(i/16))
    constant C_MANT_DN : t_int16 := (32, 31, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 17);
    constant C_MANT_UP : t_int16 := (32, 33, 35, 36, 38, 40, 41, 43, 45, 47, 49, 52, 54, 56, 59, 61);
    constant C_WAKE_BIAS : integer := 95;
    -- v3 tuning (model/tune/tune.py on footage, 2026-10-11)
    constant C_CUT_THR      : integer := 224;  -- running mean d (1/4 LSB) above = scene cut
    constant C_DETAIL_GAIN  : integer := 3;    -- edge energy = |Y[x]-Y[x-2]| << 3
    constant C_DETAIL_DECAY : integer := 64;   -- edge energy spread: -64 per pixel (rightward)

    function f_ceil_log2(n : integer) return integer;
    function f_is_sd(tid : std_logic_vector(3 downto 0)) return boolean;
    function f_is_interlaced(tid : std_logic_vector(3 downto 0)) return boolean;

end package motion_key_pkg;

package body motion_key_pkg is

    function f_ceil_log2(n : integer) return integer is
        variable v : integer := 0;
    begin
        while 2 ** v < n loop
            v := v + 1;
        end loop;
        return v;
    end function;

    -- NTSC 0, PAL 8, 480p 4, 576p 12
    function f_is_sd(tid : std_logic_vector(3 downto 0)) return boolean is
    begin
        return tid = "0000" or tid = "1000" or tid = "0100" or tid = "1100";
    end function;

    -- NTSC 0, PAL 8, 1080i60 10, 1080i59.94 2, 1080i50 1
    function f_is_interlaced(tid : std_logic_vector(3 downto 0)) return boolean is
    begin
        return tid = "0000" or tid = "1000" or tid = "1010" or tid = "0010" or tid = "0001";
    end function;

end package body motion_key_pkg;

--------------------------------------------------------------------------------
-- motion_key - motion matte engine (reusable; WAKE is its first host)
--------------------------------------------------------------------------------
-- WHAT IT DOES
--   Keys on what MOVES instead of what is bright. There is no frame store, so
--   the engine keeps a thumbnail of the picture: per cell of a C x R grid
--   (C = 32 HD / 24 SD, R = 18) four luma sub-cell averages (2 x 2) and the U/V
--   averages, at 1/4-LSB precision. Each update (frame, or one field parity when
--   interlaced) compares them with a short-memory reference (ref += (cur-ref)/4),
--   so continuous motion builds a difference while grain averages out. The
--   per-cell difference drives a peak-hold motion grid M with a WAKE release.
--   The grid is rendered back to a smooth full-raster matte (bilinear between
--   cell centres, multiply-free per pixel), shaped into a key, and optionally
--   gated by local edge energy (DETAIL).
--   Golden model: model/wake_model.py MotionKey3 (bit-exact for the grid).
--   v3 (2026-10-11): on real footage v2 keyed 0-24% of human motion; v3 keys
--   50-95% at ~0 false keying (model/tune).
--
-- PORTS
--   ana_in + timing_id + ana_params  analysis tap (the host's INPUT stream)
--   rnd_avid/rnd_vsync_n/rnd_y       render sync + luma from the host's stage D - L
--   key, matte                       valid at render input + L (L = 7, see pkg)
--                                    matte = unshaped 10-bit interpolated grid
--   stats                            per-update energy, width/height, valid, cut
--   dbg                              finalizer write ports (testbench only;
--                                    leave open in a host and it is pruned)
--   Hosts align the key to their mix stage D by feeding the render sync from
--   stage D - L. The matte depends only on position and the grid, so the
--   renderer can sit at any pipeline stage.
--
-- ANALYSIS (side tap; latency irrelevant)
--   A0  register input, 8-bit samples (10-bit >> 2)
--   A1  pixel counter, half-column Bresenham hc = floor(x*2C/W) (col = hc>>1),
--       half-row state hr = floor(y*2R/H) (row = hr>>1), edges, flags
--   A2  test-object substitution (G_TEST_OBJECT), per-line flags
--   A3  per-line partial sums: luma per half-column, U/V per column; emit at a
--       boundary or line end, >> sh, into the column accumulator RAM
--   RMW pipeline (up to 3 ops per emit - Y, U, V - on consecutive clocks):
--       issue -> bank addr reg -> EBR read -> fabric reg -> add -> write
--   Accumulator banks ACC0/ACC1 hold even/odd cell rows: luma at
--   (sub-row << 6) | hc (128 words), U/V at 128 + 2*col + ch (64 words).
--   One bank accumulates the current row (read-modify-write) while the
--   finalizer reads the other. The first line of a half-row (luma) / row
--   (chroma) treats the old value as 0.
--   Geometry: W/H measured from avid (update line 2's width, active line
--   count), +-2 hysteresis, latched at the end of each update field. A vblank
--   sequencer (serial divider + multiplier) derives cw_max = ceil(W/C),
--   ch_max = ceil(H/R), sh = max(0, bitlen(cw_max*ch_max) - 8),
--   recip = round(2^(16+sh)*C*R/(W*H)), recip_c = round(2^16*C/W),
--   recip_r = round(2^16*R/H).
--   Warm-up: after load, a timing-ID change, any geometry change or a STEADY
--   cut, the next update that finalizes ALL rows seeds (ref = cur, M = 0) -
--   generation-tagged, so a late seed from before a change cannot clear the
--   new request. No flash.
--   Interlace: with both_fields = 0 only one field parity updates (same-parity
--   compares, half rate). SD: decoder field_n + timing ID. HD: the core ties
--   field_n high, so interlace is the timing ID OR the SDK sync-phase detector
--   OR a field_n that really toggles (image tester), and without a live field_n
--   a per-field toggle picks every other field (v0.2.2: 1080i over HDMI compared
--   opposite fields -> edge-twitter blobs on a frozen image, HW-reported).
--
-- FINALIZER (one cell row at a time, during the next row / vblank; ~170 clk
--            per cell on one serial multiplier, 6 values per cell in sequence)
--   per value v (Y00 Y01 Y10 Y11 U V):
--     cur = min(1023, (A*recip + rnd) >> (12 luma sub-cell | 14 chroma))  1/4 LSB
--     diff = |cur - ((ref + 2) >> 2)|          ref is 12-bit, 1/16 LSB
--     ref  = seed ? cur << 2 : ref + (((cur << 2) - ref) >>> 2)
--   d   = (sum diff_Y >> 1) + ((diff_U + diff_V) * CHROMA) >> 9
--   STEADY: dc = max(0, d - g_prev), g_prev = ((sum d >> 4) * recipN) >> 14
--           (recipN = round(2^18 / cells)); cut = running sum d > 224*C*(row+1)
--           at any row >= 2 -> mute key + re-seed
--   dp  = min(255, max(0, dc - (FLOOR >> 4)))                    M int units
--   e16 = clamp((((WAKE >> 3)*3) >> 2) - 95 + rate16[tid] (-16 one-field), 0, 255)
--   dec = M ? max(1, ((M >> e16[7:4]) * MANT_DN[e16[3:0]]) >> 5) : 0
--   M   = max(dp << 4, M - dec)          M is 12-bit 8.4; seed: M = 0
--
-- RENDER (per pixel; L = 7)
--   R1  half-column Bresenham hc = floor(x*2C/W); segment s runs from the
--       centre of col s to col s+1 (hc 2s+1..2s+2); at a segment start load
--       DDA acc = (V_s << 8) + 128, else acc += S_s.  DETAIL: |Y[x] - Y[x-2]|
--   R2  m = clamp(acc >> 8)   (nearest: hc odd ? V_s : V_s+1); edge energy
--       e = max(min(1023, g << 3), e - 64)
--   R3  m - thr, floored at 0;  gate = min(1023, e + (1023 - DETAIL))
--   R4  << k (gain from SOFTNESS), saturation detect
--   R5  + half step (x1.5), clamp 1023                    -> raw key at R6
--   R6  min(raw key, gate); cut mute; invert              -> key at R7
--   Column processor (one column ahead, 14 clk from trigger, 2 M reads):
--   V_k = M10[r0][k] + (((M10[r1][k] - M10[r0][k]) * fy + 128) >> 8),
--   S_(k-1) = ((V_k - V_(k-1)) * recip_c) >> 8, on one 11x16 two-stage
--   multiplier. Per line, r0/r1/fy come from a half-row Bresenham, and the fy
--   DDA (jy * recip_r) >> 8 runs in hblank (V_0, V_1, S_0 before the line).
--   Minimum cell width ~15 px (the 1080p decimation-4 sim is exactly 15).
--   thr  = min(1024, (MANT_UP[e & 15] << (e >> 4)) >> 3),
--          e = ((1023 - sens) * 17) >> 7      (8 octaves; bottom ~6% = off)
--   gain = SOFTNESS < 128 ? 256 (nearest, hard) : 2^(kq >> 1) * (kq odd ? 1.5 : 1),
--          kq = 16 - ((SOFTNESS - 128) >> 6)
--
-- MEMORY (18 EBR, all inferred 1W1R, no_rw_check)
--   ACC0, ACC1   256 x 16   column accumulators, even/odd cell rows     2 EBR
--   REF          4096 x 12  reference, addr = cell*6 + v (3456 used)  12 EBR
--   MGRID        1024 x 12  motion grid, addr = row*32 + col (576)      3 EBR
--   (+1 EBR for the 4096-deep cascade decode when yosys splits it)
--   MGRID read port: the renderer has fixed priority (2 reads per column);
--   the finalizer's one read per cell takes a free cycle. Single-buffered
--   in-place update; rows mix updates N and N-1 for one frame (accepted).
--   REF/MGRID writes: finalizer only; MGRID writes wait while the renderer reads.
--
-- RESOURCES: see programs/ron/wake/PROGRESS.md (measured per build).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_stream_pkg.all;
use work.motion_key_pkg.all;

entity motion_key is
    generic (
        G_TEST_OBJECT : boolean := false;  -- bright square in the ANALYSIS path only
        G_COLS_HD     : integer := 32;
        G_COLS_SD     : integer := 24;
        G_ROWS        : integer := 18
    );
    port (
        clk         : in  std_logic;
        ana_in      : in  t_video_stream_yuv444_30b;
        timing_id   : in  std_logic_vector(3 downto 0);
        ana_params  : in  t_mk_analysis_params;
        rnd_avid    : in  std_logic;
        rnd_vsync_n : in  std_logic;
        rnd_y       : in  std_logic_vector(9 downto 0);  -- DETAIL (Phase 3)
        rnd_params  : in  t_mk_render_params;
        key         : out unsigned(9 downto 0);
        matte       : out unsigned(9 downto 0);
        stats       : out t_mk_stats;
        dbg_m_we    : out std_logic;                     -- testbench taps
        dbg_m_addr  : out unsigned(9 downto 0);
        dbg_m_data  : out unsigned(11 downto 0);
        dbg_r_we    : out std_logic;
        dbg_r_addr  : out unsigned(11 downto 0);
        dbg_r_data  : out unsigned(11 downto 0);
        dbg_upd     : out std_logic                      -- pulse: last row finalized
    );
end entity motion_key;

architecture rtl of motion_key is

    constant C_COLB  : integer := f_ceil_log2(G_COLS_HD);
    constant C_ROWB  : integer := f_ceil_log2(G_ROWS);
    constant C_CELLB : integer := C_ROWB + C_COLB;
    constant C_REFB  : integer := f_ceil_log2(6 * ((G_ROWS - 1) * 2 ** C_COLB + G_COLS_HD));
    constant C_ACCB  : integer := maximum(8, C_COLB + 3);
    constant C_R     : integer := G_ROWS;
    constant C_HCB   : integer := C_COLB + 1;              -- half-column index bits
    constant C_TC_HD : integer := C_CUT_THR * G_COLS_HD;   -- cut threshold per row
    constant C_TC_SD : integer := C_CUT_THR * G_COLS_SD;
    constant C_RN_HD : integer := (2 ** 18 + (G_ROWS * G_COLS_HD) / 2) / (G_ROWS * G_COLS_HD);
    constant C_RN_SD : integer := (2 ** 18 + (G_ROWS * G_COLS_SD) / 2) / (G_ROWS * G_COLS_SD);

    --------------------------------------------------------------------------
    -- memories
    --------------------------------------------------------------------------
    type t_acc_ram is array (0 to 2 ** C_ACCB - 1) of unsigned(15 downto 0);
    type t_ref_ram is array (0 to 2 ** C_REFB - 1) of unsigned(11 downto 0);
    type t_m_ram   is array (0 to 2 ** C_CELLB - 1) of unsigned(11 downto 0);
    signal r_acc0 : t_acc_ram := (others => (others => '0'));
    signal r_acc1 : t_acc_ram := (others => (others => '0'));
    signal r_ref  : t_ref_ram := (others => (others => '0'));
    signal r_m    : t_m_ram   := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_acc0 : signal is true;
    attribute no_rw_check of r_acc1 : signal is true;
    attribute no_rw_check of r_ref  : signal is true;
    attribute no_rw_check of r_m    : signal is true;

    signal acc0_ra, acc1_ra : unsigned(C_ACCB - 1 downto 0) := (others => '0');
    signal acc0_rd, acc1_rd : unsigned(15 downto 0) := (others => '0');
    signal acc0_rq, acc1_rq : unsigned(15 downto 0) := (others => '0');
    signal acc_we0, acc_we1 : std_logic := '0';
    signal acc_wa : unsigned(C_ACCB - 1 downto 0) := (others => '0');
    signal acc_wd : unsigned(15 downto 0) := (others => '0');

    signal ref_ra : unsigned(C_REFB - 1 downto 0) := (others => '0');
    signal ref_rd, ref_rq, ref_rqq : unsigned(11 downto 0) := (others => '0');
    signal ref_we : std_logic := '0';
    signal ref_wa : unsigned(C_REFB - 1 downto 0) := (others => '0');
    signal ref_wd : unsigned(11 downto 0) := (others => '0');

    signal m_ra   : unsigned(C_CELLB - 1 downto 0) := (others => '0');
    signal m_rd, m_rq : unsigned(11 downto 0) := (others => '0');
    signal m_tag1, m_tag2, m_tag3 : std_logic := '0';   -- '1' = finalizer read
    signal m_we   : std_logic := '0';
    signal m_wa   : unsigned(C_CELLB - 1 downto 0) := (others => '0');
    signal m_wd   : unsigned(11 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- committed geometry (vblank sequencer)
    --------------------------------------------------------------------------
    signal g_ok    : std_logic := '0';
    signal g_w     : unsigned(11 downto 0) := (others => '0');
    signal g_h     : unsigned(10 downto 0) := (others => '0');
    signal g_c     : unsigned(5 downto 0) := to_unsigned(G_COLS_HD, 6);
    signal g_sh    : unsigned(2 downto 0) := (others => '0');
    signal g_recip : unsigned(11 downto 0) := (others => '0');
    signal g_rc    : unsigned(15 downto 0) := (others => '0');
    signal g_rr    : unsigned(15 downto 0) := (others => '0');
    signal geo_busy : std_logic := '0';

    --------------------------------------------------------------------------
    -- control parameters (registered, quasi-static)
    --------------------------------------------------------------------------
    signal tid_q      : std_logic_vector(3 downto 0) := (others => '0');
    signal det_fld_n  : std_logic := '1';    -- SDK field detector (HD: core ties field_n high)
    signal det_il     : std_logic := '0';
    signal fn_prev    : std_logic := '1';    -- input field_n of the previous field
    signal fn_tog     : unsigned(1 downto 0) := (others => '0');  -- recent field_n toggles
    signal fn_live    : std_logic := '0';    -- input field_n really alternates
    signal fld_t      : std_logic := '0';    -- toggles every field: same-parity cadence
    signal one_field  : std_logic := '0';
    signal p_f4       : unsigned(5 downto 0) := (others => '0');    -- FLOOR >> 4 (1/4 LSB)
    signal p_steady   : std_logic := '0';
    signal p_dinv     : unsigned(9 downto 0) := (others => '1');    -- 1023 - DETAIL
    signal p_chroma   : unsigned(9 downto 0) := (others => '0');
    signal p_ew       : unsigned(6 downto 0) := (others => '0');
    signal p_rate     : unsigned(6 downto 0) := (others => '0');
    signal p_eint     : unsigned(3 downto 0) := (others => '0');
    signal p_mant     : unsigned(5 downto 0) := (others => '0');   -- MANT_DN(0) = 32
    signal p_te       : unsigned(6 downto 0) := (others => '0');
    signal p_thr      : unsigned(10 downto 0) := (others => '0');
    signal p_near     : std_logic := '0';
    signal p_k        : unsigned(3 downto 0) := (others => '0');
    signal p_half     : std_logic := '0';
    signal p_inv      : std_logic := '0';

    --------------------------------------------------------------------------
    -- analysis front end
    --------------------------------------------------------------------------
    signal a0_y, a0_u, a0_v : unsigned(7 downto 0) := (others => '0');
    signal a0_avid  : std_logic := '0';
    signal a0_vs_n  : std_logic := '1';
    signal a0_fld   : std_logic := '0';
    signal a1_avq   : std_logic := '0';
    signal a1_vsq   : std_logic := '1';
    signal saw_act  : std_logic := '0';
    signal xc       : unsigned(11 downto 0) := (others => '0');
    signal y_line   : unsigned(10 downto 0) := (others => '0');
    signal upd_line : std_logic := '0';
    signal fld_seen : std_logic := '0';
    signal hm_live  : unsigned(10 downto 0) := (others => '0');
    signal wm_live  : unsigned(11 downto 0) := (others => '0');
    signal hm       : unsigned(10 downto 0) := (others => '0');
    signal wm       : unsigned(11 downto 0) := (others => '0');
    signal upd_end  : std_logic := '0';
    -- seeding: set on load / timing-ID change / geometry change (new generation);
    -- cleared only when an update seeded ALL its rows under the current generation
    signal seed_pend : std_logic := '1';
    signal seed_gen  : unsigned(1 downto 0) := (others => '0');
    signal geo_chg   : std_logic := '0';
    signal seed_done : std_logic := '0';
    signal done_gen  : unsigned(1 downto 0) := (others => '0');

    -- half-row Bresenham (hr = floor(y*2R/H)); fresh: first line of a half-row
    -- (luma) / of a row (chroma)
    signal ar_cur, ar_nxt : unsigned(C_ROWB downto 0) := (others => '0');
    signal ar_acc   : unsigned(10 downto 0) := (others => '0');
    signal ar_fy, ar_fc : std_logic := '1';
    signal cut_evt  : std_logic := '0';
    signal mute     : std_logic := '0';

    signal bx_acc   : unsigned(11 downto 0) := (others => '0');
    signal bx_hc    : unsigned(C_HCB - 1 downto 0) := (others => '0');

    signal ob_x, ob_x2 : unsigned(11 downto 0) := (others => '0');
    signal ob_y, ob_y2 : unsigned(10 downto 0) := (others => '0');
    signal ob_dir   : std_logic := '0';
    signal ob_h, ob_v : std_logic := '0';

    signal a2_y, a2_u, a2_v : unsigned(7 downto 0) := (others => '0');
    signal a2_hc    : unsigned(C_HCB - 1 downto 0) := (others => '0');
    signal a2_act, a2_first, a2_end, a2_last, a2_tob : std_logic := '0';
    signal a2_fy, a2_fc, a2_bank, a2_sub : std_logic := '0';
    signal a2_row   : unsigned(C_ROWB - 1 downto 0) := (others => '0');

    -- per-line partial sums: luma over the current half-column, U/V over the column
    signal py, pu, pv : unsigned(14 downto 0) := (others => '0');
    signal ps_hc    : unsigned(C_HCB - 1 downto 0) := (others => '0');
    -- emissions (luma slot 0, chroma slots 1/2)
    signal ey_go, ec_go, ey_fresh, ec_fresh, e_bank : std_logic := '0';
    signal ey_addr  : unsigned(C_HCB downto 0) := (others => '0');   -- sub-row & hc
    signal ec_col   : unsigned(C_COLB - 1 downto 0) := (others => '0');
    signal ey_p, eu_p, ev_p : unsigned(15 downto 0) := (others => '0');
    signal sv_y, sv_c : std_logic := '0';
    signal iss      : unsigned(1 downto 0) := "11";

    -- RMW tags: stage 1 = bank address registered, 2 = EBR out, 3 = fabric reg
    type t_tag is record
        v     : std_logic;
        addr  : unsigned(C_ACCB - 1 downto 0);
        part  : unsigned(15 downto 0);
        fresh : std_logic;
        bank  : std_logic;
    end record;
    constant C_TAG0 : t_tag := ('0', (others => '0'), (others => '0'), '0', '0');
    signal t1, t2, t3 : t_tag := C_TAG0;

    -- row-done handshake
    signal rd_pend  : std_logic := '0';
    signal rd_row   : unsigned(C_ROWB - 1 downto 0) := (others => '0');
    signal rd_dly   : unsigned(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- finalizer
    --------------------------------------------------------------------------
    type t_fs is (FS_IDLE, FS_VRD, FS_VW1, FS_VW2, FS_VW3, FS_VCAP, FS_VCUR, FS_VDIF,
                  FS_VWR, FS_MW, FS_D0, FS_D1, FS_D2, FS_R0, FS_R1, FS_R2, FS_R3,
                  FS_R4, FS_W0, FS_NEXT, FS_ROW, FS_G1, FS_MUL);
    signal fs, fs_ret : t_fs := FS_IDLE;
    attribute fsm_encoding : string;
    attribute fsm_encoding of fs : signal is "none";
    signal f_row    : unsigned(C_ROWB - 1 downto 0) := (others => '0');
    signal f_col    : unsigned(C_COLB - 1 downto 0) := (others => '0');
    signal f_v      : unsigned(2 downto 0) := (others => '0');   -- 0..3 luma sub, 4 U, 5 V
    signal f_bank   : std_logic := '0';
    signal f_act    : std_logic := '0';
    signal f_seed   : std_logic := '0';
    signal f_gen0   : unsigned(1 downto 0) := (others => '0');
    signal f_allsd  : std_logic := '0';
    signal f_recip  : unsigned(11 downto 0) := (others => '0');
    signal f_cell   : unsigned(C_CELLB - 1 downto 0) := (others => '0');
    signal f_cell6  : unsigned(C_REFB - 1 downto 0) := (others => '0');
    signal f_ra     : unsigned(C_ACCB - 1 downto 0) := (others => '0');
    signal f_mpend  : std_logic := '0';
    signal f_mgot   : std_logic := '0';
    signal f_mold   : unsigned(11 downto 0) := (others => '0');
    signal f_a      : unsigned(15 downto 0) := (others => '0');
    signal f_ref    : unsigned(11 downto 0) := (others => '0');
    signal f_cur    : unsigned(9 downto 0) := (others => '0');
    signal f_dsy    : unsigned(11 downto 0) := (others => '0');  -- sum of 4 luma diffs
    signal f_dsc    : unsigned(10 downto 0) := (others => '0');  -- |dU| + |dV|
    signal f_d      : unsigned(12 downto 0) := (others => '0');
    signal f_dc     : unsigned(12 downto 0) := (others => '0');
    signal f_dp     : unsigned(7 downto 0) := (others => '0');
    signal f_msh    : unsigned(11 downto 0) := (others => '0');
    signal f_dec    : unsigned(11 downto 0) := (others => '0');
    signal f_mrel   : unsigned(11 downto 0) := (others => '0');
    signal f_mnew   : unsigned(11 downto 0) := (others => '0');
    signal f_esum   : unsigned(21 downto 0) := (others => '0');
    signal f_thr    : unsigned(21 downto 0) := (others => '0');
    signal f_cut    : std_logic := '0';                          -- cut seen this update
    signal f_energy : unsigned(21 downto 0) := (others => '0');
    signal f_gprev  : unsigned(12 downto 0) := (others => '0');
    signal f_upd    : std_logic := '0';
    signal f_g_c_hd : std_logic := '1';
    signal mp_a     : unsigned(27 downto 0) := (others => '0');
    signal mp_b     : unsigned(11 downto 0) := (others => '0');
    signal mp_p     : unsigned(27 downto 0) := (others => '0');
    signal mp_n     : unsigned(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- geometry sequencer
    --------------------------------------------------------------------------
    type t_gs is (GS_IDLE, GS_CHK, GS_D1R, GS_D2R, GS_M1R, GS_M2R, GS_D3, GS_D3R,
                  GS_D4R, GS_D5R, GS_DIV, GS_MUL);
    signal gs, gs_ret : t_gs := GS_IDLE;
    attribute fsm_encoding of gs : signal is "none";
    signal q_w      : unsigned(11 downto 0) := (others => '0');
    signal q_h      : unsigned(10 downto 0) := (others => '0');
    signal q_c      : unsigned(5 downto 0) := (others => '0');
    signal q_cw, q_ch : unsigned(6 downto 0) := (others => '0');
    signal q_sh     : unsigned(2 downto 0) := (others => '0');
    signal q_wh     : unsigned(22 downto 0) := (others => '0');
    signal q_recip  : unsigned(11 downto 0) := (others => '0');
    signal q_rc     : unsigned(15 downto 0) := (others => '0');
    signal dv_q     : unsigned(35 downto 0) := (others => '0');
    signal dv_den   : unsigned(22 downto 0) := (others => '0');
    signal dv_rem   : unsigned(23 downto 0) := (others => '0');
    signal dv_n     : unsigned(5 downto 0) := (others => '0');
    signal gm_a     : unsigned(23 downto 0) := (others => '0');
    signal gm_b     : unsigned(11 downto 0) := (others => '0');
    signal gm_p     : unsigned(23 downto 0) := (others => '0');
    signal gm_n     : unsigned(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- renderer
    --------------------------------------------------------------------------
    signal r1_avid, r1_avq : std_logic := '0';
    signal r1_vs, r1_vsq   : std_logic := '1';
    signal r_saw    : std_logic := '0';
    signal r_pre0   : std_logic := '0';
    signal r_pre0_n : unsigned(4 downto 0) := (others => '0');
    signal rh_hr    : unsigned(C_ROWB downto 0) := (others => '0');
    signal rh_acc   : unsigned(10 downto 0) := (others => '0');
    signal rh_fy    : unsigned(18 downto 0) := (others => '0');
    signal rv_set   : std_logic := '0';
    signal rv_r0, rv_r1 : unsigned(C_ROWB - 1 downto 0) := (others => '0');
    signal rv_fy    : unsigned(7 downto 0) := (others => '0');
    signal rb_acc   : unsigned(11 downto 0) := (others => '0');
    signal rb_hc    : unsigned(C_COLB downto 0) := (others => '0');
    signal rb_hcq   : std_logic := '0';
    signal cur_v, cur_vn : unsigned(9 downto 0) := (others => '0');
    signal cur_st   : signed(17 downto 0) := (others => '0');
    signal dda      : signed(21 downto 0) := (others => '0');
    signal r2_odd   : std_logic := '0';
    signal r3_m     : unsigned(9 downto 0) := (others => '0');
    signal r4_pos   : unsigned(10 downto 0) := (others => '0');
    signal r5_x     : unsigned(9 downto 0) := (others => '0');
    signal r5_sat   : std_logic := '0';
    signal r6_key   : unsigned(9 downto 0) := (others => '0');   -- raw (pre-gate)
    signal r_key    : unsigned(9 downto 0) := (others => '0');
    signal r4_m, r5_m, r6_m, r_matte : unsigned(9 downto 0) := (others => '0');
    -- DETAIL edge gate
    signal ry1, ry1q, ry1qq : unsigned(9 downto 0) := (others => '0');
    signal r1_avqq  : std_logic := '0';
    signal r2_g     : unsigned(9 downto 0) := (others => '0');
    signal r3_e     : unsigned(9 downto 0) := (others => '0');
    signal r4_gate, r5_gate, r6_gate : unsigned(9 downto 0) := (others => '1');

    -- column processor
    type t_cs is (CS_IDLE, CS_RA, CS_RB, CS_W2, CS_W3, CS_CA, CS_CB, CS_L0, CS_L1,
                  CS_L2, CS_S0, CS_S1, CS_S2, CS_S3);
    signal cs       : t_cs := CS_IDLE;
    attribute fsm_encoding of cs : signal is "none";
    signal cp_k     : unsigned(C_COLB + 1 downto 0) := (others => '0');
    signal cp_col   : unsigned(C_COLB - 1 downto 0) := (others => '0');
    signal cp_pre   : std_logic := '0';     -- pre-line pass (k = 0 then 1)
    signal cp_go    : std_logic := '0';     -- segment-start trigger
    signal cp_line  : std_logic := '0';     -- pre-line request
    signal cp_a     : unsigned(9 downto 0) := (others => '0');
    signal cp_v, cp_vprev : unsigned(9 downto 0) := (others => '0');
    signal v0_line  : unsigned(9 downto 0) := (others => '0');
    signal pn_v     : unsigned(9 downto 0) := (others => '0');
    signal pn_s     : signed(17 downto 0) := (others => '0');
    signal rnd_mreq : std_logic := '0';
    signal rnd_maddr : unsigned(C_CELLB - 1 downto 0) := (others => '0');
    signal mu_x     : signed(10 downto 0) := (others => '0');
    signal mu_y     : unsigned(15 downto 0) := (others => '0');
    signal mu_lo, mu_hi : signed(19 downto 0) := (others => '0');
    signal mu_p     : signed(27 downto 0) := (others => '0');

begin

    --------------------------------------------------------------------------
    -- control parameters (2-stage, quasi-static)
    --------------------------------------------------------------------------
    p_params : process(clk)
        variable v_e  : integer range 0 to 255;
        variable v_ew : unsigned(8 downto 0);
        variable v_s  : unsigned(9 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_th : unsigned(13 downto 0);
        variable v_kq : integer range 0 to 31;
    begin
        if rising_edge(clk) then
            tid_q <= timing_id;
            -- interlace: SD trusts the timing ID (the decoder's field_n is real). HD:
            -- the core ties field_n high, so interlace = timing ID, the SDK sync-phase
            -- detector, or a field_n that really toggles (image tester); without a live
            -- field_n, every other field updates (field toggle) -> same-parity compares
            one_field <= '0';
            if ana_params.both_fields = '0' then
                if f_is_sd(tid_q) then
                    if f_is_interlaced(tid_q) then one_field <= '1'; end if;
                elsif f_is_interlaced(tid_q) or det_il = '1' or fn_live = '1' then
                    one_field <= '1';
                end if;
            end if;
            if fn_tog /= 0 then fn_live <= '1'; else fn_live <= '0'; end if;
            p_f4     <= ana_params.floor(9 downto 4);
            p_steady <= ana_params.steady;
            p_dinv   <= not rnd_params.detail;
            p_chroma <= ana_params.chroma;
            -- WAKE: e16 = clamp((((wake >> 3) * 3) >> 2) - 95 + rate16, 0, 255)
            v_ew   := resize(ana_params.wake(9 downto 3), 9) + shift_left(resize(ana_params.wake(9 downto 3), 9), 1);
            p_ew   <= v_ew(8 downto 2);
            if one_field = '1' then
                p_rate <= to_unsigned(C_RATE_OFF16(to_integer(unsigned(tid_q))) - 16, 7);
            else
                p_rate <= to_unsigned(C_RATE_OFF16(to_integer(unsigned(tid_q))), 7);
            end if;
            v_e := 0;
            if to_integer(p_ew) + to_integer(p_rate) > C_WAKE_BIAS then
                v_e := to_integer(p_ew) + to_integer(p_rate) - C_WAKE_BIAS;
            end if;
            p_eint <= to_unsigned(v_e / 16, 4);
            p_mant <= to_unsigned(C_MANT_DN(v_e mod 16), 6);
            -- STIR threshold (v3): e = ((1023 - sens) * 17) >> 7, 8 octaves:
            -- thr = min(1024, (MANT_UP[e & 15] << (e >> 4)) >> 3)
            v_s  := not rnd_params.sens;
            v_t  := shift_left(resize(v_s, 14), 4) + resize(v_s, 14);
            p_te <= v_t(13 downto 7);
            v_th := shift_left(to_unsigned(C_MANT_UP(to_integer(p_te(3 downto 0))), 14),
                               to_integer(p_te(6 downto 4)));
            if v_th(13 downto 3) > 1024 then
                p_thr <= to_unsigned(1024, 11);
            else
                p_thr <= v_th(13 downto 3);
            end if;
            -- SOFTNESS -> gain 2^(kq/2)
            if rnd_params.softness < 128 then
                p_near <= '1';
                p_k    <= to_unsigned(8, 4);
                p_half <= '0';
            else
                v_kq   := 16 - to_integer(shift_right(rnd_params.softness - 128, 6));
                p_near <= '0';
                p_k    <= to_unsigned(v_kq / 2, 4);
                if v_kq mod 2 = 1 then p_half <= '1'; else p_half <= '0'; end if;
            end if;
            p_inv <= rnd_params.invert;
        end if;
    end process p_params;

    --------------------------------------------------------------------------
    -- A0: input register (8-bit samples)
    --------------------------------------------------------------------------
    p_a0 : process(clk)
    begin
        if rising_edge(clk) then
            a0_y    <= unsigned(ana_in.y(9 downto 2));
            a0_u    <= unsigned(ana_in.u(9 downto 2));
            a0_v    <= unsigned(ana_in.v(9 downto 2));
            a0_avid <= ana_in.avid;
            a0_vs_n <= ana_in.vsync_n;
            if f_is_sd(tid_q) or fn_live = '1' then
                a0_fld <= ana_in.field_n;
            else
                a0_fld <= fld_t;
            end if;
        end if;
    end process p_a0;

    u_fld : entity work.video_field_detector
        generic map (G_LINE_COUNTER_WIDTH => 12)
        port map (clk => clk, hsync => ana_in.hsync_n, vsync => ana_in.vsync_n,
                  field_n => det_fld_n, is_interlaced => det_il);

    --------------------------------------------------------------------------
    -- A1: field / line control, measurement, column + row Bresenham
    --------------------------------------------------------------------------
    p_a1 : process(clk)
        variable v_line : std_logic;
        variable v_upd  : std_logic;
        variable v_bx   : unsigned(12 downto 0);
        variable v_bx2  : signed(13 downto 0);
        variable v_ra   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            a1_avq  <= a0_avid;
            a1_vsq  <= a0_vs_n;
            upd_end <= '0';

            -- update-field decision: made on the first pixel of line 0 and used on
            -- that same pixel (a registered decision would drop it and leak a stale
            -- partial sum into the first sub-cell)
            v_upd := upd_line;
            if a0_avid = '1' and a1_avq = '0' and y_line = 0 then
                if one_field = '0' or a0_fld = '1' then v_upd := '1'; else v_upd := '0'; end if;
            end if;
            v_line := '0';
            if v_upd = '1' and g_ok = '1' and y_line < g_h then
                v_line := '1';
            end if;

            -- pixel counter and half-column Bresenham: hc = floor(x*2C/W)
            if a0_avid = '1' then
                xc      <= xc + 1;
                saw_act <= '1';
                v_bx  := resize(bx_acc, 13) + shift_left(resize(g_c, 13), 1);
                v_bx2 := signed(resize(v_bx, 14)) - signed(resize(g_w, 14));
                if v_bx2 >= 0 then
                    bx_acc <= unsigned(v_bx2(11 downto 0));
                    if resize(bx_hc, 7) /= shift_left(resize(g_c, 7), 1) - 1 then
                        bx_hc <= bx_hc + 1;
                    end if;
                else
                    bx_acc <= v_bx(11 downto 0);
                end if;
            else
                xc     <= (others => '0');
                bx_acc <= (others => '0');
                bx_hc  <= (others => '0');
            end if;

            -- line start: is this field an update field?
            if a0_avid = '1' and a1_avq = '0' then
                if y_line = 0 then
                    fn_prev <= ana_in.field_n;
                    if ana_in.field_n /= fn_prev then
                        fn_tog <= "11";
                    elsif fn_tog /= 0 then
                        fn_tog <= fn_tog - 1;
                    end if;
                    upd_line <= v_upd;                         -- decided once per field
                end if;
                if y_line >= ob_y and y_line < ob_y2 then ob_v <= '1'; else ob_v <= '0'; end if;
            end if;

            -- line end: count, measure, advance row Bresenham
            if a0_avid = '0' and a1_avq = '1' then
                if y_line /= 2047 then
                    y_line <= y_line + 1;
                end if;
                if upd_line = '1' then
                    fld_seen <= '1';
                    hm_live  <= y_line + 1;
                    if y_line = 2 then
                        wm_live <= xc;       -- fixed early line: field-stable width
                    end if;
                end if;
                ar_cur <= ar_nxt;
                if ar_nxt /= ar_cur then ar_fy <= '1'; else ar_fy <= '0'; end if;
                if ar_nxt(C_ROWB downto 1) /= ar_cur(C_ROWB downto 1) then
                    ar_fc <= '1';
                else
                    ar_fc <= '0';
                end if;
                v_ra := resize(ar_acc, 12) + 2 * C_R;
                if v_ra >= g_h then
                    ar_acc <= resize(v_ra - g_h, 11);
                    if ar_nxt /= to_unsigned(2 ** (C_ROWB + 1) - 1, C_ROWB + 1) then
                        ar_nxt <= ar_nxt + 1;
                    end if;
                else
                    ar_acc <= v_ra(10 downto 0);
                end if;
            end if;

            -- field boundary: vsync falling edge, once per field (serration guard)
            if a1_vsq = '1' and a0_vs_n = '0' and saw_act = '1' then
                saw_act  <= '0';
                fld_t    <= not fld_t;
                upd_end  <= fld_seen;
                fld_seen <= '0';
                if fld_seen = '1' then
                    hm <= hm_live;
                    wm <= wm_live;
                end if;
                y_line   <= (others => '0');
                ar_cur   <= (others => '0');
                ar_nxt   <= (others => '0');
                ar_acc   <= to_unsigned(2 * C_R, 11);   -- state for line 1 (H > 2R guaranteed)
                ar_fy    <= '1';
                ar_fc    <= '1';
            end if;

            -- A2 register stage
            a2_y   <= a0_y;
            a2_u   <= a0_u;
            a2_v   <= a0_v;
            a2_hc  <= bx_hc;
            a2_act   <= a0_avid and v_line;
            a2_first <= a0_avid and not a1_avq;
            a2_end   <= (not a0_avid) and a1_avq and v_line;
            if ar_nxt(C_ROWB downto 1) /= ar_cur(C_ROWB downto 1) or y_line + 1 = g_h then
                a2_last <= '1';
            else
                a2_last <= '0';
            end if;
            a2_fy   <= ar_fy;
            a2_fc   <= ar_fc;
            a2_sub  <= ar_cur(0);
            a2_bank <= ar_cur(1);
            a2_row  <= ar_cur(C_ROWB downto 1);
            if G_TEST_OBJECT and xc >= ob_x and xc < ob_x2 then ob_h <= '1'; else ob_h <= '0'; end if;
        end if;
    end process p_a1;

    a2_tob <= ob_h and ob_v when G_TEST_OBJECT else '0';

    --------------------------------------------------------------------------
    -- A3: per-line partial sums (luma per half-column, U/V per column), emit
    --------------------------------------------------------------------------
    p_a3 : process(clk)
        type t_px is array (0 to 2) of unsigned(7 downto 0);
        variable v_px  : t_px;
        variable v_col : boolean;
    begin
        if rising_edge(clk) then
            v_px := (a2_y, a2_u, a2_v);
            if a2_tob = '1' then
                v_px := (x"F0", x"80", x"80");
            end if;
            ey_go <= '0';
            ec_go <= '0';
            v_col := a2_hc(C_HCB - 1 downto 1) /= ps_hc(C_HCB - 1 downto 1);
            if a2_act = '1' then
                if a2_first = '1' or a2_hc /= ps_hc then
                    if a2_first = '0' then
                        -- luma of the finished half-column
                        ey_go    <= '1';
                        ey_addr  <= a2_sub & ps_hc;
                        ey_p     <= shift_right(resize(py, 16), to_integer(g_sh));
                        ey_fresh <= a2_fy;
                        e_bank   <= a2_bank;
                        if v_col then
                            -- U/V of the finished column
                            ec_go    <= '1';
                            ec_col   <= ps_hc(C_HCB - 1 downto 1);
                            eu_p     <= shift_right(resize(pu, 16), to_integer(g_sh));
                            ev_p     <= shift_right(resize(pv, 16), to_integer(g_sh));
                            ec_fresh <= a2_fc;
                        end if;
                    end if;
                    py    <= resize(v_px(0), 15);
                    ps_hc <= a2_hc;
                    if a2_first = '1' or v_col then
                        pu <= resize(v_px(1), 15);
                        pv <= resize(v_px(2), 15);
                    else
                        pu <= pu + v_px(1);
                        pv <= pv + v_px(2);
                    end if;
                else
                    py <= py + v_px(0);
                    pu <= pu + v_px(1);
                    pv <= pv + v_px(2);
                end if;
            elsif a2_end = '1' then
                ey_go    <= '1';
                ey_addr  <= a2_sub & ps_hc;
                ey_p     <= shift_right(resize(py, 16), to_integer(g_sh));
                ey_fresh <= a2_fy;
                ec_go    <= '1';
                ec_col   <= ps_hc(C_HCB - 1 downto 1);
                eu_p     <= shift_right(resize(pu, 16), to_integer(g_sh));
                ev_p     <= shift_right(resize(pv, 16), to_integer(g_sh));
                ec_fresh <= a2_fc;
                e_bank   <= a2_bank;
            end if;
        end if;
    end process p_a3;

    --------------------------------------------------------------------------
    -- accumulator RMW: issue up to 3 ops (Y, U, V), bank address mux, tags, add
    --------------------------------------------------------------------------
    p_rmw : process(clk)
        variable v_addr : unsigned(C_ACCB - 1 downto 0);
        variable v_old  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if ey_go = '1' or ec_go = '1' then
                iss  <= "00";
                sv_y <= ey_go;
                sv_c <= ec_go;
            elsif iss /= "11" then
                iss <= iss + 1;
            end if;
            -- slot 0: luma (sub-row & hc); 1: U at 128 + 2*col; 2: V at 129 + 2*col
            case iss is
                when "00" =>
                    v_addr   := resize(ey_addr, C_ACCB);
                    t1.v     <= sv_y;
                    t1.part  <= ey_p;
                    t1.fresh <= ey_fresh;
                when "01" =>
                    v_addr   := to_unsigned(128, C_ACCB) + shift_left(resize(ec_col, C_ACCB), 1);
                    t1.v     <= sv_c;
                    t1.part  <= eu_p;
                    t1.fresh <= ec_fresh;
                when "10" =>
                    v_addr   := to_unsigned(129, C_ACCB) + shift_left(resize(ec_col, C_ACCB), 1);
                    t1.v     <= sv_c;
                    t1.part  <= ev_p;
                    t1.fresh <= ec_fresh;
                when others =>
                    v_addr   := to_unsigned(2 ** C_ACCB - 1, C_ACCB);   -- idle: unused word
                    t1.v     <= '0';
                    t1.part  <= ey_p;
                    t1.fresh <= ey_fresh;
            end case;
            t1.addr  <= v_addr;
            t1.bank  <= e_bank;
            -- bank read addresses: the finalizer owns the bank it is reading
            if f_act = '1' and f_bank = '0' then acc0_ra <= f_ra; else acc0_ra <= v_addr; end if;
            if f_act = '1' and f_bank = '1' then acc1_ra <= f_ra; else acc1_ra <= v_addr; end if;
            t2 <= t1;
            t3 <= t2;
            acc0_rq <= acc0_rd;
            acc1_rq <= acc1_rd;
            -- add (t3 aligns with the registered read data)
            if t3.bank = '0' then v_old := acc0_rq; else v_old := acc1_rq; end if;
            if t3.fresh = '1' then v_old := (others => '0'); end if;
            acc_wd  <= v_old + t3.part;
            acc_wa  <= t3.addr;
            acc_we0 <= t3.v and not t3.bank;
            acc_we1 <= t3.v and t3.bank;
        end if;
    end process p_rmw;

    p_acc0 : process(clk)
    begin
        if rising_edge(clk) then
            if acc_we0 = '1' then
                r_acc0(to_integer(acc_wa)) <= acc_wd;
            end if;
            acc0_rd <= r_acc0(to_integer(acc0_ra));
        end if;
    end process p_acc0;

    p_acc1 : process(clk)
    begin
        if rising_edge(clk) then
            if acc_we1 = '1' then
                r_acc1(to_integer(acc_wa)) <= acc_wd;
            end if;
            acc1_rd <= r_acc1(to_integer(acc1_ra));
        end if;
    end process p_acc1;

    --------------------------------------------------------------------------
    -- row done -> finalizer request (16-clock drain for the last RMWs)
    --------------------------------------------------------------------------
    p_rowdone : process(clk)
    begin
        if rising_edge(clk) then
            if a2_end = '1' and a2_last = '1' then
                rd_pend <= '1';
                rd_row  <= a2_row;
                rd_dly  <= (others => '1');
            elsif rd_dly /= 0 then
                rd_dly <= rd_dly - 1;
            elsif fs = FS_IDLE and rd_pend = '1' then
                rd_pend <= '0';
            end if;
        end if;
    end process p_rowdone;

    --------------------------------------------------------------------------
    -- finalizer: one cell at a time, 6 values in sequence, serial 16x12 multiplier
    --------------------------------------------------------------------------
    p_fin : process(clk)
        variable v_c    : unsigned(27 downto 0);
        variable v_cur  : unsigned(9 downto 0);
        variable v_df   : signed(11 downto 0);
        variable v_ad   : unsigned(9 downto 0);
        variable v_nr   : signed(13 downto 0);
        variable v_dd   : signed(13 downto 0);
        variable v_dec  : unsigned(11 downto 0);
        variable v_thr  : unsigned(21 downto 0);
        variable v_cell : unsigned(C_CELLB - 1 downto 0);
    begin
        if rising_edge(clk) then
            ref_we    <= '0';
            m_we      <= '0';
            f_upd     <= '0';
            seed_done <= '0';
            cut_evt   <= '0';

            -- MGRID read arbiter: renderer has fixed priority
            if rnd_mreq = '1' then
                m_ra   <= rnd_maddr;
                m_tag1 <= '0';
            elsif f_mpend = '1' then
                m_ra    <= f_cell;
                m_tag1  <= '1';
                f_mpend <= '0';
            else
                m_tag1 <= '0';
            end if;
            if m_tag3 = '1' then
                f_mold <= m_rq;
                f_mgot <= '1';
            end if;

            case fs is
                when FS_IDLE =>
                    f_act <= '0';
                    if rd_pend = '1' and rd_dly = 0 then
                        f_row   <= rd_row;
                        f_bank  <= rd_row(0);
                        f_col   <= (others => '0');
                        f_v     <= (others => '0');
                        f_dsy   <= (others => '0');
                        f_dsc   <= (others => '0');
                        f_seed  <= seed_pend;
                        if rd_row = 0 then
                            f_gen0  <= seed_gen;
                            f_allsd <= seed_pend;
                            f_cut   <= '0';
                            f_esum  <= (others => '0');
                            f_thr   <= (others => '0');
                        elsif seed_pend = '0' or seed_gen /= f_gen0 then
                            f_allsd <= '0';
                        end if;
                        f_recip <= g_recip;
                        f_act   <= '1';
                        v_cell  := rd_row & to_unsigned(0, C_COLB);
                        f_cell  <= v_cell;
                        f_cell6 <= shift_left(resize(v_cell, C_REFB), 2) + shift_left(resize(v_cell, C_REFB), 1);
                        fs <= FS_VRD;
                    end if;

                -- one value: FIN bank word + REF word (+ MGRID once per cell)
                when FS_VRD =>
                    if f_v(2) = '0' then           -- luma sub-cell (sub-row f_v(1), hc 2c+f_v(0))
                        f_ra <= resize(f_v(1) & f_col & f_v(0), C_ACCB);
                    else                           -- U / V
                        f_ra <= to_unsigned(128, C_ACCB) + shift_left(resize(f_col, C_ACCB), 1)
                                + resize(f_v(0 downto 0), C_ACCB);
                    end if;
                    ref_ra <= f_cell6 + f_v;
                    if f_v = 0 then
                        f_mpend <= '1';
                        f_mgot  <= '0';
                    end if;
                    fs <= FS_VW1;
                when FS_VW1 =>
                    fs <= FS_VW2;
                when FS_VW2 =>
                    fs <= FS_VW3;
                when FS_VW3 =>
                    fs <= FS_VCAP;
                when FS_VCAP =>
                    if f_bank = '0' then f_a <= acc0_rq; else f_a <= acc1_rq; end if;
                    f_ref <= ref_rqq;
                    if f_bank = '0' then mp_a <= resize(acc0_rq, 28); else mp_a <= resize(acc1_rq, 28); end if;
                    mp_b <= f_recip; mp_p <= (others => '0');
                    mp_n <= to_unsigned(11, 4); fs_ret <= FS_VCUR; fs <= FS_MUL;
                -- cur = min(1023, (A*recip + rnd) >> 12 luma sub-cell / 14 chroma), 1/4 LSB
                when FS_VCUR =>
                    if f_v(2) = '0' then
                        v_c := mp_p + 2048;
                        if v_c(27 downto 22) /= 0 then f_cur <= (others => '1'); else f_cur <= v_c(21 downto 12); end if;
                    else
                        v_c := mp_p + 8192;
                        if v_c(27 downto 24) /= 0 then f_cur <= (others => '1'); else f_cur <= v_c(23 downto 14); end if;
                    end if;
                    fs <= FS_VDIF;
                -- diff vs the 1/16-LSB reference; reference moves 1/4 of the way
                when FS_VDIF =>
                    v_cur := f_cur;
                    v_df  := signed(resize(v_cur, 12)) - signed(resize(shift_right(resize(f_ref, 13) + 2, 2), 12));
                    v_ad  := unsigned(resize(abs(v_df), 10));
                    if f_seed = '0' then
                        if f_v(2) = '0' then
                            f_dsy <= f_dsy + v_ad;
                        else
                            f_dsc <= f_dsc + v_ad;
                        end if;
                    end if;
                    v_nr := signed(resize(shift_left(resize(v_cur, 12), 2), 14)) - signed(resize(f_ref, 14));
                    if f_seed = '1' then
                        ref_wd <= shift_left(resize(v_cur, 12), 2);
                    else
                        v_nr   := signed(resize(f_ref, 14)) + shift_right(v_nr, 2);
                        ref_wd <= unsigned(v_nr(11 downto 0));
                    end if;
                    ref_wa <= f_cell6 + f_v;
                    fs <= FS_VWR;
                when FS_VWR =>
                    ref_we <= '1';
                    if f_v = 5 then
                        f_v <= (others => '0');
                        fs  <= FS_MW;
                    else
                        f_v <= f_v + 1;
                        fs  <= FS_VRD;
                    end if;
                when FS_MW =>
                    if f_mgot = '1' then
                        mp_a <= resize(f_dsc, 28); mp_b <= "00" & p_chroma; mp_p <= (others => '0');
                        mp_n <= to_unsigned(11, 4); fs_ret <= FS_D0; fs <= FS_MUL;
                    end if;
                -- d = (sum dY >> 1) + ((|dU| + |dV|) * CHROMA) >> 9
                when FS_D0 =>
                    f_d <= resize(shift_right(f_dsy, 1), 13) + mp_p(21 downto 9);
                    fs  <= FS_D1;
                -- energy (raw d); STEADY: subtract last update's mean
                when FS_D1 =>
                    if f_seed = '0' then
                        f_esum <= f_esum + f_d;
                    end if;
                    if p_steady = '1' then
                        v_dd := signed(resize(f_d, 14)) - signed(resize(f_gprev, 14));
                        if v_dd < 0 then f_dc <= (others => '0'); else f_dc <= unsigned(v_dd(12 downto 0)); end if;
                    else
                        f_dc <= f_d;
                    end if;
                    fs <= FS_D2;
                when FS_D2 =>
                    v_dd := signed(resize(f_dc, 14)) - signed(resize(p_f4, 14));
                    if v_dd < 0 then
                        f_dp <= (others => '0');
                    elsif v_dd > 255 then
                        f_dp <= (others => '1');
                    else
                        f_dp <= unsigned(v_dd(7 downto 0));
                    end if;
                    fs <= FS_R0;

                -- release: dec = M ? max(1, ((M >> e_int) * mant) >> 5) : 0
                when FS_R0 =>
                    f_msh <= shift_right(f_mold, to_integer(p_eint));
                    fs <= FS_R1;
                when FS_R1 =>
                    mp_a <= resize(f_msh, 28); mp_b <= resize(p_mant, 12);
                    mp_p <= (others => '0'); mp_n <= to_unsigned(11, 4);
                    fs_ret <= FS_R2; fs <= FS_MUL;
                when FS_R2 =>
                    v_dec := mp_p(16 downto 5);
                    if f_mold = 0 then
                        f_dec <= (others => '0');
                    elsif v_dec = 0 then
                        f_dec <= to_unsigned(1, 12);
                    else
                        f_dec <= v_dec;
                    end if;
                    fs <= FS_R3;
                when FS_R3 =>
                    f_mrel <= f_mold - f_dec;
                    fs <= FS_R4;
                when FS_R4 =>
                    if f_seed = '1' then
                        f_mnew <= (others => '0');
                    elsif shift_left(resize(f_dp, 12), 4) > f_mrel then
                        f_mnew <= shift_left(resize(f_dp, 12), 4);
                    else
                        f_mnew <= f_mrel;
                    end if;
                    fs <= FS_W0;

                -- MGRID write: hold off while the renderer reads MGRID next edge
                -- (a same-cycle write to the read address returns undefined EBR data)
                when FS_W0 =>
                    if rnd_mreq = '0' then
                        m_we <= '1';
                        m_wa <= f_cell;
                        m_wd <= f_mnew;
                        fs   <= FS_NEXT;
                    end if;
                when FS_NEXT =>
                    f_dsy <= (others => '0');
                    f_dsc <= (others => '0');
                    if resize(f_col, 6) = g_c - 1 then
                        fs <= FS_ROW;
                    else
                        f_col   <= f_col + 1;
                        f_cell  <= f_cell + 1;
                        f_cell6 <= f_cell6 + 6;
                        fs <= FS_VRD;
                    end if;

                -- row end: STEADY cut guard (running mean d over rows so far, rows >= 2)
                when FS_ROW =>
                    if f_g_c_hd = '1' then
                        v_thr := f_thr + to_unsigned(C_TC_HD, 22);
                    else
                        v_thr := f_thr + to_unsigned(C_TC_SD, 22);
                    end if;
                    f_thr <= v_thr;
                    if f_seed = '0' and p_steady = '1' and f_row >= 2 and f_cut = '0'
                       and f_esum > v_thr then
                        f_cut   <= '1';
                        cut_evt <= '1';
                    end if;
                    if f_row = to_unsigned(C_R - 1, C_ROWB) then
                        f_upd     <= '1';
                        seed_done <= f_allsd;
                        done_gen  <= f_gen0;
                        f_energy  <= f_esum;
                        -- g_prev = ((sum d >> 4) * recipN) >> 14
                        mp_a <= resize(shift_right(f_esum, 4), 28);
                        if f_g_c_hd = '1' then
                            mp_b <= to_unsigned(C_RN_HD, 12);
                        else
                            mp_b <= to_unsigned(C_RN_SD, 12);
                        end if;
                        mp_p <= (others => '0'); mp_n <= to_unsigned(11, 4);
                        fs_ret <= FS_G1; fs <= FS_MUL;
                    else
                        f_act <= '0';
                        fs    <= FS_IDLE;
                    end if;
                when FS_G1 =>
                    f_gprev <= mp_p(26 downto 14);
                    f_esum  <= (others => '0');
                    f_thr   <= (others => '0');
                    f_act   <= '0';
                    fs      <= FS_IDLE;

                -- serial shift-add multiply: mp_p = mp_a * mp_b (12 bits of b)
                when FS_MUL =>
                    if mp_b(0) = '1' then
                        mp_p <= mp_p + mp_a;
                    end if;
                    mp_a <= shift_left(mp_a, 1);
                    mp_b <= shift_right(mp_b, 1);
                    if mp_n = 0 then
                        fs <= fs_ret;
                    else
                        mp_n <= mp_n - 1;
                    end if;
            end case;
        end if;
    end process p_fin;

    f_g_c_hd <= '1' when g_c = G_COLS_HD else '0';

    -- REF and MGRID memories (finalizer is the only writer)
    p_ref : process(clk)
    begin
        if rising_edge(clk) then
            if ref_we = '1' then
                r_ref(to_integer(ref_wa)) <= ref_wd;
            end if;
            ref_rd  <= r_ref(to_integer(ref_ra));
            ref_rq  <= ref_rd;
            ref_rqq <= ref_rq;
        end if;
    end process p_ref;

    p_m : process(clk)
    begin
        if rising_edge(clk) then
            if m_we = '1' then
                r_m(to_integer(m_wa)) <= m_wd;
            end if;
            m_rd   <= r_m(to_integer(m_ra));
            m_rq   <= m_rd;
            m_tag2 <= m_tag1;
            m_tag3 <= m_tag2;
        end if;
    end process p_m;

    --------------------------------------------------------------------------
    -- update bookkeeping: warm-up / seed counter, test object
    --------------------------------------------------------------------------
    p_upd : process(clk)
        variable v_dx : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if timing_id /= tid_q or geo_chg = '1' or cut_evt = '1' then
                seed_pend <= '1';
                seed_gen  <= seed_gen + 1;
                if cut_evt = '1' then
                    mute <= '1';                 -- cut: key off until re-seeded
                end if;
            elsif seed_done = '1' and done_gen = seed_gen then
                seed_pend <= '0';
                mute      <= '0';
            end if;
            if G_TEST_OBJECT and upd_end = '1' then
                v_dx := shift_right(g_w, 6);
                if ob_dir = '0' then
                    if ob_x + v_dx + shift_right(g_w, 4) >= g_w then
                        ob_dir <= '1';
                    else
                        ob_x <= ob_x + v_dx;
                    end if;
                else
                    if ob_x < v_dx then
                        ob_dir <= '0';
                    else
                        ob_x <= ob_x - v_dx;
                    end if;
                end if;
            end if;
            ob_x2 <= ob_x + shift_right(g_w, 4);
            ob_y  <= shift_right(g_h, 1) - shift_right(g_h, 4);
            ob_y2 <= shift_right(g_h, 1) + shift_right(g_h, 4);
        end if;
    end process p_upd;

    --------------------------------------------------------------------------
    -- geometry sequencer (after each update field; ~250 clocks)
    --------------------------------------------------------------------------
    p_geo : process(clk)
        variable v_rem : unsigned(23 downto 0);
        variable v_bl  : integer range 0 to 15;
        variable v_n   : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            geo_chg <= '0';
            case gs is
                when GS_IDLE =>
                    geo_busy <= '0';
                    -- recompute only during warm-up or when W/H move by more than 2
                    if upd_end = '1' and (g_ok = '0' or seed_pend = '1' or
                       wm > g_w + 2 or wm + 2 < g_w or hm > g_h + 2 or hm + 2 < g_h) then
                        geo_busy <= '1';
                        q_w <= wm;
                        q_h <= hm;
                        if f_is_sd(tid_q) then
                            q_c <= to_unsigned(G_COLS_SD, 6);
                        else
                            q_c <= to_unsigned(G_COLS_HD, 6);
                        end if;
                        gs <= GS_CHK;
                    end if;
                when GS_CHK =>
                    if q_w < 64 or q_h < 2 * C_R + 2 then
                        g_ok <= '0';
                        gs   <= GS_IDLE;
                    else
                        -- cw_max = ceil(W / C)
                        dv_q   <= resize(q_w + q_c - 1, 36);
                        dv_den <= resize(q_c, 23);
                        dv_rem <= (others => '0');
                        dv_n   <= to_unsigned(35, 6);
                        gs_ret <= GS_D1R;
                        gs     <= GS_DIV;
                    end if;
                when GS_D1R =>
                    q_cw   <= dv_q(6 downto 0);
                    -- ch_max = ceil(H / R)
                    dv_q   <= resize(q_h + (C_R - 1), 36);
                    dv_den <= to_unsigned(C_R, 23);
                    dv_rem <= (others => '0');
                    dv_n   <= to_unsigned(35, 6);
                    gs_ret <= GS_D2R;
                    gs     <= GS_DIV;
                when GS_D2R =>
                    q_ch <= dv_q(6 downto 0);
                    gm_a <= resize(q_cw, 24);
                    gm_b <= resize(dv_q(6 downto 0), 12);
                    gm_p <= (others => '0');
                    gm_n <= to_unsigned(11, 4);
                    gs_ret <= GS_M1R;
                    gs     <= GS_MUL;
                when GS_M1R =>
                    -- sh = max(0, bitlen(cw_max * ch_max) - 8)
                    v_n  := gm_p(13 downto 0);
                    v_bl := 0;
                    for i in 0 to 13 loop
                        if v_n(i) = '1' then v_bl := i + 1; end if;
                    end loop;
                    if v_bl > 8 then q_sh <= to_unsigned(v_bl - 8, 3); else q_sh <= "000"; end if;
                    gm_a <= resize(q_w, 24);
                    gm_b <= resize(q_h, 12);
                    gm_p <= (others => '0');
                    gm_n <= to_unsigned(11, 4);
                    gs_ret <= GS_M2R;
                    gs     <= GS_MUL;
                when GS_M2R =>
                    q_wh <= gm_p(22 downto 0);
                    gs   <= GS_D3;
                when GS_D3 =>
                    -- recip = round(2^(16+sh) * C*R / (W*H))
                    dv_q   <= shift_left(resize(q_c * to_unsigned(C_R, 6), 36), 16 + to_integer(q_sh))
                              + resize(shift_right(q_wh, 1), 36);
                    dv_den <= q_wh;
                    dv_rem <= (others => '0');
                    dv_n   <= to_unsigned(35, 6);
                    gs_ret <= GS_D3R;
                    gs     <= GS_DIV;
                when GS_D3R =>
                    if dv_q(35 downto 12) /= 0 then
                        q_recip <= (others => '1');
                    else
                        q_recip <= dv_q(11 downto 0);
                    end if;
                    -- recip_c = round(2^16 * C / W)
                    dv_q   <= shift_left(resize(q_c, 36), 16) + resize(shift_right(q_w, 1), 36);
                    dv_den <= resize(q_w, 23);
                    dv_rem <= (others => '0');
                    dv_n   <= to_unsigned(35, 6);
                    gs_ret <= GS_D4R;
                    gs     <= GS_DIV;
                when GS_D4R =>
                    q_rc   <= dv_q(15 downto 0);
                    -- recip_r = round(2^16 * R / H)
                    dv_q   <= shift_left(to_unsigned(C_R, 36), 16) + resize(shift_right(q_h, 1), 36);
                    dv_den <= resize(q_h, 23);
                    dv_rem <= (others => '0');
                    dv_n   <= to_unsigned(35, 6);
                    gs_ret <= GS_D5R;
                    gs     <= GS_DIV;
                when GS_D5R =>
                    if g_ok = '0' or q_w /= g_w or q_h /= g_h or q_c /= g_c then
                        geo_chg <= '1';           -- new geometry: the reference is stale
                    end if;
                    g_rr    <= dv_q(15 downto 0);
                    g_rc    <= q_rc;
                    g_recip <= q_recip;
                    g_sh    <= q_sh;
                    g_w     <= q_w;
                    g_h     <= q_h;
                    g_c     <= q_c;
                    g_ok    <= '1';
                    gs      <= GS_IDLE;

                -- restoring divider, 36 numerator bits MSB first (quotient in dv_q)
                when GS_DIV =>
                    v_rem := shift_left(dv_rem, 1);
                    v_rem(0) := dv_q(35);
                    if v_rem >= resize(dv_den, 24) then
                        dv_rem <= v_rem - resize(dv_den, 24);
                        dv_q   <= dv_q(34 downto 0) & '1';
                    else
                        dv_rem <= v_rem;
                        dv_q   <= dv_q(34 downto 0) & '0';
                    end if;
                    if dv_n = 0 then
                        gs <= gs_ret;
                    else
                        dv_n <= dv_n - 1;
                    end if;
                -- serial multiply, 12 bits of gm_b
                when GS_MUL =>
                    if gm_b(0) = '1' then
                        gm_p <= gm_p + gm_a;
                    end if;
                    gm_a <= shift_left(gm_a, 1);
                    gm_b <= shift_right(gm_b, 1);
                    if gm_n = 0 then
                        gs <= gs_ret;
                    else
                        gm_n <= gm_n - 1;
                    end if;
            end case;
        end if;
    end process p_geo;

    --------------------------------------------------------------------------
    -- RENDER R1: sync, vertical state, half-column Bresenham, DDA
    --------------------------------------------------------------------------
    p_r1 : process(clk)
        variable v_bx   : unsigned(12 downto 0);
        variable v_bx2  : signed(13 downto 0);
        variable v_hr   : unsigned(11 downto 0);
        variable v_new  : unsigned(C_ROWB downto 0);
        variable v_seg  : unsigned(C_ROWB downto 0);
        variable v_ls, v_ss : boolean;
    begin
        if rising_edge(clk) then
            r1_avid <= rnd_avid;
            r1_avq  <= r1_avid;
            r1_avqq <= r1_avq;
            ry1     <= unsigned(rnd_y);
            ry1q    <= ry1;
            ry1qq   <= ry1q;
            r1_vs   <= rnd_vsync_n;
            r1_vsq  <= r1_vs;
            cp_go   <= '0';
            cp_line <= '0';
            rv_set  <= '0';

            -- frame start: vsync edge once per field; line-0 state, then the
            -- pre-line pass once the geometry sequencer is idle
            if r1_avid = '1' then r_saw <= '1'; end if;
            if r1_vsq = '1' and r1_vs = '0' and r_saw = '1' then
                r_saw    <= '0';
                rh_hr    <= (others => '0');
                rh_acc   <= (others => '0');
                rh_fy    <= (others => '0');
                rv_set   <= '1';
                r_pre0   <= '1';
                r_pre0_n <= (others => '1');
            elsif r_pre0 = '1' then
                if r_pre0_n /= 0 then
                    r_pre0_n <= r_pre0_n - 1;
                elsif geo_busy = '0' then
                    r_pre0  <= '0';
                    cp_line <= '1';
                end if;
            end if;

            -- line end: advance half-row Bresenham + fy DDA, then pre-line pass
            if r1_avid = '0' and r1_avq = '1' then
                v_hr := resize(rh_acc, 12) + 2 * C_R;
                v_new := rh_hr;
                if v_hr >= g_h then
                    rh_acc <= resize(v_hr - g_h, 11);
                    if rh_hr /= 2 * C_R - 1 then
                        v_new := rh_hr + 1;
                    end if;
                else
                    rh_acc <= v_hr(10 downto 0);
                end if;
                rh_hr <= v_new;
                if v_new /= rh_hr and v_new(0) = '1' then
                    rh_fy <= (others => '0');
                else
                    rh_fy <= rh_fy + g_rr;
                end if;
                rv_set  <= '1';
            end if;
            if rv_set = '1' then
                cp_line <= not r_pre0;     -- line 0 waits for the geometry
            end if;

            -- per-line rows and fraction (registered; used by the column processor)
            if p_near = '1' then
                v_seg := shift_right(rh_hr, 1);
                rv_r0 <= v_seg(C_ROWB - 1 downto 0);
                rv_r1 <= v_seg(C_ROWB - 1 downto 0);
                rv_fy <= (others => '0');
            elsif rh_hr = 0 then
                rv_r0 <= (others => '0');
                rv_r1 <= (others => '0');
                rv_fy <= (others => '0');
            elsif rh_hr = 2 * C_R - 1 then
                rv_r0 <= to_unsigned(C_R - 1, C_ROWB);
                rv_r1 <= to_unsigned(C_R - 1, C_ROWB);
                rv_fy <= (others => '0');
            else
                v_seg := shift_right(rh_hr - 1, 1);
                rv_r0 <= v_seg(C_ROWB - 1 downto 0);
                rv_r1 <= v_seg(C_ROWB - 1 downto 0) + 1;
                if rh_fy(18 downto 8) > 255 then
                    rv_fy <= (others => '1');
                else
                    rv_fy <= rh_fy(15 downto 8);
                end if;
            end if;

            -- half-column Bresenham: hc = floor(x*2C/W), held at 0 in blanking
            if r1_avid = '1' then
                v_bx  := resize(rb_acc, 13) + shift_left(resize(g_c, 13), 1);   -- resize BEFORE shifting
                v_bx2 := signed(resize(v_bx, 14)) - signed(resize(g_w, 14));
                if v_bx2 >= 0 then
                    rb_acc <= unsigned(v_bx2(11 downto 0));
                    if resize(rb_hc, 7) /= shift_left(resize(g_c, 7), 1) - 1 then
                        rb_hc <= rb_hc + 1;
                    end if;
                else
                    rb_acc <= v_bx(11 downto 0);
                end if;
            else
                rb_acc <= (others => '0');
                rb_hc  <= (others => '0');
            end if;
            rb_hcq <= rb_hc(0);

            -- DDA: line start = left-edge constant V_0; segment start (hc turns
            -- odd) = load V_s and S_s, ask the column processor for one more
            v_ls := r1_avid = '1' and r1_avq = '0';
            v_ss := r1_avid = '1' and r1_avq = '1' and rb_hc(0) = '1' and rb_hcq = '0';
            if v_ls then
                cur_v  <= v0_line;
                cur_vn <= v0_line;
                cur_st <= (others => '0');
                dda    <= signed(shift_left(resize(v0_line, 22), 8) + 128);
            elsif v_ss then
                cur_v  <= cur_vn;
                cur_vn <= pn_v;
                cur_st <= pn_s;
                dda    <= signed(shift_left(resize(cur_vn, 22), 8) + 128);
                cp_go  <= '1';
            else
                dda <= dda + cur_st;
            end if;
            r2_odd <= rb_hc(0);
        end if;
    end process p_r1;

    --------------------------------------------------------------------------
    -- RENDER R2..R7: matte, threshold, gain, clamp, DETAIL gate, mute, invert
    --------------------------------------------------------------------------
    p_rk : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_sh : unsigned(18 downto 0);
        variable v_y  : unsigned(10 downto 0);
        variable v_g  : signed(10 downto 0);
        variable v_eg : unsigned(12 downto 0);
        variable v_e  : unsigned(9 downto 0);
        variable v_gt : unsigned(10 downto 0);
        variable v_k  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- DETAIL R1 -> R2: horizontal gradient |Y[x] - Y[x-2]| (0 for x < 2)
            v_g := signed(resize(ry1, 11)) - signed(resize(ry1qq, 11));
            if r1_avid = '1' and r1_avqq = '1' then
                r2_g <= unsigned(resize(abs(v_g), 10));
            else
                r2_g <= (others => '0');
            end if;
            -- R2 -> R3: edge energy, peak-held rightward: e = max(min(1023, g<<3), e - 64)
            v_eg := shift_left(resize(r2_g, 13), C_DETAIL_GAIN);
            if v_eg > 1023 then v_e := (others => '1'); else v_e := v_eg(9 downto 0); end if;
            if r3_e > C_DETAIL_DECAY and r3_e - C_DETAIL_DECAY > v_e then
                r3_e <= r3_e - C_DETAIL_DECAY;
            else
                r3_e <= v_e;
            end if;
            -- R3 -> R4: gate = min(1023, e + (1023 - DETAIL))
            v_gt := resize(r3_e, 11) + p_dinv;
            if v_gt(10) = '1' then r4_gate <= (others => '1'); else r4_gate <= v_gt(9 downto 0); end if;
            r5_gate <= r4_gate;
            r6_gate <= r5_gate;

            -- R2 -> R3
            if p_near = '1' then
                if r2_odd = '1' then r3_m <= cur_v; else r3_m <= cur_vn; end if;
            elsif dda < 0 then
                r3_m <= (others => '0');
            elsif dda(21 downto 18) /= 0 then
                r3_m <= (others => '1');
            else
                r3_m <= unsigned(dda(17 downto 8));
            end if;
            -- R3 -> R4
            v_d := signed(resize(r3_m, 12)) - signed(resize(p_thr, 12));
            if v_d < 0 then r4_pos <= (others => '0'); else r4_pos <= unsigned(v_d(10 downto 0)); end if;
            r4_m <= r3_m;
            -- R4 -> R5
            v_sh := shift_left(resize(r4_pos, 19), to_integer(p_k));
            r5_x <= v_sh(9 downto 0);
            if v_sh(18 downto 10) /= 0 then r5_sat <= '1'; else r5_sat <= '0'; end if;
            r5_m <= r4_m;
            -- R5 -> R6: raw key
            v_y := resize(r5_x, 11);
            if p_half = '1' then
                v_y := v_y + shift_right(r5_x, 1);
            end if;
            if r5_sat = '1' or v_y(10) = '1' then
                v_y := to_unsigned(1023, 11);
            end if;
            r6_key <= v_y(9 downto 0);
            r6_m   <= r5_m;
            -- R6 -> R7: DETAIL gate, cut mute, invert
            if r6_gate < r6_key then v_k := r6_gate; else v_k := r6_key; end if;
            if mute = '1' then v_k := (others => '0'); end if;
            if p_inv = '1' then
                r_key <= not v_k;
            else
                r_key <= v_k;
            end if;
            r_matte <= r6_m;
        end if;
    end process p_rk;

    key   <= r_key;
    matte <= r_matte;

    --------------------------------------------------------------------------
    -- column processor: P_k = (V_k, S_(k-1)), one column ahead of the DDA
    --------------------------------------------------------------------------
    p_cp : process(clk)
        variable v_col : unsigned(C_COLB + 1 downto 0);
        variable v_k   : unsigned(C_COLB + 1 downto 0);
        variable v_b   : unsigned(9 downto 0);
        variable v_lp  : signed(27 downto 0);
        variable v_v   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            rnd_mreq <= '0';
            -- 11 x 16 multiplier, two stages (operand split into bytes)
            mu_lo <= mu_x * signed(resize(mu_y(7 downto 0), 9));
            mu_hi <= mu_x * signed(resize(mu_y(15 downto 8), 9));
            mu_p  <= resize(mu_lo, 28) + shift_left(resize(mu_hi, 28), 8);

            case cs is
                when CS_IDLE | CS_RA =>
                    -- CS_RA = next column of the pre-line pass; IDLE issues on a trigger
                    v_k := cp_k;
                    if cs = CS_IDLE then
                        if cp_line = '1' then
                            v_k    := (others => '0');
                            cp_pre <= '1';
                        else
                            cp_pre <= '0';
                        end if;
                    end if;
                    if v_k >= resize(g_c, C_COLB + 2) then
                        v_col := resize(g_c - 1, C_COLB + 2);
                    else
                        v_col := v_k;
                    end if;
                    cp_k      <= v_k;
                    cp_col    <= v_col(C_COLB - 1 downto 0);
                    rnd_maddr <= rv_r0 & v_col(C_COLB - 1 downto 0);
                    if cs = CS_RA or cp_line = '1' or
                       (cp_go = '1' and cp_k <= resize(g_c, C_COLB + 2)) then
                        rnd_mreq <= '1';
                        cs <= CS_RB;
                    end if;
                when CS_RB =>
                    rnd_mreq  <= '1';
                    rnd_maddr <= rv_r1 & cp_col;
                    cs <= CS_W2;
                when CS_W2 =>
                    cs <= CS_W3;
                when CS_W3 =>
                    cs <= CS_CA;
                when CS_CA =>
                    cp_a <= m_rq(11 downto 2);
                    cs   <= CS_CB;
                when CS_CB =>
                    v_b  := m_rq(11 downto 2);
                    mu_x <= signed(resize(v_b, 11)) - signed(resize(cp_a, 11));
                    mu_y <= resize(rv_fy, 16);
                    cs   <= CS_L0;
                when CS_L0 =>
                    cs <= CS_L1;
                when CS_L1 =>
                    cs <= CS_L2;
                when CS_L2 =>
                    v_lp := shift_right(mu_p + 128, 8);
                    v_v  := signed(resize(cp_a, 12)) + v_lp(11 downto 0);
                    cp_v <= unsigned(v_v(9 downto 0));     -- lerp stays within [a, b]
                    cs   <= CS_S0;
                when CS_S0 =>
                    mu_x <= signed(resize(cp_v, 11)) - signed(resize(cp_vprev, 11));
                    mu_y <= g_rc;
                    cs   <= CS_S1;
                when CS_S1 =>
                    cs <= CS_S2;
                when CS_S2 =>
                    cs <= CS_S3;
                when CS_S3 =>
                    cp_vprev <= cp_v;
                    if cp_k = 0 then
                        v0_line <= cp_v;
                    else
                        pn_v <= cp_v;
                        pn_s <= resize(shift_right(mu_p, 8), 18);
                    end if;
                    cp_k <= cp_k + 1;
                    if cp_pre = '1' and cp_k = 0 then
                        cs <= CS_RA;
                    else
                        cs <= CS_IDLE;
                    end if;
            end case;
        end if;
    end process p_cp;

    --------------------------------------------------------------------------
    -- stats + testbench taps
    --------------------------------------------------------------------------
    stats.energy <= f_energy;
    stats.valid  <= not seed_pend;
    stats.cut    <= mute;
    stats.width  <= g_w;
    stats.height <= g_h;

    dbg_m_we   <= m_we;
    dbg_m_addr <= resize(m_wa, 10);
    dbg_m_data <= m_wd;
    dbg_r_we   <= ref_we;
    dbg_r_addr <= resize(ref_wa, 12);
    dbg_r_data <= ref_wd;
    dbg_upd    <= f_upd;

end architecture rtl;
