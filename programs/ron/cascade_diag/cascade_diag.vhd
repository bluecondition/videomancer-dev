-- Author: bluecondition

-- cascade_diag.vhd  (v2.1.2 -- 126/48 coupled discs, line-buffer mark render;
--   diag fork of programs/ron/cascade -- separate program_id)
--
-- HYPNOS CASCADE: concentric black/white discs that PUSH each other.  Move the
-- centre (K1/K2) and the innermost disc leads; each outer ring holds still until
-- the one inside closes its slack, then is shoved -- a wave of motion travelling
-- outward ring by ring.  Rings never overlap (a band of the other colour always
-- survives) and never deform.  Catch-Up (S10) glides them home concentric.
--
--   RENDER -- 1bpp dual-bank LINE BUFFER of edge MARKS.  The span engine
--     writes each disc's two scanline edges as single-bit marks; the pixel
--     path is ONE running-parity XOR over the marks.  The chain clamp keeps
--     the discs strictly nested, so edge-count parity == ring parity.
--
--   ENGINE -- runs a line EARLY: the pass for line N starts at line N-1's
--     active-video RISE (chain RAM, shared multiply and the sqrt unit are
--     all idle during active video).  E_LINE (the tracker seed) runs in
--     the hblank.
--
--   GRADIENT (K4/S9) -- the marks carry an enter/leave DIRECTION bit, so
--     the pixel path counts the true ring index; every ring carries its own
--     ramp (re-anchored at each true disc edge), K4 sets the fineness and
--     S9 picks Steps/Smooth.
--
--   VIDEO (S11/P12) -- the ring field REFRACTS the input: a one-sided
--     horizontal push of 0..53 px through a 64-tap EBR delay line, P12 =
--     depth.  Same latency (6) and sync taps as ring mode.
--
--   OUTER field -- incremental integer-radius tracker centred on the LAST
--     modelled disc's centre, seamed exactly onto it.
--
--   CHAIN -- coupled dead-zone chain in PIXEL units, C_NR entries, with an
--     OCTAGONAL coupling clamp: the slack is ~87% of the spacing in every
--     drag direction, so rings nearly touch before yielding.
--
-- v0.9 (2026-07-31), both from user reports:
--   * RING COUNT 24 -> 96 (HD) / 48 (SD).  Ring count is pure throughput,
--     not fit -- see the C_NR budget note below.  The nest now reaches the
--     screen corners for any spacing >= 12 px instead of >= 46, so the
--     cascade wave travels the whole picture rather than a centre disc.
--   * CATCH-UP now actually centres.  The settle pull was d + floor(-d/8),
--     which is a NO-OP for -7 <= d <= -1 -- and a drag right/down makes the
--     rings lag LEFT, i.e. d NEGATIVE, so the common case froze every ring
--     up to 7 px off its parent and the nest never closed.
--   * The radius clamp no longer piles rings on one radius (that dropped an
--     odd number of edge marks half the time and inverted the screen); the
--     modelled count adapts per frame to what fits under C_RCL.
--
-- v2.1 (2026-10-09, diag): S9 MOIRE -- the rings XOR a half-phase copy of
--   themselves around the outer centre (doubles the rings at rest; moire
--   flares while the cascade has them offset).  K4 setting 2 = 4 steps
--   (v2.0.3).  Built with ROUTER1 (router2 can't close this netlist).
--
-- v2.0 (2026-10-09, diag -- LEAN PASS; Ron: keep the look, cascade,
--   catch-up and gradients; Zoom/Video droppable; S9 free):
--   * ONE SERIAL MULTIPLIER (shift-add, 14 steps, product at T+15) replaces
--     TWO LUT array multipliers (~1060 LUT): the vblank 14x14 was merged into
--     the engine's (v1.9), then the engine's 15x15 became serial (v2.0).  The
--     span engine is re-pipelined around it (phase 4: stage ring k-2's edges,
--     root ring k-1, multiply ring k; marks at phase 5; 15*(nreff+2) clocks),
--     E_LINE re-scheduled (89 clocks, the two roots overlap the products), the
--     vblank sequencer issues one product at a time and stalls on busy.
--     Bit-exact vs v1.9 in four 720p sims.  LINE BUDGET 15*(nreff+2)+89:
--     HD 2009/2200, SD 839/858.  E_LINE no longer fits the image-tester's
--     fake 64-clk hblank -- sim with a 280-clk hblank copy of the tool
--     (.cascade_diag_work/vit_hb).
--   * K5 WIDTH: band balance, exact circles (every even disc's radius moves
--     by delta = f*s; slack from the narrowest band; outer field per-parity
--     thresholds).  Eased, deadbanded at the centre, off in Video mode.
--   * LEFT-EDGE SEED: the per-ring phase at x=0 is seeded from the band's own
--     off-screen left edge ((r0 - r_L)*Q1, r_L = 2nd root per line), so the
--     edge band no longer "comes to a point" during catch-up.
--   * K4 = 4 settings (hard / 2 / 8 grays / smooth); S9 Smooth folded in, S9
--     free.  K3 x144 decode -> shift-adds.
--
-- v1.8 (2026-10-09, diag; Ron: manual control only -- no auto-animation):
--   * v1.8.1: S7 Glide 2x quicker (C_GSH 6 -> 5: ~1/32 of the way per field,
--     a full-screen move settles in ~1.6 s instead of ~3.2 s).
--   * K1/K2 range +-0.625 W/H: the centre goes ~12% past each edge at most
--     (was +-2 W/H -- the rings got lost off-screen).
--   * P12 in RINGS mode = ZOOM: up to 3 octaves (x8) subtracted from K3's
--     log-spacing -> the rings expand from the centre (clamped at 2048 px).
--     (An exact band-WIDTH control was built first and did not fit at
--     ~101% LC; shelved in .cascade_diag_work/v18_width_shelved/.)
--   * P12 in VIDEO mode = wave depth, 8 levels, up to 53 px (was 4 zones,
--     18 px); delay line 32 -> 64 taps (same 2 EBR).
--
-- v1.7 (2026-10-08, Tier-1 finalization, no look change; HW-verified):
--   * ring luma in studio range 64..942 (was 0..1023: codes 0/1023 are the
--     BT.656/1120 sync-word codes);
--   * blanking gate covers BOTH modes (ring mode emitted the last pixel's
--     key through every hblank);
--   * the dead multi-ring smooth-fade phase (old S11 Smooth: ph_acc, Q
--     decode + multiply, line seed) removed; its sequencer/E_LINE slots
--     are kept as no-ops so the engine timing is unchanged.
--
-- No CORDIC, no cos/sin ROM, no per-pixel multiply anywhere; every edge is an
-- exact integer circle.  C_NR MUST stay even (mark parity == k parity).
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.core_config_pkg.all;
use work.video_stream_pkg.all;

architecture cascade_diag of program_top is

    constant C_LATENCY : integer := 6;    -- RINGS: x/read 1 + parity 1 + g2 1 + merge 1 + key 1 + out 1
    -- VIDEO path: the refraction is a ONE-SIDED push (read 1 + disp back,
    -- disp = 0..18; v1.8: 0..53).  A constant displacement term is an invisible whole-
    -- picture shift, so 0..18 one-sided looks IDENTICAL to +-9 centred --
    -- and it lands the video pipeline at delay 6 = C_LATENCY exactly, so
    -- BOTH modes share the ring mode's PROVEN sync taps.  (v1.4's deeper
    -- 15-cycle video-only tap was the ONE structural difference from every
    -- hardware-validated build, and hd_hdmi showed green + noise lines.)
    constant C_MID     : unsigned(9 downto 0) := to_unsigned(512, 10);   -- neutral chroma

    constant C_DB    : integer := 14;                                    -- detent deadband
    constant C_GSH   : integer := 5;                                     -- glide one-pole shift (v1.8.1: 6 -> 5, 2x quicker)

    -- CASCADE chain: per-ring centre offsets.  The coupling clamp is an
    -- OCTAGON (per-axis +/-A AND 45-degree +/-C = A*sqrt2): a ring yields
    -- only once the one inside has closed ~90% of the spacing, i.e. the
    -- minimum edge gap at the push point is ~10% of the spacing in ANY drag
    -- direction (a plain box at this slack would overlap on diagonals).
    -- A_px = min(s - s/8, s - s/16 - s/32 - 3) (~0.875*s) keeps every
    -- adjacent edge pair > 1 px apart on any scanline (mark writes never
    -- collide) and nesting strict.  NO accumulated cap -- the wave travels
    -- the chain.  C_NR MUST stay even (mark parity == k parity).
    --
    -- RING COUNT BUDGET (v0.9: 96 rings at HD, 48 at SD -- v0.8 was 24/24):
    --   * LC is FLAT in C_NR (7080 at 24, 7091 at 96 -- measured): the chain
    --     scan, the span pass and the pixel path are all serialized or
    --     constant-cost, and the offsets live in EBR.  Ring count is a
    --     THROUGHPUT question, never a fit one.
    --   * The per-LINE pass is E_SPAN (15 slots/ring, starts at the active
    --     rise and spills into the hblank via l_pend) + E_LINE (46 slots),
    --     and must be back in E_IDLE before the NEXT active rise or that
    --     line's pass never launches:
    --         15*C_NR + 56  <=  clocks_per_line
    --     HD  prog_clk 74.25 MHz, 2200 clk/line  ->  142 rings
    --     SD  prog_clk 13.5  MHz,  858 clk/line  ->   53 rings
    --     (SD is 858, NOT 1716: prog_clk = i_vid_dec_clk, the 13.5 MHz
    --     decoder pixel clock -- the 27 MHz PLL feeds the ENCODER only, and
    --     the 27 MHz build constraint is pure margin.)
    --     Sim raster is W/decimation + 64, so decimation 1 (1984 clk) is
    --     needed to sim the HD count; decimation 4 (544) holds only 32.
    --   * Chain words stay 15-bit: |e[k]| is bounded by the CENTRE's own
    --     travel (~10.2k worst case at W = 4095), never by k*A, so the 640
    --     slack cap does not have to scale with C_NR.
    --   * Index words are 7-bit (t_kl is mod 128), so C_NR <= 126.
    function f_nr return integer is
    begin
        if C_ENABLE_HD then return 126; else return 48; end if;
    end function;
    constant C_NR    : integer := f_nr;                                  -- modelled ring slots
    -- Settle rate: the pull is ceil(|d| / 2^C_SET) per frame.  Catch-Up is a
    -- WAVE -- ring k can only close once the one inside it has -- so the time
    -- to reach concentric is ~(7 / 2^(C_SET-3)) frames PER RING, i.e. it
    -- scales with C_NR.  Measured (spacing 30, model): C_SET 3 = 168 frames
    -- at 24 rings but 672 (11 s) at 96; C_SET 2 = 294 (4.9 s); C_SET 1 = 105
    -- (1.8 s).  Dropped 3 -> 2 with the ring count so 96 rings still glide
    -- home in about the same few seconds v0.8 took at 24.
    constant C_SET   : integer := 2;                                     -- settle glide shift (toward inner)

    constant C_RCL   : integer := 4095;                                  -- px clamp: nest disc radii
    constant C_OCL   : integer := 4095;                                  -- outer-centre offset clamp (px)

    ----------------------------------------------------------------------
    -- exp2 mantissa ROM: round(1024 * 2^(f/64)), f = 0..63  -> [1024,2048)
    -- (used by the K3 frequency decode AND its reciprocal, the pixel spacing)
    ----------------------------------------------------------------------
    type t_mant is array(0 to 63) of unsigned(10 downto 0);
    constant C_MANT : t_mant := (
        to_unsigned(1024,11), to_unsigned(1035,11), to_unsigned(1046,11), to_unsigned(1058,11),
        to_unsigned(1069,11), to_unsigned(1081,11), to_unsigned(1093,11), to_unsigned(1105,11),
        to_unsigned(1117,11), to_unsigned(1129,11), to_unsigned(1141,11), to_unsigned(1154,11),
        to_unsigned(1166,11), to_unsigned(1179,11), to_unsigned(1192,11), to_unsigned(1205,11),
        to_unsigned(1218,11), to_unsigned(1231,11), to_unsigned(1244,11), to_unsigned(1258,11),
        to_unsigned(1272,11), to_unsigned(1286,11), to_unsigned(1300,11), to_unsigned(1314,11),
        to_unsigned(1328,11), to_unsigned(1342,11), to_unsigned(1357,11), to_unsigned(1372,11),
        to_unsigned(1387,11), to_unsigned(1402,11), to_unsigned(1417,11), to_unsigned(1433,11),
        to_unsigned(1448,11), to_unsigned(1464,11), to_unsigned(1480,11), to_unsigned(1496,11),
        to_unsigned(1512,11), to_unsigned(1529,11), to_unsigned(1545,11), to_unsigned(1562,11),
        to_unsigned(1579,11), to_unsigned(1596,11), to_unsigned(1614,11), to_unsigned(1631,11),
        to_unsigned(1649,11), to_unsigned(1667,11), to_unsigned(1685,11), to_unsigned(1704,11),
        to_unsigned(1722,11), to_unsigned(1741,11), to_unsigned(1760,11), to_unsigned(1779,11),
        to_unsigned(1798,11), to_unsigned(1818,11), to_unsigned(1838,11), to_unsigned(1858,11),
        to_unsigned(1878,11), to_unsigned(1898,11), to_unsigned(1919,11), to_unsigned(1940,11),
        to_unsigned(1961,11), to_unsigned(1983,11), to_unsigned(2004,11), to_unsigned(2026,11));

    ----------------------------------------------------------------------
    -- chain offset RAM (PIXEL units).  One read port (chain scan / span
    -- engine, time-disjoint) + one write port (chain scan).  The span
    -- engine reads both axes in parallel, so no separate ring table exists.
    -- |e[k]| < 16383 by the 640-px slack cap -> 15-bit words.
    ----------------------------------------------------------------------
    type t_ring is array(0 to C_NR - 1) of signed(14 downto 0);
    signal ram_ex, ram_ey : t_ring := (others => (others => '0'));
    signal ea_addr, ew_addr : integer range 0 to C_NR - 1 := 0;
    signal ew_wex, ew_wey : std_logic := '0';
    signal ew_dinx, ew_diny : signed(14 downto 0) := (others => '0');
    signal doa_ex, doa_ey : signed(14 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- 1bpp mark line buffers, one EBR per bank.  Engine writes the NEXT
    -- line's edge marks into bank s_lb_eng while bank s_lb_disp is read
    -- (and cleared 2 px behind the read) by the pixel path.
    ----------------------------------------------------------------------
    type t_lb is array(0 to 2047) of std_logic_vector(1 downto 0);
    signal lb0, lb1 : t_lb := (others => (others => '0'));
    signal lb_ra  : unsigned(10 downto 0) := (others => '0');
    signal lbq0, lbq1 : std_logic_vector(1 downto 0) := "00";
    signal lb0_we, lb1_we : std_logic;
    signal lb0_a,  lb1_a  : unsigned(10 downto 0);
    signal lb0_d,  lb1_d  : std_logic_vector(1 downto 0);
    signal s_lb_eng, s_lb_disp : std_logic := '0';

    -- registered mark-write channel (left edge at slot 7; the right edge is
    -- staged in lbp_* and issued one cycle later -- never the same cycle)
    signal lbm_we : std_logic := '0';
    signal lbm_a  : unsigned(10 downto 0) := (others => '0');
    signal lbm_d  : std_logic_vector(1 downto 0) := "01";      -- "01" enter / "11" leave
    signal lbp_we : std_logic := '0';
    signal lbp_a  : unsigned(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_cx_k, s_cy_k : signed(10 downto 0) := (others => '0');      -- (Kn-512), deadbanded
    signal s_k3_per : unsigned(9 downto 0) := to_unsigned(430, 10);
    signal s_k4_gr  : unsigned(9 downto 0) := (others => '0');
    signal s_gn     : unsigned(1 downto 0) := "00";  -- K4 GRADIENT: hard / 2 / 8 grays / smooth
    signal s_qp1    : unsigned(14 downto 0) := (others => '0');  -- stepped step: 2^16/s (per-ring)
    signal s_h1s    : signed(13 downto 0) := (others => '0');    -- 2^(16-sft), pre-decoded
    signal s_sm1    : unsigned(11 downto 0) := to_unsigned(41, 12);  -- spacing - 1 (m wrap)
    signal s_glx, s_gly : std_logic := '1';          -- GLIDE (S7) -- default on
    signal s_dyn    : std_logic := '0';              -- cascade motion (S8)
    signal s_home   : std_logic := '0';              -- Catch-Up (S10): glide home
    signal s_vsat   : std_logic := '0';              -- S11: Rings / Video (refraction)
    -- P12 (v1.8): Rings mode = ZOOM (log-domain spacing offset), Video = wave depth
    signal s_p12    : unsigned(9 downto 0) := (others => '0');   -- latched at vsync
    signal s_lfb    : signed(15 downto 0) := to_signed(-462, 16);-- K3 log-spacing, before zoom
    signal s_zm     : unsigned(7 downto 0) := (others => '0');   -- zoom, Q6 octaves (0..191 = x8)
    -- K5 WIDTH (v1.9): band balance.  Every EVEN disc's radius moves by
    -- delta = f*s (f = (K5-512)/512, deadbanded at the centre), so bands
    -- alternate s+delta / s-delta; the cascade slack is derived from the
    -- NARROWEST band, and the outer field uses per-parity thresholds.
    signal s_k5     : unsigned(9 downto 0) := to_unsigned(512, 10);  -- raw, latched at vsync
    signal s_k5f    : unsigned(12 downto 0) := to_unsigned(4096, 13);-- eased (diff>>3 per run), Q3
    signal s_fd     : signed(13 downto 0) := (others => '0');   -- k5f - centre (Q3)
    signal s_fab    : unsigned(12 downto 0) := (others => '0'); -- |fd| (f = fd/4096)
    signal s_fdz    : std_logic := '1';                         -- inside the centre deadband
    signal s_dabs   : unsigned(12 downto 0) := (others => '0'); -- |delta| = |f|*s, capped s-4
    signal s_dneg   : std_logic := '0';                         -- delta < 0 (thinner white)
    signal s_seff   : unsigned(12 downto 0) := to_unsigned(42, 13); -- narrowest band = s - |delta|

    ----------------------------------------------------------------------
    -- runtime raster measurement
    ----------------------------------------------------------------------
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_lcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_ilace : std_logic := '0';
    signal s_fpar  : std_logic := '0';
    signal s_avid_q : std_logic := '0';

    ----------------------------------------------------------------------
    -- per-frame resolved terms
    ----------------------------------------------------------------------
    signal s_cx, s_cy : signed(13 downto 0) := (others => '0');          -- glided centre (px)
    signal s_cxs, s_cys : signed(21 downto 0) := (others => '0');        -- glide accum, Q8
    signal s_lf   : signed(15 downto 0) := to_signed(-704, 16);          -- freq log2, Q6
    signal s_nl   : unsigned(15 downto 0) := to_unsigned(704, 16);       -- -Lf (spacing log)
    signal s_mant : unsigned(10 downto 0) := to_unsigned(1024, 11);      -- spacing mantissa
    signal s_sip  : integer range 0 to 15 := 8;                          -- spacing shift
    signal s_fnum : unsigned(10 downto 0) := to_unsigned(1024, 11);      -- freq mantissa
    signal s_sft  : integer range 0 to 15 := 8;                          -- freq shift (phi Q8)
    signal s_spac : unsigned(12 downto 0) := to_unsigned(42, 13);        -- ring spacing, px
    signal s_araw : unsigned(11 downto 0) := (others => '0');            -- s - s/8 (pre-cap)
    signal s_acap : unsigned(12 downto 0) := (others => '0');            -- collision/storage cap
    signal s_apx  : unsigned(11 downto 0) := to_unsigned(37, 12);        -- axis slack A, px
    signal s_cpx  : unsigned(12 downto 0) := to_unsigned(52, 13);        -- 45-deg slack C = A*sqrt2, px

    -- vblank sequencer + one shared multiplier (centre resolve + px conversion
    -- + the frame-constant tracker terms)
    signal s_seq   : unsigned(4 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_ma    : signed(13 downto 0) := (others => '0');
    signal s_mb    : signed(13 downto 0) := (others => '0');
    -- v2.0: the sequencer / frame tail request the ONE serial multiplier
    -- (in p_engine) with s_mreq + s_ma/s_mb, and stall until it is done.
    signal s_mreq  : std_logic := '0';
    -- REGISTERED step-enable for the sequencer and frame tail (the combined
    -- avid / engine-idle / multiplier-busy / request cone feeding every
    -- slot register's enable was the HD critical path).  A slot that issues
    -- a request also holds it low, so the 1-cycle-late decision can never
    -- step past a pending product.
    signal seq_go  : std_logic := '0';

    -- CASCADE chain FSM (once per frame in vblank, 4 clocks per ring)
    signal s_cxp, s_cyp : signed(13 downto 0) := (others => '0');  -- prev inner centre (px)
    signal s_chain_cnt : integer range 0 to C_NR := C_NR;
    signal s_chain_run : std_logic := '0';
    signal s_chain_wt  : std_logic := '0';                        -- armed, waiting for the decode
    signal s_chain_ph  : integer range 0 to 13 := 0;              -- serialized octagon pipeline
    signal s_doax, s_doay : signed(14 downto 0) := (others => '0');
    signal s_settle    : std_logic := '0';
    signal s_eprevx, s_eprevy : signed(14 downto 0) := (others => '0');
    signal s_dw    : signed(15 downto 0) := (others => '0');       -- shared step work reg
    signal s_clb, s_clbn : signed(15 downto 0) := (others => '0'); -- clamp bound, pre-registered
    signal s_qdx, s_qdy : signed(12 downto 0) := (others => '0');  -- axis-clamped step
    signal s_pu,  s_pv  : signed(13 downto 0) := (others => '0');  -- rotated, clamped step
    signal s_mvx, s_mvy : signed(9 downto 0) := (others => '0');
    signal s_chain_d : std_logic := '0';
    -- local registered "blanking" + slot-4 strobe for the chain FSM (the raw
    -- core avid and the s_seq compare in its enable cones were routing-
    -- critical; the chain only ever runs in vblank anyway)
    signal ch_blank : std_logic := '0';
    signal ch_go4   : std_logic := '0';

    -- ADAPTIVE ring count.  Ring k's radius is (k+1)*spacing, and the span
    -- engine clamps radii at C_RCL -- so at coarse spacings every ring past
    -- C_RCL/spacing PILES UP at the same radius and they all write their edge
    -- mark to the SAME line-buffer cell.  The cell holds one mark, so the pile
    -- collapses to a single toggle: (pile-1) marks vanish, and when that count
    -- is odd (54% of the Period range) the mark parity INVERTS for the rest of
    -- the line -- a hard vertical seam with the screen colour-flipped beyond
    -- it.  (Only reachable on HD with the centre knob near an extreme, which
    -- is why v0.8 shipped with it.)  Fix: model only the rings that actually
    -- fit under C_RCL.  s_nreff is that count, forced EVEN (mark parity == k
    -- parity) and >= 2, recomputed per frame during the chain scan; the outer
    -- field seams onto ring s_nreff-1 instead of ring C_NR-1.  It also makes
    -- the per-line span cost scale with the spacing rather than with C_NR.
    signal s_racc  : unsigned(13 downto 0) := (others => '0');     -- (k+1)*spacing, saturating
    signal s_nrw   : unsigned(6 downto 0) := to_unsigned(C_NR, 7); -- working count during the scan
    signal s_nreff : unsigned(6 downto 0) := to_unsigned(C_NR, 7); -- published, frame-stable
    signal s_ocap  : std_logic := '0';                             -- capture e[] into the outer centre
    signal s_codd  : std_logic := '1';                             -- s_chain_cnt is odd (toggle, not a mod)

    ----------------------------------------------------------------------
    -- frame-constant tracker terms (small p_frame tail after the chain,
    -- reusing its shared multiplier)
    ----------------------------------------------------------------------
    signal s_chain_q : std_logic := '0';                          -- chain_run delayed (edge)
    signal s_tlrun : std_logic := '0';
    signal s_tlph  : integer range 0 to 3 := 0;
    signal s_cv_done : std_logic := '0';                          -- pulse: frame terms ready
    signal s_dy0   : signed(14 downto 0) := (others => '0');      -- first active line's dy
    signal s_dx0   : signed(13 downto 0) := (others => '0');      -- dx0' = line-start dx - exo
    signal s_q0    : signed(15 downto 0) := (others => '0');      -- 2*dx0'+1
    signal s_dx0q  : unsigned(26 downto 0) := (others => '0');    -- dx0'^2

    ----------------------------------------------------------------------
    -- SPAN + LINE-SEED engine and the outer-field r^2 band tracker.
    ----------------------------------------------------------------------
    type t_est is (E_IDLE, E_CLEAR, E_SPAN, E_LINE);
    signal e_st : t_est := E_IDLE;
    signal e_cnt : integer range 0 to C_NR + 1 := 0;              -- ring group counter (2 drain groups)
    signal e_ph  : integer range 0 to 14 := 0;                    -- span slot (15-slot cadence)
    signal l_ph  : integer range 0 to 98 := 0;                    -- line-seed slot
    signal e_first : std_logic := '0';                            -- cv_done pass: chain SPAN->LINE
    -- per-group ring-count flags, registered at phase 0 (the 7-bit compares
    -- against s_nreff in every phase's enable cone were routing-critical)
    signal f_lt, f_mid, f_ge2, f_last : std_logic := '0';
    signal cl_hit  : std_logic := '0';                            -- E_CLEAR exit, registered
    signal l_pend  : std_logic := '0';                            -- avid fell while SPAN still busy
    signal cl_cnt  : unsigned(11 downto 0) := (others => '0');    -- E_CLEAR address counter
    signal cl_end  : unsigned(11 downto 0) := (others => '0');    -- E_CLEAR stop (W+2, latched)
    -- v2.0 SERIAL MULTIPLIER (replaces a 15x15 LUT array, ~570 LUT): signed
    -- 15-bit multiplicand x UNSIGNED 14-bit multiplier, shift-and-add, one
    -- 17-bit add per clock.  Loaded at slot T, 14 steps, product readable at
    -- slot T+15 -- exactly one 15-slot span group.  Every user is arranged
    -- so the multiplier operand is non-negative (squares use |x|).
    signal mu_m    : signed(14 downto 0) := (others => '0');      -- multiplicand
    signal mu_a    : signed(15 downto 0) := (others => '0');      -- high product bits
    signal mu_q    : unsigned(13 downto 0) := (others => '0');    -- multiplier -> low bits
    signal mu_cnt  : integer range 0 to 14 := 0;
    signal mu_busy : std_logic := '0';
    -- next-load operand registers: every load comes from these (the operand
    -- arithmetic + load-gating compares in front of mu_* were the HD
    -- critical path).  Request at T -> load at T+1 -> product at T+16.
    signal nx_m    : signed(14 downto 0) := (others => '0');
    signal nx_q    : unsigned(13 downto 0) := (others => '0');
    signal nx_ld   : std_logic := '0';
    signal m_p     : signed(29 downto 0);                         -- product view {mu_a, mu_q}

    signal sp_dy   : signed(14 downto 0) := (others => '0');      -- NEXT line's dy (from inner centre)
    signal sp_ey   : signed(14 downto 0) := (others => '0');
    signal sp_rr   : unsigned(11 downto 0) := (others => '0');    -- ring radius R (accumulated)
    signal sp_rq   : unsigned(11 downto 0) := (others => '0');    -- R +- |delta| on even discs (WIDTH)
    signal sp_ady  : unsigned(14 downto 0) := (others => '0');    -- |dy| pre-clamp (<= 24574)
    signal sp_emp  : std_logic := '0';                            -- |dy| >= R -> empty span
    signal sp_ex   : signed(14 downto 0) := (others => '0');      -- ex of the ring in front-end
    signal sp_exc  : signed(15 downto 0) := (others => '0');      -- cx + ex (RAW screen x of centre)
    signal sp_excq0, sp_excq1 : signed(15 downto 0) := (others => '0'); -- centre x, ring k-1 / k-2
    signal sp_empq0, sp_empq1, sp_empq2 : std_logic := '0';
    signal sp_wl, sp_wr1 : signed(15 downto 0) := (others => '0');    -- staged edges, raw x (slot 6)

    -- per-pass line-start ring index (C_NR minus the discs overhanging the
    -- left edge): written by the pass (working/_done), latched live at the
    -- line's avid rise.  (The hit test is the counter itself: k < C_NR.)
    signal m_kinit      : unsigned(6 downto 0) := to_unsigned(C_NR, 7);
    signal m_kinit_done : unsigned(6 downto 0) := to_unsigned(C_NR, 7);
    signal m0_w, m0_done : std_logic := '0';          -- a mark lands at x=0 (snap the seed)
    -- v1.9 LEFT-EDGE SEED: the nearest disc edge at or left of x=0 on the
    -- NEXT line (= the left boundary of the band x=0 sits in).  E_LINE roots
    -- its outer-centre radius r_L and seeds the per-ring phase with
    -- (r0 - r_L)*Q1 -- exactly how every on-screen band starts at its own
    -- left edge -- instead of the outer-centre phase, which is wrong for an
    -- offset nest (the edge band "came to a point" during catch-up).
    signal xl_w, xl_done   : signed(15 downto 0) := (others => '0');
    signal xlv_w, xlv_done : std_logic := '0';
    signal xc_r   : signed(15 downto 0) := (others => '0');       -- candidate, registered (timing)
    signal xcv_r  : std_logic := '0';
    signal le_r0  : unsigned(13 downto 0) := (others => '0');     -- r0, kept across the 2nd root
    signal le_dl  : signed(14 downto 0) := (others => '0');       -- r0 - r_L
    -- v2.0.1: E_LINE's own copy of the spacing (s_spac's fanout made the
    -- ring-index correction the HD Analog critical path) + the correction
    -- split into 2 clocks per step (candidates, then select)
    signal s_spl   : unsigned(12 downto 0) := to_unsigned(42, 13);
    attribute keep : boolean;
    attribute keep of s_spl : signal is true;
    -- v2.0.2: per-consumer copies of the per-frame WIDTH / spacing registers
    -- (one copy feeding the sequencer, the span engine AND the pixel path
    -- across the die was HD HDMI's recurring critical path)
    signal s_dabs_e, s_dabs_p : unsigned(12 downto 0) := (others => '0');
    signal s_dneg_e, s_dneg_p : std_logic := '0';
    signal s_seff_p : unsigned(12 downto 0) := to_unsigned(42, 13);
    signal s_shalf_p : unsigned(11 downto 0) := to_unsigned(21, 12);   -- s/2 (S9 moire)
    attribute keep of s_shalf_p : signal is true;
    signal s_spac_e : unsigned(12 downto 0) := to_unsigned(42, 13);
    attribute keep of s_dabs_e : signal is true;
    attribute keep of s_dabs_p : signal is true;
    attribute keep of s_dneg_e : signal is true;
    attribute keep of s_dneg_p : signal is true;
    attribute keep of s_seff_p : signal is true;
    attribute keep of s_spac_e : signal is true;
    signal le_dm, le_dp : signed(14 downto 0) := (others => '0');

    -- ONE 1-step/cycle sqrt unit (15-slot ring cadence; also roots r0).  The
    -- bit mask is a SHIFT REGISTER (>>2 per step), so a step is just an OR +
    -- one 27-bit subtract/compare -- short single-cycle path, 14 steps.
    signal sqa_num, sqa_res, sqa_one : unsigned(26 downto 0) := (others => '0');
    signal sqa_run : std_logic := '0';

    -- line-seed temporaries
    signal le_dyp  : signed(13 downto 0) := (others => '0');      -- dy' = dy - eyo
    signal le_r0sq : unsigned(26 downto 0) := (others => '0');    -- r0^2
    signal le_k0p  : unsigned(24 downto 0) := (others => '0');    -- r0 * fnum

    -- band tracker state (seeded per line by E_LINE, stepped per active pixel)
    signal t_kl   : unsigned(6 downto 0) := (others => '0');      -- ring index mod 128 (pixel path
                                                                  -- needs only k mod 2*maxwidth)
    signal t_k    : unsigned(11 downto 0) := (others => '0');     -- ring index (E_LINE-local)
    signal t_q    : signed(15 downto 0) := (others => '0');       -- 2*dx'+1 (r^2 step)
    signal su_m   : unsigned(11 downto 0) := (others => '0');     -- r mod spacing (ring phase)
    signal k_ld1, k_ld2 : unsigned(6 downto 0) := (others => '0'); -- t_kl, delayed to merge
    signal c_e1, c_o1   : std_logic := '0';                         -- outer-field WIDTH tests
    signal h1, h2       : std_logic := '0';                         -- S9: past half a band (outer centre)

    -- SMOOTH-fade machinery: a spacing-1 SUB-TRACKER (same candidate-arm
    -- structure, taps degenerate to +4/0/+2) detects every integer-radius
    -- crossing; a 16-bit phase accumulates +/-Q per crossing and its top
    -- bits fold straight into luma.  Seeded per line by E_LINE.
    signal su_u    : signed(16 downto 0) := (others => '0');      -- r^2 - rint^2
    signal su_qmd  : signed(17 downto 0) := (others => '0');      -- q - D1
    signal su_qpdm : signed(17 downto 0) := (others => '0');      -- q + D1 - 2
    signal ph1_acc : unsigned(15 downto 0) := (others => '0');    -- PER-RING phase (period 1 ring;
                                                                  -- SNAPPED to 0 at every disc edge)
    signal ph1_f1  : unsigned(7 downto 0) := (others => '0');
    signal su_cr   : std_logic_vector(1 downto 0) := "00";        -- crossing dir, 1-cycle delayed
    signal st_v    : std_logic := '0';                            -- s_avid_p, delayed
    signal lb_lo   : signed(14 downto 0) := (others => '0');      -- B_lo maintained thru corrections
    signal le_d    : signed(14 downto 0) := (others => '0');      -- d0 = r0 - B_lo
    signal su_d1   : signed(15 downto 0) := (others => '0');      -- D1 = 2*r0+1 (seed)

    -- outer-field recentre (ring C_NR-1's converted offset), frame-stable
    signal s_exo, s_eyo : signed(12 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pixel-coordinate accumulators.  dx per pixel, dy per active line.
    ----------------------------------------------------------------------
    signal s_dx    : signed(14 downto 0) := (others => '0');
    signal s_dyb   : signed(14 downto 0) := (others => '0');
    signal s_dyc   : signed(14 downto 0) := (others => '0');
    signal s_avid_p : std_logic := '0';
    signal s_avid_p2 : std_logic := '0';
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- pixel pipeline: mark-parity path + disc-23 hit toggle + merge
    ----------------------------------------------------------------------
    signal cl_a1, cl_a2 : unsigned(10 downto 0) := (others => '0'); -- clear-behind addr pipe
    signal s_kx    : unsigned(6 downto 0) := (others => '0');     -- running ring index (stage 2)
    signal g2_hit : std_logic := '0';
    signal g2_k   : unsigned(6 downto 0) := (others => '0');

    signal r10_k    : unsigned(6 downto 0) := (others => '0');  -- ring index at the merge
    signal r10_t    : unsigned(7 downto 0) := (others => '0');  -- per-ring ramp at the merge
    signal r10_h    : std_logic := '0';                         -- S9 moire bit at the merge
    signal r12_key  : unsigned(7 downto 0) := (others => '0');
    signal s_out_y  : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S11 VIDEO: the incoming picture passes through UNTOUCHED in colour --
    -- the rings act on it only as REFRACTION (P12 Wave).  The saturation
    -- mod that used to live here (0..2x chroma about neutral) was dropped
    -- by the user once the refraction landed: the wave carries the mode.
    -- The video is NOT delayed to meet the field.  It used to go through a
    -- 30-bit BRAM delay line, which simulated pixel-exact but put noisy
    -- horizontal lines on real hardware -- silicon and GHDL do not agree on
    -- that inferred memory.  It is unnecessary anyway: the ring field is a
    -- SMOOTH pattern, so letting the gain lag the picture by a few pixels
    -- shifts the saturation pattern sideways by an invisible amount.  So the
    -- video takes its own 3-cycle path and the syncs are tapped to match.
    ----------------------------------------------------------------------
    type t_vd is array(0 to 63) of std_logic_vector(29 downto 0);
    signal s_vd    : t_vd := (others => (others => '0'));
    signal vd_wa   : unsigned(5 downto 0) := (others => '0');
    signal vd_ra   : unsigned(5 downto 0) := (others => '0');
    signal vd_q    : std_logic_vector(29 downto 0) := (others => '0');
    signal vd_hold : std_logic_vector(29 downto 0) := (others => '0');
    signal vd_tc   : unsigned(3 downto 0) := (others => '0');
    signal vd_px   : unsigned(5 downto 0) := (others => '0');  -- pixels written this line (sat 63)
    signal r13_y, r13_u, r13_v : unsigned(9 downto 0) := (others => '0');
    signal r14_y, r14_u, r14_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_u, s_out_v : unsigned(9 downto 0) := C_MID;
    -- S9 MOIRE (v2.1): XOR the rings with a half-phase copy of the rings
    -- around the outer centre.  At rest that DOUBLES the ring count; while
    -- the cascade has the rings offset (half-way points from the outer
    -- centre vs the true ring edges), moire flares and dissolves as they
    -- catch up.
    signal s_s9    : std_logic := '0';

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- chain offset RAM.  Read addr = chain scan (vblank) or the span engine
    -- (during the previous line's active video) -- time-disjoint.
    ------------------------------------------------------------------------
    ea_addr <= s_chain_cnt when s_chain_run = '1'
               else e_cnt when e_cnt < C_NR
               else 0;

    p_ram : process(clk)
    begin
        if rising_edge(clk) then
            if ew_wex = '1' then ram_ex(ew_addr) <= ew_dinx; end if;
            if ew_wey = '1' then ram_ey(ew_addr) <= ew_diny; end if;
            doa_ex <= ram_ex(ea_addr);
            doa_ey <= ram_ey(ea_addr);
        end if;
    end process p_ram;

    ------------------------------------------------------------------------
    -- mark line buffers.  Each bank: one sync write + one sync read port.
    -- E_CLEAR (vblank) wipes both; otherwise the engine bank takes mark
    -- writes and the display bank is cleared 2 px behind the pixel read.
    ------------------------------------------------------------------------
    lb_ra <= s_xcnt(10 downto 0);

    lb0_we <= '1'        when e_st = E_CLEAR
         else lbm_we     when s_lb_eng = '0'
         else s_avid_p2  when s_lb_disp = '0'
         else '0';
    lb0_a  <= cl_cnt(10 downto 0) when e_st = E_CLEAR
         else lbm_a               when s_lb_eng = '0'
         else cl_a2;
    lb0_d  <= "00" when e_st = E_CLEAR
         else lbm_d when s_lb_eng = '0'
         else "00";

    lb1_we <= '1'        when e_st = E_CLEAR
         else lbm_we     when s_lb_eng = '1'
         else s_avid_p2  when s_lb_disp = '1'
         else '0';
    lb1_a  <= cl_cnt(10 downto 0) when e_st = E_CLEAR
         else lbm_a               when s_lb_eng = '1'
         else cl_a2;
    lb1_d  <= "00" when e_st = E_CLEAR
         else lbm_d when s_lb_eng = '1'
         else "00";

    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            if lb0_we = '1' then lb0(to_integer(lb0_a)) <= lb0_d; end if;
            if lb1_we = '1' then lb1(to_integer(lb1_a)) <= lb1_d; end if;
            lbq0 <= lb0(to_integer(lb_ra));
            lbq1 <= lb1(to_integer(lb_ra));
        end if;
    end process p_lbram;

    ------------------------------------------------------------------------
    -- raster measurement: width from the avid run, height from active lines.
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then                 -- avid just fell
                if s_xcnt > 16 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
                s_lcnt <= s_lcnt + 1;
            end if;

            if s_vs_pulse = '1' then
                if s_lcnt > 8 then
                    if s_ilace = '1' then s_H <= shift_left(s_lcnt, 1);
                    else                  s_H <= s_lcnt; end if;
                end if;
                s_lcnt <= (others => '0');
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer + px conversion pass.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_kx, v_ky : signed(10 downto 0);
        variable v_ct   : signed(27 downto 0);
        variable v_targ : signed(21 downto 0);
        variable v_li   : integer;
        variable v_lfr  : integer range 0 to 63;
        variable v_fsh  : integer;
        variable v_nl   : signed(15 downto 0);
        variable v_ip   : integer;
        variable v_e    : signed(27 downto 0);
        variable v_dy0  : signed(14 downto 0);
        variable v_lz   : signed(15 downto 0);
        variable v_pe   : signed(13 downto 0);
        variable v_req  : std_logic;
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            s_cv_done    <= '0';
            s_chain_q    <= s_chain_run;
            s_mreq       <= '0';                 -- 1-clock multiplier request
            v_req        := '0';

            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';

                v_kx := signed('0' & registers_in(0)) - to_signed(512, 11);
                v_ky := signed('0' & registers_in(1)) - to_signed(512, 11);
                if abs(v_kx) < C_DB then v_kx := (others => '0'); end if;
                if abs(v_ky) < C_DB then v_ky := (others => '0'); end if;
                s_cx_k <= v_kx;
                s_cy_k <= v_ky;

                s_k3_per <= unsigned(registers_in(2));
                s_k4_gr  <= unsigned(registers_in(3));
                s_k5     <= unsigned(registers_in(4));      -- K5 Width
                s_glx <= registers_in(6)(0);   -- S7 GLIDE
                s_gly <= registers_in(6)(0);
                s_dyn <= registers_in(6)(1);   -- S8 Cascade (uniform / sequential)
                s_s9  <= registers_in(6)(2);   -- S9: Moire
                s_home<= registers_in(6)(3);   -- S10 Catch-Up (stay-dragged / glide-home)
                s_vsat<= registers_in(6)(4);   -- S11 Output: Rings / Video (refraction)
                s_p12 <= unsigned(registers_in(7));            -- P12 Zoom (Rings) / Wave depth (Video)

                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then s_ilace <= '1';
                else                              s_ilace <= '0'; end if;

                s_seq <= to_unsigned(21, 5);

            elsif s_seq /= 0 and seq_go = '1' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is
                    -- v2.0: ONE product at a time on the serial multiplier.
                    -- A request slot raises s_mreq; the sequencer then stalls
                    -- (mu_busy) and the NEXT slot reads the finished product.

                    -- ZOOM (P12 in Rings mode): the slider subtracts up to 3
                    -- octaves from K3's log-spacing -> rings expand up to x8
                    -- from the centre (clamped at the 2048 px maximum).  Off
                    -- in Video mode, where P12 is the wave depth.
                    when 21 =>
                        s_lfb <= to_signed(-704, 16)
                                 + signed(resize(shift_right(resize(s_k3_per, 14)        -- K3 * 9/16
                                   + shift_left(resize(s_k3_per, 14), 3), 4), 16));
                        if s_vsat = '1' then
                            s_zm <= (others => '0');
                        else                                -- P12 * 3/16 -> 0..191
                            s_zm <= resize(shift_right(resize(s_p12, 12)
                                    + shift_left(resize(s_p12, 12), 1), 4), 8);
                        end if;

                    -- centre: cx = W/2 + (K1-512)*1.25W/1024 (+-0.625 W: the
                    -- centre goes ~12% past each edge at most)
                    when 20 =>
                        s_ma <= resize(s_cx_k, 14);
                        s_mb <= signed(resize(resize(s_W, 13) + shift_right(resize(s_W, 13), 2), 14));
                        s_mreq <= '1';  v_req := '1';
                    when 19 =>
                        v_ct := resize(m_p, 28);                          -- slot-20 product
                        v_targ := shift_left(resize(signed('0' & s_W(11 downto 1)), 22), 8)
                                  + resize(shift_right(v_ct, 2), 22);
                        if s_glx = '1' then s_cxs <= s_cxs + shift_right(v_targ - s_cxs, C_GSH);
                        else                s_cxs <= v_targ; end if;
                        s_ma <= resize(s_cy_k, 14);                       -- centre Y
                        s_mb <= signed(resize(resize(s_H, 13) + shift_right(resize(s_H, 13), 2), 14));
                        s_mreq <= '1';  v_req := '1';
                    when 18 =>
                        v_ct := resize(m_p, 28);                          -- slot-19 product
                        v_targ := shift_left(resize(signed('0' & s_H(11 downto 1)), 22), 8)
                                  + resize(shift_right(v_ct, 2), 22);
                        if s_gly = '1' then s_cys <= s_cys + shift_right(v_targ - s_cys, C_GSH);
                        else                s_cys <= v_targ; end if;
                        -- FREQUENCY: Lf(Q6) = -704 + Period*9/16 - zoom
                        v_lz := s_lfb - signed(resize(s_zm, 16));
                        if v_lz < -704 then v_lz := to_signed(-704, 16); end if;   -- spacing <= 2048
                        s_lf <= v_lz;
                        -- K5 ease (pot noise would shimmer the ring edges)
                        v_pe   := signed(resize(s_k5f, 14)) + shift_right(
                                  signed(shift_left(resize(s_k5, 14), 3))
                                  - signed(resize(s_k5f, 14)), 3);
                        s_k5f  <= unsigned(v_pe(12 downto 0));
                    -- the decode is pipelined one step per slot (a single-slot
                    -- negate+ROM+barrel cone was hd_hdmi's critical path)
                    when 17 =>
                        v_li  := to_integer(shift_right(s_lf, 6));
                        v_lfr := to_integer(unsigned(s_lf(5 downto 0)));
                        s_fnum <= C_MANT(v_lfr);
                        v_fsh := 2 - v_li;
                        if    v_fsh < 2  then v_fsh := 2;
                        elsif v_fsh > 15 then v_fsh := 15; end if;
                        s_sft <= v_fsh;
                        s_nl  <= unsigned(-s_lf);
                        s_fd  <= signed(resize(s_k5f, 14)) - to_signed(4096, 14);
                    -- SPACING (px per ring) = 2^(-Lf/64) = C_MANT(frac) >> (10-int).
                    -- Lf in [-704,-129] -> int in [2,11] -> spacing in [4,2048].
                    when 16 =>
                        s_mant <= C_MANT(to_integer(s_nl(5 downto 0)));
                        s_sip  <= to_integer(s_nl(9 downto 6));
                        -- WIDTH fraction f = fd/4096 = (K5-512)/512 -> +-1 over
                        -- the full knob; the s-4 cap binds only at the ends
                        s_fab  <= unsigned(resize(abs(s_fd), 13));   -- |fd| <= 4096
                        s_dneg <= s_fd(13);
                        if s_fd > -64 and s_fd < 64 then s_fdz <= '1'; else s_fdz <= '0'; end if;
                    when 15 =>
                        if s_sip > 10 then
                            s_spac <= resize(shift_left(resize(s_mant, 13), s_sip - 10), 13);
                            s_spl  <= resize(shift_left(resize(s_mant, 13), s_sip - 10), 13);
                        else
                            s_spac <= resize(shift_right(s_mant, 10 - s_sip), 13);
                            s_spl  <= resize(shift_right(s_mant, 10 - s_sip), 13);
                        end if;
                    -- stepped per-ring phase step Q1 = 2^16/s = fnum*2^(16-sft)>>8
                    when 14 =>
                        s_h1s <= shift_left(to_signed(1, 14), 16 - s_sft);
                        s_spac_e <= s_spac;                          -- span-engine copy
                    -- WIDTH: |delta| = |fd| * spacing >> 12.  Video mode and
                    -- the centre deadband force classic bands.
                    when 13 =>
                        if s_vsat = '1' or s_fdz = '1' then
                            s_ma <= (others => '0');
                        else
                            s_ma <= signed(resize(s_fab, 14));
                        end if;
                        s_mb <= signed(resize(s_spac, 14));
                        s_mreq <= '1';  v_req := '1';
                    when 12 =>
                        s_dabs <= unsigned(m_p(24 downto 12));       -- slot-13 product
                        s_ma <= s_h1s;                               -- Q1 = h1s * fnum
                        s_mb <= signed(resize(s_fnum, 14));
                        s_mreq <= '1';  v_req := '1';
                    when 11 =>
                        -- Q1 capture (slot-12 product; clamp to 14 bits, the
                        -- serial multiplier's operand range)
                        if unsigned(m_p(23 downto 8)) > 16383 then
                            s_qp1 <= to_unsigned(16383, 15);
                        else
                            s_qp1 <= resize(unsigned(m_p(22 downto 8)), 15);
                        end if;
                        if s_dabs + 4 > s_spac then s_dabs <= s_spac - 4; end if; -- bands >= 4 px
                    when 10 =>
                        s_seff <= s_spac - s_dabs;
                    -- octagon slack in px from the NARROWEST band s_eff (pure
                    -- shift-adds, one short step per slot):
                    --   A = min(e - e/8,  min(e - e/16 - e/32 - 3, 640))
                    -- (~0.875*e; the cap terms keep adjacent edges > 1 px apart
                    -- on any scanline -- mark-write collision safety -- and the
                    -- chain words inside 15 bits)
                    --   C = A * ~sqrt2 = A + A/4 + A/8 + A/32 + A/128
                    when 9 =>
                        s_dabs_e <= s_dabs;  s_dabs_p <= s_dabs;    -- per-consumer copies
                        s_dneg_e <= s_dneg;  s_dneg_p <= s_dneg;
                        s_seff_p <= s_seff;
                        s_shalf_p <= resize(shift_right(s_spac, 1), 12);
                        s_araw <= resize(s_seff - shift_right(s_seff, 3), 12);
                        s_acap <= resize(s_seff - shift_right(s_seff, 4)
                                         - shift_right(s_seff, 5), 13);
                        -- K4 GRADIENT: four quarters of the knob
                        s_gn <= s_k4_gr(9 downto 8);
                    when 8 =>
                        v_e := resize(signed(s_acap), 28) - 3;
                        if v_e < 0 then v_e := (others => '0'); end if;
                        if v_e > 640 then v_e := to_signed(640, 28); end if;
                        s_acap <= unsigned(v_e(12 downto 0));
                    when 7 =>
                        if resize(s_araw, 13) < s_acap then
                            s_apx <= s_araw;
                        else
                            s_apx <= resize(s_acap, 12);
                        end if;
                    when 6 =>
                        s_cpx <= resize(s_apx, 13) + resize(shift_right(s_apx, 2), 13)
                                 + resize(shift_right(s_apx, 3), 13);
                    when 5 =>
                        s_cpx <= s_cpx + resize(shift_right(s_apx, 5), 13)
                                 + resize(shift_right(s_apx, 7), 13);
                        -- publish the glided centre in pixels
                        s_cx <= resize(shift_right(s_cxs, 8), 14);
                        s_cy <= resize(shift_right(s_cys, 8), 14);
                    when 4 =>
                        s_sm1 <= resize(s_spac - 1, 12);
                    -- (slots 3..1 free; the chain scan starts at s_seq = 4,
                    -- after the slack decode -- slot 0 never executes)
                    when others => null;
                end case;
            end if;

            --------------------------------------------------------------
            -- frame-terms tail: chain done -> the tracker's frame constants
            -- through the same shared multiplier (5 cycles).
            --------------------------------------------------------------
            if s_chain_q = '1' and s_chain_run = '0' then
                s_tlrun <= '1'; s_tlph <= 0;
            elsif s_tlrun = '1' and seq_go = '1' then
                case s_tlph is
                    when 0 =>
                        s_dx0 <= resize(-s_cx, 14) - resize(s_exo, 14);
                    when 1 =>
                        s_ma <= abs(s_dx0);  s_mb <= abs(s_dx0);  -- dx0'^2 (serial mult)
                        s_mreq <= '1';  v_req := '1';
                        s_q0 <= resize(shift_left(resize(s_dx0, 16), 1) + 1, 16);
                    when 2 =>
                        s_dx0q <= unsigned(m_p(26 downto 0));
                    when others =>                     -- phase 3: publish + done
                        -- first active line's dy (p_acc hasn't primed it yet)
                        if s_ilace = '1' and data_in.field_n = '1' then
                            v_dy0 := resize(-s_cy, 15) + 1;
                        else
                            v_dy0 := resize(-s_cy, 15);
                        end if;
                        s_dy0 <= v_dy0;
                        s_cv_done <= '1';
                        s_tlrun <= '0';
                end case;
                if s_tlph /= 3 then s_tlph <= s_tlph + 1; end if;
            end if;

            if data_in.avid = '0' and e_st = E_IDLE and mu_busy = '0'
               and s_mreq = '0' and nx_ld = '0' and v_req = '0' then
                seq_go <= '1';
            else
                seq_go <= '0';
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- coordinate accumulators.  dx loads -cx at each active-line start and
    -- steps +1 per pixel; dy loads -cy at the FIRST active line of the frame
    -- and steps per line -- by 2 on interlaced sources, half-step on field 2.
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif data_in.avid = '1' and s_avid_p = '0' then      -- active-line start
                if s_newframe = '1' then
                    if s_ilace = '1' and data_in.field_n = '1' then v_dy0 := resize(-s_cy, 15) + 1;
                    else                                            v_dy0 := resize(-s_cy, 15); end if;
                    s_dyc <= v_dy0;
                    if s_ilace = '1' then s_dyb <= v_dy0 + 2; else s_dyb <= v_dy0 + 1; end if;
                    s_newframe <= '0';
                else
                    s_dyc <= s_dyb;
                    if s_ilace = '1' then s_dyb <= s_dyb + 2; else s_dyb <= s_dyb + 1; end if;
                end if;
                s_dx <= resize(-s_cx, 15);
            elsif data_in.avid = '1' then
                s_dx <= s_dx + 1;
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- CASCADE chain: a coupled chain of per-ring centre offsets, one ring per
    -- 4 clocks during vblank.  Ring 0 = the glided centre (offset 0).  Each
    -- frame every offset shifts by how far the inner ring moved (px), then is
    -- HARD-CLAMPED to the octagon around the ring inside -- the never-overlap
    -- guarantee AND the cascade dead-zone.  No accumulated cap: the wave
    -- travels the full C_NR-ring chain.  This scan also derives s_nreff, the
    -- number of rings the span engine will actually render this frame.
    ------------------------------------------------------------------------
    p_chain : process(clk)
        variable v_mvx, v_mvy : integer;
        variable vinx, viny : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            ch_blank <= not data_in.avid;
            if s_seq = 4 then ch_go4 <= '1'; else ch_go4 <= '0'; end if;
            ew_wex <= '0';                              -- default: no write
            ew_wey <= '0';
            s_ocap <= '0';                              -- default: 1-cycle pulse
            if s_vs_pulse = '1' then
                v_mvx := to_integer(s_cx - s_cxp);
                v_mvy := to_integer(s_cy - s_cyp);
                if    v_mvx >  255 then v_mvx :=  255;
                elsif v_mvx < -255 then v_mvx := -255; end if;
                if    v_mvy >  255 then v_mvy :=  255;
                elsif v_mvy < -255 then v_mvy := -255; end if;
                s_mvx <= to_signed(v_mvx, 10);
                s_mvy <= to_signed(v_mvy, 10);
                s_cxp <= s_cx;
                s_cyp <= s_cy;
                -- reset ring 0 to offset 0; the scan itself starts once the
                -- sequencer has finished the A/C slack decode (slot 5) so no
                -- ring ever clamps with stale or half-accumulated bounds
                ew_addr  <= 0; ew_dinx <= (others => '0'); ew_diny <= (others => '0');
                ew_wex   <= '1';  ew_wey <= '1';
                s_eprevx <= (others => '0'); s_eprevy <= (others => '0');
                s_chain_cnt <= 1; s_chain_ph <= 0; s_chain_wt <= '1';
            elsif s_chain_wt = '1' then
                if ch_go4 = '1' then          -- s_seq reached 4: slack decode done
                    s_chain_wt  <= '0';
                    s_chain_run <= '1';
                    -- ring 1's radius = 2*spacing (this frame's spacing lands
                    -- at sequencer slot 14, so it must be read HERE, not at
                    -- vsync); floor the effective ring count at 2
                    s_racc <= resize(s_spac, 14) + resize(s_spac, 14);
                    s_nrw  <= to_unsigned(2, 7);
                    s_codd <= '1';                  -- scan starts at ring 1
                end if;
            elsif s_chain_run = '1' and ch_blank = '1' then
                case s_chain_ph is
                when 0 =>
                    s_chain_ph <= 1;                    -- RD: issue e[k]_old read
                when 1 =>
                    -- LATCH: re-register the slow BRAM output.
                    s_doax <= doa_ex;  s_doay <= doa_ey;
                    s_chain_ph <= 2;
                when 2 | 5 =>
                    -- STEP d = (e_old - mv) - e_prev.  One shared unit,
                    -- x at 2, y at 5.
                    if s_chain_ph = 2 then
                        s_dw <= resize(s_doax, 16) - resize(s_mvx, 16) - resize(s_eprevx, 16);
                    else
                        s_dw <= resize(s_doay, 16) - resize(s_mvy, 16) - resize(s_eprevy, 16);
                    end if;
                    s_chain_ph <= s_chain_ph + 1;
                when 3 | 6 =>
                    -- SETTLE glide toward the ring inside, SYMMETRIC: pull the
                    -- step's MAGNITUDE down by ceil(|d|/2^C_SET) so it lands on
                    -- exactly 0 from either side.  (v0.8 used d + floor(-d/8),
                    -- i.e. d - ceil(d/8): exact for d > 0 but a NO-OP for
                    -- -7 <= d <= -1.  d is the step from the ring INSIDE, so a
                    -- drag right/down makes the rings lag left/up and d comes
                    -- out NEGATIVE -- the common case.  Every ring froze up to
                    -- 7 px off its parent, cumulative down the chain (161 px
                    -- at ring 23), and Catch-Up never finished centring.)
                    -- Also pre-register the next clamp's bound pair (keeps the
                    -- clamp cone to register -> compare -> mux).
                    if s_settle = '1' then
                        if s_dw(15) = '0' then
                            s_dw <= s_dw - shift_right(s_dw + to_signed(2**C_SET - 1, 16), C_SET);
                        else
                            s_dw <= s_dw - shift_right(s_dw, C_SET);
                        end if;
                    end if;
                    s_clb  <= signed(resize(s_apx, 16));
                    s_clbn <= -signed(resize(s_apx, 16));
                    s_chain_ph <= s_chain_ph + 1;
                when 8 | 10 =>
                    -- ROTATE: u = dx+dy (8), v = dx-dy (10) into the same reg
                    if s_chain_ph = 8 then
                        s_dw <= resize(s_qdx, 16) + resize(s_qdy, 16);
                    else
                        s_dw <= resize(s_qdx, 16) - resize(s_qdy, 16);
                    end if;
                    s_clb  <= signed(resize(s_cpx, 16));
                    s_clbn <= -signed(resize(s_cpx, 16));
                    s_chain_ph <= s_chain_ph + 1;
                when 4 | 7 | 9 | 11 =>
                    -- ONE shared clamp cone: +/-A on the axis steps (4,7),
                    -- +/-C = A*sqrt2 on the rotated pair (9,11) -- the
                    -- octagon's 45-degree faces (a box alone would let
                    -- diagonal drags overlap the rings at this much slack)
                    viny := s_dw;
                    if    viny > s_clb  then viny := s_clb;
                    elsif viny < s_clbn then viny := s_clbn; end if;
                    case s_chain_ph is
                        when 4      => s_qdx <= resize(viny, 13);
                        when 7      => s_qdy <= resize(viny, 13);
                        when 9      => s_pu  <= resize(viny, 14);
                        when others => s_pv  <= resize(viny, 14);
                    end case;
                    s_chain_ph <= s_chain_ph + 1;
                when 12 | 13 =>
                    -- BACK-rotate (exact when unclipped: (u+v)/2 = dx),
                    -- rebuild the absolute offset and WRITE (x at 12, y at
                    -- 13).  |e[k]| <= k*(1.083*A+1.42) < 16383 by the A cap
                    -- -- no storage clamp needed.  Uniform mode writes zeros
                    -- so flipping S8 on starts from a clean nest.
                    if s_chain_ph = 12 then
                        vinx := resize(s_eprevx, 16)
                                + resize(shift_right(resize(s_pu, 16) + resize(s_pv, 16), 1), 16);
                    else
                        vinx := resize(s_eprevy, 16)
                                + resize(shift_right(resize(s_pu, 16) - resize(s_pv, 16), 1), 16);
                    end if;
                    if s_dyn = '0' then
                        vinx := (others => '0');
                    end if;
                    ew_addr <= s_chain_cnt;
                    if s_chain_ph = 12 then
                        ew_dinx  <= resize(vinx, 15);
                        ew_wex   <= '1';
                        s_eprevx <= resize(vinx, 15);
                        s_chain_ph <= 13;
                    else
                        ew_diny  <= resize(vinx, 15);
                        ew_wey   <= '1';
                        s_eprevy <= resize(vinx, 15);
                        s_chain_ph <= 0;
                        -- adaptive count: s_racc is ring s_chain_cnt's radius.
                        -- An ODD ring that still fits under C_RCL makes the
                        -- (even) count s_chain_cnt+1, and its offset is the
                        -- outer field's anchor.  Ring 1 always qualifies so a
                        -- 2048-px spacing still has an anchor.
                        s_codd <= not s_codd;
                        if s_codd = '1'
                           and (s_racc <= C_RCL or s_chain_cnt = 1) then
                            s_nrw  <= to_unsigned(s_chain_cnt + 1, 7);
                            s_ocap <= '1';
                        end if;
                        if s_racc <= C_RCL then
                            s_racc <= s_racc + resize(s_spac, 14);
                        end if;
                        if s_chain_cnt = C_NR - 1 then
                            s_chain_run <= '0';
                        else
                            s_chain_cnt <= s_chain_cnt + 1;
                        end if;
                    end if;
                end case;
            end if;

            -- Catch-Up (S10): glide home only when armed AND settled.  From
            -- the REGISTERED movement (stable all frame; first chain use is
            -- 3 cycles after vsync).
            if s_dyn = '1' and s_home = '1' and s_mvx < 2 and s_mvx > -2
                          and s_mvy < 2 and s_mvy > -2 then
                s_settle <= '1';
            else
                s_settle <= '0';
            end if;

            -- outer-field recentre = ring s_nreff-1's offset, clamped from the
            -- REGISTERED write value one cycle after that ring's phase 13
            -- (ew_dinx/ew_diny hold until the next write, so the pulse can
            -- read them a cycle late -- same slot the old scan-end capture
            -- used, so the frame tail still sees the new value in time).
            if s_ocap = '1' then
                if    ew_dinx >  C_OCL then s_exo <= to_signed( C_OCL, 13);
                elsif ew_dinx < -C_OCL then s_exo <= to_signed(-C_OCL, 13);
                else                        s_exo <= resize(ew_dinx, 13); end if;
                if    ew_diny >  C_OCL then s_eyo <= to_signed( C_OCL, 13);
                elsif ew_diny < -C_OCL then s_eyo <= to_signed(-C_OCL, 13);
                else                        s_eyo <= resize(ew_diny, 13); end if;
            end if;

            -- publish the effective ring count one cycle after the scan ends
            s_chain_d <= s_chain_run;
            if s_chain_d = '1' and s_chain_run = '0' then
                s_nreff <= s_nrw;
            end if;
        end if;
    end process p_chain;

    ------------------------------------------------------------------------
    -- SPAN + LINE-SEED engine + the per-pixel r^2 BAND TRACKER.
    --
    -- E_SPAN runs a line EARLY: triggered at each active-line RISE, it
    -- computes the NEXT line's disc spans (24 rings x 8 slots ~ 210 clks,
    -- inside the 720+ clk active period) and writes their edge MARKS into
    -- the engine line-buffer bank.  Only the adds-only tracker runs per
    -- pixel, so the chain RAM, engine multiply and sqrt units are free.
    -- E_LINE (the tracker seed, ~44 clks) runs in the hblank as before.
    -- A frame's first line is produced by the cv_done chain in vblank:
    -- E_CLEAR (wipe both banks) -> E_SPAN -> E_LINE.
    --
    -- Active video, per pixel (tracker; all single adds):
    --   u   += q            (r^2 advance; q = 2*dx'+1, q += 2 per pixel)
    --   out (u+q >= D):     u := u + qmD;  k++   (qmD = q - D)
    --   in  (u+q < 0):      u := u + qpDm; k--   (qpDm = q + Dm)
    ------------------------------------------------------------------------
    p_engine : process(clk)
        variable v_dy, v_ad : signed(15 downto 0);
        variable v_w  : signed(15 downto 0);
        variable v_s1, v_o1, v_i1 : signed(17 downto 0);
        variable v_q2 : std_logic_vector(1 downto 0);
        variable v_ud : signed(28 downto 0);
        variable v_tr : unsigned(26 downto 0);
        variable v_ki : unsigned(6 downto 0);
        variable v_wide : signed(15 downto 0);
        variable v_xl, v_xc : signed(15 downto 0);
        variable v_xv, v_cv : std_logic;
        -- serial-multiplier load request (any slot below may raise it)
        variable v_ld  : std_logic;
        variable v_lm  : signed(14 downto 0);
        variable v_lq  : unsigned(13 downto 0);
        variable v_sum : signed(16 downto 0);
        variable v_mq  : unsigned(13 downto 0);
        variable v_dxl : signed(14 downto 0);
        -- radix-4 sqrt step temporaries
        variable v_sqn, v_sqr, v_sqo : unsigned(26 downto 0);
        -- E_HERM temporaries
        variable v_hd : signed(15 downto 0);
        variable v_ix : integer range 0 to 7;
        variable v_rv : unsigned(17 downto 0);
        variable v_tp : unsigned(8 downto 0);
        variable v_nb : integer range 0 to 17;
        variable v_l  : signed(15 downto 0);
        variable v_fp : signed(29 downto 0);
        variable v_dp : signed(35 downto 0);
        variable v_rm : unsigned(17 downto 0);
        variable v_iap, v_oap : std_logic;
    begin
        if rising_edge(clk) then
            v_ld := '0';
            v_lm := (others => '0');
            v_lq := (others => '0');

            -- pending right-edge mark: issue one cycle after it was staged
            -- (left/right of one ring are 8 slots apart -- never collides)
            lbm_we <= '0';
            if lbp_we = '1' then
                lbm_we <= '1';
                lbm_a  <= lbp_a;
                lbm_d  <= "11";                       -- leave: k+1
                lbp_we <= '0';
            end if;

            -- the engine bank flips at every active-line start (the pass
            -- launched this line writes the NEXT line's marks)
            if data_in.avid = '1' and s_avid_p = '0' then
                s_lb_eng <= not s_lb_eng;
            end if;

            -- E_LINE request while the (early-started) span pass is busy --
            -- only possible on rasters with very short active lines (e.g.
            -- decimated simulation)
            if s_avid_p = '1' and data_in.avid = '0' and e_st /= E_IDLE then
                l_pend <= '1';
            end if;

            ------------------------------------------------------------
            -- sqrt unit: one radix-2 step per cycle while running.
            -- Loads (below) override these assignments.
            ------------------------------------------------------------
            if sqa_run = '1' then
                v_tr := sqa_res or sqa_one;
                if sqa_num >= v_tr then
                    sqa_num <= sqa_num - v_tr;
                    sqa_res <= shift_right(sqa_res, 1) or sqa_one;
                else
                    sqa_res <= shift_right(sqa_res, 1);
                end if;
                sqa_one <= shift_right(sqa_one, 2);
                if sqa_one(0) = '1' then sqa_run <= '0'; end if;
            end if;
            ----------------------------------------------------------
            -- TRACKER STEP -- runs on s_avid_p (so the line's LAST
            -- pixel steps too, on the fall cycle).  Concurrent with
            -- the span FSM below; they share no registers (E_LINE
            -- touches t_* only in hblank, when this doesn't run).
            ----------------------------------------------------------
            if s_avid_p = '1' then
                -- INTEGER-RADIUS TRACKER (spacing 1: taps degenerate to
                -- +4/0/+2, 17-bit arithmetic).  Every crossing is one radial
                -- pixel: the m = r mod s
                -- counter wraps -> ring index +/-1 (stepped mode).  This
                -- replaced the 28-bit r^2-bracket tracker outright.
                v_s1 := resize(su_u, 18) + resize(t_q, 18);
                v_o1 := resize(su_u, 18) + su_qmd;
                v_i1 := resize(su_u, 18) + su_qpdm;
                if v_s1 < 0 then                          -- inward: r-1
                    su_u    <= resize(v_i1, 17);
                    su_qmd  <= su_qmd + 4;
                    su_cr   <= "11";                      -- -1 (applied next cycle)
                    if su_m = 0 then
                        su_m <= resize(s_sm1, 12);
                        t_kl <= t_kl - 1;
                    else
                        su_m <= su_m - 1;
                    end if;
                elsif v_o1 >= 0 then                      -- outward: r+1
                    su_u    <= resize(v_o1, 17);
                    su_qpdm <= su_qpdm + 4;
                    su_cr   <= "01";                      -- +1 (applied next cycle)
                    if su_m = s_sm1 then
                        su_m <= (others => '0');
                        t_kl <= t_kl + 1;
                    else
                        su_m <= su_m + 1;
                    end if;
                else
                    su_u    <= resize(v_s1, 17);
                    su_qmd  <= su_qmd + 2;
                    su_qpdm <= su_qpdm + 2;
                    su_cr   <= "00";
                end if;
                t_q <= t_q + 2;
            end if;

            ----------------------------------------------------------
            -- PER-RING phase, applied ONE cycle after the crossing was
            -- detected so it lands on the same edge as the mark read
            -- (the mark data trails the crossing detect by a cycle):
            -- a disc-edge mark re-anchors the ramp to 0, else +/-Q1.
            ----------------------------------------------------------
            st_v <= s_avid_p;
            if st_v = '1' then
                v_q2 := lbq0;
                if s_lb_disp = '1' then v_q2 := lbq1; end if;
                if v_q2(0) = '1' then
                    ph1_acc <= (others => '0');
                elsif su_cr = "01" then
                    ph1_acc <= ph1_acc + resize(s_qp1, 16);
                elsif su_cr = "11" then
                    ph1_acc <= ph1_acc - resize(s_qp1, 16);
                end if;
            end if;

            case e_st is

            when E_IDLE =>
                if s_cv_done = '1' then
                    s_lb_eng <= '0';                  -- line 0's marks -> bank 0
                    e_first  <= '1';
                    cl_cnt   <= (others => '0');
                    cl_end   <= resize(s_W, 12) + 1;  -- registered equality exit (cl_hit)
                    cl_hit   <= '0';
                    e_st     <= E_CLEAR;              -- chain here was critical)
                elsif data_in.avid = '1' and s_avid_p = '0' then
                    sp_rr  <= resize(s_spac_e, 12);     -- pass for the NEXT line
                    m_kinit <= s_nreff;
                    m0_w   <= '0';
                    xlv_w  <= '0';
                    e_cnt  <= 0;  e_ph <= 0;
                    e_st   <= E_SPAN;
                elsif l_pend = '1' or (s_avid_p = '1' and data_in.avid = '0') then
                    l_pend <= '0';
                    l_ph   <= 0;
                    e_st   <= E_LINE;
                end if;

            when E_CLEAR =>
                -- wipe both banks 0..W+1 (write-port muxes route this state)
                cl_cnt <= cl_cnt + 1;
                if cl_cnt = cl_end then cl_hit <= '1'; else cl_hit <= '0'; end if;
                if cl_hit = '1' then                  -- cl_cnt = W+2, as before
                    sp_dy  <= s_dy0;                  -- frame's first line
                    sp_rr  <= resize(s_spac_e, 12);
                    m_kinit <= s_nreff;
                    m0_w   <= '0';
                    xlv_w  <= '0';
                    e_cnt  <= 0;  e_ph <= 0;
                    e_st   <= E_SPAN;
                end if;

            when E_SPAN =>
                case e_ph is
                    when 0 =>
                        -- chain RAM read settling; per-line trigger grabs the
                        -- next line's dy here (s_dyb updated at the rise edge)
                        f_lt   <= '0';  f_mid <= '0';  f_ge2 <= '0';  f_last <= '0';
                        if e_cnt < to_integer(s_nreff) then f_lt <= '1'; end if;
                        if e_cnt >= 1 and e_cnt <= to_integer(s_nreff) then f_mid <= '1'; end if;
                        if e_cnt >= 2 then f_ge2 <= '1'; end if;
                        if e_cnt = to_integer(s_nreff) + 1 then f_last <= '1'; end if;
                        if e_cnt = 0 and e_first = '0' then
                            sp_dy <= s_dyb;
                        end if;
                    when 1 =>
                        if f_lt = '1' then
                            sp_ey <= doa_ey;                  -- both axes arrive together
                            sp_ex <= doa_ex;
                        end if;
                    when 2 =>
                        if f_lt = '1' then
                            v_dy := resize(sp_dy, 16) - resize(sp_ey, 16);
                            v_ad := abs(v_dy);                -- <= 12500, bit 15 = 0
                            sp_ady <= unsigned(v_ad(14 downto 0));
                            sp_exc <= resize(sp_ex, 16) + resize(s_cx, 16);  -- raw screen x
                        end if;
                    when 3 =>
                        if f_lt = '1' then
                            if sp_ady > C_RCL then sp_emp <= '1'; sp_ady <= to_unsigned(C_RCL, 15);
                            else sp_emp <= '0'; end if;
                        end if;
                        -- WIDTH: even discs get R +- |delta| (odd discs, incl.
                        -- the outermost modelled one, keep R -> the outer seam
                        -- is untouched).  Stays in 4..4095 by the s-4 cap.
                        if (e_cnt mod 2) = 1 then
                            sp_rq <= sp_rr;
                        elsif s_dneg_e = '1' then
                            sp_rq <= sp_rr - resize(s_dabs_e, 12);
                        else
                            sp_rq <= sp_rr + resize(s_dabs_e, 12);
                        end if;
                    when 4 =>
                        -- v2.0 SERIAL-MULTIPLIER PIPELINE (request here, load
                        -- next clock, product 16 clocks later = phase 5 of
                        -- the next group).  Multiply ring k:
                        --   w^2 = (R - |dy|) * (R + |dy|)
                        if f_lt = '1' then
                            v_ld := '1';
                            v_lm := signed(resize(sp_rq, 15)) - signed(resize(sp_ady(12 downto 0), 15));
                            v_lq := resize(sp_rq, 14) + resize(sp_ady(12 downto 0), 14);
                        end if;
                    when 5 =>
                        -- all three read pre-edge values:
                        --  (a) stage ring k-2's edges from the finished root
                        v_w := signed(resize(sqa_res(12 downto 0), 16));
                        sp_wl  <= sp_excq1 - v_w;
                        sp_wr1 <= sp_excq1 + v_w + 1;         -- first x OUTSIDE
                        sp_empq2 <= sp_empq1;
                        --  (b) root ring k-1's product (complete this slot)
                        if f_mid = '1' then
                            if m_p < 0 then sqa_num <= (others => '0');
                            else sqa_num <= unsigned(m_p(26 downto 0)); end if;
                            sqa_res <= (others => '0');
                            sqa_one <= shift_left(to_unsigned(1, 27), 26);
                            sqa_run <= '1';
                        end if;
                        sp_excq1 <= sp_excq0;
                        if m_p < 0 then sp_empq1 <= '1';
                        else            sp_empq1 <= sp_empq0; end if;
                        --  (c) ring k's centre/empty flag ride with its product
                        sp_excq0 <= sp_exc;
                        sp_empq0 <= sp_emp;
                    when 6 =>
                        -- ring e_cnt-2: DIRECTION-coded edge marks -> engine
                        -- bank (left/enter now, right/leave staged one cycle);
                        -- discs overhanging the left screen edge decrement
                        -- the line-start ring index instead of a mark.
                        if f_ge2 = '1' then
                            v_ki := m_kinit;
                            v_wide := signed(resize(s_W, 16));
                            -- nearest-edge candidate at/left of x=0 (wr1 > wl,
                            -- so the right edge wins when both qualify);
                            -- compared into xl_w at phase 6 (a one-cycle cone
                            -- here was the HD critical path)
                            if sp_wr1 <= 0 then xc_r <= sp_wr1;
                            else                xc_r <= sp_wl; end if;
                            if sp_empq2 = '0' and (sp_wr1 <= 0 or sp_wl <= 0) then
                                xcv_r <= '1';
                            else
                                xcv_r <= '0';
                            end if;
                            if sp_empq2 = '0' then
                                if sp_wl < 0 and sp_wr1 > 0 then
                                    v_ki := v_ki - 1;
                                end if;
                                if sp_wl >= 0 and sp_wl < v_wide then
                                    lbm_we <= '1';
                                    lbm_a  <= unsigned(sp_wl(10 downto 0));
                                    lbm_d  <= "01";           -- enter: k-1
                                    if sp_wl = 0 then
                                        m0_w <= '1';          -- pixel 0 can't see its
                                    end if;                   -- own mark: snap the seed
                                end if;
                                if sp_wr1 > 0 and sp_wr1 < v_wide then
                                    lbp_we <= '1';
                                    lbp_a  <= unsigned(sp_wr1(10 downto 0));
                                end if;
                            end if;
                            m_kinit <= v_ki;
                        end if;
                    when 7 =>
                        -- next ring's radius
                        if f_lt = '1' then
                            if resize(sp_rr, 13) + resize(s_spac_e, 13) > C_RCL then
                                sp_rr <= to_unsigned(C_RCL, 12);
                            else
                                sp_rr <= sp_rr + resize(s_spac_e, 12);
                            end if;
                        end if;
                        -- nearest-edge update, then (last ring) capture the
                        -- line-start state and leave
                        if f_ge2 = '1' then
                            v_xl := xl_w;  v_xv := xlv_w;
                            if xcv_r = '1' and (xlv_w = '0' or xc_r > xl_w) then
                                v_xl := xc_r;  v_xv := '1';
                            end if;
                            xl_w <= v_xl;  xlv_w <= v_xv;
                            if f_last = '1' then
                                m_kinit_done <= m_kinit;
                                m0_done      <= m0_w;
                                xl_done      <= v_xl;
                                xlv_done     <= v_xv;
                                -- drain groups done
                                if e_first = '1' then         -- vblank chain
                                    e_first <= '0';
                                    l_ph <= 0;
                                    e_st <= E_LINE;
                                elsif l_pend = '1' then
                                    l_pend <= '0';
                                    l_ph <= 0;
                                    e_st <= E_LINE;
                                else
                                    e_st <= E_IDLE;
                                end if;
                            end if;
                        end if;
                    when others => null;                      -- sqrt stepping
                end case;
                if e_ph = 14 then
                    e_ph <= 0;
                    e_cnt <= e_cnt + 1;
                else
                    e_ph <= e_ph + 1;
                end if;

            when E_LINE =>
                -- v2.0: re-scheduled for the 15-slot serial multiplier (load
                -- at T, product at T+15); the two roots overlap the products.
                case l_ph is
                    when 0 =>
                        le_dyp <= resize(resize(sp_dy, 16) - resize(s_eyo, 16), 14);
                        t_q <= s_q0;                                      -- 2*dx0'+1 (frame tail)
                    when 1 =>                                             -- dy'^2
                        v_ld := '1';
                        v_lm := resize(abs(le_dyp), 15);
                        v_lq := unsigned(resize(abs(le_dyp), 14));
                    when 17 =>
                        le_r0sq <= s_dx0q + unsigned(m_p(26 downto 0));   -- r0^2 = dx0'^2 + dy'^2
                    when 18 =>
                        sqa_num <= le_r0sq;                               -- root r0^2
                        sqa_res <= (others => '0');
                        sqa_one <= shift_left(to_unsigned(1, 27), 26);
                        sqa_run <= '1';
                        -- LEFT-EDGE SEED: (x_L - outer cx)^2, in parallel
                        v_dxl := abs(resize(xl_done, 15) + resize(s_dx0, 15));   -- <= ~10.5k
                        v_ld := '1';
                        v_lm := v_dxl;
                        v_lq := unsigned(v_dxl(13 downto 0));
                    when 19 =>
                        le_r0sq <= le_r0sq - s_dx0q;                      -- keep dy'^2 for r_L
                    when 33 =>
                        -- r0 done: tracker seeds (u1 = the sqrt REMAINDER,
                        -- free; D1 = 2*r0+1) and r0 * fnum
                        le_r0 <= sqa_res(13 downto 0);
                        su_u  <= signed(resize(sqa_num(15 downto 0), 17));
                        su_d1 <= signed(resize(shift_left(resize(sqa_res(13 downto 0), 15), 1), 16)) + 1;
                        v_ld := '1';
                        v_lm := signed(resize(sqa_res(13 downto 0), 15));
                        v_lq := resize(s_fnum, 14);
                    when 34 =>
                        le_r0sq <= le_r0sq + unsigned(m_p(26 downto 0));  -- r_L^2 (slot-18 product)
                    when 35 =>                                            -- root r_L^2
                        sqa_num <= le_r0sq;
                        sqa_res <= (others => '0');
                        sqa_one <= shift_left(to_unsigned(1, 27), 26);
                        sqa_run <= '1';
                    when 36 =>
                        su_qmd  <= resize(t_q, 18) - resize(su_d1, 18);
                        su_qpdm <= resize(t_q, 18) + resize(su_d1, 18) - 2;
                    when 49 =>
                        le_k0p <= unsigned(m_p(24 downto 0));             -- r0 * fnum
                    when 50 =>
                        t_k   <= resize(shift_right(le_k0p, s_sft + 8), 12);
                        le_dl <= signed(resize(le_r0, 15)) - signed(resize(sqa_res(13 downto 0), 15));
                    when 51 =>                                            -- B_lo = k * spacing
                        v_ld := '1';
                        v_lm := signed(resize(t_k, 15));
                        v_lq := resize(s_spl, 14);
                    when 67 =>
                        lb_lo <= m_p(14 downto 0);
                    when 68 =>
                        -- d0 = r0 - B_lo (estimate error is a few multiples
                        -- of s -- corrected LINEARLY below, 15-bit ops only)
                        le_d <= signed(resize(le_r0, 15)) - lb_lo;
                    when 69 | 71 | 73 | 75 | 77 | 79 =>
                        -- correction step, clock A: both candidates
                        le_dm <= le_d - signed(resize(s_spl, 15));
                        le_dp <= le_d + signed(resize(s_spl, 15));
                    when 70 | 72 | 74 | 76 | 78 | 80 =>
                        -- clock B: select (d < 0 -> +s, k-1; d >= s -> -s, k+1)
                        if le_d(14) = '1' then
                            t_k  <= t_k - 1;
                            le_d <= le_dp;
                        elsif le_dm(14) = '0' then
                            t_k  <= t_k + 1;
                            le_d <= le_dm;
                        end if;
                    when 81 =>
                        -- publish ring index + ring phase m = d0; seed the
                        -- per-ring phase at x=0: (r0 - r_L)*Q1 when x=0 lies
                        -- in the cascade nest and its band's left edge is
                        -- known, else the outer-centre phase d0*Q1 (exact for
                        -- the outer field and in Uniform mode)
                        t_kl  <= t_k(6 downto 0);
                        su_m  <= unsigned(le_d(11 downto 0));
                        v_ld := '1';
                        if xlv_done = '1' and s_dyn = '1' and m_kinit_done /= s_nreff then
                            v_lm := le_dl;
                        else
                            v_lm := le_d;
                        end if;
                        v_lq := resize(s_qp1, 14);
                    when 97 =>
                        if m0_done = '1' then
                            ph1_acc <= (others => '0');
                        else
                            ph1_acc <= unsigned(m_p(15 downto 0));
                        end if;
                    when 98 =>
                        e_st <= E_IDLE;
                    when others => null;
                end case;
                if l_ph < 98 then l_ph <= l_ph + 1; end if;

            end case;

            ----------------------------------------------------------
            -- SERIAL MULTIPLIER.  Sequencer / frame-tail requests are
            -- time-disjoint with the engine (they only issue, and only
            -- load, while it is idle).  Product = {mu_a, mu_q}.
            ----------------------------------------------------------
            if s_mreq = '1' and e_st = E_IDLE then
                v_ld := '1';
                v_lm := resize(s_ma, 15);
                v_lq := unsigned(s_mb);
            end if;
            nx_ld <= v_ld;
            nx_m  <= v_lm;
            nx_q  <= v_lq;
            if nx_ld = '1' then
                mu_m    <= nx_m;
                mu_q    <= nx_q;
                mu_a    <= (others => '0');
                mu_cnt  <= 14;
                mu_busy <= '1';
            elsif mu_cnt /= 0 then
                if mu_q(0) = '1' then
                    v_sum := resize(mu_a, 17) + resize(mu_m, 17);
                else
                    v_sum := resize(mu_a, 17);
                end if;
                v_mq := shift_right(mu_q, 1);
                v_mq(13) := v_sum(0);
                mu_a <= v_sum(16 downto 1);
                mu_q <= v_mq;
                mu_cnt <= mu_cnt - 1;
                if mu_cnt = 1 then mu_busy <= '0'; end if;
            end if;
        end if;
    end process p_engine;

    m_p <= shift_left(resize(mu_a, 30), 14) or signed(resize(mu_q, 30));

    ------------------------------------------------------------------------
    -- tracker parity, delayed to meet the mark-parity path at the merge
    ------------------------------------------------------------------------
    p_par : process(clk)
        variable v_kw : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            k_ld1 <= t_kl;
            -- WIDTH (outer field): band k (radius k*s .. (k+1)*s, m = r mod s)
            -- is white if  k even and m < te,  or  k odd and m < to, with
            -- delta < 0: te = s - |d|, to = 0;  delta >= 0: te = s, to = |d|.
            if s_dneg_p = '0' or su_m < s_seff_p then c_e1 <= '1'; else c_e1 <= '0'; end if;
            if s_dneg_p = '0' and su_m < s_dabs_p then c_o1 <= '1'; else c_o1 <= '0'; end if;
            -- S9 MOIRE: past half a band, aligned with k_ld1 -> k_ld2
            if su_m >= s_shalf_p then h1 <= '1'; else h1 <= '0'; end if;
            h2 <= h1;
            v_kw := k_ld1;
            v_kw(0) := not ((not k_ld1(0) and c_e1) or (k_ld1(0) and c_o1));
            k_ld2 <= v_kw;
            -- per-ring stepped ramp: triangle fold, DARK at the ring edges
            -- (phase 0), bright mid-ring.  (One delay reg only:
            -- ph1's update runs a cycle after the crossing detect.)
            if ph1_acc(15) = '1' then ph1_f1 <= not ph1_acc(14 downto 7);
            else                      ph1_f1 <= ph1_acc(14 downto 7); end if;
        end if;
    end process p_par;

    ------------------------------------------------------------------------
    -- GEO path: running XOR over the mark line buffer = parity of the
    -- covering-disc count; nesting makes that the innermost disc's parity
    -- (C_NR even).  One pair of equality toggles tracks disc C_NR-1
    -- exactly -- the handoff boundary to the outer tracker.  Gated on the
    -- DELAYED avid so the line's last pixel updates too.
    ------------------------------------------------------------------------
    p_geo : process(clk)
        variable v_q : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p2 <= s_avid_p;
            cl_a1     <= s_xcnt(10 downto 0);
            cl_a2     <= cl_a1;

            if data_in.avid = '1' and s_avid_p = '0' then         -- line start
                s_kx    <= resize(m_kinit_done, 7);
                s_lb_disp <= s_lb_eng;      -- display the bank just written
            elsif s_avid_p = '1' then
                v_q := lbq0;
                if s_lb_disp = '1' then v_q := lbq1; end if;
                if v_q(0) = '1' then
                    if v_q(1) = '1' then s_kx <= s_kx + 1;   -- leave a disc
                    else                 s_kx <= s_kx - 1;   -- enter a disc
                    end if;
                end if;
            end if;

            -- inside the nest <=> some disc covers <=> k < C_NR (the counter
            -- IS the exact hit test -- disc 23's comparators are gone)
            if s_kx /= s_nreff and s_dyn = '1' then
                g2_hit <= '1';
            else
                g2_hit <= '0';
            end if;
            g2_k   <= s_kx;
        end if;
    end process p_geo;

    ------------------------------------------------------------------------
    -- MERGE: inside disc C_NR-1 -> the nest's mark parity (exact circles);
    -- else the outer tracker's ring parity (ring C_NR is even, matching the
    -- modelled discs' 0..C_NR-1 indexing; the tracker counts absolute rings
    -- from the outer centre, so its k parity IS the disc parity out there).
    ------------------------------------------------------------------------
    p_r10 : process(clk)
    begin
        if rising_edge(clk) then
            if g2_hit = '1' then r10_k <= g2_k;
            else                 r10_k <= k_ld2; end if;
            r10_t <= ph1_f1;     -- per-ring ramp (already edge-anchored)
            -- S9 MOIRE: XOR with the HALF-PHASE copy of the rings around the
            -- outer centre (= outer parity xor (r mod s >= s/2) = round(r/s)
            -- mod 2).  That reference only changes at HALF radii -- its two
            -- terms flip together at every full radius -- so it never meets
            -- the true ring edges (no residue dots, no d^2-1 hack needed; the
            -- hack put a 1-px line on the centre row).  At rest the result is
            -- the doubled pattern; with the nest offset, true per-pixel moire.
            r10_h <= (h2 xor k_ld2(0)) and s_s9;
        end if;
    end process p_r10;

    ------------------------------------------------------------------------
    -- GRADIENT (Stepped): EVERY RING carries its own ramp-in/ramp-out --
    -- the per-ring phase triangle (dark at both edges, bright mid-ring,
    -- re-anchored at every true disc edge so the shading cascades with the
    -- rings) -- and K4 picks the FINENESS: zone 0 = the classic hard look,
    -- zones 1..6 = 2/4/8/16/32/64 gray steps within each ring.
    -- S9 Smooth = the SAME per-ring ramp at full 8-bit resolution.
    ------------------------------------------------------------------------
    p_r12 : process(clk)
        variable v_t8  : unsigned(7 downto 0);
        variable v_key : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- K4 (v1.9, 4 settings; S9 Smooth folded into the top one):
            --   0 = classic hard black/white, 1 = 4 steps (black / 2 grays /
            --   white -- v2.0.3, Ron: the 2-step setting looked like the hard
            --   one), 2 = 8 steps, 3 = smooth (the full 8-bit per-ring ramp)
            v_t8 := r10_t;
            case s_gn is
                when "00" =>    v_key := (others => not r10_k(0));
                when "01" =>    v_key := v_t8(7 downto 6) & v_t8(7 downto 6)
                                         & v_t8(7 downto 6) & v_t8(7 downto 6);
                when "10" =>    v_key := v_t8(7 downto 5) & v_t8(7 downto 5) & v_t8(7 downto 6);
                when others =>  v_key := v_t8;
            end case;
            -- S9 MOIRE: invert where the half-phase reference ring is set
            if r10_h = '1' then
                v_key := not v_key;
            end if;
            r12_key <= v_key;
        end if;
    end process p_r12;

    ------------------------------------------------------------------------
    -- key -> studio-range 10-bit luma (64..942); VIDEO: refraction
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_y : unsigned(9 downto 0);
        variable v_we : std_logic;
        variable v_wd : std_logic_vector(29 downto 0);
        variable v_du : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- REFRACTION: the ring field horizontally displaces WHERE the
            -- video is sampled -- each ring bends the picture like a glass
            -- ripple, and the cascade drags the ripples with it.  The input
            -- is written to a BRAM delay line on ACTIVE pixels only and
            -- read 1 + disp back, where
            --   disp = L*key/32, 0..53; L = P12 top 3 bits.
            -- key respects K4/S9: hard rings shear in slices, S9 Smooth gives
            -- true waves.  d = 0 (slider down) realigns exactly -- no shift.
            -- Writes continue 12 cycles past line end, repeating the
            -- held last pixel: without this the write pointer freezes at
            -- avid fall while the output tail still reads, so the last ~7
            -- columns repeated one pixel.  The tail slots also make the
            -- next line's left-edge wrap a clean clamp of this line's edge.
            -- ONE write statement only: a second s_vd assignment in an
            -- elsif branch stops yosys inferring the single write port and
            -- the whole line falls back to flip-flops (9205 LC = 120% chip).
            v_we := '0';
            v_wd := vd_hold;
            if data_in.avid = '1' then
                v_wd    := data_in.y & data_in.u & data_in.v;
                v_we    := '1';
                vd_hold <= data_in.y & data_in.u & data_in.v;
                vd_tc   <= to_unsigned(12, 4);
            elsif vd_tc /= 0 then
                v_we  := '1';
                vd_tc <= vd_tc - 1;
            end if;
            if v_we = '1' then
                s_vd(to_integer(vd_wa)) <= v_wd;
                vd_wa <= vd_wa + 1;
            end if;
            -- pixels available so far on THIS line (saturating)
            if data_in.avid = '0' then
                vd_px <= (others => '0');
            elsif vd_px /= 63 then
                vd_px <= vd_px + 1;
            end if;
            -- one-sided displacement from the field key (v1.8): du =
            -- L*key/32, L = P12 top 3 bits (8 levels) as three gated
            -- shifted terms -> 0..53 px (v1.7: 4 zones, 0..18)
            v_du := (others => '0');
            if s_p12(9) = '1' then v_du := v_du + resize(r12_key(7 downto 3), 6); end if;
            if s_p12(8) = '1' then v_du := v_du + resize(r12_key(7 downto 4), 6); end if;
            if s_p12(7) = '1' then v_du := v_du + resize(r12_key(7 downto 5), 6); end if;
            -- LEFT-EDGE CLEANUP: never read across the line start.  The
            -- displacement is clamped to the pixels this line has actually
            -- produced, so the wave fades in over the first columns
            -- instead of smearing the previous line's tail into the edge.
            if vd_px = 0 then
                v_du := (others => '0');
            elsif v_du > vd_px - 1 then
                v_du := vd_px - 1;
            end if;
            vd_ra <= vd_wa - 1 - v_du;
            vd_q  <= s_vd(to_integer(vd_ra));

            -- video split + hold stages (two plain registers -- the depth
            -- keeps the pipeline at C_LATENCY so both modes tap together)
            r13_y <= unsigned(vd_q(29 downto 20));
            r13_u <= unsigned(vd_q(19 downto 10));
            r13_v <= unsigned(vd_q( 9 downto  0));
            r14_y <= r13_y;
            r14_u <= r13_u;
            r14_v <= r13_v;

            -- output stage.
            -- BLANKING GATE: outside active video emit exactly what ring mode
            -- emits -- neutral chroma, black luma.  Without this the free-
            -- running pipeline puts PROCESSED GARBAGE in the blanking
            -- interval (input blanking chroma 0 scaled by 0.75 = 128, an
            -- illegal non-neutral level sitting where the encoder's per-line
            -- colour reference lives) and the WHOLE picture decodes with
            -- rail-to-rail wrong colours + noisy lines, while every active
            -- pixel is bit-correct.  s_avid_sr(C_LATENCY - 2) here aligns
            -- with the s_avid_sr(C_LATENCY - 1) output tap after this register.
            -- v1.7: the gate now covers BOTH modes -- ring mode used to
            -- emit the last pixel's key (often white) through every hblank.
            -- Ring luma is legal studio range: key 0..255 -> Y 64..942
            -- (64 + 4k - k/2 - k/16, monotonic); v1.6 drove 0..1023, and
            -- codes 0/1023 are the BT.656/1120 sync-word codes.
            if s_avid_sr(C_LATENCY - 2) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            elsif s_vsat = '1' then
                s_out_y <= r14_y;
                s_out_u <= r14_u;
                s_out_v <= r14_v;
            else
                v_y := shift_left(resize(r12_key, 10), 2)
                       - resize(r12_key(7 downto 1), 10)
                       - resize(r12_key(7 downto 4), 10)
                       + to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
                s_out_y <= v_y;
            end if;
        end if;
    end process p_out;

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
    -- ONE set of sync taps for both modes -- the video pipeline is depth-
    -- matched to the ring path, so the mode switch changes DATA only.
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture cascade_diag;
