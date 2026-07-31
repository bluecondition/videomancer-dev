-- cascade.vhd  (v4 -- 24 coupled discs, line-buffer mark render)
--
-- HYPNOS CASCADE: concentric black/white discs that PUSH each other.  Move the
-- centre (K1/K2) and the innermost disc leads; each outer ring holds still until
-- the one inside closes its slack, then is shoved -- a wave of motion travelling
-- outward ring by ring.  Rings never overlap (a band of the other colour always
-- survives) and never deform.  Catch-Up (S10) glides them home concentric.
--
-- v4 rework: 24 modelled discs (v3 had 8) and the coupling slack raised to
-- ~90% of the ring spacing -- a ring now yields only when the one inside has
-- closed to ~10% edge gap, so a drag visibly SQUISHES a deep column of rings.
-- 24 discs no longer fit the v3 pixel path (per-ring comparators) nor the SD
-- hblank (24 rings x 8 engine slots = 208 clks > 138), so:
--
--   RENDER -- 1bpp dual-bank LINE BUFFER of edge MARKS.  The span engine
--     writes each disc's two scanline edges (enter at xl, leave at xr+1) as
--     single-bit marks; the pixel path is ONE running-parity XOR over the
--     marks.  The chain clamp keeps the discs strictly nested, so the count
--     of covering discs at x is contiguous (innermost k covered => k..23 all
--     covered) and, with C_NR even, edge-count parity == k parity -- the same
--     painter's colour v3's priority encoder produced.  Discs overhanging the
--     left screen edge toggle a line-start INIT bit instead of a mark; the
--     displayed bank is cleared behind the read, and a vblank clear pass
--     wipes both banks (stale + power-on marks).  Only disc 23 keeps v3-style
--     equality toggles: its exact in/out is the handoff to the outer tracker.
--
--   ENGINE -- runs a line EARLY: the pass for line N starts at line N-1's
--     active-video RISE (chain RAM, shared multiply and the sqrt unit are
--     all idle during active video -- only the adds-only tracker runs then).
--     One sqrt unit, 15-slot ring cadence: ~370 clks fit even SD's 720-clk
--     active line; the tracker line seed (E_LINE, ~50 clks) stays in the
--     hblank as before.
--
--   GRADIENT (K4) -- the marks carry an enter/leave DIRECTION bit, so the
--     pixel path counts the true ring index k (5 bits, mod 32) instead of a
--     parity; luma = a triangle fold of k over 1/2/4/8/16 rings, scaled by
--     bit replication.  Fold width 1 = the classic hard black/white look.
--     Exact in every mode, dragged or not (the shading rides the real
--     edges).
--
--   OUTER field (rings 24+) -- unchanged v3 incremental r^2 BAND TRACKER
--     centred on disc 23's centre: r^2 advances by 2*dx+1 per pixel, a
--     crossing is a sign bit, boundaries are exact spacing multiples, so the
--     field seams exactly onto disc 23.
--
--   CHAIN -- coupled dead-zone chain in PIXEL units, 24 entries, with an
--     OCTAGONAL coupling clamp (per-axis +/-A, 45-degree +/-A*sqrt2): the
--     slack A ~ 87% of the spacing in every drag direction, so the minimum
--     edge gap at the push point is ~12% of the spacing (the "rings nearly
--     touch before yielding" look).  A plain box clamp at this slack would
--     overlap rings on diagonals.
--
-- No CORDIC, no cos/sin ROM, no per-pixel multiply anywhere; every edge is an
-- exact integer circle.
--
-- Nesting guarantee: the octagon bounds |e[k]-e[k-1]| <= 1.083*A (+1.42 px
-- of back-rotation rounding) in every direction, and the collision cap keeps
-- that strictly below spacing - 1: discs stay strictly nested and adjacent
-- edges stay > 1 px apart on any scanline -- mark writes never collide.
-- (Known corner, as in v3: at extreme spacing the radius clamp can make
-- outermost discs coincide; any parity error from that sits off-screen for
-- all reachable drags.)
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

architecture nacre of program_top is

    constant C_LATENCY : integer := 6;    -- x/read 1 + parity 1 + g2 1 + merge 1 + key 1 + out 1
    constant C_MID     : unsigned(9 downto 0) := to_unsigned(512, 10);   -- neutral chroma

    constant C_DB    : integer := 14;                                    -- detent deadband
    constant C_GSH   : integer := 6;                                     -- glide one-pole shift

    -- CASCADE chain: per-ring centre offsets.  The coupling clamp is an
    -- OCTAGON (per-axis +/-A AND 45-degree +/-C = A*sqrt2): a ring yields
    -- only once the one inside has closed ~90% of the spacing, i.e. the
    -- minimum edge gap at the push point is ~10% of the spacing in ANY drag
    -- direction (a plain box at this slack would overlap on diagonals).
    -- A_px = min(s - s/8, s - s/16 - s/32 - 3) (~0.875*s) keeps every
    -- adjacent edge pair > 1 px apart on any scanline (mark writes never
    -- collide) and nesting strict.  NO accumulated cap -- the wave travels
    -- the chain.  C_NR MUST stay even (mark parity == k parity).
    -- C_NC discs are CHAINED (they push each other; offsets in the chain
    -- RAM).  Rings C_NC..C_NT-1 are RIGID: concentric about the outermost
    -- chained disc, so they cost engine slots but NO storage, and they keep
    -- the screen filled when the nest is smaller than the raster.
    -- A pass must COMPLETE inside one line period (running C_NEARLY lines
    -- ahead buys latency, not throughput -- a pass that overruns simply
    -- misses the next line's trigger).  Ring counts are set by that budget:
    -- only rings out to the screen diagonal are ever visible anyway.
    function f_nc return integer is
    begin
        if C_ENABLE_HD then return 8; else return 6; end if;
    end function;
    function f_nt return integer is
    begin
        -- 10 / 8.  Two constraints set this, not taste: the radix-2 knot
        -- frame is 30 cycles and 2*C_NT runs x >=2 knots each has to fit the
        -- line period (HD 2200, SD 1716 clk) -- AND the knot grid has to be
        -- one ring spacing rather than two, or the field is linear in x
        -- across a whole ring width and the contours come out POLYGONAL.
        -- 12/10 rings at the finer grid left only 4 % throughput margin.
        if C_ENABLE_HD then return 10; else return 8; end if;
    end function;
    -- The pass now fits one line period, so a plain double buffer is enough:
    -- the engine computes line N+1 during line N.  (Running further ahead
    -- buys LATENCY, not throughput -- it never fixed the budget.)
    function f_nearly return integer is
    begin
        return 1;
    end function;
    constant C_NEARLY : integer := f_nearly;
    constant C_NC    : integer := f_nc;
    constant C_NT    : integer := f_nt;
    constant C_NR    : integer := C_NC;    -- chain-facing alias
    constant C_SET   : integer := 3;                                     -- settle glide shift (toward inner)

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
    type t_ring is array(0 to 255) of signed(14 downto 0);   -- 256-deep: EBR
    signal ram_ex, ram_ey : t_ring := (others => (others => '0'));
    signal ea_addr, ew_addr : integer range 0 to 255 := 0;
    signal ew_wex, ew_wey : std_logic := '0';
    signal ew_dinx, ew_diny : signed(14 downto 0) := (others => '0');
    signal doa_ex, doa_ey : signed(14 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- 1bpp mark line buffers, one EBR per bank.  Engine writes the NEXT
    -- line's edge marks into bank s_lb_eng while bank s_lb_disp is read
    -- (and cleared 2 px behind the read) by the pixel path.
    ----------------------------------------------------------------------
    type t_lb is array(0 to 2047) of std_logic_vector(1 downto 0);
    signal s_lb_eng, s_lb_disp : std_logic := '0';

    -- registered mark-write channel (left edge at slot 7; the right edge is
    -- staged in lbp_* and issued one cycle later -- never the same cycle)

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_cx_k, s_cy_k : signed(10 downto 0) := (others => '0');      -- (Kn-512), deadbanded
    signal s_k3_per : unsigned(9 downto 0) := to_unsigned(430, 10);
    signal s_k4_gr  : unsigned(9 downto 0) := (others => '0');
    signal s_gn     : integer range 0 to 6 := 0;   -- 0 = smooth-of-steps off
    signal s_glx, s_gly : std_logic := '1';          -- GLIDE (S7) -- default on
    signal s_dyn    : std_logic := '0';              -- cascade motion (S8)
    signal s_ramp   : std_logic := '0';              -- S9: per-ring ramp Steps/Smooth
    signal s_home   : std_logic := '0';              -- Catch-Up (S10): glide home

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
    -- glide target and step, staged across sequencer slots: target-minus-accum
    -- minus shift plus accum in ONE slot was a 2x22-bit carry chain and, at
    -- 96 % utilisation, the design's critical path
    signal s_tgx, s_tgy : signed(21 downto 0) := (others => '0');
    signal s_dgx, s_dgy : signed(21 downto 0) := (others => '0');
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
    -- SERIAL centre multiplier: the frame needs only two products a frame
    -- ((K-512) * W and * H), so a 10-cycle shift-add pair replaces the
    -- 15x15 array multiplier the cascade lineage carried here.
    signal mx_acc, my_acc : signed(23 downto 0) := (others => '0');
    signal mx_mag, my_mag : unsigned(9 downto 0) := (others => '0');
    signal mx_sgn, my_sgn : std_logic := '0';
    signal mx_i    : integer range 0 to 10 := 10;

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

    ----------------------------------------------------------------------
    -- frame-constant tracker terms (small p_frame tail after the chain,
    -- reusing its shared multiplier)
    ----------------------------------------------------------------------
    signal s_chain_q : std_logic := '0';                          -- chain_run delayed (edge)
    signal s_tlrun : std_logic := '0';
    signal s_tlph  : integer range 0 to 5 := 0;
    signal s_cv_done : std_logic := '0';                          -- pulse: frame terms ready
    signal s_dy0   : signed(14 downto 0) := (others => '0');      -- first active line's dy
    signal s_dx0   : signed(13 downto 0) := (others => '0');      -- dx0' = line-start dx - exo
    signal s_q0    : signed(15 downto 0) := (others => '0');      -- 2*dx0'+1
    signal s_dx0q  : unsigned(26 downto 0) := (others => '0');    -- dx0'^2

    ----------------------------------------------------------------------
    -- SPAN + LINE-SEED engine and the outer-field r^2 band tracker.
    ----------------------------------------------------------------------
    type t_est is (E_IDLE, E_SPAN, E_HERM);
    signal e_st : t_est := E_IDLE;
    signal e_cnt : integer range 0 to C_NT + 1 := 0;              -- ring group counter (2 drain groups)
    signal e_ph  : integer range 0 to 14 := 0;                    -- span slot (15-slot cadence)
    signal e_first : std_logic := '0';                            -- cv_done pass: chain SPAN->LINE
    signal m_a, m_b : signed(14 downto 0) := (others => '0');     -- engine multiply (2-stage)
    signal m_ph     : signed(22 downto 0) := (others => '0');     -- a * b[14:7]
    signal m_pl     : signed(22 downto 0) := (others => '0');     -- a * b[6:0]
    signal m_p      : signed(29 downto 0) := (others => '0');

    signal sp_dy   : signed(14 downto 0) := (others => '0');      -- NEXT line's dy (from inner centre)
    signal sp_ey   : signed(14 downto 0) := (others => '0');
    signal sp_rr   : unsigned(11 downto 0) := (others => '0');    -- ring radius R (accumulated)
    signal sp_ady  : unsigned(14 downto 0) := (others => '0');    -- |dy| pre-clamp (<= 24574)
    signal sp_emp  : std_logic := '0';                            -- |dy| >= R -> empty span
    signal sp_ex   : signed(14 downto 0) := (others => '0');      -- ex of the ring in front-end
    signal sp_exc  : signed(15 downto 0) := (others => '0');      -- cx + ex (RAW screen x of centre)
    signal sp_excq1 : signed(15 downto 0) := (others => '0');      -- delayed to the mark write
    signal sp_empq1, sp_empq2 : std_logic := '0';
    signal sp_wl, sp_wr1 : signed(15 downto 0) := (others => '0');    -- staged edges, raw x (slot 6)

    -- per-pass line-start ring index (C_NR minus the discs overhanging the
    -- left edge): written by the pass (working/_done), latched live at the
    -- line's avid rise.  (The hit test is the counter itself: k < C_NR.)
    signal m_kinit_done : unsigned(4 downto 0) := to_unsigned(C_NR, 5);

    -- ONE 1-step/cycle sqrt unit (15-slot ring cadence; also roots r0).  The
    -- bit mask is a SHIFT REGISTER (>>2 per step), so a step is just an OR +
    -- one 27-bit subtract/compare -- short single-cycle path, 14 steps.
    signal sqa_num, sqa_res, sqa_one : unsigned(26 downto 0) := (others => '0');
    signal sqa_run : std_logic := '0';

    -- line-seed temporaries

    -- band tracker state (seeded per line by E_LINE, stepped per active pixel)
                                                                  -- needs only k mod 2*maxwidth)

    -- SMOOTH-fade machinery: a spacing-1 SUB-TRACKER (same candidate-arm
    -- structure, taps degenerate to +4/0/+2) detects every integer-radius
    -- crossing; a 16-bit phase accumulates +/-Q per crossing and its top
    -- bits fold straight into luma.  Seeded per line by E_LINE.
                                                                  -- SNAPPED to 0 at every disc edge)

    -- outer-field recentre (ring C_NR-1's converted offset), frame-stable

    ----------------------------------------------------------------------
    -- PER-CIRCLE GRADIENT (v0.9).  The field f = d_in/(d_in+d_out) between
    -- each disc and the disc nested inside it is rendered per scanline as a
    -- CUBIC per sub-run (between true edge marks, split at bounding-circle
    -- centre columns + kink ladder + <=128 px pseudo-marks).  The engine
    -- (E_HERM, after E_SPAN, 2 lines early) fits the cubic from exact
    -- endpoint f/df and writes forward-difference seeds; the pixel path is
    -- f += df; df += ddf; ddf += dddf, reseeded at each boundary.
    ----------------------------------------------------------------------
    -- span store: per-disc L/R edges of the line in flight (written by
    -- E_SPAN slot 7, read by E_HERM).  Raw screen x, 16-bit signed.
    type t_spst is array(0 to 255) of signed(15 downto 0);   -- 256-deep: EBR
    signal spst : t_spst := (others => (others => '0'));
    signal spst_wa : integer range 0 to 255 := 0;
    signal spst_we : std_logic := '0';
    signal spst_wd : signed(15 downto 0) := (others => '0');
    signal spst_ra : integer range 0 to 255 := 0;
    signal spst_q  : signed(15 downto 0) := (others => '0');
    -- spst_q clamped to [0, W], registered.  The EBR read (~8 ns) plus the
    -- clamp comparators in ONE cycle was the last HD critical path; this also
    -- collapses three copies of the clamp into one.
    signal sp_cl   : signed(15 downto 0) := (others => '0');

    -- 9-bit reciprocal mantissa ROM: round(2^18 / v) for v = 256..511
    -- (normalized top 9 bits, MSB implied) -> 10-bit mantissa in [512,1024]
    type t_rrom is array(0 to 255) of unsigned(9 downto 0);
    function f_rrom_init return t_rrom is
        variable r : t_rrom;
    begin
        for i in 0 to 255 loop
            r(i) := to_unsigned((2 ** 18 + (256 + i)) / (2 * (256 + i)), 10);
        end loop;
        return r;
    end function;
    constant C_RROM : t_rrom := f_rrom_init;

    -- MSB position of an 18-bit value (0 for v = 0/1)
    function f_msb18(v : unsigned(17 downto 0)) return integer is
    begin
        for i in 17 downto 1 loop
            if v(i) = '1' then return i; end if;
        end loop;
        return 0;
    end function;

    -- seed slot RAM: 2 line banks x 256 slots, one word per LINEAR piece.
    -- word: x_next(11) & f0(18) & d0(28) = 57 bits
    -- seed slot RAM: FOUR line banks x 256 pieces.  The engine runs
    -- C_NEARLY lines ahead of the raster, so it needs more than the classic
    -- double buffer; 4 banks make the bank index a free 2-bit wrap.
    -- 128 slots per bank x 2 banks = 256 deep, so each 16-bit slice is
    -- exactly ONE EBR block and the RAM loses its output mux entirely.
    -- Measured worst case is 26 seeds a line; the frame budget caps it near 63.
    type t_seed is array(0 to 255) of std_logic_vector(56 downto 0);
    signal seedram : t_seed := (others => (others => '0'));
    signal sd_wa : unsigned(7 downto 0) := (others => '0');
    signal sd_we : std_logic := '0';
    signal sd_wd : std_logic_vector(56 downto 0) := (others => '0');
    signal sd_ra : unsigned(7 downto 0) := (others => '0');
    signal sd_q  : std_logic_vector(56 downto 0) := (others => '0');
    signal s_lbank   : unsigned(1 downto 0) := (others => '0');  -- raster bank
    signal eng_bank  : unsigned(1 downto 0) := (others => '0');  -- engine bank
    signal e_pre     : integer range 0 to 3 := 0;   -- vblank pre-fill passes

    -- E_HERM walker state (runs after E_SPAN, 2 lines early; E_LINE may
    -- preempt at knot-complete points, hm_pend resumes it)
    -- knot walker / evaluation state
    signal ks_g   : signed(15 downto 0) := (others => '0');   -- next grid knot
    signal hm_last : std_logic := '0';   -- this knot closes the run
    -- the run in flight is the CENTRAL one.  Its field is a CONE (the inner
    -- circle degenerates towards a point), so edge + centre-column knots
    -- alone interpolate it as a pyramid and the innermost disc renders as a
    -- visible DIAMOND.  It gets half the grid step; the other runs cannot
    -- afford it (SD went over the line budget at a quarter).
    -- ...as a single registered STEP, not a mux at each use site.  Three
    -- 16-bit "half or whole" muxes cost ~130 LC and pushed hd_hdmi off the
    -- end of its seed range.
    signal hm_step : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal ks_n   : signed(15 downto 0) := (others => '0');   -- chosen next knot
    signal ks_n2  : signed(15 downto 0) := (others => '0');   -- and its regrid
    signal s_kstep : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal hk_x   : signed(15 downto 0) := (others => '0');   -- knot x
    signal hk_f   : signed(17 downto 0) := (others => '0');   -- f at the knot, Q16
    signal hw_di  : unsigned(18 downto 0) := (others => '0');
    -- the piece staged for the NEXT knot frame's slope divide
    signal hq_l   : unsigned(11 downto 0) := (others => '0');
    signal hq_df  : signed(10 downto 0) := (others => '0');
    signal hp_x   : signed(15 downto 0) := (others => '0');   -- open piece anchor
    signal hp_f   : signed(17 downto 0) := (others => '0');
    signal hp_v   : std_logic := '0';
    signal hw_xn  : unsigned(10 downto 0) := (others => '0'); -- staged seed word
    signal hw_f0  : signed(17 downto 0) := (others => '0');
    signal hw_pend : std_logic := '0';
    signal hm_dy_n : signed(14 downto 0) := (others => '0');
    signal hm_first : std_logic := '0';
    signal hm_xae, hm_xbe : signed(15 downto 0) := (others => '0');
    signal hm_icx, hm_ocx : signed(15 downto 0) := (others => '0');
    signal hm_idy2, hm_ody2 : unsigned(26 downto 0) := (others => '0');
    signal hm_iR, hm_oR : unsigned(12 downto 0) := (others => '0');
    signal hm_slot : unsigned(6 downto 0) := (others => '0');
    signal hm_side : std_logic := '0';
    signal hm_ph  : integer range 0 to 63 := 0;
    signal hm_ean : integer range 0 to C_NC - 1 := 0;   -- chain read addr
    signal hm_k   : integer range 0 to C_NT := 0;       -- current run's own ring
    signal hm_lowp : integer range 0 to C_NT := C_NT;   -- innermost ring the
                                                        -- scanline crosses
    -- second radix-4 sqrt unit: the two knot distances root in parallel
    signal sqb_num, sqb_res, sqb_one : unsigned(26 downto 0) := (others => '0');
    signal sqb_run : std_logic := '0';
    signal s_rmax : unsigned(12 downto 0) := (others => '0');  -- C_NT * spacing

    -- ONE shared normalise unit: nrm_in is scaled into [256,512) and the
    -- SAME shift is applied to nrm_nm, so every post-multiply shift in the
    -- engine is a CONSTANT -- one priority encoder + barrel on the die.
    signal nrm_in : unsigned(17 downto 0) := (others => '0');
    signal nrm_nm : signed(18 downto 0) := (others => '0');
    signal nrm_sh   : integer range 0 to 17 := 0;          -- stage-1 exponent
    signal nrm_in_q : unsigned(17 downto 0) := (others => '0');
    signal nrm_nm_q : signed(18 downto 0) := (others => '0');
    signal nrm_r    : integer range 0 to 3 := 0;           -- stage-2a residue
    signal nrm_in_c : unsigned(26 downto 0) := (others => '0');
    signal nrm_nm_c : signed(27 downto 0) := (others => '0');
    signal hn     : signed(14 downto 0) := (others => '0');
    signal rrom_a : unsigned(7 downto 0) := (others => '0');
    signal rrom_q : unsigned(9 downto 0) := (others => '0');

    -- pixel-path linear accumulator + seed prefetch
    signal fd_f   : signed(24 downto 0) := (others => '0');   -- Q16 + 6 guard
    signal fd_d   : signed(22 downto 0) := (others => '0');
    signal fd_xn  : unsigned(10 downto 0) := (others => '1'); -- next reseed x
    signal fd_ptr : unsigned(6 downto 0) := (others => '0');  -- seed read pointer
    signal fd_px  : unsigned(10 downto 0) := (others => '0'); -- pixel x at fd stage
    signal fd_ld  : std_logic := '0';                         -- line-start word0 load
    signal fd_lum : unsigned(7 downto 0) := (others => '0');  -- folded per-circle luma

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

    signal r10_t    : unsigned(7 downto 0) := (others => '0');  -- per-ring ramp at the merge
    signal r12_key  : unsigned(7 downto 0) := (others => '0');
    signal s_out_y  : unsigned(9 downto 0) := (others => '0');

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
               else hm_ean when e_st = E_HERM
               else e_cnt when e_cnt < C_NC
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
    ------------------------------------------------------------------------
    -- span store (E_SPAN -> E_HERM) and the per-circle seed slot RAM
    -- (E_HERM -> pixel path; 3 line banks in one 256-deep RAM).
    ------------------------------------------------------------------------
    ------------------------------------------------------------------------
    -- shared NORMALISE + reciprocal ROM.  One priority encoder and one
    -- barrel shifter serve every division in the knot engine: nrm_in is
    -- scaled into [256,512) for the ROM index and nrm_nm gets the SAME
    -- shift, so each post-multiply shift downstream is a constant.
    ------------------------------------------------------------------------
    p_norm : process(clk)
        variable v_nb : integer range 0 to 17;
        variable v_q  : integer range 0 to 4;
        variable v_tp : unsigned(26 downto 0);
        variable v_nu : signed(27 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 1: priority encode ONLY.  Encoder + barrel + clamp in one
            -- cycle measured 22.4 ns (44.7 MHz) -- the whole design's critical
            -- path -- so the barrel now gets a cycle of its own.
            v_nb := 0;
            for i in 17 downto 1 loop
                if nrm_in(i) = '1' then v_nb := i; exit; end if;
            end loop;
            nrm_sh   <= v_nb;
            nrm_in_q <= nrm_in;
            nrm_nm_q <= nrm_nm;

            -- stage 2a: COARSE shift.  Pre-shifting left by a constant 9
            -- turns the whole normalise into a RIGHT shift of nrm_sh+1
            -- (1..18); splitting that into a multiple of 4 here and the
            -- remainder next cycle halves the mux depth.  As one barrel it
            -- measured 14.0 ns / 71 MHz -- the design's critical path.
            v_q := (nrm_sh + 1) / 4;
            nrm_r    <= (nrm_sh + 1) - 4 * v_q;
            nrm_in_c <= shift_right(shift_left(resize(nrm_in_q, 27), 9), 4 * v_q);
            nrm_nm_c <= shift_right(shift_left(resize(nrm_nm_q, 28), 9), 4 * v_q);

            -- stage 2b: FINE shift (0..3) + clamp.  Both operands take the
            -- SAME shift, so every post-multiply shift downstream is constant.
            v_tp := shift_right(nrm_in_c, nrm_r);
            v_nu := shift_right(nrm_nm_c, nrm_r);
            rrom_a <= v_tp(7 downto 0);
            if    v_nu >  16383 then hn <= to_signed(16383, 15);
            elsif v_nu < -16384 then hn <= to_signed(-16384, 15);
            else                     hn <= resize(v_nu, 15); end if;

            -- stage 3: the ROM read indexes the PRE-edge rrom_a, so rrom_q
            -- ALWAYS trails hn by one cycle.  hn at T+4, rrom_q at T+5.
            rrom_q <= C_RROM(to_integer(rrom_a));
        end if;
    end process p_norm;

    p_spst : process(clk)
    begin
        if rising_edge(clk) then
            if spst_we = '1' then spst(spst_wa) <= spst_wd; end if;
            spst_q <= spst(spst_ra);
            -- Clamp to BOTH raster edges: rings wider than the screen have
            -- negative left edges, and a negative run end propagates into
            -- x_next as a huge unsigned value, desynchronising the pixel path
            -- for the rest of the line.  The right clamp is s_W, not s_W-1:
            -- x_next = s_W simply never matches, so the last column keeps its
            -- piece.
            if spst_q > signed(resize(s_W, 16)) then
                sp_cl <= signed(resize(s_W, 16));
            elsif spst_q < 0 then
                sp_cl <= (others => '0');
            else
                sp_cl <= spst_q;
            end if;
        end if;
    end process p_spst;

    p_seedram : process(clk)
    begin
        if rising_edge(clk) then
            if sd_we = '1' then seedram(to_integer(sd_wa)) <= sd_wd; end if;
            sd_q <= seedram(to_integer(sd_ra));
        end if;
    end process p_seedram;

    ------------------------------------------------------------------------
    -- PER-CIRCLE field replay (v0.9): one linear accumulator, reseeded from
    -- the slot RAM at each knot.  f += d per pixel; fold to the ramp luma.
    -- Runs at the ph1/st_v stage so fd_lum meets ph1_f1 at the r10 merge.
    ------------------------------------------------------------------------
    p_fd : process(clk)
        variable v_f  : signed(29 downto 0);
        variable v_t  : unsigned(16 downto 0);
        variable v_tr : unsigned(16 downto 0);
        variable v_nxt, v_cur : unsigned(7 downto 0);   -- bank base addresses
    begin
        if rising_edge(clk) then
            v_nxt := (others => '0');  v_nxt(7) := not s_lbank(0);  -- next line
            v_cur := (others => '0');  v_cur(7) := s_lbank(0);      -- this line
            if data_in.avid = '1' and s_avid_p = '0' then
                -- line start: word 0 is ALREADY in sd_q (parked during
                -- blanking below -- the RAM needs 2 cycles from address to
                -- data, and issuing the address here left the fd_ld load
                -- reading the PREVIOUS line's leftover word), so point at
                -- word 1 now and the first reseed can land on pixel 0
                sd_ra <= v_nxt + 1;
                fd_ptr <= (others => '0');
                fd_px  <= (others => '0');
                fd_ld  <= '1';
            elsif fd_ld = '1' then
                -- word 0: seed the accumulator (word 1 is already in flight)
                fd_f  <= shift_left(resize(signed(sd_q(45 downto 28)), 25), 6);
                fd_d  <= resize(signed(sd_q(27 downto 0)), 23);
                fd_xn <= unsigned(sd_q(56 downto 46));
                fd_ptr <= to_unsigned(1, 7);
                sd_ra <= v_cur + 1;
                fd_ld <= '0';
            elsif s_avid_p = '1' then
                if fd_px = fd_xn then
                    -- knot: reseed from the prefetched word, prefetch next
                    fd_f  <= shift_left(resize(signed(sd_q(45 downto 28)), 25), 6);
                    fd_d  <= resize(signed(sd_q(27 downto 0)), 23);
                    fd_xn <= unsigned(sd_q(56 downto 46));
                    fd_ptr <= fd_ptr + 1;
                    sd_ra <= v_cur + resize(fd_ptr, 8) + 1;
                else
                    fd_f <= fd_f + resize(fd_d, 25);
                end if;
                fd_px <= fd_px + 1;
            else
                -- blanking: park on the NEXT line's word 0 so it is in sd_q
                -- before the raster needs it
                sd_ra <= v_nxt;
            end if;

            -- fold: clamp Q16, triangle (dark at edges, bright mid-ring)
            v_f := resize(shift_right(fd_f, 6), 30);
            if    v_f < 0     then v_t := (others => '0');
            elsif v_f > 65536 then v_t := to_unsigned(65536, 17);
            else                   v_t := resize(unsigned(v_f(16 downto 0)), 17);
            end if;
            if v_t < 32768 then v_tr := v_t;
            else                v_tr := to_unsigned(65536, 17) - v_t; end if;
            if shift_right(v_tr, 7) > 255 then
                fd_lum <= (others => '1');
            else
                fd_lum <= resize(shift_right(v_tr, 7), 8);
            end if;
        end if;
    end process p_fd;

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
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            s_cv_done    <= '0';
            s_chain_q    <= s_chain_run;
            -- serial centre multiply: one bit per cycle, both axes at once
            if mx_i < 10 then
                if mx_mag(mx_i) = '1' then
                    mx_acc <= mx_acc + shift_left(resize(signed('0' & s_W), 24), mx_i);
                end if;
                if my_mag(mx_i) = '1' then
                    my_acc <= my_acc + shift_left(resize(signed('0' & s_H), 24), mx_i);
                end if;
                mx_i <= mx_i + 1;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';

                v_kx := signed('0' & registers_in(0)) - to_signed(512, 11);
                v_ky := signed('0' & registers_in(1)) - to_signed(512, 11);
                if abs(v_kx) < C_DB then v_kx := (others => '0'); end if;
                if abs(v_ky) < C_DB then v_ky := (others => '0'); end if;
                s_cx_k <= v_kx;
                s_cy_k <= v_ky;
                -- kick the serial multiply (magnitude + sign)
                mx_sgn <= v_kx(10);  my_sgn <= v_ky(10);
                if v_kx < 0 then mx_mag <= unsigned(resize(-v_kx, 10));
                else             mx_mag <= unsigned(resize(v_kx, 10)); end if;
                if v_ky < 0 then my_mag <= unsigned(resize(-v_ky, 10));
                else             my_mag <= unsigned(resize(v_ky, 10)); end if;
                mx_acc <= (others => '0');  my_acc <= (others => '0');
                mx_i   <= 0;

                s_k3_per <= unsigned(registers_in(2));
                s_k4_gr  <= unsigned(registers_in(3));
                s_glx <= registers_in(6)(0);   -- S7 GLIDE
                s_gly <= registers_in(6)(0);
                s_dyn <= registers_in(6)(1);   -- S8 Cascade (uniform / sequential)
                s_ramp <= registers_in(6)(2);  -- S9 per-ring ramp: Steps / Smooth
                s_home<= registers_in(6)(3);   -- S10 Catch-Up (stay-dragged / glide-home)

                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then s_ilace <= '1';
                else                              s_ilace <= '0'; end if;

                s_seq <= to_unsigned(20, 5);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    -- CENTRE: c = W/2 + (K-512)*W/256, Q8.  The serial
                    -- multiply (kicked at vsync, 10 cycles) has long since
                    -- finished by slot 3; the chain starts at slot 3 too and
                    -- reads s_cx/s_cy, published at slot 2.
                    when 3 =>
                        -- apply only: one add and a mux (the target and the
                        -- one-pole step were staged at slots 7 and 5)
                        if s_glx = '1' then s_cxs <= s_cxs + s_dgx;
                        else                s_cxs <= s_tgx; end if;
                        if s_gly = '1' then s_cys <= s_cys + s_dgy;
                        else                s_cys <= s_tgy; end if;

                    -- FREQUENCY: Lf(Q6) = -704 + Period*144/256.  The
                    -- decode is pipelined one step per slot (a single-slot
                    -- negate+ROM+barrel cone was hd_hdmi's critical path).
                    when 19 =>
                        s_lf <= to_signed(-704, 16)
                                + resize(shift_right(signed('0' & s_k3_per) * to_signed(144, 9), 8), 16);
                    when 18 =>
                        v_li  := to_integer(shift_right(s_lf, 6));
                        v_lfr := to_integer(unsigned(s_lf(5 downto 0)));
                        s_fnum <= C_MANT(v_lfr);
                        v_fsh := 2 - v_li;
                        if    v_fsh < 2  then v_fsh := 2;
                        elsif v_fsh > 15 then v_fsh := 15; end if;
                        s_sft <= v_fsh;
                        s_nl  <= unsigned(-s_lf);

                    -- SPACING (px per ring) = 2^(-Lf/64) = C_MANT(frac) >> (10-int).
                    -- Lf in [-704,-129] -> int in [2,11] -> spacing in [4,2048].
                    when 15 =>
                        s_mant <= C_MANT(to_integer(s_nl(5 downto 0)));
                        s_sip  <= to_integer(s_nl(9 downto 6));
                    when 14 =>
                        if s_sip > 10 then
                            s_spac <= resize(shift_left(resize(s_mant, 13), s_sip - 10), 13);
                        else
                            s_spac <= resize(shift_right(s_mant, 10 - s_sip), 13);
                        end if;

                    -- octagon slack in px (pure shift-adds, no mult; one
                    -- short step per slot -- the single-slot version was the
                    -- 17.7 ns critical path):
                    --   A = min(s - s/8,  min(s - s/16 - s/32 - 3, 640))
                    -- (~0.875*s; the cap terms keep adjacent edges > 1 px
                    -- apart on any scanline -- mark-write collision safety --
                    -- and |e[23]| inside the 15-bit chain words)
                    --   C = A * ~sqrt2 = A + A/4 + A/8 + A/32 + A/128
                    when 9 =>
                        s_araw <= resize(s_spac - shift_right(s_spac, 3), 12);
                        s_acap <= resize(s_spac - shift_right(s_spac, 4)
                                         - shift_right(s_spac, 5), 13);
                    when 8 =>
                        -- collision/storage cap on the octagon slack
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
                        -- CENTRE target: c = W/2 + (K-512)*W/256, Q8.  The
                        -- serial multiply (kicked at vsync, 10 cycles) has long
                        -- since finished by slot 7.
                        if mx_sgn = '1' then v_ct := -resize(mx_acc, 28);
                        else                 v_ct :=  resize(mx_acc, 28); end if;
                        s_tgx <= shift_left(resize(signed('0' & s_W(11 downto 1)), 22), 8)
                                 + resize(v_ct, 22);
                        if my_sgn = '1' then v_ct := -resize(my_acc, 28);
                        else                 v_ct :=  resize(my_acc, 28); end if;
                        s_tgy <= shift_left(resize(signed('0' & s_H(11 downto 1)), 22), 8)
                                 + resize(v_ct, 22);
                    when 6 =>
                        s_cpx <= resize(s_apx, 13) + resize(shift_right(s_apx, 2), 13)
                                 + resize(shift_right(s_apx, 3), 13);
                    when 5 =>
                        s_cpx <= s_cpx + resize(shift_right(s_apx, 5), 13)
                                 + resize(shift_right(s_apx, 7), 13);
                        -- one-pole glide step (s_cxs only moves at slot 3, so
                        -- reading it here sees the same value slot 3 will use)
                        s_dgx <= shift_right(s_tgx - s_cxs, C_GSH);
                        s_dgy <= shift_right(s_tgy - s_cys, C_GSH);
                        -- K4 STEPS: 7 zones -> 2..64 grey levels per ring
                        s_gn <= to_integer(shift_right(shift_left(resize(s_k4_gr, 14), 3)
                                - resize(s_k4_gr, 14), 10));
                    -- v0.9 per-circle gradient frame constants
                    when 10 =>
                        -- Knot grid.  Every ring edge is already a knot, so
                        -- the grid only has to cover the LONG tangent runs --
                        -- and those runs are what set the engine's per-line
                        -- budget (a pass must finish inside one line period;
                        -- running lines ahead buys latency, not throughput).
                        -- Two ring spacings keeps the field smooth there and
                        -- roughly halves the knot count.
                        -- ONE ring spacing, not two.  At two the only knots
                        -- inside a run are its ends, so the field is linear
                        -- in x across a whole ring width and the iso-contours
                        -- come out as POLYGONS -- the innermost disc renders
                        -- as a visible diamond at HD defaults.  (A small
                        -- raster hides this: at spacing 14 the facets are
                        -- 14 px and look round.)  One spacing halves the
                        -- error, p99 58 -> 23.
                        if s_spac >= 255 then
                            s_kstep <= to_unsigned(255, 8);
                        elsif s_spac < 8 then
                            s_kstep <= to_unsigned(8, 8);
                        else
                            s_kstep <= resize(s_spac, 8);
                        end if;

                    when 4 =>
                        -- outermost ring radius = C_NT * spacing, clamped
                        -- like the span radii.  C_NT is a compile-time
                        -- constant so this folds to shift-adds; do NOT
                        -- hand-code per-C_NT cases (a stale 16 for an
                        -- 8-ring build renders pure black).
                        v_e := resize(signed('0' & (s_spac * to_unsigned(C_NT, 6))), 28);
                        if v_e > C_RCL then v_e := to_signed(C_RCL, 28); end if;
                        s_rmax <= unsigned(v_e(12 downto 0));

                    -- (the sequencer runs slots 20..1 only -- slot 0 never
                    -- executes, so nothing may be scheduled there)

                    -- publish the glided centre in pixels
                    when 2 =>
                        s_cx <= resize(shift_right(s_cxs, 8), 14);
                        s_cy <= resize(shift_right(s_cys, 8), 14);

                    when others => null;
                end case;
            end if;

            --------------------------------------------------------------
            -- frame tail: once the chain has settled, publish the first
            -- active line's dy and pulse cv_done (the engine's frame start).
            --------------------------------------------------------------
            if s_chain_q = '1' and s_chain_run = '0' then
                if s_ilace = '1' and data_in.field_n = '1' then
                    s_dy0 <= resize(-s_cy, 15) + 1;
                else
                    s_dy0 <= resize(-s_cy, 15);
                end if;
                s_cv_done <= '1';
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
    -- travels the full 24-ring chain.
    ------------------------------------------------------------------------
    p_chain : process(clk)
        variable v_mvx, v_mvy : integer;
        variable vinx, viny : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            ew_wex <= '0';                              -- default: no write
            ew_wey <= '0';
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
                if s_seq = 3 then
                    s_chain_wt  <= '0';
                    s_chain_run <= '1';
                end if;
            elsif s_chain_run = '1' and data_in.avid = '0' then
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
                    -- SETTLE glide toward the ring inside: d + floor(-d/8)
                    -- == the old pull-toward-inner exactly.  Also pre-register
                    -- the next clamp's bound pair (keeps the clamp cone to
                    -- register -> compare -> mux).
                    if s_settle = '1' then
                        s_dw <= s_dw + shift_right(-s_dw, C_SET);
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
        variable v_ki : unsigned(4 downto 0);
        variable v_wide : signed(15 downto 0);
        -- radix-4 sqrt step temporaries
        variable v_sqn, v_sqr, v_sqo : unsigned(26 downto 0);
        -- E_HERM temporaries
        variable v_hd : signed(15 downto 0);
        variable v_ix : integer range 0 to 7;
        variable v_do : unsigned(18 downto 0);
        variable v_gp : unsigned(19 downto 0);
        variable v_fp : signed(29 downto 0);
        variable v_dp : signed(35 downto 0);
    begin
        if rising_edge(clk) then
            -- 2-stage engine multiply (partials, then combine)
            m_ph <= m_a * m_b(14 downto 7);
            m_pl <= m_a * signed('0' & m_b(6 downto 0));
            m_p  <= shift_left(resize(m_ph, 30), 7) + resize(m_pl, 30);

            -- write strobes are ONE-SHOT: without these defaults they latch
            -- high and every later cycle rewrites the span store / seed slot
            spst_we <= '0';
            sd_we   <= '0';

            -- the raster's seed bank advances at every active-line start;
            -- vsync parks it so the frame's first line lands on bank 0
            if s_vs_pulse = '1' then
                s_lbank <= (others => '1');
            elsif data_in.avid = '1' and s_avid_p = '0' then
                s_lbank <= s_lbank + 1;
            end if;

            -- the engine bank flips at every active-line start (the pass
            -- launched this line writes the NEXT line's marks)
            if data_in.avid = '1' and s_avid_p = '0' then
                s_lb_eng <= not s_lb_eng;
            end if;

            ------------------------------------------------------------
            -- sqrt units: RADIX-4 (two unrolled radix-2 steps per cycle,
            -- 7 cycles total -- v0.9, feeds the knot engine's cadence).
            -- Two chained 27-bit subtract/compares; loads override.
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
                if sqa_one = 0 then sqa_run <= '0'; end if;
            end if;
            if sqb_run = '1' then
                v_tr := sqb_res or sqb_one;
                if sqb_num >= v_tr then
                    sqb_num <= sqb_num - v_tr;
                    sqb_res <= shift_right(sqb_res, 1) or sqb_one;
                else
                    sqb_res <= shift_right(sqb_res, 1);
                end if;
                sqb_one <= shift_right(sqb_one, 2);
                if sqb_one = 0 then sqb_run <= '0'; end if;
            end if;
            case e_st is

            when E_IDLE =>
                if s_cv_done = '1' then
                    -- frame start: pre-fill lines 0 .. C_NEARLY-1 back to
                    -- back, so the raster never reads an unwritten bank
                    eng_bank <= (others => '0');
                    e_pre    <= C_NEARLY - 1;
                    sp_dy    <= s_dy0;
                    sp_rr    <= resize(s_spac, 12);
                    hm_lowp  <= C_NT;
                    e_cnt    <= 0;  e_ph <= 0;
                    e_st     <= E_SPAN;
                elsif data_in.avid = '1' and s_avid_p = '0' then
                    -- Steady state: this pass computes the line C_NEARLY
                    -- ahead of the one now starting.  BOTH s_lbank and s_dyb
                    -- are read PRE-increment here (their own updates land on
                    -- this same rising edge), hence the +1 / full C_NEARLY.
                    eng_bank <= s_lbank + C_NEARLY + 1;
                    e_pre    <= 0;
                    if s_ilace = '1' then
                        sp_dy <= s_dyb + to_signed(2 * C_NEARLY, 15);
                    else
                        sp_dy <= s_dyb + to_signed(C_NEARLY, 15);
                    end if;
                    sp_rr   <= resize(s_spac, 12);
                    hm_lowp <= C_NT;
                    e_cnt   <= 0;  e_ph <= 0;
                    e_st    <= E_SPAN;
                end if;

            when E_SPAN =>
                case e_ph is
                    when 0 =>
                        null;                         -- chain RAM read settling
                    when 1 =>
                        -- only CHAINED discs read the chain RAM; rigid outer
                        -- rings (>= C_NC) ride disc C_NC-1's offset (no storage)
                        if e_cnt < C_NC then
                            sp_ey <= doa_ey;                  -- both axes arrive together
                            sp_ex <= doa_ex;
                        end if;
                    when 2 =>
                        if e_cnt < C_NT then
                            v_dy := resize(sp_dy, 16) - resize(sp_ey, 16);
                            v_ad := abs(v_dy);                -- <= 12500, bit 15 = 0
                            sp_ady <= unsigned(v_ad(14 downto 0));
                            sp_exc <= resize(sp_ex, 16) + resize(s_cx, 16);  -- raw screen x
                        end if;
                    when 3 =>
                        if e_cnt < C_NT then
                            if sp_ady > C_RCL then sp_emp <= '1'; sp_ady <= to_unsigned(C_RCL, 15);
                            else sp_emp <= '0'; end if;
                        end if;
                    when 4 =>
                        if e_cnt < C_NT then
                            -- w^2 = (R - |dy|) * (R + |dy|)
                            m_a <= signed(resize(sp_rr, 15)) - signed(resize(sp_ady(12 downto 0), 15));
                            m_b <= signed(resize(sp_rr, 15)) + signed(resize(sp_ady(12 downto 0), 15));
                        end if;
                    when 5 | 6 => null;                       -- m_p registers
                    when 7 =>
                        -- stage ring e_cnt-1's edges from the JUST-FINISHED
                        -- sqrt (reads see the pre-load value), then hand the
                        -- unit to ring e_cnt.  14 steps run in slots 8..14
                        -- and 0..6 of the next group.
                        v_w := signed(resize(sqa_res(12 downto 0), 16));
                        sp_wl  <= sp_excq1 - v_w;
                        sp_wr1 <= sp_excq1 + v_w + 1;         -- first x OUTSIDE
                        sp_empq2 <= sp_empq1;
                        if e_cnt < C_NT then
                            if m_p < 0 then sqa_num <= (others => '0');
                            else sqa_num <= unsigned(m_p(26 downto 0)); end if;
                            sqa_res <= (others => '0');
                            sqa_one <= shift_left(to_unsigned(1, 27), 26);
                            sqa_run <= '1';
                            -- next ring's radius
                            if resize(sp_rr, 13) + resize(s_spac, 13) > C_RCL then
                                sp_rr <= to_unsigned(C_RCL, 12);
                            else
                                sp_rr <= sp_rr + resize(s_spac, 12);
                            end if;
                        end if;
                        sp_excq1 <= sp_exc;
                        if m_p < 0 then sp_empq1 <= '1';
                        else            sp_empq1 <= sp_emp; end if;
                    when 8 =>
                        -- publish ring e_cnt-1's LEFT edge to the span store
                        -- (raw screen x; E_HERM clips) and remember the
                        -- innermost disc this scanline actually crosses.
                        if e_cnt >= 1 then
                            spst_wa <= e_cnt - 1;
                            spst_wd <= sp_wl;
                            spst_we <= '1';
                            if sp_empq2 = '0' and hm_lowp = C_NT then
                                hm_lowp <= e_cnt - 1;
                            end if;
                        end if;
                    when 9 =>
                        if e_cnt >= 1 then
                            spst_wa <= C_NT + e_cnt - 1;      -- R edge (raw)
                            spst_wd <= sp_wr1;
                            spst_we <= '1';
                        end if;
                        if e_cnt = C_NT then
                            hm_first <= e_first;              -- -> knot engine
                            e_first  <= '0';
                            hm_ph    <= 48;
                            e_st     <= E_HERM;
                        end if;
                    when others => null;                      -- sqrt stepping
                end case;
                if e_ph = 14 then
                    e_ph <= 0;
                    e_cnt <= e_cnt + 1;
                else
                    e_ph <= e_ph + 1;
                end if;

            ----------------------------------------------------------
            -- E_HERM -- the PER-CIRCLE GRADIENT knot engine.
            --
            -- Walks the scanline's runs in ascending x: left edges
            -- outside-in (own ring k spans L[k]..L[k-1]), then the
            -- innermost present disc's central run (L..R), then the
            -- right edges inside-out (R[k-1]..R[k]).  Every run lies
            -- between circle k-1 (inner) and circle k (own), so
            --      f = d_in / (d_in + d_out)
            -- runs 0 at the inner circle to 1 at its own -- squeezing
            -- where the nested circle crowds it and swelling where it
            -- does not.  f is evaluated at KNOTS (every run end plus a
            -- spacing-derived grid) and the pixel path interpolates
            -- linearly between them, so per pixel it is ONE add.
            --
            -- Per knot: two Euclidean distances (radix-4 sqrt + a
            -- sub-pixel remainder correction -- integer distances are
            -- visibly wrong), one reciprocal for the gap and one for
            -- the piece length.  Every normalise goes through the ONE
            -- shared unit (p_norm -> C_RROM), so the die holds a single
            -- priority encoder + barrel shifter instead of four.
            ----------------------------------------------------------
            when E_HERM =>
                case hm_ph is

                    -- ---------------- line init (40..46) ----------------
                    when 48 =>
                        hm_slot <= (others => '0');
                        hp_v    <= '0';
                        hw_pend <= '0';
                        if hm_lowp = C_NT then
                            hm_ph <= 47;              -- no discs: ground sentinel only
                        else
                            hm_k    <= C_NT - 1;
                            hm_side <= '0';
                            hm_ean  <= C_NC - 1;      -- outermost = rigid
                            spst_ra <= C_NT - 1;      -- xa = L[C_NT-1]
                            hm_oR   <= s_rmax;
                            if hm_lowp = C_NT - 1 then
                                hm_step <= shift_right(s_kstep, 1);
                            else
                                hm_step <= s_kstep;
                            end if;
                            hm_ph   <= 49;
                        end if;
                    when 49 =>
                        if C_NT - 2 < C_NC then hm_ean <= C_NT - 2;
                        else                    hm_ean <= C_NC - 1; end if;
                        hm_ph <= 50;
                    when 50 =>
                        hm_ocx  <= resize(doa_ex, 16) + resize(s_cx, 16);
                        hm_dy_n <= resize(resize(sp_dy, 16) - resize(doa_ey, 16), 15);
                        if C_NT - 1 > hm_lowp then spst_ra <= C_NT - 2;
                        else                       spst_ra <= C_NT + C_NT - 1; end if;
                        hm_ph <= 51;
                    when 51 =>
                        m_a <= resize(hm_dy_n, 15);  m_b <= resize(hm_dy_n, 15);
                        hm_xae <= sp_cl;
                        hm_ph <= 52;                  -- OUTER dy^2 -> m_p at 54
                    when 52 =>
                        hm_icx  <= resize(doa_ex, 16) + resize(s_cx, 16);
                        hm_dy_n <= resize(resize(sp_dy, 16) - resize(doa_ey, 16), 15);
                        hm_iR   <= resize(s_rmax - resize(s_spac, 13), 13);
                        hm_ph <= 53;
                    when 53 =>
                        m_a <= resize(hm_dy_n, 15);  m_b <= resize(hm_dy_n, 15);
                        hm_xbe <= sp_cl;
                        hm_ph <= 54;                  -- INNER dy^2 -> m_p at 56
                    when 54 =>
                        -- m_p here is the multiply issued at 51.  Reading the
                        -- two dy^2 products at 53 and 54 was a ONE-CYCLE-EARLY
                        -- read that left hm_ody2 holding E_SPAN's last w^2, so
                        -- every line's FIRST run had a bogus outer circle --
                        -- masked while the nest is concentric, wrong the
                        -- moment it is dragged.
                        hm_ody2 <= unsigned(m_p(26 downto 0));
                        hm_ph <= 55;
                    when 55 =>
                        hm_ph <= 56;
                    when 56 =>
                        hm_idy2 <= unsigned(m_p(26 downto 0));
                        -- LEADING GROUND piece: flat black from x = 0 up to
                        -- the first edge (the pixel path reseeds at x_next).
                        -- hq_l = 0 makes the staged slope divide produce 0.
                        hw_xn <= unsigned(hm_xae(10 downto 0));
                        hw_f0 <= (others => '0');
                        hq_l  <= (others => '0');
                        hq_df <= (others => '0');
                        hw_pend <= '1';
                        ks_g <= hm_xae + signed(resize(hm_step, 16));
                        hk_x <= hm_xae;
                        if hm_xbe <= hm_xae + 1 then
                            hm_ph <= 30;              -- outermost ring clipped
                        else
                            hm_ph <= 0;
                        end if;

                    -- --------------- knot frame (23 cycles, 0..22) --------
                    -- LATENCIES (get these wrong and the field is garbage):
                    --   shared multiply: operands issued at T, m_p at T+3
                    --   radix-4 sqrt: loaded at T, result readable at T+8
                    --   p_norm: nrm_* loaded at T, hn at T+2, rrom_q at T+3
                    --     (rrom_q trails hn by one cycle -- p_norm's ROM read
                    --      indexes the PRE-edge rrom_a.  Reading them in the
                    --      same cycle used the PREVIOUS knot's reciprocal on
                    --      every divide; that was the whole field being wrong.)
                    -- The previous piece's slope divide is PIPELINED into this
                    -- knot's sqrt shadow (states 2..9), which is what makes the
                    -- frame fit the per-line budget.
                    when 0 =>
                        -- the hk_ti/hk_to registers are gone: the subtract
                        -- feeds the multiply operand register directly (still
                        -- FF-to-FF, and it buys a cycle back off the frame)
                        m_a <= resize(hk_x - hm_icx, 15);
                        m_b <= resize(hk_x - hm_icx, 15);
                        hm_ph <= 1;
                    when 1 =>
                        m_a <= resize(hk_x - hm_ocx, 15);
                        m_b <= resize(hk_x - hm_ocx, 15);
                        nrm_in <= resize(hq_l, 18);          -- previous piece
                        nrm_nm <= resize(hq_df, 19);
                        hm_ph <= 2;
                    when 2 =>
                        hm_ph <= 3;
                    when 3 =>
                        sqa_num <= resize(unsigned(m_p(26 downto 0)) + hm_idy2, 27);
                        sqa_res <= (others => '0');
                        sqa_one <= shift_left(to_unsigned(1, 27), 26);
                        sqa_run <= '1';
                        hm_ph <= 4;
                    when 4 =>
                        sqb_num <= resize(unsigned(m_p(26 downto 0)) + hm_ody2, 27);
                        sqb_res <= (others => '0');
                        sqb_one <= shift_left(to_unsigned(1, 27), 26);
                        sqb_run <= '1';
                        hm_ph <= 5;
                    when 5 =>
                        hm_ph <= 6;
                    when 6 =>
                        m_a <= hn;  m_b <= signed(resize(rrom_q, 15));
                        hm_ph <= 7;
                    when 7 | 8 =>
                        hm_ph <= hm_ph + 1;
                    when 9 =>
                        -- d0 (Q16 + 6 guard per px).  hn carries (df >> 8)
                        -- scaled by the SAME exponent as L, so hn*mant >> 3 is
                        -- df*2^6/L and the conversion is one CONSTANT shift.
                        -- (>>3 on df instead of >>8 saturated hn for every
                        -- piece shorter than 128 px -- the slope came out ~16x
                        -- too small and the field barely moved.)
                        if hw_pend = '1' then
                            v_dp := resize(shift_right(m_p, 3), 36);
                            if    v_dp >  4194303 then v_dp :=  to_signed(4194303, 36);
                            elsif v_dp < -4194304 then v_dp := to_signed(-4194304, 36); end if;
                            sd_wd(27 downto 0) <= std_logic_vector(resize(v_dp, 28));
                        end if;
                        hm_ph <= 10;
                    when 10 =>
                        if hw_pend = '1' then
                            sd_wa <= eng_bank(0) & hm_slot;
                            sd_wd(56 downto 46) <= std_logic_vector(hw_xn);
                            sd_wd(45 downto 28) <= std_logic_vector(hw_f0);
                            sd_we   <= '1';
                            hm_slot <= hm_slot + 1;
                            hw_pend <= '0';
                        end if;
                        hm_ph <= 11;
                    -- radix-2 sqrt: loaded at 3 and 4, readable at 18 and
                    -- 19.  Radix-4 (two chained 27-bit subtracts in one cycle)
                    -- measured 18.2 ns / 55 MHz and was THE critical path of
                    -- the whole design; one step per cycle halves it, paid for
                    -- with these idle states and a smaller ring count.
                    -- The next knot's x does not depend on this knot's f, so
                    -- the whole candidate chain runs HERE, in the sqrt's idle
                    -- window.  As one cycle at the end of the frame it was an
                    -- 18 ns compare/mux/add chain and the critical path.
                    --
                    -- Candidates: the grid, and the run's INNER circle-centre
                    -- column (where r(x) kinks -- without it the central run
                    -- interpolates straight across the innermost disc).  Every
                    -- piece must span >= 2 px, because the pixel path's seed
                    -- prefetch takes 2 cycles and back-to-back reseeds read a
                    -- STALE word; the bounds are folded into the candidate
                    -- tests rather than clamping the winner afterwards.
                    when 11 =>
                        if ks_g + 1 < hm_xbe then ks_n <= ks_g;
                        else                      ks_n <= hm_xbe; end if;
                        hm_ph <= 12;
                    when 12 =>
                        if hm_icx > hk_x + 1 and hm_icx < ks_n then
                            ks_n <= hm_icx;
                        end if;
                        hm_ph <= 13;
                    when 13 =>
                        ks_n2 <= ks_n + signed(resize(hm_step, 16));
                        -- "this knot closes the run" as a FLAG.  Leaving the
                        -- 16-bit compare in state 29 put a carry chain in
                        -- hk_x's load enable -- 18 logic levels, 13.5 ns, and
                        -- hd_hdmi's critical path.  hk_x and hm_xbe are both
                        -- stable across the whole knot frame, so it can be
                        -- decided here.
                        if hk_x >= hm_xbe then hm_last <= '1';
                        else                   hm_last <= '0'; end if;
                        hm_ph <= 14;
                    when 14 | 15 | 16 | 17 =>
                        hm_ph <= hm_ph + 1;
                    when 18 =>
                        -- d_in = r_inner - R_inner (Q6, clamped >= 0).
                        -- Integer sqrt only: the sub-pixel remainder
                        -- correction measured 1.4/255 of mean error and cost
                        -- 8 cycles a knot, which the line budget cannot pay.
                        if shift_left(resize(sqa_res(13 downto 0), 20), 6)
                           > shift_left(resize(hm_iR, 20), 6) then
                            hw_di <= resize(shift_left(resize(sqa_res(13 downto 0), 20), 6)
                                            - shift_left(resize(hm_iR, 20), 6), 19);
                        else
                            hw_di <= (others => '0');
                        end if;
                        hm_ph <= 19;
                    when 19 =>
                        -- d_out AND the gap in one cycle (a clamped subtract
                        -- plus an add -- far short of the norm path), which is
                        -- what keeps the frame at 23 cycles after p_norm grew
                        -- a stage.  gap = d_in + d_out -> recip; the numerator
                        -- is d_in, so both share an exponent and f is a
                        -- constant shift.
                        if shift_left(resize(hm_oR, 20), 6)
                           > shift_left(resize(sqb_res(13 downto 0), 20), 6) then
                            v_do := resize(shift_left(resize(hm_oR, 20), 6)
                                           - shift_left(resize(sqb_res(13 downto 0), 20), 6), 19);
                        else
                            v_do := (others => '0');
                        end if;
                        v_gp := resize(hw_di, 20) + resize(v_do, 20);
                        if    v_gp > 262143 then v_gp := to_unsigned(262143, 20);
                        elsif v_gp < 64     then v_gp := to_unsigned(64, 20); end if;
                        nrm_in <= resize(v_gp, 18);
                        nrm_nm <= resize(signed('0' & hw_di), 19);
                        hm_ph <= 20;
                    when 20 | 21 | 22 | 23 =>
                        hm_ph <= hm_ph + 1;
                    when 24 =>
                        m_a <= hn;  m_b <= signed(resize(rrom_q, 15));
                        hm_ph <= 25;
                    when 25 | 26 =>
                        hm_ph <= hm_ph + 1;
                    when 27 =>
                        v_fp := resize(shift_right(m_p, 1), 30);     -- f, Q16
                        if    v_fp < 0     then hk_f <= (others => '0');
                        elsif v_fp > 65536 then hk_f <= to_signed(65536, 18);
                        else                    hk_f <= resize(v_fp, 18); end if;
                        hm_ph <= 28;
                    when 28 =>
                        -- stage the piece [hp_x, hk_x).  A piece is only
                        -- emitted if it covers pixels: rings far larger than
                        -- the raster clip to zero-length runs, and emitting
                        -- those hands the pixel path two seeds with the same
                        -- x_next, which it can only consume one of per pixel.
                        if hp_v = '1' and hk_x > hp_x then
                            hw_xn <= unsigned(hk_x(10 downto 0));
                            hw_f0 <= hp_f;
                            hq_l  <= unsigned(resize(hk_x - hp_x, 12));
                            hq_df <= resize(shift_right(hk_f - hp_f, 8), 11);
                            hw_pend <= '1';
                        end if;
                        hp_x <= hk_x;  hp_f <= hk_f;  hp_v <= '1';
                        hm_ph <= 29;
                    when 29 =>
                        if hm_last = '1' then
                            hm_ph <= 30;                  -- roll to the next run
                        else
                            hk_x <= ks_n;                 -- chosen in 11..13
                            ks_g <= ks_n2;                -- grid re-anchors
                            hm_ph <= 0;
                        end if;

                    -- --------------- run roll (23..29) ---------------
                    when 30 =>
                        -- Walk order: left edges outside-in, the innermost
                        -- present ring's central run, then right edges
                        -- inside-out.  The walk TERMINATES here (a run must
                        -- be walked in full first, so this cannot key off a
                        -- flag raised when the run was opened).
                        if hm_side = '0' and hm_k > hm_lowp then
                            hm_k <= hm_k - 1;
                            if    hm_k < 2        then hm_ean <= 0;
                            elsif hm_k - 2 < C_NC then hm_ean <= hm_k - 2;
                            else                       hm_ean <= C_NC - 1; end if;
                            if hm_k = hm_lowp + 1 then
                                hm_step <= shift_right(s_kstep, 1);
                            else
                                hm_step <= s_kstep;
                            end if;
                            hm_ph <= 31;
                        elsif hm_side = '0' then
                            hm_side <= '1';               -- central run done
                            if hm_lowp < C_NT - 1 then
                                hm_k <= hm_lowp + 1;
                                if hm_lowp + 1 < C_NC then hm_ean <= hm_lowp + 1;
                                else                       hm_ean <= C_NC - 1; end if;
                                hm_step <= s_kstep;
                                hm_ph <= 31;
                            else
                                hm_ph <= 37;              -- nothing outside it
                            end if;
                        elsif hm_k < C_NT - 1 then
                            hm_k <= hm_k + 1;
                            if hm_k + 1 < C_NC then hm_ean <= hm_k + 1;
                            else                    hm_ean <= C_NC - 1; end if;
                            hm_step <= s_kstep;
                            hm_ph <= 31;
                        else
                            hm_ph <= 37;                  -- outermost done
                        end if;
                    when 31 =>
                        if hm_side = '0' and hm_k > hm_lowp then
                            spst_ra <= hm_k - 1;
                        else
                            spst_ra <= C_NT + hm_k;
                        end if;
                        hm_ph <= 32;
                    when 32 =>
                        -- shuffle the circle context along by one ring
                        if hm_side = '0' then
                            hm_ocx <= hm_icx;  hm_ody2 <= hm_idy2;
                            hm_oR  <= hm_iR;
                            if hm_k = 0 then hm_iR <= (others => '0');
                            else             hm_iR <= resize(hm_iR - resize(s_spac, 13), 13); end if;
                        else
                            hm_icx <= hm_ocx;  hm_idy2 <= hm_ody2;
                            hm_iR  <= hm_oR;
                            hm_oR  <= resize(hm_oR + resize(s_spac, 13), 13);
                        end if;
                        hm_dy_n <= resize(resize(sp_dy, 16) - resize(doa_ey, 16), 15);
                        hm_ph <= 33;
                    when 33 =>
                        -- doa_ex still holds ram_ex(hm_ean) here (the address
                        -- settled two cycles ago), so the hm_icx0 staging
                        -- register is unnecessary
                        if hm_side = '0' then hm_icx <= resize(doa_ex, 16) + resize(s_cx, 16);
                        else                  hm_ocx <= resize(doa_ex, 16) + resize(s_cx, 16); end if;
                        m_a <= resize(hm_dy_n, 15);  m_b <= resize(hm_dy_n, 15);
                        hm_xae <= hm_xbe;                 -- the shared edge (the
                                                          -- run just closed; hk_x
                                                          -- is stale after a skip)
                        ks_g   <= hm_xbe + signed(resize(hm_step, 16));
                        hm_ph <= 34;
                    when 34 =>
                        hm_xbe <= sp_cl;
                        hm_ph <= 35;
                    when 35 =>
                        hm_ph <= 36;
                    when 36 =>
                        if hm_side = '0' then hm_idy2 <= unsigned(m_p(26 downto 0));
                        else                  hm_ody2 <= unsigned(m_p(26 downto 0)); end if;
                        -- The shared edge belongs to BOTH runs but with
                        -- different values (f jumps 1 -> 0 across a ring
                        -- edge), so it must be re-evaluated in the new
                        -- context as this run's OPENING knot: hp_v = '0'
                        -- makes it emit no piece, only set the anchor.
                        -- Skipping straight to the next grid knot instead
                        -- leaves the span from the edge unrendered, and the
                        -- previous piece then smears across it.
                        hp_v <= '0';
                        if hm_xbe <= hm_xae + 1 then
                            hm_ph <= 30;              -- run clipped away: skip
                        else
                            hk_x <= hm_xae;           -- the shared edge
                            hm_ph <= 0;
                        end if;

                    -- --------------- finish (30..38) ---------------
                    -- the last piece's slope divide has no next knot to ride
                    -- in, so it is run out here once per line
                    when 37 =>
                        nrm_in <= resize(hq_l, 18);
                        nrm_nm <= resize(hq_df, 19);
                        hm_ph <= 38;
                    when 38 | 39 | 40 | 41 =>
                        hm_ph <= hm_ph + 1;
                    when 42 =>
                        m_a <= hn;  m_b <= signed(resize(rrom_q, 15));
                        hm_ph <= 43;
                    when 43 | 44 =>
                        hm_ph <= hm_ph + 1;
                    when 45 =>
                        if hw_pend = '1' then
                            v_dp := resize(shift_right(m_p, 3), 36);
                            if    v_dp >  4194303 then v_dp :=  to_signed(4194303, 36);
                            elsif v_dp < -4194304 then v_dp := to_signed(-4194304, 36); end if;
                            sd_wd(27 downto 0) <= std_logic_vector(resize(v_dp, 28));
                        end if;
                        hm_ph <= 46;
                    when 46 =>
                        if hw_pend = '1' then
                            sd_wa <= eng_bank(0) & hm_slot;
                            sd_wd(56 downto 46) <= std_logic_vector(hw_xn);
                            sd_wd(45 downto 28) <= std_logic_vector(hw_f0);
                            sd_we   <= '1';
                            hm_slot <= hm_slot + 1;
                            hw_pend <= '0';
                        end if;
                        hm_ph <= 47;
                    when 47 =>
                        -- TRAILING GROUND sentinel: x_next = 2047 never
                        -- matches, so the rest of the line stays flat
                        sd_wa <= eng_bank(0) & hm_slot;
                        sd_wd <= (others => '0');
                        sd_wd(56 downto 46) <= (others => '1');
                        sd_we <= '1';
                        hm_ph <= 57;
                    when others =>
                        -- more lines to pre-fill (frame start), else idle
                        if e_pre /= 0 then
                            e_pre   <= e_pre - 1;
                            eng_bank <= eng_bank + 1;
                            if s_ilace = '1' then sp_dy <= sp_dy + 2;
                            else                  sp_dy <= sp_dy + 1; end if;
                            sp_rr   <= resize(s_spac, 12);
                            hm_lowp <= C_NT;
                            e_cnt   <= 0;  e_ph <= 0;
                            e_st    <= E_SPAN;
                        else
                            e_st <= E_IDLE;
                        end if;
                end case;

            end case;
        end if;
    end process p_engine;

    ------------------------------------------------------------------------
    -- tracker parity, delayed to meet the mark-parity path at the merge
    ------------------------------------------------------------------------

    ------------------------------------------------------------------------
    -- GEO path: running XOR over the mark line buffer = parity of the
    -- covering-disc count; nesting makes that the innermost disc's parity
    -- (C_NR even).  One pair of equality toggles tracks disc C_NR-1
    -- exactly -- the handoff boundary to the outer tracker.  Gated on the
    -- DELAYED avid so the line's last pixel updates too.
    ------------------------------------------------------------------------

    ------------------------------------------------------------------------
    -- MERGE: inside disc C_NR-1 -> the nest's mark parity (exact circles);
    -- else the outer tracker's ring parity (ring C_NR is even, matching the
    -- modelled discs' 0..C_NR-1 indexing; the tracker counts absolute rings
    -- from the outer centre, so its k parity IS the disc parity out there).
    ------------------------------------------------------------------------
    p_r10 : process(clk)
    begin
        if rising_edge(clk) then
            r10_t <= fd_lum;             -- the per-circle field, folded
        end if;
    end process p_r10;

    ------------------------------------------------------------------------
    -- GRADIENT: r10_t is each ring's own gradient -- dark where it meets
    -- the circle nested inside it, bright mid-ring, dark again at its own
    -- edge -- so it squeezes and swells with the cascade.  S9 Smooth keeps
    -- the full 8 bits; Steps quantises it to K4's 2..64 levels (zone 0 is
    -- the classic hard black/white).
    ------------------------------------------------------------------------
    p_r12 : process(clk)
        variable v_t8 : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            v_t8 := r10_t;
            if s_ramp = '1' then
                r12_key <= v_t8;
            else
                case s_gn is
                    when 0 =>       -- classic hard edge
                        if v_t8(7) = '1' then r12_key <= (others => '1');
                        else                  r12_key <= (others => '0'); end if;
                    when 1 =>
                        r12_key <= (others => v_t8(7));
                    when 2 =>
                        r12_key <= v_t8(7 downto 6) & v_t8(7 downto 6)
                                   & v_t8(7 downto 6) & v_t8(7 downto 6);
                    when 3 =>
                        r12_key <= v_t8(7 downto 5) & v_t8(7 downto 5) & v_t8(7 downto 6);
                    when 4 =>
                        r12_key <= v_t8(7 downto 4) & v_t8(7 downto 4);
                    when 5 =>
                        r12_key <= v_t8(7 downto 3) & v_t8(7 downto 5);
                    when others =>
                        r12_key <= v_t8(7 downto 2) & v_t8(7 downto 6);
                end case;
            end if;
        end if;
    end process p_r12;

    ------------------------------------------------------------------------
    -- key -> true-black..true-white 10-bit luma (endpoints exact)
    ------------------------------------------------------------------------
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            s_out_y <= shift_left(resize(r12_key, 10), 2)
                       or resize(r12_key(7 downto 6), 10);
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
    data_out.u       <= std_logic_vector(C_MID);
    data_out.v       <= std_logic_vector(C_MID);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture nacre;
