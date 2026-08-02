-- Author: bluecondition

-- cascade.vhd  (v0.9 -- 96/48 coupled discs, line-buffer mark render)
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
--   GRADIENT (K4/S9/S11) -- the marks carry an enter/leave DIRECTION bit, so
--     the pixel path counts the true ring index; every ring carries its own
--     ramp (re-anchored at each true disc edge), K4 sets the fineness, S9
--     picks Steps/Smooth and S11 the multi-ring fade.
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

architecture cascade of program_top is

    constant C_LATENCY : integer := 6;    -- RINGS: x/read 1 + parity 1 + g2 1 + merge 1 + key 1 + out 1
    constant C_VLAT    : integer := 3;    -- VIDEO: split 1 + multiply 1 + recentre 1
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
    signal s_gn     : integer range 0 to 6 := 0;     -- gradient: fold k over 2^(gn+1) rings (0 = hard)
    signal s_qph    : unsigned(13 downto 0) := (others => '0');  -- smooth phase step / radial px
    signal s_q1h    : signed(13 downto 0) := (others => '0');    -- 2^(17-sft-n), pre-decoded
    signal s_qp1    : unsigned(14 downto 0) := (others => '0');  -- stepped step: 2^16/s (per-ring)
    signal s_h1s    : signed(13 downto 0) := (others => '0');    -- 2^(16-sft), pre-decoded
    signal s_sm1    : unsigned(11 downto 0) := to_unsigned(41, 12);  -- spacing - 1 (m wrap)
    signal s_glx, s_gly : std_logic := '1';          -- GLIDE (S7) -- default on
    signal s_dyn    : std_logic := '0';              -- cascade motion (S8)
    signal s_ramp   : std_logic := '0';              -- S9: per-ring ramp Steps/Smooth
    signal s_home   : std_logic := '0';              -- Catch-Up (S10): glide home
    signal s_vsat   : std_logic := '0';              -- S11: Rings / Video saturation mod

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
    signal s_mph   : signed(20 downto 0) := (others => '0');      -- a * b[13:7]
    signal s_mpl   : signed(21 downto 0) := (others => '0');      -- a * b[6:0]
    signal s_mp    : signed(27 downto 0) := (others => '0');      -- 2-stage product

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
    signal s_tlph  : integer range 0 to 5 := 0;
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
    signal l_ph  : integer range 0 to 45 := 0;                    -- line-seed slot
    signal e_first : std_logic := '0';                            -- cv_done pass: chain SPAN->LINE
    signal l_pend  : std_logic := '0';                            -- avid fell while SPAN still busy
    signal cl_cnt  : unsigned(11 downto 0) := (others => '0');    -- E_CLEAR address counter
    signal cl_end  : unsigned(11 downto 0) := (others => '0');    -- E_CLEAR stop (W+2, latched)
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
    signal m_kinit      : unsigned(6 downto 0) := to_unsigned(C_NR, 7);
    signal m_kinit_done : unsigned(6 downto 0) := to_unsigned(C_NR, 7);
    signal m0_w, m0_done : std_logic := '0';          -- a mark lands at x=0 (snap the seed)

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

    -- SMOOTH-fade machinery: a spacing-1 SUB-TRACKER (same candidate-arm
    -- structure, taps degenerate to +4/0/+2) detects every integer-radius
    -- crossing; a 16-bit phase accumulates +/-Q per crossing and its top
    -- bits fold straight into luma.  Seeded per line by E_LINE.
    signal su_u    : signed(16 downto 0) := (others => '0');      -- r^2 - rint^2
    signal su_qmd  : signed(17 downto 0) := (others => '0');      -- q - D1
    signal su_qpdm : signed(17 downto 0) := (others => '0');      -- q + D1 - 2
    signal ph_acc  : unsigned(15 downto 0) := (others => '0');    -- fold phase (wraps = period)
    signal ph_f1, ph_f2 : unsigned(7 downto 0) := (others => '0');-- folded luma, delayed to merge
    signal ph1_acc : unsigned(15 downto 0) := (others => '0');    -- PER-RING phase (period 1 ring;
                                                                  -- SNAPPED to 0 at every disc edge)
    signal ph1_f1  : unsigned(7 downto 0) := (others => '0');
    signal su_cr   : std_logic_vector(1 downto 0) := "00";        -- crossing dir, 1-cycle delayed
    signal st_v    : std_logic := '0';                            -- s_avid_p, delayed
    signal lb_lo   : signed(14 downto 0) := (others => '0');      -- B_lo maintained thru corrections
    signal le_d    : signed(14 downto 0) := (others => '0');      -- d0 = r0 - B_lo
    signal ph_base : unsigned(15 downto 0) := (others => '0');    -- (k0 mod 2w) << (15-n)
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
    signal r10_s    : unsigned(7 downto 0) := (others => '0');  -- smooth luma at the merge
    signal r10_t    : unsigned(7 downto 0) := (others => '0');  -- per-ring ramp at the merge
    signal r12_key  : unsigned(7 downto 0) := (others => '0');
    signal s_out_y  : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S11 VIDEO SATURATION MOD.  The rendered ring field scales the input's
    -- CHROMA about neutral: field black takes 50 % OFF the saturation,
    -- mid-grey leaves it EXACTLY UNCHANGED, white adds 50 %.  Luma passes
    -- through untouched -- the full incoming picture goes out, only its
    -- saturation is modulated, and the circles are never drawn.
    --   m    = 16 + (key + 4) / 8       -- 16..48, exact at black/grey/white
    --   c'   = (c * m) / 32             -- ONE 11x7 multiply per component
    -- so saturation is -50 % / UNCHANGED / +50 % at black / mid-grey / white
    -- (+-25 % was HW-approved but "not prominent enough").  Boost overflows
    -- (1.5 * 512 = 768), so the output is clamped to 0..1023.
    -- The video is NOT delayed to meet the field.  It used to go through a
    -- 30-bit BRAM delay line, which simulated pixel-exact but put noisy
    -- horizontal lines on real hardware -- silicon and GHDL do not agree on
    -- that inferred memory.  It is unnecessary anyway: the ring field is a
    -- SMOOTH pattern, so letting the gain lag the picture by a few pixels
    -- shifts the saturation pattern sideways by an invisible amount.  So the
    -- video takes its own 3-cycle path and the syncs are tapped to match.
    ----------------------------------------------------------------------
    signal r12_h   : unsigned(5 downto 0) := (others => '0');   -- sat step 0..32
    signal r13_cu, r13_cv : signed(10 downto 0) := (others => '0');
    signal r13_y   : unsigned(9 downto 0) := (others => '0');
    signal r14_pu, r14_pv : signed(17 downto 0) := (others => '0');
    signal r14_y   : unsigned(9 downto 0) := (others => '0');
    signal s_out_u, s_out_v : unsigned(9 downto 0) := C_MID;

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
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            s_cv_done    <= '0';
            s_chain_q    <= s_chain_run;
            -- 2-stage shared multiply (partials, then combine)
            s_mph <= s_ma * s_mb(13 downto 7);
            s_mpl <= s_ma * signed('0' & s_mb(6 downto 0));
            s_mp  <= shift_left(resize(s_mph, 28), 7) + resize(s_mpl, 28);

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
                s_glx <= registers_in(6)(0);   -- S7 GLIDE
                s_gly <= registers_in(6)(0);
                s_dyn <= registers_in(6)(1);   -- S8 Cascade (uniform / sequential)
                s_ramp <= registers_in(6)(2);  -- S9 per-ring ramp: Steps / Smooth
                s_home<= registers_in(6)(3);   -- S10 Catch-Up (stay-dragged / glide-home)
                s_vsat<= registers_in(6)(4);   -- S11 Video (rings / saturation-mod the input)

                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then s_ilace <= '1';
                else                              s_ilace <= '0'; end if;

                s_seq <= to_unsigned(20, 5);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    -- centre X: cx = W/2 + (K1-512)*W/256, via the shared mult
                    -- (2-stage mult: operands at slot T, product readable T-3)
                    when 20 => s_ma <= resize(s_cx_k, 14); s_mb <= resize(signed('0' & s_W), 14);
                    when 17 =>
                        v_ct := s_mp;
                        v_targ := shift_left(resize(signed('0' & s_W(11 downto 1)), 22), 8)
                                  + resize(v_ct, 22);
                        if s_glx = '1' then s_cxs <= s_cxs + shift_right(v_targ - s_cxs, C_GSH);
                        else                s_cxs <= v_targ; end if;

                    -- centre Y
                    when 16 => s_ma <= resize(s_cy_k, 14); s_mb <= resize(signed('0' & s_H), 14);
                    when 13 =>
                        v_ct := s_mp;
                        v_targ := shift_left(resize(signed('0' & s_H(11 downto 1)), 22), 8)
                                  + resize(v_ct, 22);
                        if s_gly = '1' then s_cys <= s_cys + shift_right(v_targ - s_cys, C_GSH);
                        else                s_cys <= v_targ; end if;

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
                    -- stepped per-ring phase step Q1 = 2^16/s = fnum*2^(16-sft)>>8
                    when 12 =>
                        s_h1s <= shift_left(to_signed(1, 14), 16 - s_sft);
                    when 11 =>
                        s_ma <= signed(resize(s_fnum, 14));
                        s_mb <= s_h1s;
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
                        -- Q1 capture (slot-11 product; clamp to the 15-bit
                        -- signed multiplier operand range)
                        if unsigned(s_mp(23 downto 8)) > 16383 then
                            s_qp1 <= to_unsigned(16383, 15);
                        else
                            s_qp1 <= resize(unsigned(s_mp(22 downto 8)), 15);
                        end if;
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
                        -- K4 GRADIENT width: 7 zones -> fold over
                        -- 1/2/4/8/16/32/64 rings; n = (K4*7) >> 10
                        s_gn <= to_integer(shift_right(shift_left(resize(s_k4_gr, 14), 3)
                                - resize(s_k4_gr, 14), 10));
                    when 5 =>
                        s_cpx <= s_cpx + resize(shift_right(s_apx, 5), 13)
                                 + resize(shift_right(s_apx, 7), 13);
                        -- pre-decode the Q operand (a one-hot in s_mb's mux
                        -- cone was hd_hdmi's critical path).  Exponent floors
                        -- at 0 (wide fold x huge spacing: Q clamps to
                        -- ~fnum>>10, a near-flat fade -- period maxed out)
                        v_li := 17 - s_sft - s_gn;
                        if v_li < 0 then v_li := 0; end if;
                        s_q1h <= shift_left(to_signed(1, 14), v_li);
                    -- (the sequencer runs slots 20..1 only -- slot 0 never
                    -- executes, so nothing may be scheduled there)
                    when 4 =>
                        -- smooth-fade phase step per radial px:
                        -- Q = fnum * 2^(17-sft-n) >> 10 = 2^16/(2w*s)
                        s_ma <= signed(resize(s_fnum, 14));
                        s_mb <= s_q1h;
                    when 1 =>
                        s_qph <= unsigned(s_mp(23 downto 10));
                        s_sm1 <= resize(s_spac - 1, 12);

                    -- publish the glided centre in pixels
                    when 2 =>
                        s_cx <= resize(shift_right(s_cxs, 8), 14);
                        s_cy <= resize(shift_right(s_cys, 8), 14);

                    when others => null;
                end case;
            end if;

            --------------------------------------------------------------
            -- frame-terms tail: chain done -> the tracker's frame constants
            -- through the same shared multiplier (5 cycles).
            --------------------------------------------------------------
            if s_chain_q = '1' and s_chain_run = '0' then
                s_tlrun <= '1'; s_tlph <= 0;
            elsif s_tlrun = '1' then
                case s_tlph is
                    when 0 =>
                        s_dx0 <= resize(-s_cx, 14) - resize(s_exo, 14);
                    when 1 =>
                        s_ma <= s_dx0;  s_mb <= s_dx0;            -- dx0'^2
                        s_q0 <= resize(shift_left(resize(s_dx0, 16), 1) + 1, 16);
                    when 2 | 3 => null;                -- 2-stage product in flight
                    when 4 =>
                        s_dx0q <= unsigned(s_mp(26 downto 0));
                    when others =>                     -- phase 5: publish + done
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
                if s_tlph /= 5 then s_tlph <= s_tlph + 1; end if;
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
                if s_seq = 3 then
                    s_chain_wt  <= '0';
                    s_chain_run <= '1';
                    -- ring 1's radius = 2*spacing (this frame's spacing lands
                    -- at sequencer slot 14, so it must be read HERE, not at
                    -- vsync); floor the effective ring count at 2
                    s_racc <= resize(s_spac, 14) + resize(s_spac, 14);
                    s_nrw  <= to_unsigned(2, 7);
                    s_codd <= '1';                  -- scan starts at ring 1
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
            -- 2-stage engine multiply (partials, then combine)
            m_ph <= m_a * m_b(14 downto 7);
            m_pl <= m_a * signed('0' & m_b(6 downto 0));
            m_p  <= shift_left(resize(m_ph, 30), 7) + resize(m_pl, 30);

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
                -- pixel: phase +/- Q (smooth fade), and the m = r mod s
                -- counter wraps -> ring index +/-1 (stepped mode).  This
                -- replaced the 28-bit r^2-bracket tracker outright.
                v_s1 := resize(su_u, 18) + resize(t_q, 18);
                v_o1 := resize(su_u, 18) + su_qmd;
                v_i1 := resize(su_u, 18) + su_qpdm;
                if v_s1 < 0 then                          -- inward: r-1
                    su_u    <= resize(v_i1, 17);
                    su_qmd  <= su_qmd + 4;
                    ph_acc  <= ph_acc - resize(s_qph, 16);
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
                    ph_acc  <= ph_acc + resize(s_qph, 16);
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
                    cl_end   <= resize(s_W, 12) + 2;  -- equality exit (a >= carry
                    e_st     <= E_CLEAR;              -- chain here was critical)
                elsif data_in.avid = '1' and s_avid_p = '0' then
                    sp_rr  <= resize(s_spac, 12);     -- pass for the NEXT line
                    m_kinit <= s_nreff;
                    m0_w   <= '0';
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
                if cl_cnt = cl_end then
                    sp_dy  <= s_dy0;                  -- frame's first line
                    sp_rr  <= resize(s_spac, 12);
                    m_kinit <= s_nreff;
                    m0_w   <= '0';
                    e_cnt  <= 0;  e_ph <= 0;
                    e_st   <= E_SPAN;
                end if;

            when E_SPAN =>
                case e_ph is
                    when 0 =>
                        -- chain RAM read settling; per-line trigger grabs the
                        -- next line's dy here (s_dyb updated at the rise edge)
                        if e_cnt = 0 and e_first = '0' then
                            sp_dy <= s_dyb;
                        end if;
                    when 1 =>
                        if e_cnt < to_integer(s_nreff) then
                            sp_ey <= doa_ey;                  -- both axes arrive together
                            sp_ex <= doa_ex;
                        end if;
                    when 2 =>
                        if e_cnt < to_integer(s_nreff) then
                            v_dy := resize(sp_dy, 16) - resize(sp_ey, 16);
                            v_ad := abs(v_dy);                -- <= 12500, bit 15 = 0
                            sp_ady <= unsigned(v_ad(14 downto 0));
                            sp_exc <= resize(sp_ex, 16) + resize(s_cx, 16);  -- raw screen x
                        end if;
                    when 3 =>
                        if e_cnt < to_integer(s_nreff) then
                            if sp_ady > C_RCL then sp_emp <= '1'; sp_ady <= to_unsigned(C_RCL, 15);
                            else sp_emp <= '0'; end if;
                        end if;
                    when 4 =>
                        if e_cnt < to_integer(s_nreff) then
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
                        if e_cnt < to_integer(s_nreff) then
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
                        -- ring e_cnt-1: DIRECTION-coded edge marks -> engine
                        -- bank (left/enter now, right/leave staged one cycle);
                        -- discs overhanging the left screen edge decrement
                        -- the line-start ring index instead of a mark.
                        if e_cnt >= 1 then
                            v_ki := m_kinit;
                            v_wide := signed(resize(s_W, 16));
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
                            -- last ring written: capture line-start k
                            if e_cnt = to_integer(s_nreff) then
                                m_kinit_done <= v_ki;
                                m0_done      <= m0_w;
                                -- drain group done
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
                case l_ph is
                    when 0 =>
                        le_dyp <= resize(resize(sp_dy, 16) - resize(s_eyo, 16), 14);
                        t_q <= s_q0;                                      -- 2*dx0'+1 (frame tail)
                    when 1 =>
                        m_a <= resize(le_dyp, 15);  m_b <= resize(le_dyp, 15);
                    when 2 | 3 => null;                                   -- 2-stage product
                    when 4 =>
                        le_r0sq <= s_dx0q + unsigned(m_p(26 downto 0));   -- dx0'^2 + dy'^2
                    when 5 =>                                             -- root r0^2 on unit A
                        sqa_num <= le_r0sq;
                        sqa_res <= (others => '0');
                        sqa_one <= shift_left(to_unsigned(1, 27), 26);
                        sqa_run <= '1';
                    when 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13
                       | 14 | 15 | 16 | 17 | 18 | 19 => null;             -- 14 sqrt steps
                    when 20 =>
                        m_a <= signed(resize(sqa_res(13 downto 0), 15));  -- r0
                        m_b <= signed(resize(s_fnum, 15));
                    when 21 | 22 => null;
                    when 23 =>
                        le_k0p <= unsigned(m_p(24 downto 0));             -- r0 * fnum
                    when 24 =>
                        t_k <= resize(shift_right(le_k0p, s_sft + 8), 12);
                    when 25 =>
                        m_a <= signed(resize(t_k, 15));                   -- B_lo = k * spacing
                        m_b <= signed(resize(s_spac, 15));
                    when 26 | 27 => null;
                    when 28 =>
                        lb_lo <= m_p(14 downto 0);
                    when 29 =>
                        -- d0 = r0 - B_lo (estimate error is a few multiples
                        -- of s -- corrected LINEARLY below, 15-bit ops only;
                        -- the whole r^2-bracket machinery is gone)
                        le_d <= signed(resize(sqa_res(13 downto 0), 15)) - lb_lo;
                    when 30 | 31 | 32 | 33 | 34 | 35 =>
                        v_dy := resize(le_d, 16) - signed(resize(s_spac, 16));
                        if le_d < 0 then
                            t_k  <= t_k - 1;
                            le_d <= le_d + signed(resize(s_spac, 15));
                        elsif v_dy >= 0 then
                            t_k  <= t_k + 1;
                            le_d <= resize(v_dy, 15);
                        end if;
                    when 36 =>
                        -- publish: ring index, ring phase m = d0, and the
                        -- radius-tracker seeds (u1 = the sqrt REMAINDER,
                        -- free; D1 = 2*r0+1)
                        t_kl  <= t_k(6 downto 0);
                        su_m  <= unsigned(le_d(11 downto 0));
                        su_u  <= signed(resize(sqa_num(15 downto 0), 17));
                        su_d1 <= signed(resize(shift_left(resize(sqa_res(13 downto 0), 15), 1), 16)) + 1;
                        -- (k0 mod 2w) << (15-n)
                        case s_gn is
                            when 0 => ph_base <= unsigned(t_k(0 downto 0)) & "000000000000000";
                            when 1 => ph_base <= unsigned(t_k(1 downto 0)) & "00000000000000";
                            when 2 => ph_base <= unsigned(t_k(2 downto 0)) & "0000000000000";
                            when 3 => ph_base <= unsigned(t_k(3 downto 0)) & "000000000000";
                            when 4 => ph_base <= unsigned(t_k(4 downto 0)) & "00000000000";
                            when 5 => ph_base <= unsigned(t_k(5 downto 0)) & "0000000000";
                            when others => ph_base <= unsigned(t_k(6 downto 0)) & "000000000";
                        end case;
                    when 37 =>
                        m_a <= resize(le_d, 15);                          -- d0 * Q
                        m_b <= signed(resize(s_qph, 15));
                    when 38 | 39 => null;                                 -- mult pipe
                    when 40 =>
                        ph_acc  <= ph_base + unsigned(m_p(15 downto 0));
                        su_qmd  <= resize(t_q, 18) - resize(su_d1, 18);
                        su_qpdm <= resize(t_q, 18) + resize(su_d1, 18) - 2;
                    when 41 =>
                        m_a <= resize(le_d, 15);                          -- d0 * Q1 (per-ring phase)
                        m_b <= signed(resize(s_qp1, 15));
                    when 42 | 43 => null;                                 -- mult pipe
                    when 44 =>
                        if m0_done = '1' then
                            ph1_acc <= (others => '0');
                        else
                            ph1_acc <= unsigned(m_p(15 downto 0));
                        end if;
                    when others =>
                        e_st <= E_IDLE;
                end case;
                if l_ph < 45 then l_ph <= l_ph + 1; end if;

            end case;
        end if;
    end process p_engine;

    ------------------------------------------------------------------------
    -- tracker parity, delayed to meet the mark-parity path at the merge
    ------------------------------------------------------------------------
    p_par : process(clk)
    begin
        if rising_edge(clk) then
            k_ld1 <= t_kl;
            k_ld2 <= k_ld1;
            -- smooth fade: triangle fold of the phase top bits -> 8-bit luma
            if ph_acc(15) = '1' then ph_f1 <= ph_acc(14 downto 7);
            else                     ph_f1 <= not ph_acc(14 downto 7); end if;
            ph_f2 <= ph_f1;
            -- per-ring stepped ramp: triangle fold, DARK at the ring edges
            -- (phase 0), bright mid-ring.  (One delay reg only:
            -- ph1's update runs a cycle later than the smooth phase's.)
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
            r10_s <= ph_f2;      -- smooth fade ignores the nest/field split
            r10_t <= ph1_f1;     -- per-ring ramp (already edge-anchored)
        end if;
    end process p_r10;

    ------------------------------------------------------------------------
    -- GRADIENT (Stepped): EVERY RING carries its own ramp-in/ramp-out --
    -- the per-ring phase triangle (dark at both edges, bright mid-ring,
    -- re-anchored at every true disc edge so the shading cascades with the
    -- rings) -- and K4 picks the FINENESS: zone 0 = the classic hard look,
    -- zones 1..6 = 2/4/8/16/32/64 gray steps within each ring.
    -- S9 Smooth = the SAME per-ring ramp at full 8-bit resolution.
    -- S11 Smooth = the continuous multi-ring phase ramp (K4 = its period).
    ------------------------------------------------------------------------
    p_r12 : process(clk)
        variable v_t8  : unsigned(7 downto 0);
        variable v_key : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if s_ramp = '1' then
                -- S9 SMOOTH: per-ring ramp, full resolution (edge-anchored
                -- -> cascades with the rings)
                v_key := r10_t;
            else
                v_t8 := r10_t;
                case s_gn is
                    when 0 =>       -- classic: 1-bit fold of k(0)
                        v_key := (others => not r10_k(0));
                    when 1 =>       -- 2 steps per ring
                        v_key := (others => v_t8(7));
                    when 2 =>       -- 4 steps, x85
                        v_key := v_t8(7 downto 6) & v_t8(7 downto 6)
                                 & v_t8(7 downto 6) & v_t8(7 downto 6);
                    when 3 =>       -- 8 steps, ~x36.4
                        v_key := v_t8(7 downto 5) & v_t8(7 downto 5) & v_t8(7 downto 6);
                    when 4 =>       -- 16 steps, x17
                        v_key := v_t8(7 downto 4) & v_t8(7 downto 4);
                    when 5 =>       -- 32 steps, ~x8.2
                        v_key := v_t8(7 downto 3) & v_t8(7 downto 5);
                    when others =>  -- 64 steps within each ring, ~x4.05
                        v_key := v_t8(7 downto 2) & v_t8(7 downto 6);
                end case;
            end if;
            r12_key <= v_key;
            -- Saturation GAIN m = 16 + (key+4)/8 -> 16..48, applied as
            -- c*m/32, i.e. 0.5x at field black, 1.0x (UNMODIFIED) at
            -- mid-grey, 1.5x at white.  The user's "25 / 50 / 75 %" is the
            -- proc-amp reading where 50 % is normal -- NOT a fraction of the
            -- source, which would cap the picture at 3/4 saturation and make
            -- the whole image read as washed out.
            r12_h   <= resize(shift_right(resize(v_key, 9) + 4, 3), 6)
                       + to_unsigned(16, 6);
        end if;
    end process p_r12;

    ------------------------------------------------------------------------
    -- key -> true-black..true-white 10-bit luma (endpoints exact)
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_y : unsigned(9 downto 0);
        variable v_u, v_v : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- video stage 1: split the input, chroma referred to neutral
            r13_y  <= unsigned(data_in.y);
            r13_cu <= signed(resize(unsigned(data_in.u), 11))
                      - to_signed(512, 11);
            r13_cv <= signed(resize(unsigned(data_in.v), 11))
                      - to_signed(512, 11);

            -- stage 6: ONE multiply per component.  The gain m already
            -- carries the whole 0.5x..1.5x scale (m/32), so no c/2 term and
            -- no carried copy of c is needed.
            r14_pu <= r13_cu * signed('0' & r12_h);
            r14_pv <= r13_cv * signed('0' & r12_h);
            r14_y  <= r13_y;

            -- stage 7: c' = c*m/32, re-centred on neutral.
            -- BLANKING GATE: outside active video emit exactly what ring mode
            -- emits -- neutral chroma, black luma.  Without this the free-
            -- running pipeline puts PROCESSED GARBAGE in the blanking
            -- interval (input blanking chroma 0 scaled by 0.75 = 128, an
            -- illegal non-neutral level sitting where the encoder's per-line
            -- colour reference lives) and the WHOLE picture decodes with
            -- rail-to-rail wrong colours + noisy lines, while every active
            -- pixel is bit-correct.  Ring mode never hit this because its
            -- chroma is constant 512.  s_avid_sr(C_VLAT - 2) here aligns with
            -- the s_avid_sr(C_VLAT - 1) output tap after this register.
            if s_vsat = '1' and s_avid_sr(C_VLAT - 2) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            elsif s_vsat = '1' then
                s_out_y <= r14_y;
                v_u := to_signed(512, 13) + resize(shift_right(r14_pu, 5), 13);
                v_v := to_signed(512, 13) + resize(shift_right(r14_pv, 5), 13);
                -- BOOST can overflow (1.5 * full-scale chroma = 768), so this
                -- one DOES need clamping: v is in [-256, 1280].  Test the sign
                -- bit and the two bits above the field, never resize(SIGNED,10)
                -- -- that keeps the sign and drops bit 9, folding 896 -> 384
                -- and collapsing the chroma to a saturated GREEN screen.
                if v_u(12) = '1' then
                    s_out_u <= (others => '0');
                elsif v_u(11 downto 10) /= "00" then
                    s_out_u <= (others => '1');
                else
                    s_out_u <= unsigned(v_u(9 downto 0));
                end if;
                if v_v(12) = '1' then
                    s_out_v <= (others => '0');
                elsif v_v(11 downto 10) /= "00" then
                    s_out_v <= (others => '1');
                else
                    s_out_v <= unsigned(v_v(9 downto 0));
                end if;
            else
                v_y := shift_left(resize(r12_key, 10), 2)
                       or resize(r12_key(7 downto 6), 10);
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
    -- the two modes have different pipeline depths, so the syncs are tapped
    -- to match whichever one is driving the output
    data_out.avid    <= s_avid_sr(C_VLAT - 1)    when s_vsat = '1'
                        else s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_VLAT - 1) when s_vsat = '1'
                        else s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_VLAT - 1) when s_vsat = '1'
                        else s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_VLAT - 1) when s_vsat = '1'
                        else s_field_n_sr(C_LATENCY - 1);

end architecture cascade;
