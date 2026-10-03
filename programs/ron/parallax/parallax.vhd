-- parallax.vhd
--
-- PARALLAX -- anaglyph edge slabs over a sharp monochrome picture.
--
-- The incoming video is shown as clean luma-only monochrome at full
-- bandwidth (one point-operation contrast stage, no filtering whatsoever).
-- A SEPARATE, deliberately destroyed copy of the luma drives an edge
-- analyser, and on each scanline only the LEFTMOST and the RIGHTMOST
-- detected edge throw anything:
--
--   leftmost edge of the line  -> a flat glow thrown LEFT  (blue)
--   rightmost edge of the line -> a flat glow thrown RIGHT (red)
--
-- so the subject carries one glow off its left outline and one off its
-- right -- a stereo separation of the whole silhouette -- instead of every
-- interior edge emitting a band of its own.
--
-- Architecture
-- ------------
-- Display path : data_in.y -> contrast (K5 BASE) -> compose. Chroma is
--                discarded and replaced with neutral or slab colour. No
--                filter of any kind touches it, so the mono picture is
--                exactly as sharp as the source.
--
-- Analysis path: half horizontal resolution (pixel pairs averaged), then
--                a centred box blur (1..16 samples = 2..32 px, K2 CLEAN),
--                posterise (K2), a centred gradient gx = p(m+B/2)-p(m-B/2)
--                with the baseline B tracking the blur, a per-column
--                vertical IIR over scanlines (S7 SETTLE), then a
--                two-threshold hysteresis detector with a debounce. All
--                box/gradient taps are symmetric about a FIXED centre, so
--                turning CLEAN never shifts the edge map sideways.
--
-- Throwing a slab to the RIGHT is causal. Throwing one to the LEFT is not,
-- so the edge marks for a line are stored in a bit buffer and re-scanned
-- RIGHT-TO-LEFT during the following line: in reversed order the leftward
-- dilation (and the FILL "flood until the next edge" rule) becomes an
-- ordinary countdown. The resulting paint map is written into a second bit
-- buffer and consumed, forward, by the display pass one line later. Net
-- effect: the slab map lags the picture by two scanlines -- invisible, and
-- the display path is never delayed, so nothing shifts and nothing blurs.
--
-- Buffers (all canonical 1W1R, ping-ponged by line parity so no bank is
-- ever read and written in the same line -- iCE40 EBR returns UNDEFINED on
-- a same-address read-during-write):
--   vg0/vg1  1024 x 8  vertically-smoothed gradient, per analysis column
--   hb0/hb1  1024 x 4  previous line's edge state (vertical hysteresis)
--   (the per-edge version also needed a mark buffer and a paint-map buffer
--    for its reverse scan; only the silhouette extents are kept now)
--
-- Collisions: where a left-throw and a right-throw slab overlap the two
-- colours are ADDED as light (Y and chroma offsets summed, clamped) -- a
-- real anaglyph overlap goes magenta, and the sum generalises correctly to
-- every other pair. The result is still one flat colour, not a gradient.
--
-- Controls (the panel has 6 rotaries, 5 toggles and the slider; SETTLE and
-- DENSITY are the two controls that degrade cleanly to two states, so they
-- took the switches):
--   K1  EDGE     detector threshold
--   K2  CLEAN    analysis blur + posterise macro
--   K3  RANGE    maximum slab width, as a fraction of measured width
--   K4  BALANCE  bipolar blue/red width split, detented centre
--   K5  BASE     mono contrast; 50% is bit-exact passthrough
--   K6  PAIR     8 opposed colour pairs
--   S7  SETTLE   Lively / Calm  (vertical IIR + vertical hysteresis)
--   S8  DENSITY  Wash / Solid
--   S9  FILL     fixed width / flood to the next edge
--   S10 SWAP     exchange the two sides' colours
--   S11 LAYER    slabs Over / Behind the picture
--   P12 WIDTH    master slab width, glided
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture parallax of program_top is

    constant C_SYNC_DELAY : integer := 7;

    -- A mark at analysis column c is computed from source samples c-16 to
    -- c+16 (the box window nested inside the gradient taps). At the start
    -- of a line those registers still hold the PREVIOUS line's tail, so the
    -- step between the two lines reads as an edge at column ~0 -- which
    -- then wins the leftmost-edge contest and pins the left glow to the
    -- frame edge. Nothing below this margin may produce a mark.
    constant C_MARGIN : integer := 18;

    -- The analysis chain is centred 16 samples behind the sample entering
    -- stage 1 (8 for the box centre, 8 for the gradient centre) and then
    -- runs 12 pipeline stages deep, so each buffer column trails the sample
    -- index by a fixed amount: 23 for the gradient-history read, 26 for its
    -- write and the edge-state read, 28 for the mark write. Those offsets
    -- are carried by the s_c* counters rather than recomputed per tick.

    ----------------------------------------------------------------------------
    -- Colour pairs, 10-bit YUV, BT.601 limited range, U/V SWAPPED for the
    -- Videomancer hardware (C_*U holds Cr, C_*V holds Cb -- the prism/SDK
    -- convention). Luma is capped at 768: full-scale Y clips all of RGB on
    -- hardware and kills the chroma with it. Separate 1-D constants rather
    -- than an array-of-records: GHDL synthesis crashes on parallel reads of
    -- an array-of-arrays constant.
    ----------------------------------------------------------------------------
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    --                     Blue  Cyan  Green Yellw Teal  Azure Lime  White
    constant C_LY : t_pal := (
        to_unsigned(164,10), to_unsigned(678,10), to_unsigned(578,10),
        to_unsigned(768,10), to_unsigned(522,10), to_unsigned(422,10),
        to_unsigned(743,10), to_unsigned(768,10));
    constant C_LU : t_pal := (
        to_unsigned(439,10), to_unsigned( 64,10), to_unsigned(137,10),
        to_unsigned(585,10), to_unsigned(178,10), to_unsigned(251,10),
        to_unsigned(418,10), to_unsigned(512,10));
    constant C_LV : t_pal := (
        to_unsigned(960,10), to_unsigned(663,10), to_unsigned(215,10),
        to_unsigned( 64,10), to_unsigned(625,10), to_unsigned(811,10),
        to_unsigned(120,10), to_unsigned(512,10));
    --                     Red   Orang Magen Viole Amber Rose  Purpl Black
    constant C_RY : t_pal := (
        to_unsigned(326,10), to_unsigned(584,10), to_unsigned(426,10),
        to_unsigned(318,10), to_unsigned(681,10), to_unsigned(505,10),
        to_unsigned(295,10), to_unsigned( 64,10));
    constant C_RU : t_pal := (
        to_unsigned(960,10), to_unsigned(772,10), to_unsigned(887,10),
        to_unsigned(703,10), to_unsigned(701,10), to_unsigned(829,10),
        to_unsigned(664,10), to_unsigned(512,10));
    constant C_RV : t_pal := (
        to_unsigned(361,10), to_unsigned(212,10), to_unsigned(809,10),
        to_unsigned(871,10), to_unsigned(156,10), to_unsigned(511,10),
        to_unsigned(884,10), to_unsigned(512,10));

    ----------------------------------------------------------------------------
    -- Knobs
    ----------------------------------------------------------------------------
    signal s_k_edge, s_k_clean, s_k_range : unsigned(9 downto 0);
    signal s_k_bal, s_k_base              : unsigned(9 downto 0);
    signal s_k_pair                       : unsigned(2 downto 0);
    signal s_k_width                      : unsigned(9 downto 0);
    signal s_sw_settle, s_sw_dens, s_sw_fill, s_sw_swap, s_sw_layer : std_logic;

    -- Field-latched analysis parameters (a field must never be processed
    -- with two different settings -- the seam reads as a fake edge).
    signal s_thr_hi   : signed(8 downto 0) := to_signed(50, 9);
    signal s_thr_lo   : signed(8 downto 0) := to_signed(25, 9);
    signal s_thr_hiv  : signed(8 downto 0) := to_signed(25, 9);
    signal s_bmask    : std_logic_vector(15 downto 0) := x"0180";
    signal s_bshift   : unsigned(2 downto 0) := "001";
    signal s_pmask    : std_logic_vector(7 downto 0) := x"FF";
    signal s_gsel     : unsigned(1 downto 0) := "00";
    signal s_dbnc     : unsigned(1 downto 0) := "00";
    signal s_calm     : std_logic := '0';
    signal s_gain     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_dens_sol : std_logic := '1';
    signal s_layer_bh : std_logic := '0';

    -- Field-latched slab widths (analysis samples) and colours
    signal s_wb, s_wr : unsigned(10 downto 0) := (others => '0');
    signal s_cly, s_clu, s_clv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_cry, s_cru, s_crv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------------
    -- Per-field width sequencer: one shared multiplier, 12 fixed slots.
    --   w_max = width_half * rangefrac / 1024
    --   w_cur = w_max     * p12_glide / 1024
    --   wb    = w_cur     * bal_b     / 256
    --   wr    = w_cur     * bal_r     / 256
    -- Products are valid two slots after their operands are set; the
    -- results only reach s_wb/s_wr at a field boundary.
    ----------------------------------------------------------------------------
    signal s_slot   : unsigned(3 downto 0) := (others => '0');
    signal s_ma     : unsigned(11 downto 0) := (others => '0');
    signal s_mb     : unsigned(11 downto 0) := (others => '0');
    signal s_prod   : unsigned(23 downto 0) := (others => '0');
    signal s_wmax   : unsigned(11 downto 0) := (others => '0');
    signal s_wcur   : unsigned(11 downto 0) := (others => '0');
    signal s_wb_s   : unsigned(10 downto 0) := (others => '0');
    signal s_wr_s   : unsigned(10 downto 0) := (others => '0');
    signal s_wb_c   : unsigned(10 downto 0) := (others => '0');
    signal s_wr_c   : unsigned(10 downto 0) := (others => '0');
    signal s_rfrac  : unsigned(11 downto 0) := to_unsigned(64, 12);
    signal s_balb   : unsigned(9 downto 0)  := to_unsigned(256, 10);
    signal s_balr   : unsigned(9 downto 0)  := to_unsigned(256, 10);
    signal s_p12g   : unsigned(9 downto 0)  := (others => '0');

    -- Knob-derived precomputes. Everything here is quasi-static, but a
    -- vblank cone is still fully STA-timed at the pixel clock, so each long
    -- chain gets broken by a register rather than being written as one
    -- expression inside the field latch.
    signal s_rf_a   : unsigned(11 downto 0) := (others => '0');
    signal s_bal_d  : signed(10 downto 0)   := (others => '0');
    signal s_gdiff  : signed(11 downto 0)   := (others => '0');
    signal s_ei     : unsigned(9 downto 0)  := (others => '0');
    signal s_esq    : unsigned(19 downto 0) := (others => '0');
    signal s_thr_t  : unsigned(9 downto 0)  := to_unsigned(18, 10);
    signal s_thr_h2 : unsigned(9 downto 0)  := to_unsigned(27, 10);
    signal s_thr_q2 : unsigned(9 downto 0)  := to_unsigned(39, 10);
    signal s_gain_p : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_sly, s_slu, s_slv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_sry, s_sru, s_srv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_sby, s_sbu, s_sbv : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------------
    -- Raster bookkeeping (input domain)
    ----------------------------------------------------------------------------
    signal s_prev_hs  : std_logic := '1';
    signal s_hs_edge  : std_logic := '0';
    signal s_cx       : unsigned(11 downto 0) := (others => '0');
    signal s_seen     : std_logic := '0';
    signal s_aline    : unsigned(10 downto 0) := (others => '0');
    signal s_width    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_mhalf    : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_bank     : std_logic := '0';
    signal s_fpulse   : std_logic := '0';   -- once per field
    signal s_vbrun    : std_logic := '0';
    signal s_lend     : std_logic := '0';   -- line turn-around pulse
    signal s_lblank   : std_logic := '1';   -- the line that just ended was blank
    signal s_warm     : std_logic := '1';   -- too few lines yet: no slabs
    signal s_load     : std_logic := '1';   -- field warm-up: IIR loads, not blends

    ----------------------------------------------------------------------------
    -- Half-rate analysis tick. Runs through active video and 32 samples into
    -- horizontal blanking so the pipeline drains and every column of the
    -- line's buffers is written exactly once.
    ----------------------------------------------------------------------------
    signal s_run   : std_logic := '0';
    signal s_post  : unsigned(6 downto 0) := (others => '0');
    signal s_ph    : std_logic := '0';
    signal s_ae    : std_logic := '0';
    signal s_am    : unsigned(11 downto 0) := (others => '0');

    signal s_pair_a : unsigned(7 downto 0) := (others => '0');
    signal s_ain    : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Analysis pipeline
    ----------------------------------------------------------------------------
    type t_bsr is array (0 to 15) of unsigned(7 downto 0);
    type t_gsr is array (0 to 16) of unsigned(11 downto 0);
    signal s_bsr : t_bsr := (others => (others => '0'));
    signal s_gsr : t_gsr := (others => (others => '0'));

    type t_p9  is array (0 to 7) of unsigned(8 downto 0);
    type t_q10 is array (0 to 3) of unsigned(9 downto 0);
    type t_o11 is array (0 to 1) of unsigned(10 downto 0);
    signal s_p : t_p9  := (others => (others => '0'));
    signal s_q : t_q10 := (others => (others => '0'));
    signal s_o : t_o11 := (others => (others => '0'));
    signal s_bsum : unsigned(11 downto 0) := (others => '0');
    signal s_pp   : unsigned(7 downto 0) := (others => '0');

    -- Two analysis line buffers give the detector a real, SYMMETRIC [1 2 1]
    -- vertical support before the horizontal difference -- a Sobel proper,
    -- as livewire does it. Without it the only vertical support was the
    -- lagging IIR, which reaches just 41% of a gradient on a contour four
    -- lines tall: exactly the curved contours of a face, attenuated hardest.
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb1, lb2   : t_lb := (others => (others => '0'));
    signal lq1, lq2   : std_logic_vector(7 downto 0) := (others => '0');
    signal s_lb_ra    : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wa    : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wv    : std_logic := '0';
    signal s_clb      : signed(12 downto 0) := to_signed(-13, 13);
    signal qr1, qr2   : unsigned(7 downto 0) := (others => '0');
    signal s_ppd      : unsigned(7 downto 0) := (others => '0');
    signal s_cs       : unsigned(11 downto 0) := (others => '0');

    signal s_gl, s_gr : unsigned(11 downto 0) := (others => '0');
    signal s_gh       : signed(7 downto 0) := (others => '0');
    -- The gradient history comes out of an EBR, whose clock-to-q plus the
    -- trip out of the RAM column is ~3.6 ns before any logic runs. It lands
    -- in a fabric FF (s_vqr) before ANY arithmetic, and the IIR itself is
    -- split into a subtract stage and an add/clamp stage -- unsplit and
    -- unregistered, that one path cost ~6 MHz and missed HD on every seed.
    signal s_vqr      : signed(7 downto 0) := (others => '0');
    signal s_vqd      : signed(7 downto 0) := (others => '0');
    signal s_ghd      : signed(7 downto 0) := (others => '0');
    -- The rise/decay decision and the decayed step are both resolved a
    -- stage EARLY: computing the magnitude compare and then muxing three
    -- step candidates into the add left s_avq driving the whole s_gs cone,
    -- and that was the routed critical path (71.4 MHz on HD Dual).
    signal s_rise     : std_logic := '0';
    signal s_stp      : signed(9 downto 0) := (others => '0');
    signal s_gs       : signed(7 downto 0) := (others => '0');

    -- Buffer columns as free-running per-tick counters rather than
    -- subtracting a constant from the sample index and range-checking the
    -- difference: that was a 13-bit subtract feeding a 13-bit compare in
    -- the same stage as the hysteresis state machine.
    signal s_cvgr : signed(12 downto 0) := to_signed(-24, 13);
    signal s_chbr : signed(12 downto 0) := to_signed(-27, 13);
    signal s_cvgw : signed(12 downto 0) := to_signed(-27, 13);
    signal s_cmk  : signed(12 downto 0) := to_signed(-29, 13);

    signal s_le_ab, s_le_hd, s_re_ab, s_re_hd : std_logic := '0';
    signal s_le_cnt, s_re_cnt : unsigned(1 downto 0) := (others => '0');
    signal s_le_st, s_re_st   : std_logic := '0';

    -- Mark-centring delay. A gradient run straddles the edge it belongs to,
    -- so its leading corner sits half a run-width OUTSIDE the object and a
    -- slab hung off it leaves a dark halo. Both marks are taken at the run's
    -- leading corner and then delayed by s_mdel+1 analysis samples -- which
    -- is written at the pipeline's own column, so the delay lands the mark
    -- on the middle of the run, i.e. the edge itself. The correction tracks
    -- CLEAN, since the run widens with the blur and the gradient baseline.
    type t_mdl is array (0 to 7) of std_logic_vector(1 downto 0);
    signal s_mdl  : t_mdl := (others => "00");
    signal s_mdel : unsigned(2 downto 0) := "001";

    -- Buffer addresses (analysis domain). Negative columns are masked off
    -- by the matching valid flags.
    signal s_vg_ra, s_vg_wa : unsigned(9 downto 0) := (others => '0');
    signal s_vg_wv          : std_logic := '0';
    signal s_hb_ra, s_hb_wa : unsigned(9 downto 0) := (others => '0');
    signal s_hb_wv          : std_logic := '0';
    signal s_hb_wd          : std_logic_vector(3 downto 0) := (others => '0');
    signal s_vg_wd          : std_logic_vector(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Silhouette extents. Only the LEFTMOST and the RIGHTMOST edge of each
    -- scanline throw anything: the subject gets one glow off its left
    -- outline and one off its right, instead of every interior edge
    -- emitting its own band. Both are found in the single forward analysis
    -- pass -- first mark wins the left, every mark overwrites the right --
    -- so the reverse scan, the mark buffer and the paint-map buffer that
    -- the per-edge version needed are all gone, and with them 4 EBR, a
    -- whole pipeline pass and a scanline of lag.
    ----------------------------------------------------------------------------
    signal s_lmark, s_rmark : unsigned(9 downto 0) := (others => '0');
    signal s_mk_any         : std_logic := '0';
    -- latched for the NEXT line, with the four painted bounds resolved at
    -- the line turn-around so the pixel path is only compares
    signal s_lr_v   : std_logic := '0';
    signal s_bl_lo, s_bl_hi : unsigned(10 downto 0) := (others => '0');
    signal s_rd_lo, s_rd_hi : unsigned(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- RAM banks
    ----------------------------------------------------------------------------
    type t_ram8 is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_ram4 is array (0 to 1023) of std_logic_vector(3 downto 0);
    signal vg0, vg1 : t_ram8 := (others => (others => '0'));
    signal hb0, hb1 : t_ram4 := (others => (others => '0'));
    signal vq0, vq1 : std_logic_vector(7 downto 0) := (others => '0');
    signal hq0, hq1 : std_logic_vector(3 downto 0) := (others => '0');
    signal s_vq, s_hq : std_logic_vector(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Display pipeline
    ----------------------------------------------------------------------------
    signal av    : std_logic_vector(1 to 7) := (others => '0');
    signal s_m1  : unsigned(10 downto 0) := (others => '0');
    signal s_cmp : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ph1, s_ph2, s_ph3 : std_logic := '0';
    signal s_y1  : signed(11 downto 0) := (others => '0');
    signal s_y2  : signed(24 downto 0) := (others => '0');
    signal s_y3, s_y4, s_y5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_pbl, s_pbr : std_logic := '0';
    signal s_cy, s_cu, s_cv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_has : std_logic := '0';
    signal s_oy, s_ou, s_ov : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ry, s_ru, s_rv_o : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s_avid_sr    : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');
    signal s_field_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');

    function f_clamp10(v : signed; lo : integer; hi : integer) return unsigned is
    begin
        if v < to_signed(lo, v'length) then
            return to_unsigned(lo, 10);
        elsif v > to_signed(hi, v'length) then
            return to_unsigned(hi, 10);
        else
            return unsigned(std_logic_vector(v(9 downto 0)));
        end if;
    end function;

begin

    ----------------------------------------------------------------------------
    -- Control wiring. Rotaries 1..6 = registers_in(0..5); toggles 7..11 are
    -- BITS 0..4 of registers_in(6); the P12 slider is registers_in(7) and
    -- carries the headline gesture.
    ----------------------------------------------------------------------------
    s_k_edge   <= unsigned(registers_in(0));
    s_k_clean  <= unsigned(registers_in(1));
    s_k_range  <= unsigned(registers_in(2));
    s_k_bal    <= unsigned(registers_in(3));
    s_k_base   <= unsigned(registers_in(4));
    s_k_pair   <= unsigned(registers_in(5)(9 downto 7));
    s_k_width  <= unsigned(registers_in(7));
    s_sw_settle <= registers_in(6)(0);
    s_sw_dens   <= registers_in(6)(1);
    s_sw_fill   <= registers_in(6)(2);
    s_sw_swap   <= registers_in(6)(3);
    s_sw_layer  <= registers_in(6)(4);

    ----------------------------------------------------------------------------
    -- Raster bookkeeping. The active-line counter resets on a line period
    -- with no active video -- not on the vsync edge, whose position relative
    -- to the picture varies by decoder, and which serrates into several
    -- edges per field on analog sources.
    ----------------------------------------------------------------------------
    p_raster : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hs <= data_in.hsync_n;
            s_hs_edge <= '0';
            s_fpulse  <= '0';

            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            if data_in.hsync_n = '0' and s_prev_hs = '1' then
                s_hs_edge <= '1';
                -- s_hs_edge is REGISTERED, so by the time anything sees it
                -- s_seen has already been cleared. Latch the verdict here.
                s_lblank  <= not s_seen;
                if s_seen = '1' then
                    s_seen  <= '0';
                    s_vbrun <= '0';
                else
                    -- a line period with no active video is vertical
                    -- blanking. Fire the parameter latch ONCE per field:
                    -- analog vsync serrates, and a pulse per blank line
                    -- would run the P12 glide dozens of times a frame.
                    if s_vbrun = '0' then
                        s_fpulse <= '1';
                    end if;
                    s_vbrun <= '1';
                end if;
            end if;
        end if;
    end process p_raster;

    ----------------------------------------------------------------------------
    -- Half-rate analysis tick, pixel-pair averaging and the per-line turn-
    -- around. The tick keeps running 56 clocks into horizontal blanking so
    -- the analysis pipeline drains (the deepest buffer column trails the
    -- sample index by 28 ticks): the rightmost columns of a line must
    -- have their marks written before next line's reverse pass reads them.
    -- The line ends -- and the buffer banks swap -- when that drain
    -- finishes, never on the hsync edge, which can land inside it.
    ----------------------------------------------------------------------------
    p_tick : process(clk)
    begin
        if rising_edge(clk) then
            s_lend <= '0';

            if data_in.avid = '1' then
                s_run  <= '1';
                s_post <= to_unsigned(72, 7);
                s_cx   <= s_cx + 1;
            elsif s_run = '1' then
                if s_post > 0 then
                    s_post <= s_post - 1;
                else
                    s_lend <= '1';
                end if;
            end if;

            if s_run = '1' and data_in.avid = '0'
               and (s_post = 0 or s_hs_edge = '1') then
                -- line turn-around
                s_run   <= '0';
                s_lend  <= '1';
                s_ph    <= '0';
                s_am    <= (others => '0');
                s_cx    <= (others => '0');
                s_bank  <= not s_bank;
                s_width <= s_cx;
                if s_cx > 32 then
                    s_mhalf <= s_cx(11 downto 1);
                end if;
                if s_aline < 2000 then
                    s_aline <= s_aline + 1;
                end if;
                if s_aline >= 3 then
                    s_warm <= '0';
                end if;
                if s_aline >= 4 then
                    s_load <= '0';
                end if;

                -- Latch this line's silhouette for the next line and
                -- resolve the four painted bounds here, so the pixel path
                -- is nothing but compares against registered values.
                s_lr_v <= s_mk_any and not s_warm;
                if s_lmark > s_wb then
                    s_bl_lo <= resize(s_lmark - s_wb, 11);
                else
                    s_bl_lo <= (others => '0');
                end if;
                if s_lmark > 0 then
                    s_bl_hi <= resize(s_lmark - 1, 11);
                else
                    s_bl_hi <= (others => '0');
                end if;
                s_rd_lo <= resize(s_rmark, 11) + 1;
                s_rd_hi <= resize(s_rmark, 11) + resize(s_wr, 11);
            elsif s_run = '1' then
                s_ph <= not s_ph;
                if s_ae = '1' then
                    s_am <= s_am + 1;
                end if;
            end if;

            if s_hs_edge = '1' and s_lblank = '1' then
                s_warm  <= '1';
                s_load  <= '1';
                s_aline <= (others => '0');
            end if;

            -- pixel pair -> one analysis sample
            if data_in.avid = '1' then
                if s_cx(0) = '0' then
                    s_pair_a <= unsigned(data_in.y(9 downto 2));
                else
                    s_ain <= resize(shift_right(
                                resize(s_pair_a, 9)
                                + resize(unsigned(data_in.y(9 downto 2)), 9), 1), 8);
                end if;
            end if;
        end if;
    end process p_tick;

    s_ae <= s_run and s_ph;

    ----------------------------------------------------------------------------
    -- Analysis pipeline. One operation per stage; every box/gradient window
    -- is symmetric about a fixed tap, so changing CLEAN never slides the
    -- edge map sideways.
    --
    --  A1  shift the sample into the box register
    --  A2  mask the taps (per-field window) and sum in pairs
    --  A3  sum pairs -> quads
    --  A4  quads -> octs
    --  A5  octs -> box total
    --  A6  normalise (>> log2 L), posterise
    --  A7  shift into the gradient register; present the gradient-history read
    --  A8  centred difference -> gx, halved to fit the 8-bit history word;
    --      the gradient-history read lands in a fabric FF
    --  A9  IIR part one: difference against the previous line
    --  A10 IIR part two: weight and accumulate; write the history back
    --  A11 threshold comparisons (vertical hysteresis picks the threshold)
    --  A12 run hysteresis + debounce -> edge marks; write the mark buffer
    ----------------------------------------------------------------------------
    p_analysis : process(clk)
        variable v_t    : t_bsr;
        variable v_gl, v_gr : unsigned(11 downto 0);
        variable v_d    : signed(9 downto 0);
        variable v_e    : signed(12 downto 0);
        variable v_f    : signed(12 downto 0);
        variable v_step : signed(9 downto 0);
        variable v_ns   : signed(9 downto 0);
        variable v_ler, v_rer : std_logic;
        variable v_thr  : signed(8 downto 0);
        variable v_ah, v_aq : unsigned(7 downto 0);
        variable v_g10  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_lend = '1' then
                s_mdl    <= (others => "00");
                s_le_st  <= '0';
                s_re_st  <= '0';
                s_le_cnt <= (others => '0');
                s_re_cnt <= (others => '0');
                s_hb_wv  <= '0';
                s_mk_any <= '0';
                s_vg_wv  <= '0';
                s_clb    <= to_signed(-13, 13);
                s_cvgr   <= to_signed(-24, 13);
                s_chbr   <= to_signed(-27, 13);
                s_cvgw   <= to_signed(-27, 13);
                s_cmk    <= to_signed(-29, 13);
                s_lb_wv  <= '0';
            elsif s_ae = '1' then

                -- A12: hysteresis state -> marks, write the mark buffer -----
                v_ler := s_le_st;
                if s_le_ab = '1' and s_le_cnt >= s_dbnc then
                    v_ler := '1';
                elsif s_le_hd = '0' then
                    v_ler := '0';
                end if;
                v_rer := s_re_st;
                if s_re_ab = '1' and s_re_cnt >= s_dbnc then
                    v_rer := '1';
                elsif s_re_hd = '0' then
                    v_rer := '0';
                end if;
                if s_le_ab = '1' then
                    if s_le_cnt < 3 then
                        s_le_cnt <= s_le_cnt + 1;
                    end if;
                elsif s_le_hd = '0' then
                    s_le_cnt <= (others => '0');
                end if;
                if s_re_ab = '1' then
                    if s_re_cnt < 3 then
                        s_re_cnt <= s_re_cnt + 1;
                    end if;
                elsif s_re_hd = '0' then
                    s_re_cnt <= (others => '0');
                end if;
                s_le_st <= v_ler;
                s_re_st <= v_rer;

                -- Both marks are taken at the leading corner of their run
                -- and pushed through the centring delay, so each lands on
                -- the middle of the run -- the edge itself. Blue then ends
                -- one column short of the edge and red starts on it.
                -- Either polarity counts towards the silhouette: the
                -- leftmost edge of a line is the subject's left boundary
                -- whether the subject is lighter or darker than its ground.
                if s_mdl(to_integer(s_mdel)) /= "00"
                   and s_cmk >= C_MARGIN
                   and s_cmk < signed(resize(s_mhalf, 13)) - 2 then
                    if s_mk_any = '0' then
                        s_lmark <= unsigned(std_logic_vector(s_cmk(9 downto 0)));
                    end if;
                    s_rmark <= unsigned(std_logic_vector(s_cmk(9 downto 0)));
                    s_mk_any <= '1';
                end if;

                for i in 7 downto 1 loop
                    s_mdl(i) <= s_mdl(i - 1);
                end loop;
                s_mdl(0) <= (v_rer and not s_re_st) & (v_ler and not s_le_st);
                s_hb_wd <= "00" & v_rer & v_ler;

                s_hb_wa <= unsigned(std_logic_vector(s_cmk(9 downto 0)));
                if s_cmk >= 0 and s_cmk < signed(resize(s_mhalf, 13)) then
                    s_hb_wv <= '1';
                else
                    s_hb_wv <= '0';
                end if;

                -- A11: comparisons. A column that was an edge on the previous
                -- line gets the lowered threshold -- that is what stops slab
                -- boundaries crawling line to line.
                if s_hq(0) = '1' then
                    v_thr := s_thr_hiv;
                else
                    v_thr := s_thr_hi;
                end if;
                v_g10 := resize(s_gs, 10);
                if v_g10 > resize(v_thr, 10) then s_le_ab <= '1'; else s_le_ab <= '0'; end if;
                if v_g10 > resize(s_thr_lo, 10) then s_le_hd <= '1'; else s_le_hd <= '0'; end if;
                if s_hq(1) = '1' then
                    v_thr := s_thr_hiv;
                else
                    v_thr := s_thr_hi;
                end if;
                if v_g10 < -resize(v_thr, 10) then s_re_ab <= '1'; else s_re_ab <= '0'; end if;
                if v_g10 < -resize(s_thr_lo, 10) then s_re_hd <= '1'; else s_re_hd <= '0'; end if;

                -- A12: IIR second half. It RISES instantly and only DECAYS
                -- slowly: a symmetric IIR costs sensitivity exactly where
                -- it is needed (a contour four lines tall reached 41% of
                -- its gradient), whereas rise-fast/decay-slow gives the
                -- vertical persistence that smooths a slab boundary while
                -- a new edge still fires at full strength on its first
                -- line. SETTLE sets the decay rate. The first lines of a
                -- field LOAD rather than blend, so both fields start from
                -- identical state and an interlaced still cannot twitter.
                -- On a rise the answer IS this line's gradient; on a decay
                -- it is the previous value plus the pre-shifted step. Both
                -- results provably stay inside signed 8 bits (the decay
                -- lands between the two endpoints), so there is no clamp.
                if s_rise = '1' or s_load = '1' then
                    v_ns := resize(s_ghd, 10);
                else
                    v_ns := resize(s_vqd, 10) + s_stp;
                end if;
                s_gs <= resize(v_ns, 8);
                s_vg_wa <= unsigned(std_logic_vector(s_cvgw(9 downto 0)));
                if s_cvgw >= 0 and s_cvgw < signed(resize(s_mhalf, 13)) then
                    s_vg_wv <= '1';
                else
                    s_vg_wv <= '0';
                end if;
                s_cvgw <= s_cvgw + 1;

                -- A11: IIR first half -- the difference against the previous
                -- line, plus the two magnitudes the rise/decay test needs
                v_d := resize(s_gh, 10) - resize(s_vqr, 10);
                if s_calm = '1' then
                    s_stp <= shift_right(v_d, 3);
                else
                    s_stp <= shift_right(v_d, 1);
                end if;
                s_vqd <= s_vqr;
                s_ghd <= s_gh;
                if s_gh < 0 then
                    v_ah := unsigned(-s_gh);
                else
                    v_ah := unsigned(s_gh);
                end if;
                if s_vqr < 0 then
                    v_aq := unsigned(-s_vqr);
                else
                    v_aq := unsigned(s_vqr);
                end if;
                if v_ah > v_aq then
                    s_rise <= '1';
                else
                    s_rise <= '0';
                end if;

                -- A10: centred gradient over the column sums. The sums
                -- carry a factor of four ([1 2 1]), so the >>3 leaves gh in
                -- the same half-luma units the thresholds are written in.
                v_e  := signed(resize(s_gl, 13)) - signed(resize(s_gr, 13));
                s_gh <= resize(shift_right(v_e, 3), 8);
                s_vqr <= signed(s_vq);

                -- A9: gradient register + buffer read addresses
                case s_gsel is
                    when "00"   => v_gl := s_gsr(7);  v_gr := s_gsr(9);
                    when "01"   => v_gl := s_gsr(6);  v_gr := s_gsr(10);
                    when "10"   => v_gl := s_gsr(4);  v_gr := s_gsr(12);
                    when others => v_gl := s_gsr(0);  v_gr := s_gsr(16);
                end case;
                s_gl <= v_gl;
                s_gr <= v_gr;
                for i in 16 downto 1 loop
                    s_gsr(i) <= s_gsr(i - 1);
                end loop;
                s_gsr(0) <= s_cs;

                s_vg_ra <= unsigned(std_logic_vector(s_cvgr(9 downto 0)));
                s_hb_ra <= unsigned(std_logic_vector(s_chbr(9 downto 0)));
                s_cvgr  <= s_cvgr + 1;
                s_chbr  <= s_chbr + 1;
                s_cmk   <= s_cmk + 1;

                -- A8: symmetric [1 2 1] vertical column sum -- the SOBEL
                -- support the detector was missing
                s_cs <= resize(qr2, 12) + shift_left(resize(qr1, 12), 1)
                        + resize(s_ppd, 12);

                -- A7: land the line-buffer reads in fabric FFs (an EBR read
                -- must never feed arithmetic directly) and rotate the rows
                s_ppd <= s_pp;
                qr1   <= unsigned(lq1);
                qr2   <= unsigned(lq2);
                s_lb_wa <= s_lb_ra;
                if s_lb_ra < s_mhalf then
                    s_lb_wv <= '1';
                else
                    s_lb_wv <= '0';
                end if;

                -- A6: normalise + posterise; present the line-buffer read
                s_pp <= resize(shift_right(s_bsum, to_integer(s_bshift)), 8)
                        and unsigned(s_pmask);
                s_lb_ra <= unsigned(std_logic_vector(s_clb(9 downto 0)));
                s_clb   <= s_clb + 1;

                -- A5..A2: the box-sum adder tree, one level per stage
                s_bsum <= resize(s_o(0), 12) + resize(s_o(1), 12);
                for i in 0 to 1 loop
                    s_o(i) <= resize(s_q(2 * i), 11) + resize(s_q(2 * i + 1), 11);
                end loop;
                for i in 0 to 3 loop
                    s_q(i) <= resize(s_p(2 * i), 10) + resize(s_p(2 * i + 1), 10);
                end loop;
                for i in 0 to 15 loop
                    if s_bmask(i) = '1' then
                        v_t(i) := s_bsr(i);
                    else
                        v_t(i) := (others => '0');
                    end if;
                end loop;
                for i in 0 to 7 loop
                    s_p(i) <= resize(v_t(2 * i), 9) + resize(v_t(2 * i + 1), 9);
                end loop;

                -- A1: box register
                for i in 15 downto 1 loop
                    s_bsr(i) <= s_bsr(i - 1);
                end loop;
                s_bsr(0) <= s_ain;
            end if;
        end if;
    end process p_analysis;

    s_vg_wd <= std_logic_vector(s_gs);


    ----------------------------------------------------------------------------
    -- Buffer banks. Every one is the canonical single-write / unconditional-
    -- read shape, and the bank being written is never the bank being read,
    -- so an iCE40 EBR same-address read-during-write can never reach the
    -- picture.
    ----------------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            lq1 <= lb1(to_integer(s_lb_ra));
            if s_lb_wv = '1' then
                lb1(to_integer(s_lb_wa)) <= std_logic_vector(s_ppd);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            lq2 <= lb2(to_integer(s_lb_ra));
            if s_lb_wv = '1' then
                lb2(to_integer(s_lb_wa)) <= lq1;
            end if;
        end if;
    end process p_lb2;

    p_vg0 : process(clk)
    begin
        if rising_edge(clk) then
            vq0 <= vg0(to_integer(s_vg_ra));
            if s_vg_wv = '1' and s_bank = '0' then
                vg0(to_integer(s_vg_wa)) <= s_vg_wd;
            end if;
        end if;
    end process p_vg0;

    p_vg1 : process(clk)
    begin
        if rising_edge(clk) then
            vq1 <= vg1(to_integer(s_vg_ra));
            if s_vg_wv = '1' and s_bank = '1' then
                vg1(to_integer(s_vg_wa)) <= s_vg_wd;
            end if;
        end if;
    end process p_vg1;

    p_hb0 : process(clk)
    begin
        if rising_edge(clk) then
            hq0 <= hb0(to_integer(s_hb_ra));
            if s_hb_wv = '1' and s_bank = '0' then
                hb0(to_integer(s_hb_wa)) <= s_hb_wd;
            end if;
        end if;
    end process p_hb0;

    p_hb1 : process(clk)
    begin
        if rising_edge(clk) then
            hq1 <= hb1(to_integer(s_hb_ra));
            if s_hb_wv = '1' and s_bank = '1' then
                hb1(to_integer(s_hb_wa)) <= s_hb_wd;
            end if;
        end if;
    end process p_hb1;

    -- Read the bank the PREVIOUS line wrote. Both banks are read every
    -- clock and the registered outputs are muxed -- a conditional read maps
    -- to logic instead of inferring a RAM.
    s_vq <= vq1 when s_bank = '0' else vq0;
    s_hq <= "0000" & hq1 when s_bank = '0' else "0000" & hq0;

    ----------------------------------------------------------------------------
    -- Display pipeline.
    --  D1 centre the luma on mid-grey; issue the paint-map read
    --  D2 BASE contrast multiply (alone in its stage); paint word lands
    --  D3 rescale + clamp the mono luma; advance the rightward countdown
    --  D4 pick the flat slab colour
    --  D5 LAYER key + DENSITY compose
    --  D6 blanking gate + output register
    ----------------------------------------------------------------------------
    p_display : process(clk)
        variable v_sum : signed(26 downto 0);
        variable v_bl, v_rd : std_logic;
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_wy : unsigned(13 downto 0);
        variable v_wu : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            av(1) <= data_in.avid;
            for i in 2 to 7 loop
                av(i) <= av(i - 1);
            end loop;

            -- D1: centre the luma, take this pixel's analysis column
            s_y1  <= signed(resize(unsigned(data_in.y), 12)) - to_signed(512, 12);
            s_m1  <= resize(s_cx(10 downto 1), 11);
            s_ph1 <= s_cx(0);

            -- D2: BASE contrast multiply, alone in its stage. In parallel,
            -- the four bound compares -- the whole slab geometry is now
            -- "is this column inside the band off the left outline, or the
            -- band off the right one".
            s_y2  <= s_y1 * signed('0' & std_logic_vector(s_gain));
            s_ph2 <= s_ph1;
            if s_m1 >= s_bl_lo then s_cmp(0) <= '1'; else s_cmp(0) <= '0'; end if;
            if s_m1 <= s_bl_hi then s_cmp(1) <= '1'; else s_cmp(1) <= '0'; end if;
            if s_m1 >= s_rd_lo then s_cmp(2) <= '1'; else s_cmp(2) <= '0'; end if;
            if s_m1 <= s_rd_hi then s_cmp(3) <= '1'; else s_cmp(3) <= '0'; end if;

            -- D3: rescale + clamp
            v_sum := resize(shift_right(s_y2, 8), 27) + to_signed(512, 27);
            s_y3  <= f_clamp10(v_sum, 0, 1023);
            s_ph3 <= s_ph2;

            -- D4: combine the bounds into the two glow flags
            s_y4  <= s_y3;
            s_pbl <= s_lr_v and s_cmp(0) and s_cmp(1);
            s_pbr <= s_lr_v and s_cmp(2) and s_cmp(3);

            -- D5: pick the flat slab colour. Where the two throws overlap
            -- the pair's colours have already been ADDED as light at the
            -- field latch, so the overlap is one flat third colour --
            -- magenta for blue/red, exactly like a real anaglyph.
            s_y5 <= s_y4;
            v_bl := s_pbl and not s_warm;
            v_rd := s_pbr and not s_warm;
            if v_bl = '1' and v_rd = '1' then
                s_cy <= s_cby;  s_cu <= s_cbu;  s_cv <= s_cbv;
            elsif v_bl = '1' then
                s_cy <= s_cly;  s_cu <= s_clu;  s_cv <= s_clv;
            else
                s_cy <= s_cry;  s_cu <= s_cru;  s_cv <= s_crv;
            end if;
            s_has <= v_bl or v_rd;

            -- D6: LAYER Behind lets the picture's own bright areas knock
            -- holes in the colour; DENSITY Wash mixes 3/8 of the slab into
            -- the monochrome instead of replacing it.
            v_y := s_y5;
            v_u := to_unsigned(512, 10);
            v_v := to_unsigned(512, 10);
            if s_has = '1' and not (s_layer_bh = '1' and s_y5(9) = '1') then
                if s_dens_sol = '1' then
                    v_y := s_cy;
                    v_u := s_cu;
                    v_v := s_cv;
                else
                    -- Wash keeps the monochrome readable: only a quarter
                    -- of the slab's luma but three eighths of its chroma,
                    -- so the picture shows through as a tint rather than
                    -- being dragged toward the slab colour. The luma term
                    -- is what keeps White/Black visible in this mode.
                    v_wy := shift_left(resize(s_y5, 14), 1)
                            + shift_left(resize(s_y5, 14), 2)
                            + shift_left(resize(s_cy, 14), 1);
                    v_y  := resize(shift_right(v_wy, 3), 10);
                    v_wu := to_unsigned(2560, 14)
                            + resize(s_cu, 14) + shift_left(resize(s_cu, 14), 1);
                    v_u  := resize(shift_right(v_wu, 3), 10);
                    v_wu := to_unsigned(2560, 14)
                            + resize(s_cv, 14) + shift_left(resize(s_cv, 14), 1);
                    v_v  := resize(shift_right(v_wu, 3), 10);
                end if;
            end if;
            s_oy <= v_y;
            s_ou <= v_u;
            s_ov <= v_v;

            -- D7: blanking gate. A free-running pipeline outside avid rails
            -- the encoder's colour reference, and no simulation scores
            -- anything but the active region.
            if av(6) = '1' then
                s_ry   <= s_oy;
                s_ru   <= s_ou;
                s_rv_o <= s_ov;
            else
                s_ry   <= to_unsigned(64, 10);
                s_ru   <= to_unsigned(512, 10);
                s_rv_o <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_display;

    ----------------------------------------------------------------------------
    -- Per-field parameter latch. Everything the detector or the geometry
    -- depends on changes only between fields.
    ----------------------------------------------------------------------------
    p_params : process(clk)
        variable v_g  : unsigned(12 downto 0);
        variable v_w  : unsigned(15 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_s  : signed(12 downto 0);
        variable v_ay, v_au, v_av : unsigned(9 downto 0);
        variable v_by, v_bu, v_bv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            ------------------------------------------------------------------
            -- Free-running precompute, one short hop per register
            ------------------------------------------------------------------
            -- RANGE -> fraction of the measured width. This is a FRINGE,
            -- not a slab: 0.4%..2.0% of the width, about 8..38 px at HD.
            -- Wide slabs swallowed each other and turned whole regions
            -- flat magenta, which read as blocks rather than edges.
            s_rf_a  <= resize(shift_right(resize(s_k_range, 15)
                                          + shift_left(resize(s_k_range, 15), 4), 10), 12);
            s_rfrac <= to_unsigned(4, 12) + s_rf_a;

            -- BALANCE -> 256 +/- d with a detented centre
            v_d := signed(resize(s_k_bal, 11)) - to_signed(512, 11);
            if v_d > -48 and v_d < 48 then
                s_bal_d <= (others => '0');
            else
                s_bal_d <= v_d;
            end if;
            s_balb <= unsigned(resize(to_signed(256, 11) + shift_right(s_bal_d, 1), 10));
            s_balr <= unsigned(resize(to_signed(256, 11) - shift_right(s_bal_d, 1), 10));

            -- P12 glide error (P12G only moves at a field boundary, so this
            -- has a whole field to settle)
            s_gdiff <= signed(resize(s_k_width, 12)) - signed(resize(s_p12g, 12));

            -- Glow reach, clamped. With only the two outline edges left
            -- there is no interior edge for a flood to stop at, so FILL
            -- simply means "keep going outward": a 32x reach, which clears
            -- the frame from any ordinary subject, still scaled by P12.
            if s_sw_fill = '1' then
                v_w := shift_left(resize(s_wb_s, 16), 5);
            else
                v_w := resize(s_wb_s, 16);
            end if;
            if v_w > 1023 then
                s_wb_c <= to_unsigned(1023, 11);
            else
                s_wb_c <= resize(v_w, 11);
            end if;
            if s_sw_fill = '1' then
                v_w := shift_left(resize(s_wr_s, 16), 5);
            else
                v_w := resize(s_wr_s, 16);
            end if;
            if v_w > 1023 then
                s_wr_c <= to_unsigned(1023, 11);
            else
                s_wr_c <= resize(v_w, 11);
            end if;

            -- EDGE -> detector threshold, in half-luma units. A linear
            -- sweep put 104 luma of required step at the knob's midpoint --
            -- far above any ordinary edge, so the bottom two thirds of the
            -- travel did nothing. Squaring the complement gives 132 luma at
            -- the floor, 36 at the centre and 6 at the top, which is the
            -- range real pictures actually live in.
            s_ei     <= to_unsigned(1023, 10) - s_k_edge;
            s_esq    <= s_ei * s_ei;
            s_thr_t  <= to_unsigned(3, 10) + resize(shift_right(s_esq, 14), 10);
            s_thr_h2 <= s_thr_t - shift_right(s_thr_t, 1);
            s_thr_q2 <= s_thr_t - shift_right(s_thr_t, 2);

            -- BASE: 512 is unity, so 50% on the panel is bit-exact
            if s_k_base < 512 then
                s_gain_p <= to_unsigned(64, 12)
                            + resize(shift_right(resize(s_k_base, 12)
                                                 + shift_left(resize(s_k_base, 12), 1), 3), 12);
            else
                v_g := resize(s_k_base - 512, 13);
                s_gain_p <= to_unsigned(256, 12)
                            + resize(shift_left(resize(v_g, 14), 1)
                                     + shift_right(resize(v_g, 14), 1), 12);
            end if;

            -- Colour pair (+ SWAP) into shadows, then the additive overlap.
            -- Each constant is read exactly once: N reads of one constant
            -- array infer N separate ROMs.
            v_ay := C_LY(to_integer(s_k_pair));
            v_au := C_LU(to_integer(s_k_pair));
            v_av := C_LV(to_integer(s_k_pair));
            v_by := C_RY(to_integer(s_k_pair));
            v_bu := C_RU(to_integer(s_k_pair));
            v_bv := C_RV(to_integer(s_k_pair));
            if s_sw_swap = '0' then
                s_sly <= v_ay;  s_slu <= v_au;  s_slv <= v_av;
                s_sry <= v_by;  s_sru <= v_bu;  s_srv <= v_bv;
            else
                s_sly <= v_by;  s_slu <= v_bu;  s_slv <= v_bv;
                s_sry <= v_ay;  s_sru <= v_au;  s_srv <= v_av;
            end if;
            v_s := signed(resize(s_sly, 13)) + signed(resize(s_sry, 13))
                   - to_signed(64, 13);
            s_sby <= f_clamp10(v_s, 64, 768);
            v_s := signed(resize(s_slu, 13)) + signed(resize(s_sru, 13))
                   - to_signed(512, 13);
            s_sbu <= f_clamp10(v_s, 32, 992);
            v_s := signed(resize(s_slv, 13)) + signed(resize(s_srv, 13))
                   - to_signed(512, 13);
            s_sbv <= f_clamp10(v_s, 32, 992);

            ------------------------------------------------------------------
            -- Field latch. Everything the detector or the geometry depends
            -- on changes only between fields, so a field is never processed
            -- with two different settings -- the seam reads as a fake edge.
            ------------------------------------------------------------------
            if s_fpulse = '1' then
                -- P12 glide: ease toward the target, never snap
                if s_gdiff > 7 or s_gdiff < -7 then
                    s_p12g <= unsigned(resize(signed(resize(s_p12g, 12))
                                              + shift_right(s_gdiff, 3), 10));
                else
                    s_p12g <= s_k_width;
                end if;

                s_thr_hi <= signed(resize(s_thr_t, 9));
                s_thr_lo <= signed(resize(s_thr_h2, 9));
                if s_sw_settle = '1' then
                    s_thr_hiv <= signed(resize(s_thr_h2, 9));
                else
                    s_thr_hiv <= signed(resize(s_thr_q2, 9));
                end if;

                -- CLEAN -> box window, normaliser, posterise mask, gradient
                -- baseline and the mark-centring delay
                case to_integer(s_k_clean(9 downto 7)) is
                    when 0 => s_mdel <= "000"; s_bmask <= x"0100"; s_bshift <= "000";
                              s_pmask <= x"FF"; s_gsel <= "00";
                    when 1 => s_mdel <= "000"; s_bmask <= x"0180"; s_bshift <= "001";
                              s_pmask <= x"FE"; s_gsel <= "00";
                    when 2 => s_mdel <= "001"; s_bmask <= x"0180"; s_bshift <= "001";
                              s_pmask <= x"FC"; s_gsel <= "01";
                    when 3 => s_mdel <= "001"; s_bmask <= x"03C0"; s_bshift <= "010";
                              s_pmask <= x"F8"; s_gsel <= "01";
                    when 4 => s_mdel <= "010"; s_bmask <= x"03C0"; s_bshift <= "010";
                              s_pmask <= x"F0"; s_gsel <= "10";
                    when 5 => s_mdel <= "011"; s_bmask <= x"0FF0"; s_bshift <= "011";
                              s_pmask <= x"E0"; s_gsel <= "10";
                    when 6 => s_mdel <= "101"; s_bmask <= x"0FF0"; s_bshift <= "011";
                              s_pmask <= x"C0"; s_gsel <= "11";
                    when others => s_mdel <= "111"; s_bmask <= x"FFFF"; s_bshift <= "100";
                              s_pmask <= x"C0"; s_gsel <= "11";
                end case;

                s_calm <= s_sw_settle;
                if s_sw_settle = '1' then
                    s_dbnc <= "01";
                else
                    s_dbnc <= "00";
                end if;

                s_gain     <= s_gain_p;
                s_dens_sol <= s_sw_dens;
                s_layer_bh <= s_sw_layer;

                s_wb <= s_wb_c;
                s_wr <= s_wr_c;

                s_cly <= s_sly;  s_clu <= s_slu;  s_clv <= s_slv;
                s_cry <= s_sry;  s_cru <= s_sru;  s_crv <= s_srv;
                s_cby <= s_sby;  s_cbu <= s_sbu;  s_cbv <= s_sbv;
            end if;
        end if;
    end process p_params;

    ----------------------------------------------------------------------------
    -- Width sequencer: one shared multiplier walks a fixed 12-slot schedule,
    -- each product read two slots after its operands are posted. The whole
    -- table refreshes every 12 clocks, so it is instant for a knob.
    ----------------------------------------------------------------------------
    p_wseq : process(clk)
    begin
        if rising_edge(clk) then
            s_prod <= s_ma * s_mb;
            if s_slot = 11 then
                s_slot <= (others => '0');
            else
                s_slot <= s_slot + 1;
            end if;

            case to_integer(s_slot) is
                when 0 =>
                    s_ma <= resize(s_mhalf, 12);
                    s_mb <= s_rfrac;
                when 2 =>
                    s_wmax <= resize(shift_right(s_prod, 10), 12);
                when 3 =>
                    s_ma <= s_wmax;
                    s_mb <= resize(s_p12g, 12);
                when 5 =>
                    s_wcur <= resize(shift_right(s_prod, 10), 12);
                when 6 =>
                    s_ma <= s_wcur;
                    s_mb <= resize(s_balb, 12);
                when 8 =>
                    s_wb_s <= resize(shift_right(s_prod, 8), 11);
                when 9 =>
                    s_ma <= s_wcur;
                    s_mb <= resize(s_balr, 12);
                when 11 =>
                    s_wr_s <= resize(shift_right(s_prod, 8), 11);
                when others =>
                    null;
            end case;
        end if;
    end process p_wseq;

    ----------------------------------------------------------------------------
    -- Sync delay: one latency for every mode.
    ----------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= s_avid_sr   (C_SYNC_DELAY - 2 downto 0) & data_in.avid;
            s_hsync_n_sr <= s_hsync_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.hsync_n;
            s_vsync_n_sr <= s_vsync_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.vsync_n;
            s_field_n_sr <= s_field_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.field_n;
        end if;
    end process p_sync;

    data_out.y <= std_logic_vector(s_ry);
    data_out.u <= std_logic_vector(s_ru);
    data_out.v <= std_logic_vector(s_rv_o);
    data_out.avid    <= s_avid_sr   (C_SYNC_DELAY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_SYNC_DELAY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_SYNC_DELAY - 1);
    data_out.field_n <= s_field_n_sr(C_SYNC_DELAY - 1);

end parallax;
