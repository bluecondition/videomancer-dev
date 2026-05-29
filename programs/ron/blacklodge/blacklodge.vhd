-- Blacklodge: Red Room (Twin Peaks) synthesiser
--
-- Composite view of two full-screen layers separated by a horizon:
--   Upper band : deep crimson velvet curtain with vertical folds (soft
--                triangle-wave luma modulation; ridges bright, troughs
--                near-black). Optional per-frame horizontal sway.
--   Lower band : warm cream / dark-brown chevron floor, wavelength
--                compresses toward the horizon for forced perspective.
--                "Vee" mode mirrors the pattern about the vertical
--                centre for an inward-pointing herringbone.
--
-- No multipliers on per-pixel data (HX4K-friendly).  Perspective is
-- achieved with a piecewise-case shift amount driven by distance below
-- the horizon.  The curtain fold pattern sums two triangle waves at
-- different periods to break visible uniformity.
--
-- Register map (each std_logic_vector(9 downto 0)):
--   registers_in(0) = Folds     (fold density: top 2 bits select period)
--   registers_in(1) = Depth     (fold shadow depth 0..1023)
--   registers_in(2) = Chevron   (base chevron thickness near horizon)
--   registers_in(3) = Apex      (chevron VP horizontal position;
--                                 0 = far left, 512 = centre, 1023 =
--                                 far right of the active line)
--   registers_in(4) = Scroll    (horizontal phase offset added to
--                                 dx_phase; sweep = ~one stripe pair)
--   registers_in(5) = ChevFreq  (top 3 bits: tooth period selector;
--                                 top 5 bits: perspective compression)
--   registers_in(6) = Switches  (bit0=Sway bit1=ChevDir bit2=Brown
--                                bit3=Vignette bit4=Key)
--   registers_in(7) = Horizon   (slider, REVERSED: top of slider =
--                                raised curtain (small horizon y),
--                                bottom = lowered curtain)
--
-- License: GPL-3.0
-- Author: ron

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture blacklodge of program_top is

    constant LATENCY : natural := 8;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    ----------------------------------------------------------------------
    -- Palette constants (10-bit YUV, Videomancer hardware convention:
    -- BT.601 Cr → U register, Cb → V register).  Same swap used by the
    -- CGA and Phosphor programs and verified on a scope.
    ----------------------------------------------------------------------
    -- Deep crimson curtain (#C8243A highlight)
    constant C_CURTAIN_HI_Y : unsigned(9 downto 0) := to_unsigned(350, 10);
    constant C_CURTAIN_HI_U : unsigned(9 downto 0) := to_unsigned(833, 10);  -- Cr
    constant C_CURTAIN_HI_V : unsigned(9 downto 0) := to_unsigned(445, 10);  -- Cb

    -- Deep maroon shadow (#2E0508)
    constant C_CURTAIN_LO_Y : unsigned(9 downto 0) := to_unsigned( 70, 10);
    constant C_CURTAIN_LO_U : unsigned(9 downto 0) := to_unsigned(593, 10);  -- Cr
    constant C_CURTAIN_LO_V : unsigned(9 downto 0) := to_unsigned(490, 10);  -- Cb

    -- Warm cream (#E8DCC4)
    constant C_FLOOR_LT_Y   : unsigned(9 downto 0) := to_unsigned(883, 10);
    constant C_FLOOR_LT_U   : unsigned(9 downto 0) := to_unsigned(544, 10);  -- Cr
    constant C_FLOOR_LT_V   : unsigned(9 downto 0) := to_unsigned(456, 10);  -- Cb

    -- Warm dark brown (#2A1A18)
    constant C_FLOOR_DK_Y   : unsigned(9 downto 0) := to_unsigned(122, 10);
    constant C_FLOOR_DK_U   : unsigned(9 downto 0) := to_unsigned(545, 10);  -- Cr
    constant C_FLOOR_DK_V   : unsigned(9 downto 0) := to_unsigned(497, 10);  -- Cb

    -- Saturated warm brown (Brown switch T9 swaps the dark chevron
    -- stripes from near-black to this).  More red-and-yellow saturation
    -- than the old sienna value so the floor reads as proper brown
    -- rather than tinted black.
    constant C_FLOOR_BR_Y   : unsigned(9 downto 0) := to_unsigned(170, 10);
    constant C_FLOOR_BR_U   : unsigned(9 downto 0) := to_unsigned(620, 10);  -- Cr
    constant C_FLOOR_BR_V   : unsigned(9 downto 0) := to_unsigned(458, 10);  -- Cb

    constant C_CHROMA_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- Curtain hem LUT — gives each fold a rounded bottom that hangs down
    -- below the horizon line.  Indexed by 6 top bits of fold_phase (= a
    -- position within the current fold), output is hem-offset in pixels
    -- (0..12).  Parabola: hem(p) = 4·R·p·(N-1-p)/(N-1)² → smooth bulge
    -- centred on each fold, reaching zero at the seams between folds.
    ----------------------------------------------------------------------
    type t_hem_lut is array (0 to 63) of unsigned(3 downto 0);

    function gen_hem_lut return t_hem_lut is
        variable r   : t_hem_lut;
        variable val : integer;
    begin
        for i in 0 to 63 loop
            val := (4 * 12 * i * (63 - i) + 1985) / 3969;
            if val > 15 then
                val := 15;
            end if;
            r(i) := to_unsigned(val, 4);
        end loop;
        return r;
    end function;

    constant C_HEM_LUT : t_hem_lut := gen_hem_lut;

    -- Curtain lerp lookup tables — replace per-pixel multipliers with a
    -- 64-entry constant ROM per channel.  Indexed by 6-bit blend_factor;
    -- output is (HI - LO) × blend / 64 (or LO - HI for V).
    type t_lerp_y_lut is array (0 to 63) of unsigned(8 downto 0);
    type t_lerp_u_lut is array (0 to 63) of unsigned(7 downto 0);
    type t_lerp_v_lut is array (0 to 63) of unsigned(5 downto 0);

    function gen_lerp_y_lut return t_lerp_y_lut is
        variable r : t_lerp_y_lut;
    begin
        for i in 0 to 63 loop
            r(i) := to_unsigned((280 * i) / 64, 9);
        end loop;
        return r;
    end function;

    function gen_lerp_u_lut return t_lerp_u_lut is
        variable r : t_lerp_u_lut;
    begin
        for i in 0 to 63 loop
            r(i) := to_unsigned((240 * i) / 64, 8);
        end loop;
        return r;
    end function;

    function gen_lerp_v_lut return t_lerp_v_lut is
        variable r : t_lerp_v_lut;
    begin
        for i in 0 to 63 loop
            r(i) := to_unsigned((45 * i) / 64, 6);
        end loop;
        return r;
    end function;

    constant C_LERP_Y_LUT : t_lerp_y_lut := gen_lerp_y_lut;
    constant C_LERP_U_LUT : t_lerp_u_lut := gen_lerp_u_lut;
    constant C_LERP_V_LUT : t_lerp_v_lut := gen_lerp_v_lut;

    ----------------------------------------------------------------------
    -- Position / frame counters
    ----------------------------------------------------------------------
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');
    signal max_x_r      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal max_y_r      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal h_centre_r   : unsigned(11 downto 0) := to_unsigned(960, 12);

    ----------------------------------------------------------------------
    -- Horizon (registered each vsync from pot 4)
    -- Derived once per frame so the per-pixel path only sees a compare.
    ----------------------------------------------------------------------
    signal horizon_r      : unsigned(11 downto 0) := to_unsigned(540, 12);
    -- Minimum chevron wavelength (in pixels, near horizon)
    -- Curtain fold phase DDA — frequency mapped continuously from pot 1
    signal fold_freq_r    : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal fold_phase_r   : unsigned(15 downto 0) := (others => '0');
    -- Hem amplitude shift derived from fold_freq top 2 bits.  Narrow
    -- folds shift the LUT output right (smaller hem), so the pointy
    -- ends of tight folds don't punch through with a huge drop shadow.
    signal hem_shift_r    : unsigned(1 downto 0) := "00";
    signal depth_amt_r    : unsigned(9 downto 0)  := to_unsigned(512, 10);
    -- Pot 6 now controls the V-tooth frequency (was Warmth).  3-bit
    -- selector picks one of 8 row-phase advance rates from /16 (slow,
    -- long teeth) up to ×8 (very fast, dense teeth).
    signal tooth_freq_sel_r : unsigned(2 downto 0) := "100";
    -- Precomputed signed depth offset for the curtain blend (per-frame).
    signal depth_off_s_r  : signed(11 downto 0)   := (others => '0');
    -- K5 (Scroll) → horizontal phase offset added to dx_phase init.
    -- 20-bit holds four full stripe pairs (= 4 × 2^18 of phase) so a
    -- K5 sweep slides the chevron pattern by ~four LO/LT cycles.  Each
    -- K5 step is ~2^10 phase units → sub-pixel motion at every wavelen.
    signal scroll_offset_r : unsigned(19 downto 0) := (others => '0');

    signal sway_en_r      : std_logic := '1';
    -- chev_vee_r is undriven (always '0') — leftover from the OLD phase-
    -- DDA renderer.  Kept so the dead code still compiles; synthesizer
    -- prunes the branches that gate on '1'.
    signal chev_vee_r     : std_logic := '0';
    -- T8 = Chev Dir.  '0' = horizontal (V-stripes radiate from the
    -- vanishing point with a horizontal zigzag tooth, original look);
    -- '1' = 90°-rotated (horizontal stripes parallel to the horizon
    -- with a vertical zigzag).  Both modes use a single vanishing
    -- point fixed at screen centre.
    signal chev_dir_r     : std_logic := '0';
    -- T11 = Key: when on, floor pixels (below the horizon/curtain)
    -- are replaced with the delayed input video while the curtain
    -- area continues to render the red-curtain synth.  The chevron
    -- pattern is suppressed; everything else (curtain, vignette,
    -- brightness) still applies above the horizon.
    signal key_en_r       : std_logic := '0';
    signal vign_en_r      : std_logic := '1';

    -- Pre-selected dark-stripe colour (= DK or BR depending on T9
    -- Brown).  Resolving the 2-way choice once per frame keeps s4's
    -- floor-select as a tight 2:1 mux between LT and these registers.
    signal dk_y_r         : unsigned(9 downto 0) := (others => '0');
    signal dk_u_r         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal dk_v_r         : unsigned(9 downto 0) := C_CHROMA_MID;

    -- Per-band sway: 4 accumulators advancing at coprime per-frame rates.
    -- Each curtain band hashes onto one of these so the folds don't all
    -- ping-pong in lockstep — different folds drift at different speeds.
    type t_sway_accums is array (0 to 3) of unsigned(15 downto 0);
    constant C_SWAY_SPEEDS : t_sway_accums := (
        to_unsigned(173, 16),
        to_unsigned(211, 16),
        to_unsigned(257, 16),
        to_unsigned(311, 16));
    signal sway_accums_r : t_sway_accums := (others => (others => '0'));
    -- Triangle-wave outputs precomputed at vsync — per-pixel path only
    -- needs a 4:1 mux to grab one based on band hash.
    signal sway_tris_r   : t_sway_accums := (others => (others => '0'));

    -- Pre-registered per-pixel band hash and the muxed sway value, so s1
    -- only does a 3-input 16-bit add (hash compute + mux happen one cycle
    -- earlier).  1-pixel offset is invisible since both signals are slow.
    signal pre_hash_r    : unsigned(15 downto 0) := (others => '0');
    signal pre_sway_r    : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Chevron phase DDA (horizon-anchored, linear wavelength in depth)
    --
    -- Horizontal stripe pattern: 18-bit phase accumulator, phase += freq
    -- each pixel (or -= in Vee mode left half).  stripe_colour = phase(17).
    --
    -- Per row, wavelen advances by a fractional step: wavelen_int.frac
    -- where frac is 8-bit.  Each row, frac += step; on overflow,
    -- wavelen_int += 1.  freq_row is then computed as a linearly-
    -- interpolated value between LUT(wavelen_int) and LUT(wavelen_int+1).
    --
    -- This gives strictly LINEAR wavelength growth with depth → stripe
    -- boundaries are perfectly straight perspective rays from the horizon.
    -- (The previous linear-freq scheme made wavelen hyperbolic in depth,
    -- so stripes curved on the way to the foreground.)
    --
    -- Tooth (chevron V apex): row_phase advances by freq_row every hsync
    -- on the floor — wraps at 2^17 after exactly wavelen rows, so the
    -- tooth period scales with the local wavelength, preserving 45° apex
    -- slope at every depth.  tooth_phase is a triangle of row_phase,
    -- amplitude 2^16 → apex shift of wavelen/2 pixels at any wavelength.
    --
    -- Vee mode: phase_init = horizon * freq + tooth_phase.  DDA runs
    -- backward for pixel_x < horizon, forward otherwise → inward-pointing
    -- chevrons converging at the vanishing point.
    ----------------------------------------------------------------------
    -- Reciprocal LUT for wavelen ∈ [2, 127] → 2^17 / wavelen (17-bit).
    type t_freq_lut is array (0 to 127) of unsigned(16 downto 0);

    function gen_freq_lut return t_freq_lut is
        variable r : t_freq_lut;
    begin
        r := (others => to_unsigned(1032, 17));  -- floor at wavelen=127
        for i in 2 to 127 loop
            r(i) := to_unsigned(131072 / i, 17);
        end loop;
        return r;
    end function;

    constant C_FREQ_LUT : t_freq_lut := gen_freq_lut;

    -- Per-frame constants (parameters latched at vsync)
    signal chev_min_int_r : unsigned(6 downto 0) := to_unsigned(10, 7);
    -- Vanishing-point X position from pot 4 (max_x * pot / 1024)
    signal eff_horizon_x_r: unsigned(11 downto 0) := to_unsigned(960, 12);
    -- Perspective amount fixed at a tasteful mid value
    signal step_r         : unsigned(7 downto 0) := to_unsigned(20, 8);

    -- Per-row wavelength state (fractional 7.8 fixed-point)
    signal wavelen_int_r  : unsigned(6 downto 0) := to_unsigned(10, 7);
    signal wavelen_frac_r : unsigned(7 downto 0) := (others => '0');
    -- Pre-registered "would frac+step overflow" flag — saves having a
    -- 9-bit add + compare inside the hsync register-write critical path.
    signal wavelen_carry_r : std_logic := '0';

    -- Always-on reciprocal-LUT pipeline
    signal freq_a_r       : unsigned(16 downto 0) := (others => '0');
    signal freq_b_r       : unsigned(16 downto 0) := (others => '0');
    signal freq_diff_r    : unsigned(16 downto 0) := (others => '0');
    signal freq_a_d1_r    : unsigned(16 downto 0) := (others => '0');
    signal freq_a_d2_r    : unsigned(16 downto 0) := (others => '0');
    signal freq_interp_r  : unsigned(24 downto 0) := (others => '0');
    signal freq_row_r     : unsigned(16 downto 0) := to_unsigned(13107, 17);

    -- Tooth / phase init state
    signal row_phase_r    : unsigned(16 downto 0) := (others => '0');
    signal tooth_phase_r  : unsigned(16 downto 0) := (others => '0');
    signal eff_horizon_r  : unsigned(11 downto 0) := (others => '0');

    -- 2-stage split multiplier for eff_horizon * freq_row (12x17 = 29-bit)
    --   stage 1: 6x17 partials on high/low 6 bits of eff_horizon
    --   stage 2: sum partials + add tooth_phase → phase_init_r (18-bit wrap)
    signal phase_mul_hi_r : unsigned(22 downto 0) := (others => '0');
    signal phase_mul_lo_r : unsigned(22 downto 0) := (others => '0');
    signal phase_init_r   : unsigned(17 downto 0) := (others => '0');
    signal phase_r        : unsigned(17 downto 0) := (others => '0');
    -- Pre-registered backward-direction flag (Vee mode left half).
    -- Splits the long compare-then-add path in the per-pixel chevron DDA.
    signal stripe_back_pre_r : std_logic := '0';

    -- Align stripe_color (= phase_r(17)) with the 3-stage compose pipeline
    signal stripe_color_d1 : std_logic := '0';
    signal stripe_color_d2 : std_logic := '0';
    signal stripe_color_d3 : std_logic := '0';

    ----------------------------------------------------------------------
    -- Real-chevron renderer (Option 2): V shapes drawn directly rather
    -- than as a horizontally-shifted stripe pattern.  Apex sits at
    -- (eff_horizon_x_r, horizon_r); slope = 1 → 90° apex angle.  Bands
    -- of constant chev coordinate alternate LO/LT every 2^N rows, so
    -- swing is bounded only by screen width — no stripe-wavelen wrap.
    --
    -- chev_y(x, y) = (y - horizon) - |x - eff_horizon_x_r|   (clamped ≥ 0)
    -- color       = chev_y(N)                                (band-LSB)
    ----------------------------------------------------------------------
    -- Perspective accumulator: every band-pair (LO+LT) corresponds to
    -- 2^17 chev units.  Bands are smaller (rows-wise) near horizon
    -- where freq_row is large, larger at foreground where freq_row is
    -- small — naturally producing the floor's perspective look.
    constant CHEV_BAND_BIT : natural := 16;  -- color = chev_y_pers(16)

    -- Two parallel d1 registers — one per chev direction.  The mux
    -- between them is deferred to the d1→d2 stage so the d1 input
    -- path stays just (add → bit) rather than (add → bit → mux).
    signal chev_x_color_d1 : std_logic := '0';
    signal chev_y_color_d1 : std_logic := '0';
    signal chev_color_d2 : std_logic := '0';
    signal chev_color_d3 : std_logic := '0';

    -- Perspective state for direct V-renderer
    signal y_acc_r          : unsigned(23 downto 0) := (others => '0');
    signal dx_mul_hi_r      : unsigned(22 downto 0) := (others => '0');
    signal dx_mul_lo_r      : unsigned(22 downto 0) := (others => '0');
    signal dx_phase_init_r  : unsigned(23 downto 0) := (others => '0');
    signal dx_phase_r       : signed(25 downto 0)   := (others => '0');

    ----------------------------------------------------------------------
    -- Stage 1: pre-compute fold_phase, depth, is_floor
    ----------------------------------------------------------------------
    signal s1_x         : unsigned(11 downto 0) := (others => '0');
    signal s1_y         : unsigned(11 downto 0) := (others => '0');
    signal s1_phase     : unsigned(15 downto 0) := (others => '0');
    signal s1_depth     : unsigned(11 downto 0) := (others => '0');
    signal s1_is_floor  : std_logic := '0';
    -- Drop-shadow strength under the curtain hem.  0 = no shadow,
    -- 15 = darkest (right under the hem edge).  Falls off over 16 rows.
    signal s1_shadow_amt : unsigned(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Stage 2: curtain fold triangle (single tri from continuous phase)
    ----------------------------------------------------------------------
    signal s2_x         : unsigned(11 downto 0) := (others => '0');
    signal s2_y         : unsigned(11 downto 0) := (others => '0');
    signal s2_is_floor  : std_logic := '0';
    signal s2_shadow_amt: unsigned(3 downto 0) := (others => '0');
    signal s2_tri_a     : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Stage 3: fold_luma + curtain blend_factor (depth-shifted)
    ----------------------------------------------------------------------
    signal s3_x            : unsigned(11 downto 0) := (others => '0');
    signal s3_y            : unsigned(11 downto 0) := (others => '0');
    signal s3_is_floor     : std_logic := '0';
    signal s3_shadow_amt   : unsigned(3 downto 0) := (others => '0');
    signal s3_fold_luma    : unsigned(9 downto 0) := (others => '0');
    -- 6-bit blend (64 steps) — small enough for cheap mul, smooth enough
    -- that no perceivable stepping is visible as the Depth knob is swept.
    signal s3_blend_factor : unsigned(5 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Stage 4: interpolated colour for both layers
    ----------------------------------------------------------------------
    signal s4_x         : unsigned(11 downto 0) := (others => '0');
    signal s4_y         : unsigned(11 downto 0) := (others => '0');
    signal s4_is_floor  : std_logic := '0';
    signal s4_cur_y     : unsigned(9 downto 0) := (others => '0');
    signal s4_cur_u     : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s4_cur_v     : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s4_flr_y     : unsigned(9 downto 0) := (others => '0');
    signal s4_flr_u     : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s4_flr_v     : unsigned(9 downto 0) := C_CHROMA_MID;

    -- Soft-vignette pipeline (always-on, 4-stage to relax timing).
    -- fade_lvl_r 127 = full, 0 = full black at outer edge.  7-bit
    -- gives 128 levels = ~0.78% steps → continuous fade rather than
    -- visible bands.  Applied as a 10x7 multiply at stage 5.
    signal dist_l_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_r_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_t_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_b_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_x_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_y_r     : unsigned(11 downto 0) := (others => '0');
    signal min_dist_r   : unsigned(11 downto 0) := (others => '0');
    signal fade_lvl_r   : unsigned(6 downto 0) := (others => '1');

    ----------------------------------------------------------------------
    -- Stage 5: layer select + vignette
    ----------------------------------------------------------------------
    signal s5_y : unsigned(9 downto 0) := (others => '0');
    signal s5_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s5_v : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s5_is_floor : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 6: brightness scale (3-bit selector, case shift)
    ----------------------------------------------------------------------
    signal s6_y : unsigned(9 downto 0) := (others => '0');
    signal s6_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s6_v : unsigned(9 downto 0) := C_CHROMA_MID;
    -- Pre-registered key-mux select (= key_en_r AND s5_is_floor).
    -- Folding the AND into a register keeps the output-port mux a
    -- pure 2:1 select on two registered inputs — minimal comb depth.
    signal key_floor_r : std_logic := '0';

    ----------------------------------------------------------------------
    -- 16-bit multiplicative hash for grain / variation
    ----------------------------------------------------------------------
    function hash16(a : unsigned(15 downto 0); b : unsigned(15 downto 0))
        return unsigned is
        variable h    : unsigned(15 downto 0);
        variable prod : unsigned(31 downto 0);
    begin
        h    := a xor b xor to_unsigned(16#A53F#, 16);
        prod := h * to_unsigned(16#9E37#, 16);
        h    := prod(15 downto 0);
        h    := h xor shift_right(h, 7);
        return h;
    end function;

    ----------------------------------------------------------------------
    -- Triangle wave from an n-bit phase, returns n-bit amplitude
    --   top bit of phase selects rising (phase(n-2..0)) or falling
    --   (~phase(n-2..0)).  Rendered in 10-bit output space.
    ----------------------------------------------------------------------
    function triwave10(
        ph   : unsigned;
        nbits: natural           -- phase width (includes sign bit)
    ) return unsigned is
        variable norm  : unsigned(nbits - 1 downto 0);
        variable low   : unsigned(nbits - 2 downto 0);
        variable rising: std_logic;
        variable full  : unsigned(9 downto 0);
    begin
        norm   := ph;                           -- normalise indexing
        low    := norm(nbits - 2 downto 0);
        rising := not norm(nbits - 1);
        if rising = '1' then
            full := resize(shift_left(resize(low, 10), 10 - (nbits - 1)), 10);
        else
            full := resize(shift_left(resize(
                            to_unsigned(2 ** (nbits - 1) - 1, nbits - 1) - low,
                            10),
                           10 - (nbits - 1)), 10);
        end if;
        return full;
    end function;

begin

    process(clk)
        variable v_fold_x       : unsigned(11 downto 0);
        variable v_depth        : unsigned(11 downto 0);
        variable v_sway         : signed(9 downto 0);
        variable v_tri_a_bits   : unsigned(6 downto 0);
        variable v_tri_b_bits   : unsigned(5 downto 0);
        variable v_tri_a        : unsigned(9 downto 0);
        variable v_tri_b        : unsigned(9 downto 0);
        variable v_fold_sum     : unsigned(10 downto 0);
        variable v_fold_luma    : unsigned(9 downto 0);
        variable v_dark_amt     : unsigned(19 downto 0);
        variable v_dark_scaled  : unsigned(9 downto 0);
        variable v_shift        : unsigned(2 downto 0);
        variable v_v_off        : unsigned(5 downto 0);
        variable v_zig_phase    : unsigned(5 downto 0);
        variable v_stripe_x     : unsigned(11 downto 0);
        variable v_stripe_raw   : unsigned(11 downto 0);
        variable v_stripe_bit   : std_logic;
        variable v_horizon_v    : unsigned(11 downto 0);
        variable v_chev_x       : unsigned(11 downto 0);
        variable v_half_x       : unsigned(11 downto 0);
        variable v_cur_y_diff   : unsigned(9 downto 0);
        variable v_cur_y_add    : unsigned(10 downto 0);
        variable v_grain        : unsigned(15 downto 0);
        variable v_in_vign      : std_logic;
        variable v_y_out        : unsigned(9 downto 0);
        variable v_u_out        : unsigned(9 downto 0);
        variable v_v_out        : unsigned(9 downto 0);
        variable v_vgrad        : unsigned(7 downto 0);
        variable v_cur_hi_y     : unsigned(9 downto 0);
        variable v_tri_bit_idx  : integer range 0 to 6;
        variable v_inv_fold     : unsigned(9 downto 0);
        variable v_if5          : unsigned(4 downto 0);
        variable v_dp5          : unsigned(4 downto 0);
        variable v_prod10       : unsigned(9 downto 0);
        variable v_horz_slv     : std_logic_vector(9 downto 0);
        variable v_horz_pot     : unsigned(9 downto 0);
        variable v_horz_mul     : unsigned(21 downto 0);
        variable v_apex_mul     : unsigned(21 downto 0);
        variable v_fade_mul     : unsigned(16 downto 0);
        variable v_persp_pot    : unsigned(9 downto 0);
        variable v_persp_mul    : unsigned(21 downto 0);
        variable v_fold_freq    : unsigned(11 downto 0);
        variable v_fold_freq_pre: unsigned(12 downto 0);
        variable v_phase_with   : unsigned(15 downto 0);
        variable v_band_accum   : unsigned(15 downto 0);
        variable v_sway_tri     : unsigned(15 downto 0);
        variable v_tooth_scaled : unsigned(17 downto 0);
        variable v_hem_raw      : unsigned(3 downto 0);
        variable v_hem          : unsigned(3 downto 0);
        variable v_curtain_bot  : unsigned(11 downto 0);
        variable v_shadow_d     : unsigned(11 downto 0);
        variable v_flr_y_var    : unsigned(9 downto 0);
        variable v_depth_signed : signed(11 downto 0);
        variable v_blend_signed : signed(11 downto 0);
        variable v_blend_factor : unsigned(5 downto 0);
        variable v_lerp_y_mul   : unsigned(14 downto 0);
        variable v_lerp_u_mul   : unsigned(13 downto 0);
        variable v_lerp_v_mul   : unsigned(11 downto 0);
        variable v_row_depth    : unsigned(11 downto 0);
        variable v_row_depth_s  : unsigned(11 downto 0);
        variable v_wavelen_new  : unsigned(11 downto 0);
        variable v_wavelen7     : unsigned(6 downto 0);
        variable v_tooth_next   : unsigned(7 downto 0);
        variable v_tooth_limit  : unsigned(7 downto 0);
        variable v_v_off_new    : unsigned(6 downto 0);
        variable v_eff_horz_new : unsigned(11 downto 0);
        variable v_freq_new     : unsigned(16 downto 0);
        variable v_phase_hi_18  : unsigned(17 downto 0);
        variable v_phase_lo_18  : unsigned(17 downto 0);
        variable v_freq_ext     : unsigned(17 downto 0);
        variable v_row_phase_sum: unsigned(21 downto 0);
        variable v_row_phase_17 : unsigned(16 downto 0);
        variable v_advance      : unsigned(20 downto 0);
        variable v_dist_x       : unsigned(11 downto 0);
        variable v_tooth_pers   : unsigned(19 downto 0);
        variable v_tooth_x_pers : unsigned(19 downto 0);
        variable v_chev_x_pers  : signed(25 downto 0);
        variable v_chev_y_pers  : signed(25 downto 0);
        variable v_dist_y       : unsigned(11 downto 0);
        variable v_min_dist     : unsigned(11 downto 0);
        variable v_u_signed     : signed(10 downto 0);
        variable v_v_signed     : signed(10 downto 0);
        variable v_u_recombined : unsigned(10 downto 0);
        variable v_v_recombined : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then

            ------------------------------------------------------------
            -- Input delay pipeline
            ------------------------------------------------------------
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            ------------------------------------------------------------
            -- Position counters + auto-measured resolution
            ------------------------------------------------------------
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            if data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                if pixel_x > to_unsigned(0, 12) then
                    max_x_r    <= pixel_x;
                    h_centre_r <= '0' & pixel_x(11 downto 1);
                end if;
                pixel_x <= (others => '0');
                pixel_y <= pixel_y + 1;
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                if pixel_y > to_unsigned(0, 12) then
                    max_y_r <= pixel_y;
                end if;
                pixel_y     <= (others => '0');
                frame_count <= frame_count + 1;

                -- Horizon derived from full 10-bit pot 5: horizon =
                -- max_y * pot / 1024.  12x10 once-per-frame multiply gives
                -- smooth 1080-line precision so the knob can be modulated.
                -- Horizon now from the slider (reg 7), reversed so
                -- slider-top (= max value 1023) raises the curtain
                -- (horizon y → 0) and slider-bottom drops it.
                v_horz_slv  := registers_in(7);
                v_horz_pot  := to_unsigned(1023, 10) - unsigned(v_horz_slv);
                v_horz_mul  := resize(max_y_r * v_horz_pot, 22);
                v_horizon_v := v_horz_mul(21 downto 10);
                horizon_r   <= v_horizon_v;

                -- Chevron: minimum wavelength at horizon.  Thin stripes
                -- K3 = stripe width at horizon.  Top 5 bits of K3 (0..31)
                -- + 4-px floor → 4..35 px range.  Default K3 ≈ 320 →
                -- chev_min ≈ 14 px (matches the prior hardcoded value).
                chev_min_int_r <= resize(unsigned(
                                    registers_in(2)(9 downto 5)), 7)
                                + to_unsigned(4, 7);

                -- K4 (Apex) = chevron vanishing-point horizontal
                -- position.  pot=0 → VP at far left, pot=512 → centre,
                -- pot=1023 → VP at far right.  eff_horizon_x =
                -- max_x * K4 / 1024.  Computed once per vsync; the
                -- per-pixel chevron DDA picks it up through the
                -- existing dx_phase_init split-multiply pipeline.
                v_apex_mul := resize(max_x_r * unsigned(registers_in(3)), 22);
                eff_horizon_x_r <= v_apex_mul(21 downto 10);
                -- K6 = viewer height.  Higher K6 = higher viewer = milder
                -- perspective = slower wavelen growth.  step = 35 - top5
                -- of K6 → range 4..35 (2 extra levels at the strong end
                -- vs the prior 33-offset).  Default K6 ≈ 800 (top5=25)
                -- gives step ≈ 10 (mild-moderate perspective).
                step_r <= to_unsigned(35, 8)
                        - resize(unsigned(
                            registers_in(5)(9 downto 5)), 8);

                -- Folds: continuous wavelength from pot 1.  Saturate at
                -- the value the previous build's 90% setting produced
                -- (= 3936) so the top end of the pot stays in the tight-
                -- folds zone and never wraps past the 12-bit width.
                v_fold_freq_pre := shift_left(
                                       resize(unsigned(registers_in(0)), 13),
                                       2)
                                 + to_unsigned(256, 13);
                if v_fold_freq_pre > to_unsigned(3936, 13) then
                    v_fold_freq := to_unsigned(3936, 12);
                else
                    v_fold_freq := v_fold_freq_pre(11 downto 0);
                end if;
                fold_freq_r <= v_fold_freq;
                -- Hem amplitude scales inversely with fold width: top 2
                -- bits of fold_freq pick a shift that halves the hem
                -- (and thus shadow prominence) at every doubling of freq.
                hem_shift_r <= v_fold_freq(11 downto 10);

                depth_amt_r  <= unsigned(registers_in(1));
                -- Precompute (depth - 512) signed offset for curtain blend
                depth_off_s_r <=
                    signed(resize(unsigned(registers_in(1)), 12))
                  - signed(resize(C_CHROMA_MID, 12));

                -- Pot 6 = V-tooth frequency selector (top 3 bits → 0..7,
                -- maps to row-phase advance rates from /16 to ×8).
                tooth_freq_sel_r <= unsigned(registers_in(5)(9 downto 7));

                -- K5 (Scroll): horizontal phase offset added to
                -- dx_phase init.  K5 << 10 → max ≈ 2^20 phase units ≈
                -- four full stripe pairs, so a K5 sweep slides the
                -- pattern by ~four cycles while each step stays well
                -- under one pixel at every wavelen (smooth motion).
                scroll_offset_r <= shift_left(
                    resize(unsigned(registers_in(4)), 20), 10);

                sway_en_r    <= registers_in(6)(0);
                chev_dir_r   <= registers_in(6)(1);
                vign_en_r    <= registers_in(6)(3);
                key_en_r     <= registers_in(6)(4);

                -- T9 = Brown.  Pre-select the dark-stripe colour for
                -- s4 (DK = near-black, BR = saturated warm brown).
                if registers_in(6)(2) = '1' then
                    dk_y_r <= C_FLOOR_BR_Y;
                    dk_u_r <= C_FLOOR_BR_U;
                    dk_v_r <= C_FLOOR_BR_V;
                else
                    dk_y_r <= C_FLOOR_DK_Y;
                    dk_u_r <= C_FLOOR_DK_U;
                    dk_v_r <= C_FLOOR_DK_V;
                end if;

                -- Advance each per-band sway accumulator by its (coprime)
                -- speed.  Each band picks one of these via hash, giving
                -- the curtains random non-quantised ping-pong motion.
                for i in 0 to 3 loop
                    sway_accums_r(i) <= sway_accums_r(i) + C_SWAY_SPEEDS(i);
                    -- Precompute the triangle-wave output (14-bit ≈ ¼-
                    -- wavelen amplitude) so the per-pixel path is just
                    -- a 4-way mux, no triangle math.
                    if sway_accums_r(i)(15) = '0' then
                        sway_tris_r(i) <= resize(
                            sway_accums_r(i)(14 downto 1), 16);
                    else
                        sway_tris_r(i) <= resize(
                            not sway_accums_r(i)(14 downto 1), 16);
                    end if;
                end loop;
            end if;

            -- Per-pixel curtain fold phase accumulator (resets at hsync)
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                fold_phase_r <= (others => '0');
            elsif data_in.avid = '1' then
                fold_phase_r <= fold_phase_r + resize(fold_freq_r, 16);
            end if;

            -- Pre-register per-pixel band hash and muxed sway value.  s1
            -- reads them as registered inputs (1-pixel offset, harmless).
            v_grain := hash16(
                resize(pixel_x(11 downto 5), 16),
                to_unsigned(16#7C2D#, 16));
            pre_hash_r <= v_grain;
            pre_sway_r <= sway_tris_r(to_integer(v_grain(1 downto 0)));

            ------------------------------------------------------------
            -- Chevron — linear-wavelength advance with reciprocal LUT
            --
            -- Per hsync on floor: wavelen += step/256.  Lookup C_FREQ_LUT
            -- at wavelen_int and wavelen_int+1; linearly interpolate by
            -- wavelen_frac to get freq_row.  Result: freq is a smooth
            -- continuous function of fractional wavelen, with no integer-
            -- step seams, AND wavelen itself is linear in depth → stripe
            -- boundaries trace perfectly straight rays from the horizon.
            ------------------------------------------------------------

            -- Always-on: precompute "(frac + step) >= 256" overflow flag
            -- so the hsync block only reads a registered bit.
            if ('0' & wavelen_frac_r) + ('0' & step_r)
                    >= to_unsigned(256, 9) then
                wavelen_carry_r <= '1';
            else
                wavelen_carry_r <= '0';
            end if;

            -- Per-hsync updates
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                if pixel_y + 1 >= horizon_r then
                    -- Advance wavelen_frac (8-bit, modular wrap is fine)
                    wavelen_frac_r <= wavelen_frac_r + step_r;
                    -- Carry into wavelen_int (saturated at 127)
                    if wavelen_carry_r = '1'
                            and wavelen_int_r < to_unsigned(127, 7) then
                        wavelen_int_r <= wavelen_int_r + 1;
                    end if;

                    -- Advance row_phase using K6-driven tooth frequency
                    -- selector.  Map shifted toward longer legs: /4
                    -- gives 8× the leg height of the prior default,
                    -- with full tooth amp for the longest diagonal
                    -- line possible (apex narrows to ~28° at /4).
                    case to_integer(tooth_freq_sel_r) is
                        when 0|1   => v_advance := resize(
                                        shift_right(freq_row_r, 2), 21);
                        when 2|3   => v_advance := resize(
                                        shift_right(freq_row_r, 1), 21);
                        when 4|5   => v_advance := resize(freq_row_r, 21);
                        when others => v_advance := shift_left(
                                        resize(freq_row_r, 21), 1);
                    end case;
                    v_row_phase_sum := resize(row_phase_r, 22)
                                     + resize(v_advance, 22);
                    v_row_phase_17  := v_row_phase_sum(16 downto 0);
                    row_phase_r <= v_row_phase_17;

                    if v_row_phase_17(16) = '0' then
                        tooth_phase_r <= '0' & v_row_phase_17(15 downto 0);
                    else
                        tooth_phase_r <= '0' & (not v_row_phase_17(15 downto 0));
                    end if;
                else
                    -- Above horizon: reset wavelen and row_phase
                    wavelen_int_r  <= chev_min_int_r;
                    wavelen_frac_r <= (others => '0');
                    row_phase_r    <= (others => '0');
                    tooth_phase_r  <= (others => '0');
                end if;

                if chev_vee_r = '1' then
                    -- Vee mode: anchor stripe DDA at the user-set X
                    -- vanishing-point position (pot 4).
                    eff_horizon_r <= eff_horizon_x_r;
                else
                    -- Straight mode: no anchor (parallel stripes).
                    eff_horizon_r <= (others => '0');
                end if;
            end if;

            -- At vsync, restart wavelen / row_phase
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                wavelen_int_r  <= chev_min_int_r;
                wavelen_frac_r <= (others => '0');
                row_phase_r    <= (others => '0');
                tooth_phase_r  <= (others => '0');
            end if;

            ------------------------------------------------------------
            -- Always-on freq_row pipeline (settles within blanking):
            --   freq_a = LUT(wavelen_int)
            --   freq_b = LUT(wavelen_int+1)        (clamp at 127)
            --   freq_diff = freq_a - freq_b
            --   freq_row  = freq_a - (freq_diff * wavelen_frac) >> 8
            -- 4-stage pipeline keeps each Fmax-critical hop short.
            ------------------------------------------------------------
            freq_a_r <= C_FREQ_LUT(to_integer(wavelen_int_r));
            if wavelen_int_r >= to_unsigned(127, 7) then
                freq_b_r <= C_FREQ_LUT(127);
            else
                freq_b_r <= C_FREQ_LUT(to_integer(wavelen_int_r) + 1);
            end if;

            freq_diff_r  <= freq_a_r - freq_b_r;
            freq_a_d1_r  <= freq_a_r;
            freq_a_d2_r  <= freq_a_d1_r;

            freq_interp_r <= freq_diff_r * wavelen_frac_r;

            freq_row_r <= freq_a_d2_r - freq_interp_r(24 downto 8);

            -- Always-running 2-stage split multiplier: phase_init =
            --   (eff_horizon × freq) + tooth_phase + (Vee apex offset)
            -- mod 2^18.  Settles within a few blanking cycles.
            --
            -- Vee apex offset = 2^15 (= ¼-wavelen in phase units) is the
            -- midpoint between apex-on-stripe (causes the old 2× wide
            -- centre stripe) and apex-on-boundary (causes diamonds).
            -- The centre column stays inside an LO stripe so the pattern
            -- reads as chevrons, but the centre stripe is only modestly
            -- wider than the others (not 2×).
            phase_mul_hi_r <= eff_horizon_r(11 downto 6) * freq_row_r;
            phase_mul_lo_r <= eff_horizon_r(5 downto 0)  * freq_row_r;
            v_phase_hi_18 := shift_left(
                resize(phase_mul_hi_r(11 downto 0), 18), 6);
            v_phase_lo_18 := resize(phase_mul_lo_r(17 downto 0), 18);
            -- Tooth amplitude paired with K6 advance.  /4 (default)
            -- and /2 use full amp (cap, can't reach 90° with these
            -- long legs); ×1 hits 90° apex with full amp; ×2 needs
            -- half amp to keep the 90° shape at the denser period.
            case to_integer(tooth_freq_sel_r) is
                when 0|1   => v_tooth_scaled := resize(
                                tooth_phase_r, 18);                        -- full (/4 → 28°)
                when 2|3   => v_tooth_scaled := resize(
                                tooth_phase_r, 18);                        -- full (/2 → 53°)
                when 4|5   => v_tooth_scaled := resize(
                                tooth_phase_r, 18);                        -- full (×1 → 90°)
                when others => v_tooth_scaled := resize(
                                shift_right(tooth_phase_r, 1), 18);        -- half (×2 → 90°)
            end case;
            if chev_vee_r = '1' then
                phase_init_r <= v_phase_hi_18 + v_phase_lo_18
                              + shift_right(v_tooth_scaled, 1)
                              + to_unsigned(32768, 18);
            else
                phase_init_r <= v_phase_hi_18 + v_phase_lo_18
                              + v_tooth_scaled;
            end if;

            -- Per-pixel phase DDA (anchored at eff_horizon in Vee mode).
            -- Direction compare is pre-registered into stripe_back_pre_r
            -- to break the compare-then-add critical path.
            stripe_back_pre_r <= '1' when (chev_vee_r = '1'
                                  and pixel_x < eff_horizon_r) else '0';
            v_freq_ext := resize(freq_row_r, 18);
            if data_in.avid = '0' then
                phase_r <= phase_init_r;
            elsif stripe_back_pre_r = '1' then
                phase_r <= phase_r - v_freq_ext;
            else
                phase_r <= phase_r + v_freq_ext;
            end if;

            -- Delay stripe colour (= phase_r MSB) to align with s3 compose
            stripe_color_d1 <= phase_r(17);
            stripe_color_d2 <= stripe_color_d1;
            stripe_color_d3 <= stripe_color_d2;

            ------------------------------------------------------------
            -- Real-chevron renderer (Option 2) with perspective:
            --   chev_y_pers = y_acc(y) - |dx_phase(x)|
            -- where y_acc accumulates freq_row per row below horizon
            -- (= integral of inverse-wavelen) and dx_phase accumulates
            -- freq_row per pixel from eff_horizon_x_r.  Bands at every
            -- 2·2^17 chev units → smaller bands near horizon (large
            -- freq_row), larger at foreground (small freq_row), all
            -- with the same V apex angle in world coords.
            ------------------------------------------------------------
            -- Always-on split multiply: eff_horizon_x_r * freq_row_r
            dx_mul_hi_r <= eff_horizon_x_r(11 downto 6) * freq_row_r;
            dx_mul_lo_r <= eff_horizon_x_r(5 downto 0)  * freq_row_r;
            dx_phase_init_r <= shift_left(
                                 resize(dx_mul_hi_r(17 downto 0), 24), 6)
                             + resize(dx_mul_lo_r(22 downto 0), 24);

            -- Per-pixel dx_phase DDA.  Single vanishing point at
            -- eff_horizon_x_r (driven by K4 = Apex); dx_phase = 0
            -- there in steady state.  K5 = Scroll adds a constant
            -- per-frame phase offset so the pattern can be slid
            -- horizontally without moving the VP itself.
            --   Init  : dx_phase = -(eff_horizon_x · freq_row) + scroll
            --   Per px: += freq_row
            if data_in.avid = '0' then
                dx_phase_r <= -signed(resize(dx_phase_init_r, 26))
                            + signed(resize(scroll_offset_r, 26));
            else
                dx_phase_r <= dx_phase_r
                            + signed(resize(freq_row_r, 26));
            end if;

            -- Per-row y_acc update at hsync (rows below horizon only)
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                y_acc_r <= (others => '0');
            elsif data_in.hsync_n = '0' and prev_hsync_n = '1' then
                if pixel_y + 1 >= horizon_r then
                    y_acc_r <= y_acc_r + resize(freq_row_r, 24);
                else
                    y_acc_r <= (others => '0');
                end if;
            end if;

            -- Triangle wave from y_acc_r — perspective-correct accumulator
            -- (grows by freq_row per row).  Bit 18 selects rising/falling;
            -- period in y_acc = 2^19 → period in rows = 4·wavelen.  Amp
            -- 2^18 → swing = 2·wavelen px.  Both grow together so slope
            -- = 1 (90° apex) and leg_length = √2·2·wavelen ≈ 2.83× stripe
            -- width — about half the prior length.  All boundaries trace
            -- the same 45° diagonal in lockstep, producing many parallel
            -- jagged stripes with 90° corners at every swing.
            if y_acc_r(18) = '0' then
                v_tooth_pers := "00" & y_acc_r(17 downto 0);
            else
                v_tooth_pers := "00" & (not y_acc_r(17 downto 0));
            end if;

            -- chev_coord = dx_phase + tooth.  Stripes form at every
            -- 2^18 of chev_coord (one stripe pair = 2·wavelen px); the
            -- tooth shifts the boundary back-and-forth per row by up
            -- to 2^18 of phase = 2·wavelen px to create the zigzag.
            v_chev_x_pers := dx_phase_r
                           + signed(resize(v_tooth_pers, 26));

            -- 90°-rotated form (chev_dir_r='1').  Swap the roles of x
            -- and y in the chev coord: y_acc supplies the stripe phase
            -- (advances by freq_row per row → bit 17 toggles every
            -- wavelen rows), dx_phase supplies the tooth folded into a
            -- triangle wave with period 2^19 in dx_phase ≈ 4·wavelen px
            -- and amplitude 2^18 ≈ 2·wavelen of stripe offset → 90°
            -- apex angle, matching the unrotated case.
            if dx_phase_r(18) = '0' then
                v_tooth_x_pers := "00" & unsigned(dx_phase_r(17 downto 0));
            else
                v_tooth_x_pers := "00" & not unsigned(dx_phase_r(17 downto 0));
            end if;
            v_chev_y_pers := signed(resize(y_acc_r, 26))
                           + signed(resize(v_tooth_x_pers, 26));

            chev_x_color_d1 <= v_chev_x_pers(17);
            chev_y_color_d1 <= v_chev_y_pers(17);

            -- Mux between the two pre-registered colour bits.  Both
            -- inputs come from registers, so the mux's combinational
            -- depth is just one LUT — well clear of the critical path.
            if chev_dir_r = '1' then
                chev_color_d2 <= chev_y_color_d1;
            else
                chev_color_d2 <= chev_x_color_d1;
            end if;
            chev_color_d3 <= chev_color_d2;

            ------------------------------------------------------------
            -- Always-on soft-vignette pipeline (4 cycles)
            --   Stage A : edge distances (subtractions only)
            --   Stage B : per-axis minimum
            --   Stage C : overall min distance
            --   Stage D : fade level lookup (8 levels)
            -- Output fade_lvl_r aligns with s4_x (4-stage delay matches).
            ------------------------------------------------------------
            dist_l_r <= pixel_x;
            dist_r_r <= max_x_r - pixel_x;
            dist_t_r <= pixel_y;
            dist_b_r <= max_y_r - pixel_y;

            if dist_l_r < dist_r_r then
                dist_x_r <= dist_l_r;
            else
                dist_x_r <= dist_r_r;
            end if;
            if dist_t_r < dist_b_r then
                dist_y_r <= dist_t_r;
            else
                dist_y_r <= dist_b_r;
            end if;

            if dist_x_r < dist_y_r then
                min_dist_r <= dist_x_r;
            else
                min_dist_r <= dist_y_r;
            end if;

            -- Smooth fade across a 128-pixel zone.  Inside the zone:
            -- fade = min_dist (low 7 bits = 0..127).  Outside (or when
            -- vignette disabled): fade = 127 (≈ full Y, < 1% dimming).
            if vign_en_r = '0' then
                fade_lvl_r <= (others => '1');
            elsif min_dist_r >= to_unsigned(128, 12) then
                fade_lvl_r <= (others => '1');
            else
                fade_lvl_r <= min_dist_r(6 downto 0);
            end if;

            ------------------------------------------------------------
            -- Stage 1: per-pixel derived coords
            ------------------------------------------------------------
            s1_x <= pixel_x;
            s1_y <= pixel_y;

            -- Curtain fold phase = accumulator + pre-registered sway
            -- + pre-registered hash offset (3-input 16-bit add only).
            if sway_en_r = '1' then
                v_phase_with := fold_phase_r + pre_sway_r
                              + resize(pre_hash_r(7 downto 0), 16);
            else
                v_phase_with := fold_phase_r
                              + resize(pre_hash_r(7 downto 0), 16);
            end if;
            s1_phase <= v_phase_with;

            -- Curtain bottom = horizon + per-fold hem so each fold has a
            -- rounded bottom that hangs into the floor area.  Hem
            -- amplitude is 0..12 px, indexed by the fold position via
            -- top bits of fold_phase_r (parabolic LUT, peak in middle of
            -- each fold, zero at fold seams).
            -- Look up base hem (0..12 px) then scale down for narrow
            -- folds so tight pointy ends don't get an oversized shadow.
            v_hem_raw := C_HEM_LUT(to_integer(fold_phase_r(15 downto 10)));
            case to_integer(hem_shift_r) is
                when 0      => v_hem := v_hem_raw;
                when 1      => v_hem := shift_right(v_hem_raw, 1);
                when 2      => v_hem := shift_right(v_hem_raw, 2);
                when others => v_hem := shift_right(v_hem_raw, 3);
            end case;
            v_curtain_bot := horizon_r + resize(v_hem, 12);

            if pixel_y >= v_curtain_bot then
                s1_is_floor <= '1';
                -- Drop-shadow strength: distance from curtain edge gives
                -- 15..0 falloff over 16 rows (15 at the hem, fading to 0).
                v_shadow_d := pixel_y - v_curtain_bot;
                if v_shadow_d < to_unsigned(16, 12) then
                    s1_shadow_amt <= to_unsigned(15, 4) - v_shadow_d(3 downto 0);
                else
                    s1_shadow_amt <= (others => '0');
                end if;
            else
                s1_is_floor <= '0';
                s1_shadow_amt <= (others => '0');
            end if;

            -- Floor depth (used by chevron DDA) is measured from the
            -- main horizon line, not the curved curtain hem, so the
            -- chevron pattern stays consistent across the floor.
            if pixel_y >= horizon_r then
                v_depth := pixel_y - horizon_r;
            else
                v_depth := horizon_r - pixel_y;
            end if;
            s1_depth <= v_depth;

            ------------------------------------------------------------
            -- Stage 2: triangle wave from s1_phase (smooth 10-bit output)
            -- phase(15) selects rising/falling half; phase(14..5) is the
            -- 10-bit amplitude.  Period = 2^16 / fold_freq pixels — set
            -- continuously by pot 1.
            ------------------------------------------------------------
            s2_x <= s1_x;
            s2_y <= s1_y;
            s2_is_floor <= s1_is_floor;
            s2_shadow_amt <= s1_shadow_amt;

            if s1_phase(15) = '0' then
                v_tri_a := s1_phase(14 downto 5);
            else
                v_tri_a := not s1_phase(14 downto 5);
            end if;
            s2_tri_a <= v_tri_a;

            ------------------------------------------------------------
            -- Stage 3: fold_luma (smooth triangle straight through)
            ------------------------------------------------------------
            s3_x <= s2_x;
            s3_y <= s2_y;
            s3_is_floor <= s2_is_floor;
            s3_shadow_amt <= s2_shadow_amt;

            v_fold_luma := s2_tri_a;
            s3_fold_luma <= v_fold_luma;

            -- Pre-compute the depth-shifted curtain blend factor here so
            -- s4's critical path is only the lerp multiplier (+ add).
            -- Truncate to 6 bits (64 levels) — keeps the s4 multiplier
            -- small while still smooth across a depth-pot sweep.
            -- depth_off_s_r is precomputed at vsync (= depth - 512).
            v_blend_signed := signed(resize(v_fold_luma, 12))
                            + depth_off_s_r;
            if v_blend_signed < 0 then
                s3_blend_factor <= (others => '0');
            elsif v_blend_signed > to_signed(1023, 12) then
                s3_blend_factor <= to_unsigned(63, 6);
            else
                s3_blend_factor <= unsigned(v_blend_signed(9 downto 4));
            end if;

            ------------------------------------------------------------
            -- Stage 4: colour compose (both layers in parallel)
            ------------------------------------------------------------
            s4_x <= s3_x;
            s4_y <= s3_y;
            s4_is_floor <= s3_is_floor;

            -- Curtain: lerp between LO and HI via tiny constant LUTs
            -- indexed by s3_blend_factor (6-bit).  Replaces three runtime
            -- multipliers with a fixed mux — much faster combinational.
            s4_cur_y <= C_CURTAIN_LO_Y
                      + resize(C_LERP_Y_LUT(to_integer(s3_blend_factor)),10);
            s4_cur_u <= C_CURTAIN_LO_U
                      + resize(C_LERP_U_LUT(to_integer(s3_blend_factor)),10);
            s4_cur_v <= C_CURTAIN_LO_V
                      - resize(C_LERP_V_LUT(to_integer(s3_blend_factor)),10);

            -- Floor: select cream vs dark chevron from the rotated
            -- Option-2 renderer (chev_color_d3).  Vertical zigzag
            -- stripes radiating from the horizon point — perspective
            -- via wavelen DDA, zigzag via tooth(y_acc) added to
            -- dx_phase.  Stripe wavelen widens with depth.
            if chev_color_d3 = '1' then
                v_flr_y_var := C_FLOOR_LT_Y;
                s4_flr_u <= C_FLOOR_LT_U;
                s4_flr_v <= C_FLOOR_LT_V;
            else
                v_flr_y_var := dk_y_r;
                s4_flr_u <= dk_u_r;
                s4_flr_v <= dk_v_r;
            end if;
            -- Drop-shadow under the curtain hem: attenuate Y based on
            -- shadow_amt (15 = right under the curtain, 0 = no shadow).
            -- 4 levels mapped from the top 2 bits of shadow_amt.
            case s3_shadow_amt(3 downto 2) is
                when "11" => v_flr_y_var := shift_right(v_flr_y_var, 1);
                when "10" => v_flr_y_var := v_flr_y_var
                                          - shift_right(v_flr_y_var, 2);
                when "01" => v_flr_y_var := v_flr_y_var
                                          - shift_right(v_flr_y_var, 3);
                when others => null;  -- "00" → no shadow
            end case;
            s4_flr_y <= v_flr_y_var;

            -- Corner vignette check: flag if in outer border
            -- (Soft-vignette fade_lvl is computed by the always-on 4-stage
            --  pipeline below; no per-pixel work here.)

            ------------------------------------------------------------
            -- Stage 5: layer select + smooth vignette
            ------------------------------------------------------------
            if s4_is_floor = '1' then
                v_y_out := s4_flr_y;
                v_u_out := s4_flr_u;
                v_v_out := s4_flr_v;
            else
                v_y_out := s4_cur_y;
                v_u_out := s4_cur_u;
                v_v_out := s4_cur_v;
            end if;

            -- Soft vignette: Y_faded = Y * fade_lvl / 128.  10x7
            -- multiply gives a 17-bit result; the top 10 bits are the
            -- attenuated Y.  At fade=127 this is Y * 127/128 (~0.78%
            -- dimming, invisible).  At fade=0 the corner is black.
            v_fade_mul := v_y_out * fade_lvl_r;
            v_y_out := v_fade_mul(16 downto 7);

            -- Snap chroma toward neutral when the corner is very dark
            -- so a stray Cr/Cb tint can't colour the near-black pixels.
            if fade_lvl_r < to_unsigned(8, 7) then
                v_u_out := C_CHROMA_MID;
                v_v_out := C_CHROMA_MID;
            end if;

            s5_y <= v_y_out;
            s5_u <= v_u_out;
            s5_v <= v_v_out;
            s5_is_floor <= s4_is_floor;

            ------------------------------------------------------------
            -- Stage 6: pass-through register (brightness control was
            -- removed when K5 was repurposed as Scroll).  Kept as a
            -- register stage so key_floor_r aligns with s6_y at the
            -- output mux.
            ------------------------------------------------------------
            key_floor_r <= key_en_r and s5_is_floor;
            s6_y <= s5_y;
            s6_u <= s5_u;
            s6_v <= s5_v;

        end if;
    end process;

    ------------------------------------------------------------
    -- Output: T11 = Key swaps the floor region (chevron area) for
    -- the delayed input video while leaving curtain pixels on the
    -- synth output.  Sync signals always come from the pipe so the
    -- timing aligns with the s6 colour stream.
    ------------------------------------------------------------
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;
    data_out.avid    <= pipe(LATENCY - 1).avid;

    data_out.y <= pipe(LATENCY - 1).y when key_floor_r = '1'
                  else std_logic_vector(s6_y);
    data_out.u <= pipe(LATENCY - 1).u when key_floor_r = '1'
                  else std_logic_vector(s6_u);
    data_out.v <= pipe(LATENCY - 1).v when key_floor_r = '1'
                  else std_logic_vector(s6_v);

end architecture blacklodge;
