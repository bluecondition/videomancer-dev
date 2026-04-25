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
--   registers_in(3) = Perspect  (compression rate toward horizon)
--   registers_in(4) = Horizon   (vertical horizon position 0..1)
--   registers_in(5) = Warmth    (0..1023, shifts U toward red)
--   registers_in(6) = Switches  (bit0=Sway bit1=ChevVee bit2=Grain
--                                bit3=Vignette bit4=Bypass)
--   registers_in(7) = Brightness (top 3 bits: 1/8 .. 8/8 gain)
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

    constant C_CHROMA_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    signal folds_idx_r    : unsigned(1 downto 0)  := "01";
    signal depth_amt_r    : unsigned(9 downto 0)  := to_unsigned(700, 10);
    signal warmth_r       : unsigned(9 downto 0)  := to_unsigned(512, 10);
    signal bright_sel_r   : unsigned(2 downto 0)  := "111";

    signal sway_en_r      : std_logic := '1';
    signal chev_vee_r     : std_logic := '0';
    signal grain_en_r     : std_logic := '1';
    signal vign_en_r      : std_logic := '1';
    signal bypass_r       : std_logic := '0';

    -- Per-frame sway offset (triangle of frame_count)
    signal sway_off_r     : signed(8 downto 0) := (others => '0');

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
    signal step_r         : unsigned(7 downto 0) := to_unsigned(64, 8);

    -- Per-row wavelength state (fractional 7.8 fixed-point)
    signal wavelen_int_r  : unsigned(6 downto 0) := to_unsigned(10, 7);
    signal wavelen_frac_r : unsigned(7 downto 0) := (others => '0');

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

    -- Align stripe_color (= phase_r(17)) with the 3-stage compose pipeline
    signal stripe_color_d1 : std_logic := '0';
    signal stripe_color_d2 : std_logic := '0';
    signal stripe_color_d3 : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 1: pre-compute fold_x, depth, is_floor
    ----------------------------------------------------------------------
    signal s1_x         : unsigned(11 downto 0) := (others => '0');
    signal s1_y         : unsigned(11 downto 0) := (others => '0');
    signal s1_fold_x    : unsigned(11 downto 0) := (others => '0');
    signal s1_depth     : unsigned(11 downto 0) := (others => '0');
    signal s1_is_floor  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 2: curtain fold triangles
    ----------------------------------------------------------------------
    signal s2_x         : unsigned(11 downto 0) := (others => '0');
    signal s2_y         : unsigned(11 downto 0) := (others => '0');
    signal s2_is_floor  : std_logic := '0';
    signal s2_tri_a     : unsigned(9 downto 0) := (others => '0');
    signal s2_tri_b     : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Stage 3: fold_luma (curtain path)
    ----------------------------------------------------------------------
    signal s3_x         : unsigned(11 downto 0) := (others => '0');
    signal s3_y         : unsigned(11 downto 0) := (others => '0');
    signal s3_is_floor  : std_logic := '0';
    signal s3_fold_luma : unsigned(9 downto 0) := (others => '0');

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
    -- fade_lvl_r 7 = full, 0 = full black at outer edge.
    signal dist_l_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_r_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_t_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_b_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_x_r     : unsigned(11 downto 0) := (others => '0');
    signal dist_y_r     : unsigned(11 downto 0) := (others => '0');
    signal min_dist_r   : unsigned(11 downto 0) := (others => '0');
    signal fade_lvl_r   : unsigned(2 downto 0) := "111";

    ----------------------------------------------------------------------
    -- Stage 5: layer select + vignette + warmth
    ----------------------------------------------------------------------
    signal s5_y : unsigned(9 downto 0) := (others => '0');
    signal s5_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s5_v : unsigned(9 downto 0) := C_CHROMA_MID;

    ----------------------------------------------------------------------
    -- Stage 6: brightness scale (3-bit selector, case shift)
    ----------------------------------------------------------------------
    signal s6_y : unsigned(9 downto 0) := (others => '0');
    signal s6_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s6_v : unsigned(9 downto 0) := C_CHROMA_MID;

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
        variable v_warmth_off   : unsigned(5 downto 0);
        variable v_u_sum        : unsigned(10 downto 0);
        variable v_vgrad        : unsigned(7 downto 0);
        variable v_cur_hi_y     : unsigned(9 downto 0);
        variable v_tri_bit_idx  : integer range 0 to 6;
        variable v_inv_fold     : unsigned(9 downto 0);
        variable v_if5          : unsigned(4 downto 0);
        variable v_dp5          : unsigned(4 downto 0);
        variable v_prod10       : unsigned(9 downto 0);
        variable v_horz_slv     : std_logic_vector(9 downto 0);
        variable v_horz_top4    : unsigned(3 downto 0);
        variable v_horz_mul     : unsigned(15 downto 0);
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
        variable v_row_phase_sum: unsigned(17 downto 0);
        variable v_row_phase_17 : unsigned(16 downto 0);
        variable v_dist_x       : unsigned(11 downto 0);
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

                -- Horizon derived from pot 4 top 4 bits: horizon =
                -- max_y * top4 / 16 (12x4 multiply, runs once per frame).
                v_horz_slv  := registers_in(4);
                v_horz_top4 := unsigned(v_horz_slv(9 downto 6));
                v_horz_mul  := resize(max_y_r * v_horz_top4, 16);
                v_horizon_v := v_horz_mul(15 downto 4);
                horizon_r   <= v_horizon_v;

                -- Chevron: minimum wavelength at horizon.  Range starts
                -- where the previous build's 50% setting sat (chev_min=18)
                -- and runs up to 33 pixels at full pot.
                chev_min_int_r <= resize(unsigned(
                                    registers_in(2)(9 downto 6)), 7)
                                + to_unsigned(18, 7);

                -- Perspective: per-row fractional wavelength step.  Pot 4
                -- top 5 bits maps to step (0..31).  At step=31 and
                -- chev_min=18, wavelen grows ~18→83 across a 540-row
                -- floor — strong perspective without ever hitting the
                -- 127-pixel LUT clamp (so the bottom never goes flat).
                step_r <= resize(unsigned(registers_in(3)(9 downto 5)), 8);

                folds_idx_r  <= unsigned(registers_in(0)(9 downto 8));
                depth_amt_r  <= unsigned(registers_in(1));
                warmth_r     <= unsigned(registers_in(5));
                bright_sel_r <= unsigned(registers_in(7)(9 downto 7));

                sway_en_r    <= registers_in(6)(0);
                chev_vee_r   <= registers_in(6)(1);
                grain_en_r   <= registers_in(6)(2);
                vign_en_r    <= registers_in(6)(3);
                bypass_r     <= registers_in(6)(4);

                -- Per-frame sway: small triangle of frame_count top bits
                if frame_count(9) = '0' then
                    sway_off_r <= signed(resize(frame_count(8 downto 2), 9));
                else
                    sway_off_r <= signed(resize(
                        not frame_count(8 downto 2), 9));
                end if;
            end if;

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

            -- Per-hsync updates
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                if pixel_y + 1 >= horizon_r then
                    -- Advance wavelen_frac (8-bit), carry into wavelen_int
                    if ('0' & wavelen_frac_r) + ('0' & step_r)
                            >= to_unsigned(256, 9) then
                        wavelen_frac_r <= wavelen_frac_r + step_r;  -- wraps
                        if wavelen_int_r < to_unsigned(127, 7) then
                            wavelen_int_r <= wavelen_int_r + 1;
                        end if;
                    else
                        wavelen_frac_r <= wavelen_frac_r + step_r;
                    end if;

                    -- Advance row_phase by current freq_row (wraps at 2^17
                    -- exactly every wavelen rows → 45° tooth apex slope)
                    v_row_phase_sum := ('0' & row_phase_r) + ('0' & freq_row_r);
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
                    eff_horizon_r <= horizon_r;
                else
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
            -- (eff_horizon * freq + tooth_phase) mod 2^18.  Settles within
            -- a few blanking cycles, so phase_r is preloaded before avid.
            phase_mul_hi_r <= eff_horizon_r(11 downto 6) * freq_row_r;
            phase_mul_lo_r <= eff_horizon_r(5 downto 0)  * freq_row_r;
            -- Stage 2: combine partials + tooth_phase (18-bit wraparound)
            v_phase_hi_18 := shift_left(
                resize(phase_mul_hi_r(11 downto 0), 18), 6);
            v_phase_lo_18 := resize(phase_mul_lo_r(17 downto 0), 18);
            phase_init_r  <= v_phase_hi_18 + v_phase_lo_18
                           + resize(tooth_phase_r, 18);

            -- Per-pixel phase DDA (anchored at horizon_r in Vee mode)
            v_freq_ext := resize(freq_row_r, 18);
            if data_in.avid = '0' then
                phase_r <= phase_init_r;
            elsif chev_vee_r = '1' and pixel_x < eff_horizon_r then
                phase_r <= phase_r - v_freq_ext;
            else
                phase_r <= phase_r + v_freq_ext;
            end if;

            -- Delay stripe colour (= phase_r MSB) to align with s3 compose
            stripe_color_d1 <= phase_r(17);
            stripe_color_d2 <= stripe_color_d1;
            stripe_color_d3 <= stripe_color_d2;

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

            if vign_en_r = '0' then
                fade_lvl_r <= "111";
            elsif min_dist_r >= to_unsigned(224, 12) then
                fade_lvl_r <= "111";
            elsif min_dist_r >= to_unsigned(160, 12) then
                fade_lvl_r <= "110";
            elsif min_dist_r >= to_unsigned(112, 12) then
                fade_lvl_r <= "101";
            elsif min_dist_r >= to_unsigned( 72, 12) then
                fade_lvl_r <= "100";
            elsif min_dist_r >= to_unsigned( 44, 12) then
                fade_lvl_r <= "011";
            elsif min_dist_r >= to_unsigned( 24, 12) then
                fade_lvl_r <= "010";
            elsif min_dist_r >= to_unsigned(  8, 12) then
                fade_lvl_r <= "001";
            else
                fade_lvl_r <= "000";
            end if;

            ------------------------------------------------------------
            -- Stage 1: per-pixel derived coords
            ------------------------------------------------------------
            s1_x <= pixel_x;
            s1_y <= pixel_y;

            -- Curtain fold_x: optional sway plus per-band hash jitter so
            -- the fold pattern is subtly irregular (real velvet curtains
            -- never have perfectly periodic folds).  hash_band updates
            -- every 32 pixels giving ~7 different jitter values across a
            -- HD frame width.
            if sway_en_r = '1' then
                v_fold_x := pixel_x + unsigned(resize(sway_off_r, 12));
            else
                v_fold_x := pixel_x;
            end if;
            -- Hash 5-bit band index → 0..7 pixel offset added per band
            v_grain := hash16(
                resize(pixel_x(11 downto 5), 16),
                to_unsigned(16#7C2D#, 16));
            v_fold_x := v_fold_x + resize(v_grain(2 downto 0), 12);
            s1_fold_x <= v_fold_x;

            -- Depth below horizon (0 above, positive below)
            if pixel_y >= horizon_r then
                v_depth := pixel_y - horizon_r;
                s1_is_floor <= '1';
            else
                v_depth := horizon_r - pixel_y;
                s1_is_floor <= '0';
            end if;
            s1_depth <= v_depth;

            ------------------------------------------------------------
            -- Stage 2: curtain fold triangles (floor now handled by DDA)
            ------------------------------------------------------------
            s2_x <= s1_x;
            s2_y <= s1_y;
            s2_is_floor <= s1_is_floor;

            -- Triangle A: period selected by folds_idx (wider period default)
            --   folds_idx 0: period 128 (7-bit tri) -- sparse wide folds
            --   folds_idx 1: period  64 (6-bit tri)
            --   folds_idx 2: period  32 (5-bit tri)
            --   folds_idx 3: period  16 (4-bit tri) -- dense narrow folds
            case to_integer(folds_idx_r) is
                when 0      => v_tri_a := triwave10(s1_fold_x(6 downto 0), 7);
                when 1      => v_tri_a := triwave10(s1_fold_x(5 downto 0), 6);
                when 2      => v_tri_a := triwave10(s1_fold_x(4 downto 0), 5);
                when others => v_tri_a := triwave10(s1_fold_x(3 downto 0), 4);
            end case;
            s2_tri_a <= v_tri_a;

            -- Triangle B: always one octave slower — breaks uniformity
            case to_integer(folds_idx_r) is
                when 0      => v_tri_b := triwave10(s1_fold_x(7 downto 1), 7);
                when 1      => v_tri_b := triwave10(s1_fold_x(6 downto 1), 6);
                when 2      => v_tri_b := triwave10(s1_fold_x(5 downto 1), 5);
                when others => v_tri_b := triwave10(s1_fold_x(4 downto 1), 4);
            end case;
            s2_tri_b <= v_tri_b;

            ------------------------------------------------------------
            -- Stage 3: fold_luma (sum tris + hash tweak)
            ------------------------------------------------------------
            s3_x <= s2_x;
            s3_y <= s2_y;
            s3_is_floor <= s2_is_floor;

            -- Sum the two triangles then halve → 10-bit fold luma profile
            v_fold_sum  := ('0' & s2_tri_a) + ('0' & s2_tri_b);
            v_fold_luma := v_fold_sum(10 downto 1);
            -- Optional grain: xor low 3 bits with hash for dither/texture
            if grain_en_r = '1' then
                v_grain := hash16(resize(s2_x, 16), resize(s2_y, 16));
                v_fold_luma(3 downto 1) := v_fold_luma(3 downto 1)
                                         xor v_grain(2 downto 0);
            end if;
            s3_fold_luma <= v_fold_luma;

            ------------------------------------------------------------
            -- Stage 4: colour compose (both layers in parallel)
            ------------------------------------------------------------
            s4_x <= s3_x;
            s4_y <= s3_y;
            s4_is_floor <= s3_is_floor;

            -- Curtain: interpolate highlight/shadow by (1-depth*fold_luma)
            -- Higher fold_luma → closer to highlight; Lower → closer to shadow.
            -- Depth pot attenuates the dark side (more depth = deeper troughs).
            --
            -- Simple lerp: cur_y = lo + ((hi - lo) * fold_luma) >> 10
            -- But no multiplier — substitute: dark_amt = ((hi - lo) *
            -- (1023 - fold_luma) * depth_amt) >> 20.
            -- Cheap approximation: use two shifts.
            --
            -- y_diff = hi - lo
            v_cur_hi_y  := C_CURTAIN_HI_Y;
            -- Vertical gradient: brighter near top of curtain — subtract
            -- pixel_y(9 downto 4) from highlight (max dim ~ 63 counts).
            if s3_y < to_unsigned(1024, 12) then
                v_vgrad := resize(s3_y(9 downto 4), 8);
                if v_cur_hi_y > resize(v_vgrad, 10) then
                    v_cur_hi_y := v_cur_hi_y - resize(v_vgrad, 10);
                end if;
            end if;

            -- Fold shadow: darken highlight by (1023 - fold_luma) weighted
            -- by depth_amt.  Use top 5 bits of fold_luma complement and
            -- top 5 bits of depth for a 10-bit dark_amt — purely shifted,
            -- no multiplier.
            --
            -- dark_amt = ((1023 - fold_luma)[9..5] * depth_amt[9..5])
            -- Small 5x5 = 10-bit product (HX4K: synthesises as LUTs, cheap).
            v_inv_fold    := to_unsigned(1023, 10) - s3_fold_luma;
            v_if5         := v_inv_fold(9 downto 5);
            v_dp5         := depth_amt_r(9 downto 5);
            v_prod10      := v_if5 * v_dp5;
            v_dark_scaled := v_prod10;

            -- cur_y = highlight_y - dark_scaled (clamped)
            if v_cur_hi_y > v_dark_scaled then
                s4_cur_y <= v_cur_hi_y - v_dark_scaled;
            else
                s4_cur_y <= (others => '0');
            end if;

            -- Curtain chroma: blend highlight and shadow chroma by fold_luma
            -- Top-bit decision is enough for a stylised look.
            if s3_fold_luma(9) = '1' then
                s4_cur_u <= C_CURTAIN_HI_U;
                s4_cur_v <= C_CURTAIN_HI_V;
            else
                -- Midtone: average
                s4_cur_u <= resize(shift_right(
                    resize(C_CURTAIN_HI_U, 11)
                    + resize(C_CURTAIN_LO_U, 11), 1), 10);
                s4_cur_v <= resize(shift_right(
                    resize(C_CURTAIN_HI_V, 11)
                    + resize(C_CURTAIN_LO_V, 11), 1), 10);
            end if;

            -- Floor: select cream vs dark chevron (DDA-delayed color)
            if stripe_color_d3 = '1' then
                s4_flr_y <= C_FLOOR_LT_Y;
                s4_flr_u <= C_FLOOR_LT_U;
                s4_flr_v <= C_FLOOR_LT_V;
            else
                s4_flr_y <= C_FLOOR_DK_Y;
                s4_flr_u <= C_FLOOR_DK_U;
                s4_flr_v <= C_FLOOR_DK_V;
            end if;

            -- Corner vignette check: flag if in outer border
            -- (Soft-vignette fade_lvl is computed by the always-on 4-stage
            --  pipeline below; no per-pixel work here.)

            ------------------------------------------------------------
            -- Stage 5: layer select + vignette + warmth tint
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

            -- Soft vignette Y attenuation (fade_lvl_r from always-on
            -- pipeline above).  At level 0 chroma snaps to neutral so the
            -- corner is true black.
            case to_integer(fade_lvl_r) is
                when 0 =>
                    v_y_out := (others => '0');
                    v_u_out := C_CHROMA_MID;
                    v_v_out := C_CHROMA_MID;
                when 1 => v_y_out := shift_right(v_y_out, 4);
                when 2 => v_y_out := shift_right(v_y_out, 3);
                when 3 => v_y_out := shift_right(v_y_out, 2);
                when 4 => v_y_out := shift_right(v_y_out, 1);
                when 5 => v_y_out := shift_right(v_y_out, 1)
                                   + shift_right(v_y_out, 3);
                when 6 => v_y_out := v_y_out - shift_right(v_y_out, 3);
                when others => null;  -- 7: full Y
            end case;

            -- Warmth: shift U (= Cr in Videomancer swapped convention)
            -- upward when warmth_r > mid (more red).
            -- warmth_off = (warmth_r - 512)[9..4] clamped to 0..31.
            if warmth_r > C_CHROMA_MID then
                v_warmth_off := resize(
                    shift_right(warmth_r - C_CHROMA_MID, 4), 6);
                v_u_sum := ('0' & v_u_out) + resize(v_warmth_off, 11);
                if v_u_sum > to_unsigned(1023, 11) then
                    v_u_out := to_unsigned(1023, 10);
                else
                    v_u_out := v_u_sum(9 downto 0);
                end if;
            elsif warmth_r < C_CHROMA_MID then
                v_warmth_off := resize(
                    shift_right(C_CHROMA_MID - warmth_r, 4), 6);
                if v_u_out > resize(v_warmth_off, 10) then
                    v_u_out := v_u_out - resize(v_warmth_off, 10);
                else
                    v_u_out := (others => '0');
                end if;
            end if;

            s5_y <= v_y_out;
            s5_u <= v_u_out;
            s5_v <= v_v_out;

            ------------------------------------------------------------
            -- Stage 6: brightness scale (top 3 bits of pot 12)
            --   000 = 1/8    011 = 1/2    111 = 8/8 (full)
            -- Cheap shift-add, no multiplier.
            ------------------------------------------------------------
            case to_integer(bright_sel_r) is
                when 0      => s6_y <= shift_right(s5_y, 3);
                when 1      => s6_y <= shift_right(s5_y, 2);
                when 2      => s6_y <= shift_right(s5_y, 2)
                                     + shift_right(s5_y, 3);
                when 3      => s6_y <= shift_right(s5_y, 1);
                when 4      => s6_y <= shift_right(s5_y, 1)
                                     + shift_right(s5_y, 3);
                when 5      => s6_y <= shift_right(s5_y, 1)
                                     + shift_right(s5_y, 2);
                when 6      => s6_y <= shift_right(s5_y, 1)
                                     + shift_right(s5_y, 2)
                                     + shift_right(s5_y, 3);
                when others => s6_y <= s5_y;
            end case;
            -- Chroma passes through unchanged (brightness shouldn't desaturate)
            s6_u <= s5_u;
            s6_v <= s5_v;

        end if;
    end process;

    ------------------------------------------------------------
    -- Output: bypass mux selects delayed input or generated pixel
    ------------------------------------------------------------
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;
    data_out.avid    <= pipe(LATENCY - 1).avid;

    data_out.y <= pipe(LATENCY - 1).y when bypass_r = '1'
                  else std_logic_vector(s6_y);
    data_out.u <= pipe(LATENCY - 1).u when bypass_r = '1'
                  else std_logic_vector(s6_u);
    data_out.v <= pipe(LATENCY - 1).v when bypass_r = '1'
                  else std_logic_vector(s6_v);

end architecture blacklodge;
