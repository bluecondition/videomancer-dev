-- Oracle: Glitched Divinatory Portal
--
-- A three-zone triptych effect:
--   * Top strip  : high-frequency vertical rainbow stripes (shimmering)
--   * Middle     : a torn "window" of noise containing a tall elliptical
--                  portal whose interior reveals the rainbow spectrum
--   * Bottom     : slow horizontal rainbow bands (ROYGBIV)
--
-- Zone seams are optionally "torn" (jittered per-row) for a datamoshed
-- look. The portal shape is an octagonal approximation of a 3:1 tall
-- ellipse, which costs only add + compare + mux — no squared-distance
-- multiplies. All other multiplies are split across registered stages
-- so the combinational path between any two flops is short enough to
-- close timing at 74.25 MHz on the iCE40-HX4K (no DSP blocks).
--
-- Register map (t_spi_ram, each std_logic_vector(9 downto 0)):
--   registers_in(0) = Portal Size (0..1023 -> ellipse radius scale)
--   registers_in(1) = Portal X    (0..1023 -> X center, linear across max_x)
--   registers_in(2) = Portal Y    (0..1023 -> Y center, linear across max_y)
--   registers_in(3) = Noise       (0..1023 -> hash density / variety)
--   registers_in(4) = Hue Speed   (0..1023 -> hue DDS rate)
--   registers_in(5) = Band Freq   (0..1023 -> stripe/band multiplier)
--   registers_in(6) = Switches    (b0 Top  b1 Bot  b2 Portal  b3 Tear  b4 Bypass)
--   registers_in(7) = Brightness  (0..1023 -> Y/chroma attenuator)
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

architecture oracle of program_top is

    ----------------------------------------------------------------------
    -- Pipeline depth
    ----------------------------------------------------------------------
    constant LATENCY : natural := 14;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    ----------------------------------------------------------------------
    -- Rainbow palette (ROYGBIV + wrap) in 10-bit YUV (U/V-swapped,
    -- calibrated). Copied verbatim from Phosphor's verified constants.
    ----------------------------------------------------------------------
    type t_pal10 is array (0 to 7) of unsigned(9 downto 0);

    constant C_PAL_Y : t_pal10 := (
        to_unsigned(328, 10),   -- 0 Red
        to_unsigned(584, 10),   -- 1 Orange
        to_unsigned(840, 10),   -- 2 Yellow
        to_unsigned(580, 10),   -- 3 Green
        to_unsigned(164, 10),   -- 4 Blue
        to_unsigned(192, 10),   -- 5 Indigo
        to_unsigned(349, 10),   -- 6 Violet
        to_unsigned(328, 10));  -- 7 Red (wrap)

    constant C_PAL_U : t_pal10 := (
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10),
        to_unsigned(136, 10), to_unsigned(440, 10), to_unsigned(608, 10),
        to_unsigned(756, 10), to_unsigned(960, 10));

    constant C_PAL_V : t_pal10 := (
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10),
        to_unsigned(216, 10), to_unsigned(960, 10), to_unsigned(696, 10),
        to_unsigned(852, 10), to_unsigned(360, 10));

    -- Neutral chroma for grayscale noise
    constant C_NEUTRAL_U : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_NEUTRAL_V : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- Position tracking
    ----------------------------------------------------------------------
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    -- last_x/y_r track the highest pixel_x/y observed in the current frame.
    -- Initialising to a small value means the first output frame uses stale
    -- zero-derived zone bounds, but the tester warmup skips those frames.
    signal last_x_r     : unsigned(11 downto 0) := (others => '0');
    signal last_y_r     : unsigned(11 downto 0) := (others => '0');
    signal max_x_r      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal max_y_r      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');
    signal hue_phase    : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Parameters latched at vsync (keeps timing off the user-control path)
    ----------------------------------------------------------------------
    signal portal_size_r : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal portal_x_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal portal_y_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal noise_k_r     : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal hue_speed_r   : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal band_k_r      : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal top_en_r      : std_logic := '1';
    signal bot_en_r      : std_logic := '1';
    signal portal_en_r   : std_logic := '1';
    signal tear_en_r     : std_logic := '1';
    signal bypass_r      : std_logic := '0';
    signal brt_r         : unsigned(9 downto 0) := to_unsigned(900, 10);

    ----------------------------------------------------------------------
    -- Per-frame derived constants (pipelined across vblank so the
    -- multipliers only need to close at a single pipeline stage).
    ----------------------------------------------------------------------
    -- cx = max_x * portal_x_r / 1024, cy = max_y * portal_y_r / 1024
    signal cx_mul_r       : unsigned(21 downto 0) := (others => '0');
    signal cy_mul_r       : unsigned(21 downto 0) := (others => '0');
    signal cx_r           : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal cy_r           : unsigned(11 downto 0) := to_unsigned(540, 12);

    -- Per-frame band-frequency factors (extracted from band_k_r).
    --   band_A_r: 4..67, picks vertical-stripe density in Zone A (many bars)
    --   band_C_r: 1..16, picks horizontal-band density in Zone C (few bars)
    signal band_A_r       : unsigned(6 downto 0) := to_unsigned(20, 7);
    signal band_C_r       : unsigned(4 downto 0) := to_unsigned(6, 5);

    -- Portal radius (horizontal half-width), driven by portal_size_r.
    -- Scaled so at size=1023, rx ≈ max_x/4 (vertical reach ≈ max_y*3/4 via 3:1 aspect).
    signal rx_mul_r       : unsigned(21 downto 0) := (others => '0');
    signal portal_rx_r    : unsigned(11 downto 0) := to_unsigned(200, 12);

    -- Zone seam y-positions: zoneA = [0, max_y>>3], zoneC = [max_y*7/8, max_y].
    signal zoneA_y_r      : unsigned(11 downto 0) := to_unsigned(135, 12);
    signal zoneC_y_r      : unsigned(11 downto 0) := to_unsigned(945, 12);

    -- Noise shift: maps noise_k_r to a right-shift amount 1..4
    -- (selects grain size).
    signal noise_shift_r  : integer range 0 to 5 := 3;

    -- Hue phase offset for animation (added to hue_C_idx so bands drift).
    signal hue_offset_r   : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Stage 1 signals: position register + coarse zone classification
    -- + portal absolute diffs + hash raw material.
    ----------------------------------------------------------------------
    signal s1_x           : unsigned(11 downto 0) := (others => '0');
    signal s1_y           : unsigned(11 downto 0) := (others => '0');
    signal s1_zone_A      : std_logic := '0';        -- strict zone-A (top)
    signal s1_zone_C      : std_logic := '0';        -- strict zone-C (bottom)
    signal s1_near_topA   : std_logic := '0';        -- y near A/B seam
    signal s1_near_botC   : std_logic := '0';        -- y near B/C seam
    signal s1_dx          : unsigned(11 downto 0) := (others => '0');
    signal s1_dy          : unsigned(11 downto 0) := (others => '0');
    signal s1_hue_base_A  : unsigned(9 downto 0)  := (others => '0');  -- x-only component
    signal s1_hue_base_C  : unsigned(9 downto 0)  := (others => '0');  -- y-only component
    signal s1_noise_xi    : unsigned(11 downto 0) := (others => '0');  -- x >> shift
    signal s1_noise_yi    : unsigned(11 downto 0) := (others => '0');  -- y >> shift

    ----------------------------------------------------------------------
    -- Stage 2 signals: hue indices including animation & per-row tear,
    -- scaled dx for octagonal distance, noise hash XOR step.
    ----------------------------------------------------------------------
    signal s2_x           : unsigned(11 downto 0) := (others => '0');
    signal s2_y           : unsigned(11 downto 0) := (others => '0');
    signal s2_zone_A      : std_logic := '0';
    signal s2_zone_C      : std_logic := '0';
    signal s2_hue_A_idx   : unsigned(9 downto 0) := (others => '0');   -- fast vertical stripes
    signal s2_hue_C_idx   : unsigned(9 downto 0) := (others => '0');   -- slow horizontal bands
    signal s2_dx3         : unsigned(13 downto 0) := (others => '0');  -- 3*dx for 3:1 aspect
    signal s2_dy_r        : unsigned(11 downto 0) := (others => '0');
    signal s2_noise_mix   : unsigned(15 downto 0) := (others => '0');  -- xor mix
    signal s2_noise_select: unsigned(2 downto 0) := (others => '0');   -- top bits of earlier hash
    signal s2_tear_hit_A  : std_logic := '0';
    signal s2_tear_hit_C  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 3 signals: octagonal distance pre-step (big vs small),
    -- hash multiply first half, selected hue for palette lookup.
    ----------------------------------------------------------------------
    signal s3_x           : unsigned(11 downto 0) := (others => '0');
    signal s3_y           : unsigned(11 downto 0) := (others => '0');
    signal s3_zone_A      : std_logic := '0';
    signal s3_zone_C      : std_logic := '0';
    signal s3_octa_big    : unsigned(13 downto 0) := (others => '0');  -- max(3*dx, dy)
    signal s3_octa_small  : unsigned(13 downto 0) := (others => '0');  -- min(3*dx, dy)
    signal s3_hue_sel     : unsigned(9 downto 0) := (others => '0');   -- final hue for palette
    signal s3_noise_prod  : unsigned(31 downto 0) := (others => '0');  -- hash multiply result
    signal s3_tear_hit_A  : std_logic := '0';
    signal s3_tear_hit_C  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 4 signals: octagonal distance sum, hash finalise,
    -- palette base/next indices.
    ----------------------------------------------------------------------
    signal s4_x           : unsigned(11 downto 0) := (others => '0');
    signal s4_y           : unsigned(11 downto 0) := (others => '0');
    signal s4_zone_A      : std_logic := '0';
    signal s4_zone_C      : std_logic := '0';
    signal s4_octa_dist   : unsigned(14 downto 0) := (others => '0');  -- big + (small>>1)
    signal s4_hash        : unsigned(15 downto 0) := (others => '0');  -- finalised hash bits
    signal s4_hue_base_i  : unsigned(2 downto 0) := (others => '0');
    signal s4_hue_next_i  : unsigned(2 downto 0) := (others => '0');
    signal s4_hue_frac    : unsigned(6 downto 0) := (others => '0');
    signal s4_tear_hit_A  : std_logic := '0';
    signal s4_tear_hit_C  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 5 signals: portal_hit decided, palette entries fetched,
    -- noise values computed.
    ----------------------------------------------------------------------
    signal s5_zone_A      : std_logic := '0';
    signal s5_zone_C      : std_logic := '0';
    signal s5_portal_hit  : std_logic := '0';
    signal s5_pal_base_y  : unsigned(9 downto 0) := (others => '0');
    signal s5_pal_next_y  : unsigned(9 downto 0) := (others => '0');
    signal s5_pal_base_u  : unsigned(9 downto 0) := (others => '0');
    signal s5_pal_next_u  : unsigned(9 downto 0) := (others => '0');
    signal s5_pal_base_v  : unsigned(9 downto 0) := (others => '0');
    signal s5_pal_next_v  : unsigned(9 downto 0) := (others => '0');
    signal s5_hue_frac    : unsigned(6 downto 0) := (others => '0');
    signal s5_noise_luma  : unsigned(9 downto 0) := (others => '0');
    signal s5_noise_poke  : std_logic := '0';        -- occasional rainbow pokethrough
    signal s5_noise_pkidx : unsigned(2 downto 0) := (others => '0');
    signal s5_tear_hit_A  : std_logic := '0';
    signal s5_tear_hit_C  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 6 signals: palette diffs (signed).
    ----------------------------------------------------------------------
    signal s6_zone_A      : std_logic := '0';
    signal s6_zone_C      : std_logic := '0';
    signal s6_portal_hit  : std_logic := '0';
    signal s6_diff_y      : signed(10 downto 0) := (others => '0');
    signal s6_diff_u      : signed(10 downto 0) := (others => '0');
    signal s6_diff_v      : signed(10 downto 0) := (others => '0');
    signal s6_pal_base_y  : unsigned(9 downto 0) := (others => '0');
    signal s6_pal_base_u  : unsigned(9 downto 0) := (others => '0');
    signal s6_pal_base_v  : unsigned(9 downto 0) := (others => '0');
    signal s6_hue_frac    : unsigned(6 downto 0) := (others => '0');
    signal s6_noise_luma  : unsigned(9 downto 0) := (others => '0');
    signal s6_noise_poke  : std_logic := '0';
    signal s6_noise_pk_y  : unsigned(9 downto 0) := (others => '0');
    signal s6_noise_pk_u  : unsigned(9 downto 0) := (others => '0');
    signal s6_noise_pk_v  : unsigned(9 downto 0) := (others => '0');
    signal s6_tear_hit_A  : std_logic := '0';
    signal s6_tear_hit_C  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 7 signals: palette blend multiplies (pipelined mul).
    ----------------------------------------------------------------------
    signal s7_zone_A      : std_logic := '0';
    signal s7_zone_C      : std_logic := '0';
    signal s7_portal_hit  : std_logic := '0';
    signal s7_prod_y      : signed(18 downto 0) := (others => '0');
    signal s7_prod_u      : signed(18 downto 0) := (others => '0');
    signal s7_prod_v      : signed(18 downto 0) := (others => '0');
    signal s7_pal_base_y  : unsigned(9 downto 0) := (others => '0');
    signal s7_pal_base_u  : unsigned(9 downto 0) := (others => '0');
    signal s7_pal_base_v  : unsigned(9 downto 0) := (others => '0');
    signal s7_noise_luma  : unsigned(9 downto 0) := (others => '0');
    signal s7_noise_poke  : std_logic := '0';
    signal s7_noise_pk_y  : unsigned(9 downto 0) := (others => '0');
    signal s7_noise_pk_u  : unsigned(9 downto 0) := (others => '0');
    signal s7_noise_pk_v  : unsigned(9 downto 0) := (others => '0');
    signal s7_tear_hit_A  : std_logic := '0';
    signal s7_tear_hit_C  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 8 signals: palette blend sums (rainbow_y/u/v), noise final.
    ----------------------------------------------------------------------
    signal s8_zone_A      : std_logic := '0';
    signal s8_zone_C      : std_logic := '0';
    signal s8_portal_hit  : std_logic := '0';
    signal s8_rainbow_y   : unsigned(9 downto 0) := (others => '0');
    signal s8_rainbow_u   : unsigned(9 downto 0) := (others => '0');
    signal s8_rainbow_v   : unsigned(9 downto 0) := (others => '0');
    signal s8_noise_y     : unsigned(9 downto 0) := (others => '0');
    signal s8_noise_u     : unsigned(9 downto 0) := (others => '0');
    signal s8_noise_v     : unsigned(9 downto 0) := (others => '0');
    signal s8_tear_hit_A  : std_logic := '0';
    signal s8_tear_hit_C  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 9 signals: zone color select (mux across zones / portal / noise).
    ----------------------------------------------------------------------
    signal s9_y           : unsigned(9 downto 0) := (others => '0');
    signal s9_u           : unsigned(9 downto 0) := (others => '0');
    signal s9_v           : unsigned(9 downto 0) := (others => '0');
    signal s9_is_bypass   : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 10 signals: brightness multiply (luma scale).
    ----------------------------------------------------------------------
    signal s10_y_mul      : unsigned(19 downto 0) := (others => '0');
    signal s10_u          : unsigned(9 downto 0) := (others => '0');
    signal s10_v          : unsigned(9 downto 0) := (others => '0');
    signal s10_is_bypass  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 11 signals: chroma scale toward neutral using brightness.
    -- u_out = 512 + (u_in - 512) * brt / 1024
    ----------------------------------------------------------------------
    signal s11_y          : unsigned(9 downto 0) := (others => '0');
    signal s11_u_mul      : signed(21 downto 0) := (others => '0');
    signal s11_v_mul      : signed(21 downto 0) := (others => '0');
    signal s11_is_bypass  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 12 signals: final chroma assembly.
    ----------------------------------------------------------------------
    signal s12_y          : unsigned(9 downto 0) := (others => '0');
    signal s12_u          : unsigned(9 downto 0) := (others => '0');
    signal s12_v          : unsigned(9 downto 0) := (others => '0');
    signal s12_is_bypass  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Stage 13 signals: final output mux (bypass vs effect).
    ----------------------------------------------------------------------
    signal s13_y          : unsigned(9 downto 0) := (others => '0');
    signal s13_u          : unsigned(9 downto 0) := (others => '0');
    signal s13_v          : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Helper: saturating addition (11-bit -> 10-bit)
    ----------------------------------------------------------------------
    function sat10_add(a : unsigned(9 downto 0); b : signed(10 downto 0))
        return unsigned is
        variable s : signed(11 downto 0);
    begin
        s := signed(std_logic_vector(resize(a, 12))) + resize(b, 12);
        if s < 0 then
            return to_unsigned(0, 10);
        elsif s > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(std_logic_vector(s(9 downto 0)));
        end if;
    end function;

    ----------------------------------------------------------------------
    -- Helper: clamp signed(21 downto 0) -> unsigned(9 downto 0), with
    -- centering. Used to recover u_out after multiplying centered
    -- (signed 11-bit) u by brt (unsigned 10-bit, lifted to signed 11-bit).
    ----------------------------------------------------------------------
    function sat10_from_centered(c : signed(21 downto 0))
        return unsigned is
        variable t : signed(22 downto 0);
    begin
        -- c = (u - 512) * brt, range ±512*1023; we divide by 1024
        -- then add 512 to re-center.
        t := resize(shift_right(c, 10), 23) + to_signed(512, 23);
        if t < 0 then
            return to_unsigned(0, 10);
        elsif t > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(std_logic_vector(t(9 downto 0)));
        end if;
    end function;

begin

    ----------------------------------------------------------------------
    -- Position tracking + parameter latching + per-frame derived values.
    -- All happens in one process so the control state is coherent.
    ----------------------------------------------------------------------
    p_position : process(clk)
        variable v_hsync_edge : std_logic;
        variable v_vsync_edge : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_hsync_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                v_hsync_edge := '1';
            end if;

            v_vsync_edge := '0';
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                v_vsync_edge := '1';
            end if;

            -- Track pixel_x: reset at hsync leading edge, increment when avid.
            -- last_x_r tracks the MAX pixel_x across the frame — only update
            -- on hsync when the just-finished line was longer than any prior
            -- (otherwise blanking lines with pixel_x=0 would wipe it out).
            if v_hsync_edge = '1' then
                if pixel_x > last_x_r then
                    last_x_r <= pixel_x;
                end if;
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            -- Track pixel_y: reset at vsync, increment at hsync. Also use
            -- max-tracking: we latch at vsync the highest pixel_y seen so
            -- far (resets every new frame by not-reaching-max the first
            -- time through, but it's more robust than unconditional load).
            if v_vsync_edge = '1' then
                if pixel_y > last_y_r then
                    last_y_r <= pixel_y;
                end if;
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                -- Latch screen bounds and parameters for the frame
                max_x_r <= last_x_r;
                max_y_r <= last_y_r;

                portal_size_r <= unsigned(registers_in(0));
                portal_x_r    <= unsigned(registers_in(1));
                portal_y_r    <= unsigned(registers_in(2));
                noise_k_r     <= unsigned(registers_in(3));
                hue_speed_r   <= unsigned(registers_in(4));
                band_k_r      <= unsigned(registers_in(5));
                top_en_r      <= registers_in(6)(0);
                bot_en_r      <= registers_in(6)(1);
                portal_en_r   <= registers_in(6)(2);
                tear_en_r     <= registers_in(6)(3);
                bypass_r      <= registers_in(6)(4);
                brt_r         <= unsigned(registers_in(7));

                -- Animate the palette phase (only on frame boundaries so
                -- the main pipeline sees a stable offset within a frame).
                -- At hue_speed_r=128 the phase increment is ~1024/frame, so
                -- hue_offset_r (top 10 bits) completes one rainbow cycle in
                -- ~64 frames (~1 second at 60Hz).
                hue_phase    <= hue_phase + shift_left(resize(hue_speed_r, 16), 3);
                hue_offset_r <= hue_phase(15 downto 6);

                -- Pick a noise grain size: smaller noise_k -> finer grain.
                if noise_k_r(9 downto 7) = "000" then
                    noise_shift_r <= 1;
                elsif noise_k_r(9 downto 7) = "001" then
                    noise_shift_r <= 2;
                elsif noise_k_r(9 downto 7) = "010" then
                    noise_shift_r <= 2;
                elsif noise_k_r(9 downto 7) = "011" then
                    noise_shift_r <= 3;
                elsif noise_k_r(9 downto 7) = "100" then
                    noise_shift_r <= 3;
                elsif noise_k_r(9 downto 7) = "101" then
                    noise_shift_r <= 4;
                elsif noise_k_r(9 downto 7) = "110" then
                    noise_shift_r <= 4;
                else
                    noise_shift_r <= 5;
                end if;

                -- Zone seams: Zone A = top ~3/16 (~20%), Zone C = bottom ~3/16.
                zoneA_y_r <= resize(shift_right(last_y_r, 2), 12) -
                             resize(shift_right(last_y_r, 4), 12);
                zoneC_y_r <= last_y_r -
                             (resize(shift_right(last_y_r, 2), 12) -
                              resize(shift_right(last_y_r, 4), 12));

                -- Band-frequency factors for the hue multipliers.
                -- band_A = 4 + band_k(9:4)   (high-frequency vertical)
                -- band_C = 1 + band_k(9:6)   (low-frequency horizontal)
                band_A_r <= resize(band_k_r(9 downto 4), 7) + to_unsigned(4, 7);
                band_C_r <= resize(band_k_r(9 downto 6), 5) + to_unsigned(1, 5);

            elsif v_hsync_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;

            -- Per-frame multipliers — run EVERY cycle so we don't depend on
            -- the exact timing of the vsync latch. last_x_r / last_y_r /
            -- portal_* are stable within a frame, so the product is constant
            -- during a frame and updates one cycle after parameters change.
            -- (Using last_*_r directly avoids the extra frame of staleness
            -- introduced by going through max_*_r.)
            cx_mul_r    <= last_x_r * portal_x_r;
            cy_mul_r    <= last_y_r * portal_y_r;
            rx_mul_r    <= last_x_r * ('0' & portal_size_r(9 downto 1));
            cx_r        <= cx_mul_r(21 downto 10);
            cy_r        <= cy_mul_r(21 downto 10);
            portal_rx_r <= resize(rx_mul_r(21 downto 12), 12);
        end if;
    end process p_position;

    ----------------------------------------------------------------------
    -- Stage 1: register incoming video into pipe(0), classify zones,
    -- start portal abs-diff, derive noise-sample indices, prep hue bases.
    ----------------------------------------------------------------------
    p_stage1 : process(clk)
        variable v_dx      : unsigned(11 downto 0);
        variable v_dy      : unsigned(11 downto 0);
        variable v_near_A  : std_logic;
        variable v_near_C  : std_logic;
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;

            s1_x <= pixel_x;
            s1_y <= pixel_y;

            -- Zone classification: strict A/C plus "near seam" for tear.
            if pixel_y < zoneA_y_r then
                s1_zone_A <= '1';
            else
                s1_zone_A <= '0';
            end if;

            if pixel_y > zoneC_y_r then
                s1_zone_C <= '1';
            else
                s1_zone_C <= '0';
            end if;

            -- "Near seam" is within ±16 px of the boundary
            v_near_A := '0';
            if (pixel_y + 16 >= zoneA_y_r) and (pixel_y < zoneA_y_r + 16) then
                v_near_A := '1';
            end if;
            v_near_C := '0';
            if (pixel_y + 16 >= zoneC_y_r) and (pixel_y < zoneC_y_r + 16) then
                v_near_C := '1';
            end if;
            s1_near_topA <= v_near_A;
            s1_near_botC <= v_near_C;

            -- Absolute diffs toward the portal center
            if pixel_x >= cx_r then
                v_dx := pixel_x - cx_r;
            else
                v_dx := cx_r - pixel_x;
            end if;
            if pixel_y >= cy_r then
                v_dy := pixel_y - cy_r;
            else
                v_dy := cy_r - pixel_y;
            end if;
            s1_dx <= v_dx;
            s1_dy <= v_dy;

            -- Hue base components via real multipliers — 12x7 and 12x5
            -- are comfortably within one clock at 74.25 MHz on ice40.
            -- Truncating to bottom 10 bits implicitly wraps the hue.
            s1_hue_base_A <= resize(pixel_x * band_A_r, 10);
            s1_hue_base_C <= resize(pixel_y * band_C_r, 10);

            -- Noise sampling indices. X shifted 1 more than Y → cells ~2x
            -- wider than tall, giving subtle horizontal streaks without
            -- dominating the texture.
            case noise_shift_r is
                when 1 => s1_noise_xi <= shift_right(pixel_x, 2);
                          s1_noise_yi <= shift_right(pixel_y, 1);
                when 2 => s1_noise_xi <= shift_right(pixel_x, 3);
                          s1_noise_yi <= shift_right(pixel_y, 2);
                when 3 => s1_noise_xi <= shift_right(pixel_x, 4);
                          s1_noise_yi <= shift_right(pixel_y, 3);
                when 4 => s1_noise_xi <= shift_right(pixel_x, 5);
                          s1_noise_yi <= shift_right(pixel_y, 4);
                when others => s1_noise_xi <= shift_right(pixel_x, 6);
                               s1_noise_yi <= shift_right(pixel_y, 5);
            end case;
        end if;
    end process p_stage1;

    ----------------------------------------------------------------------
    -- Stage 2: derive full hue indices with band_k/hue_offset applied,
    -- build 3*dx (octagonal aspect), start noise hash (XOR mix).
    ----------------------------------------------------------------------
    p_stage2 : process(clk)
        variable v_dx3   : unsigned(13 downto 0);
        variable v_hueA  : unsigned(9 downto 0);
        variable v_hueC  : unsigned(9 downto 0);
        variable v_mix   : unsigned(15 downto 0);
        variable v_tearA : std_logic;
        variable v_tearC : std_logic;
    begin
        if rising_edge(clk) then
            pipe(1) <= pipe(0);

            s2_x      <= s1_x;
            s2_y      <= s1_y;
            s2_zone_A <= s1_zone_A;
            s2_zone_C <= s1_zone_C;

            -- Hue A index (vertical stripes) with per-row shimmer + slow
            -- frame-based twinkle. The per-row term offsets each scanline
            -- by a few palette slots, producing the "scanline jitter" look
            -- visible in the reference image's top strip.
            v_hueA := s1_hue_base_A +
                      resize(s1_y(5 downto 1), 10) +
                      resize(frame_count(8 downto 3), 10);
            s2_hue_A_idx <= v_hueA;

            -- Hue C index (horizontal bands) with slow animated scroll.
            v_hueC := s1_hue_base_C + hue_offset_r;
            s2_hue_C_idx <= v_hueC;

            -- 3*dx for octagonal 3:1 aspect ratio (tall ellipse shape)
            v_dx3 := resize(s1_dx, 14) + shift_left(resize(s1_dx, 14), 1);
            s2_dx3  <= v_dx3;
            s2_dy_r <= s1_dy;

            -- Noise hash XOR mix (combine x, y, frame, seed)
            v_mix := s1_noise_xi(7 downto 0) & s1_noise_yi(7 downto 0);
            v_mix := v_mix xor frame_count;
            v_mix := v_mix xor to_unsigned(16#A3B1#, 16);
            s2_noise_mix   <= v_mix;
            s2_noise_select <= s1_noise_xi(2 downto 0) xor s1_noise_yi(2 downto 0);

            -- Tear seam detection: only fire when tear_en and near a seam
            v_tearA := s1_near_topA and tear_en_r;
            v_tearC := s1_near_botC and tear_en_r;
            s2_tear_hit_A <= v_tearA;
            s2_tear_hit_C <= v_tearC;
        end if;
    end process p_stage2;

    ----------------------------------------------------------------------
    -- Stage 3: octagonal distance pre-compute (big/small), start 16x16
    -- hash multiply, select which hue to feed into the palette.
    ----------------------------------------------------------------------
    p_stage3 : process(clk)
        variable v_dy14  : unsigned(13 downto 0);
        variable v_big   : unsigned(13 downto 0);
        variable v_small : unsigned(13 downto 0);
        variable v_sel   : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            pipe(2) <= pipe(1);

            s3_x      <= s2_x;
            s3_y      <= s2_y;
            s3_zone_A <= s2_zone_A;
            s3_zone_C <= s2_zone_C;

            -- max(3*dx, dy), min(3*dx, dy) for octagonal distance
            v_dy14 := resize(s2_dy_r, 14);
            if s2_dx3 >= v_dy14 then
                v_big   := s2_dx3;
                v_small := v_dy14;
            else
                v_big   := v_dy14;
                v_small := s2_dx3;
            end if;
            s3_octa_big   <= v_big;
            s3_octa_small <= v_small;

            -- Pre-select hue index: zone A uses hue_A, everything else uses hue_C
            if s2_zone_A = '1' then
                v_sel := s2_hue_A_idx;
            else
                v_sel := s2_hue_C_idx;
            end if;
            s3_hue_sel <= v_sel;

            -- Hash multiply: noise_mix * Fibonacci constant 0x9E37.
            -- 16x16 -> 32. On HX4K this is a ~400-LC combinational mul;
            -- registering the product breaks the long path.
            s3_noise_prod <= s2_noise_mix * to_unsigned(16#9E37#, 16);

            s3_tear_hit_A <= s2_tear_hit_A;
            s3_tear_hit_C <= s2_tear_hit_C;
        end if;
    end process p_stage3;

    ----------------------------------------------------------------------
    -- Stage 4: octagonal distance sum, hash final XOR shift,
    -- split hue_sel into base palette index + fractional blend weight.
    ----------------------------------------------------------------------
    p_stage4 : process(clk)
        variable v_dist   : unsigned(14 downto 0);
        variable v_h      : unsigned(15 downto 0);
        variable v_base_i : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            pipe(3) <= pipe(2);

            s4_x      <= s3_x;
            s4_y      <= s3_y;
            s4_zone_A <= s3_zone_A;
            s4_zone_C <= s3_zone_C;

            -- octa = big + (small >> 1) — very close to true L2 distance
            v_dist := resize(s3_octa_big, 15) + resize(shift_right(s3_octa_small, 1), 15);
            s4_octa_dist <= v_dist;

            -- Hash finalise: XOR-shift mixing for avalanche
            v_h := s3_noise_prod(23 downto 8);
            v_h := v_h xor shift_right(v_h, 5);
            s4_hash <= v_h;

            -- Palette indexing
            v_base_i := s3_hue_sel(9 downto 7);
            s4_hue_base_i <= v_base_i;
            s4_hue_next_i <= v_base_i + 1;   -- modulo 8 by natural 3-bit wrap
            s4_hue_frac   <= s3_hue_sel(6 downto 0);

            s4_tear_hit_A <= s3_tear_hit_A;
            s4_tear_hit_C <= s3_tear_hit_C;
        end if;
    end process p_stage4;

    ----------------------------------------------------------------------
    -- Stage 5: portal_hit compare, palette LUT fetch, noise value compute.
    ----------------------------------------------------------------------
    p_stage5 : process(clk)
        variable v_thresh  : unsigned(14 downto 0);
        variable v_hit     : std_logic;
        variable v_noise_y : unsigned(9 downto 0);
        variable v_poke    : std_logic;
    begin
        if rising_edge(clk) then
            pipe(4) <= pipe(3);

            s5_zone_A <= s4_zone_A;
            s5_zone_C <= s4_zone_C;

            -- Portal hit if octa_dist < 3*portal_rx_r (scaled 15-bit).
            v_thresh := resize(portal_rx_r, 15) + resize(shift_left(portal_rx_r, 1), 15);
            if s4_octa_dist < v_thresh then
                v_hit := '1';
            else
                v_hit := '0';
            end if;
            s5_portal_hit <= v_hit;

            -- Palette fetch (constant ROM lookup — inlines to LUT4s)
            s5_pal_base_y <= C_PAL_Y(to_integer(s4_hue_base_i));
            s5_pal_next_y <= C_PAL_Y(to_integer(s4_hue_next_i));
            s5_pal_base_u <= C_PAL_U(to_integer(s4_hue_base_i));
            s5_pal_next_u <= C_PAL_U(to_integer(s4_hue_next_i));
            s5_pal_base_v <= C_PAL_V(to_integer(s4_hue_base_i));
            s5_pal_next_v <= C_PAL_V(to_integer(s4_hue_next_i));
            s5_hue_frac   <= s4_hue_frac;

            -- Noise luma: map top 8 bits of hash to [100..865] grayscale.
            -- v_noise_y = 100 + 3 * hash_byte, where 3*x = (x<<1) + x.
            v_noise_y := to_unsigned(100, 10) +
                         resize(s4_hash(15 downto 8), 10) +
                         resize(shift_left(resize(s4_hash(15 downto 8), 10), 1), 10);
            s5_noise_luma <= v_noise_y;

            -- Rainbow pokethrough: 1/32 probability (bottom 5 bits = 0)
            if s4_hash(4 downto 0) = "00000" then
                v_poke := '1';
            else
                v_poke := '0';
            end if;
            s5_noise_poke  <= v_poke;
            s5_noise_pkidx <= s4_hash(10 downto 8);

            s5_tear_hit_A <= s4_tear_hit_A;
            s5_tear_hit_C <= s4_tear_hit_C;
        end if;
    end process p_stage5;

    ----------------------------------------------------------------------
    -- Stage 6: palette diffs (signed) and noise pokethrough palette.
    ----------------------------------------------------------------------
    p_stage6 : process(clk)
    begin
        if rising_edge(clk) then
            pipe(5) <= pipe(4);

            s6_zone_A     <= s5_zone_A;
            s6_zone_C     <= s5_zone_C;
            s6_portal_hit <= s5_portal_hit;

            -- Signed differences for palette interpolation
            s6_diff_y <= signed('0' & std_logic_vector(s5_pal_next_y)) -
                         signed('0' & std_logic_vector(s5_pal_base_y));
            s6_diff_u <= signed('0' & std_logic_vector(s5_pal_next_u)) -
                         signed('0' & std_logic_vector(s5_pal_base_u));
            s6_diff_v <= signed('0' & std_logic_vector(s5_pal_next_v)) -
                         signed('0' & std_logic_vector(s5_pal_base_v));

            s6_pal_base_y <= s5_pal_base_y;
            s6_pal_base_u <= s5_pal_base_u;
            s6_pal_base_v <= s5_pal_base_v;
            s6_hue_frac   <= s5_hue_frac;

            s6_noise_luma <= s5_noise_luma;
            s6_noise_poke <= s5_noise_poke;
            -- Pokethrough palette fetch — constant LUT, no combinational delay concern
            s6_noise_pk_y <= C_PAL_Y(to_integer(s5_noise_pkidx));
            s6_noise_pk_u <= C_PAL_U(to_integer(s5_noise_pkidx));
            s6_noise_pk_v <= C_PAL_V(to_integer(s5_noise_pkidx));

            s6_tear_hit_A <= s5_tear_hit_A;
            s6_tear_hit_C <= s5_tear_hit_C;
        end if;
    end process p_stage6;

    ----------------------------------------------------------------------
    -- Stage 7: palette blend multiplies (diff * frac, signed 11b x 7b).
    ----------------------------------------------------------------------
    p_stage7 : process(clk)
        variable v_frac : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            pipe(6) <= pipe(5);

            s7_zone_A     <= s6_zone_A;
            s7_zone_C     <= s6_zone_C;
            s7_portal_hit <= s6_portal_hit;

            v_frac := signed('0' & std_logic_vector(s6_hue_frac));
            s7_prod_y <= s6_diff_y * v_frac;
            s7_prod_u <= s6_diff_u * v_frac;
            s7_prod_v <= s6_diff_v * v_frac;

            s7_pal_base_y <= s6_pal_base_y;
            s7_pal_base_u <= s6_pal_base_u;
            s7_pal_base_v <= s6_pal_base_v;

            s7_noise_luma <= s6_noise_luma;
            s7_noise_poke <= s6_noise_poke;
            s7_noise_pk_y <= s6_noise_pk_y;
            s7_noise_pk_u <= s6_noise_pk_u;
            s7_noise_pk_v <= s6_noise_pk_v;

            s7_tear_hit_A <= s6_tear_hit_A;
            s7_tear_hit_C <= s6_tear_hit_C;
        end if;
    end process p_stage7;

    ----------------------------------------------------------------------
    -- Stage 8: palette blend sums -> rainbow_y/u/v, noise_y/u/v finalise.
    ----------------------------------------------------------------------
    p_stage8 : process(clk)
        variable v_shift_y : signed(11 downto 0);
        variable v_shift_u : signed(11 downto 0);
        variable v_shift_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(7) <= pipe(6);

            s8_zone_A     <= s7_zone_A;
            s8_zone_C     <= s7_zone_C;
            s8_portal_hit <= s7_portal_hit;

            -- rainbow = base + (diff * frac) >> 7
            v_shift_y := resize(shift_right(s7_prod_y, 7), 12);
            v_shift_u := resize(shift_right(s7_prod_u, 7), 12);
            v_shift_v := resize(shift_right(s7_prod_v, 7), 12);
            s8_rainbow_y <= sat10_add(s7_pal_base_y, resize(v_shift_y, 11));
            s8_rainbow_u <= sat10_add(s7_pal_base_u, resize(v_shift_u, 11));
            s8_rainbow_v <= sat10_add(s7_pal_base_v, resize(v_shift_v, 11));

            -- Noise output: either grayscale or rainbow pokethrough
            if s7_noise_poke = '1' then
                s8_noise_y <= s7_noise_pk_y;
                s8_noise_u <= s7_noise_pk_u;
                s8_noise_v <= s7_noise_pk_v;
            else
                s8_noise_y <= s7_noise_luma;
                s8_noise_u <= C_NEUTRAL_U;
                s8_noise_v <= C_NEUTRAL_V;
            end if;

            s8_tear_hit_A <= s7_tear_hit_A;
            s8_tear_hit_C <= s7_tear_hit_C;
        end if;
    end process p_stage8;

    ----------------------------------------------------------------------
    -- Stage 9: zone color select. Combines tear-seam swap, zone disables.
    --   * bypass -> pass-through flag (applied at final mux)
    --   * tear seam hit -> use opposite-zone color (creates ragged edges)
    --   * zone_A + top_en -> rainbow (vertical stripes)
    --   * zone_C + bot_en -> rainbow (horizontal bands)
    --   * zone_B + portal_hit + portal_en -> rainbow (band through portal)
    --   * zone_B otherwise -> noise
    ----------------------------------------------------------------------
    p_stage9 : process(clk)
        variable v_is_zone_A   : std_logic;
        variable v_is_zone_C   : std_logic;
        variable v_in_portal   : std_logic;
        variable v_y           : unsigned(9 downto 0);
        variable v_u           : unsigned(9 downto 0);
        variable v_v           : unsigned(9 downto 0);
        variable v_tear_hash   : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            pipe(8) <= pipe(7);

            -- Tear effect: if within seam band, use a per-row hash to
            -- decide whether to apply zone A or zone B/C coloring.
            v_tear_hash := s8_rainbow_y(4 downto 2) xor s8_noise_y(4 downto 2);

            v_is_zone_A := s8_zone_A;
            v_is_zone_C := s8_zone_C;
            if s8_tear_hit_A = '1' and v_tear_hash(0) = '1' then
                v_is_zone_A := not v_is_zone_A;
            end if;
            if s8_tear_hit_C = '1' and v_tear_hash(1) = '1' then
                v_is_zone_C := not v_is_zone_C;
            end if;

            v_in_portal := s8_portal_hit and portal_en_r;

            if bypass_r = '1' then
                -- Bypass path handled at final mux; put placeholder.
                v_y := s8_rainbow_y;
                v_u := s8_rainbow_u;
                v_v := s8_rainbow_v;
            elsif v_is_zone_A = '1' and top_en_r = '1' then
                v_y := s8_rainbow_y;
                v_u := s8_rainbow_u;
                v_v := s8_rainbow_v;
            elsif v_is_zone_C = '1' and bot_en_r = '1' then
                v_y := s8_rainbow_y;
                v_u := s8_rainbow_u;
                v_v := s8_rainbow_v;
            elsif v_is_zone_A = '0' and v_is_zone_C = '0' and v_in_portal = '1' then
                v_y := s8_rainbow_y;
                v_u := s8_rainbow_u;
                v_v := s8_rainbow_v;
            elsif v_is_zone_A = '0' and v_is_zone_C = '0' then
                -- Zone B without portal: noise
                v_y := s8_noise_y;
                v_u := s8_noise_u;
                v_v := s8_noise_v;
            else
                -- Zone enabled-off fallback: desaturated neutral dark gray
                v_y := to_unsigned(64, 10);
                v_u := C_NEUTRAL_U;
                v_v := C_NEUTRAL_V;
            end if;

            s9_y          <= v_y;
            s9_u          <= v_u;
            s9_v          <= v_v;
            s9_is_bypass  <= bypass_r;
        end if;
    end process p_stage9;

    ----------------------------------------------------------------------
    -- Stage 10: luma brightness multiply.
    ----------------------------------------------------------------------
    p_stage10 : process(clk)
    begin
        if rising_edge(clk) then
            pipe(9) <= pipe(8);

            s10_y_mul     <= s9_y * brt_r;
            s10_u         <= s9_u;
            s10_v         <= s9_v;
            s10_is_bypass <= s9_is_bypass;
        end if;
    end process p_stage10;

    ----------------------------------------------------------------------
    -- Stage 11: chroma scale toward neutral using brightness.
    --   u_out_centered = (u_in - 512) * brt  (range ±512*1023)
    -- Final sum & shift happens in stage 12.
    ----------------------------------------------------------------------
    p_stage11 : process(clk)
        variable v_uc : signed(10 downto 0);
        variable v_vc : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            pipe(10) <= pipe(9);

            -- Final Y = (y*brt) >> 10, saturated
            if s10_y_mul > to_unsigned(1023 * 1024, 20) then
                s11_y <= to_unsigned(1023, 10);
            else
                s11_y <= s10_y_mul(19 downto 10);
            end if;

            -- Signed, centered chroma
            v_uc := signed('0' & std_logic_vector(s10_u)) - to_signed(512, 11);
            v_vc := signed('0' & std_logic_vector(s10_v)) - to_signed(512, 11);
            s11_u_mul <= v_uc * signed('0' & std_logic_vector(brt_r));
            s11_v_mul <= v_vc * signed('0' & std_logic_vector(brt_r));
            s11_is_bypass <= s10_is_bypass;
        end if;
    end process p_stage11;

    ----------------------------------------------------------------------
    -- Stage 12: chroma re-center and saturate.
    ----------------------------------------------------------------------
    p_stage12 : process(clk)
    begin
        if rising_edge(clk) then
            pipe(11) <= pipe(10);

            s12_y         <= s11_y;
            s12_u         <= sat10_from_centered(s11_u_mul);
            s12_v         <= sat10_from_centered(s11_v_mul);
            s12_is_bypass <= s11_is_bypass;
        end if;
    end process p_stage12;

    ----------------------------------------------------------------------
    -- Stage 13: bypass mux (effect vs data_in).
    ----------------------------------------------------------------------
    p_stage13 : process(clk)
    begin
        if rising_edge(clk) then
            pipe(12) <= pipe(11);

            if s12_is_bypass = '1' then
                s13_y <= unsigned(pipe(11).y);
                s13_u <= unsigned(pipe(11).u);
                s13_v <= unsigned(pipe(11).v);
            else
                s13_y <= s12_y;
                s13_u <= s12_u;
                s13_v <= s12_v;
            end if;
        end if;
    end process p_stage13;

    ----------------------------------------------------------------------
    -- Final pipe stage — last sync delay slot.
    ----------------------------------------------------------------------
    p_stage_last : process(clk)
    begin
        if rising_edge(clk) then
            pipe(13) <= pipe(12);
        end if;
    end process p_stage_last;

    ----------------------------------------------------------------------
    -- Output: merge the sync/avid pipeline delay with the processed pixel.
    ----------------------------------------------------------------------
    data_out.y       <= std_logic_vector(s13_y);
    data_out.u       <= std_logic_vector(s13_u);
    data_out.v       <= std_logic_vector(s13_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture oracle;
