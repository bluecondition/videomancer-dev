-- Bajaweave: Two Smooth Curvy Sine Lines (Phase-Modulated Carriers)
--
-- Generates two vertical "lines" whose sway travels up / down the screen.
-- The bars themselves stay put horizontally.  Lines are smooth because:
--   * Carrier is a continuous sine wave (1024-entry LUT), not a square.
--   * Modulator is a sine LFO (also from the LUT), not triangle/sawtooth.
--   * Modulation is applied as a PHASE OFFSET to the carrier (PM), so the
--     resulting waveform is sin(carrier_phase + lfo_sin(line)) -- no
--     integration artefacts, no discontinuities.
--   * One per-field drift accumulator offsets the LFO angle, making the
--     sway pattern travel vertically.  The P12 slider is BIPOLAR: centre
--     = frozen, above centre the flow runs one way (full speed at the
--     top), below centre the other way (full speed at the bottom).  Osc1
--     always follows the slider; S10 selects whether osc2 flows the SAME
--     way or OPPOSITE (counter-flow).
--
-- The two oscillators are layered Woven (over/under alternating at each
-- crossing) or Stacked (S7), both keeping BOTH lines intact, in fully
-- saturated complementary colours.  S8 draws dark full-colour outlines
-- with a highlight on the outside of each bend; S9 selects Normal or
-- Darker overall brightness.  Every crossing also rings: a damped ripple
-- trailing to the right.
--
-- The per-line LFO accumulators are FRAME-LOCKED (reset at field start), so
-- K2/K5 purely set the sway's vertical wavelength.  The swing is a fixed
-- number of PIXELS proportional to that wavelength (constant lean), so it
-- stays visible at every Freq setting; K6 sets the amount.  The
-- ONLY temporal motion is the shared drift accumulator above (slider
-- centre = frozen).
-- S11 selects the waveshape: sine or triangle (carriers and LFOs both),
-- gliding between them over ~0.5 s.
--
-- Resources:
--   1x video_timing_generator
--   2x 20-bit per-line LFO accumulators (step at active-line end)
--   1x per-field gain sequencer (serial mult + divide, both oscs)
--   1x per-line LFO interpolation (sin + cos*frac) + serial PM-offset
--      multiply (both oscs, in hblank)
--   2x video_timing_accumulator (G_W=21, C_HORIZONTAL, lock=1)  : per-pixel carriers
--   1x 16-bit drift register (once per field, serration-guarded) : sway drift
--   2x sin_cos_full_lut_10x10 (sin+cos)                          : per-osc LFO
--   2x sin_cos_full_lut_10x10                                    : per-osc carrier sine
--   1x sin_cos_full_lut_10x10                                    : K3 hue -> (U,V)
--
-- Hardware contracts (v0.6):
--   * Luma is drawn into 64..765 (black..~75%): full-scale Y clips RGB and
--     kills the chroma at every line's crest, and 1..63 is below black.
--   * The program generates its OWN avid (Cubist's parity-locked version):
--     it starts 2-3 clocks after the source's first avid rise, keeping the
--     hsync->start parity at the source's usual value, and runs exactly the
--     field-latched line width.  The encoders pair Cb/Cr by counting from
--     hsync, so a wandering source avid swaps U/V for a line (thin coloured
--     lines over the bars).  The timing generator is fed this avid too, so
--     the carriers reset and the LFOs step off the clean signal.
--   * Y/U/V are gated to 64/512/512 outside the generated avid.
--   * The sway drift steps once per FIELD -- only on the first vsync edge
--     after active video, since analog vsync serrates (several edges).
--
-- Pipeline (data_in -> data_out):
--   1 clk : video_timing_generator
--   2 clk : video_timing_accumulator (input reg + accum)
--   1 clk : register PM phase (carrier_top + LFO_sin)
--   1 clk : carrier sine LUT lookup, register raw sin & tri
--   1 clk : second sin/tri register (the sine LUT infers as EBR and eats
--           the first register; this one splits EBR read from the output mux)
--   3 clk : sine/triangle glide (diff, partial sums, sum) + 512 -> luma
--   1 clk : S8 edge outlines + outer-bend highlight (per oscillator)
--   4 clk : layering keyer (select, B*m partials, scale, max)
--   4 clk : analog crossing reaction (ringing tail, clamp)
--   2 clk : luma-scaled chroma (partial products, then sum)
--   1 clk : luma remap to 64..765 + blanking gate -> output register
--   Total: 21 clk  (sync/avid delay, all modes)
--   (The LFO side chain runs in horizontal blanking: the LFO steps at the
--   end of each active line, and the next line's PM offset lfo*G/512 is
--   ready ~20 clocks later.  G = inc*D/step is computed once per field.)
--
-- Register map:
--   registers_in(0)  = K1: OSC1 spatial frequency (bars per screen)
--   registers_in(1)  = K2: OSC1 sway wavelength (~1..17 cycles/screen);
--                          the swing in pixels scales with the wavelength
--   registers_in(2)  = K3: COLOUR -- osc1 hue around the wheel, osc2 always
--                          its complement (eased per field)
--   registers_in(3)  = K4: OSC2 spatial frequency
--   registers_in(4)  = K5: OSC2 sway wavelength
--   registers_in(5)  = K6: SWAY amount, both oscillators (0 = straight
--                          lines .. max = ~4x the default swing)
--   registers_in(6)  = switches:
--                        bit 0 = S7  layering: 0 = Woven, 1 = Stacked
--                        bit 1 = S8  edges: dark full-colour outlines +
--                                    outer-bend highlight
--                        bit 2 = S9  brightness: 0 = Normal (~75% peak),
--                                    1 = Darker (~60% peak)
--                        bit 3 = S10 flow: 0 = osc2 opposite osc1, 1 = same
--                        bit 4 = S11 shape: 0 = sine, 1 = triangle
--                                (carriers and LFOs; GLIDES between the
--                                two over 32 fields, ~0.53 s at 60 Hz)
--   registers_in(7)  = slider 12: Speed, BIPOLAR -- centre = frozen,
--                                 ends = ~0.53 s per sway cycle each way;
--                                 speed only (eased), never the wave shape
--
-- License: GPL-3.0
-- Author: ron

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture bajaweave of program_top is

    constant C_DELAY_CLKS    : integer := 21;

    -- Carrier accumulators are 21 bits: the top 11 bits are an 11-bit PM
    -- phase whose bit 10 is the stripe PARITY (odd/even line) for Weave.
    constant C_CARRIER_W     : integer := 21;
    constant C_LFO_W         : integer := 20;

    -- Carrier increment scaling.  Per-pixel inc = (base << 5) + C_FREQ_MIN.
    --   At K=0     : inc = 1456,  cycle ~720 px = 1 sine peak across a line
    --   At K=1023  : inc = 34192, cycle ~30  px = ~23 peaks per line
    -- The non-zero floor is important: when inc=0 the carrier doesn't sweep
    -- horizontally at all, and the per-line LFO becomes the only variation
    -- in the picture -- which paints HORIZONTAL stripes instead of the
    -- vertical lines the program is supposed to draw.
    constant C_FREQ_KNOB_SH  : integer := 5;
    constant C_FREQ_MIN      : integer := 1456;
    -- the same increment at the 21-bit carrier width (x2)
    constant C_FREQ_KNOB_SH_C : integer := 6;
    constant C_FREQ_MIN_C     : integer := 2912;

    -- Analog crossing reaction.  x = crossing strength = the DIMMER wave's
    -- luma above mid (0..511; zero unless both are bright).
    --   (v0.11 also tinted crossings a third hue on S9 -- removed in v0.12.)
    --   ring  : peak-hold e decays C_RING_DECAY per pixel after each crossing
    --           and drives a damped 8-px ripple of +/-(e*3/8) trailing to the
    --           right -- an overdriven, band-limited video amp
    --   (v0.9-v0.11 also added +/-16 LFSR grain in the tail -- REMOVED: it
    --   re-rolled every frame, and a shift-register LFSR's adjacent pixels
    --   are bit-shifted copies, so it read as thin flickering horizontal
    --   streaks on hardware once the crossing flare no longer clipped it.)
    constant C_RING_DECAY    : integer := 8;

    -- LFO step (sway wavelength) from K2/K5 -- a two-segment knee, so the
    -- lower half of the knob covers long waves and the upper half tight ones:
    --   K <  512 : step =  960 +  6*K        (~1.0 .. ~4.2 cycles at 1080)
    --   K >= 512 : step = 4032 + 24*(K-512)  (~4.2 .. ~16.8 cycles)
    -- The LFO accumulator is frame-locked (reset at field start), so this is
    -- a pure SPATIAL wavelength of the sway down the screen.
    constant C_LFO_FLOOR     : integer := 960;
    constant C_LFO_KNEE      : integer := 4032;

    -- Sway AMOUNT.  The swing is a fixed number of PIXELS -- independent of
    -- the carrier frequency (K1/K4) -- and proportional to the wavelength,
    -- so every K2/K5 setting leans the lines by the same maximum angle:
    --   PM offset = (lfo/512) * G,   G = inc * D / step  (per field)
    -- An offset of u carrier-phase units is u*1024/inc pixels, so the swing
    -- is D*wavelength/1024 px and the max lean is 2*pi*D/1024:
    -- D = K6/2 (the Sway knob):
    --   K6 = 0    : D = 0   -> straight lines, no sway
    --   K6 = 256  : D = 128 -> ~0.79 -> 38 deg (the v0.7-v0.9 look)
    --   K6 = 1023 : D = 511 -> ~3.1  -> 72 deg, lines knot/fold

    -- Sway travel rate from the BIPOLAR P12 Speed slider (K = 0..1023):
    --   v = K - 512 (signed), |v| < C_SPEED_DEADBAND -> rate 0 (a centre
    --   detent you can actually hit), otherwise rate = 4*|v| per field
    --   into a 16-bit accumulator, run backwards when v < 0 (the rate is
    --   passed as 16-bit two's complement; the phase add wraps mod 2^16).
    -- Full sway cycle = 2^16/rate fields:
    --   slider end    (|v|=512) : rate 2048, ~32 fields ~= 0.53 s @ 60
    --   +/-25% travel (|v|=256) : rate 1024, ~64 fields ~= 1.1 s
    --   +/-6%         (|v|= 64) : rate  256, ~256 fields ~= 4.3 s
    --   centre        (|v|<  8) : frozen -- holds phase, resumes pop-free.
    -- Osc1 always follows the slider direction; osc2 adds or subtracts
    -- the same phase per S10 (Flow: opposite / same).
    -- SMOOTHNESS: the rate is carried with 4 fraction bits and EASED toward
    -- the slider by 1/8 of the gap per field (slider moves accelerate
    -- smoothly, pot noise is filtered); the 20-bit drift keeps 16 bits of
    -- sway angle, and the LFO interpolates between sine-table entries, so
    -- the pattern moves exactly rate/65536 of a cycle EVERY field instead of
    -- in whole 1/1024 table steps (the old 6,6,7,6,6,7 judder).
    constant C_DRIFT_W        : integer := 20;
    constant C_SPEED_DEADBAND : integer := 8;

    -- Combinational knob view
    signal s_k1, s_k2, s_k3 : unsigned(9 downto 0);
    signal s_k4, s_k5, s_k6 : unsigned(9 downto 0);
    signal s_s7             : std_logic;
    signal s_s8, s_s9, s_s10 : std_logic;
    signal s_s11            : std_logic;
    signal s_k12            : unsigned(9 downto 0);

    -- Vsync-latched (frame-stable)
    signal r_osc1_base : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal r_osc2_base : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal r_osc1_lfo  : unsigned(9 downto 0) := to_unsigned(200, 10);
    signal r_osc2_lfo  : unsigned(9 downto 0) := to_unsigned(320, 10);
    signal r_osc1_hue  : unsigned(9 downto 0) := (others => '0');
    signal r_osc2_hue  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_stacked   : std_logic := '0';   -- S7 Layering: 0 Woven, 1 Stacked
    signal r_edges     : std_logic := '0';   -- S8 Edges
    signal r_dark      : std_logic := '0';   -- S9 Brightness: 1 = Darker
    signal r_flow_same : std_logic := '0';
    signal r_speed     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_tri_en    : std_logic := '0';

    -- Bipolar speed -> signed drift rate, two registered stages off the
    -- frame-stable r_speed (magnitude/sign split, then deadband + scale).
    -- r_rate_tgt / r_rate_s are signed 16.4 fixed point.
    signal r_vel_mag    : unsigned(9 downto 0) := (others => '0');
    signal r_vel_neg    : std_logic := '0';
    signal r_rate_tgt   : signed(19 downto 0) := (others => '0');  -- 16.4
    signal r_rate_s     : signed(19 downto 0) := (others => '0');  -- eased
    signal r_rate_d, r_rate_nxt : signed(19 downto 0) := (others => '0');
    signal r_rate_snap  : std_logic := '1';
    signal r_hue_d      : signed(14 downto 0) := (others => '0');
    signal r_hue_nxt    : unsigned(13 downto 0) := (others => '0');
    signal r_hue_snap   : std_logic := '1';

    -- Timing record
    signal s_timing : t_video_timing_port;

    -- LFO accumulator IO
    signal s_lfo1_phase     : unsigned(C_LFO_W-1 downto 0);
    signal s_lfo2_phase     : unsigned(C_LFO_W-1 downto 0);

    -- LFO steps: knee operands registered, then one 3-input add
    signal r_st1_a, r_st1_b, r_st1_c : unsigned(13 downto 0) := (others => '0');
    signal r_st2_a, r_st2_b, r_st2_c : unsigned(13 downto 0) := (others => '0');
    signal r_lfo1_step, r_lfo2_step  : unsigned(13 downto 0) := to_unsigned(2160, 14);
    signal r_lfo1_acc,  r_lfo2_acc   : unsigned(C_LFO_W-1 downto 0) := (others => '0');

    -- Carrier increments (frame-stable, registered for the gain sequencer)
    signal r_osc1_inc, r_osc2_inc : unsigned(15 downto 0) := to_unsigned(9648, 16);

    -- Sway depth D (128 = x1 ... 512 = x4 at the slider ends)
    signal r_depth : unsigned(9 downto 0) := to_unsigned(128, 10);
    -- K3 colour: eased 10.4 hue (osc1); osc2 is always the complement
    signal r_hue14 : unsigned(13 downto 0) := (others => '0');

    -- Per-field gain sequencer: G = inc * D / step (serial multiply, then
    -- serial restoring divide; ~55 clocks after the field start)
    signal fg_st   : unsigned(1 downto 0) := "00";
    signal fg_cnt  : unsigned(4 downto 0) := (others => '0');
    signal fg_fin  : std_logic := '0';
    signal fg_done : std_logic := '0';
    signal fg_d    : unsigned(9 downto 0) := (others => '0');
    signal fg_n1, fg_n2 : unsigned(24 downto 0) := (others => '0');
    signal fg_r1, fg_r2 : unsigned(13 downto 0) := (others => '0');
    signal fg_q1, fg_q2 : unsigned(15 downto 0) := (others => '0');
    signal r_g1,  r_g2  : unsigned(15 downto 0) := (others => '0');

    -- Per-line PM offset = lfo * G / 512 (serial shift-add in hblank)
    signal r_ae_sr : std_logic_vector(15 downto 0) := (others => '0');
    signal lm_run, lm_fin : std_logic := '0';
    signal lm_cnt  : unsigned(3 downto 0) := (others => '0');
    signal lm_m1, lm_m2 : unsigned(15 downto 0) := (others => '0');
    signal lm_s1, lm_s2 : std_logic := '0';
    signal lm_a1, lm_a2 : unsigned(25 downto 0) := (others => '0');
    signal r_lfo1_off, r_lfo2_off : unsigned(10 downto 0) := (others => '0');

    -- Sway drift accumulator (steps once per field by the eased r_rate_s)
    signal s_drift_phase : unsigned(C_DRIFT_W-1 downto 0) := (others => '0');

    -- Raster measurement + serration-guarded field start
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_sawact     : std_logic := '0';
    signal s_fstart     : std_logic := '0';
    signal s_avid_q     : std_logic := '0';
    signal s_avid_r     : std_logic := '0';
    signal s_hs_q       : std_logic := '1';
    signal s_hs_r       : std_logic := '0';
    signal s_xcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_W          : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_Wl         : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- Generated avid: parity-locked start, runs exactly s_Wl pixels
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc  : unsigned(3 downto 0) := "1000";
    signal g_x   : unsigned(11 downto 0) := (others => '0');

    -- Registered LFO LUT angle = LFO phase [top 10b] +/- drift [top 10b]
    -- (+ for osc1, - for osc2: equal speed, opposite directions).
    -- Registered to keep the add out of the sine-LUT lookup path.
    signal r_lfo1_angle : unsigned(15 downto 0) := (others => '0');
    signal r_lfo2_angle : unsigned(15 downto 0) := (others => '0');
    signal s_lfo1_cos, s_lfo2_cos : signed(9 downto 0);
    -- interpolation pipeline (line-rate): cos, triangle slope, fraction
    signal r_lfo1_cos_r, r_lfo1_cos_r2, r_lfo2_cos_r, r_lfo2_cos_r2 : signed(9 downto 0) := (others => '0');
    signal r_lfo1_up_r, r_lfo1_up_r2, r_lfo2_up_r, r_lfo2_up_r2 : std_logic := '0';
    signal r_lfo1_fr_r, r_lfo1_fr_r2, r_lfo2_fr_r, r_lfo2_fr_r2 : unsigned(5 downto 0) := (others => '0');
    signal r_i1_pa, r_i1_pb, r_i2_pa, r_i2_pb : signed(16 downto 0) := (others => '0');
    signal r_i1_p, r_i2_p   : signed(16 downto 0) := (others => '0');
    signal r_i1_td, r_i2_td, r_i1_td2, r_i2_td2 : signed(8 downto 0) := (others => '0');
    signal r_i1_s, r_i1_t, r_i1_s2, r_i1_t2 : signed(9 downto 0) := (others => '0');
    signal r_i2_s, r_i2_t, r_i2_s2, r_i2_t2 : signed(9 downto 0) := (others => '0');
    signal r_i1_a, r_i1_b, r_i2_a, r_i2_b : signed(16 downto 0) := (others => '0');
    signal r_i1_tf, r_i2_tf : signed(16 downto 0) := (others => '0');
    signal r_i1_sf, r_i2_sf, r_i1_tf2, r_i2_tf2 : signed(16 downto 0) := (others => '0');
    -- fine LFO glide (units of 1/64 of a sine-table step)
    signal r_lf1_gd, r_lf2_gd : signed(17 downto 0) := (others => '0');
    signal r_lf1_gs, r_lf2_gs, r_lf1_gs2, r_lf2_gs2 : signed(16 downto 0) := (others => '0');
    signal r_lf1_gh, r_lf2_gh, r_lf1_gl, r_lf2_gl : signed(23 downto 0) := (others => '0');
    signal r_lfo1_fine, r_lfo2_fine : signed(16 downto 0) := (others => '0');

    -- LFO sine LUT outputs (combinational; registered LFO angle -> sin)
    signal s_lfo1_sin : signed(9 downto 0);
    signal s_lfo2_sin : signed(9 downto 0);

    -- LFO sine and triangle, registered TWICE before the morph: the sine
    -- LUTs infer as block RAM and yosys absorbs the first register into
    -- the EBR output stage, so a single register left the morph hanging
    -- straight off the 2 ns EBR read -- that was the post-route critical
    -- path.  The second stage restores the split (same on the carrier side).
    signal r_lfo1_sin_r, r_lfo1_sin_r2 : signed(9 downto 0) := (others => '0');
    signal r_lfo2_sin_r, r_lfo2_sin_r2 : signed(9 downto 0) := (others => '0');
    signal r_lfo1_tri_r, r_lfo1_tri_r2 : signed(9 downto 0) := (others => '0');
    signal r_lfo2_tri_r, r_lfo2_tri_r2 : signed(9 downto 0) := (others => '0');

    -- Shape-morphed LFO value registered (used as PM offset).  Stable
    -- across a scanline (LFO ticks per line).

    -- LFO and carrier triangle outputs (combinational fold of phase top 10b)
    signal s_lfo1_tri : signed(9 downto 0);
    signal s_lfo2_tri : signed(9 downto 0);
    signal s_osc1_tri : signed(9 downto 0);
    signal s_osc2_tri : signed(9 downto 0);

    -- Carrier sin and tri registered TWICE (see LFO note above: stage 1 is
    -- absorbed into the sine-LUT EBR, stage 2 splits EBR read from morph).
    signal r_osc1_sin_r, r_osc1_sin_r2 : signed(9 downto 0) := (others => '0');
    signal r_osc2_sin_r, r_osc2_sin_r2 : signed(9 downto 0) := (others => '0');
    signal r_osc1_tri_r, r_osc1_tri_r2 : signed(9 downto 0) := (others => '0');
    signal r_osc2_tri_r, r_osc2_tri_r2 : signed(9 downto 0) := (others => '0');

    -- Shape-morphed LFO / carrier outputs (combinational mix of sin & tri)
    -- Shape glide (S11): m eases 0 <-> 32 by one per field (~0.53 s at 60)
    signal r_glide : unsigned(5 downto 0) := (others => '0');
    -- glide stage A: d = tri - sin, sin delayed; stage B: partial sums
    signal r_lfo1_gd, r_lfo2_gd, r_osc1_gd, r_osc2_gd : signed(10 downto 0) := (others => '0');
    signal r_lfo1_gs, r_lfo2_gs, r_osc1_gs, r_osc2_gs : signed(9 downto 0)  := (others => '0');
    signal r_lfo1_gh, r_lfo2_gh, r_osc1_gh, r_osc2_gh : signed(16 downto 0) := (others => '0');
    signal r_lfo1_gl, r_lfo2_gl, r_osc1_gl, r_osc2_gl : signed(16 downto 0) := (others => '0');
    signal r_lfo1_gs2, r_lfo2_gs2, r_osc1_gs2, r_osc2_gs2 : signed(9 downto 0) := (others => '0');

    -- Carrier accumulator IO
    signal s_osc1_inc_slv : std_logic_vector(C_CARRIER_W-1 downto 0);
    signal s_osc2_inc_slv : std_logic_vector(C_CARRIER_W-1 downto 0);
    signal s_osc1_acc_slv : std_logic_vector(C_CARRIER_W-1 downto 0);
    signal s_osc2_acc_slv : std_logic_vector(C_CARRIER_W-1 downto 0);

    -- PM phase = carrier[top10] + LFO sin (modular)
    -- Combinational sum and registered version (registered breaks the
    -- 3-input add + sin LUT critical path so HD timing closes).
    signal s_osc1_pm_phase : unsigned(10 downto 0);
    signal s_osc2_pm_phase : unsigned(10 downto 0);
    signal r_osc1_pm_phase : unsigned(10 downto 0) := (others => '0');
    signal r_osc2_pm_phase : unsigned(10 downto 0) := (others => '0');
    -- stripe parity (Weave), delayed to line up with the edge stage output;
    -- flank side (phase bit 8: 0 = rising/left, 1 = falling/right), lined
    -- up with r_luma1/2
    signal r_par1_sr, r_par2_sr   : std_logic_vector(5 downto 0) := (others => '0');
    signal r_side1_sr, r_side2_sr : std_logic_vector(4 downto 0) := (others => '0');
    -- per-line outer-bend highlight: |lfo|/128 and the lfo sign
    signal lm_h1, lm_h2 : unsigned(8 downto 0) := (others => '0');
    signal r_hl1, r_hl2 : unsigned(8 downto 0) := (others => '0');
    signal r_ls1, r_ls2 : std_logic := '0';
    -- edge stage outputs (per oscillator)
    signal r_eL1, r_eL2 : unsigned(9 downto 0) := (others => '0');
    signal r_eu1, r_ev1, r_eu2, r_ev2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- rim flags: a darkened outline keeps FULL chroma (the luma-scaled
    -- chroma stage would otherwise grey it out); carried with the winner
    signal r_eR1, r_eR2 : std_logic := '0';
    signal r_kRT, r_kRB, r_kRT2, r_kRB2, r_kRT3, r_kRB3, r_rim_k : std_logic := '0';
    signal r_rim1, r_rim2, r_rim3, r_rim4 : std_logic := '0';

    -- Carrier sine LUT outputs (combinational; PM phase -> sin)
    signal s_osc1_sin : signed(9 downto 0);
    signal s_osc2_sin : signed(9 downto 0);

    -- Registered luma per pixel (sin + 512 mapped to 0..1023)
    signal r_luma1 : unsigned(9 downto 0) := (others => '0');
    signal r_luma2 : unsigned(9 downto 0) := (others => '0');

    -- Hue LUT outputs, plus a register stage: the LUT -> (mux, add) -> UV
    -- chain is per-frame logic but was the post-route critical path when
    -- combinational end-to-end.  We have all of vblank, so split it.
    signal s_sin1, s_cos1 : signed(9 downto 0);
    signal r_sin1_r, r_cos1_r : signed(9 downto 0) := (others => '0');

    -- Scaled chroma (Bright: c/2, Deep: c - c/8 = +/-448, the legal limit),
    -- one more register stage before the 512 offset add.
    signal r_du1, r_dv1 : signed(9 downto 0) := (others => '0');   -- +/-448

    -- Per-frame UV
    signal r_u1, r_v1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_u2, r_v2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_vsync_d  : std_logic := '0';
    signal r_vsync_d2 : std_logic := '0';

    -- Keyer register (luma still 0..1023 here)
    signal r_y_k : unsigned(9 downto 0) := (others => '0');
    signal r_u_k : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_v_k : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_cx  : unsigned(8 downto 0) := (others => '0');
    -- keyer stages K1..K3 (the K4 register is r_y_k/u/v)
    signal r_kT, r_kB, r_kT2, r_kT3, r_kBs : unsigned(9 downto 0) := (others => '0');
    signal r_kuT, r_kvT, r_kuB, r_kvB : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_kuT2, r_kvT2, r_kuB2, r_kvB2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_kuT3, r_kvT3, r_kuB3, r_kvB3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_km  : unsigned(5 downto 0) := (others => '0');
    signal r_kh, r_kl : signed(16 downto 0) := (others => '0');
    signal r_cx0, r_cx1, r_cx2 : unsigned(8 downto 0) := (others => '0');

    -- Crossing reaction stages R1..R4
    signal r_e    : unsigned(8 downto 0) := (others => '0');
    signal r_rc   : unsigned(2 downto 0) := (others => '0');
    signal r_y_1  : unsigned(9 downto 0) := (others => '0');
    signal r_u_1, r_v_1, r_u_2, r_v_2, r_u_3, r_v_3, r_u_4, r_v_4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_ya   : unsigned(10 downto 0) := (others => '0');
    signal r_rmag : unsigned(7 downto 0) := (others => '0');
    signal r_rneg : std_logic := '0';
    signal r_ys   : signed(12 downto 0) := (others => '0');
    signal r_y_r  : unsigned(9 downto 0) := (others => '0');

    -- Luma-scaled chroma (dark gaps go black, lines glow): two stages
    signal r_ua, r_ub, r_va, r_vb : signed(13 downto 0) := (others => '0');
    signal r_cfull : std_logic := '0';
    signal r_y_k2  : unsigned(9 downto 0) := (others => '0');
    signal r_u_k2  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_v_k2  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_y_c   : unsigned(9 downto 0) := (others => '0');
    signal r_u_c   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_v_c   : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Output register (luma remapped to 64..765, blanking-gated)
    signal r_y_out : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal r_u_out : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_v_out : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Sync delay (align the generated avid and data_in syncs with Y/U/V)
    signal s_avid_sr    : std_logic_vector(0 to C_DELAY_CLKS-1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_DELAY_CLKS-1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_DELAY_CLKS-1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_DELAY_CLKS-1) := (others => '1');

    -- ========================================================================
    -- Triangle wave from 10-bit phase, range matches sine LUT (-510..+510).
    -- Cheap: 1 XOR + 1 conditional negate, no multiplier.
    --   phase(9) = sign half (0 = positive, 1 = negative)
    --   phase(8) = fold (1 = ramp down within half)
    --   phase(7:0) = magnitude within fold
    -- ========================================================================
    function tri_fold(phase : unsigned(9 downto 0)) return signed is
        variable v_mag_a : unsigned(7 downto 0);
        variable v_mag9  : unsigned(8 downto 0);
        variable v_pos   : signed(9 downto 0);
    begin
        if phase(8) = '0' then
            v_mag_a := phase(7 downto 0);
        else
            v_mag_a := not phase(7 downto 0);
        end if;
        v_mag9 := shift_left(resize(v_mag_a, 9), 1);
        v_pos  := signed(resize(v_mag9, 10));
        if phase(9) = '0' then
            return v_pos;
        else
            return -v_pos;
        end if;
    end function;

    -- Shape glide: sin + (tri - sin) * m/32, m = 0..32, as shift-adds over
    -- the bits of m, split into two partial sums (one registered stage),
    -- then glide_sum adds them back onto sin (next stage).  m = 32 has only
    -- bit 5 set, so t1 is either d<<5 or d<<4, never both.
    function glide_hi(d : signed(10 downto 0); m : unsigned(5 downto 0)) return signed is
        variable v_d  : signed(16 downto 0);
        variable v_t1 : signed(16 downto 0);
        variable v_t2 : signed(16 downto 0);
        variable v_t3 : signed(16 downto 0);
    begin
        v_d  := resize(d, 17);
        v_t1 := (others => '0');
        v_t2 := (others => '0');
        v_t3 := (others => '0');
        if m(5) = '1' then
            v_t1 := shift_left(v_d, 5);
        elsif m(4) = '1' then
            v_t1 := shift_left(v_d, 4);
        end if;
        if m(3) = '1' then v_t2 := shift_left(v_d, 3); end if;
        if m(2) = '1' then v_t3 := shift_left(v_d, 2); end if;
        return v_t1 + v_t2 + v_t3;
    end function;

    function glide_lo(d : signed(10 downto 0); m : unsigned(5 downto 0)) return signed is
        variable v_d  : signed(16 downto 0);
        variable v_t1 : signed(16 downto 0);
        variable v_t2 : signed(16 downto 0);
    begin
        v_d  := resize(d, 17);
        v_t1 := (others => '0');
        v_t2 := (others => '0');
        if m(1) = '1' then v_t1 := shift_left(v_d, 1); end if;
        if m(0) = '1' then v_t2 := v_d; end if;
        return v_t1 + v_t2;
    end function;

    -- sin + (hi + lo)/32; the result lies between sin and tri, so the low
    -- 10 bits are exact.
    function glide_sum(sn : signed(9 downto 0); hi, lo : signed(16 downto 0)) return signed is
        variable v : signed(16 downto 0);
    begin
        v := resize(sn, 17) + shift_right(hi + lo, 5);
        return v(9 downto 0);
    end function;

    -- Width-generic versions for the fine (interpolated) LFO values.
    function glide_hi_g(d : signed; m : unsigned(5 downto 0)) return signed is
        variable v_d  : signed(d'length+5 downto 0);
        variable v_t1 : signed(d'length+5 downto 0);
        variable v_t2 : signed(d'length+5 downto 0);
        variable v_t3 : signed(d'length+5 downto 0);
    begin
        v_d  := resize(d, d'length+6);
        v_t1 := (others => '0');
        v_t2 := (others => '0');
        v_t3 := (others => '0');
        if m(5) = '1' then
            v_t1 := shift_left(v_d, 5);
        elsif m(4) = '1' then
            v_t1 := shift_left(v_d, 4);
        end if;
        if m(3) = '1' then v_t2 := shift_left(v_d, 3); end if;
        if m(2) = '1' then v_t3 := shift_left(v_d, 2); end if;
        return v_t1 + v_t2 + v_t3;
    end function;

    function glide_lo_g(d : signed; m : unsigned(5 downto 0)) return signed is
        variable v_d  : signed(d'length+5 downto 0);
        variable v_t1 : signed(d'length+5 downto 0);
        variable v_t2 : signed(d'length+5 downto 0);
    begin
        v_d  := resize(d, d'length+6);
        v_t1 := (others => '0');
        v_t2 := (others => '0');
        if m(1) = '1' then v_t1 := shift_left(v_d, 1); end if;
        if m(0) = '1' then v_t2 := v_d; end if;
        return v_t1 + v_t2;
    end function;

    -- 512 + a signed offset (|x| <= 511) as a 10-bit code, by SLICING.
    -- (v0.1-0.5 used unsigned(resize(<11-bit signed>, 10)): signed resize
    -- keeps the sign bit + low 9 bits, so every code >= 512 folded down by
    -- 512 -- luma never exceeded 511 and snapped black at each zero
    -- crossing, and U/V were confined below 512.)
    function off512(x : signed(9 downto 0)) return unsigned is
        variable v : signed(10 downto 0);
    begin
        v := resize(x, 11) + to_signed(512, 11);
        return unsigned(v(9 downto 0));
    end function;

begin

    -- ========================================================================
    -- Register Mapping
    -- ========================================================================
    s_k1  <= unsigned(registers_in(0));
    s_k2  <= unsigned(registers_in(1));
    s_k3  <= unsigned(registers_in(2));
    s_k4  <= unsigned(registers_in(3));
    s_k5  <= unsigned(registers_in(4));
    s_k6  <= unsigned(registers_in(5));
    s_s7  <= registers_in(6)(0);
    s_s8  <= registers_in(6)(1);
    s_s9  <= registers_in(6)(2);
    s_s10 <= registers_in(6)(3);
    s_s11 <= registers_in(6)(4);
    s_k12 <= unsigned(registers_in(7));

    -- ========================================================================
    -- Raster measurement, serration-guarded field start, generated avid
    -- ========================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;   -- falling edge of _n

            -- Source line width (avid run length), latched per field below.
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                if s_xcnt > 16 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
            end if;

            -- Field start: only the FIRST vsync edge after active video.
            -- Analog vsync serrates (several edges per field); acting on
            -- every edge would step the drift several times per field.
            s_fstart <= '0';
            if s_avid_r = '1' then
                s_sawact <= '1';
            end if;
            if s_vs_pulse = '1' and s_sawact = '1' then
                s_sawact <= '0';
                s_fstart <= '1';
                s_Wl     <= s_W;
            end if;

            -- CLEAN AVID (from Cubist v0.3.3, HW-confirmed).  The encoders
            -- get no data-enable, so they pair Cb/Cr by counting from hsync,
            -- while the core's 4:4:4->4:2:2 packer restarts on Cb at every
            -- avid rise.  A source avid that wanders a clock against hsync,
            -- or drops out mid-line, swaps U/V for that line.  So: start 2 or
            -- 3 clocks after the source's first avid rise of the line --
            -- whichever keeps the hsync->start distance at the source's USUAL
            -- parity (saturating vote) -- and end after exactly s_Wl pixels,
            -- so dropouts and ragged line ends are ignored.
            if s_hs_r = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if s_avid_r = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_x    <= to_unsigned(1, 12);
            elsif g_avid = '1' then
                if g_x >= s_Wl then
                    g_avid <= '0';
                else
                    g_x <= g_x + 1;
                end if;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Video Timing Generator (fed the generated avid: carriers reset and
    -- LFOs step off the clean line start)
    -- ========================================================================
    timing_gen_inst : entity work.video_timing_generator
        port map (
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => g_avid,
            timing      => s_timing
        );

    -- ========================================================================
    -- Per-frame knob latch + per-frame UV computation
    -- ========================================================================
    p_vsync_latch : process(clk)
        variable v_d20 : signed(19 downto 0);
        variable v_d14 : signed(14 downto 0);
        variable v_h15 : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                r_osc1_base <= s_k1;
                r_osc1_lfo  <= s_k2;
                r_osc1_hue  <= s_k3;          -- colour target (eased below)
                r_osc2_base <= s_k4;
                r_osc2_lfo  <= s_k5;
                r_osc2_hue  <= s_k6;          -- K6 = Sway amount
                r_stacked   <= s_s7;
                r_edges     <= s_s8;
                r_dark      <= s_s9;
                r_flow_same <= s_s10;
                r_tri_en    <= s_s11;
                r_speed     <= s_k12;
            end if;

            -- Sway amount D = K6/2 (0..511)
            r_depth <= '0' & r_osc2_hue(9 downto 1);

            -- Bipolar sway rate target (see the header constants), in 16.4
            -- fixed point: +/- 4|v| << 4.
            if r_speed >= to_unsigned(512, 10) then
                r_vel_mag <= r_speed - to_unsigned(512, 10);
                r_vel_neg <= '0';
            else
                r_vel_mag <= to_unsigned(512, 10) - r_speed;
                r_vel_neg <= '1';
            end if;
            if r_vel_mag < to_unsigned(C_SPEED_DEADBAND, 10) then
                r_rate_tgt <= (others => '0');
            elsif r_vel_neg = '0' then
                r_rate_tgt <= signed(resize(r_vel_mag & "000000", 20));
            else
                r_rate_tgt <= -signed(resize(r_vel_mag & "000000", 20));
            end if;

            -- Once per field: ease the rate toward the target by 1/8 of the
            -- gap (snap inside +/-8 = half a rate LSB), and ease the colour
            -- the same way (10.4 hue, linear -- the knob can't wrap).
            -- Pipelined: the gap (E1) and the eased step + snap test (E2) are
            -- registered every clock, so the field-start update is a plain
            -- mux (both only move once per field, but STA times them at the
            -- pixel clock -- the fused version was the HD critical path).
            v_d20      := r_rate_tgt - r_rate_s;                       -- E1
            r_rate_d   <= v_d20;
            v_d14      := signed('0' & r_osc1_hue & "0000") - signed('0' & r_hue14);
            r_hue_d    <= v_d14;
            r_rate_nxt <= r_rate_s + shift_right(r_rate_d, 3);          -- E2
            if r_rate_d > -8 and r_rate_d < 8 then
                r_rate_snap <= '1';
            else
                r_rate_snap <= '0';
            end if;
            v_h15      := signed('0' & r_hue14) + shift_right(r_hue_d, 3);
            r_hue_nxt  <= unsigned(v_h15(13 downto 0));
            if r_hue_d > -8 and r_hue_d < 8 then
                r_hue_snap <= '1';
            else
                r_hue_snap <= '0';
            end if;
            if s_fstart = '1' then
                if r_rate_snap = '1' then
                    r_rate_s <= r_rate_tgt;
                else
                    r_rate_s <= r_rate_nxt;
                end if;
                if r_hue_snap = '1' then
                    r_hue14 <= r_osc1_hue & "0000";
                else
                    r_hue14 <= r_hue_nxt;
                end if;
            end if;

            -- Register the hue LUT output (free-running; the eased hue only
            -- changes at the field start).
            r_cos1_r <= s_cos1;
            r_sin1_r <= s_sin1;

            -- Chroma at full legal saturation: c - c/8 = +/-448 (the 10-bit
            -- inverse-BT.601 chroma divisor is 896).
            r_du1 <= r_cos1_r - shift_right(r_cos1_r, 3);
            r_dv1 <= r_sin1_r - shift_right(r_sin1_r, 3);

            -- K3 COLOUR: osc1 takes the hue, osc2 its COMPLEMENT (+180 deg =
            -- negated chroma), so the sweep passes through red/cyan,
            -- orange/blue, yellow/violet, green/magenta ...  Free-running;
            -- the inputs only move in vertical blanking.
            r_u1  <= off512(r_du1);
            r_v1  <= off512(r_dv1);
            r_u2  <= off512(-r_du1);
            r_v2  <= off512(-r_dv1);
        end if;
    end process;

    -- ========================================================================
    -- Hue LUT (combinational; angle = eased K3 colour)
    -- ========================================================================
    sin_hue1_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_hue14(13 downto 4)),
            sin_out  => s_sin1,
            cos_out  => s_cos1
        );

    -- ========================================================================
    -- LFO Accumulators (per-line, frame-locked)
    --
    -- Reset at the guarded field start, stepped at the END of each active
    -- line (active lines only -- no interlace twitter), so the NEXT line's
    -- LFO value and PM offset are computed during horizontal blanking.  The
    -- sway pattern is a pure function of line number; temporal motion comes
    -- only from the drift accumulator below.
    -- ========================================================================
    p_lfo_acc : process(clk)
    begin
        if rising_edge(clk) then
            -- step knee: K<512 -> 960 + 4k + 2k ; K>=512 -> 4032 + 16k + 8k
            -- (k = low 9 bits).  Operands registered first, one add after.
            if r_osc1_lfo(9) = '0' then
                r_st1_a <= to_unsigned(C_LFO_FLOOR, 14);
                r_st1_b <= shift_left(resize(r_osc1_lfo(8 downto 0), 14), 2);
                r_st1_c <= shift_left(resize(r_osc1_lfo(8 downto 0), 14), 1);
            else
                r_st1_a <= to_unsigned(C_LFO_KNEE, 14);
                r_st1_b <= shift_left(resize(r_osc1_lfo(8 downto 0), 14), 4);
                r_st1_c <= shift_left(resize(r_osc1_lfo(8 downto 0), 14), 3);
            end if;
            if r_osc2_lfo(9) = '0' then
                r_st2_a <= to_unsigned(C_LFO_FLOOR, 14);
                r_st2_b <= shift_left(resize(r_osc2_lfo(8 downto 0), 14), 2);
                r_st2_c <= shift_left(resize(r_osc2_lfo(8 downto 0), 14), 1);
            else
                r_st2_a <= to_unsigned(C_LFO_KNEE, 14);
                r_st2_b <= shift_left(resize(r_osc2_lfo(8 downto 0), 14), 4);
                r_st2_c <= shift_left(resize(r_osc2_lfo(8 downto 0), 14), 3);
            end if;
            r_lfo1_step <= r_st1_a + r_st1_b + r_st1_c;
            r_lfo2_step <= r_st2_a + r_st2_b + r_st2_c;

            if s_fstart = '1' then
                r_lfo1_acc <= (others => '0');
                r_lfo2_acc <= (others => '0');
            elsif s_timing.avid_end = '1' then
                r_lfo1_acc <= r_lfo1_acc + resize(r_lfo1_step, C_LFO_W);
                r_lfo2_acc <= r_lfo2_acc + resize(r_lfo2_step, C_LFO_W);
            end if;
        end if;
    end process;

    s_lfo1_phase <= r_lfo1_acc;
    s_lfo2_phase <= r_lfo2_acc;

    -- ========================================================================
    -- Per-field sway gain  G = inc * D / step  (see the Sway AMOUNT constants above)
    --   inc <= 34192, D <= 512 -> N < 2^25;  step >= 960 -> G < 2^15.
    -- One serial MSB-first shift-add multiply (10 clocks) then a serial
    -- restoring divide (25 clocks), both oscillators in parallel, started
    -- 16 clocks after the guarded field start (knobs latched by then).
    -- fg_done pulses once G is valid; it also kicks the line multiply so
    -- the first line of the field gets an offset from the fresh G.
    -- ========================================================================
    p_gain : process(clk)
        variable v_r1, v_r2 : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            r_osc1_inc <= shift_left(resize(r_osc1_base, 16), C_FREQ_KNOB_SH)
                          + to_unsigned(C_FREQ_MIN, 16);
            r_osc2_inc <= shift_left(resize(r_osc2_base, 16), C_FREQ_KNOB_SH)
                          + to_unsigned(C_FREQ_MIN, 16);


            fg_done <= '0';
            case fg_st is
                when "00" =>                          -- idle
                    if fg_fin = '1' then
                        fg_fin  <= '0';
                        r_g1    <= fg_q1;
                        r_g2    <= fg_q2;
                        fg_done <= '1';
                    end if;
                    if s_fstart = '1' then
                        fg_cnt <= (others => '0');
                        fg_st  <= "01";
                    end if;
                when "01" =>                          -- settle, then load
                    fg_cnt <= fg_cnt + 1;
                    if fg_cnt = 15 then
                        fg_d   <= r_depth;
                        fg_n1  <= (others => '0');
                        fg_n2  <= (others => '0');
                        fg_cnt <= (others => '0');
                        fg_st  <= "10";
                    end if;
                when "10" =>                          -- N = inc * D
                    if fg_d(9) = '1' then
                        fg_n1 <= (fg_n1(23 downto 0) & '0') + resize(r_osc1_inc, 25);
                        fg_n2 <= (fg_n2(23 downto 0) & '0') + resize(r_osc2_inc, 25);
                    else
                        fg_n1 <= fg_n1(23 downto 0) & '0';
                        fg_n2 <= fg_n2(23 downto 0) & '0';
                    end if;
                    fg_d   <= fg_d(8 downto 0) & '0';
                    fg_cnt <= fg_cnt + 1;
                    if fg_cnt = 9 then
                        fg_r1  <= (others => '0');
                        fg_r2  <= (others => '0');
                        fg_cnt <= (others => '0');
                        fg_st  <= "11";
                    end if;
                when others =>                        -- G = N / step
                    v_r1 := fg_r1 & fg_n1(24);
                    v_r2 := fg_r2 & fg_n2(24);
                    if v_r1 >= resize(r_lfo1_step, 15) then
                        fg_r1 <= resize(v_r1 - resize(r_lfo1_step, 15), 14);
                        fg_q1 <= fg_q1(14 downto 0) & '1';
                    else
                        fg_r1 <= v_r1(13 downto 0);
                        fg_q1 <= fg_q1(14 downto 0) & '0';
                    end if;
                    if v_r2 >= resize(r_lfo2_step, 15) then
                        fg_r2 <= resize(v_r2 - resize(r_lfo2_step, 15), 14);
                        fg_q2 <= fg_q2(14 downto 0) & '1';
                    else
                        fg_r2 <= v_r2(13 downto 0);
                        fg_q2 <= fg_q2(14 downto 0) & '0';
                    end if;
                    fg_n1  <= fg_n1(23 downto 0) & '0';
                    fg_n2  <= fg_n2(23 downto 0) & '0';
                    fg_cnt <= fg_cnt + 1;
                    if fg_cnt = 24 then
                        fg_fin <= '1';
                        fg_st  <= "00";
                    end if;
            end case;
        end if;
    end process;

    -- ========================================================================
    -- Sway Drift Accumulator (per-field, signed rate from the bipolar
    -- Speed slider; a negative rate is 2^16-|rate|, which decrements the
    -- phase via the natural mod-2^16 wrap).  Slider centre = rate 0 = the
    -- accumulator holds its phase, so the picture freezes in place and
    -- resumes without a pop.  Steps on s_fstart (the first vsync edge after
    -- active video), NOT every vsync edge: the SDK frame_phase_accumulator
    -- steps per edge, which on a serrated analog vsync ran the flow several
    -- times too fast.
    -- ========================================================================
    p_drift : process(clk)
    begin
        if rising_edge(clk) then
            if s_fstart = '1' then
                s_drift_phase <= s_drift_phase + unsigned(r_rate_s);
            end if;
        end if;
    end process;

    -- Register LFO angle = phase +/- drift (keeps the add out of the sine
    -- LUT path).  Osc1 always adds the drift (follows the slider); osc2
    -- adds it too when S10 = Same, subtracts when S10 = Opposite
    -- (counter-flow).  One extra clk on the LFO side chain only; the LFO
    -- value is line-stable so the pixel path is unaffected.
    p_lfo_angle_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_lfo1_angle <= s_lfo1_phase(C_LFO_W-1 downto C_LFO_W-16)
                          + s_drift_phase(C_DRIFT_W-1 downto C_DRIFT_W-16);
            if r_flow_same = '1' then
                r_lfo2_angle <= s_lfo2_phase(C_LFO_W-1 downto C_LFO_W-16)
                              + s_drift_phase(C_DRIFT_W-1 downto C_DRIFT_W-16);
            else
                r_lfo2_angle <= s_lfo2_phase(C_LFO_W-1 downto C_LFO_W-16)
                              - s_drift_phase(C_DRIFT_W-1 downto C_DRIFT_W-16);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- LFO sine/cosine LUTs (top 10 bits of the 16-bit angle)
    -- ========================================================================
    sin_lfo1_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_lfo1_angle(15 downto 6)),
            sin_out  => s_lfo1_sin,
            cos_out  => s_lfo1_cos
        );
    sin_lfo2_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_lfo2_angle(15 downto 6)),
            sin_out  => s_lfo2_sin,
            cos_out  => s_lfo2_cos
        );

    s_lfo1_tri <= tri_fold(r_lfo1_angle(15 downto 6));
    s_lfo2_tri <= tri_fold(r_lfo2_angle(15 downto 6));

    -- Register LUT outputs twice (the first is absorbed into the EBR), with
    -- the triangle, its slope direction and the 6-bit angle fraction.
    p_lfo_sin_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_lfo1_sin_r <= s_lfo1_sin;
            r_lfo2_sin_r <= s_lfo2_sin;
            r_lfo1_cos_r <= s_lfo1_cos;
            r_lfo2_cos_r <= s_lfo2_cos;
            r_lfo1_tri_r <= s_lfo1_tri;
            r_lfo2_tri_r <= s_lfo2_tri;
            -- triangle rises (+2 per step) in quadrants 0 and 3
            r_lfo1_up_r  <= not (r_lfo1_angle(15) xor r_lfo1_angle(14));
            r_lfo2_up_r  <= not (r_lfo2_angle(15) xor r_lfo2_angle(14));
            r_lfo1_fr_r  <= r_lfo1_angle(5 downto 0);
            r_lfo2_fr_r  <= r_lfo2_angle(5 downto 0);
            r_lfo1_sin_r2 <= r_lfo1_sin_r;
            r_lfo2_sin_r2 <= r_lfo2_sin_r;
            r_lfo1_cos_r2 <= r_lfo1_cos_r;
            r_lfo2_cos_r2 <= r_lfo2_cos_r;
            r_lfo1_tri_r2 <= r_lfo1_tri_r;
            r_lfo2_tri_r2 <= r_lfo2_tri_r;
            r_lfo1_up_r2  <= r_lfo1_up_r;
            r_lfo2_up_r2  <= r_lfo2_up_r;
            r_lfo1_fr_r2  <= r_lfo1_fr_r;
            r_lfo2_fr_r2  <= r_lfo2_fr_r;
        end if;
    end process;

    -- ========================================================================
    -- LFO interpolation (line-rate; units of 1/64 of a table step).
    --   sin_f = 64*sin + cos*f*2pi/1024  ~= 64*sin + (p>>8)+(p>>9)+(p>>12),
    --           p = cos*f  (2pi/1024 = 0.006136; the shifts give 0.006104)
    --   tri_f = 64*tri +/- 2f
    --   I1: partial products of cos*f   I2: p, triangle delta
    --   I3: 64*sin + p>>8, p>>9 + p>>12, 64*tri + delta   I4: sin_f
    -- ========================================================================
    p_lfo_interp : process(clk)
        function pp_hi(c : signed(9 downto 0); f : unsigned(5 downto 0)) return signed is
            variable v_c : signed(16 downto 0);
            variable v   : signed(16 downto 0);
        begin
            v_c := resize(c, 17);
            v   := (others => '0');
            if f(5) = '1' then v := v + shift_left(v_c, 5); end if;
            if f(4) = '1' then v := v + shift_left(v_c, 4); end if;
            if f(3) = '1' then v := v + shift_left(v_c, 3); end if;
            return v;
        end function;
        function pp_lo(c : signed(9 downto 0); f : unsigned(5 downto 0)) return signed is
            variable v_c : signed(16 downto 0);
            variable v   : signed(16 downto 0);
        begin
            v_c := resize(c, 17);
            v   := (others => '0');
            if f(2) = '1' then v := v + shift_left(v_c, 2); end if;
            if f(1) = '1' then v := v + shift_left(v_c, 1); end if;
            if f(0) = '1' then v := v + v_c; end if;
            return v;
        end function;
    begin
        if rising_edge(clk) then
            -- I1
            r_i1_pa <= pp_hi(r_lfo1_cos_r2, r_lfo1_fr_r2);
            r_i1_pb <= pp_lo(r_lfo1_cos_r2, r_lfo1_fr_r2);
            r_i2_pa <= pp_hi(r_lfo2_cos_r2, r_lfo2_fr_r2);
            r_i2_pb <= pp_lo(r_lfo2_cos_r2, r_lfo2_fr_r2);
            if r_lfo1_up_r2 = '1' then
                r_i1_td <= signed(resize(r_lfo1_fr_r2 & '0', 9));
            else
                r_i1_td <= -signed(resize(r_lfo1_fr_r2 & '0', 9));
            end if;
            if r_lfo2_up_r2 = '1' then
                r_i2_td <= signed(resize(r_lfo2_fr_r2 & '0', 9));
            else
                r_i2_td <= -signed(resize(r_lfo2_fr_r2 & '0', 9));
            end if;
            r_i1_s <= r_lfo1_sin_r2;  r_i1_t <= r_lfo1_tri_r2;
            r_i2_s <= r_lfo2_sin_r2;  r_i2_t <= r_lfo2_tri_r2;
            -- I2
            r_i1_p <= r_i1_pa + r_i1_pb;
            r_i2_p <= r_i2_pa + r_i2_pb;
            r_i1_td2 <= r_i1_td;  r_i2_td2 <= r_i2_td;
            r_i1_s2 <= r_i1_s;  r_i1_t2 <= r_i1_t;
            r_i2_s2 <= r_i2_s;  r_i2_t2 <= r_i2_t;
            -- I3
            r_i1_a  <= shift_left(resize(r_i1_s2, 17), 6) + shift_right(r_i1_p, 8);
            r_i1_b  <= shift_right(r_i1_p, 9) + shift_right(r_i1_p, 12);
            r_i2_a  <= shift_left(resize(r_i2_s2, 17), 6) + shift_right(r_i2_p, 8);
            r_i2_b  <= shift_right(r_i2_p, 9) + shift_right(r_i2_p, 12);
            r_i1_tf <= shift_left(resize(r_i1_t2, 17), 6) + resize(r_i1_td2, 17);
            r_i2_tf <= shift_left(resize(r_i2_t2, 17), 6) + resize(r_i2_td2, 17);
            -- I4
            r_i1_sf  <= r_i1_a + r_i1_b;
            r_i2_sf  <= r_i2_a + r_i2_b;
            r_i1_tf2 <= r_i1_tf;
            r_i2_tf2 <= r_i2_tf;
        end if;
    end process;

    -- ========================================================================
    -- Shape glide factor (S11).  Eases one step per field toward 32
    -- (triangle) or 0 (sine) -- 32 fields end to end.  Steps on the guarded
    -- field start, so it is frame-stable and serration-proof.
    -- ========================================================================
    p_glide : process(clk)
    begin
        if rising_edge(clk) then
            if s_fstart = '1' then
                if r_tri_en = '1' and r_glide /= 32 then
                    r_glide <= r_glide + 1;
                elsif r_tri_en = '0' and r_glide /= 0 then
                    r_glide <= r_glide - 1;
                end if;
            end if;
        end if;
    end process;

    -- LFO waveshape glide on the fine values (A: diff, B: partial sums,
    -- C: sum -> r_lfo*_fine).  Line-stable, so the stages only delay it
    -- inside horizontal blanking (the line multiply waits for it).
    p_lfo_reg : process(clk)
        variable v1, v2 : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            r_lf1_gd  <= resize(r_i1_tf2, 18) - resize(r_i1_sf, 18);
            r_lf2_gd  <= resize(r_i2_tf2, 18) - resize(r_i2_sf, 18);
            r_lf1_gs  <= r_i1_sf;
            r_lf2_gs  <= r_i2_sf;
            r_lf1_gh  <= glide_hi_g(r_lf1_gd, r_glide);
            r_lf2_gh  <= glide_hi_g(r_lf2_gd, r_glide);
            r_lf1_gl  <= glide_lo_g(r_lf1_gd, r_glide);
            r_lf2_gl  <= glide_lo_g(r_lf2_gd, r_glide);
            r_lf1_gs2 <= r_lf1_gs;
            r_lf2_gs2 <= r_lf2_gs;
            v1 := resize(r_lf1_gs2, 24) + shift_right(r_lf1_gh + r_lf1_gl, 5);
            v2 := resize(r_lf2_gs2, 24) + shift_right(r_lf2_gh + r_lf2_gl, 5);
            r_lfo1_fine <= v1(16 downto 0);
            r_lfo2_fine <= v2(16 downto 0);
        end if;
    end process;

    -- ========================================================================
    -- Per-line PM offset = lfo_fine * G / 32768 (mod 2048) -- lfo_fine is in
    -- 1/64 table steps -- serial MSB-first shift-add over |lfo_fine| (16
    -- clocks).  Kicked 16 clocks after each active line ends (the stepped
    -- LFO settles through angle reg, LUT, 2 regs, 4 interpolation and 3
    -- glide stages in 11) and on fg_done at the field start.
    -- Done ~34 clocks into horizontal blanking (138 clocks even in SD), so
    -- the offset never changes mid-line.
    -- ========================================================================
    p_line_mul : process(clk)
        variable v_l1, v_l2 : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            r_ae_sr <= r_ae_sr(14 downto 0) & s_timing.avid_end;
            lm_fin  <= '0';
            if r_ae_sr(15) = '1' or fg_done = '1' then
                v_l1 := resize(r_lfo1_fine, 18);
                v_l2 := resize(r_lfo2_fine, 18);
                if v_l1 < 0 then v_l1 := -v_l1; end if;
                if v_l2 < 0 then v_l2 := -v_l2; end if;
                lm_m1  <= unsigned(v_l1(15 downto 0));
                lm_m2  <= unsigned(v_l2(15 downto 0));
                lm_h1  <= unsigned(v_l1(14 downto 6));
                lm_h2  <= unsigned(v_l2(14 downto 6));
                lm_s1  <= r_lfo1_fine(16);
                lm_s2  <= r_lfo2_fine(16);
                lm_a1  <= (others => '0');
                lm_a2  <= (others => '0');
                lm_cnt <= (others => '0');
                lm_run <= '1';
            elsif lm_run = '1' then
                if lm_m1(15) = '1' then
                    lm_a1 <= (lm_a1(24 downto 0) & '0') + resize(r_g1, 26);
                else
                    lm_a1 <= lm_a1(24 downto 0) & '0';
                end if;
                if lm_m2(15) = '1' then
                    lm_a2 <= (lm_a2(24 downto 0) & '0') + resize(r_g2, 26);
                else
                    lm_a2 <= lm_a2(24 downto 0) & '0';
                end if;
                lm_m1  <= lm_m1(14 downto 0) & '0';
                lm_m2  <= lm_m2(14 downto 0) & '0';
                lm_cnt <= lm_cnt + 1;
                if lm_cnt = 15 then
                    lm_run <= '0';
                    lm_fin <= '1';
                end if;
            end if;
            if lm_fin = '1' then
                r_hl1 <= lm_h1;  r_ls1 <= lm_s1;
                r_hl2 <= lm_h2;  r_ls2 <= lm_s2;
                if lm_s1 = '1' then
                    r_lfo1_off <= to_unsigned(0, 11) - lm_a1(25 downto 15);
                else
                    r_lfo1_off <= lm_a1(25 downto 15);
                end if;
                if lm_s2 = '1' then
                    r_lfo2_off <= to_unsigned(0, 11) - lm_a2(25 downto 15);
                else
                    r_lfo2_off <= lm_a2(25 downto 15);
                end if;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Carrier Accumulators (per-pixel, reset at line start, C_HORIZONTAL)
    --
    -- inc = base << C_FREQ_KNOB_SH.  We do NOT FM the increment;
    -- modulation is applied as a PHASE OFFSET below, which gives smoother
    -- waveforms than true FM.
    -- ========================================================================
    s_osc1_inc_slv <= std_logic_vector(shift_left(resize(r_osc1_base, C_CARRIER_W), C_FREQ_KNOB_SH_C)
                                        + to_unsigned(C_FREQ_MIN_C, C_CARRIER_W));
    s_osc2_inc_slv <= std_logic_vector(shift_left(resize(r_osc2_base, C_CARRIER_W), C_FREQ_KNOB_SH_C)
                                        + to_unsigned(C_FREQ_MIN_C, C_CARRIER_W));

    osc1_inst : entity work.video_timing_accumulator
        generic map (G_ACCUMULATOR_WIDTH => C_CARRIER_W)
        port map (
            clk           => clk,
            i_timing      => s_timing,
            i_range       => C_HORIZONTAL,
            i_reset       => '0',
            i_lock        => '1',
            i_accumulator => s_osc1_inc_slv,
            o_accumulator => s_osc1_acc_slv,
            o_clock       => open,
            o_pulse       => open
        );

    osc2_inst : entity work.video_timing_accumulator
        generic map (G_ACCUMULATOR_WIDTH => C_CARRIER_W)
        port map (
            clk           => clk,
            i_timing      => s_timing,
            i_range       => C_HORIZONTAL,
            i_reset       => '0',
            i_lock        => '1',
            i_accumulator => s_osc2_inc_slv,
            o_accumulator => s_osc2_acc_slv,
            o_clock       => open,
            o_pulse       => open
        );

    -- ========================================================================
    -- PM Phase Computation
    --
    -- PM_phase[11b] = carrier_acc[top 11b] + PM offset (both modulo 2048).
    -- The low 10 bits address the sine LUT; bit 10 counts stripes, so its
    -- parity (taken at the trough, phase + 256) tells odd lines from even.
    -- ========================================================================
    s_osc1_pm_phase <= unsigned(s_osc1_acc_slv(C_CARRIER_W-1 downto C_CARRIER_W-11))
                     + r_lfo1_off;
    s_osc2_pm_phase <= unsigned(s_osc2_acc_slv(C_CARRIER_W-1 downto C_CARRIER_W-11))
                     + r_lfo2_off;

    p_pm_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_osc1_pm_phase <= s_osc1_pm_phase;
            r_osc2_pm_phase <= s_osc2_pm_phase;

            -- stripe parity = bit 10 of (phase + 256); flank side = bit 8
            r_par1_sr <= r_par1_sr(4 downto 0)
                         & (r_osc1_pm_phase(10) xor (r_osc1_pm_phase(9) and r_osc1_pm_phase(8)));
            r_par2_sr <= r_par2_sr(4 downto 0)
                         & (r_osc2_pm_phase(10) xor (r_osc2_pm_phase(9) and r_osc2_pm_phase(8)));
            r_side1_sr <= r_side1_sr(3 downto 0) & r_osc1_pm_phase(8);
            r_side2_sr <= r_side2_sr(3 downto 0) & r_osc2_pm_phase(8);
        end if;
    end process;

    -- ========================================================================
    -- Carrier Sine LUTs (per-pixel; registered PM phase -> sin)
    -- ========================================================================
    sin_osc1_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_osc1_pm_phase(9 downto 0)),
            sin_out  => s_osc1_sin,
            cos_out  => open
        );
    sin_osc2_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_osc2_pm_phase(9 downto 0)),
            sin_out  => s_osc2_sin,
            cos_out  => open
        );

    -- Triangle wave of registered PM phase (combinational, free).
    s_osc1_tri <= tri_fold(r_osc1_pm_phase(9 downto 0));
    s_osc2_tri <= tri_fold(r_osc2_pm_phase(9 downto 0));

    -- Register raw sin LUT output and triangle.  Splits the slow LUT path
    -- from the morph path so HD timing closes.
    p_carrier_sin_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_osc1_sin_r <= s_osc1_sin;
            r_osc2_sin_r <= s_osc2_sin;
            r_osc1_tri_r <= s_osc1_tri;
            r_osc2_tri_r <= s_osc2_tri;
            r_osc1_sin_r2 <= r_osc1_sin_r;
            r_osc2_sin_r2 <= r_osc2_sin_r;
            r_osc1_tri_r2 <= r_osc1_tri_r;
            r_osc2_tri_r2 <= r_osc2_tri_r;
        end if;
    end process;

    -- ========================================================================
    -- Carrier waveshape glide + luma (per pixel)
    --   A: d = tri - sin      B: partial sums of d*m      C: sin + sum/32
    --   luma = shaped + 512, landing in 1..1023 (full 10-bit range)
    -- ========================================================================
    p_luma_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_osc1_gd  <= resize(r_osc1_tri_r2, 11) - resize(r_osc1_sin_r2, 11);
            r_osc2_gd  <= resize(r_osc2_tri_r2, 11) - resize(r_osc2_sin_r2, 11);
            r_osc1_gs  <= r_osc1_sin_r2;
            r_osc2_gs  <= r_osc2_sin_r2;
            r_osc1_gh  <= glide_hi(r_osc1_gd, r_glide);
            r_osc2_gh  <= glide_hi(r_osc2_gd, r_glide);
            r_osc1_gl  <= glide_lo(r_osc1_gd, r_glide);
            r_osc2_gl  <= glide_lo(r_osc2_gd, r_glide);
            r_osc1_gs2 <= r_osc1_gs;
            r_osc2_gs2 <= r_osc2_gs;
            r_luma1 <= off512(glide_sum(r_osc1_gs2, r_osc1_gh, r_osc1_gl));
            r_luma2 <= off512(glide_sum(r_osc2_gs2, r_osc2_gh, r_osc2_gl));
        end if;
    end process;

    -- ========================================================================
    -- S8 EDGES (per oscillator, one stage).  Each line's flanks -- the band
    -- where its luma is 576..831 (sin 0.125..0.625, ~1/11 of a period per
    -- side) -- become an OUTLINE: luma halved, chroma kept at full
    -- saturation (rim flag).  The rim on the OUTER side of each bend also
    -- gets a lighter accent of |lfo|/64 (up to +511 luma, strongest at the
    -- sway's extremes where the curve is tightest).
    -- Outer side: a positive PM offset moves a line LEFT, so with lfo >= 0
    -- the outer rim is the left (rising, phase bit 8 = 0) flank, and with
    -- lfo < 0 the right one.
    -- ========================================================================
    p_edges : process(clk)
        variable v_rim1, v_rim2 : boolean;
    begin
        if rising_edge(clk) then
            v_rim1 := r_edges = '1' and r_luma1 >= to_unsigned(576, 10)
                                    and r_luma1 <  to_unsigned(832, 10);
            v_rim2 := r_edges = '1' and r_luma2 >= to_unsigned(576, 10)
                                    and r_luma2 <  to_unsigned(832, 10);
            r_eu1 <= r_u1;  r_ev1 <= r_v1;
            r_eu2 <= r_u2;  r_ev2 <= r_v2;
            if v_rim1 then
                r_eR1 <= '1';
                if (r_side1_sr(4) xor r_ls1) = '0' then
                    r_eL1 <= shift_right(r_luma1, 1) + resize(r_hl1, 10);
                else
                    r_eL1 <= shift_right(r_luma1, 1);
                end if;
            else
                r_eR1 <= '0';
                r_eL1 <= r_luma1;
            end if;
            if v_rim2 then
                r_eR2 <= '1';
                if (r_side2_sr(4) xor r_ls2) = '0' then
                    r_eL2 <= shift_right(r_luma2, 1) + resize(r_hl2, 10);
                else
                    r_eL2 <= shift_right(r_luma2, 1);
                end if;
            else
                r_eR2 <= '0';
                r_eL2 <= r_luma2;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Layering keyer (S7) -- keeps BOTH lines intact:
    --   Woven   over/under ALTERNATES at each crossing: the upper line is
    --           osc1 when the two stripes' parities match, else osc2 -- a
    --           checkerboard in (stripe1, stripe2) = a plain weave.
    --   Stacked osc1 always over osc2.
    -- Layering is CONTINUOUS: out = max(T, B*m/32), where the lower wave's
    -- gain m/32 fades 1 -> 0 (32 steps) as the upper wave T rises from 256
    -- to 768 -- the lower line dims as it passes under, with no hard edge to
    -- crawl as the pattern moves.  4 stages: select, partial sums, scale,
    -- max.  Also carries the crossing strength x to the reaction.
    -- ========================================================================
    p_keyer : process(clk)
        variable v_swap : boolean;
        variable v_lt, v_lb : unsigned(9 downto 0);
        variable v_ut, v_ub : unsigned(9 downto 0);
        variable v_vt, v_vb : unsigned(9 downto 0);
        variable v_min      : unsigned(9 downto 0);
        variable v_bs       : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            -- K1: pick upper (T) / lower (B) and the lower wave's gain m/32
            v_swap := r_stacked = '0' and (r_par1_sr(5) xor r_par2_sr(5)) = '1';
            if v_swap then
                v_lt := r_eL2; v_ut := r_eu2; v_vt := r_ev2;
                v_lb := r_eL1; v_ub := r_eu1; v_vb := r_ev1;
                r_kRT <= r_eR2;  r_kRB <= r_eR1;
            else
                v_lt := r_eL1; v_ut := r_eu1; v_vt := r_ev1;
                v_lb := r_eL2; v_ub := r_eu2; v_vb := r_ev2;
                r_kRT <= r_eR1;  r_kRB <= r_eR2;
            end if;
            r_kT <= v_lt;  r_kuT <= v_ut;  r_kvT <= v_vt;
            r_kB <= v_lb;  r_kuB <= v_ub;  r_kvB <= v_vb;
            if v_lt(9 downto 8) = "00" then
                r_km <= to_unsigned(32, 6);                  -- T < 256: full
            elsif v_lt(9 downto 8) = "11" then
                r_km <= (others => '0');                     -- T >= 768: hidden
            else
                -- T = 256..767: m = 32 - (T-256)/16
                r_km <= to_unsigned(32, 6) - resize(v_lt(9) & v_lt(7 downto 4), 6);
            end if;
            if r_eL1 < r_eL2 then v_min := r_eL1; else v_min := r_eL2; end if;
            if v_min(9) = '1' then
                r_cx0 <= v_min(8 downto 0);
            else
                r_cx0 <= (others => '0');
            end if;

            -- K2: partial sums of B*m
            r_kh   <= glide_hi(signed('0' & r_kB), r_km);
            r_kl   <= glide_lo(signed('0' & r_kB), r_km);
            r_kT2  <= r_kT;
            r_kuT2 <= r_kuT;  r_kvT2 <= r_kvT;
            r_kuB2 <= r_kuB;  r_kvB2 <= r_kvB;
            r_cx1  <= r_cx0;
            r_kRT2 <= r_kRT;  r_kRB2 <= r_kRB;

            -- K3: Bs = B*m/32
            v_bs   := shift_right(r_kh + r_kl, 5);
            r_kBs  <= unsigned(v_bs(9 downto 0));
            r_kT3  <= r_kT2;
            r_kuT3 <= r_kuT2;  r_kvT3 <= r_kvT2;
            r_kuB3 <= r_kuB2;  r_kvB3 <= r_kvB2;
            r_cx2  <= r_cx1;
            r_kRT3 <= r_kRT2;  r_kRB3 <= r_kRB2;

            -- K4: out = max(T, Bs) -- continuous: no hard key edges and no
            -- seam where Weave's upper/lower swap (the swap happens at a
            -- wave's trough, where it is ~0 and loses the max anyway)
            if r_kT3 >= r_kBs then
                r_y_k <= r_kT3;  r_u_k <= r_kuT3;  r_v_k <= r_kvT3;
                r_rim_k <= r_kRT3;
            else
                r_y_k <= r_kBs;  r_u_k <= r_kuB3;  r_v_k <= r_kvB3;
                r_rim_k <= r_kRB3;
            end if;
            -- crossing strength, gated to active video so no reaction tail
            -- leaks from blanking into a line start
            if s_avid_sr(C_DELAY_CLKS-9) = '1' then
                r_cx <= r_cx2;
            else
                r_cx <= (others => '0');
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Analog crossing reaction (see C_RING_DECAY)
    --   R1: peak-hold e (decays C_RING_DECAY/px) + ring phase counter
    --   R2: ring magnitude from e and the counter (+/- e/8, 3e/8 over 8 px)
    --   R3: +/- ring
    --   R4: clamp 0..1023
    -- ========================================================================
    p_react : process(clk)
        variable v_ed  : unsigned(8 downto 0);
        variable v_su  : unsigned(10 downto 0);
        variable v_sv  : unsigned(10 downto 0);
        variable v_ys  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- R1
            if r_e > to_unsigned(C_RING_DECAY, 9) then
                v_ed := r_e - to_unsigned(C_RING_DECAY, 9);
            else
                v_ed := (others => '0');
            end if;
            if r_cx >= v_ed then
                r_e  <= r_cx;
                r_rc <= (others => '0');
            else
                r_e  <= v_ed;
                r_rc <= r_rc + 1;
            end if;
            r_y_1 <= r_y_k;
            r_u_1  <= r_u_k;
            r_v_1  <= r_v_k;
            r_rim1 <= r_rim_k;

            -- R2
            if r_rc(1 downto 0) = "01" or r_rc(1 downto 0) = "10" then
                r_rmag <= resize(shift_right(r_e, 2), 8) + resize(shift_right(r_e, 3), 8);
            else
                r_rmag <= resize(shift_right(r_e, 3), 8);
            end if;
            r_rneg <= r_rc(2);
            r_ya  <= '0' & r_y_1;
            r_u_2 <= r_u_1;  r_v_2 <= r_v_1;  r_rim2 <= r_rim1;

            -- R3
            if r_rneg = '1' then
                r_ys <= signed(resize(r_ya, 13)) - signed(resize(r_rmag, 13));
            else
                r_ys <= signed(resize(r_ya, 13)) + signed(resize(r_rmag, 13));
            end if;
            r_u_3 <= r_u_2;  r_v_3 <= r_v_2;  r_rim3 <= r_rim2;

            -- R4
            v_ys := r_ys;
            if v_ys < 0 then
                r_y_r <= (others => '0');
            elsif v_ys > 1023 then
                r_y_r <= (others => '1');
            else
                r_y_r <= unsigned(v_ys(9 downto 0));
            end if;
            r_u_4 <= r_u_3;  r_v_4 <= r_v_3;  r_rim4 <= r_rim3;
        end if;
    end process;

    -- ========================================================================
    -- Luma-scaled chroma: the chroma excursion fades with the output luma,
    -- so the dark gaps between lines go black instead of carrying the
    -- winning line's full colour, and the lines glow out of them.
    --   L >= 512 (upper half of the wave) : full chroma
    --   L <  512                          : chroma * L(8:5)/16
    -- c = code - 512 is just the MSB inverted; c*n = sum of shifted terms
    -- over the bits of n, two adds per stage.
    -- ========================================================================
    p_cscale : process(clk)
        variable v_cu, v_cv : signed(13 downto 0);
        variable v_n        : unsigned(3 downto 0);
        variable v_su, v_sv : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 1: partial products
            v_cu := resize(signed((not r_u_4(9)) & r_u_4(8 downto 0)), 14);
            v_cv := resize(signed((not r_v_4(9)) & r_v_4(8 downto 0)), 14);
            v_n  := r_y_r(8 downto 5);
            r_ua <= (others => '0');
            r_ub <= (others => '0');
            r_va <= (others => '0');
            r_vb <= (others => '0');
            if v_n(3) = '1' and v_n(2) = '1' then
                r_ua <= shift_left(v_cu, 3) + shift_left(v_cu, 2);
                r_va <= shift_left(v_cv, 3) + shift_left(v_cv, 2);
            elsif v_n(3) = '1' then
                r_ua <= shift_left(v_cu, 3);
                r_va <= shift_left(v_cv, 3);
            elsif v_n(2) = '1' then
                r_ua <= shift_left(v_cu, 2);
                r_va <= shift_left(v_cv, 2);
            end if;
            if v_n(1) = '1' and v_n(0) = '1' then
                r_ub <= shift_left(v_cu, 1) + v_cu;
                r_vb <= shift_left(v_cv, 1) + v_cv;
            elsif v_n(1) = '1' then
                r_ub <= shift_left(v_cu, 1);
                r_vb <= shift_left(v_cv, 1);
            elsif v_n(0) = '1' then
                r_ub <= v_cu;
                r_vb <= v_cv;
            end if;
            r_cfull <= r_y_r(9) or r_rim4;
            r_y_k2  <= r_y_r;
            r_u_k2  <= r_u_4;
            r_v_k2  <= r_v_4;

            -- stage 2: sum, /16, back to offset-512 codes
            r_y_c <= r_y_k2;
            if r_cfull = '1' then
                r_u_c <= r_u_k2;
                r_v_c <= r_v_k2;
            else
                v_su := shift_right(r_ua + r_ub, 4);   -- |.| <= 15*511/16
                v_sv := shift_right(r_va + r_vb, 4);
                r_u_c <= off512(v_su(9 downto 0));
                r_v_c <= off512(v_sv(9 downto 0));
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Output stage: luma remap + blanking gate
    --   y = 64 + L*(1/2 + 1/8 + 1/16) = 64..765 for L = 0..1023 -- black to
    --   ~75%: full-scale Y clips every RGB channel and kills the chroma.
    --   S9 Darker: y = 64 + L*(1/2 + 1/64) = 64..592 (~60%).  Chroma was
    --   scaled from the undimmed L, so darker reads richer, not duller.
    --   Outside the (latency-aligned, generated) avid: y/u/v = 64/512/512.
    --   The gate taps one stage before the output tap, so it lines up with
    --   data_out.avid.
    -- ========================================================================
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_sr(C_DELAY_CLKS-2) = '1' then
                if r_dark = '1' then
                    -- Darker: 64 + L*(1/2 + 1/64) = 64..592 (~60%)
                    r_y_out <= to_unsigned(64, 10) + shift_right(r_y_c, 1)
                             + shift_right(r_y_c, 6);
                else
                    r_y_out <= to_unsigned(64, 10) + shift_right(r_y_c, 1)
                             + shift_right(r_y_c, 3) + shift_right(r_y_c, 4);
                end if;
                r_u_out <= r_u_c;
                r_v_out <= r_v_c;
            else
                r_y_out <= to_unsigned(64, 10);
                r_u_out <= to_unsigned(512, 10);
                r_v_out <= to_unsigned(512, 10);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Sync Delay (generated avid + data_in syncs, C_DELAY_CLKS in ALL modes)
    -- ========================================================================
    p_sync_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= g_avid          & s_avid_sr   (0 to C_DELAY_CLKS-2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_DELAY_CLKS-2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_DELAY_CLKS-2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_DELAY_CLKS-2);
        end if;
    end process;

    -- ========================================================================
    -- Output assignment
    -- ========================================================================
    data_out.y       <= std_logic_vector(r_y_out);
    data_out.u       <= std_logic_vector(r_u_out);
    data_out.v       <= std_logic_vector(r_v_out);
    data_out.avid    <= s_avid_sr   (C_DELAY_CLKS-1);
    data_out.hsync_n <= s_hsync_n_sr(C_DELAY_CLKS-1);
    data_out.vsync_n <= s_vsync_n_sr(C_DELAY_CLKS-1);
    data_out.field_n <= s_field_n_sr(C_DELAY_CLKS-1);

end architecture bajaweave;
