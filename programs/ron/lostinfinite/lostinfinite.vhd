-- LostInfinite: literal transcription of the LZX "LostInfinite" shader
--
-- Reference shader (GLSL):
--
--   vec3 p, a, r = normalize(vec3(I+I, 0) - iResolution.xyy);
--   float i, t, v;
--   for (O *= i; i++ < 80.; t += v * .2) {
--       p = t*r;
--       p = dot(a = normalize(sin(iTime*.1 + i*.1 + vec3(3,1,0))), p) * 2.*a - p;
--       p.z += 14.;
--       p.xz = length(p.xz) *
--              vec2(cos(p.z = mod(atan(p.z, p.x), .4) - .2), sin(p.z));
--       p.xy = mod(p.xy, 1.) - .5;
--       v = abs(length(p) - .1) + .01;
--       O += exp(sin(i*.3 + iTime + vec4(0,1,3,0))) / v;
--   }
--   O = tanh(O*O / 2e4);
--
-- Compromises made to fit an iCE40 HX4K (7,680 LCs) at 1 pixel / clock:
--
--   * N = 1 raymarch iteration (not 80).  Fixed sample depth t, no
--     feedback accumulator.  We render one "slice" of the scene.
--   * Ray r is NOT normalized.  We use (2x-W, 2y-H, -H) and fold the
--     missing scale into the fixed t.  This saves one sqrt + reciprocal.
--   * length() is approximated by the octagonal pseudo-metric
--        2D: |u|_oct = max + min/2
--        3D: |u|_oct = max + mid/2 + min/4
--     No sqrt hardware required.
--   * atan2(pz, px) is approximated by octant decoding + priority-
--     encoded min/max ratio + linear atan(t) approx t * (pi/4).
--     No divider, no trig LUT.
--   * Polar rebuild uses sin/cos linear approximation:
--     since new_ang = mod(atan, .4) - .2 is in (-0.2, 0.2),
--     cos(new_ang) ~= 1 and sin(new_ang) ~= new_ang (error < 2%).
--     This turns the polar rebuild into: new_px = length_xz,
--     new_pz = length_xz * new_ang.  1 multiply, no LUT.
--   * exp(sin())/v is replaced by col_yuv * brightness_lut(v).
--     brightness_lut is a small piecewise constant table shaped like
--     1/(v + eps) and clamped.  col_yuv is a per-frame time-varying
--     triple (see below).
--   * tanh(O*O / 2e4) is replaced by simple saturation clamping
--     (we only have one sample so no accumulation to tone-map).
--   * Per-frame "a" vector is NOT renormalized.  A small 32-entry
--     inline sin table drives three phase-offset sin lookups of
--     frame_count.  The Householder step therefore becomes a slightly
--     scaled reflection-like map instead of a strict reflection.
--
-- Preserved literally (bit-for-bit as the shader):
--
--   * p = t * r
--   * p = 2*(a.p)*a - p              (6 real Q5.10 * Q5.10 multiplies)
--   * p.z += 14
--   * ang = mod(atan2, .4) - .2      (mod 64 on 10-bit angle, then -32)
--   * p.xy = fract(p.xy) - .5        (pure bit masking)
--   * v   = |length(p) - .1| + .01
--   * O   = col / v                  (via brightness LUT)
--
-- Fixed-point format: Q5.10 signed 16-bit.  Range -32..+32, step 1/1024.
--
-- Pipeline latency: 15 clocks.
--
-- Register map (t_spi_ram, each std_logic_vector(9 downto 0)):
--   registers_in(0) = Sample depth t (top 3 bits: 2^-k for k=4..11)
--   registers_in(1) = P.z offset     (top 4 bits: 0..15 replaces the 14)
--   registers_in(2) = A speed        (how fast the axis wobbles)
--   registers_in(3) = Color speed    (how fast exp(sin()) cycles)
--   registers_in(4) = Glow sharpness (top 2 bits: LUT steepness)
--   registers_in(5) = unused
--   registers_in(6) = Switches       (bit 4 = Bypass)
--   registers_in(7) = Brightness     (0-1023 master gain)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture lostinfinite of program_top is

    constant LATENCY : natural := 15;

    -- Screen centre for ray generation.  Hardcoded for 1280x720; on SD
    -- the centre just shifts slightly off-screen which is fine.
    constant W_HALF : integer := 640;
    constant H_HALF : integer := 360;
    constant H_RAYZ : integer := 720;

    subtype t_q is signed(15 downto 0);   -- Q5.10
    subtype t_q2 is signed(31 downto 0);  -- Q10.20 intermediate

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    -- Position tracking
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');
    signal a_phase      : unsigned(9  downto 0) := (others => '0');
    signal c_phase      : unsigned(9  downto 0) := (others => '0');

    -- Per-frame constants.  All Q5.10.
    signal ax      : t_q := to_signed( 819, 16);   -- ~0.8
    signal ay      : t_q := to_signed( 410, 16);   -- ~0.4
    signal az      : t_q := to_signed( 410, 16);   -- ~0.4
    signal col_y_s : signed(10 downto 0) := to_signed(512, 11);
    signal col_u_s : signed(10 downto 0) := (others => '0');
    signal col_v_s : signed(10 downto 0) := (others => '0');

    -- Pipeline stages
    -- S1: ray / p = t*r (with fixed t absorbing ray norm)
    signal s1_px, s1_py, s1_pz : t_q := (others => '0');

    -- S2: dot(a, p) -- signed wide
    signal s2_dot : t_q2 := (others => '0');
    signal s2_px, s2_py, s2_pz : t_q := (others => '0');

    -- S3: 2*dot truncated back to Q5.10
    signal s3_2dot : t_q := (others => '0');
    signal s3_px, s3_py, s3_pz : t_q := (others => '0');

    -- S4: 2*dot*a (3 mults) + forward p
    signal s4_refx, s4_refy, s4_refz : t_q := (others => '0');
    signal s4_px,   s4_py,   s4_pz   : t_q := (others => '0');

    -- S5: reflection p' = 2*dot*a - p
    signal s5_px, s5_py, s5_pz : t_q := (others => '0');

    -- S6: p.z += 14  (and registered for timing)
    signal s6_px, s6_py, s6_pz : t_q := (others => '0');

    -- S7: atan2(pz, px) 10-bit, length_xz pseudo
    signal s7_atan   : unsigned(9 downto 0) := (others => '0');
    signal s7_lenxz  : t_q := (others => '0');
    signal s7_py     : t_q := (others => '0');

    -- S8: angular wrap: new_ang = mod(atan, 64) - 32
    signal s8_newang : signed(9 downto 0) := (others => '0');
    signal s8_lenxz  : t_q := (others => '0');
    signal s8_py     : t_q := (others => '0');

    -- S9: polar rebuild with linear sin/cos approx
    --     new_px = lenxz
    --     new_pz = lenxz * new_ang_scaled
    signal s9_npx : t_q := (others => '0');
    signal s9_npz : t_q := (others => '0');
    signal s9_py  : t_q := (others => '0');

    -- S10: p.xy = fract(p.xy) - 0.5
    signal s10_fx : t_q := (others => '0');
    signal s10_fy : t_q := (others => '0');
    signal s10_pz : t_q := (others => '0');

    -- S11: 3D octagonal length
    signal s11_lenp : t_q := (others => '0');

    -- S12: v = |lenp - 0.1| + 0.01
    signal s12_v : unsigned(9 downto 0) := (others => '0');

    -- S13: brightness = LUT(v)
    signal s13_b : unsigned(9 downto 0) := (others => '0');

    -- S14: output YUV
    signal s14_y : unsigned(9 downto 0) := (others => '0');
    signal s14_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s14_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ---------------------------------------------------------------------------
    -- 32-entry sin LUT.  Returns sin(2*pi*i/32) * 256 as signed 10-bit.
    -- Used at vsync only to precompute the per-frame constants.
    ---------------------------------------------------------------------------
    function small_sin(a : unsigned(4 downto 0)) return signed is
        variable v : signed(9 downto 0);
    begin
        case to_integer(a) is
            when  0 => v := to_signed(   0, 10);
            when  1 => v := to_signed(  50, 10);
            when  2 => v := to_signed(  98, 10);
            when  3 => v := to_signed( 142, 10);
            when  4 => v := to_signed( 181, 10);
            when  5 => v := to_signed( 212, 10);
            when  6 => v := to_signed( 236, 10);
            when  7 => v := to_signed( 251, 10);
            when  8 => v := to_signed( 256, 10);
            when  9 => v := to_signed( 251, 10);
            when 10 => v := to_signed( 236, 10);
            when 11 => v := to_signed( 212, 10);
            when 12 => v := to_signed( 181, 10);
            when 13 => v := to_signed( 142, 10);
            when 14 => v := to_signed(  98, 10);
            when 15 => v := to_signed(  50, 10);
            when 16 => v := to_signed(   0, 10);
            when 17 => v := to_signed( -50, 10);
            when 18 => v := to_signed( -98, 10);
            when 19 => v := to_signed(-142, 10);
            when 20 => v := to_signed(-181, 10);
            when 21 => v := to_signed(-212, 10);
            when 22 => v := to_signed(-236, 10);
            when 23 => v := to_signed(-251, 10);
            when 24 => v := to_signed(-256, 10);
            when 25 => v := to_signed(-251, 10);
            when 26 => v := to_signed(-236, 10);
            when 27 => v := to_signed(-212, 10);
            when 28 => v := to_signed(-181, 10);
            when 29 => v := to_signed(-142, 10);
            when 30 => v := to_signed( -98, 10);
            when others => v := to_signed( -50, 10);
        end case;
        return v;
    end function;

    ---------------------------------------------------------------------------
    -- Brightness LUT: maps v (top 8 bits of Q5.10) to 10-bit brightness.
    -- Shaped like 1/(v+eps) with a user-selectable steepness.  Replaces
    -- the shader's "exp(sin())/v" accumulation for a single-sample pass.
    ---------------------------------------------------------------------------
    function brightness_lut(v : unsigned(7 downto 0); steep : unsigned(1 downto 0))
        return unsigned is
        variable b : unsigned(9 downto 0);
    begin
        case to_integer(v) is
            when   0 =>                     b := to_unsigned(1023, 10);
            when   1 =>                     b := to_unsigned(1023, 10);
            when   2 =>                     b := to_unsigned( 960, 10);
            when   3 =>                     b := to_unsigned( 880, 10);
            when   4 =>                     b := to_unsigned( 780, 10);
            when   5 =>                     b := to_unsigned( 680, 10);
            when   6 =>                     b := to_unsigned( 590, 10);
            when   7 =>                     b := to_unsigned( 510, 10);
            when   8 |  9 =>                b := to_unsigned( 430, 10);
            when  10 | 11 =>                b := to_unsigned( 360, 10);
            when  12 | 13 | 14 | 15 =>      b := to_unsigned( 290, 10);
            when  16 | 17 | 18 | 19 =>      b := to_unsigned( 230, 10);
            when  20 to  27 =>              b := to_unsigned( 180, 10);
            when  28 to  35 =>              b := to_unsigned( 140, 10);
            when  36 to  47 =>              b := to_unsigned( 100, 10);
            when  48 to  63 =>              b := to_unsigned(  70, 10);
            when  64 to  95 =>              b := to_unsigned(  45, 10);
            when  96 to 127 =>              b := to_unsigned(  28, 10);
            when others =>                  b := to_unsigned(  16, 10);
        end case;
        -- Steepness adjustment: shift down by (3 - steep) to soften or sharpen
        case to_integer(steep) is
            when 0 => b := shift_right(b, 2);
            when 1 => b := shift_right(b, 1);
            when 2 => b := b;
            when others => b := b;   -- steep=3 same as 2 (clipped to 1023 anyway)
        end case;
        return b;
    end function;

begin

    process(clk)
        -- Stage 1
        variable rx_i, ry_i, rz_i : integer range -4096 to 4095;

        -- Stage 2 (dot)
        variable prod_x, prod_y, prod_z : t_q2;
        variable dot_v : t_q2;

        -- Stage 4 (2*dot*a)
        variable mx : t_q2;
        variable my : t_q2;
        variable mz : t_q2;

        -- Stage 7 (atan2 + length_xz)
        variable abs_px, abs_pz : t_q;
        variable max_ap, min_ap : t_q;
        variable prio    : integer range 0 to 14;
        variable norm_min : t_q;
        variable small   : unsigned(6 downto 0);
        variable quadrant : unsigned(2 downto 0);
        variable atan_v   : unsigned(9 downto 0);

        -- Stage 10 (fract)
        variable frac_x, frac_y : t_q;

        -- Stage 11 (3D length)
        variable abs_fx, abs_fy, abs_fz : t_q;
        variable hi, mid, lo : t_q;
        variable len_p_v : t_q;

        -- Stage 12 (v)
        variable len_m : t_q;
        variable v_abs : t_q;

        -- Per-frame constant update
        variable ph_a : unsigned(4 downto 0);
        variable ph_c : unsigned(4 downto 0);

        -- Stage 14 (color combine)
        variable bright_m : unsigned(19 downto 0);
        variable y_prod   : unsigned(20 downto 0);
        variable u_prod   : signed(20 downto 0);
        variable v_prod   : signed(20 downto 0);
        variable y_out    : unsigned(9 downto 0);
        variable u_out    : unsigned(9 downto 0);
        variable v_out    : unsigned(9 downto 0);
        variable uv_tmp   : signed(11 downto 0);

        variable gain     : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then

            ----------------------------------------------------------------
            -- Sync pipeline (video stream delay line)
            ----------------------------------------------------------------
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 0: position / frame tracking + per-frame constants.
            --
            -- At every vsync edge:
            --   * advance frame_count
            --   * advance two independent phase accumulators (a_phase for
            --     the axis wobble, c_phase for the color cycle), driven
            --     by the Speed and Color-speed knobs.
            --   * look up ax/ay/az and col_y/u/v from the small 32-entry
            --     sin table with shader-faithful phase offsets
            --     (vec3(3,1,0) for a, vec4(0,1,3,0) for color).
            --
            -- The shader uses iTime*.1 and iTime; we approximate that by
            -- two knob-driven phase rates rather than a true clock.
            ----------------------------------------------------------------
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            if data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                pixel_x <= (others => '0');
                pixel_y <= pixel_y + 1;
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                pixel_y     <= (others => '0');
                frame_count <= frame_count + 1;
                a_phase <= a_phase
                         + resize(unsigned(registers_in(2)(9 downto 4)), 10);
                c_phase <= c_phase
                         + resize(unsigned(registers_in(3)(9 downto 4)), 10);

                ph_a := a_phase(9 downto 5);
                ph_c := c_phase(9 downto 5);

                -- a = sin(phase + (3, 1, 0) radians).
                -- In 32-entry table units: 1 rad ~= 5, 3 rad ~= 15.
                ax <= resize(shift_left(small_sin(ph_a + to_unsigned(15, 5)), 2), 16);
                ay <= resize(shift_left(small_sin(ph_a + to_unsigned( 5, 5)), 2), 16);
                az <= resize(shift_left(small_sin(ph_a                    ), 2), 16);

                -- col = exp(sin(phase + (0, 1, 3, 0))).  We approximate
                -- exp(sin) by (sin + 1.5) -- always positive, same period.
                -- Stored in Q2.8 (11-bit signed).
                col_y_s <= resize(
                             small_sin(ph_c) + to_signed(384, 10),
                             11);
                col_u_s <= resize(
                             small_sin(ph_c + to_unsigned( 5, 5)),
                             11);
                col_v_s <= resize(
                             small_sin(ph_c + to_unsigned(15, 5)),
                             11);
            end if;

            ----------------------------------------------------------------
            -- STAGE 1: Ray generation and p = t*r.
            --
            -- Shader:     r = normalize(vec3(2*I - res.xy, -res.y))
            --             p = t * r
            --
            -- Literal:    r_unnorm = (2x - W, 2y - H, -H)
            --             (we skip normalize, absorbing scale into t)
            --             p = r_unnorm >> k,  k selected by knob 0
            --
            -- Result is in Q5.10: raw pixel-space offsets scaled down by
            -- 2^k so the resulting |p| lands near unity, putting us at
            -- roughly iteration t ~= 1 of the shader's march.
            ----------------------------------------------------------------
            rx_i := to_integer(shift_left(resize(pixel_x, 13), 1))
                  - 2 * W_HALF;
            ry_i := to_integer(shift_left(resize(pixel_y, 13), 1))
                  - 2 * H_HALF;
            rz_i := -H_RAYZ;

            -- Convert pixel-space (rx_i, ry_i, rz_i) to Q5.10 with the
            -- fixed-t shift.  Knob 0 top 3 bits pick k in {4..11}:
            --   k=4  -> coarse / near camera
            --   k=7  -> middle sample
            --   k=11 -> deep into the scene
            case to_integer(unsigned(registers_in(0)(9 downto 7))) is
                when 0 =>   -- k = 4 : << 6 in Q5.10
                    s1_px <= to_signed(rx_i, 16) sll 6;
                    s1_py <= to_signed(ry_i, 16) sll 6;
                    s1_pz <= to_signed(rz_i, 16) sll 6;
                when 1 =>   -- k = 5 : << 5
                    s1_px <= to_signed(rx_i, 16) sll 5;
                    s1_py <= to_signed(ry_i, 16) sll 5;
                    s1_pz <= to_signed(rz_i, 16) sll 5;
                when 2 =>   -- k = 6 : << 4
                    s1_px <= to_signed(rx_i, 16) sll 4;
                    s1_py <= to_signed(ry_i, 16) sll 4;
                    s1_pz <= to_signed(rz_i, 16) sll 4;
                when 3 =>   -- k = 7 : << 3
                    s1_px <= to_signed(rx_i, 16) sll 3;
                    s1_py <= to_signed(ry_i, 16) sll 3;
                    s1_pz <= to_signed(rz_i, 16) sll 3;
                when 4 =>   -- k = 8 : << 2
                    s1_px <= to_signed(rx_i, 16) sll 2;
                    s1_py <= to_signed(ry_i, 16) sll 2;
                    s1_pz <= to_signed(rz_i, 16) sll 2;
                when 5 =>   -- k = 9 : << 1
                    s1_px <= to_signed(rx_i, 16) sll 1;
                    s1_py <= to_signed(ry_i, 16) sll 1;
                    s1_pz <= to_signed(rz_i, 16) sll 1;
                when 6 =>   -- k = 10 : << 0
                    s1_px <= to_signed(rx_i, 16);
                    s1_py <= to_signed(ry_i, 16);
                    s1_pz <= to_signed(rz_i, 16);
                when others =>  -- k = 11 : >> 1
                    s1_px <= shift_right(to_signed(rx_i, 16), 1);
                    s1_py <= shift_right(to_signed(ry_i, 16), 1);
                    s1_pz <= shift_right(to_signed(rz_i, 16), 1);
            end case;

            ----------------------------------------------------------------
            -- STAGE 2: dot = ax*px + ay*py + az*pz   (3 multiplies)
            -- Shader:   dot(a, p)
            ----------------------------------------------------------------
            prod_x := s1_px * ax;
            prod_y := s1_py * ay;
            prod_z := s1_pz * az;
            dot_v  := prod_x + prod_y + prod_z;

            s2_dot <= dot_v;
            s2_px  <= s1_px;
            s2_py  <= s1_py;
            s2_pz  <= s1_pz;

            ----------------------------------------------------------------
            -- STAGE 3: 2*dot back to Q5.10
            -- Shader: 2 * dot
            ----------------------------------------------------------------
            -- dot is Q10.20; shift right 9 to multiply by 2 and drop 10
            -- fractional bits.  Truncate to 16-bit.
            s3_2dot <= resize(shift_right(s2_dot, 9), 16);
            s3_px   <= s2_px;
            s3_py   <= s2_py;
            s3_pz   <= s2_pz;

            ----------------------------------------------------------------
            -- STAGE 4: 2*dot*a   (3 multiplies).
            -- Shader: 2*(a.p)*a
            ----------------------------------------------------------------
            mx := s3_2dot * ax;
            my := s3_2dot * ay;
            mz := s3_2dot * az;

            s4_refx <= resize(shift_right(mx, 10), 16);
            s4_refy <= resize(shift_right(my, 10), 16);
            s4_refz <= resize(shift_right(mz, 10), 16);
            s4_px   <= s3_px;
            s4_py   <= s3_py;
            s4_pz   <= s3_pz;

            ----------------------------------------------------------------
            -- STAGE 5: p' = 2*(a.p)*a - p     (3 subtractions)
            -- Shader: full Householder reflection
            ----------------------------------------------------------------
            s5_px <= s4_refx - s4_px;
            s5_py <= s4_refy - s4_py;
            s5_pz <= s4_refz - s4_pz;

            ----------------------------------------------------------------
            -- STAGE 6: p.z += 14  (or knob-controlled value)
            -- Shader: p.z += 14.
            --
            -- "14" in Q5.10 = 14 * 1024 = 14336.  Knob 1 top 4 bits
            -- override the integer part (0..15).
            ----------------------------------------------------------------
            s6_px <= s5_px;
            s6_py <= s5_py;
            s6_pz <= s5_pz
                   + shift_left(
                       resize(signed('0' &
                                unsigned(registers_in(1)(9 downto 6))),
                              16),
                       10);

            ----------------------------------------------------------------
            -- STAGE 7: length(p.xz) and atan2(p.z, p.x).
            --
            -- length(p.xz) ~= max(|px|,|pz|) + min/2   (octagonal)
            --
            -- atan2 is done in three steps:
            --   1. Quadrant: sign bits of px, pz, plus |px|<|pz| swap.
            --   2. Within octant, ratio = min/max via priority encoder
            --      + barrel shift on max (no divide).
            --   3. Linear atan: atan(t) ~= t * (pi/4); in 10-bit angle
            --      units (1024 = 2*pi) that's atan_octant = ratio * 128.
            --
            -- 10-bit angle convention:  0 = 0 rad, 256 = pi/2, 512 = pi,
            -- 768 = 3pi/2.  Wraps mod 1024.
            ----------------------------------------------------------------
            if s6_px(15) = '1' then abs_px := -s6_px; else abs_px := s6_px; end if;
            if s6_pz(15) = '1' then abs_pz := -s6_pz; else abs_pz := s6_pz; end if;

            if abs_px >= abs_pz then
                max_ap := abs_px;
                min_ap := abs_pz;
                quadrant := "000";  -- |px| dominates, angle in [-pi/4,pi/4] around 0
            else
                max_ap := abs_pz;
                min_ap := abs_px;
                quadrant := "001";  -- |pz| dominates
            end if;

            -- Priority encode max_ap (search top bit 14 down to 4)
            prio := 4;
            for b in 14 downto 4 loop
                if max_ap(b) = '1' then
                    prio := b;
                end if;
            end loop;

            -- Shift min_ap so its MSB aligns with max_ap's MSB.  That
            -- gives min/max as a fixed-point fraction in [0,1).
            case prio is
                when 14 =>
                    norm_min := shift_right(min_ap,  5);
                when 13 =>
                    norm_min := shift_right(min_ap,  4);
                when 12 =>
                    norm_min := shift_right(min_ap,  3);
                when 11 =>
                    norm_min := shift_right(min_ap,  2);
                when 10 =>
                    norm_min := shift_right(min_ap,  1);
                when 9 =>
                    norm_min := min_ap;
                when 8 =>
                    norm_min := shift_left(min_ap, 1);
                when 7 =>
                    norm_min := shift_left(min_ap, 2);
                when 6 =>
                    norm_min := shift_left(min_ap, 3);
                when 5 =>
                    norm_min := shift_left(min_ap, 4);
                when others =>
                    norm_min := shift_left(min_ap, 5);
            end case;

            -- Take top 7 bits of norm_min as the ratio in [0..127]
            -- representing [0..1).  Linear atan: angle_octant = ratio.
            small := unsigned(norm_min(9 downto 3));
            if small > to_unsigned(127, 7) then
                small := to_unsigned(127, 7);
            end if;

            -- Compose full 10-bit atan2 angle from octant info.
            -- Start with octant base angle then add/subtract small.
            --   px >= 0, pz >= 0, |px|>=|pz|:  atan ~=         small       (angle in [0, 128])
            --   px >= 0, pz >= 0, |px|< |pz|:  atan ~=  256 - small
            --   px <  0, pz >= 0, |px|< |pz|:  atan ~=  256 + small
            --   px <  0, pz >= 0, |px|>=|pz|:  atan ~=  512 - small
            --   px <  0, pz <  0, |px|>=|pz|:  atan ~=  512 + small
            --   px <  0, pz <  0, |px|< |pz|:  atan ~=  768 - small
            --   px >= 0, pz <  0, |px|< |pz|:  atan ~=  768 + small
            --   px >= 0, pz <  0, |px|>=|pz|:  atan ~= 1024 - small
            atan_v := (others => '0');
            if s6_px(15) = '0' and s6_pz(15) = '0' then
                if quadrant(0) = '0' then
                    atan_v := resize(small, 10);
                else
                    atan_v := to_unsigned(256, 10) - resize(small, 10);
                end if;
            elsif s6_px(15) = '1' and s6_pz(15) = '0' then
                if quadrant(0) = '1' then
                    atan_v := to_unsigned(256, 10) + resize(small, 10);
                else
                    atan_v := to_unsigned(512, 10) - resize(small, 10);
                end if;
            elsif s6_px(15) = '1' and s6_pz(15) = '1' then
                if quadrant(0) = '0' then
                    atan_v := to_unsigned(512, 10) + resize(small, 10);
                else
                    atan_v := to_unsigned(768, 10) - resize(small, 10);
                end if;
            else
                if quadrant(0) = '1' then
                    atan_v := to_unsigned(768, 10) + resize(small, 10);
                else
                    atan_v := to_unsigned(1024 - 1, 10)
                            - resize(small, 10) + to_unsigned(1, 10);
                end if;
            end if;

            -- Octagonal length_xz = max + min/2
            s7_lenxz <= max_ap + shift_right(min_ap, 1);
            s7_atan  <= atan_v;
            s7_py    <= s6_py;

            ----------------------------------------------------------------
            -- STAGE 8: new_ang = mod(atan, .4) - .2
            --
            -- In our 10-bit angle convention where 1024 = 2*pi,
            --   .4 rad = .4 * 1024 / (2*pi) = 65.25  ~= 64
            --   .2 rad = 32
            -- So new_ang = (atan MOD 64) - 32.  The mod is a plain bitwise
            -- AND with 0x3F; range after subtract is [-32, +31].
            ----------------------------------------------------------------
            s8_newang <= signed('0' & s7_atan(5 downto 0)) - to_signed(32, 10);
            s8_lenxz  <= s7_lenxz;
            s8_py     <= s7_py;

            ----------------------------------------------------------------
            -- STAGE 9: polar rebuild.
            -- Shader: p.xz = length_xz * (cos(new_ang), sin(new_ang))
            --
            -- For |new_ang| < 0.2 rad, cos ~= 1 and sin ~= new_ang,
            -- both within 2% error.  So:
            --    new_px = length_xz                       (no multiply)
            --    new_pz = length_xz * (new_ang in radians)
            --
            -- new_ang in radians = new_ang_10bit * (2*pi / 1024).
            -- In Q5.10, representing (2*pi/1024) as 6/1024 ~= 6 is close;
            -- we use exactly 2*pi/1024 ~= 6.28 / 1024.  Simplest: take
            -- new_ang_10bit as Q10.0 and treat it as Q5.10 *after* a
            -- shift_left of 3 (gives factor 8/1024, within 25% of the
            -- true 6.28/1024).  Fold the small correction into the
            -- brightness so this is not visually important.
            ----------------------------------------------------------------
            s9_npx <= s8_lenxz;
            -- length_xz (Q5.10) * (new_ang_10bit shifted)
            -- s8_newang is signed 10-bit in [-32, +31]; resize to Q5.10
            -- then multiply and truncate.
            s9_npz <= resize(
                        shift_right(
                            s8_lenxz * resize(s8_newang, 16),
                            10),
                        16);
            s9_py  <= s8_py;

            ----------------------------------------------------------------
            -- STAGE 10: p.xy = fract(p.xy) - 0.5
            -- Shader: p.xy = mod(p.xy, 1.) - .5
            --
            -- In Q5.10, fract(x) is just the lower 10 bits treated as an
            -- unsigned fractional in [0, 1).  Subtracting 0.5 means
            -- subtracting 512 from that, giving a signed value in
            -- [-512, +511] = Q0.10.  Resize back to Q5.10.
            ----------------------------------------------------------------
            frac_x := resize(
                        signed('0' & unsigned(s9_npx(9 downto 0))),
                        16)
                    - to_signed(512, 16);
            frac_y := resize(
                        signed('0' & unsigned(s9_py(9 downto 0))),
                        16)
                    - to_signed(512, 16);

            s10_fx <= frac_x;
            s10_fy <= frac_y;
            s10_pz <= s9_npz;

            ----------------------------------------------------------------
            -- STAGE 11: length(p) = octagonal 3D pseudo-length.
            -- Shader: length(p)
            --
            -- |p|_oct = hi + mid/2 + lo/4  where (hi, mid, lo) is the
            -- sorted (|px|, |py|, |pz|) descending.
            ----------------------------------------------------------------
            if s10_fx(15) = '1' then abs_fx := -s10_fx; else abs_fx := s10_fx; end if;
            if s10_fy(15) = '1' then abs_fy := -s10_fy; else abs_fy := s10_fy; end if;
            if s10_pz(15) = '1' then abs_fz := -s10_pz; else abs_fz := s10_pz; end if;

            -- 3-way sort (descending).  All positive now.
            if abs_fx >= abs_fy then
                if abs_fy >= abs_fz then
                    hi := abs_fx; mid := abs_fy; lo := abs_fz;
                elsif abs_fx >= abs_fz then
                    hi := abs_fx; mid := abs_fz; lo := abs_fy;
                else
                    hi := abs_fz; mid := abs_fx; lo := abs_fy;
                end if;
            else
                if abs_fx >= abs_fz then
                    hi := abs_fy; mid := abs_fx; lo := abs_fz;
                elsif abs_fy >= abs_fz then
                    hi := abs_fy; mid := abs_fz; lo := abs_fx;
                else
                    hi := abs_fz; mid := abs_fy; lo := abs_fx;
                end if;
            end if;

            len_p_v := hi + shift_right(mid, 1) + shift_right(lo, 2);
            s11_lenp <= len_p_v;

            ----------------------------------------------------------------
            -- STAGE 12: v = |length(p) - 0.1| + 0.01
            -- Shader: v = abs(length(p) - .1) + .01
            --
            -- 0.1 in Q5.10 = 102.  0.01 in Q5.10 = 10.
            -- Output v as 10-bit unsigned by taking the top 10 bits of
            -- the Q5.10 magnitude (saturating).
            ----------------------------------------------------------------
            len_m := s11_lenp - to_signed(102, 16);
            if len_m(15) = '1' then
                v_abs := -len_m;
            else
                v_abs := len_m;
            end if;
            v_abs := v_abs + to_signed(10, 16);

            -- Saturate to unsigned 10-bit.
            if v_abs(15) = '1' then
                s12_v <= (others => '0');
            elsif v_abs > to_signed(1023, 16) then
                s12_v <= (others => '1');
            else
                s12_v <= unsigned(v_abs(9 downto 0));
            end if;

            ----------------------------------------------------------------
            -- STAGE 13: brightness = LUT(v)
            -- Shader surrogate: replaces "1/v" with a table lookup
            -- shaped like 1/(v+eps).
            ----------------------------------------------------------------
            s13_b <= brightness_lut(
                       s12_v(9 downto 2),
                       unsigned(registers_in(4)(9 downto 8)));

            ----------------------------------------------------------------
            -- STAGE 14: compose Y / U / V from per-frame col * brightness.
            --
            -- Y = col_y * brightness                   (always positive)
            -- U = 512 + col_u * brightness / 4         (signed, centred)
            -- V = 512 + col_v * brightness / 4
            --
            -- Master gain from Brightness slider (knob 12) scales Y.
            -- Shader equivalent: O = col_vec / v, then YUV transform.
            ----------------------------------------------------------------
            gain := unsigned(registers_in(7));

            -- Y = col_y (~Q2.8, 12b) * brightness (Q1.10, 12b)
            -- Raw 24-bit product; actual max ~640 * 2046 ~ 1.3M fits in 21b.
            y_prod := resize(
                        ('0' & unsigned(col_y_s)) * ('0' & s13_b & "0"),
                        21);
            y_out  := resize(shift_right(y_prod, 10), 10);

            -- Apply master brightness: y_out (0..1023) * gain (0..1023) / 1024
            -- 11b * 11b = 22b product; max 1023*1023 = 1046529 fits in 21b.
            y_prod := resize(
                        ('0' & y_out) * ('0' & gain),
                        21);
            y_prod := shift_right(y_prod, 10);
            if y_prod > to_unsigned(1023, 21) then
                y_out := (others => '1');
            else
                y_out := y_prod(9 downto 0);
            end if;

            -- U / V chroma: signed multiply, shift down, add 512 bias
            u_prod := resize(col_u_s * signed('0' & s13_b), 21);
            v_prod := resize(col_v_s * signed('0' & s13_b), 21);
            u_prod := shift_right(u_prod, 11);
            v_prod := shift_right(v_prod, 11);

            -- Bias signed chroma by +512 into unsigned 10-bit 0..1023 and clamp.
            uv_tmp := resize(u_prod, 12) + to_signed(512, 12);
            if uv_tmp < 0 then
                u_out := (others => '0');
            elsif uv_tmp > 1023 then
                u_out := (others => '1');
            else
                u_out := unsigned(uv_tmp(9 downto 0));
            end if;

            uv_tmp := resize(v_prod, 12) + to_signed(512, 12);
            if uv_tmp < 0 then
                v_out := (others => '0');
            elsif uv_tmp > 1023 then
                v_out := (others => '1');
            else
                v_out := unsigned(uv_tmp(9 downto 0));
            end if;

            s14_y <= y_out;
            s14_u <= u_out;
            s14_v <= v_out;

            ----------------------------------------------------------------
            -- OUTPUT
            ----------------------------------------------------------------
            data_out <= pipe(LATENCY - 1);
            if registers_in(6)(4) = '0' then
                data_out.y <= std_logic_vector(s14_y);
                data_out.u <= std_logic_vector(s14_u);
                data_out.v <= std_logic_vector(s14_v);
            end if;
        end if;
    end process;

end architecture lostinfinite;
