-- Echotube: per-column persistent canvas with feedback trails.
--
-- 3 BRAM line buffers hold the running per-column Y, U, V values.  Each
-- pixel: read fb[active_x - h_shift] from BRAM, blend with current input
-- via two coefficients (Decay = feedback strength, Inject = new-input
-- weight), then write the result back to bram[active_x] and emit it as
-- the output.  The buffer therefore accumulates a horizontal feedback
-- trail that persists across scanlines.
--
-- Modes:
--   * Persistence slider (param 12) sets a master feedback gain;
--     pushing it high keeps the trail alive for many frames.
--   * H Shift offsets read vs. write so trails sweep horizontally.
--   * Hue Drift adds a small U/V rotation each frame so old data
--     shifts through hues, producing rainbow tails on moving objects.
--   * Posterize Trail snaps the feedback Y to the top 3 bits for a
--     chunky/blocky look.
--   * Freeze halts new writes so the buffer holds the last frame.
--
-- BRAM: 3 line buffers × 1 BRAM each = 3 BRAMs.
-- Designed for 74.25 MHz HD timing on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Decay        (feedback channel gain)
--   registers_in(1) = H Shift      (read-vs-write column offset)
--   registers_in(2) = Hue Drift    (per-frame chroma rotation step)
--   registers_in(3) = Brightness   (output Y gain)
--   registers_in(4) = Saturation   (output chroma gain)
--   registers_in(5) = Inject       (current-input weight in the loop)
--   registers_in(6) = Switches:
--     b0 Channels (0=YUV / 1=Y Only — chroma passes through dry)
--     b1 Drift Dir (0=CW / 1=CCW)
--     b2 Freeze (0=Live / 1=Freeze, halts BRAM writes)
--     b3 Posterize Trail (0=Smooth / 1=Posterized)
--     b4 Invert
--   registers_in(7) = Persistence (slider, KEY — master trail length)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture echotube of program_top is

    constant LATENCY    : natural := 14;
    constant C_BUF_DEPTH : integer := 11;
    constant C_BUF_SIZE  : integer := 2**C_BUF_DEPTH;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal prev_avid    : std_logic := '0';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    -- Params
    signal decay_r       : unsigned(9 downto 0) := to_unsigned(850, 10);
    signal h_shift_r     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal hue_drift_r   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal bright_r      : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal sat_r         : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal inject_r      : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal persistence_r : unsigned(9 downto 0) := to_unsigned(700, 10);
    -- Effective coefficients derived from persistence × decay or × inject.
    -- These are pre-multiplied at vsync so the per-pixel multipliers stay
    -- to a single 10×10 product.
    signal fb_coef_r  : unsigned(9 downto 0) := to_unsigned(580, 10);
    signal in_coef_r  : unsigned(9 downto 0) := to_unsigned(64,  10);
    signal y_only_r   : std_logic := '0';
    signal ccw_r      : std_logic := '0';
    signal freeze_r   : std_logic := '0';
    signal poster_r   : std_logic := '0';
    signal invert_r   : std_logic := '0';

    -- Frame chroma drift accumulator (built from hue_drift_r).
    signal drift_acc_r : unsigned(9 downto 0) := (others => '0');

    signal active_x       : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal active_width_r : unsigned(C_BUF_DEPTH - 1 downto 0) :=
                              to_unsigned(1920 mod C_BUF_SIZE, C_BUF_DEPTH);

    -- BRAMs.  Initial neutral chroma (512) keeps the canvas grey on power-up.
    type t_bram is array (0 to C_BUF_SIZE - 1)
        of std_logic_vector(9 downto 0);
    signal bram_y : t_bram := (others => "0000000000");
    signal bram_u : t_bram := (others => "1000000000");
    signal bram_v : t_bram := (others => "1000000000");

    signal rd_addr  : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal wr_addr  : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal rd_y_d   : std_logic_vector(9 downto 0) := (others => '0');
    signal rd_u_d   : std_logic_vector(9 downto 0) := (others => '0');
    signal rd_v_d   : std_logic_vector(9 downto 0) := (others => '0');

    -- ------------------------------------------------------------------
    -- Pipeline signals
    -- ------------------------------------------------------------------
    -- S0: register input
    signal s0_y : unsigned(9 downto 0);
    signal s0_u : unsigned(9 downto 0);
    signal s0_v : unsigned(9 downto 0);

    -- S1: bring along while BRAM read completes
    signal s1_y : unsigned(9 downto 0);
    signal s1_u : unsigned(9 downto 0);
    signal s1_v : unsigned(9 downto 0);

    -- S2: rotated chroma feedback (optional) + register raw fb / current
    signal s2_fb_y     : unsigned(9 downto 0);
    signal s2_fb_u     : unsigned(9 downto 0);
    signal s2_fb_v     : unsigned(9 downto 0);
    signal s2_y_in     : unsigned(9 downto 0);
    signal s2_u_in     : unsigned(9 downto 0);
    signal s2_v_in     : unsigned(9 downto 0);

    -- S3: raw products (fb × fb_coef, current × in_coef).
    signal s3_fby_p : unsigned(19 downto 0);
    signal s3_fbu_p : unsigned(19 downto 0);
    signal s3_fbv_p : unsigned(19 downto 0);
    signal s3_cy_p  : unsigned(19 downto 0);
    signal s3_cu_p  : unsigned(19 downto 0);
    signal s3_cv_p  : unsigned(19 downto 0);

    -- S4: combined feedback values (clipped to 10-bit) — these are written
    -- back to BRAM and continue down the pipeline.
    signal s4_y : unsigned(9 downto 0);
    signal s4_u : unsigned(9 downto 0);
    signal s4_v : unsigned(9 downto 0);

    -- S5..S10: post-processing (brightness, saturation, invert).
    signal s5_y_mul : unsigned(19 downto 0);
    signal s5_u     : unsigned(9 downto 0);
    signal s5_v     : unsigned(9 downto 0);
    signal s6_y     : unsigned(9 downto 0);
    signal s6_u_mul : signed(22 downto 0);
    signal s6_v_mul : signed(22 downto 0);
    signal s7_y : unsigned(9 downto 0);
    signal s7_u : unsigned(9 downto 0);
    signal s7_v : unsigned(9 downto 0);
    signal s8_y : unsigned(9 downto 0);
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);

    function sat10_s(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

    function sat10_u(v : unsigned) return unsigned is
    begin
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(resize(v, 10));
    end function;

begin

    -- ========================================================================
    -- Position + param latch + coefficient derivation
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_pers   : unsigned(9 downto 0);
        variable v_decay  : unsigned(9 downto 0);
        variable v_fb_c   : unsigned(19 downto 0);
        variable v_in_c   : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_h_edge := '0';
            v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
                active_x <= (others => '0');
            else
                if data_in.avid = '1' and prev_avid = '0' then
                    active_x <= (others => '0');
                elsif data_in.avid = '1' then
                    active_x <= active_x + 1;
                end if;
                if data_in.avid = '1' then
                    pixel_x <= pixel_x + 1;
                end if;
            end if;
            if data_in.avid = '0' and prev_avid = '1' then
                active_width_r <= active_x;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                decay_r       <= unsigned(registers_in(0));
                h_shift_r     <= unsigned(registers_in(1));
                hue_drift_r   <= unsigned(registers_in(2));
                bright_r      <= unsigned(registers_in(3));
                sat_r         <= unsigned(registers_in(4));
                inject_r      <= unsigned(registers_in(5));
                y_only_r      <= registers_in(6)(0);
                ccw_r         <= registers_in(6)(1);
                freeze_r      <= registers_in(6)(2);
                poster_r      <= registers_in(6)(3);
                invert_r      <= registers_in(6)(4);
                persistence_r <= unsigned(registers_in(7));

                -- Pre-compute feedback coefficient = persistence × decay /
                -- 1024 so the per-pixel multiplier path is just one 10×10.
                -- Similarly for inject: in_coef = (1023-persistence) × inject
                -- / 1024.  These coefficients always sum to ≤ ~1023 so the
                -- per-pixel blend is stable.
                v_pers  := unsigned(registers_in(7));
                v_decay := unsigned(registers_in(0));
                v_fb_c  := v_pers * v_decay;
                v_in_c  := (to_unsigned(1023, 10) - v_pers) *
                           unsigned(registers_in(5));
                fb_coef_r <= v_fb_c(19 downto 10);
                in_coef_r <= v_in_c(19 downto 10);

                -- Frame chroma drift accumulator.
                if ccw_r = '0' then
                    drift_acc_r <= drift_acc_r +
                                   unsigned(registers_in(2)(9 downto 4));
                else
                    drift_acc_r <= drift_acc_r -
                                   unsigned(registers_in(2)(9 downto 4));
                end if;
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- BRAM address generation.
    -- ========================================================================
    p_addr : process(clk)
        variable v_off : unsigned(C_BUF_DEPTH - 1 downto 0);
        variable v_rd  : signed(C_BUF_DEPTH downto 0);
    begin
        if rising_edge(clk) then
            wr_addr <= active_x;
            -- H Shift maps 0..1023 to 0..2047 BRAM offset (just left-shift 1).
            v_off := h_shift_r & '0';
            v_rd := signed('0' & std_logic_vector(active_x)) -
                    signed('0' & std_logic_vector(v_off));
            if v_rd < 0 then
                v_rd := v_rd +
                        signed('0' & std_logic_vector(active_width_r));
            elsif v_rd >= signed('0' & std_logic_vector(active_width_r)) then
                v_rd := v_rd -
                        signed('0' & std_logic_vector(active_width_r));
            end if;
            rd_addr <= unsigned(std_logic_vector(v_rd(C_BUF_DEPTH - 1 downto 0)));
        end if;
    end process p_addr;

    p_bram_ry : process(clk) begin
        if rising_edge(clk) then rd_y_d <= bram_y(to_integer(rd_addr)); end if;
    end process;
    p_bram_ru : process(clk) begin
        if rising_edge(clk) then rd_u_d <= bram_u(to_integer(rd_addr)); end if;
    end process;
    p_bram_rv : process(clk) begin
        if rising_edge(clk) then rd_v_d <= bram_v(to_integer(rd_addr)); end if;
    end process;

    p_bram_wy : process(clk) begin
        if rising_edge(clk) then
            if freeze_r = '0' then
                bram_y(to_integer(wr_addr)) <= std_logic_vector(s4_y);
            end if;
        end if;
    end process;
    p_bram_wu : process(clk) begin
        if rising_edge(clk) then
            if freeze_r = '0' and y_only_r = '0' then
                bram_u(to_integer(wr_addr)) <= std_logic_vector(s4_u);
            end if;
        end if;
    end process;
    p_bram_wv : process(clk) begin
        if rising_edge(clk) then
            if freeze_r = '0' and y_only_r = '0' then
                bram_v(to_integer(wr_addr)) <= std_logic_vector(s4_v);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Main pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_sum_y : unsigned(20 downto 0);
        variable v_sum_u : unsigned(20 downto 0);
        variable v_sum_v : unsigned(20 downto 0);
        variable v_u_c   : signed(11 downto 0);
        variable v_v_c   : signed(11 downto 0);
        variable v_u_re  : signed(13 downto 0);
        variable v_v_re  : signed(13 downto 0);
        variable v_tu    : signed(11 downto 0);
        variable v_tv    : signed(11 downto 0);
        variable v_rotu  : unsigned(9 downto 0);
        variable v_rotv  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: register input.
            s0_y <= unsigned(data_in.y);
            s0_u <= unsigned(data_in.u);
            s0_v <= unsigned(data_in.v);

            -- S1: bring along while BRAM read completes.
            s1_y <= s0_y;
            s1_u <= s0_u;
            s1_v <= s0_v;

            -- S2: optional 90° chroma rotation of the feedback path when
            -- drift_acc_r passes specific thresholds — gives an obvious
            -- rainbow tail when Hue Drift is on.
            if drift_acc_r(9 downto 8) = "01" then
                v_tu := signed('0' & std_logic_vector(unsigned(rd_u_d))) -
                        to_signed(512, 12);
                v_tv := signed('0' & std_logic_vector(unsigned(rd_v_d))) -
                        to_signed(512, 12);
                v_rotu := sat10_s(to_signed(512, 14) + resize(v_tv, 14));
                v_rotv := sat10_s(to_signed(512, 14) - resize(v_tu, 14));
                s2_fb_u <= v_rotu; s2_fb_v <= v_rotv;
            elsif drift_acc_r(9 downto 8) = "10" then
                s2_fb_u <= to_unsigned(1023, 10) - unsigned(rd_u_d);
                s2_fb_v <= to_unsigned(1023, 10) - unsigned(rd_v_d);
            elsif drift_acc_r(9 downto 8) = "11" then
                v_tu := signed('0' & std_logic_vector(unsigned(rd_u_d))) -
                        to_signed(512, 12);
                v_tv := signed('0' & std_logic_vector(unsigned(rd_v_d))) -
                        to_signed(512, 12);
                v_rotu := sat10_s(to_signed(512, 14) - resize(v_tv, 14));
                v_rotv := sat10_s(to_signed(512, 14) + resize(v_tu, 14));
                s2_fb_u <= v_rotu; s2_fb_v <= v_rotv;
            else
                s2_fb_u <= unsigned(rd_u_d);
                s2_fb_v <= unsigned(rd_v_d);
            end if;
            -- Posterize fb Y to coarse bands when enabled.
            if poster_r = '1' then
                s2_fb_y <= unsigned(rd_y_d(9 downto 7)) & "0000000";
            else
                s2_fb_y <= unsigned(rd_y_d);
            end if;
            s2_y_in <= s1_y;
            s2_u_in <= s1_u;
            s2_v_in <= s1_v;

            -- S3: raw products.  fb_coef + in_coef sum < 1024 → output ≤ 1023.
            s3_fby_p <= s2_fb_y * fb_coef_r;
            s3_fbu_p <= s2_fb_u * fb_coef_r;
            s3_fbv_p <= s2_fb_v * fb_coef_r;
            s3_cy_p  <= s2_y_in * in_coef_r;
            s3_cu_p  <= s2_u_in * in_coef_r;
            s3_cv_p  <= s2_v_in * in_coef_r;

            -- S4: sum + extract (sum is at most ~1023*1024 in each term;
            -- both terms sum to <= ~2046*1024 hence (>>10) ≤ 2046, clamped).
            v_sum_y := resize(s3_fby_p, 21) + resize(s3_cy_p, 21);
            v_sum_u := resize(s3_fbu_p, 21) + resize(s3_cu_p, 21);
            v_sum_v := resize(s3_fbv_p, 21) + resize(s3_cv_p, 21);
            s4_y <= sat10_u(v_sum_y(19 downto 10));
            if y_only_r = '1' then
                s4_u <= s2_u_in;
                s4_v <= s2_v_in;
            else
                s4_u <= sat10_u(v_sum_u(19 downto 10));
                s4_v <= sat10_u(v_sum_v(19 downto 10));
            end if;

            -- S5: brightness multiply.
            s5_y_mul <= s4_y * bright_r;
            s5_u     <= s4_u;
            s5_v     <= s4_v;

            -- S6: extract Y; sat mul.
            s6_y <= s5_y_mul(19 downto 10);
            v_u_c := signed('0' & std_logic_vector(s5_u)) - to_signed(512, 12);
            v_v_c := signed('0' & std_logic_vector(s5_v)) - to_signed(512, 12);
            s6_u_mul <= v_u_c * signed('0' & std_logic_vector(sat_r));
            s6_v_mul <= v_v_c * signed('0' & std_logic_vector(sat_r));

            -- S7: re-center + clamp.
            s7_y <= s6_y;
            v_u_re := resize(shift_right(s6_u_mul, 10), 14) +
                      to_signed(512, 14);
            v_v_re := resize(shift_right(s6_v_mul, 10), 14) +
                      to_signed(512, 14);
            s7_u <= sat10_s(v_u_re);
            s7_v <= sat10_s(v_v_re);

            -- S8: invert + final.
            if invert_r = '1' then
                s8_y <= to_unsigned(1023, 10) - s7_y;
            else
                s8_y <= s7_y;
            end if;
            s8_u <= s7_u;
            s8_v <= s7_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s8_y);
    data_out.u       <= std_logic_vector(s8_u);
    data_out.v       <= std_logic_vector(s8_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture echotube;
