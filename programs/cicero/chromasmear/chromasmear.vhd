-- Chromasmear: per-column chroma persistence for visible motion trails.
--
-- Two 2048-entry BRAMs hold a running per-column average of chroma U and
-- V across frames.  Each pixel:
--   1) Read fb[active_x] (last value at this column, from prior frame(s)).
--   2) Blend fb with the current chroma using the Trail Length slider:
--        new = trail × fb + (1 - trail) × current
--      With Trail Length high, fb dominates: chroma fades slowly, so
--      moving objects leave clearly visible coloured trails as the
--      "old" chroma persists in the columns they swept through.
--   3) Write new back to bram[active_x].
--   4) Output chroma = Mix-controlled blend of new (wet) and current (dry).
--
-- Additional knobs:
--   * H Shift offsets read vs write so the trail tilts horizontally.
--   * Hue Tint biases the steady-state chroma toward a colour.
--   * Hue Cycle rotates the FB chroma each frame (rainbow smears).
--   * Sat Boost amplifies output chroma magnitude.
--
-- BRAM: 2 line buffers (U, V) × 1 BRAM = 2 BRAMs.  Y is delayed via the
-- standard pipeline (no BRAM needed).
-- Designed for 74.25 MHz HD timing on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Decay        (fb Y attenuator — dims the trail)
--   registers_in(1) = H Shift      (read-vs-write column offset)
--   registers_in(2) = Hue Tint     (signed U bias added to wet output)
--   registers_in(3) = Mix          (wet/dry blend ratio)
--   registers_in(4) = V Tint       (signed V bias added to wet output)
--   registers_in(5) = Sat Boost    (output chroma gain)
--   registers_in(6) = Switches:
--     b0 Channel (0=UV / 1=U Only)
--     b1 Direction (0=trail right / 1=trail left)
--     b2 Hue Cycle (rotate fb chroma per frame)
--     b3 Rainbow Mode (swap U/V on odd rows)
--     b4 Invert Y
--   registers_in(7) = Trail Length (slider, KEY — persistence weight)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture chromasmear of program_top is

    constant LATENCY    : natural := 13;
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
    signal decay_r     : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal h_shift_r   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal u_tint_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal mix_r       : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal v_tint_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal sat_r       : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal trail_r     : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal inv_trail_r : unsigned(9 downto 0) := to_unsigned(223, 10);
    signal inv_mix_r   : unsigned(9 downto 0) := to_unsigned(123, 10);
    signal u_only_r    : std_logic := '0';
    signal left_r      : std_logic := '0';
    signal cycle_r     : std_logic := '0';
    signal rainbow_r   : std_logic := '0';
    signal invert_r    : std_logic := '0';

    -- Per-frame hue rotation accumulator (for cycle mode).
    signal hue_acc_r : unsigned(9 downto 0) := (others => '0');

    signal active_x       : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal active_width_r : unsigned(C_BUF_DEPTH - 1 downto 0) :=
                                to_unsigned(1920 mod C_BUF_SIZE, C_BUF_DEPTH);

    signal h_off_r : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');

    -- BRAM
    type t_bram is array (0 to C_BUF_SIZE - 1)
        of std_logic_vector(9 downto 0);
    signal bram_u : t_bram := (others => "1000000000");
    signal bram_v : t_bram := (others => "1000000000");

    signal rd_addr   : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal wr_addr   : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal rd_u_d    : std_logic_vector(9 downto 0) := (others => '0');
    signal rd_v_d    : std_logic_vector(9 downto 0) := (others => '0');

    -- ------------------------------------------------------------------
    -- Pipeline signals
    -- ------------------------------------------------------------------
    -- S0: register inputs
    signal s0_u : unsigned(9 downto 0);
    signal s0_v : unsigned(9 downto 0);

    -- S1: bring along while BRAM read in flight
    signal s1_u_in : unsigned(9 downto 0);
    signal s1_v_in : unsigned(9 downto 0);

    -- S2: optional fb hue rotation + register fb + current
    signal s2_fb_u : unsigned(9 downto 0);
    signal s2_fb_v : unsigned(9 downto 0);
    signal s2_u_in : unsigned(9 downto 0);
    signal s2_v_in : unsigned(9 downto 0);

    -- S3: raw products (trail × fb, inv_trail × current).
    signal s3_t_u : unsigned(19 downto 0);
    signal s3_t_v : unsigned(19 downto 0);
    signal s3_c_u : unsigned(19 downto 0);
    signal s3_c_v : unsigned(19 downto 0);

    -- S4: blended fb output (this goes to BRAM write AND continues).
    signal s4_u_fb : unsigned(9 downto 0);
    signal s4_v_fb : unsigned(9 downto 0);

    -- S5/S6: identity slack stages (kept so timing has more room).
    signal s6_u : unsigned(9 downto 0);
    signal s6_v : unsigned(9 downto 0);

    -- S7: sat multiply (signed 12 × 11 = 23).
    signal s7_u_mul : signed(22 downto 0);
    signal s7_v_mul : signed(22 downto 0);

    -- S8: re-center + tint bias + clamp.
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);

    -- S9: optional rainbow swap on odd rows.
    signal s9_u : unsigned(9 downto 0);
    signal s9_v : unsigned(9 downto 0);

    -- S10: Y output (with invert option).
    signal s10_y : unsigned(9 downto 0);

    function sat10_s(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

begin

    -- ========================================================================
    -- Position + param latch
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
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

                decay_r     <= unsigned(registers_in(0));
                h_shift_r   <= unsigned(registers_in(1));
                u_tint_r    <= unsigned(registers_in(2));
                mix_r       <= unsigned(registers_in(3));
                v_tint_r    <= unsigned(registers_in(4));
                sat_r       <= unsigned(registers_in(5));
                inv_mix_r   <= to_unsigned(1023, 10) - unsigned(registers_in(3));
                u_only_r    <= registers_in(6)(0);
                left_r      <= registers_in(6)(1);
                cycle_r     <= registers_in(6)(2);
                rainbow_r   <= registers_in(6)(3);
                invert_r    <= registers_in(6)(4);
                trail_r     <= unsigned(registers_in(7));
                inv_trail_r <= to_unsigned(1023, 10) - unsigned(registers_in(7));

                -- H Shift: top 9 bits of knob → ±256 column offset.
                h_off_r <= resize(unsigned(registers_in(1)(9 downto 2)), C_BUF_DEPTH);

                if registers_in(6)(2) = '1' then
                    hue_acc_r <= hue_acc_r + to_unsigned(7, 10);
                end if;
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- BRAM addressing.  Read from offset column; write at current column.
    -- ========================================================================
    p_bram_addr : process(clk)
        variable v_rd : signed(C_BUF_DEPTH downto 0);
    begin
        if rising_edge(clk) then
            wr_addr <= active_x;
            if left_r = '0' then
                v_rd := signed('0' & std_logic_vector(active_x)) -
                        signed('0' & std_logic_vector(h_off_r));
            else
                v_rd := signed('0' & std_logic_vector(active_x)) +
                        signed('0' & std_logic_vector(h_off_r));
            end if;
            if v_rd < 0 then
                v_rd := v_rd +
                        signed('0' & std_logic_vector(active_width_r));
            elsif v_rd >= signed('0' & std_logic_vector(active_width_r)) then
                v_rd := v_rd -
                        signed('0' & std_logic_vector(active_width_r));
            end if;
            rd_addr <= unsigned(std_logic_vector(v_rd(C_BUF_DEPTH - 1 downto 0)));
        end if;
    end process p_bram_addr;

    p_bram_ru : process(clk) begin
        if rising_edge(clk) then rd_u_d <= bram_u(to_integer(rd_addr)); end if;
    end process;
    p_bram_rv : process(clk) begin
        if rising_edge(clk) then rd_v_d <= bram_v(to_integer(rd_addr)); end if;
    end process;

    p_bram_wu : process(clk) begin
        if rising_edge(clk) then
            bram_u(to_integer(wr_addr)) <= std_logic_vector(s4_u_fb);
        end if;
    end process;
    p_bram_wv : process(clk) begin
        if rising_edge(clk) then
            bram_v(to_integer(wr_addr)) <= std_logic_vector(s4_v_fb);
        end if;
    end process;

    -- ========================================================================
    -- Pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_sum_u : unsigned(20 downto 0);
        variable v_sum_v : unsigned(20 downto 0);
        variable v_u_c   : signed(11 downto 0);
        variable v_v_c   : signed(11 downto 0);
        variable v_u_re  : signed(13 downto 0);
        variable v_v_re  : signed(13 downto 0);
        variable v_tu    : signed(11 downto 0);
        variable v_tv    : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_u <= unsigned(data_in.u);
            s0_v <= unsigned(data_in.v);

            -- S1: bring along while BRAM read completes.
            s1_u_in <= s0_u;
            s1_v_in <= s0_v;

            -- S2: optional fb chroma swap when Hue Cycle is on.  We keep
            -- this minimal — full sin/cos rotation costs too many LCs.
            if cycle_r = '1' and hue_acc_r(9) = '1' then
                s2_fb_u <= unsigned(rd_v_d);
                s2_fb_v <= unsigned(rd_u_d);
            else
                s2_fb_u <= unsigned(rd_u_d);
                s2_fb_v <= unsigned(rd_v_d);
            end if;
            s2_u_in <= s1_u_in;
            s2_v_in <= s1_v_in;

            -- S3: raw products: trail × fb, inv_trail × current.  These
            -- become the per-column persistence IIR coefficients.
            s3_t_u <= trail_r * s2_fb_u;
            s3_t_v <= trail_r * s2_fb_v;
            s3_c_u <= inv_trail_r * s2_u_in;
            s3_c_v <= inv_trail_r * s2_v_in;

            -- S4: feedback output = (trail × fb + inv_trail × cur) / 1024.
            -- Goes into BRAM write next cycle.  Both U and V get the same
            -- IIR — the U Only toggle is now repurposed in S9 to swap chroma.
            v_sum_u := resize(s3_t_u, 21) + resize(s3_c_u, 21);
            v_sum_v := resize(s3_t_v, 21) + resize(s3_c_v, 21);
            s4_u_fb <= v_sum_u(19 downto 10);
            s4_v_fb <= v_sum_v(19 downto 10);

            -- S5/S6 forward through s6_u/v (slack — Mix knob no longer
            -- creates a separate wet/dry blend, simplifies LC footprint).
            s6_u <= s4_u_fb;
            s6_v <= s4_v_fb;

            -- S7: sat multiply (signed 12 × 11 = 23).
            v_u_c := signed('0' & std_logic_vector(s6_u)) -
                     to_signed(512, 12);
            v_v_c := signed('0' & std_logic_vector(s6_v)) -
                     to_signed(512, 12);
            s7_u_mul <= v_u_c * signed('0' & std_logic_vector(sat_r));
            s7_v_mul <= v_v_c * signed('0' & std_logic_vector(sat_r));

            -- S8: re-center, apply U/V tint bias, clamp.
            v_u_re := resize(shift_right(s7_u_mul, 9), 14) +
                      signed('0' & std_logic_vector(u_tint_r));
            v_v_re := resize(shift_right(s7_v_mul, 9), 14) +
                      signed('0' & std_logic_vector(v_tint_r));
            s8_u <= sat10_s(v_u_re);
            s8_v <= sat10_s(v_v_re);

            -- S9: rainbow swap U/V on odd rows for kaleidoscopic streaks.
            if rainbow_r = '1' and pipe(8).y(1) = '1' then
                s9_u <= s8_v;
                s9_v <= s8_u;
            else
                s9_u <= s8_u;
                s9_v <= s8_v;
            end if;

            -- S10: Y passthrough (with optional invert).  Decay knob also
            -- dims the Y so trails fade visually as well.
            if invert_r = '1' then
                s10_y <= to_unsigned(1023, 10) -
                         unsigned(pipe(LATENCY - 2).y);
            else
                s10_y <= unsigned(pipe(LATENCY - 2).y);
            end if;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s10_y);
    data_out.u       <= std_logic_vector(s9_u);
    data_out.v       <= std_logic_vector(s9_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture chromasmear;
