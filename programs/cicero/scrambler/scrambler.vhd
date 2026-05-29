-- Scrambler: address-XOR datamosh.
--
-- A 2048-deep BRAM stores incoming Y, U, V (3 BRAMs total, sharing the
-- write address).  At read time, the read address is derived as
--   rd_addr = (wr_addr - read_delay) XOR scramble_mask
-- where scramble_mask is the active high bits derived from the Scramble
-- slider (top bits) AND'd with a per-frame jitter (when Drift is on).
-- This rearranges pixels into glitchy blocks whose size is controlled
-- by Block Size (which masks the upper bits of scramble_mask to align
-- block boundaries).  Dropout sporadically replaces pixels with the
-- prior frame's value via a separate read-modify-write feedback layer.
--
-- BRAM: 3 line buffers × 1 BRAM each = 3 BRAMs.
-- Designed for 74.25 MHz HD timing on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Block Size  (top bits of scramble mask)
--   registers_in(1) = Y Shift     (vertical-line shift for tearing)
--   registers_in(2) = Dropout     (chance of holding previous pixel)
--   registers_in(3) = Mix         (wet/dry blend)
--   registers_in(4) = Brightness
--   registers_in(5) = Saturation
--   registers_in(6) = Switches:
--     b0 Mode    (0=XOR / 1=Reverse — reverse address)
--     b1 Animate (0=Static / 1=Drift mask each frame)
--     b2 Channel (0=YUV scrambled / 1=Y only)
--     b3 Tint    (apply blue-purple tint on glitched pixels)
--     b4 Invert
--   registers_in(7) = Scramble (slider, KEY)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture scrambler of program_top is

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

    -- Param latches
    signal block_size_r : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal y_shift_r    : unsigned(9 downto 0) := (others => '0');
    signal dropout_r    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal mix_r        : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal inv_mix_r    : unsigned(9 downto 0) := to_unsigned(123, 10);
    signal bright_r     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal sat_r        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal scramble_r   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal reverse_r    : std_logic := '0';
    signal drift_r      : std_logic := '1';
    signal y_only_r     : std_logic := '0';
    signal tint_r       : std_logic := '0';
    signal invert_r     : std_logic := '0';

    -- Per-frame drift mask (XOR-able with scramble base)
    signal drift_mask_r : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');

    -- Per-line write gate.  When y_shift_r > 0, BRAM is only written every
    -- Nth scanline (where N = top 5 bits of y_shift_r + 1) — earlier lines
    -- persist in the buffer to create vertical tearing.
    signal write_gate_r  : std_logic := '1';
    signal line_count_r  : unsigned(4 downto 0) := (others => '0');

    -- Active-x counter
    signal active_x       : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal active_width_r : unsigned(C_BUF_DEPTH - 1 downto 0) :=
                                to_unsigned(1920 mod C_BUF_SIZE, C_BUF_DEPTH);

    -- BRAM (3 line buffers)
    type t_bram is array (0 to C_BUF_SIZE - 1)
        of std_logic_vector(9 downto 0);
    signal bram_y : t_bram := (others => "0000000000");
    signal bram_u : t_bram := (others => "1000000000");
    signal bram_v : t_bram := (others => "1000000000");

    -- Read addr (combinational), registered values.
    signal rd_addr   : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal wr_addr   : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal rd_y_d    : std_logic_vector(9 downto 0) := (others => '0');
    signal rd_u_d    : std_logic_vector(9 downto 0) := (others => '0');
    signal rd_v_d    : std_logic_vector(9 downto 0) := (others => '0');

    -- LFSR for dropout decisions
    signal drop_lfsr : std_logic_vector(15 downto 0) := x"E7C1";

    -- ------------------------------------------------------------------
    -- Pipeline signals
    -- ------------------------------------------------------------------
    signal s0_y : unsigned(9 downto 0);
    signal s0_u : unsigned(9 downto 0);
    signal s0_v : unsigned(9 downto 0);

    signal s1_y_dry : unsigned(9 downto 0);
    signal s1_u_dry : unsigned(9 downto 0);
    signal s1_v_dry : unsigned(9 downto 0);
    signal s1_y_wet : unsigned(9 downto 0);
    signal s1_u_wet : unsigned(9 downto 0);
    signal s1_v_wet : unsigned(9 downto 0);

    signal s2_y_drop : unsigned(9 downto 0);
    signal s2_u_drop : unsigned(9 downto 0);
    signal s2_v_drop : unsigned(9 downto 0);
    signal s2_y_dry  : unsigned(9 downto 0);
    signal s2_u_dry  : unsigned(9 downto 0);
    signal s2_v_dry  : unsigned(9 downto 0);

    -- S3: mix raw products.
    signal s3a_mix_y  : unsigned(19 downto 0);
    signal s3a_inv_y  : unsigned(19 downto 0);
    signal s3a_mix_u  : unsigned(19 downto 0);
    signal s3a_inv_u  : unsigned(19 downto 0);
    signal s3a_mix_v  : unsigned(19 downto 0);
    signal s3a_inv_v  : unsigned(19 downto 0);

    -- S4: mix sums.
    signal s4_y : unsigned(9 downto 0);
    signal s4_u : unsigned(9 downto 0);
    signal s4_v : unsigned(9 downto 0);
    signal s4_glitched : std_logic;

    -- S5: brightness mul.
    signal s5_y_mul : unsigned(19 downto 0);
    signal s5_u     : unsigned(9 downto 0);
    signal s5_v     : unsigned(9 downto 0);
    signal s5_glitched : std_logic;

    -- S6: sat mul.
    signal s6_y     : unsigned(9 downto 0);
    signal s6_u_mul : signed(22 downto 0);
    signal s6_v_mul : signed(22 downto 0);
    signal s6_glitched : std_logic;

    -- S7: re-center + tint.
    signal s7_y : unsigned(9 downto 0);
    signal s7_u : unsigned(9 downto 0);
    signal s7_v : unsigned(9 downto 0);

    -- S8: invert.
    signal s8_y : unsigned(9 downto 0);
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);

    function sat10_s(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

begin

    -- ========================================================================
    -- Position + param latch + drift LFSR
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_fb     : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_h_edge := '0';
            v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- Dropout LFSR runs every cycle.
            v_fb := drop_lfsr(15) xor drop_lfsr(14) xor drop_lfsr(12) xor
                    drop_lfsr(3);
            drop_lfsr <= drop_lfsr(14 downto 0) & v_fb;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
                active_x <= (others => '0');
                -- Y-shift write gate: skip BRAM writes for N-1 of every N
                -- scanlines, where N = (y_shift_r>>5) + 1.
                if y_shift_r(9 downto 5) = "00000" then
                    write_gate_r <= '1';
                    line_count_r <= (others => '0');
                elsif line_count_r >= y_shift_r(9 downto 5) then
                    write_gate_r <= '1';
                    line_count_r <= (others => '0');
                else
                    write_gate_r <= '0';
                    line_count_r <= line_count_r + 1;
                end if;
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

                block_size_r <= unsigned(registers_in(0));
                y_shift_r    <= unsigned(registers_in(1));
                dropout_r    <= unsigned(registers_in(2));
                mix_r        <= unsigned(registers_in(3));
                inv_mix_r    <= to_unsigned(1023, 10) - unsigned(registers_in(3));
                bright_r     <= unsigned(registers_in(4));
                sat_r        <= unsigned(registers_in(5));
                reverse_r    <= registers_in(6)(0);
                drift_r      <= registers_in(6)(1);
                y_only_r     <= registers_in(6)(2);
                tint_r       <= registers_in(6)(3);
                invert_r     <= registers_in(6)(4);
                scramble_r   <= unsigned(registers_in(7));

                if registers_in(6)(1) = '1' then
                    -- More dramatic drift step: use full LFSR low byte.
                    drift_mask_r <= drift_mask_r +
                        resize(unsigned(drop_lfsr(8 downto 0)), C_BUF_DEPTH);
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
        variable v_mask  : unsigned(C_BUF_DEPTH - 1 downto 0);
        variable v_block : unsigned(C_BUF_DEPTH - 1 downto 0);
        variable v_rev   : unsigned(C_BUF_DEPTH - 1 downto 0);
        variable v_rd    : unsigned(C_BUF_DEPTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            wr_addr <= active_x;

            -- Compute scramble mask: top bits of scramble_r XOR drift_mask.
            v_mask := resize(scramble_r, C_BUF_DEPTH) xor drift_mask_r;

            -- Block size: zero out the bottom `block_top` bits of mask.
            -- Higher block_size_r → preserve larger blocks (smaller mask LSBs).
            v_block := v_mask;
            if block_size_r(9 downto 8) = "11" then
                v_block := v_mask(C_BUF_DEPTH - 1 downto 5) & "00000";
            elsif block_size_r(9 downto 8) = "10" then
                v_block := v_mask(C_BUF_DEPTH - 1 downto 4) & "0000";
            elsif block_size_r(9 downto 8) = "01" then
                v_block := v_mask(C_BUF_DEPTH - 1 downto 3) & "000";
            else
                v_block := v_mask;
            end if;

            if reverse_r = '0' then
                v_rd := active_x xor v_block;
            else
                -- Reverse mode: bit-reverse low bits of active_x.
                for i in 0 to 6 loop
                    v_rev(i) := active_x(6 - i);
                end loop;
                v_rev(C_BUF_DEPTH - 1 downto 7) :=
                    active_x(C_BUF_DEPTH - 1 downto 7);
                v_rd := v_rev xor v_block;
            end if;

            if active_width_r > 0 and v_rd >= active_width_r then
                v_rd := v_rd - active_width_r;
            end if;
            rd_addr <= v_rd;
        end if;
    end process p_addr;

    -- BRAM writes gated by write_gate_r so Y Shift can hold lines.
    p_bram_wy : process(clk) begin
        if rising_edge(clk) then
            if write_gate_r = '1' then
                bram_y(to_integer(wr_addr)) <= std_logic_vector(s0_y);
            end if;
        end if;
    end process;
    p_bram_wu : process(clk) begin
        if rising_edge(clk) then
            if write_gate_r = '1' then
                bram_u(to_integer(wr_addr)) <= std_logic_vector(s0_u);
            end if;
        end if;
    end process;
    p_bram_wv : process(clk) begin
        if rising_edge(clk) then
            if write_gate_r = '1' then
                bram_v(to_integer(wr_addr)) <= std_logic_vector(s0_v);
            end if;
        end if;
    end process;
    p_bram_ry : process(clk) begin
        if rising_edge(clk) then rd_y_d <= bram_y(to_integer(rd_addr)); end if;
    end process;
    p_bram_ru : process(clk) begin
        if rising_edge(clk) then rd_u_d <= bram_u(to_integer(rd_addr)); end if;
    end process;
    p_bram_rv : process(clk) begin
        if rising_edge(clk) then rd_v_d <= bram_v(to_integer(rd_addr)); end if;
    end process;

    -- ========================================================================
    -- Pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_drop_hit : std_logic;
        variable v_u_c      : signed(11 downto 0);
        variable v_v_c      : signed(11 downto 0);
        variable v_u_re     : signed(13 downto 0);
        variable v_v_re     : signed(13 downto 0);
        variable v_sum_y    : unsigned(19 downto 0);
        variable v_sum_u    : unsigned(19 downto 0);
        variable v_sum_v    : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: register input
            s0_y <= unsigned(data_in.y);
            s0_u <= unsigned(data_in.u);
            s0_v <= unsigned(data_in.v);

            -- S1: dry and wet (BRAM read) versions.  Y-only mode keeps UV dry.
            s1_y_dry <= s0_y; s1_u_dry <= s0_u; s1_v_dry <= s0_v;
            s1_y_wet <= unsigned(rd_y_d);
            if y_only_r = '1' then
                s1_u_wet <= s0_u;
                s1_v_wet <= s0_v;
            else
                s1_u_wet <= unsigned(rd_u_d);
                s1_v_wet <= unsigned(rd_v_d);
            end if;

            -- S2: dropout: chance of replacing wet with dry (or 0).
            -- LFSR bottom byte compared to dropout_r/4.
            if unsigned(drop_lfsr(7 downto 0)) <
               dropout_r(9 downto 2) then
                s2_y_drop <= s1_y_dry;  -- dropout fallback to dry
                s2_u_drop <= s1_u_dry;
                s2_v_drop <= s1_v_dry;
            else
                s2_y_drop <= s1_y_wet;
                s2_u_drop <= s1_u_wet;
                s2_v_drop <= s1_v_wet;
            end if;
            s2_y_dry <= s1_y_dry;
            s2_u_dry <= s1_u_dry;
            s2_v_dry <= s1_v_dry;

            -- S3: mix raw products.
            s3a_mix_y <= mix_r * s2_y_drop;
            s3a_inv_y <= inv_mix_r * s2_y_dry;
            s3a_mix_u <= mix_r * s2_u_drop;
            s3a_inv_u <= inv_mix_r * s2_u_dry;
            s3a_mix_v <= mix_r * s2_v_drop;
            s3a_inv_v <= inv_mix_r * s2_v_dry;

            -- S4: sum + shift.
            v_sum_y := s3a_mix_y + s3a_inv_y;
            v_sum_u := s3a_mix_u + s3a_inv_u;
            v_sum_v := s3a_mix_v + s3a_inv_v;
            s4_y <= v_sum_y(19 downto 10);
            s4_u <= v_sum_u(19 downto 10);
            s4_v <= v_sum_v(19 downto 10);
            -- glitched if mix_r > 0 (signals when wet is at least partially used).
            if mix_r > to_unsigned(256, 10) then
                s4_glitched <= '1';
            else
                s4_glitched <= '0';
            end if;

            -- S5: brightness mul.
            s5_y_mul <= s4_y * bright_r;
            s5_u     <= s4_u;
            s5_v     <= s4_v;
            s5_glitched <= s4_glitched;

            -- S6: extract Y; sat mul.
            s6_y <= s5_y_mul(19 downto 10);
            v_u_c := signed('0' & std_logic_vector(s5_u)) - to_signed(512, 12);
            v_v_c := signed('0' & std_logic_vector(s5_v)) - to_signed(512, 12);
            s6_u_mul <= v_u_c * signed('0' & std_logic_vector(sat_r));
            s6_v_mul <= v_v_c * signed('0' & std_logic_vector(sat_r));
            s6_glitched <= s5_glitched;

            -- S7: re-center, optional tint on glitched pixels.
            v_u_re := resize(shift_right(s6_u_mul, 10), 14) +
                      to_signed(512, 14);
            v_v_re := resize(shift_right(s6_v_mul, 10), 14) +
                      to_signed(512, 14);
            s7_y <= s6_y;
            if tint_r = '1' and s6_glitched = '1' then
                -- Bias toward blue-magenta on scrambled.
                s7_u <= sat10_s(v_u_re + to_signed(128, 14));
                s7_v <= sat10_s(v_v_re - to_signed(64, 14));
            else
                s7_u <= sat10_s(v_u_re);
                s7_v <= sat10_s(v_v_re);
            end if;

            -- S8: invert.
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

end architecture scrambler;
