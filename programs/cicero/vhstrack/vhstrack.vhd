-- VHS Track: tape-tracking-error emulation for incoming video.
--
-- Three classic VHS artifacts are stacked on the incoming video:
--   1. Horizontal skew: each row reads from a line buffer at an offset
--      that varies with row position, giving the curved-pull look of a
--      mis-tracked tape.
--   2. Chroma drift: U/V are individually phase-modulated by a sine
--      wave that scrolls vertically, producing color smear.
--   3. Tape hiss + tracking band: LFSR noise overlays luma; a movable
--      horizontal band scrambles the picture to mimic head-switch flutter.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).
--
-- Register map:
--   registers_in(0) = Skew Top       (rotary 1 — H shift at top of frame)
--   registers_in(1) = Band Pos       (rotary 2 — tracking band Y center)
--   registers_in(2) = Band Width     (rotary 3 — vertical band size)
--   registers_in(3) = Chroma Drift   (rotary 4 — color smear amount)
--   registers_in(4) = Drift Speed    (rotary 5 — chroma drift rate)
--   registers_in(5) = Tape Hiss      (rotary 6 — luma noise gain)
--   registers_in(6) = Switches:
--     b0   Color    (0=Full color / 1=B&W)
--     b1   Sat Down (0=Normal / 1=Under-saturated VHS look)
--     b2   Band     (0=Off / 1=Show tracking band)
--     b3   Snow     (0=Off / 1=Full-frame static)
--     b4   Invert
--   registers_in(7) = Skew Amount (slider, KEY — overall tracking skew)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture vhstrack of program_top is

    constant LATENCY : natural := 10;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal field_flip_r : std_logic := '0';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal skew_top_r  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal band_pos_r  : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal band_w_r    : unsigned(9 downto 0) := to_unsigned(80,  10);
    signal cdrift_r    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal dspeed_r    : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal hiss_r      : unsigned(9 downto 0) := to_unsigned(64,  10);
    signal skew_r      : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal color_r     : std_logic := '0';
    signal sat_down_r  : std_logic := '1';
    signal band_en_r   : std_logic := '1';
    signal snow_r      : std_logic := '0';
    signal invert_r    : std_logic := '0';

    -- Chroma drift phase
    signal cdrift_phase_r : unsigned(11 downto 0) := (others => '0');

    -- LFSR
    signal lfsr_r : unsigned(15 downto 0) := to_unsigned(38271, 16);

    -- Line buffer ports
    signal lb_ab      : std_logic;
    signal lb_wr_addr : unsigned(10 downto 0);
    signal lb_rd_addr : unsigned(10 downto 0);
    signal lb_y_out   : std_logic_vector(9 downto 0);
    signal lb_u_out   : std_logic_vector(9 downto 0);
    signal lb_v_out   : std_logic_vector(9 downto 0);

    -- 64-entry quarter-wave sine LUT 0..127
    type t_quad is array (0 to 63) of unsigned(7 downto 0);
    constant C_SIN_QUAD : t_quad := (
        to_unsigned(  0, 8), to_unsigned(  3, 8), to_unsigned(  6, 8), to_unsigned( 10, 8),
        to_unsigned( 13, 8), to_unsigned( 16, 8), to_unsigned( 19, 8), to_unsigned( 22, 8),
        to_unsigned( 25, 8), to_unsigned( 28, 8), to_unsigned( 31, 8), to_unsigned( 35, 8),
        to_unsigned( 38, 8), to_unsigned( 41, 8), to_unsigned( 44, 8), to_unsigned( 47, 8),
        to_unsigned( 49, 8), to_unsigned( 52, 8), to_unsigned( 55, 8), to_unsigned( 58, 8),
        to_unsigned( 61, 8), to_unsigned( 63, 8), to_unsigned( 66, 8), to_unsigned( 69, 8),
        to_unsigned( 71, 8), to_unsigned( 74, 8), to_unsigned( 76, 8), to_unsigned( 79, 8),
        to_unsigned( 81, 8), to_unsigned( 83, 8), to_unsigned( 86, 8), to_unsigned( 88, 8),
        to_unsigned( 90, 8), to_unsigned( 92, 8), to_unsigned( 94, 8), to_unsigned( 96, 8),
        to_unsigned( 98, 8), to_unsigned(100, 8), to_unsigned(102, 8), to_unsigned(104, 8),
        to_unsigned(106, 8), to_unsigned(107, 8), to_unsigned(109, 8), to_unsigned(110, 8),
        to_unsigned(112, 8), to_unsigned(113, 8), to_unsigned(114, 8), to_unsigned(116, 8),
        to_unsigned(117, 8), to_unsigned(118, 8), to_unsigned(119, 8), to_unsigned(120, 8),
        to_unsigned(121, 8), to_unsigned(122, 8), to_unsigned(123, 8), to_unsigned(124, 8),
        to_unsigned(124, 8), to_unsigned(125, 8), to_unsigned(125, 8), to_unsigned(126, 8),
        to_unsigned(126, 8), to_unsigned(127, 8), to_unsigned(127, 8), to_unsigned(127, 8));

    -- Pre-computed skew offset per row (computed at hsync)
    signal row_skew_r : signed(10 downto 0) := (others => '0');
    -- Pre-computed chroma U offset and V offset per row
    signal cu_offset_r : signed(10 downto 0) := (others => '0');
    signal cv_offset_r : signed(10 downto 0) := (others => '0');

    -- Per-frame constant: skew_top * skew (registered every cycle so the
    -- multiplier doesn't sit on a long combinational path into row_skew_r).
    signal skew_mul_r : unsigned(19 downto 0) := (others => '0');
    -- Per-cycle registered sin(pixel_y + phase) and its multiplication with
    -- cdrift_r — split across stages to keep critical path short.
    signal sin_u_r    : signed(9 downto 0) := (others => '0');
    signal sin_v_r    : signed(9 downto 0) := (others => '0');
    signal cdrift_u_mul_r : signed(19 downto 0) := (others => '0');
    signal cdrift_v_mul_r : signed(19 downto 0) := (others => '0');

    -- Pipeline signals
    signal s0_x      : unsigned(11 downto 0);
    signal s0_y      : unsigned(11 downto 0);
    signal s0_lfsr   : unsigned(15 downto 0);

    -- S1: rd_addr = pixel_x + row_skew_r (clamped to 0..1919)
    signal s1_rd_x : signed(12 downto 0);
    signal s1_y    : unsigned(11 downto 0);
    signal s1_lfsr : unsigned(15 downto 0);

    -- S2: clamped rd_addr
    signal s2_rd_addr : unsigned(10 downto 0);
    signal s2_y       : unsigned(11 downto 0);
    signal s2_lfsr    : unsigned(15 downto 0);

    -- S3: buffer read happens; output appears next cycle.

    -- S4: latched line buffer output + LFSR + in-band check
    signal s4_y : unsigned(9 downto 0);
    signal s4_u : unsigned(9 downto 0);
    signal s4_v : unsigned(9 downto 0);
    signal s4_in_band : std_logic;
    signal s4_lfsr : unsigned(15 downto 0);
    signal s4_pixel_y : unsigned(11 downto 0);

    -- S5: combine luma noise + band noise + snow
    signal s5_y : unsigned(9 downto 0);
    signal s5_u : unsigned(9 downto 0);
    signal s5_v : unsigned(9 downto 0);

    -- S6: chroma offset shift
    signal s6_y : unsigned(9 downto 0);
    signal s6_u : unsigned(9 downto 0);
    signal s6_v : unsigned(9 downto 0);

    -- S7: saturation reduction + invert
    signal s7_y : unsigned(9 downto 0);
    signal s7_u : unsigned(9 downto 0);
    signal s7_v : unsigned(9 downto 0);

    function sat10s(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

    function quad_sin(ang : unsigned(9 downto 0)) return signed is
        variable v_q : unsigned(1 downto 0);
        variable v_idx : integer range 0 to 63;
        variable v_mag : unsigned(7 downto 0);
        variable v_s   : signed(9 downto 0);
    begin
        v_q := ang(9 downto 8);
        v_idx := to_integer(ang(7 downto 2));
        case v_q is
            when "00"   => v_mag := C_SIN_QUAD(v_idx);
            when "01"   => v_mag := C_SIN_QUAD(63 - v_idx);
            when "10"   => v_mag := C_SIN_QUAD(v_idx);
            when others => v_mag := C_SIN_QUAD(63 - v_idx);
        end case;
        if v_q(1) = '0' then
            v_s := signed(resize(v_mag, 10));
        else
            v_s := -signed(resize(v_mag, 10));
        end if;
        return v_s;
    end function;

begin

    -- Line buffers
    lb_y_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr,
            i_data    => data_in.y,
            o_data    => lb_y_out
        );
    lb_u_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr,
            i_data    => data_in.u,
            o_data    => lb_u_out
        );
    lb_v_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr,
            i_data    => data_in.v,
            o_data    => lb_v_out
        );

    lb_ab      <= field_flip_r;
    lb_wr_addr <= pixel_x(10 downto 0);
    lb_rd_addr <= s2_rd_addr;

    -- ========================================================================
    -- Position + parameters + per-row skew/chroma offset computation
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_drift  : unsigned(9 downto 0);
        variable v_tap    : std_logic;
        variable v_row_skew : signed(20 downto 0);
        variable v_y_phase  : unsigned(9 downto 0);
        variable v_cdrift_mul : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0';
            v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                v_h_edge := '1';
            end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                v_v_edge := '1';
            end if;

            -- LFSR
            v_tap := lfsr_r(15) xor lfsr_r(13) xor lfsr_r(12) xor lfsr_r(10);
            lfsr_r <= lfsr_r(14 downto 0) & v_tap;

            -- Always-on multipliers: registered every cycle so the
            -- combinational mul delay doesn't stretch into the per-row
            -- subtract chain at hsync.
            skew_mul_r <= skew_top_r * skew_r;

            -- Sin lookup of pixel_y + drift_phase, registered each cycle.
            v_y_phase := unsigned(std_logic_vector(pixel_y(9 downto 0))) +
                         cdrift_phase_r(11 downto 2);
            sin_u_r <= resize(quad_sin(v_y_phase), 10);
            sin_v_r <= resize(quad_sin(v_y_phase + to_unsigned(256, 10)), 10);

            -- Chroma drift multiply, registered each cycle (uses prev sin_r).
            cdrift_u_mul_r <= resize(sin_u_r *
                              signed('0' & std_logic_vector(cdrift_r)), 20);
            cdrift_v_mul_r <= resize(sin_v_r *
                              signed('0' & std_logic_vector(cdrift_r)), 20);

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
                field_flip_r <= not field_flip_r;
                -- Per-row subtract uses pre-registered mul result.
                row_skew_r <= resize(
                               shift_right(signed('0' & std_logic_vector(skew_mul_r)),
                                           10), 11) -
                              signed(resize(pixel_y(11 downto 4), 11));
                cu_offset_r <= resize(shift_right(cdrift_u_mul_r, 8), 11);
                cv_offset_r <= resize(shift_right(cdrift_v_mul_r, 8), 11);
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;
                skew_top_r <= unsigned(registers_in(0));
                band_pos_r <= unsigned(registers_in(1));
                band_w_r   <= unsigned(registers_in(2));
                cdrift_r   <= unsigned(registers_in(3));
                dspeed_r   <= unsigned(registers_in(4));
                hiss_r     <= unsigned(registers_in(5));
                color_r    <= registers_in(6)(0);
                sat_down_r <= registers_in(6)(1);
                band_en_r  <= registers_in(6)(2);
                snow_r     <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                skew_r     <= unsigned(registers_in(7));

                v_drift := "00000" & unsigned(registers_in(4)(9 downto 5));
                cdrift_phase_r <= cdrift_phase_r + ("00" & v_drift);
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_band_y_diff : signed(12 downto 0);
        variable v_in_band     : std_logic;
        variable v_y_w_noise   : signed(11 downto 0);
        variable v_u_off       : signed(11 downto 0);
        variable v_v_off       : signed(11 downto 0);
        variable v_chroma_u    : signed(11 downto 0);
        variable v_chroma_v    : signed(11 downto 0);
        variable v_u_scaled    : signed(11 downto 0);
        variable v_v_scaled    : signed(11 downto 0);
        variable v_noise       : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;
            s0_lfsr <= lfsr_r;

            -- S1: compute read address with skew applied
            s1_rd_x <= signed(resize(s0_x, 13)) + resize(row_skew_r, 13);
            s1_y    <= s0_y;
            s1_lfsr <= s0_lfsr;

            -- S2: clamp to 0..1919
            if s1_rd_x < 0 then
                s2_rd_addr <= (others => '0');
            elsif s1_rd_x > to_signed(1919, 13) then
                s2_rd_addr <= to_unsigned(1919, 11);
            else
                s2_rd_addr <= unsigned(std_logic_vector(s1_rd_x(10 downto 0)));
            end if;
            s2_y    <= s1_y;
            s2_lfsr <= s1_lfsr;

            -- S3: BRAM read happens

            -- S4: capture
            s4_y <= unsigned(lb_y_out);
            s4_u <= unsigned(lb_u_out);
            s4_v <= unsigned(lb_v_out);

            -- Check in-band: |y - band_pos*1.08| < band_w
            v_band_y_diff := signed(resize(s2_y, 13)) -
                             signed(resize(band_pos_r, 13));
            if v_band_y_diff < 0 then v_band_y_diff := -v_band_y_diff; end if;
            if band_en_r = '1' and unsigned(std_logic_vector(v_band_y_diff(10 downto 0))) <
               ("0" & band_w_r) then
                s4_in_band <= '1';
            else
                s4_in_band <= '0';
            end if;
            s4_lfsr <= s2_lfsr;
            s4_pixel_y <= s2_y;

            -- S5: noise + band + snow.  Noise scaled by hiss top bits via
            -- fixed shift (avoids combinational multiply on critical path).
            v_noise := signed(resize(s4_lfsr(8 downto 0), 11)) -
                       to_signed(255, 11);
            case to_integer(hiss_r(9 downto 7)) is
                when 0      => v_noise := to_signed(0, 11);
                when 1      => v_noise := resize(shift_right(v_noise, 4), 11);
                when 2      => v_noise := resize(shift_right(v_noise, 3), 11);
                when 3      => v_noise := resize(shift_right(v_noise, 2), 11);
                when others => v_noise := resize(shift_right(v_noise, 1), 11);
            end case;
            v_y_w_noise := signed(resize(s4_y, 12)) + resize(v_noise, 12);

            if snow_r = '1' then
                -- Full snow
                s5_y <= unsigned(std_logic_vector(s4_lfsr(9 downto 0)));
                s5_u <= to_unsigned(512, 10);
                s5_v <= to_unsigned(512, 10);
            elsif s4_in_band = '1' then
                -- Tracking band: mix snow heavily
                s5_y <= ("0" & s4_lfsr(8 downto 0));
                s5_u <= to_unsigned(512, 10);
                s5_v <= to_unsigned(512, 10);
            else
                s5_y <= sat10s(v_y_w_noise);
                s5_u <= s4_u;
                s5_v <= s4_v;
            end if;

            -- S6: chroma drift offset
            v_u_off := signed(resize(s5_u, 12)) + resize(cu_offset_r, 12);
            v_v_off := signed(resize(s5_v, 12)) + resize(cv_offset_r, 12);
            s6_u <= sat10s(v_u_off);
            s6_v <= sat10s(v_v_off);
            s6_y <= s5_y;

            -- S7: saturation/B&W/invert
            if color_r = '1' then
                -- B&W: neutral chroma
                s7_u <= to_unsigned(512, 10);
                s7_v <= to_unsigned(512, 10);
            elsif sat_down_r = '1' then
                -- Under-saturated: pull chroma toward neutral
                v_chroma_u := signed(resize(s6_u, 12)) - to_signed(512, 12);
                v_chroma_v := signed(resize(s6_v, 12)) - to_signed(512, 12);
                v_u_scaled := resize(shift_right(v_chroma_u, 1), 12)
                              + to_signed(512, 12);
                v_v_scaled := resize(shift_right(v_chroma_v, 1), 12)
                              + to_signed(512, 12);
                s7_u <= sat10s(v_u_scaled);
                s7_v <= sat10s(v_v_scaled);
            else
                s7_u <= s6_u;
                s7_v <= s6_v;
            end if;
            if invert_r = '1' then
                s7_y <= to_unsigned(1023, 10) - s6_y;
            else
                s7_y <= s6_y;
            end if;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s7_y);
    data_out.u       <= std_logic_vector(s7_u);
    data_out.v       <= std_logic_vector(s7_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture vhstrack;
