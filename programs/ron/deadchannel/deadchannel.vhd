-- Dead Channel: a whole dying analog chain in one program.
--
-- Three cicero programs are folded into one signal path, in the order the
-- artifacts really happen:
--
--   TAPE  (from cicero/vhstrack) - wrap-around per-row horizontal skew, a
--         rolling head-switch tear band, tape hiss and vertical chroma drift.
--   BEAM  (from cicero/chromaticab) - U/V read the line buffer from opposite
--         X positions, the split growing toward the screen edges.
--   TUBE  (from cicero/crt) - scanline or RGB-triad grille, corner vignette
--         and phosphor glow (horizontal IIR bleed on brights).
--
-- Differences from the three sources, deliberately:
--   * the skew is a wrapped DDA, not a clamped offset, so the picture folds
--     around the frame instead of smearing into the edge pixel at max;
--   * every control is live at every position of every other control (the
--     sources latched several knobs they never used);
--   * blanking gate, runtime-measured width/height, serration-guarded frame
--     anchor and interlace-correct per-line accumulators.
--
-- Register map:
--   registers_in(0) = Dropout  (rotary 1 - head-switch band height; 0 = no
--                               band at all, up = taller and faster rolling)
--   registers_in(1) = Hiss     (rotary 2 - tape noise on luma)
--   registers_in(2) = Drift    (rotary 3 - scrolling chroma wobble, depth
--                               and rate together)
--   registers_in(3) = Converge (rotary 4, BIPOLAR - beam misconvergence,
--                               centre = converged, ends = spread / squeeze)
--   registers_in(4) = Glow     (rotary 5 - phosphor bleed trail)
--   registers_in(5) = Tube     (rotary 6 - grille depth + vignette, floored
--                               so the Grille switch always reads)
--   registers_in(6) = switches:
--     b0  Grille  (0 = Scan lines / 1 = RGB Triad)
--     b1  Cast    (0 = Cool       / 1 = Sick)
--     b2  Chroma  (0 = Rich       / 1 = Faded)
--     b3  Band    (0 = Tear       / 1 = Flash)
--     b4  Invert  (0 = Off        / 1 = On)
--   registers_in(7) = Skew (slider, KEY, BIPOLAR - bounded tape pull: a flag
--                            at the top of the frame plus a head-switch tear,
--                            centre = straight, ends = a quarter-width bend)
--
-- Chroma note: this is a processing program, so the dry path is identity and
-- no U/V swap is applied.  The generated Cast tints are authored in HARDWARE
-- sense (data_out.u carries Cr / red-ness, data_out.v carries Cb / blue-ness).
--
-- BRAM: Y line buffer 10b x 2048 dual-bank (10 EBR) + U and V line buffers
-- 8b x 1024 dual-bank (4 EBR each) = 18 EBR.  Chroma is stored at half
-- horizontal resolution and 8 bits (steps of 4 codes, neutral 128 -> 512
-- exactly) purely to stay off the 30/32 EBR congestion cliff.
--
-- Pipeline depth (both the video data and the sync tap): 13 clocks, the same
-- in every mode.
--
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture deadchannel of program_top is

    -- 12 pipe taps; the output register makes the sync delay 13 clocks,
    -- identical in every mode.
    constant LATENCY : natural := 12;

    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ========================================================================
    -- Input registers / timing
    -- ========================================================================
    signal vs_r, hs_r, av_r, fl_r          : std_logic := '0';
    signal prev_hsync_n                    : std_logic := '1';
    signal prev_vsync_n                    : std_logic := '1';
    signal prev_avid                       : std_logic := '0';

    signal px        : unsigned(11 downto 0) := (others => '0');
    signal aline     : unsigned(11 downto 0) := (others => '0');
    signal act_w     : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal act_h     : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal frame_act : std_logic := '0';
    signal anc_fld   : std_logic := '0';
    signal ilace     : std_logic := '0';

    -- ========================================================================
    -- Parameters (latched once per frame at the active-video anchor)
    -- ========================================================================
    -- Only the values the pixel path actually reads are stored; the rest are
    -- reduced to their derived form at latch time.
    signal hiss_r   : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal grille_r : std_logic := '0';
    signal cast_r   : std_logic := '0';
    signal faded_r  : std_logic := '1';
    signal bmode_r  : std_logic := '0';
    signal inv_r    : std_logic := '0';

    -- Derived, per frame
    signal roll_step : signed(7 downto 0)  := (others => '0'); -- lines / frame
    signal conv_s    : signed(6 downto 0)  := (others => '0'); -- misconvergence
    signal drift_s   : unsigned(6 downto 0):= (others => '0');
    signal tube6     : unsigned(6 downto 0):= (others => '0'); -- vignette gain
    signal scan_amt  : unsigned(9 downto 0):= (others => '0'); -- grille depth
    signal triad_amt : unsigned(7 downto 0):= (others => '0');
    signal glow6     : unsigned(5 downto 0):= (others => '0');
    signal cx_r      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal cy_r      : unsigned(11 downto 0) := to_unsigned(540, 12);

    -- ========================================================================
    -- Tape transport state
    -- ========================================================================
    -- Skew profile.  NOT a cumulative shear: a cumulative DDA is unbounded by
    -- construction (even 1 px/line is a full screen width over 1080 lines and
    -- shreds the picture into diagonal slabs).  Instead the row offset is a
    -- BOUNDED position function - a decaying "flag" pull anchored at the top of
    -- the frame plus a second decaying pull anchored at the head-switch band -
    -- so the extremes of the slider bend the picture instead of destroying it.
    -- The whole slider -> amplitude -> decay-rate derivation runs FREE-RUNNING,
    -- one small op per cycle, and the frame anchor only samples the results.
    -- Doing it inside the anchor cone cost 13 MHz: an SPI read, a deadband, a
    -- variable shift and an abs all landed in one pixel-clock path.
    signal conv_d_r : signed(11 downto 0) := (others => '0');
    signal drop_r   : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal band_mul : unsigned(9 downto 0) := (others => '0');
    signal skew_d_r : signed(11 downto 0) := (others => '0');
    signal skew_z_r : signed(11 downto 0) := (others => '0');
    signal top_amp  : signed(11 downto 0) := (others => '0');  -- px, +-w/4
    -- Decay is EXPONENTIAL (bend - bend>>k), one subtract per line.  The
    -- linear "subtract a rate, clamp at zero" form chains two compares and an
    -- add into one cone and cost 6 MHz; the exponential form also happens to
    -- be the shape real tape flagging has.  dshift picks the rate from the
    -- raster class and interlace so the pull spans the same fraction of the
    -- frame in every mode.
    signal dshift   : unsigned(1 downto 0) := (others => '0');
    signal bend_seed: signed(15 downto 0) := (others => '0'); -- top_amp * 16
    signal bend2_sd : signed(15 downto 0) := (others => '0'); -- top_amp * 8
    signal wcls     : unsigned(1 downto 0) := (others => '0'); -- raster class
    signal row_pre  : signed(13 downto 0) := (others => '0'); -- offset pre-wrap
    signal bend1    : signed(15 downto 0) := (others => '0');  -- 1/16 px
    signal bend2    : signed(15 downto 0) := (others => '0');  -- 1/16 px
    signal row_eff  : unsigned(11 downto 0) := (others => '0'); -- + tear jog
    -- Band membership used to be |aline - band_pos| < half, which chains a
    -- subtract, an abs and two compares into ONE pixel-clock cone: 19 ns, the
    -- critical path of the whole design.  The four edges are registered once
    -- per frame instead, leaving a plain register-to-register compare per line.
    -- band_pos is signed and travels a little past both ends of the raster so
    -- the tear rolls off one edge and back on at the other.
    signal band_pos : signed(13 downto 0) := (others => '0');
    signal band_hlf : unsigned(11 downto 0) := to_unsigned(24, 12);
    signal bt1, bb1 : signed(13 downto 0) := (others => '0');
    signal bt2, bb2 : signed(13 downto 0) := (others => '0');
    signal aline_s  : signed(13 downto 0) := (others => '0');
    signal in_band  : std_logic := '0';
    signal near_bnd : std_logic := '0';
    signal in_band_p: std_logic := '0';
    signal dyd_r    : signed(12 downto 0) := (others => '0');
    signal dy_r     : unsigned(11 downto 0) := (others => '0');
    signal scan_par : std_logic := '0';

    -- Chroma drift: free-running sine * depth, sampled once per line.
    signal drift_ph  : unsigned(13 downto 0) := (others => '0');
    signal sin_r     : signed(8 downto 0)  := (others => '0');
    signal drift_mul : signed(15 downto 0) := (others => '0');
    signal drift_off : signed(7 downto 0)  := (others => '0');

    signal lfsr_r    : unsigned(15 downto 0) := to_unsigned(38271, 16);

    -- ========================================================================
    -- Line buffers
    -- ========================================================================
    signal lb_ab      : std_logic := '0';
    signal lb_wr_y    : unsigned(10 downto 0);
    signal lb_wr_c    : unsigned(9 downto 0);
    signal lb_rd_y    : unsigned(10 downto 0);
    signal lb_rd_u    : unsigned(9 downto 0);
    signal lb_rd_v    : unsigned(9 downto 0);
    signal lb_y_out   : std_logic_vector(9 downto 0);
    signal lb_u_out   : std_logic_vector(7 downto 0);
    signal lb_v_out   : std_logic_vector(7 downto 0);
    signal c_in_u     : std_logic_vector(7 downto 0);
    signal c_in_v     : std_logic_vector(7 downto 0);

    -- 64-entry quarter-wave sine LUT, 0..127
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

    -- ========================================================================
    -- Pixel pipeline (depth in comments; pipe(N-1) is the depth-N sync tap)
    -- ========================================================================
    signal c1_x    : unsigned(11 downto 0);
    signal c1_dx   : signed(11 downto 0);
    signal c1_lf   : unsigned(15 downto 0);

    signal c2_base : unsigned(12 downto 0);
    signal c2_cmul : signed(16 downto 0);
    signal c2_dist : unsigned(12 downto 0);
    signal c2_x    : unsigned(11 downto 0);
    signal c2_lf   : unsigned(15 downto 0);

    signal c3_base : unsigned(11 downto 0);
    signal c3_coff : signed(8 downto 0);
    signal c3_vmul : unsigned(16 downto 0);
    signal c3_x    : unsigned(11 downto 0);
    signal c3_lf   : unsigned(15 downto 0);

    signal c4_ya   : unsigned(10 downto 0);
    signal c4_ua   : unsigned(10 downto 0);
    signal c4_va   : unsigned(10 downto 0);
    signal c4_vamt : unsigned(9 downto 0);
    signal c4_x    : unsigned(11 downto 0);
    signal c4_lf   : unsigned(15 downto 0);

    signal c5_vamt : unsigned(9 downto 0);
    signal c5_x    : unsigned(11 downto 0);
    signal c5_lf   : unsigned(15 downto 0);

    signal c6_y    : unsigned(9 downto 0);
    signal c6_u    : unsigned(9 downto 0);
    signal c6_v    : unsigned(9 downto 0);
    signal c6_vamt : unsigned(9 downto 0);
    signal c6_x    : unsigned(11 downto 0);
    signal c6_lf   : unsigned(15 downto 0);

    signal c7_y    : unsigned(9 downto 0);
    signal c7_u    : unsigned(9 downto 0);
    signal c7_v    : unsigned(9 downto 0);
    signal c7_vamt : unsigned(9 downto 0);
    signal c7_x    : unsigned(11 downto 0);

    signal bleed_r : unsigned(9 downto 0) := (others => '0');
    signal c8_y    : unsigned(9 downto 0);
    signal c8_u    : unsigned(9 downto 0);
    signal c8_v    : unsigned(9 downto 0);
    signal c8_vamt : unsigned(9 downto 0);
    signal c8_x    : unsigned(11 downto 0);

    signal c9_gmul : unsigned(15 downto 0);
    signal c9_y    : unsigned(9 downto 0);
    signal c9_u    : unsigned(9 downto 0);
    signal c9_v    : unsigned(9 downto 0);
    signal c9_vamt : unsigned(9 downto 0);
    signal c9_x    : unsigned(11 downto 0);

    signal c10_y   : unsigned(9 downto 0);
    signal c10_u   : unsigned(9 downto 0);
    signal c10_v   : unsigned(9 downto 0);
    signal c10_dim : unsigned(10 downto 0);
    signal c10_x   : unsigned(11 downto 0);

    signal c11_y   : unsigned(9 downto 0);
    signal c11_u   : unsigned(9 downto 0);
    signal c11_v   : unsigned(9 downto 0);

    signal c12_y   : unsigned(9 downto 0);
    signal c12_u   : unsigned(9 downto 0);
    signal c12_v   : unsigned(9 downto 0);

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);
    signal s_io : t_video_stream_yuv444_30b;

    -- ========================================================================
    -- Helpers
    -- ========================================================================
    function sat10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        end if;
        -- v is proven in [0,1023] by the tests above; take the low 10 bits.
        -- resize(signed, 10) would keep the SIGN plus only 9 magnitude bits,
        -- folding every value >= 512 (512 -> 0 = saturated green).
        return resize(unsigned(std_logic_vector(v)), 10);
    end function;

    function quad_sin(ang : unsigned(9 downto 0)) return signed is
        variable v_q   : unsigned(1 downto 0);
        variable v_idx : integer range 0 to 63;
        variable v_mag : unsigned(7 downto 0);
    begin
        v_q   := ang(9 downto 8);
        v_idx := to_integer(ang(7 downto 2));
        if v_q(0) = '0' then
            v_mag := C_SIN_QUAD(v_idx);
        else
            v_mag := C_SIN_QUAD(63 - v_idx);
        end if;
        if v_q(1) = '0' then
            return signed(resize(v_mag, 9));
        else
            return -signed(resize(v_mag, 9));
        end if;
    end function;

begin

    -- ========================================================================
    -- Line buffers: full-rate 10-bit Y, half-rate 8-bit U and V.
    -- ========================================================================
    c_in_u <= data_in.u(9 downto 2);
    c_in_v <= data_in.v(9 downto 2);

    lb_y_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(clk => clk, i_ab => lb_ab, i_wr_addr => lb_wr_y,
                 i_rd_addr => lb_rd_y, i_data => data_in.y, o_data => lb_y_out);

    lb_u_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 8, G_DEPTH => 10)
        port map(clk => clk, i_ab => lb_ab, i_wr_addr => lb_wr_c,
                 i_rd_addr => lb_rd_u, i_data => c_in_u, o_data => lb_u_out);

    lb_v_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 8, G_DEPTH => 10)
        port map(clk => clk, i_ab => lb_ab, i_wr_addr => lb_wr_c,
                 i_rd_addr => lb_rd_v, i_data => c_in_v, o_data => lb_v_out);

    lb_wr_y <= px(10 downto 0);
    lb_wr_c <= px(10 downto 1);
    lb_rd_y <= c4_ya;
    lb_rd_u <= c4_ua(10 downto 1);
    lb_rd_v <= c4_va(10 downto 1);

    -- ========================================================================
    -- Timing, measurement, per-frame and per-line state
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_b1n    : signed(15 downto 0);
        variable v_b2n    : signed(15 downto 0);
        variable v_jraw   : signed(11 downto 0);
        variable v_bp     : signed(13 downto 0);
        variable v_hlf    : signed(13 downto 0);
        variable v_marg   : signed(13 downto 0);
        variable v_span   : signed(13 downto 0);
        variable v_jog    : signed(11 downto 0);
        variable v_eff    : signed(13 downto 0);
        variable v_ang    : unsigned(9 downto 0);
        variable v_tap    : std_logic;
    begin
        if rising_edge(clk) then
            vs_r <= data_in.vsync_n;
            hs_r <= data_in.hsync_n;
            av_r <= data_in.avid;
            fl_r <= data_in.field_n;
            prev_hsync_n <= hs_r;
            prev_vsync_n <= vs_r;
            prev_avid    <= av_r;

            v_h_edge := '0';
            if hs_r = '0' and prev_hsync_n = '1' then
                v_h_edge := '1';
            end if;

            -- Free-running noise
            v_tap  := lfsr_r(15) xor lfsr_r(13) xor lfsr_r(12) xor lfsr_r(10);
            lfsr_r <= lfsr_r(14 downto 0) & v_tap;

            -- Free-running chroma-drift sine and its depth multiply.  Both are
            -- registered every cycle and only SAMPLED at line start, so the
            -- multiply never sits in the per-line decision cone.
            v_ang     := unsigned(std_logic_vector(aline(9 downto 0))) +
                         drift_ph(13 downto 4);
            sin_r     <= quad_sin(v_ang);
            drift_mul <= resize(sin_r * signed(resize(drift_s, 8)), 16);

            -- ===============================================================
            -- PARAMETER DECODE - free-running, every clock.
            --
            -- These were latched once per frame at the video anchor.  On
            -- hardware that anchor stopped re-firing and every knob and switch
            -- went dead while the picture still rendered, so nothing here is
            -- allowed to depend on a frame event any more: the registers are
            -- read continuously and pipelined one op per stage.  It is also
            -- what the house rule asks for - decode SPI registers into
            -- free-running values, never inside the vsync decision cone.
            -- ===============================================================
            hiss_r    <= unsigned(registers_in(1));
            grille_r  <= registers_in(6)(0);
            cast_r    <= registers_in(6)(1);
            faded_r   <= registers_in(6)(2);
            bmode_r   <= registers_in(6)(3);
            inv_r     <= registers_in(6)(4);
            drop_r    <= unsigned(registers_in(0));
            drift_s   <= unsigned(registers_in(2)(9 downto 3));
            glow6     <= unsigned(registers_in(4)(9 downto 4));
            tube6     <= resize(unsigned(registers_in(5)(9 downto 4)), 7) +
                         to_unsigned(4, 7);
            scan_amt  <= resize(unsigned(registers_in(5)(9 downto 2)), 10) +
                         to_unsigned(24, 10);
            triad_amt <= resize(unsigned(registers_in(5)(9 downto 3)), 8) +
                         to_unsigned(12, 8);

            -- Misconvergence: bipolar with a dead band, split over two stages
            conv_d_r <= signed(resize(unsigned(registers_in(3)), 12)) -
                        to_signed(512, 12);
            if conv_d_r < 40 and conv_d_r > -40 then
                conv_s <= (others => '0');
            else
                conv_s <= resize(shift_right(conv_d_r, 3), 7);
            end if;

            -- Dropout: half-height of the head-switch band as a fraction of
            -- the raster.  Zero means NO band - the bar has to be defeatable.
            band_mul <= resize(act_h(11 downto 7) * drop_r(9 downto 6), 10);
            band_hlf <= resize(band_mul, 12);
            roll_step <= signed(resize(drop_r(9 downto 7), 8)) + to_signed(1, 8);

            -- Free-running skew derivation (one op per stage, never in a
            -- decision cone).  Settles in five clocks - instant next to a frame.
            skew_d_r <= signed(resize(unsigned(registers_in(7)), 12)) -
                        to_signed(512, 12);
            if skew_d_r < 40 and skew_d_r > -40 then
                skew_z_r <= (others => '0');
            else
                skew_z_r <= skew_d_r;
            end if;
            case to_integer(wcls) is
                when 0      => top_amp <= skew_z_r;
                when 1      => top_amp <= resize(shift_right(skew_z_r, 1), 12);
                when others => top_amp <= resize(shift_right(skew_z_r, 2), 12);
            end case;
            bend_seed <= shift_left(resize(top_amp, 16), 4);
            bend2_sd  <= shift_left(resize(top_amp, 16), 3);
            if wcls = "10" then
                if ilace = '1' then
                    dshift <= "10";
                else
                    dshift <= "01";
                end if;
            elsif ilace = '1' then
                dshift <= "01";
            else
                dshift <= "00";
            end if;

            -- Free-running band edges and line geometry.  Everything here is
            -- register-in / register-out and settles long before active video.
            aline_s <= signed(resize(aline, 14));
            v_hlf   := signed(resize(band_hlf, 14));
            bt1 <= band_pos - v_hlf;
            bb1 <= band_pos + v_hlf;
            bt2 <= band_pos - shift_left(v_hlf, 1);
            bb2 <= band_pos + shift_left(v_hlf, 1);
            if aline_s >= bt1 and aline_s < bb1 then
                in_band <= '1';
            else
                in_band <= '0';
            end if;
            if aline_s >= bt2 and aline_s < bb2 then
                near_bnd <= '1';
            else
                near_bnd <= '0';
            end if;
            scan_par <= aline(0);

            -- Vertical distance from centre for the vignette
            dyd_r <= signed(resize(aline, 13)) - signed(resize(cy_r, 13));
            if dyd_r < 0 then
                dy_r <= unsigned(std_logic_vector(resize(-dyd_r, 12)));
            else
                dy_r <= unsigned(std_logic_vector(resize(dyd_r, 12)));
            end if;

            -- ---------------------------------------------------------------
            -- Horizontal counter + measured active width
            -- ---------------------------------------------------------------
            if v_h_edge = '1' then
                px    <= (others => '0');
                lb_ab <= not lb_ab;
            elsif av_r = '1' then
                px <= px + 1;
            end if;

            -- ---------------------------------------------------------------
            -- End of an active line: advance every per-line accumulator here
            -- (active lines only, so both interlaced fields stay in step).
            -- ---------------------------------------------------------------
            if prev_avid = '1' and av_r = '0' then
                if px > 100 then
                    act_w <= px;
                end if;
                aline   <= aline + 1;

                -- Both pulls decay once per ACTIVE line, so the two
                -- interlaced fields stay in step.
                -- Time constants: the frame flag spans roughly the top
                -- quarter of the picture and the head-switch tear about a
                -- sixteenth of it, in every raster.
                case to_integer(dshift) is
                    when 0 =>       -- ~1080 line frame
                        v_b1n := bend1 - shift_right(bend1, 7);
                        v_b2n := bend2 - shift_right(bend2, 5);
                    when 1 =>       -- ~540 line field / ~480 line frame
                        v_b1n := bend1 - shift_right(bend1, 6);
                        v_b2n := bend2 - shift_right(bend2, 4);
                    when others =>  -- ~240 line field
                        v_b1n := bend1 - shift_right(bend1, 5);
                        v_b2n := bend2 - shift_right(bend2, 3);
                end case;
                bend1 <= v_b1n;

                -- Raw tear jog for the next line, from the registered flags
                if in_band = '1' then
                    v_jraw := signed(resize(lfsr_r(7 downto 0), 12)) -
                              to_signed(128, 12);
                elsif near_bnd = '1' then
                    v_jraw := signed(resize(lfsr_r(11 downto 7), 12)) -
                              to_signed(16, 12);
                else
                    v_jraw := (others => '0');
                end if;

                -- The head-switch pull re-arms every time the band's top edge
                -- is crossed, then decays over ~64 lines below it.
                in_band_p <= in_band;
                if in_band = '1' and in_band_p = '0' then
                    v_b2n := bend2_sd;
                end if;
                bend2 <= v_b2n;

                -- Row offset for the next line, formed here (a whole line of
                -- slack) so hsync only has to wrap it.  The tear jog keys off
                -- the REGISTERED band flags - one line of lag, invisible.
                case to_integer(wcls) is
                    when 0      => v_jog := v_jraw;
                    when 1      => v_jog := resize(shift_right(v_jraw, 1), 12);
                    when others => v_jog := resize(shift_right(v_jraw, 2), 12);
                end case;
                row_pre <= resize(shift_right(v_b1n + v_b2n, 4), 14) +
                           resize(v_jog, 14);

                drift_off <= resize(shift_right(drift_mul, 9), 8);
            end if;

            -- ---------------------------------------------------------------
            -- Tear jog folded into the row pointer, one line ahead of use.
            -- ---------------------------------------------------------------
            if v_h_edge = '1' then
                v_eff := row_pre;
                if v_eff >= signed(resize(act_w, 14)) then
                    v_eff := v_eff - signed(resize(act_w, 14));
                elsif v_eff < 0 then
                    v_eff := v_eff + signed(resize(act_w, 14));
                end if;
                if v_eff < 0 or v_eff >= signed(resize(act_w, 14)) then
                    row_eff <= (others => '0');
                else
                    row_eff <= unsigned(std_logic_vector(v_eff(11 downto 0)));
                end if;
            end if;

            -- ---------------------------------------------------------------
            -- Frame anchor: first active pixel after vsync.  Serration-safe -
            -- frame_act is armed on the vsync saw and fires exactly once.
            -- ---------------------------------------------------------------
            -- The vsync arm is backed up by a line-count ceiling: no supported
            -- format has more than 1080 active lines, so if the anchor has not
            -- fired by line 1200 the frame housekeeping runs anyway.  Without
            -- this a stuck anchor leaves aline free-running and the tear band
            -- wanders and flashes at random.
            if vs_r = '0' then
                frame_act <= '0';
            elsif av_r = '1' and (frame_act = '0' or aline >= 1200) then
                frame_act <= '1';

                if aline > 50 then
                    act_h <= aline;
                    cy_r  <= shift_right(aline, 1);
                end if;
                aline <= (others => '0');
                cx_r  <= shift_right(act_w, 1);

                if fl_r /= anc_fld then
                    ilace <= '1';
                else
                    ilace <= '0';
                end if;
                anc_fld <= fl_r;

                -- Head-switch band drift: travels two band-heights past each
                -- end of the raster so it rolls off one edge and back on.
                v_bp  := band_pos + resize(roll_step, 14);
                v_marg := signed(resize(band_hlf, 14));
                v_span := signed(resize(act_h, 14)) + shift_left(v_marg, 2);
                if v_bp >= signed(resize(act_h, 14)) + shift_left(v_marg, 1) then
                    v_bp := v_bp - v_span;
                elsif v_bp < -shift_left(v_marg, 1) then
                    v_bp := v_bp + v_span;
                end if;
                band_pos <= v_bp;

                drift_ph <= drift_ph + resize(drift_s, 14) + to_unsigned(24, 14);

                -- The frame flag re-arms at the top of every frame; the
                -- amplitude and raster class are already resolved.
                bend1 <= bend_seed;
                if act_w > 1400 then
                    wcls <= "00";
                elsif act_w > 900 then
                    wcls <= "01";
                else
                    wcls <= "10";
                end if;

            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_dx    : signed(11 downto 0);
        variable v_sum   : unsigned(12 downto 0);
        variable v_ca    : signed(13 downto 0);
        variable v_w13   : signed(13 downto 0);
        variable v_noise : signed(11 downto 0);
        variable v_yn    : signed(12 downto 0);
        variable v_bl    : unsigned(12 downto 0);
        variable v_g     : signed(12 downto 0);
        variable v_dim   : unsigned(10 downto 0);
        variable v_cu    : signed(11 downto 0);
        variable v_cv    : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- ---- depth 1: sample position, distance from centre, noise -----
            c1_x  <= px;
            c1_lf <= lfsr_r;
            v_dx  := signed(resize(px, 12)) - signed(resize(cx_r, 12));
            c1_dx <= v_dx;

            -- ---- depth 2: skew sum, misconvergence multiply, vignette dist --
            c2_base <= resize(row_eff, 13) + resize(c1_x, 13);
            c2_cmul <= resize(c1_dx(11 downto 2) * conv_s, 17);
            if c1_dx < 0 then
                c2_dist <= resize(unsigned(std_logic_vector(-c1_dx)), 13) +
                           resize(dy_r, 13);
            else
                c2_dist <= resize(unsigned(std_logic_vector(c1_dx)), 13) +
                           resize(dy_r, 13);
            end if;
            c2_x  <= c1_x;
            c2_lf <= c1_lf;

            -- ---- depth 3: wrap the skewed X, form the chroma offset --------
            if c2_base >= resize(act_w, 13) then
                c3_base <= resize(c2_base - resize(act_w, 13), 12);
            else
                c3_base <= resize(c2_base, 12);
            end if;
            c3_coff <= resize(shift_right(c2_cmul, 8), 9) + resize(drift_off, 9);
            c3_vmul <= resize(c2_dist(12 downto 3) * tube6, 17);
            c3_x    <= c2_x;
            c3_lf   <= c2_lf;

            -- ---- depth 4: three read addresses ----------------------------
            c4_ya <= c3_base(10 downto 0);
            v_w13 := signed(resize(act_w, 14));
            v_ca  := signed(resize(c3_base, 14)) + resize(c3_coff, 14);
            if v_ca >= v_w13 then
                v_ca := v_ca - v_w13;
            elsif v_ca < 0 then
                v_ca := v_ca + v_w13;
            end if;
            c4_ua <= unsigned(std_logic_vector(v_ca(10 downto 0)));
            v_ca  := signed(resize(c3_base, 14)) - resize(c3_coff, 14);
            if v_ca >= v_w13 then
                v_ca := v_ca - v_w13;
            elsif v_ca < 0 then
                v_ca := v_ca + v_w13;
            end if;
            c4_va <= unsigned(std_logic_vector(v_ca(10 downto 0)));
            if c3_vmul(16 downto 4) > to_unsigned(700, 13) then
                c4_vamt <= to_unsigned(700, 10);
            else
                c4_vamt <= c3_vmul(13 downto 4);
            end if;
            c4_x  <= c3_x;
            c4_lf <= c3_lf;

            -- ---- depth 5: line buffer read in flight ----------------------
            c5_vamt <= c4_vamt;
            c5_x    <= c4_x;
            c5_lf   <= c4_lf;

            -- ---- depth 6: capture the line buffer outputs ------------------
            c6_y    <= unsigned(lb_y_out);
            c6_u    <= shift_left(resize(unsigned(lb_u_out), 10), 2);
            c6_v    <= shift_left(resize(unsigned(lb_v_out), 10), 2);
            c6_vamt <= c5_vamt;
            c6_x    <= c5_x;
            c6_lf   <= c5_lf;

            -- ---- depth 7: tape hiss + head-switch band --------------------
            v_noise := signed(resize(c6_lf(8 downto 0), 12)) - to_signed(256, 12);
            case to_integer(hiss_r(9 downto 7)) is
                when 0      => v_noise := to_signed(0, 12);
                when 1      => v_noise := resize(shift_right(v_noise, 4), 12);
                when 2      => v_noise := resize(shift_right(v_noise, 3), 12);
                when 3      => v_noise := resize(shift_right(v_noise, 2), 12);
                when 4 | 5  => v_noise := resize(shift_right(v_noise, 1), 12);
                when others => null;
            end case;
            v_yn := signed(resize(c6_y, 13)) + resize(v_noise, 13);

            if in_band = '1' and bmode_r = '0' then
                -- Tear: the band is torn-off tape, luma is pure static
                c7_y <= resize(c6_lf(10 downto 2), 10) + to_unsigned(128, 10);
                c7_u <= C_MID;
                c7_v <= C_MID;
            elsif in_band = '1' then
                -- Flash: head-switch light bar, washed out and bleached
                c7_y <= sat10(signed(resize(c6_y, 13)) + to_signed(360, 13) +
                              resize(shift_right(v_noise, 1), 13));
                c7_u <= C_MID;
                c7_v <= C_MID;
            else
                c7_y <= sat10(v_yn);
                c7_u <= c6_u;
                c7_v <= c6_v;
            end if;
            c7_vamt <= c6_vamt;
            c7_x    <= c6_x;

            -- ---- depth 8: phosphor bleed (one-pole IIR along the line) -----
            v_bl := resize(bleed_r, 13) - resize(shift_right(bleed_r, 2), 13) +
                    resize(shift_right(c7_y, 2), 13);
            if c7_x = 0 then
                bleed_r <= c7_y;
            else
                bleed_r <= v_bl(9 downto 0);
            end if;
            c8_y    <= c7_y;
            c8_u    <= c7_u;
            c8_v    <= c7_v;
            c8_vamt <= c7_vamt;
            c8_x    <= c7_x;

            -- ---- depth 9: glow multiply -----------------------------------
            c9_gmul <= resize(bleed_r * glow6, 16);
            c9_y    <= c8_y;
            c9_u    <= c8_u;
            c9_v    <= c8_v;
            c9_vamt <= c8_vamt;
            c9_x    <= c8_x;

            -- ---- depth 10: add the glow, gather the tube darkening --------
            v_g := signed(resize(c9_y, 13)) +
                   signed(resize(c9_gmul(15 downto 8), 13));
            c10_y <= sat10(v_g);
            c10_u <= c9_u;
            c10_v <= c9_v;
            if grille_r = '0' and scan_par = '1' then
                v_dim := resize(c9_vamt, 11) + resize(scan_amt, 11);
            else
                v_dim := resize(c9_vamt, 11);
            end if;
            c10_dim <= v_dim;
            c10_x   <= c9_x;

            -- ---- depth 11: apply the darkening, RGB triad tint ------------
            c11_y <= sat10(signed(resize(c10_y, 13)) -
                           signed(resize(c10_dim, 13)));
            if grille_r = '1' then
                -- 4-column phosphor stripe: red, green, blue, gap.  Hardware
                -- sense: u carries Cr (red), v carries Cb (blue).
                case to_integer(c10_x(1 downto 0)) is
                    when 0 =>
                        c11_u <= sat10(signed(resize(c10_u, 12)) +
                                       signed(resize(triad_amt, 12)));
                        c11_v <= sat10(signed(resize(c10_v, 12)) -
                                       signed(resize(triad_amt, 12)));
                    when 1 =>
                        c11_u <= sat10(signed(resize(c10_u, 12)) -
                                       signed(resize(triad_amt, 12)));
                        c11_v <= sat10(signed(resize(c10_v, 12)) -
                                       signed(resize(triad_amt, 12)));
                    when 2 =>
                        c11_u <= sat10(signed(resize(c10_u, 12)) -
                                       signed(resize(triad_amt, 12)));
                        c11_v <= sat10(signed(resize(c10_v, 12)) +
                                       signed(resize(triad_amt, 12)));
                    when others =>
                        c11_u <= c10_u;
                        c11_v <= c10_v;
                end case;
            else
                c11_u <= c10_u;
                c11_v <= c10_v;
            end if;

            -- ---- depth 12: chroma cast, fade, luma invert -----------------
            v_cu := signed(resize(c11_u, 12)) - to_signed(512, 12);
            v_cv := signed(resize(c11_v, 12)) - to_signed(512, 12);
            if faded_r = '1' then
                v_cu := resize(shift_right(v_cu, 1), 12);
                v_cv := resize(shift_right(v_cv, 1), 12);
            end if;
            if cast_r = '0' then
                -- Cool: toward the blue axis (Cb up, Cr down)
                v_cu := v_cu - to_signed(40, 12);
                v_cv := v_cv + to_signed(72, 12);
            else
                -- Sick: toward sallow green (both below neutral)
                v_cu := v_cu - to_signed(56, 12);
                v_cv := v_cv - to_signed(64, 12);
            end if;
            c12_u <= sat10(v_cu + to_signed(512, 12));
            c12_v <= sat10(v_cv + to_signed(512, 12));
            if inv_r = '1' then
                c12_y <= to_unsigned(1023, 10) - c11_y;
            else
                c12_y <= c11_y;
            end if;

            -- ---- depth 13: blanking gate + sync tap (output register) -----
            if pipe(LATENCY - 1).avid = '0' then
                s_io.y <= std_logic_vector(C_BLKY);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                s_io.y <= std_logic_vector(c12_y);
                s_io.u <= std_logic_vector(c12_u);
                s_io.v <= std_logic_vector(c12_v);
            end if;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.avid    <= s_io.avid;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.field_n <= s_io.field_n;

end architecture deadchannel;
