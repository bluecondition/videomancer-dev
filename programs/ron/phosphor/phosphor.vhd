-- phosphor.vhd
--
-- Edge detection with rainbow band-trail to the right of each edge.
--
-- At each detected edge the output resets to RED; subsequent pixels on the
-- same scanline step through ORANGE, YELLOW, GREEN, BLUE, INDIGO, VIOLET,
-- then black, each band occupying `band_width` pixels (knob 1). Pipeline
-- resets at hsync so trails never cross lines.
--
-- Edge detect : 3x3 Sobel on preprocessed luma (the Edge Lab detector).
--               Four chained line buffers give a 5-row window; the Sobel
--               rows are raw, 3x3 Gaussian-blurred (switch 9, default on,
--               per Edge Lab's Smooth) or separable-3x3-median denoised
--               (switch 10, wins over smooth) -- the median kills speckle
--               WITHOUT softening the edge, so magnitudes stay strong.
--               Magnitude |gx| + |gy| against the knob 2 threshold.
-- Solidify    : the thresholded edge bit is OR'd with the same column of
--               the previous two lines (two 1-bit line buffers), so a line
--               whose magnitude dips just under threshold still fires where
--               its neighbours detected the contour -- no gaps in the sheet.
--               The trigger is then HELD for the whole solid run, pinning
--               the band machine at red: the outline renders as a solid red
--               region as thick as the detected edge, rainbow trailing off
--               its right side.
-- Preprocess  : contrast around mid-grey (K5), then posterize (K4,
--               128..2 luma levels, Edge Lab mapping), then 8-bit reduce.
-- Color path  : 3-bit band index → 8-entry YUV LUT (7 rainbow + black).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture phosphor of program_top is

    constant C_SYNC_DELAY : integer := 13;

    -- 8-entry rainbow palette in 10-bit YUV, BT.601 limited range, with U/V
    -- swapped for Videomancer hardware (SDK convention). The C_PAL_U constant
    -- holds what BT.601 calls Cr, and C_PAL_V holds Cb.
    -- Index 7 is black (past violet).
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(328, 10),  -- Red    (255,   0,   0)
        to_unsigned(584, 10),  -- Orange (255, 128,   0)
        to_unsigned(840, 10),  -- Yellow (255, 255,   0)
        to_unsigned(580, 10),  -- Green  (  0, 255,   0)
        to_unsigned(164, 10),  -- Blue   (  0,   0, 255)
        to_unsigned(192, 10),  -- Indigo ( 75,   0, 130)
        to_unsigned(349, 10),  -- Violet (180,   0, 255)
        to_unsigned(  0, 10)); -- Black
    constant C_PAL_U : t_pal := (
        to_unsigned(960, 10),  -- Red
        to_unsigned(772, 10),  -- Orange
        to_unsigned(584, 10),  -- Yellow
        to_unsigned(136, 10),  -- Green
        to_unsigned(440, 10),  -- Blue
        to_unsigned(608, 10),  -- Indigo
        to_unsigned(756, 10),  -- Violet
        to_unsigned(512, 10)); -- Black
    constant C_PAL_V : t_pal := (
        to_unsigned(360, 10),  -- Red
        to_unsigned(212, 10),  -- Orange
        to_unsigned( 64, 10),  -- Yellow
        to_unsigned(216, 10),  -- Green
        to_unsigned(960, 10),  -- Blue
        to_unsigned(696, 10),  -- Indigo
        to_unsigned(852, 10),  -- Violet
        to_unsigned(512, 10)); -- Black

    function f_c255(v : unsigned) return unsigned is
    begin
        if v > 255 then
            return to_unsigned(255, 8);
        else
            return resize(v, 8);
        end if;
    end function;

    function f_absu(v : signed) return unsigned is
    begin
        if v < 0 then
            return unsigned(resize(-v, v'length));
        else
            return unsigned(v);
        end if;
    end function;

    -- Median of 3: two comparator levels (a/b sort, then c against both)
    function f_med3(a, b, c : unsigned(7 downto 0)) return unsigned is
        variable v_mn, v_mx : unsigned(7 downto 0);
    begin
        if a < b then
            v_mn := a;  v_mx := b;
        else
            v_mn := b;  v_mx := a;
        end if;
        if c < v_mn then
            return v_mn;
        elsif c > v_mx then
            return v_mx;
        else
            return c;
        end if;
    end function;

    -- Parameters
    signal s_knob_width      : unsigned(9 downto 0);
    signal s_knob_noise      : unsigned(9 downto 0);
    signal s_knob_contrast   : unsigned(9 downto 0);
    signal s_knob_curve      : unsigned(9 downto 0);
    signal s_poster_mask     : std_logic_vector(9 downto 0);
    signal s_curve_steep     : unsigned(1 downto 0);  -- curve steepness 0..3
    signal s_sw_video_key    : std_logic;
    signal s_sw_curve_en     : std_logic;
    signal s_sw_smooth       : std_logic;
    signal s_sw_median       : std_logic;
    signal s_knob_color      : unsigned(9 downto 0);  -- K1: rainbow phase, animatable
    signal s_sw_reverse      : std_logic;             -- S11: violet-first band order
    signal s_band_width      : unsigned(7 downto 0);  -- 1..128 (8 bits: 128 must not wrap)
    signal s_min_run_m1      : unsigned(4 downto 0);  -- min run length - 1 (0..15)
    signal s_contrast_gain   : unsigned(8 downto 0);  -- 128..255 (1.0x..~2.0x in 7-bit fixed)
    signal s_thr             : unsigned(7 downto 0);  -- Sobel magnitude threshold

    -- Valid chain: av(i) = data_in.avid delayed i clocks
    signal av : std_logic_vector(1 to 12) := (others => '0');

    -- S1-S2 contrast: Y_out = 512 + (Y_in - 512) * gain / 128
    signal a1_prod : signed(19 downto 0) := (others => '0');
    signal a2_y    : unsigned(9 downto 0) := (others => '0');

    -- S3 posterize + reduce to 8 bit
    signal a3_y8 : unsigned(7 downto 0) := (others => '0');

    -- Luma line buffers (BRAM): 4 chained lines for the 5-row window that
    -- feeds the 3x3 Gaussian blur (and the raw Sobel rows when smooth is
    -- off). Canonical 1W1R single-process BRAMs; read addr leads write by
    -- one column.
    type t_lb8 is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_lb1 is array (0 to 2047) of std_logic_vector(0 downto 0);
    signal lb1, lb2, lb3, lb4 : t_lb8 := (others => (others => '0'));
    signal q1, q2, q3, q4     : std_logic_vector(7 downto 0) := (others => '0');
    signal lrx, lwx           : unsigned(10 downto 0) := (others => '0');

    -- S4 registered copies of the BRAM read data. The line-buffer Tco plus
    -- routing was the head of the critical path (14.2 ns through the median
    -- comparators); landing the reads in plain FFs first decouples it.
    signal qr1, qr2, qr3, qr4 : unsigned(7 downto 0) := (others => '0');
    signal y8d                : unsigned(7 downto 0) := (others => '0');

    -- S5 vertical [1 2 1] sums for the 3 blur rows (rows n-1, n-2, n-3)
    -- plus 2-column histories for the horizontal blur pass, and 2-column
    -- delayed raw taps so raw mode aligns with the blur center column.
    signal ga, ga1, ga2 : unsigned(9 downto 0) := (others => '0');  -- bottom (n-1)
    signal gb, gb1, gb2 : unsigned(9 downto 0) := (others => '0');  -- mid    (n-2)
    signal gc, gc1, gc2 : unsigned(9 downto 0) := (others => '0');  -- top    (n-3)
    signal q1d, q2d, q3d : unsigned(7 downto 0) := (others => '0');
    signal q1e, q2e, q3e : unsigned(7 downto 0) := (others => '0');

    -- S4 separable-median first pass: vertical median-of-3 per row + 2-col
    -- histories for the horizontal pass (vm0 = top row n-3 ... vm2 = n-1)
    signal vm0, vm0d, vm0e : unsigned(7 downto 0) := (others => '0');
    signal vm1, vm1d, vm1e : unsigned(7 downto 0) := (others => '0');
    signal vm2, vm2d, vm2e : unsigned(7 downto 0) := (others => '0');

    -- S5 effective Sobel rows (blurred or raw; e0 = top, e2 = bottom)
    signal e0, e1, e2 : unsigned(7 downto 0) := (others => '0');

    -- S6 Sobel column sums + histories
    signal ss, ss1, ss2 : unsigned(9 downto 0) := (others => '0');  -- e0+2e1+e2
    signal dy, dy1, dy2 : signed(8 downto 0) := (others => '0');    -- e0-e2

    -- S7-S9 gradients, magnitude, normalized+guarded magnitude
    signal s_gx, s_gy : signed(11 downto 0) := (others => '0');
    signal s_mag      : unsigned(11 downto 0) := (others => '0');
    signal s_mag8     : unsigned(7 downto 0) := (others => '0');

    -- S10 thresholded edge bit + vertical-solidify bit line buffers
    signal s_eb       : std_logic := '0';
    signal elb1, elb2 : t_lb1 := (others => (others => '0'));
    signal eq1, eq2   : std_logic_vector(0 downto 0) := (others => '0');
    signal ebr, ebw   : unsigned(10 downto 0) := (others => '0');

    -- Blanking guards: left columns (shift/history chains crossing hblank)
    -- and field warm-up lines (window straddling vblank)
    signal cbx     : unsigned(10 downto 0) := (others => '0');
    signal s_aline : unsigned(9 downto 0) := (others => '0');
    signal s_seen  : std_logic := '0';
    signal s_prev_hsync_n_p : std_logic := '1';

    -- S11 solidify + min-run
    signal s_edge_bit  : std_logic := '0';
    signal s_run_count : unsigned(4 downto 0) := (others => '0');

    -- Rainbow band state
    signal s_prev_hsync_n : std_logic := '1';
    signal s_band_pixel : unsigned(6 downto 0) := (others => '0');  -- 0..band_width-1
    signal s_band_idx   : unsigned(2 downto 0) := "111";            -- 7 = black at line start
    -- Colour remap table: band_idx -> palette index, with the K1 cycle
    -- phase and S11 reverse folded in. Registered off the pixel path so
    -- the output mux stays shallow; index 7 always maps to 7 (black/key).
    type t_cmap is array (0 to 7) of unsigned(2 downto 0);
    signal s_cmap      : t_cmap := ("000", "001", "010", "011", "100", "101", "110", "111");
    signal s_disp_idx  : unsigned(2 downto 0);                      -- palette index after remap
    signal s_key_video : std_logic;                                 -- pass source video this pixel

    -- Per-band effective widths MINUS ONE (curve-adjusted terminal counts),
    -- indexed directly by s_band_idx so the value is never stale when the
    -- band advances and the compare needs no subtract.
    type t_sizes is array (0 to 7) of unsigned(7 downto 0);
    signal s_sizes_m1 : t_sizes := (others => to_unsigned(127, 8));

    -- Delayed incoming Y/U/V for video key-through (needs full SYNC_DELAY alignment)
    type t_yuv_delay is array (0 to C_SYNC_DELAY - 1) of std_logic_vector(9 downto 0);
    signal s_y_in_sr : t_yuv_delay := (others => (others => '0'));
    signal s_u_in_sr : t_yuv_delay := (others => (others => '0'));
    signal s_v_in_sr : t_yuv_delay := (others => (others => '0'));

    -- Sync shift register
    signal s_avid_sr    : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');
    signal s_field_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');

begin

    -- Band W lives on the P12 slider (registers_in(7)). Rotary 1 is the
    -- rainbow colour-cycle phase: turning it rotates the colours through
    -- the bands, animating the trail.
    s_knob_width    <= unsigned(registers_in(7));
    s_knob_color    <= unsigned(registers_in(0));
    s_knob_noise    <= unsigned(registers_in(2));
    s_knob_contrast <= unsigned(registers_in(4));
    s_knob_curve    <= unsigned(registers_in(5));
    s_sw_video_key  <= registers_in(6)(0);
    s_sw_curve_en   <= registers_in(6)(1);
    -- Toggle switches 7..11 arrive as bits 0..4 of registers_in(6)
    s_sw_smooth     <= registers_in(6)(2);
    s_sw_median     <= registers_in(6)(3);
    s_sw_reverse    <= registers_in(6)(4);

    -- Band width: knob top 7 bits + 1 gives 1..128 pixels. Must be 8 bits:
    -- in 7 bits the max setting wraps 127+1 -> 0 and the band machine sticks
    -- on red forever.
    s_band_width <= resize(unsigned(s_knob_width(9 downto 3)), 8) + to_unsigned(1, 8);
    -- Min run length: knob top 4 bits + 1 gives 1..16 pixels; stored minus 1
    -- so the run-count compare needs no subtract
    s_min_run_m1 <= resize(unsigned(s_knob_noise(9 downto 6)), 5);
    -- Sobel threshold: knob top 8 bits cover the full 0..255 magnitude range
    s_thr <= unsigned(registers_in(1)(9 downto 2));
    -- Contrast gain: 7-bit fixed-point (128 = 1.0x). 128 + knob(9:3) = 128..255.
    s_contrast_gain <= resize(unsigned(s_knob_contrast(9 downto 3)), 9) + to_unsigned(128, 9);
    -- Curve steepness: knob top 2 bits = 0..3 shift-per-band for the
    -- geometric band-width curve (0 = even, 1 = doubling, 3 = steepest).
    s_curve_steep <= s_knob_curve(9 downto 8);
    -- Posterize (Edge Lab mapping): knob top 3 bits → off, then 128..2 levels
    with to_integer(unsigned(registers_in(3)(9 downto 7))) select
        s_poster_mask <= "1111111111" when 0,
                         "1111111000" when 1,
                         "1111110000" when 2,
                         "1111100000" when 3,
                         "1111000000" when 4,
                         "1110000000" when 5,
                         "1100000000" when 6,
                         "1000000000" when others;

    ----------------------------------------------------------------------------
    -- Edge pipeline (all luma), Edge Lab Sobel with optional Gaussian smooth
    -- and a vertical edge-bit solidify:
    --   S1  contrast stage A: (Y - 512) * gain
    --   S2  contrast stage B: shift back, recenter, clamp
    --   S3  posterize, reduce to 8 bit; present line-buffer read address
    --   S4  register BRAM reads into FFs; line-buffer rotate writes
    --   S5  vertical [1 2 1] sums + vertical medians for rows n-1/n-2/n-3
    --       + column histories; delayed raw taps
    --   S6  horizontal [1 2 1], /16 -> blurred rows; horizontal median;
    --       source mux -> e-rows
    --   S7  Sobel column sums ss / dy + two-column histories
    --   S8  gx = ss - ss2 (horizontal), gy = dy + 2*dy1 + dy2 (vertical)
    --   S9  magnitude |gx| + |gy|
    --   S10 normalize to 0..255, blank warm-up lines / left columns
    --   S11 threshold -> edge bit; present bit-buffer read address
    --   S12 solidify (OR with previous 2 lines) + min-run → trigger
    ----------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_diff  : signed(10 downto 0);
        variable v_sum   : signed(13 downto 0);
        variable v_p10   : unsigned(9 downto 0);
        variable v_m8    : unsigned(7 downto 0);
        variable v_solid : std_logic;
    begin
        if rising_edge(clk) then
            s_prev_hsync_n_p <= data_in.hsync_n;

            av(1) <= data_in.avid;
            for i in 2 to 12 loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            -- S1: difference from mid-grey, multiply by contrast gain
            if data_in.avid = '1' then
                v_diff := signed(resize(unsigned(data_in.y), 11)) - to_signed(512, 11);
                a1_prod <= v_diff * signed(std_logic_vector(s_contrast_gain));
            end if;

            -- S2: shift back, recenter, clamp
            if av(1) = '1' then
                v_sum := resize(shift_right(a1_prod, 7), 14) + to_signed(512, 14);
                if v_sum < 0 then
                    a2_y <= (others => '0');
                elsif v_sum > 1023 then
                    a2_y <= (others => '1');
                else
                    a2_y <= unsigned(std_logic_vector(v_sum(9 downto 0)));
                end if;
            end if;

            -- S3: posterize, reduce to 8 bit; luma reads presented this cycle
            if av(2) = '1' then
                v_p10 := a2_y and unsigned(s_poster_mask);
                a3_y8 <= v_p10(9 downto 2);
                lrx   <= lrx + 1;
            end if;

            -- S4: land the BRAM reads in plain FFs (window rows: current =
            -- a3_y8 -> y8d, q1 = 1 line up ... q4 = 4 up); buffer rotate
            -- writes happen in the BRAM processes
            if av(3) = '1' then
                qr1 <= unsigned(q1);
                qr2 <= unsigned(q2);
                qr3 <= unsigned(q3);
                qr4 <= unsigned(q4);
                y8d <= a3_y8;
                lwx <= lwx + 1;
            end if;

            -- S5: vertical [1 2 1] sums and vertical medians for the 3 blur
            -- rows, two-column histories for the horizontal passes, delayed
            -- raw taps
            if av(4) = '1' then
                ga <= resize(y8d, 10)
                      + shift_left(resize(qr1, 10), 1)
                      + resize(qr2, 10);
                gb <= resize(qr1, 10)
                      + shift_left(resize(qr2, 10), 1)
                      + resize(qr3, 10);
                gc <= resize(qr2, 10)
                      + shift_left(resize(qr3, 10), 1)
                      + resize(qr4, 10);
                ga1 <= ga;  ga2 <= ga1;
                gb1 <= gb;  gb2 <= gb1;
                gc1 <= gc;  gc2 <= gc1;
                q1d <= qr1;  q1e <= q1d;
                q2d <= qr2;  q2e <= q2d;
                q3d <= qr3;  q3e <= q3d;
                vm0 <= f_med3(qr2, qr3, qr4);
                vm1 <= f_med3(qr1, qr2, qr3);
                vm2 <= f_med3(y8d, qr1, qr2);
                vm0d <= vm0;  vm0e <= vm0d;
                vm1d <= vm1;  vm1e <= vm1d;
                vm2d <= vm2;  vm2e <= vm2d;
            end if;

            -- S6: horizontal [1 2 1] over the vertical sums, /16 -> blurred
            -- rows; separable median second pass (horizontal median-of-3
            -- over the vertical medians). The mux picks median (S10, wins
            -- over smooth), blurred (S9) or raw rows -- all three centered
            -- on the same column, so latency is identical in every mode.
            if av(5) = '1' then
                if s_sw_median = '1' then
                    e0 <= f_med3(vm0, vm0d, vm0e);
                    e1 <= f_med3(vm1, vm1d, vm1e);
                    e2 <= f_med3(vm2, vm2d, vm2e);
                elsif s_sw_smooth = '1' then
                    e2 <= resize(shift_right(resize(ga2, 12)
                              + shift_left(resize(ga1, 12), 1)
                              + resize(ga, 12), 4), 8);
                    e1 <= resize(shift_right(resize(gb2, 12)
                              + shift_left(resize(gb1, 12), 1)
                              + resize(gb, 12), 4), 8);
                    e0 <= resize(shift_right(resize(gc2, 12)
                              + shift_left(resize(gc1, 12), 1)
                              + resize(gc, 12), 4), 8);
                else
                    e2 <= q1e;
                    e1 <= q2e;
                    e0 <= q3e;
                end if;
            end if;

            -- S7: Sobel column sums (e0 = top row, e2 = bottom) and
            -- two-column histories
            if av(6) = '1' then
                ss <= resize(e0, 10) + shift_left(resize(e1, 10), 1)
                      + resize(e2, 10);
                dy <= signed(resize(e0, 9)) - signed(resize(e2, 9));
                ss1 <= ss;  ss2 <= ss1;
                dy1 <= dy;  dy2 <= dy1;
            end if;

            -- S8: gradients
            if av(7) = '1' then
                s_gx <= signed(resize(ss, 12)) - signed(resize(ss2, 12));
                s_gy <= resize(dy, 12) + shift_left(resize(dy1, 12), 1)
                        + resize(dy2, 12);
            end if;

            -- S9: magnitude
            if av(8) = '1' then
                s_mag <= resize(f_absu(s_gx), 12) + resize(f_absu(s_gy), 12);
            end if;

            -- S10: normalize to 0..255; blank warm-up lines (window
            -- straddles the previous field) and left columns (histories
            -- carry the previous line's tail)
            if av(9) = '1' then
                v_m8 := f_c255(shift_right(s_mag, 3));
                if s_aline < 5 or cbx < 10 then
                    v_m8 := (others => '0');
                end if;
                s_mag8 <= v_m8;
                cbx <= cbx + 1;
            end if;

            -- S11: threshold; bit-buffer reads presented this cycle
            if av(10) = '1' then
                if s_mag8 > s_thr then
                    s_eb <= '1';
                else
                    s_eb <= '0';
                end if;
                ebr <= ebr + 1;
            end if;

            -- S12: vertical solidify -- OR with the same column of the
            -- previous two lines -- then min-run. Once a run qualifies
            -- (s_min_run consecutive solid pixels) the edge bit is HELD
            -- high for the rest of the run, pinning the band machine at
            -- red: the outline is drawn solid, exactly as thick as the
            -- detected edge region, and the rainbow starts at its trailing
            -- side. The aline guard keeps the previous field's buffered
            -- bits from triggering on the first lines.
            if av(11) = '1' then
                v_solid := s_eb or eq1(0) or eq2(0);
                if s_aline < 5 then
                    v_solid := '0';
                end if;
                if v_solid = '1' then
                    if s_run_count >= s_min_run_m1 then
                        s_edge_bit <= '1';
                    else
                        s_edge_bit <= '0';
                    end if;
                    if s_run_count < 31 then
                        s_run_count <= s_run_count + 1;
                    end if;
                else
                    s_run_count <= (others => '0');
                    s_edge_bit <= '0';
                end if;
                ebw <= ebw + 1;
            else
                s_edge_bit <= '0';
            end if;

            -- New line: reset counters; count active lines per field.
            -- A line period with no active video is vertical blanking, so
            -- the active-line counter resets THERE -- not on the vsync
            -- edge, whose position relative to the last active lines varies
            -- by source/decoder. (An early vsync parked the warm-up guard's
            -- 5-line black band inside the picture near the bottom on
            -- hardware.) Saturate rather than wrap so an unexpectedly long
            -- field can never re-trigger the guard mid-picture.
            if data_in.hsync_n = '0' and s_prev_hsync_n_p = '1' then
                lrx <= (others => '0');
                lwx <= (others => '0');
                ebr <= (others => '0');
                ebw <= (others => '0');
                cbx <= (others => '0');
                s_run_count <= (others => '0');
                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 1000 then
                        s_aline <= s_aline + 1;
                    end if;
                else
                    s_aline <= (others => '0');
                end if;
            end if;
        end if;
    end process p_pix;

    ----------------------------------------------------------------------------
    -- Luma line buffers: read at lrx (one column ahead of the write), rotate
    -- rows by rewriting each read value one buffer down.
    ----------------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            q1 <= lb1(to_integer(lrx));
            if av(3) = '1' then
                lb1(to_integer(lwx)) <= std_logic_vector(a3_y8);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            q2 <= lb2(to_integer(lrx));
            if av(3) = '1' then
                lb2(to_integer(lwx)) <= q1;
            end if;
        end if;
    end process p_lb2;

    p_lb3 : process(clk)
    begin
        if rising_edge(clk) then
            q3 <= lb3(to_integer(lrx));
            if av(3) = '1' then
                lb3(to_integer(lwx)) <= q2;
            end if;
        end if;
    end process p_lb3;

    p_lb4 : process(clk)
    begin
        if rising_edge(clk) then
            q4 <= lb4(to_integer(lrx));
            if av(3) = '1' then
                lb4(to_integer(lwx)) <= q3;
            end if;
        end if;
    end process p_lb4;

    ----------------------------------------------------------------------------
    -- Solidify bit line buffers: elb1 holds the previous line's edge bits,
    -- elb2 the line before that. Same read-leads-write chaining as the luma
    -- buffers.
    ----------------------------------------------------------------------------
    p_elb1 : process(clk)
    begin
        if rising_edge(clk) then
            eq1 <= elb1(to_integer(ebr));
            if av(11) = '1' then
                elb1(to_integer(ebw)) <= (0 => s_eb);
            end if;
        end if;
    end process p_elb1;

    p_elb2 : process(clk)
    begin
        if rising_edge(clk) then
            eq2 <= elb2(to_integer(ebr));
            if av(11) = '1' then
                elb2(to_integer(ebw)) <= eq1;
            end if;
        end if;
    end process p_elb2;

    ----------------------------------------------------------------------------
    -- Curve: per-band effective widths. With curve enabled (switch 8) the
    -- widths grow geometrically toward violet:
    --   width(i) = max(1, band_width >> ((6 - i) * steepness))
    -- so red is the thinnest band and violet gets the full K1 width. At
    -- steepness 1 each colour doubles the previous one; higher settings pin
    -- more of the early colours at 1 px. The whole table is registered every
    -- clock (it only depends on knobs), so p_band can index it by band_idx
    -- with no stale-size lag when a band advances.
    ----------------------------------------------------------------------------
    p_curve : process(clk)
        variable v_sz : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to 6 loop
                if s_sw_curve_en = '1' then
                    v_sz := shift_right(s_band_width,
                                        (6 - i) * to_integer(s_curve_steep));
                    if v_sz = 0 then
                        v_sz := to_unsigned(1, 8);
                    end if;
                    s_sizes_m1(i) <= v_sz - 1;
                else
                    s_sizes_m1(i) <= s_band_width - 1;
                end if;
            end loop;
            s_sizes_m1(7) <= s_band_width - 1;  -- black region, don't-care
        end if;
    end process p_curve;

    ----------------------------------------------------------------------------
    -- Rainbow band state machine:
    --   edge        → reset to red (band 0, pixel 0)
    --   within band → increment band_pixel
    --   end of band → advance band_idx (saturates at 7 = black)
    --   hsync edge  → back to black (idx 7)
    --
    -- With curve enabled (switch 8), band widths follow the geometric curve
    -- in s_sizes: thin red first, each colour wider toward violet.
    -- The >= compare (not =) means a band can never overshoot its terminal
    -- count and run away to the 7-bit counter wrap.
    ----------------------------------------------------------------------------
    p_band : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_band_idx   <= "111";
                s_band_pixel <= (others => '0');
            elsif av(12) = '1' then
                if s_edge_bit = '1' then
                    s_band_idx   <= (others => '0');
                    s_band_pixel <= (others => '0');
                elsif resize(s_band_pixel, 8) >= s_sizes_m1(to_integer(s_band_idx)) then
                    s_band_pixel <= (others => '0');
                    if s_band_idx /= "111" then
                        s_band_idx <= s_band_idx + 1;
                    end if;
                else
                    s_band_pixel <= s_band_pixel + 1;
                end if;
            end if;
        end if;
    end process p_band;

    ----------------------------------------------------------------------------
    -- Video-key delay: incoming Y/U/V delayed by full SYNC_DELAY so it lines
    -- up with data_out. Used when switch 7 is on and the band index falls
    -- past violet — the "black" region is replaced with pass-through video.
    ----------------------------------------------------------------------------
    p_video_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_y_in_sr(0) <= data_in.y;
            s_u_in_sr(0) <= data_in.u;
            s_v_in_sr(0) <= data_in.v;
            for i in 1 to C_SYNC_DELAY - 1 loop
                s_y_in_sr(i) <= s_y_in_sr(i - 1);
                s_u_in_sr(i) <= s_u_in_sr(i - 1);
                s_v_in_sr(i) <= s_v_in_sr(i - 1);
            end loop;
        end if;
    end process p_video_delay;

    ----------------------------------------------------------------------------
    -- Sync delay: 13 stages — contrast (2) + posterize (1) + BRAM-read
    -- register (1) + column sums/medians (1) + blur/mux (1) + Sobel sums (1)
    -- + gradients (1) + magnitude (1) + normalize (1) + threshold (1) +
    -- solidify/min-run (1) + band (1); palette lookup is combinational on
    -- band_idx.
    ----------------------------------------------------------------------------
    p_sync_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= s_avid_sr   (C_SYNC_DELAY - 2 downto 0) & data_in.avid;
            s_hsync_n_sr <= s_hsync_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.hsync_n;
            s_vsync_n_sr <= s_vsync_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.vsync_n;
            s_field_n_sr <= s_field_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.field_n;
        end if;
    end process p_sync_delay;

    ----------------------------------------------------------------------------
    -- Colour remap table: fold the K1 cycle phase (0..6, knob*7/1024 so a
    -- full turn is one full rotation with no dead zone) and the S11 reverse
    -- into a registered band->palette map. Turning K1 rotates the colours
    -- through the bands, animating the trail; geometry (band widths, curve,
    -- video key) is untouched.
    ----------------------------------------------------------------------------
    p_cmap : process(clk)
        variable v_k7 : unsigned(12 downto 0);
        variable v_s  : integer range 0 to 13;
    begin
        if rising_edge(clk) then
            v_k7 := shift_left(resize(s_knob_color, 13), 3)
                    - resize(s_knob_color, 13);              -- knob * 7
            for i in 0 to 6 loop
                if s_sw_reverse = '1' then
                    v_s := (6 - i) + to_integer(v_k7(12 downto 10));
                else
                    v_s := i + to_integer(v_k7(12 downto 10));
                end if;
                if v_s > 6 then
                    v_s := v_s - 7;
                end if;
                s_cmap(i) <= to_unsigned(v_s, 3);
            end loop;
            s_cmap(7) <= "111";  -- black/key region never remaps
        end if;
    end process p_cmap;

    s_disp_idx <= s_cmap(to_integer(s_band_idx));

    -- Video key: source video fills the past-violet black region
    s_key_video <= '1' when s_sw_video_key = '1' and s_band_idx = "111"
                   else '0';

    data_out.y <= s_y_in_sr(C_SYNC_DELAY - 1)
                      when s_key_video = '1'
                      else std_logic_vector(C_PAL_Y(to_integer(s_disp_idx)));
    data_out.u <= s_u_in_sr(C_SYNC_DELAY - 1)
                      when s_key_video = '1'
                      else std_logic_vector(C_PAL_U(to_integer(s_disp_idx)));
    data_out.v <= s_v_in_sr(C_SYNC_DELAY - 1)
                      when s_key_video = '1'
                      else std_logic_vector(C_PAL_V(to_integer(s_disp_idx)));
    data_out.avid    <= s_avid_sr   (C_SYNC_DELAY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_SYNC_DELAY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_SYNC_DELAY - 1);
    data_out.field_n <= s_field_n_sr(C_SYNC_DELAY - 1);

end phosphor;
