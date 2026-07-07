-- Edge Lab: an edge-detection laboratory.
--
-- One program, eight classic edge detectors on a knob, plus the standard
-- image-cleanup chain you want in front of a detector (brightness, contrast,
-- posterize, optional Gaussian smoothing), edge thickening, a not-edges
-- negative, overlay and raw-gradient views.  Built to *compare* detectors:
-- flip K1 and nothing else changes.
--
-- Detectors (K1):
--   0 Roberts   2x2 cross difference          |a-d| + |b-c|
--   1 Prewitt   3x3 box-derivative            [1 1 1] columns
--   2 Sobel     3x3 smoothed derivative       [1 2 1] columns
--   3 Scharr    3x3 rotation-optimal          [3 10 3] columns
--   4 Kirsch    8-direction compass, max of |5,-3| templates.
--               Uses max_i |8*Si - 3*T| = max(8*maxSi-3T, 3T-8*minSi)
--               (Si = 3 contiguous ring neighbours, T = ring sum), so only
--               a min/max tree over the 8 Si is needed -- no 8 wide ALUs.
--   5 Laplace   3x3 8-neighbour second derivative |9c - sum9|
--   6 LoG       true 5x5 Laplacian-of-Gaussian kernel
--                  0  0 -1  0  0
--                  0 -1 -2 -1  0
--                 -1 -2 16 -2 -1     (all coefficients shift-only)
--                  0 -1 -2 -1  0
--                  0  0 -1  0  0
--   7 Canny     Sobel + gradient-direction quantize + non-maximum
--               suppression + double threshold + causal hysteresis
--               (weak edges survive next to an accepted left/upper edge).
--   8 Phosphor  1-D horizontal first difference |e(x) - e(x-1)|, the
--               phosphor program's detector (luma-only): orientation-blind,
--               sees only vertical edges.  Here for comparison.
--
-- Cleanup / display:
--   K2 Threshold  binarize level (Canny: high; low = high/2)
--   K3 Brightness pre-offset, +-512
--   K4 Contrast   pre-gain 0..4x around mid-grey (unity at 25%)
--   K5 Posterize  quantize luma to 128..2 levels before detection
--   K6 Thickness  dilate the edge map: 1px / 2x2 / 3x3 / 5x3
--   S7 Smooth     3x3 Gaussian pre-blur feeding every 3x3 detector
--                 (Laplace+Smooth is the classic separable LoG recipe)
--   S8 Invert     negative: not-edges (black lines on white)
--   S9 View       Edges / Source (see exactly what the detector sees,
--                 for dialing in K3-K5 before flipping back)
--   S10 Overlay   draw edges on top of the live picture instead of black
--   S11 Output    Binary / Gradient (raw magnitude as grayscale; Canny
--                 shows the post-NMS magnitude -- thinned ridges)
--   P12 Mix       dry/wet crossfade
--
-- Structure (all luma; chroma is neutral except overlay/mix):
--   S1-S4   preprocess: (y-512)*gain>>4 + bright, clamp, posterize -> 8 bit
--   S5      4 chained line buffers -> 5x5 raw window + per-column sums
--   S6-S8   LoG column pipeline; [1 2 1]^2 blurred 3x3; Smooth mux -> e-cols
--   S9-S14  detector arithmetic (everything computed in parallel, one
--           registered add-level per stage; Kirsch min/max tree sets depth)
--   S15     per-detector normalize to 0..255, clamp, mode mux, edge blank
--   S16-S17 magnitude line buffers -> 3x3 mag window, NMS (Canny only,
--           passthrough otherwise so every mode shares one latency)
--   S18     threshold + hysteresis (accept-bit line buffer)
--   S19     thickness dilate (2-line packed bit buffer)
--   S20     compose views; then interpolator_u dry/wet mix (4 clocks)
--
-- All line buffers are canonical 1W1R single-process BRAMs, chained by
-- rewriting each read value one buffer down (inferno lesson).  The dry-video
-- delay for overlay/mix is a 32-deep EBR ring (2 EBR) instead of a 24-stage
-- 30-bit shift register (~700 LCs saved).  ~29 EBR total.
--
-- The output is a fixed few pixels left / 3 lines up of the input (window
-- centering); constant, and invisible in practice.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture edgelab of program_top is

    constant C_DW            : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_TOTAL_LATENCY : integer := 24;                  -- 20 wet + 4 mix

    ----------------------------------------------------------------------
    -- Helpers
    ----------------------------------------------------------------------
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

    ----------------------------------------------------------------------
    -- Line buffers (BRAM)
    ----------------------------------------------------------------------
    type t_lb8 is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_lb2 is array (0 to 2047) of std_logic_vector(1 downto 0);
    type t_lb1 is array (0 to 2047) of std_logic_vector(0 downto 0);

    signal lb1, lb2, lb3, lb4 : t_lb8 := (others => (others => '0'));  -- luma
    signal mb1, mb2           : t_lb8 := (others => (others => '0'));  -- mag
    signal db1                : t_lb2 := (others => (others => '0'));  -- dir
    signal ab1                : t_lb1 := (others => (others => '0'));  -- accept
    signal dd1                : t_lb2 := (others => (others => '0'));  -- dilate

    signal q1, q2, q3, q4 : std_logic_vector(7 downto 0) := (others => '0');
    signal mq1, mq2       : std_logic_vector(7 downto 0) := (others => '0');
    signal dq             : std_logic_vector(1 downto 0) := (others => '0');
    signal aq             : std_logic_vector(0 downto 0) := (others => '0');
    signal ddq            : std_logic_vector(1 downto 0) := (others => '0');

    -- Dry video delay ring (overlay + mix): 32 x 30 EBR
    type t_ring is array (0 to 31) of std_logic_vector(29 downto 0);
    signal vring : t_ring := (others => (others => '0'));
    signal vr_wp : unsigned(4 downto 0) := (others => '0');
    signal vr_q  : std_logic_vector(29 downto 0) := (others => '0');
    signal vr_q2 : std_logic_vector(29 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Valid chain / counters
    ----------------------------------------------------------------------
    signal av : std_logic_vector(1 to 20) := (others => '0');

    signal lrx, lwx : unsigned(10 downto 0) := (others => '0');  -- luma rd/wr
    signal mrx, mwx : unsigned(10 downto 0) := (others => '0');  -- mag rd/wr
    signal abr, abw : unsigned(10 downto 0) := (others => '0');  -- accept
    signal drx, dwx : unsigned(10 downto 0) := (others => '0');  -- dilate
    signal cbx      : unsigned(10 downto 0) := (others => '0');  -- blank @S15
    signal gbx      : unsigned(10 downto 0) := (others => '0');  -- guard @S18
    signal obx      : unsigned(10 downto 0) := (others => '0');  -- guard @S19

    signal s_prev_hsync_n, s_prev_vsync_n : std_logic := '1';
    signal s_aline : unsigned(9 downto 0) := (others => '0');
    signal s_seen  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Frame-latched controls
    ----------------------------------------------------------------------
    signal s_mode   : unsigned(3 downto 0) := (others => '0');
    signal s_thr    : unsigned(7 downto 0) := (others => '0');
    signal s_thrlo  : unsigned(7 downto 0) := (others => '0');
    signal s_base   : signed(11 downto 0) := to_signed(512, 12);  -- 512+bright
    signal s_glo    : unsigned(2 downto 0) := "000";
    signal s_ghi    : unsigned(2 downto 0) := "010";
    signal s_pmask  : std_logic_vector(9 downto 0) := (others => '1');
    signal s_thick  : unsigned(1 downto 0) := (others => '0');
    signal s_smooth : std_logic := '0';
    signal s_inv    : std_logic := '0';
    signal s_view   : std_logic := '0';
    signal s_ovl    : std_logic := '0';
    signal s_gview  : std_logic := '0';

    ----------------------------------------------------------------------
    -- S1-S4 preprocess
    ----------------------------------------------------------------------
    signal a1_c        : signed(11 downto 0) := (others => '0');
    signal a2_lo, a2_hi : signed(15 downto 0) := (others => '0');
    signal a3_y        : unsigned(9 downto 0) := (others => '0');
    signal a4_y8       : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S5 window + column sums
    ----------------------------------------------------------------------
    type t_row5 is array (0 to 4) of unsigned(7 downto 0);
    type t_win  is array (0 to 4) of t_row5;
    signal win : t_win := (others => (others => (others => '0')));
    signal w2x : unsigned(7 downto 0) := (others => '0');  -- win(2) 6th tap

    signal a5_ga, a5_gb, a5_gc : unsigned(9 downto 0) := (others => '0');
    signal a5_ldn : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S6-S8 LoG / blur / e-columns
    ----------------------------------------------------------------------
    signal a6_ld : signed(13 downto 0) := (others => '0');
    signal ld1   : signed(13 downto 0) := (others => '0');
    signal g1c, g2c, g3c, g4c : unsigned(9 downto 0) := (others => '0');
    signal t1c, t2c           : unsigned(9 downto 0) := (others => '0');
    signal b1c, b2c           : unsigned(9 downto 0) := (others => '0');

    signal a7_lp : signed(14 downto 0) := (others => '0');
    signal a7_ws : unsigned(8 downto 0) := (others => '0');
    signal a7_bt, a7_bm, a7_bb : unsigned(7 downto 0) := (others => '0');
    signal a8_log : signed(14 downto 0) := (others => '0');
    signal a8_e0, a8_e1, a8_e2 : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S9 sums + e-column history
    ----------------------------------------------------------------------
    signal a9_sp : unsigned(9 downto 0) := (others => '0');
    signal a9_ss : unsigned(9 downto 0) := (others => '0');
    signal a9_sc : unsigned(11 downto 0) := (others => '0');
    signal a9_dy : signed(8 downto 0) := (others => '0');
    signal sp1, sp2 : unsigned(9 downto 0) := (others => '0');
    signal ss1, ss2 : unsigned(9 downto 0) := (others => '0');
    signal sc1, sc2 : unsigned(11 downto 0) := (others => '0');
    signal dy1, dy2 : signed(8 downto 0) := (others => '0');
    type t_ecol is array (0 to 2) of unsigned(7 downto 0);
    signal ec0, ec1, ec2 : t_ecol := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- S10 gradients / ring
    ----------------------------------------------------------------------
    signal a10_gxp, a10_gyp : signed(10 downto 0) := (others => '0');
    signal a10_gxs, a10_gys : signed(11 downto 0) := (others => '0');
    signal a10_gxc, a10_gyc : signed(12 downto 0) := (others => '0');
    signal a10_r1, a10_r2   : unsigned(7 downto 0) := (others => '0');
    signal a10_ph           : unsigned(7 downto 0) := (others => '0');
    signal a10_t9  : unsigned(11 downto 0) := (others => '0');
    signal a10_l9c : unsigned(11 downto 0) := (others => '0');
    signal a10_cen : unsigned(7 downto 0) := (others => '0');
    type t_si is array (0 to 7) of unsigned(9 downto 0);
    signal a10_si : t_si := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- S11-S14 magnitudes, Kirsch tree, dir, LoG delay
    ----------------------------------------------------------------------
    signal a11_mp  : unsigned(10 downto 0) := (others => '0');
    signal a11_ms  : unsigned(11 downto 0) := (others => '0');
    signal a11_mc  : unsigned(12 downto 0) := (others => '0');
    signal a11_rob : unsigned(8 downto 0) := (others => '0');
    signal a11_ph  : unsigned(7 downto 0) := (others => '0');
    signal ph_d1, ph_d2, ph_d3 : unsigned(7 downto 0) := (others => '0');
    signal a11_lap : unsigned(11 downto 0) := (others => '0');
    signal a11_t3  : unsigned(12 downto 0) := (others => '0');
    signal a11_ax, a11_ay : unsigned(11 downto 0) := (others => '0');
    signal a11_ds  : std_logic := '0';
    type t_si4 is array (0 to 3) of unsigned(9 downto 0);
    type t_si2 is array (0 to 1) of unsigned(9 downto 0);
    signal simax1, simin1 : t_si4 := (others => (others => '0'));
    signal simax2, simin2 : t_si2 := (others => (others => '0'));
    signal simax3, simin3 : unsigned(9 downto 0) := (others => '0');
    signal t3d1, t3d2 : unsigned(12 downto 0) := (others => '0');
    signal a14_kir : unsigned(12 downto 0) := (others => '0');

    signal a12_dir : unsigned(1 downto 0) := (others => '0');
    signal dir_d1, dir_d2 : unsigned(1 downto 0) := (others => '0');

    -- pass-along delays to the S15 mux (Kirsch is the long pole)
    signal mp_d1, mp_d2, mp_d3    : unsigned(10 downto 0) := (others => '0');
    signal ms_d1, ms_d2, ms_d3    : unsigned(11 downto 0) := (others => '0');
    signal mc_d1, mc_d2, mc_d3    : unsigned(12 downto 0) := (others => '0');
    signal rob_d1, rob_d2, rob_d3 : unsigned(8 downto 0) := (others => '0');
    signal lap_d1, lap_d2, lap_d3 : unsigned(11 downto 0) := (others => '0');
    signal log_a  : unsigned(13 downto 0) := (others => '0');  -- |log| @S11
    signal log_d1, log_d2, log_d3 : unsigned(13 downto 0) := (others => '0');
    signal logp_1, logp_2 : signed(14 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S15-S19 mag select, NMS, threshold, dilate
    ----------------------------------------------------------------------
    signal a15_mag : unsigned(7 downto 0) := (others => '0');
    signal a15_dir : unsigned(1 downto 0) := (others => '0');
    signal mq1r, mq2r : unsigned(7 downto 0) := (others => '0');

    type t_m3 is array (0 to 2) of unsigned(7 downto 0);
    signal mr0, mr1, mr2 : t_m3 := (others => (others => '0'));
    signal dird1, dird2, dird3 : unsigned(1 downto 0) := (others => '0');

    signal a17_mag : unsigned(7 downto 0) := (others => '0');
    signal a18_mag : unsigned(7 downto 0) := (others => '0');
    signal a18_bit : std_logic := '0';
    signal leftacc : std_logic := '0';
    signal aw1, aw2 : std_logic := '0';

    signal ddqr : std_logic_vector(1 downto 0) := (others => '0');
    signal hb : std_logic_vector(0 to 3) := (others => '0');
    signal v1, v2 : std_logic_vector(0 to 4) := (others => '0');
    signal a19_bit : std_logic := '0';
    signal mg_0, mg_1, mg_2 : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- S20 compose + source-view delay
    ----------------------------------------------------------------------
    type t_src is array (0 to 10) of unsigned(7 downto 0);
    signal srcd : t_src := (others => (others => '0'));
    signal a20_y, a20_u, a20_v : unsigned(C_DW - 1 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Sync delay + mix
    ----------------------------------------------------------------------
    type t_bit_shift is array (0 to C_TOTAL_LATENCY - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

    signal s_mix_t : unsigned(C_DW - 1 downto 0);
    signal s_interp_y_result, s_interp_u_result, s_interp_v_result
        : unsigned(C_DW - 1 downto 0);
    signal s_interp_y_valid, s_interp_u_valid, s_interp_v_valid : std_logic;

begin

    s_mix_t <= unsigned(registers_in(7));

    ----------------------------------------------------------------------
    -- Main pipeline
    ----------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sum   : signed(19 downto 0);
        variable v_v     : signed(12 downto 0);
        variable v_wc    : t_row5;
        variable v_p13   : unsigned(9 downto 0);
        variable v_p2    : unsigned(8 downto 0);
        variable v_e02   : unsigned(8 downto 0);
        variable v_tot9  : unsigned(11 downto 0);
        variable v_ring  : t_si;
        variable v_p10   : unsigned(9 downto 0);
        variable v_dysum : signed(9 downto 0);
        variable v_t     : unsigned(11 downto 0);
        variable v_ka, v_kb : signed(14 downto 0);
        variable v_m     : unsigned(7 downto 0);
        variable v_na, v_nb : unsigned(7 downto 0);
        variable v_keep  : std_logic;
        variable v_str, v_wk, v_acc : std_logic;
        variable v_bit   : std_logic;
        variable v_mag   : unsigned(7 downto 0);
        variable v_g6    : unsigned(5 downto 0);
        variable v_k1    : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            -- valid chain
            av(1) <= data_in.avid;
            for i in 2 to 20 loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            ------------------------------------------------------------------
            -- S1: center luma for contrast (pivot mid-grey)
            ------------------------------------------------------------------
            if data_in.avid = '1' then
                a1_c <= signed(resize(unsigned(data_in.y), 12)) - 512;
            end if;

            ------------------------------------------------------------------
            -- S2: contrast gain split into two 12x3 partial products
            ------------------------------------------------------------------
            if av(1) = '1' then
                a2_lo <= a1_c * signed(resize(s_glo, 4));
                a2_hi <= a1_c * signed(resize(s_ghi, 4));
            end if;

            ------------------------------------------------------------------
            -- S3: combine, add base (512 + brightness), clamp
            ------------------------------------------------------------------
            if av(2) = '1' then
                v_sum := resize(a2_lo, 20) + shift_left(resize(a2_hi, 20), 3);
                v_v   := resize(s_base, 13) + resize(shift_right(v_sum, 4), 13);
                if v_v < 0 then
                    a3_y <= (others => '0');
                elsif v_v > 1023 then
                    a3_y <= (others => '1');
                else
                    a3_y <= unsigned(v_v(9 downto 0));
                end if;
            end if;

            ------------------------------------------------------------------
            -- S4: posterize, reduce to 8 bit
            ------------------------------------------------------------------
            if av(3) = '1' then
                v_p10 := a3_y and unsigned(s_pmask);
                a4_y8 <= v_p10(9 downto 2);
                lrx   <= lrx + 1;   -- luma reads were presented this cycle
            end if;

            ------------------------------------------------------------------
            -- S5: 5x5 window shift, buffer rotate writes happen in the BRAM
            -- processes; per-column vertical sums.
            -- Rows: w0 = current line ... w4 = 4 lines up (top of window).
            ------------------------------------------------------------------
            if av(4) = '1' then
                v_wc(0) := a4_y8;
                v_wc(1) := unsigned(q1);
                v_wc(2) := unsigned(q2);
                v_wc(3) := unsigned(q3);
                v_wc(4) := unsigned(q4);
                for r in 0 to 4 loop
                    win(r)(0) <= v_wc(r);
                    for c in 1 to 4 loop
                        win(r)(c) <= win(r)(c - 1);
                    end loop;
                end loop;
                w2x <= win(2)(4);

                -- [1 2 1] vertical sums for blurred rows 1..3
                a5_ga <= resize(v_wc(0), 10) + resize(v_wc(2), 10)
                         + shift_left(resize(v_wc(1), 10), 1);
                a5_gb <= resize(v_wc(1), 10) + resize(v_wc(3), 10)
                         + shift_left(resize(v_wc(2), 10), 1);
                a5_gc <= resize(v_wc(2), 10) + resize(v_wc(4), 10)
                         + shift_left(resize(v_wc(3), 10), 1);
                -- LoG center-column negative part: w0 + 2w1 + 2w3 + w4
                v_p13 := resize(v_wc(1), 10) + resize(v_wc(3), 10);
                v_p2  := resize(v_wc(0), 9) + resize(v_wc(4), 9);
                a5_ldn <= resize(v_p2, 12) + shift_left(resize(v_p13, 12), 1);

                lwx <= lwx + 1;
            end if;

            ------------------------------------------------------------------
            -- S6: LoG center column 16*w2 - ldn; start delay chains
            ------------------------------------------------------------------
            if av(5) = '1' then
                a6_ld <= signed(shift_left(resize(win(2)(0), 14), 4))
                         - signed(resize(a5_ldn, 14));
                g1c <= a5_gb;  g2c <= g1c;  g3c <= g2c;  g4c <= g3c;
                t1c <= a5_ga;  t2c <= t1c;
                b1c <= a5_gc;  b2c <= b1c;
            end if;

            ------------------------------------------------------------------
            -- S7: LoG partial (center - inner columns); 3x3 Gaussian blur
            -- (horizontal [1 2 1] over the vertical sums, /16)
            ------------------------------------------------------------------
            if av(6) = '1' then
                ld1   <= a6_ld;
                a7_lp <= resize(ld1, 15)
                         - signed(resize(resize(g1c, 11) + g3c, 15));
                a7_ws <= resize(win(2)(1), 9) + w2x;
                a7_bt <= resize(shift_right(resize(t2c, 12)
                         + shift_left(resize(t1c, 12), 1)
                         + resize(a5_ga, 12), 4), 8);
                a7_bm <= resize(shift_right(resize(g2c, 12)
                         + shift_left(resize(g1c, 12), 1)
                         + resize(a5_gb, 12), 4), 8);
                a7_bb <= resize(shift_right(resize(b2c, 12)
                         + shift_left(resize(b1c, 12), 1)
                         + resize(a5_gc, 12), 4), 8);
            end if;

            ------------------------------------------------------------------
            -- S8: LoG complete; Smooth mux -> effective 3-row column
            -- (e0 = upper row, e1 = center, e2 = lower row)
            ------------------------------------------------------------------
            if av(7) = '1' then
                a8_log <= a7_lp - signed(resize(a7_ws, 15));
                if s_smooth = '1' then
                    a8_e0 <= a7_bt;
                    a8_e1 <= a7_bm;
                    a8_e2 <= a7_bb;
                else
                    a8_e0 <= win(1)(2);
                    a8_e1 <= win(2)(2);
                    a8_e2 <= win(3)(2);
                end if;
            end if;

            ------------------------------------------------------------------
            -- S9: per-column detector sums + histories
            ------------------------------------------------------------------
            if av(8) = '1' then
                v_e02 := resize(a8_e0, 9) + a8_e2;
                a9_sp <= resize(v_e02, 10) + a8_e1;
                a9_ss <= resize(v_e02, 10) + shift_left(resize(a8_e1, 10), 1);
                -- Scharr column: 3*(e0+e2) + 10*e1
                a9_sc <= resize(v_e02, 12) + shift_left(resize(v_e02, 12), 1)
                         + shift_left(resize(a8_e1, 12), 3)
                         + shift_left(resize(a8_e1, 12), 1);
                a9_dy <= signed(resize(a8_e0, 9)) - signed(resize(a8_e2, 9));
                sp1 <= a9_sp;  sp2 <= sp1;
                ss1 <= a9_ss;  ss2 <= ss1;
                sc1 <= a9_sc;  sc2 <= sc1;
                dy1 <= a9_dy;  dy2 <= dy1;
                ec0(0) <= a8_e0;  ec0(1) <= a8_e1;  ec0(2) <= a8_e2;
                ec1 <= ec0;
                ec2 <= ec1;
                srcd(0) <= a8_e1;
                for i in 1 to 10 loop
                    srcd(i) <= srcd(i - 1);
                end loop;
            end if;

            ------------------------------------------------------------------
            -- S10: gradients (center = middle column of the histories),
            -- Roberts diffs, Laplace terms, Kirsch ring sums
            ------------------------------------------------------------------
            if av(9) = '1' then
                a10_gxp <= signed(resize(a9_sp, 11)) - signed(resize(sp2, 11));
                a10_gxs <= signed(resize(a9_ss, 12)) - signed(resize(ss2, 12));
                a10_gxc <= signed(resize(a9_sc, 13)) - signed(resize(sc2, 13));
                a10_gyp <= resize(a9_dy, 11) + resize(dy1, 11) + resize(dy2, 11);
                a10_gys <= resize(a9_dy, 12) + shift_left(resize(dy1, 12), 1)
                           + resize(dy2, 12);
                v_dysum := resize(a9_dy, 10) + resize(dy2, 10);
                a10_gyc <= resize(v_dysum, 13)
                           + shift_left(resize(v_dysum, 13), 1)
                           + shift_left(resize(dy1, 13), 3)
                           + shift_left(resize(dy1, 13), 1);
                -- Roberts cross on the 2x2 (rows e0/e1, this + previous col)
                a10_r1 <= f_absu(signed(resize(ec1(0), 9))
                                 - signed(resize(ec0(1), 9)))(7 downto 0);
                a10_r2 <= f_absu(signed(resize(ec0(0), 9))
                                 - signed(resize(ec1(1), 9)))(7 downto 0);
                -- Phosphor: adjacent horizontal difference, center row
                a10_ph <= f_absu(signed(resize(ec0(1), 9))
                                 - signed(resize(ec1(1), 9)))(7 downto 0);
                -- Laplace: 9*center vs 3x3 total
                v_tot9 := resize(a9_sp, 12) + sp1 + sp2;
                a10_t9  <= v_tot9;
                a10_l9c <= shift_left(resize(ec1(1), 12), 3) + ec1(1);
                a10_cen <= ec1(1);
                -- Kirsch ring, clockwise from upper-left (row 0 = up)
                v_ring(0) := resize(ec2(0), 10);   -- UL
                v_ring(1) := resize(ec1(0), 10);   -- U
                v_ring(2) := resize(ec0(0), 10);   -- UR
                v_ring(3) := resize(ec0(1), 10);   -- R
                v_ring(4) := resize(ec0(2), 10);   -- DR
                v_ring(5) := resize(ec1(2), 10);   -- D
                v_ring(6) := resize(ec2(2), 10);   -- DL
                v_ring(7) := resize(ec2(1), 10);   -- L
                for i in 0 to 7 loop
                    a10_si(i) <= v_ring(i) + v_ring((i + 1) mod 8)
                                 + v_ring((i + 2) mod 8);
                end loop;
            end if;

            ------------------------------------------------------------------
            -- S11: |gx|+|gy| magnitudes, Roberts/Laplace finals, Kirsch
            -- 3T and min/max level 1, Canny |gx| |gy| for direction
            ------------------------------------------------------------------
            if av(10) = '1' then
                a11_mp  <= resize(f_absu(a10_gxp), 11) + f_absu(a10_gyp);
                a11_ms  <= resize(f_absu(a10_gxs), 12) + f_absu(a10_gys);
                a11_mc  <= resize(f_absu(a10_gxc), 13) + f_absu(a10_gyc);
                a11_rob <= resize(a10_r1, 9) + a10_r2;
                a11_ph  <= a10_ph;
                a11_lap <= f_absu(signed(resize(a10_l9c, 13))
                                  - signed(resize(a10_t9, 13)))(11 downto 0);
                v_t := a10_t9 - a10_cen;                       -- ring sum T
                a11_t3 <= resize(v_t, 13) + shift_left(resize(v_t, 13), 1);
                for i in 0 to 3 loop
                    if a10_si(2 * i) > a10_si(2 * i + 1) then
                        simax1(i) <= a10_si(2 * i);
                        simin1(i) <= a10_si(2 * i + 1);
                    else
                        simax1(i) <= a10_si(2 * i + 1);
                        simin1(i) <= a10_si(2 * i);
                    end if;
                end loop;
                a11_ax <= f_absu(a10_gxs);
                a11_ay <= f_absu(a10_gys);
                a11_ds <= a10_gxs(11) xor a10_gys(11);
                logp_1 <= a8_log;   -- LoG align: +2 to reach S11 timing
                logp_2 <= logp_1;
                log_a  <= f_absu(logp_2)(13 downto 0);
            end if;

            ------------------------------------------------------------------
            -- S12: Canny direction quantize; Kirsch level 2; delays
            ------------------------------------------------------------------
            if av(11) = '1' then
                if a11_ay > shift_left(resize(a11_ax, 13), 1) then
                    a12_dir <= "10";                        -- vertical grad
                elsif a11_ax > shift_left(resize(a11_ay, 13), 1) then
                    a12_dir <= "00";                        -- horizontal grad
                elsif a11_ds = '1' then
                    a12_dir <= "11";                        -- gx,gy opposite
                else
                    a12_dir <= "01";
                end if;
                for i in 0 to 1 loop
                    if simax1(2 * i) > simax1(2 * i + 1) then
                        simax2(i) <= simax1(2 * i);
                    else
                        simax2(i) <= simax1(2 * i + 1);
                    end if;
                    if simin1(2 * i) < simin1(2 * i + 1) then
                        simin2(i) <= simin1(2 * i);
                    else
                        simin2(i) <= simin1(2 * i + 1);
                    end if;
                end loop;
                t3d1 <= a11_t3;
                mp_d1 <= a11_mp;  ms_d1 <= a11_ms;  mc_d1 <= a11_mc;
                rob_d1 <= a11_rob;  lap_d1 <= a11_lap;  log_d1 <= log_a;
                ph_d1 <= a11_ph;
            end if;

            ------------------------------------------------------------------
            -- S13: Kirsch level 3; delays
            ------------------------------------------------------------------
            if av(12) = '1' then
                if simax2(0) > simax2(1) then
                    simax3 <= simax2(0);
                else
                    simax3 <= simax2(1);
                end if;
                if simin2(0) < simin2(1) then
                    simin3 <= simin2(0);
                else
                    simin3 <= simin2(1);
                end if;
                t3d2 <= t3d1;
                dir_d1 <= a12_dir;
                mp_d2 <= mp_d1;  ms_d2 <= ms_d1;  mc_d2 <= mc_d1;
                rob_d2 <= rob_d1;  lap_d2 <= lap_d1;  log_d2 <= log_d1;
                ph_d2 <= ph_d1;
            end if;

            ------------------------------------------------------------------
            -- S14: Kirsch final: max(8*maxSi - 3T, 3T - 8*minSi)
            ------------------------------------------------------------------
            if av(13) = '1' then
                v_ka := signed(resize(shift_left(resize(simax3, 13), 3), 15))
                        - signed(resize(t3d2, 15));
                v_kb := signed(resize(t3d2, 15))
                        - signed(resize(shift_left(resize(simin3, 13), 3), 15));
                if v_ka > v_kb then
                    a14_kir <= unsigned(v_ka(12 downto 0));
                else
                    a14_kir <= unsigned(v_kb(12 downto 0));
                end if;
                dir_d2 <= dir_d1;
                mp_d3 <= mp_d2;  ms_d3 <= ms_d2;  mc_d3 <= mc_d2;
                rob_d3 <= rob_d2;  lap_d3 <= lap_d2;  log_d3 <= log_d2;
                ph_d3 <= ph_d2;
                mrx <= mrx + 1;   -- mag-window reads presented this cycle
            end if;

            ------------------------------------------------------------------
            -- S15: normalize every detector to ~0..255, mode mux, blanking
            ------------------------------------------------------------------
            if av(14) = '1' then
                case to_integer(s_mode) is
                    when 0      => v_m := f_c255(shift_right(resize(rob_d3, 9), 1));
                    when 1      => v_m := f_c255(shift_right(mp_d3, 2));
                    when 2      => v_m := f_c255(shift_right(ms_d3, 3));
                    when 3      => v_m := f_c255(shift_right(mc_d3, 5));
                    when 4      => v_m := f_c255(shift_right(a14_kir, 4));
                    when 5      => v_m := f_c255(shift_right(lap_d3, 3));
                    when 6      => v_m := f_c255(shift_right(log_d3, 4));
                    when 8      => v_m := ph_d3;
                    when others => v_m := f_c255(shift_right(ms_d3, 3));
                end case;
                -- blank warm-up lines (window straddles previous frame) and
                -- left columns (shift chains carry the previous line's tail)
                if s_aline < 6 or cbx < 12 then
                    v_m := (others => '0');
                end if;
                a15_mag <= v_m;
                a15_dir <= dir_d2;
                mq1r <= unsigned(mq1);
                mq2r <= unsigned(mq2);
                dird1 <= unsigned(dq);
                cbx  <= cbx + 1;
            end if;

            ------------------------------------------------------------------
            -- S16: 3x3 magnitude window (rows: mr0 = current line = below,
            -- mr1 = center line, mr2 = up; col 0 = right)
            ------------------------------------------------------------------
            if av(15) = '1' then
                mr0(0) <= a15_mag;  mr0(1) <= mr0(0);  mr0(2) <= mr0(1);
                mr1(0) <= mq1r;     mr1(1) <= mr1(0);  mr1(2) <= mr1(1);
                mr2(0) <= mq2r;     mr2(1) <= mr2(0);  mr2(2) <= mr2(1);
                dird2 <= dird1;
                dird3 <= dird2;
                mwx <= mwx + 1;
            end if;

            ------------------------------------------------------------------
            -- S17: non-maximum suppression (Canny), passthrough otherwise
            ------------------------------------------------------------------
            if av(16) = '1' then
                case to_integer(dird3) is
                    when 0 =>      -- horizontal gradient: left/right
                        v_na := mr1(0);  v_nb := mr1(2);
                    when 2 =>      -- vertical gradient: up/down
                        v_na := mr2(1);  v_nb := mr0(1);
                    when 3 =>      -- gx,gy opposite signs: DR / UL
                        v_na := mr0(0);  v_nb := mr2(2);
                    when others => -- same signs: DL / UR
                        v_na := mr0(2);  v_nb := mr2(0);
                end case;
                v_keep := '0';
                if mr1(1) >= v_na and mr1(1) > v_nb then
                    v_keep := '1';
                end if;
                if to_integer(s_mode) = 7 and v_keep = '0' then
                    a17_mag <= (others => '0');
                else
                    a17_mag <= mr1(1);
                end if;
            end if;

            ------------------------------------------------------------------
            -- S18: threshold; Canny double threshold + causal hysteresis
            ------------------------------------------------------------------
            if av(17) = '1' then
                v_str := '0';
                v_wk  := '0';
                if a17_mag > s_thr then
                    v_str := '1';
                end if;
                if a17_mag > s_thrlo then
                    v_wk := '1';
                end if;
                if to_integer(s_mode) = 7 then
                    v_acc := v_str or (v_wk and (leftacc or aq(0) or aw1 or aw2));
                else
                    v_acc := v_str;
                end if;
                if gbx < 16 then
                    v_acc := '0';
                end if;
                a18_bit <= v_acc;
                leftacc <= v_acc;
                a18_mag <= a17_mag;
                abw <= abw + 1;
                gbx <= gbx + 1;
            end if;
            if av(16) = '1' then
                aw1 <= aq(0);
                aw2 <= aw1;
                drx <= drx + 1;
            end if;
            if av(15) = '1' then
                abr <= abr + 1;  -- accept reads lead the write by two columns
            end if;

            ------------------------------------------------------------------
            -- S19: thickness dilate over a 5x3 bit window
            -- rows: current (a18_bit + hb), v1 = line-1 (center), v2 = line-2
            ------------------------------------------------------------------
            if av(17) = '1' then
                ddqr <= ddq;
            end if;
            if av(18) = '1' then
                hb(0) <= a18_bit;
                for i in 1 to 3 loop
                    hb(i) <= hb(i - 1);
                end loop;
                v1(0) <= ddqr(1);
                v2(0) <= ddqr(0);
                for i in 1 to 4 loop
                    v1(i) <= v1(i - 1);
                    v2(i) <= v2(i - 1);
                end loop;
                case to_integer(s_thick) is
                    when 0 =>
                        v_bit := v1(2);
                    when 1 =>
                        v_bit := v1(2) or v1(1) or hb(1) or hb(0);
                    when 2 =>
                        v_bit := hb(0) or hb(1) or hb(2)
                              or v1(1) or v1(2) or v1(3)
                              or v2(1) or v2(2) or v2(3);
                    when others =>
                        v_bit := a18_bit or hb(0) or hb(1) or hb(2) or hb(3)
                              or v1(0) or v1(1) or v1(2) or v1(3) or v1(4)
                              or v2(0) or v2(1) or v2(2) or v2(3) or v2(4);
                end case;
                if obx < 20 then
                    v_bit := '0';
                end if;
                a19_bit <= v_bit;
                mg_0 <= a18_mag;
                mg_1 <= mg_0;
                mg_2 <= mg_1;
                dwx <= dwx + 1;
                obx <= obx + 1;
            end if;

            ------------------------------------------------------------------
            -- S20: compose the wet pixel
            ------------------------------------------------------------------
            if av(19) = '1' then
                v_mag := mg_2;
                if s_inv = '1' then
                    v_mag := 255 - v_mag;
                end if;
                v_bit := a19_bit xor s_inv;
                if s_view = '1' then                     -- Source view
                    a20_y <= srcd(10) & "00";
                    a20_u <= to_unsigned(512, C_DW);
                    a20_v <= to_unsigned(512, C_DW);
                elsif s_gview = '1' then                 -- Gradient view
                    a20_y <= v_mag & "00";
                    a20_u <= to_unsigned(512, C_DW);
                    a20_v <= to_unsigned(512, C_DW);
                elsif s_ovl = '1' then                   -- Overlay
                    if a19_bit = '1' then
                        if s_inv = '1' then
                            a20_y <= (others => '0');
                        else
                            a20_y <= (others => '1');
                        end if;
                        a20_u <= to_unsigned(512, C_DW);
                        a20_v <= to_unsigned(512, C_DW);
                    else
                        a20_y <= unsigned(vr_q(29 downto 20));
                        a20_u <= unsigned(vr_q(19 downto 10));
                        a20_v <= unsigned(vr_q(9 downto 0));
                    end if;
                else                                     -- Binary edges
                    if v_bit = '1' then
                        a20_y <= (others => '1');
                    else
                        a20_y <= (others => '0');
                    end if;
                    a20_u <= to_unsigned(512, C_DW);
                    a20_v <= to_unsigned(512, C_DW);
                end if;
            end if;

            ------------------------------------------------------------------
            -- New line / new frame
            ------------------------------------------------------------------
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                lrx <= (others => '0');
                lwx <= (others => '0');
                mrx <= (others => '0');
                mwx <= (others => '0');
                abr <= (others => '0');
                abw <= (others => '0');
                drx <= (others => '0');
                dwx <= (others => '0');
                cbx <= (others => '0');
                gbx <= (others => '0');
                obx <= (others => '0');
                leftacc <= '0';
                if s_seen = '1' then
                    s_seen  <= '0';
                    s_aline <= s_aline + 1;
                end if;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_aline <= (others => '0');
                s_seen  <= '0';

                -- latch controls once per frame; K1 has 9 zones (1024/9)
                v_k1 := unsigned(registers_in(0));
                if    v_k1 < 114 then s_mode <= to_unsigned(0, 4);
                elsif v_k1 < 228 then s_mode <= to_unsigned(1, 4);
                elsif v_k1 < 341 then s_mode <= to_unsigned(2, 4);
                elsif v_k1 < 455 then s_mode <= to_unsigned(3, 4);
                elsif v_k1 < 569 then s_mode <= to_unsigned(4, 4);
                elsif v_k1 < 683 then s_mode <= to_unsigned(5, 4);
                elsif v_k1 < 796 then s_mode <= to_unsigned(6, 4);
                elsif v_k1 < 910 then s_mode <= to_unsigned(7, 4);
                else                  s_mode <= to_unsigned(8, 4);
                end if;
                s_thr   <= unsigned(registers_in(1)(9 downto 2));
                s_thrlo <= unsigned('0' & registers_in(1)(9 downto 3));
                s_base <= to_signed(512, 12)
                          + (signed(resize(unsigned(registers_in(2)), 12)) - 512);
                v_g6  := unsigned(registers_in(3)(9 downto 4));
                s_glo <= v_g6(2 downto 0);
                s_ghi <= v_g6(5 downto 3);
                case to_integer(unsigned(registers_in(4)(9 downto 7))) is
                    when 0      => s_pmask <= "1111111111";
                    when 1      => s_pmask <= "1111111000";
                    when 2      => s_pmask <= "1111110000";
                    when 3      => s_pmask <= "1111100000";
                    when 4      => s_pmask <= "1111000000";
                    when 5      => s_pmask <= "1110000000";
                    when 6      => s_pmask <= "1100000000";
                    when others => s_pmask <= "1000000000";
                end case;
                s_thick  <= unsigned(registers_in(5)(9 downto 8));
                s_smooth <= registers_in(6)(0);
                s_inv    <= registers_in(6)(1);
                s_view   <= registers_in(6)(2);
                s_ovl    <= registers_in(6)(3);
                s_gview  <= registers_in(6)(4);
            end if;

            ------------------------------------------------------------------
            -- Sync delay chain
            ------------------------------------------------------------------
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_TOTAL_LATENCY - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_pix;

    ----------------------------------------------------------------------
    -- Luma line buffers: read at lrx (one column ahead of the write),
    -- rotate rows by rewriting each read value one buffer down.
    ----------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            q1 <= lb1(to_integer(lrx));
            if av(4) = '1' then
                lb1(to_integer(lwx)) <= std_logic_vector(a4_y8);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            q2 <= lb2(to_integer(lrx));
            if av(4) = '1' then
                lb2(to_integer(lwx)) <= q1;
            end if;
        end if;
    end process p_lb2;

    p_lb3 : process(clk)
    begin
        if rising_edge(clk) then
            q3 <= lb3(to_integer(lrx));
            if av(4) = '1' then
                lb3(to_integer(lwx)) <= q2;
            end if;
        end if;
    end process p_lb3;

    p_lb4 : process(clk)
    begin
        if rising_edge(clk) then
            q4 <= lb4(to_integer(lrx));
            if av(4) = '1' then
                lb4(to_integer(lwx)) <= q3;
            end if;
        end if;
    end process p_lb4;

    ----------------------------------------------------------------------
    -- Magnitude + direction line buffers (NMS window)
    ----------------------------------------------------------------------
    p_mb1 : process(clk)
    begin
        if rising_edge(clk) then
            mq1 <= mb1(to_integer(mrx));
            if av(15) = '1' then
                mb1(to_integer(mwx)) <= std_logic_vector(a15_mag);
            end if;
        end if;
    end process p_mb1;

    p_mb2 : process(clk)
    begin
        if rising_edge(clk) then
            mq2 <= mb2(to_integer(mrx));
            if av(15) = '1' then
                mb2(to_integer(mwx)) <= std_logic_vector(mq1r);
            end if;
        end if;
    end process p_mb2;

    p_db1 : process(clk)
    begin
        if rising_edge(clk) then
            dq <= db1(to_integer(mrx));
            if av(15) = '1' then
                db1(to_integer(mwx)) <= std_logic_vector(a15_dir);
            end if;
        end if;
    end process p_db1;

    ----------------------------------------------------------------------
    -- Hysteresis accept-bit line buffer
    ----------------------------------------------------------------------
    p_ab1 : process(clk)
    begin
        if rising_edge(clk) then
            aq <= ab1(to_integer(abr));
            if av(17) = '1' then
                ab1(to_integer(abw)) <= (0 => a18_bit);
            end if;
        end if;
    end process p_ab1;

    ----------------------------------------------------------------------
    -- Dilate bit buffer: 2 lines packed (bit 1 = line-1, bit 0 = line-2)
    ----------------------------------------------------------------------
    p_dd1 : process(clk)
    begin
        if rising_edge(clk) then
            ddq <= dd1(to_integer(drx));
            if av(18) = '1' then
                dd1(to_integer(dwx)) <= a18_bit & v1(0);
            end if;
        end if;
    end process p_dd1;

    ----------------------------------------------------------------------
    -- Dry video delay ring: tap 20 clocks (compose) and 21 (mix dry leg)
    ----------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            vring(to_integer(vr_wp)) <= data_in.y & data_in.u & data_in.v;
            vr_q  <= vring(to_integer(vr_wp - 18));
            vr_q2 <= vr_q;
            vr_wp <= vr_wp + 1;
        end if;
    end process p_ring;

    ----------------------------------------------------------------------
    -- Dry/wet mix
    ----------------------------------------------------------------------
    interp_y_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(vr_q2(29 downto 20)),
            b      => a20_y,
            t      => s_mix_t,
            result => s_interp_y_result,
            valid  => s_interp_y_valid
        );

    interp_u_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(vr_q2(19 downto 10)),
            b      => a20_u,
            t      => s_mix_t,
            result => s_interp_u_result,
            valid  => s_interp_u_valid
        );

    interp_v_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(vr_q2(9 downto 0)),
            b      => a20_v,
            t      => s_mix_t,
            result => s_interp_v_result,
            valid  => s_interp_v_valid
        );

    ----------------------------------------------------------------------
    -- Output
    ----------------------------------------------------------------------
    data_out.hsync_n <= s_hsync_sr(C_TOTAL_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_TOTAL_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_TOTAL_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_TOTAL_LATENCY - 1);

    data_out.y <= std_logic_vector(s_interp_y_result);
    data_out.u <= std_logic_vector(s_interp_u_result);
    data_out.v <= std_logic_vector(s_interp_v_result);

end architecture edgelab;
