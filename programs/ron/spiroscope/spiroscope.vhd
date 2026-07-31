-- spiroscope.vhd
--
-- Spiroscope v5: a pure spirograph VECTOR renderer at full frame resolution
-- (up to 1920x1080 -- raster-adaptive via the firmware timing ID).
--
-- No framebuffer (BRAM caps any framebuffer at 320x180 -- never crisp).
-- Instead the curve lives as horizontal INTERVALS bucketed per scanline:
--   * STORE : 1080 rows x 7 slots x 14 bits (x:11 + runwidth:3) = 27 EBR.
--             Empty slot = x field 2047. Run-merging keeps near-horizontal
--             curve sections (lobe tops) from flooding a line's slots.
--   * PEN   : walks the epicycle continuously; the walker merges consecutive
--             same-row pixels into runs (max 8 px) and INSERTS each run.
--   * ERASER: re-runs the exact same curve W substeps behind the pen and
--             REMOVES its runs (exact recomputation => identical intervals).
--             The trailing window W is the animation period (slider = hero).
--   * PAINT (S7): eraser off -> ink accumulates and figures layer.
--   * DISPLAY: each hblank, the next row's slots are stamped into a full-res
--             1-bit dual-bank line buffer (120 words x 16 bit per bank, bank
--             = row parity); the beam reads the other bank. Crisp strokes.
--
-- Interlaced modes (NTSC/PAL/1080i) share the store between fields (curve y
-- is computed in field coordinates with the vertical amplitude halved).
--
-- Exact-erase requires frozen geometry between clears, so K3 (arm) and K4
-- (phase) are 32-zone quantized -- like the discrete pen holes of a real
-- spirograph. Geometry changes (gears/arm/phase/harmonic/kaleido) CLEAR and
-- redraw fresh; in Paint mode figures layer instead. Slider slam = erase.
--
-- Control map:
--   K1 (reg0) Gear A (16 zones)         S7 (reg6.0) Paint (trail) mode
--   K2 (reg1) Gear B (16 zones)         S8 (reg6.1) Harmonic (3rd term)
--   K3 (reg2) Arm (32 zones)            S9 (reg6.2) Kaleidoscope (4-fold)
--   K4 (reg3) Phase (32 zones)          S10(reg6.3) Rainbow (pinwheel)
--   K5 (reg4) Color (8 palette zones)   S11(reg6.4) Background Black/Video
--   K6 (reg5) Draw Speed                Slider(reg7) Trail window (hero)
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture spiroscope of program_top is

    constant C_LATENCY : integer := 3;
    -- 10 slots packed to 30/32 EBR (congestion cliff, seed-exhausted builds);
    -- 8 slots = 5760x14 = ~24 EBR store, 26/32 total, and x8 = pure shift.
    constant C_SLOTS   : integer := 8;    -- crossings kept per line (v5.1: 7->8)
    -- 720 store rows cover: 720p (720), 1080i fields (540), SD (<=576), and
    -- 1080p via ROW PAIRING (bucket = display row >> 1 -> 540 buckets,
    -- 1080i-equivalent vertical resolution on the rare 1080p24/25/30 modes).
    -- Keeps the store at 18 EBRs (20/32 total) -- off the congestion cliff.
    constant C_ROWS    : integer := 720;
    constant C_STORE   : integer := C_ROWS * C_SLOTS;   -- 5760 x 14b = ~24 EBR
    constant C_EMPTYX  : unsigned(10 downto 0) := to_unsigned(2047, 11);
    constant C_LBW     : integer := 120;                -- 1920 px / 16

    -- store address = row*8 + slot (pure shift)
    function f_saddr(row : unsigned(10 downto 0); slot : integer) return unsigned is
        variable v : unsigned(13 downto 0);
    begin
        v := resize(row, 14);
        return shift_left(v, 3) + to_unsigned(slot, 14);
    end function;

    -- ==== Timing / pixel counters ====
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_x_count      : unsigned(11 downto 0) := (others => '0');
    signal s_y_count      : unsigned(11 downto 0) := (others => '0');
    signal s_line_width   : unsigned(11 downto 0) := to_unsigned(1280, 12);
    signal s_frame_height : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_line_active  : std_logic := '0';
    signal s_seen_active  : std_logic := '0';
    signal s_frame_cnt    : unsigned(7 downto 0) := (others => '0');
    signal s_hs_fall      : std_logic := '0';

    signal s_cx_center : unsigned(10 downto 0) := to_unsigned(640, 11);
    signal s_cy_center : unsigned(10 downto 0) := to_unsigned(360, 11);
    signal s_mirw      : unsigned(10 downto 0) := to_unsigned(1279, 11);
    signal s_mirh      : unsigned(10 downto 0) := to_unsigned(719, 11);
    signal s_ilace     : std_logic := '0';
    signal s_ppair     : std_logic := '0';   -- 1080p: pair display rows per bucket
    signal s_rbase     : unsigned(9 downto 0) := to_unsigned(350, 10);

    -- ==== Interval store BRAM (1W1R): 7560 x 14-bit {x[13:3], w[2:0]} ====
    type t_store is array(0 to C_STORE - 1) of std_logic_vector(13 downto 0);
    signal s_store    : t_store := (others => (others => '1'));
    signal s_st_we    : std_logic := '0';
    signal s_st_waddr : unsigned(13 downto 0) := (others => '0');
    signal s_st_wdata : std_logic_vector(13 downto 0) := (others => '1');
    signal s_st_raddr : unsigned(13 downto 0) := (others => '0');
    signal s_st_rdata : std_logic_vector(13 downto 0) := (others => '1');
    signal s_st_rdata_r : std_logic_vector(13 downto 0) := (others => '1');
    signal s_st_raddr_d : unsigned(13 downto 0) := (others => '0');
    signal s_st_raddr_p : unsigned(13 downto 0) := (others => '0');
    signal s_disp_busy  : std_logic := '0';
    signal s_plot_busy  : std_logic := '0';

    -- ==== Line buffer: 2 banks x 120 words x 16 bit ====
    type t_lb is array(0 to C_LBW - 1) of std_logic_vector(15 downto 0);
    signal s_lb0, s_lb1 : t_lb := (others => (others => '0'));
    signal s_lb0_we, s_lb1_we : std_logic := '0';
    signal s_lb0_waddr, s_lb1_waddr : unsigned(6 downto 0) := (others => '0');
    signal s_lb0_wdata, s_lb1_wdata : std_logic_vector(15 downto 0) := (others => '0');
    signal s_lb0_raddr, s_lb1_raddr : unsigned(6 downto 0) := (others => '0');
    signal s_lb0_rdata, s_lb1_rdata : std_logic_vector(15 downto 0) := (others => '0');
    signal s_dr_addr : unsigned(6 downto 0) := (others => '0');

    -- ==== Display scan/stamp FSM ====
    signal s_d_st     : integer range 0 to 9 := 0;
    signal s_d_pt     : std_logic_vector(13 downto 0) := (others => '1');
    signal s_d_row    : unsigned(10 downto 0) := (others => '0');
    signal s_d_bank   : std_logic := '0';
    signal s_d_i      : integer range 0 to 15 := 0;
    signal s_d_b      : integer range 0 to 2 := 0;      -- dilation bank loop
    signal s_d_word   : unsigned(6 downto 0) := (others => '0');
    signal s_d_x      : unsigned(10 downto 0) := (others => '0');
    signal s_d_w      : unsigned(2 downto 0)  := (others => '0');
    signal s_d_m0, s_d_m1 : std_logic_vector(15 downto 0) := (others => '0');
    signal s_d_ones   : unsigned(15 downto 0) := (others => '0');
    signal s_d_spill  : std_logic := '0';
    signal s_d_clrend : unsigned(6 downto 0) := to_unsigned(79, 7);  -- raster-width clear
    -- 3-row interval cache: stamping row R uses intervals of rows R-1/R/R+1
    -- (vertical stroke dilation with no extra store reads -- rotate per hsync)
    type t_pts is array(0 to C_SLOTS - 1) of std_logic_vector(13 downto 0);
    signal s_dp_prev, s_dp_cur, s_dp_next : t_pts := (others => (others => '1'));
    signal s_thick : unsigned(3 downto 0) := to_unsigned(2, 4);      -- K6: 1..8 px
    signal s_dil3  : std_logic := '0';    -- 3-row dilation (thick>=3, HD only)

    -- ==== Beam readout pipeline ====
    signal s_px2  : unsigned(3 downto 0) := (others => '0');
    signal s_av2  : std_logic := '0';
    signal s_bit3 : std_logic := '0';
    signal s_adx2 : unsigned(10 downto 0) := (others => '0');
    signal s_ady2 : unsigned(10 downto 0) := (others => '0');
    signal s_sx2, s_sy2 : std_logic := '0';
    signal s_idx3 : unsigned(3 downto 0) := (others => '0');

    -- ==== Pen / eraser plotter ====
    signal s_pA_h, s_pB_h, s_pC_h : unsigned(17 downto 0) := (others => '0');
    signal s_pA_t, s_pB_t, s_pC_t : unsigned(17 downto 0) := (others => '0');
    signal s_lag  : unsigned(19 downto 0) := (others => '0');
    signal s_tail_due : std_logic := '0';
    signal s_incA : unsigned(11 downto 0) := to_unsigned(643, 12);
    signal s_incB : unsigned(11 downto 0) := to_unsigned(1027, 12);
    signal s_incC : unsigned(12 downto 0) := to_unsigned(1670, 13);
    signal s_R1   : unsigned(9 downto 0)  := to_unsigned(222, 10);
    signal s_R2   : unsigned(9 downto 0)  := to_unsigned(128, 10);
    signal s_R3   : unsigned(9 downto 0)  := (others => '0');
    -- arm radii pipeline: base latched at vsync (shallow); the zone*base/32
    -- multiply and the R1 subtract free-run in their own register stages
    signal s_base_r : unsigned(9 downto 0) := to_unsigned(350, 10);
    signal s_r2p    : unsigned(9 downto 0) := (others => '0');
    signal s_phaseOff : unsigned(9 downto 0) := (others => '0');
    signal s_budget   : unsigned(12 downto 0) := (others => '0');
    signal s_window   : unsigned(19 downto 0) := to_unsigned(512, 20);
    signal s_paint    : std_logic := '0';
    signal s_role_t   : std_logic := '0';

    signal s_st : integer range 0 to 31 := 0;

    signal s_sinA, s_cosA, s_sinB, s_cosB : signed(9 downto 0) := (others => '0');
    signal s_sinC, s_cosC : signed(9 downto 0) := (others => '0');
    signal s_accX, s_accY : signed(20 downto 0) := (others => '0');
    signal s_npx : unsigned(10 downto 0) := (others => '0');
    signal s_npy : unsigned(10 downto 0) := (others => '0');
    signal s_dxa : unsigned(10 downto 0) := (others => '0');
    signal s_dya : unsigned(10 downto 0) := (others => '0');

    signal s_lut_angle : std_logic_vector(9 downto 0) := (others => '0');
    signal s_lut_sin   : signed(9 downto 0);
    signal s_lut_cos   : signed(9 downto 0);
    signal s_mul_a     : signed(10 downto 0) := (others => '0');
    signal s_mul_b     : signed(9 downto 0)  := (others => '0');
    signal s_mul_p     : signed(20 downto 0) := (others => '0');

    -- walker + per-role run merging
    signal s_hpx, s_tpx : unsigned(10 downto 0) := (others => '0');
    signal s_hpy, s_tpy : unsigned(10 downto 0) := (others => '0');
    signal s_hvalid, s_tvalid : std_logic := '0';
    signal s_wx, s_tx : unsigned(10 downto 0) := (others => '0');
    signal s_wy, s_ty : unsigned(10 downto 0) := (others => '0');
    signal s_hrx, s_trx : unsigned(10 downto 0) := (others => '0');
    signal s_hry, s_try : unsigned(10 downto 0) := (others => '0');
    signal s_hrw, s_trw : unsigned(2 downto 0)  := (others => '0');
    signal s_hrv, s_trv : std_logic := '0';

    -- store-op engine
    signal s_op_erase : std_logic := '0';
    signal s_op_mstep : integer range 0 to 3 := 0;
    signal s_op_x  : unsigned(10 downto 0) := (others => '0');
    signal s_op_y  : unsigned(10 downto 0) := (others => '0');
    signal s_op_w  : unsigned(2 downto 0)  := (others => '0');
    signal s_mx    : unsigned(10 downto 0) := (others => '0');
    signal s_my    : unsigned(10 downto 0) := (others => '0');
    signal s_sc_i    : integer range 0 to 15 := 0;
    signal s_sc_hit  : integer range 0 to C_SLOTS - 1 := 0;
    signal s_sc_have : std_logic := '0';
    signal s_sc_skip : std_logic := '0';

    -- registered extend-or-emit flags (split off the run-update muxes)
    signal s_f_extr, s_f_extl : std_logic := '0';

    signal s_clear_req : std_logic := '0';
    signal s_clr_k     : unsigned(13 downto 0) := (others => '0');

    signal s_zA_l, s_zB_l : unsigned(3 downto 0) := "0100";
    signal s_z3_l, s_z4_l : unsigned(4 downto 0) := (others => '0');
    signal s_s8_l, s_s9_l : std_logic := '0';

    signal s_rainbow   : std_logic := '0';
    signal s_bg_video  : std_logic := '0';
    signal s_kaleido   : std_logic := '0';
    signal s_color_sel : unsigned(2 downto 0) := to_unsigned(3, 3);

    -- ==== Palettes (U/V swapped for HW: stored U = Cr, stored V = Cb) ====
    type t_pal8 is array(0 to 7) of unsigned(9 downto 0);
    constant C_PAL_U : t_pal8 := (
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10), to_unsigned(136, 10),
        to_unsigned(440, 10), to_unsigned(608, 10), to_unsigned(756, 10), to_unsigned(512, 10));
    constant C_PAL_V : t_pal8 := (
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10), to_unsigned(216, 10),
        to_unsigned(960, 10), to_unsigned(696, 10), to_unsigned(852, 10), to_unsigned(512, 10));
    type t_pal16 is array(0 to 15) of unsigned(9 downto 0);
    constant C_WHL_U : t_pal16 := (
        to_unsigned(512, 10), to_unsigned(596, 10), to_unsigned(668, 10), to_unsigned(715, 10),
        to_unsigned(732, 10), to_unsigned(715, 10), to_unsigned(668, 10), to_unsigned(596, 10),
        to_unsigned(512, 10), to_unsigned(428, 10), to_unsigned(356, 10), to_unsigned(309, 10),
        to_unsigned(292, 10), to_unsigned(309, 10), to_unsigned(356, 10), to_unsigned(428, 10));
    constant C_WHL_V : t_pal16 := (
        to_unsigned(732, 10), to_unsigned(715, 10), to_unsigned(668, 10), to_unsigned(596, 10),
        to_unsigned(512, 10), to_unsigned(428, 10), to_unsigned(356, 10), to_unsigned(309, 10),
        to_unsigned(292, 10), to_unsigned(309, 10), to_unsigned(356, 10), to_unsigned(428, 10),
        to_unsigned(512, 10), to_unsigned(596, 10), to_unsigned(668, 10), to_unsigned(715, 10));

    signal s_d_y   : unsigned(9 downto 0) := (others => '0');
    signal s_d_u   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_d_v   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_drawn : std_logic := '0';

    type t_data_sr is array(0 to C_LATENCY - 1) of std_logic_vector(9 downto 0);
    type t_bit_sr  is array(0 to C_LATENCY - 1) of std_logic;
    signal s_y_sr, s_u_sr, s_v_sr : t_data_sr := (others => (others => '0'));
    signal s_hsync_sr, s_vsync_sr, s_field_sr, s_avid_sr : t_bit_sr := (others => '0');

begin

    -- ====================================================================
    -- counters + per-frame geometry (radius/interlace from the timing ID)
    -- ====================================================================
    p_count : process(clk)
        variable v_tid : std_logic_vector(3 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_hs_fall <= '0';

            if data_in.avid = '1' then
                s_x_count     <= s_x_count + 1;
                s_line_active <= '1';
                s_seen_active <= '1';
            end if;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_x_count > 0 then s_line_width <= s_x_count; end if;
                s_x_count <= (others => '0');
                s_hs_fall <= '1';
                if s_line_active = '1' then
                    s_y_count     <= s_y_count + 1;
                    s_line_active <= '0';
                end if;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_y_count > 0 then s_frame_height <= s_y_count; end if;
                s_y_count     <= (others => '0');
                s_x_count     <= (others => '0');
                s_line_active <= '0';
                s_seen_active <= '0';
                s_frame_cnt   <= s_frame_cnt + 1;
                s_rainbow   <= registers_in(6)(3);
                s_bg_video  <= registers_in(6)(4);
                s_color_sel <= unsigned(registers_in(4)(9 downto 7));
                -- K6 = line thickness (1..8 px); 3-row dilation on HD rasters
                s_thick <= resize(unsigned(registers_in(5)(9 downto 7)), 4) + 1;
                if unsigned(registers_in(5)(9 downto 7)) >= 2
                   and s_line_width >= to_unsigned(900, 12) then
                    s_dil3 <= '1';
                else
                    s_dil3 <= '0';
                end if;
                s_d_clrend  <= resize(shift_right(s_line_width - 1, 4), 7);
                s_cx_center <= resize(shift_right(s_line_width, 1), 11);
                s_cy_center <= resize(shift_right(s_frame_height, 1), 11);
                s_mirw      <= resize(s_line_width - 1, 11);
                s_mirh      <= resize(s_frame_height - 1, 11);
                -- radius + interlace from the firmware timing ID
                v_tid := registers_in(8)(3 downto 0);
                if    v_tid = C_720P60 or v_tid = C_720P5994 or v_tid = C_720P50 then
                    s_rbase <= to_unsigned(350, 10);  s_ilace <= '0';  s_ppair <= '0';
                elsif v_tid = C_1080I60 or v_tid = C_1080I5994 or v_tid = C_1080I50 then
                    s_rbase <= to_unsigned(500, 10);  s_ilace <= '1';  s_ppair <= '0';
                elsif v_tid = C_NTSC then
                    s_rbase <= to_unsigned(220, 10);  s_ilace <= '1';  s_ppair <= '0';
                elsif v_tid = C_PAL then
                    s_rbase <= to_unsigned(260, 10);  s_ilace <= '1';  s_ppair <= '0';
                elsif v_tid = C_480P then
                    s_rbase <= to_unsigned(220, 10);  s_ilace <= '0';  s_ppair <= '0';
                elsif v_tid = C_576P then
                    s_rbase <= to_unsigned(260, 10);  s_ilace <= '0';  s_ppair <= '0';
                else                                             -- 1080p family
                    s_rbase <= to_unsigned(500, 10);  s_ilace <= '0';  s_ppair <= '1';
                end if;
            end if;
        end if;
    end process p_count;

    -- ====================================================================
    -- Interval store BRAM (1W1R); read data registered (27-EBR output mux)
    -- ====================================================================
    s_st_raddr <= s_st_raddr_d when s_disp_busy = '1' else s_st_raddr_p;

    p_store : process(clk)
    begin
        if rising_edge(clk) then
            if s_st_we = '1' then
                s_store(to_integer(s_st_waddr)) <= s_st_wdata;
            end if;
            s_st_rdata   <= s_store(to_integer(s_st_raddr));
            s_st_rdata_r <= s_st_rdata;
        end if;
    end process p_store;

    -- ==== line buffer banks ====
    s_lb0_raddr <= s_dr_addr when (s_d_bank = '0' and s_d_st /= 0)
                   else resize(s_x_count(10 downto 4), 7);
    s_lb1_raddr <= s_dr_addr when (s_d_bank = '1' and s_d_st /= 0)
                   else resize(s_x_count(10 downto 4), 7);

    p_lb0 : process(clk)
    begin
        if rising_edge(clk) then
            if s_lb0_we = '1' then
                s_lb0(to_integer(s_lb0_waddr)) <= s_lb0_wdata;
            end if;
            s_lb0_rdata <= s_lb0(to_integer(s_lb0_raddr));
        end if;
    end process p_lb0;

    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            if s_lb1_we = '1' then
                s_lb1(to_integer(s_lb1_waddr)) <= s_lb1_wdata;
            end if;
            s_lb1_rdata <= s_lb1(to_integer(s_lb1_raddr));
        end if;
    end process p_lb1;

    -- ====================================================================
    -- Display scan/stamp FSM (per hsync, all in hblank)
    -- ====================================================================
    p_disp : process(clk)
        variable v_row  : unsigned(10 downto 0);
        variable v_pt   : std_logic_vector(13 downto 0);
        variable v_x    : unsigned(10 downto 0);
        variable v_len  : integer range 0 to 31;
        variable v_ones : unsigned(15 downto 0);
        variable v_m31  : unsigned(30 downto 0);
        variable v_msk  : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            s_lb0_we <= '0';
            s_lb1_we <= '0';

            case s_d_st is
                when 0 =>
                    if s_hs_fall = '1' then
                        if s_seen_active = '0' then
                            v_row := (others => '0');
                        else
                            v_row := resize(s_y_count + 1, 11);
                        end if;
                        s_d_bank <= v_row(0);          -- bank = DISPLAY row parity
                        if s_ppair = '1' then
                            v_row := shift_right(v_row, 1);   -- 1080p bucket pairing
                        end if;
                        if v_row < to_unsigned(C_ROWS, 11) then
                            s_d_row <= v_row;
                            -- rotate the 3-row interval cache
                            s_dp_prev <= s_dp_cur;
                            s_dp_cur  <= s_dp_next;
                            s_d_st  <= 1;
                        end if;
                    end if;
                when 1 =>
                    if s_plot_busy = '0' then
                        s_disp_busy <= '1';
                        s_d_word <= (others => '0');
                        s_d_i    <= 0;
                        s_d_st   <= 2;
                    end if;
                when 2 =>
                    -- clear the stamp bank up to the raster width (a fixed
                    -- 120-word clear overran SD's short hblank) while the
                    -- target row's slots stream into the "next" cache bank
                    if s_d_bank = '0' then
                        s_lb0_we <= '1'; s_lb0_waddr <= s_d_word; s_lb0_wdata <= (others => '0');
                    else
                        s_lb1_we <= '1'; s_lb1_waddr <= s_d_word; s_lb1_wdata <= (others => '0');
                    end if;
                    if s_d_i < C_SLOTS then
                        s_st_raddr_d <= f_saddr(s_d_row, s_d_i);
                    end if;
                    if s_d_i >= 3 and s_d_i <= C_SLOTS + 2 then
                        s_dp_next(s_d_i - 3) <= s_st_rdata_r;
                    end if;
                    if s_d_i < 15 then s_d_i <= s_d_i + 1; end if;
                    if s_d_word >= s_d_clrend and s_d_i > C_SLOTS + 2 then
                        s_disp_busy <= '0';
                        s_d_i  <= 0;
                        s_d_b  <= 0;
                        s_d_st <= 4;
                    elsif s_d_word < to_unsigned(C_LBW - 1, 7) then
                        s_d_word <= s_d_word + 1;
                    end if;
                when 4 =>
                    -- select interval into a register (bank 0 = current row;
                    -- 1/2 = neighbours when 3-row dilation is active);
                    -- validity check pipelined into state 9
                    if s_d_i = C_SLOTS then
                        if s_d_b = 2 or s_dil3 = '0' then
                            s_d_st <= 0;
                        else
                            s_d_b <= s_d_b + 1;
                            s_d_i <= 0;
                        end if;
                    else
                        case s_d_b is
                            when 0      => s_d_pt <= s_dp_cur(s_d_i);
                            when 1      => s_d_pt <= s_dp_prev(s_d_i);
                            when others => s_d_pt <= s_dp_next(s_d_i);
                        end case;
                        s_d_st <= 9;
                    end if;
                when 9 =>
                    v_x := unsigned(s_d_pt(13 downto 3));
                    if v_x < to_unsigned(1920, 11) then
                        s_d_x <= v_x;
                        s_d_w <= unsigned(s_d_pt(2 downto 0));
                        s_dr_addr <= resize(v_x(10 downto 4), 7);
                        s_d_st <= 8;
                    else
                        s_d_i <= s_d_i + 1;
                        s_d_st <= 4;
                    end if;
                when 8 =>
                    -- stamp mask part 1: ones run of (width + thickness) px
                    -- (split from the position shift -- two chained barrels
                    -- in one state were the v5.1 critical path)
                    v_len := to_integer(s_d_w) + to_integer(s_thick);
                    if v_len > 15 then v_len := 15; end if;
                    if v_len < 1 then v_len := 1; end if;
                    s_d_ones <= shift_left(to_unsigned(1, 16), v_len) - 1;
                    s_d_st <= 5;
                when 5 =>
                    -- stamp mask part 2: shift to bit position + spill split
                    v_m31 := shift_left(resize(s_d_ones, 31), to_integer(s_d_x(3 downto 0)));
                    s_d_m0 <= std_logic_vector(v_m31(15 downto 0));
                    s_d_m1 <= '0' & std_logic_vector(v_m31(30 downto 16));
                    if v_m31(30 downto 16) /= 0 then s_d_spill <= '1'; else s_d_spill <= '0'; end if;
                    s_d_st <= 6;
                when 6 =>
                    if s_d_bank = '0' then v_msk := s_lb0_rdata or s_d_m0;
                    else                   v_msk := s_lb1_rdata or s_d_m0; end if;
                    if s_d_bank = '0' then
                        s_lb0_we <= '1'; s_lb0_waddr <= resize(s_d_x(10 downto 4), 7); s_lb0_wdata <= v_msk;
                    else
                        s_lb1_we <= '1'; s_lb1_waddr <= resize(s_d_x(10 downto 4), 7); s_lb1_wdata <= v_msk;
                    end if;
                    if s_d_spill = '1' and s_d_x(10 downto 4) /= to_unsigned(C_LBW - 1, 7) then
                        s_dr_addr <= resize(s_d_x(10 downto 4), 7) + 1;
                        s_d_st <= 7;
                    else
                        s_d_i  <= s_d_i + 1;
                        s_d_st <= 4;
                    end if;
                when 7 =>
                    s_d_st <= 3;
                when 3 =>
                    if s_d_bank = '0' then v_msk := s_lb0_rdata or s_d_m1;
                    else                   v_msk := s_lb1_rdata or s_d_m1; end if;
                    if s_d_bank = '0' then
                        s_lb0_we <= '1'; s_lb0_waddr <= resize(s_d_x(10 downto 4), 7) + 1; s_lb0_wdata <= v_msk;
                    else
                        s_lb1_we <= '1'; s_lb1_waddr <= resize(s_d_x(10 downto 4), 7) + 1; s_lb1_wdata <= v_msk;
                    end if;
                    s_d_i  <= s_d_i + 1;
                    s_d_st <= 4;
            end case;
        end if;
    end process p_disp;

    -- ====================================================================
    -- Beam readout (C_LATENCY = 3)
    -- ====================================================================
    p_beam : process(clk)
        variable v_dx  : signed(11 downto 0);
        variable v_dy  : signed(11 downto 0);
        variable v_oct : unsigned(2 downto 0);
        variable v_m   : boolean;
        variable v_w   : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            s_px2 <= s_x_count(3 downto 0);
            s_av2 <= data_in.avid;
            v_dx := signed(resize(s_x_count, 12)) - signed(resize(s_cx_center, 12));
            v_dy := signed(resize(s_y_count, 12)) - signed(resize(s_cy_center, 12));
            if v_dx >= 0 then s_sx2 <= '1'; s_adx2 <= resize(unsigned(v_dx), 11);
            else              s_sx2 <= '0'; s_adx2 <= resize(unsigned(-v_dx), 11); end if;
            if v_dy >= 0 then s_sy2 <= '1'; s_ady2 <= resize(unsigned(v_dy), 11);
            else              s_sy2 <= '0'; s_ady2 <= resize(unsigned(-v_dy), 11); end if;

            if s_y_count(0) = '0' then v_w := s_lb0_rdata;
            else                       v_w := s_lb1_rdata; end if;
            if s_av2 = '1' and v_w(to_integer(s_px2)) = '1' then
                s_bit3 <= '1';
            else
                s_bit3 <= '0';
            end if;

            v_m := s_ady2 > s_adx2;
            if    s_sx2 = '1' and s_sy2 = '1' and not v_m then v_oct := "000";
            elsif s_sx2 = '1' and s_sy2 = '1' and     v_m then v_oct := "001";
            elsif s_sx2 = '0' and s_sy2 = '1' and     v_m then v_oct := "010";
            elsif s_sx2 = '0' and s_sy2 = '1' and not v_m then v_oct := "011";
            elsif s_sx2 = '0' and s_sy2 = '0' and not v_m then v_oct := "100";
            elsif s_sx2 = '0' and s_sy2 = '0' and     v_m then v_oct := "101";
            elsif s_sx2 = '1' and s_sy2 = '0' and     v_m then v_oct := "110";
            else                                               v_oct := "111";
            end if;
            s_idx3 <= (v_oct & '0')
                      + resize(s_adx2(6 downto 6), 4)
                      + resize(s_ady2(6 downto 6), 4)
                      + resize(s_frame_cnt(7 downto 4), 4);

            if s_bit3 = '1' then
                s_d_y <= to_unsigned(1023, 10);
                if s_rainbow = '1' then
                    s_d_u <= C_WHL_U(to_integer(s_idx3));
                    s_d_v <= C_WHL_V(to_integer(s_idx3));
                else
                    s_d_u <= C_PAL_U(to_integer(s_color_sel));
                    s_d_v <= C_PAL_V(to_integer(s_color_sel));
                end if;
                s_drawn <= '1';
            else
                s_d_y   <= (others => '0');
                s_d_u   <= to_unsigned(512, 10);
                s_d_v   <= to_unsigned(512, 10);
                s_drawn <= '0';
            end if;
        end if;
    end process p_beam;

    -- ====================================================================
    -- Pen / eraser plotter
    -- ====================================================================
    lut : entity work.sin_cos_full_lut_10x10
        port map (angle_in => s_lut_angle, sin_out => s_lut_sin, cos_out => s_lut_cos);

    p_mul : process(clk)
    begin
        if rising_edge(clk) then
            s_mul_p <= s_mul_a * s_mul_b;
        end if;
    end process p_mul;

    p_plot : process(clk)
        variable v_zA, v_zB : unsigned(3 downto 0);
        variable v_z3, v_z4 : unsigned(4 downto 0);
        variable v_ia, v_ib : unsigned(11 downto 0);
        variable v_sh   : integer range 0 to 2;
        variable v_sum  : unsigned(4 downto 0);
        variable v_base : unsigned(9 downto 0);
        variable v_r2   : unsigned(9 downto 0);
        variable v_sl   : unsigned(9 downto 0);
        variable v_pnew : std_logic;
        variable v_clr  : boolean;
        variable v_pxs  : signed(11 downto 0);
        variable v_pys  : signed(11 downto 0);
        variable v_nx   : unsigned(10 downto 0);
        variable v_ny   : unsigned(10 downto 0);
        variable v_accy : signed(20 downto 0);
        variable v_pA, v_pB, v_pC : unsigned(17 downto 0);
        variable v_hit  : boolean;
        variable v_tele : boolean;
        variable v_rx   : unsigned(10 downto 0);
        variable v_ry   : unsigned(10 downto 0);
        variable v_rw   : unsigned(2 downto 0);
        variable v_rv   : std_logic;
        variable v_emit : boolean;
        variable v_rdx  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_st_we <= '0';

            if s_lag > s_window then s_tail_due <= '1'; else s_tail_due <= '0'; end if;

            -- free-running arm split: r2 = zone * (base/32); R1 = base - r2.
            -- (Settles a few clocks after vsync; geometry changes clear the
            -- store anyway, so the brief transient never reaches the screen.)
            s_r2p <= resize(s_z3_l * resize(shift_right(s_base_r, 5), 5), 10);
            s_R2  <= s_r2p;
            s_R1  <= s_base_r - s_r2p;

            -- ---- per-frame latch, zone params, clear triggers ----
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                v_zA := unsigned(registers_in(0)(9 downto 6));
                v_zB := unsigned(registers_in(1)(9 downto 6));
                v_z3 := unsigned(registers_in(2)(9 downto 5));
                v_z4 := unsigned(registers_in(3)(9 downto 5));
                v_sl := unsigned(registers_in(7));
                v_pnew := registers_in(6)(0);

                v_sum := resize(v_zA, 5) + resize(v_zB, 5);
                if    v_sum <= 6  then v_sh := 0;
                elsif v_sum <= 14 then v_sh := 1;
                else                   v_sh := 2;
                end if;
                v_ia := shift_left(resize(v_zA, 12) + 1, 7) + to_unsigned(3, 12);
                v_ib := shift_left(resize(v_zB, 12) + 1, 7) + to_unsigned(3, 12);
                s_incA <= shift_right(v_ia, v_sh);
                s_incB <= shift_right(v_ib, v_sh);
                s_incC <= shift_right(resize(v_ia, 13) + resize(v_ib, 13), v_sh);

                -- harmonic takes a quarter of the base radius; the arm split
                -- itself free-runs below (multiply cone off the vsync latch)
                if registers_in(6)(1) = '1' then
                    s_base_r <= s_rbase - resize(shift_right(s_rbase, 2), 10);
                    s_R3     <= resize(shift_right(s_rbase, 2), 10);
                else
                    s_base_r <= s_rbase;
                    s_R3     <= (others => '0');
                end if;
                s_phaseOff <= resize(v_z4, 10) sll 5;

                -- fixed draw rate (K6 is now line thickness); kaleido quarters
                -- the substep budget to bound store-op clocks
                if registers_in(6)(2) = '1' then
                    s_budget <= shift_left(to_unsigned(96, 13), v_sh);
                else
                    s_budget <= shift_left(to_unsigned(384, 13), v_sh);
                end if;
                s_window <= resize(v_sl & '0', 20) + to_unsigned(64, 20);
                s_paint   <= v_pnew;
                s_kaleido <= registers_in(6)(2);

                v_clr := false;
                if v_sl > to_unsigned(1010, 10) then v_clr := true; end if;
                if s_paint = '1' and v_pnew = '0' then v_clr := true; end if;
                if v_pnew = '0' then
                    if v_zA /= s_zA_l or v_zB /= s_zB_l then v_clr := true; end if;
                    if v_z3 /= s_z3_l or v_z4 /= s_z4_l then v_clr := true; end if;
                    if registers_in(6)(1) /= s_s8_l then v_clr := true; end if;
                    if registers_in(6)(2) /= s_s9_l then v_clr := true; end if;
                end if;
                s_zA_l <= v_zA;  s_zB_l <= v_zB;
                s_z3_l <= v_z3;  s_z4_l <= v_z4;
                s_s8_l <= registers_in(6)(1);
                s_s9_l <= registers_in(6)(2);
                if v_clr then s_clear_req <= '1'; end if;
            end if;

            -- ---- plotter FSM ----
            case s_st is
                when 0 =>
                    s_plot_busy <= '0';
                    if s_clear_req = '1' then
                        s_clr_k <= (others => '0');
                        s_st <= 30;
                    elsif s_paint = '0' and s_tail_due = '1' then
                        s_role_t <= '1';
                        s_st <= 1;
                    elsif s_budget /= 0 then
                        s_role_t <= '0';
                        s_budget <= s_budget - 1;
                        s_st <= 1;
                    end if;

                -- ---- curve evaluation (8-bit LUT angle, 2-cycle reads) ----
                when 1 =>
                    if s_role_t = '1' then v_pA := s_pA_t; else v_pA := s_pA_h; end if;
                    s_lut_angle <= std_logic_vector(v_pA(17 downto 10)) & "00";
                    s_st <= 2;
                when 2 =>
                    s_st <= 3;
                when 3 =>
                    s_sinA <= s_lut_sin;  s_cosA <= s_lut_cos;
                    if s_role_t = '1' then v_pB := s_pB_t; else v_pB := s_pB_h; end if;
                    s_lut_angle <= std_logic_vector(unsigned(v_pB(17 downto 10))
                                                    + s_phaseOff(9 downto 2)) & "00";
                    s_st <= 4;
                when 4 =>
                    s_st <= 5;
                when 5 =>
                    s_sinB <= s_lut_sin;  s_cosB <= s_lut_cos;
                    if s_role_t = '1' then v_pC := s_pC_t; else v_pC := s_pC_h; end if;
                    s_lut_angle <= std_logic_vector(v_pC(17 downto 10)) & "00";
                    s_st <= 6;
                when 6 =>
                    s_mul_a <= signed('0' & s_R1);  s_mul_b <= s_cosA;
                    s_st <= 7;
                when 7 =>
                    s_sinC <= s_lut_sin;  s_cosC <= s_lut_cos;
                    s_mul_a <= signed('0' & s_R2);  s_mul_b <= s_cosB;
                    s_st <= 8;
                when 8 =>
                    s_accX <= resize(s_mul_p, 21);
                    s_mul_a <= signed('0' & s_R3);  s_mul_b <= s_cosC;
                    s_st <= 9;
                when 9 =>
                    s_accX <= s_accX + resize(s_mul_p, 21);
                    s_mul_a <= signed('0' & s_R1);  s_mul_b <= s_sinA;
                    s_st <= 10;
                when 10 =>
                    s_accX <= s_accX + resize(s_mul_p, 21);
                    s_mul_a <= signed('0' & s_R2);  s_mul_b <= s_sinB;
                    s_st <= 11;
                when 11 =>
                    s_accY <= resize(s_mul_p, 21);
                    s_mul_a <= signed('0' & s_R3);  s_mul_b <= s_sinC;
                    s_st <= 12;
                when 12 =>
                    s_accY <= s_accY + resize(s_mul_p, 21);
                    s_st <= 13;
                when 13 =>
                    -- position calc (interlaced fields halve the y amplitude)
                    v_accy := s_accY + resize(s_mul_p, 21);
                    v_pxs := resize(signed('0' & s_cx_center), 12) + resize(shift_right(s_accX, 9), 12);
                    if s_ilace = '1' then
                        v_pys := resize(signed('0' & s_cy_center), 12) + resize(shift_right(v_accy, 10), 12);
                    else
                        v_pys := resize(signed('0' & s_cy_center), 12) + resize(shift_right(v_accy, 9), 12);
                    end if;
                    if    v_pxs < 0 then s_npx <= (others => '0');
                    elsif v_pxs > signed(resize(s_mirw, 12)) then s_npx <= s_mirw;
                    else  s_npx <= unsigned(v_pxs(10 downto 0)); end if;
                    if    v_pys < 0 then s_npy <= (others => '0');
                    elsif v_pys > signed(resize(s_mirh, 12)) then s_npy <= s_mirh;
                    else  s_npy <= unsigned(v_pys(10 downto 0)); end if;
                    s_st <= 14;
                when 14 =>
                    -- distance registers (split off the teleport decision)
                    if s_role_t = '1' then
                        if s_tpx > s_npx then s_dxa <= s_tpx - s_npx; else s_dxa <= s_npx - s_tpx; end if;
                        if s_tpy > s_npy then s_dya <= s_tpy - s_npy; else s_dya <= s_npy - s_tpy; end if;
                    else
                        if s_hpx > s_npx then s_dxa <= s_hpx - s_npx; else s_dxa <= s_npx - s_hpx; end if;
                        if s_hpy > s_npy then s_dya <= s_hpy - s_npy; else s_dya <= s_npy - s_hpy; end if;
                    end if;
                    s_st <= 15;
                when 15 =>
                    -- teleport decision + phase/lag bookkeeping + walker init
                    s_tx <= s_npx;
                    s_ty <= s_npy;
                    if s_role_t = '1' then
                        v_tele := (s_tvalid = '0') or (s_dxa > 48) or (s_dya > 48);
                        if v_tele then
                            s_wx <= s_npx; s_wy <= s_npy;
                            s_trx <= s_npx; s_try <= s_npy; s_trw <= "000"; s_trv <= '0';
                        else
                            s_wx <= s_tpx; s_wy <= s_tpy;
                        end if;
                        s_tpx <= s_npx;  s_tpy <= s_npy;  s_tvalid <= '1';
                        s_pA_t <= s_pA_t + resize(s_incA, 18);
                        s_pB_t <= s_pB_t + resize(s_incB, 18);
                        s_pC_t <= s_pC_t + resize(s_incC, 18);
                        s_lag  <= s_lag - 1;
                        s_op_erase <= '1';
                    else
                        v_tele := (s_hvalid = '0') or (s_dxa > 48) or (s_dya > 48);
                        if v_tele then
                            s_wx <= s_npx; s_wy <= s_npy;
                            s_hrx <= s_npx; s_hry <= s_npy; s_hrw <= "000"; s_hrv <= '0';
                        else
                            s_wx <= s_hpx; s_wy <= s_hpy;
                        end if;
                        s_hpx <= s_npx;  s_hpy <= s_npy;  s_hvalid <= '1';
                        s_pA_h <= s_pA_h + resize(s_incA, 18);
                        s_pB_h <= s_pB_h + resize(s_incB, 18);
                        s_pC_h <= s_pC_h + resize(s_incC, 18);
                        s_lag  <= s_lag + 1;
                        s_op_erase <= '0';
                    end if;
                    if v_tele then s_st <= 0; else s_st <= 16; end if;

                -- ---- walker: step, then extend the role's run or emit it ----
                when 16 =>
                    if s_wx = s_tx and s_wy = s_ty then
                        s_st <= 0;
                    else
                        if s_wx < s_tx then s_wx <= s_wx + 1;
                        elsif s_wx > s_tx then s_wx <= s_wx - 1; end if;
                        if s_wy < s_ty then s_wy <= s_wy + 1;
                        elsif s_wy > s_ty then s_wy <= s_wy - 1; end if;
                        s_st <= 22;
                    end if;
                when 22 =>
                    -- register the extend comparisons (was part of state 17's
                    -- cone -- adder+compare feeding the run-update muxes was
                    -- the ~75 MHz marginal path)
                    if s_role_t = '1' then
                        v_rx := s_trx; v_ry := s_try; v_rw := s_trw;
                    else
                        v_rx := s_hrx; v_ry := s_hry; v_rw := s_hrw;
                    end if;
                    if s_wy = v_ry and v_rw /= "111"
                       and s_wx = v_rx + resize(v_rw, 11) + 1 then
                        s_f_extr <= '1';
                    else
                        s_f_extr <= '0';
                    end if;
                    if s_wy = v_ry and v_rw /= "111"
                       and s_wx + 1 = v_rx then
                        s_f_extl <= '1';
                    else
                        s_f_extl <= '0';
                    end if;
                    s_st <= 17;
                when 17 =>
                    if s_role_t = '1' then
                        v_rx := s_trx; v_ry := s_try; v_rw := s_trw; v_rv := s_trv;
                    else
                        v_rx := s_hrx; v_ry := s_hry; v_rw := s_hrw; v_rv := s_hrv;
                    end if;
                    v_emit := false;
                    if v_rv = '0' then
                        v_rx := s_wx; v_ry := s_wy; v_rw := "000"; v_rv := '1';
                    elsif s_f_extr = '1' then
                        v_rw := v_rw + 1;
                    elsif s_f_extl = '1' then
                        v_rx := s_wx;  v_rw := v_rw + 1;
                    else
                        v_emit := true;
                    end if;
                    if v_emit then
                        s_op_x <= v_rx;  s_op_y <= v_ry;  s_op_w <= v_rw;
                        s_op_mstep <= 0;
                        v_rx := s_wx; v_ry := s_wy; v_rw := "000"; v_rv := '1';
                        s_st <= 18;
                    else
                        s_st <= 16;
                    end if;
                    if s_role_t = '1' then
                        s_trx <= v_rx; s_try <= v_ry; s_trw <= v_rw; s_trv <= v_rv;
                    else
                        s_hrx <= v_rx; s_hry <= v_ry; s_hrw <= v_rw; s_hrv <= v_rv;
                    end if;

                -- ---- store-op: mirror loop; pipelined scan; write ----
                when 18 =>
                    -- mirror setup; bucket row = display row (>>1 for 1080p)
                    case s_op_mstep is
                        when 0 => s_mx <= s_op_x;
                                  v_ry := s_op_y;
                        when 1 => s_mx <= s_mirw - s_op_x - resize(s_op_w, 11) - 1;
                                  v_ry := s_op_y;
                        when 2 => s_mx <= s_op_x;
                                  v_ry := s_mirh - s_op_y;
                        when others => s_mx <= s_mirw - s_op_x - resize(s_op_w, 11) - 1;
                                  v_ry := s_mirh - s_op_y;
                    end case;
                    if s_ppair = '1' then
                        s_my <= shift_right(v_ry, 1);
                    else
                        s_my <= v_ry;
                    end if;
                    s_st <= 19;
                when 19 =>
                    if s_disp_busy = '0' then
                        s_plot_busy <= '1';
                        s_sc_i    <= 0;
                        s_sc_have <= '0';
                        s_sc_skip <= '0';
                        s_st <= 20;
                    end if;
                when 20 =>
                    if s_disp_busy = '1' then
                        s_plot_busy <= '0';
                        s_st <= 19;
                    else
                        if s_sc_i < C_SLOTS then
                            s_st_raddr_p <= f_saddr(s_my, s_sc_i);
                        end if;
                        if s_sc_i >= 3 and s_sc_i <= C_SLOTS + 2 then
                            v_hit := false;
                            v_rdx := unsigned(s_st_rdata_r(13 downto 3));
                            if s_op_erase = '1' then
                                if v_rdx = s_mx then v_hit := true; end if;
                            else
                                if v_rdx = C_EMPTYX then v_hit := true; end if;
                                if s_paint = '1' and v_rdx /= C_EMPTYX then
                                    if (v_rdx <= s_mx and s_mx - v_rdx <= 2)
                                    or (v_rdx > s_mx and v_rdx - s_mx <= 2) then
                                        s_sc_skip <= '1';
                                    end if;
                                end if;
                            end if;
                            if v_hit and s_sc_have = '0' then
                                s_sc_have <= '1';
                                s_sc_hit  <= s_sc_i - 3;
                            end if;
                        end if;
                        if s_sc_i = C_SLOTS + 3 then
                            s_st <= 21;
                        else
                            s_sc_i <= s_sc_i + 1;
                        end if;
                    end if;
                when 21 =>
                    if s_sc_have = '1' and s_sc_skip = '0' then
                        s_st_we    <= '1';
                        s_st_waddr <= f_saddr(s_my, s_sc_hit);
                        if s_op_erase = '1' then
                            s_st_wdata <= (others => '1');
                        else
                            s_st_wdata <= std_logic_vector(s_mx) & std_logic_vector(s_op_w);
                        end if;
                    end if;
                    s_plot_busy <= '0';
                    if s_kaleido = '1' and s_op_mstep /= 3 then
                        s_op_mstep <= s_op_mstep + 1;
                        s_st <= 18;
                    else
                        s_st <= 16;
                    end if;

                -- ---- clear sweep ----
                when 30 =>
                    s_st_we    <= '1';
                    s_st_waddr <= s_clr_k;
                    s_st_wdata <= (others => '1');
                    if s_clr_k = to_unsigned(C_STORE - 1, 14) then
                        s_clear_req <= '0';
                        s_pA_t <= s_pA_h;  s_pB_t <= s_pB_h;  s_pC_t <= s_pC_h;
                        s_lag  <= (others => '0');
                        s_hvalid <= '0';
                        s_tvalid <= '0';
                        s_hrv <= '0';
                        s_trv <= '0';
                        s_st <= 0;
                    else
                        s_clr_k <= s_clr_k + 1;
                    end if;

                when others =>
                    s_st <= 0;
            end case;
        end if;
    end process p_plot;

    -- ====================================================================
    -- sync / video delay
    -- ====================================================================
    p_sr : process(clk)
    begin
        if rising_edge(clk) then
            s_y_sr(0)     <= data_in.y;
            s_u_sr(0)     <= data_in.u;
            s_v_sr(0)     <= data_in.v;
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LATENCY - 1 loop
                s_y_sr(i)     <= s_y_sr(i - 1);
                s_u_sr(i)     <= s_u_sr(i - 1);
                s_v_sr(i)     <= s_v_sr(i - 1);
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_sr;

    -- ====================================================================
    -- Output
    -- ====================================================================
    data_out.hsync_n <= s_hsync_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);

    data_out.y <= s_y_sr(C_LATENCY - 1) when (s_drawn = '0' and s_bg_video = '1')
                  else std_logic_vector(s_d_y);
    data_out.u <= s_u_sr(C_LATENCY - 1) when (s_drawn = '0' and s_bg_video = '1')
                  else std_logic_vector(s_d_u);
    data_out.v <= s_v_sr(C_LATENCY - 1) when (s_drawn = '0' and s_bg_video = '1')
                  else std_logic_vector(s_d_v);

end architecture spiroscope;
