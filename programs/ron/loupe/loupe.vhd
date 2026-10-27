-- loupe.vhd  (v0.1.0) -- a glass loupe over torn scallops
--
-- LOUPE.  An analog colourizer patch rebuilt in one pass, made from a
-- reference capture: white scallops pushing in from the right edge, nested
-- in cyan and grey with concave spikes between them, mirrored top to bottom,
-- the bands torn line by line into ragged edges and yellow lightning, and a
-- glass bubble in the middle holding a diamond of magenta chevrons.
-- Synthesis: no input video is read.
--
-- PICTURE
--  1. FIELD.  d = the distance to the nearest of a column of points just
--     off the right edge (Scallops points per frame height), horizontally
--     stretched: d = sqrt((k.(Xc - x))^2 + t^2), t = the folded distance to
--     the nearest point row.  The scallops are its contours, the spikes the
--     cusps between neighbours, and the fold gives the mirror.  Twin mirrors
--     the field about the centre: a second wall of scallops on the left.
--  2. CONTOURS.  index = d.Contours - flow, through a 224-entry palette of
--     hard bands with one- and two-entry fringes; Speed makes the contours
--     stream out of the points.
--  3. TEAR.  Each line's d is offset by a random amount (held runs, bursts,
--     a fine per-line jitter), ramped on beyond an onset level: the cores
--     and their first ring stay clean, the bands beyond shred, and a thin
--     yellow band in the grey becomes lightning.  Tear (P12) raises the
--     amount and pulls the onset in until everything shreds.
--  4. LOUPE.  A glass disc with a thin grey rim.  Facet: a diamond of
--     chevron stripes (their own palette entries), dense rim lines along the
--     diamond's edges, the field behind it washed out (or Smoke: darkened).
--     Window: the disc shows the field UNTORN, lightly tinted.  Drift: the
--     lens drifts and bounces around the middle of the frame.
--  5. GHOST.  Thin pale chevron lines (a linear cousin of the field, apexes
--     on the cusp rows) laid over everything outside the lens.
--
-- ENGINE
--   d^2 comes from an exact second-order DDA (a^2 += D, D += 2k^2; Twin
--   holds a^2 at the centre and turns D into 2k^2 - D, so the left half is
--   the right half reversed) plus a per-line t^2; the square root is a
--   normalize -> 2048 x 11 mantissa ROM -> denormalize pipeline.  A
--   sequencer with one serial multiplier (op code and one-hot operand
--   selects registered from the state number, wraith's fix), one serial
--   divider and one serial square root does the per-field set-up (first
--   vsync edge after active video) and a per-line engine (kicked at the
--   line's start, computing the NEXT line, ~200 clocks): the fold and t^2,
--   the tear draw, the lens chords (two square roots), the diamond limit
--   and the chevron / ghost line constants.  The knob glides run on one
--   shared datapath over a rotating bank, five clocks after each vsync.
--   The lens flags ride a block-RAM delay ring to the palette and blends.
--
-- CONTROLS
--   K1 Scallops  1 .. 6 points per frame height (square law)
--   K2 Stretch   round scallops -> long spikes (k = 1.0 .. 0.08)
--   K3 Contours  0.5 .. 5 palette cycles over the field (square law)
--   K4 Speed     bipolar contour flow (+ a slow vertical drift), still in the
--                centre
--   K5 Lens      lens radius 0.12 .. 0.62 frame heights
--   K6 Palette   Loupe / Ember / Abyss / Toxic / Candy / Mono / Neon / Rust
--   S7 Lens      Facet (chevron diamond) / Window (the untorn field)
--   S8 Twin      the field mirrored about the centre
--   S9 Drift     the lens drifts and bounces
--   S10 Glass    Clear (washed out) / Smoke (darkened)
--   S11 Ghost    pale ghost chevron lines
--   P12 Tear     0 = pristine scallops, 100 % = everything shredded
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, fixed
-- width); one latency for all modes (23 stages); outputs gated to neutral
-- outside the generated avid; palettes stored U/V-swapped (loupe.py);
-- per-field work on the first vsync EDGE after active video; line engine
-- on active lines only; bottom field starts half a field line down; HD HDMI
-- bar conditioning ([1 2 1] on Y/U/V, luma to ~775) before the gate.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.loupe_pkg.all;

architecture loupe of program_top is

    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- multiplier op table: a source, b source (0 = square of a), iterations
    type t_ops is array(0 to 37) of integer range 0 to 31;
    constant OP_A : t_ops := (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 4, 11, 12, 13, 11,
                              14, 15, 14, 14, 3, 16, 5, 17, 18, 19, 20, 21, 22, 23, 22, 24,
                              25, 25, 25, 25, 26);
    constant OP_B : t_ops := (0, 1, 0, 2, 3, 0, 0, 0, 4, 0, 5, 6, 7, 8, 0, 0, 0,
                              9, 10, 8, 11, 12, 13, 14, 0, 14, 15, 0, 16, 0, 17, 18, 19,
                              20, 21, 22, 23, 24);
    constant OP_N : t_ops := (10, 11, 10, 9, 11, 10, 10, 9, 15, 10, 10, 12, 11, 10, 10, 10, 9,
                              9, 12, 10, 10, 8, 12, 12, 20, 12, 13, 13, 8, 12, 17, 13, 10,
                              11, 10, 11, 11, 8);

    ------------------------------------------------------------------------
    -- raster timing
    ------------------------------------------------------------------------
    signal s_timing : t_video_timing_port;
    signal s_hcnt, s_meas_w : unsigned(11 downto 0) := (others => '0');
    signal s_vcnt, s_meas_h : unsigned(10 downto 0) := (others => '0');
    signal s_ilace  : std_logic := '0';
    signal s_saw    : std_logic := '0';
    signal s_vs_go  : std_logic := '0';

    -- generated avid (aurora / cubist / damier recipe)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_go   : std_logic := '0';

    ------------------------------------------------------------------------
    -- knob glides: a rotating bank of (hold, output 10.4), one shared
    -- datapath; channel 0 K1, 1 K2, 2 K3, 3 K5, 4 P12
    ------------------------------------------------------------------------
    signal gb_h   : unsigned(49 downto 0) := (others => '0');   -- 5 x 10
    signal gb_o   : unsigned(69 downto 0) := (others => '0');   -- 5 x 14
    signal gl_n   : unsigned(3 downto 0) := (others => '0');
    signal gl_h, gl_r : unsigned(9 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- sequencer
    ------------------------------------------------------------------------
    signal q_st : unsigned(6 downto 0) := (others => '0');
    attribute fsm_encoding : string;
    attribute fsm_encoding of q_st : signal is "none";
    signal q_fpend, q_lpend, q_prime : std_logic := '0';

    -- serial multiplier: op code + one-hot operand selects registered from
    -- the state number; the fetch is a flat AND-OR into o_*
    signal mul_go, mul_busy, m_ld : std_logic := '0';
    signal mul_op : unsigned(5 downto 0) := (others => '0');
    signal sel_a  : std_logic_vector(26 downto 0) := (others => '0');
    signal sel_b  : std_logic_vector(24 downto 0) := (others => '0');
    signal o_a    : signed(31 downto 0) := (others => '0');
    signal o_b    : unsigned(19 downto 0) := (others => '0');
    signal o_n    : unsigned(4 downto 0) := (others => '0');
    signal m_acc, m_a : signed(41 downto 0) := (others => '0');
    signal m_b    : unsigned(19 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');

    -- serial divider: q = 2^(k-1) / den (restoring)
    signal div_go, div_busy : std_logic := '0';
    signal div_k   : unsigned(4 downto 0) := (others => '0');
    signal div_den : unsigned(11 downto 0) := (others => '0');
    signal d_rem   : unsigned(11 downto 0) := (others => '0');
    signal d_q     : unsigned(24 downto 0) := (others => '0');
    signal d_cnt   : unsigned(4 downto 0) := (others => '0');
    signal d_first : std_logic := '0';

    -- serial square root: root = floor(sqrt(in)), 20-bit in, 10-bit root
    signal sq_go, sq_busy : std_logic := '0';
    signal sq_x    : unsigned(19 downto 0) := (others => '0');
    signal sq_rem  : unsigned(11 downto 0) := (others => '0');
    signal sq_root : unsigned(9 downto 0) := (others => '0');
    signal sq_cnt  : unsigned(3 downto 0) := (others => '0');

    -- per-field values
    signal c_k1, c_s2, c_k3, c_k5, c_p12, c_k4 : unsigned(9 downto 0) := (others => '0');
    signal c_pal  : unsigned(2 downto 0) := (others => '0');
    signal c_win, c_twin, c_drift, c_smoke, c_ghost : std_logic := '0';
    signal c_il   : std_logic := '0';
    signal c_W    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal c_Hf   : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal c_H    : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal c_recw : unsigned(15 downto 0) := to_unsigned(8738, 16);   -- 2^24 / W
    signal c_dy   : unsigned(12 downto 0) := to_unsigned(970, 13);    -- 2^20 / field lines
    signal c_t20  : unsigned(19 downto 0) := (others => '0');         -- scratch: knob^2
    signal c_npts : unsigned(10 downto 0) := to_unsigned(532, 11);    -- points x 256
    signal c_Pd   : unsigned(12 downto 0) := to_unsigned(1971, 13);   -- d-units per period
    signal c_ky   : unsigned(19 downto 0) := (others => '0');         -- point cycles/line x 2^24
    signal c_sxq  : unsigned(10 downto 0) := to_unsigned(307, 11);    -- stretch k x 1024
    signal c_r455 : unsigned(24 downto 0) := (others => '0');
    signal c_kk   : unsigned(9 downto 0) := to_unsigned(72, 10);      -- a-units per px
    signal c_K    : unsigned(18 downto 0) := (others => '0');         -- kk^2
    signal c_G16  : unsigned(10 downto 0) := to_unsigned(396, 11);    -- index x16 per d-unit
    signal c_dir  : std_logic := '0';
    signal c_magv : unsigned(8 downto 0) := (others => '0');
    signal c_spd  : signed(18 downto 0) := (others => '0');
    signal c_yoff : unsigned(23 downto 0) := (others => '0');
    signal c_tY   : unsigned(23 downto 0) := (others => '0');
    signal c_tP, c_tC : unsigned(15 downto 0) := (others => '0');
    signal c_phi0 : unsigned(23 downto 0) := (others => '0');
    signal c_tg   : unsigned(7 downto 0) := (others => '0');
    signal c_d0   : unsigned(10 downto 0) := to_unsigned(1037, 11);
    signal c_k5b  : unsigned(9 downto 0) := (others => '0');
    signal c_Rl, c_Rin, c_Rout, c_Rd : unsigned(9 downto 0) := (others => '0');
    signal c_Lx   : unsigned(11 downto 0) := to_unsigned(975, 12);
    signal c_Ly   : unsigned(10 downto 0) := to_unsigned(532, 11);
    signal c_ldx, c_ldy : std_logic := '0';                           -- drift directions
    signal c_Xc, c_Dx, c_xm1, c_bx3 : unsigned(11 downto 0) := (others => '0');
    signal c_by3  : unsigned(10 downto 0) := (others => '0');
    signal c_t25  : unsigned(24 downto 0) := (others => '0');
    signal c_kb, c_ikb : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal c_Rpx  : unsigned(10 downto 0) := (others => '0');
    signal c_Rin2, c_Rout2 : unsigned(19 downto 0) := (others => '0');
    signal c_ikb2 : unsigned(16 downto 0) := (others => '0');
    signal c_ks   : unsigned(12 downto 0) := (others => '0');
    signal c_kbs  : unsigned(13 downto 0) := (others => '0');
    signal c_kcs  : unsigned(9 downto 0) := (others => '0');
    signal c_bxd0 : signed(23 downto 0) := (others => '0');
    signal c_rimw : unsigned(17 downto 0) := (others => '0');
    signal c_Rdks : unsigned(20 downto 0) := (others => '0');
    signal c_kg   : unsigned(9 downto 0) := (others => '0');
    signal c_gx0  : unsigned(15 downto 0) := (others => '0');
    signal c_A2st  : unsigned(38 downto 0) := (others => '0');          -- the DDA's start
    signal c_Dst   : signed(31 downto 0) := (others => '0');
    signal q_X    : unsigned(11 downto 0) := (others => '0');
    signal q_a0   : unsigned(19 downto 0) := (others => '0');

    -- per-line engine (the pixel path loads the line's values at g_go)
    signal l_phi  : unsigned(23 downto 0) := (others => '0');
    signal l_yf   : unsigned(11 downto 0) := (others => '0');
    signal l_rng  : unsigned(31 downto 0) := x"2545F491";
    signal l_bc   : unsigned(5 downto 0) := (others => '0');
    signal l_bl   : unsigned(1 downto 0) := (others => '0');
    signal l_j    : signed(7 downto 0) := (others => '0');
    signal l_je   : signed(7 downto 0) := (others => '0');
    signal l_tri  : unsigned(15 downto 0) := (others => '0');
    signal l_t    : unsigned(12 downto 0) := (others => '0');
    signal l_t2   : unsigned(23 downto 0) := (others => '0');
    signal l_n, l_n34 : signed(11 downto 0) := (others => '0');
    signal l_aby  : unsigned(11 downto 0) := (others => '0');
    signal l_by2  : unsigned(21 downto 0) := (others => '0');
    signal l_q    : unsigned(19 downto 0) := (others => '0');
    signal l_qok  : std_logic := '0';
    signal l_hwin, l_hwout : unsigned(9 downto 0) := (others => '0');
    signal l_vin, l_vout : std_logic := '0';
    signal l_byks : unsigned(20 downto 0) := (others => '0');
    signal l_lim  : signed(21 downto 0) := (others => '0');
    signal l_cl, l_g0 : unsigned(15 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pixel path
    ------------------------------------------------------------------------
    -- S0: accumulators
    signal a_A2   : unsigned(38 downto 0) := (others => '0');
    signal a_D    : signed(31 downto 0) := (others => '0');
    signal a_x    : unsigned(11 downto 0) := (others => '0');
    signal a_mid  : std_logic := '0';
    signal a_bxd  : signed(23 downto 0) := (others => '0');
    signal a_g    : unsigned(15 downto 0) := (others => '0');
    signal px_t2  : unsigned(23 downto 0) := (others => '0');
    signal px_n, px_n34 : signed(11 downto 0) := (others => '0');
    signal px_hwin, px_hwout : unsigned(9 downto 0) := (others => '0');
    signal px_vin, px_vout : std_logic := '0';
    signal px_lim : signed(21 downto 0) := (others => '0');
    signal px_cl  : unsigned(15 downto 0) := (others => '0');

    signal p1_s   : unsigned(25 downto 0) := (others => '0');
    signal p1_ax  : unsigned(11 downto 0) := (others => '0');
    signal p1_abx : unsigned(22 downto 0) := (others => '0');
    signal p1_chev : unsigned(15 downto 0) := (others => '0');
    signal p1_gl  : std_logic := '0';
    signal p2_v   : unsigned(25 downto 0) := (others => '0');
    signal p2_sh  : unsigned(4 downto 0) := (others => '0');
    signal p2_in, p2_ring, p2_india, p2_gl : std_logic := '0';
    signal p2_rimd : signed(23 downto 0) := (others => '0');
    signal p2_sent : unsigned(4 downto 0) := (others => '0');
    signal p3_v   : unsigned(25 downto 0) := (others => '0');
    signal p3_sh  : unsigned(4 downto 0) := (others => '0');
    signal p4_a   : unsigned(10 downto 0) := (others => '0');
    signal p4_r, p5_r, p6_r : unsigned(3 downto 0) := (others => '0');
    signal r5_v   : std_logic_vector(10 downto 0) := (others => '0');
    signal p6_v   : unsigned(12 downto 0) := (others => '0');
    signal p7_v   : unsigned(12 downto 0) := (others => '0');
    signal p7_r   : unsigned(1 downto 0) := (others => '0');
    signal p8_d, p9_d, p10_d : unsigned(12 downto 0) := (others => '0');
    signal p9_e   : signed(13 downto 0) := (others => '0');
    signal p10_nr : signed(11 downto 0) := (others => '0');
    signal p11_dt : signed(14 downto 0) := (others => '0');
    signal p12_h  : signed(20 downto 0) := (others => '0');
    signal p12_l  : signed(21 downto 0) := (others => '0');
    signal p13_s  : signed(26 downto 0) := (others => '0');
    signal p14_i  : unsigned(15 downto 0) := (others => '0');
    signal p15_e  : unsigned(7 downto 0) := (others => '0');
    signal p16_a  : unsigned(10 downto 0) := (others => '0');
    signal r17_y, r17_u, r17_v : std_logic_vector(7 downto 0) := (others => '0');
    signal t18_y, t18_u, t18_v : unsigned(9 downto 0) := (others => '0');
    signal p19_y, p19_u, p19_v : unsigned(9 downto 0) := (others => '0');
    signal p19_dy, p19_du, p19_dv : signed(10 downto 0) := (others => '0');
    signal p19_sh : unsigned(1 downto 0) := (others => '0');
    signal p20_y, p20_u, p20_v : unsigned(9 downto 0) := (others => '0');

    -- Window: no tear inside the lens (S3 .. S9)
    signal nt     : std_logic_vector(3 to 9) := (others => '0');

    -- lens flags {stripe, glass, rim line, ring, ghost} & stripe entry:
    -- written at S3 (the RAM write lands a clock later), read back as S14,
    -- re-registered (S15), then the five flags ride FFs to S18
    type t_flram is array(0 to 255) of std_logic_vector(15 downto 0);
    signal fl_ram : t_flram := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of fl_ram : signal is true;
    signal fl_wp  : unsigned(7 downto 0) := to_unsigned(10, 8);
    signal fl_rp  : unsigned(7 downto 0) := (others => '0');
    signal fl_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal fl_rd  : std_logic_vector(15 downto 0) := (others => '0');
    signal fl15   : std_logic_vector(9 downto 0) := (others => '0');
    type t_fd is array(16 to 18) of std_logic_vector(4 downto 0);
    signal fld    : t_fd := (others => (others => '0'));

    -- output conditioning (the HD HDMI bar fix, damier v0.4.4): [1 2 1]
    -- horizontal smoothing on Y/U/V, luma scaled toward black
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal p22_y, p22_u, p22_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- syncs (generated avid, hsync, vsync, field) from S1 to S23
    type t_sd is array(1 to 23) of std_logic_vector(3 downto 0);
    signal sd : t_sd := (others => "0110");

begin

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- Raster measurement + once-per-field anchor
    -- ====================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_hcnt > 0 then s_meas_w <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_vcnt > 0 then s_meas_h <= s_vcnt; end if;
                s_vcnt  <= (others => '0');
                s_ilace <= s_timing.is_interlaced;
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;

            -- first vsync edge after active video only (analog vsync serrates)
            s_vs_go <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_vs_go <= '1';
            end if;
        end if;
    end process p_measure;

    -- ====================================================================
    -- Clean avid: starts 3 or 4 clocks after the source's first avid rise,
    -- whichever keeps the hsync->start parity at its voted usual value;
    -- ends after the field-latched width.  g_go is the cycle BEFORE it
    -- rises, when the pixel path loads the line's accumulators.
    -- ====================================================================
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;

            if g_hsr = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if (g_sel = '0' and g_rsr(0) = '1') or (g_sel = '1' and g_rsr(1) = '1') then
                g_go <= '1';
            else
                g_go <= '0';
            end if;
            if g_avr = '1' and g_arm = '1' then
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

            if g_go = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            if s_vs_go = '1' then
                g_wm1 <= s_meas_w - 1;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Knob glides, eleven clocks after each vsync edge, two per channel: the
    -- head's hold follows its pot only beyond a +-4 deadband (snapping to
    -- the rails), then the output eases toward it by 1/8 (the Tear slider
    -- by 1/4) and the bank rotates one place.
    -- ====================================================================
    p_glide : process(clk)
        variable v_e : signed(11 downto 0);
        variable v_t : unsigned(13 downto 0);
        variable v_d : signed(15 downto 0);
        variable v_o : unsigned(13 downto 0);
        variable v_s : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            if s_vs_go = '1' then
                gl_n <= to_unsigned(11, 4);
            elsif gl_n /= 0 then
                gl_n <= gl_n - 1;
                -- the next channel's pot, a clock ahead of its hold
                case to_integer(gl_n) is
                    when 11     => gl_r <= unsigned(registers_in(0));
                    when 9      => gl_r <= unsigned(registers_in(1));
                    when 7      => gl_r <= unsigned(registers_in(2));
                    when 5      => gl_r <= unsigned(registers_in(4));
                    when 3      => gl_r <= unsigned(registers_in(7));
                    when others => null;
                end case;
                if gl_n(0) = '0' then
                    -- the hold
                    v_e := signed(resize(gl_r, 12)) - signed(resize(gb_h(9 downto 0), 12));
                    gl_h <= gb_h(9 downto 0);
                    if gl_r < 4 then
                        gl_h <= (others => '0');
                    elsif gl_r > 1019 then
                        gl_h <= (others => '1');
                    elsif v_e >= 4 or v_e <= -4 then
                        gl_h <= gl_r;
                    end if;
                elsif gl_n /= 11 then
                    -- the ease, and rotate
                    v_t := gl_h & "0000";
                    v_d := signed(resize(v_t, 16)) - signed(resize(gb_o(13 downto 0), 16));
                    if gl_n = 1 then
                        v_s := signed(resize(gb_o(13 downto 0), 16)) + shift_right(v_d, 2);
                    else
                        v_s := signed(resize(gb_o(13 downto 0), 16)) + shift_right(v_d, 3);
                    end if;
                    if v_d < 8 and v_d > -8 then
                        v_o := v_t;
                    else
                        v_o := unsigned(v_s(13 downto 0));
                    end if;
                    gb_h <= gl_h & gb_h(49 downto 10);
                    gb_o <= v_o & gb_o(69 downto 14);
                end if;
            end if;
        end if;
    end process p_glide;

    -- ====================================================================
    -- The multiplier's op code and one-hot operand selects are a registered
    -- function of the state, with no enable (the issuing state is still
    -- current at the edge where it raises mul_go, so the fetch one clock
    -- later reads that state's op).
    -- ====================================================================
    p_opmap : process(clk)
        variable v_op : integer range 0 to 37;
    begin
        if rising_edge(clk) then
            case to_integer(q_st) is
                when 3  => v_op := 0;
                when 4  => v_op := 33;
                when 6  => v_op := 1;
                when 7  => v_op := 2;
                when 8  => v_op := 34;
                when 9  => v_op := 3;
                when 11 => v_op := 4;
                when 13 => v_op := 5;
                when 14 => v_op := 6;
                when 15 => v_op := 35;
                when 18 => v_op := 7;
                when 19 => v_op := 8;
                when 21 => v_op := 9;
                when 22 => v_op := 36;
                when 24 => v_op := 10;
                when 27 => v_op := 11;
                when 28 => v_op := 12;
                when 30 => v_op := 13;
                when 31 => v_op := 37;
                when 33 => v_op := 14;
                when 34 => v_op := 15;
                when 35 => v_op := 16;
                when 37 => v_op := 17;
                when 39 => v_op := 18;
                when 40 => v_op := 19;
                when 41 => v_op := 20;
                when 42 => v_op := 21;
                when 43 => v_op := 22;
                when 45 => v_op := 23;
                when 46 => v_op := 24;
                when 47 => v_op := 25;
                when 98 => v_op := 26;
                when 99 => v_op := 27;
                when 103 => v_op := 28;
                when 106 => v_op := 29;
                when 109 | 112 => v_op := 30;
                when 115 => v_op := 31;
                when 117 => v_op := 32;
                when others => v_op := 0;
            end case;
            mul_op <= to_unsigned(v_op, 6);
            sel_a  <= (others => '0');
            sel_a(OP_A(v_op)) <= '1';
            sel_b  <= (others => '0');
            sel_b(OP_B(v_op)) <= '1';
        end if;
    end process p_opmap;

    -- ====================================================================
    -- Serial multiplier (shift-add, one bit of b per clock).  go -> operand
    -- fetch (flat AND-OR, registered) -> load -> n iterations.
    -- ====================================================================
    p_mul : process(clk)
        variable v_fa : signed(31 downto 0);
        variable v_fb : unsigned(19 downto 0);
        function f_a(s : std_logic; v : signed) return signed is
        begin
            return (31 downto 0 => s) and resize(v, 32);
        end function;
        function f_u(s : std_logic; v : unsigned) return signed is
        begin
            return (31 downto 0 => s) and signed(resize(v, 32));
        end function;
        function f_b(s : std_logic; v : unsigned) return unsigned is
        begin
            return (19 downto 0 => s) and resize(v, 20);
        end function;
    begin
        if rising_edge(clk) then
            m_ld <= '0';
            if mul_go = '1' then
                v_fa := f_u(sel_a(0), c_k1) or f_u(sel_a(1), c_dy) or f_u(sel_a(2), c_s2)
                     or f_u(sel_a(3), c_recw) or f_u(sel_a(4), c_r455) or f_u(sel_a(5), c_kk)
                     or f_u(sel_a(6), c_k3) or f_u(sel_a(7), c_magv) or f_u(sel_a(8), c_npts)
                     or f_u(sel_a(9), c_p12) or f_u(sel_a(10), c_H) or f_u(sel_a(11), c_ikb)
                     or f_u(sel_a(12), c_Rin) or f_u(sel_a(13), c_Rout) or f_u(sel_a(14), c_ks)
                     or f_u(sel_a(15), c_kbs) or f_u(sel_a(16), c_kg) or f_u(sel_a(17), q_a0)
                     or f_u(sel_a(18), c_K) or f_u(sel_a(19), l_tri) or f_u(sel_a(20), l_t)
                     or f_a(sel_a(21), l_je) or f_u(sel_a(22), l_aby) or f_u(sel_a(23), l_q)
                     or f_a(sel_a(24), l_n) or f_u(sel_a(25), c_t20) or f_u(sel_a(26), c_Rl);
                v_fb := f_b(sel_b(1), c_npts) or f_b(sel_b(2), to_unsigned(455, 9))
                     or f_b(sel_b(3), c_sxq) or f_b(sel_b(4), to_unsigned(31555, 15))
                     or f_b(sel_b(5), c_k5b) or f_b(sel_b(6), c_W) or f_b(sel_b(7), c_H)
                     or f_b(sel_b(8), c_Rl) or f_b(sel_b(9), c_kb) or f_b(sel_b(10), c_Dx)
                     or f_b(sel_b(11), c_Rd) or f_b(sel_b(12), to_unsigned(219, 8))
                     or f_b(sel_b(13), c_Xc) or f_b(sel_b(14), q_X) or f_b(sel_b(15), c_Pd)
                     or f_b(sel_b(16), c_tg) or f_b(sel_b(17), c_ikb2) or f_b(sel_b(18), c_ks)
                     or f_b(sel_b(19), c_kcs) or f_b(sel_b(20), to_unsigned(1280, 11))
                     or f_b(sel_b(21), to_unsigned(942, 10)) or f_b(sel_b(22), to_unsigned(1782, 11))
                     or f_b(sel_b(23), to_unsigned(1300, 11)) or f_b(sel_b(24), to_unsigned(218, 8));
                o_a <= v_fa;
                if sel_b(0) = '1' then
                    o_b <= unsigned(v_fa(19 downto 0));
                else
                    o_b <= v_fb;
                end if;
                o_n      <= to_unsigned(OP_N(to_integer(mul_op)), 5);
                m_ld     <= '1';
                mul_busy <= '1';
            elsif m_ld = '1' then
                m_acc <= (others => '0');
                m_a   <= resize(o_a, 42);
                m_b   <= o_b;
                m_cnt <= o_n;
            elsif mul_busy = '1' then
                if m_b(0) = '1' then
                    m_acc <= m_acc + m_a;
                end if;
                m_a   <= shift_left(m_a, 1);
                m_b   <= shift_right(m_b, 1);
                m_cnt <= m_cnt - 1;
                if m_cnt = 1 then
                    mul_busy <= '0';
                end if;
            end if;
        end if;
    end process p_mul;

    -- ====================================================================
    -- Serial divider: q = floor(2^(k-1) / den), one quotient bit per clock
    -- ====================================================================
    p_div : process(clk)
        variable v_r : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            if div_go = '1' then
                d_rem    <= (others => '0');
                d_q      <= (others => '0');
                d_cnt    <= div_k;
                d_first  <= '1';
                div_busy <= '1';
            elsif div_busy = '1' then
                v_r := d_rem & d_first;
                if v_r >= resize(div_den, 13) then
                    v_r := v_r - resize(div_den, 13);
                    d_q <= d_q(23 downto 0) & '1';
                else
                    d_q <= d_q(23 downto 0) & '0';
                end if;
                d_rem   <= v_r(11 downto 0);
                d_first <= '0';
                d_cnt   <= d_cnt - 1;
                if d_cnt = 1 then
                    div_busy <= '0';
                end if;
            end if;
        end if;
    end process p_div;

    -- ====================================================================
    -- Serial square root (restoring, two input bits per clock) of the
    -- multiplier's result / 2^16
    -- ====================================================================
    p_sqrt : process(clk)
        variable v_r, v_t : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if sq_go = '1' then
                sq_x    <= unsigned(m_acc(35 downto 16));
                sq_rem  <= (others => '0');
                sq_root <= (others => '0');
                sq_cnt  <= to_unsigned(10, 4);
                sq_busy <= '1';
            elsif sq_busy = '1' then
                v_r := sq_rem(9 downto 0) & sq_x(19 downto 18);
                v_t := sq_root & "01";
                if v_r >= v_t then
                    sq_rem  <= v_r - v_t;
                    sq_root <= sq_root(8 downto 0) & '1';
                else
                    sq_rem  <= v_r;
                    sq_root <= sq_root(8 downto 0) & '0';
                end if;
                sq_x   <= sq_x(17 downto 0) & "00";
                sq_cnt <= sq_cnt - 1;
                if sq_cnt = 1 then
                    sq_busy <= '0';
                end if;
            end if;
        end if;
    end process p_sqrt;

    -- ====================================================================
    -- Sequencer: per-field set-up (states 1..49) and the per-line engine
    -- (96..118).  One small operation per state.  A unit is free when its
    -- go is low and busy is low (go is registered, busy follows one clock
    -- later), so a wait state can follow its issue state directly.
    -- ====================================================================
    p_seq : process(clk)
        variable v_k6 : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_ad : unsigned(9 downto 0);
        variable v_jr : signed(7 downto 0);
        variable v_j  : signed(7 downto 0);
        variable v_f  : unsigned(15 downto 0);
        variable v_by : signed(12 downto 0);
        variable v_q  : signed(22 downto 0);
        variable v_P  : signed(31 downto 0);
    begin
        if rising_edge(clk) then
            mul_go <= '0';
            div_go <= '0';
            sq_go  <= '0';
            if s_vs_go = '1' then
                q_fpend <= '1';
            end if;
            if g_go = '1' then
                q_lpend <= '1';                       -- an active line starts
            end if;

            case to_integer(q_st) is
                when 0 =>
                    if q_fpend = '1' then
                        q_fpend <= '0';
                        q_lpend <= '0';
                        q_st    <= to_unsigned(1, 7);
                    elsif q_lpend = '1' then
                        q_lpend <= '0';
                        q_st    <= to_unsigned(96, 7);
                    end if;

                ------------------------------------------------------------
                -- per field
                ------------------------------------------------------------
                when 1 =>       -- latch the raster and switches; start 2^24 / W
                    c_k4  <= unsigned(registers_in(3));
                    -- labelled knob: 0..7 at program load is the label index
                    v_k6 := unsigned(registers_in(5));
                    if v_k6 < 8 then
                        c_pal <= v_k6(2 downto 0);
                    else
                        c_pal <= v_k6(9 downto 7);
                    end if;
                    c_win   <= registers_in(6)(0);
                    c_twin  <= registers_in(6)(1);
                    c_drift <= registers_in(6)(2);
                    c_smoke <= registers_in(6)(3);
                    c_ghost <= registers_in(6)(4);
                    c_il    <= s_ilace;
                    c_W     <= s_meas_w;
                    c_Hf    <= s_meas_h;
                    if s_ilace = '1' then
                        c_H <= s_meas_h(9 downto 0) & '0';
                    else
                        c_H <= s_meas_h;
                    end if;
                    div_den <= s_meas_w;
                    div_k   <= to_unsigned(25, 5);
                    div_go  <= '1';
                    q_st    <= to_unsigned(2, 7);

                when 2 =>       -- W reciprocal; start 2^20 / field lines
                    if div_go = '0' and div_busy = '0' then
                        if d_q(24 downto 16) /= 0 then
                            c_recw <= (others => '1');
                        else
                            c_recw <= d_q(15 downto 0);
                        end if;
                        div_den <= resize(c_Hf, 12);
                        div_k   <= to_unsigned(21, 5);
                        div_go  <= '1';
                        q_st    <= to_unsigned(3, 7);
                    end if;

                when 3 =>       -- line step; the glided knobs (the glide burst
                                -- ended long ago); K1 squared
                    if div_go = '0' and div_busy = '0' then
                        if d_q(24 downto 13) /= 0 then
                            c_dy <= (others => '1');
                        else
                            c_dy <= d_q(12 downto 0);
                        end if;
                        c_k1   <= gb_o(13 downto 4);
                        c_s2   <= not gb_o(27 downto 18);     -- 1023 - K2
                        c_k3   <= gb_o(41 downto 32);
                        c_k5   <= gb_o(55 downto 46);
                        c_p12  <= gb_o(69 downto 60);
                        mul_go <= '1';
                        q_st   <= to_unsigned(4, 7);
                    end if;

                when 4 =>       -- . 1280
                    if mul_go = '0' and mul_busy = '0' then
                        c_t20  <= unsigned(m_acc(19 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(5, 7);
                    end if;

                when 5 =>       -- points x 256 = 256 + 1280 k1^2 / 2^20 (1 .. 6)
                    if mul_go = '0' and mul_busy = '0' then
                        c_npts <= to_unsigned(256, 11) + unsigned(m_acc(30 downto 20));
                        q_st   <= to_unsigned(6, 7);
                    end if;

                when 6 =>       -- Pd = 2^20 / points; ky = points . dy / 16
                    div_den <= resize(c_npts, 12);
                    div_k   <= to_unsigned(21, 5);
                    div_go  <= '1';
                    mul_go  <= '1';
                    q_st    <= to_unsigned(7, 7);

                when 7 =>       -- (1023 - K2)^2
                    if mul_go = '0' and mul_busy = '0' then
                        c_ky   <= unsigned(m_acc(23 downto 4));
                        mul_go <= '1';
                        q_st   <= to_unsigned(8, 7);
                    end if;

                when 8 =>       -- . 942
                    if mul_go = '0' and mul_busy = '0' then
                        c_t20  <= unsigned(m_acc(19 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(9, 7);
                    end if;

                when 9 =>       -- stretch k x 1024 = 82 + 942 s2^2 / 2^20; recw . 455
                    if mul_go = '0' and mul_busy = '0' then
                        c_sxq  <= to_unsigned(82, 11) + resize(unsigned(m_acc(29 downto 20)), 11);
                        mul_go <= '1';
                        q_st   <= to_unsigned(10, 7);
                    end if;

                when 10 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_r455 <= unsigned(m_acc(24 downto 0));
                        q_st   <= to_unsigned(11, 7);
                    end if;

                when 11 =>      -- kk = sxq . recw . 455 / 2^24
                    mul_go <= '1';
                    q_st   <= to_unsigned(12, 7);

                when 12 =>
                    if mul_go = '0' and mul_busy = '0' and div_go = '0' and div_busy = '0' then
                        if d_q(24 downto 13) /= 0 then
                            c_Pd <= (others => '1');
                        else
                            c_Pd <= d_q(12 downto 0);
                        end if;
                        c_kk <= unsigned(m_acc(33 downto 24));
                        q_st <= to_unsigned(13, 7);
                    end if;

                when 13 =>      -- K = kk^2
                    mul_go <= '1';
                    q_st   <= to_unsigned(14, 7);

                when 14 =>      -- K3 squared
                    if mul_go = '0' and mul_busy = '0' then
                        c_K    <= unsigned(m_acc(18 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(15, 7);
                    end if;

                when 15 =>      -- . 1782
                    if mul_go = '0' and mul_busy = '0' then
                        c_t20  <= unsigned(m_acc(19 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(16, 7);
                    end if;

                when 16 =>      -- Contours: G16 = 198 + 1782 k3^2 / 2^20 (0.5 .. 5)
                    if mul_go = '0' and mul_busy = '0' then
                        c_G16 <= to_unsigned(198, 11) + unsigned(m_acc(30 downto 20));
                        q_st  <= to_unsigned(17, 7);
                    end if;

                when 17 =>      -- Speed: bipolar with a +-24 still zone
                    v_d := signed(resize(c_k4, 11)) - 512;
                    if v_d < 0 then
                        c_dir <= '1';
                        v_ad  := unsigned(resize(-v_d, 10));
                    else
                        c_dir <= '0';
                        v_ad  := unsigned(resize(v_d, 10));
                    end if;
                    if v_ad <= 24 then
                        c_magv <= (others => '0');
                    else
                        c_magv <= resize(v_ad - 24, 9);
                    end if;
                    q_st <= to_unsigned(18, 7);

                when 18 =>      -- speed = mag^2
                    mul_go <= '1';
                    q_st   <= to_unsigned(19, 7);

                when 19 =>      -- the mirror row's phase: 0.4815 . points
                    if mul_go = '0' and mul_busy = '0' then
                        if c_dir = '1' then
                            c_spd <= -signed('0' & m_acc(17 downto 0));
                        else
                            c_spd <= signed('0' & m_acc(17 downto 0));
                        end if;
                        mul_go <= '1';
                        q_st   <= to_unsigned(20, 7);
                    end if;

                when 20 =>      -- time: the contours flow out of the points
                                -- (positive), the points drift, the chevrons flow
                    if mul_go = '0' and mul_busy = '0' then
                        c_yoff <= unsigned(m_acc(23 downto 0));
                        c_tP   <= c_tP + unsigned(resize(c_spd(18 downto 6), 16));
                        c_tY   <= c_tY + unsigned(resize(c_spd(18 downto 1), 24));
                        c_tC   <= c_tC + unsigned(resize(c_spd(18 downto 4), 16));
                        q_st   <= to_unsigned(21, 7);
                    end if;

                when 21 =>      -- line-0 phase; Tear squared
                    c_phi0 <= c_tY - c_yoff;
                    mul_go <= '1';
                    q_st   <= to_unsigned(22, 7);

                when 22 =>      -- tear gain p12^2 / 2^12; . 1300
                    if mul_go = '0' and mul_busy = '0' then
                        c_t20  <= unsigned(m_acc(19 downto 0));
                        c_tg   <= unsigned(m_acc(19 downto 12));
                        mul_go <= '1';
                        q_st   <= to_unsigned(23, 7);
                    end if;

                when 23 =>      -- tear onset 1300 (1 - p12^2)
                    if mul_go = '0' and mul_busy = '0' then
                        c_d0 <= to_unsigned(1300, 11) - unsigned(m_acc(30 downto 20));
                        q_st <= to_unsigned(24, 7);
                    end if;

                when 24 =>      -- lens radius = H (0.12 + 0.5 k5)
                    c_k5b  <= to_unsigned(123, 10) + resize(c_k5(9 downto 1), 10);
                    mul_go <= '1';
                    q_st   <= to_unsigned(25, 7);

                when 25 =>      -- lens centre: (W/2 + W/128, H/2 - H/128), or the
                                -- drift's next step (W/512 + 1 px, 1 + H/1024 lines)
                    if mul_go = '0' and mul_busy = '0' then
                        c_Rl <= unsigned(m_acc(19 downto 10));
                        if c_drift = '1' then
                            if c_ldx = '1' then
                                c_Lx <= c_Lx - resize(c_W(11 downto 9), 12) - 1;
                            else
                                c_Lx <= c_Lx + resize(c_W(11 downto 9), 12) + 1;
                            end if;
                            if c_ldy = '1' then
                                c_Ly <= c_Ly - resize(c_H(10 downto 10), 11) - 1;
                            else
                                c_Ly <= c_Ly + resize(c_H(10 downto 10), 11) + 1;
                            end if;
                        else
                            c_Lx <= ('0' & c_W(11 downto 1)) + c_W(11 downto 7);
                            c_Ly <= ('0' & c_H(10 downto 1)) - c_H(10 downto 7);
                        end if;
                        c_bx3 <= c_W - c_W(11 downto 2);
                        c_by3 <= c_H - c_H(10 downto 2);
                        q_st  <= to_unsigned(26, 7);
                    end if;

                when 26 =>      -- bounce inside [W/4, 3W/4] x [H/4, 3H/4]; the
                                -- point column 1.125 W; the Twin centre
                    if c_Lx >= c_bx3 then
                        c_ldx <= '1';
                    elsif c_Lx <= c_W(11 downto 2) then
                        c_ldx <= '0';
                    end if;
                    if c_Ly >= c_by3 then
                        c_ldy <= '1';
                    elsif c_Ly <= c_H(10 downto 2) then
                        c_ldy <= '0';
                    end if;
                    c_Xc  <= c_W + c_W(11 downto 3);
                    c_xm1 <= ('0' & c_W(11 downto 1)) - 2;
                    q_st  <= to_unsigned(27, 7);

                when 27 =>      -- W . dy (lens px per line)
                    mul_go <= '1';
                    q_st   <= to_unsigned(28, 7);

                when 28 =>      -- r455 . H
                    if mul_go = '0' and mul_busy = '0' then
                        c_t25  <= unsigned(m_acc(24 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(29, 7);
                    end if;

                when 29 =>      -- kb = 256 (16/9) H / W; ikb = 256 (9/16) W / H
                    if mul_go = '0' and mul_busy = '0' then
                        c_kb <= unsigned(m_acc(32 downto 24));
                        if c_il = '1' then
                            c_ikb <= resize(shift_right(resize(c_t25(24 downto 13), 10)
                                                        + resize(c_t25(24 downto 16), 10), 1), 9);
                        else
                            c_ikb <= resize(resize(c_t25(24 downto 13), 10)
                                            + resize(c_t25(24 downto 16), 10), 9);
                        end if;
                        q_st <= to_unsigned(30, 7);
                    end if;

                when 30 =>      -- lens radius in px
                    mul_go <= '1';
                    q_st   <= to_unsigned(31, 7);

                when 31 =>      -- rim radii; diamond 0.85 R
                    if mul_go = '0' and mul_busy = '0' then
                        c_Rpx  <= unsigned(m_acc(18 downto 8));
                        c_Rin  <= c_Rl - 1 - resize(c_H(10 downto 10), 10);
                        c_Rout <= c_Rl + 1 + resize(c_H(10 downto 10), 10);
                        mul_go <= '1';
                        q_st   <= to_unsigned(32, 7);
                    end if;

                when 32 =>      -- the diamond sits 0.11 R left of centre
                    if mul_go = '0' and mul_busy = '0' then
                        c_Rd <= unsigned(m_acc(17 downto 8));
                        c_Dx <= c_Lx - resize(c_Rpx(10 downto 3), 12) + resize(c_Rpx(10 downto 6), 12);
                        q_st <= to_unsigned(33, 7);
                    end if;

                when 33 =>      -- Rin^2
                    mul_go <= '1';
                    q_st   <= to_unsigned(34, 7);

                when 34 =>      -- Rout^2
                    if mul_go = '0' and mul_busy = '0' then
                        c_Rin2 <= unsigned(m_acc(19 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(35, 7);
                    end if;

                when 35 =>      -- ikb^2
                    if mul_go = '0' and mul_busy = '0' then
                        c_Rout2 <= unsigned(m_acc(19 downto 0));
                        mul_go  <= '1';
                        q_st    <= to_unsigned(36, 7);
                    end if;

                when 36 =>      -- stripe units per frame line: 36 per frame height
                    if mul_go = '0' and mul_busy = '0' then
                        c_ikb2 <= unsigned(m_acc(16 downto 0));
                        if c_il = '1' then
                            c_ks <= resize(shift_right(resize(c_dy, 19) sll 5, 5)
                                           + shift_right(resize(c_dy, 19) sll 2, 5), 13);
                        else
                            c_ks <= resize(shift_right(resize(c_dy, 19) sll 5, 4)
                                           + shift_right(resize(c_dy, 19) sll 2, 4), 13);
                        end if;
                        q_st <= to_unsigned(37, 7);
                    end if;

                when 37 =>      -- per px: ks . kb
                    mul_go <= '1';
                    q_st   <= to_unsigned(38, 7);

                when 38 =>      -- chevron tear scale ks / 8
                    if mul_go = '0' and mul_busy = '0' then
                        c_kbs <= unsigned(m_acc(21 downto 8));
                        c_kcs <= resize(c_ks(12 downto 3), 10);
                        q_st  <= to_unsigned(39, 7);
                    end if;

                when 39 =>      -- diamond phase start = -Dx . kbs
                    mul_go <= '1';
                    q_st   <= to_unsigned(40, 7);

                when 40 =>      -- rim band = R . ks / 8
                    if mul_go = '0' and mul_busy = '0' then
                        c_bxd0 <= -m_acc(23 downto 0);
                        mul_go <= '1';
                        q_st   <= to_unsigned(41, 7);
                    end if;

                when 41 =>      -- Rd . ks
                    if mul_go = '0' and mul_busy = '0' then
                        c_rimw <= unsigned(m_acc(20 downto 3));
                        mul_go <= '1';
                        q_st   <= to_unsigned(42, 7);
                    end if;

                when 42 =>      -- ghost step: 117 / 2^16 cycles per HD px
                    if mul_go = '0' and mul_busy = '0' then
                        c_Rdks <= unsigned(m_acc(20 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(43, 7);
                    end if;

                when 43 =>      -- ghost start = kg . Xc
                    if mul_go = '0' and mul_busy = '0' then
                        c_kg   <= unsigned(m_acc(23 downto 14));
                        mul_go <= '1';
                        q_st   <= to_unsigned(44, 7);
                    end if;

                when 44 =>      -- the DDA's first position: Xc (Twin: the mirror
                                -- of the right edge, W/8 + 1)
                    if mul_go = '0' and mul_busy = '0' then
                        c_gx0 <= unsigned(m_acc(15 downto 0)) + to_unsigned(21000, 16);
                        if c_twin = '1' then
                            q_X <= resize(c_W(11 downto 3), 12) + 1;
                        else
                            q_X <= c_Xc;
                        end if;
                        q_st <= to_unsigned(45, 7);
                    end if;

                when 45 =>      -- a0 = kk . X
                    mul_go <= '1';
                    q_st   <= to_unsigned(46, 7);

                when 46 =>      -- A2 = a0^2
                    if mul_go = '0' and mul_busy = '0' then
                        q_a0   <= unsigned(m_acc(19 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(47, 7);
                    end if;

                when 47 =>      -- K . X
                    if mul_go = '0' and mul_busy = '0' then
                        c_A2st <= unsigned(m_acc(38 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(48, 7);
                    end if;

                when 48 =>      -- D = K - 2KX (Twin: the distance grows, K + 2KX)
                    if mul_go = '0' and mul_busy = '0' then
                        v_P := m_acc(30 downto 0) & '0';
                        if c_twin = '1' then
                            c_Dst <= signed(resize(c_K, 32)) + v_P;
                        else
                            c_Dst <= signed(resize(c_K, 32)) - v_P;
                        end if;
                        q_st <= to_unsigned(49, 7);
                    end if;

                when 49 =>      -- field start; prime the line engine for line 0
                    if s_ilace = '1' and s_timing.field_n = '0' then
                        -- bottom field: half a field line (one frame line) down
                        l_phi <= c_phi0 + resize(c_ky(19 downto 1), 24);
                        l_yf  <= to_unsigned(1, 12);
                    else
                        l_phi <= c_phi0;
                        l_yf  <= (others => '0');
                    end if;
                    q_prime <= '1';
                    q_st    <= to_unsigned(96, 7);

                ------------------------------------------------------------
                -- per line (for the NEXT line)
                ------------------------------------------------------------
                when 96 =>
                    if q_prime = '0' then
                        l_phi <= l_phi + resize(c_ky, 24);
                        if c_il = '1' then
                            l_yf <= l_yf + 2;
                        else
                            l_yf <= l_yf + 1;
                        end if;
                    end if;
                    q_prime <= '0';
                    l_rng   <= l_rng xor shift_left(l_rng, 13);
                    q_st    <= to_unsigned(97, 7);

                when 97 =>      -- fold: distance to the nearest point row
                    l_rng <= l_rng xor shift_right(l_rng, 17);
                    v_f   := l_phi(23 downto 8);
                    if v_f(15) = '1' then
                        l_tri <= (not v_f) + 1;
                    else
                        l_tri <= v_f;
                    end if;
                    q_st <= to_unsigned(98, 7);

                when 98 =>      -- t = tri . Pd
                    l_rng  <= l_rng xor shift_left(l_rng, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(99, 7);

                when 99 =>      -- t^2
                    if mul_go = '0' and mul_busy = '0' then
                        l_t    <= unsigned(m_acc(28 downto 16));
                        mul_go <= '1';
                        q_st   <= to_unsigned(100, 7);
                    end if;

                when 100 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_t2 <= unsigned(m_acc(23 downto 0));
                        q_st <= to_unsigned(101, 7);
                    end if;

                when 101 =>     -- tear bursts: 1 in 32 lines starts a run of
                                -- 8..39 lines at one of four strengths
                    if l_bc = 0 then
                        if l_rng(4 downto 0) = 0 then
                            l_bc <= to_unsigned(8, 6) + resize(l_rng(9 downto 5), 6);
                            l_bl <= l_rng(11 downto 10);
                        end if;
                    else
                        l_bc <= l_bc - 1;
                    end if;
                    q_st <= to_unsigned(102, 7);

                when 102 =>     -- a new draw on 1 line in 8 (else held): +-8,
                                -- +-8..64 in a burst; plus a +-2 jitter
                    v_j := l_j;
                    if l_rng(14 downto 12) = "000" then
                        v_jr := signed(resize(l_rng(21 downto 15), 8)) - 64;
                        if l_bc /= 0 then
                            case to_integer(l_bl) is
                                when 0      => v_jr := shift_right(v_jr, 3);
                                when 1      => v_jr := shift_right(v_jr, 2);
                                when 2      => v_jr := shift_right(v_jr, 1);
                                when others => null;
                            end case;
                        else
                            v_jr := shift_right(v_jr, 3);
                        end if;
                        v_j := v_jr;
                    end if;
                    l_j  <= v_j;
                    l_je <= v_j + signed(resize(l_rng(23 downto 22), 8)) - 2;
                    q_st <= to_unsigned(103, 7);

                when 103 =>     -- n = je . tg / 16
                    mul_go <= '1';
                    q_st   <= to_unsigned(104, 7);

                when 104 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_n  <= m_acc(15 downto 4);
                        q_st <= to_unsigned(105, 7);
                    end if;

                when 105 =>     -- the line's distance from the lens centre
                    l_n34 <= shift_right(l_n, 1) + shift_right(l_n, 2);
                    v_by  := signed(resize(l_yf, 13)) - signed(resize(c_Ly, 13));
                    if v_by < 0 then
                        v_by := -v_by;
                    end if;
                    l_aby <= unsigned(v_by(11 downto 0));
                    q_st  <= to_unsigned(106, 7);

                when 106 =>     -- by^2
                    mul_go <= '1';
                    q_st   <= to_unsigned(107, 7);

                when 107 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_by2 <= unsigned(m_acc(21 downto 0));
                        q_st  <= to_unsigned(108, 7);
                    end if;

                when 108 | 111 =>   -- rim chords: R^2 - by^2 (inner, then outer)
                    if sq_go = '0' and sq_busy = '0' then
                        if q_st(0) = '0' then
                            v_q := signed('0' & resize(c_Rin2, 22)) - signed('0' & l_by2);
                        else
                            l_hwin <= sq_root;
                            l_vin  <= l_qok;
                            v_q := signed('0' & resize(c_Rout2, 22)) - signed('0' & l_by2);
                        end if;
                        if v_q < 0 then
                            l_q   <= (others => '0');
                            l_qok <= '0';
                        else
                            l_q   <= unsigned(v_q(19 downto 0));
                            l_qok <= '1';
                        end if;
                        q_st <= q_st + 1;
                    end if;

                when 109 | 112 =>   -- . ikb^2
                    mul_go <= '1';
                    q_st   <= q_st + 1;

                when 110 | 113 =>   -- sqrt(. / 2^16)
                    if mul_go = '0' and mul_busy = '0' then
                        sq_go <= '1';
                        q_st  <= q_st + 1;
                    end if;

                when 114 =>
                    if sq_go = '0' and sq_busy = '0' then
                        l_hwout <= sq_root;
                        l_vout  <= l_qok;
                        q_st    <= to_unsigned(115, 7);
                    end if;

                when 115 =>     -- |by| . ks
                    mul_go <= '1';
                    q_st   <= to_unsigned(116, 7);

                when 116 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_byks <= unsigned(m_acc(20 downto 0));
                        q_st   <= to_unsigned(117, 7);
                    end if;

                when 117 =>     -- diamond half-width; n . kcs
                    l_lim  <= signed('0' & c_Rdks) - signed('0' & l_byks);
                    mul_go <= '1';
                    q_st   <= to_unsigned(118, 7);

                when 118 =>     -- chevron line constant; ghost start
                    if mul_go = '0' and mul_busy = '0' then
                        l_cl <= c_tC - l_byks(15 downto 0) + unsigned(m_acc(15 downto 0));
                        l_g0 <= c_gx0 + (l_t(9 downto 0) & "000000") - (l_t(12 downto 0) & "000");
                        q_st <= to_unsigned(0, 7);
                    end if;

                when others =>
                    q_st <= to_unsigned(0, 7);
            end case;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Pixel path.  S0 = the clock where the accumulators hold the pixel's
    -- values and g_avid the pixel's avid; S23 = the output register.
    -- ====================================================================
    p_pix : process(clk)
        variable v_s   : unsigned(27 downto 0);
        variable v_x   : signed(12 downto 0);
        variable v_b   : signed(23 downto 0);
        variable v_r   : signed(24 downto 0);
        variable v_v   : unsigned(25 downto 0);
        variable v_sh  : unsigned(4 downto 0);
        variable v_e   : unsigned(7 downto 0);
        variable v_in  : std_logic;
        variable v_st  : std_logic;
        variable v_rim : std_logic;
        variable v_oy, v_ou, v_ov : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- S0: the line's accumulators (loaded at g_go from the engine)
            if g_go = '1' then
                a_A2     <= c_A2st;
                a_D      <= c_Dst;
                a_x      <= (others => '0');
                a_mid    <= '0';
                a_bxd    <= c_bxd0;
                a_g      <= l_g0;
                px_t2    <= l_t2;
                px_n     <= l_n;
                px_n34   <= l_n34;
                px_hwin  <= l_hwin;
                px_hwout <= l_hwout;
                px_vin   <= l_vin;
                px_vout  <= l_vout;
                px_lim   <= l_lim;
                px_cl    <= l_cl;
            else
                if a_mid = '1' then
                    -- the centre: repeat, then run the right half forward
                    a_D <= signed(resize(c_K & '0', 32)) - a_D;
                else
                    a_A2 <= unsigned(signed(a_A2) + resize(a_D, 39));
                    a_D  <= a_D + signed(resize(c_K & '0', 32));
                end if;
                a_x   <= a_x + 1;
                -- set for the clock at the centre pixel W/2 - 1
                if c_twin = '1' and a_x = c_xm1 then
                    a_mid <= '1';
                else
                    a_mid <= '0';
                end if;
                a_bxd <= a_bxd + signed(resize(c_kbs, 24));
                a_g   <= a_g - resize(c_kg, 16);
            end if;

            -- S1: s = a^2 / 4096 + t^2 (saturated); lens / diamond coordinates
            v_s := resize(a_A2(38 downto 12), 28) + resize(px_t2, 28);
            if v_s(27 downto 26) /= "00" then
                p1_s <= (others => '1');
            else
                p1_s <= v_s(25 downto 0);
            end if;
            v_x := signed('0' & a_x) - signed('0' & c_Lx);
            if v_x < 0 then
                v_x := -v_x;
            end if;
            p1_ax <= unsigned(v_x(11 downto 0));
            v_b := a_bxd;
            if v_b < 0 then
                v_b := -v_b;
            end if;
            p1_abx  <= unsigned(v_b(22 downto 0));
            p1_chev <= unsigned(a_bxd(15 downto 0)) + px_cl;
            if a_g < 768 then
                p1_gl <= '1';
            else
                p1_gl <= '0';
            end if;

            -- S2: normalize, steps of 16 and 8; lens tests
            v_v  := p1_s;
            v_sh := (others => '0');
            if v_v(25 downto 10) = 0 then
                v_v  := v_v(9 downto 0) & x"0000";
                v_sh := v_sh + 16;
            end if;
            if v_v(25 downto 18) = 0 then
                v_v  := v_v(17 downto 0) & x"00";
                v_sh := v_sh + 8;
            end if;
            p2_v  <= v_v;
            p2_sh <= v_sh;
            if px_vin = '1' and p1_ax <= ("00" & px_hwin) then
                v_in := '1';
            else
                v_in := '0';
            end if;
            p2_in <= v_in;
            if v_in = '0' and px_vout = '1' and p1_ax <= ("00" & px_hwout) then
                p2_ring <= '1';
            else
                p2_ring <= '0';
            end if;
            v_r := signed('0' & resize(p1_abx, 24)) - resize(px_lim, 25);
            p2_india <= v_r(24);
            p2_rimd  <= v_r(23 downto 0);
            p2_sent  <= p1_chev(15 downto 11);
            p2_gl    <= p1_gl;

            -- S3: steps of 4, 2, 1 (leading one at bit 25); lens flags
            v_v  := p2_v;
            v_sh := p2_sh;
            if v_v(25 downto 22) = 0 then
                v_v  := v_v(21 downto 0) & x"0";
                v_sh := v_sh + 4;
            end if;
            if v_v(25 downto 24) = 0 then
                v_v  := v_v(23 downto 0) & "00";
                v_sh := v_sh + 2;
            end if;
            if v_v(25) = '0' then
                v_v  := v_v(24 downto 0) & '0';
                v_sh := v_sh + 1;
            end if;
            p3_v  <= v_v;
            p3_sh <= v_sh;
            v_st := (not c_win) and p2_in and p2_india;
            if c_win = '0' and p2_in = '1' and p2_india = '0' and p2_rimd(23 downto 18) = 0
               and unsigned(p2_rimd(17 downto 0)) < c_rimw and p2_rimd(14 downto 13) = "00" then
                v_rim := '1';
            else
                v_rim := '0';
            end if;
            fl_wd <= "000000" & v_st & (p2_in and not v_st) & v_rim & p2_ring
                     & (c_ghost and p2_gl and not p2_in and not p2_ring)
                     & std_logic_vector(p2_sent);
            nt(3) <= c_win and p2_in;
            for i in 4 to 9 loop
                nt(i) <= nt(i - 1);
            end loop;

            -- S4: mantissa ROM address {L(0), m}, L = 25 - sh; output
            -- shift r = sh / 2
            p4_a <= (not p3_sh(0)) & p3_v(24 downto 15);
            p4_r <= p3_sh(4 downto 1);

            -- S5: the square-root ROM (block RAM)
            r5_v <= C_SQRT(to_integer(p4_a));
            p5_r <= p4_r;

            -- S6: into fabric: (2048 + v) << 1
            p6_v <= '1' & unsigned(r5_v) & '0';
            p6_r <= p5_r;

            -- S7/S8: d = p6 >> r
            case to_integer(p6_r(3 downto 2)) is
                when 0      => p7_v <= p6_v;
                when 1      => p7_v <= shift_right(p6_v, 4);
                when 2      => p7_v <= shift_right(p6_v, 8);
                when others => p7_v <= shift_right(p6_v, 12);
            end case;
            p7_r <= p6_r(1 downto 0);
            p8_d <= shift_right(p7_v, to_integer(p7_r));

            -- S9: tear onset
            p9_e <= signed('0' & p8_d) - signed("000" & c_d0);
            p9_d <= p8_d;

            -- S10: the tear ramp: 0 below the onset (and in the Window),
            -- quarters over 128 units
            if p9_e < 0 or nt(9) = '1' then
                p10_nr <= (others => '0');
            elsif p9_e >= 128 then
                p10_nr <= px_n;
            else
                case to_integer(p9_e(6 downto 5)) is
                    when 0      => p10_nr <= shift_right(px_n, 2);
                    when 1      => p10_nr <= shift_right(px_n, 1);
                    when others => p10_nr <= px_n34;
                end case;
            end if;
            p10_d <= p9_d;

            -- S11: torn distance
            p11_dt <= signed("00" & p10_d) + resize(p10_nr, 15);

            -- S12: x Contours, operand split
            p12_h <= p11_dt * signed("0" & c_G16(10 downto 6));
            p12_l <= p11_dt * signed("0" & c_G16(5 downto 0));

            -- S13
            p13_s <= shift_left(resize(p12_h, 27), 6) + resize(p12_l, 27);

            -- S14: index (16 bits per palette cycle) - flow
            p14_i <= unsigned(p13_s(19 downto 4)) - c_tP;

            -- S15: 256 -> 224 entries
            v_e   := p14_i(15 downto 8);
            p15_e <= v_e - shift_right(v_e, 3);

            -- S16: palette address: field entry or chevron stripe
            if fl15(9) = '1' then
                p16_a <= c_pal & "111" & unsigned(fl15(4 downto 0));
            else
                p16_a <= c_pal & p15_e;
            end if;

            -- S17: palette ROMs (block RAM)
            r17_y <= C_PY(to_integer(p16_a));
            r17_u <= C_PU(to_integer(p16_a));
            r17_v <= C_PV(to_integer(p16_a));

            -- S18: into fabric, 10-bit
            t18_y <= unsigned(r17_y) & "00";
            t18_u <= unsigned(r17_u) & "00";
            t18_v <= unsigned(r17_v) & "00";

            -- S19: blend targets: ring > rim line > glass > ghost
            p19_y <= t18_y;
            p19_u <= t18_u;
            p19_v <= t18_v;
            if fld(18)(1) = '1' then                     -- ring: half to grey
                p19_dy <= to_signed(560, 11) - signed('0' & t18_y);
                p19_du <= to_signed(512, 11) - signed('0' & t18_u);
                p19_dv <= to_signed(512, 11) - signed('0' & t18_v);
                p19_sh <= "01";
            elsif fld(18)(2) = '1' then                  -- rim lines: half to olive
                p19_dy <= to_signed(440, 11) - signed('0' & t18_y);
                p19_du <= to_signed(538, 11) - signed('0' & t18_u);
                p19_dv <= to_signed(454, 11) - signed('0' & t18_v);
                p19_sh <= "01";
            elsif fld(18)(3) = '1' then                  -- glass / smoke
                if c_smoke = '1' then
                    p19_dy <= to_signed(110, 11) - signed('0' & t18_y);
                else
                    p19_dy <= to_signed(920, 11) - signed('0' & t18_y);
                end if;
                p19_du <= to_signed(512, 11) - signed('0' & t18_u);
                p19_dv <= to_signed(512, 11) - signed('0' & t18_v);
                if c_win = '1' then
                    p19_sh <= "10";
                else
                    p19_sh <= "01";
                end if;
            elsif fld(18)(0) = '1' then                  -- ghost: a quarter to white
                p19_dy <= to_signed(940, 11) - signed('0' & t18_y);
                p19_du <= to_signed(512, 11) - signed('0' & t18_u);
                p19_dv <= to_signed(512, 11) - signed('0' & t18_v);
                p19_sh <= "10";
            else
                p19_dy <= (others => '0');
                p19_du <= (others => '0');
                p19_dv <= (others => '0');
                p19_sh <= "00";
            end if;

            -- S20
            case to_integer(p19_sh) is
                when 1 =>
                    v_oy := signed('0' & p19_y) + shift_right(p19_dy, 1);
                    v_ou := signed('0' & p19_u) + shift_right(p19_du, 1);
                    v_ov := signed('0' & p19_v) + shift_right(p19_dv, 1);
                when 2 =>
                    v_oy := signed('0' & p19_y) + shift_right(p19_dy, 2);
                    v_ou := signed('0' & p19_u) + shift_right(p19_du, 2);
                    v_ov := signed('0' & p19_v) + shift_right(p19_dv, 2);
                when others =>
                    v_oy := signed('0' & p19_y);
                    v_ou := signed('0' & p19_u);
                    v_ov := signed('0' & p19_v);
            end case;
            p20_y <= unsigned(v_oy(9 downto 0));
            p20_u <= unsigned(v_ou(9 downto 0));
            p20_v <= unsigned(v_ov(9 downto 0));
        end if;
    end process p_pix;

    -- ====================================================================
    -- The lens flags' delay ring: written from S3, read 10 clocks later
    -- (the read register holds S14), re-registered at S15; the blend flags
    -- then ride three FFs to S18.  The read never meets the write address.
    -- ====================================================================
    p_flring : process(clk)
    begin
        if rising_edge(clk) then
            fl_ram(to_integer(fl_wp)) <= fl_wd;
            fl_rd <= fl_ram(to_integer(fl_rp));
            fl_wp <= fl_wp + 1;
            fl_rp <= fl_rp + 1;
        end if;
    end process p_flring;

    p_fl : process(clk)
    begin
        if rising_edge(clk) then
            fl15    <= fl_rd(9 downto 0);
            fld(16) <= fl15(9 downto 5);
            fld(17) <= fld(16);
            fld(18) <= fld(17);
        end if;
    end process p_fl;

    -- ====================================================================
    -- S21..S23: output conditioning and the blanking gate
    -- ====================================================================
    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S21/S22: horizontal [1 2 1]/4 (g1 is the centre pixel)
            g1_y <= p20_y;  g2_y <= g1_y;
            g1_u <= p20_u;  g2_u <= g1_u;
            g1_v <= p20_v;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(p20_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(p20_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(p20_v, 12);
            p22_y <= v_sy(11 downto 2);
            p22_u <= v_su(11 downto 2);
            p22_v <= v_sv(11 downto 2);

            -- S23: luma 64 + 13/16 (y - 64) (white ~775); neutral outside
            -- the generated avid
            v_d  := p22_y - 64;
            v_yo := p22_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if sd(22)(3) = '0' then
                s_out_y <= C_BLK;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo;
                s_out_u <= p22_u;
                s_out_v <= p22_v;
            end if;

            sd(1) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            for k in 2 to 23 loop
                sd(k) <= sd(k - 1);
            end loop;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= sd(23)(3);
    data_out.hsync_n <= sd(23)(2);
    data_out.vsync_n <= sd(23)(1);
    data_out.field_n <= sd(23)(0);

end architecture loupe;
