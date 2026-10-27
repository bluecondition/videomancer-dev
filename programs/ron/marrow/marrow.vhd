-- marrow.vhd  (v0.1.0) -- a spine of contoured blobs in signal yellow
--
-- MARROW.  An analog colourizer patch rebuilt in one pass, made from a
-- reference capture: a column of rounded rings stacked down the middle of
-- the frame, side lobes breaking off into islands, everything drawn as
-- false-colour contours (dark-red outlines, azure halos, peach and orange
-- interiors) in a flood of flat signal yellow, smeared sideways and torn
-- line by line.  Synthesis: no input video is read.
--
-- PICTURE
--  1. FIELD.  Two cosine oscillators, one across (period W/N) and one down
--     (period H/N), summed into an egg-crate lattice of hills and valleys,
--     plus a diagonal third wave.  Warp displaces the down-oscillator by a
--     slower wave across, the diagonal wave's weight, and (per line) the
--     across-oscillator's phase: a regular lattice melts into blobs.
--     Mirror reflects every across-phase about the centre (|x - c|).
--  2. FOLD.  G = Depth . egg + Level is folded by a triangle (|G| for
--     |G| < 1, then back down): where the field crosses zero a thin
--     outline is drawn, and deep valleys come back up as rings.  Depth
--     (P12) is how many folds: a few soft contours at 0, dense op-art
--     rings at 100 %.  Breathe swings Level slowly: rings swell and shrink.
--  3. ENVELOPE.  index = fold + slope . max(0, |x - c| - width(y)): outside
--     a centred column the index climbs to the palette's clip (yellow), so
--     only the deepest valleys poke through as islands.  width(y) is waisted
--     by a slow vertical wave; Spread sets it.
--  4. SMEAR.  A one-pole lag runs left to right over the index (so the
--     colours trail through the palette's own order), Drip makes rising
--     edges 8x slower than falling ones (long trails out of every blob).
--  5. TEAR.  Each line is shifted by a random offset, held for runs of
--     lines and boosted in bursts (+-64 px HD): through the smear it draws
--     the streaks of the reference.
--  6. PALETTE.  Eight 256-entry palettes; entry 0 = the fold, 255 = the clip.
--
-- ENGINE
--   A small sequencer with one serial multiplier (op-code operand fetch,
--   totem's fix: states write only an op code) and one serial divider.
--   Per field (first vsync edge after active video): 2^24/W, 2^20/H, the
--   control curves, oscillator steps, the centre start phases, speed, the
--   Breathe level.  Per active line (kicked at the line's start, computing
--   the NEXT line, ~120 clocks): the vertical phases, the wiggle and waist
--   LFOs (sine ROM + multiply), the tear offset, and the four accumulator
--   start values -(c + j) . k.
--
-- CONTROLS
--   K1 Cells    2 .. 16 cells across and down (square law)
--   K2 Warp     lattice -> melted blobs (warp, wiggle, diagonal weight)
--   K3 Smear    lag 2 .. 256 px (exponential, dithered)
--   K4 Speed    bipolar drift, still in the centre (positive = blobs rise)
--   K5 Spread   width of the column before the yellow floods in
--   K6 Palette  Marrow / Abyss / Thermal / Acid / Lagoon / Ink / Candy / Venom
--   S7 Mirror   reflect the lattice about the centre (the spine)
--   S8 Tear     per-line tearing, held runs and bursts
--   S9 Drip     rising edges 8x slower: trails out of every blob
--   S10 Shape   Round (cosines) / Diamond (triangles)
--   S11 Breathe slow swing of the fold level (rings swell and shrink)
--   P12 Depth   fold depth: soft contours at 0, dense rings at 100 %
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, fixed
-- width); one latency for all modes (21 stages); outputs gated to neutral
-- outside the generated avid; palettes stored U/V-swapped (marrow.py);
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
use work.marrow_pkg.all;

architecture marrow of program_top is

    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- one-hot lag shift 0..12
    function f_onehot(s : unsigned(3 downto 0)) return std_logic_vector is
        variable v_o : std_logic_vector(12 downto 0) := (others => '0');
    begin
        for i in 0 to 12 loop
            if to_integer(s) = i then
                v_o(i) := '1';
            end if;
        end loop;
        return v_o;
    end function;

    -- x >> s for a one-hot s
    function f_shr(x : unsigned(15 downto 0); oh : std_logic_vector(12 downto 0)) return unsigned is
        variable v_r : unsigned(15 downto 0) := (others => '0');
    begin
        for i in 0 to 12 loop
            if oh(i) = '1' then
                v_r := v_r or shift_right(x, i);
            end if;
        end loop;
        return v_r;
    end function;

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
    -- knob glides: 0 K1 Cells, 1 K2 Warp, 2 K3 Smear, 3 K5 Spread,
    -- 4 P12 Depth.  Deadband hold + per-field ease, 10.4
    ------------------------------------------------------------------------
    signal gl_raw : unsigned(49 downto 0) := (others => '0');
    signal gl_o   : unsigned(69 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- sequencer
    ------------------------------------------------------------------------
    signal q_st : unsigned(5 downto 0) := (others => '0');
    attribute fsm_encoding : string;
    attribute fsm_encoding of q_st : signal is "none";
    signal q_fpend, q_lpend, q_prime : std_logic := '0';

    -- the sequencer's sine ROM (address -> data 1 clock -> fabric 1 clock)
    signal q_sa : unsigned(7 downto 0) := (others => '0');
    signal q_sd : std_logic_vector(7 downto 0) := (others => '0');
    signal q_sr : signed(7 downto 0) := (others => '0');

    -- serial multiplier: the issuing state writes an op code; the operands
    -- are fetched into o_* (one registered mux), then loaded
    signal mul_go, mul_busy, m_ld : std_logic := '0';
    signal mul_op : unsigned(3 downto 0) := (others => '0');
    signal o_a    : signed(23 downto 0) := (others => '0');
    signal o_b    : unsigned(15 downto 0) := (others => '0');
    signal o_n    : unsigned(4 downto 0) := (others => '0');
    signal m_acc, m_a : signed(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');

    -- serial divider: q = 2^(k-1) / den (restoring)
    signal div_go, div_busy : std_logic := '0';
    signal div_k   : unsigned(4 downto 0) := (others => '0');
    signal div_den : unsigned(11 downto 0) := (others => '0');
    signal d_rem   : unsigned(11 downto 0) := (others => '0');
    signal d_q     : unsigned(24 downto 0) := (others => '0');
    signal d_cnt   : unsigned(4 downto 0) := (others => '0');
    signal d_first : std_logic := '0';

    -- per-field values
    signal c_k1, c_k2, c_k3, c_k5, c_p12, c_k4 : unsigned(9 downto 0) := (others => '0');
    signal c_pal  : unsigned(2 downto 0) := (others => '0');
    signal c_mir, c_tear, c_drip, c_shape, c_brth : std_logic := '0';
    signal c_jm   : std_logic := '1';                                 -- HD-width tear
    signal c_W2   : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal c_recw : unsigned(15 downto 0) := to_unsigned(8738, 16);   -- 2^24 / W
    signal c_dy   : unsigned(12 downto 0) := to_unsigned(970, 13);    -- 2^20 / field lines
    signal c_c2   : unsigned(19 downto 0) := (others => '0');
    signal c_c7   : unsigned(23 downto 0) := (others => '0');
    signal c_cols : unsigned(12 downto 0) := to_unsigned(1484, 13);   -- cells x 256
    signal c_kx, c_kv, c_kdx : unsigned(20 downto 0) := (others => '0');  -- cycles/px x 2^24
    signal c_ky, c_kdy : unsigned(20 downto 0) := (others => '0');        -- cycles/line x 2^24
    signal c_ke   : unsigned(10 downto 0) := to_unsigned(409, 11);    -- index/px x 2^8
    signal c_kwl, c_kel : unsigned(17 downto 0) := (others => '0');   -- LFO steps per line
    signal c_ncx, c_ncv, c_ncd : signed(29 downto 0) := (others => '0');  -- -c . k
    signal c_nce  : signed(21 downto 0) := (others => '0');
    signal c_A    : unsigned(9 downto 0) := to_unsigned(182, 10);     -- Depth x 64
    signal c_dir  : std_logic := '0';
    signal c_mag  : unsigned(8 downto 0) := (others => '0');
    signal c_spd  : unsigned(19 downto 0) := (others => '0');
    signal c_tY, c_tV, c_tW, c_tB : unsigned(23 downto 0) := (others => '0');
    signal c_tD   : unsigned(23 downto 0) := x"400000";
    signal c_tE   : unsigned(23 downto 0) := x"C00000";
    signal c_wd0, c_wdm : unsigned(11 downto 0) := (others => '0');
    signal c_wA   : unsigned(7 downto 0) := (others => '0');
    signal c_awig : unsigned(14 downto 0) := (others => '0');
    signal c_smv  : unsigned(7 downto 0) := to_unsigned(77, 8);
    signal c_ohf0, c_ohf1, c_ohr0, c_ohr1 : std_logic_vector(12 downto 0) := "0000000010000";
    signal c_sfr  : unsigned(3 downto 0) := (others => '0');
    signal c_off  : signed(11 downto 0) := to_signed(100, 12);

    -- per-line engine
    signal l_py, l_pyd, l_pw, l_pe : unsigned(23 downto 0) := (others => '0');
    signal l_wig  : signed(15 downto 0) := (others => '0');
    signal l_wd   : unsigned(11 downto 0) := (others => '0');
    signal l_sp   : unsigned(7 downto 0) := (others => '0');
    signal l_rng  : unsigned(31 downto 0) := x"2545F491";
    signal l_bc   : unsigned(5 downto 0) := (others => '0');
    signal l_bl   : unsigned(1 downto 0) := (others => '0');
    signal l_j    : signed(7 downto 0) := (others => '0');
    signal l_ja   : unsigned(6 downto 0) := (others => '0');
    signal l_jn   : std_logic := '0';
    signal l_sx, l_sv, l_sd : signed(29 downto 0) := (others => '0');
    signal l_se   : signed(21 downto 0) := (others => '0');
    -- handed to the pixel path at g_go
    signal n_py, n_pyd : unsigned(15 downto 0) := (others => '0');
    signal n_wig  : signed(15 downto 0) := (others => '0');
    signal n_wd   : unsigned(11 downto 0) := (others => '0');
    signal n_sx, n_sv, n_sd : signed(29 downto 0) := (others => '0');
    signal n_se   : signed(21 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pixel path
    ------------------------------------------------------------------------
    -- S0: accumulators (x 2^24 per cycle; the envelope in index x 2^8)
    signal a_x, a_v, a_d : signed(29 downto 0) := (others => '0');
    signal a_e    : signed(21 downto 0) := (others => '0');
    signal px_py, px_pyd : unsigned(15 downto 0) := (others => '0');
    signal px_wig : signed(15 downto 0) := (others => '0');
    signal px_wd  : unsigned(11 downto 0) := (others => '0');
    signal px_x4  : unsigned(3 downto 0) := (others => '0');
    signal px_ls  : std_logic := '0';

    -- {line start, x4 dither phase} ride along S1..S14
    type t_xl is array(1 to 14) of std_logic_vector(4 downto 0);
    signal xl : t_xl := (others => (others => '0'));

    signal p1_px, p1_pv, p1_pd : unsigned(15 downto 0) := (others => '0');
    signal p1_ue  : unsigned(12 downto 0) := (others => '0');
    signal p2_ax, p2_av, p2_ad : unsigned(8 downto 0) := (others => '0');
    signal p2_e   : signed(13 downto 0) := (others => '0');
    signal r3_x, r3_v, r3_d : std_logic_vector(7 downto 0) := (others => '0');
    signal p3_env, p4_env, p5_env, p6_env, p7_env, p8_env : unsigned(9 downto 0) := (others => '0');
    signal p9_env, p10_env, p11_env : unsigned(9 downto 0) := (others => '0');
    signal s4_x, s4_v, s4_d : signed(7 downto 0) := (others => '0');
    signal p5_x   : signed(7 downto 0) := (others => '0');
    signal m5_v, m5_d : signed(16 downto 0) := (others => '0');
    signal p6_ay  : unsigned(8 downto 0) := (others => '0');
    signal p6_xd, p7_xd, p8_xd : signed(9 downto 0) := (others => '0');
    signal r7_y   : std_logic_vector(7 downto 0) := (others => '0');
    signal s8_y   : signed(7 downto 0) := (others => '0');
    signal p9_egg : signed(9 downto 0) := (others => '0');
    signal m10    : signed(20 downto 0) := (others => '0');
    signal p11_g  : signed(14 downto 0) := (others => '0');
    signal p12_idx : unsigned(9 downto 0) := (others => '0');
    signal p13_i, p14_i, p14_p : unsigned(15 downto 0) := (others => '0');
    signal p13_oh, p14_oh : std_logic_vector(12 downto 0) := (others => '0');
    signal lag_l  : unsigned(15 downto 0) := (others => '0');
    signal p16_a  : unsigned(10 downto 0) := (others => '0');
    signal r17_y, r17_u, r17_v : std_logic_vector(7 downto 0) := (others => '0');
    signal t18_y, t18_u, t18_v : unsigned(9 downto 0) := (others => '0');
    -- output conditioning (the HD HDMI bar fix, damier v0.4.4): [1 2 1]
    -- horizontal smoothing on Y/U/V, luma scaled toward black
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal p20_y, p20_u, p20_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- syncs (generated avid, hsync, vsync, field) from S1 to S21
    type t_sd is array(1 to 21) of std_logic_vector(3 downto 0);
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
    -- Knob glides: the hold follows the pot only beyond a +-4 deadband
    -- (snapping to the rails), the output eases toward it once per field
    -- (by 1/8; the Depth slider by 1/4 so the gesture stays quick)
    -- ====================================================================
    gl_raw <= unsigned(registers_in(7)) & unsigned(registers_in(4))
              & unsigned(registers_in(2)) & unsigned(registers_in(1))
              & unsigned(registers_in(0));

    g_glide : for i in 0 to 4 generate
        signal b_h : unsigned(9 downto 0) := (others => '0');
        signal b_o : unsigned(13 downto 0) := (others => '0');
    begin
        p_glide : process(clk)
            variable v_r : unsigned(9 downto 0);
            variable v_e : signed(11 downto 0);
            variable v_t : unsigned(13 downto 0);
            variable v_d : signed(15 downto 0);
            variable v_s : signed(15 downto 0);
        begin
            if rising_edge(clk) then
                v_r := gl_raw(10 * i + 9 downto 10 * i);
                v_e := signed(resize(v_r, 12)) - signed(resize(b_h, 12));
                if v_r < 4 then
                    b_h <= (others => '0');
                elsif v_r > 1019 then
                    b_h <= (others => '1');
                elsif v_e >= 4 or v_e <= -4 then
                    b_h <= v_r;
                end if;

                if s_vs_go = '1' then
                    v_t := b_h & "0000";
                    v_d := signed(resize(v_t, 16)) - signed(resize(b_o, 16));
                    if v_d < 8 and v_d > -8 then
                        b_o <= v_t;
                    else
                        if i = 4 then
                            v_s := signed(resize(b_o, 16)) + shift_right(v_d, 2);
                        else
                            v_s := signed(resize(b_o, 16)) + shift_right(v_d, 3);
                        end if;
                        b_o <= unsigned(v_s(13 downto 0));
                    end if;
                end if;
            end if;
        end process p_glide;
        gl_o(14 * i + 13 downto 14 * i) <= b_o;
    end generate g_glide;

    -- ====================================================================
    -- The sequencer's sine ROM: registered read, then a fabric register
    -- ====================================================================
    p_qsin : process(clk)
    begin
        if rising_edge(clk) then
            q_sd <= C_SIN(to_integer(q_sa));
            q_sr <= signed(q_sd);
        end if;
    end process p_qsin;

    -- ====================================================================
    -- Serial multiplier (shift-add, one bit of b per clock).  go -> operand
    -- fetch by op code (registered) -> load -> n iterations.
    -- ====================================================================
    p_mul : process(clk)
    begin
        if rising_edge(clk) then
            m_ld <= '0';
            if mul_go = '1' then
                case to_integer(mul_op) is
                    when 0 =>       -- K1^2
                        o_a <= signed(resize(c_k1, 24));
                        o_b <= resize(c_k1, 16);
                        o_n <= to_unsigned(10, 5);
                    when 1 =>       -- kx = cells x 2^24 / W (x 256)
                        o_a <= signed(resize(c_recw, 24));
                        o_b <= resize(c_cols, 16);
                        o_n <= to_unsigned(13, 5);
                    when 2 =>       -- ky = cells x 2^20 / H (x 256)
                        o_a <= signed(resize(c_dy, 24));
                        o_b <= resize(c_cols, 16);
                        o_n <= to_unsigned(13, 5);
                    when 3 =>       -- c . kx
                        o_a <= signed(resize(c_kx, 24));
                        o_b <= resize(c_W2, 16);
                        o_n <= to_unsigned(11, 5);
                    when 4 =>
                        o_a <= signed(resize(c_kv, 24));
                        o_b <= resize(c_W2, 16);
                        o_n <= to_unsigned(11, 5);
                    when 5 =>
                        o_a <= signed(resize(c_kdx, 24));
                        o_b <= resize(c_W2, 16);
                        o_n <= to_unsigned(11, 5);
                    when 6 =>
                        o_a <= signed(resize(c_ke, 24));
                        o_b <= resize(c_W2, 16);
                        o_n <= to_unsigned(11, 5);
                    when 7 =>       -- P12^2
                        o_a <= signed(resize(c_p12, 24));
                        o_b <= resize(c_p12, 16);
                        o_n <= to_unsigned(10, 5);
                    when 8 =>       -- speed^2
                        o_a <= signed(resize(c_mag, 24));
                        o_b <= resize(c_mag, 16);
                        o_n <= to_unsigned(9, 5);
                    when 9 =>       -- Breathe swing
                        o_a <= resize(q_sr, 24);
                        o_b <= to_unsigned(200, 16);
                        o_n <= to_unsigned(8, 5);
                    when 10 =>      -- wiggle
                        o_a <= resize(q_sr, 24);
                        o_b <= resize(c_awig, 16);
                        o_n <= to_unsigned(15, 5);
                    when 11 =>      -- waist
                        o_a <= signed(resize(c_wdm, 24));
                        o_b <= resize(l_sp, 16);
                        o_n <= to_unsigned(8, 5);
                    when 12 =>      -- |j| . k
                        o_a <= signed(resize(c_kx, 24));
                        o_b <= resize(l_ja, 16);
                        o_n <= to_unsigned(7, 5);
                    when 13 =>
                        o_a <= signed(resize(c_kv, 24));
                        o_b <= resize(l_ja, 16);
                        o_n <= to_unsigned(7, 5);
                    when 14 =>
                        o_a <= signed(resize(c_kdx, 24));
                        o_b <= resize(l_ja, 16);
                        o_n <= to_unsigned(7, 5);
                    when others =>
                        o_a <= signed(resize(c_ke, 24));
                        o_b <= resize(l_ja, 16);
                        o_n <= to_unsigned(7, 5);
                end case;
                m_ld     <= '1';
                mul_busy <= '1';
            elsif m_ld = '1' then
                m_acc <= (others => '0');
                m_a   <= resize(o_a, 40);
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
    -- Sequencer: per-field set-up (states 1..31) and the per-line engine
    -- (32..49).  One small operation per state.  A unit is free when its
    -- go is low and busy is low (go is registered, busy follows one clock
    -- later), so a wait state can follow its issue state directly.
    -- ====================================================================
    p_seq : process(clk)
        variable v_k6 : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_ad : unsigned(9 downto 0);
        variable v_s  : unsigned(3 downto 0);
        variable v_r  : unsigned(3 downto 0);
        variable v_jr : signed(7 downto 0);
        variable v_m  : signed(29 downto 0);
    begin
        if rising_edge(clk) then
            mul_go <= '0';
            div_go <= '0';
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
                        q_st    <= to_unsigned(1, 6);
                    elsif q_lpend = '1' then
                        q_lpend <= '0';
                        q_st    <= to_unsigned(32, 6);
                    end if;

                ------------------------------------------------------------
                -- per field
                ------------------------------------------------------------
                when 1 =>       -- latch the controls; start 2^24 / W
                    c_k1  <= gl_o(13 downto 4);
                    c_k2  <= gl_o(27 downto 18);
                    c_k3  <= gl_o(41 downto 32);
                    c_k5  <= gl_o(55 downto 46);
                    c_p12 <= gl_o(69 downto 60);
                    c_k4  <= unsigned(registers_in(3));
                    -- labelled knob: 0..7 at program load is the label index
                    v_k6 := unsigned(registers_in(5));
                    if v_k6 < 8 then
                        c_pal <= v_k6(2 downto 0);
                    else
                        c_pal <= v_k6(9 downto 7);
                    end if;
                    c_mir   <= registers_in(6)(0);
                    c_tear  <= registers_in(6)(1);
                    c_drip  <= registers_in(6)(2);
                    c_shape <= registers_in(6)(3);
                    c_brth  <= registers_in(6)(4);
                    c_W2    <= s_meas_w(11 downto 1);
                    c_jm    <= s_meas_w(11) or s_meas_w(10);
                    div_den <= s_meas_w;
                    div_k   <= to_unsigned(25, 5);
                    div_go  <= '1';
                    q_st    <= to_unsigned(2, 6);

                when 2 =>       -- W reciprocal; start 2^20 / H
                    if div_go = '0' and div_busy = '0' then
                        if d_q(24 downto 16) /= 0 then
                            c_recw <= (others => '1');
                        else
                            c_recw <= d_q(15 downto 0);
                        end if;
                        div_den <= resize(s_meas_h, 12);
                        div_k   <= to_unsigned(21, 5);
                        div_go  <= '1';
                        q_st    <= to_unsigned(3, 6);
                    end if;

                when 3 =>       -- line step; Cells squared
                    if div_go = '0' and div_busy = '0' then
                        if d_q(24 downto 13) /= 0 then
                            c_dy <= (others => '1');
                        else
                            c_dy <= d_q(12 downto 0);
                        end if;
                        mul_op <= to_unsigned(0, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(4, 6);
                    end if;

                when 4 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_c2 <= unsigned(m_acc(19 downto 0));
                        q_st <= to_unsigned(5, 6);
                    end if;

                when 5 =>       -- 7 k1^2
                    c_c7 <= shift_left(resize(c_c2, 24), 3) - resize(c_c2, 24);
                    q_st <= to_unsigned(6, 6);

                when 6 =>       -- cells x 256 = 512 + 7 k1^2 / 2048 (2 .. 16)
                    c_cols <= to_unsigned(512, 13) + resize(c_c7(22 downto 11), 13);
                    q_st   <= to_unsigned(7, 6);

                when 7 =>       -- kx
                    mul_op <= to_unsigned(1, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(8, 6);

                when 8 =>       -- ky
                    if mul_go = '0' and mul_busy = '0' then
                        c_kx   <= unsigned(m_acc(28 downto 8));
                        mul_op <= to_unsigned(2, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(9, 6);
                    end if;

                when 9 =>       -- warp-x 5/8 and diagonal 3/4 of kx
                    if mul_go = '0' and mul_busy = '0' then
                        c_ky  <= unsigned(m_acc(24 downto 4));
                        c_kv  <= ('0' & c_kx(20 downto 1)) + ("000" & c_kx(20 downto 3));
                        c_kdx <= ('0' & c_kx(20 downto 1)) + ("00" & c_kx(20 downto 2));
                        q_st  <= to_unsigned(10, 6);
                    end if;

                when 10 =>      -- diagonal 3/4 of ky; envelope slope 1.5 spans per
                                -- half width = 3 . 2^18 / W; LFO steps
                    c_kdy <= ('0' & c_ky(20 downto 1)) + ("00" & c_ky(20 downto 2));
                    c_ke  <= resize(shift_right(resize(c_recw, 18)
                                                + shift_left(resize(c_recw, 18), 1), 6), 11);
                    c_kwl <= shift_left(resize(c_dy, 18), 4) + shift_left(resize(c_dy, 18), 3)
                             + resize(c_dy, 18);                      -- 1.56 cycles / frame
                    c_kel <= shift_left(resize(c_dy, 18), 3) + shift_left(resize(c_dy, 18), 2)
                             + resize(c_dy, 18);                      -- 0.81 cycles / frame
                    mul_op <= to_unsigned(3, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(11, 6);

                when 11 =>      -- -c . kx
                    if mul_go = '0' and mul_busy = '0' then
                        c_ncx  <= -m_acc(29 downto 0);
                        mul_op <= to_unsigned(4, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(12, 6);
                    end if;

                when 12 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_ncv  <= -m_acc(29 downto 0);
                        mul_op <= to_unsigned(5, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(13, 6);
                    end if;

                when 13 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_ncd  <= -m_acc(29 downto 0);
                        mul_op <= to_unsigned(6, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(14, 6);
                    end if;

                when 14 =>      -- Depth squared
                    if mul_go = '0' and mul_busy = '0' then
                        c_nce  <= -m_acc(21 downto 0);
                        mul_op <= to_unsigned(7, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(15, 6);
                    end if;

                when 15 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_c2 <= unsigned(m_acc(19 downto 0));
                        q_st <= to_unsigned(16, 6);
                    end if;

                when 16 =>      -- 11 p12^2
                    c_c7 <= shift_left(resize(c_c2, 24), 3) + shift_left(resize(c_c2, 24), 1)
                            + resize(c_c2, 24);
                    q_st <= to_unsigned(17, 6);

                when 17 =>      -- Depth x 64 = 64 + 11 p12^2 / 16384 (1.0 .. 12.0)
                    c_A  <= to_unsigned(64, 10) + resize(c_c7(23 downto 14), 10);
                    q_st <= to_unsigned(18, 6);

                when 18 =>      -- Speed: bipolar with a +-24 still zone
                    v_d := signed(resize(c_k4, 11)) - 512;
                    if v_d < 0 then
                        c_dir <= '1';
                        v_ad  := unsigned(resize(-v_d, 10));
                    else
                        c_dir <= '0';
                        v_ad  := unsigned(resize(v_d, 10));
                    end if;
                    if v_ad <= 24 then
                        c_mag <= (others => '0');
                    else
                        c_mag <= resize(v_ad - 24, 9);
                    end if;
                    q_st <= to_unsigned(19, 6);

                when 19 =>
                    mul_op <= to_unsigned(8, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(20, 6);

                when 20 =>      -- speed = 4 mag^2 (up to ~3.4 cells/s)
                    if mul_go = '0' and mul_busy = '0' then
                        c_spd <= unsigned(m_acc(17 downto 0)) & "00";
                        q_st  <= to_unsigned(21, 6);
                    end if;

                when 21 =>      -- positive speed: the blobs rise, the warp and
                                -- the diagonal drift against each other
                    if c_dir = '0' then
                        c_tY <= c_tY + resize(c_spd, 24);
                        c_tV <= c_tV + resize(c_spd(19 downto 1), 24) + resize(c_spd(19 downto 2), 24);
                        c_tD <= c_tD - resize(c_spd, 24) - resize(c_spd(19 downto 2), 24);
                        c_tW <= c_tW + resize(c_spd(19 downto 1), 24);
                        c_tE <= c_tE - resize(c_spd(19 downto 2), 24);
                    else
                        c_tY <= c_tY - resize(c_spd, 24);
                        c_tV <= c_tV - resize(c_spd(19 downto 1), 24) - resize(c_spd(19 downto 2), 24);
                        c_tD <= c_tD + resize(c_spd, 24) + resize(c_spd(19 downto 2), 24);
                        c_tW <= c_tW - resize(c_spd(19 downto 1), 24);
                        c_tE <= c_tE + resize(c_spd(19 downto 2), 24);
                    end if;
                    c_tB <= c_tB + to_unsigned(46603, 24);   -- ~6 s per breath
                    q_st <= to_unsigned(22, 6);

                when 22 =>      -- Spread: column width 0.08 .. 1.3 half-widths (in
                                -- index units at the envelope slope); Warp
                    c_wd0  <= to_unsigned(120, 12) + resize(c_k5, 12)
                              + resize(c_k5(9 downto 1), 12) + resize(c_k5(9 downto 2), 12)
                              + resize(c_k5(9 downto 4), 12);
                    c_wA   <= c_k2(9 downto 2);
                    c_awig <= shift_left(resize(c_k2, 15), 4) + shift_left(resize(c_k2, 15), 1)
                              + resize(c_k2, 15);              -- wiggle up to 0.3 cycle
                    q_st   <= to_unsigned(23, 6);

                when 23 =>      -- waist depth 0.84; Smear s = 1 .. 7.9 (4.4)
                    c_wdm <= c_wd0 - resize(c_wd0(11 downto 3), 12) - resize(c_wd0(11 downto 5), 12);
                    c_smv <= to_unsigned(16, 8) + resize(shift_right(shift_left(resize(c_k3, 13), 3)
                                                                     - resize(c_k3, 13), 6), 8);
                    q_st  <= to_unsigned(24, 6);

                when 24 =>      -- lag one-hots: fall s / s+1, rise (Drip: +3)
                    v_s := c_smv(7 downto 4);
                    if c_drip = '1' then
                        v_r := v_s + 3;
                    else
                        v_r := v_s;
                    end if;
                    c_ohf0 <= f_onehot(v_s);
                    c_ohf1 <= f_onehot(v_s + 1);
                    c_ohr0 <= f_onehot(v_r);
                    c_ohr1 <= f_onehot(v_r + 1);
                    c_sfr  <= c_smv(3 downto 0);
                    q_sa   <= c_tB(23 downto 16);
                    q_st   <= to_unsigned(25, 6);

                when 25 =>      -- (sine ROM in flight)
                    q_st <= to_unsigned(26, 6);

                when 26 =>
                    q_st <= to_unsigned(27, 6);

                when 27 =>      -- Breathe: level 100 +- 200 . sin
                    mul_op <= to_unsigned(9, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(28, 6);

                when 28 =>
                    if mul_go = '0' and mul_busy = '0' then
                        if c_brth = '1' then
                            c_off <= to_signed(100, 12) + m_acc(18 downto 7);
                        else
                            c_off <= to_signed(100, 12);
                        end if;
                        q_st <= to_unsigned(29, 6);
                    end if;

                when 29 =>      -- field start; prime the line engine for line 0
                    if s_ilace = '1' and s_timing.field_n = '0' then
                        -- bottom field: half a field line down
                        l_py  <= c_tY + resize(c_ky(20 downto 1), 24);
                        l_pyd <= c_tD + resize(c_kdy(20 downto 1), 24);
                        l_pw  <= c_tW + resize(c_kwl(17 downto 1), 24);
                        l_pe  <= c_tE + resize(c_kel(17 downto 1), 24);
                    else
                        l_py  <= c_tY;
                        l_pyd <= c_tD;
                        l_pw  <= c_tW;
                        l_pe  <= c_tE;
                    end if;
                    q_prime <= '1';
                    q_st    <= to_unsigned(32, 6);

                ------------------------------------------------------------
                -- per line (for the NEXT line)
                ------------------------------------------------------------
                when 32 =>
                    if q_prime = '0' then
                        l_py  <= l_py + resize(c_ky, 24);
                        l_pyd <= l_pyd + resize(c_kdy, 24);
                        l_pw  <= l_pw + resize(c_kwl, 24);
                        l_pe  <= l_pe + resize(c_kel, 24);
                    end if;
                    q_prime <= '0';
                    q_st    <= to_unsigned(33, 6);

                when 33 =>      -- wiggle sine; xorshift32
                    q_sa  <= l_pw(23 downto 16);
                    l_rng <= l_rng xor shift_left(l_rng, 13);
                    q_st  <= to_unsigned(34, 6);

                when 34 =>
                    l_rng <= l_rng xor shift_right(l_rng, 17);
                    q_st  <= to_unsigned(35, 6);

                when 35 =>
                    l_rng <= l_rng xor shift_left(l_rng, 5);
                    q_st  <= to_unsigned(36, 6);

                when 36 =>      -- wiggle = sin . awig / 128; waist sine
                    mul_op <= to_unsigned(10, 4);
                    mul_go <= '1';
                    q_sa   <= l_pe(23 downto 16);
                    q_st   <= to_unsigned(37, 6);

                when 37 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_wig <= m_acc(22 downto 7);
                        q_st  <= to_unsigned(38, 6);
                    end if;

                when 38 =>      -- (the waist sine has long arrived)
                    l_sp <= unsigned(std_logic_vector'(not q_sr(7) & std_logic_vector(q_sr(6 downto 0))));  -- sin + 128
                    q_st <= to_unsigned(39, 6);

                when 39 =>      -- width = wd0 - wdm . (sin + 128) / 256
                    mul_op <= to_unsigned(11, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(40, 6);

                when 40 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_wd <= c_wd0 - unsigned(m_acc(19 downto 8));
                        q_st <= to_unsigned(41, 6);
                    end if;

                when 41 =>      -- tear bursts: 1 in 32 lines starts a run of
                                -- 8..39 lines at one of four strengths
                    if l_bc = 0 then
                        if l_rng(4 downto 0) = 0 then
                            l_bc <= to_unsigned(8, 6) + resize(l_rng(9 downto 5), 6);
                            l_bl <= l_rng(11 downto 10);
                        end if;
                    else
                        l_bc <= l_bc - 1;
                    end if;
                    q_st <= to_unsigned(42, 6);

                when 42 =>      -- tear offset: held on half the lines, else a new
                                -- draw: +-8 px, +-8..64 px in a burst (HD)
                    if c_tear = '0' then
                        l_j <= (others => '0');
                    elsif l_rng(12) = '0' then
                        v_jr := signed(resize(l_rng(19 downto 13), 8)) - 64;
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
                        if c_jm = '0' then
                            v_jr := shift_right(v_jr, 1);
                        end if;
                        l_j <= v_jr;
                    end if;
                    q_st <= to_unsigned(43, 6);

                when 43 =>      -- |j| by slice (-64 must not fold to 0)
                    if l_j < 0 then
                        v_jr := -l_j;
                        l_ja <= unsigned(v_jr(6 downto 0));
                        l_jn <= '1';
                    else
                        l_ja <= unsigned(l_j(6 downto 0));
                        l_jn <= '0';
                    end if;
                    q_st <= to_unsigned(44, 6);

                when 44 =>      -- starts = -(c . k) + j . k
                    mul_op <= to_unsigned(12, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(45, 6);

                when 45 =>
                    if mul_go = '0' and mul_busy = '0' then
                        v_m := m_acc(29 downto 0);
                        if l_jn = '1' then
                            l_sx <= c_ncx - v_m;
                        else
                            l_sx <= c_ncx + v_m;
                        end if;
                        mul_op <= to_unsigned(13, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(46, 6);
                    end if;

                when 46 =>
                    if mul_go = '0' and mul_busy = '0' then
                        v_m := m_acc(29 downto 0);
                        if l_jn = '1' then
                            l_sv <= c_ncv - v_m;
                        else
                            l_sv <= c_ncv + v_m;
                        end if;
                        mul_op <= to_unsigned(14, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(47, 6);
                    end if;

                when 47 =>
                    if mul_go = '0' and mul_busy = '0' then
                        v_m := m_acc(29 downto 0);
                        if l_jn = '1' then
                            l_sd <= c_ncd - v_m;
                        else
                            l_sd <= c_ncd + v_m;
                        end if;
                        mul_op <= to_unsigned(15, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(48, 6);
                    end if;

                when 48 =>
                    if mul_go = '0' and mul_busy = '0' then
                        if l_jn = '1' then
                            l_se <= c_nce - m_acc(21 downto 0);
                        else
                            l_se <= c_nce + m_acc(21 downto 0);
                        end if;
                        q_st <= to_unsigned(49, 6);
                    end if;

                when 49 =>      -- hand the line over
                    n_py  <= l_py(23 downto 8);
                    n_pyd <= l_pyd(23 downto 8);
                    n_wig <= l_wig;
                    n_wd  <= l_wd;
                    n_sx  <= l_sx;
                    n_sv  <= l_sv;
                    n_sd  <= l_sd;
                    n_se  <= l_se;
                    q_st  <= to_unsigned(0, 6);

                when others =>
                    q_st <= to_unsigned(0, 6);
            end case;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Pixel path.  S0 = the clock where the accumulators hold the pixel's
    -- phases and g_avid the pixel's avid; S21 = the output register.
    -- ====================================================================
    p_pix : process(clk)
        variable v_a   : signed(29 downto 0);
        variable v_ae  : signed(21 downto 0);
        variable v_p   : unsigned(15 downto 0);
        variable v_t   : unsigned(10 downto 0);
        variable v_rr  : unsigned(9 downto 0);
        variable v_sum : unsigned(10 downto 0);
        variable v_br  : unsigned(3 downto 0);
        variable v_nb  : unsigned(15 downto 0);
        variable v_cs  : unsigned(15 downto 0);
        variable v_cc  : unsigned(15 downto 0);
        variable v_ohf, v_ohr : std_logic_vector(12 downto 0);
    begin
        if rising_edge(clk) then
            -- S0: the line's accumulators (loaded at g_go)
            if g_go = '1' then
                a_x    <= n_sx;
                a_v    <= n_sv;
                a_d    <= n_sd;
                a_e    <= n_se;
                px_py  <= n_py;
                px_pyd <= n_pyd;
                px_wig <= n_wig;
                px_wd  <= n_wd;
                px_x4  <= (others => '0');
            else
                a_x   <= a_x + signed(resize(c_kx, 30));
                a_v   <= a_v + signed(resize(c_kv, 30));
                a_d   <= a_d + signed(resize(c_kdx, 30));
                a_e   <= a_e + signed(resize(c_ke, 22));
                px_x4 <= px_x4 + 1;
            end if;
            px_ls <= g_go;

            -- S1: Mirror folds the across-phases about the centre (one's
            -- complement); the envelope always does
            v_a := a_x;
            if c_mir = '1' and v_a(29) = '1' then v_a := not v_a; end if;
            p1_px <= unsigned(v_a(23 downto 8));
            v_a := a_v;
            if c_mir = '1' and v_a(29) = '1' then v_a := not v_a; end if;
            p1_pv <= unsigned(v_a(23 downto 8));
            v_a := a_d;
            if c_mir = '1' and v_a(29) = '1' then v_a := not v_a; end if;
            p1_pd <= unsigned(v_a(23 downto 8));
            v_ae := a_e;
            if v_ae(21) = '1' then v_ae := not v_ae; end if;
            p1_ue <= unsigned(v_ae(20 downto 8));
            xl(1) <= px_ls & std_logic_vector(px_x4);
            for i in 2 to 14 loop
                xl(i) <= xl(i - 1);
            end loop;

            -- S2: wave addresses (wiggle on the across-phase, time on the
            -- warp, the line's phase on the diagonal); envelope excess
            v_p   := p1_px + unsigned(px_wig);
            p2_ax <= c_shape & v_p(15 downto 8);
            v_p   := p1_pv + c_tV(23 downto 8);
            p2_av <= c_shape & v_p(15 downto 8);
            v_p   := p1_pd + px_pyd;
            p2_ad <= c_shape & v_p(15 downto 8);
            p2_e  <= signed('0' & p1_ue) - signed("00" & px_wd);

            -- S3: wave ROMs (block RAM); envelope clamp 0..1023
            r3_x <= C_WAVE(to_integer(p2_ax));
            r3_v <= C_WAVE(to_integer(p2_av));
            r3_d <= C_WAVE(to_integer(p2_ad));
            if p2_e < 0 then
                p3_env <= (others => '0');
            elsif p2_e > 1023 then
                p3_env <= (others => '1');
            else
                p3_env <= unsigned(p2_e(9 downto 0));
            end if;

            -- S4: into fabric
            s4_x   <= signed(r3_x);
            s4_v   <= signed(r3_v);
            s4_d   <= signed(r3_d);
            p4_env <= p3_env;

            -- S5: Warp products
            m5_v   <= s4_v * signed('0' & c_wA);
            m5_d   <= s4_d * signed('0' & c_wA);
            p5_x   <= s4_x;
            p5_env <= p4_env;

            -- S6: down-phase displaced by the warp; -(across + diagonal)
            v_p    := px_py + unsigned(m5_v(15 downto 0));
            p6_ay  <= c_shape & v_p(15 downto 8);
            p6_xd  <= -resize(p5_x, 10) - resize(m5_d(16 downto 8), 10);
            p6_env <= p5_env;

            -- S7: the down wave (block RAM)
            r7_y   <= C_WAVE(to_integer(p6_ay));
            p7_xd  <= p6_xd;
            p7_env <= p6_env;

            -- S8: into fabric
            s8_y   <= signed(r7_y);
            p8_xd  <= p7_xd;
            p8_env <= p7_env;

            -- S9: the egg crate: valleys where the cosines peak
            p9_egg <= p8_xd - resize(s8_y, 10);
            p9_env <= p8_env;

            -- S10: x Depth
            m10     <= p9_egg * signed('0' & c_A);
            p10_env <= p9_env;

            -- S11: G = Depth . egg + Level (index units, 1024 per span)
            p11_g   <= resize(shift_right(m10, 6), 15) + resize(c_off, 15);
            p11_env <= p10_env;

            -- S12: triangle fold, plus the envelope, clipped to the palette
            v_t := unsigned(p11_g(10 downto 0));
            if v_t(10) = '1' then
                v_rr := not v_t(9 downto 0);
            else
                v_rr := v_t(9 downto 0);
            end if;
            v_sum := ('0' & v_rr) + ('0' & p11_env);
            if v_sum(10) = '1' then
                p12_idx <= (others => '1');
            else
                p12_idx <= v_sum(9 downto 0);
            end if;

            -- S13: lag shift for this pixel: ordered dither s / s+1, and
            -- (Drip) the rise shift when the index is above the lag
            v_br := unsigned'(xl(12)(0) & xl(12)(1) & xl(12)(2) & xl(12)(3));
            if v_br < c_sfr then
                v_ohf := c_ohf1;
                v_ohr := c_ohr1;
            else
                v_ohf := c_ohf0;
                v_ohr := c_ohr0;
            end if;
            if (p12_idx & "000000") > lag_l then
                p13_oh <= v_ohr;
            else
                p13_oh <= v_ohf;
            end if;
            p13_i <= p12_idx & "000000";

            -- S14: the target's share, i >> s
            p14_p  <= f_shr(p13_i, p13_oh);
            p14_i  <= p13_i;
            p14_oh <= p13_oh;

            -- S15: the lag L + (i >> s) - (L >> s), carry-saved into one
            -- adder; settled to the index at the line's first pixel
            if xl(14)(4) = '1' then
                lag_l <= p14_i;
            else
                v_nb  := not f_shr(lag_l, p14_oh);
                v_cs  := lag_l xor v_nb xor p14_p;
                v_cc  := (lag_l and v_nb) or (lag_l and p14_p) or (v_nb and p14_p);
                lag_l <= v_cs + (v_cc(14 downto 0) & '1');
            end if;

            -- S16: palette address
            p16_a <= c_pal & lag_l(15 downto 8);

            -- S17: palette ROMs (block RAM)
            r17_y <= C_PY(to_integer(p16_a));
            r17_u <= C_PU(to_integer(p16_a));
            r17_v <= C_PV(to_integer(p16_a));

            -- S18: into fabric, 10-bit
            t18_y <= unsigned(r17_y) & "00";
            t18_u <= unsigned(r17_u) & "00";
            t18_v <= unsigned(r17_v) & "00";
        end if;
    end process p_pix;

    -- ====================================================================
    -- S19..S21: output conditioning and the blanking gate
    -- ====================================================================
    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S19/S20: horizontal [1 2 1]/4 (g1 is the centre pixel)
            g1_y <= t18_y;  g2_y <= g1_y;
            g1_u <= t18_u;  g2_u <= g1_u;
            g1_v <= t18_v;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(t18_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(t18_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(t18_v, 12);
            p20_y <= v_sy(11 downto 2);
            p20_u <= v_su(11 downto 2);
            p20_v <= v_sv(11 downto 2);

            -- S21: luma 64 + 13/16 (y - 64) (white ~775); neutral outside
            -- the generated avid
            v_d  := p20_y - 64;
            v_yo := p20_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if sd(20)(3) = '0' then
                s_out_y <= C_BLK;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo;
                s_out_u <= p20_u;
                s_out_v <= p20_v;
            end if;

            sd(1) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            for k in 2 to 21 loop
                sd(k) <= sd(k - 1);
            end loop;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= sd(21)(3);
    data_out.hsync_n <= sd(21)(2);
    data_out.vsync_n <= sd(21)(1);
    data_out.field_n <= sd(21)(0);

end architecture marrow;
