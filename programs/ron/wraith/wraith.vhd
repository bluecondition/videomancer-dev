-- wraith.vhd  (v0.1.0) -- neon wraiths: hard-synced flame lobes, tiled
--
-- WRAITH.  An analog-synth patch rebuilt in one pass, made from a reference
-- capture: glowing neon flame forms (violet rims, cyan cores, pale white,
-- pink, peach and signal green) on an azure ramp, repeated three times
-- across the frame, with tapered horizontal fins, a hard colour seam down
-- the middle of every tile, V-hold slips that cut the picture into bands,
-- and thin magenta tracking lines.  Synthesis: no input video is read.
--
-- PICTURE
--  1. TILES.  A master oscillator runs at Tiles cycles per line (phase m,
--     0 at the spine in the middle of each tile); T = |1 - 2m| is a
--     triangle, 1 at the spine and 0 in the trough between tiles.  A slow
--     per-line wiggle bends every tile.  The background is a palette ramp
--     indexed by T (the azure ramp and dark trough of the reference).
--  2. FLAME.  A slave oscillator is hard-synced to the master at every
--     spine: phi = w(y) + u . v(y), u = m (Mirror: u = T), so its lobes
--     restart at the spine (the colour seam) and their width and position
--     follow the per-line v and w.  F = cos(phi).
--  3. WINDOW.  G = (F + 1)/2 + 1.625 (T - 1) + L(y): the lobes are only
--     lit near the spine and taper to points as they swing away from it
--     (the tapered wings and spikes of the reference).  Glow sets L.
--  4. FINS.  A vertical sawtooth s(y), Bands per frame, curved by s^2:
--     the phase sweep w = 0.3 sin + Lash . s^2 throws the lobes sideways
--     faster and faster until the saw resets - a hard horizontal edge with
--     a long tapered fin.  Flare narrows the lobes after every reset,
--     v = Lobes . (1 + 0.35 sin) . (1 + Flare (1 - s)^2).
--  5. PALETTE.  index = G/2 (Rings: 2G) + a slow per-line hue wave, into a
--     cyclic 256-entry flame palette where G > 0, else the background.
--  6. TEAR / LINES.  V-hold slips jump the line phase of every vertical
--     wave at random lines, with a per-line +-3 px jitter (the fine streak
--     texture); tracking lines lay a magenta chroma tint (5 lines, soft
--     profile) or a dark dip (2 lines) across the frame.  Both patterns are
--     held for 8..39 fields, then jump.
--
-- ENGINE
--   A small sequencer with one serial multiplier and one serial divider.
--   No state writes the multiplier's operands or op code: both the op code
--   and one-hot operand-source selects are registered every clock from the
--   state number, and the fetch is a flat AND-OR (writing mul_op from 17
--   states made the state decode -> CEN fan-out the critical path).
--   Per field (first vsync edge after active video): 2^24/W, 2^24/H, the
--   tile step and centre phase, the control curves, speed, the time
--   phases, the tear/lines seed.  Per active line (kicked at the line's
--   start, computing the NEXT line, ~200 clocks): xorshift32, the line
--   phase (+ slip), six wave phases, five sines, s^2, (1 - s)^2, v, w, L,
--   hue, wiggle, jitter and the master start phase, the tracking-line tint.
--
-- CONTROLS
--   K1 Tiles    1 .. 7 flames across (2.95 in the reference)
--   K2 Lobes    0.5 .. 3.5 slave cycles per tile: one fat body -> ribbons
--   K3 Flare    how much the lobes narrow after each fin reset
--   K4 Speed    bipolar drift, still in the centre (positive = fins rise)
--   K5 Glow     flame level: thin slivers -> flames flooding the tile
--   K6 Palette  Silk / Ember / Venom / Frost / Orchid / Aurora / Spectrum / Mono
--   S7 Mirror   sync on the triangle: symmetric flames, no seam
--   S8 Tear     V-hold slips + per-line jitter
--   S9 Lines    magenta tracking lines and dark dips
--   S10 Rings   index = 2G: contour rings through the palette
--   S11 Flip    reverse the fin saw: fins flare the other way
--   P12 Lash    fin sweep 0.1 .. 2.1 turns and 1.5 .. 4.5 bands: calm
--               flowing flames at 0, shredded lashing fins at 100 %
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, fixed
-- width); one latency for all modes (17 stages); outputs gated to neutral
-- outside the generated avid; palettes stored U/V-swapped (wraith.py);
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
use work.wraith_pkg.all;

architecture wraith of program_top is

    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    -- knob glides: 0 K1 Tiles, 1 K2 Lobes, 2 K3 Flare, 3 K5 Glow,
    -- 4 P12 Lash.  Deadband hold + per-field ease, 10.4
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
    signal q_sa : unsigned(9 downto 0) := (others => '0');
    signal q_sd : std_logic_vector(15 downto 0) := (others => '0');
    signal q_sr : signed(15 downto 0) := (others => '0');

    -- serial multiplier: the issuing state writes an op code; the operands
    -- are fetched into o_* (one registered mux), then loaded
    signal mul_go, mul_busy, m_ld : std_logic := '0';
    signal mul_op : unsigned(4 downto 0) := (others => '0');
    -- one-hot operand-source selects, registered from the state with the
    -- op code: the fetch is a flat AND-OR, no decode
    signal sel_a  : std_logic_vector(10 downto 0) := (others => '0');
    signal sel_b  : std_logic_vector(16 downto 0) := (others => '0');
    signal o_a    : signed(27 downto 0) := (others => '0');
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
    signal c_mir, c_tear, c_lines, c_rings, c_flip : std_logic := '0';
    signal c_jm   : std_logic := '1';                                 -- HD-width
    signal c_W2   : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal c_recw : unsigned(15 downto 0) := to_unsigned(8738, 16);   -- 2^24 / W
    signal c_dq   : unsigned(17 downto 0) := to_unsigned(15534, 18);  -- 2^24 / field lines
    signal c_tl   : unsigned(10 downto 0) := to_unsigned(755, 11);    -- tiles x 256
    signal c_kT   : unsigned(18 downto 0) := to_unsigned(25770, 19);  -- tile turns/px x 2^24
    signal c_nc   : unsigned(23 downto 0) := (others => '0');         -- -c . kT
    signal c_v0   : unsigned(9 downto 0) := to_unsigned(320, 10);     -- lobes x 256
    signal c_fl   : unsigned(9 downto 0) := to_unsigned(230, 10);     -- flare x 256
    signal c_L0   : signed(10 downto 0) := to_signed(77, 11);         -- glow x 256
    signal c_lash : unsigned(9 downto 0) := to_unsigned(179, 10);     -- turns x 256
    signal c_bands : unsigned(8 downto 0) := to_unsigned(153, 9);     -- bands x 64
    signal c_dir  : std_logic := '0';
    signal c_mag  : unsigned(8 downto 0) := (others => '0');
    signal c_spd  : unsigned(19 downto 0) := (others => '0');
    signal c_tS, c_t1, c_t2, c_t3, c_t4, c_t5, c_th : unsigned(23 downto 0) := (others => '0');
    signal c_seed : unsigned(31 downto 0) := x"2545F491";
    signal c_hold : unsigned(5 downto 0) := (others => '0');

    -- per-line engine
    signal l_q    : signed(26 downto 0) := (others => '0');           -- line phase, 2^24 / field
    signal l_rng  : unsigned(31 downto 0) := x"2545F491";
    signal l_jm   : unsigned(1 downto 0) := (others => '0');          -- |jitter|
    signal l_jn   : std_logic := '0';
    signal l_s12  : unsigned(11 downto 0) := (others => '0');         -- the fin saw
    signal l_p1, l_p2, l_p3, l_p4, l_p5 : unsigned(15 downto 0) := (others => '0');
    signal l_sin1, l_sin2, l_sin3, l_sin4, l_sin5 : signed(15 downto 0) := (others => '0');
    signal l_s2   : unsigned(15 downto 0) := (others => '0');         -- s^2
    signal l_r    : unsigned(15 downto 0) := (others => '0');         -- (1 - s)^2
    signal l_a    : unsigned(15 downto 0) := to_unsigned(32768, 16);  -- x 2^15
    signal l_b    : unsigned(13 downto 0) := to_unsigned(4096, 14);   -- x 2^12
    signal l_v1   : unsigned(14 downto 0) := (others => '0');         -- x 2^12
    signal l_v    : unsigned(15 downto 0) := (others => '0');         -- cycles/tile x 2^12
    signal l_ws   : unsigned(15 downto 0) := (others => '0');
    signal l_ww   : signed(15 downto 0) := (others => '0');
    signal l_Lw, l_hw : signed(7 downto 0) := (others => '0');
    signal l_wg   : unsigned(23 downto 0) := (others => '0');
    signal l_jt   : unsigned(23 downto 0) := (others => '0');
    signal l_w16  : unsigned(15 downto 0) := (others => '0');
    signal l_E    : signed(11 downto 0) := (others => '0');
    signal l_hue  : unsigned(7 downto 0) := (others => '0');
    signal l_a0   : unsigned(23 downto 0) := (others => '0');
    -- tracking-line events: lines left, dark, peak
    signal ev_n   : unsigned(2 downto 0) := (others => '0');
    signal ev_k   : std_logic := '0';
    signal ev_a   : unsigned(1 downto 0) := (others => '0');
    -- handed to the pixel path at g_go
    signal n_a0   : unsigned(23 downto 0) := (others => '0');
    signal n_v    : unsigned(15 downto 0) := (others => '0');
    signal n_w    : unsigned(15 downto 0) := (others => '0');
    signal n_E    : signed(11 downto 0) := (others => '0');
    signal n_hue  : unsigned(7 downto 0) := (others => '0');
    signal n_tc, n_ty : unsigned(7 downto 0) := (others => '0');
    signal n_dk   : unsigned(1 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pixel path
    ------------------------------------------------------------------------
    signal a_m    : unsigned(23 downto 0) := (others => '0');
    signal px_v   : unsigned(15 downto 0) := (others => '0');
    signal px_w   : unsigned(15 downto 0) := (others => '0');
    signal px_E   : signed(11 downto 0) := (others => '0');
    signal px_hue : unsigned(7 downto 0) := (others => '0');
    signal px_tc, px_ty : unsigned(7 downto 0) := (others => '0');
    signal px_dk  : unsigned(1 downto 0) := (others => '0');

    signal p1_u   : unsigned(11 downto 0) := (others => '0');
    signal p1_T8  : unsigned(7 downto 0) := (others => '0');
    signal p2_ph, p2_pl : unsigned(19 downto 0) := (others => '0');
    signal p2_cT  : unsigned(9 downto 0) := (others => '0');
    signal p2_T7, p3_T7, p4_T7, p5_T7, p6_T7, p7_T7, p8_T7, p9_T7 : unsigned(6 downto 0) := (others => '0');
    signal p3_pr  : unsigned(27 downto 0) := (others => '0');
    signal p3_E, p4_E, p5_E, p6_E : signed(11 downto 0) := (others => '0');
    signal p4_phi : unsigned(15 downto 0) := (others => '0');
    signal r5_c   : std_logic_vector(7 downto 0) := (others => '0');
    signal s6_f   : signed(7 downto 0) := (others => '0');
    signal p7_g   : signed(11 downto 0) := (others => '0');
    signal p8_sh, p9_sh, p10_sh, p11_sh : std_logic := '0';
    signal p8_gs  : unsigned(7 downto 0) := (others => '0');
    signal p9_sa  : unsigned(10 downto 0) := (others => '0');
    signal p9_ba  : unsigned(9 downto 0) := (others => '0');
    signal r10_sy, r10_su, r10_sv : std_logic_vector(7 downto 0) := (others => '0');
    signal r10_by, r10_bu, r10_bv : std_logic_vector(7 downto 0) := (others => '0');
    signal t11_sy, t11_su, t11_sv, t11_by, t11_bu, t11_bv : unsigned(7 downto 0) := (others => '0');
    signal t12_y, t12_u, t12_v : unsigned(9 downto 0) := (others => '0');
    signal t13_y, t13_u, t13_v : unsigned(9 downto 0) := (others => '0');
    signal t14_y, t14_u, t14_v : unsigned(9 downto 0) := (others => '0');
    -- output conditioning (the HD HDMI bar fix, damier v0.4.4): [1 2 1]
    -- horizontal smoothing on Y/U/V, luma scaled toward black
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- syncs (generated avid, hsync, vsync, field) from S1 to S17
    type t_sd is array(1 to 17) of std_logic_vector(3 downto 0);
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
    -- rises, when the pixel path loads the line's accumulator.
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
    -- (by 1/8; the Lash slider by 1/4 so the gesture stays quick)
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
    -- The multiplier's op code is a registered function of the state, with
    -- no enable (the issuing state is still current at the edge where it
    -- raises mul_go, so the fetch one clock later reads that state's op).
    -- Writing it from every issuing state made the state decode -> op CEN
    -- the critical path (totem's lesson, again).
    -- ====================================================================
    p_opmap : process(clk)
        variable v_op : integer range 0 to 31;
    begin
        if rising_edge(clk) then
            case to_integer(q_st) is
                when 3      => v_op := 0;       -- kT
                when 4      => v_op := 1;       -- c . kT
                when 7      => v_op := 2;       -- speed^2
                when 36 to 49 =>                -- the line engine: ops 3 .. 16
                    v_op := to_integer(q_st) - 33;
                when others => v_op := 0;
            end case;
            mul_op <= to_unsigned(v_op, 5);
            -- o_a sources: 0 recw, 1 kT, 2 mag, 3 q, 4 s, 5 ~s, 6 Flare,
            -- 7 Lobes, 8 v1, 9 Lash, 10 sin5
            sel_a <= (others => '0');
            case v_op is
                when 0       => sel_a(0) <= '1';
                when 1 | 16  => sel_a(1) <= '1';
                when 2       => sel_a(2) <= '1';
                when 3 to 8  => sel_a(3) <= '1';
                when 9       => sel_a(4) <= '1';
                when 10      => sel_a(5) <= '1';
                when 11      => sel_a(6) <= '1';
                when 12      => sel_a(7) <= '1';
                when 13      => sel_a(8) <= '1';
                when 14      => sel_a(9) <= '1';
                when others  => sel_a(10) <= '1';
            end case;
            -- o_b sources: one per op
            sel_b <= (others => '0');
            sel_b(v_op) <= '1';
        end if;
    end process p_opmap;

    -- ====================================================================
    -- Serial multiplier (shift-add, one bit of b per clock).  go -> operand
    -- fetch by op code (registered) -> load -> n iterations.
    -- ====================================================================
    p_mul : process(clk)
    begin
        if rising_edge(clk) then
            m_ld <= '0';
            if mul_go = '1' then
                o_a <= ((27 downto 0 => sel_a(0)) and signed(resize(c_recw, 28)))
                    or ((27 downto 0 => sel_a(1)) and signed(resize(c_kT, 28)))
                    or ((27 downto 0 => sel_a(2)) and signed(resize(c_mag, 28)))
                    or ((27 downto 0 => sel_a(3)) and resize(l_q, 28))
                    or ((27 downto 0 => sel_a(4)) and signed(resize(l_s12, 28)))
                    or ((27 downto 0 => sel_a(5)) and signed(resize(not l_s12, 28)))
                    or ((27 downto 0 => sel_a(6)) and signed(resize(c_fl, 28)))
                    or ((27 downto 0 => sel_a(7)) and signed(resize(c_v0, 28)))
                    or ((27 downto 0 => sel_a(8)) and signed(resize(l_v1, 28)))
                    or ((27 downto 0 => sel_a(9)) and signed(resize(c_lash, 28)))
                    or ((27 downto 0 => sel_a(10)) and resize(l_sin5, 28));
                -- q x bands, then the wave phases q x 70, 45, 58, 38, 109 (/64)
                o_b <= ((15 downto 0 => sel_b(0)) and resize(c_tl, 16))
                    or ((15 downto 0 => sel_b(1)) and resize(c_W2, 16))
                    or ((15 downto 0 => sel_b(2)) and resize(c_mag, 16))
                    or ((15 downto 0 => sel_b(3)) and resize(c_bands, 16))
                    or ((15 downto 0 => sel_b(4)) and to_unsigned(70, 16))
                    or ((15 downto 0 => sel_b(5)) and to_unsigned(45, 16))
                    or ((15 downto 0 => sel_b(6)) and to_unsigned(58, 16))
                    or ((15 downto 0 => sel_b(7)) and to_unsigned(38, 16))
                    or ((15 downto 0 => sel_b(8)) and to_unsigned(109, 16))
                    or ((15 downto 0 => sel_b(9)) and resize(l_s12, 16))
                    or ((15 downto 0 => sel_b(10)) and resize(not l_s12, 16))
                    or ((15 downto 0 => sel_b(11)) and l_r)
                    or ((15 downto 0 => sel_b(12)) and l_a)
                    or ((15 downto 0 => sel_b(13)) and resize(l_b, 16))
                    or ((15 downto 0 => sel_b(14)) and l_s2)
                    or ((15 downto 0 => sel_b(15)) and to_unsigned(18, 16))
                    or ((15 downto 0 => sel_b(16)) and resize(l_jm, 16));
                -- iterations = the b operand's width
                case to_integer(mul_op) is
                    when 0 | 1      => o_n <= to_unsigned(11, 5);
                    when 2 | 3      => o_n <= to_unsigned(9, 5);
                    when 4 | 8      => o_n <= to_unsigned(7, 5);
                    when 5 | 6 | 7  => o_n <= to_unsigned(6, 5);
                    when 9 | 10     => o_n <= to_unsigned(12, 5);
                    when 11 | 12 | 14 => o_n <= to_unsigned(16, 5);
                    when 13         => o_n <= to_unsigned(14, 5);
                    when 15         => o_n <= to_unsigned(5, 5);
                    when others     => o_n <= to_unsigned(2, 5);
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
    -- Sequencer: per-field set-up (states 1..11) and the per-line engine
    -- (32..52).  One small operation per state.  A unit is free when its
    -- go is low and busy is low (go is registered, busy follows one clock
    -- later), so a wait state can follow its issue state directly.
    -- ====================================================================
    p_seq : process(clk)
        variable v_k6 : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_ad : unsigned(9 downto 0);
        variable v_p  : unsigned(15 downto 0);
        variable v_s  : unsigned(11 downto 0);
        variable v_t  : signed(16 downto 0);
        variable v_vv : unsigned(16 downto 0);
        variable v_pk : unsigned(7 downto 0);
        variable v_tc : unsigned(7 downto 0);
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
                    c_lines <= registers_in(6)(2);
                    c_rings <= registers_in(6)(3);
                    c_flip  <= registers_in(6)(4);
                    c_W2    <= s_meas_w(11 downto 1);
                    c_jm    <= s_meas_w(11) or s_meas_w(10);
                    div_den <= s_meas_w;
                    div_k   <= to_unsigned(25, 5);
                    div_go  <= '1';
                    q_st    <= to_unsigned(2, 6);

                when 2 =>       -- W reciprocal; start 2^24 / H
                    if div_go = '0' and div_busy = '0' then
                        if d_q(24 downto 16) /= 0 then
                            c_recw <= (others => '1');
                        else
                            c_recw <= d_q(15 downto 0);
                        end if;
                        div_den <= resize(s_meas_h, 12);
                        div_k   <= to_unsigned(25, 5);
                        div_go  <= '1';
                        q_st    <= to_unsigned(3, 6);
                    end if;

                when 3 =>       -- line step; tiles x 256 = 256 + 1.5 k1 (1 .. 7)
                    if div_go = '0' and div_busy = '0' then
                        if d_q(24 downto 18) /= 0 then
                            c_dq <= (others => '1');
                        else
                            c_dq <= d_q(17 downto 0);
                        end if;
                        c_tl   <= to_unsigned(256, 11) + resize(c_k1, 11) + resize(c_k1(9 downto 1), 11);
                        mul_go <= '1';
                        q_st   <= to_unsigned(4, 6);
                    end if;

                when 4 =>       -- tile step; centre phase
                    if mul_go = '0' and mul_busy = '0' then
                        c_kT   <= unsigned(m_acc(26 downto 8));
                        mul_go <= '1';
                        q_st   <= to_unsigned(5, 6);
                    end if;

                when 5 =>       -- -c . kT; control curves
                    if mul_go = '0' and mul_busy = '0' then
                        c_nc    <= unsigned(-m_acc(23 downto 0));
                        c_v0    <= to_unsigned(128, 10) + resize(c_k2(9 downto 1), 10)
                                   + resize(c_k2(9 downto 2), 10);         -- 0.5 .. 3.5
                        c_fl    <= resize(c_k3(9 downto 1), 10) + resize(c_k3(9 downto 2), 10);
                        c_L0    <= signed(resize(c_k5(9 downto 2), 11)) + signed(resize(c_k5(9 downto 3), 11))
                                   - 96;                                   -- -0.375 .. 1.12
                        c_lash  <= to_unsigned(26, 10) + resize(c_p12(9 downto 1), 10);
                        c_bands <= to_unsigned(96, 9) + resize(c_p12(9 downto 3), 9)
                                   + resize(c_p12(9 downto 4), 9);         -- 1.5 .. 4.5
                        q_st    <= to_unsigned(6, 6);
                    end if;

                when 6 =>       -- Speed: bipolar with a +-24 still zone
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
                    q_st <= to_unsigned(7, 6);

                when 7 =>
                    mul_go <= '1';
                    q_st   <= to_unsigned(8, 6);

                when 8 =>       -- speed = 4 mag^2
                    if mul_go = '0' and mul_busy = '0' then
                        c_spd <= unsigned(m_acc(17 downto 0)) & "00";
                        q_st  <= to_unsigned(9, 6);
                    end if;

                when 9 =>       -- time phases at their own ratios; positive speed
                                -- makes the fins rise
                    if c_dir = '0' then
                        c_tS <= c_tS + resize(c_spd(19 downto 1), 24);
                        c_t1 <= c_t1 + resize(c_spd(19 downto 1), 24) + resize(c_spd(19 downto 2), 24);
                        c_t2 <= c_t2 - resize(c_spd(19 downto 1), 24) - resize(c_spd(19 downto 3), 24);
                        c_t3 <= c_t3 + resize(c_spd(19 downto 2), 24) + resize(c_spd(19 downto 3), 24);
                        c_t4 <= c_t4 + resize(c_spd(19 downto 2), 24);
                        c_t5 <= c_t5 + resize(c_spd, 24);
                        c_th <= c_th + resize(c_spd(19 downto 4), 24);
                    else
                        c_tS <= c_tS - resize(c_spd(19 downto 1), 24);
                        c_t1 <= c_t1 - resize(c_spd(19 downto 1), 24) - resize(c_spd(19 downto 2), 24);
                        c_t2 <= c_t2 + resize(c_spd(19 downto 1), 24) + resize(c_spd(19 downto 3), 24);
                        c_t3 <= c_t3 - resize(c_spd(19 downto 2), 24) - resize(c_spd(19 downto 3), 24);
                        c_t4 <= c_t4 - resize(c_spd(19 downto 2), 24);
                        c_t5 <= c_t5 - resize(c_spd, 24);
                        c_th <= c_th - resize(c_spd(19 downto 4), 24);
                    end if;
                    q_st <= to_unsigned(10, 6);

                when 10 =>      -- tear / lines pattern: held 8 .. 39 fields
                    if c_hold = 0 then
                        c_seed <= l_rng;
                        c_hold <= to_unsigned(8, 6) + resize(l_rng(4 downto 0), 6);
                    else
                        c_hold <= c_hold - 1;
                    end if;
                    q_st <= to_unsigned(11, 6);

                when 11 =>      -- field start; prime the line engine for line 0
                    if s_ilace = '1' and s_timing.field_n = '0' then
                        -- bottom field: half a field line down
                        l_q <= signed(resize(c_dq(17 downto 1), 27));
                    else
                        l_q <= (others => '0');
                    end if;
                    l_rng   <= c_seed;
                    ev_n    <= (others => '0');
                    l_jm    <= (others => '0');
                    q_prime <= '1';
                    q_st    <= to_unsigned(32, 6);

                ------------------------------------------------------------
                -- per line (for the NEXT line)
                ------------------------------------------------------------
                when 32 =>      -- xorshift32
                    l_rng <= l_rng xor shift_left(l_rng, 13);
                    q_st  <= to_unsigned(33, 6);

                when 33 =>
                    l_rng <= l_rng xor shift_right(l_rng, 17);
                    q_st  <= to_unsigned(34, 6);

                when 34 =>
                    l_rng <= l_rng xor shift_left(l_rng, 5);
                    q_st  <= to_unsigned(35, 6);

                when 35 =>      -- line phase; tracking-line events (HD 1 in 256
                                -- lines, SD 1 in 64; a quarter of them dark)
                    if q_prime = '0' then
                        l_q <= l_q + signed(resize(c_dq, 27));
                    end if;
                    q_prime <= '0';
                    if c_lines = '0' then
                        ev_n <= (others => '0');
                    elsif ev_n = 0 then
                        if (c_jm = '1' and l_rng(23 downto 16) = 0)
                           or (c_jm = '0' and l_rng(21 downto 16) = 0) then
                            if l_rng(25 downto 24) = "00" then
                                ev_k <= '1';
                                ev_n <= to_unsigned(2, 3);
                            else
                                ev_k <= '0';
                                ev_n <= to_unsigned(5, 3);
                            end if;
                            ev_a <= l_rng(27 downto 26);
                        end if;
                    else
                        ev_n <= ev_n - 1;
                    end if;
                    q_st <= to_unsigned(36, 6);

                when 36 =>      -- Tear: V-hold slip (HD 1 in 256 lines, SD 1 in
                                -- 64) up to half a frame; jitter +-3 px (SD
                                -- +-1), held on half the lines
                    if c_tear = '1' then
                        if (c_jm = '1' and l_rng(7 downto 0) = 0)
                           or (c_jm = '0' and l_rng(5 downto 0) = 0) then
                            l_q <= l_q + resize(signed(l_rng(31 downto 8)), 27);
                        end if;
                        if l_rng(15) = '0' then
                            if c_jm = '1' then
                                l_jm <= l_rng(13 downto 12);
                            else
                                l_jm <= '0' & l_rng(12);
                            end if;
                            l_jn <= l_rng(14);
                        end if;
                    else
                        l_jm <= (others => '0');
                    end if;
                    mul_go <= '1';
                    q_st   <= to_unsigned(37, 6);

                when 37 =>      -- the fin saw (Flip: reversed)
                    if mul_go = '0' and mul_busy = '0' then
                        v_p := unsigned(m_acc(29 downto 14)) + c_tS(23 downto 8);
                        if c_flip = '1' then
                            l_s12 <= not v_p(15 downto 4);
                        else
                            l_s12 <= v_p(15 downto 4);
                        end if;
                        mul_go <= '1';
                        q_st   <= to_unsigned(38, 6);
                    end if;

                when 38 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_p1   <= unsigned(m_acc(29 downto 14)) + c_t1(23 downto 8);
                        mul_go <= '1';
                        q_st   <= to_unsigned(39, 6);
                    end if;

                when 39 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_p2   <= unsigned(m_acc(29 downto 14)) + c_t2(23 downto 8);
                        mul_go <= '1';
                        q_st   <= to_unsigned(40, 6);
                    end if;

                when 40 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_p3   <= unsigned(m_acc(29 downto 14)) + c_t3(23 downto 8);
                        mul_go <= '1';
                        q_st   <= to_unsigned(41, 6);
                    end if;

                when 41 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_p4   <= unsigned(m_acc(29 downto 14)) + c_t4(23 downto 8);
                        mul_go <= '1';
                        q_st   <= to_unsigned(42, 6);
                    end if;

                when 42 =>      -- sin 1 in flight; s^2
                    if mul_go = '0' and mul_busy = '0' then
                        l_p5   <= unsigned(m_acc(29 downto 14)) + c_t5(23 downto 8);
                        q_sa   <= l_p1(15 downto 6);
                        mul_go <= '1';
                        q_st   <= to_unsigned(43, 6);
                    end if;

                when 43 =>      -- (1 - s)^2; sin 2 in flight
                    if mul_go = '0' and mul_busy = '0' then
                        l_s2   <= unsigned(m_acc(23 downto 8));
                        l_sin1 <= q_sr;
                        q_sa   <= l_p2(15 downto 6);
                        mul_go <= '1';
                        q_st   <= to_unsigned(44, 6);
                    end if;

                when 44 =>      -- Flare . (1 - s)^2; sin 3
                    if mul_go = '0' and mul_busy = '0' then
                        l_r    <= unsigned(m_acc(23 downto 8));
                        l_sin2 <= q_sr;
                        q_sa   <= l_p3(15 downto 6);
                        mul_go <= '1';
                        q_st   <= to_unsigned(45, 6);
                    end if;

                when 45 =>      -- b = 1 + Flare (1 - s)^2 (x 2^12); a = 1 + 0.34 sin1
                                -- (x 2^15); Lobes . a; sin 4
                    if mul_go = '0' and mul_busy = '0' then
                        l_b    <= to_unsigned(4096, 14) + unsigned(m_acc(25 downto 12));
                        l_sin3 <= q_sr;
                        q_sa   <= l_p4(15 downto 6);
                        v_t    := resize(shift_right(l_sin1, 2), 17) + resize(shift_right(l_sin1, 4), 17)
                                  + resize(shift_right(l_sin1, 5), 17) + 32768;
                        l_a    <= unsigned(v_t(15 downto 0));
                        mul_go <= '1';
                        q_st   <= to_unsigned(46, 6);
                    end if;

                when 46 =>      -- . b; sin 5
                    if mul_go = '0' and mul_busy = '0' then
                        l_v1   <= unsigned(m_acc(25 downto 11));
                        l_sin4 <= q_sr;
                        q_sa   <= l_p5(15 downto 6);
                        mul_go <= '1';
                        q_st   <= to_unsigned(47, 6);
                    end if;

                when 47 =>      -- v (clamped to 16 cycles / tile); Lash . s^2
                    if mul_go = '0' and mul_busy = '0' then
                        v_vv := unsigned(m_acc(28 downto 12));
                        if v_vv(16) = '1' then
                            l_v <= (others => '1');
                        else
                            l_v <= v_vv(15 downto 0);
                        end if;
                        l_sin5 <= q_sr;
                        mul_go <= '1';
                        q_st   <= to_unsigned(48, 6);
                    end if;

                when 48 =>      -- the waves' shares: w 0.3 turn, L 0.15, hue 0.12;
                                -- the wiggle
                    if mul_go = '0' and mul_busy = '0' then
                        l_ws   <= unsigned(m_acc(23 downto 8));
                        l_ww   <= shift_right(l_sin2, 1) + shift_right(l_sin2, 4)
                                  + shift_right(l_sin2, 5) + shift_right(l_sin2, 7);   -- 0.3 turn
                        l_Lw   <= resize(shift_right(l_sin3, 10) + shift_right(l_sin3, 13), 8);  -- 0.14
                        l_hw   <= resize(shift_right(l_sin4, 10), 8);                         -- 0.125
                        mul_go <= '1';
                        q_st   <= to_unsigned(49, 6);
                    end if;

                when 49 =>      -- w, the window level, the hue; jitter . kT
                    if mul_go = '0' and mul_busy = '0' then
                        l_wg   <= unsigned(m_acc(23 downto 0));
                        l_w16  <= l_ws + unsigned(l_ww);
                        l_E    <= resize(c_L0, 12) + resize(l_Lw, 12) - 288;
                        l_hue  <= unsigned(l_hw) + c_th(23 downto 16);
                        mul_go <= '1';
                        q_st   <= to_unsigned(50, 6);
                    end if;

                when 50 =>      -- start phase = -c . kT + wiggle
                    if mul_go = '0' and mul_busy = '0' then
                        l_jt <= unsigned(m_acc(23 downto 0));
                        l_a0 <= c_nc + l_wg;
                        q_st <= to_unsigned(51, 6);
                    end if;

                when 51 =>      -- +- jitter
                    if l_jn = '1' then
                        l_a0 <= l_a0 - l_jt;
                    else
                        l_a0 <= l_a0 + l_jt;
                    end if;
                    q_st <= to_unsigned(52, 6);

                when 52 =>      -- hand the line over, with its tracking-line tint
                                -- (magenta: 1/4 3/4 1 1/2 1/4 of a 96..192 peak;
                                -- dark: >> 1 then >> 2)
                    v_pk := to_unsigned(96, 8) + (ev_a & "00000");
                    v_tc := (others => '0');
                    n_dk <= (others => '0');
                    if ev_k = '0' then
                        case to_integer(ev_n) is
                            when 5      => v_tc := shift_right(v_pk, 2);
                            when 4      => v_tc := shift_right(v_pk, 1) + shift_right(v_pk, 2);
                            when 3      => v_tc := v_pk;
                            when 2      => v_tc := shift_right(v_pk, 1);
                            when 1      => v_tc := shift_right(v_pk, 2);
                            when others => v_tc := (others => '0');
                        end case;
                    else
                        if ev_n = 2 then
                            n_dk <= to_unsigned(1, 2);
                        elsif ev_n = 1 then
                            n_dk <= to_unsigned(2, 2);
                        end if;
                    end if;
                    n_tc  <= v_tc;
                    n_ty  <= shift_right(v_tc, 1);
                    n_a0  <= l_a0;
                    n_v   <= l_v;
                    n_w   <= l_w16;
                    n_E   <= l_E;
                    n_hue <= l_hue;
                    q_st  <= to_unsigned(0, 6);

                when others =>
                    q_st <= to_unsigned(0, 6);
            end case;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Pixel path.  S0 = the clock where the accumulator holds the pixel's
    -- tile phase and g_avid the pixel's avid; S17 = the output register.
    -- ====================================================================
    p_pix : process(clk)
        variable v_m  : unsigned(15 downto 0);
        variable v_T  : unsigned(14 downto 0);
        variable v_d  : unsigned(9 downto 0);
        variable v_c  : signed(10 downto 0);
        variable v_y  : unsigned(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_v  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- S0: the line's tile phase (loaded at g_go) and constants
            if g_go = '1' then
                a_m    <= n_a0;
                px_v   <= n_v;
                px_w   <= n_w;
                px_E   <= n_E;
                px_hue <= n_hue;
                px_tc  <= n_tc;
                px_ty  <= n_ty;
                px_dk  <= n_dk;
            else
                a_m <= a_m + resize(c_kT, 24);
            end if;

            -- S1: T = |1 - 2m| (1 at the spine); the slave's sync phase u
            v_m := a_m(23 downto 8);
            if v_m(15) = '1' then
                v_T := v_m(14 downto 0);
            else
                v_T := not v_m(14 downto 0);
            end if;
            if c_mir = '1' then
                p1_u <= v_T(14 downto 3);
            else
                p1_u <= v_m(15 downto 4);
            end if;
            p1_T8 <= v_T(14 downto 7);

            -- S2: u . v, split into v's high and low bytes; 1.625 T
            p2_ph <= p1_u * px_v(15 downto 8);
            p2_pl <= p1_u * px_v(7 downto 0);
            p2_cT <= resize(p1_T8, 10) + resize(p1_T8(7 downto 1), 10) + resize(p1_T8(7 downto 3), 10);
            p2_T7 <= p1_T8(7 downto 1);

            -- S3: the product (turns x 2^24); window E = 1.625 T + L - 288
            p3_pr <= shift_left(resize(p2_ph, 28), 8) + resize(p2_pl, 28);
            p3_E  <= signed(resize(p2_cT, 12)) + px_E;
            p3_T7 <= p2_T7;

            -- S4: phi = w + u . v
            p4_phi <= px_w + p3_pr(23 downto 8);
            p4_E   <= p3_E;
            p4_T7  <= p3_T7;

            -- S5: cosine (block RAM)
            r5_c  <= C_COS(to_integer(p4_phi(15 downto 6)));
            p5_E  <= p4_E;
            p5_T7 <= p4_T7;

            -- S6: into fabric
            s6_f  <= signed(r5_c);
            p6_E  <= p5_E;
            p6_T7 <= p5_T7;

            -- S7: G = (F + 1)/2 + 1.625 (T - 1) + L, x 256 (the +128 is in E)
            p7_g  <= resize(s6_f, 12) + p6_E;
            p7_T7 <= p6_T7;

            -- S8: lit where G > 0; index share G/2 (Rings: 2G)
            if p7_g > 0 then
                p8_sh <= '1';
            else
                p8_sh <= '0';
            end if;
            if c_rings = '1' then
                p8_gs <= unsigned(p7_g(6 downto 0)) & '0';
            else
                p8_gs <= unsigned(p7_g(8 downto 1));
            end if;
            p8_T7 <= p7_T7;

            -- S9: palette addresses (flame: palette & index + hue; ground:
            -- palette & T)
            p9_sa <= c_pal & (p8_gs + px_hue);
            p9_ba <= c_pal & p8_T7;
            p9_sh <= p8_sh;

            -- S10: palette ROMs (block RAM)
            r10_sy <= C_SY(to_integer(p9_sa));
            r10_su <= C_SU(to_integer(p9_sa));
            r10_sv <= C_SV(to_integer(p9_sa));
            r10_by <= C_BY(to_integer(p9_ba));
            r10_bu <= C_BU(to_integer(p9_ba));
            r10_bv <= C_BV(to_integer(p9_ba));
            p10_sh <= p9_sh;

            -- S11: into fabric
            t11_sy <= unsigned(r10_sy);
            t11_su <= unsigned(r10_su);
            t11_sv <= unsigned(r10_sv);
            t11_by <= unsigned(r10_by);
            t11_bu <= unsigned(r10_bu);
            t11_bv <= unsigned(r10_bv);
            p11_sh <= p10_sh;

            -- S12: flame or ground, 10-bit
            if p11_sh = '1' then
                t12_y <= t11_sy & "00";
                t12_u <= t11_su & "00";
                t12_v <= t11_sv & "00";
            else
                t12_y <= t11_by & "00";
                t12_u <= t11_bu & "00";
                t12_v <= t11_bv & "00";
            end if;

            -- S13: dark tracking line: luma and chroma toward neutral
            if px_dk = 0 then
                t13_y <= t12_y;
                t13_u <= t12_u;
                t13_v <= t12_v;
            else
                v_d := t12_y - 64;
                v_c := signed(resize(t12_u, 11)) - 512;
                if px_dk = 1 then
                    t13_y <= 64 + shift_right(v_d, 1);
                    v_c   := shift_right(v_c, 1);
                else
                    t13_y <= 64 + shift_right(v_d, 2);
                    v_c   := shift_right(v_c, 2);
                end if;
                v_c   := v_c + 512;
                t13_u <= unsigned(v_c(9 downto 0));
                v_c := signed(resize(t12_v, 11)) - 512;
                if px_dk = 1 then
                    v_c := shift_right(v_c, 1);
                else
                    v_c := shift_right(v_c, 2);
                end if;
                v_c   := v_c + 512;
                t13_v <= unsigned(v_c(9 downto 0));
            end if;

            -- S14: magenta tracking line: chroma up on both axes, luma lifted
            v_y := resize(t13_y, 11) + resize(px_ty, 11);
            v_u := resize(t13_u, 11) + resize(px_tc, 11);
            v_v := resize(t13_v, 11) + resize(px_tc, 11);
            if v_y > 940 then t14_y <= to_unsigned(940, 10); else t14_y <= v_y(9 downto 0); end if;
            if v_u > 960 then t14_u <= to_unsigned(960, 10); else t14_u <= v_u(9 downto 0); end if;
            if v_v > 960 then t14_v <= to_unsigned(960, 10); else t14_v <= v_v(9 downto 0); end if;
        end if;
    end process p_pix;

    -- ====================================================================
    -- S15..S17: output conditioning and the blanking gate
    -- ====================================================================
    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S15/S16: horizontal [1 2 1]/4 (g1 is the centre pixel)
            g1_y <= t14_y;  g2_y <= g1_y;
            g1_u <= t14_u;  g2_u <= g1_u;
            g1_v <= t14_v;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(t14_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(t14_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(t14_v, 12);
            p16_y <= v_sy(11 downto 2);
            p16_u <= v_su(11 downto 2);
            p16_v <= v_sv(11 downto 2);

            -- S17: luma 64 + 13/16 (y - 64) (white ~775); neutral outside
            -- the generated avid
            v_d  := p16_y - 64;
            v_yo := p16_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if sd(16)(3) = '0' then
                s_out_y <= C_BLK;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo;
                s_out_u <= p16_u;
                s_out_v <= p16_v;
            end if;

            sd(1) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            for k in 2 to 17 loop
                sd(k) <= sd(k - 1);
            end loop;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= sd(17)(3);
    data_out.hsync_n <= sd(17)(2);
    data_out.vsync_n <= sd(17)(1);
    data_out.field_n <= sd(17)(0);

end architecture wraith;
