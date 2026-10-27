-- Copyright (C) 2026 bluecondition
-- SPDX-License-Identifier: GPL-3.0-only
--
-- pewter.vhd  (v0.1.0) -- brushed-metal weave
--
-- PEWTER.  Rebuilt from a reference capture: brushed-metal tiles woven over
-- lit rods, grey-green, 6.3 repeats across, 8 different rows.  Every row
-- of the capture is ONE fixed waveform repeating along the line: flat
-- tiles with lag-smoothed sheen, clusters of thin bright/dark edges where
-- the tiles meet (the "rods"), a fine 8 px ripple on some tiles.  The
-- streaks are noise smeared ~25 px along the line, new on every line.
-- Synthesis: no input video is read.
--
-- PICTURE
--  1. WAVETABLE.  The 8 measured row waveforms (pewter.py) are played
--     across each line by a phase accumulator, 256 samples per period,
--     read from even/odd banks and interpolated.  Row r of the screen
--     plays waveform r mod 8 (Shuffle: a hash of r).
--  2. COORDINATES.  phi(x) = phi_c + (x - W/2) . inc,  v(y) = v_c +
--     (y - H/2) . vinc: Zoom scales both about the screen centre (centred
--     on the reference's brightest plate), Rows sets vinc / inc.  Drift
--     and Roll move phi_c / v_c every field.  Per row: Shuttle (odd rows
--     half a period over, drifting the other way), Twill (+1/8 period per
--     row), Mirror (odd rows play backwards).
--  3. SCRATCHES.  Value noise along the UNWRAPPED phase (16 knots per
--     period, a hash of the knot and a per-line seed; the seed hashes the
--     row coordinate in 1/128 rows), so the scratches ride with the
--     weave, scale with Zoom and roll with the rows.  Weighted by level
--     (black stays black, the brights a little calmer, as in the
--     capture), scaled by Brush; at Brush 33 % it matches the capture.
--  4. SHEEN + METAL.  Contrast about mid-grey (matte .. chrome), optional
--     Negative, then a luma -> chroma metal palette (Pewter is measured
--     from the capture).
--
-- ENGINE
--   Sequencer with one serial shift-add multiplier and one serial
--   divider.  Per field (first vsync edge after active video): 2^24/W,
--   the zoom curve (2^-x from a 256-entry ROM), inc, vinc, the centring
--   terms, Sheen gain, speeds.  Per active line (from the generated avid's
--   fall, ~20 clocks): the row coordinate, the row's waveform, the line's
--   noise seed (op codes into a small mix ALU) and the row's start phase.
--
-- CONTROLS
--   K1 Rows     rows per period 1 .. 16 (square law; the capture is 4.15)
--   K2 Sheen    contrast x0.4 (matte) .. x2.6 (chrome), unity at 27 %
--   K3 Brush    scratches 0 .. 3x the capture (1x at 33 %)
--   K4 Drift    bipolar sideways speed, still in the centre
--   K5 Roll     bipolar vertical speed, still in the centre
--   K6 Metal    Pewter / Steel / Silver / Gold / Brass / Copper / Bronze /
--               Verdigris
--   S7 Shuttle  odd rows half a period over, drifting the other way
--   S8 Mirror   odd rows play their waveform backwards
--   S9 Shuffle  hashed row order (no 8-row repeat)
--   S10 Twill   each row 1/8 period further on: diagonal twill
--   S11 Negative
--   P12 Zoom    16 periods across (fine mesh) .. 1.3 (two huge plates),
--               centred; the capture is at 37 %
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, fixed
-- width: the encoders pair Cb/Cr from hsync); one latency for all modes
-- (23 stages); outputs gated to neutral outside the generated avid;
-- palettes stored U/V-swapped (pewter.py); per-field work on the first
-- vsync EDGE after active video; line engine on active lines only; the
-- bottom field (field_n = '0') starts one frame line down; HD HDMI bar
-- conditioning ([1 2 1] on Y/U/V, luma to ~775) before the gate.
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
use work.pewter_pkg.all;

architecture pewter of program_top is

    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- zoom centre: 0.10 period into the brightest row (1.5 rows down)
    constant C_PHC : unsigned(29 downto 0) := to_unsigned(1677722, 30);
    constant C_VC0 : unsigned(31 downto 0) := x"01800000";

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
    signal g_avd  : std_logic := '0';

    ------------------------------------------------------------------------
    -- knob glides: 0 K1 Rows, 1 K2 Sheen, 2 K3 Brush, 3 P12 Zoom.
    -- Deadband hold + per-field ease, 10.4
    ------------------------------------------------------------------------
    signal gl_raw : unsigned(39 downto 0) := (others => '0');
    signal gl_o   : unsigned(55 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- sequencer
    ------------------------------------------------------------------------
    signal q_st : unsigned(5 downto 0) := (others => '0');
    attribute fsm_encoding : string;
    attribute fsm_encoding of q_st : signal is "none";
    signal q_fpend, q_lpend, q_prime : std_logic := '0';

    -- serial multiplier: signed a x unsigned b, n iterations
    signal mul_go, mul_busy : std_logic := '0';
    signal mul_a  : signed(25 downto 0) := (others => '0');
    signal mul_b  : unsigned(15 downto 0) := (others => '0');
    signal mul_n  : unsigned(4 downto 0) := (others => '0');
    signal m_acc, m_a : signed(41 downto 0) := (others => '0');
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
    signal c_k1, c_k2, c_k3, c_p12, c_k4, c_k5 : unsigned(9 downto 0) := (others => '0');
    signal c_metal : unsigned(2 downto 0) := (others => '0');
    signal c_shut, c_mirr, c_shuf, c_twill, c_neg : std_logic := '0';
    signal c_W2   : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal c_H2   : unsigned(9 downto 0) := to_unsigned(540, 10);
    signal c_recw : unsigned(15 downto 0) := to_unsigned(8738, 16);   -- 2^24 / W
    signal c_x8   : unsigned(9 downto 0) := (others => '0');           -- zoom, octaves x 256
    signal c_e16, c_e16f : unsigned(15 downto 0) := (others => '1');
    signal c_inc  : unsigned(19 downto 0) := (others => '0');          -- periods/px x 2^24
    signal c_c2   : unsigned(19 downto 0) := (others => '0');
    signal c_rho  : unsigned(11 downto 0) := to_unsigned(1062, 12);    -- rows/period x 256
    signal c_vinc : unsigned(23 downto 0) := (others => '0');          -- rows/frame line x 2^24
    signal c_vf   : unsigned(23 downto 0) := (others => '0');          -- rows/field line x 2^24
    signal c_hw   : unsigned(29 downto 0) := (others => '0');          -- inc . W/2
    signal c_vh   : unsigned(31 downto 0) := (others => '0');          -- vf . H/2
    signal c_g7   : unsigned(7 downto 0) := to_unsigned(64, 8);        -- Sheen, 64 = 1.0
    signal c_b6   : unsigned(5 downto 0) := (others => '0');           -- Brush
    signal c_mag  : unsigned(8 downto 0) := (others => '0');
    signal c_dir  : std_logic := '0';
    signal c_d    : unsigned(29 downto 0) := (others => '0');          -- drift phase
    signal c_vc   : unsigned(31 downto 0) := (others => '0');          -- roll position

    -- per-line engine
    signal l_v    : unsigned(31 downto 0) := (others => '0');          -- 8.24 rows
    signal l_k    : unsigned(7 downto 0) := (others => '0');
    signal l_key  : unsigned(14 downto 0) := (others => '0');
    signal l_pc   : unsigned(29 downto 0) := (others => '0');
    signal l_prof : unsigned(2 downto 0) := (others => '0');
    signal l_odd  : std_logic := '0';
    -- mix ALU: the sequencer issues one op code per state
    signal l_mop  : unsigned(2 downto 0) := (others => '0');
    signal l_h    : unsigned(15 downto 0) := (others => '0');
    -- handed to the pixel path at g_go
    signal n_start : unsigned(29 downto 0) := (others => '0');
    signal n_step  : unsigned(29 downto 0) := (others => '0');
    signal n_prof  : unsigned(2 downto 0) := (others => '0');
    signal n_seed  : unsigned(15 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pixel path
    ------------------------------------------------------------------------
    -- S0: phase accumulator, 6 period bits + 24 fraction bits
    signal px_acc  : unsigned(29 downto 0) := (others => '0');
    signal px_step : unsigned(29 downto 0) := (others => '0');
    signal px_prof : unsigned(2 downto 0) := (others => '0');
    signal px_seed : unsigned(15 downto 0) := (others => '0');

    -- wavetable S1..S6
    signal t1_ea, t1_oa : unsigned(9 downto 0) := (others => '0');
    signal t1_sw  : std_logic := '0';
    signal t1_fr  : unsigned(3 downto 0) := (others => '0');
    signal t2_e, t2_o : std_logic_vector(7 downto 0) := (others => '0');
    signal t2_sw, t3_sw : std_logic := '0';
    signal t2_fr, t3_fr, t4_fr : unsigned(3 downto 0) := (others => '0');
    signal t3_e, t3_o : unsigned(7 downto 0) := (others => '0');
    signal t4_a, t5_a : unsigned(7 downto 0) := (others => '0');
    signal t4_d   : signed(8 downto 0) := (others => '0');
    signal t5_m   : signed(13 downto 0) := (others => '0');
    signal t6_v   : unsigned(11 downto 0) := (others => '0');

    -- scratch noise S1..S11 (two hashes: knots j and j+1)
    signal z1_j0, z1_j1 : unsigned(9 downto 0) := (others => '0');
    signal z1_s   : unsigned(15 downto 0) := (others => '0');
    signal z2_a0, z2_a1, z3_a0, z3_a1, z4_a0, z4_a1 : unsigned(15 downto 0) := (others => '0');
    signal z5_a0, z5_a1, z6_a0, z6_a1 : unsigned(15 downto 0) := (others => '0');
    -- knot fraction rides S1..S7
    type t_jf is array(1 to 7) of unsigned(5 downto 0);
    signal zjf : t_jf := (others => (others => '0'));
    signal z7_h0  : unsigned(7 downto 0) := (others => '0');
    signal z7_hd  : signed(8 downto 0) := (others => '0');
    signal z8_h0  : unsigned(7 downto 0) := (others => '0');
    signal z8_hm  : signed(15 downto 0) := (others => '0');
    signal z9_n   : signed(9 downto 0) := (others => '0');
    signal z10_nb : signed(16 downto 0) := (others => '0');
    signal z11_nbg : signed(23 downto 0) := (others => '0');

    -- level path S7..S12
    signal p7_vs, p8_vs, p9_vs, p10_vs, p11_vs : signed(11 downto 0) := (others => '0');
    signal p7_g, p8_g, p9_g, p10_g : unsigned(5 downto 0) := (others => '0');
    signal p12_d  : signed(13 downto 0) := (others => '0');
    -- sheen S13..S16
    signal p13_pl, p13_ph : signed(18 downto 0) := (others => '0');
    signal p14_sm : signed(22 downto 0) := (others => '0');
    signal p15_w  : signed(17 downto 0) := (others => '0');
    signal p16_y  : unsigned(9 downto 0) := (others => '0');
    -- luma + metal S17..S20
    signal p17_y, p18_y  : unsigned(9 downto 0) := (others => '0');
    signal p17_ya, p18_yb, p19_yc : unsigned(10 downto 0) := (others => '0');
    signal p17_pa : unsigned(8 downto 0) := (others => '0');
    signal p18_u, p18_v : std_logic_vector(7 downto 0) := x"80";
    signal p19_u, p19_v : unsigned(7 downto 0) := x"80";
    signal p20_y, p20_u, p20_v : unsigned(9 downto 0) := (others => '0');
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
    -- rises, when the pixel path loads the line's oscillator.
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
            g_avd <= g_avid;

            if s_vs_go = '1' then
                g_wm1 <= s_meas_w - 1;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Knob glides: the hold follows the pot only beyond a +-4 deadband
    -- (snapping to the rails), the output eases toward it once per field
    -- (by 1/8; the Zoom slider by 1/4 so the gesture stays quick)
    -- ====================================================================
    gl_raw <= unsigned(registers_in(7)) & unsigned(registers_in(2))
              & unsigned(registers_in(1)) & unsigned(registers_in(0));

    g_glide : for i in 0 to 3 generate
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
                        if i = 3 then
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
    -- Serial multiplier (shift-add, one bit of b per clock)
    -- ====================================================================
    p_mul : process(clk)
    begin
        if rising_edge(clk) then
            if mul_go = '1' then
                m_acc    <= (others => '0');
                m_a      <= resize(mul_a, 42);
                m_b      <= mul_b;
                m_cnt    <= mul_n;
                mul_busy <= '1';
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
    -- Mix ALU for the line engine's hashes: applies the op code the
    -- sequencer issued on the previous clock (16-bit, wrapping).
    --   1: a + a<<5   2: a xor a>>9   3: a + a<<2       (mix_b)
    --   4: a xor a>>7 5: a + a<<4 + a<<9                 (mix_c)
    --   6: load key + 0x5A3C   7: load k.257 + 0x3C5A
    -- ====================================================================
    p_mix : process(clk)
    begin
        if rising_edge(clk) then
            case to_integer(l_mop) is
                when 1 => l_h <= l_h + shift_left(l_h, 5);
                when 2 => l_h <= l_h xor shift_right(l_h, 9);
                when 3 => l_h <= l_h + shift_left(l_h, 2);
                when 4 => l_h <= l_h xor shift_right(l_h, 7);
                when 5 => l_h <= l_h + shift_left(l_h, 4) + shift_left(l_h, 9);
                when 6 => l_h <= ('0' & l_key) + x"5A3C";
                when 7 => l_h <= (l_k & l_k) + x"3C5A";
                when others => null;
            end case;
        end if;
    end process p_mix;

    -- ====================================================================
    -- Sequencer: per-field set-up (states 1..27) and the per-line engine
    -- (40..58).  One small operation per state.  A unit is free when its
    -- go is low and busy is low (go is registered, busy follows one clock
    -- later), so a wait state can follow its issue state directly.
    -- ====================================================================
    p_seq : process(clk)
        variable v_k6  : unsigned(9 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_ad  : unsigned(9 downto 0);
        variable v_c   : unsigned(23 downto 0);
        variable v_g   : unsigned(15 downto 0);
        variable v_spd : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            mul_go <= '0';
            div_go <= '0';
            l_mop  <= (others => '0');
            if s_vs_go = '1' then
                q_fpend <= '1';
            end if;
            if g_avd = '1' and g_avid = '0' then
                q_lpend <= '1';                       -- an active line ended
            end if;

            case to_integer(q_st) is
                when 0 =>
                    if q_fpend = '1' then
                        q_fpend <= '0';
                        q_lpend <= '0';
                        q_st    <= to_unsigned(1, 6);
                    elsif q_lpend = '1' then
                        q_lpend <= '0';
                        q_st    <= to_unsigned(40, 6);
                    end if;

                ------------------------------------------------------------
                -- per field
                ------------------------------------------------------------
                when 1 =>       -- latch the controls; start 2^24 / W
                    c_k1  <= gl_o(13 downto 4);
                    c_k2  <= gl_o(27 downto 18);
                    c_k3  <= gl_o(41 downto 32);
                    c_p12 <= gl_o(55 downto 46);
                    c_k4  <= unsigned(registers_in(3));
                    c_k5  <= unsigned(registers_in(4));
                    -- labelled knob: 0..7 at program load is the label index
                    v_k6 := unsigned(registers_in(5));
                    if v_k6 < 8 then
                        c_metal <= v_k6(2 downto 0);
                    else
                        c_metal <= v_k6(9 downto 7);
                    end if;
                    c_shut  <= registers_in(6)(0);
                    c_mirr  <= registers_in(6)(1);
                    c_shuf  <= registers_in(6)(2);
                    c_twill <= registers_in(6)(3);
                    c_neg   <= registers_in(6)(4);
                    c_W2    <= s_meas_w(11 downto 1);
                    c_H2    <= s_meas_h(10 downto 1);
                    div_den <= s_meas_w;
                    div_k   <= to_unsigned(25, 5);
                    div_go  <= '1';
                    q_st    <= to_unsigned(2, 6);

                when 2 =>       -- W reciprocal; zoom in octaves (0 .. 3.62)
                    if div_go = '0' and div_busy = '0' then
                        if d_q(24 downto 16) /= 0 then
                            c_recw <= (others => '1');
                        else
                            c_recw <= d_q(15 downto 0);
                        end if;
                        c_x8 <= c_p12 - resize(c_p12(9 downto 4), 10)
                                - resize(c_p12(9 downto 5), 10);
                        q_st <= to_unsigned(3, 6);
                    end if;

                when 3 =>       -- 2^(-fraction) (ROM)
                    c_e16 <= unsigned(C_EXP(to_integer(c_x8(7 downto 0))));
                    q_st  <= to_unsigned(4, 6);

                when 4 =>       -- into fabric
                    c_e16f <= c_e16;
                    q_st   <= to_unsigned(5, 6);

                when 5 =>       -- 2^24/W x 2^-f
                    mul_a  <= signed(resize(c_recw, 26));
                    mul_b  <= c_e16f;
                    mul_n  <= to_unsigned(16, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(6, 6);

                when 6 =>       -- inc = 16 cols x 2^24/W x 2^-x
                    if mul_go = '0' and mul_busy = '0' then
                        case to_integer(c_x8(9 downto 8)) is
                            when 0 => c_inc <= unsigned(m_acc(31 downto 12));
                            when 1 => c_inc <= unsigned(m_acc(32 downto 13));
                            when 2 => c_inc <= unsigned(m_acc(33 downto 14));
                            when others => c_inc <= unsigned(m_acc(34 downto 15));
                        end case;
                        q_st <= to_unsigned(7, 6);
                    end if;

                when 7 =>       -- Rows squared
                    mul_a  <= signed(resize(c_k1, 26));
                    mul_b  <= resize(c_k1, 16);
                    mul_n  <= to_unsigned(10, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(8, 6);

                when 8 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_c2 <= unsigned(m_acc(19 downto 0));
                        q_st <= to_unsigned(9, 6);
                    end if;

                when 9 =>       -- rows/period x 256 = 256 + 15 k1^2 / 4096 (1 .. 16)
                    v_c   := shift_left(resize(c_c2, 24), 4) - resize(c_c2, 24);
                    c_rho <= to_unsigned(256, 12) + resize(v_c(23 downto 12), 12);
                    q_st  <= to_unsigned(10, 6);

                when 10 =>      -- vinc = inc x rho / 256
                    mul_a  <= signed(resize(c_inc, 26));
                    mul_b  <= resize(c_rho, 16);
                    mul_n  <= to_unsigned(12, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(11, 6);

                when 11 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_vinc <= unsigned(m_acc(31 downto 8));
                        q_st   <= to_unsigned(12, 6);
                    end if;

                when 12 =>      -- per field line (x2 interlaced); centring inc x W/2
                    if s_ilace = '1' then
                        c_vf <= c_vinc(22 downto 0) & '0';
                    else
                        c_vf <= c_vinc;
                    end if;
                    mul_a  <= signed(resize(c_inc, 26));
                    mul_b  <= resize(c_W2, 16);
                    mul_n  <= to_unsigned(11, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(13, 6);

                when 13 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_hw <= unsigned(m_acc(29 downto 0));
                        q_st <= to_unsigned(14, 6);
                    end if;

                when 14 =>      -- vf x H/2 (field lines)
                    mul_a  <= signed(resize(c_vf, 26));
                    mul_b  <= resize(c_H2, 16);
                    mul_n  <= to_unsigned(10, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(15, 6);

                when 15 =>      -- Sheen x0.4 .. x2.6 (64 = 1); Brush 0 .. 63
                    if mul_go = '0' and mul_busy = '0' then
                        c_vh <= unsigned(m_acc(31 downto 0));
                        v_g  := shift_left(resize(c_k2, 16), 5) + shift_left(resize(c_k2, 16), 1)
                                + resize(c_k2, 16);
                        c_g7 <= to_unsigned(26, 8) + v_g(15 downto 8);
                        c_b6 <= c_k3(9 downto 4);
                        q_st <= to_unsigned(16, 6);
                    end if;

                when 16 =>      -- Drift: bipolar with a +-24 still zone
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
                    q_st <= to_unsigned(17, 6);

                when 17 =>
                    mul_a  <= signed(resize(c_mag, 26));
                    mul_b  <= resize(c_mag, 16);
                    mul_n  <= to_unsigned(9, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(18, 6);

                when 18 =>      -- speed = 2 mag^2 (up to ~1.7 periods/s); positive
                                -- Drift moves the weave RIGHT
                    if mul_go = '0' and mul_busy = '0' then
                        v_spd := unsigned(m_acc(18 downto 0)) & '0';
                        if c_dir = '0' then
                            c_d <= c_d - resize(v_spd, 30);
                        else
                            c_d <= c_d + resize(v_spd, 30);
                        end if;
                        q_st <= to_unsigned(19, 6);
                    end if;

                when 19 =>      -- Roll: the same curve
                    v_d := signed(resize(c_k5, 11)) - 512;
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
                    q_st <= to_unsigned(20, 6);

                when 20 =>
                    mul_a  <= signed(resize(c_mag, 26));
                    mul_b  <= resize(c_mag, 16);
                    mul_n  <= to_unsigned(9, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(21, 6);

                when 21 =>      -- positive Roll moves the rows DOWN
                    if mul_go = '0' and mul_busy = '0' then
                        v_spd := unsigned(m_acc(18 downto 0)) & '0';
                        if c_dir = '0' then
                            c_vc <= c_vc - resize(v_spd, 32);
                        else
                            c_vc <= c_vc + resize(v_spd, 32);
                        end if;
                        q_st <= to_unsigned(22, 6);
                    end if;

                when 22 =>      -- field start: v = vc + VC0 - vf.H/2 ...
                    l_v  <= c_vc - c_vh;
                    q_st <= to_unsigned(23, 6);

                when 23 =>
                    l_v  <= l_v + C_VC0;
                    q_st <= to_unsigned(24, 6);

                when 24 =>      -- ... and the bottom field one frame line down
                    if s_ilace = '1' and s_timing.field_n = '0' then
                        l_v <= l_v + resize(c_vinc, 32);
                    end if;
                    q_prime <= '1';
                    q_st    <= to_unsigned(40, 6);

                ------------------------------------------------------------
                -- per line (for the NEXT line)
                ------------------------------------------------------------
                when 40 =>
                    if q_prime = '0' then
                        l_v <= l_v + resize(c_vf, 32);
                    end if;
                    q_prime <= '0';
                    q_st    <= to_unsigned(41, 6);

                when 41 =>      -- row number; noise key = row coordinate in 1/128 rows
                    l_k   <= l_v(31 downto 24);
                    l_odd <= l_v(24);
                    l_key <= l_v(31 downto 17);
                    l_mop <= to_unsigned(7, 3);             -- shuffle hash of k
                    q_st  <= to_unsigned(42, 6);

                when 42 =>
                    l_mop <= to_unsigned(1, 3);
                    -- row centre phase: Shuttle puts odd rows half a period
                    -- over and drifts them the other way
                    if c_shut = '1' and l_odd = '1' then
                        l_pc <= to_unsigned(8388608, 30) - c_d;
                    else
                        l_pc <= c_d;
                    end if;
                    q_st <= to_unsigned(43, 6);

                when 43 =>
                    l_mop <= to_unsigned(2, 3);
                    l_pc  <= l_pc + C_PHC;
                    q_st  <= to_unsigned(44, 6);

                when 44 =>      -- Twill: 1/8 period further per row
                    l_mop <= to_unsigned(3, 3);
                    if c_twill = '1' then
                        l_pc <= l_pc + shift_left(resize(l_k, 30), 21);
                    end if;
                    q_st <= to_unsigned(45, 6);

                when 45 =>      -- Mirror: odd rows play backwards
                    l_mop <= to_unsigned(4, 3);
                    if c_mirr = '1' and l_odd = '1' then
                        n_start <= l_pc + c_hw;
                        n_step  <= (not resize(c_inc, 30)) + 1;
                    else
                        n_start <= l_pc - c_hw;
                        n_step  <= resize(c_inc, 30);
                    end if;
                    q_st <= to_unsigned(46, 6);

                when 46 =>
                    l_mop <= to_unsigned(5, 3);
                    q_st  <= to_unsigned(47, 6);

                when 47 =>      -- (mix_c lands this clock)
                    q_st <= to_unsigned(48, 6);

                when 48 =>      -- waveform; then the line's noise seed
                    if c_shuf = '1' then
                        n_prof <= l_h(15 downto 13);
                    else
                        n_prof <= l_k(2 downto 0);
                    end if;
                    l_mop <= to_unsigned(6, 3);
                    q_st  <= to_unsigned(49, 6);

                when 49 => l_mop <= to_unsigned(1, 3); q_st <= to_unsigned(50, 6);
                when 50 => l_mop <= to_unsigned(2, 3); q_st <= to_unsigned(51, 6);
                when 51 => l_mop <= to_unsigned(3, 3); q_st <= to_unsigned(52, 6);
                when 52 => l_mop <= to_unsigned(4, 3); q_st <= to_unsigned(53, 6);
                when 53 => l_mop <= to_unsigned(5, 3); q_st <= to_unsigned(54, 6);
                when 54 => l_mop <= to_unsigned(1, 3); q_st <= to_unsigned(55, 6);
                when 55 => l_mop <= to_unsigned(2, 3); q_st <= to_unsigned(56, 6);
                when 56 => l_mop <= to_unsigned(3, 3); q_st <= to_unsigned(57, 6);
                when 57 => q_st <= to_unsigned(58, 6);   -- (last op lands)

                when 58 =>
                    n_seed <= l_h;
                    q_st   <= to_unsigned(0, 6);

                when others =>
                    q_st <= to_unsigned(0, 6);
            end case;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Pixel path.  S0 = the clock where px_acc holds the pixel's phase and
    -- g_avid the pixel's avid; S23 = the output register.
    -- ====================================================================
    p_pix : process(clk)
        variable v_n, v_n1 : unsigned(7 downto 0);
        variable v_j       : unsigned(9 downto 0);
        variable v_a       : unsigned(15 downto 0);
        variable v_v       : signed(13 downto 0);
        variable v_l       : unsigned(7 downto 0);
        variable v_n9      : signed(9 downto 0);
        variable v_d       : signed(13 downto 0);
        variable v_w       : signed(17 downto 0);
        variable v_y       : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S0: the line's oscillator (loaded at g_go)
            if g_go = '1' then
                px_acc  <= n_start;
                px_step <= n_step;
                px_prof <= n_prof;
                px_seed <= n_seed;
            else
                px_acc <= px_acc + px_step;
            end if;

            -- S1: wavetable addresses (samples n and n+1 from the even /
            -- odd banks); noise knots j, j+1 along the unwrapped phase
            v_n   := px_acc(23 downto 16);
            v_n1  := v_n + 1;
            t1_ea <= px_prof & v_n1(7 downto 1);
            t1_oa <= px_prof & v_n(7 downto 1);
            t1_sw <= v_n(0);
            t1_fr <= px_acc(15 downto 12);
            v_j   := px_acc(29 downto 20);
            z1_j0 <= v_j;
            z1_j1 <= v_j + 1;
            z1_s  <= px_seed;
            zjf(1) <= px_acc(19 downto 14);
            for i in 2 to 7 loop
                zjf(i) <= zjf(i - 1);
            end loop;

            -- S2: wavetable ROMs (block RAM); hash = (seed xor j.rot) + j
            t2_e  <= C_TE(to_integer(t1_ea));
            t2_o  <= C_TO(to_integer(t1_oa));
            t2_sw <= t1_sw;
            t2_fr <= t1_fr;
            z2_a0 <= (z1_s xor (z1_j0 & z1_j0(9 downto 4))) + resize(z1_j0, 16);
            z2_a1 <= (z1_s xor (z1_j1 & z1_j1(9 downto 4))) + resize(z1_j1, 16);

            -- S3: into fabric; mix_b: a + a<<5
            t3_e  <= unsigned(t2_e);
            t3_o  <= unsigned(t2_o);
            t3_sw <= t2_sw;
            t3_fr <= t2_fr;
            z3_a0 <= z2_a0 + shift_left(z2_a0, 5);
            z3_a1 <= z2_a1 + shift_left(z2_a1, 5);

            -- S4: a = sample n, d = sample n+1 - sample n; mix_b: xor, + a<<2
            if t3_sw = '0' then
                t4_a <= t3_e;
                t4_d <= signed('0' & t3_o) - signed('0' & t3_e);
            else
                t4_a <= t3_o;
                t4_d <= signed('0' & t3_e) - signed('0' & t3_o);
            end if;
            t4_fr <= t3_fr;
            v_a   := z3_a0 xor shift_right(z3_a0, 9);
            z4_a0 <= v_a + shift_left(v_a, 2);
            v_a   := z3_a1 xor shift_right(z3_a1, 9);
            z4_a1 <= v_a + shift_left(v_a, 2);

            -- S5: d x fraction; mix_c: xor a>>7
            t5_a  <= t4_a;
            t5_m  <= t4_d * signed('0' & t4_fr);
            z5_a0 <= z4_a0 xor shift_right(z4_a0, 7);
            z5_a1 <= z4_a1 xor shift_right(z4_a1, 7);

            -- S6: level 8.4 = 16 a + d f; mix_c: + a<<4 + a<<9
            v_v   := signed(resize(t5_a, 14) sll 4) + t5_m;
            t6_v  <= unsigned(v_v(11 downto 0));
            z6_a0 <= z5_a0 + shift_left(z5_a0, 4) + shift_left(z5_a0, 9);
            z6_a1 <= z5_a1 + shift_left(z5_a1, 4) + shift_left(z5_a1, 9);

            -- S7: knot values h0, h1 - h0; level weight (black stays black,
            -- the brights a little calmer); level - 2048 (flip the MSB)
            z7_h0 <= z6_a0(15 downto 8);
            z7_hd <= signed('0' & z6_a1(15 downto 8)) - signed('0' & z6_a0(15 downto 8));
            v_l   := t6_v(11 downto 4);
            if v_l(7 downto 6) = "00" then
                p7_g <= v_l(5 downto 0);
            else
                p7_g <= to_unsigned(63, 6) - resize(v_l(7 downto 3) - 8, 6);
            end if;
            p7_vs <= signed((not t6_v(11)) & t6_v(10 downto 0));

            -- S8: (h1 - h0) x knot fraction
            z8_h0 <= z7_h0;
            z8_hm <= z7_hd * signed('0' & zjf(7));
            p8_vs <= p7_vs;
            p8_g  <= p7_g;

            -- S9: noise -128..127
            v_n9 := signed(resize(z8_h0, 10)) - 128 + resize(shift_right(z8_hm, 6), 10);
            z9_n <= v_n9;
            p9_vs <= p8_vs;
            p9_g  <= p8_g;

            -- S10: x Brush
            z10_nb <= z9_n * signed('0' & c_b6);
            p10_vs <= p9_vs;
            p10_g  <= p9_g;

            -- S11: x level weight
            z11_nbg <= z10_nb * signed('0' & p10_g);
            p11_vs  <= p10_vs;

            -- S12: centred level + scratches
            p12_d <= resize(p11_vs, 14) + resize(shift_right(z11_nbg, 10), 14);

            -- S13: Sheen, the gain split into nibbles
            p13_pl <= p12_d * signed('0' & c_g7(3 downto 0));
            p13_ph <= p12_d * signed('0' & c_g7(7 downto 4));

            -- S14
            p14_sm <= resize(p13_pl, 23) + shift_left(resize(p13_ph, 23), 4);

            -- S15: back about mid-grey (gain 64 = 1)
            v_w := resize(shift_right(p14_sm, 6), 18) + 2048;
            p15_w <= v_w;

            -- S16: clamp to 10 bits; Negative
            if p15_w < 0 then
                v_y := (others => '0');
            elsif p15_w > 4095 then
                v_y := (others => '1');
            else
                v_y := unsigned(p15_w(11 downto 2));
            end if;
            if c_neg = '1' then
                p16_y <= not v_y;
            else
                p16_y <= v_y;
            end if;

            -- S17: metal palette address; luma 64 + y . 0.859 (white 940)
            p17_pa <= c_metal & p16_y(9 downto 4);
            p17_y  <= p16_y;
            p17_ya <= resize(p16_y, 11) - resize(p16_y(9 downto 3), 11);

            -- S18: palette ROMs (block RAM)
            p18_u  <= C_PU(to_integer(p17_pa));
            p18_v  <= C_PV(to_integer(p17_pa));
            p18_y  <= p17_y;
            p18_yb <= p17_ya - resize(p17_y(9 downto 6), 11);

            -- S19: into fabric
            p19_u  <= unsigned(p18_u);
            p19_v  <= unsigned(p18_v);
            p19_yc <= p18_yb + 64;

            -- S20: 10-bit colour
            p20_y <= p19_yc(9 downto 0);
            p20_u <= p19_u & "00";
            p20_v <= p19_v & "00";
        end if;
    end process p_pix;

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

end architecture pewter;
