-- tallow.vhd  (v0.3.0) -- melting rainbow columns
--
-- TALLOW.  An analog colourizer patch rebuilt in one pass: a horizontal
-- oscillator, hard-synced to every line and frequency-modulated by a
-- vertical LFO, keys the picture into RAINBOW and STRIPE columns; a
-- horizontal lag smears the key; the smeared key plus a vertical ramp
-- index a two-ramp palette.  Synthesis: no input video is read.
--
-- PICTURE
--  1. OSCILLATOR.  phase(x) = ph0 + x . f(line), f = fa0 . (1 + Bend . lfo),
--     lfo a vertical wave: Drip = -cos of a skewed phase (slow rise over
--     80 % of the cycle, fast fall over 20 %: saw-like leaning edges that
--     hook back, with NO seam -- v0.1's saw reset broke every edge along a
--     horizontal line) or Sine.  Hard sync means the left columns stand
--     still and the right ones swing: every edge is a scaled copy of the
--     LFO.  Anchor Centre puts the still point mid-screen (ph0 = -f.W/2).
--  2. KEY.  w = tri(phase) [Twin: (tri(phase) + tri(2.phase + psi)) / 2],
--     key = (w >= Width threshold).  key 0 = rainbow, 1 = stripes.
--     Noise: the threshold moves +-192 by a value-noise field indexed by
--     the column number (the phase's integer bits, so the noise rides the
--     bending columns) and a vertical cell (6 per frame, smoothstep-
--     interpolated, drifting with Speed): every column thins and swells
--     on its own, as in the reference photo.
--  3. LAG.  Ks += (key - Ks) >> s per pixel in ONE adder: the target is
--     all-ones or zero, so key - Ks is ~Ks shifted with the fill bit = not
--     key (+ carry-in).  Smear sets s; an ordered x-dither between s and
--     s+1 makes the knob continuous.  The tail bends the stripes into each
--     edge, and the fast sweep through the palette draws the fringe lines.
--  4. PALETTE.  n = ramp(y) + Ks (9.4 bits).  Entries 0..255 are the
--     rainbow ramp, 256..511 the stripe ramp (key 1 adds 256).  Even/odd
--     bank ROMs give entries n and n+1 on one clock; the fraction
--     interpolates.  The x3 palettes fold the ramp (up, down, up).
--
-- ENGINE
--   A small sequencer with one serial multiplier and one serial divider.
--   Per field (first vsync edge after active video): 2^24/W, 2^20/H, the
--   control curves, base frequency, FM amplitude, LFO step, speed.  Per
--   active line (from the generated avid's fall, ~50 clocks): the LFO
--   (parabolic sine by multiply), f, and the centre-anchor phase.
--
-- CONTROLS
--   K1 Columns  1 .. 32 columns across (square law)
--   K2 Width    rainbow share of each column, 6 .. 92 %
--   K3 Smear    lag 1 .. 512 px (exponential)
--   K4 Speed    LFO roll, bipolar, still in the centre (positive = down)
--   K5 Bends    LFO cycles down the frame, 0.25 .. 6 (square law)
--   K6 Palette  Signal / Ember / Glacier / Acid, each Single or x3 (the ramp
--               folded up-down-up)
--   S7 Wave     Drip (lean + quick hook, seamless) / Sine
--   S8 Twin     second-harmonic key: columns split into pairs
--   S9 Jitter   per-line 0..7 px start jitter (analog tearing)
--   S10 Anchor  Left / Centre
--   S11 Noise   per-column width noise (thinner here, wider there)
--   P12 Bend    FM depth 6 .. 84 %: a gentle sway at 0, giant zigzag fans at 100 %
--               (capped: near 100 % depth f stalls at the wave's trough and a
--               blank band opens)
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, fixed
-- width: the encoders pair Cb/Cr from hsync); one latency for all modes
-- (19 stages); outputs gated to neutral outside the generated avid;
-- palette stored U/V-swapped (tallow.py); per-field work on the first
-- vsync EDGE after active video; line engine on active lines only; bottom
-- field (field_n = '0') starts half a field line down; HD HDMI bar
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
use work.tallow_pkg.all;

architecture tallow of program_top is

    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- 9-bit triangle of a 16-bit phase: 0 at phase 0, 511 at one half
    function f_tri(p : unsigned(15 downto 0)) return unsigned is
        variable v_t : unsigned(8 downto 0);
    begin
        if p(15) = '0' then
            v_t := p(14 downto 6);
        else
            v_t := not p(14 downto 6);
        end if;
        return v_t;
    end function;

    function f_onehot(s : unsigned(3 downto 0)) return std_logic_vector is
        variable v_o : std_logic_vector(9 downto 0) := (others => '0');
    begin
        for i in 0 to 9 loop
            if to_integer(s) = i then
                v_o(i) := '1';
            end if;
        end loop;
        return v_o;
    end function;

    -- smoothstep 3t^2 - 2t^3, t = i/64, scaled to 0..64 (noise interpolation)
    type t_int64 is array(0 to 63) of integer range 0 to 64;
    constant C_SMOOTH : t_int64 := (
         0,  0,  0,  0,  1,  1,  2,  2,  3,  3,  4,  5,  6,  7,  8,  9,
        10, 11, 12, 14, 15, 16, 17, 19, 20, 22, 23, 25, 26, 28, 29, 31,
        32, 33, 35, 36, 38, 39, 41, 42, 44, 45, 47, 48, 49, 50, 52, 53,
        54, 55, 56, 57, 58, 59, 60, 61, 61, 62, 62, 63, 63, 64, 64, 64);

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
    -- knob glides: 0 K1 Columns, 1 K2 Width, 2 K3 Smear, 3 K5 Bends,
    -- 4 P12 Bend.  Deadband hold + per-field ease, 10.4
    ------------------------------------------------------------------------
    -- (flat vectors: an array-of-unsigned type would make every
    -- `unsigned & literal` inside a conversion ambiguous)
    signal gl_raw : unsigned(49 downto 0) := (others => '0');
    signal gl_o   : unsigned(69 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- sequencer
    ------------------------------------------------------------------------
    signal q_st : unsigned(5 downto 0) := (others => '0');
    attribute fsm_encoding : string;
    attribute fsm_encoding of q_st : signal is "none";
    signal q_fpend, q_lpend, q_prime : std_logic := '0';

    -- serial multiplier: signed a x unsigned b, n iterations
    signal mul_go, mul_busy : std_logic := '0';
    signal mul_a  : signed(23 downto 0) := (others => '0');
    signal mul_b  : unsigned(13 downto 0) := (others => '0');
    signal mul_n  : unsigned(3 downto 0) := (others => '0');
    signal m_acc, m_a : signed(39 downto 0) := (others => '0');
    signal m_b    : unsigned(13 downto 0) := (others => '0');
    signal m_cnt  : unsigned(3 downto 0) := (others => '0');

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
    signal c_pal  : unsigned(1 downto 0) := (others => '0');
    signal c_sine, c_twin, c_jit, c_ctr, c_band3, c_noise : std_logic := '0';
    signal c_tn   : unsigned(24 downto 0) := (others => '0');         -- noise drift
    signal c_jm   : std_logic := '1';                         -- jitter 0..7 (else 0..3)
    signal c_W2   : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal c_recw : unsigned(15 downto 0) := to_unsigned(8738, 16);   -- 2^24 / W
    signal c_dy   : unsigned(12 downto 0) := to_unsigned(970, 13);    -- 2^20 / field lines
    signal c_c2   : unsigned(19 downto 0) := (others => '0');
    signal c_c31  : unsigned(24 downto 0) := (others => '0');
    signal c_cols : unsigned(13 downto 0) := to_unsigned(1536, 14);   -- columns x 256
    signal c_fa0  : unsigned(21 downto 0) := (others => '0');         -- cycles/px x 2^24
    signal c_A, c_Am : unsigned(21 downto 0) := (others => '0');      -- FM amplitude, fa0 - A
    signal c_bd   : unsigned(10 downto 0) := (others => '0');         -- FM depth x 1024
    signal c_fv   : unsigned(10 downto 0) := to_unsigned(256, 11);    -- LFO cycles/frame x 256
    signal c_dphi : unsigned(23 downto 0) := (others => '0');         -- LFO step per line
    signal c_mag  : unsigned(8 downto 0) := (others => '0');
    signal c_dir  : std_logic := '0';
    signal c_spd  : unsigned(18 downto 0) := (others => '0');
    signal c_tv   : unsigned(23 downto 0) := x"8C0000";              -- LFO time phase
    signal c_psi  : unsigned(15 downto 0) := (others => '0');         -- Twin phase
    signal c_th   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal c_smv  : unsigned(7 downto 0) := (others => '0');
    signal c_ohs, c_ohs1 : std_logic_vector(9 downto 0) := "0000010000";
    signal c_sfr  : unsigned(3 downto 0) := (others => '0');

    -- per-line engine
    signal l_ry   : unsigned(19 downto 0) := (others => '0');
    signal l_phv  : unsigned(23 downto 0) := (others => '0');
    signal l_x    : signed(11 downto 0) := (others => '0');
    signal l_dw   : signed(12 downto 0) := (others => '0');
    signal l_w    : unsigned(12 downto 0) := (others => '0');
    signal l_yb   : unsigned(21 downto 0) := (others => '0');
    signal l_sin  : signed(9 downto 0) := (others => '0');
    signal l_lfo  : unsigned(9 downto 0) := (others => '0');
    signal l_f    : unsigned(22 downto 0) := (others => '0');
    signal l_ph0  : unsigned(29 downto 0) := (others => '0');
    signal l_yn6  : unsigned(22 downto 0) := (others => '0');
    signal l_yn   : unsigned(24 downto 0) := (others => '0');         -- noise y, 2^20 / cell
    signal l_nje, l_njo : unsigned(3 downto 0) := (others => '0');
    signal l_nsw  : std_logic := '0';
    signal l_ns   : unsigned(6 downto 0) := (others => '0');
    signal l_lfsr : std_logic_vector(15 downto 0) := x"ACE1";
    -- handed to the pixel path at g_go
    signal n_f    : unsigned(22 downto 0) := (others => '0');
    signal n_ph0  : unsigned(29 downto 0) := (others => '0');
    signal n_nje, n_njo : unsigned(3 downto 0) := (others => '0');
    signal n_nsw  : std_logic := '0';
    signal n_ns   : unsigned(6 downto 0) := (others => '0');
    signal n_yi   : unsigned(11 downto 0) := (others => '0');
    signal n_j    : unsigned(2 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pixel path
    ------------------------------------------------------------------------
    -- S0: phase accumulator: 24 fraction bits + a 6-bit column number
    signal px_acc : unsigned(29 downto 0) := (others => '0');
    signal px_f   : unsigned(22 downto 0) := (others => '0');
    signal px_yi  : unsigned(11 downto 0) := (others => '0');
    signal px_j   : unsigned(2 downto 0) := (others => '0');
    signal px_x4  : unsigned(3 downto 0) := (others => '0');
    signal px_ls  : std_logic := '0';
    signal px_nje, px_njo : unsigned(3 downto 0) := (others => '0');
    signal px_nsw : std_logic := '0';
    signal px_ns  : unsigned(6 downto 0) := (others => '0');

    -- key path S1..S8 (the waveform waits at S4..S7 for the noise)
    signal p1_t1  : unsigned(8 downto 0) := (others => '0');
    signal p1_ph2 : unsigned(15 downto 0) := (others => '0');
    signal p2_t1, p2_t2 : unsigned(8 downto 0) := (others => '0');
    signal p3_w, p4_w, p5_w, p6_w, p7_w : unsigned(8 downto 0) := (others => '0');
    -- {line start, x4 dither phase} ride along S1..S8
    type t_xl is array(1 to 8) of std_logic_vector(4 downto 0);
    signal xl : t_xl := (others => (others => '0'));
    signal p8_key : std_logic := '0';
    signal p8_oh  : std_logic_vector(9 downto 0) := (others => '0');

    -- noise path S1..S7
    signal z1_ea, z1_oa : unsigned(9 downto 0) := (others => '0');
    signal z1_sw, z2_sw, z3_sw : std_logic := '0';
    signal z1_s, z2_s, z3_s, z4_s : unsigned(6 downto 0) := (others => '0');
    signal z2_e, z2_o : std_logic_vector(7 downto 0) := (others => '0');
    signal z3_e, z3_o : unsigned(7 downto 0) := (others => '0');
    signal z4_a, z5_a : unsigned(7 downto 0) := (others => '0');
    signal z4_d   : signed(8 downto 0) := (others => '0');
    signal z5_m   : signed(16 downto 0) := (others => '0');
    signal z6_c   : signed(9 downto 0) := (others => '0');
    signal z7_th  : signed(11 downto 0) := (others => '0');

    -- lag and colour S9..S16
    signal p9_ks  : unsigned(15 downto 0) := (others => '0');
    signal p10_n  : unsigned(12 downto 0) := (others => '0');
    signal p11_ea, p11_oa : unsigned(9 downto 0) := (others => '0');
    signal p11_sw : std_logic := '0';
    signal p11_fr : unsigned(3 downto 0) := (others => '0');
    signal p12_ey, p12_eu, p12_ev, p12_oy, p12_ou, p12_ov : std_logic_vector(7 downto 0) := (others => '0');
    signal p12_sw : std_logic := '0';
    signal p12_fr : unsigned(3 downto 0) := (others => '0');
    signal p13_ey, p13_eu, p13_ev, p13_oy, p13_ou, p13_ov : unsigned(7 downto 0) := (others => '0');
    signal p13_sw : std_logic := '0';
    signal p13_fr : unsigned(3 downto 0) := (others => '0');
    signal p14_ay, p14_au, p14_av : unsigned(7 downto 0) := (others => '0');
    signal p14_dy, p14_du, p14_dv : signed(8 downto 0) := (others => '0');
    signal p14_fr : unsigned(3 downto 0) := (others => '0');
    signal p15_ay, p15_au, p15_av : unsigned(7 downto 0) := (others => '0');
    signal p15_my, p15_mu, p15_mv : signed(13 downto 0) := (others => '0');
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := (others => '0');
    -- output conditioning (the HD HDMI bar fix, damier v0.4.4): [1 2 1]
    -- horizontal smoothing on Y/U/V, luma scaled toward black
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- syncs (generated avid, hsync, vsync, field) from S1 to S19
    type t_sd is array(1 to 19) of std_logic_vector(3 downto 0);
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
    -- (by 1/8; the Bend slider by 1/4 so the gesture stays quick)
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
    -- Serial multiplier (shift-add, one bit of b per clock)
    -- ====================================================================
    p_mul : process(clk)
    begin
        if rising_edge(clk) then
            if mul_go = '1' then
                m_acc    <= (others => '0');
                m_a      <= resize(mul_a, 40);
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
    -- Sequencer: per-field set-up (states 1..22) and the per-line engine
    -- (32..43).  One small operation per state.  A unit is free when its
    -- go is low and busy is low (go is registered, busy follows one clock
    -- later), so a wait state can follow its issue state directly.
    -- ====================================================================
    p_seq : process(clk)
        variable v_k6  : unsigned(9 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_ad  : unsigned(9 downto 0);
        variable v_ax  : unsigned(12 downto 0);
        variable v_p   : signed(39 downto 0);
        variable v_lfo : signed(10 downto 0);
        variable v_s   : unsigned(3 downto 0);
        variable v_yi  : unsigned(13 downto 0);
        variable v_j, v_j1 : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            mul_go <= '0';
            div_go <= '0';
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
                    -- labelled knob: 0..3 at program load is the label index
                    -- 8 positions: palette x {Single, x3}
                    v_k6 := unsigned(registers_in(5));
                    if v_k6 < 8 then
                        c_pal   <= v_k6(2 downto 1);
                        c_band3 <= v_k6(0);
                    else
                        c_pal   <= v_k6(9 downto 8);
                        c_band3 <= v_k6(7);
                    end if;
                    c_sine  <= registers_in(6)(0);
                    c_twin  <= registers_in(6)(1);
                    c_jit   <= registers_in(6)(2);
                    c_ctr   <= registers_in(6)(3);
                    c_noise <= registers_in(6)(4);
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

                when 3 =>       -- line step; Columns squared
                    if div_go = '0' and div_busy = '0' then
                        if d_q(24 downto 13) /= 0 then
                            c_dy <= (others => '1');
                        else
                            c_dy <= d_q(12 downto 0);
                        end if;
                        mul_a  <= signed(resize(c_k1, 24));
                        mul_b  <= resize(c_k1, 14);
                        mul_n  <= to_unsigned(10, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(4, 6);
                    end if;

                when 4 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_c2 <= unsigned(m_acc(19 downto 0));
                        q_st <= to_unsigned(5, 6);
                    end if;

                when 5 =>       -- c2 x 31
                    c_c31 <= shift_left(resize(c_c2, 25), 5) - resize(c_c2, 25);
                    q_st  <= to_unsigned(6, 6);

                when 6 =>       -- columns x 256 = 256 + 31 k1^2 / 4096 (1 .. 32)
                    c_cols <= to_unsigned(256, 14) + resize(c_c31(24 downto 12), 14);
                    q_st   <= to_unsigned(7, 6);

                when 7 =>       -- fa0 = columns x 2^24 / W
                    mul_a  <= signed(resize(c_recw, 24));
                    mul_b  <= c_cols;
                    mul_n  <= to_unsigned(14, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(8, 6);

                when 8 =>       -- Bend depth 64 .. 863 / 1024: never quite still
                                -- (so Bends / Wave / Speed always show) and never
                                -- so deep that f stalls into a blank band
                    if mul_go = '0' and mul_busy = '0' then
                        c_fa0 <= unsigned(m_acc(29 downto 8));
                        c_bd  <= to_unsigned(64, 11) + resize(c_p12, 11)
                                 - resize(c_p12(9 downto 2), 11)
                                 + resize(c_p12(9 downto 5), 11);
                        q_st  <= to_unsigned(9, 6);
                    end if;

                when 9 =>       -- FM amplitude A = fa0 x depth
                    mul_a  <= signed(resize(c_fa0, 24));
                    mul_b  <= resize(c_bd, 14);
                    mul_n  <= to_unsigned(11, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(10, 6);

                when 10 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_A  <= unsigned(m_acc(31 downto 10));
                        q_st <= to_unsigned(11, 6);
                    end if;

                when 11 =>      -- fa0 - A; Bends squared
                    c_Am   <= c_fa0 - c_A;
                    mul_a  <= signed(resize(c_k5, 24));
                    mul_b  <= resize(c_k5, 14);
                    mul_n  <= to_unsigned(10, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(12, 6);

                when 12 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_c2 <= unsigned(m_acc(19 downto 0));
                        q_st <= to_unsigned(13, 6);
                    end if;

                when 13 =>      -- LFO cycles/frame x 256 = 64 + 1.5 k5^2 / 1024
                    c_fv <= to_unsigned(64, 11) + resize(c_c2(19 downto 10), 11)
                            + resize(c_c2(19 downto 11), 11);
                    q_st <= to_unsigned(14, 6);

                when 14 =>      -- LFO step per line = dy x fv / 16
                    mul_a  <= signed(resize(c_dy, 24));
                    mul_b  <= resize(c_fv, 14);
                    mul_n  <= to_unsigned(11, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(15, 6);

                when 15 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_dphi <= unsigned(m_acc(27 downto 4));
                        q_st   <= to_unsigned(16, 6);
                    end if;

                when 16 =>      -- Speed: bipolar with a +-24 still zone
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
                    mul_a  <= signed(resize(c_mag, 24));
                    mul_b  <= resize(c_mag, 14);
                    mul_n  <= to_unsigned(9, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(18, 6);

                when 18 =>      -- speed = 2 mag^2 (up to ~1.7 LFO cycles/s)
                    if mul_go = '0' and mul_busy = '0' then
                        c_spd <= unsigned(m_acc(17 downto 0)) & '0';
                        q_st  <= to_unsigned(19, 6);
                    end if;

                when 19 =>      -- positive speed rolls the shapes DOWN
                    if c_dir = '0' then
                        c_tv  <= c_tv - resize(c_spd, 24);
                        c_psi <= c_psi - resize(c_spd(18 downto 10), 16);
                        c_tn  <= c_tn - resize(c_spd(18 downto 3), 25);
                    else
                        c_tv  <= c_tv + resize(c_spd, 24);
                        c_psi <= c_psi + resize(c_spd(18 downto 10), 16);
                        c_tn  <= c_tn + resize(c_spd(18 downto 3), 25);
                    end if;
                    q_st <= to_unsigned(20, 6);

                when 20 =>      -- Width threshold 31 .. 471; Smear 0 .. 143 (s.f)
                    c_th  <= to_unsigned(31, 9) + resize(c_k2(9 downto 1), 9)
                             - resize(c_k2(9 downto 4), 9) - resize(c_k2(9 downto 7), 9);
                    c_smv <= resize(shift_right(shift_left(resize(c_k3, 14), 3)
                                                + resize(c_k3, 14), 6), 8);
                    q_st  <= to_unsigned(21, 6);

                when 21 =>
                    v_s    := c_smv(7 downto 4);
                    c_ohs  <= f_onehot(v_s);
                    c_ohs1 <= f_onehot(v_s + 1);
                    c_sfr  <= c_smv(3 downto 0);
                    q_st   <= to_unsigned(22, 6);

                when 22 =>      -- field start; prime the line engine for line 0
                    if s_ilace = '1' and s_timing.field_n = '0' then
                        -- bottom field: half a field line down
                        l_ry  <= resize(c_dy(12 downto 1), 20);
                        l_phv <= c_tv + ('0' & c_dphi(23 downto 1));
                    else
                        l_ry  <= (others => '0');
                        l_phv <= c_tv;
                    end if;
                    q_prime <= '1';
                    q_st    <= to_unsigned(32, 6);

                ------------------------------------------------------------
                -- per line (for the NEXT line)
                ------------------------------------------------------------
                when 32 =>
                    if q_prime = '0' then
                        l_ry  <= l_ry + c_dy;
                        l_phv <= l_phv + c_dphi;
                    end if;
                    q_prime <= '0';
                    q_st    <= to_unsigned(33, 6);

                when 33 =>      -- LFO phase u; Drip's fall starts at u = 0.8
                    l_x  <= signed(l_phv(23 downto 12));
                    l_dw <= signed(resize(l_phv(23 downto 12), 13)) - 3277;
                    l_yn6 <= shift_left(resize(l_ry, 23), 2) + shift_left(resize(l_ry, 23), 1);
                    if c_band3 = '1' then
                        l_yb <= resize(l_ry, 22) + shift_left(resize(l_ry, 22), 1);
                    else
                        l_yb <= resize(l_ry, 22);
                    end if;
                    q_st <= to_unsigned(42, 6);

                when 42 =>      -- Drip: skewed phase w (half a cycle per segment)
                    if l_dw < 0 then            -- rise: w = 0.625 u
                        l_w <= resize(l_phv(23 downto 13), 13) + resize(l_phv(23 downto 15), 13);
                    else                        -- fall: w = 2048 + 2.5 (u - 3277)
                        l_w <= unsigned(to_signed(2048, 13) + shift_left(l_dw, 1)
                                        + shift_right(l_dw, 1));
                    end if;
                    l_yn <= resize(l_yn6, 25) + c_tn;          -- 6 noise cells per frame
                    q_st <= to_unsigned(43, 6);

                when 43 =>      -- -cos(w) = sin(w - quarter cycle); noise rows
                    if c_sine = '0' then
                        l_x <= signed(l_w(11 downto 0) - 1024);
                    end if;
                    -- cell j in bank (j even ? E : O) at j/2, cell j+1 in the other
                    v_j  := l_yn(24 downto 20);
                    v_j1 := v_j + 1;
                    l_nje <= v_j1(4 downto 1);
                    l_njo <= v_j(4 downto 1);
                    l_nsw <= v_j(0);
                    l_ns  <= to_unsigned(C_SMOOTH(to_integer(l_yn(19 downto 14))), 7);
                    q_st <= to_unsigned(34, 6);

                when 34 =>      -- parabolic sine: x (2048 - |x|)
                    v_ax   := unsigned(abs(resize(l_x, 13)));
                    mul_a  <= resize(l_x, 24);
                    mul_b  <= resize(to_unsigned(2048, 13) - v_ax, 14);
                    mul_n  <= to_unsigned(12, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(35, 6);

                when 35 =>
                    if mul_go = '0' and mul_busy = '0' then
                        v_p := shift_right(m_acc, 11);
                        if v_p > 511 then
                            l_sin <= to_signed(511, 10);
                        elsif v_p < -511 then
                            l_sin <= to_signed(-511, 10);
                        else
                            l_sin <= v_p(9 downto 0);
                        end if;
                        q_st <= to_unsigned(36, 6);
                    end if;

                when 36 =>      -- offset binary 1 .. 1023
                    v_lfo := resize(l_sin, 11) + 512;
                    l_lfo <= unsigned(v_lfo(9 downto 0));
                    q_st  <= to_unsigned(37, 6);

                when 37 =>      -- f = (fa0 - A) + A lfo / 512
                    mul_a  <= signed(resize(c_A, 24));
                    mul_b  <= resize(l_lfo, 14);
                    mul_n  <= to_unsigned(10, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(38, 6);

                when 38 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_f  <= resize(c_Am, 23) + unsigned(m_acc(31 downto 9));
                        q_st <= to_unsigned(39, 6);
                    end if;

                when 39 =>
                    if c_ctr = '1' then        -- still point mid-screen
                        mul_a  <= signed(resize(l_f, 24));
                        mul_b  <= resize(c_W2, 14);
                        mul_n  <= to_unsigned(11, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(40, 6);
                    else
                        l_ph0 <= (others => '0');
                        q_st  <= to_unsigned(41, 6);
                    end if;

                when 40 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_ph0 <= unsigned(-m_acc(29 downto 0));
                        q_st  <= to_unsigned(41, 6);
                    end if;

                when 41 =>      -- hand the line over; x3 palettes fold the ramp
                    n_f   <= l_f;
                    n_ph0 <= l_ph0;
                    n_nje <= l_nje;
                    n_njo <= l_njo;
                    n_nsw <= l_nsw;
                    n_ns  <= l_ns;
                    v_yi  := l_yb(21 downto 8);
                    if v_yi(12) = '1' then
                        n_yi <= not v_yi(11 downto 0);
                    else
                        n_yi <= v_yi(11 downto 0);
                    end if;
                    if c_jit = '0' then
                        n_j <= (others => '0');
                    elsif c_jm = '1' then
                        n_j <= unsigned(l_lfsr(2 downto 0));
                    else
                        n_j <= unsigned('0' & l_lfsr(1 downto 0));
                    end if;
                    if l_lfsr(0) = '1' then
                        l_lfsr <= ('0' & l_lfsr(15 downto 1)) xor x"B400";
                    else
                        l_lfsr <= '0' & l_lfsr(15 downto 1);
                    end if;
                    q_st <= to_unsigned(0, 6);

                when others =>
                    q_st <= to_unsigned(0, 6);
            end case;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Pixel path.  S0 = the clock where px_acc holds the pixel's phase and
    -- g_avid the pixel's avid; S19 = the output register.
    -- ====================================================================
    p_pix : process(clk)
        variable v_ws  : unsigned(9 downto 0);
        variable v_br  : unsigned(3 downto 0);
        variable v_k   : unsigned(5 downto 0);
        variable v_zc  : signed(9 downto 0);
        variable v_nk  : unsigned(15 downto 0);
        variable v_op  : unsigned(15 downto 0);
        variable v_sum : unsigned(16 downto 0);
        variable v_n   : unsigned(8 downto 0);
        variable v_n1  : unsigned(8 downto 0);
        variable v_cy, v_cu, v_cv : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- S0: the line's oscillator (hard sync at g_go); Jitter holds
            -- the start phase for 0..7 pixels
            if g_go = '1' then
                px_acc <= n_ph0;
                px_f   <= n_f;
                px_yi  <= n_yi;
                px_j   <= n_j;
                px_x4  <= (others => '0');
                px_nje <= n_nje;
                px_njo <= n_njo;
                px_nsw <= n_nsw;
                px_ns  <= n_ns;
            else
                if px_j /= 0 then
                    px_j <= px_j - 1;
                else
                    px_acc <= px_acc + resize(px_f, 30);
                end if;
                px_x4 <= px_x4 + 1;
            end if;
            px_ls <= g_go;

            -- S1: triangle of the phase; twin phase 2 phi + psi; noise
            -- addresses (column number = the phase's integer bits)
            p1_t1  <= f_tri(px_acc(23 downto 8));
            p1_ph2 <= (px_acc(22 downto 8) & '0') + c_psi;
            v_k    := px_acc(29 downto 24);
            z1_ea  <= px_nje & v_k;
            z1_oa  <= px_njo & v_k;
            z1_sw  <= px_nsw;
            z1_s   <= px_ns;
            xl(1)  <= px_ls & std_logic_vector(px_x4);
            for i in 2 to 8 loop
                xl(i) <= xl(i - 1);
            end loop;

            -- S2: twin triangle; noise ROMs (block RAM)
            p2_t1 <= p1_t1;
            p2_t2 <= f_tri(p1_ph2);
            z2_e  <= C_NE(to_integer(z1_ea));
            z2_o  <= C_NO(to_integer(z1_oa));
            z2_sw <= z1_sw;
            z2_s  <= z1_s;

            -- S3: key waveform; noise into fabric
            if c_twin = '1' then
                v_ws := resize(p2_t1, 10) + resize(p2_t2, 10);
                p3_w <= v_ws(9 downto 1);
            else
                p3_w <= p2_t1;
            end if;
            z3_e  <= unsigned(z2_e);
            z3_o  <= unsigned(z2_o);
            z3_sw <= z2_sw;
            z3_s  <= z2_s;

            -- S4: noise a = cell j, d = cell j+1 - cell j
            p4_w <= p3_w;
            if z3_sw = '0' then
                z4_a <= z3_e;
                z4_d <= signed('0' & z3_o) - signed('0' & z3_e);
            else
                z4_a <= z3_o;
                z4_d <= signed('0' & z3_e) - signed('0' & z3_o);
            end if;
            z4_s <= z3_s;

            -- S5: d x smoothstep
            p5_w <= p4_w;
            z5_a <= z4_a;
            z5_m <= z4_d * signed('0' & z4_s);

            -- S6: centred noise -128..127
            p6_w <= p5_w;
            v_zc := signed(resize(z5_a, 10)) - 128 + resize(shift_right(z5_m, 6), 10);
            z6_c <= v_zc;

            -- S7: threshold, +-1.5 x noise when Noise is on
            p7_w <= p6_w;
            if c_noise = '1' then
                z7_th <= signed(resize(c_th, 12)) + resize(z6_c, 12)
                         + resize(shift_right(z6_c, 1), 12);
            else
                z7_th <= signed(resize(c_th, 12));
            end if;

            -- S8: key; lag shift for this pixel (ordered dither s / s+1)
            if signed(resize(p7_w, 12)) >= z7_th then
                p8_key <= '1';
            else
                p8_key <= '0';
            end if;
            v_br := xl(7)(0) & xl(7)(1) & xl(7)(2) & xl(7)(3);
            if v_br < c_sfr then
                p8_oh <= c_ohs1;
            else
                p8_oh <= c_ohs;
            end if;

            -- S9: the lag.  key 1: Ks + (~Ks >> s); key 0: Ks - (Ks >> s),
            -- written as Ks + ~(Ks >> s) + 1.  One 17-bit add carries the +1.
            if xl(8)(4) = '1' then
                p9_ks <= (others => p8_key);          -- settled at the left edge
            else
                v_nk := not p9_ks;
                v_op := (others => '0');
                for i in 0 to 9 loop
                    if p8_oh(i) = '1' then
                        if p8_key = '1' then
                            v_op := v_op or shift_right(v_nk, i);
                        else
                            v_op := v_op or not shift_right(p9_ks, i);
                        end if;
                    end if;
                end loop;
                v_sum := (p9_ks & '1') + (v_op & (not p8_key));
                p9_ks <= v_sum(16 downto 1);
            end if;

            -- S10: palette index n = ramp + key (9.4)
            p10_n <= resize(px_yi, 13) + resize(p9_ks(15 downto 4), 13);

            -- S11: even/odd bank addresses for entries n, n+1
            v_n    := p10_n(12 downto 4);
            v_n1   := v_n + 1;
            p11_ea <= c_pal & v_n1(8 downto 1);
            p11_oa <= c_pal & v_n(8 downto 1);
            p11_sw <= v_n(0);
            p11_fr <= p10_n(3 downto 0);

            -- S12: palette ROMs (block RAM)
            p12_ey <= C_EY(to_integer(p11_ea));
            p12_eu <= C_EU(to_integer(p11_ea));
            p12_ev <= C_EV(to_integer(p11_ea));
            p12_oy <= C_OY(to_integer(p11_oa));
            p12_ou <= C_OU(to_integer(p11_oa));
            p12_ov <= C_OV(to_integer(p11_oa));
            p12_sw <= p11_sw;
            p12_fr <= p11_fr;

            -- S13: into fabric before any arithmetic
            p13_ey <= unsigned(p12_ey);
            p13_eu <= unsigned(p12_eu);
            p13_ev <= unsigned(p12_ev);
            p13_oy <= unsigned(p12_oy);
            p13_ou <= unsigned(p12_ou);
            p13_ov <= unsigned(p12_ov);
            p13_sw <= p12_sw;
            p13_fr <= p12_fr;

            -- S14: a = entry n, b = entry n+1; d = b - a
            if p13_sw = '0' then
                p14_ay <= p13_ey;
                p14_au <= p13_eu;
                p14_av <= p13_ev;
                p14_dy <= signed('0' & p13_oy) - signed('0' & p13_ey);
                p14_du <= signed('0' & p13_ou) - signed('0' & p13_eu);
                p14_dv <= signed('0' & p13_ov) - signed('0' & p13_ev);
            else
                p14_ay <= p13_oy;
                p14_au <= p13_ou;
                p14_av <= p13_ov;
                p14_dy <= signed('0' & p13_ey) - signed('0' & p13_oy);
                p14_du <= signed('0' & p13_eu) - signed('0' & p13_ou);
                p14_dv <= signed('0' & p13_ev) - signed('0' & p13_ov);
            end if;
            p14_fr <= p13_fr;

            -- S15: d x fraction
            p15_ay <= p14_ay;
            p15_au <= p14_au;
            p15_av <= p14_av;
            p15_my <= p14_dy * signed('0' & p14_fr);
            p15_mu <= p14_du * signed('0' & p14_fr);
            p15_mv <= p14_dv * signed('0' & p14_fr);

            -- S16: 10-bit colour = (16 a + d f) / 4
            v_cy := signed(resize(p15_ay, 14) sll 4) + p15_my;
            v_cu := signed(resize(p15_au, 14) sll 4) + p15_mu;
            v_cv := signed(resize(p15_av, 14) sll 4) + p15_mv;
            p16_y <= unsigned(v_cy(11 downto 2));
            p16_u <= unsigned(v_cu(11 downto 2));
            p16_v <= unsigned(v_cv(11 downto 2));
        end if;
    end process p_pix;

    -- ====================================================================
    -- S17..S19: output conditioning and the blanking gate
    -- ====================================================================
    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S17/S18: horizontal [1 2 1]/4 (g1 is the centre pixel)
            g1_y <= p16_y;  g2_y <= g1_y;
            g1_u <= p16_u;  g2_u <= g1_u;
            g1_v <= p16_v;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(p16_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(p16_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(p16_v, 12);
            p18_y <= v_sy(11 downto 2);
            p18_u <= v_su(11 downto 2);
            p18_v <= v_sv(11 downto 2);

            -- S19: luma 64 + 13/16 (y - 64) (white ~775); neutral outside
            -- the generated avid
            v_d  := p18_y - 64;
            v_yo := p18_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if sd(18)(3) = '0' then
                s_out_y <= C_BLK;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo;
                s_out_u <= p18_u;
                s_out_v <= p18_v;
            end if;

            sd(1) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            for k in 2 to 19 loop
                sd(k) <= sd(k - 1);
            end loop;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= sd(19)(3);
    data_out.hsync_n <= sd(19)(2);
    data_out.vsync_n <= sd(19)(1);
    data_out.field_n <= sd(19)(0);

end architecture tallow;
