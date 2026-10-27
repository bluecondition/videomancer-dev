-- colonnade.vhd  (v0.1.0) -- chrome sheets hung before a fluted colonnade
--
-- COLONNADE.  An analog colourizer patch rebuilt from a reference capture:
-- mirrored "egg-crate" sheets of chrome contour rings (dark navy blobs
-- ringed white / yellow / red, magenta and cyan plateaus, X-shaped saddles)
-- hang in front of a colonnade of lit flutes (crimson edge, cream
-- highlight, purple, blue, black shadow).  The sheets' edges bow with the
-- melt, and each sheet's left edge carries a thin rainbow outline where the
-- key sweeps the palette.  Synthesis: no input video is read.
--
-- PICTURE
--  1. TILES.  A horizontal phase pt (Tiles cycles across, centred on the
--     screen).  Mirror folds it into a triangle so each tile is symmetric
--     about its centre; Repeat runs it as a full turn (seamless, no mirror).
--  2. FIELD.  S = sin(A + B2) + sin(A - B2) = 2 sin A cos B2 (an egg-crate:
--     peaks and troughs give rings, the zero crossings give X saddles).
--     A = the tile coordinate + a per-line wobble; B = the vertical phase
--     (0.56 x Tiles cycles down the frame, so cells stay round); B2 = B +
--     Melt . sin(2A + q/2): the melt bends the grid into liquid shapes.
--  3. SHEETS.  key = frac(pt + D(line)) < Sheet.  D carries the same
--     wobble (x4), so the sheets' edges bow as the field melts.  Lattice
--     XORs diagonal bands into the key (boxes, as in the reference's top
--     right).  The first 6 px after every rise of the key (3 on SD) sweep
--     the palette: a thin rainbow outline.
--  4. FLUTES.  A second phase pc (Flutes per tile, not mirrored: lit from
--     the left everywhere) indexes the lit-tube ramp; the field bends the
--     shading (S/4) into the reference's highlight fingers.  Glass: inside
--     the flutes the field is sampled at 1.5 x the flute phase instead, so
--     every flute shows its own compressed copy of the sheets (S x 0.62).
--  5. COLOUR.  Sheets: index = S . Contours + offset (+ Flow); flutes:
--     index = flute position + field.  512-entry palettes (256 sheet + 256
--     flute, both cyclic), even/odd banks, 4-bit interpolation.
--  6. TEAR.  Per-line index jitter (+-4 entries) and a 0..3 px start hold
--     (0..1 on SD): the reference's horizontal streaks.
--
-- ENGINE
--   A sequencer with one serial multiplier and one serial divider.  Per
--   field (first vsync edge after active video): 2^24/W, 2^20/H, the knob
--   curves, tile / flute / lattice steps, vertical steps, duty, contour
--   gain, speed and the time phases.  Per active line (from the generated
--   avid's fall, ~20 clocks): the DDAs, two wobble sines (an EBR sine ROM),
--   the melt-scaled wobble (one multiply), the key offset, the jitter.
--
-- CONTROLS
--   K1 Tiles     1 .. 8 mirror tiles across (square law)
--   K2 Flutes    2 .. 12 flutes per tile
--   K3 Sheet     sheet share of each tile, 12 .. 88 %
--   K4 Melt      0 = a crisp egg-crate grid and straight sheet edges;
--                100 % = liquid swirls and deeply bowed edges
--   K5 Speed     bipolar, still in the centre: the field rolls, the sheets
--                and flutes slide in opposite directions, the bows travel
--   K6 Palette   Chrome (fitted to the reference) / Oil / Candy / Thermal
--   S7 Mirror    Mirror / Repeat
--   S8 Lattice   diagonal box bands XOR the sheets
--   S9 Flutes    Lit (tube shading) / Glass (refracted field)
--   S10 Tear     per-line jitter streaks
--   S11 Flow     the palette flows through the contours
--   P12 Contours 0.5 .. 12 palette cycles over the field: broad smooth
--                chrome at 0, dense rainbow rings at 100 %
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, fixed
-- width: the encoders pair Cb/Cr from hsync); one latency for all modes
-- (26 stages); outputs gated to neutral outside the generated avid;
-- palette stored U/V-swapped (colonnade.py); per-field work on the first
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
use work.colonnade_pkg.all;

architecture colonnade of program_top is

    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- quarter-wave sine index of a 16-bit phase (1024 steps per turn)
    function f_qidx(p : unsigned(15 downto 0)) return unsigned is
        variable v_i : unsigned(7 downto 0);
    begin
        if p(14) = '1' then
            v_i := not p(13 downto 6);
        else
            v_i := p(13 downto 6);
        end if;
        return v_i;
    end function;

    -- rainbow outline: palette position for the n-th pixel after a key rise
    -- (HD 6 px x 43 entries, SD 3 px x 86), as a 12-bit index
    function f_oidx(c : unsigned(3 downto 0); hd : std_logic) return unsigned is
        variable v_o : unsigned(11 downto 0);
    begin
        if hd = '1' then
            v_o := resize(c * to_unsigned(43, 6), 8) & "0000";
        else
            v_o := resize(c * to_unsigned(86, 7), 8) & "0000";
        end if;
        return v_o;
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
    signal g_avd  : std_logic := '0';

    ------------------------------------------------------------------------
    -- knob glides: 0 K1 Tiles, 1 K2 Flutes, 2 K3 Sheet, 3 K4 Melt,
    -- 4 P12 Contours.  Deadband hold + per-field ease, 10.4
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

    -- line-engine sine ROM: address -> EBR -> fabric (2 clocks)
    signal ls_addr : unsigned(15 downto 0) := (others => '0');
    signal ls_q    : std_logic_vector(11 downto 0) := (others => '0');
    signal ls_r    : unsigned(11 downto 0) := (others => '0');
    signal ls_s1, ls_s2 : std_logic := '0';

    -- per-field values
    signal c_k1, c_k2, c_k3, c_k4, c_p12, c_k5 : unsigned(9 downto 0) := (others => '0');
    signal c_pal  : unsigned(1 downto 0) := (others => '0');
    signal c_mirror, c_lat, c_glass, c_tear, c_flow : std_logic := '0';
    signal c_hd   : std_logic := '1';
    signal c_recw : unsigned(15 downto 0) := to_unsigned(8738, 16);   -- 2^24 / W
    signal c_dy   : unsigned(12 downto 0) := to_unsigned(970, 13);    -- 2^20 / field lines
    signal c_c2   : unsigned(19 downto 0) := (others => '0');
    signal c_t256 : unsigned(10 downto 0) := to_unsigned(820, 11);    -- tiles x 256
    signal c_p16  : unsigned(7 downto 0) := to_unsigned(80, 8);       -- flutes/tile x 16
    signal c_Ft   : unsigned(19 downto 0) := (others => '0');         -- tile step / px (2^24 = 1)
    signal c_Fd   : unsigned(19 downto 0) := (others => '0');         -- lattice step / px
    signal c_Fc, c_Fg : unsigned(23 downto 0) := (others => '0');     -- flute, glass steps / px
    signal c_P    : unsigned(23 downto 0) := (others => '0');         -- t256 x dy
    signal c_dB, c_dPd : unsigned(23 downto 0) := (others => '0');    -- per-line steps
    signal c_dW1, c_dW2 : unsigned(23 downto 0) := (others => '0');
    signal c_ka   : unsigned(15 downto 0) := (others => '0');
    signal c_kb   : unsigned(9 downto 0) := (others => '0');
    signal c_duty : unsigned(15 downto 0) := to_unsigned(39000, 16);
    signal c_koff : unsigned(15 downto 0) := (others => '0');
    signal c_melt : unsigned(7 downto 0) := (others => '0');
    signal c_ga, c_gb : unsigned(27 downto 0) := (others => '0');
    signal c_G    : unsigned(7 downto 0) := to_unsigned(24, 8);
    signal c_mag  : unsigned(8 downto 0) := (others => '0');
    signal c_dir  : std_logic := '0';
    signal c_spd  : unsigned(18 downto 0) := (others => '0');
    signal c_tb, c_tk, c_tc, c_tw1, c_tw2 : unsigned(23 downto 0) := (others => '0');
    signal c_tp   : unsigned(11 downto 0) := (others => '0');
    signal c_pt0, c_tg0 : unsigned(23 downto 0) := (others => '0');
    signal c_off  : unsigned(11 downto 0) := to_unsigned(2580, 12);

    -- per-line engine (read by the pixel path at g_go)
    signal l_q    : unsigned(20 downto 0) := (others => '0');         -- 2^20 per frame
    signal l_B, l_W1, l_W2, l_Pd : unsigned(23 downto 0) := (others => '0');
    signal l_s1, l_s2 : signed(11 downto 0) := (others => '0');
    signal l_t5   : signed(14 downto 0) := (others => '0');
    signal l_wsum : signed(10 downto 0) := (others => '0');
    signal l_wob  : signed(10 downto 0) := (others => '0');
    signal l_A0   : unsigned(15 downto 0) := (others => '0');
    signal l_D1, l_D : unsigned(15 downto 0) := (others => '0');
    signal l_offs, l_offp : unsigned(11 downto 0) := (others => '0');
    signal l_hold : unsigned(1 downto 0) := (others => '0');
    signal l_lfsr : std_logic_vector(15 downto 0) := x"ACE1";

    ------------------------------------------------------------------------
    -- pixel path
    ------------------------------------------------------------------------
    -- S0: phase accumulators (2^24 = one turn) and the line's constants
    signal px_pt, px_pc, px_pg, px_pd : unsigned(23 downto 0) := (others => '0');
    signal px_hold : unsigned(1 downto 0) := (others => '0');
    signal px_ls  : std_logic := '0';
    signal pl_A0, pl_D, pl_B, pl_cq : unsigned(15 downto 0) := (others => '0');
    signal pl_offs, pl_offp : unsigned(11 downto 0) := (others => '0');

    signal p1_A, p1_fk, p1_g : unsigned(15 downto 0) := (others => '0');
    signal p1_s   : unsigned(11 downto 0) := (others => '0');
    signal p1_band, p1_ls : std_logic := '0';
    signal p2_A, p2_g : unsigned(15 downto 0) := (others => '0');
    signal p2_kv, p2_band, p2_ls : std_logic := '0';
    signal p3_As  : unsigned(15 downto 0) := (others => '0');
    signal p3_key, p3_ls : std_logic := '0';
    signal p4_ma, p4_As : unsigned(15 downto 0) := (others => '0');
    signal p4_kprev : std_logic := '1';
    signal p4_cnt : unsigned(3 downto 0) := (others => '1');
    signal p5_mq  : std_logic_vector(11 downto 0) := (others => '0');
    signal p5_msg : std_logic := '0';
    signal p5_As  : unsigned(15 downto 0) := (others => '0');
    signal p6_mq  : unsigned(10 downto 0) := (others => '0');
    signal p6_msg : std_logic := '0';
    signal p6_As  : unsigned(15 downto 0) := (others => '0');
    signal p7_lo, p7_hi : unsigned(14 downto 0) := (others => '0');
    signal p7_msg : std_logic := '0';
    signal p7_As  : unsigned(15 downto 0) := (others => '0');
    signal p8_disp : unsigned(13 downto 0) := (others => '0');
    signal p8_msg : std_logic := '0';
    signal p8_As  : unsigned(15 downto 0) := (others => '0');
    signal p9_B2, p9_As : unsigned(15 downto 0) := (others => '0');
    signal p10_a1, p10_a2 : unsigned(15 downto 0) := (others => '0');
    signal p11_q1, p11_q2 : std_logic_vector(11 downto 0) := (others => '0');
    signal p11_g1, p11_g2 : std_logic := '0';
    signal p12_q1, p12_q2 : unsigned(10 downto 0) := (others => '0');
    signal p12_g1, p12_g2 : std_logic := '0';
    signal p13_S  : signed(13 downto 0) := (others => '0');
    signal p14_lo, p14_hi : signed(18 downto 0) := (others => '0');
    signal p14_sp : signed(13 downto 0) := (others => '0');
    signal p15_sg : signed(22 downto 0) := (others => '0');
    signal p15_ip : unsigned(11 downto 0) := (others => '0');
    signal p16_is, p16_ip, p16_oi : unsigned(11 downto 0) := (others => '0');
    signal p16_out : std_logic := '0';
    signal p17_idx : unsigned(11 downto 0) := (others => '0');
    signal p17_grp : std_logic := '0';
    -- {key, outline count} ride S4..S16; the flute position S1..S15
    type t_kc is array(5 to 16) of std_logic_vector(4 downto 0);
    signal kc : t_kc := (others => (others => '0'));
    type t_sp is array(2 to 15) of unsigned(11 downto 0);
    signal sp : t_sp := (others => (others => '0'));

    -- palette S18..S23
    signal p18_ea, p18_oa : unsigned(9 downto 0) := (others => '0');
    signal p18_sw : std_logic := '0';
    signal p18_fr : unsigned(3 downto 0) := (others => '0');
    signal p19_ey, p19_eu, p19_ev, p19_oy, p19_ou, p19_ov : std_logic_vector(7 downto 0) := (others => '0');
    signal p19_sw : std_logic := '0';
    signal p19_fr : unsigned(3 downto 0) := (others => '0');
    signal p20_ey, p20_eu, p20_ev, p20_oy, p20_ou, p20_ov : unsigned(7 downto 0) := (others => '0');
    signal p20_sw : std_logic := '0';
    signal p20_fr : unsigned(3 downto 0) := (others => '0');
    signal p21_ay, p21_au, p21_av : unsigned(7 downto 0) := (others => '0');
    signal p21_dy, p21_du, p21_dv : signed(8 downto 0) := (others => '0');
    signal p21_fr : unsigned(3 downto 0) := (others => '0');
    signal p22_ay, p22_au, p22_av : unsigned(7 downto 0) := (others => '0');
    signal p22_my, p22_mu, p22_mv : signed(13 downto 0) := (others => '0');
    signal p23_y, p23_u, p23_v : unsigned(9 downto 0) := (others => '0');
    -- output conditioning (the HD HDMI bar fix, damier v0.4.4): [1 2 1]
    -- horizontal smoothing on Y/U/V, luma scaled toward black
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal p25_y, p25_u, p25_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- syncs (generated avid, hsync, vsync, field) from S1 to S26
    type t_sd is array(1 to 26) of std_logic_vector(3 downto 0);
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
    -- rises, when the pixel path loads the line.
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
    -- (by 1/8; the Contours slider by 1/4 so the gesture stays quick)
    -- ====================================================================
    gl_raw <= unsigned(registers_in(7)) & unsigned(registers_in(3))
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
    -- Line-engine sine ROM (its own EBR): data 2 clocks after the address
    -- ====================================================================
    p_lsin : process(clk)
    begin
        if rising_edge(clk) then
            ls_q  <= C_SINQ(to_integer(f_qidx(ls_addr)));
            ls_s1 <= ls_addr(15);
            ls_r  <= unsigned(ls_q);
            ls_s2 <= ls_s1;
        end if;
    end process p_lsin;

    -- ====================================================================
    -- Sequencer: per-field set-up (states 1..26) and the per-line engine
    -- (32..44).  One small operation per state.  A unit is free when its
    -- go is low and busy is low (go is registered, busy follows one clock
    -- later), so a wait state can follow its issue state directly.
    -- ====================================================================
    p_seq : process(clk)
        variable v_k6  : unsigned(9 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_ad  : unsigned(9 downto 0);
        variable v_c7  : unsigned(22 downto 0);
        variable v_P9  : unsigned(27 downto 0);
        variable v_g   : unsigned(27 downto 0);
        variable v_j   : signed(3 downto 0);
        variable v_s   : signed(11 downto 0);
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
                    c_k4  <= gl_o(55 downto 46);
                    c_p12 <= gl_o(69 downto 60);
                    c_k5  <= unsigned(registers_in(4));
                    -- labelled knob: 0..3 at program load is the label index
                    v_k6 := unsigned(registers_in(5));
                    if v_k6 < 4 then
                        c_pal <= v_k6(1 downto 0);
                    else
                        c_pal <= v_k6(9 downto 8);
                    end if;
                    c_mirror <= not registers_in(6)(0);
                    c_lat    <= registers_in(6)(1);
                    c_glass  <= registers_in(6)(2);
                    c_tear   <= registers_in(6)(3);
                    c_flow   <= registers_in(6)(4);
                    c_hd     <= s_meas_w(11) or s_meas_w(10);
                    div_den  <= s_meas_w;
                    div_k    <= to_unsigned(25, 5);
                    div_go   <= '1';
                    q_st     <= to_unsigned(2, 6);

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

                when 3 =>       -- line step; Tiles squared
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

                when 5 =>       -- tiles x 256 = 256 + 7 k1^2 / 4096 (1 .. 8)
                    v_c7   := shift_left(resize(c_c2, 23), 3) - resize(c_c2, 23);
                    c_t256 <= to_unsigned(256, 11) + resize(v_c7(22 downto 12), 11);
                    q_st   <= to_unsigned(6, 6);

                when 6 =>       -- tile step Ft = tiles x 2^24 / W
                    mul_a  <= signed(resize(c_recw, 24));
                    mul_b  <= resize(c_t256, 14);
                    mul_n  <= to_unsigned(11, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(7, 6);

                when 7 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_Ft <= unsigned(m_acc(27 downto 8));
                        q_st <= to_unsigned(8, 6);
                    end if;

                when 8 =>       -- flutes per tile x 16 = 32 + 160 k2 / 1024 (2 .. 12)
                    c_p16 <= to_unsigned(32, 8)
                             + resize(shift_right(shift_left(resize(c_k2, 18), 7)
                                                  + shift_left(resize(c_k2, 18), 5), 10), 8);
                    q_st  <= to_unsigned(9, 6);

                when 9 =>       -- flute step Fc = Ft x flutes
                    mul_a  <= signed(resize(c_Ft, 24));
                    mul_b  <= resize(c_p16, 14);
                    mul_n  <= to_unsigned(8, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(10, 6);

                when 10 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_Fc <= unsigned(m_acc(27 downto 4));
                        q_st <= to_unsigned(11, 6);
                    end if;

                when 11 =>      -- glass 1.5 Fc; lattice 0.75 Ft; start t256 x dy
                    c_Fg   <= c_Fc + shift_right(c_Fc, 1);
                    c_Fd   <= c_Ft - shift_right(c_Ft, 2);
                    mul_a  <= signed(resize(c_dy, 24));
                    mul_b  <= resize(c_t256, 14);
                    mul_n  <= to_unsigned(11, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(12, 6);

                when 12 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_P  <= unsigned(m_acc(23 downto 0));
                        q_st <= to_unsigned(13, 6);
                    end if;

                when 13 =>      -- field: 0.56 tiles cycles / frame; lattice 0.8
                    v_P9  := shift_left(resize(c_P, 28), 3) + resize(c_P, 28);
                    c_dB  <= resize(v_P9(27 downto 8), 24);
                    c_dPd <= shift_right(c_P, 5) + shift_right(c_P, 6) + shift_right(c_P, 9);
                    q_st  <= to_unsigned(14, 6);

                when 14 =>      -- wobble: 0.6 and 1.7 cycles / frame
                    c_dW1 <= shift_left(resize(c_dy, 24), 3) + shift_left(resize(c_dy, 24), 1)
                             - resize(c_dy(12 downto 1), 24) + resize(c_dy(12 downto 3), 24);
                    c_dW2 <= shift_left(resize(c_dy, 24), 5) - shift_left(resize(c_dy, 24), 2)
                             - resize(c_dy(12 downto 1), 24);
                    q_st  <= to_unsigned(15, 6);

                when 15 =>      -- Sheet duty 0.12 .. 0.88 (x 65536)
                    c_ka <= shift_left(resize(c_k3, 16), 5) + shift_left(resize(c_k3, 16), 4);
                    c_kb <= resize(c_k3(9 downto 1), 10) + resize(c_k3(9 downto 3), 10);
                    q_st <= to_unsigned(16, 6);

                when 16 =>
                    c_duty <= to_unsigned(7864, 16) + c_ka + resize(c_kb, 16);
                    q_st   <= to_unsigned(17, 6);

                when 17 =>      -- sheet centred on the tile centre; melt; Contours^2
                    c_koff <= '1' & c_duty(15 downto 1);
                    c_melt <= c_k4(9 downto 2);
                    mul_a  <= signed(resize(c_p12, 24));
                    mul_b  <= resize(c_p12, 14);
                    mul_n  <= to_unsigned(10, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(18, 6);

                when 18 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_c2 <= unsigned(m_acc(19 downto 0));
                        q_st <= to_unsigned(19, 6);
                    end if;

                when 19 =>      -- 184 c2 = 128 c2 + 32 c2 + 16 c2 + 8 c2
                    c_ga <= shift_left(resize(c_c2, 28), 7) + shift_left(resize(c_c2, 28), 5);
                    c_gb <= shift_left(resize(c_c2, 28), 4) + shift_left(resize(c_c2, 28), 3);
                    q_st <= to_unsigned(20, 6);

                when 20 =>      -- contour gain 8 .. 191 (0.5 .. 12 cycles over the field)
                    v_g  := c_ga + c_gb;
                    c_G  <= to_unsigned(8, 8) + resize(v_g(27 downto 20), 8);
                    q_st <= to_unsigned(21, 6);

                when 21 =>      -- Speed: bipolar with a +-24 still zone
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
                    q_st <= to_unsigned(22, 6);

                when 22 =>
                    mul_a  <= signed(resize(c_mag, 24));
                    mul_b  <= resize(c_mag, 14);
                    mul_n  <= to_unsigned(9, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(23, 6);

                when 23 =>      -- speed = 2 mag^2 (up to ~1.7 turns/s)
                    if mul_go = '0' and mul_busy = '0' then
                        c_spd <= unsigned(m_acc(17 downto 0)) & '0';
                        q_st  <= to_unsigned(24, 6);
                    end if;

                when 24 =>      -- time: field rolls (down for positive speed),
                                -- sheets right, flutes left, bows travel
                    if c_dir = '0' then
                        c_tb  <= c_tb  - resize(c_spd, 24);
                        c_tk  <= c_tk  - resize(c_spd(18 downto 1), 24);
                        c_tc  <= c_tc  + resize(c_spd(18 downto 1), 24) + resize(c_spd(18 downto 2), 24);
                        c_tw1 <= c_tw1 - resize(c_spd(18 downto 1), 24);
                        c_tw2 <= c_tw2 + resize(c_spd, 24);
                    else
                        c_tb  <= c_tb  + resize(c_spd, 24);
                        c_tk  <= c_tk  + resize(c_spd(18 downto 1), 24);
                        c_tc  <= c_tc  - resize(c_spd(18 downto 1), 24) - resize(c_spd(18 downto 2), 24);
                        c_tw1 <= c_tw1 + resize(c_spd(18 downto 1), 24);
                        c_tw2 <= c_tw2 - resize(c_spd, 24);
                    end if;
                    if c_flow = '1' then          -- ~0.1 .. 1.8 palette cycles / s
                        c_tp <= c_tp + 7 + resize(c_spd(18 downto 12), 12);
                    end if;
                    q_st <= to_unsigned(25, 6);

                when 25 =>      -- tiles centred: pt(W/2) = half a turn
                    c_pt0 <= x"800000" - shift_left(resize(c_t256, 24), 15);
                    c_tg0 <= c_tc + shift_right(c_tc, 1);
                    c_off <= to_unsigned(2580, 12) + c_tp;    -- saddles on magenta
                    q_st  <= to_unsigned(26, 6);

                when 26 =>      -- field start; prime the line engine for line 0
                    if s_ilace = '1' and s_timing.field_n = '0' then
                        -- bottom field: half a field line down
                        l_q  <= resize(c_dy(12 downto 1), 21);
                        l_B  <= c_tb + shift_right(c_dB, 1);
                        l_W1 <= c_tw1 + x"300000" + shift_right(c_dW1, 1);
                        l_W2 <= c_tw2 + shift_right(c_dW2, 1);
                        l_Pd <= c_tk + shift_right(c_dPd, 1);
                    else
                        l_q  <= (others => '0');
                        l_B  <= c_tb;
                        l_W1 <= c_tw1 + x"300000";
                        l_W2 <= c_tw2;
                        l_Pd <= c_tk;
                    end if;
                    q_prime <= '1';
                    q_st    <= to_unsigned(32, 6);

                ------------------------------------------------------------
                -- per line (for the NEXT line)
                ------------------------------------------------------------
                when 32 =>
                    if q_prime = '0' then
                        l_q  <= l_q + c_dy;
                        l_B  <= l_B + c_dB;
                        l_W1 <= l_W1 + c_dW1;
                        l_W2 <= l_W2 + c_dW2;
                        l_Pd <= l_Pd + c_dPd;
                    end if;
                    q_prime <= '0';
                    q_st    <= to_unsigned(33, 6);

                when 33 =>      -- wobble sines from the line ROM
                    ls_addr <= l_W1(23 downto 8);
                    q_st    <= to_unsigned(34, 6);

                when 34 =>
                    ls_addr <= l_W2(23 downto 8);
                    q_st    <= to_unsigned(35, 6);

                when 35 =>
                    q_st <= to_unsigned(36, 6);

                when 36 =>      -- sin(W1) (one's complement sign)
                    v_s := signed(ls_r);
                    if ls_s2 = '1' then
                        l_s1 <= not v_s;
                    else
                        l_s1 <= v_s;
                    end if;
                    q_st <= to_unsigned(37, 6);

                when 37 =>      -- sin(W2)
                    v_s := signed(ls_r);
                    if ls_s2 = '1' then
                        l_s2 <= not v_s;
                    else
                        l_s2 <= v_s;
                    end if;
                    q_st <= to_unsigned(38, 6);

                when 38 =>      -- 5 s1
                    l_t5 <= shift_left(resize(l_s1, 15), 2) + resize(l_s1, 15);
                    q_st <= to_unsigned(39, 6);

                when 39 =>      -- wsum = 0.31 s1 + 0.06 s2  (+-768)
                    l_wsum <= resize(shift_right(l_t5, 4), 11) + resize(shift_right(l_s2, 4), 11);
                    q_st   <= to_unsigned(40, 6);

                when 40 =>      -- wobble = wsum x melt
                    mul_a  <= resize(l_wsum, 24);
                    mul_b  <= resize(c_melt, 14);
                    mul_n  <= to_unsigned(8, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(41, 6);

                when 41 =>
                    if mul_go = '0' and mul_busy = '0' then
                        l_wob <= m_acc(18 downto 8);
                        q_st  <= to_unsigned(42, 6);
                    end if;

                when 42 =>      -- field shift wob/8 turn; key shift 1/2 tile
                    l_A0 <= unsigned(shift_left(resize(l_wob, 16), 3));
                    l_D1 <= unsigned(shift_left(resize(l_wob, 16), 5)) + c_koff;
                    q_st <= to_unsigned(43, 6);

                when 43 =>
                    l_D  <= l_D1 + c_tk(23 downto 8);
                    q_st <= to_unsigned(44, 6);

                when 44 =>      -- Tear: index jitter +-4 entries, 0..3 px hold
                    if c_tear = '1' then
                        v_j    := signed(l_lfsr(3 downto 0));
                        l_offs <= c_off + unsigned(resize(shift_left(resize(v_j, 8), 3), 12));
                        l_offp <= unsigned(resize(shift_left(resize(v_j, 8), 2), 12));
                        if c_hd = '1' then
                            l_hold <= unsigned(l_lfsr(5 downto 4));
                        else
                            l_hold <= '0' & l_lfsr(4);
                        end if;
                    else
                        l_offs <= c_off;
                        l_offp <= (others => '0');
                        l_hold <= (others => '0');
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
    -- Pixel path.  S0 = the clock where the accumulators hold the pixel's
    -- phases and g_avid the pixel's avid; S26 = the output register.
    -- ====================================================================
    p_pix : process(clk)
        variable v_kv  : std_logic;
        variable v_v1, v_v2 : signed(13 downto 0);
        variable v_sg  : signed(22 downto 0);
        variable v_n, v_n1 : unsigned(7 downto 0);
        variable v_cy, v_cu, v_cv : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- S0: the line's phases (reloaded at g_go); Tear holds them for
            -- 0..3 pixels
            if g_go = '1' then
                px_pt   <= c_pt0;
                px_pc   <= c_tc;
                px_pg   <= c_tg0;
                px_pd   <= l_Pd;
                px_hold <= l_hold;
                pl_A0   <= l_A0;
                pl_D    <= l_D;
                pl_B    <= l_B(23 downto 8);
                pl_cq   <= l_q(20 downto 5);
                pl_offs <= l_offs;
                pl_offp <= l_offp;
            else
                if px_hold /= 0 then
                    px_hold <= px_hold - 1;
                else
                    px_pt <= px_pt + resize(c_Ft, 24);
                    px_pc <= px_pc + c_Fc;
                    px_pg <= px_pg + c_Fg;
                    px_pd <= px_pd - resize(c_Fd, 24);
                end if;
            end if;
            px_ls <= g_go;

            -- S1: tile coordinate (Mirror: a triangle, half a turn per half
            -- tile; Repeat: a full turn per tile); key phase; flute position
            if c_mirror = '1' then
                if px_pt(23) = '1' then
                    p1_A <= '0' & not px_pt(22 downto 8);
                else
                    p1_A <= '0' & px_pt(22 downto 8);
                end if;
            else
                p1_A <= px_pt(23 downto 8);
            end if;
            p1_fk   <= px_pt(23 downto 8) + pl_D;
            p1_s    <= px_pc(23 downto 12);
            p1_g    <= px_pg(23 downto 8);
            p1_band <= px_pd(23);
            p1_ls   <= px_ls;

            -- S2: sheet test; field A with the line's wobble
            if p1_fk < c_duty then
                p2_kv <= '1';
            else
                p2_kv <= '0';
            end if;
            p2_A    <= p1_A + pl_A0;
            p2_g    <= p1_g;
            p2_band <= p1_band and c_lat;
            p2_ls   <= p1_ls;
            sp(2)   <= p1_s;

            -- S3: key; Glass samples the field at the flute phase in flutes
            v_kv   := p2_kv xor p2_band;
            p3_key <= v_kv;
            if c_glass = '1' and v_kv = '0' then
                p3_As <= p2_g;
            else
                p3_As <= p2_A;
            end if;
            p3_ls <= p2_ls;

            -- S4: melt sine address 2A + q/2; outline count since the key rose
            p4_ma <= (p3_As(14 downto 0) & '0') + pl_cq;
            p4_As <= p3_As;
            if p3_ls = '1' then
                p4_kprev <= p3_key;
                p4_cnt   <= (others => '1');
            else
                p4_kprev <= p3_key;
                if p3_key = '1' and p4_kprev = '0' then
                    p4_cnt <= (others => '0');
                elsif p4_cnt /= 15 then
                    p4_cnt <= p4_cnt + 1;
                end if;
            end if;

            -- S5: melt sine ROM (block RAM)
            p5_mq  <= C_SINQ(to_integer(f_qidx(p4_ma)));
            p5_msg <= p4_ma(15);
            p5_As  <= p4_As;

            -- S6: into fabric
            p6_mq  <= unsigned(p5_mq(10 downto 0));
            p6_msg <= p5_msg;
            p6_As  <= p5_As;

            -- S7: |sin| x melt, operand split hi/lo
            p7_lo  <= p6_mq * c_melt(3 downto 0);
            p7_hi  <= p6_mq * c_melt(7 downto 4);
            p7_msg <= p6_msg;
            p7_As  <= p6_As;

            -- S8: displacement up to a quarter turn
            p8_disp <= resize(shift_right(shift_left(resize(p7_hi, 19), 4) + resize(p7_lo, 19), 5), 14);
            p8_msg  <= p7_msg;
            p8_As   <= p7_As;

            -- S9: B2 = B +- displacement
            if p8_msg = '1' then
                p9_B2 <= pl_B - resize(p8_disp, 16);
            else
                p9_B2 <= pl_B + resize(p8_disp, 16);
            end if;
            p9_As <= p8_As;

            -- S10: the egg-crate's two phases
            p10_a1 <= p9_As + p9_B2;
            p10_a2 <= p9_As - p9_B2;

            -- S11: field sine ROMs (block RAM)
            p11_q1 <= C_SINQ(to_integer(f_qidx(p10_a1)));
            p11_q2 <= C_SINQ(to_integer(f_qidx(p10_a2)));
            p11_g1 <= p10_a1(15);
            p11_g2 <= p10_a2(15);

            -- S12: into fabric
            p12_q1 <= unsigned(p11_q1(10 downto 0));
            p12_q2 <= unsigned(p11_q2(10 downto 0));
            p12_g1 <= p11_g1;
            p12_g2 <= p11_g2;

            -- S13: S = sin(A + B2) + sin(A - B2), +-4094 (one's complement signs)
            v_v1 := signed(resize(p12_q1, 14));
            v_v2 := signed(resize(p12_q2, 14));
            if p12_g1 = '1' then v_v1 := not v_v1; end if;
            if p12_g2 = '1' then v_v2 := not v_v2; end if;
            p13_S <= v_v1 + v_v2;

            -- S14: S x Contours, operand split hi/lo; the flutes' share of S
            p14_lo <= p13_S * signed('0' & c_G(3 downto 0));
            p14_hi <= p13_S * signed('0' & c_G(7 downto 4));
            if c_glass = '1' then
                p14_sp <= shift_right(p13_S, 1) + shift_right(p13_S, 3);
            else
                p14_sp <= shift_right(p13_S, 2);
            end if;

            -- S15: sheet product; flute index = position + field + jitter
            p15_sg <= shift_left(resize(p14_hi, 23), 4) + resize(p14_lo, 23);
            p15_ip <= sp(14) + unsigned(p14_sp(11 downto 0)) + pl_offp;

            -- S16: sheet index = S.G/32 + offset (+ flow + jitter); outline
            v_sg   := p15_sg;
            p16_is <= unsigned(v_sg(16 downto 5)) + pl_offs;
            p16_ip <= p15_ip;
            p16_oi <= f_oidx(unsigned(kc(15)(3 downto 0)), c_hd);
            if c_hd = '1' then
                if unsigned(kc(15)(3 downto 0)) < 6 then p16_out <= '1'; else p16_out <= '0'; end if;
            else
                if unsigned(kc(15)(3 downto 0)) < 3 then p16_out <= '1'; else p16_out <= '0'; end if;
            end if;

            -- S17: outline / sheet / flute
            if p16_out = '1' and kc(16)(4) = '1' then
                p17_idx <= p16_oi;
                p17_grp <= '0';
            elsif kc(16)(4) = '1' then
                p17_idx <= p16_is;
                p17_grp <= '0';
            else
                p17_idx <= p16_ip;
                p17_grp <= '1';
            end if;

            -- {key, count}: S5 holds S4's results
            kc(5) <= p4_kprev & std_logic_vector(p4_cnt);
            for i in 6 to 16 loop
                kc(i) <= kc(i - 1);
            end loop;
            for i in 3 to 15 loop
                sp(i) <= sp(i - 1);
            end loop;

            -- S18: even/odd bank addresses for entries n, n+1
            v_n    := p17_idx(11 downto 4);
            v_n1   := v_n + 1;
            p18_ea <= c_pal & p17_grp & v_n1(7 downto 1);
            p18_oa <= c_pal & p17_grp & v_n(7 downto 1);
            p18_sw <= v_n(0);
            p18_fr <= p17_idx(3 downto 0);

            -- S19: palette ROMs (block RAM)
            p19_ey <= C_EY(to_integer(p18_ea));
            p19_eu <= C_EU(to_integer(p18_ea));
            p19_ev <= C_EV(to_integer(p18_ea));
            p19_oy <= C_OY(to_integer(p18_oa));
            p19_ou <= C_OU(to_integer(p18_oa));
            p19_ov <= C_OV(to_integer(p18_oa));
            p19_sw <= p18_sw;
            p19_fr <= p18_fr;

            -- S20: into fabric before any arithmetic
            p20_ey <= unsigned(p19_ey);
            p20_eu <= unsigned(p19_eu);
            p20_ev <= unsigned(p19_ev);
            p20_oy <= unsigned(p19_oy);
            p20_ou <= unsigned(p19_ou);
            p20_ov <= unsigned(p19_ov);
            p20_sw <= p19_sw;
            p20_fr <= p19_fr;

            -- S21: a = entry n, b = entry n+1; d = b - a
            if p20_sw = '0' then
                p21_ay <= p20_ey;
                p21_au <= p20_eu;
                p21_av <= p20_ev;
                p21_dy <= signed('0' & p20_oy) - signed('0' & p20_ey);
                p21_du <= signed('0' & p20_ou) - signed('0' & p20_eu);
                p21_dv <= signed('0' & p20_ov) - signed('0' & p20_ev);
            else
                p21_ay <= p20_oy;
                p21_au <= p20_ou;
                p21_av <= p20_ov;
                p21_dy <= signed('0' & p20_ey) - signed('0' & p20_oy);
                p21_du <= signed('0' & p20_eu) - signed('0' & p20_ou);
                p21_dv <= signed('0' & p20_ev) - signed('0' & p20_ov);
            end if;
            p21_fr <= p20_fr;

            -- S22: d x fraction
            p22_ay <= p21_ay;
            p22_au <= p21_au;
            p22_av <= p21_av;
            p22_my <= p21_dy * signed('0' & p21_fr);
            p22_mu <= p21_du * signed('0' & p21_fr);
            p22_mv <= p21_dv * signed('0' & p21_fr);

            -- S23: 10-bit colour = (16 a + d f) / 4
            v_cy := signed(resize(p22_ay, 14) sll 4) + p22_my;
            v_cu := signed(resize(p22_au, 14) sll 4) + p22_mu;
            v_cv := signed(resize(p22_av, 14) sll 4) + p22_mv;
            p23_y <= unsigned(v_cy(11 downto 2));
            p23_u <= unsigned(v_cu(11 downto 2));
            p23_v <= unsigned(v_cv(11 downto 2));
        end if;
    end process p_pix;

    -- ====================================================================
    -- S24..S26: output conditioning and the blanking gate
    -- ====================================================================
    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S24/S25: horizontal [1 2 1]/4 (g1 is the centre pixel)
            g1_y <= p23_y;  g2_y <= g1_y;
            g1_u <= p23_u;  g2_u <= g1_u;
            g1_v <= p23_v;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(p23_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(p23_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(p23_v, 12);
            p25_y <= v_sy(11 downto 2);
            p25_u <= v_su(11 downto 2);
            p25_v <= v_sv(11 downto 2);

            -- S26: luma 64 + 13/16 (y - 64) (white ~775); neutral outside
            -- the generated avid
            v_d  := p25_y - 64;
            v_yo := p25_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if sd(25)(3) = '0' then
                s_out_y <= C_BLK;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo;
                s_out_u <= p25_u;
                s_out_v <= p25_v;
            end if;

            sd(1) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            for k in 2 to 26 loop
                sd(k) <= sd(k - 1);
            end loop;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= sd(26)(3);
    data_out.hsync_n <= sd(26)(2);
    data_out.vsync_n <= sd(26)(1);
    data_out.field_n <= sd(26)(0);

end architecture colonnade;
