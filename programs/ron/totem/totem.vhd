-- totem.vhd  (v0.1.0) -- rippling rainbow curtains behind a stepped totem
--
-- TOTEM.  An analog colourizer patch rebuilt in one pass, after a reference
-- capture: lit rainbow curtain bands ripple across the screen (more toward
-- the bottom), a translucent diagonal sheet tints them, and a tapering
-- column of alternating slats stands in the middle -- the same band
-- oscillator seen through a perspective lens.  Synthesis: no input video
-- is read.
--
-- PICTURE
--  1. CURTAINS.  A line-synced band oscillator, phase = tb + x.F, is
--     displaced by a two-sine travelling wave,
--        D = A(y) . (sin(w1) + sin(w2)),  w1 = y.ky + x.kx,  w2 = y.ky2 - x.kx2,
--     whose amplitude A(y) grows down the frame (q^2, opening toward q as
--     Ripple passes half way).  The band index picks a palette ROW, the
--     fraction the ramp along it: each band is a lit tube, dark at its
--     left edge and peaking 71 % across.  The beating of the two waves
--     gives irregular S-bends; at full Ripple the bottom folds over.
--  2. SHEET.  Diagonal strips (x.sx + y.sy) swap alternate strips to the
--     hue-turned SHEET rows.
--  3. TOTEM.  Inside |x - c| < hw_eff the phase is the LENS coordinate
--     tI - |x - c| . k(y), k = F . hw_top / hw(y): the ribbons keep their
--     place across the column, so they converge with the taper.  The
--     column is cut into slats; even slats read the LENS rows, odd slats
--     the flat FLAME rows; Stepped edges hold each slat's width (odd slats
--     inset by 3/16).  The wave reaches into the lens at 1/2 (even) and
--     1/8 (odd) strength.
--  4. PALETTE.  entry = palette & group & row & knot (8 knots per row);
--     even/odd bank ROMs give knots k and k+1 on one clock, interpolated by
--     the ramp position r.7/256 -- never across a row, so band edges stay
--     hard.
--
-- ENGINE
--   One serial shift-add multiplier and one serial restoring divider.
--   Per field (first vsync edge after active video): 2^24/W, 2^20/H, the
--   control curves, the band frequency, column geometry, ripple
--   amplitude, wave and sheet steps, speed and the time phases.  Per
--   active line (kicked at the generated avid's start, for the NEXT line,
--   ~110 clocks): the line DDAs, the amplitude profile, the slat latch, the
--   lens step k = Kin / hw (divider) and its start phase -c.k, the column
--   edges.
--
-- CONTROLS
--   K1 Bands    3 .. 40 curtain bands across (square law)
--   K2 Width    totem width at the top, 8 .. 92 % of the screen
--   K3 Taper    bipolar: vase (flares down) / pillar (centre) / funnel
--   K4 Slats    2 .. 24 slats down the frame (square law)
--   K5 Speed    bipolar, still in the centre: waves travel, bands drift,
--               the totem's ribbons flow, the sheet slides
--   K6 Palette  Signal / Ember / Neon / Dusk
--   S7 Sheet    diagonal tint strips Off / On
--   S8 Mirror   totem ribbons mirrored about the centre Off / On
--   S9 Edges    Stepped slats (odd slats inset) / Smooth taper
--   S10 Swell   Long / Short waves
--   S11 Tear    per-line 0..3 px start jitter Off / On
--   P12 Ripple  wave amplitude 1 .. 37 % of the width at the bottom; past
--               half way the ripple climbs up the frame.  100 % = the lower
--               half folds into swirls and the totem buckles
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, fixed width:
-- the encoders pair Cb/Cr from hsync); one latency for all modes (18
-- stages); outputs gated to neutral outside the generated avid; palette
-- stored U/V-swapped (totem.py); per-field work on the first vsync EDGE
-- after active video; line engine on active lines only; bottom field
-- (field_n = '0') starts half a field line down; HD HDMI bar conditioning
-- ([1 2 1] on Y/U/V, luma to ~775) before the gate.
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
use work.totem_pkg.all;

architecture totem of program_top is

    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- quarter-wave sine table index of a 16-bit phase
    function f_qidx(p : unsigned(15 downto 0)) return unsigned is
    begin
        if p(14) = '1' then
            return not p(13 downto 6);
        else
            return p(13 downto 6);
        end if;
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
    -- knob glides: 0 K1 Bands, 1 K2 Width, 2 K3 Taper, 3 K4 Slats,
    -- 4 P12 Ripple.  Deadband hold + per-field ease, 10.4
    ------------------------------------------------------------------------
    signal gl_raw : unsigned(49 downto 0) := (others => '0');
    signal gl_o   : unsigned(69 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- sequencer
    ------------------------------------------------------------------------
    signal q_st : unsigned(6 downto 0) := (others => '0');
    attribute fsm_encoding : string;
    attribute fsm_encoding of q_st : signal is "none";
    signal q_fpend, q_lpend, q_prime : std_logic := '0';

    -- multiplier operation codes (operands a x b, iterations n)
    constant OP_K1SQ : integer := 0;    -- k1 x k1
    constant OP_X37  : integer := 1;    -- m x 37
    constant OP_F    : integer := 2;    -- 2^24/W x bands.256
    constant OP_K2   : integer := 3;    -- k2 x 430
    constant OP_HWT  : integer := 4;    -- W x width
    constant OP_TP   : integer := 5;    -- hw_top x taper
    constant OP_X75  : integer := 6;    -- m x 7 (funnel) / 5 (vase)
    constant OP_DHW  : integer := 7;    -- (bottom - top) x 2^20/H
    constant OP_K4SQ : integer := 8;    -- k4 x k4
    constant OP_X22  : integer := 9;    -- m x 22
    constant OP_SST  : integer := 10;   -- 2^20/H x slats.256
    constant OP_KIN  : integer := 11;   -- F x hw_top
    constant OP_PSQ  : integer := 12;   -- p x p
    constant OP_X220 : integer := 13;   -- m x 220
    constant OP_P150 : integer := 14;   -- p x 150
    constant OP_APX  : integer := 15;   -- W x amplitude
    constant OP_APH  : integer := 16;   -- F x amplitude px (m)
    constant OP_KX   : integer := 17;   -- 2^24/W x 400 / 180
    constant OP_KY   : integer := 18;   -- 2^20/H x 1536 / 640
    constant OP_SX   : integer := 19;   -- 2^24/W x 187
    constant OP_SY   : integer := 20;   -- 2^20/H x 211
    constant OP_SPD  : integer := 21;   -- mag x mag
    constant OP_QQ   : integer := 22;   -- q x q
    constant OP_PROF : integer := 23;   -- pe x (q - q^2)
    constant OP_A    : integer := 24;   -- Aph x profile
    constant OP_U0   : integer := 25;   -- k x W/2

    -- serial multiplier: signed a x unsigned b.  The sequencer writes only
    -- an operation code; the multiplier muxes its own operands from it one
    -- clock later (so no wide operand mux hangs off the state decode)
    signal mul_go, mul_busy : std_logic := '0';
    signal mul_op : unsigned(4 downto 0) := (others => '0');
    signal m_acc, m_a : signed(39 downto 0) := (others => '0');
    signal m_b    : unsigned(13 downto 0) := (others => '0');
    signal m_cnt  : unsigned(3 downto 0) := (others => '0');
    signal m_load : std_logic := '0';
    signal o_a    : signed(24 downto 0) := (others => '0');   -- fetched operands
    signal o_b    : unsigned(13 downto 0) := (others => '0');
    signal o_n    : unsigned(3 downto 0) := (others => '0');

    -- serial divider: q = num / den (restoring, 32 quotient bits), by code
    signal div_go, div_busy : std_logic := '0';
    signal div_op  : unsigned(1 downto 0) := (others => '0');
    signal d_num   : unsigned(31 downto 0) := (others => '0');
    signal d_den   : unsigned(11 downto 0) := (others => '0');
    signal d_rem   : unsigned(11 downto 0) := (others => '0');
    signal d_q     : unsigned(31 downto 0) := (others => '0');
    signal d_cnt   : unsigned(5 downto 0) := (others => '0');

    -- per-field values
    signal c_k1, c_k2, c_k3, c_k4, c_p12, c_k5 : unsigned(9 downto 0) := (others => '0');
    signal c_pal  : unsigned(1 downto 0) := (others => '0');
    signal c_sheet, c_mirror, c_smooth, c_short, c_tear : std_logic := '0';
    signal c_jm   : std_logic := '1';                         -- jitter 0..3 (else 0..1)
    signal c_W    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal c_H    : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal c_c    : unsigned(10 downto 0) := to_unsigned(960, 11);   -- W / 2
    signal c_recw : unsigned(15 downto 0) := to_unsigned(8738, 16);  -- 2^24 / W
    signal c_rech : unsigned(12 downto 0) := to_unsigned(970, 13);   -- 2^20 / field lines
    signal c_bq8, c_m : unsigned(13 downto 0) := (others => '0');   -- multiplier operands
    signal c_dh   : signed(12 downto 0) := (others => '0');
    signal c_F    : unsigned(21 downto 0) := (others => '0');        -- band phase / px (2^24 / band)
    signal c_hwt  : unsigned(10 downto 0) := (others => '0');        -- half-width at the top
    signal c_hwb  : unsigned(11 downto 0) := (others => '0');        -- half-width at the bottom
    signal c_tdir : std_logic := '0';                                -- 1 = funnel
    signal c_tp   : unsigned(8 downto 0) := (others => '0');
    signal c_dhw  : signed(31 downto 0) := (others => '0');          -- hw step / line (2^-20 px)
    signal c_sst  : unsigned(17 downto 0) := (others => '0');        -- slat step / line (2^-20)
    signal c_Kin  : unsigned(29 downto 0) := (others => '0');        -- F . hw_top
    signal c_t2   : unsigned(7 downto 0) := (others => '0');
    signal c_pa   : unsigned(8 downto 0) := (others => '0');
    signal c_Aph  : unsigned(17 downto 0) := (others => '0');        -- ripple amplitude (2^14 / band)
    signal c_pe   : unsigned(9 downto 0) := (others => '0');         -- profile opening
    signal c_kx, c_kx2, c_sx, c_sy : unsigned(7 downto 0) := (others => '0');
    signal c_ky   : unsigned(10 downto 0) := (others => '0');
    signal c_ky2  : unsigned(11 downto 0) := (others => '0');
    signal c_dir  : std_logic := '0';
    signal c_mag  : unsigned(8 downto 0) := (others => '0');
    signal c_spd  : unsigned(17 downto 0) := (others => '0');
    signal c_tw1  : unsigned(15 downto 0) := x"4CCD";
    signal c_tw2  : unsigned(15 downto 0) := x"B333";
    signal c_tb   : unsigned(27 downto 0) := (others => '0');
    signal c_ti   : unsigned(27 downto 0) := x"0800000";
    signal c_ts   : unsigned(15 downto 0) := x"C000";

    -- per-line engine.  The DDAs hold the CURRENT line's values until the
    -- engine (kicked at this line's start) advances them; the results
    -- (l_A .. l_j) are written during this line for the next one.  The
    -- pixel path copies both at g_go.
    signal l_ry   : unsigned(20 downto 0) := (others => '0');
    signal l_sp   : unsigned(24 downto 0) := (others => '0');
    signal l_hw   : signed(31 downto 0) := (others => '0');          -- 11.20 half-width
    signal l_w1, l_w2, l_sh : unsigned(15 downto 0) := (others => '0');
    signal l_first : std_logic := '0';
    signal l_sidx : unsigned(4 downto 0) := (others => '0');
    signal l_par  : std_logic := '0';
    signal l_hws  : unsigned(10 downto 0) := (others => '0');
    signal l_hwc  : unsigned(10 downto 0) := to_unsigned(4, 11);
    signal l_q, l_q2, l_d : unsigned(9 downto 0) := (others => '0');
    signal l_prof : unsigned(10 downto 0) := (others => '0');
    signal l_A    : unsigned(12 downto 0) := (others => '0');
    signal l_k    : unsigned(23 downto 0) := (others => '0');
    signal l_u0   : unsigned(29 downto 0) := (others => '0');
    signal l_hwe  : unsigned(10 downto 0) := (others => '0');
    signal l_L, l_R : unsigned(11 downto 0) := (others => '0');
    signal l_j    : unsigned(1 downto 0) := (others => '0');
    signal l_lfsr : std_logic_vector(15 downto 0) := x"ACE1";

    ------------------------------------------------------------------------
    -- pixel path.  The phase is carried as bits 27..14 (band 4 bits, ramp
    -- 8 bits, 2 guard bits) once the accumulators are left behind.
    ------------------------------------------------------------------------
    signal px_x   : unsigned(11 downto 0) := (others => '0');
    signal px_pbg : unsigned(27 downto 0) := (others => '0');
    signal px_u   : unsigned(29 downto 0) := (others => '0');
    signal px_w1, px_w2, px_sh : unsigned(15 downto 0) := (others => '0');
    signal px_A   : unsigned(12 downto 0) := (others => '0');
    signal px_k   : unsigned(23 downto 0) := (others => '0');
    signal px_L, px_R : unsigned(11 downto 0) := (others => '0');
    signal px_par : std_logic := '0';
    signal px_j   : unsigned(1 downto 0) := (others => '0');

    signal p1_a1, p1_a2 : unsigned(7 downto 0) := (others => '0');
    signal p1_s1, p1_s2, p1_inl, p1_inr, p1_sh : std_logic := '0';
    signal p1_nu, p1_pbg : unsigned(13 downto 0) := (others => '0');
    signal p2_m1, p2_m2 : std_logic_vector(7 downto 0) := (others => '0');
    signal p2_s1, p2_s2, p2_in, p2_sh : std_logic := '0';
    signal p2_pin, p2_pbg : unsigned(13 downto 0) := (others => '0');
    signal p3_m1, p3_m2 : unsigned(7 downto 0) := (others => '0');
    signal p3_s1, p3_s2, p3_in : std_logic := '0';
    signal p3_ph  : unsigned(13 downto 0) := (others => '0');
    signal p3_grp : unsigned(1 downto 0) := (others => '0');
    signal p4_sum : signed(9 downto 0) := (others => '0');
    signal p4_in  : std_logic := '0';
    signal p4_ph  : unsigned(13 downto 0) := (others => '0');
    signal p4_grp : unsigned(1 downto 0) := (others => '0');
    signal p5_mag : unsigned(8 downto 0) := (others => '0');
    signal p5_sgn : std_logic := '0';
    signal p5_Ae  : unsigned(12 downto 0) := (others => '0');
    signal p5_ph  : unsigned(13 downto 0) := (others => '0');
    signal p5_grp : unsigned(1 downto 0) := (others => '0');
    signal p6_lo  : unsigned(17 downto 0) := (others => '0');
    signal p6_hi  : unsigned(16 downto 0) := (others => '0');
    signal p6_sgn : std_logic := '0';
    signal p6_ph  : unsigned(13 downto 0) := (others => '0');
    signal p6_grp : unsigned(1 downto 0) := (others => '0');
    signal p7_prod : unsigned(21 downto 0) := (others => '0');
    signal p7_sgn : std_logic := '0';
    signal p7_ph  : unsigned(13 downto 0) := (others => '0');
    signal p7_grp : unsigned(1 downto 0) := (others => '0');
    signal p8_ph  : unsigned(13 downto 0) := (others => '0');
    signal p8_grp : unsigned(1 downto 0) := (others => '0');
    signal p9_band : unsigned(3 downto 0) := (others => '0');
    signal p9_t7  : unsigned(10 downto 0) := (others => '0');
    signal p9_grp : unsigned(1 downto 0) := (others => '0');
    signal p10_ea, p10_oa : unsigned(9 downto 0) := (others => '0');
    signal p10_sw : std_logic := '0';
    signal p10_fr : unsigned(3 downto 0) := (others => '0');
    signal p11_ey, p11_eu, p11_ev, p11_oy, p11_ou, p11_ov : std_logic_vector(7 downto 0) := (others => '0');
    signal p11_sw : std_logic := '0';
    signal p11_fr : unsigned(3 downto 0) := (others => '0');
    signal p12_ey, p12_eu, p12_ev, p12_oy, p12_ou, p12_ov : unsigned(7 downto 0) := (others => '0');
    signal p12_sw : std_logic := '0';
    signal p12_fr : unsigned(3 downto 0) := (others => '0');
    signal p13_ay, p13_au, p13_av : unsigned(7 downto 0) := (others => '0');
    signal p13_dy, p13_du, p13_dv : signed(8 downto 0) := (others => '0');
    signal p13_fr : unsigned(3 downto 0) := (others => '0');
    signal p14_ay, p14_au, p14_av : unsigned(7 downto 0) := (others => '0');
    signal p14_my, p14_mu, p14_mv : signed(13 downto 0) := (others => '0');
    signal p15_y, p15_u, p15_v : unsigned(9 downto 0) := (others => '0');
    -- output conditioning (the HD HDMI bar fix, damier v0.4.4): [1 2 1]
    -- horizontal smoothing on Y/U/V, luma scaled toward black
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal p17_y, p17_u, p17_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- syncs (generated avid, hsync, vsync, field) from S1 to S18
    type t_sd is array(1 to 18) of std_logic_vector(3 downto 0);
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

            if s_vs_go = '1' then
                g_wm1 <= s_meas_w - 1;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Knob glides: the hold follows the pot only beyond a +-4 deadband
    -- (snapping to the rails), the output eases toward it once per field
    -- (by 1/8; the Ripple slider by 1/4 so the gesture stays quick)
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
    -- Serial multiplier (shift-add, one bit of b per clock).  Operands are
    -- fetched by the operation code at go into o_ registers, and loaded
    -- one clock later (the fetch mux is a stage of its own).
    -- ====================================================================
    p_mul : process(clk)
        variable v_a : signed(24 downto 0);
        variable v_b : unsigned(13 downto 0);
        variable v_n : unsigned(3 downto 0);
        variable v_m : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            if mul_go = '1' then
                v_m := unsigned(m_acc(19 downto 0));
                v_a := (others => '0');
                v_b := (others => '0');
                v_n := to_unsigned(10, 4);
                case to_integer(mul_op) is
                    when OP_K1SQ =>
                        v_a := signed(resize(c_k1, 25)); v_b := resize(c_k1, 14);
                    when OP_X37 =>
                        v_a := signed(resize(v_m, 25)); v_b := to_unsigned(37, 14);
                        v_n := to_unsigned(6, 4);
                    when OP_F =>
                        v_a := signed(resize(c_recw, 25)); v_b := c_bq8;
                        v_n := to_unsigned(14, 4);
                    when OP_K2 =>
                        v_a := signed(resize(c_k2, 25)); v_b := to_unsigned(430, 14);
                        v_n := to_unsigned(9, 4);
                    when OP_HWT =>
                        v_a := signed(resize(c_W, 25)); v_b := c_m;
                    when OP_TP =>
                        v_a := signed(resize(c_hwt, 25)); v_b := resize(c_tp, 14);
                        v_n := to_unsigned(9, 4);
                    when OP_X75 =>
                        v_a := signed(resize(v_m, 25));
                        if c_tdir = '1' then
                            v_b := to_unsigned(7, 14);
                        else
                            v_b := to_unsigned(5, 14);
                        end if;
                        v_n := to_unsigned(3, 4);
                    when OP_DHW =>
                        v_a := resize(c_dh, 25); v_b := resize(c_rech, 14);
                        v_n := to_unsigned(13, 4);
                    when OP_K4SQ =>
                        v_a := signed(resize(c_k4, 25)); v_b := resize(c_k4, 14);
                    when OP_X22 =>
                        v_a := signed(resize(v_m, 25)); v_b := to_unsigned(22, 14);
                        v_n := to_unsigned(5, 4);
                    when OP_SST =>
                        v_a := signed(resize(c_rech, 25)); v_b := c_bq8;
                        v_n := to_unsigned(13, 4);
                    when OP_KIN =>
                        v_a := signed(resize(c_F, 25)); v_b := resize(c_hwt, 14);
                        v_n := to_unsigned(11, 4);
                    when OP_PSQ =>
                        v_a := signed(resize(c_p12, 25)); v_b := resize(c_p12, 14);
                    when OP_X220 =>
                        v_a := signed(resize(v_m, 25)); v_b := to_unsigned(220, 14);
                        v_n := to_unsigned(8, 4);
                    when OP_P150 =>
                        v_a := signed(resize(c_p12, 25)); v_b := to_unsigned(150, 14);
                        v_n := to_unsigned(8, 4);
                    when OP_APX =>
                        v_a := signed(resize(c_W, 25)); v_b := resize(c_pa, 14);
                        v_n := to_unsigned(9, 4);
                    when OP_APH =>
                        v_a := signed(resize(c_F, 25)); v_b := resize(unsigned(m_acc(20 downto 10)), 14);
                        v_n := to_unsigned(11, 4);
                    when OP_KX =>
                        v_a := signed(resize(c_recw, 25));
                        if c_short = '1' then
                            v_b := to_unsigned(400, 14);
                        else
                            v_b := to_unsigned(180, 14);
                        end if;
                        v_n := to_unsigned(9, 4);
                    when OP_KY =>
                        v_a := signed(resize(c_rech, 25));
                        if c_short = '1' then
                            v_b := to_unsigned(1536, 14);
                        else
                            v_b := to_unsigned(640, 14);
                        end if;
                        v_n := to_unsigned(11, 4);
                    when OP_SX =>
                        v_a := signed(resize(c_recw, 25)); v_b := to_unsigned(187, 14);
                        v_n := to_unsigned(8, 4);
                    when OP_SY =>
                        v_a := signed(resize(c_rech, 25)); v_b := to_unsigned(211, 14);
                        v_n := to_unsigned(8, 4);
                    when OP_SPD =>
                        v_a := signed(resize(c_mag, 25)); v_b := resize(c_mag, 14);
                        v_n := to_unsigned(9, 4);
                    when OP_QQ =>
                        v_a := signed(resize(l_q, 25)); v_b := resize(l_q, 14);
                    when OP_PROF =>
                        v_a := signed(resize(c_pe, 25)); v_b := resize(l_d, 14);
                    when OP_A =>
                        v_a := signed(resize(c_Aph, 25)); v_b := resize(l_prof, 14);
                        v_n := to_unsigned(11, 4);
                    when others =>          -- OP_U0
                        v_a := signed(resize(l_k, 25)); v_b := resize(c_c, 14);
                        v_n := to_unsigned(11, 4);
                end case;
                o_a      <= v_a;
                o_b      <= v_b;
                o_n      <= v_n;
                m_load   <= '1';
                mul_busy <= '1';
            elsif m_load = '1' then
                m_acc  <= (others => '0');
                m_a    <= resize(o_a, 40);
                m_b    <= o_b;
                m_cnt  <= o_n;
                m_load <= '0';
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
    -- Serial divider: q = floor(num / den), one quotient bit per clock
    -- (den = 0 gives all ones, which the users clamp).  0: 2^24 / W,
    -- 1: 2^20 / H, 2: Kin / hw
    -- ====================================================================
    p_div : process(clk)
        variable v_r : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            if div_go = '1' then
                case to_integer(div_op) is
                    when 0 =>
                        d_num <= to_unsigned(2 ** 24, 32);
                        d_den <= c_W;
                    when 1 =>
                        d_num <= to_unsigned(2 ** 20, 32);
                        d_den <= resize(c_H, 12);
                    when others =>
                        d_num <= resize(c_Kin, 32);
                        d_den <= resize(l_hwc, 12);
                end case;
                d_rem    <= (others => '0');
                d_q      <= (others => '0');
                d_cnt    <= to_unsigned(32, 6);
                div_busy <= '1';
            elsif div_busy = '1' then
                v_r := d_rem & d_num(31);
                if v_r >= resize(d_den, 13) then
                    v_r := v_r - resize(d_den, 13);
                    d_q <= d_q(30 downto 0) & '1';
                else
                    d_q <= d_q(30 downto 0) & '0';
                end if;
                d_rem <= v_r(11 downto 0);
                d_num <= d_num(30 downto 0) & '0';
                d_cnt <= d_cnt - 1;
                if d_cnt = 1 then
                    div_busy <= '0';
                end if;
            end if;
        end if;
    end process p_div;

    -- ====================================================================
    -- Sequencer: per-field set-up (states 1..29) and the per-line engine
    -- (64..71).  One small operation per state; a state may capture a
    -- result and issue the next operation (the units read their operands
    -- one clock after go, so a value written in the issuing state is
    -- seen).  A unit is free when its go and busy are both low.
    -- ====================================================================
    p_seq : process(clk)
        variable v_k6  : unsigned(9 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_ad  : unsigned(9 downto 0);
        variable v_hb  : signed(13 downto 0);
        variable v_q   : unsigned(9 downto 0);
        variable v_hc  : unsigned(10 downto 0);
        variable v_ins : unsigned(10 downto 0);
        variable v_s8  : unsigned(15 downto 0);
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
                        q_st    <= to_unsigned(1, 7);
                    elsif q_lpend = '1' then
                        q_lpend <= '0';
                        q_st    <= to_unsigned(64, 7);
                    end if;

                ------------------------------------------------------------
                -- per field
                ------------------------------------------------------------
                when 1 =>       -- latch the controls; 2^24 / W
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
                    c_sheet  <= registers_in(6)(0);
                    c_mirror <= registers_in(6)(1);
                    c_smooth <= registers_in(6)(2);
                    c_short  <= registers_in(6)(3);
                    c_tear   <= registers_in(6)(4);
                    c_W      <= s_meas_w;
                    c_H      <= s_meas_h;
                    c_c      <= s_meas_w(11 downto 1);
                    c_jm     <= s_meas_w(11) or s_meas_w(10);
                    div_op   <= to_unsigned(0, 2);
                    div_go   <= '1';
                    q_st     <= to_unsigned(2, 7);

                when 2 =>       -- W reciprocal; 2^20 / H
                    if div_go = '0' and div_busy = '0' then
                        if d_q(31 downto 16) /= 0 then
                            c_recw <= (others => '1');
                        else
                            c_recw <= d_q(15 downto 0);
                        end if;
                        div_op <= to_unsigned(1, 2);
                        div_go <= '1';
                        q_st   <= to_unsigned(3, 7);
                    end if;

                when 3 =>       -- line step; Bands squared
                    if div_go = '0' and div_busy = '0' then
                        if d_q(31 downto 13) /= 0 then
                            c_rech <= (others => '1');
                        else
                            c_rech <= d_q(12 downto 0);
                        end if;
                        mul_op <= to_unsigned(OP_K1SQ, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(4, 7);
                    end if;

                when 4 =>
                    if mul_go = '0' and mul_busy = '0' then
                        mul_op <= to_unsigned(OP_X37, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(5, 7);
                    end if;

                when 5 =>       -- bands x 256 = 768 + 37 k1^2 / 4096 (3 .. 40);
                                -- F = bands x 2^24 / W
                    if mul_go = '0' and mul_busy = '0' then
                        c_bq8  <= to_unsigned(768, 14) + resize(unsigned(m_acc(31 downto 12)), 14);
                        mul_op <= to_unsigned(OP_F, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(6, 7);
                    end if;

                when 6 =>       -- Width: k2 x 430
                    if mul_go = '0' and mul_busy = '0' then
                        c_F    <= unsigned(m_acc(29 downto 8));
                        mul_op <= to_unsigned(OP_K2, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(7, 7);
                    end if;

                when 7 =>       -- hw_top = W (41 + 430 k2 / 1024) / 1024 (4 .. 46 %)
                    if mul_go = '0' and mul_busy = '0' then
                        c_m    <= to_unsigned(41, 14) + resize(unsigned(m_acc(19 downto 10)), 14);
                        mul_op <= to_unsigned(OP_HWT, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(8, 7);
                    end if;

                when 8 =>       -- Taper: bipolar with a +-16 pillar zone
                    if mul_go = '0' and mul_busy = '0' then
                        c_hwt <= unsigned(m_acc(20 downto 10));
                        v_d := signed(resize(c_k3, 11)) - 512;
                        if v_d < 0 then
                            c_tdir <= '0';
                            v_ad   := unsigned(resize(-v_d, 10));
                        else
                            c_tdir <= '1';
                            v_ad   := unsigned(resize(v_d, 10));
                        end if;
                        if v_ad <= 16 then
                            c_tp <= (others => '0');
                        else
                            c_tp <= resize(v_ad - 16, 9);
                        end if;
                        mul_op <= to_unsigned(OP_TP, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(9, 7);
                    end if;

                when 9 =>       -- hw_top . tp x 7 (funnel) or x 5 (vase)
                    if mul_go = '0' and mul_busy = '0' then
                        mul_op <= to_unsigned(OP_X75, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(10, 7);
                    end if;

                when 10 =>      -- funnel: top - 0.85 top . tp / 496; vase: top + 1.2 top . tp / 496
                    if mul_go = '0' and mul_busy = '0' then
                        if c_tdir = '1' then
                            v_hb := signed(resize(c_hwt, 14)) - signed(resize(unsigned(m_acc(30 downto 12)), 14));
                        else
                            v_hb := signed(resize(c_hwt, 14)) + signed(resize(unsigned(m_acc(30 downto 11)), 14));
                        end if;
                        if v_hb < 6 then
                            c_hwb <= to_unsigned(6, 12);
                        elsif v_hb > signed(resize(c_c, 14)) - 2 then
                            c_hwb <= resize(c_c - 2, 12);
                        else
                            c_hwb <= unsigned(v_hb(11 downto 0));
                        end if;
                        q_st <= to_unsigned(11, 7);
                    end if;

                when 11 =>      -- hw step per line = (bottom - top) x 2^20 / H
                    c_dh   <= signed(resize(c_hwb, 13)) - signed(resize(c_hwt, 13));
                    mul_op <= to_unsigned(OP_DHW, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(12, 7);

                when 12 =>      -- Slats squared
                    if mul_go = '0' and mul_busy = '0' then
                        c_dhw  <= m_acc(31 downto 0);
                        mul_op <= to_unsigned(OP_K4SQ, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(13, 7);
                    end if;

                when 13 =>
                    if mul_go = '0' and mul_busy = '0' then
                        mul_op <= to_unsigned(OP_X22, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(14, 7);
                    end if;

                when 14 =>      -- slats x 256 = 512 + 22 k4^2 / 4096 (2 .. 24); x 2^20 / H
                    if mul_go = '0' and mul_busy = '0' then
                        c_bq8  <= to_unsigned(512, 14) + resize(unsigned(m_acc(31 downto 12)), 14);
                        mul_op <= to_unsigned(OP_SST, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(15, 7);
                    end if;

                when 15 =>      -- lens numerator Kin = F . hw_top
                    if mul_go = '0' and mul_busy = '0' then
                        c_sst  <= unsigned(m_acc(25 downto 8));
                        mul_op <= to_unsigned(OP_KIN, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(16, 7);
                    end if;

                when 16 =>      -- Ripple squared
                    if mul_go = '0' and mul_busy = '0' then
                        c_Kin  <= unsigned(m_acc(29 downto 0));
                        mul_op <= to_unsigned(OP_PSQ, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(17, 7);
                    end if;

                when 17 =>
                    if mul_go = '0' and mul_busy = '0' then
                        mul_op <= to_unsigned(OP_X220, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(18, 7);
                    end if;

                when 18 =>
                    if mul_go = '0' and mul_busy = '0' then
                        c_t2   <= unsigned(m_acc(27 downto 20));
                        mul_op <= to_unsigned(OP_P150, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(19, 7);
                    end if;

                when 19 =>      -- amplitude / W x 1024 = 12 + 150 p + 220 p^2
                    if mul_go = '0' and mul_busy = '0' then
                        c_pa   <= to_unsigned(12, 9) + resize(unsigned(m_acc(17 downto 10)), 9)
                                  + resize(c_t2, 9);
                        mul_op <= to_unsigned(OP_APX, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(20, 7);
                    end if;

                when 20 =>      -- amplitude in band phase = px x F
                    if mul_go = '0' and mul_busy = '0' then
                        mul_op <= to_unsigned(OP_APH, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(21, 7);
                    end if;

                when 21 =>      -- past half way the ripple climbs the frame
                    if mul_go = '0' and mul_busy = '0' then
                        if m_acc(39 downto 28) /= 0 then
                            c_Aph <= (others => '1');
                        else
                            c_Aph <= unsigned(m_acc(27 downto 10));
                        end if;
                        if c_p12(9) = '1' then
                            c_pe <= c_p12(8 downto 0) & '0';
                        else
                            c_pe <= (others => '0');
                        end if;
                        mul_op <= to_unsigned(OP_KX, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(22, 7);
                    end if;

                when 22 =>      -- wave 1 across; down (Long 2.5, Short 6)
                    if mul_go = '0' and mul_busy = '0' then
                        c_kx   <= unsigned(m_acc(23 downto 16));
                        mul_op <= to_unsigned(OP_KY, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(23, 7);
                    end if;

                when 23 =>      -- wave 2 = 5/4 across (the other way)
                    if mul_go = '0' and mul_busy = '0' then
                        c_ky   <= unsigned(m_acc(22 downto 12));
                        c_kx2  <= c_kx + shift_right(c_kx, 2);
                        mul_op <= to_unsigned(OP_SX, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(24, 7);
                    end if;

                when 24 =>      -- wave 2 = 13/8 down; sheet strips 0.36 across
                    if mul_go = '0' and mul_busy = '0' then
                        c_ky2  <= resize(c_ky, 12) + resize(c_ky(10 downto 1), 12)
                                  + resize(c_ky(10 downto 3), 12);
                        c_sx   <= unsigned(m_acc(23 downto 16));
                        mul_op <= to_unsigned(OP_SY, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(25, 7);
                    end if;

                when 25 =>      -- sheet 0.82 down; Speed: bipolar with a +-24 still zone
                    if mul_go = '0' and mul_busy = '0' then
                        c_sy <= unsigned(m_acc(19 downto 12));
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
                        mul_op <= to_unsigned(OP_SPD, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(26, 7);
                    end if;

                when 26 =>      -- speed = mag^2
                    if mul_go = '0' and mul_busy = '0' then
                        c_spd <= unsigned(m_acc(17 downto 0));
                        q_st  <= to_unsigned(27, 7);
                    end if;

                when 27 =>      -- waves travel (positive = down), opposite ways
                    v_s8 := resize(c_spd(17 downto 8), 16);
                    if c_dir = '0' then
                        c_tw1 <= c_tw1 - v_s8;
                        c_tw2 <= c_tw2 + v_s8 - resize(c_spd(17 downto 10), 16);
                    else
                        c_tw1 <= c_tw1 + v_s8;
                        c_tw2 <= c_tw2 - v_s8 + resize(c_spd(17 downto 10), 16);
                    end if;
                    q_st <= to_unsigned(28, 7);

                when 28 =>      -- bands drift, ribbons flow, the sheet slides
                    if c_dir = '0' then
                        c_tb <= c_tb - resize(c_spd(17 downto 2), 28);
                        c_ti <= c_ti + resize(c_spd(17 downto 1), 28);
                        c_ts <= c_ts - resize(c_spd(17 downto 12), 16);
                    else
                        c_tb <= c_tb + resize(c_spd(17 downto 2), 28);
                        c_ti <= c_ti - resize(c_spd(17 downto 1), 28);
                        c_ts <= c_ts + resize(c_spd(17 downto 12), 16);
                    end if;
                    q_st <= to_unsigned(29, 7);

                when 29 =>      -- field start; prime the line engine for line 0
                    l_sidx <= (others => '0');
                    if s_ilace = '1' and s_timing.field_n = '0' then
                        -- bottom field: half a field line down
                        l_ry <= resize(c_rech(12 downto 1), 21);
                        l_sp <= resize(c_sst(17 downto 1), 25);
                        l_hw <= shift_left(signed(resize(c_hwt, 32)), 20) + shift_right(c_dhw, 1);
                        l_w1 <= c_tw1 + resize(c_ky(10 downto 1), 16);
                        l_w2 <= c_tw2 + resize(c_ky2(11 downto 1), 16);
                        l_sh <= c_ts + resize(c_sy(7 downto 1), 16);
                    else
                        l_ry <= (others => '0');
                        l_sp <= (others => '0');
                        l_hw <= shift_left(signed(resize(c_hwt, 32)), 20);
                        l_w1 <= c_tw1;
                        l_w2 <= c_tw2;
                        l_sh <= c_ts;
                    end if;
                    q_prime <= '1';
                    q_st    <= to_unsigned(64, 7);

                ------------------------------------------------------------
                -- per line (for the NEXT line)
                ------------------------------------------------------------
                when 64 =>
                    if q_prime = '0' then
                        l_ry <= l_ry + c_rech;
                        l_sp <= l_sp + c_sst;
                        l_hw <= l_hw + c_dhw;
                        l_w1 <= l_w1 + c_ky;
                        l_w2 <= l_w2 + c_ky2;
                        l_sh <= l_sh + c_sy;
                    end if;
                    l_first <= q_prime;
                    q_prime <= '0';
                    q_st    <= to_unsigned(65, 7);

                when 65 =>      -- q = line / H; q^2; the continuous half-width
                    if l_ry(20) = '1' then
                        v_q := (others => '1');
                    else
                        v_q := l_ry(19 downto 10);
                    end if;
                    l_q <= v_q;
                    if l_hw < 0 then
                        v_hc := (others => '0');
                    else
                        v_hc := unsigned(l_hw(30 downto 20));
                    end if;
                    if v_hc < 4 then
                        l_hwc <= to_unsigned(4, 11);
                    else
                        l_hwc <= v_hc;
                    end if;
                    mul_op <= to_unsigned(OP_QQ, 5);
                    mul_go <= '1';
                    q_st   <= to_unsigned(66, 7);

                when 66 =>      -- slat latch (Stepped holds each slat's width);
                                -- profile opening x (q - q^2)
                    if mul_go = '0' and mul_busy = '0' then
                        l_q2 <= unsigned(m_acc(19 downto 10));
                        l_d  <= l_q - unsigned(m_acc(19 downto 10));
                        if l_first = '1' or l_sp(24 downto 20) /= l_sidx then
                            l_hws  <= l_hwc;
                            l_sidx <= l_sp(24 downto 20);
                            l_par  <= l_sp(20);
                        end if;
                        mul_op <= to_unsigned(OP_PROF, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(67, 7);
                    end if;

                when 67 =>      -- profile = q^2 + pe (q - q^2)
                    if mul_go = '0' and mul_busy = '0' then
                        l_prof <= resize(l_q2, 11) + resize(unsigned(m_acc(19 downto 10)), 11);
                        mul_op <= to_unsigned(OP_A, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(68, 7);
                    end if;

                when 68 =>      -- line amplitude (13 bits); the lens step Kin / hw
                    if mul_go = '0' and mul_busy = '0' then
                        if m_acc(39 downto 28) /= 0 then
                            l_A <= (others => '1');
                        else
                            l_A <= unsigned(m_acc(27 downto 15));
                        end if;
                        div_op <= to_unsigned(2, 2);
                        div_go <= '1';
                        q_st   <= to_unsigned(69, 7);
                    end if;

                when 69 =>      -- lens start phase -c . k
                    if div_go = '0' and div_busy = '0' then
                        if d_q(31 downto 24) /= 0 then
                            l_k <= (others => '1');
                        else
                            l_k <= d_q(23 downto 0);
                        end if;
                        mul_op <= to_unsigned(OP_U0, 5);
                        mul_go <= '1';
                        q_st   <= to_unsigned(70, 7);
                    end if;

                when 70 =>      -- effective half-width (Stepped: odd slats inset 3/16)
                    if mul_go = '0' and mul_busy = '0' then
                        l_u0 <= unsigned(-m_acc(29 downto 0));
                        if c_smooth = '1' then
                            l_hwe <= l_hwc;
                        elsif l_par = '1' then
                            v_ins := resize(l_hws(10 downto 3), 11) + resize(l_hws(10 downto 4), 11);
                            l_hwe <= l_hws - v_ins;
                        else
                            l_hwe <= l_hws;
                        end if;
                        q_st <= to_unsigned(71, 7);
                    end if;

                when 71 =>      -- column edges; jitter
                    l_L <= resize(c_c - l_hwe, 12);
                    l_R <= resize(c_c, 12) + resize(l_hwe, 12);
                    if c_tear = '0' then
                        l_j <= (others => '0');
                    elsif c_jm = '1' then
                        l_j <= unsigned(l_lfsr(1 downto 0));
                    else
                        l_j <= unsigned('0' & l_lfsr(0 downto 0));
                    end if;
                    if l_lfsr(0) = '1' then
                        l_lfsr <= ('0' & l_lfsr(15 downto 1)) xor x"B400";
                    else
                        l_lfsr <= '0' & l_lfsr(15 downto 1);
                    end if;
                    q_st <= to_unsigned(0, 7);

                when others =>
                    q_st <= to_unsigned(0, 7);
            end case;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Pixel path.  S0 = the clock where the px_ accumulators hold the
    -- pixel and g_avid its avid; S18 = the output register.
    -- ====================================================================
    p_pix : process(clk)
        variable v_sum : signed(9 downto 0);
        variable v_neg : signed(9 downto 0);
        variable v_t   : unsigned(10 downto 0);
        variable v_e   : unsigned(10 downto 0);
        variable v_k1  : unsigned(1 downto 0);
        variable v_cy, v_cu, v_cv : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- S0: the line's oscillators (hard sync at g_go); Tear holds
            -- them for 0..3 pixels
            if g_go = '1' then
                px_x   <= (others => '0');
                px_pbg <= c_tb;
                px_u   <= l_u0;
                px_w1  <= l_w1;
                px_w2  <= l_w2;
                px_sh  <= l_sh;
                px_A   <= l_A;
                px_k   <= l_k;
                px_L   <= l_L;
                px_R   <= l_R;
                px_par <= l_par;
                px_j   <= l_j;
            elsif px_j /= 0 then
                px_j <= px_j - 1;
            else
                px_x   <= px_x + 1;
                px_pbg <= px_pbg + resize(c_F, 28);
                px_u   <= px_u + resize(px_k, 30);
                px_w1  <= px_w1 + resize(c_kx, 16);
                px_w2  <= px_w2 - resize(c_kx2, 16);
                px_sh  <= px_sh + resize(c_sx, 16);
            end if;

            -- S1: sine table indexes; column edges; lens coordinate -|u|
            -- (one's complement: 1 LSB of 2^-24 band off, no adder)
            p1_a1  <= f_qidx(px_w1);
            p1_s1  <= px_w1(15);
            p1_a2  <= f_qidx(px_w2);
            p1_s2  <= px_w2(15);
            if px_x >= px_L then p1_inl <= '1'; else p1_inl <= '0'; end if;
            if px_x < px_R then p1_inr <= '1'; else p1_inr <= '0'; end if;
            if c_mirror = '1' and px_u(29) = '0' then
                p1_nu <= not px_u(27 downto 14);
            else
                p1_nu <= px_u(27 downto 14);
            end if;
            p1_pbg <= px_pbg(27 downto 14);
            p1_sh  <= px_sh(15);

            -- S2: sine ROMs (block RAM); lens phase tI - |u|
            p2_m1  <= C_SIN(to_integer(p1_a1));
            p2_m2  <= C_SIN(to_integer(p1_a2));
            p2_s1  <= p1_s1;
            p2_s2  <= p1_s2;
            p2_in  <= p1_inl and p1_inr;
            p2_pin <= p1_nu + c_ti(27 downto 14);
            p2_pbg <= p1_pbg;
            p2_sh  <= p1_sh and c_sheet;

            -- S3: into fabric; pick the phase and the palette group
            p3_m1 <= unsigned(p2_m1);
            p3_m2 <= unsigned(p2_m2);
            p3_s1 <= p2_s1;
            p3_s2 <= p2_s2;
            p3_in <= p2_in;
            if p2_in = '1' then
                p3_ph  <= p2_pin;
                p3_grp <= '1' & px_par;
            else
                p3_ph  <= p2_pbg;
                p3_grp <= '0' & p2_sh;
            end if;

            -- S4: the two waves summed
            if p3_s1 = '1' then
                v_sum := -signed(resize(p3_m1, 10));
            else
                v_sum := signed(resize(p3_m1, 10));
            end if;
            if p3_s2 = '1' then
                p4_sum <= v_sum - signed(resize(p3_m2, 10));
            else
                p4_sum <= v_sum + signed(resize(p3_m2, 10));
            end if;
            p4_in  <= p3_in;
            p4_ph  <= p3_ph;
            p4_grp <= p3_grp;

            -- S5: magnitude (a slice, never a narrowing resize: |sum|
            -- reaches 510); amplitude (lens slats 1/2, flame slats 1/8)
            v_neg := -p4_sum;
            if p4_sum < 0 then
                p5_mag <= unsigned(v_neg(8 downto 0));
                p5_sgn <= '1';
            else
                p5_mag <= unsigned(p4_sum(8 downto 0));
                p5_sgn <= '0';
            end if;
            if p4_in = '0' then
                p5_Ae <= px_A;
            elsif px_par = '1' then
                p5_Ae <= shift_right(px_A, 3);
            else
                p5_Ae <= shift_right(px_A, 1);
            end if;
            p5_ph  <= p4_ph;
            p5_grp <= p4_grp;

            -- S6: |s| x A, operand split lo/hi
            p6_lo  <= p5_mag(4 downto 0) * p5_Ae;
            p6_hi  <= p5_mag(8 downto 5) * p5_Ae;
            p6_sgn <= p5_sgn;
            p6_ph  <= p5_ph;
            p6_grp <= p5_grp;

            -- S7
            p7_prod <= resize(p6_lo, 22) + shift_left(resize(p6_hi, 22), 5);
            p7_sgn  <= p6_sgn;
            p7_ph   <= p6_ph;
            p7_grp  <= p6_grp;

            -- S8: displaced phase.  D = 64 |s| A13 in band phase, so its
            -- bits 27..14 are the product's 21..8
            if p7_sgn = '1' then
                p8_ph <= p7_ph - p7_prod(21 downto 8);
            else
                p8_ph <= p7_ph + p7_prod(21 downto 8);
            end if;
            p8_grp <= p7_grp;

            -- S9: band row; ramp position r x 7
            p9_band <= p8_ph(13 downto 10);
            p9_t7   <= shift_left(resize(p8_ph(9 downto 2), 11), 3) - resize(p8_ph(9 downto 2), 11);
            p9_grp  <= p8_grp;

            -- S10: even/odd bank addresses for knots k, k+1 (k = 0..6)
            v_t := p9_t7;
            v_e := c_pal & p9_grp & p9_band & v_t(10 downto 8);
            if v_t(8) = '1' then
                v_k1 := v_t(10 downto 9) + 1;
            else
                v_k1 := v_t(10 downto 9);
            end if;
            p10_oa <= v_e(10 downto 1);
            p10_ea <= v_e(10 downto 3) & v_k1;
            p10_sw <= v_t(8);
            p10_fr <= v_t(7 downto 4);

            -- S11: palette ROMs (block RAM)
            p11_ey <= C_EY(to_integer(p10_ea));
            p11_eu <= C_EU(to_integer(p10_ea));
            p11_ev <= C_EV(to_integer(p10_ea));
            p11_oy <= C_OY(to_integer(p10_oa));
            p11_ou <= C_OU(to_integer(p10_oa));
            p11_ov <= C_OV(to_integer(p10_oa));
            p11_sw <= p10_sw;
            p11_fr <= p10_fr;

            -- S12: into fabric before any arithmetic
            p12_ey <= unsigned(p11_ey);
            p12_eu <= unsigned(p11_eu);
            p12_ev <= unsigned(p11_ev);
            p12_oy <= unsigned(p11_oy);
            p12_ou <= unsigned(p11_ou);
            p12_ov <= unsigned(p11_ov);
            p12_sw <= p11_sw;
            p12_fr <= p11_fr;

            -- S13: a = knot k, b = knot k+1; d = b - a
            if p12_sw = '0' then
                p13_ay <= p12_ey;
                p13_au <= p12_eu;
                p13_av <= p12_ev;
                p13_dy <= signed('0' & p12_oy) - signed('0' & p12_ey);
                p13_du <= signed('0' & p12_ou) - signed('0' & p12_eu);
                p13_dv <= signed('0' & p12_ov) - signed('0' & p12_ev);
            else
                p13_ay <= p12_oy;
                p13_au <= p12_ou;
                p13_av <= p12_ov;
                p13_dy <= signed('0' & p12_ey) - signed('0' & p12_oy);
                p13_du <= signed('0' & p12_eu) - signed('0' & p12_ou);
                p13_dv <= signed('0' & p12_ev) - signed('0' & p12_ov);
            end if;
            p13_fr <= p12_fr;

            -- S14: d x fraction
            p14_ay <= p13_ay;
            p14_au <= p13_au;
            p14_av <= p13_av;
            p14_my <= p13_dy * signed('0' & p13_fr);
            p14_mu <= p13_du * signed('0' & p13_fr);
            p14_mv <= p13_dv * signed('0' & p13_fr);

            -- S15: 10-bit colour = (16 a + d f) / 4
            v_cy := signed(resize(p14_ay, 14) sll 4) + p14_my;
            v_cu := signed(resize(p14_au, 14) sll 4) + p14_mu;
            v_cv := signed(resize(p14_av, 14) sll 4) + p14_mv;
            p15_y <= unsigned(v_cy(11 downto 2));
            p15_u <= unsigned(v_cu(11 downto 2));
            p15_v <= unsigned(v_cv(11 downto 2));
        end if;
    end process p_pix;

    -- ====================================================================
    -- S16..S18: output conditioning and the blanking gate
    -- ====================================================================
    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S16/S17: horizontal [1 2 1]/4 (g1 is the centre pixel)
            g1_y <= p15_y;  g2_y <= g1_y;
            g1_u <= p15_u;  g2_u <= g1_u;
            g1_v <= p15_v;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(p15_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(p15_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(p15_v, 12);
            p17_y <= v_sy(11 downto 2);
            p17_u <= v_su(11 downto 2);
            p17_v <= v_sv(11 downto 2);

            -- S18: luma 64 + 13/16 (y - 64) (white ~775); neutral outside
            -- the generated avid
            v_d  := p17_y - 64;
            v_yo := p17_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if sd(17)(3) = '0' then
                s_out_y <= C_BLK;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo;
                s_out_u <= p17_u;
                s_out_v <= p17_v;
            end if;

            sd(1) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            for k in 2 to 18 loop
                sd(k) <= sd(k - 1);
            end loop;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= sd(18)(3);
    data_out.hsync_n <= sd(18)(2);
    data_out.vsync_n <= sd(18)(1);
    data_out.field_n <= sd(18)(0);

end architecture totem;
