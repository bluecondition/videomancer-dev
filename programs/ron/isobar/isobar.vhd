-- isobar.vhd  (v0.1.0) -- kaleidoscope contour colourizer
--
-- ISOBAR.  An analog colourizer patch rebuilt in one pass, after a reference
-- capture: folded ramps feed a curved field, the field's contours are drawn
-- as soft chroma-only stripes in bands, smooth colour pools sit between the
-- bands (a blue heart, an orange glow ...), and a separate luma spotlight
-- lights it all from the top.  Synthesis: no input video is read.
--
-- PICTURE
--  1. PANES.  X runs -K..K across the screen; p = tri(X) (period 1) folds it
--     into a kaleidoscope: a mirror at the centre and creases every 1/(2K)
--     of the half-width.  p = 0 on the mirror lines, 1 on the creases.
--  2. FIELD.  Per line yn runs -1 (top) .. +1 (bottom).
--        F1 = beta |yn - yc| + alpha (yn - y1) p + C p^2
--     beta 2.06, yc -0.055, y1 0.09, C 3: the cross term alpha (Shape)
--     makes the pool a heart (notch on top, tip below).  A sqrt ROM turns F1
--     into an even-spaced distance D1.  The crease diamond is
--        D2 = 2 (1 - p) + 2 |yn - 0.25|
--     and D = min(D1, D2) (Crease); the seams where the two meet are the
--     V-kinks.  A sine bends D: Warp . sin(pi p + 2 pi (0.7 yn + phase)).
--  3. RINGS.  m = D . g + phase (g = Density).  floor(m) is the ring (palette
--     row mod 4), frac(m) the place in it: the first Pools fraction is a
--     smooth pool, the rest six contour stripes.  The palette ROM gives the
--     chroma (and a small luma offset) for (palette, row, pool, pos 0..63).
--  4. LUMA.  A spotlight tent, peak(y) - 5/16 (1 - p), brightest on the
--     creases at the top (Glow), or flat.  Chroma fades in over the first
--     62 codes above black.  Etch cuts the stripes into the luma.
--
-- ENGINE
--   One serial shift-add multiplier and one serial restoring divider, both
--   driven by operation codes.  Per field (first vsync edge after active
--   video): 2^24/W, 2^20/H, the knob curves, the X step, the pool and
--   stripe reciprocals, the density (with the heartbeat), speed and phases.
--   Per active line (kicked at the generated avid's start, for the NEXT
--   line, ~80 clocks): yn, the mirror, beta|yn-yc|, alpha(yn-y1), the warp
--   line phase, the luma peak and the crease term.
--
-- CONTROLS
--   K1 Shape    heart cross-term alpha -2.5 .. 5: bowtie / oval / heart / spade
--   K2 Panes    K 0.5 .. 4 (square law): one mirror .. many narrow panes
--   K3 Pools    pool share of each ring 6 .. 85 %
--   K4 Warp     sine bend of the contours, 0 .. 1/2 of a D unit
--   K5 Speed    bipolar, still in the centre: rings flow out (right) / in
--   K6 Palette  Psyche (fitted to the reference) / Ember / Ice / Acid
--   S7 Crease   crease diamonds Off / On
--   S8 Mirror   top mirrored onto the bottom Off / On
--   S9 Pulse    heartbeat on the density Off / On
--   S10 Glow    flat luma / spotlight tent
--   S11 Etch    stripes cut into the luma Off / On
--   P12 Density rings across the picture, g 0.15 .. 4 (square law).  0 = one
--               calm pool, 100 % = dense vibrating contours everywhere
--
-- HARDWARE CONTRACTS: own generated avid (parity-locked start, fixed width:
-- the encoders pair Cb/Cr from hsync); one latency for all modes (30
-- stages); outputs gated to neutral outside the generated avid; palette
-- stored U/V-swapped (isobar.py); per-field work on the first vsync EDGE
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
use work.isobar_pkg.all;

architecture isobar of program_top is

    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- field constants (yn in 2^-12)
    constant C_YC   : integer := -225;     -- heart centre line, -0.055
    constant C_Y1   : integer := 369;      -- cross-term zero line, 0.09
    constant C_Y2   : integer := 1024;     -- crease diamond centre, 0.25
    constant C_BETA : integer := 527;      -- 2.06 (2^-8)
    constant C_LT   : integer := 764;      -- luma peak at the top
    constant C_LTB  : integer := 364;      -- top - bottom (400 at the bottom)

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
    -- knob glides: 0 K1 Shape, 1 K2 Panes, 2 K3 Pools, 3 K4 Warp,
    -- 4 P12 Density.  Deadband hold + per-field ease, 10.4
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
    constant OP_K2SQ : integer := 0;    -- k2 x k2
    constant OP_KQ   : integer := 1;    -- k2^2 x 896
    constant OP_STEP : integer := 2;    -- 2^24/W x Kq
    constant OP_AL   : integer := 3;    -- k1 x 15
    constant OP_PF   : integer := 4;    -- k3 x 3236
    constant OP_GSQ  : integer := 5;    -- k12 x k12
    constant OP_G985 : integer := 6;    -- k12^2 x 985
    constant OP_GB   : integer := 7;    -- g x beat
    constant OP_SPD  : integer := 8;    -- mag x mag
    constant OP_B    : integer := 9;    -- |yn - yc| x beta
    constant OP_A    : integer := 10;   -- (yn - y1) x |alpha|
    constant OP_W    : integer := 11;   -- yn x 179
    constant OP_L    : integer := 12;   -- (yn + 1) x 434

    -- serial multiplier: signed a x unsigned b.  The sequencer writes only
    -- an operation code; the multiplier muxes its own operands from it one
    -- clock later (so no wide operand mux hangs off the state decode)
    signal mul_go, mul_busy : std_logic := '0';
    signal mul_op : unsigned(3 downto 0) := (others => '0');
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
    signal c_crease, c_mirror, c_pulse, c_glow, c_etch : std_logic := '0';
    signal c_W    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal c_H    : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal c_recw : unsigned(15 downto 0) := to_unsigned(8738, 16);  -- 2^24 / W
    signal c_rech : unsigned(12 downto 0) := to_unsigned(970, 13);   -- 2^20 / field lines
    signal c_Kq   : unsigned(10 downto 0) := to_unsigned(278, 11);   -- panes K (2^-8)
    signal c_stepx  : unsigned(19 downto 0) := (others => '0');      -- X step (2^-24 / px)
    signal c_startx : signed(27 downto 0) := (others => '0');
    signal c_alq  : signed(11 downto 0) := (others => '0');          -- alpha (2^-8)
    signal c_absal : unsigned(10 downto 0) := (others => '0');
    signal c_alneg : std_logic := '0';
    signal c_pfq  : unsigned(11 downto 0) := to_unsigned(1844, 12);  -- pool fraction (2^-12)
    signal c_rpf  : unsigned(12 downto 0) := (others => '0');        -- 2^18 / pf
    signal c_rstr : unsigned(12 downto 0) := (others => '0');        -- 6 . 2^18 / (1 - pf)
    signal c_Wq   : unsigned(7 downto 0) := (others => '0');         -- warp depth
    signal c_gq   : unsigned(10 downto 0) := to_unsigned(137, 11);   -- density (2^-8)
    signal c_geff : unsigned(10 downto 0) := to_unsigned(137, 11);
    signal c_bph  : unsigned(15 downto 0) := (others => '0');        -- heartbeat phase
    signal c_bt : unsigned(7 downto 0) := (others => '0');
    signal c_dir  : std_logic := '0';
    signal c_mag  : unsigned(8 downto 0) := (others => '0');
    signal c_spd  : unsigned(7 downto 0) := (others => '0');
    signal c_phit : unsigned(13 downto 0) := to_unsigned(900, 14);   -- ring phase (2^-12)
    signal c_phiw : unsigned(15 downto 0) := (others => '0');        -- warp phase

    -- per-line engine.  The DDA holds the CURRENT line's yn until the
    -- engine (kicked at this line's start) advances it; the results (l_B ..
    -- l_L) are written during this line for the next one.  The pixel path
    -- copies them at g_go.
    signal l_ya   : signed(21 downto 0) := (others => '0');          -- yn (2^-20)
    signal l_ynm  : signed(13 downto 0) := (others => '0');          -- yn (2^-12), mirrored
    signal l_rc, l_r2 : signed(14 downto 0) := (others => '0');
    signal l_dy1  : signed(14 downto 0) := (others => '0');
    signal l_li   : unsigned(13 downto 0) := (others => '0');        -- yn + 1
    signal l_dyc, l_dy2 : unsigned(13 downto 0) := (others => '0');
    signal l_Ar   : signed(15 downto 0) := (others => '0');
    signal l_B    : unsigned(12 downto 0) := (others => '0');        -- beta |yn - yc| (2^-10)
    signal l_A    : signed(15 downto 0) := (others => '0');          -- alpha (yn - y1) (2^-10)
    signal l_E    : unsigned(11 downto 0) := (others => '0');        -- crease term (2^-9)
    signal l_w    : unsigned(15 downto 0) := (others => '0');        -- warp line phase
    signal l_L    : unsigned(9 downto 0) := (others => '0');         -- luma peak

    ------------------------------------------------------------------------
    -- pixel path
    ------------------------------------------------------------------------
    signal px_xa  : signed(27 downto 0) := (others => '0');
    signal px_B   : unsigned(12 downto 0) := (others => '0');
    signal px_A   : signed(15 downto 0) := (others => '0');
    signal px_E   : unsigned(11 downto 0) := (others => '0');
    signal px_w   : unsigned(15 downto 0) := (others => '0');
    signal px_L   : unsigned(9 downto 0) := (others => '0');

    signal s1_p   : unsigned(11 downto 0) := (others => '0');
    signal s2_p   : unsigned(11 downto 0) := (others => '0');
    signal s2_p3  : unsigned(13 downto 0) := (others => '0');
    signal s2_D2  : unsigned(12 downto 0) := (others => '0');
    signal s2_wph : unsigned(15 downto 0) := (others => '0');
    signal s2_tent : unsigned(9 downto 0) := (others => '0');
    signal s3_S   : signed(15 downto 0) := (others => '0');
    signal s3_p   : unsigned(11 downto 0) := (others => '0');
    signal s3_qa  : unsigned(7 downto 0) := (others => '0');
    signal s3_ws  : std_logic := '0';
    signal s3_D2  : unsigned(11 downto 0) := (others => '0');
    signal s3_Lb  : signed(11 downto 0) := (others => '0');
    signal s4_lo, s4_hi : signed(22 downto 0) := (others => '0');
    signal s4_sin : std_logic_vector(7 downto 0) := (others => '0');
    signal s5_Sp  : signed(17 downto 0) := (others => '0');
    signal s5_sv  : unsigned(7 downto 0) := (others => '0');
    signal s6_F   : signed(17 downto 0) := (others => '0');
    signal s6_wp  : unsigned(15 downto 0) := (others => '0');
    signal s7_addr : unsigned(10 downto 0) := (others => '0');
    signal s7_wp  : unsigned(7 downto 0) := (others => '0');
    signal s8_D1  : std_logic_vector(11 downto 0) := (others => '0');
    signal s8_wp  : unsigned(7 downto 0) := (others => '0');
    signal s9_D1  : unsigned(11 downto 0) := (others => '0');
    signal s9_wp  : unsigned(7 downto 0) := (others => '0');
    signal s10_D  : unsigned(11 downto 0) := (others => '0');
    signal s10_wp : unsigned(7 downto 0) := (others => '0');
    signal s11_Dw : signed(13 downto 0) := (others => '0');
    signal s12_Dw : unsigned(11 downto 0) := (others => '0');
    signal s13_lo, s13_hi : unsigned(16 downto 0) := (others => '0');
    signal s14_m  : unsigned(13 downto 0) := (others => '0');
    signal s15_m  : unsigned(13 downto 0) := (others => '0');
    signal s16_d  : signed(12 downto 0) := (others => '0');
    signal s16_f  : unsigned(11 downto 0) := (others => '0');
    signal s17_a  : unsigned(11 downto 0) := (others => '0');
    signal s17_b  : unsigned(12 downto 0) := (others => '0');
    signal s18_lo, s18_hi : unsigned(18 downto 0) := (others => '0');
    signal s19_pos : unsigned(5 downto 0) := (others => '0');
    signal s20_addr : unsigned(10 downto 0) := (others => '0');
    signal s20_t  : unsigned(4 downto 0) := (others => '0');
    signal s21_py : std_logic_vector(3 downto 0) := (others => '0');
    signal s21_pu, s21_pv : std_logic_vector(7 downto 0) := (others => '0');
    signal s21_dip : unsigned(7 downto 0) := (others => '0');
    signal s22_py : signed(3 downto 0) := (others => '0');
    signal s22_pu, s22_pv : signed(7 downto 0) := (others => '0');
    signal s22_L1 : signed(11 downto 0) := (others => '0');
    signal s23_L2 : signed(11 downto 0) := (others => '0');
    signal s23_pu, s23_pv : signed(7 downto 0) := (others => '0');
    signal s24_L  : unsigned(9 downto 0) := (others => '0');
    signal s24_pu, s24_pv : signed(7 downto 0) := (others => '0');
    signal s25_L  : unsigned(9 downto 0) := (others => '0');
    signal s25_cs : unsigned(4 downto 0) := (others => '0');
    signal s25_pu, s25_pv : signed(7 downto 0) := (others => '0');
    signal s26_L  : unsigned(9 downto 0) := (others => '0');
    signal s26_mu, s26_mv : signed(13 downto 0) := (others => '0');
    signal p27_y, p27_u, p27_v : unsigned(9 downto 0) := (others => '0');

    -- row / pool flags and the luma base ride along the path
    type t_row is array(16 to 19) of unsigned(1 downto 0);
    signal r_row  : t_row := (others => (others => '0'));
    signal r_pool : std_logic_vector(17 to 19) := (others => '0');
    type t_d2 is array(4 to 9) of unsigned(11 downto 0);
    signal r_D2   : t_d2 := (others => (others => '0'));
    type t_ws is array(4 to 10) of std_logic;
    signal r_ws   : t_ws := (others => '0');
    type t_lb is array(4 to 21) of signed(11 downto 0);
    signal r_Lb   : t_lb := (others => (others => '0'));

    -- output conditioning (the HD HDMI bar fix, damier v0.4.4): [1 2 1]
    -- horizontal smoothing on Y/U/V, luma scaled toward black
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal p29_y, p29_u, p29_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- syncs (generated avid, hsync, vsync, field) from S1 to S30
    type t_sd is array(1 to 30) of std_logic_vector(3 downto 0);
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
    -- (by 1/8; the Density slider by 1/4 so the gesture stays quick)
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
    begin
        if rising_edge(clk) then
            if mul_go = '1' then
                v_a := (others => '0');
                v_b := (others => '0');
                v_n := to_unsigned(10, 4);
                case to_integer(mul_op) is
                    when OP_K2SQ =>
                        v_a := signed(resize(c_k2, 25)); v_b := resize(c_k2, 14);
                    when OP_KQ =>
                        v_a := signed(resize(unsigned(m_acc(19 downto 0)), 25));
                        v_b := to_unsigned(896, 14);
                    when OP_STEP =>
                        v_a := signed(resize(c_recw, 25)); v_b := resize(c_Kq, 14);
                        v_n := to_unsigned(11, 4);
                    when OP_AL =>
                        v_a := signed(resize(c_k1, 25)); v_b := to_unsigned(15, 14);
                        v_n := to_unsigned(4, 4);
                    when OP_PF =>
                        v_a := signed(resize(c_k3, 25)); v_b := to_unsigned(3236, 14);
                        v_n := to_unsigned(12, 4);
                    when OP_GSQ =>
                        v_a := signed(resize(c_p12, 25)); v_b := resize(c_p12, 14);
                    when OP_G985 =>
                        v_a := signed(resize(unsigned(m_acc(19 downto 0)), 25));
                        v_b := to_unsigned(985, 14);
                    when OP_GB =>
                        v_a := signed(resize(c_gq, 25)); v_b := resize(c_bt, 14);
                        v_n := to_unsigned(8, 4);
                    when OP_SPD =>
                        v_a := signed(resize(c_mag, 25)); v_b := resize(c_mag, 14);
                        v_n := to_unsigned(9, 4);
                    when OP_B =>
                        v_a := signed(resize(l_dyc, 25)); v_b := to_unsigned(C_BETA, 14);
                    when OP_A =>
                        v_a := resize(l_dy1, 25); v_b := resize(c_absal, 14);
                        v_n := to_unsigned(11, 4);
                    when OP_W =>
                        v_a := resize(l_ynm, 25); v_b := to_unsigned(179, 14);
                        v_n := to_unsigned(8, 4);
                    when others =>          -- OP_L
                        v_a := signed(resize(l_li, 25)); v_b := to_unsigned(C_LTB, 14);
                        v_n := to_unsigned(9, 4);
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
    -- Serial divider: q = floor(num / den), one quotient bit per clock.
    -- 0: 2^24 / W, 1: 2^20 / H, 2: 2^18 / pf, 3: 6 . 2^18 / (1 - pf)
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
                    when 2 =>
                        d_num <= to_unsigned(2 ** 18, 32);
                        d_den <= c_pfq;
                    when others =>
                        d_num <= to_unsigned(6 * 2 ** 18, 32);
                        d_den <= (not c_pfq) + 1;            -- 4096 - pf
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
    -- Sequencer: per-field set-up (states 1..16) and the per-line engine
    -- (64..71).  One small operation per state; a state may capture a
    -- result and issue the next operation (the units read their operands
    -- one clock after go, so a value written in the issuing state is
    -- seen).  A unit is free when its go and busy are both low.
    -- ====================================================================
    p_seq : process(clk)
        variable v_k6  : unsigned(9 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_ad  : unsigned(9 downto 0);
        variable v_st  : unsigned(19 downto 0);
        variable v_yn  : signed(13 downto 0);
        variable v_g   : unsigned(11 downto 0);
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
                    c_crease <= registers_in(6)(0);
                    c_mirror <= registers_in(6)(1);
                    c_pulse  <= registers_in(6)(2);
                    c_glow   <= registers_in(6)(3);
                    c_etch   <= registers_in(6)(4);
                    c_W      <= s_meas_w;
                    c_H      <= s_meas_h;
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

                when 3 =>       -- line step; Panes squared
                    if div_go = '0' and div_busy = '0' then
                        if d_q(31 downto 13) /= 0 then
                            c_rech <= (others => '1');
                        else
                            c_rech <= d_q(12 downto 0);
                        end if;
                        mul_op <= to_unsigned(OP_K2SQ, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(4, 7);
                    end if;

                when 4 =>
                    if mul_go = '0' and mul_busy = '0' then
                        mul_op <= to_unsigned(OP_KQ, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(5, 7);
                    end if;

                when 5 =>       -- K = 0.5 + 3.5 k2^2 (2^-8); X step = K . 2^25 / W
                    if mul_go = '0' and mul_busy = '0' then
                        c_Kq   <= to_unsigned(128, 11) + resize(unsigned(m_acc(29 downto 20)), 11);
                        mul_op <= to_unsigned(OP_STEP, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(6, 7);
                    end if;

                when 6 =>       -- X starts at -K (plus half a step: pixel centres
                                -- mirror exactly about the screen centre); Shape
                    if mul_go = '0' and mul_busy = '0' then
                        v_st     := unsigned(m_acc(26 downto 7));
                        c_stepx  <= v_st;
                        c_startx <= signed(resize(v_st(19 downto 1), 28))
                                    - signed(shift_left(resize(c_Kq, 28), 16));
                        mul_op   <= to_unsigned(OP_AL, 4);
                        mul_go   <= '1';
                        q_st     <= to_unsigned(7, 7);
                    end if;

                when 7 =>       -- alpha = -2.5 + 7.5 k1 (2^-8); Pools
                    if mul_go = '0' and mul_busy = '0' then
                        c_alq  <= signed(resize(unsigned(m_acc(13 downto 3)), 12)) - 640;
                        mul_op <= to_unsigned(OP_PF, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(8, 7);
                    end if;

                when 8 =>       -- pf = 0.06 + 0.79 k3 (2^-12); |alpha|; 2^18 / pf
                    if mul_go = '0' and mul_busy = '0' then
                        c_pfq <= to_unsigned(245, 12) + resize(unsigned(m_acc(21 downto 10)), 12);
                        if c_alq < 0 then
                            c_alneg <= '1';
                            c_absal <= unsigned(resize(-c_alq, 11));
                        else
                            c_alneg <= '0';
                            c_absal <= unsigned(resize(c_alq, 11));
                        end if;
                        div_op <= to_unsigned(2, 2);
                        div_go <= '1';
                        q_st   <= to_unsigned(9, 7);
                    end if;

                when 9 =>       -- pool reciprocal; stripe scale 6 / (1 - pf)
                    if div_go = '0' and div_busy = '0' then
                        if d_q(31 downto 13) /= 0 then
                            c_rpf <= (others => '1');
                        else
                            c_rpf <= d_q(12 downto 0);
                        end if;
                        div_op <= to_unsigned(3, 2);
                        div_go <= '1';
                        q_st   <= to_unsigned(10, 7);
                    end if;

                when 10 =>      -- Density squared
                    if div_go = '0' and div_busy = '0' then
                        if d_q(31 downto 13) /= 0 then
                            c_rstr <= (others => '1');
                        else
                            c_rstr <= d_q(12 downto 0);
                        end if;
                        mul_op <= to_unsigned(OP_GSQ, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(11, 7);
                    end if;

                when 11 =>
                    if mul_go = '0' and mul_busy = '0' then
                        mul_op <= to_unsigned(OP_G985, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(12, 7);
                    end if;

                when 12 =>      -- g = 0.15 + 3.85 k12^2 (2^-8); heartbeat ~72 bpm
                    if mul_go = '0' and mul_busy = '0' then
                        c_gq   <= to_unsigned(38, 11) + resize(unsigned(m_acc(29 downto 20)), 11);
                        c_bph  <= c_bph + 1311;
                        c_bt <= unsigned(C_BEAT(to_integer(c_bph(15 downto 11))));
                        mul_op <= to_unsigned(OP_GB, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(13, 7);
                    end if;

                when 13 =>      -- Pulse: g . (1 + beat / 2); Speed: bipolar with a
                                -- +-24 still zone
                    if mul_go = '0' and mul_busy = '0' then
                        v_g := resize(c_gq, 12) + resize(unsigned(m_acc(18 downto 9)), 12);
                        if c_pulse = '0' then
                            c_geff <= c_gq;
                        elsif v_g(11) = '1' then
                            c_geff <= (others => '1');
                        else
                            c_geff <= v_g(10 downto 0);
                        end if;
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
                        mul_op <= to_unsigned(OP_SPD, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(14, 7);
                    end if;

                when 14 =>      -- speed = mag^2 / 1024 (ring phase per field)
                    if mul_go = '0' and mul_busy = '0' then
                        c_spd <= unsigned(m_acc(17 downto 10));
                        c_Wq  <= c_k4(9 downto 2);
                        q_st  <= to_unsigned(15, 7);
                    end if;

                when 15 =>      -- rings flow out (right of centre) or in; the
                                -- warp undulates at 4x the ring rate (in its own turns)
                    if c_dir = '0' then
                        c_phit <= c_phit - resize(c_spd, 14);
                        c_phiw <= c_phiw - shift_left(resize(c_spd, 16), 2);
                    else
                        c_phit <= c_phit + resize(c_spd, 14);
                        c_phiw <= c_phiw + shift_left(resize(c_spd, 16), 2);
                    end if;
                    q_st <= to_unsigned(16, 7);

                when 16 =>      -- field start; prime the line engine for line 0
                    if s_ilace = '1' and s_timing.field_n = '0' then
                        -- bottom field: one frame line (half a field step) down
                        l_ya <= to_signed(-(2 ** 20), 22) + signed(resize(c_rech, 22));
                    else
                        l_ya <= to_signed(-(2 ** 20), 22);
                    end if;
                    q_prime <= '1';
                    q_st    <= to_unsigned(64, 7);

                ------------------------------------------------------------
                -- per line (for the NEXT line)
                ------------------------------------------------------------
                when 64 =>      -- yn steps 2/H per field line
                    if q_prime = '0' then
                        l_ya <= l_ya + signed(shift_left(resize(c_rech, 22), 1));
                    end if;
                    q_prime <= '0';
                    q_st    <= to_unsigned(65, 7);

                when 65 =>      -- yn (2^-12); Mirror folds the bottom onto the top
                    v_yn := l_ya(21 downto 8);
                    if c_mirror = '1' and v_yn > 0 then
                        l_ynm <= -v_yn;
                    else
                        l_ynm <= v_yn;
                    end if;
                    q_st <= to_unsigned(66, 7);

                when 66 =>      -- the field's line offsets
                    l_rc  <= resize(l_ynm, 15) - C_YC;
                    l_dy1 <= resize(l_ynm, 15) - C_Y1;
                    l_r2  <= resize(l_ynm, 15) - C_Y2;
                    l_li  <= unsigned(resize(l_ynm, 14) + 4096);
                    q_st  <= to_unsigned(67, 7);

                when 67 =>      -- magnitudes; beta |yn - yc|
                    l_dyc  <= unsigned(resize(abs(l_rc), 14));
                    l_dy2  <= unsigned(resize(abs(l_r2), 14));
                    mul_op <= to_unsigned(OP_B, 4);
                    mul_go <= '1';
                    q_st   <= to_unsigned(68, 7);

                when 68 =>      -- (2^-10); alpha (yn - y1)
                    if mul_go = '0' and mul_busy = '0' then
                        l_B    <= unsigned(m_acc(22 downto 10));
                        mul_op <= to_unsigned(OP_A, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(69, 7);
                    end if;

                when 69 =>      -- (2^-10, sign of alpha applied next); warp phase
                    if mul_go = '0' and mul_busy = '0' then
                        l_Ar   <= m_acc(25 downto 10);
                        mul_op <= to_unsigned(OP_W, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(70, 7);
                    end if;

                when 70 =>      -- warp: 0.7 turn per unit yn; luma peak
                    if mul_go = '0' and mul_busy = '0' then
                        l_w <= c_phiw + unsigned(m_acc(19 downto 4));
                        if c_alneg = '1' then
                            l_A <= -l_Ar;
                        else
                            l_A <= l_Ar;
                        end if;
                        mul_op <= to_unsigned(OP_L, 4);
                        mul_go <= '1';
                        q_st   <= to_unsigned(71, 7);
                    end if;

                when 71 =>      -- peak 764 at the top .. 400 at the bottom;
                                -- crease term 2 |yn - y2| (2^-9)
                    if mul_go = '0' and mul_busy = '0' then
                        l_L  <= to_unsigned(C_LT, 10) - unsigned(m_acc(22 downto 13));
                        l_E  <= resize(l_dy2(13 downto 2), 12);
                        q_st <= to_unsigned(0, 7);
                    end if;

                when others =>
                    q_st <= to_unsigned(0, 7);
            end case;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Pixel path.  S0 = the clock where px_xa holds the pixel and g_avid
    -- its avid; S30 = the output register.
    -- ====================================================================
    p_pix : process(clk)
        variable v_ms  : std_logic_vector(11 downto 0);
        variable v_np  : unsigned(11 downto 0);
        variable v_t3  : unsigned(14 downto 0);
        variable v_s30 : signed(29 downto 0);
        variable v_m   : unsigned(23 downto 0);
        variable v_g   : unsigned(25 downto 0);
        variable v_t   : unsigned(4 downto 0);
        variable v_c   : unsigned(9 downto 0);
        variable v_u, v_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- S0: X across the screen (2^-24 pane units); line values
            if g_go = '1' then
                px_xa <= c_startx;
                px_B  <= l_B;
                px_A  <= l_A;
                px_E  <= l_E;
                px_w  <= l_w;
                px_L  <= l_L;
            else
                px_xa <= px_xa + signed(resize(c_stepx, 28));
            end if;

            -- S1: p = tri(X): the triangle is even, so the fraction bits
            -- fold straight from two's complement (no abs)
            v_ms := (others => px_xa(23));
            s1_p <= unsigned(std_logic_vector(px_xa(22 downto 11)) xor v_ms);

            -- S2: 3p (for C p = 3p/4); crease term; warp phase; luma tent
            -- 5/16 (1 - p)
            v_np   := not s1_p;                        -- 4095 - p
            s2_p   <= s1_p;
            s2_p3  <= resize(s1_p, 14) + shift_left(resize(s1_p, 14), 1);
            s2_D2  <= resize(v_np(11 downto 2), 13) + resize(px_E, 13);
            s2_wph <= px_w + shift_left(resize(s1_p, 16), 3);
            if c_glow = '1' then
                v_t3    := resize(v_np, 15) + shift_left(resize(v_np, 15), 2);
                s2_tent <= resize(v_t3(14 downto 6), 10);
            else
                s2_tent <= to_unsigned(160, 10);
            end if;

            -- S3: S = A + C p; sine index; D2 clamp; luma base
            s3_S  <= px_A + signed(resize(s2_p3(13 downto 2), 16));
            s3_p  <= s2_p;
            s3_qa <= f_qidx(s2_wph);
            s3_ws <= s2_wph(15);
            if s2_D2(12) = '1' then
                s3_D2 <= (others => '1');
            else
                s3_D2 <= s2_D2(11 downto 0);
            end if;
            s3_Lb <= signed(resize(px_L, 12)) - signed(resize(s2_tent, 12));

            -- S4: S x p, operand split lo/hi; sine ROM (block RAM)
            s4_lo  <= s3_S * signed('0' & s3_p(5 downto 0));
            s4_hi  <= s3_S * signed('0' & s3_p(11 downto 6));
            s4_sin <= C_SIN(to_integer(s3_qa));
            r_ws(4) <= s3_ws;
            r_D2(4) <= s3_D2;
            r_Lb(4) <= s3_Lb;

            -- S5: S p (2^-10); sine into fabric
            v_s30 := resize(s4_lo, 30) + shift_left(resize(s4_hi, 30), 6);
            s5_Sp <= v_s30(29 downto 12);
            s5_sv <= unsigned(s4_sin);

            -- S6: F1 = B + S p; warp = sin x depth
            s6_F  <= signed(resize(px_B, 18)) + s5_Sp;
            s6_wp <= s5_sv * c_Wq;

            -- S7: sqrt address: fine range below F = 1, coarse above
            if s6_F < 0 then
                s7_addr <= (others => '0');
            elsif s6_F(17 downto 14) /= "0000" then
                s7_addr <= (others => '1');
            elsif s6_F(13 downto 10) = "0000" then
                s7_addr <= '0' & unsigned(s6_F(9 downto 0));
            else
                s7_addr <= '1' & unsigned(s6_F(13 downto 4));
            end if;
            s7_wp <= s6_wp(15 downto 8);

            -- S8: sqrt ROM (block RAM)
            s8_D1 <= C_SQ(to_integer(s7_addr));
            s8_wp <= s7_wp;

            -- S9: into fabric
            s9_D1 <= unsigned(s8_D1);
            s9_wp <= s8_wp;

            -- S10: D = min(D1, D2) with Crease
            if c_crease = '1' and r_D2(9) < s9_D1 then
                s10_D <= r_D2(9);
            else
                s10_D <= s9_D1;
            end if;
            s10_wp <= s9_wp;

            -- S11: warp
            if r_ws(10) = '1' then
                s11_Dw <= signed(resize(s10_D, 14)) - signed(resize(s10_wp, 14));
            else
                s11_Dw <= signed(resize(s10_D, 14)) + signed(resize(s10_wp, 14));
            end if;

            -- S12: clamp to 0 .. 8 D units
            if s11_Dw < 0 then
                s12_Dw <= (others => '0');
            elsif s11_Dw(13 downto 12) /= "00" then
                s12_Dw <= (others => '1');
            else
                s12_Dw <= unsigned(s11_Dw(11 downto 0));
            end if;

            -- S13: D x g, operand split lo/hi
            s13_lo <= s12_Dw(5 downto 0) * c_geff;
            s13_hi <= s12_Dw(11 downto 6) * c_geff;

            -- S14: m = D g (2^-12 per ring), 14 bits (ring mod 4)
            v_m   := resize(s13_lo, 24) + shift_left(resize(s13_hi, 24), 6);
            s14_m <= v_m(18 downto 5);

            -- S15: + the ring phase
            s15_m <= s14_m + c_phit;

            -- S16: place in the ring minus the pool fraction
            s16_d    <= signed(resize(s15_m(11 downto 0), 13)) - signed(resize(c_pfq, 13));
            s16_f    <= s15_m(11 downto 0);
            r_row(16) <= s15_m(13 downto 12);

            -- S17: pool (f < pf) or stripes: multiplier operands
            r_pool(17) <= s16_d(12);
            if s16_d(12) = '1' then
                s17_a <= s16_f;
                s17_b <= c_rpf;
            else
                s17_a <= unsigned(s16_d(11 downto 0));
                s17_b <= c_rstr;
            end if;
            r_row(17) <= r_row(16);

            -- S18: position product, operand split lo/hi
            s18_lo <= s17_a(5 downto 0) * s17_b;
            s18_hi <= s17_a(11 downto 6) * s17_b;
            r_pool(18) <= r_pool(17);
            r_row(18)  <= r_row(17);

            -- S19: pos = 64 f / pf (pool) or the place in one stripe
            v_g     := resize(s18_lo, 26) + shift_left(resize(s18_hi, 26), 6);
            s19_pos <= v_g(17 downto 12);
            r_pool(19) <= r_pool(18);
            r_row(19)  <= r_row(18);

            -- S20: palette address; Etch dip (deepest on the green pole)
            s20_addr <= c_pal & r_row(19) & r_pool(19) & s19_pos;
            if s19_pos(5) = '1' then
                v_t := not s19_pos(4 downto 0);
            else
                v_t := s19_pos(4 downto 0);
            end if;
            if c_etch = '1' and r_pool(19) = '0' then
                s20_t <= v_t;
            else
                s20_t <= (others => '0');
            end if;

            -- S21: palette ROMs (block RAM); dip x 6
            s21_py  <= C_PY(to_integer(s20_addr));
            s21_pu  <= C_PU(to_integer(s20_addr));
            s21_pv  <= C_PV(to_integer(s20_addr));
            s21_dip <= shift_left(resize(s20_t, 8), 2) + shift_left(resize(s20_t, 8), 1);

            -- S22: into fabric; luma base minus the dip
            s22_py <= signed(s21_py);
            s22_pu <= signed(s21_pu);
            s22_pv <= signed(s21_pv);
            s22_L1 <= r_Lb(21) - signed(resize(s21_dip, 12));

            -- S23: + the palette's luma offset (x16)
            s23_L2 <= s22_L1 + shift_left(resize(s22_py, 12), 4);
            s23_pu <= s22_pu;
            s23_pv <= s22_pv;

            -- S24: luma 64 .. 768
            if s23_L2 < 64 then
                s24_L <= to_unsigned(64, 10);
            elsif s23_L2 > 768 then
                s24_L <= to_unsigned(768, 10);
            else
                s24_L <= unsigned(s23_L2(9 downto 0));
            end if;
            s24_pu <= s23_pu;
            s24_pv <= s23_pv;

            -- S25: chroma fades in over the first 62 codes above black
            v_c := s24_L - 64;
            if v_c(9 downto 6) /= "0000" or v_c(5 downto 1) = "11111" then
                s25_cs <= (others => '1');
            else
                s25_cs <= v_c(5 downto 1);
            end if;
            s25_L  <= s24_L;
            s25_pu <= s24_pu;
            s25_pv <= s24_pv;

            -- S26: chroma x fade
            s26_mu <= s25_pu * signed('0' & s25_cs);
            s26_mv <= s25_pv * signed('0' & s25_cs);
            s26_L  <= s25_L;

            -- S27: 10-bit colour (offsets were x4: (c x cs / 32) x 4)
            v_u   := to_signed(512, 12) + resize(shift_right(s26_mu, 3), 12);
            v_v   := to_signed(512, 12) + resize(shift_right(s26_mv, 3), 12);
            p27_y <= s26_L;
            p27_u <= unsigned(v_u(9 downto 0));
            p27_v <= unsigned(v_v(9 downto 0));

            -- the ride-along pipes
            for k in 5 to 10 loop
                r_ws(k) <= r_ws(k - 1);
            end loop;
            for k in 5 to 9 loop
                r_D2(k) <= r_D2(k - 1);
            end loop;
            for k in 5 to 21 loop
                r_Lb(k) <= r_Lb(k - 1);
            end loop;
        end if;
    end process p_pix;

    -- ====================================================================
    -- S28..S30: output conditioning and the blanking gate
    -- ====================================================================
    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S28/S29: horizontal [1 2 1]/4 (g1 is the centre pixel)
            g1_y <= p27_y;  g2_y <= g1_y;
            g1_u <= p27_u;  g2_u <= g1_u;
            g1_v <= p27_v;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(p27_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(p27_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(p27_v, 12);
            p29_y <= v_sy(11 downto 2);
            p29_u <= v_su(11 downto 2);
            p29_v <= v_sv(11 downto 2);

            -- S30: luma 64 + 13/16 (y - 64) (white ~775); neutral outside
            -- the generated avid
            v_d  := p29_y - 64;
            v_yo := p29_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if sd(29)(3) = '0' then
                s_out_y <= C_BLK;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo;
                s_out_u <= p29_u;
                s_out_v <= p29_v;
            end if;

            sd(1) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            for k in 2 to 30 loop
                sd(k) <= sd(k - 1);
            end loop;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= sd(30)(3);
    data_out.hsync_n <= sd(30)(2);
    data_out.vsync_n <= sd(30)(1);
    data_out.field_n <= sd(30)(0);

end architecture isobar;
