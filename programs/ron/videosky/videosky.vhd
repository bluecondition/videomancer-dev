-- videosky.vhd  (v0.1 -- the picture cut into a copper line-screen)
--
-- Lifted out of PENROSE, whose "Video Sky" switch engraved the incoming video
-- into the sky behind the impossible staircase.  That was eight lines of code:
-- horizontal rules at a fixed 8-pixel pitch, each rule's thickness set by the
-- top three bits of the inverted luma, drawn in ink on paper (penrose.vhd,
-- stage C2).  Dark video -> fat rule -> the paper reads dark.  It is the oldest
-- reproduction trick there is: a mezzotint line screen.
--
-- VideoSky keeps that kernel exactly and gives it a whole instrument:
--
--  * CONTINUOUS PITCH AND ANGLE.  Penrose tested `v_count mod 8`, so the screen
--    was locked horizontal at a power-of-two spacing.  Here the screen
--    coordinate is a fixed-point ACCUMULATOR pair -- P steps by dpx per pixel
--    and dpy per line, Q is P rotated 90 degrees -- where (dpx,dpy) =
--    pitch * (sin,cos) of the angle knob, resolved once per frame.  Lines are
--    the level sets of P, so ANY angle and ANY spacing cost the same two adds
--    per pixel and no multiply.  Q comes free with the rotation and pays for
--    itself twice: it is the crosshatch field AND the waver argument.
--
--  * RELIEF (P12, the headline).  Luma is added to P's phase before the
--    thickness compare, so the rules bow AROUND the image instead of merely
--    thickening: bank-note engraving, Unknown Pleasures.  At 0 the screen is
--    dead flat and you have Penrose's sky verbatim; at max the luma swings the
--    phase through ~4 line periods and the field turns into pure topography.
--
--  * Waver (K5) wobbles the rules along their length by a sine of Q -- the
--    burin wandering in the hand.  Cross (S7) lays a second field down the Q
--    axis, but only into the dark half of the tone ramp, which is how a real
--    engraver builds shadow.  Width (S10) freezes the rules at a constant
--    weight so Relief is the ONLY thing shaping the picture.
--
-- Renderer: streaming, 7 stages, no BRAM, no line buffer.  Three small
-- pixel-rate multiplies (contrast, relief, waver); the per-frame angle resolve
-- shares ONE registered 9x8 multiplier, split-operand across vblank cycles, so
-- no wide combinational product ever lands on a vsync-latched register
-- (the ziffern trap: those paths are still timed).
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture videosky of program_top is

    constant C_LATENCY : integer := 7;
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- quarter-wave folded sine, 64-entry logic ROM (intaglio lineage)
    ----------------------------------------------------------------------
    type t_qsin is array(0 to 63) of signed(9 downto 0);
    constant C_QSIN : t_qsin := (
        to_signed(  0, 10), to_signed( 13, 10), to_signed( 25, 10), to_signed( 38, 10), to_signed( 50, 10), to_signed( 63, 10), to_signed( 75, 10), to_signed( 87, 10),
        to_signed(100, 10), to_signed(112, 10), to_signed(124, 10), to_signed(136, 10), to_signed(148, 10), to_signed(160, 10), to_signed(172, 10), to_signed(184, 10),
        to_signed(196, 10), to_signed(207, 10), to_signed(218, 10), to_signed(230, 10), to_signed(241, 10), to_signed(252, 10), to_signed(263, 10), to_signed(273, 10),
        to_signed(284, 10), to_signed(294, 10), to_signed(304, 10), to_signed(314, 10), to_signed(324, 10), to_signed(334, 10), to_signed(343, 10), to_signed(352, 10),
        to_signed(361, 10), to_signed(370, 10), to_signed(379, 10), to_signed(387, 10), to_signed(395, 10), to_signed(403, 10), to_signed(410, 10), to_signed(418, 10),
        to_signed(425, 10), to_signed(432, 10), to_signed(438, 10), to_signed(445, 10), to_signed(451, 10), to_signed(456, 10), to_signed(462, 10), to_signed(467, 10),
        to_signed(472, 10), to_signed(477, 10), to_signed(481, 10), to_signed(485, 10), to_signed(489, 10), to_signed(492, 10), to_signed(496, 10), to_signed(499, 10),
        to_signed(501, 10), to_signed(503, 10), to_signed(505, 10), to_signed(507, 10), to_signed(509, 10), to_signed(510, 10), to_signed(510, 10), to_signed(511, 10));

    function f_qsin(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then
            v_i := a(7 downto 2);
        else
            v_i := not a(7 downto 2);
        end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then
            return -v_t;
        else
            return v_t;
        end if;
    end function;

    -- Clamp into the legal 10-bit luma swing.  Returns UNSIGNED on purpose:
    -- unsigned(resize(v,10)) of a signed value saturates at the SIGNED range,
    -- so anything >= 512 wraps to black (the penrose lesson).
    function f_cy(v : signed) return unsigned is
    begin
        if    v < 16   then return to_unsigned(16, 10);
        elsif v > 1008 then return to_unsigned(1008, 10);
        else                return resize(unsigned(v(9 downto 0)), 10);
        end if;
    end function;

    -- Clamp a tone into 0..255 (the thickness/phase domain).
    function f_ct(v : signed) return unsigned is
    begin
        if    v < 0   then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else               return resize(unsigned(v(7 downto 0)), 8);
        end if;
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_k1_ink   : unsigned(9 downto 0) := to_unsigned(512, 10);   -- K1 Ink
    signal s_k2_con   : unsigned(9 downto 0) := to_unsigned(256, 10);   -- K2 Contrast
    signal s_k3_pit   : unsigned(9 downto 0) := to_unsigned(400, 10);   -- K3 Pitch
    signal s_k4_ang   : unsigned(9 downto 0) := (others => '0');        -- K4 Angle
    signal s_k5_wav   : unsigned(9 downto 0) := (others => '0');        -- K5 Waver
    signal s_scheme   : unsigned(9 downto 0) := (others => '0');        -- K6 Scheme
    signal s_p12      : unsigned(9 downto 0) := (others => '0');        -- P12 Relief
    signal s_cross    : std_logic := '0';                               -- S7  Cross
    signal s_invert   : std_logic := '0';                               -- S8  Invert
    signal s_tint     : std_logic := '0';                               -- S9  Tint
    signal s_wmod     : std_logic := '1';                               -- S10 Width
    signal s_drift    : std_logic := '0';                               -- S11 Drift

    ----------------------------------------------------------------------
    -- per-frame resolved terms
    ----------------------------------------------------------------------
    signal s_cos8, s_sin8 : signed(7 downto 0) := (others => '0');
    signal s_step   : unsigned(14 downto 0) := to_unsigned(6912, 15);
    signal s_dpx, s_dpy : signed(17 downto 0) := (others => '0');
    signal s_dqx, s_dqy : signed(17 downto 0) := (others => '0');
    signal s_bias   : signed(9 downto 0)  := (others => '0');   -- Ink, +/-256
    signal s_gain   : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s_depth6 : unsigned(5 downto 0) := (others => '0');
    signal s_wav4   : unsigned(3 downto 0) := (others => '0');
    signal s_fixedw : unsigned(7 downto 0) := to_unsigned(64, 8);

    signal s_ink_y, s_ink_u, s_ink_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_pap_y, s_pap_u, s_pap_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- vblank sequencer: one shared, fully registered 9x8 multiplier
    ----------------------------------------------------------------------
    signal s_seq   : unsigned(4 downto 0) := (others => '0');
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');
    signal s_qs_r  : signed(9 downto 0)  := (others => '0');
    signal s_ma    : signed(8 downto 0) := (others => '0');
    signal s_mb    : signed(7 downto 0) := (others => '0');
    signal s_mp    : signed(16 downto 0) := (others => '0');
    signal s_pah_c, s_pal_c : signed(16 downto 0) := (others => '0');
    signal s_pah_s, s_pal_s : signed(16 downto 0) := (others => '0');
    signal s_prod_c, s_prod_s : signed(24 downto 0) := (others => '0');

    signal s_prev_vsync_n : std_logic := '1';
    signal s_vs_pulse : std_logic := '0';
    signal s_fpar  : std_logic := '0';
    signal s_ilace : std_logic := '0';
    signal s_dracc : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- screen-coordinate accumulators.  24 bits with 8 fractional bits; the
    -- phase byte is acc(15 downto 8).  A 24-bit wrap is a multiple of 2^16,
    -- so the phase is continuous across the wrap and the width is free.
    ----------------------------------------------------------------------
    signal s_accp, s_accq : signed(23 downto 0) := (others => '0');
    signal s_linp, s_linq : signed(23 downto 0) := (others => '0');
    signal s_avid_p : std_logic := '0';

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    signal r1_ydiff : signed(10 downto 0) := (others => '0');
    signal r1_gain  : signed(7 downto 0)  := (others => '0');
    signal r1_qsa   : unsigned(9 downto 0) := (others => '0');
    signal r1_p, r1_q : unsigned(7 downto 0) := (others => '0');

    signal r2_yp    : signed(18 downto 0) := (others => '0');
    signal r2_qs    : signed(9 downto 0)  := (others => '0');
    signal r2_p, r2_q : unsigned(7 downto 0) := (others => '0');

    signal r3_y8    : unsigned(7 downto 0) := (others => '0');
    signal r3_qs    : signed(9 downto 0)  := (others => '0');
    signal r3_p, r3_q : unsigned(7 downto 0) := (others => '0');

    signal r4_wob   : signed(11 downto 0) := (others => '0');
    signal r4_rel   : signed(11 downto 0) := (others => '0');
    signal r4_t8    : signed(9 downto 0)  := (others => '0');
    signal r4_p, r4_q : unsigned(7 downto 0) := (others => '0');

    signal r5_ph1, r5_ph2 : unsigned(7 downto 0) := (others => '0');
    signal r5_t     : unsigned(7 downto 0) := (others => '0');

    signal r6_ink   : std_logic := '0';

    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- chroma delay to the colour resolve
    type t_c is array(0 to 5) of unsigned(9 downto 0);
    signal s_ud, s_vd : t_c := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared qsin port: the sequencer takes the cos and sin taps two cycles
    -- after presenting each angle.  Address and ROM output both registered.
    ------------------------------------------------------------------------
    s_qs_a <= s_k4_ang + to_unsigned(256, 10) when s_seq = 31 else s_k4_ang;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_t : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);
            s_mp    <= s_ma * s_mb;

            s_vs_pulse <= '0';

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_vs_pulse <= '1';

                s_k1_ink <= unsigned(registers_in(0));
                s_k2_con <= unsigned(registers_in(1));
                s_k3_pit <= unsigned(registers_in(2));
                s_k4_ang <= unsigned(registers_in(3));
                s_k5_wav <= unsigned(registers_in(4));
                s_scheme <= unsigned(registers_in(5));
                s_p12    <= unsigned(registers_in(7));
                s_cross  <= registers_in(6)(0);      -- S7
                s_invert <= registers_in(6)(1);      -- S8
                s_tint   <= registers_in(6)(2);      -- S9
                s_wmod   <= registers_in(6)(3);      -- S10
                s_drift  <= registers_in(6)(4);      -- S11

                -- interlace detect: did the field parity toggle since last vsync?
                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then
                    s_ilace <= '1';
                else
                    s_ilace <= '0';
                end if;

                if registers_in(6)(4) = '1' then
                    s_dracc <= s_dracc + 2;
                end if;

                s_seq <= to_unsigned(31, 5);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    -- 31/30 present the cos and sin angles into the qsin port.
                    when 30 =>
                        -- Pitch is linear in FREQUENCY: 512 (period 128 px) up to
                        -- 16880 (period ~4 px).  Phase unit 256 = one line period,
                        -- 8 fractional bits.
                        s_step <= to_unsigned(512, 15)
                                  + shift_left(resize(s_k3_pit, 15), 4);

                    when 29 => s_cos8 <= resize(shift_right(s_qs_r, 2), 8);
                    when 28 => s_sin8 <= resize(shift_right(s_qs_r, 2), 8);

                    -- step*cos and step*sin, split-operand through the one
                    -- registered multiplier: step = sh*256 + sl.
                    when 27 => s_ma <= signed('0' & s_step(14 downto 8) & '0');
                               s_mb <= s_cos8;
                    when 26 => s_ma <= signed('0' & s_step(7 downto 0));
                               s_mb <= s_cos8;
                    when 25 => s_pah_c <= s_mp;
                    when 24 => s_pal_c <= s_mp;
                    when 23 => s_ma <= signed('0' & s_step(14 downto 8) & '0');
                               s_mb <= s_sin8;
                    when 22 => s_ma <= signed('0' & s_step(7 downto 0));
                               s_mb <= s_sin8;
                    when 21 => s_pah_s <= s_mp;
                    when 20 => s_pal_s <= s_mp;

                    -- sh was pre-shifted left by 1 above, so its partial lands
                    -- at <<7 not <<8; the two halves then recombine.
                    when 19 =>
                        s_prod_c <= shift_left(resize(s_pah_c, 25), 7)
                                    + resize(s_pal_c, 25);
                        s_prod_s <= shift_left(resize(s_pah_s, 25), 7)
                                    + resize(s_pal_s, 25);

                    -- Angle 0 must give Penrose's HORIZONTAL rules, so cos drives
                    -- the per-LINE step and sin the per-pixel one.
                    when 18 =>
                        s_dpy <= resize(shift_right(s_prod_c, 7), 18);
                        s_dpx <= resize(shift_right(s_prod_s, 7), 18);

                    -- Q is P rotated 90 degrees.
                    when 17 =>
                        s_dqx <= -s_dpy;
                        s_dqy <=  s_dpx;

                    when 16 =>      -- Ink: a bias on the whole tone ramp
                        s_bias <= signed(resize(shift_right(s_k1_ink, 1), 10))
                                  - to_signed(256, 10);

                    when 15 =>      -- Contrast: 1.0x .. 4.9x about mid-grey
                        s_gain <= to_unsigned(16, 7) + resize(s_k2_con(9 downto 4), 7);

                    when 14 => s_depth6 <= s_p12(9 downto 4);
                    when 13 => s_wav4   <= s_k5_wav(9 downto 6);

                    when 12 =>      -- constant rule weight when Width is Fixed
                        v_t := to_signed(64, 12) + resize(shift_right(s_bias, 2), 12);
                        s_fixedw <= f_ct(v_t);

                    when 11 =>      -- print scheme (K6)
                        if s_scheme < 102 then                  -- Parchment
                            s_pap_y <= to_unsigned(740, 10);
                            s_pap_u <= to_unsigned(470, 10); s_pap_v <= to_unsigned(540, 10);
                            s_ink_y <= to_unsigned(30, 10);
                            s_ink_u <= to_unsigned(495, 10); s_ink_v <= to_unsigned(522, 10);
                        elsif s_scheme < 307 then               -- Gallery
                            s_pap_y <= to_unsigned(830, 10);
                            s_pap_u <= C_CHROMA_MID;         s_pap_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(40, 10);
                            s_ink_u <= C_CHROMA_MID;         s_ink_v <= C_CHROMA_MID;
                        elsif s_scheme < 511 then               -- Night
                            s_pap_y <= to_unsigned(120, 10);
                            s_pap_u <= C_CHROMA_MID;         s_pap_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(750, 10);
                            s_ink_u <= C_CHROMA_MID;         s_ink_v <= C_CHROMA_MID;
                        elsif s_scheme < 716 then               -- Blueprint
                            s_pap_y <= to_unsigned(300, 10);
                            s_pap_u <= to_unsigned(700, 10); s_pap_v <= to_unsigned(450, 10);
                            s_ink_y <= to_unsigned(850, 10);
                            s_ink_u <= to_unsigned(500, 10); s_ink_v <= to_unsigned(506, 10);
                        elsif s_scheme < 920 then               -- Cyanotype
                            s_pap_y <= to_unsigned(200, 10);
                            s_pap_u <= to_unsigned(640, 10); s_pap_v <= to_unsigned(470, 10);
                            s_ink_y <= to_unsigned(880, 10);
                            s_ink_u <= to_unsigned(520, 10); s_ink_v <= to_unsigned(505, 10);
                        else                                    -- Copper
                            s_pap_y <= to_unsigned(800, 10);
                            s_pap_u <= to_unsigned(480, 10); s_pap_v <= to_unsigned(530, 10);
                            s_ink_y <= to_unsigned(300, 10);
                            s_ink_u <= to_unsigned(430, 10); s_ink_v <= to_unsigned(600, 10);
                        end if;

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- Screen accumulators.  Origin at the top-left active pixel; Drift walks
    -- the frame origin so the rules crawl.  On an interlaced source each field
    -- starts half a line step down so the two fields interleave instead of
    -- printing the same rules twice.
    ------------------------------------------------------------------------
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            if s_vs_pulse = '1' then
                if s_ilace = '1' and data_in.field_n = '1' then
                    s_linp <= shift_left(resize(signed('0' & s_dracc), 24), 8)
                              + shift_right(resize(s_dpy, 24), 1);
                    s_linq <= shift_right(resize(s_dqy, 24), 1);
                else
                    s_linp <= shift_left(resize(signed('0' & s_dracc), 24), 8);
                    s_linq <= (others => '0');
                end if;
            elsif data_in.avid = '1' and s_avid_p = '0' then
                s_accp <= s_linp;
                s_accq <= s_linq;
                s_linp <= s_linp + resize(s_dpy, 24);
                s_linq <= s_linq + resize(s_dqy, 24);
            elsif data_in.avid = '1' then
                s_accp <= s_accp + resize(s_dpx, 24);
                s_accq <= s_accq + resize(s_dqx, 24);
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- R1: capture.  The contrast multiply's operands are registered here.
    -- The waver argument is Q's HIGH byte pair, so one sine period spans four
    -- line spacings along the rule -- a slow wander, not a buzz.
    ------------------------------------------------------------------------
    p_r1 : process(clk)
    begin
        if rising_edge(clk) then
            r1_ydiff <= signed('0' & unsigned(data_in.y)) - to_signed(512, 11);
            r1_gain  <= signed('0' & s_gain);
            r1_p     <= unsigned(s_accp(15 downto 8));
            r1_q     <= unsigned(s_accq(15 downto 8));
            r1_qsa   <= unsigned(s_accq(17 downto 8));
            s_ud(0)  <= unsigned(data_in.u);
            s_vd(0)  <= unsigned(data_in.v);
            for i in 1 to 5 loop
                s_ud(i) <= s_ud(i - 1);
                s_vd(i) <= s_vd(i - 1);
            end loop;
        end if;
    end process p_r1;

    ------------------------------------------------------------------------
    -- R2: contrast multiply + the sine tap.
    ------------------------------------------------------------------------
    p_r2 : process(clk)
    begin
        if rising_edge(clk) then
            r2_yp <= r1_ydiff * r1_gain;
            r2_qs <= f_qsin(r1_qsa);
            r2_p  <= r1_p;
            r2_q  <= r1_q;
        end if;
    end process p_r2;

    ------------------------------------------------------------------------
    -- R3: clamp the stretched luma back into the legal swing and take the
    -- top 8 bits as the working tone.
    ------------------------------------------------------------------------
    p_r3 : process(clk)
        variable v_y : signed(13 downto 0);
        variable v_c : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_y := to_signed(512, 14) + resize(shift_right(r2_yp, 4), 14);
            v_c := f_cy(v_y);
            r3_y8 <= v_c(9 downto 2);
            r3_qs <= r2_qs;
            r3_p  <= r2_p;
            r3_q  <= r2_q;
        end if;
    end process p_r3;

    ------------------------------------------------------------------------
    -- R4: the two shaping multiplies.
    --   rel = luma * Relief  -- up to ~4 line periods of phase swing
    --   wob = sin(Q) * Waver -- up to ~+/-60, a quarter period of wander
    -- and the raw ink tone: 255 = black = full ink (Penrose's `not luma`).
    ------------------------------------------------------------------------
    p_r4 : process(clk)
    begin
        if rising_edge(clk) then
            r4_rel <= resize(shift_right(signed('0' & r3_y8) * signed('0' & s_depth6), 4), 12);
            r4_wob <= resize(shift_right(r3_qs * signed('0' & s_wav4), 7), 12);
            r4_t8  <= to_signed(255, 10) - signed(resize(r3_y8, 10));
            r4_p   <= r3_p;
            r4_q   <= r3_q;
        end if;
    end process p_r4;

    ------------------------------------------------------------------------
    -- R5: displace both fields and settle the tone.  Only the low 8 bits of
    -- the sum matter -- (phase + wob + rel) mod 256 -- so the 8-bit phase byte
    -- carries all the information the wider accumulator had.
    ------------------------------------------------------------------------
    p_r5 : process(clk)
        variable v_1, v_2 : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v_1 := signed(resize(r4_p, 12)) + r4_wob + r4_rel;
            v_2 := signed(resize(r4_q, 12)) + r4_wob + r4_rel;
            r5_ph1 <= unsigned(v_1(7 downto 0));
            r5_ph2 <= unsigned(v_2(7 downto 0));
            r5_t   <= f_ct(r4_t8 + resize(s_bias, 10));
        end if;
    end process p_r5;

    ------------------------------------------------------------------------
    -- R6: the compare that has been the whole point since Penrose --
    -- phase-within-period < thickness.  Cross lays its field only into the
    -- dark half of the ramp, doubled, the way an engraver builds a shadow.
    ------------------------------------------------------------------------
    p_r6 : process(clk)
        variable v_w1, v_w2 : unsigned(7 downto 0);
        variable v_i1, v_i2 : std_logic;
    begin
        if rising_edge(clk) then
            if s_wmod = '1' then
                v_w1 := r5_t;
            else
                v_w1 := s_fixedw;
            end if;

            if r5_t < 128 then
                v_w2 := (others => '0');
            else
                v_w2 := shift_left(r5_t - 128, 1);
            end if;

            if r5_ph1 < v_w1 then v_i1 := '1'; else v_i1 := '0'; end if;
            if r5_ph2 < v_w2 and s_cross = '1' then v_i2 := '1'; else v_i2 := '0'; end if;

            r6_ink <= v_i1 or v_i2;
        end if;
    end process p_r6;

    ------------------------------------------------------------------------
    -- R7: ink or paper.  Tint hands the ink the source's own chroma, so the
    -- rules print in the colour of whatever they are cutting.
    ------------------------------------------------------------------------
    p_r7 : process(clk)
    begin
        if rising_edge(clk) then
            if (r6_ink xor s_invert) = '1' then
                s_out_y <= s_ink_y;
                if s_tint = '1' then
                    s_out_u <= s_ud(5);
                    s_out_v <= s_vd(5);
                else
                    s_out_u <= s_ink_u;
                    s_out_v <= s_ink_v;
                end if;
            else
                s_out_y <= s_pap_y;
                s_out_u <= s_pap_u;
                s_out_v <= s_pap_v;
            end if;
        end if;
    end process p_r7;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture videosky;
