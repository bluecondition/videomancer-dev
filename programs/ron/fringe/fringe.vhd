-- fringe.vhd
--
-- FRINGE -- a moire interference instrument.  Two layers of transparent
-- black-and-white line-work are composited over a solid field; when their
-- pitches, forms or phases approach one another, large-scale interference
-- fringes bloom across the frame.  The program itself never animates: every
-- movement is the performer's hand, smoothed by glide.
--
-- Controls
--   K1 TEXTURE A   10 zoned textures, snap select (deadbanded, no glide)
--   K2 SCALE A     pattern pitch, 1024 px .. 4 px, continuous + glided
--   K3 PHASE A     X phase, +/-1024 px of translation (rotation for spokes)
--   K4 TEXTURE B / K5 SCALE B / K6 PHASE B  -- same for the second layer
--   P12 Y PHASE    Y phase of whatever layer is currently on TOP -- the
--                  primary gesture: rakes the front pattern across the back
--   S7  INVERT A   / S8 INVERT B   (meaning set by S11)
--   S9  ORDER      swap which layer is on top (also re-targets P12)
--   S10 BACKGROUND black / white
--   S11 INVERT MODE   OFF = mask invert (ink and gaps swap, ink stays black)
--                     ON  = colour flip (mask unchanged, ink goes black<->white)
--
-- ENGINE.  Every texture reduces to a 10-bit PHASE -- 1024 units = one
-- pattern period -- and a comparison.  Two ways of getting there:
--
--   * cartesian textures ride a pair of 20-bit DDA accumulators per layer
--     carrying the scaled coordinate (x-cx-offx)*step in Q8, modulo 2^20.
--     No pixel multiplier at all, and -- the trick that makes it free -- the
--     DDAs are clocked by the avid delayed to the stage that consumes them,
--     so they line up with the CORDIC output with no delay registers.
--   * radial textures (circles, spokes) take r and theta from a per-layer
--     8-iteration vectoring CORDIC, then ONE shared 13x16 multiplier turns
--     either r*step_r (circles) or (theta+rot)*blades (spokes) into the same
--     10-bit phase.  The CORDIC's 1.6468 gain is undone in the per-frame
--     constant step_r, never in the pixel path.
--
-- The DDAs wrap, so they carry no sign -- and none is needed: concentric
-- squares want max(|px|,|py|), and because both axes are scaled by the same
-- positive step, the CORDIC's own |dx| vs |dy| comparison already answers
-- which axis wins.  Sign and winner ride down as three bits, and the
-- magnitude fold is a ten-bit complement.  That removes every wide absolute
-- value from the pixel path.
--
-- Per-frame, ONE 12-iteration shift-add engine sequences all eight products
-- the frame needs -- the 0.7071 diagonal fold, the 0.6094 CORDIC-gain fold
-- and the four DDA start values -- inside a >=17000-cycle vertical blank.
-- Nothing per-frame is left as a combinational cone: a fat one there caps
-- Fmax exactly like a pixel-path one.
--
-- Pipeline (C_LATENCY = 16): acc -> abs -> max/min -> CORDIC x5 -> fold ->
-- operand -> mul x2 -> windows -> phases -> ink -> composite.
--
-- No BRAM, no video in.  Author: bluecondition
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture fringe of program_top is

    constant C_LATENCY : integer := 16;
    constant C_DDA_TAP : integer := 10;   -- avid delay that clocks the DDAs

    ----------------------------------------------------------------------
    -- texture menu
    ----------------------------------------------------------------------
    constant T_CIRCLE : integer := 0;
    constant T_SQUARE : integer := 1;
    constant T_SPOKE  : integer := 2;
    constant T_WAVEH  : integer := 3;
    constant T_WAVEV  : integer := 4;
    constant T_DIAG   : integer := 5;
    constant T_GRID   : integer := 6;
    constant T_HEX    : integer := 7;
    constant T_CHECK  : integer := 8;
    constant T_BRICK  : integer := 9;

    -- ink geometry, in phase units (1024 = one period)
    constant C_DUTY  : integer := 512;    -- grating ink duty, 50%
    constant C_WGRID : integer := 192;    -- square-grid line width (18.75%)
    constant C_WHEXV : integer := 84;     -- honeycomb vertical edge, half-width
    constant C_WHEXD : integer := 190;    -- honeycomb slanted edge (grad sqrt5)
    constant C_WMORT : integer := 128;    -- brick mortar width
    constant C_GSH   : integer := 4;      -- glide one-pole shift (~16 frames)

    constant C_BLACK : unsigned(9 downto 0) := to_unsigned(0, 10);
    constant C_WHITE : unsigned(9 downto 0) := to_unsigned(1023, 10);

    ----------------------------------------------------------------------
    -- quarter-wave sine, half-sample offset so no entry needs the peak:
    --   C_SIN(i) = round(127 * sin(2*pi*(i+0.5)/256))
    ----------------------------------------------------------------------
    type t_sin is array(0 to 63) of signed(7 downto 0);
    constant C_SIN : t_sin := (
        to_signed(2, 8), to_signed(5, 8), to_signed(8, 8), to_signed(11, 8), to_signed(14, 8), to_signed(17, 8), to_signed(20, 8), to_signed(23, 8),
        to_signed(26, 8), to_signed(29, 8), to_signed(32, 8), to_signed(35, 8), to_signed(38, 8), to_signed(41, 8), to_signed(44, 8), to_signed(47, 8),
        to_signed(50, 8), to_signed(53, 8), to_signed(56, 8), to_signed(58, 8), to_signed(61, 8), to_signed(64, 8), to_signed(67, 8), to_signed(69, 8),
        to_signed(72, 8), to_signed(74, 8), to_signed(77, 8), to_signed(79, 8), to_signed(82, 8), to_signed(84, 8), to_signed(86, 8), to_signed(89, 8),
        to_signed(91, 8), to_signed(93, 8), to_signed(95, 8), to_signed(97, 8), to_signed(99, 8), to_signed(101, 8), to_signed(103, 8), to_signed(105, 8),
        to_signed(106, 8), to_signed(108, 8), to_signed(110, 8), to_signed(111, 8), to_signed(113, 8), to_signed(114, 8), to_signed(115, 8), to_signed(117, 8),
        to_signed(118, 8), to_signed(119, 8), to_signed(120, 8), to_signed(121, 8), to_signed(122, 8), to_signed(123, 8), to_signed(124, 8), to_signed(124, 8),
        to_signed(125, 8), to_signed(125, 8), to_signed(126, 8), to_signed(126, 8), to_signed(127, 8), to_signed(127, 8), to_signed(127, 8), to_signed(127, 8)
    );

    function f_sin(a : unsigned(7 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_s : signed(7 downto 0);
    begin
        if a(6) = '0' then v_i := a(5 downto 0);
        else               v_i := not a(5 downto 0); end if;
        v_s := C_SIN(to_integer(v_i));
        if a(7) = '1' then return -v_s; else return v_s; end if;
    end function;

    ----------------------------------------------------------------------
    -- CORDIC: 4 stages x 2 vectoring iterations, 3 guard bits, 4096 = turn.
    -- Worst case is a centre driven a full 1024 px off screen: hypot <=
    -- 2526, K*hypot <= 4160, <<3 guard <= 33280, and the iteration overshoot
    -- stays inside a 17-bit signed datapath.
    ----------------------------------------------------------------------
    constant C_NS : integer := 4;
    constant C_CG : integer := 3;
    type t_atan is array(0 to 7) of integer;
    constant C_ATAN : t_atan := (512, 302, 160, 81, 41, 20, 10, 5);

    -- flat (layer, stage) arrays: index = L*(C_NS+1) + s.  Flat, not an
    -- array-of-arrays -- GHDL's synth backend is happier that way.
    constant C_CW : integer := 2 * (C_NS + 1);
    type t_cord is array(0 to C_CW - 1) of signed(16 downto 0);
    type t_cang is array(0 to C_CW - 1) of signed(12 downto 0);
    type t_cb   is array(0 to C_CW - 1) of std_logic;
    signal cordx, cordy : t_cord := (others => (others => '0'));
    signal cordz        : t_cang := (others => (others => '0'));
    signal cordsw, cdxs, cdys : t_cb := (others => '0');

    ----------------------------------------------------------------------
    -- per-layer types
    ----------------------------------------------------------------------
    type t_l20  is array(0 to 1) of unsigned(19 downto 0);
    type t_l16  is array(0 to 1) of unsigned(15 downto 0);
    type t_l13s is array(0 to 1) of signed(12 downto 0);
    type t_l13  is array(0 to 1) of unsigned(12 downto 0);
    type t_l12  is array(0 to 1) of unsigned(11 downto 0);
    type t_l11  is array(0 to 1) of unsigned(10 downto 0);
    type t_l10  is array(0 to 1) of unsigned(9 downto 0);
    type t_l8s  is array(0 to 1) of signed(7 downto 0);
    type t_l23  is array(0 to 1) of unsigned(22 downto 0);
    type t_l17  is array(0 to 1) of unsigned(16 downto 0);
    type t_l4   is array(0 to 1) of integer range 0 to 9;
    type t_l1   is array(0 to 1) of std_logic;

    ----------------------------------------------------------------------
    -- raster measurement
    ----------------------------------------------------------------------
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_avid_q     : std_logic := '0';
    signal s_xcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_lcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_W          : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H          : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_ilace      : std_logic := '0';
    signal s_fpar       : std_logic := '0';
    signal s_cx, s_cy   : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- controls: raw latch, glide accumulators (Q6), switches
    ----------------------------------------------------------------------
    signal s_ktexa, s_ktexb : unsigned(9 downto 0) := (others => '0');
    type t_g5 is array(0 to 4) of unsigned(15 downto 0);
    type t_v5 is array(0 to 4) of unsigned(9 downto 0);
    --  0 scale A   1 phase A   2 scale B   3 phase B   4 Y phase (slider)
    signal s_gtgt : t_v5 := (to_unsigned(512, 10), to_unsigned(512, 10),
                             to_unsigned(560, 10), to_unsigned(512, 10),
                             to_unsigned(512, 10));
    signal s_gacc : t_g5 := (to_unsigned(32768, 16), to_unsigned(32768, 16),
                             to_unsigned(35840, 16), to_unsigned(32768, 16),
                             to_unsigned(32768, 16));

    signal s_invl  : t_l1 := (others => '0');
    signal s_swap  : std_logic := '0';    -- S9  0 = layer B on top
    signal s_bgw   : std_logic := '1';    -- S10 1 = white background
    signal s_cmode : std_logic := '0';    -- S11 0 = mask invert

    ----------------------------------------------------------------------
    -- per-frame derived terms
    ----------------------------------------------------------------------
    signal s_kq   : t_l10 := (others => (others => '0'));   -- deadbanded knob
    signal s_tex  : t_l4  := (others => 0);
    signal s_sb   : t_l16 := (others => to_unsigned(4096, 16));  -- base step Q8
    signal s_step : t_l16 := (others => to_unsigned(4096, 16));  -- final step Q8
    signal s_mulb : t_l16 := (others => (others => '0'));   -- multiplier operand
    signal s_n64  : t_l16 := (others => (others => '0'));   -- blades * 64
    signal s_rot  : t_l12 := (others => (others => '0'));   -- spoke rotation
    signal s_offx, s_offy : t_l13s := (others => (others => '0'));
    signal s_mx, s_my     : t_l13s := (others => (others => '0'));
    signal s_ndx, s_ndy   : t_l13s := (others => (others => '0'));
    signal s_startx, s_starty : t_l20 := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- vblank sequencer + the one shift-add engine that computes every
    -- per-frame product: 0/1 the 0.7071 diagonal fold, 2/3 the 0.6094
    -- CORDIC-gain fold, 4..7 the four DDA start values.
    ----------------------------------------------------------------------
    signal s_ph    : integer range 0 to 2 := 0;
    signal s_seq   : integer range 0 to 7 := 0;
    signal s_midx  : integer range 0 to 7 := 0;
    signal s_mbit  : integer range 0 to 13 := 0;
    signal s_mcand : unsigned(23 downto 0) := (others => '0');
    signal s_mplr  : unsigned(11 downto 0) := (others => '0');
    signal s_macc  : unsigned(23 downto 0) := (others => '0');
    signal s_mneg  : std_logic := '0';

    ----------------------------------------------------------------------
    -- pixel-domain coordinate accumulators (CORDIC feed)
    ----------------------------------------------------------------------
    signal s_avid_p   : std_logic := '0';
    signal s_newframe : std_logic := '1';
    signal s_dx       : t_l13s := (others => (others => '0'));
    signal s_dyc      : t_l13s := (others => (others => '0'));
    signal s_dyb      : t_l13s := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- phase-domain DDAs (cartesian textures), clocked at C_DDA_TAP
    ----------------------------------------------------------------------
    signal s_atap     : std_logic := '0';
    signal s_nf_dda   : std_logic := '1';
    signal s_px, s_py : t_l20 := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- CORDIC front end / tail
    ----------------------------------------------------------------------
    signal r1_a, r1_b : t_l12 := (others => (others => '0'));
    signal r1_xs, r1_ys : t_l1 := (others => '0');
    signal r2_hi, r2_lo : t_l12 := (others => (others => '0'));
    signal r2_sw, r2_xs, r2_ys : t_l1 := (others => '0');

    signal ru_r   : t_l13 := (others => (others => '0'));
    signal ru_ang : t_l13s := (others => (others => '0'));
    signal op_a   : t_l13 := (others => (others => '0'));
    signal op_b   : t_l16 := (others => (others => '0'));
    signal pp0    : t_l23 := (others => (others => '0'));
    signal pp1    : t_l17 := (others => (others => '0'));
    signal rphase : t_l10 := (others => (others => '0'));

    -- the three geometry bits the square metric needs, walked down to c13
    type t_sr is array(0 to 4) of std_logic;
    type t_sr2 is array(0 to 1) of t_sr;
    signal d_sw, d_xs, d_ys : t_sr2 := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- window / phase / ink stages
    ----------------------------------------------------------------------
    signal w_fx, w_fy   : t_l12 := (others => (others => '0'));
    signal w_lox, w_loy : t_l1  := (others => '0');   -- low byte all-zero flags
    signal w_dg         : t_l10 := (others => (others => '0'));
    signal w_base       : t_l10 := (others => (others => '0'));
    signal w_sin        : t_l8s := (others => (others => '0'));
    signal w_rp         : t_l10 := (others => (others => '0'));

    signal q_str  : t_l10 := (others => (others => '0'));
    signal q_phx  : t_l10 := (others => (others => '0'));
    signal q_phy  : t_l10 := (others => (others => '0'));
    signal q_hexa : t_l10 := (others => (others => '0'));
    signal q_hexb : t_l10 := (others => (others => '0'));
    signal q_brk  : t_l11 := (others => (others => '0'));

    signal s_mask : t_l1 := (others => '0');

    ----------------------------------------------------------------------
    -- output
    ----------------------------------------------------------------------
    signal o_y : unsigned(9 downto 0) := (others => '0');

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement (interlace-aware)
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                if s_xcnt > 16 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
                s_lcnt <= s_lcnt + 1;
            end if;

            if s_vs_pulse = '1' then
                if s_lcnt > 8 then
                    if s_ilace = '1' then s_H <= shift_left(s_lcnt, 1);
                    else                  s_H <= s_lcnt; end if;
                end if;
                s_lcnt <= (others => '0');
                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then s_ilace <= '1';
                else                              s_ilace <= '0'; end if;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- CONTROLS + per-frame sequencer.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_tg  : signed(17 downto 0);
        variable v_ac  : signed(17 downto 0);
        variable v_nx  : signed(17 downto 0);
        variable v_k   : unsigned(9 downto 0);
        variable v_e   : integer range 0 to 7;
        variable v_m   : unsigned(15 downto 0);
        variable v_tx  : integer range 0 to 9;
        variable v_ph  : signed(12 downto 0);
        variable v_sl  : signed(12 downto 0);
        variable v_r2  : signed(12 downto 0);
        variable v_top : integer range 0 to 1;
        variable v_mv  : signed(12 downto 0);
        variable v_mc  : unsigned(15 downto 0);
        variable v_ng  : unsigned(23 downto 0);
        variable v_L   : integer range 0 to 1;
    begin
        if rising_edge(clk) then
            if s_vs_pulse = '1' then
                -- glide targets: scale A, phase A, scale B, phase B, slider
                s_gtgt(0) <= unsigned(registers_in(1));
                s_gtgt(1) <= unsigned(registers_in(2));
                s_gtgt(2) <= unsigned(registers_in(4));
                s_gtgt(3) <= unsigned(registers_in(5));
                s_gtgt(4) <= unsigned(registers_in(7));
                -- one-pole slew toward the target latched LAST frame; a frame
                -- of extra lag is nothing against a 16-frame time constant,
                -- and it keeps the panel read off this cycle's adder path.
                for i in 0 to 4 loop
                    v_tg := shift_left(signed(resize(s_gtgt(i), 18)), 6);
                    v_ac := signed(resize(s_gacc(i), 18));
                    v_nx := v_ac + shift_right(v_tg - v_ac, C_GSH);
                    s_gacc(i) <= unsigned(std_logic_vector(v_nx(15 downto 0)));
                end loop;

                s_ktexa   <= unsigned(registers_in(0));
                s_ktexb   <= unsigned(registers_in(3));
                s_invl(0) <= registers_in(6)(0);      -- S7
                s_invl(1) <= registers_in(6)(1);      -- S8
                s_swap    <= registers_in(6)(2);      -- S9
                s_bgw     <= registers_in(6)(3);      -- S10
                s_cmode   <= registers_in(6)(4);      -- S11

                s_cx <= '0' & s_W(11 downto 1);
                s_cy <= '0' & s_H(11 downto 1);

                s_ph  <= 1;
                s_seq <= 6;

            elsif s_ph = 1 and data_in.avid = '0' then
                case s_seq is

                    when 6 =>
                        -- TEXTURE select: an 8-count deadband keeps converter
                        -- dither from flickering a zone boundary.  No glide --
                        -- detents must snap.
                        for L in 0 to 1 loop
                            if L = 0 then v_k := s_ktexa; else v_k := s_ktexb; end if;
                            if (v_k > s_kq(L) and v_k - s_kq(L) > 8) or
                               (v_k < s_kq(L) and s_kq(L) - v_k > 8) then
                                s_kq(L) <= v_k;
                            end if;
                        end loop;

                    when 5 =>
                        -- ten equal zones: index = kq * 10 / 1024
                        for L in 0 to 1 loop
                            v_tx := to_integer(shift_right(
                                        shift_left(resize(s_kq(L), 14), 3)
                                      + shift_left(resize(s_kq(L), 14), 1), 10));
                            if v_tx > 9 then v_tx := 9; end if;
                            s_tex(L) <= v_tx;
                        end loop;

                    when 4 =>
                        -- SCALE: mantissa/exponent split of the glided knob.
                        -- step = (256 + 2m) << e in Q8 phase units per pixel,
                        -- i.e. a period of 1024 px (knob 0) down to 4.0 px
                        -- (knob 1023) -- continuous, monotonic, no dead zone.
                        for L in 0 to 1 loop
                            v_k := s_gacc(2 * L)(15 downto 6);
                            v_e := to_integer(v_k(9 downto 7));
                            v_m := shift_left(resize(v_k(6 downto 0), 16), 1);
                            s_sb(L) <= shift_left(v_m + to_unsigned(256, 16), v_e);
                            -- spokes: 4..63 blades off the same knob (x60/1024)
                            s_n64(L) <= shift_left(
                                shift_right(shift_left(resize(v_k, 16), 6)
                                          - shift_left(resize(v_k, 16), 2), 10)
                                + to_unsigned(4, 16), 6);
                        end loop;

                    when 3 =>
                        -- PHASE.  X phase translates the layer +/-1024 px; the
                        -- slider does the same in Y but only for the TOP layer.
                        -- Spokes keep their centre and rotate instead: X winds
                        -- the fan a full turn, the slider counter-winds it.
                        if s_swap = '0' then v_top := 1; else v_top := 0; end if;
                        for L in 0 to 1 loop
                            v_ph := shift_left(signed(resize(s_gacc(2 * L + 1)(15 downto 6), 13))
                                               - to_signed(512, 13), 1);
                            v_sl := shift_left(signed(resize(s_gacc(4)(15 downto 6), 13))
                                               - to_signed(512, 13), 1);
                            if s_tex(L) = T_SPOKE then
                                s_offx(L) <= (others => '0');
                                s_offy(L) <= (others => '0');
                                if L = v_top then v_r2 := shift_left(v_ph, 1) - v_sl;
                                else              v_r2 := shift_left(v_ph, 1); end if;
                                s_rot(L) <= unsigned(std_logic_vector(v_r2(11 downto 0)));
                            else
                                s_offx(L) <= v_ph;
                                if L = v_top then s_offy(L) <= v_sl;
                                else              s_offy(L) <= (others => '0'); end if;
                                s_rot(L) <= (others => '0');
                            end if;
                        end loop;

                    when 2 =>
                        for L in 0 to 1 loop
                            s_mx(L) <= signed(resize(s_cx, 13)) + s_offx(L);
                            s_my(L) <= signed(resize(s_cy, 13)) + s_offy(L);
                        end loop;

                    when others =>
                        for L in 0 to 1 loop
                            s_ndx(L) <= -(signed(resize(s_cx, 13)) + s_offx(L));
                            s_ndy(L) <= -(signed(resize(s_cy, 13)) + s_offy(L));
                        end loop;
                        s_midx <= 0;
                        s_mbit <= 0;
                        s_ph   <= 2;
                end case;
                if s_seq > 0 then s_seq <= s_seq - 1; end if;

            elsif s_ph = 2 and data_in.avid = '0' then
                if s_midx < 2 then v_L := s_midx;
                elsif s_midx < 4 then v_L := s_midx - 2;
                else                  v_L := (s_midx - 4) / 2; end if;

                if s_mbit = 0 then
                    -- load the multiplier / multiplicand for this product
                    case s_midx is
                        when 0 | 1 =>              -- step * 181 / 256  (1/sqrt2)
                            s_mplr <= to_unsigned(181, 12);
                            s_mcand <= resize(s_sb(v_L), 24);
                            s_mneg <= '0';
                        when 2 | 3 =>              -- step * 39 / 64  (1/1.6468)
                            s_mplr <= to_unsigned(39, 12);
                            s_mcand <= resize(s_step(v_L), 24);
                            s_mneg <= '0';
                        when others =>             -- -(centre + offset) * step
                            if (s_midx mod 2) = 0 then v_mv := s_mx(v_L);
                            else                       v_mv := s_my(v_L); end if;
                            v_mc := s_step(v_L);
                            if v_mv < 0 then
                                s_mplr <= unsigned(std_logic_vector(resize(-v_mv, 12)));
                                s_mneg <= '1';
                            else
                                s_mplr <= unsigned(std_logic_vector(resize(v_mv, 12)));
                                s_mneg <= '0';
                            end if;
                            s_mcand <= resize(v_mc, 24);
                    end case;
                    s_macc <= (others => '0');
                    s_mbit <= 1;

                elsif s_mbit < 13 then
                    if s_mplr(0) = '1' then s_macc <= s_macc + s_mcand; end if;
                    s_mcand <= shift_left(s_mcand, 1);
                    s_mplr  <= shift_right(s_mplr, 1);
                    s_mbit  <= s_mbit + 1;

                else
                    case s_midx is
                        when 0 | 1 =>
                            -- 45-degree textures run both axes at 1/sqrt(2) so
                            -- the pitch ACROSS the stripes matches the menu
                            if s_tex(v_L) = T_DIAG then
                                s_step(v_L) <= resize(shift_right(s_macc, 8), 16);
                            else
                                s_step(v_L) <= s_sb(v_L);
                            end if;
                        when 2 | 3 =>
                            if s_tex(v_L) = T_CIRCLE then
                                s_mulb(v_L) <= resize(shift_right(s_macc, 6), 16);
                            else
                                s_mulb(v_L) <= s_n64(v_L);
                            end if;
                        when others =>
                            -- start = -(centre + offset) * step, mod 2^20
                            if s_mneg = '0' then v_ng := unsigned(-signed(s_macc));
                            else                 v_ng := s_macc; end if;
                            if (s_midx mod 2) = 0 then
                                s_startx(v_L) <= v_ng(19 downto 0);
                            else
                                s_starty(v_L) <= v_ng(19 downto 0);
                            end if;
                    end case;
                    s_mbit <= 0;
                    if s_midx = 7 then s_ph <= 0;
                    else               s_midx <= s_midx + 1; end if;
                end if;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- pixel-domain coordinates for the CORDIC: dx = x - cx - offx per pixel,
    -- dy = y - cy - offy per active line (+2 per line on interlaced sources,
    -- with the odd field offset by one).
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;
            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif data_in.avid = '1' and s_avid_p = '0' then
                for L in 0 to 1 loop
                    if s_newframe = '1' then
                        if s_ilace = '1' and data_in.field_n = '1' then
                            v_dy0 := s_ndy(L) + 1;
                        else
                            v_dy0 := s_ndy(L);
                        end if;
                        s_dyc(L) <= v_dy0;
                        if s_ilace = '1' then s_dyb(L) <= v_dy0 + 2;
                        else                  s_dyb(L) <= v_dy0 + 1; end if;
                    else
                        s_dyc(L) <= s_dyb(L);
                        if s_ilace = '1' then s_dyb(L) <= s_dyb(L) + 2;
                        else                  s_dyb(L) <= s_dyb(L) + 1; end if;
                    end if;
                    s_dx(L) <= s_ndx(L);
                end loop;
                s_newframe <= '0';
            elsif data_in.avid = '1' then
                for L in 0 to 1 loop
                    s_dx(L) <= s_dx(L) + 1;
                end loop;
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- PHASE-DOMAIN DDAs, modulo 2^20.  Clocked by the avid delayed to
    -- C_DDA_TAP so they present the right pixel to the window stage with no
    -- delay registers of their own.
    ------------------------------------------------------------------------
    p_dda : process(clk)
        variable v_st : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_atap <= s_avid_sr(C_DDA_TAP);
            if s_vs_pulse = '1' then
                s_nf_dda <= '1';
            end if;
            if s_avid_sr(C_DDA_TAP) = '1' and s_atap = '0' then
                for L in 0 to 1 loop
                    v_st := resize(s_step(L), 20);
                    s_px(L) <= s_startx(L);
                    if s_nf_dda = '1' then
                        if s_ilace = '1' and s_fpar = '1' then
                            s_py(L) <= s_starty(L) + v_st;
                        else
                            s_py(L) <= s_starty(L);
                        end if;
                    elsif s_ilace = '1' then
                        s_py(L) <= s_py(L) + shift_left(v_st, 1);
                    else
                        s_py(L) <= s_py(L) + v_st;
                    end if;
                end loop;
                s_nf_dda <= '0';
            elsif s_avid_sr(C_DDA_TAP) = '1' then
                for L in 0 to 1 loop
                    s_px(L) <= s_px(L) + resize(s_step(L), 20);
                end loop;
            end if;
        end if;
    end process p_dda;

    ------------------------------------------------------------------------
    -- c1 absolute values, c2 max / min + octant flags
    ------------------------------------------------------------------------
    p_cord_in : process(clk)
    begin
        if rising_edge(clk) then
            for L in 0 to 1 loop
                if s_dx(L)(12) = '1' then
                    r1_a(L) <= unsigned(std_logic_vector(resize(-s_dx(L), 12)));
                else
                    r1_a(L) <= unsigned(std_logic_vector(resize(s_dx(L), 12)));
                end if;
                if s_dyc(L)(12) = '1' then
                    r1_b(L) <= unsigned(std_logic_vector(resize(-s_dyc(L), 12)));
                else
                    r1_b(L) <= unsigned(std_logic_vector(resize(s_dyc(L), 12)));
                end if;
                r1_xs(L) <= s_dx(L)(12);
                r1_ys(L) <= s_dyc(L)(12);

                if r1_a(L) >= r1_b(L) then
                    r2_hi(L) <= r1_a(L); r2_lo(L) <= r1_b(L); r2_sw(L) <= '0';
                else
                    r2_hi(L) <= r1_b(L); r2_lo(L) <= r1_a(L); r2_sw(L) <= '1';
                end if;
                r2_xs(L) <= r1_xs(L);
                r2_ys(L) <= r1_ys(L);
            end loop;
        end if;
    end process p_cord_in;

    ------------------------------------------------------------------------
    -- CORDIC vectoring: C_NS stages of two iterations each.  cordx converges
    -- to K*hypot (K ~ 1.6468, undone per-frame in s_mulb); cordz accumulates
    -- the first-octant angle.
    ------------------------------------------------------------------------
    p_cordic : process(clk)
        variable x0, y0 : signed(16 downto 0);
        variable z0     : signed(12 downto 0);
        variable b      : integer;
    begin
        if rising_edge(clk) then
            for L in 0 to 1 loop
                b := L * (C_NS + 1);
                cordx (b) <= shift_left(signed(resize(r2_hi(L), 17)), C_CG);
                cordy (b) <= shift_left(signed(resize(r2_lo(L), 17)), C_CG);
                cordz (b) <= (others => '0');
                cordsw(b) <= r2_sw(L);
                cdxs  (b) <= r2_xs(L);
                cdys  (b) <= r2_ys(L);

                for s in 0 to C_NS - 1 loop
                    if cordy(b + s) >= 0 then
                        x0 := cordx(b + s) + shift_right(cordy(b + s), 2 * s);
                        y0 := cordy(b + s) - shift_right(cordx(b + s), 2 * s);
                        z0 := cordz(b + s) + to_signed(C_ATAN(2 * s), 13);
                    else
                        x0 := cordx(b + s) - shift_right(cordy(b + s), 2 * s);
                        y0 := cordy(b + s) + shift_right(cordx(b + s), 2 * s);
                        z0 := cordz(b + s) - to_signed(C_ATAN(2 * s), 13);
                    end if;
                    if y0 >= 0 then
                        cordx(b + s + 1) <= x0 + shift_right(y0, 2 * s + 1);
                        cordy(b + s + 1) <= y0 - shift_right(x0, 2 * s + 1);
                        cordz(b + s + 1) <= z0 + to_signed(C_ATAN(2 * s + 1), 13);
                    else
                        cordx(b + s + 1) <= x0 - shift_right(y0, 2 * s + 1);
                        cordy(b + s + 1) <= y0 + shift_right(x0, 2 * s + 1);
                        cordz(b + s + 1) <= z0 - to_signed(C_ATAN(2 * s + 1), 13);
                    end if;
                    cordsw(b + s + 1) <= cordsw(b + s);
                    cdxs  (b + s + 1) <= cdxs  (b + s);
                    cdys  (b + s + 1) <= cdys  (b + s);
                end loop;
            end loop;
        end if;
    end process p_cordic;

    ------------------------------------------------------------------------
    -- RU: strip the guard bits off the magnitude; fold the octant angle out
    -- to a full 0..4095 turn using the swap flag and the original signs.
    -- The same three bits start their walk down to the square-ring stage.
    ------------------------------------------------------------------------
    p_ru : process(clk)
        variable v_m : signed(12 downto 0);
        variable v_r : signed(16 downto 0);
        variable b   : integer;
    begin
        if rising_edge(clk) then
            for L in 0 to 1 loop
                b := L * (C_NS + 1) + C_NS;
                v_r := shift_right(cordx(b), C_CG);
                ru_r(L) <= unsigned(std_logic_vector(v_r(12 downto 0)));
                if cordsw(b) = '1' then v_m := to_signed(1024, 13) - cordz(b);
                else                    v_m := cordz(b); end if;
                if    cdxs(b) = '0' and cdys(b) = '0' then ru_ang(L) <= v_m;
                elsif cdxs(b) = '1' and cdys(b) = '0' then ru_ang(L) <= to_signed(2048, 13) - v_m;
                elsif cdxs(b) = '1' and cdys(b) = '1' then ru_ang(L) <= to_signed(2048, 13) + v_m;
                else                                       ru_ang(L) <= -v_m; end if;

                d_sw(L)(0) <= cordsw(b);
                d_xs(L)(0) <= cdxs(b);
                d_ys(L)(0) <= cdys(b);
                for i in 1 to 4 loop
                    d_sw(L)(i) <= d_sw(L)(i - 1);
                    d_xs(L)(i) <= d_xs(L)(i - 1);
                    d_ys(L)(i) <= d_ys(L)(i - 1);
                end loop;
            end loop;
        end if;
    end process p_ru;

    ------------------------------------------------------------------------
    -- one shared 13x16 multiplier per layer: circles take r * step_r, spokes
    -- take (theta + rotation) * blades.  Only the low 18 bits of the product
    -- can reach the phase window, so both partial products are cut to size.
    ------------------------------------------------------------------------
    p_mul : process(clk)
        variable v_a : unsigned(12 downto 0);
        variable v_s : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            for L in 0 to 1 loop
                if s_tex(L) = T_SPOKE then
                    v_a := resize(unsigned(std_logic_vector(ru_ang(L)(11 downto 0)))
                                  + s_rot(L), 13);
                else
                    v_a := ru_r(L);
                end if;
                op_a(L) <= v_a;
                op_b(L) <= s_mulb(L);

                pp0(L) <= op_a(L)(6 downto 0) * op_b(L);
                pp1(L) <= op_a(L)(12 downto 7) * op_b(L)(10 downto 0);

                v_s := resize(pp0(L), 24) + shift_left(resize(pp1(L), 24), 7);
                rphase(L) <= v_s(17 downto 8);
            end loop;
        end if;
    end process p_mul;

    ------------------------------------------------------------------------
    -- c12 WINDOWS: slice the DDAs into phase windows, fold the diagonal sum,
    -- read the sine.  The sine index and the coordinate it displaces are
    -- both chosen by the per-frame texture, so one table read per layer
    -- covers both wavies.  The low-byte zero flags are the borrow bits the
    -- square-ring complement needs one stage later.
    ------------------------------------------------------------------------
    p_win : process(clk)
        variable v_i : unsigned(7 downto 0);
        variable v_s : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            for L in 0 to 1 loop
                w_fx(L) <= s_px(L)(19 downto 8);
                w_fy(L) <= s_py(L)(19 downto 8);
                if s_px(L)(7 downto 0) = 0 then w_lox(L) <= '1';
                else                            w_lox(L) <= '0'; end if;
                if s_py(L)(7 downto 0) = 0 then w_loy(L) <= '1';
                else                            w_loy(L) <= '0'; end if;

                v_s := resize(s_px(L)(17 downto 0), 19)
                     + resize(s_py(L)(17 downto 0), 19);
                w_dg(L) <= v_s(17 downto 8);

                -- wavy H is y displaced by sin(x); wavy V is the transpose
                if s_tex(L) = T_WAVEH then
                    v_i       := s_px(L)(19 downto 12);
                    w_base(L) <= s_py(L)(17 downto 8);
                else
                    v_i       := s_py(L)(19 downto 12);
                    w_base(L) <= s_px(L)(17 downto 8);
                end if;
                w_sin(L) <= f_sin(v_i);

                w_rp(L) <= rphase(L);
            end loop;
        end if;
    end process p_win;

    ------------------------------------------------------------------------
    -- c13 PHASES: resolve the one stripe phase this layer's texture needs,
    -- plus the cell-space intermediates the tiled textures want.
    ------------------------------------------------------------------------
    p_ph : process(clk)
        variable v_w  : signed(12 downto 0);
        variable v_sq : unsigned(9 downto 0);
        variable v_pm : unsigned(9 downto 0);
        variable v_sg : std_logic;
        variable v_lo : std_logic;
        variable v_u  : signed(11 downto 0);
        variable v_v  : signed(11 downto 0);
        variable v_au : signed(11 downto 0);
        variable v_av : signed(11 downto 0);
        variable v_r  : std_logic;
        variable v_ux : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            for L in 0 to 1 loop
                q_phx(L) <= w_fx(L)(9 downto 0);
                q_phy(L) <= w_fy(L)(9 downto 0);

                -- CONCENTRIC SQUARES.  max(|px|,|py|) is the square-ring
                -- metric, and since both axes carry the same positive step
                -- the CORDIC's |dx| vs |dy| verdict already picked the
                -- winner.  All that is left is folding the magnitude of a
                -- wrapped accumulator: a ten-bit complement plus the borrow
                -- out of the byte below.
                if d_sw(L)(4) = '0' then
                    v_pm := w_fx(L)(9 downto 0); v_sg := d_xs(L)(4); v_lo := w_lox(L);
                else
                    v_pm := w_fy(L)(9 downto 0); v_sg := d_ys(L)(4); v_lo := w_loy(L);
                end if;
                if v_sg = '1' then
                    if v_lo = '1' then v_sq := (not v_pm) + 1;
                    else               v_sq := not v_pm; end if;
                else
                    v_sq := v_pm;
                end if;

                -- sine displacement: +/-254 phase units, one wave per 4
                -- periods.  Only the top ten bits of the base are carried,
                -- so the sine is added there and the low byte only supplies
                -- its carry.
                v_w := signed(resize(w_base(L), 13))
                     + shift_left(resize(w_sin(L), 13), 1);

                case s_tex(L) is
                    when T_CIRCLE => q_str(L) <= w_rp(L);
                    when T_SPOKE  => q_str(L) <= w_rp(L);
                    when T_SQUARE => q_str(L) <= v_sq;
                    when T_DIAG   => q_str(L) <= w_dg(L);
                    when others   => q_str(L) <= unsigned(std_logic_vector(
                                                    v_w(9 downto 0)));
                end case;

                -- honeycomb: the hex Voronoi of a half-offset (brick) point
                -- lattice.  Cells come out 1024 wide x 1280 tall in phase
                -- units -- within 8% of regular -- and both edge families
                -- are plain linear bisector tests.  No squares, no divides.
                v_r := w_fy(L)(10);
                if v_r = '1' then v_ux := w_fx(L)(9 downto 0) + 512;
                else              v_ux := w_fx(L)(9 downto 0); end if;
                v_u := signed(resize(v_ux, 12)) - to_signed(512, 12);
                v_v := signed(resize(w_fy(L)(9 downto 0), 12)) - to_signed(512, 12);
                -- |u| and |v| reach 512, which a SIGNED resize to 10 bits would
                -- fold to 256 -- so negate at full width and slice.  Missing
                -- that put a wrong-ink line through the cell edge once per
                -- period, wherever the phase landed exactly on zero.
                if v_u < 0 then v_au := -v_u; else v_au := v_u; end if;
                if v_v < 0 then v_av := -v_v; else v_av := v_v; end if;
                q_hexa(L) <= unsigned(std_logic_vector(v_au(9 downto 0)));
                q_hexb(L) <= unsigned(std_logic_vector(v_av(9 downto 0)));

                -- running bond: courses are two periods wide, half-offset
                if v_r = '1' then q_brk(L) <= w_fx(L)(10 downto 0) + 1024;
                else              q_brk(L) <= w_fx(L)(10 downto 0); end if;
            end loop;
        end if;
    end process p_ph;

    ------------------------------------------------------------------------
    -- c14 INK: the tiled textures' masks plus the shared stripe comparison,
    -- muxed by the frame's texture index.
    ------------------------------------------------------------------------
    p_ink : process(clk)
        variable v_d  : signed(12 downto 0);
        variable v_ad : unsigned(11 downto 0);
        variable v_av : unsigned(9 downto 0);
        variable v_m  : std_logic;
    begin
        if rising_edge(clk) then
            for L in 0 to 1 loop
                -- honeycomb edge tests: |u| = 512 is the vertical bisector,
                -- |u| + 2|v| = 1280 the slanted one.
                v_d := signed(resize(q_hexa(L), 13))
                     + shift_left(signed(resize(q_hexb(L), 13)), 1)
                     - to_signed(1280, 13);
                if v_d < 0 then v_ad := unsigned(std_logic_vector(resize(-v_d, 12)));
                else            v_ad := unsigned(std_logic_vector(resize( v_d, 12))); end if;
                if q_hexa(L) < 512 then v_av := 512 - q_hexa(L);
                else                    v_av := q_hexa(L) - 512; end if;

                case s_tex(L) is
                    when T_GRID =>
                        if q_phx(L) < C_WGRID or q_phy(L) < C_WGRID then
                            v_m := '1'; else v_m := '0'; end if;
                    when T_HEX =>
                        if (v_av < C_WHEXV and v_d < 0) or v_ad < C_WHEXD then
                            v_m := '1'; else v_m := '0'; end if;
                    when T_CHECK =>
                        v_m := q_phx(L)(9) xor q_phy(L)(9);
                    when T_BRICK =>
                        if q_phy(L) < C_WMORT or q_brk(L) < C_WMORT then
                            v_m := '1'; else v_m := '0'; end if;
                    when others =>
                        if q_str(L) < C_DUTY then v_m := '1'; else v_m := '0'; end if;
                end case;
                s_mask(L) <= v_m;
            end loop;
        end if;
    end process p_ink;

    ------------------------------------------------------------------------
    -- c15 COMPOSITE: background, back layer, front layer.  Transparency is
    -- binary -- the moire is optical, never a blend.
    --   mask invert (S11 off): ink and gaps swap, the ink stays black
    --   colour flip (S11 on):  the mask is untouched, the ink goes white
    ------------------------------------------------------------------------
    p_comp : process(clk)
        variable v_mk : t_l1;
        variable v_co : t_l10;
        variable v_t  : integer range 0 to 1;
        variable v_b  : integer range 0 to 1;
        variable v_y  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            for L in 0 to 1 loop
                if s_cmode = '0' then
                    v_mk(L) := s_mask(L) xor s_invl(L);
                    v_co(L) := C_BLACK;
                else
                    v_mk(L) := s_mask(L);
                    if s_invl(L) = '1' then v_co(L) := C_WHITE;
                    else                    v_co(L) := C_BLACK; end if;
                end if;
            end loop;

            if s_swap = '0' then v_t := 1; v_b := 0;    -- layer B on top
            else                 v_t := 0; v_b := 1; end if;

            if s_bgw = '1' then v_y := C_WHITE; else v_y := C_BLACK; end if;
            if v_mk(v_b) = '1' then v_y := v_co(v_b); end if;
            if v_mk(v_t) = '1' then v_y := v_co(v_t); end if;
            o_y <= v_y;
        end if;
    end process p_comp;

    ------------------------------------------------------------------------
    -- sync delay + output.  Pure luma: chroma sits at neutral, so there is
    -- no U/V swap to worry about.
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

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture fringe;
