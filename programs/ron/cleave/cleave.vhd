-- cleave.vhd  (v0.4 — "Cleave": dark/light split re-texturizer)
--
-- Full-screen two-texture split:
--
--   DARK  -> thermal scan-bands through a 16-entry UNEQUAL palette: white
--            and black bars exactly THREE TIMES the width of every color
--            stripe (whites straddle the cycle wrap at 15/0/1, blacks at
--            8/9/10, ten single-width colors between).  Bowed by a
--            triangle wobble, displaced by a high-frequency per-column
--            random serration, bowed by live luma relief, and fuzzed
--            per-pixel by the static field.
--   LIGHT -> TV STATIC: per-pixel (or 1/2/4/8 px chunk) random black/white,
--            density thresholded by inverted luma so the content reads
--            through the noise; boils per frame by default; optional
--            rainbow fringe tints it with the local band hue.
--   OUTLINE: black contour wherever the dark/light class flips —
--            horizontal via a class history, vertical via a {class,age}
--            ping-pong line buffer.  S9 selects Thin / Thick.
--
-- S7 Patchwork is a FULL EFFECT when on — every 128px patch gets: a large
-- vertical band-phase displacement (sync-tear tiles), a palette rotation
-- (bar positions shift per patch), band frequency doubling (~25%), a
-- strong static density swing (+/-64), per-patch static POLARITY
-- inversion (negative-static patches), and a serration boost.  Off =
-- genuinely uniform textures; the band rhythm still pulses the static
-- density (stripe-shaped, not a grid).
--
-- S8 "Contrast" applies a x2 centered luma stretch BEFORE posterization,
-- sharpening the class split, the band relief and the static density —
-- the incoming image defines the whole picture harder.
--
-- P12 "Flip" rotates the posterized class wheel: class = (level*128 +
-- wobble/4 + dither + flip) mod 1024, dark when < 512.  Mid-travel bands
-- the luma extremes and statics the midtones; full travel is the inversion.
--
-- Controls:
--   K1 Band Size
--   K2 Wave (STAGED: grows the chevron, then stretches it — frequency
--      halves while amplitude climbs, full-swing 512 px swells at the end;
--      serration strength rides the same knob)
--   K3 Drift (BIDIRECTIONAL: centre stopped, left up, right down)
--   K4 Static Size K5 Ink                          K6 Fringe
--   S7 Patchwork (Off/On: 128px patches double band frequency)
--   S8 Contrast (Normal/Boost)   S9 Outline (Thin/Thick)
--   S10 Bands (Horizontal/Vertical)
--   S11 Grain (Still/Boil — STATIC LAYER ONLY: the band serration, band
--       fuzz and class boundary run on fixed seeds and never shimmer)
--   P12 Flip
--
-- The band fuzz and class-boundary dither run on a dedicated ALWAYS-1px
-- hash, decoupled from K4's static cell size — reusing the static cell
-- hash there put visible square "cubes" into the band layer.
--
-- 4 EBR (class+age line buffer), 0 multipliers.  Generated colors stored
-- U/V-swapped (stored U = Cr) per the HW-verified palette convention.
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

architecture cleave of program_top is

    constant LATENCY : natural := 12;

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- 16-entry thermal cycle (10-bit YUV, stored U = Cr / V = Cb, already
    -- U/V-swapped; luma capped at 768).  White bars = slots 15+0+1, black
    -- bars = slots 8+9+10 (each exactly 3x a color stripe); colors 1 slot.
    -- Order per the user's reference strip: white -> (faint green) -> blue
    -- -> magenta -> purple -> indigo (darkest beside black) -> BLACK ->
    -- dark red -> red -> orange -> yellow -> (faint green-white) -> white.
    type t_pal16 is array (0 to 15) of unsigned(9 downto 0);
    constant PAL_Y : t_pal16 := (
        to_unsigned(768,10),   -- 0  white
        to_unsigned(768,10),   -- 1  white
        to_unsigned(730,10),   -- 2  pale mint (faint green hint)
        to_unsigned(538,10),   -- 3  light blue
        to_unsigned(164,10),   -- 4  blue
        to_unsigned(426,10),   -- 5  magenta
        to_unsigned(349,10),   -- 6  purple
        to_unsigned(192,10),   -- 7  indigo (darkest magenta, beside black)
        to_unsigned( 64,10),   -- 8  black
        to_unsigned( 64,10),   -- 9  black
        to_unsigned( 64,10),   -- 10 black
        to_unsigned(208,10),   -- 11 dark red
        to_unsigned(328,10),   -- 12 red
        to_unsigned(584,10),   -- 13 orange
        to_unsigned(768,10),   -- 14 yellow
        to_unsigned(768,10));  -- 15 near-white (faint green-yellow)
    constant PAL_U : t_pal16 := (
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(424,10),
        to_unsigned(342,10), to_unsigned(440,10), to_unsigned(887,10),
        to_unsigned(756,10), to_unsigned(608,10), to_unsigned(512,10),
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(758,10),
        to_unsigned(960,10), to_unsigned(772,10), to_unsigned(584,10),
        to_unsigned(480,10));
    constant PAL_V : t_pal16 := (
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(508,10),
        to_unsigned(744,10), to_unsigned(960,10), to_unsigned(809,10),
        to_unsigned(852,10), to_unsigned(696,10), to_unsigned(512,10),
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(429,10),
        to_unsigned(360,10), to_unsigned(212,10), to_unsigned( 64,10),
        to_unsigned(448,10));

    constant PSEED : unsigned(9 downto 0) := "0101010101";  -- patch seed

    --------------------------------------------------------------------------
    -- Raster position / accumulators.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal px          : unsigned(11 downto 0) := (others => '0');
    signal py          : unsigned(10 downto 0) := (others => '0');
    signal frame_act   : std_logic := '0';
    signal frame_armed : std_logic := '0';
    signal line_act    : std_logic := '0';

    signal lfsr   : unsigned(15 downto 0) := x"ACE1";
    signal seed10 : unsigned(9 downto 0) := "0110100101";

    signal bph_v : unsigned(9 downto 0) := (others => '0');
    signal bph_h : unsigned(9 downto 0) := (others => '0');
    signal wob_v : unsigned(9 downto 0) := (others => '0');
    signal wob_h : unsigned(9 downto 0) := (others => '0');

    signal scroll_acc : unsigned(15 downto 0) := (others => '0');
    signal wobt       : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Free-running control decode.
    --------------------------------------------------------------------------
    signal rk1, rk2, rk3, rk4, rk5, rk6, rp12 : unsigned(9 downto 0) := (others => '0');
    signal rsw : std_logic_vector(4 downto 0) := (others => '0');

    signal s_vstep  : unsigned(6 downto 0) := to_unsigned(14, 7);
    signal k2g      : unsigned(9 downto 0) := to_unsigned(560, 10);
    signal s_wstep  : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal s_wamp   : integer range 0 to 6 := 3;
    signal s_wfrac  : unsigned(1 downto 0) := "00";
    signal s_jbase  : unsigned(1 downto 0) := "10";
    signal rd3s     : signed(10 downto 0) := (others => '0');
    signal s_spd    : unsigned(11 downto 0) := (others => '0');
    signal s_dneg   : std_logic := '0';
    signal s_gsel   : unsigned(1 downto 0) := "01";
    signal s_dens   : unsigned(6 downto 0) := (others => '0');
    signal s_frsel  : unsigned(2 downto 0) := (others => '0');
    signal s_flip   : unsigned(9 downto 0) := (others => '0');

    signal sw_patch, sw_boost, sw_thick, sw_vert, sw_boil : std_logic := '0';

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe.
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Class+age line buffer: {class, age(3)} ping-pong by line parity.
    --------------------------------------------------------------------------
    type t_lb is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal lb_a, lb_b   : t_lb := (others => (others => '0'));
    signal rd_a, rd_b   : std_logic_vector(3 downto 0) := (others => '0');
    signal rda_f, rdb_f : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Datapath stage registers.
    --------------------------------------------------------------------------
    -- s1: capture
    signal r1_l    : unsigned(9 downto 0) := (others => '0');
    signal r1_bph  : unsigned(9 downto 0) := (others => '0');
    signal r1_warg : unsigned(9 downto 0) := (others => '0');
    signal r1_jarg : unsigned(9 downto 0) := (others => '0');
    signal r1_cx, r1_cy : unsigned(9 downto 0) := (others => '0');
    signal r1_ppx  : unsigned(9 downto 0) := (others => '0');
    signal r1_ppy  : unsigned(9 downto 0) := (others => '0');
    signal r1_pyl  : unsigned(9 downto 0) := (others => '0');
    signal r1_px   : unsigned(11 downto 0) := (others => '0');
    signal r1_par  : std_logic := '0';
    signal r1_pok  : std_logic := '0';
    -- s2: contrast boost, posterize, tri fold, base hash products
    signal r2_tri  : signed(9 downto 0) := (others => '0');
    signal r2_lev  : unsigned(2 downto 0) := (others => '0');
    signal r2_l    : unsigned(9 downto 0) := (others => '0');
    signal r2_bph  : unsigned(9 downto 0) := (others => '0');
    signal r2_ha, r2_hb : unsigned(9 downto 0) := (others => '0');
    signal r2_qa, r2_qb : unsigned(9 downto 0) := (others => '0');
    signal r2_jh   : unsigned(9 downto 0) := (others => '0');
    signal r2_pa, r2_pb : unsigned(9 downto 0) := (others => '0');
    signal r2_px   : unsigned(11 downto 0) := (others => '0');
    signal r2_par, r2_pok : std_logic := '0';
    -- s3: wobble sel, hash combine, 1D/patch rounds, relief
    signal r3_wob  : signed(9 downto 0) := (others => '0');
    signal r3_h1   : unsigned(9 downto 0) := (others => '0');
    signal r3_q1   : unsigned(9 downto 0) := (others => '0');
    signal r3_jh2  : unsigned(9 downto 0) := (others => '0');
    signal r3_p1   : unsigned(9 downto 0) := (others => '0');
    signal r3_rel  : unsigned(7 downto 0) := (others => '0');
    signal r3_lev  : unsigned(2 downto 0) := (others => '0');
    signal r3_l    : unsigned(9 downto 0) := (others => '0');
    signal r3_bph  : unsigned(9 downto 0) := (others => '0');
    signal r3_px   : unsigned(11 downto 0) := (others => '0');
    signal r3_par, r3_pok : std_logic := '0';
    -- s4: hash round B, band+wobble, jitter value, patch round, class base
    signal r4_h1b  : unsigned(9 downto 0) := (others => '0');
    signal r4_q1b  : unsigned(9 downto 0) := (others => '0');
    signal r4_bph  : unsigned(9 downto 0) := (others => '0');
    signal r4_jsv  : signed(10 downto 0) := (others => '0');
    signal r4_p2   : unsigned(9 downto 0) := (others => '0');
    signal r4_c1   : signed(11 downto 0) := (others => '0');
    signal r4_rel  : unsigned(7 downto 0) := (others => '0');
    signal r4_l    : unsigned(9 downto 0) := (others => '0');
    signal r4_px   : unsigned(11 downto 0) := (others => '0');
    signal r4_par, r4_pok : std_logic := '0';
    -- s5: hash round C, band+relief, patch final, class+dither
    signal r5_h2   : unsigned(9 downto 0) := (others => '0');
    signal r5_q2   : unsigned(9 downto 0) := (others => '0');
    signal r5_bph  : unsigned(9 downto 0) := (others => '0');
    signal r5_pf   : unsigned(9 downto 0) := (others => '0');
    signal r5_c2   : signed(11 downto 0) := (others => '0');
    signal r5_jsv  : signed(10 downto 0) := (others => '0');
    signal r5_l    : unsigned(9 downto 0) := (others => '0');
    signal r5_px   : unsigned(11 downto 0) := (others => '0');
    signal r5_par, r5_pok : std_logic := '0';
    -- s6: static value final, scaled jitter, class+flip
    signal r6_hn   : unsigned(9 downto 0) := (others => '0');
    signal r6_qn   : unsigned(9 downto 0) := (others => '0');
    signal r6_bph  : unsigned(9 downto 0) := (others => '0');
    signal r6_jit  : signed(9 downto 0) := (others => '0');
    signal r6_pf   : unsigned(9 downto 0) := (others => '0');
    signal r6_class: unsigned(9 downto 0) := (others => '0');
    signal r6_l    : unsigned(9 downto 0) := (others => '0');
    signal r6_px   : unsigned(11 downto 0) := (others => '0');
    signal r6_par, r6_pok : std_logic := '0';
    -- s7: band + serration, dark bit + h-edge, base ink threshold
    signal r7_hn   : unsigned(9 downto 0) := (others => '0');
    signal r7_qn   : unsigned(9 downto 0) := (others => '0');
    signal r7_bph  : unsigned(9 downto 0) := (others => '0');
    signal r7_dark : std_logic := '0';
    signal r7_hedge: std_logic := '0';
    signal r7_thr  : unsigned(7 downto 0) := (others => '0');
    signal r7_pf   : unsigned(9 downto 0) := (others => '0');
    signal r7_px   : unsigned(11 downto 0) := (others => '0');
    signal r7_par  : std_logic := '0';
    signal d_hist  : std_logic_vector(5 downto 0) := (others => '0');
    -- s8: static-fuzzed band phase, contour resolve
    signal r8_hn   : unsigned(9 downto 0) := (others => '0');
    signal r8_bph  : unsigned(9 downto 0) := (others => '0');
    signal r8_dark : std_logic := '0';
    signal r8_ctr  : std_logic := '0';
    signal r8_age  : unsigned(2 downto 0) := (others => '0');
    signal r8_thr  : unsigned(7 downto 0) := (others => '0');
    signal r8_pf   : unsigned(9 downto 0) := (others => '0');
    signal r8_px   : unsigned(11 downto 0) := (others => '0');
    signal r8_par  : std_logic := '0';
    -- s9: palette index, patch density mod, band pulse
    signal r9_hn   : unsigned(9 downto 0) := (others => '0');
    signal r9_idx  : unsigned(3 downto 0) := (others => '0');
    signal r9_thr  : unsigned(7 downto 0) := (others => '0');
    signal r9_bp   : signed(9 downto 0) := (others => '0');
    signal r9_dark, r9_ctr, r9_pol : std_logic := '0';
    -- s10: palette lookup, pulsed threshold
    signal r10_hn  : unsigned(9 downto 0) := (others => '0');
    signal r10_py  : unsigned(9 downto 0) := C_BLKY;
    signal r10_du, r10_dv : signed(9 downto 0) := (others => '0');
    signal r10_thr : unsigned(7 downto 0) := (others => '0');
    signal r10_dark, r10_ctr, r10_pol : std_logic := '0';
    -- s11: fringe shift
    signal r11_hn  : unsigned(9 downto 0) := (others => '0');
    signal r11_py  : unsigned(9 downto 0) := C_BLKY;
    signal r11_du, r11_dv : signed(9 downto 0) := (others => '0');
    signal r11_fu, r11_fv : signed(9 downto 0) := (others => '0');
    signal r11_thr : unsigned(7 downto 0) := (others => '0');
    signal r11_dark, r11_ctr, r11_pol : std_logic := '0';
    -- s12: ink decision, light luma, blanking flag
    signal r12_py  : unsigned(9 downto 0) := C_BLKY;
    signal r12_du, r12_dv : signed(9 downto 0) := (others => '0');
    signal r12_fu, r12_fv : signed(9 downto 0) := (others => '0');
    signal r12_ly  : unsigned(9 downto 0) := (others => '0');
    signal r12_dark, r12_ctr : std_logic := '0';
    signal r12_blank : std_logic := '1';

    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Free-running control decode.
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_m12 : signed(11 downto 0);
        variable v_mag : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            rk1  <= unsigned(registers_in(0)(9 downto 0));
            rk2  <= unsigned(registers_in(1)(9 downto 0));
            rk3  <= unsigned(registers_in(2)(9 downto 0));
            rk4  <= unsigned(registers_in(3)(9 downto 0));
            rk5  <= unsigned(registers_in(4)(9 downto 0));
            rk6  <= unsigned(registers_in(5)(9 downto 0));
            rsw  <= registers_in(6)(4 downto 0);
            rp12 <= unsigned(registers_in(7)(9 downto 0));

            s_vstep  <= to_unsigned(3, 7) + resize(rk1(9 downto 4), 7);

            -- K2 staged wave, decoded from the GLIDED value k2g: base amp
            -- shift from the top 3 bits with a 2-bit mantissa (25% sub-
            -- steps), wavelength constant 64 px in the lower half and a
            -- CONTINUOUS unit-step stretch (step 16 down to 2) in the top
            -- half — every change is accumulator-phase-continuous.
            case to_integer(k2g(9 downto 7)) is
                when 0      => s_wamp <= 6;
                when 1      => s_wamp <= 5;
                when 2      => s_wamp <= 4;
                when 3      => s_wamp <= 3;
                when 4      => s_wamp <= 2;
                when 5      => s_wamp <= 1;
                when 6      => s_wamp <= 1;
                when others => s_wamp <= 0;
            end case;
            s_wfrac <= k2g(6 downto 5);
            if k2g(9) = '0' then
                s_wstep <= to_unsigned(16, 5);
            elsif k2g(8 downto 5) >= 14 then
                s_wstep <= to_unsigned(2, 5);
            else
                s_wstep <= to_unsigned(16, 5) - resize(k2g(8 downto 5), 5);
            end if;
            s_jbase  <= k2g(9 downto 8);

            -- K3 bidirectional drift: centre = stopped (16-count deadband),
            -- right = down, left = up, speed grows toward the extremes.
            rd3s <= signed(resize(rk3, 11)) - to_signed(512, 11);
            v_m12 := abs(resize(rd3s, 12));
            v_mag := unsigned(v_m12(10 downto 0));
            s_dneg <= rd3s(10);
            if v_mag <= 16 then
                s_spd <= (others => '0');
            else
                s_spd <= resize(shift_left(v_mag - 16, 2), 12);
            end if;

            s_gsel   <= rk4(9 downto 8);
            s_dens   <= rk5(9 downto 3);
            s_frsel  <= rk6(9 downto 7);
            -- (rp12+1)/2 reaches 512 at full travel = the EXACT class
            -- complement (511 left the blackest level unflipped, flickering
            -- on the boundary at slider max)
            s_flip   <= resize(shift_right(resize(rp12, 11) + 1, 1), 10);

            sw_patch <= rsw(0);   -- S7  Patchwork (freq-doubling) Off/On
            sw_boost <= rsw(1);   -- S8  Contrast Normal/Boost
            sw_thick <= rsw(2);   -- S9  Outline Thin/Thick
            sw_vert  <= rsw(3);   -- S10
            sw_boil  <= rsw(4);   -- S11
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Raster counters, accumulators, per-frame housekeeping.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_kd, v_ks, v_kn : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- free-running 16-bit Galois LFSR
            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

            -- frame anchor, guarded against serrated analog vsync
            if v_v_edge = '1' and frame_armed = '1' then
                frame_armed <= '0';
                frame_act   <= '0';
                py          <= (others => '0');
                bph_v       <= scroll_acc(14 downto 5);
                wob_v       <= wobt;
                if s_dneg = '1' then
                    scroll_acc <= scroll_acc - resize(s_spd, 16);
                else
                    scroll_acc <= scroll_acc + resize(s_spd, 16);
                end if;
                wobt <= wobt + 2;

                -- K2 glide: ease toward the pot by diff/8 per frame, with
                -- a +/-1 nudge so it always reaches the target (no snap)
                v_kd := signed(resize(rk2, 12)) - signed(resize(k2g, 12));
                v_ks := shift_right(v_kd, 3);
                if v_ks = 0 and v_kd > 0 then
                    v_ks := to_signed(1, 12);
                elsif v_ks = 0 and v_kd < 0 then
                    v_ks := to_signed(-1, 12);
                end if;
                v_kn := signed(resize(k2g, 12)) + v_ks;
                k2g  <= unsigned(v_kn(9 downto 0));

                if sw_boil = '1' then
                    seed10 <= lfsr(9 downto 0);
                else
                    seed10 <= "0110100101";
                end if;
            elsif v_h_edge = '1' then
                px    <= (others => '0');
                bph_h <= scroll_acc(14 downto 5);
                wob_h <= wobt;
                if line_act = '1' then
                    line_act <= '0';
                    py       <= py + 1;
                    bph_v    <= bph_v + resize(s_vstep, 10);
                    wob_v    <= wob_v + resize(s_wstep, 10);
                end if;
            elsif data_in.avid = '1' then
                frame_armed <= '1';
                line_act    <= '1';
                px    <= px + 1;
                bph_h <= bph_h + resize(s_vstep, 10);
                wob_h <= wob_h + resize(s_wstep, 10);
                if frame_act = '0' then
                    frame_act <= '1';
                    py        <= (others => '0');
                end if;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Main datapath.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_lsub : signed(11 downto 0);
        variable v_lb   : signed(11 downto 0);
        variable v_t9   : unsigned(8 downto 0);
        variable v_wob  : signed(9 downto 0);
        variable v_sum  : signed(11 downto 0);
        variable v_jf   : unsigned(9 downto 0);
        variable v_hr   : unsigned(9 downto 0);
        variable v_p3   : unsigned(9 downto 0);
        variable v_dith : signed(11 downto 0);
        variable v_cls  : signed(11 downto 0);
        variable v_jsel : unsigned(2 downto 0);
        variable v_dark : std_logic;
        variable v_wrp  : signed(11 downto 0);
        variable v_prvc : std_logic;
        variable v_agep : unsigned(2 downto 0);
        variable v_edge : std_logic;
        variable v_bt9  : unsigned(8 downto 0);
        variable v_thrx : signed(9 downto 0);
        variable v_idx4 : unsigned(3 downto 0);
        variable v_fu   : signed(9 downto 0);
        variable v_fv   : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -----------------------------------------------------------------
            -- s1: capture — luma clamp, phase/wobble/jitter args, static
            -- chunk coords, patch coords.
            -----------------------------------------------------------------
            v_lsub := signed(resize(unsigned(data_in.y), 12)) - 64;
            if v_lsub < 0 then
                r1_l <= (others => '0');
            else
                r1_l <= unsigned(v_lsub(9 downto 0));
            end if;

            if sw_vert = '1' then
                r1_bph  <= bph_h;
                r1_warg <= wob_v;
                r1_jarg <= resize(py, 10);
            else
                r1_bph  <= bph_v;
                r1_warg <= wob_h;
                r1_jarg <= px(9 downto 0);
            end if;

            case to_integer(s_gsel) is
                when 0 =>      -- 1 px static
                    r1_cx <= px(9 downto 0);
                    r1_cy <= resize(py(9 downto 0), 10);
                when 1 =>      -- 2 px
                    r1_cx <= px(10 downto 1);
                    r1_cy <= resize(py(10 downto 1), 10);
                when 2 =>      -- 4 px
                    r1_cx <= px(11 downto 2);
                    r1_cy <= resize(py(10 downto 2), 10);
                when others => -- 8 px
                    r1_cx <= resize(px(11 downto 3), 10);
                    r1_cy <= resize(py(10 downto 3), 10);
            end case;

            r1_ppx <= resize(px(11 downto 7), 10);
            r1_ppy <= resize(py(10 downto 7), 10);
            r1_pyl <= resize(py(9 downto 0), 10);
            r1_px  <= px;
            r1_par <= py(0);
            if px >= 8 and py /= 0 then r1_pok <= '1'; else r1_pok <= '0'; end if;

            -----------------------------------------------------------------
            -- s2: contrast boost (x2 centered stretch), posterize, triangle
            -- fold, base hash products.
            -----------------------------------------------------------------
            if sw_boost = '1' then
                v_lb := shift_left(signed(resize(r1_l, 12)), 1)
                      - to_signed(512, 12);
                if v_lb < 0 then
                    r2_l   <= (others => '0');
                    r2_lev <= (others => '0');
                elsif v_lb > 1023 then
                    r2_l   <= (others => '1');
                    r2_lev <= (others => '1');
                else
                    r2_l   <= unsigned(v_lb(9 downto 0));
                    r2_lev <= unsigned(v_lb(9 downto 7));
                end if;
            else
                r2_l   <= r1_l;
                r2_lev <= r1_l(9 downto 7);
            end if;

            v_t9 := r1_warg(8 downto 0);
            if r1_warg(9) = '1' then v_t9 := not v_t9; end if;
            r2_tri <= signed(resize(v_t9, 10)) - to_signed(256, 10);

            r2_bph <= r1_bph;

            r2_ha <= r1_cx + shift_left(r1_cx, 3);                  -- cx*9
            r2_hb <= r1_cy + shift_left(r1_cy, 2);                  -- cy*5
            -- dedicated ALWAYS-1px hash (band fuzz + class dither): keeps
            -- those free of K4's static cell size (no cube artifacts).
            -- FIXED seeds here and on the serration — only the static
            -- layer's hash boils (S11 must never shimmer the bands).
            r2_qa <= r1_px(9 downto 0) + shift_left(r1_px(9 downto 0), 3);
            r2_qb <= r1_pyl + shift_left(r1_pyl, 2);
            r2_jh <= (r1_jarg + shift_left(r1_jarg, 3))
                   xor "0011010110";                                -- 1D col
            r2_pa <= r1_ppx + shift_left(r1_ppx, 3);                -- patch
            r2_pb <= r1_ppy + shift_left(r1_ppy, 2);

            r2_px <= r1_px; r2_par <= r1_par; r2_pok <= r1_pok;

            -----------------------------------------------------------------
            -- s3: wobble amp (case), hash combine + seed, 1D/patch rounds.
            -----------------------------------------------------------------
            case s_wamp is
                when 6      => v_wob := shift_right(r2_tri, 6);
                when 5      => v_wob := shift_right(r2_tri, 5);
                when 4      => v_wob := shift_right(r2_tri, 4);
                when 3      => v_wob := shift_right(r2_tri, 3);
                when 2      => v_wob := shift_right(r2_tri, 2);
                when 1      => v_wob := shift_right(r2_tri, 1);
                when others => v_wob := r2_tri;
            end case;
            -- 2-bit mantissa: x1.0 / x1.25 / x1.5 / x1.75 between stages
            if s_wfrac(1) = '1' then
                v_wob := v_wob + shift_right(v_wob, 1);
            end if;
            if s_wfrac(0) = '1' then
                v_wob := v_wob + shift_right(v_wob, 2);
            end if;
            r3_wob <= v_wob;

            r3_h1  <= (r2_ha xor rotate_left(r2_hb, 5)) xor seed10;
            r3_q1  <= (r2_qa xor rotate_left(r2_qb, 5)) xor "1010011100";
            r3_jh2 <= (r2_jh + shift_left(r2_jh, 2)) + to_unsigned(341, 10);
            r3_p1  <= (r2_pa xor rotate_left(r2_pb, 5)) xor PSEED;
            r3_rel <= r2_l(9 downto 2);
            r3_lev <= r2_lev;
            r3_l   <= r2_l;
            r3_bph <= r2_bph;
            r3_px <= r2_px; r3_par <= r2_par; r3_pok <= r2_pok;

            -----------------------------------------------------------------
            -- s4: hash round B, band+wobble, jitter value, patch round B,
            -- class base.
            -----------------------------------------------------------------
            v_hr := r3_h1 + shift_left(r3_h1, 2);
            r4_h1b <= v_hr xor shift_right(v_hr, 4);
            v_hr := r3_q1 + shift_left(r3_q1, 2);
            r4_q1b <= v_hr xor shift_right(v_hr, 4);

            v_sum  := signed(resize(r3_bph, 12)) + resize(r3_wob, 12);
            r4_bph <= unsigned(v_sum(9 downto 0));

            v_jf   := r3_jh2 xor shift_right(r3_jh2, 4);
            r4_jsv <= signed(resize(v_jf, 11)) - to_signed(512, 11);

            r4_p2  <= (r3_p1 + shift_left(r3_p1, 2))
                    xor shift_right(r3_p1 + shift_left(r3_p1, 2), 4);

            r4_c1  <= shift_left(signed(resize(r3_lev, 12)), 7)
                    + resize(shift_right(r3_wob, 2), 12);

            r4_rel <= r3_rel;
            r4_l   <= r3_l;
            r4_px <= r3_px; r4_par <= r3_par; r4_pok <= r3_pok;

            -----------------------------------------------------------------
            -- s5: hash round C, band+relief, patch final, class + dither.
            -----------------------------------------------------------------
            r5_h2  <= (r4_h1b + shift_left(r4_h1b, 2)) + to_unsigned(341, 10);
            r5_q2  <= (r4_q1b + shift_left(r4_q1b, 2)) + to_unsigned(341, 10);

            r5_bph <= r4_bph + resize(r4_rel, 10);

            v_p3  := (r4_p2 + shift_left(r4_p2, 2)) + to_unsigned(341, 10);
            r5_pf <= v_p3 xor shift_right(v_p3, 4);

            -- class dither from the 1px hash (K4-independent boundary)
            v_dith := shift_left(signed(resize(r4_q1b(3 downto 0), 12)), 2)
                    - to_signed(32, 12);
            r5_c2  <= r4_c1 + v_dith;

            r5_jsv <= r4_jsv;
            r5_l   <= r4_l;
            r5_px <= r4_px; r5_par <= r4_par; r5_pok <= r4_pok;

            -----------------------------------------------------------------
            -- s6: final static value, serration jitter (K2 + patch scaled),
            -- class + flip.  Also issue the class line-buffer read.
            -----------------------------------------------------------------
            r6_hn <= r5_h2 xor shift_right(r5_h2, 4);
            r6_qn <= r5_q2 xor shift_right(r5_q2, 4);

            -- per-patch serration boost only when Patchwork is on
            if sw_patch = '1' then
                v_jsel := resize(s_jbase, 3)
                        + resize(r5_pf(1 downto 0), 3);
            else
                v_jsel := resize(s_jbase, 3) + 1;
            end if;
            case to_integer(v_jsel) is
                when 0      => r6_jit <= resize(shift_right(r5_jsv, 7), 10);
                when 1      => r6_jit <= resize(shift_right(r5_jsv, 6), 10);
                when 2      => r6_jit <= resize(shift_right(r5_jsv, 5), 10);
                when 3      => r6_jit <= resize(shift_right(r5_jsv, 4), 10);
                when 4      => r6_jit <= resize(shift_right(r5_jsv, 3), 10);
                when others => r6_jit <= resize(shift_right(r5_jsv, 2), 10);
            end case;

            v_cls    := r5_c2 + signed(resize(s_flip, 12));
            r6_class <= unsigned(v_cls(9 downto 0));

            -- Patchwork: large per-patch band-phase displacement (tiles of
            -- bands tear vertically like lost sync)
            if sw_patch = '1' then
                r6_bph <= r5_bph + shift_left(resize(r5_pf(7 downto 4), 10), 4);
            else
                r6_bph <= r5_bph;
            end if;
            r6_pf  <= r5_pf;
            r6_l   <= r5_l;
            r6_px <= r5_px; r6_par <= r5_par; r6_pok <= r5_pok;

            -- class line-buffer read (both banks, unconditional)
            rd_a <= lb_a(to_integer(r5_px(10 downto 0)));
            rd_b <= lb_b(to_integer(r5_px(10 downto 0)));

            -----------------------------------------------------------------
            -- s7: band + serration, dark bit + h-edge, base ink threshold;
            -- fabric re-register of LB reads.
            -----------------------------------------------------------------
            v_sum  := signed(resize(r6_bph, 12)) + resize(r6_jit, 12);
            r7_bph <= unsigned(v_sum(9 downto 0));

            v_dark  := not r6_class(9);
            r7_dark <= v_dark;
            d_hist  <= d_hist(4 downto 0) & v_dark;
            if sw_thick = '1' then
                if (v_dark xor d_hist(0)) = '1' or (v_dark xor d_hist(1)) = '1'
                   or (v_dark xor d_hist(2)) = '1' or (v_dark xor d_hist(3)) = '1'
                   or (v_dark xor d_hist(4)) = '1' or (v_dark xor d_hist(5)) = '1' then
                    r7_hedge <= r6_pok;
                else
                    r7_hedge <= '0';
                end if;
            else
                if (v_dark xor d_hist(0)) = '1' or (v_dark xor d_hist(1)) = '1' then
                    r7_hedge <= r6_pok;
                else
                    r7_hedge <= '0';
                end if;
            end if;

            -- ink threshold: density bias + inverted luma (content term)
            r7_thr <= resize(s_dens, 8) + resize(not r6_l(9 downto 3), 8);

            rda_f <= rd_a;
            rdb_f <= rd_b;

            r7_hn <= r6_hn;
            r7_qn <= r6_qn;
            r7_pf <= r6_pf;
            r7_px <= r6_px; r7_par <= r6_par;

            -----------------------------------------------------------------
            -- s8: static fuzz on the band phase (cross-mod static->bands),
            -- contour resolve (h-edge + v-edge + age).
            -----------------------------------------------------------------
            -- per-pixel band fuzz from the 1px hash (never cell-blocky)
            v_wrp  := shift_right(signed(resize(r7_qn, 12))
                                  - to_signed(512, 12), 5);        -- +/-16
            r8_bph <= r7_bph + unsigned(v_wrp(9 downto 0));

            if r7_par = '0' then
                v_prvc := rdb_f(3);
                v_agep := unsigned(rdb_f(2 downto 0));
            else
                v_prvc := rda_f(3);
                v_agep := unsigned(rda_f(2 downto 0));
            end if;
            v_edge := r7_hedge or (v_prvc xor r7_dark);
            if v_edge = '1' or v_agep /= 0 then
                r8_ctr <= '1';
            else
                r8_ctr <= '0';
            end if;
            if v_edge = '1' then
                if sw_thick = '1' then r8_age <= "101";
                else                   r8_age <= "001";
                end if;
            elsif v_agep /= 0 then
                r8_age <= v_agep - 1;
            else
                r8_age <= (others => '0');
            end if;

            r8_hn   <= r7_hn;
            r8_dark <= r7_dark;
            r8_thr  <= r7_thr;
            r8_pf   <= r7_pf;
            r8_px <= r7_px; r8_par <= r7_par;

            -----------------------------------------------------------------
            -- s9: palette index (patch frequency doubling), patch density
            -- mod, band pulse (cross-mod bands->static).  LB write.
            -----------------------------------------------------------------
            -- S7 Patchwork: ~25% of patches at double band freq, plus a
            -- per-patch palette rotation (bars shift position per patch)
            if sw_patch = '1' and r8_pf(3 downto 2) = "11" then
                v_idx4 := r8_bph(8 downto 5);
            else
                v_idx4 := r8_bph(9 downto 6);
            end if;
            if sw_patch = '1' then
                v_idx4 := v_idx4 + resize(r8_pf(1 downto 0), 4);
            end if;
            r9_idx <= v_idx4;

            -- per-patch static density swing (strong) when Patchwork is on
            if sw_patch = '1' then
                case to_integer(r8_pf(5 downto 4)) is
                    when 0      => v_thrx := signed(resize(r8_thr, 10)) - 64;
                    when 1      => v_thrx := signed(resize(r8_thr, 10)) - 24;
                    when 2      => v_thrx := signed(resize(r8_thr, 10)) + 24;
                    when others => v_thrx := signed(resize(r8_thr, 10)) + 64;
                end case;
            else
                v_thrx := signed(resize(r8_thr, 10));
            end if;
            -- per-patch static polarity: some patches render negative
            r9_pol <= sw_patch and r8_pf(6);
            if v_thrx < 0 then
                r9_thr <= (others => '0');
            elsif v_thrx > 255 then
                r9_thr <= (others => '1');
            else
                r9_thr <= unsigned(v_thrx(7 downto 0));
            end if;

            v_bt9 := r8_bph(8 downto 0);
            if r8_bph(9) = '1' then v_bt9 := not v_bt9; end if;
            r9_bp <= resize(shift_right(signed(resize(v_bt9, 10))
                                        - to_signed(256, 10), 4), 10);

            r9_hn   <= r8_hn;
            r9_dark <= r8_dark;
            r9_ctr  <= r8_ctr;

            -- line-buffer write: single write per bank, aligned-avid gated
            if pipe(7).avid = '1' then
                if r8_par = '0' then
                    lb_a(to_integer(r8_px(10 downto 0)))
                        <= r8_dark & std_logic_vector(r8_age);
                else
                    lb_b(to_integer(r8_px(10 downto 0)))
                        <= r8_dark & std_logic_vector(r8_age);
                end if;
            end if;

            -----------------------------------------------------------------
            -- s10: palette lookup, pulsed threshold.
            -----------------------------------------------------------------
            r10_py <= PAL_Y(to_integer(r9_idx));
            r10_du <= signed(PAL_U(to_integer(r9_idx)) xor "1000000000");
            r10_dv <= signed(PAL_V(to_integer(r9_idx)) xor "1000000000");

            v_thrx := signed(resize(r9_thr, 10)) + r9_bp;
            if v_thrx < 0 then
                r10_thr <= (others => '0');
            elsif v_thrx > 255 then
                r10_thr <= (others => '1');
            else
                r10_thr <= unsigned(v_thrx(7 downto 0));
            end if;

            r10_hn   <= r9_hn;
            r10_dark <= r9_dark;
            r10_ctr  <= r9_ctr;
            r10_pol  <= r9_pol;

            -----------------------------------------------------------------
            -- s11: fringe chroma shift (case).
            -----------------------------------------------------------------
            case to_integer(s_frsel) is
                when 0      => v_fu := shift_right(r10_du, 7);
                                v_fv := shift_right(r10_dv, 7);
                when 1      => v_fu := shift_right(r10_du, 6);
                                v_fv := shift_right(r10_dv, 6);
                when 2      => v_fu := shift_right(r10_du, 5);
                                v_fv := shift_right(r10_dv, 5);
                when 3      => v_fu := shift_right(r10_du, 4);
                                v_fv := shift_right(r10_dv, 4);
                when 4      => v_fu := shift_right(r10_du, 3);
                                v_fv := shift_right(r10_dv, 3);
                when 5      => v_fu := shift_right(r10_du, 2);
                                v_fv := shift_right(r10_dv, 2);
                when 6      => v_fu := shift_right(r10_du, 1);
                                v_fv := shift_right(r10_dv, 1);
                when others => v_fu := r10_du;
                                v_fv := r10_dv;
            end case;
            r11_fu <= v_fu;
            r11_fv <= v_fv;

            r11_hn <= r10_hn;
            r11_py <= r10_py; r11_du <= r10_du; r11_dv <= r10_dv;
            r11_thr <= r10_thr;
            r11_dark <= r10_dark; r11_ctr <= r10_ctr;
            r11_pol <= r10_pol;

            -----------------------------------------------------------------
            -- s12: static ink decision, light luma, blanking flag.
            -----------------------------------------------------------------
            if (r11_hn(9 downto 2) < r11_thr) xor (r11_pol = '1') then
                r12_ly <= to_unsigned(96, 10);      -- static: dark fleck
            else
                r12_ly <= to_unsigned(720, 10);     -- static: light fleck
            end if;

            r12_py <= r11_py; r12_du <= r11_du; r12_dv <= r11_dv;
            r12_fu <= r11_fu; r12_fv <= r11_fv;
            r12_dark <= r11_dark; r12_ctr <= r11_ctr;
            r12_blank <= not pipe(LATENCY - 2).avid;

            -----------------------------------------------------------------
            -- s13 (output register): contour / bands / static + blanking.
            -----------------------------------------------------------------
            if r12_blank = '1' then
                s_io.y <= std_logic_vector(C_BLKY);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            elsif r12_ctr = '1' then
                -- thick black separating outline
                s_io.y <= std_logic_vector(C_BLKY);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            elsif r12_dark = '1' then
                s_io.y <= std_logic_vector(r12_py);
                -- +512 on a 10-bit signed delta = flip the MSB
                s_io.u <= std_logic_vector(unsigned(r12_du) xor "1000000000");
                s_io.v <= std_logic_vector(unsigned(r12_dv) xor "1000000000");
            else
                s_io.y <= std_logic_vector(r12_ly);
                s_io.u <= std_logic_vector(unsigned(r12_fu) xor "1000000000");
                s_io.v <= std_logic_vector(unsigned(r12_fv) xor "1000000000");
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture cleave;
