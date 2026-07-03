-- glitchscape.vhd  (v0.1 — "Pixel-Sort Landscape")
--
-- Databends a live source into the glitch-landscape look: a dense rectangular
-- mass of vertically-streaked, hyper-saturated jewel-tone color floating on a
-- flat teal field, punctured by blown-white hot-zones, threaded with fine white
-- contour filaments, with a jagged torn top edge and columns dripping downward.
--
-- STREAMING pipeline (no frame buffer / no BRAM), forked from the revenant
-- engine.  The bright parts of the source become the colored mass; everything
-- below the Mass Key falls into the teal "sky".  The vertical "sorted" streaks
-- are iso-bars of a height field = px + (luma >> shift) + ripple/jitter: because
-- luma bows the field, the bars bend around the form like pixel-sorted smears.
-- Each bar's HUE is locked to its bar index (a column hash), so neighbouring
-- bars are different jewel tones while the hue stays ~constant down a column —
-- the corrupted-JPEG / pixel-rain signature.  Brightness rides the live luma so
-- the source content still reads through the corruption.
--
-- Fixed colours are stored with U/V (Cb/Cr) SWAPPED to match the Videomancer
-- hardware output convention used by the HW-validated mondrian / revenant
-- palettes (stored U = Cr, stored V = Cb).
--
-- Controls:
--   K1 Mass Key      (crushed-luma above which the mass is drawn; high = sparse)
--   K2 Streak Width  (fill fraction of each vertical bar; high = fat bars)
--   K3 Saturation    (chroma intensity of the jewel palette)
--   K4 Palette       (Jewel / Neon / Vaporwave / Mono)
--   K5 Channel Shift (base chroma-separation fringe amount)
--   K6 Filaments     (density of the fine white contour veins)
--   S7 Channel Sep   (RGB-style U/V fringe on/off; amount = K5 + chaos)
--   S8 Hot Zones     (blown-white bloom cores at the brightest source spots)
--   S9 Drips         (mass bleeds downward in the lower frame)
--   S10 Compress     (block/scanline JPEG artifacting + grain)
--   S11 Bypass       (raw input passthrough)
--   P12 Glitch Intensity (HERO fader: master meltdown.  Scales the field bend,
--        ripple, jitter, chroma fringe, bloom, drip reach and grain at once.
--        0 = mostly-intact lightly-streaked source; 1 = full corruption.)
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

architecture glitchscape of program_top is

    -- Total datapath latency: 8 assembly stages + CHR_MAX (8) chroma-delay taps.
    constant CHR_MAX : natural := 8;
    constant LATENCY : natural := 8 + CHR_MAX;          -- = 16

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Teal background #18A9A6 in 10-bit YUV (BT.601), stored U=Cr, V=Cb.
    constant TEAL_Y : unsigned(9 downto 0) := to_unsigned(502, 10);
    constant TEAL_U : unsigned(9 downto 0) := to_unsigned(224, 10);   -- = Cr
    constant TEAL_V : unsigned(9 downto 0) := to_unsigned(604, 10);   -- = Cb

    constant PITCH_W : integer := 4;                    -- bar pitch = 2^4 = 16 px
    constant BAR_W   : integer := 11;                   -- bar fill (px of 16, fixed)

    --------------------------------------------------------------------------
    -- Jewel-tone bar palettes.  Indexed 0..7 by a per-bar column hash.  Only
    -- chroma is stored (U=Cr, V=Cb); bar luma rides the live source luma.
    -- 4 palettes selected by K4: Jewel / Neon / Vaporwave / Mono.
    --------------------------------------------------------------------------
    type t_pal8   is array(0 to 7) of unsigned(9 downto 0);
    type t_palset is array(0 to 3) of t_pal8;

    -- U = Cr (red-difference)
    constant PAL_U : t_palset := (
        -- Jewel: crimson, magenta, violet, blue, cyan, green, orange, hotpink
        (to_unsigned(760,10),to_unsigned(700,10),to_unsigned(630,10),to_unsigned(440,10),
         to_unsigned(300,10),to_unsigned(340,10),to_unsigned(648,10),to_unsigned(610,10)),
        -- Neon: hotter, more extreme
        (to_unsigned(820,10),to_unsigned(760,10),to_unsigned(680,10),to_unsigned(400,10),
         to_unsigned(250,10),to_unsigned(300,10),to_unsigned(700,10),to_unsigned(680,10)),
        -- Vaporwave: pink / lilac / cyan biased
        (to_unsigned(690,10),to_unsigned(660,10),to_unsigned(600,10),to_unsigned(470,10),
         to_unsigned(360,10),to_unsigned(470,10),to_unsigned(660,10),to_unsigned(620,10)),
        -- Mono: neutral (grayscale streaks)
        (others => to_unsigned(512,10)));

    -- V = Cb (blue-difference)
    constant PAL_V : t_palset := (
        -- Jewel
        (to_unsigned(360,10),to_unsigned(636,10),to_unsigned(700,10),to_unsigned(760,10),
         to_unsigned(600,10),to_unsigned(330,10),to_unsigned(360,10),to_unsigned(520,10)),
        -- Neon
        (to_unsigned(330,10),to_unsigned(660,10),to_unsigned(730,10),to_unsigned(790,10),
         to_unsigned(640,10),to_unsigned(300,10),to_unsigned(340,10),to_unsigned(540,10)),
        -- Vaporwave
        (to_unsigned(560,10),to_unsigned(660,10),to_unsigned(700,10),to_unsigned(720,10),
         to_unsigned(640,10),to_unsigned(560,10),to_unsigned(520,10),to_unsigned(600,10)),
        -- Mono
        (others => to_unsigned(512,10)));

    function clamp10(v : signed) return unsigned is
    begin
        if v < 0 then       return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else                return unsigned(resize(v, 10));
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Raster position / animation.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal px        : unsigned(11 downto 0) := (others => '0');
    signal py        : unsigned(10 downto 0) := (others => '0');
    signal frame_act : std_logic := '0';
    signal frame_cnt : unsigned(9 downto 0) := (others => '0');

    signal act_h     : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal max_px    : unsigned(11 downto 0) := (others => '0');
    signal max_py    : unsigned(10 downto 0) := (others => '0');
    signal s_half    : unsigned(10 downto 0) := to_unsigned(360, 11);  -- act_h/2

    signal lfsr      : unsigned(15 downto 0) := x"ACE1";
    signal rwalk     : signed(8 downto 0)    := (others => '0');  -- leaky brown-noise wander

    --------------------------------------------------------------------------
    -- Per-frame latched parameters.
    --------------------------------------------------------------------------
    signal s_mask    : unsigned(9 downto 0) := to_unsigned(300,10);   -- mass key
    signal s_blk     : unsigned(9 downto 0) := to_unsigned(64,10);    -- contrast black point
    signal s_satsel  : unsigned(2 downto 0) := to_unsigned(3,3);      -- saturation step
    signal s_palsel  : unsigned(1 downto 0) := (others => '0');       -- palette select
    signal s_fildens : unsigned(5 downto 0) := (others => '0');       -- filament gate

    signal s_chsep   : std_logic := '0';
    signal s_hot     : std_logic := '0';
    signal s_drip    : std_logic := '0';
    signal s_comp    : std_logic := '0';
    signal s_bypass  : std_logic := '0';

    -- P12-derived (all shift-based, no multiplies)
    signal s_dispshr : integer range 3 to 6 := 6;        -- luma->bend shift
    signal s_riplsh  : integer range 0 to 3 := 0;        -- ripple amplitude shift
    signal s_jitmsk  : unsigned(5 downto 0) := (others => '0');
    signal s_chroff  : integer range 0 to CHR_MAX := 0;  -- chroma fringe (px)
    signal s_noisem  : unsigned(5 downto 0) := (others => '0');
    signal s_hotkey  : unsigned(9 downto 0) := to_unsigned(980,10);
    signal s_bloom   : unsigned(9 downto 0) := (others => '0');
    signal s_dripshr : integer range 2 to 6 := 6;
    signal s_qmask   : unsigned(9 downto 0) := (others => '0'); -- compress luma quantize

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe.
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Datapath stage registers.
    --------------------------------------------------------------------------
    -- s1: capture luma, ripple/jitter
    signal r1_lsub : signed(11 downto 0) := (others => '0');
    signal r1_px   : unsigned(11 downto 0) := (others => '0');
    signal r1_py   : unsigned(10 downto 0) := (others => '0');
    signal r1_rip  : signed(11 downto 0) := (others => '0');
    signal r1_drp  : unsigned(9 downto 0) := (others => '0');   -- drip threshold relief
    -- s2: contrast crush
    signal r2_lp   : signed(13 downto 0) := (others => '0');
    signal r2_px   : unsigned(11 downto 0) := (others => '0');
    signal r2_py   : unsigned(10 downto 0) := (others => '0');
    signal r2_rip  : signed(11 downto 0) := (others => '0');
    signal r2_drp  : unsigned(9 downto 0) := (others => '0');
    -- s3: crushed luma Lc
    signal r3_lc   : unsigned(9 downto 0) := (others => '0');
    signal r3_px   : unsigned(11 downto 0) := (others => '0');
    signal r3_py   : unsigned(10 downto 0) := (others => '0');
    signal r3_rip  : signed(11 downto 0) := (others => '0');
    signal r3_drp  : unsigned(9 downto 0) := (others => '0');
    -- s4: field displacement = Lc >> shift
    signal r4_disp : unsigned(13 downto 0) := (others => '0');
    signal r4_lc   : unsigned(9 downto 0) := (others => '0');
    signal r4_px   : unsigned(11 downto 0) := (others => '0');
    signal r4_py   : unsigned(10 downto 0) := (others => '0');
    signal r4_rip  : signed(11 downto 0) := (others => '0');
    signal r4_drp  : unsigned(9 downto 0) := (others => '0');
    -- s5: field -> bar/streak/idx + mass/hot/filament flags
    signal r5_idx  : integer range 0 to 7 := 0;
    signal r5_streak : std_logic := '0';
    signal r5_mass : std_logic := '0';
    signal r5_hot  : std_logic := '0';
    signal r5_halo : std_logic := '0';
    signal r5_fil  : std_logic := '0';
    signal r5_lc   : unsigned(9 downto 0) := (others => '0');
    signal r5_px   : unsigned(11 downto 0) := (others => '0');
    signal r5_py   : unsigned(10 downto 0) := (others => '0');
    -- s6: palette chroma + saturation products
    signal r6_su, r6_sv : signed(13 downto 0) := (others => '0');
    signal r6_streak, r6_mass, r6_hot, r6_halo, r6_fil : std_logic := '0';
    signal r6_lc   : unsigned(9 downto 0) := (others => '0');
    signal r6_px   : unsigned(11 downto 0) := (others => '0');
    signal r6_py   : unsigned(10 downto 0) := (others => '0');
    -- s7: assembled base colour
    signal r7_y, r7_u, r7_v : unsigned(9 downto 0) := C_MID;
    signal r7_px   : unsigned(11 downto 0) := (others => '0');
    signal r7_py   : unsigned(10 downto 0) := (others => '0');

    -- chroma-separation delay lines.
    type t_chr is array(0 to 2*CHR_MAX) of unsigned(9 downto 0);
    signal yd : t_chr := (others => C_MID);
    signal ud : t_chr := (others => C_MID);
    signal vd : t_chr := (others => C_MID);

    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Raster position, animation counters, per-frame parameter latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_chaos, v_chr : integer;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- free-running LFSR (16-bit Galois)
            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

            -- X counter
            if v_h_edge = '1' then
                if px > max_px then max_px <= px; end if;
                px <= (others => '0');
                -- leaky random walk -> smooth organic wander (brown noise);
                -- step amplitude rides chaos via s_jitmsk.
                rwalk <= rwalk - shift_right(rwalk, 2)
                       + resize(signed(resize(lfsr(5 downto 0) and s_jitmsk, 8))
                              - signed(resize('0' & s_jitmsk(5 downto 1), 8)), 9);
            elsif data_in.avid = '1' then
                px <= px + 1;
            end if;

            -- Y counter (anchored to first active line)
            if v_v_edge = '1' then
                if py > max_py then max_py <= py; end if;
                py        <= (others => '0');
                frame_act <= '0';
                frame_cnt <= frame_cnt + 1;
                act_h  <= max_py;
                s_half <= '0' & max_py(10 downto 1);
                max_px <= (others => '0');
                max_py <= (others => '0');

                ---------------------------------------------------------------
                -- Parameter latch (once per frame).
                ---------------------------------------------------------------
                s_mask   <= unsigned(registers_in(0)(9 downto 0));          -- K1 Mass Key
                s_blk    <= resize(unsigned(registers_in(1)(9 downto 2)), 10); -- K2 Contrast: black point 0..255
                s_satsel <= unsigned(registers_in(2)(9 downto 7));          -- K3 saturation step
                s_palsel <= unsigned(registers_in(3)(9 downto 8));          -- K4 palette
                s_fildens<= unsigned(registers_in(5)(9 downto 4));          -- K6 filament gate 0..63

                s_chsep  <= registers_in(6)(0);   -- S7
                s_hot    <= registers_in(6)(1);   -- S8
                s_drip   <= registers_in(6)(2);   -- S9
                s_comp   <= registers_in(6)(3);   -- S10
                s_bypass <= registers_in(6)(4);   -- S11

                -- P12 master chaos -> per-effect intensities.
                v_chaos := to_integer(unsigned(registers_in(7)(9 downto 0)));   -- 0..1023
                s_dispshr <= 6 - v_chaos / 256;                            -- 6..3 (more bend)
                s_riplsh  <= v_chaos / 256;                                -- 0..2
                s_jitmsk  <= to_unsigned((2 ** (2 + v_chaos / 256)) - 1, 6); -- 3..15
                if registers_in(6)(0) = '1' then
                    v_chr := to_integer(unsigned(registers_in(4)(9 downto 6)))/2 + v_chaos/256; -- K5 + chaos
                    if v_chr > CHR_MAX then v_chr := CHR_MAX; end if;
                    s_chroff <= v_chr;
                else
                    s_chroff <= 0;
                end if;
                s_noisem  <= to_unsigned(v_chaos / 32, 6);                 -- 0..31
                s_hotkey  <= to_unsigned(980 - v_chaos / 4, 10);           -- 980..724
                s_bloom   <= to_unsigned(v_chaos / 8, 10);                 -- 0..127
                s_dripshr <= 6 - v_chaos / 256;                            -- 6..3 (longer drips)
                if registers_in(6)(3) = '1' then
                    s_qmask <= to_unsigned((2 ** (3 + v_chaos / 256)) - 1, 10); -- 7..63
                else
                    s_qmask <= (others => '0');
                end if;

            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                py        <= (others => '0');
            elsif v_h_edge = '1' then
                py <= py + 1;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Main datapath.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_a, v_b  : unsigned(7 downto 0);
        variable v_ta, v_tb : signed(8 downto 0);
        variable v_curve  : signed(11 downto 0);
        variable v_lp     : signed(13 downto 0);
        variable v_field  : unsigned(13 downto 0);
        variable v_phase  : unsigned(6 downto 0);
        variable v_bar    : unsigned(13 downto 0);
        variable v_hash   : unsigned(2 downto 0);
        variable v_fil    : unsigned(13 downto 0);
        variable v_thr    : signed(11 downto 0);
        variable v_idx    : integer range 0 to 7;
        variable v_du, v_dv : signed(11 downto 0);
        variable v_su, v_sv : signed(13 downto 0);
        variable v_y      : unsigned(9 downto 0);
        variable v_u, v_v  : signed(13 downto 0);
        variable v_dimy   : signed(13 downto 0);
        variable v_noise  : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -----------------------------------------------------------------
            -- s1: capture luma; ripple+jitter; drip threshold relief.
            -----------------------------------------------------------------
            r1_lsub <= signed(resize(unsigned(data_in.y), 12)) - signed(resize(s_blk, 12)); -- K2 contrast crush
            r1_px <= px; r1_py <= py;

            -- organic horizontal bend: two diagonal triangle waves mixing px & py
            -- at different angles/periods -> rounded, non-triangular curves that
            -- differ column-to-column; summed with the leaky random walk for
            -- organic wander.  Amplitude rides chaos via s_riplsh.
            v_a := resize(py(8 downto 1), 8) + resize(px(9 downto 5), 8)
                 + resize(frame_cnt(6 downto 0), 8);
            if v_a(7) = '1' then v_a := not v_a; end if;          -- fold -> 0..127
            v_b := resize(py(7 downto 0), 8) - resize(px(8 downto 4), 8)
                 + resize(frame_cnt(5 downto 0), 8);
            if v_b(7) = '1' then v_b := not v_b; end if;          -- fold -> 0..127
            v_ta := signed(resize(v_a(6 downto 0), 9)) - 64;      -- -64..63
            v_tb := signed(resize(v_b(6 downto 0), 9)) - 64;      -- -64..63
            v_curve := resize(v_ta, 12) + resize(shift_right(v_tb, 1), 12);
            r1_rip <= shift_left(resize(v_curve, 12), s_riplsh) + resize(rwalk, 12);

            -- drips: in the lower frame, lower the mass threshold so the mass
            -- bleeds downward.  relief grows with depth past the mid-line.
            if s_drip = '1' and py >= s_half then
                r1_drp <= resize(shift_right(py - s_half, s_dripshr), 10);
            else
                r1_drp <= (others => '0');
            end if;

            -----------------------------------------------------------------
            -- s2: contrast crush (fixed x1.5 via shift-add).
            -----------------------------------------------------------------
            r2_lp <= resize(r1_lsub, 14) + resize(shift_right(r1_lsub, 1), 14);
            r2_px <= r1_px; r2_py <= r1_py; r2_rip <= r1_rip; r2_drp <= r1_drp;

            -----------------------------------------------------------------
            -- s3: crushed luma Lc.
            -----------------------------------------------------------------
            v_lp := r2_lp;
            r3_lc <= clamp10(resize(v_lp, 13));
            r3_px <= r2_px; r3_py <= r2_py; r3_rip <= r2_rip; r3_drp <= r2_drp;

            -----------------------------------------------------------------
            -- s4: field displacement = luma >> shift (no multiply).
            -----------------------------------------------------------------
            r4_disp <= shift_right(resize(r3_lc, 14), s_dispshr);
            r4_lc <= r3_lc;
            r4_px <= r3_px; r4_py <= r3_py; r4_rip <= r3_rip; r4_drp <= r3_drp;

            -----------------------------------------------------------------
            -- s5: height field -> vertical bars; bar hash -> jewel hue; mass /
            -- hot / halo / filament flags.
            -----------------------------------------------------------------
            v_field := resize(r4_px, 14)
                     + resize(r4_disp, 14)
                     + unsigned(resize(r4_rip + 1024, 14));   -- bias rip positive
            v_phase := resize(v_field(PITCH_W - 1 downto 0), 7);
            v_bar   := shift_right(v_field, PITCH_W);
            v_hash  := v_bar(2 downto 0) xor v_bar(5 downto 3) xor v_bar(8 downto 6);
            r5_idx  <= to_integer(v_hash);

            if v_phase < to_unsigned(BAR_W, 7) then r5_streak <= '1'; else r5_streak <= '0'; end if;

            -- mass membership: crushed luma above (key - drip relief), with a
            -- column-stable edge fray + per-row jitter so the top edge tears.
            v_thr := signed(resize(s_mask, 12))
                   - signed(resize(r4_drp, 12))
                   + signed(resize((r4_px(4 downto 1) xor r4_px(8 downto 5)), 12));  -- edge fray
            if signed(resize(r4_lc, 12)) >= v_thr then r5_mass <= '1'; else r5_mass <= '0'; end if;

            -- hot zones / bloom halo.
            r5_hot <= '0'; r5_halo <= '0';
            if s_hot = '1' then
                if r4_lc >= s_hotkey then
                    r5_hot <= '1';
                elsif r4_lc >= (s_hotkey - to_unsigned(120, 10)) then
                    r5_halo <= '1';
                end if;
            end if;

            -- fine white contour filaments: thin iso-lines of a second field
            -- (finer pitch, different luma weight), gated sparse by the LFSR.
            v_fil := resize(r4_px, 14) + shift_right(resize(r4_lc,14), 2)
                   + unsigned(resize(r4_rip + 1024, 14));
            if v_fil(2 downto 0) = "000"
               and (lfsr(5 downto 0) < s_fildens)
               and r4_lc >= to_unsigned(120, 10) then
                r5_fil <= '1';
            else
                r5_fil <= '0';
            end if;

            r5_lc <= r4_lc; r5_px <= r4_px; r5_py <= r4_py;

            -----------------------------------------------------------------
            -- s6: palette chroma lookup (by bar hash) + saturation products.
            -----------------------------------------------------------------
            v_idx := r5_idx;
            v_du := signed(resize(PAL_U(to_integer(s_palsel))(v_idx), 12)) - 512;
            v_dv := signed(resize(PAL_V(to_integer(s_palsel))(v_idx), 12)) - 512;
            case to_integer(s_satsel) is
                when 0 =>      v_su := (others => '0'); v_sv := (others => '0');
                when 1 =>      v_su := resize(shift_right(v_du,1),14); v_sv := resize(shift_right(v_dv,1),14);
                when 2 =>      v_su := resize(v_du,14); v_sv := resize(v_dv,14);
                when 3 =>      v_su := resize(v_du,14)+resize(shift_right(v_du,1),14);
                               v_sv := resize(v_dv,14)+resize(shift_right(v_dv,1),14);
                when 4 =>      v_su := shift_left(resize(v_du,14),1); v_sv := shift_left(resize(v_dv,14),1);
                when 5 =>      v_su := shift_left(resize(v_du,14),1)+resize(shift_right(v_du,1),14);
                               v_sv := shift_left(resize(v_dv,14),1)+resize(shift_right(v_dv,1),14);
                when 6 =>      v_su := shift_left(resize(v_du,14),1)+resize(v_du,14);
                               v_sv := shift_left(resize(v_dv,14),1)+resize(v_dv,14);
                when others => v_su := shift_left(resize(v_du,14),2); v_sv := shift_left(resize(v_dv,14),2);
            end case;
            r6_su <= v_su; r6_sv <= v_sv;
            r6_streak <= r5_streak; r6_mass <= r5_mass; r6_hot <= r5_hot;
            r6_halo <= r5_halo; r6_fil <= r5_fil;
            r6_lc <= r5_lc; r6_px <= r5_px; r6_py <= r5_py;

            -----------------------------------------------------------------
            -- s7: assemble base colour.
            --   teal sky  -> outside the mass
            --   hot/halo  -> blown-white cores + bloom
            --   filament  -> white veins
            --   streak    -> bar: live luma + jewel chroma
            --   gap       -> deep shadow tint between bars
            -----------------------------------------------------------------
            v_u := to_signed(512,14) + r6_su;
            v_v := to_signed(512,14) + r6_sv;
            if r6_mass = '0' then
                r7_y <= TEAL_Y; r7_u <= TEAL_U; r7_v <= TEAL_V;
            elsif r6_hot = '1' then
                r7_y <= to_unsigned(1020,10); r7_u <= C_MID; r7_v <= C_MID;   -- blown white
            elsif r6_fil = '1' then
                r7_y <= to_unsigned(990,10);  r7_u <= C_MID; r7_v <= C_MID;   -- white vein
            elsif r6_halo = '1' then
                r7_y <= clamp10(signed(resize(r6_lc,13)) + signed(resize(s_bloom,13)));
                r7_u <= clamp10(v_u); r7_v <= clamp10(v_v);
            elsif r6_streak = '1' then
                r7_y <= clamp10(signed(resize(r6_lc,13)) + signed(resize(shift_right(s_bloom,1),13)));
                r7_u <= clamp10(v_u); r7_v <= clamp10(v_v);
            else
                -- gap: dark jewel shadow (quarter luma, half chroma about mid)
                r7_y <= shift_right(r6_lc, 2);
                r7_u <= clamp10(to_signed(512,14) + shift_right(r6_su,1));
                r7_v <= clamp10(to_signed(512,14) + shift_right(r6_sv,1));
            end if;
            r7_px <= r6_px; r7_py <= r6_py;

            -----------------------------------------------------------------
            -- s8: compression (block luma quantize + scanline dim) + grain,
            -- then feed the chroma-separation delay line.
            -----------------------------------------------------------------
            v_dimy := signed(resize(r7_y, 14));
            if s_comp = '1' then
                -- block luma quantize: snap to coarse steps (clears low bits)
                v_y    := r7_y and not s_qmask;
                v_dimy := signed(resize(v_y, 14));
                -- scanline dim on odd lines
                if r7_py(0) = '1' then
                    v_dimy := v_dimy - signed(resize(shift_right(v_y,3),14));
                end if;
            end if;
            -- grain (rides chaos)
            v_noise := signed(resize(lfsr(5 downto 0) and s_noisem, 8))
                     - signed(resize('0' & s_noisem(5 downto 1), 8));
            v_dimy := v_dimy + resize(v_noise, 14);

            yd(0) <= clamp10(v_dimy);
            ud(0) <= r7_u;
            vd(0) <= r7_v;
            for i in 1 to 2*CHR_MAX loop
                yd(i) <= yd(i-1); ud(i) <= ud(i-1); vd(i) <= vd(i-1);
            end loop;

            -----------------------------------------------------------------
            -- output: chroma sep pulls U ahead / pushes V back -> opposite
            -- fringes.  bypass passes the delayed raw input.
            -----------------------------------------------------------------
            if s_bypass = '1' then
                s_io.y <= pipe(LATENCY - 1).y;
                s_io.u <= pipe(LATENCY - 1).u;
                s_io.v <= pipe(LATENCY - 1).v;
            else
                s_io.y <= std_logic_vector(yd(CHR_MAX));
                s_io.u <= std_logic_vector(ud(CHR_MAX - s_chroff));
                s_io.v <= std_logic_vector(vd(CHR_MAX + s_chroff));
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

end architecture glitchscape;
