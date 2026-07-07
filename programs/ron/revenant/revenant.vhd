-- revenant.vhd  (v0.1 — "Ghost in the Signal": analog luma-contour glitch)
--
-- Re-draws a live source (a portrait on near-black ground) as a degrading analog
-- signal: the BRIGHT parts of the figure are redrawn by swarms of thin electric
-- contour lines that hug the volume, the DARK parts fall into true black, the very
-- brightest cores (eyes) blow out into a solid icy fill + halo, and the whole thing
-- frays / fringes / jitters under a master "signal decay" fader.
--
-- This is a STREAMING pipeline (no frame buffer): every effect is computed per
-- pixel in the coordinate + luma domain, so it needs no BRAM.  The contour lines
-- are iso-lines of a height field = (row) + (luma * displacement) + ripple + jitter;
-- because luma varies across the form, the iso-lines bow around the volume — a
-- topographic / oscilloscope redraw.  Line presence is gated by a luma key, so the
-- silhouette frays into the black exactly where the body ends.
--
-- Fixed colours are stored with U/V (Cb/Cr) SWAPPED to match the Videomancer
-- hardware output convention used by the HW-validated mondrian / c64 palettes
-- (stored U = Cr, stored V = Cb).
--
-- Controls:
--   K1 Luma Key     (brightness above which line-fill begins; high = sparse)
--   K2 Line Density (contour pitch; high = dense engraved hatching)
--   K3 Contrast     (black-point crush + gain; high = more falls to black)
--   K4 Hue Rotate   (spin the violet/cyan/white tritone in 90-deg steps)
--   K5 Saturation   (chroma intensity of the palette map)
--   K6 Eye Key      (top-luma percentile that flips lines -> solid icy fill + halo)
--   S7 Warp         (contour displacement on/off; amount rides P12)
--   S8 Chroma Sep   (RGB-style U/V fringe on/off; amount rides P12)
--   S9 Text         (reserved — phase 2 glyph overlay)
--   S10 Windows     (cascading wireframe dialog boxes, lower-right)
--   S11 Bypass      (pass the raw input through)
--   P12 Master Chaos (HERO fader: scales warp + ripple + jitter + chroma fringe +
--                     tracking-band activity + line bloom all at once.  0 = nearly
--                     clean stable portrait; 1 = fully possessed, fraying ghost.)
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

architecture revenant of program_top is

    -- Total datapath latency: 11 assembly stages + CHR_MAX (8) chroma-delay taps.
    constant CHR_MAX : natural := 8;
    constant LATENCY : natural := 11 + CHR_MAX;       -- = 19

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK   : unsigned(9 downto 0) := to_unsigned( 24, 10);  -- true-black floor

    --------------------------------------------------------------------------
    -- Electric tritone palette: black -> indigo -> violet -> blue -> cyan ->
    -- bright cyan -> white, indexed by crushed-luma zone (top 3 bits).
    -- Stored U = Cr, stored V = Cb (HW convention).
    --------------------------------------------------------------------------
    type t_pal8 is array(0 to 7) of unsigned(9 downto 0);
    constant PAL_Y : t_pal8 := (
        to_unsigned( 24,10), to_unsigned(140,10), to_unsigned(250,10), to_unsigned(330,10),
        to_unsigned(470,10), to_unsigned(640,10), to_unsigned(820,10), to_unsigned(980,10));
    constant PAL_U : t_pal8 := (   -- = Cr
        to_unsigned(512,10), to_unsigned(560,10), to_unsigned(590,10), to_unsigned(486,10),
        to_unsigned(360,10), to_unsigned(220,10), to_unsigned(330,10), to_unsigned(500,10));
    constant PAL_V : t_pal8 := (   -- = Cb
        to_unsigned(512,10), to_unsigned(684,10), to_unsigned(720,10), to_unsigned(760,10),
        to_unsigned(700,10), to_unsigned(640,10), to_unsigned(600,10), to_unsigned(520,10));

    -- Eye solid-key + halo colours (icy blue-white).
    constant EYE_Y  : unsigned(9 downto 0) := to_unsigned(990, 10);
    constant EYE_U  : unsigned(9 downto 0) := to_unsigned(470, 10);   -- = Cr
    constant EYE_V  : unsigned(9 downto 0) := to_unsigned(560, 10);   -- = Cb
    constant HALO_Y : unsigned(9 downto 0) := to_unsigned(560, 10);

    -- Window-cascade stroke colour (hot cyan-white).
    constant WIN_Y  : unsigned(9 downto 0) := to_unsigned(900, 10);
    constant WIN_U  : unsigned(9 downto 0) := to_unsigned(300, 10);
    constant WIN_V  : unsigned(9 downto 0) := to_unsigned(610, 10);

    constant N_WIN     : natural := 5;     -- window-cascade depth
    constant WIN_STROK : integer := 2;     -- stroke thickness (px)

    function clamp10(v : signed) return unsigned is
    begin
        if v < 0 then       return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else                return unsigned(resize(v, 10));
        end if;
    end function;

    -- Triangle wave: 8-bit phase -> signed -64..63.  Summed at several frequencies
    -- to build an organic, multi-octave wobble (no single straight zig-zag).
    function tri(p : unsigned(7 downto 0)) return signed is
        variable q : unsigned(7 downto 0);
    begin
        q := p;
        if q(7) = '1' then q := not q; end if;     -- fold to a 0..127 up/down ramp
        return resize(signed('0' & q(6 downto 0)), 9) - 64;
    end function;

    --------------------------------------------------------------------------
    -- Raster position / animation.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal px        : unsigned(11 downto 0) := (others => '0');  -- x in active line
    signal py        : unsigned(10 downto 0) := (others => '0');  -- line index
    signal frame_act : std_logic := '0';
    signal frame_cnt : unsigned(9 downto 0) := (others => '0');   -- frame counter

    signal act_w     : unsigned(11 downto 0) := to_unsigned(1280, 12); -- active width
    signal act_h     : unsigned(10 downto 0) := to_unsigned( 720, 11); -- active height
    signal max_px    : unsigned(11 downto 0) := (others => '0');
    signal max_py    : unsigned(10 downto 0) := (others => '0');

    signal lfsr      : unsigned(15 downto 0) := x"ACE1";          -- free-running noise
    signal row_jit   : signed(7 downto 0)    := (others => '0');  -- per-row sync jitter
    signal noise_walk: signed(9 downto 0)    := (others => '0');  -- chaotic per-row drift
    signal s_glitchframe : std_logic := '0';                     -- this frame tears (occasional)
    signal s_glitchp : integer range 0 to 1023 := 0;             -- glitch-frame probability (P12)
    signal trk_top   : unsigned(10 downto 0) := (others => '0');  -- tracking-band top

    --------------------------------------------------------------------------
    -- Per-frame latched parameters.
    --------------------------------------------------------------------------
    signal s_blk     : unsigned(9 downto 0) := (others => '0');   -- black point (crush)
    signal s_key     : unsigned(9 downto 0) := to_unsigned(300,10);-- luma key (line start)
    signal s_key2    : unsigned(9 downto 0) := to_unsigned(660,10);-- dense-hatch zone
    signal s_phw     : integer range 2 to 6 := 4;                 -- pitch shift (pitch=2^phw)
    signal s_thick   : unsigned(4 downto 0) := to_unsigned(4,5);  -- line thickness (px)
    signal s_hue     : unsigned(1 downto 0) := (others => '0');   -- hue rotation (x90)
    signal s_satsel  : unsigned(2 downto 0) := to_unsigned(2,3);  -- saturation step (2=unity)
    signal s_eyek    : unsigned(9 downto 0) := to_unsigned(950,10);-- eye solid-key
    signal s_eyeh    : unsigned(9 downto 0) := to_unsigned(880,10);-- eye halo threshold

    signal s_warp    : std_logic := '0';
    signal s_chroma  : std_logic := '0';
    signal s_windows : std_logic := '0';
    signal s_bypass  : std_logic := '0';

    -- chaos-derived (P12) intensities (all shift-based — no multiplies)
    signal s_dispshr : integer range 3 to 12 := 12;              -- luma->displacement shift (12=off)
    signal s_riplsh  : integer range 0 to 3 := 0;                 -- ripple amplitude shift
    signal s_walkshr : integer range 4 to 7 := 7;                 -- random-walk drift shift (less=more)
    signal s_jitmsk  : unsigned(5 downto 0) := (others => '0');   -- row-jitter mask
    signal s_chroff  : integer range 0 to CHR_MAX := 0;           -- base chroma fringe (px)
    signal s_noisem  : unsigned(5 downto 0) := (others => '0');   -- dither mask
    signal s_trk     : std_logic := '0';                         -- tracking active
    signal s_bright  : unsigned(9 downto 0) := (others => '0');   -- line bloom boost

    -- window-cascade edges, precomputed once per frame (keeps act_w/act_h adds
    -- out of the per-pixel path).
    type t_wedge is array(0 to N_WIN - 1) of unsigned(11 downto 0);
    signal win_l,  win_r  : t_wedge := (others => (others => '0'));  -- outer edges
    signal win_t,  win_b  : t_wedge := (others => (others => '0'));
    signal win_li, win_ri : t_wedge := (others => (others => '0'));  -- inner (stroke) edges
    signal win_ti, win_bi : t_wedge := (others => (others => '0'));

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe.
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Datapath stage registers.
    --------------------------------------------------------------------------
    -- s1: input capture + crush stage 1
    signal r1_lsub   : signed(11 downto 0) := (others => '0');
    signal r1_px     : unsigned(11 downto 0) := (others => '0');
    signal r1_py     : unsigned(10 downto 0) := (others => '0');
    signal r1_rip    : signed(11 downto 0) := (others => '0');   -- ripple+jitter offset
    signal r1_trk    : std_logic := '0';
    -- s2: crush stage (black-point sub then x1.5 via shift-add)
    signal r2_lp     : signed(13 downto 0) := (others => '0');
    signal r2_px     : unsigned(11 downto 0) := (others => '0');
    signal r2_py     : unsigned(10 downto 0) := (others => '0');
    signal r2_rip    : signed(11 downto 0) := (others => '0');
    signal r2_trk    : std_logic := '0';
    -- s3: crushed luma Lc
    signal r3_lc     : unsigned(9 downto 0) := (others => '0');
    signal r3_px     : unsigned(11 downto 0) := (others => '0');
    signal r3_py     : unsigned(10 downto 0) := (others => '0');
    signal r3_rip    : signed(11 downto 0) := (others => '0');
    signal r3_trk    : std_logic := '0';
    -- s4: contour displacement (luma >> shift, field-ready px)
    signal r4_disp   : unsigned(13 downto 0) := (others => '0');
    signal r4_lc     : unsigned(9 downto 0) := (others => '0');
    signal r4_px     : unsigned(11 downto 0) := (others => '0');
    signal r4_py     : unsigned(10 downto 0) := (others => '0');
    signal r4_rip    : signed(11 downto 0) := (others => '0');
    signal r4_trk    : std_logic := '0';
    -- s5: line decision + flags
    signal r5_line   : std_logic := '0';
    signal r5_eye    : std_logic := '0';
    signal r5_halo   : std_logic := '0';
    signal r5_idx    : integer range 0 to 7 := 0;
    signal r5_lc     : unsigned(9 downto 0) := (others => '0');
    signal r5_px     : unsigned(11 downto 0) := (others => '0');
    signal r5_py     : unsigned(10 downto 0) := (others => '0');
    signal r5_trk    : std_logic := '0';
    -- s6: palette lookup + sat products
    signal r6_line   : std_logic := '0';
    signal r6_eye    : std_logic := '0';
    signal r6_halo   : std_logic := '0';
    signal r6_py     : unsigned(10 downto 0) := (others => '0');
    signal r6_px     : unsigned(11 downto 0) := (others => '0');
    signal r6_trk    : std_logic := '0';
    signal r6_paly   : unsigned(9 downto 0) := (others => '0');
    signal r6_su     : signed(13 downto 0) := (others => '0');   -- saturated chroma delta U
    signal r6_sv     : signed(13 downto 0) := (others => '0');   -- saturated chroma delta V
    -- s7: hue-rotated palette chroma (line/fill colour) + carries
    signal r7_pu, r7_pv : signed(12 downto 0) := (others => '0');
    signal r7_paly   : unsigned(9 downto 0) := (others => '0');
    signal r7_line, r7_eye, r7_halo, r7_trk : std_logic := '0';
    signal r7_px     : unsigned(11 downto 0) := (others => '0');
    signal r7_py     : unsigned(10 downto 0) := (others => '0');
    -- s8: colour select (eye / halo / line / gap)
    signal r8_y, r8_u, r8_v : unsigned(9 downto 0) := C_MID;
    signal r8_trk    : std_logic := '0';
    signal r8_px     : unsigned(11 downto 0) := (others => '0');
    signal r8_py     : unsigned(10 downto 0) := (others => '0');
    -- s9: analog degradation
    signal r9_y, r9_u, r9_v : unsigned(9 downto 0) := C_MID;
    signal r9_px     : unsigned(11 downto 0) := (others => '0');
    signal r9_py     : unsigned(10 downto 0) := (others => '0');
    -- s10: per-window membership bits (X/Y factored) + carried base colour
    signal r10_y, r10_u, r10_v : unsigned(9 downto 0) := C_MID;
    signal r10_xin, r10_xstr : std_logic_vector(N_WIN - 1 downto 0) := (others => '0');
    signal r10_yin, r10_ystr : std_logic_vector(N_WIN - 1 downto 0) := (others => '0');

    -- chroma-separation delay lines (length 0..2*CHR_MAX).
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
        variable v_k3, v_chaos, v_k2, v_phw_base : integer;
        variable v_nw : signed(9 downto 0);
        variable v_hash : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- free-running LFSR (16-bit Galois, taps 16,14,13,11)
            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

            -- X counter
            if v_h_edge = '1' then
                if px > max_px then max_px <= px; end if;
                px <= (others => '0');
                -- per-row texture jitter from a DETERMINISTIC py hash: same every
                -- frame, so the lines have a fixed jagged character that does NOT
                -- shimmer/scroll.  Amplitude rides chaos (s_jitmsk).
                v_hash := py(5 downto 0) xor py(10 downto 5);
                row_jit <= signed(resize(v_hash and s_jitmsk, 8))
                         - signed(resize('0' & s_jitmsk(5 downto 1), 8));
                -- random-walk drift ONLY during a glitch frame -> mostly still, with
                -- occasional tearing.  Otherwise noise_walk is left to settle (below).
                if s_glitchframe = '1' then
                    v_nw := noise_walk + resize(signed(resize(lfsr(6 downto 0), 8)) - 64, 10);
                    if    v_nw >  480 then v_nw := to_signed( 480, 10);
                    elsif v_nw < -480 then v_nw := to_signed(-480, 10);
                    end if;
                    noise_walk <= v_nw;
                end if;
            elsif data_in.avid = '1' then
                px <= px + 1;
            end if;

            -- Y counter (anchored to first active line)
            if v_v_edge = '1' then
                if py > max_py then max_py <= py; end if;
                py        <= (others => '0');
                frame_act <= '0';
                frame_cnt <= frame_cnt + 1;
                -- latch measured active size, reset trackers
                act_w  <= max_px;
                act_h  <= max_py;
                max_px <= (others => '0');
                max_py <= (others => '0');
                -- Glitch-frame gate: occasional frames get the tearing drift (the
                -- probability rises with P12); calm frames let the drift settle back
                -- toward rest, so the picture is mostly still between glitches.
                if to_integer(unsigned(lfsr(9 downto 0))) < s_glitchp then
                    s_glitchframe <= '1';
                else
                    s_glitchframe <= '0';
                    noise_walk <= noise_walk - shift_right(noise_walk, 2);
                end if;

                ---------------------------------------------------------------
                -- Parameter latch (once per frame).
                ---------------------------------------------------------------
                -- K3 Contrast: black point (crush comes from a fixed x1.5 gain in
                -- the datapath, so no multiply).
                v_k3 := to_integer(unsigned(registers_in(2)(9 downto 2)));   -- 0..255
                s_blk  <= to_unsigned(v_k3, 10);

                -- K1 Luma Key + derived dense-hatch zone.
                s_key  <= unsigned(registers_in(0)(9 downto 0));
                s_key2 <= unsigned('0' & registers_in(0)(9 downto 1)) + to_unsigned(384, 10);

                -- K2 Line Density: vertical-stroke pitch 2^phw (32..8 px), with a
                -- fat ~3/8-pitch stroke so the lines read THICK.
                v_k2 := to_integer(unsigned(registers_in(1)(9 downto 7)));   -- 0..7
                v_phw_base := 5 - v_k2 / 3;                                  -- 5..3 (pitch 32..8)
                s_phw   <= v_phw_base;
                s_thick <= to_unsigned((2 ** v_phw_base) * 3 / 8, 5);       -- ~3/8 of pitch

                -- K4 Hue rotate (90-deg steps).
                s_hue <= unsigned(registers_in(3)(9 downto 8));

                -- K5 Saturation step (shift-add scale; 2 = unity).
                s_satsel <= unsigned(registers_in(4)(9 downto 7));

                -- K6 Eye solid-key + halo (a touch below).
                s_eyek <= unsigned(registers_in(5)(9 downto 0));
                if unsigned(registers_in(5)(9 downto 0)) > to_unsigned(80, 10) then
                    s_eyeh <= unsigned(registers_in(5)(9 downto 0)) - to_unsigned(80, 10);
                else
                    s_eyeh <= (others => '0');
                end if;

                -- toggles
                s_warp    <= registers_in(6)(0);   -- S7
                s_chroma  <= registers_in(6)(1);   -- S8
                s_windows <= registers_in(6)(3);   -- S10
                s_bypass  <= registers_in(6)(4);   -- S11

                -- P12 master chaos -> per-effect intensities (all shift-based).
                v_chaos := to_integer(unsigned(registers_in(7)(9 downto 0)));   -- 0..1023
                -- Contour curvature is ALWAYS on (lines must follow the luma); a base
                -- of luma>>6 (~16 px max bend), with Warp+chaos pushing it stronger.
                if registers_in(6)(0) = '1' then
                    s_dispshr <= 6 - v_chaos / 256;                            -- 6..3 (more chaos = bigger bend)
                else
                    s_dispshr <= 6;                                            -- base curvature
                end if;
                s_riplsh <= v_chaos / 256;                                     -- 0..3
                s_walkshr <= 7 - v_chaos / 256;                                -- 7..4 (more chaos = bigger drift)
                s_jitmsk <= to_unsigned((2 ** (2 + v_chaos / 256)) - 1, 6);    -- 3..31
                if registers_in(6)(1) = '1' then
                    s_chroff <= 1 + v_chaos / 128;                             -- 1..8
                else
                    s_chroff <= 0;
                end if;
                s_noisem <= to_unsigned(v_chaos / 32, 6);                      -- 0..31
                s_glitchp <= v_chaos / 4;                                       -- glitch-frame prob (0..255/1024)
                s_bright <= to_unsigned(v_chaos / 4, 10);                      -- 0..255

                -- Precompute window-cascade edges from measured active size, so
                -- the per-pixel overlay only does compares (no act_w/act_h adds).
                for k in 0 to N_WIN - 1 loop
                    win_l(k)  <= act_w - to_unsigned(360, 12) + to_unsigned(k*46, 12);
                    win_r(k)  <= act_w - to_unsigned( 40, 12) + to_unsigned(k*46, 12);
                    win_t(k)  <= ('0' & act_h) - to_unsigned(300, 12) + to_unsigned(k*38, 12);
                    win_b(k)  <= ('0' & act_h) - to_unsigned( 24, 12) + to_unsigned(k*38, 12);
                    win_li(k) <= act_w - to_unsigned(360, 12) + to_unsigned(k*46 + WIN_STROK, 12);
                    win_ri(k) <= act_w - to_unsigned( 42, 12) + to_unsigned(k*46, 12);  -- 40+stroke
                    win_ti(k) <= ('0' & act_h) - to_unsigned(300, 12) + to_unsigned(k*38 + WIN_STROK, 12);
                    win_bi(k) <= ('0' & act_h) - to_unsigned( 26, 12) + to_unsigned(k*38, 12);  -- 24+stroke
                end loop;

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
        variable v_field : unsigned(13 downto 0);
        variable v_phase : unsigned(6 downto 0);
        variable v_pmask : unsigned(6 downto 0);
        variable v_phw   : integer range 2 to 6;
        variable v_lc    : unsigned(9 downto 0);
        variable v_lp    : signed(13 downto 0);
        variable v_ph1, v_ph2, v_ph3 : unsigned(7 downto 0);
        variable v_wave  : signed(11 downto 0);
        variable v_rip   : signed(11 downto 0);
        variable v_idx   : integer range 0 to 7;
        variable v_du, v_dv : signed(11 downto 0);   -- raw palette chroma deltas
        variable v_su, v_sv : signed(13 downto 0);   -- saturation-scaled deltas
        variable v_y     : unsigned(9 downto 0);
        variable v_u, v_v : signed(12 downto 0);
        variable v_cu, v_cv : signed(12 downto 0);
        variable v_dimy  : signed(12 downto 0);
        variable v_noise : signed(7 downto 0);
        variable v_anystroke : boolean;
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -----------------------------------------------------------------
            -- s1: capture + crush stage 1 (subtract black point) + ripple.
            -----------------------------------------------------------------
            r1_lsub <= signed(resize(unsigned(data_in.y), 12)) - signed(resize(s_blk, 12));
            r1_px <= px; r1_py <= py;

            -- Organic curvy displacement: sum of 3 triangle waves at different
            -- vertical frequencies + animation speeds (multi-octave wobble breaks
            -- up the straight single-triangle zig-zag), plus a chaotic per-row
            -- random-walk drift and fine sync jitter.  Amplitude rides chaos.
            -- STATIC multi-octave wave (function of py only -> lines stand still, no
            -- constant scroll).  All frame-to-frame motion now comes from the glitch
            -- random walk (noise_walk), which is only active on occasional frames.
            v_ph1 := resize(py(9 downto 2), 8);   -- slow
            v_ph2 := resize(py(8 downto 1), 8);   -- medium
            v_ph3 := resize(py(7 downto 0), 8);   -- fast
            v_wave := resize(tri(v_ph1), 12)
                    + resize(shift_right(tri(v_ph2), 1), 12)
                    + resize(shift_right(tri(v_ph3), 2), 12);
            -- Keep the wave a SUBTLE secondary wobble (>>3) so the luma-curvature
            -- (r4_disp) stays the primary shape -> mostly-vertical lines that follow
            -- the form, not steep diagonals.  Amplitude still ramps with P12.
            v_rip := shift_right(shift_left(v_wave, s_riplsh), 3)
                   + resize(shift_right(noise_walk, s_walkshr), 12)
                   + resize(row_jit, 12);
            r1_rip <= v_rip;

            r1_trk <= '0';   -- tracking band removed (no full-width horizontal bands)

            -----------------------------------------------------------------
            -- s2: crush — fixed x1.5 gain via shift-add (steepens contrast).
            -----------------------------------------------------------------
            r2_lp <= resize(r1_lsub, 14) + resize(shift_right(r1_lsub, 1), 14);
            r2_px <= r1_px; r2_py <= r1_py; r2_rip <= r1_rip; r2_trk <= r1_trk;

            -----------------------------------------------------------------
            -- s3: crushed luma Lc.
            -----------------------------------------------------------------
            v_lp := r2_lp;
            v_lc := clamp10(resize(v_lp, 13));
            r3_lc <= v_lc;
            r3_px <= r2_px; r3_py <= r2_py; r3_rip <= r2_rip; r3_trk <= r2_trk;

            -----------------------------------------------------------------
            -- s4: contour displacement = luma >> shift (no multiply).
            -----------------------------------------------------------------
            r4_disp <= shift_right(resize(r3_lc, 14), s_dispshr);
            r4_lc <= r3_lc;
            r4_px <= r3_px; r4_py <= r3_py; r4_rip <= r3_rip; r4_trk <= r3_trk;

            -----------------------------------------------------------------
            -- s5: contour-line decision, luma key gate, eye / halo flags.
            -- Height field = row + (luma*disp) + ripple/jitter.  Iso-lines of
            -- this field bow around the form -> topographic redraw.
            -----------------------------------------------------------------
            -- VERTICAL contour lines: field is dominated by px, so iso-lines run
            -- vertically and bow left/right as the luma (r4_disp) changes down the
            -- column -> thick vertical strokes that follow the form's curvature,
            -- with the y-dependent wave + row jitter adding chaotic wobble/noise.
            v_field := resize(r4_px, 14)
                     + resize(r4_disp, 14)
                     + unsigned(resize(r4_rip + 1024, 14));   -- bias rip positive
            v_phw := s_phw;
            v_pmask := to_unsigned((2 ** v_phw) - 1, 7);
            v_phase := resize(v_field(6 downto 0) and v_pmask, 7);

            -- Eye solid-key is gated on RAW luma (pipe(3).y, aligned with r4) so ONLY
            -- genuinely bright specular highlights flip to solid.  The contrast crush
            -- pushes whole lit areas to full Lc, which must still render as vertical
            -- lines (not a solid block), so the key can't use Lc here.
            r5_line <= '0'; r5_eye <= '0'; r5_halo <= '0';
            if unsigned(pipe(3).y) >= s_eyek then
                r5_eye  <= '1';
            elsif r4_lc >= s_key and v_phase < resize(s_thick, 7) then
                r5_line <= '1';
            end if;

            r5_idx <= to_integer(r4_lc(9 downto 7));
            r5_lc  <= r4_lc;
            r5_px  <= r4_px; r5_py <= r4_py; r5_trk <= r4_trk;

            -----------------------------------------------------------------
            -- s6: palette lookup (line colour by luma zone) + sat products.
            -- Hue rotate handled at assembly via swap/negate of the products.
            -----------------------------------------------------------------
            v_idx := r5_idx;
            r6_paly <= PAL_Y(v_idx);
            -- saturation via shift-add scale of the chroma delta about mid.
            v_du := signed(resize(PAL_U(v_idx), 12)) - 512;
            v_dv := signed(resize(PAL_V(v_idx), 12)) - 512;
            case to_integer(s_satsel) is
                when 0 =>      v_su := (others => '0');                         -- grey
                               v_sv := (others => '0');
                when 1 =>      v_su := resize(shift_right(v_du,1), 14);         -- x0.5
                               v_sv := resize(shift_right(v_dv,1), 14);
                when 2 =>      v_su := resize(v_du, 14);                        -- x1
                               v_sv := resize(v_dv, 14);
                when 3 =>      v_su := resize(v_du,14) + resize(shift_right(v_du,1),14);  -- x1.5
                               v_sv := resize(v_dv,14) + resize(shift_right(v_dv,1),14);
                when 4 =>      v_su := shift_left(resize(v_du,14),1);           -- x2
                               v_sv := shift_left(resize(v_dv,14),1);
                when 5 =>      v_su := shift_left(resize(v_du,14),1) + resize(shift_right(v_du,1),14); -- x2.5
                               v_sv := shift_left(resize(v_dv,14),1) + resize(shift_right(v_dv,1),14);
                when 6 =>      v_su := shift_left(resize(v_du,14),1) + resize(v_du,14);  -- x3
                               v_sv := shift_left(resize(v_dv,14),1) + resize(v_dv,14);
                when others => v_su := shift_left(resize(v_du,14),2);          -- x4
                               v_sv := shift_left(resize(v_dv,14),2);
            end case;
            r6_su <= v_su; r6_sv <= v_sv;
            r6_line <= r5_line; r6_eye <= r5_eye; r6_halo <= r5_halo;
            r6_px <= r5_px; r6_py <= r5_py; r6_trk <= r5_trk;

            -----------------------------------------------------------------
            -- s7: hue rotation of the saturated palette chroma (90-deg steps).
            -----------------------------------------------------------------
            v_cu := to_signed(512, 13) + resize(r6_su, 13);
            v_cv := to_signed(512, 13) + resize(r6_sv, 13);
            case to_integer(s_hue) is
                when 1 =>
                    v_u := to_signed(512,13) - (v_cv - 512);
                    v_v := to_signed(512,13) + (v_cu - 512);
                when 2 =>
                    v_u := to_signed(512,13) - (v_cu - 512);
                    v_v := to_signed(512,13) - (v_cv - 512);
                when 3 =>
                    v_u := to_signed(512,13) + (v_cv - 512);
                    v_v := to_signed(512,13) - (v_cu - 512);
                when others =>
                    v_u := v_cu; v_v := v_cv;
            end case;
            r7_pu <= v_u; r7_pv <= v_v;
            r7_paly <= r6_paly;
            r7_line <= r6_line; r7_eye <= r6_eye; r7_halo <= r6_halo; r7_trk <= r6_trk;
            r7_px <= r6_px; r7_py <= r6_py;

            -----------------------------------------------------------------
            -- s8: colour select (eye solid / halo / contour line / black gap).
            -----------------------------------------------------------------
            if r7_eye = '1' then
                r8_y <= EYE_Y;  r8_u <= EYE_U; r8_v <= EYE_V;
            elsif r7_halo = '1' then
                r8_y <= HALO_Y; r8_u <= EYE_U; r8_v <= EYE_V;   -- icy halo, dim luma
            elsif r7_line = '1' then
                r8_y <= clamp10(signed(resize(r7_paly,13)) + signed(resize(s_bright,13)));
                r8_u <= clamp10(r7_pu); r8_v <= clamp10(r7_pv);
            else
                r8_y <= C_BLK;  r8_u <= C_MID; r8_v <= C_MID;    -- true black gap
            end if;
            r8_trk <= r7_trk; r8_px <= r7_px; r8_py <= r7_py;

            -----------------------------------------------------------------
            -- s9: analog degradation — fine luma dither only (scanline dimming
            -- removed: it striped solid regions into horizontal bars).
            -----------------------------------------------------------------
            v_dimy := signed(resize(r8_y,13));
            v_noise := signed(resize(lfsr(5 downto 0) and s_noisem, 8))
                     - signed(resize('0' & s_noisem(5 downto 1), 8));
            v_dimy := v_dimy + resize(v_noise, 13);
            r9_y <= clamp10(v_dimy);
            r9_u <= r8_u; r9_v <= r8_v;
            r9_px <= r8_px; r9_py <= r8_py;

            -----------------------------------------------------------------
            -- s10: window-cascade membership.  A rectangle test factorises into
            -- independent X-only and Y-only comparisons, so compute all per-window
            -- bits here (short parallel compares) and just AND/OR them next stage.
            -----------------------------------------------------------------
            r10_y <= r9_y; r10_u <= r9_u; r10_v <= r9_v;
            for k in 0 to N_WIN - 1 loop
                if (r9_px >= win_l(k)) and (r9_px <= win_r(k)) then
                    r10_xin(k) <= '1'; else r10_xin(k) <= '0'; end if;
                if (r9_px <= win_li(k)) or (r9_px >= win_ri(k)) then
                    r10_xstr(k) <= '1'; else r10_xstr(k) <= '0'; end if;
                if (resize(r9_py,12) >= win_t(k)) and (resize(r9_py,12) <= win_b(k)) then
                    r10_yin(k) <= '1'; else r10_yin(k) <= '0'; end if;
                if (resize(r9_py,12) <= win_ti(k)) or (resize(r9_py,12) >= win_bi(k)) then
                    r10_ystr(k) <= '1'; else r10_ystr(k) <= '0'; end if;
            end loop;

            -----------------------------------------------------------------
            -- s11: combine membership -> stroke, overlay, and feed chroma-delay.
            -- chroma sep: Y centred, U pulled ahead, V pushed back -> opposite
            -- cyan/magenta fringes.  Fixed total delay CHR_MAX.
            -----------------------------------------------------------------
            v_anystroke := false;
            if s_windows = '1' then
                for k in 0 to N_WIN - 1 loop
                    if r10_xin(k) = '1' and r10_yin(k) = '1'
                       and (r10_xstr(k) = '1' or r10_ystr(k) = '1') then
                        v_anystroke := true;
                    end if;
                end loop;
            end if;
            if v_anystroke then
                yd(0) <= WIN_Y; ud(0) <= WIN_U; vd(0) <= WIN_V;
            else
                yd(0) <= r10_y; ud(0) <= r10_u; vd(0) <= r10_v;
            end if;
            for i in 1 to 2*CHR_MAX loop
                yd(i) <= yd(i-1); ud(i) <= ud(i-1); vd(i) <= vd(i-1);
            end loop;

            -----------------------------------------------------------------
            -- output: bypass passes raw input (delayed to match LATENCY).
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

end architecture revenant;
