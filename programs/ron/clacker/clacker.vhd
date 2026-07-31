-- clacker.vhd  (v0.2 — video as a mechanical flip-disc wall)
--
-- v0.2: 1080i keeps discs round (field_n-toggle interlace detect, row pitch
-- halved in field lines, dy steps 2 with odd-field +1 interleave offset);
-- Clack top band = PERPETUAL (settled discs re-flip forever); refresh wave
-- sweeps top-down; square cards get left/right bevel.
--
-- The incoming video is re-rendered as a grid of discs (or square cards)
-- mounted on a dark backing panel.  Unlike a per-pixel shader, every disc HOLDS
-- ITS STATE between frames: when the video's value for a cell changes, that
-- disc physically SWINGS to its new position over the following frames — a
-- damped, overshooting settle.
--
-- The trick that makes it fit: a disc is EDGE-ON (invisible) at the instant its
-- state changes, so the face it was previously showing is never needed on
-- screen.  That means no per-cell history — the grid stores only the current
-- target value and a frame stamp.  age = frame_cnt - stamp indexes a 32-entry
-- swing-curve ROM; when the overshoot carries the disc past flat, the curve
-- folds and you briefly see the disc's black BACK at the peak of the bounce.
--
-- Nothing on the pixel path scales age by the Clack rate: phase is a pure
-- function of a 6-bit age, so a 64-entry age -> swing-curve table is rebuilt
-- once per frame in vblank by a plain accumulator (acc += spd, no multiply at
-- all).  The pixel path just reads CV_LUT(age).  Doing that arithmetic per
-- pixel instead cost a 7x10 multiply that the router smeared across the die and
-- it held HD Analog at 73.9 MHz.
--
-- Geometry: a plain square lattice, so exactly ONE cell is read per pixel.  The
-- shape test is an ellipse squashed vertically by the swing factor s:
--     dy^2 < s^2 * (r^2 - dx^2)            -- disc
--     dy^2 < s^2 * r^2  and  dx^2 < r^2    -- square card
-- both share one multiply; no divide anywhere in the program.  dy^2 is advanced
-- incrementally at each h edge (adds only), so no line-rate multiplier exists
-- for STA to time at the pixel clock.
--
-- Update path: row R's cells are sampled at row R's centre line, snooping the
-- pixel path's OWN grid read for the old state (the address is already exactly
-- {R, c} at the cell centre).  The result is stashed in a 64-word row buffer
-- and burst-committed into grid row R during the h-blank of row R+1's first
-- line — the raster is inside row R+1 by then, so nothing is written while it
-- is being displayed.  A Refresh mask gates which rows re-evaluate each frame,
-- giving a travelling update wave instead of a whole-wall jump.
--
-- Modes:
--   Colour   — every disc rests FLAT; its face carries the quantized video
--              colour (Levels luma steps x 8x8 chroma).
--   Luma Mod — every face is the same fixed dot colour; the video's luma sets
--              each disc's RESTING TILT ANGLE.  Bright = flat and catching the
--              light, dark = edge-on and nearly invisible against the panel.
--              Greyscale rendered by mechanical angle alone.
--
-- Controls:
--   K1  Disc Size   (cell pitch, smooth 34..162 px)
--   K2  Levels      (2..16 quantization steps: colour steps / tilt steps)
--   K3  Fill        (disc radius vs cell: gapped -> touching -> overfilled)
--   K4  Refresh     (travelling 16-frame update wave -> every row every frame)
--   K5  Tilt Shade  (0 = flat unlit, max = full edge-on blackout)
--   K6  Chroma      (chroma gain on the faces, 0.75x .. 2.7x)
--   S7  Shape       (Disc / Square card)
--   S8  Mode        (Colour / Luma Mod)
--   S9  Backing     (Black / dimmed live video behind the panel)
--   S10 Grid        (dark bezel lines between cells)
--   S11 Bypass
--   P12 Clack       (THE fader: 0 = instant snap to the new state, max = long
--                    lazy overshooting tumbles — the wall never stops moving)
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

architecture clacker of program_top is

    constant LATENCY : natural := 14;                  -- compute stages c1..c14

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Grid: 64 columns x 32 rows.  The address is a plain concatenation
    -- {row(5), col(6)} = 11 bits, so there is no row*COLS multiply anywhere.
    -- Min cell pitch is 34 px, which caps 1920x1080 at 56 x 31 cells.
    constant C_MAXCOL : natural := 64;
    constant C_MAXROW : natural := 32;

    -- Cell word: val(15..6) & stamp(5..0), val = y4 & u3 & v3.
    subtype t_cword is std_logic_vector(15 downto 0);

    ----------------------------------------------------------------------------
    -- Swing curve.  CV(phase) = 256 * (1 - e^(-t/5) * cos(2*pi*t/13)), the
    -- damped ring of a disc released edge-on and settling flat.  Values above
    -- 256 are an overshoot PAST flat: the squash folds back (512 - cv) and, once
    -- clear of C_FOLD_HYST, the disc has turned far enough to show its black
    -- back.  The hysteresis keeps the ~2% tail ripple (t=17..22) from flashing
    -- the back face for a squash change that is invisible.
    ----------------------------------------------------------------------------
    type t_cv is array (0 to 31) of unsigned(8 downto 0);
    constant CV_ROM : t_cv := (
        to_unsigned(  0, 9), to_unsigned( 70, 9), to_unsigned(159, 9), to_unsigned(239, 9),
        to_unsigned(297, 9), to_unsigned(326, 9), to_unsigned(331, 9), to_unsigned(317, 9),
        to_unsigned(295, 9), to_unsigned(271, 9), to_unsigned(252, 9), to_unsigned(240, 9),
        to_unsigned(235, 9), to_unsigned(237, 9), to_unsigned(242, 9), to_unsigned(249, 9),
        to_unsigned(255, 9), to_unsigned(259, 9), to_unsigned(261, 9), to_unsigned(262, 9),
        to_unsigned(261, 9), to_unsigned(259, 9), to_unsigned(257, 9), to_unsigned(256, 9),
        to_unsigned(255, 9), to_unsigned(254, 9), to_unsigned(255, 9), to_unsigned(255, 9),
        to_unsigned(255, 9), to_unsigned(256, 9), to_unsigned(256, 9), to_unsigned(256, 9));

    constant C_FOLD_HYST : natural := 280;             -- back face only past this

    -- Rest squash per luma level (Q8).  sqrt-shaped so apparent brightness
    -- (disc area and tilt shade, both linear in s) tracks the level linearly.
    type t_rs is array (0 to 15) of unsigned(8 downto 0);
    constant RS_ROM : t_rs := (
        to_unsigned( 13, 9), to_unsigned( 76, 9), to_unsigned(102, 9), to_unsigned(122, 9),
        to_unsigned(138, 9), to_unsigned(153, 9), to_unsigned(167, 9), to_unsigned(179, 9),
        to_unsigned(190, 9), to_unsigned(201, 9), to_unsigned(211, 9), to_unsigned(221, 9),
        to_unsigned(230, 9), to_unsigned(239, 9), to_unsigned(248, 9), to_unsigned(256, 9));

    -- Level-expand reciprocal: R15(N-2) = 3840/(N-1), so quantizing a 4-bit
    -- value to N levels and expanding it back is (q * R15) >> 8 — no divider.
    type t_r15 is array (0 to 14) of unsigned(11 downto 0);
    constant R15_ROM : t_r15 := (
        to_unsigned(3840, 12), to_unsigned(1920, 12), to_unsigned(1280, 12),
        to_unsigned( 960, 12), to_unsigned( 768, 12), to_unsigned( 640, 12),
        to_unsigned( 549, 12), to_unsigned( 480, 12), to_unsigned( 427, 12),
        to_unsigned( 384, 12), to_unsigned( 349, 12), to_unsigned( 320, 12),
        to_unsigned( 295, 12), to_unsigned( 274, 12), to_unsigned( 256, 12));

    -- Fixed colours.  Standard BT.601 (U = Cb, V = Cr) per the verified sim
    -- convention.  Dot = flip-dot fluorescent yellow-green (RGB ~230,230,60).
    constant C_DOT_Y  : unsigned(9 downto 0) := to_unsigned(845, 10);
    constant C_DOT_U  : unsigned(9 downto 0) := to_unsigned(171, 10);
    constant C_DOT_V  : unsigned(9 downto 0) := to_unsigned(568, 10);
    constant C_BACK_Y : unsigned(9 downto 0) := to_unsigned( 96, 10);   -- disc back
    constant C_PANEL  : unsigned(9 downto 0) := to_unsigned( 64, 10);   -- backing
    constant C_BEZEL  : unsigned(9 downto 0) := to_unsigned( 32, 10);   -- grid line

    ----------------------------------------------------------------------------
    -- Raster position.
    ----------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal r_hedge      : std_logic := '0';

    signal x_loc     : unsigned(7 downto 0) := (others => '0');
    signal x_idx     : unsigned(5 downto 0) := (others => '0');
    signal celly_loc : unsigned(7 downto 0) := (others => '0');
    signal celly_idx : unsigned(4 downto 0) := (others => '0');
    signal frame_act : std_logic := '0';
    signal fcnt      : unsigned(5 downto 0) := (others => '0');

    -- Interlace detect (videosky pattern): field_n toggling across vsyncs =
    -- interlaced.  Recomputed every vsync, so it self-clears on a mode change.
    signal s_fprev : std_logic := '0';
    signal s_ilace : std_logic := '0';
    signal s_fodd  : std_logic := '0';

    ----------------------------------------------------------------------------
    -- Per-frame parameters (latched at vsync, refined by the frame FSM).
    ----------------------------------------------------------------------------
    signal lk1, lk2, lk3, lk4, lk5, lk6, lp12 : unsigned(9 downto 0) := (others => '0');

    signal s_w     : unsigned(7 downto 0) := to_unsigned(81, 8);    -- cell pitch (px)
    signal s_wm1   : unsigned(7 downto 0) := to_unsigned(80, 8);
    signal s_wh    : unsigned(7 downto 0) := to_unsigned(40, 8);    -- W/2
    signal s_hh    : unsigned(7 downto 0) := to_unsigned(40, 8);    -- half-height, SCREEN lines
    -- Row pitch in FIELD lines: W progressive, W/2 interlaced (a 1080i field
    -- carries every other screen line, so W/2 field lines = W screen lines and
    -- the discs stay round instead of stretching 2:1).
    signal s_rp    : unsigned(7 downto 0) := to_unsigned(81, 8);
    signal s_rpm1  : unsigned(7 downto 0) := to_unsigned(80, 8);
    signal s_rc    : unsigned(7 downto 0) := to_unsigned(40, 8);    -- centre (sample) line
    signal s_rfrac : unsigned(10 downto 0) := to_unsigned(1000, 11);
    signal s_r     : unsigned(7 downto 0) := to_unsigned(36, 8);    -- disc radius
    signal s_r2    : unsigned(13 downto 0) := to_unsigned(1296, 14);
    signal s_nlev  : unsigned(4 downto 0) := to_unsigned(16, 5);    -- 2..16
    signal s_r15   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_spd   : unsigned(9 downto 0) := to_unsigned(80, 10);   -- clack rate
    signal s_span  : unsigned(7 downto 0) := to_unsigned(175, 8);   -- tilt shade
    signal s_amb   : unsigned(7 downto 0) := to_unsigned(80, 8);
    signal s_sg    : unsigned(7 downto 0) := to_unsigned(98, 8);    -- chroma gain
    signal s_rmask : unsigned(4 downto 0) := (others => '0');       -- refresh mask
    signal s_r2rim : unsigned(13 downto 0) := (others => '0');      -- r^2 * 7/8 (side bevel)
    signal s_perp  : std_logic := '0';                              -- P12 top band

    signal s_square : std_logic := '0';                -- S7
    signal s_lmod   : std_logic := '0';                -- S8
    signal s_bgvid  : std_logic := '0';                -- S9
    signal s_gridln : std_logic := '0';                -- S10
    signal s_bypass : std_logic := '0';                -- S11

    -- frame FSM shared multiplier
    signal fsm_t  : unsigned(3 downto 0) := (others => '1');
    signal ma, mb : signed(11 downto 0) := (others => '0');
    signal mp     : signed(23 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- age -> swing-curve table, rebuilt every frame in vblank.  phase(age) =
    -- min(31, ((age+1)*spd - 16) >> 4), and (age+1)*spd is just a running sum
    -- of spd — so the whole table costs 64 adds and no multiplier, and the pixel
    -- path is left with a single lookup.
    ----------------------------------------------------------------------------
    type t_cvlut is array (0 to 63) of std_logic_vector(8 downto 0);
    signal cvlut : t_cvlut := (others => (others => '0'));

    signal lg_start : std_logic := '0';
    signal lg_run   : std_logic := '0';
    signal lg_acc   : unsigned(15 downto 0) := (others => '0');
    signal lg_i     : unsigned(5 downto 0) := (others => '0');
    signal lg_wen   : std_logic := '0';
    signal lg_a     : unsigned(5 downto 0) := (others => '0');
    signal lg_d     : unsigned(8 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Per-line dy and dy^2.  dy advances by exactly +1 per line, so the square
    -- is updated incrementally at each h edge: (d+1)^2 = d^2 + 2(d+1) - 1.
    -- The band-wrap reset value (H/2)^2 is precomputed per frame.  dy^2 is a
    -- line constant, so the pixel path reads it directly — it settles during the
    -- back porch, long before the first active pixel.
    ----------------------------------------------------------------------------
    signal lm_dy : signed(8 downto 0) := (others => '0');
    signal dyq_r : unsigned(15 downto 0) := (others => '0');
    signal hh2q  : unsigned(15 downto 0) := (others => '0');     -- (H/2)^2
    signal hh2qo : unsigned(15 downto 0) := (others => '0');     -- (H/2 - 1)^2, odd field

    ----------------------------------------------------------------------------
    -- Cell grid (canonical 1W1R, registered read, pre-muxed write word) and the
    -- 64-word commit row buffer.
    ----------------------------------------------------------------------------
    -- Reset word: black face, NEUTRAL chroma, stamp 0.  Chroma code 0 is not
    -- neutral (it expands to Cb=Cr=0, i.e. saturated green), so an all-zero
    -- grid would show a green wall — at power-on, and again on any cell that
    -- Disc Size has just remapped and that has not been refreshed yet.  With
    -- u3=v3=4 an unwritten cell is a black disc on a black panel: invisible,
    -- so the wall simply fills itself in as the refresh wave reaches each row.
    constant C_CELL_RST : t_cword := "0000" & "100" & "100" & "000000";

    type t_grid is array (0 to C_MAXCOL * C_MAXROW - 1) of t_cword;
    signal grid  : t_grid := (others => C_CELL_RST);
    signal rd_w  : t_cword := (others => '0');
    signal gw_en : std_logic := '0';
    signal gw_a  : unsigned(10 downto 0) := (others => '0');
    signal gw_d  : t_cword := (others => '0');

    -- Reset to C_CELL_RST, NOT zeros: the commit burst copies this buffer into
    -- the grid wholesale, and an all-zero word renders as a saturated GREEN
    -- disc (chroma code 0 is not neutral).
    type t_rowbuf is array (0 to C_MAXCOL - 1) of t_cword;
    signal rowbuf : t_rowbuf := (others => C_CELL_RST);
    signal rb_rd  : t_cword := (others => '0');
    signal rb_ra  : unsigned(5 downto 0) := (others => '0');
    signal rb_hi  : unsigned(5 downto 0) := (others => '0');   -- last sampled column

    ----------------------------------------------------------------------------
    -- Sample / read-modify-write pipe (w0..w3).
    ----------------------------------------------------------------------------
    signal w0_stb : std_logic := '0';
    signal w0_idx : unsigned(5 downto 0) := (others => '0');
    signal w0_y, w0_u, w0_v : unsigned(9 downto 0) := C_MID;

    signal w1_stb : std_logic := '0';
    signal w1_idx : unsigned(5 downto 0) := (others => '0');
    signal w1_pu, w1_pv : signed(19 downto 0) := (others => '0');
    signal w1_pq : unsigned(8 downto 0) := (others => '0');

    signal w2_stb : std_logic := '0';
    signal w2_idx : unsigned(5 downto 0) := (others => '0');
    signal w2_old : t_cword := (others => '0');
    signal w2_py  : unsigned(15 downto 0) := (others => '0');
    signal w2_u3, w2_v3 : unsigned(2 downto 0) := (others => '0');

    signal w3_stb : std_logic := '0';
    signal w3_idx : unsigned(5 downto 0) := (others => '0');
    signal w3_val : std_logic_vector(9 downto 0) := (others => '0');
    signal w3_stm : unsigned(5 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Commit burst.
    ----------------------------------------------------------------------------
    signal pend      : std_logic := '0';
    signal pend_row  : unsigned(4 downto 0) := (others => '0');
    signal row_fresh : std_logic := '0';               -- this row re-evaluates
    signal bst_run   : std_logic := '0';
    signal bst_cnt   : unsigned(6 downto 0) := (others => '0');
    signal bst_row   : unsigned(4 downto 0) := (others => '0');
    signal bst_hi    : unsigned(5 downto 0) := (others => '0');
    signal bst_a1    : unsigned(5 downto 0) := (others => '0');
    signal bst_d1    : std_logic := '0';
    signal bst_d2    : std_logic := '0';

    ----------------------------------------------------------------------------
    -- Pixel pipeline (c1..c14).  pN_* is valid in cycle N+1.
    ----------------------------------------------------------------------------
    signal g0_dx   : signed(7 downto 0) := (others => '0');
    signal g0_addr : unsigned(10 downto 0) := (others => '0');
    signal g0_bez  : std_logic := '0';

    signal p2_dxq : unsigned(15 downto 0) := (others => '0');
    signal p2_bez : std_logic := '0';

    signal p3_word : t_cword := (others => '0');
    signal p3_dxq  : unsigned(15 downto 0) := (others => '0');
    signal p3_bez  : std_logic := '0';

    signal p4_age : unsigned(5 downto 0) := (others => '0');
    signal p4_t   : unsigned(13 downto 0) := (others => '0');

    signal p5_rs, p5_cv : unsigned(8 downto 0) := (others => '0');
    signal p6_sraw : unsigned(9 downto 0) := (others => '0');
    signal p7_s    : unsigned(8 downto 0) := (others => '0');
    signal p7_back : std_logic := '0';
    signal p8_s2   : unsigned(8 downto 0) := (others => '0');
    signal p8_shm  : unsigned(8 downto 0) := (others => '0');
    signal p9_hi   : unsigned(18 downto 0) := (others => '0');
    signal p9_lo   : unsigned(17 downto 0) := (others => '0');
    signal p9_sh   : unsigned(7 downto 0) := (others => '0');

    signal p10_m   : unsigned(13 downto 0) := (others => '0');
    signal p10_sh  : unsigned(7 downto 0) := (others => '0');

    signal p11_y   : unsigned(9 downto 0) := C_MID;
    signal p11_u, p11_v : unsigned(9 downto 0) := C_MID;
    signal p11_sh  : unsigned(7 downto 0) := (others => '0');
    signal p11_in, p11_rim : std_logic := '0';

    signal p12_ym  : unsigned(17 downto 0) := (others => '0');
    signal p12_u, p12_v : unsigned(9 downto 0) := C_MID;
    signal p12_sh  : unsigned(7 downto 0) := (others => '0');
    signal p12_in, p12_rim : std_logic := '0';

    signal p13_y   : unsigned(9 downto 0) := C_MID;
    signal p13_um, p13_vm : signed(19 downto 0) := (others => '0');
    signal p13_in, p13_rim : std_logic := '0';

    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := C_MID;

    ----------------------------------------------------------------------------
    -- Carry pipes.  t must arrive with p9_s2 (cycle 10); the cell value, the
    -- x-membership flag and the level index must arrive with p10_m (cycle 11).
    ----------------------------------------------------------------------------
    type t_t14 is array (natural range <>) of unsigned(13 downto 0);
    signal tp : t_t14(0 to 3) := (others => (others => '0'));       -- valid c6..c9

    type t_v10 is array (natural range <>) of std_logic_vector(9 downto 0);
    signal valp : t_v10(0 to 6) := (others => (others => '0'));     -- valid c5..c11

    signal inxp  : std_logic_vector(0 to 6) := (others => '0');     -- valid c5..c11
    signal xrimp : std_logic_vector(0 to 6) := (others => '0');     -- valid c5..c11
    signal backp : std_logic_vector(0 to 2) := (others => '0');     -- valid c9..c11
    signal bez_p : std_logic_vector(0 to 9) := (others => '0');     -- valid c5..c14

    ----------------------------------------------------------------------------
    -- Sync / video alignment pipe
    ----------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal s_io : t_video_stream_yuv444_30b;

begin

    ----------------------------------------------------------------------------
    -- Raster position counters + sample trigger.
    ----------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_trig : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            r_hedge <= v_h_edge;

            if v_v_edge = '1' then
                fcnt    <= fcnt + 1;
                s_fprev <= data_in.field_n;
                s_ilace <= data_in.field_n xor s_fprev;
                s_fodd  <= data_in.field_n;
            end if;

            -- X lattice
            if v_h_edge = '1' then
                x_loc <= (others => '0');
                x_idx <= (others => '0');
            elsif data_in.avid = '1' then
                if x_loc = s_wm1 then
                    x_loc <= (others => '0');
                    if x_idx /= C_MAXCOL - 1 then
                        x_idx <= x_idx + 1;
                    end if;
                else
                    x_loc <= x_loc + 1;
                end if;
            end if;

            -- Y row counter, anchored to the first active line of the field.
            -- Cells are square on SCREEN; the row pitch in field lines (s_rp)
            -- is halved when the source is interlaced.
            if v_v_edge = '1' then
                celly_loc <= (others => '0');
                celly_idx <= (others => '0');
                frame_act <= '0';
            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                celly_loc <= (others => '0');
                celly_idx <= (others => '0');
            elsif v_h_edge = '1' and frame_act = '1' then
                if celly_loc = s_rpm1 then
                    celly_loc <= (others => '0');
                    if celly_idx /= C_MAXROW - 1 then
                        celly_idx <= celly_idx + 1;
                    end if;
                else
                    celly_loc <= celly_loc + 1;
                end if;
            end if;

            -- Refresh gate: rows satisfying ((row - fcnt) and mask) = 0 are
            -- re-evaluated this frame.  With mask = 15 that is one row in 16,
            -- and the selected row INCREASES by one each frame, so the update
            -- wave sweeps TOP-DOWN like a real flip sign repainting.
            -- Gated on the REGISTERED h edge: celly_idx has been incremented to
            -- the new row by then, so this tests the row we are entering.
            if r_hedge = '1' then
                if ((celly_idx - fcnt(4 downto 0)) and s_rmask) = 0 then
                    row_fresh <= '1';
                else
                    row_fresh <= '0';
                end if;
            end if;

            -- Sample trigger: at row R's centre line, at each cell's centre
            -- column.  The pixel path's own grid read is already addressing
            -- {R, c} here, so the old cell word arrives with the w1 stage.
            v_trig := '0';
            if data_in.avid = '1' and frame_act = '1' and row_fresh = '1'
               and celly_loc = s_rc and x_loc = s_wh then
                v_trig := '1';
            end if;

            w0_stb <= v_trig;
            w0_idx <= x_idx;
            w0_y   <= unsigned(data_in.y);
            w0_u   <= unsigned(data_in.u);
            w0_v   <= unsigned(data_in.v);

            if v_trig = '1' then
                pend     <= '1';
                pend_row <= celly_idx;
            elsif bst_run = '1' then
                pend <= '0';
            end if;
        end if;
    end process p_position;

    ----------------------------------------------------------------------------
    -- Sample RMW: chroma gain, luma quantization to K2 levels, then decide
    -- whether the disc must start a new swing.
    ----------------------------------------------------------------------------
    p_sample : process(clk)
        variable v_u, v_vv : signed(11 downto 0);
        variable v_y4  : unsigned(3 downto 0);
        variable v_new : std_logic_vector(9 downto 0);
        variable v_age : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- w1: chroma gain, and quantize luma to one of N levels
            w1_pu <= (signed(resize(w0_u, 11)) - to_signed(512, 11))
                   * signed(resize(s_sg, 9));
            w1_pv <= (signed(resize(w0_v, 11)) - to_signed(512, 11))
                   * signed(resize(s_sg, 9));
            w1_pq  <= resize(shift_right(w0_y(9 downto 6) * s_nlev, 4), 9);
            w1_stb <= w0_stb;
            w1_idx <= w0_idx;

            -- w2: expand the level index back to a 4-bit face value.  In Luma
            -- Mod the stored chroma is forced to zero: it drives nothing there,
            -- and leaving it live would let invisible chroma changes trip the
            -- change detector and flip discs at random.
            w2_py <= resize(w1_pq(3 downto 0) * s_r15, 16);
            -- Chroma is signed about 512, so it is quantized to 8 steps of 128
            -- and expanded back by a plain shift (u3 * 128): code 4 is EXACTLY
            -- neutral.  Rounding first keeps near-neutral from falling into the
            -- bucket below and tinting every grey.  The cost is the top of the
            -- chroma range clipping at 896, which K6 rides into gracefully.
            v_u := to_signed(512 + 64, 12) + resize(shift_right(w1_pu, 6), 12);
            if v_u < 0 then v_u := (others => '0'); end if;
            if v_u > 1023 then v_u := to_signed(1023, 12); end if;
            v_vv := to_signed(512 + 64, 12) + resize(shift_right(w1_pv, 6), 12);
            if v_vv < 0 then v_vv := (others => '0'); end if;
            if v_vv > 1023 then v_vv := to_signed(1023, 12); end if;
            if s_lmod = '1' then
                -- Chroma drives nothing in Luma Mod, but store NEUTRAL (code
                -- 4 = 512), not zero: zero is saturated green, and it would
                -- both flash a green wall for a refresh period when S8 is
                -- toggled back to Colour, and make the toggle's change-detect
                -- fire from an invisible chroma difference.
                w2_u3 <= "100";
                w2_v3 <= "100";
            else
                w2_u3 <= unsigned(v_u(9 downto 7));
                w2_v3 <= unsigned(v_vv(9 downto 7));
            end if;
            w2_stb <= w1_stb;
            w2_idx <= w1_idx;
            -- rd_w is the registered grid read of the address the pixel path
            -- latched one cycle ago — which at the cell centre is exactly this
            -- cell.  Captured here rather than at w1 so it lands in the same
            -- cycle as w2_py, with no reliance on the neighbouring pixel
            -- happening to share the address.
            w2_old <= rd_w;

            ------------------------------------------------------------------
            -- w3: changed -> restamp (a new swing starts).  Unchanged -> keep
            -- the old stamp so an in-flight swing carries on, but once it has
            -- settled (age >= 47) re-pin the stamp, so age cannot run past the
            -- 6-bit wrap before the next refresh (<= 16 frames away) and fake a
            -- spurious re-flip.
            ------------------------------------------------------------------
            if w2_py(15 downto 12) /= "0000" then
                v_y4 := (others => '1');
            else
                v_y4 := w2_py(11 downto 8);
            end if;
            v_new := std_logic_vector(v_y4) & std_logic_vector(w2_u3)
                   & std_logic_vector(w2_v3);
            v_age := fcnt - unsigned(w2_old(5 downto 0));

            w3_val <= v_new;
            if v_new /= w2_old(15 downto 6) then
                w3_stm <= fcnt;
            elsif v_age >= 47 then
                -- Settled.  Normally pin age at 47 (guards the 6-bit wrap);
                -- with Clack in its top band, re-flip instead — same value,
                -- full swing — so the wall tumbles forever even on a still.
                if s_perp = '1' then
                    w3_stm <= fcnt;
                else
                    w3_stm <= fcnt - 47;
                end if;
            else
                w3_stm <= unsigned(w2_old(5 downto 0));
            end if;
            w3_stb <= w2_stb;
            w3_idx <= w2_idx;
        end if;
    end process p_sample;

    ----------------------------------------------------------------------------
    -- Row buffer (1W1R): holds row R's new words until the commit burst.
    ----------------------------------------------------------------------------
    p_rowbuf : process(clk)
    begin
        if rising_edge(clk) then
            if w3_stb = '1' then
                rowbuf(to_integer(w3_idx)) <= w3_val & std_logic_vector(w3_stm);
            end if;
            rb_rd <= rowbuf(to_integer(rb_ra));
        end if;
    end process p_rowbuf;

    ----------------------------------------------------------------------------
    -- Commit burst: walk the row buffer into grid row R during the h-blank of
    -- the next row's first line (or of the first blanking line of the frame,
    -- which catches a partial bottom row).  The raster has left row R by then,
    -- so the pixel path is never reading the row being written.
    ----------------------------------------------------------------------------
    p_commit : process(clk)
    begin
        if rising_edge(clk) then
            -- Highest column sampled into the row buffer this row.  Samples
            -- ascend from column 0, so a plain overwrite tracks the maximum;
            -- the burst captures it before the next row starts sampling.
            if w3_stb = '1' then
                rb_hi <= w3_idx;
            end if;

            if bst_run = '0' then
                bst_d1 <= '0';
                bst_d2 <= '0';
                gw_en  <= '0';
                if r_hedge = '1' and pend = '1'
                   and (celly_loc = 0 or data_in.vsync_n = '0') then
                    bst_run <= '1';
                    bst_cnt <= (others => '0');
                    bst_row <= pend_row;
                    bst_hi  <= rb_hi;
                end if;
            else
                -- read address -> rb_rd (1 cycle) -> grid write (1 cycle)
                rb_ra  <= bst_cnt(5 downto 0);
                bst_a1 <= rb_ra;
                if bst_cnt < C_MAXCOL then
                    bst_d1 <= '1';
                else
                    bst_d1 <= '0';
                end if;
                bst_d2 <= bst_d1;

                gw_en <= bst_d2;
                gw_a  <= bst_row & bst_a1;
                -- Columns beyond the last sampled cell (a partial right-edge
                -- cell whose CENTRE column is off-screen never samples) get
                -- the neutral reset word, not whatever the buffer holds —
                -- otherwise green/stale half-discs appear at the right edge,
                -- coming and going with Disc Size.
                if bst_a1 <= bst_hi then
                    gw_d <= rb_rd;
                else
                    gw_d <= C_CELL_RST;
                end if;

                if bst_cnt = C_MAXCOL + 3 then
                    bst_run <= '0';
                else
                    bst_cnt <= bst_cnt + 1;
                end if;
            end if;
        end if;
    end process p_commit;

    ----------------------------------------------------------------------------
    -- Swing-curve table generator.  Runs 64 cycles in vblank, long before any
    -- active pixel can read the table.
    ----------------------------------------------------------------------------
    p_lutgen : process(clk)
        variable v_ph : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            lg_wen <= '0';
            if lg_run = '1' then
                v_ph := resize(shift_right(lg_acc - 16, 4), 12);
                if v_ph > 31 then
                    lg_d <= CV_ROM(31);
                else
                    lg_d <= CV_ROM(to_integer(v_ph(4 downto 0)));
                end if;
                lg_a   <= lg_i;
                lg_wen <= '1';
                lg_acc <= lg_acc + resize(s_spd, 16);
                if lg_i = 63 then
                    lg_run <= '0';
                else
                    lg_i <= lg_i + 1;
                end if;
            elsif lg_start = '1' then
                lg_acc <= resize(s_spd, 16);       -- age 0 -> 1 * spd
                lg_i   <= (others => '0');
                lg_run <= '1';
            end if;
        end if;
    end process p_lutgen;

    p_cvlut : process(clk)
    begin
        if rising_edge(clk) then
            if lg_wen = '1' then
                cvlut(to_integer(lg_a)) <= std_logic_vector(lg_d);
            end if;
            p5_cv <= unsigned(cvlut(to_integer(p4_age)));
        end if;
    end process p_cvlut;

    ----------------------------------------------------------------------------
    -- Cell grid (canonical 1W1R, registered read).
    ----------------------------------------------------------------------------
    p_grid : process(clk)
    begin
        if rising_edge(clk) then
            if gw_en = '1' then
                grid(to_integer(gw_a)) <= gw_d;
            end if;
            rd_w <= grid(to_integer(g0_addr));
        end if;
    end process p_grid;

    ----------------------------------------------------------------------------
    -- Per-line dy^2, updated incrementally (adds only) after each h edge.
    ----------------------------------------------------------------------------
    p_line : process(clk)
        -- sq(d+1) = sq(d) + 2*dnew - 1
        function upd1(sq : unsigned(15 downto 0); dn : signed(8 downto 0))
            return unsigned is
            variable v : signed(17 downto 0);
        begin
            v := signed(resize(sq, 18)) + resize(dn & '0', 18) - 1;
            return unsigned(v(15 downto 0));
        end function;
        -- sq(d+2) = sq(d) + 4*dnew - 4  (interlaced: dy steps 2 screen lines
        -- per field line — still adds only, no multiplier for STA to time)
        function upd2(sq : unsigned(15 downto 0); dn : signed(8 downto 0))
            return unsigned is
            variable v : signed(17 downto 0);
        begin
            v := signed(resize(sq, 18)) + resize(dn & "00", 18) - 4;
            return unsigned(v(15 downto 0));
        end function;
    begin
        if rising_edge(clk) then
            if r_hedge = '1' then
                if celly_loc = 0 then
                    -- band wrap.  Odd interlaced fields start one SCREEN line
                    -- down, so the two fields interleave into one round disc.
                    if s_ilace = '1' and s_fodd = '1' then
                        lm_dy <= -signed(resize(s_hh, 9)) + 1;
                        dyq_r <= hh2qo;
                    else
                        lm_dy <= -signed(resize(s_hh, 9));
                        dyq_r <= hh2q;
                    end if;
                elsif s_ilace = '1' then
                    lm_dy <= lm_dy + 2;
                    dyq_r <= upd2(dyq_r, lm_dy + 2);
                else
                    lm_dy <= lm_dy + 1;
                    dyq_r <= upd1(dyq_r, lm_dy + 1);
                end if;
            end if;
        end if;
    end process p_line;

    ----------------------------------------------------------------------------
    -- Frame FSM: knob shaping.  No divider; the three products run on one
    -- shared registered multiplier so nothing combinational feeds a latch.
    ----------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_w : integer range 0 to 511;
        variable v_n : integer range 0 to 31;
    begin
        if rising_edge(clk) then
            mp <= ma * mb;
            lg_start <= '0';

            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                lk1  <= unsigned(registers_in(0)(9 downto 0));
                lk2  <= unsigned(registers_in(1)(9 downto 0));
                lk3  <= unsigned(registers_in(2)(9 downto 0));
                lk4  <= unsigned(registers_in(3)(9 downto 0));
                lk5  <= unsigned(registers_in(4)(9 downto 0));
                lk6  <= unsigned(registers_in(5)(9 downto 0));
                lp12 <= unsigned(registers_in(7)(9 downto 0));
                s_square <= registers_in(6)(0);
                s_lmod   <= registers_in(6)(1);
                s_bgvid  <= registers_in(6)(2);
                s_gridln <= registers_in(6)(3);
                s_bypass <= registers_in(6)(4);
                fsm_t <= (others => '0');
            elsif fsm_t /= x"F" then
                fsm_t <= fsm_t + 1;

                case to_integer(fsm_t) is
                    when 0 =>
                        v_w := 34 + to_integer(lk1(9 downto 3));
                        if v_w > 162 then v_w := 162; end if;
                        s_w <= to_unsigned(v_w, 8);

                        -- Levels: 2..16
                        v_n := 2 + to_integer(lk2(9 downto 6));
                        if v_n > 16 then v_n := 16; end if;
                        s_nlev <= to_unsigned(v_n, 5);

                        -- Fill: r/W = 0.25 .. 0.56
                        s_rfrac <= resize(to_unsigned(512, 11) + lk3(9 downto 1)
                                        + lk3(9 downto 3), 11);

                        -- Tilt Shade: span 0..255, the ambient floor takes the rest
                        s_span <= lk5(9 downto 2);
                        s_amb  <= 255 - lk5(9 downto 2);

                        s_sg <= resize(to_unsigned(48, 8) + lk6(9 downto 3), 8);

                        -- Clack: spd 527 (snap) .. 16 (31-frame lazy tumble).
                        -- Top ~6% of travel = PERPETUAL: settled discs re-flip
                        -- instead of resting, so the wall never stops moving.
                        s_spd <= resize(to_unsigned(16, 10)
                                      + shift_right(1023 - lp12, 1), 10);
                        if lp12(9 downto 6) = "1111" then
                            s_perp <= '1';
                        else
                            s_perp <= '0';
                        end if;
                    when 1 =>
                        s_wm1 <= s_w - 1;
                        s_wh  <= '0' & s_w(7 downto 1);
                        s_r15 <= R15_ROM(to_integer(s_nlev) - 2);

                        -- Refresh: mask 0 (every row every frame) .. 15
                        case to_integer(lk4(9 downto 8)) is
                            when 3      => s_rmask <= to_unsigned( 0, 5);
                            when 2      => s_rmask <= to_unsigned( 1, 5);
                            when 1      => s_rmask <= to_unsigned( 3, 5);
                            when others => s_rmask <= to_unsigned(15, 5);
                        end case;

                        ma <= signed(resize(s_w, 12));
                        mb <= signed(resize(s_rfrac, 12));

                        -- row pitch in field lines: W/2 when interlaced
                        if s_ilace = '1' then
                            s_rp <= '0' & s_w(7 downto 1);
                        else
                            s_rp <= s_w;
                        end if;

                        -- s_spd settled at t=0; rebuild the swing-curve table
                        lg_start <= '1';
                    when 2 =>
                        s_hh   <= s_wh;
                        s_rpm1 <= s_rp - 1;
                        s_rc   <= '0' & s_rp(7 downto 1);
                    when 4 =>
                        -- r = W * rfrac / 2048
                        if mp(18 downto 11) < 3 then
                            s_r <= to_unsigned(3, 8);
                        else
                            s_r <= unsigned(mp(18 downto 11));
                        end if;
                        ma <= signed(resize(s_hh, 12));
                        mb <= signed(resize(s_hh, 12));
                    when 7 =>
                        hh2q <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_r, 12));
                        mb <= signed(resize(s_r, 12));
                    when 8 =>
                        -- (H/2 - 1)^2 = (H/2)^2 - H + 1, adds only
                        hh2qo <= hh2q - resize(s_hh & '0', 16) + 1;
                    when 10 =>
                        s_r2 <= unsigned(mp(13 downto 0));
                    when 11 =>
                        -- side-bevel threshold: r^2 * 7/8
                        s_r2rim <= s_r2 - ("000" & s_r2(13 downto 3));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ----------------------------------------------------------------------------
    -- Pixel pipeline.
    ----------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_word : t_cword;
        variable v_t    : signed(15 downto 0);
        variable v_sh   : unsigned(9 downto 0);
        variable v_lim  : unsigned(13 downto 0);
        variable v_dq   : unsigned(13 downto 0);
        variable v_su   : signed(11 downto 0);
        variable v_lev  : unsigned(3 downto 0);
        variable v_u3, v_v3 : unsigned(2 downto 0);
        variable v_fy, v_fu, v_fv : unsigned(9 downto 0);
        variable v_bgy, v_bgu, v_bgv : unsigned(9 downto 0);
        variable v_oy, v_ou, v_ov : unsigned(9 downto 0);
        variable v_ry : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            ------------------------------------------------------------------
            -- c1: local dx, grid address, bezel flag.
            ------------------------------------------------------------------
            g0_dx   <= signed(resize(x_loc, 8)) - signed(resize(s_wh, 8));
            g0_addr <= celly_idx & x_idx;
            if x_loc = 0 or x_loc = s_wm1 or celly_loc = 0 or celly_loc = s_rpm1 then
                g0_bez <= '1';
            else
                g0_bez <= '0';
            end if;

            -- c2: dx^2  (the grid read of g0_addr lands in rd_w this cycle)
            p2_dxq <= resize(unsigned(g0_dx * g0_dx), 16);
            p2_bez <= g0_bez;

            -- c3: capture the cell word
            p3_word <= rd_w;
            p3_dxq  <= p2_dxq;
            p3_bez  <= p2_bez;

            ------------------------------------------------------------------
            -- c4: unpack the cell, age it, and form the ellipse's x term.
            -- Square cards use t = r^2 (flat top and bottom edges); discs use
            -- t = r^2 - dx^2.  Both then test dy^2 * 256 < s^2 * t.
            ------------------------------------------------------------------
            v_word := p3_word;
            valp(0) <= v_word(15 downto 6);
            p4_age  <= fcnt - unsigned(v_word(5 downto 0));

            if s_square = '1' then
                p4_t <= s_r2;
            else
                v_t := signed(resize(s_r2, 16)) - signed(resize(p3_dxq, 16));
                if v_t < 0 then v_t := (others => '0'); end if;
                p4_t <= unsigned(v_t(13 downto 0));
            end if;
            if p3_dxq < s_r2 then
                inxp(0) <= '1';
            else
                inxp(0) <= '0';
            end if;
            -- side-bevel band: outer r^2/8 of the x extent (square cards get
            -- their left/right bezel from this; discs already get side rim
            -- from the ellipse's dy test as the limit curves to zero)
            if p3_dxq >= s_r2rim then
                xrimp(0) <= '1';
            else
                xrimp(0) <= '0';
            end if;

            ------------------------------------------------------------------
            -- c5: swing-curve + rest-squash lookups.  The curve table already
            -- folds in the Clack rate.  In Colour mode every disc rests flat,
            -- so the rest index is forced to full.  (p5_cv is read from the
            -- table in p_cvlut.)
            ------------------------------------------------------------------
            if s_lmod = '1' then
                p5_rs <= RS_ROM(to_integer(unsigned(valp(0)(9 downto 6))));
            else
                p5_rs <= RS_ROM(15);
            end if;

            -- c6: squash before the fold
            p6_sraw <= resize(shift_right(p5_rs * p5_cv, 8), 10);

            ------------------------------------------------------------------
            -- c7: fold.  Past flat the disc keeps turning, so the squash comes
            -- back down (512 - s) and, once clear of the hysteresis band, the
            -- black back of the disc comes into view.
            ------------------------------------------------------------------
            if p6_sraw > 256 then
                p7_s <= resize(to_unsigned(512, 10) - p6_sraw, 9);
            else
                p7_s <= p6_sraw(8 downto 0);
            end if;
            if p6_sraw > C_FOLD_HYST then
                p7_back <= '1';
            else
                p7_back <= '0';
            end if;

            ------------------------------------------------------------------
            -- c8: s^2 for the shape test, and the tilt-shade PRODUCT only.
            -- Folding the ambient add and its clamp in here too put a multiply,
            -- an add and a compare in series and held HD Analog at 71.7 MHz —
            -- the add moves to c9, where it sits parallel to a multiply rather
            -- than behind one.
            ------------------------------------------------------------------
            p8_s2  <= resize(shift_right(p7_s * p7_s, 8), 9);
            p8_shm <= resize(shift_right(p7_s * s_span, 8), 9);

            ------------------------------------------------------------------
            -- c9: the shape limit s2*t, as two HALF-WIDTH products.  As one
            -- 9x14 multiply, s2 fanned out across the whole partial-product
            -- tree and the router smeared it over the die — 9.6 ns of routing
            -- on a 3.2 ns logic path, holding HD Analog at 73.6 MHz.  Splitting
            -- s2 into nibbles halves each tree and each fanout; c10 recombines.
            --
            -- The shade sum cannot actually exceed 255 (amb = 255 - span and
            -- s <= 256 after the fold); the clamp is a guard, and free here
            -- because it sits parallel to the multiplies rather than behind one.
            ------------------------------------------------------------------
            p9_hi <= p8_s2(8 downto 4) * tp(3);
            p9_lo <= p8_s2(3 downto 0) * tp(3);
            v_sh  := resize(s_amb, 10) + resize(p8_shm, 10);
            if v_sh > 255 then v_sh := to_unsigned(255, 10); end if;
            p9_sh <= v_sh(7 downto 0);

            ------------------------------------------------------------------
            -- c10: recombine, and carry the limit as m/256 rather than m.
            -- dy^2 is an integer, so testing dy^2 < m/256 instead of
            -- dy^2*256 < m costs at most one unit of dy^2 (a sub-pixel shift of
            -- the disc edge) and leaves the c11 compares 14 bits wide, not 23.
            ------------------------------------------------------------------
            p10_m <= resize(shift_right(shift_left(resize(p9_hi, 23), 4)
                                      + resize(p9_lo, 23), 8), 14);
            p10_sh <= p9_sh;

            ------------------------------------------------------------------
            -- c11: inside / rim tests, and pick the face colour.  The rim band
            -- is a fixed FRACTION of the limit, so it tracks the squashed edge
            -- for free — no per-frame rim constant.  dy^2 is a line constant
            -- and is read straight from the line register.
            ------------------------------------------------------------------
            v_dq  := resize(dyq_r, 14);
            v_lim := p10_m;
            if v_dq < v_lim and inxp(6) = '1' then
                p11_in <= '1';
                if v_dq >= v_lim - shift_right(v_lim, 3)
                   or (s_square = '1' and xrimp(6) = '1') then
                    p11_rim <= '1';
                else
                    p11_rim <= '0';
                end if;
            else
                p11_in  <= '0';
                p11_rim <= '0';
            end if;

            v_lev := unsigned(valp(6)(9 downto 6));
            v_u3  := unsigned(valp(6)(5 downto 3));
            v_v3  := unsigned(valp(6)(2 downto 0));
            if backp(2) = '1' then
                v_fy := C_BACK_Y;  v_fu := C_MID;    v_fv := C_MID;
            elsif s_lmod = '1' then
                v_fy := C_DOT_Y;   v_fu := C_DOT_U;  v_fv := C_DOT_V;
            else
                -- luma is unipolar, so bit-replication expands it exactly
                -- (0 -> 0, 15 -> 1023); chroma is a signed step (see w2).
                v_fy := v_lev & v_lev & v_lev(3 downto 2);
                v_fu := v_u3 & "0000000";
                v_fv := v_v3 & "0000000";
            end if;
            p11_y  <= v_fy;
            p11_u  <= v_fu;
            p11_v  <= v_fv;
            p11_sh <= p10_sh;

            -- c12: face luma * tilt shade
            p12_ym  <= resize(p11_y * ('0' & p11_sh), 18);
            p12_u   <= p11_u;
            p12_v   <= p11_v;
            p12_sh  <= p11_sh;
            p12_in  <= p11_in;
            p12_rim <= p11_rim;

            -- c13: chroma * the same shade (a tilting disc darkens AND desaturates)
            p13_um <= (signed(resize(p12_u, 11)) - to_signed(512, 11))
                    * signed(resize(p12_sh, 9));
            p13_vm <= (signed(resize(p12_v, 11)) - to_signed(512, 11))
                    * signed(resize(p12_sh, 9));
            p13_y   <= p12_ym(17 downto 8);
            p13_in  <= p12_in;
            p13_rim <= p12_rim;

            ------------------------------------------------------------------
            -- c14: compose the disc over the backing panel.
            ------------------------------------------------------------------
            v_su := to_signed(512, 12) + resize(shift_right(p13_um, 8), 12);
            v_ou := unsigned(v_su(9 downto 0));
            v_su := to_signed(512, 12) + resize(shift_right(p13_vm, 8), 12);
            v_ov := unsigned(v_su(9 downto 0));
            v_oy := p13_y;

            if s_bgvid = '1' then
                v_bgy := resize(unsigned(pipe(12).y(9 downto 1)), 10) + 32;
                v_bgu := resize(unsigned(pipe(12).u(9 downto 1)), 10) + 256;
                v_bgv := resize(unsigned(pipe(12).v(9 downto 1)), 10) + 256;
            else
                v_bgy := C_PANEL;
                v_bgu := C_MID;
                v_bgv := C_MID;
            end if;

            if p13_in = '1' then
                if p13_rim = '1' then
                    -- lit metal bevel around the disc edge
                    v_ry := resize(v_oy, 11) + 220;
                    if v_ry > 1023 then v_ry := to_unsigned(1023, 11); end if;
                    v_oy := v_ry(9 downto 0);
                    v_ou := ('0' & v_ou(9 downto 1)) + 256;
                    v_ov := ('0' & v_ov(9 downto 1)) + 256;
                end if;
            else
                v_oy := v_bgy;  v_ou := v_bgu;  v_ov := v_bgv;
            end if;

            if s_gridln = '1' and bez_p(9) = '1' then
                v_oy := C_BEZEL;  v_ou := C_MID;  v_ov := C_MID;
            end if;

            p14_y <= v_oy;
            p14_u <= v_ou;
            p14_v <= v_ov;

            ------------------------------------------------------------------
            -- output: bypass mux + sync attach
            ------------------------------------------------------------------
            if s_bypass = '1' then
                s_io.y <= pipe(LATENCY - 1).y;
                s_io.u <= pipe(LATENCY - 1).u;
                s_io.v <= pipe(LATENCY - 1).v;
            else
                s_io.y <= std_logic_vector(p14_y);
                s_io.u <= std_logic_vector(p14_u);
                s_io.v <= std_logic_vector(p14_v);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;

            ------------------------------------------------------------------
            -- carry pipes
            ------------------------------------------------------------------
            tp(0) <= p4_t;
            for i in 1 to 3 loop tp(i) <= tp(i - 1); end loop;
            for i in 1 to 6 loop valp(i) <= valp(i - 1); end loop;
            for i in 1 to 6 loop inxp(i) <= inxp(i - 1); end loop;
            for i in 1 to 6 loop xrimp(i) <= xrimp(i - 1); end loop;
            backp(0) <= p7_back;
            for i in 1 to 2 loop backp(i) <= backp(i - 1); end loop;
            bez_p(0) <= p3_bez;
            for i in 1 to 9 loop bez_p(i) <= bez_p(i - 1); end loop;
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture clacker;
