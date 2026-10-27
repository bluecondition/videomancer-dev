-- haywire.vhd  (v0.3.0 — analog glitch-box emulation)
--
-- Makes live video look as if it went through a circuit-bent analog glitch
-- box (the Tachyons+ DreamWeapon / Psychenizer / Mezkalin class).  Their demo
-- stills show three signatures, and this program builds each one in the
-- signal path rather than as an overlay:
--
--   LINES THAT RADIATE FROM EDGES.  A per-scanline oscillator is HARD-SYNCED
--   by the picture.  Whenever the low-passed luma has moved more than the
--   Trigger step since the last sync (a delta comparator, the way the boxes'
--   comparators lock onto video), the phase resets.  The oscillator's wrap
--   points are then thick lines at fixed distances after every contour:
--   echoes that copy each contour's shape, spaced apart, fading with
--   distance, in the colour of the edge's bright side (a falling edge
--   echoes the brightest colour since the last sync, a rising edge the
--   brightest of the first pixels after it).  The oscillator is
--   frequency-modulated by luma (Bend), so the echoes bunch and swerve
--   through bright areas and snake down the picture; Chaos random-walks it
--   from a column-keyed generator (the same on every row) and drops and
--   fattens echoes per 8-line segment, so the lines wander and break but
--   stay lines.
--
--   HOLD / MOVE.  The echoes do not crawl like a free-running oscillator:
--   every random that places a line (column pattern, segment dropouts and
--   widths, per-line phase scatter and jitter, the wobble) is restarted
--   from a seed each frame, so the picture holds dead still.  At random
--   moments (Chaos sets how often: never at 0, every ~2 s at the default,
--   several times a second at the top) the seeds snap to new values and the
--   lines slide for 2..17 frames at a random speed and direction (pattern
--   crawl, a global phase slip, the wobble advancing), then freeze again.  Soft sync keeps the phase
--   free-running: the contours only light the lines, which snake through
--   the whole picture on luma alone.
--
--   OVERDRIVEN, SNOWY PEAKS.  A luma gain stage (Drive, 0.25x .. 4x, unity
--   at the centre).  Where the boosted signal passes the knee the peak clips
--   and breaks up into 4-px dashes of white/black snow whose density follows
--   the overdrive.
--
--   INSTABILITY (P12, the headline control, all continuous): per-line sync
--   jitter (picture and echoes shake), a travelling wobble of the oscillator
--   phase (the lines snake in time), a rolling hum bar, random torn bands
--   (displaced, sheared, sync lost, hue flipped, snowy), lines that drop
--   sync, chroma slipping off the luma, colour drift and a snow floor.
--
-- Controls:
--   K1 Drive    luma gain 0.25x .. 4x (50% = unchanged); peaks blow out into snow
--   K2 Trigger  sync sensitivity: how small a luma change launches an echo train
--   K3 Spacing  echo spacing, ~6 .. 160 px at HD
--   K4 Ring     how far the echo trains radiate, ~24 px .. full width
--   K5 Bend     luma FM depth: echoes bunch and swerve through bright areas
--   K6 Chaos    how often the lines jump and slide (still in between), and
--               how much they wander, break up and vary in thickness
--   S7 Colour   Natural / Psyche (luma-to-rainbow colourizer)
--   S8 Picture  Full / Dim (picture at a quarter, the lines dominate)
--   S9 Sync     Hard (echoes copy the contours) / Soft (free-running FM lines)
--   S10 Smear   Off / On (bright areas streak to the right in their colour)
--   S11 Invert  Off / On (video negative before the box)
--   P12 Instability
--
-- Structure (all streaming, no frame store):
--   * Input video is written into EBR rings every clock (neutral in blanking).
--     Ring A (8-bit luma) feeds the trigger/oscillator path; rings Y and UV
--     feed the picture path KD = 5 clocks later.  Both read the same per-line
--     displacement (the instability jitter and tears, rightward only) because
--     the Y/UV read address IS ring A's address delayed KD clocks; UV reads
--     cs px further back (chroma slip).
--   * Knobs run through one time-shared control engine (deadband, glide,
--     curve ROM in EBR, one multiplier) instead of per-knob hardware.
--   * Syncs ride a separate EBR ring at a fixed SYNC_D, so the program has one
--     latency in every mode (22 clocks).
--   * Output: Psyche/natural colour, line composite, snow, then the HD HDMI
--     bar guard ([1 2 1] on Y, U and V, luma scaled to white ~775)
--     and a neutral blanking gate on a generated, hsync-parity-locked avid.
--
-- Generated colours (Psyche palette, snow tint) are authored in HARDWARE sense: the U register carries Cr, the V register carries Cb.  The
-- natural video path is identity.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture haywire of program_top is

    constant C_BLK    : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID    : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- Ring Y/UV read address trails ring A's by KD clocks (the depth of the
    -- line-render path minus the picture path up to the composite).
    constant KD       : natural := 5;
    -- Sync ring read offset: data_out is 22 clocks behind data_in.
    constant SYNC_D   : natural := 19;
    constant C_KNEE   : integer := 700;   -- drive level where peaks start
    constant C_YMAX   : integer := 940;
    constant C_INCMAX : integer := 24576; -- oscillator step cap (2.67 px period)
    constant C_TDEC   : integer := 2;     -- smear decay per pixel
    constant C_AMP0   : integer := 98303; -- train envelope start, 1.5 (full for 1/3)
    constant C_DMAXPX : integer := 480;   -- displacement cap (ring is 512)

    -- ========================================================================
    -- Knob curves for the control engine, one EBR: address = slot & point,
    -- 17 points per slot, interpolated on knob(5:0).
    --   0 Drive gain x64 (0.25 .. 4, unity at the centre)
    --   1 Trigger step, 8-bit luma (160 .. 8; knob up = more sensitive)
    --   2 Spacing: oscillator step for a 6 .. 160 px period
    --   3 Ring: envelope decay per pixel (trains 24 .. 4096 px long)
    --   4 Bend: FM depth x64 (0 .. 4, square law)
    --   5 (no curve: this slot computes fb = f0 * bend / 64)
    --   6 slider squared, 7 slider cubed
    -- ========================================================================
    type t_crom is array (0 to 255) of std_logic_vector(15 downto 0);
    constant C_CROM : t_crom := (
        x"0010", x"0013", x"0017", x"001B", x"0020", x"0026", x"002D", x"0036",
        x"0040", x"004C", x"005B", x"006C", x"0080", x"0098", x"00B5", x"00D7",
        x"0100", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"00A0", x"0085", x"006E", x"005B", x"004C", x"003F", x"0034", x"002B",
        x"0024", x"001E", x"0019", x"0014", x"0011", x"000E", x"000C", x"000A",
        x"0008", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"2AAB", x"22C0", x"1C4E", x"170D", x"12C7", x"0F4B", x"0C75", x"0A25",
        x"0843", x"06BB", x"057B", x"0477", x"03A3", x"02F6", x"0269", x"01F7",
        x"019A", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0AAB", x"07BC", x"059C", x"0412", x"02F3", x"0224", x"018D", x"0120",
        x"00D1", x"0098", x"006E", x"0050", x"003A", x"002A", x"001E", x"0016",
        x"0010", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0001", x"0004", x"0009", x"0010", x"0019", x"0024", x"0031",
        x"0040", x"0051", x"0064", x"0079", x"0090", x"00A9", x"00C4", x"00E1",
        x"00FF", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0004", x"0010", x"0024", x"0040", x"0064", x"0090", x"00C4",
        x"0100", x"0144", x"0190", x"01E4", x"023F", x"02A3", x"030F", x"0383",
        x"03FF", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0002", x"0007", x"0010", x"001F", x"0036", x"0056",
        x"0080", x"00B6", x"00FA", x"014C", x"01B0", x"0225", x"02AD", x"034B",
        x"03FF", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000"
    );

    -- ========================================================================
    -- Hue wheel ROM: 256 hues at constant luma 560, max in-gamut chroma x0.92,
    -- 8-bit (128 = neutral), word = U(Cr) & V(Cb).  Read by the Psyche colourizer.
    -- ========================================================================
    type t_hrom is array (0 to 255) of std_logic_vector(15 downto 0);
    constant C_HUE : t_hrom := (
        x"80B2", x"81B2", x"82B2", x"84B2",
        x"85B2", x"86B2", x"87B2", x"89B2",
        x"8AB2", x"8BB2", x"8DB2", x"8EB2",
        x"8FB2", x"91B2", x"92B2", x"93B2",
        x"95B2", x"96B2", x"98B2", x"99B2",
        x"9BB2", x"9DB2", x"9EB2", x"A0B2",
        x"A2B2", x"A4B2", x"A5B2", x"A7B2",
        x"A9B2", x"ACB2", x"AEB2", x"B0B2",
        x"B2B2", x"B5B2", x"B8B2", x"BAB2",
        x"BDB2", x"C0B2", x"C0AF", x"C0AD",
        x"C0AB", x"C0A8", x"C0A6", x"C0A4",
        x"C0A2", x"C0A0", x"C09E", x"C09C",
        x"C09A", x"C099", x"C097", x"C095",
        x"C093", x"C092", x"C090", x"C08E",
        x"C08D", x"C08B", x"C089", x"C088",
        x"C086", x"C085", x"C083", x"C082",
        x"C080", x"C07E", x"C07D", x"C07B",
        x"C07A", x"C078", x"C077", x"C075",
        x"C073", x"C072", x"C070", x"C06E",
        x"C06D", x"C06B", x"C069", x"C067",
        x"C066", x"C064", x"C062", x"C060",
        x"C05E", x"C05C", x"C05A", x"C058",
        x"C055", x"C053", x"C051", x"C04E",
        x"C04C", x"C049", x"C046", x"C043",
        x"C040", x"BF3E", x"BC3E", x"B93E",
        x"B63E", x"B33E", x"B13E", x"AE3E",
        x"AC3E", x"AA3E", x"A73E", x"A53E",
        x"A33E", x"A13E", x"9F3E", x"9D3E",
        x"9B3E", x"993E", x"983E", x"963E",
        x"943E", x"923E", x"903E", x"8F3E",
        x"8D3E", x"8B3E", x"8A3E", x"883E",
        x"863E", x"853E", x"833E", x"823E",
        x"803E", x"7E3E", x"7D3E", x"7B3E",
        x"7A3E", x"783E", x"763E", x"753E",
        x"733E", x"713E", x"703E", x"6E3E",
        x"6C3E", x"6A3E", x"683E", x"673E",
        x"653E", x"633E", x"613E", x"5F3E",
        x"5D3E", x"5B3E", x"593E", x"563E",
        x"543E", x"523E", x"4F3E", x"4D3E",
        x"4A3E", x"473E", x"443E", x"413E",
        x"3E3E", x"3B3E", x"373E", x"343E",
        x"303E", x"2D3F", x"2D42", x"2D45",
        x"2D48", x"2D4B", x"2D4E", x"2D51",
        x"2D54", x"2D56", x"2D59", x"2D5B",
        x"2D5E", x"2D60", x"2D62", x"2D65",
        x"2D67", x"2D69", x"2D6B", x"2D6D",
        x"2D6F", x"2D72", x"2D74", x"2D76",
        x"2D78", x"2D7A", x"2D7C", x"2D7E",
        x"2D80", x"2D82", x"2D84", x"2D86",
        x"2D88", x"2D8A", x"2D8C", x"2D8E",
        x"2D91", x"2D93", x"2D95", x"2D97",
        x"2D99", x"2D9B", x"2D9E", x"2DA0",
        x"2DA2", x"2DA5", x"2DA7", x"2DAA",
        x"2DAC", x"2DAF", x"2DB2", x"30B2",
        x"34B2", x"38B2", x"3CB2", x"3FB2",
        x"43B2", x"46B2", x"48B2", x"4BB2",
        x"4EB2", x"50B2", x"52B2", x"54B2",
        x"57B2", x"59B2", x"5BB2", x"5CB2",
        x"5EB2", x"60B2", x"62B2", x"63B2",
        x"65B2", x"67B2", x"68B2", x"6AB2",
        x"6BB2", x"6DB2", x"6EB2", x"6FB2",
        x"71B2", x"72B2", x"73B2", x"75B2",
        x"76B2", x"77B2", x"79B2", x"7AB2",
        x"7BB2", x"7CB2", x"7EB2", x"7FB2"
    );

    -- echo n -> spread constant (n * 0x9E37), mixed with the 8-line segment
    -- random to pick each echo's dropout and thickness
    type t_nm is array (0 to 15) of unsigned(15 downto 0);
    constant C_NM : t_nm := (
        x"0000", x"9E37", x"3C6E", x"DAA5", x"78DC", x"1713", x"B54A", x"5381",
        x"F1B8", x"8FEF", x"2E26", x"CC5D", x"6A94", x"08CB", x"A702", x"4539");

    function xs32(v : unsigned(31 downto 0)) return unsigned is
        variable t : unsigned(31 downto 0);
    begin
        t := v xor shift_left(v, 13);
        t := t xor shift_right(t, 17);
        return t xor shift_left(t, 5);
    end function;

    -- 64-entry quarter-wave sine, 0..127
    type t_quad is array (0 to 63) of unsigned(7 downto 0);
    constant C_SIN_QUAD : t_quad := (
        to_unsigned(  0, 8), to_unsigned(  3, 8), to_unsigned(  6, 8), to_unsigned( 10, 8),
        to_unsigned( 13, 8), to_unsigned( 16, 8), to_unsigned( 19, 8), to_unsigned( 22, 8),
        to_unsigned( 25, 8), to_unsigned( 28, 8), to_unsigned( 31, 8), to_unsigned( 35, 8),
        to_unsigned( 38, 8), to_unsigned( 41, 8), to_unsigned( 44, 8), to_unsigned( 47, 8),
        to_unsigned( 49, 8), to_unsigned( 52, 8), to_unsigned( 55, 8), to_unsigned( 58, 8),
        to_unsigned( 61, 8), to_unsigned( 63, 8), to_unsigned( 66, 8), to_unsigned( 69, 8),
        to_unsigned( 71, 8), to_unsigned( 74, 8), to_unsigned( 76, 8), to_unsigned( 79, 8),
        to_unsigned( 81, 8), to_unsigned( 83, 8), to_unsigned( 86, 8), to_unsigned( 88, 8),
        to_unsigned( 90, 8), to_unsigned( 92, 8), to_unsigned( 94, 8), to_unsigned( 96, 8),
        to_unsigned( 98, 8), to_unsigned(100, 8), to_unsigned(102, 8), to_unsigned(104, 8),
        to_unsigned(106, 8), to_unsigned(107, 8), to_unsigned(109, 8), to_unsigned(110, 8),
        to_unsigned(112, 8), to_unsigned(113, 8), to_unsigned(114, 8), to_unsigned(116, 8),
        to_unsigned(117, 8), to_unsigned(118, 8), to_unsigned(119, 8), to_unsigned(120, 8),
        to_unsigned(121, 8), to_unsigned(122, 8), to_unsigned(123, 8), to_unsigned(124, 8),
        to_unsigned(124, 8), to_unsigned(125, 8), to_unsigned(125, 8), to_unsigned(126, 8),
        to_unsigned(126, 8), to_unsigned(127, 8), to_unsigned(127, 8), to_unsigned(127, 8));

    function quad_sin(ang : unsigned(9 downto 0)) return signed is
        variable v_idx : integer range 0 to 63;
        variable v_mag : unsigned(7 downto 0);
    begin
        v_idx := to_integer(ang(7 downto 2));
        if ang(8) = '0' then
            v_mag := C_SIN_QUAD(v_idx);
        else
            v_mag := C_SIN_QUAD(63 - v_idx);
        end if;
        if ang(9) = '0' then
            return signed(resize(v_mag, 9));
        else
            return -signed(resize(v_mag, 9));
        end if;
    end function;

    -- chroma byte (128 = neutral) scaled toward neutral by 2^-sh, pure wiring
    function chroma_sra(c : unsigned(7 downto 0); sh : natural) return unsigned is
        variable v_s : signed(7 downto 0);
    begin
        v_s := signed(std_logic_vector(c xor x"80"));
        v_s := shift_right(v_s, sh);
        return unsigned(std_logic_vector(v_s)) xor x"80";
    end function;

    -- ========================================================================
    -- Controls
    -- ========================================================================
    type t_k10 is array (0 to 6) of unsigned(9 downto 0);
    type t_cv  is array (0 to 7) of unsigned(15 downto 0);
    -- knobs 1..6 = entries 0..5, slider = entry 6
    signal k_hold : t_k10 := (others => C_MID);
    signal k_gl   : t_k10 := (others => C_MID);
    signal gpend  : std_logic_vector(0 to 6) := (others => '0');
    signal gl_cnt : unsigned(18 downto 0) := (others => '0');
    signal gl_tick: std_logic := '0';
    -- serial control engine: slot ce_c, step ce_t
    signal ce_c   : unsigned(2 downto 0) := (others => '0');
    signal ce_sel : std_logic_vector(0 to 7) := (others => '0');  -- one-hot ce_c, 1 clk late
    signal ce_pend: std_logic := '0';
    signal ce_t   : unsigned(3 downto 0) := (others => '0');
    signal ce_raw, ce_h, ce_g : unsigned(9 downto 0) := C_MID;
    signal ce_d   : signed(10 downto 0) := (others => '0');
    signal ce_dh  : signed(10 downto 0) := (others => '0');
    signal ce_rail: std_logic := '0';
    signal ce_f   : unsigned(7 downto 0) := (others => '0');
    signal ce_addr: unsigned(7 downto 0) := (others => '0');
    signal ce_romq: std_logic_vector(15 downto 0) := (others => '0');
    signal ce_ra, ce_rb : unsigned(15 downto 0) := (others => '0');
    signal ce_dd  : signed(16 downto 0) := (others => '0');
    signal ce_plo : signed(21 downto 0) := (others => '0');
    signal ce_phi : signed(21 downto 0) := (others => '0');
    signal ce_pr  : signed(25 downto 0) := (others => '0');
    signal cv     : t_cv := (others => (others => '0'));

    signal sw_psy, sw_dim, sw_soft, sw_smear, sw_inv : std_logic := '0';
    signal drive_g  : unsigned(8 downto 0)  := to_unsigned(64, 9);
    signal trig_q   : unsigned(7 downto 0)  := to_unsigned(30, 8);
    signal rmin_m1  : unsigned(5 downto 0)  := to_unsigned(5, 6);
    signal f0       : unsigned(14 downto 0) := to_unsigned(3000, 15);
    signal rate     : unsigned(11 downto 0) := to_unsigned(150, 12);
    signal bend     : unsigned(7 downto 0)  := (others => '0');
    signal fb       : unsigned(15 downto 0) := (others => '0');
    signal chaos8   : unsigned(7 downto 0)  := (others => '0');
    signal dropth   : unsigned(7 downto 0)  := (others => '0');
    -- per-consumer registered copies (placement: each sits by its compare)
    signal chaos_a, chaos_l, dropth_a, dropth_l : unsigned(7 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of chaos_a, chaos_l, dropth_a, dropth_l : signal is true;
    signal f0_7     : unsigned(17 downto 0) := (others => '0');
    signal d10, d05, d15, d20 : unsigned(15 downto 0) := to_unsigned(8192, 16);
    signal f0p, f0m : unsigned(15 downto 0) := (others => '0');
    signal s_lin, s_sq, s_cu : unsigned(9 downto 0) := (others => '0');
    -- instability terms
    signal jmax     : unsigned(5 downto 0)  := (others => '0');
    signal pjamp    : unsigned(6 downto 0)  := (others => '0');
    signal wamp     : unsigned(7 downto 0)  := (others => '0');
    signal hamp     : unsigned(6 downto 0)  := (others => '0');
    signal drop_thr : unsigned(7 downto 0)  := (others => '0');
    signal pthr     : unsigned(15 downto 0) := (others => '0');
    signal dmax     : unsigned(8 downto 0)  := (others => '0');
    signal snowfl   : unsigned(7 downto 0)  := (others => '0');
    signal cs_base  : unsigned(4 downto 0)  := to_unsigned(2, 5);
    signal hspeed   : unsigned(2 downto 0)  := (others => '0');
    signal wspeed   : unsigned(15 downto 0) := to_unsigned(300, 16);
    signal hmax     : unsigned(10 downto 0) := (others => '0');

    -- ========================================================================
    -- Input timing, measurement, frame anchor, per-line draws
    -- ========================================================================
    signal vin_y, vin_u, vin_v : std_logic_vector(9 downto 0) := (others => '0');
    signal vin_av, vin_hs, vin_vs, vin_fl : std_logic := '0';
    signal p_av, p_hs, p_vs : std_logic := '0';
    signal px_cnt   : unsigned(11 downto 0) := (others => '0');
    signal act_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal aline    : unsigned(11 downto 0) := (others => '0');
    signal act_h    : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal saw_act  : std_logic := '0';
    signal fr_tick  : std_logic := '0';
    signal cls      : unsigned(1 downto 0) := "00";   -- 1080 / 720 / SD
    signal wstep, hstep : unsigned(15 downto 0) := to_unsigned(182, 16);

    signal rng      : unsigned(31 downto 0) := x"2545F491";  -- xorshift32
    signal rng2     : unsigned(30 downto 0) := to_unsigned(1234567, 31); -- LFSR

    -- per-line phase accumulators (advance on ACTIVE lines only)
    signal wph, wph_f, hph, hph_f : unsigned(15 downto 0) := (others => '0');
    signal hdrift   : unsigned(7 downto 0) := (others => '0');
    signal sinw, sinh : signed(8 downto 0) := (others => '0');

    -- torn-band event
    signal band_on  : std_logic := '0';
    signal b0, b1   : signed(12 downto 0) := (others => '0');
    signal th       : unsigned(10 downto 0) := (others => '0');
    signal tdisp    : unsigned(8 downto 0) := (others => '0');
    signal tskew    : signed(2 downto 0) := (others => '0');
    signal thold    : unsigned(1 downto 0) := (others => '0');
    signal tvel     : signed(5 downto 0) := (others => '0');
    signal tacc     : unsigned(9 downto 0) := (others => '0');
    signal in_band  : std_logic := '0';
    signal r_pos    : unsigned(10 downto 0) := (others => '0');
    signal r_h      : unsigned(10 downto 0) := (others => '0');
    signal r_d      : unsigned(8 downto 0) := (others => '0');
    signal r_hit    : std_logic := '0';

    -- per-line draws
    signal hs_st    : std_logic_vector(7 downto 0) := (others => '0');
    signal lr       : unsigned(31 downto 0) := (others => '0');
    signal jit_v    : unsigned(5 downto 0) := (others => '0');
    signal php_v    : unsigned(15 downto 0) := (others => '0');
    signal wob_m    : signed(17 downto 0) := (others => '0');
    signal hum_m    : signed(16 downto 0) := (others => '0');
    signal disp_n   : unsigned(9 downto 0) := (others => '0');
    -- committed per-line values
    signal disp_line  : unsigned(8 downto 0) := (others => '0');
    signal ph0_line   : unsigned(15 downto 0) := (others => '0');
    signal hum_line   : signed(7 downto 0) := (others => '0');
    signal drop_line  : std_logic := '0';
    signal cs_line    : unsigned(4 downto 0) := to_unsigned(2, 5);
    signal hoff_line  : unsigned(7 downto 0) := (others => '0');
    signal sph_line   : unsigned(1 downto 0) := (others => '0');
    signal bsn_line   : std_logic := '0';
    signal hpsy       : unsigned(7 downto 0) := (others => '0');
    signal seg_line   : unsigned(15 downto 0) := (others => '0');
    signal crng       : unsigned(31 downto 0) := x"6C8E9CF5";  -- column random
    signal col_seed   : unsigned(31 downto 0) := x"6C8E9CF5";
    -- hold / move: per-line randoms restart from line_seed every frame
    signal line_seed  : unsigned(31 downto 0) := x"1F123BB5";
    signal mv_thr     : unsigned(15 downto 0) := (others => '0');
    signal mv_hit     : std_logic := '0';
    signal mv_n       : unsigned(4 downto 0) := (others => '0');
    signal mv_vel     : signed(15 downto 0) := (others => '0');
    signal phg        : unsigned(15 downto 0) := (others => '0');

    -- generated avid (parity-locked start, fixed-width end)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    -- generated avid aligned to each pipeline depth: gv(m) is valid in the
    -- cycle a pixel whose vin cycle was c0 reaches c0 + m
    signal gv     : std_logic_vector(0 to 15) := (others => '0');

    -- ========================================================================
    -- Rings (EBR)
    -- ========================================================================
    type t_r8  is array (0 to 511) of std_logic_vector(7 downto 0);
    type t_r16 is array (0 to 511) of std_logic_vector(15 downto 0);
    signal ram_a : t_r8  := (others => (others => '0'));
    signal ram_y : t_r16 := (others => (others => '0'));
    signal ram_c : t_r16 := (others => (others => '0'));
    signal ram_s : t_r8  := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram_a : signal is true;
    attribute no_rw_check of ram_y : signal is true;
    attribute no_rw_check of ram_c : signal is true;
    attribute no_rw_check of ram_s : signal is true;
    signal wp    : unsigned(8 downto 0) := (others => '0');
    signal ra_a, ra_y, ra_c, ra_s : unsigned(8 downto 0) := (others => '0');
    type t_ad is array (1 to KD - 1) of unsigned(8 downto 0);
    signal ra_d  : t_ad := (others => (others => '0'));
    signal q_a   : std_logic_vector(7 downto 0)  := (others => '0');
    signal q_y   : std_logic_vector(15 downto 0) := (others => '0');
    signal q_c   : std_logic_vector(15 downto 0) := (others => '0');
    signal q_s   : std_logic_vector(7 downto 0)  := (others => '0');

    -- ========================================================================
    -- Luma / trigger / oscillator path (aN valid at c0 + 2 + N)
    -- ========================================================================
    signal a1_y   : unsigned(7 downto 0) := (others => '0');
    signal a2_yd  : unsigned(7 downto 0) := (others => '0');
    signal a2_s   : unsigned(8 downto 0) := (others => '0');
    signal a3_d1, a3_d2 : unsigned(8 downto 0) := (others => '0');
    signal a3_s   : unsigned(9 downto 0) := (others => '0');
    signal a4_l   : unsigned(7 downto 0) := (others => '0');
    signal a4_lp, a4_lm : signed(9 downto 0) := (others => '0');
    signal tr_hi  : signed(9 downto 0) := to_signed(46, 10);
    signal tr_lo  : signed(9 downto 0) := to_signed(-14, 10);
    signal tr_dist: unsigned(5 downto 0) := (others => '1');
    signal tr_ok  : std_logic := '1';
    signal drop_a : std_logic := '0';
    signal rk_a   : unsigned(8 downto 0) := (others => '0');  -- per-row key
    signal a5_ev, a6_ev, a7_ev : std_logic := '0';
    signal a5_pol, a6_pol, a7_pol : std_logic := '0';
    signal a4_mlo, a4_mhi : unsigned(15 downto 0) := (others => '0');
    signal a5_fm  : unsigned(19 downto 0) := (others => '0');
    signal a6_inc : unsigned(16 downto 0) := (others => '0');
    signal a7_inc : unsigned(14 downto 0) := (others => '0');
    signal ph0_a  : unsigned(15 downto 0) := (others => '0');
    signal os_phi : unsigned(15 downto 0) := (others => '0');
    signal os_amp : unsigned(16 downto 0) := (others => '0');
    signal os_n   : unsigned(3 downto 0) := (others => '0');
    signal os_wr  : std_logic := '0';
    signal os_pol : std_logic := '0';
    signal os_ev  : std_logic := '0';
    signal a5_f0w : unsigned(15 downto 0) := (others => '0');
    signal a4_hit, a4_sgn : std_logic := '0';
    signal seg_a  : unsigned(15 downto 0) := (others => '0');
    signal a9_rv  : unsigned(15 downto 0) := (others => '0');
    signal a9_c   : std_logic_vector(3 downto 0) := (others => '0');
    signal a9_amp : unsigned(7 downto 0) := (others => '0');
    signal a10_r  : unsigned(7 downto 0) := (others => '0');
    signal a11_r  : unsigned(7 downto 0) := (others => '0');
    signal a12_r  : unsigned(7 downto 0) := (others => '0');
    signal ev9, ev10, ev11 : std_logic := '0';
    signal pl9, pl10, pl11 : std_logic := '0';
    -- echo colour: segment max (brightest since the last sync) and the line
    -- colour itself (boosted luma, chroma), valid with a12
    signal sm_y, lc_y : unsigned(9 downto 0) := C_BLK;
    signal sm_u, sm_v, lc_u, lc_v : unsigned(9 downto 0) := C_MID;
    signal lc_lock : std_logic := '1';
    signal lc_cnt  : unsigned(2 downto 0) := (others => '0');

    -- ========================================================================
    -- Picture path (bN valid at c0 + 7 + N; b7 lines up with a12)
    -- ========================================================================
    signal hum_b  : signed(7 downto 0) := (others => '0');
    signal bsn_b  : std_logic := '0';
    signal sph_b  : unsigned(1 downto 0) := (others => '0');
    signal b1_y, b1_u, b1_v : unsigned(9 downto 0) := (others => '0');
    signal b2_yo  : signed(11 downto 0) := (others => '0');
    signal b2_u, b2_v : unsigned(9 downto 0) := C_MID;
    signal b2_pi  : unsigned(7 downto 0) := (others => '0');
    signal b3_plo : signed(17 downto 0) := (others => '0');
    signal b3_phi : signed(16 downto 0) := (others => '0');
    signal b3_u, b3_v : unsigned(9 downto 0) := C_MID;
    signal romq_p : std_logic_vector(15 downto 0) := x"8080";
    signal b4_yp  : signed(21 downto 0) := (others => '0');
    signal b4_u, b4_v : unsigned(9 downto 0) := C_MID;
    signal b4_pu, b4_pv : unsigned(7 downto 0) := x"80";
    signal b5_y, b5_y2 : unsigned(9 downto 0) := C_BLK;
    signal b6_yb  : unsigned(9 downto 0) := C_BLK;
    signal b5_e   : unsigned(9 downto 0) := (others => '0');
    signal b5_uc, b5_vc : unsigned(9 downto 0) := C_MID;
    signal tr_y   : unsigned(9 downto 0) := C_BLK;
    signal tr_u, tr_v : unsigned(9 downto 0) := C_MID;
    signal b6_y   : unsigned(9 downto 0) := C_BLK;
    signal b6_dec : std_logic := '1';
    signal b6_uc, b6_vc : unsigned(9 downto 0) := C_MID;
    signal b6_dn  : unsigned(7 downto 0) := (others => '0');
    signal b7_y   : unsigned(9 downto 0) := C_BLK;
    signal b7_u, b7_v : unsigned(9 downto 0) := C_MID;
    signal b7_sn, b7_sw : std_logic := '0';
    signal dcnt   : unsigned(1 downto 0) := (others => '0');
    signal dash_r : unsigned(7 downto 0) := (others => '1');
    signal dash_w : std_logic := '0';
    signal b8_dy, b8_du, b8_dv : signed(10 downto 0) := (others => '0');
    signal b8_r   : unsigned(7 downto 0) := (others => '0');
    signal b8_y   : unsigned(9 downto 0) := C_BLK;
    signal b8_u, b8_v : unsigned(9 downto 0) := C_MID;
    signal b8_sn, b8_sw : std_logic := '0';
    signal b9_py  : signed(19 downto 0) := (others => '0');
    signal b9_pu, b9_pv : signed(17 downto 0) := (others => '0');
    signal b9_y   : unsigned(9 downto 0) := C_BLK;
    signal b9_u, b9_v : unsigned(9 downto 0) := C_MID;
    signal b9_sn, b9_sw : std_logic := '0';
    signal x8_y   : unsigned(9 downto 0) := C_BLK;
    signal x8_u, x8_v : unsigned(9 downto 0) := C_MID;

    -- output conditioning
    signal w0y, w1y, w2y : unsigned(9 downto 0) := C_BLK;
    signal w0u, w1u, w2u : unsigned(9 downto 0) := C_MID;
    signal w0v, w1v, w2v : unsigned(9 downto 0) := C_MID;
    signal c9_ya  : unsigned(10 downto 0) := (others => '0');
    signal c9_yb  : unsigned(9 downto 0) := C_BLK;
    signal c9_ua, c9_va : unsigned(10 downto 0) := (others => '0');
    signal c9_ub, c9_vb : unsigned(9 downto 0) := C_MID;
    signal c10_y  : unsigned(9 downto 0) := C_BLK;
    signal c10_u, c10_v : unsigned(9 downto 0) := C_MID;
    signal out_y  : unsigned(9 downto 0) := C_BLK;
    signal out_u, out_v : unsigned(9 downto 0) := C_MID;
    signal out_s  : std_logic_vector(3 downto 0) := "0110";

begin

    -- ========================================================================
    -- CONTROLS — decoded free-running every clock, never in a frame branch.
    -- Each knob: deadband hold (ADC flicker) + glide on a free-running tick
    -- (~140 Hz HD / ~26 Hz SD), then a 17-point curve with interpolation.
    -- ========================================================================
    p_tick : process(clk)
    begin
        if rising_edge(clk) then
            gl_cnt  <= gl_cnt + 1;
            if gl_cnt = 0 then
                gl_tick <= '1';
            else
                gl_tick <= '0';
            end if;
        end if;
    end process p_tick;

    -- One time-shared engine for every knob: 8 slots x 16 steps, a full
    -- pass every 128 clocks.  Per slot: deadband hold (ADC flicker), a glide
    -- step if the tick has fired since this knob's last pass, then a curve
    -- lookup with linear interpolation on one multiplier.  Slot 7 reuses the
    -- slider (second curve) and never writes its state; slot 5 (Hue, no
    -- curve) spends the multiplier on fb = f0 * bend / 64.
    p_ctl : process(clk)
        variable v_kc : integer range 0 to 6;
        variable v_g  : signed(10 downto 0);
        variable v_o  : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            ce_romq <= C_CROM(to_integer(ce_addr));
            ce_t <= ce_t + 1;
            if ce_t = 15 then
                ce_c <= ce_c + 1;
            end if;
            if ce_c = 7 then
                v_kc := 6;
            else
                v_kc := to_integer(ce_c);
            end if;
            -- one-hot slot, registered: the writes (steps 2, 3, 12) use it as
            -- plain enables instead of decoding ce_c in their cones
            for i in 0 to 7 loop
                if to_integer(ce_c) = i then
                    ce_sel(i) <= '1';
                else
                    ce_sel(i) <= '0';
                end if;
            end loop;
            case to_integer(ce_t) is
                when 0 =>
                    if v_kc < 6 then
                        ce_raw <= unsigned(registers_in(v_kc));
                    else
                        ce_raw <= unsigned(registers_in(7));
                    end if;
                    ce_h <= k_hold(v_kc);
                    ce_g <= k_gl(v_kc);
                when 1 =>
                    ce_pend <= gpend(v_kc) and not ce_sel(7);
                    ce_dh   <= signed('0' & ce_raw) - signed('0' & ce_h);
                    if ce_raw < 4 or ce_raw > 1019 then
                        ce_rail <= '1';
                    else
                        ce_rail <= '0';
                    end if;
                when 2 =>
                    if ce_dh >= 4 or ce_dh <= -4 or ce_rail = '1' then
                        ce_h <= ce_raw;
                    end if;
                when 3 =>
                    ce_d <= signed('0' & ce_h) - signed('0' & ce_g);
                    for i in 0 to 6 loop
                        if ce_sel(i) = '1' then
                            k_hold(i) <= ce_h;
                        end if;
                    end loop;
                when 4 =>
                    if ce_pend = '1' then
                        if ce_d < 4 and ce_d > -4 then
                            v_g := signed('0' & ce_h);
                        else
                            v_g := signed('0' & ce_g) + shift_right(ce_d, 2);
                        end if;
                        ce_g <= unsigned(std_logic_vector(v_g(9 downto 0)));
                        for i in 0 to 6 loop
                            if ce_sel(i) = '1' then
                                k_gl(i)  <= unsigned(std_logic_vector(v_g(9 downto 0)));
                                gpend(i) <= '0';
                            end if;
                        end loop;
                    end if;
                when 5 =>
                    ce_addr <= ce_c & '0' & ce_g(9 downto 6);
                    ce_f    <= "00" & ce_g(5 downto 0);
                when 6 =>
                    ce_addr <= ce_c & (('0' & ce_g(9 downto 6)) + 1);
                -- the EBR output holds point i during step 7 (address set at
                -- step 5, read at step 6's edge) and point i + 1 during step 8
                when 7 =>
                    ce_ra <= unsigned(ce_romq);   -- point i (fabric register)
                when 8 =>
                    ce_rb <= unsigned(ce_romq);   -- point i + 1
                when 9 =>
                    if ce_c = 5 then
                        ce_ra <= (others => '0');
                        ce_dd <= signed(resize(f0, 17));
                        ce_f  <= bend;
                    else
                        ce_dd <= signed('0' & ce_rb) - signed('0' & ce_ra);
                    end if;
                when 10 =>
                    ce_plo <= ce_dd * signed('0' & ce_f(3 downto 0));
                    ce_phi <= ce_dd * signed('0' & ce_f(7 downto 4));
                when 11 =>
                    ce_pr <= resize(ce_plo, 26) + shift_left(resize(ce_phi, 26), 4);
                when 12 =>
                    v_o := signed(resize(ce_ra, 18)) + resize(shift_right(ce_pr, 6), 18);
                    for i in 0 to 7 loop
                        if ce_sel(i) = '1' then
                            cv(i) <= unsigned(std_logic_vector(v_o(15 downto 0)));
                        end if;
                    end loop;
                when others =>
                    null;
            end case;
            -- after the clear above, so a tick in the same clock re-arms
            if gl_tick = '1' then
                gpend <= (others => '1');
            end if;
        end if;
    end process p_ctl;

    -- Engine outputs and glided knobs are registers already: these are wires.
    drive_g  <= cv(0)(8 downto 0);
    fb       <= cv(5);
    trig_q   <= cv(1)(7 downto 0);
    f0       <= cv(2)(14 downto 0);
    rate     <= cv(3)(11 downto 0);
    bend     <= cv(4)(7 downto 0);
    s_sq     <= cv(6)(9 downto 0);
    s_cu     <= cv(7)(9 downto 0);
    s_lin    <= k_gl(6);
    chaos8   <= k_gl(5)(9 downto 2);
    dropth   <= "00" & k_gl(5)(9 downto 4);
    -- instability terms (HD-first; displacements halve at SD)
    jmax     <= '0' & s_cu(9 downto 5) when cls = "10" else s_cu(9 downto 4);
    dmax     <= '0' & s_cu(9 downto 2) when cls = "10" else s_cu(9 downto 1);
    pjamp    <= s_sq(9 downto 3);
    drop_thr <= '0' & s_cu(9 downto 3);
    pthr     <= "00" & s_cu & "0000";
    snowfl   <= "00" & s_cu(9 downto 4);
    hspeed   <= s_lin(9 downto 7);

    p_derive : process(clk)
    begin
        if rising_edge(clk) then
            sw_psy   <= registers_in(6)(0);
            sw_dim   <= registers_in(6)(1);
            sw_soft  <= registers_in(6)(2);
            sw_smear <= registers_in(6)(3);
            sw_inv   <= registers_in(6)(4);

            rmin_m1 <= to_unsigned(2, 6) + resize(trig_q(7 downto 3), 6);
            chaos_a  <= chaos8;
            chaos_l  <= chaos8;
            dropth_a <= dropth;
            dropth_l <= dropth;
            wamp     <= s_lin(9 downto 2);
            hamp     <= s_lin(9 downto 3);

            -- line width: ~7 px at the base step (phase threshold f0 * 7),
            -- with half / 1.5x / 2x variants for Chaos; capped at 0.69 period
            f0_7 <= shift_left(resize(f0, 18), 3) - resize(f0, 18);
            if f0_7 > 45000 then
                d10 <= to_unsigned(45000, 16);
            else
                d10 <= f0_7(15 downto 0);
            end if;
            d05 <= shift_right(d10, 1);
            if d10 > 30000 then
                d15 <= to_unsigned(45000, 16);
            else
                d15 <= d10 + shift_right(d10, 1);
            end if;
            if d10 > 22500 then
                d20 <= to_unsigned(45000, 16);
            else
                d20 <= shift_left(d10, 1);
            end if;
            -- Chaos move rate, per frame (x/65536), exponential: never at 0,
            -- ~1/1000 frames at the bottom, ~1/120 at the default, 1/4 at
            -- the top
            case to_integer(chaos8(7 downto 4)) is
                when 0      => mv_thr <= to_unsigned(0, 16);
                when 1      => mv_thr <= to_unsigned(64, 16);
                when 2      => mv_thr <= to_unsigned(128, 16);
                when 3      => mv_thr <= to_unsigned(218, 16);
                when 4      => mv_thr <= to_unsigned(364, 16);
                when 5      => mv_thr <= to_unsigned(546, 16);
                when 6      => mv_thr <= to_unsigned(819, 16);
                when 7      => mv_thr <= to_unsigned(1092, 16);
                when 8      => mv_thr <= to_unsigned(1456, 16);
                when 9      => mv_thr <= to_unsigned(2048, 16);
                when 10     => mv_thr <= to_unsigned(2731, 16);
                when 11     => mv_thr <= to_unsigned(4096, 16);
                when 12     => mv_thr <= to_unsigned(5461, 16);
                when 13     => mv_thr <= to_unsigned(8192, 16);
                when 14     => mv_thr <= to_unsigned(10923, 16);
                when others => mv_thr <= to_unsigned(16384, 16);
            end case;
            -- Chaos random-walk step: f0/2 below half travel, f0 above
            if chaos8(7) = '1' then
                f0p <= resize(f0, 16) + resize(f0, 16);
                f0m <= (others => '0');
            else
                f0p <= resize(f0, 16) + resize(shift_right(f0, 1), 16);
                f0m <= resize(f0, 16) - resize(shift_right(f0, 1), 16);
            end if;

            cs_base  <= to_unsigned(2, 5) + resize(s_lin(9 downto 6), 5);
            wspeed   <= to_unsigned(300, 16) + shift_left(resize(s_lin, 16), 2);
            case s_lin(9 downto 7) is
                when "111"         => hmax <= act_h(11 downto 1);
                when "110"         => hmax <= resize(act_h(11 downto 2), 11);
                when "101" | "100" => hmax <= resize(act_h(11 downto 3), 11);
                when "011" | "010" => hmax <= resize(act_h(11 downto 4), 11);
                when others        => hmax <= resize(act_h(11 downto 5), 11);
            end case;
        end if;
    end process p_derive;

    -- ========================================================================
    -- INPUT TIMING: registers, width/height, frame anchor, per-line draws
    -- ========================================================================
    p_time : process(clk)
        variable v_hfall : std_logic;
        variable v_rp    : unsigned(10 downto 0);
        variable v_d     : unsigned(10 downto 0);
        variable v_ta    : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            vin_y  <= data_in.y;
            vin_u  <= data_in.u;
            vin_v  <= data_in.v;
            vin_av <= data_in.avid;
            vin_hs <= data_in.hsync_n;
            vin_vs <= data_in.vsync_n;
            vin_fl <= data_in.field_n;
            p_av <= vin_av;
            p_hs <= vin_hs;
            p_vs <= vin_vs;
            fr_tick <= '0';

            -- free-running generators: xorshift32 (per-pixel snow, per-line
            -- draws) and an independent 31-bit LFSR (per-frame tear draws)
            rng  <= xs32(rng);
            rng2 <= rng2(29 downto 0) & (rng2(30) xor rng2(27));

            -- free-running tear draws (sampled by the anchor), no multiplies:
            -- position = a raster-sized random folded once into the height,
            -- size and offset = the maximum >> a random 0..3
            case cls is
                when "00"   => v_rp := rng2(10 downto 0);
                when "01"   => v_rp := '0' & rng2(9 downto 0);
                when others => v_rp := "00" & rng2(8 downto 0);
            end case;
            if v_rp >= act_h(10 downto 0) then
                r_pos <= v_rp - act_h(10 downto 0);
            else
                r_pos <= v_rp;
            end if;
            r_h <= shift_right(hmax, to_integer(rng2(13 downto 12))) + 4;
            r_d <= shift_right(dmax, to_integer(rng2(15 downto 14))) + 16;
            if rng(15 downto 0) < pthr then
                r_hit <= '1';
            else
                r_hit <= '0';
            end if;
            if rng(31 downto 16) < mv_thr then
                mv_hit <= '1';
            else
                mv_hit <= '0';
            end if;
            b1      <= b0 + signed(resize(th, 13));
            if band_on = '1' and signed(resize(aline, 13)) >= b0 and
               signed(resize(aline, 13)) < b1 then
                in_band <= '1';
            else
                in_band <= '0';
            end if;

            -- per-line sines (wobble of the oscillator phase, hum bar)
            sinw <= quad_sin(wph(15 downto 6));
            sinh <= quad_sin(hph(15 downto 6));

            -- horizontal: measured width, active-line count
            v_hfall := p_hs and not vin_hs;
            if v_hfall = '1' then
                px_cnt <= (others => '0');
            elsif vin_av = '1' then
                px_cnt <= px_cnt + 1;
            end if;

            -- end of an active line: advance per-line accumulators (active
            -- lines only, so interlaced fields stay in step)
            if p_av = '1' and vin_av = '0' then
                if px_cnt > 100 then
                    act_w <= px_cnt;
                end if;
                aline <= aline + 1;
                wph   <= wph + wstep;
                hph   <= hph + hstep;
                -- next line's randoms: a fixed sequence from line_seed
                lr    <= xs32(lr);
            end if;

            -- frame anchor: first vsync EDGE after active video (serration
            -- guard), backed by a line ceiling outside any vsync condition
            if saw_act = '1' and
               ((vin_vs = '0' and p_vs = '1') or aline >= 1200) then
                saw_act <= '0';
                fr_tick <= '1';
                if aline > 50 then
                    act_h <= aline;
                end if;
                aline <= (others => '0');
                g_wm1 <= act_w - 1;
                if act_w > 1400 then
                    cls <= "00";
                    wstep <= to_unsigned(182, 16);
                    hstep <= to_unsigned(58, 16);
                elsif act_w > 900 then
                    cls <= "01";
                    wstep <= to_unsigned(262, 16);
                    hstep <= to_unsigned(87, 16);
                else
                    cls <= "10";
                    wstep <= to_unsigned(720, 16);
                    hstep <= to_unsigned(240, 16);
                end if;
                wph    <= wph_f;
                hph_f  <= hph_f + to_unsigned(700, 16);
                hph    <= hph_f;
                hdrift <= hdrift + resize(hspeed, 8);
                lr     <= line_seed;

                -- HOLD / MOVE.  Holding, nothing that places a line changes
                -- (every frame replays the same seeds).  A move snaps the
                -- line seed (dropouts, widths, scatter, jitter all change),
                -- then for 2..17 frames slides the column pattern a pixel a
                -- frame, slips the global phase by a random velocity and
                -- advances the wobble, then freezes.
                if mv_n /= 0 then
                    mv_n     <= mv_n - 1;
                    phg      <= phg + unsigned(std_logic_vector(mv_vel));
                    col_seed <= xs32(col_seed);
                    wph_f    <= wph_f + wspeed;
                elsif mv_hit = '1' then
                    mv_n      <= resize(rng2(3 downto 0), 5) + 2;
                    mv_vel    <= resize(signed(std_logic_vector(rng2(11 downto 4))), 16) sll 6;
                    line_seed <= rng;
                end if;

                -- torn-band events: a live band scrolls and counts down its
                -- hold; an idle one waits for a random hit
                if band_on = '1' then
                    if thold = 0 then
                        band_on <= '0';
                    else
                        thold <= thold - 1;
                    end if;
                    b0 <= b0 + resize(tvel, 13);
                elsif r_hit = '1' then
                    band_on <= '1';
                    b0      <= signed(resize(r_pos, 13));
                    th      <= r_h;
                    tdisp   <= r_d;
                    tskew   <= signed(std_logic_vector(rng2(26 downto 24)));
                    thold   <= rng2(28 downto 27);
                    tvel    <= signed(std_logic_vector(rng2(30 downto 29))) & "0000";
                end if;
            elsif vin_av = '1' and saw_act = '0' then
                saw_act <= '1';
            end if;

            -- ---------------------------------------------------------------
            -- Per-line draws at hsync.  The products run free-running on the
            -- latched draw; everything is committed 6 clocks after hsync (in
            -- blanking at every pipeline depth that reads it directly).
            -- ---------------------------------------------------------------
            hs_st <= hs_st(6 downto 0) & v_hfall;
            -- Chaos column random: restarted from the same seed every line,
            -- so its sequence is a function of the column; the seed steps only
            -- while a move runs, a pixel of pattern slide per frame
            if v_hfall = '1' then
                crng <= col_seed;
            else
                crng <= xs32(crng);
            end if;
            -- one random per 8-line segment picks each echo's dropout/width
            if hs_st(2) = '1' and aline(2 downto 0) = "000" then
                seg_line <= lr(31 downto 16);
            end if;
            jit_v <= lr(5 downto 0) and jmax;
            php_v <= lr(15 downto 0) and (pjamp & "111111111");
            wob_m <= sinw * signed('0' & wamp);
            hum_m <= sinh * signed('0' & hamp);

            if hs_st(1) = '1' then
                -- tear offset grows by the skew on every band line
                if in_band = '1' then
                    v_ta := signed('0' & tacc) + resize(tskew, 11);
                    if v_ta < 0 then
                        tacc <= (others => '0');
                    elsif v_ta > C_DMAXPX then
                        tacc <= to_unsigned(C_DMAXPX, 10);
                    else
                        tacc <= unsigned(v_ta(9 downto 0));
                    end if;
                else
                    tacc <= resize(tdisp, 10);
                end if;
            end if;
            if hs_st(3) = '1' then
                if in_band = '1' then
                    disp_n <= resize(jit_v, 10) + tacc;
                else
                    disp_n <= resize(jit_v, 10);
                end if;
            end if;
            if hs_st(5) = '1' then
                if disp_n > C_DMAXPX then
                    disp_line <= to_unsigned(C_DMAXPX, 9);
                else
                    disp_line <= disp_n(8 downto 0);
                end if;
                ph0_line <= php_v + phg +
                            unsigned(std_logic_vector(wob_m(14 downto 0) & '0'));
                hum_line <= hum_m(14 downto 7);
                if lr(23 downto 16) < drop_thr or in_band = '1' then
                    drop_line <= '1';
                else
                    drop_line <= '0';
                end if;
                if s_lin(9) = '1' then
                    cs_line <= cs_base + resize(lr(25 downto 24), 5);
                else
                    cs_line <= cs_base;
                end if;
                if in_band = '1' then
                    hoff_line <= to_unsigned(128, 8) + resize(lr(31 downto 26), 8);
                else
                    hoff_line <= (others => '0');
                end if;
                sph_line <= lr(27 downto 26);
                bsn_line <= in_band;
            end if;
            -- Psyche palette rotation (instability drift, torn-band flips)
            hpsy      <= hdrift + hoff_line;
        end if;
    end process p_time;

    -- ========================================================================
    -- Generated avid: hsync-parity-locked start, fixed-width end (the encoders
    -- pair Cb/Cr by counting from hsync; a wandering source avid swaps U/V on
    -- a line — see cubist / aurora).
    -- ========================================================================
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
            if g_pc = 15 then
                g_par <= '1';
            elsif g_pc = 0 then
                g_par <= '0';
            end if;

            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            gv(0) <= g_avid;
            for i in 1 to 15 loop
                gv(i) <= gv(i - 1);
            end loop;
        end if;
    end process p_gavid;

    -- ========================================================================
    -- RINGS.  One write statement each, written every clock (neutral in
    -- blanking).  Read addresses never equal the write address (offsets
    -- 1+disp .. 6+disp+cs <= 506), so no read-before-write emulation.
    -- ========================================================================
    p_ring : process(clk)
        variable v_w8  : std_logic_vector(7 downto 0);
        variable v_wy  : std_logic_vector(15 downto 0);
        variable v_wc  : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            if vin_av = '1' then
                v_w8 := vin_y(9 downto 2);
                v_wy := "000000" & vin_y;
                v_wc := vin_u(9 downto 2) & vin_v(9 downto 2);
            else
                v_w8 := x"10";
                v_wy := "0000000001000000";
                v_wc := x"8080";
            end if;
            ram_a(to_integer(wp)) <= v_w8;
            ram_y(to_integer(wp)) <= v_wy;
            ram_c(to_integer(wp)) <= v_wc;
            ram_s(to_integer(wp)) <= "000" & gv(0) & vin_av & vin_hs & vin_vs & vin_fl;
            wp <= wp + 1;

            ra_a    <= wp - disp_line;
            ra_d(1) <= ra_a;
            for i in 2 to KD - 1 loop
                ra_d(i) <= ra_d(i - 1);
            end loop;
            ra_y <= ra_d(KD - 1);
            ra_c <= ra_d(KD - 1) - resize(cs_line, 9);
            ra_s <= wp - to_unsigned(SYNC_D, 9);

            q_a <= ram_a(to_integer(ra_a));
            q_y <= ram_y(to_integer(ra_y));
            q_c <= ram_c(to_integer(ra_c));
            q_s <= ram_s(to_integer(ra_s));
        end if;
    end process p_ring;

    -- ========================================================================
    -- LUMA PATH: 4-tap box, delta trigger, FM step
    -- ========================================================================
    p_luma : process(clk)
        variable v_l  : unsigned(7 downto 0);
        variable v_hi : std_logic;
        variable v_lo : std_logic;
    begin
        if rising_edge(clk) then
            -- a1..a3: fabric re-register, 4-tap box sum
            a1_y  <= unsigned(q_a);
            a2_yd <= a1_y;
            a2_s  <= ('0' & a1_y) + ('0' & a2_yd);
            a3_d1 <= a2_s;
            a3_d2 <= a3_d1;
            a3_s  <= ('0' & a2_s) + ('0' & a3_d2);

            -- a4: low-passed luma and the two thresholds a sync AT this pixel
            -- would leave behind (kept out of the trigger loop)
            v_l   := a3_s(9 downto 2);
            a4_l  <= v_l;
            a4_lp <= signed(resize(v_l, 10)) + signed(resize(trig_q, 10));
            a4_lm <= signed(resize(v_l, 10)) - signed(resize(trig_q, 10));

            -- per-line sync drop, taken in this stage's own blanking
            if gv(5) = '0' then
                drop_a <= drop_line;
                rk_a   <= lr(31 downto 23);
            end if;

            -- a5: delta trigger.  Sync when the luma has left the band
            -- (ref - Q, ref + Q) set at the last sync, and at least rmin px
            -- have passed (qualification: analog noise and fine texture
            -- cannot chain syncs).  The band edges are registers, so the loop
            -- is two parallel compares.
            if signed(resize(a4_l, 10)) >= tr_hi then
                v_hi := '1';
            else
                v_hi := '0';
            end if;
            if signed(resize(a4_l, 10)) <= tr_lo then
                v_lo := '1';
            else
                v_lo := '0';
            end if;
            if gv(6) = '0' then
                -- blanking: reference = black, so a bright picture edge syncs
                tr_hi   <= to_signed(16, 10) + signed(resize(trig_q, 10));
                tr_lo   <= to_signed(16, 10) - signed(resize(trig_q, 10));
                tr_dist <= (others => '1');
                tr_ok   <= '1';
                a5_ev   <= '0';
            elsif (v_hi = '1' or v_lo = '1') and tr_ok = '1' and drop_a = '0' then
                tr_hi   <= a4_lp;
                tr_lo   <= a4_lm;
                tr_dist <= (others => '0');
                tr_ok   <= '0';
                a5_ev   <= '1';
                a5_pol  <= v_hi;
            else
                a5_ev <= '0';
                if tr_dist /= 63 then
                    tr_dist <= tr_dist + 1;
                end if;
                if tr_dist >= rmin_m1 then
                    tr_ok <= '1';
                end if;
            end if;

            -- a4..a7: FM step = f0 + fb * luma / 256 (operands split hi/lo),
            -- capped at a 2.67 px period.  Chaos: on a hit the base step is
            -- swapped for f0 +- step (column random, the same on every row,
            -- plus a quarter as many per-row hits for roughness: the column
            -- random xor a per-line key, so it is still frame-stable).
            a4_mlo <= fb(15 downto 4) * v_l(3 downto 0);
            a4_mhi <= fb(15 downto 4) * v_l(7 downto 4);
            if crng(7 downto 0) < chaos_a then
                a4_hit <= '1';
                a4_sgn <= crng(8);
            elsif (crng(15 downto 8) xor rk_a(7 downto 0)) < dropth_a then
                a4_hit <= '1';
                a4_sgn <= crng(16) xor rk_a(8);
            else
                a4_hit <= '0';
                a4_sgn <= '0';
            end if;
            if a4_hit = '0' then
                a5_f0w <= resize(f0, 16);
            elsif a4_sgn = '1' then
                a5_f0w <= f0p;
            else
                a5_f0w <= f0m;
            end if;
            a5_fm  <= resize(a4_mlo, 20) + shift_left(resize(a4_mhi, 20), 4);
            a6_inc <= resize(a5_f0w, 17) + resize(a5_fm(19 downto 4), 17);
            if a6_inc > C_INCMAX then
                a7_inc <= to_unsigned(C_INCMAX, 15);
            else
                a7_inc <= a6_inc(14 downto 0);
            end if;
            a6_ev <= a5_ev;   a6_pol <= a5_pol;
            a7_ev <= a6_ev;   a7_pol <= a6_pol;
        end if;
    end process p_luma;

    -- ========================================================================
    -- OSCILLATOR (a8): hard-synced phase, envelope, echo count
    -- ========================================================================
    p_osc : process(clk)
        variable v_s : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            if gv(9) = '0' then
                ph0_a <= ph0_line;
            end if;
            v_s := ('0' & os_phi) + resize(a7_inc, 17);
            os_ev <= '0';
            if gv(9) = '0' then
                os_phi <= ph0_a;
                os_amp <= (others => '0');
                os_n   <= (others => '0');
                os_wr  <= '0';
                os_pol <= '0';
            elsif a7_ev = '1' then
                if sw_soft = '1' then
                    os_phi <= v_s(15 downto 0);
                else
                    os_phi <= ph0_a;
                end if;
                -- 1.5 x full scale: the train holds full brightness for its
                -- first third, then fades
                os_amp <= to_unsigned(C_AMP0, 17);
                os_n   <= (others => '0');
                os_wr  <= '0';
                os_pol <= a7_pol;
                os_ev  <= '1';
            else
                os_phi <= v_s(15 downto 0);
                os_wr  <= v_s(16);
                if os_amp > rate then
                    os_amp <= os_amp - rate;
                else
                    os_amp <= (others => '0');
                end if;
                -- the wrap is registered and counted a pixel late
                if os_wr = '1' and os_n /= 15 then
                    os_n <= os_n + 1;
                end if;
            end if;
        end if;
    end process p_osc;

    -- ========================================================================
    -- LINE RENDER (a9..a12): flat ~7 px line at each wrap; per echo (and per
    -- 8-line segment) Chaos drops it or picks a half / 1.5x / 2x width; the
    -- envelope sets its weight.  The colour is captured beside the picture
    -- path at the a12 edge (p_lcol).
    -- ========================================================================
    p_line : process(clk)
        variable v_n  : unsigned(3 downto 0);
        variable v_in : std_logic;
        variable v_wc : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            if gv(10) = '0' then
                seg_a <= seg_line;
            end if;
            -- a9: this echo's index (a wrap still pending counts), its
            -- segment random, and the four width tests in parallel
            if os_wr = '1' and os_n /= 15 then
                v_n := os_n + 1;
            else
                v_n := os_n;
            end if;
            a9_rv <= seg_a + C_NM(to_integer(v_n));
            if os_phi < d05 then a9_c(0) <= '1'; else a9_c(0) <= '0'; end if;
            if os_phi < d10 then a9_c(1) <= '1'; else a9_c(1) <= '0'; end if;
            if os_phi < d15 then a9_c(2) <= '1'; else a9_c(2) <= '0'; end if;
            if os_phi < d20 then a9_c(3) <= '1'; else a9_c(3) <= '0'; end if;
            if os_amp(16) = '1' then
                a9_amp <= (others => '1');
            else
                a9_amp <= os_amp(15 downto 8);
            end if;

            -- a10: dropout and width class, line weight
            if a9_rv(13 downto 6) < chaos_l then
                v_wc := a9_rv(15 downto 14);
            else
                v_wc := "01";
            end if;
            v_in := a9_c(to_integer(v_wc));
            if v_in = '1' and a9_rv(7 downto 0) >= dropth_l then
                a10_r <= a9_amp;
            else
                a10_r <= (others => '0');
            end if;
            a11_r <= a10_r;
            a12_r <= a11_r;

            -- sync event and polarity carried to the a12 edge
            ev9  <= os_ev;  pl9  <= os_pol;
            ev10 <= ev9;    pl10 <= pl9;
            ev11 <= ev10;   pl11 <= pl10;
        end if;
    end process p_line;

    -- ========================================================================
    -- ECHO COLOUR, at the a12 edge.  Here b6 carries the sync pixel itself
    -- (final picture colour, boosted luma).  A falling edge echoes
    -- the brightest colour since the previous sync (its bright side); a
    -- rising edge follows the brightest of its first 8 pixels.
    -- ========================================================================
    p_lcol : process(clk)
    begin
        if rising_edge(clk) then
            if gv(13) = '0' then
                sm_y    <= C_BLK;
                sm_u    <= C_MID;
                sm_v    <= C_MID;
                lc_lock <= '1';
            elsif ev11 = '1' then
                if pl11 = '1' then
                    lc_y    <= b6_yb;
                    lc_u    <= b6_uc;
                    lc_v    <= b6_vc;
                    lc_lock <= '0';
                    lc_cnt  <= (others => '0');
                else
                    lc_y    <= sm_y;
                    lc_u    <= sm_u;
                    lc_v    <= sm_v;
                    lc_lock <= '1';
                end if;
                sm_y <= b6_yb;
                sm_u <= b6_uc;
                sm_v <= b6_vc;
            else
                if b6_yb > sm_y then
                    sm_y <= b6_yb;
                    sm_u <= b6_uc;
                    sm_v <= b6_vc;
                end if;
                if lc_lock = '0' then
                    if b6_yb > lc_y then
                        lc_y <= b6_yb;
                        lc_u <= b6_uc;
                        lc_v <= b6_vc;
                    end if;
                    lc_cnt <= lc_cnt + 1;
                    if lc_cnt = 7 then
                        lc_lock <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_lcol;

    -- ========================================================================
    -- PICTURE PATH (b1..b7): invert, hum, drive, peaks, colour, smear
    -- ========================================================================
    p_pic : process(clk)
        variable v_y   : signed(11 downto 0);
        variable v_t   : signed(21 downto 0);
        variable v_sum : signed(21 downto 0);
        variable v_yb  : unsigned(11 downto 0);
        variable v_e   : signed(21 downto 0);
        variable v_cu  : unsigned(7 downto 0);
        variable v_cv  : unsigned(7 downto 0);
        variable v_trd : unsigned(9 downto 0);
        variable v_dn  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- per-line values, taken in each stage's own blanking
            if gv(8) = '0' then
                hum_b <= hum_line;
            end if;
            if gv(12) = '0' then
                bsn_b <= bsn_line;
            end if;

            -- b1: fabric re-register of the rings
            b1_y <= unsigned(q_y(9 downto 0));
            b1_u <= unsigned(q_c(15 downto 8)) & "00";
            b1_v <= unsigned(q_c(7 downto 0)) & "00";

            -- b2: blank to neutral, invert, hum offset; Psyche palette index
            if gv(8) = '0' then
                b2_yo <= (others => '0');
                b2_u  <= C_MID;
                b2_v  <= C_MID;
            else
                if sw_inv = '1' then
                    v_y  := to_signed(1004, 12) - signed(resize(b1_y, 12));
                    b2_u <= not b1_u;
                    b2_v <= not b1_v;
                else
                    v_y  := signed(resize(b1_y, 12));
                    b2_u <= b1_u;
                    b2_v <= b1_v;
                end if;
                b2_yo <= v_y - 64 + resize(hum_b, 12);
            end if;
            b2_pi <= (b1_y(8 downto 2) & '0') + hpsy;

            -- b3: drive multiply, operand split lo/hi; Psyche ROM read
            b3_plo <= b2_yo * signed('0' & drive_g(4 downto 0));
            b3_phi <= b2_yo * signed('0' & drive_g(8 downto 5));
            b3_u   <= b2_u;
            b3_v   <= b2_v;
            romq_p <= C_HUE(to_integer(b2_pi));

            -- b4: product sum; ROM out re-registered
            b4_yp <= resize(b3_plo, 22) + shift_left(resize(b3_phi, 22), 5);
            b4_u  <= b3_u;
            b4_v  <= b3_v;
            b4_pu <= unsigned(romq_p(15 downto 8));
            b4_pv <= unsigned(romq_p(7 downto 0));

            -- b5: clamp to the legal range (and the same + 2 for the smear
            -- compare), overdrive excess past the knee, colour.  Psyche chroma
            -- fades toward neutral in the darks so black stays black.
            v_t   := shift_right(b4_yp, 6);
            v_sum := v_t + 64;

            if v_sum < 64 then
                b5_y  <= C_BLK;
                b5_y2 <= to_unsigned(66, 10);
            elsif v_sum > C_YMAX then
                b5_y  <= to_unsigned(C_YMAX, 10);
                b5_y2 <= to_unsigned(C_YMAX + 2, 10);
            else
                b5_y  <= unsigned(v_sum(9 downto 0));
                b5_y2 <= unsigned(v_sum(9 downto 0)) + 2;
            end if;
            v_e := v_t - (C_KNEE - 64);
            if v_e < 0 then
                b5_e <= (others => '0');
            elsif v_e > 1023 then
                b5_e <= (others => '1');
            else
                b5_e <= unsigned(v_e(9 downto 0));
            end if;
            if v_t < 96 then
                v_cu := chroma_sra(b4_pu, 2);
                v_cv := chroma_sra(b4_pv, 2);
            elsif v_t < 236 then
                v_cu := chroma_sra(b4_pu, 1);
                v_cv := chroma_sra(b4_pv, 1);
            else
                v_cu := b4_pu;
                v_cv := b4_pv;
            end if;
            if sw_psy = '1' then
                b5_uc <= v_cu & "00";
                b5_vc <= v_cv & "00";
            else
                b5_uc <= b4_u;
                b5_vc <= b4_v;
            end if;

            -- b6: smear level.  A decaying max-hold; the loop is one compare
            -- against the pre-added y + 2 beside a parallel decrement.
            if tr_y > to_unsigned(64 + C_TDEC, 10) then
                v_trd := tr_y - C_TDEC;
            else
                v_trd := C_BLK;
            end if;
            if gv(12) = '0' or sw_smear = '0' then
                tr_y   <= C_BLK;
                b6_dec <= '1';
                b6_y   <= b5_y;
            elsif b5_y2 >= tr_y then
                tr_y   <= b5_y;
                b6_dec <= '1';
                b6_y   <= b5_y;
            else
                tr_y   <= v_trd;
                b6_dec <= '0';
                b6_y   <= v_trd;
            end if;
            b6_uc <= b5_uc;
            b6_vc <= b5_vc;
            -- echo luma: the picture lifted a quarter of the way to white,
            -- (3y + 940) / 4, from the clamped luma (64..940 -> 283..940)
            v_yb  := resize(b5_y, 12) + shift_left(resize(b5_y, 12), 1) + 940;
            b6_yb <= v_yb(11 downto 2);
            -- snow density = overdrive excess / 4 + instability floor
            -- (+ a quarter more on torn lines)
            v_dn := resize(b5_e(9 downto 2), 10) + resize(snowfl, 10);
            if bsn_b = '1' then
                v_dn := v_dn + 64;
            end if;
            if v_dn > 255 then
                b6_dn <= (others => '1');
            else
                b6_dn <= v_dn(7 downto 0);
            end if;

            -- b7: smear colour.  The trail carries the colour of the pixel
            -- that set it, updated one pixel late (consistently).
            if b6_dec = '1' then
                tr_u <= b6_uc;
                tr_v <= b6_vc;
                b7_u <= b6_uc;
                b7_v <= b6_vc;
            else
                b7_u <= tr_u;
                b7_v <= tr_v;
            end if;
            b7_y <= b6_y;
        end if;
    end process p_pic;

    -- ========================================================================
    -- COMPOSITE (b7..b10): snow decision, lines lerped over the picture, snow
    -- ========================================================================
    p_comp : process(clk)
        variable v_by : unsigned(9 downto 0);
        variable v_bu : unsigned(9 downto 0);
        variable v_bv : unsigned(9 downto 0);
        variable v_sy : signed(10 downto 0);
        variable v_su : signed(10 downto 0);
        variable v_sv : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- snow dashes: one draw held for 4 px, phase random per line
            if gv(13) = '0' then
                sph_b <= sph_line;
                dcnt  <= sph_b;
            else
                dcnt <= dcnt + 1;
            end if;
            if dcnt = 0 then
                dash_r <= rng(23 downto 16);
                if rng(25 downto 24) = "00" then
                    dash_w <= '0';
                else
                    dash_w <= '1';
                end if;
            end if;
            -- b7: snow decision
            if gv(13) = '1' and dash_r < b6_dn then
                b7_sn <= '1';
            else
                b7_sn <= '0';
            end if;
            b7_sw <= dash_w;

            -- b8: echo colour minus picture (line weight a12 and colour lc are
            -- valid with b7).  Dim: picture luma to a quarter above black,
            -- chroma halved, so the echoes dominate.
            if sw_dim = '1' then
                v_by := shift_right(b7_y, 2) + 48;
                v_bu := shift_right(b7_u, 1) + 256;
                v_bv := shift_right(b7_v, 1) + 256;
            else
                v_by := b7_y;
                v_bu := b7_u;
                v_bv := b7_v;
            end if;
            b8_dy <= signed('0' & lc_y) - signed('0' & v_by);
            b8_du <= signed('0' & lc_u) - signed('0' & v_bu);
            b8_dv <= signed('0' & lc_v) - signed('0' & v_bv);
            b8_r  <= a12_r;
            b8_y  <= v_by;
            b8_u  <= v_bu;
            b8_v  <= v_bv;
            b8_sn <= b7_sn;
            b8_sw <= b7_sw;

            -- b9: lerp products (luma 8-bit weight, chroma 6-bit)
            b9_py <= b8_dy * signed('0' & b8_r);
            b9_pu <= b8_du * signed('0' & b8_r(7 downto 2));
            b9_pv <= b8_dv * signed('0' & b8_r(7 downto 2));
            b9_y  <= b8_y;
            b9_u  <= b8_u;
            b9_v  <= b8_v;
            b9_sn <= b8_sn;
            b9_sw <= b8_sw;

            -- b10: sum; snow over everything (white dashes faintly cyan)
            if b9_sn = '1' then
                if b9_sw = '1' then
                    x8_y <= to_unsigned(C_YMAX, 10);
                    x8_u <= to_unsigned(464, 10);
                    x8_v <= to_unsigned(544, 10);
                else
                    x8_y <= C_BLK;
                    x8_u <= C_MID;
                    x8_v <= C_MID;
                end if;
            else
                v_sy := signed('0' & b9_y) + resize(shift_right(b9_py, 8), 11);
                v_su := signed('0' & b9_u) + resize(shift_right(b9_pu, 6), 11);
                v_sv := signed('0' & b9_v) + resize(shift_right(b9_pv, 6), 11);
                x8_y <= unsigned(std_logic_vector(v_sy(9 downto 0)));
                x8_u <= unsigned(std_logic_vector(v_su(9 downto 0)));
                x8_v <= unsigned(std_logic_vector(v_sv(9 downto 0)));
            end if;
        end if;
    end process p_comp;

    -- ========================================================================
    -- OUTPUT CONDITIONING (HD HDMI bar guard) + blanking gate.
    -- Centre = the pixel two behind x8; [1 2 1]/4 on Y, U and V.  (v0.1's
    -- [1 3 3 1] 4:2:2 pair averaging was for 1-2 px coloured lines; the echo
    -- lines are 3.5 px and up since v0.2.)
    -- ========================================================================
    p_out : process(clk)
        variable v_y  : unsigned(11 downto 0);
        variable v_u  : unsigned(11 downto 0);
        variable v_v  : unsigned(11 downto 0);
        variable v_d  : unsigned(9 downto 0);
        variable v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            w0y <= x8_y;  w1y <= w0y;  w2y <= w1y;
            w0u <= x8_u;  w1u <= w0u;  w2u <= w1u;
            w0v <= x8_v;  w1v <= w0v;  w2v <= w1v;
            c9_ya <= ('0' & w2y) + ('0' & w0y);
            c9_yb <= w1y;
            c9_ua <= ('0' & w2u) + ('0' & w0u);
            c9_ub <= w1u;
            c9_va <= ('0' & w2v) + ('0' & w0v);
            c9_vb <= w1v;

            v_y := ('0' & c9_ya) + ("0" & c9_yb & '0');
            v_u := ('0' & c9_ua) + ("0" & c9_ub & '0');
            v_v := ('0' & c9_va) + ("0" & c9_vb & '0');
            c10_y <= v_y(11 downto 2);
            c10_u <= v_u(11 downto 2);
            c10_v <= v_v(11 downto 2);

            -- luma toward black: 64 + 13/16 (y - 64), white 940 -> ~775
            v_d  := c10_y - 64;
            v_yo := c10_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            -- gate on BOTH the generated and the source avid
            if q_s(4) = '1' and q_s(3) = '1' then
                out_y <= v_yo;
                out_u <= c10_u;
                out_v <= c10_v;
            else
                out_y <= C_BLK;
                out_u <= C_MID;
                out_v <= C_MID;
            end if;
            out_s <= q_s(4) & q_s(2 downto 0);
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(out_y);
    data_out.u       <= std_logic_vector(out_u);
    data_out.v       <= std_logic_vector(out_v);
    data_out.avid    <= out_s(3);
    data_out.hsync_n <= out_s(2);
    data_out.vsync_n <= out_s(1);
    data_out.field_n <= out_s(0);

end architecture haywire;
