-- Chroma Phaser v0.1.0: the colour of the picture slides off its luma on a
-- travelling sine wave, splits into its two chroma components, rings out in
-- resonant echo trains and swings in hue -- while the luma stays sharp and
-- exactly where it was.
--
-- Concept:
--   Chroma is written into a dual-bank line buffer (bank = line toggle) and
--   read back on the next line at a displaced column:
--     U(x) = buf(x - D_U),  D_U = A * wave(phase)
--     V(x) = buf(x - D_V),  D_V = A * wave(phase + Spread)
--     phase = frame drift (Rate) + Waves * line + Tilt * column (+ luma, Bend)
--   The read pair is rotated in hue by an angle that rides the same wave
--   (Hue Swing). Resonance writes a mix of the fresh input chroma and the
--   displaced, rotated, smoothed OUTPUT back into the buffer, so each line
--   carries the last one's echoes: echo k sits at the summed displacement
--   of k lines and is rotated by k swings -- ringing rainbow trains.
--   Out-of-range reads wrap around the measured width.
--
-- Raw video stays shallow: luma and syncs are 4 FFs from data_in to
-- data_out in every mode. The chroma read pipeline is 17 stages, so it reads
-- the previous line C_K = 16 px AHEAD of the luma (a fixed offset, not a
-- latency). The first 16 px of each line read with the column held at 0.
--
-- Chroma frame: the encoders pair Cb/Cr by counting from hsync, the core's
-- 4:4:4 conversion follows avid, so U/V labels swap on lines whose
-- hsync->avid parity differs. A line replays the previous line's chroma
-- (drape's trap), and hue rotation / Spread are not swap-symmetric, so
-- chroma is NORMALISED at input (swap when this line's parity is odd) and
-- de-normalised at output with the same line's parity.
--
-- Chroma pipeline (stage k holds output pixel p at clock T_p - C_K + k,
-- where T_p is the clock s0 holds pixel p):
--   P1   pU = phiX[15:6] + bend
--   P2   pV = pU + spread
--   P3   quarter-wave ROM indexes + signs
--   P4   sine ROMs (EBR)            P5  fabric register
--   P6   signed wave (square / mirror)
--   P7   D_U = A*sU, D_V = A*sV, th = swing*sU
--   P8   raw read column x + K - D; hue ROM index
--   P9   wrap at the width, block mask; hue ROM (EBR)
--   P10  RAM read address (bank = not line toggle)
--   P11  RAM (EBR)                  P12 fabric register
--   P13  rotation products          P14 sums
--   P15  saturate                   P17 [1 2 1] smoothing (cf)
--   P18  expand to 10 bits, de-normalise      P19 blanking gate = output
-- Feedback: W1 diff = cf - cin (T_p+2), W2 x resonance (T_p+3),
--   W3 cin + prod (T_p+4) = write data, written at column p of this
--   line's bank.
--
-- Register map:
--   registers_in(0)    K1 Rate       bipolar wave travel, centre = frozen
--   registers_in(1)    K2 Waves      wave cycles down the screen (exp)
--   registers_in(2)    K3 Tilt       bipolar column frequency (exp)
--   registers_in(3)    K4 Spread     U/V wave phase split 0..180 deg
--   registers_in(4)    K5 Resonance  echo feedback 0..0.94 (eased curve)
--   registers_in(5)    K6 Hue Swing  hue rotation riding the wave, 0..+/-180
--   registers_in(6)(0) S7  Shape     Sine / Square
--   registers_in(6)(1) S8  Mirror    wave mirrored about the screen centre
--   registers_in(6)(2) S9  Bend      luma bends the wave phase
--   registers_in(6)(3) S10 Glow      picture dimmed, chroma lights the luma
--   registers_in(6)(4) S11 Blocks    chroma read in W/30 blocks
--   registers_in(7)    P12 Depth     displacement amplitude, W/60..W/4 (exp)
--
-- BRAM: 2 x 4096x8 chroma line buffers (16 EBR) + 2 x 256x8 quarter-sine
-- ROMs + 256x16 cos/sin ROM = 19 EBR.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture chromaphaser of program_top is

    constant C_K        : integer := 16;
    constant C_Y_BLACK  : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_UV_ZERO  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_GLOW_CAP : unsigned(10 downto 0) := to_unsigned(800, 11);

    -- Quarter sine: Q(i) = round(255 * sin((i + 0.5) * pi / 512))
    type t_rom8 is array(0 to 255) of std_logic_vector(7 downto 0);
    constant C_QSIN : t_rom8 := (
        x"01", x"02", x"04", x"05", x"07", x"09", x"0A", x"0C", x"0D", x"0F", x"10", x"12", x"14", x"15", x"17", x"18",
        x"1A", x"1B", x"1D", x"1E", x"20", x"22", x"23", x"25", x"26", x"28", x"29", x"2B", x"2C", x"2E", x"2F", x"31",
        x"33", x"34", x"36", x"37", x"39", x"3A", x"3C", x"3D", x"3F", x"40", x"42", x"43", x"45", x"46", x"48", x"49",
        x"4B", x"4C", x"4E", x"4F", x"51", x"52", x"54", x"55", x"57", x"58", x"5A", x"5B", x"5D", x"5E", x"5F", x"61",
        x"62", x"64", x"65", x"67", x"68", x"69", x"6B", x"6C", x"6E", x"6F", x"71", x"72", x"73", x"75", x"76", x"78",
        x"79", x"7A", x"7C", x"7D", x"7E", x"80", x"81", x"82", x"84", x"85", x"86", x"88", x"89", x"8A", x"8C", x"8D",
        x"8E", x"90", x"91", x"92", x"93", x"95", x"96", x"97", x"99", x"9A", x"9B", x"9C", x"9E", x"9F", x"A0", x"A1",
        x"A2", x"A4", x"A5", x"A6", x"A7", x"A8", x"AA", x"AB", x"AC", x"AD", x"AE", x"AF", x"B0", x"B2", x"B3", x"B4",
        x"B5", x"B6", x"B7", x"B8", x"B9", x"BA", x"BB", x"BC", x"BD", x"BF", x"C0", x"C1", x"C2", x"C3", x"C4", x"C5",
        x"C6", x"C7", x"C8", x"C9", x"CA", x"CA", x"CB", x"CC", x"CD", x"CE", x"CF", x"D0", x"D1", x"D2", x"D3", x"D4",
        x"D4", x"D5", x"D6", x"D7", x"D8", x"D9", x"DA", x"DA", x"DB", x"DC", x"DD", x"DD", x"DE", x"DF", x"E0", x"E1",
        x"E1", x"E2", x"E3", x"E3", x"E4", x"E5", x"E6", x"E6", x"E7", x"E8", x"E8", x"E9", x"E9", x"EA", x"EB", x"EB",
        x"EC", x"EC", x"ED", x"EE", x"EE", x"EF", x"EF", x"F0", x"F0", x"F1", x"F1", x"F2", x"F2", x"F3", x"F3", x"F4",
        x"F4", x"F5", x"F5", x"F6", x"F6", x"F6", x"F7", x"F7", x"F8", x"F8", x"F8", x"F9", x"F9", x"F9", x"FA", x"FA",
        x"FA", x"FB", x"FB", x"FB", x"FB", x"FC", x"FC", x"FC", x"FC", x"FD", x"FD", x"FD", x"FD", x"FD", x"FE", x"FE",
        x"FE", x"FE", x"FE", x"FE", x"FE", x"FE", x"FF", x"FF", x"FF", x"FF", x"FF", x"FF", x"FF", x"FF", x"FF", x"FF"
    );

    -- Hue rotation: cos & sin of 2*pi*i/256, signed 8-bit x127
    type t_rom16 is array(0 to 255) of std_logic_vector(15 downto 0);
    constant C_HUE : t_rom16 := (
        x"7F00", x"7F03", x"7F06", x"7F09", x"7E0C", x"7E10", x"7E13", x"7D16",
        x"7D19", x"7C1C", x"7B1F", x"7A22", x"7A25", x"7928", x"782B", x"762E",
        x"7531", x"7433", x"7336", x"7139", x"703C", x"6F3F", x"6D41", x"6B44",
        x"6A47", x"6849", x"664C", x"644E", x"6251", x"6053", x"5E55", x"5C58",
        x"5A5A", x"585C", x"555E", x"5360", x"5162", x"4E64", x"4C66", x"4968",
        x"476A", x"446B", x"416D", x"3F6F", x"3C70", x"3971", x"3673", x"3374",
        x"3175", x"2E76", x"2B78", x"2879", x"257A", x"227A", x"1F7B", x"1C7C",
        x"197D", x"167D", x"137E", x"107E", x"0C7E", x"097F", x"067F", x"037F",
        x"007F", x"FD7F", x"FA7F", x"F77F", x"F47E", x"F07E", x"ED7E", x"EA7D",
        x"E77D", x"E47C", x"E17B", x"DE7A", x"DB7A", x"D879", x"D578", x"D276",
        x"CF75", x"CD74", x"CA73", x"C771", x"C470", x"C16F", x"BF6D", x"BC6B",
        x"B96A", x"B768", x"B466", x"B264", x"AF62", x"AD60", x"AB5E", x"A85C",
        x"A65A", x"A458", x"A255", x"A053", x"9E51", x"9C4E", x"9A4C", x"9849",
        x"9647", x"9544", x"9341", x"913F", x"903C", x"8F39", x"8D36", x"8C33",
        x"8B31", x"8A2E", x"882B", x"8728", x"8625", x"8622", x"851F", x"841C",
        x"8319", x"8316", x"8213", x"8210", x"820C", x"8109", x"8106", x"8103",
        x"8100", x"81FD", x"81FA", x"81F7", x"82F4", x"82F0", x"82ED", x"83EA",
        x"83E7", x"84E4", x"85E1", x"86DE", x"86DB", x"87D8", x"88D5", x"8AD2",
        x"8BCF", x"8CCD", x"8DCA", x"8FC7", x"90C4", x"91C1", x"93BF", x"95BC",
        x"96B9", x"98B7", x"9AB4", x"9CB2", x"9EAF", x"A0AD", x"A2AB", x"A4A8",
        x"A6A6", x"A8A4", x"ABA2", x"ADA0", x"AF9E", x"B29C", x"B49A", x"B798",
        x"B996", x"BC95", x"BF93", x"C191", x"C490", x"C78F", x"CA8D", x"CD8C",
        x"CF8B", x"D28A", x"D588", x"D887", x"DB86", x"DE86", x"E185", x"E484",
        x"E783", x"EA83", x"ED82", x"F082", x"F482", x"F781", x"FA81", x"FD81",
        x"0081", x"0381", x"0681", x"0981", x"0C82", x"1082", x"1382", x"1683",
        x"1983", x"1C84", x"1F85", x"2286", x"2586", x"2887", x"2B88", x"2E8A",
        x"318B", x"338C", x"368D", x"398F", x"3C90", x"3F91", x"4193", x"4495",
        x"4796", x"4998", x"4C9A", x"4E9C", x"519E", x"53A0", x"55A2", x"58A4",
        x"5AA6", x"5CA8", x"5EAB", x"60AD", x"62AF", x"64B2", x"66B4", x"68B7",
        x"6AB9", x"6BBC", x"6DBF", x"6FC1", x"70C4", x"71C7", x"73CA", x"74CD",
        x"75CF", x"76D2", x"78D5", x"79D8", x"7ADB", x"7ADE", x"7BE1", x"7CE4",
        x"7DE7", x"7DEA", x"7EED", x"7EF0", x"7EF4", x"7FF7", x"7FFA", x"7FFD"
    );

    -- Per-frame ease toward the knob: g += (k - g) / 8 (stops short of a
    -- rising target by < 8 LSB = free deadband against pot noise)
    function f_glide(g : unsigned(9 downto 0); k : unsigned(9 downto 0))
        return unsigned is
        variable d : signed(10 downto 0);
        variable r : signed(10 downto 0);
    begin
        d := signed('0' & k) - signed('0' & g);
        r := signed('0' & g) + shift_right(d, 3);
        return unsigned(r(9 downto 0));
    end function;

    -- Saturate a signed value to +/-127
    function f_sat8(v : signed) return signed is
    begin
        if v > 127 then
            return to_signed(127, 8);
        elsif v < -127 then
            return to_signed(-127, 8);
        else
            return resize(v, 8);
        end if;
    end function;

    -- 10-bit offset-binary chroma -> signed 8-bit (-127..127)
    function f_c8(c : std_logic_vector(9 downto 0)) return signed is
        variable v : signed(7 downto 0);
    begin
        v := signed((not c(9)) & c(8 downto 2));
        if v = -128 then
            v := to_signed(-127, 8);
        end if;
        return v;
    end function;

    ----------------------------------------------------------------------------
    -- Input register, counters, line/frame tracking
    ----------------------------------------------------------------------------
    signal s0_y, s0_u, s0_v : std_logic_vector(9 downto 0) := (others => '0');
    signal s0_avid      : std_logic := '0';
    signal s0_hs, s0_vs, s0_fld : std_logic := '1';
    signal prev_avid0   : std_logic := '0';
    signal prev_vs0     : std_logic := '1';
    signal din_prev_hs  : std_logic := '1';
    signal din_prev_av  : std_logic := '0';
    signal hpar         : std_logic := '0';  -- clock parity since hsync
    signal hap          : std_logic := '0';  -- hsync->avid parity, this line

    signal x_cnt        : unsigned(10 downto 0) := (others => '0');
    signal w_line       : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal w_r          : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal aline        : unsigned(10 downto 0) := (others => '0');
    signal h_cnt        : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal h_r          : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal saw_active   : std_logic := '0';
    signal frame_tick   : std_logic := '0';
    signal bank         : std_logic := '0';
    signal fl0          : std_logic := '0';  -- this line is the field's first

    ----------------------------------------------------------------------------
    -- Controls
    ----------------------------------------------------------------------------
    signal k1_r, k2_r, k3_r, k4_r, k5_r, k6_r, k12_r : unsigned(9 downto 0)
        := (others => '0');
    signal sq_r, mir_r, bend_r, glow_r, blk_r : std_logic := '0';

    signal g1  : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal g2  : unsigned(9 downto 0) := to_unsigned(420, 10);
    signal g3  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal g4  : unsigned(9 downto 0) := to_unsigned(160, 10);
    signal g5  : unsigned(9 downto 0) := to_unsigned(420, 10);
    signal g6  : unsigned(9 downto 0) := to_unsigned(200, 10);
    signal g12 : unsigned(9 downto 0) := to_unsigned(380, 10);

    -- Mode classes from the measured raster
    signal wcls         : unsigned(1 downto 0) := "00";  -- 1080 / 720 / SD
    signal hcls         : unsigned(1 downto 0) := "00";  -- 1080 / 720 / 480 / field

    -- Rate
    signal rt_m         : unsigned(8 downto 0) := (others => '0');
    signal rt_neg, rt_neg2, rt_neg3 : std_logic := '0';
    signal rt_v         : unsigned(13 downto 0) := (others => '0');
    signal rt_dz        : std_logic := '1';
    signal rt_w         : unsigned(12 downto 0) := (others => '0');
    signal rate_s       : signed(15 downto 0) := (others => '0');
    -- Tilt
    signal tl_m         : unsigned(8 downto 0) := (others => '0');
    signal tl_neg, tl_neg2, tl_neg3 : std_logic := '0';
    signal tl_v         : unsigned(13 downto 0) := (others => '0');
    signal tl_dz        : std_logic := '1';
    signal kx_hd        : unsigned(7 downto 0) := (others => '0');
    signal kx_u         : unsigned(9 downto 0) := (others => '0');
    signal kx_s, kx_n   : signed(15 downto 0) := (others => '0');
    -- Waves
    signal wv_v         : unsigned(14 downto 0) := (others => '0');
    signal ky_hd        : unsigned(10 downto 0) := (others => '0');
    signal ky_r         : unsigned(15 downto 0) := (others => '0');
    -- Depth
    signal dp_v         : unsigned(11 downto 0) := (others => '0');
    signal a_hd         : unsigned(8 downto 0) := (others => '0');
    signal a_r          : unsigned(8 downto 0) := (others => '0');
    -- Spread, swing, resonance
    signal sp10_r       : unsigned(9 downto 0) := (others => '0');
    signal hs8_r        : unsigned(7 downto 0) := (others => '0');
    signal rs_q2        : unsigned(11 downto 0) := (others => '0');
    signal rs_raw       : unsigned(5 downto 0) := (others => '0');
    signal f_r          : unsigned(5 downto 0) := (others => '0');
    signal f_eff        : unsigned(5 downto 0) := (others => '0');
    -- Geometry
    signal hm_r, hm1_r  : unsigned(10 downto 0) := to_unsigned(944, 11);
    signal w13          : signed(12 downto 0) := to_signed(1920, 13);

    ----------------------------------------------------------------------------
    -- Wave phase engine
    ----------------------------------------------------------------------------
    signal phi0         : unsigned(15 downto 0) := (others => '0');
    signal phiL         : unsigned(15 downto 0) := (others => '0');
    signal phiX         : unsigned(15 downto 0) := (others => '0');
    signal step_r       : signed(15 downto 0) := (others => '0');
    signal m_r          : std_logic := '0';

    ----------------------------------------------------------------------------
    -- Chroma read pipeline
    ----------------------------------------------------------------------------
    signal p1_pU        : unsigned(9 downto 0) := (others => '0');
    signal p1_x         : unsigned(10 downto 0) := (others => '0');
    signal p1_m         : std_logic := '0';
    signal p2_pU, p2_pV : unsigned(9 downto 0) := (others => '0');
    signal p2_x         : unsigned(10 downto 0) := (others => '0');
    signal p2_m         : std_logic := '0';
    signal p3_iU, p3_iV : unsigned(7 downto 0) := (others => '0');
    signal p3_nU, p3_nV : std_logic := '0';
    signal p3_x         : unsigned(10 downto 0) := (others => '0');
    signal p3_m         : std_logic := '0';
    signal romU_q, romV_q : std_logic_vector(7 downto 0) := (others => '0');
    signal p4_nU, p4_nV : std_logic := '0';
    signal p4_x         : unsigned(10 downto 0) := (others => '0');
    signal p4_m         : std_logic := '0';
    signal p5_qU, p5_qV : unsigned(7 downto 0) := (others => '0');
    signal p5_nU, p5_nV : std_logic := '0';
    signal p5_x         : unsigned(10 downto 0) := (others => '0');
    signal p5_m         : std_logic := '0';
    signal p6_sUr, p6_sUd, p6_sVd : signed(8 downto 0) := (others => '0');
    signal p6_x         : unsigned(10 downto 0) := (others => '0');
    signal p7_dU, p7_dV : signed(18 downto 0) := (others => '0');
    signal p7_th        : signed(17 downto 0) := (others => '0');
    signal p7_xk        : unsigned(11 downto 0) := (others => '0');
    signal p8_aU, p8_aV : signed(12 downto 0) := (others => '0');
    signal p8_th8       : unsigned(7 downto 0) := (others => '0');
    signal p9_aU, p9_aV : unsigned(10 downto 0) := (others => '0');
    signal hue_q        : std_logic_vector(15 downto 0) := (others => '0');
    signal p10_raU, p10_raV : unsigned(11 downto 0) := (others => '0');
    signal p10_h, p11_h : std_logic_vector(15 downto 0) := (others => '0');
    signal rdU, rdV     : std_logic_vector(7 downto 0) := (others => '0');
    signal p12_RU, p12_RV, p12_c, p12_s : signed(7 downto 0) := (others => '0');
    signal p13_uc, p13_vs, p13_us, p13_vc : signed(15 downto 0) := (others => '0');
    signal p14_u, p14_v : signed(16 downto 0) := (others => '0');
    signal p15_u, p15_v : signed(7 downto 0) := (others => '0');
    signal r1_u, r1_v, r2_u, r2_v : signed(7 downto 0) := (others => '0');
    signal cf_u, cf_v   : signed(7 downto 0) := (others => '0');
    signal p18_u, p18_v : unsigned(9 downto 0) := C_UV_ZERO;

    ----------------------------------------------------------------------------
    -- Feedback write path
    ----------------------------------------------------------------------------
    signal cin1_u, cin1_v, cin2_u, cin2_v, cin3_u, cin3_v : signed(7 downto 0)
        := (others => '0');
    signal w1_du, w1_dv : signed(8 downto 0) := (others => '0');
    signal w2_pu, w2_pv : signed(15 downto 0) := (others => '0');
    signal w3_bu, w3_bv : std_logic_vector(7 downto 0) := (others => '0');
    signal wx3          : unsigned(10 downto 0) := (others => '0');
    signal wa_r         : unsigned(11 downto 0) := (others => '0');
    signal we_r         : std_logic := '0';

    -- Chroma line buffers: {bank, column}, signed 8-bit (0 = neutral).
    -- Reads use the other bank than writes at every clock (the toggle is a
    -- single register), so no read ever meets a write: no_rw_check.
    type t_lb is array(0 to 4095) of std_logic_vector(7 downto 0);
    signal lbU : t_lb := (others => (others => '0'));
    signal lbV : t_lb := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbU, lbV : signal is true;

    ----------------------------------------------------------------------------
    -- Luma / sync path (4 FFs data_in -> data_out)
    ----------------------------------------------------------------------------
    signal l1_y, l2_y   : unsigned(9 downto 0) := C_Y_BLACK;
    signal gl_r         : unsigned(9 downto 0) := (others => '0');
    signal av1, av2, av3 : std_logic := '0';
    signal hs1, hs2, hs3 : std_logic := '1';
    signal vs1, vs2, vs3 : std_logic := '1';
    signal fd1, fd2, fd3 : std_logic := '1';
    signal out_y        : unsigned(9 downto 0) := C_Y_BLACK;
    signal out_u, out_v : unsigned(9 downto 0) := C_UV_ZERO;

begin

    main_p : process(clk)
        variable v_b      : signed(10 downto 0);
        variable v_mag    : signed(8 downto 0);
        variable v_tp     : signed(12 downto 0);
        variable v_tm     : signed(12 downto 0);
        variable v_a      : signed(12 downto 0);
        variable v_au     : unsigned(10 downto 0);
        variable v_t      : signed(16 downto 0);
        variable v_sm     : signed(10 downto 0);
        variable v_c      : signed(10 downto 0);
        variable v_wb     : signed(9 downto 0);
        variable v_sum    : unsigned(10 downto 0);
        variable v_absu   : unsigned(7 downto 0);
        variable v_absv   : unsigned(7 downto 0);
        variable v_x      : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -------------------------------------------------------------------
            -- Controls
            -------------------------------------------------------------------
            k1_r   <= unsigned(registers_in(0)(9 downto 0));
            k2_r   <= unsigned(registers_in(1)(9 downto 0));
            k3_r   <= unsigned(registers_in(2)(9 downto 0));
            k4_r   <= unsigned(registers_in(3)(9 downto 0));
            k5_r   <= unsigned(registers_in(4)(9 downto 0));
            k6_r   <= unsigned(registers_in(5)(9 downto 0));
            k12_r  <= unsigned(registers_in(7)(9 downto 0));
            sq_r   <= registers_in(6)(0);
            mir_r  <= registers_in(6)(1);
            bend_r <= registers_in(6)(2);
            glow_r <= registers_in(6)(3);
            blk_r  <= registers_in(6)(4);

            -------------------------------------------------------------------
            -- Input register; hsync->avid parity from data_in (ready for the
            -- line's first pixel at s0)
            -------------------------------------------------------------------
            s0_y    <= data_in.y;
            s0_u    <= data_in.u;
            s0_v    <= data_in.v;
            s0_avid <= data_in.avid;
            s0_hs   <= data_in.hsync_n;
            s0_vs   <= data_in.vsync_n;
            s0_fld  <= data_in.field_n;

            din_prev_hs <= data_in.hsync_n;
            din_prev_av <= data_in.avid;
            hpar <= not hpar;
            if data_in.hsync_n = '0' and din_prev_hs = '1' then
                hpar <= '0';
            end if;
            if data_in.avid = '1' and din_prev_av = '0' then
                hap <= hpar;
            end if;

            -------------------------------------------------------------------
            -- Counters (on s0). Column holds 0 through blanking. Lines count
            -- ACTIVE lines (advance at avid fall); vsync resets are
            -- idempotent, the per-field tick needs the saw-active guard.
            -------------------------------------------------------------------
            prev_avid0 <= s0_avid;
            prev_vs0   <= s0_vs;
            frame_tick <= '0';

            if s0_avid = '1' then
                x_cnt      <= x_cnt + 1;
                saw_active <= '1';
            else
                x_cnt <= (others => '0');
            end if;

            if s0_avid = '1' and prev_avid0 = '0' then
                bank <= not bank;
                if aline = 0 then
                    fl0 <= '1';
                else
                    fl0 <= '0';
                end if;
            end if;

            if s0_vs = '0' and prev_vs0 = '1' then
                aline <= (others => '0');
                if saw_active = '1' then
                    saw_active <= '0';
                    frame_tick <= '1';
                    phi0 <= phi0 + unsigned(rate_s);
                    phiL <= phi0 + unsigned(rate_s);
                else
                    phiL <= phi0;
                end if;
            elsif s0_avid = '0' and prev_avid0 = '1' then
                aline <= aline + 1;
                h_cnt <= aline + 1;
                phiL  <= phiL + ky_r;
                if x_cnt > to_unsigned(64, 11) then
                    w_line <= x_cnt;
                end if;
            end if;

            -------------------------------------------------------------------
            -- Per-field: glides, raster measure
            -------------------------------------------------------------------
            if frame_tick = '1' then
                g1  <= f_glide(g1, k1_r);
                g2  <= f_glide(g2, k2_r);
                g3  <= f_glide(g3, k3_r);
                g4  <= f_glide(g4, k4_r);
                g5  <= f_glide(g5, k5_r);
                g6  <= f_glide(g6, k6_r);
                g12 <= f_glide(g12, k12_r);
                w_r <= w_line;
                if h_cnt > to_unsigned(32, 11) then
                    h_r <= h_cnt;
                end if;
            end if;

            -------------------------------------------------------------------
            -- Derived per-field values (quasi-static, one op per stage)
            -------------------------------------------------------------------
            if w_r >= to_unsigned(1600, 11) then
                wcls <= "00";
            elsif w_r >= to_unsigned(1000, 11) then
                wcls <= "01";
            else
                wcls <= "10";
            end if;
            if h_r >= to_unsigned(900, 11) then
                hcls <= "00";
            elsif h_r >= to_unsigned(600, 11) then
                hcls <= "01";
            elsif h_r >= to_unsigned(400, 11) then
                hcls <= "10";
            else
                hcls <= "11";
            end if;
            hm_r  <= ("0" & w_r(10 downto 1)) - to_unsigned(C_K, 11);
            hm1_r <= hm_r - 1;
            w13   <= signed(resize(w_r, 13));

            -- Rate: |k - 512| -> (64 + m) << e - 64, 0..16192, /2
            if g1(9) = '1' then
                rt_m <= g1(8 downto 0);
            else
                rt_m <= not g1(8 downto 0);
            end if;
            rt_neg  <= not g1(9);
            rt_v    <= shift_left(resize(to_unsigned(64, 7) + resize(rt_m(5 downto 0), 7), 14),
                                  to_integer(rt_m(8 downto 6)));
            if rt_m < to_unsigned(12, 9) then
                rt_dz <= '1';
            else
                rt_dz <= '0';
            end if;
            rt_neg2 <= rt_neg;
            if rt_dz = '1' then
                rt_w <= (others => '0');
            else
                rt_w <= resize(shift_right(rt_v - 64, 1), 13);
            end if;
            rt_neg3 <= rt_neg2;
            if rt_neg3 = '1' then
                rate_s <= -signed(resize(rt_w, 16));
            else
                rate_s <= signed(resize(rt_w, 16));
            end if;

            -- Tilt: same curve, /64 -> 0..253 per px (7.4 cycles over 1920)
            if g3(9) = '1' then
                tl_m <= g3(8 downto 0);
            else
                tl_m <= not g3(8 downto 0);
            end if;
            tl_neg  <= not g3(9);
            tl_v    <= shift_left(resize(to_unsigned(64, 7) + resize(tl_m(5 downto 0), 7), 14),
                                  to_integer(tl_m(8 downto 6)));
            if tl_m < to_unsigned(12, 9) then
                tl_dz <= '1';
            else
                tl_dz <= '0';
            end if;
            tl_neg2 <= tl_neg;
            if tl_dz = '1' then
                kx_hd <= (others => '0');
            else
                kx_hd <= resize(shift_right(tl_v - 64, 6), 8);
            end if;
            tl_neg3 <= tl_neg2;
            case wcls is
                when "00"   => kx_u <= resize(kx_hd, 10);
                when "01"   => kx_u <= resize(kx_hd, 10) + resize(kx_hd(7 downto 1), 10);
                when others => kx_u <= shift_left(resize(kx_hd, 10), 1)
                                     + resize(kx_hd(7 downto 1), 10);
            end case;
            if tl_neg3 = '1' then
                kx_s <= -signed(resize(kx_u, 16));
                kx_n <= signed(resize(kx_u, 16));
            else
                kx_s <= signed(resize(kx_u, 16));
                kx_n <= -signed(resize(kx_u, 16));
            end if;

            -- Waves: (128 + m) << e, /16 -> 8..2040 per line at 1080
            wv_v  <= shift_left(resize(to_unsigned(128, 8) + resize(g2(6 downto 0), 8), 15),
                                to_integer(g2(9 downto 7)));
            ky_hd <= wv_v(14 downto 4);
            case hcls is
                when "00"   => ky_r <= resize(ky_hd, 16);
                when "01"   => ky_r <= resize(ky_hd, 16) + resize(ky_hd(10 downto 1), 16);
                when "10"   => ky_r <= shift_left(resize(ky_hd, 16), 1);
                when others => ky_r <= shift_left(resize(ky_hd, 16), 2);
            end case;

            -- Depth: (256 + m) << e, /8 -> 32..511 px at 1920 wide
            dp_v <= shift_left(resize(to_unsigned(256, 9) + resize(g12(7 downto 0), 9), 12),
                               to_integer(g12(9 downto 8)));
            a_hd <= dp_v(11 downto 3);
            case wcls is
                when "00"   => a_r <= a_hd;
                when "01"   => a_r <= ("0" & a_hd(8 downto 1)) + ("000" & a_hd(8 downto 3))
                                    + ("0000" & a_hd(8 downto 4));
                when others => a_r <= ("00" & a_hd(8 downto 2)) + ("000" & a_hd(8 downto 3));
            end case;

            sp10_r <= "0" & g4(9 downto 1);
            hs8_r  <= g6(9 downto 2);

            -- Resonance: f = 62 - (63 - k)^2 / 64, capped at 60 (/64)
            rs_q2  <= (not g5(9 downto 4)) * (not g5(9 downto 4));
            rs_raw <= to_unsigned(62, 6) - rs_q2(11 downto 6);
            if rs_raw > to_unsigned(60, 6) then
                f_r <= to_unsigned(60, 6);
            else
                f_r <= rs_raw;
            end if;
            if fl0 = '1' then
                f_eff <= (others => '0');
            else
                f_eff <= f_r;
            end if;

            -------------------------------------------------------------------
            -- Wave phase across the line. step_r/m_r are for the column
            -- x_cnt holds NEXT clock (Mirror flips the step at hm, so the
            -- phase is symmetric about output column W/2).
            -------------------------------------------------------------------
            if mir_r = '1' and x_cnt >= hm1_r then
                step_r <= kx_n;
                m_r    <= '1';
            else
                step_r <= kx_s;
                m_r    <= '0';
            end if;
            if s0_avid = '1' then
                phiX <= phiX + unsigned(step_r);
            else
                phiX <= phiL;
            end if;

            -------------------------------------------------------------------
            -- P1..P3: phases, quarter-wave indexes
            -------------------------------------------------------------------
            if bend_r = '1' then
                p1_pU <= phiX(15 downto 6) + unsigned(s0_y);
            else
                p1_pU <= phiX(15 downto 6);
            end if;
            p1_x <= x_cnt;
            p1_m <= m_r;

            p2_pU <= p1_pU;
            p2_pV <= p1_pU + sp10_r;
            p2_x  <= p1_x;
            p2_m  <= p1_m;

            if p2_pU(8) = '1' then
                p3_iU <= not p2_pU(7 downto 0);
            else
                p3_iU <= p2_pU(7 downto 0);
            end if;
            if p2_pV(8) = '1' then
                p3_iV <= not p2_pV(7 downto 0);
            else
                p3_iV <= p2_pV(7 downto 0);
            end if;
            p3_nU <= p2_pU(9);
            p3_nV <= p2_pV(9);
            p3_x  <= p2_x;
            p3_m  <= p2_m;

            -- P4: ROMs (rom processes below)
            p4_nU <= p3_nU;
            p4_nV <= p3_nV;
            p4_x  <= p3_x;
            p4_m  <= p3_m;

            -- P5: fabric register on ROM data
            p5_qU <= unsigned(romU_q);
            p5_qV <= unsigned(romV_q);
            p5_nU <= p4_nU;
            p5_nV <= p4_nV;
            p5_x  <= p4_x;
            p5_m  <= p4_m;

            -------------------------------------------------------------------
            -- P6: signed wave. Square = full swing with the sine's sign.
            -- Mirror negates the displacement (not the hue angle) on the
            -- right half.
            -------------------------------------------------------------------
            if sq_r = '1' then
                v_mag := to_signed(255, 9);
            else
                v_mag := signed('0' & p5_qU);
            end if;
            if p5_nU = '1' then
                p6_sUr <= -v_mag;
            else
                p6_sUr <= v_mag;
            end if;
            if (p5_nU xor p5_m) = '1' then
                p6_sUd <= -v_mag;
            else
                p6_sUd <= v_mag;
            end if;
            if sq_r = '1' then
                v_mag := to_signed(255, 9);
            else
                v_mag := signed('0' & p5_qV);
            end if;
            if (p5_nV xor p5_m) = '1' then
                p6_sVd <= -v_mag;
            else
                p6_sVd <= v_mag;
            end if;
            p6_x <= p5_x;

            -- P7: products
            p7_dU <= signed('0' & a_r) * p6_sUd;
            p7_dV <= signed('0' & a_r) * p6_sVd;
            p7_th <= signed('0' & hs8_r) * p6_sUr;
            p7_xk <= resize(p6_x, 12) + to_unsigned(C_K, 12);

            -- P8: raw read columns; hue angle (+/-127 = +/-179 deg)
            p8_aU  <= signed(resize(p7_xk, 13)) - resize(shift_right(p7_dU, 8), 13);
            p8_aV  <= signed(resize(p7_xk, 13)) - resize(shift_right(p7_dV, 8), 13);
            p8_th8 <= unsigned(p7_th(16 downto 9));

            -------------------------------------------------------------------
            -- P9: wrap into 0..W-1 (|D| + K < W, one correction suffices);
            -- Blocks masks the column to 64 px (1080) / 32 px blocks
            -------------------------------------------------------------------
            v_tp := p8_aU + w13;
            v_tm := p8_aU - w13;
            if p8_aU(12) = '1' then
                v_a := v_tp;
            elsif v_tm(12) = '0' then
                v_a := v_tm;
            else
                v_a := p8_aU;
            end if;
            v_au := unsigned(v_a(10 downto 0));
            if blk_r = '1' then
                v_au(4 downto 0) := (others => '0');
                if wcls = "00" then
                    v_au(5) := '0';
                end if;
            end if;
            p9_aU <= v_au;

            v_tp := p8_aV + w13;
            v_tm := p8_aV - w13;
            if p8_aV(12) = '1' then
                v_a := v_tp;
            elsif v_tm(12) = '0' then
                v_a := v_tm;
            else
                v_a := p8_aV;
            end if;
            v_au := unsigned(v_a(10 downto 0));
            if blk_r = '1' then
                v_au(4 downto 0) := (others => '0');
                if wcls = "00" then
                    v_au(5) := '0';
                end if;
            end if;
            p9_aV <= v_au;

            -- P10: RAM read address in the previous line's bank
            p10_raU <= (not bank) & p9_aU;
            p10_raV <= (not bank) & p9_aV;
            p10_h   <= hue_q;

            -- P11: RAM (ram processes below)
            p11_h <= p10_h;

            -- P12: fabric register on RAM data
            p12_RU <= signed(rdU);
            p12_RV <= signed(rdV);
            p12_c  <= signed(p11_h(15 downto 8));
            p12_s  <= signed(p11_h(7 downto 0));

            -------------------------------------------------------------------
            -- P13..P15: hue rotation (U' = U c - V s, V' = U s + V c)
            -------------------------------------------------------------------
            p13_uc <= p12_RU * p12_c;
            p13_vs <= p12_RV * p12_s;
            p13_us <= p12_RU * p12_s;
            p13_vc <= p12_RV * p12_c;

            p14_u <= resize(p13_uc, 17) - resize(p13_vs, 17);
            p14_v <= resize(p13_us, 17) + resize(p13_vc, 17);

            v_t   := shift_right(p14_u + 64, 7);
            p15_u <= f_sat8(v_t);
            v_t   := shift_right(p14_v + 64, 7);
            p15_v <= f_sat8(v_t);

            -------------------------------------------------------------------
            -- [1 2 1] smoothing: cf = pixel p at T_p+1
            -------------------------------------------------------------------
            r1_u <= p15_u;
            r1_v <= p15_v;
            r2_u <= r1_u;
            r2_v <= r1_v;
            v_sm := resize(p15_u, 11) + shift_left(resize(r1_u, 11), 1)
                  + resize(r2_u, 11) + 2;
            cf_u <= resize(shift_right(v_sm, 2), 8);
            v_sm := resize(p15_v, 11) + shift_left(resize(r1_v, 11), 1)
                  + resize(r2_v, 11) + 2;
            cf_v <= resize(shift_right(v_sm, 2), 8);

            -- P18: 10-bit offset binary, de-normalised to this line's labels
            v_c := to_signed(514, 11) + shift_left(resize(cf_u, 11), 2);
            if hap = '1' then
                p18_v <= unsigned(v_c(9 downto 0));
            else
                p18_u <= unsigned(v_c(9 downto 0));
            end if;
            v_c := to_signed(514, 11) + shift_left(resize(cf_v, 11), 2);
            if hap = '1' then
                p18_u <= unsigned(v_c(9 downto 0));
            else
                p18_v <= unsigned(v_c(9 downto 0));
            end if;

            -------------------------------------------------------------------
            -- Feedback: B = cin + f * (cf - cin), written to this line's bank
            -------------------------------------------------------------------
            if hap = '1' then
                cin1_u <= f_c8(s0_v);
                cin1_v <= f_c8(s0_u);
            else
                cin1_u <= f_c8(s0_u);
                cin1_v <= f_c8(s0_v);
            end if;
            cin2_u <= cin1_u;
            cin2_v <= cin1_v;
            cin3_u <= cin2_u;
            cin3_v <= cin2_v;

            w1_du <= resize(cf_u, 9) - resize(cin1_u, 9);
            w1_dv <= resize(cf_v, 9) - resize(cin1_v, 9);

            w2_pu <= w1_du * signed('0' & f_eff);
            w2_pv <= w1_dv * signed('0' & f_eff);

            v_wb  := resize(cin3_u, 10) + resize(shift_right(w2_pu + 32, 6), 10);
            w3_bu <= std_logic_vector(f_sat8(v_wb));
            v_wb  := resize(cin3_v, 10) + resize(shift_right(w2_pv + 32, 6), 10);
            w3_bv <= std_logic_vector(f_sat8(v_wb));

            if av3 = '1' then
                wx3 <= wx3 + 1;
            else
                wx3 <= (others => '0');
            end if;
            wa_r <= bank & wx3;
            we_r <= av3;

            -------------------------------------------------------------------
            -- Luma: Glow dims the picture to half and adds 2(|U|+|V|) of the
            -- displaced chroma, capped at 800
            -------------------------------------------------------------------
            if glow_r = '1' then
                v_x  := unsigned(s0_y);
                l1_y <= resize(shift_right(resize(v_x, 11) + 64, 1), 10);
            else
                l1_y <= unsigned(s0_y);
            end if;
            l2_y <= l1_y;
            if glow_r = '1' then
                if cf_u < 0 then
                    v_absu := unsigned(-cf_u);
                else
                    v_absu := unsigned(cf_u);
                end if;
                if cf_v < 0 then
                    v_absv := unsigned(-cf_v);
                else
                    v_absv := unsigned(cf_v);
                end if;
                gl_r <= shift_left(resize(v_absu, 10) + resize(v_absv, 10), 1);
            else
                gl_r <= (others => '0');
            end if;

            -------------------------------------------------------------------
            -- Output register with blanking gate (avid of the same pixel)
            -------------------------------------------------------------------
            v_sum := resize(l2_y, 11) + resize(gl_r, 11);
            if av2 = '0' then
                out_y <= C_Y_BLACK;
                out_u <= C_UV_ZERO;
                out_v <= C_UV_ZERO;
            else
                if glow_r = '1' and v_sum > C_GLOW_CAP then
                    out_y <= C_GLOW_CAP(9 downto 0);
                else
                    out_y <= v_sum(9 downto 0);
                end if;
                out_u <= p18_u;
                out_v <= p18_v;
            end if;

            -------------------------------------------------------------------
            -- Sync delay (s0 + 3 = output)
            -------------------------------------------------------------------
            av1 <= s0_avid;
            av2 <= av1;
            av3 <= av2;
            hs1 <= s0_hs;
            hs2 <= hs1;
            hs3 <= hs2;
            vs1 <= s0_vs;
            vs2 <= vs1;
            vs3 <= vs2;
            fd1 <= s0_fld;
            fd2 <= fd1;
            fd3 <= fd2;
        end if;
    end process main_p;

    ----------------------------------------------------------------------------
    -- ROMs (registered address in, registered data out)
    ----------------------------------------------------------------------------
    rom_u_p : process(clk)
    begin
        if rising_edge(clk) then
            romU_q <= C_QSIN(to_integer(p3_iU));
        end if;
    end process rom_u_p;

    rom_v_p : process(clk)
    begin
        if rising_edge(clk) then
            romV_q <= C_QSIN(to_integer(p3_iV));
        end if;
    end process rom_v_p;

    rom_h_p : process(clk)
    begin
        if rising_edge(clk) then
            hue_q <= C_HUE(to_integer(p8_th8));
        end if;
    end process rom_h_p;

    ----------------------------------------------------------------------------
    -- Line buffers: one write, one unconditional read each
    ----------------------------------------------------------------------------
    ram_u_p : process(clk)
    begin
        if rising_edge(clk) then
            if we_r = '1' then
                lbU(to_integer(wa_r)) <= w3_bu;
            end if;
            rdU <= lbU(to_integer(p10_raU));
        end if;
    end process ram_u_p;

    ram_v_p : process(clk)
    begin
        if rising_edge(clk) then
            if we_r = '1' then
                lbV(to_integer(wa_r)) <= w3_bv;
            end if;
            rdV <= lbV(to_integer(p10_raV));
        end if;
    end process ram_v_p;

    data_out.hsync_n <= hs3;
    data_out.vsync_n <= vs3;
    data_out.field_n <= fd3;
    data_out.avid    <= av3;
    data_out.y       <= std_logic_vector(out_y);
    data_out.u       <= std_logic_vector(out_u);
    data_out.v       <= std_logic_vector(out_v);

end architecture chromaphaser;
