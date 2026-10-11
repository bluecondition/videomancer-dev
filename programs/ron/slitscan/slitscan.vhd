-- slitscan.vhd  (v2.0.0 -- "Slit Scan": a true time-strip slit-scan)
--
-- A slit -- one column of the picture (Vertical) or one row (Horizontal) --
-- is sampled every frame and pushed into a history canvas in EBR.  The
-- history flows away from the slit across the screen, so one axis of the
-- output is TIME: static scenery becomes streaks, anything moving through
-- the slit is drawn as a time-warped shape.  (v1 ran a 2048-sample delay
-- line -- less than one HD line -- so it only ever slid bands sideways.)
--
-- Canvas: 128 time columns x 128 slit cells of 4-bit luma (16 EBR) and
-- 128 x 32 cells of U4 V4 chroma (8 EBR), chroma stored canonical and
-- signed, so zero RAM = black + neutral.  Each canvas is split into 4
-- sub-banks by (time parity, slit parity): one registered read per
-- sub-bank gives all four bilinear taps (the feedback store).  The time
-- axis is circular: physical column p_h is the newest committed column,
-- older ones sit at p_h+1, p_h+2, ... so a pixel's age is an address
-- offset.  Columns are written at p_h-1 (and p_h-2); the display never
-- reads past age 125, so a written column is never read (no_rw_check).
--
-- Capture: Vertical -- an 8 px (4 px below 1024 wide) window at the slit,
-- box-summed over the first 2^kl lines of each slit cell; Horizontal --
-- the first line at or below the slit, box-summed over the first 2^kw px
-- of each cell.  Samples are pre-shifted so every sum lands at the same
-- scale.  Each cell then runs an exposure IIR held in a 256x16
-- accumulator RAM (pipelined read-modify-write), and on commit frames the
-- IIR value is written, stochastically rounded, into the next column.
-- Rainbow turns each new column's chroma by a per-column angle on the way
-- in (one multiplier, serial), so the hue stripes scroll with the history.
--
-- Scroll: a per-frame phase accumulator (P12 Speed, 1/32 .. 2 columns
-- per frame) commits a column on each carry; the display subtracts the
-- fraction, so the history glides smoothly at any speed.
--
-- Read: lookahead DDAs (C_LOOK px ahead of the raster) give the time and
-- slit coordinates in canvas units; layout / flow / wave / time key build
-- the age; the age picks two columns (bilinear in time), the slit
-- coordinate two cells (bilinear along the slit); fade acts on the age.
-- The live picture comes from a 16-deep EBR delay (7 clocks); output
-- latency 9 in every mode.
--
-- Controls:
--   K1 Slit       slit position (glided)
--   K2 Time Key   bipolar: right = bright parts of the live picture reach
--                 further back in time, left = dark parts do; centre off
--   K3 Exposure   motion blur of each new column (instant .. ~1 s)
--   K4 Fade       history darkens with age
--   K5 Rainbow    bipolar: each new column's hue turns a step further
--                 (+ colour from luma); centre off
--   K6 Layout     Right/Down, Left/Up, Mirror, Full
--   S7 Axis       Vertical slit (time runs sideways) / Horizontal slit
--   S8 Flow       Out (newest at the slit) / In (newest at the far edge)
--   S9 Pixels     Smooth / Blocks (nearest cell)
--   S10 Ghost     live picture lightened over the history
--   S11 Wave      the slit bends into a travelling sine wave
--   P12 Speed     how fast the history flows
--
-- Author: ron

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture slitscan of program_top is

    constant C_PIPE : natural := 8;      -- sync / live taps pipe(0..7); output latency 9
    constant C_LOOK : natural := 9;      -- read DDA lookahead = 18 read stages - 9
                                         -- (p_seq steps 4/5 hard-code 9.5 kx)

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- collision-free RAMs: skip yosys's read-during-write emulation
    attribute no_rw_check : boolean;

    -- bipolar knob: (|k - 512| - 16-code centre deadband) / 8 -> 0..62
    function bip6(k : unsigned(9 downto 0)) return unsigned is
        variable d : signed(10 downto 0);
        variable a : signed(10 downto 0);
        variable m : unsigned(9 downto 0);
    begin
        d := signed('0' & k) - to_signed(512, 11);
        if d < 0 then a := -d; else a := d; end if;
        if a > 16 then
            m := unsigned(a(9 downto 0)) - 16;
        else
            m := (others => '0');
        end if;
        return m(8 downto 3);
    end function;

    -- signed d times a 4-bit weight: AND-gated partials, balanced adds
    function smul4(d : signed; w : unsigned(3 downto 0); n : natural) return signed is
        variable dd, p0, p1, p2, p3 : signed(n - 1 downto 0);
    begin
        dd := resize(d, n);
        p0 := (others => '0'); p1 := (others => '0');
        p2 := (others => '0'); p3 := (others => '0');
        if w(0) = '1' then p0 := dd; end if;
        if w(1) = '1' then p1 := shift_left(dd, 1); end if;
        if w(2) = '1' then p2 := shift_left(dd, 2); end if;
        if w(3) = '1' then p3 := shift_left(dd, 3); end if;
        return (p0 + p1) + (p2 + p3);
    end function;

    --------------------------------------------------------------------------
    -- Control decode (free-running).
    --------------------------------------------------------------------------
    signal k_slit : unsigned(15 downto 0) := to_unsigned(16128, 16);   -- Q8.8 columns
    signal k_key  : unsigned(5 downto 0) := (others => '0');
    signal k_kdark : std_logic := '0';
    signal k_erev : unsigned(10 downto 0) := to_unsigned(700, 11);     -- 1024 - K3
    signal k_fade : unsigned(5 downto 0) := (others => '0');
    signal k_rain : unsigned(5 downto 0) := (others => '0');
    signal k_rneg : std_logic := '0';
    signal k_lay  : unsigned(1 downto 0) := "00";
    signal k_spd  : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal sw_axis, sw_flow, sw_blk, sw_ghost, sw_wave : std_logic := '0';

    --------------------------------------------------------------------------
    -- Input raster, measurement, frame anchor.
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to C_PIPE - 1);

    signal prev_hsync_n, prev_vsync_n, prev_avid : std_logic := '1';
    signal lfsr : unsigned(15 downto 0) := x"ACE1";
    signal px   : unsigned(11 downto 0) := (others => '0');
    signal py   : unsigned(10 downto 0) := (others => '0');
    signal line_act, frame_act, frame_armed : std_logic := '0';
    signal w_meas : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal h_meas : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal g_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal g_h    : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal anchor : std_logic := '0';
    signal hpar, line_par : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-frame sequencer (vblank): one serial multiplier, one divider.
    --------------------------------------------------------------------------
    signal sq     : unsigned(3 downto 0) := (others => '0');
    signal ma, mb : unsigned(10 downto 0) := (others => '0');
    signal macc   : unsigned(21 downto 0) := (others => '0');
    signal mcnt   : unsigned(3 downto 0) := (others => '0');
    signal dden   : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal drem   : unsigned(11 downto 0) := (others => '0');
    signal dq     : unsigned(16 downto 0) := (others => '0');
    signal dcnt   : unsigned(4 downto 0) := (others => '0');
    signal commit : std_logic := '0';
    signal g_d    : signed(16 downto 0) := (others => '0');            -- slit glide step
    signal g_go   : std_logic := '0';
    signal sx     : unsigned(6 downto 0) := (others => '0');           -- parabola operand

    signal F      : unsigned(15 downto 0) := (others => '0');            -- scroll fraction
    signal cpend  : unsigned(1 downto 0) := (others => '0');             -- columns written last frame
    signal p_h    : unsigned(6 downto 0) := (others => '0');             -- newest committed column
    signal delta  : unsigned(17 downto 0) := to_unsigned(32768, 18);    -- columns / frame (Q16)
    signal phi    : unsigned(9 downto 0) := (others => '0');             -- rainbow angle, Q8.2

    -- per-frame values used by the pixel paths
    signal kx     : unsigned(16 downto 0) := to_unsigned(4369, 17);      -- 2^23 / W
    signal ky     : unsigned(16 downto 0) := to_unsigned(7767, 17);      -- 2^23 / H
    signal c_x0   : unsigned(23 downto 0) := to_unsigned(41505, 24);     -- (C_LOOK + 1/2) kx
    signal c_xh   : unsigned(23 downto 0) := to_unsigned(2184, 24);      -- kx / 2
    signal c_x4   : unsigned(23 downto 0) := to_unsigned(19660, 24);     -- 4.5 kx
    signal c_y0   : unsigned(23 downto 0) := to_unsigned(3883, 24);      -- ky / 2
    signal c_kw   : std_logic := '1';                                    -- window 8 px (else 4)
    signal c_kl   : unsigned(1 downto 0) := "11";                        -- 2^kl lines per V cell
    signal c_nsh  : unsigned(2 downto 0) := "000";                       -- sample pre-shift
    signal c_m    : unsigned(6 downto 0) := to_unsigned(16, 7);          -- exposure, /64
    signal c_S    : unsigned(15 downto 0) := to_unsigned(16128, 16);     -- glided slit, Q8.8
    signal c_F    : unsigned(7 downto 0) := (others => '0');
    signal c_pw, c_pw2 : unsigned(6 downto 0) := (others => '0');
    signal c_wr, c_dbl : std_logic := '0';
    signal c_phw  : unsigned(7 downto 0) := (others => '0');
    signal c_axis, c_flow, c_blk, c_ghost, c_wave : std_logic := '0';
    signal c_kdark, c_bloom : std_logic := '0';
    signal c_lay  : unsigned(1 downto 0) := "00";
    signal c_key, c_fade : unsigned(5 downto 0) := (others => '0');
    signal c_rC   : signed(6 downto 0) := to_signed(32, 7);              -- this frame's column hue
    signal c_rS   : signed(6 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- DDAs: read side runs C_LOOK px ahead; x_in is the input raster.
    --------------------------------------------------------------------------
    signal x_acc, x_in, x_s, y_line : unsigned(23 downto 0) := (others => '0');

    -- live picture delay: a 16-deep EBR circular buffer (not the FF pipe),
    -- read 5 behind the write + EBR register + fabric register = 7 clocks
    type t_vdly is array (0 to 15) of std_logic_vector(31 downto 0);
    signal vdly_ram : t_vdly := (others => (others => '0'));
    signal vdly_wa  : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal vdly_ra  : unsigned(3 downto 0) := to_unsigned(11, 4);
    signal vdly_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal vdly_q2  : std_logic_vector(31 downto 0) := (others => '0');
    attribute no_rw_check of vdly_ram : signal is true;

    --------------------------------------------------------------------------
    -- Read pipeline R1..R18.
    --------------------------------------------------------------------------
    signal r1_t   : unsigned(15 downto 0) := (others => '0');
    signal r1_kl  : unsigned(7 downto 0) := (others => '0');
    signal r1_wph : unsigned(7 downto 0) := (others => '0');

    signal r2_t   : unsigned(15 downto 0) := (others => '0');
    signal r2_key : unsigned(13 downto 0) := (others => '0');
    signal r2_wp  : unsigned(13 downto 0) := (others => '0');
    signal r2_wx  : unsigned(6 downto 0) := (others => '0');
    signal r2_wn  : std_logic := '0';

    signal r3_d1, r3_d2, r3_d3, r3_t : signed(16 downto 0) := (others => '0');
    signal r3_kf, r3_w : signed(16 downto 0) := (others => '0');

    signal r4_base : signed(16 downto 0) := (others => '0');
    signal r4_e    : signed(17 downto 0) := (others => '0');
    signal r4_p1, r4_p3 : std_logic := '0';

    signal r5_a    : signed(18 downto 0) := (others => '0');
    signal r5_hist : std_logic := '0';

    signal r6_ia   : unsigned(6 downto 0) := (others => '0');
    signal r6_fa   : unsigned(3 downto 0) := (others => '0');
    signal r6_a11  : unsigned(10 downto 0) := (others => '0');
    signal r6_iv   : unsigned(6 downto 0) := (others => '0');
    signal r6_wy   : unsigned(3 downto 0) := (others => '0');
    signal r6_ivc  : unsigned(4 downto 0) := (others => '0');
    signal r6_wyc  : unsigned(3 downto 0) := (others => '0');
    signal r6_hist : std_logic := '0';

    signal ya00, ya10, ya01, ya11 : unsigned(11 downto 0) := (others => '0');
    signal ca00, ca10, ca01, ca11 : unsigned(9 downto 0) := (others => '0');
    -- side band: 14 iu0 13 iv0 12..9 wx 8..5 wy 4 ivc0 3..0 wyc
    signal sb7, sb8 : std_logic_vector(14 downto 0) := (others => '0');
    signal r7_pf  : unsigned(16 downto 0) := (others => '0');
    signal r7_hist, r8_hist : std_logic := '0';

    type t_yram is array (0 to 4095) of std_logic_vector(3 downto 0);
    type t_cram is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal yr00, yr10, yr01, yr11 : t_yram := (others => (others => '0'));
    signal cr00, cr10, cr01, cr11 : t_cram := (others => (others => '0'));
    signal q00, q10, q01, q11 : std_logic_vector(3 downto 0) := (others => '0');
    signal e00, e10, e01, e11 : std_logic_vector(7 downto 0) := (others => '0');
    attribute no_rw_check of yr00 : signal is true;
    attribute no_rw_check of yr10 : signal is true;
    attribute no_rw_check of yr01 : signal is true;
    attribute no_rw_check of yr11 : signal is true;
    attribute no_rw_check of cr00 : signal is true;
    attribute no_rw_check of cr10 : signal is true;
    attribute no_rw_check of cr01 : signal is true;
    attribute no_rw_check of cr11 : signal is true;

    signal r8_g   : unsigned(6 downto 0) := to_unsigned(64, 7);

    -- R9 reordered taps (Y unsigned 0..15 in 5-bit signed, chroma -8..7)
    signal r9_ytl, r9_ytr, r9_ybl, r9_ybr : signed(4 downto 0) := (others => '0');
    signal r9_utl, r9_utr, r9_ubl, r9_ubr : signed(4 downto 0) := (others => '0');
    signal r9_vtl, r9_vtr, r9_vbl, r9_vbr : signed(4 downto 0) := (others => '0');
    signal r9_wx, r9_wy, r9_wyc : unsigned(3 downto 0) := (others => '0');
    signal r9_g   : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal r9_hist : std_logic := '0';

    -- R10 differences (Blocks: nearest tap, zero differences)
    signal r10_ytl, r10_ybl, r10_dyt, r10_dyb : signed(4 downto 0) := (others => '0');
    signal r10_utl, r10_ubl, r10_dut, r10_dub : signed(4 downto 0) := (others => '0');
    signal r10_vtl, r10_vbl, r10_dvt, r10_dvb : signed(4 downto 0) := (others => '0');
    signal r10_wx, r10_wy, r10_wyc : unsigned(3 downto 0) := (others => '0');
    signal r10_g  : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal r10_hist : std_logic := '0';

    -- R11 time-weight products
    signal r11_ytl, r11_ybl, r11_utl, r11_ubl, r11_vtl, r11_vbl : signed(4 downto 0) := (others => '0');
    signal r11_pyt, r11_pyb, r11_put, r11_pub, r11_pvt, r11_pvb : signed(9 downto 0) := (others => '0');
    signal r11_wy, r11_wyc : unsigned(3 downto 0) := (others => '0');
    signal r11_g  : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal r11_hist : std_logic := '0';

    -- R12 upper cell (x16) and difference along the slit
    signal r12_yt, r12_ut, r12_vt : signed(9 downto 0) := (others => '0');
    signal r12_dy, r12_du, r12_dv : signed(10 downto 0) := (others => '0');
    signal r12_wy, r12_wyc : unsigned(3 downto 0) := (others => '0');
    signal r12_g  : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal r12_hist : std_logic := '0';

    -- R13 slit-weight products
    signal r13_yt, r13_ut, r13_vt : signed(9 downto 0) := (others => '0');
    signal r13_py, r13_pu, r13_pv : signed(14 downto 0) := (others => '0');
    signal r13_g  : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal r13_hist : std_logic := '0';

    -- R14 interpolated, Q4 levels
    signal r14_y  : unsigned(7 downto 0) := (others => '0');
    signal r14_u, r14_v : signed(8 downto 0) := (others => '0');
    signal r14_g  : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal r14_hist : std_logic := '0';

    -- R15..R17 fade / convert / compose
    signal r15_yg : unsigned(14 downto 0) := (others => '0');
    signal r15_ug, r15_vg : signed(16 downto 0) := (others => '0');
    signal r15_hist : std_logic := '0';
    signal r16_y, r16_u, r16_v : unsigned(9 downto 0) := C_BLKY;
    signal r16_hist : std_logic := '0';
    signal r17_y, r17_u, r17_v : unsigned(9 downto 0) := C_BLKY;
    signal s_io : t_video_stream_yuv444_30b;

    --------------------------------------------------------------------------
    -- Capture C0..C3 and the exposure read-modify-write W0..W7.
    --------------------------------------------------------------------------
    signal c0_y, c0_u, c0_v : unsigned(9 downto 0) := (others => '0');
    signal c0_av, c0_ls : std_logic := '0';
    signal c0_xs  : unsigned(15 downto 0) := (others => '0');
    signal c0_cell : unsigned(6 downto 0) := (others => '0');
    signal cr_row : unsigned(6 downto 0) := (others => '0');
    signal cr_lr  : unsigned(3 downto 0) := (others => '0');
    signal cap_armed, cap_line : std_logic := '0';
    signal cw_done : std_logic := '0';
    signal cw_cnt : unsigned(2 downto 0) := (others => '0');
    signal ch_pc  : unsigned(3 downto 0) := (others => '0');
    signal ch_cell : unsigned(6 downto 0) := (others => '0');

    signal c1_ys  : unsigned(15 downto 0) := (others => '0');
    signal c1_us, c1_vs : signed(14 downto 0) := (others => '0');
    signal c1_inw, c1_fy, c1_fc, c1_ey, c1_ec : std_logic := '0';
    signal c1_cell : unsigned(6 downto 0) := (others => '0');
    signal ysum   : unsigned(16 downto 0) := (others => '0');
    signal usum, vsum : signed(18 downto 0) := (others => '0');
    signal c2_ey, c2_ec : std_logic := '0';
    signal c2_cell : unsigned(6 downto 0) := (others => '0');

    signal ev_v, ev_c : std_logic := '0';
    signal ev_cy  : unsigned(6 downto 0) := (others => '0');
    signal ev_cc  : unsigned(4 downto 0) := (others => '0');
    signal ev_sy  : unsigned(15 downto 0) := (others => '0');
    signal ev_su, ev_sv : signed(15 downto 0) := (others => '0');

    signal slot   : unsigned(1 downto 0) := (others => '0');
    signal pu, pv : signed(15 downto 0) := (others => '0');
    signal pcc    : unsigned(4 downto 0) := (others => '0');
    signal ra     : unsigned(7 downto 0) := (others => '0');

    type t_acc is array (0 to 255) of std_logic_vector(15 downto 0);
    signal accram : t_acc := (others => (others => '0'));
    signal aq     : std_logic_vector(15 downto 0) := (others => '0');
    attribute no_rw_check of accram : signal is true;

    -- RMW tags: valid, kind (00 Y, 01 U, 10 V), RAM address (cell = low bits)
    type t_w is record
        v : std_logic;
        k : unsigned(1 downto 0);
        a : unsigned(7 downto 0);
    end record;
    constant C_W0 : t_w := ('0', "00", (others => '0'));
    signal w0, w1, w2, w3, w4, w5, w6 : t_w := C_W0;
    signal w0_val, w1_val, w2_val : signed(16 downto 0) := (others => '0');
    signal w2_af, w3_af, w4_af, w5_af : signed(15 downto 0) := (others => '0');
    signal w3_d   : signed(17 downto 0) := (others => '0');
    signal w4_s01, w4_s23, w4_s45, w4_s6 : signed(24 downto 0) := (others => '0');
    signal w5_p   : signed(24 downto 0) := (others => '0');
    signal w6_new : signed(15 downto 0) := (others => '0');

    signal wy_v, wy_px, wy_py, rep_y : std_logic := '0';
    signal wy_a   : unsigned(11 downto 0) := (others => '0');
    signal wy_d   : std_logic_vector(3 downto 0) := (others => '0');
    signal wc_v, wc_px, wc_py, rep_c : std_logic := '0';
    signal wc_a   : unsigned(9 downto 0) := (others => '0');
    signal wc_d   : std_logic_vector(7 downto 0) := (others => '0');

    -- chroma hue turn: one multiplier over phases
    signal hold_y, hold_u : signed(15 downto 0) := (others => '0');
    signal rt_ph  : unsigned(2 downto 0) := (others => '0');
    signal rt_u, rt_v : signed(8 downto 0) := (others => '0');
    signal rt_c   : unsigned(4 downto 0) := (others => '0');
    signal rm_a   : signed(8 downto 0) := (others => '0');
    signal rm_b   : signed(6 downto 0) := (others => '0');
    signal rm_p   : signed(15 downto 0) := (others => '0');
    signal rt_au, rt_av : signed(16 downto 0) := (others => '0');

begin

    --------------------------------------------------------------------------
    -- Free-running control decode.
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable rk1, rk2, rk3, rk4, rk5, rk6, rp12 : unsigned(9 downto 0);
        variable rsw : std_logic_vector(4 downto 0);
    begin
        if rising_edge(clk) then
            rk1  := unsigned(registers_in(0)(9 downto 0));
            rk2  := unsigned(registers_in(1)(9 downto 0));
            rk3  := unsigned(registers_in(2)(9 downto 0));
            rk4  := unsigned(registers_in(3)(9 downto 0));
            rk5  := unsigned(registers_in(4)(9 downto 0));
            rk6  := unsigned(registers_in(5)(9 downto 0));
            rsw  := registers_in(6)(4 downto 0);
            rp12 := unsigned(registers_in(7)(9 downto 0));

            -- K1: slit 0 .. 125.9 columns (Q8.8) = 32 K1 - K1/2; the capture
            -- window then always ends inside the line
            k_slit <= shift_left(resize(rk1, 16), 5) - resize(rk1(9 downto 1), 16);
            k_key  <= bip6(rk2); k_kdark <= not rk2(9);
            k_erev <= to_unsigned(1024, 11) - resize(rk3, 11);
            k_fade <= rk4(9 downto 4);
            k_rain <= bip6(rk5); k_rneg <= not rk5(9);
            -- K6 Layout (labelled: firmware may load the label INDEX 0..3)
            if rk6 < 4 then
                k_lay <= rk6(1 downto 0);
            else
                k_lay <= rk6(9 downto 8);
            end if;
            k_spd <= rp12;

            sw_axis  <= rsw(0);   -- S7  Axis Vertical / Horizontal
            sw_flow  <= rsw(1);   -- S8  Flow Out / In
            sw_blk   <= rsw(2);   -- S9  Pixels Smooth / Blocks
            sw_ghost <= rsw(3);   -- S10 Ghost
            sw_wave  <= rsw(4);   -- S11 Wave
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Input raster: measurement, hsync->avid parity, frame anchor.
    --------------------------------------------------------------------------
    p_input : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

            -- encoder pairs Cb/Cr by counting from hsync: record each line's
            -- hsync->avid offset parity (drape pattern)
            hpar <= not hpar;
            if v_h_edge = '1' then hpar <= '0'; end if;
            if data_in.avid = '1' and prev_avid = '0' then line_par <= hpar; end if;

            if v_h_edge = '1' then
                if px > 32 then w_meas <= px; end if;
                px <= (others => '0');
                if line_act = '1' then
                    line_act <= '0';
                    py       <= py + 1;
                    h_meas   <= resize(py, 12) + 1;
                end if;
            elsif data_in.avid = '1' then
                frame_armed <= '1';
                line_act    <= '1';
                px <= px + 1;
                if frame_act = '0' then
                    frame_act <= '1';
                    py        <= (others => '0');
                end if;
            end if;

            anchor <= '0';
            if v_v_edge = '1' and frame_armed = '1' then
                -- field anchor (serration-guarded): latch geometry
                frame_armed <= '0';
                frame_act   <= '0';
                py          <= (others => '0');
                anchor      <= '1';
                g_w <= w_meas;
                g_h <= h_meas;
            end if;
        end if;
    end process p_input;

    --------------------------------------------------------------------------
    -- Per-frame sequencer.  At the anchor: commit last frame's column(s),
    -- advance the scroll accumulator and the rainbow angle, start the slit
    -- glide, latch the controls.  Then kx = 2^23/W, ky = 2^23/H (128 cells
    -- across), the next frame's speed, this frame's exposure, the DDA
    -- starts, the column hue (parabolic sine / cosine), commit.  ~170
    -- clocks, all in vblank.  One add or compare per step.
    --------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_sum : unsigned(17 downto 0);
        variable v_ph  : unsigned(6 downto 0);
        variable v_rem : unsigned(12 downto 0);
        variable v_nb  : std_logic;
        variable v_kl  : unsigned(1 downto 0);
        variable v_n   : unsigned(2 downto 0);
        variable v_r   : unsigned(9 downto 0);
        variable v_m   : unsigned(14 downto 0);
        variable v_gs  : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            commit <= '0';

            -- serial multiplier: macc = ma * mb (11 x 11), MSB-first
            if mcnt /= 0 then
                if mb(10) = '1' then
                    macc <= (macc(20 downto 0) & '0') + resize(ma, 22);
                else
                    macc <= macc(20 downto 0) & '0';
                end if;
                mb   <= mb(9 downto 0) & '0';
                mcnt <= mcnt - 1;
            end if;

            -- serial restoring divider: dq = 2^23 / dden, 24 steps; the
            -- numerator is a single 1 bit, generated from the count
            if dcnt /= 0 then
                if dcnt = 24 then v_nb := '1'; else v_nb := '0'; end if;
                v_rem := drem & v_nb;
                if v_rem >= resize(dden, 13) then
                    drem <= resize(v_rem - resize(dden, 13), 12);
                    dq   <= dq(15 downto 0) & '1';
                else
                    drem <= v_rem(11 downto 0);
                    dq   <= dq(15 downto 0) & '0';
                end if;
                dcnt <= dcnt - 1;
            end if;

            -- slit glide: 1/8 of the way per frame (difference taken at the anchor)
            if g_go = '1' then
                v_gs := signed('0' & c_S) + shift_right(g_d, 3);
                c_S  <= unsigned(v_gs(15 downto 0));
                g_go <= '0';
            end if;

            case to_integer(sq) is
                when 0 =>
                    if anchor = '1' then
                        v_ph  := p_h - resize(cpend, 7);
                        p_h   <= v_ph;
                        c_pw  <= v_ph - 1;
                        c_pw2 <= v_ph - 2;
                        c_F   <= F(15 downto 8);
                        v_sum := resize(F, 18) + delta;
                        F     <= v_sum(15 downto 0);
                        cpend <= v_sum(17 downto 16);
                        if v_sum(17 downto 16) /= "00" then c_wr <= '1'; else c_wr <= '0'; end if;
                        c_dbl <= v_sum(17);
                        c_phw <= c_phw + 2;
                        -- rainbow: the angle steps once per new column (0 = off)
                        if k_rain = 0 then
                            phi <= (others => '0');
                        elsif v_sum(17 downto 16) /= "00" then
                            if v_sum(17) = '1' then v_r := shift_left(resize(k_rain, 10), 1);
                            else                    v_r := resize(k_rain, 10); end if;
                            if k_rneg = '1' then phi <= phi - v_r; else phi <= phi + v_r; end if;
                        end if;
                        g_d  <= signed('0' & k_slit) - signed('0' & c_S);
                        g_go <= '1';
                        -- frame-latched controls
                        c_axis  <= sw_axis;  c_flow <= sw_flow; c_blk <= sw_blk;
                        c_ghost <= sw_ghost; c_wave <= sw_wave;
                        c_lay   <= k_lay;
                        c_key   <= k_key;    c_kdark <= k_kdark;
                        c_fade  <= k_fade;
                        if k_rain >= 4 then c_bloom <= '1'; else c_bloom <= '0'; end if;
                        -- kx = 2^23 / W
                        dden <= g_w; drem <= (others => '0'); dq <= (others => '0');
                        dcnt <= to_unsigned(24, 5);
                        sq <= to_unsigned(1, 4);
                    end if;
                when 1 =>
                    if dcnt = 0 then
                        kx <= dq;
                        dden <= g_h; drem <= (others => '0'); dq <= (others => '0');
                        dcnt <= to_unsigned(24, 5);
                        sq <= to_unsigned(2, 4);
                    end if;
                when 2 =>
                    if dcnt = 0 then
                        ky <= dq;
                        ma <= '0' & k_spd; mb <= '0' & k_spd;
                        macc <= (others => '0'); mcnt <= to_unsigned(11, 4);
                        sq <= to_unsigned(3, 4);
                    end if;
                when 3 =>
                    if mcnt = 0 then
                        -- speed: 1/32 + P12^2 / 2^19 columns per frame, max 2
                        v_sum := to_unsigned(2048, 18) + resize(macc(19 downto 3), 18);
                        if v_sum > 131072 then
                            delta <= to_unsigned(131072, 18);
                        else
                            delta <= v_sum;
                        end if;
                        ma <= k_erev; mb <= k_erev;
                        macc <= (others => '0'); mcnt <= to_unsigned(11, 4);
                        sq <= to_unsigned(4, 4);
                    end if;
                when 4 =>
                    if mcnt = 0 then
                        -- exposure: m = (1024 - K3)^2 / 2^14, 64 = instant, min 1
                        if macc(21 downto 14) >= 64 then
                            c_m <= to_unsigned(64, 7);
                        elsif macc(21 downto 14) = 0 then
                            c_m <= to_unsigned(1, 7);
                        else
                            c_m <= macc(20 downto 14);
                        end if;
                        -- (C_LOOK + 1/2) kx = 9.5 kx, part 1: 8 kx + kx
                        c_x0 <= shift_left(resize(kx, 24), 3) + resize(kx, 24);
                        sq <= to_unsigned(5, 4);
                    end if;
                when 5 =>
                    c_x0 <= c_x0 + resize(kx(16 downto 1), 24);
                    c_xh <= resize(kx(16 downto 1), 24);
                    c_x4 <= shift_left(resize(kx, 24), 2) + resize(kx(16 downto 1), 24);
                    c_y0 <= resize(ky(16 downto 1), 24);
                    -- capture geometry: window 2^kw px, 2^kl lines per V cell
                    if g_w >= 1024 then c_kw <= '1'; else c_kw <= '0'; end if;
                    if    g_h >= 1024 then v_kl := "11";
                    elsif g_h >= 512  then v_kl := "10";
                    elsif g_h >= 256  then v_kl := "01";
                    else                   v_kl := "00"; end if;
                    c_kl <= v_kl;
                    -- pre-shift so every sum is 64 x the sample mean
                    if g_w >= 1024 then v_n := "011"; else v_n := "010"; end if;
                    if c_axis = '0' then v_n := v_n + resize(v_kl, 3); end if;
                    c_nsh <= to_unsigned(6, 3) - v_n;
                    -- column hue: sin(phi) via x (127 - x) + x = x (128 - x)
                    sx <= phi(8 downto 2);
                    ma <= resize(phi(8 downto 2), 11); mb <= resize(not phi(8 downto 2), 11);
                    macc <= (others => '0'); mcnt <= to_unsigned(11, 4);
                    sq <= to_unsigned(6, 4);
                when 6 =>
                    if mcnt = 0 then
                        v_m := macc(14 downto 0) + resize(sx, 15);
                        if phi(9) = '1' then
                            c_rS <= not signed(resize(v_m(13 downto 7), 7));
                        else
                            c_rS <= signed(resize(v_m(13 downto 7), 7));
                        end if;
                        v_r := phi + 256;                    -- cos = sin(+90 deg)
                        sx <= v_r(8 downto 2);
                        ma <= resize(v_r(8 downto 2), 11); mb <= resize(not v_r(8 downto 2), 11);
                        macc <= (others => '0'); mcnt <= to_unsigned(11, 4);
                        sq <= to_unsigned(7, 4);
                    end if;
                when 7 =>
                    if mcnt = 0 then
                        v_m := macc(14 downto 0) + resize(sx, 15);
                        v_r := phi + 256;
                        if v_r(9) = '1' then
                            c_rC <= not signed(resize(v_m(13 downto 7), 7));
                        else
                            c_rC <= signed(resize(v_m(13 downto 7), 7));
                        end if;
                        sq <= to_unsigned(8, 4);
                    end if;
                when 8 =>
                    commit <= '1';
                    sq <= to_unsigned(0, 4);
                when others =>
                    sq <= to_unsigned(0, 4);
            end case;
        end if;
    end process p_seq;

    --------------------------------------------------------------------------
    -- DDAs in canvas units (Q8.16, 128 across).  x_acc holds pixel
    -- C_LOOK's centre while avid is low and steps per input pixel, so
    -- during input pixel p it holds pixel p + C_LOOK.  x_s holds p + 4 (the
    -- slit coordinate at R6, instead of piping it), x_in the input pixel's
    -- own centre.  y_line steps at each active-video fall and holds the
    -- current line's centre (R1..R6 are in blanking when it steps).
    --------------------------------------------------------------------------
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            if commit = '1' then
                y_line <= c_y0;
            elsif data_in.avid = '0' and prev_avid = '1' then
                y_line <= y_line + resize(ky, 24);
            end if;
            if data_in.avid = '1' then
                x_acc <= x_acc + resize(kx, 24);
                x_s   <= x_s + resize(kx, 24);
                x_in  <= x_in + resize(kx, 24);
            else
                x_acc <= c_x0;
                x_s   <= c_x4;
                x_in  <= c_xh;
            end if;
        end if;
    end process p_dda;

    --------------------------------------------------------------------------
    -- R1..R18: coordinates, age, addresses, EBR, bilinear, fade, compose.
    -- Coordinates and ages are Q8.8 canvas columns.
    --------------------------------------------------------------------------
    p_read : process(clk)
        variable v_t, v_s : unsigned(15 downto 0);
        variable v_yi  : unsigned(9 downto 0);
        variable v_kd  : unsigned(9 downto 0);
        variable v_wm  : unsigned(14 downto 0);
        variable v_w   : signed(16 downto 0);
        variable v_h   : signed(16 downto 0);
        variable v_base : signed(16 downto 0);
        variable v_v   : signed(16 downto 0);
        variable v_vc  : signed(14 downto 0);
        variable v_iu, v_iu1 : unsigned(6 downto 0);
        variable v_colE, v_colO, v_rowE, v_rowO : unsigned(5 downto 0);
        variable v_crE, v_crO : unsigned(3 downto 0);
        variable v_lx, v_ty : std_logic;
        variable v_ytl, v_ytr, v_ybl, v_ybr : std_logic_vector(3 downto 0);
        variable v_ctl, v_ctr, v_cbl, v_cbr : std_logic_vector(7 downto 0);
        variable v_n   : signed(4 downto 0);
        variable v_t10, v_b10 : signed(9 downto 0);
        variable v_y256, v_u256, v_v256 : signed(14 downto 0);
        variable v_yo  : unsigned(10 downto 0);
        variable v_uo, v_vo : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            ---------------------------------------------------------------
            -- R1: time / slit coordinates by axis, key luma, wave phase
            ---------------------------------------------------------------
            if c_axis = '0' then
                v_t := x_acc(23 downto 8); v_s := y_line(23 downto 8);
            else
                v_t := y_line(23 downto 8); v_s := x_acc(23 downto 8);
            end if;
            r1_t <= v_t;
            -- two wave cycles along the slit, drifting with the frame count
            r1_wph <= v_s(13 downto 6) + c_phw;
            -- Time Key luma (input pixel C_LOOK px behind: invisible offset)
            v_yi := unsigned(data_in.y);
            if c_kdark = '1' then
                if v_yi > 940 then v_kd := (others => '0'); else v_kd := to_unsigned(940, 10) - v_yi; end if;
            else
                if v_yi < 64 then v_kd := (others => '0'); else v_kd := v_yi - 64; end if;
            end if;
            r1_kl <= v_kd(9 downto 2);

            ---------------------------------------------------------------
            -- R2: key product; parabolic wave sine, product half
            ---------------------------------------------------------------
            r2_t  <= r1_t;
            r2_key <= r1_kl * c_key;                        -- <= 57 columns
            r2_wx <= r1_wph(6 downto 0);
            r2_wp <= r1_wph(6 downto 0) * (not r1_wph(6 downto 0));
            r2_wn <= r1_wph(7);

            ---------------------------------------------------------------
            -- R3: distances from the slit and the far edge; wave offset
            -- (+-8 columns; one's complement on the negative half); key
            -- minus scroll fraction
            ---------------------------------------------------------------
            v_wm := shift_right(resize(r2_wp, 15) + resize(r2_wx, 15), 7);   -- 0..32
            v_w  := signed(resize(shift_left(resize(v_wm(5 downto 0), 12), 6), 17));
            if c_wave = '0' then
                r3_w <= (others => '0');
            elsif r2_wn = '1' then
                r3_w <= not v_w;
            else
                r3_w <= v_w;
            end if;
            r3_kf <= signed(resize(r2_key, 17)) - signed(resize(c_F, 17));
            r3_d1 <= signed('0' & r2_t) - signed('0' & c_S);
            r3_d3 <= signed('0' & c_S) - signed('0' & r2_t);
            r3_d2 <= to_signed(32768, 17) - signed('0' & r2_t);
            r3_t  <= signed('0' & r2_t);

            ---------------------------------------------------------------
            -- R4: layout / flow -> base age; history-side tests (waved)
            --   Right  Out d1 = t - S       In d2 = 128 - t
            --   Left   Out d3 = S - t       In t
            --   Mirror Out |t - S|          In nearer edge
            --   Full   Out t                In d2
            ---------------------------------------------------------------
            v_h := r3_d1 + r3_w;
            r4_p1 <= not v_h(16);
            v_h := r3_d3 + r3_w;
            r4_p3 <= not v_h(16);
            case c_lay is
                when "00" =>
                    if c_flow = '0' then v_base := r3_d1; else v_base := r3_d2; end if;
                when "01" =>
                    if c_flow = '0' then v_base := r3_d3; else v_base := r3_t; end if;
                when "10" =>
                    if r3_d1(16) = '1' then
                        if c_flow = '0' then v_base := r3_d3; else v_base := r3_t; end if;
                    else
                        if c_flow = '0' then v_base := r3_d1; else v_base := r3_d2; end if;
                    end if;
                when others =>
                    if c_flow = '0' then v_base := r3_t; else v_base := r3_d2; end if;
            end case;
            r4_base <= v_base;
            r4_e    <= resize(r3_kf, 18) + resize(r3_w, 18);

            ---------------------------------------------------------------
            -- R5: age, history region
            ---------------------------------------------------------------
            r5_a <= resize(r4_base, 19) + resize(r4_e, 19);
            case c_lay is
                when "00"   => r5_hist <= r4_p1;
                when "01"   => r5_hist <= r4_p3;
                when others => r5_hist <= '1';
            end case;

            ---------------------------------------------------------------
            -- R6: clamp the age to 0 .. 124.99 columns; slit cells (centres
            -- at +0.5), edges hold the end cell
            ---------------------------------------------------------------
            if r5_a(18) = '1' then
                r6_ia <= (others => '0'); r6_fa <= (others => '0'); r6_a11 <= (others => '0');
            elsif r5_a >= 32000 then
                r6_ia <= to_unsigned(124, 7); r6_fa <= (others => '1');
                r6_a11 <= to_unsigned(1999, 11);
            else
                r6_ia <= unsigned(r5_a(14 downto 8));
                r6_fa <= unsigned(r5_a(7 downto 4));
                r6_a11 <= unsigned(r5_a(14 downto 4));
            end if;
            r6_hist <= r5_hist;

            if c_axis = '0' then v_s := y_line(23 downto 8); else v_s := x_s(23 downto 8); end if;
            v_v := signed('0' & v_s) - 128;
            if v_v(16) = '1' then
                r6_iv <= (others => '0'); r6_wy <= (others => '0');
            elsif v_v(16 downto 8) >= 127 then
                r6_iv <= to_unsigned(127, 7); r6_wy <= (others => '0');
            else
                r6_iv <= unsigned(v_v(14 downto 8)); r6_wy <= unsigned(v_v(7 downto 4));
            end if;
            v_vc := signed('0' & v_s(15 downto 2)) - 128;      -- chroma cells: s / 4
            if v_vc(14) = '1' then
                r6_ivc <= (others => '0'); r6_wyc <= (others => '0');
            elsif v_vc(14 downto 8) >= 31 then
                r6_ivc <= to_unsigned(31, 5); r6_wyc <= (others => '0');
            else
                r6_ivc <= unsigned(v_vc(12 downto 8)); r6_wyc <= unsigned(v_vc(7 downto 4));
            end if;

            ---------------------------------------------------------------
            -- R7: sub-bank addresses {col, row}; fade product
            ---------------------------------------------------------------
            v_iu  := p_h + r6_ia;                    -- newer tap
            v_iu1 := p_h + r6_ia + 1;                -- older tap
            v_colO := v_iu(6 downto 1);
            v_colE := v_iu1(6 downto 1);
            v_rowO := r6_iv(6 downto 1);
            v_rowE := r6_iv(6 downto 1) + resize(r6_iv(0 downto 0), 6);
            v_crO  := r6_ivc(4 downto 1);
            v_crE  := r6_ivc(4 downto 1) + resize(r6_ivc(0 downto 0), 4);
            ya00 <= v_colE & v_rowE;
            ya10 <= v_colO & v_rowE;
            ya01 <= v_colE & v_rowO;
            ya11 <= v_colO & v_rowO;
            ca00 <= v_colE & v_crE;
            ca10 <= v_colO & v_crE;
            ca01 <= v_colE & v_crO;
            ca11 <= v_colO & v_crO;
            sb7 <= v_iu(0) & r6_iv(0) & std_logic_vector(r6_fa) & std_logic_vector(r6_wy)
                 & r6_ivc(0) & std_logic_vector(r6_wyc);
            r7_pf <= r6_a11 * c_fade;
            r7_hist <= r6_hist;

            -- R8 = EBR read (p_ram); fade gain 64 .. 0 (gone by 16 columns
            -- at full Fade)
            sb8 <= sb7;
            r8_hist <= r7_hist;
            if r7_pf(16 downto 8) >= 64 then
                r8_g <= (others => '0');
            else
                r8_g <= to_unsigned(64, 7) - resize(r7_pf(13 downto 8), 7);
            end if;

            ---------------------------------------------------------------
            -- R9: reorder taps (left = newer column, top = upper cell)
            ---------------------------------------------------------------
            v_lx := sb8(14); v_ty := sb8(13);
            if v_lx = '0' and v_ty = '0' then
                v_ytl := q00; v_ytr := q10; v_ybl := q01; v_ybr := q11;
            elsif v_lx = '1' and v_ty = '0' then
                v_ytl := q10; v_ytr := q00; v_ybl := q11; v_ybr := q01;
            elsif v_lx = '0' and v_ty = '1' then
                v_ytl := q01; v_ytr := q11; v_ybl := q00; v_ybr := q10;
            else
                v_ytl := q11; v_ytr := q01; v_ybl := q10; v_ybr := q00;
            end if;
            r9_ytl <= signed('0' & v_ytl); r9_ytr <= signed('0' & v_ytr);
            r9_ybl <= signed('0' & v_ybl); r9_ybr <= signed('0' & v_ybr);

            v_ty := sb8(4);
            if v_lx = '0' and v_ty = '0' then
                v_ctl := e00; v_ctr := e10; v_cbl := e01; v_cbr := e11;
            elsif v_lx = '1' and v_ty = '0' then
                v_ctl := e10; v_ctr := e00; v_cbl := e11; v_cbr := e01;
            elsif v_lx = '0' and v_ty = '1' then
                v_ctl := e01; v_ctr := e11; v_cbl := e00; v_cbr := e10;
            else
                v_ctl := e11; v_ctr := e01; v_cbl := e10; v_cbr := e00;
            end if;
            r9_utl <= resize(signed(v_ctl(7 downto 4)), 5); r9_vtl <= resize(signed(v_ctl(3 downto 0)), 5);
            r9_utr <= resize(signed(v_ctr(7 downto 4)), 5); r9_vtr <= resize(signed(v_ctr(3 downto 0)), 5);
            r9_ubl <= resize(signed(v_cbl(7 downto 4)), 5); r9_vbl <= resize(signed(v_cbl(3 downto 0)), 5);
            r9_ubr <= resize(signed(v_cbr(7 downto 4)), 5); r9_vbr <= resize(signed(v_cbr(3 downto 0)), 5);
            r9_wx  <= unsigned(sb8(12 downto 9));
            r9_wy  <= unsigned(sb8(8 downto 5));
            r9_wyc <= unsigned(sb8(3 downto 0));
            r9_g   <= r8_g;
            r9_hist <= r8_hist;

            ---------------------------------------------------------------
            -- R10: differences along time; Blocks = nearest tap, flat
            ---------------------------------------------------------------
            if c_blk = '1' then
                if r9_wy(3) = '0' then
                    if r9_wx(3) = '0' then v_n := r9_ytl; else v_n := r9_ytr; end if;
                else
                    if r9_wx(3) = '0' then v_n := r9_ybl; else v_n := r9_ybr; end if;
                end if;
                r10_ytl <= v_n; r10_ybl <= v_n; r10_dyt <= (others => '0'); r10_dyb <= (others => '0');
                if r9_wyc(3) = '0' then
                    if r9_wx(3) = '0' then v_n := r9_utl; else v_n := r9_utr; end if;
                else
                    if r9_wx(3) = '0' then v_n := r9_ubl; else v_n := r9_ubr; end if;
                end if;
                r10_utl <= v_n; r10_ubl <= v_n; r10_dut <= (others => '0'); r10_dub <= (others => '0');
                if r9_wyc(3) = '0' then
                    if r9_wx(3) = '0' then v_n := r9_vtl; else v_n := r9_vtr; end if;
                else
                    if r9_wx(3) = '0' then v_n := r9_vbl; else v_n := r9_vbr; end if;
                end if;
                r10_vtl <= v_n; r10_vbl <= v_n; r10_dvt <= (others => '0'); r10_dvb <= (others => '0');
            else
                r10_ytl <= r9_ytl; r10_ybl <= r9_ybl;
                r10_dyt <= r9_ytr - r9_ytl; r10_dyb <= r9_ybr - r9_ybl;
                r10_utl <= r9_utl; r10_ubl <= r9_ubl;
                r10_dut <= r9_utr - r9_utl; r10_dub <= r9_ubr - r9_ubl;
                r10_vtl <= r9_vtl; r10_vbl <= r9_vbl;
                r10_dvt <= r9_vtr - r9_vtl; r10_dvb <= r9_vbr - r9_vbl;
            end if;
            r10_wx <= r9_wx; r10_wy <= r9_wy; r10_wyc <= r9_wyc;
            r10_g <= r9_g;
            r10_hist <= r9_hist;

            ---------------------------------------------------------------
            -- R11: time-weight products
            ---------------------------------------------------------------
            r11_ytl <= r10_ytl; r11_ybl <= r10_ybl;
            r11_utl <= r10_utl; r11_ubl <= r10_ubl;
            r11_vtl <= r10_vtl; r11_vbl <= r10_vbl;
            r11_pyt <= smul4(r10_dyt, r10_wx, 10); r11_pyb <= smul4(r10_dyb, r10_wx, 10);
            r11_put <= smul4(r10_dut, r10_wx, 10); r11_pub <= smul4(r10_dub, r10_wx, 10);
            r11_pvt <= smul4(r10_dvt, r10_wx, 10); r11_pvb <= smul4(r10_dvb, r10_wx, 10);
            r11_wy <= r10_wy; r11_wyc <= r10_wyc;
            r11_g  <= r10_g;
            r11_hist <= r10_hist;

            ---------------------------------------------------------------
            -- R12: upper / lower cell (x16) and their difference
            ---------------------------------------------------------------
            v_t10 := shift_left(resize(r11_ytl, 10), 4) + r11_pyt;
            v_b10 := shift_left(resize(r11_ybl, 10), 4) + r11_pyb;
            r12_yt <= v_t10; r12_dy <= resize(v_b10, 11) - resize(v_t10, 11);
            v_t10 := shift_left(resize(r11_utl, 10), 4) + r11_put;
            v_b10 := shift_left(resize(r11_ubl, 10), 4) + r11_pub;
            r12_ut <= v_t10; r12_du <= resize(v_b10, 11) - resize(v_t10, 11);
            v_t10 := shift_left(resize(r11_vtl, 10), 4) + r11_pvt;
            v_b10 := shift_left(resize(r11_vbl, 10), 4) + r11_pvb;
            r12_vt <= v_t10; r12_dv <= resize(v_b10, 11) - resize(v_t10, 11);
            r12_wy <= r11_wy; r12_wyc <= r11_wyc;
            r12_g  <= r11_g;
            r12_hist <= r11_hist;

            ---------------------------------------------------------------
            -- R13: slit-weight products
            ---------------------------------------------------------------
            r13_yt <= r12_yt; r13_ut <= r12_ut; r13_vt <= r12_vt;
            r13_py <= smul4(r12_dy, r12_wy, 15);
            r13_pu <= smul4(r12_du, r12_wyc, 15);
            r13_pv <= smul4(r12_dv, r12_wyc, 15);
            r13_g  <= r12_g;
            r13_hist <= r12_hist;

            ---------------------------------------------------------------
            -- R14: interpolated (x256) -> Q4 levels
            ---------------------------------------------------------------
            v_y256 := shift_left(resize(r13_yt, 15), 4) + r13_py;
            v_u256 := shift_left(resize(r13_ut, 15), 4) + r13_pu;
            v_v256 := shift_left(resize(r13_vt, 15), 4) + r13_pv;
            if v_y256(14) = '1' then
                r14_y <= (others => '0');
            else
                r14_y <= unsigned(v_y256(11 downto 4));
            end if;
            r14_u <= v_u256(12 downto 4);
            r14_v <= v_v256(12 downto 4);
            r14_g <= r13_g;
            r14_hist <= r13_hist;

            ---------------------------------------------------------------
            -- R15: fade products
            ---------------------------------------------------------------
            r15_yg <= r14_y * r14_g;
            r15_ug <= r14_u * signed('0' & r14_g);
            r15_vg <= r14_v * signed('0' & r14_g);
            r15_hist <= r14_hist;

            ---------------------------------------------------------------
            -- R16: to 10-bit video (Y = 64 + 64 L, C = 512 + 64 L); chroma
            -- is stored canonical, swap back on odd hsync->avid rows
            ---------------------------------------------------------------
            v_yo := shift_left(resize(r15_yg(13 downto 6), 11), 2) + 64;
            if v_yo > 1019 then v_yo := to_unsigned(1019, 11); end if;
            v_uo := shift_left(resize(r15_ug(16 downto 6), 13), 2) + 512;
            v_vo := shift_left(resize(r15_vg(16 downto 6), 13), 2) + 512;
            if v_uo < 64 then v_uo := to_signed(64, 13); elsif v_uo > 960 then v_uo := to_signed(960, 13); end if;
            if v_vo < 64 then v_vo := to_signed(64, 13); elsif v_vo > 960 then v_vo := to_signed(960, 13); end if;
            r16_y <= v_yo(9 downto 0);
            if line_par = '1' then
                r16_u <= unsigned(v_vo(9 downto 0)); r16_v <= unsigned(v_uo(9 downto 0));
            else
                r16_u <= unsigned(v_uo(9 downto 0)); r16_v <= unsigned(v_vo(9 downto 0));
            end if;
            r16_hist <= r15_hist;

            ---------------------------------------------------------------
            -- R17: compose.  Live = vdly_q2 (this pixel, 7 clocks old).
            ---------------------------------------------------------------
            if r16_hist = '1' and not (c_ghost = '1' and unsigned(vdly_q2(29 downto 20)) > r16_y) then
                r17_y <= r16_y; r17_u <= r16_u; r17_v <= r16_v;
            else
                r17_y <= unsigned(vdly_q2(29 downto 20));
                r17_u <= unsigned(vdly_q2(19 downto 10));
                r17_v <= unsigned(vdly_q2(9 downto 0));
            end if;

            ---------------------------------------------------------------
            -- R18: blanking gate (avid of pipe(7) is this pixel), output
            ---------------------------------------------------------------
            if pipe(7).avid = '0' then
                s_io.y <= std_logic_vector(C_BLKY);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                s_io.y <= std_logic_vector(r17_y);
                s_io.u <= std_logic_vector(r17_u);
                s_io.v <= std_logic_vector(r17_v);
            end if;
            s_io.hsync_n <= pipe(C_PIPE - 1).hsync_n;
            s_io.vsync_n <= pipe(C_PIPE - 1).vsync_n;
            s_io.avid    <= pipe(C_PIPE - 1).avid;
            s_io.field_n <= pipe(C_PIPE - 1).field_n;
        end if;
    end process p_read;

    p_pipe : process(clk)
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to C_PIPE - 1 loop pipe(i) <= pipe(i - 1); end loop;
            -- live video delay line (one write, one registered read -> EBR)
            vdly_ram(to_integer(vdly_wa)) <= "00" & data_in.y & data_in.u & data_in.v;
            vdly_q  <= vdly_ram(to_integer(vdly_ra));
            vdly_q2 <= vdly_q;
            vdly_wa <= vdly_wa + 1;
            vdly_ra <= vdly_ra + 1;
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

    --------------------------------------------------------------------------
    -- Canvas: 8 arrays, each one registered read + one write statement.
    --------------------------------------------------------------------------
    p_ram : process(clk)
    begin
        if rising_edge(clk) then
            q00 <= yr00(to_integer(ya00));
            q10 <= yr10(to_integer(ya10));
            q01 <= yr01(to_integer(ya01));
            q11 <= yr11(to_integer(ya11));
            e00 <= cr00(to_integer(ca00));
            e10 <= cr10(to_integer(ca10));
            e01 <= cr01(to_integer(ca01));
            e11 <= cr11(to_integer(ca11));

            if wy_v = '1' and wy_px = '0' and wy_py = '0' then yr00(to_integer(wy_a)) <= wy_d; end if;
            if wy_v = '1' and wy_px = '1' and wy_py = '0' then yr10(to_integer(wy_a)) <= wy_d; end if;
            if wy_v = '1' and wy_px = '0' and wy_py = '1' then yr01(to_integer(wy_a)) <= wy_d; end if;
            if wy_v = '1' and wy_px = '1' and wy_py = '1' then yr11(to_integer(wy_a)) <= wy_d; end if;
            if wc_v = '1' and wc_px = '0' and wc_py = '0' then cr00(to_integer(wc_a)) <= wc_d; end if;
            if wc_v = '1' and wc_px = '1' and wc_py = '0' then cr10(to_integer(wc_a)) <= wc_d; end if;
            if wc_v = '1' and wc_px = '0' and wc_py = '1' then cr01(to_integer(wc_a)) <= wc_d; end if;
            if wc_v = '1' and wc_px = '1' and wc_py = '1' then cr11(to_integer(wc_a)) <= wc_d; end if;
        end if;
    end process p_ram;

    --------------------------------------------------------------------------
    -- Capture.  C0 registers the input pixel with its own raster coordinate
    -- and makes the per-line decisions (V: cell row and line-in-row; H: is
    -- this the slit line).  C1 runs the window / cell counters and
    -- pre-shifts the sample by nsh, so a box of 2^n samples always sums to
    -- 64 x its mean.  C2 accumulates, C3 hands finished cells to the RMW.
    --------------------------------------------------------------------------
    p_cap : process(clk)
        variable v_row  : unsigned(6 downto 0);
        variable v_done : std_logic;
        variable v_cnt  : unsigned(2 downto 0);
        variable v_pc   : unsigned(3 downto 0);
        variable v_kmax : unsigned(2 downto 0);
        variable v_lmax : unsigned(3 downto 0);
        variable v_inw, v_first, v_last, v_fy, v_fc, v_ey, v_ec : std_logic;
        variable v_uu, v_vv : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- C0
            c0_y  <= unsigned(data_in.y);
            c0_u  <= unsigned(data_in.u);
            c0_v  <= unsigned(data_in.v);
            c0_av <= data_in.avid;
            c0_xs <= x_in(23 downto 8);
            c0_cell <= x_in(22 downto 16);
            c0_ls <= '0';
            if data_in.avid = '1' and prev_avid = '0' then
                c0_ls <= '1';
                v_row := y_line(22 downto 16);
                if v_row /= cr_row then
                    cr_lr <= (others => '0');
                elsif cr_lr /= 15 then
                    cr_lr <= cr_lr + 1;
                end if;
                cr_row <= v_row;
                if cap_armed = '1' and y_line(23 downto 8) >= c_S then
                    cap_line  <= '1';
                    cap_armed <= '0';
                else
                    cap_line  <= '0';
                end if;
            end if;
            if commit = '1' then cap_armed <= '1'; end if;

            -- C1
            if c_kw = '1' then v_kmax := "111"; else v_kmax := "011"; end if;
            case c_kl is
                when "00"   => v_lmax := "0000";
                when "01"   => v_lmax := "0001";
                when "10"   => v_lmax := "0011";
                when others => v_lmax := "0111";
            end case;
            v_inw := '0'; v_fy := '0'; v_fc := '0'; v_ey := '0'; v_ec := '0';
            if c_axis = '0' then
                -- Vertical: a 2^kw px window from the slit, first 2^kl lines
                -- of each cell row
                v_done := cw_done; v_cnt := cw_cnt;
                if c0_ls = '1' then v_done := '0'; v_cnt := (others => '0'); end if;
                v_first := '0'; v_last := '0';
                if c0_av = '1' then
                    if v_done = '0' and c0_xs >= c_S then
                        v_done := '1'; v_cnt := v_kmax; v_first := '1'; v_inw := '1';
                    elsif v_cnt /= 0 then
                        v_inw := '1';
                        if v_cnt = 1 then v_last := '1'; end if;
                        v_cnt := v_cnt - 1;
                    end if;
                end if;
                cw_done <= v_done; cw_cnt <= v_cnt;
                if cr_lr > v_lmax then v_inw := '0'; end if;
                if cr_lr = 0 then
                    v_fy := v_first;
                    if cr_row(1 downto 0) = "00" then v_fc := v_first; end if;
                end if;
                if cr_lr = v_lmax then
                    v_ey := v_last;
                    if cr_row(1 downto 0) = "11" then v_ec := v_last; end if;
                end if;
                c1_cell <= cr_row;
            else
                -- Horizontal: on the slit line, the first 2^kw px of each cell
                v_pc := ch_pc;
                if c0_ls = '1' or c0_cell /= ch_cell then
                    v_pc := (others => '0');
                elsif v_pc /= 15 then
                    v_pc := v_pc + 1;
                end if;
                ch_pc <= v_pc; ch_cell <= c0_cell;
                if cap_line = '1' and c0_av = '1' and v_pc <= v_kmax then
                    v_inw := '1';
                    if v_pc = 0 then
                        v_fy := '1';
                        if c0_cell(1 downto 0) = "00" then v_fc := '1'; end if;
                    end if;
                    if v_pc = v_kmax then
                        v_ey := '1';
                        if c0_cell(1 downto 0) = "11" then v_ec := '1'; end if;
                    end if;
                end if;
                c1_cell <= c0_cell;
            end if;
            c1_inw <= v_inw; c1_fy <= v_fy; c1_fc <= v_fc;
            c1_ey  <= v_ey and v_inw; c1_ec <= v_ec and v_inw;
            -- canonical chroma (swap on odd hsync->avid lines), pre-shift
            if line_par = '1' then
                v_uu := signed(resize(c0_v, 11)) - 512;
                v_vv := signed(resize(c0_u, 11)) - 512;
            else
                v_uu := signed(resize(c0_u, 11)) - 512;
                v_vv := signed(resize(c0_v, 11)) - 512;
            end if;
            c1_ys <= shift_left(resize(c0_y, 16), to_integer(c_nsh));
            c1_us <= shift_left(resize(v_uu, 15), to_integer(c_nsh));
            c1_vs <= shift_left(resize(v_vv, 15), to_integer(c_nsh));

            -- C2
            if c1_inw = '1' then
                if c1_fy = '1' then
                    ysum <= resize(c1_ys, 17);
                else
                    ysum <= ysum + resize(c1_ys, 17);
                end if;
                if c1_fc = '1' then
                    usum <= resize(c1_us, 19);
                    vsum <= resize(c1_vs, 19);
                else
                    usum <= usum + resize(c1_us, 19);
                    vsum <= vsum + resize(c1_vs, 19);
                end if;
            end if;
            c2_ey <= c1_ey; c2_ec <= c1_ec; c2_cell <= c1_cell;

            -- C3: finished cell, Q10.5 (32 x mean)
            ev_v  <= c2_ey;
            ev_c  <= c2_ec;
            ev_cy <= c2_cell;
            ev_cc <= c2_cell(6 downto 2);
            ev_sy <= ysum(16 downto 1);
            ev_su <= usum(18 downto 3);
            ev_sv <= vsum(18 downto 3);
        end if;
    end process p_cap;

    --------------------------------------------------------------------------
    -- Exposure: acc += (sample - acc) * m / 64 per cell per frame, in a
    -- 256x16 RAM (Y at 0..127, U at 128..159, V at 160..191).  Pipelined
    -- read-modify-write: W0 issue, W1 EBR, W2 fabric, W3 difference,
    -- W4/W5 shift-add product (+1/2 rounding), W6 sum (a convex blend of
    -- acc and sample, so it never leaves the range) + write-back.  A
    -- cell's next update is a frame away, so there is no hazard.  W7
    -- writes luma into the canvas on commit frames (4 bit, stochastic
    -- rounding); chroma goes through the hue turn first.  Above 1
    -- column/frame each write repeats into a second column.
    --------------------------------------------------------------------------
    p_accram : process(clk)
    begin
        if rising_edge(clk) then
            aq <= accram(to_integer(ra));
            if w6.v = '1' then
                accram(to_integer(w6.a)) <= std_logic_vector(w6_new);
            end if;
        end if;
    end process p_accram;

    p_rmw : process(clk)
        variable v_p0, v_p1, v_p2, v_p3, v_p4, v_p5, v_p6, v_dd : signed(24 downto 0);
        variable v_l   : signed(11 downto 0);
        variable v_c   : signed(11 downto 0);
        variable v_q, v_qu : std_logic_vector(3 downto 0);
        variable v_b   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- W0: Y now, then U and V on the next two clocks
            w0.v <= '0';
            if ev_v = '1' then
                w0.v <= '1'; w0.k <= "00"; w0.a <= '0' & ev_cy;
                w0_val <= signed(resize(ev_sy, 17));
                ra <= '0' & ev_cy;
                if ev_c = '1' then slot <= "01"; end if;
                pu <= ev_su; pv <= ev_sv; pcc <= ev_cc;
            elsif slot = "01" then
                w0.v <= '1'; w0.k <= "01"; w0.a <= "100" & pcc;
                w0_val <= resize(pu, 17);
                ra <= "100" & pcc;
                slot <= "10";
            elsif slot = "10" then
                w0.v <= '1'; w0.k <= "10"; w0.a <= "101" & pcc;
                w0_val <= resize(pv, 17);
                ra <= "101" & pcc;
                slot <= "00";
            end if;

            -- W1 (EBR read in p_accram), W2 fabric copy
            w1 <= w0; w1_val <= w0_val;
            w2 <= w1; w2_val <= w1_val;
            w2_af <= signed(aq);

            -- W3: difference
            w3 <= w2;
            w3_af <= w2_af;
            w3_d <= resize(w2_val, 18) - resize(w2_af, 18);

            -- W4: partial products of d * m (m = 1..64), +32 for rounding
            w4 <= w3;
            w4_af <= w3_af;
            v_dd := resize(w3_d, 25);
            v_p0 := (others => '0'); v_p1 := (others => '0'); v_p2 := (others => '0');
            v_p3 := (others => '0'); v_p4 := (others => '0'); v_p5 := (others => '0');
            v_p6 := (others => '0');
            if c_m(0) = '1' then v_p0 := v_dd; end if;
            if c_m(1) = '1' then v_p1 := shift_left(v_dd, 1); end if;
            if c_m(2) = '1' then v_p2 := shift_left(v_dd, 2); end if;
            if c_m(3) = '1' then v_p3 := shift_left(v_dd, 3); end if;
            if c_m(4) = '1' then v_p4 := shift_left(v_dd, 4); end if;
            if c_m(5) = '1' then v_p5 := shift_left(v_dd, 5); end if;
            if c_m(6) = '1' then v_p6 := shift_left(v_dd, 6); end if;
            w4_s01 <= v_p0 + v_p1; w4_s23 <= v_p2 + v_p3;
            w4_s45 <= v_p4 + v_p5; w4_s6  <= v_p6 + 32;

            -- W5: product
            w5 <= w4;
            w5_af <= w4_af;
            w5_p <= (w4_s01 + w4_s23) + (w4_s45 + w4_s6);

            -- W6: acc + product / 64
            w6 <= w5;
            w6_new <= w5_af + resize(shift_right(w5_p, 6), 16);

            -- W7: canvas writes on commit frames
            wy_v <= '0';
            wc_v <= '0';
            if rep_y = '1' then
                wy_v <= '1'; wy_px <= c_pw2(0);
                wy_a(11 downto 6) <= c_pw2(6 downto 1);
                rep_y <= '0';
            end if;
            if w6.v = '1' then
                if w6.k = "00" then
                    hold_y <= w6_new;
                    v_l := signed(resize(unsigned(w6_new(14 downto 5)), 12)) - 64
                         + signed(resize(lfsr(5 downto 0), 12));
                    if v_l < 0 then
                        v_q := "0000";
                    elsif v_l >= 1024 then
                        v_q := "1111";
                    else
                        v_q := std_logic_vector(v_l(9 downto 6));
                    end if;
                    if c_wr = '1' then
                        wy_v <= '1'; wy_px <= c_pw(0); wy_py <= w6.a(0);
                        wy_a <= c_pw(6 downto 1) & w6.a(6 downto 1);
                        wy_d <= v_q;
                        rep_y <= c_dbl;
                    end if;
                elsif w6.k = "01" then
                    hold_u <= w6_new;
                else
                    -- start the hue turn: U, V in 4-code units (+ bloom from
                    -- this group's luma, (Y - 64) / 16)
                    v_b := resize(hold_y(15 downto 5), 12) - 64;
                    if c_bloom = '1' and v_b(11) = '0' then
                        rt_u <= hold_u(15 downto 7) + signed(resize(unsigned(v_b(9 downto 4)), 9));
                    else
                        rt_u <= hold_u(15 downto 7);
                    end if;
                    rt_v <= w6_new(15 downto 7);
                    rt_c <= w6.a(4 downto 0);
                    rt_ph <= "001";
                end if;
            end if;

            -- Hue turn U' = U C - V S, V' = U S + V C on one multiplier
            -- (C, S = +-32 per frame); then quantise to U4 V4 and write
            rm_p <= rm_a * rm_b;
            if rep_c = '1' then
                wc_v <= '1'; wc_px <= c_pw2(0);
                wc_a(9 downto 4) <= c_pw2(6 downto 1);
                rep_c <= '0';
            end if;
            case rt_ph is
                when "001" =>
                    rm_a <= rt_u; rm_b <= c_rC; rt_ph <= "010";
                when "010" =>
                    rm_a <= rt_v; rm_b <= c_rS; rt_ph <= "011";
                when "011" =>
                    rm_a <= rt_u; rm_b <= c_rS; rt_au <= resize(rm_p, 17); rt_ph <= "100";
                when "100" =>
                    rm_a <= rt_v; rm_b <= c_rC; rt_au <= rt_au - resize(rm_p, 17); rt_ph <= "101";
                when "101" =>
                    rt_av <= resize(rm_p, 17); rt_ph <= "110";
                when "110" =>
                    rt_av <= rt_av + resize(rm_p, 17); rt_ph <= "111";
                when "111" =>
                    -- level = (U' + dither) / 16 in 4-code units, -8..7
                    v_c := resize(rt_au(16 downto 5), 12) + signed(resize(lfsr(9 downto 6), 12));
                    if v_c(11 downto 4) < -8 then v_qu := "1000";
                    elsif v_c(11 downto 4) > 7 then v_qu := "0111";
                    else v_qu := std_logic_vector(v_c(7 downto 4)); end if;
                    v_c := resize(rt_av(16 downto 5), 12) + signed(resize(lfsr(13 downto 10), 12));
                    if v_c(11 downto 4) < -8 then v_q := "1000";
                    elsif v_c(11 downto 4) > 7 then v_q := "0111";
                    else v_q := std_logic_vector(v_c(7 downto 4)); end if;
                    if c_wr = '1' then
                        wc_v <= '1'; wc_px <= c_pw(0); wc_py <= rt_c(0);
                        wc_a <= c_pw(6 downto 1) & rt_c(4 downto 1);
                        wc_d <= v_qu & v_q;
                        rep_c <= c_dbl;
                    end if;
                    rt_ph <= "000";
                when others =>
                    null;
            end case;
        end if;
    end process p_rmw;

end architecture slitscan;
