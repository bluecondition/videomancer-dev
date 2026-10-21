-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Streak v0.5.0 -- Heirloom #7, rebuilt from the photo WIN_20230129_00_40_34.
--
-- The photo: the upper two thirds are horizontally smeared noise through a
-- three-channel colourizer (green / blue folds of the source, red set per
-- band), cut by hard horizontal seams; the bands get thinner down the frame
-- (78 lines at the top, 32 at the bottom: a band clock swept by the vertical
-- ramp); every eighth band is a clean blue/green strip.  Below a diagonal
-- (x = 1.74 y - 90) the picture is frozen into a mosaic: each band holds one
-- line, sampled-and-held into 41-px blocks with slewed edges, on full blue
-- (magenta / cyan / white / blue blocks).  The same diagonal ramp lights a
-- green ridge along the edge, fringes the edge with fold stripes, and darkens
-- green and blue toward the top-right corner, which stays red.
--
-- The patch (see SUMMARY.md):
--   source: smeared noise (xorshift16 white noise seeded per line by a hash ->
--           2-pole x IIR -> 1-pole y IIR through a line RAM, restarted at every
--           band seam) + Osc blobs (a slower 2-pole/1-pole octave) or camera luma
--   band clock: phase(line) by a 2nd-order DDA (phi += F; F += K), rolling
--   8-step sequencer per band: red level; step 0 = a clean held strip
--   diagonal ramp d = x - edge(y): wipe key (mosaic), ridge, edge stripes,
--           blue bias, green VCA
--   mosaic: S&H at the block clock (phase per band) + slew, line hold per band
--           with slow drift (hold RAM)
--   colourizer: G = fold(theta), B = 1 - fold + bias, R = the red sequence read
--           at (band + x slant, +-1.5 steps across the frame) + blob noise / 2
--
--   K1  Bands    band density, 4 .. 64 per frame (23.9 = the photo)
--   K2  Taper    how much thinner the bands get down the frame (2.4x = photo)
--   K3  Blocks   mosaic block width 8 .. 240 px (41 = photo)
--   K4  Speed    bipolar: the bands roll and the noise rides with them; still at the centre
--   K5  Edge     where the diagonal sits: a corner of mosaic .. all mosaic
--   K6  Hue      colourizer phase (bipolar, photo at the centre)
--   S7  Mosaic   Blocks / Lines (no S&H: the held lines show as a barcode)
--   S8  Strips   Sparse (every 8th band) / Dense (every other band)
--   S9  Flip     mosaic on the right, the diagonal mirrored
--   S10 Smear    Short / Long streaks
--   S11 Source   Video / Osc
--   P12 STREAK   noise depth: 0 = clean colour bands .. storm
--
-- Generated avid; output conditioning (luma [1 2 1], chroma 9-tap, luma 13/16)
-- before the blanking gate.  Latency 44 clocks in every mode.  20 EBR: noise and
-- hold line RAMs (1024 x 16, half resolution), a 256 x 32 delay ring, three RGB->YUV
-- contribution ROMs, the red-sequence ROM, the sync delay ring and the chroma
-- 8-delay ring.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture streak of program_top is
    constant C_LAT : integer := 44;   -- matches processing_delay_clks

    constant C_SALT1 : unsigned(15 downto 0) := x"01DE";   -- 0x1D3 + 11
    constant C_SALT2 : unsigned(15 downto 0) := x"7A9C";   -- 0x7A91 + 11
    constant C_PHI0  : unsigned(31 downto 0) := x"FF9A1CF3";     -- -0.398 band, Q24

    ---------------------------------------------------------------------------
    -- per-timing-ID constants
    ---------------------------------------------------------------------------
    function f_ys(t : std_logic_vector(3 downto 0)) return unsigned is   -- virtual lines per line, Q8
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 11);
            when 4          => return to_unsigned(576, 11);
            when 8          => return to_unsigned(960, 11);
            when 12         => return to_unsigned(480, 11);
            when 5 | 6 | 14 => return to_unsigned(384, 11);
            when 1 | 2 | 10 => return to_unsigned(512, 11);
            when others     => return to_unsigned(256, 11);
        end case;
    end function;
    function f_s2(t : std_logic_vector(3 downto 0)) return unsigned is   -- ys^2, Q16
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1295044, 21);
            when 4          => return to_unsigned(331776, 21);
            when 8          => return to_unsigned(921600, 21);
            when 12         => return to_unsigned(230400, 21);
            when 5 | 6 | 14 => return to_unsigned(147456, 21);
            when 1 | 2 | 10 => return to_unsigned(262144, 21);
            when others     => return to_unsigned(65536, 21);
        end case;
    end function;
    function f_kx(t : std_logic_vector(3 downto 0)) return unsigned is   -- virtual px per px, Q8
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(683, 10);
            when 5 | 6 | 14     => return to_unsigned(384, 10);
            when others         => return to_unsigned(256, 10);
        end case;
    end function;
    function f_xc(t : std_logic_vector(3 downto 0)) return unsigned is   -- x pole class
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(2, 2);
            when 5 | 6 | 14     => return to_unsigned(1, 2);
            when others         => return to_unsigned(0, 2);
        end case;
    end function;
    function f_vc(t : std_logic_vector(3 downto 0)) return unsigned is   -- y pole class
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8                        => return to_unsigned(2, 2);
            when 1 | 2 | 4 | 5 | 6 | 10 | 12 | 14 => return to_unsigned(1, 2);
            when others                       => return to_unsigned(0, 2);
        end case;
    end function;

    -- the pixel / line hash, one round per function (adds give the carries)
    function rotl(a : unsigned(15 downto 0); k : integer) return unsigned is
    begin
        return a(15 - k downto 0) & a(15 downto 16 - k);
    end function;
    function hr2(a : unsigned(15 downto 0)) return unsigned is
    begin
        return a xor rotl(a, 5) xor rotl(a, 11);
    end function;
    function hr3(a : unsigned(15 downto 0)) return unsigned is
    begin
        return a + (a(12 downto 0) & "000") + x"6A09";
    end function;
    function hr4(a : unsigned(15 downto 0)) return unsigned is
    begin
        return a xor rotl(a, 7);
    end function;
    function hr5(a : unsigned(15 downto 0)) return unsigned is
    begin
        return a + (a(10 downto 0) & "00000") + (a(14 downto 0) & '0');
    end function;
    function hr6(a : unsigned(15 downto 0)) return unsigned is
    begin
        return a xor (x"00" & a(15 downto 8));
    end function;


    -- saturate a signed value to a signed width
    function sats(v : signed; w : integer) return signed is
        variable r : signed(w - 1 downto 0);
        variable ok : std_logic;
    begin
        ok := '1';
        for i in w - 1 to v'length - 2 loop
            if v(i) /= v(v'length - 1) then ok := '0'; end if;
        end loop;
        if ok = '1' then
            r := v(w - 1 downto 0);
        else
            r := (others => not v(v'length - 1));
            r(w - 1) := v(v'length - 1);
        end if;
        return r;
    end function;

    -- RGB -> YUV contribution ROMs: {Y 11, U 10, V 10} per 8-bit channel value (each term / 32)
    type t_rom is array (0 to 255) of std_logic_vector(31 downto 0);
    function f_rom(ky, ku, kv : integer) return t_rom is
        variable t : t_rom;
    begin
        for i in 0 to 255 loop
            t(i) := '0' & std_logic_vector(to_unsigned((i * ky) / 32, 11)) & std_logic_vector(to_unsigned((i * ku) / 32, 10))
                    & std_logic_vector(to_unsigned((i * kv) / 32, 10));
        end loop;
        return t;
    end function;
    constant C_ROM_R : t_rom := f_rom(81, 112, 38);
    constant C_ROM_G : t_rom := f_rom(159, 94, 74);
    constant C_ROM_B : t_rom := f_rom(31, 18, 112);

    -- the 8-step red sequence with 32 interpolated sub-steps per step (read at band + x slant)
    type t_rom8 is array (0 to 255) of std_logic_vector(7 downto 0);
    function f_rseq return t_rom8 is
        type t_seq is array (0 to 8) of integer;
        constant sq : t_seq := (30, 93, 166, 221, 213, 171, 117, 92, 30);
        variable t : t_rom8;
    begin
        for i in 0 to 255 loop
            t(i) := std_logic_vector(to_unsigned((32 * sq(i / 32) + (sq(i / 32 + 1) - sq(i / 32)) * (i mod 32)) / 32, 8));
        end loop;
        return t;
    end function;
    constant C_RSEQ : t_rom8 := f_rseq;

    function f_km(t : std_logic_vector(3 downto 0)) return unsigned is   -- noise gain per mode, Q6
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8          => return to_unsigned(44, 7);
            when 4 | 12         => return to_unsigned(64, 7);
            when 5 | 6 | 14     => return to_unsigned(94, 7);
            when 1 | 2 | 10     => return to_unsigned(92, 7);
            when others         => return to_unsigned(125, 7);
        end case;
    end function;
    function f_gb(t : std_logic_vector(3 downto 0)) return unsigned is   -- blob noise gain: x4 x3 x2 x1.5 x1
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8          => return to_unsigned(4, 3);
            when 4 | 12         => return to_unsigned(3, 3);
            when 5 | 6 | 14     => return to_unsigned(2, 3);
            when 1 | 2 | 10     => return to_unsigned(1, 3);
            when others         => return to_unsigned(0, 3);
        end case;
    end function;

    ---------------------------------------------------------------------------
    -- RAMs: noise line {vF 8, vB 8} and hold line {dev 10, r 5, keyed 1}
    ---------------------------------------------------------------------------
    type t_ram is array (0 to 1023) of std_logic_vector(15 downto 0);
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal nram : t_ram := (others => (others => '0'));
    signal hram : t_ram := (others => (others => '0'));
    signal rram : t_ring := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of nram : signal is true;
    attribute no_rw_check of hram : signal is true;
    attribute no_rw_check of rram : signal is true;
    signal n_ra, n_wa, h_ra, h_wa, c21 : unsigned(10 downto 0) := (others => '0');
    signal r_wd, r_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_we : std_logic := '0';
    signal rom_r, rom_g, rom_b : std_logic_vector(31 downto 0) := (others => '0');
    signal rseq : std_logic_vector(7 downto 0) := (others => '0');
    -- sync delay ring (44 clocks) and the chroma 8-delay ring of the output conditioning
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sram : t_sring := (others => (others => '1'));
    type t_cring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal cram : t_cring := (others => "10000000001000000000");
    attribute no_rw_check of sram : signal is true;
    attribute no_rw_check of cram : signal is true;
    signal s_wc : unsigned(7 downto 0) := (others => '0');
    signal s_in, s_q, s_out : std_logic_vector(3 downto 0) := (others => '1');
    signal c_q, c_old : std_logic_vector(19 downto 0) := "10000000001000000000";
    signal n_wd, h_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal n_we, h_we : std_logic := '0';
    signal n_q, h_q : std_logic_vector(15 downto 0) := (others => '0');

    -- timing
    signal t_ys : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_s2 : unsigned(20 downto 0) := to_unsigned(65536, 21);
    signal t_kx : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal t_xc, t_vc : unsigned(1 downto 0) := (others => '0');
    signal t_ty : unsigned(11 downto 0) := to_unsigned(448, 12);
    signal t_km : unsigned(6 downto 0) := to_unsigned(125, 7);
    signal t_gb : unsigned(2 downto 0) := (others => '0');

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K1, K2, K3, K5, P12 (Q.3)
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(340 * 8, 13), to_unsigned(526 * 8, 13), to_unsigned(605 * 8, 13),
                         to_unsigned(293 * 8, 13), to_unsigned(445 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- serial multiplier (26 x 22), 15 slots of 32 clocks
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 12) := (others => '0');
    signal m_a  : unsigned(25 downto 0) := (others => '0');
    signal m_b  : unsigned(21 downto 0) := (others => '0');
    signal m_h  : unsigned(25 downto 0) := (others => '0');
    signal m_l  : unsigned(21 downto 0) := (others => '0');

    -- per-field values
    signal f_bq   : unsigned(14 downto 0) := to_unsigned(6125, 15);
    signal f_t10  : unsigned(9 downto 0) := to_unsigned(427, 10);
    signal f_b1   : unsigned(21 downto 0) := to_unsigned(216583, 22);
    signal f_b2   : unsigned(17 downto 0) := to_unsigned(36742, 18);
    signal f_f0   : unsigned(23 downto 0) := to_unsigned(216726, 24);
    signal f_kk   : unsigned(14 downto 0) := to_unsigned(287, 15);
    signal f_inc  : unsigned(13 downto 0) := to_unsigned(1598, 14);
    signal f_incp : unsigned(14 downto 0) := to_unsigned(1592, 15);
    signal f_tq   : unsigned(11 downto 0) := to_unsigned(600, 12);
    signal f_tc   : unsigned(9 downto 0) := (others => '0');
    signal f_tneg : std_logic := '0';
    signal f_t    : signed(12 downto 0) := (others => '0');
    signal f_cx   : signed(12 downto 0) := to_signed(863, 13);
    signal f_cy   : signed(11 downto 0) := to_signed(548, 12);
    signal f_ncy  : signed(11 downto 0) := to_signed(-548, 12);
    signal f_sl   : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal f_phi0 : unsigned(31 downto 0) := C_PHI0;
    signal f_oy8 : unsigned(19 downto 0) := (others => '0');
    signal f_sp   : signed(10 downto 0) := (others => '0');
    signal f_base : signed(11 downto 0) := to_signed(220, 12);
    signal f_gp   : unsigned(10 downto 0) := to_unsigned(778, 11);
    signal f_gm   : unsigned(4 downto 0) := to_unsigned(24, 5);
    signal f_ge   : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal f_lines, f_dense, f_flip, f_long, f_osc : std_logic := '0';
    signal f_fx  : unsigned(2 downto 0) := to_unsigned(4, 3);   -- fine x pole shift 3..6
    signal f_fxr : unsigned(6 downto 0) := to_unsigned(8, 7);   -- its rounding constant
    signal f_bx  : unsigned(2 downto 0) := to_unsigned(7, 3);   -- big x pole shift 5..7
    signal f_bxr : unsigned(6 downto 0) := to_unsigned(64, 7);  -- its rounding constant
    signal f_fy0 : unsigned(2 downto 0) := to_unsigned(3, 3);   -- fine y pole shift 1..4
    signal f_fy0r : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal f_by  : unsigned(2 downto 0) := to_unsigned(5, 3);   -- big y pole shift 3..5
    signal f_byr : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal f_cs  : std_logic := '1';                            -- camera smear 1/8 (else 1/4)

    -- per-line engine (next line) and the committed line values
    signal le_st : unsigned(5 downto 0) := (others => '0');
    signal le_first : std_logic := '1';
    signal q_tgo : std_logic := '0';
    signal q_tsel : std_logic_vector(1 downto 0) := "00";
    signal ln_ph  : unsigned(31 downto 0) := C_PHI0;
    signal ln_f   : unsigned(23 downto 0) := (others => '0');
    signal ln_thr : signed(23 downto 0) := (others => '0');
    signal ln_yv  : unsigned(19 downto 0) := (others => '0');
    signal ln_line : unsigned(11 downto 0) := (others => '0');
    signal ln_np  : unsigned(7 downto 0) := (others => '0');
    signal hu     : unsigned(15 downto 0) := (others => '0');
    signal l_thr  : signed(13 downto 0) := (others => '0');
    signal l_kline, l_clean, l_bst : std_logic := '0';
    signal l_jit  : signed(7 downto 0) := (others => '0');
    signal l_off  : signed(10 downto 0) := (others => '0');
    signal l_step : unsigned(2 downto 0) := (others => '0');
    signal l_seed : unsigned(15 downto 0) := (others => '0');
    signal l_shp  : unsigned(7 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- camera (source time): smear, then the colourizer and red terms
    signal c_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_av, c_avq : std_logic := '0';
    signal c_sm : unsigned(13 downto 0) := to_unsigned(502 * 16, 14);
    signal c_dv : signed(10 downto 0) := (others => '0');
    signal c_sv : signed(10 downto 0) := (others => '0');
    signal c_rv : signed(9 downto 0) := (others => '0');

    -- beam (stage 0)
    signal b_xv   : unsigned(18 downto 0) := (others => '0');
    signal b_first : std_logic := '0';

    -- pixel pipeline
    type t_sl is array (natural range <>) of std_logic;
    signal p_first : t_sl(1 to 26) := (others => '0');
    signal p_key   : t_sl(2 to 32) := (others => '0');
    signal x9, x27 : unsigned(18 downto 0) := (others => '0');
    signal a22     : unsigned(15 downto 0) := (others => '0');
    signal w22     : std_logic := '0';
    signal p1_d    : signed(13 downto 0) := (others => '0');
    signal xs      : unsigned(15 downto 0) := x"ACE1";
    signal p6_w9   : signed(8 downto 0) := (others => '0');
    signal p7_f1   : signed(15 downto 0) := (others => '0');
    signal p7_c1   : signed(16 downto 0) := (others => '0');
    signal p8_f2   : signed(15 downto 0) := (others => '0');
    signal p8_c2   : signed(16 downto 0) := (others => '0');
    signal p9_inf  : signed(15 downto 0) := (others => '0');
    signal p9_inb  : signed(18 downto 0) := (others => '0');
    signal p9_nq   : std_logic_vector(15 downto 0) := (others => '0');
    signal p10_dl  : signed(16 downto 0) := (others => '0');
    signal p10_dlb : signed(19 downto 0) := (others => '0');
    signal p10_v, p10_vb : signed(11 downto 0) := (others => '0');
    signal p10_inf : signed(15 downto 0) := (others => '0');
    signal p10_d   : signed(13 downto 0) := (others => '0');
    signal p11_a : signed(15 downto 0) := (others => '0');
    signal p11_nb  : signed(17 downto 0) := (others => '0');
    signal p11_v   : signed(11 downto 0) := (others => '0');
    signal p11_ih  : signed(14 downto 0) := (others => '0');
    signal p11_dm  : signed(13 downto 0) := (others => '0');
    signal p12_nv  : signed(16 downto 0) := (others => '0');
    signal p12_nb  : signed(11 downto 0) := (others => '0');
    signal p12_ad  : unsigned(13 downto 0) := (others => '0');
    signal p13_nf, p13_nb : signed(11 downto 0) := (others => '0');
    signal p13_rg  : unsigned(8 downto 0) := (others => '0');
    signal p14_wf, p14_wb : signed(7 downto 0) := (others => '0');
    signal p14_pm  : signed(17 downto 0) := (others => '0');
    signal p14_so, p14_ro, p14_rv : signed(11 downto 0) := (others => '0');
    signal p14_rg  : unsigned(9 downto 0) := (others => '0');
    signal p15_sn  : signed(13 downto 0) := (others => '0');
    signal p15_src : signed(11 downto 0) := (others => '0');
    signal p15_rg  : unsigned(9 downto 0) := (others => '0');
    signal p16_sq  : signed(13 downto 0) := (others => '0');
    signal p16_a1  : signed(11 downto 0) := (others => '0');
    signal p16_src : signed(11 downto 0) := (others => '0');
    signal p17_a1, p17_a2 : signed(13 downto 0) := (others => '0');
    signal p18_ds  : signed(14 downto 0) := (others => '0');
    signal p19_dev : signed(11 downto 0) := (others => '0');
    signal p19_e17 : signed(12 downto 0) := (others => '0');
    signal p20_th  : unsigned(9 downto 0) := (others => '0');
    signal p20_s1, p20_s2 : signed(13 downto 0) := (others => '0');
    signal p20_d8  : signed(9 downto 0) := (others => '0');
    signal p21_dk, p21_rk : signed(13 downto 0) := (others => '0');
    signal p22_dk8 : signed(9 downto 0) := (others => '0');
    signal p22_rk5 : signed(4 downto 0) := (others => '0');
    signal p23_sht : signed(9 downto 0) := (others => '0');
    signal p23_shr : signed(4 downto 0) := (others => '0');
    signal p24_st  : signed(13 downto 0) := (others => '0');
    signal p24_sr  : signed(8 downto 0) := (others => '0');
    signal p24_hq  : std_logic_vector(15 downto 0) := (others => '0');
    signal p25_lt, p25_ht : signed(9 downto 0) := (others => '0');
    signal p25_lr, p25_hr : signed(4 downto 0) := (others => '0');
    signal p25_dt  : signed(11 downto 0) := (others => '0');
    signal p25_cap : std_logic := '0';
    signal p26_nt  : signed(9 downto 0) := (others => '0');
    signal p24_rq  : std_logic_vector(31 downto 0) := (others => '0');
    signal p25_th, p26_th : unsigned(9 downto 0) := (others => '0');
    signal p25_rn, p26_rn : signed(9 downto 0) := (others => '0');
    signal p26_nr  : signed(4 downto 0) := (others => '0');
    signal p27_th  : unsigned(9 downto 0) := (others => '0');
    signal p27_rs  : signed(9 downto 0) := (others => '0');
    signal p28_tg  : unsigned(8 downto 0) := (others => '0');
    signal p28_rh, p29_rh, p30_rh, p31_rh : signed(9 downto 0) := (others => '0');
    signal p28_sa : signed(7 downto 0) := (others => '0');
    signal p29_ra : unsigned(7 downto 0) := (others => '0');
    signal p31_rl : unsigned(7 downto 0) := (others => '0');
    signal p28_d   : signed(13 downto 0) := (others => '0');
    signal p29_g8  : unsigned(7 downto 0) := (others => '0');
    signal p29_b9a : signed(10 downto 0) := (others => '0');
    signal p29_vg  : unsigned(4 downto 0) := (others => '0');
    signal p29_bb  : signed(11 downto 0) := (others => '0');
    signal p29_e   : unsigned(7 downto 0) := (others => '0');
    signal p30_gp  : unsigned(12 downto 0) := (others => '0');
    signal p30_b9b : signed(11 downto 0) := (others => '0');
    signal p30_bbe : signed(12 downto 0) := (others => '0');
    signal p31_g8  : unsigned(7 downto 0) := (others => '0');
    signal p31_b9  : signed(13 downto 0) := (others => '0');
    signal p32_r, p32_g, p32_b : unsigned(7 downto 0) := (others => '0');
    signal p34_ys : unsigned(11 downto 0) := (others => '0');
    signal p34_us, p34_vs : signed(11 downto 0) := (others => '0');
    signal p35_y, p35_u, p35_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p36_y, p36_u, p36_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- short delays
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    type t_s10a is array (natural range <>) of signed(9 downto 0);
    signal p_e   : t_u8a(12 to 18) := (others => (others => '0'));
    signal p_rn  : t_s10a(16 to 20) := (others => (others => '0'));
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y36, s_y37, s_y38, s_y39, s_y40, s_y41x, s_yf41 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u41, s_v41 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y42, s_y43, s_y44 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u42, s_v42, s_u43, s_v43, s_u44, s_v44 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line RAMs (one write statement each, registered read; collision-free:
    -- the noise RAM reads 8 pixels ahead of its write, the hold RAM 5)
    ---------------------------------------------------------------------------
    p_nram : process(clk)
    begin
        if rising_edge(clk) then
            if n_we = '1' then nram(to_integer(n_wa(10 downto 1))) <= n_wd; end if;
            n_q <= nram(to_integer(n_ra(10 downto 1)));
        end if;
    end process p_nram;

    p_hram : process(clk)
    begin
        if rising_edge(clk) then
            if h_we = '1' then hram(to_integer(h_wa(10 downto 1))) <= h_wd; end if;
            h_q <= hram(to_integer(h_ra(10 downto 1)));
        end if;
    end process p_hram;

    -- a 256-deep ring that carries the live theta / deviation / red noise from
    -- stage 21 to stage 24 (written one pixel ahead of the read)
    p_rram : process(clk)
    begin
        if rising_edge(clk) then
            if r_we = '1' then rram(to_integer(c21(7 downto 0))) <= r_wd; end if;
            r_q <= rram(to_integer(h_ra(7 downto 0)));
        end if;
    end process p_rram;

    -- RGB -> YUV ROMs (registered address = stage 32, output = stage 33)
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            rom_r <= C_ROM_R(to_integer(p32_r));
            rom_g <= C_ROM_G(to_integer(p32_g));
            rom_b <= C_ROM_B(to_integer(p32_b));
            rseq  <= C_RSEQ(to_integer(p29_ra));
        end if;
    end process p_rom;

    -- syncs: input register, ring (write at s_wc, read 41 behind), RAM register, fabric copy = 44 clocks
    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            s_in <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & '1';
            s_wc <= s_wc + 1;
            sram(to_integer(s_wc)) <= s_in;
            s_q <= sram(to_integer(s_wc - 41));
            s_out <= s_q;
        end if;
    end process p_sring;

    -- chroma ring: u/v of stage 36 written at s_wc, read 6 behind -> c_old = u/v of 8 clocks earlier
    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            cram(to_integer(s_wc)) <= std_logic_vector(p36_u) & std_logic_vector(p36_v);
            c_q <= cram(to_integer(s_wc - 6));
            c_old <= c_q;
        end if;
    end process p_cring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(26 downto 0);
        variable v_pr : unsigned(47 downto 0);
        variable v_sl : integer range 0 to 7;
        variable v_sp : signed(31 downto 0);
    begin
        if rising_edge(clk) then
            t_ys <= f_ys(registers_in(8)(3 downto 0));
            t_s2 <= f_s2(registers_in(8)(3 downto 0));
            t_kx <= f_kx(registers_in(8)(3 downto 0));
            t_xc <= f_xc(registers_in(8)(3 downto 0));
            t_vc <= f_vc(registers_in(8)(3 downto 0));
            t_km <= f_km(registers_in(8)(3 downto 0));
            t_gb <= f_gb(registers_in(8)(3 downto 0));
            t_ty <= resize(t_ys, 12) + resize(t_ys(10 downto 1), 12) + resize(t_ys(10 downto 2), 12);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 1023 then
                s_zc <= s_zc + 1;
            end if;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            g_wm1 <= s_w - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K4 Speed: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 7 then f_sp <= v_x - 8;
                elsif v_x < -7 then f_sp <= v_x + 8;
                else f_sp <= (others => '0'); end if;
                -- K6 Hue: colourizer phase, photo at the centre
                f_base <= to_signed(220 - 512, 12) + signed(resize(unsigned(registers_in(5)), 12));
                f_lines <= registers_in(6)(0);
                f_dense <= registers_in(6)(1);
                f_flip  <= registers_in(6)(2);
                f_long  <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- motion: the band phase rolls, the noise rides with it and flows right
            end if;
            -- per-field motion, one add per zone clock: phase += sp * 13 / 256 band (Q24),
            -- noise scroll += sp * 2.28 lines (Q8)
            v_sp := resize(f_sp, 32);
            if s_zone = '1' and s_zc = 1 then
                f_phi0 <= f_phi0 + unsigned(v_sp sll 11); f_oy8 <= f_oy8 + unsigned(resize(f_sp, 20) sll 1);
            elsif s_zone = '1' and s_zc = 2 then
                f_phi0 <= f_phi0 + unsigned(v_sp sll 10); f_oy8 <= f_oy8 + unsigned(resize(shift_right(f_sp, 2), 20));
            elsif s_zone = '1' and s_zc = 3 then
                f_phi0 <= f_phi0 + unsigned(v_sp sll 8); f_oy8 <= f_oy8 + unsigned(resize(shift_right(f_sp, 5), 20));
            end if;

            -- glides: K1, K2, K3, K5, P12 ease 1/8 of the gap per field (deadband 4)
            if s_zone = '1' and s_zc >= 4 and s_zc < 24 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when others => s_g(4) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- serial products: slot p = zc/32 (1..15), phase 0 load, 1..22 step, 24 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 12 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 12 and v_ph >= 1 and v_ph <= 22 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 12 loop
                if s_zone = '1' and v_p = k and v_ph = 24 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            v_pr := m_h & m_l;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 1  => m_a <= resize(f_bq, 26);                m_b <= to_unsigned(1024, 22) - resize(f_t10, 22);
                    when 2  => m_a <= v_pr(25 downto 0);               m_b <= to_unsigned(3883, 22);
                    when 3  => m_a <= resize(f_bq, 26);                m_b <= resize(f_t10, 22);
                    when 4  => m_a <= resize(v_pr(24 downto 0), 26);   m_b <= to_unsigned(921, 22);
                    when 5  => m_a <= resize(f_b1, 26);                m_b <= resize(t_ys, 22);
                    when 6  => m_a <= resize(t_s2, 26);                m_b <= resize(f_b2, 22);
                    when 7  => m_a <= resize(1023 - s_g(2)(12 downto 3), 26); m_b <= resize(1023 - s_g(2)(12 downto 3), 22);
                    when 8  => m_a <= resize(v_pr(19 downto 0), 26);   m_b <= to_unsigned(7919, 22);
                    when 9  => m_a <= resize(f_inc, 26);               m_b <= resize(t_kx, 22);
                    when 10 => m_a <= resize(s_g(3)(12 downto 3), 26); m_b <= to_unsigned(2100, 22);
                    when 11 => m_a <= resize(unsigned(abs(f_t)), 26);  m_b <= to_unsigned(154, 22);
                    when others => m_a <= resize(s_g(4)(12 downto 3), 26); m_b <= resize(t_km, 22);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 27) + resize(m_a, 27); else v_s := resize(m_h, 27); end if;
                m_h <= v_s(26 downto 1);
                m_l <= v_s(0) & m_l(21 downto 1);
                m_b <= '0' & m_b(21 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(2)  = '1' then f_b1  <= v_pr(37 downto 16); end if;
            if q_wr(4)  = '1' then f_b2  <= v_pr(33 downto 16); end if;
            if q_wr(5)  = '1' then f_f0 <= v_pr(31 downto 8); end if;
            if q_wr(6)  = '1' then f_f0 <= f_f0 + resize(v_pr(37 downto 24), 24); f_kk <= v_pr(37 downto 23); end if;
            if q_wr(8)  = '1' then f_inc <= to_unsigned(273, 14) + resize(v_pr(32 downto 20), 14); end if;
            if q_wr(9)  = '1' then f_incp <= v_pr(22 downto 8); end if;
            if q_wr(10) = '1' then f_tq  <= v_pr(21 downto 10); end if;
            if q_wr(11) = '1' then f_tc  <= v_pr(17 downto 8); end if;
            if q_wr(12) = '1' then f_gp  <= v_pr(16 downto 6); end if;
            -- K1: 4 .. 64 bands per frame (linear, Q8 / 4); K2: taper 0 .. 0.81
            f_bq  <= to_unsigned(1024, 15) + resize(shift_left(resize(s_g(0)(12 downto 3), 15), 4) - resize(s_g(0)(12 downto 3), 15), 15);
            f_t10 <= resize(shift_right(shift_left(resize(s_g(1)(12 downto 3), 14), 3) + shift_left(resize(s_g(1)(12 downto 3), 14), 2)
                                        + resize(s_g(1)(12 downto 3), 14), 4), 10);

            -- derived per-field values (settle long before the line engine reads them)
            f_t <= signed(resize(f_tq, 13)) - 600;
            if f_t < 0 then f_tneg <= '1'; else f_tneg <= '0'; end if;
            f_cx <= to_signed(863, 13) + f_t;
            if f_tneg = '1' then f_cy <= to_signed(548, 12) + signed(resize(f_tc, 12));
            else f_cy <= to_signed(548, 12) - signed(resize(f_tc, 12)); end if;
            f_ncy <= -f_cy;
            -- P12 depth as a 5-bit mantissa and a shift
            if f_gp(10) = '1' then f_ge <= to_unsigned(6, 3); f_gm <= f_gp(10 downto 6);
            elsif f_gp(9) = '1' then f_ge <= to_unsigned(5, 3); f_gm <= f_gp(9 downto 5);
            elsif f_gp(8) = '1' then f_ge <= to_unsigned(4, 3); f_gm <= f_gp(8 downto 4);
            elsif f_gp(7) = '1' then f_ge <= to_unsigned(3, 3); f_gm <= f_gp(7 downto 3);
            elsif f_gp(6) = '1' then f_ge <= to_unsigned(2, 3); f_gm <= f_gp(6 downto 2);
            elsif f_gp(5) = '1' then f_ge <= to_unsigned(1, 3); f_gm <= f_gp(5 downto 1);
            else f_ge <= to_unsigned(0, 3); f_gm <= f_gp(4 downto 0); end if;
            if f_inc > 5461 then v_sl := 1;
            elsif f_inc > 2731 then v_sl := 2;
            elsif f_inc > 1365 then v_sl := 3;
            else v_sl := 4; end if;
            v_sl := v_sl - to_integer(t_xc);
            if v_sl < 1 then v_sl := 1; end if;
            f_sl <= to_unsigned(v_sl, 3);
            -- pole selects
            if t_xc = 2 then
                if f_long = '1' then f_fx <= to_unsigned(5, 3); f_fxr <= to_unsigned(16, 7);
                else f_fx <= to_unsigned(3, 3); f_fxr <= to_unsigned(4, 7); end if;
            else
                if f_long = '1' then f_fx <= to_unsigned(6, 3); f_fxr <= to_unsigned(32, 7);
                else f_fx <= to_unsigned(4, 3); f_fxr <= to_unsigned(8, 7); end if;
            end if;
            f_bx <= to_unsigned(7, 3) - resize(t_xc, 3);
            case to_integer(t_xc) is
                when 0 => f_bxr <= to_unsigned(64, 7);
                when 1 => f_bxr <= to_unsigned(32, 7);
                when others => f_bxr <= to_unsigned(16, 7);
            end case;
            case to_integer(t_vc) is
                when 0 => f_by <= to_unsigned(5, 3); f_byr <= to_unsigned(16, 5);
                when 1 => f_by <= to_unsigned(4, 3); f_byr <= to_unsigned(8, 5);
                when others => f_by <= to_unsigned(3, 3); f_byr <= to_unsigned(4, 5);
            end case;
            case to_integer(t_vc) is
                when 0 =>
                    if f_long = '1' then f_fy0 <= to_unsigned(4, 3); f_fy0r <= to_unsigned(8, 4);
                    else f_fy0 <= to_unsigned(3, 3); f_fy0r <= to_unsigned(4, 4); end if;
                when 1 =>
                    if f_long = '1' then f_fy0 <= to_unsigned(3, 3); f_fy0r <= to_unsigned(4, 4);
                    else f_fy0 <= to_unsigned(2, 3); f_fy0r <= to_unsigned(2, 4); end if;
                when others =>
                    if f_long = '1' then f_fy0 <= to_unsigned(2, 3); f_fy0r <= to_unsigned(2, 4);
                    else f_fy0 <= to_unsigned(1, 3); f_fy0r <= to_unsigned(1, 4); end if;
            end case;
            if t_xc = 0 then f_cs <= '1'; else f_cs <= '0'; end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    -- Generated avid (parity-locked start 2/3 clocks after the source's avid
    -- rise, field-latched width)
    ---------------------------------------------------------------------------
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
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;
            if g_start = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';


    ---------------------------------------------------------------------------
    -- Per-line engine: at the source avid fall (and once late in the vblank
    -- zone for line 0) it computes the next line: band index and seam, step,
    -- band hashes, line noise seed, the diagonal; committed at the avid rise
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_j : signed(11 downto 0);
        variable v_o : signed(10 downto 0);
        variable v_n : unsigned(7 downto 0);
        variable v_cl : std_logic;
        variable v_ad : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            -- line 0: phase, step, the diagonal (cx - 1.75 cy, one add per zone clock)
            -- (the add select is registered a clock ahead)
            q_tgo <= '0';
            if s_zone = '1' and s_zc = 686 then q_tgo <= '1'; q_tsel <= "01";
            elsif s_zone = '1' and s_zc = 687 then q_tgo <= '1'; q_tsel <= "10";
            elsif s_zone = '1' and s_zc = 688 then q_tgo <= '1'; q_tsel <= "11";
            elsif le_st = 59 then q_tgo <= '1'; q_tsel <= "00"; end if;
            case q_tsel is
                when "01" => v_ad := resize(f_ncy, 24) sll 8;
                when "10" => v_ad := resize(f_ncy, 24) sll 7;
                when "11" => v_ad := resize(f_ncy, 24) sll 6;
                when others => v_ad := signed(resize(t_ty, 24));
            end case;
            if s_zone = '1' and s_zc = 686 then
                ln_ph <= f_phi0; ln_f <= f_f0; ln_thr <= resize(f_cx, 24) sll 8;
                ln_yv <= (others => '0'); ln_line <= (others => '0');
                le_first <= '1';
            elsif q_tgo = '1' then
                ln_thr <= ln_thr + v_ad;
            end if;

            -- the engine runs 34..60 clocks after the source avid fall (the last pixels of the
            -- line have left every stage that reads l_*), writing the next line's l_* directly
            if s_zone = '1' and s_zc = 700 then
                le_st <= to_unsigned(1, 6);
            elsif data_in.avid = '0' and s_avq = '1' then
                le_st <= to_unsigned(1, 6);
            elsif le_st /= 0 then
                if le_st = 60 then le_st <= (others => '0'); else le_st <= le_st + 1; end if;
            end if;

            v_n := ln_ph(31 downto 24);
            if f_dense = '1' then v_cl := not v_n(0);
            elsif v_n(2 downto 0) = "000" then v_cl := '1'; else v_cl := '0'; end if;
            case to_integer(le_st) is
                when 35 =>
                    if le_first = '1' or v_n /= ln_np then l_bst <= '1'; else l_bst <= '0'; end if;
                    if signed('0' & ln_yv(19 downto 8)) > resize(f_cy, 13) then l_kline <= '1'; else l_kline <= '0'; end if;
                    l_thr <= resize(shift_right(ln_thr, 8), 14);
                    l_clean <= v_cl;
                    l_step <= v_n(2 downto 0);
                    hu <= (v_n & v_n) + C_SALT1;                                     -- band hash 1 (n * 257)
                when 36 | 42 | 48 | 54 => hu <= hr2(hu);
                when 37 | 43 | 49 | 55 => hu <= hr3(hu);
                when 38 | 44 | 50 | 56 => hu <= hr4(hu);
                when 39 | 45 | 51 | 57 => hu <= hr5(hu);
                when 40 | 46 | 52 | 58 => hu <= hr6(hu);
                when 41 =>
                    v_o := signed(resize(hu(7 downto 0), 11)) - 128;
                    v_o := v_o(9 downto 0) & '0';
                    if l_clean = '1' then v_o := v_o + 102; end if;
                    l_off <= v_o;
                    v_j := signed(resize(hu(15 downto 8), 12)) - 128;
                    v_j := shift_right((v_j sll 2) + v_j, 4);
                    l_jit <= v_j(7 downto 0);
                    hu <= hu + C_SALT2;                                              -- band hash 2
                when 47 =>
                    l_shp <= hu(7 downto 0);
                    hu <= resize((ln_line + f_oy8(19 downto 8)) and x"FFF", 16) + x"2B7E";   -- line seed
                when 53 =>
                    hu <= hu + x"51F1";
                when 59 =>
                    l_seed <= hu;
                when 60 =>
                    ln_np <= v_n; le_first <= '0';
                    ln_ph <= ln_ph + resize(ln_f, 32);
                    ln_f  <= ln_f + resize(f_kk, 24);
                    ln_yv <= ln_yv + resize(t_ys, 20);
                    ln_line <= ln_line + 1;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Camera (source time, ~12 px ahead of the beam: invisible in a smear):
    -- luma smeared along the line, held through the blanking
    ---------------------------------------------------------------------------
    p_cam : process(clk)
        variable v_d : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            c_y <= unsigned(data_in.y);
            c_av <= data_in.avid; c_avq <= c_av;
            if c_av = '1' and c_avq = '0' then
                c_sm <= c_y & "0000";
            elsif c_av = '1' then
                v_d := signed(resize(c_y & "0000", 15)) - signed(resize(c_sm, 15));
                if f_cs = '1' then v_d := shift_right(v_d + 4, 3); else v_d := shift_right(v_d + 2, 2); end if;
                v_d := signed(resize(c_sm, 15)) + v_d;
                c_sm <= unsigned(v_d(13 downto 0));
            end if;
            c_dv <= signed(resize(c_sm(13 downto 4), 11)) - 502;
            c_sv <= shift_right(c_dv, 1) + shift_right(c_dv, 3) + shift_right(c_dv, 4);
            c_rv <= resize(shift_right(resize(c_dv, 13) + (resize(c_dv, 13) sll 1), 3), 10);
        end if;
    end process p_cam;

    ---------------------------------------------------------------------------
    -- Beam: virtual x (mirrored by Flip), hash x
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                if f_flip = '1' then b_xv <= to_unsigned(1919 * 256, 19); else b_xv <= (others => '0'); end if;
                b_first <= '1';
            elsif g_avid = '1' then
                if f_flip = '1' then b_xv <= b_xv - t_kx; else b_xv <= b_xv + t_kx; end if;
                b_first <= '0';
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t   : signed(15 downto 0);
        variable v_s   : signed(15 downto 0);
        variable v_b   : signed(16 downto 0);
        variable v_c   : signed(19 downto 0);
        variable v_cx  : signed(17 downto 0);
        variable v_w16 : std_logic_vector(15 downto 0);
        variable v_n   : signed(16 downto 0);
        variable v_n2  : signed(16 downto 0);
        variable v_10  : signed(9 downto 0);
        variable v_11  : signed(11 downto 0);
        variable v_12  : signed(12 downto 0);
        variable v_13  : signed(13 downto 0);
        variable v_9   : signed(8 downto 0);
        variable v_e   : unsigned(7 downto 0);
        variable v_h16 : unsigned(15 downto 0);
        variable v_wu  : unsigned(8 downto 0);
        variable v_th  : unsigned(10 downto 0);
        variable v_a   : unsigned(16 downto 0);
        variable v_u   : signed(21 downto 0);
        variable v_d, v_d2 : signed(14 downto 0);
        variable v_g   : unsigned(12 downto 0);
        variable v_y   : unsigned(11 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dly : unsigned(9 downto 0);
        variable v_ix  : unsigned(10 downto 0);
        variable v_rd, v_rn : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: d = x - edge(y) (for the key); hash round 1
            p1_d <= signed(resize(b_xv(18 downto 8), 14)) - l_thr;
            -- 2: key; hash round 2
            if l_kline = '1' and p1_d < resize(l_jit, 14) then p_key(2) <= '1'; else p_key(2) <= '0'; end if;
            -- 3..5: hash rounds
            -- 6: white noise: xorshift16 (7, 9, 8), seeded per line by the line hash; odd-symmetric 9 bits
            if p_first(5) = '1' then v_h16 := l_seed(15 downto 1) & '1'; else v_h16 := xs; end if;
            v_h16 := v_h16 xor (v_h16(8 downto 0) & "0000000");
            v_h16 := v_h16 xor ("000000000" & v_h16(15 downto 9));
            v_h16 := v_h16 xor (v_h16(7 downto 0) & "00000000");
            xs <= v_h16;
            v_e := v_h16(15 downto 8);
            v_wu := (not v_e(7)) & v_e(6 downto 0) & '1';
            p6_w9 <= signed(v_wu);

            -- 7: x IIRs, first poles (fine: input rounded; big: floor); noise RAM read address
            if p_first(6) = '1' then v_t := (others => '0'); else v_t := p7_f1; end if;
            v_s := shift_left(resize(p6_w9, 16), 5) + signed(resize(f_fxr, 16)) - v_t;
            case to_integer(f_fx) is
                when 3 => v_s := shift_right(v_s, 3);
                when 4 => v_s := shift_right(v_s, 4);
                when 5 => v_s := shift_right(v_s, 5);
                when others => v_s := shift_right(v_s, 6);
            end case;
            p7_f1 <= v_t + v_s;
            if p_first(6) = '1' then v_b := (others => '0'); else v_b := p7_c1; end if;
            v_w16 := std_logic_vector(p6_w9) & std_logic_vector(f_bxr);
            v_cx := resize(signed(v_w16), 18) - resize(v_b, 18);   -- (w << 7) + round
            case to_integer(f_bx) is
                when 5 => v_cx := shift_right(v_cx, 5);
                when 6 => v_cx := shift_right(v_cx, 6);
                when others => v_cx := shift_right(v_cx, 7);
            end case;
            p7_c1 <= v_b + resize(v_cx, 17);
            if p_first(6) = '1' then n_ra <= (others => '0'); else n_ra <= n_ra + 1; end if;

            -- 8: x IIRs, second poles (floor)
            if p_first(7) = '1' then v_t := (others => '0'); else v_t := p8_f2; end if;
            v_s := p7_f1 - v_t;
            case to_integer(f_fx) is
                when 3 => v_s := shift_right(v_s, 3);
                when 4 => v_s := shift_right(v_s, 4);
                when 5 => v_s := shift_right(v_s, 5);
                when others => v_s := shift_right(v_s, 6);
            end case;
            p8_f2 <= v_t + v_s;
            if p_first(7) = '1' then v_b := (others => '0'); else v_b := p8_c2; end if;
            v_cx := resize(p7_c1, 18) - resize(v_b, 18) + signed(resize(f_bxr, 18));
            case to_integer(f_bx) is
                when 5 => v_cx := shift_right(v_cx, 5);
                when 6 => v_cx := shift_right(v_cx, 6);
                when others => v_cx := shift_right(v_cx, 7);
            end case;
            p8_c2 <= v_b + resize(v_cx, 17);

            -- 9: inputs to the vertical IIRs; RAM data (fabric copy); x of the stage-9 pixel
            p9_inf <= p8_f2;
            case to_integer(t_gb) is
                when 0 => p9_inb <= shift_left(resize(p8_c2, 19), 2);
                when 1 => p9_inb <= shift_left(resize(p8_c2, 19), 1) + resize(p8_c2, 19);
                when 2 => p9_inb <= shift_left(resize(p8_c2, 19), 1);
                when 3 => p9_inb <= resize(p8_c2, 19) + shift_right(resize(p8_c2, 19), 1);
                when others => p9_inb <= resize(p8_c2, 19);
            end case;
            p9_nq <= n_q;
            if p_first(8) = '1' then
                if f_flip = '1' then x9 <= to_unsigned(1919 * 256, 19); else x9 <= (others => '0'); end if;
            elsif f_flip = '1' then x9 <= x9 - t_kx; else x9 <= x9 + t_kx; end if;

            -- 10: differences against the stored line (states x16); d for the ridge / edge
            p10_v  <= shift_left(resize(signed(p9_nq(15 downto 8)), 12), 4);
            p10_vb <= shift_left(resize(signed(p9_nq(7 downto 0)), 12), 4);
            p10_dl  <= resize(p9_inf, 17) - shift_left(resize(signed(p9_nq(15 downto 8)), 17), 4);
            p10_dlb <= resize(p9_inb, 20) - shift_left(resize(signed(p9_nq(7 downto 0)), 20), 4);
            p10_inf <= p9_inf;
            p10_d <= signed(resize(x9(18 downto 8), 14)) - l_thr;

            -- 11: vertical IIR terms (rounded); ridge distance; edge
            v_n := p10_dl + signed(resize(f_fy0r, 17));
            case to_integer(f_fy0) is
                when 1 => v_n := shift_right(v_n, 1);
                when 2 => v_n := shift_right(v_n, 2);
                when 3 => v_n := shift_right(v_n, 3);
                when others => v_n := shift_right(v_n, 4);
            end case;
            p11_a <= resize(v_n, 16);
            v_c := p10_dlb + signed(resize(f_byr, 20));
            case to_integer(f_by) is
                when 3 => v_c := shift_right(v_c, 3);
                when 4 => v_c := shift_right(v_c, 4);
                when others => v_c := shift_right(v_c, 5);
            end case;
            p11_nb <= resize(p10_vb, 18) + resize(v_c, 18);
            p11_v <= p10_v;
            p11_ih <= resize(shift_right(p10_inf, 1), 15);
            p11_dm <= p10_d - 90;
            if l_kline = '0' then p_e(12) <= (others => '0');
            elsif p10_d < 0 then p_e(12) <= to_unsigned(128, 8);
            elsif p10_d < 128 then p_e(12) <= to_unsigned(128, 8) - unsigned(p10_d(7 downto 0));
            else p_e(12) <= (others => '0'); end if;

            -- 12: the new fine state (restarted at every band seam); big state saturated; |d - 90|
            if l_bst = '1' then
                p12_nv <= resize(p11_ih, 17);
            else
                p12_nv <= resize(p11_v, 17) + resize(p11_a, 17);
            end if;
            p12_nb <= sats(p11_nb, 12);
            p12_ad <= unsigned(abs(p11_dm));

            -- 13: fine state saturated; ridge
            p13_nf <= sats(p12_nv, 12);
            p13_nb <= p12_nb;
            if p12_ad < 384 then p13_rg <= to_unsigned(384, 9) - resize(p12_ad(8 downto 0), 9);
            else p13_rg <= (others => '0'); end if;

            -- 14: stored copies (x16, rounded); P12 depth product (split); Osc / camera terms
            v_12 := shift_right(resize(p13_nf, 13) + 8, 4);
            if v_12 > 127 then p14_wf <= to_signed(127, 8); else p14_wf <= v_12(7 downto 0); end if;
            v_12 := shift_right(resize(p13_nb, 13) + 8, 4);
            if v_12 > 127 then p14_wb <= to_signed(127, 8); else p14_wb <= v_12(7 downto 0); end if;
            p14_pm <= p13_nf * signed('0' & f_gm);
            p14_so <= resize(shift_right((resize(p13_nb, 16) sll 3) + resize(p13_nb, 16), 5), 12);
            p14_ro <= shift_right(p13_nb, 3) + shift_right(p13_nb, 6);
            p14_rv <= resize(c_rv, 12) + shift_right(p13_nb, 4);
            p14_rg <= resize(p13_rg, 10) + resize(p13_rg(8 downto 3), 10);

            -- 15: noise RAM write; sigma * N; source select
            if p_first(14) = '1' then v_ix := (others => '0'); else v_ix := n_wa + 1; end if;
            n_wa <= v_ix;
            n_wd <= std_logic_vector(p14_wf) & std_logic_vector(p14_wb);
            n_we <= s_gav_sr(13) and not v_ix(0);
            case to_integer(f_ge) is
                when 0 => v_n := resize(shift_right(p14_pm, 10), 17);
                when 1 => v_n := resize(shift_right(p14_pm, 9), 17);
                when 2 => v_n := resize(shift_right(p14_pm, 8), 17);
                when 3 => v_n := resize(shift_right(p14_pm, 7), 17);
                when 4 => v_n := resize(shift_right(p14_pm, 6), 17);
                when 5 => v_n := resize(shift_right(p14_pm, 5), 17);
                when others => v_n := resize(shift_right(p14_pm, 4), 17);
            end case;
            p15_sn <= resize(v_n, 14);
            if f_osc = '1' then
                p15_src <= p14_so;
                p_rn(16) <= p14_ro(9 downto 0);
            else
                p15_src <= resize(c_sv, 12);
                p_rn(16) <= p14_rv(9 downto 0);
            end if;
            p15_rg <= p14_rg;

            -- 16: quiet strips; ridge + band offset
            if l_clean = '1' then p16_sq <= shift_right(p15_sn, 2); else p16_sq <= p15_sn; end if;
            p16_a1 <= signed(resize(p15_rg, 12)) + resize(l_off, 12);
            p16_src <= p15_src;
            -- 17
            p17_a1 <= resize(p16_a1, 14);
            p17_a2 <= p16_sq + resize(p16_src, 14);
            -- 18
            p18_ds <= resize(p17_a1, 15) + resize(p17_a2, 15);
            -- 19: the deviation; edge x 17
            p19_dev <= sats(p18_ds, 12);
            p19_e17 <= signed(shift_left(resize(p_e(18), 13), 4)) + signed(resize(p_e(18), 13));
            -- 20: theta (live); key-path gain partial sums; Q8 copy
            v_th := unsigned(resize(f_base, 11) + resize(p19_dev, 11) + resize(p19_e17, 11));
            p20_th <= v_th(9 downto 0);
            p20_s1 <= resize(p19_dev, 14) + resize(shift_right(p19_dev, 2), 14);
            p20_s2 <= resize(shift_right(p19_dev, 3), 14) + resize(shift_right(p19_dev, 5), 14);
            v_11 := shift_right(p19_dev, 2);
            p20_d8 <= v_11(9 downto 0);
            -- 21: key-path value; red noise x 1.40625; hold RAM read address
            p21_dk <= p20_s1 + p20_s2;
            p21_rk <= resize(p_rn(20), 14) + resize(shift_right(p_rn(20), 2), 14)
                      + resize(shift_right(p_rn(20), 3), 14) + resize(shift_right(p_rn(20), 5), 14);
            if p_first(20) = '1' then c21 <= (others => '0'); else c21 <= c21 + 1; end if;
            r_wd <= std_logic_vector(p20_th) & std_logic_vector(p20_d8) & std_logic_vector(p_rn(20)) & "00";
            r_we <= '1';
            -- 22: Q8 / 5-bit key values; the block clock
            p22_dk8 <= sats(shift_right(p21_dk, 2), 10);
            p22_rk5 <= sats(shift_right(p21_rk, 3), 5);
            if p_first(21) = '1' then h_ra <= (others => '0'); else h_ra <= h_ra + 1; end if;
            if p_first(21) = '1' then
                v_a := shift_left(resize(l_shp, 17), 8) + resize(f_incp, 17);
            else
                v_a := resize(a22, 17) + resize(f_incp, 17);
            end if;
            a22 <= v_a(15 downto 0); w22 <= v_a(16);
            -- 23: sample-and-hold at the block clock (Lines: every pixel)
            if p_first(22) = '1' or w22 = '1' or f_lines = '1' then
                p23_sht <= p22_dk8;
                p23_shr <= p22_rk5;
            end if;
            -- 24: slew; hold RAM data (fabric copy)
            if p_first(23) = '1' or f_lines = '1' then
                p24_st <= shift_left(resize(p23_sht, 14), 4);
                p24_sr <= shift_left(resize(p23_shr, 9), 4);
            else
                v_13 := shift_left(resize(p23_sht, 14), 4) - p24_st;
                v_9 := shift_left(resize(p23_shr, 9), 4) - p24_sr;
                case to_integer(f_sl) is
                    when 1 => v_13 := shift_right(v_13, 1); v_9 := shift_right(v_9, 1);
                    when 2 => v_13 := shift_right(v_13, 2); v_9 := shift_right(v_9, 2);
                    when 3 => v_13 := shift_right(v_13, 3); v_9 := shift_right(v_9, 3);
                    when others => v_13 := shift_right(v_13, 4); v_9 := shift_right(v_9, 4);
                end case;
                p24_st <= p24_st + v_13;
                p24_sr <= p24_sr + v_9;
            end if;
            p24_hq <= h_q;
            p24_rq <= r_q;
            -- 25: hold: live value, capture (band start or entering the key), drift
            v_13 := shift_right(p24_st, 4);
            v_9 := shift_right(p24_sr, 4);
            v_rd := signed(p24_rq(21 downto 12));
            v_rn := signed(p24_rq(11 downto 2));
            if p_key(24) = '1' then
                p25_lt <= v_13(9 downto 0); p25_lr <= v_9(4 downto 0); v_10 := v_13(9 downto 0);
            else
                p25_lt <= v_rd; p25_lr <= sats(shift_right(v_rn, 3), 5); v_10 := v_rd;
            end if;
            p25_th <= unsigned(p24_rq(31 downto 22)); p25_rn <= v_rn;
            p25_ht <= signed(p24_hq(15 downto 6));
            p25_hr <= signed(p24_hq(5 downto 1));
            p25_dt <= resize(v_10, 12) - resize(signed(p24_hq(15 downto 6)), 12) + 32;
            if l_bst = '1' or (p_key(24) = '1' and p24_hq(0) = '0') then p25_cap <= '1'; else p25_cap <= '0'; end if;
            -- 26: the new held values
            p26_th <= p25_th; p26_rn <= p25_rn;
            if p25_cap = '1' then
                p26_nt <= p25_lt; p26_nr <= p25_lr;
            else
                p26_nt <= sats(resize(p25_ht, 12) + shift_right(p25_dt, 6), 10);
                p26_nr <= p25_hr;
            end if;
            -- 27: hold RAM write; theta select; red noise select
            if p_first(26) = '1' then v_ix := (others => '0'); else v_ix := h_wa + 1; end if;
            h_wa <= v_ix;
            h_wd <= std_logic_vector(p26_nt) & std_logic_vector(p26_nr) & p_key(26);
            h_we <= s_gav_sr(25) and not v_ix(0);
            if p_key(26) = '1' then
                v_th := unsigned(resize(f_base, 11) - 246 + shift_left(resize(p26_nt, 11), 2));
                p27_th <= v_th(9 downto 0);
                p27_rs <= shift_left(resize(p26_nr, 10), 3);
            elsif l_clean = '1' then
                v_th := unsigned(resize(f_base, 11) + shift_left(resize(p26_nt, 11), 2));
                p27_th <= v_th(9 downto 0);
                p27_rs <= shift_left(resize(p26_nr, 10), 3);
            else
                p27_th <= p26_th;
                p27_rs <= p26_rn;
            end if;
            if p_first(26) = '1' then
                if f_flip = '1' then x27 <= to_unsigned(1919 * 256, 19); else x27 <= (others => '0'); end if;
            elsif f_flip = '1' then x27 <= x27 - t_kx; else x27 <= x27 + t_kx; end if;
            -- 28: fold; red noise x 45 (partials); d for the colour ramps
            if p27_th(9) = '1' then p28_tg <= 511 - p27_th(8 downto 0); else p28_tg <= p27_th(8 downto 0); end if;
            p28_rh <= shift_right(p27_rs, 1);
            v_12 := signed(resize(x27(18 downto 8), 13)) - 960;
            v_13 := resize(shift_right(v_12, 5), 14) + resize(shift_right(v_12, 6), 14) + resize(shift_right(v_12, 8), 14);
            p28_sa <= v_13(7 downto 0);
            p28_d <= signed(resize(x27(18 downto 8), 14)) - l_thr;
            -- 29: green level, blue fold, red noise; green VCA, blue bias, edge
            if p28_tg < 77 then p29_g8 <= (others => '0');
            else p29_g8 <= resize(shift_right(p28_tg - 77, 1), 8); end if;
            p29_b9a <= signed(resize(511 - p28_tg, 11)) - 51;
            p29_ra <= (l_step & "00000") + unsigned(p28_sa);
            p29_rh <= p28_rh;
            v_d := to_signed(1550, 15) - resize(p28_d, 15);
            if p_key(28) = '1' then p29_vg <= (others => '1');
            elsif v_d < 0 then p29_vg <= (others => '0');
            elsif v_d(14 downto 10) /= 0 then p29_vg <= (others => '1');
            else p29_vg <= unsigned(v_d(9 downto 5)); end if;
            v_d := to_signed(400, 15) - resize(p28_d, 15);
            v_d2 := to_signed(850, 15) - resize(p28_d, 15);
            if v_d > 0 then p29_bb <= resize(shift_right(v_d, 1), 12);
            elsif v_d2 < 0 then p29_bb <= resize(shift_right(v_d2, 1), 12);
            else p29_bb <= (others => '0'); end if;
            if l_kline = '0' then p29_e <= (others => '0');
            elsif p28_d < 0 then p29_e <= to_unsigned(128, 8);
            elsif p28_d < 128 then p29_e <= to_unsigned(128, 8) - unsigned(p28_d(7 downto 0));
            else p29_e <= (others => '0'); end if;
            -- 30: green x VCA; blue x 1.1875; red level; blue bias (+ edge fringe)
            p30_gp <= resize(p29_g8 * p29_vg, 13) + resize(p29_g8, 13);
            p30_b9b <= resize(p29_b9a, 12) + resize(shift_right(p29_b9a, 3), 12) + resize(shift_right(p29_b9a, 4), 12);
            p30_rh <= p29_rh;
            if l_clean = '1' then p30_bbe <= resize(p29_bb, 13);
            else p30_bbe <= resize(p29_bb, 13) + signed(shift_left(resize(p29_e, 13), 1)); end if;
            -- 31
            p31_g8 <= p30_gp(12 downto 5);
            p31_b9 <= resize(p30_b9b, 14) + resize(p30_bbe, 14);
            p31_rl <= unsigned(rseq);
            p31_rh <= p30_rh;
            -- 32: blue level (full in the key)
            -- red: sequence level + red noise / 2 (+16 in the mosaic)
            v_11 := signed(resize(p31_rl, 12)) + resize(p31_rh, 12);
            if p_key(31) = '1' then v_11 := v_11 + 16; end if;
            if v_11 < 0 then p32_r <= (others => '0');
            elsif v_11 > 255 then p32_r <= (others => '1');
            else p32_r <= unsigned(v_11(7 downto 0)); end if;
            p32_g <= p31_g8;
            if p_key(31) = '1' then p32_b <= (others => '1');
            elsif p31_b9 < 0 then p32_b <= (others => '0');
            elsif p31_b9 > 511 then p32_b <= (others => '1');
            else p32_b <= unsigned(p31_b9(8 downto 1)); end if;
            -- 33: ROM outputs (p_rom); 34: sums
            p34_ys <= resize(unsigned(rom_r(30 downto 20)), 12) + resize(unsigned(rom_g(30 downto 20)), 12)
                      + resize(unsigned(rom_b(30 downto 20)), 12);
            p34_us <= signed(resize(unsigned(rom_r(19 downto 10)), 12)) - signed(resize(unsigned(rom_g(19 downto 10)), 12))
                      - signed(resize(unsigned(rom_b(19 downto 10)), 12));
            p34_vs <= signed(resize(unsigned(rom_b(9 downto 0)), 12)) - signed(resize(unsigned(rom_r(9 downto 0)), 12))
                      - signed(resize(unsigned(rom_g(9 downto 0)), 12));
            -- 35: offsets; luma clamp (pre-boosted 16/13; the conditioning takes white to ~777).
            -- Chroma stays within 512 +- 223 by construction; neutral outside the active line.
            v_y := resize(p34_ys(11 downto 1), 12) + 64;
            if v_y > 940 then p35_y <= to_unsigned(940, 10); else p35_y <= v_y(9 downto 0); end if;
            v_11 := shift_right(p34_us, 1) + 512;
            if s_gav_sr(33) = '1' then p35_u <= unsigned(v_11(9 downto 0)); else p35_u <= to_unsigned(512, 10); end if;
            v_11 := shift_right(p34_vs, 1) + 512;
            if s_gav_sr(33) = '1' then p35_v <= unsigned(v_11(9 downto 0)); else p35_v <= to_unsigned(512, 10); end if;
            -- 36: pad
            p36_y <= p35_y; p36_u <= p35_u; p36_v <= p35_v;

            -- 37..42: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]; centre at stage 41 -> 42)
            -- 8-sums from the chroma ring; re-anchored to 8 x neutral once the window is all blanking
            if s_gav_sr(35) = '0' and s_gav_sr(42) = '0' then
                s_boxu <= to_unsigned(4096, 14); s_boxv <= to_unsigned(4096, 14);
            else
                s_boxu <= s_boxu + p36_u - unsigned(c_old(19 downto 10));
                s_boxv <= s_boxv + p36_v - unsigned(c_old(9 downto 0));
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u41 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v41 <= v_sc(13 downto 4);
            s_y36 <= p36_y; s_y37 <= s_y36; s_y38 <= s_y37; s_y39 <= s_y38;
            s_y40 <= s_y39; s_y41x <= s_y40;
            v_sy := resize(s_y39, 12) + shift_left(resize(s_y40, 12), 1) + resize(s_y41x, 12);
            s_yf41 <= v_sy(11 downto 2);
            -- 43: luma toward black (white 940 -> 777); blanking gate
            if s_yf41 > 64 then v_dly := s_yf41 - 64; else v_dly := (others => '0'); end if;
            if s_gav_sr(41) = '1' then
                s_y42 <= s_yf41 - shift_right(v_dly, 3) - shift_right(v_dly, 4);
                s_u42 <= s_u41; s_v42 <= s_v41;
            else
                s_y42 <= to_unsigned(64, 10); s_u42 <= to_unsigned(512, 10); s_v42 <= to_unsigned(512, 10);
            end if;
            -- 44: output register (44-clock latency)
            s_y44 <= s_y42; s_u44 <= s_u42; s_v44 <= s_v42;

            -- delays
            p_first(1) <= b_first;
            for i in 2 to 26 loop p_first(i) <= p_first(i - 1); end loop;
            for i in 3 to 32 loop p_key(i) <= p_key(i - 1); end loop;
            for i in 13 to 18 loop p_e(i) <= p_e(i - 1); end loop;
            for i in 17 to 20 loop p_rn(i) <= p_rn(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y44);
    data_out.u       <= std_logic_vector(s_u44);
    data_out.v       <= std_logic_vector(s_v44);
    data_out.hsync_n <= s_out(3);
    data_out.vsync_n <= s_out(2);
    data_out.field_n <= s_out(1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture streak;
