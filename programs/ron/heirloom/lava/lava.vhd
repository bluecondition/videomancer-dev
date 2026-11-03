-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lava v0.6.0 (phase 2) -- Heirloom #43, rebuilt from the photo WIN_20230605_01_14_56.
--
-- The photo: a red, glossy lava sheet hangs over violet / magenta vertical stripes.  Across the right half
-- it is a lattice of rows -- peach-cored worms, a red zigzag rim, olive diamonds with pink hearts, a white web
-- with orange highlights, rimmed red blobs, green / violet scallops -- and on the left the same rows bend up
-- into long hanging drapes whose tips step down to the right, with the stripes showing below them.  Every
-- column of it is ONE period (167 px) of a horizontal oscillator; the rows are a warped vertical coordinate.
--
-- The program draws exactly that: the oscillator phase picks a column of a stored TILE (one period of the
-- photo, fitted: 24 x 164 luma knots, 12 x 82 chroma knots, bilinear), and a stored WARP (48 x 42 knots, fitted)
-- gives the row coordinate t(x, y).  A per-line engine interpolates the warp row for the NEXT line; the pixel
-- path interpolates it across the line, applies the controls to t, adds the 8.5-line comb to the phase and
-- reads the tile.
--
-- Phase 2 (v0.6.0): the drops, the row bands and each drop's length are separate modules.
--   K1  Columns  the drops (and stripes) across, 0.25x .. 4x (exponential, the photo at 50 %)
--   K2  Bend     the drapes: -1 (the rows bow DOWN on the left: a valley) .. 0 flat .. 1 the photo (50 %) .. 3
--   K3  Rows     the row bands' heights, bipolar: left = the top bands (worms) swell and the lower bands squeeze
--                to the edge, right = the reverse; the sheet's outline stays (a per-field remap RAM of t)
--   K4  Speed    bipolar (centre +-8 still): the drops slide sideways, twice v0.5's rate per step (the photo's
--                drift at the new default 560, one drop per second at the ends)
--   K5  Drip     bipolar: each drop hangs its own tongue (right, down) or pulls up into an arch (left), every
--                drop a different length, breathing slowly; the edge serration deepens with it
--   K6  Melt     how far the video (or Osc) bends the rows: K6 / 16, twice v0.5's gain per step (the photo's at
--                the new default 100, about +-500 lines at the top)
--   S7  Mirror   the left half reflected: twin curtains
--   S8  Ground   Stripes / Camera (the camera's luma in violet, or the Osc waves as clouds, under the lava)
--   S9  Flip     the sheet rises from the bottom
--   S10 Palette  Lava / Ice (U and V exchanged)
--   S11 Source   Video / Osc
--   P12 FLOW     0 % = the lava drawn back into the top-right corner, 50 % = the photo, 100 % = it floods down
--
-- Virtual 1080-line geometry (the same picture at SD, 720p and 1080; 4:3 SD shows the central 1440 virtual px).
-- Generated avid; output conditioning (luma [1 2 1], chroma box pair [1 2 .. 2 1], luma to ~777) before the
-- blanking gate.  Latency 32 clocks in every mode (v0.5: 36).  Generated colours in the hardware convention (U = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lava_pkg.all;
use work.all;

architecture lava of program_top is
    constant C_LAT : integer := 32;          -- matches processing_delay_clks (v0.5: 36; four output copies dropped)
    constant C_K0  : integer := 100523;      -- 2^24 / 166.9: the photo's oscillator, phase per virtual px
    constant C_PHC : integer := 1156009;     -- frac((960 - 114) / 166.9) * 2^24
    constant C_KVG : integer := 100424;      -- C_K0 * 1023 / 1024: the photo's oscillator at the default Columns

    -- per-timing-ID constants
    type t_mode is record
        vx   : unsigned(9 downto 0);   -- virtual px per pixel, Q8
        ku   : unsigned(11 downto 0);  -- warp cells (42 virtual px) per pixel, Q16
        u0   : unsigned(18 downto 0);  -- warp cell of the first pixel, Q16
        xm   : unsigned(9 downto 0);   -- 960 - first virtual px
        cmid : unsigned(10 downto 0);  -- pixel index of the virtual centre
    end record;
    function f_mode(t : std_logic_vector(3 downto 0)) return t_mode is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return (to_unsigned(512, 10), to_unsigned(3121, 12), to_unsigned(374491, 19),
                                           to_unsigned(720, 10), to_unsigned(360, 11));
            when 5 | 6 | 14     => return (to_unsigned(384, 10), to_unsigned(2341, 12), to_unsigned(0, 19),
                                           to_unsigned(960, 10), to_unsigned(640, 11));
            when others         => return (to_unsigned(256, 10), to_unsigned(1560, 12), to_unsigned(0, 19),
                                           to_unsigned(960, 10), to_unsigned(960, 11));
        end case;
    end function;
    -- virtual lines per line (Q8), knot-row step per line (Q16 of 27 lines), comb step (Q16 of 8.5 lines)
    type t_vmode is record
        vy    : unsigned(10 downto 0);
        jst   : unsigned(13 downto 0);
        jf0   : unsigned(12 downto 0);
        comb  : unsigned(15 downto 0);
    end record;
    function f_vmode(t : std_logic_vector(3 downto 0)) return t_vmode is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return (to_unsigned(1138, 11), to_unsigned(10788, 14), to_unsigned(5394, 13), to_unsigned(34274, 16));
            when 4          => return (to_unsigned(576, 11),  to_unsigned(5460, 14),  to_unsigned(2730, 13), to_unsigned(17348, 16));
            when 8          => return (to_unsigned(960, 11),  to_unsigned(9101, 14),  to_unsigned(4550, 13), to_unsigned(28913, 16));
            when 12         => return (to_unsigned(480, 11),  to_unsigned(4550, 14),  to_unsigned(2275, 13), to_unsigned(14456, 16));
            when 5 | 6 | 14 => return (to_unsigned(384, 11),  to_unsigned(3640, 14),  to_unsigned(1820, 13), to_unsigned(11565, 16));
            when 1 | 2 | 10 => return (to_unsigned(512, 11),  to_unsigned(4854, 14),  to_unsigned(2427, 13), to_unsigned(15420, 16));
            when others     => return (to_unsigned(256, 11),  to_unsigned(2427, 14),  to_unsigned(1213, 13), to_unsigned(7710, 16));
        end case;
    end function;
    function f_isshort(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 | 5 | 6 | 14 => return '1';
            when others => return '0';
        end case;
    end function;
    -- 16-bit phase -> triangle -129 .. 127
    function f_tri(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 10) - signed(resize(v(15 downto 7), 10));
    end function;

    -- 8-bit phase -> triangle 0 .. 254 (peak at 127 / 128)
    function f_tri8(p : unsigned(7 downto 0)) return unsigned is
        variable v : unsigned(6 downto 0);
    begin
        if p(7) = '1' then v := not p(6 downto 0); else v := p(6 downto 0); end if;
        return v & '0';
    end function;

    -- timing
    signal t_m  : t_mode := (to_unsigned(256, 10), to_unsigned(1560, 12), to_unsigned(0, 19), to_unsigned(960, 10), to_unsigned(960, 11));
    signal t_v  : t_vmode := (to_unsigned(256, 11), to_unsigned(2427, 14), to_unsigned(1213, 13), to_unsigned(7710, 16));
    signal t_short : std_logic := '0';
    signal f_cmid2 : unsigned(10 downto 0) := to_unsigned(958, 11);     -- the pixel before the Mirror turn (cmid - 2)
    signal b_hit, lp_hit : std_logic := '0';                            -- the Mirror turn, flagged a clock early

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K1, K2, K3, K5, K6, P12 (Q.3), power up at the defaults
    type t_g is array(0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(512 * 8, 13), to_unsigned(100 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- serial multiplier (per-field products)
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 7) := (others => '0');
    signal m_a  : unsigned(19 downto 0) := (others => '0');
    signal m_b  : unsigned(17 downto 0) := (others => '0');
    signal m_h  : unsigned(19 downto 0) := (others => '0');
    signal m_l  : unsigned(17 downto 0) := (others => '0');

    -- per-field values
    signal f_kv    : unsigned(18 downto 0) := to_unsigned(100424, 19);   -- phase per virtual px (Q24)
    signal f_kph   : unsigned(19 downto 0) := to_unsigned(100424, 20);   -- phase per pixel
    signal f_ph0   : unsigned(23 downto 0) := (others => '0');           -- phase of the first pixel
    signal f_p0    : unsigned(20 downto 0) := (others => '0');           -- {drop index 5, phase 16} at virtual x 0
    signal f_ks    : unsigned(16 downto 0) := (others => '0');           -- phase per warp knot (42 virtual px), Q16
    signal f_drift : unsigned(28 downto 0) := (others => '0');           -- {drops, phase}: the index survives wraps
    signal f_dstep : unsigned(18 downto 0) := (others => '0');
    signal f_sm    : unsigned(8 downto 0) := (others => '0');
    signal f_sneg  : std_logic := '0';
    signal f_bend  : signed(9 downto 0) := to_signed(128, 10);          -- Q7, -1 .. 2.25
    signal f_ca    : unsigned(11 downto 0) := (others => '0');           -- comb gain (phase / 2^21 per tri unit)
    signal f_ramp  : unsigned(7 downto 0) := to_unsigned(40, 8);         -- comb amplitude, Q4 px
    signal f_melt  : unsigned(5 downto 0) := to_unsigned(6, 6);          -- Melt gain (src * m / 32 lines)
    -- ground stripes: their own oscillator at the photo's pitch (= the drops' at the default Columns)
    -- Drip: depth (lines, signed: negative hangs down), breath phase
    signal f_dd    : signed(9 downto 0) := (others => '0');
    signal f_td    : unsigned(9 downto 0) := (others => '0');
    -- Drip knot table: the tongue offset (signed lines) of each of the 48 warp knots, built per field into the
    -- bank of the field parity (the line engine reads the other bank) and added to the knot's t
    signal f_kb    : std_logic := '0';
    signal kb_on   : std_logic := '0';
    signal kb_cnt  : unsigned(10 downto 0) := (others => '0');          -- knot = cnt(10..5), step = cnt(4..0)
    signal kb_ph   : unsigned(20 downto 0) := (others => '0');
    signal kb_md   : unsigned(8 downto 0) := (others => '0');
    signal kb_mr   : unsigned(7 downto 0) := (others => '0');
    signal kb_acc  : unsigned(17 downto 0) := (others => '0');
    signal kt_we   : std_logic := '0';
    signal kt_wa, kt_ra : unsigned(7 downto 0) := (others => '0');
    signal kt_wd   : std_logic_vector(9 downto 0) := (others => '0');
    type t_kt is array (0 to 255) of std_logic_vector(9 downto 0);
    signal kt_ram  : t_kt := (others => (others => '0'));
    signal kt_q, kt_q2 : std_logic_vector(9 downto 0) := (others => '0');
    -- Rows: band remap t -> t' = t + c t (1034 - t) / 1034, c in Q10 (field-latched); c = 0 bypasses it
    signal f_rc0   : std_logic := '1';
    signal f_rc    : signed(10 downto 0) := (others => '0');             -- c, field-latched
    signal rw_on   : std_logic := '0';
    signal rw_i    : unsigned(9 downto 0) := (others => '0');
    signal rw_g    : signed(24 downto 0) := (others => '0');             -- G, Q16 lines
    signal rw_d    : signed(19 downto 0) := (others => '0');
    signal rm_ra   : unsigned(9 downto 0) := (others => '0');
    signal rm_g13  : signed(12 downto 0);
    signal rm_wd   : std_logic_vector(10 downto 0);                     -- (combinational from the writer)
    type t_rmap is array (0 to 1023) of std_logic_vector(10 downto 0);
    signal rm_ram  : t_rmap := (others => (others => '0'));
    signal f_flow  : signed(10 downto 0) := (others => '0');
    signal f_fk    : signed(10 downto 0) := (others => '0');
    signal f_lph   : unsigned(15 downto 0) := to_unsigned(16384, 16);    -- breath LFO (starts at its zero)
    signal f_br    : signed(6 downto 0) := (others => '0');
    signal f_comb0 : unsigned(15 downto 0) := (others => '0');
    signal f_ta, f_tb : unsigned(15 downto 0) := (others => '0');
    signal f_mir, f_cam, f_flip, f_ice, f_osc : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- per-line engine
    signal e_yv   : unsigned(18 downto 0) := (others => '0');            -- virtual line, Q8
    signal e_jf   : unsigned(21 downto 0) := (others => '0');            -- warp knot row, Q16
    signal e_comb : unsigned(15 downto 0) := (others => '0');
    signal e_wb, r_bank : std_logic := '0';
    signal e_st   : unsigned(4 downto 0) := (others => '0');
    signal e_ye   : unsigned(10 downto 0) := (others => '0');            -- the line's virtual line (flipped)
    signal e_j    : unsigned(5 downto 0) := (others => '0');
    signal e_fy   : unsigned(7 downto 0) := (others => '0');
    signal e_base : unsigned(10 downto 0) := (others => '0');
    signal e_ct   : signed(9 downto 0) := (others => '0');
    signal e_ym   : signed(11 downto 0) := (others => '0');
    signal em_a   : signed(23 downto 0) := (others => '0');             -- serial multiplier
    signal em_b   : unsigned(11 downto 0) := (others => '0');
    signal em_acc : signed(23 downto 0) := (others => '0');
    signal e_lrp  : signed(11 downto 0) := (others => '0');
    signal e_af   : signed(13 downto 0) := (others => '0');
    signal e_iss  : std_logic := '0';
    signal e_i    : unsigned(5 downto 0) := (others => '0');
    signal e_r    : std_logic := '0';
    signal e_tv   : std_logic_vector(0 to 3) := (others => '0');
    signal e_tr   : std_logic_vector(0 to 3) := (others => '0');
    type t_u6a is array (natural range <>) of unsigned(5 downto 0);
    signal e_ti   : t_u6a(0 to 3) := (others => (others => '0'));
    signal r_wa   : unsigned(10 downto 0) := (others => '0');
    signal wr_q, wr_q2 : std_logic_vector(7 downto 0) := (others => '0');
    signal e_ka   : signed(7 downto 0) := (others => '0');
    signal e_dab  : signed(8 downto 0) := (others => '0');
    signal e_ka2  : signed(7 downto 0) := (others => '0');
    signal e_pr   : signed(17 downto 0) := (others => '0');
    signal e_kv, e_kv2, e_kv3, e_kv4 : std_logic := '0';
    signal e_ki, e_ki2, e_ki3, e_ki4 : unsigned(5 downto 0) := (others => '0');
    signal e_ko, e_ko2, e_ko3 : signed(9 downto 0) := (others => '0');   -- the knot's Drip offset
    signal e_yo   : signed(12 downto 0) := (others => '0');              -- ye + offset
    signal e_bh   : signed(18 downto 0) := (others => '0');              -- Bend x warp, split 6 | 4
    signal e_bl   : signed(17 downto 0) := (others => '0');
    signal e_bs   : signed(22 downto 0) := (others => '0');
    signal d_ye   : unsigned(10 downto 0) := (others => '0');
    signal d_lrp  : signed(11 downto 0) := (others => '0');
    signal d_af   : signed(13 downto 0) := (others => '0');
    signal l_ye   : unsigned(10 downto 0) := (others => '0');            -- the line on screen
    signal l_lrp  : signed(11 downto 0) := (others => '0');
    signal l_af   : signed(13 downto 0) := (others => '0');

    -- line table (ping-pong by line): even / odd knots, t (13 bit signed)
    type t_lt is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lt_e, lt_o : t_lt := (others => (others => '0'));
    signal lt_we_e, lt_we_o : std_logic := '0';
    signal lt_wa : unsigned(7 downto 0) := (others => '0');
    signal lt_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal lt_rae, lt_rao : unsigned(7 downto 0) := (others => '0');
    signal lt_qe, lt_qo : std_logic_vector(15 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of lt_e : signal is true;
    attribute no_rw_check of lt_o : signal is true;
    attribute no_rw_check of rm_ram : signal is true;
    attribute no_rw_check of kt_ram : signal is true;

    -- beam: warp cell (stage 0), the oscillator phase (a late copy of the beam, stage 13), Osc phases
    signal b_u   : unsigned(21 downto 0) := (others => '0');
    signal b_cnt : unsigned(10 downto 0) := (others => '0');
    signal b_dir : std_logic := '0';
    signal b_pa  : unsigned(15 downto 0) := (others => '0');
    signal l_pb  : unsigned(15 downto 0) := (others => '0');
    signal gs_sr : std_logic_vector(0 to 14) := (others => '0');
    signal lp_ph : unsigned(23 downto 0) := (others => '0');
    signal lp_cnt : unsigned(10 downto 0) := (others => '0');
    signal lp_dir : std_logic := '0';


    -- video tap
    type t_u8v is array (natural range <>) of unsigned(7 downto 0);
    signal s_vy : t_u8v(1 to 2) := (others => to_unsigned(128, 8));
    signal s_vs : signed(9 downto 0) := (others => '0');
    -- camera ring (block RAM): the camera luma (8 bits), read back at the compose stage for Ground = Camera
    type t_vring is array (0 to 255) of std_logic_vector(7 downto 0);
    signal rv_ram : t_vring := (others => x"10");
    attribute no_rw_check of rv_ram : signal is true;
    signal rv_wd, rv_q, rv_q2 : std_logic_vector(7 downto 0) := x"10";
    signal rv_ra : unsigned(7 downto 0) := (others => '0');

    -- pixel path (stage N = N registers after the beam)
    signal q1_fx, q2_fx, q3_fx, q4_fx, q5_fx : unsigned(7 downto 0) := (others => '0');
    signal q1_odd, q2_odd, q3_odd : std_logic := '0';
    signal q3_e, q3_o : signed(12 downto 0) := (others => '0');
    signal q4_w0, q4_w1, q5_w0, q6_w0, q7_w0 : signed(12 downto 0) := (others => '0');
    signal q5_dw : signed(13 downto 0) := (others => '0');
    signal q6_ph, q6_pl : signed(17 downto 0) := (others => '0');
    signal q7_pr : signed(21 downto 0) := (others => '0');
    signal q8_t  : signed(13 downto 0) := (others => '0');
    signal q8_src : signed(9 downto 0) := (others => '0');
    signal q9_d  : signed(14 downto 0) := (others => '0');
    signal q9_ms : signed(16 downto 0) := (others => '0');
    type t_bit_a is array (natural range <>) of std_logic;
    signal p_lf  : t_bit_a(9 to 15) := (others => '0');
    signal q10_d, q11_d : signed(14 downto 0) := (others => '0');       -- t - ye (Bend is applied per knot)
    signal q10_ms, q11_ms : signed(18 downto 0) := (others => '0');
    signal q12_t3 : signed(15 downto 0) := (others => '0');
    signal q13_tm : unsigned(10 downto 0) := (others => '0');
    signal p_bg  : t_bit_a(13 to 27) := (others => '0');
    signal rm_q, rm_q2 : std_logic_vector(10 downto 0) := (others => '0');   -- the Rows remap RAM (stage 14, 15)
    signal q14_tm, q15_tm : unsigned(10 downto 0) := (others => '0');
    signal q16_vy, q16_vc : unsigned(15 downto 0) := (others => '0');
    signal q16_q : unsigned(11 downto 0) := (others => '0');
    signal q17_uy, q17_uc : unsigned(16 downto 0) := (others => '0');
    signal q17_jy, q17_jc : unsigned(7 downto 0) := (others => '0');
    signal q17_fvy : unsigned(3 downto 0) := (others => '0');
    signal q18_iy0, q18_iy1 : unsigned(4 downto 0) := (others => '0');
    signal q18_ic0, q18_ic1 : unsigned(3 downto 0) := (others => '0');
    signal q18_jy0, q18_jy1 : unsigned(7 downto 0) := (others => '0');
    signal q18_jc : unsigned(6 downto 0) := (others => '0');
    type t_u4_a is array (natural range <>) of unsigned(3 downto 0);
    signal p_fuy, p_fuc : t_u4_a(18 to 22) := (others => (others => '0'));
    signal p_fvy : t_u4_a(18 to 25) := (others => (others => '0'));
    type t_u2_a is array (natural range <>) of std_logic_vector(1 downto 0);
    signal p_py : t_u2_a(18 to 21) := (others => (others => '0'));       -- (i0 odd, j0 odd)
    signal p_pc : t_bit_a(18 to 21) := (others => '0');                  -- i0 odd (chroma)
    signal ry_a0, ry_a1, ry_a2, ry_a3 : unsigned(9 downto 0) := (others => '0');
    signal rc_a0, rc_a1 : unsigned(8 downto 0) := (others => '0');
    signal ry_q0, ry_q1, ry_q2, ry_q3 : std_logic_vector(7 downto 0) := (others => '0');
    signal rc_q0, rc_q1 : std_logic_vector(15 downto 0) := (others => '0');
    signal q21_y0, q21_y1, q21_y2, q21_y3 : std_logic_vector(7 downto 0) := (others => '0');
    signal q21_c0, q21_c1 : std_logic_vector(15 downto 0) := (others => '0');
    signal q22_c00, q22_c01, q23_c00, q23_c01 : signed(9 downto 0) := (others => '0');
    signal q22_d0, q22_d1 : signed(9 downto 0) := (others => '0');
    signal q22_r0, q22_b0, q23_r0, q23_b0 : signed(8 downto 0) := (others => '0');
    signal q22_rd, q22_bd : signed(9 downto 0) := (others => '0');
    signal q23_p0, q23_p1 : signed(13 downto 0) := (others => '0');
    signal q23_pr, q23_pb : signed(13 downto 0) := (others => '0');
    signal q24_a, q24_b, q25_a, q26_a : signed(9 downto 0) := (others => '0');
    signal q24_cr, q24_cb, q25_cr, q25_cb, q26_cr, q26_cb, q27_cr, q27_cb : signed(9 downto 0) := (others => '0');
    signal q25_e : signed(9 downto 0) := (others => '0');
    signal q26_pe : signed(13 downto 0) := (others => '0');
    signal q27_y : signed(9 downto 0) := (others => '0');
    signal q28_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q28_u, q28_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- chroma box ring and sync ring
    type t_ring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal rb_ram : t_ring := (others => "1000000000" & "1000000000");
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal rs_ram : t_sring := (others => "111");
    attribute no_rw_check of rb_ram : signal is true;
    attribute no_rw_check of rs_ram : signal is true;
    signal s_wp, rb_ra, rs_ra : unsigned(7 downto 0) := (others => '0');
    signal rb_wd : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal rb_q, rb_q2 : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal rs_q, rs_q2 : std_logic_vector(2 downto 0) := "111";
    signal s_usum, s_vsum, s_ud, s_vd : unsigned(13 downto 0) := (others => '0');
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');   -- ring-input gate of the last 8 box inputs
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly : t_u10a(0 to 6) := (others => to_unsigned(64, 10));
    signal q31_y, q32_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q31_u, q31_v, q32_u, q32_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs
    ---------------------------------------------------------------------------
    p_wrom : process(clk)
    begin
        if rising_edge(clk) then
            wr_q <= C_LV_WARP(to_integer(r_wa));
        end if;
    end process p_wrom;

    p_lte : process(clk)
    begin
        if rising_edge(clk) then
            if lt_we_e = '1' then lt_e(to_integer(lt_wa)) <= lt_wd; end if;
            lt_qe <= lt_e(to_integer(lt_rae));
        end if;
    end process p_lte;

    p_lto : process(clk)
    begin
        if rising_edge(clk) then
            if lt_we_o = '1' then lt_o(to_integer(lt_wa)) <= lt_wd; end if;
            lt_qo <= lt_o(to_integer(lt_rao));
        end if;
    end process p_lto;

    -- entries past 517 (t > 1034) are never read; up to there the curve stays in 0 .. 1034 (monotone)
    rm_g13 <= signed(resize(shift_left(resize(rw_i, 12), 1), 13)) + resize(rw_g(24 downto 16), 13);
    rm_wd  <= std_logic_vector(rm_g13(10 downto 0));

    p_rmap : process(clk)                                   -- Rows remap: written in the vblank zone
    begin
        if rising_edge(clk) then
            if rw_on = '1' then rm_ram(to_integer(rw_i)) <= rm_wd; end if;
            rm_q <= rm_ram(to_integer(rm_ra));
        end if;
    end process p_rmap;

    p_kt : process(clk)                                     -- Drip knot table (banked by field parity)
    begin
        if rising_edge(clk) then
            if kt_we = '1' then kt_ram(to_integer(kt_wa)) <= kt_wd; end if;
            kt_q <= kt_ram(to_integer(kt_ra));
            kt_q2 <= kt_q;
        end if;
    end process p_kt;

    p_vring : process(clk)                                  -- camera ring (Ground = Camera)
    begin
        if rising_edge(clk) then
            rv_ram(to_integer(s_wp)) <= rv_wd;
            rv_q <= rv_ram(to_integer(rv_ra));
            rv_q2 <= rv_q;
        end if;
    end process p_vring;

    p_ty0 : process(clk) begin if rising_edge(clk) then ry_q0 <= C_LV_Y0(to_integer(ry_a0)); end if; end process;
    p_ty1 : process(clk) begin if rising_edge(clk) then ry_q1 <= C_LV_Y1(to_integer(ry_a1)); end if; end process;
    p_ty2 : process(clk) begin if rising_edge(clk) then ry_q2 <= C_LV_Y2(to_integer(ry_a2)); end if; end process;
    p_ty3 : process(clk) begin if rising_edge(clk) then ry_q3 <= C_LV_Y3(to_integer(ry_a3)); end if; end process;
    p_tc0 : process(clk) begin if rising_edge(clk) then rc_q0 <= C_LV_C0(to_integer(rc_a0)); end if; end process;
    p_tc1 : process(clk) begin if rising_edge(clk) then rc_q1 <= C_LV_C1(to_integer(rc_a1)); end if; end process;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field products (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(20 downto 0);
        variable v_pr : unsigned(37 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_k  : signed(10 downto 0);
        variable v_13 : signed(14 downto 0);
        variable v_t  : signed(9 downto 0);
        variable v_b  : signed(14 downto 0);
        variable v_w  : unsigned(9 downto 0);
        variable v_c  : signed(11 downto 0);
        variable v_h  : unsigned(7 downto 0);
        variable v_p0 : unsigned(28 downto 0);
    begin
        if rising_edge(clk) then
            t_m     <= f_mode(registers_in(8)(3 downto 0));
            t_v     <= f_vmode(registers_in(8)(3 downto 0));
            t_short <= f_isshort(registers_in(8)(3 downto 0));
            f_cmid2 <= t_m.cmid - 2;

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
                -- K4 Speed: bipolar, deadband 8, twice v0.5's rate per step (one drop per second at the ends);
                -- the photo's drift (v0.5 default 600) at the new default 560
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 8 then f_sneg <= '0'; v_u := unsigned(v_x) - 8; f_sm <= v_u(8 downto 0);
                elsif v_x < -8 then f_sneg <= '1'; v_u := unsigned(-v_x) - 8; f_sm <= v_u(8 downto 0);
                else f_sneg <= '0'; f_sm <= (others => '0'); end if;
                f_kb <= not f_kb;                                                   -- Drip table bank
                -- Rows: the remap curve is latched here and written into the RAM from zone clock 2
                -- K3 Rows: c = 2 (K3 - 512 -+ 8) (Q10, +-0.98), deadband +-8 = no remap (the photo)
                v_c := signed(resize(s_g(2)(12 downto 3), 12)) - 512;
                if v_c > 8 then f_rc <= resize(shift_left(v_c - 8, 1), 11); f_rc0 <= '0';
                elsif v_c < -8 then f_rc <= resize(shift_left(v_c + 8, 1), 11); f_rc0 <= '0';
                else f_rc <= (others => '0'); f_rc0 <= '1'; end if;
                f_mir  <= registers_in(6)(0);
                f_cam  <= registers_in(6)(1);
                f_flip <= registers_in(6)(2);
                f_ice  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- the drops drift (+ = right), the breath LFO, the comb crawls, the Osc drift
                if f_sneg = '1' then f_drift <= f_drift - resize(f_dstep, 29);
                else f_drift <= f_drift + resize(f_dstep, 29); end if;
                f_lph <= f_lph + 91;
                f_td <= f_td + 3;                                                   -- Drip breath, ~5.7 s
                f_comb0 <= f_comb0 + 7417;
                f_ta <= f_ta + 151; f_tb <= f_tb - 113;
            end if;

            -- glides: ease 1/8 of the gap per field (deadband 4); one channel per four zone clocks (4..27)
            if s_zone = '1' and s_zc >= 4 and s_zc < 28 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when 4 => gl_k <= unsigned(registers_in(5)) & "000"; gl_o <= s_g(4);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(5);
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
                                when 4 => s_g(4) <= gl_n;
                                when others => s_g(5) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- knob curves (settle long before the sequencer and the pixel path read them)
            -- K1 Columns (slot 1): kv = C_KVG 2^(4 K1 / 1024) / 4 (piecewise-linear exp2: the octave picks the
            -- multiplicand, the mantissa is the multiplier), x0.25 .. x4, the photo (kv = 100424) at 512
            -- K2 Bend (Q7): K2 / 2 - 128: -1 at 0 .. 1 (the photo) at 512 .. 3 at 1023
            v_g := s_g(1)(12 downto 3);
            f_bend <= signed(resize(shift_right(v_g, 1), 10)) - 128;
            -- K5 Drip: depth -3/4 line per knob step from the centre (+ = tongues hang down = t pulled up);
            -- the comb serration grows with |Drip| (2.5 px at the centre .. 10 px at the ends)
            v_c := signed(resize(s_g(3)(12 downto 3), 12)) - 512;
            f_dd <= resize(shift_right(v_c, 2) - v_c, 10);
            if v_c(11) = '1' then v_w := unsigned(not v_c(9 downto 0)); else v_w := unsigned(v_c(9 downto 0)); end if;
            f_ramp <= to_unsigned(40, 8) + resize(shift_right(v_w, 2), 8);
            -- K6 Melt: K6 / 16 (0 .. 63, twice v0.5's gain per step); the photo's 6 at the new default 100
            f_melt <= s_g(4)(12 downto 7);
            v_k := signed(resize(s_g(5)(12 downto 3), 11)) - 512;                  -- P12 Flow: +-416 lines
            v_13 := shift_left(resize(v_k, 15), 3) + shift_left(resize(v_k, 15), 2) + resize(v_k, 15);
            f_fk <= resize(shift_right(v_13, 4), 11);
            v_t := f_tri(f_lph);                                                    -- breath: +-20 lines
            v_b := shift_left(resize(v_t, 15), 4) + shift_left(resize(v_t, 15), 2);
            f_br <= resize(shift_right(v_b, 7), 7);
            f_flow <= f_fk + resize(f_br, 11);

            -- serial products: slot p = zc/32 (1..4), phase 0 load, 1..18 step, 20 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 7 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 7 and v_ph >= 1 and v_ph <= 18 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 7 loop
                if s_zone = '1' and v_p = k and v_ph = 20 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 1 =>                                    m_b <= resize("1" & s_g(0)(10 downto 3) & "00", 18);
                        case to_integer(s_g(0)(12 downto 11)) is
                            when 0 => m_a <= to_unsigned(C_KVG, 20);
                            when 1 => m_a <= to_unsigned(C_KVG * 2, 20);
                            when 2 => m_a <= to_unsigned(C_KVG * 4, 20);
                            when others => m_a <= to_unsigned(C_KVG * 8, 20);
                        end case;
                    when 2 => m_a <= resize(f_kv, 20);            m_b <= resize(t_m.vx, 18);
                    when 3 => m_a <= resize(f_kv, 20);            m_b <= resize(t_m.xm, 18);
                    when 4 => m_a <= resize(f_sm, 20);            m_b <= to_unsigned(554, 18);
                    when 5 => m_a <= resize(f_kv, 20);            m_b <= to_unsigned(960, 18);
                    when 6 => m_a <= resize(f_kv, 20);            m_b <= resize(f_ramp, 18);
                    when others => m_a <= resize(f_kv, 20);       m_b <= to_unsigned(42, 18);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(17 downto 1);
                m_b <= '0' & m_b(17 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1) = '1' then f_kv <= v_pr(30 downto 12); end if;                     -- C_KVG 2^ci cm / 4096
            if q_wr(2) = '1' then f_kph <= v_pr(27 downto 8); end if;                     -- per pixel
            if q_wr(3) = '1' then f_ph0 <= to_unsigned(C_PHC, 24) + f_drift(23 downto 0) - v_pr(23 downto 0); end if;
            if q_wr(5) = '1' then
                v_p0 := to_unsigned(C_PHC, 29) + f_drift - v_pr(28 downto 0); f_p0 <= v_p0(28 downto 8);
            end if;
            if q_wr(7) = '1' then f_ks <= v_pr(24 downto 8); end if;                       -- 42 kv
            if q_wr(4) = '1' then f_dstep <= v_pr(18 downto 0); end if;
            if q_wr(6) = '1' then f_ca <= v_pr(25 downto 14); end if;                       -- comb gain, ramp kv / 2^14

            -- Rows remap writer: entry i = t' of t = 2i, by forward differences (G += d, d -= dd); the RAM is written
            -- with 2i + G (rm_wd, combinational) while rw_on
            if s_zone = '1' and s_zc = 2 then
                rw_on <= '1'; rw_i <= (others => '0'); rw_g <= (others => '0'); rw_d <= shift_left(resize(f_rc, 20), 7);   -- c * 128
            elsif rw_on = '1' then
                rw_g <= rw_g + resize(rw_d, 25);
                rw_d <= rw_d - resize(shift_right(f_rc, 1) - shift_right(f_rc, 8), 20);         -- c * 0.496
                rw_i <= rw_i + 1;
                if rw_i = 1023 then rw_on <= '0'; end if;
            end if;

            -- Drip knot table builder: from zone clock 256, 32 clocks per warp knot (48): the knot's drop phase
            -- and index; window w (triangle peaking at the drop centre, phase 0.73) times the drop's length
            -- (0.25 .. 1, its own breath phase), times |depth|, on one serial shift-add, signed
            if s_zone = '1' and s_zc = 256 then
                kb_on <= '1'; kb_cnt <= (others => '0'); kb_ph <= f_p0;
            elsif kb_on = '1' then
                kb_cnt <= kb_cnt + 1;
                if kb_cnt = 1535 then kb_on <= '0'; end if;
            end if;
            kt_we <= '0';
            if kb_on = '1' then
                case to_integer(kb_cnt(4 downto 0)) is
                    when 0 =>
                        v_h := unsigned(C_LV_DRIP(to_integer(kb_ph(20 downto 16)))) + f_td(9 downto 2);
                        kb_md <= resize(64 + shift_right(f_tri8(v_h), 1) + shift_right(f_tri8(v_h), 2), 9);
                        kb_mr <= f_tri8(kb_ph(15 downto 8) - 59);
                        kb_acc <= (others => '0');
                    when 1 to 8 | 10 to 17 =>
                        if kb_mr(0) = '1' then
                            kb_acc <= shift_right(kb_acc + shift_left(resize(kb_md, 18), 8), 1);
                        else
                            kb_acc <= shift_right(kb_acc, 1);
                        end if;
                        kb_mr <= shift_right(kb_mr, 1);
                    when 9 =>
                        kb_mr <= kb_acc(15 downto 8);
                        if f_dd < 0 then kb_md <= unsigned(resize(-f_dd, 9)); else kb_md <= unsigned(f_dd(8 downto 0)); end if;
                        kb_acc <= (others => '0');
                    when 18 =>
                        kt_we <= '1'; kt_wa <= "0" & f_kb & kb_cnt(10 downto 5);
                        if f_dd < 0 then kt_wd <= std_logic_vector(-signed(resize(kb_acc(16 downto 8), 10)));
                        else kt_wd <= std_logic_vector(signed(resize(kb_acc(16 downto 8), 10))); end if;
                        kb_ph <= kb_ph + resize(f_ks, 21);
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
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
    -- Per-line engine: at the source avid rise of line n, compute line n+1:
    -- its flipped virtual line, the warp knot row and fraction, the comb's
    -- phase offset, and the 48 knot values t = ye + 8 (a + (b - a) fy) of the
    -- line into the bank line n+1 will read.  Line 0 is computed in the zone.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_ye : unsigned(18 downto 0);
        variable v_jf : unsigned(21 downto 0);
        variable v_w  : signed(17 downto 0);
        variable v_a  : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- kick
            if s_zone = '1' and s_zc = 300 then
                e_yv <= resize(t_v.vy(10 downto 1), 19);
                e_jf <= resize(t_v.jf0, 22);
                e_comb <= f_comb0;
                e_wb <= '0'; e_st <= to_unsigned(1, 5);
            elsif data_in.avid = '1' and s_avq = '0' then
                r_bank <= e_wb; e_wb <= not e_wb;
                l_ye <= d_ye; l_lrp <= d_lrp; l_af <= d_af;
                e_yv <= e_yv + resize(t_v.vy, 19);
                e_jf <= e_jf + resize(t_v.jst, 22);
                e_comb <= e_comb + t_v.comb;
                e_st <= to_unsigned(1, 5);
            elsif e_st /= 0 then
                if e_st = 29 then e_st <= (others => '0'); else e_st <= e_st + 1; end if;
            end if;

            -- setup; one serial shift-add multiplier: (ye - 80) rows (states 3..14), tri(comb) gain (16..27)
            case to_integer(e_st) is
                when 1 =>
                    if f_flip = '1' then
                        v_ye := to_unsigned(276480, 19) - e_yv;
                        v_jf := to_unsigned(2621440, 22) - e_jf;
                    else
                        v_ye := e_yv; v_jf := e_jf;
                    end if;
                    e_ye <= v_ye(18 downto 8);
                    if v_jf(21 downto 16) > 40 then e_j <= to_unsigned(40, 6); else e_j <= v_jf(21 downto 16); end if;
                    e_fy <= v_jf(15 downto 8);
                    e_ct <= f_tri(e_comb);
                when 2 =>
                    e_base <= shift_left(resize(e_j, 11), 5) + shift_left(resize(e_j, 11), 4);   -- j * 48
                    em_a <= resize(signed(resize(e_ye, 12)) - 80, 24);
                    em_b <= to_unsigned(128, 12);                    -- rows fixed at 1.0 (Rows is the remap now)
                    em_acc <= (others => '0');
                    e_iss <= '1'; e_i <= (others => '0'); e_r <= '0';
                when 3 to 14 | 16 to 27 =>
                    if em_b(0) = '1' then em_acc <= em_acc + em_a; end if;
                    em_a <= shift_left(em_a, 1);
                    em_b <= '0' & em_b(11 downto 1);
                when 15 =>
                    v_a := to_signed(80, 20) + resize(shift_right(em_acc, 7), 20) - resize(f_flow, 20);
                    e_af <= resize(v_a, 14);
                    em_a <= resize(e_ct, 24);
                    em_b <= f_ca;
                    em_acc <= (others => '0');
                when 28 =>
                    e_lrp <= em_acc(20 downto 9);                    -- phase / 4096 (a slice: the phase wraps)
                when 29 =>
                    d_ye <= e_ye; d_lrp <= e_lrp; d_af <= e_af;
                when others => null;
            end case;

            -- issue: 48 knots x 2 rows (a = row j, b = row j + 1)
            e_tv(0) <= e_iss; e_tr(0) <= e_r; e_ti(0) <= e_i;
            if e_iss = '1' then
                if e_r = '0' then r_wa <= e_base + resize(e_i, 11);
                else r_wa <= e_base + 48 + resize(e_i, 11); end if;
                kt_ra <= "0" & (not f_kb) & e_i;                     -- the knot's Drip offset (last field's bank)
                if e_r = '1' then
                    if e_i = 47 then e_iss <= '0'; else e_i <= e_i + 1; end if;
                end if;
                e_r <= not e_r;
            end if;
            -- tags: 0 = address, 1 = ROM out, 2 = fabric copy
            for k in 1 to 3 loop
                e_tv(k) <= e_tv(k - 1); e_tr(k) <= e_tr(k - 1); e_ti(k) <= e_ti(k - 1);
            end loop;
            wr_q2 <= wr_q;
            e_kv <= '0';
            if e_tv(2) = '1' then
                if e_tr(2) = '0' then
                    e_ka <= signed(wr_q2); e_ko <= signed(kt_q2);
                else
                    e_dab <= resize(signed(wr_q2), 9) - resize(e_ka, 9);
                    e_ka2 <= e_ka; e_kv <= '1'; e_ki <= e_ti(2);
                end if;
            end if;
            e_kv2 <= e_kv; e_ki2 <= e_ki; e_ko2 <= e_ko;
            e_kv3 <= e_kv2; e_ki3 <= e_ki2; e_ko3 <= e_ko2;
            e_kv4 <= e_kv3; e_ki4 <= e_ki3;
            e_pr <= resize(e_dab * signed('0' & e_fy), 18) + shift_left(resize(e_ka2, 18), 8);
            -- the drape: Bend x the knot's warp w = (a 256 + (b - a) fy) / 32 lines (per knot, so the pixel path
            -- just interpolates; exact at the photo's Bend 1.0), split 6 | 4; then + ye + the Drip offset
            e_bh <= resize(shift_right(e_pr, 5) * f_bend(9 downto 4), 19);
            e_bl <= resize(shift_right(e_pr, 5) * signed('0' & f_bend(3 downto 0)), 18);
            e_bs <= shift_left(resize(e_bh, 23), 4) + resize(e_bl, 23);
            e_yo <= signed(resize(e_ye, 13)) + resize(e_ko3, 13);
            -- t = ye + Bend w + Drip -> the line table
            lt_we_e <= '0'; lt_we_o <= '0';
            if e_kv4 = '1' then
                v_w := resize(e_yo, 18) + resize(shift_right(e_bs, 7), 18);
                lt_wd <= std_logic_vector(resize(v_w, 16));
                lt_wa <= "00" & e_wb & e_ki4(5 downto 1);
                if e_ki4(0) = '0' then lt_we_e <= '1'; else lt_we_o <= '1'; end if;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Beam: the warp cell coordinate (Mirror reverses it at the virtual centre)
    -- and the oscillator phase, run as a late copy of the beam (started by
    -- g_start delayed 13 clocks) so the phase needs no delay line.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                l_pb <= f_tb;
            elsif data_in.avid = '1' and s_avq = '0' then
                l_pb <= l_pb + 121;
            end if;
            if g_start = '1' then
                b_u <= resize(t_m.u0, 22);
                b_cnt <= (others => '0');
                b_dir <= '0';
                b_pa <= f_ta;
            elsif g_avid = '1' then
                b_cnt <= b_cnt + 1;
                b_pa <= b_pa + 89;
                if b_hit = '1' then
                    b_dir <= '1';                                    -- hold: the centre pixel repeats
                elsif b_dir = '0' then
                    b_u <= b_u + resize(t_m.ku, 22);
                else
                    b_u <= b_u - resize(t_m.ku, 22);
                end if;
            end if;
            -- the turn pixel (cmid - 1) is held; its flag is set while the counter passes cmid - 2
            if f_mir = '1' and g_avid = '1' and b_cnt = f_cmid2 then b_hit <= '1'; else b_hit <= '0'; end if;
            if f_mir = '1' and s_gav_sr(14) = '1' and lp_cnt = f_cmid2 then lp_hit <= '1'; else lp_hit <= '0'; end if;
            gs_sr(0) <= g_start;
            for k in 1 to 14 loop gs_sr(k) <= gs_sr(k - 1); end loop;
            -- late copy (stage 15): the drops' phase
            if gs_sr(14) = '1' then
                lp_ph <= f_ph0;
                lp_cnt <= (others => '0');
                lp_dir <= '0';
            elsif s_gav_sr(14) = '1' then
                lp_cnt <= lp_cnt + 1;
                if lp_hit = '1' then
                    lp_dir <= '1';
                elsif lp_dir = '0' then
                    lp_ph <= lp_ph + resize(f_kph, 24);
                else
                    lp_ph <= lp_ph - resize(f_kph, 24);
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Rings (block RAM): the chroma box window {U, V} and the sync delay
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rb_ram(to_integer(s_wp)) <= rb_wd;
            rb_q <= rb_ram(to_integer(rb_ra));
            rb_q2 <= rb_q;
        end if;
    end process p_ring;

    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            rs_ram(to_integer(s_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            rs_q <= rs_ram(to_integer(rs_ra));
            rs_q2 <= rs_q;
        end if;
    end process p_sring;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ci  : unsigned(5 downto 0);
        variable v_t   : signed(13 downto 0);
        variable v_t3  : signed(15 downto 0);
        variable v_tm  : signed(15 downto 0);
        variable v_q   : unsigned(11 downto 0);
        variable v_jc  : unsigned(7 downto 0);
        variable v_iy0, v_iy1, v_iye, v_iyo : unsigned(4 downto 0);
        variable v_ic0, v_ic1, v_ice, v_ico : unsigned(3 downto 0);
        variable v_jye, v_jyo : unsigned(7 downto 0);
        variable v_ya, v_yb, v_yc, v_yd : signed(9 downto 0);
        variable v_c00, v_c10, v_c01, v_c11 : signed(9 downto 0);
        variable v_ca, v_cb : std_logic_vector(15 downto 0);
        variable v_o   : signed(10 downto 0);
        variable v_os  : signed(10 downto 0);
        variable v_iu, v_iv, v_iy : unsigned(9 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_tr  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: 2-px luma average -> signed (Y - 512) / 2 (a few px early: invisible in a bend)
            s_vy(1) <= unsigned(data_in.y(9 downto 2)); s_vy(2) <= s_vy(1);
            s_vs <= signed(resize(resize(s_vy(1), 9) + resize(s_vy(2), 9), 10)) - 256;
            -- camera ring input: the camera luma, or in Osc mode the Osc waves as clouds
            if f_osc = '1' then
                rv_wd <= std_logic_vector(unsigned(resize(shift_right(f_tri(b_pa) + f_tri(l_pb), 2), 8)) + 128);
            else
                rv_wd <= data_in.y(9 downto 2);
            end if;
            rv_ra <= s_wp - 28;

            -- 1: warp cell, fraction, line-table addresses (knot i in bank i & 1)
            if b_u(21 downto 16) > 46 then v_ci := to_unsigned(46, 6); q1_fx <= (others => '1');
            else v_ci := b_u(21 downto 16); q1_fx <= b_u(15 downto 8); end if;
            q1_odd <= v_ci(0);
            lt_rao <= "00" & r_bank & v_ci(5 downto 1);
            lt_rae <= "00" & r_bank & resize(shift_right(v_ci + 1, 1), 5);
            -- 2: line table (block RAM)
            q2_fx <= q1_fx; q2_odd <= q1_odd;
            -- 3: fabric copies
            q3_e <= signed(lt_qe(12 downto 0)); q3_o <= signed(lt_qo(12 downto 0));
            q3_fx <= q2_fx; q3_odd <= q2_odd;
            -- 4: knots i, i + 1
            if q3_odd = '1' then q4_w0 <= q3_o; q4_w1 <= q3_e;
            else q4_w0 <= q3_e; q4_w1 <= q3_o; end if;
            q4_fx <= q3_fx;
            -- 5
            q5_dw <= resize(q4_w1, 14) - resize(q4_w0, 14);
            q5_w0 <= q4_w0; q5_fx <= q4_fx;
            -- 6: (w1 - w0) fx, split
            q6_ph <= resize(q5_dw * signed('0' & q5_fx(7 downto 4)), 18);
            q6_pl <= resize(q5_dw * signed('0' & q5_fx(3 downto 0)), 18);
            q6_w0 <= q5_w0;
            -- 7
            q7_pr <= shift_left(resize(q6_ph, 22), 4) + resize(q6_pl, 22);
            q7_w0 <= q6_w0;
            -- 8: t (the warp's row coordinate); the source (video luma or Osc)
            v_t := resize(q7_w0, 14) + resize(shift_right(q7_pr, 8), 14);
            q8_t <= v_t;
            if f_osc = '1' then
                q8_src <= shift_right(f_tri(b_pa) + f_tri(l_pb), 2);   -- a quarter of the video's swing
            else
                q8_src <= s_vs;
            end if;
            -- 9: d = t - ye; the lava flag (the comb rides on it); melt products
            q9_d <= resize(q8_t, 15) - signed(resize(l_ye, 15));
            if q8_t < 1060 then p_lf(9) <= '1'; else p_lf(9) <= '0'; end if;
            q9_ms <= resize(q8_src * signed('0' & f_melt), 17);
            -- 10, 11: d copies (Bend is applied per knot in the line engine)
            q10_d <= q9_d;
            q10_ms <= resize(q9_ms, 19);
            -- 11
            q11_d <= q10_d;
            q11_ms <= resize(shift_right(q10_ms, 5), 19);
            -- 12: t3 = ye - flow + d + melt src
            v_t3 := resize(l_af, 16) + resize(q11_d, 16) + resize(q11_ms, 16);
            q12_t3 <= v_t3;
            -- 13: clamp to the tile; the background flag
            v_tm := q12_t3 - 80;
            if v_tm < 0 then q13_tm <= (others => '0'); rm_ra <= (others => '0');
            elsif v_tm > 1034 then q13_tm <= to_unsigned(1034, 11); rm_ra <= to_unsigned(517, 10);
            else q13_tm <= unsigned(v_tm(10 downto 0)); rm_ra <= unsigned(v_tm(10 downto 1)); end if;
            if q12_t3 > 1076 then p_bg(13) <= '1'; else p_bg(13) <= '0'; end if;
            -- 14: Rows remap RAM (block RAM, t' of t >> 1); 15: fabric copy
            q14_tm <= q13_tm;
            rm_q2 <= rm_q; q15_tm <= q14_tm;
            -- 14: tile rows (Y: 40 / 256 knots per line; C: 20 / 256, rounded); phase (+ comb on the lava)
            -- 16: the remapped row (Rows at its centre and the ground take t as it is); tile rows; phase (+ comb
            -- on the lava)
            if f_rc0 = '1' or p_bg(15) = '1' then v_tr := q15_tm; else v_tr := unsigned(rm_q2); end if;
            q16_vy <= shift_left(resize(v_tr, 16), 5) + shift_left(resize(v_tr, 16), 3);
            q16_vc <= shift_left(resize(v_tr, 16), 4) + shift_left(resize(v_tr, 16), 2) + 128;
            if p_lf(15) = '1' then q16_q <= lp_ph(23 downto 12) + unsigned(l_lrp);
            else q16_q <= lp_ph(23 downto 12); end if;
            -- 15: phase * 24, * 12
            v_q := q16_q;
            q17_uy <= shift_left(resize(v_q, 17), 4) + shift_left(resize(v_q, 17), 3);
            q17_uc <= shift_left(resize(v_q, 17), 3) + shift_left(resize(v_q, 17), 2);
            q17_jy <= q16_vy(15 downto 8); q17_fvy <= q16_vy(7 downto 4);
            q17_jc <= q16_vc(15 downto 8);
            -- 16: knot indices
            v_iy0 := q17_uy(16 downto 12);
            if v_iy0 = 23 then v_iy1 := (others => '0'); else v_iy1 := v_iy0 + 1; end if;
            v_ic0 := q17_uc(15 downto 12);
            if v_ic0 = 11 then v_ic1 := (others => '0'); else v_ic1 := v_ic0 + 1; end if;
            q18_iy0 <= v_iy0; q18_iy1 <= v_iy1; q18_ic0 <= v_ic0; q18_ic1 <= v_ic1;
            q18_jy0 <= q17_jy; q18_jy1 <= q17_jy + 1;
            if q17_jc > 81 then v_jc := to_unsigned(81, 8); else v_jc := q17_jc; end if;
            q18_jc <= v_jc(6 downto 0);
            p_fuy(18) <= q17_uy(11 downto 8); p_fuc(18) <= q17_uc(11 downto 8); p_fvy(18) <= q17_fvy;
            p_py(18) <= v_iy0(0) & q17_jy(0);
            p_pc(18) <= v_ic0(0);
            -- 17: bank addresses (Y: bank (i & 1) + 2 (j & 1), (j >> 1) * 12 + (i >> 1); C: bank i & 1, j * 6 + (i >> 1))
            if q18_iy0(0) = '0' then v_iye := q18_iy0; v_iyo := q18_iy1; else v_iye := q18_iy1; v_iyo := q18_iy0; end if;
            if q18_jy0(0) = '0' then v_jye := q18_jy0; v_jyo := q18_jy1; else v_jye := q18_jy1; v_jyo := q18_jy0; end if;
            if q18_ic0(0) = '0' then v_ice := q18_ic0; v_ico := q18_ic1; else v_ice := q18_ic1; v_ico := q18_ic0; end if;
            ry_a0 <= shift_left(resize(v_jye(7 downto 1), 10), 3) + shift_left(resize(v_jye(7 downto 1), 10), 2) + resize(v_iye(4 downto 1), 10);
            ry_a1 <= shift_left(resize(v_jye(7 downto 1), 10), 3) + shift_left(resize(v_jye(7 downto 1), 10), 2) + resize(v_iyo(4 downto 1), 10);
            ry_a2 <= shift_left(resize(v_jyo(7 downto 1), 10), 3) + shift_left(resize(v_jyo(7 downto 1), 10), 2) + resize(v_iye(4 downto 1), 10);
            ry_a3 <= shift_left(resize(v_jyo(7 downto 1), 10), 3) + shift_left(resize(v_jyo(7 downto 1), 10), 2) + resize(v_iyo(4 downto 1), 10);
            rc_a0 <= shift_left(resize(q18_jc, 9), 2) + shift_left(resize(q18_jc, 9), 1) + resize(v_ice(3 downto 1), 9);
            rc_a1 <= shift_left(resize(q18_jc, 9), 2) + shift_left(resize(q18_jc, 9), 1) + resize(v_ico(3 downto 1), 9);
            -- 18: tile ROMs (block RAM); 19: fabric copies
            q21_y0 <= ry_q0; q21_y1 <= ry_q1; q21_y2 <= ry_q2; q21_y3 <= ry_q3;
            q21_c0 <= rc_q0; q21_c1 <= rc_q1;
            -- 20: corners in (i, j) order, differences along the phase
            v_ya := signed(resize(unsigned(q21_y0), 10)); v_yb := signed(resize(unsigned(q21_y1), 10));
            v_yc := signed(resize(unsigned(q21_y2), 10)); v_yd := signed(resize(unsigned(q21_y3), 10));
            case p_py(21) is   -- (i0 odd, j0 odd)
                when "00" => v_c00 := v_ya; v_c10 := v_yb; v_c01 := v_yc; v_c11 := v_yd;
                when "10" => v_c00 := v_yb; v_c10 := v_ya; v_c01 := v_yd; v_c11 := v_yc;
                when "01" => v_c00 := v_yc; v_c10 := v_yd; v_c01 := v_ya; v_c11 := v_yb;
                when others => v_c00 := v_yd; v_c10 := v_yc; v_c01 := v_yb; v_c11 := v_ya;
            end case;
            q22_c00 <= v_c00; q22_c01 <= v_c01;
            q22_d0 <= v_c10 - v_c00; q22_d1 <= v_c11 - v_c01;
            if p_pc(21) = '0' then v_ca := q21_c0; v_cb := q21_c1; else v_ca := q21_c1; v_cb := q21_c0; end if;
            q22_r0 <= resize(signed(v_ca(15 downto 8)), 9); q22_b0 <= resize(signed(v_ca(7 downto 0)), 9);
            q22_rd <= resize(signed(v_cb(15 downto 8)), 10) - resize(signed(v_ca(15 downto 8)), 10);
            q22_bd <= resize(signed(v_cb(7 downto 0)), 10) - resize(signed(v_ca(7 downto 0)), 10);
            -- 21: products along the phase
            q23_p0 <= resize(q22_d0 * signed('0' & p_fuy(22)), 14);
            q23_p1 <= resize(q22_d1 * signed('0' & p_fuy(22)), 14);
            q23_pr <= resize(q22_rd * signed('0' & p_fuc(22)), 14);
            q23_pb <= resize(q22_bd * signed('0' & p_fuc(22)), 14);
            q23_c00 <= q22_c00; q23_c01 <= q22_c01; q23_r0 <= q22_r0; q23_b0 <= q22_b0;
            -- 22
            q24_a <= q23_c00 + resize(shift_right(q23_p0, 4), 10);
            q24_b <= q23_c01 + resize(shift_right(q23_p1, 4), 10);
            q24_cr <= resize(q23_r0, 10) + resize(shift_right(q23_pr, 4), 10);
            q24_cb <= resize(q23_b0, 10) + resize(shift_right(q23_pb, 4), 10);
            -- 23
            q25_e <= q24_b - q24_a; q25_a <= q24_a; q25_cr <= q24_cr; q25_cb <= q24_cb;
            -- 24: product along t
            q26_pe <= resize(q25_e * signed('0' & p_fvy(25)), 14);
            q26_a <= q25_a; q26_cr <= q25_cr; q26_cb <= q25_cb;
            -- 25
            q27_y <= q26_a + resize(shift_right(q26_pe, 4), 10);
            q27_cr <= q26_cr; q27_cb <= q26_cb;
            -- 26: compose (Y = 64 + 4 y; U = Cr, V = Cb at 512 + 4 c); Ice; Ground = Black
            v_o := shift_left(resize(q27_cr, 11), 2);
            v_os := to_signed(512, 11) + v_o;
            v_iu := unsigned(v_os(9 downto 0));
            v_o := shift_left(resize(q27_cb, 11), 2);
            v_os := to_signed(512, 11) + v_o;
            v_iv := unsigned(v_os(9 downto 0));
            if f_cam = '1' and p_bg(27) = '1' then           -- Ground = Camera: the picture's luma in violet
                q28_y <= unsigned(rv_q2) & "00";
                q28_u <= to_unsigned(624, 10); q28_v <= to_unsigned(668, 10);
            else
                if q27_y < 0 then q28_y <= to_unsigned(64, 10);
                else q28_y <= to_unsigned(64, 10) + shift_left(resize(unsigned(q27_y(7 downto 0)), 10), 2); end if;
                if f_ice = '1' then q28_u <= v_iv; q28_v <= v_iu;
                else q28_u <= v_iu; q28_v <= v_iv; end if;
            end if;

            -- delay lines
            for k in 10 to 15 loop p_lf(k) <= p_lf(k - 1); end loop;
            for k in 14 to 27 loop p_bg(k) <= p_bg(k - 1); end loop;
            for k in 19 to 22 loop p_fuy(k) <= p_fuy(k - 1); p_fuc(k) <= p_fuc(k - 1); end loop;
            for k in 19 to 25 loop p_fvy(k) <= p_fvy(k - 1); end loop;
            for k in 19 to 21 loop p_py(k) <= p_py(k - 1); p_pc(k) <= p_pc(k - 1); end loop;

            -- 29..: chroma box sums over the ring window: 8 taps at 1080, 4 at 720p / SD (box pair =
            -- [1 2 2 2 2 2 2 2 1] / 16).  The ring's input is forced neutral in the vblank zone and outside
            -- the generated avid; luma rides a register chain aligned with the chroma window's centre.
            s_wp <= s_wp + 1;
            if t_short = '1' then rb_ra <= s_wp - 1; else rb_ra <= s_wp - 5; end if;
            rs_ra <= s_wp - (C_LAT - 3);
            if s_zone = '1' or s_gav_sr(27) = '0' then
                v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10); v_iy := to_unsigned(64, 10);
            else
                v_iu := q28_u; v_iv := q28_v; v_iy := q28_y;
            end if;
            rb_wd <= std_logic_vector(v_iu) & std_logic_vector(v_iv);
            s_bxh <= s_bxh(6 downto 0) & (s_gav_sr(27) and not s_zone);   -- the same gate as rb_wd
            s_ly(0) <= v_iy;
            for k in 1 to 6 loop s_ly(k) <= s_ly(k - 1); end loop;
            -- the sums snap back to neutral once 8 blank samples are in (every blanking interval, and the
            -- vblank zone), so a sample the ring returns wrong can't offset the chroma for good
            if s_bxh = "00000000" then
                if t_short = '1' then s_usum <= to_unsigned(2048, 14); s_vsum <= to_unsigned(2048, 14);
                else s_usum <= to_unsigned(4096, 14); s_vsum <= to_unsigned(4096, 14); end if;
            else
                s_usum <= s_usum + unsigned(rb_wd(19 downto 10)) - unsigned(rb_q2(19 downto 10));
                s_vsum <= s_vsum + unsigned(rb_wd(9 downto 0)) - unsigned(rb_q2(9 downto 0));
            end if;
            -- 31: chroma box pair; luma [1 2 1] centred on the same pixel
            s_ud <= s_usum; s_vd <= s_vsum;
            v_sc := resize(s_usum, 15) + s_ud;
            if t_short = '1' then q31_u <= v_sc(12 downto 3); else q31_u <= v_sc(13 downto 4); end if;
            v_sc := resize(s_vsum, 15) + s_vd;
            if t_short = '1' then q31_v <= v_sc(12 downto 3); else q31_v <= v_sc(13 downto 4); end if;
            if t_short = '1' then
                v_sy := resize(s_ly(2), 12) + shift_left(resize(s_ly(3), 12), 1) + resize(s_ly(4), 12);
            else
                v_sy := resize(s_ly(4), 12) + shift_left(resize(s_ly(5), 12), 1) + resize(s_ly(6), 12);
            end if;
            q31_y <= v_sy(11 downto 2);
            -- 32: luma toward black (white 940 -> 777); blanking gate
            if q31_y > 64 then v_dl := q31_y - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(31) = '1' then
                q32_y <= q31_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                q32_u <= q31_u; q32_v <= q31_v;
            else
                q32_y <= to_unsigned(64, 10); q32_u <= to_unsigned(512, 10); q32_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(q32_y);
    data_out.u       <= std_logic_vector(q32_u);
    data_out.v       <= std_logic_vector(q32_v);
    data_out.hsync_n <= rs_q2(2);
    data_out.vsync_n <= rs_q2(1);
    data_out.field_n <= rs_q2(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lava;
