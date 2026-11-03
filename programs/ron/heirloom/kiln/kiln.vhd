-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Kiln v0.6.1 (phase 2 final) -- Heirloom #32, rebuilt from the photo WIN_20230515_23_17_11.
--
-- The photo: camera video in a frame store zoomed to a quadrant (992 px x 540
-- rows), repeated across the frame (period 992 px = 1.94 copies) with the
-- bottom row of quadrants mirrored (axis at mid-frame), run through a fire
-- colouriser -- royal blue, violet, purple, magenta, red, orange, yellow --
-- with a brightness lift, and torn into horizontal fibres after the store:
-- every SD line (2.25 HD rows) carries its own noise, partly a sideways
-- displacement of the picture, partly an offset into the colouriser, with an
-- interlace comb (alternate SD lines pulled apart) strongest beside the yellow
-- blocks ("venetian blinds").
--
-- The program: a 128 x 64 cell store of one quadrant (Osc: a ROM fitted to the
-- photo -- the colouriser scalar s (4 bits), the brightness lift b (3 bits) and
-- a comb-map bit; Video: the camera's luma, written live), read back with the
-- tile wrap and the top/bottom mirror through a bilinear interpolation (four
-- bank reads per pixel), plus per-SD-line hashed fibre noise (a 32-vpx line
-- layer and a 64 x 32 smooth layer, scaled by Fibres at the knots) as a
-- displacement and a colouriser offset, into a 128-entry fire palette (photo
-- ramp, yellow headroom, white-hot) with a luma lift.
--
--   K1  Tiles    copies across 0.5 .. 6.3, exponential (1.94 = the photo, at 551)
--   K2  Fibres   fibre colour noise 0 .. 2x (16 steps) and the sideways tear
--                0 .. 3.9x (square law above the photo's 1x at 512)
--   K3  Blocks   colouriser contrast 0.25 .. 4x, exponential: the yellow blocks
--                and blue patches swell, or the picture melts into the red mids
--   K4  Drift    bipolar tile scroll, still at the centre (+-8), square law
--   K5  Rims     the brightness lift (pink rims, ring edges) -1x .. 2.9x; Video:
--                rims traced on the camera's edges (an edge map of the store)
--   K6  Glaze    colourways: Fire / Ice / Ember / Toxic / Violet / Mono / Neon / Rust
--   S7  Mirror   Mirror / Repeat (bottom row of quadrants reflected or repeated)
--   S8  Kaleido  Off / On: every other copy flipped left/right
--   S9  Strips   Off / On: 8 bands per half (mirrored) slid by their own quarter-copy
--                offsets; half of them stand while the rest scroll with Drift
--   S10 Tile Lock Free / Lock: K1 snaps to whole copies across (1 .. 6, the nearest),
--                the X scale gliding into each new count, so the tiles always fit evenly
--   S11 Source   Video / Osc
--   P12 HEAT     colouriser offset: 0 = cold (blue, red embers) .. 50 % = the
--                photo .. 100 % = white-hot
-- Motion: the fibres rise one SD line every two fields, the glow breathes
-- (+-3 palette steps over 512 fields), the tiles drift with K4.
--
-- Generated avid; output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16,
-- luma to ~777) before the blanking gate.  Latency 32 clocks in every mode.
-- Colours in the hardware convention (U = Cr, V = Cb).  The integer model
-- (.heirloom_work/rb_kiln/imodel.py) is the spec; the RTL renders bit-exact.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.kiln_pkg.all;
use work.all;

architecture kiln of program_top is
    constant C_LAT : integer := 32;   -- matches processing_delay_clks
    constant C_SDL : std_logic_vector(15 downto 0) := x"5A3C";   -- line-noise seed
    constant C_SD2 : std_logic_vector(15 downto 0) := x"9E37";   -- smooth-layer seed

    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;

    -- hash rounds (khash.py)
    function f_ha(v : unsigned(15 downto 0)) return unsigned is
        variable a : unsigned(15 downto 0);
    begin
        a := v + shift_left(v, 5);
        return a xor shift_right(a, 7);
    end function;
    function f_hb(v : unsigned(15 downto 0)) return unsigned is
        variable a : unsigned(15 downto 0);
    begin
        a := (v + shift_left(v, 3)) xor x"3C6B";
        return a xor shift_right(a, 9);
    end function;
    function f_hc(v : unsigned(15 downto 0)) return unsigned is
        variable a : unsigned(15 downto 0);
    begin
        a := v + shift_left(v, 11);
        return a xor shift_right(a, 6);
    end function;
    -- s level table (palette steps)
    function f_ls(c : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(c)) is
            when 0 => return to_unsigned(0, 7);   when 1 => return to_unsigned(6, 7);
            when 2 => return to_unsigned(12, 7);  when 3 => return to_unsigned(18, 7);
            when 4 => return to_unsigned(24, 7);  when 5 => return to_unsigned(30, 7);
            when 6 => return to_unsigned(36, 7);  when 7 => return to_unsigned(42, 7);
            when 8 => return to_unsigned(48, 7);  when 9 => return to_unsigned(54, 7);
            when 10 => return to_unsigned(60, 7); when 11 => return to_unsigned(66, 7);
            when 12 => return to_unsigned(72, 7); when 13 => return to_unsigned(80, 7);
            when 14 => return to_unsigned(88, 7); when others => return to_unsigned(96, 7);
        end case;
    end function;
    -- b level table (x16)
    function f_lb(c : std_logic_vector(2 downto 0)) return signed is
    begin
        case to_integer(unsigned(c)) is
            when 0 => return to_signed(-22, 6); when 1 => return to_signed(-16, 6);
            when 2 => return to_signed(-10, 6); when 3 => return to_signed(-3, 6);
            when 4 => return to_signed(3, 6);   when 5 => return to_signed(10, 6);
            when 6 => return to_signed(16, 6);  when others => return to_signed(22, 6);
        end case;
    end function;
    -- x {0, 1, 2, 3}
    function f_m2(d : signed(6 downto 0); f : unsigned(1 downto 0)) return signed is
    begin
        case f is
            when "00" => return to_signed(0, 9);
            when "01" => return resize(d, 9);
            when "10" => return shift_left(resize(d, 9), 1);
            when others => return shift_left(resize(d, 9), 1) + resize(d, 9);
        end case;
    end function;
    -- S9 Strips: band (8 per half, mirrored store row) -> offset in quarter copies (of the 2-copy period)
    function f_soff(b : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(b) is
            when 0 => return "010"; when 1 => return "111"; when 2 => return "001"; when 3 => return "101";
            when 4 => return "110"; when 5 => return "011"; when 6 => return "100"; when others => return "001";
        end case;
    end function;
    -- '1' = the band scrolls with Drift, '0' = it stands
    function f_sscr(b : unsigned(2 downto 0)) return std_logic is
    begin
        case to_integer(b) is
            when 1 | 2 | 4 | 7 => return '1';
            when others => return '0';
        end case;
    end function;
    -- ones'-complement |a - b| of two 4-bit codes (the Video rims' edge map)
    function f_ad(a, b : std_logic_vector(3 downto 0)) return unsigned is
        variable d : unsigned(4 downto 0);
    begin
        d := resize(unsigned(a), 5) - resize(unsigned(b), 5);
        if d(4) = '1' then return not d(3 downto 0); else return d(3 downto 0); end if;
    end function;
    -- edge -> lift: min(e, 15) x 2
    function f_em(e : unsigned(4 downto 0)) return signed is
    begin
        if e(4) = '1' then return to_signed(30, 6); else return signed('0' & e(3 downto 0) & '0'); end if;
    end function;
    function bidx(rp, cp : std_logic) return integer is
    begin
        if rp = '1' then
            if cp = '1' then return 3; else return 2; end if;
        else
            if cp = '1' then return 1; else return 0; end if;
        end if;
    end function;

    -- zone / width / lines
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_lcnt, s_hl : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_wd : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_hd : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal t_hd, t_hdl : std_logic := '1';

    -- glides (Q.3): 0 K1 Tiles, 1 K2 Fibres, 2 K3 Blocks, 3 K5 Rims, 4 P12 Heat
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(551 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w, q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');
    signal s_m4 : unsigned(8 downto 0) := (others => '0');
    signal s_s4 : std_logic := '0';
    signal s_kd : unsigned(8 downto 0) := (others => '0');          -- K2 above 512
    signal s_fqd : unsigned(5 downto 0) := to_unsigned(8, 6);       -- tear factor 0 .. 39 (8 = 1x), clipped to 31
    signal s_k2hi : std_logic := '0';
    signal s_e1 : unsigned(10 downto 0) := to_unsigned(495, 11);     -- K1 exponent (Q8 octaves)

    -- serial multiplier (drift speed) and divider (6 slots)
    signal m_a, m_acc : unsigned(19 downto 0) := (others => '0');
    signal m_b : unsigned(9 downto 0) := (others => '0');
    signal d_n : unsigned(27 downto 0) := (others => '0');
    signal d_r : unsigned(12 downto 0) := (others => '0');
    signal d_v : unsigned(11 downto 0) := (others => '0');

    -- per-field values
    signal f_n256  : unsigned(10 downto 0) := to_unsigned(495, 11);
    signal f_fq    : unsigned(3 downto 0) := to_unsigned(8, 4);     -- fibre colour noise, 8 = 1x
    signal f_dm    : unsigned(4 downto 0) := to_unsigned(8, 5);     -- tear (displacement) factor 0 .. 31, 8 = 1x
    signal f_kcm   : unsigned(6 downto 0) := to_unsigned(66, 7);
    signal f_g64   : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal f_rim   : signed(5 downto 0) := to_signed(8, 6);         -- rims -8..23, 8 = 1x
    signal f_heat  : signed(9 downto 0) := (others => '0');
    signal f_sp    : signed(17 downto 0) := (others => '0');
    signal f_dr    : unsigned(23 downto 0) := (others => '0');   -- wraps at two copies (Kaleido period)
    signal f_cx0   : unsigned(23 downto 0) := to_unsigned(16777216 - 32768, 24);
    signal f_fc    : unsigned(9 downto 0) := (others => '0');
    signal f_rise  : unsigned(8 downto 0) := (others => '0');
    signal f_rv    : unsigned(27 downto 0) := (others => '0');
    signal f_br    : signed(5 downto 0) := (others => '0');
    signal f_pl    : unsigned(2 downto 0) := (others => '0');
    signal f_rep, f_kal, f_strp, f_lock, f_osc : std_logic := '0';
    -- S10 Tile Lock: the tiles x256 the divider uses, glided onto whole copies; f_nret = gliding back after Lock
    signal f_nl    : unsigned(10 downto 0) := to_unsigned(495, 11);
    signal f_nret  : std_logic := '0';
    signal nl_d    : signed(11 downto 0) := (others => '0');
    signal t_cx  : unsigned(16 downto 0) := to_unsigned(8448, 17);
    signal t_vx  : unsigned(17 downto 0) := to_unsigned(65536, 18);
    signal t_vy  : unsigned(18 downto 0) := to_unsigned(65536, 19);
    signal t_L   : unsigned(17 downto 0) := to_unsigned(29127, 18);
    signal t_ry  : unsigned(15 downto 0) := to_unsigned(7767, 16);
    signal t_wc  : unsigned(15 downto 0) := to_unsigned(4369, 16);

    -- per-line engine
    signal le_st : unsigned(3 downto 0) := (others => '0');
    signal le_dl : unsigned(4 downto 0) := (others => '0');
    signal n_vy  : unsigned(26 downto 0) := (others => '0');
    signal n_L   : unsigned(25 downto 0) := (others => '0');
    signal n_ry  : unsigned(23 downto 0) := (others => '0');
    signal n_rr  : unsigned(23 downto 0) := (others => '0');
    signal n_vyk : unsigned(27 downto 0) := (others => '0');
    signal n_Lk  : unsigned(9 downto 0) := (others => '0');
    signal n_wr, n_wrp : unsigned(5 downto 0) := (others => '0');
    signal n_ln, n_first : std_logic := '0';
    signal n_r   : unsigned(5 downto 0) := (others => '0');
    signal n_fy  : unsigned(3 downto 0) := (others => '0');
    signal l_ra0, l_ra1 : unsigned(4 downto 0) := (others => '0');
    signal l_r0p, l_sg, l_wok : std_logic := '0';
    signal l_fy, l_fy2 : unsigned(3 downto 0) := (others => '0');
    signal l_Lk  : unsigned(9 downto 0) := (others => '0');
    signal l_j0  : unsigned(6 downto 0) := (others => '0');
    signal l_j1  : unsigned(7 downto 0) := to_unsigned(1, 8);          -- l_j0 + 1
    signal l_wrow : unsigned(5 downto 0) := (others => '0');
    signal pr_st : unsigned(3 downto 0) := (others => '0');
    signal l_r63 : std_logic := '0';                                   -- store row 63: no row below
    signal l_boff : unsigned(2 downto 0) := (others => '0');          -- S9 Strips: band offset (quarter copies)
    signal l_bst  : std_logic := '0';                                  -- S9 Strips: the band stands (undrifted base)

    -- knot engine: line layer (hash -> sN, bN x Fibres), smooth layer (two rows -> y-lerped D, C x Fibres)
    signal hs_v0, hs_v1, hs_v2, hs_v3 : std_logic := '0';
    signal hs_t0, hs_t1, hs_t2, hs_t3 : unsigned(3 downto 0) := (others => '0');
    signal hs_k0, hs_k1, hs_k2, hs_k3 : unsigned(15 downto 0) := (others => '0');
    signal h2_vl, h2_vs : std_logic := '0';
    signal h2_tl, h2_ts : unsigned(2 downto 0) := (others => '0');
    signal h2_aD, h2_aC, h2_bD, h2_bC : signed(7 downto 0) := (others => '0');

    signal h2_dD, h2_dC : signed(12 downto 0) := (others => '0');
    signal h2_yD, h2_yC : signed(7 downto 0) := (others => '0');
    signal k2_rqa, k2_rqb : std_logic := '0';
    signal k2_rm : unsigned(4 downto 0) := (others => '0');
    type t_kv is array(0 to 2) of signed(9 downto 0);
    signal kS, kB : t_kv := (others => (others => '0'));
    type t_mv is array(0 to 2) of signed(11 downto 0);
    signal mC : t_mv := (others => (others => '0'));
    type t_mvd is array(0 to 2) of signed(12 downto 0);
    signal mD : t_mvd := (others => (others => '0'));

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- beam (stage 0)
    signal b_vx  : unsigned(26 downto 0) := (others => '0');
    signal b_kc, b_mc : std_logic := '0';
    signal p_gs  : t_sl(1 to 4) := (others => '0');
    signal c_cx  : unsigned(23 downto 0) := (others => '0');

    -- stores
    signal a_s0, a_s1, a_s2, a_s3 : unsigned(10 downto 0) := (others => '0');
    signal a_b0, a_b1, a_b2, a_b3 : unsigned(9 downto 0) := (others => '0');
    signal a_p : unsigned(7 downto 0) := (others => '0');
    signal r_s0, r_s1, r_s2, r_s3 : std_logic_vector(3 downto 0) := (others => '0');
    signal r_b0, r_b1, r_b2, r_b3 : std_logic_vector(3 downto 0) := (others => '0');
    signal r_v0, r_v1, r_v2, r_v3 : std_logic_vector(3 downto 0) := (others => '0');
    signal r_p0, r_p1, r_p2 : std_logic_vector(15 downto 0) := (others => '0');
    type t_vram is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal vr0, vr1, vr2, vr3 : t_vram := (others => (others => '0'));
    -- noise delay ring (stage 5 -> stage 15)
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal dr : t_ring := (others => (others => '0'));
    signal r_dr : std_logic_vector(15 downto 0) := (others => '0');
    signal rc : unsigned(7 downto 0) := (others => '0');   -- the rings' shared read address; writes at rc + their delay
    signal dr_d : std_logic_vector(15 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of vr0 : signal is true;
    attribute no_rw_check of vr1 : signal is true;
    attribute no_rw_check of vr2 : signal is true;
    attribute no_rw_check of vr3 : signal is true;
    attribute no_rw_check of dr  : signal is true;
    -- video write side
    signal w_yf : unsigned(13 downto 0) := (others => '0');
    signal w_uw : unsigned(22 downto 0) := (others => '0');
    signal w_ev, w_ev2 : std_logic := '0';
    signal w_y0 : unsigned(9 downto 0) := (others => '0');
    signal w_col, w_col2 : unsigned(6 downto 0) := (others => '0');
    signal w_cd : unsigned(14 downto 0) := (others => '0');
    signal w_we : std_logic_vector(3 downto 0) := (others => '0');
    signal w_a : unsigned(10 downto 0) := (others => '0');
    signal w_d : std_logic_vector(3 downto 0) := (others => '0');

    -- pixel path
    signal p1_fx5, p1_fxm : unsigned(4 downto 0) := (others => '0');
    signal p2_s0, p2_b0 : signed(9 downto 0) := (others => '0');
    signal p2_d0 : signed(12 downto 0) := (others => '0');
    signal p2_c0 : signed(11 downto 0) := (others => '0');
    signal p2_ps, p2_pb : signed(15 downto 0) := (others => '0');
    signal p2_pd : signed(18 downto 0) := (others => '0');
    signal p2_pc : signed(17 downto 0) := (others => '0');
    signal p3_sn, p3_bn : signed(9 downto 0) := (others => '0');
    signal p3_d : signed(12 downto 0) := (others => '0');
    signal p3_c : signed(11 downto 0) := (others => '0');
    signal p4_dc : signed(19 downto 0) := (others => '0');
    signal p4_s5 : signed(11 downto 0) := (others => '0');
    signal p4_c7 : signed(14 downto 0) := (others => '0');
    signal p4_b26 : signed(14 downto 0) := (others => '0');
    signal p5_cx : unsigned(23 downto 0) := (others => '0');
    signal p5_ns : signed(9 downto 0) := (others => '0');
    signal p5_nb : signed(5 downto 0) := (others => '0');
    signal p6_cp, p6_bp : std_logic := '0';
    signal p6_fx : unsigned(3 downto 0) := (others => '0');
    signal p6_fxb : unsigned(1 downto 0) := (others => '0');
    type t_u4a is array (natural range <>) of unsigned(3 downto 0);
    type t_u2a is array (natural range <>) of unsigned(1 downto 0);
    signal p_fx : t_u4a(7 to 11) := (others => (others => '0'));
    signal p_fxb : t_u2a(7 to 11) := (others => (others => '0'));
    signal p_cp, p_bp : t_sl(7 to 8) := (others => '0');
    signal f8_s0, f8_s1, f8_s2, f8_s3 : std_logic_vector(3 downto 0) := (others => '0');
    signal f8_b0, f8_b1, f8_b2, f8_b3 : std_logic_vector(3 downto 0) := (others => '0');
    signal p9_q00, p9_q01, p9_q10, p9_q11 : unsigned(6 downto 0) := (others => '0');
    signal p9_b00, p9_b01, p9_b10, p9_b11 : signed(5 downto 0) := (others => '0');
    signal p_cm : t_sl(9 to 15) := (others => '0');
    signal p10_q00, p10_q01 : unsigned(6 downto 0) := (others => '0');
    signal p10_ps0, p10_ps1 : signed(11 downto 0) := (others => '0');
    signal p10_b00, p10_b01 : signed(5 downto 0) := (others => '0');
    signal p10_pb0, p10_pb1 : signed(8 downto 0) := (others => '0');
    signal p11_sa, p11_sb : signed(11 downto 0) := (others => '0');
    signal p11_ba, p11_bb : signed(8 downto 0) := (others => '0');
    signal p12_sa : signed(11 downto 0) := (others => '0');
    signal p12_ps : signed(16 downto 0) := (others => '0');
    signal p12_ba : signed(10 downto 0) := (others => '0');
    signal p12_pb : signed(10 downto 0) := (others => '0');
    signal p13_s : signed(11 downto 0) := (others => '0');
    signal p13_b : signed(6 downto 0) := (others => '0');
    signal p14_gp : signed(19 downto 0) := (others => '0');
    signal p14_bp : signed(12 downto 0) := (others => '0');
    signal p15_s : signed(12 downto 0) := (others => '0');
    signal p15_b : signed(9 downto 0) := (others => '0');
    signal f15_dr : std_logic_vector(15 downto 0) := (others => '0');
    signal p16_s : signed(13 downto 0) := (others => '0');
    signal p16_b : signed(9 downto 0) := (others => '0');
    type t_s6a is array (natural range <>) of signed(5 downto 0);
    signal p_b6 : t_s6a(17 to 19) := (others => (others => '0'));
    signal p19_p0, p19_p1, p19_p2 : std_logic_vector(15 downto 0) := (others => '0');
    signal p20_dy : signed(14 downto 0) := (others => '0');
    signal p20_py, p20_pu, p20_pv : unsigned(9 downto 0) := (others => '0');
    signal p21_y : signed(11 downto 0) := (others => '0');
    signal p21_u, p21_v : signed(10 downto 0) := (others => '0');
    signal p22_y, p22_u, p22_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- chroma box-filter delay (8 back) in two 256 x 16 block-RAM rings: read pointer = write - 6, RAM register, fabric register
    type t_cring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal cru, crv : t_cring := (others => (others => '0'));
    signal r_cru, r_crv : std_logic_vector(15 downto 0) := (others => '0');
    signal s_csu7, s_csv7 : unsigned(9 downto 0) := to_unsigned(512, 10);
    attribute no_rw_check of cru : signal is true;
    attribute no_rw_check of crv : signal is true;
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 box inputs
    signal s_y23, s_y24, s_y25, s_y26, s_y27, s_y28x, s_yf28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29, s_y30, s_y31, s_y32 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29, s_u30, s_v30, s_u31, s_v31, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel
    -- syncs delayed C_LAT in a 256 x 4 block-RAM ring (write at rc + 30, read rc, RAM register, fabric register)
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_sring := (others => (others => '1'));
    signal r_sr, s_so : std_logic_vector(3 downto 0) := (others => '1');
    attribute no_rw_check of sr : signal is true;

begin

    ---------------------------------------------------------------------------
    -- Scene ROMs (Osc), camera store (Video), palette ROMs, noise delay ring
    ---------------------------------------------------------------------------
    p_srom : process(clk)
    begin
        if rising_edge(clk) then
            r_s0 <= C_S0(to_integer(a_s0)); r_s1 <= C_S1(to_integer(a_s1));
            r_s2 <= C_S2(to_integer(a_s2)); r_s3 <= C_S3(to_integer(a_s3));
            r_b0 <= C_B0(to_integer(a_b0)); r_b1 <= C_B1(to_integer(a_b1));
            r_b2 <= C_B2(to_integer(a_b2)); r_b3 <= C_B3(to_integer(a_b3));
            r_p0 <= C_P0(to_integer(a_p)); r_p1 <= C_P1(to_integer(a_p)); r_p2 <= C_P2(to_integer(a_p));
        end if;
    end process p_srom;

    p_vr0 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(0) = '1' then vr0(to_integer(w_a)) <= w_d; end if;
            r_v0 <= vr0(to_integer(a_s0));
        end if;
    end process p_vr0;
    p_vr1 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(1) = '1' then vr1(to_integer(w_a)) <= w_d; end if;
            r_v1 <= vr1(to_integer(a_s1));
        end if;
    end process p_vr1;
    p_vr2 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(2) = '1' then vr2(to_integer(w_a)) <= w_d; end if;
            r_v2 <= vr2(to_integer(a_s2));
        end if;
    end process p_vr2;
    p_vr3 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(3) = '1' then vr3(to_integer(w_a)) <= w_d; end if;
            r_v3 <= vr3(to_integer(a_s3));
        end if;
    end process p_vr3;
    -- ring: written every clock with the stage-5 noise, read 8 clocks later (never the write address)
    -- chroma box-filter delay: written every clock with p22_u / p22_v at rc + 6, read at rc, RAM register,
    -- fabric register = the value of 8 clocks before (never the write address); the sync ring likewise (32)
    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            cru(to_integer(rc + 6)) <= "000000" & std_logic_vector(p22_u);
            crv(to_integer(rc + 6)) <= "000000" & std_logic_vector(p22_v);
            r_cru <= cru(to_integer(rc));
            r_crv <= crv(to_integer(rc));
            s_csu7 <= unsigned(r_cru(9 downto 0));
            s_csv7 <= unsigned(r_crv(9 downto 0));
            -- syncs: C_LAT = 30 + RAM register + fabric register
            sr(to_integer(rc + 30)) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            r_sr <= sr(to_integer(rc));
            s_so <= r_sr;
        end if;
    end process p_cring;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            dr(to_integer(rc + 8)) <= dr_d;
            r_dr <= dr(to_integer(rc));
            rc <= rc + 1;
        end if;
    end process p_ring;
    dr_d <= std_logic_vector(p5_ns) & std_logic_vector(p5_nb);

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_rm : unsigned(12 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_h  : signed(13 downto 0);
        variable v_t  : unsigned(9 downto 0);
        variable v_q  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then

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

            -- width (source avid pixels per line, max over the field) and active lines
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
                if s_lcnt /= 2047 then s_lcnt <= s_lcnt + 1; end if;
            end if;
            g_wm1 <= s_w - 1;
            if s_w < 512 then s_wd <= to_unsigned(1920, 12); else s_wd <= s_w; end if;
            if s_hl < 200 then s_hd <= to_unsigned(1080, 11); else s_hd <= s_hl; end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_hl <= s_lcnt; s_lcnt <= (others => '0');
                -- K4 Drift: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 7 then s_s4 <= '0'; v_u := unsigned(v_x) - 8; s_m4 <= v_u(8 downto 0);
                elsif v_x < -7 then s_s4 <= '1'; v_u := unsigned(-v_x) - 8; s_m4 <= v_u(8 downto 0);
                else s_s4 <= '0'; s_m4 <= (others => '0'); end if;
                -- labelled knob: 0..7 at program load is the label index
                if unsigned(registers_in(5)) < 8 then f_pl <= unsigned(registers_in(5)(2 downto 0));
                else f_pl <= unsigned(registers_in(5)(9 downto 7)); end if;
                f_rep   <= registers_in(6)(0);
                f_kal   <= registers_in(6)(1);
                f_strp  <= registers_in(6)(2);
                f_lock  <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- motion counters
                f_fc <= f_fc + 1;
                f_dr <= f_dr + unsigned(resize(f_sp, 24));

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

            -- per-field values from the glides (zc 40..43)
            if s_zone = '1' and s_zc = 40 then
                -- K2: colour noise k/64 (as v0.5); the tear = the same below 512, 8 + (k - 512)^2 / 2048 above
                v_k := s_g(1)(12 downto 3);
                f_fq <= v_k(9 downto 6);
                s_kd <= v_k(8 downto 0); s_k2hi <= v_k(9);
                -- K3: gain (256 + frac) << oct >> 4 = 16 .. 255 (0.25x .. 4x of 64), 64 at k 512
                v_k := s_g(2)(12 downto 3);
                case to_integer(v_k(9 downto 8)) is
                    when 0 => f_g64 <= resize('1' & v_k(7 downto 4), 9);
                    when 1 => f_g64 <= resize('1' & v_k(7 downto 3), 9);
                    when 2 => f_g64 <= resize('1' & v_k(7 downto 2), 9);
                    when others => f_g64 <= resize('1' & v_k(7 downto 1), 9);
                end case;
                -- K5: rims k/32 - 8 (-8 .. 23; 8 = 1x at k 512)
                v_k := s_g(3)(12 downto 3);
                f_rim <= signed(resize(v_k(9 downto 5), 6)) - 8;
                f_rise <= f_fc(9 downto 1);
            elsif s_zone = '1' and s_zc = 41 then
                -- comb-map term: 66 x fq / 8 (Q3 palette steps)
                v_q := shift_left(resize(f_fq, 11), 6) + shift_left(resize(f_fq, 11), 1);
                f_kcm <= v_q(9 downto 3);

            end if;
            -- breathing: tri = 256 - |ph - 256| - 128, br = (tri * 3) >> 4; P12 heat (p12 - 512) * 11 / 16
            if s_zone = '1' and s_zc = 42 then
                v_t := f_fc + 128;
                if v_t(8) = '1' then
                    v_h := signed(resize(not v_t(7 downto 0), 14)) - 127;
                else
                    v_h := signed(resize(v_t(7 downto 0), 14)) - 128;
                end if;
                v_h := shift_left(v_h, 1) + v_h;
                f_br <= resize(shift_right(v_h, 4), 6);
                v_h := signed(resize(s_g(4)(12 downto 3), 14)) - 512;
                v_h := shift_left(v_h, 3) + shift_left(v_h, 1) + v_h;
                f_heat <= resize(shift_right(v_h, 4), 10);
            elsif s_zone = '1' and s_zc = 43 then
                f_heat <= f_heat + resize(f_br, 10);
            end if;

            -- serial multiplier, one product per slot: zc 64..95 K1 x 230 -> the tiles exponent (Q8 octaves of
            -- 0.5 .. 6.4 copies; 495 = the photo at k 551); zc 96..127 m4^2 -> drift speed; zc 416..447 (K2 - 512)^2 -> tear
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            if s_zone = '1' and (v_p = 2 or v_p = 3 or v_p = 13) then
                if v_ph = 0 then
                    m_acc <= (others => '0');
                    if v_p = 2 then m_a <= to_unsigned(230, 20); m_b <= s_g(0)(12 downto 3);
                    elsif v_p = 3 then m_a <= resize(s_m4, 20); m_b <= '0' & s_m4;
                    else m_a <= resize(s_kd, 20); m_b <= '0' & s_kd; end if;
                elsif v_ph <= 10 then
                    if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                    m_a <= shift_left(m_a, 1); m_b <= '0' & m_b(9 downto 1);
                elsif v_ph = 12 then
                    if v_p = 2 then
                        s_e1 <= m_acc(18 downto 8);
                    elsif v_p = 3 then
                        if s_s4 = '1' then f_sp <= -signed(resize(m_acc(18 downto 1), 18));
                        else f_sp <= signed(resize(m_acc(18 downto 1), 18)); end if;
                    else
                        if s_k2hi = '1' then s_fqd <= to_unsigned(8, 6) + resize(m_acc(17 downto 13), 6);
                        else s_fqd <= resize(f_fq, 6); end if;
                    end if;
                end if;
            end if;

            -- serial divider: slots at zc = 32 * k, k = 5..10: phase 0 load, 1..28 steps, 29 store
            if s_zone = '1' and v_p >= 5 and v_p <= 10 then
                if v_ph = 0 then
                    d_r <= (others => '0');
                    case v_p is
                        when 5 => d_n <= resize(f_nl & "000000000000000", 28); d_v <= s_wd;
                        when 6 => d_n <= to_unsigned(125829120, 28); d_v <= s_wd;
                        when 7 => d_n <= to_unsigned(70778880, 28);  d_v <= resize(s_hd, 12);
                        when 8 => d_n <= to_unsigned(31457280, 28);  d_v <= resize(s_hd, 12);
                        when 9 => d_n <= to_unsigned(8388608, 28);   d_v <= resize(s_hd, 12);
                        when others => d_n <= to_unsigned(8388608, 28); d_v <= s_wd;
                    end case;
                elsif v_ph <= 28 then
                    v_rm := d_r(11 downto 0) & d_n(27);
                    if v_rm >= resize(d_v, 13) then
                        d_r <= v_rm - resize(d_v, 13); d_n <= d_n(26 downto 0) & '1';
                    else
                        d_r <= v_rm; d_n <= d_n(26 downto 0) & '0';
                    end if;
                elsif v_ph = 29 then
                    case v_p is
                        when 5 => t_cx <= d_n(16 downto 0);
                        when 6 => t_vx <= d_n(17 downto 0);
                        when 7 => t_vy <= d_n(18 downto 0);
                        when 8 => t_L  <= d_n(17 downto 0);
                        when 9 => t_ry <= d_n(15 downto 0);
                        when others => t_wc <= d_n(15 downto 0);
                    end case;
                end if;
            end if;
            -- K1: tiles x256 = (256 + frac) << oct >> 1, e from the multiplier slot 2 (zc 100)
            if s_zone = '1' and s_zc = 100 then
                case to_integer(s_e1(9 downto 8)) is
                    when 0 => f_n256 <= resize('1' & s_e1(7 downto 1), 11);
                    when 1 => f_n256 <= resize('1' & s_e1(7 downto 0), 11);
                    when 2 => f_n256 <= resize('1' & s_e1(7 downto 0) & '0', 11);
                    when others => f_n256 <= '1' & s_e1(7 downto 0) & "00";
                end case;
            end if;
            -- S10 Tile Lock (zc 101..102): Lock glides f_nl onto a whole copy count, 1 .. 6 in six equal bands of
            -- the (glided) K1 travel (1 + K1 x 6 / 1024), by 1/16 of the gap per field (the last 15/256 one step
            -- per field, so it lands exactly); Free passes K1 straight through, after gliding back from wherever
            -- Lock left it
            if s_zone = '1' and s_zc = 101 then
                v_q := resize(s_g(0)(12 downto 3), 11) + resize(s_g(0)(12 downto 4), 11);   -- K1 x 1.5
                v_q := resize(v_q(10 downto 8) + 1, 11);
                if f_lock = '1' then
                    nl_d <= signed(resize(v_q(2 downto 0) & "00000000", 12)) - signed(resize(f_nl, 12));
                else
                    nl_d <= signed(resize(f_n256, 12)) - signed(resize(f_nl, 12));
                end if;
            elsif s_zone = '1' and s_zc = 102 then
                if f_lock = '0' and f_nret = '0' then
                    f_nl <= f_n256;
                elsif nl_d > 0 and nl_d < 16 then
                    f_nl <= f_nl + 1;
                else
                    f_nl <= unsigned(resize(signed(resize(f_nl, 12)) + shift_right(nl_d, 4), 11));
                end if;
                if f_lock = '1' then f_nret <= '1'; elsif nl_d = 0 then f_nret <= '0'; end if;
            end if;
            -- tear factor clipped to 31 (3.9x) (zc 460)
            if s_zone = '1' and s_zc = 460 then
                if s_fqd(5) = '1' then f_dm <= to_unsigned(31, 5); else f_dm <= s_fqd(4 downto 0); end if;
            end if;
            -- derived (zc 400)
            if s_zone = '1' and s_zc = 400 then
                f_cx0 <= to_unsigned(16777216 - 32768, 24) - f_dr;
                f_rv <= shift_left(resize(f_rise, 28), 17) + shift_left(resize(f_rise, 28), 14);
                if s_wd >= 1280 then t_hd <= '1'; else t_hd <= '0'; end if;
                if s_hd >= 400 then t_hdl <= '1'; else t_hdl <= '0'; end if;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

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
    -- Per-line engine: 24 clocks after the source avid fall (the pixel path
    -- has drained) it computes the next line straight into the line
    -- registers and pre-rolls the first knots; line 0 runs in the vblank zone.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_r : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                n_vy <= (others => '0'); n_L <= (others => '0'); n_ry <= (others => '0');
                n_ln <= '0'; n_first <= '1';
            end if;
            if data_in.avid = '0' and s_avq = '1' then
                le_dl <= to_unsigned(24, 5);
            elsif le_dl /= 0 then
                le_dl <= le_dl - 1;
            end if;
            if s_zone = '1' and s_zc = 904 then
                le_st <= to_unsigned(2, 4);
            elsif le_dl = 1 then
                le_st <= to_unsigned(1, 4);
            elsif le_st /= 0 then
                if le_st = 5 then le_st <= (others => '0'); else le_st <= le_st + 1; end if;
            end if;

            case to_integer(le_st) is
                when 1 =>
                    n_vy <= n_vy + t_vy;
                    n_L  <= n_L + t_L;
                    n_ry <= n_ry + t_ry;
                    n_ln <= not n_ln;
                    n_first <= '0';
                when 2 =>
                    -- top half, or the bottom half mirrored / repeated (64 store rows)
                    if n_ry < 64 * 65536 then n_rr <= n_ry;
                    elsif f_rep = '0' then n_rr <= to_unsigned(128 * 65536 - 1, 24) - n_ry;
                    else n_rr <= n_ry - 64 * 65536; end if;
                    n_Lk <= n_L(25 downto 16) + resize(f_rise, 10);
                    n_vyk <= resize(n_vy, 28) + f_rv;
                    if n_ry(23 downto 17) > 63 then n_wr <= to_unsigned(63, 6); else n_wr <= n_ry(22 downto 17); end if;
                when 3 =>
                    if n_rr < 32768 then v_r := (others => '0'); else v_r := n_rr - 32768; end if;
                    if v_r(21 downto 16) = 63 then n_fy <= (others => '0'); else n_fy <= v_r(15 downto 12); end if;
                    n_r <= v_r(21 downto 16);
                    l_j0 <= n_vyk(27 downto 21); l_fy2 <= n_vyk(20 downto 17);
                    l_j1 <= resize(n_vyk(27 downto 21), 8) + 1;
                    l_Lk <= n_Lk;
                    if t_hdl = '1' then l_sg <= n_L(16);
                    else l_sg <= n_ln; end if;
                    if n_first = '1' or n_wr /= n_wrp then l_wok <= '1'; else l_wok <= '0'; end if;
                    n_wrp <= n_wr; l_wrow <= n_wr;
                when 4 =>
                    if n_r(0) = '0' then
                        l_ra0 <= n_r(5 downto 1); l_ra1 <= n_r(5 downto 1);
                    else
                        l_ra1 <= n_r(5 downto 1); l_ra0 <= n_r(5 downto 1) + 1;
                    end if;
                    l_r0p <= n_r(0); l_fy <= n_fy;
                    if n_r = 63 then l_r63 <= '1'; else l_r63 <= '0'; end if;
                    -- S9 Strips: the band of the mirrored store row -> its offset and whether it stands
                    if f_strp = '1' then
                        l_boff <= f_soff(n_rr(21 downto 19)); l_bst <= not f_sscr(n_rr(21 downto 19));
                    else
                        l_boff <= (others => '0'); l_bst <= '0';
                    end if;
                when others => null;
            end case;

        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Knot engine: two pipelined 16-bit hashes.  Line layer: knot k -> {sN, bN}
    -- x Fibres; smooth layer: rows j0, j0+1 at knot m -> y-lerped {D, C} x Fibres.
    -- Pre-rolled (knots 0..2) by the line engine; during the line each knot
    -- crossing shifts the three registers and requests knot +2.
    ---------------------------------------------------------------------------
    p_knot : process(clk)
        variable v_kn  : unsigned(5 downto 0);
        variable v_mn  : unsigned(4 downto 0);
        variable v_fq  : signed(4 downto 0);
        variable v_q   : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            v_fq := signed(resize(f_fq, 5));
            -- requests into ONE shared hash pipeline; tag bit 3 = smooth layer, then the layer's own tag
            hs_v0 <= '0';
            k2_rqa <= '0'; k2_rqb <= '0';
            if le_st = 5 then
                pr_st <= to_unsigned(1, 4);
            elsif pr_st /= 0 then
                if pr_st = 9 then pr_st <= (others => '0'); else pr_st <= pr_st + 1; end if;
            end if;
            if pr_st /= 0 then
                -- pre-roll: line knots 0..2 (pr 1..3), then smooth knots 0..2 x rows j0 / j0+1 (pr 4..9)
                hs_v0 <= '1';
                if pr_st <= 3 then
                    v_q := resize(pr_st - 1, 3);
                    hs_t0 <= "00" & v_q(1 downto 0);
                    hs_k0 <= (l_Lk & resize(v_q, 6)) xor unsigned(C_SDL);
                else
                    v_q := resize(pr_st - 4, 3);
                    v_mn := resize(v_q(2 downto 1), 5);
                    hs_t0 <= '1' & (not v_q(0)) & v_q(2 downto 1);   -- bit 2 = row j0 (first of the pair)
                    if v_q(0) = '0' then
                        hs_k0 <= (resize(l_j0, 11) & v_mn) xor unsigned(C_SD2);
                    else
                        hs_k0 <= (resize(l_j1, 11) & v_mn) xor unsigned(C_SD2);
                    end if;
                end if;
            else
                -- a line knot at its crossing; at a smooth crossing (also a line crossing) its two rows follow
                -- on the next two clocks (crossings are >= 12 clocks apart)
                if b_kc = '1' and g_avid = '1' then
                    hs_v0 <= '1'; hs_t0 <= "0010";
                    v_kn := b_vx(26 downto 21) + 2;
                    hs_k0 <= (l_Lk & v_kn) xor unsigned(C_SDL);
                end if;
                if b_mc = '1' and g_avid = '1' then
                    k2_rm <= b_vx(26 downto 22) + 2; k2_rqa <= '1';
                end if;
                if k2_rqa = '1' then
                    hs_v0 <= '1'; hs_t0 <= "1110"; k2_rqb <= '1';
                    hs_k0 <= (resize(l_j0, 11) & k2_rm) xor unsigned(C_SD2);
                end if;
                if k2_rqb = '1' then
                    hs_v0 <= '1'; hs_t0 <= "1010";
                    hs_k0 <= (resize(l_j1, 11) & k2_rm) xor unsigned(C_SD2);
                end if;
            end if;
            -- shared hash pipeline
            hs_v1 <= hs_v0; hs_t1 <= hs_t0; hs_k1 <= f_ha(hs_k0);
            hs_v2 <= hs_v1; hs_t2 <= hs_t1; hs_k2 <= f_hb(hs_k1);
            hs_v3 <= hs_v2; hs_t3 <= hs_t2; hs_k3 <= f_hc(hs_k2);
            -- smooth pair: hold row j0, at row j0+1 form the y products, then the lerp, then x Fibres
            h2_vl <= '0';
            if hs_v3 = '1' and hs_t3(3) = '1' then
                if hs_t3(2) = '1' then
                    h2_aD <= signed(hs_k3(7 downto 0)); h2_aC <= signed(hs_k3(15 downto 8));
                else
                    h2_dD <= resize((resize(signed(hs_k3(7 downto 0)), 9) - resize(h2_aD, 9)) * signed(resize(l_fy2, 5)), 13);
                    h2_dC <= resize((resize(signed(hs_k3(15 downto 8)), 9) - resize(h2_aC, 9)) * signed(resize(l_fy2, 5)), 13);
                    h2_bD <= h2_aD; h2_bC <= h2_aC;
                    h2_vl <= '1'; h2_tl <= hs_t3(2 downto 0);
                end if;
            end if;
            h2_vs <= h2_vl; h2_ts <= h2_tl;
            h2_yD <= resize(resize(h2_bD, 13) + shift_right(h2_dD, 4), 8);
            h2_yC <= resize(resize(h2_bC, 13) + shift_right(h2_dC, 4), 8);


            -- knot registers: shift at the crossing pixel's S0 -> S1 edge; results land by tag
            if pr_st = 0 and b_kc = '1' and g_avid = '1' then
                kS(0) <= kS(1); kS(1) <= kS(2); kB(0) <= kB(1); kB(1) <= kB(2);
            elsif hs_v3 = '1' and hs_t3(3) = '0' then
                kS(to_integer(hs_t3(1 downto 0))) <= resize(signed(hs_k3(9 downto 4)) * v_fq, 10);
                kB(to_integer(hs_t3(1 downto 0))) <= resize(signed(hs_k3(15 downto 10)) * v_fq, 10);
            end if;
            if pr_st = 0 and b_mc = '1' and g_avid = '1' then
                mD(0) <= mD(1); mD(1) <= mD(2); mC(0) <= mC(1); mC(1) <= mC(2);
            elsif h2_vs = '1' then
                -- the tear takes its own factor (mantissa; the shift is applied in the pixel path)
                mD(to_integer(h2_ts(1 downto 0))) <= resize(h2_yD * signed('0' & f_dm), 13);
                mC(to_integer(h2_ts(1 downto 0))) <= resize(h2_yC * v_fq, 12);
            end if;
        end if;
    end process p_knot;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): virtual x (Q16) and knot crossings; the store column
    -- accumulator runs four stages later (c_cx, aligned with p4)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(26 downto 0);
        variable v_b : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_vx <= (others => '0'); b_kc <= '0'; b_mc <= '0';
            elsif g_avid = '1' then
                v_n := b_vx + resize(t_vx, 27);
                b_vx <= v_n;
                b_kc <= v_n(21) xor b_vx(21);
                b_mc <= v_n(22) xor b_vx(22);
            end if;
            p_gs(1) <= g_start;
            for i in 2 to 4 loop p_gs(i) <= p_gs(i - 1); end loop;
            if p_gs(4) = '1' then
                -- the line's start column: the drifted base, or under Strips the undrifted one for a standing
                -- band, + the band's offset in quarter copies (top 3 bits)
                if l_bst = '1' then v_b := to_unsigned(16777216 - 32768, 24); else v_b := f_cx0; end if;
                c_cx <= (v_b(23 downto 21) + l_boff) & v_b(20 downto 0);
            else c_cx <= c_cx + resize(t_cx, 24); end if;
        end if;
    end process p_beam;

    p_vw : process(clk)
        variable v_by : signed(14 downto 0);
        variable v_y  : signed(14 downto 0);
        variable v_uw : unsigned(22 downto 0);
        variable v_col : unsigned(6 downto 0);
        variable v_yi : signed(10 downto 0);
        variable v_c  : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            w_ev <= '0';
            if data_in.avid = '1' then
                if s_avq = '0' then
                    v_by := to_signed(64 * 16, 15); v_uw := (others => '0');
                else
                    v_by := signed('0' & w_yf); v_uw := w_uw;
                end if;
                v_y := signed(shift_left(resize(unsigned(data_in.y), 15), 4)) - v_by;
                if t_hd = '1' then v_y := v_by + shift_right(v_y, 4); else v_y := v_by + shift_right(v_y, 3); end if;
                w_yf <= unsigned(v_y(13 downto 0));
                v_col := v_uw(22 downto 16);
                v_uw := v_uw + resize(t_wc, 23);
                w_uw <= v_uw;
                if v_uw(22 downto 16) /= v_col then
                    w_ev <= '1'; w_col <= v_col;
                    w_y0 <= unsigned(v_y(13 downto 4));
                end if;
            elsif s_avq = '1' then
                w_ev <= '1'; w_col <= w_uw(22 downto 16);
                w_y0 <= w_yf(13 downto 4);
            end if;
            -- code = (Y - 64) * 18 >> 10, 0..15
            v_yi := signed(resize(w_y0, 11)) - 64;
            if v_yi < 0 then v_c := (others => '0');
            else v_c := shift_left(resize(unsigned(v_yi(9 downto 0)), 15), 4) + shift_left(resize(unsigned(v_yi(9 downto 0)), 15), 1); end if;
            w_cd <= v_c; w_ev2 <= w_ev; w_col2 <= w_col;
            if w_cd(14) = '1' then w_d <= "1111"; else w_d <= std_logic_vector(w_cd(13 downto 10)); end if;
            w_a <= l_wrow(5 downto 1) & w_col2(6 downto 1);
            for b in 0 to 3 loop
                if w_ev2 = '1' and l_wok = '1' and bidx(l_wrow(0), w_col2(0)) = b then w_we(b) <= '1'; else w_we(b) <= '0'; end if;
            end loop;
        end if;
    end process p_vw;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d : signed(12 downto 0);
        variable v_c : signed(11 downto 0);
        variable v_cx : unsigned(23 downto 0);
        variable v_col, v_c1 : unsigned(6 downto 0);
        variable v_fx : unsigned(3 downto 0);
        variable v_ce, v_co : unsigned(5 downto 0);
        variable v_be, v_bo : unsigned(4 downto 0);
        type t_q4 is array (0 to 3) of std_logic_vector(3 downto 0);
        variable v_q, v_bq : t_q4;
        variable v_s  : signed(12 downto 0);
        variable v_t  : signed(12 downto 0);
        variable v_ix : signed(14 downto 0);
        variable v_cm : signed(13 downto 0);
        variable v_y  : signed(11 downto 0);
        variable v_u, v_v, v_w : signed(10 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_db : signed(8 downto 0);
        variable v_pb : signed(10 downto 0);
        variable v_h0, v_h1, v_v0, v_v1 : unsigned(3 downto 0);
        type t_e4 is array (0 to 3) of signed(5 downto 0);
        variable v_e : t_e4;
        variable v_pc : std_logic;
    begin
        if rising_edge(clk) then
            -- 1: fractions (the knot registers shift on this edge)
            p1_fx5 <= b_vx(20 downto 16);
            p1_fxm <= b_vx(21 downto 17);

            -- 2: x-lerp products of the four noise layers
            p2_s0 <= kS(0); p2_b0 <= kB(0); p2_d0 <= mD(0); p2_c0 <= mC(0);
            p2_ps <= resize((resize(kS(1), 11) - resize(kS(0), 11)) * signed(resize(p1_fx5, 6)), 16);
            p2_pb <= resize((resize(kB(1), 11) - resize(kB(0), 11)) * signed(resize(p1_fx5, 6)), 16);
            p2_pd <= resize((resize(mD(1), 14) - resize(mD(0), 14)) * signed(resize(p1_fxm, 6)), 19);
            p2_pc <= resize((resize(mC(1), 13) - resize(mC(0), 13)) * signed(resize(p1_fxm, 6)), 18);

            -- 3: layer values; comb sign on the smooth layer
            p3_sn <= resize(resize(p2_s0, 11) + resize(shift_right(p2_ps, 5), 11), 10);
            p3_bn <= resize(resize(p2_b0, 11) + resize(shift_right(p2_pb, 5), 11), 10);
            v_d := resize(resize(p2_d0, 14) + resize(shift_right(p2_pd, 5), 14), 13);
            v_c := resize(resize(p2_c0, 13) + resize(shift_right(p2_pc, 5), 13), 12);
            if l_sg = '1' then p3_d <= -v_d; p3_c <= -v_c; else p3_d <= v_d; p3_c <= v_c; end if;

            -- 4: displacement (cells Q16) = 56 D; s-noise partials 5 sN, 7 C; b-noise 26 bN
            p4_dc <= shift_left(resize(p3_d, 20), 6) - shift_left(resize(p3_d, 20), 3);
            p4_s5 <= shift_left(resize(p3_sn, 12), 2) + resize(p3_sn, 12);
            p4_c7 <= shift_left(resize(p3_c, 15), 3) - resize(p3_c, 15);
            p4_b26 <= shift_left(resize(p3_bn, 15), 4) + shift_left(resize(p3_bn, 15), 3) + shift_left(resize(p3_bn, 15), 1);

            -- 5: store column (the column accumulator is aligned with p4); s noise (Q3), b noise (x16) -> ring
            p5_cx <= c_cx + unsigned(resize(p4_dc, 24));
            p5_ns <= resize(shift_right(p4_s5, 4), 10) + resize(shift_right(p4_c7, 8), 10);
            p5_nb <= resize(shift_right(p4_b26, 10), 6);

            -- 6: column, Kaleido flip, bank addresses
            v_cx := p5_cx;
            v_col := v_cx(22 downto 16); v_fx := v_cx(15 downto 12);
            if f_kal = '1' and v_cx(23) = '1' then v_col := 127 - v_col; v_fx := 15 - v_fx; end if;
            v_c1 := v_col + 1;
            v_co := v_col(6 downto 1);
            if v_col(0) = '1' then v_ce := v_c1(6 downto 1); else v_ce := v_col(6 downto 1); end if;
            a_s0 <= l_ra0 & v_ce; a_s1 <= l_ra0 & v_co;
            a_s2 <= l_ra1 & v_ce; a_s3 <= l_ra1 & v_co;
            v_bo := v_col(6 downto 2);
            if v_col(1) = '1' then v_be := resize(shift_right(resize(v_col(6 downto 1), 6) + 1, 1), 5); else v_be := v_col(6 downto 2); end if;
            a_b0 <= l_ra0 & v_be; a_b1 <= l_ra0 & v_bo;
            a_b2 <= l_ra1 & v_be; a_b3 <= l_ra1 & v_bo;
            p6_cp <= v_col(0); p6_bp <= v_col(1);
            p6_fx <= v_fx;
            if f_osc = '1' then p6_fxb <= v_col(0) & v_fx(3); else p6_fxb <= v_fx(3 downto 2); end if;

            -- 7: (ROM / RAM read) ; 8: fabric copies
            p_fx(7) <= p6_fx; p_fxb(7) <= p6_fxb; p_cp(7) <= p6_cp; p_bp(7) <= p6_bp;
            if f_osc = '1' then
                f8_s0 <= r_s0; f8_s1 <= r_s1; f8_s2 <= r_s2; f8_s3 <= r_s3;
            else
                f8_s0 <= r_v0; f8_s1 <= r_v1; f8_s2 <= r_v2; f8_s3 <= r_v3;
            end if;
            f8_b0 <= r_b0; f8_b1 <= r_b1; f8_b2 <= r_b2; f8_b3 <= r_b3;

            -- 9: corners by parity, level decode (Video: no lift, no comb map); comb map nearest
            v_q := (f8_s0, f8_s1, f8_s2, f8_s3);
            v_bq := (f8_b0, f8_b1, f8_b2, f8_b3);
            p9_q00 <= f_ls(v_q(bidx(l_r0p, p_cp(8))));
            p9_q01 <= f_ls(v_q(bidx(l_r0p, not p_cp(8))));
            p9_q10 <= f_ls(v_q(bidx(not l_r0p, p_cp(8))));
            p9_q11 <= f_ls(v_q(bidx(not l_r0p, not p_cp(8))));
            -- lift per bank: Osc the stored b level; Video the rims' edge map (each corner = its two neighbours'
            -- |code difference| on the camera grid; no row below store row 63); one corner mux for both
            v_h0 := f_ad(f8_s0, f8_s1); v_h1 := f_ad(f8_s2, f8_s3);
            if l_r63 = '1' then v_v0 := (others => '0'); v_v1 := (others => '0');
            else v_v0 := f_ad(f8_s0, f8_s2); v_v1 := f_ad(f8_s1, f8_s3); end if;
            if f_osc = '1' then
                for i in 0 to 3 loop v_e(i) := f_lb(v_bq(i)(3 downto 1)); end loop;
                v_pc := p_bp(8);
            else
                v_e(0) := f_em(resize(v_h0, 5) + resize(v_v0, 5));
                v_e(1) := f_em(resize(v_h0, 5) + resize(v_v1, 5));
                v_e(2) := f_em(resize(v_h1, 5) + resize(v_v0, 5));
                v_e(3) := f_em(resize(v_h1, 5) + resize(v_v1, 5));
                v_pc := p_cp(8);
            end if;
            p9_b00 <= v_e(bidx(l_r0p, v_pc));
            p9_b01 <= v_e(bidx(l_r0p, not v_pc));
            p9_b10 <= v_e(bidx(not l_r0p, v_pc));
            p9_b11 <= v_e(bidx(not l_r0p, not v_pc));
            p_cm(9) <= f_osc and v_bq(bidx(l_r0p, p_bp(8)))(0);

            -- 10: y-lerp products (s: 4-bit fraction; b: 2-bit)
            p10_q00 <= p9_q00; p10_q01 <= p9_q01; p10_b00 <= p9_b00; p10_b01 <= p9_b01;
            p10_ps0 <= resize((signed(resize(p9_q10, 8)) - signed(resize(p9_q00, 8))) * signed(resize(l_fy, 5)), 12);
            p10_ps1 <= resize((signed(resize(p9_q11, 8)) - signed(resize(p9_q01, 8))) * signed(resize(l_fy, 5)), 12);
            p10_pb0 <= f_m2(resize(p9_b10, 7) - resize(p9_b00, 7), l_fy(3 downto 2));
            p10_pb1 <= f_m2(resize(p9_b11, 7) - resize(p9_b01, 7), l_fy(3 downto 2));

            -- 11: y-lerp sums (s Q3, b x16 Q2)
            p11_sa <= signed(shift_left(resize(p10_q00, 12), 3)) + shift_right(p10_ps0, 1);
            p11_sb <= signed(shift_left(resize(p10_q01, 12), 3)) + shift_right(p10_ps1, 1);
            p11_ba <= shift_left(resize(p10_b00, 9), 2) + p10_pb0;
            p11_bb <= shift_left(resize(p10_b01, 9), 2) + p10_pb1;

            -- 12: x-lerp products
            p12_sa <= p11_sa; p12_ba <= shift_left(resize(p11_ba, 11), 2);
            p12_ps <= resize((resize(p11_sb, 13) - resize(p11_sa, 13)) * signed(resize(p_fx(11), 5)), 17);
            v_db := p11_bb - p11_ba;
            case p_fxb(11) is
                when "00" => v_pb := (others => '0');
                when "01" => v_pb := resize(v_db, 11);
                when "10" => v_pb := shift_left(resize(v_db, 11), 1);
                when others => v_pb := shift_left(resize(v_db, 11), 1) + resize(v_db, 11);
            end case;
            p12_pb <= v_pb;

            -- 13: x-lerp sums
            v_s := resize(p12_sa, 13) + resize(shift_right(p12_ps, 4), 13);
            p13_s <= resize(v_s, 12);
            p13_b <= resize(shift_right(p12_ba + p12_pb, 4), 7);

            -- 14: Blocks gain, Rims
            p14_gp <= resize((resize(p13_s, 13) - 320) * signed(resize(f_g64, 10)), 20);
            p14_bp <= resize(p13_b * f_rim, 13);

            -- 15: s offset (Heat); b x Rims; (ring read -> fabric)
            p15_s <= resize(shift_right(p14_gp, 6), 13) + 320 + resize(f_heat, 13);
            p15_b <= resize(shift_right(p14_bp + 4, 3), 10);
            f15_dr <= r_dr;

            -- 16: + s noise + comb-map term (field sign); + b noise
            if p_cm(15) = '0' then v_cm := (others => '0');
            elsif l_sg = '1' then v_cm := -signed(resize(f_kcm, 14));
            else v_cm := signed(resize(f_kcm, 14)); end if;
            p16_s <= resize(p15_s, 14) + resize(signed(f15_dr(15 downto 6)), 14) + v_cm;
            p16_b <= p15_b + resize(signed(f15_dr(5 downto 0)), 10);

            -- 17: palette index; clip b
            v_ix := resize(shift_right(resize(p16_s, 15) + 4, 3), 15);
            if v_ix < 0 then a_p <= (others => '0');
            elsif v_ix > 127 then a_p <= to_unsigned(127, 8);
            else a_p <= unsigned(v_ix(7 downto 0)); end if;
            if p16_b > 28 then p_b6(17) <= to_signed(28, 6);
            elsif p16_b < -28 then p_b6(17) <= to_signed(-28, 6);
            else p_b6(17) <= resize(p16_b, 6); end if;

            -- 18: (palette ROM) ; 19: fabric copy
            p_b6(18) <= p_b6(17); p_b6(19) <= p_b6(18);
            p19_p0 <= r_p0; p19_p1 <= r_p1; p19_p2 <= r_p2;

            -- 20: luma lift product b x DY
            p20_py <= unsigned(p19_p0(15 downto 6)); p20_pu <= unsigned(p19_p1(15 downto 6)); p20_pv <= unsigned(p19_p2(15 downto 6));
            p20_dy <= resize(p_b6(19) * signed(std_logic_vector'(p19_p0(5 downto 0) & p19_p1(5 downto 3))), 15);

            -- 21: colour; Glaze (chroma swaps / turns / complement / mono) on the signed chroma
            p21_y <= signed(resize(p20_py, 12)) + resize(shift_right(p20_dy + 8, 4), 12);
            v_u := signed(resize(p20_pu, 11)) - 512; v_v := signed(resize(p20_pv, 11)) - 512;
            case to_integer(f_pl) is
                when 1 => v_w := v_u; v_u := v_v; v_v := v_w;
                when 2 => v_v := -shift_right(v_v, 1);
                when 3 => v_u := -v_u; v_v := -v_v;
                when 4 => v_w := v_u; v_u := -v_v; v_v := v_w;
                when 5 => v_u := (others => '0'); v_v := (others => '0');
                when 6 => v_w := v_u; v_u := v_v; v_v := -v_w;
                when 7 => v_u := shift_right(v_u, 1); v_v := shift_right(v_v, 1);
                when others => null;
            end case;
            p21_u <= v_u; p21_v <= v_v;

            -- 22: clamps (chroma to 512 +- 448: clamped in the signed domain, + 512 is a bit flip)
            v_u := p21_u; v_v := p21_v;
            if v_u < -448 then v_u := to_signed(-448, 11); elsif v_u > 448 then v_u := to_signed(448, 11); end if;
            if v_v < -448 then v_v := to_signed(-448, 11); elsif v_v > 448 then v_v := to_signed(448, 11); end if;
            v_u := v_u + 512; v_v := v_v + 512;
            v_y := p21_y;
            if v_y < 64 then v_y := to_signed(64, 12); elsif v_y > 941 then v_y := to_signed(941, 12); end if;
            -- chroma neutral off the generated avid, so the 8-sums below resync in every blanking interval
            p22_y <= unsigned(v_y(9 downto 0));
            if s_gav_sr(20) = '0' then p22_u <= to_unsigned(512, 10); p22_v <= to_unsigned(512, 10);
            else p22_u <= unsigned(v_u(9 downto 0)); p22_v <= unsigned(v_v(9 downto 0)); end if;
            s_bxh <= s_bxh(6 downto 0) & s_gav_sr(20);

            -- 23..28: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]).  The running 8-sums snap back to neutral once 8 blank
            -- samples are in, so a glitched sample can't offset the chroma for good.
            -- s_csu7 / s_csv7 = p22_u / p22_v of 8 clocks before (block-RAM rings, see p_cring)
            if s_bxh = "00000000" then
                s_boxu <= to_unsigned(4096, 14); s_boxv <= to_unsigned(4096, 14);
            else
                s_boxu <= s_boxu + p22_u - s_csu7;
                s_boxv <= s_boxv + p22_v - s_csv7;
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u28 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v28 <= v_sc(13 downto 4);
            s_y23 <= p22_y; s_y24 <= s_y23; s_y25 <= s_y24; s_y26 <= s_y25;
            s_y27 <= s_y26; s_y28x <= s_y27;
            v_sy := resize(s_y26, 12) + shift_left(resize(s_y27, 12), 1) + resize(s_y28x, 12);
            s_yf28 <= v_sy(11 downto 2);
            -- 29: luma toward black (white 940 -> 777); blanking gate
            if s_yf28 > 64 then v_dl := s_yf28 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(27) = '1' then
                s_y29 <= s_yf28 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u29 <= s_u28; s_v29 <= s_v28;
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
            -- 30..32: pad to the 32-clock latency
            s_y30 <= s_y29; s_u30 <= s_u29; s_v30 <= s_v29;
            s_y31 <= s_y30; s_u31 <= s_u30; s_v31 <= s_v30;
            s_y32 <= s_y31; s_u32 <= s_u31; s_v32 <= s_v31;

            -- delays
            for i in 8 to 11 loop p_fx(i) <= p_fx(i - 1); p_fxb(i) <= p_fxb(i - 1); end loop;
            p_cp(8) <= p_cp(7); p_bp(8) <= p_bp(7);
            for i in 10 to 15 loop p_cm(i) <= p_cm(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y32);
    data_out.u       <= std_logic_vector(s_u32);
    data_out.v       <= std_logic_vector(s_v32);
    data_out.hsync_n <= s_so(2);
    data_out.vsync_n <= s_so(1);
    data_out.field_n <= s_so(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture kiln;
