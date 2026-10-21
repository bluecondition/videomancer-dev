-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Mandala v0.5.0 -- Heirloom #6, rebuilt from the photo WIN_20230128_01_06_08.
--
-- The photo: a left/right mirrored psychedelic mask -- two eye lobes of nested
-- rings with red/yellow cores, a brow cusp where the ring systems cross, red
-- forehead and cheek lobes, a blue face outline, a black mouth at the bottom
-- centre, red lobes with dark cores at the corners.  Every colour is three
-- channels driven by ONE smooth field F: G is a soft pulse of frac(F) (the fine
-- pink/white/green/yellow contour stripes), B a monotone ramp of F (the blue
-- glow), R a soft key on a second, slower field FR whose mirror axis sits 16 px
-- right of F's (the red channel is misregistered on the original).
--
-- The program: F and FR are the fitted surfaces themselves -- quadratic
-- B-splines on a 24-virtual-px grid (43 x 46 control points, mandala.py).  A
-- per-line engine blends three control rows into one row of knots (3 MACs per
-- knot, ~140 clocks, computing the NEXT line into a ping-pong cell table); the
-- pixel path evaluates the cell polynomial F = P0 + s P1 + s^2 P2 at the
-- mirrored column and runs the colourizer: stripe pulse ROM (8 widths), blue
-- ramp ROM, linear red key, RGB -> YUV, box-8 Y/U/V (box-4 at 720p/SD; the original's soft,
-- band-limited stripes).  The video luma (or the Osc) is added into the field,
-- so the contours bend around the picture.
--
--   K1  Flow     stripe drift, bipolar (centre +-8 = still): + rings stream out of the eyes
--   K2  Width    stripe width: fat bands .. thin lines (8 steps, step 4 = the photo)
--   K3  Blue     the blue glow level, bipolar (centre = the photo)
--   K4  Red      the red lobes' size, bipolar (centre = the photo)
--   K5  Melt     how far the video (or Osc) bends the contours
--   K6  Breathe  depth of a slow LFO on the field: the eyes and mouth swell and shrink
--   S7  Stripes  Green (the photo) / Blue: the stripe and glow channels swap
--   S8  Lobes    Red (the photo) / Green: the key and stripe channels swap
--   S9  Edge     Soft (the photo) / Hard comparator stripes
--   S10 Faces    One / Two (a second fold: two masks side by side)
--   S11 Source   Video / Osc
--   P12 RINGS    stripe density: 0 = a few broad bands, 50 % = the photo, 100 % = 3.2x
--
-- Virtual 1080-line geometry (round at SD, 720p and 1080).  Generated avid;
-- output conditioning (luma box + [1 2 1], chroma box pair [1 2 .. 2 1], luma to ~777)
-- before the blanking gate.  Latency 28 clocks in every mode.
-- Generated colours in the hardware convention (U carries Cr, V carries Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.mandala_pkg.all;
use work.all;

architecture mandala of program_top is
    constant C_LAT : integer := 28;   -- matches processing_delay_clks

    -- per-timing-ID constants: kx = cells (24 virtual px) per pixel, Q16; ky = cells per line, Q16
    function f_kx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(5461, 14);
            when 5 | 6 | 14     => return to_unsigned(4096, 14);
            when others         => return to_unsigned(2731, 14);
        end case;
    end function;
    function f_ky(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(12139, 14);
            when 4          => return to_unsigned(6144, 14);
            when 8          => return to_unsigned(10240, 14);
            when 12         => return to_unsigned(5120, 14);
            when 5 | 6 | 14 => return to_unsigned(4096, 14);
            when 1 | 2 | 10 => return to_unsigned(5461, 14);
            when others     => return to_unsigned(2731, 14);
        end case;
    end function;
    -- Osc spatial steps (Q16 cycles per pixel / per line)
    function f_kax(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(89, 9);
            when 5 | 6 | 14     => return to_unsigned(67, 9);
            when others         => return to_unsigned(44, 9);
        end case;
    end function;
    function f_kay(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(270, 9);
            when 4          => return to_unsigned(137, 9);
            when 8          => return to_unsigned(228, 9);
            when 12         => return to_unsigned(114, 9);
            when 5 | 6 | 14 => return to_unsigned(91, 9);
            when 1 | 2 | 10 => return to_unsigned(121, 9);
            when others     => return to_unsigned(61, 9);
        end case;
    end function;
    function f_issd(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return '1';
            when others         => return '0';
        end case;
    end function;
    function f_isb8(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 | 5 | 6 | 14 => return '1';
            when others => return '0';
        end case;
    end function;
    -- 8-bit triangle -> signed -127..127
    function f_tri(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;

    -- timing
    signal t_kx : unsigned(13 downto 0) := to_unsigned(2731, 14);
    signal t_ky : unsigned(13 downto 0) := to_unsigned(2731, 14);
    signal t_kax, t_kay : unsigned(8 downto 0) := (others => '0');
    signal t_sd, t_b8 : std_logic := '0';   -- t_b8: 720p / SD (the short box)

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K2, K3, K4, K5, K6, P12 (Q.3), power up at the defaults
    type t_g is array(0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(576 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(400 * 8, 13), to_unsigned(384 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- serial multiplier (per-field products)
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 5) := (others => '0');
    signal m_a  : unsigned(19 downto 0) := (others => '0');
    signal m_b  : unsigned(17 downto 0) := (others => '0');
    signal m_h  : unsigned(19 downto 0) := (others => '0');
    signal m_l  : unsigned(17 downto 0) := (others => '0');

    -- per-field values
    signal f_wl    : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal f_blue  : signed(11 downto 0) := (others => '0');     -- Q9 stripes
    signal f_red   : signed(9 downto 0) := (others => '0');      -- Q7 key units
    signal f_melt  : unsigned(7 downto 0) := to_unsigned(100, 8);
    signal f_d8    : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal f_p8    : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal f_gain  : unsigned(9 downto 0) := to_unsigned(256, 10); -- Q8
    signal f_whc   : unsigned(10 downto 0) := to_unsigned(953, 11);
    signal f_u0    : signed(24 downto 0) := (others => '0');      -- Q16 cells
    signal f_fm    : unsigned(8 downto 0) := (others => '0');     -- flow magnitude
    signal f_fneg  : std_logic := '0';
    signal f_spd   : unsigned(6 downto 0) := (others => '0');     -- Q9 per field
    signal f_flow  : unsigned(8 downto 0) := (others => '0');     -- Q9 phase (mod 1 stripe)
    signal f_lph   : unsigned(15 downto 0) := (others => '0');    -- breathe LFO phase
    signal f_la    : unsigned(15 downto 0) := (others => '0');
    signal f_lneg  : std_logic := '0';
    signal f_s9    : unsigned(8 downto 0) := (others => '0');     -- |sin| 0..256
    signal f_off   : signed(11 downto 0) := (others => '0');      -- Q9 field offset
    signal f_ta, f_tb : unsigned(15 downto 0) := (others => '0');
    signal f_sw7, f_sw8, f_hard, f_tw, f_osc : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- per-line engine
    signal e_V    : unsigned(21 downto 0) := (others => '0');     -- Q16 cells
    signal e_wb   : std_logic := '0';
    signal r_bank : std_logic := '0';
    signal e_st   : unsigned(2 downto 0) := (others => '0');
    signal e_iss  : std_logic := '0';
    signal e_i    : unsigned(5 downto 0) := (others => '0');
    signal e_r    : unsigned(1 downto 0) := (others => '0');
    signal e_j    : unsigned(5 downto 0) := (others => '0');
    signal e_t8   : unsigned(7 downto 0) := (others => '0');
    signal e_t2   : unsigned(15 downto 0) := (others => '0');
    signal e_base : unsigned(10 downto 0) := (others => '0');
    signal e_abi  : unsigned(10 downto 0) := (others => '0');
    signal e_w0, e_w1, e_w2 : unsigned(8 downto 0) := (others => '0');
    signal r_ca   : unsigned(10 downto 0) := (others => '0');
    signal cf_q, cf_q2 : std_logic_vector(11 downto 0) := (others => '0');
    signal cr_q, cr_q2 : std_logic_vector(7 downto 0) := (others => '0');
    type t_u2a is array (natural range <>) of unsigned(1 downto 0);
    type t_u6a is array (natural range <>) of unsigned(5 downto 0);
    signal e_tv : std_logic_vector(0 to 4) := (others => '0');
    signal e_tr : t_u2a(0 to 4) := (others => (others => '0'));
    signal e_ti : t_u6a(0 to 4) := (others => (others => '0'));
    signal e_wc : unsigned(8 downto 0) := (others => '0');
    signal e_fh, e_fl : signed(17 downto 0) := (others => '0');
    signal e_rh, e_rl : signed(13 downto 0) := (others => '0');
    signal e_pf : signed(21 downto 0) := (others => '0');
    signal e_pr : signed(17 downto 0) := (others => '0');
    signal e_af : signed(21 downto 0) := (others => '0');
    signal e_ar : signed(17 downto 0) := (others => '0');
    signal e_dv : std_logic := '0';
    signal e_di : unsigned(5 downto 0) := (others => '0');
    signal e_df : signed(15 downto 0) := (others => '0');          -- D, Q9
    signal e_dr : signed(11 downto 0) := (others => '0');          -- D_R, Q7
    signal e_f1, e_f2 : signed(15 downto 0) := (others => '0');    -- D[i-1], D[i-2]
    signal e_r1, e_r2 : signed(11 downto 0) := (others => '0');

    -- cell tables (ping-pong by line), {P0 14 | P1 13 | P2 12} and {P0 10 | P1 10 | P2 10}
    type t_tf is array (0 to 127) of std_logic_vector(38 downto 0);
    type t_tr is array (0 to 127) of std_logic_vector(29 downto 0);
    signal tf_ram : t_tf := (others => (others => '0'));
    signal tr_ram : t_tr := (others => (others => '0'));
    signal t_we : std_logic := '0';
    signal t_wa : unsigned(6 downto 0) := (others => '0');
    signal t_wdf : std_logic_vector(38 downto 0) := (others => '0');
    signal t_wdr : std_logic_vector(29 downto 0) := (others => '0');
    signal tf_ra, tr_ra : unsigned(6 downto 0) := (others => '0');
    signal tf_q : std_logic_vector(38 downto 0) := (others => '0');
    signal tr_q : std_logic_vector(29 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of tf_ram : signal is true;
    attribute no_rw_check of tr_ram : signal is true;

    -- beam
    signal b_u  : signed(24 downto 0) := (others => '0');
    signal b_pa : unsigned(15 downto 0) := (others => '0');
    signal l_pb : unsigned(15 downto 0) := (others => '0');

    -- video tap
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    type t_u8v is array (natural range <>) of unsigned(7 downto 0);
    signal s_vy : t_u8v(1 to 2) := (others => to_unsigned(128, 8));
    signal s_vs : signed(9 downto 0) := (others => '0');

    -- pixel path (stage N = N registers after the beam); coordinates in Q8 cells
    signal q1_au : unsigned(15 downto 0) := (others => '0');
    signal q1_u, q2_x : signed(16 downto 0) := (others => '0');
    signal q1_osc, q8_src : signed(9 downto 0) := (others => '0');
    signal q3_af : unsigned(15 downto 0) := (others => '0');
    signal q3_dr : signed(16 downto 0) := (others => '0');
    signal q4_if : unsigned(5 downto 0) := (others => '0');
    signal q4_sf, q5_sf, q6_sf, q7_sf : unsigned(7 downto 0) := (others => '0');
    signal q5_qf, q6_qf, q7_qf : unsigned(7 downto 0) := (others => '0');
    signal q4_ar : unsigned(15 downto 0) := (others => '0');
    signal q5_sr, q6_sr, q7_sr, q6_qr, q7_qr : unsigned(3 downto 0) := (others => '0');
    signal q7_f0 : signed(13 downto 0) := (others => '0');
    signal q7_f1 : signed(12 downto 0) := (others => '0');
    signal q7_f2 : signed(11 downto 0) := (others => '0');
    signal q7_r0, q7_r1, q7_r2 : signed(9 downto 0) := (others => '0');
    signal q8_fah, q8_fal : signed(17 downto 0) := (others => '0');
    signal q8_fbh, q8_fbl : signed(16 downto 0) := (others => '0');
    signal q8_ra, q8_rb : signed(14 downto 0) := (others => '0');
    signal q8_f0 : signed(13 downto 0) := (others => '0');
    signal q8_r0 : signed(9 downto 0) := (others => '0');
    signal q9_fa, q9_fb : signed(22 downto 0) := (others => '0');
    signal q9_fr : signed(11 downto 0) := (others => '0');
    signal q9_m  : signed(18 downto 0) := (others => '0');
    signal q10_f  : signed(15 downto 0) := (others => '0');
    signal q10_fr : signed(11 downto 0) := (others => '0');
    signal q10_om : signed(12 downto 0) := (others => '0');
    signal q11_f  : signed(15 downto 0) := (others => '0');
    signal q11_fr : signed(11 downto 0) := (others => '0');
    signal q12_gh, q12_gl : signed(21 downto 0) := (others => '0');
    signal q12_es, q12_bs : signed(15 downto 0) := (others => '0');
    signal q12_rx : signed(11 downto 0) := (others => '0');
    signal q13_fg : signed(26 downto 0) := (others => '0');
    signal q13_env, q14_env, q15_env, q16_env : unsigned(4 downto 0) := (others => '0');
    signal q13_bi : unsigned(7 downto 0) := (others => '0');
    signal q13_rx : unsigned(6 downto 0) := (others => '0');
    signal r_ga : unsigned(9 downto 0) := (others => '0');
    signal r_ba : unsigned(7 downto 0) := (others => '0');
    signal r_gq, r_bq : std_logic_vector(7 downto 0) := (others => '0');
    signal q14_r, q15_r, q16_r, q17_r : unsigned(7 downto 0) := (others => '0');
    signal q16_pu, q16_b, q17_b : unsigned(7 downto 0) := (others => '0');
    signal q17_g : unsigned(13 downto 0) := (others => '0');
    signal q18_r, q18_g, q18_b : unsigned(7 downto 0) := (others => '0');
    signal q19_yrg, q19_ugb, q19_vrg : unsigned(16 downto 0) := (others => '0');
    signal q19_yb, q19_ur, q19_vb : unsigned(15 downto 0) := (others => '0');
    signal q20_y : unsigned(16 downto 0) := (others => '0');
    signal q20_u, q20_v : signed(17 downto 0) := (others => '0');
    signal q21_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q21_u, q21_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- box ring (luma box + chroma box pair over the same 8-tap (4 at 720p/SD) window) and sync ring
    type t_ring is array (0 to 255) of std_logic_vector(29 downto 0);
    signal rb_ram : t_ring := (others => "0000000000" & "1000000000" & "1000000000");
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal rs_ram : t_sring := (others => "111");
    attribute no_rw_check of rb_ram : signal is true;
    attribute no_rw_check of rs_ram : signal is true;
    signal s_wp, rb_ra, rs_ra : unsigned(7 downto 0) := (others => '0');
    signal rb_wd : std_logic_vector(29 downto 0) := "0000000000" & "1000000000" & "1000000000";
    signal rb_q, rb_q2 : std_logic_vector(29 downto 0) := "0000000000" & "1000000000" & "1000000000";
    signal rs_q, rs_q2 : std_logic_vector(2 downto 0) := "111";
    signal s_ysum, s_usum, s_vsum, s_ud, s_vd : unsigned(13 downto 0) := (others => '0');
    signal q23_y, q24_y, q25_y, q25_yf : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q24_u, q24_v, q25_u, q25_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal q27_y, q28_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q26_u, q26_v, q27_u, q27_v, q28_u, q28_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- syncs ride the block-RAM ring (p_sring)   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs and the cell tables
    ---------------------------------------------------------------------------
    p_crom : process(clk)
    begin
        if rising_edge(clk) then
            cf_q <= C_CF(to_integer(r_ca));
            cr_q <= C_CR(to_integer(r_ca));
        end if;
    end process p_crom;

    p_tf : process(clk)
    begin
        if rising_edge(clk) then
            if t_we = '1' then tf_ram(to_integer(t_wa)) <= t_wdf; end if;
            tf_q <= tf_ram(to_integer(tf_ra));
        end if;
    end process p_tf;

    p_tr : process(clk)
    begin
        if rising_edge(clk) then
            if t_we = '1' then tr_ram(to_integer(t_wa)) <= t_wdr; end if;
            tr_q <= tr_ram(to_integer(tr_ra));
        end if;
    end process p_tr;

    p_prom : process(clk)
    begin
        if rising_edge(clk) then
            r_gq <= C_GP(to_integer(r_ga));
            r_bq <= C_BR(to_integer(r_ba));
        end if;
    end process p_prom;

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
        variable v_k  : signed(10 downto 0);
        variable v_k3 : signed(12 downto 0);
        variable v_g3 : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            t_kx  <= f_kx(registers_in(8)(3 downto 0));
            t_ky  <= f_ky(registers_in(8)(3 downto 0));
            t_kax <= f_kax(registers_in(8)(3 downto 0));
            t_kay <= f_kay(registers_in(8)(3 downto 0));
            t_sd  <= f_issd(registers_in(8)(3 downto 0));
            t_b8  <= f_isb8(registers_in(8)(3 downto 0));

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
                -- K1 Flow: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(0)), 11)) - 512;
                if v_x > 8 then f_fneg <= '0'; v_u := unsigned(v_x) - 8; f_fm <= v_u(8 downto 0);
                elsif v_x < -8 then f_fneg <= '1'; v_u := unsigned(-v_x) - 8; f_fm <= v_u(8 downto 0);
                else f_fneg <= '0'; f_fm <= (others => '0'); end if;
                f_sw7  <= registers_in(6)(0);
                f_sw8  <= registers_in(6)(1);
                f_hard <= registers_in(6)(2);
                f_tw   <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- phases: + flow drifts the rings out of the eyes (phase down)
                if f_fneg = '1' then f_flow <= f_flow + resize(f_spd, 9);
                else f_flow <= f_flow - resize(f_spd, 9); end if;
                f_lph <= f_lph + 72;
                f_ta <= f_ta + 151; f_tb <= f_tb - 113;
            end if;

            -- glides: ease 1/8 of the gap per field (deadband 4); one channel per four zone clocks (4..27)
            if s_zone = '1' and s_zc >= 4 and s_zc < 28 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(3)) & "000"; gl_o <= s_g(2);
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

            -- knob curves (settle long before the sequencer reads them)
            f_wl <= s_g(0)(12 downto 10);
            v_k := signed(resize(s_g(1)(12 downto 3), 11)) - 512;
            f_blue <= shift_left(resize(v_k, 12), 1);                          -- +-1024 Q9 = +-2 stripes
            v_k := signed(resize(s_g(2)(12 downto 3), 11)) - 512;
            v_k3 := resize(v_k, 13) + shift_left(resize(v_k, 13), 1);          -- 3x, +-1536
            f_red <= resize(shift_right(v_k3, 3), 10);                         -- +-192 Q7 = +-1.5 units
            f_melt <= s_g(3)(12 downto 5);
            f_d8 <= s_g(4)(12 downto 5);
            f_p8 <= s_g(5)(12 downto 5);
            if t_b8 = '1' then f_whc <= s_w(11 downto 1) - 1; else f_whc <= s_w(11 downto 1) - 3; end if;
            f_la <= f_lph;
            if f_lph(15) = '1' then f_la <= (not f_lph) + 1; f_lneg <= '1'; else f_lneg <= '0'; end if;

            -- serial products: slot p = zc/32 (1..5), phase 0 load, 1..18 step, 20 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 5 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 5 and v_ph >= 1 and v_ph <= 18 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 5 loop
                if s_zone = '1' and v_p = k and v_ph = 20 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 1 => m_a <= resize(f_p8, 20);  m_b <= resize(f_p8, 18);
                    when 2 => m_a <= resize(f_whc, 20); m_b <= resize(t_kx, 18);
                    when 3 => m_a <= resize(f_fm, 20);  m_b <= resize(f_fm, 18);
                    when 4 => m_a <= resize(f_la, 20);  m_b <= resize(to_unsigned(32768, 17) - resize(f_la, 17), 18);
                    when others => m_a <= resize(f_d8, 20); m_b <= resize(f_s9, 18);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(17 downto 1);
                m_b <= '0' & m_b(17 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1) = '1' then
                v_g3 := resize(v_pr(15 downto 0), 18) + shift_left(resize(v_pr(15 downto 0), 18), 1);   -- 3 p^2
                f_gain <= to_unsigned(64, 10) + resize(v_g3(17 downto 8), 10);
            end if;
            if q_wr(2) = '1' then f_u0 <= -signed(resize(v_pr(23 downto 0), 25)); end if;
            if q_wr(3) = '1' then f_spd <= v_pr(17 downto 11); end if;
            if q_wr(4) = '1' then
                if v_pr(29 downto 20) > 256 then f_s9 <= to_unsigned(256, 9); else f_s9 <= v_pr(28 downto 20); end if;
            end if;
            if q_wr(5) = '1' then
                if f_lneg = '1' then f_off <= -signed(resize(v_pr(16 downto 6), 12));
                else f_off <= signed(resize(v_pr(16 downto 6), 12)); end if;
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
    -- Per-line engine: at the source avid rise of line n, blend control rows
    -- j..j+2 of line n+1 into 43 knots D[i] (3 MACs each) and write the 41
    -- cells' {P0, P1, P2} into the bank line n+1 will read.  Line 0 is
    -- computed late in the vblank zone.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_jr : unsigned(5 downto 0);
        variable v_d  : signed(22 downto 0);
        variable v_dr : signed(18 downto 0);
        variable v_p0 : signed(16 downto 0);
        variable v_p2 : signed(17 downto 0);
        variable v_q0 : signed(12 downto 0);
        variable v_q2 : signed(13 downto 0);
        variable v_w  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- kick
            if s_zone = '1' and s_zc = 200 then
                e_V <= (others => '0'); e_wb <= '0'; e_st <= to_unsigned(1, 3);
            elsif data_in.avid = '1' and s_avq = '0' then
                r_bank <= e_wb; e_wb <= not e_wb;
                e_V <= e_V + resize(t_ky, 22); e_st <= to_unsigned(1, 3);
            elsif e_st /= 0 then
                if e_st = 5 then e_st <= (others => '0'); else e_st <= e_st + 1; end if;
            end if;

            -- setup: row, fraction, weights (w0 + w1 + w2 = 256)
            case to_integer(e_st) is
                when 1 =>
                    v_jr := e_V(21 downto 16);
                    if v_jr >= 44 then e_j <= to_unsigned(43, 6); e_t8 <= (others => '1');
                    else e_j <= v_jr; e_t8 <= e_V(15 downto 8); end if;
                when 2 =>
                    e_t2 <= e_t8 * e_t8;
                    e_base <= shift_left(resize(e_j, 11), 5) + shift_left(resize(e_j, 11), 3)
                              + shift_left(resize(e_j, 11), 1) + resize(e_j, 11);       -- j * 43
                when 3 =>
                    v_w := resize(shift_right(e_t2 + 256, 9), 10);
                    e_w2 <= v_w(8 downto 0);
                when 4 =>
                    e_w0 <= to_unsigned(128, 9) - resize(e_t8, 9) + e_w2;
                when 5 =>
                    e_w1 <= to_unsigned(256, 9) - e_w0 - e_w2;
                    e_iss <= '1'; e_i <= (others => '0'); e_r <= (others => '0'); e_abi <= e_base;
                when others => null;
            end case;

            -- issue: 43 knots x 3 rows
            e_tv(0) <= e_iss; e_tr(0) <= e_r; e_ti(0) <= e_i;
            if e_iss = '1' then
                case to_integer(e_r) is
                    when 0 => r_ca <= e_abi;
                    when 1 => r_ca <= e_abi + 43;
                    when others => r_ca <= e_abi + 86;
                end case;
                if e_r = 2 then
                    e_r <= (others => '0');
                    e_abi <= e_abi + 1;
                    if e_i = 42 then e_iss <= '0'; else e_i <= e_i + 1; end if;
                else
                    e_r <= e_r + 1;
                end if;
            end if;
            -- tags: 0 = address, 1 = ROM out, 2 = fabric copy + weight, 3 = products, 4 = sum
            for k in 1 to 4 loop
                e_tv(k) <= e_tv(k - 1); e_tr(k) <= e_tr(k - 1); e_ti(k) <= e_ti(k - 1);
            end loop;
            cf_q2 <= cf_q; cr_q2 <= cr_q;
            case to_integer(e_tr(1)) is
                when 0 => e_wc <= e_w0;
                when 1 => e_wc <= e_w1;
                when others => e_wc <= e_w2;
            end case;
            -- products (weight split 5 | 4 bits)
            e_fh <= signed(cf_q2) * signed('0' & e_wc(8 downto 4));
            e_fl <= resize(signed(cf_q2) * signed('0' & e_wc(3 downto 0)), 18);
            e_rh <= signed(cr_q2) * signed('0' & e_wc(8 downto 4));
            e_rl <= resize(signed(cr_q2) * signed('0' & e_wc(3 downto 0)), 14);
            e_pf <= shift_left(resize(e_fh, 22), 4) + resize(e_fl, 22);
            e_pr <= shift_left(resize(e_rh, 18), 4) + resize(e_rl, 18);
            -- accumulate over the 3 rows; D = round(sum / 64)
            e_dv <= '0';
            if e_tv(4) = '1' then
                if e_tr(4) = 0 then
                    e_af <= e_pf; e_ar <= e_pr;
                elsif e_tr(4) = 1 then
                    e_af <= e_af + e_pf; e_ar <= e_ar + e_pr;
                else
                    v_d := resize(e_af, 23) + resize(e_pf, 23) + 32;
                    e_df <= resize(shift_right(v_d, 6), 16);
                    v_dr := resize(e_ar, 19) + resize(e_pr, 19) + 32;
                    e_dr <= resize(shift_right(v_dr, 6), 12);
                    e_dv <= '1'; e_di <= e_ti(4);
                end if;
            end if;
            -- cells: P0 = (D[i-2] + D[i-1]) / 2, P1 = D[i-1] - D[i-2], P2 = (D[i-2] - 2 D[i-1] + D[i]) / 2
            t_we <= '0';
            if e_dv = '1' then
                e_f2 <= e_f1; e_f1 <= e_df;
                e_r2 <= e_r1; e_r1 <= e_dr;
                if e_di >= 2 then
                    t_we <= '1';
                    t_wa <= e_wb & (e_di(5 downto 0) - 2);
                    v_p0 := resize(e_f2, 17) + resize(e_f1, 17);
                    v_p2 := resize(e_f2, 18) - shift_left(resize(e_f1, 18), 1) + resize(e_df, 18);
                    t_wdf <= std_logic_vector(resize(shift_right(v_p0, 1), 14))
                             & std_logic_vector(resize(e_f1 - e_f2, 13))
                             & std_logic_vector(resize(shift_right(v_p2, 1), 12));
                    v_q0 := resize(e_r2, 13) + resize(e_r1, 13);
                    v_q2 := resize(e_r2, 14) - shift_left(resize(e_r1, 14), 1) + resize(e_dr, 14);
                    t_wdr <= std_logic_vector(resize(shift_right(v_q0, 1), 10))
                             & std_logic_vector(resize(e_r1 - e_r2, 10))
                             & std_logic_vector(resize(shift_right(v_q2, 1), 10));
                end if;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Beam: the field's column coordinate U (Q16 cells, 0 at the mirror axis),
    -- Osc phases
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                l_pb <= f_tb;
            elsif data_in.avid = '1' and s_avq = '0' then
                l_pb <= l_pb + resize(t_kay, 16);
            end if;
            if g_start = '1' then
                b_u <= f_u0;
                b_pa <= f_ta;
            elsif g_avid = '1' then
                b_u <= b_u + signed(resize(t_kx, 25));
                b_pa <= b_pa + resize(t_kax, 16);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Rings (block RAM): the box window {Y, U, V} and the sync delay
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
        variable v_a   : signed(16 downto 0);
        variable v_s2  : unsigned(15 downto 0);
        variable v_q4  : unsigned(7 downto 0);
        variable v_t   : signed(15 downto 0);
        variable v_ph  : unsigned(8 downto 0);
        variable v_rx  : unsigned(11 downto 0);
        variable v_g   : unsigned(13 downto 0);
        variable v_c   : unsigned(7 downto 0);
        variable v_y   : unsigned(16 downto 0);
        variable v_cs  : signed(17 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_m   : signed(18 downto 0);
        variable v_r, v_gg, v_b : unsigned(15 downto 0);
        variable v_iy, v_iu, v_iv : unsigned(9 downto 0);
        variable v_ru  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: 2-px luma average -> signed (Y - 512) / 2 (a few px early: invisible in a bend)
            s_vy(1) <= unsigned(data_in.y(9 downto 2)); s_vy(2) <= s_vy(1);
            s_vs <= signed(resize(resize(s_vy(1), 9) + resize(s_vy(2), 9), 10)) - 256;

            -- 1: |U|, Osc (the Osc pattern is smooth: used at stage 8 without re-aligning)
            if b_u < 0 then q1_au <= unsigned(resize(-b_u(24 downto 8), 16)); else q1_au <= unsigned(resize(b_u(24 downto 8), 16)); end if;
            q1_u <= b_u(24 downto 8);
            q1_osc <= resize(f_tri(b_pa), 10) + resize(f_tri(l_pb), 10);
            -- 2: column coordinate X (Faces: a second fold at +-20 cells = 480 virtual px)
            if f_tw = '1' then
                v_a := signed(resize(q1_au, 17)) - 5120;
                if v_a < 0 then q2_x <= -v_a; else q2_x <= v_a; end if;
            else
                q2_x <= q1_u;
            end if;
            -- 3: |X| for F; X - 16/24 cell for FR (its axis sits 16 virtual px right)
            if q2_x < 0 then q3_af <= unsigned(resize(-q2_x, 16)); else q3_af <= unsigned(resize(q2_x, 16)); end if;
            q3_dr <= q2_x - 171;
            -- 4: F cell index / fraction; |X - axisR|
            if q3_af(15 downto 8) > 40 then q4_if <= to_unsigned(40, 6); q4_sf <= (others => '1');
            else q4_if <= q3_af(13 downto 8); q4_sf <= q3_af(7 downto 0); end if;
            if q3_dr < 0 then q4_ar <= unsigned(resize(-q3_dr, 16)); else q4_ar <= unsigned(resize(q3_dr, 16)); end if;
            -- 5: table addresses (FR's from |X - axisR| directly); s^2
            tf_ra <= r_bank & q4_if;
            if q4_ar(15 downto 8) > 40 then tr_ra <= r_bank & to_unsigned(40, 6); q5_sr <= (others => '1');
            else tr_ra <= r_bank & q4_ar(13 downto 8); q5_sr <= q4_ar(7 downto 4); end if;
            q5_sf <= q4_sf;
            v_s2 := q4_sf * q4_sf; q5_qf <= v_s2(15 downto 8);
            -- 6: table read (block RAM)
            q6_sf <= q5_sf; q6_qf <= q5_qf; q6_sr <= q5_sr;
            v_q4 := q5_sr * q5_sr; q6_qr <= v_q4(7 downto 4);
            -- 7: fabric copy
            q7_f0 <= signed(tf_q(38 downto 25)); q7_f1 <= signed(tf_q(24 downto 12)); q7_f2 <= signed(tf_q(11 downto 0));
            q7_r0 <= signed(tr_q(29 downto 20)); q7_r1 <= signed(tr_q(19 downto 10)); q7_r2 <= signed(tr_q(9 downto 0));
            q7_sf <= q6_sf; q7_qf <= q6_qf; q7_sr <= q6_sr; q7_qr <= q6_qr;

            -- 8: products (F fraction split in nibbles; FR fraction 4 bits); source select
            q8_fah <= resize(q7_f1 * signed('0' & q7_sf(7 downto 4)), 18);
            q8_fal <= resize(q7_f1 * signed('0' & q7_sf(3 downto 0)), 18);
            q8_fbh <= resize(q7_f2 * signed('0' & q7_qf(7 downto 4)), 17);
            q8_fbl <= resize(q7_f2 * signed('0' & q7_qf(3 downto 0)), 17);
            q8_ra <= resize(q7_r1 * signed('0' & q7_sr), 15);
            q8_rb <= resize(q7_r2 * signed('0' & q7_qr), 15);
            q8_f0 <= q7_f0; q8_r0 <= q7_r0;
            if f_osc = '1' then q8_src <= q1_osc; else q8_src <= s_vs; end if;
            -- 9: product sums (P0 folded in, Q17 for F); FR = P0 + s P1 + s^2 P2 (Q7); the melt term
            q9_fa <= shift_left(resize(q8_fah, 23), 4) + resize(q8_fal, 23) + shift_left(resize(q8_f0, 23), 8);
            q9_fb <= shift_left(resize(q8_fbh, 23), 4) + resize(q8_fbl, 23);
            q9_fr <= resize(q8_r0, 12) + resize(shift_right(q8_ra, 4), 12) + resize(shift_right(q8_rb, 4), 12);
            q9_m <= q8_src * signed('0' & f_melt);
            -- 10: F (Q9 stripes); offset = breathe + melt
            q10_f <= resize(shift_right(q9_fa + q9_fb, 8), 16);
            q10_fr <= q9_fr;
            v_m := resize(f_off, 19) + resize(shift_right(q9_m, 6), 19);
            q10_om <= resize(v_m, 13);
            -- 11: offset field and key field
            q11_f <= q10_f + resize(q10_om, 16);
            q11_fr <= q10_fr + resize(f_red, 12) + resize(shift_right(q10_om, 3), 12);
            -- 12: x Rings (gain split 5 | 5 bits); envelope, ramp and key sums
            q12_gh <= resize(q11_f * signed('0' & f_gain(9 downto 5)), 22);
            q12_gl <= resize(q11_f * signed('0' & f_gain(4 downto 0)), 22);
            q12_es <= q11_f + 922;
            q12_bs <= q11_f + resize(f_blue, 16) + 512;
            q12_rx <= q11_fr + 38;
            -- 13: Fg; clamps
            q13_fg <= shift_left(resize(q12_gh, 27), 5) + resize(q12_gl, 27);
            v_t := shift_right(q12_es, 4);
            if v_t < 0 then q13_env <= (others => '0'); elsif v_t > 31 then q13_env <= (others => '1');
            else q13_env <= unsigned(v_t(4 downto 0)); end if;
            v_t := shift_right(q12_bs, 4);
            if v_t < 0 then q13_bi <= (others => '0'); elsif v_t > 255 then q13_bi <= (others => '1');
            else q13_bi <= unsigned(v_t(7 downto 0)); end if;
            if q12_rx < 0 then q13_rx <= (others => '0'); elsif q12_rx > 76 then q13_rx <= to_unsigned(76, 7);
            else q13_rx <= unsigned(q12_rx(6 downto 0)); end if;
            -- 14: stripe phase -> pulse ROM address (width & phase); ramp ROM address; red key x 27 / 8
            v_ph := unsigned(q13_fg(16 downto 8)) + f_flow;
            r_ga <= f_wl & v_ph(8 downto 2);
            r_ba <= q13_bi;
            v_rx := shift_left(resize(q13_rx, 12), 4) + shift_left(resize(q13_rx, 12), 3)
                    + shift_left(resize(q13_rx, 12), 1) + resize(q13_rx, 12);
            if v_rx(11 downto 3) > 255 then q14_r <= (others => '1'); else q14_r <= v_rx(10 downto 3); end if;
            q14_env <= q13_env;
            -- 15: ROM outputs (block RAM)
            q15_r <= q14_r; q15_env <= q14_env;
            -- 16: fabric copy; Hard edge
            if f_hard = '1' then
                if r_gq(7) = '1' then q16_pu <= (others => '1'); else q16_pu <= (others => '0'); end if;
            else
                q16_pu <= unsigned(r_gq);
            end if;
            q16_b <= unsigned(r_bq); q16_r <= q15_r; q16_env <= q15_env;
            -- 17: G = pulse x envelope
            v_g := q16_pu * (resize(q16_env, 6) + 1);
            q17_g <= v_g;
            q17_b <= q16_b; q17_r <= q16_r;
            -- 18: channel routing (S7 G <-> B, then S8 R <-> G)
            v_c := q17_g(12 downto 5);
            if f_sw7 = '1' then
                if f_sw8 = '1' then q18_r <= q17_b; q18_g <= q17_r; q18_b <= v_c;
                else q18_r <= q17_r; q18_g <= q17_b; q18_b <= v_c; end if;
            else
                if f_sw8 = '1' then q18_r <= v_c; q18_g <= q17_r; q18_b <= q17_b;
                else q18_r <= q17_r; q18_g <= v_c; q18_b <= q17_b; end if;
            end if;
            -- 19: RGB -> YUV terms, Q6 (hardware convention: U = Cr, V = Cb)
            --     Y = 66 R + 129 G + 25 B;  U = 112 R - (94 G + 18 B);  V = 112 B - (38 R + 75 G)
            v_r := resize(q18_r, 16); v_gg := resize(q18_g, 16); v_b := resize(q18_b, 16);
            q19_yrg <= resize(shift_left(v_r, 6), 17) + resize(shift_left(v_r, 1), 17)
                       + resize(shift_left(v_gg, 7), 17) + resize(v_gg, 17);
            q19_yb  <= shift_left(v_b, 4) + shift_left(v_b, 3) + v_b;
            q19_ur  <= shift_left(v_r, 7) - shift_left(v_r, 4);
            q19_ugb <= resize(shift_left(v_gg, 6), 17) + resize(shift_left(v_gg, 5), 17) - resize(shift_left(v_gg, 1), 17)
                       + resize(shift_left(v_b, 4), 17) + resize(shift_left(v_b, 1), 17);
            q19_vb  <= shift_left(v_b, 7) - shift_left(v_b, 4);
            q19_vrg <= resize(shift_left(v_r, 5), 17) + resize(shift_left(v_r, 2), 17) + resize(shift_left(v_r, 1), 17)
                       + resize(shift_left(v_gg, 6), 17) + resize(shift_left(v_gg, 3), 17) + resize(shift_left(v_gg, 1), 17)
                       + resize(v_gg, 17);
            -- 20: sums
            q20_y <= q19_yrg + resize(q19_yb, 17);
            q20_u <= signed(resize(q19_ur, 18)) - signed(resize(q19_ugb, 18));
            q20_v <= signed(resize(q19_vb, 18)) - signed(resize(q19_vrg, 18));
            -- 21: Y, U, V (10 bit)
            v_y := shift_right(q20_y, 6);
            q21_y <= to_unsigned(64, 10) + v_y(9 downto 0);
            v_cs := shift_right(q20_u, 6) + 512;
            if v_cs < 0 then q21_u <= (others => '0'); elsif v_cs > 1023 then q21_u <= (others => '1');
            else q21_u <= unsigned(v_cs(9 downto 0)); end if;
            v_cs := shift_right(q20_v, 6) + 512;
            if v_cs < 0 then q21_v <= (others => '0'); elsif v_cs > 1023 then q21_v <= (others => '1');
            else q21_v <= unsigned(v_cs(9 downto 0)); end if;

            -- 22: box sums over the ring window: 8 taps at 1080, 4 at 720p / SD (the original's
            -- band-limited, soft stripes; luma box then [1 2 1], chroma box pair).  The ring's input
            -- is forced to black/neutral in the vblank zone and the sums re-seeded there.
            s_wp <= s_wp + 1;
            if t_b8 = '1' then rb_ra <= s_wp - 1; else rb_ra <= s_wp - 5; end if;
            rs_ra <= s_wp - (C_LAT - 3);
            if s_zone = '1' then
                v_iy := (others => '0'); v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10);
            else
                v_iy := q21_y; v_iu := q21_u; v_iv := q21_v;
            end if;
            rb_wd <= std_logic_vector(v_iy) & std_logic_vector(v_iu) & std_logic_vector(v_iv);
            if s_zone = '1' and s_zc = 40 then
                s_ysum <= (others => '0');
                if t_b8 = '1' then s_usum <= to_unsigned(2048, 14); s_vsum <= to_unsigned(2048, 14);
                else s_usum <= to_unsigned(4096, 14); s_vsum <= to_unsigned(4096, 14); end if;
            else
                s_ysum <= s_ysum + unsigned(rb_wd(29 downto 20)) - unsigned(rb_q2(29 downto 20));
                s_usum <= s_usum + unsigned(rb_wd(19 downto 10)) - unsigned(rb_q2(19 downto 10));
                s_vsum <= s_vsum + unsigned(rb_wd(9 downto 0)) - unsigned(rb_q2(9 downto 0));
            end if;
            -- 23..24: luma box out; chroma box pair
            if t_b8 = '1' then q23_y <= s_ysum(11 downto 2); else q23_y <= s_ysum(12 downto 3); end if;
            s_ud <= s_usum; s_vd <= s_vsum;
            v_sc := resize(s_usum, 15) + s_ud;
            if t_b8 = '1' then q24_u <= v_sc(12 downto 3); else q24_u <= v_sc(13 downto 4); end if;
            v_sc := resize(s_vsum, 15) + s_vd;
            if t_b8 = '1' then q24_v <= v_sc(12 downto 3); else q24_v <= v_sc(13 downto 4); end if;
            -- 24..25: luma [1 2 1]
            q24_y <= q23_y; q25_y <= q24_y;
            v_sy := resize(q23_y, 12) + shift_left(resize(q24_y, 12), 1) + resize(q25_y, 12);
            q25_yf <= v_sy(11 downto 2);
            q25_u <= q24_u; q25_v <= q24_v;

            q26_u <= q25_u; q26_v <= q25_v;

            -- 27: luma toward black (white 940 -> 777); blanking gate.  (Stages: ring input 22, box
            -- sums 23, box out 24, [1 2 1] centre 26, chroma pair 24 -> 26.)
            if q25_yf > 64 then v_dl := q25_yf - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(26) = '1' then
                q27_y <= q25_yf - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                q27_u <= q26_u; q27_v <= q26_v;
            else
                q27_y <= to_unsigned(64, 10); q27_u <= to_unsigned(512, 10); q27_v <= to_unsigned(512, 10);
            end if;
            -- 28
            q28_y <= q27_y; q28_u <= q27_u; q28_v <= q27_v;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(q28_y);
    data_out.u       <= std_logic_vector(q28_u);
    data_out.v       <= std_logic_vector(q28_v);
    data_out.hsync_n <= rs_q2(2);
    data_out.vsync_n <= rs_q2(1);
    data_out.field_n <= rs_q2(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture mandala;
