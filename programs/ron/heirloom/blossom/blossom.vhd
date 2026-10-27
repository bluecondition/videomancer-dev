-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Blossom v0.5.0 -- Heirloom #22, rebuilt from the photo WIN_20230406_23_49_00.
--
-- The photo: a field of blossoms mirrored top/bottom with a 524-line repeat (2.06 per frame).  On
-- the heart rows (y = 0, 522, 1046) magenta hearts with a cleft, wrapped in red, pink, green and
-- violet bands, point right to a tip at x ~ 1155; beyond it nested chevrons and red fringe
-- "eyes" run into a flat magenta flood that fills the right third.  On the lobe rows (y = 260, 784)
-- magenta lobes sit at the left edge and a tangle of red / green / yellow-green bands fills the
-- middle.  Every colour is ONE smooth field through a colourizer whose palette shifts from the
-- heart rows to the lobe rows; the field grows to the right until the colourizer clips to magenta.
--
-- The program: the field is the fitted surface itself -- a quadratic B-spline on the folded domain
-- (x = 0..1920 virtual px, Y = 0..262 = the distance to the nearest heart row), 82 x 46 control
-- points (24 virtual px x 1/44 of the fold apart, blossom.py).  A per-line engine folds the line's
-- vertical phase (the V oscillator: triangle, or saw), blends three control rows into one row of
-- 82 knots (3 MACs per knot) and writes the 80 cells' {P0, P1, P2} into a ping-pong table for the
-- NEXT line; the pixel path evaluates F = P0 + s P1 + s^2 P2, applies the controls, indexes two
-- 256-entry palettes (heart-row / lobe-row) and blends them by the line's fold position.
--
--   K1  Rows     petal rows per frame, 1 .. 5 (2.06 = the photo)
--   K2  Bands    contour density (field gain 0.62 .. 1.62 about the flood edge)
--   K3  Garden   the tangle's colour: violet / teal lines .. the photo .. yellow-green
--   K4  Speed    bipolar (centre +-8 still): the rows roll through the frame
--   K5  Melt     how far the video (or Osc) bends the bands
--   K6  Breathe  depth of a slow LFO on the bloom: the hearts swell and shrink
--   S7  Mirror   Off / On: the left half mirrored, twin hearts point at each other
--   S8  Palette  Photo / Indigo (U and V exchanged)
--   S9  Edge     Soft / Hard (16-entry poster bands)
--   S10 Rows     Fold / Saw (the V oscillator's waveform)
--   S11 Source   Video / Osc
--   P12 BLOOM    0 % = closed buds on a green garden, 50 % = the photo, 100 % = magenta floods all
--
-- Virtual 1080-line geometry (the same picture at SD, 720p and 1080; 4:3 SD shows the central 1440
-- virtual px).  Generated avid; output conditioning (luma [1 2 1], chroma box pair [1 2 .. 2 1],
-- luma to ~777) before the blanking gate.  Latency 28 clocks in every mode.  Generated colours in
-- the hardware convention (U carries Cr, V carries Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.blossom_pkg.all;
use work.all;

architecture blossom of program_top is
    constant C_LAT : integer := 28;   -- matches processing_delay_clks
    constant C_PIV : integer := 1638; -- the flood edge, (3.4 + 3) * 256 (Q3 control-point units)

    -- per-timing-ID constants: kx = cells (24 virtual px) per pixel, Q16; vs = virtual lines per line, Q8
    function f_kx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(5461, 14);
            when 5 | 6 | 14     => return to_unsigned(4096, 14);
            when others         => return to_unsigned(2731, 14);
        end case;
    end function;
    function f_vs(t : std_logic_vector(3 downto 0)) return unsigned is
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
    function f_isshort(t : std_logic_vector(3 downto 0)) return std_logic is
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
    signal t_vs : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_kax, t_kay : unsigned(8 downto 0) := (others => '0');
    signal t_short : std_logic := '0';   -- 720p / SD: the short chroma box

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K1, K2, K3, K5, K6, P12 (Q.3), power up at the defaults
    type t_g is array(0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(528 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(460 * 8, 13), to_unsigned(384 * 8, 13), to_unsigned(512 * 8, 13));
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
    signal f_k1    : unsigned(9 downto 0) := to_unsigned(528, 10);
    signal f_kk    : unsigned(19 downto 0) := to_unsigned(278784, 20);
    signal f_base  : unsigned(16 downto 0) := to_unsigned(32054, 17);
    signal f_pinc  : unsigned(18 downto 0) := to_unsigned(32054, 19);   -- Q24 period per line
    signal f_gain  : unsigned(7 downto 0) := to_unsigned(128, 8);       -- Q7
    signal f_goff  : signed(9 downto 0) := (others => '0');
    signal f_bloom : signed(10 downto 0) := (others => '0');
    signal f_melt  : unsigned(7 downto 0) := to_unsigned(115, 8);
    signal f_depth : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal f_sm    : unsigned(8 downto 0) := (others => '0');            -- speed magnitude
    signal f_sneg  : std_logic := '0';
    signal f_spd   : unsigned(18 downto 0) := (others => '0');           -- Q24 period per field
    signal f_roll  : unsigned(23 downto 0) := (others => '0');
    signal f_lph   : unsigned(15 downto 0) := (others => '0');           -- breathe LFO phase
    signal f_la    : unsigned(15 downto 0) := (others => '0');
    signal f_lneg  : std_logic := '0';
    signal f_s9    : unsigned(8 downto 0) := (others => '0');            -- |sin| 0..256
    signal f_off   : signed(11 downto 0) := (others => '0');             -- bloom + breathe, Q3
    signal f_u0    : signed(24 downto 0) := (others => '0');             -- Q16 cells
    signal f_ta, f_tb : unsigned(15 downto 0) := (others => '0');
    signal f_mir, f_pal, f_hard, f_saw, f_osc : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- per-line engine
    signal e_ph   : unsigned(23 downto 0) := (others => '0');            -- the V oscillator, Q24 of a period
    signal e_wb   : std_logic := '0';
    signal r_bank : std_logic := '0';
    signal e_st   : unsigned(3 downto 0) := (others => '0');
    signal e_u16  : unsigned(15 downto 0) := (others => '0');            -- fold position, Q16 of a period (0..32768)
    signal e_y8   : unsigned(18 downto 0) := (others => '0');
    signal e_yc   : unsigned(13 downto 0) := (others => '0');            -- Q8 rows
    signal e_wr   : signed(10 downto 0) := (others => '0');
    signal e_wbv  : unsigned(8 downto 0) := (others => '0');            -- palette blend for the line being computed
    signal l_wb   : unsigned(8 downto 0) := (others => '0');            -- palette blend of the line on screen
    signal e_j    : unsigned(5 downto 0) := (others => '0');
    signal e_t8   : unsigned(7 downto 0) := (others => '0');
    signal e_t2   : unsigned(15 downto 0) := (others => '0');
    signal e_base : unsigned(11 downto 0) := (others => '0');
    signal e_w0, e_w1, e_w2 : unsigned(7 downto 0) := (others => '0');
    signal e_iss  : std_logic := '0';
    signal e_i    : unsigned(6 downto 0) := (others => '0');
    signal e_r    : unsigned(1 downto 0) := (others => '0');
    signal e_abi  : unsigned(11 downto 0) := (others => '0');
    signal r_ca   : unsigned(11 downto 0) := (others => '0');
    signal cp_q, cp_q2 : std_logic_vector(7 downto 0) := (others => '0');
    type t_u2a is array (natural range <>) of unsigned(1 downto 0);
    type t_u7a is array (natural range <>) of unsigned(6 downto 0);
    signal e_tv : std_logic_vector(0 to 4) := (others => '0');
    signal e_tr : t_u2a(0 to 4) := (others => (others => '0'));
    signal e_ti : t_u7a(0 to 4) := (others => (others => '0'));
    signal e_wc : unsigned(7 downto 0) := (others => '0');
    signal e_mh, e_ml : unsigned(11 downto 0) := (others => '0');
    signal e_pp : unsigned(15 downto 0) := (others => '0');
    signal e_acc : unsigned(17 downto 0) := (others => '0');
    signal e_dv : std_logic := '0';
    signal e_di : unsigned(6 downto 0) := (others => '0');
    signal e_d, e_d1, e_d2 : unsigned(10 downto 0) := (others => '0');  -- knots, Q3

    -- cell table (ping-pong by line): {P0 11 | P1 12 | P2 13}
    type t_tf is array (0 to 255) of std_logic_vector(35 downto 0);
    signal tf_ram : t_tf := (others => (others => '0'));
    signal t_we : std_logic := '0';
    signal t_wa : unsigned(7 downto 0) := (others => '0');
    signal t_wd : std_logic_vector(35 downto 0) := (others => '0');
    signal tf_ra : unsigned(7 downto 0) := (others => '0');
    signal tf_q : std_logic_vector(35 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of tf_ram : signal is true;

    -- beam
    signal b_u  : signed(24 downto 0) := (others => '0');
    signal b_pa : unsigned(15 downto 0) := (others => '0');
    signal l_pb : unsigned(15 downto 0) := (others => '0');

    -- video tap
    type t_u8v is array (natural range <>) of unsigned(7 downto 0);
    signal s_vy : t_u8v(1 to 2) := (others => to_unsigned(128, 8));
    signal s_vs : signed(9 downto 0) := (others => '0');

    -- pixel path (stage N = N registers after the beam)
    signal q1_u, q2_u : unsigned(22 downto 0) := (others => '0');
    signal q1_osc : signed(9 downto 0) := (others => '0');
    signal q3_s, q4_s, q5_s : unsigned(7 downto 0) := (others => '0');
    signal q4_s2, q5_s2 : unsigned(7 downto 0) := (others => '0');
    signal q5_p0, q6_p0, q7_p0 : unsigned(10 downto 0) := (others => '0');
    signal q5_p1 : signed(11 downto 0) := (others => '0');
    signal q5_p2 : signed(12 downto 0) := (others => '0');
    signal q6_ah, q6_al : signed(16 downto 0) := (others => '0');
    signal q6_bh, q6_bl : signed(17 downto 0) := (others => '0');
    signal q7_fa : signed(21 downto 0) := (others => '0');
    signal q7_fb : signed(22 downto 0) := (others => '0');
    signal q7_src : signed(9 downto 0) := (others => '0');
    signal q8_f  : signed(14 downto 0) := (others => '0');
    signal q8_m  : signed(18 downto 0) := (others => '0');
    signal q9_d  : signed(14 downto 0) := (others => '0');
    signal q9_om, q10_om, q11_om : signed(12 downto 0) := (others => '0');
    signal q10_gh, q10_gl : signed(20 downto 0) := (others => '0');
    signal q11_gs : signed(24 downto 0) := (others => '0');
    signal q12_fp : signed(16 downto 0) := (others => '0');
    signal q13_i  : signed(13 downto 0) := (others => '0');
    signal q13_r  : signed(13 downto 0) := (others => '0');
    signal q13_lt, q13_gt : std_logic := '0';
    signal r_la   : unsigned(7 downto 0) := (others => '0');
    signal r_lq, q16_pal : std_logic_vector(59 downto 0) := (others => '0');
    type t_s11a is array (0 to 2) of signed(10 downto 0);
    type t_u10a3 is array (0 to 2) of unsigned(9 downto 0);
    signal q17_a, q18_a : t_u10a3 := (others => (others => '0'));
    signal q17_d : t_s11a := (others => (others => '0'));
    type t_s17a is array (0 to 2) of signed(16 downto 0);
    type t_s16a is array (0 to 2) of signed(15 downto 0);
    signal q18_h : t_s17a := (others => (others => '0'));
    signal q18_l : t_s16a := (others => (others => '0'));
    type t_s19a is array (0 to 2) of signed(18 downto 0);
    signal q19_bl : t_s19a := (others => (others => '0'));
    signal q19_a, q20_px : t_u10a3 := (others => (others => '0'));
    signal q21_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q21_u, q21_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- chroma box ring (pair of box sums over the same 8-tap (4 at 720p / SD) window) and sync ring
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
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly : t_u10a(0 to 6) := (others => to_unsigned(64, 10));
    signal q24_y, q25_y, q26_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q24_u, q24_v, q25_u, q25_v, q26_u, q26_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal q27_y, q28_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q27_u, q27_v, q28_u, q28_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs and the cell table
    ---------------------------------------------------------------------------
    p_crom : process(clk)
    begin
        if rising_edge(clk) then
            cp_q <= C_BL_CP(to_integer(r_ca));
        end if;
    end process p_crom;

    p_tf : process(clk)
    begin
        if rising_edge(clk) then
            if t_we = '1' then tf_ram(to_integer(t_wa)) <= t_wd; end if;
            tf_q <= tf_ram(to_integer(tf_ra));
        end if;
    end process p_tf;

    p_prom : process(clk)
    begin
        if rising_edge(clk) then
            r_lq <= C_BL_PAL(to_integer(r_la));
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
        variable v_k5 : signed(13 downto 0);
        variable v_o  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            t_kx    <= f_kx(registers_in(8)(3 downto 0));
            t_vs    <= f_vs(registers_in(8)(3 downto 0));
            t_kax   <= f_kax(registers_in(8)(3 downto 0));
            t_kay   <= f_kay(registers_in(8)(3 downto 0));
            t_short <= f_isshort(registers_in(8)(3 downto 0));

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
                if v_x > 8 then f_sneg <= '0'; v_u := unsigned(v_x) - 8; f_sm <= v_u(8 downto 0);
                elsif v_x < -8 then f_sneg <= '1'; v_u := unsigned(-v_x) - 8; f_sm <= v_u(8 downto 0);
                else f_sneg <= '0'; f_sm <= (others => '0'); end if;
                f_mir  <= registers_in(6)(0);
                f_pal  <= registers_in(6)(1);
                f_hard <= registers_in(6)(2);
                f_saw  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- the rows roll (+ = up the screen), the breathe LFO and the Osc drift
                if f_sneg = '1' then f_roll <= f_roll - resize(f_spd, 24);
                else f_roll <= f_roll + resize(f_spd, 24); end if;
                f_lph <= f_lph + 91;
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
            f_k1 <= s_g(0)(12 downto 3);
            v_k := signed(resize(s_g(1)(12 downto 3), 11)) - 512;                 -- K2 Bands
            if v_k >= 0 then
                v_k5 := shift_left(resize(v_k, 14), 2) + resize(v_k, 14);         -- 5 v
                f_gain <= to_unsigned(128, 8) + unsigned(v_k5(12 downto 5));
            else
                v_k5 := -(shift_left(resize(v_k, 14), 1) + resize(v_k, 14));      -- 3 |v|
                f_gain <= to_unsigned(128, 8) - unsigned(v_k5(12 downto 5));
            end if;
            v_k := signed(resize(s_g(2)(12 downto 3), 11)) - 512;                 -- K3 Garden: +-192
            v_k3 := shift_left(resize(v_k, 13), 1) + resize(v_k, 13);
            f_goff <= resize(shift_right(v_k3, 3), 10);
            v_k := signed(resize(s_g(5)(12 downto 3), 11)) - 512;                 -- P12 Bloom: +-384 (1.5 units)
            v_k3 := shift_left(resize(v_k, 13), 1) + resize(v_k, 13);
            f_bloom <= resize(shift_right(v_k3, 2), 11);
            f_melt  <= s_g(3)(12 downto 5);
            f_depth <= s_g(4)(12 downto 5);
            f_la <= f_lph;
            if f_lph(15) = '1' then f_la <= (not f_lph) + 1; f_lneg <= '1'; else f_lneg <= '0'; end if;

            -- serial products: slot p = zc/32 (1..7), phase 0 load, 1..18 step, 20 store
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
                    when 1 => m_a <= resize(f_k1, 20);                m_b <= resize(f_k1, 18);
                    when 2 => m_a <= to_unsigned(62137, 20);          m_b <= resize(f_kk(19 downto 8), 18);
                    when 3 => m_a <= resize(f_base, 20);              m_b <= resize(t_vs, 18);
                    when 4 => m_a <= resize(f_sm, 20);                m_b <= resize(f_sm, 18);
                    when 5 => m_a <= resize(f_la, 20);                m_b <= resize(to_unsigned(32768, 17) - resize(f_la, 17), 18);
                    when 6 => m_a <= resize(f_depth, 20);             m_b <= resize(f_s9, 18);
                    when others => m_a <= resize(s_w(11 downto 1), 20); m_b <= resize(t_kx, 18);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(17 downto 1);
                m_b <= '0' & m_b(17 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1) = '1' then f_kk <= v_pr(19 downto 0); end if;                                  -- k1^2
            if q_wr(2) = '1' then f_base <= to_unsigned(15534, 17) + resize(v_pr(27 downto 12), 17); end if;   -- rows * 2^24 / 1080
            if q_wr(3) = '1' then f_pinc <= v_pr(26 downto 8); end if;                                -- x lines per line
            if q_wr(4) = '1' then f_spd <= resize(v_pr(17 downto 0), 19) + resize(v_pr(17 downto 2), 19); end if;
            if q_wr(5) = '1' then
                if v_pr(29 downto 20) > 256 then f_s9 <= to_unsigned(256, 9); else f_s9 <= v_pr(28 downto 20); end if;
            end if;
            if q_wr(6) = '1' then
                if f_lneg = '1' then v_o := resize(f_bloom, 12) - signed(resize(v_pr(16 downto 8), 12));
                else v_o := resize(f_bloom, 12) + signed(resize(v_pr(16 downto 8), 12)); end if;
                f_off <= v_o;
            end if;
            if q_wr(7) = '1' then f_u0 <= to_signed(2621440, 25) - signed(resize(v_pr(24 downto 0), 25)); end if;

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
    -- Per-line engine: at the source avid rise of line n, fold line n+1's
    -- vertical phase, blend control rows j..j+2 into 82 knots D[i] (3 MACs
    -- each) and write the 80 cells' {P0, P1, P2} into the bank line n+1 will
    -- read.  Line 0 is computed late in the vblank zone.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_v  : unsigned(23 downto 0);
        variable v_yc : unsigned(18 downto 0);
        variable v_w  : unsigned(9 downto 0);
        variable v_t  : unsigned(16 downto 0);
        variable v_a  : unsigned(17 downto 0);
        variable v_p0 : unsigned(11 downto 0);
        variable v_p1 : signed(11 downto 0);
        variable v_p2 : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- kick
            if s_zone = '1' and s_zc = 300 then
                e_ph <= f_roll; e_wb <= '0'; e_st <= to_unsigned(1, 4);
            elsif data_in.avid = '1' and s_avq = '0' then
                r_bank <= e_wb; e_wb <= not e_wb; l_wb <= e_wbv;
                e_ph <= e_ph + resize(f_pinc, 24); e_st <= to_unsigned(1, 4);
            elsif e_st /= 0 then
                if e_st = 8 then e_st <= (others => '0'); else e_st <= e_st + 1; end if;
            end if;

            -- setup: fold, rows, weights (w0 + w1 + w2 = 256)
            case to_integer(e_st) is
                when 1 =>
                    if f_saw = '1' then v_v := '0' & e_ph(22 downto 0);
                    elsif e_ph(23) = '0' then v_v := e_ph;
                    else v_v := (not e_ph) + 1; end if;                        -- 2^24 - ph
                    e_u16 <= v_v(23 downto 8);
                when 2 =>
                    e_y8 <= shift_left(resize(e_u16, 19), 3) + shift_left(resize(e_u16, 19), 1);
                    e_wr <= signed(resize(e_u16(15 downto 7), 11)) + resize(f_goff, 11);
                when 3 =>
                    v_yc := e_y8 + resize(e_u16, 19);
                    e_yc <= v_yc(18 downto 5);
                    if e_wr < 0 then e_wbv <= (others => '0');
                    elsif e_wr > 256 then e_wbv <= to_unsigned(256, 9);
                    else e_wbv <= unsigned(e_wr(8 downto 0)); end if;
                when 4 =>
                    if e_yc(13 downto 8) > 43 then e_j <= to_unsigned(43, 6); e_t8 <= (others => '1');
                    else e_j <= e_yc(13 downto 8); e_t8 <= e_yc(7 downto 0); end if;
                when 5 =>
                    e_t2 <= e_t8 * e_t8;
                    e_base <= shift_left(resize(e_j, 12), 6) + shift_left(resize(e_j, 12), 4)
                              + shift_left(resize(e_j, 12), 1);                     -- j * 82
                when 6 =>
                    v_t := resize(e_t2, 17) + 256;
                    e_w2 <= v_t(16 downto 9);
                when 7 =>
                    v_w := to_unsigned(128, 10) - resize(e_t8, 10) + resize(e_w2, 10);
                    e_w0 <= v_w(7 downto 0);
                when 8 =>
                    v_w := to_unsigned(256, 10) - resize(e_w0, 10) - resize(e_w2, 10);
                    e_w1 <= v_w(7 downto 0);
                    e_iss <= '1'; e_i <= (others => '0'); e_r <= (others => '0'); e_abi <= e_base;
                when others => null;
            end case;

            -- issue: 82 knots x 3 rows
            e_tv(0) <= e_iss; e_tr(0) <= e_r; e_ti(0) <= e_i;
            if e_iss = '1' then
                case to_integer(e_r) is
                    when 0 => r_ca <= e_abi;
                    when 1 => r_ca <= e_abi + 82;
                    when others => r_ca <= e_abi + 164;
                end case;
                if e_r = 2 then
                    e_r <= (others => '0');
                    e_abi <= e_abi + 1;
                    if e_i = 81 then e_iss <= '0'; else e_i <= e_i + 1; end if;
                else
                    e_r <= e_r + 1;
                end if;
            end if;
            -- tags: 0 = address, 1 = ROM out, 2 = fabric copy + weight, 3 = products, 4 = product sum
            for k in 1 to 4 loop
                e_tv(k) <= e_tv(k - 1); e_tr(k) <= e_tr(k - 1); e_ti(k) <= e_ti(k - 1);
            end loop;
            cp_q2 <= cp_q;
            case to_integer(e_tr(1)) is
                when 0 => e_wc <= e_w0;
                when 1 => e_wc <= e_w1;
                when others => e_wc <= e_w2;
            end case;
            e_mh <= unsigned(cp_q2) * e_wc(7 downto 4);
            e_ml <= unsigned(cp_q2) * e_wc(3 downto 0);
            e_pp <= shift_left(resize(e_mh, 16), 4) + resize(e_ml, 16);
            -- accumulate over the 3 rows; D = round(sum / 32)
            e_dv <= '0';
            if e_tv(4) = '1' then
                if e_tr(4) = 0 then
                    e_acc <= resize(e_pp, 18);
                elsif e_tr(4) = 1 then
                    e_acc <= e_acc + resize(e_pp, 18);
                else
                    v_a := e_acc + resize(e_pp, 18) + 16;
                    e_d <= v_a(15 downto 5);
                    e_dv <= '1'; e_di <= e_ti(4);
                end if;
            end if;
            -- cells: P0 = (D[i] + D[i+1]) / 2, P1 = D[i+1] - D[i], P2 = D[i] - 2 D[i+1] + D[i+2]
            t_we <= '0';
            if e_dv = '1' then
                e_d2 <= e_d1; e_d1 <= e_d;
                if e_di >= 2 then
                    t_we <= '1';
                    t_wa <= e_wb & (e_di - 2);
                    v_p0 := resize(e_d2, 12) + resize(e_d1, 12);
                    v_p1 := signed(resize(e_d1, 12)) - signed(resize(e_d2, 12));
                    v_p2 := signed(resize(e_d2, 13)) - shift_left(signed(resize(e_d1, 13)), 1) + signed(resize(e_d, 13));
                    t_wd <= std_logic_vector(v_p0(11 downto 1)) & std_logic_vector(v_p1) & std_logic_vector(v_p2);
                end if;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Beam: the field's column coordinate U (Q16 cells, 0 at the left edge of
    -- the 1920-px virtual frame), Osc phases
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
        variable v_u   : signed(24 downto 0);
        variable v_s2  : unsigned(15 downto 0);
        variable v_f   : signed(22 downto 0);
        variable v_m   : signed(12 downto 0);
        variable v_g   : signed(24 downto 0);
        variable v_fp  : signed(16 downto 0);
        variable v_i   : signed(13 downto 0);
        variable v_r   : signed(13 downto 0);
        variable v_ix  : unsigned(7 downto 0);
        variable v_px  : signed(11 downto 0);
        variable v_iu, v_iv, v_iy : unsigned(9 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dl  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: 2-px luma average -> signed (Y - 512) / 2 (a few px early: invisible in a bend)
            s_vy(1) <= unsigned(data_in.y(9 downto 2)); s_vy(2) <= s_vy(1);
            s_vs <= signed(resize(resize(s_vy(1), 9) + resize(s_vy(2), 9), 10)) - 256;

            -- 1: U clamped to the frame (0 .. 80 cells); Osc (two slow drifting triangles, half the video's swing)
            if b_u < 0 then q1_u <= (others => '0');
            elsif b_u > 5242879 then q1_u <= to_unsigned(5242879, 23);
            else q1_u <= unsigned(b_u(22 downto 0)); end if;
            q1_osc <= shift_right(resize(f_tri(b_pa), 10) + resize(f_tri(l_pb), 10), 1);   -- +-127: a gentle wobble
            -- 2: Mirror: the right half shows the left half reflected (twin hearts)
            if f_mir = '1' and q1_u >= 2621440 then q2_u <= to_unsigned(5242880, 23) - q1_u;
            else q2_u <= q1_u; end if;
            -- 3: cell address, fraction
            if q2_u(22 downto 16) > 79 then
                tf_ra <= r_bank & to_unsigned(79, 7); q3_s <= (others => '1');
            else
                tf_ra <= r_bank & q2_u(22 downto 16); q3_s <= q2_u(15 downto 8);
            end if;
            -- 4: table read (block RAM); s^2
            q4_s <= q3_s;
            v_s2 := q3_s * q3_s; q4_s2 <= v_s2(15 downto 8);
            -- 5: fabric copy
            q5_p0 <= unsigned(tf_q(35 downto 25)); q5_p1 <= signed(tf_q(24 downto 13)); q5_p2 <= signed(tf_q(12 downto 0));
            q5_s <= q4_s; q5_s2 <= q4_s2;
            -- 6: products (fractions split in nibbles)
            q6_ah <= resize(q5_p1 * signed('0' & q5_s(7 downto 4)), 17);
            q6_al <= resize(q5_p1 * signed('0' & q5_s(3 downto 0)), 17);
            q6_bh <= resize(q5_p2 * signed('0' & q5_s2(7 downto 4)), 18);
            q6_bl <= resize(q5_p2 * signed('0' & q5_s2(3 downto 0)), 18);
            q6_p0 <= q5_p0;
            -- 7: s P1, s^2 P2; the source (video luma or Osc)
            q7_fa <= shift_left(resize(q6_ah, 22), 4) + resize(q6_al, 22);
            q7_fb <= shift_left(resize(q6_bh, 23), 4) + resize(q6_bl, 23);
            q7_p0 <= q6_p0;
            if f_osc = '1' then q7_src <= q1_osc; else q7_src <= s_vs; end if;
            -- 8: F = P0 + s P1 + s^2 P2 (Q3 control-point units); the melt product
            v_f := signed(resize(q7_p0, 23)) + shift_right(resize(q7_fa, 23), 8) + shift_right(q7_fb, 9);
            q8_f <= resize(v_f, 15);
            q8_m <= q7_src * signed('0' & f_melt);
            -- 9: F - flood edge; offset = bloom + breathe + melt
            q9_d <= q8_f - C_PIV;
            v_m := resize(f_off, 13) + resize(shift_right(q8_m, 7), 13);
            q9_om <= v_m;
            -- 10: x Bands gain (split 4 | 4 bits)
            q10_gh <= resize(q9_d * signed('0' & f_gain(7 downto 4)), 21);
            q10_gl <= resize(q9_d * signed('0' & f_gain(3 downto 0)), 21);
            q10_om <= q9_om;
            -- 11
            q11_gs <= shift_left(resize(q10_gh, 25), 4) + resize(q10_gl, 25);
            q11_om <= q10_om;
            -- 12: F' = edge + gain (F - edge) + offset
            v_g := shift_right(q11_gs, 7);
            v_fp := to_signed(C_PIV, 17) + resize(v_g, 17) + resize(q11_om, 17);
            q12_fp <= v_fp;
            -- 13: palette entry = F' / 8; reflect below the first band
            v_i := resize(shift_right(q12_fp, 3), 14);
            q13_i <= v_i;
            q13_r <= to_signed(28, 14) - v_i;
            if v_i < 14 then q13_lt <= '1'; else q13_lt <= '0'; end if;
            if v_i > 255 then q13_gt <= '1'; else q13_gt <= '0'; end if;
            -- 14: final entry (Hard: 16-entry poster bands) -> palette address
            if q13_lt = '1' then
                if q13_r > 110 then v_ix := to_unsigned(110, 8); else v_ix := unsigned(q13_r(7 downto 0)); end if;
            elsif q13_gt = '1' then
                v_ix := (others => '1');
            else
                v_ix := unsigned(q13_i(7 downto 0));
            end if;
            if f_hard = '1' then v_ix := v_ix(7 downto 4) & "1000"; end if;
            r_la <= v_ix;
            -- 15: palette ROM (block RAM)
            -- 16: fabric copy
            q16_pal <= r_lq;
            -- 17: B - A per channel; A
            for c in 0 to 2 loop
                q17_a(c) <= unsigned(q16_pal(59 - 10 * c downto 50 - 10 * c));
                q17_d(c) <= signed(resize(unsigned(q16_pal(29 - 10 * c downto 20 - 10 * c)), 11))
                            - signed(resize(unsigned(q16_pal(59 - 10 * c downto 50 - 10 * c)), 11));
            end loop;
            -- 18: w (B - A), w split 5 | 4 bits (w = the line's fold position + Garden)
            for c in 0 to 2 loop
                q18_h(c) <= resize(q17_d(c) * signed('0' & l_wb(8 downto 4)), 17);
                q18_l(c) <= resize(q17_d(c) * signed('0' & l_wb(3 downto 0)), 16);
                q18_a(c) <= q17_a(c);
            end loop;
            -- 19: w (B - A)
            for c in 0 to 2 loop
                q19_bl(c) <= shift_left(resize(q18_h(c), 19), 4) + resize(q18_l(c), 19);   -- |w (B - A)| < 2^18
                q19_a(c) <= q18_a(c);
            end loop;
            -- 20: A + w (B - A) / 256
            for c in 0 to 2 loop
                v_px := signed(resize(q19_a(c), 12)) + resize(shift_right(q19_bl(c), 8), 12);
                if v_px < 0 then q20_px(c) <= (others => '0');
                elsif v_px > 1023 then q20_px(c) <= (others => '1');
                else q20_px(c) <= unsigned(v_px(9 downto 0)); end if;
            end loop;
            -- 21: Palette (Indigo exchanges U and V)
            q21_y <= q20_px(0);
            if f_pal = '1' then q21_u <= q20_px(2); q21_v <= q20_px(1);
            else q21_u <= q20_px(1); q21_v <= q20_px(2); end if;

            -- 22: chroma box sums over the ring window: 8 taps at 1080, 4 at 720p / SD (box pair =
            -- [1 2 2 2 2 2 2 2 1] / 16).  The ring's input is forced neutral in the vblank zone and the
            -- sums re-seeded there.  Luma rides a register chain aligned with the chroma window's centre.
            s_wp <= s_wp + 1;
            if t_short = '1' then rb_ra <= s_wp - 1; else rb_ra <= s_wp - 5; end if;
            rs_ra <= s_wp - (C_LAT - 3);
            -- (outside the generated avid -- the vblank zone and each hblank -- the input is black /
            -- neutral, so the first pixels of a line soften from black like the photo's left border)
            if s_zone = '1' or s_gav_sr(20) = '0' then
                v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10); v_iy := to_unsigned(64, 10);
            else
                v_iu := q21_u; v_iv := q21_v; v_iy := q21_y;
            end if;
            rb_wd <= std_logic_vector(v_iu) & std_logic_vector(v_iv);
            s_ly(0) <= v_iy;
            for k in 1 to 6 loop s_ly(k) <= s_ly(k - 1); end loop;
            if s_zone = '1' and s_zc = 40 then
                if t_short = '1' then s_usum <= to_unsigned(2048, 14); s_vsum <= to_unsigned(2048, 14);
                else s_usum <= to_unsigned(4096, 14); s_vsum <= to_unsigned(4096, 14); end if;
            else
                s_usum <= s_usum + unsigned(rb_wd(19 downto 10)) - unsigned(rb_q2(19 downto 10));
                s_vsum <= s_vsum + unsigned(rb_wd(9 downto 0)) - unsigned(rb_q2(9 downto 0));
            end if;
            -- 24: chroma box pair; luma [1 2 1] centred on the same pixel
            s_ud <= s_usum; s_vd <= s_vsum;
            v_sc := resize(s_usum, 15) + s_ud;
            if t_short = '1' then q24_u <= v_sc(12 downto 3); else q24_u <= v_sc(13 downto 4); end if;
            v_sc := resize(s_vsum, 15) + s_vd;
            if t_short = '1' then q24_v <= v_sc(12 downto 3); else q24_v <= v_sc(13 downto 4); end if;
            if t_short = '1' then
                v_sy := resize(s_ly(2), 12) + shift_left(resize(s_ly(3), 12), 1) + resize(s_ly(4), 12);
            else
                v_sy := resize(s_ly(4), 12) + shift_left(resize(s_ly(5), 12), 1) + resize(s_ly(6), 12);
            end if;
            q24_y <= v_sy(11 downto 2);
            -- 25, 26
            q25_y <= q24_y; q25_u <= q24_u; q25_v <= q24_v;
            q26_y <= q25_y; q26_u <= q25_u; q26_v <= q25_v;

            -- 27: luma toward black (white 940 -> 777); blanking gate
            if q26_y > 64 then v_dl := q26_y - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(26) = '1' then
                q27_y <= q26_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
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

end architecture blossom;
