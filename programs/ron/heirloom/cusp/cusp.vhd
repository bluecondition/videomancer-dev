-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cusp v0.5.0 -- Heirloom #17, rebuilt from the photo WIN_20230315_00_30_57.
--
-- The photo: rows of fine horizontal stripes (green / navy / blue above and
-- below black cores; cyan / blue / navy in the orange rows) cut by repeating
-- arrowheads that point right: black cores with a pointed ">" tip and a blue X,
-- cyan "fish" with needle and bullet tips that grow from left to right, broad
-- rainbow bands (cyan, purple, white-pink, magenta, yellow, orange) between
-- them, a red X after every reset in the orange rows, a sync strip at the left.
-- Row period 520 lines (2.08 rows), mirror axes at y = 259.5 and 519.5; the
-- orange rows (B zone, 166 lines) cut the black-core rows (A zone, 353 lines)
-- at hard seams.
--
-- The patch: a horizontal key oscillator (phase and frequency modulated by a
-- vertical cosine, frequency set per zone) is compared against a threshold
-- (H ramp minus V cosine, clipped between a floor and a ceiling): the key picks
-- the L layer (the broad rainbow) or the S layer (fine stripes / black core),
-- both functions of the vertical position only.  The key is low-passed per
-- colour channel before the channels' clip / fold, so the contours bend, rims
-- and arcs form at the key edges; after each reset the L layer's vertical gain
-- rings (the X's).  All fitted to the photo (.heirloom_work/rb_cusp; tables in
-- cusp.py).
--
--   K1  Cusps    cusps across (key-oscillator frequency) x0.4 .. x2.5
--   K2  Rows     rows down the frame x0.35 .. x2.8 (2.08 rows = the photo)
--   K3  Fish     how fast the fish grow across the frame, 0 .. 2x
--   K4  Speed    the key oscillator drifts (cusps stream right / left), bipolar
--   K5  Stripes  density of the fine stripes x0.4 .. x2.5
--   K6  Bend     how far the source (camera luma / Osc) bends the cusps
--   S7  Flip     the key ramp runs backwards: arrowheads point left
--   S8  Rows     Alternate (the photo) / Same: every row a black-core row
--   S9  Glow     Soft (the photo) / Hard: key edges without glow, rims or X's
--   S10 Palette  Photo / Swap: red and blue exchanged (amber / orange)
--   S11 Source   Video / Osc
--   P12 POINT    the vertical modulation: 0 = flat bars, 47 % = the photo,
--                100 % = deep needle spikes
--
-- Virtual 1080-line coordinates; the layout spans the width (4:3 SD shows the
-- same layout ~19 % narrower).  Generated avid; output conditioning (luma
-- [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777) before the blanking
-- gate.  Latency 32 clocks in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cusp_pkg.all;
use work.all;

architecture cusp of program_top is
    constant C_LAT : integer := 32;   -- matches processing_delay_clks

    -- fitted constants (see cusp.py / tables.json)
    constant C_PM0A : unsigned(31 downto 0) := x"C78226A0";
    constant C_PM0B : unsigned(31 downto 0) := to_unsigned(1298171636, 32);
    constant C_FQ0A : unsigned(15 downto 0) := to_unsigned(32925, 16);     -- FQ0 >> 8 (Q.24 cycles / virtual px)
    constant C_FQ0B : unsigned(15 downto 0) := to_unsigned(44154, 16);
    constant C_FL0A : integer := 1235;
    constant C_FL0B : integer := 852;
    constant C_RA0A : integer := -581;
    constant C_RA0B : integer := -5536;
    constant C_HI0A : integer := 3193;
    constant C_HI0B : integer := 3039;
    constant C_CH0A : integer := 609;      -- 2 g 2^32 (per virtual px^2), sign below
    constant C_CH0B : integer := 498;      -- negative
    constant C_F1A  : integer := 1772;     -- Q.24 per virtual px, negative
    constant C_F1B  : integer := 359;
    constant C_R1A  : integer := 11161;
    constant C_R1B  : integer := 34027;
    constant C_XS0  : integer := 253;
    constant C_UH   : integer := 901;      -- hold-strip tint phase (0.22)
    constant C_UC   : integer := 1434;     -- tint centre (0.35)
    constant C_PT0  : integer := 8372463;  -- 259.5/520 * 2^24
    constant C_ZLO  : integer := -5468565; -- A zone: [-169.5, 183.5) lines about the A centre
    constant C_ZHI  : integer := 5920263;

    -- timing classes: kx (Q.12 virtual px per px), rkx = 4096^2/kx, ys (2^24 per row period, per line)
    function f_kx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(10923, 14);
            when 5 | 6 | 14     => return to_unsigned(6144, 14);
            when others         => return to_unsigned(4096, 14);
        end case;
    end function;
    function f_rkx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(1536, 13);
            when 5 | 6 | 14     => return to_unsigned(2731, 13);
            when others         => return to_unsigned(4096, 13);
        end case;
    end function;
    function f_ys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(145188, 18);   -- 480i  (4.5 virtual lines)
            when 4          => return to_unsigned(72594, 18);    -- 480p  (2.25)
            when 8          => return to_unsigned(120990, 18);   -- 576i  (3.75)
            when 12         => return to_unsigned(60495, 18);    -- 576p  (1.875)
            when 5 | 6 | 14 => return to_unsigned(48396, 18);    -- 720p  (1.5)
            when 1 | 2 | 10 => return to_unsigned(64528, 18);    -- 1080i (2)
            when others     => return to_unsigned(32264, 18);    -- 1080p (1)
        end case;
    end function;
    function f_fast(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 | 5 | 6 | 14 => return '1';
            when others                      => return '0';
        end case;
    end function;
    -- K * d partial pair: (k(i) ? d << i : 0) + (k(i+1) ? d << (i+1) : 0)
    function f_pp(d : signed(13 downto 0); k : unsigned(6 downto 0); i : integer) return signed is
        variable a, b : signed(19 downto 0);
    begin
        if k(i) = '1' then a := shift_left(resize(d, 20), i); else a := (others => '0'); end if;
        if k(i + 1) = '1' then b := shift_left(resize(d, 20), i + 1); else b := (others => '0'); end if;
        return a + b;
    end function;
    -- soft-key step: d * (2^-a + 2^-b), one shift less when fast
    function f_st(d : signed(13 downto 0); a, b : integer; fast : std_logic) return signed is
    begin
        if fast = '1' then return shift_right(d, a - 1) + shift_right(d, b - 1); end if;
        return shift_right(d, a) + shift_right(d, b);
    end function;

    -- timing
    signal t_kx  : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal t_rkx : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal t_ys  : unsigned(17 downto 0) := to_unsigned(32264, 18);
    signal t_fast : std_logic := '0';   -- SD and 720p: soft keys one shift faster

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls: glides K1 K2 K3 K5 K6 P12 (Q.3)
    type t_g is array(0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(512 * 8, 13), to_unsigned(300 * 8, 13), to_unsigned(481 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 20) := (others => '0');
    signal s_a4 : unsigned(8 downto 0) := (others => '0');
    signal s_s4 : std_logic := '0';

    -- serial multiplier (per field)
    signal m_a  : unsigned(23 downto 0) := (others => '0');
    signal m_b  : unsigned(17 downto 0) := (others => '0');
    signal m_h  : unsigned(23 downto 0) := (others => '0');
    signal m_l  : unsigned(17 downto 0) := (others => '0');

    -- per-field values
    signal f_m1, f_m2, f_m5, f_m12 : unsigned(9 downto 0) := to_unsigned(256, 10);   -- Q.8
    signal f_m3   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_m6   : unsigned(7 downto 0) := to_unsigned(52, 8);
    signal f_sq   : unsigned(17 downto 0) := (others => '0');
    signal f_sp   : unsigned(26 downto 0) := (others => '0');
    signal f_incs : unsigned(11 downto 0) := to_unsigned(256, 12);    -- m1 * kx, Q.8
    signal f_inc2 : unsigned(17 downto 0) := to_unsigned(4096, 18);   -- INCS^2 Q.12
    signal f_cha, f_chb : signed(17 downto 0) := (others => '0');
    signal f_f1a, f_f1b : signed(17 downto 0) := (others => '0');
    signal f_r1ak, f_r1bk : unsigned(17 downto 0) := (others => '0');
    signal f_r1a, f_r1b : unsigned(19 downto 0) := (others => '0');
    signal f_step : unsigned(23 downto 0) := to_unsigned(32264, 24);
    signal f_ptop : unsigned(23 downto 0) := (others => '0');
    signal f_dr   : unsigned(31 downto 0) := (others => '0');
    signal f_neg  : std_logic := '0';
    signal f_flip, f_same, f_hard, f_swap, f_osc : std_logic := '0';
    signal f_ot   : unsigned(15 downto 0) := (others => '0');
    signal f_ot2  : unsigned(15 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- knot ROM / L ROM / env ROM
    signal r_ka  : unsigned(9 downto 0) := (others => '0');
    signal r_kq, r_kq2 : std_logic_vector(15 downto 0) := (others => '0');
    signal r_la  : unsigned(7 downto 0) := (others => '0');
    signal r_lq, r_lq2 : std_logic_vector(47 downto 0) := (others => '0');
    signal r_ea  : unsigned(7 downto 0) := (others => '0');
    signal r_eq, r_eq2 : std_logic_vector(8 downto 0) := (others => '0');

    -- per-line engine (computes the NEXT line) and committed line values
    signal le_st  : unsigned(4 downto 0) := (others => '0');
    signal le_op  : unsigned(3 downto 0) := (others => '0');
    signal le_P   : unsigned(23 downto 0) := (others => '0');
    signal le_dA, le_dz : unsigned(23 downto 0) := (others => '0');
    signal le_zA  : std_logic := '1';
    signal le_k   : unsigned(5 downto 0) := (others => '0');
    signal le_f   : unsigned(7 downto 0) := (others => '0');
    signal le_v0, le_v : signed(16 downto 0) := (others => '0');
    signal lm_a, lm_acc : signed(27 downto 0) := (others => '0');
    signal le_kick : std_logic := '0';
    signal lm_b   : unsigned(15 downto 0) := (others => '0');
    signal lm_n   : unsigned(4 downto 0) := (others => '0');
    signal n_phi, n_inc : unsigned(31 downto 0) := (others => '0');
    signal n_f0, n_r0 : signed(27 downto 0) := (others => '0');
    signal n_hi   : signed(15 downto 0) := (others => '0');
    signal n_xs   : unsigned(10 downto 0) := (others => '0');
    signal n_sr, n_sb : signed(12 downto 0) := (others => '0');
    signal n_sg   : signed(12 downto 0) := (others => '0');
    signal n_dz   : unsigned(9 downto 0) := (others => '0');
    signal n_zA   : std_logic := '1';
    signal n_oy   : unsigned(15 downto 0) := (others => '0');
    signal l_hi   : signed(15 downto 0) := (others => '0');
    signal l_xs   : unsigned(10 downto 0) := (others => '0');
    signal l_sr, l_sb, l_sg : signed(12 downto 0) := (others => '0');
    signal l_dz   : unsigned(9 downto 0) := (others => '0');
    signal l_zA   : std_logic := '1';
    signal l_oy   : unsigned(15 downto 0) := (others => '0');

    -- beam (DDAs)
    signal b_phi, b_inc : unsigned(31 downto 0) := (others => '0');
    signal b_ch   : signed(17 downto 0) := (others => '0');
    signal b_f, b_r : signed(27 downto 0) := (others => '0');
    signal b_f1   : signed(17 downto 0) := (others => '0');
    signal b_r1   : unsigned(19 downto 0) := (others => '0');
    signal b_x    : unsigned(11 downto 0) := (others => '0');
    signal b_ox   : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    signal p1_u   : unsigned(11 downto 0) := (others => '0');
    signal p1_f, p1_r : signed(15 downto 0) := (others => '0');
    signal p1_xs  : std_logic := '0';
    signal p1_yc  : signed(10 downto 0) := (others => '0');
    signal p1_tx, p1_ty : unsigned(9 downto 0) := (others => '0');
    signal p2_u   : unsigned(11 downto 0) := (others => '0');
    signal p2_bd  : signed(18 downto 0) := (others => '0');
    signal p2_f, p2_t1 : signed(15 downto 0) := (others => '0');
    signal p2_xs  : std_logic := '0';
    signal p3_u   : unsigned(11 downto 0) := (others => '0');
    signal p3_th  : signed(15 downto 0) := (others => '0');
    signal p3_xs  : std_logic := '0';
    signal p4_u, p4_th : unsigned(11 downto 0) := (others => '0');
    signal p4_xs  : std_logic := '0';
    signal p5_key, p6_key : std_logic := '1';
    signal p5_u   : unsigned(11 downto 0) := (others => '0');
    signal p5_xs  : std_logic := '0';
    signal p6_dx  : unsigned(10 downto 0) := (others => '1');
    signal p9_de  : unsigned(17 downto 0) := (others => '0');
    type t_u12a is array (natural range <>) of unsigned(11 downto 0);
    type t_sla  is array (natural range <>) of std_logic;
    type t_u6a is array (natural range <>) of unsigned(5 downto 0);
    signal pu : t_u6a(6 to 13) := (others => (others => '0'));
    signal pxs : t_sla(6 to 13) := (others => '0');
    signal pk  : t_sla(6 to 14) := (others => '1');
    signal p13_pr : signed(14 downto 0) := (others => '0');
    signal p13_lr : signed(10 downto 0) := (others => '0');
    signal p13_lg : unsigned(9 downto 0) := (others => '0');
    signal p13_lb : unsigned(8 downto 0) := (others => '0');
    signal p14_lr, p14_lb : signed(12 downto 0) := (others => '0');
    signal p14_lg : signed(12 downto 0) := (others => '0');
    signal p15_dr, p15_dg, p15_db : signed(13 downto 0) := (others => '0');
    signal k1r, k2r, k1g, k2g, k1b, k2b : signed(13 downto 0) := to_signed(4095, 14);
    signal p15_kr, p15_kg, p15_kb : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal p16_ar, p16_br, p16_cr, p16_ag, p16_bg, p16_cg, p16_ab, p16_bb, p16_cb : signed(19 downto 0) := (others => '0');
    signal p17_r, p17_g, p17_b : signed(13 downto 0) := (others => '0');
    signal p18_r, p18_b : unsigned(7 downto 0) := (others => '0');
    signal p18_t : unsigned(7 downto 0) := (others => '0');
    signal p18_n : std_logic := '0';
    signal p19_r, p19_g, p19_b : unsigned(7 downto 0) := (others => '0');
    signal p20_y, p20_u, p20_v : signed(13 downto 0) := (others => '0');
    signal p21_y, p21_u, p21_v : signed(13 downto 0) := (others => '0');
    signal p22_y, p22_u, p22_v : signed(12 downto 0) := (others => '0');
    signal p23_y, p23_u, p23_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p24_y, p24_u, p24_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p_zA : t_sla(1 to 15) := (others => '1');
    signal p_ky : std_logic := '0';

    -- output conditioning
    -- delay rings in block RAM: syncs by C_LAT, chroma by 8 (box filter); read = write pointer - (delay - 2)
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    type t_r10 is array (0 to 255) of std_logic_vector(9 downto 0);
    signal rs_ram : t_r4 := (others => "1110");
    signal ru_ram, rv_ram : t_r10 := (others => "1000000000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of rs_ram : signal is true;
    attribute no_rw_check of ru_ram : signal is true;
    attribute no_rw_check of rv_ram : signal is true;
    signal rg_wp  : unsigned(7 downto 0) := (others => '0');
    signal rs_ra, ru_ra : unsigned(7 downto 0) := (others => '0');
    signal rs_q, rs_q2 : std_logic_vector(3 downto 0) := "1110";
    signal ru_q, rv_q, ru_q2, rv_q2 : std_logic_vector(9 downto 0) := "1000000000";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y25, s_y26, s_y27, s_y28, s_y29, s_y30x, s_yf30 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u30, s_v30 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y31, s_y32 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered address and output, then a fabric copy)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_kq <= C_KN(to_integer(r_ka));
            r_lq <= C_LR(to_integer(r_la));
            r_eq <= C_ENV(to_integer(r_ea));
            r_kq2 <= r_kq;
            r_lq2 <= r_lq;
            r_eq2 <= r_eq;
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Delay rings (one write statement each, collision-free: read 30 / 6 behind)
    ---------------------------------------------------------------------------
    p_ringw : process(clk)
    begin
        if rising_edge(clk) then
            rs_ram(to_integer(rg_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & '0';
            ru_ram(to_integer(rg_wp)) <= std_logic_vector(p24_u);
            rv_ram(to_integer(rg_wp)) <= std_logic_vector(p24_v);
        end if;
    end process p_ringw;
    p_ringr : process(clk)
    begin
        if rising_edge(clk) then
            rs_q <= rs_ram(to_integer(rs_ra));
            ru_q <= ru_ram(to_integer(ru_ra));
            rv_q <= rv_ram(to_integer(ru_ra));
        end if;
    end process p_ringr;
    p_ringa : process(clk)
        variable v_wp : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            v_wp := rg_wp + 1;
            rg_wp <= v_wp;
            rs_ra <= v_wp - (C_LAT - 2);   -- address used in the next clock
            ru_ra <= v_wp - 6;
            rs_q2 <= rs_q; ru_q2 <= ru_q; rv_q2 <= rv_q;
        end if;
    end process p_ringa;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(24 downto 0);
        variable v_pr : unsigned(41 downto 0);
        variable v_g  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            t_kx  <= f_kx(registers_in(8)(3 downto 0));
            t_rkx <= f_rkx(registers_in(8)(3 downto 0));
            t_ys  <= f_ys(registers_in(8)(3 downto 0));
            t_fast <= f_fast(registers_in(8)(3 downto 0));
            -- threshold slopes per virtual px scaled by kx (class constants): F1 = -1772, 359; R1 = 11161, 34027
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => f_f1a <= to_signed(-4725, 18); f_f1b <= to_signed(957, 18);
                                       f_r1ak <= to_unsigned(29763, 18); f_r1bk <= to_unsigned(90741, 18);
                when 5 | 6 | 14     => f_f1a <= to_signed(-2658, 18); f_f1b <= to_signed(538, 18);
                                       f_r1ak <= to_unsigned(16741, 18); f_r1bk <= to_unsigned(51040, 18);
                when others         => f_f1a <= to_signed(-1772, 18); f_f1b <= to_signed(359, 18);
                                       f_r1ak <= to_unsigned(11161, 18); f_r1bk <= to_unsigned(34027, 18);
            end case;

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
                -- K4 Speed: bipolar, deadband 16
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 16 then s_s4 <= '0'; v_u := unsigned(v_x) - 16; s_a4 <= v_u(8 downto 0);
                elsif v_x < -16 then s_s4 <= '1'; v_u := unsigned(-v_x) - 16; s_a4 <= v_u(8 downto 0);
                else s_s4 <= '0'; s_a4 <= (others => '0'); end if;
                f_flip <= registers_in(6)(0);
                f_same <= registers_in(6)(1);
                f_hard <= registers_in(6)(2);
                f_swap <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- drift (last field's speed)
                if f_neg = '1' then f_dr <= f_dr + resize(f_sp, 32);
                else f_dr <= f_dr - resize(f_sp, 32); end if;
                f_neg <= s_s4;
                f_ot  <= f_ot + 197;
                f_ot2 <= f_ot2 + 131;
            end if;

            -- glides: K1, K2, K3, K5, K6, P12 ease 1/8 of the gap per field (deadband 4); one channel per
            -- four zone clocks (4..27): select, difference, ease, write
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

            -- serial products: slot p = zc/32 (1..20), phase 0 load, 1..18 step, 20 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 20 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 20 and v_ph >= 1 and v_ph <= 18 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 20 loop
                if s_zone = '1' and v_p = k and v_ph = 20 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    -- knob curves (two segments about the centre, Q.8)
                    when 1  => v_g := s_g(0)(12 downto 3);
                               if v_g < 512 then m_a <= resize(v_g, 24); m_b <= to_unsigned(154, 18);
                               else m_a <= resize(v_g - 512, 24); m_b <= to_unsigned(384, 18); end if;
                    when 2  => v_g := s_g(1)(12 downto 3);
                               if v_g < 512 then m_a <= resize(v_g, 24); m_b <= to_unsigned(166, 18);
                               else m_a <= resize(v_g - 512, 24); m_b <= to_unsigned(461, 18); end if;
                    when 3  => v_g := s_g(3)(12 downto 3);
                               if v_g < 512 then m_a <= resize(v_g, 24); m_b <= to_unsigned(154, 18);
                               else m_a <= resize(v_g - 512, 24); m_b <= to_unsigned(384, 18); end if;
                    when 4  => v_g := s_g(5)(12 downto 3);
                               if v_g < 481 then m_a <= resize(v_g, 24); m_b <= to_unsigned(545, 18);
                               else m_a <= resize(v_g - 481, 24); m_b <= to_unsigned(1252, 18); end if;
                    when 5  => m_a <= resize(s_g(4)(12 downto 3), 24); m_b <= to_unsigned(179, 18);
                    when 6  => m_a <= resize(s_a4, 24);  m_b <= resize(s_a4, 18);
                    when 7  => m_a <= resize(f_sq, 24);  m_b <= to_unsigned(349, 18);
                    when 8  => m_a <= resize(f_m1, 24);  m_b <= resize(t_kx, 18);
                    when 9  => m_a <= resize(f_incs, 24); m_b <= resize(f_incs, 18);
                    when 10 => m_a <= to_unsigned(C_CH0A, 24); m_b <= f_inc2;
                    when 11 => m_a <= to_unsigned(C_CH0B, 24); m_b <= f_inc2;
                    when 15 => m_a <= resize(f_r1ak, 24);      m_b <= resize(f_m3, 18);
                    when 17 => m_a <= resize(f_r1bk, 24);      m_b <= resize(f_m3, 18);
                    when 18 => m_a <= resize(t_ys, 24);        m_b <= resize(f_m2, 18);
                    when 19 => m_a <= to_unsigned(C_PT0, 24);  m_b <= resize(f_m2, 18);
                    when others => m_a <= (others => '0');     m_b <= (others => '0');
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 25) + resize(m_a, 25); else v_s := resize(m_h, 25); end if;
                m_h <= v_s(24 downto 1);
                m_l <= v_s(0) & m_l(17 downto 1);
                m_b <= '0' & m_b(17 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1) = '1' then
                if s_g(0)(12 downto 3) < 512 then f_m1 <= to_unsigned(102, 10) + resize(v_pr(18 downto 9), 10);
                else f_m1 <= to_unsigned(256, 10) + resize(v_pr(18 downto 9), 10); end if;
            end if;
            if q_wr(2) = '1' then
                if s_g(1)(12 downto 3) < 512 then f_m2 <= to_unsigned(90, 10) + resize(v_pr(18 downto 9), 10);
                else f_m2 <= to_unsigned(256, 10) + resize(v_pr(18 downto 9), 10); end if;
                f_m3 <= resize(s_g(2)(12 downto 4), 10);
            end if;
            if q_wr(3) = '1' then
                if s_g(3)(12 downto 3) < 512 then f_m5 <= to_unsigned(102, 10) + resize(v_pr(18 downto 9), 10);
                else f_m5 <= to_unsigned(256, 10) + resize(v_pr(18 downto 9), 10); end if;
            end if;
            if q_wr(4) = '1' then
                if s_g(5)(12 downto 3) < 481 then f_m12 <= resize(v_pr(19 downto 10), 10);
                else f_m12 <= to_unsigned(256, 10) + resize(v_pr(19 downto 10), 10); end if;
            end if;
            if q_wr(5) = '1' then f_m6 <= v_pr(17 downto 10); end if;
            if q_wr(6) = '1' then f_sq <= v_pr(17 downto 0); end if;
            if q_wr(7) = '1' then f_sp <= v_pr(26 downto 0); end if;
            if q_wr(8) = '1' then f_incs <= v_pr(23 downto 12); end if;
            if q_wr(9) = '1' then f_inc2 <= v_pr(21 downto 4); end if;
            if q_wr(10) = '1' then f_cha <= signed(v_pr(29 downto 12)); end if;
            if q_wr(11) = '1' then f_chb <= -signed(v_pr(29 downto 12)); end if;
            if q_wr(15) = '1' then f_r1a <= v_pr(27 downto 8); end if;
            if q_wr(17) = '1' then f_r1b <= v_pr(27 downto 8); end if;
            if q_wr(18) = '1' then f_step <= v_pr(31 downto 8); end if;
            if q_wr(19) = '1' then f_ptop <= to_unsigned(0, 24) - v_pr(31 downto 8); end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
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
    -- Per-line engine.  Kicked at the source avid rise (and for line 0 late in
    -- the vblank zone); computes the NEXT line: line phase, zone, distance,
    -- then nine piecewise-linear table reads (two knots, the interpolation
    -- and a control scaling on one serial multiplier each).
    --   op 0 PMD -> phase start   1 FQD -> phase increment   2 FLD -> floor
    --   op 3 RAD -> ramp start    4 HID -> ceiling           5 XS -> sync strip
    --   op 6 aSR  7 aSG (x Stripes, + B offset)  8 aSB  (S layer over d_A)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_qs  : signed(23 downto 0);
        variable v_t   : unsigned(3 downto 0);
        variable v_k   : unsigned(5 downto 0);
        variable v_f   : unsigned(7 downto 0);
        variable v_x   : signed(16 downto 0);
        variable v_ph  : signed(27 downto 0);
        variable v_w   : signed(17 downto 0);
        variable v_c   : integer range -8192 to 8191;
        variable v_c32 : unsigned(31 downto 0);
        variable v_c18 : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 680 then
                le_P <= f_ptop; n_oy <= to_unsigned(0, 16) - f_ot2;
            end if;

            if s_zone = '1' and s_zc = 699 then le_kick <= '1'; else le_kick <= '0'; end if;
            if le_kick = '1' then
                le_st <= to_unsigned(2, 5);
            elsif g_start = '1' then
                -- this line's values were taken (p_beam loads the DDAs from n_*): commit, compute the next line
                l_hi <= n_hi;
                l_xs <= n_xs; l_sr <= n_sr; l_sg <= n_sg; l_sb <= n_sb; l_dz <= n_dz; l_zA <= n_zA;
                l_oy <= n_oy;
                n_oy <= n_oy + resize(t_ys(17 downto 9), 16);
                le_st <= to_unsigned(1, 5);
            else
                case to_integer(le_st) is
                    when 1 =>
                        le_P <= le_P + f_step;
                        le_st <= le_st + 1;
                    when 2 =>
                        v_qs := signed(le_P);
                        if v_qs(23) = '1' then le_dA <= unsigned(-v_qs); else le_dA <= le_P; end if;
                        if (v_qs >= to_signed(C_ZLO, 24) and v_qs < to_signed(C_ZHI, 24)) or f_same = '1' then
                            le_zA <= '1';
                        else
                            le_zA <= '0';
                        end if;
                        le_st <= le_st + 1;
                    when 3 =>
                        if le_zA = '1' then le_dz <= le_dA; else le_dz <= to_unsigned(8388608, 24) - le_dA; end if;
                        le_op <= (others => '0');
                        le_st <= le_st + 1;
                    -- table op: knots, interpolation, scaling
                    when 4 =>
                        case to_integer(le_op) is
                            when 0 => v_t := "000" & not le_zA;
                            when 1 => v_t := "001" & not le_zA;
                            when 2 => v_t := "010" & not le_zA;
                            when 3 => v_t := "011" & not le_zA;
                            when 4 => v_t := "100" & not le_zA;
                            when 5 => v_t := "1010";
                            when 6 => v_t := "1011";
                            when 7 => v_t := "1100";
                            when others => v_t := "1101";
                        end case;
                        if le_op >= 6 then
                            v_k := le_dA(23 downto 18); v_f := le_dA(17 downto 10);
                        else
                            v_k := le_dz(23 downto 18); v_f := le_dz(17 downto 10);
                        end if;
                        le_k <= v_k; le_f <= v_f;
                        r_ka <= v_t & v_k;
                        le_st <= le_st + 1;
                    when 5 =>
                        r_ka <= r_ka + 1;
                        le_st <= le_st + 1;
                    when 6 =>
                        le_st <= le_st + 1;
                    when 7 =>
                        le_v0 <= resize(signed(r_kq2), 17);
                        le_st <= le_st + 1;
                    when 8 =>
                        lm_a <= resize(resize(signed(r_kq2), 17) - le_v0, 28);
                        lm_b <= resize(le_f, 16); lm_n <= to_unsigned(8, 5); lm_acc <= (others => '0');
                        le_st <= le_st + 1;
                    when 9 | 11 | 13 =>
                        if lm_b(0) = '1' then lm_acc <= lm_acc + lm_a; end if;
                        lm_a <= shift_left(lm_a, 1); lm_b <= '0' & lm_b(15 downto 1);
                        if lm_n = 1 then le_st <= le_st + 1; end if;
                        lm_n <= lm_n - 1;
                    when 10 =>
                        -- interpolated value; second multiply
                        v_x := le_v0 + resize(shift_right(lm_acc, 8), 17);
                        le_v <= v_x;
                        lm_acc <= (others => '0');
                        case to_integer(le_op) is
                            when 1 =>
                                -- (FQ0 >> 8) + FQD/4  times INCS
                                if le_zA = '1' then v_w := signed(resize(C_FQ0A, 18)); else v_w := signed(resize(C_FQ0B, 18)); end if;
                                v_w := v_w + resize(shift_right(v_x, 2), 18);
                                lm_a <= resize(v_w, 28); lm_b <= resize(f_incs, 16); lm_n <= to_unsigned(12, 5);
                            when 5 =>
                                lm_a <= resize(v_x - C_XS0, 28); lm_b <= resize(f_m12, 16); lm_n <= to_unsigned(10, 5);
                            when 6 | 8 =>
                                lm_a <= resize(v_x, 28); lm_b <= to_unsigned(1, 16); lm_n <= to_unsigned(1, 5);
                            when 7 =>
                                lm_a <= resize(v_x, 28); lm_b <= resize(f_m5, 16); lm_n <= to_unsigned(10, 5);
                            when others =>
                                lm_a <= resize(v_x, 28); lm_b <= resize(f_m12, 16); lm_n <= to_unsigned(10, 5);
                        end case;
                        le_st <= le_st + 1;
                    when 12 =>
                        -- store (or third multiply for XS)
                        lm_acc <= (others => '0');
                        case to_integer(le_op) is
                            when 0 =>
                                v_ph := shift_right(lm_acc, 8);
                                if le_zA = '1' then v_c32 := C_PM0A; else v_c32 := C_PM0B; end if;
                                n_phi <= (v_c32 + f_dr) + unsigned(v_ph(15 downto 0) & x"0000");
                                le_st <= to_unsigned(15, 5);
                            when 1 =>
                                if le_zA = '1' then v_c18 := f_cha; else v_c18 := f_chb; end if;
                                n_inc <= unsigned(resize(lm_acc, 32) + resize(shift_right(v_c18, 1), 32));
                                le_st <= to_unsigned(15, 5);
                            when 2 =>
                                if le_zA = '1' then v_c := C_FL0A; else v_c := C_FL0B; end if;
                                n_f0 <= shift_left(resize(shift_right(lm_acc, 8), 28) + v_c, 12);
                                le_st <= to_unsigned(15, 5);
                            when 3 =>
                                if le_zA = '1' then v_c := C_RA0A; else v_c := C_RA0B; end if;
                                n_r0 <= shift_left(resize(shift_right(lm_acc, 8), 28) + v_c, 12);
                                le_st <= to_unsigned(15, 5);
                            when 4 =>
                                if le_zA = '1' then v_c := C_HI0A; else v_c := C_HI0B; end if;
                                n_hi <= resize(shift_right(lm_acc, 8), 16) + v_c;
                                le_st <= to_unsigned(15, 5);
                            when 5 =>
                                lm_a <= resize(shift_right(lm_acc, 8), 28) + C_XS0;
                                lm_b <= resize(t_rkx, 16); lm_n <= to_unsigned(13, 5);
                                le_st <= le_st + 1;
                            when 6 =>
                                n_sr <= resize(shift_right(lm_acc, 4), 13);
                                le_st <= to_unsigned(15, 5);
                            when 7 =>
                                if le_zA = '1' then n_sg <= resize(shift_right(lm_acc, 8), 13);
                                else n_sg <= resize(shift_right(lm_acc, 8), 13) - 512; end if;
                                le_st <= to_unsigned(15, 5);
                            when others =>
                                n_sb <= resize(shift_right(lm_acc, 4), 13);
                                le_st <= to_unsigned(15, 5);
                        end case;
                    when 14 =>
                        if le_zA = '1' then n_xs <= unsigned(lm_acc(22 downto 12)); else n_xs <= (others => '0'); end if;
                        le_st <= le_st + 1;
                    when 15 =>
                        -- next op
                        if le_op = 8 then
                            n_dz <= le_dz(23 downto 14); n_zA <= le_zA;
                            le_st <= (others => '0');
                        else
                            le_op <= le_op + 1;
                            le_st <= to_unsigned(4, 5);
                        end if;
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: key-oscillator DDA (2nd order), threshold DDAs, Osc triangle
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_phi <= n_phi; b_inc <= n_inc; b_f <= n_f0; b_r <= n_r0;
                if n_zA = '1' then b_ch <= f_cha; b_f1 <= f_f1a; b_r1 <= f_r1a;
                else b_ch <= f_chb; b_f1 <= f_f1b; b_r1 <= f_r1b; end if;
                b_x <= (others => '0');
                b_ox <= f_ot;
            elsif g_avid = '1' then
                b_phi <= b_phi + b_inc;
                b_inc <= unsigned(signed(b_inc) + resize(b_ch, 32));
                b_f <= b_f + resize(b_f1, 28);
                b_r <= b_r + signed(resize(b_r1, 28));
                b_x <= b_x + 1;
                b_ox <= b_ox + resize(t_kx(13 downto 7), 16);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_bd  : signed(21 downto 0);
        variable v_u   : unsigned(11 downto 0);
        variable v_th  : signed(15 downto 0);
        variable v_k   : std_logic;
        variable v_dx  : unsigned(11 downto 0);
        variable v_de  : unsigned(9 downto 0);
        variable v_la  : unsigned(9 downto 0);
        variable v_uu  : signed(6 downto 0);
        variable v_t   : signed(13 downto 0);
        variable v_d   : signed(13 downto 0);
        variable v_s   : signed(13 downto 0);
        variable v_tr  : unsigned(8 downto 0);
        variable v_gg  : unsigned(15 downto 0);
        variable v_r, v_b : unsigned(7 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_oc  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: phase, thresholds, sync strip, source
            p1_u <= b_phi(31 downto 20);
            p1_f <= b_f(27 downto 12); p1_r <= b_r(27 downto 12);
            if resize(b_x, 11) < l_xs and b_x(11) = '0' then p1_xs <= '1'; else p1_xs <= '0'; end if;
            p1_yc <= signed(resize(unsigned(data_in.y), 11)) - 512;
            if b_ox(15) = '1' then p1_tx <= b_ox(15 downto 6) - 512; else p1_tx <= 512 - b_ox(15 downto 6); end if;
            if l_oy(15) = '1' then p1_ty <= l_oy(15 downto 6) - 512; else p1_ty <= 512 - l_oy(15 downto 6); end if;
            p_zA(1) <= l_zA;

            -- 2: bend = source * Bend; ceiling clip of the ramp
            if f_osc = '1' then
                v_oc := signed(resize(shift_right(resize(p1_tx, 12) + resize(p1_ty, 12), 1), 12)) - 256;
            else
                v_oc := resize(p1_yc, 12);
            end if;
            p2_bd <= resize(resize(shift_right(v_oc, 4), 8) * signed('0' & f_m6(7 downto 2)), 19);
            p2_u <= p1_u;
            if p1_r < l_hi then p2_t1 <= p1_r; else p2_t1 <= l_hi; end if;
            p2_f <= p1_f; p2_xs <= p1_xs;

            -- 3: bent phase; floor
            v_bd := resize(p2_bd, 22);
            p3_u <= p2_u + unsigned(v_bd(11 downto 0));
            if p2_f > p2_t1 then p3_th <= p2_f; else p3_th <= p2_t1; end if;
            p3_xs <= p2_xs;

            -- 4: flip; threshold to 12 bits
            if f_flip = '1' then p4_u <= not p3_u; else p4_u <= p3_u; end if;
            v_th := p3_th;
            if v_th < 0 then p4_th <= (others => '0');
            elsif v_th > 4095 then p4_th <= (others => '1');
            else p4_th <= unsigned(v_th(11 downto 0)); end if;
            p4_xs <= p3_xs;

            -- 5: key (1 = L layer)
            if p_zA(4) = '1' then
                if p4_u < p4_th or p4_xs = '1' then v_k := '1'; else v_k := '0'; end if;
            else
                if (not p4_u) < p4_th then v_k := '0'; else v_k := '1'; end if;
            end if;
            p5_key <= v_k; p5_u <= p4_u; p5_xs <= p4_xs;

            -- 6: virtual distance since the last S -> L edge (Q.4), env address
            if s_gav_sr(4) = '0' then
                v_dx := to_unsigned(2047, 12);
            elsif p5_key = '1' and p6_key = '0' then
                v_dx := (others => '0');
            else
                v_dx := resize(p6_dx, 12) + resize(t_kx(13 downto 8), 12);
                if v_dx > 2047 then v_dx := to_unsigned(2047, 12); end if;
            end if;
            p6_dx <= v_dx(10 downto 0);
            if s_gav_sr(4) = '0' then p6_key <= '1'; else p6_key <= p5_key; end if;
            r_ea <= (not p_zA(5)) & v_dx(10 downto 4);
            pu(6) <= p5_u(11 downto 6); pxs(6) <= p5_xs; pk(6) <= p5_key;

            -- 7: env ROM; 8: fabric copy (p_rom)
            -- 9: d_eff = d_z * gain
            p9_de <= l_dz * unsigned(r_eq2(8 downto 1));

            -- 10: L ROM address (A: d_eff, B: 1/2 period - d_eff)
            if p9_de(17 downto 7) > 511 then v_de := to_unsigned(511, 10); else v_de := p9_de(16 downto 7); end if;
            if p_zA(9) = '1' then v_la := v_de; else v_la := to_unsigned(512, 10) - v_de; end if;
            if v_la(9) = '1' then r_la <= (others => '1'); else r_la <= v_la(8 downto 1); end if;

            -- 11: L ROM; 12: fabric copy (p_rom)
            -- 13: tints: TR * (u - 0.35), TB * (u - 0.35)
            if pxs(12) = '1' then v_uu := to_signed(-8, 7);
            else v_uu := signed(resize(pu(12), 7)) - 22; end if;
            p13_pr <= resize(signed(r_lq2(17 downto 10)) * v_uu, 15);

            p13_lr <= signed(r_lq2(47 downto 37));
            p13_lg <= unsigned(r_lq2(36 downto 27));
            p13_lb <= unsigned(r_lq2(26 downto 18));

            -- 14: L layer pre-signals
            p14_lr <= resize(p13_lr, 13) + resize(shift_right(p13_pr, 6), 13);
            p14_lb <= signed(resize(p13_lb, 13));
            p14_lg <= signed(resize(p13_lg, 13));

            -- 15: differences L - S; soft keys (2-pole per channel; Glow Hard = the key itself)
            p15_dr <= resize(p14_lr, 14) - resize(l_sr, 14);
            p15_dg <= resize(p14_lg, 14) - resize(l_sg, 14);
            p15_db <= resize(p14_lb, 14) - resize(l_sb, 14);
            -- (k1 takes the stage-12 key, k2 one clock later: K at stage 15 = the model's K of this pixel)
            -- one rate per channel: R tau 5.9, G 12.3, B 2.2 px (HD); SD / 720p one shift faster
            if pk(12) = '1' then v_t := to_signed(4095, 14); else v_t := to_signed(0, 14); end if;
            if s_gav_sr(11) = '0' then
                k1r <= to_signed(4095, 14); k1g <= to_signed(4095, 14); k1b <= to_signed(4095, 14);
            else
                k1r <= k1r + f_st(v_t - k1r, 3, 5, t_fast);
                k1g <= k1g + f_st(v_t - k1g, 4, 6, t_fast);
                k1b <= k1b + f_st(v_t - k1b, 2, 3, t_fast);
            end if;
            if s_gav_sr(12) = '0' then
                k2r <= to_signed(4095, 14); k2g <= to_signed(4095, 14); k2b <= to_signed(4095, 14);
            else
                k2r <= k2r + f_st(k1r - k2r, 3, 5, t_fast);
                k2g <= k2g + f_st(k1g - k2g, 4, 6, t_fast);
                k2b <= k2b + f_st(k1b - k2b, 2, 3, t_fast);
            end if;
            -- K (0..64); Glow Hard = the key itself
            if f_hard = '1' then
                if pk(14) = '1' then p15_kr <= to_unsigned(64, 7); p15_kg <= to_unsigned(64, 7); p15_kb <= to_unsigned(64, 7);
                else p15_kr <= (others => '0'); p15_kg <= (others => '0'); p15_kb <= (others => '0'); end if;
            else
                p15_kr <= resize(unsigned(shift_right(k2r + 32, 6)), 7);
                p15_kg <= resize(unsigned(shift_right(k2g + 32, 6)), 7);
                p15_kb <= resize(unsigned(shift_right(k2b + 32, 6)), 7);
            end if;

            -- 16: K * (L - S)
            -- (K = 64 is bit 6 alone: the third pair carries it)
            p16_ar <= f_pp(p15_dr, p15_kr, 0); p16_br <= f_pp(p15_dr, p15_kr, 2);
            p16_ag <= f_pp(p15_dg, p15_kg, 0); p16_bg <= f_pp(p15_dg, p15_kg, 2);
            p16_ab <= f_pp(p15_db, p15_kb, 0); p16_bb <= f_pp(p15_db, p15_kb, 2);
            if p15_kr(6) = '1' then p16_cr <= shift_left(resize(p15_dr, 20), 6); else p16_cr <= f_pp(p15_dr, p15_kr, 4); end if;
            if p15_kg(6) = '1' then p16_cg <= shift_left(resize(p15_dg, 20), 6); else p16_cg <= f_pp(p15_dg, p15_kg, 4); end if;
            if p15_kb(6) = '1' then p16_cb <= shift_left(resize(p15_db, 20), 6); else p16_cb <= f_pp(p15_db, p15_kb, 4); end if;

            -- 17: S + K (L - S)
            p17_r <= resize(l_sr, 14) + resize(shift_right(p16_ar + p16_br + p16_cr, 6), 14);
            p17_g <= resize(l_sg, 14) + resize(shift_right(p16_ag + p16_bg + p16_cg, 6), 14);
            p17_b <= resize(l_sb, 14) + resize(shift_right(p16_ab + p16_bb + p16_cb, 6), 14);

            -- 18: clip red / blue; green fold (triangle of the fraction, black below zero)
            if p17_r < 0 then p18_r <= (others => '0'); elsif p17_r > 255 then p18_r <= (others => '1'); else p18_r <= unsigned(p17_r(7 downto 0)); end if;
            if p17_b < 0 then p18_b <= (others => '0'); elsif p17_b > 255 then p18_b <= (others => '1'); else p18_b <= unsigned(p17_b(7 downto 0)); end if;
            if p17_g(7) = '0' then v_tr := '0' & unsigned(p17_g(6 downto 0)) & '0';
            else v_tr := to_unsigned(510, 9) - (unsigned(p17_g(7 downto 0)) & '0'); end if;
            p18_t <= v_tr(7 downto 0);
            p18_n <= p17_g(13);

            -- 19: green = 15 + tri * 230 / 256; palette swap
            v_gg := shift_left(resize(p18_t, 16), 8) - shift_left(resize(p18_t, 16), 4) - shift_left(resize(p18_t, 16), 3) - shift_left(resize(p18_t, 16), 1);
            if p18_n = '1' then p19_g <= to_unsigned(8, 8);
            else p19_g <= to_unsigned(15, 8) + v_gg(15 downto 8); end if;
            if f_swap = '1' then p19_r <= p18_b; p19_b <= p18_r; else p19_r <= p18_r; p19_b <= p18_b; end if;

            -- 20: RGB -> YUV, 2-term constants (Y pre-boosted 16/13: 320 640 128; U = Cr: 448 -384 -64;
            --     V = Cb: -160 -288 448; all /256) summed directly: Y/64 = 5R + 10G + 2B,
            --     U/64 = 7R - 6G - B, V/32 = 14B - 5R - 9G
            p20_y <= signed(resize(p19_r & "00", 14)) + signed(resize(p19_r, 14)) + signed(resize(p19_g & "000", 14))
                     + signed(resize(p19_g & '0', 14)) + signed(resize(p19_b & '0', 14));
            p20_u <= signed(resize(p19_r & "000", 14)) - signed(resize(p19_r, 14)) - signed(resize(p19_g & "00", 14))
                     - signed(resize(p19_g & '0', 14)) - signed(resize(p19_b, 14));
            p20_v <= signed(resize(p19_b & "0000", 14)) - signed(resize(p19_b & '0', 14)) - signed(resize(p19_r & "00", 14))
                     - signed(resize(p19_r, 14)) - signed(resize(p19_g & "000", 14)) - signed(resize(p19_g, 14));

            -- 21: pad; 22: scale and offset
            p21_y <= p20_y; p21_u <= p20_u; p21_v <= p20_v;
            p22_y <= resize(shift_right(p21_y, 2), 13) + 64;
            p22_u <= resize(shift_right(p21_u, 2), 13) + 512;
            p22_v <= resize(shift_right(p21_v, 3), 13) + 512;

            -- 23: clamp (luma to 940: the conditioning brings it to ~777); 24: pad
            if p22_y > 940 then p23_y <= to_unsigned(940, 10); else p23_y <= unsigned(p22_y(9 downto 0)); end if;
            if p22_u < 0 then p23_u <= (others => '0'); elsif p22_u > 1023 then p23_u <= (others => '1'); else p23_u <= unsigned(p22_u(9 downto 0)); end if;
            if p22_v < 0 then p23_v <= (others => '0'); elsif p22_v > 1023 then p23_v <= (others => '1'); else p23_v <= unsigned(p22_v(9 downto 0)); end if;
            p24_y <= p23_y; p24_u <= p23_u; p24_v <= p23_v;

            -- 25..30: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]; centre at stage 29 -> 30)
            s_boxu <= s_boxu + p24_u - unsigned(ru_q2);
            s_boxv <= s_boxv + p24_v - unsigned(rv_q2);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u30 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v30 <= v_sc(13 downto 4);
            s_y25 <= p24_y; s_y26 <= s_y25; s_y27 <= s_y26; s_y28 <= s_y27;
            s_y29 <= s_y28; s_y30x <= s_y29;
            v_sy := resize(s_y28, 12) + shift_left(resize(s_y29, 12), 1) + resize(s_y30x, 12);
            s_yf30 <= v_sy(11 downto 2);
            -- 31: luma toward black (white 940 -> 777); blanking gate
            if s_yf30 > 64 then v_dl := s_yf30 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(29) = '1' then
                s_y31 <= s_yf30 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u31 <= s_u30; s_v31 <= s_v30;
            else
                s_y31 <= to_unsigned(64, 10); s_u31 <= to_unsigned(512, 10); s_v31 <= to_unsigned(512, 10);
            end if;
            -- 32: pad to the 32-clock latency
            s_y32 <= s_y31; s_u32 <= s_u31; s_v32 <= s_v31;

            -- delays
            for i in 2 to 15 loop p_zA(i) <= p_zA(i - 1); end loop;
            for i in 7 to 13 loop pu(i) <= pu(i - 1); pxs(i) <= pxs(i - 1); end loop;
            for i in 7 to 14 loop pk(i) <= pk(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y32);
    data_out.u       <= std_logic_vector(s_u32);
    data_out.v       <= std_logic_vector(s_v32);
    data_out.hsync_n <= rs_q2(3);
    data_out.vsync_n <= rs_q2(2);
    data_out.field_n <= rs_q2(1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cusp;
