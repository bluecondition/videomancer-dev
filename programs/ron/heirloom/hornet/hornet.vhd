-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Hornet v0.1.1 -- Heirloom (phase 3, video clips), from clip WIN_20240207_23_34_28 at 0.5 s.
--
-- v0.1.1: the key fill folds (orange edge, fat yellow body, orange core), the key path delay is 28 virtual px,
-- the pupils are flat grey discs, the reds are darker; the camera luma is boxed (8 px), decimated by 2 and
-- vertically smoothed (IIR 1/4 per line) and its default bend is a quarter of v0.1.0's (the Osc keeps its strength).
--
-- The still: a band of "hornet faces" across the middle of the frame -- two red eyes with grey pupils, four
-- tilted yellow lemons in dark crescents, yellow Y-ridges between them, cyan brows -- between fine red / cyan
-- stripes stepped by a square wave at the top and wavy white / red / cyan bands and rounded cyan / red blocks
-- below, the whole picture cut every 65 lines by a screen-locked sawtooth.  In the clip the whole vertical
-- structure rolls up (~90 lines/s, one period = 1150 lines) through the fixed cuts, and Ron leans the lemons.
--
-- The patch, as rebuilt (hornet.py has the fitted tables):
--   theta  = (x - 535) / 950                         the eye oscillator (2 faces across)
--   g      = V(yr) + E1(yr) cos(theta) + E2(yr) cos(2 theta - lean (yr - 470)) + E3(yr) cos(3 theta)
--            + D(yr) sq(x) + KC(yr) saw65(y) - pupil + etch * luma
--            yr = content row (rolls), y = screen row (the cuts are screen-locked); the envelopes V, E1..E3, D, KC
--            were fitted per row to the still (256 entries over the 1150-line content period, interpolated)
--   key    = g(x - Dk) in the window (1 - kw, 1) on the lemon rows (a window key, its path delayed, plus
--            comparator noise: the jaggy lemon edges and their dark crescents)
--   f      = pupil ? grey : key ? fold(C + d) : g  (d = -g; the fill rises orange -> yellow and folds back to an
--            orange core); colour = a 48-colour cyclic palette (fitted to the still) at f mod 1
--
--   K1  Eyes      eye oscillator depth (eyes, brows, the V under the eyes)   0 .. 2x (512 = the still)
--   K2  Lemons    lemon / ridge harmonic depth                                 0 .. 2x
--   K3  Stripes   vertical sweep: stripe density and band spacing              0.3x .. 3x
--   K4  Roll      bipolar vertical roll (742 = the clip's 90 lines/s up)
--   K5  Blocks    the 574-px square wave: steps on the stripes, rounded blocks 0 .. 2x
--   K6  Etch      camera (or Osc) luma into the field
--   S7  Cuts      Fine (the 65-line screen-locked cuts) / Shred (16-line cuts across the whole picture)
--   S8  Colours   Fire (the still) / Wasp (yellow, black, white, cyan: clip 23_33_13)
--   S9  Twin      2 faces / 4 faces across
--   S10 Key       On (keyed yellow lemons) / Off (the raw field: dark-cored rings)
--   S11 Source    Video / Osc
--   P12 LEAN      the lemons' lean (Ron's knob in the clip): mirrored "angry face" .. the still .. swept W's
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: content row, table interpolation, knob scales, cut, lean, pupil), a 24-stage
-- pixel path with three quarter-wave cosine ROMs.  Virtual 1080-line coordinates (SD shows the centre 4:3).
-- Generated avid; output conditioning (U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum
-- resync, their 8-sample delays in a 16-deep block-RAM ring; luma [1 2 1] and 13/16) before the blanking gate.
-- Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.hornet_pkg.all;
use work.all;

architecture hornet of program_top is
    constant C_LAT  : integer := 24;      -- = processing_delay_clks
    constant C_PC8  : integer := 294400;  -- content period, Q8 lines (1150)
    constant C_TIX  : integer := 14589;   -- 2^24 * 256 / (1150 * 256): content Q8 line -> table position Q24

    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);
            when 4          => return to_unsigned(576, 12);
            when 8          => return to_unsigned(960, 12);
            when 12         => return to_unsigned(480, 12);
            when 5 | 6 | 14 => return to_unsigned(384, 12);
            when 1 | 2 | 10 => return to_unsigned(512, 12);
            when others     => return to_unsigned(256, 12);
        end case;
    end function;
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    attribute no_rw_check : boolean;
    -- camera / Osc luma: an 8-px box, every second sample kept, a vertical IIR (1/4 per line) held in a 1024 x 8
    -- line memory whose one read port alternates between the IIR's read-modify-write and the beam
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal s_bx    : t_u8(0 to 7) := (others => x"00");
    signal s_bs    : unsigned(10 downto 0) := (others => '0');
    signal s_rq, s_rq1, s_rq2, s_rq3 : std_logic := '0';
    signal s_ka, s_ka1, s_ka2, s_ka3 : unsigned(9 downto 0) := (others => '0');
    signal s_av, s_av1, s_av2, s_av3 : unsigned(7 downto 0) := (others => '0');
    signal s_st    : unsigned(7 downto 0) := (others => '0');
    signal s_bm    : unsigned(7 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_c1_a, r_c2_a, r_c3_a : unsigned(7 downto 0) := (others => '0');
    signal r_c1_q, r_c2_q, r_c3_q : unsigned(11 downto 0) := (others => '0');
    signal r_pl_a : unsigned(8 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');
    signal t_a    : unsigned(7 downto 0) := (others => '0');
    signal r_ta_q, r_tb_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_tc_q : std_logic_vector(15 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw roll, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(C_P12DEF * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(742, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_shred, s_wasp, s_twin, s_keyoff, s_src : std_logic := '0';

    -- per-field constants
    signal c_th0, c_thi : unsigned(23 downto 0) := (others => '0');   -- eye oscillator phase at the first pixel, step
    signal c_ps0, c_psi : unsigned(23 downto 0) := (others => '0');   -- square-wave phase, step
    signal c_ks1, c_ks2, c_ks5 : unsigned(9 downto 0) := to_unsigned(512, 10);   -- knob scales, Q9
    signal c_ks3  : unsigned(10 downto 0) := to_unsigned(512, 11);    -- stripes scale, Q9 (0.3 .. 3)
    signal c_lean : unsigned(15 downto 0) := (others => '0');          -- lean, Q24 cycles of 2 theta per line
    signal c_eg   : unsigned(15 downto 0) := (others => '0');          -- etch gain (Q16 per luma step << 4)
    signal c_roll : unsigned(18 downto 0) := to_unsigned(C_ROLL0, 19); -- content row of screen row 0, Q8
    signal c_rstep : unsigned(11 downto 0) := (others => '0');         -- roll step per field, Q8 lines
    signal c_sawi : unsigned(15 downto 0) := to_unsigned(1005, 16);    -- cut saw step per line
    signal c_ky, c_ku, c_kv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- palette black
    signal c_tap  : unsigned(1 downto 0) := "00";                      -- key delay tap by mode

    -- field serial multiplier
    signal f_a   : unsigned(27 downto 0) := (others => '0');
    signal f_b   : unsigned(11 downto 0) := (others => '0');
    signal f_acc : unsigned(27 downto 0) := (others => '0');
    signal z_fa  : unsigned(27 downto 0) := (others => '0');
    signal z_fb  : unsigned(11 downto 0) := (others => '0');
    signal z_sel : unsigned(2 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal z_k   : unsigned(9 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ys   : unsigned(19 downto 0) := (others => '0');
    signal e_yr   : unsigned(18 downto 0) := (others => '0');
    signal e_idx  : unsigned(7 downto 0) := (others => '0');
    signal e_fr   : unsigned(7 downto 0) := (others => '0');
    signal e_yri  : unsigned(10 downto 0) := (others => '0');
    signal e_ta0, e_ta1, e_tb0, e_tb1 : std_logic_vector(31 downto 0) := (others => '0');
    signal e_tc0, e_tc1 : std_logic_vector(15 downto 0) := (others => '0');
    signal e_v    : signed(21 downto 0) := (others => '0');
    signal e_e1, e_e2, e_e3, e_d : signed(11 downto 0) := (others => '0');
    signal e_saw  : unsigned(15 downto 0) := (others => '0');
    signal e_sneg : std_logic := '0';
    signal e_smag : unsigned(14 downto 0) := (others => '0');
    signal e_kc   : unsigned(7 downto 0) := (others => '0');
    signal e_vs   : unsigned(15 downto 0) := (others => '0');
    signal e_lf   : unsigned(15 downto 0) := x"ACE1";
    signal e_nh   : std_logic := '0';
    signal e_np, e_pair : std_logic := '0';
    signal m_a    : unsigned(29 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(29 downto 0) := (others => '0');
    signal n_vl   : unsigned(15 downto 0) := (others => '0');
    signal n_e1, n_e2, n_e3, n_d : signed(11 downto 0) := (others => '0');
    signal n_phi  : unsigned(23 downto 0) := (others => '0');
    signal n_key  : std_logic := '0';
    signal n_pu   : unsigned(4 downto 0) := (others => '0');
    signal n_nz   : unsigned(15 downto 0) := (others => '0');

    -- beam / line registers
    signal b_th  : unsigned(23 downto 0) := (others => '0');
    signal b_ps  : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_vl  : unsigned(15 downto 0) := (others => '0');
    signal l_e1, l_e2, l_e3, l_d : signed(11 downto 0) := (others => '0');
    signal l_phi : unsigned(23 downto 0) := (others => '0');
    signal l_key : std_logic := '0';
    signal l_pu  : unsigned(4 downto 0) := (others => '0');
    signal l_nz  : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    signal p2_a1, p2_a2, p2_a3 : unsigned(9 downto 0) := (others => '0');
    signal p2_ps : unsigned(15 downto 0) := (others => '0');
    signal p3_n1, p3_n2, p3_n3, p4_n1, p4_n2, p4_n3, p5_n1, p5_n2, p5_n3 : std_logic := '0';
    signal p3_sq, p4_sq, p5_sq : signed(9 downto 0) := (others => '0');
    signal p3_lm, p4_lm, p5_lm : unsigned(7 downto 0) := (others => '0');
    signal p5_c1, p5_c2, p5_c3 : unsigned(11 downto 0) := (others => '0');
    signal p6_h1, p6_l1, p6_h2, p6_l2, p6_h3, p6_l3 : signed(18 downto 0) := (others => '0');
    signal p6_n1, p6_n2, p6_n3, p6_nd : std_logic := '0';
    signal p6_hd, p6_ld : signed(17 downto 0) := (others => '0');
    signal p6_ea, p6_eb : unsigned(19 downto 0) := (others => '0');
    signal p6_pd : signed(12 downto 0) := (others => '0');
    signal p7_t1, p7_t2, p7_t3, p7_td, p7_et : unsigned(15 downto 0) := (others => '0');
    signal p7_pm, p8_pm, p9_pm, p10_pm, p11_pm, p11_fd : std_logic := '0';
    signal p8_a, p8_b, p8_c : unsigned(15 downto 0) := (others => '0');
    signal p9_ab, p9_cp : unsigned(15 downto 0) := (others => '0');
    signal p10_g : unsigned(15 downto 0) := (others => '0');
    signal p11_g, p11_c, p11_c2 : unsigned(15 downto 0) := (others => '0');
    signal p11_k : std_logic := '0';
    signal s_ks  : std_logic_vector(31 downto 0) := (others => '0');
    signal p12_f : unsigned(15 downto 0) := (others => '0');
    signal p13_f, p14_f, p15_f : unsigned(7 downto 0) := (others => '0');
    signal p18_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    -- the boxes' 8-sample delays (both chroma boxes' inputs and the luma) in one 16-deep block-RAM ring
    type t_rg is array (0 to 15) of std_logic_vector(49 downto 0);
    signal rg : t_rg := (others => (others => '0'));
    attribute no_rw_check of rg : signal is true;
    signal rg_wa : unsigned(3 downto 0) := (others => '0');
    signal rg_q, rg_f : std_logic_vector(49 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ly8, s_ly9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    -- the read port: IIR read of sample k when requested, else the beam (sample b_px / 2 - 2, written well before);
    -- IIR: new = old + (box - old + 2) / 4, written back three clocks after the read (pipelined read-modify-write)
    p_lbctl : process(clk)
        variable v_n : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_rq = '1' then s_lb_ra <= s_ka; else s_lb_ra <= b_px(10 downto 1) - 2; end if;
            s_rq1 <= s_rq; s_ka1 <= s_ka; s_av1 <= s_av;
            s_rq2 <= s_rq1; s_ka2 <= s_ka1; s_av2 <= s_av1;          -- s_lb_q now holds the read issued with s_rq1
            if s_rq2 = '1' then s_st <= unsigned(s_lb_q); else s_bm <= unsigned(s_lb_q); end if;
            s_rq3 <= s_rq2; s_ka3 <= s_ka2; s_av3 <= s_av2;
            v_n := signed(resize(s_av3, 10)) - signed(resize(s_st, 10)) + 2;
            s_lb_we <= s_rq3;
            s_lb_wa <= s_ka3;
            s_lb_wd <= std_logic_vector(s_st + unsigned(resize(shift_right(v_n, 2), 8)));
        end if;
    end process p_lbctl;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_c1_q <= C_HN_COS(to_integer(r_c1_a));
            r_c2_q <= C_HN_COS(to_integer(r_c2_a));
            r_c3_q <= C_HN_COS(to_integer(r_c3_a));
            r_pl_q <= C_HN_PAL(to_integer(r_pl_a));
            r_ta_q <= C_HN_TA(to_integer(t_a));
            r_tb_q <= C_HN_TB(to_integer(t_a));
            r_tc_q <= C_HN_TC(to_integer(t_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_r  : unsigned(19 downto 0);
        variable v_eo : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

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

            -- roll: bipolar, +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;     -- |v| - 8
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_shred  <= registers_in(6)(0);
                s_wasp   <= registers_in(6)(1);
                s_twin   <= registers_in(6)(2);
                s_keyoff <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); c_tap <= "10";
                    when 3      => s_oxs <= to_unsigned(153, 9); c_tap <= "01";
                    when others => s_oxs <= to_unsigned(102, 9); c_tap <= "00";
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 64 + 16 k (12 iterations, result at load + 13);
            -- slot number decoded at 62 + 16 k, operands pre-muxed at 63 + 16 k
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 62  => z_sel <= to_unsigned(0, 3); z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(1, 3); z_pre <= '1';
                    when 94  => z_sel <= to_unsigned(2, 3); z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(3, 3); z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 3); z_pre <= '1';
                    when 142 => z_sel <= to_unsigned(5, 3); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 => z_fa <= resize(s_vm, 28);   z_fb <= resize(s_vm, 12);                          -- roll speed^2
                    when 1 => z_fa <= resize(z_k, 28);    z_fb <= to_unsigned(358, 12);                      -- stripes, lower slope
                    when 2 => z_fa <= resize(s_g6(12 downto 3), 28); z_fb <= resize(s_g6(12 downto 3), 12);  -- etch^2
                    when 3 => z_fa <= resize(f_acc(19 downto 4), 28); z_fb <= to_unsigned(C_EGK, 12);       -- etch gain
                    when 4 => z_fa <= resize(s_gp(12 downto 3), 28); z_fb <= to_unsigned(C_LEANK, 12);      -- lean
                    when others =>                                                                            -- cut saw step
                        z_fa <= resize(s_ys, 28);
                        if s_shred = '1' then z_fb <= to_unsigned(4021, 12); else z_fb <= to_unsigned(1005, 12); end if;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(26 downto 0) & '0';
                f_b <= '0' & f_b(11 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's step); oscillator phases for this field
                    when 20 =>
                        if s_vneg = '1' then v_r := resize(c_roll, 20) - resize(c_rstep, 20);
                        else v_r := resize(c_roll, 20) + resize(c_rstep, 20); end if;
                        if v_r(19) = '1' then v_r := v_r + C_PC8;                    -- below 0
                        elsif v_r >= C_PC8 then v_r := v_r - C_PC8; end if;
                        c_roll <= v_r(18 downto 0);
                    when 21 =>
                        -- eye oscillator: theta = (x - 535) / 950 at the first pixel (beam starts at x0 + 8 px)
                        case to_integer(s_xs) is
                            when 4 =>
                                c_th0 <= to_unsigned(11850076, 24); c_thi <= to_unsigned(35320, 24);
                                c_ps0 <= to_unsigned(16514146, 24); c_psi <= to_unsigned(58460, 24);
                            when 3 =>
                                c_th0 <= to_unsigned(7541036, 24);  c_thi <= to_unsigned(26490, 24);
                                c_ps0 <= to_unsigned(9382026, 24);  c_psi <= to_unsigned(43845, 24);
                            when others =>
                                c_th0 <= to_unsigned(7470396, 24);  c_thi <= to_unsigned(17660, 24);
                                c_ps0 <= to_unsigned(9265106, 24);  c_psi <= to_unsigned(29230, 24);
                        end case;
                    when 22 =>
                        if s_twin = '1' then
                            c_th0 <= c_th0(22 downto 0) & '0'; c_thi <= c_thi(22 downto 0) & '0';
                        end if;
                        if s_wasp = '1' then v_k := C_HN_PAL(256 + C_PBLK); else v_k := C_HN_PAL(C_PBLK); end if;
                        c_ky <= unsigned(v_k(31 downto 22)); c_ku <= unsigned(v_k(21 downto 12)); c_kv <= unsigned(v_k(11 downto 2));
                        -- knob scales (Q9: 512 = the still)
                        c_ks1 <= s_g1(12 downto 3); c_ks2 <= s_g2(12 downto 3); c_ks5 <= s_g5(12 downto 3);
                    when 23 =>
                        -- stripes: 0.3x .. 1x below the centre (k * 358 >> 9 + 154), 1x .. 3x above (2 per step)
                        if s_g3(12 downto 3) >= 512 then
                            c_ks3 <= to_unsigned(512, 11) + resize(s_g3(11 downto 3) & '0', 11);
                            z_k <= (others => '0');
                        else
                            z_k <= s_g3(12 downto 3);
                        end if;
                    when 77  => c_rstep <= f_acc(18 downto 7);                         -- speed^2 >> 7 (Q8 lines)
                    when 93  =>
                        if s_g3(12 downto 3) < 512 then c_ks3 <= to_unsigned(154, 11) + resize(f_acc(18 downto 9), 11); end if;
                    -- 109: etch^2 (in f_acc) -> slot 3
                    when 125 =>
                        if s_src = '1' then
                            -- Osc: 12x the camera's gain (the camera default is a quarter of v0.1.0's; the Osc stays as strong)
                            v_eo := resize(f_acc(22 downto 6), 18) + resize(f_acc(22 downto 7), 18);
                            if v_eo(17 downto 16) /= "00" then c_eg <= (others => '1'); else c_eg <= v_eo(15 downto 0); end if;
                        else
                            c_eg <= resize(f_acc(22 downto 9), 16);
                        end if;
                    when 141 => c_lean <= f_acc(15 downto 0);                        -- P12 * 55 (Q24 per line)
                    when 157 => c_sawi <= f_acc(23 downto 8);                        -- 65536 / 65.2 per virtual line
                    when others => null;
                end case;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer
    -- write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            -- source luma (or the Osc lattice) through an 8-px running box (cleared off the source avid)
            if s_src = '1' then
                v_l := s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then v_l := v_l - 16; else v_l := (others => '0'); end if;
            end if;
            if data_in.avid = '1' then
                s_bx(0) <= v_l;
                for i in 1 to 7 loop s_bx(i) <= s_bx(i - 1); end loop;
                s_bs <= s_bs + v_l - s_bx(7);
            else
                s_bx <= (others => x"00"); s_bs <= (others => '0');
            end if;
            -- every second source pixel: sample k = s_wcol / 2 -> IIR read request
            if data_in.avid = '1' and s_wcol(0) = '1' then
                s_rq <= '1'; s_ka <= s_wcol(10 downto 1); s_av <= s_bs(10 downto 3);
            else
                s_rq <= '0';
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17): content row -> table position, table interpolation, knob
    -- scales, the cut saw, the lean phase, the pupil profile and the key noise
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(21 downto 0);
        variable v_dy : signed(11 downto 0);
        variable v_t  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 200 then e_zk <= '1'; end if;

            -- line pairs: at 1080p and 720p the engine computes the even lines only and each odd line repeats the
            -- even line's constants, so no feature of the field is ever a single row (>= 2 virtual rows everywhere;
            -- SD and interlaced lines are already >= 2 virtual rows)
            if s_ys = 256 or s_ys = 384 then e_pair <= '1'; else e_pair <= '0'; end if;
            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_saw <= to_unsigned(C_SAW0, 16);
                e_np <= '1';
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_saw <= e_saw + c_sawi;
                e_np <= not e_np;
                if e_np = '0' or e_pair = '0' then e_lc <= (others => '0'); end if;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(28 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_ys <= resize(e_vy, 20) + resize(c_roll, 20);
                    -- cut saw (screen-locked): saw = phase - 1/2, signed
                    e_sneg <= not e_saw(15);
                    if e_saw(15) = '1' then e_smag <= e_saw(14 downto 0); else e_smag <= not e_saw(14 downto 0); end if;
                    -- key noise: a new draw every second line (no single-row jags on near-horizontal key edges)
                    e_nh <= not e_nh;
                    if e_nh = '1' then e_lf <= e_lf(14 downto 0) & (e_lf(15) xor e_lf(13) xor e_lf(12) xor e_lf(10)); end if;
                when 2 =>
                    if e_ys >= C_PC8 then e_yr <= resize(e_ys - C_PC8, 19); else e_yr <= e_ys(18 downto 0); end if;
                when 3 =>
                    m_a <= resize(e_yr(18 downto 3), 30); m_b <= to_unsigned(C_TIX, 16); m_acc <= (others => '0');
                    e_yri <= e_yr(18 downto 8);
                when 20 =>
                    e_idx <= m_acc(28 downto 21); e_fr <= m_acc(20 downto 13);
                when 21 => t_a <= e_idx;
                when 22 => t_a <= e_idx + 1;
                when 23 => e_ta0 <= r_ta_q; e_tb0 <= r_tb_q; e_tc0 <= r_tc_q;
                when 24 => e_ta1 <= r_ta_q; e_tb1 <= r_tb_q; e_tc1 <= r_tc_q;
                -- interpolation: x0 + (x1 - x0) * fr / 256
                when 25 =>
                    v_d := signed(e_ta1(31 downto 10)) - signed(e_ta0(31 downto 10));
                    m_a <= unsigned(resize(v_d, 30)); m_b <= resize(e_fr, 16); m_acc <= (others => '0');
                    e_kc <= unsigned(e_ta0(9 downto 2));
                    n_key <= e_ta0(1) and not s_keyoff;
                when 42 =>
                    e_v <= signed(e_ta0(31 downto 10)) + signed(m_acc(29 downto 8));
                    v_d := resize(signed(e_tb1(31 downto 22)), 22) - resize(signed(e_tb0(31 downto 22)), 22);
                    m_a <= unsigned(resize(v_d, 30)); m_b <= resize(e_fr, 16); m_acc <= (others => '0');
                when 59 =>
                    e_e1 <= resize(signed(e_tb0(31 downto 22)), 12) + signed(m_acc(19 downto 8));
                    v_d := resize(signed(e_tb1(21 downto 12)), 22) - resize(signed(e_tb0(21 downto 12)), 22);
                    m_a <= unsigned(resize(v_d, 30)); m_b <= resize(e_fr, 16); m_acc <= (others => '0');
                when 76 =>
                    e_e2 <= resize(signed(e_tb0(21 downto 12)), 12) + signed(m_acc(19 downto 8));
                    v_d := resize(signed(e_tb1(11 downto 2)), 22) - resize(signed(e_tb0(11 downto 2)), 22);
                    m_a <= unsigned(resize(v_d, 30)); m_b <= resize(e_fr, 16); m_acc <= (others => '0');
                when 93 =>
                    e_e3 <= resize(signed(e_tb0(11 downto 2)), 12) + signed(m_acc(19 downto 8));
                    v_d := resize(signed(e_tc1(15 downto 6)), 22) - resize(signed(e_tc0(15 downto 6)), 22);
                    m_a <= unsigned(resize(v_d, 30)); m_b <= resize(e_fr, 16); m_acc <= (others => '0');
                when 110 =>
                    e_d <= resize(signed(e_tc0(15 downto 6)), 12) + signed(m_acc(19 downto 8));
                    -- scales: V' = V * stripes (Q9) ; the table V is relative to the band level C_VB
                    m_a <= unsigned(resize(e_v, 30)); m_b <= resize(c_ks3, 16); m_acc <= (others => '0');
                when 127 =>
                    e_vs <= m_acc(24 downto 9);
                    m_a <= unsigned(resize(e_e1, 30)); m_b <= resize(c_ks1, 16); m_acc <= (others => '0');
                when 144 =>
                    n_e1 <= signed(m_acc(20 downto 9));
                    m_a <= unsigned(resize(e_e2, 30)); m_b <= resize(c_ks2, 16); m_acc <= (others => '0');
                when 161 =>
                    n_e2 <= signed(m_acc(20 downto 9));
                    m_a <= unsigned(resize(e_e3, 30)); m_b <= resize(c_ks1, 16); m_acc <= (others => '0');
                when 178 =>
                    n_e3 <= signed(m_acc(20 downto 9));
                    m_a <= unsigned(resize(e_d, 30)); m_b <= resize(c_ks5, 16); m_acc <= (others => '0');
                when 195 =>
                    n_d <= signed(m_acc(20 downto 9));
                    -- cut: KC (Q9 cycles) * |saw| (Q16 of 1/2) ; Shred = a fixed depth on every row
                    m_a <= resize(e_smag, 30);
                    if s_shred = '1' then m_b <= to_unsigned(C_KCSH, 16); else m_b <= resize(e_kc, 16); end if;
                    m_acc <= (others => '0');
                when 212 =>
                    -- KC * |saw| >> 9 (Q16 cycles: KC/512 * saw_mag)
                    if e_sneg = '1' then v_t := e_vs - m_acc(24 downto 9); else v_t := e_vs + m_acc(24 downto 9); end if;
                    n_vl <= v_t + to_unsigned(C_VB, 16);
                    -- lean: phase of the 2-theta harmonic = lean * (yr - 470)
                    v_dy := signed(resize(e_yri, 12)) - 470;
                    m_a <= unsigned(resize(v_dy, 30)); m_b <= c_lean; m_acc <= (others => '0');
                    -- pupil profile (content rows 300 +- 15)
                    v_dy := signed(resize(e_yri, 12)) - C_PUY;
                    if v_dy > -16 and v_dy < 16 then
                        n_pu <= C_HN_PUY(to_integer(unsigned(abs(v_dy(4 downto 0)))));
                    else
                        n_pu <= (others => '0');
                    end if;
                    -- key noise: a signed per-line draw, +-512 (Q16)
                    n_nz <= resize(e_lf(9 downto 0), 16) - to_unsigned(512, 16);
                when 229 =>
                    n_phi <= m_acc(23 downto 0);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the eye oscillator phase and the square-wave phase per pixel
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_th <= c_th0;
                b_ps <= c_ps0;
                b_px <= (others => '0');
                l_vl <= n_vl; l_e1 <= n_e1; l_e2 <= n_e2; l_e3 <= n_e3; l_d <= n_d;
                l_phi <= n_phi; l_key <= n_key; l_pu <= n_pu; l_nz <= n_nz;
            elsif g_avid = '1' then
                b_th <= b_th + c_thi;
                b_ps <= b_ps + c_psi;
                b_px <= b_px + 1;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t2, v_t3 : unsigned(23 downto 0);
        variable v_s  : unsigned(15 downto 0);
        variable v_m  : unsigned(19 downto 0);
        variable v_q  : signed(10 downto 0);
        variable v_aq : unsigned(9 downto 0);
        variable v_sum : signed(24 downto 0);
        variable v_e  : unsigned(23 downto 0);
        variable v_p  : signed(12 downto 0);
        variable v_g  : unsigned(15 downto 0);
        variable v_k  : std_logic;
        variable v_d  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: phases of theta, 2 theta - lean, 3 theta (10 bits each); square phase
            v_t2 := (b_th(22 downto 0) & '0') + l_phi;
            v_t3 := b_th + (b_th(22 downto 0) & '0');
            p2_a1 <= b_th(23 downto 14); p2_a2 <= v_t2(23 downto 14); p2_a3 <= v_t3(23 downto 14);
            p2_ps <= b_ps(23 downto 8);
            -- 3: quarter-wave addresses and signs; slewed square (+-511, 30 virtual px edges)
            if p2_a1(8) = '1' then r_c1_a <= not p2_a1(7 downto 0); else r_c1_a <= p2_a1(7 downto 0); end if;
            if p2_a2(8) = '1' then r_c2_a <= not p2_a2(7 downto 0); else r_c2_a <= p2_a2(7 downto 0); end if;
            if p2_a3(8) = '1' then r_c3_a <= not p2_a3(7 downto 0); else r_c3_a <= p2_a3(7 downto 0); end if;
            p3_n1 <= p2_a1(9) xor p2_a1(8); p3_n2 <= p2_a2(9) xor p2_a2(8); p3_n3 <= p2_a3(9) xor p2_a3(8);
            v_s := '0' & p2_ps(14 downto 0);                        -- phase within the half period
            v_m := shift_left(resize(v_s, 20), 4) + shift_left(resize(v_s, 20), 1) + resize(v_s, 20);   -- * 19
            if v_m(19 downto 16) /= "0000" then v_q := to_signed(511, 11);          -- 2 * ramp, saturated
            else v_q := signed(resize(v_m(15 downto 7), 11)); end if;              -- ramp 0 .. 511 over 30 virtual px
            if p2_ps(15) = '0' then v_q := shift_left(v_q, 1) - to_signed(511, 11);  -- rising half: -511 .. 511
            else v_q := to_signed(511, 11) - shift_left(v_q, 1); end if;           -- falling half
            p3_sq <= v_q(9 downto 0);
            p3_lm <= s_bm;
            -- 4: cosine ROMs (EBR)
            p4_n1 <= p3_n1; p4_n2 <= p3_n2; p4_n3 <= p3_n3; p4_sq <= p3_sq; p4_lm <= p3_lm;
            -- 5: fabric copies
            p5_c1 <= r_c1_q; p5_c2 <= r_c2_q; p5_c3 <= r_c3_q;
            p5_n1 <= p4_n1; p5_n2 <= p4_n2; p5_n3 <= p4_n3; p5_sq <= p4_sq; p5_lm <= p4_lm;
            -- 6: partial products (envelope x cosine magnitude hi / lo 6 bits; D x |sq| hi / lo; etch nibbles);
            --    pupil distance
            p6_h1 <= l_e1 * signed('0' & p5_c1(11 downto 6)); p6_l1 <= l_e1 * signed('0' & p5_c1(5 downto 0));
            p6_h2 <= l_e2 * signed('0' & p5_c2(11 downto 6)); p6_l2 <= l_e2 * signed('0' & p5_c2(5 downto 0));
            p6_h3 <= l_e3 * signed('0' & p5_c3(11 downto 6)); p6_l3 <= l_e3 * signed('0' & p5_c3(5 downto 0));
            p6_n1 <= p5_n1; p6_n2 <= p5_n2; p6_n3 <= p5_n3; p6_nd <= p5_sq(9);
            v_aq := unsigned(abs(p5_sq));
            p6_hd <= l_d * signed('0' & v_aq(8 downto 4));
            p6_ld <= resize(l_d * signed('0' & v_aq(3 downto 0)), 18);
            p6_ea <= p5_lm(3 downto 0) * c_eg;
            p6_eb <= p5_lm(7 downto 4) * c_eg;
            -- pupil: radius^2 profile of the line minus 2047 (1 - cos theta) (~ x^2), only near theta = 0
            if p5_n1 = '1' then p6_pd <= to_signed(-1, 13);
            else p6_pd <= signed(resize(l_pu, 13)) - to_signed(2047, 13) + signed(resize(p5_c1, 13)); end if;
            -- 7: terms (Q16 cycles, mod 1)
            v_sum := shift_left(resize(p6_h1, 25), 6) + resize(p6_l1, 25);
            if p6_n1 = '1' then p7_t1 <= unsigned(-v_sum(20 downto 5)); else p7_t1 <= unsigned(v_sum(20 downto 5)); end if;
            v_sum := shift_left(resize(p6_h2, 25), 6) + resize(p6_l2, 25);
            if p6_n2 = '1' then p7_t2 <= unsigned(-v_sum(20 downto 5)); else p7_t2 <= unsigned(v_sum(20 downto 5)); end if;
            v_sum := shift_left(resize(p6_h3, 25), 6) + resize(p6_l3, 25);
            if p6_n3 = '1' then p7_t3 <= unsigned(-v_sum(20 downto 5)); else p7_t3 <= unsigned(v_sum(20 downto 5)); end if;
            v_sum := shift_left(resize(p6_hd, 25), 4) + resize(p6_ld, 25);       -- D (Q10) * |sq| (Q9) = Q19
            if p6_nd = '1' then p7_td <= unsigned(-v_sum(18 downto 3)); else p7_td <= unsigned(v_sum(18 downto 3)); end if;
            v_e := resize(p6_ea, 24) + shift_left(resize(p6_eb, 24), 4);
            p7_et <= v_e(19 downto 4);
            if p6_pd > 0 then p7_pm <= '1'; else p7_pm <= '0'; end if;            -- inside a pupil: a flat grey disc
            -- 8 .. 10: g = V + terms + etch - pupil
            p8_a <= p7_t1 + p7_t2; p8_b <= p7_t3 + p7_td; p8_c <= l_vl + p7_et; p8_pm <= p7_pm;
            p9_ab <= p8_a + p8_b; p9_cp <= p8_c; p9_pm <= p8_pm;
            p10_g <= p9_ab + p9_cp; p10_pm <= p9_pm;
            -- 11: the key window (on the noisy copy), the inverted field
            v_g := p10_g + l_nz;
            if v_g >= to_unsigned(C_KTHR, 16) and l_key = '1' and s_gav_sr(8) = '1' then p11_k <= '1'; else p11_k <= '0'; end if;
            -- the key fill rises from orange at the key edge (depth d = -g) to yellow at C_DPK and folds back to orange
            -- at the lemon core; just right of a lemon (d < 0, the delayed key) it falls to red
            p11_g <= p10_g;
            v_d := (others => '0'); v_d := v_d - p10_g;
            p11_c  <= to_unsigned(C_CINV, 16) + v_d;
            p11_c2 <= to_unsigned(C_CINV2, 16) - v_d;
            if signed(v_d) > C_DPK then p11_fd <= '1'; else p11_fd <= '0'; end if;
            p11_pm <= p10_pm;
            -- 12: the key path is delayed (28 / 19 / 14 px at 1080 / 720 / SD = 28 virtual px)
            s_ks <= s_ks(30 downto 0) & p11_k;
            case c_tap is
                when "10"   => v_k := s_ks(13);
                when "01"   => v_k := s_ks(18);
                when others => v_k := s_ks(27);
            end case;
            if p11_pm = '1' then p12_f <= to_unsigned(C_PGREY * 256, 16);
            elsif v_k = '1' and p11_fd = '1' then p12_f <= p11_c2;
            elsif v_k = '1' then p12_f <= p11_c;
            else p12_f <= p11_g; end if;
            -- 13 .. 15: pipeline padding (one latency in every mode)
            p13_f <= p12_f(15 downto 8); p14_f <= p13_f; p15_f <= p14_f;
            -- 16: palette address; 17: palette (EBR); 18: fabric copy; 19: compose
            r_pl_a <= s_wasp & p15_f;
            p18_q <= r_pl_q;
            if s_gav_sr(16) = '0' then
                s_y19 <= c_ky; s_u19 <= c_ku; s_v19 <= c_kv;
            else
                s_y19 <= unsigned(p18_q(31 downto 22)); s_u19 <= unsigned(p18_q(21 downto 12)); s_v19 <= unsigned(p18_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix), luma [1 2 1] and 13/16,
    -- the blanking gate.  The running sums resync to the palette black once 8
    -- blank samples are in.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ly : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- ring: written with this clock's box inputs, read 6 back + a fabric copy = the inputs of 8 clocks ago
            -- (the read never meets the write address: no_rw_check)
            rg_q <= rg(to_integer(rg_wa - 6));
            rg(to_integer(rg_wa)) <= std_logic_vector(s_u19) & std_logic_vector(s_v19) & std_logic_vector(s_u21) &
                                     std_logic_vector(s_v21) & std_logic_vector(s_y19);
            rg_wa <= rg_wa + 1;
            rg_f <= rg_q;
            -- chroma box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bau <= c_ku & "000"; s_bav <= c_kv & "000";
            else
                s_bau <= s_bau + s_u19 - unsigned(rg_f(49 downto 40));
                s_bav <= s_bav + s_v19 - unsigned(rg_f(39 downto 30));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_u21 <= c_ku; s_v21 <= c_kv;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- chroma box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bbu <= c_ku & "000"; s_bbv <= c_kv & "000";
            else
                s_bbu <= s_bbu + s_u21 - unsigned(rg_f(29 downto 20));
                s_bbv <= s_bbv + s_v21 - unsigned(rg_f(19 downto 10));
            end if;
            s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bbu, 14) + s_bbud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v23 <= v_sc(13 downto 4);
            -- luma [1 2 1] over the samples of 8, 9 and 10 clocks ago, then matched to the chroma latency
            s_ly8 <= unsigned(rg_f(9 downto 0)); s_ly9 <= s_ly8;
            v_ly := resize(unsigned(rg_f(9 downto 0)), 12) + shift_left(resize(s_ly8, 12), 1) + resize(s_ly9, 12);
            s_y21 <= v_ly(11 downto 2);                       -- off-avid inputs are already the palette black
            s_y22 <= s_y21; s_y23 <= s_y22;
            -- 24: luma 13/16
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u23; s_v24 <= s_v23;
            -- 25: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture hornet;
