-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tatter v0.1.1 -- Heirloom (video phase), from the clip WIN_20230424_23_47_33 at 8.0 s.
--
-- The still: a sum lattice of white tubes with dark-grey edges (eyes with a dark core and a thin orange ring in the
-- column at x 990, white X crossings in the column at x 510, a wavy white band down the right edge) over an
-- orange / yellow / red checker with dark-violet zones; the left of the frame torn away along a parabolic crack:
-- a blue / green / orange / violet strip at the left edge, a white bar (x 240 .. 740) and the same checker in
-- the sync-lost colours, in ragged bands, with thin white spikes and slipped bands.  In the clip the lattice
-- rises at ~238 lines/s (one eye row every 1.2 s) while the checker rows wobble in place and the tear flickers.
--
-- The patch, as fitted (tatter.py has every table):
--   u     = Fx(x) + Fy(y) + E      Fx: an H waveform with two regimes split by the crack (eyes / crosses), Fy: a
--                                  V cosine (280 lines); 256 LSB = one tube-palette cycle
--   u >= 0: the tube palette;  u < 0: the checker (rows FM'd by the V oscillator alternate two clipped-sawtooth H
--   oscillators, 100.5 / 176.5 px) through an RGB colouriser whose zone rides the V oscillator
--   tear  = the crack x = X0 + 0.00075 y^2; left of it the cross regime, the sync-lost Q picture (strip, white
--           bar, Q colours) below a roving top edge and left of a diagonal, bands of 8..71 lines slipping
--
--   K1  Columns  the lattice's H scale x0.41 .. x2.46 (512 = the still)
--   K2  Rows     the lattice's V period 792 .. 99 lines (512 = the still's 280)
--   K3  Eyes     field offset: checker floods, eyes shrink (0) .. the still (512) .. tubes swell, more rings (1023)
--   K4  Scroll   bipolar: the lattice rises (+) or sinks (-); 770 = the clip's ~4 lines per field (238 lines/s)
--   K5  Checker  the checker's block and row scale x0.35 .. x2.83 (512 = the still)
--   K6  Etch     how far the camera bends the field (square law, 8 px luma box); the Osc keeps a linear 2 k6 gain
--   S7  Crosses  the two H regimes swapped across the crack: crosses on the right, the eye column on the left
--   S8  Colours  Sunset (the still) / Lagoon (the sync-lost blue / green / orange / violet everywhere)
--   S9  Rings    Tubes (the still) / Contours (the field x3: nested ring lines)
--   S10 Invert   the field inverted: checker eyes, white tubes where the checker was
--   S11 Source   Video / Osc
--   P12 TEAR     0 = in sync (the clean eye lattice) .. 50 % = the still .. 100 % = the crack swept across, shredded
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: the V phase and ROM reads, the row parity, the crack, the tear bands, the slipped
-- L-regime start), a beam with four accumulators (the H phase, the slippable H phase with its hold, the two
-- checker oscillators), a 20-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated
-- avid; output conditioning (Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box, its taps and the syncs in
-- block-RAM rings) and luma 13/16 before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tatter_pkg.all;
use work.all;

architecture tatter of program_top is
    constant C_LAT  : integer := 20;   -- = processing_delay_clks

    constant C_XC2  : integer := 1980;        -- H scale centre (h = half virtual px): the eye column
    constant C_BASE : integer := 32440320;    -- 495 * 2^16: Fx table index at the centre
    constant C_KV0  : integer := 14980;       -- 2^22 / 280: V phase per virtual line at the still's period
    constant C_S0A  : integer := 5217;        -- 2^20 / 201: checker A phase per h
    constant C_S0B  : integer := 2970;        -- 2^20 / 353: checker B phase per h
    constant C_KR0  : integer := 2785;        -- 2^16 / 23.53: row phase per virtual line
    constant C_INV  : integer := 100;         -- Invert: u -> 100 - u

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
    function f_xs16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x xor shift_left(x, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        return v;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice), every other source pixel
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_v_a  : unsigned(8 downto 0) := (others => '0');
    signal r_v_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal r_fx_a : unsigned(10 downto 0) := (others => '0');
    signal r_fx_q : std_logic_vector(9 downto 0) := (others => '0');
    signal r_pa_a, r_ck_a : unsigned(8 downto 0) := (others => '0');
    signal r_pa_q, r_ck_q : std_logic_vector(29 downto 0) := (others => '0');

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
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(770, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_swp, s_pal, s_cnt, s_inv, s_src : std_logic := '0';

    -- per-field constants
    signal z_neg  : std_logic := '0';
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(9 downto 0) := (others => '0');
    signal z_t    : unsigned(20 downto 0) := (others => '0');          -- sequencer scratch
    signal z_y1, z_y2 : signed(12 downto 0) := (others => '0');
    signal c_step : unsigned(15 downto 0) := to_unsigned(16384, 16);   -- Fx phase per h (2^-16 entries)
    signal c_ks   : unsigned(17 downto 0) := to_unsigned(32768, 18);   -- ... per pixel
    signal c_xst  : unsigned(11 downto 0) := (others => '0');          -- first pixel (h)
    signal c_d0   : signed(28 downto 0) := (others => '0');            -- H phase offset at the first pixel
    signal c_kv   : unsigned(15 downto 0) := to_unsigned(C_KV0, 16);   -- V phase per virtual line (period 2^22)
    signal c_sa, c_sb : unsigned(15 downto 0) := (others => '0');      -- checker phases per h (Q20 cycles)
    signal c_sap, c_sbp : unsigned(17 downto 0) := (others => '0');    -- ... per pixel
    signal c_kr   : unsigned(15 downto 0) := to_unsigned(C_KR0, 16);   -- row phase per virtual line (Q16 rows)
    signal c_a0, c_b0 : unsigned(19 downto 0) := (others => '0');      -- checker phases at the first pixel
    signal c_scr  : unsigned(21 downto 0) := (others => '0');          -- scroll phase
    signal c_ps   : unsigned(21 downto 0) := (others => '0');          -- V phase at virtual line 0
    signal c_spd0 : unsigned(21 downto 0) := (others => '0');          -- scroll step per field
    signal c_spd  : unsigned(21 downto 0) := (others => '0');          -- ... with the wobble
    signal c_e    : signed(11 downto 0) := (others => '0');            -- Eyes offset (LSB)
    signal c_eg   : unsigned(13 downto 0) := (others => '0');          -- etch gain (Q8 per luma step)
    signal c_em   : unsigned(6 downto 0) := (others => '0');           -- ... as mantissa << c_ee
    signal c_ee   : unsigned(2 downto 0) := (others => '0');
    signal c_x0   : signed(12 downto 0) := to_signed(690, 13);         -- crack top x (virtual px)
    signal c_t    : unsigned(7 downto 0) := to_unsigned(128, 8);       -- tear amount (128 = the still)
    signal c_qen  : std_logic := '1';
    signal c_ws   : unsigned(8 downto 0) := to_unsigned(240, 9);       -- strip width (virtual px)
    signal c_yt   : signed(11 downto 0) := to_signed(280, 12);         -- Q top edge (virtual line)
    signal c_fc   : unsigned(15 downto 0) := to_unsigned(1, 16);       -- field counter
    signal c_bsd  : unsigned(15 downto 0) := to_unsigned(4321, 16);    -- band seed (every 4 fields)
    signal c_tw   : unsigned(8 downto 0) := (others => '0');           -- wobble triangle (fast)
    signal c_ty   : unsigned(8 downto 0) := to_unsigned(256, 9);       -- Q-top triangle (slow)

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');          -- next line, Q8 virtual lines
    signal e_ln   : std_logic := '0';
    signal e_bs   : unsigned(15 downto 0) := to_unsigned(1, 16);
    signal e_bct  : signed(15 downto 0) := (others => '0');            -- band: Q8 virtual lines left
    signal e_kind : std_logic_vector(1 downto 0) := "00";              -- 00 plain, 01 slip, 10 hold (active)
    signal e_dqr  : signed(6 downto 0) := (others => '0');
    signal e_slr  : signed(8 downto 0) := (others => '0');
    signal e_hx   : unsigned(8 downto 0) := (others => '0');
    signal e_hl   : unsigned(8 downto 0) := (others => '0');
    signal e_cj   : signed(2 downto 0) := (others => '0');
    signal e_psi  : unsigned(21 downto 0) := (others => '0');
    signal e_fy, e_fm, e_fyh : signed(9 downto 0) := (others => '0');
    signal e_bnew : std_logic := '0';
    signal e_z    : unsigned(2 downto 0) := (others => '0');
    signal e_rb   : unsigned(23 downto 0) := (others => '0');
    signal e_xb   : signed(12 downto 0) := (others => '0');
    signal e_h0b  : signed(13 downto 0) := (others => '0');
    signal e_xa, e_xj : signed(12 downto 0) := (others => '0');
    signal e_ro   : signed(24 downto 0) := (others => '0');
    signal e_dy   : signed(12 downto 0) := (others => '0');
    signal e_qn   : std_logic := '0';
    signal e_xm, e_hb, e_q1, e_q2 : signed(13 downto 0) := (others => '0');
    signal e_dq   : signed(9 downto 0) := (others => '0');
    signal e_sl   : signed(10 downto 0) := (others => '0');
    signal e_sg   : std_logic := '0';
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_fy, n_fyh : signed(9 downto 0) := (others => '0');
    signal n_z    : unsigned(2 downto 0) := (others => '0');
    signal n_par  : std_logic := '0';
    signal n_xbh  : signed(13 downto 0) := (others => '0');
    signal n_xqh  : signed(13 downto 0) := (others => '0');
    signal n_h0, n_h1 : signed(13 downto 0) := to_signed(8191, 14);
    signal n_dl0  : signed(28 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_x    : unsigned(11 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_d, b_dl : signed(28 downto 0) := (others => '0');
    signal b_a, b_b : unsigned(19 downto 0) := (others => '0');
    signal b_hd   : std_logic := '0';
    signal l_fy, l_fyh : signed(9 downto 0) := (others => '0');
    signal l_z    : unsigned(2 downto 0) := (others => '0');
    signal l_par  : std_logic := '0';
    signal l_xbh, l_xqh : signed(13 downto 0) := (others => '0');
    signal l_h0, l_h1 : signed(13 downto 0) := to_signed(8191, 14);
    signal l_wsh  : unsigned(9 downto 0) := (others => '0');

    -- pixel path
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_u5 is array (natural range <>) of unsigned(4 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal p_lm   : t_u8(0 to 6) := (others => (others => '0'));
    signal p_hd   : t_sl(2 to 6) := (others => '0');
    signal p2_lt, p2_q : std_logic := '0';
    signal p2_ds  : signed(28 downto 0) := (others => '0');
    signal p2_bl, p2_br : signed(12 downto 0) := (others => '0');
    signal p2_cd  : signed(14 downto 0) := (others => '0');
    signal p2_psi : unsigned(7 downto 0) := (others => '0');
    signal p2_pr  : std_logic := '0';
    signal p3_lt  : std_logic := '0';
    signal p3_ds  : signed(28 downto 0) := (others => '0');
    signal p3_vb  : unsigned(8 downto 0) := (others => '0');
    signal p3_pr  : std_logic := '0';
    signal p3_bm  : signed(12 downto 0) := (others => '0');
    signal p3_ls0, p3_ls1 : unsigned(8 downto 0) := (others => '0');
    signal p4_ls  : unsigned(9 downto 0) := (others => '0');
    signal p4_bar : std_logic := '0';
    signal p4_ub  : unsigned(7 downto 0) := (others => '0');
    signal p5_lm  : signed(8 downto 0) := (others => '0');
    signal p6_ea  : unsigned(14 downto 0) := (others => '0');
    signal p6_en  : std_logic := '0';
    signal p6_fx  : signed(9 downto 0) := (others => '0');
    signal p7_e   : signed(12 downto 0) := (others => '0');
    signal p7_u   : signed(12 downto 0) := (others => '0');
    signal p8_u   : signed(13 downto 0) := (others => '0');
    signal p9_ub  : unsigned(12 downto 0) := (others => '0');
    signal p10_b  : unsigned(15 downto 0) := (others => '0');
    signal p_s    : t_u5(4 to 11) := (others => (others => '0'));
    signal p_q    : t_sl(3 to 15) := (others => '0');
    signal p_bar  : t_sl(5 to 15) := (others => '0');
    signal p_sc   : t_sl(3 to 15) := (others => '0');      -- scar core
    signal p_sd   : t_sl(3 to 15) := (others => '0');      -- scar dark side
    signal p_ub   : t_u8(5 to 11) := (others => (others => '0'));
    signal p11_u  : signed(15 downto 0) := (others => '0');
    signal p11_f, p11_neg, p11_fr : std_logic := '0';
    signal p12_neg, p13_neg, p14_neg, p12_fr, p13_fr, p14_fr : std_logic := '0';
    signal p14_pa, p14_ck : std_logic_vector(29 downto 0) := (others => '0');
    signal p15_c  : std_logic_vector(29 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: one [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V; the 8-back taps from a block-RAM ring;
    -- the syncs ride a second ring of C_LAT
    type t_cr is array (0 to 255) of std_logic_vector(29 downto 0);
    signal cr : t_cr := (others => (others => '0'));
    type t_sr is array (0 to 255) of std_logic_vector(2 downto 0);
    signal sr : t_sr := (others => (others => '1'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of cr : signal is true;
    attribute no_rw_check of sr : signal is true;
    signal rc     : unsigned(7 downto 0) := (others => '0');
    signal cr_q, cr_f : std_logic_vector(29 downto 0) := (others => '0');
    signal sr_q, sr_f : std_logic_vector(2 downto 0) := (others => '1');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_boy, s_bou, s_bov : unsigned(13 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19, s_y20, s_u20, s_v20, s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
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

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_TT_EXP2(to_integer(r_ex_a));
            r_v_q  <= C_TT_V(to_integer(r_v_a));
            r_fx_q <= C_TT_FX(to_integer(r_fx_a));
            r_pa_q <= C_TT_PAL(to_integer(r_pa_a));
            r_ck_q <= C_TT_CHK(to_integer(r_ck_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_ab : signed(10 downto 0);
        variable v_d  : signed(11 downto 0);
        variable v_y  : signed(12 downto 0);
        variable v_t  : unsigned(9 downto 0);
        variable v_a  : unsigned(9 downto 0);
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
            elsif s_zc /= 2047 then
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
            -- Scroll: bipolar, +-8 deadband
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
                s_swp <= registers_in(6)(0);
                s_pal <= registers_in(6)(1);
                s_cnt <= registers_in(6)(2);
                s_inv <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                c_fc  <= c_fc + 1;
                if c_fc(1 downto 0) = "11" then c_bsd <= f_xs16(c_bsd); end if;
                c_tw  <= c_tw + 6;
                c_ty  <= c_ty + 2;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from load + 17
            -- (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands pre-muxed at
            -- 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_a, 36);                  z_fb <= to_unsigned(167, 16);   -- K1 exponent
                    when 1  => z_fa <= resize(z_a, 36);                  z_fb <= to_unsigned(192, 16);   -- K2 exponent
                    when 2  => z_fa <= to_unsigned(C_KV0, 36);           z_fb <= z_mt;                   -- KV
                    when 3  => z_fa <= resize(z_a, 36);                  z_fb <= to_unsigned(192, 16);   -- K5 exponent
                    when 4  => z_fa <= to_unsigned(C_S0A, 36);           z_fb <= z_mt;                   -- A step
                    when 5  => z_fa <= to_unsigned(C_S0B, 36);           z_fb <= z_mt;                   -- B step
                    when 6  => z_fa <= to_unsigned(C_KR0, 36);           z_fb <= z_mt;                   -- row step
                    when 7  => z_fa <= resize(C_XC2 - c_xst, 36);       z_fb <= c_step;                 -- D0
                    when 8  => z_fa <= resize(2000 - c_xst, 36);        z_fb <= c_sa;                   -- A0
                    when 9  => z_fa <= resize(2000 - c_xst, 36);        z_fb <= c_sb;                   -- B0
                    when 10 => z_fa <= resize(s_vm, 36);                 z_fb <= resize(s_vm, 16);       -- d^2
                    when 11 => z_fa <= resize(z_t, 36);                  z_fb <= to_unsigned(4244, 16);  -- lines / field
                    when 12 => z_fa <= resize(z_t, 36);                  z_fb <= c_kv;                   -- speed
                    when 13 => z_fa <= resize(c_spd0, 36);               z_fb <= resize(f_tri7(c_tw(8 downto 1)) & '0', 16);  -- wobble
                    when 14 => z_fa <= resize(c_kv, 36);                 z_fb <= to_unsigned(232, 16);   -- 232 KV
                    when 15 => z_fa <= resize(s_g6(12 downto 3), 36);    z_fb <= resize(s_g6(12 downto 3), 16);  -- k6^2
                    when others =>                                                                       -- crack X0
                        if s_gp(12 downto 3) > 512 then
                            z_fa <= resize(s_gp(12 downto 3) - 512, 36); z_fb <= to_unsigned(1623, 16);
                        else
                            z_fa <= resize(s_gp(12 downto 3), 36);       z_fb <= to_unsigned(3180, 16);
                        end if;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- scroll (last field's step); line-start constants
                    when 20 =>
                        if s_vneg = '1' then c_scr <= c_scr - c_spd; else c_scr <= c_scr + c_spd; end if;
                        c_xst <= resize(s_x0, 12);
                    when 28 =>                        -- columns: a = |k1 - 512|
                        v_s := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        z_neg <= v_s(10); v_ab := abs(v_s); z_a <= unsigned(v_ab(9 downto 0));
                    when 50  =>                       -- e = +-(a * 167 >> 8): +-1.3 octaves
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 53  =>                       -- step = 16384 * 2^(e / 256)
                        case to_integer(z_sh) is
                            when -2     => c_step <= shift_right(r_ex_q, 3);
                            when -1     => c_step <= shift_right(r_ex_q, 2);
                            when 0      => c_step <= shift_right(r_ex_q, 1);
                            when others => c_step <= r_ex_q;
                        end case;
                        v_s := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        z_neg <= v_s(10); v_ab := abs(v_s); z_a <= unsigned(v_ab(9 downto 0));
                    when 56 =>
                        case to_integer(s_xs) is
                            when 4      => c_ks <= shift_left(resize(c_step, 18), 2);
                            when 3      => c_ks <= shift_left(resize(c_step, 18), 1) + resize(c_step, 18);
                            when others => c_ks <= shift_left(resize(c_step, 18), 1);
                        end case;
                    when 74  =>                       -- rows: e = +-(a * 192 >> 8): +-1.5 octaves
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 77  => z_mt <= r_ex_q;
                        -- checker: a = |k5 - 512|, bigger blocks to the right (negative exponent)
                        v_s := signed(resize(s_g5(12 downto 3), 11)) - 512;
                        z_neg <= not v_s(10); v_ab := abs(v_s); z_a <= unsigned(v_ab(9 downto 0));
                    when 98  =>                       -- KV = KV0 mt >> (15 - sh)
                        case to_integer(z_sh) is
                            when -2     => c_kv <= f_acc(32 downto 17);
                            when -1     => c_kv <= f_acc(31 downto 16);
                            when 0      => c_kv <= f_acc(30 downto 15);
                            when others => c_kv <= f_acc(29 downto 14);
                        end case;
                    when 122 =>
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 125 => z_mt <= r_ex_q;
                    when 146 | 170 | 194 =>           -- A / B / row steps = S0 mt >> (15 - sh)
                        case to_integer(z_sh) is
                            when -2     => v_t := f_acc(26 downto 17);  z_t <= resize(f_acc(32 downto 17), 21);
                            when -1     => v_t := f_acc(25 downto 16);  z_t <= resize(f_acc(31 downto 16), 21);
                            when 0      => v_t := f_acc(24 downto 15);  z_t <= resize(f_acc(30 downto 15), 21);
                            when others => v_t := f_acc(23 downto 14);  z_t <= resize(f_acc(29 downto 14), 21);
                        end case;
                    when 147 => c_sa <= z_t(15 downto 0);
                    when 171 => c_sb <= z_t(15 downto 0);
                    when 195 => c_kr <= z_t(15 downto 0);
                    when 196 =>
                        case to_integer(s_xs) is
                            when 4      => c_sap <= shift_left(resize(c_sa, 18), 2); c_sbp <= shift_left(resize(c_sb, 18), 2);
                            when 3      => c_sap <= shift_left(resize(c_sa, 18), 1) + resize(c_sa, 18);
                                           c_sbp <= shift_left(resize(c_sb, 18), 1) + resize(c_sb, 18);
                            when others => c_sap <= shift_left(resize(c_sa, 18), 1); c_sbp <= shift_left(resize(c_sb, 18), 1);
                        end case;
                    when 218 => c_d0 <= -signed(resize(f_acc(27 downto 0), 29));
                    when 242 => c_a0 <= to_unsigned(104858, 20) - f_acc(19 downto 0);
                    when 266 => c_b0 <= to_unsigned(62915, 20) - f_acc(19 downto 0);
                    when 290 => z_t <= f_acc(20 downto 0);                                   -- d^2 (<= 253009)
                    when 314 => z_t <= f_acc(30 downto 10);                                  -- Q16 lines / field
                    when 338 => c_spd0 <= resize(f_acc(35 downto 16), 22);                    -- (t kv) >> 16
                    when 362 => c_spd <= c_spd0 - shift_right(c_spd0, 2) + f_acc(30 downto 9);
                    when 386 => c_ps <= c_scr - f_acc(21 downto 0);
                    when 410 => z_t <= f_acc(20 downto 0);
                    when 434 =>                       -- etch gain (Q8 per luma step): camera k6^2 / 2048 (a gentle bend at
                                                      -- the default, the same tear at full); the Osc 2 k6 (as v0.1.0)
                        if s_src = '1' then c_eg <= resize(s_g6(12 downto 3) & '0', 14);
                        else c_eg <= resize(z_t(20 downto 11), 14); end if;
                        c_ee <= (others => '0');
                    when 436 | 437 | 438 | 439 | 440 | 441 | 442 =>   -- normalise the etch gain to 7 bits << ee
                        if c_eg(13 downto 7) /= "0000000" then c_eg <= shift_right(c_eg, 1); c_ee <= c_ee + 1; end if;
                    when 443 => c_em <= c_eg(6 downto 0);
                    when 446 =>                       -- Eyes: (k3 - 512) * 0.39 LSB to the left, * 0.28 to the right
                        -- (on the magnitude: no sign-extended self-adds, which hang router1)
                        if s_g3(12) = '1' then
                            v_a := resize(s_g3(12 downto 3) - 512, 10);
                            c_e <= signed(resize(shift_right(v_a, 2) + shift_right(v_a, 5), 12));
                        else
                            v_a := to_unsigned(512, 10) - resize(s_g3(12 downto 3), 10);
                            c_e <= -signed(resize(shift_right(v_a, 2) + shift_right(v_a, 3) + shift_right(v_a, 6), 12));
                        end if;
                        -- tear amount and strip width
                        c_t <= s_gp(12 downto 5);
                        if s_gp(12 downto 3) < 32 then c_qen <= '0'; else c_qen <= '1'; end if;
                        if s_gp(12 downto 11) /= "00" then c_ws <= to_unsigned(240, 9);
                        else c_ws <= resize(shift_right(s_gp(10 downto 3) * to_unsigned(15, 4), 4), 9); end if;
                    when 448 =>                       -- Q top: 540 - 2.03 T, +-160 on the slow triangle
                        z_y1 <= to_signed(540, 13) - signed(resize(c_t & '0', 13)) - signed(resize(c_t(7 downto 5), 13));
                        z_y2 <= signed(resize(f_tri7(c_ty(8 downto 1)) & '0', 13)) + signed(resize(shift_right(f_tri7(c_ty(8 downto 1)), 1), 13));
                    when 449 =>
                        z_y1 <= z_y1 + z_y2 - 160;
                    when 450 =>
                        if z_y1 < 0 then c_yt <= (others => '0'); else c_yt <= z_y1(11 downto 0); end if;
                    when 482 =>                       -- crack X0
                        if s_gp(12 downto 3) > 512 then c_x0 <= to_signed(690, 13) + signed(resize(f_acc(20 downto 10), 13));
                        else c_x0 <= to_signed(-900, 13) + signed(resize(f_acc(20 downto 10), 13)); end if;
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
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol(10 downto 1);
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17):
    --   psi = vy KV + ps -> V ROM -> Fy, zone; the FM read (psi - 218 entries)
    --   the row phase vy kr + 376 FM -> parity; the crack X0 + 49 vy^2 / 2^16 + Fy / 32 + jitter
    --   the tear bands (even lines: an xorshift draws kind, height, Q-edge offset, slip, hold), the Q edge,
    --   the band's Q offset and slip times the tear amount, the slipped L-regime start
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_r  : unsigned(15 downto 0);
        variable v_i  : signed(9 downto 0);
        variable v_p  : unsigned(21 downto 0);
        variable v_ro : signed(24 downto 0);
        variable v_x  : signed(13 downto 0);
        variable v_dy : signed(12 downto 0);
        variable v_q  : signed(13 downto 0);
        variable v_h  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_ln <= '0';
                e_bs <= c_bsd;
                e_bct <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_ln <= not e_ln;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_bnew <= '0';
                    -- tear band (even lines only, so every band is 2 lines at least): a new draw when it runs out
                    if e_ln = '0' then
                        e_bs <= f_xs16(e_bs);
                        if e_bct <= 0 then
                            e_bnew <= '1';
                            v_r := e_bs;
                            if v_r(15 downto 13) = "111" then
                                e_bct <= signed(resize(shift_left(resize(v_r(1 downto 0), 16), 8), 16))
                                         + to_signed(1024, 16) - signed(resize(s_ys & '0', 16));
                            else
                                e_bct <= signed(resize(shift_left(resize(v_r(5 downto 0), 16), 8), 16))
                                         + to_signed(2048, 16) - signed(resize(s_ys & '0', 16));
                            end if;
                            if v_r(15 downto 13) = "101" or v_r(15 downto 13) = "110" then
                                e_kind <= "01";
                            elsif v_r(15 downto 13) = "111" and v_r(10 downto 8) < c_t(7 downto 5) then
                                e_kind <= "10";
                            else
                                e_kind <= "00";
                            end if;
                            e_dqr <= signed(v_r(12 downto 6)) ;
                            v_i := signed(resize(shift_left(resize(v_r(11 downto 7), 10), 2), 10)) + 20;
                            if v_r(0) = '1' then e_slr <= -v_i(8 downto 0); else e_slr <= v_i(8 downto 0); end if;
                            e_hx <= resize(shift_left(resize(v_r(9 downto 4), 9), 3), 9);
                            e_hl <= to_unsigned(96, 9) + resize(shift_left(resize(v_r(12 downto 9), 9), 4), 9)
                                    + resize(shift_left(resize(v_r(12 downto 9), 9), 3), 9);
                            e_cj <= signed(v_r(2 downto 0));
                        else
                            e_bct <= e_bct - signed(resize(s_ys & '0', 16));
                        end if;
                    end if;
                when 2 =>
                    m_a <= resize(e_vy, 36); m_b <= c_kv; m_acc <= (others => '0');
                when 19 =>
                    v_p := m_acc(29 downto 8) + c_ps;
                    e_psi <= v_p;
                    r_v_a <= v_p(21 downto 13);
                when 20 =>
                    m_a <= resize(e_vy, 36); m_b <= c_kr; m_acc <= (others => '0');
                when 21 =>
                    e_fy <= signed(r_v_q(9 downto 0));
                    e_z  <= unsigned(r_v_q(12 downto 10));
                    if e_bnew = '1' then e_fyh <= signed(r_v_q(9 downto 0)); end if;
                    r_v_a <= e_psi(21 downto 13) - 218;
                when 23 =>
                    e_fm <= signed(r_v_q(9 downto 0));
                when 37 =>
                    e_rb <= m_acc(31 downto 8);
                    e_sg <= e_fm(9);
                    m_a <= resize(unsigned(abs(e_fm)), 36); m_b <= to_unsigned(376, 16); m_acc <= (others => '0');
                when 54 =>
                    -- row phase (Q16 rows): vy kr + FM - 14205
                    if e_sg = '1' then e_ro <= signed(resize(e_rb, 25)) - signed(resize(m_acc(16 downto 0), 25)) - 14205;
                    else e_ro <= signed(resize(e_rb, 25)) + signed(resize(m_acc(16 downto 0), 25)) - 14205; end if;
                    n_fy <= e_fy; n_fyh <= e_fyh; n_z <= e_z;
                    m_a <= resize(e_vy(18 downto 8), 36); m_b <= resize(e_vy(18 downto 8), 16); m_acc <= (others => '0');
                when 56 =>
                    if e_ln = '0' then n_par <= e_ro(16); end if;      -- row edges on even lines (no 1-line slivers)
                when 71 =>
                    m_a <= resize(m_acc(21 downto 0), 36); m_b <= to_unsigned(49, 16); m_acc <= (others => '0');
                when 88 =>
                    -- crack: X0 + 49 vy^2 / 2^16 + Fy / 32 + jitter (virtual px)
                    e_xa <= c_x0 + signed(resize(m_acc(25 downto 16), 13));
                    e_xj <= resize(shift_right(e_fy, 5), 13) + resize(e_cj, 13);
                    e_sg <= e_dqr(6);
                    m_a <= resize(unsigned(abs(resize(e_dqr, 8))), 36); m_b <= resize(c_t, 16); m_acc <= (others => '0');
                when 89 =>
                    e_xb <= e_xa + e_xj;
                when 90 =>
                    n_xbh <= shift_left(resize(e_xb, 14), 1);
                when 105 =>
                    if e_sg = '1' then e_dq <= -signed(resize(m_acc(14 downto 6), 10));
                    else e_dq <= signed(resize(m_acc(14 downto 6), 10)); end if;
                    e_sg <= e_slr(8);
                    if e_kind = "01" then
                        m_a <= resize(unsigned(abs(resize(e_slr, 10))), 36);
                    else
                        m_a <= (others => '0');
                    end if;
                    m_b <= resize(c_t, 16); m_acc <= (others => '0');
                when 106 =>
                    -- Q edge: 340 + 1.094 (vy - top) + dq below the top, then left of the crack - 40
                    v_dy := signed(resize(e_vy(18 downto 8), 13)) - resize(c_yt, 13);
                    e_dy <= v_dy;
                    if c_qen = '0' or v_dy < 0 then e_qn <= '1'; else e_qn <= '0'; end if;
                    e_xm <= resize(e_xb, 14) - 40;
                    e_hb <= resize(e_xb, 14) - 40 - signed(resize(e_hx, 14));
                when 107 =>
                    e_q1 <= to_signed(340, 14) + resize(e_dy, 14) + resize(e_dq, 14);
                    e_q2 <= signed(resize(shift_right(unsigned(e_dy(11 downto 0)), 3) - shift_right(unsigned(e_dy(11 downto 0)), 5), 14));
                    -- hold window (h), only in an active hold band
                    -- (latched at the band's first line, so every line of a held band freezes the same phase)
                    if e_bnew = '1' then e_h0b <= shift_left(e_hb, 1); end if;
                when 108 =>
                    if e_qn = '1' then e_q1 <= to_signed(-64, 14); else e_q1 <= e_q1 + e_q2; end if;
                    if e_kind = "10" and c_qen = '1' then
                        n_h0 <= e_h0b;
                        n_h1 <= e_h0b + signed(resize(e_hl & '0', 14));
                    else
                        n_h0 <= to_signed(8191, 14); n_h1 <= to_signed(8191, 14);
                    end if;
                when 109 =>
                    if e_ln = '0' then
                        if e_q1 > e_xm then n_xqh <= shift_left(e_xm, 1); else n_xqh <= shift_left(e_q1, 1); end if;
                    end if;
                when 122 =>
                    -- slip (virtual px) = raw x tear / 128; times 2 c_step = the L phase offset
                    if e_sg = '1' then e_sl <= -signed(resize(m_acc(16 downto 7), 11));
                    else e_sl <= signed(resize(m_acc(16 downto 7), 11)); end if;
                    m_a <= resize(shift_left(resize(m_acc(16 downto 7), 11), 1), 36); m_b <= c_step; m_acc <= (others => '0');
                when 139 =>
                    if e_sl(10) = '1' then n_dl0 <= c_d0 + signed(resize(m_acc(27 downto 0), 29));
                    else n_dl0 <= c_d0 - signed(resize(m_acc(27 downto 0), 29)); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: x (h), the H phase D = (x - XC2) step, the L-regime phase DL (the
    -- band's slip, frozen through the hold window), the two checker
    -- oscillators; the per-line values load at the line start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_nx : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= resize(s_x0, 12);
                b_px <= (others => '0');
                b_d  <= c_d0;
                b_dl <= n_dl0;
                b_a  <= c_a0;
                b_b  <= c_b0;
                if signed(resize(s_x0, 14)) >= n_h0 and signed(resize(s_x0, 14)) < n_h1 then b_hd <= '1'; else b_hd <= '0'; end if;
                l_fy <= n_fy; l_fyh <= n_fyh; l_z <= n_z; l_par <= n_par;
                l_xbh <= n_xbh; l_xqh <= n_xqh; l_h0 <= n_h0; l_h1 <= n_h1;
                if c_qen = '1' then l_wsh <= c_ws & '0'; else l_wsh <= (others => '0'); end if;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
                b_d  <= b_d + signed(resize(c_ks, 29));
                if b_hd = '0' then b_dl <= b_dl + signed(resize(c_ks, 29)); end if;
                b_a  <= b_a + c_sap;
                b_b  <= b_b + c_sbp;
                v_nx := signed(resize(b_x + s_xs, 14));
                if v_nx >= l_h0 and v_nx < l_h1 then b_hd <= '1'; else b_hd <= '0'; end if;
            end if;
            s_lb_ra <= b_px(10 downto 1);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_bx  : signed(13 downto 0);
        variable v_ps  : unsigned(7 downto 0);
        variable v_ds  : signed(28 downto 0);
        variable v_ix27 : signed(26 downto 0);
        variable v_sv  : signed(11 downto 0);
        variable v_ab  : signed(14 downto 0);
        variable v_al  : signed(8 downto 0);
        variable v_e21 : unsigned(21 downto 0);
        variable v_es  : signed(15 downto 0);
        variable v_u9  : signed(14 downto 0);
        variable v_ub  : signed(12 downto 0);
        variable v_ck  : unsigned(8 downto 0);
        variable v_c   : std_logic_vector(29 downto 0);
    begin
        if rising_edge(clk) then
            v_bx := signed(resize(b_x, 14));
            -- 2: regime (left of the crack = crosses, unless swapped), the phase to use, Q / strip, bar distances,
            -- crack distance, the checker phase of this row; luma history (camera: 0 off the line; the Osc: its mean)
            if (v_bx < l_xbh) xor (s_swp = '1') then p2_lt <= '1'; else p2_lt <= '0'; end if;
            if v_bx < l_xbh then p2_ds <= b_dl; else p2_ds <= b_d; end if;
            if v_bx < l_xqh or b_x < l_wsh then p2_q <= '1'; else p2_q <= '0'; end if;
            p2_bl <= signed(resize(b_x, 13)) - 480;
            p2_br <= to_signed(1480, 13) - signed(resize(b_x, 13));
            p2_cd <= resize(v_bx, 15) - resize(l_xbh, 15);
            if l_par = '1' then p2_psi <= b_b(19 downto 12); else p2_psi <= b_a(19 downto 12); end if;
            p2_pr <= l_par;
            p_hd(2) <= b_hd;
            for i in 3 to 6 loop p_hd(i) <= p_hd(i - 1); end loop;
            if s_gav_sr(0) = '1' then p_lm(0) <= unsigned(s_lb_q);
            elsif s_src = '1' then p_lm(0) <= to_unsigned(110, 8); else p_lm(0) <= (others => '0'); end if;
            for i in 1 to 6 loop p_lm(i) <= p_lm(i - 1); end loop;
            -- 3: the phase (as is), the sawtooth's start (Q: half a cycle on), the bar distance, the scar, luma pairs
            p3_ds <= p2_ds;
            p3_lt <= p2_lt;
            v_ps := p2_psi;
            if p2_q = '1' then v_ps(7) := not v_ps(7); end if;
            -- (biased by 128 so the sawtooth's shift-adds run on an unsigned value: no sign-extended self-adds)
            if p2_pr = '1' then p3_vb <= resize(v_ps, 9) + 27; else p3_vb <= resize(v_ps, 9) + 41; end if;
            p3_pr <= p2_pr;
            if p2_bl < p2_br then p3_bm <= p2_bl; else p3_bm <= p2_br; end if;
            p_q(3) <= p2_q;
            v_ab := abs(p2_cd);
            if c_qen = '1' and v_ab < 5 then p_sc(3) <= '1'; else p_sc(3) <= '0'; end if;
            if c_qen = '1' and v_ab >= 5 and v_ab < 10 then p_sd(3) <= '1'; else p_sd(3) <= '0'; end if;
            p3_ls0 <= resize(p_lm(0), 9) + p_lm(2); p3_ls1 <= resize(p_lm(4), 9) + p_lm(6);    -- 8 px box (4 taps)
            -- 4: Fx ROM address {regime, index}; the checker s (A: 2.25 (psi - 97), B: 3.75 (psi - 113), clipped);
            -- the bar index (first palette cycle, white from 92); luma 4-sum
            v_ix27 := to_signed(C_BASE, 27) + resize(p3_ds, 27);
            r_fx_a <= (not p3_lt) & unsigned(v_ix27(25 downto 16));
            if p3_pr = '1' then
                v_sv := signed(shift_left(resize(p3_vb, 12), 2) - shift_right(resize(p3_vb, 12), 2)) - 480;
            else
                v_sv := signed(shift_left(resize(p3_vb, 12), 1) + shift_right(resize(p3_vb, 12), 2)) - 288;
            end if;
            if v_sv < 0 then p_s(4) <= (others => '0');
            elsif v_sv > 255 then p_s(4) <= (others => '1');
            else p_s(4) <= unsigned(v_sv(7 downto 3)); end if;
            if p3_bm < 0 then p4_bar <= '0'; else p4_bar <= '1'; end if;
            v_ub := shift_right(p3_bm, 1) + 10;
            if v_ub > 131 then p4_ub <= to_unsigned(131, 8); else p4_ub <= unsigned(v_ub(7 downto 0)); end if;
            p_q(4) <= p_q(3);
            p4_ls <= resize(p3_ls0, 10) + p3_ls1;
            for i in 5 to 11 loop p_s(i) <= p_s(i - 1); end loop;
            for i in 5 to 15 loop p_q(i) <= p_q(i - 1); end loop;
            for i in 4 to 15 loop p_sc(i) <= p_sc(i - 1); p_sd(i) <= p_sd(i - 1); end loop;
            p_bar(5) <= p4_bar;
            for i in 6 to 15 loop p_bar(i) <= p_bar(i - 1); end loop;
            p_ub(5) <= p4_ub;
            for i in 6 to 11 loop p_ub(i) <= p_ub(i - 1); end loop;
            -- 5: Fx ROM (EBR); luma (camera adds: black = the still; the Osc: centred)
            if s_src = '1' then p5_lm <= signed(resize(p4_ls(9 downto 2), 9)) - 110;
            else p5_lm <= signed(resize(p4_ls(9 downto 2), 9)); end if;
            -- 6: fabric copy of Fx; etch partials (|luma| x gain)
            p6_fx <= signed(r_fx_q);
            v_al := abs(p5_lm);
            p6_ea <= unsigned(v_al(7 downto 0)) * c_em;
            p6_en <= p5_lm(8);
            -- 7: u = Fx + Fy + Eyes; E = +-(luma gain) >> 8
            -- (the hold window keeps the band's first-line Fy, so a held streak is one colour down the band)
            if p_hd(6) = '1' then p7_u <= resize(p6_fx, 13) + resize(l_fyh, 13) + resize(c_e, 13);
            else p7_u <= resize(p6_fx, 13) + resize(l_fy, 13) + resize(c_e, 13); end if;
            v_e21 := shift_left(resize(p6_ea, 22), to_integer(c_ee));
            v_es := signed(resize(v_e21(21 downto 8), 16));
            if p6_en = '1' then v_es := -v_es; end if;
            if v_es > 2047 then p7_e <= to_signed(2047, 13);
            elsif v_es < -2048 then p7_e <= to_signed(-2048, 13);
            else p7_e <= v_es(12 downto 0); end if;
            -- 8: + the camera / Osc
            p8_u <= resize(p7_u, 14) + resize(p7_e, 14);
            -- 9: Invert (100 - u), biased by 4096 (u stays within +-4095): the Contours add then runs unsigned
            if s_inv = '1' then v_u9 := to_signed(C_INV + 4096, 15) - resize(p8_u, 15);
            else v_u9 := resize(p8_u, 15) + 4096; end if;
            p9_ub <= unsigned(v_u9(12 downto 0));
            -- 10: Contours (3 u), both ways biased by 12288 = 48 * 256
            if s_cnt = '1' then p10_b <= shift_left(resize(p9_ub, 16), 1) + resize(p9_ub, 16);
            else p10_b <= resize(p9_ub, 16) + 8192; end if;
            -- 11: the checker side and its fringe; the first palette cycle
            p11_u <= signed(p10_b);
            if p10_b(15 downto 8) = to_unsigned(48, 8) then p11_f <= '1'; else p11_f <= '0'; end if;
            if p10_b < 12288 then p11_neg <= '1'; else p11_neg <= '0'; end if;
            if p10_b < 12288 and p10_b >= 12272 then p11_fr <= '1'; else p11_fr <= '0'; end if;
            -- 12: ROM addresses: the tube palette {first cycle, t} (the bar: its own first-cycle index), the checker
            -- {set (Q xor Lagoon), zone, s}
            if p_q(11) = '1' and p_bar(11) = '1' then
                r_pa_a <= '1' & p_ub(11);
            else
                r_pa_a <= p11_f & unsigned(p11_u(7 downto 0));
            end if;
            v_ck := (p_q(11) xor s_pal) & l_z & p_s(11);
            r_ck_a <= v_ck;
            p12_neg <= p11_neg; p12_fr <= p11_fr;
            -- 13: palette ROMs (EBR); 14: fabric copies
            p13_neg <= p12_neg; p13_fr <= p12_fr;
            p14_pa <= r_pa_q; p14_ck <= r_ck_q;
            p14_neg <= p13_neg; p14_fr <= p13_fr;
            -- 15: tube or checker (Q: the bar or the Q checker), the fringe, the crack scar
            if p_q(14) = '1' then
                if p_bar(14) = '1' then v_c := p14_pa; else v_c := p14_ck; end if;
            elsif p14_neg = '1' then
                if p14_fr = '1' then v_c := C_TT_FRINGE; else v_c := p14_ck; end if;
            else
                v_c := p14_pa;
            end if;
            if p_sc(14) = '1' then v_c := C_TT_SCAR; elsif p_sd(14) = '1' then v_c := C_TT_SCARD; end if;
            p15_c <= v_c;
            -- 16: black off the generated avid
            if s_gav_sr(13) = '0' then
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            else
                s_y16 <= unsigned(p15_c(29 downto 20)); s_u16 <= unsigned(p15_c(19 downto 10)); s_v16 <= unsigned(p15_c(9 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(13);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Rings: the box's 8-back taps (written at rc + 6, RAM register, fabric
    -- register = the input of 8 clocks before; never the write address) and the
    -- syncs (written at rc + 18: C_LAT clocks)
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rc <= rc + 1;
            cr(to_integer(rc + 6)) <= std_logic_vector(s_y16) & std_logic_vector(s_u16) & std_logic_vector(s_v16);
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + 18)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box
    -- (two adjacent 8-sums; the HD HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 17: running 8-sums (snap back to black once 8 blank samples are in, so a glitched sample can't
            -- offset the colours for good)
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y16 - unsigned(cr_f(29 downto 20));
                s_bau <= s_bau + s_u16 - unsigned(cr_f(19 downto 10));
                s_bav <= s_bav + s_v16 - unsigned(cr_f(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- 18: [1 2 2 2 2 2 2 2 1] = two adjacent 8-sums
            s_boy <= resize(s_bay, 14) + s_bayd;
            s_bou <= resize(s_bau, 14) + s_baud;
            s_bov <= resize(s_bav, 14) + s_bavd;
            -- 19: / 16; black off the generated avid
            if s_gav_sr(16) = '1' then
                s_y19 <= s_boy(13 downto 4); s_u19 <= s_bou(13 downto 4); s_v19 <= s_bov(13 downto 4);
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture tatter;
