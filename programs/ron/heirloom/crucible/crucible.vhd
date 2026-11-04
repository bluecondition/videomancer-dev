-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Crucible v0.1.1 -- Heirloom from the video clips, built from the still WIN_20230518_23_23_28 t0.5
-- v0.1.1: S7 Twin On = two separate lens columns at +-420 virtual px from the axis (one each side of the
-- figure); S10 Tear On = frequent large sync-tear bursts (1 start in 8 lines, 8..39 lines, 40..198 vpx).
-- Both Off states unchanged (bit-identical with v0.1.0).
-- (an 8.7 s clip: the flame stripes roll down at 172 lines/s, period 2.07 s, and the black figure
-- breathes between a head-and-shoulders and a horizontal diamond band with the same period).
--
-- The still: an orange/yellow flame cross on magenta.  Violet ground with hot-pink/red flame wedges
-- pointing at the centre column, an orange/yellow X band across the middle flanked by red, a black
-- head-and-shoulders figure above it and an inverted black diamond below, pale striated rims and mauve
-- shading around the black, and a column of stacked blue lenses (offset left of the axis) that the
-- cross turns cream where it passes.
--
-- The patch, as fitted (crucible.py has every table; .heirloom_work/rb_crucible/notes.md the measurements):
--   W(y)  = cos(2 pi ((y - 102) / P - rp))      the rolling vertical oscillator (P 356 lines)
--   F     = W + 1.4 |x - 992| / 960 + heat + camera
--   zone  = |x - 992| < hw(y)                    per-line half-width: head <-> diamond tables, lerped by the
--                                                morph b (locked to the roll), scaled by Figure
--   inside the zone: yellow/orange (cream over a lens) where F > TY(y), the lens where F > TL(y) or on a
--                    trough line (the eyes, cyan rims), black elsewhere
--   outside: the lens; red/pink where F > TR(y) (or F > TB(y) in the border band); the pale rim; violet
--            shaded by y with mauve toward the figure
--   lens column: (x - 875)^2 < LT(y), LT from the chirped row table, breathing 9 % with the roll
--
--   K1  Stripes   the flame stripes' period (x0.35 .. x2.8; 512 = the photo's 356 lines)
--   K2  Roll      bipolar roll speed (still in the centre +-8); 770 = the clip's 172 lines/s down
--   K3  Figure    the black figure's size (x0.44 .. x2.3)
--   K4  Morph     0 = the head holds, 512 = the clip's head <-> diamond breathing, 1023 = the diamond holds
--   K5  Lenses    the blue lenses' size: slivers .. the photo (512) .. fat lenses merging into a column
--   K6  Camera    how far the camera's light (or the Osc) heats the flame field
--   S7  Twin      one lens column (the photo, offset) / two columns at +-420 vpx, one each side of the figure
--   S8  Flip      the figure upright (head on top) / inverted
--   S9  Palette   Flame (the photo) / Ice (Cr and Cb swapped: a blue cross on teal and orange)
--   S10 Tear      Off (the photo's faint line jitter) / On (sync tear: bursts of 8..39 lines shifted 40..198 vpx)
--   S11 Source    Video / Osc
--   P12 HEAT      the flame level: 0 = cold (violet, black figure, lenses), 546 = the photo, 100 % = molten
--
-- Engine: vblank-zone sequencer (shared glide, exp2 / cos ROM knob curves, one serial multiplier for the
-- per-field products), a per-line engine kicked at each line start computing the NEXT line (the roll
-- phase and W, the zone half-widths from the tables, the flame thresholds, the lens row threshold, the
-- tear offset and the lens-DDA starts), a beam with two exact second-order lens DDAs, a 15-stage pixel
-- path ending in a 1024-entry palette, then the shell's output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps sit in block-RAM rings, luma 13/16, blanking gate).
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Colours in the hardware convention.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.crucible_pkg.all;
use work.all;

architecture crucible of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

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

    attribute no_rw_check : boolean;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a : unsigned(7 downto 0) := (others => '0');
    signal r_cs_q : std_logic_vector(9 downto 0) := (others => '0');
    signal z_csa, e_csa : unsigned(7 downto 0) := (others => '0');
    signal s_csz  : std_logic := '0';
    signal r_zn_a : unsigned(9 downto 0) := (others => '0');
    signal r_zn_q : std_logic_vector(11 downto 0) := (others => '0');
    signal r_ln_a : unsigned(8 downto 0) := (others => '0');
    signal r_ln_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_pa_a : unsigned(9 downto 0) := to_unsigned(448, 10);
    signal r_py_q, r_pu_q, r_pv_q : std_logic_vector(7 downto 0) := (others => '0');

    -- output box rings (the 8-back taps of the two running sums) and the sync delay ring
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ba1, ba2, bb1, bb2 : t_r16 := (others => x"0000");
    attribute no_rw_check of ba1 : signal is true;
    attribute no_rw_check of ba2 : signal is true;
    attribute no_rw_check of bb1 : signal is true;
    attribute no_rw_check of bb2 : signal is true;
    signal rg_wa, rg_ra, sy_ra : unsigned(7 downto 0) := (others => '0');
    signal ba1_q, ba2_q, bb1_q, bb2_q : std_logic_vector(15 downto 0) := x"0000";
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal syr : t_r4 := (others => "1111");
    attribute no_rw_check of syr : signal is true;
    signal syr_q, syr_c : std_logic_vector(3 downto 0) := "1111";

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

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(770 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(546 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_twin, s_flip, s_ice, s_tear, s_src : std_logic := '0';

    -- per-field constants
    signal z_sh1, z_sh3, z_sh3n : signed(2 downto 0) := (others => '0');
    signal z_e3   : signed(11 downto 0) := (others => '0');
    signal z_mt1, z_mt3, z_mt3n : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_invp : unsigned(9 downto 0) := to_unsigned(184, 10);    -- roll phase per virtual line, Q16 cycles
    signal c_s    : signed(10 downto 0) := (others => '0');           -- speed knob, deadbanded
    signal c_s2   : unsigned(17 downto 0) := (others => '0');
    signal c_vneg : std_logic := '0';
    signal c_vst  : unsigned(15 downto 0) := (others => '0');         -- |roll step| per field
    signal c_rp   : unsigned(15 downto 0) := (others => '0');         -- roll phase (Q16 cycles)
    signal c_ph0  : unsigned(15 downto 0) := (others => '0');         -- W phase at virtual line 0
    signal c_b8   : unsigned(7 downto 0) := (others => '0');          -- morph LFO 0 .. 255
    signal c_b    : unsigned(8 downto 0) := (others => '0');          -- morph after the K4 bias 0 .. 256
    signal c_cb, c_sn : signed(9 downto 0) := (others => '0');
    signal c_ls8  : unsigned(8 downto 0) := to_unsigned(256, 9);      -- lens row scale (breathing), Q8
    signal c_zs8, c_izs8 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_th8  : signed(9 downto 0) := to_signed(166, 10);         -- lens threshold, Q8
    signal c_g    : unsigned(4 downto 0) := to_unsigned(4, 5);        -- camera gain
    signal c_h8   : signed(10 downto 0) := (others => '0');           -- heat, Q8
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal c_dd   : unsigned(5 downto 0) := to_unsigned(8, 6);        -- 2 xs^2
    signal c_xsq  : unsigned(4 downto 0) := to_unsigned(4, 5);        -- xs^2

    -- field serial multiplier
    signal f_a   : unsigned(27 downto 0) := (others => '0');
    signal f_b   : unsigned(17 downto 0) := (others => '0');
    signal f_acc : unsigned(27 downto 0) := (others => '0');
    signal z_fa  : unsigned(27 downto 0) := (others => '0');
    signal z_fb  : unsigned(17 downto 0) := (others => '0');
    signal z_sel : unsigned(2 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal z_k   : unsigned(2 downto 0) := (others => '0');
    signal z_kc  : unsigned(4 downto 0) := (others => '0');
    signal z_run : std_logic := '0';

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
    signal e_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal e_bc   : unsigned(5 downto 0) := (others => '0');          -- tear burst lines left
    signal e_bo   : signed(9 downto 0) := (others => '0');            -- tear burst offset (h)
    signal e_bm   : unsigned(8 downto 0) := (others => '0');          -- tear burst magnitude (h)
    signal e_vl   : unsigned(10 downto 0) := (others => '0');
    signal e_vf   : unsigned(10 downto 0) := (others => '0');
    signal e_d, e_d2, e_dl, e_dv : signed(11 downto 0) := (others => '0');
    signal e_ad, e_ad2, e_adl, e_adv : unsigned(10 downto 0) := (others => '0');
    signal e_top  : std_logic := '0';
    signal e_w8   : signed(9 downto 0) := (others => '0');
    signal e_adz, e_d2z : unsigned(10 downto 0) := (others => '0');
    signal e_md   : unsigned(10 downto 0) := (others => '0');
    signal e_s1, e_s2, e_s3, e_s4 : unsigned(19 downto 0) := (others => '0');
    signal e_mty : unsigned(11 downto 0) := (others => '0');
    signal e_mtl, e_tr : unsigned(10 downto 0) := (others => '0');
    signal e_h1, e_h1b, e_h2, e_h2b : unsigned(11 downto 0) := (others => '0');
    signal e_dh   : signed(12 downto 0) := (others => '0');
    signal e_t    : signed(13 downto 0) := (others => '0');
    signal e_b80  : unsigned(7 downto 0) := (others => '0');
    signal e_cv   : signed(9 downto 0) := (others => '0');
    signal e_ox   : signed(9 downto 0) := (others => '0');
    signal e_xb   : signed(12 downto 0) := (others => '0');
    signal e_dx1, e_dx2 : signed(12 downto 0) := (others => '0');
    signal e_mx1, e_mx2 : unsigned(14 downto 0) := (others => '0');   -- |dx| * xs
    signal e_ng1, e_ng2 : std_logic := '0';
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    -- results for the next line
    signal n_hw, n_hb : unsigned(11 downto 0) := (others => '0');
    signal n_ty, n_tyi, n_tb, n_tl, n_tr : signed(12 downto 0) := (others => '0');
    signal n_lt   : unsigned(19 downto 0) := (others => '0');
    signal n_eye, n_erim, n_small : std_logic := '0';
    signal n_band : unsigned(2 downto 0) := (others => '0');
    signal n_frac : unsigned(1 downto 0) := (others => '0');
    signal n_pidx : unsigned(5 downto 0) := (others => '0');
    signal n_xb   : signed(12 downto 0) := (others => '0');
    signal n_a1, n_a2 : unsigned(24 downto 0) := (others => '0');
    signal n_d1, n_d2 : signed(18 downto 0) := (others => '0');

    -- beam (stage 1) and the line constants
    signal b_x   : signed(12 downto 0) := (others => '0');
    signal b_a1, b_a2 : unsigned(24 downto 0) := (others => '0');
    signal b_d1, b_d2 : signed(18 downto 0) := (others => '0');
    signal b_pc  : unsigned(10 downto 0) := (others => '0');
    signal b_lp  : std_logic := '0';
    signal l_hw, l_hb : unsigned(11 downto 0) := (others => '0');
    signal l_ty, l_tyi, l_tb, l_tl, l_tr : signed(12 downto 0) := (others => '0');
    signal l_lt   : unsigned(19 downto 0) := (others => '0');
    signal l_eye, l_erim, l_small : std_logic := '0';
    signal l_band : unsigned(2 downto 0) := (others => '0');
    signal l_frac : unsigned(1 downto 0) := (others => '0');
    signal l_pidx : unsigned(5 downto 0) := (others => '0');

    -- pixel path
    signal p2_ax : unsigned(11 downto 0) := (others => '0');
    signal p2_a  : unsigned(24 downto 0) := (others => '0');
    signal p2_pp : std_logic := '0';
    signal p3_kf : unsigned(9 downto 0) := (others => '0');
    signal p3_e, p3_eb : signed(13 downto 0) := (others => '0');
    signal p3_da : signed(25 downto 0) := (others => '0');
    signal p3_pp : std_logic := '0';
    signal p4_kf : unsigned(9 downto 0) := (others => '0');
    signal p4_lm : unsigned(7 downto 0) := (others => '0');
    signal p4_inz, p4_inb, p4_inl, p4_rim : std_logic := '0';
    signal p4_dep : unsigned(5 downto 0) := (others => '0');
    signal p4_mid : unsigned(4 downto 0) := (others => '0');
    signal p4_band : unsigned(2 downto 0) := (others => '0');
    signal p5_kf : unsigned(9 downto 0) := (others => '0');
    signal p5_ct : unsigned(12 downto 0) := (others => '0');
    signal p6_f  : signed(12 downto 0) := (others => '0');
    signal p7_dy, p7_dyi, p7_dr, p7_dl, p7_db : signed(13 downto 0) := (others => '0');
    signal p8_fy, p8_fr, p8_fl, p8_fb : std_logic := '0';
    signal p8_yi : unsigned(5 downto 0) := (others => '0');
    type t_fl is array (5 to 8) of std_logic_vector(17 downto 0);   -- inz inb inl rim | dep(6) mid(5) band(3)
    signal p_fl : t_fl := (others => (others => '0'));
    signal p11_y, p11_u, p11_v : unsigned(7 downto 0) := (others => '0');
    signal p12_y, p12_u, p12_v : unsigned(7 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(7 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(7 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories: line buffer (collisions possible: read-before-write kept),
    -- ROMs, the box rings, the sync ring
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
            r_ex_q <= unsigned(C_CR_EXP2(to_integer(r_ex_a)));
            if s_csz = '1' then r_cs_a <= z_csa; else r_cs_a <= e_csa; end if;
            r_cs_q <= C_CR_COS(to_integer(r_cs_a));
            r_zn_q <= C_CR_ZONE(to_integer(r_zn_a));
            r_ln_q <= C_CR_LENS(to_integer(r_ln_a));
        end if;
    end process p_rom;

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            r_py_q <= C_CR_PY(to_integer(r_pa_a));
            r_pu_q <= C_CR_PU(to_integer(r_pa_a));
            r_pv_q <= C_CR_PV(to_integer(r_pa_a));
        end if;
    end process p_pal;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u15(3 downto 0)) & std_logic_vector(s_v15) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y17) & std_logic_vector(s_u17(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u17(3 downto 0)) & std_logic_vector(s_v17) & "00";
            syr_q <= syr(to_integer(sy_ra));
            syr(to_integer(rg_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_u  : unsigned(15 downto 0);
        variable v_s  : signed(15 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_t13 : unsigned(12 downto 0);
        variable v_e13 : signed(12 downto 0);
        variable v_mg : unsigned(13 downto 0);
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
            if s_zone = '1' and s_zc(10) = '0' then s_csz <= '1'; else s_csz <= '0'; end if;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_twin <= registers_in(6)(0);
                s_flip <= registers_in(6)(1);
                s_ice  <= registers_in(6)(2);
                s_tear <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                                   c_dd <= to_unsigned(32, 6); c_xsq <= to_unsigned(16, 5);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                                   c_dd <= to_unsigned(18, 6); c_xsq <= to_unsigned(9, 5);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                                   c_dd <= to_unsigned(8, 6);  c_xsq <= to_unsigned(4, 5);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, seven channels
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 5      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when 3      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 5      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 6 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- knob curves (exp2 ROM, cos ROM) and shift-add constants: zc 30 .. 60, 118 .. 130
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 =>                                        -- stripes: 2^(1.5 (k1 - 512) / 512)
                        -- (3 k - 1536) >> 2, formed unsigned (no shared operand bits in the adder)
                        v_t13 := shift_left(resize(s_g1(12 downto 3), 13), 1) + resize(s_g1(12 downto 3), 13);
                        v_e13 := shift_right(signed(v_t13) - 1536, 2);
                        r_ex_a <= unsigned(v_e13(7 downto 0)); z_sh1 <= v_e13(10 downto 8);
                    when 32 => z_mt1 <= r_ex_q;
                    when 33 =>                                        -- figure: 2^(+-0.59 (k3 - 512) / 256)
                        v_k := s_g3(12 downto 3);
                        v_t13 := resize(shift_right(v_k, 1), 13) + resize(shift_right(v_k, 4), 13) + resize(shift_right(v_k, 5), 13);
                        v_e13 := signed(v_t13) - 304;
                        r_ex_a <= unsigned(v_e13(7 downto 0)); z_sh3 <= v_e13(10 downto 8);
                        z_e3 <= v_e13(11 downto 0);
                    when 35 => z_mt3 <= r_ex_q;
                    when 36 =>
                        v_e := -z_e3;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh3n <= v_e(10 downto 8);
                    when 38 => z_mt3n <= r_ex_q;
                    when 39 =>
                        case to_integer(z_sh3) is
                            when -2     => c_zs8 <= "000" & z_mt3(15 downto 9);
                            when -1     => c_zs8 <= "00" & z_mt3(15 downto 8);
                            when 0      => c_zs8 <= "0" & z_mt3(15 downto 7);
                            when others => c_zs8 <= z_mt3(15 downto 6);
                        end case;
                        case to_integer(z_sh3n) is
                            when -2     => c_izs8 <= "000" & z_mt3n(15 downto 9);
                            when -1     => c_izs8 <= "00" & z_mt3n(15 downto 8);
                            when 0      => c_izs8 <= "0" & z_mt3n(15 downto 7);
                            when others => c_izs8 <= z_mt3n(15 downto 6);
                        end case;
                    when 40 =>                                        -- roll speed: deadband +-8
                        v_d := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        if v_d > 8 then c_s <= resize(v_d - 8, 11); c_vneg <= '0';
                        elsif v_d < -8 then c_s <= resize(-(v_d + 8), 11); c_vneg <= '1';
                        else c_s <= (others => '0'); c_vneg <= '0'; end if;
                    when 41 =>                                        -- lens threshold (Q8)
                        v_k := s_g5(12 downto 3);
                        if v_k >= 512 then
                            v_d := signed(resize(v_k - 512, 12));
                            c_th8 <= resize(166 + shift_right(v_d, 1) - shift_right(v_d, 5) + shift_right(v_d, 7), 10);
                        else
                            v_d := signed(resize(512 - v_k, 12));
                            c_th8 <= resize(166 - shift_right(v_d, 1) - shift_right(v_d, 3), 10);
                        end if;
                    when 42 =>                                        -- heat (Q8): 0.75 p - 410
                        v_t13 := shift_left(resize(s_gp(12 downto 3), 13), 1) + resize(s_gp(12 downto 3), 13);
                        c_h8 <= resize(signed(shift_right(v_t13, 2)) - 410, 11);
                    when others => null;
                end case;
            end if;

            -- serial products: slot k (0 .. 5) loads at zc = 48 + 24 k, 18 iterations, result stable from
            -- load + 19.  Operands pre-muxed by the slot number two clocks ahead.
            z_pre <= '0';
            if s_zgo = '1' then
                z_k <= (others => '0'); z_kc <= (others => '0'); z_run <= '1';
            elsif z_run = '1' and s_zone = '1' and s_zc >= 46 then
                if z_kc = 23 then
                    z_kc <= (others => '0');
                    if z_k = 5 then z_run <= '0'; end if;
                    z_k <= z_k + 1;
                else
                    z_kc <= z_kc + 1;
                end if;
                if z_kc = 0 then z_sel <= z_k; z_pre <= '1'; end if;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 => z_fa <= resize(z_mt1, 28); z_fb <= to_unsigned(184, 18);                   -- invP
                    when 1 => z_fa <= resize(unsigned(c_s), 28); z_fb <= resize(unsigned(c_s), 18);      -- s^2
                    when 2 => z_fa <= resize(c_s2, 28); z_fb <= resize(c_invp, 18);                     -- s^2 invP
                    when 3 => z_fa <= resize(c_invp, 28); z_fb <= to_unsigned(102, 18);                 -- 102 invP
                    when 4 =>                                                                           -- morph
                        if s_g4(12) = '0' then
                            z_fa <= resize(c_b8, 28); z_fb <= resize(s_g4(12 downto 3), 18);
                        else
                            z_fa <= resize(255 - c_b8, 28); z_fb <= resize(s_g4(11 downto 3), 18);
                        end if;
                    when others => z_fa <= resize(s_g6(12 downto 3), 28); z_fb <= resize(s_g6(12 downto 3), 18);  -- camera gain
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(26 downto 0) & '0';
                f_b <= '0' & f_b(17 downto 1);
            end if;

            -- product results (z_kc = 21 of each slot) and the cos-ROM lookups for the morph / breathing
            if s_zone = '1' and z_run = '1' and z_kc = 21 then
                case to_integer(z_k) is
                    when 0 =>
                        case to_integer(z_sh1) is
                            when -2     => c_invp <= resize(f_acc(27 downto 17), 10);
                            when -1     => c_invp <= resize(f_acc(26 downto 16), 10);
                            when 0      => c_invp <= resize(f_acc(25 downto 15), 10);
                            when others => c_invp <= resize(f_acc(24 downto 14), 10);
                        end case;
                    when 1 => c_s2 <= f_acc(17 downto 0);
                    when 2 =>
                        v_u := resize(f_acc(27 downto 14), 16);             -- (prod >> 14)
                        c_vst <= v_u - shift_right(v_u, 2);                 -- x 3/4
                    when 3 =>
                        -- new roll phase and the W phase at line 0
                        if c_vneg = '1' then c_rp <= c_rp - c_vst; else c_rp <= c_rp + c_vst; end if;
                    when 4 =>
                        if s_g4(12) = '0' then
                            c_b <= resize(f_acc(17 downto 9), 9);
                        else
                            c_b <= resize(c_b8, 9) + resize(f_acc(17 downto 9), 9);
                        end if;
                    when others =>
                        c_g <= f_acc(19 downto 15);
                end case;
            end if;
            if s_zone = '1' and z_run = '1' and z_kc = 22 and z_k = 3 then
                c_ph0 <= unsigned(-signed(f_acc(15 downto 0))) - c_rp;
            end if;
            -- cos lookups (the cos ROM belongs to the sequencer while zc < 1024): after slot 3 (zc 120 ..)
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 146 => z_csa <= c_rp(15 downto 8) - 12;                 -- morph LFO phase (- 0.048 cycle)
                    when 149 => c_cb <= signed(r_cs_q);
                    when 150 => z_csa <= c_rp(15 downto 8) - 64;                 -- breathing: sin
                    when 151 => c_b8 <= resize(unsigned(shift_right(255 - c_cb, 1)), 8);
                    when 153 => c_sn <= signed(r_cs_q);
                    when 154 =>
                        -- 256 +- (23 |sn|) >> 8 (sign-magnitude: no shared operand bits in the adder)
                        if c_sn < 0 then v_mg := resize(unsigned(-c_sn), 14); else v_mg := resize(unsigned(c_sn), 14); end if;
                        v_mg := shift_left(v_mg, 4) + shift_left(v_mg, 2) + shift_left(v_mg, 1) + v_mg;
                        if c_sn < 0 then c_ls8 <= 256 - resize(v_mg(13 downto 8), 9);
                        else c_ls8 <= 256 + resize(v_mg(13 downto 8), 9); end if;
                    when others => null;
                end case;
            end if;

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
            s_lb_wa <= s_wcol;
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
    -- 0), computes the NEXT line.  One serial product at a time (16
    -- iterations, result stable from load + 17); one small operation per
    -- clock otherwise.  ~200 clocks: fits SD's 858-clock line.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_vf  : signed(12 downto 0);
        variable v_i   : unsigned(8 downto 0);
        variable v_t   : signed(13 downto 0);
        variable v_th  : signed(13 downto 0);
        variable v_u   : unsigned(11 downto 0);
        variable v_r   : unsigned(15 downto 0);
        variable v_m   : signed(9 downto 0);
        variable v_x   : signed(15 downto 0);
        variable v_d   : signed(18 downto 0);
        variable v_a   : signed(11 downto 0);
        variable v_p   : unsigned(15 downto 0);
        variable v_mm  : signed(10 downto 0);
        variable v_ux, v_uy : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_lfsr <= e_lfsr(14 downto 0) & (e_lfsr(15) xor e_lfsr(13) xor e_lfsr(12) xor e_lfsr(10));
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then e_vl <= to_unsigned(1079, 11); else e_vl <= e_vy(18 downto 8); end if;
                when 2 =>
                    if s_flip = '1' then
                        v_vf := 1030 - signed(resize(e_vl, 13));
                        if v_vf < 0 then e_vf <= (others => '0'); else e_vf <= unsigned(v_vf(10 downto 0)); end if;
                    else
                        e_vf <= e_vl;
                    end if;
                    -- W phase: e_vy * invP (Q8 lines x Q16 cycles / line)
                    m_a <= resize(e_vy, 36); m_b <= resize(c_invp, 16); m_acc <= (others => '0');
                when 3 =>
                    e_d  <= signed(resize(e_vf, 12)) - 490;
                    e_d2 <= signed(resize(e_vf, 12)) - 535;
                    e_dl <= signed(resize(e_vf, 12)) - 520;
                    e_dv <= signed(resize(e_vl, 12)) - 540;
                when 4 =>
                    v_a := abs(e_d);  e_ad  <= unsigned(v_a(10 downto 0));
                    e_top <= e_d(11);
                    v_a := abs(e_d2); e_ad2 <= unsigned(v_a(10 downto 0));
                    v_a := abs(e_dl); e_adl <= unsigned(v_a(10 downto 0));
                    v_a := abs(e_dv); e_adv <= unsigned(v_a(10 downto 0));
                    e_bm <= 80 + shift_left(resize(e_lfsr(14 downto 9), 9), 2) + resize(e_lfsr(14 downto 9), 9);
                when 5 =>
                    -- tear / jitter offset (h) for this line, from the line's LFSR
                    v_r := e_lfsr;
                    if s_tear = '1' then
                        -- bursts: 1 start in 8 lines, 8..39 lines long, shifted +-(80 + 5 r) h = 40..198 vpx
                        if e_bc /= 0 then
                            e_bc <= e_bc - 1;
                            e_ox <= e_bo + signed(resize(v_r(1 downto 0), 10)) - 2;
                        elsif v_r(2 downto 0) = "000" then
                            e_bc <= resize(v_r(8 downto 4), 6) + 7;
                            if v_r(15) = '1' then
                                e_bo <= -signed(resize(e_bm, 10)); e_ox <= -signed(resize(e_bm, 10));
                            else
                                e_bo <= signed(resize(e_bm, 10)); e_ox <= signed(resize(e_bm, 10));
                            end if;
                        else
                            e_ox <= (others => '0');
                        end if;
                    else
                        e_bc <= (others => '0');
                        if v_r(2 downto 0) = "000" then
                            if v_r(7) = '1' then e_ox <= -signed(resize(v_r(6 downto 4), 10)) - 2;
                            else e_ox <= signed(resize(v_r(6 downto 4), 10)) + 2; end if;
                        else
                            e_ox <= (others => '0');
                        end if;
                    end if;
                when 6 =>
                    e_xb  <= signed(resize(c_xb0, 13)) + resize(e_ox, 13);
                    -- violet band / pink index by |y - 540|
                    if e_adv > 100 then v_u := resize(e_adv - 100, 12); else v_u := (others => '0'); end if;
                    if v_u(11 downto 6) > 7 then n_band <= "111"; else n_band <= v_u(8 downto 6); end if;
                    n_frac <= v_u(5 downto 4);
                when 7 =>
                    if s_twin = '1' then                      -- two columns at 992 -+ 420 vpx
                        e_dx1 <= e_xb - 1144;
                        e_dx2 <= e_xb - 2824;
                    else                                      -- the photo's column (x 875) and its mirror
                        e_dx1 <= e_xb - 1750;
                        e_dx2 <= e_xb - 2218;
                    end if;
                    if e_adv > 280 then
                        v_u := resize(e_adv - 280, 12);
                        v_u := shift_right(v_u, 2) + shift_right(v_u, 4);
                        if v_u > 63 then n_pidx <= (others => '1'); else n_pidx <= v_u(5 downto 0); end if;
                    else
                        n_pidx <= (others => '0');
                    end if;
                when 8 =>
                    -- |dx| * xs (the lens DDA step base), from the magnitude (no shared operand bits)
                    v_ux := unsigned(abs(e_dx1)); e_ng1 <= e_dx1(12);
                    v_uy := unsigned(abs(e_dx2)); e_ng2 <= e_dx2(12);
                    case to_integer(s_xs) is
                        when 4      => e_mx1 <= shift_left(resize(v_ux, 15), 2); e_mx2 <= shift_left(resize(v_uy, 15), 2);
                        when 3      => e_mx1 <= shift_left(resize(v_ux, 15), 1) + resize(v_ux, 15);
                                       e_mx2 <= shift_left(resize(v_uy, 15), 1) + resize(v_uy, 15);
                        when others => e_mx1 <= shift_left(resize(v_ux, 15), 1); e_mx2 <= shift_left(resize(v_uy, 15), 1);
                    end case;
                when 9 =>
                    -- D0 = 2 dx xs + xs^2
                    if e_ng1 = '1' then n_d1 <= signed(resize(c_xsq, 19)) - signed(resize(shift_left(resize(e_mx1, 18), 1), 19));
                    else n_d1 <= signed(resize(shift_left(resize(e_mx1, 18), 1), 19)) + signed(resize(c_xsq, 19)); end if;
                    if e_ng2 = '1' then n_d2 <= signed(resize(c_xsq, 19)) - signed(resize(shift_left(resize(e_mx2, 18), 1), 19));
                    else n_d2 <= signed(resize(shift_left(resize(e_mx2, 18), 1), 19)) + signed(resize(c_xsq, 19)); end if;
                    n_xb <= e_xb;
                when 19 =>
                    v_p := c_ph0 + m_acc(23 downto 8);
                    e_csa <= v_p(15 downto 8);
                when 20 =>
                    m_a <= resize(e_ad, 36); m_b <= resize(c_izs8, 16); m_acc <= (others => '0');
                when 22 =>
                    e_w8 <= signed(r_cs_q);
                when 37 =>
                    if m_acc(20 downto 8) > 1023 then e_adz <= to_unsigned(1023, 11); else e_adz <= m_acc(18 downto 8); end if;
                    m_a <= resize(e_ad2, 36); m_b <= resize(c_izs8, 16); m_acc <= (others => '0');
                when 38 =>
                    if e_adz(10 downto 9) /= "00" then v_i := to_unsigned(255, 9); else v_i := '0' & e_adz(8 downto 1); end if;
                    if e_top = '1' then r_zn_a <= '0' & v_i; else r_zn_a <= to_unsigned(256, 10) + v_i; end if;
                    if e_adz > 110 then e_md <= e_adz - 110; else e_md <= (others => '0'); end if;
                when 39 =>
                    -- m terms: md * 589 / 256 and md * 236 / 256; the red term ad * 166 / 256
                    e_s1 <= shift_left(resize(e_md, 20), 9) + shift_left(resize(e_md, 20), 6);
                    e_s2 <= shift_left(resize(e_md, 20), 3) + shift_left(resize(e_md, 20), 2) + resize(e_md, 20);
                    e_s3 <= shift_left(resize(e_md, 20), 8) - shift_left(resize(e_md, 20), 4) - shift_left(resize(e_md, 20), 2);
                    e_s4 <= shift_left(resize(e_ad, 20), 7) + shift_left(resize(e_ad, 20), 5)
                          + shift_left(resize(e_ad, 20), 2) + shift_left(resize(e_ad, 20), 1);
                when 40 =>
                    e_h1 <= unsigned(r_zn_q);
                    v_u := resize(e_s1(19 downto 8), 12) + resize(e_s2(19 downto 8), 12);
                    e_mty <= v_u;
                    e_mtl <= e_s3(18 downto 8);
                    e_tr  <= e_s4(18 downto 8);
                when 41 =>
                    if e_adz > 50 then v_u := resize(e_adz - 50, 12); else v_u := (others => '0'); end if;
                    if v_u(10 downto 9) /= "00" then v_i := to_unsigned(255, 9); else v_i := '0' & v_u(8 downto 1); end if;
                    if e_top = '1' then r_zn_a <= '0' & v_i; else r_zn_a <= to_unsigned(256, 10) + v_i; end if;
                    -- thresholds (Q8 F units)
                    v_th := 205 - resize(e_w8, 14) - resize(c_h8, 14) + signed(resize(e_mty, 14));
                    n_ty <= resize(v_th, 13);
                    n_tb <= resize(v_th - 102, 13);
                    n_tyi <= resize(v_th + signed(resize(e_lfsr(11 downto 8), 14)) - 8, 13);
                    v_th := 64 - resize(e_w8, 14) - resize(c_h8, 14) + signed(resize(e_mtl, 14));
                    n_tl <= resize(v_th, 13);
                    v_th := 105 - resize(e_w8, 14) - resize(c_h8, 14) + signed(resize(e_tr, 14));
                    n_tr <= resize(v_th, 13);
                when 43 =>
                    e_h1b <= unsigned(r_zn_q);
                when 54 =>
                    if m_acc(20 downto 8) > 1023 then e_d2z <= to_unsigned(1023, 11); else e_d2z <= m_acc(18 downto 8); end if;
                    m_a <= resize(e_adl, 36); m_b <= resize(c_ls8, 16); m_acc <= (others => '0');
                when 55 =>
                    if e_d2z(10 downto 9) /= "00" then v_i := to_unsigned(255, 9); else v_i := '0' & e_d2z(8 downto 1); end if;
                    r_zn_a <= to_unsigned(512, 10) + v_i;
                when 57 =>
                    e_h2 <= unsigned(r_zn_q);
                    if e_d2z > 50 then v_u := resize(e_d2z - 50, 12); else v_u := (others => '0'); end if;
                    if v_u(10 downto 9) /= "00" then v_i := to_unsigned(255, 9); else v_i := '0' & v_u(8 downto 1); end if;
                    r_zn_a <= to_unsigned(512, 10) + v_i;
                when 59 =>
                    e_h2b <= unsigned(r_zn_q);
                    e_dh <= signed(resize(e_h2, 13)) - signed(resize(e_h1, 13));
                    e_b80 <= resize(shift_right(shift_left(resize(c_b, 16), 6) + shift_left(resize(c_b, 16), 4), 8), 8);
                when 71 =>
                    if m_acc(19 downto 9) > 511 then r_ln_a <= (others => '1'); else r_ln_a <= m_acc(17 downto 9); end if;
                    m_a <= resize(unsigned(abs(e_dh)), 36); m_b <= resize(c_b, 16); m_acc <= (others => '0');
                when 73 =>
                    e_cv <= shift_left(resize(signed(r_ln_q), 10), 1);
                when 74 =>
                    e_dh <= signed(resize(e_h2b, 13)) - signed(resize(e_h1b, 13));
                    e_t  <= resize(e_dh, 14);     -- keep the sign of the first difference
                when 88 =>
                    if e_t(13) = '1' then
                        v_t := signed(resize(e_h1, 14)) - signed(resize(m_acc(20 downto 8), 14)) - signed(resize(e_b80, 14));
                    else
                        v_t := signed(resize(e_h1, 14)) + signed(resize(m_acc(20 downto 8), 14)) - signed(resize(e_b80, 14));
                    end if;
                    if v_t < 0 then v_t := (others => '0'); end if;
                    e_t <= v_t;
                when 89 =>
                    m_a <= resize(unsigned(e_t), 36); m_b <= resize(c_zs8, 16); m_acc <= (others => '0');
                when 106 =>
                    if m_acc(23 downto 8) > 4095 then n_hw <= (others => '1'); else n_hw <= m_acc(19 downto 8); end if;
                    m_a <= resize(unsigned(abs(e_dh)), 36); m_b <= resize(c_b, 16); m_acc <= (others => '0');
                    e_t <= resize(e_dh, 14);
                when 123 =>
                    if e_t(13) = '1' then
                        v_t := signed(resize(e_h1b, 14)) - signed(resize(m_acc(20 downto 8), 14)) - signed(resize(e_b80, 14));
                    else
                        v_t := signed(resize(e_h1b, 14)) + signed(resize(m_acc(20 downto 8), 14)) - signed(resize(e_b80, 14));
                    end if;
                    if v_t < 0 then v_t := (others => '0'); end if;
                    e_t <= v_t;
                when 124 =>
                    m_a <= resize(unsigned(e_t), 36); m_b <= resize(c_zs8, 16); m_acc <= (others => '0');
                    -- per-line flags that need n_hw
                    if e_w8 < -197 and n_hw > 400 then n_eye <= '1'; else n_eye <= '0'; end if;
                    if e_w8 >= -207 and e_w8 <= -187 and n_hw > 400 then n_erim <= '1'; else n_erim <= '0'; end if;
                    if n_hw < 24 then n_small <= '1'; else n_small <= '0'; end if;
                when 141 =>
                    if m_acc(23 downto 8) > 4095 then n_hb <= (others => '1'); else n_hb <= m_acc(19 downto 8); end if;
                    v_mm := resize(c_th8, 11) + resize(e_cv, 11);
                    if v_mm < 0 then v_mm := (others => '0'); end if;
                    m_a <= resize(unsigned(v_mm), 36); m_b <= to_unsigned(824, 16); m_acc <= (others => '0');
                when 158 =>
                    n_lt <= m_acc(19 downto 0);
                    m_a <= resize(unsigned(abs(e_dx1)), 36); m_b <= unsigned(resize(abs(e_dx1), 16)); m_acc <= (others => '0');
                when 175 =>
                    n_a1 <= m_acc(24 downto 0);
                    m_a <= resize(unsigned(abs(e_dx2)), 36); m_b <= unsigned(resize(abs(e_dx2), 16)); m_acc <= (others => '0');
                when 192 =>
                    n_a2 <= m_acc(24 downto 0);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): x in h, the two lens DDAs A = (x - c)^2; the line
    -- constants loaded at the line start; the camera read issued
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= n_xb;
                b_a1 <= n_a1; b_d1 <= n_d1;
                b_a2 <= n_a2; b_d2 <= n_d2;
                b_pc <= (others => '0');
                b_lp <= not b_lp;
                l_hw <= n_hw; l_hb <= n_hb;
                l_ty <= n_ty; l_tyi <= n_tyi; l_tb <= n_tb; l_tl <= n_tl; l_tr <= n_tr;
                l_lt <= n_lt;
                l_eye <= n_eye; l_erim <= n_erim; l_small <= n_small;
                l_band <= n_band; l_frac <= n_frac; l_pidx <= n_pidx;
            elsif g_avid = '1' then
                b_x  <= b_x + signed(resize(s_xs, 13));
                b_a1 <= unsigned(signed(b_a1) + resize(b_d1, 25));
                b_d1 <= b_d1 + signed(resize(c_dd, 19));
                b_a2 <= unsigned(signed(b_a2) + resize(b_d2, 25));
                b_d2 <= b_d2 + signed(resize(c_dd, 19));
                b_pc <= b_pc + 1;
            end if;
            s_lb_ra <= b_pc;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ax : signed(12 downto 0);
        variable v_bay : unsigned(1 downto 0);
        variable v_bd : unsigned(3 downto 0);
        variable v_a  : unsigned(9 downto 0);
        variable v_f  : std_logic_vector(17 downto 0);
        variable v_dq : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: |x - axis|, the lens distance^2 (min of the twin columns)
            v_ax := abs(b_x - 1984);
            p2_ax <= unsigned(v_ax(11 downto 0));
            if s_twin = '1' and b_a2 < b_a1 then p2_a <= b_a2; else p2_a <= b_a1; end if;
            p2_pp <= b_pc(0);
            -- 3: the H-ramp term 3 ax / 16, distances to the figure edge and border, lens depth
            p3_kf <= resize(shift_right(shift_left(resize(p2_ax, 14), 1) + resize(p2_ax, 14), 4), 10);
            p3_e  <= signed(resize(p2_ax, 14)) - signed(resize(l_hw, 14));
            p3_eb <= signed(resize(p2_ax, 14)) - signed(resize(l_hb, 14));
            p3_da <= signed(resize(l_lt, 26)) - signed(resize(p2_a, 26));
            p3_pp <= p2_pp;
            -- 4: flags and indices; the camera luma (fabric copy of the line-buffer read)
            p4_kf <= p3_kf;
            p4_lm <= unsigned(s_lb_q);
            p4_inz <= p3_e(13);
            p4_inb <= p3_eb(13);
            if p3_da > 0 then p4_inl <= '1'; else p4_inl <= '0'; end if;
            if p3_da(25) = '1' then p4_dep <= (others => '0');
            elsif p3_da(24 downto 19) /= "000000" then p4_dep <= (others => '1');
            else p4_dep <= unsigned(p3_da(18 downto 13)); end if;
            if l_small = '1' or p3_e(13 downto 9) /= "00000" then p4_mid <= (others => '1');
            else p4_mid <= unsigned(p3_e(8 downto 4)); end if;
            if l_small = '0' and p3_e >= 4 and p3_e < 20 then p4_rim <= '1'; else p4_rim <= '0'; end if;
            v_bay := (b_lp xor p3_pp) & p3_pp;                    -- 2x2 Bayer 0 2 / 3 1
            if l_frac > v_bay and l_band /= 7 then p4_band <= l_band + 1; else p4_band <= l_band; end if;
            -- 5: camera product (luma x gain)
            p5_kf <= p4_kf;
            p5_ct <= p4_lm * c_g;
            p_fl(5) <= p4_inz & p4_inb & p4_inl & p4_rim & std_logic_vector(p4_dep) & std_logic_vector(p4_mid) & std_logic_vector(p4_band);
            -- 6: F = H term + camera
            p6_f <= signed(resize(p5_kf, 13)) + signed(resize(shift_right(p5_ct, 3), 13));
            p_fl(6) <= p_fl(5);
            -- 7: distances to the per-line thresholds
            p7_dy  <= resize(p6_f, 14) - resize(l_ty, 14);
            p7_dyi <= resize(p6_f, 14) - resize(l_tyi, 14);
            p7_dr  <= resize(p6_f, 14) - resize(l_tr, 14);
            p7_dl  <= resize(p6_f, 14) - resize(l_tl, 14);
            p7_db  <= resize(p6_f, 14) - resize(l_tb, 14);
            p_fl(7) <= p_fl(6);
            -- 8: decisions, the yellow index
            if p7_dy > 0 then p8_fy <= '1'; else p8_fy <= '0'; end if;
            if p7_dr > 0 then p8_fr <= '1'; else p8_fr <= '0'; end if;
            if p7_dl > 0 then p8_fl <= '1'; else p8_fl <= '0'; end if;
            if p7_db > 0 then p8_fb <= '1'; else p8_fb <= '0'; end if;
            v_dq := shift_right(p7_dyi, 3);
            if v_dq < 0 then p8_yi <= (others => '0');
            elsif v_dq > 63 then p8_yi <= (others => '1');
            else p8_yi <= unsigned(v_dq(5 downto 0)); end if;
            p_fl(8) <= p_fl(7);
            -- 9: the palette address (priority)
            v_f := p_fl(8);
            -- v_f: 17 inz, 16 inb, 15 inl, 14 rim, 13..8 dep, 7..3 mid, 2..0 band
            if v_f(17) = '1' then
                if p8_fy = '1' then
                    if v_f(15) = '1' then v_a := to_unsigned(449, 10); else v_a := to_unsigned(320, 10) + resize(p8_yi, 10); end if;
                elsif v_f(15) = '1' and (p8_fl = '1' or l_eye = '1') then
                    if (unsigned(v_f(13 downto 8)) < 3 and p8_fl = '1') or l_erim = '1' then v_a := to_unsigned(450, 10);
                    else v_a := to_unsigned(384, 10) + resize(unsigned(v_f(13 downto 8)), 10); end if;
                else
                    v_a := to_unsigned(448, 10);
                end if;
            else
                if v_f(15) = '1' then
                    v_a := to_unsigned(384, 10) + resize(unsigned(v_f(13 downto 8)), 10);
                elsif (v_f(16) = '1' and p8_fb = '1') or p8_fr = '1' then
                    v_a := to_unsigned(256, 10) + resize(l_pidx, 10);
                elsif v_f(14) = '1' then
                    v_a := to_unsigned(451, 10);
                else
                    v_a := "00" & unsigned(v_f(2 downto 0)) & unsigned(v_f(7 downto 3));
                end if;
            end if;
            r_pa_a <= v_a;
            -- 10: palette data (EBR); 11: fabric copy
            p11_y <= unsigned(r_py_q); p11_u <= unsigned(r_pu_q); p11_v <= unsigned(r_pv_q);
            -- 12, 13: pads (one latency, a multiple of 4)
            p12_y <= p11_y; p12_u <= p11_u; p12_v <= p11_v;
            p13_y <= p12_y; p13_u <= p12_u; p13_v <= p12_v;
            p14_y <= p13_y; p14_u <= p13_u; p14_v <= p13_v;
            -- 15: Ice (Cr <-> Cb), black outside the generated avid (the box input)
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= p14_y & "00";
                if s_ice = '1' then s_u15 <= p14_v & "00"; s_v15 <= p14_u & "00";
                else s_u15 <= p14_u & "00"; s_v15 <= p14_v & "00"; end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the 8-back taps come from block-RAM rings:
    -- the still's analog softness, and the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate; the syncs ride a block-RAM ring of the same latency
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 5;
            sy_ra <= rg_wa - (C_LAT - 3);
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- box A: stage 16 sums, stage 17 output (the running sums snap to neutral after 8 blank inputs)
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y15 - s_cay;
                s_bau <= s_bau + s_u15 - s_cau;
                s_bav <= s_bav + s_v15 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y17 - s_cby;
                s_bbu <= s_bbu + s_u17 - s_cbu;
                s_bbv <= s_bbv + s_v17 - s_cbv;
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v19 <= v_sc(13 downto 4);
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
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture crucible;
