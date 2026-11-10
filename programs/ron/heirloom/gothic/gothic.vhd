-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Gothic v0.1.1 -- Heirloom (video clips), from clip WIN_20230817_00_06_49 at 0.5 s.
--
-- v0.1.1: the camera luma is smoothed before it bends the field (pair sums at half horizontal resolution,
-- an 8 px running box reset every line, a vertical IIR f = (prev + new) / 2 in a second line buffer), the
-- default camera bend is about a fifth of v0.1.0's (Etch 120), and the Osc rides its own x32 shift so its
-- flip stays as strong as before.
--
-- The still: nested pointed (gothic) arches of ribbons over soft green/black horizontal stripes.  The
-- ribbons carry blue leaves with dark green flanks, patches of wavy red lines on black, and a thick cyan
-- line along the edge of each gap; the stripes show through the gaps.  In the clip the arches sink
-- (the field offset runs -36 F-px/s: apexes move down, sides close in), the stripes roll down at
-- ~106 lines/s and the red patches rise.
--
-- The patch, as fitted (gothic.py has every constant):
--   F    = (CX x - PH tri(u) + g(y) - flow) / P + warp(x) / P0 + Etch * luma     arch field, colouriser cycles
--          u = (x - XP) / PH, tri = 0 at the apex .. 1/2 at the trough; g(y) = (S / w) sin(w (y - YZ)) + GOFF
--   f    = frac(F): gap (stripes show through) / cyan line / ribbon body
--   Q    = 0.656 * 2 tri + 0.320 u + 0.2405 cos(2 pi (Y - 0.381 + rise) / 2) - 0.068 + teeth(y)
--          body: red waves where frac(Q) in [0.70, 0.12) or [0.36, 0.44), else blue leaf / dark green
--   red  = sin^3 profile of x N / 324 + wob(f) sin(2 pi (y / 117 + x / 1800 + t))
--   bg   = clip(0.596 cos(2 pi (y - 152 - roll) / 173) + 0.556) green
--
--   K1  Arches    arch frequency across (x0.35 .. x2.8)
--   K2  Curve     vertical sine frequency at a fixed zero crossing: straight chevrons .. gothic .. wavy sway
--   K3  Ribbons   colouriser density: nested ribbons per arch (x0.33 .. x3)
--   K4  Speed     bipolar: arches sink, stripes roll, red patches rise (700 = the clip)
--   K5  Waves     red line spacing (x0.35 .. x2.8: fine straight lines .. broad looping swirls)
--   K6  Etch      how far the camera (or the Osc) bends the arch field
--   S7  Stripes   Green / Cyan (white-cored stripes, as later in the clip)
--   S8  Swap      red waves <-> blue leaves
--   S9  Tear      rolling V-hold slips: alternate 512-line bands (moving with Speed) where the arches
--                 jump 0.37 of a ribbon cycle, the ground goes black and red / blue swap
--   S10 Flip      vertical term inverted: hanging arches
--   S11 Source    Video / Osc
--   P12 VAULT     vertical slope: straight pillars (0) .. the still (400) .. tight stacked arches (1023)
--
-- Engine: vblank-zone sequencer and a line engine (kicked at each line start, computes the NEXT line:
-- interpolated sine for the vertical term, stripe level, key term) sharing one serial multiplier; shared
-- glide; x terms as DDAs (the triangle times 1/P as a folded modulo DDA, so no pixel multiplier); a
-- 20-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; one
-- latency (C_LAT 20) in every mode; Y, U and V through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum
-- resync, luma 13/16, blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.gothic_pkg.all;
use work.all;

architecture gothic of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (see gothic.py)
    constant K_DU1  : integer := 8687;
    constant K_XP2  : integer := 4083;
    constant K_INVP : integer := 3141;
    constant K_AG   : integer := 40042;
    constant K_OM   : integer := 359410;
    constant K_DWH  : integer := 233016;
    constant K_PHT  : integer := 15451;
    constant K_PS0  : integer := 2036541;
    constant K_QP0  : integer := 13581156;
    constant K_QC   : integer := -279;
    constant C_TJ   : integer := 6207570;   -- tear band: +0.37 colouriser cycles (a vertical slip of the arches)
    constant C_K    : std_logic_vector(31 downto 0) := C_GT_PAL(0);   -- palette black
    constant C_KY   : unsigned(9 downto 0) := unsigned(C_K(31 downto 22));
    constant C_KU   : unsigned(9 downto 0) := unsigned(C_K(21 downto 12));
    constant C_KV   : unsigned(9 downto 0) := unsigned(C_K(11 downto 2));

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
    function f_min6(a, b : unsigned(5 downto 0)) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;
    -- per-line phase steps ys * (379, 30, 14, 560), by timing ID
    function f_lstep(t : std_logic_vector(3 downto 0); k : integer) return unsigned is
        variable ys : integer;
    begin
        case to_integer(unsigned(t)) is
            when 0          => ys := 1138;
            when 4          => ys := 576;
            when 8          => ys := 960;
            when 12         => ys := 480;
            when 5 | 6 | 14 => ys := 384;
            when 1 | 2 | 10 => ys := 512;
            when others     => ys := 256;
        end case;
        return to_unsigned(ys * k, 24);
    end function;

    -- luma: (luma >> 2) - 16, clipped at 0 (or the Osc lattice), smoothed at half horizontal resolution: pair sums,
    -- an 8 px running box (reset every line), a vertical IIR f = (prev + new) / 2.  ir holds the previous line's
    -- f for the IIR; lb gets the same writes and is read by the pixel path.  Neither RAM is ever read at the
    -- address being written (the IIR reads pair k + 1 / k + 2 while pair k is written; the pixel path reads ~5
    -- pairs ahead of the write), so both carry no_rw_check.
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal ir : t_lb := (others => x"00");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    attribute no_rw_check of ir : signal is true;
    signal s_lb_wa : unsigned(10 downto 0) := (others => '0');
    signal s_lb_ra, ir_ra, ir_wa : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we, ir_we : std_logic := '0';
    signal s_lb_q, ir_q : std_logic_vector(7 downto 0) := x"00";
    signal ir_wd  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal q_lp   : unsigned(7 downto 0) := (others => '0');
    signal q_pr, q_h1, q_h2, q_h3, q_h4 : unsigned(8 downto 0) := (others => '0');
    signal q_hs   : unsigned(10 downto 0) := (others => '0');
    signal q_hb2, q_ic : unsigned(7 downto 0) := (others => '0');
    signal q_k, q_k2, q_k3 : unsigned(9 downto 0) := (others => '0');
    signal q_pv, q_pv1, q_pv2, q_pv3 : std_logic := '0';
    signal q_avh  : std_logic_vector(3 downto 0) := (others => '0');

    -- ROMs (registered address and data; every consumer adds a fabric copy)
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_vt_a, r_sp_a : unsigned(7 downto 0) := (others => '0');
    signal r_vt_q  : unsigned(13 downto 0) := (others => '0');
    signal r_sp_q  : unsigned(10 downto 0) := (others => '0');
    signal r_sn_a  : unsigned(9 downto 0) := (others => '0');
    signal r_sn_q  : signed(15 downto 0) := (others => '0');
    signal r_ws_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ws_q  : signed(7 downto 0) := (others => '0');
    signal r_wt_a  : unsigned(8 downto 0) := (others => '0');
    signal r_wt_q  : signed(15 downto 0) := (others => '0');
    signal r_fr_a  : unsigned(7 downto 0) := (others => '0');
    signal r_fr_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_pro_a : unsigned(5 downto 0) := (others => '0');
    signal r_pro_q : unsigned(5 downto 0) := (others => '0');
    signal r_pal_a : unsigned(8 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(120 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_cyan, s_swap, s_tear, s_flip, s_src : std_logic := '0';

    -- per-field sequencer scratch
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_e1, z_e2, z_e3, z_e2n, z_e5n : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(3 downto 0) := (others => '0');
    signal z_m    : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_vt   : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal z_sp   : unsigned(10 downto 0) := (others => '0');
    signal z_d1   : unsigned(12 downto 0) := (others => '0');
    signal z_pos  : std_logic := '1';
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal z_t    : unsigned(25 downto 0) := (others => '0');
    signal z_eg0  : unsigned(22 downto 0) := (others => '0');
    signal z_dU1  : unsigned(15 downto 0) := to_unsigned(8687, 16);
    signal z_dwh  : unsigned(19 downto 0) := to_unsigned(233016, 20);
    signal z_om   : unsigned(21 downto 0) := to_unsigned(359410, 22);
    signal z_drh  : unsigned(15 downto 0) := to_unsigned(5266, 16);

    -- per-field constants
    signal c_du   : unsigned(17 downto 0) := to_unsigned(17374, 18);
    signal c_u0   : unsigned(23 downto 0) := (others => '0');
    signal c_invp : unsigned(13 downto 0) := to_unsigned(3141, 14);
    signal c_kt   : unsigned(27 downto 0) := (others => '0');
    signal c_dv   : unsigned(21 downto 0) := (others => '0');
    signal c_kmd  : unsigned(27 downto 0) := (others => '0');    -- Kt - dv: the wrap threshold
    signal c_v0   : unsigned(27 downto 0) := (others => '0');
    signal c_drc  : unsigned(17 downto 0) := (others => '0');
    signal c_rc0  : unsigned(23 downto 0) := (others => '0');
    signal c_ag   : unsigned(21 downto 0) := to_unsigned(40042, 22);
    signal c_dph  : unsigned(27 downto 0) := (others => '0');
    signal c_ph0  : unsigned(27 downto 0) := (others => '0');
    signal c_c0   : unsigned(23 downto 0) := (others => '0');
    signal c_dflow : unsigned(23 downto 0) := (others => '0');
    signal c_flow : unsigned(23 downto 0) := (others => '0');
    signal c_drol, c_dris, c_dwav : unsigned(15 downto 0) := (others => '0');
    signal c_roll, c_rise, c_wave : unsigned(15 downto 0) := (others => '0');
    signal c_top  : unsigned(17 downto 0) := to_unsigned(300 * 256, 18);
    signal c_dw, c_dw9 : unsigned(15 downto 0) := (others => '0');
    signal c_w0   : unsigned(15 downto 0) := (others => '0');
    signal c_wsw  : unsigned(15 downto 0) := (others => '0');
    signal c_eg   : unsigned(18 downto 0) := (others => '0');

    -- shared serial multiplier (zone sequencer and line engine)
    signal m_a   : unsigned(41 downto 0) := (others => '0');
    signal m_b   : unsigned(23 downto 0) := (others => '0');
    signal m_acc : unsigned(41 downto 0) := (others => '0');
    signal z_fa  : unsigned(25 downto 0) := (others => '0');
    signal z_fb  : unsigned(23 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_phl  : unsigned(27 downto 0) := (others => '0');
    signal e_psl, e_qpl : unsigned(23 downto 0) := (others => '0');
    signal e_wpl  : unsigned(15 downto 0) := (others => '0');
    signal e_tpl  : unsigned(15 downto 0) := (others => '0');
    signal e_band : std_logic := '0';
    signal e_s0, e_c : signed(15 downto 0) := (others => '0');
    signal e_sd   : signed(16 downto 0) := (others => '0');
    signal e_sdn, e_neg, e_nq : std_logic := '0';
    signal e_s25  : signed(27 downto 0) := (others => '0');
    signal e_cf   : unsigned(23 downto 0) := (others => '0');
    signal e_qa   : signed(12 downto 0) := (others => '0');
    signal n_gc   : unsigned(23 downto 0) := (others => '0');
    signal n_ql   : unsigned(11 downto 0) := (others => '0');
    signal n_lv   : unsigned(5 downto 0) := (others => '0');
    signal n_band : std_logic := '0';

    -- beam (stage 0) and the line's values
    signal b_u    : unsigned(23 downto 0) := (others => '0');
    signal b_v    : unsigned(27 downto 0) := (others => '0');
    signal b_rc   : unsigned(23 downto 0) := (others => '0');
    signal b_w, b_ws : unsigned(15 downto 0) := (others => '0');
    signal b_x    : unsigned(11 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal l_gc   : unsigned(23 downto 0) := (others => '0');
    signal l_ql   : unsigned(11 downto 0) := (others => '0');
    signal l_lv   : unsigned(5 downto 0) := (others => '0');
    signal l_band : std_logic := '0';

    -- pixel path
    signal p1_tri, p1_u12 : unsigned(11 downto 0) := (others => '0');
    signal p1_v, p1_vk : unsigned(23 downto 0) := (others => '0');
    signal p1_lo  : std_logic := '0';
    signal p1_rc  : unsigned(23 downto 0) := (others => '0');
    signal p2_t, p2_rc : unsigned(23 downto 0) := (others => '0');
    signal p2_qt, p2_qu : unsigned(11 downto 0) := (others => '0');
    signal p3_rt  : unsigned(23 downto 0) := (others => '0');
    signal p3_wt  : signed(15 downto 0) := (others => '0');
    signal p3_q   : unsigned(11 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_f   : unsigned(23 downto 0) := (others => '0');
    signal p4_q   : unsigned(11 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(15 downto 0) := (others => '0');
    signal p5_f, p5_e : unsigned(23 downto 0) := (others => '0');
    signal p5_q   : unsigned(12 downto 0) := (others => '0');
    signal p5_rg  : unsigned(1 downto 0) := (others => '0');
    signal p6_dl, p6_dh : unsigned(12 downto 0) := (others => '0');
    signal p7_d   : unsigned(12 downto 0) := (others => '0');
    signal p8_fr  : std_logic_vector(31 downto 0) := (others => '0');
    signal p8_ws  : signed(7 downto 0) := (others => '0');
    signal p8_tp  : unsigned(5 downto 0) := (others => '0');
    signal p9_prod : signed(13 downto 0) := (others => '0');
    signal p9_mred, p9_lb1, p9_dgl, p9_gap, p9_env : unsigned(5 downto 0) := (others => '0');
    signal p9_md  : std_logic_vector(1 downto 0) := "00";
    signal p10_z  : unsigned(2 downto 0) := (others => '0');
    signal p10_lv, p10_mred, p10_lb1, p10_dgl : unsigned(5 downto 0) := (others => '0');
    signal p10_rb : std_logic := '0';
    signal p11_z  : unsigned(2 downto 0) := (others => '0');
    signal p11_lv, p11_mred : unsigned(5 downto 0) := (others => '0');
    signal p11_rb : std_logic := '0';
    signal p12_z  : unsigned(2 downto 0) := (others => '0');
    signal p12_lv, p12_mred, p12_wl : unsigned(5 downto 0) := (others => '0');
    signal p12_rb : std_logic := '0';
    signal p15_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal pc_red : std_logic_vector(9 downto 5) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: block-RAM rings (the box's 8-back sample and the syncs, C_LAT back)
    type t_br is array (0 to 255) of std_logic_vector(31 downto 0);
    signal br : t_br := (others => x"10200800");
    type t_sr is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_sr := (others => "1111");
    attribute no_rw_check of br : signal is true;
    attribute no_rw_check of sr : signal is true;
    signal o_wa, o_ra, o_sa : unsigned(7 downto 0) := (others => '0');
    signal o_rq : std_logic_vector(31 downto 0) := x"10200800";
    signal o_q  : std_logic_vector(31 downto 0) := x"10200800";
    signal o_sq : std_logic_vector(3 downto 0) := "1111";
    signal o_so : std_logic_vector(3 downto 0) := "1111";
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if ir_we = '1' then lb(to_integer(ir_wa)) <= ir_wd; end if;
            ir_q <= ir(to_integer(ir_ra));
            if ir_we = '1' then ir(to_integer(ir_wa)) <= ir_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q  <= C_GT_EXP2(to_integer(r_ex_a));
            r_vt_q  <= C_GT_VT(to_integer(r_vt_a));
            r_sp_q  <= C_GT_SP(to_integer(r_sp_a));
            r_sn_q  <= C_GT_SIN(to_integer(r_sn_a));
            r_ws_q  <= C_GT_WS(to_integer(r_ws_a));
            r_wt_q  <= C_GT_WARP(to_integer(r_wt_a));
            r_fr_q  <= C_GT_FR(to_integer(r_fr_a));
            r_pro_q <= C_GT_PRO(to_integer(r_pro_a));
            r_pal_q <= C_GT_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer and the line engine.  They share
    -- one serial multiplier (a x b, b <= 24 bits; load at clock L, result in m_acc
    -- from L + 1 + bits(b)): the zone slots end at zc 635, the line engine runs
    -- from the zone kick (zc 1300) and at each line start.
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_t  : unsigned(23 downto 0);
        variable v_tp : unsigned(19 downto 0);
        variable v_u  : unsigned(19 downto 0);
        variable v_25 : signed(27 downto 0);
        variable v_19 : signed(21 downto 0);
        variable v_a  : signed(21 downto 0);
        variable v_l  : signed(9 downto 0);
        variable v_q  : signed(12 downto 0);
        variable v_p  : unsigned(23 downto 0);
        variable v_g  : unsigned(23 downto 0);
        variable v_ls : unsigned(23 downto 0);
        variable v_d  : unsigned(15 downto 0);
        variable v_bd : unsigned(17 downto 0);
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
                s_cyan <= registers_in(6)(0);
                s_swap <= registers_in(6)(1);
                s_tear <= registers_in(6)(2);
                s_flip <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is                 -- ys * 121 / 256
                when 0          => s_oys <= to_unsigned(537, 10);
                when 4          => s_oys <= to_unsigned(272, 10);
                when 8          => s_oys <= to_unsigned(453, 10);
                when 12         => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10);
                when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others     => s_oys <= to_unsigned(121, 10);
            end case;

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

            -- zone products: slot k loads at zc = 64 + 32 k, results from load + 25 (stored at load + 26).  The
            -- slot number is decoded two clocks ahead and the operands pre-muxed one clock ahead (z_fa / z_fb).
            -- Exponential controls: the exp2 mantissa is read just before its slot and shifted by its octave
            -- on the b operand (one shifter), so every result is a fixed slice.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 62   => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 94   => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 126  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 158  => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 190  => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 222  => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 254  => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 286  => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 318  => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 350  => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 382  => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 414  => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 446  => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 478  => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 510  => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 542  => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 574  => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 606  => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  =>                                     -- du = xs dU1 (the x step folded into the constant)
                        case to_integer(s_xs) is
                            when 4      => z_fa <= to_unsigned(4 * K_DU1, 26);
                            when 3      => z_fa <= to_unsigned(3 * K_DU1, 26);
                            when others => z_fa <= to_unsigned(2 * K_DU1, 26);
                        end case;
                        z_fb <= shift_left(resize(z_m, 24), to_integer(z_sh) + 4);
                    when 1  =>                                     -- u0 = -(x0 - XP) dU1 = -du (2 XP - 2 x0) 4 / xs / 8
                        z_fa <= resize(c_du, 26);
                        case to_integer(s_xs) is
                            when 4      => z_fb <= to_unsigned(3123, 24);
                            when 3      => z_fb <= to_unsigned(5444, 24);
                            when others => z_fb <= to_unsigned(8166, 24);
                        end case;
                    when 2  => z_fa <= to_unsigned(K_INVP, 26);  z_fb <= shift_left(resize(z_m, 24), to_integer(z_sh) + 4);
                    when 3  => z_fa <= to_unsigned(K_OM, 26);    z_fb <= shift_left(resize(z_m, 24), to_integer(z_sh) + 4);
                    when 4  => z_fa <= to_unsigned(K_AG, 26);    z_fb <= shift_left(resize(z_m, 24), to_integer(z_sh) + 4);
                    when 5  => z_fa <= resize(z_a, 26);                     z_fb <= resize(z_vt, 24);
                    when 6  => z_fa <= resize(z_a, 26);          z_fb <= shift_left(resize(z_m, 24), to_integer(z_sh) + 4);
                    when 7  => z_fa <= resize(z_om, 26);                    z_fb <= resize(s_ys, 24);
                    when 8  => z_fa <= resize(z_om, 26);                    z_fb <= to_unsigned(2150, 24);
                    when 9  => z_fa <= resize(c_invp, 26);                  z_fb <= to_unsigned(50064, 24);
                    when 10 => z_fa <= resize(c_invp, 26);                  z_fb <= resize(z_sp, 24);
                    when 11 => z_fa <= z_t;                                 z_fb <= to_unsigned(2458, 24);
                    when 12 =>                                     -- dw = xs dwh
                        case to_integer(s_xs) is
                            when 4      => z_fa <= to_unsigned(4 * K_DWH, 26);
                            when 3      => z_fa <= to_unsigned(3 * K_DWH, 26);
                            when others => z_fa <= to_unsigned(2 * K_DWH, 26);
                        end case;
                        z_fb <= shift_left(resize(z_m, 24), to_integer(z_sh) + 4);
                    when 13 => z_fa <= resize(s_g6(12 downto 3), 26);       z_fb <= resize(s_g6(12 downto 3), 24);
                    when 14 => z_fa <= resize(c_invp, 26);                  z_fb <= to_unsigned(K_PHT, 24);
                    when 15 => z_fa <= resize(c_kt(27 downto 6), 26);       z_fb <= resize(c_du, 24);
                    when 16 => z_fa <= resize(c_kt(27 downto 6), 26);       z_fb <= resize(c_u0(23 downto 4), 24);
                    when others =>                                 -- drc = xs drh = invP 6868 xs >> 12
                        z_fa <= resize(c_invp, 26);
                        case to_integer(s_xs) is
                            when 4      => z_fb <= to_unsigned(4 * 6868, 24);
                            when 3      => z_fb <= to_unsigned(3 * 6868, 24);
                            when others => z_fb <= to_unsigned(2 * 6868, 24);
                        end case;
                end case;
            end if;

            -- the shared multiplier step (loads below override it)
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(40 downto 0) & '0';
            m_b <= '0' & m_b(23 downto 1);
            if z_ld = '1' then
                m_acc <= (others => '0'); m_a <= resize(z_fa, 42); m_b <= z_fb;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps)
                    when 20 =>
                        -- the steps are stored signed (speed sign applied when they are computed)
                        c_flow <= c_flow + c_dflow;
                        c_roll <= c_roll + c_drol; c_rise <= c_rise + c_dris; c_wave <= c_wave + c_dwav;
                        c_top <= c_top + resize(c_drol, 18);
                    when 21 =>
                        case to_integer(s_xs) is                  -- ws0 = 480 * 4660 / 256 (Q16)
                            when 4      => c_wsw <= to_unsigned(8737, 16) + c_wave;
                            when others => c_wsw <= c_wave;
                        end case;
                    when 22 =>
                        r_sp_a <= s_vm(8 downto 1);
                        z_pos <= not s_vneg;
                    -- exponents (Q8 octaves) and exp2 mantissas; the glide has settled each channel by now
                    -- e = c (k - 512) as floor(c k) - 512 c on the unsigned knob value (no sign-extended
                    -- self-adds, so no carry cell with I0 = I1)
                    when 30 =>
                        v_u := resize(s_g1(12 downto 3), 20);
                        v_u := shift_right(v_u + shift_left(v_u, 1), 2);                                -- 3/4 k1
                        z_e1 <= signed(resize(v_u, 12)) - 384;
                        z_sp <= r_sp_q;
                    when 31 =>
                        v_u := resize(s_g3(12 downto 3), 20);
                        v_u := shift_right(shift_left(v_u, 3) + shift_left(v_u, 2) + v_u, 4);          -- 13/16 k3
                        z_e3 <= signed(resize(v_u, 12)) - 416;
                    when 32 =>
                        v_u := resize(s_g2(12 downto 3), 20);
                        v_u := shift_right(shift_left(v_u, 8) + shift_left(v_u, 7) + v_u, 8);          -- 385/256 k2
                        z_e2 <= signed(resize(v_u, 12)) - 770;
                    when 33 =>
                        z_e2n <= -z_e2;
                        v_u := resize(s_g5(12 downto 3), 20);
                        v_u := shift_right(v_u + shift_left(v_u, 1), 2);                                -- 3/4 k5
                        z_e5n <= 384 - signed(resize(v_u, 12));
                    when 35 =>
                        r_vt_a <= s_gp(12 downto 5);
                    -- exp2 mantissa reads, just before each exponential slot (address at L - 7, copy at L - 5)
                    when 57  => r_ex_a <= unsigned(z_e1(7 downto 0));
                    when 59  => z_m <= r_ex_q; z_sh <= z_e1(11 downto 8);
                    when 121 => r_ex_a <= unsigned(z_e3(7 downto 0));
                    when 123 => z_m <= r_ex_q; z_sh <= z_e3(11 downto 8);
                    when 153 => r_ex_a <= unsigned(z_e2(7 downto 0));
                    when 155 => z_m <= r_ex_q; z_sh <= z_e2(11 downto 8);
                    when 185 => r_ex_a <= unsigned(z_e3(7 downto 0));
                    when 187 => z_m <= r_ex_q; z_sh <= z_e3(11 downto 8);
                    when 249 => r_ex_a <= unsigned(z_e2n(7 downto 0));
                    when 251 => z_m <= r_ex_q; z_sh <= z_e2n(11 downto 8);
                    when 441 => r_ex_a <= unsigned(z_e5n(7 downto 0));
                    when 443 => z_m <= r_ex_q; z_sh <= z_e5n(11 downto 8);
                    when 37 =>
                        z_vt <= r_vt_q;
                        z_d1 <= resize(shift_left(resize(z_sp, 13), 1) + shift_right(resize(z_sp, 13), 1), 13);
                    when 38 =>                                    -- roll / tear step 2.625 sp; rise 0.203 sp; wave 1.75 sp
                        v_d := resize(z_d1 + shift_right(resize(z_sp, 13), 3), 16);
                        if z_pos = '1' then c_drol <= v_d; else c_drol <= 0 - v_d; end if;
                        z_d1 <= shift_right(resize(z_sp, 13), 3) + shift_right(resize(z_sp, 13), 4);
                    when 39 =>
                        v_d := resize(z_d1 + shift_right(resize(z_sp, 13), 6), 16);
                        if z_pos = '1' then c_dris <= v_d; else c_dris <= 0 - v_d; end if;
                    when 40 =>
                        v_d := resize(shift_left(resize(z_sp, 16), 1) - shift_right(resize(z_sp, 16), 2), 16);
                        if z_pos = '1' then c_dwav <= v_d; else c_dwav <= 0 - v_d; end if;
                    -- slot results (slot k loads at 64 + 32 k, stored at 90 + 32 k)
                    when 90  => c_du <= m_acc(36 downto 19);
                    when 122 => c_u0 <= (not m_acc(26 downto 3)) + 1;                 -- -(t >> 3)
                    when 154 => c_invp <= m_acc(32 downto 19);
                    when 186 => z_om <= m_acc(40 downto 19);
                    when 218 => z_a <= m_acc(38 downto 19);                           -- A1
                    when 250 => z_a <= m_acc(31 downto 12);                           -- A2 = A1 v >> 12
                    when 282 => c_ag <= m_acc(40 downto 19);
                    when 314 => c_dph <= m_acc(39 downto 12);
                    when 346 => c_ph0 <= m_acc(31 downto 4);
                    when 378 => c_c0 <= (not m_acc(23 downto 0)) + 1;
                    when 410 => z_t <= m_acc(25 downto 0);
                    when 442 =>                                   -- the arches sink at positive speed
                        if z_pos = '1' then c_dflow <= 0 - resize(m_acc(37 downto 16), 24);
                        else c_dflow <= resize(m_acc(37 downto 16), 24); end if;
                    when 474 => c_dw <= resize(m_acc(41 downto 27), 16);
                    when 475 =>                                   -- w0 = 480 dwh = 120 dw (SD only)
                        if s_xs = 4 then c_w0 <= shift_left(c_dw, 7) - shift_left(c_dw, 3);
                        else c_w0 <= (others => '0'); end if;
                        c_dw9 <= shift_left(c_dw, 3) + c_dw;
                    when 506 => z_eg0 <= shift_left(resize(m_acc(19 downto 0), 23), 2) + resize(m_acc(19 downto 0), 23);   -- 5 k6^2
                    when 507 =>                                                       -- eg = 5 k6^2 / 32
                        c_eg <= resize(shift_right(z_eg0, 5), 19);
                    when 538 => c_kt <= m_acc(27 downto 0);
                    when 570 => c_dv <= m_acc(39 downto 18);
                    when 571 => c_kmd <= c_kt - resize(c_dv, 28);
                    when 602 => c_v0 <= m_acc(41 downto 14);
                    when 634 => c_drc <= m_acc(29 downto 12);
                    when 635 =>                                   -- rc0 = 480 drh = 120 drc (SD only)
                        if s_xs = 4 then c_rc0 <= shift_left(resize(c_drc, 24), 7) - shift_left(resize(c_drc, 24), 3);
                        else c_rc0 <= (others => '0'); end if;
                    when others => null;
                end case;
            end if;

            -- line engine: kicked in the zone for line 0 and at each line start; computes the NEXT line
            if s_zone = '1' and s_zc = 1300 then
                e_vy  <= (others => '0');
                e_phl <= c_ph0;
                e_psl <= to_unsigned(K_PS0, 24);
                e_qpl <= to_unsigned(K_QP0, 24);
                e_tpl <= (others => '0');
                e_wpl <= (others => '0');
                e_lc  <= (others => '0');
            elsif g_start = '1' then
                e_vy  <= e_vy + resize(s_ys, 20);
                e_phl <= e_phl + c_dph;
                e_psl <= e_psl + f_lstep(s_tid, 379);
                e_qpl <= e_qpl + f_lstep(s_tid, 30);
                v_ls := f_lstep(s_tid, 14);
                e_tpl <= e_tpl + v_ls(15 downto 0);
                v_ls := f_lstep(s_tid, 560);
                e_wpl <= e_wpl + v_ls(23 downto 8);
                e_lc  <= (others => '0');
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    v_bd := e_vy(17 downto 0) - c_top;
                    e_band <= s_tear and v_bd(17);
                    r_sn_a <= e_phl(27 downto 18);
                when 2 =>
                    r_sn_a <= e_phl(27 downto 18) + 1;
                when 3 =>
                    e_s0 <= r_sn_q;
                when 4 =>
                    e_sd <= resize(r_sn_q, 17) - resize(e_s0, 17);
                    e_cf <= c_c0 + c_flow;
                when 5 =>
                    if e_band = '1' then e_cf <= e_cf + to_unsigned(C_TJ, 24); end if;
                when 6 =>
                    m_a <= resize(unsigned(abs(e_sd)), 42); m_b <= resize(e_phl(17 downto 8), 24); m_acc <= (others => '0');
                    e_sdn <= e_sd(16);
                when 17 =>                    -- s25 = s0 * 1024 + (s1 - s0) * frac
                    if e_sdn = '1' then
                        v_25 := shift_left(resize(e_s0, 28), 10) - signed(resize(m_acc(19 downto 0), 28));
                    else
                        v_25 := shift_left(resize(e_s0, 28), 10) + signed(resize(m_acc(19 downto 0), 28));
                    end if;
                    e_s25 <= v_25;
                when 18 =>                    -- s19 = s25 >> 6; G = Ag |s19| >> 7
                    v_19 := resize(shift_right(e_s25, 6), 22);
                    v_a := abs(v_19);
                    m_a <= resize(c_ag, 42); m_b <= resize(unsigned(v_a(19 downto 0)), 24); m_acc <= (others => '0');
                    e_neg <= v_19(21) xor s_flip;
                when 37 =>
                    v_p := e_psl - shift_left(resize(c_roll, 24), 8);
                    r_sn_a <= v_p(23 downto 14) + 256;
                when 39 =>
                    v_g := m_acc(30 downto 7);
                    if e_neg = '1' then n_gc <= e_cf - v_g; else n_gc <= e_cf + v_g; end if;
                    e_c <= r_sn_q;
                    v_p := e_qpl + shift_left(resize(c_rise, 24), 8);
                    r_sn_a <= v_p(23 downto 14) + 256;
                when 40 =>                    -- stripe level: (|cs| 75) >> 16
                    v_a := abs(resize(e_c, 22));
                    m_a <= resize(unsigned(v_a(15 downto 0)), 42); m_b <= to_unsigned(75, 24); m_acc <= (others => '0');
                    e_neg <= e_c(15);
                when 41 =>
                    e_c <= r_sn_q;
                when 48 =>
                    if e_neg = '1' then v_l := 35 - signed(resize(m_acc(22 downto 16), 10));
                    else v_l := 35 + signed(resize(m_acc(22 downto 16), 10)); end if;
                    if e_band = '1' or v_l < 0 then n_lv <= (others => '0');
                    elsif v_l > 63 then n_lv <= (others => '1');
                    else n_lv <= unsigned(v_l(5 downto 0)); end if;
                    -- key: (|cq| 985) >> 15
                    v_a := abs(resize(e_c, 22));
                    m_a <= resize(unsigned(v_a(15 downto 0)), 42); m_b <= to_unsigned(985, 24); m_acc <= (others => '0');
                    e_nq <= e_c(15);
                when 59 =>
                    if e_nq = '1' then v_q := to_signed(K_QC, 13) - signed(resize(m_acc(24 downto 15), 13));
                    else v_q := to_signed(K_QC, 13) + signed(resize(m_acc(24 downto 15), 13)); end if;
                    e_qa <= v_q;
                    -- teeth: (t6 41) >> 6
                    m_a <= resize(e_tpl(15 downto 10), 42); m_b <= to_unsigned(41, 24); m_acc <= (others => '0');
                when 66 =>
                    n_ql <= unsigned(e_qa(11 downto 0)) + resize(m_acc(11 downto 6), 12);
                    n_band <= e_band;
                when others => null;
            end case;

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
        variable v_f : unsigned(8 downto 0);
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

            -- smoothing: pair sums (odd pixel), 8 px running box of the last 4 pairs, IIR with the previous line
            q_lp <= unsigned(s_lb_wd);
            if s_lb_we = '1' and s_lb_wa(0) = '1' then
                q_pr <= resize(q_lp, 9) + resize(unsigned(s_lb_wd), 9); q_pv <= '1'; q_k <= s_lb_wa(10 downto 1);
            else
                q_pv <= '0';
            end if;
            q_avh <= q_avh(2 downto 0) & s_lb_we;
            if q_avh = "0000" then                     -- between lines: the box restarts empty
                q_hs <= (others => '0');
                q_h1 <= (others => '0'); q_h2 <= (others => '0'); q_h3 <= (others => '0'); q_h4 <= (others => '0');
            elsif q_pv = '1' then
                q_hs <= q_hs + resize(q_pr, 11) - resize(q_h4, 11);
                q_h4 <= q_h3; q_h3 <= q_h2; q_h2 <= q_h1; q_h1 <= q_pr;
            end if;
            ir_ra <= q_k; q_pv1 <= q_pv;
            q_k2 <= ir_ra; q_pv2 <= q_pv1;
            q_ic <= unsigned(ir_q); q_hb2 <= q_hs(10 downto 3); q_k3 <= q_k2; q_pv3 <= q_pv2;   -- the box sum is steady for 2 clocks
            v_f := resize(q_ic, 9) + resize(q_hb2, 9);
            ir_wd <= std_logic_vector(v_f(8 downto 1)); ir_wa <= q_k3; ir_we <= q_pv3;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): x in h units and the x DDAs; the line's values load at the
    -- line start.  b_v is the triangle-times-1/P DDA, modulo Kt (one arch
    -- period); both candidates are formed in parallel and the wrapped one wins.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_a, v_b : unsigned(27 downto 0);
        variable v_ws : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= resize(s_x0, 12);
                b_px <= (others => '0');
                b_u  <= c_u0; b_v <= c_v0; b_rc <= c_rc0; b_w <= c_w0;
                b_ws <= e_wpl + c_wsw;
                l_gc <= n_gc; l_ql <= n_ql; l_lv <= n_lv; l_band <= n_band;
            else
                if g_avid = '1' then b_x <= b_x + s_xs; end if;
                b_px <= b_px + 1;
                b_u  <= b_u + c_du;
                v_a := b_v + resize(c_dv, 28);
                v_b := b_v - c_kmd;
                if b_v >= c_kmd then b_v <= v_b; else b_v <= v_a; end if;
                b_rc <= b_rc + c_drc;
                b_w  <= b_w + c_dw;
                case to_integer(s_xs) is
                    when 4      => b_ws <= b_ws + to_unsigned(73, 16);
                    when 3      => b_ws <= b_ws + to_unsigned(55, 16);
                    when others => b_ws <= b_ws + to_unsigned(36, 16);
                end case;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path.  Late-stage x quantities are read from the beam with a lag
    -- (b_ws - 5 dws, b_w - 9 dw) instead of being carried.
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t   : unsigned(11 downto 0);
        variable v_q   : unsigned(12 downto 0);
        variable v_f   : unsigned(23 downto 0);
        variable v_w   : unsigned(15 downto 0);
        variable v_z   : unsigned(2 downto 0);
        variable v_lv  : unsigned(5 downto 0);
        variable v_red : std_logic;
        variable v_wb  : signed(6 downto 0);
        variable v_h   : unsigned(27 downto 0);
        variable v_x   : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: triangle, warp ROM address, fold test of the v DDA, line-buffer address
            if b_u(23) = '1' then v_t := not b_u(22 downto 11); else v_t := b_u(22 downto 11); end if;
            p1_tri <= v_t;
            p1_u12 <= b_u(23 downto 12);
            if b_x(11 downto 3) > 479 then r_wt_a <= to_unsigned(479, 9); else r_wt_a <= b_x(11 downto 3); end if;
            if b_v(26 downto 0) < c_kt(27 downto 1) and b_v(27) = '0' then p1_lo <= '1'; else p1_lo <= '0'; end if;
            p1_v <= b_v(23 downto 0);
            v_h := c_kt - b_v;
            p1_vk <= v_h(23 downto 0);
            p1_rc <= b_rc;
            v_x := b_px + 8;                               -- the smoothed luma, read ahead of its write
            s_lb_ra <= v_x(10 downto 1);
            -- 2: T = PH tri / P (mod one cycle); key partials 0.656 tri, 0.320 u
            if p1_lo = '1' then p2_t <= p1_v; else p2_t <= p1_vk; end if;
            p2_rc <= p1_rc;
            p2_qt <= shift_right(p1_tri, 1) + shift_right(p1_tri, 3) + shift_right(p1_tri, 5);
            p2_qu <= shift_right(p1_u12, 2) + shift_right(p1_u12, 4) + shift_right(p1_u12, 7);
            -- 3: R - T; warp and luma copies; key sum
            p3_rt <= p2_rc - p2_t;
            p3_wt <= r_wt_q;
            p3_lm <= unsigned(s_lb_q);
            p3_q <= p2_qt + p2_qu;
            -- 4: + warp; key + line term; etch partials (luma nibbles x gain)
            p4_f <= p3_rt + unsigned(shift_left(resize(p3_wt, 24), 8));
            p4_q <= p3_q + l_ql;
            p4_ea <= p3_lm(3 downto 0) * c_eg(17 downto 6);
            p4_eb <= p3_lm(7 downto 4) * c_eg(17 downto 6);
            -- 5: + line term; etch sum; key region edges and the red test
            p5_f <= p4_f + l_gc;
            if s_src = '1' then                           -- the Osc rides x32 the camera's bend
                p5_e <= shift_left(resize(p4_ea, 24), 11) + shift_left(resize(p4_eb, 24), 15);
            else
                p5_e <= shift_left(resize(p4_ea, 24), 6) + shift_left(resize(p4_eb, 24), 10);
            end if;
            v_q := resize(p4_q, 13);
            if p4_q < 492 then
                p5_rg <= "00"; v_q := v_q + 4096; v_red := '1';
            elsif p4_q < 1475 then
                p5_rg <= "01"; v_red := '0';
            elsif p4_q < 1802 then
                p5_rg <= "10"; v_red := '1';
            elsif p4_q < 2867 then
                p5_rg <= "11"; v_red := '0';
            else
                p5_rg <= "00"; v_red := '1';
            end if;
            p5_q <= v_q;
            pc_red(5) <= v_red xor s_swap xor l_band;
            -- 6: + etch -> frame ROM address; wobble sine address (pixel's phase = beam - 5 dws); edge distances
            v_f := p5_f + p5_e;
            r_fr_a <= v_f(23 downto 16);
            case to_integer(s_xs) is
                when 4      => v_w := b_ws - to_unsigned(365, 16);
                when 3      => v_w := b_ws - to_unsigned(275, 16);
                when others => v_w := b_ws - to_unsigned(180, 16);
            end case;
            r_ws_a <= v_w(15 downto 8);
            case p5_rg is                                  -- region edges: red [2867, 4588), blue [492, 1475), ...
                when "00"   => p6_dl <= p5_q - 2867; p6_dh <= 4588 - p5_q;
                when "01"   => p6_dl <= p5_q - 492;  p6_dh <= 1475 - p5_q;
                when "10"   => p6_dl <= p5_q - 1475; p6_dh <= 1802 - p5_q;
                when others => p6_dl <= p5_q - 1802; p6_dh <= 2867 - p5_q;
            end case;
            -- 7: nearest key edge
            if p6_dl < p6_dh then p7_d <= p6_dl; else p7_d <= p6_dh; end if;
            -- 8: frame word and wobble sine copies; key taper
            p8_fr <= r_fr_q;
            p8_ws <= r_ws_q;
            if p7_d(12 downto 3) > 63 then p8_tp <= (others => '1'); else p8_tp <= p7_d(8 downto 3); end if;
            -- 9: wobble product; minima
            v_wb := signed(resize(unsigned(p8_fr(5 downto 1)), 7));
            p9_prod <= resize(v_wb(5 downto 0) * p8_ws(7 downto 1), 14);
            p9_mred <= f_min6(unsigned(p8_fr(11 downto 6)), unsigned(p8_fr(29 downto 24)));
            p9_lb1  <= f_min6(unsigned(p8_fr(23 downto 18)), p8_tp);
            p9_dgl  <= f_min6(unsigned(p8_fr(17 downto 12)), unsigned(p8_fr(29 downto 24)));
            p9_gap  <= f_min6(l_lv, unsigned(p8_fr(29 downto 24)));
            p9_env  <= unsigned(p8_fr(29 downto 24));
            p9_md   <= p8_fr(31 downto 30);
            -- 10: red line phase (pixel's phase = beam - 9 dw) -> profile ROM address; non-red zone choice
            v_w := b_w - c_dw9;
            v_t := v_w(15 downto 4) + shift_left(unsigned(p9_prod(11 downto 0)), 2);
            r_pro_a <= v_t(11 downto 6);
            p10_lb1 <= f_min6(p9_lb1, p9_env);
            p10_dgl <= p9_dgl;
            p10_mred <= p9_mred;
            if p9_md = "00" then
                if s_cyan = '1' then p10_z <= "001"; else p10_z <= "000"; end if;
                p10_lv <= p9_gap; p10_rb <= '0';
            elsif p9_md = "01" then
                p10_z <= "010"; p10_lv <= p9_env; p10_rb <= '0';
            else
                p10_z <= "100"; p10_lv <= p9_env; p10_rb <= pc_red(9);
            end if;
            -- 11: blue vs dark green
            p11_z <= p10_z; p11_lv <= p10_lv; p11_rb <= p10_rb; p11_mred <= p10_mred;
            if p10_z = "100" then
                if p10_lb1 >= p10_dgl then p11_lv <= p10_lb1; else p11_z <= "101"; p11_lv <= p10_dgl; end if;
            end if;
            -- 12: profile copy
            p12_wl <= r_pro_q;
            p12_z <= p11_z; p12_lv <= p11_lv; p12_rb <= p11_rb; p12_mred <= p11_mred;
            -- 13: zone and level -> palette address
            if p12_rb = '1' then
                v_z := "011"; v_lv := f_min6(p12_wl, p12_mred);
            else
                v_z := p12_z; v_lv := p12_lv;
            end if;
            r_pal_a <= v_z & v_lv;
            -- 15: palette copy
            p15_q <= r_pal_q;
            -- 16: compose; palette black outside the generated avid
            if s_gav_sr(14) = '0' then
                s_y16 <= C_KY; s_u16 <= C_KU; s_v16 <= C_KV;
            else
                s_y16 <= unsigned(p15_q(31 downto 22)); s_u16 <= unsigned(p15_q(21 downto 12)); s_v16 <= unsigned(p15_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
            for i in 6 to 9 loop pc_red(i) <= pc_red(i - 1); end loop;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the HD HDMI bar fix and the still's softness), luma
    -- 13/16 and the blanking gate.  The running 8-sums snap back to black once
    -- 8 blank samples are in, so a glitched sample can't offset the colours.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- rings: write this clock's compose output and input syncs; the box reads 8 back (o_q), the syncs
            -- C_LAT back (o_so): read address registered, EBR output register, one fabric register
            br(to_integer(o_wa)) <= std_logic_vector(s_y16) & std_logic_vector(s_u16) & std_logic_vector(s_v16) & "00";
            sr(to_integer(o_wa)) <= '1' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            o_wa <= o_wa + 1;
            o_ra <= o_wa - 5;
            o_sa <= o_wa - (C_LAT - 3);
            o_rq <= br(to_integer(o_ra));
            o_sq <= sr(to_integer(o_sa));
            o_q  <= o_rq;
            o_so <= o_sq;
            -- box: stage 17 sums, stage 18 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y16 - unsigned(o_q(31 downto 22));
                s_bau <= s_bau + s_u16 - unsigned(o_q(21 downto 12));
                s_bav <= s_bav + s_v16 - unsigned(o_q(11 downto 2));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bay, 14) + s_bayd; s_y18 <= v_sc(13 downto 4);
            v_sc := resize(s_bau, 14) + s_baud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v18 <= v_sc(13 downto 4);
            -- 19: luma 13/16
            if s_y18 > 64 then v_dl := s_y18 - 64; else v_dl := (others => '0'); end if;
            s_y19 <= s_y18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u19 <= s_u18; s_v19 <= s_v18;
            -- 20: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= o_so(0);
    data_out.vsync_n <= o_so(1);
    data_out.field_n <= o_so(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture gothic;
