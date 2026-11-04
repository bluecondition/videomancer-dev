-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Pebble v0.1.0 -- Heirloom phase 3 (video clips), from the clip WIN_20230731_01_19_22 at 0.5 s.
--
-- The still: five red "(" ripples about 337 px apart, each an orange -> yellow ramp with a window onto a
-- blue / magenta layer at its crest; a 64-line square wave turns every other band of 32 rows into navy -> green
-- ramps (the "dotted" rows); a mirrored two-copy figure hangs in from the top, its bottom a crown of rounded
-- tongues outlined in yellow.  In the clip the ripples' bulge scrolls down ~75 lines/s and Ron lowers the figure
-- (by hand) until it fills the screen.
--
-- The patch, as fitted (pebble.py has every constant):
--   u     = frac((x - Xc) g(y) + phi0),  g(y) = a + b cos(2 pi (y - yv) / 1750)   a ramp times (DC + V cosine)
--   rows  = a vertical square wave (63.75 lines): FIRE_A / FIRE_B colourisers over u
--   key   = a window on the colouriser's crest showing the blue layer
--   blue  = a two-copy mirrored figure layer (axes 525 + 960 k), COOL inside / GREEN outside, chevron-indexed
--   veil  = the figure keyed from the top, crown-tongue bottom edge, yellow outline
--
--   K1  Ripples   ring count x1/4 .. x4 (512 = the photo)
--   K2  Bend      the ripples' bow, bipolar: ")" .. straight (512) .. "(" (671 = the photo) .. folded loops
--   K3  Origin    the ramp origin (the drop point): far left .. the photo (512) .. mid screen (800) .. right
--   K4  Speed     bipolar: the bulge scrolls down (+) / up (-); 682 = the clip's 75 lines/s; the figure flows
--   K5  Inlay     width of the window at each ripple's crest: none .. the photo (512) .. nearly the whole ring
--   K6  Rows      height of the dotted rows: 8 .. 64 (512, the photo) .. 512 lines
--   S7  Twin      the ripples mirrored about the screen centre
--   S8  Palette   Fire (the photo) / Ice (Cb <-> Cr: blue ripples, cyan crests, red figure)
--   S9  Weave     the B rows shifted half a ripple (a brick / checker weave)
--   S10 Bend      Arc (the photo's cosine) / Zigzag (a triangle at 4x: zigzag ripples)
--   S11 Source    Video / Osc
--   P12 VEIL      the figure's descent: gone (0) .. the photo's crown at the top (314) .. two full figures (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: interpolated cosine, g = a + b wave, the DDA start and step, rows, figure terms),
-- a ripple DDA (reversing at the centre for Twin) and a mirrored counter for the figure, a 21-stage pixel path.
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (Y, U and V
-- through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum resync, luma 13/16) before the blanking gate.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.pebble_pkg.all;
use work.all;

architecture pebble of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_A0   : integer := 21935;      -- ring gain a per half virtual px (2^24 per ring) at K1 = 512
    constant C_PHI0 : integer := 12918456;   -- ring phase at the ramp origin (0.770)
    constant C_WPHI : integer := 182842893;  -- cosine phase at line 0 (1 - 558 / 1750), Q28
    constant C_CA   : integer := 34177;      -- A-row window centre (0.5215), Q16
    constant C_CB   : integer := 38011;      -- B-row window centre (0.58), Q16

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
    -- triangle of a Q16 phase: 0 at 0, 65535 at 1/2 (Q16 0..1)
    function f_tri16(v : unsigned(15 downto 0)) return unsigned is
    begin
        if v(15) = '0' then return v(14 downto 0) & '0'; else return (not v(14 downto 0)) & '1'; end if;
    end function;

    -- line buffer: 4-pixel box of (luma >> 2) - 16
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    attribute no_rw_check : boolean;
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_l0, s_l1, s_l2, s_l3 : unsigned(7 downto 0) := (others => '0');
    signal s_lav   : std_logic := '0';

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a : unsigned(9 downto 0) := (others => '0');
    signal r_cs_q : signed(15 downto 0) := (others => '0');
    signal r_fi_a : unsigned(8 downto 0) := (others => '0');
    signal r_fi_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_bl_a : unsigned(7 downto 0) := (others => '0');
    signal r_bl_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_cr_a : unsigned(7 downto 0) := (others => '0');
    signal r_cr_q : unsigned(7 downto 0) := (others => '0');

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
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(671 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(314 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(682, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_ice, s_weave, s_zig, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_d    : unsigned(8 downto 0) := (others => '0');
    signal z_x    : unsigned(35 downto 0) := (others => '0');
    signal c_a    : unsigned(17 downto 0) := to_unsigned(C_A0, 18);  -- ring gain a per h, 2^24 per ring
    signal c_b    : unsigned(17 downto 0) := to_unsigned(3162, 18);  -- |b|
    signal c_bneg : std_logic := '0';
    signal c_dx0  : signed(13 downto 0) := to_signed(386, 14);       -- line-start x - Xc (h)
    signal c_vst  : unsigned(20 downto 0) := (others => '0');        -- |bulge scroll| per field, Q28 cycles
    signal c_wph0 : unsigned(27 downto 0) := to_unsigned(C_WPHI, 28);
    signal c_tfl  : unsigned(15 downto 0) := (others => '0');        -- figure flow, Q16 palette cycles
    signal c_winc : unsigned(21 downto 0) := to_unsigned(153344, 22);
    signal c_krow : unsigned(21 downto 0) := to_unsigned(263172, 22);
    signal c_rinc : unsigned(23 downto 0) := to_unsigned(263172, 24);
    signal c_rp0  : unsigned(23 downto 0) := (others => '0');
    signal c_hwa, c_hwb : unsigned(15 downto 0) := to_unsigned(8832, 16);
    signal c_p2   : unsigned(19 downto 0) := (others => '0');
    signal c_veil : signed(11 downto 0) := to_signed(50, 12);
    signal c_va   : unsigned(10 downto 0) := (others => '0');
    signal c_amp  : unsigned(8 downto 0) := to_unsigned(45, 9);      -- crown tongue depth (lines)
    signal c_ca0, c_cb0 : unsigned(10 downto 0) := (others => '0');  -- figure counter start (A / B rows)
    signal c_nc   : unsigned(10 downto 0) := to_unsigned(960, 11);   -- Twin: the centre pixel

    -- field serial multiplier (20-bit b operand, 20 iterations)
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(19 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(19 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
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
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_wp   : unsigned(27 downto 0) := (others => '0');
    signal e_rp   : unsigned(23 downto 0) := (others => '0');
    signal e_fr   : unsigned(7 downto 0) := (others => '0');
    signal e_zt   : signed(16 downto 0) := (others => '0');
    signal e_c0, e_c1 : signed(15 downto 0) := (others => '0');
    signal e_dneg, e_wneg, e_gneg : std_logic := '0';
    signal e_wave : signed(16 downto 0) := (others => '0');
    signal e_g    : signed(18 downto 0) := (others => '0');
    signal e_yp   : signed(12 downto 0) := (others => '0');
    signal e_sv   : signed(15 downto 0) := (others => '0');
    signal e_isb  : std_logic := '0';
    signal m_a    : unsigned(33 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(33 downto 0) := (others => '0');
    -- next line's values (loaded with the line at g_start)
    signal n_u0   : unsigned(23 downto 0) := (others => '0');
    signal n_du   : signed(21 downto 0) := (others => '0');
    signal n_isb  : std_logic := '0';
    signal n_c0   : unsigned(10 downto 0) := (others => '0');
    signal n_pl   : unsigned(9 downto 0) := (others => '0');
    signal n_cl   : unsigned(15 downto 0) := (others => '0');
    signal n_dl   : signed(11 downto 0) := (others => '0');
    signal n_sc, n_se, n_si : unsigned(9 downto 0) := (others => '0');

    -- beam
    signal b_u   : unsigned(23 downto 0) := (others => '0');
    signal b_du  : signed(21 downto 0) := (others => '0');
    signal b_c   : unsigned(10 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_back : std_logic := '0';
    signal l_isb : std_logic := '0';
    signal l_pl  : unsigned(9 downto 0) := (others => '0');
    signal l_cl  : unsigned(15 downto 0) := (others => '0');
    signal l_dl  : signed(11 downto 0) := (others => '0');
    signal l_sc, l_se, l_si : unsigned(9 downto 0) := (others => '0');

    -- pixel path
    signal p2_mh  : signed(11 downto 0) := (others => '0');
    signal p2_u, p3_u, p4_u, p5_u : unsigned(23 downto 0) := (others => '0');
    signal p3_m, p4_m : unsigned(9 downto 0) := (others => '0');
    signal p3_s, p4_s : unsigned(7 downto 0) := (others => '0');
    signal p4_m1a, p4_m1b, p4_m2a, p4_m2b, p4_mca, p4_mcb : unsigned(15 downto 0) := (others => '0');
    signal p4_maa, p4_mab : unsigned(10 downto 0) := (others => '0');
    signal p4_s3a : unsigned(9 downto 0) := (others => '0');
    signal p4_s7a, p4_s7b : unsigned(14 downto 0) := (others => '0');
    signal p4_ue  : unsigned(23 downto 0) := (others => '0');
    signal p5_m1, p5_m2, p5_mc : unsigned(15 downto 0) := (others => '0');
    signal p5_ma  : unsigned(10 downto 0) := (others => '0');
    signal p5_s35 : unsigned(9 downto 0) := (others => '0');
    signal p5_s75 : unsigned(14 downto 0) := (others => '0');
    signal p5_ue  : unsigned(23 downto 0) := (others => '0');
    signal p6_uf  : unsigned(23 downto 0) := (others => '0');
    signal p6_t1, p6_ph2, p6_fc : unsigned(15 downto 0) := (others => '0');
    signal p6_pp, p6_s35, p7_pp : unsigned(9 downto 0) := (others => '0');
    signal p6_dd, p7_dd, p8_dd, p9_dd, p10_dd, p11_dd : signed(12 downto 0) := (others => '0');
    signal p7_da, p7_db : signed(15 downto 0) := (others => '0');
    signal p7_t2  : unsigned(15 downto 0) := (others => '0');
    signal p7_t1s : unsigned(9 downto 0) := (others => '0');
    signal p8_aa, p8_ab : unsigned(15 downto 0) := (others => '0');
    signal p7_fa  : unsigned(8 downto 0) := (others => '0');
    signal p9_ba  : unsigned(7 downto 0) := (others => '0');
    signal p8_p   : unsigned(9 downto 0) := (others => '0');
    signal p9_key, p10_key, p11_key, p12_key : std_logic := '0';
    signal p8_fa, p9_fa : unsigned(8 downto 0) := (others => '0');
    signal p12_fi : std_logic_vector(31 downto 0) := (others => '0');
    signal p9_cs  : unsigned(7 downto 0) := (others => '0');
    signal p10_pr : unsigned(16 downto 0) := (others => '0');
    signal p12_bl : std_logic_vector(31 downto 0) := (others => '0');
    signal p12_cur, p12_outl : std_logic := '0';
    -- flag pipes, indexed by stage: row type, outer (green) side, inside the veil's sides, on the side outline
    signal q_isb  : std_logic_vector(2 to 8) := (others => '0');
    signal q_out  : std_logic_vector(4 to 8) := (others => '0');
    signal q_ine, q_msd : std_logic_vector(4 to 11) := (others => '0');
    signal s_y13, s_u13, s_v13, s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: the boxes' 8-back taps come from block-RAM rings (read 6 back + two registers)
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal r_ra, r_rb : t_ring := (others => x"10200800");
    attribute no_rw_check of r_ra : signal is true;
    attribute no_rw_check of r_rb : signal is true;
    signal r_wp  : unsigned(7 downto 0) := (others => '0');
    signal r_aq, r_aq2, r_bq, r_bq2 : std_logic_vector(31 downto 0) := x"10200800";
    -- sync ring (hsync, vsync, field): read C_LAT - 2 back + two registers = C_LAT clocks
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal r_rs  : t_sring := (others => (others => '1'));
    attribute no_rw_check of r_rs : signal is true;
    signal r_sq, r_sq2 : std_logic_vector(3 downto 0) := (others => '1');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
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
            r_ex_q <= C_PB_EXP2(to_integer(r_ex_a));
            r_cs_q <= C_PB_COS(to_integer(r_cs_a));
            r_fi_q <= C_PB_FIRE(to_integer(r_fi_a));
            r_bl_q <= C_PB_BLUE(to_integer(r_bl_a));
            r_cr_q <= C_PB_CROWN(to_integer(r_cr_a));
        end if;
    end process p_rom;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_wp <= r_wp + 1;
            r_rs(to_integer(r_wp)) <= '1' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            r_sq <= r_rs(to_integer(r_wp - (C_LAT - 2)));
            r_sq2 <= r_sq;
            r_ra(to_integer(r_wp)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15) & std_logic_vector(s_v15) & "00";
            r_aq <= r_ra(to_integer(r_wp - 6));
            r_aq2 <= r_aq;
            r_rb(to_integer(r_wp)) <= std_logic_vector(s_y17) & std_logic_vector(s_u17) & std_logic_vector(s_v17) & "00";
            r_bq <= r_rb(to_integer(r_wp - 6));
            r_bq2 <= r_bq;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_q  : unsigned(15 downto 0);
        variable v_x  : signed(14 downto 0);
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
                s_twin  <= registers_in(6)(0);
                s_ice   <= registers_in(6)(1);
                s_weave <= registers_in(6)(2);
                s_zig   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
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

            -- serial products: slot k loads at zc = 72 + 24 k, 20 iterations, result stable from
            -- load + 21 (stores at load + 22).  The slot number is decoded at 70 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 71 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 70  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 94  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 118 => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 142 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 166 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 190 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 214 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 238 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 262 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 286 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 310 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 334 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(C_A0, 36);           z_fb <= resize(z_mt, 20);        -- a
                    when 1  => z_fa <= resize(z_d, 36);                 z_fb <= resize(z_d, 20);         -- bend d^2
                    when 2  => z_fa <= resize(c_a, 36);                 z_fb <= z_x(19 downto 0);        -- a d^2
                    when 3  => z_fa <= resize(z_x(34 downto 16), 36);   z_fb <= to_unsigned(27160, 20);  -- b
                    when 4  => z_fa <= resize(s_vm, 36);                z_fb <= resize(s_vm, 20);        -- speed^2
                    when 5  => z_fa <= resize(z_x(17 downto 0), 36);    z_fb <= to_unsigned(29801, 20);  -- scroll
                    when 6  => z_fa <= resize(s_ys, 36);                z_fb <= to_unsigned(599, 20);    -- cosine step
                    when 7  => z_fa <= to_unsigned(263172, 36);         z_fb <= resize(z_mt, 20);        -- rows
                    when 8  => z_fa <= resize(c_krow, 36);              z_fb <= resize(s_ys, 20);        -- row step
                    when 9  => z_fa <= resize(c_krow, 36);              z_fb <= to_unsigned(38, 20);     -- row phase
                    when 10 => z_fa <= resize(s_gp(12 downto 3), 36);   z_fb <= resize(s_gp(12 downto 3), 20); -- p12^2
                    when others => z_fa <= resize(c_p2, 36);            z_fb <= to_unsigned(1174, 20);   -- veil
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(19 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- ripples: e = k1 - 512 (Q8 octaves)
                    when 40 =>
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                        -- bend: d = |k2 - 512| - 8
                        v_k := s_g2(12 downto 3);
                        if v_k >= 520 then z_d <= resize(v_k - 520, 9); c_bneg <= '0';
                        elsif v_k <= 504 then z_d <= resize(504 - v_k, 9); c_bneg <= '1';
                        else z_d <= (others => '0'); c_bneg <= '0'; end if;
                    when 42 => z_mt <= r_ex_q;
                    when 93 =>                       -- a = 21935 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_a <= resize(f_acc(35 downto 17), 18);
                            when -1     => c_a <= resize(f_acc(35 downto 16), 18);
                            when 0      => c_a <= resize(f_acc(35 downto 15), 18);
                            when others => c_a <= resize(f_acc(35 downto 14), 18);
                        end case;
                    when 117 => z_x <= f_acc;                                  -- d^2
                    when 141 => z_x <= f_acc;                                  -- a d^2
                    when 165 => c_b <= resize(f_acc(35 downto 16), 18);       -- b = (a d^2 >> 16) 27160 >> 16
                    when 189 => z_x <= f_acc;                                  -- speed^2
                    when 213 => c_vst <= resize(f_acc(35 downto 12), 21);      -- scroll per field, Q28
                    when 214 =>                       -- rows exponent: -1.5 (k6 - 512)
                        v_e := signed(resize(s_g6(12 downto 3), 11)) - 512;
                        v_e := -(v_e + shift_right(v_e, 1));
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 216 => z_mt <= r_ex_q;
                    when 237 => c_winc <= f_acc(21 downto 0);
                    when 261 =>                       -- krow = 263172 mt >> (15 - sh), sh = -3 .. 2
                        case to_integer(z_sh) is
                            when -3     => c_krow <= resize(f_acc(35 downto 18), 22);
                            when -2     => c_krow <= resize(f_acc(35 downto 17), 22);
                            when -1     => c_krow <= resize(f_acc(35 downto 16), 22);
                            when 0      => c_krow <= resize(f_acc(35 downto 15), 22);
                            when 1      => c_krow <= f_acc(35 downto 14);
                            when 2      => c_krow <= f_acc(34 downto 13);
                            when others => c_krow <= f_acc(33 downto 12);
                        end case;
                    when 285 => c_rinc <= f_acc(31 downto 8);
                    when 309 => c_rp0 <= (not f_acc(23 downto 0)) + 1;
                    when 333 => c_p2 <= f_acc(19 downto 0);
                    when 357 => c_veil <= signed(resize(f_acc(30 downto 20), 12)) - 60;
                    when 358 =>
                        if c_veil > 50 then c_va <= unsigned(c_veil(10 downto 0)) - 50; else c_va <= (others => '0'); end if;
                        -- inlay half-widths (Q16 of a ring)
                        v_k := s_g5(12 downto 3);
                        if v_k < 512 then
                            v_q := shift_left(resize(v_k, 16), 4) + resize(v_k, 16) + resize(v_k(9 downto 2), 16);
                        else
                            v_m := v_k - 512;
                            v_q := to_unsigned(8832, 16) + shift_left(resize(v_m, 16), 5) + shift_left(resize(v_m, 16), 3)
                                   + shift_left(resize(v_m, 16), 2) + resize(v_m(9 downto 2), 16);
                        end if;
                        c_hwa <= v_q;
                    when 359 =>
                        c_amp <= to_unsigned(45, 9) + resize(c_va(10 downto 3), 9) + resize(c_va(10 downto 6), 9)
                                 + resize(c_va(10 downto 7), 9);
                        if c_hwa > 7176 then c_hwb <= c_hwa - 7176; else c_hwb <= (others => '0'); end if;
                        -- ramp origin Xc = -386 + 8 (k3 - 512) h; dx0 = x0 - Xc
                        v_x := signed(resize(s_x0, 15)) + 386 - shift_left(signed(resize(s_g3(12 downto 3), 15)) - 512, 3);
                        c_dx0 <= v_x(13 downto 0);
                        if s_xs = 4 then
                            c_ca0 <= to_unsigned(390, 11); c_cb0 <= to_unsigned(438, 11);
                        else
                            c_ca0 <= to_unsigned(1830, 11); c_cb0 <= to_unsigned(1878, 11);
                        end if;
                        c_nc <= s_w(11 downto 1);
                    when 360 =>                       -- motion: the bulge scrolls, the figure flows
                        if s_vneg = '1' then
                            c_wph0 <= c_wph0 + resize(c_vst, 28);
                            c_tfl  <= c_tfl - resize(c_vst(20 downto 10), 16);
                        else
                            c_wph0 <= c_wph0 - resize(c_vst, 28);
                            c_tfl  <= c_tfl + resize(c_vst(20 downto 10), 16);
                        end if;
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
    -- write of a 4-pixel box of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_t : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            if s_src = '1' then v_l := s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then v_l := v_l - 16; else v_l := (others => '0'); end if;
            end if;
            s_lav <= data_in.avid;
            s_l0 <= v_l; s_l1 <= s_l0; s_l2 <= s_l1; s_l3 <= s_l2;
            v_t := resize(s_l0, 10) + resize(s_l1, 10) + resize(s_l2, 10) + resize(s_l3, 10);
            s_lb_wd <= std_logic_vector(v_t(9 downto 2));
            s_lb_we <= s_lav;
            s_lb_wa <= s_wcol;
            if s_lav = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17): the interpolated cosine (or the zigzag), g = a +
    -- b wave, the DDA step g xs and start phi0 + g dx0, the row type, the
    -- figure's side wobble and line terms
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(16 downto 0);
        variable v_w  : signed(16 downto 0);
        variable v_g  : signed(18 downto 0);
        variable v_t  : unsigned(15 downto 0);
        variable v_y  : signed(12 downto 0);
        variable v_sp : signed(20 downto 0);
        variable v_sd : signed(20 downto 0);
        variable v_p  : signed(15 downto 0);
        variable v_du : signed(21 downto 0);
        variable v_u  : unsigned(23 downto 0);
        variable v_bw : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 700 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0'); e_wp <= c_wph0; e_rp <= c_rp0;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_wp <= e_wp + resize(c_winc, 28);
                e_rp <= e_rp + c_rinc;
                e_lc <= (others => '0');
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(32 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    r_cs_a <= e_wp(27 downto 18);
                    e_fr <= e_wp(17 downto 10);
                    -- zigzag: 1 - 2 tri(4 p), Q15
                    v_t := f_tri16(e_wp(25 downto 10));
                    e_zt <= to_signed(32767, 17) - signed(resize(v_t, 17));
                    e_isb <= not e_rp(23);
                    v_y := signed(resize(e_vy(18 downto 8), 13)) - resize(c_veil, 13);
                    e_yp <= v_y;
                    n_dl <= resize(c_veil, 12) - signed(resize(e_vy(18 downto 8), 12));
                when 2 =>
                    r_cs_a <= e_wp(27 downto 18) + 1;
                    if e_isb = '1' then n_c0 <= c_cb0; else n_c0 <= c_ca0; end if;
                    n_isb <= e_isb;
                    -- figure line terms: pline = 1.703 yp + 282 + flow (mod 1024), cl = 728 yp (Q16)
                    v_p := resize(e_yp, 16) + shift_right(resize(e_yp, 16), 1) + shift_right(resize(e_yp, 16), 3)
                           + shift_right(resize(e_yp, 16), 4) + shift_right(resize(e_yp, 16), 6) + 282
                           + signed(resize(c_tfl(15 downto 6), 16));
                    n_pl <= unsigned(v_p(9 downto 0));
                    v_p := shift_left(resize(e_yp, 16), 9) + shift_left(resize(e_yp, 16), 7) + shift_left(resize(e_yp, 16), 6)
                           + shift_left(resize(e_yp, 16), 4) + shift_left(resize(e_yp, 16), 3);
                    n_cl <= unsigned(v_p);
                when 3 =>
                    e_c0 <= r_cs_q;
                    -- side wobble: sin(2 pi yp / 420) = cos at (yp 156 >> 6) - 256
                    v_p := shift_left(resize(e_yp, 16), 7) + shift_left(resize(e_yp, 16), 4) + shift_left(resize(e_yp, 16), 3)
                           + shift_left(resize(e_yp, 16), 2);
                    r_cs_a <= unsigned(v_p(15 downto 6)) - 256;
                when 4 =>
                    e_c1 <= r_cs_q;
                when 5 =>
                    v_d := resize(e_c1, 17) - resize(e_c0, 17);
                    e_dneg <= v_d(16);
                    m_a <= resize(unsigned(abs(v_d)), 34); m_b <= resize(e_fr, 16); m_acc <= (others => '0');
                    e_sv <= r_cs_q;
                when 6 =>                     -- side = 255 + 22 sin >> 15 (virtual px)
                    v_sp := shift_left(resize(e_sv, 21), 4) + shift_left(resize(e_sv, 21), 2) + shift_left(resize(e_sv, 21), 1);
                    v_sd := shift_right(v_sp, 15) + 255;
                    n_sc <= unsigned(v_sd(9 downto 0));
                    n_se <= unsigned(v_sd(9 downto 0)) + 60;
                    n_si <= unsigned(v_sd(9 downto 0)) + 90;
                when 22 =>                    -- wave = c0 + (c1 - c0) fr >> 8, or the zigzag
                    if s_zig = '1' then
                        v_w := e_zt;
                    elsif e_dneg = '1' then
                        v_w := resize(e_c0, 17) - signed(resize(m_acc(23 downto 8), 17));
                    else
                        v_w := resize(e_c0, 17) + signed(resize(m_acc(23 downto 8), 17));
                    end if;
                    e_wave <= v_w;
                when 23 =>
                    e_wneg <= e_wave(16);
                    v_w := abs(e_wave);
                    m_a <= resize(c_b, 34); m_b <= unsigned(v_w(15 downto 0)); m_acc <= (others => '0');
                when 40 =>                    -- g = a +- (b |wave|) >> 15
                    v_bw := m_acc(33 downto 15);
                    if (c_bneg xor e_wneg) = '1' then
                        v_g := signed(resize(c_a, 19)) - signed(v_bw);
                    else
                        v_g := signed(resize(c_a, 19)) + signed(v_bw);
                    end if;
                    e_g <= v_g;
                when 41 =>
                    case to_integer(s_xs) is
                        when 4      => v_du := shift_left(resize(e_g, 22), 2);
                        when 3      => v_du := shift_left(resize(e_g, 22), 1) + resize(e_g, 22);
                        when others => v_du := shift_left(resize(e_g, 22), 1);
                    end case;
                    n_du <= v_du;
                    e_gneg <= e_g(18) xor c_dx0(13);
                    m_a <= resize(unsigned(abs(e_g)), 34); m_b <= resize(unsigned(abs(c_dx0)), 16); m_acc <= (others => '0');
                when 58 =>                    -- u0 = phi0 +- |g| |dx0| (+ half a ring on B rows under Weave)
                    if e_gneg = '1' then
                        v_u := to_unsigned(C_PHI0, 24) - m_acc(23 downto 0);
                    else
                        v_u := to_unsigned(C_PHI0, 24) + m_acc(23 downto 0);
                    end if;
                    if s_weave = '1' and e_isb = '1' then v_u := v_u + x"800000"; end if;
                    n_u0 <= v_u;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the ripple phase DDA (u += g xs per pixel; under Twin it runs back
    -- after the centre pixel, mirroring the field) and the figure's mirrored
    -- counter (period 1920 h = 960 virtual px)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_c : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_u  <= n_u0;
                b_du <= n_du;
                b_c  <= n_c0;
                b_px <= (others => '0');
                b_back <= '0';
                l_isb <= n_isb; l_pl <= n_pl; l_cl <= n_cl; l_dl <= n_dl;
                l_sc <= n_sc; l_se <= n_se; l_si <= n_si;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_back = '1' then b_u <= b_u - unsigned(resize(b_du, 24));
                else b_u <= b_u + unsigned(resize(b_du, 24)); end if;
                if s_twin = '1' and b_px + 1 >= c_nc then b_back <= '1'; end if;
                v_c := resize(b_c, 12) + s_xs;
                if v_c >= 1920 then b_c <= resize(v_c - 1920, 11); else b_c <= v_c(10 downto 0); end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ma : signed(11 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_t18 : unsigned(17 downto 0);
        variable v_m  : unsigned(15 downto 0);
        variable v_s  : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: offset from the figure axis (h)
            p2_mh <= signed(resize(b_c, 12)) - 960;
            p2_u <= b_u;
            q_isb(2) <= l_isb;
            -- 3: mirrored distance (virtual px); source luma
            v_ma := abs(p2_mh);
            p3_m <= unsigned(v_ma(10 downto 1));
            p3_s <= unsigned(s_lb_q);
            p3_u <= p2_u;
            q_isb(3) <= q_isb(2);
            -- 4: the m and source products, first halves (shift-adds); side flags
            v_m := resize(p3_m, 16);
            p4_m1a <= shift_left(v_m, 8) - shift_left(v_m, 5);                    -- m 218 = 256 - 32
            p4_m1b <= shift_left(v_m, 2) + shift_left(v_m, 1);                    --          - (4 + 2)
            p4_m2a <= shift_left(v_m, 9) + shift_left(v_m, 6);                    -- m 582 = 512 + 64
            p4_m2b <= shift_left(v_m, 2) + shift_left(v_m, 1);                    --          + 4 + 2
            p4_mca <= shift_left(v_m, 9) + shift_left(v_m, 5);                    -- m 570 = 512 + 32
            p4_mcb <= shift_left(v_m, 4) + shift_left(v_m, 3) + shift_left(v_m, 1); --        + 16 + 8 + 2
            p4_maa <= resize(p3_m, 11) + resize(p3_m(9 downto 3), 11);           -- m 1.1953 = 1 + 1/8
            p4_mab <= resize(p3_m(9 downto 4), 11) + resize(p3_m(9 downto 7), 11); --          + 1/16 + 1/128
            p4_s3a <= shift_left(resize(p3_s, 10), 1) + resize(p3_s, 10);         -- s 3.5 = 3 s + s / 2
            p4_s7a <= shift_left(resize(p3_s, 15), 6) + shift_left(resize(p3_s, 15), 3);   -- s 75 = 64 + 8
            p4_s7b <= shift_left(resize(p3_s, 15), 1) + resize(p3_s, 15);         --        + 2 + 1
            p4_ue <= shift_left(resize(p3_s, 24), 12) + shift_left(resize(p3_s, 24), 11);  -- s 6144
            p4_s <= p3_s; p4_m <= p3_m; p4_u <= p3_u;
            if p3_m >= l_si then q_out(4) <= '1'; else q_out(4) <= '0'; end if;
            if p3_m < l_se then q_ine(4) <= '1'; else q_ine(4) <= '0'; end if;
            if p3_m >= l_sc then q_msd(4) <= '1'; else q_msd(4) <= '0'; end if;
            q_isb(4) <= q_isb(3);
            -- 5: the products, second halves
            p5_m1 <= p4_m1a - p4_m1b;
            p5_m2 <= p4_m2a + p4_m2b;
            p5_mc <= p4_mca + p4_mcb;
            p5_ma <= p4_maa + p4_mab;
            p5_s35 <= p4_s3a + resize(p4_s(7 downto 1), 10);
            p5_s75 <= p4_s7a + p4_s7b;
            p5_ue <= p4_ue;
            p5_u <= p4_u;
            q_isb(5) <= q_isb(4);
            q_out(5) <= q_out(4); q_ine(5) <= q_ine(4); q_msd(5) <= q_msd(4);
            -- 6: etched ripple phase; triangle 1, chevron and crown phases; partial index; veil distance
            p6_uf <= p5_u + p5_ue;
            p6_t1 <= f_tri16(p5_m1);
            p6_ph2 <= l_cl - p5_m2;
            p6_fc <= p5_mc - 35625;
            p6_pp <= l_pl - p5_ma(9 downto 0);
            p6_s35 <= p5_s35;
            p6_dd <= resize(l_dl, 13) + signed(resize(p5_s75(14 downto 8), 13));
            q_isb(6) <= q_isb(5);
            q_out(6) <= q_out(5); q_ine(6) <= q_ine(5); q_msd(6) <= q_msd(5);
            -- 7: window distances, fire address, triangle 2, crown address
            p7_da <= signed(p6_uf(23 downto 8) - to_unsigned(C_CA, 16));
            p7_db <= signed(p6_uf(23 downto 8) - to_unsigned(C_CB, 16));
            p7_fa <= q_isb(6) & p6_uf(23 downto 16);
            p7_t2 <= f_tri16(p6_ph2);
            v_t18 := shift_left(resize(p6_t1, 18), 1) + resize(p6_t1, 18);    -- t1 3 >> 10 (0 .. 191)
            p7_t1s <= resize(v_t18(17 downto 10), 10);
            r_cr_a <= p6_fc(15 downto 8);
            p7_pp <= p6_pp + p6_s35;
            p7_dd <= p6_dd;
            q_isb(7) <= q_isb(6);
            q_out(7) <= q_out(6); q_ine(7) <= q_ine(6); q_msd(7) <= q_msd(6);
            -- 8: |distances|; palette index (fire / crown ROM data)
            p8_aa <= unsigned(abs(p7_da));
            p8_ab <= unsigned(abs(p7_db));
            p8_p <= p7_pp + p7_t1s + resize(p7_t2(15 downto 9), 10);
            p8_dd <= p7_dd;
            p8_fa <= p7_fa;
            q_isb(8) <= q_isb(7);
            q_out(8) <= q_out(7); q_ine(8) <= q_ine(7); q_msd(8) <= q_msd(7);
            -- 9: window test; fire / crown fabric copies; blue address
            if q_isb(8) = '1' then
                if p8_ab < c_hwb then p9_key <= '1'; else p9_key <= '0'; end if;
            else
                if p8_aa < c_hwa then p9_key <= '1'; else p9_key <= '0'; end if;
            end if;
            p9_fa <= p8_fa;
            p9_cs <= r_cr_q;
            p9_ba <= q_out(8) & p8_p(9 downto 3);
            p9_dd <= p8_dd;
            q_ine(9) <= q_ine(8); q_msd(9) <= q_msd(8);
            -- 10: crown depth product; fire and blue ROM addresses (issued late: an address is cheaper to delay)
            p10_pr <= c_amp * p9_cs;
            p10_dd <= p9_dd;
            r_fi_a <= p9_fa;
            r_bl_a <= p9_ba;
            p10_key <= p9_key;
            q_ine(10) <= q_ine(9); q_msd(10) <= q_msd(9);
            -- 11: veil distance (fire / blue ROM data)
            p11_dd <= p10_dd - signed(resize(p10_pr(16 downto 8), 13));
            p11_key <= p10_key;
            q_ine(11) <= q_ine(10); q_msd(11) <= q_msd(10);
            -- 12: veil flags
            if p11_dd > 0 and q_ine(11) = '1' then p12_cur <= '1'; else p12_cur <= '0'; end if;
            if p11_dd <= 11 or q_msd(11) = '1' then p12_outl <= '1'; else p12_outl <= '0'; end if;
            p12_bl <= r_bl_q; p12_fi <= r_fi_q; p12_key <= p11_key;
            -- 13: compose (neutral black outside the generated avid)
            if s_gav_sr(10) = '0' then
                s_y13 <= to_unsigned(64, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            elsif p12_cur = '1' and p12_outl = '1' then
                s_y13 <= to_unsigned(C_PB_YEL_Y, 10); s_u13 <= to_unsigned(C_PB_YEL_U, 10); s_v13 <= to_unsigned(C_PB_YEL_V, 10);
            elsif p12_cur = '1' or p12_key = '1' then
                s_y13 <= unsigned(p12_bl(31 downto 22)); s_u13 <= unsigned(p12_bl(21 downto 12)); s_v13 <= unsigned(p12_bl(11 downto 2));
            else
                s_y13 <= unsigned(p12_fi(31 downto 22)); s_u13 <= unsigned(p12_fi(21 downto 12)); s_v13 <= unsigned(p12_fi(11 downto 2));
            end if;
            -- 14: Ice swaps Cb and Cr
            s_y14 <= s_y13;
            if s_ice = '1' then s_u14 <= s_v13; s_v14 <= s_u13; else s_u14 <= s_u13; s_v14 <= s_v13; end if;
            -- 15: copy (box A input)
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix), luma 13/16 and the
    -- blanking gate.  The running 8-sums snap back to neutral once 8 blank
    -- samples are in (the per-line resync).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 16 sums, stage 17 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bay <= s_bay + s_y15 - unsigned(r_aq2(31 downto 22));
                s_bau <= s_bau + s_u15 - unsigned(r_aq2(21 downto 12));
                s_bav <= s_bav + s_v15 - unsigned(r_aq2(11 downto 2));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid
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
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(512 * 8, 13); s_bbv <= to_unsigned(512 * 8, 13);
            else
                s_bby <= s_bby + s_y17 - unsigned(r_bq2(31 downto 22));
                s_bbu <= s_bbu + s_u17 - unsigned(r_bq2(21 downto 12));
                s_bbv <= s_bbv + s_v17 - unsigned(r_bq2(11 downto 2));
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
    data_out.hsync_n <= r_sq2(0);
    data_out.vsync_n <= r_sq2(1);
    data_out.field_n <= r_sq2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture pebble;
