-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Stairwell v0.1.0 -- Onevolt 1: LZX "Your First Patch" taken through Stairs.
--
-- The rack (LZX Gen3) and the patch:
--   Scrolls ramp 1 (H) and ramp 2 (V), Mirror waveform, Orbital motion -> DSG3 Shape 1 H in, V in
--   DSG3 Shape 1: Fold on both inputs (S9: Fold 1 / Fold 2), Curve on both (K4, exp .. lin .. log)
--   Shape 1 Average (S7: or Absolute) -> Shape 2 H in ; Shape 1 Minimum (S8: or Maximum) -> Shape 2 V in
--   Shape 2 Average -> Stairs G in (the base shape) ; TBC2 luma (S11: a DWO3 lattice) -> Stairs R in (K6)
--   Stairs: Steps (P12), Phase (K3) + a slow LFO on the Phase CV (S10: Ribbons in its place, Span <- P12,
--   Center <- K3) ; three Stairs outputs (Ribbons D0 D1 D2) -> ESG3 R, G, B (K5: which three, and inverts)
--
--   K1  Width      ramp 1 gain: figure 3.2x .. 0.4x the screen (Mirror: the figure repeats below 1x)
--   K2  Height     ramp 2 gain, the same law
--   K3  Phase      Stairs starting phase (Ribbons Center): low = bands wall to wall .. high = a small core
--   K4  Curve      DSG3 Curve, continuous: exponential (t^2: pinched stars) .. linear .. logarithmic (2t - t^2)
--   K5  Colour     eight cable arrangements of the Stairs outputs into ESG3 R, G, B (with inverts), blended
--   K6  Camera     the attenuator on Stairs' R input, 0 .. 100 %
--   S7  H In       Shape 2 H input: Shape 1 Average / Absolute
--   S8  V In       Shape 2 V input: Shape 1 Minimum / Maximum
--   S9  Fold       DSG3 Fold 1 / Fold 2
--   S10 Folder     Stairs (six doublers) / Ribbons (3-bit digitiser)
--   S11 Source     Video / Osc
--   P12 STEPS      Stairs Steps (pre-fold gain, 0.25 .. 8) / Ribbons Span (2 .. 0.125)
--
-- Engine: vblank-zone sequencer (one 32-bit serial multiplier, 10 slots, shared knob glide, shift-add knob laws),
-- then a table writer (a radix-16 serial multiplier, 13 clocks an entry) that writes the two separable 1-D
-- functions of the patch, Steps x Curve(Fold(Scrolls ramp)), over x (8 virtual px an entry) and over y (8 virtual
-- lines an entry) into one block RAM; a per-line engine interpolates the y function in hblank and takes the band
-- density from its slope; the pixel path interpolates the x function, combines the two as Shape 2's Average of
-- Shape 1's outputs, adds the camera, subtracts the phase and takes the six Stairs taps (or the Ribbons bits),
-- each faded by the band density, into the ESG3 channel ROMs.
-- Generated avid; the shell's output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the
-- per-line resync, luma 13/16) before the blanking gate.  Virtual 1080-line coordinates.  Colours in the hardware
-- convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.stairwell_pkg.all;
use work.all;

architecture stairwell of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

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
    -- the Osc lattice's line step: (ys x 121) >> 8
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(537, 10);
            when 4          => return to_unsigned(272, 10);
            when 8          => return to_unsigned(453, 10);
            when 12         => return to_unsigned(226, 10);
            when 5 | 6 | 14 => return to_unsigned(181, 10);
            when 1 | 2 | 10 => return to_unsigned(242, 10);
            when others     => return to_unsigned(121, 10);
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
    -- quarter-octave log of a slope: 4 x (leading one) + the next two bits (0 for 0)
    function f_lq(g : unsigned(17 downto 0)) return unsigned is
        variable v_p : integer range 0 to 17;
        variable v_q : unsigned(1 downto 0);
    begin
        if g = 0 then return to_unsigned(0, 7); end if;
        v_p := 0;
        for b in 0 to 17 loop
            if g(b) = '1' then v_p := b; end if;
        end loop;
        if v_p < 2 then return shift_left(to_unsigned(v_p, 7), 2); end if;
        v_q := g(v_p - 1) & g(v_p - 2);
        return shift_left(to_unsigned(v_p, 7), 2) + resize(v_q, 7);
    end function;

    -- luma cell store: per 16-pixel cell its 12-bit vertical-slew state (read a cell ahead of the write)
    type t_lb is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(7 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav, s_f1, s_f2 : std_logic := '0';
    signal s_lc    : unsigned(3 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(7 downto 0) := (others => '0');
    signal s_acc, s_sum : unsigned(11 downto 0) := (others => '0');
    signal s_cur, s_old : std_logic_vector(11 downto 0) := (others => '0');
    signal s_n1 : unsigned(11 downto 0) := (others => '0');
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_h1 : unsigned(15 downto 0) := (others => '0');
    signal s_cpl, s_cph : unsigned(16 downto 0) := (others => '0');
    signal s_cam   : unsigned(19 downto 0) := (others => '0');
    signal s_cmo   : signed(23 downto 0) := (others => '0');               -- camera - Steps x Phase (Q14)

    -- the patch table: x entries 0 .. 255 (8 virtual px each), y entries 256 .. 511 (8 virtual lines each);
    -- value = Steps x Curve(Fold(ramp)) in Q14 (20 bits), written in the zone
    type t_tab is array (0 to 511) of std_logic_vector(31 downto 0);
    signal tab : t_tab := (others => (others => '0'));
    attribute no_rw_check of tab : signal is true;
    signal r_t_a  : unsigned(8 downto 0) := (others => '0');
    signal r_t_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal wt_we  : std_logic := '0';
    signal wt_a   : unsigned(8 downto 0) := (others => '0');
    signal wt_d   : std_logic_vector(31 downto 0) := (others => '0');
    -- ESG3 channel ROMs (registered address and data)
    signal r_ra, r_ga, r_ba : unsigned(7 downto 0) := (others => '0');
    signal r_rq, r_gq, r_bq : std_logic_vector(31 downto 0) := (others => '0');

    -- output filter stores (as the shell)
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da, db : t_dl := (others => (others => '0'));
    attribute no_rw_check of da : signal is true;
    attribute no_rw_check of db : signal is true;
    signal s_dp : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_dr : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_qa, s_qb, s_fa, s_fb : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(438 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(600 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_hin, s_vin, s_fold, s_rib, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal z_phx, z_phy, z_phl : unsigned(15 downto 0) := to_unsigned(16384, 16);   -- Scrolls x / y, phase LFO (start mid-swing)
    signal z_e1, z_e2 : unsigned(11 downto 0) := (others => '0');
    signal z_ep  : unsigned(12 downto 0) := (others => '0');
    signal z_m1, z_m2 : unsigned(11 downto 0) := (others => '0');
    signal z_k4c : signed(11 downto 0) := (others => '0');
    signal z_kq  : signed(11 downto 0) := (others => '0');
    signal z_ak  : unsigned(11 downto 0) := (others => '0');               -- |k|, |Phase| and their signs (operands)
    signal z_aphi : unsigned(13 downto 0) := (others => '0');
    signal z_ks, z_phs : std_logic := '0';
    signal z_q1  : unsigned(12 downto 0) := (others => '0');               -- the density sum, one add a clock
    signal z_l1, z_adj : signed(9 downto 0) := (others => '0');
    signal z_pa, z_pb : signed(13 downto 0) := (others => '0');
    signal z_pos : unsigned(12 downto 0) := (others => '0');
    signal z_f5  : unsigned(4 downto 0) := (others => '0');
    signal z_ar  : unsigned(2 downto 0) := (others => '0');
    signal z_so, z_co, z_sl : unsigned(14 downto 0) := (others => '0');       -- the triangles, 0 .. 32767
    signal z_cx16, z_cy16 : unsigned(14 downto 0) := to_unsigned(15360, 15);
    signal z_lfo : signed(12 downto 0) := (others => '0');
    signal z_phi : signed(13 downto 0) := (others => '0');
    signal c_dx, c_dy : unsigned(26 downto 0) := (others => '0');
    signal c_bx0 : unsigned(15 downto 0) := to_unsigned(2048, 16);          -- beam start, 1/16 virtual px
    signal c_vy0 : unsigned(19 downto 0) := to_unsigned(17408, 20);          -- line start, Q8 virtual lines
    signal s_x0q : unsigned(15 downto 0) := (others => '0');
    signal c_s   : unsigned(17 downto 0) := to_unsigned(6144, 18);           -- Steps (Q12)
    signal c_b   : signed(20 downto 0) := (others => '0');                   -- S k (Q12)
    signal c_a   : signed(20 downto 0) := (others => '0');                   -- S (1 + k) (Q12)
    signal c_off : signed(23 downto 0) := (others => '0');                   -- phase offset (Q14)
    signal c_cg  : unsigned(17 downto 0) := (others => '0');                 -- camera gain
    signal c_f5  : unsigned(4 downto 0) := (others => '0');                  -- K5 blend 0 .. 16
    signal c_arA, c_arB : std_logic_vector(11 downto 0) := (others => '0');  -- K5 arrangements
    type t_ce is array (0 to 2) of signed(7 downto 0);
    signal c_ce  : t_ce := (others => (others => '0'));                       -- fade offsets of the R, G, B cables
    type t_src is array (0 to 2) of std_logic_vector(3 downto 0);
    signal c_src : t_src := (others => (others => '0'));                     -- the Stairs output on each cable {inv, tap}
    type t_g3 is array (0 to 2) of unsigned(4 downto 0);
    signal c_g   : t_g3 := (others => to_unsigned(16, 5));                   -- each cable's ESG3 gain, 0 .. 16
    type t_lv is array (0 to 2) of unsigned(2 downto 0);
    signal c_lv  : t_lv := (others => (others => '0'));                       -- fade levels of the R, G, B cables
    signal c_lq  : signed(8 downto 0) := (others => '0');                    -- band density, quarter octaves
    signal s_lqo : signed(4 downto 0) := (others => '0');                    -- per-mode offset (field line / 8 lines)


    -- field serial multiply: acc = +- a x b (mod 2^32)
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(17 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_ng  : std_logic := '0';
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

    -- table writer
    signal w_go, w_run, w_y, w_dn, w_done : std_logic := '0';
    signal w_c   : unsigned(3 downto 0) := (others => '0');
    signal w_i   : unsigned(8 downto 0) := (others => '0');
    signal w_acc : unsigned(31 downto 0) := (others => '0');
    signal w_d   : unsigned(26 downto 0) := (others => '0');
    signal w_t   : unsigned(15 downto 0) := (others => '0');
    signal w_ma  : signed(21 downto 0) := (others => '0');
    signal w_pp  : signed(25 downto 0) := (others => '0');
    signal w_pa  : signed(37 downto 0) := (others => '0');
    signal w_lo, w_hi : unsigned(8 downto 0) := (others => '0');
    signal w_dlx, w_dly : unsigned(15 downto 0) := (others => '0');          -- the fold's flat top / bottom (t), x / y

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (the y function, interpolated in hblank, and its band density)
    signal l_vy  : unsigned(19 downto 0) := (others => '0');
    signal l_c   : unsigned(3 downto 0) := (others => '0');
    signal l_run, l_kz, l_kl, l_zon, l_tz : std_logic := '0';
    signal p2_lj, p3_lj, p4_lj, p5_lj, p6_lj, p7_lj : std_logic := '0';
    signal l_tvn, l_tv : unsigned(19 downto 0) := (others => '0');
    signal c_tx0 : unsigned(19 downto 0) := (others => '0');
    signal s_gq4 : std_logic := '0';

    -- beam
    signal b_vx  : unsigned(15 downto 0) := (others => '0');
    signal s_gst : std_logic_vector(3 downto 0) := (others => '0');   -- s_gst(k) = g_start delayed k + 1

    -- pixel path (pN_ = registered for stage N)
    signal p2_i, p3_i, p4_i : unsigned(8 downto 0) := (others => '0');
    signal p2_f, p3_f, p4_f, p5_f : unsigned(3 downto 0) := (others => '0');
    signal p4_tn, p4_tc, p5_tc, p6_tc : unsigned(19 downto 0) := (others => '0');
    signal p5_d  : signed(20 downto 0) := (others => '0');
    signal p6_q  : signed(24 downto 0) := (others => '0');
    signal p7_th, p8_th, p7_tj : unsigned(19 downto 0) := (others => '0');
    signal p8_dp : signed(20 downto 0) := (others => '0');
    signal p9_mx, p9_mn : unsigned(19 downto 0) := (others => '0');
    signal p9_ad : unsigned(19 downto 0) := (others => '0');
    signal p10_x0 : signed(21 downto 0) := (others => '0');
    signal p11_x  : signed(24 downto 0) := (others => '0');
    type t_v3 is array (0 to 2) of unsigned(7 downto 0);
    type t_s3 is array (0 to 2) of signed(13 downto 0);
    signal p12_t  : t_v3 := (others => (others => '0'));
    signal p12_fe, p13_fe : std_logic := '0';
    signal p13_q  : t_s3 := (others => (others => '0'));
    signal p14_g1, p14_b1, p14_b2 : unsigned(7 downto 0) := (others => '0');
    signal p16_r, p16_g, p16_b : std_logic_vector(31 downto 0) := (others => '0');
    signal p17_y : unsigned(11 downto 0) := (others => '0');
    signal p17_u, p17_v : signed(12 downto 0) := (others => '0');
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KU : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KV : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    -- the patch table: written only in the zone (the pixel path's reads there are unused)
    p_tab : process(clk)
    begin
        if rising_edge(clk) then
            r_t_q <= tab(to_integer(r_t_a));
            if wt_we = '1' then tab(to_integer(wt_a)) <= wt_d; end if;
        end if;
    end process p_tab;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_rq <= C_SW_R(to_integer(r_ra));
            r_gq <= C_SW_G(to_integer(r_ga));
            r_bq <= C_SW_B(to_integer(r_ba));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_t  : unsigned(15 downto 0);
        variable v_n  : integer range 0 to 7;
        variable v_s  : signed(13 downto 0);
        variable v_w  : unsigned(12 downto 0);
        variable v_x  : std_logic_vector(3 downto 0);
        variable v_k  : unsigned(11 downto 0);
        variable v_q  : unsigned(12 downto 0);
        variable v_l  : signed(9 downto 0);
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
            elsif s_zc /= 4095 then
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_hin  <= registers_in(6)(0);
                s_vin  <= registers_in(6)(1);
                s_fold <= registers_in(6)(2);
                s_rib  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                z_phx  <= z_phx + 47;            -- Scrolls Ping Pong: x 23 s
                z_phy  <= z_phy + 64;            --                    y 17 s
                z_phl  <= z_phl + 121;           -- LFO on Stairs Phase (a triangle): 9 s
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); s_x0q <= to_unsigned(3840, 16);
                    when 3      => s_oxs <= to_unsigned(153, 9); s_x0q <= to_unsigned(0, 16);
                    when others => s_oxs <= to_unsigned(102, 9); s_x0q <= to_unsigned(0, 16);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);
            case to_integer(unsigned(s_tid)) is
                when 0          => s_lqo <= to_signed(-3, 5);
                when 4          => s_lqo <= to_signed(-7, 5);
                when 8          => s_lqo <= to_signed(-4, 5);
                when 12         => s_lqo <= to_signed(-8, 5);
                when 5 | 6 | 14 => s_lqo <= to_signed(-10, 5);
                when 1 | 2 | 10 => s_lqo <= to_signed(-8, 5);
                when others     => s_lqo <= to_signed(-12, 5);
            end case;

            -- shared glide (1/8 of the gap per field, +-2 deadband; +-4 on the camera channel): four clocks
            -- per channel, run from the zone start by its own phase / channel counters
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
                        if s_gd > 32 or s_gd < -32 or (z_gc /= 5 and (s_gd > 16 or s_gd < -16)) then
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

            -- serial multiply slots: slot k loads at zc = 48 + 24 k, 16 iterations, result stable from load + 17
            -- (stored at load + 18 = 66 + 24 k).  The slot number is decoded at 46 + 24 k and the operands and the
            -- sign are pre-muxed at 47 + 24 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc >= 46 and s_zc <= 94 then
                case to_integer(s_zc) is
                    when 46  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 70  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 94  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_ng <= '0';
                case to_integer(z_sel) is
                    when 0  => z_fa <= c_s; z_ng <= z_ks;  z_fb <= resize(z_ak, 16);                                                    -- S k
                    when 1  => z_fa <= c_s; z_ng <= z_phs; z_fb <= resize(z_aphi, 16);                                                  -- S phi
                    when others => z_fa <= c_s; z_fb <= resize(s_g6(12 downto 3), 16);                                                  -- S K6
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                if z_ng = '1' then f_a <= unsigned(-signed(resize(z_fa, 32))); else f_a <= resize(z_fa, 32); end if;
                f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- knob decodes (after the glide): exponential laws are piecewise linear per octave
                    when 34 =>
                        z_e1 <= resize(s_g1(12 downto 3), 12) + shift_left(resize(s_g1(12 downto 3), 12), 1) + 512;   -- 3 octaves
                        z_e2 <= resize(s_g2(12 downto 3), 12) + shift_left(resize(s_g2(12 downto 3), 12), 1) + 736;
                        if s_rib = '1' then
                            z_ep <= shift_left(resize(s_gp(12 downto 3), 13), 2);                                   -- 4 octaves
                        else
                            z_ep <= shift_left(resize(s_gp(12 downto 3), 13), 2) + resize(s_gp(12 downto 3), 13);    -- 5 octaves
                        end if;
                        z_k4c <= shift_left(signed(resize(s_g4(12 downto 3), 12)) - 512, 1);
                        z_pa <= signed(resize(shift_left(resize(s_g3(12 downto 3), 13), 2) + resize(s_g3(12 downto 6), 13), 14));
                        z_pb <= signed(resize(resize(s_g3(12 downto 7), 13) + resize(s_g3(12 downto 10), 13), 14)) - 1229;
                        z_pos <= shift_left(resize(s_g5(12 downto 3), 13), 3) - resize(s_g5(12 downto 3), 13);
                        -- triangle LFOs (0 .. 32767; the offsets below take 16384 as their centre)
                        if z_phx(15) = '1' then z_so <= not z_phx(14 downto 0); else z_so <= z_phx(14 downto 0); end if;
                        if z_phy(15) = '1' then z_co <= not z_phy(14 downto 0); else z_co <= z_phy(14 downto 0); end if;
                        if z_phl(15) = '1' then z_sl <= not z_phl(14 downto 0); else z_sl <= z_phl(14 downto 0); end if;
                    when 35 =>
                        z_kq <= z_k4c;
                        z_ar <= z_pos(12 downto 10);
                        if z_pos(9 downto 8) = "00" then z_f5 <= (others => '0');
                        elsif z_pos(9 downto 8) = "11" then z_f5 <= to_unsigned(16, 5);
                        else v_w := resize(z_pos(9 downto 0), 13) - 256; z_f5 <= resize(v_w(9 downto 5), 5); end if;
                        z_m1 <= to_unsigned(2048, 12) - resize(z_e1(9 downto 0), 12);
                        z_m2 <= to_unsigned(2048, 12) - resize(z_e2(9 downto 0), 12);
                        -- Steps (Stairs 0.25 .. 8) / Ribbons 4 / Span (2 .. 32), Q12
                        v_n := to_integer(z_ep(12 downto 10));
                        if s_rib = '1' then v_n := v_n + 3; end if;
                        c_s <= shift_left(resize(to_unsigned(1024, 11) + resize(z_ep(9 downto 0), 11), 18), v_n);
                    when 36 =>
                        c_arA <= C_SW_ARR(to_integer(z_ar));
                        if z_ar = 7 then c_arB <= C_SW_ARR(7); else c_arB <= C_SW_ARR(to_integer(z_ar) + 1); end if;
                        c_f5 <= z_f5;
                        if z_kq(11) = '1' then z_ak <= unsigned(-z_kq); else z_ak <= unsigned(z_kq); end if;
                        z_ks <= z_kq(11);
                        -- ramp phase steps per table entry (Q32): m 2^(15 - n) (ramp 1) and m 2^(16 - n) (ramp 2)
                        case to_integer(z_e1(11 downto 10)) is
                            when 0 => c_dx <= shift_left(resize(z_m1, 27), 15);
                            when 1 => c_dx <= shift_left(resize(z_m1, 27), 14);
                            when 2 => c_dx <= shift_left(resize(z_m1, 27), 13);
                            when others => c_dx <= shift_left(resize(z_m1, 27), 12);
                        end case;
                        case to_integer(z_e2(11 downto 10)) is
                            when 0 => c_dy <= shift_left(resize(z_m2, 27), 16);
                            when 1 => c_dy <= shift_left(resize(z_m2, 27), 15);
                            when 2 => c_dy <= shift_left(resize(z_m2, 27), 14);
                            when others => c_dy <= shift_left(resize(z_m2, 27), 13);
                        end case;
                        -- Scrolls: the Ping Pong offsets (72 x 40 virtual px) and the LFO on the phase (0.094)
                        -- (all on the unsigned triangles: a sum of shifts of one signed value puts its sign net on two
                        -- inputs of a LUT, which router1 can hang on)
                        z_cx16 <= to_unsigned(14208, 15) + resize(shift_right(z_so, 4), 15) + resize(shift_right(z_so, 7), 15);
                        z_cy16 <= to_unsigned(8000, 15) + resize(shift_right(z_co, 5), 15) + resize(shift_right(z_co, 7), 15);
                        z_lfo  <= signed(resize(shift_right(z_sl, 6), 13) + resize(shift_right(z_sl, 7), 13)) - 384;
                    when 37 =>
                        -- K5: per cable the source of the nearer arrangement; a cable that differs between the two
                        -- dims to mid-grey (ESG3 contrast) and comes back as the new source; fade offsets by the source
                        -- (Stairs: 4 k - 48 with k = tap + 1; Ribbons: by bit): full above a band period of 8 lines,
                        -- flat at 4 or less
                        for c in 0 to 2 loop
                            if c_f5(4 downto 3) = "00" then v_x := c_arA(11 - 4 * c downto 8 - 4 * c);
                            else v_x := c_arB(11 - 4 * c downto 8 - 4 * c); end if;
                            c_src(c) <= v_x;
                            if c_arA(11 - 4 * c downto 8 - 4 * c) = c_arB(11 - 4 * c downto 8 - 4 * c) then
                                c_g(c) <= to_unsigned(16, 5);
                            elsif c_f5(4 downto 3) = "00" then
                                c_g(c) <= to_unsigned(16, 5) - shift_left(c_f5, 1);
                            else
                                c_g(c) <= shift_left(c_f5, 1) - 16;
                            end if;
                            if s_rib = '0' then
                                c_ce(c) <= shift_left(signed(resize(unsigned(v_x(2 downto 0)), 8)), 2) - 44;
                            else
                                case to_integer(unsigned(v_x(2 downto 0))) is
                                    when 0 | 3  => c_ce(c) <= to_signed(-56, 8);
                                    when 1 | 4  => c_ce(c) <= to_signed(-52, 8);
                                    when others => c_ce(c) <= to_signed(-48, 8);
                                end case;
                            end if;
                        end loop;
                    when 38 =>
                        c_bx0 <= s_x0q + 17408 - resize(z_cx16, 16);   -- the Scrolls offsets move the beam's origin
                        c_vy0 <= to_unsigned(155648, 20) - shift_left(resize(z_cy16, 20), 4);
                        z_phi <= z_pa + z_pb + resize(z_lfo, 14);
                    when 39 =>
                        if s_rib = '1' then z_phi <= z_phi + 1434; end if;
                        -- band density per field (quarter octaves of the steepest vertical slope per field line):
                        -- log2 Steps + log2 ramp-2 gain + log2 (1 + |Curve|) + Fold, each piecewise linear; one add a clock
                        z_q1 <= resize(z_ep(9 downto 0), 13) + resize(z_m2, 13);
                        v_l := resize(s_lqo, 10) - 4;              -- (q - 1024) >> 8 = (q >> 8) - 4
                        if s_fold = '1' then v_l := v_l + 4; end if;
                        if s_rib = '1' then v_l := v_l + 12; end if;
                        if s_hin = '0' then v_l := v_l - 2; end if;
                        z_adj <= v_l;
                    when 40 =>
                        if z_phi(13) = '1' then z_aphi <= unsigned(-z_phi); else z_aphi <= unsigned(z_phi); end if;
                        z_phs <= z_phi(13);
                        z_q1 <= z_q1 + resize(z_ak, 13);
                        z_l1 <= shift_left(to_signed(7, 10) + signed(resize(z_ep(12 downto 10), 10))
                                           - signed(resize(z_e2(11 downto 10), 10)), 2) + z_adj;
                    when 41 =>
                        c_lq <= resize(z_l1 + signed(resize(z_q1(12 downto 8), 10)), 9);
                    when 43 =>
                        for c in 0 to 2 loop
                            v_l := resize(c_lq, 10) + resize(c_ce(c), 10);
                            if v_l <= 0 then c_lv(c) <= "000";
                            elsif v_l >= 4 then c_lv(c) <= "100";
                            else c_lv(c) <= unsigned(v_l(2 downto 0)); end if;
                        end loop;
                    -- slot results (stored at 66 + 24 k)
                    when 66 => c_b <= resize(shift_right(signed(f_acc), 10), 21);
                    when 67 => c_a <= signed(resize(c_s, 21)) + c_b;
                    when 90 =>
                        if s_rib = '1' then
                            c_off <= resize(shift_right(signed(f_acc), 10), 24) - 65536;
                        else
                            c_off <= resize(shift_right(signed(f_acc), 10), 24);
                        end if;
                    when 114 => c_cg <= resize(shift_right(f_acc + shift_right(f_acc, 2), 11), 18);
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
    -- Table writer (from zone clock 300, 13 clocks an entry): the ramp phase (Q32, wrapping = Scrolls Mirror),
    -- DSG3 Fold (t = triangle of the phase, x2 or x4), Curve and Steps as X = t (A - B t) with A = S (1 + k),
    -- B = S k, on one radix-16 serial multiplier (Horner, a 4-bit partial product each clock).  x entries
    -- i_lo .. i_hi (the visible window), then y entries 0 .. 136.
    ---------------------------------------------------------------------------
    p_wr : process(clk)
        variable v_v  : unsigned(15 downto 0);
        variable v_nb : unsigned(3 downto 0);
        variable v_g0, v_g1, v_g2, v_g3 : signed(25 downto 0);
        variable v_x  : signed(23 downto 0);
        variable v_xc : unsigned(19 downto 0);
        variable v_st, v_tt, v_u, v_dl : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            w_go <= '0';
            if s_zone = '1' and s_zc = 120 then w_go <= '1'; end if;
            wt_we <= '0';
            w_done <= '0';
            -- the rectifier's knee: t is held within 3/4 of one entry's step of its peak and its floor, so every
            -- apex and valley of the fold is about 1.5 entries flat (no one-row crest line)
            if s_fold = '1' then v_st := resize(shift_right(c_dx, 14), 16); else v_st := resize(shift_right(c_dx, 15), 16); end if;
            w_dlx <= shift_right(v_st, 1) + shift_right(v_st, 2);
            if s_fold = '1' then v_st := resize(shift_right(c_dy, 14), 16); else v_st := resize(shift_right(c_dy, 15), 16); end if;
            w_dly <= shift_right(v_st, 1) + shift_right(v_st, 2);
            if w_y = '1' then w_lo <= to_unsigned(3, 9); w_hi <= to_unsigned(149, 9);
            elsif s_xs = 4 then w_lo <= to_unsigned(37, 9); w_hi <= to_unsigned(236, 9);
            else w_lo <= to_unsigned(7, 9); w_hi <= to_unsigned(266, 9); end if;
            if w_go = '1' then
                w_run <= '1'; w_y <= '0'; w_dn <= '0'; w_i <= to_unsigned(136, 9); w_c <= (others => '0');
                w_acc <= x"80000000"; w_d <= c_dx;
            elsif w_run = '1' then
                -- the 4-bit partial product of this clock (nibble by the phase)
                case to_integer(w_c) is
                    when 1 | 7  => v_nb := w_t(15 downto 12);
                    when 2 | 8  => v_nb := w_t(11 downto 8);
                    when 3 | 9  => v_nb := w_t(7 downto 4);
                    when others => v_nb := w_t(3 downto 0);
                end case;
                if v_nb(0) = '1' then v_g0 := resize(w_ma, 26); else v_g0 := (others => '0'); end if;
                if v_nb(1) = '1' then v_g1 := shift_left(resize(w_ma, 26), 1); else v_g1 := (others => '0'); end if;
                if v_nb(2) = '1' then v_g2 := shift_left(resize(w_ma, 26), 2); else v_g2 := (others => '0'); end if;
                if v_nb(3) = '1' then v_g3 := shift_left(resize(w_ma, 26), 3); else v_g3 := (others => '0'); end if;
                w_pp <= (v_g0 + v_g1) + (v_g2 + v_g3);
                case to_integer(w_c) is
                    when 0 =>
                        v_v := w_acc(31 downto 16);
                        if s_fold = '0' then
                            if v_v(15) = '1' then v_tt := (not v_v(14 downto 0)) & '0'; else v_tt := v_v(14 downto 0) & '0'; end if;
                        else
                            if v_v(14) = '1' then v_tt := (not v_v(13 downto 0)) & "00"; else v_tt := v_v(13 downto 0) & "00"; end if;
                        end if;
                        if v_tt(15) = '1' then v_u := not v_tt; else v_u := v_tt; end if;     -- distance to the nearer end
                        if w_y = '1' then v_dl := w_dly; else v_dl := w_dlx; end if;
                        if v_u >= v_dl then w_t <= v_tt;
                        elsif v_tt(15) = '1' then w_t <= not v_dl;
                        else w_t <= v_dl; end if;
                        w_ma <= resize(c_b, 22);
                    when 2 | 8 => w_pa <= resize(w_pp, 38);
                    when 3 | 4 | 5 | 9 | 10 | 11 => w_pa <= shift_left(w_pa, 4) + resize(w_pp, 38);
                    when 6 => w_ma <= resize(c_a, 22) - resize(shift_right(w_pa, 16), 22);
                    when 12 =>
                        v_x := resize(shift_right(w_pa, 14), 24);
                        wt_we <= '1';
                        if w_y = '1' then wt_a <= w_i + 320; else wt_a <= w_i; end if;
                        if v_x(23) = '1' then
                            v_xc := (others => '0');
                        elsif v_x(22 downto 20) /= "000" then
                            v_xc := (others => '1');
                        else
                            v_xc := unsigned(v_x(19 downto 0));
                        end if;
                        wt_d <= x"000" & std_logic_vector(v_xc);
                        -- walk out from the centre entry (phase 0.5): up to the window's end, then down from the centre
                        if w_dn = '0' and w_i = w_hi then
                            w_dn <= '1';
                            if w_y = '1' then w_i <= to_unsigned(75, 9); w_acc <= x"80000000" - resize(c_dy, 32);
                            else w_i <= to_unsigned(135, 9); w_acc <= x"80000000" - resize(c_dx, 32); end if;
                        elsif w_dn = '1' and w_i = w_lo then
                            if w_y = '1' then
                                w_run <= '0'; w_done <= '1';
                            else
                                w_y <= '1'; w_dn <= '0'; w_i <= to_unsigned(76, 9); w_acc <= x"80000000"; w_d <= c_dy;
                            end if;
                        elsif w_dn = '1' then
                            w_i <= w_i - 1; w_acc <= w_acc - w_d;
                        else
                            w_i <= w_i + 1; w_acc <= w_acc + w_d;
                        end if;
                    when others => null;
                end case;
                if w_c = 12 then w_c <= (others => '0'); else w_c <= w_c + 1; end if;
            end if;
        end if;
    end process p_wr;

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
    -- Source side (TBC2 luma or the DWO3 lattice): ((Y >> 2) - 16), summed over
    -- 16-pixel cells, each cell slewed down the screen by a one-pole (1/64 per
    -- line; the state lives in the cell store), then slewed across by a one-pole
    -- (1/16 per pixel) and scaled by the camera gain (Osc at twice the camera's);
    -- less Steps x Phase.  All the filters decay, so none needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_e : signed(16 downto 0);
        variable v_p : unsigned(25 downto 0);
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

            -- 1: sample
            if s_src = '1' then
                s_lin <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lin <= v_l - 16; else s_lin <= (others => '0'); end if;
            end if;
            s_lav   <= data_in.avid;
            s_lc    <= s_wcol(3 downto 0);
            s_lcell <= resize(s_wcol(10 downto 4), 8);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= resize(s_wcol(10 downto 4), 8) + 1;
            else
                s_wcol  <= (others => '0');
                s_lb_ra <= (others => '0');
            end if;
            -- 2: sum sixteen; at the sixteenth, take the cell's states (read a cell ahead) for the update
            s_f1 <= '0';
            if s_lav = '0' then
                s_cur <= s_lb_q(11 downto 0);
            else
                if s_lc = 0 then s_acc <= resize(s_lin, 12); else s_acc <= s_acc + s_lin; end if;
                if s_lc = 15 then
                    s_sum   <= s_acc + s_lin;
                    s_old   <= s_cur;
                    s_cur   <= s_lb_q(11 downto 0);
                    s_fcell <= s_lcell;
                    s_f1    <= '1';
                end if;
            end if;
            -- 3: state += (sum - state) / 64
            v_d := signed(resize(s_sum, 14)) - signed(resize(unsigned(s_old), 14));
            v_d := signed(resize(unsigned(s_old), 14)) + shift_right(v_d, 6);
            s_n1  <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2  <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= x"0" & std_logic_vector(s_n1);

            -- pixel side: the cell stream (the line above's state), one slew across, gain
            s_lv <= unsigned(s_lb_q(11 downto 0));
            v_e  := signed(resize(s_lv & "0000", 17)) - signed(resize(s_h1, 17));
            v_e  := signed(resize(s_h1, 17)) + shift_right(v_e, 4);
            s_h1 <= unsigned(v_e(15 downto 0));
            s_cpl <= resize(c_cg(17 downto 5) * s_h1(11 downto 8), 17);
            s_cph <= resize(c_cg(17 downto 5) * s_h1(15 downto 12), 17);
            v_p := resize(s_cpl, 26) + shift_left(resize(s_cph, 26), 4);   -- luma x gain
            if s_src = '1' then s_cam <= v_p(19 downto 0); else s_cam <= v_p(20 downto 1); end if;
            s_cmo <= signed(resize(s_cam, 24)) - c_off;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine (kicked 6 clocks after the generated avid falls; in the zone after the table writer for line
    -- 0, which also reads the x function's first entry for the beam): the y function at the next line, read and
    -- interpolated; its slope per field line, weighted for the larger and the smaller input, as quarter octaves
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            s_gq4 <= s_gav_sr(4);
            l_kl <= s_gq4 and not s_gav_sr(4);
            l_kz <= w_done;
            if (l_kl = '1' or l_kz = '1') and l_run = '0' then
                l_run <= '1'; l_c <= (others => '0'); l_zon <= l_kz;
                if l_kz = '1' then l_vy <= c_vy0; l_tz <= '0'; end if;
            elsif l_run = '1' then
                l_c <= l_c + 1;
                if l_c = 8 then l_run <= '0'; end if;
            end if;
            if l_run = '1' then
                if l_zon = '1' and l_c = 4 then c_tx0 <= unsigned(r_t_q(19 downto 0)); end if;
                if l_c = 2 then l_vy <= l_vy + s_ys; end if;
            end if;
            -- the interpolated y function comes back from the pixel path's stage 7 (its job sample)
            if p7_lj = '1' then l_tvn <= p7_tj; l_tz <= '1'; end if;
            -- line 0 (from the zone run) takes its values at once
            if l_run = '0' and l_zon = '1' and l_tz = '1' then
                l_tv <= l_tvn; l_zon <= '0'; l_tz <= '0';
            end if;
            if g_start = '1' then
                l_tv <= l_tvn;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (half virtual px) and the patch-table read port: the line engine in hblank, else the beam's next entry
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_vx <= c_bx0;
            elsif g_avid = '1' then
                b_vx <= b_vx + shift_left(resize(s_xs, 16), 3);
            end if;
            s_gst <= s_gst(2 downto 0) & g_start;
            -- 2: table address
            if l_run = '1' and l_c = 0 then
                r_t_a <= resize(l_vy(18 downto 11), 9) + 320;
            elsif l_run = '1' and l_c = 1 then
                r_t_a <= resize(l_vy(18 downto 11), 9) + 321;
            elsif l_run = '1' and l_c = 2 then
                r_t_a <= c_bx0(15 downto 7);
            else
                r_t_a <= b_vx(15 downto 7) + 1;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : signed(24 downto 0);
        variable v_g0, v_g1, v_g2, v_g3 : signed(24 downto 0);
        variable v_th  : signed(20 downto 0);
        variable v_b   : signed(21 downto 0);
        variable v_code : unsigned(2 downto 0);
        variable v_val : unsigned(7 downto 0);
        variable v_src : std_logic_vector(3 downto 0);
        variable v_t   : unsigned(7 downto 0);
        variable v_e   : signed(8 downto 0);
        variable v_dd  : signed(8 downto 0);
        variable v_r   : signed(8 downto 0);
        variable v_ch  : signed(13 downto 0);
        variable v_h0, v_h1 : signed(13 downto 0);
        variable v_y   : unsigned(11 downto 0);
        variable v_lo, v_hi : std_logic;
        variable v_dn  : signed(20 downto 0);
        type t_tpv is array (1 to 5) of unsigned(7 downto 0);
        variable v_tp  : t_tpv;
        type t_c3v is array (0 to 2) of unsigned(7 downto 0);
        variable v_c3  : t_c3v;
    begin
        if rising_edge(clk) then
            -- 2: entry, fraction
            if l_run = '1' and l_c = 0 then
                p2_i <= resize(l_vy(18 downto 11), 9) + 320; p2_f <= (others => '0'); p2_lj <= '0';
            elsif l_run = '1' and l_c = 1 then
                p2_i <= resize(l_vy(18 downto 11), 9) + 321; p2_f <= l_vy(10 downto 7); p2_lj <= '1';
            else
                p2_i <= b_vx(15 downto 7); p2_f <= b_vx(6 downto 3); p2_lj <= '0';
            end if;
            p3_lj <= p2_lj; p4_lj <= p3_lj; p5_lj <= p4_lj; p6_lj <= p5_lj; p7_lj <= p6_lj;
            -- 3: table (EBR)
            p3_i <= p2_i; p3_f <= p2_f;
            -- 4: fabric copy of the next entry; the current entry follows it across entry boundaries
            p4_tn <= unsigned(r_t_q(19 downto 0));
            if s_gst(2) = '1' then
                p4_tc <= c_tx0;
            elsif p3_i /= p4_i then
                p4_tc <= p4_tn;
            end if;
            p4_i <= p3_i; p4_f <= p3_f;
            -- 5: difference
            p5_d <= signed(resize(p4_tn, 21)) - signed(resize(p4_tc, 21));
            p5_tc <= p4_tc; p5_f <= p4_f;
            -- 6: d x f
            v_d := resize(p5_d, 25);
            if p5_f(0) = '1' then v_g0 := v_d; else v_g0 := (others => '0'); end if;
            if p5_f(1) = '1' then v_g1 := shift_left(v_d, 1); else v_g1 := (others => '0'); end if;
            if p5_f(2) = '1' then v_g2 := shift_left(v_d, 2); else v_g2 := (others => '0'); end if;
            if p5_f(3) = '1' then v_g3 := shift_left(v_d, 3); else v_g3 := (others => '0'); end if;
            p6_q <= (v_g0 + v_g1) + (v_g2 + v_g3);
            p6_tc <= p5_tc;
            -- 7: TH = Tc + d f / 16 (the x function at the pixel; in hblank the line engine's y sample)
            v_th := signed(resize(p6_tc, 21)) + resize(shift_right(p6_q, 4), 21);
            p7_th <= unsigned(v_th(19 downto 0));
            p7_tj <= unsigned(v_th(19 downto 0));
            -- 8: TH - TV, TV - TH
            p8_dp <= signed(resize(p7_th, 21)) - signed(resize(l_tv, 21));
            p8_th <= p7_th;
            -- 9: Maximum, Minimum (DSG3 analog OR / AND), |TH - TV| ; the band density of the y input's role
            if p8_dp(20) = '0' then
                p9_mx <= p8_th; p9_mn <= l_tv; p9_ad <= unsigned(p8_dp(19 downto 0));
            else
                p9_mx <= l_tv; p9_mn <= p8_th; v_dn := -p8_dp; p9_ad <= unsigned(v_dn(19 downto 0));
            end if;
            -- 10: Shape 2 Average of (Shape 1 Average | Absolute) and (Shape 1 Minimum | Maximum)
            if s_hin = '0' and s_vin = '0' then          -- (M + 3 m) / 4 = m + |H - V| / 4: diamond + rectangle
                v_b := signed(resize(p9_mn, 22)) + signed(resize(shift_right(p9_ad, 2), 22));
            elsif s_hin = '0' then                       -- (3 M + m) / 4 = M - |H - V| / 4: diamond + cross
                v_b := signed(resize(p9_mx, 22)) - signed(resize(shift_right(p9_ad, 2), 22));
            elsif s_vin = '0' then                       -- (|H - V| + min) / 2 = max / 2, at full scale: saltire + rectangle
                v_b := signed(resize(p9_mx, 22));
            else                                         -- (|H - V| + max) / 2 = M - m / 2, at full scale: saltire + cross
                v_b := signed(resize(p9_mx, 22)) - signed(resize(shift_right(p9_mn, 1), 22));
            end if;
            p10_x0 <= v_b;
            -- 11: + the camera (Stairs R in) - Steps x Phase (Ribbons: Center - Span)
            p11_x <= resize(p10_x0, 25) + resize(s_cmo, 25);
            -- 12: the Stairs taps 2X .. 32X (triangles of the phase's bits; 0 below the starting phase) or the
            --     Ribbons bits D0 D1 D2; the source on each cable (K5) and its ESG3 invert
            v_lo := p11_x(24);
            if p11_x(24) = '0' and p11_x(23 downto 17) /= "0000000" then v_hi := '1'; else v_hi := '0'; end if;
            if v_lo = '1' then v_code := "000";
            elsif v_hi = '1' then v_code := "111";
            else v_code := unsigned(p11_x(16 downto 14)); end if;
            for k in 1 to 5 loop
                if s_rib = '0' then
                    v_val := unsigned(p11_x(13 - k downto 6 - k));
                    if v_lo = '1' or p11_x = 0 then v_tp(k) := (others => '0');
                    elsif p11_x(14 - k) = '1' then v_tp(k) := not v_val;
                    else v_tp(k) := v_val; end if;
                else
                    if v_code(2 - ((k - 1) mod 3)) = '1' then v_tp(k) := (others => '1');
                    else v_tp(k) := (others => '0'); end if;
                end if;
            end loop;
            for c in 0 to 2 loop
                case to_integer(unsigned(c_src(c)(2 downto 0))) is
                    when 0 => v_t := v_tp(1);
                    when 1 => v_t := v_tp(2);
                    when 2 => v_t := v_tp(3);
                    when 3 => v_t := v_tp(4);
                    when others => v_t := v_tp(5);
                end case;
                if c_src(c)(3) = '1' then p12_t(c) <= not v_t; else p12_t(c) <= v_t; end if;
            end loop;
            if s_rib = '0' then
                if v_lo = '0' and p11_x /= 0 then p12_fe <= '1'; else p12_fe <= '0'; end if;
            else
                p12_fe <= not (v_lo or v_hi);
            end if;
            -- 13: (cable - 0.5 V) x the cable's ESG3 gain (K5)
            for c in 0 to 2 loop
                v_ch := resize(signed(resize(p12_t(c), 9)) - 128, 14);
                if c_g(c)(4) = '1' then
                    p13_q(c) <= shift_left(v_ch, 4);
                else
                    if c_g(c)(0) = '1' then v_h0 := v_ch; else v_h0 := (others => '0'); end if;
                    if c_g(c)(1) = '1' then v_h1 := shift_left(v_ch, 1); else v_h1 := (others => '0'); end if;
                    if c_g(c)(2) = '1' then v_h0 := v_h0 + shift_left(v_ch, 2); end if;
                    if c_g(c)(3) = '1' then v_h1 := v_h1 + shift_left(v_ch, 3); end if;
                    p13_q(c) <= v_h0 + v_h1;
                end if;
            end loop;
            p13_fe <= p12_fe;
            -- 14: / 16, and the band-density fade toward the mean (per field: full, 1/4, 1/8, 1/16, 0; not on the
            --     black ground) ; + 0.5 V ; black off the generated avid ; the ESG3 ROM addresses, G one pixel and
            --     B two pixels behind R (path delay)
            for c in 0 to 2 loop
                if p13_fe = '0' or c_lv(c) = 0 then v_ch := shift_right(p13_q(c), 4);
                elsif c_lv(c) = 1 then v_ch := shift_right(p13_q(c), 6);
                elsif c_lv(c) = 2 then v_ch := shift_right(p13_q(c), 7);
                elsif c_lv(c) = 3 then v_ch := shift_right(p13_q(c), 8);
                else v_ch := (others => '0'); end if;
                v_ch := v_ch + 128;
                if s_gav_sr(11) = '1' then v_c3(c) := unsigned(v_ch(7 downto 0)); else v_c3(c) := (others => '0'); end if;
            end loop;
            r_ra <= v_c3(0); p14_g1 <= v_c3(1); r_ga <= p14_g1;
            p14_b1 <= v_c3(2); p14_b2 <= p14_b1; r_ba <= p14_b2;
            -- 15: ROMs (EBR) ; 16: fabric copy
            p16_r <= r_rq; p16_g <= r_gq; p16_b <= r_bq;
            -- 17: the encoder's sums
            p17_y <= resize(unsigned(p16_r(31 downto 22)), 12) + resize(unsigned(p16_g(31 downto 22)), 12)
                     + resize(unsigned(p16_b(31 downto 22)), 12) + 64;
            p17_u <= resize(signed(p16_r(21 downto 11)), 13) + resize(signed(p16_g(21 downto 11)), 13)
                     + resize(signed(p16_b(21 downto 11)), 13) + 512;
            p17_v <= resize(signed(p16_r(10 downto 0)), 13) + resize(signed(p16_g(10 downto 0)), 13)
                     + resize(signed(p16_b(10 downto 0)), 13) + 512;
            -- 18: the encoder clips at 1 V (luma 940, pre-scaled for the output's 13/16)
            if p17_y > 940 then p18_y <= to_unsigned(940, 10); else p18_y <= p17_y(9 downto 0); end if;
            p18_u <= unsigned(p17_u(9 downto 0)); p18_v <= unsigned(p17_v(9 downto 0));
            -- 19: black off the generated avid
            if s_gav_sr(16) = '0' then
                s_y19 <= C_KY; s_u19 <= C_KU; s_v19 <= C_KV;
            else
                s_y19 <= p18_y; s_u19 <= p18_u; s_v19 <= p18_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output filter stores
    ---------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_dp <= s_dp + 1; s_dr <= s_dr + 1;
            s_qa <= da(to_integer(s_dr));
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y19) & std_logic_vector(s_u19) & std_logic_vector(s_v19);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y21) & std_logic_vector(s_u21) & std_logic_vector(s_v21);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the rack's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the black (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y19 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u19 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v19 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= C_KY; s_u21 <= C_KU; s_v21 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y21 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u21 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v21 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v23 <= v_sc(13 downto 4);
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

end architecture stairwell;
