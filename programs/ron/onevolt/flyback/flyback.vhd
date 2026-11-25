-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Flyback v0.1.0 -- Onevolt 4: a shape patched into an oscillator's sync input.
--
-- The rack (LZX Expedition):
--   Visual Cortex H ramp (1 - P12) + shape gradient (P12; S7 Diamond = H+V triangles / Ellipse = parabolic ramps)
--     + camera luma (K6; S11 Osc = a drifting lattice) + a 13 s LFO   -> Passage (sum)
--   Passage -> Staircase (Frequency = K2 Contours, 1 .. 12; Phase swayed by a 21 s LFO)
--   Staircase /1 -> Prismatic Ray Sync / Reset in (0.5 V comparator, a little hysteresis)
--   Prismatic Ray: Frequency K1 (bars per contour interval), Sync mode S8 (Trigger / Gate), Pedestal K4, Gain K3,
--     Multiply = the density fade (bars fade out where the restart column runs away from line to line),
--     Frequency CV = the shape gradient (S9 Out / In: bars tighten toward the centre)
--   Prismatic Ray Triangle, Parabolic, Exponential -> Color Chords layers 1-3 (K5 sweeps eight colourings)
--   S10: a second Staircase fed by the triangle (/1 /2 /4 -> R G B; K5 = its Frequency) in Color Chords' place
--   a 37 s LFO leans the figure's centre up and down
--
-- Engine: vblank-zone sequencer (one 48-bit serial multiplier, 13 slots, shared knob glide, shared exp2 ROM); a
-- palette writer that runs the Pedestal + Gain stage as a DDA over 256 levels of the faded triangle and crossfades
-- the two colouriser tables either side of K5 into a 256-entry live palette; a line engine (the shape's vertical
-- part by second differences, two serial products per line); pixel DDAs (the sync phase, a virtual-x counter and
-- its square for the Frequency CV); the oscillator loop and a 14-stage pixel path.  Virtual 1080-line coordinates;
-- generated avid; output conditioning (two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16, a vertical [1 1]/2 with the
-- line above) before the blanking gate.  Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.flyback_pkg.all;
use work.all;

architecture flyback of program_top is
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
    -- triangle LFO of a 16-bit phase: +-32767
    function f_lfo(t : unsigned(15 downto 0)) return signed is
        variable v_x : unsigned(14 downto 0);
    begin
        if t(15) = '1' then v_x := not t(14 downto 0); else v_x := t(14 downto 0); end if;
        return signed(resize(v_x & '0', 17)) - 32767;
    end function;
    -- Mitchell log2 (Q4) of a 19-bit value: the leading-one position and the four bits after it
    function f_log(x : unsigned(18 downto 0)) return unsigned is
        variable v_e : integer range 0 to 18 := 0;
        variable v_x : unsigned(22 downto 0);
        variable v_m : unsigned(3 downto 0);
    begin
        for i in 0 to 18 loop
            if x(i) = '1' then v_e := i; end if;
        end loop;
        v_x := x & "0000";
        for k in 0 to 3 loop
            v_m(k) := v_x(v_e + k);
        end loop;
        return to_unsigned(v_e, 5) & v_m;
    end function;

    -- camera cell store: per 16-pixel cell the two vertical-slew states (s1, s2), read a cell ahead of the write
    type t_lb is array (0 to 127) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(6 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav, s_f1, s_f2 : std_logic := '0';
    signal s_lc    : unsigned(3 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(6 downto 0) := (others => '0');
    signal s_acc, s_sum : unsigned(11 downto 0) := (others => '0');
    signal s_c1, s_c2, s_o1, s_o2 : unsigned(11 downto 0) := (others => '0');
    signal s_n1, s_n2 : unsigned(11 downto 0) := (others => '0');
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_ga    : unsigned(23 downto 0) := (others => '0');
    signal s_gb    : unsigned(11 downto 0) := (others => '0');
    signal s_gacc  : unsigned(23 downto 0) := (others => '0');
    signal s_g16   : unsigned(15 downto 0) := (others => '0');
    signal s_h1, s_h2 : unsigned(19 downto 0) := (others => '0');
    signal s_cam   : unsigned(15 downto 0) := (others => '0');

    -- live palette: 256 entries, rewritten in the zone
    type t_pl is array (0 to 255) of std_logic_vector(31 downto 0);
    signal pl : t_pl := (others => C_FB_K);
    attribute no_rw_check of pl : signal is true;
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := C_FB_K;
    signal wt_we  : std_logic := '0';
    signal wt_a   : unsigned(7 downto 0) := (others => '0');
    signal wt_d   : std_logic_vector(31 downto 0) := C_FB_K;
    -- ROMs (registered address and data)
    signal r_e_a  : unsigned(7 downto 0) := (others => '0');
    signal r_e_q  : unsigned(15 downto 0) := (others => '0');
    signal r_p_a  : unsigned(9 downto 0) := (others => '0');
    signal r_p_q  : std_logic_vector(31 downto 0) := (others => '0');

    -- output filter stores (as the shell)
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da, db : t_dl := (others => (others => '0'));
    attribute no_rw_check of da : signal is true;
    attribute no_rw_check of db : signal is true;
    signal s_dp : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_dr : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_qa, s_qb, s_fa, s_fb : std_logic_vector(31 downto 0) := (others => '0');
    type t_vy is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_vc is array (0 to 511) of std_logic_vector(15 downto 0);
    signal vy : t_vy := (others => x"18");
    signal vc : t_vc := (others => x"8080");
    attribute no_rw_check of vy : signal is true;
    attribute no_rw_check of vc : signal is true;
    signal v_k17, v_k19 : unsigned(10 downto 0) := (others => '0');
    signal v_qy, v_fy, v_wyd : std_logic_vector(7 downto 0) := x"18";
    signal v_qc, v_fc, v_wcd : std_logic_vector(15 downto 0) := x"8080";
    signal v_wya : unsigned(9 downto 0) := (others => '0');
    signal v_wca : unsigned(8 downto 0) := (others => '0');
    signal v_wye, v_wce : std_logic := '0';
    signal v_py, v_pu, v_pv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal m_ys2  : unsigned(20 downto 0) := (others => '0');          -- per-mode constants used in arithmetic
    signal m_xadj : unsigned(8 downto 0) := (others => '0');
    signal m_icm1 : unsigned(10 downto 0) := to_unsigned(959, 11);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(650 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(620 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(120 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(560 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(780 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_shp, s_gate, s_fm, s_stair, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- LFO phases and values
    signal z_t13, z_t21, z_t37 : unsigned(15 downto 0) := (others => '0');
    signal c_s13, c_s21 : signed(16 downto 0) := (others => '0');
    -- per-field values
    signal c_cy8  : unsigned(17 downto 0) := to_unsigned(138240, 18);
    signal z_e2   : unsigned(9 downto 0) := (others => '0');
    signal z_e1   : unsigned(10 downto 0) := (others => '0');
    signal z_lq   : signed(12 downto 0) := (others => '0');
    signal z_sl   : signed(12 downto 0) := (others => '0');
    signal z_ea   : unsigned(7 downto 0) := (others => '0');
    signal c_kc   : unsigned(15 downto 0) := to_unsigned(4096, 16);
    signal c_q    : unsigned(12 downto 0) := to_unsigned(4000, 13);
    signal c_g8   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal c_p    : signed(10 downto 0) := (others => '0');
    signal c_km, c_kr : unsigned(15 downto 0) := (others => '0');
    signal c_u0   : signed(23 downto 0) := (others => '0');
    signal c_bank : unsigned(3 downto 0) := (others => '0');
    signal c_fr   : unsigned(6 downto 0) := (others => '0');
    signal c_pq0  : unsigned(29 downto 0) := (others => '0');
    signal c_pd0, c_pdd, c_pjd : signed(27 downto 0) := (others => '0');
    signal c_sy0  : signed(31 downto 0) := (others => '0');
    signal c_dsy0 : signed(26 downto 0) := (others => '0');
    signal c_ddsy : signed(26 downto 0) := (others => '0');
    signal c_cgx  : unsigned(11 downto 0) := (others => '0');
    signal z_tt   : signed(31 downto 0) := (others => '0');

    -- serial multiply (the zone's slots; the line engine's two products during the picture):
    -- acc = preload +- a x b (mod 2^42)
    signal f_a   : unsigned(41 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(41 downto 0) := (others => '0');
    signal z_fa  : signed(32 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_ng, z_keep, z_pl : std_logic := '0';
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal e_ld  : std_logic := '0';
    signal e_a   : signed(23 downto 0) := (others => '0');
    signal e_b   : unsigned(15 downto 0) := (others => '0');

    -- palette writer
    signal w_j   : unsigned(11 downto 0) := (others => '0');
    signal w_run : std_logic := '0';
    signal w_u   : signed(23 downto 0) := (others => '0');
    signal w_lv  : unsigned(5 downto 0) := (others => '0');
    signal w_ay, w_au, w_av : unsigned(9 downto 0) := (others => '0');
    signal w_by, w_bu, w_bv : unsigned(9 downto 0) := (others => '0');
    signal w_sy, w_su, w_sv : signed(17 downto 0) := (others => '0');
    signal w_cy, w_cu, w_cv : signed(17 downto 0) := (others => '0');
    signal w_frs : unsigned(6 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine: r (diamond, sigma = |r|) or sigma (ellipse) by second differences; for the NEXT line
    -- LQ = KM sigma >> 12 (Q30 fold periods, mod 1) and LamL (Q8 octaves) on one serial multiplier, LD and its log
    signal e_st  : integer range 0 to 9 := 0;
    signal e_z   : std_logic := '0';
    signal e_r   : signed(31 downto 0) := (others => '0');
    signal e_d   : signed(26 downto 0) := (others => '0');
    signal e_dd  : signed(26 downto 0) := (others => '0');
    signal e_sg  : signed(23 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_nlq : unsigned(29 downto 0) := (others => '0');
    signal e_nll : signed(11 downto 0) := (others => '0');
    signal e_nld : signed(29 downto 0) := (others => '0');
    signal e_av  : unsigned(18 downto 0) := (others => '0');
    signal e_nlg : signed(9 downto 0) := to_signed(-384, 10);
    signal l_lq  : unsigned(29 downto 0) := (others => '0');
    signal l_ll  : signed(11 downto 0) := (others => '0');
    signal l_lg  : signed(9 downto 0) := to_signed(-384, 10);

    -- beam (stage 1): sync-phase pixel DDA; the virtual x (half px) and its square for the Frequency CV
    signal b_pq  : unsigned(29 downto 0) := (others => '0');
    signal b_pd  : signed(27 downto 0) := (others => '0');
    signal b_h   : unsigned(11 downto 0) := (others => '0');
    signal b_s   : unsigned(21 downto 0) := (others => '0');
    signal b_sd  : signed(14 downto 0) := (others => '0');
    signal b_cd  : unsigned(10 downto 0) := (others => '0');
    signal b_z   : std_logic := '0';
    signal s_gst : std_logic_vector(6 downto 0) := (others => '0');   -- s_gst(k) = g_start delayed k + 1

    -- pixel path
    signal p2_phi : unsigned(15 downto 0) := (others => '0');
    signal p2_lx  : unsigned(9 downto 0) := (others => '0');
    signal p2_apd : unsigned(18 downto 0) := (others => '0');
    signal p3_phi : unsigned(15 downto 0) := (others => '0');
    signal p3_lb  : signed(12 downto 0) := (others => '0');
    signal p3_lg  : unsigned(8 downto 0) := (others => '0');
    signal p4_hi, p4_lo, p5_hi, p5_lo, p6_hi, p6_lo, p7_hi, p7_lo : std_logic := '0';
    signal p4_lam : unsigned(12 downto 0) := (others => '0');
    signal p4_lpd : unsigned(8 downto 0) := (others => '0');
    signal p5_sh, p6_sh : unsigned(4 downto 0) := (others => '0');
    signal p5_lv  : signed(11 downto 0) := (others => '0');
    signal p6_ex  : unsigned(12 downto 0) := (others => '0');
    signal p6_w, p7_w : unsigned(4 downto 0) := (others => '0');
    signal p7_f   : unsigned(23 downto 0) := (others => '0');
    signal o_c, o_held : std_logic := '0';
    signal o_a    : unsigned(23 downto 0) := (others => '0');
    signal o_w    : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal p9_tri : unsigned(9 downto 0) := (others => '0');
    signal p9_m   : unsigned(9 downto 0) := (others => '0');      -- |tri - c0| (sign p9_n): no self-adds of a sign bit
    signal p9_n, p10_n, p11_n : std_logic := '0';
    signal p9_w   : unsigned(4 downto 0) := (others => '0');
    signal p10_pa, p10_pb : unsigned(13 downto 0) := (others => '0');
    signal p10_tri, p11_tri : unsigned(9 downto 0) := (others => '0');
    signal p10_full, p11_full : std_logic := '0';
    signal p11_s  : unsigned(13 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := unsigned(C_FB_K(31 downto 22));
    constant C_KU : unsigned(9 downto 0) := unsigned(C_FB_K(21 downto 12));
    constant C_KV : unsigned(9 downto 0) := unsigned(C_FB_K(11 downto 2));

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

    -- the live palette: written only in the zone (the pixel path's reads there are unused)
    p_pl : process(clk)
    begin
        if rising_edge(clk) then
            r_pl_q <= pl(to_integer(r_pl_a));
            if wt_we = '1' then pl(to_integer(wt_a)) <= wt_d; end if;
        end if;
    end process p_pl;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_e_q <= C_FB_EXP(to_integer(r_e_a));
            r_p_q <= C_FB_PAL(to_integer(r_p_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_kk : unsigned(12 downto 0);
        variable v_g  : unsigned(10 downto 0);
        variable v_g2 : unsigned(13 downto 0);
        variable v_s  : signed(17 downto 0);
        variable v_t  : signed(18 downto 0);
        variable v_x  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xsr <= f_vstep_x(s_tid);
            m_ys2 <= f_yys2(s_tid); m_xadj <= f_xxadj(s_tid); m_icm1 <= f_xic(s_tid) - 1;

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
                s_shp   <= registers_in(6)(0);
                s_gate  <= registers_in(6)(1);
                s_fm    <= registers_in(6)(2);
                s_stair <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                z_t13 <= z_t13 + 84; z_t21 <= z_t21 + 52; z_t37 <= z_t37 + 30;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

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
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 32 or s_gd < -32 or (z_gc /= 4 and (s_gd > 16 or s_gd < -16)) then
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
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial multiply slots (32 clocks each): slot k is decoded at zc 62 + 32 k, its operands, sign and
            -- preload / keep are pre-muxed at 63 + 32 k, it loads at 64 + 32 k (16 iterations) and is captured at
            -- 82 + 32 k.  Outside the zone the line engine loads its own products (e_ld).
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(11 downto 5) >= 1 and s_zc(11 downto 5) <= 13 then
                z_sel <= resize(s_zc(8 downto 5) - 1, 4); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_ng <= '0'; z_keep <= '0'; z_pl <= '0';
                case to_integer(z_sel) is
                    when 0  => z_fa <= signed(resize(c_kc, 33));               z_fb <= resize(s_gp(12 downto 3), 16);   -- KM
                    when 1  => z_fa <= resize(c_p, 33) - 512;                  z_fb <= resize(c_g8, 16);                -- writer start
                    when 2  => z_fa <= shift_left(resize(c_s13, 33), 11);      z_fb <= c_kc; z_pl <= '1';               -- phase sway + LFO13 x Kc
                    when 3  => if s_shp = '1' then z_fa <= signed(resize(f_xeq0(s_tid), 33)); else z_fa <= signed(resize(f_xdq0(s_tid), 33)); end if;
                               z_fb <= c_km; z_keep <= '1';
                    when 4  => z_fa <= signed(resize(f_xrq0(s_tid), 33));      z_fb <= c_kr; z_keep <= '1';
                    when 5  => z_fa <= signed(resize(f_xra(s_tid), 33));       z_fb <= c_kr;
                    when 6  => if s_shp = '1' then z_fa <= signed(resize(f_xed0(s_tid), 33)); else z_fa <= signed(resize(f_xbd(s_tid), 33)); end if;
                               z_fb <= c_km; z_ng <= '1'; z_keep <= '1';
                    when 7  => if s_shp = '1' then z_fa <= signed(resize(f_xedd(s_tid), 33)); else z_fa <= signed(resize(f_xbd(s_tid) & '0', 33)); end if;
                               z_fb <= c_km;
                    when 8  => if s_shp = '1' then z_fa <= signed(resize(c_cy8(17 downto 2), 33)); z_fb <= c_cy8(17 downto 2);
                               else z_fa <= signed(resize(c_cy8, 33)); z_fb <= to_unsigned(3531, 16); z_ng <= '1'; end if;
                    when 9  => z_fa <= signed(resize(f_acc(31 downto 4), 33)); z_fb <= to_unsigned(29459, 16);          -- (cy^2 >> 4) x c
                    when 10 => z_fa <= signed(resize(c_cy8, 33));              z_fb <= resize(f_vstep_y(s_tid), 16);
                    when 11 => z_fa <= resize(z_tt, 33); z_ng <= '1';
                               z_fb <= to_unsigned(1841, 16);
                    when others => z_fa <= signed(resize(c_kc, 33));           z_fb <= resize(s_g6(12 downto 3), 16);   -- camera gain
                end case;
            end if;
            if z_ld = '1' then
                if z_keep = '1' then null;
                elsif z_pl = '1' then f_acc <= unsigned(shift_left(resize(c_s21, 42), 26));
                else f_acc <= (others => '0'); end if;
                if z_ng = '1' then f_a <= unsigned(-resize(z_fa, 42)); else f_a <= unsigned(resize(z_fa, 42)); end if;
                f_b <= z_fb;
            elsif e_ld = '1' then
                f_acc <= (others => '0'); f_a <= unsigned(resize(e_a, 42)); f_b <= e_b;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(40 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 =>                          -- knob decodes (after the glide) and the LFOs
                        c_p <= signed(resize(s_g4(12 downto 3), 11)) - 512;
                        v_kk := shift_left(resize(unsigned(registers_in(4)), 13), 3) - resize(unsigned(registers_in(4)), 13);
                        c_bank <= s_stair & v_kk(12 downto 10);
                        c_fr <= v_kk(9 downto 3);
                        v_g2 := resize(s_g2(12 downto 3), 14);
                        v_g2 := shift_right(shift_left(v_g2, 3) - v_g2, 3) + shift_right(v_g2, 5);
                        z_e2 <= v_g2(9 downto 0);
                        v_g := resize(s_g1(12 downto 3), 11);
                        z_e1 <= v_g + shift_right(v_g, 2) - shift_right(v_g, 5);
                        c_s13 <= f_lfo(z_t13); c_s21 <= f_lfo(z_t21);
                        v_s := resize(f_lfo(z_t37), 18);
                        v_t := to_signed(138240, 19) + resize(shift_right(v_s, 2), 19) + resize(shift_right(v_s, 4), 19);
                        c_cy8 <= unsigned(v_t(17 downto 0));
                    when 31 =>
                        z_lq <= signed(resize(z_e1, 13)) + signed(resize(z_e2, 13)) - 3131;
                        z_ea <= z_e2(7 downto 0);
                    when 32 =>                          -- the soft limit of log2 f0 (x 0.61 above -7 octaves)
                        v_x := resize(z_lq, 14) + 1792;
                        z_sl <= resize(shift_right(v_x, 1) + shift_right(v_x, 3) - shift_right(v_x, 6), 13);
                    when 33 =>
                        if z_lq > -1792 then
                            v_x := to_signed(-1792 + 6144, 14) + resize(z_sl, 14) + signed(resize(m_xadj, 14));
                        else
                            v_x := resize(z_lq, 14) + 6144 + signed(resize(m_xadj, 14));
                        end if;
                        c_q <= unsigned(v_x(12 downto 0));
                    when 35 =>
                        c_kc <= shift_left(resize(r_e_q(12 downto 0), 16), to_integer(z_e2(9 downto 8)));
                        z_ea <= s_g3(10 downto 3);
                    when 39 => c_g8 <= resize(shift_right(shift_left(resize(r_e_q(12 downto 0), 16), to_integer(s_g3(12 downto 11))), 4), 12);
                    when 82  => c_km <= f_acc(25 downto 10);
                    when 83  => c_kr <= c_kc - c_km;
                    when 114 => c_u0 <= signed(f_acc(23 downto 0));
                    when 210 => c_pq0 <= f_acc(41 downto 12);                -- (sway + LFO + KM x shape + KR x ramp) >> 12
                    when 274 => c_pd0 <= signed(f_acc(39 downto 12));        -- (KR x ramp - KM x shape slope) >> 12
                    when 306 =>
                        c_pjd <= signed(f_acc(39 downto 12));
                        if s_shp = '1' then c_pdd <= signed(f_acc(39 downto 12)); else c_pdd <= (others => '0'); end if;
                    when 338 => if s_shp = '0' then c_sy0 <= signed(f_acc(31 downto 0)); end if;
                    when 370 => if s_shp = '1' then c_sy0 <= signed(resize(f_acc(41 downto 12), 32)); end if;
                    when 402 => z_tt <= signed(f_acc(30 downto 0) & '0') - signed(resize(m_ys2, 32));
                    when 434 =>
                        if s_shp = '1' then c_dsy0 <= resize(signed(f_acc(41 downto 16)), 27); c_ddsy <= signed(resize(f_yeddy(s_tid), 27));
                        else c_dsy0 <= signed(resize(f_ydsd(s_tid), 27)); c_ddsy <= (others => '0'); end if;
                    when 466 => c_cgx <= f_acc(25 downto 14);              -- camera gain into Passage: (Kc x K6) >> 14
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
    -- Palette writer (zone clocks 400 .. 4495, 16 clocks per entry): for each level of the faded triangle
    -- t' = 4 e the Prismatic Ray's Pedestal + Gain stage y = clip(G (t' + p - 512) / 256 + 512) as a DDA
    -- (u += 4 G), then live[e] = A + ((B - A) x fr >> 7), A and B the colouriser tables either side of K5 at
    -- y >> 4 (the crossfade as a serial shift-add, three channels at once)
    ---------------------------------------------------------------------------
    p_wr : process(clk)
        variable v_y : signed(16 downto 0);
        variable v_n : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            wt_we <= '0';
            if s_zone = '1' and s_zc = 400 then
                w_j <= (others => '0'); w_run <= '1'; w_u <= c_u0;
            elsif w_run = '1' then
                w_j <= w_j + 1;
                if w_j = 4095 then w_run <= '0'; end if;
            end if;
            if w_run = '1' then
                case to_integer(w_j(3 downto 0)) is
                    when 0 =>
                        v_y := resize(shift_right(w_u, 8), 17) + 512;
                        if v_y < 0 then w_lv <= (others => '0');
                        elsif v_y > 1023 then w_lv <= (others => '1');
                        else w_lv <= unsigned(v_y(9 downto 4)); end if;
                    when 1 => r_p_a <= c_bank & w_lv;
                    when 2 => r_p_a <= (c_bank + 1) & w_lv;
                    when 3 => w_ay <= unsigned(r_p_q(31 downto 22)); w_au <= unsigned(r_p_q(21 downto 12)); w_av <= unsigned(r_p_q(11 downto 2));
                    when 4 => w_by <= unsigned(r_p_q(31 downto 22)); w_bu <= unsigned(r_p_q(21 downto 12)); w_bv <= unsigned(r_p_q(11 downto 2));
                    when 5 =>
                        w_sy <= signed(resize(w_by, 18)) - signed(resize(w_ay, 18));
                        w_su <= signed(resize(w_bu, 18)) - signed(resize(w_au, 18));
                        w_sv <= signed(resize(w_bv, 18)) - signed(resize(w_av, 18));
                        w_cy <= (others => '0'); w_cu <= (others => '0'); w_cv <= (others => '0');
                        w_frs <= c_fr;
                    when 6 | 7 | 8 | 9 | 10 | 11 | 12 =>
                        if w_frs(0) = '1' then w_cy <= w_cy + w_sy; w_cu <= w_cu + w_su; w_cv <= w_cv + w_sv; end if;
                        w_sy <= shift_left(w_sy, 1); w_su <= shift_left(w_su, 1); w_sv <= shift_left(w_sv, 1);
                        w_frs <= '0' & w_frs(6 downto 1);
                    when 13 =>
                        wt_a <= w_j(11 downto 4);
                        v_n := signed(resize(w_ay, 19)) + resize(shift_right(w_cy, 7), 19); wt_d(31 downto 22) <= std_logic_vector(v_n(9 downto 0));
                        v_n := signed(resize(w_au, 19)) + resize(shift_right(w_cu, 7), 19); wt_d(21 downto 12) <= std_logic_vector(v_n(9 downto 0));
                        v_n := signed(resize(w_av, 19)) + resize(shift_right(w_cv, 7), 19); wt_d(11 downto 2) <= std_logic_vector(v_n(9 downto 0));
                        wt_d(1 downto 0) <= "00";
                    when 14 => wt_we <= '1';
                    when 15 => w_u <= w_u + signed(resize(c_g8 & "00", 24));
                    when others => null;
                end case;
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
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- ((Y >> 2) - 16), summed over 16-pixel cells, each cell slewed down the
    -- screen by two poles (1/16 per line each; the states live in the cell
    -- store), scaled by the source gain once per cell (a serial multiplier),
    -- then slewed across by two one-pole filters (1/32 per pixel; Osc at twice
    -- the camera's gain).  All the filters decay, so none needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_n : signed(12 downto 0);
        variable v_e : signed(20 downto 0);
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
            s_lcell <= s_wcol(10 downto 4);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= s_wcol(10 downto 4) + 1;
            else
                s_wcol  <= (others => '0');
                s_lb_ra <= (others => '0');
            end if;
            -- 2: sum sixteen; at the sixteenth, take the cell's states (read a cell ahead) for the update
            s_f1 <= '0';
            if s_lav = '0' then
                s_c1 <= unsigned(s_lb_q(27 downto 16)); s_c2 <= unsigned(s_lb_q(11 downto 0));
            else
                if s_lc = 0 then s_acc <= resize(s_lin, 12); else s_acc <= s_acc + s_lin; end if;
                if s_lc = 15 then
                    s_sum   <= s_acc + s_lin;
                    s_o1    <= s_c1; s_o2 <= s_c2;
                    s_c1    <= unsigned(s_lb_q(27 downto 16)); s_c2 <= unsigned(s_lb_q(11 downto 0));
                    s_fcell <= s_lcell;
                    s_f1    <= '1';
                end if;
            end if;
            -- 3: s1 += (cell sum - s1) / 16 ; s2 += (s1' - s2) / 16
            v_d := signed(resize(s_sum, 13)) - signed(resize(s_o1, 13));
            v_n := signed(resize(s_o1, 13)) + shift_right(v_d, 4);
            s_n1 <= unsigned(v_n(11 downto 0));
            v_d := v_n - signed(resize(s_o2, 13));
            v_d := signed(resize(s_o2, 13)) + shift_right(v_d, 4);
            s_n2 <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_n1) & "0000" & std_logic_vector(s_n2);

            -- pixel side: the cell stream (the line above's state) x the source gain, once per cell (12 steps)
            s_lv <= unsigned(s_lb_q(11 downto 0));
            if s_lc = 2 then
                s_ga <= resize(s_lv, 24); s_gb <= c_cgx; s_gacc <= (others => '0');
            else
                if s_gb(0) = '1' then s_gacc <= s_gacc + s_ga; end if;
                s_ga <= s_ga(22 downto 0) & '0';
                s_gb <= '0' & s_gb(11 downto 1);
            end if;
            if s_lc = 15 then s_g16 <= s_gacc(23 downto 8); end if;
            -- two slews across
            v_e  := signed(resize(s_g16 & "0000", 21)) - signed(resize(s_h1, 21));
            v_e  := signed(resize(s_h1, 21)) + shift_right(v_e, 5);
            s_h1 <= unsigned(v_e(19 downto 0));
            v_e  := signed(resize(s_h1, 21)) - signed(resize(s_h2, 21));
            v_e  := signed(resize(s_h2, 21)) + shift_right(v_e, 5);
            s_h2 <= unsigned(v_e(19 downto 0));
            if s_src = '1' then s_cam <= s_h2(14 downto 0) & '0'; else s_cam <= s_h2(15 downto 0); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked in the zone (line 0) and at each line start; steps
    -- the shape's vertical part, then for the NEXT line LQ = (KM (sigma >> 8))
    -- >> 4 (Q30 fold periods, mod 1), LamL = ((sigma >> 8) C) >> 22 (Q8
    -- octaves), LD = LQ - LQ(this line) and log2 (|LD| >> 10)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_r  : signed(31 downto 0);
        variable v_ab : signed(29 downto 0);
    begin
        if rising_edge(clk) then
            e_ld <= '0';
            if s_zone = '1' and s_zc = 500 then
                e_r <= c_sy0; e_d <= c_dsy0; e_dd <= c_ddsy; e_z <= '1'; e_st <= 2;
            elsif g_start = '1' then
                l_lq <= e_nlq; l_ll <= e_nll; l_lg <= e_nlg; e_z <= '0'; e_st <= 1;
            else
                case e_st is
                    when 1 =>                           -- step to the next line
                        e_r <= e_r + resize(e_d, 32); e_d <= e_d + e_dd;
                        e_st <= 2;
                    when 2 =>                           -- sigma = |r| (diamond) or r (ellipse), >> 8
                        if s_shp = '0' and e_r < 0 then v_r := -e_r; else v_r := e_r; end if;
                        e_sg <= v_r(31 downto 8);
                        e_a <= v_r(31 downto 8); e_b <= c_km; e_ld <= '1'; e_cnt <= (others => '0');
                        e_st <= 3;
                    when 3 | 5 =>                       -- (the shared serial multiplier)
                        e_cnt <= e_cnt + 1;
                        if e_cnt = 17 then
                            if e_st = 3 then e_st <= 4; else e_st <= 6; end if;
                        end if;
                    when 4 =>
                        e_nlq <= f_acc(33 downto 4);
                        e_a <= e_sg; e_ld <= '1'; e_cnt <= (others => '0');
                        if s_shp = '1' then e_b <= to_unsigned(1283, 16); else e_b <= to_unsigned(891, 16); end if;
                        e_st <= 5;
                    when 6 =>
                        e_nll <= signed(f_acc(33 downto 22));
                        if e_z = '1' then e_nld <= (others => '0'); else e_nld <= signed(e_nlq - l_lq); end if;
                        e_st <= 7;
                    when 7 =>                           -- |LD| (one's complement) >> 10 ; the serial normaliser
                        v_ab := e_nld xor (29 downto 0 => e_nld(29)); e_av <= unsigned(v_ab(28 downto 10));
                        e_cnt <= to_unsigned(18, 5); e_st <= 8;
                    when 8 =>
                        if e_av(18) = '1' or e_cnt = 0 then
                            e_nlg <= signed(resize(e_cnt & e_av(17 downto 14), 10)) - 384; e_st <= 0;
                        else
                            e_av <= e_av(17 downto 0) & '0'; e_cnt <= e_cnt - 1;
                        end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): the sync phase's pixel part PQ (Q30 fold periods, mod 1)
    -- and its step PD (the diamond's slope flips at the centre column, b_z; the
    -- ellipse's grows); the virtual x in half pixels and its square
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_pq <= c_pq0; b_pd <= c_pd0;
                b_cd <= m_icm1; b_z <= '0';
                case to_integer(s_xs) is
                    when 4      => b_h <= to_unsigned(480, 12); b_s <= to_unsigned(2073600, 22); b_sd <= to_signed(-11504, 15);
                    when 3      => b_h <= to_unsigned(0, 12);   b_s <= to_unsigned(3686400, 22); b_sd <= to_signed(-11511, 15);
                    when others => b_h <= to_unsigned(0, 12);   b_s <= to_unsigned(3686400, 22); b_sd <= to_signed(-7676, 15);
                end case;
            elsif g_avid = '1' then
                b_pq <= b_pq + unsigned(resize(b_pd, 30));
                if b_z = '1' then b_pd <= b_pd + c_pjd; else b_pd <= b_pd + c_pdd; end if;
                b_cd <= b_cd - 1;
                if b_cd = 1 then b_z <= '1'; else b_z <= '0'; end if;
                b_h <= b_h + s_xs;
                b_s <= unsigned(signed(b_s) + resize(b_sd, 22));
                case to_integer(s_xs) is
                    when 4      => b_sd <= b_sd + 32;
                    when 3      => b_sd <= b_sd + 18;
                    when others => b_sd <= b_sd + 8;
                end case;
            end if;
            s_gst <= s_gst(5 downto 0) & g_start;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_l   : signed(13 downto 0);
        variable v_ap  : signed(27 downto 0);
        variable v_hx  : signed(12 downto 0);
        variable v_f   : unsigned(23 downto 0);
        variable v_k   : integer range 0 to 10;
        variable v_d2  : signed(12 downto 0);
        variable v_w   : signed(15 downto 0);
        variable v_pc  : std_logic;
        variable v_cn  : std_logic;
        variable v_st  : std_logic;
        variable v_rs  : std_logic;
        variable v_aa  : unsigned(23 downto 0);
        variable v_wl  : unsigned(4 downto 0);
        variable v_t   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: sync phase = line part + pixel part (top 16 bits) ; the Frequency CV's horizontal part ; |PD| >> 10
            p2_phi <= l_lq(29 downto 14) + b_pq(29 downto 14);
            if s_shp = '1' then
                p2_lx <= resize(b_s(21 downto 12), 10);
            else
                v_hx := signed(resize(b_h, 13)) - 1920;
                v_hx := v_hx xor (12 downto 0 => v_hx(12));
                p2_lx <= resize(unsigned(v_hx(11 downto 2)), 10);
            end if;
            v_ap := b_pd xor (27 downto 0 => b_pd(27)); p2_apd <= resize(unsigned(v_ap(26 downto 10)), 19);
            -- 3: + the source ; the Frequency CV's exponent ; log2 (|PD| >> 10)
            p3_phi <= p2_phi + s_cam;
            p3_lb <= to_signed(512, 13) - signed(resize(p2_lx, 13)) - resize(l_ll, 13);
            p3_lg <= f_log(p2_apd);
            -- 4: Staircase comparator (set above 0.65, clear below 0.35) ; log2 f (capped at 1/32 cycle per pixel)
            if p3_phi(15 downto 6) >= 333 and p3_phi(15 downto 6) <= 691 then p4_hi <= '1'; else p4_hi <= '0'; end if;
            if p3_phi(15 downto 6) <= 179 or p3_phi(15 downto 6) >= 845 then p4_lo <= '1'; else p4_lo <= '0'; end if;
            if s_fm = '1' and p3_lb > 0 then
                v_l := signed(resize(c_q, 14)) + resize(p3_lb, 14);
            else
                v_l := signed(resize(c_q, 14));
            end if;
            if v_l > 4864 then v_l := to_signed(4864, 14); end if;
            p4_lam <= unsigned(v_l(12 downto 0));
            if s_zone = '1' then r_e_a <= z_ea; else r_e_a <= unsigned(v_l(7 downto 0)); end if;   -- the shared exp2 ROM
            p4_lpd <= p3_lg;
            -- 5: (exp2 ROM) ; the shift ; the fade measure log2 (f |LD| / |PD|) in Q4 (cycles per line)
            p5_sh <= p4_lam(12 downto 8);
            p5_lv <= signed(resize(p4_lam(12 downto 4), 12)) + resize(l_lg, 12) - signed(resize(p4_lpd, 12));
            p5_hi <= p4_hi; p5_lo <= p4_lo;
            -- 6: fabric copy of the mantissa ; fade weight w = clamp(((-36 - lv) x 5) >> 3, 0, 16)
            p6_ex <= r_e_q(12 downto 0); p6_sh <= p5_sh;
            v_d2 := to_signed(-36, 13) - resize(p5_lv, 13);
            v_w := shift_right(shift_left(resize(v_d2, 16), 2) + resize(v_d2, 16), 3);
            if v_w < 0 then p6_w <= (others => '0');
            elsif v_w > 16 then p6_w <= to_unsigned(16, 5);
            else p6_w <= unsigned(v_w(4 downto 0)); end if;
            p6_hi <= p5_hi; p6_lo <= p5_lo;
            -- 7: f = exp2 (cycles x 2^24 per pixel)
            if p6_sh >= 11 and p6_sh <= 21 then v_k := to_integer(p6_sh) - 11; else v_k := 0; end if;
            v_f := shift_left(resize(p6_ex, 24), v_k);
            p7_f <= '0' & v_f(23 downto 1);
            p7_w <= p6_w; p7_hi <= p6_hi; p7_lo <= p6_lo;
            -- 8: Prismatic Ray: comparator with hysteresis ; Trigger = reset on the rise, Gate = held while high ;
            --    the core's phase ; the fade weight latched at the train's launch ; line start resets
            if s_gst(6) = '1' then
                v_pc := '0'; v_aa := (others => '0'); v_wl := to_unsigned(16, 5);
            else
                v_pc := o_c; v_aa := o_a; v_wl := o_w;
            end if;
            if p7_hi = '1' then v_cn := '1'; elsif p7_lo = '1' then v_cn := '0'; else v_cn := v_pc; end if;
            if s_gate = '1' then
                v_st := v_pc and not v_cn; v_rs := v_cn;
            else
                v_st := v_cn and not v_pc; v_rs := v_st;
            end if;
            if v_rs = '1' then o_a <= (others => '0'); else o_a <= v_aa + p7_f; end if;
            if v_st = '1' then o_w <= p7_w; else o_w <= v_wl; end if;
            o_c <= v_cn;
            o_held <= v_cn and s_gate;
            -- 9: triangle ; the fade's target c0 (the hold level under Gate, the middle under Trigger) ; w
            if o_a(23) = '1' then v_t := signed(resize(not o_a(22 downto 13), 12)); else v_t := signed(resize(o_a(22 downto 13), 12)); end if;
            p9_tri <= unsigned(v_t(9 downto 0));
            if s_gate = '1' then
                p9_m <= unsigned(v_t(9 downto 0)); p9_n <= '0';
            elsif v_t(9) = '1' then
                p9_m <= '0' & unsigned(v_t(8 downto 0)); p9_n <= '0';                 -- tri - 512 >= 0
            else
                p9_m <= to_unsigned(512, 10) - unsigned(v_t(9 downto 0)); p9_n <= '1'; -- 512 - tri > 0
            end if;
            if o_held = '1' then p9_w <= to_unsigned(16, 5); else p9_w <= o_w; end if;
            -- 10: (the core's Multiply) w x |tri - c0|, shift-add on the magnitude
            p10_pa <= (others => '0'); p10_pb <= (others => '0');
            if p9_w(0) = '1' and p9_w(1) = '1' then p10_pa <= resize(p9_m, 14) + shift_left(resize(p9_m, 14), 1);
            elsif p9_w(0) = '1' then p10_pa <= resize(p9_m, 14);
            elsif p9_w(1) = '1' then p10_pa <= shift_left(resize(p9_m, 14), 1); end if;
            if p9_w(2) = '1' and p9_w(3) = '1' then p10_pb <= shift_left(resize(p9_m, 14), 2) + shift_left(resize(p9_m, 14), 3);
            elsif p9_w(2) = '1' then p10_pb <= shift_left(resize(p9_m, 14), 2);
            elsif p9_w(3) = '1' then p10_pb <= shift_left(resize(p9_m, 14), 3); end if;
            p10_full <= p9_w(4); p10_tri <= p9_tri; p10_n <= p9_n;
            -- 11: sum
            p11_s <= p10_pa + p10_pb; p11_full <= p10_full; p11_tri <= p10_tri; p11_n <= p10_n;
            -- 12: t' = c0 + (w (tri - c0) >> 4) ; the live palette's address
            if p11_full = '1' then
                r_pl_a <= p11_tri(9 downto 2);
            elsif s_gate = '1' then
                r_pl_a <= p11_s(13 downto 6);                                         -- (w tri) >> 4, >> 2
            else
                if p11_n = '1' then v_t := resize(shift_right(-signed(resize(p11_s, 15)), 4), 12) + 512;
                else v_t := signed(resize(p11_s(13 downto 4), 12)) + 512; end if;
                r_pl_a <= unsigned(v_t(9 downto 2));
            end if;
            -- 13: (live palette) ; 14: black off the generated avid
            if s_gav_sr(11) = '0' then
                s_y14 <= C_KY; s_u14 <= C_KU; s_v14 <= C_KV;
            else
                s_y14 <= unsigned(r_pl_q(31 downto 22)); s_u14 <= unsigned(r_pl_q(21 downto 12)); s_v14 <= unsigned(r_pl_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(11);
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
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y14) & std_logic_vector(s_u14) & std_logic_vector(s_v14);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y16) & std_logic_vector(s_u16) & std_logic_vector(s_v16);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    p_vs : process(clk)
        variable v_s : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(15) = '1' then v_k17 <= v_k17 + 1; else v_k17 <= (others => '0'); end if;   -- column of the stage-17 pixel
            if s_gav_sr(17) = '1' then v_k19 <= v_k19 + 1; else v_k19 <= (others => '0'); end if;   -- column of the stage-19 pixel
            v_qy <= vy(to_integer(v_k17(10 downto 1)));
            v_qc <= vc(to_integer(v_k17(10 downto 2)));
            v_fy <= v_qy; v_fc <= v_qc;
            v_py <= s_y19; v_pu <= s_u19; v_pv <= s_v19;
            v_wye <= s_gav_sr(17) and v_k19(0);
            v_wya <= v_k19(10 downto 1);
            v_s := resize(s_y19, 11) + resize(v_py, 11); v_wyd <= std_logic_vector(v_s(10 downto 3));
            if s_gav_sr(17) = '1' and v_k19(1 downto 0) = "01" then v_wce <= '1'; else v_wce <= '0'; end if;
            v_wca <= v_k19(10 downto 2);
            v_s := resize(s_u19, 11) + resize(v_pu, 11); v_wcd(15 downto 8) <= std_logic_vector(v_s(10 downto 3));
            v_s := resize(s_v19, 11) + resize(v_pv, 11); v_wcd(7 downto 0) <= std_logic_vector(v_s(10 downto 3));
            if v_wye = '1' then vy(to_integer(v_wya)) <= v_wyd; end if;
            if v_wce = '1' then vc(to_integer(v_wca)) <= v_wcd; end if;
        end if;
    end process p_vs;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the rack's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16, a vertical [1 1]/2 with the line above (no
    -- single-row lines), and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_mx : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the black (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 15 sums, stage 16 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y14 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u14 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v14 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(13) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y16 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u16 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v16 <= v_sc(13 downto 4);
            else
                s_y16 <= C_KY; s_u16 <= C_KU; s_v16 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(13);
            -- box B: stage 17 sums, stage 18 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y16 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u16 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v16 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y18 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v18 <= v_sc(13 downto 4);
            -- 19: luma 13/16
            if s_y18 > 64 then v_dl := s_y18 - 64; else v_dl := (others => '0'); end if;
            s_y19 <= s_y18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u19 <= s_u18; s_v19 <= s_v18;
            -- 20: vertical mix with the line above
            v_mx := resize(s_y19, 11) + resize(unsigned(v_fy) & "10", 11);             s_y20 <= v_mx(10 downto 1);
            v_mx := resize(s_u19, 11) + resize(unsigned(v_fc(15 downto 8)) & "10", 11); s_u20 <= v_mx(10 downto 1);
            v_mx := resize(s_v19, 11) + resize(unsigned(v_fc(7 downto 0)) & "10", 11);  s_v20 <= v_mx(10 downto 1);
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
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture flyback;
