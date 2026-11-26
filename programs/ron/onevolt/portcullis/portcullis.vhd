-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Portcullis v0.1.0 -- Onevolt: an LZX Castle rack, three-bit video in clocked columns.
--
-- The patch:
--   DSG3 (internal H / V ramps, Fold 1): Average (diamond) or Maximum (windows) -> Stairs (Steps = K2 Bands)
--   Stairs Phase CV: K3 Bias + an 11 s LFO + a free V-range DWO3 sine (ripple, rolls) + camera luma x K6
--   Castle ADC -> D0 (MSB) D1 D2 ; Clock VCO (Frequency = P12, VC Level = K1 from the V ramp + a 27 s LFO,
--   hard-synced at every line start) clocks the D Flip Flops (each bit held for one clock period; the clock edge
--   falls between pixels and the field is sampled at the edge)
--   S9: the flip-flops take the Counter (D3 D2 D1, MSB to MSB; clock = VCO, reset = line sync and every ADC D0 edge,
--   Up / Down = ADC D0) instead of the ADC
--   FF D1 -> Shift Register A (K4 Echo e clocks), FF D2 -> Shift Register B (2e clocks), fine sub-clock steps
--   S7 Multi Gate: Thru / XOR (each plane with its own copy w clocks later)
--   S10: bits -> R G B (K5 = cable order / polarity, blended) / DAC -> SMX3 (K5 = matrix family, blended)
--
--   K1 Clock FM  bipolar: the columns fan (narrow at the top, wide at the bottom, or the reverse)
--   K2 Bands     Stairs Steps: 1 .. 8 half-folds from the centre to the edge
--   K3 Bias      slides the band sequence through the figure (0 .. 1.5 half-folds)
--   K4 Echo      shift-register delay 0 .. 3 clock periods per plane
--   K5 Colour    nine keyframes (cable configurations / SMX3 families), blended
--   K6 Camera    camera / Osc strength on the Stairs Phase CV
--   S7 Gate Thru / XOR ; S8 Shape Diamond / Windows ; S9 Bits ADC / Counter ; S10 Output RGB / DAC ;
--   S11 Source Video / Osc ; P12 CLOCK 384 -> 60 -> 6 columns
--
-- Engine: vblank-zone sequencer (one 40-bit serial multiplier, 9 slots and a 24-step palette interpolation loop,
-- shared knob glide), a per-line engine (V term, ripple, FM'd clock increment and its sub-pixel thresholds), a
-- 13-stage pixel path.  Virtual 1080-line coordinates.  Generated avid; output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16, a vertical [1 2 1]/4 with the two lines above) before the blanking gate.
-- Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.portcullis_pkg.all;
use work.all;

architecture portcullis of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- the field's V gain per field line and the FM line step (15534.5 ys / 512)
    function f_cyt(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(34528, 16);
            when 4          => return to_unsigned(17476, 16);
            when 8          => return to_unsigned(29127, 16);
            when 12         => return to_unsigned(14564, 16);
            when 5 | 6 | 14 => return to_unsigned(11651, 16);
            when 1 | 2 | 10 => return to_unsigned(15534, 16);
            when others     => return to_unsigned(7767, 16);
        end case;
    end function;
    -- the ripple's phase step per field line (212.4 ys / 256)
    function f_crt(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(944, 10);
            when 4          => return to_unsigned(478, 10);
            when 8          => return to_unsigned(796, 10);
            when 12         => return to_unsigned(398, 10);
            when 5 | 6 | 14 => return to_unsigned(319, 10);
            when 1 | 2 | 10 => return to_unsigned(425, 10);
            when others     => return to_unsigned(212, 10);
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
    -- x gain per real pixel (2^24 / 1920 virtual px x xs / 2) and the clock's count-keeping factor (Q8)
    function f_cxs(x : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(x) is
            when 4      => return to_unsigned(17476, 16);
            when 3      => return to_unsigned(13107, 16);
            when others => return to_unsigned(8738, 16);
        end case;
    end function;
    function f_cm8(x : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(x) is
            when 4      => return to_unsigned(683, 10);
            when 3      => return to_unsigned(384, 10);
            when others => return to_unsigned(256, 10);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- luma cell store: one 12-bit vertical-slew state per 8-pixel cell (read a cell ahead of the write)
    type t_lb is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(7 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(15 downto 0) := x"0000";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(15 downto 0) := x"0000";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav, s_f1, s_f2 : std_logic := '0';
    signal s_lc    : unsigned(2 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(7 downto 0) := (others => '0');
    signal s_acc, s_sum8 : unsigned(10 downto 0) := (others => '0');
    signal s_cur, s_old, s_new : unsigned(11 downto 0) := (others => '0');
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_h1, s_h2 : unsigned(15 downto 0) := (others => '0');
    signal s_la, s_lbh, s_lch : unsigned(12 downto 0) := (others => '0');
    signal s_lterm : unsigned(18 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_s_a  : unsigned(7 downto 0) := (others => '0');
    signal r_s_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal r_e_a  : unsigned(7 downto 0) := (others => '0');
    signal r_e_q  : unsigned(15 downto 0) := (others => '0');
    signal r_k_a  : unsigned(7 downto 0) := (others => '0');
    signal r_k_q  : std_logic_vector(31 downto 0) := (others => '0');
    -- palette RAM (8 entries used): written by the zone's palette loop, read by the pixel path
    type t_pr is array (0 to 255) of std_logic_vector(31 downto 0);
    signal pr : t_pr := (others => C_PC_K);
    attribute no_rw_check of pr : signal is true;
    signal r_pr_a : unsigned(7 downto 0) := (others => '0');
    signal r_pr_q : std_logic_vector(31 downto 0) := C_PC_K;
    signal pw_we  : std_logic := '0';
    signal pw_a   : unsigned(7 downto 0) := (others => '0');

    -- output filter stores (as the shell) and the two line stores of the vertical mix
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da, db : t_dl := (others => (others => '0'));
    attribute no_rw_check of da : signal is true;
    attribute no_rw_check of db : signal is true;
    signal s_dp : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_dr : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_qa, s_qb, s_fa, s_fb : std_logic_vector(31 downto 0) := (others => '0');
    type t_vy is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_vc is array (0 to 511) of std_logic_vector(15 downto 0);
    signal vy, vy2 : t_vy := (others => x"18");
    signal vc, vc2 : t_vc := (others => x"8080");
    attribute no_rw_check of vy : signal is true;
    attribute no_rw_check of vc : signal is true;
    attribute no_rw_check of vy2 : signal is true;
    attribute no_rw_check of vc2 : signal is true;
    signal v_k17, v_k19 : unsigned(10 downto 0) := (others => '0');
    signal v_qy, v_fy, v_wyd, v_qy2, v_fy2, v_wyd2 : std_logic_vector(7 downto 0) := x"18";
    signal v_qc, v_fc, v_wcd, v_qc2, v_fc2, v_wcd2 : std_logic_vector(15 downto 0) := x"8080";
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
    signal s_cyt  : unsigned(15 downto 0) := to_unsigned(7767, 16);
    signal s_crt  : unsigned(9 downto 0) := to_unsigned(212, 10);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(760 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(390 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_gate, s_shape, s_cnt, s_dac, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal z_l11, z_l27, z_roll : unsigned(15 downto 0) := (others => '0');
    signal z_eR  : unsigned(14 downto 0) := (others => '0');
    signal z_iR  : unsigned(1 downto 0) := (others => '0');
    signal z_pp  : unsigned(12 downto 0) := (others => '0');
    signal z_pq  : unsigned(11 downto 0) := (others => '0');
    signal z_kq  : unsigned(10 downto 0) := (others => '0');
    signal z_ebn : unsigned(16 downto 0) := (others => '0');
    signal z_b   : unsigned(12 downto 0) := (others => '0');
    signal z_bs  : std_logic := '0';
    signal z_ma, z_mb, z_mc, z_m12 : unsigned(11 downto 0) := (others => '0');
    signal z_e8  : unsigned(9 downto 0) := (others => '0');
    signal z_w1  : unsigned(9 downto 0) := (others => '0');
    signal z_ph  : unsigned(16 downto 0) := (others => '0');
    signal z_i5  : unsigned(2 downto 0) := (others => '0');
    signal z_fr  : unsigned(9 downto 0) := (others => '0');
    signal z_fq  : unsigned(9 downto 0) := (others => '0');
    signal z_mt  : unsigned(15 downto 0) := (others => '0');
    signal z_AR  : unsigned(17 downto 0) := (others => '0');
    signal z_a11, z_a27 : unsigned(10 downto 0) := (others => '0');   -- |sine|
    signal z_n11, z_n27 : std_logic := '0';
    signal z_la, z_lb : unsigned(15 downto 0) := (others => '0');
    signal z_lfo : signed(17 downto 0) := (others => '0');
    signal z_mla : unsigned(8 downto 0) := (others => '0');
    signal z_mL  : signed(11 downto 0) := (others => '0');
    signal z_m   : signed(13 downto 0) := (others => '0');
    signal z_ma2 : unsigned(11 downto 0) := (others => '0');
    signal z_mneg : std_logic := '0';
    signal z_ne  : unsigned(16 downto 0) := (others => '0');
    signal z_nI  : unsigned(2 downto 0) := (others => '0');
    signal z_cmn : unsigned(15 downto 0) := (others => '0');
    signal z_3n  : unsigned(7 downto 0) := (others => '0');
    signal z_wf  : unsigned(11 downto 0) := (others => '0');
    signal z_w   : unsigned(11 downto 0) := (others => '0');
    signal z_P   : unsigned(19 downto 0) := (others => '0');
    signal z_ia0 : signed(21 downto 0) := (others => '0');
    signal z_t8  : unsigned(8 downto 0) := (others => '0');
    signal c_gxp : unsigned(17 downto 0) := (others => '0');
    signal c_gyl : unsigned(19 downto 0) := (others => '0');
    signal c_ux0 : signed(27 downto 0) := (others => '0');
    signal c_pf  : unsigned(16 downto 0) := (others => '0');
    signal c_inc0, c_incmin : unsigned(19 downto 0) := (others => '0');
    signal c_dinc : signed(22 downto 0) := (others => '0');
    signal c_gl  : unsigned(8 downto 0) := (others => '0');
    signal c_dG, c_dB, c_dW, c_dGW, c_dBW : unsigned(11 downto 0) := (others => '0');

    -- serial multiply: acc = a x b (mod 2^40); loaded by a zone slot or by the palette loop
    signal f_a   : unsigned(39 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(39 downto 0) := (others => '0');
    signal z_fa  : signed(23 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

    -- palette loop: 8 levels x 3 channels, colour = A + (B - A) t8 / 256
    signal pw_run : std_logic := '0';
    signal pw_t   : unsigned(4 downto 0) := (others => '0');
    signal pw_n   : unsigned(2 downto 0) := (others => '0');
    signal pw_c   : unsigned(1 downto 0) := (others => '0');
    signal pw_ka  : std_logic_vector(31 downto 0) := (others => '0');
    signal pw_d   : signed(10 downto 0) := (others => '0');
    signal pw_ld  : std_logic := '0';
    signal pw_word : std_logic_vector(31 downto 0) := C_PC_K;     -- {Y, U, V, 00}

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_uy  : signed(27 downto 0) := (others => '0');
    signal e_rp  : unsigned(15 downto 0) := (others => '0');
    signal e_ia  : signed(29 downto 0) := (others => '0');
    signal z_sw  : std_logic := '0';
    signal z_sa  : unsigned(7 downto 0) := (others => '0');
    signal n_sr  : signed(11 downto 0) := (others => '0');
    signal n_rip : signed(13 downto 0) := (others => '0');
    signal n_inc : unsigned(19 downto 0) := (others => '0');
    signal l_hy  : unsigned(18 downto 0) := (others => '0');
    signal l_pl  : unsigned(16 downto 0) := (others => '0');
    signal l_inc, l_t3 : unsigned(19 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_ux  : signed(27 downto 0) := (others => '0');
    signal b_ph  : unsigned(19 downto 0) := (others => '0');
    signal b_w, b_first : std_logic := '0';

    -- pixel path
    type t_dq is array (2 to 6) of unsigned(1 downto 0);
    signal p_dq  : t_dq := (others => "00");
    signal p_w, p_f : std_logic_vector(7 downto 2) := (others => '0');
    type t_cf is array (2 to 7) of std_logic_vector(4 downto 0);   -- shift-register tap fractions (phase < fraction)
    signal p_cf  : t_cf := (others => (others => '0'));
    signal p2_hx : unsigned(18 downto 0) := (others => '0');
    signal p3_cmp : std_logic := '0';
    signal p3_avg : unsigned(16 downto 0) := (others => '0');
    signal p3_hx, p3_hy : unsigned(15 downto 0) := (others => '0');
    signal p4_fig, p4_cp, p5_s, p6_s, p6_sp, p6_ds, p7_se : unsigned(16 downto 0) := (others => '0');
    type t_sr is array (0 to 15) of unsigned(2 downto 0);
    signal sr    : t_sr := (others => "000");
    signal c_cnt : unsigned(3 downto 0) := (others => '0');
    signal c_mp  : std_logic := '0';
    signal p8_oG, p8_oB, p8_oW, p8_oGW, p8_oBW : unsigned(3 downto 0) := (others => '0');
    signal p9_r0, p9_g, p9_b, p9_rw, p9_gw, p9_bw : std_logic := '0';
    signal p10_o : std_logic_vector(2 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := (others => '0');
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
    constant C_KY : unsigned(9 downto 0) := unsigned(C_PC_K(31 downto 22));
    constant C_KU : unsigned(9 downto 0) := unsigned(C_PC_K(21 downto 12));
    constant C_KV : unsigned(9 downto 0) := unsigned(C_PC_K(11 downto 2));

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

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_s_q <= C_PC_SIN(to_integer(r_s_a));
            r_e_q <= C_PC_EXP(to_integer(r_e_a));
            r_k_q <= C_PC_PAL(to_integer(r_k_a));
        end if;
    end process p_rom;

    -- the palette RAM: written only in the zone (the pixel path's reads there are blanked)
    p_pr : process(clk)
    begin
        if rising_edge(clk) then
            r_pr_q <= pr(to_integer(r_pr_a));
            if pw_we = '1' then pr(to_integer(pw_a)) <= pw_word; end if;
        end if;
    end process p_pr;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_m  : signed(13 downto 0);
        variable v_bs : signed(14 downto 0);
        variable v_i  : signed(21 downto 0);
        variable v_y  : signed(11 downto 0);
        variable v_s  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xsr <= f_vstep_x(s_tid);
            s_cyt <= f_cyt(s_tid);
            s_crt <= f_crt(s_tid);
            s_oys <= f_oys(s_tid);

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
                s_gate  <= registers_in(6)(0);
                s_shape <= registers_in(6)(1);
                s_cnt   <= registers_in(6)(2);
                s_dac   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                z_l11   <= z_l11 + 99;
                z_l27   <= z_l27 + 40;
                z_roll  <= z_roll - 47;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;

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

            -- serial multiply slots: slot k loads at zc = 56 + 24 k, 16 iterations, result stable from load + 17
            -- (stored at load + 18).  The slot number is decoded at load - 2 and the operands pre-muxed at
            -- load - 1.  The palette loop loads the same multiplier after the slots (pw_ld).
            z_pre <= '0';
            if s_zone = '1' and s_zc >= 54 and s_zc <= 246 then
                case to_integer(s_zc) is
                    when 54  => z_sel <= to_unsigned(0, 4); z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(1, 4); z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(2, 4); z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(3, 4); z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(4, 4); z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(5, 4); z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(6, 4); z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(7, 4); z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(8, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= signed(resize(z_pq, 24));    z_fb <= resize(z_kq, 16);     -- eT slope
                    when 1  => z_fa <= signed(resize(z_AR, 24));    z_fb <= f_cxs(s_xs);          -- x gain
                    when 2  => z_fa <= signed(resize(z_AR, 24));    z_fb <= s_cyt;                -- y gain
                    when 3  => z_fa <= signed(resize(z_mt, 24));    z_fb <= z_cmn;                -- inc0
                    when 4  => z_fa <= signed(resize(z_mt, 24));    z_fb <= resize(z_3n, 16);     -- XOR floor
                    when 5  => z_fa <= signed(resize(c_inc0, 24));  z_fb <= resize(z_ma2, 16);    -- FM product
                    when 6  => z_fa <= signed(resize(z_P, 24));     z_fb <= s_cyt;                -- FM line step
                    when 7  => z_fa <= signed(resize(s_g6, 24));    z_fb <= to_unsigned(599, 16); -- camera gain
                    when others => z_fa <= signed(resize(z_fq, 24)); z_fb <= to_unsigned(423, 16); -- palette blend
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= unsigned(resize(z_fa, 40));
                f_b <= z_fb;
            elsif pw_ld = '1' then
                f_acc <= (others => '0');
                f_a <= unsigned(resize(pw_d, 40));
                f_b <= resize(z_t8, 16);
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- zone ROM windows: the sine ROM reads the two LFOs at zc 35 / 37 (the line engine owns it otherwise)
            z_sw <= '0';
            if s_zone = '1' and s_zc >= 34 and s_zc <= 38 then
                z_sw <= '1';
                if s_zc < 36 then z_sa <= z_l11(15 downto 8); else z_sa <= z_l27(15 downto 8); end if;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- knob decodes (after the glide, zc 0 .. 29)
                    when 32 =>
                        z_eR <= resize(s_g2, 15) + shift_left(resize(s_g2, 15), 1);
                        z_pp <= s_gp;
                        v_bs := signed(resize(s_g1, 15)) - 4096;
                        if v_bs(14) = '1' then z_bs <= '1'; v_bs := -v_bs; else z_bs <= '0'; end if;
                        z_b <= unsigned(v_bs(12 downto 0));
                        z_e8 <= resize(shift_right(resize(s_g4, 15) + shift_left(resize(s_g4, 15), 1), 5), 10);
                        z_ph <= shift_left(resize(s_g3, 17), 3) + shift_left(resize(s_g3, 17), 2);
                        z_i5 <= s_g5(12 downto 10); z_fr <= s_g5(9 downto 0);
                    when 33 =>
                        r_e_a <= z_eR(12 downto 5); z_iR <= z_eR(14 downto 13);
                        if z_b > 64 then z_b <= z_b - 64; else z_b <= (others => '0'); end if;
                        if z_pp < 4096 then      -- the exponent's complement 73728 - eT starts at 73728 - 19005 / - 40960
                            z_pq <= z_pp(11 downto 0); z_kq <= to_unsigned(1372, 11); z_ebn <= to_unsigned(54723, 17);
                        else
                            z_pq <= resize(z_pp - 4096, 12); z_kq <= to_unsigned(1700, 11); z_ebn <= to_unsigned(32768, 17);
                        end if;
                        z_w1 <= to_unsigned(192, 10) + resize(shift_right(resize(z_e8, 12) + shift_left(resize(z_e8, 12), 1), 2), 10);
                        if z_fr > 195 then z_fq <= z_fr - 195; else z_fq <= (others => '0'); end if;
                    when 34 =>
                        z_ma <= resize(shift_right(z_b, 1), 12) + resize(shift_right(z_b, 3), 12);
                        z_mb <= resize(shift_right(z_b, 4), 12) + resize(shift_right(z_b, 6), 12);
                    when 35 =>
                        case to_integer(z_iR) is
                            when 0      => z_AR <= resize(r_e_q, 18);
                            when 1      => z_AR <= shift_left(resize(r_e_q, 18), 1);
                            when others => z_AR <= shift_left(resize(r_e_q, 18), 2);
                        end case;
                        z_mc <= z_ma + z_mb;
                    when 36 =>
                        z_m12 <= z_mc + resize(shift_right(z_b, 7), 12);
                    when 37 =>
                        v_s := signed(r_s_q(11 downto 0));
                        if v_s(11) = '1' then z_n11 <= '1'; v_s := -v_s; else z_n11 <= '0'; end if;
                        z_a11 <= unsigned(v_s(10 downto 0));
                        if s_xs = 4 then
                            c_ux0 <= -signed(resize(shift_left(resize(z_AR, 28), 8) - shift_left(resize(z_AR, 28), 6), 28));
                        else
                            c_ux0 <= -signed(shift_left(resize(z_AR, 28), 8));
                        end if;
                    when 38 =>
                        z_la <= shift_left(resize(z_a11, 16), 4) + resize(z_a11, 16);
                        z_lb <= resize(shift_right(z_a11, 1), 16) + resize(shift_right(z_a11, 3), 16);
                    when 39 =>
                        v_s := signed(r_s_q(11 downto 0));
                        if v_s(11) = '1' then z_n27 <= '1'; v_s := -v_s; else z_n27 <= '0'; end if;
                        z_a27 <= unsigned(v_s(10 downto 0));
                        if z_n11 = '1' then z_lfo <= -signed(resize(z_la + z_lb, 18)); else z_lfo <= signed(resize(z_la + z_lb, 18)); end if;
                    when 40 =>
                        c_pf <= z_ph + unsigned(z_lfo(16 downto 0));
                        z_mla <= resize(shift_right(z_a27, 3), 9) + resize(shift_right(z_a27, 4), 9);
                    when 41 =>
                        if z_n27 = '1' then z_mL <= -signed(resize(z_mla + resize(shift_right(z_a27, 6), 9), 12));
                        else z_mL <= signed(resize(z_mla + resize(shift_right(z_a27, 6), 9), 12)); end if;
                    when 42 =>
                        if z_bs = '1' then v_m := -signed(resize(z_m12, 14)); else v_m := signed(resize(z_m12, 14)); end if;
                        z_m <= v_m + resize(z_mL, 14);
                    when 43 =>
                        if z_m(13) = '1' then z_mneg <= '1'; v_m := -z_m; else z_mneg <= '0'; v_m := z_m; end if;
                        z_ma2 <= unsigned(v_m(11 downto 0));
                    when 74 => z_ne <= z_ebn - resize(f_acc(24 downto 8), 17);
                    when 76 => r_e_a <= z_ne(12 downto 5); z_nI <= z_ne(15 downto 13);
                    when 78 => z_mt <= r_e_q;
                    when 79 =>
                        z_cmn <= shift_left(resize(f_cm8(s_xs), 16), to_integer(z_nI));
                        z_3n  <= shift_left(to_unsigned(3, 8), to_integer(z_nI));
                    when 98  => c_gxp <= f_acc(32 downto 15);
                    when 122 => c_gyl <= f_acc(33 downto 14);
                    when 146 => c_inc0 <= f_acc(31 downto 12);
                    when 147 => c_incmin <= shift_right(c_inc0, 3) + shift_right(c_inc0, 4);
                    when 148 => c_incmin <= c_incmin + shift_right(c_inc0, 6);
                    when 170 => z_wf <= f_acc(25 downto 14);
                    when 171 =>
                        if resize(z_w1, 12) > z_wf then z_w <= resize(z_w1, 12); else z_w <= z_wf; end if;
                    when 172 =>
                        c_dG  <= resize(z_e8, 12);
                        c_dB  <= shift_left(resize(z_e8, 12), 1);
                        c_dW  <= z_w;
                        c_dGW <= resize(z_e8, 12) + z_w;
                        c_dBW <= shift_left(resize(z_e8, 12), 1) + z_w;
                    when 194 => z_P <= f_acc(31 downto 12);
                    when 195 =>
                        if z_mneg = '1' then v_i := signed(resize(c_inc0, 22)) - signed(resize(z_P, 22));
                        else v_i := signed(resize(c_inc0, 22)) + signed(resize(z_P, 22)); end if;
                        z_ia0 <= v_i;
                    when 218 =>
                        if z_mneg = '1' then c_dinc <= signed(resize(f_acc(35 downto 14), 23));
                        else c_dinc <= -signed(resize(f_acc(35 downto 14), 23)); end if;
                    when 242 => c_gl <= f_acc(22 downto 14);
                    when 266 =>
                        if f_acc(19 downto 10) > 256 then z_t8 <= to_unsigned(256, 9);
                        else z_t8 <= f_acc(18 downto 10); end if;
                    when others => null;
                end case;
            end if;

            -- palette loop (zc 280 ..): per level read keyframes A and B, then per channel one multiply
            -- (B - A) x t8 and colour = A + product / 256; the word is written after the third channel
            pw_ld <= '0'; pw_we <= '0';
            if s_zone = '1' and s_zc = 280 then
                pw_run <= '1'; pw_t <= (others => '0'); pw_n <= (others => '0'); pw_c <= (others => '0');
            elsif pw_run = '1' then
                case to_integer(pw_t) is
                    when 0 => r_k_a(7) <= s_dac; r_k_a(6) <= '0'; r_k_a(5 downto 3) <= z_i5; r_k_a(2 downto 0) <= pw_n;
                    when 1 => r_k_a <= r_k_a + 8;
                    when 2 => if pw_c = 0 then pw_ka <= r_k_q; end if;
                    when 3 =>                       -- r_k_q = keyframe B
                        case to_integer(pw_c) is
                            when 0 => pw_d <= signed(resize(unsigned(r_k_q(31 downto 22)), 11)) - signed(resize(unsigned(pw_ka(31 downto 22)), 11));
                            when 1 => pw_d <= signed(resize(unsigned(r_k_q(21 downto 12)), 11)) - signed(resize(unsigned(pw_ka(21 downto 12)), 11));
                            when others => pw_d <= signed(resize(unsigned(r_k_q(11 downto 2)), 11)) - signed(resize(unsigned(pw_ka(11 downto 2)), 11));
                        end case;
                    when 5 => pw_ld <= '1';
                    when 23 =>
                        case to_integer(pw_c) is
                            when 0 =>
                                v_y := signed(resize(unsigned(pw_ka(31 downto 22)), 12)) + signed(f_acc(19 downto 8));
                                pw_word(31 downto 22) <= std_logic_vector(v_y(9 downto 0));
                            when 1 =>
                                v_y := signed(resize(unsigned(pw_ka(21 downto 12)), 12)) + signed(f_acc(19 downto 8));
                                pw_word(21 downto 12) <= std_logic_vector(v_y(9 downto 0));
                            when others =>
                                v_y := signed(resize(unsigned(pw_ka(11 downto 2)), 12)) + signed(f_acc(19 downto 8));
                                pw_word(11 downto 2) <= std_logic_vector(v_y(9 downto 0));
                        end case;
                    when others => null;
                end case;
                if pw_t = 24 then
                    pw_t <= (others => '0');
                    if pw_c = 2 then
                        pw_we <= '1'; pw_a <= resize(pw_n, 8);
                        pw_c <= "00";
                        if pw_n = 7 then pw_run <= '0'; end if;
                        pw_n <= pw_n + 1;
                    else
                        pw_c <= pw_c + 1;
                    end if;
                else
                    pw_t <= pw_t + 1;
                end if;
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
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- ((Y >> 2) - 16), averaged over 8-pixel cells, each cell slewed down the
    -- screen (one pole, 1/32 per line; the state lives in the cell store), then
    -- slewed across by two one-pole filters (1/8 per pixel) and scaled by the
    -- source gain (Osc at twice the camera's).  All three filters decay, so none
    -- needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(16 downto 0);
        variable v_p : unsigned(21 downto 0);
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
            s_lc    <= s_wcol(2 downto 0);
            s_lcell <= s_wcol(10 downto 3);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= s_wcol(10 downto 3) + 1;
            else
                s_wcol  <= (others => '0');
                s_lb_ra <= (others => '0');
            end if;
            -- 2: sum eight; at the eighth, take the cell's state (read a cell ahead) for the update
            s_f1 <= '0';
            if s_lav = '0' then
                s_cur <= unsigned(s_lb_q(11 downto 0));
            else
                if s_lc = 0 then s_acc <= resize(s_lin, 11); else s_acc <= s_acc + s_lin; end if;
                if s_lc = 7 then
                    s_sum8  <= s_acc + s_lin;
                    s_old   <= s_cur;
                    s_cur   <= unsigned(s_lb_q(11 downto 0));
                    s_fcell <= s_lcell;
                    s_f1    <= '1';
                end if;
            end if;
            -- 3: state += (cell average * 16 - state) / 32
            v_d := signed(resize(s_sum8 & '0', 13)) - signed(resize(s_old, 13));
            v_d := signed(resize(s_old, 13)) + shift_right(v_d, 5);
            s_new   <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);

            -- pixel side: the cell stream (the line above's state), two slews across, gain
            s_lv <= unsigned(s_lb_q(11 downto 0));
            v_e  := signed(resize(s_lv & "0000", 17)) - signed(resize(s_h1, 17));
            v_e  := signed(resize(s_h1, 17)) + shift_right(v_e, 3);
            s_h1 <= unsigned(v_e(15 downto 0));
            v_e  := signed(resize(s_h1, 17)) - signed(resize(s_h2, 17));
            v_e  := signed(resize(s_h2, 17)) + shift_right(v_e, 3);
            s_h2 <= unsigned(v_e(15 downto 0));
            s_la  <= s_h2(7 downto 4) * c_gl;
            s_lbh <= s_h2(11 downto 8) * c_gl;
            s_lch <= s_h2(15 downto 12) * c_gl;
            v_p := resize(s_la, 22) + shift_left(resize(s_lbh, 22), 4) + shift_left(resize(s_lch, 22), 8);   -- luma x 16 x gain
            if s_src = '1' then s_lterm <= v_p(21 downto 3); else s_lterm <= resize(v_p(21 downto 4), 19); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: the V term |y - 540| R, the ripple (sine ROM), the FM'd clock
    -- increment (clamped) and its quarter thresholds; loaded in the zone,
    -- latched by each line start for that line, advanced to the next line
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_a  : signed(27 downto 0);
        variable v_ia : signed(29 downto 0);
        variable v_rp : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 900 then
                e_uy <= -signed(shift_left(resize(z_AR, 28), 8));
                e_rp <= z_roll;
                e_ia <= shift_left(resize(z_ia0, 30), 8);
            elsif g_start = '1' then
                e_uy <= e_uy + signed(resize(c_gyl, 28));
                e_rp <= e_rp + resize(s_crt, 16);
                e_ia <= e_ia + resize(c_dinc, 30);
            end if;
            if z_sw = '1' then r_s_a <= z_sa; else r_s_a <= e_rp(15 downto 8); end if;
            n_sr  <= signed(r_s_q(11 downto 0));
            v_rp  := shift_left(resize(n_sr, 14), 1) - resize(shift_right(n_sr, 2), 14);
            n_rip <= v_rp;
            v_ia  := shift_right(e_ia, 8);
            if v_ia(29) = '1' or v_ia < signed(resize(c_incmin, 30)) then
                n_inc <= c_incmin;
            elsif v_ia > 1048575 then
                n_inc <= (others => '1');
            else
                n_inc <= unsigned(v_ia(19 downto 0));
            end if;
            -- latched for the line that starts (e_uy / e_rp / n_inc are this line's until the update above)
            if g_start = '1' then
                v_a  := abs(e_uy);
                l_hy <= unsigned(v_a(26 downto 8));
                l_pl <= c_pf + unsigned(resize(n_rip, 17));
                l_inc <= n_inc;
                l_t3 <= shift_right(n_inc, 2) + shift_right(n_inc, 1);
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the diamond's signed x term (Q8) and the clock phase (Q20) with its
    -- wrap flag (a clock edge between the previous pixel and this one)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ux <= c_ux0; b_ph <= (others => '0'); b_w <= '0'; b_first <= '1';
            elsif g_avid = '1' then
                v_n  := resize(b_ph, 21) + resize(l_inc, 21);
                b_ph <= v_n(19 downto 0); b_w <= v_n(20);
                b_ux <= b_ux + signed(resize(c_gxp, 28));
                b_first <= '0';
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a   : signed(27 downto 0);
        variable v_ds  : signed(16 downto 0);
        variable v_lev : unsigned(2 downto 0);
        variable v_v   : unsigned(2 downto 0);
        variable v_cn  : unsigned(3 downto 0);
        variable v_nh  : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: |x term| ; the clock edge's position inside the pixel (quarters) ; the shift-register tap
            --    fractions (clock phase short of each tap's fraction: one clock further back)
            v_a := abs(b_ux);
            p2_hx <= unsigned(v_a(26 downto 8));
            if b_ph >= l_t3 then p_dq(2) <= "11";
            elsif b_ph >= shift_right(l_inc, 1) then p_dq(2) <= "10";
            elsif b_ph >= shift_right(l_inc, 2) then p_dq(2) <= "01";
            else p_dq(2) <= "00"; end if;
            p_w(2) <= b_w; p_f(2) <= b_first;
            if b_ph(19 downto 12) < c_dG(7 downto 0) then p_cf(2)(0) <= '1'; else p_cf(2)(0) <= '0'; end if;
            if b_ph(19 downto 12) < c_dB(7 downto 0) then p_cf(2)(1) <= '1'; else p_cf(2)(1) <= '0'; end if;
            if b_ph(19 downto 12) < c_dW(7 downto 0) then p_cf(2)(2) <= '1'; else p_cf(2)(2) <= '0'; end if;
            if b_ph(19 downto 12) < c_dGW(7 downto 0) then p_cf(2)(3) <= '1'; else p_cf(2)(3) <= '0'; end if;
            if b_ph(19 downto 12) < c_dBW(7 downto 0) then p_cf(2)(4) <= '1'; else p_cf(2)(4) <= '0'; end if;
            for k in 3 to 7 loop
                p_w(k) <= p_w(k - 1); p_f(k) <= p_f(k - 1); p_cf(k) <= p_cf(k - 1);
            end loop;
            for k in 3 to 6 loop p_dq(k) <= p_dq(k - 1); end loop;
            -- 3: DSG3 combiners: Average (hx + hy), Maximum (compare)
            if p2_hx > l_hy then p3_cmp <= '1'; else p3_cmp <= '0'; end if;
            p3_avg <= resize(p2_hx + l_hy, 17);
            p3_hx <= p2_hx(15 downto 0); p3_hy <= l_hy(15 downto 0);
            -- 4: the selected output (S8) ; the Phase CV (line term + camera)
            if s_shape = '1' then
                if p3_cmp = '1' then p4_fig <= p3_hx & '0'; else p4_fig <= p3_hy & '0'; end if;
            else
                p4_fig <= p3_avg;
            end if;
            p4_cp <= resize(s_lterm, 17) + l_pl;
            -- 5: the fold argument s (mod one fold period)
            p5_s <= p4_fig + p4_cp;
            -- 6: the step from the previous pixel
            p6_ds <= p5_s - p6_s;
            p6_sp <= p6_s;
            p6_s  <= p5_s;
            -- 7: the field at the clock edge (dq quarters of a pixel before this one): s - ds/4, or the
            --    previous pixel + ds/2, + ds/4
            v_ds := signed(p6_ds);
            if p_f(6) = '1' or p_dq(6) = "00" then
                p7_se <= p6_s;
            elsif p_dq(6) = "01" then
                p7_se <= p6_s - unsigned(shift_right(v_ds, 2));
            elsif p_dq(6) = "10" then
                p7_se <= p6_sp + unsigned(shift_right(v_ds, 1));
            else
                p7_se <= p6_sp + unsigned(shift_right(v_ds, 2));
            end if;
            -- 8: Stairs fold + ADC ; the flip-flops (or the Counter) and the shift register, clocked by the VCO
            v_lev := p7_se(15 downto 13);
            if p7_se(16) = '1' then v_v := not v_lev; else v_v := v_lev; end if;
            if p_f(7) = '1' then
                if s_cnt = '1' then
                    c_cnt <= (others => '0'); c_mp <= v_v(2); v_nh := "000";
                else
                    v_nh := v_v;
                end if;
                for k in 0 to 15 loop sr(k) <= v_nh; end loop;
            elsif p_w(7) = '1' then
                if s_cnt = '1' then
                    if v_v(2) /= c_mp then v_cn := (others => '0');
                    elsif v_v(2) = '1' then v_cn := c_cnt + 1;
                    else v_cn := c_cnt - 1; end if;
                    c_cnt <= v_cn; c_mp <= v_v(2); v_nh := v_cn(3 downto 1);
                else
                    v_nh := v_v;
                end if;
                sr(0) <= v_nh;
                for k in 1 to 15 loop sr(k) <= sr(k - 1); end loop;
            end if;
            -- shift-register taps: whole clocks back, one more when the clock phase is short of the fraction
            if p_cf(7)(0) = '1' then p8_oG  <= c_dG(11 downto 8) + 1;  else p8_oG  <= c_dG(11 downto 8);  end if;
            if p_cf(7)(1) = '1' then p8_oB  <= c_dB(11 downto 8) + 1;  else p8_oB  <= c_dB(11 downto 8);  end if;
            if p_cf(7)(2) = '1' then p8_oW  <= c_dW(11 downto 8) + 1;  else p8_oW  <= c_dW(11 downto 8);  end if;
            if p_cf(7)(3) = '1' then p8_oGW <= c_dGW(11 downto 8) + 1; else p8_oGW <= c_dGW(11 downto 8); end if;
            if p_cf(7)(4) = '1' then p8_oBW <= c_dBW(11 downto 8) + 1; else p8_oBW <= c_dBW(11 downto 8); end if;
            -- 9: the taps
            p9_r0 <= sr(0)(2);
            p9_g  <= sr(to_integer(p8_oG))(1);
            p9_b  <= sr(to_integer(p8_oB))(0);
            p9_rw <= sr(to_integer(p8_oW))(2);
            p9_gw <= sr(to_integer(p8_oGW))(1);
            p9_bw <= sr(to_integer(p8_oBW))(0);
            -- 10: Multi Gate (S7) ; 11: palette address
            if s_gate = '1' then
                p10_o <= (p9_r0 xor p9_rw) & (p9_g xor p9_gw) & (p9_b xor p9_bw);
            else
                p10_o <= p9_r0 & p9_g & p9_b;
            end if;
            r_pr_a <= resize(unsigned(p10_o), 8);
            -- 12: palette RAM ; 13: fabric copy
            p13_y <= unsigned(r_pr_q(31 downto 22)); p13_u <= unsigned(r_pr_q(21 downto 12)); p13_v <= unsigned(r_pr_q(11 downto 2));
            -- 14: black off the generated avid
            if s_gav_sr(11) = '0' then
                s_y14 <= C_KY; s_u14 <= C_KU; s_v14 <= C_KV;
            else
                s_y14 <= p13_y; s_u14 <= p13_u; s_v14 <= p13_v;
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

    -- the two line stores of the vertical mix: line n - 1 (vy / vc, written from this line) and line n - 2
    -- (vy2 / vc2, written from vy / vc's read data)
    p_vs : process(clk)
        variable v_s : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(15) = '1' then v_k17 <= v_k17 + 1; else v_k17 <= (others => '0'); end if;   -- column of the stage-17 pixel
            if s_gav_sr(17) = '1' then v_k19 <= v_k19 + 1; else v_k19 <= (others => '0'); end if;   -- column of the stage-19 pixel
            v_qy  <= vy(to_integer(v_k17(10 downto 1)));
            v_qc  <= vc(to_integer(v_k17(10 downto 2)));
            v_qy2 <= vy2(to_integer(v_k17(10 downto 1)));
            v_qc2 <= vc2(to_integer(v_k17(10 downto 2)));
            v_fy <= v_qy; v_fc <= v_qc; v_fy2 <= v_qy2; v_fc2 <= v_qc2;
            v_py <= s_y19; v_pu <= s_u19; v_pv <= s_v19;
            v_wye <= s_gav_sr(17) and v_k19(0);
            v_wya <= v_k19(10 downto 1);
            v_s := resize(s_y19, 11) + resize(v_py, 11); v_wyd <= std_logic_vector(v_s(10 downto 3));
            v_wyd2 <= v_fy;
            if s_gav_sr(17) = '1' and v_k19(1 downto 0) = "01" then v_wce <= '1'; else v_wce <= '0'; end if;
            v_wca <= v_k19(10 downto 2);
            v_s := resize(s_u19, 11) + resize(v_pu, 11); v_wcd(15 downto 8) <= std_logic_vector(v_s(10 downto 3));
            v_s := resize(s_v19, 11) + resize(v_pv, 11); v_wcd(7 downto 0) <= std_logic_vector(v_s(10 downto 3));
            v_wcd2 <= v_fc;
            if v_wye = '1' then vy(to_integer(v_wya)) <= v_wyd; end if;
            if v_wce = '1' then vc(to_integer(v_wca)) <= v_wcd; end if;
            if v_wye = '1' then vy2(to_integer(v_wya)) <= v_wyd2; end if;
            if v_wce = '1' then vc2(to_integer(v_wca)) <= v_wcd2; end if;
        end if;
    end process p_vs;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the rack's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16, a vertical [1 2 1]/4 with the two lines above
    -- (no single-row lines), and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_mx : unsigned(11 downto 0);
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
            -- 20: vertical [1 2 1]/4 with the two lines above (stored as 4x 8-bit pair averages)
            v_mx := resize(s_y19, 12) + resize(unsigned(v_fy) & "000", 12) + resize(unsigned(v_fy2) & "00", 12) + 6;
            s_y20 <= v_mx(11 downto 2);
            v_mx := resize(s_u19, 12) + resize(unsigned(v_fc(15 downto 8)) & "000", 12) + resize(unsigned(v_fc2(15 downto 8)) & "00", 12) + 6;
            s_u20 <= v_mx(11 downto 2);
            v_mx := resize(s_v19, 12) + resize(unsigned(v_fc(7 downto 0)) & "000", 12) + resize(unsigned(v_fc2(7 downto 0)) & "00", 12) + 6;
            s_v20 <= v_mx(11 downto 2);
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

end architecture portcullis;
