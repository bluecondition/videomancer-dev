-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Chladni v0.1.0 -- Patchbay: standing waves on a plate.
--
-- The patch:
--   H-locked sine VCO "N" (n half-cycles across the plate) --\
--   V-locked sine VCO "M" (m half-cycles down the plate)  ---- multiplier 1 --\
--   H-locked sine VCO "M" ---------------------------------\                   mixer (cos / sin of the Mix angle)
--   V-locked sine VCO "N" ---------------------------------- multiplier 2 --/        |
--   camera luma (slewed) or the Osc lattice ------------------------------------> + F
--   F -> window comparator at F = 0  -> the sand lines (keyed over everything)
--   |F| -> colouriser (one palette cycle per period of |F|, the lobe's sign picks the palette half)
--   plate key: a bordered square (or the whole screen); black outside
--
--   F = cos(th) sin(n X + a) sin(m Y) + sin(th) sin(m X) sin(n Y + d),  X, Y = pi * (position / plate side)
--
--   K1  Mode N     VCO N: 1 .. 9 half-cycles, detented at the integers, glided
--   K2  Mode M     VCO M: 1 .. 9
--   K3  Mix        mixer angle: plate B alone (-90) .. the minus figure (-45, detent) .. plate A alone (0, detent)
--                  .. the plus figure (+45)
--   K4  Speed      bipolar phase drift of VCO N (H across, V down at a third of the rate, opposite) and the
--                  colouriser's band flow; centre = locked (the drift phases snap to zero): the still classic figure
--   K5  Sand       window comparator width (tracks the tone generator, so the lines keep their weight)
--   K6  Bands      colouriser gain: 0.4 .. 3.4 palette cycles per unit of |F|
--   S7  Plate      Square (bordered square plate) / Screen (the field fills the picture)
--   S8  Look       Bands / Sand (pale sand on black, a dust ring per colouriser period)
--   S9  Twin       one plate / two mirrored plates
--   S10 Colours    Ember / Verdigris
--   S11 Source     Video (slewed camera luma adds to F) / Osc (a drifting triangle lattice, 3.5x the level)
--   P12 RESONANCE  a tone generator stepped up the plate: both modes -2 .. +7 (snapping, with a short glide)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared knob glide), a per-line engine kicked at
-- each line start (computes the NEXT line's two V-oscillator values times the mixer gains), two 32-bit
-- phase DDAs along the line, an 18-stage pixel path.  Virtual 1080-line coordinates.  Generated avid;
-- output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16, then a vertical
-- [1 1]/2 with the line above) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.chladni_pkg.all;
use work.all;

architecture chladni of program_top is
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
    -- quarter-wave table index of an 11-bit phase (2048 per cycle); bit 10 of the phase is the sign
    function f_fold(p : unsigned(10 downto 0)) return unsigned is
    begin
        if p(9) = '1' then return not p(8 downto 0); else return p(8 downto 0); end if;
    end function;
    -- detented mode map on a glided Q3 knob: 1.0 .. 9.0 in Q8, flat for 32 codes either side of an integer
    function f_det(g : unsigned(12 downto 0)) return unsigned is
        variable v_t : unsigned(9 downto 0);
        variable v_r : unsigned(11 downto 0);
    begin
        if g(9 downto 8) = "00" then v_t := (others => '0');
        elsif g(9 downto 8) = "11" then v_t := to_unsigned(256, 10);
        else v_t := shift_right(g(9 downto 0) - 256, 1); end if;
        v_r := shift_left(resize(g(12 downto 10), 12) + 1, 8) + resize(v_t, 12);
        return v_r;
    end function;
    -- resonance step of the slider: 0 .. 9
    function f_ridx(p : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(13 downto 0);
    begin
        v := shift_left(resize(p, 14), 3) + shift_left(resize(p, 14), 1);
        return v(13 downto 10);
    end function;
    -- luma cell store: one 12-bit vertical-slew state per 8-pixel cell (read a cell ahead of the write)
    type t_lb is array (0 to 511) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(8 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(15 downto 0) := x"0000";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(15 downto 0) := x"0000";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav, s_f1, s_f2 : std_logic := '0';
    signal s_lc    : unsigned(2 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(8 downto 0) := (others => '0');
    signal s_acc, s_sum8 : unsigned(10 downto 0) := (others => '0');
    signal s_cur, s_old, s_new : unsigned(11 downto 0) := (others => '0');
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_h1, s_h2 : unsigned(15 downto 0) := (others => '0');
    signal s_la, s_lbh : unsigned(16 downto 0) := (others => '0');
    signal s_lterm : unsigned(14 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_sn_a, r_sm_a, r_sy_a : unsigned(8 downto 0) := (others => '0');
    signal r_sn_q, r_sm_q, r_sy_q : unsigned(15 downto 0) := (others => '0');
    signal r_pal_a : unsigned(9 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    -- output filter stores: the two 8-pixel delay lines of the boxes (16-deep rings, read 6 behind the write
    -- plus a fabric register = 8 pixels), and the line above for the vertical mix (pair averages, 8 bits:
    -- Y per 2 pixels, U and V per 4 pixels; read two columns ahead of the write)
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
    signal v_k21, v_k23 : unsigned(10 downto 0) := (others => '0');
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
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(256 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(320 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(660, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_scr, s_look, s_twin, s_pal, s_src : std_logic := '0';
    signal s_p12 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal z_iu, z_id : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal s_ri : unsigned(3 downto 0) := to_unsigned(2, 4);          -- resonance step 0 .. 9 (= -2 .. +7)
    signal s_rq : unsigned(11 downto 0) := to_unsigned(512, 12);      -- glided, Q8

    -- per-field values (phases: 2^26 per cycle)
    signal z_n0, z_m0 : unsigned(11 downto 0) := to_unsigned(768, 12);
    signal z_ne, z_me : signed(13 downto 0) := (others => '0');
    signal z_mec  : unsigned(12 downto 0) := to_unsigned(1280, 13);
    signal c_ne   : unsigned(12 downto 0) := to_unsigned(768, 13);    -- mode N, Q8
    signal c_me   : unsigned(12 downto 0) := to_unsigned(1280, 13);   -- mode M, Q8
    signal z_th   : signed(10 downto 0) := to_signed(-256, 11);       -- mixer angle, 2048 per turn
    signal c_gq   : unsigned(9 downto 0) := to_unsigned(402, 10);     -- colouriser gain, Q8 periods per unit F
    signal z_wk   : unsigned(10 downto 0) := to_unsigned(612, 11);
    signal z_trk  : unsigned(8 downto 0) := to_unsigned(128, 9);
    signal z_wt   : unsigned(14 downto 0) := (others => '0');
    signal c_c12, c_s12 : unsigned(11 downto 0) := (others => '0');
    signal c_cneg, c_sneg : std_logic := '0';
    signal c_gca, c_gcb : unsigned(13 downto 0) := (others => '0');   -- gain x |cos|, gain x |sin| (x 4095)
    signal c_wb   : unsigned(13 downto 0) := to_unsigned(600, 14);    -- sand half-window, phase units (4096 / period)
    signal z_p    : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal c_p    : unsigned(3 downto 0) := to_unsigned(9, 4);        -- msb of the window
    signal c_minv : unsigned(4 downto 0) := to_unsigned(24, 5);       -- 4096 / normalised window
    signal c_gl   : unsigned(12 downto 0) := (others => '0');         -- source gain
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_a, c_d : unsigned(25 downto 0) := (others => '0');       -- drift phases
    signal c_flq  : unsigned(23 downto 0) := (others => '0');         -- band flow, Q12.12 phase units
    signal z_cxs  : unsigned(14 downto 0) := to_unsigned(8388, 15);   -- phase per pixel per mode, x 64
    signal z_cyy  : unsigned(15 downto 0) := to_unsigned(8388, 16);   -- phase per field line per mode, x 64
    signal c_incn, c_incm : unsigned(21 downto 0) := (others => '0'); -- phase per pixel
    signal c_dpn, c_dpm : unsigned(22 downto 0) := (others => '0');   -- phase per field line
    signal c_pxn0, c_pxm0, c_pyn0, c_pym0 : unsigned(25 downto 0) := (others => '0');
    -- layout (h units across, virtual lines down)
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal c_xl   : unsigned(11 downto 0) := to_unsigned(920, 12);
    signal c_xr, c_xri : unsigned(11 downto 0) := to_unsigned(4095, 12);
    signal c_xli  : unsigned(11 downto 0) := to_unsigned(940, 12);
    signal c_yt   : unsigned(10 downto 0) := to_unsigned(40, 11);
    signal c_yb   : unsigned(10 downto 0) := to_unsigned(1040, 11);
    signal c_yti  : unsigned(10 downto 0) := to_unsigned(50, 11);
    signal c_ybi  : unsigned(10 downto 0) := to_unsigned(1030, 11);
    signal c_cx   : unsigned(12 downto 0) := to_unsigned(4194, 13);
    signal c_cy   : unsigned(13 downto 0) := to_unsigned(8388, 14);
    signal c_cyt  : unsigned(15 downto 0) := to_unsigned(5243, 16);   -- Cy x plate top / 64
    signal c_il   : unsigned(8 downto 0) := to_unsigned(452, 9);      -- |pixel index of the plate's left edge|
    signal c_ilneg : std_logic := '0';
    signal c_rim  : std_logic_vector(31 downto 0) := C_CH_RIM(0);
    constant C_KY : unsigned(9 downto 0) := unsigned(C_CH_K(31 downto 22));
    constant C_KU : unsigned(9 downto 0) := unsigned(C_CH_K(21 downto 12));
    constant C_KV : unsigned(9 downto 0) := unsigned(C_CH_K(11 downto 2));

    -- field serial multiply-accumulate: acc = preload +- a x b (mod 2^32)
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(31 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_pl  : unsigned(25 downto 0) := (others => '0');
    signal z_ng  : std_logic := '0';
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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_pyn, e_pym : unsigned(25 downto 0) := (others => '0');
    signal e_s1, e_s2 : std_logic := '0';
    signal e_q1, e_q2 : unsigned(11 downto 0) := (others => '0');
    signal m_a    : unsigned(25 downto 0) := (others => '0');
    signal m_b    : unsigned(11 downto 0) := (others => '0');
    signal m_acc  : unsigned(25 downto 0) := (others => '0');
    signal n_a, n_b : unsigned(13 downto 0) := (others => '0');
    signal n_as, n_bs, n_yin, n_yrim : std_logic := '0';

    -- beam
    signal b_x, b_x1 : unsigned(11 downto 0) := (others => '0');
    signal b_pn, b_pm : unsigned(25 downto 0) := (others => '0');
    signal b_dn  : std_logic := '0';
    signal l_a, l_b : unsigned(13 downto 0) := (others => '0');
    signal l_as, l_bs, l_yin, l_yrim : std_logic := '0';

    -- pixel path
    signal p2_ns, p2_ms, p3_ns, p3_ms, p4_ns, p4_ms, p5_ns, p5_ms, p6_ns, p6_ms : std_logic := '0';
    signal p2_xf : unsigned(11 downto 0) := (others => '0');
    signal p4_sn, p4_sm : unsigned(9 downto 0) := (others => '0');
    signal p5_nl, p5_nh, p5_ml, p5_mh : unsigned(16 downto 0) := (others => '0');
    signal p6_pn, p6_pm : unsigned(13 downto 0) := (others => '0');
    signal p7_n, p7_m : signed(14 downto 0) := (others => '0');
    signal p8_f  : signed(15 downto 0) := (others => '0');
    signal p9_f  : signed(17 downto 0) := (others => '0');
    signal p10_af : unsigned(16 downto 0) := (others => '0');
    signal p10_sg, p11_sg, p12_sg, p13_sg, p14_sg : std_logic := '0';
    signal p11_ph, p12_ph, p13_ph, p14_ph : unsigned(6 downto 0) := (others => '0');
    signal p11_d : signed(17 downto 0) := (others => '0');
    signal p12_dn : unsigned(7 downto 0) := (others => '0');
    signal p13_t : unsigned(12 downto 0) := (others => '0');
    signal p14_t5 : unsigned(4 downto 0) := (others => '0');
    signal p17_c : std_logic_vector(31 downto 0) := (others => '0');
    signal p_in, p_rim : std_logic_vector(17 downto 3) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
    -- Luma cell store (the read runs a cell ahead of the write: no collisions) and ROMs
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
            r_sn_q  <= C_CH_SIN10(to_integer(r_sn_a));
            r_sm_q  <= C_CH_SIN10(to_integer(r_sm_a));
            r_sy_q  <= C_CH_SIN12(to_integer(r_sy_a));
            r_pal_q <= C_CH_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_w  : unsigned(13 downto 0);
        variable v_d  : signed(12 downto 0);
        variable v_e  : signed(12 downto 0);
        variable v_r  : signed(13 downto 0);
        variable v_j  : unsigned(2 downto 0);
        variable v_sd : std_logic;
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
            s_p12 <= unsigned(registers_in(7));

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_scr  <= registers_in(6)(0);
                s_look <= registers_in(6)(1);
                s_twin <= registers_in(6)(2);
                s_pal  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
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
                            when others => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when others => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial multiply-accumulate slots: slot k loads at zc = 32 + 24 k, 16 iterations, result stable
            -- from load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands,
            -- the preload and the sign are pre-muxed at 31 + 24 k, so no zc decode sits on the multiplier's path.
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
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_pl <= (others => '0'); z_ng <= '0';
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(c_gq, 32);   z_fb <= resize(c_c12, 16);          -- gain x |cos|
                    when 1  => z_fa <= resize(c_gq, 32);   z_fb <= resize(c_s12, 16);          -- gain x |sin|
                    when 2  => z_fa <= resize(z_wk, 32);   z_fb <= resize(z_trk, 16);          -- sand x tracking
                    when 3  => z_fa <= resize(z_wt, 32);   z_fb <= resize(c_gq, 16);           -- x gain
                    when 4  => z_fa <= resize(s_vm, 32);   z_fb <= resize(s_vm, 16);           -- speed^2
                    when 5  => z_fa <= resize(c_vsq, 32);  z_fb <= to_unsigned(3, 16);         -- drift, VCO N across
                               z_pl <= c_a; z_ng <= s_vneg;
                    when 6  => z_fa <= resize(c_vsq, 32);  z_fb <= to_unsigned(1, 16);         -- drift, VCO N down
                               z_pl <= c_d; z_ng <= not s_vneg;
                    when 7  => z_fa <= resize(c_vsq, 32);  z_fb <= to_unsigned(1, 16);         -- band flow
                               z_pl <= resize(c_flq, 26); z_ng <= s_vneg;
                    when 8  => z_fa <= resize(c_ne, 32);   z_fb <= resize(z_cxs, 16);          -- phase per pixel
                    when 9  => z_fa <= resize(c_me, 32);   z_fb <= resize(z_cxs, 16);
                    when 10 => z_fa <= resize(c_cy, 32);   z_fb <= resize(s_ys, 16);           -- line step per mode
                    when 11 => z_fa <= resize(c_ne, 32);   z_fb <= z_cyy;                      -- phase per field line
                    when 12 => z_fa <= resize(c_me, 32);   z_fb <= z_cyy;
                    when 13 => z_fa <= resize(c_incn, 32); z_fb <= resize(c_il, 16);           -- phase at the first pixel
                               z_pl <= c_a; z_ng <= not c_ilneg;
                    when 14 => z_fa <= resize(c_incm, 32); z_fb <= resize(c_il, 16);
                               z_ng <= not c_ilneg;
                    when 15 => z_fa <= resize(c_ne, 32);   z_fb <= c_cyt;                      -- phase at line 0
                               z_pl <= c_d; z_ng <= '1';
                    when 16 => z_fa <= resize(c_me, 32);   z_fb <= c_cyt;
                               z_ng <= '1';
                    when others =>                                                              -- source gain
                        z_fa <= resize(c_gq, 32);
                        if s_src = '1' then z_fb <= to_unsigned(77, 16); else z_fb <= to_unsigned(11, 16); end if;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= resize(z_pl, 32);
                if z_ng = '1' then f_a <= (not z_fa) + 1; else f_a <= z_fa; end if;
                f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- layout
                    when 20 =>
                        if s_xs = 4 then v_sd := '1'; else v_sd := '0'; end if;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                        c_rim <= C_CH_RIM(to_integer(unsigned'(s_look & s_pal)));
                        if s_scr = '1' then
                            c_xl <= resize(s_x0, 12); c_xli <= resize(s_x0, 12);
                            c_xr <= to_unsigned(4095, 12); c_xri <= to_unsigned(4095, 12);
                            c_yt <= to_unsigned(0, 11); c_yti <= to_unsigned(0, 11);
                            c_yb <= to_unsigned(1080, 11); c_ybi <= to_unsigned(1080, 11);
                            c_cy <= to_unsigned(7767, 14); c_cyt <= to_unsigned(0, 16);
                            c_il <= to_unsigned(8, 9); c_ilneg <= '1';
                            if s_twin = '1' then
                                if v_sd = '1' then c_cx <= to_unsigned(5825, 13); else c_cx <= to_unsigned(4369, 13); end if;
                            else
                                if v_sd = '1' then c_cx <= to_unsigned(2913, 13); else c_cx <= to_unsigned(2185, 13); end if;
                            end if;
                        elsif s_twin = '1' then
                            c_ilneg <= '0';
                            if v_sd = '1' then
                                c_xl <= to_unsigned(512, 12); c_xli <= to_unsigned(532, 12);
                                c_xr <= to_unsigned(1872, 12); c_xri <= to_unsigned(1852, 12);
                                c_yt <= to_unsigned(200, 11); c_yti <= to_unsigned(210, 11);
                                c_yb <= to_unsigned(880, 11); c_ybi <= to_unsigned(870, 11);
                                c_cx <= to_unsigned(6168, 13); c_cy <= to_unsigned(12336, 14); c_cyt <= to_unsigned(38550, 16);
                                c_il <= to_unsigned(0, 9);
                            else
                                c_xl <= to_unsigned(80, 12); c_xli <= to_unsigned(100, 12);
                                c_xr <= to_unsigned(1880, 12); c_xri <= to_unsigned(1860, 12);
                                c_yt <= to_unsigned(90, 11); c_yti <= to_unsigned(100, 11);
                                c_yb <= to_unsigned(990, 11); c_ybi <= to_unsigned(980, 11);
                                c_cx <= to_unsigned(4660, 13); c_cy <= to_unsigned(9320, 14); c_cyt <= to_unsigned(13106, 16);
                                if s_xs = 3 then c_il <= to_unsigned(19, 9); else c_il <= to_unsigned(32, 9); end if;
                            end if;
                        else
                            c_ilneg <= '0';
                            c_xl <= to_unsigned(920, 12); c_xli <= to_unsigned(940, 12);
                            c_xr <= to_unsigned(4095, 12); c_xri <= to_unsigned(4095, 12);
                            c_yt <= to_unsigned(40, 11); c_yti <= to_unsigned(50, 11);
                            c_yb <= to_unsigned(1040, 11); c_ybi <= to_unsigned(1030, 11);
                            c_cx <= to_unsigned(4194, 13); c_cy <= to_unsigned(8388, 14); c_cyt <= to_unsigned(5243, 16);
                            if v_sd = '1' then c_il <= to_unsigned(102, 9);
                            elsif s_xs = 3 then c_il <= to_unsigned(299, 9);
                            else c_il <= to_unsigned(452, 9); end if;
                        end if;
                    when 21 =>
                        case to_integer(s_xs) is
                            when 4      => z_cxs <= shift_left(resize(c_cx, 15), 2);
                            when 3      => z_cxs <= shift_left(resize(c_cx, 15), 1) + resize(c_cx, 15);
                            when others => z_cxs <= shift_left(resize(c_cx, 15), 1);
                        end case;
                        -- resonance step with +-12 codes of hysteresis
                        if s_p12 > 12 then z_iu <= f_ridx(s_p12 - 12); else z_iu <= (others => '0'); end if;
                        if s_p12 < 1011 then z_id <= f_ridx(s_p12 + 12); else z_id <= to_unsigned(9, 4); end if;
                    when 22 =>
                        if z_iu > s_ri then s_ri <= z_iu; elsif z_id < s_ri then s_ri <= z_id; end if;
                    when 23 =>                    -- the step glides: 1/8 of the gap per field, landing exactly
                        v_d := signed(resize(s_ri & x"00", 13)) - signed(resize(s_rq, 13));
                        v_e := shift_right(v_d, 3);
                        if v_d > 0 and v_e = 0 then v_e := to_signed(1, 13); end if;
                        v_e := signed(resize(s_rq, 13)) + v_e;
                        s_rq <= unsigned(v_e(11 downto 0));
                    when 26 =>
                        z_n0 <= f_det(s_g1); z_m0 <= f_det(s_g2);
                        -- mixer angle: -512 (k = 0) .. -256 (256 .. 383) .. 0 (640 .. 767) .. +255 (1023)
                        v_k := s_g3(12 downto 3);
                        if v_k < 256 then z_th <= signed(resize(v_k, 11)) - 512;
                        elsif v_k < 384 then z_th <= to_signed(-256, 11);
                        elsif v_k < 640 then z_th <= signed(resize(v_k, 11)) - 640;
                        elsif v_k < 768 then z_th <= (others => '0');
                        else z_th <= signed(resize(v_k, 11)) - 768; end if;
                        -- colouriser gain 102 + 3 k / 4; sand k + 100
                        v_w := shift_left(resize(s_g6(12 downto 3), 14), 1) + resize(s_g6(12 downto 3), 14);
                        c_gq <= resize(v_w(13 downto 2), 10) + 102;
                        z_wk <= resize(s_g5(12 downto 3), 11) + 100;
                    when 27 =>
                        z_ne <= signed(resize(z_n0, 14)) + signed(resize(s_rq, 14)) - 512;
                        z_me <= signed(resize(z_m0, 14)) + signed(resize(s_rq, 14)) - 512;
                    when 28 =>
                        if z_ne < 256 then c_ne <= to_unsigned(256, 13); else c_ne <= unsigned(z_ne(12 downto 0)); end if;
                        if z_me < 256 then z_mec <= to_unsigned(256, 13); else z_mec <= unsigned(z_me(12 downto 0)); end if;
                    when 29 =>                    -- the two VCOs never sit on the same harmonic (F would vanish)
                        v_r := signed(resize(z_mec, 14)) - signed(resize(c_ne, 14));
                        if v_r > -256 and v_r < 256 then
                            if v_r >= 0 then c_me <= c_ne + 256; else c_me <= c_ne - 256; end if;
                        else
                            c_me <= z_mec;
                        end if;
                    when 30 =>
                        z_trk <= resize(shift_right(resize(c_ne, 14) + resize(c_me, 14), 5), 9) + 64;

                    when 50  => c_gca <= f_acc(21 downto 8);
                    when 74  => c_gcb <= f_acc(21 downto 8);
                    when 98  => z_wt  <= f_acc(18 downto 4);
                    when 122 =>
                        if f_acc(24 downto 15) = 0 then c_wb <= to_unsigned(16, 14);
                        else c_wb <= f_acc(24 downto 11); end if;
                    when 123 =>
                        z_p <= to_unsigned(4, 4);
                        for i in 5 to 13 loop
                            if c_wb(i) = '1' then z_p <= to_unsigned(i, 4); end if;
                        end loop;
                    when 124 =>
                        c_p <= z_p;
                        case to_integer(z_p) is
                            when 4      => v_j := c_wb(3 downto 1);
                            when 5      => v_j := c_wb(4 downto 2);
                            when 6      => v_j := c_wb(5 downto 3);
                            when 7      => v_j := c_wb(6 downto 4);
                            when 8      => v_j := c_wb(7 downto 5);
                            when 9      => v_j := c_wb(8 downto 6);
                            when 10     => v_j := c_wb(9 downto 7);
                            when 11     => v_j := c_wb(10 downto 8);
                            when 12     => v_j := c_wb(11 downto 9);
                            when others => v_j := c_wb(12 downto 10);
                        end case;
                        case to_integer(v_j) is
                            when 0      => c_minv <= to_unsigned(30, 5);
                            when 1      => c_minv <= to_unsigned(27, 5);
                            when 2      => c_minv <= to_unsigned(24, 5);
                            when 3      => c_minv <= to_unsigned(22, 5);
                            when 4      => c_minv <= to_unsigned(20, 5);
                            when 5      => c_minv <= to_unsigned(19, 5);
                            when 6      => c_minv <= to_unsigned(18, 5);
                            when others => c_minv <= to_unsigned(17, 5);
                        end case;
                    when 146 => c_vsq <= f_acc(17 downto 0);
                    -- drift; locked (Speed at the centre): the drift phases snap to zero
                    when 170 => if s_vm = 0 then c_a <= (others => '0'); else c_a <= f_acc(25 downto 0); end if;
                    when 194 => if s_vm = 0 then c_d <= (others => '0'); else c_d <= f_acc(25 downto 0); end if;
                    when 218 => c_flq  <= f_acc(23 downto 0);
                    when 242 => c_incn <= f_acc(27 downto 6);
                    when 266 => c_incm <= f_acc(27 downto 6);
                    when 290 => z_cyy  <= f_acc(23 downto 8);
                    when 314 => c_dpn  <= f_acc(28 downto 6);
                    when 338 => c_dpm  <= f_acc(28 downto 6);
                    when 362 => c_pxn0 <= f_acc(25 downto 0);
                    when 386 => c_pxm0 <= f_acc(25 downto 0);
                    when 410 => c_pyn0 <= f_acc(25 downto 0);
                    when 434 => c_pym0 <= f_acc(25 downto 0);
                    when 458 =>
                        if s_src = '1' then c_gl <= f_acc(16 downto 4); else c_gl <= f_acc(15 downto 3); end if;
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
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- ((Y >> 2) - 16), averaged over 8-pixel cells, each cell slewed down the
    -- screen (one pole, 1/32 per line; the state lives in the cell store), then
    -- slewed across by two one-pole filters (1/16 per pixel) and scaled by the
    -- source gain.  All three filters decay, so none needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(16 downto 0);
        variable v_p : unsigned(20 downto 0);
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

            -- 1: sample
            if s_src = '1' then
                s_lin <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lin <= v_l - 16; else s_lin <= (others => '0'); end if;
            end if;
            s_lav   <= data_in.avid;
            s_lc    <= s_wcol(2 downto 0);
            s_lcell <= resize(s_wcol(10 downto 3), 9);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= resize(s_wcol(10 downto 3), 9) + 1;
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
            v_e  := signed(resize(s_h1, 17)) + shift_right(v_e, 4);
            s_h1 <= unsigned(v_e(15 downto 0));
            v_e  := signed(resize(s_h1, 17)) - signed(resize(s_h2, 17));
            v_e  := signed(resize(s_h2, 17)) + shift_right(v_e, 4);
            s_h2 <= unsigned(v_e(15 downto 0));
            s_la  <= s_h2(11 downto 8) * c_gl;
            s_lbh <= s_h2(15 downto 12) * c_gl;
            v_p := resize(s_la, 21) + shift_left(resize(s_lbh, 21), 4);
            s_lterm <= v_p(20 downto 6);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line: the two V oscillators (phase DDAs, the sine
    -- table) times the mixer gains on one serial multiplier (16 iterations,
    -- result at load + 17), and the plate's row flags.  The same sine table
    -- gives the mixer's cos / sin early in the zone.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_p : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 480 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy  <= (others => '0');
                e_pyn <= c_pyn0; e_pym <= c_pym0;
                e_lc  <= (others => '0');
            elsif g_start = '1' then
                e_vy  <= e_vy + resize(s_ys, 19);
                e_pyn <= e_pyn + resize(c_dpn, 26); e_pym <= e_pym + resize(c_dpm, 26);
                e_lc  <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(24 downto 0) & '0';
            m_b <= '0' & m_b(11 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    r_sy_a <= f_fold(e_pym(25 downto 15)); e_s1 <= e_pym(25);
                    if s_scr = '1' then
                        n_yin <= '1'; n_yrim <= '0';
                    else
                        if e_vy(18 downto 8) >= c_yt and e_vy(18 downto 8) < c_yb then n_yin <= '1'; else n_yin <= '0'; end if;
                        if e_vy(18 downto 8) < c_yti or e_vy(18 downto 8) >= c_ybi then n_yrim <= '1'; else n_yrim <= '0'; end if;
                    end if;
                when 2 =>
                    r_sy_a <= f_fold(e_pyn(25 downto 15)); e_s2 <= e_pyn(25);
                when 3 => e_q1 <= r_sy_q(11 downto 0);
                when 4 => e_q2 <= r_sy_q(11 downto 0);
                when 5 =>
                    m_a <= resize(c_gca, 26); m_b <= e_q1; m_acc <= (others => '0');
                when 22 =>
                    n_a <= m_acc(25 downto 12); n_as <= e_s1 xor c_cneg;
                    m_a <= resize(c_gcb, 26); m_b <= e_q2; m_acc <= (others => '0');
                when 39 =>
                    n_b <= m_acc(25 downto 12); n_bs <= e_s2 xor c_sneg;
                when others => null;
            end case;

            -- the mixer's cos / sin (the line engine is idle in the zone until its kick)
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 27 => v_p := unsigned(z_th) + 512; r_sy_a <= f_fold(v_p); c_cneg <= v_p(10);
                    when 28 => v_p := unsigned(z_th);       r_sy_a <= f_fold(v_p); c_sneg <= v_p(10);
                    when 29 => c_c12 <= r_sy_q(11 downto 0);
                    when 30 => c_s12 <= r_sy_q(11 downto 0);
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the two H oscillators' phases (2^26 per cycle), stepped per pixel;
    -- Twin runs them back down from the screen centre (a mirror)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_x1 <= c_xb0 + s_xs;
                b_pn <= c_pxn0; b_pm <= c_pxm0;
                b_dn <= '0';
                l_a <= n_a; l_as <= n_as; l_b <= n_b; l_bs <= n_bs; l_yin <= n_yin; l_yrim <= n_yrim;
            elsif g_avid = '1' then
                b_x  <= b_x1;
                b_x1 <= b_x1 + s_xs;
                if b_dn = '1' then
                    b_pn <= b_pn - resize(c_incn, 26); b_pm <= b_pm - resize(c_incm, 26);
                else
                    b_pn <= b_pn + resize(c_incn, 26); b_pm <= b_pm + resize(c_incm, 26);
                end if;
                if b_x1 >= 1920 then b_dn <= s_twin; else b_dn <= '0'; end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p24 : unsigned(23 downto 0);
        variable v_f  : signed(17 downto 0);
        variable v_ph : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: sine table addresses; x folded about the screen centre for the plate test
            r_sn_a <= f_fold(b_pn(25 downto 15)); p2_ns <= b_pn(25);
            r_sm_a <= f_fold(b_pm(25 downto 15)); p2_ms <= b_pm(25);
            if b_x >= 3840 then p2_xf <= (others => '0');
            elsif b_x > 1920 then p2_xf <= to_unsigned(3840, 12) - b_x;
            else p2_xf <= b_x; end if;
            -- 3: table (EBR); plate key
            p3_ns <= p2_ns; p3_ms <= p2_ms;
            if s_scr = '1' then
                p_in(3) <= '1'; p_rim(3) <= '0';
            else
                if p2_xf >= c_xl and p2_xf <= c_xr then p_in(3) <= l_yin; else p_in(3) <= '0'; end if;
                if p2_xf < c_xli or p2_xf > c_xri then p_rim(3) <= '1'; else p_rim(3) <= l_yrim; end if;
            end if;
            p_in(17 downto 4)  <= p_in(16 downto 3);
            p_rim(17 downto 4) <= p_rim(16 downto 3);
            -- 4: fabric copy
            p4_sn <= r_sn_q(9 downto 0); p4_sm <= r_sm_q(9 downto 0);
            p4_ns <= p3_ns; p4_ms <= p3_ms;
            -- 5: the two multipliers (each H oscillator x its line value, operands split low / high)
            p5_nl <= p4_sn * l_a(6 downto 0);
            p5_nh <= p4_sn * l_a(13 downto 7);
            p5_ml <= p4_sm * l_b(6 downto 0);
            p5_mh <= p4_sm * l_b(13 downto 7);
            p5_ns <= p4_ns xor l_as; p5_ms <= p4_ms xor l_bs;
            -- 6: products
            v_p24 := shift_left(resize(p5_nh, 24), 7) + resize(p5_nl, 24);
            p6_pn <= v_p24(23 downto 10);
            v_p24 := shift_left(resize(p5_mh, 24), 7) + resize(p5_ml, 24);
            p6_pm <= v_p24(23 downto 10);
            p6_ns <= p5_ns; p6_ms <= p5_ms;
            -- 7: signs
            if p6_ns = '1' then p7_n <= -signed(resize(p6_pn, 15)); else p7_n <= signed(resize(p6_pn, 15)); end if;
            if p6_ms = '1' then p7_m <= -signed(resize(p6_pm, 15)); else p7_m <= signed(resize(p6_pm, 15)); end if;
            -- 8: the mixer
            p8_f <= resize(p7_n, 16) + resize(p7_m, 16);
            -- 9: + the source
            p9_f <= resize(p8_f, 18) + signed(resize(s_lterm, 18));
            -- 10: |F|, sign
            v_f := p9_f;
            if v_f < 0 then v_f := -v_f; p10_sg <= '1'; else p10_sg <= '0'; end if;
            p10_af <= unsigned(v_f(16 downto 0));
            -- 11: colouriser phase (- flow); distance inside the sand window
            v_ph := p10_af(11 downto 0) - c_flq(23 downto 12);
            p11_ph <= v_ph(11 downto 5); p11_sg <= p10_sg;
            p11_d <= signed(resize(c_wb, 18)) - signed(resize(p10_af, 18));
            -- 12: window distance normalised to 8 bits
            if p11_d <= 0 then
                p12_dn <= (others => '0');
            else
                case to_integer(c_p) is
                    when 4      => p12_dn <= unsigned(p11_d(4 downto 0)) & "000";
                    when 5      => p12_dn <= unsigned(p11_d(5 downto 0)) & "00";
                    when 6      => p12_dn <= unsigned(p11_d(6 downto 0)) & "0";
                    when 7      => p12_dn <= unsigned(p11_d(7 downto 0));
                    when 8      => p12_dn <= unsigned(p11_d(8 downto 1));
                    when 9      => p12_dn <= unsigned(p11_d(9 downto 2));
                    when 10     => p12_dn <= unsigned(p11_d(10 downto 3));
                    when 11     => p12_dn <= unsigned(p11_d(11 downto 4));
                    when 12     => p12_dn <= unsigned(p11_d(12 downto 5));
                    when others => p12_dn <= unsigned(p11_d(13 downto 6));
                end case;
            end if;
            p12_ph <= p11_ph; p12_sg <= p11_sg;
            -- 13: t = 32 (1 - |F| / window)
            p13_t <= p12_dn * c_minv;
            p13_ph <= p12_ph; p13_sg <= p12_sg;
            -- 14: clamp
            if p13_t(12) = '1' then p14_t5 <= (others => '1'); else p14_t5 <= p13_t(11 downto 7); end if;
            p14_ph <= p13_ph; p14_sg <= p13_sg;
            -- 15: colouriser address: inside the window the sand's cross-section (keyed over the bands),
            -- outside it the band of |F| (the lobe's sign picks the palette half; the Sand look has none)
            r_pal_a(8) <= s_pal;
            if p14_t5 /= 0 then
                r_pal_a(9) <= '1'; r_pal_a(7) <= '1'; r_pal_a(6) <= '0'; r_pal_a(5) <= s_look;
                r_pal_a(4 downto 0) <= p14_t5;
            else
                r_pal_a(9) <= s_look; r_pal_a(7) <= p14_sg and not s_look;
                r_pal_a(6 downto 0) <= p14_ph;
            end if;
            -- 16: colouriser (EBR); 17: fabric copy
            p17_c <= r_pal_q;
            -- 18: plate key; black outside the plate and off the generated avid
            if s_gav_sr(15) = '0' or p_in(17) = '0' then
                s_y18 <= C_KY; s_u18 <= C_KU; s_v18 <= C_KV;
            elsif p_rim(17) = '1' then
                s_y18 <= unsigned(c_rim(31 downto 22)); s_u18 <= unsigned(c_rim(21 downto 12)); s_v18 <= unsigned(c_rim(11 downto 2));
            else
                s_y18 <= unsigned(p17_c(31 downto 22)); s_u18 <= unsigned(p17_c(21 downto 12)); s_v18 <= unsigned(p17_c(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(15);
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
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y18) & std_logic_vector(s_u18) & std_logic_vector(s_v18);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y20) & std_logic_vector(s_u20) & std_logic_vector(s_v20);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    p_vs : process(clk)
        variable v_s : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(19) = '1' then v_k21 <= v_k21 + 1; else v_k21 <= (others => '0'); end if;   -- column of the stage-21 pixel
            if s_gav_sr(21) = '1' then v_k23 <= v_k23 + 1; else v_k23 <= (others => '0'); end if;   -- column of the stage-23 pixel
            v_qy <= vy(to_integer(v_k21(10 downto 1)));
            v_qc <= vc(to_integer(v_k21(10 downto 2)));
            v_fy <= v_qy; v_fc <= v_qc;
            v_py <= s_y23; v_pu <= s_u23; v_pv <= s_v23;
            v_wye <= s_gav_sr(21) and v_k23(0);
            v_wya <= v_k23(10 downto 1);
            v_s := resize(s_y23, 11) + resize(v_py, 11); v_wyd <= std_logic_vector(v_s(10 downto 3));
            if s_gav_sr(21) = '1' and v_k23(1 downto 0) = "01" then v_wce <= '1'; else v_wce <= '0'; end if;
            v_wca <= v_k23(10 downto 2);
            v_s := resize(s_u23, 11) + resize(v_pu, 11); v_wcd(15 downto 8) <= std_logic_vector(v_s(10 downto 3));
            v_s := resize(s_v23, 11) + resize(v_pv, 11); v_wcd(7 downto 0) <= std_logic_vector(v_s(10 downto 3));
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
            -- box A: stage 19 sums, stage 20 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y18 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u18 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v18 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(17) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y20 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u20 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v20 <= v_sc(13 downto 4);
            else
                s_y20 <= C_KY; s_u20 <= C_KU; s_v20 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(17);
            -- box B: stage 21 sums, stage 22 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y20 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u20 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v20 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y22 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u22 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v22 <= v_sc(13 downto 4);
            -- 23: luma 13/16
            if s_y22 > 64 then v_dl := s_y22 - 64; else v_dl := (others => '0'); end if;
            s_y23 <= s_y22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u23 <= s_u22; s_v23 <= s_v22;
            -- 24: vertical mix with the line above
            v_mx := resize(s_y23, 11) + resize(unsigned(v_fy) & "10", 11);             s_y24 <= v_mx(10 downto 1);
            v_mx := resize(s_u23, 11) + resize(unsigned(v_fc(15 downto 8)) & "10", 11); s_u24 <= v_mx(10 downto 1);
            v_mx := resize(s_v23, 11) + resize(unsigned(v_fc(7 downto 0)) & "10", 11);  s_v24 <= v_mx(10 downto 1);
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

end architecture chladni;
