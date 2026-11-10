-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Oilslick v0.1.0 -- Heirloom from the video clips (phase 3b), made from WIN_20231022_00_05_49 at 0.5 s.
--
-- The still: a camera-feedback frame of Ron's modular synth seen through a horizontal kaleidoscope (period
-- 942 px, dome axis x = 1020) with a vertical ramp that resets at a horizon (y = 600): red / orange domes with
-- magenta and peach arcs around blue cores and two navy "eye" blobs with cyan rings, black voids with wavy green
-- edges hanging between the domes, a purple sky at the top, and a dark lower tier with red flames, a scalloped
-- green line and an orange band.  In the clip Ron sweeps a level knob: the lower tier rises from black through
-- red, orange and yellow to white while the domes cycle backwards through the palette (about a cycle in 2 s).
--
-- The patch, as rebuilt (oilslick.py has the tables):
--   phi, V = the frozen feedback frame: two stored maps (upper tier 64 x 64 cells, lower 32 x 64; one period of
--            the kaleidoscope), bilinear per pixel; the line engine interpolates the next line's row into a
--            ping-pong row buffer, the pixel path interpolates across the row
--   phase  = pivot + (phi - pivot) * Rings + flow + tide offset + oil ripple + Etch * luma
--   colour = Palette(phase) * V' + green line where V crosses 0.15   (V' = V + Voids, the lower tier lifted by Tide)
--
--   K1  Domes    kaleidoscope period: ~0.7 .. 5.8 domes across (512 = the still's 2.04)
--   K2  Horizon  the horizon height, y 293 .. 907 (512 = 600): the domes sink under it or gain a mirror image
--   K3  Rings    contour density about the dome's red rim: x0.35 (an ember wash) .. x2.83 (fine oil rings)
--   K4  Flow     bipolar palette flow through the rings (centre +-8 still); 640 = a slow drift
--   K5  Voids    the darkness: no voids (0) .. the still (512) .. black swallowing all but the domes (1023)
--   K6  Etch     how far the camera (or the Osc, x4) bends the rings
--   S7  Slick    Dark (the still) / Light: the negative, the voids glow and the domes sink into shadow
--   S8  Colours  Oil (the still) / Contour (the green / magenta / cyan look of clip 00_04_37)
--   S9  Reflect  Tier (the still's own lower tier) / Mirror: the domes reflected upside-down in the water
--   S10 Boil     Calm / Boil: strong ripple warp, the oil boils and the water heaves
--   S11 Source   Video / Osc
--   P12 TIDE     Ron's sweep: 0 = moonlit (white-hot cores in blue domes over black water) .. 50 % = the still ..
--                100 % = high tide (the lower tier blazes white / yellow, the domes run red / orange)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a per-line engine kicked at
-- each line start (computes the NEXT line: map row and fraction, wobble, tile start, tide lift; and builds its
-- interpolated row), a 16-stage pixel path, then the shell's output conditioning.  Virtual 1080-line coordinates
-- (SD shows the centre 4:3).  Generated avid; U and V through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum
-- resync, Y through [1 2 1] on the same centre, luma 13/16, blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.oilslick_pkg.all;
use work.all;

architecture oilslick of program_top is
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

    -- video line buffer: (luma >> 2) - 16, every other source pixel
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs: exp2 (sequencer), maps (row builder), palette (pixel path)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal rb_addr : unsigned(11 downto 0) := (others => '0');
    signal mu_q, ml_q : unsigned(9 downto 0) := (others => '0');
    signal r_pa   : unsigned(8 downto 0) := (others => '0');
    signal r_pq   : std_logic_vector(31 downto 0) := (others => '0');

    -- row buffer: 2 banks x 64 words {phi0, V0, phi1, V1} (phi 6.2, V 4.4); written by the row builder in the
    -- bank the pixel path is not reading
    type t_rw is array (0 to 255) of std_logic_vector(31 downto 0);
    signal rw : t_rw := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of rw : signal is true;
    signal rw_we : std_logic := '0';
    signal rw_wa, rw_ra : unsigned(7 downto 0) := (others => '0');
    signal rw_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal rw_q  : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 240 x 180 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(273, 10);
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

    -- knobs: glided (Q3), raw flow, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(350 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_neg, s_pal, s_refl, s_boil, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(9 downto 0) := (others => '0');
    signal z_hi   : std_logic := '0';                                   -- tide above the still
    signal z_d    : unsigned(9 downto 0) := (others => '0');            -- |tide - 512|
    signal c_kt   : unsigned(15 downto 0) := to_unsigned(8905, 16);     -- tile step per h, Q8 (2^24 = one period)
    signal c_c0   : unsigned(23 downto 0) := (others => '0');           -- tile phase at the line start
    signal c_step : unsigned(17 downto 0) := to_unsigned(17810, 18);    -- tile step per pixel
    signal c_g    : unsigned(9 downto 0) := to_unsigned(208, 10);       -- rings gain (phase per 1/16 map unit, Q2)
    signal c_dfl  : unsigned(9 downto 0) := (others => '0');            -- flow per field
    signal c_fl   : unsigned(15 downto 0) := (others => '0');           -- flow phase
    signal c_eg   : unsigned(9 downto 0) := (others => '0');            -- etch per luma step (Osc x4)
    signal c_ra   : unsigned(7 downto 0) := to_unsigned(8, 8);          -- ripple amplitude
    signal c_au, c_al : unsigned(7 downto 0) := to_unsigned(8, 8);      -- wobble amplitudes (h), upper / lower
    signal c_i1, c_i2 : unsigned(15 downto 0) := (others => '0');   -- per-line phase steps
    signal c_t1, c_t2 : unsigned(15 downto 0) := (others => '0');   -- per-field phase starts
    signal c_oys  : unsigned(15 downto 0) := to_unsigned(364, 16);
    signal c_yh   : unsigned(10 downto 0) := to_unsigned(600, 11);      -- horizon (virtual line)
    signal c_vo   : signed(11 downto 0) := to_signed(-58, 12);          -- voids offset (V 4.6 units, 960 = 1)
    signal c_lbA  : signed(11 downto 0) := (others => '0');             -- lower-tier V lift at the horizon
    signal c_lbB  : unsigned(11 downto 0) := (others => '0');           -- lift slope per rr (256 = the full ramp)
    signal c_offu, c_offl : unsigned(15 downto 0) := (others => '0');   -- tier phase offsets (incl. pivot, flow)
    signal c_dl   : unsigned(13 downto 0) := (others => '0');           -- lower extra phase (tide > 50 %)
    signal c_ly, c_lu, c_lv : signed(9 downto 0) := (others => '0');    -- line colour deltas (x 32)

    -- field serial multiplier
    signal f_a   : unsigned(17 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(33 downto 0) := (others => '0');
    signal z_fa  : unsigned(17 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal f_cnt : unsigned(4 downto 0) := (others => '0');
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
    signal e_ym   : unsigned(17 downto 0) := (others => '0');
    signal e_yl   : unsigned(17 downto 0) := (others => '0');
    signal e_low, e_lmap : std_logic := '0';
    signal e_w1, e_w2 : unsigned(15 downto 0) := (others => '0');
    signal e_s1, e_s2 : unsigned(7 downto 0) := (others => '0');
    signal e_g1, e_g2, e_gw : std_logic := '0';
    signal e_rr   : unsigned(8 downto 0) := (others => '0');
    signal e_wb   : std_logic := '0';
    signal m_a    : unsigned(17 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal m_acc  : unsigned(33 downto 0) := (others => '0');
    signal n_tp0  : unsigned(23 downto 0) := (others => '0');
    signal n_low  : std_logic := '0';
    signal n_lb   : signed(11 downto 0) := (others => '0');
    signal n_rpy  : unsigned(15 downto 0) := (others => '0');

    -- row builder
    signal rb_run : std_logic := '0';
    signal rb_rc, rb_mx : unsigned(5 downto 0) := (others => '0');
    signal rb_w0, rb_w2 : unsigned(7 downto 0) := (others => '0');
    signal rb_w1  : unsigned(8 downto 0) := (others => '0');
    signal e_t8   : unsigned(7 downto 0) := (others => '0');
    signal rb_sub : unsigned(1 downto 0) := (others => '0');
    signal rb_cel : unsigned(5 downto 0) := (others => '0');
    signal rb_row : unsigned(5 downto 0) := (others => '0');
    signal rb_sp  : unsigned(13 downto 0) := (others => '0');
    signal rb_sv  : unsigned(11 downto 0) := (others => '0');
    signal rb_mp  : unsigned(13 downto 0) := (others => '0');
    signal rb_mv  : unsigned(11 downto 0) := (others => '0');
    signal rb_t4  : std_logic_vector(8 downto 0) := (others => '0');
    signal rb_lm, rb_bank : std_logic := '0';
    signal rb_t1, rb_t2, rb_t3 : std_logic_vector(8 downto 0) := (others => '0');   -- {valid, sub(2), cell}
    signal rb_q2  : unsigned(9 downto 0) := (others => '0');
    signal rb_t5, rb_t6 : std_logic_vector(8 downto 0) := (others => '0');
    signal rb_cur, rb_prv, rb_first : std_logic_vector(15 downto 0) := (others => '0');
    signal rb_cc  : unsigned(5 downto 0) := (others => '0');
    signal rb_cv, rb_last : std_logic := '0';

    -- beam
    signal b_tp  : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_low : std_logic := '0';
    signal l_lb  : signed(11 downto 0) := (others => '0');
    signal l_rpy : unsigned(15 downto 0) := (others => '0');
    signal l_off : unsigned(15 downto 0) := (others => '0');
    signal l_rb  : std_logic := '0';

    -- pixel path
    signal p2_fx, p3_fx, p4_fx, p5_fx : unsigned(5 downto 0) := (others => '0');
    signal p2_r  : unsigned(15 downto 0) := (others => '0');
    signal p3_tr : signed(8 downto 0) := (others => '0');
    signal p4_w  : std_logic_vector(31 downto 0) := (others => '0');
    signal p4_rip : signed(16 downto 0) := (others => '0');
    signal p4_lm : unsigned(7 downto 0) := (others => '0');
    signal p5_dp : signed(8 downto 0) := (others => '0');
    signal p5_dv : signed(8 downto 0) := (others => '0');
    signal p5_p0, p5_v0 : unsigned(7 downto 0) := (others => '0');
    signal p5_rip : signed(16 downto 0) := (others => '0');
    signal p5_ea : unsigned(15 downto 0) := (others => '0');
    signal p6_pp : signed(15 downto 0) := (others => '0');
    signal p6_pv : signed(15 downto 0) := (others => '0');
    signal p6_p0, p6_v0 : unsigned(7 downto 0) := (others => '0');
    signal p6_rip : signed(16 downto 0) := (others => '0');
    signal p6_et : unsigned(17 downto 0) := (others => '0');
    signal p7_ph : unsigned(13 downto 0) := (others => '0');
    signal p7_v  : unsigned(13 downto 0) := (others => '0');
    signal p7_s  : unsigned(15 downto 0) := (others => '0');
    signal p8_d  : signed(8 downto 0) := (others => '0');
    signal p8_vq : unsigned(9 downto 0) := (others => '0');
    signal p8_s  : unsigned(15 downto 0) := (others => '0');
    signal p8_low : std_logic := '0';
    signal p9_hi, p9_lo : signed(14 downto 0) := (others => '0');
    signal p9_vq : unsigned(9 downto 0) := (others => '0');
    signal p9_s  : unsigned(15 downto 0) := (others => '0');
    signal p9_dl : signed(12 downto 0) := (others => '0');
    signal p9_low : std_logic := '0';
    signal p10_r : unsigned(15 downto 0) := (others => '0');
    signal p10_s : unsigned(15 downto 0) := (others => '0');
    signal p10_ve : signed(13 downto 0) := (others => '0');
    signal p10_bp : unsigned(5 downto 0) := (others => '0');
    signal p11_ph : unsigned(15 downto 0) := (others => '0');
    signal p11_vn : signed(13 downto 0) := (others => '0');
    signal p11_bp : unsigned(5 downto 0) := (others => '0');
    signal p12_vs : unsigned(9 downto 0) := (others => '0');
    signal p12_g, p13_g, p14_g : unsigned(4 downto 0) := (others => '0');
    signal p13_b, p14_b : unsigned(5 downto 0) := (others => '0');
    signal p14_q : std_logic_vector(31 downto 0) := (others => '0');
    signal p15_y : unsigned(15 downto 0) := (others => '0');
    signal p15_u, p15_v : signed(15 downto 0) := (others => '0');
    signal p15_gy, p15_gu, p15_gv : signed(10 downto 0) := (others => '0');
    signal s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_u19, s_u20, s_v18, s_v19, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- RAMs and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rw : process(clk)
    begin
        if rising_edge(clk) then
            rw_q <= rw(to_integer(rw_ra));
            if rw_we = '1' then rw(to_integer(rw_wa)) <= rw_wd; end if;
        end if;
    end process p_rw;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_OS_EXP2(to_integer(r_ex_a));
            mu_q   <= C_OS_MAPU(to_integer(rb_addr));
            ml_q   <= C_OS_MAPL(to_integer(rb_addr(10 downto 0)));
            r_pq   <= C_OS_PAL(to_integer(r_pa));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_t  : unsigned(15 downto 0);
        variable v_h  : unsigned(18 downto 0);
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
                s_neg  <= registers_in(6)(0);
                s_pal  <= registers_in(6)(1);
                s_refl <= registers_in(6)(2);
                s_boil <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(546, 10);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(410, 10);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(273, 10);
                end case;
            end if;
            g_wm1 <= s_w - 1;

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
            -- (stored at load + 18).  Slot decoded at 30 + 24 k, operands pre-muxed at 31 + 24 k.
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
                    when 318 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(8905, 18);          z_fb <= z_mt;                       -- kt
                    when 1  => z_fa <= resize(c_kt, 18);                z_fb <= resize(to_unsigned(2040, 12) - s_x0, 16);  -- C0
                    when 2  => z_fa <= resize(c_kt, 18);                z_fb <= resize(s_xs, 16);           -- step
                    when 3  => z_fa <= to_unsigned(208, 18);            z_fb <= z_mt;                       -- rings
                    when 4  => z_fa <= resize(s_vm, 18);                z_fb <= resize(s_vm, 16);           -- flow^2
                    when 5  => z_fa <= f_acc(17 downto 0);  z_fb <= to_unsigned(226, 16);
                    when 6  => z_fa <= resize(s_g6(12 downto 3), 18);   z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 7  => z_fa <= resize(z_a, 18);                 z_fb <= to_unsigned(231, 16);
                    when 8  => z_fa <= resize(s_ys, 18);                z_fb <= to_unsigned(434, 16);       -- wobble 1
                    when 9  => z_fa <= resize(s_ys, 18);                z_fb <= to_unsigned(2849, 16);      -- wobble 2
                    when 12 => z_fa <= resize(s_ys, 18);                z_fb <= to_unsigned(364, 16);       -- osc y
                    when 13 =>                                                                              -- tide lift
                        z_fa <= resize(z_d, 18);
                        if z_hi = '1' then z_fb <= to_unsigned(740, 16); else z_fb <= to_unsigned(1154, 16); end if;
                    when 14 => z_fa <= resize(z_d, 18);                 z_fb <= to_unsigned(1374, 16);      -- tide slope
                    when 16 => z_fa <= resize(z_a, 18);                 z_fb <= to_unsigned(1056, 16);      -- voids
                    when 17 => z_fa <= resize(z_a, 18);                 z_fb <= to_unsigned(614, 16);       -- horizon
                    when others => z_fa <= resize(z_d, 18);             z_fb <= to_unsigned(19, 16);        -- tide lower
                end case;
            end if;
            -- right-shifting accumulator: after 16 steps f_acc = a * b
            -- (16 steps, then the product holds)
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb; f_cnt <= to_unsigned(16, 5);
            elsif f_cnt /= 0 then
                if f_b(0) = '1' then v_h := resize(f_acc(33 downto 16), 19) + f_a;
                else v_h := resize(f_acc(33 downto 16), 19); end if;
                f_acc <= v_h & f_acc(15 downto 1);
                f_b <= '0' & f_b(15 downto 1);
                f_cnt <= f_cnt - 1;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion: last field's flow, drifting phases; domes exponent
                    when 20 =>
                        if s_vneg = '1' then c_fl <= c_fl - resize(c_dfl, 16); else c_fl <= c_fl + resize(c_dfl, 16); end if;
                        c_t1 <= c_t1 - 339; c_t2 <= c_t2 - 983;
                        -- domes exponent 0.75 (k1 - 512) = (k1 - k1 / 4) - 384: +-1.5 octaves
                        z_e <= signed(resize(s_g1(12 downto 3) - shift_right(s_g1(12 downto 3), 2), 11)) - 384;
                    when 23 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 25 => z_mt <= r_ex_q;
                    when 26 =>
                        if s_boil = '1' then c_ra <= to_unsigned(82, 8); c_au <= to_unsigned(88, 8); c_al <= to_unsigned(80, 8);
                        else c_ra <= to_unsigned(8, 8); c_au <= to_unsigned(8, 8); c_al <= to_unsigned(12, 8); end if;
                        if s_pal = '1' then
                            c_ly <= to_signed(387, 10); c_lu <= to_signed(347, 10); c_lv <= to_signed(215, 10);
                        else
                            c_ly <= to_signed(496, 10); c_lu <= to_signed(-294, 10); c_lv <= to_signed(-233, 10);
                        end if;
                    when 50 =>                       -- kt = 8905 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_kt <= resize(f_acc(31 downto 17), 16);
                            when -1     => c_kt <= f_acc(31 downto 16);
                            when 0      => c_kt <= f_acc(30 downto 15);
                            when others => c_kt <= f_acc(29 downto 14);
                        end case;
                    when 74  => c_c0 <= to_unsigned(8257536, 24) - f_acc(23 downto 0);    -- 2^23 - half a cell
                    when 98  =>
                        c_step <= f_acc(17 downto 0);
                        -- rings exponent: 0.75 (k3 - 512) Q8 octaves
                        z_e <= signed(resize(s_g3(12 downto 3) - shift_right(s_g3(12 downto 3), 2), 11)) - 384;
                    when 101 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 103 => z_mt <= r_ex_q;
                    when 122 =>                      -- G = 208 mt >> (15 - sh)
                        case to_integer(z_sh) is
                            when -2     => c_g <= resize(f_acc(31 downto 17), 10);
                            when -1     => c_g <= f_acc(25 downto 16);
                            when 0      => c_g <= f_acc(24 downto 15);
                            when others => c_g <= f_acc(23 downto 14);
                        end case;
                    when 170 => c_dfl <= f_acc(25 downto 16);
                    when 194 => z_a <= f_acc(19 downto 10);
                    when 218 =>                      -- etch per luma step: k6^2 231 / 2^20; Osc x4
                        if s_src = '1' then c_eg <= f_acc(17 downto 8); else c_eg <= resize(f_acc(17 downto 10), 10); end if;
                        -- tide: |p12 - 512|, above / below the still
                        if s_gp(12 downto 3) >= 512 then z_d <= s_gp(12 downto 3) - 512; z_hi <= '1';
                        else z_d <= 512 - s_gp(12 downto 3); z_hi <= '0'; end if;
                    when 242 => c_i1 <= f_acc(23 downto 8);
                    when 266 => c_i2 <= f_acc(23 downto 8);
                    when 338 => c_oys <= f_acc(23 downto 8);
                    when 362 =>                      -- lift at the horizon: below the still -0.6 (1/2 - t), above +0.72 (t - 1/2)
                        if z_hi = '1' then c_lbA <= signed(resize(f_acc(20 downto 10), 12));
                        else c_lbA <= -signed(resize(f_acc(20 downto 10), 12)); end if;
                    when 386 =>
                        if z_hi = '1' then c_lbB <= f_acc(21 downto 10); else c_lbB <= (others => '0'); end if;
                    when 410 =>
                        -- voids: |512 - k5|
                        if s_g5(12 downto 3) >= 512 then z_a <= s_g5(12 downto 3) - 512; z_hi <= '1';
                        else z_a <= 512 - s_g5(12 downto 3); z_hi <= '0'; end if;
                    when 434 =>                      -- vo = (512 - k5) 1056 / 1024 - 58
                        if z_hi = '1' then c_vo <= -signed(resize(f_acc(20 downto 10), 12)) - 58;
                        else c_vo <= signed(resize(f_acc(20 downto 10), 12)) - 58; end if;
                        -- horizon: |k2 - 512|
                        if s_g2(12 downto 3) >= 512 then z_a <= s_g2(12 downto 3) - 512; z_hi <= '1';
                        else z_a <= 512 - s_g2(12 downto 3); z_hi <= '0'; end if;
                    when 458 =>                      -- yh = 600 +- |k2 - 512| 614 / 1024
                        if z_hi = '1' then c_yh <= to_unsigned(600, 11) + resize(f_acc(18 downto 10), 11);
                        else c_yh <= to_unsigned(600, 11) - resize(f_acc(18 downto 10), 11); end if;
                        if s_gp(12 downto 3) >= 512 then z_hi <= '1'; else z_hi <= '0'; end if;
                    when 482 =>                      -- lower extra phase (tide above the still): -19 |t - 512|
                        if z_hi = '1' then c_dl <= f_acc(13 downto 0); else c_dl <= (others => '0'); end if;
                    when 483 =>                      -- tier offsets: pivot 6554 + flow - 32 (t - 512); lower +2621 - dl
                        v_t := resize(shift_left(resize(s_gp(12 downto 3), 16), 5), 16);
                        c_offu <= to_unsigned(6554 + 16384, 16) + c_fl - v_t;           -- 512 * 32 = 16384
                    when 484 =>
                        c_offl <= c_offu + to_unsigned(2621, 16) - resize(c_dl, 16);
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
    -- Source side: oscillator lattice, line-buffer write of (luma >> 2) - 16
    -- (every other source pixel)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 8) + shift_left(resize(s_ot, 16), 6);       -- ot * 320
                s_oy <= s_oy + c_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 7) + shift_left(resize(s_ot, 16), 6); end if;  -- ot * 192
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid and not s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17): the map row and B-spline weights (the row builder starts at
    -- e_lc 54), the tide ramp, three wobble sines (parabolic), the wobble -> tile
    -- start, the lower-tier lift (done at e_lc 189)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : signed(20 downto 0);
        variable v_q  : unsigned(15 downto 0);
        variable v_t  : unsigned(7 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_m  : unsigned(8 downto 0);
        variable v_o  : unsigned(23 downto 0);
        variable v_b  : signed(12 downto 0);
        variable v_h  : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_w1 <= c_t1; e_w2 <= c_t2;
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_w1 <= e_w1 + c_i1; e_w2 <= e_w2 + c_i2;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            -- right-shifting accumulator, 16 steps then the product holds (a load below overrides this
            -- step's m_acc / m_b and restarts the count)
            if m_cnt /= 0 then
                if m_b(0) = '1' then v_h := resize(m_acc(33 downto 16), 19) + m_a;
                else v_h := resize(m_acc(33 downto 16), 19); end if;
                m_acc <= v_h & m_acc(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
                m_cnt <= m_cnt - 1;
            end if;
            case to_integer(e_lc) is
                when 2 | 19 | 36 | 53 | 70 | 87 | 138 | 155 | 172 => m_cnt <= to_unsigned(16, 5);
                when others => null;
            end case;

            case to_integer(e_lc) is
                when 1 =>
                    -- tier and map y (Q8 virtual lines)
                    v_y := signed(resize(e_vy, 21)) - signed(resize(shift_left(resize(c_yh, 19), 8), 21));
                    if v_y >= 0 then
                        e_low <= '1';
                        e_yl <= unsigned(v_y(17 downto 0));
                        if s_refl = '1' then
                            e_lmap <= '0';
                            v_y := to_signed(153600, 21) - v_y;
                            if v_y < 0 then e_ym <= (others => '0'); else e_ym <= unsigned(v_y(17 downto 0)); end if;
                        else
                            e_lmap <= '1';
                            if v_y >= 122880 then v_y := to_signed(245760, 21) - v_y; end if;
                            if v_y < 0 then e_ym <= (others => '0'); else e_ym <= unsigned(v_y(17 downto 0)); end if;
                        end if;
                    else
                        e_low <= '0'; e_lmap <= '0';
                        e_yl <= (others => '0');
                        if e_vy < 153600 then
                            e_ym <= e_vy(17 downto 0);
                        else
                            v_y := to_signed(307200, 21) - signed(resize(e_vy, 21));
                            if v_y < 0 then e_ym <= (others => '0'); else e_ym <= unsigned(v_y(17 downto 0)); end if;
                        end if;
                    end if;
                when 2 =>
                    m_a <= resize(e_ym, 18); m_acc <= (others => '0');
                    if e_lmap = '1' then m_b <= to_unsigned(4369, 16); else m_b <= to_unsigned(6991, 16); end if;
                when 19 =>
                    -- row coordinate (Q8; row r's centre at r + 1/2), clamped; rc and t for the B-spline
                    v_q := m_acc(31 downto 16);
                    if (e_lmap = '1' and v_q(15 downto 8) > 31) or (e_lmap = '0' and v_q(15 downto 8) > 63) then
                        if e_lmap = '1' then rb_rc <= to_unsigned(31, 6); else rb_rc <= to_unsigned(63, 6); end if;
                        e_t8 <= to_unsigned(128, 8);
                        m_a <= to_unsigned(128, 18);
                    else
                        rb_rc <= v_q(13 downto 8);
                        e_t8 <= v_q(7 downto 0);
                        m_a <= resize(v_q(7 downto 0), 18);
                    end if;
                    if e_lmap = '1' then rb_mx <= to_unsigned(31, 6); else rb_mx <= to_unsigned(63, 6); end if;
                    rb_lm <= e_lmap; rb_bank <= e_wb;
                    m_acc <= (others => '0');
                    if (e_lmap = '1' and v_q(15 downto 8) > 31) or (e_lmap = '0' and v_q(15 downto 8) > 63) then
                        m_b <= to_unsigned(128, 16);
                    else
                        m_b <= resize(v_q(7 downto 0), 16);
                    end if;
                when 36 =>
                    rb_w2 <= m_acc(16 downto 9);                       -- t^2 / 2
                    m_a <= resize(to_unsigned(256, 9) - e_t8, 18); m_b <= resize(to_unsigned(256, 9) - e_t8, 16);
                    m_acc <= (others => '0');
                when 53 =>
                    rb_w0 <= m_acc(16 downto 9);                       -- (1 - t)^2 / 2
                    rb_w1 <= to_unsigned(256, 9) - resize(m_acc(16 downto 9), 9) - resize(rb_w2, 9);
                    m_a <= resize(e_yl, 18); m_b <= to_unsigned(137, 16); m_acc <= (others => '0');
                when 70 =>
                    if m_acc(33 downto 16) >= 256 then e_rr <= to_unsigned(256, 9); else e_rr <= m_acc(24 downto 16); end if;
                    v_t := e_w1(14 downto 7);
                    m_a <= resize(v_t, 18); m_b <= resize(not v_t, 16); m_acc <= (others => '0');
                    e_g1 <= e_w1(15);
                when 87 =>
                    e_s1 <= m_acc(13 downto 6);
                    v_t := e_w2(14 downto 7);
                    m_a <= resize(v_t, 18); m_b <= resize(not v_t, 16); m_acc <= (others => '0');
                    e_g2 <= e_w2(15);
                when 104 =>
                    e_s2 <= m_acc(13 downto 6);
                when 138 =>
                    if e_low = '1' then
                        m_a <= resize(e_s2, 18); m_b <= resize(c_al, 16); e_gw <= e_g2;
                    else
                        m_a <= resize(e_s1, 18); m_b <= resize(c_au, 16); e_gw <= e_g1;
                    end if;
                    m_acc <= (others => '0');
                when 155 =>
                    -- wobble (h) -> tile offset = wob * kt
                    m_a <= resize(m_acc(15 downto 8), 18); m_b <= c_kt; m_acc <= (others => '0');
                when 172 =>
                    v_o := m_acc(23 downto 0);
                    if e_gw = '1' then n_tp0 <= c_c0 - v_o; else n_tp0 <= c_c0 + v_o; end if;
                    m_a <= resize(e_rr, 18); m_b <= resize(c_lbB, 16); m_acc <= (others => '0');
                when 189 =>
                    v_b := resize(c_lbA, 13) + signed(resize(m_acc(19 downto 8), 13));
                    if v_b > 2047 then n_lb <= to_signed(2047, 12); else n_lb <= v_b(11 downto 0); end if;
                    n_low <= e_low;
                    n_rpy <= e_w1;
                when others => null;
            end case;
            if e_zk = '1' then e_wb <= '0';
            elsif g_start = '1' then e_wb <= not e_wb; end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Row builder (e_lc 54 .. ~255): for each of the 64 cells, read rows rc - 1,
    -- rc, rc + 1 of the selected map and weight them by the quadratic B-spline
    -- of t ((1 - t)^2 / 2, the rest, t^2 / 2), then write the row-buffer word pair
    -- {cell c, cell c + 1} (word 63 wraps to cell 0).  C1-smooth in y, so no
    -- contour can pinch to a single row at a map row (no 1-row lines).
    ---------------------------------------------------------------------------
    p_rb : process(clk)
        variable v_w : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- items: cells 0 .. 63, three rows each (rc - 1, rc, rc + 1, clamped to the map)
            if e_lc = 54 then
                rb_run <= '1'; rb_sub <= "00"; rb_cel <= (others => '0');
            elsif rb_run = '1' then
                if rb_sub = 2 then
                    rb_sub <= "00";
                    if rb_cel = 63 then rb_run <= '0'; end if;
                    rb_cel <= rb_cel + 1;
                else
                    rb_sub <= rb_sub + 1;
                end if;
            end if;
            -- 1: row and tag; 2: address; 3: ROM data; 4: fabric copy
            case rb_sub is
                when "00"   => if rb_rc = 0 then rb_row <= rb_rc; else rb_row <= rb_rc - 1; end if;
                when "01"   => rb_row <= rb_rc;
                when others => if rb_rc = rb_mx then rb_row <= rb_rc; else rb_row <= rb_rc + 1; end if;
            end case;
            rb_t1 <= rb_run & std_logic_vector(rb_sub) & std_logic_vector(rb_cel);
            rb_addr <= rb_row & unsigned(rb_t1(5 downto 0));
            if rb_lm = '1' then rb_addr(11) <= '0'; end if;
            rb_t2 <= rb_t1;
            rb_t3 <= rb_t2;
            if rb_lm = '1' then rb_q2 <= ml_q; else rb_q2 <= mu_q; end if;
            rb_t4 <= rb_t3;
            -- 5: weighted products (phi 6 x 9 bits, V 4 x 9 bits)
            case rb_t4(7 downto 6) is
                when "00"   => v_w := resize(rb_w0, 9);
                when "01"   => v_w := rb_w1;
                when others => v_w := resize(rb_w2, 9);
            end case;
            rb_mp <= resize(rb_q2(9 downto 4) * v_w, 14);
            rb_mv <= resize(rb_q2(3 downto 0) * v_w, 12);
            rb_t5 <= rb_t4;
            -- 6: accumulate over the three rows
            if rb_t5(7 downto 6) = "00" then
                rb_sp <= rb_mp; rb_sv <= rb_mv;
            else
                rb_sp <= rb_sp + rb_mp; rb_sv <= rb_sv + rb_mv;
            end if;
            rb_t6 <= rb_t5;
            -- 7: cell result (phi 6.2 = sum >> 6, V 4.4 = sum >> 4)
            rb_cv <= rb_t6(8) and rb_t6(7);
            rb_cc <= unsigned(rb_t6(5 downto 0));
            rb_cur <= std_logic_vector(rb_sp(13 downto 6)) & std_logic_vector(rb_sv(11 downto 4));
            -- 8: write {prev, cur} at cell - 1; after cell 63 also {cell 63, cell 0} at word 63
            rw_we <= '0';
            rb_last <= '0';
            if rb_cv = '1' then
                rb_prv <= rb_cur;
                if rb_cc = 0 then
                    rb_first <= rb_cur;
                else
                    rw_we <= '1'; rw_wa <= rb_bank & '0' & (rb_cc - 1); rw_wd <= rb_prv & rb_cur;
                end if;
                if rb_cc = 63 then rb_last <= '1'; end if;
            elsif rb_last = '1' then
                rw_we <= '1'; rw_wa <= rb_bank & '0' & to_unsigned(63, 6); rw_wd <= rb_prv & rb_first;
            end if;
        end if;
    end process p_rb;

    ---------------------------------------------------------------------------
    -- Beam: tile phase (Q8, 2^24 = one kaleidoscope period), ripple x phase,
    -- pixel count; per-line values loaded at the line start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_tp  <= n_tp0;
                b_px  <= (others => '0');
                l_low <= n_low; l_lb <= n_lb; l_rpy <= n_rpy; l_rb <= e_wb;
                if n_low = '1' then l_off <= c_offl; else l_off <= c_offu; end if;
            elsif g_avid = '1' then
                b_tp  <= b_tp + c_step;
                b_px  <= b_px + 1;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t   : unsigned(7 downto 0);
        variable v_ve  : signed(13 downto 0);
        variable v_dl  : signed(12 downto 0);
        variable v_lw  : unsigned(6 downto 0);
        variable v_rg  : signed(19 downto 0);
        variable v_lp  : unsigned(11 downto 0);
        variable v_vs  : unsigned(25 downto 0);
        variable v_y   : signed(12 downto 0);
        variable v_u   : signed(12 downto 0);
        variable v_v   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: row-buffer and line-buffer addresses, ripple phase
            rw_ra <= l_rb & '0' & b_tp(23 downto 18);
            p2_fx <= b_tp(17 downto 12);
            p2_r  <= b_tp(20 downto 5) + l_rpy;           -- ripple: 8 per kaleidoscope period, diagonal
            s_lb_ra <= b_px(10 downto 1);
            -- 3: (EBR) triangle ripple
            p3_fx <= p2_fx;
            if p2_r(15) = '1' then v_t := not p2_r(14 downto 7); else v_t := p2_r(14 downto 7); end if;
            p3_tr <= signed(resize(v_t, 9)) - 128;
            -- 4: fabric copies; ripple product; luma
            p4_w <= rw_q; p4_fx <= p3_fx;
            p4_rip <= resize(p3_tr * signed('0' & c_ra), 17);
            p4_lm <= unsigned(s_lb_q);
            -- 5: differences across the row; etch partials
            p5_p0 <= unsigned(p4_w(31 downto 24)); p5_v0 <= unsigned(p4_w(23 downto 16));
            p5_dp <= signed('0' & p4_w(15 downto 8)) - signed('0' & p4_w(31 downto 24));
            p5_dv <= signed('0' & p4_w(7 downto 0)) - signed('0' & p4_w(23 downto 16));
            p5_fx <= p4_fx; p5_rip <= p4_rip;
            p5_ea <= resize(p4_lm(7 downto 2) * c_eg, 16);
            -- 6: products by fx; etch sum
            p6_pp <= resize(p5_dp * signed('0' & p5_fx), 16);
            p6_pv <= resize(p5_dv * signed('0' & p5_fx), 16);
            p6_p0 <= p5_p0; p6_v0 <= p5_v0; p6_rip <= p5_rip;
            p6_et <= shift_left(resize(p5_ea, 18), 2);
            -- 7: phi (6.8), V (4.10); ripple + etch (phase units, mod 2^16)
            -- (mod 2^14: the true sums lie in 0 .. 16128; slices, never a narrowing signed resize)
            p7_ph <= shift_left(resize(p6_p0, 14), 6) + unsigned(p6_pp(13 downto 0));
            p7_v  <= shift_left(resize(p6_v0, 14), 6) + unsigned(p6_pv(13 downto 0));
            p7_s  <= unsigned(p6_rip(15 downto 0)) + p6_et(15 downto 0);
            -- 8: d = phi - pivot (6.4 units), V 4.6; + the line's offset
            p8_d  <= signed(resize(p7_ph(13 downto 6), 9)) - 32;          -- phi - pivot (6.2 units)
            p8_vq <= p7_v(13 downto 4);
            p8_s  <= p7_s + l_off;
            p8_low <= l_low;
            -- 9: rings partial products; lower-tier V scale; line distance (un-lifted)
            p9_hi <= resize(p8_d * signed('0' & c_g(9 downto 5)), 15);
            p9_lo <= resize(p8_d * signed('0' & c_g(4 downto 0)), 15);
            p9_vq <= p8_vq;
            p9_s <= p8_s; p9_low <= p8_low;
            p9_dl <= signed(resize(p8_vq, 13)) + resize(c_vo, 13) - 144;
            -- 10: rings = d G / 4; V' = scaled + lift + voids; line bump
            v_rg  := shift_left(resize(p9_hi, 20), 5) + resize(p9_lo, 20);
            p10_r <= unsigned(v_rg(15 downto 0));                      -- phase, mod one cycle
            p10_s <= p9_s;
            if p9_low = '1' then v_ve := signed(resize(p9_vq, 14)) + resize(l_lb, 14) + resize(c_vo, 14);
            else v_ve := signed(resize(p9_vq, 14)) + resize(c_vo, 14); end if;
            p10_ve <= v_ve;
            v_dl := abs(p9_dl);
            v_lw := resize(unsigned(88 - v_dl), 7);
            v_lp := shift_left(resize(v_lw, 12), 4) + shift_left(resize(v_lw, 12), 3);      -- x 24
            if v_dl >= 88 then p10_bp <= (others => '0');
            elsif v_lp(11 downto 6) > 32 then p10_bp <= to_unsigned(32, 6);
            else p10_bp <= v_lp(11 downto 6); end if;
            -- 11: phase; negative
            p11_ph <= p10_r + p10_s;
            if s_neg = '1' then p11_vn <= 960 - p10_ve; else p11_vn <= p10_ve; end if;
            p11_bp <= p10_bp;
            -- 12: palette address; V clamp
            r_pa <= s_pal & p11_ph(15 downto 8);
            if p11_vn < 0 then p12_vs <= (others => '0');
            elsif p11_vn > 960 then p12_vs <= to_unsigned(960, 10);
            else p12_vs <= unsigned(p11_vn(9 downto 0)); end if;
            p12_g <= p11_bp(5 downto 1);                 -- line 0 .. 16 (sixteenths)
            -- 13: (palette EBR) brightness b = Vs / 15 (0 .. 64)
            v_vs := resize(p12_vs, 26) + shift_left(resize(p12_vs, 26), 4) + shift_left(resize(p12_vs, 26), 8)
                    + shift_left(resize(p12_vs, 26), 12);
            p13_b <= v_vs(22 downto 17);                 -- brightness 0 .. 31 (of 32)
            p13_g <= p12_g;
            -- 14: fabric copy
            p14_q <= r_pq; p14_b <= p13_b; p14_g <= p13_g;
            -- 15: palette x b; line colour x g / 32
            p15_y <= resize((unsigned(p14_q(31 downto 22)) - 64) * p14_b, 16);
            p15_u <= resize((signed('0' & p14_q(21 downto 12)) - 512) * signed('0' & p14_b), 16);
            p15_v <= resize((signed('0' & p14_q(11 downto 2)) - 512) * signed('0' & p14_b), 16);
            p15_gy <= resize(shift_right(c_ly * signed('0' & p14_g), 4), 11);
            p15_gu <= resize(shift_right(c_lu * signed('0' & p14_g), 4), 11);
            p15_gv <= resize(shift_right(c_lv * signed('0' & p14_g), 4), 11);
            -- 16: compose (black outside the generated avid)
            v_y := signed(resize(p15_y(15 downto 5), 13)) + 64 + resize(p15_gy, 13);
            v_u := resize(shift_right(p15_u, 5), 13) + 512 + resize(p15_gu, 13);
            v_v := resize(shift_right(p15_v, 5), 13) + 512 + resize(p15_gv, 13);
            if s_gav_sr(13) = '0' then
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            else
                if v_y > 940 then s_y16 <= to_unsigned(940, 10); elsif v_y < 64 then s_y16 <= to_unsigned(64, 10);
                else s_y16 <= unsigned(v_y(9 downto 0)); end if;
                if v_u > 960 then s_u16 <= to_unsigned(960, 10); elsif v_u < 64 then s_u16 <= to_unsigned(64, 10);
                else s_u16 <= unsigned(v_u(9 downto 0)); end if;
                if v_v > 960 then s_v16 <= to_unsigned(960, 10); elsif v_v < 64 then s_v16 <= to_unsigned(64, 10);
                else s_v16 <= unsigned(v_v(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(13);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums, with the running-sum resync), Y through [1 2 1] centred on
    -- the same sample (the HD HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_yy : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 17 sums, stage 18 output; the 8-sums snap back to neutral once 8 blank samples are in
            s_cay(0) <= s_y16; s_cau(0) <= s_u16; s_cav(0) <= s_v16;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u16 - s_cau(7);
                s_bav <= s_bav + s_v16 - s_cav(7);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 14) + s_baud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v18 <= v_sc(13 downto 4);
            -- luma [1 2 1] centred on the box's centre sample (4 back)
            v_yy := resize(s_cay(3), 12) + shift_left(resize(s_cay(4), 12), 1) + resize(s_cay(5), 12);
            s_y18 <= v_yy(11 downto 2);
            -- 19: delay
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
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
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture oilslick;
