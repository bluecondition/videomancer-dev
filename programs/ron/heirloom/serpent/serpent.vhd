-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Serpent v0.1.0 -- Heirloom phase 3 (video clips), from the clip WIN_20230529_01_27_21 at 0.5 s.
--
-- The still: a coiled ribbon (an S) on black.  Its colours are one field u per line, a horizontal ramp whose
-- offset and gain are vertical CVs: green is a sawtooth of u (hard edge, slow ramp), red and blue a diagonal
-- ramp, so the S reads cyan / green / yellow with purple and magenta tongues after every wrap.  The body is
-- a soft window around the coil keyed against a sawtooth column oscillator (22.5 columns, fast fall): fins
-- with vertical right edges along the tip, teeth along the lower-left edge, lens holes in rows at the
-- coil's turning points, and below line 630 a fan that compresses the field into tails running off to
-- the right.  A red layer (with black stripes) glows around the body and shows through every hole.  In the
-- clip the coil scrolls down at 160 lines/s (one turn every 4.9 s) while the fan stays put.
--
-- The patch, as fitted (serpent.py has every constant):
--   th     = 2 pi (y - 110) turns / 1080 - scroll                    the vertical oscillator
--   u      = phi(y) + m(y) (x - 1000) / 1000 (+ Etch * luma)          phi, m: offset / gain CVs (cos th, fan E)
--   G      = 10 + 213 frac(u);  R, B = diagonal ramps - frac(u) / 10
--   T      = window(xL(y), xR(y)) min tail(dist to integer u) max -0.12
--   on     = column saw > T, minus lens holes (alternate columns, rows where |cos th'| > 0.62)
--   red    = window + tails widened (a halo), black stripes inside
--
--   K1  Turns    the coil's vertical frequency (0.41 .. 4.6 turns; the still 1.37)
--   K2  Bands    colour bands: u scaled x1/4 .. x4 (more wraps, more tails)
--   K3  Columns  the fin / hole column oscillator, 8 .. 64 across (the still 22.5)
--   K4  Speed    bipolar: the coil rolls down (+) or up (-); 747 = the clip's 160 lines/s
--   K5  Holes    the lens holes: none .. the still .. 3x
--   K6  Etch     how far the camera (or the Osc) bends the colour field
--   S7  Mirror   the lower half folded up: the fan and tails at both ends
--   S8  Halo     Red (the still) / Off: the ribbon perforated on black
--   S9  Key      Fins (the still) / Lattice: the key inverted inside the halo
--   S10 Palette  Still / Ice (red and blue guns swapped: a blue halo)
--   S11 Source   Video / Osc
--   P12 COIL     the coil's swing and the fan together: 0 = a straight ribbon .. 33 % = the still ..
--                100 % = a wild coil with a dense comb of tails
--
-- Engine: vblank-zone sequencer (one serial multiplier, 30 slots, shared glide), a per-line engine kicked
-- at each line start (computes the NEXT line on its own serial multiplier: 15 products, ~260 clocks, fits
-- SD's 858-clock line).  The pixel path has no multipliers but the 8 x 9 Etch: every per-pixel ramp is a
-- DDA that runs at the pipeline stage where it is consumed (stepped by the delayed generated avid), so no
-- wide value rides a register pipe.  Per-gun YCrCb ROMs.  Virtual 1080-line coordinates (SD shows the
-- centre 4:3).  Generated avid; output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 off a block-RAM ring,
-- luma [1 2 1]/4 centred with it, luma 13/16) with running-sum resync before the blanking gate.  Colours
-- in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.serpent_pkg.all;
use work.all;

architecture serpent of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (serpent.py): u Q24, x in h (half virtual px), lines Q4
    constant K_PHI0  : integer := 14531920;
    constant K_CPHI  : integer := 4554836;
    constant K_FPHI  : integer := 85136;
    constant K_MH0   : integer := 122098;
    constant K_MH1   : integer := 12647;
    constant K_XL0   : integer := 371979;
    constant K_CL    : integer := 38107;
    constant K_FLM   : integer := 11201;
    constant K_XR0   : integer := 715059;
    constant K_CR    : integer := 99315;
    constant K_PSIP  : integer := 17;          -- phases of the three cosines, 1024ths of a period
    constant K_PSIL  : integer := 444;
    constant K_PSIR  : integer := 502;
    constant K_XK8   : integer := 711680;      -- 2780 h, Q8
    constant K_XKH   : integer := 2780;
    constant K_YF4   : integer := 10080;
    constant K_TQ0   : integer := 21237;
    constant K_DH    : integer := 318553;
    constant K_HW    : integer := 1051;
    constant K_BL    : integer := 16182;
    constant K_HV0   : integer := 20316;
    constant K_TC    : integer := 6878658;
    constant K_TST   : integer := 60398;
    constant K_CPH   : integer := 98225;
    constant K_B00   : integer := 1216512;     -- blue ramp 297, Q12

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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    function f_clamp8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- chroma ring for the output box (written every clock, read 6 behind: never the written address)
    type t_oc is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_oc : t_oc := (others => x"8080");
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_oc : signal is true;
    signal r_wp  : unsigned(7 downto 0) := (others => '0');
    signal r_ocq : std_logic_vector(15 downto 0) := x"8080";
    signal r_ocd : std_logic_vector(15 downto 0) := x"8080";

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a : unsigned(9 downto 0) := (others => '0');
    signal r_cs_q : signed(15 downto 0) := (others => '0');
    signal r_gr_a, r_gg_a, r_gb_a : unsigned(7 downto 0) := (others => '0');
    signal r_gr_q, r_gg_q, r_gb_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(341 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(747, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_hoff, s_lat, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_sh1, z_sh3 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt3 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_b14  : unsigned(15 downto 0) := to_unsigned(16384, 16);
    signal z_s12  : unsigned(13 downto 0) := to_unsigned(4092, 14);
    signal z_f12  : unsigned(13 downto 0) := to_unsigned(4092, 14);
    signal z_t    : unsigned(25 downto 0) := (others => '0');      -- scratch (one product at a time)
    signal z_hhi, z_neg : std_logic := '0';
    signal c_x0   : unsigned(8 downto 0) := to_unsigned(8, 9);      -- X0 = x0 + 4 xs (h)
    signal c_sc   : unsigned(23 downto 0) := (others => '0');
    signal c_dsc  : unsigned(23 downto 0) := (others => '0');       -- signed step (two's complement)
    signal c_cphr : unsigned(19 downto 0) := to_unsigned(K_CPH, 20);
    signal c_tst  : unsigned(19 downto 0) := (others => '0');
    signal c_cph0 : unsigned(24 downto 0) := (others => '0');
    signal c_cst  : unsigned(21 downto 0) := (others => '0');
    signal c_et   : unsigned(8 downto 0) := (others => '0');        -- etch gain per luma step, Q16 u
    signal c_rb   : unsigned(7 downto 0) := to_unsigned(1, 8);
    signal c_rx0  : unsigned(19 downto 0) := (others => '0');       -- X0 * 205 (red ramp at pixel 0, Q12)
    signal c_bx0  : signed(21 downto 0) := to_signed(K_B00, 22);    -- 297 - X0 * 279 / 4096 (Q12)
    signal c_rys  : unsigned(14 downto 0) := (others => '0');       -- ys * 23 (red ramp per line, Q16)
    signal c_bys  : unsigned(14 downto 0) := (others => '0');       -- ys * 14
    signal c_rst  : unsigned(9 downto 0) := to_unsigned(410, 10);   -- xs * 205
    signal c_bst  : unsigned(10 downto 0) := to_unsigned(558, 11);  -- xs * 279

    -- the line engine's constants, written by the field sequencer in the zone, read by the line engine
    -- (0 tq, 1 t0, 2 phi0, 3 Cphi, 4 CL, 5 CR, 6 Fphi, 7 FL, 8 mh0, 9 mh1, 10 bl, 11 hw, 12 tc, 13 Ha, 14 N0)
    type t_kr is array (0 to 15) of std_logic_vector(31 downto 0);
    signal kram : t_kr := (others => (others => '0'));
    attribute no_rw_check of kram : signal is true;
    signal kw_e  : std_logic := '0';
    signal kw_a, kr_a : unsigned(3 downto 0) := (others => '0');
    signal kw_d  : std_logic_vector(31 downto 0) := (others => '0');
    signal kr_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_k   : unsigned(27 downto 0) := (others => '0');

    -- field serial products run on the line engine's multiplier (idle in the zone)
    signal z_fa  : unsigned(25 downto 0) := (others => '0');
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

    -- line engine (computes the next line)
    signal e_lc   : unsigned(8 downto 0) := to_unsigned(511, 9);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ar, e_ab : unsigned(22 downto 0) := (others => '0');   -- 23 vy and 14 vy (Q8 lines), unmirrored
    signal e_v4   : unsigned(14 downto 0) := (others => '0');
    signal e_dy   : unsigned(12 downto 0) := (others => '0');
    signal e_th   : unsigned(23 downto 0) := (others => '0');
    signal e_cs   : unsigned(14 downto 0) := (others => '0');      -- |cos| of the last fetch
    signal e_sg   : std_logic := '0';
    signal e_hv   : unsigned(14 downto 0) := (others => '0');
    signal e_phi  : signed(31 downto 0) := (others => '0');
    signal e_xl   : signed(21 downto 0) := (others => '0');
    signal e_xrs  : signed(21 downto 0) := (others => '0');
    signal e_xd   : signed(21 downto 0) := (others => '0');
    signal e_xr   : signed(21 downto 0) := (others => '0');
    signal e_bl   : unsigned(14 downto 0) := (others => '0');
    signal e_fan  : std_logic := '0';
    signal e_mh   : unsigned(23 downto 0) := (others => '0');
    signal m_a    : unsigned(43 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(43 downto 0) := (others => '0');
    signal m_neg  : std_logic := '0';
    -- next line's values (loaded with the line at g_start)
    signal n_u0, n_ut0 : signed(31 downto 0) := (others => '0');
    signal n_ms   : unsigned(23 downto 0) := (others => '0');
    signal n_xl2, n_xr2, n_xr, n_xlt : signed(13 downto 0) := (others => '0');
    signal n_fan  : std_logic := '0';
    signal n_h    : unsigned(8 downto 0) := (others => '0');
    signal n_r0   : unsigned(20 downto 0) := (others => '0');
    signal n_b0   : signed(21 downto 0) := (others => '0');

    -- this line's values
    signal l_ut0  : signed(31 downto 0) := (others => '0');
    signal l_ms   : unsigned(23 downto 0) := (others => '0');
    signal l_xl2, l_xr2, l_xr, l_xlt : signed(13 downto 0) := (others => '0');
    signal l_fan  : std_logic := '0';
    signal l_h    : unsigned(8 downto 0) := (others => '0');

    -- beam (stage 0): pixel index and the kink flag
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_hx   : unsigned(11 downto 0) := (others => '0');
    signal b_kon  : std_logic := '0';
    signal q_kon  : std_logic_vector(1 to 5) := (others => '0');
    -- DDAs that run at the stage that consumes them (stepped by the delayed generated avid)
    signal d_u    : signed(31 downto 0) := (others => '0');       -- stage 5
    signal d_hx   : unsigned(11 downto 0) := (others => '0');     -- stage 6
    signal d_cph  : unsigned(24 downto 0) := (others => '0');     -- stage 9
    signal d_r    : unsigned(20 downto 0) := (others => '0');     -- stage 10
    signal d_b    : signed(21 downto 0) := (others => '0');       -- stage 10

    -- pixel path
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(12 downto 0) := (others => '0');
    signal p5_le  : unsigned(16 downto 0) := (others => '0');
    signal p6_ue  : signed(31 downto 0) := (others => '0');
    signal p7_fu, p7_du : unsigned(7 downto 0) := (others => '0');
    signal p7_rok : std_logic := '0';
    signal p7_tl, p7_tr : signed(13 downto 0) := (others => '0');
    signal p7_rt, p7_kr : std_logic := '0';
    signal p8_tt, p8_trt, p8_tb, p8_trb : signed(9 downto 0) := (others => '0');
    signal p8_st  : std_logic := '0';
    signal p8_fu, p9_fu, p10_fu : unsigned(7 downto 0) := (others => '0');
    signal p9_t, p9_tr : signed(9 downto 0) := (others => '0');
    signal p9_st  : std_logic := '0';
    signal p10_t78, p10_t : signed(9 downto 0) := (others => '0');
    signal p10_am : unsigned(7 downto 0) := (others => '0');
    signal p10_inh, p10_st : std_logic := '0';
    signal p10_ph : unsigned(7 downto 0) := (others => '0');
    signal p10_par : std_logic := '0';
    signal p10_tri : unsigned(8 downto 0) := (others => '0');
    signal p10_f8 : unsigned(11 downto 0) := (others => '0');
    signal p11_on1, p11_hole, p11_inh, p11_st : std_logic := '0';
    signal p11_am : unsigned(7 downto 0) := (others => '0');
    signal p11_r, p11_b : signed(9 downto 0) := (others => '0');
    signal p11_g  : unsigned(7 downto 0) := (others => '0');
    signal p12_on : std_logic := '0';
    signal p12_rr, p12_r, p12_g, p12_b : unsigned(7 downto 0) := (others => '0');
    signal p15_r, p15_g, p15_b : std_logic_vector(31 downto 0) := (others => '0');
    signal p16_y  : unsigned(11 downto 0) := (others => '0');
    signal p16_cr, p16_cb : signed(12 downto 0) := (others => '0');
    signal s_y17  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(7 downto 0) := to_unsigned(128, 8);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_yd  : t_u10(1 to 6) := (others => to_unsigned(64, 10));
    signal s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19 : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), chroma ring, ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_oc : process(clk)
    begin
        if rising_edge(clk) then
            r_ocq <= r_oc(to_integer(r_wp - 6));      -- written 6 clocks ago; r_ocd = s_u17 / s_v17 8 clocks back
            r_ocd <= r_ocq;
            r_oc(to_integer(r_wp)) <= std_logic_vector(s_u17) & std_logic_vector(s_v17);
            r_wp <= r_wp + 1;
        end if;
    end process p_oc;

    p_kram : process(clk)
    begin
        if rising_edge(clk) then
            kr_q <= kram(to_integer(kr_a));
            if kw_e = '1' then kram(to_integer(kw_a)) <= kw_d; end if;
        end if;
    end process p_kram;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_SP_EXP2(to_integer(r_ex_a));
            r_cs_q <= C_SP_COS(to_integer(r_cs_a));
            r_gr_q <= C_SP_GR(to_integer(r_gr_a));
            r_gg_q <= C_SP_GG(to_integer(r_gg_a));
            r_gb_q <= C_SP_GB(to_integer(r_gb_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_q  : unsigned(19 downto 0);
        variable v_st : unsigned(23 downto 0);
        variable v_tq : unsigned(16 downto 0);
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
                s_mir  <= registers_in(6)(0);
                s_hoff <= registers_in(6)(1);
                s_lat  <= registers_in(6)(2);
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


            -- serial products: slot k loads at zc = 40 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at load + 18).  The slot number is decoded at 38 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 39 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' and s_zc(2 downto 0) = "110" then       -- 38 + 24 k = 6 mod 8
                case to_integer(s_zc) is
                    when 38 => z_sel <= to_unsigned(0, 5); z_pre <= '1';
                    when 62 => z_sel <= to_unsigned(1, 5); z_pre <= '1';
                    when 86 => z_sel <= to_unsigned(2, 5); z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(3, 5); z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(4, 5); z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(5, 5); z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(6, 5); z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(7, 5); z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(8, 5); z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(9, 5); z_pre <= '1';
                    when 278 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 302 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 326 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 350 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 374 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 398 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 422 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 446 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 470 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 494 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when 518 => z_sel <= to_unsigned(20, 5); z_pre <= '1';
                    when 542 => z_sel <= to_unsigned(21, 5); z_pre <= '1';
                    when 566 => z_sel <= to_unsigned(22, 5); z_pre <= '1';
                    when 590 => z_sel <= to_unsigned(23, 5); z_pre <= '1';
                    when 614 => z_sel <= to_unsigned(24, 5); z_pre <= '1';
                    when 638 => z_sel <= to_unsigned(25, 5); z_pre <= '1';
                    when 662 => z_sel <= to_unsigned(26, 5); z_pre <= '1';
                    when 686 => z_sel <= to_unsigned(27, 5); z_pre <= '1';
                    when 710 => z_sel <= to_unsigned(28, 5); z_pre <= '1';
                    when 734 => z_sel <= to_unsigned(29, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 => z_fa <= to_unsigned(K_TQ0, 26);  z_fb <= z_mt1;                       -- turns
                    when 1 => z_fa <= z_t;                     z_fb <= to_unsigned(110, 16);        -- t0 (z_t = tq)
                    when 2 => z_fa <= to_unsigned(K_PHI0, 26); z_fb <= z_b14;                       -- phi0
                    when 3 => z_fa <= to_unsigned(K_CPHI, 26); z_fb <= z_b14;
                    when 4 => z_fa <= z_t;                     z_fb <= resize(z_s12, 16);           -- Cphi
                    when 5 => z_fa <= to_unsigned(K_CL, 26);   z_fb <= resize(z_s12, 16);
                    when 6 => z_fa <= to_unsigned(K_CR, 26);   z_fb <= resize(z_s12, 16);
                    when 7 => z_fa <= to_unsigned(K_FPHI, 26); z_fb <= z_b14;
                    when 8 => z_fa <= z_t;                     z_fb <= resize(z_f12, 16);           -- Fphi
                    when 9 => z_fa <= to_unsigned(K_FLM, 26);  z_fb <= resize(z_f12, 16);
                    when 10 => z_fa <= to_unsigned(K_MH0, 26);  z_fb <= z_b14;
                    when 11 => z_fa <= to_unsigned(K_MH1, 26);  z_fb <= z_b14;
                    when 12 => z_fa <= z_t;                     z_fb <= resize(z_f12, 16);           -- mh1
                    when 13 => z_fa <= to_unsigned(K_BL, 26);   z_fb <= resize(z_f12, 16);
                    when 14 => z_fa <= to_unsigned(K_HW, 26);   z_fb <= resize(z_f12, 16);
                    when 15 => z_fa <= to_unsigned(K_TC, 26);   z_fb <= z_b14;
                    when 16 => z_fa <= to_unsigned(K_TST, 26);  z_fb <= z_b14;
                    when 17 => z_fa <= z_t;                     z_fb <= resize(s_xs, 16);            -- tail step per pixel
                    when 18 => z_fa <= to_unsigned(K_CPH, 26);  z_fb <= z_mt3;                       -- columns
                    when 19 => z_fa <= resize(c_cphr, 26);       z_fb <= resize(s_xs, 16);            -- column step per pixel
                    when 20 => z_fa <= resize(c_cphr, 26);       z_fb <= resize(c_x0, 16);            -- column phase at X0
                    when 21 => z_fa <= resize(c_cphr, 26);       z_fb <= to_unsigned(37, 16);         -- - 18.5 px
                    when 22 => z_fa <= resize(s_vm, 26);        z_fb <= resize(s_vm, 16);            -- speed^2
                    when 23 => z_fa <= resize(s_g6(12 downto 3), 26); z_fb <= resize(s_g6(12 downto 3), 16);   -- etch^2
                    when 24 => z_fa <= z_t;                     z_fb <= to_unsigned(97, 16);
                    when 25 =>                                                                            -- holes
                        if z_hhi = '1' then z_fa <= resize(s_g5(12 downto 3) - 512, 26);
                        else z_fa <= resize(s_g5(12 downto 3), 26); end if;
                        z_fb <= to_unsigned(205, 16);
                    when 26 => z_fa <= resize(c_x0, 26);        z_fb <= to_unsigned(205, 16);        -- red ramp start
                    when 27 => z_fa <= resize(c_x0, 26);        z_fb <= to_unsigned(279, 16);        -- blue ramp start
                    when 28 => z_fa <= resize(s_ys, 26);        z_fb <= to_unsigned(23, 16);         -- red ramp per line
                    when others => z_fa <= resize(s_ys, 26); z_fb <= to_unsigned(14, 16);   -- blue ramp per line
                end case;
            end if;

            kw_e <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's step), geometry of the mode
                    when 20 =>
                        c_sc <= c_sc + c_dsc;
                        c_x0 <= s_x0 + shift_left(resize(s_xs, 9), 2);
                        -- turns exponent: 7/8 (k1 - 512) (+-1.75 octaves)
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        z_e <= v_e - shift_right(v_e, 3);
                    when 21 =>
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh1 <= z_e(10 downto 8);
                        kw_e <= '1'; kw_a <= to_unsigned(14, 4); kw_d <= std_logic_vector(resize(to_unsigned(2000, 11) - resize(c_x0, 11), 32));   -- N0
                        case to_integer(s_xs) is
                            when 4      => c_rst <= to_unsigned(820, 10); c_bst <= to_unsigned(1116, 11);
                            when 3      => c_rst <= to_unsigned(615, 10); c_bst <= to_unsigned(837, 11);
                            when others => c_rst <= to_unsigned(410, 10); c_bst <= to_unsigned(558, 11);
                        end case;
                    when 23 =>
                        z_mt1 <= r_ex_q;
                        -- bands exponent: k2 - 512 (+-2 octaves)
                        v_e := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_e <= v_e;
                    when 25 =>
                        -- b14 = mt * 2^(sh - 1): 0.25 .. 4 (Q14)
                        case to_integer(z_e(10 downto 8)) is
                            when -2     => z_b14 <= shift_right(r_ex_q, 3);
                            when -1     => z_b14 <= shift_right(r_ex_q, 2);
                            when 0      => z_b14 <= shift_right(r_ex_q, 1);
                            when others => z_b14 <= r_ex_q;
                        end case;
                        -- columns exponent: 3/4 (k3 - 512) (+-1.5 octaves)
                        v_e := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        z_e <= v_e - shift_right(v_e, 2);
                    when 26 =>
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh3 <= z_e(10 downto 8);
                        c_rb <= resize(shift_right(z_b14 + 8192, 14), 8);
                    when 28 =>
                        z_mt3 <= r_ex_q;
                        -- coil: swing s = 2.5 min(1.2 p, 1), fan f = min(3 p, 1) + max(3 p - 1, 0) / 2  (Q12)
                        v_g := s_gp(12 downto 3);
                        if v_g > 853 then v_m := to_unsigned(853, 10); else v_m := v_g; end if;
                        z_s12 <= shift_left(resize(v_m, 14), 3) + shift_left(resize(v_m, 14), 2);
                        if v_g <= 341 then
                            z_f12 <= shift_left(resize(v_g, 14), 3) + shift_left(resize(v_g, 14), 2);
                        else
                            z_f12 <= to_unsigned(4092, 14) + shift_left(resize(v_g - 341, 14), 2) + shift_left(resize(v_g - 341, 14), 1);
                        end if;
                        if s_g5(12 downto 3) > 512 then z_hhi <= '1'; else z_hhi <= '0'; end if;

                    -- results (slot k at 40 + 24 k + 18)
                    when 58  =>                       -- tq = 21237 mt >> (15 - sh), sh -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => v_tq := m_acc(33 downto 17);
                            when -1     => v_tq := m_acc(32 downto 16);
                            when 0      => v_tq := m_acc(31 downto 15);
                            when others => v_tq := m_acc(30 downto 14);
                        end case;
                        z_t <= resize(v_tq, 26);
                        kw_e <= '1'; kw_a <= to_unsigned(0, 4); kw_d <= std_logic_vector(resize(v_tq, 32));
                    when 82  => kw_e <= '1'; kw_a <= to_unsigned(1, 4); kw_d <= std_logic_vector(resize((not m_acc(23 downto 0)) + 1 - c_sc, 32));   -- t0 = -110 tq - scroll
                    when 106 => kw_e <= '1'; kw_a <= to_unsigned(2, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 14), 32));   -- phi0
                    when 130 => z_t <= m_acc(39 downto 14);
                    when 154 => kw_e <= '1'; kw_a <= to_unsigned(3, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 12), 32));   -- Cphi
                    when 178 => kw_e <= '1'; kw_a <= to_unsigned(4, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 12), 32));   -- CL
                    when 202 => kw_e <= '1'; kw_a <= to_unsigned(5, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 12), 32));   -- CR
                    when 226 => z_t <= m_acc(39 downto 14);
                    when 250 => kw_e <= '1'; kw_a <= to_unsigned(6, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 12), 32));   -- Fphi
                    when 274 => kw_e <= '1'; kw_a <= to_unsigned(7, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 12), 32));   -- |FL|
                    when 298 => kw_e <= '1'; kw_a <= to_unsigned(8, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 14), 32));   -- mh0
                    when 322 => z_t <= m_acc(39 downto 14);
                    when 346 => kw_e <= '1'; kw_a <= to_unsigned(9, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 12), 32));   -- mh1
                    when 370 => kw_e <= '1'; kw_a <= to_unsigned(10, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 12), 32));  -- bl
                    when 394 => kw_e <= '1'; kw_a <= to_unsigned(11, 4); kw_d <= std_logic_vector(resize(m_acc(23 downto 12), 32));  -- hw (mod one period)
                    when 418 => kw_e <= '1'; kw_a <= to_unsigned(12, 4); kw_d <= std_logic_vector(resize(m_acc(39 downto 14), 32));  -- tc
                    when 442 => z_t <= m_acc(39 downto 14);
                    when 466 => c_tst <= m_acc(23 downto 4);
                    when 490 =>                       -- cph = 98225 mt >> (15 - sh), sh -2 .. 1
                        case to_integer(z_sh3) is
                            when -2     => c_cphr <= m_acc(36 downto 17);
                            when -1     => c_cphr <= m_acc(35 downto 16);
                            when 0      => c_cphr <= m_acc(34 downto 15);
                            when others => c_cphr <= m_acc(33 downto 14);
                        end case;
                    when 514 => c_cst <= m_acc(21 downto 0);
                    when 538 => z_t <= m_acc(25 downto 0);
                    when 562 => c_cph0 <= z_t(24 downto 0) - m_acc(24 downto 0);      -- (X0 - 37) cph, mod 2 columns
                    when 586 =>                       -- scroll step = sq + sq/8 - sq/32 (Q24 periods per field), signed
                        v_q := m_acc(19 downto 0);
                        v_st := resize(v_q + shift_right(v_q, 3) - shift_right(v_q, 5), 24);
                        if s_vneg = '1' then c_dsc <= (not v_st) + 1; else c_dsc <= v_st; end if;
                    when 610 => z_t <= m_acc(25 downto 0);
                    when 634 => c_et <= m_acc(26 downto 18);
                    when 658 =>
                        kw_e <= '1'; kw_a <= to_unsigned(13, 4);
                        if z_hhi = '1' then kw_d <= std_logic_vector(resize(to_unsigned(3277, 14) + m_acc(17 downto 4), 32));
                        else kw_d <= std_logic_vector(resize(m_acc(18 downto 5), 32)); end if;
                    when 682 => c_rx0 <= m_acc(19 downto 0);
                    when 706 => c_bx0 <= to_signed(K_B00, 22) - signed(resize(m_acc(19 downto 0), 22));
                    when 730 => c_rys <= m_acc(14 downto 0);
                    when 754 => c_bys <= m_acc(14 downto 0);
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
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
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
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17).  Products are sign-magnitude; the cosines are
    -- fetched just before use (|cos| and its sign).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_v4 : unsigned(14 downto 0);
        variable v_t  : unsigned(23 downto 0);
        variable v_p  : signed(31 downto 0);
        variable v_x  : signed(21 downto 0);
        variable v_a  : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_ar <= (others => '0'); e_ab <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            -- the serial multiplier: field products (zone slots, operands pre-muxed by p_ctrl) and the line's
            if z_ld = '1' then
                m_a <= resize(z_fa, 44); m_b <= z_fb; m_acc <= (others => '0');
            else
                if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                m_a <= m_a(42 downto 0) & '0';
                m_b <= '0' & m_b(15 downto 1);
            end if;

            -- constant reads (kram, data two clocks after the address)
            case to_integer(e_lc) is
                when 0   => kr_a <= to_unsigned(0, 4);      -- tq      (P1 at 2)
                when 16  => kr_a <= to_unsigned(1, 4);      -- t0      (e_k at 18, add at 19)
                when 17  => kr_a <= to_unsigned(11, 4);     -- hw      (P2 at 19)
                when 34  => kr_a <= to_unsigned(3, 4);      -- Cphi    (P3 at 36)
                when 50  => kr_a <= to_unsigned(2, 4);      -- phi0    (e_k at 52, add at 53)
                when 51  => kr_a <= to_unsigned(6, 4);      -- Fphi    (P4 at 53)
                when 68  => kr_a <= to_unsigned(4, 4);      -- CL      (P5 at 70)
                when 85  => kr_a <= to_unsigned(7, 4);      -- |FL|    (P6 at 87)
                when 102 => kr_a <= to_unsigned(5, 4);      -- CR      (P7 at 104)
                when 119 => kr_a <= to_unsigned(10, 4);     -- bl      (P8 at 121)
                when 154 => kr_a <= to_unsigned(13, 4);     -- Ha      (P10 at 156)
                when 171 => kr_a <= to_unsigned(9, 4);      -- mh1     (P11 at 173)
                when 187 => kr_a <= to_unsigned(8, 4);      -- mh0     (e_k at 189, add at 190)
                when 189 => kr_a <= to_unsigned(14, 4);     -- N0      (P12 at 191)
                when 240 => kr_a <= to_unsigned(12, 4);     -- tc      (e_k at 242, add at 243)
                when others => null;
            end case;
            if e_lc = 18 or e_lc = 52 or e_lc = 189 or e_lc = 242 then e_k <= unsigned(kr_q(27 downto 0)); end if;

            case to_integer(e_lc) is
                when 1 =>                     -- virtual line, Q4 (Mirror: the lower half folded up)
                    v_v4 := e_vy(18 downto 4);
                    if s_mir = '1' and v_v4 < 8640 then v_v4 := to_unsigned(17280, 15) - v_v4; end if;
                    e_v4 <= v_v4;
                    -- colour ramps (unmirrored): R0 = X0 * 205 + 23 vy / 16, B0 = 297 - X0 * 279 - 14 vy / 16 (Q12)
                    n_r0 <= c_rx0 + resize(e_ar(22 downto 4), 21);
                    n_b0 <= c_bx0 - signed(resize(e_ab(22 downto 4), 22));
                    e_ar <= e_ar + c_rys; e_ab <= e_ab + c_bys;
                when 2 =>
                    if e_v4 > K_YF4 then e_dy <= resize(e_v4 - K_YF4, 13); else e_dy <= (others => '0'); end if;
                    m_a <= resize(unsigned(kr_q(16 downto 0)), 44); m_b <= resize(e_v4, 16); m_acc <= (others => '0');
                when 19 =>
                    e_th <= m_acc(27 downto 4) + e_k(23 downto 0);
                    m_a <= resize(unsigned(kr_q(11 downto 0)), 44); m_b <= resize(e_dy, 16); m_acc <= (others => '0');
                when 20 => r_cs_a <= e_th(23 downto 14) - K_PSIP;
                when 22 =>                    -- cos(th - psi_phi)
                    v_a := r_cs_q; e_sg <= v_a(15);
                    if v_a(15) = '1' then v_a := -v_a; end if;
                    e_cs <= unsigned(v_a(14 downto 0));
                when 36 =>                    -- hole rows: cos(th + 15/790 + warp * dy)
                    v_t := e_th + K_DH + m_acc(23 downto 0);
                    r_cs_a <= v_t(23 downto 14);
                    m_a <= resize(unsigned(kr_q(25 downto 0)), 44); m_b <= resize(e_cs, 16); m_neg <= e_sg; m_acc <= (others => '0');
                when 38 =>
                    v_a := r_cs_q;
                    if v_a(15) = '1' then v_a := -v_a; end if;
                    e_cs <= unsigned(v_a(14 downto 0));
                when 39 =>                    -- |cos| - 0.62, clipped at 0
                    if e_cs > K_HV0 then e_hv <= e_cs - K_HV0; else e_hv <= (others => '0'); end if;
                when 50 => r_cs_a <= e_th(23 downto 14) - K_PSIL;
                when 52 =>                    -- cos(th - psi_L)
                    v_a := r_cs_q; e_sg <= v_a(15);
                    if v_a(15) = '1' then v_a := -v_a; end if;
                    e_cs <= unsigned(v_a(14 downto 0));
                when 53 =>
                    v_p := signed(resize(m_acc(40 downto 15), 32));
                    if m_neg = '1' then e_phi <= signed(resize(e_k, 32)) - v_p; else e_phi <= signed(resize(e_k, 32)) + v_p; end if;
                    m_a <= resize(unsigned(kr_q(19 downto 0)), 44); m_b <= resize(e_dy, 16); m_acc <= (others => '0');
                when 70 =>
                    e_phi <= e_phi + signed(resize(m_acc(35 downto 4), 32));
                    m_a <= resize(unsigned(kr_q(16 downto 0)), 44); m_b <= resize(e_cs, 16); m_neg <= e_sg; m_acc <= (others => '0');
                when 84 => r_cs_a <= e_th(23 downto 14) - K_PSIR;
                when 86 =>                    -- cos(th - psi_R)
                    v_a := r_cs_q; e_sg <= v_a(15);
                    if v_a(15) = '1' then v_a := -v_a; end if;
                    e_cs <= unsigned(v_a(14 downto 0));
                when 87 =>
                    v_x := signed(resize(m_acc(32 downto 15), 22));
                    if m_neg = '1' then e_xl <= to_signed(K_XL0, 22) - v_x; else e_xl <= to_signed(K_XL0, 22) + v_x; end if;
                    m_a <= resize(unsigned(kr_q(14 downto 0)), 44); m_b <= resize(e_dy, 16); m_acc <= (others => '0');
                when 104 =>
                    e_xl <= e_xl - signed(resize(m_acc(35 downto 8), 22));
                    m_a <= resize(unsigned(kr_q(17 downto 0)), 44); m_b <= resize(e_cs, 16); m_neg <= e_sg; m_acc <= (others => '0');
                when 121 =>
                    v_x := signed(resize(m_acc(33 downto 15), 22));
                    if m_neg = '1' then e_xrs <= to_signed(K_XR0, 22) - v_x; else e_xrs <= to_signed(K_XR0, 22) + v_x; end if;
                    m_a <= resize(unsigned(kr_q(14 downto 0)), 44); m_b <= resize(e_dy, 16); m_acc <= (others => '0');
                when 122 =>
                    e_xd <= to_signed(K_XK8, 22) - e_xrs;
                    -- window thresholds per pixel: TbL = (xL + 256 - x) >> 1, TbR = x - (xR - 128); right of xL + 500
                    n_xl2 <= resize(shift_right(e_xl, 8), 14) + 256;
                    n_xlt <= resize(shift_right(e_xl, 8), 14) + 500;
                when 138 =>                   -- blend = min(f E / 0.06, 1) (Q14); the fan from f E > 0.03
                    if m_acc(43 downto 24) /= 0 then e_bl <= to_unsigned(16384, 15);
                    else e_bl <= resize(m_acc(23 downto 10), 15); end if;
                    if m_acc(43 downto 23) /= 0 then e_fan <= '1'; else e_fan <= '0'; end if;
                when 139 =>
                    if e_xd(21) = '1' then m_a <= resize(unsigned(-e_xd), 44); else m_a <= resize(unsigned(e_xd), 44); end if;
                    m_b <= resize(e_bl, 16); m_neg <= e_xd(21); m_acc <= (others => '0');
                when 156 =>
                    v_x := signed(resize(m_acc(35 downto 14), 22));
                    if m_neg = '1' then e_xr <= e_xrs - v_x; else e_xr <= e_xrs + v_x; end if;
                    m_a <= resize(unsigned(kr_q(13 downto 0)), 44); m_b <= resize(e_hv, 16); m_acc <= (others => '0');
                when 157 =>
                    n_xr  <= resize(shift_right(e_xr, 8), 14);
                    n_xr2 <= resize(shift_right(e_xr, 8), 14) - 128;
                    n_fan <= e_fan;
                when 173 =>
                    if m_acc(43 downto 28) /= 0 then n_h <= (others => '1'); else n_h <= m_acc(27 downto 19); end if;
                    m_a <= resize(unsigned(kr_q(16 downto 0)), 44); m_b <= resize(e_dy, 16); m_acc <= (others => '0');
                when 190 =>
                    e_mh <= resize(e_k(18 downto 0), 24) + m_acc(31 downto 8);
                when 191 =>
                    m_a <= resize(e_mh, 44); m_b <= unsigned(kr_q(15 downto 0)); m_acc <= (others => '0');
                when 208 =>
                    n_u0 <= e_phi - signed(resize(m_acc(35 downto 4), 32));
                    m_a <= resize(e_mh, 44); m_b <= to_unsigned(780, 16); m_acc <= (others => '0');
                when 225 =>                   -- u at the kink, uk = phi + mh 780 >> 4, straight into the x19
                    v_p := e_phi + signed(resize(m_acc(35 downto 4), 32));
                    m_a <= resize(unsigned(v_p), 44); m_b <= to_unsigned(19, 16); m_acc <= (others => '0');
                when 243 =>                   -- tail field start: 19/32 uk + 0.41 b
                    n_ut0 <= signed(resize(m_acc(36 downto 5), 32)) + signed(resize(e_k, 32));
                    m_a <= resize(e_mh, 44); m_b <= resize(s_xs, 16); m_acc <= (others => '0');
                when 260 =>
                    n_ms <= m_acc(27 downto 4);               -- per-pixel step = mh * xs >> 4
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): the pixel index (line buffer) and the kink flag (hx >=
    -- 2780).  The per-pixel ramps are DDAs that run at the stage that consumes
    -- them, stepped by the generated avid of that stage, so no wide value
    -- rides a register pipe:  u at stage 5 (switching to the tail field at the
    -- kink on fan lines), x at stage 6, the column phase at 9, the red / blue
    -- ramps at 10.  The pixel path runs 4 pixels ahead (X0 = x0 + 4 xs) for
    -- the output filter's centring.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_h : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px  <= (others => '0');
                b_hx  <= resize(c_x0, 12);
                b_kon <= '0';
                l_ut0 <= n_ut0; l_ms <= n_ms;
                l_xl2 <= n_xl2; l_xr2 <= n_xr2; l_xr <= n_xr; l_xlt <= n_xlt; l_fan <= n_fan; l_h <= n_h;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                v_h := b_hx + s_xs;
                b_hx <= v_h;
                if v_h >= K_XKH then b_kon <= '1'; else b_kon <= '0'; end if;
            end if;
            s_lb_ra <= b_px;
            q_kon(1) <= b_kon;
            for i in 2 to 5 loop q_kon(i) <= q_kon(i - 1); end loop;

            -- stage 5: u (q_kon(5) = this pixel in the tail region, q_kon(4) = the next one)
            if g_start = '1' then
                d_u <= n_u0;
            elsif s_gav_sr(4) = '1' then
                if l_fan = '1' and q_kon(5) = '1' then
                    d_u <= d_u + signed(resize(c_tst, 32));
                elsif l_fan = '1' and q_kon(4) = '1' then
                    d_u <= l_ut0;
                else
                    d_u <= d_u + signed(resize(l_ms, 32));
                end if;
            end if;
            -- stage 6: x (h)
            if g_start = '1' then
                d_hx <= resize(c_x0, 12);
            elsif s_gav_sr(5) = '1' then
                d_hx <= d_hx + s_xs;
            end if;
            -- stage 9: the column phase (Q24 per column, bit 24 = column parity)
            if g_start = '1' then
                d_cph <= c_cph0;
            elsif s_gav_sr(8) = '1' then
                d_cph <= d_cph + resize(c_cst, 25);
            end if;
            -- stage 10: the red / blue ramps (Q12)
            if g_start = '1' then
                d_r <= n_r0; d_b <= n_b0;
            elsif s_gav_sr(9) = '1' then
                d_r <= d_r + c_rst;
                d_b <= d_b - signed(resize(c_bst, 22));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_w   : signed(31 downto 0);
        variable v_ri  : unsigned(7 downto 0);
        variable v_rt  : std_logic;
        variable v_du  : unsigned(12 downto 0);
        variable v_t   : signed(13 downto 0);
        variable v_t2  : signed(13 downto 0);
        variable v_tb  : signed(13 downto 0);
        variable v_c10 : signed(9 downto 0);
        variable v_a   : signed(13 downto 0);
        variable v_d8  : signed(7 downto 0);
        variable v_d9  : signed(8 downto 0);
        variable v_on  : std_logic;
        variable v_y   : unsigned(11 downto 0);
        variable v_c   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: luma (line buffer address at stage 1, data at 2)
            p3_lm <= unsigned(s_lb_q);
            -- 4: etch partials (luma nibbles x 9-bit gain)
            p4_ea <= resize(p3_lm(3 downto 0) * c_et, 13);
            p4_eb <= resize(p3_lm(7 downto 4) * c_et, 13);
            -- 5: etch (Q16 u)
            p5_le <= resize(p4_ea, 17) + shift_left(resize(p4_eb, 17), 4);
            -- 6: u + etch
            p6_ue <= d_u + signed(resize(p5_le & x"00", 32));
            -- 7: frac(u), distance to the nearest integer, the integer; window thresholds from x
            p7_fu <= unsigned(p6_ue(23 downto 16));
            if p6_ue(23) = '0' then p7_du <= unsigned(p6_ue(23 downto 16));
            else p7_du <= 0 - unsigned(p6_ue(23 downto 16)); end if;
            v_w := p6_ue + to_signed(8388608, 32);
            v_ri := unsigned(v_w(31 downto 24));
            if l_fan = '1' or v_ri = c_rb then p7_rok <= '1'; else p7_rok <= '0'; end if;
            p7_tl <= l_xl2 - signed(resize(d_hx, 14));                    -- 2 TbL: (xL + 256 - x)
            p7_tr <= signed(resize(d_hx, 14)) - l_xr2;                    -- TbR
            if signed(resize(d_hx, 14)) > l_xlt then p7_rt <= '1'; else p7_rt <= '0'; end if;
            if signed(resize(d_hx, 14)) > l_xr and l_fan = '1' then p7_kr <= '1'; else p7_kr <= '0'; end if;
            -- 8: window (max of left / right, 10-bit clamp), its halo form, the stripe band; tail thresholds
            v_t := shift_right(p7_tl, 1);
            if v_t > p7_tr then v_tb := v_t; else v_tb := p7_tr; end if;
            if v_tb > 511 then v_c10 := to_signed(511, 10); elsif v_tb < -512 then v_c10 := to_signed(-512, 10); else v_c10 := v_tb(9 downto 0); end if;
            p8_tb <= v_c10;
            if p7_kr = '1' then p8_trb <= to_signed(511, 10); else p8_trb <= v_c10; end if;
            if v_t >= 13 and v_t < 56 then p8_st <= '1'; else p8_st <= '0'; end if;
            v_rt := p7_rt and p7_rok;
            v_du := resize(p7_du, 13);
            if v_rt = '1' then
                v_t2 := signed(resize(shift_left(v_du, 2) + v_du + shift_right(v_du, 1) + shift_right(v_du, 4), 14));
                if v_t2 > 511 then p8_tt <= to_signed(511, 10); else p8_tt <= v_t2(9 downto 0); end if;
                v_t2 := signed(resize(to_unsigned(128, 13) + shift_left(v_du, 1) + v_du + shift_right(v_du, 1) + shift_right(v_du, 4), 14));
                if v_t2 > 511 then p8_trt <= to_signed(511, 10); else p8_trt <= v_t2(9 downto 0); end if;
            else
                p8_tt <= to_signed(511, 10); p8_trt <= to_signed(511, 10);
            end if;
            p8_fu <= p7_fu;
            -- 9: T = max(min(window, tail), -31); Tr = min(halo window, tail halo)
            if p8_tt < p8_tb then v_c10 := p8_tt; else v_c10 := p8_tb; end if;
            if v_c10 < -31 then p9_t <= to_signed(-31, 10); else p9_t <= v_c10; end if;
            if p8_trt < p8_trb then p9_tr <= p8_trt; else p9_tr <= p8_trb; end if;
            p9_st <= p8_st; p9_fu <= p8_fu;
            -- 10: T x 7/8; red amount ((1 - Tr) x 5 clamped); stripes; inside the halo; the column phase
            p10_t78 <= p9_t - shift_right(p9_t, 3);
            p10_t <= p9_t;
            if p9_tr <= 204 then p10_am <= to_unsigned(255, 8);
            elsif p9_tr >= 256 then p10_am <= (others => '0');
            else
                v_a := resize(256 - p9_tr, 14);
                v_a := shift_left(v_a, 2) + v_a;
                p10_am <= unsigned(v_a(7 downto 0));
            end if;
            if p9_tr < 256 then p10_inh <= '1'; else p10_inh <= '0'; end if;
            if p9_st = '1' or (p9_tr >= 186 and p9_tr < 218) then p10_st <= '1'; else p10_st <= '0'; end if;
            p10_ph <= d_cph(23 downto 16); p10_par <= d_cph(24);
            v_d8 := signed(d_cph(23 downto 16) - 51);                     -- hole phase, centred 0.2 into the column
            v_d9 := resize(v_d8, 9);
            if v_d9 < 0 then v_d9 := -v_d9; end if;
            p10_tri <= unsigned(v_d9(7 downto 0)) & '0';
            p10_f8 <= shift_left(resize(256 - resize(d_cph(23 downto 16), 9), 12), 3);
            p10_fu <= p9_fu;
            -- 11: the column key (saw: rise over 7/8, fall over 1/8) and the lens holes; ribbon guns
            if p10_ph < 224 then
                if signed(resize(p10_ph, 11)) > resize(p10_t78, 11) then p11_on1 <= '1'; else p11_on1 <= '0'; end if;
            else
                if signed(resize(p10_f8, 13)) > resize(p10_t, 13) then p11_on1 <= '1'; else p11_on1 <= '0'; end if;
            end if;
            if p10_tri < l_h and p10_par = '0' then p11_hole <= '1'; else p11_hole <= '0'; end if;
            p11_am <= p10_am; p11_inh <= p10_inh; p11_st <= p10_st;
            p11_r <= signed(resize(d_r(20 downto 12), 10)) - signed(resize(shift_right(p10_fu, 4), 10)) - signed(resize(shift_right(p10_fu, 6), 10));
            p11_g <= to_unsigned(10, 8) + p10_fu - shift_right(p10_fu, 3) - shift_right(p10_fu, 5);
            p11_b <= resize(d_b(21 downto 12), 10) - signed(resize(shift_right(p10_fu, 3), 10)) + signed(resize(shift_right(p10_fu, 5), 10));
            -- 12: on / Lattice; red layer; clamp the guns
            v_on := p11_on1 and not p11_hole;
            if s_lat = '1' then p12_on <= p11_inh and not v_on; else p12_on <= v_on; end if;
            if s_hoff = '1' then p12_rr <= (others => '0');
            elsif p11_st = '1' then p12_rr <= shift_right(p11_am, 5);
            else p12_rr <= p11_am - shift_right(p11_am, 5); end if;
            p12_r <= f_clamp8(p11_r); p12_g <= p11_g; p12_b <= f_clamp8(p11_b);
            -- 13: guns (Ice: red and blue swapped) -> ROM addresses
            if p12_on = '1' then
                if s_pal = '1' then r_gr_a <= p12_b; r_gb_a <= p12_r; else r_gr_a <= p12_r; r_gb_a <= p12_b; end if;
                r_gg_a <= p12_g;
            else
                if s_pal = '1' then r_gr_a <= (others => '0'); r_gb_a <= p12_rr; else r_gr_a <= p12_rr; r_gb_a <= (others => '0'); end if;
                r_gg_a <= (others => '0');
            end if;
            -- 14: ROM data (EBR); 15: fabric copies
            p15_r <= r_gr_q; p15_g <= r_gg_q; p15_b <= r_gb_q;
            -- 16: sums
            p16_y  <= resize(unsigned(p15_r(31 downto 22)), 12) + resize(unsigned(p15_g(31 downto 22)), 12) + resize(unsigned(p15_b(31 downto 22)), 12);
            p16_cr <= resize(signed(p15_r(21 downto 11)), 13) + resize(signed(p15_g(21 downto 11)), 13) + resize(signed(p15_b(21 downto 11)), 13);
            p16_cb <= resize(signed(p15_r(10 downto 0)), 13) + resize(signed(p15_g(10 downto 0)), 13) + resize(signed(p15_b(10 downto 0)), 13);
            -- 17: compose: luma 64 + Y capped at 940, chroma (8 bits) 128 + C / 4 clamped; neutral outside the generated avid
            if s_gav_sr(15) = '0' then
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(128, 8); s_v17 <= to_unsigned(128, 8);
            else
                v_y := p16_y + 64;
                if v_y > 940 then s_y17 <= to_unsigned(940, 10); else s_y17 <= v_y(9 downto 0); end if;
                v_c := p16_cr + 512;                -- 24 .. 1000 after the clamp: slice, never resize (sign fold)
                if v_c < 24 then s_u17 <= to_unsigned(6, 8); elsif v_c > 1000 then s_u17 <= to_unsigned(250, 8);
                else s_u17 <= unsigned(v_c(9 downto 2)); end if;
                v_c := p16_cb + 512;
                if v_c < 24 then s_v17 <= to_unsigned(6, 8); elsif v_c > 1000 then s_v17 <= to_unsigned(250, 8);
                else s_v17 <= unsigned(v_c(9 downto 2)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(15);     -- the same pixel as s_u17 / s_v17 (the compose gate)
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma through [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums, the sample 8 back from the block-RAM ring), luma through
    -- [1 2 1]/4 centred on the same pixel (4 back), then luma 13/16 and the
    -- blanking gate in one stage
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_s3 : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to neutral once 8 blank samples are in, so a glitched sample can't
            -- offset the colours for good.  18: sums, 19: output (centred 4 px back).
            if s_bah = "00000000" then
                s_bau <= to_unsigned(128 * 8, 11); s_bav <= to_unsigned(128 * 8, 11);
            else
                s_bau <= s_bau + s_u17 - unsigned(r_ocd(15 downto 8));
                s_bav <= s_bav + s_v17 - unsigned(r_ocd(7 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 12) + s_baud; s_u19 <= v_sc(11 downto 4);
            v_sc := resize(s_bav, 12) + s_bavd; s_v19 <= v_sc(11 downto 4);
            -- luma: s_yd(k) = s_y17 k clocks back; [1 2 1] around s_yd(5): s_y19 lags its stage by 4 pixels, as the
            -- chroma does (both centred on s_y17 six clocks back)
            s_yd(1) <= s_y17;
            for i in 2 to 6 loop s_yd(i) <= s_yd(i - 1); end loop;
            v_s3 := resize(s_yd(4), 12) + shift_left(resize(s_yd(5), 12), 1) + resize(s_yd(6), 12);
            s_y19 <= v_s3(11 downto 2);
            -- 20: luma 13/16 and the blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
                s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u20 <= s_u19 & "00"; s_v20 <= s_v19 & "00";
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture serpent;
