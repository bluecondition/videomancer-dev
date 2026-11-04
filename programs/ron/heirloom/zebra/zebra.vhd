-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Zebra v0.1.0 -- Heirloom (phase 3, video clips), from the clip still WIN_20230404_23_59_18 at 9.5 s.
--
-- The still: a neon zebra swirl.  A stripe field of nested tall diamonds (Λ chevrons with their crease at
-- x = 1000, flat horizontal stripes beyond x = 332 / 1645) runs through a folded colouriser (dark -> red ->
-- orange -> green -> pale cyan -> lavender -> pink -> violet -> blue and back, the dark folds black: the thin
-- zebra lines; the troughs black: the big cells).  Concentric arc seams sweep from the lower left to the upper
-- right, and every ribbon between two seams shows the stripe field shifted by half a cycle (a stepped radial
-- staircase into the stripe oscillator's phase), so the black cells stagger like bricks.  The stripes curl into
-- each seam, and every seam carries a dark line, a brown line and a lavender glow.  Beyond the outermost seam
-- (upper left) the stripes are dense and horizontal.  In the clip Ron sweeps the seam density down; the stripe
-- field itself does not move.
--
-- The patch, as fitted (zebra.py has every constant):
--   S   = 3.5 + sc (S0 - 3.5),  S0 = a1 u + a2 v + a4 u v + a5 v^2 + c3 u v^2 - phi   (5 terms; linear in x per line)
--   W   = f ky (|y - 520| m(x) + crease(x)) + delta floor(S) + w0 + hook(frac S) + etch luma + flow
--   F   = G tri(W) - T;  black where F < 0, else palette(fold(F));  rims on frac(S) near 0
--
--   K1  Bands     colouriser gain G: fat bands .. hairline zebra (x0.35 .. x2.8, 512 = the photo)
--   K2  Chevron   the crease: V (left) .. flat (centre) .. the photo's Λ (713) .. needles (1023)
--   K3  Shift     how far each ribbon's stripes jump at a seam: 0 .. half a cycle (512, the bricks) .. 1.35
--   K4  Speed     bipolar: the seams drift out of (right) / into (left) the swirl, the stripes flow against them
--   K5  Cells     the black threshold: no black (0) .. the photo (420) .. mostly black (1023)
--   K6  Etch      how far the camera (or the Osc) bends the stripes and the seams
--   S7  Curl      On (the photo: stripes curl into the seams, lavender glow) / Off (clean cuts)
--   S8  Mirror    the seam field mirrored about the centre: a symmetric arch
--   S9  Seams     Arcs (the photo) / Saddle: hyperbolic seams (S0 = -16 u v), two calm corners
--   S10 Colours   Neon (the photo) / Fire (the same patch's red-yellow colouriser, clip WIN_20230405_00_01_57)
--   S11 Source    Video / Osc
--   P12 SWIRL     seam density x0.36 (a few wide ribbons) .. x1 (461, the photo) .. x3.5 (shredded)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide) that writes the seam coefficients into a small
-- block RAM and refills a 1024-entry fold table (G tri - T -> {black, palette position}); a per-line engine kicked at
-- each line start (computes the NEXT line: the seam line start and step from the polynomial, the stripe line terms,
-- the side-zone ramp; 10 serial products); beam DDAs for the seam phase (reflected at the centre for Mirror), the
-- crease and the side-zone ramp; an 18-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).
-- Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-sample
-- histories ride a block-RAM ring, with the per-line resync; luma 13/16) before the blanking gate.  Colours in the
-- hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.zebra_pkg.all;
use work.all;

architecture zebra of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks
    constant C_CMP : integer := 16;   -- s_gav_sr index of the composed pixel (stage 18 data)

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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    -- per x class (0: 1080, 1: 720p, 2: SD): |u0| (Q14), du (Q23), ramp step constant
    function c_u0_f(x : unsigned(1 downto 0)) return unsigned is
    begin
        case x is
            when "10"   => return to_unsigned(C_ZB_U0_2, 16);
            when "01"   => return to_unsigned(C_ZB_U0_1, 16);
            when others => return to_unsigned(C_ZB_U0_0, 16);
        end case;
    end function;
    function c_du_f(x : unsigned(1 downto 0)) return unsigned is
    begin
        case x is
            when "10"   => return to_unsigned(C_ZB_DU_2, 16);
            when "01"   => return to_unsigned(C_ZB_DU_1, 16);
            when others => return to_unsigned(C_ZB_DU_0, 16);
        end case;
    end function;
    function c_crs_f(x : unsigned(1 downto 0)) return signed is
    begin
        case x is
            when "10"   => return to_signed(C_ZB_CRS_2, 40);
            when "01"   => return to_signed(C_ZB_CRS_1, 40);
            when others => return to_signed(C_ZB_CRS_0, 40);
        end case;
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

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_hk_a : unsigned(7 downto 0) := (others => '0');
    signal r_hk_q : unsigned(15 downto 0) := (others => '0');
    signal r_pal_a : unsigned(8 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    -- seam coefficient RAM (sequencer writes in the zone, the line engine reads; never the same clock)
    type t_cram is array (0 to 255) of std_logic_vector(31 downto 0);
    signal cram : t_cram := (others => (others => '0'));
    attribute no_rw_check of cram : signal is true;
    signal c_we  : std_logic := '0';
    signal c_wa, c_ra : unsigned(7 downto 0) := (others => '0');
    signal c_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal c_rq  : std_logic_vector(31 downto 0) := (others => '0');
    signal c_cf  : signed(31 downto 0) := (others => '0');

    -- fold table: 1024 x {black, fold position}; refilled in the zone, read by the pixel path
    type t_ftab is array (0 to 1023) of std_logic_vector(15 downto 0);
    signal ftab : t_ftab := (others => (others => '0'));
    attribute no_rw_check of ftab : signal is true;
    signal ft_we : std_logic := '0';
    signal ft_wa, r_ft_a : unsigned(9 downto 0) := (others => '0');
    signal ft_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal r_ft_q : std_logic_vector(15 downto 0) := (others => '0');
    signal ft_run, ft_v : std_logic := '0';
    signal ft_i  : unsigned(9 downto 0) := (others => '0');
    signal ft_ii : unsigned(9 downto 0) := (others => '0');
    signal ft_p  : unsigned(21 downto 0) := (others => '0');

    -- output-box history ring: the 8-sample-old inputs of both boxes
    type t_ring is array (0 to 255) of std_logic_vector(63 downto 0);
    signal rg : t_ring := (others => (others => '0'));
    attribute no_rw_check of rg : signal is true;
    signal rg_wa : unsigned(7 downto 0) := to_unsigned(6, 8);
    signal rg_ra : unsigned(7 downto 0) := (others => '0');
    signal rg_q  : std_logic_vector(63 downto 0) := (others => '0');
    signal rg_f  : std_logic_vector(63 downto 0) := (others => '0');

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
    signal s_xcl  : unsigned(1 downto 0) := (others => '0');
    signal c_x0h  : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(713 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(420 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(180 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(461 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_curloff, s_mirror, s_sad, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_shg  : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_mtg  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal z_sq   : unsigned(17 downto 0) := (others => '0');
    signal c_sc   : unsigned(15 downto 0) := to_unsigned(4096, 16);       -- seam density, Q12
    signal c_g6   : unsigned(9 downto 0) := to_unsigned(227, 10);         -- colouriser gain, Q6
    signal c_kk, c_kkr : signed(10 downto 0) := (others => '0');         -- crease slope, Q17 per half-px
    signal c_hl   : unsigned(23 downto 0) := (others => '0');            -- crease height left of the clip (Q20, wraps)
    signal c_sl, c_sr : signed(15 downto 0) := (others => '0');          -- crease steps per pixel (Q20)
    signal c_dl   : unsigned(9 downto 0) := to_unsigned(512, 10);         -- ribbon step, 1/1024 cycle
    signal c_t12  : unsigned(13 downto 0) := to_unsigned(3700, 14);       -- black threshold, Q12
    signal c_eg   : unsigned(13 downto 0) := (others => '0');            -- etch gain, Q20 W per luma step
    signal c_dstep : unsigned(17 downto 0) := (others => '0');
    signal c_fstep : unsigned(12 downto 0) := (others => '0');
    signal c_drift : unsigned(31 downto 0) := (others => '0');           -- seam phase, Q22 (wraps)
    signal c_flow  : unsigned(23 downto 0) := (others => '0');           -- stripe phase, Q20 (wraps)
    signal c_td, c_tb, c_tg, c_tg3 : unsigned(13 downto 0) := (others => '0');   -- rim widths, Q14 seam fraction

    -- field serial multiplier (signed a x unsigned b)
    signal f_a   : signed(39 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : signed(39 downto 0) := (others => '0');
    signal z_fa  : signed(27 downto 0) := (others => '0');
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

    -- line engine (serial signed a x unsigned b, result at load + 17)
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_sv   : std_logic := '0';
    signal e_av5, e_ay5 : unsigned(15 downto 0) := (others => '0');
    signal e_p    : signed(31 downto 0) := (others => '0');
    signal e_t, e_c1, e_a0, e_ss : signed(31 downto 0) := (others => '0');
    signal e_wa, e_wb : unsigned(23 downto 0) := (others => '0');
    signal m_a    : signed(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : signed(39 downto 0) := (others => '0');
    signal n_ss, n_d : signed(31 downto 0) := (others => '0');
    signal n_wa, n_wb : unsigned(23 downto 0) := (others => '0');
    signal n_r0   : unsigned(19 downto 0) := (others => '0');
    signal n_rs   : unsigned(12 downto 0) := (others => '0');

    -- beam
    signal b_x, b_x1, b_x2 : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_s   : signed(31 downto 0) := (others => '0');
    signal b_d   : signed(31 downto 0) := (others => '0');
    signal b_h   : unsigned(23 downto 0) := (others => '0');
    signal b_r   : unsigned(19 downto 0) := (others => '0');
    signal f_ref, f_hd, f_ha, f_rd, f_rz, f_ru : std_logic := '0';
    signal l_wa, l_wb : unsigned(23 downto 0) := (others => '0');
    signal l_rs  : unsigned(12 downto 0) := (others => '0');

    -- pixel path
    signal p2_s, p3_s, p4_s, p5_s : unsigned(23 downto 0) := (others => '0');
    signal p2_r0d : signed(23 downto 0) := (others => '0');
    signal p2_wa, p3_wa, p4_wa, p5_wa : unsigned(23 downto 0) := (others => '0');
    signal p2_r  : unsigned(19 downto 0) := (others => '0');
    signal p3_lm : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(17 downto 0) := (others => '0');
    signal p5_e  : unsigned(21 downto 0) := (others => '0');
    signal p6_sg : unsigned(23 downto 0) := (others => '0');
    signal p6_wa, p6_wb, p7_wa, p7_wb, p8_wa, p8_wb, p9_wa, p9_wb : unsigned(23 downto 0) := (others => '0');
    signal p7_ofa, p7_ofb : unsigned(9 downto 0) := (others => '0');
    signal p8_off : unsigned(9 downto 0) := (others => '0');
    signal p9_hk : unsigned(15 downto 0) := (others => '0');
    signal p10_w : unsigned(23 downto 0) := (others => '0');
    signal p13_ft : std_logic_vector(8 downto 0) := (others => '0');
    signal p16_q : std_logic_vector(31 downto 0) := (others => '0');
    -- flags: stages 3..6 {r0, r0line}; 7..17 {r0, hook enable, dark, brown, glow level(2:0)}
    type t_f2 is array (natural range <>) of std_logic_vector(1 downto 0);
    type t_f7 is array (natural range <>) of std_logic_vector(6 downto 0);
    signal pf2 : t_f2(3 to 6) := (others => (others => '0'));
    signal pf7 : t_f7(7 to 17) := (others => (others => '0'));
    signal p_blk : std_logic_vector(14 to 16) := (others => '0');
    signal p17_y, p17_u, p17_v : unsigned(9 downto 0) := (others => '0');
    signal p17_dy, p17_du, p17_dv : signed(13 downto 0) := (others => '0');
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := (others => '0');
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

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

    constant C_BLK_Y : unsigned(9 downto 0) := to_unsigned(C_ZB_BLACK_Y, 10);
    constant C_BLK_U : unsigned(9 downto 0) := to_unsigned(C_ZB_BLACK_U, 10);
    constant C_BLK_V : unsigned(9 downto 0) := to_unsigned(C_ZB_BLACK_V, 10);

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs, RAMs
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
            r_ex_q  <= C_ZB_EXP2(to_integer(r_ex_a));
            r_hk_q  <= C_ZB_HOOK(to_integer(r_hk_a));
            r_pal_q <= C_ZB_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            c_rq <= cram(to_integer(c_ra));
            if c_we = '1' then cram(to_integer(c_wa)) <= c_wd; end if;
        end if;
    end process p_cram;

    p_ftab : process(clk)
    begin
        if rising_edge(clk) then
            r_ft_q <= ftab(to_integer(r_ft_a));
            if ft_we = '1' then ftab(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_ftab;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rg_q <= rg(to_integer(rg_ra));
            rg(to_integer(rg_wa)) <= std_logic_vector(s_y19) & std_logic_vector(s_u19) & std_logic_vector(s_v19) &
                                     std_logic_vector(s_y21) & std_logic_vector(s_u21) & std_logic_vector(s_v21) & "0000";
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
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
                s_curloff <= registers_in(6)(0);
                s_mirror  <= registers_in(6)(1);
                s_sad     <= registers_in(6)(2);
                s_pal     <= registers_in(6)(3);
                s_src     <= registers_in(6)(4);
                s_xs      <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => c_x0h <= to_unsigned(512, 12); s_xcl <= "10"; s_oxs <= to_unsigned(204, 9);
                    when 3      => c_x0h <= to_unsigned(24, 12);  s_xcl <= "01"; s_oxs <= to_unsigned(153, 9);
                    when others => c_x0h <= to_unsigned(16, 12);  s_xcl <= "00"; s_oxs <= to_unsigned(102, 9);
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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from load + 17
            -- (used at 50 + 24 k).  The slot number is decoded at 30 + 24 k and the operands are pre-muxed into
            -- z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
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
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 486 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when 510 => z_sel <= to_unsigned(20, 5); z_pre <= '1';
                    when 534 => z_sel <= to_unsigned(21, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(signed(resize(s_gp(12 downto 3), 11)) - 460, 28); z_fb <= to_unsigned(846, 16);  -- swirl exponent
                    when 2  =>
                        if s_sad = '1' then z_fa <= to_signed(C_ZB_ZA1C, 28); else z_fa <= to_signed(C_ZB_A1C, 28); end if;
                        z_fb <= c_sc;
                    when 3  =>
                        if s_sad = '1' then z_fa <= to_signed(C_ZB_ZA4C, 28); else z_fa <= to_signed(C_ZB_A4C, 28); end if;
                        z_fb <= c_sc;
                    when 4  =>
                        if s_sad = '1' then z_fa <= to_signed(C_ZB_ZC3C, 28); else z_fa <= to_signed(C_ZB_C3C, 28); end if;
                        z_fb <= c_sc;
                    when 5  =>
                        if s_sad = '1' then z_fa <= to_signed(C_ZB_ZA2C, 28); else z_fa <= to_signed(C_ZB_A2C, 28); end if;
                        z_fb <= c_sc;
                    when 6  =>
                        if s_sad = '1' then z_fa <= to_signed(C_ZB_ZA5C, 28); else z_fa <= to_signed(C_ZB_A5C, 28); end if;
                        z_fb <= c_sc;
                    when 7  =>
                        if s_sad = '1' then z_fa <= to_signed(C_ZB_ZP35C, 28); else z_fa <= to_signed(C_ZB_P35C, 28); end if;
                        z_fb <= c_sc;
                    when 8  => z_fa <= resize(signed(resize(s_g2(12 downto 3), 11)) - 512, 28); z_fb <= to_unsigned(C_ZB_KKC, 16);  -- crease
                    when 9  => z_fa <= resize(c_kk, 28); z_fb <= to_unsigned(C_ZB_HLC, 16);                     -- crease height
                    when 10 => z_fa <= signed(resize(s_g5(12 downto 3), 28)); z_fb <= to_unsigned(9021, 16);   -- cells T
                    when 11 => z_fa <= to_signed(C_ZB_G_PHOTO_Q8, 28); z_fb <= z_mtg;                          -- bands G
                    when 13 => z_fa <= signed(resize(s_g6(12 downto 3), 28)); z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 14 => z_fa <= signed(resize(z_a, 28)); z_fb <= to_unsigned(12070, 16);
                    when 15 => z_fa <= signed(resize(s_vm, 28)); z_fb <= resize(s_vm, 16);                     -- speed^2
                    when 16 => z_fa <= signed(resize(z_sq, 28)); z_fb <= to_unsigned(845, 16);
                    when 17 => z_fa <= signed(resize(z_sq, 28)); z_fb <= to_unsigned(26, 16);
                    when 18 => z_fa <= to_signed(C_ZB_TDC, 28); z_fb <= c_sc;                                    -- rim widths
                    when 19 => z_fa <= to_signed(C_ZB_TBC, 28); z_fb <= c_sc;
                    when 20 => z_fa <= to_signed(C_ZB_TGC, 28); z_fb <= c_sc;
                    when others => z_fa <= resize(signed(resize(s_g3(12 downto 3), 11)) - 512, 28); z_fb <= to_unsigned(1744, 16);  -- shift
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 40); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            c_we <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps): the seams drift (right = outward), the stripes flow against them
                    when 20 =>
                        if s_vneg = '1' then
                            c_drift <= c_drift + resize(c_dstep, 32); c_flow <= c_flow - resize(c_fstep, 24);
                        else
                            c_drift <= c_drift - resize(c_dstep, 32); c_flow <= c_flow + resize(c_fstep, 24);
                        end if;
                    -- swirl: e = 3.3 (p12 / 1023 - 0.45) octaves, sc = 2^e (Q12)
                    when 50 => z_e <= resize(shift_right(f_acc, 10), 12);
                    when 51 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 53 => z_mt <= r_ex_q;
                    when 54 =>
                        case to_integer(z_sh) is
                            when -2     => c_sc <= shift_right(z_mt, 5);
                            when -1     => c_sc <= shift_right(z_mt, 4);
                            when 0      => c_sc <= shift_right(z_mt, 3);
                            when others => c_sc <= shift_right(z_mt, 2);
                        end case;
                    -- seam coefficients x sc -> the coefficient RAM (0 KA1, 1 KA4, 2 KC3, 3 KA2, 4 KA5, 5 K0)
                    when 98 | 122 | 146 | 170 | 194 =>
                        c_we <= '1'; c_wd <= std_logic_vector(resize(shift_right(f_acc, 12), 32));
                        case to_integer(s_zc) is
                            when 98     => c_wa <= to_unsigned(0, 8);
                            when 122    => c_wa <= to_unsigned(1, 8);
                            when 146    => c_wa <= to_unsigned(2, 8);
                            when 170    => c_wa <= to_unsigned(3, 8);
                            when others => c_wa <= to_unsigned(4, 8);
                        end case;
                    when 218 =>
                        c_we <= '1'; c_wa <= to_unsigned(5, 8);
                        c_wd <= std_logic_vector(to_signed(C_ZB_K35, 32) - resize(shift_right(f_acc, 12), 32));
                    when 242 => c_kk <= resize(shift_right(f_acc, 13), 11);
                    when 243 => c_kkr <= c_kk - shift_right(c_kk, 4) - shift_right(c_kk, 6);
                    when 244 =>                       -- crease steps: kk * 8 xs
                        case to_integer(s_xs) is
                            when 4      => c_sl <= shift_left(resize(c_kk, 16), 5); c_sr <= shift_left(resize(c_kkr, 16), 5);
                            when 3      => c_sl <= shift_left(resize(c_kk, 16), 4) + shift_left(resize(c_kk, 16), 3);
                                           c_sr <= shift_left(resize(c_kkr, 16), 4) + shift_left(resize(c_kkr, 16), 3);
                            when others => c_sl <= shift_left(resize(c_kk, 16), 4); c_sr <= shift_left(resize(c_kkr, 16), 4);
                        end case;
                    when 266 => c_hl <= unsigned(f_acc(23 downto 0));
                        -- bands exponent e = 0.75 (k1 - 512) (Q8 octaves) for slot 11
                        v_e := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        v_e := v_e - shift_right(v_e, 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_shg <= v_e(10 downto 8);
                    when 268 => z_mtg <= r_ex_q;
                    when 290 => c_t12 <= resize(unsigned(shift_right(f_acc, 10)), 14);
                    when 314 =>
                        case to_integer(z_shg) is
                            when -2     => c_g6 <= resize(unsigned(shift_right(f_acc, 19)), 10);
                            when -1     => c_g6 <= resize(unsigned(shift_right(f_acc, 18)), 10);
                            when 0      => c_g6 <= resize(unsigned(shift_right(f_acc, 17)), 10);
                            when others => c_g6 <= resize(unsigned(shift_right(f_acc, 16)), 10);
                        end case;
                    when 362 => z_a <= unsigned(f_acc(19 downto 0));
                    when 386 => c_eg <= resize(unsigned(shift_right(f_acc, 20)), 14);
                    when 410 => z_sq <= unsigned(f_acc(17 downto 0));
                    when 434 => c_dstep <= resize(unsigned(shift_right(f_acc, 10)), 18);
                    when 458 => c_fstep <= resize(unsigned(shift_right(f_acc, 10)), 13);
                    when 482 => c_td <= resize(unsigned(shift_right(f_acc, 20)), 14);
                    when 506 => c_tb <= resize(unsigned(shift_right(f_acc, 20)), 14);
                    when 530 => c_tg <= resize(unsigned(shift_right(f_acc, 20)), 14);
                    when 531 => c_tg3 <= c_tg + shift_left(c_tg, 1);
                    when 554 =>
                        if s_g3(12 downto 3) <= 512 then c_dl <= s_g3(12 downto 3);
                        else c_dl <= resize(unsigned(shift_right(f_acc, 10)), 10) + to_unsigned(512, 10); end if;
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
    -- Fold table refill (zone, zc 600 .. 1624): entry i holds F = (G * 4 i >> 6) - T
    -- folded to {black (F < 0), palette position}; P accumulates G << 2
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_f : signed(16 downto 0);
        variable v_x : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            ft_we <= '0';
            if s_zone = '1' and s_zc = 600 then
                ft_run <= '1'; ft_i <= (others => '0'); ft_p <= (others => '0');
            elsif ft_run = '1' then
                ft_p <= ft_p + shift_left(resize(c_g6, 22), 2);
                ft_i <= ft_i + 1;
                if ft_i = 1023 then ft_run <= '0'; end if;
            end if;
            ft_v <= ft_run; ft_ii <= ft_i;
            if ft_v = '1' then
                v_f := signed(resize(ft_p(21 downto 6), 17)) - signed(resize(c_t12, 17));
                v_x := unsigned(v_f(11 downto 4));
                if v_f(12) = '1' then v_x := not v_x; end if;
                ft_we <= '1'; ft_wa <= ft_ii;
                ft_wd <= "0000000" & v_f(16) & std_logic_vector(v_x);
            end if;
        end if;
    end process p_fill;

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
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (signed a, unsigned b,
    -- 16 iterations, result at load + 17).  Coefficients come from the RAM,
    -- addressed three clocks before use.
    --   C1 = KA1 + v (KA4 + v KC3),  A0 = v (KA2 + v KA5)    (v = y - 540 in 1/32 lines)
    --   S_start = K0 + A0 - C1 |u0|  (Q20),  D = C1 du  (Q30)
    --   Wy, Wr0 from |y - 520|;  WA = Wy + w0 + flow,  WB = Wr0 + o_top + flow
    --   side ramp: start R0 (Rmax, or 38 steps on SD) and step RS
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d : signed(19 downto 0);
        variable v_p : signed(31 downto 0);
        variable v_r : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1700 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);
            c_cf <= signed(c_rq);

            if e_sv = '1' then v_p := -e_p; else v_p := e_p; end if;
            case to_integer(e_lc) is
                when 0 => c_ra <= to_unsigned(2, 8);                         -- KC3
                when 1 =>
                    v_d := signed(resize(e_vy, 20)) - to_signed(540 * 256, 20);
                    if v_d < 0 then e_sv <= '1'; v_d := -v_d; else e_sv <= '0'; end if;
                    e_av5 <= resize(unsigned(v_d(17 downto 3)), 16);
                    v_d := signed(resize(e_vy, 20)) - to_signed(520 * 256, 20);
                    if v_d < 0 then v_d := -v_d; end if;
                    e_ay5 <= resize(unsigned(v_d(17 downto 3)), 16);
                when 3 =>
                    m_a <= resize(c_cf, 40); m_b <= e_av5; m_acc <= (others => '0');
                when 18 => c_ra <= to_unsigned(4, 8);                        -- KA5
                when 19 => c_ra <= to_unsigned(1, 8);                        -- KA4
                when 20 => e_p <= resize(shift_right(m_acc, 15), 32);
                when 21 =>
                    m_a <= resize(c_cf, 40); m_b <= e_av5; m_acc <= (others => '0');
                when 22 => e_t <= c_cf + v_p;                                -- t1 = KA4 + v KC3
                when 36 => c_ra <= to_unsigned(3, 8);                        -- KA2
                when 38 => e_p <= resize(shift_right(m_acc, 15), 32);
                when 39 =>
                    m_a <= resize(e_t, 40); m_b <= e_av5; m_acc <= (others => '0');
                    e_t <= c_cf + v_p;                                       -- t3 = KA2 + v KA5
                when 54 => c_ra <= to_unsigned(0, 8);                        -- KA1
                when 56 => e_p <= resize(shift_right(m_acc, 15), 32);
                when 57 =>
                    m_a <= resize(e_t, 40); m_b <= e_av5; m_acc <= (others => '0');
                    e_c1 <= c_cf + v_p;                                      -- C1 = KA1 + v t1
                when 74 => e_p <= resize(shift_right(m_acc, 15), 32);
                when 75 =>
                    m_a <= resize(e_c1, 40); m_b <= c_u0_f(s_xcl); m_acc <= (others => '0');
                    e_a0 <= v_p;                                             -- A0 = v t3
                when 90 => c_ra <= to_unsigned(5, 8);                        -- K0
                when 92 => e_p <= resize(shift_right(m_acc, 14), 32);
                when 93 =>
                    m_a <= resize(e_c1, 40); m_b <= c_du_f(s_xcl); m_acc <= (others => '0');
                    e_ss <= c_cf + e_a0;
                when 94 => n_ss <= e_ss - e_p;
                when 110 => e_p <= resize(shift_right(m_acc, 13), 32);
                when 111 =>
                    m_a <= signed(resize(e_ay5, 40)); m_b <= to_unsigned(C_ZB_CWY, 16); m_acc <= (others => '0');
                    n_d <= e_p;
                when 128 => e_p <= resize(shift_right(m_acc, 10), 32);
                when 129 =>
                    m_a <= to_signed(C_ZB_CRM, 40); m_b <= e_ay5; m_acc <= (others => '0');
                    e_wa <= unsigned(e_p(23 downto 0)) + to_unsigned(C_ZB_W0_Q20, 24);
                when 130 => n_wa <= e_wa + c_flow;
                when 146 => e_p <= resize(shift_right(m_acc, 18), 32);
                when 147 =>
                    m_a <= c_crs_f(s_xcl); m_b <= e_ay5; m_acc <= (others => '0');
                    if s_xcl /= "10" then n_r0 <= unsigned(e_p(19 downto 0)); end if;
                when 164 => e_p <= resize(shift_right(m_acc, 18), 32);
                when 165 =>
                    m_a <= signed(resize(e_ay5, 40)); m_b <= to_unsigned(C_ZB_CW0, 16); m_acc <= (others => '0');
                    n_rs <= unsigned(e_p(12 downto 0));
                    if s_xcl = "10" then                                     -- SD: the left ramp starts 38 steps in
                        v_r := shift_left(resize(unsigned(e_p(12 downto 0)), 20), 5) + shift_left(resize(unsigned(e_p(12 downto 0)), 20), 2)
                             + shift_left(resize(unsigned(e_p(12 downto 0)), 20), 1);
                        n_r0 <= v_r;
                    end if;
                when 182 => e_p <= resize(shift_right(m_acc, 9), 32);
                when 183 => e_wb <= unsigned(e_p(23 downto 0)) + to_unsigned(C_ZB_OTOP_Q20, 24);
                when 184 => n_wb <= e_wb + c_flow;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: x (half virtual px), the seam DDA S (Q22): S += D >> 8 (D reflects
    -- at the centre for Mirror), the crease DDA H (held, descending to the crease,
    -- ascending to the right clip, held) and the side-zone ramp R (down to 0 at
    -- the left clip, up from the right clip).  The zone flags for the next pixel
    -- are registered a clock ahead from x + 2 steps.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_dd : signed(31 downto 0);
        variable v_rr : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x   <= c_x0h;
                b_x1  <= c_x0h + s_xs;
                b_x2  <= c_x0h + shift_left(resize(s_xs, 12), 1);
                b_px  <= (others => '0');
                b_s   <= shift_left(n_ss, 2);
                b_d   <= n_d;
                b_h   <= c_hl;
                b_r   <= n_r0;
                f_ref <= '0'; f_hd <= '0'; f_ha <= '0'; f_rz <= '0'; f_ru <= '0';
                if s_xcl = "10" then f_rd <= '1'; else f_rd <= '0'; end if;
                l_wa <= n_wa; l_wb <= n_wb; l_rs <= n_rs;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x1;
                b_x1 <= b_x2;
                b_x2 <= b_x2 + s_xs;
                -- seam: the centre pixel's increment reflects (Mirror)
                if f_ref = '1' then v_dd := -b_d; else v_dd := b_d; end if;
                b_s <= b_s + shift_right(v_dd, 8);
                b_d <= v_dd;
                if b_x1 = 1920 and s_mirror = '1' then f_ref <= '1'; else f_ref <= '0'; end if;
                -- crease
                if f_hd = '1' then b_h <= b_h - unsigned(resize(c_sl, 24));
                elsif f_ha = '1' then b_h <= b_h + unsigned(resize(c_sr, 24)); end if;
                -- side ramp
                v_rr := signed(resize(b_r, 21)) - signed(resize(l_rs, 21));
                if f_rd = '1' then
                    if v_rr < 0 then b_r <= (others => '0'); else b_r <= unsigned(v_rr(19 downto 0)); end if;
                elsif f_rz = '1' then b_r <= (others => '0');
                elsif f_ru = '1' then b_r <= b_r + resize(l_rs, 20); end if;
                -- flags for the pixel after next (x2 becomes x1 at this edge)
                if b_x2 >= C_ZB_XL_H and b_x2 < C_ZB_XC_H then f_hd <= '1'; else f_hd <= '0'; end if;
                if b_x2 > C_ZB_XC_H and b_x2 < C_ZB_XR_H then f_ha <= '1'; else f_ha <= '0'; end if;
                if b_x2 >= C_ZB_RL_H and b_x2 < C_ZB_XL_H then f_rd <= '1'; else f_rd <= '0'; end if;
                if b_x2 >= C_ZB_XL_H and b_x2 <= C_ZB_XR_H then f_rz <= '1'; else f_rz <= '0'; end if;
                if b_x2 > C_ZB_XR_H and b_x2 <= C_ZB_RR_H then f_ru <= '1'; else f_ru <= '0'; end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n  : unsigned(9 downto 0);
        variable v_sg : unsigned(13 downto 0);
        variable v_fl : std_logic_vector(6 downto 0);
        variable v_w  : unsigned(23 downto 0);
        variable v_t  : unsigned(11 downto 0);
        variable v_by, v_bu, v_bv : unsigned(9 downto 0);
        variable v_b  : signed(10 downto 0);
        variable v_d  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: S (Q14) with the drift; the outer-ring distance; W partial (line term + crease)
            p2_s   <= unsigned(b_s(31 downto 8)) + c_drift(31 downto 8);
            p2_r0d <= b_s(31 downto 8) - to_signed(3 * 16384, 24);
            p2_wa  <= l_wa + b_h;
            p2_r   <= b_r;
            -- 3: outer-ring flags; + side ramp; luma
            p3_s <= p2_s;
            pf2(3)(1) <= not p2_r0d(23);
            if p2_r0d(23) = '0' and p2_r0d < signed(resize(c_td, 24)) then pf2(3)(0) <= '1'; else pf2(3)(0) <= '0'; end if;
            p3_wa <= p2_wa + resize(p2_r, 24);
            p3_lm <= unsigned(s_lb_q);
            -- 4: etch partials (luma nibbles x gain)
            p4_s <= p3_s; pf2(4) <= pf2(3); p4_wa <= p3_wa;
            p4_ea <= resize(p3_lm(3 downto 0) * c_eg, 18);
            p4_eb <= resize(p3_lm(7 downto 4) * c_eg, 18);
            -- 5: etch
            p5_s <= p4_s; pf2(5) <= pf2(4); p5_wa <= p4_wa;
            p5_e <= resize(p4_ea, 22) + shift_left(resize(p4_eb, 22), 4);
            -- 6: etch into S (a quarter, in seam units) and into both W branches
            p6_sg <= p5_s + resize(p5_e(21 downto 8), 24);
            p6_wa <= p5_wa + resize(p5_e, 24);
            p6_wb <= l_wb + resize(p5_e, 24);
            pf2(6) <= pf2(5);
            -- 7: ribbon n and fraction sigma: ribbon-step partials, hook address, rim flags
            v_n  := p6_sg(23 downto 14);
            v_sg := p6_sg(13 downto 0);
            p7_ofa <= resize(c_dl * v_n(4 downto 0), 10);
            p7_ofb <= resize(c_dl * v_n(9 downto 5), 10);
            r_hk_a <= v_sg(11 downto 4);
            v_fl := (others => '0');
            v_fl(6) := pf2(6)(1);                                            -- outer ring
            if pf2(6)(1) = '0' then
                if s_curloff = '0' and v_sg(13 downto 12) = "00" then v_fl(5) := '1'; end if;   -- hook
                if v_sg < c_td then v_fl(4) := '1'; end if;                                      -- dark line
                if v_sg < c_tb then v_fl(3) := '1'; end if;                                      -- brown line
                if s_curloff = '0' then                                                          -- glow level
                    if v_sg < c_tg then v_fl(2 downto 0) := "100";
                    elsif v_sg < shift_left(c_tg, 1) then v_fl(2 downto 0) := "011";
                    elsif v_sg < c_tg3 then v_fl(2 downto 0) := "010";
                    elsif v_sg < shift_left(c_tg, 2) then v_fl(2 downto 0) := "001";
                    end if;
                end if;
            elsif pf2(6)(0) = '1' then
                v_fl(4) := '1';                                              -- the outer ring's edge line
            end if;
            pf7(7) <= v_fl;
            p7_wa <= p6_wa; p7_wb <= p6_wb;
            -- 8: ribbon step (hook ROM data at 8)
            p8_off <= p7_ofa + shift_left(p7_ofb, 5);
            p8_wa <= p7_wa; p8_wb <= p7_wb; pf7(8) <= pf7(7);
            -- 9: hook fabric copy, + ribbon step
            p9_hk <= r_hk_q;
            p9_wa <= p8_wa + shift_left(resize(p8_off, 24), 10);
            p9_wb <= p8_wb; pf7(9) <= pf7(8);
            -- 10: W (the outer ring takes its own dense horizontal stripes)
            if pf7(9)(6) = '1' then
                v_w := p9_wb;
            elsif pf7(9)(5) = '1' then
                v_w := p9_wa + shift_left(resize(p9_hk, 24), 8);
            else
                v_w := p9_wa;
            end if;
            p10_w <= v_w; pf7(10) <= pf7(9);
            -- 11: tri(W) top 10 bits -> fold table address
            if p10_w(19) = '1' then v_t := p10_w(18 downto 7); else v_t := not p10_w(18 downto 7); end if;
            r_ft_a <= v_t(11 downto 2);
            pf7(11) <= pf7(10);
            -- 12: fold table data; 13: fabric copy
            pf7(12) <= pf7(11);
            p13_ft <= r_ft_q(8 downto 0); pf7(13) <= pf7(12);
            -- 14: palette address
            r_pal_a <= s_pal & unsigned(p13_ft(7 downto 0));
            p_blk(14) <= p13_ft(8); pf7(14) <= pf7(13);
            -- 15: palette ROM data; 16: fabric copy
            p_blk(15) <= p_blk(14); pf7(15) <= pf7(14);
            p16_q <= r_pal_q; p_blk(16) <= p_blk(15); pf7(16) <= pf7(15);
            -- 17: base colour, lavender glow (lav - c) * {8, 4, 2, 1} (/16 at 18)
            if p_blk(16) = '1' then
                v_by := C_BLK_Y; v_bu := C_BLK_U; v_bv := C_BLK_V;
            else
                v_by := unsigned(p16_q(31 downto 22)); v_bu := unsigned(p16_q(21 downto 12)); v_bv := unsigned(p16_q(11 downto 2));
            end if;
            p17_y <= v_by; p17_u <= v_bu; p17_v <= v_bv;
            for c in 0 to 2 loop
                if c = 0 then v_b := to_signed(C_ZB_LAV_Y, 11) - signed(resize(v_by, 11));
                elsif c = 1 then v_b := to_signed(C_ZB_LAV_U, 11) - signed(resize(v_bu, 11));
                else v_b := to_signed(C_ZB_LAV_V, 11) - signed(resize(v_bv, 11)); end if;
                case pf7(16)(2 downto 0) is
                    when "100"  => v_d := shift_left(resize(v_b, 14), 3);
                    when "011"  => v_d := shift_left(resize(v_b, 14), 2);
                    when "010"  => v_d := shift_left(resize(v_b, 14), 1);
                    when "001"  => v_d := resize(v_b, 14);
                    when others => v_d := (others => '0');
                end case;
                if c = 0 then p17_dy <= v_d; elsif c = 1 then p17_du <= v_d; else p17_dv <= v_d; end if;
            end loop;
            pf7(17) <= pf7(16);
            -- 18: rim lines over the glow (dark at the seam and on the outer ring's edge, brown after it)
            if pf7(17)(4) = '1' then
                p18_y <= to_unsigned(C_ZB_DARK_Y, 10); p18_u <= to_unsigned(C_ZB_DARK_U, 10); p18_v <= to_unsigned(C_ZB_DARK_V, 10);
            elsif pf7(17)(3) = '1' then
                p18_y <= to_unsigned(C_ZB_BROWN_Y, 10); p18_u <= to_unsigned(C_ZB_BROWN_U, 10); p18_v <= to_unsigned(C_ZB_BROWN_V, 10);
            else
                p18_y <= unsigned(signed(p17_y) + resize(shift_right(p17_dy, 4), 10));
                p18_u <= unsigned(signed(p17_u) + resize(shift_right(p17_du, 4), 10));
                p18_v <= unsigned(signed(p17_v) + resize(shift_right(p17_dv, 4), 10));
            end if;
            -- 19: compose; black outside the generated avid
            if s_gav_sr(C_CMP) = '0' then
                s_y19 <= C_BLK_Y; s_u19 <= C_BLK_U; s_v19 <= C_BLK_V;
            else
                s_y19 <= p18_y; s_u19 <= p18_u; s_v19 <= p18_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(C_CMP);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the photo's softness, and the HD HDMI bar fix),
    -- luma 13/16 and the blanking gate.  The 8-sample-old inputs come from the
    -- block-RAM ring (written every clock, read 6 behind, data + fabric register).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            rg_wa <= rg_wa + 1; rg_ra <= rg_ra + 1;
            rg_f <= rg_q;
            -- The running 8-sums snap back to the black (the off-avid input of both boxes) once 8 blank samples
            -- are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bay <= C_BLK_Y & "000"; s_bau <= C_BLK_U & "000"; s_bav <= C_BLK_V & "000";
            else
                s_bay <= s_bay + s_y19 - unsigned(rg_f(63 downto 54));
                s_bau <= s_bau + s_u19 - unsigned(rg_f(53 downto 44));
                s_bav <= s_bav + s_v19 - unsigned(rg_f(43 downto 34));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(C_CMP + 2) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= C_BLK_Y; s_u21 <= C_BLK_U; s_v21 <= C_BLK_V;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(C_CMP + 2);
            -- box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bby <= C_BLK_Y & "000"; s_bbu <= C_BLK_U & "000"; s_bbv <= C_BLK_V & "000";
            else
                s_bby <= s_bby + s_y21 - unsigned(rg_f(33 downto 24));
                s_bbu <= s_bbu + s_u21 - unsigned(rg_f(23 downto 14));
                s_bbv <= s_bbv + s_v21 - unsigned(rg_f(13 downto 4));
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

end architecture zebra;
