-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Portal v0.1.0 -- Heirloom (video phase), from the clip WIN_20231121_21_36_42 at 0.5 s.
--
-- The still: an RGB patch of the old modular synth.  Two mirrored tiles (946 px).  One parabolic ring field
-- F = |y - 576| + q (q = curvature x u^2, u = the folded tile coordinate) feeds three channel profiles: blue rings
-- (B), magenta / red domes (R) and a cyan arc (G).  Over it: a G trapezoid key (yellow bar -> green -> cyan ->
-- yellow), "eye" windows of leaning blue / green stripes beside it, a pink dome band with a red V at each cusp,
-- and a vertical oscillator S(y) (a half cosine, bunched toward y 330) cut into pink R bars and a cyan lens
-- lattice.  In the clip the bars rise (~100 lines/s) and multiply as the oscillator's level breathes.
--
--   K1  Tiles      tiles across x0.33 .. x3 (2.03 at the still)
--   K2  Domes      the curvature of the rings, arcs and dome band x1/4 .. x4
--   K3  Trapezoid  the trapezoid's width x0.4 .. x2.5
--   K4  Speed      bipolar: the bars rise (+) / sink (-); 700 = the clip's drift
--   K5  Stripes    the eye stripes' density x1/4 .. x4
--   K6  Etch       how far the camera (or the Osc, 16x) bends the rings, bars and stripes
--   S7  Flip       upside down (domes become bowls)
--   S8  Arches     the bars and lattice bend into arches along the dome curve
--   S9  Colours    Magenta / Citrus: RGB rotated (R <- B, G <- R, B <- G): yellow domes, green V, violet trapezoid
--   S10 Polarity   Positive / Negative (every channel inverted)
--   S11 Source     Video / Osc
--   P12 BARS       the bar oscillator's level: none (0) .. the still (455) .. a dense climax of bars and lattice
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line start
-- (computes the NEXT line with its own serial multiplier), two phase accumulators, a 21-stage pixel path, then the
-- shell's output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16) and the blanking gate
-- (C_LAT 24).
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid.  Colours in the hardware convention.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.portal_pkg.all;
use work.all;

architecture portal of program_top is
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
    -- the tile fold: 0 at a tile centre, 511 at the cusp
    function f_fold(p : unsigned(19 downto 0)) return unsigned is
    begin
        if p(19) = '1' then return not p(18 downto 10); else return p(18 downto 10); end if;
    end function;
    function f_c8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;

    -- line buffer: (luma >> 2) - 16 of every other source pixel (or the Osc lattice)
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_sq_a : unsigned(8 downto 0) := (others => '0');
    signal r_sq_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_ey_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_gt_a : unsigned(9 downto 0) := (others => '0');
    signal r_gt_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_pb_a, r_pr_a : unsigned(9 downto 0) := (others => '0');
    signal r_pb_q, r_pr_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_po_a : unsigned(8 downto 0) := (others => '0');
    signal r_po_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_mr_a, r_mg_a, r_mb_a : unsigned(7 downto 0) := (others => '0');
    signal r_mr_q, r_mg_q, r_mb_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(455 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_flp, s_arc, s_rot, s_neg, s_src : std_logic := '0';

    -- per-field constants
    signal z_k1b  : unsigned(11 downto 0) := to_unsigned(1108, 12);
    signal z_n960 : unsigned(9 downto 0) := to_unsigned(960, 10);
    signal z_e1   : signed(5 downto 0) := (others => '0');
    signal z_e2   : signed(6 downto 0) := to_signed(5, 7);
    signal z_e3   : signed(4 downto 0) := (others => '0');
    signal z_e5   : unsigned(6 downto 0) := to_unsigned(35, 7);
    signal z_ep   : unsigned(5 downto 0) := (others => '0');
    signal z_lf   : unsigned(8 downto 0) := (others => '0');
    signal z_br   : signed(7 downto 0) := (others => '0');            -- breath LFO -96 .. 96
    signal z_br15 : unsigned(10 downto 0) := (others => '0');          -- |br| x 15
    signal z_na   : unsigned(11 downto 0) := to_unsigned(1008, 12);    -- bar level (Q8)
    signal c_ck   : unsigned(12 downto 0) := to_unsigned(1108, 13);    -- tile phase step per pixel (Q20 cycles)
    signal c_ph   : unsigned(19 downto 0) := (others => '0');          -- phase of the first pixel
    signal c_pht  : unsigned(19 downto 0) := (others => '0');          -- ... the trapezoid's (13 px left)
    signal c_s2   : signed(2 downto 0) := (others => '0');             -- curvature: q = sq x (16 + m2) >> (13 - s2)
    signal c_m2   : unsigned(3 downto 0) := to_unsigned(5, 4);
    signal c_s3   : signed(1 downto 0) := (others => '0');             -- trapezoid: hw = hwb x (8 + m3) >> (3 - s3)
    signal c_m3   : unsigned(2 downto 0) := (others => '0');
    signal c_sfq  : unsigned(8 downto 0) := to_unsigned(76, 9);        -- stripe step (Q12 per u)
    signal c_slk  : unsigned(7 downto 0) := to_unsigned(16, 8);        -- stripe lean per line
    signal c_eg   : unsigned(7 downto 0) := to_unsigned(9, 8);         -- etch gain
    signal c_nab  : unsigned(12 downto 0) := to_unsigned(1008, 13);    -- bar level with the breath (Q8)
    signal c_smax : unsigned(17 downto 0) := (others => '0');
    signal c_s720 : unsigned(17 downto 0) := (others => '0');
    signal c_bph  : unsigned(11 downto 0) := to_unsigned(1844, 12);    -- bar phase (Q12)
    signal c_drift : signed(9 downto 0) := (others => '0');
    signal c_bh0  : unsigned(9 downto 0) := (others => '0');           -- bar half width (Q12)
    signal c_dt   : unsigned(8 downto 0) := to_unsigned(222, 9);       -- dome band
    signal c_db   : unsigned(8 downto 0) := to_unsigned(340, 9);
    signal c_ydr  : unsigned(8 downto 0) := to_unsigned(357, 9);

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(31 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
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

    -- line engine (with its own serial multiplier)
    signal e_zk   : std_logic := '0';
    signal e_y    : unsigned(19 downto 0) := (others => '0');           -- virtual line, Q8
    signal e_run  : std_logic := '0';
    signal e_st   : unsigned(2 downto 0) := (others => '0');
    signal e_pk   : unsigned(3 downto 0) := (others => '0');
    signal e_pp   : unsigned(4 downto 0) := (others => '0');
    signal e_yv   : unsigned(10 downto 0) := (others => '0');
    signal e_gq   : unsigned(15 downto 0) := (others => '0');
    signal e_d400, e_d720, e_d330, e_d510 : unsigned(9 downto 0) := (others => '0');
    signal e_n400, e_n720, e_n330 : std_logic := '0';
    signal e_ysq  : unsigned(21 downto 0) := (others => '0');
    signal e_hwb  : unsigned(9 downto 0) := (others => '0');
    signal l_a    : unsigned(31 downto 0) := (others => '0');
    signal l_b    : unsigned(15 downto 0) := (others => '0');
    signal l_acc  : unsigned(31 downto 0) := (others => '0');
    signal n_dy   : unsigned(9 downto 0) := (others => '0');
    signal n_top, n_lowl, n_veel, n_drl : std_logic := '0';
    signal n_yv   : unsigned(10 downto 0) := (others => '0');
    signal n_hw   : unsigned(9 downto 0) := (others => '0');
    signal n_s    : unsigned(17 downto 0) := (others => '0');
    signal n_lw, n_ur : signed(10 downto 0) := (others => '0');
    signal n_vth  : unsigned(8 downto 0) := (others => '0');
    signal n_gl   : unsigned(7 downto 0) := (others => '0');
    signal n_sl   : unsigned(11 downto 0) := (others => '0');

    -- beam and line registers
    signal b_ph, b_pt : unsigned(19 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_dy  : unsigned(9 downto 0) := (others => '0');
    signal l_top, l_lowl, l_veel, l_drl : std_logic := '0';
    signal l_yv  : unsigned(10 downto 0) := (others => '0');
    signal l_hw  : unsigned(9 downto 0) := (others => '0');
    signal l_s   : unsigned(17 downto 0) := (others => '0');
    signal l_lw, l_ur : signed(10 downto 0) := (others => '0');
    signal l_vth : unsigned(8 downto 0) := (others => '0');
    signal l_gl  : unsigned(7 downto 0) := (others => '0');
    signal l_sl  : unsigned(11 downto 0) := (others => '0');

    -- pixel path
    signal p2_u, p2_ut : unsigned(8 downto 0) := (others => '0');
    signal p2_sd : std_logic := '0';
    signal p2_l1, p2_l2, p2_l3 : unsigned(7 downto 0) := (others => '0');
    signal p3_u, p3_ut : unsigned(8 downto 0) := (others => '0');
    signal p3_hwj : unsigned(10 downto 0) := (others => '0');
    signal p3_lm  : signed(8 downto 0) := (others => '0');
    signal p3_trap, p3_low, p3_vee, p3_ur : std_logic := '0';
    signal p4_sq  : unsigned(15 downto 0) := (others => '0');
    signal p4_ey  : std_logic_vector(15 downto 0) := (others => '0');
    signal p4_et  : signed(16 downto 0) := (others => '0');
    signal p4_dg  : signed(11 downto 0) := (others => '0');
    signal p4_pg, p4_ein : std_logic := '0';
    signal p5_qa, p5_qb : unsigned(19 downto 0) := (others => '0');
    signal p5_s16 : unsigned(19 downto 0) := (others => '0');
    signal p5_e   : signed(10 downto 0) := (others => '0');
    signal p5_spl : unsigned(13 downto 0) := (others => '0');
    signal p5_sph : unsigned(12 downto 0) := (others => '0');
    signal p5_ur  : unsigned(8 downto 0) := (others => '0');
    signal p5_ge  : unsigned(7 downto 0) := (others => '0');
    signal p6_qs  : unsigned(20 downto 0) := (others => '0');
    signal p6_s16 : unsigned(19 downto 0) := (others => '0');
    signal p6_sp  : unsigned(11 downto 0) := (others => '0');
    signal p6_dq, p6_fr : signed(12 downto 0) := (others => '0');
    signal p6_e   : signed(10 downto 0) := (others => '0');
    signal p7_qm  : unsigned(21 downto 0) := (others => '0');
    signal p7_sph : unsigned(11 downto 0) := (others => '0');
    signal p7_dq  : signed(12 downto 0) := (others => '0');
    signal p8_q   : unsigned(9 downto 0) := (others => '0');
    signal p8_tri : unsigned(7 downto 0) := (others => '0');
    signal p8_dq  : signed(12 downto 0) := (others => '0');
    signal p9_F   : signed(12 downto 0) := (others => '0');
    signal p9_q   : unsigned(9 downto 0) := (others => '0');
    signal p9_qa  : unsigned(9 downto 0) := (others => '0');
    signal p9_qb  : unsigned(6 downto 0) := (others => '0');
    signal p9_a29 : unsigned(14 downto 0) := (others => '0');
    signal p9_sv  : unsigned(7 downto 0) := (others => '0');
    signal p9_pr  : std_logic_vector(7 downto 0) := (others => '0');
    signal p9_dq  : signed(12 downto 0) := (others => '0');
    signal p10_q8 : unsigned(9 downto 0) := (others => '0');
    signal p10_F  : signed(12 downto 0) := (others => '0');
    signal p10_a1 : std_logic := '0';
    signal p10_aq : unsigned(11 downto 0) := (others => '0');
    signal p10_dq : signed(12 downto 0) := (others => '0');
    signal p10_t  : unsigned(8 downto 0) := (others => '0');
    signal p10_au : unsigned(8 downto 0) := (others => '0');
    signal p10_ao : std_logic := '0';
    signal p11_F2 : signed(12 downto 0) := (others => '0');
    signal p11_yd : signed(11 downto 0) := (others => '0');
    signal p11_t12 : unsigned(11 downto 0) := (others => '0');
    signal p11_a1 : std_logic := '0';
    signal p11_bt : unsigned(16 downto 0) := (others => '0');
    signal p11_hp : unsigned(15 downto 0) := (others => '0');
    signal p12_ins, p12_arc, p12_dome : std_logic := '0';
    signal p12_phb : unsigned(11 downto 0) := (others => '0');
    signal p12_sc : signed(19 downto 0) := (others => '0');
    signal p12_pb : std_logic_vector(7 downto 0) := (others => '0');
    signal p12_bh, p13_bh : signed(10 downto 0) := (others => '0');
    signal p12_hl, p13_hl : unsigned(11 downto 0) := (others => '0');
    signal p13_d  : unsigned(10 downto 0) := (others => '0');
    signal p13_band, p13_pz : std_logic := '0';
    signal p14_po : std_logic_vector(7 downto 0) := (others => '0');
    signal p14_bar, p14_lat : std_logic := '0';
    signal p15_r, p15_g, p15_b : unsigned(7 downto 0) := (others => '0');
    signal p20_mr, p20_mg, p20_mb : std_logic_vector(31 downto 0) := (others => '0');
    signal p21_y  : unsigned(10 downto 0) := (others => '0');
    signal p21_u, p21_v : signed(12 downto 0) := (others => '0');
    signal p22_y, p22_u, p22_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- values carried down the path
    -- flags: bit 0 trap, 1 pillar (geometry), 2 eye, 3 low, 4 vee, 5 dome-R window
    type t_fl is array (4 to 14) of std_logic_vector(5 downto 0);
    signal p_fl  : t_fl := (others => (others => '0'));
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal p_ge  : t_u8a(6 to 14) := (others => (others => '0'));     -- eye glow
    signal p_sv  : t_u8a(10 to 14) := (others => (others => '0'));    -- stripes
    signal p_pr  : t_u8a(10 to 14) := (others => (others => '0'));    -- R inside the arc
    type t_bit_a is array (natural range <>) of std_logic;
    signal p_top : t_bit_a(4 to 9) := (others => '0');
    signal p_ins, p_arc, p_dome : t_bit_a(13 to 14) := (others => '0');
    signal p_pb  : t_u8a(13 to 14) := (others => (others => '0'));
    type t_u9a is array (natural range <>) of unsigned(8 downto 0);
    signal p_u   : t_u9a(4 to 9) := (others => (others => '0'));       -- u for the late bar / lattice widths
    type t_u12a is array (natural range <>) of unsigned(11 downto 0);
    signal p_x0  : t_u12a(6 to 10) := (others => (others => '0'));    -- bar phase offset (bph + 32 E) mod 4096

    -- output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums, the 8-back tap from a
    -- block-RAM ring), luma [1 2 1]/4 on taps aligned with the chroma centre, luma 13/16
    type t_cring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal cr : t_cring := (others => (others => '0'));
    signal cr_q, cr_f : std_logic_vector(19 downto 0) := (others => '0');
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal sr : t_sring := (others => (others => '1'));
    signal sr_q, sr_f : std_logic_vector(2 downto 0) := (others => '1');
    attribute no_rw_check : boolean;
    attribute no_rw_check of cr : signal is true;
    attribute no_rw_check of sr : signal is true;
    signal rc     : unsigned(7 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly   : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y25, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y26, s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);

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
            r_sq_q <= C_PO_SQ(to_integer(r_sq_a));
            r_ey_q <= C_PO_EY(to_integer(r_sq_a));
            r_gt_q <= C_PO_GT(to_integer(r_gt_a));
            r_pb_q <= C_PO_PB(to_integer(r_pb_a));
            r_pr_q <= C_PO_PR(to_integer(r_pr_a));
            r_po_q <= C_PO_PO(to_integer(r_po_a));
            r_mr_q <= C_PO_MR(to_integer(r_mr_a));
            r_mg_q <= C_PO_MG(to_integer(r_mg_a));
            r_mb_q <= C_PO_MB(to_integer(r_mb_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_t  : signed(16 downto 0);
        variable v_u  : unsigned(19 downto 0);
        variable v_e  : signed(6 downto 0);
        variable v_b  : signed(9 downto 0);
        variable v_bp : unsigned(6 downto 0);
        variable v_g  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_tid  <= registers_in(8)(3 downto 0);
            s_ys   <= f_vstep_y(s_tid);
            s_xsr  <= f_vstep_x(s_tid);

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
            -- speed: bipolar, +-8 deadband
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
                s_flp <= registers_in(6)(0);
                s_arc <= registers_in(6)(1);
                s_rot <= registers_in(6)(2);
                s_neg <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); z_k1b <= to_unsigned(2217, 12); z_n960 <= to_unsigned(360, 10);
                    when 3      => s_oxs <= to_unsigned(153, 9); z_k1b <= to_unsigned(1662, 12); z_n960 <= to_unsigned(640, 10);
                    when others => s_oxs <= to_unsigned(102, 9); z_k1b <= to_unsigned(1108, 12); z_n960 <= to_unsigned(960, 10);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is                              -- Osc lattice step per line = vstep 121 / 256
                when 0 => s_oys <= to_unsigned(537, 10); when 4 => s_oys <= to_unsigned(272, 10);
                when 8 => s_oys <= to_unsigned(453, 10); when 12 => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10); when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others => s_oys <= to_unsigned(121, 10);
            end case;

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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result read at 50 + 24 k.
            -- The slot number is decoded at 30 + 24 k and the operands are pre-muxed into z_fa / z_fb at
            -- 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_k1b, 32);  z_fb <= resize(unsigned(z_e1(3 downto 0)), 16) + 16;   -- tile step
                    when 1  => z_fa <= resize(c_ck, 32);   z_fb <= resize(z_n960, 16);                           -- start phase
                    when 2  => z_fa <= resize(z_na, 32);   z_fb <= resize(z_br15, 16);                           -- breath
                    when 3  => z_fa <= resize(c_nab, 32);  z_fb <= to_unsigned(20104, 16);                       -- S at y 880
                    when 4  => z_fa <= resize(c_nab, 32);  z_fb <= to_unsigned(11628, 16);                       -- S at y 720
                    when 5  => z_fa <= resize(s_vm, 32);   z_fb <= resize(s_vm, 16);                             -- speed^2
                    when 6  => z_fa <= f_acc;              z_fb <= to_unsigned(81, 16);
                    when 7  => z_fa <= resize(s_g6(12 downto 3), 32); z_fb <= resize(s_g6(12 downto 3), 16);    -- etch^2
                    when others => z_fa <= resize(c_sfq, 32); z_fb <= to_unsigned(55, 16);                       -- stripe lean
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion: last field's drift, the breath LFO step; exponents
                    when 20 =>
                        c_bph <= unsigned(signed(c_bph) + resize(c_drift, 12));
                        if z_lf = 383 then z_lf <= (others => '0'); else z_lf <= z_lf + 1; end if;
                        v_d := signed(resize(s_g1(12 downto 3), 11)) - 512;                 -- tiles: 51 / 1024
                        v_t := shift_left(resize(v_d, 17), 5) + shift_left(resize(v_d, 17), 4) + shift_left(resize(v_d, 17), 1)
                               + resize(v_d, 17) + 512;
                        z_e1 <= resize(shift_right(v_t, 10), 6);
                        v_d := signed(resize(s_g2(12 downto 3), 11)) - 512;                 -- domes: 64 / 1024
                        v_t := resize(v_d, 17) + 8;
                        z_e2 <= resize(shift_right(v_t, 4), 7) + 5;
                        v_d := signed(resize(s_g3(12 downto 3), 11)) - 512;                 -- trapezoid: 21 / 1024
                        v_t := shift_left(resize(v_d, 17), 4) + shift_left(resize(v_d, 17), 2) + resize(v_d, 17) + 512;
                        z_e3 <= resize(shift_right(v_t, 10), 5);
                        v_d := signed(resize(s_g5(12 downto 3), 11)) - 512;                 -- stripes: 48 / 1024
                        v_t := shift_left(resize(v_d, 17), 5) + shift_left(resize(v_d, 17), 4) + 512;
                        v_e := resize(shift_right(v_t, 10), 7) + 35;
                        z_e5 <= unsigned(v_e);
                        v_g := s_gp(12 downto 3);
                        z_ep <= resize(shift_right(resize(v_g, 12) + shift_left(resize(v_g, 12), 1), 6), 6);   -- (p12 x 3) >> 6
                        if v_g < 16 then c_bh0 <= (others => '0');
                        elsif v_g >= 132 then c_bh0 <= to_unsigned(704, 10);
                        else c_bh0 <= resize(shift_left(v_g, 2), 10) + 176; end if;
                    when 21 =>
                        if z_lf < 96 then v_b := signed(resize(z_lf, 10));
                        elsif z_lf < 288 then v_b := 192 - signed(resize(z_lf, 10));
                        else v_b := signed(resize(z_lf, 10)) - 384; end if;
                        z_br <= resize(v_b, 8);
                        if v_b < 0 then v_bp := (others => '0'); else v_bp := unsigned(v_b(6 downto 0)); end if;
                        c_dt  <= 222 + resize(shift_right(resize(v_bp, 9) + shift_left(resize(v_bp, 9), 2), 4), 9);
                        c_db  <= 340 - resize(v_bp(6 downto 3), 9);
                        c_ydr <= 357 - resize(v_bp(6 downto 3), 9);
                        z_na <= resize(shift_right(shift_left(resize(z_ep(3 downto 0), 15) + 16, 8)
                                + shift_left(resize(z_ep(3 downto 0), 15) + 16, 7), 4 - to_integer(z_ep(5 downto 4))), 12);
                        c_sfq <= resize(shift_left(resize(z_e5(3 downto 0), 9) + 16, to_integer(z_e5(6 downto 4))), 9);
                        c_s2 <= resize(shift_right(z_e2, 4), 3); c_m2 <= unsigned(z_e2(3 downto 0));
                        c_s3 <= resize(shift_right(z_e3, 3), 2); c_m3 <= unsigned(z_e3(2 downto 0));
                    when 22 =>
                        if z_br < 0 then v_u := resize(unsigned(-resize(z_br, 9)), 20); else v_u := resize(unsigned(z_br), 20); end if;
                        z_br15 <= resize(shift_left(v_u, 4) - v_u, 11);

                    -- products
                    when 50  =>
                        case z_e1(5 downto 4) is
                            when "10"   => c_ck <= resize(shift_right(f_acc, 6), 13);
                            when "11"   => c_ck <= resize(shift_right(f_acc, 5), 13);
                            when "00"   => c_ck <= resize(shift_right(f_acc, 4), 13);
                            when others => c_ck <= resize(shift_right(f_acc, 3), 13);
                        end case;
                    when 74  => c_ph <= to_unsigned(461111, 20) - f_acc(19 downto 0);
                    when 75  => c_pht <= c_ph + 14408;
                    when 98  =>
                        if z_br < 0 then c_nab <= resize(z_na, 13) - resize(shift_right(f_acc, 12), 13);
                        else c_nab <= resize(z_na, 13) + resize(shift_right(f_acc, 12), 13); end if;
                    when 122 => c_smax <= resize(shift_right(f_acc, 10), 18);
                    when 146 => c_s720 <= resize(shift_right(f_acc, 10), 18);
                    when 194 =>
                        if s_vneg = '1' then c_drift <= -signed(resize(shift_right(f_acc, 16), 10));
                        else c_drift <= signed(resize(shift_right(f_acc, 16), 10)); end if;
                    when 218 => c_eg <= resize(shift_right(f_acc, 12), 8);
                    when 242 => c_slk <= resize(shift_right(f_acc, 8), 8);
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
    -- write of (luma >> 2) - 16 for every other source pixel
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
            s_lb_we <= data_in.avid and not s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
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
    -- computes the NEXT line's terms with a serial multiplier (9 products, ~190 clocks):
    --   yv (Flip: 1079 - yv), dy = |yv - 576|, the trapezoid half width hw = (35 + yv^2 x 221 / 2^20) x gain,
    --   the bar oscillator S = (1 - cos) x level, the stripe lean, and the window / V / glow line constants
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : unsigned(10 downto 0);
        variable v_h  : unsigned(13 downto 0);
        variable v_s  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' or g_start = '1' then
                if e_zk = '1' then e_y <= (others => '0'); else e_y <= e_y + resize(s_ys, 20); end if;
                e_run <= '1'; e_st <= (others => '0');
                e_pk <= (others => '0'); e_pp <= (others => '0');
            elsif e_run = '1' then
                case to_integer(e_st) is
                    when 0 =>                                                       -- yv, GT address
                        v_y := e_y(18 downto 8);
                        if s_flp = '1' then v_y := 1079 - v_y; end if;
                        e_yv <= v_y;
                        if v_y > 330 then r_gt_a <= resize(v_y - 330, 10); else r_gt_a <= (others => '0'); end if;
                        e_st <= "001";
                    when 1 =>                                                       -- distances, simple line flags
                        if e_yv >= 576 then n_dy <= resize(e_yv - 576, 10); n_top <= '0';
                        else n_dy <= resize(576 - e_yv, 10); n_top <= '1'; end if;
                        n_yv <= e_yv;
                        if e_yv >= 400 then e_d400 <= resize(e_yv - 400, 10); e_n400 <= '0';
                        else e_d400 <= resize(400 - e_yv, 10); e_n400 <= '1'; end if;
                        if e_yv >= 720 then e_d720 <= resize(e_yv - 720, 10); e_n720 <= '0';
                        else e_d720 <= resize(720 - e_yv, 10); e_n720 <= '1'; end if;
                        if e_yv >= 330 then e_d330 <= resize(e_yv - 330, 10); e_n330 <= '0';
                        else e_d330 <= resize(330 - e_yv, 10); e_n330 <= '1'; end if;
                        if e_yv >= 510 then e_d510 <= resize(e_yv - 510, 10); else e_d510 <= resize(510 - e_yv, 10); end if;
                        if e_yv < 940 then n_lowl <= '1'; else n_lowl <= '0'; end if;
                        if e_yv > 240 and e_yv < 330 then n_veel <= '1'; else n_veel <= '0'; end if;
                        if e_yv < c_ydr then n_drl <= '1'; else n_drl <= '0'; end if;
                        e_st <= "010";
                    when 2 =>
                        e_gq <= unsigned(r_gt_q);
                        e_st <= "011";
                    when others =>                                                  -- serial products
                        if e_pp = 0 then
                            l_acc <= (others => '0');
                            case to_integer(e_pk) is
                                when 0 => l_a <= resize(e_yv, 32);   l_b <= resize(e_yv, 16);            -- yv^2
                                when 1 => l_a <= resize(e_ysq, 32);  l_b <= to_unsigned(221, 16);
                                when 2 => l_a <= resize(e_hwb, 32);  l_b <= resize(c_m3, 16) + 8;        -- hw gain
                                when 3 => l_a <= resize(e_gq, 32);   l_b <= resize(c_nab, 16);           -- S
                                when 4 => l_a <= resize(e_d400, 32); l_b <= resize(c_slk, 16);           -- stripe lean
                                when 5 => l_a <= resize(e_d720, 32); l_b <= to_unsigned(155, 16);        -- lower windows
                                when 6 => l_a <= resize(e_d330, 32); l_b <= to_unsigned(186, 16);        -- V
                                when 7 => l_a <= resize(e_d330, 32); l_b <= to_unsigned(139, 16);        -- dome R end
                                when others => l_a <= resize(e_d510, 32); l_b <= to_unsigned(299, 16);   -- glow
                            end case;
                        elsif e_pp <= 16 then
                            if l_b(0) = '1' then l_acc <= l_acc + l_a; end if;
                            l_a <= l_a(30 downto 0) & '0';
                            l_b <= '0' & l_b(15 downto 1);
                        elsif e_pp = 17 then
                            case to_integer(e_pk) is
                                when 0 => e_ysq <= l_acc(21 downto 0);
                                when 1 => e_hwb <= resize(shift_right(l_acc, 20), 10) + 35;
                                when 2 =>
                                    v_h := resize(l_acc, 14);
                                    case c_s3 is
                                        when "10"   => v_h := shift_right(v_h, 5);
                                        when "11"   => v_h := shift_right(v_h, 4);
                                        when "00"   => v_h := shift_right(v_h, 3);
                                        when others => v_h := shift_right(v_h, 2);
                                    end case;
                                    if v_h > 1023 then n_hw <= (others => '1'); else n_hw <= v_h(9 downto 0); end if;
                                when 3 => n_s <= resize(shift_right(l_acc, 10), 18);
                                when 4 =>
                                    if e_n400 = '1' then n_sl <= unsigned(-signed(l_acc(11 downto 0)));
                                    else n_sl <= l_acc(11 downto 0); end if;
                                when 5 =>
                                    if e_n720 = '1' then n_lw <= 447 + signed(resize(shift_right(l_acc, 8), 11));
                                    else n_lw <= 447 - signed(resize(shift_right(l_acc, 8), 11)); end if;
                                when 6 =>
                                    if e_n330 = '1' then n_vth <= resize(shift_right(l_acc, 8), 9);
                                    else n_vth <= (others => '0'); end if;
                                when 7 =>
                                    if e_n330 = '1' then n_ur <= 424 - signed(resize(shift_right(l_acc, 8), 11));
                                    else n_ur <= 424 + signed(resize(shift_right(l_acc, 8), 11)); end if;
                                when others =>
                                    v_s := 262 - signed(resize(shift_right(l_acc, 8), 12));
                                    if v_s < 0 then n_gl <= (others => '0');
                                    elsif v_s > 175 then n_gl <= to_unsigned(175, 8);
                                    else n_gl <= unsigned(v_s(7 downto 0)); end if;
                                    e_run <= '0';
                            end case;
                        end if;
                        if e_pp = 18 then
                            e_pp <= (others => '0'); e_pk <= e_pk + 1;
                        else
                            e_pp <= e_pp + 1;
                        end if;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the tile phase (Q20 cycles) and the trapezoid's, the line registers
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ph <= c_ph; b_pt <= c_pht;
                b_px <= (others => '0');
                l_dy <= n_dy; l_top <= n_top; l_lowl <= n_lowl; l_veel <= n_veel; l_drl <= n_drl; l_yv <= n_yv;
                l_hw <= n_hw; l_s <= n_s; l_lw <= n_lw; l_ur <= n_ur; l_vth <= n_vth; l_gl <= n_gl;
                l_sl <= n_sl;
            elsif g_avid = '1' then
                b_ph <= b_ph + resize(c_ck, 20);
                b_pt <= b_pt + resize(c_ck, 20);
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px(10 downto 1);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_l4 : unsigned(9 downto 0);
        variable v_e  : signed(16 downto 0);
        variable v_q  : unsigned(21 downto 0);
        variable v_r, v_g, v_b : unsigned(7 downto 0);
        variable v_y  : unsigned(10 downto 0);
        variable v_c  : signed(12 downto 0);
        variable v_f  : signed(13 downto 0);
        variable v_d  : signed(12 downto 0);
        variable v_x  : unsigned(12 downto 0);
        variable v_fl : std_logic_vector(5 downto 0);
        variable v_ey : std_logic;
        variable first : boolean;
    begin
        if rising_edge(clk) then
            -- 2: folds; the square / eye-edge ROM address
            p2_u <= f_fold(b_ph); p2_ut <= f_fold(b_pt); p2_sd <= not b_pt(19);
            r_sq_a <= f_fold(b_ph);
            -- 3: luma box (4 reads of the half-rate line buffer); the trapezoid / window / V / dome-end tests
            p3_u <= p2_u; p3_ut <= p2_ut;
            if p2_sd = '1' then p3_hwj <= resize(l_hw, 11) + 39; else p3_hwj <= resize(l_hw, 11) + 2; end if;
            p2_l1 <= unsigned(s_lb_q); p2_l2 <= p2_l1; p2_l3 <= p2_l2;
            v_l4 := (resize(unsigned(s_lb_q), 10) + resize(p2_l1, 10)) + (resize(p2_l2, 10) + resize(p2_l3, 10));
            if s_src = '1' then p3_lm <= signed(resize(v_l4(9 downto 2), 9)) - 111;
            else p3_lm <= signed(resize(v_l4(9 downto 2), 9)); end if;
            if p2_ut < l_hw then p3_trap <= '1'; else p3_trap <= '0'; end if;
            if l_lowl = '1' and signed(resize(p2_u, 11)) > l_lw then p3_low <= '1'; else p3_low <= '0'; end if;
            if l_veel = '1' and (512 - resize(p2_u, 10)) < resize(l_vth, 10) then p3_vee <= '1'; else p3_vee <= '0'; end if;
            if l_drl = '1' and signed(resize(p2_u, 11)) < l_ur then p3_ur <= '1'; else p3_ur <= '0'; end if;
            -- 4: square and eye edges (fabric); etch product; pillar / eye-inner tests; glow distance
            p4_sq <= unsigned(r_sq_q); p4_ey <= r_ey_q; p_u(4) <= p3_u;
            p4_et <= resize(p3_lm * signed('0' & c_eg), 17);
            if resize(p3_ut, 11) < p3_hwj then p4_pg <= '1'; else p4_pg <= '0'; end if;
            if resize(p3_ut, 11) > p3_hwj then p4_ein <= '1'; else p4_ein <= '0'; end if;
            p4_dg <= signed(resize(p3_ut, 12)) - signed(resize(p3_hwj, 12));
            p_fl(4) <= "00" & p3_ur & p3_vee & p3_low & p3_trap;
            p_top(4) <= l_top;
            -- 5: curvature partial products; etch scale; stripe / Fr operands; glow; the eye test
            if c_m2(0) = '1' then v_q := resize(p4_sq, 22); else v_q := (others => '0'); end if;
            if c_m2(1) = '1' then v_q := v_q + shift_left(resize(p4_sq, 22), 1); end if;
            p5_qa <= v_q(19 downto 0);
            if c_m2(2) = '1' then v_q := shift_left(resize(p4_sq, 22), 2); else v_q := (others => '0'); end if;
            if c_m2(3) = '1' then v_q := v_q + shift_left(resize(p4_sq, 22), 3); end if;
            p5_qb <= v_q(19 downto 0);
            p5_s16 <= shift_left(resize(p4_sq, 20), 4);
            if s_src = '1' then v_e := shift_right(p4_et, 3); else v_e := shift_right(p4_et, 8); end if;
            if v_e > 1023 then p5_e <= to_signed(1023, 11); elsif v_e < -1023 then p5_e <= to_signed(-1023, 11);
            else p5_e <= resize(v_e, 11); end if;
            p5_spl <= resize(p_u(4) * c_sfq(4 downto 0), 14);
            p5_sph <= resize(p_u(4) * c_sfq(8 downto 5), 13);
            p5_ur <= resize(shift_right(p_u(4), 2) + shift_right(p_u(4), 5) - shift_right(p_u(4), 8), 9);
            if p4_dg(11) = '1' then p5_ge <= l_gl;
            elsif resize(shift_right(unsigned(p4_dg), 1), 12) >= resize(l_gl, 12) then p5_ge <= (others => '0');
            else p5_ge <= l_gl - resize(shift_right(unsigned(p4_dg), 1), 8); end if;
            -- eye: yv - 410 > t1 and 690 - yv > t2
            if p4_ey(15) = '1' and p4_ein = '1' and l_yv > 410 + resize(unsigned(p4_ey(14 downto 8)), 11)
               and resize(l_yv, 12) + resize(unsigned(p4_ey(7 downto 0)), 12) < 690 then v_ey := '1'; else v_ey := '0'; end if;
            p_fl(5) <= p_fl(4)(3) & p_fl(4)(2) & p_fl(4)(1) & v_ey & p4_pg & p_fl(4)(0);   -- ur vee low eye pg trap
            p_top(5) <= p_top(4); p_u(5) <= p_u(4);
            -- 6: curvature sum; stripe phase; etched distances; bar phase offset
            p6_qs <= resize(p5_qa, 21) + resize(p5_qb, 21); p6_s16 <= p5_s16;
            p6_sp <= resize(p5_spl + shift_left(resize(p5_sph, 14), 5), 12);
            p6_dq <= signed(resize(l_dy, 13)) + resize(p5_e, 13);
            p6_fr <= signed(resize(l_dy, 13)) - signed(resize(p5_ur, 13)) + resize(p5_e, 13);
            p_x0(6) <= c_bph + unsigned(resize(shift_left(resize(p5_e, 16), 5), 12));
            p6_e <= p5_e;
            p_ge(6) <= p5_ge;
            p_fl(6) <= p_fl(5); p_top(6) <= p_top(5); p_u(6) <= p_u(5);
            -- 7: curvature total; stripe phase; PR address
            p7_qm <= resize(p6_qs, 22) + resize(p6_s16, 22);
            p7_sph <= p6_sp + l_sl + unsigned(resize(shift_left(resize(p6_e, 16), 4), 12));
            v_f := resize(p6_fr, 14) + 256;
            if v_f < 0 then v_x := (others => '0');
            elsif v_f > 1023 then v_x := to_unsigned(511, 13);
            else v_x := resize(unsigned(v_f(9 downto 1)), 13); end if;
            if p_top(6) = '1' then r_pr_a <= resize(v_x(8 downto 0), 10); else r_pr_a <= resize(v_x(8 downto 0), 10) + 512; end if;
            p7_dq <= p6_dq; p_x0(7) <= p_x0(6);
            p_ge(7) <= p_ge(6);
            p_fl(7) <= p_fl(6); p_top(7) <= p_top(6); p_u(7) <= p_u(6);
            -- 8: curvature shift and clamp; stripe triangle
            case c_s2 is
                when "110"  => v_q := shift_right(p7_qm, 15);
                when "111"  => v_q := shift_right(p7_qm, 14);
                when "000"  => v_q := shift_right(p7_qm, 13);
                when "001"  => v_q := shift_right(p7_qm, 12);
                when others => v_q := shift_right(p7_qm, 11);
            end case;
            if v_q > 1023 then p8_q <= (others => '1'); else p8_q <= v_q(9 downto 0); end if;
            if p7_sph >= 2048 then p8_tri <= resize(shift_right(p7_sph - 2048, 3), 8);
            elsif p7_sph < 8 then p8_tri <= (others => '1');                                      -- |0 - 2048| / 8 = 256: saturate
            else p8_tri <= resize(shift_right(2048 - resize(p7_sph, 13), 3), 8); end if;
            p8_dq <= p7_dq; p_x0(8) <= p_x0(7);
            p_ge(8) <= p_ge(7);
            p_fl(8) <= p_fl(7); p_top(8) <= p_top(7); p_u(8) <= p_u(7);
            -- 9: F; q split for q8 and the arch term; stripe level; PR data
            p9_F <= p8_dq + signed(resize(p8_q, 13));
            p9_q <= p8_q;
            p9_qa <= p8_q - shift_right(p8_q, 3);
            p9_qb <= resize(shift_right(p8_q, 4), 7) + resize(shift_right(p8_q, 6), 7);
            p9_a29 <= shift_left(resize(p8_q, 15), 5) - shift_left(resize(p8_q, 15), 1);
            v_d := signed(resize(shift_right(shift_left(resize(p8_tri, 11), 2) + resize(p8_tri, 11), 2), 13)) - 32;
            if v_d < 0 then p9_sv <= (others => '0'); elsif v_d > 255 then p9_sv <= (others => '1');
            else p9_sv <= unsigned(v_d(7 downto 0)); end if;
            p9_pr <= r_pr_q;
            p9_dq <= p8_dq; p_x0(9) <= p_x0(8);
            p_ge(9) <= p_ge(8);
            p_fl(9) <= p_fl(8); p_top(9) <= p_top(8); p_u(9) <= p_u(8);
            -- 10: q8; PB address; arc lower test; arch term; bar taper / lattice operands
            p10_q8 <= p9_qa - resize(p9_qb, 10);
            p10_F <= p9_F;
            if p9_F > 573 then p10_a1 <= '1'; else p10_a1 <= '0'; end if;
            if p_top(9) = '1' then v_f := resize(p9_F, 14) + 1024; else v_f := 1024 - resize(p9_F, 14); end if;
            if v_f < 0 then r_pb_a <= (others => '0');
            elsif v_f > 2047 then r_pb_a <= (others => '1');
            else r_pb_a <= unsigned(v_f(10 downto 1)); end if;
            if s_arc = '1' then p10_aq <= resize(p9_a29 - resize(p9_q, 15), 12); else p10_aq <= (others => '0'); end if;
            p10_dq <= p9_dq; p_x0(10) <= p_x0(9);
            p10_t <= 511 - p_u(9);
            if p_u(9) > 292 then p10_au <= p_u(9) - 292; p10_ao <= '1';
            else p10_au <= resize(292 - resize(p_u(9), 10), 9); p10_ao <= '0'; end if;
            p_sv(10) <= p9_sv; p_pr(10) <= unsigned(p9_pr);
            p_ge(10) <= p_ge(9);
            p_fl(10) <= p_fl(9);
            -- 11: F2; dome coordinate; bar phase (mod 4096); taper / lattice products
            p11_F2 <= p10_dq + signed(resize(p10_q8, 13));
            p11_yd <= signed(resize(l_yv, 12)) - signed(resize(p10_q8, 12));
            p11_t12 <= l_s(11 downto 0) + p_x0(10) + p10_aq;
            p11_a1 <= p10_a1;
            -- taper 211 / 512 = 1/2 - 1/16 - 1/32; lattice slopes 5.25 (outer) and 3.06 (inner) per u
            p11_bt <= resize(shift_right(p10_t, 1), 17) - resize(shift_right(p10_t, 4), 17) - resize(shift_right(p10_t, 5), 17);
            if p10_ao = '1' then
                p11_hp <= shift_left(resize(p10_au, 16), 2) + resize(p10_au, 16) + resize(shift_right(p10_au, 2), 16);
            else
                p11_hp <= shift_left(resize(p10_au, 16), 1) + resize(p10_au, 16) + resize(shift_right(p10_au, 4), 16);
            end if;
            p_sv(11) <= p_sv(10); p_pr(11) <= p_pr(10);
            p_ge(11) <= p_ge(10);
            p_fl(11) <= p_fl(10);
            -- 12: arc / inside tests; PO address; bar phase and the bar-centre level; dome test; PB data; widths
            if p11_F2 < 593 then p12_ins <= '1'; else p12_ins <= '0'; end if;
            if p11_F2 < 593 and p11_a1 = '1' then p12_arc <= '1'; else p12_arc <= '0'; end if;
            v_f := resize(p11_F2, 14) - 512;
            if v_f < 0 then v_x := (others => '0');
            elsif v_f > 1023 then v_x := to_unsigned(255, 13);
            else v_x := resize(unsigned(v_f(9 downto 2)), 13); end if;
            if l_top = '1' then r_po_a <= resize(v_x(7 downto 0), 9); else r_po_a <= resize(v_x(7 downto 0), 9) + 256; end if;
            p12_phb <= p11_t12;
            p12_sc <= signed(resize(l_s, 20)) - signed(resize(p11_t12, 20)) + 2048;
            if p11_yd > signed(resize(c_dt, 12)) and p11_yd < signed(resize(c_db, 12)) then p12_dome <= '1'; else p12_dome <= '0'; end if;
            p12_pb <= r_pb_q;
            p12_bh <= signed(resize(c_bh0, 11)) - signed(resize(p11_bt, 11));
            v_x := 704 + resize(p11_hp, 13);
            if v_x > 1640 then v_x := to_unsigned(1640, 13); end if;
            p12_hl <= resize(v_x - (704 - resize(c_bh0, 13)), 12);
            p_sv(12) <= p_sv(11); p_pr(12) <= p_pr(11);
            p_ge(12) <= p_ge(11);
            p_fl(12) <= p_fl(11);
            -- 13: bar distance; band and pillar-zone tests
            if p12_phb >= 2048 then p13_d <= resize(p12_phb - 2048, 11); else p13_d <= resize(2048 - resize(p12_phb, 13), 11); end if;
            if p12_sc > 2458 and p12_sc < signed(resize(c_smax, 20)) then p13_band <= '1'; else p13_band <= '0'; end if;
            if p12_sc < signed(resize(c_s720, 20)) then p13_pz <= '1'; else p13_pz <= '0'; end if;
            p13_bh <= p12_bh; p13_hl <= p12_hl;
            p_ins(13) <= p12_ins; p_arc(13) <= p12_arc; p_dome(13) <= p12_dome; p_pb(13) <= unsigned(p12_pb);
            p_sv(13) <= p_sv(12); p_pr(13) <= p_pr(12);
            p_ge(13) <= p_ge(12);
            p_fl(13) <= p_fl(12);
            -- 14: bar and lattice keys; PO data
            if signed(resize(p13_d, 12)) < resize(p13_bh, 12) and p13_band = '1' and not (p_fl(13)(1) = '1' and p13_pz = '1') then
                p14_bar <= '1';
            else
                p14_bar <= '0';
            end if;
            if resize(p13_d, 12) < p13_hl and p13_band = '1' then p14_lat <= '1'; else p14_lat <= '0'; end if;
            p14_po <= r_po_q;
            p_ins(14) <= p_ins(13); p_arc(14) <= p_arc(13); p_dome(14) <= p_dome(13); p_pb(14) <= p_pb(13);
            p_sv(14) <= p_sv(13); p_pr(14) <= p_pr(13);
            p_ge(14) <= p_ge(13);
            p_fl(14) <= p_fl(13);
            -- 15: composite
            v_fl := p_fl(14);
            if p_arc(14) = '1' then v_r := (others => '0');
            elsif p_ins(14) = '1' then v_r := p_pr(14);
            else v_r := unsigned(p14_po); end if;
            if p_arc(14) = '1' then v_g := to_unsigned(150, 8); else v_g := (others => '0'); end if;
            v_b := p_pb(14);
            if v_fl(3) = '1' then v_r := (others => '0'); v_g := to_unsigned(90, 8); v_b := p_sv(14); end if;       -- lower windows
            if p14_lat = '1' then v_g := to_unsigned(125, 8); v_b := to_unsigned(160, 8); end if;                 -- lattice
            if v_fl(2) = '1' then v_r := (others => '0'); v_g := p_ge(14); v_b := p_sv(14); end if;               -- eyes
            if v_fl(0) = '1' then v_g := to_unsigned(186, 8); end if;                                              -- trapezoid
            if p_dome(14) = '1' then
                v_g := to_unsigned(120, 8);
                if v_fl(5) = '1' then v_r := to_unsigned(238, 8); v_b := to_unsigned(200, 8); end if;              -- dome band
            end if;
            if v_fl(4) = '1' then v_r := to_unsigned(205, 8); end if;                                              -- red V
            if p14_bar = '1' then v_r := to_unsigned(240, 8); end if;                                              -- bars
            p15_r <= v_r; p15_g <= v_g; p15_b <= v_b;
            -- 16: Colours (rotate: R <- B, G <- R, B <- G) and Polarity; matrix ROM addresses
            if s_rot = '1' then v_r := p15_b; v_g := p15_r; v_b := p15_g;
            else v_r := p15_r; v_g := p15_g; v_b := p15_b; end if;
            if s_neg = '1' then v_r := not v_r; v_g := not v_g; v_b := not v_b; end if;
            r_mr_a <= v_r; r_mg_a <= v_g; r_mb_a <= v_b;
            -- 17: matrix ROM data (EBR); 18: fabric
            p20_mr <= r_mr_q; p20_mg <= r_mg_q; p20_mb <= r_mb_q;
            -- 19: sums
            p21_y <= to_unsigned(64, 11) + resize(unsigned(p20_mr(31 downto 22)), 11) + resize(unsigned(p20_mg(31 downto 22)), 11)
                     + resize(unsigned(p20_mb(31 downto 22)), 11);
            p21_u <= 512 + resize(signed(p20_mr(21 downto 11)), 13) + resize(signed(p20_mg(21 downto 11)), 13)
                     + resize(signed(p20_mb(21 downto 11)), 13);
            p21_v <= 512 + resize(signed(p20_mr(10 downto 0)), 13) + resize(signed(p20_mg(10 downto 0)), 13)
                     + resize(signed(p20_mb(10 downto 0)), 13);
            -- 20: luma pre-scaled 16/13 (capped at 940); chroma clamped
            v_y := p21_y - 64;
            v_y := p21_y + shift_right(v_y, 2) - shift_right(v_y, 6);
            if v_y > 940 then p22_y <= to_unsigned(940, 10); else p22_y <= v_y(9 downto 0); end if;
            v_c := p21_u;
            if v_c < 0 then p22_u <= (others => '0'); elsif v_c > 1023 then p22_u <= (others => '1');
            else p22_u <= unsigned(v_c(9 downto 0)); end if;
            v_c := p21_v;
            if v_c < 0 then p22_v <= (others => '0'); elsif v_c > 1023 then p22_v <= (others => '1');
            else p22_v <= unsigned(v_c(9 downto 0)); end if;
            -- 21: filter input, black outside the generated avid
            if s_gav_sr(18) = '0' then
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            else
                s_y23 <= p22_y; s_u23 <= p22_u; s_v23 <= p22_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(18);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma through one [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the 8-back tap from a block-RAM ring), luma [1 2 1]/4 on
    -- taps aligned with the chroma centre (the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate.  The syncs ride a block-RAM ring of C_LAT.
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rc <= rc + 1;
            -- chroma ring: written at rc + 6, read at rc, RAM register, fabric register = the input of 8
            -- clocks before (never the write address)
            cr(to_integer(rc + 6)) <= std_logic_vector(s_u23) & std_logic_vector(s_v23);
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            -- sync ring: written at rc + C_LAT - 2, RAM register, fabric register = C_LAT clocks
            sr(to_integer(rc + to_unsigned(C_LAT - 2, 8))) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
        end if;
    end process p_ring;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_l  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 24: chroma running 8-sums (snap back to neutral once 8 blank samples are in, so a glitched
            -- sample can't offset the colours for good); luma taps
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u23 - unsigned(cr_f(19 downto 10));
                s_bav <= s_bav + s_v23 - unsigned(cr_f(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_ly(0) <= s_y23;
            for i in 1 to 5 loop s_ly(i) <= s_ly(i - 1); end loop;
            -- 25: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 (both centred 4 pixels back)
            v_sc := resize(s_bau, 14) + s_baud; s_u25 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v25 <= v_sc(13 downto 4);
            v_l := resize(s_ly(3), 12) + shift_left(resize(s_ly(4), 12), 1) + resize(s_ly(5), 12);
            s_y25 <= v_l(11 downto 2);
            -- 26: luma 13/16
            if s_y25 > 64 then v_dl := s_y25 - 64; else v_dl := (others => '0'); end if;
            s_y26 <= s_y25 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u26 <= s_u25; s_v26 <= s_v25;
            -- 25: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y29 <= s_y26; s_u29 <= s_u26; s_v29 <= s_v26;
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y29);
    data_out.u       <= std_logic_vector(s_u29);
    data_out.v       <= std_logic_vector(s_v29);
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture portal;
