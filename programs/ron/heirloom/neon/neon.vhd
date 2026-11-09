-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Neon v0.1.0 -- Heirloom (video phase), from the clip WIN_20230919_00_32_29 at 3.0 s.
--
-- The still: three neon bands sweep down the frame on a teal ground, bent into S-curves that lean further the
-- further right they are.  Each band is a white tube in a red-brown glow, a red gap and a lavender tube, then a
-- violet interior, a red-magenta rim and a second white tube in its glow.  Inside, magenta stripes with white
-- edges run in ")(" pairs whose cusps sit on fixed rows (255 / 775), with lens-shaped eyes where the cusps meet,
-- and dark cores near the middle rows.  In the clip the S-curve rolls down the frame (316 lines/s, one cycle per
-- 7.1 s) while the interior pattern stays put: the bands are moving windows onto a frame-locked pattern.
--
-- The patch, as fitted (neon.py has every constant):
--   bands    U = (x + 184) * F0 * (1 + A cos(2 pi (y - yp) / 2240))     an H-synced oscillator FM'd by a slow
--            vertical sine (F0 = 1/647.7 band per px, A = 0.146), coloured by the band profile ROM
--   interior P = |x - 992| * 1024/770 folded to the nearest axis (ax); phi = (y - 515) / 2080
--            S = min(|ax - 127 - D| - W, |ax + 827 sqrt(dphi) - 105| - 35),  D = 44 (1 - |cos 4 pi phi|),
--            W = 42 |cos 2 pi phi|; coloured by the interior palette ROM, dark cores near the mid rows
--
--   K1  Bands    the bands' count, x1/3 .. x3 (512 = the clip's 2.6 across)
--   K2  Gap      the interior's width between the tube pairs, x0.25 .. x2
--   K3  Drift    bipolar: the S-curve rolls down (+) or up (-); 700 = the clip's roll
--   K4  Waves    the vertical sine's frequency: one gentle arc .. a tight serpentine
--   K5  Facets   the interior pattern's scale (stripes, cusps and eyes)
--   K6  Etch     how far the picture (or the Osc) bends the tubes and the interior contours
--   S7  Wave     Sine (the clip) / Square: the bands break into stepped segments
--   S8  Mirror   the bands mirrored about the centre
--   S9  Glow     Halo (the clip) / Bare: no red glow around the tubes
--   S10 Palette  Neon (the clip) / Argon: cyan tubes and interiors on a forest-green ground
--   S11 Source   Video / Osc
--   P12 SWAY     the vertical sine's depth: nearly straight upright tubes .. hairpins (395 = the clip)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: the FM'd band frequency, the band phase start, the interior's line terms),
-- two beam DDAs (band phase, interior phase), a 25-stage pixel path.  Virtual 1080-line coordinates (SD shows
-- the centre 4:3).  Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes,
-- luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.neon_pkg.all;
use work.all;

architecture neon of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_UC16 : integer := 47349;         -- interior centre of the band profile, 0.7225 band (Q16)
    constant K_TH0  : integer := 5294872;       -- FM phase at power-up: the still (-1533/2240 cycle)
    constant C_X4   : integer := 37632;         -- (992 + 184) virtual px = 2352 h, Q4
    constant C_A0   : integer := 127;           -- interior arms: centre, q units
    constant C_E0   : integer := 105;           -- eye ring centre
    constant C_E1   : integer := 35;            -- eye ring half-width
    constant K_DR   : integer := 135;           -- dark-core radius
    constant C_PJ2  : integer := 2670177;       -- zipper 2 phase at line 0 (1 rad)
    constant C_TEAL : std_logic_vector(31 downto 0) := C_NE_PROF(0);

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

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
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
    signal r_cs_a : unsigned(9 downto 0) := (others => '0');
    signal r_cs_q, r_cs_qq : unsigned(15 downto 0) := (others => '0');
    signal r_sq_a : unsigned(7 downto 0) := (others => '0');
    signal r_sq_q : unsigned(15 downto 0) := (others => '0');
    signal r_pf_a, r_ip_a : unsigned(8 downto 0) := (others => '0');
    signal r_pf_q, r_ip_q : std_logic_vector(31 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), raw drift, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(395 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k3 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_sq, s_mir, s_bare, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal z_sh1, z_sh4, z_sh5 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt4, z_mt5 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_e1, z_e4, z_e5 : signed(10 downto 0) := (others => '0');
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal c_f0   : unsigned(16 downto 0) := to_unsigned(25903, 17);   -- band frequency, Q24 band / virtual px
    signal c_ka   : unsigned(16 downto 0) := to_unsigned(7490, 17);    -- FM sine, Q24 cycles / virtual line
    signal c_kas  : unsigned(19 downto 0) := to_unsigned(7490, 20);    -- ... per line
    signal c_kz   : unsigned(18 downto 0) := to_unsigned(87156, 19);   -- interior, Q16 q / virtual px
    signal c_kzy  : unsigned(15 downto 0) := to_unsigned(8066, 16);    -- interior phi, Q24 cycles / virtual line
    signal c_kzys : unsigned(17 downto 0) := to_unsigned(8066, 18);    -- ... per line
    signal c_phi0 : unsigned(23 downto 0) := (others => '0');          -- phi of line 0
    signal c_ps0  : unsigned(28 downto 0) := (others => '0');          -- interior beam start (signed)
    signal c_stpp : unsigned(19 downto 0) := (others => '0');          -- interior beam step per pixel
    signal c_kp2  : unsigned(19 downto 0) := (others => '0');
    signal c_amp  : unsigned(11 downto 0) := to_unsigned(597, 12);     -- FM depth, Q12
    signal c_d2   : unsigned(17 downto 0) := (others => '0');
    signal c_dr   : unsigned(18 downto 0) := (others => '0');          -- drift per field, Q24 cycles
    signal c_th0  : unsigned(23 downto 0) := to_unsigned(K_TH0, 24);   -- FM phase of line 0
    signal c_g6   : unsigned(9 downto 0) := to_unsigned(156, 10);      -- etch gain
    signal c_kj1  : unsigned(23 downto 0) := (others => '0');          -- zipper phases per line
    signal c_kj2  : unsigned(23 downto 0) := (others => '0');
    signal c_gap  : signed(23 downto 0) := (others => '0');            -- interior width offset (Q24 band)
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);      -- beam start x (h)
    signal c_soff : signed(10 downto 0) := to_signed(96, 11);          -- interior palette offset (- Osc mean)
    signal c_uco  : unsigned(15 downto 0) := to_unsigned(C_UC16, 16);  -- profile centre (+ Osc mean), Q16
    signal c_ty, c_tu, c_tv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- teal (off-avid box input)

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
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

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_tha, e_phi, e_pj1, e_pj2 : unsigned(23 downto 0) := (others => '0');
    signal e_ng   : std_logic_vector(2 downto 0) := (others => '0');
    signal e_w    : unsigned(14 downto 0) := (others => '0');
    signal e_wn   : std_logic := '0';
    signal e_j4   : signed(15 downto 0) := (others => '0');
    signal e_x4   : unsigned(15 downto 0) := to_unsigned(C_X4, 16);
    signal e_d0   : unsigned(14 downto 0) := (others => '0');
    signal e_d44  : unsigned(20 downto 0) := (others => '0');
    signal e_dph  : unsigned(21 downto 0) := (others => '0');
    signal e_dd   : unsigned(11 downto 0) := (others => '0');
    signal e_dd14 : unsigned(15 downto 0) := (others => '0');
    signal e_aw   : unsigned(14 downto 0) := (others => '0');
    signal e_f    : unsigned(17 downto 0) := (others => '0');
    signal m_a    : unsigned(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    signal n_uc   : unsigned(23 downto 0) := (others => '0');
    signal n_us0  : unsigned(28 downto 0) := (others => '0');
    signal n_stp  : unsigned(18 downto 0) := (others => '0');
    signal n_w    : unsigned(5 downto 0) := (others => '0');
    signal n_a0d  : unsigned(7 downto 0) := to_unsigned(C_A0, 8);
    signal n_ey   : unsigned(8 downto 0) := (others => '1');
    signal n_dv   : unsigned(4 downto 0) := (others => '0');

    -- beam and line registers
    signal b_us, b_ps : unsigned(28 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_uc  : unsigned(23 downto 0) := (others => '0');
    signal l_stp : unsigned(18 downto 0) := (others => '0');
    signal l_w   : unsigned(5 downto 0) := (others => '0');
    signal l_a0d : unsigned(7 downto 0) := to_unsigned(C_A0, 8);
    signal l_ey  : unsigned(8 downto 0) := (others => '1');
    signal l_dv  : unsigned(4 downto 0) := (others => '0');

    -- pixel path
    signal p2_ua  : unsigned(23 downto 0) := (others => '0');
    signal p2_pa  : unsigned(10 downto 0) := (others => '0');
    signal p3_u16 : unsigned(15 downto 0) := (others => '0');
    signal p3_r   : unsigned(9 downto 0) := (others => '0');
    signal p3_k0  : std_logic := '0';
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(13 downto 0) := (others => '0');
    signal p4_v   : unsigned(15 downto 0) := (others => '0');
    signal p4_ax  : unsigned(9 downto 0) := (others => '0');
    signal p4_out, p4_tr : std_logic := '0';
    signal p5_pe  : unsigned(18 downto 0) := (others => '0');
    signal p5_v   : unsigned(15 downto 0) := (others => '0');
    signal p5_ax  : unsigned(9 downto 0) := (others => '0');
    signal p5_m   : unsigned(9 downto 0) := (others => '0');
    signal p5_ee  : unsigned(10 downto 0) := (others => '0');
    signal p5_out, p5_tr : std_logic := '0';
    signal p6_v   : unsigned(15 downto 0) := (others => '0');
    signal p6_sarm : signed(11 downto 0) := (others => '0');
    signal p6_seye : unsigned(10 downto 0) := (others => '0');
    signal p6_tx  : unsigned(9 downto 0) := (others => '0');
    signal p6_pen : unsigned(9 downto 0) := (others => '0');
    signal p6_es  : unsigned(10 downto 0) := (others => '0');
    signal p7_vp  : signed(16 downto 0) := (others => '0');
    signal p7_sarm, p7_seye : signed(11 downto 0) := (others => '0');
    signal p7_t140 : unsigned(17 downto 0) := (others => '0');
    signal p7_es  : unsigned(10 downto 0) := (others => '0');
    signal p8_pa, p9_pa, p10_pa : unsigned(8 downto 0) := (others => '0');
    signal p8_s   : signed(12 downto 0) := (others => '0');
    signal p8_dkx : unsigned(4 downto 0) := (others => '0');
    signal p9_dk, p10_dk, p11_dk : unsigned(3 downto 0) := (others => '0');
    signal p11_ip, p13_pf : std_logic_vector(31 downto 0) := (others => '0');
    signal p12_dy, p12_du, p12_dv : signed(15 downto 0) := (others => '0');
    signal p12_py, p12_pu, p12_pv : unsigned(9 downto 0) := (others => '0');
    signal p13_iy, p13_iu, p13_iv, p14_iy, p14_iu, p14_iv : unsigned(9 downto 0) := (others => '0');
    signal p14_ry, p14_ru, p14_rv : unsigned(9 downto 0) := (others => '0');
    signal p14_al : unsigned(1 downto 0) := (others => '0');
    signal p15_y, p15_u, p15_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the box's last 8 inputs
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
            r_ex_q <= C_NE_EXP2(to_integer(r_ex_a));
            r_cs_q <= C_NE_COSQ(to_integer(r_cs_a));
            r_sq_q <= C_NE_SQRT(to_integer(r_sq_a));
            r_pf_q <= C_NE_PROF(to_integer(r_pf_a));
            r_ip_q <= C_NE_IPAL(to_integer(r_ip_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_t  : unsigned(31 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_u  : unsigned(9 downto 0);
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
            -- drift: bipolar, +-8 deadband
            s_k3 <= unsigned(registers_in(2));
            v_s := signed(resize(s_k3, 11)) - 512;
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
                s_sq   <= registers_in(6)(0);
                s_mir  <= registers_in(6)(1);
                s_bare <= registers_in(6)(2);
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
                            when 2      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
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
                            when 2      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
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
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 486 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  =>                                                                        -- |k1 - 512| * 1.6
                        if s_g1(12 downto 3) >= 512 then z_fa <= resize(s_g1(12 downto 3) - 512, 36);
                        else z_fa <= resize(512 - s_g1(12 downto 3), 36); end if;
                        z_fb <= to_unsigned(410, 16);
                    when 1  =>                                                                        -- k4: -1.7 / +3.4 oct
                        if s_g4(12 downto 3) >= 512 then z_fa <= resize(s_g4(12 downto 3) - 512, 36); z_fb <= to_unsigned(870, 16);
                        else z_fa <= resize(512 - s_g4(12 downto 3), 36); z_fb <= to_unsigned(435, 16); end if;
                    when 2  =>                                                                        -- k5: -1.3 / +1.7 oct
                        if s_g5(12 downto 3) >= 512 then z_fa <= resize(s_g5(12 downto 3) - 512, 36); z_fb <= to_unsigned(435, 16);
                        else z_fa <= resize(512 - s_g5(12 downto 3), 36); z_fb <= to_unsigned(333, 16); end if;
                    when 3  => z_fa <= to_unsigned(25903, 36);     z_fb <= z_mt1;                     -- F0
                    when 4  => z_fa <= to_unsigned(7490, 36);      z_fb <= z_mt4;                     -- FM sine frequency
                    when 5  => z_fa <= to_unsigned(87156, 36);     z_fb <= z_mt5;                     -- interior x
                    when 6  => z_fa <= to_unsigned(8066, 36);      z_fb <= z_mt5;                     -- interior y
                    when 7  => z_fa <= resize(c_ka, 36);           z_fb <= resize(s_ys, 16);          -- per line
                    when 8  => z_fa <= resize(c_kzy, 36);          z_fb <= resize(s_ys, 16);
                    when 9  => z_fa <= resize(c_kzy, 36);          z_fb <= to_unsigned(515, 16);      -- phi of line 0
                    when 10 => z_fa <= resize(c_kz, 36);           z_fb <= resize(to_unsigned(1984, 12) - c_xb0, 16);  -- P start
                    when 11 => z_fa <= resize(s_gp(12 downto 3), 36); z_fb <= resize(s_gp(12 downto 3), 16);  -- sway^2
                    when 12 => z_fa <= resize(c_kp2, 36);          z_fb <= to_unsigned(2902, 16);
                    when 13 => z_fa <= resize(s_vm, 36);           z_fb <= resize(s_vm, 16);          -- drift^2
                    when 14 => z_fa <= resize(c_d2, 36);           z_fb <= to_unsigned(311, 16);
                    when 15 => z_fa <= resize(s_g6(12 downto 3), 36); z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 16 => z_fa <= to_unsigned(3647190, 36);   z_fb <= resize(s_ys, 16);          -- zipper 1
                    when 17 => z_fa <= to_unsigned(1729624, 36);   z_fb <= resize(s_ys, 16);          -- zipper 2
                    when 19 => z_fa <= resize(c_g6, 36);           z_fb <= to_unsigned(111, 16);      -- Osc mean offset
                    when others =>                                                                    -- gap
                        if s_g2(12 downto 3) >= 512 then z_fa <= resize(s_g2(12 downto 3) - 512, 36); z_fb <= to_unsigned(4022, 16);
                        else z_fa <= resize(512 - s_g2(12 downto 3), 36); z_fb <= to_unsigned(3010, 16); end if;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's drift step); beam origin; off-avid teal
                    when 20 =>
                        if s_vneg = '1' then c_th0 <= c_th0 + resize(c_dr, 24);
                        else c_th0 <= c_th0 - resize(c_dr, 24); end if;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                    when 22 =>
                        v_k := C_TEAL;
                        c_ty <= unsigned(v_k(31 downto 22));
                        if s_pal = '1' then
                            c_tu <= 1023 - unsigned(v_k(12 downto 4) & '0'); c_tv <= unsigned(v_k(21 downto 13) & '0');
                        else
                            c_tu <= unsigned(v_k(21 downto 13) & '0'); c_tv <= unsigned(v_k(12 downto 4) & '0');
                        end if;

                    -- exponents (Q8 octaves, signed): e = |k - 512| * slope >> 9
                    when 50  =>
                        if s_g1(12 downto 3) >= 512 then z_e1 <= signed(resize(f_acc(18 downto 9), 11));
                        else z_e1 <= -signed(resize(f_acc(18 downto 9), 11)); end if;
                    when 74  =>
                        if s_g4(12 downto 3) >= 512 then z_e4 <= signed(resize(f_acc(19 downto 9), 11));
                        else z_e4 <= -signed(resize(f_acc(19 downto 9), 11)); end if;
                    when 98  =>
                        if s_g5(12 downto 3) >= 512 then z_e5 <= signed(resize(f_acc(18 downto 9), 11));
                        else z_e5 <= -signed(resize(f_acc(18 downto 9), 11)); end if;
                    when 99  => r_ex_a <= unsigned(z_e1(7 downto 0)); z_sh1 <= z_e1(10 downto 8);
                    when 100 => r_ex_a <= unsigned(z_e4(7 downto 0)); z_sh4 <= z_e4(10 downto 8);
                    when 101 => r_ex_a <= unsigned(z_e5(7 downto 0)); z_sh5 <= z_e5(10 downto 8); z_mt1 <= r_ex_q;
                    when 102 => z_mt4 <= r_ex_q;
                    when 103 => z_mt5 <= r_ex_q;
                    when 122 =>                       -- F0 = 25903 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_f0 <= resize(f_acc(32 downto 17), 17);
                            when -1     => c_f0 <= f_acc(32 downto 16);
                            when 0      => c_f0 <= f_acc(31 downto 15);
                            when others => c_f0 <= f_acc(30 downto 14);
                        end case;
                    when 146 =>                       -- kA = 7490 mt >> (15 - sh), sh = -2 .. 3
                        case to_integer(z_sh4) is
                            when -2     => c_ka <= resize(f_acc(32 downto 17), 17);
                            when -1     => c_ka <= f_acc(32 downto 16);
                            when 0      => c_ka <= f_acc(31 downto 15);
                            when 1      => c_ka <= f_acc(30 downto 14);
                            when 2      => c_ka <= f_acc(29 downto 13);
                            when others => c_ka <= f_acc(28 downto 12);
                        end case;
                    when 170 =>                       -- kz = 87156 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh5) is
                            when -2     => c_kz <= resize(f_acc(34 downto 17), 19);
                            when -1     => c_kz <= resize(f_acc(34 downto 16), 19);
                            when 0      => c_kz <= f_acc(33 downto 15);
                            when others => c_kz <= f_acc(32 downto 14);
                        end case;
                    when 194 =>                       -- kzy = 8066 mt >> (15 - sh)
                        case to_integer(z_sh5) is
                            when -2     => c_kzy <= f_acc(32 downto 17);
                            when -1     => c_kzy <= f_acc(31 downto 16);
                            when 0      => c_kzy <= f_acc(30 downto 15);
                            when others => c_kzy <= f_acc(29 downto 14);
                        end case;
                    when 195 =>                       -- interior step per pixel = kz * xs / 2
                        case to_integer(s_xs) is
                            when 4      => c_stpp <= shift_left(resize(c_kz, 20), 1);
                            when 3      => c_stpp <= shift_right(resize(c_kz, 20) + shift_left(resize(c_kz, 20), 1), 1);
                            when others => c_stpp <= resize(c_kz, 20);
                        end case;
                    when 218 => c_kas  <= f_acc(27 downto 8);
                    when 242 => c_kzys <= f_acc(25 downto 8);
                    when 266 => c_phi0 <= (not f_acc(23 downto 0)) + 1;
                    when 290 =>
                        v_t := resize(f_acc(32 downto 1), 32);
                        v_t := (not v_t) + 1;
                        c_ps0 <= v_t(28 downto 0);
                    when 314 => c_kp2 <= f_acc(19 downto 0);
                    when 338 => c_amp <= to_unsigned(164, 12) + f_acc(31 downto 20);
                    when 362 => c_d2  <= f_acc(17 downto 0);
                    when 386 => c_dr  <= f_acc(26 downto 8);
                    when 410 => c_g6  <= f_acc(19 downto 10);
                    when 434 => c_kj1 <= f_acc(31 downto 8);
                    when 458 => c_kj2 <= f_acc(31 downto 8);
                    when 482 =>
                        if s_g2(12 downto 3) >= 512 then c_gap <= signed(f_acc(23 downto 0));
                        else c_gap <= -signed(f_acc(23 downto 0)); end if;
                    when 506 =>                       -- Osc: etch centred on the lattice's mean (111)
                        if s_src = '1' then
                            c_soff <= to_signed(96, 11) - signed(resize(f_acc(16 downto 8), 11));
                            c_uco  <= to_unsigned(C_UC16, 16) + resize(f_acc(16 downto 3), 16);
                        else
                            c_soff <= to_signed(96, 11);
                            c_uco  <= to_unsigned(C_UC16, 16);
                        end if;
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
    -- result at load + 17):
    --   w = cos(2 pi thA) (Square: its sign), aw = A w, f = F0 (1 + aw)
    --   J = the tube zipper (two cosines), Uc = (992 + 184 + J) f, Us0 = (x0 - 992) f, step = f xs / 2
    --   W = 42 |cos 2 pi phi|, D = 44 (1 - |cos 4 pi phi|), the eye term (sqrt ROM), the dark-core weight
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_a  : unsigned(11 downto 0);
        variable v_c  : unsigned(14 downto 0);
        variable v_cs : signed(15 downto 0);
        variable v_j  : signed(15 downto 0);
        variable v_t  : unsigned(31 downto 0);
        variable v_d  : unsigned(21 downto 0);
        variable v_ps : signed(23 downto 0);
        variable v_dv : signed(16 downto 0);
        variable v_x4 : signed(17 downto 0);
        variable v_w42 : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_tha <= c_th0; e_phi <= c_phi0;
                e_pj1 <= (others => '0'); e_pj2 <= to_unsigned(C_PJ2, 24);
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_tha <= e_tha + resize(c_kas, 24);
                e_phi <= e_phi + resize(c_kzys, 24);
                e_pj1 <= e_pj1 + c_kj1;
                e_pj2 <= e_pj2 + c_kj2;
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- quarter-wave cosine lookups: address at lc n, value (r_cs_qq, sign e_ng(2)) at lc n + 3
            r_cs_qq <= r_cs_q;
            e_ng <= e_ng(1 downto 0) & '0';
            case to_integer(e_lc) is
                when 1 | 2 | 3 | 4 | 5 =>
                    case to_integer(e_lc) is
                        when 1      => v_a := e_tha(23 downto 12);
                        when 2      => v_a := e_pj1(23 downto 12);
                        when 3      => v_a := e_pj2(23 downto 12);
                        when 4      => v_a := e_phi(23 downto 12);
                        when others => v_a := e_phi(22 downto 11);
                    end case;
                    if v_a(10) = '1' then r_cs_a <= not v_a(9 downto 0); else r_cs_a <= v_a(9 downto 0); end if;
                    e_ng(0) <= v_a(11) xor v_a(10);
                when others => null;
            end case;
            v_c := r_cs_qq(14 downto 0);
            if e_ng(2) = '1' then v_cs := -signed(resize(v_c, 16)); else v_cs := signed(resize(v_c, 16)); end if;

            case to_integer(e_lc) is
                when 4 =>                     -- w (Square: full scale, its sign)
                    if s_sq = '1' then e_w <= to_unsigned(32767, 15); else e_w <= v_c; end if;
                    e_wn <= e_ng(2);
                when 5 =>                     -- zipper 1; A |w|
                    e_j4 <= shift_right(v_cs, 9);
                    m_a <= resize(c_amp, 40); m_b <= '0' & e_w; m_acc <= (others => '0');
                when 6 =>                     -- zipper 2
                    e_j4 <= e_j4 + shift_right(v_cs, 10) + shift_right(v_cs, 12);
                when 7 =>                     -- W = 42 |cos 2 pi phi| >> 15; X4 = (992 + 184) h Q4 + zipper
                    v_w42 := shift_left(resize(v_c, 21), 5) + shift_left(resize(v_c, 21), 3) + shift_left(resize(v_c, 21), 1);
                    n_w <= v_w42(20 downto 15);
                    v_x4 := to_signed(C_X4, 18) + resize(e_j4, 18);
                    e_x4 <= unsigned(v_x4(15 downto 0));
                when 8 =>
                    e_d0 <= to_unsigned(32767, 15) - v_c;
                when 10 =>
                    e_d44 <= shift_left(resize(e_d0, 21), 5) + shift_left(resize(e_d0, 21), 3) + shift_left(resize(e_d0, 21), 2);
                    v_d := e_phi(21 downto 0);
                    if v_d(21) = '1' then e_dph <= v_d - to_unsigned(2097152, 22);
                    else e_dph <= to_unsigned(2097152, 22) - v_d; end if;
                    v_ps := signed(e_phi);
                    if v_ps < 0 then v_ps := -v_ps; end if;
                    e_dd <= unsigned(v_ps(23 downto 12));
                when 11 =>
                    n_a0d <= to_unsigned(C_A0, 8) + resize(e_d44(20 downto 15), 8);
                    if e_dph(21 downto 13) > 255 then r_sq_a <= (others => '1'); else r_sq_a <= e_dph(20 downto 13); end if;
                    e_dd14 <= shift_left(resize(e_dd, 16), 4) - shift_left(resize(e_dd, 16), 1);
                when 12 =>
                    v_dv := 22 - signed(resize(e_dd14(15 downto 8), 17));
                    if v_dv < 0 then n_dv <= (others => '0');
                    elsif v_dv > 16 then n_dv <= to_unsigned(16, 5);
                    else n_dv <= unsigned(v_dv(4 downto 0)); end if;
                when 14 =>
                    n_ey <= r_sq_q(8 downto 0);
                when 22 =>                    -- aw = A |w| >> 12
                    e_aw <= m_acc(26 downto 12);
                when 23 =>
                    m_a <= resize(c_f0, 40); m_b <= '0' & e_aw; m_acc <= (others => '0');      -- F0 aw
                when 40 =>                    -- f = F0 (1 +- aw)
                    if e_wn = '1' then e_f <= resize(c_f0, 18) - resize(m_acc(31 downto 15), 18);
                    else e_f <= resize(c_f0, 18) + resize(m_acc(31 downto 15), 18); end if;
                when 41 =>
                    m_a <= resize(e_f, 40); m_b <= e_x4; m_acc <= (others => '0');            -- Uc
                    case to_integer(s_xs) is
                        when 4      => n_stp <= shift_left(resize(e_f, 19), 1);
                        when 3      => n_stp <= shift_right(resize(e_f, 19) + shift_left(resize(e_f, 19), 1), 1);
                        when others => n_stp <= resize(e_f, 19);
                    end case;
                when 58 =>
                    n_uc <= m_acc(28 downto 5);
                when 59 =>
                    m_a <= resize(e_f, 40); m_b <= resize(to_unsigned(1984, 12) - c_xb0, 16); m_acc <= (others => '0');  -- Us0
                when 76 =>
                    v_t := m_acc(32 downto 1);
                    v_t := (not v_t) + 1;
                    n_us0 <= v_t(28 downto 0);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the band phase (signed, relative to the centre) and the interior
    -- phase (signed), both stepped per pixel; the line registers load at g_start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_us <= n_us0;
                b_ps <= c_ps0;
                b_px <= (others => '0');
                l_uc <= n_uc; l_stp <= n_stp; l_w <= n_w; l_a0d <= n_a0d; l_ey <= n_ey; l_dv <= n_dv;
            elsif g_avid = '1' then
                b_us <= b_us + resize(l_stp, 29);
                b_ps <= b_ps + resize(c_stpp, 29);
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : signed(10 downto 0);
        variable v_e   : signed(11 downto 0);
        variable v_s   : signed(13 downto 0);
        variable v_dk  : signed(5 downto 0);
        variable v_p   : unsigned(9 downto 0);
        variable v_al  : unsigned(2 downto 0);
        variable v_ad  : signed(10 downto 0);
        variable v_ae  : signed(11 downto 0);
        variable v_w   : signed(15 downto 0);
        variable v_t24 : unsigned(23 downto 0);
        variable v_m   : signed(11 downto 0);
        variable v_ry, v_ru, v_rv : unsigned(9 downto 0);
        variable v_h   : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: |band phase| (Mirror; ones' complement, 24 bits) and |interior phase| (bits 26..16)
            if s_mir = '1' and b_us(28) = '1' then p2_ua <= not b_us(23 downto 0); else p2_ua <= b_us(23 downto 0); end if;
            if b_ps(28) = '1' then p2_pa <= not b_ps(26 downto 16); else p2_pa <= b_ps(26 downto 16); end if;
            -- 3: band phase fraction (Q16); interior position in the half period; luma
            v_t24 := l_uc + p2_ua;
            p3_u16 <= v_t24(23 downto 8);
            p3_r   <= p2_pa(9 downto 0);
            p3_k0  <= p2_pa(10);
            p3_lm  <= unsigned(s_lb_q);
            -- 4: etch partials (luma nibbles x gain); v = u - centre; fold to the nearest axis
            p4_ea <= p3_lm(3 downto 0) * c_g6;
            p4_eb <= p3_lm(7 downto 4) * c_g6;
            p4_v  <= p3_u16 - c_uco;
            if p3_r(9) = '0' then
                p4_ax <= p3_r; p4_out <= '1';
            else
                p4_ax <= (not p3_r) + 1; p4_out <= '0';
            end if;
            p4_tr <= p3_k0 xor p3_r(9);
            -- 5: etch product; arm distance; eye radius
            if s_src = '1' then
                p5_pe <= shift_left(resize(p4_ea, 19) + shift_left(resize(p4_eb, 19), 4), 1);
            else
                p5_pe <= resize(p4_ea, 19) + shift_left(resize(p4_eb, 19), 4);
            end if;
            p5_v <= p4_v;
            v_d := signed(resize(p4_ax, 11)) - signed(resize(l_a0d, 11));
            v_ad := abs(v_d);
            p5_m <= unsigned(v_ad(9 downto 0));
            p5_ee <= resize(p4_ax, 11) + resize(l_ey, 11);
            p5_ax <= p4_ax; p5_out <= p4_out; p5_tr <= p4_tr;
            -- 6: etched band phase; arm and eye fields; dark-core distance; eye gating
            p6_v <= p5_v + resize(p5_pe(18 downto 4), 16);
            p6_sarm <= signed(resize(p5_m, 12)) - signed(resize(l_w, 12));
            v_e := signed(resize(p5_ee, 12)) - C_E0;
            v_ae := abs(v_e);
            p6_seye <= unsigned(v_ae(10 downto 0));
            v_d := signed(resize(p5_m, 11)) - K_DR;
            v_ad := abs(v_d);
            p6_tx <= unsigned(v_ad(9 downto 0));
            if p5_tr = '0' then
                p6_pen <= (others => '0');
            elsif p5_out = '0' then
                p6_pen <= to_unsigned(511, 10);
            elsif p5_ax < 18 then
                v_p := to_unsigned(18, 10) - p5_ax;
                p6_pen <= shift_left(v_p, 3);
            else
                p6_pen <= (others => '0');
            end if;
            p6_es <= resize(p5_pe(18 downto 9), 11);
            -- 7: interior width (Gap); eye with its gating; dark-core ramp
            if p6_v(15) = '1' then
                p7_vp <= resize(signed(p6_v), 17) + resize(c_gap(23 downto 8), 17);
            else
                p7_vp <= resize(signed(p6_v), 17) - resize(c_gap(23 downto 8), 17);
            end if;
            v_e := signed(resize(p6_seye, 12)) - C_E1 + signed(resize(p6_pen, 12));
            if v_e > 1023 then p7_seye <= to_signed(1023, 12); else p7_seye <= v_e; end if;
            p7_sarm <= p6_sarm;
            p7_t140 <= shift_left(resize(p6_tx, 18), 7) + shift_left(resize(p6_tx, 18), 3) + shift_left(resize(p6_tx, 18), 2);
            p7_es <= p6_es;
            -- 8: profile index (outside one band: teal at 0); S = min(arm, eye) + etch; dark ramp
            if p7_vp(16) = p7_vp(15) then
                p8_pa <= unsigned(p7_vp(15 downto 7)) + to_unsigned(256, 9);
            else
                p8_pa <= (others => '0');
            end if;
            if p7_sarm < p7_seye then v_m := p7_sarm; else v_m := p7_seye; end if;
            p8_s <= resize(v_m, 13) + signed(resize(p7_es, 13));
            v_dk := to_signed(21, 6) - signed(resize(p7_t140(17 downto 8), 6));
            if p7_t140(17 downto 8) > 21 then p8_dkx <= (others => '0');
            elsif v_dk > 16 then p8_dkx <= to_unsigned(16, 5);
            else p8_dkx <= unsigned(v_dk(4 downto 0)); end if;
            -- 9: palette address; dark weight
            v_s := resize(p8_s, 14) + resize(c_soff, 14);
            if v_s < 0 then r_ip_a <= (others => '0');
            elsif v_s > 511 then r_ip_a <= (others => '1');
            else r_ip_a <= unsigned(v_s(8 downto 0)); end if;
            p9_dk <= resize(shift_right(p8_dkx * l_dv, 5), 4);
            p9_pa <= p8_pa;
            -- 10: palette ROM data (EBR)
            p10_dk <= p9_dk; p10_pa <= p9_pa;
            -- 11: palette fabric copy; profile ROM address
            p11_ip <= r_ip_q; p11_dk <= p10_dk;
            r_pf_a <= p10_pa;
            -- 12: dark-core products (profile ROM data, EBR)
            p12_py <= unsigned(p11_ip(31 downto 22));
            p12_pu <= unsigned(p11_ip(21 downto 13) & '0');
            p12_pv <= unsigned(p11_ip(12 downto 4) & '0');
            p12_dy <= resize((signed(resize(unsigned(p11_ip(31 downto 22)), 11)) - C_NE_DKY) * signed(resize(p11_dk, 5)), 16);
            p12_du <= resize((signed(resize(unsigned(p11_ip(21 downto 13) & '0'), 11)) - C_NE_DKU) * signed(resize(p11_dk, 5)), 16);
            p12_dv <= resize((signed(resize(unsigned(p11_ip(12 downto 4) & '0'), 11)) - C_NE_DKV) * signed(resize(p11_dk, 5)), 16);
            -- 13: interior colour = palette darkened toward the core colour; profile fabric copy
            v_w := signed(resize(p12_py, 16)) - shift_right(p12_dy, 3); p13_iy <= unsigned(v_w(9 downto 0));
            v_w := signed(resize(p12_pu, 16)) - shift_right(p12_du, 3); p13_iu <= unsigned(v_w(9 downto 0));
            v_w := signed(resize(p12_pv, 16)) - shift_right(p12_dv, 3); p13_iv <= unsigned(v_w(9 downto 0));
            p13_pf <= r_pf_q;
            -- 14: profile colour (Bare: teal over the glow); interior weight 0 / 1/2 / 1
            if s_bare = '1' and p13_pf(3) = '1' then
                p14_ry <= unsigned(C_TEAL(31 downto 22)); p14_ru <= unsigned(C_TEAL(21 downto 13) & '0'); p14_rv <= unsigned(C_TEAL(12 downto 4) & '0');
            else
                p14_ry <= unsigned(p13_pf(31 downto 22)); p14_ru <= unsigned(p13_pf(21 downto 13) & '0'); p14_rv <= unsigned(p13_pf(12 downto 4) & '0');
            end if;
            p14_al <= unsigned(p13_pf(1 downto 0));
            p14_iy <= p13_iy; p14_iu <= p13_iu; p14_iv <= p13_iv;
            -- 15: compose; palette (Argon: Cr' = -Cb, Cb' = Cr)
            case to_integer(p14_al) is
                when 0 => v_ry := p14_ry; v_ru := p14_ru; v_rv := p14_rv;
                when 1 =>
                    v_h := resize(p14_ry, 11) + p14_iy; v_ry := v_h(10 downto 1);
                    v_h := resize(p14_ru, 11) + p14_iu; v_ru := v_h(10 downto 1);
                    v_h := resize(p14_rv, 11) + p14_iv; v_rv := v_h(10 downto 1);
                when others => v_ry := p14_iy; v_ru := p14_iu; v_rv := p14_iv;
            end case;
            p15_y <= v_ry;
            if s_pal = '1' then p15_u <= not v_rv; p15_v <= v_ru;
            else p15_u <= v_ru; p15_v <= v_rv; end if;
            -- 16: filter input, teal outside the generated avid
            if s_gav_sr(13) = '0' then
                s_y16 <= c_ty; s_u16 <= c_tu; s_v16 <= c_tv;
            else
                s_y16 <= p15_y; s_u16 <= p15_u; s_v16 <= p15_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(13);
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
            cr(to_integer(rc + 6)) <= std_logic_vector(s_u16) & std_logic_vector(s_v16);
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            -- sync ring: written at rc + 18, RAM register, fabric register = C_LAT clocks
            sr(to_integer(rc + 18)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
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
            -- 17: chroma running 8-sums (snap back to teal once 8 blank samples are in, so a glitched
            -- sample can't offset the colours for good); luma taps
            if s_bah = "00000000" then
                s_bau <= c_tu & "000"; s_bav <= c_tv & "000";
            else
                s_bau <= s_bau + s_u16 - unsigned(cr_f(19 downto 10));
                s_bav <= s_bav + s_v16 - unsigned(cr_f(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_ly(0) <= s_y16;
            for i in 1 to 5 loop s_ly(i) <= s_ly(i - 1); end loop;
            -- 18: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 (both centred 4 pixels back)
            v_sc := resize(s_bau, 14) + s_baud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v18 <= v_sc(13 downto 4);
            v_l := resize(s_ly(3), 12) + shift_left(resize(s_ly(4), 12), 1) + resize(s_ly(5), 12);
            s_y18 <= v_l(11 downto 2);
            -- 19: luma 13/16
            if s_y18 > 64 then v_dl := s_y18 - 64; else v_dl := (others => '0'); end if;
            s_y19 <= s_y18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u19 <= s_u18; s_v19 <= s_v18;
            -- 20: pipeline
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
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
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture neon;
