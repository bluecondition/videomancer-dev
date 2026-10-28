-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ovals v0.5.1 -- Heirloom #50, rebuilt from the photo WIN_20231003_23_01_50.
--
-- The photo: a hot-pink ground with horizontal stripes (39.4-line bands), carrying columns of vesica
-- lenses ("ovals") stacked into chains that touch at X points (two lens rows, X rows at y ~256 / ~781).
-- Every lens has the same two ropes: the LEFT edge a green -> blue -> violet -> brown -> magenta band,
-- the RIGHT edge a pure-green band with teal / olive fringes.  The middle column is two overlapping
-- lenses (a double oval); between the columns sit nested diamonds (red-orange cores at the top, pink at
-- the bottom), with cyan stars at the X points.  Lens interiors: red-orange / peach / white rings cut
-- into blocks by the stripes.  A white glow fills the bottom.
--
-- The patch, as fitted (ovals.py, .heirloom_work/rb_ovals/geo3.json):
--   a horizontal triangle (period 1513.9 px), folded at m = 255.3 px and again at L(y) = 185.4 + 59.3 t
--   + 25.2 t^2 (t = (y - 520)/520), keyed against a rectified vertical sine M(y) = 127.1 |sin|^0.816
--   (period 525.6 lines): four lenses per period, edges V_j = c_j - M, G_j = c_j + 1.047 M + dl(y).
--   Q = max_j (h_j - |w - c_j|) (the union of the four lenses: > 0 inside, -distance outside; continuous,
--   the inner pair overlaps into the double oval with a crease), side = the winning lens's half.  Colour =
--   palette ROM[bank, Q] read for BOTH sides at once and blended near every side flip (lens axis, crease,
--   gap midpoint) so no straight seam shows; bank = stripe parity inside a lens and in the deep ground,
--   diamond / ground fringe elsewhere.  Interiors carry cream / white rings and thin dark lines.  Cyan X stars near the X rows, a white glow at the bottom.
--   The oscillator is hsync-reset: left of the first trough the phase holds (the photo's plain left edge).
--
--   K1  Columns   lens columns across (x0.5 .. x2, zoom about the centre)
--   K2  Links     lens rows down the frame (x0.5 .. x2)
--   K3  Split     fold level L: 0 = the pairs merge, 512 = the photo, 1023 = the inner pair crosses
--   K4  Speed     bipolar: the chains flow down / up, the stripes roll with them
--   K5  Ropes     rope / ring width: thin .. fat
--   K6  Etch      how far the camera (or the Osc) bends the rings
--   S7  Stripes   the stripe bands (ground stripes and interior blocks) on / off
--   S8  Glow      the bottom white glow on / off
--   S9  Twist     the ropes swap sides (green on the left, violet on the right)
--   S10 Palette   Candy (the photo) / Ice (U and V exchanged)
--   S11 Source    Video / Osc
--   P12 SWELL     lens bulge: 0 = straight knotted ropes, 512 = the photo, 1023 = huge interlocking ovals
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, knob-curve table), a per-line
-- engine kicked at each line start that computes the NEXT line (sine ROM, the fold level, 12 lens edges,
-- star / glow / stripe values), a wrapping phase DDA beam, a 19-stage pixel path (latency 28).  Virtual 1080-line
-- coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (chroma [1 2 2 2 2 2 2 2 1]
-- /16, luma [1 2 1], luma 13/16) before the blanking gate.  u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.ovals_pkg.all;
use work.all;

architecture ovals of program_top is
    constant C_LAT : integer := 28;          -- = processing_delay_clks (output register = stage 28)
    constant C_PHQ : integer := 3100467;     -- lens period 3027.8 h-units (half px), Q10
    constant C_PHH : integer := 1514;        -- half period, h-units
    constant C_WRP : integer := 1550336;     -- C_PHH * 1024
    constant C_P16 : integer := 48444;       -- period, h-units Q4
    constant C_M16 : integer := 8170;        -- fold centre m = 510.6 h-units, Q4
    constant C_ST0 : integer := 3837275;     -- stripe phase offset 18 / 78.7 lines, Q24
    constant C_WY  : integer := 940;  constant C_WU : integer := 511; constant C_WV : integer := 499;   -- glow white
    constant C_BY  : integer := 546;  constant C_BU : integer := 377; constant C_BV : integer := 732;   -- X star blue

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
    -- x step in h-units per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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

    type t_s12 is array (0 to 3) of signed(11 downto 0);
    type t_s13 is array (0 to 3) of signed(12 downto 0);
    type t_u12 is array (0 to 3) of unsigned(11 downto 0);
    type t_u8  is array (natural range <>) of unsigned(7 downto 0);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);

    -- ROMs (registered address and data)
    signal r_pa  : unsigned(9 downto 0) := (others => '0');
    signal r_py0, r_pu0, r_pv0, r_py1, r_pu1, r_pv1 : std_logic_vector(7 downto 0) := (others => '0');
    signal r_sa  : unsigned(9 downto 0) := (others => '0');
    signal r_sq  : std_logic_vector(7 downto 0) := (others => '0');
    signal r_exi : unsigned(6 downto 0) := (others => '0');
    signal r_exq : unsigned(11 downto 0) := to_unsigned(1024, 12);

    -- oscillator source (Source = Osc): 2-D triangle lattice, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_l8  : unsigned(7 downto 0) := (others => '0');
    signal s_ld  : t_u8(0 to 9) := (others => (others => '0'));

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ysf  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg, s_vnegf : std_logic := '0';
    signal s_strp, s_glow, s_twst, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_i2a, z_i2b : signed(8 downto 0) := (others => '0');
    signal z_i2 : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal c_d    : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal c_dv   : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal c_gq   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_du   : unsigned(13 downto 0) := to_unsigned(2048, 14);
    signal c_u0   : signed(23 downto 0) := to_signed(0, 24);
    signal c_wlim : signed(25 downto 0) := to_signed(C_WRP - 2048, 26);   -- wrap when the phase reaches WRP - du
    signal c_dum  : signed(25 downto 0) := to_signed(2048 - C_PHQ, 26);   -- du - period
    signal c_dphi : unsigned(23 downto 0) := (others => '0');
    signal c_phb  : unsigned(23 downto 0) := (others => '0');
    signal c_a    : unsigned(9 downto 0) := to_unsigned(254, 10);
    signal c_dst  : unsigned(23 downto 0) := (others => '0');
    signal z_t1   : unsigned(23 downto 0) := (others => '0');
    signal z_vq   : unsigned(11 downto 0) := (others => '0');
    signal z_t7   : unsigned(16 downto 0) := (others => '0');
    signal c_fls  : unsigned(23 downto 0) := (others => '0');
    signal c_srs  : unsigned(23 downto 0) := (others => '0');
    signal c_flow : unsigned(23 downto 0) := (others => '0');
    signal c_srl  : unsigned(23 downto 0) := (others => '0');
    signal c_phi0 : unsigned(23 downto 0) := (others => '0');
    signal c_stp0 : unsigned(23 downto 0) := (others => '0');
    signal c_k3   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_k6   : unsigned(9 downto 0) := to_unsigned(200, 10);

    -- field serial multiplier
    signal f_a   : unsigned(39 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(39 downto 0) := (others => '0');
    signal z_fa  : unsigned(39 downto 0) := (others => '0');
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

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_phi  : unsigned(23 downto 0) := (others => '0');
    signal e_stp  : unsigned(23 downto 0) := (others => '0');
    signal e_tqa  : unsigned(9 downto 0) := (others => '0');
    signal e_tneg : std_logic := '0';
    signal e_yl   : unsigned(10 downto 0) := (others => '0');
    signal e_s8   : unsigned(7 downto 0) := (others => '0');
    signal e_lq   : unsigned(9 downto 0) := (others => '0');
    signal e_la   : unsigned(10 downto 0) := (others => '0');
    signal e_lc16 : unsigned(13 downto 0) := (others => '0');
    signal e_m16  : unsigned(12 downto 0) := (others => '0');
    signal e_mg16 : unsigned(13 downto 0) := (others => '0');
    signal e_l16  : unsigned(14 downto 0) := (others => '0');
    signal e_xa   : unsigned(8 downto 0) := (others => '0');
    signal e_da   : unsigned(9 downto 0) := (others => '0');
    signal e_xc16 : signed(16 downto 0) := (others => '0');
    signal e_dl16 : signed(16 downto 0) := (others => '0');
    signal e_hb16 : signed(16 downto 0) := (others => '0');
    signal e_xcl16 : signed(16 downto 0) := (others => '0');
    signal e_gy0  : signed(11 downto 0) := (others => '0');
    signal e_gy   : signed(12 downto 0) := (others => '0');
    signal e_t1, e_t2 : signed(16 downto 0) := (others => '0');
    signal e_lh, e_hb : signed(11 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    signal n_lq, n_hq : t_s12 := (others => (others => '0'));
    signal n_k1, n_k2 : signed(11 downto 0) := (others => '0');
    signal n_fs   : unsigned(6 downto 0) := (others => '0');
    signal n_gy   : unsigned(5 downto 0) := (others => '0');
    signal n_st   : std_logic := '0';
    signal n_w0   : signed(25 downto 0) := (others => '0');
    signal l_lq, l_hq : t_s12 := (others => (others => '0'));
    signal l_k1, l_k2 : signed(11 downto 0) := (others => '0');
    signal l_fs   : unsigned(6 downto 0) := (others => '0');
    signal l_gy   : unsigned(5 downto 0) := (others => '0');
    signal l_st   : std_logic := '0';
    signal l_w0   : signed(25 downto 0) := (others => '0');

    -- beam
    signal b_wq   : signed(25 downto 0) := (others => '0');

    -- pixel path
    signal p1_t   : unsigned(10 downto 0) := (others => '0');
    signal p1_sw, p1_cl : std_logic := '0';
    signal p2_ae  : unsigned(10 downto 0) := (others => '0');
    signal p2_j   : unsigned(1 downto 0) := (others => '0');
    signal p2_do  : signed(12 downto 0) := (others => '0');
    signal p2_se, p2_sw, p2_cl : std_logic := '0';
    signal p3_g   : signed(12 downto 0) := (others => '0');
    signal p3_j   : unsigned(1 downto 0) := (others => '0');
    signal p3_ado : unsigned(11 downto 0) := (others => '0');
    signal p3_ae  : unsigned(10 downto 0) := (others => '0');
    signal p3_so, p3_se, p3_sw, p3_cl : std_logic := '0';
    signal p4_ag  : signed(12 downto 0) := (others => '0');
    signal p4_hq  : signed(11 downto 0) := (others => '0');
    signal p4_qo  : signed(12 downto 0) := (others => '0');
    signal p4_ado : unsigned(11 downto 0) := (others => '0');
    signal p4_ae  : unsigned(10 downto 0) := (others => '0');
    signal p4_gp, p4_gz, p4_sg, p4_se, p4_cl, p4_so, p4_sw : std_logic := '0';
    signal p5_q   : signed(14 downto 0) := (others => '0');
    signal p5_qo  : signed(12 downto 0) := (others => '0');
    signal p5_ag, p5_ado : unsigned(11 downto 0) := (others => '0');
    signal p5_ae  : unsigned(10 downto 0) := (others => '0');
    signal p5_s, p5_gr, p5_cl, p5_so, p5_sw, p5_se : std_logic := '0';
    signal p6_q   : signed(14 downto 0) := (others => '0');
    signal p6_df  : signed(14 downto 0) := (others => '0');
    signal p6_ea  : unsigned(11 downto 0) := (others => '0');
    signal p6_ae  : unsigned(10 downto 0) := (others => '0');
    signal p6_s, p6_gr, p6_cl, p6_fd, p6_fc : std_logic := '0';
    signal p7_dq, p7_e1 : unsigned(11 downto 0) := (others => '0');
    signal p7_fc  : std_logic := '0';
    signal p8_e   : unsigned(11 downto 0) := (others => '0');
    signal p8_bs  : std_logic := '0';
    signal p9_b, p10_b, p11_b, p12_b, p13_b : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal p7_pa  : signed(20 downto 0) := (others => '0');
    signal p7_pb  : signed(19 downto 0) := (others => '0');
    signal p7_et  : unsigned(17 downto 0) := (others => '0');
    signal p7_s, p7_gr, p7_in, p7_cl : std_logic := '0';
    signal p8_s1  : signed(25 downto 0) := (others => '0');
    signal p8_e2  : unsigned(8 downto 0) := (others => '0');
    signal p8_s, p8_gr, p8_in, p8_cl : std_logic := '0';
    signal p9_qg  : signed(17 downto 0) := (others => '0');
    signal p9_s, p9_gr, p9_in, p9_cl : std_logic := '0';
    signal p10_v  : signed(17 downto 0) := (others => '0');
    signal p10_aq : unsigned(5 downto 0) := (others => '0');
    signal p10_s, p10_gr, p10_in, p10_cl : std_logic := '0';
    signal p11_ki : unsigned(6 downto 0) := (others => '0');
    signal p11_aq : unsigned(5 downto 0) := (others => '0');
    signal p12_k  : unsigned(3 downto 0) := (others => '0');
    signal p12_ws : unsigned(12 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p13_ws : unsigned(6 downto 0) := (others => '0');
    signal p13_wg : unsigned(9 downto 0) := (others => '0');
    signal p13_ay, p13_au, p13_av, p13_by, p13_bu, p13_bv : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal p14_ay, p14_au, p14_av : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal p14_ey, p14_eu, p14_ev : signed(8 downto 0) := (others => '0');
    signal p14_b  : unsigned(3 downto 0) := (others => '0');
    signal p15_ay, p15_au, p15_av : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal p15_ey, p15_eu, p15_ev : signed(13 downto 0) := (others => '0');
    signal p14_ws, p15_ws, p16_ws : unsigned(6 downto 0) := (others => '0');
    signal p14_wg, p15_wg, p16_wg : unsigned(9 downto 0) := (others => '0');
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p17_y, p17_u, p17_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p17_dy, p17_du, p17_dv : signed(11 downto 0) := (others => '0');
    signal p17_w  : unsigned(6 downto 0) := (others => '0');
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p18_py, p18_pu, p18_pv : signed(19 downto 0) := (others => '0');
    signal p19_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p19_u, p19_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums, luma [1 2 1])
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17, s_y18, s_y19, s_y20, s_y21, s_y22x, s_yf22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y23, s_y24, s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs: palette (6 x 1024 x 8: both sides per address), lens-row bulge sine (1024 x 8), knob curve
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_py0 <= C_OV_PY0(to_integer(r_pa));
            r_pu0 <= C_OV_PU0(to_integer(r_pa));
            r_pv0 <= C_OV_PV0(to_integer(r_pa));
            r_py1 <= C_OV_PY1(to_integer(r_pa));
            r_pu1 <= C_OV_PU1(to_integer(r_pa));
            r_pv1 <= C_OV_PV1(to_integer(r_pa));
            r_sq <= C_OV_SIN(to_integer(r_sa));
            r_exq <= C_OV_EXP(to_integer(r_exi));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_i  : signed(8 downto 0);
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
            elsif s_zc /= 1023 then
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
                s_strp <= not registers_in(6)(0);       -- S7 Stripes: on by default
                s_glow <= not registers_in(6)(1);       -- S8 Glow: on by default
                s_twst <= registers_in(6)(2);
                s_pal  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_ysf  <= s_ys;
                s_vnegf <= s_vneg;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

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
                        elsif s_gd > 0 then
                            s_gd <= to_signed(1, 15);
                        elsif s_gd < 0 then
                            s_gd <= to_signed(-1, 15);
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

            -- knob-curve lookups (after the glide, zc 40..52): density, links, rope gain
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 => r_exi <= s_g1(12 downto 6);
                               z_i2a <= signed(resize(s_g2(12 downto 6), 9)) - 64;
                    when 41 => z_i2b <= shift_right(shift_left(z_i2a, 2) + z_i2a, 4);
                    when 42 => c_d <= r_exq;
                               v_i := z_i2a + z_i2b + 64;
                               if v_i < 0 then z_i2 <= (others => '0');
                               elsif v_i > 127 then z_i2 <= (others => '1');
                               else z_i2 <= unsigned(v_i(6 downto 0)); end if;
                    when 43 => r_exi <= z_i2;
                               case to_integer(s_xs) is
                                   when 4      => c_du <= shift_left(resize(c_d, 14), 2);
                                   when 3      => c_du <= shift_left(resize(c_d, 14), 1) + resize(c_d, 14);
                                   when others => c_du <= shift_left(resize(c_d, 14), 1);
                               end case;
                    when 45 => c_dv <= r_exq;
                               r_exi <= 127 - s_g5(12 downto 6);
                    when 46 => c_wlim <= to_signed(C_WRP, 26) - signed(resize(c_du, 26));
                               c_dum  <= signed(resize(c_du, 26)) - C_PHQ;
                    when 47 => c_gq <= r_exq(11 downto 2);
                               c_k3 <= s_g3(12 downto 3);
                               c_k6 <= s_g6(12 downto 3);
                    when others => null;
                end case;
            end if;

            -- serial products: slot k loads at zc = 64 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at 82 + 24 k).  Operands pre-muxed at 63 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 62  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 278 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 =>                                                            -- (1920 - xst) * d
                        z_fa <= resize(c_d, 40);
                        if s_xs = 4 then z_fb <= to_unsigned(1440, 16); else z_fb <= to_unsigned(1920, 16); end if;
                    when 1 => z_fa <= resize(c_dv, 40);   z_fb <= resize(s_ysf, 16);      -- ys * dv
                    when 2 => z_fa <= resize(z_t1, 40);   z_fb <= to_unsigned(7979, 16);   -- dphi
                    when 3 => z_fa <= to_unsigned(8158890, 40); z_fb <= resize(c_dv, 16);  -- y0 / P
                    when 4 => z_fa <= resize(s_gp(12 downto 3), 40); z_fb <= to_unsigned(254, 16);  -- swell
                    when 5 => z_fa <= resize(s_ysf, 40);  z_fb <= to_unsigned(833, 16);    -- stripe step
                    when 6 => z_fa <= resize(s_vm, 40);   z_fb <= to_unsigned(780, 16);    -- speed
                    when 7 => z_fa <= resize(c_dv, 40);   z_fb <= to_unsigned(7979, 16);
                    when 8 => z_fa <= resize(z_t7, 40);   z_fb <= resize(z_vq, 16);        -- flow step
                    when others => z_fa <= resize(z_vq, 40); z_fb <= to_unsigned(833, 16); -- stripe roll
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 82  => c_u0 <= to_signed(1966080, 24) - signed(f_acc(23 downto 0));
                    when 106 => z_t1 <= f_acc(23 downto 0);
                    when 130 => c_dphi <= resize(f_acc(39 downto 16), 24);
                    when 154 => c_phb <= f_acc(33 downto 10);
                    when 178 => c_a <= f_acc(18 downto 9);
                    when 202 => c_dst <= f_acc(23 downto 0);
                    when 226 => z_vq <= f_acc(19 downto 8);
                    when 250 => z_t7 <= f_acc(24 downto 8);
                    when 274 => c_fls <= f_acc(31 downto 8);
                    when 298 => c_srs <= f_acc(31 downto 8);
                    when 300 =>
                        if s_vnegf = '1' then c_flow <= c_flow - c_fls; c_srl <= c_srl - c_srs;
                        else c_flow <= c_flow + c_fls; c_srl <= c_srl + c_srs; end if;
                    when 302 =>
                        c_phi0 <= 0 - c_phb;
                        c_stp0 <= 0 - to_unsigned(C_ST0, 24);
                    when 304 =>
                        c_phi0 <= c_phi0 - c_flow;
                        c_stp0 <= c_stp0 - c_srl;
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
    -- Source side: oscillator (2-D triangle lattice, drifting) or the camera's
    -- luma, (y - 64) >> 2; a 10-clock delay brings it to the pixel path's etch
    -- stage
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_s : unsigned(7 downto 0);
        variable v_y : unsigned(9 downto 0);
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
            v_y := unsigned(data_in.y);
            if s_src = '1' then
                s_l8 <= s_osum;
            elsif v_y > 940 then
                s_l8 <= to_unsigned(219, 8);
            elsif v_y > 64 then
                s_l8 <= resize(shift_right(v_y - 64, 2), 8);
            else
                s_l8 <= (others => '0');
            end if;
            s_ld(0) <= s_l8;
            for i in 1 to 9 loop s_ld(i) <= s_ld(i - 1); end loop;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17):
    --   tq = y - 520 (virtual lines);  s = SIN[phase];  M = A s;  MG = 1.047 M
    --   L = (370.8 + 0.228 tq + 1.86e-4 tq^2) * split;  xc, dl linear in tq
    --   lens j: centre c_j = -+(m +- L) + o_j, half-width h_j = (M + MG + dl)/2 + p_j;
    --   edges V_j = c_j - h_j, G_j = c_j + h_j relative to xcl = xc + (MG - M + dl)/2, plus
    --   the lens one period either side; star, glow, stripe values; the line's start phase
    -- n_* are copied to l_* in the horizontal blanking, after the line has drained.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t  : signed(11 downto 0);
        variable v_tn : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy  <= (others => '0');
                e_phi <= c_phi0;
                e_stp <= c_stp0;
                e_lc  <= (others => '0');
            elsif g_start = '1' then
                e_vy  <= e_vy + resize(s_ysf, 19);
                e_phi <= e_phi + c_dphi;
                e_stp <= e_stp + c_dst;
                e_lc  <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_yl <= e_vy(18 downto 8);
                    r_sa <= e_phi(23 downto 14);
                when 2 =>
                    v_t := signed(resize(e_yl, 12)) - 520;
                    if v_t < 0 then e_tneg <= '1'; v_tn := 0 - v_t; e_tqa <= unsigned(v_tn(9 downto 0));
                    else e_tneg <= '0'; e_tqa <= unsigned(v_t(9 downto 0)); end if;
                    e_gy0 <= signed(resize(e_yl, 12)) - 945;
                when 3 =>
                    m_a <= resize(e_tqa, 32); m_b <= resize(e_tqa, 16); m_acc <= (others => '0');
                    e_s8 <= unsigned(r_sq);
                when 20 =>
                    m_a <= resize(m_acc(19 downto 0), 32); m_b <= to_unsigned(195, 16); m_acc <= (others => '0');
                when 37 =>
                    e_lq <= m_acc(25 downto 16);
                    m_a <= resize(e_tqa, 32); m_b <= to_unsigned(934, 16); m_acc <= (others => '0');
                when 54 =>
                    e_la <= m_acc(18 downto 8);
                    m_a <= resize(c_a, 32); m_b <= resize(e_s8, 16); m_acc <= (others => '0');
                when 55 =>
                    if e_tneg = '1' then
                        e_lc16 <= to_unsigned(5933, 14) - resize(e_la, 14) + resize(e_lq, 14);
                    else
                        e_lc16 <= to_unsigned(5933, 14) + resize(e_la, 14) + resize(e_lq, 14);
                    end if;
                when 71 =>
                    e_m16 <= m_acc(16 downto 4);
                    m_a <= resize(e_lc16, 32); m_b <= resize(c_k3, 16); m_acc <= (others => '0');
                when 72 =>
                    e_mg16 <= resize(e_m16, 14) + resize(shift_right(e_m16, 5), 14) + resize(shift_right(e_m16, 6), 14);
                when 88 =>
                    e_l16 <= m_acc(23 downto 9);
                    m_a <= resize(e_tqa, 32); m_b <= to_unsigned(478, 16); m_acc <= (others => '0');
                when 105 =>
                    e_xa <= m_acc(18 downto 10);
                    m_a <= resize(e_tqa, 32); m_b <= to_unsigned(1058, 16); m_acc <= (others => '0');
                when 122 =>
                    e_da <= m_acc(19 downto 10);
                when 123 =>
                    if e_tneg = '1' then
                        e_xc16 <= to_signed(30963, 17) - signed(resize(e_xa, 17));
                        e_dl16 <= to_signed(782, 17) + signed(resize(e_da, 17));
                    else
                        e_xc16 <= to_signed(30963, 17) + signed(resize(e_xa, 17));
                        e_dl16 <= to_signed(782, 17) - signed(resize(e_da, 17));
                    end if;
                when 124 =>
                    e_t1 <= signed(resize(e_m16, 17)) + signed(resize(e_mg16, 17));
                    e_t2 <= signed(resize(e_mg16, 17)) - signed(resize(e_m16, 17));
                    if e_gy0 <= 0 then e_gy <= (others => '0');
                    else e_gy <= shift_right(resize(e_gy0, 13) + shift_left(resize(e_gy0, 13), 3), 3); end if;
                when 125 =>
                    e_hb16 <= shift_right(e_t1 + e_dl16, 1);
                    e_t2   <= shift_right(e_t2 + e_dl16, 1);
                when 126 =>
                    e_xcl16 <= e_xc16 + e_t2;
                    e_lh <= signed(resize(e_l16(14 downto 4), 12));
                    e_hb <= e_hb16(15 downto 4);
                when 127 =>
                    -- per lens quadrant: L + sigma_j * centre offset, half-width + offset (h-units); and for the
                    -- union with the other half's inner lens: m - L_1, m - L_2 (its centre's distance from w = 0)
                    -- (each clamped at 0: at Split 0 the pairs merge into one lens instead of folding inside out)
                    if e_lh < 0 then n_lq(0) <= (others => '0'); else n_lq(0) <= e_lh; end if;
                    if e_lh < -12 then n_lq(1) <= (others => '0'); else n_lq(1) <= e_lh + 12; end if;
                    if e_lh < 16 then n_lq(2) <= (others => '0'); else n_lq(2) <= e_lh - 16; end if;
                    if e_lh < 25 then n_lq(3) <= (others => '0'); else n_lq(3) <= e_lh - 25; end if;
                when 128 =>
                    n_k1 <= to_signed(511, 12) - n_lq(1); n_k2 <= to_signed(511, 12) - n_lq(2);
                    n_hq(0) <= e_hb - 4;  n_hq(1) <= e_hb - 3;  n_hq(2) <= e_hb - 6;  n_hq(3) <= e_hb - 25;
                    if e_s8 < 64 then n_fs <= to_unsigned(64, 7) - resize(e_s8, 7); else n_fs <= (others => '0'); end if;
                    n_st <= e_stp(23);
                    n_w0 <= resize(c_u0, 26) - shift_left(resize(e_xcl16, 26), 6);
                    if e_gy > 61 then n_gy <= to_unsigned(61, 6); else n_gy <= unsigned(e_gy(5 downto 0)); end if;
                when others => null;
            end case;

            -- hand-over in the blanking, once the previous line has left every stage that reads l_*
            if g_avid = '0' and s_gav_sr(14) = '0' and s_gav_sr(0) = '0' then
                l_lq <= n_lq; l_hq <= n_hq; l_k1 <= n_k1; l_k2 <= n_k2; l_fs <= n_fs; l_gy <= n_gy; l_st <= n_st; l_w0 <= n_w0;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: lens phase DDA (h-units Q10), wrapped at +half period; loaded with
    -- the line's start phase in the blanking
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_avid = '0' then
                b_wq <= l_w0;
            elsif b_wq >= c_wlim then
                b_wq <= b_wq + c_dum;
            else
                b_wq <= b_wq + signed(resize(c_du, 26));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = the beam register of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_w   : signed(15 downto 0);
        variable v_n   : signed(12 downto 0);
        variable v_df  : signed(14 downto 0);
        variable v_dg  : signed(14 downto 0);
        variable v_e4  : unsigned(10 downto 0);
        variable v_qp  : signed(17 downto 0);
        variable v_i   : unsigned(8 downto 0);
        variable v_b   : std_logic;
        variable v_sd  : std_logic;
        variable v_st  : std_logic;
        variable v_ty, v_tu, v_tv : unsigned(9 downto 0);
        variable v_wt  : unsigned(6 downto 0);
        variable v_y, v_u, v_v : signed(11 downto 0);
        variable v_cy, v_cu, v_cv : signed(11 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: T = |w| (the lens triangle); the phase holds left of the first trough (hsync-reset oscillator)
            v_w := b_wq(25 downto 10);
            if v_w < -C_PHH then p1_t <= to_unsigned(C_PHH, 11); p1_sw <= '0'; p1_cl <= '1';
            elsif v_w < 0 then v_n := resize(0 - v_w, 13); p1_t <= unsigned(v_n(10 downto 0)); p1_sw <= '0'; p1_cl <= '0';
            else p1_t <= unsigned(v_w(10 downto 0)); p1_sw <= '1'; p1_cl <= '0'; end if;
            -- 2: first fold at m: |T - m|; this half's lens j = {w >= 0, (w >= 0) xnor (T >= m)};
            --    the other half's inner lens: do = T + (m - L_other)
            if p1_t >= 511 then p2_ae <= p1_t - 511; p2_se <= '1'; p2_j <= p1_sw & p1_sw;
            else p2_ae <= 511 - p1_t; p2_se <= '0'; p2_j <= p1_sw & not p1_sw; end if;
            if p1_sw = '1' then p2_do <= signed(resize(p1_t, 13)) + resize(l_k1, 13);
            else p2_do <= signed(resize(p1_t, 13)) + resize(l_k2, 13); end if;
            p2_sw <= p1_sw; p2_cl <= p1_cl;
            -- 3: second fold level: g = |T - m| - (L + offset_j); |do| (slice, not a narrowing resize), its side
            p3_g <= signed(resize(p2_ae, 13)) - resize(l_lq(to_integer(p2_j)), 13);
            if p2_do < 0 then v_n := 0 - p2_do; p3_ado <= unsigned(v_n(11 downto 0));
            else p3_ado <= unsigned(p2_do(11 downto 0)); end if;
            if p2_sw = '1' then
                if p2_do >= 0 then p3_so <= '1'; else p3_so <= '0'; end if;
            else
                if p2_do <= 0 then p3_so <= '1'; else p3_so <= '0'; end if;
            end if;
            p3_j <= p2_j; p3_se <= p2_se; p3_sw <= p2_sw; p3_ae <= p2_ae; p3_cl <= p2_cl;
            -- 4: |g|, the quadrant's half-width; the other lens: qo = h_other - |do|
            if p3_g < 0 then p4_ag <= 0 - p3_g; p4_gp <= '0'; else p4_ag <= p3_g; p4_gp <= '1'; end if;
            if p3_g > 0 then p4_gz <= '1'; else p4_gz <= '0'; end if;
            p4_hq <= l_hq(to_integer(p3_j));
            if p3_sw = '1' then p4_qo <= resize(l_hq(1), 13) - signed(resize(p3_ado, 13));
            else p4_qo <= resize(l_hq(2), 13) - signed(resize(p3_ado, 13)); end if;
            p4_sg <= p3_j(0); p4_se <= p3_se; p4_cl <= p3_cl;
            p4_ado <= p3_ado; p4_so <= p3_so; p4_sw <= p3_sw; p4_ae <= p3_ae;
            -- 5: Qf = h_j - |g| (> 0 inside a lens, -distance outside); its side (1 = the right / G half);
            --    ground = beyond the outer lenses
            p5_q <= resize(p4_hq, 15) - resize(p4_ag, 15);
            p5_s <= not (p4_gp xor p4_sg);
            p5_gr <= p4_se and p4_gz;
            p5_ag <= unsigned(p4_ag(11 downto 0));
            p5_qo <= p4_qo; p5_ado <= p4_ado; p5_so <= p4_so; p5_sw <= p4_sw; p5_se <= p4_se; p5_ae <= p4_ae;
            p5_cl <= p4_cl;
            -- 6: the union Q = max(Qf, qo); side, ground, the winning lens's axis distance; which flips are near
            v_df := resize(p5_qo, 15) - p5_q;
            p6_df <= v_df;
            if v_df > 0 then
                p6_q <= resize(p5_qo, 15); p6_s <= p5_so; p6_gr <= '0'; p6_ea <= p5_ado; p6_fd <= '0';
                p6_fc <= not (p5_so xor p5_sw);
            else
                p6_q <= p5_q; p6_s <= p5_s; p6_gr <= p5_gr; p6_ea <= p5_ag;
                if p5_se = '0' then p6_fd <= not (p5_s xor p5_sw); p6_fc <= p5_s xor p5_sw;
                else p6_fd <= p5_s xor p5_sw; p6_fc <= '0'; end if;
            end if;
            p6_ae <= p5_ae; p6_cl <= p5_cl;
            -- 7: crease / gap half-distance |Qf - qo| / 2; e1 = min(axis, diamond centre line if it faces it)
            if p6_df < 0 then v_dg := 0 - p6_df; else v_dg := p6_df; end if;
            if v_dg(14 downto 13) /= "00" then p7_dq <= (others => '1'); else p7_dq <= unsigned(v_dg(12 downto 1)); end if;
            if p6_fd = '1' and resize(p6_ae, 12) < p6_ea then p7_e1 <= resize(p6_ae, 12); else p7_e1 <= p6_ea; end if;
            p7_fc <= p6_fc;
            -- 8: e = min(e1, the crease / gap with the other inner lens if it faces it)
            if p7_fc = '1' and p7_dq < p7_e1 then p8_e <= p7_dq; else p8_e <= p7_e1; end if;
            p8_bs <= p7_s;
            -- 9: blend weight of the G-side palette, b = 8 +- e/4 clamped 0..16
            v_e4 := resize(p8_e(11 downto 2), 11);
            if v_e4 >= 8 then
                if p8_bs = '1' then p9_b <= to_unsigned(16, 5); else p9_b <= to_unsigned(0, 5); end if;
            elsif p8_bs = '1' then p9_b <= to_unsigned(8, 5) + resize(v_e4(2 downto 0), 5);
            else p9_b <= to_unsigned(8, 5) - resize(v_e4(2 downto 0), 5); end if;
            p10_b <= p9_b; p11_b <= p10_b; p12_b <= p11_b; p13_b <= p12_b;
            -- 7: rope gain partial products, etch product, ground-gap flag
            p7_pa <= p6_q * signed('0' & c_gq(4 downto 0));
            p7_pb <= resize(p6_q * signed('0' & c_gq(9 downto 5)), 20);
            p7_et <= s_ld(9) * c_k6;
            p7_s  <= p6_s;
            p7_gr <= p6_gr;
            if p6_q >= 0 then p7_in <= '1'; else p7_in <= '0'; end if;
            p7_cl <= p6_cl;
            -- 8: Q * gain; etch >> 9
            p8_s1 <= resize(p7_pa, 26) + shift_left(resize(p7_pb, 26), 5);
            p8_e2 <= p7_et(17 downto 9);
            p8_s <= p7_s; p8_gr <= p7_gr; p8_in <= p7_in; p8_cl <= p7_cl;
            -- 9: Qg (h-units)
            p9_qg <= resize(p8_s1(25 downto 8), 18) + signed(resize(p8_e2, 18));
            p9_s <= p8_s; p9_gr <= p8_gr; p9_in <= p8_in; p9_cl <= p8_cl;
            -- 10: px index + 320, |px|
            v_qp := shift_right(p9_qg, 1);
            p10_v <= v_qp + 320;
            if v_qp > 31 or v_qp < -31 then p10_aq <= to_unsigned(32, 6);
            elsif v_qp(17) = '1' then p10_aq <= unsigned(0 - v_qp(5 downto 0));
            else p10_aq <= unsigned(v_qp(5 downto 0)); end if;
            p10_s <= p9_s; p10_gr <= p9_gr; p10_in <= p9_in; p10_cl <= p9_cl;
            -- 11: palette address {bank, index} (one word = both sides); glow-table side = the dominant palette
            if p10_cl = '1' or p10_v < 0 then v_i := (others => '0');
            elsif p10_v > 511 then v_i := (others => '1');
            else v_i := unsigned(p10_v(8 downto 0)); end if;
            v_st := l_st or not s_strp;
            if p10_in = '1' or p10_cl = '1' or p10_v < 170 then v_b := v_st; else v_b := not p10_gr; end if;
            v_sd := p10_b(4) or p10_b(3);              -- b >= 8: the G-side palette dominates
            v_sd := v_sd xor s_twst;
            r_pa <= v_b & v_i;
            p11_ki <= v_sd & v_i(8 downto 3);
            p11_aq <= to_unsigned(32, 6) - p10_aq;
            -- 12: palette ROMs (EBR); glow weight table; star weight
            p12_k <= C_OV_KAP(to_integer(p11_ki));
            p12_ws <= resize(l_fs * p11_aq, 13);
            -- 13: fabric copy of both sides (A = the V-side palette, B = the G side; Twist swaps), weights
            if s_twst = '1' then
                p13_ay <= unsigned(r_py1); p13_au <= unsigned(r_pu1); p13_av <= unsigned(r_pv1);
                p13_by <= unsigned(r_py0); p13_bu <= unsigned(r_pu0); p13_bv <= unsigned(r_pv0);
            else
                p13_ay <= unsigned(r_py0); p13_au <= unsigned(r_pu0); p13_av <= unsigned(r_pv0);
                p13_by <= unsigned(r_py1); p13_bu <= unsigned(r_pu1); p13_bv <= unsigned(r_pv1);
            end if;
            p13_ws <= p12_ws(11 downto 5);
            if s_glow = '1' then p13_wg <= l_gy * p12_k; else p13_wg <= (others => '0'); end if;
            -- 14: side blend difference B - A (8-bit palette values); weight b/2 in eighths
            p14_ey <= signed(resize(p13_by, 9)) - signed(resize(p13_ay, 9));
            p14_eu <= signed(resize(p13_bu, 9)) - signed(resize(p13_au, 9));
            p14_ev <= signed(resize(p13_bv, 9)) - signed(resize(p13_av, 9));
            p14_ay <= p13_ay; p14_au <= p13_au; p14_av <= p13_av;
            p14_b <= p13_b(4 downto 1); p14_ws <= p13_ws; p14_wg <= p13_wg;
            -- 15: (B - A) * b8
            p15_ey <= p14_ey * signed('0' & p14_b);
            p15_eu <= p14_eu * signed('0' & p14_b);
            p15_ev <= p14_ev * signed('0' & p14_b);
            p15_ay <= p14_ay; p15_au <= p14_au; p15_av <= p14_av;
            p15_ws <= p14_ws; p15_wg <= p14_wg;
            -- 16: blended palette colour (A + (B - A) b8 / 8) x 4 + 2
            v_cy := signed(resize(p15_ay, 12)) + resize(p15_ey(13 downto 3), 12);
            v_cu := signed(resize(p15_au, 12)) + resize(p15_eu(13 downto 3), 12);
            v_cv := signed(resize(p15_av, 12)) + resize(p15_ev(13 downto 3), 12);
            p16_y <= unsigned(v_cy(7 downto 0)) & "10"; p16_u <= unsigned(v_cu(7 downto 0)) & "10"; p16_v <= unsigned(v_cv(7 downto 0)) & "10";
            p16_ws <= p15_ws; p16_wg <= p15_wg;
            -- 17: blend target (X star blue first, else the glow white) and weight
            if p16_ws /= 0 then
                v_ty := to_unsigned(C_BY, 10); v_tu := to_unsigned(C_BU, 10); v_tv := to_unsigned(C_BV, 10); v_wt := p16_ws;
            else
                v_ty := to_unsigned(C_WY, 10); v_tu := to_unsigned(C_WU, 10); v_tv := to_unsigned(C_WV, 10); v_wt := resize(p16_wg(9 downto 4), 7);
            end if;
            p17_dy <= signed(resize(v_ty, 12)) - signed(resize(p16_y, 12));
            p17_du <= signed(resize(v_tu, 12)) - signed(resize(p16_u, 12));
            p17_dv <= signed(resize(v_tv, 12)) - signed(resize(p16_v, 12));
            p17_w <= v_wt;
            p17_y <= p16_y; p17_u <= p16_u; p17_v <= p16_v;
            -- 18: blend products
            p18_py <= p17_dy * signed('0' & p17_w);
            p18_pu <= p17_du * signed('0' & p17_w);
            p18_pv <= p17_dv * signed('0' & p17_w);
            p18_y <= p17_y; p18_u <= p17_u; p18_v <= p17_v;
            -- 19: compose; palette swap; black outside the generated avid
            v_y := signed(resize(p18_y, 12)) + resize(p18_py(19 downto 6), 12);
            v_u := signed(resize(p18_u, 12)) + resize(p18_pu(19 downto 6), 12);
            v_v := signed(resize(p18_v, 12)) + resize(p18_pv(19 downto 6), 12);
            if s_gav_sr(18) = '0' then
                p19_y <= to_unsigned(64, 10); p19_u <= to_unsigned(512, 10); p19_v <= to_unsigned(512, 10);
            elsif s_pal = '1' then
                p19_y <= unsigned(v_y(9 downto 0)); p19_u <= unsigned(v_v(9 downto 0)); p19_v <= unsigned(v_u(9 downto 0));
            else
                p19_y <= unsigned(v_y(9 downto 0)); p19_u <= unsigned(v_u(9 downto 0)); p19_v <= unsigned(v_v(9 downto 0));
            end if;

            -- 20..25: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]; centre at stage 24 -> 25)
            s_csu(0) <= p19_u; s_csv(0) <= p19_v;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + p19_u - s_csu(7);
            s_boxv <= s_boxv + p19_v - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u22 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v22 <= v_sc(13 downto 4);
            s_y17 <= p19_y; s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19;
            s_y21 <= s_y20; s_y22x <= s_y21;
            v_sy := resize(s_y20, 12) + shift_left(resize(s_y21, 12), 1) + resize(s_y22x, 12);
            s_yf22 <= v_sy(11 downto 2);
            -- 26: luma toward black (white 940 -> 777); blanking gate
            if s_yf22 > 64 then v_dl := s_yf22 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(24) = '1' then
                s_y23 <= s_yf22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
            -- 27, 28: output registers (latency padded to a multiple of 4)
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;
            s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ovals;
