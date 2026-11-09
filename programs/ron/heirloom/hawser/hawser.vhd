-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Hawser v0.1.0 -- Heirloom from the video clips, built from the still WIN_20231116_23_41_37 t3.5
-- (clip WIN_20231116_23_41_37, 9.5 s; the same patch in clip 23_43_35, turned to lemon).
--
-- The still: solid amber rope tubes with dark-orange edges and a silver rim swing in big S-curves over
-- bundles of thin peach / amber / silver cords on black, with red glints on the cords.  The swing grows from
-- nothing at the left edge to ~300 px at the right, and the S-curves travel up the frame (~3 s per writhe).
--
-- The patch, as fitted (hawser.py has the tables; .heirloom_work/rb_hawser/notes.md the measurements), in
-- virtual 1080-line coordinates:
--   tubes : p1 = (x + 290) k1 (1 - a1 w1(y))      two hsync-synced horizontal oscillators, each FREQUENCY-
--   cords : p2 = (x + 146) k2 (1 + a2 w2(y))      modulated by a vertical LFO: w = (1 - cos 2 pi (y F + phi)) / 2
--           k1 = 1 / 420 px, k2 = 1 / 304 px, 1 / F1 = 1120 lines, F2 = 2 F1, a1 = 0.153, a2 = 0.366 a1
--   colour: the tube profile (keyed) over the cord profile over black; glints over everything
--   glints: a cell per cord bundle and row (310 / 620 lines): G = size - |dx + slant dy| - 0.585 |dy| -
--           zigzags + camera, coloured in rings (red, salmon, pink, a white line, red, tinted red ...)
--
--   K1  Tubes     tube count: 1.4 .. 15 across (4.6 = the photo)
--   K2  Wave      the LFO wavelength: 4480 .. 280 lines (1120 = the photo); the cords at half of it
--   K3  Cords     cord-bundle count: 1.9 .. 21 across (6.3 = the photo)
--   K4  Speed     bipolar (+-8 deadband at 512): the S-curves travel up (right) or down (left); 700 = the clip
--   K5  Glints    glint size: 0 = none, 512 = the photo, 1023 = big red rings
--   K6  Camera    how far the picture (or the Osc) bends the ropes and sprouts glints (square law)
--   S7  Braid     Off (the photo) / On: the cords become a second set of tubes swinging against the first
--   S8  Palette   Amber (the photo) / Lemon (clip 23_43_35: lemon tubes, maroon rims, white cords)
--   S9  Mirror    Off / On: the right half mirrors the left (still edges, a wild middle)
--   S10 Wire      Solid / Wire: the tubes are hollow (only their edges and rims), the cords show through
--   S11 Source    Video / Osc
--   P12 TWIST     the tubes' swing depth a1: 0 = straight hanging ropes, 435 = the photo, 100 % = 0.36
--
-- Engine: vblank-zone sequencer (shared glide, exp2 knob curves, one serial multiplier), a per-line engine kicked
-- at each line start computing the NEXT line (both LFO cosines, the two FM line steps and line starts, the glint
-- row terms), an 18-stage pixel path (two phase DDAs that reverse at the centre under Mirror, the camera bend,
-- two palette ROMs, the glint cell ROM, one camera and one glint product), the shell's output conditioning (Y, U, V
-- through two [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps sit in block-RAM rings, with the per-line resync;
-- luma 13/16) and the blanking gate.  Generated avid; one latency (24) in every mode; hardware colour convention.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.hawser_pkg.all;
use work.all;

architecture hawser of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks
    constant C_PHI1_0 : integer := 931551;   -- the still's LFO phases at power-up (Q20 cycles)
    constant C_PHI2_0 : integer := 131296;

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
    -- triangle of a Q16 phase: 2047 at frac 0, 0 at 1/2
    function f_tri11(z : unsigned(15 downto 0)) return unsigned is
    begin
        if z(15) = '1' then return z(14 downto 4); else return not z(14 downto 4); end if;
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
    signal r_t_a  : unsigned(8 downto 0) := (others => '0');
    signal r_t_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_c_a  : unsigned(9 downto 0) := (others => '0');
    signal r_c_q  : std_logic_vector(23 downto 0) := (others => '0');
    signal r_sq_a : unsigned(8 downto 0) := (others => '0');
    signal r_sq_q : std_logic_vector(15 downto 0) := (others => '0');
    signal z_sqa, e_sqa : unsigned(8 downto 0) := (others => '0');
    signal r_sb_a : unsigned(7 downto 0) := (others => '0');
    signal r_sb_q : std_logic_vector(15 downto 0) := (others => '0');

    attribute no_rw_check : boolean;

    -- output box rings (the 8-back taps of the two running sums) and the sync delay ring
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ba1, ba2, bb1, bb2 : t_r16 := (others => x"0000");
    attribute no_rw_check of ba1 : signal is true;
    attribute no_rw_check of ba2 : signal is true;
    attribute no_rw_check of bb1 : signal is true;
    attribute no_rw_check of bb2 : signal is true;
    signal rg_wa, rg_ra, sy_ra : unsigned(7 downto 0) := (others => '0');
    signal ba1_q, ba2_q, bb1_q, bb2_q : std_logic_vector(15 downto 0) := x"0000";
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal syr : t_r4 := (others => "1111");
    attribute no_rw_check of syr : signal is true;
    signal syr_q, syr_c : std_logic_vector(3 downto 0) := "1111";

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

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(700 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(384 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(435 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_braid, s_lemon, s_mirror, s_wire, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal f_a    : unsigned(16 downto 0) := (others => '0');
    signal f_b    : unsigned(15 downto 0) := (others => '0');
    signal f_acc  : unsigned(33 downto 0) := (others => '0');
    signal f_cnt  : unsigned(4 downto 0) := (others => '0');
    signal z_fms  : unsigned(2 downto 0) := (others => '0');
    signal z_fld  : std_logic := '0';
    signal z_e3   : signed(11 downto 0) := (others => '0');
    signal z_d    : unsigned(8 downto 0) := (others => '0');
    signal z_dn   : std_logic := '0';
    signal z_dsq  : unsigned(15 downto 0) := (others => '0');
    signal z_ksq  : unsigned(15 downto 0) := (others => '0');
    signal z_k6   : unsigned(9 downto 0) := (others => '0');
    signal z_pp   : unsigned(9 downto 0) := (others => '0');
    signal c_k1   : unsigned(17 downto 0) := to_unsigned(39946, 18);
    signal c_k2   : unsigned(17 downto 0) := to_unsigned(55188, 18);
    signal c_p2   : unsigned(9 downto 0) := to_unsigned(304, 10);
    signal c_F1   : unsigned(11 downto 0) := to_unsigned(936, 12);
    signal c_a1   : unsigned(14 downto 0) := to_unsigned(10005, 15);
    signal c_a2   : unsigned(14 downto 0) := to_unsigned(3673, 15);
    signal c_v1   : unsigned(15 downto 0) := (others => '0');   -- |speed|, Q20 cycles per field
    signal c_vn   : std_logic := '0';
    signal c_phi1 : unsigned(19 downto 0) := to_unsigned(C_PHI1_0, 20);
    signal c_phi2 : unsigned(19 downto 0) := to_unsigned(C_PHI2_0, 20);
    signal c_gph  : unsigned(17 downto 0) := (others => '0');    -- glint phase, Q16
    signal c_gso  : signed(8 downto 0) := (others => '0');      -- glint size offset (px)
    signal c_g6h  : unsigned(9 downto 0) := to_unsigned(80, 10); -- camera bend gain
    signal c_xp1  : unsigned(9 downto 0) := to_unsigned(290, 10); -- tube ramp origin + first pixel (virtual px)
    signal c_xp2  : unsigned(9 downto 0) := to_unsigned(146, 10);
    signal c_nc   : unsigned(10 downto 0) := to_unsigned(960, 11); -- the centre pixel (Mirror)
    signal c_zst  : unsigned(11 downto 0) := to_unsigned(1236, 12); -- glint zigzag step per pixel (Q16)
    signal c_zx0  : unsigned(15 downto 0) := (others => '0');
    signal c_gpz  : unsigned(15 downto 0) := (others => '0');
    signal c_tk   : unsigned(7 downto 0) := to_unsigned(11, 8);   -- tick * 29 + 11

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(8 downto 0) := to_unsigned(511, 9);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_yv   : unsigned(10 downto 0) := (others => '0');
    signal e_th1, e_th2 : unsigned(19 downto 0) := (others => '0');
    signal e_sg   : std_logic := '0';
    signal e_cs   : unsigned(15 downto 0) := (others => '0');
    signal e_w1, e_w2 : unsigned(14 downto 0) := (others => '0');
    signal e_aw   : unsigned(15 downto 0) := (others => '0');
    signal e_g1, e_g2 : unsigned(17 downto 0) := (others => '0');
    signal e_dys, e_dq : signed(11 downto 0) := (others => '0');
    signal e_ady  : unsigned(10 downto 0) := (others => '0');
    signal e_dyp  : unsigned(10 downto 0) := (others => '0');
    signal e_zzy  : unsigned(4 downto 0) := (others => '0');
    signal e_row  : std_logic := '0';
    signal m_a    : unsigned(21 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(38 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal e_ms   : unsigned(2 downto 0) := (others => '0');
    signal e_ld   : std_logic := '0';
    signal n_p1, n_p2 : unsigned(28 downto 0) := (others => '0');
    signal n_st1, n_st2 : unsigned(18 downto 0) := (others => '0');
    signal n_ls   : signed(11 downto 0) := (others => '0');
    signal n_dys  : signed(12 downto 0) := (others => '0');
    signal n_cr   : unsigned(7 downto 0) := (others => '0');

    -- beam and the line's values
    signal b_ph1, b_ph2 : unsigned(28 downto 0) := (others => '0');
    signal b_zx  : unsigned(15 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_st1, l_st2 : unsigned(18 downto 0) := (others => '0');
    signal l_ls  : signed(11 downto 0) := (others => '0');
    signal l_dys : signed(12 downto 0) := (others => '0');
    signal l_cr  : unsigned(7 downto 0) := (others => '0');

    -- pixel path
    type t_ph is array (2 to 6) of unsigned(28 downto 0);
    signal q_ph1, q_ph2 : t_ph := (others => (others => '0'));
    signal p2_zx, p3_zx, p4_zx : unsigned(15 downto 0) := (others => '0');
    signal p4_cm  : unsigned(7 downto 0) := (others => '0');
    signal p5_c8  : signed(11 downto 0) := (others => '0');
    signal p5_zt  : unsigned(10 downto 0) := (others => '0');
    signal p6_bp  : signed(22 downto 0) := (others => '0');
    signal p6_zz, p7_zz : unsigned(3 downto 0) := (others => '0');
    signal p7_q1, p7_q2 : unsigned(28 downto 0) := (others => '0');
    signal p7_cg  : unsigned(8 downto 0) := (others => '0');
    signal p8_df  : signed(8 downto 0) := (others => '0');
    signal p8_b   : unsigned(4 downto 0) := (others => '0');
    signal p8_b9  : unsigned(7 downto 0) := (others => '0');
    signal p8_t   : signed(12 downto 0) := (others => '0');
    signal p9_dxp : signed(19 downto 0) := (others => '0');
    signal p9_t, p10_t, p11_t : signed(12 downto 0) := (others => '0');
    signal p10_dx, p11_dx : signed(11 downto 0) := (others => '0');
    signal p10_tw : std_logic_vector(31 downto 0) := (others => '0');
    signal p10_cw : std_logic_vector(23 downto 0) := (others => '0');
    signal p11_sb : std_logic_vector(15 downto 0) := (others => '0');
    type t_c8 is array (11 to 17) of unsigned(7 downto 0);
    signal q_y, q_u, q_v : t_c8 := (others => (others => '0'));
    signal p12_dx : signed(12 downto 0) := (others => '0');
    signal p12_k, p13_k : signed(12 downto 0) := (others => '0');
    signal p13_ad : unsigned(12 downto 0) := (others => '0');
    signal p14_G  : signed(13 downto 0) := (others => '0');
    signal p14_yt, p15_yt : unsigned(7 downto 0) := (others => '0');
    signal p15_r  : unsigned(3 downto 0) := (others => '0');
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

    type t_i9 is array (0 to 8) of integer;
    constant C_RAY : t_i9 := (C_RA0_Y, C_RA1_Y, C_RA2_Y, C_RA3_Y, C_RA4_Y, C_RA5_Y, C_RA6_Y, C_RA7_Y, C_RA8_Y);
    constant C_RAU : t_i9 := (C_RA0_U, C_RA1_U, C_RA2_U, C_RA3_U, C_RA4_U, C_RA5_U, C_RA6_U, C_RA7_U, C_RA8_U);
    constant C_RAV : t_i9 := (C_RA0_V, C_RA1_V, C_RA2_V, C_RA3_V, C_RA4_V, C_RA5_V, C_RA6_V, C_RA7_V, C_RA8_V);
    constant C_RLY : t_i9 := (C_RL0_Y, C_RL1_Y, C_RL2_Y, C_RL3_Y, C_RL4_Y, C_RL5_Y, C_RL6_Y, C_RL7_Y, C_RL8_Y);
    constant C_RLU : t_i9 := (C_RL0_U, C_RL1_U, C_RL2_U, C_RL3_U, C_RL4_U, C_RL5_U, C_RL6_U, C_RL7_U, C_RL8_U);
    constant C_RLV : t_i9 := (C_RL0_V, C_RL1_V, C_RL2_V, C_RL3_V, C_RL4_V, C_RL5_V, C_RL6_V, C_RL7_V, C_RL8_V);

begin

    ---------------------------------------------------------------------------
    -- Memories: line buffer (collisions possible: read-before-write kept),
    -- ROMs, the box rings, the sync ring
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    -- the sequencer owns the shared exp2 / sine ROM in the first half of the vblank zone, the line engine
    -- the rest of the time (its line-0 kick is at zc 1300)
    r_sq_a <= z_sqa when (s_zone = '1' and s_zc(10) = '0') else e_sqa;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_t_q  <= C_HW_TUBE(to_integer(r_t_a));
            r_c_q  <= C_HW_CORD(to_integer(r_c_a));
            r_sq_q <= C_HW_SEQ(to_integer(r_sq_a));
            r_sb_q <= C_HW_SBOX(to_integer(r_sb_a));
        end if;
    end process p_rom;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y19) & std_logic_vector(s_u19(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u19(3 downto 0)) & std_logic_vector(s_v19) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y21) & std_logic_vector(s_u21(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u21(3 downto 0)) & std_logic_vector(s_v21) & "00";
            syr_q <= syr(to_integer(sy_ra));
            syr(to_integer(rg_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_e  : signed(13 downto 0);
        variable v_q  : unsigned(19 downto 0);
        variable v_fs : unsigned(18 downto 0);
        variable v_p  : unsigned(31 downto 0);
        variable v_a  : unsigned(14 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_braid  <= registers_in(6)(0);
                s_lemon  <= registers_in(6)(1);
                s_mirror <= registers_in(6)(2);
                s_wire   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, seven channels
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

            -- serial multiplier, right-shifting accumulator: f_a (17 bits) x f_b (16 bits), 16 iterations; the
            -- product is complete 17 clocks after the load.  Loads use a job code registered one clock ahead
            -- (z_fms / z_fld), so no zone-counter decode sits in front of the operand mux.
            if f_cnt /= 0 then
                if f_b(0) = '1' then v_fs := ('0' & f_acc(33 downto 16)) + resize(f_a, 19);
                else v_fs := '0' & f_acc(33 downto 16); end if;
                f_acc <= v_fs & f_acc(15 downto 1);
                f_b <= '0' & f_b(15 downto 1);
                f_cnt <= f_cnt - 1;
            end if;
            if z_fld = '1' then
                f_acc <= (others => '0'); f_cnt <= to_unsigned(16, 5);
                case to_integer(z_fms) is
                    when 0      => f_a <= resize(unsigned(r_sq_q), 17); f_b <= to_unsigned(39946, 16);
                    when 1      => f_a <= resize(unsigned(r_sq_q), 17); f_b <= to_unsigned(55188, 16);
                    when 2      => f_a <= resize(unsigned(r_sq_q), 17); f_b <= to_unsigned(304, 16);
                    when 3      => f_a <= resize(unsigned(r_sq_q), 17); f_b <= to_unsigned(936, 16);
                    when 4      => f_a <= resize(z_d, 17);              f_b <= resize(z_d, 16);
                    when 5      => f_a <= to_unsigned(45, 17);          f_b <= z_dsq;
                    when 6      => f_a <= resize(z_k6, 17);             f_b <= resize(z_k6, 16);
                    when others => f_a <= to_unsigned(2298, 17);        f_b <= z_ksq;
                end case;
            end if;
            z_fld <= '0';

            -- per-field values: zc 20 .. 200 (one multiplier job at a time)
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20 =>                                        -- motion: last field's speed
                        if c_vn = '1' then
                            c_phi1 <= c_phi1 - resize(c_v1, 20);
                            c_phi2 <= c_phi2 - resize(c_v1(15 downto 4), 20);
                        else
                            c_phi1 <= c_phi1 + resize(c_v1, 20);
                            c_phi2 <= c_phi2 + resize(c_v1(15 downto 4), 20);
                        end if;
                        c_gph <= c_gph + resize(c_v1(15 downto 2), 18);
                    when 21 =>
                        c_gpz <= shift_right(c_gph(15 downto 0), 1) + shift_right(c_gph(15 downto 0), 3);
                        c_zx0 <= shift_right(c_gph(15 downto 0), 2) + shift_right(c_gph(15 downto 0), 3);
                        -- tick * 29 + 11 (the glint cells re-roll every 2^15 of the glint phase)
                        c_tk <= resize(shift_left(resize(c_gph(17 downto 15), 8), 5) - shift_left(resize(c_gph(17 downto 15), 8), 1)
                                - resize(c_gph(17 downto 15), 8) + 11, 8);
                    when 22 =>                                        -- resolution constants
                        case to_integer(s_xs) is
                            when 4      => c_xp1 <= to_unsigned(530, 10); c_xp2 <= to_unsigned(386, 10);
                                           c_nc <= to_unsigned(360, 11); c_zst <= to_unsigned(2472, 12);
                            when 3      => c_xp1 <= to_unsigned(290, 10); c_xp2 <= to_unsigned(146, 10);
                                           c_nc <= to_unsigned(640, 11); c_zst <= to_unsigned(1854, 12);
                            when others => c_xp1 <= to_unsigned(290, 10); c_xp2 <= to_unsigned(146, 10);
                                           c_nc <= to_unsigned(960, 11); c_zst <= to_unsigned(1236, 12);
                        end case;
                    when 29 =>                                        -- tubes: e1 = 7/8 (k1 - 512) Q8 octaves
                        v_d := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        v_e := resize(shift_right(shift_left(resize(v_d, 14), 3) - resize(v_d, 14), 3), 14);
                        z_sqa <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                        -- cords: e3 = 7/8 (k3 - 512)
                        v_d := signed(resize(s_g3(12 downto 3), 12)) - 512;
                        v_e := resize(shift_right(shift_left(resize(v_d, 14), 3) - resize(v_d, 14), 3), 14);
                        z_e3 <= v_e(11 downto 0);
                    when 31 => z_fms <= to_unsigned(0, 3); z_fld <= '1';      -- job 0 at 32: 39946 mant
                    when 47 =>
                        z_sqa <= '0' & unsigned(z_e3(7 downto 0));
                    when 49 =>
                        v_p := f_acc(31 downto 0);
                        case to_integer(z_sh) is
                            when -2     => c_k1 <= resize(v_p(31 downto 17), 18);
                            when -1     => c_k1 <= resize(v_p(31 downto 16), 18);
                            when 0      => c_k1 <= resize(v_p(31 downto 15), 18);
                            when others => c_k1 <= v_p(31 downto 14);
                        end case;
                        z_sh <= z_e3(10 downto 8);
                        z_fms <= to_unsigned(1, 3); z_fld <= '1';             -- job 1 at 50: 55188 mant(e3)
                    when 65 =>
                        v_d := -z_e3;
                        z_sqa <= '0' & unsigned(v_d(7 downto 0));
                    when 67 =>
                        v_p := f_acc(31 downto 0);
                        case to_integer(z_sh) is
                            when -2     => c_k2 <= resize(v_p(31 downto 17), 18);
                            when -1     => c_k2 <= resize(v_p(31 downto 16), 18);
                            when 0      => c_k2 <= resize(v_p(31 downto 15), 18);
                            when others => c_k2 <= v_p(31 downto 14);
                        end case;
                        v_d := -z_e3;
                        z_sh <= v_d(10 downto 8);
                        z_fms <= to_unsigned(2, 3); z_fld <= '1';             -- job 2 at 68: 304 mant(-e3)
                    when 83 =>                                        -- wave: e2 = k2 - 512
                        v_d := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        z_sqa <= '0' & unsigned(v_d(7 downto 0));
                    when 85 =>
                        v_p := f_acc(31 downto 0);
                        case to_integer(z_sh) is
                            when -2     => c_p2 <= resize(v_p(31 downto 17), 10);
                            when -1     => c_p2 <= resize(v_p(31 downto 16), 10);
                            when 0      => c_p2 <= resize(v_p(31 downto 15), 10);
                            when others => c_p2 <= resize(v_p(31 downto 14), 10);
                        end case;
                        v_d := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        z_sh <= v_d(10 downto 8);
                        z_fms <= to_unsigned(3, 3); z_fld <= '1';             -- job 3 at 86: 936 mant(e2)
                    when 86 =>                                        -- speed: |k4 - 520| or |k4 - 504|, +-8 deadband
                        v_d := signed(resize(s_g4(12 downto 3), 12));
                        -- (slices, not resize: a signed resize to 9 bits would fold |d| >= 256)
                        if v_d > 520 then v_d := v_d - 520; z_d <= unsigned(v_d(8 downto 0)); z_dn <= '0';
                        elsif v_d < 504 then v_d := 504 - v_d; z_d <= unsigned(v_d(8 downto 0)); z_dn <= '1';
                        else z_d <= (others => '0'); z_dn <= '0'; end if;
                    when 103 =>
                        v_p := f_acc(31 downto 0);
                        case to_integer(z_sh) is
                            when -2     => c_F1 <= resize(v_p(31 downto 17), 12);
                            when -1     => c_F1 <= resize(v_p(31 downto 16), 12);
                            when 0      => c_F1 <= resize(v_p(31 downto 15), 12);
                            when others => c_F1 <= resize(v_p(31 downto 14), 12);
                        end case;
                        z_fms <= to_unsigned(4, 3); z_fld <= '1';             -- job 4 at 104: d^2
                    when 121 =>
                        z_dsq <= f_acc(17 downto 2);
                        z_fms <= to_unsigned(5, 3); z_fld <= '1';             -- job 5 at 122: (d^2 / 4) 45
                    when 122 =>
                        z_k6 <= s_g6(12 downto 3);
                    when 139 =>                                       -- v1 = d^2 45 / 256 (Q20 cycles per field)
                        c_v1 <= f_acc(21 downto 6);
                        c_vn <= z_dn;
                        z_fms <= to_unsigned(6, 3); z_fld <= '1';             -- job 6 at 140: k6^2
                    when 157 =>
                        z_ksq <= f_acc(19 downto 4);
                        z_fms <= to_unsigned(7, 3); z_fld <= '1';             -- job 7 at 158: (k6^2 / 16) 2298
                    when 160 =>                                       -- twist a1 = 23 p12 (Q16), cords a2
                        z_pp <= s_gp(12 downto 3);
                    when 161 =>
                        c_a1 <= resize(shift_left(resize(z_pp, 15), 4) + shift_left(resize(z_pp, 15), 2)
                                + shift_left(resize(z_pp, 15), 1) + resize(z_pp, 15), 15);
                    when 162 =>
                        v_a := c_a1;
                        if s_braid = '1' then
                            c_a2 <= v_a - shift_right(v_a, 3) + shift_right(v_a, 5);
                        else
                            c_a2 <= shift_right(v_a, 2) + shift_right(v_a, 3) - shift_right(v_a, 7);
                        end if;
                        c_gso <= resize(shift_right(signed(resize(s_g5(12 downto 3), 12)) - 512, 2), 9);
                    when 175 =>
                        c_g6h <= f_acc(27 downto 18);
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
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
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
    -- 0), computes the NEXT line: the two LFO cosines (w1, w2), the FM line
    -- steps g1 = k1 (1 - a1 w1), g2 = k2 (1 + a2 w2) and line starts, the glint
    -- row terms.  One serial product at a time (issued at N, read at N + 18).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ms : unsigned(23 downto 0);
        variable v_u  : unsigned(19 downto 0);
        variable v_c  : signed(16 downto 0);
        variable v_w  : signed(17 downto 0);
        variable v_z  : unsigned(15 downto 0);
        variable v_t  : unsigned(10 downto 0);
        variable v_l  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            -- serial multiplier, right-shifting accumulator: m_a (22 bits) x m_b (16 bits); complete 17 clocks after the load
            if m_cnt /= 0 then
                if m_b(0) = '1' then v_ms := ('0' & m_acc(38 downto 16)) + resize(m_a, 24);
                else v_ms := '0' & m_acc(38 downto 16); end if;
                m_acc <= v_ms & m_acc(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
                m_cnt <= m_cnt - 1;
            end if;
            if e_ld = '1' then
                m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                case to_integer(e_ms) is
                    when 0      => m_a <= resize(c_F1, 22);  m_b <= resize(e_yv, 16);
                    when 1      => m_a <= resize(e_w1, 22);  m_b <= resize(c_a1, 16);
                    when 2      => m_a <= resize(c_k1, 22);  m_b <= e_aw;
                    when 3      => m_a <= resize(e_w2, 22);  m_b <= resize(c_a2, 16);
                    when 4      => m_a <= resize(c_k2, 22);  m_b <= e_aw;
                    when 5      => m_a <= resize(e_g1, 22);  m_b <= resize(c_xp1, 16);
                    when 6      => m_a <= resize(e_g2, 22);  m_b <= resize(c_xp2, 16);
                    when others => m_a <= to_unsigned(1057, 22); m_b <= resize(e_yv, 16);
                end case;
            end if;
            e_ld <= '0';

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then e_yv <= to_unsigned(1079, 11); else e_yv <= e_vy(18 downto 8); end if;
                when 2 =>                                             -- job 0: y F1
                    e_ms <= to_unsigned(0, 3); e_ld <= '1';
                    if e_yv > 480 then e_row <= '1'; e_dys <= signed(resize(e_yv, 12)) - 620;
                    else e_row <= '0'; e_dys <= signed(resize(e_yv, 12)) - 310; end if;
                when 3 =>
                    if e_dys < 0 then e_ady <= unsigned(resize(-e_dys, 11)); else e_ady <= unsigned(e_dys(10 downto 0)); end if;
                    e_dq <= shift_right(e_dys, 2);
                when 4 =>                                             -- 0.586 |dy|
                    e_dyp <= shift_right(e_ady, 1) + shift_right(e_ady, 4) + shift_right(e_ady, 6) + shift_right(e_ady, 7);
                    n_dys <= resize(e_dys, 13) + resize(e_dq, 13);           -- 1.25 dy (the slant's scale)
                    if e_row = '1' then n_cr <= c_tk + 151; else n_cr <= c_tk; end if;
                when 20 =>                                            -- the LFO phases
                    e_th1 <= m_acc(19 downto 0) + c_phi1;
                    e_th2 <= (m_acc(18 downto 0) & '0') + c_phi2;
                when 21 =>                                            -- cos th1 = sin(th1 + 1/4): quarter-sine address
                    v_u := e_th1 + to_unsigned(262144, 20);
                    if v_u(18) = '1' then e_sqa <= '1' & not v_u(17 downto 10); else e_sqa <= '1' & v_u(17 downto 10); end if;
                    e_sg <= v_u(19);
                    if s_braid = '1' then e_th2 <= e_th1; end if;
                when 22 =>
                    v_u := e_th2 + to_unsigned(262144, 20);
                    if v_u(18) = '1' then e_sqa <= '1' & not v_u(17 downto 10); else e_sqa <= '1' & v_u(17 downto 10); end if;
                when 23 =>
                    e_cs <= unsigned(r_sq_q);                         -- fabric copy of the ROM word (cos th1 magnitude)
                    v_u := e_th2 + to_unsigned(262144, 20);
                    e_th2(19) <= v_u(19);                             -- keep th2's cos sign in its top bit
                when 24 =>                                            -- w1 = (32767 - cos) / 2
                    if e_sg = '1' then v_c := signed(resize(e_cs, 17)); else v_c := -signed(resize(e_cs, 17)); end if;
                    v_w := to_signed(32767, 18) + resize(v_c, 18);
                    e_w1 <= unsigned(v_w(15 downto 1));
                    e_cs <= unsigned(r_sq_q);
                when 25 =>                                            -- w2; job 1: a1 w1
                    if e_th2(19) = '1' then v_c := signed(resize(e_cs, 17)); else v_c := -signed(resize(e_cs, 17)); end if;
                    v_w := to_signed(32767, 18) + resize(v_c, 18);
                    e_w2 <= unsigned(v_w(15 downto 1));
                    e_ms <= to_unsigned(1, 3); e_ld <= '1';
                when 43 =>                                            -- aw1 (Q16); job 2: k1 aw1
                    e_aw <= m_acc(30 downto 15);
                    e_ms <= to_unsigned(2, 3); e_ld <= '1';
                when 60 =>                                            -- job 3: a2 w2 (job 2 is read at 61)
                    e_ms <= to_unsigned(3, 3); e_ld <= '1';
                when 61 =>                                            -- g1 = k1 - k1 aw1
                    e_g1 <= c_k1 - m_acc(33 downto 16);
                when 62 =>
                    case to_integer(s_xs) is
                        when 4      => n_st1 <= e_g1 & '0';
                        when 3      => n_st1 <= resize(e_g1, 19) + resize(e_g1(17 downto 1), 19);
                        when others => n_st1 <= resize(e_g1, 19);
                    end case;
                when 78 =>                                            -- aw2; job 4: k2 aw2
                    e_aw <= m_acc(30 downto 15);
                    e_ms <= to_unsigned(4, 3); e_ld <= '1';
                when 95 =>                                            -- job 5: (x0 + 290) g1 (job 4 is read at 96)
                    e_ms <= to_unsigned(5, 3); e_ld <= '1';
                when 96 =>                                            -- g2 = k2 + k2 aw2
                    e_g2 <= c_k2 + m_acc(33 downto 16);
                when 97 =>
                    case to_integer(s_xs) is
                        when 4      => n_st2 <= e_g2 & '0';
                        when 3      => n_st2 <= resize(e_g2, 19) + resize(e_g2(17 downto 1), 19);
                        when others => n_st2 <= resize(e_g2, 19);
                    end case;
                when 112 =>                                           -- job 6: (x0 + 146) g2 (job 5 is read at 113)
                    e_ms <= to_unsigned(6, 3); e_ld <= '1';
                when 113 =>
                    n_p1 <= m_acc(28 downto 0);
                when 129 =>                                           -- job 7: y 1057 (glint zigzag)
                    e_ms <= to_unsigned(7, 3); e_ld <= '1';
                when 130 =>
                    n_p2 <= m_acc(28 downto 0);
                when 147 =>
                    v_z := m_acc(15 downto 0) + c_gpz;
                    v_t := f_tri11(v_z);
                    e_zzy <= resize(shift_right(resize(v_t, 13) + shift_left(resize(v_t, 13), 1), 8), 5);
                when 148 =>                                           -- the line's glint sum: 0.586 |dy| + zigzag - size offset
                    v_l := signed(resize(e_dyp, 13)) + signed(resize(e_zzy, 13)) - resize(c_gso, 13);
                    n_ls <= v_l(11 downto 0);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): the two phase DDAs and the glint zigzag DDA (reversing at
    -- the centre pixel under Mirror); the line's values latched at the line start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ph1 <= n_p1;
                b_ph2 <= n_p2;
                b_zx  <= c_zx0;
                b_px  <= (others => '0');
                l_st1 <= n_st1; l_st2 <= n_st2;
                l_ls  <= n_ls;  l_dys <= n_dys; l_cr <= n_cr;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if s_mirror = '1' and b_px >= c_nc then
                    b_ph1 <= b_ph1 - resize(l_st1, 29);
                    b_ph2 <= b_ph2 - resize(l_st2, 29);
                    b_zx  <= b_zx - resize(c_zst, 16);
                else
                    b_ph1 <= b_ph1 + resize(l_st1, 29);
                    b_ph2 <= b_ph2 + resize(l_st2, 29);
                    b_zx  <= b_zx + resize(c_zst, 16);
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_c   : signed(11 downto 0);
        variable v_bp  : signed(28 downto 0);
        variable v_cg  : unsigned(13 downto 0);
        variable v_t   : signed(12 downto 0);
        variable v_dx  : signed(12 downto 0);
        variable v_sl  : signed(12 downto 0);
        variable v_key : std_logic;
        variable v_y   : unsigned(7 downto 0);
        variable v_r   : integer range 0 to 9;
        variable v_g   : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 2..6: the phases ride along until the camera bend is ready
            q_ph1(2) <= b_ph1; q_ph2(2) <= b_ph2;
            for i in 3 to 6 loop q_ph1(i) <= q_ph1(i - 1); q_ph2(i) <= q_ph2(i - 1); end loop;
            p2_zx <= b_zx; p3_zx <= p2_zx; p4_zx <= p3_zx;
            -- 4: the camera / Osc sample (EBR -> fabric)
            p4_cm <= unsigned(s_lb_q);
            -- 5: signed camera term (Osc: the lattice, zero-mean and x8); the zigzag triangle
            if s_src = '1' then v_c := shift_left(signed(resize(p4_cm, 12)) - 120, 3);
            else v_c := signed(resize(p4_cm, 12)); end if;
            p5_c8 <= v_c;
            p5_zt <= f_tri11(p4_zx);
            -- 6: the camera product (12 x 10 bits); zigzag x = tri 3 / 512
            p6_bp <= p5_c8 * signed('0' & c_g6h);
            p6_zz <= resize(shift_right(resize(p5_zt, 13) + shift_left(resize(p5_zt, 13), 1), 9), 4);
            -- 7: the bend (x 64, Q24 periods) on both phases; the camera's glint term (positive part)
            v_bp := shift_left(resize(p6_bp, 29), 6);
            p7_q1 <= q_ph1(6) + unsigned(v_bp);
            p7_q2 <= q_ph2(6) + unsigned(v_bp);
            if p6_bp > 0 then
                if s_src = '1' then v_cg := resize(unsigned(p6_bp(22 downto 11)), 14); else v_cg := unsigned(p6_bp(22 downto 9)); end if;
                if v_cg > 511 then p7_cg <= (others => '1'); else p7_cg <= v_cg(8 downto 0); end if;
            else
                p7_cg <= (others => '0');
            end if;
            p7_zz <= p6_zz;
            -- 8: palette addresses; the glint cell's x offset; bundle index x 9; zigzag + line sum - camera
            r_t_a <= s_lemon & p7_q1(23 downto 16);
            r_c_a <= s_braid & s_lemon & p7_q2(23 downto 16);
            p8_df <= signed('0' & p7_q2(23 downto 16)) - 143;
            p8_b  <= p7_q2(28 downto 24);
            p8_b9 <= shift_left(resize(p7_q2(28 downto 24), 8), 3) + resize(p7_q2(28 downto 24), 8);
            p8_t  <= resize(signed('0' & p7_zz), 13) + resize(l_ls, 13) - signed(resize(p7_cg, 13));
            -- 9: the cell ROM address (bundle 73 + row 151 + tick 29 + 11); dx = df P2 / 256 (px)
            r_sb_a <= shift_left(p8_b9, 3) + resize(p8_b, 8) + l_cr;
            p9_dxp <= p8_df * signed('0' & c_p2);
            p9_t <= p8_t;
            -- 10: palette words (EBR registers in stage 9) -> fabric; dx (px)
            p10_tw <= r_t_q; p10_cw <= r_c_q;
            p10_dx <= p9_dxp(19 downto 8);
            p10_t <= p9_t;
            -- 11: the cell word (EBR register in stage 10) -> fabric; tube (keyed; Wire hollows its core) over the cords
            p11_sb <= r_sb_q;
            v_key := p10_tw(7) and not (s_wire and p10_tw(6));
            if v_key = '1' then
                q_y(11) <= unsigned(p10_tw(31 downto 24)); q_u(11) <= unsigned(p10_tw(23 downto 16)); q_v(11) <= unsigned(p10_tw(15 downto 8));
            else
                q_y(11) <= unsigned(p10_cw(23 downto 16)); q_u(11) <= unsigned(p10_cw(15 downto 8)); q_v(11) <= unsigned(p10_cw(7 downto 0));
            end if;
            p11_dx <= p10_dx; p11_t <= p10_t;
            for i in 12 to 15 loop q_y(i) <= q_y(i - 1); q_u(i) <= q_u(i - 1); q_v(i) <= q_v(i - 1); end loop;
            -- 12: slant (the cell's 2 bits x the line's dy); size - (zigzag + line sum - camera)
            case p11_sb(7 downto 6) is
                when "00"   => v_sl := -shift_right(l_dys, 2);
                when "01"   => v_sl := -shift_right(l_dys, 3);
                when "10"   => v_sl := shift_right(l_dys, 3);
                when others => v_sl := shift_right(l_dys, 2);
            end case;
            p12_dx <= resize(p11_dx, 13) + v_sl;
            p12_k  <= signed(resize(unsigned(p11_sb(15 downto 8)), 13)) - p11_t;
            -- 13: |dx|
            if p12_dx < 0 then p13_ad <= unsigned(-p12_dx); else p13_ad <= unsigned(p12_dx); end if;
            p13_k <= p12_k;
            -- 14: G = depth inside the glint; the tinted luma (strand Y x 13/16, clamped)
            p14_G <= resize(p13_k, 14) - signed(resize(p13_ad, 14));
            v_y := q_y(13) - shift_right(q_y(13), 3) - shift_right(q_y(13), 4);
            if v_y < 43 then p14_yt <= to_unsigned(43, 8); elsif v_y > 205 then p14_yt <= to_unsigned(205, 8); else p14_yt <= v_y; end if;
            -- 15: the ring (0 = no glint)
            v_g := p14_G;
            if v_g <= 0 then v_r := 0;
            elsif v_g <= 14 then v_r := 1;
            elsif v_g <= 20 then v_r := 2;
            elsif v_g <= 22 then v_r := 3;
            elsif v_g <= 24 then v_r := 4;
            elsif v_g <= 26 then v_r := 5;
            elsif v_g <= 48 then v_r := 6;
            elsif v_g <= 60 then v_r := 7;
            elsif v_g <= 72 then v_r := 8;
            else v_r := 9; end if;
            p15_r <= to_unsigned(v_r, 4);
            p15_yt <= p14_yt;
            -- 16: glint colour over the strands (rings 7 and 9 keep the strand's luma, tinted red)
            if p15_r = 0 then
                q_y(16) <= q_y(15); q_u(16) <= q_u(15); q_v(16) <= q_v(15);
            else
                v_r := to_integer(p15_r) - 1;
                if p15_r = 7 or p15_r = 9 then q_y(16) <= p15_yt;
                elsif s_lemon = '1' then q_y(16) <= to_unsigned(C_RLY(v_r), 8);
                else q_y(16) <= to_unsigned(C_RAY(v_r), 8); end if;
                if s_lemon = '1' then
                    q_u(16) <= to_unsigned(C_RLU(v_r), 8); q_v(16) <= to_unsigned(C_RLV(v_r), 8);
                else
                    q_u(16) <= to_unsigned(C_RAU(v_r), 8); q_v(16) <= to_unsigned(C_RAV(v_r), 8);
                end if;
            end if;
            -- 17: pad (keeps the shell's stage count)
            q_y(17) <= q_y(16); q_u(17) <= q_u(16); q_v(17) <= q_v(16);
            -- 18: 10 bits
            p18_y <= q_y(17) & "00"; p18_u <= q_u(17) & "00"; p18_v <= q_v(17) & "00";
            -- 19: neutral outside the generated avid
            if s_gav_sr(16) = '0' then
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            else
                s_y19 <= p18_y; s_u19 <= p18_u; s_v19 <= p18_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the 8-back taps come from block-RAM rings:
    -- the still's analog softness, and the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate; the syncs ride a block-RAM ring of the same latency
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- ring addresses: one write per clock; the read, issued two clocks ahead and copied once,
            -- is the sample written 8 clocks before the one being added
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 5;
            sy_ra <= rg_wa - 21;
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y19 - s_cay;
                s_bau <= s_bau + s_u19 - s_cau;
                s_bav <= s_bav + s_v19 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y21 - s_cby;
                s_bbu <= s_bbu + s_u21 - s_cbu;
                s_bbv <= s_bbv + s_v21 - s_cbv;
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
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture hawser;
