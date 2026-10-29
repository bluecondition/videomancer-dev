-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Archway v0.6.0 -- Heirloom #36, rebuilt from the photo WIN_20230523_01_26_43 (phase 2: v0.6).
--
-- The photo: a pointed arch (one parabola family, centre x 1013) drawn three ways at once.  Its flanks
-- are a staircase mosaic of 52 px cells, one colour per cell from a stepped colouriser (yellow, olive,
-- hot pink, maroon, blue, navy, black going outward), cut by four smooth black arcs; its middle is a fan
-- of cyan arch lines (thin over the mosaic on the left half, fat cyan with navy gaps on the right half,
-- green where they cross the centre); a pink chevron at the bottom centre (one smooth edge and one
-- stepped edge per leg, pale stepped cells beside it) holds nested cyan lines.  Thin streaks of other
-- colours ring through the first part of the cells next to the chevron and under the lines.
--
-- The patch, as fitted (archway.py has every constant):
--   F    = |x' - 1013| - s(y) + etch * luma      the field (x' soft-held near the line start)
--   s(y) = Arch * 3.87e-4 * (y + 16.5)^2 - 2.6   the vertical ramp through a squarer
--   Fs   = F sampled and held on a cell x cell grid in (x, s) (right-hand rows a quarter row late)
--   n    = floor((Fs - 20) / 52)                 the colouriser step -> one palette entry per cell
--   arcs = F in {282, 380, 486, 600} +- 7.5       lines = two summed triangles of F over 3 thresholds
--   chevron: P = F in [-218, -60] AND Fs in [-188, -32);  W = the S&H band alone
--   the colour window and the right half's black gap sit on the field 46 px behind (the colour path delay)
--
--   K1  Steps     mosaic cell size (5.5 .. 194 px; 52 = the photo)
--   K2  Lines     the cyan line fan's density
--   K3  Etch      how far the picture (or the Osc) bends every contour and the mosaic (cube law, to ~750 px)
--   K4  Speed     bipolar, still at the centre: left = the lines stream into the arch fast (square law) while
--                 the S&H grid creeps (linear); right = the grid races (cube law) while the lines drift out
--                 slowly (linear); both sides meet the v0.5 motion at 640
--   K5  Chevron   0 = gone (below the screen at any Arch), up through the photo (381) to a pink band on
--                 the outermost arc with nested lines filling the arch (1023)
--   K6  Palette   a continuous morph through Archway (the photo) / Lagoon / Orchard / Jade / Ultra / Citrus /
--                 Ember / Frost in 1/32 steps (each pixel shows A, B or their midpoint in an 8-pixel pattern,
--                 alternated on line pairs; the output box's 8-sums turn that into an exact flat mix)
--   S7  Split     On (the photo: the two halves differ) / Off (both halves like the left)
--   S8  Arcs      the black arcs On / Off
--   S9  Flip      Off / On: the arch upside down (the generated field only; the camera's etch stays upright)
--   S10 Mosaic    Stepped (the photo) / Smooth (S&H bypassed: smooth arch bands)
--   S11 Source    Video / Osc
--   P12 ARCH      curvature: straight pillars (0) .. the photo (575) .. a squat flared dome (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, the S&H grid origin), a per-line
-- engine kicked at each line start (computes the NEXT line: s(y), the S&H rows, the ring index, the
-- chevron gates, the tear wobble), a pixel path with the S&H at stage 7, three region-flag ROMs (F, F - d,
-- Fs - d) instead of comparator banks, and the palette ROM {A, B} at stage 17 (latency 24 in every mode).
-- Virtual 1080-line coordinates (SD shows the centre 4:3).
-- Generated avid; output conditioning (Y, U and V through a [1 2 2 2 2 2 2 2 1]/16 box = the photo's
-- horizontal softness and the HD HDMI bar fix, luma 13/16) before the blanking gate.  Colours
-- in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.archway_pkg.all;
use work.all;

architecture archway of program_top is
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
    -- Q14 triangle of a 16-bit phase: -16384 at 0, +16384 at half
    function f_tri14(u : unsigned(15 downto 0)) return signed is
        variable d : unsigned(15 downto 0);
    begin
        if u(15) = '1' then d := u - 32768; else d := 32768 - u; end if;
        return to_signed(16384, 17) - signed(resize(d, 17));
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_pal_a : unsigned(8 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(63 downto 0) := (others => '0');   -- {A y u v, B y u v, 0000}
    -- region-flag ROMs of F, F - d (stage-7 address) and Fs - d (stage-8 address); fabric copies 2 stages later
    signal r_ffl_a, r_frl_a, r_fsl_a : unsigned(10 downto 0) := (others => '0');
    signal r_ffl_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_frl_q, r_fsl_q : std_logic_vector(3 downto 0) := (others => '0');
    signal p9_ffl  : std_logic_vector(7 downto 0) := (others => '0');
    signal p9_frl, p10_fsl : std_logic_vector(3 downto 0) := (others => '0');

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
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(381 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := (others => '0');
    signal s_gp : unsigned(12 downto 0) := to_unsigned(575 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_splitoff, s_arcsoff, s_flip, s_smooth, s_src : std_logic := '0';
    signal s_pal : unsigned(2 downto 0) := (others => '0');
    signal s_palf : unsigned(4 downto 0) := (others => '0');     -- the morph toward palette s_pal + 1, /32

    -- per-field constants
    signal z_neg  : std_logic := '0';
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_e8c  : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(3 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sq   : unsigned(19 downto 0) := (others => '0');          -- the sequencer's chained temporary
    signal z_m2   : unsigned(17 downto 0) := (others => '0');
    signal c_cell : unsigned(12 downto 0) := to_unsigned(1664, 13);    -- cell, Q4 h
    signal c_phx  : unsigned(10 downto 0) := to_unsigned(320, 11);     -- column phase, Q4 h
    signal c_phs  : unsigned(9 downto 0) := to_unsigned(175, 10);      -- row phase, Q4 h
    signal c_lag  : unsigned(10 downto 0) := to_unsigned(383, 11);     -- right-hand row lag, Q4 h
    signal c_rk   : unsigned(14 downto 0) := to_unsigned(2048, 15);    -- ring scale 52 / cell_h, Q12
    signal c_rlp  : unsigned(10 downto 0) := to_unsigned(885, 11);     -- line phase per h, Q16
    signal c_k    : unsigned(17 downto 0) := to_unsigned(51990, 18);   -- curvature
    signal c_ge   : unsigned(10 downto 0) := (others => '0');          -- etch gain per luma step, Q8 h
    signal c_dst  : unsigned(15 downto 0) := (others => '0');          -- line drift per field, Q16 periods
    signal c_cst  : unsigned(10 downto 0) := (others => '0');          -- S&H clock crawl per field, Q4 h (< a cell)
    signal c_bst  : unsigned(15 downto 0) := (others => '0');          -- chevron breathing step
    signal c_sta : unsigned(15 downto 0) := (others => '0');    -- tear wobble step per line
    signal c_ja0 : unsigned(15 downto 0) := (others => '0');
    signal c_drift : unsigned(23 downto 0) := (others => '0');
    signal c_crawl : unsigned(13 downto 0) := (others => '0');
    signal c_bph  : unsigned(15 downto 0) := (others => '0');
    signal c_jph  : unsigned(19 downto 0) := (others => '0');
    signal c_brt  : signed(16 downto 0) := (others => '0');
    signal c_br   : signed(5 downto 0) := (others => '0');
    signal c_d    : signed(13 downto 0) := (others => '0');
    signal c_x016 : unsigned(15 downto 0) := to_unsigned(256, 16);
    signal c_ce   : signed(17 downto 0) := (others => '0');
    signal c_ck   : signed(9 downto 0) := (others => '0');
    signal c_e0   : unsigned(15 downto 0) := (others => '0');
    signal c_k0   : signed(9 downto 0) := (others => '0');
    signal c_sq0, c_sqb0 : signed(17 downto 0) := (others => '0');
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(31 downto 0) := (others => '0');
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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_sq, e_sqb : signed(17 downto 0) := (others => '0');
    signal e_s16  : signed(17 downto 0) := (others => '0');
    signal e_ja : unsigned(15 downto 0) := (others => '0');
    signal e_ta : signed(16 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(17 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_sh   : signed(13 downto 0) := (others => '0');
    signal n_vh, n_vbh : unsigned(9 downto 0) := (others => '0');
    signal n_vn, n_vnb : unsigned(5 downto 0) := (others => '0');
    signal n_glam, n_gpale : std_logic := '0';
    signal n_jm   : signed(13 downto 0) := (others => '0');
    signal n_shd, n_jmd : signed(13 downto 0) := (others => '0');
    signal l_shd, l_jms : signed(13 downto 0) := (others => '0');
    signal l_sh   : signed(13 downto 0) := (others => '0');
    signal l_vh, l_vbh : unsigned(9 downto 0) := (others => '0');
    signal l_vn, l_vnb : unsigned(5 downto 0) := (others => '0');
    signal l_glam, l_gpale : std_logic := '0';

    -- beam
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_nxt : unsigned(17 downto 0) := (others => '0');
    signal b_k   : signed(9 downto 0) := (others => '0');
    signal b_first : std_logic := '0';

    -- pixel path
    signal p2_x  : unsigned(11 downto 0) := (others => '0');
    signal p2_ed, p2_lag : std_logic := '0';
    signal p3_dx : signed(12 downto 0) := (others => '0');
    signal p3_lm : unsigned(7 downto 0) := (others => '0');
    signal p3_ed, p3_lag : std_logic := '0';
    signal p4_adx : unsigned(11 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(14 downto 0) := (others => '0');
    signal p4_rt, p4_ed, p4_lag : std_logic := '0';
    signal p5_e   : unsigned(10 downto 0) := (others => '0');
    signal p5_as, p5_asj, p5_asr : signed(13 downto 0) := (others => '0');
    signal p5_rt, p5_gr, p5_ed, p5_lag : std_logic := '0';
    signal p6_f, p6_flm, p6_fr : signed(13 downto 0) := (others => '0');
    signal p6_rt, p6_gr, p6_ed, p6_lag : std_logic := '0';
    signal p7_fs, p7_fsr : signed(13 downto 0) := (others => '0');
    signal p7_vs  : unsigned(5 downto 0) := (others => '0');
    signal p7_pl  : signed(20 downto 0) := (others => '0');
    signal p7_ph  : signed(18 downto 0) := (others => '0');
    signal p7_fl  : std_logic_vector(12 downto 0) := (others => '0');
    signal p7_rt, p7_gr : std_logic := '0';
    signal p8_a80 : signed(18 downto 0) := (others => '0');
    signal p8_a1  : signed(18 downto 0) := (others => '0');
    signal p8_phi : signed(25 downto 0) := (others => '0');
    signal p8_vs  : unsigned(5 downto 0) := (others => '0');
    signal p8_fl  : std_logic_vector(12 downto 0) := (others => '0');
    signal p8_rt, p8_gr : std_logic := '0';
    signal p9_n   : signed(8 downto 0) := (others => '0');
    signal p9_phd : unsigned(23 downto 0) := (others => '0');
    signal p9_vs  : unsigned(5 downto 0) := (others => '0');
    signal p9_fl  : std_logic_vector(12 downto 0) := (others => '0');
    signal p9_rt, p9_gr : std_logic := '0';
    signal p10_t1 : signed(16 downto 0) := (others => '0');
    signal p10_u2 : unsigned(15 downto 0) := (others => '0');
    signal p10_ni : unsigned(3 downto 0) := (others => '0');
    signal p10_nok, p10_n6, p10_n5, p10_nm1, p10_nodd : std_logic := '0';
    signal p10_vs : unsigned(5 downto 0) := (others => '0');
    signal p10_fl : std_logic_vector(12 downto 0) := (others => '0');
    signal p10_rt, p10_gr : std_logic := '0';
    signal p11_t1, p11_t2 : signed(16 downto 0) := (others => '0');
    signal p12_t1 : signed(16 downto 0) := (others => '0');
    signal p12_t2 : signed(18 downto 0) := (others => '0');
    signal p13_v  : signed(17 downto 0) := (others => '0');
    signal p13_e  : unsigned(5 downto 0) := (others => '0');
    signal p13_fat, p13_lreg, p13_gr, p13_wa, p13_pa, p13_arc : std_logic := '0';
    signal p14_v  : signed(17 downto 0) := (others => '0');
    signal p14_e  : unsigned(5 downto 0) := (others => '0');
    signal p14_fat, p14_lreg, p14_gr, p14_wa, p14_pa, p14_arc : std_logic := '0';
    signal p15_cl : unsigned(1 downto 0) := (others => '0');
    signal p15_e  : unsigned(5 downto 0) := (others => '0');
    signal p15_lreg, p15_gr, p15_wa, p15_pa, p15_arc : std_logic := '0';
    signal p11_e, p12_e : unsigned(5 downto 0) := (others => '0');
    signal p11_fat, p11_lreg, p11_gr, p11_wa, p11_pa, p11_arc : std_logic := '0';
    signal p12_fat, p12_lreg, p12_gr, p12_wa, p12_pa, p12_arc : std_logic := '0';
    signal p18_q : std_logic_vector(63 downto 0) := (others => '0');
    -- palette morph dither: per line, nb of every 8 pixels show B and (half) one shows (A + B) / 2
    signal s_dx  : unsigned(2 downto 0) := (others => '0');
    signal s_lpar, l_half, p18_sb, p18_sm : std_logic := '0';
    signal l_nb  : unsigned(3 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
    -- Line buffer and ROMs
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
            r_ex_q  <= unsigned(C_AW_EXP2(to_integer(r_ex_a)));
            r_pal_q <= C_AW_PAL(to_integer(r_pal_a));
            r_ffl_q <= C_AW_FFL(to_integer(r_ffl_a));
            r_frl_q <= C_AW_FRL(to_integer(r_frl_a));
            r_fsl_q <= C_AW_FSL(to_integer(r_fsl_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_c  : signed(14 downto 0);
        variable v_p  : unsigned(13 downto 0);
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
            -- speed: |k4 - 512| - 8 (+-8 deadband), sign
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_splitoff <= registers_in(6)(0);
                s_arcsoff  <= registers_in(6)(1);
                s_flip     <= registers_in(6)(2);
                s_smooth   <= registers_in(6)(3);
                s_src      <= registers_in(6)(4);
                -- palette position (k6 + 1) * 7 / 32 = 0 .. 224: palette s_pal, morph s_palf / 32 toward the next
                v_p := resize(s_g6(12 downto 3), 14) + 1;
                v_p := shift_left(v_p, 3) - v_p;
                s_pal <= v_p(12 downto 10); s_palf <= v_p(9 downto 5);
                s_xs       <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel (K1 K2 K3 K5 K6 P12)
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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k.
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
                    when 510 => z_sel <= to_unsigned(20, 5); z_pre <= '1';
                    when 534 => z_sel <= to_unsigned(21, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  =>                                     -- steps exponent: -3.25 .. +1.9 octaves
                        if s_g1(12 downto 3) >= 512 then
                            z_fa <= resize(s_g1(12 downto 3) - 512, 32); z_neg <= '0'; z_fb <= to_unsigned(974, 16);
                        else
                            z_fa <= resize(512 - s_g1(12 downto 3), 32); z_neg <= '1'; z_fb <= to_unsigned(1664, 16);
                        end if;
                    when 1  => z_fa <= to_unsigned(1664, 32);          z_fb <= z_mt;                      -- cell
                    when 2  => z_fa <= resize(c_cell, 32);             z_fb <= to_unsigned(197, 16);      -- column phase
                    when 3  => z_fa <= resize(c_cell, 32);             z_fb <= to_unsigned(108, 16);      -- row phase
                    when 4  => z_fa <= resize(c_cell, 32);             z_fb <= to_unsigned(236, 16);      -- row lag
                    when 5  =>                                                                            -- lines exponent
                        if s_g2(12 downto 3) <= 512 then z_fa <= resize(512 - s_g2(12 downto 3), 32); z_neg <= '0';
                        else z_fa <= resize(s_g2(12 downto 3) - 512, 32); z_neg <= '1'; end if;
                        z_fb <= to_unsigned(614, 16);
                    when 6  => z_fa <= to_unsigned(885, 32);           z_fb <= z_mt;                      -- line rate
                    when 7  => z_fa <= resize(s_gp(12 downto 3), 32);  z_fb <= to_unsigned(1058, 16);     -- arch exponent
                    when 8  => z_fa <= to_unsigned(51990, 32);         z_fb <= z_mt;                      -- curvature
                    -- slots 9 .. 21 chain through the one temporary z_sq
                    when 9  => z_fa <= resize(s_g3(12 downto 3), 32);  z_fb <= resize(s_g3(12 downto 3), 16);  -- etch^2
                    when 10 => z_fa <= resize(s_g3(12 downto 3), 32);  z_fb <= z_sq(19 downto 4);         -- etch^3
                    when 11 => z_fa <= to_unsigned(1762, 32);          z_fb <= z_sq(15 downto 0);         -- etch gain
                    when 12 => z_fa <= resize(s_vm, 32);               z_fb <= resize(s_vm, 16);          -- speed^2
                    when 13 =>                                                                            -- line drift
                        if s_vneg = '0' then z_fa <= resize(s_vm, 32); z_fb <= to_unsigned(7936, 16);     -- right: m
                        else z_fa <= resize(z_m2, 32); z_fb <= to_unsigned(66, 16); end if;               -- left: m^2
                    when 14 => z_fa <= resize(z_m2, 32);               z_fb <= resize(s_vm, 16);          -- speed^3
                    when 15 =>                                                                            -- grid crawl
                        if s_vneg = '0' then z_fa <= resize(z_sq, 32); z_fb <= to_unsigned(43, 16);       -- right: m^3
                        else z_fa <= resize(s_vm, 32); z_fb <= to_unsigned(608, 16); end if;              -- left: m
                    when 16 => z_fa <= resize(c_cell, 32);             z_fb <= z_sq(15 downto 0);         -- crawl, Q4 h
                    when 17 => z_fa <= resize(z_m2, 32);               z_fb <= to_unsigned(621, 16);      -- breathing
                    when 18 => z_fa <= resize(s_ys, 32);               z_fb <= to_unsigned(5243, 16);     -- tear steps
                    when 20 => z_fa <= to_unsigned(75144, 32);         z_fb <= c_k(17 downto 2);          -- s(1080)
                    when 21 => z_fa <= resize(z_sq, 32);               z_fb <= resize(s_g5(12 downto 3), 16);  -- chevron
                    when others => z_fa <= resize(c_jph, 32);          z_fb <= to_unsigned(5243, 16);     -- 19
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
                    -- motion (last field's steps), beam start, black of the palette
                    when 20 =>
                        if s_vneg = '1' then c_drift <= c_drift - resize(c_dst, 24);
                        else c_drift <= c_drift + resize(c_dst, 24); end if;
                        if s_vneg = '1' then c_crawl <= c_crawl + c_cell - resize(c_cst, 14);
                        else c_crawl <= c_crawl + resize(c_cst, 14); end if;
                        c_bph <= c_bph + c_bst;
                        c_jph <= c_jph + 230;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 2);   -- the box's 4 px delay
                    when 21 =>
                        if c_crawl >= c_cell then c_crawl <= c_crawl - c_cell; end if;
                        c_x016 <= shift_left(resize(c_xb0, 16), 4);
                    when 22 =>
                        if c_crawl >= c_cell then c_crawl <= c_crawl - c_cell; end if;
                    when 25 =>
                        if c_bph(15) = '1' then c_brt <= to_signed(16384, 17) - signed(resize(c_bph - 32768, 17));
                        else c_brt <= to_signed(16384, 17) - signed(resize(32768 - c_bph, 17)); end if;

                    -- steps: e = -(|k1 - 512| * 1664 >> 10) below 512, +(|k1 - 512| * 974 >> 10) above
                    when 50  =>
                        if z_neg = '1' then z_e8c <= -signed(resize(f_acc(19 downto 10), 12));
                        else z_e8c <= signed(resize(f_acc(19 downto 10), 12)); end if;
                    when 51  => r_ex_a <= unsigned(z_e8c(7 downto 0)); z_sh <= resize(z_e8c(11 downto 8), 4);
                    when 53  => z_mt <= r_ex_q;
                    when 74  =>                        -- cell = 1664 * mt >> (15 - sh), sh = -4 .. 1
                        case to_integer(z_sh) is
                            when -4     => c_cell <= f_acc(31 downto 19);
                            when -3     => c_cell <= f_acc(30 downto 18);
                            when -2     => c_cell <= f_acc(29 downto 17);
                            when -1     => c_cell <= f_acc(28 downto 16);
                            when 0      => c_cell <= f_acc(27 downto 15);
                            when others => c_cell <= f_acc(26 downto 14);
                        end case;
                    when 75  =>
                        z_e <= -z_e8c;
                    when 76  => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= resize(z_e(11 downto 8), 4);
                    when 78  =>                        -- ring scale = 2048 * mt >> (15 - sh) = mt >> (4 - sh), sh = -2 .. 3
                        case to_integer(z_sh) is
                            when -2     => c_rk <= resize(r_ex_q(15 downto 6), 15);
                            when -1     => c_rk <= resize(r_ex_q(15 downto 5), 15);
                            when 0      => c_rk <= resize(r_ex_q(15 downto 4), 15);
                            when 1      => c_rk <= resize(r_ex_q(15 downto 3), 15);
                            when 2      => c_rk <= resize(r_ex_q(15 downto 2), 15);
                            when others => c_rk <= resize(r_ex_q(15 downto 1), 15);
                        end case;
                    when 98  => c_phx <= f_acc(20 downto 10);
                    when 122 => c_phs <= f_acc(19 downto 10);
                    when 146 => c_lag <= f_acc(20 downto 10);
                    -- lines: e = +-(|512 - k2| * 614 >> 10); rate = 885 * 2^-e
                    when 170 =>
                        if z_neg = '1' then z_e <= signed(resize(f_acc(19 downto 10), 12));
                        else z_e <= -signed(resize(f_acc(19 downto 10), 12)); end if;
                    when 171 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= resize(z_e(11 downto 8), 4);
                    when 173 => z_mt <= r_ex_q;
                    when 194 =>
                        case to_integer(z_sh) is
                            when -2     => c_rlp <= resize(f_acc(29 downto 17), 11);
                            when -1     => c_rlp <= resize(f_acc(28 downto 16), 11);
                            when 0      => c_rlp <= resize(f_acc(27 downto 15), 11);
                            when others => c_rlp <= resize(f_acc(26 downto 14), 11);
                        end case;
                    -- arch: e = -594 + (p12 * 1058 >> 10); K = 51990 * 2^e
                    when 218 => z_e <= signed(resize(f_acc(20 downto 10), 12)) - 594;
                    when 219 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= resize(z_e(11 downto 8), 4);
                    when 221 => z_mt <= r_ex_q;
                    when 242 =>
                        case to_integer(z_sh) is
                            when -3     => c_k <= resize(f_acc(31 downto 18), 18);
                            when -2     => c_k <= resize(f_acc(31 downto 17), 18);
                            when -1     => c_k <= resize(f_acc(31 downto 16), 18);
                            when 0      => c_k <= resize(f_acc(31 downto 15), 18);
                            when others => c_k <= f_acc(31 downto 14);
                        end case;
                    when 266 => z_sq <= f_acc(19 downto 0);
                    -- etch gain = k3^3 * 1762 >> 30 (400 -> 105 = v0.5's 49 px, 1023 -> 1756)
                    when 290 => z_sq <= resize(f_acc(25 downto 10), 20);
                    when 314 => c_ge <= f_acc(26 downto 16);
                    -- speed: lines (Q16 periods per field) and grid crawl (Q16 cells per field); 640 = v0.5
                    when 338 => z_m2 <= f_acc(17 downto 0);
                    when 362 => c_dst <= f_acc(25 downto 10);
                    when 386 => z_sq <= resize(f_acc(26 downto 10), 20);
                    when 410 => z_sq <= resize(f_acc(22 downto 8), 20);
                    when 434 => c_cst <= f_acc(26 downto 16);
                    when 458 => c_bst <= f_acc(31 downto 16);
                    when 482 => c_sta <= f_acc(23 downto 8);
                    when 506 => c_ja0 <= f_acc(23 downto 8);
                    -- chevron offset d = 150 - s(1080) + (s(1080) + 1170) * k5 / 1024 - breathing (20 h):
                    -- 0 puts the apex below the bottom line, 1023 its outer edge on the outer arc (F = 1200 h)
                    when 530 => z_sq <= resize(f_acc(31 downto 20), 20) + 1170;          -- s(1080) + 1170
                    when 540 =>
                        v_c := resize(shift_right(shift_left(resize(c_brt, 22), 4) + shift_left(resize(c_brt, 22), 2), 14), 15);
                        c_br <= resize(v_c, 6);
                    when 554 => c_d <= resize(signed(resize(f_acc(22 downto 10), 15)) - signed(resize(z_sq, 15)), 14);
                    when 555 => c_d <= c_d + 1320 - resize(c_br, 14);
                    -- S&H grid origin: first column edge right of the beam start, and its index
                    when 560 =>
                        c_ce <= signed(resize(to_unsigned(32416, 16) + resize(c_phx, 16) + resize(c_crawl, 16), 18));
                        c_ck <= (others => '0');
                    when 860 =>
                        c_e0 <= unsigned(c_ce(15 downto 0)) + c_cell;
                        c_k0 <= c_ck + 1;
                        c_sq0  <= signed(resize(c_phs, 18)) - signed(resize(c_cell, 18));
                        c_sqb0 <= signed(resize(c_phs, 18)) + signed(resize(c_lag, 18)) - signed(resize(c_cell, 18));
                    when others => null;
                end case;
                if s_zc > 560 and s_zc < 860 then        -- up to 299 cells (the smallest cells need ~186)
                    if c_ce > signed(resize(c_x016, 18)) then
                        c_ce <= c_ce - signed(resize(c_cell, 18));
                        c_ck <= c_ck - 1;
                    end if;
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
    -- 0), computes the NEXT line: s(y) (two serial products), the S&H rows
    -- (normal and lagged), the ring index, the chevron gates, the tear wobble.
    -- Flip mirrors y in the squarer, so s falls down the screen: the row
    -- tops step either way, one row per clock, holding the engine at state 42
    -- until both have settled (line 0 of a flipped field takes up to ~300
    -- clocks, in the zone; a picture line at most a few)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_q  : unsigned(18 downto 0);
        variable v_s  : signed(17 downto 0);
        variable v_v  : signed(17 downto 0);
        variable v_j  : signed(23 downto 0);
        variable v_hold : std_logic;
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 880 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_sq <= c_sq0; e_sqb <= c_sqb0;
                e_ja <= c_ja0;
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_ja <= e_ja + c_sta;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(17 downto 1);

            v_hold := '0';
            case to_integer(e_lc) is
                when 1 =>        -- (y + 16.5) * 256; Flip mirrors y about the bottom line (1079)
                    if s_flip = '1' then v_q := to_unsigned(280448, 19) - e_vy; else v_q := e_vy + 4224; end if;
                    m_a <= resize(v_q(18 downto 4), 36); m_b <= resize(v_q(18 downto 4), 18); m_acc <= (others => '0');
                when 20 =>
                    m_a <= resize(m_acc(29 downto 12), 36); m_b <= c_k; m_acc <= (others => '0');
                when 41 =>
                    e_s16 <= signed(resize(m_acc(35 downto 18), 18)) - 83;
                -- rows: step the S&H row tops onto s (row top <= s < row top + cell), either way
                when 42 =>
                    v_v := e_s16 - e_sq;
                    if v_v >= signed(resize(c_cell, 18)) then e_sq <= e_sq + signed(resize(c_cell, 18)); v_hold := '1';
                    elsif v_v < 0 then e_sq <= e_sq - signed(resize(c_cell, 18)); v_hold := '1'; end if;
                    v_v := e_s16 - e_sqb;
                    if v_v >= signed(resize(c_cell, 18)) then e_sqb <= e_sqb + signed(resize(c_cell, 18)); v_hold := '1';
                    elsif v_v < 0 then e_sqb <= e_sqb - signed(resize(c_cell, 18)); v_hold := '1'; end if;
                when 43 =>
                    v_v := e_s16 - e_sq;
                    if v_v < 0 then n_vh <= (others => '0'); else n_vh <= unsigned(v_v(13 downto 4)); end if;
                    v_v := e_s16 - e_sqb;
                    if v_v < 0 then n_vbh <= (others => '0'); else n_vbh <= unsigned(v_v(13 downto 4)); end if;
                    n_sh <= resize(shift_right(e_s16, 4), 14);
                    e_ta <= f_tri14(e_ja);
                when 44 =>
                    m_a <= resize(n_vh, 36); m_b <= resize(c_rk, 18); m_acc <= (others => '0');
                    n_shd <= n_sh + resize(c_d, 14);

                    -- tear: (ta * 24) >> 17 (+-3 h), minus the line phase offset 73
                    v_j := shift_left(resize(e_ta, 24), 4) + shift_left(resize(e_ta, 24), 3);
                    n_jm <= resize(shift_right(v_j, 17), 14) - 73;
                when 45 =>
                    if n_shd >= 190 then n_glam <= '1'; else n_glam <= '0'; end if;
                    if n_shd >= 634 and n_shd < 842 then n_gpale <= '1'; else n_gpale <= '0'; end if;
                    n_jmd <= n_jm - n_sh;           -- jit - 73 - s
                when 63 =>
                    if m_acc(23 downto 12) > 63 then n_vn <= (others => '1'); else n_vn <= m_acc(17 downto 12); end if;
                    m_a <= resize(n_vbh, 36); m_b <= resize(c_rk, 18); m_acc <= (others => '0');
                when 82 =>
                    if m_acc(23 downto 12) > 63 then n_vnb <= (others => '1'); else n_vnb <= m_acc(17 downto 12); end if;
                when others => null;
            end case;
            if v_hold = '1' and e_zk = '0' and g_start = '0' then e_lc <= e_lc; end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: x in h, the S&H column edges (Q4 h), the column index (for the
    -- right-hand row lag), the line-buffer read address
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_ed : std_logic;
        variable v_h  : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x   <= c_xb0;
                b_px  <= (others => '0');
                b_nxt <= resize(c_e0, 18);
                b_k   <= c_k0;
                b_first <= '1';
                l_sh <= n_sh; l_vh <= n_vh; l_vbh <= n_vbh; l_vn <= n_vn; l_vnb <= n_vnb;
                l_glam <= n_glam; l_gpale <= n_gpale; l_jms <= n_jmd;
                l_shd <= n_shd;
                p2_ed <= '0';
                -- palette morph: (f + line parity) / 2 half-pixels of B in every 8 (f / 32 over a line pair)
                s_lpar <= not s_lpar;
                v_h := resize(s_palf, 6) + ("00000" & s_lpar);
                l_nb <= v_h(5 downto 2); l_half <= v_h(1);
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
                b_first <= '0';
                if shift_left(resize(b_x, 18), 4) >= b_nxt then v_ed := '1'; else v_ed := '0'; end if;
                if v_ed = '1' then
                    b_nxt <= b_nxt + c_cell;
                    b_k <= b_k + 1;
                end if;
                -- stage 2: this pixel's edge (or the line's first pixel) latches the S&H
                p2_ed <= v_ed or b_first;
                if v_ed = '1' and b_k >= 8 then p2_lag <= '1'; else p2_lag <= '0'; end if;
            end if;
            p2_x <= b_x;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_xe  : unsigned(11 downto 0);
        variable v_a   : signed(14 downto 0);
        variable v_e   : unsigned(5 downto 0);
        variable v_n, v_n2 : signed(8 downto 0);
        variable v_t   : signed(16 downto 0);
        variable v_u   : unsigned(26 downto 0);
        variable v_rs  : std_logic;
        variable v_ar  : std_logic;
        variable v_gap : std_logic;
        variable v_lr  : std_logic;
        variable v_vs  : unsigned(5 downto 0);
        variable v_ra  : signed(13 downto 0);
        variable v_md  : unsigned(10 downto 0);
        variable v_r   : unsigned(2 downto 0);
        variable v_t2  : signed(10 downto 0);
        variable v_rt, v_ks, v_in, v_rb, v_ob, v_nok, v_n6, v_n5, v_nm1, v_nod, v_kf, v_wl : std_logic;
    begin
        if rising_edge(clk) then
            -- 3: x' (line-start hold) - centre; luma
            if p2_x < 160 then v_xe := to_unsigned(200, 12);
            elsif p2_x < 200 then v_xe := to_unsigned(200, 12) + shift_right(p2_x - 160, 2);         -- (d^2/160, two segments)
            elsif p2_x < 240 then v_xe := to_unsigned(210, 12) + shift_right(shift_left(p2_x - 200, 1) + (p2_x - 200), 2);
            else v_xe := p2_x; end if;
            p3_dx <= signed(resize(v_xe, 13)) - 2026;
            p3_lm <= unsigned(s_lb_q);
            p3_ed <= p2_ed; p3_lag <= p2_lag;
            -- 4: |dx|, etch partials
            if p3_dx < 0 then p4_adx <= unsigned(resize(-p3_dx, 12)); else p4_adx <= unsigned(resize(p3_dx, 12)); end if;
            if p3_dx > 0 then p4_rt <= '1'; else p4_rt <= '0'; end if;
            p4_ea <= resize(p3_lm(3 downto 0) * c_ge, 15);
            p4_eb <= resize(p3_lm(7 downto 4) * c_ge, 15);
            p4_ed <= p3_ed; p4_lag <= p3_lag;
            -- 5: etch; |dx| - s (with the tear for the line path)
            p5_e   <= resize(shift_right(resize(p4_ea, 19) + shift_left(resize(p4_eb, 19), 4), 8), 11);
            p5_as  <= signed(resize(p4_adx, 14)) - l_sh;
            p5_asj <= signed(resize(p4_adx, 14)) + l_jms;
            p5_asr <= signed(resize(p4_adx, 14)) - l_shd;
            if p4_adx < 28 then p5_gr <= '1'; else p5_gr <= '0'; end if;
            p5_rt <= p4_rt; p5_ed <= p4_ed; p5_lag <= p4_lag;
            -- 6: F, Fd, Fl - 73
            p6_f   <= p5_as + signed(resize(p5_e, 14));
            p6_flm <= p5_asj + signed(resize(p5_e, 14));
            p6_fr  <= p5_asr + signed(resize(p5_e, 14));        -- F - d (the chevron's frame)
            p6_rt <= p5_rt; p6_gr <= p5_gr; p6_ed <= p5_ed; p6_lag <= p5_lag;
            -- 7: the 2-D sample-and-hold; line phase partials; F / Fd tests
            if s_smooth = '1' then
                p7_fs <= p6_f; p7_fsr <= p6_fr; p7_vs <= (others => '1');
            elsif p6_ed = '1' then
                if p6_lag = '1' then
                    p7_fs <= p6_f + signed(resize(l_vbh, 14)); p7_fsr <= p6_fr + signed(resize(l_vbh, 14)); p7_vs <= l_vnb;
                else
                    p7_fs <= p6_f + signed(resize(l_vh, 14)); p7_fsr <= p6_fr + signed(resize(l_vh, 14)); p7_vs <= l_vn;
                end if;
            end if;
            p7_pl <= resize(p6_flm * signed('0' & c_rlp(5 downto 0)), 21);
            p7_ph <= resize(p6_flm * signed('0' & c_rlp(10 downto 6)), 19);
            v_rs := p6_rt and not s_splitoff;
            -- flags from logic: 9 right, 10 right & split, 11 green; the region flags of F and F - d come from
            -- the flag ROMs (address now, merged at stage 10)
            p7_fl <= (others => '0');
            p7_fl(9) <= p6_rt; p7_fl(10) <= v_rs; p7_fl(11) <= p6_gr;
            v_ra := p6_f + 4096; r_ffl_a <= unsigned(v_ra(12 downto 2));
            -- F - d runs past the ROM's +-4096 h at the chevron's extremes: clamp to the end bins
            if p6_fr(13 downto 12) = "01" then r_frl_a <= (others => '1');
            elsif p6_fr(13 downto 12) = "10" then r_frl_a <= (others => '0');
            else r_frl_a <= (not p6_fr(12)) & unsigned(p6_fr(11 downto 2)); end if;
            -- 8: colouriser step partials ((Fs - 40) >> 3) * 79; line phase sum; Fs tests
            v_a := shift_right(resize(p7_fs, 15) - 40, 3);
            p8_a80 <= shift_left(resize(v_a, 19), 6) + shift_left(resize(v_a, 19), 4);
            p8_a1  <= resize(v_a, 19);
            p8_phi  <= shift_left(resize(p7_ph, 26), 6) + resize(p7_pl, 26);
            if p7_fsr(13 downto 12) = "01" then r_fsl_a <= (others => '1');
            elsif p7_fsr(13 downto 12) = "10" then r_fsl_a <= (others => '0');
            else r_fsl_a <= (not p7_fsr(12)) & unsigned(p7_fsr(11 downto 2)); end if;
            p8_vs <= p7_vs; p8_fl <= p7_fl;
            -- 9: step n; phase minus the drift
            p9_n   <= resize(shift_right(p8_a80 - p8_a1, 10), 9);
            p9_phd <= unsigned(p8_phi(23 downto 0)) - c_drift;
            p9_vs <= p8_vs; p9_fl <= p8_fl;
            p9_ffl <= r_ffl_q; p9_frl <= r_frl_q;
            -- 10: step tests; fast triangle; slow phase
            v_n := p9_n;
            if v_n >= -2 and v_n <= 12 then p10_nok <= '1'; else p10_nok <= '0'; end if;
            v_n2 := v_n + 2;
            p10_ni <= unsigned(v_n2(3 downto 0));      -- a slice (resize would fold 8..14)
            if v_n = 6 then p10_n6 <= '1'; else p10_n6 <= '0'; end if;
            if v_n <= 5 then p10_n5 <= '1'; else p10_n5 <= '0'; end if;
            if v_n >= -1 then p10_nm1 <= '1'; else p10_nm1 <= '0'; end if;
            p10_nodd <= v_n(0);
            p10_t1 <= f_tri14(p9_phd(15 downto 0));
            v_u := shift_left(resize(p9_phd, 27), 3) + resize(p9_phd, 27);
            p10_u2 <= v_u(21 downto 6) + 6547;
            p10_vs <= p9_vs; p10_fsl <= r_fsl_q;
            p10_fl(0) <= p9_ffl(1); p10_fl(1) <= p9_frl(2);      -- arch region, nested-lines region
            p10_fl(2) <= p9_frl(0); p10_fl(3) <= p9_frl(1);
            p10_fl(4) <= p9_ffl(1); p10_fl(5) <= p9_ffl(2); p10_fl(6) <= p9_ffl(3);
            p10_fl(7) <= (p9_fl(9) and p9_ffl(5)) or (not p9_fl(9) and p9_ffl(4));
            p10_fl(8) <= p9_ffl(0) and not s_arcsoff;
            p10_fl(12 downto 9) <= p9_fl(12 downto 9);
            -- 11: slow triangle; the base entry (mosaic, chevron, rings, the right half's gap)
            p11_t1 <= p10_t1;
            p11_t2 <= f_tri14(p10_u2);
            v_rs := p10_fl(10);
            v_rt := p10_fl(9);
            v_ks := p10_fsl(0) and l_glam; v_in := p10_fsl(1); v_rb := p10_fsl(2); v_ob := p10_fsl(3);
            v_vs := p10_vs;
            v_nok := p10_nok; v_n6 := p10_n6; v_n5 := p10_n5; v_nm1 := p10_nm1; v_nod := p10_nodd;
            v_kf := p10_fl(2); v_wl := p10_fl(3);
            v_e := to_unsigned(15, 6);
            -- mosaic cells inside the colour window (steps n = -2 .. 12; the right half's step 6 = olive)
            if p10_fl(7) = '1' and v_nok = '1' then
                v_e := resize(p10_ni, 6);
                if v_rs = '1' and v_n6 = '1' then v_e := to_unsigned(16, 6); end if;
            end if;
            if v_in = '1' then v_e := to_unsigned(15, 6); end if;                                    -- inside
            if v_ks = '1' and v_kf = '0' and v_wl = '1' then v_e := to_unsigned(18, 6); end if;       -- W
            if v_ks = '1' and v_kf = '1' then v_e := to_unsigned(17, 6); end if;                      -- P
            if v_ks = '1' and v_rb = '1' then                                                        -- ring
                if (v_rt = '1' and v_vs < 6) or (v_rt = '0' and v_vs < 30) then
                    v_e := 24 + v_vs;
                end if;
            end if;
            if v_in = '1' and v_rt = '1' and v_ob = '1' and l_gpale = '1' then v_e := to_unsigned(18, 6); end if;
            v_gap := v_rs and p10_fl(5) and (p10_fl(6) or v_n5);
            v_lr := (not v_rs) and p10_fl(4) and v_nm1 and v_nod;
            if v_lr = '1' and v_vs < 9 then v_e := 24 + v_vs; end if;
            v_ar := p10_fl(0);
            if v_rs = '1' and v_ar = '1' then v_e := to_unsigned(23, 6); end if;
            if v_gap = '1' then
                if v_n6 = '1' and v_vs < 8 then v_e := 24 + v_vs; else v_e := to_unsigned(15, 6); end if;
            end if;
            p11_e <= v_e;
            p11_fat <= v_rs and v_ar;
            p11_lreg <= v_ar or (p10_fl(1) and v_in and not (v_rt and v_ob));
            p11_wa <= v_ar and v_ks and not v_kf and v_wl;
            p11_pa <= v_ar and v_ks and v_kf;
            p11_gr <= p10_fl(11);
            p11_arc <= p10_fl(8);
            -- 12: 0.636 * T2 partials
            p12_t1 <= p11_t1;
            v_t2 := resize(shift_right(p11_t2, 6), 11);                 -- T2 at 1/256 (Q8)
            p12_t2 <= shift_left(resize(v_t2, 19), 7) + shift_left(resize(v_t2, 19), 5) + shift_left(resize(v_t2, 19), 1) + resize(v_t2, 19);
            p12_e <= p11_e; p12_fat <= p11_fat; p12_lreg <= p11_lreg; p12_gr <= p11_gr;
            p12_wa <= p11_wa; p12_pa <= p11_pa; p12_arc <= p11_arc;
            -- 13: V
            p13_v <= resize(p12_t1, 18) + resize(shift_right(p12_t2, 2), 18);     -- T1 + 0.636 T2 (Q14)
            p13_e <= p12_e; p13_fat <= p12_fat; p13_lreg <= p12_lreg; p13_gr <= p12_gr;
            p13_wa <= p12_wa; p13_pa <= p12_pa; p13_arc <= p12_arc;
            -- 14: register
            p14_v <= p13_v; p14_e <= p13_e; p14_fat <= p13_fat; p14_lreg <= p13_lreg; p14_gr <= p13_gr;
            p14_wa <= p13_wa; p14_pa <= p13_pa; p14_arc <= p13_arc;
            -- 15: line class (0 see-through, 1 teal, 2 blue rim, 3 cyan)
            if p14_fat = '1' then
                if p14_v > -7504 then p15_cl <= "11"; elsif p14_v > -14303 then p15_cl <= "10";
                elsif p14_v > -17465 then p15_cl <= "01"; else p15_cl <= "00"; end if;
            else
                if p14_v > 8126 then p15_cl <= "11"; elsif p14_v > 426 then p15_cl <= "10";
                elsif p14_v > -4194 then p15_cl <= "01"; else p15_cl <= "00"; end if;
            end if;
            p15_e <= p14_e; p15_lreg <= p14_lreg; p15_gr <= p14_gr; p15_wa <= p14_wa; p15_pa <= p14_pa; p15_arc <= p14_arc;
            -- 16: final entry, palette address
            v_e := p15_e;
            if p15_lreg = '1' then
                case p15_cl is
                    when "01"   => v_e := to_unsigned(21, 6);
                    when "10"   => v_e := to_unsigned(20, 6);
                    when "11"   => if p15_gr = '1' then v_e := to_unsigned(22, 6); else v_e := to_unsigned(19, 6); end if;
                    when others => null;
                end case;
            end if;
            if p15_wa = '1' then v_e := to_unsigned(18, 6); end if;
            if p15_pa = '1' then v_e := to_unsigned(17, 6); end if;
            if p15_arc = '1' then v_e := to_unsigned(15, 6); end if;
            -- (black outside the generated avid, so the box filter's edges and the zone read the morphed black)
            if s_gav_sr(13) = '0' then r_pal_a <= s_pal & "001111"; else r_pal_a <= s_pal & v_e; end if;
            -- 17: ROM {A, B}; 18: fabric copy, the dither choice (bit-reversed pixel phase, so the B pixels spread)
            p18_q <= r_pal_q;
            s_dx <= s_dx + 1;
            v_r := s_dx(0) & s_dx(1) & s_dx(2);
            if resize(v_r, 4) < l_nb then p18_sb <= '1'; else p18_sb <= '0'; end if;
            if v_r = l_nb(2 downto 0) and l_half = '1' then p18_sm <= '1'; else p18_sm <= '0'; end if;
            -- 19: compose: A, B or (A + B) / 2
            if p18_sb = '1' then
                s_y19 <= unsigned(p18_q(33 downto 24)); s_u19 <= unsigned(p18_q(23 downto 14)); s_v19 <= unsigned(p18_q(13 downto 4));
            elsif p18_sm = '1' then
                v_md := ('0' & unsigned(p18_q(63 downto 54))) + unsigned(p18_q(33 downto 24)); s_y19 <= v_md(10 downto 1);
                v_md := ('0' & unsigned(p18_q(53 downto 44))) + unsigned(p18_q(23 downto 14)); s_u19 <= v_md(10 downto 1);
                v_md := ('0' & unsigned(p18_q(43 downto 34))) + unsigned(p18_q(13 downto 4));  s_v19 <= v_md(10 downto 1);
            else
                s_y19 <= unsigned(p18_q(63 downto 54)); s_u19 <= unsigned(p18_q(53 downto 44)); s_v19 <= unsigned(p18_q(43 downto 34));
            end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through a [1 2 2 2 2 2 2 2 1]/16 box
    -- (two adjacent 8-sums; the photo's horizontal softness, and the HD HDMI
    -- bar fix, and the palette morph's mixer), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 20 sums, stage 21 output
            s_cay(0) <= s_y19; s_cau(0) <= s_u19; s_cav(0) <= s_v19;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            s_bay <= s_bay + s_y19 - s_cay(7);
            s_bau <= s_bau + s_u19 - s_cau(7);
            s_bav <= s_bav + s_v19 - s_cav(7);
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
            v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            -- 22, 23: delay (one box gives the photo's ~7 px horizontal softness; latency kept a multiple of 4)
            s_y22 <= s_y21; s_u22 <= s_u21; s_v22 <= s_v21;
            s_y23 <= s_y22; s_u23 <= s_u22; s_v23 <= s_v22;
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

end architecture archway;
