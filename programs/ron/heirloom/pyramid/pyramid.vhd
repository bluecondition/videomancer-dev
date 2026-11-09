-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Pyramid v0.1.0 -- Heirloom (video phase), from the clip WIN_20231124_00_45_44 at 5.0 s.
--
-- The still: a curved-sided pyramid (an ogive: half-width 421 px * (d / 1000)^0.755 at d lines below the apex,
-- apex at (728, 28)) filled with three oscillators patched into R, G and B -- a screen-fixed wavefolded stripe
-- on G (period 322.75 px), a slow cosine on B (700 px, drifting right) and an egg-crate of a horizontal and a
-- vertical cosine through a folding shaper on R (wavy bands and ovals, dark at the apex) -- on a teal ground
-- keyed by the same shape: brightest just outside the edge, falling off in nested arches, with a shallow roof
-- field taking over in the top corners (L = 172 - min(0.14 A, 0.219 y - 0.043 |dx| + 84)).  In the clip the
-- pyramid drifts LEFT at ~123 px/s (2 px per field), the ground follows it, the stripes stay on screen, the
-- B wave drifts right and the R pattern evolves.
--
--   K1  Width     pyramid half-width x0.35 .. x2.8 (the still at 512)
--   K2  Curve     edge exponent 1.6 (spire) .. 0.755 (the still, 512) .. 0.355 (dome)
--   K3  Stripes   G stripe period x4 .. x1/4 (322.75 px at 512)
--   K4  Drift     bipolar pyramid drift (+-8 deadband, square law); 324 = the clip's 2 px / field left
--   K5  Rings     R shaper gain x1/8 .. x4: flat red .. dense rings and ovals
--   K6  Warp      how far the source (camera luma or the Osc) bends the stripes and the rings
--   S7  Ground    Teal (the still) / Grey (the clip two minutes later)
--   S8  Twin      one pyramid / a row of pyramids 960 px apart
--   S9  Colour    RGB (the still) / GBR: the oscillators rotated through the guns
--   S10 Arches    Smooth (the still) / Banded ground
--   S11 Source    Video / Osc
--   P12 RISE      apex height: a small peak at the bottom (0) .. the still (538) .. a towering wall (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (the NEXT line's half-width by log2 -> * exponent -> exp2 -> * width, the ground's roof term, the R
-- y-cosine, the apex darkening), a beam DDA (key distance with wrap, three phase accumulators), a 25-stage pixel
-- path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (U and V
-- through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum resync, Y through [1 2 1]/4 on the same centre,
-- luma 13/16) before the blanking gate.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.pyramid_pkg.all;
use work.all;

architecture pyramid of program_top is
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

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_g_a, r_b_a, r_rx_a, r_ry_a : unsigned(8 downto 0) := (others => '0');
    signal r_g_q, r_b_q : unsigned(7 downto 0) := (others => '0');
    signal r_rx_q, r_ry_q : signed(7 downto 0) := (others => '0');
    signal r_lg_a, r_ex_a, r_ex2_a : unsigned(7 downto 0) := (others => '0');
    signal r_lg_q : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_ex2_q : unsigned(15 downto 0) := (others => '0');
    signal r_bg_a : unsigned(8 downto 0) := (others => '0');
    signal r_bgr_q, r_bgg_q, r_bgb_q : unsigned(7 downto 0) := (others => '0');
    signal r_mr_a, r_mg_a, r_mb_a : unsigned(7 downto 0) := (others => '0');
    signal r_mr_q, r_mg_q, r_mb_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 642 x 541 virtual px, drifting
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
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(384 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(538 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(324, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_gnd, s_twin, s_col, s_band, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh1, z_sh3, z_sh5 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt3, z_mt5 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_hi   : std_logic := '0';
    signal c_m2   : unsigned(17 downto 0) := (others => '0');
    signal c_dstep : unsigned(13 downto 0) := (others => '0');      -- drift, Q8 h per field
    signal c_xa   : signed(22 downto 0) := to_signed(728 * 2 * 256, 23);   -- apex x, Q8 h, in [0, P)
    signal c_xai  : signed(13 downto 0) := (others => '0');
    signal c_dx0  : signed(13 downto 0) := (others => '0');          -- beam start dx (h)
    signal c_per  : signed(13 downto 0) := to_signed(5120, 14);      -- period (h)
    signal c_ph   : signed(13 downto 0) := to_signed(2560, 14);      -- half period (h)
    signal c_phm  : signed(13 downto 0) := to_signed(2558, 14);      -- half period - xs
    signal c_xsmp : signed(13 downto 0) := to_signed(-5118, 14);     -- xs - period
    signal c_w0   : unsigned(11 downto 0) := to_unsigned(842, 12);   -- half-width at d = 1000 (h)
    signal c_p    : unsigned(8 downto 0) := to_unsigned(193, 9);     -- edge exponent, Q8
    signal c_fg   : unsigned(16 downto 0) := to_unsigned(25991, 17); -- G phase per h (2^24 per cycle)
    signal c_fgs  : unsigned(19 downto 0) := to_unsigned(51982, 20); -- per pixel
    signal c_fg0  : unsigned(23 downto 0) := (others => '0');        -- at the line start
    signal c_bs, c_rxs : unsigned(15 downto 0) := (others => '0');
    signal c_b0, c_rx0 : unsigned(23 downto 0) := (others => '0');
    signal c_ob   : unsigned(23 downto 0) := to_unsigned(16777216 - 20852160 mod 16777216, 24);   -- B offset
    signal c_orx  : unsigned(23 downto 0) := to_unsigned(12149000, 24);
    signal c_ory  : unsigned(23 downto 0) := to_unsigned(14125000, 24);
    signal c_gk   : unsigned(8 downto 0) := to_unsigned(101, 9);     -- R shaper gain, Q8
    signal c_wg   : unsigned(9 downto 0) := to_unsigned(384, 10);    -- warp gain
    signal c_ya   : signed(12 downto 0) := to_signed(28, 13);        -- apex y (virtual lines)
    signal c_rys  : unsigned(16 downto 0) := to_unsigned(18725, 17); -- R y-phase per line

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(17 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ry   : unsigned(23 downto 0) := (others => '0');
    signal e_d    : signed(12 downto 0) := (others => '0');
    signal e_dpos : std_logic := '0';
    signal e_dd   : unsigned(11 downto 0) := (others => '0');
    signal e_art  : unsigned(8 downto 0) := (others => '0');
    signal e_e    : unsigned(3 downto 0) := (others => '0');
    signal e_lq   : unsigned(11 downto 0) := (others => '0');
    signal e_tq   : signed(13 downto 0) := (others => '0');
    signal e_tneg : std_logic := '0';
    signal e_tt   : signed(13 downto 0) := (others => '0');
    signal e_sf   : unsigned(4 downto 0) := (others => '0');
    signal e_pc   : unsigned(15 downto 0) := (others => '0');
    signal m_a    : unsigned(27 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(27 downto 0) := (others => '0');
    signal n_w    : signed(14 downto 0) := (others => '0');
    signal n_bq   : unsigned(12 downto 0) := (others => '0');
    signal n_ar   : unsigned(7 downto 0) := (others => '0');
    signal n_ry   : signed(7 downto 0) := (others => '0');

    -- beam
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_dx  : signed(13 downto 0) := (others => '0');
    signal b_g   : unsigned(23 downto 0) := (others => '0');
    signal b_b   : unsigned(23 downto 0) := (others => '0');
    signal b_rx  : unsigned(23 downto 0) := (others => '0');
    signal l_w   : signed(14 downto 0) := (others => '0');
    signal l_bq  : unsigned(12 downto 0) := (others => '0');
    signal l_ar  : unsigned(7 downto 0) := (others => '0');
    signal l_ry  : signed(7 downto 0) := (others => '0');

    -- pixel path (the number is the stage)
    signal p2_ad, p3_ad : unsigned(11 downto 0) := (others => '0');
    signal p2_side, p3_side, p4_side, p5_side : std_logic := '0';
    signal p3_a  : signed(14 downto 0) := (others => '0');
    signal p4_lm : unsigned(7 downto 0) := (others => '0');
    signal p4_aq : unsigned(12 downto 0) := (others => '0');
    signal p4_bs : unsigned(11 downto 0) := (others => '0');
    signal p4_na : signed(9 downto 0) := (others => '0');
    signal p4_rx : signed(7 downto 0) := (others => '0');
    signal p4_rim, p5_rim, p6_rim, p7_rim, p8_rim, p9_rim, p10_rim, p11_rim, p12_rim, p13_rim : std_logic := '0';
    signal p5_wp : unsigned(17 downto 0) := (others => '0');
    signal p5_bq : signed(13 downto 0) := (others => '0');
    signal p5_aq : unsigned(12 downto 0) := (others => '0');
    signal p5_al, p5_al2, p5_ar : signed(9 downto 0) := (others => '0');
    signal p5_f  : signed(8 downto 0) := (others => '0');
    signal p6_u  : signed(18 downto 0) := (others => '0');
    signal p6_k  : unsigned(15 downto 0) := (others => '0');
    signal p6_m  : unsigned(12 downto 0) := (others => '0');
    signal p6_ar : signed(10 downto 0) := (others => '0');
    signal p7_u16 : unsigned(15 downto 0) := (others => '0');
    signal p7_l  : unsigned(7 downto 0) := (others => '0');
    signal p7_al, p8_al, p9_al, p10_al, p11_al, p12_al : unsigned(5 downto 0) := (others => '0');
    signal p8_tri : unsigned(7 downto 0) := (others => '0');
    signal p8_l   : unsigned(7 downto 0) := (others => '0');
    signal p8_gv, p9_gv, p10_gv : unsigned(7 downto 0) := (others => '0');
    signal p9_r0  : unsigned(8 downto 0) := (others => '0');
    signal p10_r  : unsigned(7 downto 0) := (others => '0');
    signal p11_fr, p11_fg, p11_fb : unsigned(7 downto 0) := (others => '0');
    signal p11_br, p11_bg, p11_bb, p12_br, p12_bg, p12_bb, p13_br, p13_bg, p13_bb : unsigned(7 downto 0) := (others => '0');
    signal p12_dr, p12_dg, p12_db : signed(8 downto 0) := (others => '0');
    signal p13_pr, p13_pg, p13_pb : signed(14 downto 0) := (others => '0');
    signal p14_or, p14_og, p14_ob : signed(9 downto 0) := (others => '0');
    signal p17_mr, p17_mg, p17_mb : std_logic_vector(31 downto 0) := (others => '0');
    signal p18_y1, p18_yb : unsigned(11 downto 0) := (others => '0');
    signal p18_u1, p18_ub, p18_v1, p18_vg : signed(11 downto 0) := (others => '0');
    signal p19_y : unsigned(12 downto 0) := (others => '0');
    signal p19_u, p19_v : signed(12 downto 0) := (others => '0');
    signal s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums), luma [1 2 1]/4 on the same centre
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_cau, s_cav : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y23 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
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
            r_g_q   <= C_PY_G(to_integer(r_g_a));
            r_b_q   <= C_PY_B(to_integer(r_b_a));
            r_rx_q  <= C_PY_RX(to_integer(r_rx_a));
            r_ry_q  <= C_PY_RY(to_integer(r_ry_a));
            r_lg_q  <= C_PY_LOG(to_integer(r_lg_a));
            r_ex_q  <= C_PY_EXP(to_integer(r_ex_a));
            r_ex2_q <= C_PY_EXP(to_integer(r_ex2_a));
            r_bgr_q <= C_PY_BGR(to_integer(r_bg_a));
            r_bgg_q <= C_PY_BGG(to_integer(r_bg_a));
            r_bgb_q <= C_PY_BGB(to_integer(r_bg_a));
            r_mr_q  <= C_PY_MR(to_integer(r_mr_a));
            r_mg_q  <= C_PY_MG(to_integer(r_mg_a));
            r_mb_q  <= C_PY_MB(to_integer(r_mb_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_x  : signed(22 downto 0);
        variable v_d  : signed(13 downto 0);
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
            -- drift: |k4 - 512| - 8 (deadband +-8), sign
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
                s_gnd  <= registers_in(6)(0);
                s_twin <= registers_in(6)(1);
                s_col  <= registers_in(6)(2);
                s_band <= registers_in(6)(3);
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
            -- from the zone start by its own phase / channel counters
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

            -- serial products: slot k loads at zc = 48 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at load + 18).  The slot number is decoded at 46 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 47 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 46  => z_sel <= to_unsigned(0, 4); z_pre <= '1';
                    when 70  => z_sel <= to_unsigned(1, 4); z_pre <= '1';
                    when 94  => z_sel <= to_unsigned(2, 4); z_pre <= '1';
                    when 118 => z_sel <= to_unsigned(3, 4); z_pre <= '1';
                    when 142 => z_sel <= to_unsigned(4, 4); z_pre <= '1';
                    when 166 => z_sel <= to_unsigned(5, 4); z_pre <= '1';
                    when 190 => z_sel <= to_unsigned(6, 4); z_pre <= '1';
                    when 214 => z_sel <= to_unsigned(7, 4); z_pre <= '1';
                    when 238 => z_sel <= to_unsigned(8, 4); z_pre <= '1';
                    when 262 => z_sel <= to_unsigned(9, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 => z_fa <= resize(s_vm, 18);    z_fb <= resize(s_vm, 16);              -- speed^2
                    when 1 => z_fa <= resize(c_m2, 18);    z_fb <= to_unsigned(2122, 16);         -- drift step
                    when 2 => z_fa <= to_unsigned(842, 18); z_fb <= z_mt1;                        -- width
                    when 3 =>                                                                      -- exponent
                        if s_g2(12 downto 3) >= 512 then
                            z_fa <= resize(s_g2(12 downto 3) - 512, 18); z_fb <= to_unsigned(51, 16); z_hi <= '1';
                        else
                            z_fa <= resize(512 - s_g2(12 downto 3), 18); z_fb <= to_unsigned(108, 16); z_hi <= '0';
                        end if;
                    when 4 => z_fa <= to_unsigned(25991, 18); z_fb <= z_mt3;                      -- stripes
                    when 5 => z_fa <= resize(c_fg, 18);    z_fb <= resize(s_xs, 16);              -- per pixel
                    when 6 => z_fa <= resize(c_fg, 18);    z_fb <= resize(s_x0, 16);              -- line start
                    when 7 => z_fa <= to_unsigned(101, 18); z_fb <= z_mt5;                        -- rings
                    when 8 =>                                                                      -- rise
                        if s_gp(12 downto 3) <= 538 then
                            z_fa <= resize(s_gp(12 downto 3), 18); z_fb <= to_unsigned(348, 16); z_hi <= '0';
                        else
                            z_fa <= resize(s_gp(12 downto 3) - 538, 18); z_fb <= to_unsigned(490, 16); z_hi <= '1';
                        end if;
                    when others => z_fa <= to_unsigned(18725, 18); z_fb <= resize(s_ys, 16);     -- R y step
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 32); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's drift step) and the slow oscillators
                    when 20 =>
                        if s_vneg = '1' then c_xa <= c_xa - signed(resize(c_dstep, 23));
                        else c_xa <= c_xa + signed(resize(c_dstep, 23)); end if;
                        c_ob  <= c_ob - 47936;        -- B wave 2 px per field to the right
                        c_orx <= c_orx + 89100;       -- R x-phase 2 rad/s
                        c_ory <= c_ory - 133650;      -- R y-phase -3 rad/s
                        if s_twin = '1' then
                            c_per <= to_signed(1920, 14); c_ph <= to_signed(960, 14);
                        else
                            c_per <= to_signed(5120, 14); c_ph <= to_signed(2560, 14);
                        end if;
                    when 21 | 22 | 23 =>             -- wrap the apex into [0, P)
                        v_x := shift_left(resize(c_per, 23), 8);
                        if c_xa < 0 then c_xa <= c_xa + v_x;
                        elsif c_xa >= v_x then c_xa <= c_xa - v_x; end if;
                    when 24 =>
                        c_xai <= c_xa(21 downto 8);
                        c_phm <= c_ph - signed(resize(s_xs, 14));
                        c_xsmp <= signed(resize(s_xs, 14)) - c_per;
                    when 25 =>
                        c_dx0 <= signed(resize(s_x0, 14)) - c_xai;
                    when 26 =>
                        if c_dx0 < -c_ph then c_dx0 <= c_dx0 + c_per; end if;
                        -- width exponent (Q8 octaves) = 0.75 (k1 - 512): x0.35 .. x2.8
                        v_e := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        v_e := v_e - shift_right(v_e, 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh1 <= v_e(10 downto 8);
                    when 28 =>
                        z_mt1 <= r_ex_q;
                        -- stripes exponent = k3 - 512: x1/4 .. x4 frequency
                        v_e := signed(resize(s_g3(12 downto 3), 12)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh3 <= v_e(10 downto 8);
                    when 30 =>
                        z_mt3 <= r_ex_q;
                        -- rings exponent: -1.5 (512 - k5) below, (k5 - 512) above: x1/8 .. x4
                        v_e := signed(resize(s_g5(12 downto 3), 12)) - 512;
                        if v_e < 0 then v_e := v_e + shift_right(v_e, 1); end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh5 <= v_e(10 downto 8);
                    when 32 =>
                        z_mt5 <= r_ex_q;
                        c_wg <= s_g6(12 downto 3);
                        -- B and R x phases at the line start, steps per pixel
                        case to_integer(s_xs) is
                            when 4 =>
                                c_bs <= to_unsigned(47936, 16); c_rxs <= to_unsigned(25172, 16);
                                c_b0 <= c_ob + 5752320; c_rx0 <= c_orx + 3020640;
                            when 3 =>
                                c_bs <= to_unsigned(35952, 16); c_rxs <= to_unsigned(18879, 16);
                                c_b0 <= c_ob; c_rx0 <= c_orx;
                            when others =>
                                c_bs <= to_unsigned(23968, 16); c_rxs <= to_unsigned(12586, 16);
                                c_b0 <= c_ob; c_rx0 <= c_orx;
                        end case;

                    when 66  => c_m2 <= f_acc(17 downto 0);
                    when 90  => c_dstep <= f_acc(29 downto 16);
                    when 114 =>                       -- W0 = 842 * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_w0 <= resize(f_acc(29 downto 17), 12);
                            when -1     => c_w0 <= resize(f_acc(29 downto 16), 12);
                            when 0      => c_w0 <= f_acc(26 downto 15);
                            when others => c_w0 <= f_acc(25 downto 14);
                        end case;
                    when 138 =>                       -- p = 193 -+ acc / 256
                        if z_hi = '1' then c_p <= to_unsigned(193, 9) - resize(f_acc(15 downto 8), 9);
                        else c_p <= to_unsigned(193, 9) + resize(f_acc(16 downto 8), 9); end if;
                    when 162 =>                       -- fG = 25991 * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh3) is
                            when -2     => c_fg <= resize(f_acc(31 downto 17), 17);
                            when -1     => c_fg <= resize(f_acc(31 downto 16), 17);
                            when 0      => c_fg <= f_acc(31 downto 15);
                            when others => c_fg <= f_acc(30 downto 14);
                        end case;
                    when 186 => c_fgs <= f_acc(19 downto 0);
                    when 210 => c_fg0 <= f_acc(23 downto 0);
                    when 234 =>                       -- gk = 101 * mt >> (15 - sh), sh = -3 .. 1
                        case to_integer(z_sh5) is
                            when -3     => c_gk <= resize(f_acc(22 downto 18), 9);
                            when -2     => c_gk <= resize(f_acc(22 downto 17), 9);
                            when -1     => c_gk <= resize(f_acc(22 downto 16), 9);
                            when 0      => c_gk <= resize(f_acc(22 downto 15), 9);
                            when others => c_gk <= f_acc(22 downto 14);
                        end case;
                    when 258 =>                       -- ya = 760 - gp * 1.36 (to the still at 538), then 28 - ..*1.91
                        if z_hi = '1' then c_ya <= to_signed(28, 13) - signed(resize(f_acc(18 downto 8), 13));
                        else c_ya <= to_signed(760, 13) - signed(resize(f_acc(18 downto 8), 13)); end if;
                    when 282 => c_rys <= f_acc(24 downto 8);
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
    -- 0), computes the NEXT line: the half-width w = W0 (d / 1000)^p by
    -- log2 (normalize + mantissa ROM), * p, exp2 ROM, * W0 (one serial
    -- multiplier, 16 iterations, result at load + 17), the ground's roof term
    -- (3.5 y + 1344, Q4), the apex darkening of R, and the R y-cosine.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_vy : unsigned(10 downto 0);
        variable v_d  : signed(12 downto 0);
        variable v_t  : signed(13 downto 0);
        variable v_ti : signed(5 downto 0);
        variable v_s  : signed(6 downto 0);
        variable v_p  : unsigned(27 downto 0);
        variable v_a  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 400 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_ry <= c_ory;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_ry <= e_ry + resize(c_rys, 24);
                e_lc <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(26 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_vy := e_vy(18 downto 8);
                    e_d <= signed(resize(v_vy, 13)) - c_ya;
                    n_bq <= shift_left(resize(v_vy, 13), 2) - resize(v_vy(10 downto 1), 13) + 1344;
                    r_ry_a <= e_ry(23 downto 15);
                when 2 =>
                    if e_d > 0 then e_dpos <= '1'; else e_dpos <= '0'; end if;
                    if e_d > 4095 then e_dd <= (others => '1');
                    elsif e_d > 0 then e_dd <= unsigned(e_d(11 downto 0));
                    else e_dd <= (others => '0'); end if;
                    e_art <= resize(unsigned(e_d(8 downto 1)), 9) + resize(unsigned(e_d(8 downto 3)), 9);
                when 3 =>
                    n_ry <= r_ry_q;
                    -- apex darkening of R: 193 (1 - d / 300) over the first 300 lines
                    if e_d < 0 then n_ar <= to_unsigned(193, 8);
                    elsif e_d < 300 then
                        v_a := to_unsigned(193, 10) - resize(e_art, 10) - resize(unsigned(e_d(8 downto 6)), 10);
                        n_ar <= v_a(7 downto 0);
                    else n_ar <= (others => '0'); end if;
                    -- log2 normalize: position of the leading one
                    e_e <= (others => '0');
                    for i in 0 to 11 loop
                        if e_dd(i) = '1' then e_e <= to_unsigned(i, 4); end if;
                    end loop;
                when 4 =>
                    r_lg_a <= shift_left(e_dd, 11 - to_integer(e_e))(10 downto 3);
                when 6 =>
                    e_lq <= shift_left(resize(e_e, 12), 8) + resize(r_lg_q, 12);
                when 7 =>
                    v_t := signed(resize(e_lq, 14)) - 2551;            -- log2(d / 1000), Q8
                    e_tneg <= v_t(13);
                    m_a <= resize(unsigned(abs(v_t)), 28); m_b <= resize(c_p, 16); m_acc <= (others => '0');
                when 24 =>
                    if e_tneg = '1' then v_t := -signed(resize(m_acc(20 downto 8), 14));
                    else v_t := signed(resize(m_acc(20 downto 8), 14)); end if;
                    e_tt <= v_t;
                when 25 =>
                    r_ex2_a <= unsigned(e_tt(7 downto 0));
                    v_ti := e_tt(13 downto 8);                        -- floor(t), -17 .. 3
                    v_s := to_signed(3, 7) - resize(v_ti, 7);         -- right shift beyond 12: 0 .. 20
                    if v_s < 0 then e_sf <= (others => '0');
                    elsif v_s > 20 then e_sf <= to_unsigned(21, 5);
                    else e_sf <= unsigned(v_s(4 downto 0)); end if;
                when 27 =>
                    m_a <= resize(c_w0, 28); m_b <= r_ex2_q; m_acc <= (others => '0');
                when 44 =>
                    -- P = W0 * 2^frac (Q15); w = P >> (12 + sf): coarse step (0 / 8 / 16)
                    v_p := shift_right(m_acc, 12);
                    case e_sf(4 downto 3) is
                        when "00"   => e_pc <= v_p(15 downto 0);
                        when "01"   => e_pc <= resize(v_p(15 downto 8), 16);
                        when others => e_pc <= (others => '0');
                    end case;
                when 45 =>
                    if e_dpos = '1' then
                        v_p := resize(shift_right(e_pc, to_integer(e_sf(2 downto 0))), 28);
                        if v_p > 8191 then n_w <= to_signed(8191, 15);
                        else n_w <= signed(v_p(14 downto 0)); end if;
                    else
                        n_w <= shift_left(resize(e_d, 15), 2);        -- above the apex: -4 |d| h
                    end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: key distance dx (h, wrapping every period), the G / B / R-x phase
    -- accumulators; the line's engine values are loaded at g_start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px <= (others => '0');
                b_dx <= c_dx0;
                b_g  <= c_fg0;
                b_b  <= c_b0;
                b_rx <= c_rx0;
                l_w <= n_w; l_bq <= n_bq; l_ar <= n_ar; l_ry <= n_ry;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_dx >= c_phm then b_dx <= b_dx + c_xsmp;
                else b_dx <= b_dx + signed(resize(s_xs, 14)); end if;
                b_rx <= b_rx + resize(c_rxs, 24);
            end if;
            -- the G and B phases run late: b_g holds the stage-5 pixel's phase, b_b the stage-8 pixel's
            -- (advanced while that stage's pixel is active, s_gav_sr(k) = avid of the stage-(k+2) pixel)
            if g_start = '0' then
                if s_gav_sr(3) = '1' then b_g <= b_g + resize(c_fgs, 24); end if;
                if s_gav_sr(6) = '1' then b_b <= b_b + resize(c_bs, 24); end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a   : signed(14 downto 0);
        variable v_n   : signed(9 downto 0);
        variable v_t   : signed(10 downto 0);
        variable v_l   : signed(13 downto 0);
        variable v_u8  : unsigned(7 downto 0);
        variable v_s   : unsigned(23 downto 0);
        variable v_r   : signed(9 downto 0);
        variable v_o   : signed(9 downto 0);
        variable v_y   : unsigned(12 downto 0);
        variable v_c   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: |dx|, side, ROM addresses of B and R-x
            if b_dx < 0 then p2_ad <= unsigned(resize(-b_dx, 12)); p2_side <= '1';
            else p2_ad <= unsigned(b_dx(11 downto 0)); p2_side <= '0'; end if;
            r_rx_a <= b_rx(23 downto 15);
            -- 3: A = |dx| - w (h, outside the key > 0)
            p3_a <= signed(resize(p2_ad, 15)) - l_w;
            p3_ad <= p2_ad; p3_side <= p2_side;
            -- 4: luma, ground terms, alpha input, rim
            p4_lm <= unsigned(s_lb_q);
            if p3_a > 0 then p4_aq <= unsigned(p3_a(12 downto 0)) + resize(unsigned(p3_a(12 downto 3)), 13);
            else p4_aq <= (others => '0'); end if;
            p4_bs <= shift_right(p3_ad, 2) + shift_right(p3_ad, 4) + shift_right(p3_ad, 5);
            v_a := -p3_a;
            if v_a > 255 then p4_na <= to_signed(255, 10);
            elsif v_a < -64 then p4_na <= to_signed(-64, 10);
            else p4_na <= v_a(9 downto 0); end if;
            if p3_side = '1' and p3_a < 0 and p3_a > -21 then p4_rim <= '1'; else p4_rim <= '0'; end if;
            p4_rx <= r_rx_q; p4_side <= p3_side;
            -- 5: warp product, ground roof, alpha partials, R field
            p5_wp <= p4_lm * c_wg;
            p5_bq <= signed(resize(l_bq, 14)) - signed(resize(p4_bs, 14));
            p5_aq <= p4_aq;
            p5_al  <= shift_right(p4_na, 1) + shift_right(p4_na, 3);
            p5_al2 <= shift_right(p4_na, 4) + shift_right(p4_na, 5);
            p5_ar  <= shift_right(p4_na, 2) + shift_right(p4_na, 6);
            p5_f <= resize(p4_rx, 9) + resize(l_ry, 9);
            p5_side <= p4_side; p5_rim <= p4_rim;
            -- 6: G address (phase + warp), R shaper product, ground min, alpha sum
            v_s := b_g + shift_left(resize(p5_wp, 24), 5);
            r_g_a <= v_s(23 downto 15);
            p6_u <= resize(p5_f * signed('0' & c_gk), 19);
            p6_k <= to_unsigned(13369, 16) + resize(p5_wp(17 downto 2), 16);
            if p5_bq < 0 then p6_m <= (others => '0');
            elsif signed(resize(p5_aq, 14)) < p5_bq then p6_m <= p5_aq;
            else p6_m <= unsigned(p5_bq(12 downto 0)); end if;
            if p5_side = '1' then p6_ar <= resize(p5_al, 11) + resize(p5_al2, 11) + 5;
            else p6_ar <= resize(p5_ar, 11) + 5; end if;
            p6_rim <= p5_rim;
            -- 7: R phase, ground level, alpha clamp
            p7_u16 <= unsigned(p6_u(14 downto 0) & '0') + p6_k;
            if p6_m >= 2752 then p7_l <= (others => '0');
            else
                v_l := to_signed(2752, 14) - signed(resize(p6_m, 14));
                p7_l <= unsigned(v_l(11 downto 4));
            end if;
            if p6_ar < 0 then p7_al <= (others => '0');
            elsif p6_ar > 32 then p7_al <= to_unsigned(32, 6);
            else p7_al <= unsigned(p6_ar(5 downto 0)); end if;
            p7_rim <= p6_rim;
            -- 8: R fold (triangle), banded arches, G data
            v_u8 := p7_u16(15 downto 8);
            if v_u8(7) = '1' then p8_tri <= (not v_u8(6 downto 0)) & '0';
            else p8_tri <= v_u8(6 downto 0) & '0'; end if;
            if s_band = '1' and p7_l(3) = '1' then
                if p7_l >= 24 then p8_l <= p7_l - 24; else p8_l <= (others => '0'); end if;
            else
                p8_l <= p7_l;
            end if;
            p8_gv <= r_g_q;
            p8_al <= p7_al; p8_rim <= p7_rim;
            -- 9: ground ROM address; R before the apex darkening
            r_bg_a <= s_gnd & p8_l;
            r_b_a  <= b_b(23 downto 15);
            p9_r0 <= to_unsigned(40, 9) + resize(p8_tri(7 downto 1), 9) + resize(p8_tri(7 downto 5), 9);
            p9_gv <= p8_gv; p9_al <= p8_al; p9_rim <= p8_rim;
            -- 10: R
            v_r := signed(resize(p9_r0, 10)) - signed(resize(l_ar, 10));
            if v_r < 0 then p10_r <= (others => '0');
            elsif v_r > 255 then p10_r <= (others => '1');
            else p10_r <= unsigned(v_r(7 downto 0)); end if;
            p10_gv <= p9_gv; p10_al <= p9_al; p10_rim <= p9_rim;
            -- 11: ground colour, the oscillators into the guns (Colour: GBR)
            p11_br <= r_bgr_q; p11_bg <= r_bgg_q; p11_bb <= r_bgb_q;
            if s_col = '1' then p11_fr <= p10_gv; p11_fg <= r_b_q; p11_fb <= p10_r;
            else p11_fr <= p10_r; p11_fg <= p10_gv; p11_fb <= r_b_q; end if;
            p11_al <= p10_al; p11_rim <= p10_rim;
            -- 12: fg - ground
            p12_dr <= signed(resize(p11_fr, 9)) - signed(resize(p11_br, 9));
            p12_dg <= signed(resize(p11_fg, 9)) - signed(resize(p11_bg, 9));
            p12_db <= signed(resize(p11_fb, 9)) - signed(resize(p11_bb, 9));
            p12_br <= p11_br; p12_bg <= p11_bg; p12_bb <= p11_bb; p12_al <= p11_al; p12_rim <= p11_rim;
            -- 13: x alpha (1/32)
            p13_pr <= resize(p12_dr * signed('0' & p12_al), 15);
            p13_pg <= resize(p12_dg * signed('0' & p12_al), 15);
            p13_pb <= resize(p12_db * signed('0' & p12_al), 15);
            p13_br <= p12_br; p13_bg <= p12_bg; p13_bb <= p12_bb; p13_rim <= p12_rim;
            -- 14: mix + rim
            if p13_rim = '1' then
                p14_or <= signed(resize(p13_br, 10)) + resize(shift_right(p13_pr, 5), 10) + 6;
                p14_og <= signed(resize(p13_bg, 10)) + resize(shift_right(p13_pg, 5), 10) + 8;
                p14_ob <= signed(resize(p13_bb, 10)) + resize(shift_right(p13_pb, 5), 10) + 10;
            else
                p14_or <= signed(resize(p13_br, 10)) + resize(shift_right(p13_pr, 5), 10);
                p14_og <= signed(resize(p13_bg, 10)) + resize(shift_right(p13_pg, 5), 10);
                p14_ob <= signed(resize(p13_bb, 10)) + resize(shift_right(p13_pb, 5), 10);
            end if;
            -- 15: clamp -> matrix ROM addresses
            if p14_or < 0 then r_mr_a <= (others => '0'); elsif p14_or > 255 then r_mr_a <= (others => '1');
            else r_mr_a <= unsigned(p14_or(7 downto 0)); end if;
            if p14_og < 0 then r_mg_a <= (others => '0'); elsif p14_og > 255 then r_mg_a <= (others => '1');
            else r_mg_a <= unsigned(p14_og(7 downto 0)); end if;
            if p14_ob < 0 then r_mb_a <= (others => '0'); elsif p14_ob > 255 then r_mb_a <= (others => '1');
            else r_mb_a <= unsigned(p14_ob(7 downto 0)); end if;
            -- 16: matrix ROM data (EBR); 17: fabric copy
            p17_mr <= r_mr_q; p17_mg <= r_mg_q; p17_mb <= r_mb_q;
            -- 18: partial sums  {Y 31..21, |Cr| 20..11, |Cb| 10..1}
            p18_y1 <= resize(unsigned(p17_mr(31 downto 21)), 12) + resize(unsigned(p17_mg(31 downto 21)), 12);
            p18_yb <= resize(unsigned(p17_mb(31 downto 21)), 12);
            p18_u1 <= signed(resize(unsigned(p17_mr(20 downto 11)), 12)) - signed(resize(unsigned(p17_mg(20 downto 11)), 12));
            p18_ub <= signed(resize(unsigned(p17_mb(20 downto 11)), 12));
            p18_v1 <= signed(resize(unsigned(p17_mb(10 downto 1)), 12)) - signed(resize(unsigned(p17_mr(10 downto 1)), 12));
            p18_vg <= signed(resize(unsigned(p17_mg(10 downto 1)), 12));
            -- 19: sums
            p19_y <= resize(p18_y1, 13) + resize(p18_yb, 13);
            p19_u <= resize(p18_u1, 13) - resize(p18_ub, 13);
            p19_v <= resize(p18_v1, 13) - resize(p18_vg, 13);
            -- 20: 10-bit Y (pre-scaled 16/13, capped at 930), Cr, Cb
            v_y := shift_right(p19_y, 1) + 64;
            if v_y > 930 then s_y20 <= to_unsigned(930, 10); else s_y20 <= v_y(9 downto 0); end if;
            v_c := shift_right(p19_u, 1) + 512;
            if v_c < 0 then s_u20 <= (others => '0'); elsif v_c > 1023 then s_u20 <= (others => '1');
            else s_u20 <= unsigned(v_c(9 downto 0)); end if;
            v_c := shift_right(p19_v, 1) + 512;
            if v_c < 0 then s_v20 <= (others => '0'); elsif v_c > 1023 then s_v20 <= (others => '1');
            else s_v20 <= unsigned(v_c(9 downto 0)); end if;
            -- 21: compose (neutral black outside the generated avid)
            if s_gav_sr(18) = '0' then
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            else
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(18);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums), Y through [1 2 1]/4 on the same centre (4 px back): the
    -- HD HDMI bar fix; luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_ls : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 22: chroma 8-sums (they snap back to neutral once 8 blank samples are in, so a glitched
            -- sample can't offset the colours for good); luma taps
            s_cau(0) <= s_u21; s_cav(0) <= s_v21;
            for i in 1 to 7 loop s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u21 - s_cau(7);
                s_bav <= s_bav + s_v21 - s_cav(7);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_ly(0) <= s_y21;
            for i in 1 to 5 loop s_ly(i) <= s_ly(i - 1); end loop;
            -- 23: filter outputs
            v_sc := resize(s_bau, 14) + s_baud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v23 <= v_sc(13 downto 4);
            v_ls := resize(s_ly(3), 12) + shift_left(resize(s_ly(4), 12), 1) + resize(s_ly(5), 12);
            s_y23 <= v_ls(11 downto 2);
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

end architecture pyramid;
