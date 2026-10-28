-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Halo v0.5.0 -- Heirloom #47, rebuilt from the photo WIN_20230801_00_19_00.
--
-- The photo: a lattice of 2.02 cells across (949 px) by 542 lines.  Every cell
-- holds a glowing yellow-white ellipse (164 x 185 px, 71 px right of the cell
-- centre) on dark olive, a red-orange crescent that starts hot at the cell's
-- left edge and decays to brown across it, and between the cells blue columns
-- of thin purple / teal / white contour lines that pinch into hourglasses and
-- nested concave diamonds at the 4-way points.  Right of every seam the
-- contour family is cut and a white noisy band boils; the first ~430 px of the
-- lines around each cusp row are torn (line-start phase errors).
--
-- The patch: an H triangle (curved sides, fitted S(w)) plus a V parabola
-- (fitted V(|dy|)) = one contour field G through a two-family colourizer (left
-- of the seam / right of the cut); a cell key (G < 58) switches to the ring
-- layer: an ellipse field E2 = dx^2 + dy^2 from the same oscillators, offset
-- 71 px, through a glow window (scaled by Halo, wider on the right), an
-- interior key with a jittered edge, and a red one-shot fired by the cell
-- key's leading edge.  A line-start phase error tears the lines near the cusps.
--
--   K1  Rings   ring size 45 .. 150 % (100 % = the photo's 164 x 185 px)
--   K2  Cells   cells across 1 .. 4 (2.02 = the photo), the whole lattice zooms
--   K3  Pinch   hourglass depth: 0 = straight blue columns, 160 % = deep diamonds
--   K4  Speed   bipolar vertical roll of the lattice (cusps, diamonds, tears ride with it)
--   K5  Tear    line-start tear amount 0 .. 200 %
--   K6  Warp    how far the source (camera luma / Osc) bends the contours and rings
--   S7  Seam    Cut (the photo: white noise band right of the seam) / Mirror
--   S8  Noise   the white band boils (On) / clean (Off)
--   S9  Ember   red crescent Short (the photo) / Long (whole cells ember)
--   S10 Flip    the lattice mirrored left-right (tears stay at the line start)
--   S11 Source  Video / Osc
--   P12 HALO    glow: 0 = bare ellipse outlines, photo at 46 %, 100 % = blazing discs
--
-- Lattice geometry in virtual 1080-line units (round rings at SD / 720p /
-- 1080p).  Generated avid; output conditioning (luma [1 2 1], chroma
-- [1 2 2 2 2 2 2 2 1]/16, luma to ~777) before the blanking gate.  Latency 32
-- clocks in every mode.  Colours in the hardware convention (u = Cr), see halo.py.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.halo_pkg.all;
use work.all;

architecture halo of program_top is
    constant C_LAT : integer := 32;   -- matches processing_delay_clks

    constant C_PHC   : unsigned(23 downto 0) := to_unsigned(16167323, 24);  -- H phase at the screen centre
    constant C_2PHC  : unsigned(23 downto 0) := to_unsigned(15557429, 24);  -- 2 * C_PHC mod 2^24 (flip mirror)
    constant C_PVC   : unsigned(23 downto 0) := to_unsigned(835805, 24);    -- V phase at the vertical centre
    constant C_HRING : unsigned(15 downto 0) := to_unsigned(36953, 16);     -- ring centre, seam-relative
    constant C_CUT   : unsigned(15 downto 0) := to_unsigned(1519, 16);      -- the right family's cut (22 px)

    function f_tri8(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;

    -- per timing ID
    signal t_rw : unsigned(14 downto 0) := to_unsigned(8738, 15);
    signal t_sx : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_ta : unsigned(8 downto 0) := to_unsigned(273, 9);   -- tear px -> phase factor (/2048 per cell)
    signal t_sd : std_logic := '0';                              -- 4:3 SD: ring kx x 4/3 (round rings)
    signal f_tvk  : unsigned(16 downto 0) := to_unsigned(15299, 17);
    signal f_nsy  : unsigned(13 downto 0) := to_unsigned(2731, 14);
    signal f_kax  : unsigned(7 downto 0) := to_unsigned(57, 8);
    signal f_kby  : unsigned(9 downto 0) := to_unsigned(94, 10);

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K1, K2, K3, K5, K6, P12 (Q.3)
    type t_g is array(0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(550 * 8, 13), to_unsigned(349 * 8, 13), to_unsigned(639 * 8, 13),
                         to_unsigned(512 * 8, 13), to_unsigned(153 * 8, 13), to_unsigned(472 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- knob decodes
    signal k_rq    : unsigned(9 downto 0) := to_unsigned(473, 10);   -- 1023 - K1 (ring size, square law)
    signal k_cells : unsigned(10 downto 0) := to_unsigned(517, 11);  -- cells across, Q8 (256 .. 1022)
    signal k_pinch : unsigned(7 downto 0) := to_unsigned(128, 8);    -- Q7 (0 .. 207)
    signal k_tear  : unsigned(8 downto 0) := to_unsigned(256, 9);    -- Q8 (0 .. 511)
    signal k_warp  : unsigned(5 downto 0) := to_unsigned(9, 6);
    signal k_hi    : unsigned(7 downto 0) := to_unsigned(118, 8);    -- Halo table index (P12 / 4)
    signal r_p12   : std_logic_vector(31 downto 0) := (others => '0');

    -- serial multiplier (per-frame sequencer, vblank only)
    signal m_a  : unsigned(19 downto 0) := (others => '0');
    signal m_b  : unsigned(17 downto 0) := (others => '0');
    signal m_h  : unsigned(19 downto 0) := (others => '0');
    signal m_l  : unsigned(17 downto 0) := (others => '0');
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 9) := (others => '0');
    signal f_kq : unsigned(15 downto 0) := (others => '0');      -- (1023 - K1)^2 / 16
    signal s_m4 : unsigned(8 downto 0) := (others => '0');
    signal s_n4 : std_logic := '0';

    -- per-frame values
    signal f_hinc : unsigned(19 downto 0) := to_unsigned(17680, 20);
    signal f_h0   : unsigned(23 downto 0) := (others => '0');
    signal f_vinc : unsigned(19 downto 0) := to_unsigned(30957, 20);
    signal f_v0   : unsigned(23 downto 0) := (others => '0');
    signal f_tamp : unsigned(10 downto 0) := to_unsigned(518, 11);
    signal f_kx   : unsigned(11 downto 0) := to_unsigned(1482, 12);
    signal f_ky   : unsigned(11 downto 0) := to_unsigned(750, 12);
    signal f_roll : unsigned(23 downto 0) := (others => '0');
    signal f_spd  : signed(15 downto 0) := (others => '0');
    signal f_s    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_sk   : unsigned(8 downto 0) := to_unsigned(108, 9);
    signal f_kv   : std_logic := '1';
    signal f_tk   : signed(16 downto 0) := (others => '0');
    signal f_pa0, f_pb0 : unsigned(15 downto 0) := (others => '0');
    signal f_fr   : unsigned(7 downto 0) := (others => '0');
    signal f_seam, f_noise, f_ember, f_flip, f_osc : std_logic := '0';

    -- per-line engine (next line) and committed line values
    signal le_st : unsigned(7 downto 0) := (others => '0');
    signal le_first : std_logic := '0';
    signal n_v24 : unsigned(23 downto 0) := (others => '0');
    signal n_fv  : unsigned(15 downto 0) := (others => '0');
    signal n_dy  : unsigned(15 downto 0) := (others => '0');
    signal n_va  : unsigned(7 downto 0) := (others => '0');
    signal r_vq, r_vq2 : std_logic_vector(7 downto 0) := (others => '0');
    signal n_vp  : unsigned(8 downto 0) := (others => '0');
    signal n_dyn2 : unsigned(19 downto 0) := (others => '0');
    signal n_cu  : unsigned(15 downto 0) := (others => '0');
    signal n_ty  : unsigned(1 downto 0) := (others => '0');     -- 0 none / stretch, 1 shift, 2 centre (flare colour)
    signal n_shp : std_logic := '0';
    signal n_a   : unsigned(7 downto 0) := (others => '0');
    signal n_amp : unsigned(15 downto 0) := (others => '0');
    signal n_kj  : signed(15 downto 0) := (others => '0');
    signal n_tkj : signed(17 downto 0) := (others => '0');
    signal n_m9  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal n_lf  : unsigned(15 downto 0) := x"ACE1";
    signal n_ra  : unsigned(23 downto 0) := (others => '0');
    signal n_pb  : unsigned(15 downto 0) := (others => '0');
    signal n_g0  : signed(8 downto 0) := (others => '0');
    signal n_nd  : signed(9 downto 0) := (others => '0');
    signal n_nz  : signed(8 downto 0) := (others => '0');
    signal lm_a : unsigned(18 downto 0) := (others => '0');
    signal lm_b : unsigned(15 downto 0) := (others => '0');
    signal lm_h : unsigned(18 downto 0) := (others => '0');
    signal lm_l : unsigned(15 downto 0) := (others => '0');
    signal lm_on : std_logic := '0';

    signal l_vp   : unsigned(8 downto 0) := (others => '0');
    signal l_dyn2 : unsigned(19 downto 0) := (others => '0');
    signal l_ty   : unsigned(1 downto 0) := (others => '0');
    signal l_shp  : std_logic := '0';
    signal l_amp  : unsigned(11 downto 0) := (others => '0');
    signal l_tkj  : signed(17 downto 0) := (others => '0');
    signal l_pb   : unsigned(15 downto 0) := (others => '0');
    signal l_nz   : signed(8 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';
    signal g_avd : std_logic := '0';

    -- video tap
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal v_s8 : signed(7 downto 0) := (others => '0');

    -- beam (stage 0)
    signal b_h  : unsigned(23 downto 0) := (others => '0');
    signal b_xv : unsigned(19 downto 0) := (others => '0');
    signal b_pa : unsigned(15 downto 0) := (others => '0');
    -- noise lattice ROM port (line engine)
    signal z_ad : unsigned(7 downto 0) := (others => '0');
    signal z_q, z_q2 : std_logic_vector(7 downto 0) := (others => '0');

    -- ROM ports
    signal r_ta : unsigned(8 downto 0) := (others => '0');
    signal r_tq : std_logic_vector(7 downto 0) := (others => '0');
    signal r_sa : unsigned(8 downto 0) := (others => '0');
    signal r_sq : std_logic_vector(9 downto 0) := (others => '0');
    signal r_qa : unsigned(8 downto 0) := (others => '0');
    signal r_qq : std_logic_vector(15 downto 0) := (others => '0');
    signal r_pa : unsigned(9 downto 0) := (others => '0');
    signal r_py, r_pu, r_pv : std_logic_vector(7 downto 0) := (others => '0');
    signal r_ra : unsigned(7 downto 0) := (others => '0');
    signal r_rq : std_logic_vector(7 downto 0) := (others => '0');
    signal r_ga : unsigned(8 downto 0) := (others => '0');
    signal r_gq : std_logic_vector(31 downto 0) := (others => '0');

    -- pixel pipeline
    signal p1_hm : unsigned(15 downto 0) := (others => '0');
    signal p1_s8 : signed(7 downto 0) := (others => '0');
    signal p2_ws : signed(12 downto 0) := (others => '0');
    signal p3_sh : signed(7 downto 0) := (others => '0');
    signal p3_gw : signed(7 downto 0) := (others => '0');
    signal p3_ew : signed(9 downto 0) := (others => '0');
    signal p4_d  : signed(18 downto 0) := (others => '0');
    signal p5_hb : unsigned(15 downto 0) := (others => '0');
    signal p6_ax : unsigned(8 downto 0) := (others => '0');
    signal p6_dng, p6_rs : std_logic := '0';
    signal p6_fd : unsigned(2 downto 0) := (others => '0');
    signal p7_m  : unsigned(17 downto 0) := (others => '0');
    signal p7_dng, p7_rs : std_logic := '0';
    signal p7_fd : unsigned(2 downto 0) := (others => '0');
    signal p8_s  : signed(10 downto 0) := (others => '0');
    signal p8_adn : unsigned(9 downto 0) := (others => '0');
    signal p8_nt : signed(12 downto 0) := (others => '0');
    signal p8_dng, p8_rs : std_logic := '0';
    signal p9_g  : signed(11 downto 0) := (others => '0');
    signal p9_dxc : signed(6 downto 0) := (others => '0');
    signal p9_nterm : signed(9 downto 0) := (others => '0');
    signal p9_rs : std_logic := '0';
    signal p10_gn : signed(11 downto 0) := (others => '0');
    signal p10_rs2 : std_logic := '0';
    signal p10_skd : signed(12 downto 0) := (others => '0');
    signal p11_sq : unsigned(15 downto 0) := (others => '0');
    signal p11_m : signed(11 downto 0) := (others => '0');
    signal p12_e2 : signed(22 downto 0) := (others => '0');
    signal p12_m2 : unsigned(9 downto 0) := (others => '0');
    signal p13_er : signed(8 downto 0) := (others => '0');
    signal p13_m2 : unsigned(7 downto 0) := (others => '0');
    signal p14_t : signed(17 downto 0) := (others => '0');
    signal p14_cprev, p14_xtv : std_logic := '0';
    signal p14_xt : unsigned(11 downto 0) := (others => '0');
    signal p15_int : std_logic := '0';
    signal p16_r8 : unsigned(7 downto 0) := (others => '0');
    signal p17_gq : std_logic_vector(31 downto 0) := (others => '0');
    signal p17_rc : unsigned(7 downto 0) := (others => '0');
    signal p18_v : unsigned(11 downto 0) := (others => '0');
    signal p18_ty, p18_tu, p18_tv : unsigned(8 downto 0) := (others => '0');
    signal p19_y, p19_u, p19_v : signed(11 downto 0) := (others => '0');
    signal p20_y, p20_u, p20_v : signed(11 downto 0) := (others => '0');
    signal p21_y, p21_u, p21_v : signed(11 downto 0) := (others => '0');
    signal p21_dy, p21_du, p21_dv : signed(11 downto 0) := (others => '0');
    signal p21_w : unsigned(2 downto 0) := (others => '0');
    signal p22_y, p22_u, p22_v : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- x counters (virtual px, Q8) at stages 13 and 20
    signal x13, x20 : unsigned(19 downto 0) := (others => '0');

    -- delay lines
    type t_u16 is array (natural range <>) of unsigned(15 downto 0);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_s8a is array (natural range <>) of signed(7 downto 0);
    type t_s10 is array (natural range <>) of signed(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal p_hm : t_u16(2 to 4) := (others => (others => '0'));
    signal p_gw : t_s8a(4 to 8) := (others => (others => '0'));
    signal p_ew : t_s10(4 to 11) := (others => (others => '0'));
    signal p_cell : t_sl(10 to 19) := (others => '0');
    signal p_int : t_sl(16 to 17) := (others => '0');
    signal p_pa : t_u10(11 to 17) := (others => (others => '0'));

    -- output conditioning: chroma window and sync rings (block RAM), luma [1 2 1]
    type t_uring is array(0 to 255) of std_logic_vector(19 downto 0);
    type t_sring is array(0 to 255) of std_logic_vector(2 downto 0);
    signal ru_ram : t_uring := (others => "1000000000" & "1000000000");
    signal rs_ram : t_sring := (others => "111");
    attribute no_rw_check : boolean;
    attribute no_rw_check of ru_ram : signal is true;
    attribute no_rw_check of rs_ram : signal is true;
    signal s_wp, ru_ra, rs_ra : unsigned(7 downto 0) := (others => '0');
    signal ru_q, ru_q2 : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal rs_q, rs_q2 : std_logic_vector(2 downto 0) := "111";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y23, s_y24, s_y25, s_y26, s_y27, s_y28x, s_yf28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29, s_y30, s_y31, s_y32 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29, s_u30, s_v30, s_u31, s_v31, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered address and output) and the conditioning rings
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_tq <= C_TEAR(to_integer(r_ta));
            r_sq <= C_S(to_integer(r_sa));
            r_qq <= C_SQ(to_integer(r_qa));
            r_py <= C_PY(to_integer(r_pa)); r_pu <= C_PU(to_integer(r_pa)); r_pv <= C_PV(to_integer(r_pa));
            r_rq <= C_RED(to_integer(r_ra));
            r_gq <= C_GLOW(to_integer(r_ga));
            z_q  <= C_NZ(to_integer(z_ad));
            r_vq <= C_V(to_integer(n_va));
            r_p12 <= C_P12(to_integer(k_hi));
        end if;
    end process p_rom;

    p_uring : process(clk)
    begin
        if rising_edge(clk) then
            ru_ram(to_integer(s_wp)) <= std_logic_vector(p22_u) & std_logic_vector(p22_v);
            ru_q <= ru_ram(to_integer(ru_ra));
            ru_q2 <= ru_q;
        end if;
    end process p_uring;

    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            rs_ram(to_integer(s_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            rs_q <= rs_ram(to_integer(rs_ra));
            rs_q2 <= rs_q;
        end if;
    end process p_sring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-frame sequencer (one serial multiplier + divider)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_id : integer range 0 to 15;
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(20 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_pr : unsigned(37 downto 0);
        variable v_r  : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            v_id := to_integer(unsigned(registers_in(8)(3 downto 0)));
            t_rw <= unsigned(C_T_RW(v_id)); t_sx <= unsigned(C_T_SX(v_id));
            t_ta <= unsigned(C_T_TA(v_id)); t_sd <= C_T_SD(v_id)(0);
            f_tvk <= unsigned(C_T_VK(v_id)); f_nsy <= unsigned(C_T_NSY(v_id));
            f_kax <= unsigned(C_T_KAX(v_id)); f_kby <= unsigned(C_T_KBY(v_id));

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
            g_wm1 <= s_w - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K4 Speed: bipolar, deadband 8 (magnitude squared on the multiplier, slot 8)
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 7 then v_u := unsigned(v_x) - 8; s_n4 <= '0';
                elsif v_x < -7 then v_u := unsigned(-v_x) - 8; s_n4 <= '1';
                else v_u := (others => '0'); s_n4 <= '0'; end if;
                s_m4 <= v_u(8 downto 0);
                f_seam  <= registers_in(6)(0);
                f_noise <= registers_in(6)(1);
                f_ember <= registers_in(6)(2);
                f_flip  <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                v_r := unsigned(shift_left(resize(f_spd, 24), 8));
                f_roll <= f_roll + v_r;
                f_pa0 <= f_pa0 + 300; f_pb0 <= f_pb0 - 200;
                f_fr <= f_fr + 1;
                -- Halo: glow scale, interior threshold
                f_s <= unsigned(r_p12(27 downto 18)); f_kv <= r_p12(17); f_tk <= signed(r_p12(16 downto 0));
            end if;
            f_sk <= resize(shift_right(f_s, 2), 9) + resize(shift_right(f_s, 3), 9) + resize(shift_right(f_s, 5), 9)
                    + resize(shift_right(f_s, 6), 9);

            -- glides: 1/8 of the gap per field (deadband 4); one channel per four zone clocks (4..27)
            if s_zone = '1' and s_zc >= 4 and s_zc < 28 then q_gon <= '1'; else q_gon <= '0'; end if;
            v_s := resize(s_zc(4 downto 2), 21) - 1;
            q_gi <= v_s(2 downto 0); q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when 4 => gl_k <= unsigned(registers_in(5)) & "000"; gl_o <= s_g(4);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(5);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when 4 => s_g(4) <= gl_n;
                                when others => s_g(5) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;
            -- knob curves (shift-add, settle long before the sequencer reads them)
            k_rq <= to_unsigned(1023, 10) - s_g(0)(12 downto 3);
            v_k := s_g(1)(12 downto 3);
            k_cells <= to_unsigned(256, 11) + resize(shift_right(v_k, 1), 11) + resize(shift_right(v_k, 2), 11);
            v_k := s_g(2)(12 downto 3);
            k_pinch <= resize(shift_right(v_k, 3), 8) + resize(shift_right(v_k, 4), 8) + resize(shift_right(v_k, 6), 8);
            k_tear  <= s_g(3)(12 downto 4);
            k_warp  <= s_g(4)(12 downto 7);
            k_hi    <= s_g(5)(12 downto 5);

            -- serial products: slot p = zc/32 (2..10), phase 0 load, 1..18 step, 20 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 2 and v_p <= 10 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 2 and v_p <= 10 and v_ph >= 1 and v_ph <= 18 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 9 loop
                if s_zone = '1' and v_p = k + 1 and v_ph = 20 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 2 => m_a <= resize(k_cells, 20);   m_b <= resize(t_rw, 18);
                    when 3 => m_a <= f_hinc;                m_b <= resize(s_w(11 downto 1), 18);
                    when 4 => m_a <= resize(k_cells, 20);   m_b <= resize(f_tvk, 18);
                    when 5 => m_a <= resize(k_cells, 20);   m_b <= to_unsigned(32270, 18);
                    when 6 => m_a <= resize(k_tear, 20);    m_b <= resize(k_cells, 18);
                    when 7 => m_a <= resize(k_rq, 20);      m_b <= resize(k_rq, 18);
                    when 8 => m_a <= resize(f_kq, 20);      m_b <= to_unsigned(145, 18);
                    when 9 => m_a <= resize(f_kx, 20);      m_b <= to_unsigned(518, 18);
                    when others => m_a <= resize(s_m4, 20); m_b <= resize(s_m4, 18);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(17 downto 1);
                m_b <= '0' & m_b(17 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1) = '1' then f_hinc <= v_pr(27 downto 8); end if;
            if q_wr(2) = '1' then f_h0 <= C_PHC - v_pr(23 downto 0); end if;
            if q_wr(3) = '1' then f_vinc <= v_pr(27 downto 8); end if;
            if q_wr(4) = '1' then f_v0 <= C_PVC + f_roll - v_pr(23 downto 0); end if;
            if q_wr(5) = '1' then f_tamp <= v_pr(18 downto 8); end if;
            if q_wr(6) = '1' then f_kq <= v_pr(19 downto 4); end if;
            if q_wr(7) = '1' then f_kx <= to_unsigned(988, 12) + resize(v_pr(23 downto 12), 12); end if;
            if q_wr(8) = '1' then
                f_ky <= v_pr(21 downto 10);
                if t_sd = '1' then
                    f_kx <= f_kx + resize(shift_right(f_kx, 2), 12) + resize(shift_right(f_kx, 4), 12) + resize(shift_right(f_kx, 6), 12);
                end if;
            end if;
            if q_wr(9) = '1' then
                if s_n4 = '1' then f_spd <= -signed(resize(v_pr(17 downto 6), 16));
                else f_spd <= signed(resize(v_pr(17 downto 6), 16)); end if;
            end if;

            -- rings: write every clock; chroma window read 8 back, syncs C_LAT back
            s_wp <= s_wp + 1;
            ru_ra <= s_wp - 5;
            rs_ra <= s_wp - (C_LAT - 3);
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
    -- Per-line engine: kicked at a line's g_start (line 0 late in the vblank
    -- zone), computes the NEXT line; committed at the next g_start.  One serial
    -- multiplier (16 steps per product), ~130 clocks.
    --   1 advance, 2 fold, 3 V address / cusp distance, 4 strip + P1 load,
    --   22 P1 store (stretch amplitude) + P2 load (V * pinch), 40 P2 store + P3 load (a * tamp),
    --   58 P3 store + P4 load (* tear factor), 76 P4 store (amp) + P5 load (|dy| * ky),
    --   94 P5 store (dyn) + P6 load (dyn^2), 112 P6 store + P7 load (|jitter| * s), 130 P7 store,
    --   131..134 the white band's noise lattice (rows cy, cy + 1), 136 P8 load (lerp by fy), 154 P8 store.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_fv : unsigned(15 downto 0);
        variable v_cu : unsigned(15 downto 0);
        variable v_d  : unsigned(15 downto 0);
        variable v_s  : signed(15 downto 0);
        variable v_ms : unsigned(19 downto 0);
        variable v_pr : unsigned(34 downto 0);
        variable v_j  : signed(17 downto 0);
        variable v_n  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            r_vq2 <= r_vq;
            z_q2 <= z_q;
            if s_zone = '1' and s_zc = 700 then
                n_v24 <= f_v0; n_ra <= (others => '0'); n_pb <= f_pb0;
                n_lf <= x"ACE1"; n_m9 <= to_unsigned(8, 4);
                le_first <= '1'; le_st <= to_unsigned(1, 8);
            elsif g_start = '1' then
                le_first <= '0'; le_st <= to_unsigned(1, 8);
            elsif le_st /= 0 then
                if le_st = 154 then le_st <= (others => '0'); else le_st <= le_st + 1; end if;
            end if;

            if lm_on = '1' then
                if lm_b(0) = '1' then v_ms := resize(lm_h, 20) + resize(lm_a, 20); else v_ms := resize(lm_h, 20); end if;
                lm_h <= v_ms(19 downto 1);
                lm_l <= v_ms(0) & lm_l(15 downto 1);
                lm_b <= '0' & lm_b(15 downto 1);
            end if;
            v_pr := lm_h & lm_l;

            case to_integer(le_st) is
                when 1 =>
                    if le_first = '0' then
                        n_v24 <= n_v24 + resize(f_vinc, 24);
                        n_ra <= n_ra + resize(f_nsy, 24);
                        n_pb <= n_pb + resize(f_kby, 16);
                    end if;
                    n_lf <= ('0' & n_lf(15 downto 1)) xor (x"B400" and (15 downto 0 => n_lf(0)));
                    if n_m9 = 8 then n_m9 <= (others => '0'); else n_m9 <= n_m9 + 1; end if;
                when 2 =>
                    v_fv := n_v24(23 downto 8);
                    if v_fv(15) = '1' then v_fv := 0 - v_fv; end if;
                    n_fv <= v_fv;
                    if n_v24(23) = '1' then n_dy <= 0 - n_v24(23 downto 8); else n_dy <= n_v24(23 downto 8); end if;
                when 3 =>
                    if n_fv(15) = '1' then n_va <= (others => '1'); else n_va <= n_fv(14 downto 7); end if;
                    n_cu <= to_unsigned(32768, 16) - n_fv;
                when 4 =>
                    v_cu := n_cu;
                    if v_cu < 3748 then n_ty <= "10"; n_shp <= '0'; n_a <= to_unsigned(170, 8);
                    elsif v_cu >= 4715 and v_cu < 9914 then n_ty <= "00"; n_shp <= '1'; n_a <= to_unsigned(115, 8);
                    elsif v_cu >= 10518 and v_cu < 14992 then n_ty <= "01"; n_shp <= '0'; n_a <= to_unsigned(150, 8);
                    else n_ty <= "00"; n_shp <= '0'; n_a <= (others => '0'); end if;
                    lm_a <= resize(v_cu - 4715, 19); lm_b <= to_unsigned(693, 16);
                    lm_h <= (others => '0'); lm_l <= (others => '0'); lm_on <= '1';
                when 20 | 38 | 56 | 74 | 92 | 110 | 128 | 152 =>
                    lm_on <= '0';
                when 22 =>
                    if n_shp = '1' then
                        v_d := v_pr(31 downto 16);
                        if v_d > 55 then v_d := to_unsigned(55, 16); end if;
                        n_a <= to_unsigned(115, 8) - v_d(7 downto 0);
                    end if;
                    lm_a <= resize(unsigned(r_vq2), 19); lm_b <= resize(k_pinch, 16);
                    lm_h <= (others => '0'); lm_l <= (others => '0'); lm_on <= '1';
                when 40 =>
                    n_vp <= v_pr(15 downto 7);
                    lm_a <= resize(n_a, 19); lm_b <= resize(f_tamp, 16);
                    lm_h <= (others => '0'); lm_l <= (others => '0'); lm_on <= '1';
                when 58 =>
                    lm_a <= v_pr(18 downto 0); lm_b <= resize(t_ta, 16);
                    lm_h <= (others => '0'); lm_l <= (others => '0'); lm_on <= '1';
                when 76 =>
                    n_amp <= v_pr(26 downto 11);
                    lm_a <= resize(n_dy, 19); lm_b <= resize(f_ky, 16);
                    lm_h <= (others => '0'); lm_l <= (others => '0'); lm_on <= '1';
                when 94 =>
                    if v_pr(27 downto 16) > 1023 then v_d := to_unsigned(1023, 16); else v_d := resize(v_pr(27 downto 16), 16); end if;
                    lm_a <= resize(v_d(9 downto 0), 19); lm_b <= resize(v_d(9 downto 0), 16);
                    lm_h <= (others => '0'); lm_l <= (others => '0'); lm_on <= '1';
                when 112 =>
                    n_dyn2 <= v_pr(19 downto 0);
                    if n_kj < 0 then lm_a <= resize(unsigned(-n_kj), 19); else lm_a <= resize(unsigned(n_kj), 19); end if;
                    lm_b <= resize(f_s, 16);
                    lm_h <= (others => '0'); lm_l <= (others => '0'); lm_on <= '1';
                when 130 =>
                    v_j := signed(resize(v_pr(25 downto 8), 18));
                    if n_kj < 0 then n_tkj <= resize(f_tk, 18) - v_j; else n_tkj <= resize(f_tk, 18) + v_j; end if;
                    z_ad <= resize(n_ra(23 downto 16) * to_unsigned(37, 6), 8) + resize(f_fr * to_unsigned(13, 4), 8);
                when 131 =>
                    z_ad <= resize((n_ra(23 downto 16) + 1) * to_unsigned(37, 6), 8) + resize(f_fr * to_unsigned(13, 4), 8);
                when 133 =>
                    n_g0 <= resize(signed(z_q2), 9);
                when 134 =>
                    n_nd <= resize(signed(z_q2), 10) - resize(n_g0, 10);
                when 136 =>
                    -- |d| * fy on the multiplier (sign applied at the store)
                    if n_nd < 0 then lm_a <= resize(unsigned(-n_nd), 19); else lm_a <= resize(unsigned(n_nd), 19); end if;
                    lm_b <= resize(n_ra(15 downto 13), 16);
                    lm_h <= (others => '0'); lm_l <= (others => '0'); lm_on <= '1';
                when 154 =>
                    v_n := signed(resize(v_pr(12 downto 3), 10));
                    if n_nd < 0 then v_n := -v_n; end if;
                    n_nz <= n_g0 + resize(v_n, 9);
                when others => null;
            end case;

            -- key jitter (E2 units): 9-line sawtooth + LFSR
            v_s := to_signed(to_integer(n_m9) * 2 - 8, 16);
            n_kj <= resize(v_s * to_signed(300, 10), 16) + resize((signed(resize(n_lf(7 downto 0), 9)) - 128) * to_signed(10, 5), 16);

            -- commit at the next line start
            if g_start = '1' then
                l_vp <= n_vp; l_dyn2 <= n_dyn2; l_ty <= n_ty; l_shp <= n_shp; l_amp <= n_amp(15 downto 4); l_tkj <= n_tkj;
                l_pb <= n_pb; l_nz <= n_nz;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: H phase, virtual x, noise lattice cell (this / next / after-next
    -- values), Osc phase
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_h <= f_h0; b_xv <= (others => '0'); b_pa <= f_pa0;
            elsif g_avid = '1' then
                b_h <= b_h + resize(f_hinc, 24);
                b_xv <= b_xv + resize(t_sx, 20);
                b_pa <= b_pa + resize(f_kax, 16);
            end if;
        end if;
    end process p_beam;


    ---------------------------------------------------------------------------
    -- Pixel path (stage k = the pixel's register after k clocks; a ROM whose
    -- address is set at stage k has its data at k+1)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_h  : unsigned(23 downto 0);
        variable v_x  : signed(10 downto 0);
        variable v_t  : signed(9 downto 0);
        variable v_d  : signed(20 downto 0);
        variable v_w  : unsigned(15 downto 0);
        variable v_u  : unsigned(15 downto 0);
        variable v_g  : signed(11 downto 0);
        variable v_e  : signed(22 downto 0);
        variable v_i  : signed(21 downto 0);
        variable v_r  : unsigned(11 downto 0);
        variable v_c  : signed(11 downto 0);
        variable v_n  : signed(9 downto 0);
        variable v_nt : signed(9 downto 0);
        variable v_rc : unsigned(7 downto 0);
        variable v_ig : unsigned(5 downto 0);
        variable v_y  : signed(13 downto 0);
        variable v_y2 : unsigned(21 downto 0);
        variable v_fy, v_fu, v_fv : integer range 0 to 1023;
        variable v_q  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: source luma -> signed source value (a few px early: invisible in a bend field)
            v_y1 <= unsigned(data_in.y);
            v_x := signed(resize(v_y1, 11)) - 512;
            v_x := shift_right(v_x, 2);
            if v_x > 127 then v_s8 <= to_signed(127, 8);
            elsif v_x < -127 then v_s8 <= to_signed(-127, 8);
            else v_s8 <= v_x(7 downto 0); end if;

            -- 1: flip, tear ROM address, source
            if f_flip = '1' then v_h := C_2PHC - b_h; else v_h := b_h; end if;
            p1_hm <= v_h(23 downto 8);
            if b_xv(19 downto 18) = "00" then r_ta <= l_shp & b_xv(17 downto 10); else r_ta <= l_shp & x"FF"; end if;
            if f_osc = '1' then
                v_t := shift_right(resize(f_tri8(b_pa), 10) + resize(f_tri8(l_pb), 10), 1);
                p1_s8 <= v_t(7 downto 0);
            else
                p1_s8 <= v_s8;
            end if;

            -- 2: tear ROM (r_tq); source * warp
            p2_ws <= resize(resize(shift_right(p1_s8, 2), 6) * signed('0' & k_warp), 13);

            -- 3: tear shape; warp terms
            p3_sh <= signed(r_tq);
            p3_gw <= resize(shift_right(p2_ws, 5) + shift_right(p2_ws, 7), 8);
            p3_ew <= resize(shift_right(p2_ws, 3), 10);

            -- 4: tear product (shape * amp / 64)
            p4_d <= resize(p3_sh * signed('0' & l_amp(11 downto 2)), 19);

            -- 5: torn H phase (displacement = shape * amp / 128)
            v_d := resize(shift_right(p4_d, 1), 21);
            if f_flip = '1' then p5_hb <= p_hm(4) - unsigned(v_d(15 downto 0));
            else p5_hb <= p_hm(4) + unsigned(v_d(15 downto 0)); end if;

            -- 6: S ROM address (fold from the seam), ring |dx| / 8, cut side and noise fade
            if p5_hb(15) = '1' then v_w := 0 - p5_hb; else v_w := p5_hb; end if;
            if v_w(15) = '1' then r_sa <= (others => '1'); else r_sa <= v_w(14 downto 6); end if;
            v_u := p5_hb - C_HRING;
            if v_u(15) = '1' then
                v_w := 0 - v_u; p6_dng <= '1';
                if v_w(15) = '1' then p6_ax <= (others => '1'); else p6_ax <= v_w(14 downto 6); end if;
            else
                p6_ax <= v_u(14 downto 6); p6_dng <= '0';
            end if;
            v_u := p5_hb - C_CUT;
            if v_u(15) = '0' and f_seam = '0' then p6_rs <= '1'; else p6_rs <= '0'; end if;
            if v_u(14 downto 13) /= "00" then p6_fd <= (others => '0'); else p6_fd <= 7 - v_u(12 downto 10); end if;

            -- 7: S ROM (r_sq); |dx| * kx
            p7_m <= p6_ax * f_kx(11 downto 3);
            p7_dng <= p6_dng; p7_rs <= p6_rs; p7_fd <= p6_fd;

            -- 8: S value; |dxn| (Q8, saturated 1023); noise * fade
            p8_s <= signed(resize(unsigned(r_sq), 11)) - 160;
            if p7_m(17) = '1' then p8_adn <= (others => '1'); else p8_adn <= p7_m(16 downto 7); end if;
            p8_nt <= resize(l_nz * signed('0' & p7_fd), 13);
            p8_dng <= p7_dng; p8_rs <= p7_rs;

            -- 9: G = S + V * pinch + warp; square ROM address; signed dxn / 8 (clamped +-48); noise term
            p9_g <= resize(p8_s, 12) + signed(resize(l_vp, 12)) + resize(p_gw(8), 12);
            r_qa <= p8_adn(9 downto 1);
            if p8_adn > 384 then v_n := to_signed(48, 10); else v_n := signed(resize(p8_adn(9 downto 3), 10)); end if;
            if p8_dng = '1' then p9_dxc <= resize(-v_n, 7); else p9_dxc <= resize(v_n, 7); end if;
            v_nt := resize(shift_right(p8_nt, 3), 10);
            p9_nterm <= shift_right(v_nt, 1);
            p9_rs <= p8_rs;

            -- 10: cell key; noisy G (right of the cut, below the diamonds); glow-scale skew product
            if p9_g < 58 then p_cell(10) <= '1'; else p_cell(10) <= '0'; end if;
            if p9_rs = '1' and p9_g < 300 then
                p10_rs2 <= '1';
                if f_noise = '0' then p10_gn <= p9_g + resize(p9_nterm, 12); else p10_gn <= p9_g; end if;
            else
                p10_rs2 <= '0'; p10_gn <= p9_g;
            end if;
            p10_skd <= resize(signed('0' & f_sk(8 downto 3)) * resize(shift_right(p9_dxc, 1), 6), 13);

            -- 11: palette address; dxn^2 (fabric copy); glow scale m
            v_g := p10_gn - 32;
            if v_g < 0 then v_g := (others => '0'); elsif v_g > 511 then v_g := to_signed(511, 12); end if;
            p_pa(11) <= p10_rs2 & unsigned(v_g(8 downto 0));
            p11_sq <= unsigned(r_qq);
            p11_m <= signed(resize(f_s, 12)) - resize(shift_right(p10_skd, 1), 12);

            -- 12: E2 = dx^2 + dy^2 + warp
            p12_e2 <= signed(resize(shift_left(resize(p11_sq, 20), 4), 23)) + signed(resize(l_dyn2, 23))
                      + resize(shift_left(resize(p_ew(11), 16), 6), 23);
            if p11_m < 0 then p12_m2 <= (others => '0'); else p12_m2 <= unsigned(p11_m(10 downto 1)); end if;

            -- 13: e = (E2 - 1) / 128, clamped
            v_e := p12_e2 - 65536;
            v_e := shift_right(v_e, 9);
            if v_e > 255 then p13_er <= to_signed(255, 9);
            elsif v_e < -255 then p13_er <= to_signed(-255, 9);
            else p13_er <= v_e(8 downto 0); end if;
            p13_m2 <= p12_m2(9 downto 2);

            -- 14: glow index product t = e * m; red one-shot index (fired by the cell key's leading edge)
            p14_t <= resize(p13_er * signed('0' & p13_m2), 18);
            if p_cell(13) = '1' and p14_cprev = '0' then
                p14_xt <= x13(19 downto 8); p14_xtv <= '1'; r_ra <= (others => '0');
            else
                v_r := x13(19 downto 8) - p14_xt;
                if f_ember = '1' then v_r := shift_right(v_r, 3); else v_r := shift_right(v_r, 1); end if;
                if p14_xtv = '0' or v_r > 255 then r_ra <= (others => '1'); else r_ra <= v_r(7 downto 0); end if;
            end if;
            p14_cprev <= p_cell(13) and s_gav_sr(12);
            if s_gav_sr(12) = '0' then p14_xtv <= '0'; end if;

            -- 15: glow ROM address; interior key
            v_i := shift_right(resize(p14_t, 22) + 4096, 5);
            if v_i < 0 then r_ga <= (others => '0'); elsif v_i > 511 then r_ga <= (others => '1');
            else r_ga <= unsigned(v_i(8 downto 0)); end if;
            if f_kv = '1' and resize(p14_t, 19) < resize(shift_right(l_tkj, 4), 19) then p15_int <= '1'; else p15_int <= '0'; end if;

            -- 16: red level (fabric copy)
            p16_r8 <= unsigned(r_rq);
            p_int(16) <= p15_int;

            -- 17: glow word (fabric copy); interior red
            p17_gq <= r_gq;
            if p_int(16) = '1' then v_rc := to_unsigned(38, 8) + shift_right(p16_r8 - 36, 2); else v_rc := p16_r8; end if;
            p17_rc <= v_rc;
            p_int(17) <= p_int(16);

            -- 18: red * (1 - glow); the glow colour (or the interior's); palette address
            if p_int(17) = '1' then
                v_ig := to_unsigned(32, 6);
                p18_ty <= to_unsigned(102, 9); p18_tu <= to_unsigned(215, 9); p18_tv <= to_unsigned(224, 9);
            else
                v_ig := resize(unsigned(p17_gq(4 downto 0)), 6);
                p18_ty <= unsigned(p17_gq(31 downto 23)); p18_tu <= unsigned(p17_gq(22 downto 14)); p18_tv <= unsigned(p17_gq(13 downto 5));
            end if;
            p18_v <= p17_rc(7 downto 2) * v_ig;
            r_pa <= p_pa(17);

            -- 19: cell colour = 2T + red * (1 - glow) * k
            v_y2 := resize(p18_v * to_unsigned(81, 8), 22);
            p19_y <= signed(resize(p18_ty & '0', 12)) + signed(resize(v_y2(21 downto 9), 12));
            v_y2 := resize(p18_v * to_unsigned(225, 8), 22);
            p19_u <= signed(resize(p18_tu & '0', 12)) + signed(resize(v_y2(21 downto 10), 12));
            v_y2 := resize(p18_v * to_unsigned(19, 8), 22);
            p19_v <= signed(resize(p18_tv & '0', 12)) - signed(resize(v_y2(21 downto 8), 12));

            -- 20: luma soft knee; cell layer or the band palette (EBR output, x4)
            if p_cell(19) = '1' then
                if p19_y > 822 then
                    v_q := p19_y - 822;
                    p20_y <= to_signed(822, 12) + shift_right(v_q, 2) + shift_right(v_q, 3);
                else
                    p20_y <= p19_y;
                end if;
                p20_u <= p19_u; p20_v <= p19_v;
            else
                p20_y <= signed(shift_left(resize(unsigned(r_py), 12), 2));
                p20_u <= signed(shift_left(resize(unsigned(r_pu), 12), 2));
                p20_v <= signed(shift_left(resize(unsigned(r_pv), 12), 2));
            end if;

            -- 21: line-start flare: colour - c, weight in quarters
            case to_integer(l_ty) is
                when 1 => v_fy := 261; v_fu := 506; v_fv := 428;
                when 2 => v_fy := 383; v_fu := 423; v_fv := 729;
                when others => v_fy := 867; v_fu := 459; v_fv := 532;
            end case;
            p21_y <= p20_y; p21_u <= p20_u; p21_v <= p20_v;
            p21_dy <= to_signed(v_fy, 12) - p20_y;
            p21_du <= to_signed(v_fu, 12) - p20_u;
            p21_dv <= to_signed(v_fv, 12) - p20_v;
            if x20(19 downto 14) = 0 then p21_w <= unsigned(C_FL(to_integer(x20(13 downto 10)))); else p21_w <= (others => '0'); end if;

            -- 22: flare blend (between two in-range colours: no clamp)
            for k in 0 to 2 loop
                if k = 0 then v_c := p21_y; v_q := p21_dy;
                elsif k = 1 then v_c := p21_u; v_q := p21_du;
                else v_c := p21_v; v_q := p21_dv; end if;
                case to_integer(p21_w) is
                    when 1 => v_c := v_c + shift_right(v_q, 2);
                    when 2 => v_c := v_c + shift_right(v_q, 1);
                    when 3 => v_c := v_c + shift_right(v_q, 1) + shift_right(v_q, 2);
                    when 4 => v_c := v_c + v_q;
                    when others => null;
                end case;
                if k = 0 then p22_y <= unsigned(v_c(9 downto 0));
                elsif k = 1 then p22_u <= unsigned(v_c(9 downto 0));
                else p22_v <= unsigned(v_c(9 downto 0)); end if;
            end loop;

            -- x counters: the virtual x of the stage-13 and stage-20 pixels
            if s_gav_sr(12) = '1' then x13 <= x13 + resize(t_sx, 20); else x13 <= (others => '0'); end if;
            if s_gav_sr(19) = '1' then x20 <= x20 + resize(t_sx, 20); else x20 <= (others => '0'); end if;

            -- 23..30: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent
            -- 8-sums over the ring window, luma [1 2 1]); the heirloom shell's stages, offset +10
            s_boxu <= s_boxu + p22_u - unsigned(ru_q2(19 downto 10));
            s_boxv <= s_boxv + p22_v - unsigned(ru_q2(9 downto 0));
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            s_u28 <= resize(shift_right(resize(s_boxu, 15) + s_boxud, 4), 10);
            s_v28 <= resize(shift_right(resize(s_boxv, 15) + s_boxvd, 4), 10);
            s_y23 <= p22_y; s_y24 <= s_y23; s_y25 <= s_y24; s_y26 <= s_y25;
            s_y27 <= s_y26; s_y28x <= s_y27;
            s_yf28 <= resize(shift_right(resize(s_y26, 12) + shift_left(resize(s_y27, 12), 1) + resize(s_y28x, 12), 2), 10);
            -- 29: luma toward black (white 940 -> 777); blanking gate
            if s_gav_sr(27) = '1' then
                if s_yf28 > 64 then
                    s_y29 <= s_yf28 - shift_right(s_yf28 - 64, 3) - shift_right(s_yf28 - 64, 4);
                else
                    s_y29 <= s_yf28;
                end if;
                s_u29 <= s_u28; s_v29 <= s_v28;
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
            -- 30: output register; 31..32: pad to the 32-clock latency
            s_y30 <= s_y29; s_u30 <= s_u29; s_v30 <= s_v29;
            s_y31 <= s_y30; s_u31 <= s_u30; s_v31 <= s_v30;
            s_y32 <= s_y31; s_u32 <= s_u31; s_v32 <= s_v31;

            -- delays
            p_hm(2) <= p1_hm; for i in 3 to 4 loop p_hm(i) <= p_hm(i - 1); end loop;
            p_gw(4) <= p3_gw; for i in 5 to 8 loop p_gw(i) <= p_gw(i - 1); end loop;
            p_ew(4) <= p3_ew; for i in 5 to 11 loop p_ew(i) <= p_ew(i - 1); end loop;
            for i in 11 to 19 loop p_cell(i) <= p_cell(i - 1); end loop;
            for i in 12 to 17 loop p_pa(i) <= p_pa(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y32);
    data_out.u       <= std_logic_vector(s_u32);
    data_out.v       <= std_logic_vector(s_v32);
    data_out.hsync_n <= rs_q2(2);
    data_out.vsync_n <= rs_q2(1);
    data_out.field_n <= rs_q2(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture halo;
