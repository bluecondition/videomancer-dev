-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Mummer v0.1.1 -- Heirloom from the video clips (phase 3b), made from the still WIN_20230804_22_27_52 @ 6.0 s.
--
-- The still: two green mask faces (axes x 550 and 1483: one horizontal period of 933 px), each a wide lens with
-- pointed cheeks over a narrower chin, fluted by soft vertical stripes (period 109.5 px) and glowing cyan in the
-- middle, outlined dark green / black / blue / violet; a busy red / crimson / magenta / violet ground cut by
-- zigzag lightning and thin cyan and white lines; nested blue chevrons over each face, a red band across the top
-- of the faces, green lobes at the top and green blobs between the faces at the bottom.  In the clip the whole
-- structure rolls DOWN (about 230 lines/s): the faces narrow as they descend and green comes in from the top.
--
-- The patch, as fitted (mummer.py has every table):
--   F      = V + g T1 + g2 T2 - |g| a R w(F0) + Etch * luma        (F_hw = F * 4096 / 1680)
--   T1     = the face triangle oscillator (hn: 0 on a face axis .. 1023 between faces), T2 = the same at twice the rate
--   R      = an independent stripes oscillator; outside the green its phase jitters per line (the lightning)
--   V, g, g2 = a rolling vertical oscillator (scroll table, d = (Y - s) mod 1400 lines) plus a screen-fixed
--            ramp and gain, so the default frame at s = 0 is the still
--   colour = palette(F), a triangle-folding colouriser (1024 entries per fold period; Gold = a second palette)
--
--   K1  Faces      face count: one wide face (0) .. the clip's two (512) .. six (1023)
--   K2  Stripes    the vertical stripes' depth: smooth (0) .. the clip (512) .. deep fluting (1023)
--   K3  Lightning  the zigzag jitter: straight streaks (0) .. the clip (512) .. lightning through the outlines (1023)
--   K4  Roll       bipolar roll speed, +-8 deadband; 700 = the clip (down)
--   K5  Bands      colouriser fold density x0.44 .. x2.3 about the green edge
--   K6  Etch       how far the camera (or the Osc lattice) bends the field
--   S7  Twin       two rows of faces (the vertical oscillator at twice the rate)
--   S8  Colours    Mask (the clip) / Gold
--   S9  Mirror     the lower half is the upper half mirrored (a totem)
--   S10 Tear       sync slips: 64-line bands shifted sideways, rerolled every 48 fields
--   S11 Source     Video / Osc
--   P12 MASK       the faces' level and depth: small faces in a busy ground (0) .. the clip (512) .. swollen
--                  faces with ringed cores (1023)
--
-- v0.1.1: the line engine reads the scroll table at the line's fractional position (64 sub-steps per entry) and
-- carries g and g2 at Q11, the bridge is 320 lines and the screen gain 0.6, and the palette's thin lines have soft
-- luma ramps: no single-row lines at the defaults or at P12 0 / 100 %.
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: the scroll-table read with interpolation, the screen ramp / gain, V, g, g2, the
-- stripes gain, the lightning jitter, the tear offset, the two oscillators' start phases), a 17-stage pixel path.
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (chroma
-- [1 2 2 2 2 2 2 2 1]/16 with the running-sum resync, luma [1 2 1], luma 13/16) before the blanking gate.  Colours
-- in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.mummer_pkg.all;
use work.all;

architecture mummer of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_PIV  : integer := 853;        -- the green edge (F_hw)
    constant C_POFF : integer := 488;        -- palette address offset (F = -200 -> entry 0)
    constant C_RST1 : integer := 76608;      -- stripes phase per h (2^24 per 219 h)
    constant C_STH0 : integer := 8991;       -- face phase per h at the still (2^24 per 1866 h)
    constant C_PC   : integer := 7363398;    -- face phase at the screen centre (h 1920)

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

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sc_a : unsigned(7 downto 0) := (others => '0');
    signal r_sc_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_th_a : unsigned(7 downto 0) := (others => '0');
    signal r_th_q : signed(10 downto 0) := (others => '0');
    signal r_cz_a, r_c0_a : unsigned(7 downto 0) := (others => '0');
    signal r_cz_q, r_c0_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_p_a  : unsigned(10 downto 0) := (others => '0');
    signal r_p_q  : std_logic_vector(23 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), raw roll, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(140 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_gold, s_mirror, s_tear, s_src : std_logic := '0';

    -- per-field constants
    signal z_a    : unsigned(9 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_q    : unsigned(20 downto 0) := (others => '0');
    signal z_vsq  : unsigned(17 downto 0) := (others => '0');
    signal z_offd : unsigned(9 downto 0) := (others => '0');
    signal z_offn : std_logic := '0';
    signal c_sth1 : unsigned(15 downto 0) := to_unsigned(C_STH0, 16);  -- face phase per h
    signal c_step : unsigned(17 downto 0) := to_unsigned(17982, 18);   -- per pixel
    signal c_pst  : unsigned(23 downto 0) := (others => '0');          -- face phase at the line's first pixel
    signal c_xb0  : unsigned(11 downto 0) := (others => '0');          -- beam start (h)
    signal c_kr   : unsigned(13 downto 0) := to_unsigned(4096, 14);    -- Bands, Q12
    signal c_kv   : unsigned(13 downto 0) := to_unsigned(4096, 14);    -- Mask depth, Q12
    signal c_kvr  : unsigned(15 downto 0) := to_unsigned(4096, 16);    -- kv kr, Q12
    signal c_off  : signed(10 downto 0) := (others => '0');            -- Mask level (F_hw)
    signal c_ka   : unsigned(15 downto 0) := to_unsigned(16842, 16);   -- stripes gain (Q8 of 65.8 F_hw per g unit)
    signal c_za   : unsigned(14 downto 0) := to_unsigned(9216, 15);    -- lightning depth (Q8 h)
    signal c_fj   : unsigned(7 downto 0) := (others => '0');           -- face jitter share (Q8)
    signal c_eg   : unsigned(9 downto 0) := (others => '0');           -- etch per luma step (Q8 F_hw)
    signal c_vst  : unsigned(15 downto 0) := (others => '0');          -- roll per field (Q8 D10 units)
    signal c_s    : unsigned(17 downto 0) := (others => '0');          -- roll phase (Q8 D10, mod 1024)
    signal c_rstep : unsigned(19 downto 0) := to_unsigned(153216, 20); -- stripes phase per pixel
    signal c_rpst : unsigned(23 downto 0) := (others => '0');          -- stripes phase at the line's first pixel
    signal c_z1i  : unsigned(15 downto 0) := to_unsigned(2048, 16);    -- lightning phase steps per line (Q16 cycles)
    signal c_z2i  : unsigned(15 downto 0) := to_unsigned(1394, 16);
    signal c_z3i  : unsigned(15 downto 0) := to_unsigned(379, 16);
    signal c_tz1, c_tz2, c_tz3 : unsigned(15 downto 0) := (others => '0');  -- lightning phases at line 0
    signal t_fc   : unsigned(5 downto 0) := (others => '0');           -- fields since the last tear reroll
    signal c_tb   : unsigned(10 downto 0) := (others => '0');          -- tear band phase (lines)
    signal c_so   : unsigned(7 downto 0) := (others => '0');           -- tear table offset

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(17 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(17 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(8 downto 0) := to_unsigned(511, 9);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_yi   : unsigned(10 downto 0) := (others => '0');
    signal e_ymq  : unsigned(18 downto 0) := (others => '0');   -- mirrored (S9)
    signal e_dyq  : signed(19 downto 0) := (others => '0');     -- Ym - 540, Q8
    signal e_tbs  : unsigned(11 downto 0) := (others => '0');
    signal e_t1, e_t2, e_t3 : unsigned(15 downto 0) := (others => '0');
    signal e_s    : signed(9 downto 0) := (others => '0');
    signal e_dq   : unsigned(17 downto 0) := (others => '0');   -- scroll-table position: 1024 per LW, Q8
    signal e_f6   : unsigned(5 downto 0) := (others => '0');
    signal e_scA, e_scB : std_logic_vector(31 downto 0) := (others => '0');
    signal e_dv   : signed(13 downto 0) := (others => '0');
    signal e_dg   : signed(11 downto 0) := (others => '0');
    signal e_d2   : signed(8 downto 0) := (others => '0');
    signal e_v2   : signed(15 downto 0) := (others => '0');     -- 2 Vs (F_hw)
    signal e_gs   : signed(16 downto 0) := (others => '0');     -- gs, Q12
    signal e_g2s  : signed(13 downto 0) := (others => '0');     -- g2s, Q11
    signal e_p    : signed(11 downto 0) := (others => '0');
    signal e_gf   : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal e_v    : signed(15 downto 0) := (others => '0');
    signal e_x    : signed(15 downto 0) := (others => '0');
    signal e_m2   : unsigned(15 downto 0) := (others => '0');
    signal e_zig  : signed(9 downto 0) := (others => '0');
    signal e_th   : signed(10 downto 0) := (others => '0');
    signal e_off  : signed(11 downto 0) := (others => '0');
    signal m_a    : unsigned(29 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(29 downto 0) := (others => '0');
    -- next-line results
    signal n_v    : unsigned(11 downto 0) := (others => '0');    -- V (F_hw mod 4096: the palette and the depth table read F mod 4096)
    signal n_gn   : std_logic := '0';
    signal n_g2   : unsigned(13 downto 0) := (others => '0');   -- |g2|, Q11
    signal n_g2n  : std_logic := '0';
    signal e_gss  : signed(17 downto 0) := (others => '0');     -- g, Q11 (signed)
    signal e_g2d  : signed(17 downto 0) := (others => '0');     -- 2 g2, Q11 (signed)
    signal e_cb   : signed(15 downto 0) := (others => '0');     -- 2046 g2 >> 11 (F_hw)
    signal n_gnear, n_gfar : unsigned(16 downto 0) := (others => '0'); -- |g +- 2 g2|, Q11
    signal n_snear, n_sfar : std_logic := '0';
    signal n_vb   : unsigned(11 downto 0) := (others => '0');   -- V + 2046 g2 (the far half of the triangle)
    signal n_ga   : unsigned(11 downto 0) := (others => '0');
    signal n_p0   : unsigned(23 downto 0) := (others => '0');
    signal n_rp0  : unsigned(23 downto 0) := (others => '0');
    signal n_zo   : unsigned(7 downto 0) := (others => '0');

    -- beam / per-line values in use
    signal b_p   : unsigned(23 downto 0) := (others => '0');
    signal b_r   : unsigned(23 downto 0) := (others => '0');
    signal l_v   : unsigned(11 downto 0) := (others => '0');
    signal l_gnear, l_gfar : unsigned(16 downto 0) := (others => '0');
    signal l_snear, l_sfar : std_logic := '0';
    signal l_vb  : unsigned(11 downto 0) := (others => '0');
    signal l_ga  : unsigned(11 downto 0) := (others => '0');
    signal l_zo  : unsigned(7 downto 0) := (others => '0');
    signal l_step  : unsigned(17 downto 0) := (others => '0');
    signal l_rstep : unsigned(19 downto 0) := (others => '0');
    -- oscillator source (Source = Osc): a drifting 2-D triangle lattice, 240 x 180 virtual px
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(273, 10);
    signal s_oys : unsigned(11 downto 0) := to_unsigned(364, 12);

    -- pixel path
    signal q2_hn  : unsigned(9 downto 0) := (others => '0');
    signal q2_gm  : unsigned(16 downto 0) := (others => '0');
    signal q2_sg, q3_sg, q4_sg, q2_hi, q3_hi, q4_hi, q5_hi : std_logic := '0';
    signal q2_lm, q3_lm : unsigned(7 downto 0) := (others => '0');
    signal q3_gh, q3_gl : unsigned(21 downto 0) := (others => '0');
    signal q4_gt  : unsigned(26 downto 0) := (others => '0');
    signal q4_el, q4_eh : unsigned(13 downto 0) := (others => '0');
    signal q2_az, q3_az, q4_az, q5_az, q6_az : unsigned(7 downto 0) := (others => '0');
    signal q9_cz, q9_c0 : signed(7 downto 0) := (others => '0');
    signal q5_gt  : unsigned(11 downto 0) := (others => '0');
    signal q5_et  : unsigned(9 downto 0) := (others => '0');
    signal q6_f1, q6_f2 : unsigned(11 downto 0) := (others => '0');
    signal q7_f0, q8_f0, q9_f0, q10_f0, q11_f0, q12_f0 : unsigned(11 downto 0) := (others => '0');
    signal q8_wi  : unsigned(5 downto 0) := (others => '0');
    signal q9_w   : std_logic_vector(4 downto 0) := (others => '0');
    signal q10_rw : unsigned(9 downto 0) := (others => '0');
    signal q10_rn, q11_rn, q12_rn : std_logic := '0';
    signal q11_ph, q11_pl : unsigned(16 downto 0) := (others => '0');
    signal q12_rip : unsigned(12 downto 0) := (others => '0');
    signal q13_f  : unsigned(11 downto 0) := (others => '0');
    signal q16_p  : std_logic_vector(23 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_MU_EXP2(to_integer(r_ex_a));
            r_sc_q <= C_MU_SC(to_integer(r_sc_a));
            r_th_q <= C_MU_TH(to_integer(r_th_a));
            r_cz_q <= C_MU_COS(to_integer(r_cz_a));
            r_c0_q <= C_MU_COS(to_integer(r_c0_a));
            r_p_q  <= C_MU_PAL(to_integer(r_p_a));
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
        variable v_k  : unsigned(9 downto 0);
        variable v_t  : unsigned(23 downto 0);
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
                s_twin   <= registers_in(6)(0);
                s_gold   <= registers_in(6)(1);
                s_mirror <= registers_in(6)(2);
                s_tear   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(546, 10);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(410, 10);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(273, 10);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is     -- Osc lattice step per line = ys * 364 / 256
                when 0          => s_oys <= to_unsigned(1618, 12);
                when 4          => s_oys <= to_unsigned(819, 12);
                when 8          => s_oys <= to_unsigned(1365, 12);
                when 12         => s_oys <= to_unsigned(682, 12);
                when 5 | 6 | 14 => s_oys <= to_unsigned(546, 12);
                when 1 | 2 | 10 => s_oys <= to_unsigned(728, 12);
                when others     => s_oys <= to_unsigned(364, 12);
            end case;

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

            -- serial products: slot k loads at zc = 32 + 24 k, 18 iterations, result stable from load + 19
            -- (stores at 52 + 24 k).  Slot number decoded at 30 + 24 k, operands pre-muxed at 31 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
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
                    when 510 => z_sel <= to_unsigned(20, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_a, 36);              z_fb <= to_unsigned(198, 18);     -- Faces exponent
                    when 1  => z_fa <= resize(z_a, 36);              z_fb <= to_unsigned(154, 18);     -- Bands exponent
                    when 2  => z_fa <= resize(z_a, 36);              z_fb <= to_unsigned(96, 18);      -- Mask exponent
                    when 3  => z_fa <= to_unsigned(C_STH0, 36);      z_fb <= resize(z_mt, 18);         -- face step / h
                    when 5  => z_fa <= resize(c_sth1, 36);           z_fb <= resize(to_unsigned(1920, 12) - c_xb0, 18); -- centre phase
                    when 6  => z_fa <= resize(c_kv, 36);             z_fb <= resize(c_kr, 18);         -- KVR
                    when 7  => z_fa <= resize(s_g2(12 downto 3), 36); z_fb <= resize(s_g2(12 downto 3), 18) + 256;  -- k2 (k2 + 256)
                    when 8  => z_fa <= resize(z_q, 36);              z_fb <= to_unsigned(2807, 18);    -- stripes gain
                    when 9  => z_fa <= resize(s_g3(12 downto 3), 36); z_fb <= resize(s_g3(12 downto 3), 18) + 256;  -- k3 (k3 + 256)
                    when 10 => z_fa <= resize(z_q, 36);              z_fb <= to_unsigned(1536, 18);    -- lightning depth
                    when 11 => z_fa <= resize(z_a, 36);              z_fb <= to_unsigned(307, 18);     -- face jitter share
                    when 12 => z_fa <= resize(s_g6(12 downto 3), 36); z_fb <= to_unsigned(2048, 18) - resize(s_g6(12 downto 3), 18); -- k6 (2048 - k6)
                    when 13 => z_fa <= resize(z_q, 36);              z_fb <= to_unsigned(133, 18);     -- etch gain
                    when 14 => z_fa <= resize(s_vm, 36);             z_fb <= resize(s_vm, 18);         -- roll^2
                    when 15 => z_fa <= resize(z_vsq, 36);            z_fb <= to_unsigned(1488, 18);    -- roll step
                    when others => z_fa <= resize(z_offd, 36);       z_fb <= to_unsigned(293, 18);     -- Mask level
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(17 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- roll (last field's step), lightning time phases, tear reroll, beam origin
                    when 20 =>
                        if s_vneg = '1' then c_s <= c_s - resize(c_vst, 18); else c_s <= c_s + resize(c_vst, 18); end if;
                        c_tz1 <= c_tz1 + 1857; c_tz2 <= c_tz2 - 1201; c_tz3 <= c_tz3 - 404;
                        if t_fc = 47 then
                            t_fc <= (others => '0'); c_tb <= c_tb + 37; c_so <= c_so + 61;
                        else
                            t_fc <= t_fc + 1;
                        end if;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                        c_z1i <= shift_left(resize(s_ys, 16), 3);
                    when 21 =>
                        v_k := s_g1(12 downto 3);
                        if v_k >= 512 then z_a <= v_k - 512; z_neg <= '0'; else z_a <= 512 - v_k; z_neg <= '1'; end if;
                    when 52 =>                        -- e1 = 0.775 (k1 - 512) / 256 octaves
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= resize(v_e(11 downto 8), 3);
                        v_k := s_g5(12 downto 3);
                        if v_k >= 512 then z_a <= v_k - 512; z_neg <= '0'; else z_a <= 512 - v_k; z_neg <= '1'; end if;
                    when 54 => z_mt <= r_ex_q;
                    when 76 =>                        -- Bands: 0.6 (k5 - 512) / 256 octaves
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0));
                        z_e <= v_e;
                        v_k := s_gp(12 downto 3);     -- Mask: kv = 2^(0.375 (p12 - 512) / 256)
                        if v_k >= 512 then z_a <= v_k - 512; z_neg <= '0'; else z_a <= 512 - v_k; z_neg <= '1'; end if;
                    when 78 =>                        -- kr = 2^(e / 256), Q12
                        case to_integer(z_e(11 downto 8)) is
                            when 1      => c_kr <= resize(shift_right(r_ex_q, 2), 14);
                            when 0      => c_kr <= resize(shift_right(r_ex_q, 3), 14);
                            when -1     => c_kr <= resize(shift_right(r_ex_q, 4), 14);
                            when others => c_kr <= resize(shift_right(r_ex_q, 5), 14);
                        end case;
                    when 100 =>
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_e <= v_e;
                        -- Mask level |p12 - 512| (applied with the sign below)
                        v_k := s_gp(12 downto 3);
                        if v_k >= 512 then z_offd <= v_k - 512; z_offn <= '0';
                        else z_offd <= 512 - v_k; z_offn <= '1'; end if;
                    when 102 =>                       -- kv = 2^(e / 256), Q12
                        case to_integer(z_e(11 downto 8)) is
                            when 1      => c_kv <= resize(shift_right(r_ex_q, 2), 14);
                            when 0      => c_kv <= resize(shift_right(r_ex_q, 3), 14);
                            when -1     => c_kv <= resize(shift_right(r_ex_q, 4), 14);
                            when others => c_kv <= resize(shift_right(r_ex_q, 5), 14);
                        end case;
                    when 124 =>                       -- step / h = 8991 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when 1      => c_sth1 <= f_acc(29 downto 14);
                            when 0      => c_sth1 <= f_acc(30 downto 15);
                            when -1     => c_sth1 <= f_acc(31 downto 16);
                            when others => c_sth1 <= f_acc(32 downto 17);
                        end case;
                    when 148 =>                       -- face step per pixel = sth1 xs
                        case to_integer(s_xs) is
                            when 4      => c_step <= shift_left(resize(c_sth1, 18), 2);
                            when 3      => c_step <= shift_left(resize(c_sth1, 18), 1) + resize(c_sth1, 18);
                            when others => c_step <= shift_left(resize(c_sth1, 18), 1);
                        end case;
                    when 172 =>                       -- start phase = PC - (1920 - xb0) sth1 (mod 2^24)
                        v_t := f_acc(23 downto 0);
                        c_pst <= to_unsigned(C_PC, 24) - v_t;
                    when 196 => c_kvr <= f_acc(27 downto 12);
                    when 220 => z_q <= f_acc(20 downto 0);
                    when 244 => c_ka <= f_acc(31 downto 16);
                    when 268 => z_q <= f_acc(20 downto 0);
                    when 292 =>
                        c_za <= f_acc(30 downto 16);
                        v_k := s_g3(12 downto 3);     -- face jitter share: (k3 - 512) above the centre
                        if v_k > 512 then z_a <= v_k - 512; else z_a <= (others => '0'); end if;
                    when 316 => c_fj <= f_acc(17 downto 10);
                    when 340 => z_q <= f_acc(20 downto 0);
                    when 364 =>
                        if s_src = '1' then c_eg <= resize(f_acc(27 downto 19), 10) + shift_left(resize(f_acc(27 downto 19), 10), 1);
                        else c_eg <= resize(f_acc(27 downto 19), 10); end if;
                    when 388 => z_vsq <= f_acc(17 downto 0);
                    when 412 => c_vst <= f_acc(31 downto 16);
                    when 436 =>                       -- stripes start phase / step per pixel, by mode
                        case to_integer(s_xs) is
                            when 4      => c_rpst <= to_unsigned(5899008, 24); c_rstep <= to_unsigned(306432, 20);
                            when 3      => c_rpst <= to_unsigned(2068736, 24); c_rstep <= to_unsigned(229824, 20);
                            when others => c_rpst <= to_unsigned(1455872, 24); c_rstep <= to_unsigned(153216, 20);
                        end case;
                        case to_integer(unsigned(s_tid)) is   -- lightning steps per line: ys 1394 / 256, ys 379 / 256
                            when 0          => c_z2i <= to_unsigned(6196, 16); c_z3i <= to_unsigned(1684, 16);
                            when 4          => c_z2i <= to_unsigned(3136, 16); c_z3i <= to_unsigned(852, 16);
                            when 8          => c_z2i <= to_unsigned(5227, 16); c_z3i <= to_unsigned(1421, 16);
                            when 12         => c_z2i <= to_unsigned(2613, 16); c_z3i <= to_unsigned(710, 16);
                            when 5 | 6 | 14 => c_z2i <= to_unsigned(2091, 16); c_z3i <= to_unsigned(568, 16);
                            when 1 | 2 | 10 => c_z2i <= to_unsigned(2788, 16); c_z3i <= to_unsigned(758, 16);
                            when others     => c_z2i <= to_unsigned(1394, 16); c_z3i <= to_unsigned(379, 16);
                        end case;
                    when 532 =>
                        if z_offn = '1' then c_off <= -signed(resize(f_acc(19 downto 10), 11));
                        else c_off <= signed(resize(f_acc(19 downto 10), 11)); end if;
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
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ym  : unsigned(18 downto 0);
        variable v_s   : unsigned(9 downto 0);
        variable v_r   : unsigned(23 downto 0);
        variable v_o   : signed(11 downto 0);
        variable v_x   : signed(15 downto 0);
        variable v_q   : unsigned(17 downto 0);
        variable v_c   : unsigned(25 downto 0);
        variable v_ga, v_gb : signed(18 downto 0);
        variable v_pm  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_t1 <= c_tz1; e_t2 <= c_tz2; e_t3 <= c_tz3;
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 20);
                e_lc <= (others => '0');
                e_t1 <= e_t1 + c_z1i; e_t2 <= e_t2 + c_z2i; e_t3 <= e_t3 + c_z3i;
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(28 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_yi <= e_vy(18 downto 8);
                when 2 =>                     -- mirror; tear band position; lightning sum
                    if s_mirror = '1' and e_yi >= 540 then v_ym := to_unsigned(276480, 19) - e_vy(18 downto 0); else v_ym := e_vy(18 downto 0); end if;
                    e_ymq <= v_ym;
                    e_tbs <= resize(e_yi, 12) + resize(c_tb, 12);
                    v_s := shift_left(resize(f_tri7(e_t1(15 downto 8)), 10), 1) + resize(f_tri7(e_t2(15 downto 8)), 10)
                           + resize(f_tri7(e_t3(15 downto 8)), 10);
                    e_s <= signed(v_s) - 256;
                when 3 =>                     -- D = Ym 1024 / LW (Q8 Ym); dy = Ym - 540 (Q8)
                    m_a <= resize(e_ymq, 30); m_b <= to_unsigned(749, 16); m_acc <= (others => '0');
                    e_dyq <= signed(resize(e_ymq, 20)) - 138240;
                    r_th_a <= e_tbs(11 downto 6) + c_so;
                when 5 =>
                    if s_tear = '1' then e_th <= r_th_q; else e_th <= (others => '0'); end if;
                when 20 =>                    -- table position = D [x2 Twin] - roll (Q8, mod 1024 entries x 4); then P
                    if s_twin = '1' then v_q := m_acc(26 downto 9); else v_q := m_acc(27 downto 10); end if;
                    e_dq <= v_q - c_s;
                    m_a <= resize(unsigned(abs(e_dyq)), 30); m_b <= to_unsigned(549, 16); m_acc <= (others => '0');
                when 21 =>
                    r_sc_a <= e_dq(17 downto 10); e_f6 <= e_dq(9 downto 4);
                when 22 =>
                    r_sc_a <= e_dq(17 downto 10) + 1;
                when 23 =>
                    e_scA <= r_sc_q;
                when 24 =>
                    e_scB <= r_sc_q;
                when 25 =>
                    e_dv <= resize(signed(e_scB(31 downto 19)), 14) - resize(signed(e_scA(31 downto 19)), 14);
                    e_dg <= resize(signed(e_scB(18 downto 8)), 12) - resize(signed(e_scA(18 downto 8)), 12);
                    e_d2 <= resize(signed(e_scB(7 downto 0)), 9) - resize(signed(e_scA(7 downto 0)), 9);
                when 37 =>                    -- P = +-(|dy| 549 >> 18); Gf = 1024 + 2.125 |P| below the centre (0.6 (Y - 540) / 540)
                    v_pm := m_acc(27 downto 18);
                    if e_dyq < 0 then
                        e_p <= -signed(resize(v_pm, 12)); e_gf <= to_unsigned(1024, 11);
                    else
                        e_p <= signed(resize(v_pm, 12));
                        e_gf <= to_unsigned(1024, 11) + shift_left(resize(v_pm, 11), 1) + resize(v_pm(9 downto 3), 11);
                    end if;
                when 54 =>                    -- the V interpolation |dv| f6
                    m_a <= resize(unsigned(abs(e_dv)), 30); m_b <= resize(e_f6, 16); m_acc <= (others => '0');
                when 71 =>                    -- 2 Vs = 2 A +- (|dv| f6 >> 5); then |dg| f6
                    if e_dv < 0 then e_v2 <= shift_left(resize(signed(e_scA(31 downto 19)), 16), 1) - signed(resize(m_acc(19 downto 5), 16));
                    else e_v2 <= shift_left(resize(signed(e_scA(31 downto 19)), 16), 1) + signed(resize(m_acc(19 downto 5), 16)); end if;
                    m_a <= resize(unsigned(abs(e_dg)), 30); m_b <= resize(e_f6, 16); m_acc <= (others => '0');
                when 72 =>                    -- V = 2 Vs + P
                    e_v <= e_v2 + resize(e_p, 16);
                when 73 =>
                    e_x <= e_v - C_PIV;
                when 88 =>                    -- gs (Q12) = 16 A +- (|dg| f6 >> 2); then |d2| f6
                    if e_dg < 0 then e_gs <= shift_left(resize(signed(e_scA(18 downto 8)), 17), 4) - signed(resize(m_acc(16 downto 2), 17));
                    else e_gs <= shift_left(resize(signed(e_scA(18 downto 8)), 17), 4) + signed(resize(m_acc(16 downto 2), 17)); end if;
                    m_a <= resize(unsigned(abs(e_d2)), 30); m_b <= resize(e_f6, 16); m_acc <= (others => '0');
                when 105 =>                   -- g2s (Q11) = 16 A +- (|d2| f6 >> 2); then |gs| Gf
                    if e_d2 < 0 then e_g2s <= shift_left(resize(signed(e_scA(7 downto 0)), 14), 4) - signed(resize(m_acc(13 downto 2), 14));
                    else e_g2s <= shift_left(resize(signed(e_scA(7 downto 0)), 14), 4) + signed(resize(m_acc(13 downto 2), 14)); end if;
                    m_a <= resize(unsigned(abs(e_gs)), 30); m_b <= resize(e_gf, 16); m_acc <= (others => '0');
                when 122 =>                   -- (|gs| Gf >> 10) kr
                    m_a <= resize(m_acc(24 downto 10), 30); m_b <= resize(c_kr, 16); m_acc <= (others => '0');
                when 139 =>                   -- |g| = min(.. >> 13, 65535) (Q11); then |g2s| Gf
                    if m_acc(29) = '1' then e_m2 <= (others => '1'); else e_m2 <= m_acc(28 downto 13); end if;
                    n_gn <= e_gs(16);
                    m_a <= resize(unsigned(abs(e_g2s)), 30); m_b <= resize(e_gf, 16); m_acc <= (others => '0');
                when 156 =>                   -- (|g2s| Gf >> 10) kr
                    m_a <= resize(m_acc(21 downto 10), 30); m_b <= resize(c_kr, 16); m_acc <= (others => '0');
                when 173 =>                   -- |g2| = min(.. >> 12, 16383) (Q11); then ga = |g| (ka / 4) >> 17
                    if m_acc(29 downto 26) /= 0 then n_g2 <= (others => '1'); else n_g2 <= m_acc(25 downto 12); end if;
                    n_g2n <= e_g2s(13);
                    m_a <= resize(c_ka(15 downto 2), 30); m_b <= resize(e_m2, 16); m_acc <= (others => '0');
                when 174 =>                   -- signed g, 2 g2 and 2046 g2 >> 11
                    if n_gn = '1' then e_gss <= -signed(resize(e_m2, 18)); else e_gss <= signed(resize(e_m2, 18)); end if;
                    if n_g2n = '1' then e_g2d <= -signed(resize(n_g2 & '0', 18)); else e_g2d <= signed(resize(n_g2 & '0', 18)); end if;
                    v_c := shift_left(resize(n_g2, 26), 11) - shift_left(resize(n_g2, 26), 1);
                    if n_g2n = '1' then e_cb <= -signed(resize(v_c(25 downto 11), 16)); else e_cb <= signed(resize(v_c(25 downto 11), 16)); end if;
                when 175 =>                   -- the near half (hn < 512): |g + 2 g2|; the far half: |g - 2 g2|
                    v_ga := resize(e_gss, 19) + resize(e_g2d, 19);
                    n_snear <= v_ga(18); v_ga := abs(v_ga); n_gnear <= unsigned(v_ga(16 downto 0));
                    v_gb := resize(e_gss, 19) - resize(e_g2d, 19);
                    n_sfar <= v_gb(18); v_gb := abs(v_gb); n_gfar <= unsigned(v_gb(16 downto 0));
                when 190 =>                   -- ga = min(.., 4095); then |X| kvr
                    if m_acc(29) = '1' then n_ga <= (others => '1'); else n_ga <= m_acc(28 downto 17); end if;
                    m_a <= resize(unsigned(abs(e_x)), 30); m_b <= c_kvr; m_acc <= (others => '0');
                when 207 =>                   -- V' = PIV +- (|X| kvr >> 12) - off; then the lightning |S| za
                    if e_x < 0 then v_x := to_signed(C_PIV, 16) - signed(resize(m_acc(26 downto 12), 16));
                    else v_x := to_signed(C_PIV, 16) + signed(resize(m_acc(26 downto 12), 16)); end if;
                    v_x := v_x - resize(c_off, 16);
                    n_v <= unsigned(v_x(11 downto 0));
                    m_a <= resize(unsigned(abs(e_s)), 30); m_b <= resize(c_za, 16); m_acc <= (others => '0');
                when 224 =>                   -- zig = +-(|S| za >> 15) (h); then the face jitter |zig| fj
                    if e_s < 0 then e_zig <= -signed(resize(m_acc(23 downto 15), 10));
                    else e_zig <= signed(resize(m_acc(23 downto 15), 10)); end if;
                    m_a <= resize(m_acc(23 downto 15), 30); m_b <= resize(c_fj, 16); m_acc <= (others => '0');
                when 208 =>
                    n_vb <= n_v + unsigned(e_cb(11 downto 0));
                when 225 =>                   -- stripes offset = tear + zig
                    e_off <= resize(e_th, 12) + resize(e_zig, 12);
                when 241 =>                   -- face offset = tear +- fj; then |stripes offset| rst1
                    if e_zig < 0 then v_o := resize(e_th, 12) - signed(resize(m_acc(15 downto 8), 12));
                    else v_o := resize(e_th, 12) + signed(resize(m_acc(15 downto 8), 12)); end if;
                    e_th <= v_o(10 downto 0);
                    m_a <= to_unsigned(C_RST1, 30); m_b <= resize(unsigned(abs(e_off)), 16); m_acc <= (others => '0');
                when 258 =>                   -- stripes start phase; then |face offset| sth1
                    v_r := m_acc(23 downto 0);
                    if e_off < 0 then n_rp0 <= c_rpst - v_r; else n_rp0 <= c_rpst + v_r; end if;
                    m_a <= resize(c_sth1, 30); m_b <= resize(unsigned(abs(e_th)), 16); m_acc <= (others => '0');
                when 275 =>                   -- face start phase; then |zig| rst1 (the inside-the-green phase offset)
                    v_r := m_acc(23 downto 0);
                    if e_th < 0 then n_p0 <= c_pst - v_r; else n_p0 <= c_pst + v_r; end if;
                    m_a <= to_unsigned(C_RST1, 30); m_b <= resize(unsigned(abs(e_zig)), 16); m_acc <= (others => '0');
                when 292 =>
                    v_r := m_acc(23 downto 0);
                    if e_zig < 0 then v_r := (not v_r) + 1; end if;
                    n_zo <= v_r(23 downto 16);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the face and stripes oscillators' phases, the Osc lattice
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_p <= n_p0; b_r <= n_rp0;
                l_v <= n_v; l_vb <= n_vb; l_gnear <= n_gnear; l_snear <= n_snear; l_gfar <= n_gfar; l_sfar <= n_sfar; l_ga <= n_ga; l_zo <= n_zo;
                l_step <= c_step; l_rstep <= c_rstep;
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            elsif g_avid = '1' then
                b_p <= b_p + resize(l_step, 24);
                b_r <= b_r + resize(l_rstep, 24);
                s_ox <= s_ox + s_oxs;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_hn  : unsigned(9 downto 0);
        variable v_l   : unsigned(7 downto 0);
        variable v_os  : unsigned(7 downto 0);
        variable v_g   : unsigned(22 downto 0);
        variable v_r   : signed(7 downto 0);
        variable v_ra  : unsigned(6 downto 0);
        variable v_rp  : unsigned(23 downto 0);
        variable v_f   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: triangle -> hn, T2; stripes ROM addresses; luma (Video: Y >> 2 - 16; Osc: the lattice)
            if b_p(23) = '1' then v_hn := not b_p(22 downto 13); else v_hn := b_p(22 downto 13); end if;
            q2_hn <= v_hn; q2_hi <= v_hn(9);
            if v_hn(9) = '1' then q2_gm <= l_gfar; q2_sg <= l_sfar; else q2_gm <= l_gnear; q2_sg <= l_snear; end if;
            q2_az <= b_r(23 downto 16);
            if s_src = '1' then
                v_os := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
                q2_lm <= v_os;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then q2_lm <= v_l - 16; else q2_lm <= (others => '0'); end if;
            end if;
            -- 3: ROMs; (g +- 2 g2) x hn partials (split 5 / 5)
            q3_gh <= resize(q2_gm * q2_hn(9 downto 5), 22);
            q3_gl <= resize(q2_gm * q2_hn(4 downto 0), 22);
            q3_sg <= q2_sg; q3_hi <= q2_hi;
            q3_lm <= q2_lm;
            -- 4: fabric copies; sums; etch partials (luma nibbles x gain)
            q3_az <= q2_az; q4_az <= q3_az; q5_az <= q4_az; q6_az <= q5_az;
            q4_gt  <= shift_left(resize(q3_gh, 27), 5) + resize(q3_gl, 27);
            q4_sg <= q3_sg; q4_hi <= q3_hi;
            q4_el <= resize(q3_lm(3 downto 0) * c_eg, 14);
            q4_eh <= resize(q3_lm(7 downto 4) * c_eg, 14);
            -- 5: scale and sign; etch sum
            if q4_sg = '1' then q5_gt <= (not q4_gt(22 downto 11)) + 1;
            else q5_gt <= q4_gt(22 downto 11); end if;
            q5_hi <= q4_hi;
            v_g := resize(q4_el, 23) + shift_left(resize(q4_eh, 23), 4);
            q5_et <= v_g(17 downto 8);
            -- 6, 7: F0 = V + g T1 + g2 T2 + etch
            if q5_hi = '1' then q6_f1 <= l_vb + q5_gt; else q6_f1 <= l_v + q5_gt; end if;
            q6_f2 <= resize(q5_et, 12);
            q7_f0 <= q6_f1 + q6_f2;
            -- 7: stripes ROM addresses (the jittered phase, and the straight one inside the green)
            r_cz_a <= q6_az; r_c0_a <= q6_az - l_zo;
            -- 8: stripes-depth table index (palette address / 16 of F0)
            v_f := q7_f0 + C_POFF;
            q8_wi <= v_f(11 downto 6);
            q8_f0 <= q7_f0;
            -- 9: {inside, w4}
            q9_w <= C_MU_W(to_integer(q8_wi));
            q9_f0 <= q8_f0; q9_cz <= signed(r_cz_q); q9_c0 <= signed(r_c0_q);
            -- 10: R = the straight stripes inside the green, the jittered ones outside; |R| w4
            if q9_w(4) = '1' then v_r := q9_c0; else v_r := q9_cz; end if;
            if v_r(7) = '1' then v_ra := unsigned(not v_r(6 downto 0)) + 1; else v_ra := unsigned(v_r(6 downto 0)); end if;
            q10_rw <= resize(v_ra(6 downto 1) * unsigned(q9_w(3 downto 0)), 10);
            if v_r > 0 then q10_rn <= '1'; else q10_rn <= '0'; end if;
            q10_f0 <= q9_f0;
            -- 11: ga x rw partials (rw split 5 / 6)
            q11_ph <= resize(l_ga * q10_rw(9 downto 5), 17);
            q11_pl <= resize(l_ga * q10_rw(4 downto 0), 17);
            q11_rn <= q10_rn; q11_f0 <= q10_f0;
            -- 12: stripes term
            v_rp := shift_left(resize(q11_ph, 24), 5) + resize(q11_pl, 24);
            q12_rip <= v_rp(21 downto 9);
            q12_rn <= q11_rn; q12_f0 <= q11_f0;
            -- 13: F = F0 -+ the stripes
            if q12_rn = '1' then q13_f <= q12_f0 - q12_rip(11 downto 0);
            else q13_f <= q12_f0 + q12_rip(11 downto 0); end if;
            -- 14: palette address (F + 488) / 4 mod 1024, + 1024 for Gold
            v_f := q13_f + C_POFF;
            r_p_a <= s_gold & v_f(11 downto 2);
            -- 15: palette EBR; 16: fabric copy
            q16_p <= r_p_q;
            -- 17: compose; neutral outside the generated avid
            if s_gav_sr(14) = '0' then
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            else
                s_y17 <= unsigned(q16_p(23 downto 16)) & "00";
                s_u17 <= unsigned(q16_p(15 downto 8)) & "10";
                s_v17 <= unsigned(q16_p(7 downto 0)) & "10";
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums, with the running-sum resync), Y through [1 2 1] centred on
    -- the same sample, luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_yy : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 18 sums, stage 19 output; the 8-sums snap back to neutral once 8 blank samples are in
            s_cay(0) <= s_y17; s_cau(0) <= s_u17; s_cav(0) <= s_v17;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u17 - s_cau(7);
                s_bav <= s_bav + s_v17 - s_cav(7);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 14) + s_baud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v19 <= v_sc(13 downto 4);
            -- luma [1 2 1] centred on the box's centre sample (4 back)
            v_yy := resize(s_cay(3), 12) + shift_left(resize(s_cay(4), 12), 1) + resize(s_cay(5), 12);
            s_y19 <= v_yy(11 downto 2);
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

end architecture mummer;
