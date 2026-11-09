-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Owlet v0.1.0 -- Heirloom from the video clips (phase 3b), made from the still WIN_20230818_01_17_40 @ 3.0 s.
--
-- The still: two contour eyes (axes x 530 and 1470) of nested blue / cyan / pink rings with a black outline
-- (corners on row 520, flat top, pointed bottom) and violet triangular pupils, a black X between them on the
-- saddle, sawtooth "fins" (tongues) in the bands above the eyes, a red band with chevrons across the top, and the
-- left 70 px held into horizontal stripes.  In the clip the eye row rolls UP the frame (one row per 5.9 s), the
-- eyes morph from hearts (low) to lenses (centre) to upturned triangles (high) as they pass the screen centre,
-- and a red band rolls up twice as fast, so every other eye row passes through it in red.
--
-- The patch, as fitted (owlet.py has every table):
--   F      = V(Y, d) + g(Y, d) S(hn) + a(d) Z(hn) + Etch * luma       (palette-index units, Q2)
--   hn     = distance to the nearest eye axis / half the eye period (a triangle oscillator, held left of x 70)
--   d      = Y - 610 + s: the rolling vertical oscillator; V = Vs(d) + P(Y), g = gs(d) Gf(Y) (P, Gf: the still's
--            screen-fixed residuals, so the default frame at s = 0 is the still and the eyes morph as they roll)
--   colour = palette(F); inside the red band (a second roll at twice the rate) the red monochrome of it
--
--   K1  Eyes    eye count: one wide eye (0) .. the clip's two (512) .. six narrow eyes (1023)
--   K2  Rings   ring density x0.44 .. x2.3 about the pupil level
--   K3  Fins    the sawtooth fins (tongues): none (0) .. the clip (512) .. flames across most of the frame
--   K4  Roll    bipolar roll speed, +-8 deadband; 700 = the clip (one eye row per 5.9 s, up)
--   K5  Ember   the red band's height: none (0) .. the clip (512) .. most of the frame (1023)
--   K6  Etch    how far the camera (or the Osc lattice) pushes the field
--   S7  Twin    two rows of eyes (the oscillator at twice the vertical rate)
--   S8  Shape   Lens (the clip) / Diamond (pinched, concave sides)
--   S9  Melt    the sibling clip's liquid wobble: each line's phase shifted by two rolling triangle waves
--   S10 Fire    the pupils burn red (the sibling clip's red blobs)
--   S11 Source  Video / Osc
--   P12 GAZE    the eyes' opening: slits (0) .. the clip (512) .. wide open with dilated pink-cored pupils (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: table reads with interpolation, V, g, the fin depth, the red threshold and the
-- Melt phase), a triangle-phase beam, a 15-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre
-- 4:3).  Generated avid; output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 with the running-sum resync, luma
-- [1 2 1], luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.owlet_pkg.all;
use work.all;

architecture owlet of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_FPQ   : integer := -280;       -- the pupil level F = -70, Q2
    constant C_TPOS  : integer := 131071;     -- red threshold sentinels (18-bit signed)
    constant C_TNEG  : integer := -131072;

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
    function f_tri15(v : unsigned(15 downto 0)) return unsigned is
    begin
        if v(15) = '1' then return not v(14 downto 0); else return v(14 downto 0); end if;
    end function;

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sc_a, r_sf_a : unsigned(7 downto 0) := (others => '0');
    signal r_sc_q, r_sf_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_s_a  : unsigned(9 downto 0) := (others => '0');
    signal r_s_q  : unsigned(9 downto 0) := (others => '0');
    signal r_z_a  : unsigned(8 downto 0) := (others => '0');
    signal r_z_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal r_p_a  : unsigned(9 downto 0) := (others => '0');
    signal r_p_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_rd_a : unsigned(7 downto 0) := (others => '0');
    signal r_rd_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_twin, s_shape, s_melt, s_fire, s_src : std_logic := '0';

    -- per-field constants
    signal z_a    : unsigned(9 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_off, z_offd, z_k3d : unsigned(9 downto 0) := (others => '0');
    signal z_k3h  : std_logic := '0';
    signal z_k3t  : unsigned(15 downto 0) := (others => '0');
    signal z_h    : unsigned(11 downto 0) := (others => '0');
    signal z_offn : std_logic := '0';
    signal z_fg   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal z_k6s  : unsigned(19 downto 0) := (others => '0');
    signal z_eg   : unsigned(15 downto 0) := (others => '0');
    signal z_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_sth1 : unsigned(15 downto 0) := to_unsigned(8923, 16);  -- triangle phase per h, Q4 (2^24 per eye period)
    signal c_step : unsigned(17 downto 0) := to_unsigned(17846, 18); -- per pixel
    signal c_pst  : unsigned(23 downto 0) := (others => '0');        -- phase at the line's first pixel
    signal c_xb0  : unsigned(11 downto 0) := (others => '0');        -- beam start (h)
    signal c_xsh  : unsigned(11 downto 0) := to_unsigned(140, 12);   -- max(x0, 140): the hold point
    signal c_kr   : unsigned(13 downto 0) := to_unsigned(4096, 14);  -- Rings, Q12
    signal c_kv   : unsigned(13 downto 0) := to_unsigned(4096, 14);  -- Gaze opening, Q12
    signal c_kvr  : unsigned(15 downto 0) := to_unsigned(4096, 16);  -- kv kr, Q12
    signal c_kg   : unsigned(15 downto 0) := to_unsigned(4096, 16);  -- (3/4 + kv/4) kr, Q12
    signal c_cv   : signed(15 downto 0) := to_signed(C_FPQ, 16);     -- FP + off kr (Q2)
    signal c_ka   : unsigned(15 downto 0) := to_unsigned(1963, 16);  -- fin depth per wn step, Q8 (of Q2)
    signal c_ron  : unsigned(7 downto 0) := to_unsigned(140, 8);
    signal c_eg   : unsigned(17 downto 0) := (others => '0');        -- etch per luma step, Q8 (of Q2)
    signal c_t0   : signed(17 downto 0) := to_signed(2760, 18);      -- red ramp centre (Q2)
    signal c_ts   : unsigned(15 downto 0) := to_unsigned(3264, 16);  -- red ramp slope per line, Q8 (of Q2)
    signal c_tf   : signed(17 downto 0) := to_signed(320, 18);       -- Fire threshold (Q2)
    signal c_h    : unsigned(10 downto 0) := to_unsigned(840, 11);   -- red band height (lines)
    signal c_loe  : signed(11 downto 0) := to_signed(115, 12);       -- red band lower edge at s = 0
    signal c_vst  : unsigned(15 downto 0) := (others => '0');        -- roll per field, Q8 lines
    signal c_s    : unsigned(17 downto 0) := (others => '0');        -- roll phase, Q8 lines (mod 1024)
    signal c_m1i, c_m2i : unsigned(9 downto 0) := (others => '0');   -- melt phase steps per line (Q16 cycles)
    signal c_jst  : unsigned(11 downto 0) := to_unsigned(243, 12);   -- screen-table step per line (Q10)
    signal c_tm1, c_tm2 : unsigned(15 downto 0) := (others => '0');  -- melt phases at line 0

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
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_yi   : unsigned(10 downto 0) := (others => '0');
    signal e_d, e_dd : unsigned(9 downto 0) := (others => '0');
    signal e_xt   : signed(16 downto 0) := (others => '0');
    signal e_q, e_qn : unsigned(10 downto 0) := (others => '0');
    signal e_p1, e_p2 : unsigned(13 downto 0) := (others => '0');
    signal e_jj   : unsigned(19 downto 0) := (others => '0');   -- screen-table index, Q10 (Y * 243 / 1024)
    signal e_fi, e_fj : unsigned(1 downto 0) := (others => '0');
    signal e_scA, e_scB, e_sfA, e_sfB : std_logic_vector(31 downto 0) := (others => '0');
    signal e_dv, e_dp : signed(14 downto 0) := (others => '0');
    signal e_dg   : signed(11 downto 0) := (others => '0');
    signal e_dgf  : signed(12 downto 0) := (others => '0');
    signal e_dw   : signed(8 downto 0) := (others => '0');
    signal e_tv, e_tp : signed(16 downto 0) := (others => '0');
    signal e_tg   : signed(13 downto 0) := (others => '0');
    signal e_tgf  : signed(14 downto 0) := (others => '0');
    signal e_tw   : signed(10 downto 0) := (others => '0');
    signal e_vs, e_pp : signed(15 downto 0) := (others => '0');
    signal e_gs   : unsigned(10 downto 0) := (others => '0');
    signal e_gf   : unsigned(11 downto 0) := (others => '0');
    signal e_wn   : unsigned(7 downto 0) := (others => '0');
    signal e_x    : signed(16 downto 0) := (others => '0');
    signal e_xn   : std_logic := '0';
    signal e_elo, e_ehi : signed(11 downto 0) := (others => '0');
    signal e_tz   : unsigned(1 downto 0) := "00";       -- 0 outside, 1 inside, 2 lower ramp, 3 upper ramp
    signal e_ten  : std_logic := '0';                   -- ramp sign: '1' = T0 - TS e
    signal e_te   : unsigned(6 downto 0) := (others => '0');
    signal e_m1, e_m2 : unsigned(15 downto 0) := (others => '0');
    signal e_mt1, e_mt2 : unsigned(6 downto 0) := (others => '0');
    signal e_mh   : signed(9 downto 0) := (others => '0');   -- melt shift, h
    signal m_a    : unsigned(29 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(29 downto 0) := (others => '0');
    -- next-line results
    signal n_v    : signed(17 downto 0) := (others => '0');
    signal n_g    : unsigned(11 downto 0) := (others => '0');
    signal n_a    : unsigned(13 downto 0) := (others => '0');
    signal n_t    : signed(17 downto 0) := to_signed(C_TPOS, 18);
    signal n_p0   : unsigned(23 downto 0) := (others => '0');

    -- beam / per-line values in use
    signal b_p   : unsigned(23 downto 0) := (others => '0');
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_run : std_logic := '0';
    signal l_v   : signed(17 downto 0) := (others => '0');
    signal l_g   : unsigned(11 downto 0) := (others => '0');
    signal l_a   : unsigned(13 downto 0) := (others => '0');
    signal l_t   : signed(17 downto 0) := to_signed(C_TPOS, 18);
    signal l_step : unsigned(17 downto 0) := (others => '0');
    -- oscillator source (Source = Osc): a drifting 2-D triangle lattice, 240 x 180 virtual px
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(136, 10);
    signal s_oys : unsigned(11 downto 0) := to_unsigned(364, 12);

    -- pixel path
    signal q2_lm, q3_lm : unsigned(7 downto 0) := (others => '0');
    signal q4_s  : unsigned(9 downto 0) := (others => '0');
    signal q4_z  : signed(7 downto 0) := (others => '0');
    signal q4_el, q4_eh : unsigned(21 downto 0) := (others => '0');
    signal q5_gh, q5_gl : unsigned(16 downto 0) := (others => '0');
    signal q5_ah  : unsigned(16 downto 0) := (others => '0');
    signal q5_al  : unsigned(17 downto 0) := (others => '0');
    signal q5_zn, q6_zn : std_logic := '0';
    signal q5_et  : unsigned(25 downto 0) := (others => '0');
    signal q6_gs  : unsigned(13 downto 0) := (others => '0');
    signal q6_az  : signed(14 downto 0) := (others => '0');
    signal q6_et  : unsigned(17 downto 0) := (others => '0');
    signal q7_f1, q7_f2 : signed(17 downto 0) := (others => '0');
    signal q8_f   : signed(17 downto 0) := (others => '0');
    type t_flags is array (9 to 14) of std_logic;
    signal q_red, q_fir : t_flags := (others => '0');
    signal q11_p  : std_logic_vector(31 downto 0) := (others => '0');
    signal q12_y, q12_u, q12_v : unsigned(9 downto 0) := (others => '0');
    signal q14_ry, q14_ru, q14_rv : unsigned(9 downto 0) := (others => '0');
    signal q13_y, q13_u, q13_v : unsigned(9 downto 0) := (others => '0');
    signal q14_y, q14_u, q14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
            r_ex_q <= C_OW_EXP2(to_integer(r_ex_a));
            r_sc_q <= C_OW_SC(to_integer(r_sc_a));
            r_sf_q <= C_OW_SF(to_integer(r_sf_a));
            r_s_q  <= C_OW_S(to_integer(r_s_a));
            r_z_q  <= C_OW_Z(to_integer(r_z_a));
            r_p_q  <= C_OW_PAL(to_integer(r_p_a));
            r_rd_q <= C_OW_RED(to_integer(r_rd_a));
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
                s_twin  <= registers_in(6)(0);
                s_shape <= registers_in(6)(1);
                s_melt  <= registers_in(6)(2);
                s_fire  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(546, 10);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(410, 10);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(273, 10);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 21) * to_unsigned(364, 9), 8), 12);

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
                    when 558 => z_sel <= to_unsigned(22, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_a, 36);              z_fb <= to_unsigned(198, 18);     -- Eyes exponent
                    when 1  => z_fa <= resize(z_a, 36);              z_fb <= to_unsigned(154, 18);     -- Rings exponent
                    when 2  => z_fa <= resize(z_a, 36);              z_fb <= to_unsigned(205, 18);     -- Gaze exponent
                    when 3  => z_fa <= to_unsigned(8923, 36);        z_fb <= resize(z_mt, 18);         -- triangle step / h
                    when 4  => z_fa <= resize(c_sth1, 36);           z_fb <= resize(s_xs, 18);         -- per pixel
                    when 5  => z_fa <= resize(c_sth1, 36);           z_fb <= resize(to_unsigned(2000, 12) - c_xsh, 18); -- start phase
                    when 6  => z_fa <= resize(c_kv, 36);             z_fb <= resize(c_kr, 18);         -- KVR
                    when 7  => z_fa <= resize(c_kv(13 downto 2), 36) + 3072; z_fb <= resize(c_kr, 18); -- KG
                    when 8  => z_fa <= resize(z_off, 36);            z_fb <= resize(c_kr, 18);         -- off * kr
                    when 9  => z_fa <= to_unsigned(1963, 36);        z_fb <= resize(z_fg, 18);         -- fin depth
                    when 10 => z_fa <= resize(c_ka, 36);             z_fb <= resize(c_kr, 18);         -- x kr
                    when 11 => z_fa <= resize(s_g6(12 downto 3), 36); z_fb <= resize(s_g6(12 downto 3), 18); -- etch^2
                    when 12 => z_fa <= resize(z_k6s(19 downto 6), 36); z_fb <= resize(c_kr, 18);       -- x kr
                    when 13 => z_fa <= resize(z_eg, 36);             z_fb <= to_unsigned(154, 18);     -- x 0.6
                    when 14 => z_fa <= to_unsigned(3040, 36);        z_fb <= resize(c_kr, 18);         -- red ramp centre
                    when 15 => z_fa <= resize(c_kr, 36);             z_fb <= to_unsigned(204, 18);     -- red ramp slope
                    when 16 => z_fa <= to_unsigned(600, 36);         z_fb <= resize(c_kr, 18);         -- Fire threshold
                    when 17 => z_fa <= resize(s_vm, 36);             z_fb <= resize(s_vm, 18);         -- roll^2
                    when 18 => z_fa <= resize(z_vsq, 36);            z_fb <= to_unsigned(1498, 18);    -- roll step
                    when 19 =>                                                                         -- red band height
                        if s_g5(12 downto 3) > 512 then
                            z_fa <= resize(s_g5(12 downto 3) - 512, 36); z_fb <= to_unsigned(581, 18);
                        else
                            z_fa <= resize(s_g5(12 downto 3), 36);       z_fb <= to_unsigned(420, 18);
                        end if;
                    when 20 => z_fa <= resize(s_ys, 36);             z_fb <= to_unsigned(252, 18);     -- melt step 1
                    when 21 => z_fa <= resize(s_ys, 36);             z_fb <= to_unsigned(676, 18);     -- melt step 2
                    when others => z_fa <= resize(s_ys, 36);         z_fb <= to_unsigned(243, 18);     -- screen-table step
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
                    -- roll (last field's step), melt time phases, beam origin; Eyes exponent operand
                    when 20 =>
                        if s_vneg = '1' then c_s <= c_s - resize(c_vst, 18); else c_s <= c_s + resize(c_vst, 18); end if;
                        if s_vneg = '1' then
                            c_tm1 <= c_tm1 - 339; c_tm2 <= c_tm2 + 579;
                        else
                            c_tm1 <= c_tm1 + 339; c_tm2 <= c_tm2 - 579;
                        end if;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                    when 21 =>
                        if c_xb0 > 140 then c_xsh <= c_xb0; else c_xsh <= to_unsigned(140, 12); end if;
                        v_k := s_g1(12 downto 3);
                        if v_k >= 512 then z_a <= v_k - 512; z_neg <= '0'; else z_a <= 512 - v_k; z_neg <= '1'; end if;
                    when 52 =>                        -- e1 = 0.775 (k1 - 512) / 256 octaves
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= resize(v_e(11 downto 8), 3);
                        v_k := s_g2(12 downto 3);
                        if v_k >= 512 then z_a <= v_k - 512; z_neg <= '0'; else z_a <= 512 - v_k; z_neg <= '1'; end if;
                    when 54 => z_mt <= r_ex_q;
                    when 76 =>                        -- Rings: 0.6 (k2 - 512) / 256 octaves
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0));
                        v_k := s_gp(12 downto 3);     -- Gaze: kv = 2^(-0.8 (p12 - 512) / 256)
                        if v_k >= 512 then z_a <= v_k - 512; z_neg <= '1'; else z_a <= 512 - v_k; z_neg <= '0'; end if;
                        z_e <= v_e;
                    when 78 =>                        -- kr = 2^(e / 256), Q12 (e = -1.2 .. +1.2 octaves)
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
                        -- Gaze pupil offset (Q2): -1040 (p12 - 512) / 512 above the centre, +480 (512 - p12) / 512 below
                        v_k := s_gp(12 downto 3);
                        if v_k >= 512 then z_offd <= v_k - 512; z_offn <= '1';
                        else z_offd <= 512 - v_k; z_offn <= '0'; end if;
                    when 101 =>
                        if z_offn = '1' then z_off <= shift_left(z_offd, 1);
                        else z_off <= z_offd - shift_right(z_offd, 4); end if;
                    when 102 =>                       -- kv = 2^(e / 256), Q12 (e = -1.6 .. +1.6 octaves)
                        case to_integer(z_e(11 downto 8)) is
                            when 1      => c_kv <= resize(shift_right(r_ex_q, 2), 14);
                            when 0      => c_kv <= resize(shift_right(r_ex_q, 3), 14);
                            when -1     => c_kv <= resize(shift_right(r_ex_q, 4), 14);
                            when others => c_kv <= resize(shift_right(r_ex_q, 5), 14);
                        end case;
                    when 124 =>                       -- step / h = 8923 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when 1      => c_sth1 <= f_acc(29 downto 14);
                            when 0      => c_sth1 <= f_acc(30 downto 15);
                            when -1     => c_sth1 <= f_acc(31 downto 16);
                            when others => c_sth1 <= f_acc(32 downto 17);
                        end case;
                    when 148 => c_step <= f_acc(17 downto 0);
                    when 172 =>                       -- start phase = -(2000 - xs) sth1 - 2^23 (mod 2^24)
                        v_t := f_acc(23 downto 0);
                        c_pst <= (not v_t) + 1 - to_unsigned(8388608, 24);
                    when 196 => c_kvr <= f_acc(27 downto 12);
                    when 220 => c_kg  <= f_acc(27 downto 12);
                    when 244 =>
                        if z_offn = '1' then c_cv <= to_signed(C_FPQ, 16) - signed(resize(f_acc(26 downto 12), 16));
                        else c_cv <= to_signed(C_FPQ, 16) + signed(resize(f_acc(26 downto 12), 16)); end if;
                        -- fin gain fg (Q8): k3 / 2 below the centre, 1 + 2.5 (k3 - 512) / 511 above; ron
                        v_k := s_g3(12 downto 3);
                        if v_k > 512 then z_k3d <= v_k - 512; z_k3h <= '1';
                        else z_k3d <= v_k; z_k3h <= '0'; end if;
                    when 245 =>
                        if z_k3h = '1' then z_fg <= to_unsigned(256, 10) + resize(shift_right(resize(z_k3d, 12) + shift_left(resize(z_k3d, 12), 2), 2), 10);
                        else z_fg <= resize(z_k3d(9 downto 1), 10); end if;
                        z_k3t <= shift_left(resize(z_k3d, 16), 5) + shift_left(resize(z_k3d, 16), 4);
                    when 246 =>
                        z_k3t <= z_k3t - shift_left(resize(z_k3d, 16), 1);
                    when 247 =>
                        if z_k3h = '1' then c_ron <= to_unsigned(140, 8) - resize(z_k3t(15 downto 8), 8);
                        else c_ron <= to_unsigned(140, 8); end if;
                    when 268 => c_ka <= resize(f_acc(25 downto 8), 16);
                    when 292 => c_ka <= f_acc(27 downto 12);
                    when 316 => z_k6s <= f_acc(19 downto 0);
                    when 340 => z_eg <= f_acc(27 downto 12);
                    when 364 =>
                        if s_src = '1' then c_eg <= resize(f_acc(23 downto 8), 18) + shift_left(resize(f_acc(23 downto 8), 18), 1);
                        else c_eg <= resize(f_acc(23 downto 8), 18); end if;
                    when 388 => c_t0 <= to_signed(C_FPQ, 18) + signed(resize(f_acc(27 downto 12), 18));
                    when 412 => c_ts <= f_acc(23 downto 8);
                    when 436 => c_tf <= to_signed(C_FPQ, 18) + signed(resize(f_acc(25 downto 12), 18));
                    when 460 => z_vsq <= f_acc(17 downto 0);
                    when 484 => c_vst <= f_acc(31 downto 16);
                    when 508 =>                       -- band height; lower edge = H / 2 - 305
                        if s_g5(12 downto 3) > 512 then z_h <= to_unsigned(840, 12) + f_acc(19 downto 8);
                        else z_h <= f_acc(19 downto 8); end if;
                    when 509 =>
                        if z_h > 2000 then z_h <= to_unsigned(2000, 12); end if;
                    when 510 =>
                        c_h <= z_h(10 downto 0);
                        c_loe <= signed(resize(z_h(11 downto 1), 12)) - 305;
                    when 532 => c_m1i <= f_acc(17 downto 8);
                    when 556 => c_m2i <= f_acc(17 downto 8);
                    when 580 => c_jst <= f_acc(19 downto 8);
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
    -- load + 17): table reads + interpolation, V, g, the fin depth, the red
    -- threshold, the Melt phase
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d   : signed(11 downto 0);
        variable v_j   : unsigned(8 downto 0);
        variable v_q   : unsigned(10 downto 0);
        variable v_lo  : signed(11 downto 0);
        variable v_hi  : signed(11 downto 0);
        variable v_x   : signed(16 downto 0);
        variable v_w   : signed(8 downto 0);
        variable v_r   : signed(18 downto 0);
        variable v_m   : unsigned(17 downto 0);
        variable v_pt  : unsigned(23 downto 0);
        variable v_mh  : signed(11 downto 0);
        variable v_g12 : signed(11 downto 0);
        variable v_g13 : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_jj <= (others => '0');
                e_lc <= (others => '0');
                e_m1 <= c_tm1; e_m2 <= c_tm2;
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 20);
                e_jj <= e_jj + resize(c_jst, 20);
                e_lc <= (others => '0');
                e_m1 <= e_m1 + resize(c_m1i, 16); e_m2 <= e_m2 + resize(c_m2i, 16);
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(28 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_yi <= e_vy(18 downto 8);
                when 2 =>                     -- d = (Y - 610) (x2 Twin) + s (mod 1024); Y * 243 partials
                    v_d := signed(resize(e_yi, 12)) - 610;
                    if s_twin = '1' then v_d := shift_left(v_d, 1); end if;
                    e_dd <= unsigned(v_d(9 downto 0));
                when 3 =>
                    e_d <= e_dd + c_s(17 downto 8);
                when 4 =>
                    r_sc_a <= e_d(9 downto 2); e_fi <= e_d(1 downto 0);
                when 5 =>
                    r_sc_a <= e_d(9 downto 2) + 1;
                    v_j := e_jj(18 downto 10);
                    if e_jj(19) = '1' then v_j := (others => '1'); end if;
                    if v_j > 254 then
                        r_sf_a <= to_unsigned(254, 8); e_fj <= "11";
                    else
                        r_sf_a <= v_j(7 downto 0); e_fj <= e_jj(9 downto 8);
                    end if;
                when 6 =>
                    r_sf_a <= r_sf_a + 1;
                    e_scA <= r_sc_q;
                when 7 =>
                    e_scB <= r_sc_q; e_sfA <= r_sf_q;
                when 8 =>
                    e_sfB <= r_sf_q;
                    e_dv <= resize(signed(e_scB(31 downto 19)), 15) - resize(signed(e_scA(31 downto 19)), 15);
                    e_dg <= signed(resize(unsigned(e_scB(18 downto 8)), 12)) - signed(resize(unsigned(e_scA(18 downto 8)), 12));
                    e_dw <= signed(resize(unsigned(e_scB(7 downto 0)), 9)) - signed(resize(unsigned(e_scA(7 downto 0)), 9));
                when 9 =>
                    e_dp  <= resize(signed(e_sfB(31 downto 18)), 15) - resize(signed(e_sfA(31 downto 18)), 15);
                    e_dgf <= signed(resize(unsigned(e_sfB(17 downto 6)), 13)) - signed(resize(unsigned(e_sfA(17 downto 6)), 13));
                    -- scroll interpolation terms: diff * f (f = 0..3)
                    e_tv <= (others => '0'); e_tg <= (others => '0'); e_tw <= (others => '0');
                    if e_fi = "01" then
                        e_tv <= resize(e_dv, 17); e_tg <= resize(e_dg, 14); e_tw <= resize(e_dw, 11);
                    elsif e_fi = "10" then
                        e_tv <= shift_left(resize(e_dv, 17), 1); e_tg <= shift_left(resize(e_dg, 14), 1);
                        e_tw <= shift_left(resize(e_dw, 11), 1);
                    elsif e_fi = "11" then
                        e_tv <= resize(e_dv, 17) + shift_left(resize(e_dv, 17), 1);
                        e_tg <= resize(e_dg, 14) + shift_left(resize(e_dg, 14), 1);
                        e_tw <= resize(e_dw, 11) + shift_left(resize(e_dw, 11), 1);
                    end if;
                when 10 =>
                    e_tp <= (others => '0'); e_tgf <= (others => '0');
                    if e_fj = "01" then e_tp <= resize(e_dp, 17); e_tgf <= resize(e_dgf, 15);
                    elsif e_fj = "10" then e_tp <= shift_left(resize(e_dp, 17), 1); e_tgf <= shift_left(resize(e_dgf, 15), 1);
                    elsif e_fj = "11" then
                        e_tp <= resize(e_dp, 17) + shift_left(resize(e_dp, 17), 1);
                        e_tgf <= resize(e_dgf, 15) + shift_left(resize(e_dgf, 15), 1);
                    end if;
                    e_vs <= resize(signed(e_scA(31 downto 19)), 16) + resize(shift_right(e_tv, 2), 16);
                    v_g12 := signed(resize(unsigned(e_scA(18 downto 8)), 12)) + resize(shift_right(e_tg, 2), 12);
                    e_gs <= unsigned(v_g12(10 downto 0));
                    v_w := signed(resize(unsigned(e_scA(7 downto 0)), 9)) + resize(shift_right(e_tw, 2), 9);
                    e_wn <= unsigned(v_w(7 downto 0));
                when 11 =>
                    if s_twin = '1' then
                        e_pp <= (others => '0'); e_gf <= to_unsigned(1024, 12);
                    else
                        e_pp <= resize(signed(e_sfA(31 downto 18)), 16) + resize(shift_right(e_tp, 2), 16);
                        v_g13 := signed(resize(unsigned(e_sfA(17 downto 6)), 13)) + resize(shift_right(e_tgf, 2), 13);
                        e_gf <= unsigned(v_g13(11 downto 0));
                    end if;
                when 12 =>                    -- X = Vs + P - FP
                    e_xt <= resize(e_vs, 17) + resize(e_pp, 17);
                when 13 =>
                    e_x <= e_xt - C_FPQ;
                when 14 =>
                    if e_x(16) = '1' then m_a <= resize(unsigned(-e_x), 30); e_xn <= '1';
                    else m_a <= resize(unsigned(e_x), 30); e_xn <= '0'; end if;
                    m_b <= c_kvr; m_acc <= (others => '0');
                when 31 =>                    -- V = CV +- X KVR >> 12
                    if e_xn = '1' then n_v <= resize(c_cv, 18) - signed(resize(m_acc(29 downto 12), 18));
                    else n_v <= resize(c_cv, 18) + signed(resize(m_acc(29 downto 12), 18)); end if;
                    m_a <= resize(e_gs, 30); m_b <= resize(e_gf, 16); m_acc <= (others => '0');
                when 48 =>                    -- gs Gf >> 10, then x KG >> 12
                    m_a <= resize(m_acc(23 downto 10), 30); m_b <= c_kg; m_acc <= (others => '0');
                when 65 =>
                    if m_acc(29 downto 24) /= 0 then n_g <= (others => '1'); else n_g <= m_acc(23 downto 12); end if;
                    if e_wn > c_ron then m_a <= resize(e_wn - c_ron, 30); else m_a <= (others => '0'); end if;
                    m_b <= c_ka; m_acc <= (others => '0');
                when 82 =>                    -- fin depth; red band position q = (LO - Y - 2 s) mod 2048
                    if m_acc(29 downto 22) /= 0 then n_a <= (others => '1'); else n_a <= m_acc(21 downto 8); end if;
                    e_q <= unsigned(c_loe(10 downto 0)) - resize(e_yi, 11);
                when 83 =>
                    e_q <= e_q - (c_s(17 downto 8) & '0');
                when 84 =>                    -- q = lines above the band's lower edge (mod 2048)
                    e_elo <= signed(resize(e_q, 12));
                    e_qn <= (not e_q) + 1;
                when 85 =>
                    e_ehi <= e_elo - signed(resize(c_h, 12));
                when 86 =>
                    if c_h < 4 then
                        e_tz <= "00";
                    elsif e_q < 100 then
                        e_tz <= "10"; e_te <= e_q(6 downto 0); e_ten <= '1';
                    elsif e_q > 1948 then
                        e_tz <= "10"; e_te <= e_qn(6 downto 0); e_ten <= '0';
                    elsif e_ehi > -100 and e_ehi < 100 then
                        e_tz <= "11";
                        if e_ehi < 0 then e_te <= unsigned(resize(-e_ehi, 7)); e_ten <= '1';
                        else e_te <= unsigned(e_ehi(6 downto 0)); e_ten <= '0'; end if;
                    elsif e_ehi < 0 then
                        e_tz <= "01";
                    else
                        e_tz <= "00";
                    end if;
                when 87 =>
                    m_a <= resize(e_te, 30); m_b <= c_ts; m_acc <= (others => '0');
                    -- Melt: two rolling triangles (140 and 70 h peak), centred
                    e_mt1 <= f_tri7(e_m1(15 downto 8)); e_mt2 <= f_tri7(e_m2(15 downto 8));
                when 88 =>
                    e_p1 <= shift_left(resize(e_mt1, 14), 6) + shift_left(resize(e_mt1, 14), 2) + shift_left(resize(e_mt1, 14), 1);
                    e_p2 <= shift_left(resize(e_mt2, 14), 5) + shift_left(resize(e_mt2, 14), 1) + resize(e_mt2, 14);
                when 89 =>
                    v_mh := signed(resize(e_p1(13 downto 5), 12)) + signed(resize(e_p2(13 downto 5), 12)) - 210;
                    if s_melt = '1' then e_mh <= v_mh(9 downto 0); else e_mh <= (others => '0'); end if;
                when 104 =>                   -- red threshold
                    case e_tz is
                        when "00"   => n_t <= to_signed(C_TPOS, 18);
                        when "01"   => n_t <= to_signed(C_TNEG, 18);
                        when others =>
                            if e_ten = '1' then n_t <= c_t0 - signed(resize(m_acc(23 downto 8), 18));
                            else n_t <= c_t0 + signed(resize(m_acc(23 downto 8), 18)); end if;
                    end case;
                    if e_mh(9) = '1' then m_a <= resize(unsigned(-e_mh), 30); else m_a <= resize(unsigned(e_mh), 30); end if;
                    m_b <= c_sth1; m_acc <= (others => '0');
                when 121 =>                   -- line start phase (+ melt)
                    v_pt := m_acc(23 downto 0);
                    if e_mh(9) = '1' then n_p0 <= c_pst - v_pt; else n_p0 <= c_pst + v_pt; end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the triangle oscillator's phase (held left of x = 70 virtual px),
    -- the Osc lattice
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_p <= n_p0;
                b_x <= c_xb0;
                if c_xb0 + s_xs > c_xsh then b_run <= '1'; else b_run <= '0'; end if;
                l_v <= n_v; l_g <= n_g; l_a <= n_a; l_t <= n_t; l_step <= c_step;
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            elsif g_avid = '1' then
                b_x <= b_x + s_xs;
                if b_run = '1' then b_p <= b_p + resize(l_step, 24); end if;
                if b_x + shift_left(resize(s_xs, 12), 1) > c_xsh then b_run <= '1'; end if;
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
        variable v_za  : unsigned(6 downto 0);
        variable v_g   : unsigned(21 downto 0);
        variable v_az  : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: triangle -> hn; ROM addresses; luma (Video: Y >> 2 - 16; Osc: the lattice)
            if b_p(23) = '1' then v_hn := not b_p(22 downto 13); else v_hn := b_p(22 downto 13); end if;
            r_s_a <= s_shape & v_hn(9 downto 1);
            r_z_a <= v_hn(9 downto 1);
            if s_src = '1' then
                v_os := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
                q2_lm <= v_os;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then q2_lm <= v_l - 16; else q2_lm <= (others => '0'); end if;
            end if;
            -- 3: ROMs
            q3_lm <= q2_lm;
            -- 4: fabric copies; etch partials (luma nibbles x gain)
            q4_s <= r_s_q;
            q4_z <= signed(r_z_q);
            q4_el <= resize(q3_lm(3 downto 0) * c_eg, 22);
            q4_eh <= resize(q3_lm(7 downto 4) * c_eg, 22);
            -- 5: g x S partials (S split 5 / 5), a x |Z| partials (|Z| split 3 / 4), etch sum
            q5_gh <= resize(l_g * q4_s(9 downto 5), 17);
            q5_gl <= resize(l_g * q4_s(4 downto 0), 17);
            if q4_z(7) = '1' then v_za := unsigned(not q4_z(6 downto 0)); q5_zn <= '1';
            else v_za := unsigned(q4_z(6 downto 0)); q5_zn <= '0'; end if;
            q5_ah <= resize(l_a * v_za(6 downto 4), 17);
            q5_al <= resize(l_a * v_za(3 downto 0), 18);
            q5_et <= resize(q4_el, 26) + shift_left(resize(q4_eh, 26), 4);
            -- 6: sums >> scale
            v_g := shift_left(resize(q5_gh, 22), 5) + resize(q5_gl, 22);
            q6_gs <= v_g(21 downto 8) srl 1;
            v_az := shift_left(resize(q5_ah, 21), 4) + resize(q5_al, 21);
            q6_az <= signed(resize(v_az(20 downto 8), 15)); q6_zn <= q5_zn;
            q6_et <= q5_et(25 downto 8);
            -- 7, 8: F = V + gS + aZ + etch (Q2)
            q7_f1 <= l_v + signed(resize(q6_gs, 18));
            if q6_zn = '1' then q7_f2 <= signed(resize(q6_et, 18)) - resize(q6_az, 18);
            else q7_f2 <= signed(resize(q6_et, 18)) + resize(q6_az, 18); end if;
            q8_f  <= q7_f1 + q7_f2;
            -- 9: palette address (F / 2 + 320, wrapping); red band and Fire tests
            r_p_a <= unsigned(q8_f(12 downto 3)) + 320;
            if q8_f > l_t then q_red(9) <= '1'; else q_red(9) <= '0'; end if;
            if s_fire = '1' and q8_f < c_tf then q_fir(9) <= '1'; else q_fir(9) <= '0'; end if;
            -- 10: palette EBR; 11: fabric copy
            q11_p <= r_p_q;
            -- 12: the red level (Fire: bright, 255 - R / 2) -> red ROM address; the palette colour x 4
            if q_fir(11) = '1' then r_rd_a <= 255 - ('0' & unsigned(q11_p(7 downto 1)));
            else r_rd_a <= unsigned(q11_p(7 downto 0)); end if;
            q12_y <= unsigned(q11_p(31 downto 24)) & "00";
            q12_u <= unsigned(q11_p(23 downto 16)) & "10";
            q12_v <= unsigned(q11_p(15 downto 8)) & "10";
            -- 13: red ROM (Y = 64 + 1.453 r, Cr = 512 + 1.641 r, Cb = 512 - 0.625 r; luma pre-scaled 16/13)
            q13_y <= q12_y; q13_u <= q12_u; q13_v <= q12_v;
            -- 14: fabric copies
            q14_ry <= unsigned(r_rd_q(29 downto 20)); q14_ru <= unsigned(r_rd_q(19 downto 10)); q14_rv <= unsigned(r_rd_q(9 downto 0));
            q14_y <= q13_y; q14_u <= q13_u; q14_v <= q13_v;
            for i in 10 to 14 loop q_red(i) <= q_red(i - 1); q_fir(i) <= q_fir(i - 1); end loop;
            -- 15: compose; neutral outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            elsif q_red(14) = '1' or q_fir(14) = '1' then
                s_y15 <= q14_ry; s_u15 <= q14_ru; s_v15 <= q14_rv;
            else
                s_y15 <= q14_y; s_u15 <= q14_u; s_v15 <= q14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
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
            -- stage 16 sums, stage 17 output; the 8-sums snap back to neutral once 8 blank samples are in
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u15 - s_cau(7);
                s_bav <= s_bav + s_v15 - s_cav(7);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            -- luma [1 2 1] centred on the box's centre sample (4 back)
            v_yy := resize(s_cay(3), 12) + shift_left(resize(s_cay(4), 12), 1) + resize(s_cay(5), 12);
            s_y17 <= v_yy(11 downto 2);
            -- 18, 19: delay
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
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

end architecture owlet;
