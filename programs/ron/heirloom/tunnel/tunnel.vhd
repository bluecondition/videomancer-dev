-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tunnel v0.1.0 -- Heirloom (video phase), from the clip WIN_20230723_00_52_24 at 0.5 s.
--
-- The still: a black and white relief.  Concentric diamond glow rings around an "eye" (centre x 1000, y 512 of
-- 1920 x 1080) stream out of the middle; a static weave of horizontal stripes (17 per frame) and dark vertical
-- bars (20 per line, bent into "<" chevrons near the axis) fills the frame; two eye corners 255 px either side of
-- the centre throw a fan of three thin rays each way, the rings stop inside the fans and the stripes stop above
-- the corners.  Everything is screened together and camera-softened.
--
-- The patch (tunnel.py and .heirloom_work/rb_tunnel/imodel.py hold every constant):
--   D      = a T(|dx|) + |dy| (1 -/+ skew)    T: parabola to 128 px, linear, x3 past 690 px (a per-field table)
--   ring   = profile(frac((D - flow) G + etch))                    G = rings per px (P12), etch = camera / Osc
--   weave  = V(x) (g0 + Ah (H(y) - g0))      H: 17 stripes per frame, V: 20 bars per line, chevron 0.625 |dy|
--   rays   = |dy| = n rs (|dx| - Xe), n = 1..3, outside the eye; the fan = inside the outer ray
--   out    = screen(weave, ring (1 - fan) + rays / 2), luma -> grey with a cool / green tint (a colour ROM)
--
--   K1  Slope     the diamonds' aspect: tall (0) .. the still (512) .. flat (1023), x1/4 .. x4
--   K2  Skew      the rings bunch toward the top or the bottom (bipolar); the clip's sway LFO rides on it
--   K3  Weave     stripe and bar density, x1/3 .. x3
--   K4  Speed     bipolar: rings stream out of (+) or into (-) the eye
--   K5  Eye       the eye corners' spread (40 .. 255 .. 700 px) and the fan (wide .. the still .. narrow)
--   K6  Etch      how far the camera (or the Osc, 4x) bends the rings
--   S7  Twin      one eye / two eyes side by side
--   S8  Rings     Glow (the still) / Line (thin outlines)
--   S9  Weave     On (the still) / Off (rings and rays on black)
--   S10 Ink       Light (the still) / Dark (negative)
--   S11 Source    Video / Osc
--   P12 DEPTH     ring density: one giant ring (0) .. the still (400) .. a deep tunnel of fine rings (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, the table fill), a per-line engine kicked at
-- each line start (computes the NEXT line), a 28-stage pixel path.  Virtual 1080-line coordinates (SD shows the
-- centre 4:3).  Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the
-- blank resync, luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tunnel_pkg.all;
use work.all;

architecture tunnel of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    constant C_PH0  : integer := 49770;    -- ring phase offset (180 px of D at the still's gain)
    constant C_CXH  : integer := 2000;     -- eye centre x (h = half virtual px)
    constant C_CY8  : integer := 131072;   -- eye centre y, Q8 virtual lines (512)
    constant C_G0   : integer := 70784;    -- ring gain at P12 = 400, Q8 phase per px (2^16 / 237 px)
    constant C_KNEE : integer := 690;      -- |dx| where the diamond's side steepens x3
    constant C_GZ   : integer := 61;       -- weave level where the stripes are keyed off

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
    signal r_pf_a : unsigned(9 downto 0) := (others => '0');
    signal r_pf_q : unsigned(7 downto 0) := (others => '0');
    signal r_cl_a : unsigned(7 downto 0) := (others => '0');
    signal r_cl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- diamond x-term table: G a T(r) mod one ring (Q16 phase) by r = |dx| in virtual px; written by the
    -- per-field DDA in the vblank zone only, read by the pixel path during active video (never both)
    type t_tb is array (0 to 1023) of unsigned(15 downto 0);
    signal tb : t_tb := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of tb : signal is true;
    signal t_we, t_run : std_logic := '0';
    signal t_wa, t_ra : unsigned(9 downto 0) := (others => '0');
    signal t_wd, t_rq : unsigned(15 downto 0) := (others => '0');
    signal t_i  : unsigned(9 downto 0) := (others => '0');
    signal t_q, t_d : unsigned(23 downto 0) := (others => '0');
    signal t_fp, t_fl, t_fk : std_logic := '0';

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

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(160 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_line, s_woff, s_ink, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(9 downto 0) := (others => '0');
    signal c_g    : unsigned(18 downto 0) := to_unsigned(C_G0, 19);    -- ring gain, Q8 phase per px
    signal c_a    : unsigned(9 downto 0) := to_unsigned(179, 10);      -- slope a, Q8
    signal c_ga   : unsigned(28 downto 0) := (others => '0');          -- G a, Q16.16 phase per px
    signal c_sk   : signed(9 downto 0) := (others => '0');             -- skew, Q8
    signal c_gup, c_gdn : unsigned(19 downto 0) := to_unsigned(C_G0, 20);
    signal c_lf   : unsigned(15 downto 0) := to_unsigned(46625, 16);   -- sway LFO phase (the still: +9 falling)
    signal c_tri  : unsigned(14 downto 0) := (others => '0');
    signal c_lfo  : signed(7 downto 0) := (others => '0');
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_step : unsigned(19 downto 0) := (others => '0');
    signal c_fl   : unsigned(23 downto 0) := (others => '0');          -- ring flow, Q8 phase
    signal c_ph0f : unsigned(15 downto 0) := to_unsigned(C_PH0, 16);   -- PH0 + flow
    signal c_kv8  : unsigned(19 downto 0) := to_unsigned(174763, 20);  -- V bars, Q8 phase per virtual px (2^24 / cycle)
    signal c_kvx  : unsigned(20 downto 0) := to_unsigned(174763, 21);  -- per output pixel
    signal c_vx   : unsigned(23 downto 0) := (others => '0');          -- 26 px of V phase
    signal c_hk8  : unsigned(19 downto 0) := to_unsigned(264090, 20);  -- H stripes, Q8 phase per virtual line
    signal c_hst  : unsigned(23 downto 0) := to_unsigned(264090, 24);  -- per output line
    signal c_xe   : unsigned(9 downto 0) := to_unsigned(255, 10);      -- eye corner |dx| (px)
    signal c_rs   : unsigned(6 downto 0) := to_unsigned(37, 7);        -- ray slope step, Q8
    signal c_eg   : unsigned(15 downto 0) := to_unsigned(400, 16);     -- etch gain
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);      -- beam start x (h)
    signal c_xs8  : unsigned(16 downto 0) := to_unsigned(2048, 17);    -- beam start x, Q8 px
    signal c_curve : unsigned(1 downto 0) := "00";

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
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

    -- line engine (computes the next line)
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_hph  : unsigned(23 downto 0) := (others => '0');
    signal e_ady  : unsigned(17 downto 0) := (others => '0');
    signal e_top  : std_logic := '0';
    signal e_cq   : unsigned(17 downto 0) := (others => '0');
    signal e_yt   : unsigned(15 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(19 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal n_yt   : unsigned(15 downto 0) := (others => '0');
    signal n_vph  : unsigned(23 downto 0) := (others => '0');
    signal n_ady2 : unsigned(11 downto 0) := (others => '0');
    signal n_top  : std_logic := '0';
    signal n_hs   : unsigned(7 downto 0) := (others => '0');
    signal y_cnt  : unsigned(2 downto 0) := (others => '0');
    signal t_hsel : std_logic := '0';

    -- beam and line registers
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_yt  : unsigned(15 downto 0) := (others => '0');
    signal l_ady2 : unsigned(11 downto 0) := (others => '0');
    signal l_top : std_logic := '0';
    signal l_hs  : unsigned(7 downto 0) := (others => '0');

    -- ray profile: 24 - |d| for |d| < 24 (quarter px), else 0
    function f_ray(d : signed(12 downto 0)) return unsigned is
        variable v : signed(12 downto 0);
    begin
        if d >= 0 then v := 24 - d; else v := 24 + d; end if;
        if v > 0 then return unsigned(v(4 downto 0)); else return "00000"; end if;
    end function;

    -- pixel path
    signal q2_dx  : signed(12 downto 0) := (others => '0');
    signal q3_px  : unsigned(10 downto 0) := (others => '0');
    signal q4_t   : signed(11 downto 0) := (others => '0');
    signal q4_px  : unsigned(10 downto 0) := (others => '0');
    signal q5_adx, q6_adx : unsigned(9 downto 0) := (others => '0');
    signal q7_e   : signed(10 downto 0) := (others => '0');
    signal q7_far, q8_far, q9_far, q10_far : unsigned(7 downto 0) := (others => '0');
    signal q7_tx  : unsigned(15 downto 0) := (others => '0');
    signal q8_r1  : unsigned(16 downto 0) := (others => '0');
    signal q8_hn  : unsigned(11 downto 0) := (others => '0');
    signal q8_ph, q9_ph : unsigned(15 downto 0) := (others => '0');
    signal q8_epos, q8_eneg, q9_epos, q9_eneg, q10_epos, q10_eneg, q11_epos, q11_eneg, q12_eneg : std_logic := '0';
    signal q9_r1, q9_kop : unsigned(11 downto 0) := (others => '0');
    signal q10_d1, q10_d2, q10_d3, q10_u : signed(12 downto 0) := (others => '0');
    signal d_va   : unsigned(15 downto 0) := (others => '0');
    signal d_vph  : unsigned(23 downto 0) := (others => '0');
    signal q11_v1, q11_v2, q11_v3, q12_vm : unsigned(4 downto 0) := (others => '0');
    signal q11_fr, q11_hv : unsigned(7 downto 0) := (others => '0');
    signal q11_vd : signed(16 downto 0) := (others => '0');
    signal q12_fan, q12_ah, q12_ring, q12_vb : unsigned(7 downto 0) := (others => '0');
    signal q13_ray, q13_ring, q13_vb, q13_hs : unsigned(7 downto 0) := (others => '0');
    signal q13_ah4, q13_f4 : unsigned(3 downto 0) := (others => '0');
    signal q13_fq : unsigned(1 downto 0) := (others => '0');
    signal q14_v, q14_rg, q14_ray : unsigned(7 downto 0) := (others => '0');
    signal q14_am : unsigned(11 downto 0) := (others => '0');
    signal q14_gm : unsigned(9 downto 0) := (others => '0');
    signal q15_v, q15_b, q15_rg, q15_ray : unsigned(7 downto 0) := (others => '0');
    signal q16_bm  : unsigned(15 downto 0) := (others => '0');
    signal q16_lit : unsigned(7 downto 0) := (others => '0');
    signal q17_bl  : unsigned(15 downto 0) := (others => '0');
    signal q17_s   : unsigned(8 downto 0) := (others => '0');
    signal q18_lum : unsigned(7 downto 0) := (others => '0');
    -- camera / Osc etch
    signal k_d1, k_d2, k_d3, k_d4, k_d5, k_d6, k_d7 : unsigned(7 downto 0) := (others => '0');
    signal k_s4a, k_s4b : unsigned(9 downto 0) := (others => '0');
    signal k_sum  : unsigned(10 downto 0) := (others => '0');
    signal k_pa, k_pb : unsigned(19 downto 0) := (others => '0');
    signal k_et   : unsigned(15 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_ky, c_ku, c_kv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
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
            r_ex_q <= C_TN_EXP2(to_integer(r_ex_a));
            r_pf_q <= C_TN_PROF(to_integer(r_pf_a));
            r_cl_q <= C_TN_COL(to_integer(r_cl_a));
        end if;
    end process p_rom;

    p_tb : process(clk)
    begin
        if rising_edge(clk) then
            t_rq <= tb(to_integer(t_ra));
            if t_we = '1' then tb(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_tb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_t  : unsigned(35 downto 0);
        variable v_sk : signed(10 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_l  : unsigned(21 downto 0);
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
            -- speed: +-8 deadband, magnitude and sign
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
                s_twin <= registers_in(6)(0);
                s_line <= registers_in(6)(1);
                s_woff <= registers_in(6)(2);
                s_ink  <= registers_in(6)(3);
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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from load + 17
            -- (stored at load + 18).  The slot number is decoded at 30 + 24 k and the operands pre-muxed
            -- into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
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
                    when 222 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_a, 36);                    -- |p12 - 400| x 285 / 245
                               if z_neg = '1' then z_fb <= to_unsigned(285, 16); else z_fb <= to_unsigned(245, 16); end if;
                    when 1  => z_fa <= to_unsigned(C_G0, 36);             z_fb <= z_mt;                 -- G
                    when 2  => z_fa <= to_unsigned(179, 36);              z_fb <= z_mt;                 -- A
                    when 3  => z_fa <= resize(c_g, 36);                   z_fb <= resize(c_a, 16);      -- G a
                    when 4  => z_fa <= resize(z_a, 36);                   z_fb <= to_unsigned(77, 16);  -- |skew knob|
                    when 5  => z_fa <= resize(c_g, 36);                                                 -- G (1 + s)
                               z_fb <= resize(unsigned(to_signed(256, 11) + resize(c_sk, 11)), 16);
                    when 6  => z_fa <= resize(c_g, 36);                                                 -- G (1 - s)
                               z_fb <= resize(unsigned(to_signed(256, 11) - resize(c_sk, 11)), 16);
                    when 7  => z_fa <= resize(s_vm, 36);                  z_fb <= resize(s_vm, 16);     -- speed^2
                    when 8  => z_fa <= resize(z_a, 36);                   z_fb <= to_unsigned(203, 16); -- |k3 - 512|
                    when 9  => z_fa <= to_unsigned(174763, 36);           z_fb <= z_mt;                 -- V bars
                    when 10 => z_fa <= to_unsigned(264090, 36);           z_fb <= z_mt;                 -- H stripes
                    when 11 => z_fa <= resize(c_hk8, 36);                 z_fb <= resize(s_ys, 16);     -- per line
                    when 12 => z_fa <= resize(z_a, 36);                                                 -- eye corner
                               if z_neg = '1' then z_fb <= to_unsigned(215, 16); else z_fb <= to_unsigned(445, 16); end if;
                    when 13 => z_fa <= resize(z_a, 36);                                                 -- ray slope
                               if z_neg = '1' then z_fb <= to_unsigned(40, 16); else z_fb <= to_unsigned(19, 16); end if;
                    when others => z_fa <= resize(s_g6(12 downto 3), 36); z_fb <= resize(s_g6(12 downto 3), 16); -- etch^2
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
                    -- motion (last field's steps), sway LFO, beam origin
                    when 20 =>
                        if s_vneg = '1' then c_fl <= c_fl - resize(c_step, 24);
                        else c_fl <= c_fl + resize(c_step, 24); end if;
                        c_lf <= c_lf + 148;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                    when 21 =>                                       -- sway LFO: triangle 0 .. 32767
                        if c_lf(15) = '1' then c_tri <= not c_lf(14 downto 0); else c_tri <= c_lf(14 downto 0); end if;
                        c_xs8 <= shift_left(resize(c_xb0, 17), 7);   -- h -> Q8 px
                        c_ph0f <= to_unsigned(C_PH0, 16) + c_fl(23 downto 8);
                        v_k := C_TN_COL(0);
                        c_ky <= unsigned(v_k(31 downto 22)); c_ku <= unsigned(v_k(21 downto 12)); c_kv <= unsigned(v_k(11 downto 2));
                    when 22 =>                                       -- lfo = (tri * 124 >> 15) - 62 (+-0.24)
                        v_l := shift_left(resize(c_tri, 22), 7) - shift_left(resize(c_tri, 22), 2);
                        c_lfo <= signed(resize(v_l(21 downto 15), 8)) - 62;
                    when 23 =>                                       -- P12 = 400 is the still
                        if s_gp(12 downto 3) < 400 then z_a <= 400 - s_gp(12 downto 3); z_neg <= '1';
                        else z_a <= s_gp(12 downto 3) - 400; z_neg <= '0'; end if;

                    -- G = 70784 * 2^(e / 256), e = +-(|p12 - 400| * 285 or 245) >> 8
                    when 50  =>
                        if z_neg = '1' then z_e <= -signed(resize(f_acc(19 downto 8), 12));
                        else z_e <= signed(resize(f_acc(19 downto 8), 12)); end if;
                    when 51  => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 53  => z_mt <= r_ex_q;
                    when 74  =>
                        v_t := f_acc;
                        case to_integer(z_sh) is
                            when -2     => c_g <= v_t(35 downto 17);
                            when -1     => c_g <= v_t(34 downto 16);
                            when 0      => c_g <= v_t(33 downto 15);
                            when 1      => c_g <= v_t(32 downto 14);
                            when others => c_g <= v_t(31 downto 13);
                        end case;
                        -- slope exponent: k1 - 512 (Q8 octaves, +-2)
                        v_s := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_s(7 downto 0)); z_sh <= v_s(10 downto 8);
                    when 77  => z_mt <= r_ex_q;
                    when 98  =>
                        v_t := f_acc;
                        case to_integer(z_sh) is
                            when -2     => c_a <= v_t(26 downto 17);
                            when -1     => c_a <= v_t(25 downto 16);
                            when 0      => c_a <= v_t(24 downto 15);
                            when others => c_a <= v_t(23 downto 14);
                        end case;
                        -- skew knob, +-8 deadband
                        v_s := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        if v_s > 8 then z_a <= unsigned(v_s(9 downto 0)) - 8; z_neg <= '0';
                        elsif v_s < -8 then z_a <= unsigned(-v_s(9 downto 0)) - 8; z_neg <= '1';
                        else z_a <= (others => '0'); z_neg <= '0'; end if;
                    when 122 => c_ga <= f_acc(28 downto 0);
                    when 146 =>
                        if z_neg = '1' then v_sk := -signed(resize(f_acc(17 downto 8), 11));
                        else v_sk := signed(resize(f_acc(17 downto 8), 11)); end if;
                        v_sk := v_sk + resize(c_lfo, 11);
                        if v_sk > 224 then c_sk <= to_signed(224, 10);
                        elsif v_sk < -224 then c_sk <= to_signed(-224, 10);
                        else c_sk <= v_sk(9 downto 0); end if;
                    when 170 => c_gup <= f_acc(27 downto 8);
                    when 194 => c_gdn <= f_acc(27 downto 8);
                        -- weave exponent: |k3 - 512| * 203 >> 8
                        if s_g3(12 downto 3) < 512 then z_a <= 512 - s_g3(12 downto 3); z_neg <= '1';
                        else z_a <= s_g3(12 downto 3) - 512; z_neg <= '0'; end if;
                    when 218 => c_vsq <= f_acc(17 downto 0);
                    when 219 => c_step <= shift_left(resize(c_vsq, 20), 1) + resize(c_vsq(17 downto 1), 20);   -- 5/2 v^2
                    when 242 =>
                        if z_neg = '1' then z_e <= -signed(resize(f_acc(17 downto 8), 12));
                        else z_e <= signed(resize(f_acc(17 downto 8), 12)); end if;
                    when 243 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 245 => z_mt <= r_ex_q;
                    when 266 =>
                        v_t := f_acc;
                        case to_integer(z_sh) is
                            when -2     => c_kv8 <= resize(v_t(35 downto 17), 20);
                            when -1     => c_kv8 <= v_t(35 downto 16);
                            when 0      => c_kv8 <= v_t(34 downto 15);
                            when others => c_kv8 <= v_t(33 downto 14);
                        end case;
                    when 267 =>
                        case to_integer(s_xs) is
                            when 4      => c_kvx <= shift_left(resize(c_kv8, 21), 1);
                            when 3      => c_kvx <= resize(c_kv8, 21) + resize(c_kv8(19 downto 1), 21);
                            when others => c_kvx <= resize(c_kv8, 21);
                        end case;
                        c_vx <= shift_left(resize(c_kv8, 24), 4) + shift_left(resize(c_kv8, 24), 3) + shift_left(resize(c_kv8, 24), 1);
                    when 290 =>
                        v_t := f_acc;
                        case to_integer(z_sh) is
                            when -2     => c_hk8 <= resize(v_t(35 downto 17), 20);
                            when -1     => c_hk8 <= v_t(35 downto 16);
                            when 0      => c_hk8 <= v_t(34 downto 15);
                            when others => c_hk8 <= v_t(33 downto 14);
                        end case;
                    when 314 => c_hst <= resize(f_acc(31 downto 8), 24);
                        -- eye corner / ray slope operand: k5 below or above 512
                        if s_g5(12 downto 3) < 512 then z_a <= s_g5(12 downto 3); z_neg <= '1';
                        else z_a <= s_g5(12 downto 3) - 512; z_neg <= '0'; end if;
                    when 338 =>
                        if z_neg = '1' then c_xe <= to_unsigned(40, 10) + f_acc(18 downto 9);
                        else c_xe <= to_unsigned(255, 10) + f_acc(18 downto 9); end if;
                    when 362 =>
                        if z_neg = '1' then c_rs <= to_unsigned(77, 7) - f_acc(15 downto 9);
                        else c_rs <= to_unsigned(37, 7) - f_acc(15 downto 9); end if;
                    when 386 =>
                        if s_src = '1' then c_eg <= f_acc(19 downto 6) & "00";
                        else c_eg <= f_acc(21 downto 6); end if;
                        if s_line = '1' then c_curve <= "10";
                        elsif s_gp(12 downto 3) >= 768 or s_gp(12 downto 3) < 160 then c_curve <= "01";
                        else c_curve <= "00"; end if;
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
    -- Diamond table fill (vblank zone, zc 420 .. 1444): Q(r) = G a T(r), T = r^2 / 256 to r = 128, then r - 64,
    -- slope x3 from r = 690.  Second-order DDA (Q16.8, mod 2^24): Q += D; D += Ga / 128 in the parabola, D = Ga at
    -- r = 128, D = 3 Ga at the knee.  The table keeps the integer part (Q16 phase, mod one ring).
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            t_we <= '0';
            if s_zone = '1' and s_zc = 420 then
                t_run <= '1'; t_i <= (others => '0'); t_q <= (others => '0');
                t_d <= resize(c_ga(28 downto 16), 24);
                t_fp <= '1'; t_fl <= '0'; t_fk <= '0';
            elsif t_run = '1' then
                t_we <= '1'; t_wa <= t_i; t_wd <= t_q(23 downto 8);
                t_q <= t_q + t_d;
                if t_fk = '1' then
                    t_d <= resize(c_ga(28 downto 8), 24) + shift_left(resize(c_ga(28 downto 8), 24), 1);
                elsif t_fl = '1' then
                    t_d <= resize(c_ga(28 downto 8), 24);
                elsif t_fp = '1' then
                    t_d <= t_d + resize(c_ga(28 downto 15), 24);
                end if;
                -- flags: D(r + 1) = D(r) + d2 for r <= 126 (parabola), Ga at r = 127, 3 Ga at r = 689 (the knee)
                if t_i = 126 then t_fp <= '0'; t_fl <= '1';
                elsif t_i = 127 then t_fl <= '0';
                end if;
                if t_i = C_KNEE - 2 then t_fk <= '1'; else t_fk <= '0'; end if;
                t_i <= t_i + 1;
                if t_i = 1023 then t_run <= '0'; end if;
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
    -- Line engine: kicked at each line start (and once in the zone for line 0), computes the NEXT line:
    -- the ring y term |dy| G (1 -/+ skew) - PH0 - flow, the V-bar start phase (x0 + 5/8 min(|dy|, 250)) kv - 26 kv,
    -- |dy| (Q4) and the H stripe level (read from the profile ROM at the end of the active line)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1500 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_hph <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_hph <= e_hph + c_hst;
                e_lc <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(19 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_d := signed(resize(e_vy, 20)) - C_CY8;
                    if v_d < 0 then v_d := -v_d; e_top <= '1'; else e_top <= '0'; end if;
                    e_ady <= unsigned(v_d(17 downto 0));
                when 2 =>
                    if e_top = '1' then m_a <= resize(c_gup, 32); else m_a <= resize(c_gdn, 32); end if;
                    m_b <= resize(e_ady, 20); m_acc <= (others => '0');
                    if e_ady > 64000 then e_cq <= to_unsigned(64000, 18); else e_cq <= e_ady; end if;
                when 3 =>
                    e_cq <= resize(shift_right(shift_left(resize(e_cq, 21), 2) + resize(e_cq, 21), 3), 18);
                when 23 =>
                    e_yt <= m_acc(31 downto 16);
                    m_a <= resize(c_kv8, 32); m_b <= resize(c_xs8, 20) + resize(e_cq, 20); m_acc <= (others => '0');
                when 24 =>
                    n_yt <= e_yt - c_ph0f;
                    n_ady2 <= e_ady(17 downto 6);
                    n_top <= e_top;
                when 46 =>
                    n_vph <= m_acc(31 downto 8) - c_vx;
                when others => null;
            end case;

            -- H stripe level of the next line: read the profile ROM (curve 3) once the line's last pixel has
            -- left the pixel path's ROM stage (or in the zone for line 0)
            if (s_gav_sr(14) = '0' and s_gav_sr(15) = '1') or (s_zone = '1' and s_zc = 1540) then
                y_cnt <= to_unsigned(1, 3);
            elsif y_cnt /= 0 then
                y_cnt <= y_cnt + 1;
            end if;
            if y_cnt = 1 then t_hsel <= '1'; else t_hsel <= '0'; end if;
            if y_cnt = 4 then n_hs <= r_pf_q; end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: virtual x (h units) and the V-bar phase, one step per output pixel; line registers load at g_start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x   <= c_xb0;
                b_px  <= (others => '0');
                l_yt <= n_yt; l_ady2 <= n_ady2; l_top <= n_top; l_hs <= n_hs;
            elsif g_avid = '1' then
                b_px  <= b_px + 1;
                b_x   <= b_x + s_xs;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel).  Key arithmetic in quarter px (Q2).
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a   : unsigned(11 downto 0);
        variable v_x   : unsigned(10 downto 0);
        variable v_sa  : signed(12 downto 0);
        variable v_st  : signed(11 downto 0);
        variable v_e   : unsigned(10 downto 0);
        variable v_r3  : unsigned(12 downto 0);
        variable v_vb  : signed(19 downto 0);
        variable v_h   : signed(13 downto 0);
        variable v_f   : unsigned(9 downto 0);
        variable v_m   : unsigned(4 downto 0);
        variable v_w   : unsigned(7 downto 0);
        variable v_lm  : unsigned(9 downto 0);
        variable v_rg  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: distance from the eye centre (h)
            q2_dx <= signed(resize(b_x, 13)) - C_CXH;
            -- 3: |dx| in px
            v_sa := abs(q2_dx); v_a := unsigned(v_sa(11 downto 0));
            q3_px <= v_a(11 downto 1);
            -- 4: twin fold operand
            q4_t  <= signed(resize(q3_px, 12)) - 480;
            q4_px <= q3_px;
            -- 5: r = |dx| (or the distance to the nearer twin centre), saturated; table address
            v_st := abs(q4_t);
            if s_twin = '1' then v_x := unsigned(v_st(10 downto 0)); else v_x := q4_px; end if;
            if v_x(10) = '1' then v_x := to_unsigned(1023, 11); end if;
            q5_adx <= v_x(9 downto 0);
            t_ra   <= v_x(9 downto 0);
            -- 6: table data (EBR)
            q6_adx <= q5_adx;
            -- 7: fabric copy; e = r - Xe; the edge-column key term
            q7_tx <= t_rq;
            q7_e  <= signed(resize(q6_adx, 11)) - signed(resize(c_xe, 11));
            if q6_adx > 860 then q7_far <= resize(q6_adx - 860, 8); else q7_far <= (others => '0'); end if;
            -- 8: r1 = rs e (Q8 px); the eye's stripe key (-e) 9/16 (Q2); ring phase = x term + line term + etch
            if q7_e > 0 then q8_r1 <= c_rs * unsigned(q7_e(9 downto 0)); else q8_r1 <= (others => '0'); end if;
            if q7_e < 0 then
                v_e := unsigned(-q7_e);
                q8_hn <= resize(shift_right(shift_left(resize(v_e, 14), 3) + resize(v_e, 14), 2), 12);
            else
                q8_hn <= (others => '0');
            end if;
            if q7_e > 0 then q8_epos <= '1'; else q8_epos <= '0'; end if;
            if q7_e < 0 then q8_eneg <= '1'; else q8_eneg <= '0'; end if;
            q8_ph  <= q7_tx + l_yt + k_et;
            q8_far <= q7_far;
            -- 9: r1 (Q2), the key operand (3 r1 outside the eye, the eye key inside)
            q9_r1 <= resize(q8_r1(16 downto 6), 12);
            v_r3 := resize(q8_r1(16 downto 6), 13) + shift_left(resize(q8_r1(16 downto 6), 13), 1);
            if q8_epos = '1' then q9_kop <= v_r3(11 downto 0); else q9_kop <= q8_hn; end if;
            q9_ph <= q8_ph; q9_far <= q8_far; q9_epos <= q8_epos; q9_eneg <= q8_eneg;
            -- 10: ray distances, the shared key difference; ring profile address (or the line's H stripe read)
            q10_d1 <= signed(resize(l_ady2, 13)) - signed(resize(q9_r1, 13));
            q10_d2 <= signed(resize(l_ady2, 13)) - signed(resize(q9_r1 & '0', 13));
            q10_d3 <= signed(resize(l_ady2, 13)) - signed(resize(q9_kop, 13));
            q10_u  <= signed(resize(q9_kop, 13)) - signed(resize(l_ady2, 13));
            q10_far <= q9_far; q10_epos <= q9_epos; q10_eneg <= q9_eneg;
            if t_hsel = '1' then r_pf_a <= "11" & e_hph(23 downto 16);
            else r_pf_a <= c_curve & q9_ph(15 downto 8); end if;
            -- 11: ray profiles (24 - |d|), fan and stripe keys clamped; profile ROM data (EBR)
            q11_v1 <= f_ray(q10_d1); q11_v2 <= f_ray(q10_d2); q11_v3 <= f_ray(q10_d3);
            v_h := resize(q10_u, 14) + 80;
            if v_h < 0 then q11_fr <= (others => '0');
            elsif v_h > 160 then q11_fr <= to_unsigned(160, 8);
            else q11_fr <= unsigned(v_h(7 downto 0)); end if;
            if l_top = '0' then
                q11_hv <= to_unsigned(200, 8);
            else
                v_h := resize(q10_u, 14) + signed(resize(shift_left(resize(q10_far, 12), 4), 14)) + 100;
                if v_h < 0 then q11_hv <= (others => '0');
                elsif v_h > 200 then q11_hv <= to_unsigned(200, 8);
                else q11_hv <= unsigned(v_h(7 downto 0)); end if;
            end if;
            q11_epos <= q10_epos; q11_eneg <= q10_eneg;
            q11_vd <= signed(resize(d_va, 17)) - 7209;
            -- 12: max ray, fan and stripe key levels, V-bar key; ring level (fabric copy)
            v_m := q11_v1;
            if q11_v2 > v_m then v_m := q11_v2; end if;
            if q11_v3 > v_m then v_m := q11_v3; end if;
            if q11_epos = '1' then q12_vm <= v_m; else q12_vm <= (others => '0'); end if;
            v_f := resize(q11_fr, 10) + resize(q11_fr(7 downto 1), 10) + resize(q11_fr(7 downto 4), 10);
            if q11_epos = '0' then q12_fan <= (others => '0');
            elsif v_f > 255 then q12_fan <= (others => '1');
            else q12_fan <= v_f(7 downto 0); end if;
            v_f := resize(q11_hv, 10) + resize(q11_hv(7 downto 2), 10) + resize(q11_hv(7 downto 5), 10);
            if v_f > 255 then q12_ah <= (others => '1'); else q12_ah <= v_f(7 downto 0); end if;
            q12_eneg <= q11_eneg;
            q12_ring <= r_pf_q;
            v_vb := shift_left(resize(q11_vd, 20), 2) + resize(q11_vd, 20);
            if v_vb < 0 then q12_vb <= (others => '0');
            elsif v_vb(19 downto 8) > 255 then q12_vb <= (others => '1');
            else q12_vb <= unsigned(v_vb(15 downto 8)); end if;
            -- 13: ray level, eye-dimmed stripe key, fan weights
            q13_ray <= shift_left(resize(q12_vm, 8), 2) + resize(q12_vm, 8) + resize(q12_vm(4 downto 2), 8);
            if q12_eneg = '1' then v_w := ('0' & q12_ah(7 downto 1)) - ("000" & q12_ah(7 downto 3)); else v_w := q12_ah; end if;
            q13_ah4 <= v_w(7 downto 4);
            q13_fq  <= q12_fan(7 downto 6);
            v_f := to_unsigned(256, 10) - resize(q12_fan, 10);
            q13_f4  <= v_f(8 downto 5);
            q13_ring <= q12_ring; q13_vb <= q12_vb; q13_hs <= l_hs;
            -- 14: V = vb + (255 - vb) q / 4; ring x (1 - fan); stripe-key mix products
            v_w := 255 - q13_vb;
            case to_integer(q13_fq) is
                when 0      => q14_v <= q13_vb;
                when 1      => q14_v <= q13_vb + resize(v_w(7 downto 2), 8);
                when 2      => q14_v <= q13_vb + resize(v_w(7 downto 1), 8);
                when others => q14_v <= q13_vb + resize(v_w(7 downto 2), 8) + resize(v_w(7 downto 1), 8);
            end case;
            v_rg := (others => '0');
            if q13_f4(0) = '1' then v_rg := v_rg + resize(q13_ring, 12); end if;
            if q13_f4(1) = '1' then v_rg := v_rg + shift_left(resize(q13_ring, 12), 1); end if;
            if q13_f4(2) = '1' then v_rg := v_rg + shift_left(resize(q13_ring, 12), 2); end if;
            if q13_f4(3) = '1' then v_rg := shift_left(resize(q13_ring, 12), 3); end if;
            q14_rg <= v_rg(10 downto 3);
            q14_am <= q13_ah4 * q13_hs;
            q14_gm <= resize(shift_left(resize(16 - resize(q13_ah4, 5), 10), 6) - shift_left(resize(16 - resize(q13_ah4, 5), 10), 1)
                      - resize(16 - resize(q13_ah4, 5), 10), 10);
            q14_ray <= q13_ray;
            -- 15: B = (ah Hs + (16 - ah) 61) / 16
            v_lm := resize(shift_right(resize(q14_am, 13) + resize(q14_gm, 13), 4), 10);
            q15_b <= v_lm(7 downto 0);
            q15_v <= q14_v; q15_rg <= q14_rg; q15_ray <= q14_ray;
            -- 16: weave = V B; lit = ring + rays
            if s_woff = '1' then q16_bm <= (others => '0'); else q16_bm <= q15_v * q15_b; end if;
            v_lm := resize(q15_rg, 10) + resize(q15_ray, 10);
            if v_lm > 255 then q16_lit <= (others => '1'); else q16_lit <= v_lm(7 downto 0); end if;
            -- 17: screen product
            q17_bl <= q16_bm(15 downto 8) * q16_lit;
            q17_s  <= resize(q16_bm(15 downto 8), 9) + resize(q16_lit, 9);
            -- 18: lum = base + lit - base lit
            v_lm := resize(q17_s, 10) - resize(q17_bl(15 downto 8), 10);
            if v_lm > 255 then q18_lum <= (others => '1'); else q18_lum <= v_lm(7 downto 0); end if;
            -- 19: Ink (negative) and the colour ROM address
            if s_ink = '1' then
                r_cl_a <= to_unsigned(230, 8) - q18_lum + ("000" & q18_lum(7 downto 3)) + ("00000" & q18_lum(7 downto 5));
            else
                r_cl_a <= q18_lum;
            end if;
            -- 20: colour ROM (EBR); 21: compose, black outside the generated avid
            if s_gav_sr(18) = '0' then
                s_y21 <= c_ky; s_u21 <= c_ku; s_v21 <= c_kv;
            else
                s_y21 <= unsigned(r_cl_q(31 downto 22)); s_u21 <= unsigned(r_cl_q(21 downto 12)); s_v21 <= unsigned(r_cl_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(18);

            -- V bars: phase DDA holding the stage-9 pixel (starts when the line's first pixel leaves stage 8)
            if s_gav_sr(6) = '1' and s_gav_sr(7) = '0' then
                d_vph <= n_vph;
            elsif s_gav_sr(6) = '1' then
                d_vph <= d_vph + resize(c_kvx, 24);
            end if;
            -- 10 (into q10_va): distance from the bar centre
            if d_vph(23) = '1' then d_va <= resize(d_vph(22 downto 8), 16); else d_va <= to_unsigned(32768, 16) - d_vph(23 downto 8); end if;

            -- camera / Osc etch: line buffer luma (3), two 4-px sums (4), 8-px sum (5), x gain in halves (6), >> 2 (7)
            k_d1 <= unsigned(s_lb_q); k_d2 <= k_d1; k_d3 <= k_d2; k_d4 <= k_d3; k_d5 <= k_d4; k_d6 <= k_d5; k_d7 <= k_d6;
            k_s4a <= resize(unsigned(s_lb_q), 10) + resize(k_d1, 10) + resize(k_d2, 10) + resize(k_d3, 10);
            k_s4b <= resize(k_d4, 10) + resize(k_d5, 10) + resize(k_d6, 10) + resize(k_d7, 10);
            k_sum <= resize(k_s4a, 11) + resize(k_s4b, 11);
            k_pa <= k_sum(6 downto 3) * c_eg;
            k_pb <= k_sum(10 downto 7) * c_eg;
            k_et <= resize(shift_right(resize(k_pa, 24) + shift_left(resize(k_pb, 24), 4), 2), 16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box (two adjacent 8-sums; the
    -- still's horizontal softness, and the HD HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black (the off-avid input) once 8 blank samples are in, so a
            -- glitched sample can't offset the colours for good.  Stage 22 sums, stage 23 output.
            s_cay(0) <= s_y21; s_cau(0) <= s_u21; s_cav(0) <= s_v21;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= c_ky & "000"; s_bau <= c_ku & "000"; s_bav <= c_kv & "000";
            else
                s_bay <= s_bay + s_y21 - s_cay(7);
                s_bau <= s_bau + s_u21 - s_cau(7);
                s_bav <= s_bav + s_v21 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bay, 14) + s_bayd; s_y23 <= v_sc(13 downto 4);
            v_sc := resize(s_bau, 14) + s_baud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v23 <= v_sc(13 downto 4);
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

end architecture tunnel;
