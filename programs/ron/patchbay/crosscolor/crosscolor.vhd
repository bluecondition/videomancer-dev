-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Crosscolor v0.1.1 -- Patchbay: composite artifact colour.
--
-- The patch:
--   pattern VCO (zone plate, or a sheared chirped grating)  -> soft clip -> luma
--   camera (or the Osc) luma -> non-additive mixer (max) with the pattern luma
--   mixed luma -> "composite" -> decoder:
--       half-subcarrier-period comb (the chroma band-pass)
--       x square subcarrier (0 deg -> U, 90 deg -> V; the subcarrier's phase steps per "line": 180 deg
--         flips or 90 deg steps, and advances per field)
--       running box (the chroma low-pass), a recursive line comb, a limiter
--   decoder chroma -> second encoder (its own crawling line phase) -> added to luma (dot crawl)
-- Luma detail near the subcarrier is mistaken for chroma: rainbow ghost rings where the plate's pitch
-- crosses the subcarrier.
--
--   K1  Pattern     the zone plate's chirp / the gratings' pitch (x0.35 .. x2)
--   K2  Subcarrier  its period, 48 .. 14 virtual px (where on the pattern the rainbows land)
--   K3  Saturation  the limiter's level (the chroma's peak)
--   K4  Speed       bipolar: the pattern's phase drifts (rings stream out / in)
--   K5  Bandwidth   the box length, 0.5 .. 8 subcarrier cycles (fine fringes .. broad pastel smears)
--   K6  Camera      how much of the camera (or the Osc) goes into the mixer and the decoder: 0 = none
--                   (the patch alone), 25 % = v0.1.0's level, 50 % (default) = 1.5x, 100 % = 2x (compressive:
--                   a linear 3x already washes the plate out under a bright picture)
--   S7  Pattern     Zone Plate / Gratings
--   S8  Comb        On (compact ghosts) / Off (the raw line-striped decoder)
--   S9  Dot Crawl   Off / On
--   S10 Standard    180 deg line flip / 90 deg line steps
--   S11 Source      Video / Osc
--   P12 BLEED       how hard the luma is driven into the chroma demodulator: clean mono .. rainbows everywhere
--
-- Engine: vblank-zone sequencer (one serial multiplier: the quadratic phase's start values and second
-- differences mod one cycle, the subcarrier, the box length and gain), a per-line stepper (five adds), the
-- pixel DDAs (phase, first difference, subcarrier), a 24-stage pixel path.  Virtual
-- 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (chroma
-- [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1] and 13/16) before the blanking gate; one latency (48) in every mode.
-- Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K6 Hue (burst phase) dropped, fixed at 512
-- (C_KFIX_6); nothing moved.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.crosscolor_pkg.all;
use work.all;

architecture crosscolor of program_top is
    constant C_KFIX_6 : unsigned(9 downto 0) := to_unsigned(512, 10);   -- K6 camera pass: Hue fixed at its default
    constant C_LAT : integer := 48;   -- = processing_delay_clks
    constant C_LD  : integer := 25;   -- the chroma stream runs 25 pixels ahead of the luma it is laid over

    -- y step in 1/16 virtual lines per field line
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(71, 7);
            when 4          => return to_unsigned(36, 7);
            when 8          => return to_unsigned(60, 7);
            when 12         => return to_unsigned(30, 7);
            when 5 | 6 | 14 => return to_unsigned(24, 7);
            when 1 | 2 | 10 => return to_unsigned(32, 7);
            when others     => return to_unsigned(16, 7);
        end case;
    end function;
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    function f_abs12(v : unsigned(11 downto 0)) return unsigned is
    begin
        if v(11) = '1' then return (not v) + 1; else return v; end if;
    end function;

    attribute no_rw_check : boolean;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc); the read address trails the write address by 3 or 4
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, z_mq : unsigned(15 downto 0) := (others => '0');
    signal r_sin_a : unsigned(8 downto 0) := (others => '0');
    signal r_sin_q : unsigned(7 downto 0) := (others => '0');

    -- rings (256 deep, written every clock): luma + quadrant + valid (two taps), demodulated products,
    -- the output chroma (8-sum delay), syncs
    type t_ring16 is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_ring32 is array (0 to 255) of std_logic_vector(31 downto 0);
    type t_ring4  is array (0 to 255) of std_logic_vector(3 downto 0);
    signal ry, rl : t_ring16 := (others => (others => '0'));
    signal rp, rc : t_ring32 := (others => (others => '0'));
    signal rs : t_ring4 := (others => (others => '1'));
    attribute no_rw_check of ry : signal is true;
    attribute no_rw_check of rl : signal is true;
    attribute no_rw_check of rp : signal is true;
    attribute no_rw_check of rc : signal is true;
    attribute no_rw_check of rs : signal is true;
    signal r_wp, r_ah, r_al, r_an, r_as, r_ac : unsigned(7 downto 0) := (others => '0');
    signal ry_q, ry_q2, rl_q, rl_q2 : std_logic_vector(15 downto 0) := (others => '0');
    signal rp_q, rp_q2, rc_q, rc_q2 : std_logic_vector(31 downto 0) := (others => '0');
    signal rs_q, rs_q2 : std_logic_vector(3 downto 0) := (others => '1');
    signal r_yw : std_logic_vector(15 downto 0) := (others => '0');

    -- comb store: one {U, V} state (Q4) per two pixels
    type t_cram is array (0 to 1023) of std_logic_vector(31 downto 0);
    signal cram : t_cram := (others => (others => '0'));
    attribute no_rw_check of cram : signal is true;
    signal c_we : std_logic := '0';
    signal c_wa : unsigned(9 downto 0) := (others => '0');
    signal c_q, c_q2 : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc)
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_fo, s_fl, s_ff : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_olat : unsigned(7 downto 0) := (others => '0');
    signal s_ofin : unsigned(6 downto 0) := (others => '0');
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ysr, s_ys : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal s_xsr, s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(585 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(440 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(500 * 8, 13);
    signal s_gk : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);   -- K6 camera (glided x8)
    signal c_cg : unsigned(9 downto 0) := to_unsigned(384, 10);       -- camera gain, Q8 of 2 l (256 = v0.1.0)
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_grat, s_ncomb, s_dots, s_stdb, s_src, s_sd : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- modulators: free-running phase counters, folded to triangles
    signal m_cx : unsigned(15 downto 0) := x"4000";
    signal m_cy : unsigned(15 downto 0) := x"D000";
    signal m_t  : unsigned(15 downto 0) := x"3000";
    signal c_psi, c_dps : unsigned(19 downto 0) := (others => '0');
    signal c_f   : unsigned(31 downto 0) := (others => '0');

    -- sequencer
    signal z_e1  : signed(9 downto 0) := (others => '0');
    signal z_e2  : unsigned(8 downto 0) := to_unsigned(255, 9);
    signal z_en  : signed(11 downto 0) := (others => '0');
    signal z_l8  : signed(12 downto 0) := (others => '0');
    signal z_a   : unsigned(21 downto 0) := (others => '0');
    signal z_kt, z_ktt : unsigned(16 downto 0) := (others => '0');
    signal z_x0m : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal z_y0m : unsigned(13 downto 0) := to_unsigned(8640, 14);
    signal z_t14 : unsigned(13 downto 0) := to_unsigned(9000, 14);
    signal z_u   : unsigned(12 downto 0) := (others => '0');
    signal z_w   : unsigned(14 downto 0) := (others => '0');
    signal z_s   : unsigned(37 downto 0) := (others => '0');
    signal z_run, z_ldp, z_st, z_st1, z_st2 : std_logic := '0';
    signal z_ph  : unsigned(4 downto 0) := (others => '0');
    signal z_sel : unsigned(5 downto 0) := (others => '0');
    signal c_kg  : unsigned(11 downto 0) := to_unsigned(1615, 12);
    signal c_lgy : unsigned(10 downto 0) := to_unsigned(1024, 11);

    -- per-field constants
    signal c_kap : unsigned(16 downto 0) := to_unsigned(46603, 17);
    signal c_ee, c_dl, c_dd : unsigned(27 downto 0) := (others => '0');
    signal c_w   : unsigned(19 downto 0) := to_unsigned(348000, 20);
    signal c_dth : unsigned(19 downto 0) := (others => '0');
    signal c_wx  : unsigned(11 downto 0) := (others => '0');
    signal c_vst : unsigned(17 downto 0) := (others => '0');
    signal c_n   : unsigned(7 downto 0) := to_unsigned(26, 8);
    signal c_h   : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal c_nc  : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal c_rnd : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal c_l   : unsigned(8 downto 0) := to_unsigned(253, 9);
    signal c_g   : unsigned(15 downto 0) := (others => '0');

    -- field serial multiplier (mod 2^38), with a pre-shift of the multiplicand
    signal f_a, f_acc : unsigned(37 downto 0) := (others => '0');
    signal f_b : unsigned(16 downto 0) := (others => '0');
    signal f_k : unsigned(2 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line stepper
    signal e_zk  : std_logic := '0';
    signal e_lc  : unsigned(2 downto 0) := (others => '0');
    signal v_q0, v_d0, v_e : unsigned(27 downto 0) := (others => '0');
    signal v_ph  : unsigned(19 downto 0) := (others => '0');
    signal n_th  : unsigned(11 downto 0) := (others => '0');
    signal n_doff, l_doff : unsigned(1 downto 0) := (others => '0');
    signal l_fy  : unsigned(11 downto 0) := (others => '0');

    -- beam
    signal b_q, b_d : unsigned(27 downto 0) := (others => '0');
    signal b_th  : unsigned(19 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');

    -- valid chain: s_v(k) = generated avid of the stage-k pixel; s_gx(k) = the chroma stream's (C_LD longer)
    signal s_v  : std_logic_vector(2 to 6) := (others => '0');
    signal s_gx : std_logic_vector(7 to 14) := (others => '0');
    signal s_vl : std_logic_vector(18 to 24) := (others => '0');   -- the luma stream's, stages 18 .. 24
    signal s_ext : unsigned(4 downto 0) := (others => '0');
    signal s_gxq : std_logic := '0';

    -- pixel path
    type t_q2 is array (natural range <>) of unsigned(1 downto 0);
    signal p_qd  : t_q2(2 to 9) := (others => "00");             -- subcarrier quadrant of the stage-k pixel
    signal p2_sg, p3_sg, p4_sg, p5_sg : std_logic := '0';
    signal p2_fx, p3_fm : unsigned(11 downto 0) := (others => '0');
    signal p4_sin : unsigned(7 downto 0) := (others => '0');
    signal p4_amp : unsigned(6 downto 0) := (others => '0');
    signal p5_m   : unsigned(8 downto 0) := (others => '0');
    signal p3_cam : unsigned(7 downto 0) := (others => '0');
    signal p4_cg : unsigned(17 downto 0) := (others => '0');          -- camera x gain
    signal p5_cam : signed(8 downto 0) := to_signed(-255, 9);
    signal p6_yc  : signed(8 downto 0) := (others => '0');
    signal p6_cnt : unsigned(4 downto 0) := (others => '0');
    signal p7_ybp : signed(10 downto 0) := (others => '0');
    signal p8_a, p8_b : signed(19 downto 0) := (others => '0');
    signal p9_s   : signed(15 downto 0) := (others => '0');
    signal p10_u, p10_v : signed(15 downto 0) := (others => '0');
    signal p11_du, p11_dv : signed(16 downto 0) := (others => '0');
    signal p12_au, p12_av : signed(23 downto 0) := (others => '0');
    signal p_cb   : unsigned(8 downto 0) := (others => '0');
    signal p_rsy   : std_logic := '0';
    signal p13_u, p13_v : signed(15 downto 0) := (others => '0');
    signal p11_cc, p14_cw : unsigned(10 downto 0) := (others => '0');
    signal p14_du, p14_dv : signed(16 downto 0) := (others => '0');
    signal p14_su, p14_sv, p15_u, p15_v : signed(15 downto 0) := (others => '0');
    signal p16_u, p16_v : signed(12 downto 0) := (others => '0');
    signal p17_u, p17_v, p18_u, p18_v : signed(9 downto 0) := (others => '0');
    signal s_first : std_logic := '0';
    signal p18_ds : signed(10 downto 0) := (others => '0');
    signal p18_yp : unsigned(10 downto 0) := (others => '0');
    signal p19_y  : signed(12 downto 0) := (others => '0');
    signal p19_u, p19_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_yd1, s_yd2 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22, s_y23 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(b_px));
            if s_lb_we = '1' then lb(to_integer(s_wcol)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q  <= C_CC_EXP2(to_integer(r_ex_a));
            z_mq    <= r_ex_q;
            r_sin_q <= C_CC_SIN(to_integer(r_sin_a));
        end if;
    end process p_rom;

    -- rings: a word written at clock t is in the fabric copy at t + delay + 2 (never the write address)
    p_ptr : process(clk)
    begin
        if rising_edge(clk) then
            r_wp <= r_wp + 1;
            r_ah <= r_wp + 3 - resize(c_h, 8);                 -- the half-period tap (h back)
            r_al <= r_wp + 1 - to_unsigned(C_LD + 9, 8);       -- luma: stage 6 -> stage 17, C_LD pixels back
            r_an <= r_wp + 3 - c_n;                            -- the box's far end (N back)
            r_as <= r_wp + 1 - to_unsigned(C_LAT - 2, 8);      -- syncs
            r_ac <= r_wp + 1 - to_unsigned(6, 8);              -- the chroma 8-sums' far end
        end if;
    end process p_ptr;
    p_ry : process(clk)
    begin
        if rising_edge(clk) then
            ry_q <= ry(to_integer(r_ah));
            ry(to_integer(r_wp)) <= r_yw;
            ry_q2 <= ry_q;
        end if;
    end process p_ry;
    p_rl : process(clk)
    begin
        if rising_edge(clk) then
            rl_q <= rl(to_integer(r_al));
            rl(to_integer(r_wp)) <= r_yw;
            rl_q2 <= rl_q;
        end if;
    end process p_rl;
    p_rp : process(clk)
    begin
        if rising_edge(clk) then
            rp_q <= rp(to_integer(r_an));
            rp(to_integer(r_wp)) <= std_logic_vector(p10_u) & std_logic_vector(p10_v);
            rp_q2 <= rp_q;
        end if;
    end process p_rp;
    p_rc : process(clk)
    begin
        if rising_edge(clk) then
            rc_q <= rc(to_integer(r_ac));
            rc(to_integer(r_wp)) <= x"000" & std_logic_vector(s_u20) & std_logic_vector(s_v20);
            rc_q2 <= rc_q;
        end if;
    end process p_rc;
    p_rs : process(clk)
    begin
        if rising_edge(clk) then
            rs_q <= rs(to_integer(r_as));
            rs(to_integer(r_wp)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            rs_q2 <= rs_q;
        end if;
    end process p_rs;
    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            c_q <= cram(to_integer(p11_cc(10 downto 1)));
            if c_we = '1' then cram(to_integer(c_wa)) <= std_logic_vector(p15_u) & std_logic_vector(p15_v); end if;
            c_q2 <= c_q;
        end if;
    end process p_cram;
    r_yw <= "0000" & s_v(6) & std_logic_vector(p_qd(6)) & std_logic_vector(p6_yc);

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_d3 : signed(12 downto 0);
        variable v_k7 : unsigned(12 downto 0);
        variable v_n  : unsigned(8 downto 0);
        variable v_l  : signed(12 downto 0);
        variable v_k5 : unsigned(9 downto 0);
        variable v_kc : unsigned(9 downto 0);
        variable v_fa : unsigned(37 downto 0);
        variable v_fb : unsigned(16 downto 0);
        variable v_fk : unsigned(2 downto 0);
        variable v_neh : signed(9 downto 0);
        variable v_nen : signed(11 downto 0);
        variable v_tr : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ysr <= f_vstep_y(s_tid);
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
                s_grat  <= registers_in(6)(0);
                s_ncomb <= registers_in(6)(1);
                s_dots  <= registers_in(6)(2);
                s_stdb  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_ys    <= s_ysr;
                case to_integer(s_xsr) is
                    when 4      => s_sd <= '1'; c_kg <= to_unsigned(3230, 12);
                    when 3      => s_sd <= '0'; c_kg <= to_unsigned(2422, 12);
                    when others => s_sd <= '0'; c_kg <= to_unsigned(1615, 12);
                end case;
                case to_integer(s_ysr) is
                    when 71     => c_lgy <= to_unsigned(1574, 11);
                    when 36     => c_lgy <= to_unsigned(1323, 11);
                    when 60     => c_lgy <= to_unsigned(1512, 11);
                    when 30     => c_lgy <= to_unsigned(1256, 11);
                    when 24     => c_lgy <= to_unsigned(1174, 11);
                    when 32     => c_lgy <= to_unsigned(1280, 11);
                    when others => c_lgy <= to_unsigned(1024, 11);
                end case;
                -- modulators: the plate's centre (a slow Lissajous), the gratings' shear, the subcarrier's
                -- field advance (crawl) and the second encoder's extra crawl
                m_cx <= m_cx + 47;
                m_cy <= m_cy + 35;
                m_t  <= m_t + 64;
                c_psi <= c_psi + to_unsigned(2266, 20);
                c_dps <= c_dps - to_unsigned(39062, 20);
            end if;
            g_wm1 <= s_w - 1;

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
                            when 2      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 3      => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                            when others => s_gm <= s_gk; s_gr <= unsigned(registers_in(5));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        -- deadband +-2 knob codes (+-4 on the camera channel, 4)
                        if s_gd > 32 or s_gd < -32 or (z_gc /= 4 and (s_gd > 16 or s_gd < -16)) then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gk <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- fixed states: exponents (Q8 octaves) of the glided knobs, geometry, the comb's length
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 28 =>
                        v_d := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        if v_d(10) = '1' then
                            v_d3 := resize(v_d, 13) + shift_left(resize(v_d, 13), 1);
                            z_e1 <= resize(shift_right(v_d3, 2), 10);                -- -1.5 octaves at 0
                        else
                            z_e1 <= resize(shift_right(v_d, 1), 10);                 -- +1 octave at 1023
                        end if;
                        v_k7 := (s_g2(12 downto 3) & "000") - resize(s_g2(12 downto 3), 13);
                        z_e2 <= v_k7(12 downto 4);                                   -- 0 .. 1.75 octaves
                        -- triangles of the modulator counters
                        if m_cx(15) = '1' then v_tr := resize(not m_cx(14 downto 6), 13); else v_tr := resize(m_cx(14 downto 6), 13); end if;
                        z_x0m <= to_unsigned(1664, 12) + v_tr(11 downto 0);          -- centre x, half px: 1920 +- 256
                        if m_cy(15) = '1' then v_tr := resize(not m_cy(14 downto 4), 13); else v_tr := resize(m_cy(14 downto 4), 13); end if;
                        z_y0m <= to_unsigned(7616, 14) + v_tr;                       -- centre y, 1/16 lines: 8640 +- 1024
                        if m_t(15) = '1' then v_tr := not m_t(14 downto 2); else v_tr := m_t(14 downto 2); end if;
                        z_t14 <= to_unsigned(5734, 14) + v_tr;                       -- shear 0.35 .. 0.85, Q14
                    when 29 =>
                        z_en <= signed(resize(s_g5(12 downto 3), 12)) - signed(resize(z_e2, 12));
                        if s_sd = '1' then z_x0m <= z_x0m - 480; end if;             -- SD: the centre 1440 px
                        z_l8 <= to_signed(2647, 13) - signed(resize(z_e2, 13)) - signed(resize(c_lgy, 13));
                    when 30 =>
                        z_u <= (z_x0m & '0') - resize(s_xs, 13);
                        z_w <= (z_y0m & '0') - resize(s_ys, 15);
                        -- comb length: log2(lines) = log2(strip height * 2.2) - log2(line step), + up to 1 with Bandwidth
                        v_k5 := s_g5(12 downto 3);
                        if v_k5 >= 768 then v_l := to_signed(256, 13);
                        elsif v_k5 > 512 then v_l := signed(resize(v_k5 - 512, 13));
                        else v_l := (others => '0'); end if;
                        if s_stdb = '1' then v_l := v_l - 150; end if;
                        z_l8 <= z_l8 + v_l + 128;
                    when 31 =>
                        -- camera gain (K6), compressive: k (0 .. 256: off .. v0.1.0's 2 l), 128 + k / 2 (.. 384 = 1.5x
                        -- at 512), 256 + k / 4 (.. 512 = 2x at 1023).  Linear 3x already washes the plate out.
                        v_kc := s_gk(12 downto 3);
                        if v_kc <= 256 then c_cg <= v_kc;
                        elsif v_kc <= 512 then c_cg <= to_unsigned(128, 10) + v_kc(9 downto 1);
                        else c_cg <= to_unsigned(256, 10) + v_kc(9 downto 2); end if;
                        if s_ncomb = '1' then c_nc <= "000"; c_rnd <= (others => '0');
                        elsif z_l8 < 512 then c_nc <= "001"; c_rnd <= to_unsigned(1, 6);
                        elsif z_l8 >= 1536 then c_nc <= "110"; c_rnd <= to_unsigned(32, 6);
                        else
                            c_nc <= unsigned(z_l8(10 downto 8));
                            case to_integer(unsigned(z_l8(10 downto 8))) is
                                when 2      => c_rnd <= to_unsigned(2, 6);
                                when 3      => c_rnd <= to_unsigned(4, 6);
                                when 4      => c_rnd <= to_unsigned(8, 6);
                                when others => c_rnd <= to_unsigned(16, 6);
                            end case;
                        end if;
                    when others => null;
                end case;
            end if;

            -- serial products (mod 2^38: the phase terms only matter mod one cycle): 37 slots of 32 clocks from
            -- zc = 40.  The exp2 ROM is addressed by the slot (its data is ready at phase 3); phase 4 loads the
            -- operands (the multiplicand may be the previous product), then an optional pre-shift (the
            -- exponent's integer part) and the iterations; phase 28 stores (29, 30: follow-ups).
            if s_zone = '1' and s_zc = 39 then
                z_run <= '1'; z_ph <= (others => '0'); z_sel <= (others => '0');
            elsif z_run = '1' then
                z_ph <= z_ph + 1;
                if z_ph = 31 then
                    z_sel <= z_sel + 1;
                    if z_sel = 36 then z_run <= '0'; end if;
                end if;
            end if;
            if z_run = '1' and z_ph = 3 then z_ldp <= '1'; else z_ldp <= '0'; end if;
            if z_run = '1' and z_ph = 27 then z_st <= '1'; else z_st <= '0'; end if;
            z_st1 <= z_st; z_st2 <= z_st1;

            v_neh := -signed(resize(z_e2, 10));
            v_nen := -z_en;
            case to_integer(z_sel) is
                when 0      => r_ex_a <= unsigned(z_e1(7 downto 0));
                when 24     => r_ex_a <= z_e2(7 downto 0);
                when 29     => r_ex_a <= unsigned(z_en(7 downto 0));
                when 30     => r_ex_a <= unsigned(v_neh(7 downto 0));
                when others => r_ex_a <= unsigned(v_nen(7 downto 0));
            end case;

            -- operands
            v_fk := "000";
            case to_integer(z_sel) is
                when 0  => v_fa := to_unsigned(46603, 38);              v_fb := resize(z_mq, 17);       -- chirp
                           v_fk := '0' & (unsigned(z_e1(9 downto 8)) + 2);
                when 1  => v_fa := resize(c_kap, 38);                   v_fb := resize(z_t14, 17);      -- chirp * shear
                when 2  => v_fa := resize(z_kt, 38);                    v_fb := resize(z_t14, 17);      -- chirp * shear^2
                when 3  => v_fa := resize(s_vm, 38);                    v_fb := resize(s_vm, 17);       -- speed^2
                when 4  => v_fa := resize(z_x0m, 38);                   v_fb := resize(z_x0m, 17);      -- X0^2
                when 5 | 12 => v_fa := f_acc(31 downto 0) & "000000";   v_fb := c_kap;                  -- x A
                when 6  => v_fa := resize(z_x0m, 38);                   v_fb := resize(z_y0m, 17);      -- X0 Y0
                when 7 | 14 | 16 | 22 =>                                                                -- x B
                    v_fa := f_acc(33 downto 0) & "0000";
                    if s_grat = '1' then v_fb := z_kt; else v_fb := (others => '0'); end if;
                when 8  => v_fa := resize(z_y0m, 38);                   v_fb := resize(z_y0m, 17);      -- Y0^2
                when 9 | 18 | 20 =>                                                                     -- x C
                    v_fa := f_acc;
                    if s_grat = '1' then v_fb := z_ktt; else v_fb := c_kap; end if;
                when 10 => v_fa := resize(c_f, 38);                     v_fb := to_unsigned(64, 17);    -- drift phase
                when 11 => v_fa := resize(z_u, 38);                     v_fb := resize(s_xs, 17);       -- (2 |X0| - xs) xs
                when 13 => v_fa := resize(z_y0m, 38);                   v_fb := resize(s_xs, 17);       -- |Y0| xs
                when 15 => v_fa := resize(z_x0m, 38);                   v_fb := resize(s_ys, 17);       -- |X0| ys
                when 17 => v_fa := resize(z_w, 38);                     v_fb := resize(s_ys, 17);       -- (2 |Y0| - ys) ys
                when 19 => v_fa := resize(s_ys, 38);                    v_fb := resize(s_ys & '0', 17); -- 2 ys^2
                when 21 => v_fa := resize(s_xs, 38);                    v_fb := resize(s_ys, 17);       -- xs ys
                when 23 => v_fa := resize(c_kap, 38);                                                   -- 2 A xs^2
                    case to_integer(s_xs) is
                        when 4      => v_fb := to_unsigned(32, 17);
                        when 3      => v_fb := to_unsigned(18, 17);
                        when others => v_fb := to_unsigned(8, 17);
                    end case;
                when 24 => v_fa := to_unsigned(174763, 38);             v_fb := resize(z_mq, 17);       -- subcarrier
                           v_fk := "00" & z_e2(8);
                when 25 => v_fa := resize(c_w, 38);                     v_fb := resize(s_xs, 17);       -- per pixel
                when 26 => v_fa := resize(c_w, 38);                     v_fb := to_unsigned(480, 17);   -- SD window offset
                when 27 => v_fa := resize(c_w, 38);                     v_fb := to_unsigned(43691, 17); -- / 6
                when 28 => v_fa := resize(z_a, 38);                     v_fb := resize(s_ys, 17);       -- line phase step
                when 29 | 30 =>                                                                         -- box length, half period
                    case to_integer(s_xs) is
                        when 4      => v_fa := to_unsigned(12, 38);
                        when 3      => v_fa := to_unsigned(16, 38);
                        when others => v_fa := to_unsigned(24, 38);
                    end case;
                    v_fb := resize(z_mq, 17);
                    if z_sel(0) = '1' then v_fk := unsigned(z_en(10 downto 8)) + 2;
                    else v_fk := '0' & (unsigned(v_neh(9 downto 8)) + 2); end if;
                when 31 => v_fa := resize(unsigned(registers_in(2)), 38); v_fb := to_unsigned(330, 17); -- level
                when 32 => v_fa := resize(s_gp(12 downto 3), 38);       v_fb := resize(s_gp(12 downto 3), 17);  -- bleed^2
                when 33 => v_fa := resize(z_a, 38);                     v_fb := resize(s_gp(12 downto 3), 17);  -- bleed^3
                when 34 => v_fa := resize(z_a, 38);                     v_fb := resize(c_l, 17);        -- x level
                when 35 => v_fa := resize(z_a, 38);                     v_fb := resize(z_mq, 17);       -- / box length
                when others => v_fa := resize(z_a, 38);                 v_fb := resize(c_kg, 17);
                           v_fk := unsigned(v_nen(10 downto 8)) + 4;
            end case;
            if z_ldp = '1' then
                f_acc <= (others => '0');
                f_a <= v_fa; f_b <= v_fb; f_k <= v_fk;
            elsif f_k /= 0 then
                f_a <= f_a(36 downto 0) & '0';
                f_k <= f_k - 1;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(36 downto 0) & '0';
                f_b <= '0' & f_b(16 downto 1);
            end if;

            -- slot stores
            if z_st = '1' then
                case to_integer(z_sel) is
                    when 0  => c_kap <= f_acc(33 downto 17);                -- chirp: 46603 * 2^e1
                    when 1  => z_kt  <= f_acc(30 downto 14);
                    when 2  => z_ktt <= f_acc(30 downto 14);
                    when 3  =>                        -- pattern drift (speed^2)
                        if s_vneg = '1' then c_f <= c_f - resize(f_acc(17 downto 0) & "0000000000", 32);
                        else c_f <= c_f + resize(f_acc(17 downto 0) & "0000000000", 32); end if;
                    when 20 => c_ee <= f_acc(37 downto 10);                 -- the line step's line step: 2 C ys^2
                    when 22 => c_dl <= f_acc(37 downto 10);                 -- the first difference's line step: xs B ys
                    when 23 => c_dd <= f_acc(31 downto 4);                  -- second difference along the line: 2 A xs^2
                    when 24 => c_w  <= f_acc(34 downto 15);                 -- subcarrier: cycles per half px, Q24
                    when 25 => c_dth <= f_acc(23 downto 4);                 -- per pixel, Q20
                    when 26 => if s_sd = '1' then c_wx <= f_acc(23 downto 12); else c_wx <= (others => '0'); end if;
                    when 27 =>                        -- strips per virtual line: 0.75 (flip) or 0.5 (steps) periods tall
                        if s_stdb = '1' then z_a <= c_w & "00"; else z_a <= f_acc(35 downto 14); end if;
                    when 28 => c_vst <= f_acc(27 downto 10);                -- strips per field line, Q18
                    when 29 =>                        -- box length (real pixels)
                        v_n := f_acc(25 downto 17);
                        if v_n > 250 then c_n <= to_unsigned(250, 8);
                        elsif v_n < 4 then c_n <= to_unsigned(4, 8);
                        else c_n <= v_n(7 downto 0); end if;
                    when 30 =>                        -- half a subcarrier period (real pixels)
                        v_n := f_acc(25 downto 17);
                        if v_n > 30 then c_h <= to_unsigned(30, 5);
                        elsif v_n < 3 then c_h <= to_unsigned(3, 5);
                        else c_h <= v_n(4 downto 0); end if;
                    when 31 => c_l <= to_unsigned(60, 9) + f_acc(18 downto 10);     -- limiter level 60 .. 389
                    when 32 => z_a <= resize(f_acc(19 downto 0), 22);
                    when 33 => z_a <= resize(f_acc(29 downto 10), 22);
                    when 34 => z_a <= resize(f_acc(28 downto 9), 22);
                    when 35 => z_a <= resize(f_acc(35 downto 16), 22);
                    when 36 =>                        -- gain per luma step, Q12: bleed^3 * level / box length
                        if f_acc(37 downto 36) /= "00" then c_g <= (others => '1'); else c_g <= f_acc(35 downto 20); end if;
                    when others => null;
                end case;
            end if;
            -- sums of products: line-0 phase (slots 5, 7, 9, 10), first difference (12, 14), line step (16, 18)
            if z_st = '1' then
                case to_integer(z_sel) is
                    when 4                 => z_s <= (others => '0');
                    when 5 | 7 | 9         => z_s <= z_s + f_acc;
                    when 10 | 12 | 14 | 16 | 18 => z_s <= z_s - f_acc;
                    when others => null;
                end case;
            end if;
            if z_st2 = '1' and (z_sel = 10 or z_sel = 14) then z_s <= (others => '0'); end if;

            s_v(2) <= g_avid;
            for i in 3 to 6 loop s_v(i) <= s_v(i - 1); end loop;
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
    -- Source side: the Osc (the slow 640 x 540 triangle lattice plus a fine
    -- tilted triangle grating, 26 virtual px, drifting) and the line-buffer
    -- write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_o : signed(10 downto 0);
        variable v_f : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                case to_integer(s_xs) is            -- per pixel: 51 / half px (lattice), 1260 / half px (grating)
                    when 4      => s_ox <= s_ox + 204; s_fo <= s_fo + 5040;
                    when 3      => s_ox <= s_ox + 153; s_fo <= s_fo + 3780;
                    when others => s_ox <= s_ox + 102; s_fo <= s_fo + 2520;
                end case;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + resize(shift_right(resize(s_ys, 15) * to_unsigned(121, 7), 4), 16);
                s_fl <= s_fl + resize(s_ys * to_unsigned(102, 7), 16);
            else
                s_fo <= s_fl;
            end if;
            if s_zone = '1' then
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);    -- ot * 96
                s_fl <= s_ff;
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; s_ff <= s_ff + 983; end if;
            s_olat <= resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_ofin <= f_tri7(s_fo(15 downto 8));
            v_o := signed(resize(s_olat, 11)) - 127;
            v_o := shift_right(v_o + shift_left(v_o, 1), 2);                                  -- x 3/4
            v_f := signed(resize(s_ofin, 11)) - 64;
            v_o := v_o + v_f + shift_right(v_f, 2) + 127;
            if v_o < 0 then s_osum <= (others => '0');
            elsif v_o > 255 then s_osum <= (others => '1');
            else s_osum <= unsigned(v_o(7 downto 0)); end if;
            s_lb_we <= data_in.avid;
            if s_src = '1' then s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if s_lb_we = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line stepper: the vertical second-order DDA of the line-start phase, the
    -- first difference's line step, and the subcarrier's line phase (strips);
    -- stepped after each line start for the NEXT line.  The sequencer loads the
    -- line-0 values in the zone (no line starts there).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_lo, v_lo2 : unsigned(1 downto 0);
        variable v_p2 : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1240 then e_zk <= '1'; end if;

            if z_st1 = '1' and z_sel = 10 then v_q0 <= z_s(37 downto 10); end if;
            if z_st1 = '1' and z_sel = 14 then v_d0 <= z_s(37 downto 10); end if;
            if z_st1 = '1' and z_sel = 18 then v_e  <= z_s(37 downto 10); end if;
            if e_zk = '1' then
                v_ph <= c_psi;
                e_lc <= to_unsigned(5, 3);
            elsif g_start = '1' then
                e_lc <= to_unsigned(1, 3);
            else
                case to_integer(e_lc) is
                    when 1 => v_q0 <= v_q0 + v_e;   e_lc <= to_unsigned(2, 3);
                    when 2 => v_e  <= v_e + c_ee;   e_lc <= to_unsigned(3, 3);
                    when 3 => v_d0 <= v_d0 + c_dl;  e_lc <= to_unsigned(4, 3);
                    when 4 => v_ph <= v_ph + resize(c_vst, 20); e_lc <= to_unsigned(5, 3);
                    when 5 =>
                        v_p2 := v_ph + c_dps;                               -- the second encoder's line phase
                        if s_stdb = '1' then v_lo := v_ph(19 downto 18); v_lo2 := v_p2(19 downto 18);
                        else v_lo := v_ph(18) & '0'; v_lo2 := v_p2(18) & '0'; end if;
                        -- line-start phase (top 12 bits): burst phase (Hue) + the SD window offset + the line's quadrant
                        n_th <= (C_KFIX_6 & "00") + c_wx + (v_lo & "0000000000");
                        n_doff <= v_lo2 - v_lo;
                        e_lc <= to_unsigned(0, 3);
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the pattern phase by second differences (Q += D; D += DD), the
    -- subcarrier
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_q  <= v_q0;
                b_d  <= v_d0;
                b_th <= n_th & x"00";
                b_px <= (others => '0');
                l_doff <= n_doff;
                l_fy <= f_abs12(v_e(27 downto 16));        -- |vertical frequency| of this line (cycles / 4096 per line)
            elsif g_avid = '1' then
                b_q  <= b_q + b_d;
                b_d  <= b_d + c_dd;
                b_th <= b_th + c_dth;
                b_px <= b_px + 1;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p  : unsigned(14 downto 0);
        variable v_c  : signed(9 downto 0);
        variable v_m  : signed(8 downto 0);
        variable v_s28 : signed(27 downto 0);
        variable v_s22 : signed(21 downto 0);
        variable v_u22 : signed(21 downto 0);
        variable v_su, v_sv : signed(15 downto 0);
        variable v_d17 : signed(16 downto 0);
        variable v_gx : std_logic;
        variable v_o  : signed(16 downto 0);
        variable v_ln : signed(12 downto 0);
        variable v_q  : unsigned(1 downto 0);
        variable v_yl : signed(8 downto 0);
        variable v_ds : signed(10 downto 0);
        variable v_y  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: sine address, |horizontal frequency| (cycles / 4096 per pixel), subcarrier quadrant
            r_sin_a <= b_q(26 downto 18);
            p2_sg <= b_q(27);
            p2_fx <= f_abs12(b_d(27 downto 16));
            p_qd(2) <= b_th(19 downto 18);
            for i in 3 to 9 loop p_qd(i) <= p_qd(i - 1); end loop;
            -- 3: the larger frequency; camera / Osc sample
            if p2_fx > l_fy then p3_fm <= p2_fx; else p3_fm <= l_fy; end if;
            p3_sg <= p2_sg;
            p3_cam <= unsigned(s_lb_q);
            -- 4: sine (fabric copy); amplitude: the oscillator's bandwidth roll-off (1 -> 1/2 by a quarter cycle
            -- per pixel or line), then the fade to nothing before Nyquist; camera x gain
            p4_sin <= r_sin_q;
            p4_sg <= p3_sg;
            if p3_fm(11 downto 10) = "00" then p4_amp <= to_unsigned(64, 7) - resize(p3_fm(9 downto 5), 7);
            elsif p3_fm < 1408 then p4_amp <= to_unsigned(32, 7);
            elsif p3_fm < 1664 then p4_amp <= resize(shift_right(to_unsigned(1664, 12) - p3_fm, 3), 7);
            else p4_amp <= (others => '0'); end if;
            p4_cg <= p3_cam * c_cg;                     -- camera x gain (K6), its own stage
            -- 5: |pattern luma|
            v_p := p4_sin * p4_amp;
            p5_m <= v_p(14 downto 6);
            p5_sg <= p4_sg;
            -- camera as g l - 255, clipped at 255 (v0.1.0: 2 l - 255, black = -255)
            if p4_cg(17 downto 16) /= "00" then p5_cam <= to_signed(255, 9);
            else
                v_c := signed(resize(p4_cg(15 downto 7), 10)) - 255;
                if v_c > 255 then p5_cam <= to_signed(255, 9); else p5_cam <= v_c(8 downto 0); end if;
            end if;
            -- 6: signed; the non-additive mix with the camera
            if p5_sg = '1' then v_m := -signed(p5_m); else v_m := signed(p5_m); end if;
            if p5_cam > v_m then p6_yc <= p5_cam; else p6_yc <= v_m; end if;
            if s_v(6) = '1' then
                if p6_cnt /= 31 then p6_cnt <= p6_cnt + 1; end if;
            else
                p6_cnt <= (others => '0');
            end if;
            -- 7: the chroma band-pass: Y(x) - Y(x - half a subcarrier period)
            if s_v(6) = '1' and p6_cnt >= c_h then
                p7_ybp <= resize(p6_yc, 11) - resize(signed(ry_q2(8 downto 0)), 11);
            else
                p7_ybp <= (others => '0');
            end if;
            -- 8, 9: x gain (Q12) -> Q6, saturated
            p8_a <= p7_ybp * signed('0' & c_g(7 downto 0));
            p8_b <= p7_ybp * signed('0' & c_g(15 downto 8));
            v_s28 := resize(p8_a, 28) + shift_left(resize(p8_b, 28), 8) + 32;
            v_s22 := v_s28(27 downto 6);
            if v_s22 > 32767 then p9_s <= to_signed(32767, 16);
            elsif v_s22 < -32767 then p9_s <= to_signed(-32767, 16);
            else p9_s <= v_s22(15 downto 0); end if;
            -- 10: square-wave demodulation (0 deg -> U, 90 deg -> V)
            if p_qd(9)(1) = '0' then p10_u <= p9_s; else p10_u <= -p9_s; end if;
            if p_qd(9) = "00" or p_qd(9) = "11" then p10_v <= p9_s; else p10_v <= -p9_s; end if;
            -- 11, 12: running box sums; they snap back to zero once the box is full of blank samples
            p11_du <= resize(p10_u, 17) - resize(signed(rp_q2(31 downto 16)), 17);
            p11_dv <= resize(p10_v, 17) - resize(signed(rp_q2(15 downto 0)), 17);
            if s_v(6) = '1' then
                p_cb <= (others => '0');
            elsif p_cb /= 511 then
                p_cb <= p_cb + 1;
            end if;
            if p_cb > resize(c_n, 9) + 12 then p_rsy <= '1'; else p_rsy <= '0'; end if;
            if p_rsy = '1' then
                p12_au <= (others => '0'); p12_av <= (others => '0');
            else
                p12_au <= p12_au + resize(p11_du, 24);
                p12_av <= p12_av + resize(p11_dv, 24);
            end if;
            -- 13: Q4, limited to +-1024 chroma units (the comb's headroom)
            v_u22 := p12_au(23 downto 2);
            if v_u22(21) = '0' and v_u22(20 downto 14) /= "0000000" then p13_u <= to_signed(16383, 16);
            elsif v_u22(21) = '1' and v_u22(20 downto 14) /= "1111111" then p13_u <= to_signed(-16383, 16);
            else p13_u <= v_u22(15 downto 0); end if;
            v_u22 := p12_av(23 downto 2);
            if v_u22(21) = '0' and v_u22(20 downto 14) /= "0000000" then p13_v <= to_signed(16383, 16);
            elsif v_u22(21) = '1' and v_u22(20 downto 14) /= "1111111" then p13_v <= to_signed(-16383, 16);
            else p13_v <= v_u22(15 downto 0); end if;
            -- the chroma stream's valid: the line plus C_LD pixels
            if s_v(6) = '1' then s_ext <= to_unsigned(C_LD, 5);
            elsif s_ext /= 0 then s_ext <= s_ext - 1; end if;
            if s_v(6) = '1' or s_ext /= 0 then s_gx(7) <= '1'; else s_gx(7) <= '0'; end if;
            for i in 8 to 14 loop s_gx(i) <= s_gx(i - 1); end loop;
            -- comb store read: the pixel's index while it is in stage 11
            if s_gx(11) = '1' then p11_cc <= p11_cc + 1; else p11_cc <= (others => '0'); end if;
            -- 14: difference to the stored line state (zero on the field's first line, and with the comb off)
            if s_first = '1' or s_ncomb = '1' then v_su := (others => '0'); v_sv := (others => '0');
            else v_su := signed(c_q2(31 downto 16)); v_sv := signed(c_q2(15 downto 0)); end if;
            p14_du <= resize(p13_u, 17) - resize(v_su, 17); p14_su <= v_su;
            p14_dv <= resize(p13_v, 17) - resize(v_sv, 17); p14_sv <= v_sv;
            -- 15: the new state = state + difference / 2^n (rounded), one per two pixels, written back
            if s_gx(14) = '1' then p14_cw <= p14_cw + 1; else p14_cw <= (others => '0'); end if;
            if p14_cw(0) = '0' or s_ncomb = '1' then
                v_d17 := shift_right(p14_du + signed(resize(c_rnd, 17)), to_integer(c_nc));
                v_d17 := resize(p14_su, 17) + v_d17; p15_u <= v_d17(15 downto 0);
                v_d17 := shift_right(p14_dv + signed(resize(c_rnd, 17)), to_integer(c_nc));
                v_d17 := resize(p14_sv, 17) + v_d17; p15_v <= v_d17(15 downto 0);
            end if;
            c_we <= s_gx(14) and not p14_cw(0) and not s_ncomb;
            c_wa <= p14_cw(10 downto 1);
            s_gxq <= s_gx(14);
            if s_zone = '1' then s_first <= '1';
            elsif s_gxq = '1' and s_gx(14) = '0' then s_first <= '0'; end if;
            -- 16: x 1.5 (the comb's loss on a ghost) and to integer
            if s_ncomb = '1' then
                p16_u <= resize(shift_right(p15_u, 4), 13); p16_v <= resize(shift_right(p15_v, 4), 13);
            else
                v_o := resize(p15_u, 17) + resize(shift_right(p15_u, 1), 17); p16_u <= v_o(16 downto 4);
                v_o := resize(p15_v, 17) + resize(shift_right(p15_v, 1), 17); p16_v <= v_o(16 downto 4);
            end if;
            -- 17: limiter (the Saturation level)
            v_ln := signed(resize(c_l, 13));
            if p16_u > v_ln then p17_u <= v_ln(9 downto 0);
            elsif p16_u < -v_ln then p17_u <= -v_ln(9 downto 0);
            else p17_u <= p16_u(9 downto 0); end if;
            if p16_v > v_ln then p17_v <= v_ln(9 downto 0);
            elsif p16_v < -v_ln then p17_v <= -v_ln(9 downto 0);
            else p17_v <= p16_v(9 downto 0); end if;
            -- 18: the luma of C_LD pixels back (the ring's fabric copy is aligned with stage 17) to 10 bits;
            -- dot crawl: the chroma remodulated on the second encoder's subcarrier
            v_q := unsigned(rl_q2(10 downto 9)) + l_doff;
            v_ds := (others => '0');
            if v_q(1) = '0' then v_ds := v_ds + resize(p17_u, 11); else v_ds := v_ds - resize(p17_u, 11); end if;
            if v_q = "00" or v_q = "11" then p18_ds <= v_ds + resize(p17_v, 11); else p18_ds <= v_ds - resize(p17_v, 11); end if;
            v_yl := signed(rl_q2(8 downto 0));
            v_y := to_signed(502, 12) + resize(v_yl, 12) + resize(shift_right(v_yl, 1), 12) + resize(shift_right(v_yl, 3), 12);
            p18_yp <= unsigned(v_y(10 downto 0));
            p18_u <= p17_u; p18_v <= p17_v;
            s_vl(18) <= rl_q2(11);
            for i in 19 to 24 loop s_vl(i) <= s_vl(i - 1); end loop;
            -- 19
            if s_dots = '1' then
                p19_y <= signed(resize(p18_yp, 13)) + resize(p18_ds, 13) - resize(shift_right(p18_ds, 2), 13);
            else
                p19_y <= signed(resize(p18_yp, 13));
            end if;
            p19_u <= unsigned(p18_u) + to_unsigned(512, 10);
            p19_v <= unsigned(p18_v) + to_unsigned(512, 10);
            -- 20: clamp; neutral outside the generated avid
            if s_vl(19) = '0' then
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            else
                if p19_y < 64 then s_y20 <= to_unsigned(64, 10);
                elsif p19_y > 940 then s_y20 <= to_unsigned(940, 10);
                else s_y20 <= unsigned(p19_y(9 downto 0)); end if;
                s_u20 <= p19_u; s_v20 <= p19_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_vl(19);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums,
    -- resynced; centred 3 pixels behind the luma), luma [1 2 1]/4 and 13/16,
    -- the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_yf : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to neutral once 8 blank samples are in, so a glitched
            -- sample can't offset the colours for good.
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u20 - unsigned(rc_q2(19 downto 10));
                s_bav <= s_bav + s_v20 - unsigned(rc_q2(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 14) + s_baud; s_u22 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v22 <= v_sc(13 downto 4);
            -- luma: [1 2 1], 13/16
            s_yd1 <= s_y20; s_yd2 <= s_yd1;
            v_yf := resize(s_y20, 12) + resize(s_yd1 & '0', 12) + resize(s_yd2, 12);
            s_y22 <= v_yf(11 downto 2);
            if s_y22 > 64 then v_dl := s_y22 - 64; else v_dl := (others => '0'); end if;
            s_y23 <= s_y22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- 24: blanking gate
            if s_vl(23) = '1' then
                s_y24 <= s_y23; s_u24 <= s_u22; s_v24 <= s_v22;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= rs_q2(0);
    data_out.vsync_n <= rs_q2(1);
    data_out.field_n <= rs_q2(2);
    data_out.avid    <= s_vl(24);

end architecture crosscolor;
