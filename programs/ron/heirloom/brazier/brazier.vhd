-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Brazier v0.1.0 -- Heirloom from the video clips (phase 3), made from the still WIN_20230930_01_37_53 @ 0.5 s.
--
-- The still: a blue / violet nested diamond with glowing pink rims and red / green fringes, keyed over a vertical
-- ground ramp (near black at the top, bright red at the bottom) that carries three rows of keyed fins: grey
-- crescents hanging from the top, pink fins across the middle (stretched into long spikes on the right) and
-- magenta flames rising from the bottom.  In the clip one 5.5 s LFO drives everything: the fins' FM wave scrolls
-- down through the rows (the fins hook, stretch and flatten), and the diamond breathes: it squeezes to a wavy
-- sliver, drops, and rises again from below.
--
-- The patch, as fitted (brazier.py has every constant):
--   fins    phi = F(y) (x + 134) + 0.7786,  F(y) = F0 (1 - m cos(2 pi (y - 338 - 570 L) / 570))
--           lit where frac(phi) > T(y) (+ 1/4 on every other fin: big / small); colour by 1 - frac(phi)
--   diamond D = kx |x - cx(y)| + V(y);  g = m * cubic(D) (resets at D = 86, 398, 575 px);  c = frac(g), reversed
--           right of the centre; keyed D < 575 with a dark outline; ring palette per vertical zone
--   ground  a per-line colour ramp
--
--   K1  Rings    ring density x1/4 .. x4 (512 = the still)
--   K2  Fins     fin count across, x0.35 .. x2.8
--   K3  Sweep    the fins' FM depth: upright identical fins (0) .. the still (360) .. long slashes (1023)
--   K4  Speed    bipolar: the LFO (fin wave scroll + diamond breath); 700 = the clip's 5.5 s cycle
--   K5  Size     diamond zoom x0.3 .. x3.3
--   K6  Etch     how far the camera (or the Osc) bends the rings and the fins
--   S7  Twin     one diamond / two side by side
--   S8  Flames   Rose (the still) / Ember (orange-yellow fins)
--   S9  Slip     off / sync slips: whole bands of the picture torn sideways
--   S10 Colours  Violet (the still) / Copper (the diamond's chroma axes swapped)
--   S11 Source   Video / Osc
--   P12 MELT     breath depth + wobble + tear: a crisp still diamond (0) .. the clip (512) .. liquid (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, a cubic DDA that refills the 1024-entry ring
-- table every field), a per-line engine kicked at each line start (computes the NEXT line: fin frequency and start
-- phase, the diamond's centre, vertical term and start value, the ground colour), a beam with two accumulators,
-- a 15-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output
-- conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 with the running-sum resync, luma [1 2 1], luma 13/16) before the
-- blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.brazier_pkg.all;
use work.all;

architecture brazier of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_CX_PX : integer := 1012;      -- diamond centre x (virtual px)
    constant C_CY_PX : integer := 515;       -- diamond centre y (virtual lines)
    constant C_P0    : integer := 13062000;  -- fin phase offset 0.7786 (Q24)
    constant C_XORG  : integer := 268;       -- fin ramp origin, -134 virtual px = -268 h
    constant C_DKEY  : integer := 2300;      -- key: D < 575 px (Q2)
    constant C_DOUT  : integer := 2356;      -- outline: D < 589 px

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

    -- line buffer: luma >> 6 (4 bits; Osc: the lattice >> 4)
    type t_lb is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal lb : t_lb := (others => x"0");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(3 downto 0) := x"0";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(3 downto 0) := x"0";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sv_a : unsigned(8 downto 0) := (others => '0');
    signal r_sv_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_bg_a : unsigned(7 downto 0) := (others => '0');
    signal r_bg_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_tt_a : unsigned(8 downto 0) := (others => '0');
    signal r_tt_q : unsigned(7 downto 0) := (others => '0');
    signal r_br_a : unsigned(7 downto 0) := (others => '0');
    signal r_br_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_rp_a : unsigned(8 downto 0) := (others => '0');
    signal r_rp_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_fp_a : unsigned(7 downto 0) := (others => '0');
    signal r_fp_q : std_logic_vector(31 downto 0) := (others => '0');

    -- ring table: frac(m g(D)) (Q16) by D / 2 px; written by the cubic DDA in the vblank zone only
    type t_tb is array (0 to 1023) of unsigned(15 downto 0);
    signal tb : t_tb := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of tb : signal is true;
    signal t_we, t_run : std_logic := '0';
    signal t_wa, t_ra : unsigned(9 downto 0) := (others => '0');
    signal t_wd : unsigned(15 downto 0) := (others => '0');
    signal t_rq : unsigned(15 downto 0) := (others => '0');
    signal t_i  : unsigned(9 downto 0) := (others => '0');
    signal t_q, t_d1, t_d2 : unsigned(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 160 x 135 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(408, 10);
    signal s_oys : unsigned(11 downto 0) := to_unsigned(484, 12);
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
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(360 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_ember, s_slip, s_copper, s_src : std_logic := '0';

    -- per-field constants
    signal z_sh   : signed(1 downto 0) := (others => '0');
    signal z_m8   : unsigned(9 downto 0) := to_unsigned(256, 10);    -- ring density, Q8
    signal z_e2   : unsigned(15 downto 0) := to_unsigned(32768, 16); -- exp2 mantissa for the fin frequency
    signal z_sh2  : signed(1 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal z_ks   : unsigned(17 downto 0) := (others => '0');
    signal z_bre  : std_logic_vector(31 downto 0) := (others => '0');
    signal c_q0, c_d1, c_d2, c_d3 : unsigned(31 downto 0) := (others => '0');
    signal c_f0   : unsigned(16 downto 0) := to_unsigned(37010, 17);   -- fin frequency, Q24 cycles per h
    signal c_a    : unsigned(16 downto 0) := to_unsigned(5211, 17);    -- F0 * sweep
    signal c_kd   : unsigned(9 downto 0) := to_unsigned(256, 10);      -- size, Q8
    signal c_kxe  : unsigned(10 downto 0) := (others => '0');          -- (kx - 1) * melt, Q8
    signal c_cypx : unsigned(10 downto 0) := to_unsigned(C_CY_PX, 11); -- diamond centre line
    signal c_wob  : unsigned(8 downto 0) := to_unsigned(8, 9);         -- wobble px
    signal c_jit  : unsigned(6 downto 0) := to_unsigned(8, 7);         -- jitter, Q3
    signal c_kx   : unsigned(14 downto 0) := to_unsigned(256, 15);     -- horizontal gain, Q8 (twin x2)
    signal c_kv   : unsigned(11 downto 0) := to_unsigned(256, 12);     -- vertical gain, Q8 (twin x2)
    signal c_gd   : unsigned(7 downto 0) := (others => '0');           -- etch gain, D units per luma step
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_lst  : unsigned(15 downto 0) := (others => '0');          -- LFO step per field
    signal c_lfo  : unsigned(15 downto 0) := (others => '0');          -- LFO phase
    signal c_th, c_thw : unsigned(10 downto 0) := (others => '0');     -- per-line phase steps
    signal c_th0, c_thw0 : unsigned(15 downto 0) := (others => '0');
    signal c_tw   : unsigned(26 downto 0) := (others => '0');          -- twin reload KX * 1960
    signal c_kxs  : unsigned(16 downto 0) := to_unsigned(512, 17);     -- KX * xs (beam step)

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
    signal e_vyi  : unsigned(11 downto 0) := (others => '0');
    signal e_v15  : unsigned(15 downto 0) := (others => '0');
    signal e_th, e_thw : unsigned(15 downto 0) := (others => '0');
    signal e_t    : signed(12 downto 0) := (others => '0');
    signal e_cos  : signed(11 downto 0) := (others => '0');
    signal e_sin  : signed(11 downto 0) := (others => '0');
    signal e_vp   : unsigned(10 downto 0) := (others => '0');
    signal e_va   : unsigned(9 downto 0) := (others => '0');
    signal e_ta   : unsigned(10 downto 0) := (others => '0');
    signal e_fl   : unsigned(17 downto 0) := (others => '0');
    signal e_w    : signed(9 downto 0) := (others => '0');
    signal e_cxh  : signed(13 downto 0) := (others => '0');
    signal e_xa   : signed(13 downto 0) := (others => '0');
    signal e_neg  : std_logic := '0';
    signal e_jb   : unsigned(8 downto 0) := (others => '1');
    signal e_sb   : unsigned(7 downto 0) := (others => '1');
    signal e_rnd  : unsigned(31 downto 0) := x"2545F491";
    signal e_jn   : std_logic := '0';
    signal e_jpx  : signed(8 downto 0) := (others => '0');     -- jitter + tear, px
    signal e_jbase : signed(8 downto 0) := (others => '0');    -- jitter of the band, px
    signal e_tear : signed(8 downto 0) := (others => '0');
    signal e_tcnt : unsigned(3 downto 0) := (others => '0');
    signal e_slp  : signed(9 downto 0) := (others => '0');     -- slip, px
    signal m_a    : unsigned(29 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(29 downto 0) := (others => '0');
    -- next-line results
    signal n_fst  : unsigned(19 downto 0) := (others => '0');
    signal n_ph0  : unsigned(24 downto 0) := (others => '0');
    signal n_vl   : unsigned(12 downto 0) := (others => '0');
    signal n_s0   : signed(27 downto 0) := (others => '0');
    signal n_cxh  : unsigned(12 downto 0) := (others => '1');
    signal n_t    : unsigned(7 downto 0) := (others => '1');
    signal n_tsm  : unsigned(8 downto 0) := (others => '1');
    signal n_band : unsigned(1 downto 0) := (others => '0');
    signal n_zone : unsigned(2 downto 0) := (others => '0');
    signal n_bg, n_bgd : std_logic_vector(29 downto 0) := (others => '0');

    -- beam / per-line values in use
    signal b_s   : signed(27 downto 0) := (others => '0');
    signal b_ph  : unsigned(24 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_xh, b_xh1, b_xh2 : unsigned(12 downto 0) := (others => '0');
    signal b_tc  : std_logic := '0';
    signal l_fst : unsigned(19 downto 0) := (others => '0');
    signal l_vl  : unsigned(12 downto 0) := (others => '0');
    signal l_cxh : unsigned(12 downto 0) := (others => '1');
    signal l_t   : unsigned(7 downto 0) := (others => '1');
    signal l_tsm : unsigned(8 downto 0) := (others => '1');
    signal l_band : unsigned(1 downto 0) := (others => '0');
    signal l_zone : unsigned(2 downto 0) := (others => '0');
    signal l_bg, l_bgd : std_logic_vector(29 downto 0) := (others => '0');
    signal l_kxs : unsigned(16 downto 0) := (others => '0');

    -- pixel path
    signal q2_d, q3_d, q4_d, q5_d, q6_d : unsigned(12 downto 0) := (others => '1');
    signal q2_ph, q3_ph, q4_ph, q5_ph, q6_ph : unsigned(16 downto 0) := (others => '0');
    signal q4_lm : unsigned(3 downto 0) := (others => '0');
    signal q5_ed : unsigned(11 downto 0) := (others => '0');
    type t_flags is array (2 to 14) of std_logic;
    signal q_rev, q_key, q_out, q_fin : t_flags := (others => '0');
    signal q9_tq  : unsigned(15 downto 0) := (others => '0');
    signal q9_fc, q10_fc, q11_fc, q12_fc, q13_fc, q14_fc : std_logic_vector(29 downto 0) := (others => '0');
    signal q12_rc, q13_rc, q14_rc : std_logic_vector(29 downto 0) := (others => '0');
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
    -- Line buffer (collisions possible: read-before-write kept), ROMs, ring table
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
            r_ex_q <= C_BZ_EXP2(to_integer(r_ex_a));
            r_sv_q <= C_BZ_SV(to_integer(r_sv_a));
            r_bg_q <= C_BZ_BG(to_integer(r_bg_a));
            r_tt_q <= C_BZ_TT(to_integer(r_tt_a));
            r_br_q <= C_BZ_BRE(to_integer(r_br_a));
            r_rp_q <= C_BZ_RPAL(to_integer(r_rp_a));
            r_fp_q <= C_BZ_FPAL(to_integer(r_fp_a));
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
        variable v_e  : signed(10 downto 0);
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
                s_ember  <= registers_in(6)(1);
                s_slip   <= registers_in(6)(2);
                s_copper <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(816, 10);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(612, 10);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(408, 10);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 21) * to_unsigned(484, 9), 8), 12);

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
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(7549747, 36); z_fb <= resize(z_m8, 18);             -- Q0 = 0.45 m
                    when 1  => z_fa <= (others => '0');          z_fb <= (others => '0');              -- (spare)
                    when 2  => z_fa <= to_unsigned(262448, 36);  z_fb <= resize(z_m8, 18);             -- delta 1
                    when 3  => z_fa <= to_unsigned(2528, 36);    z_fb <= resize(z_m8, 18);             -- -delta 2
                    when 4  => z_fa <= to_unsigned(4666, 36);    z_fb <= resize(z_m8, 18);             -- delta 3 (<< 8)
                    when 5  => z_fa <= to_unsigned(37010, 36); z_fb <= resize(z_e2, 18);               -- F0
                    when 6  => z_fa <= resize(s_g3(12 downto 3), 36); z_fb <= to_unsigned(205, 18);    -- sweep
                    when 7  => z_fa <= resize(c_f0, 36);  z_fb <= z_ks;                                -- A
                    when 8  =>                                                                         -- size exponent
                        if s_g5(12 downto 3) > 512 then z_fa <= resize(s_g5(12 downto 3) - 512, 36); z_neg <= '1';
                        else z_fa <= resize(512 - s_g5(12 downto 3), 36); z_neg <= '0'; end if;
                        z_fb <= to_unsigned(218, 18);
                    when 9  => z_fa <= resize(unsigned(z_bre(31 downto 22)), 36); z_fb <= resize(s_gp(12 downto 3), 18); -- kx - 1
                    when 10 => z_fa <= resize(unsigned(z_bre(21 downto 14)), 36); z_fb <= resize(s_gp(12 downto 3), 18); -- cy
                    when 11 => z_fa <= resize(unsigned(z_bre(13 downto 6)), 36);  z_fb <= resize(s_gp(12 downto 3), 18); -- wobble
                    when 12 => z_fa <= resize(unsigned(z_bre(5 downto 0)), 36);   z_fb <= resize(s_gp(12 downto 3), 18); -- jitter
                    when 13 => z_fa <= resize(c_kd, 36);  z_fb <= resize(c_kxe, 18) + 256;             -- KX
                    when 14 => z_fa <= resize(s_g6(12 downto 3), 36); z_fb <= resize(s_g6(12 downto 3), 18); -- etch^2
                    when 15 => z_fa <= resize(s_vm, 36);  z_fb <= resize(s_vm, 18);                    -- speed^2
                    when 16 => z_fa <= resize(c_vsq, 36); z_fb <= to_unsigned(201, 18);                -- LFO step
                    when 17 => z_fa <= resize(s_ys, 36);  z_fb <= to_unsigned(460, 18);                -- fin wave step
                    when 18 => z_fa <= resize(s_ys, 36);  z_fb <= to_unsigned(1092, 18);               -- wobble step
                    when others => z_fa <= resize(c_kx, 36); z_fb <= to_unsigned(1960, 18);            -- twin reload
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
                    -- LFO (last field's step); breath table; ring density exponent
                    when 20 =>
                        if s_vneg = '1' then c_lfo <= c_lfo - c_lst; else c_lfo <= c_lfo + c_lst; end if;
                    when 21 =>
                        r_br_a <= c_lfo(15 downto 8);
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(9 downto 8);
                        c_th0  <= to_unsigned(65536 - 38861, 16) - c_lfo;
                        c_thw0 <= (c_lfo(14 downto 0) & '0') + 13107;
                    when 23 =>
                        z_bre <= r_br_q;
                        case to_integer(z_sh) is                       -- m8 = 2^(e / 256), Q8
                            when 1      => z_m8 <= resize(shift_right(r_ex_q, 6), 10);
                            when 0      => z_m8 <= resize(shift_right(r_ex_q, 7), 10);
                            when -1     => z_m8 <= resize(shift_right(r_ex_q, 8), 10);
                            when others => z_m8 <= resize(shift_right(r_ex_q, 9), 10);
                        end case;
                        -- fin frequency exponent: 3/4 (k2 - 512) (+-1.5 octaves)
                        v_e := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        v_e := v_e - shift_right(v_e, 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh2 <= v_e(9 downto 8);
                    when 25 => z_e2 <= r_ex_q;

                    when 52  => c_q0 <= f_acc(31 downto 0);
                    when 100 => c_d1 <= f_acc(31 downto 0);
                    when 124 => c_d2 <= (not f_acc(31 downto 0)) + 1;
                    when 148 => c_d3 <= resize(f_acc(31 downto 8), 32);
                    when 172 =>                       -- F0 = 37010 * mt >> (15 - sh2)
                        case to_integer(z_sh2) is
                            when 1      => c_f0 <= f_acc(30 downto 14);
                            when 0      => c_f0 <= resize(f_acc(31 downto 15), 17);
                            when -1     => c_f0 <= resize(f_acc(31 downto 16), 17);
                            when others => c_f0 <= resize(f_acc(31 downto 17), 17);
                        end case;
                    when 196 => z_ks <= f_acc(17 downto 0);
                    when 220 => c_a <= resize(f_acc(35 downto 19), 17);
                    when 244 =>                       -- size exponent e5 = (512 - k5) 218 / 256
                        v_e := signed(resize(f_acc(17 downto 8), 11));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(9 downto 8);
                    when 246 =>
                        case to_integer(z_sh) is
                            when 1      => c_kd <= resize(shift_right(r_ex_q, 6), 10);
                            when 0      => c_kd <= resize(shift_right(r_ex_q, 7), 10);
                            when -1     => c_kd <= resize(shift_right(r_ex_q, 8), 10);
                            when others => c_kd <= resize(shift_right(r_ex_q, 9), 10);
                        end case;
                    when 268 => c_kxe <= resize(f_acc(19 downto 9), 11);
                    when 292 => c_cypx <= to_unsigned(C_CY_PX, 11) + f_acc(18 downto 8);
                    when 316 => c_wob <= f_acc(17 downto 9);
                    when 340 => c_jit <= f_acc(15 downto 9);
                    when 364 =>
                        if s_twin = '1' then
                            c_kx <= f_acc(21 downto 7); c_kv <= resize(c_kd, 11) & '0';
                        else
                            c_kx <= resize(f_acc(22 downto 8), 15); c_kv <= resize(c_kd, 12);
                        end if;
                    when 365 =>
                        case to_integer(s_xs) is
                            when 4      => c_kxs <= resize(c_kx, 17) sll 2;
                            when 3      => c_kxs <= (resize(c_kx, 17) sll 1) + resize(c_kx, 17);
                            when others => c_kxs <= resize(c_kx, 17) sll 1;
                        end case;
                    when 388 => c_gd  <= f_acc(19 downto 12);
                    when 412 => c_vsq <= f_acc(17 downto 0);
                    when 436 => c_lst <= f_acc(30 downto 15);
                    when 460 => c_th  <= f_acc(20 downto 10);
                    when 484 => c_thw <= f_acc(20 downto 10);
                    when 508 => c_tw  <= f_acc(26 downto 0);
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
    -- Ring table fill (vblank zone, zc 600 .. 1624): a cubic forward-difference
    -- DDA, Q(i + 1) = Q + D1, D1 += D2, D2 += D3, mod 2^32 (Q16.16 cycles); the
    -- table keeps the fraction, frac(m g(2 i)).
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            t_we <= '0';
            if s_zone = '1' and s_zc = 600 then
                t_run <= '1'; t_i <= (others => '0'); t_q <= c_q0; t_d1 <= c_d1; t_d2 <= c_d2;
            elsif t_run = '1' then
                t_we <= '1'; t_wa <= t_i; t_wd <= t_q(31 downto 16);
                t_q <= t_q + t_d1; t_d1 <= t_d1 + t_d2; t_d2 <= t_d2 + c_d3; t_i <= t_i + 1;
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
    -- write of the luma's top 4 bits above black
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
            s_osum <= v_s;
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_osum(7 downto 4));
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then v_l := v_l - 16; else v_l := (others => '0'); end if;
                s_lb_wd <= std_logic_vector(v_l(7 downto 4));
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17):
    --   fin frequency Fl = F0 -/+ A |cos| >> 11, its pixel step and start phase
    --   wobble w = wob sin >> 11; jitter / tear (bands of 8 virtual lines), slip
    --   (bands of 16); the diamond centre, V term and start value S0
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t   : signed(12 downto 0);
        variable v_a   : unsigned(12 downto 0);
        variable v_p   : signed(13 downto 0);
        variable v_x   : signed(13 downto 0);
        variable v_r   : unsigned(31 downto 0);
        variable v_jm  : signed(13 downto 0);
        variable v_tm  : signed(14 downto 0);
        variable v_cx  : signed(11 downto 0);
        variable v_ph  : unsigned(24 downto 0);
        variable v_bg  : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1700 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0'); e_th <= c_th0; e_thw <= c_thw0;
                e_lc <= (others => '0');
                e_jb <= (others => '1'); e_sb <= (others => '1'); e_tcnt <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 20);
                e_th <= e_th + resize(c_th, 16); e_thw <= e_thw + resize(c_thw, 16);
                e_lc <= (others => '0');
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(28 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_vyi <= e_vy(19 downto 8);
                    r_sv_a <= '0' & (e_th(15 downto 8) + 64);           -- cos
                when 2 =>
                    e_v15 <= shift_left(resize(e_vyi, 16), 4) - resize(e_vyi, 16);
                    e_t <= signed(resize(e_vyi, 13)) - signed(resize(c_cypx, 13));
                    -- new jitter band (8 virtual lines) / slip band (16)
                    if e_vyi(11 downto 3) /= e_jb then e_jn <= '1'; else e_jn <= '0'; end if;
                    e_jb <= e_vyi(11 downto 3);
                    if e_vyi(11 downto 4) /= e_sb then
                        e_sb <= e_vyi(11 downto 4);
                        v_r := e_rnd;
                        if s_slip = '1' and v_r(21 downto 20) = "00" then
                            e_slp <= resize(signed(v_r(31 downto 24)), 10) sll 1;
                        else
                            e_slp <= (others => '0');
                        end if;
                    end if;
                when 3 =>
                    if e_t < 0 then e_ta <= unsigned(resize(-e_t, 11)); else e_ta <= unsigned(resize(e_t, 11)); end if;
                    r_bg_a <= e_v15(13 downto 6);
                    r_tt_a <= e_v15(13 downto 5);
                    n_zone <= e_v15(13 downto 11);
                    -- xorshift32 per line
                    v_r := e_rnd;
                    v_r := v_r xor (v_r(18 downto 0) & "0000000000000");
                    v_r := v_r xor ("00000000000000000" & v_r(31 downto 17));
                    v_r := v_r xor (v_r(26 downto 0) & "00000");
                    e_rnd <= v_r;
                when 4 =>
                    e_cos <= signed(r_sv_q(11 downto 0));
                    r_sv_a <= '0' & e_thw(15 downto 8);                   -- sin
                    -- jitter: +-(r6 jit) >> 7 px, tears +-(r7 jit) >> 5 for 2..9 bands (1 band in 32)
                    if e_jn = '1' then
                        v_jm := signed(e_rnd(5 downto 0)) * signed('0' & c_jit);
                        v_tm := signed(e_rnd(22 downto 16)) * signed('0' & c_jit);
                        if e_tcnt /= 0 then
                            e_tcnt <= e_tcnt - 1;
                        elsif e_rnd(13 downto 8) < 2 then
                            e_tcnt <= resize(e_rnd(26 downto 24), 4) + 2;
                            e_tear <= resize(v_tm(13 downto 5), 9);
                        else
                            e_tear <= (others => '0');
                        end if;
                        e_jbase <= resize(v_jm(12 downto 7), 9);
                    end if;
                when 5 =>
                    if e_cos < 0 then e_neg <= '1'; v_a := unsigned(resize(-e_cos, 13));
                    else e_neg <= '0'; v_a := unsigned(resize(e_cos, 13)); end if;
                    m_a <= resize(c_a, 30); m_b <= resize(v_a, 16); m_acc <= (others => '0');
                    if e_t > 1019 then r_sv_a <= (others => '1');
                    else r_sv_a <= '1' & unsigned(e_t(9 downto 2)); end if;
                when 6 =>
                    e_sin <= signed(r_sv_q(11 downto 0));
                    if e_t > 1019 then r_sv_a <= (others => '1');
                    else r_sv_a <= '1' & (unsigned(e_t(9 downto 2)) + 1); end if;
                    v_bg := r_bg_q;
                    n_bg <= v_bg(31 downto 2);
                    n_band <= unsigned(v_bg(1 downto 0));
                    n_t <= r_tt_q;
                    n_tsm <= resize(r_tt_q, 9) + 64;
                    e_jpx <= e_jbase + e_tear;
                when 7 =>
                    e_va <= unsigned(r_sv_q(9 downto 0));
                when 8 =>                     -- V profile: below, the table interpolated over its 4-line steps
                    if e_t >= 0 then
                        v_a := resize(unsigned(r_sv_q(9 downto 0)), 13) - resize(e_va, 13);   -- step (0..6)
                        case to_integer(unsigned(e_t(1 downto 0))) is
                            when 0      => v_a := (others => '0');
                            when 1      => v_a := shift_right(v_a, 2);
                            when 2      => v_a := shift_right(v_a, 1);
                            when others => v_a := shift_right(v_a, 1) + shift_right(v_a, 2);
                        end case;
                        e_vp <= resize(e_va, 11) + resize(v_a, 11);
                    else
                        e_vp <= resize(e_ta - shift_right(e_ta, 4), 11);
                    end if;
                when 10 =>                    -- above: 0.953 |t| (second subtraction a clock later)
                    if e_t < 0 then e_vp <= e_vp - resize(shift_right(e_ta, 6), 11); end if;
                when 9 =>
                    -- outline: the ground at half level
                    n_bgd(29 downto 20) <= std_logic_vector(to_unsigned(32, 10) + shift_right(unsigned(n_bg(29 downto 20)), 1));
                    n_bgd(19 downto 10) <= std_logic_vector(to_unsigned(256, 10) + shift_right(unsigned(n_bg(19 downto 10)), 1));
                    n_bgd(9 downto 0)   <= std_logic_vector(to_unsigned(256, 10) + shift_right(unsigned(n_bg(9 downto 0)), 1));
                when 22 =>                    -- Fl = F0 -/+ A |cos| >> 11
                    if e_neg = '1' then e_fl <= resize(c_f0, 18) + resize(m_acc(28 downto 11), 18);
                    else e_fl <= resize(c_f0, 18) - resize(m_acc(28 downto 11), 18); end if;
                    if e_sin < 0 then e_neg <= '1'; v_a := unsigned(resize(-e_sin, 13));
                    else e_neg <= '0'; v_a := unsigned(resize(e_sin, 13)); end if;
                    m_a <= resize(c_wob, 30); m_b <= resize(v_a, 16); m_acc <= (others => '0');
                when 23 =>
                    case to_integer(s_xs) is
                        when 4      => n_fst <= resize(e_fl, 20) sll 2;
                        when 3      => n_fst <= (resize(e_fl, 20) sll 1) + resize(e_fl, 20);
                        when others => n_fst <= resize(e_fl, 20) sll 1;
                    end case;
                when 39 =>                    -- wobble, centre (h); fin start: Fl (x0 + 268 - 2 slip)
                    if e_neg = '1' then e_w <= -signed(resize(m_acc(19 downto 11), 10));
                    else e_w <= signed(resize(m_acc(19 downto 11), 10)); end if;
                    v_x := signed(resize(s_x0, 14)) + C_XORG - shift_left(resize(e_slp, 14), 1);
                    if v_x < 0 then e_neg <= '1'; v_x := -v_x; else e_neg <= '0'; end if;
                    m_a <= resize(e_fl, 30); m_b <= resize(unsigned(v_x(12 downto 0)), 16); m_acc <= (others => '0');
                when 40 =>
                    v_cx := to_signed(C_CX_PX, 12) + resize(e_w, 12) + resize(e_jpx, 12) + resize(e_slp, 12);
                    e_cxh <= resize(v_cx, 13) & '0';
                when 41 =>
                    if s_twin = '1' then
                        e_xa <= e_cxh - 980 - signed(resize(s_x0, 14));
                        n_cxh <= unsigned(e_cxh(12 downto 0));
                    else
                        e_xa <= e_cxh - signed(resize(s_x0, 14));
                        n_cxh <= (others => '1');
                    end if;
                when 56 =>
                    if e_neg = '1' then v_ph := to_unsigned(C_P0, 25) - m_acc(24 downto 0);
                    else v_ph := to_unsigned(C_P0, 25) + m_acc(24 downto 0); end if;
                    n_ph0 <= v_ph;
                    m_a <= resize(c_kv, 30); m_b <= resize(e_vp, 16); m_acc <= (others => '0');
                when 73 =>                    -- V term (Q2 px), saturated
                    if m_acc(29 downto 19) /= 0 then n_vl <= (others => '1');
                    else n_vl <= m_acc(18 downto 6); end if;
                    if e_xa < 0 then e_neg <= '1'; v_p := -e_xa; else e_neg <= '0'; v_p := e_xa; end if;
                    m_a <= resize(c_kx, 30); m_b <= resize(unsigned(v_p(12 downto 0)), 16); m_acc <= (others => '0');
                when 90 =>                    -- S0 = KX (cxl - x0)
                    if e_neg = '1' then n_s0 <= -signed(resize(m_acc(26 downto 0), 28));
                    else n_s0 <= signed(resize(m_acc(26 downto 0), 28)); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: S = KX (cx - x) (diamond, signed), Phi = fin phase; twin adds
    -- KX * 1960 where x crosses the original centre
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_st : signed(27 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_s  <= n_s0;
                b_ph <= n_ph0;
                b_px <= (others => '0');
                b_xh  <= resize(s_x0, 13);
                b_xh1 <= resize(s_x0, 13) + s_xs;
                b_xh2 <= resize(s_x0, 13) + shift_left(resize(s_xs, 13), 1);
                if resize(s_x0, 13) + s_xs >= n_cxh and resize(s_x0, 13) < n_cxh then b_tc <= '1'; else b_tc <= '0'; end if;
                l_fst <= n_fst; l_vl <= n_vl; l_cxh <= n_cxh; l_t <= n_t; l_tsm <= n_tsm;
                l_band <= n_band; l_zone <= n_zone; l_bg <= n_bg; l_bgd <= n_bgd; l_kxs <= c_kxs;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_xh <= b_xh1; b_xh1 <= b_xh2; b_xh2 <= b_xh2 + s_xs;
                if b_xh2 >= l_cxh and b_xh1 < l_cxh then b_tc <= '1'; else b_tc <= '0'; end if;
                if b_tc = '1' then v_st := signed(resize(c_tw, 28)) - signed(resize(l_kxs, 28));
                else v_st := -signed(resize(l_kxs, 28)); end if;
                b_s  <= b_s + v_st;
                b_ph <= b_ph + resize(l_fst, 25);
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : unsigned(13 downto 0);
        variable v_ab  : unsigned(26 downto 0);
        variable v_c6  : unsigned(5 downto 0);
        variable v_rc, v_fc : std_logic_vector(29 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: |S| >> 7 (ones' complement: within 1/128 of a D step), saturated; side; fin phase Q16
            if b_s(27) = '1' then v_ab := not unsigned(b_s(26 downto 0)); else v_ab := unsigned(b_s(26 downto 0)); end if;
            if v_ab(26 downto 20) /= 0 then q2_d <= (others => '1'); else q2_d <= v_ab(19 downto 7); end if;
            q_rev(2) <= b_s(27);
            q2_ph <= b_ph(24 downto 8);
            -- 3: + V term
            v_d := resize(q2_d, 14) + l_vl;
            if v_d(13) = '1' then q3_d <= (others => '1'); else q3_d <= v_d(12 downto 0); end if;
            q3_ph <= q2_ph;
            -- 4: luma from the line buffer
            q4_d <= q3_d; q4_ph <= q3_ph; q4_lm <= unsigned(s_lb_q);
            -- 5: etch product
            q5_ed <= q4_lm * c_gd;
            q5_d <= q4_d; q5_ph <= q4_ph;
            -- 6: + etch (diamond field and fin phase)
            v_d := resize(q5_d, 14) + q5_ed;
            if v_d(13) = '1' then q6_d <= (others => '1'); else q6_d <= v_d(12 downto 0); end if;
            q6_ph <= q5_ph + shift_left(resize(q5_ed, 17), 4);
            -- 7: ring table address, key / outline tests; fin test and palette address
            t_ra <= q6_d(12 downto 3);
            if q6_d < C_DKEY then q_key(7) <= '1'; else q_key(7) <= '0'; end if;
            if q6_d < C_DOUT then q_out(7) <= '1'; else q_out(7) <= '0'; end if;
            if q6_ph(16) = '1' then
                if resize(q6_ph(15 downto 8), 9) > l_tsm then q_fin(7) <= '1'; else q_fin(7) <= '0'; end if;
            else
                if q6_ph(15 downto 8) > l_t then q_fin(7) <= '1'; else q_fin(7) <= '0'; end if;
            end if;
            r_fp_a <= l_band & not q6_ph(15 downto 10);
            -- 8: table / fin palette in the EBRs
            -- 9: fabric copies
            q9_tq <= t_rq;
            q9_fc <= r_fp_q(31 downto 2);
            -- 10: colour index (reversed right of the centre) -> ring palette address
            if q_rev(9) = '1' then v_c6 := not q9_tq(15 downto 10); else v_c6 := q9_tq(15 downto 10); end if;
            r_rp_a <= l_zone & v_c6;
            q10_fc <= q9_fc;
            -- 11: ring palette in the EBR
            q11_fc <= q10_fc;
            -- 12: fabric copy
            q12_rc <= r_rp_q(31 downto 2);
            q12_fc <= q11_fc;
            -- 13: Copper (diamond U <-> V), Ember (fins: V -> 1023 - V)
            if s_copper = '1' then v_rc := q12_rc(29 downto 20) & q12_rc(9 downto 0) & q12_rc(19 downto 10);
            else v_rc := q12_rc; end if;
            q13_rc <= v_rc;
            if s_ember = '1' then v_fc := q12_fc(29 downto 10) & std_logic_vector(1023 - unsigned(q12_fc(9 downto 0)));
            else v_fc := q12_fc; end if;
            q13_fc <= v_fc;
            -- 14: copy
            q14_rc <= q13_rc; q14_fc <= q13_fc;
            for i in 3 to 14 loop
                q_rev(i) <= q_rev(i - 1);
                if i > 7 then q_key(i) <= q_key(i - 1); q_out(i) <= q_out(i - 1); q_fin(i) <= q_fin(i - 1); end if;
            end loop;
            -- 15: compose (diamond > fins > outline > ground); neutral outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            elsif q_key(14) = '1' then
                s_y15 <= unsigned(q14_rc(29 downto 20)); s_u15 <= unsigned(q14_rc(19 downto 10)); s_v15 <= unsigned(q14_rc(9 downto 0));
            elsif q_fin(14) = '1' then
                s_y15 <= unsigned(q14_fc(29 downto 20)); s_u15 <= unsigned(q14_fc(19 downto 10)); s_v15 <= unsigned(q14_fc(9 downto 0));
            elsif q_out(14) = '1' then
                s_y15 <= unsigned(l_bgd(29 downto 20)); s_u15 <= unsigned(l_bgd(19 downto 10)); s_v15 <= unsigned(l_bgd(9 downto 0));
            else
                s_y15 <= unsigned(l_bg(29 downto 20)); s_u15 <= unsigned(l_bg(19 downto 10)); s_v15 <= unsigned(l_bg(9 downto 0));
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

end architecture brazier;
