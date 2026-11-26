-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Overshoot v0.1.0 -- Onevolt: horizontal-only filters (lit edges with comet tails, bas-relief, smear).
--
-- The rack:
--   ramps -> LZX Navigator (K2 Rotation + a 50 s sway) -> LZX DSG3 (H fold, V folded twice, Average)
--         -> LZX Keychain (K1 Thresh + a 9 s LFO; camera on its CV) -> a hard key with a 1-px coverage edge
--   S10 Bars: DSG3 unplugged, an LZX DWO3 triangle (H range, hard reset, FM by a V-range sine and the
--         Navigator H ramp) into the Keychain instead (a curtain of wavy bars, K1 = their width)
--   S9: the Keychain's cable moves from DSG3 Average to DSG3 Maximum (rhombi -> bricks)
--   key + camera luma (K6; S11 Osc) -> LZX Contour: three high-passes, one input normalled to all three
--         (P12 master cutoff, K3 spread, a 23 s lean)  |  S7: Syntonie Detail Blur in Contour's place (the same
--         filters' low-pass outputs)
--         -> LZX Curtain rectifier (S8: Both edges / Signed on a pedestal), Curtain Mix (K4: the dry key) -> gain
--         -> LZX SMX3 (K5: which filter feeds R, G, B, 8 colourings blended) -> encoder soft clip
--
-- Engine: vblank-zone sequencer (one 32-bit serial multiplier, 13 slots with ROM constants, shared knob glide);
-- per-line DDAs for the rotated ramps and the DWO3 frequency (its V-range sine by a line engine); per-pixel
-- Navigator / DWO3 accumulators; the filter bank = three one-pole integrators (2^-3, 2^-6, 2^-9, reset each
-- line) plus a one-pixel difference, each channel crossfading two adjacent members; a 28-stage pixel path.
-- Generated avid; output conditioning (luma [1 2 1], chroma one [1 2 2 2 2 2 2 2 1]/16 box with the per-line
-- resync, luma 13/16) before the blanking gate.  Virtual 1080-line coordinates; colours in the hardware
-- convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.overshoot_pkg.all;
use work.all;

architecture overshoot of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks
    constant C_GLIDE_BYPASS : boolean := false;   -- sim-only copies set true (knob = glide state at once)

    -- per timing id: x step (h units / px), virtual lines per field line (Q8)
    function f_xs(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_cls(t : std_logic_vector(3 downto 0)) return integer is   -- 0: 1080, 1: 720p, 2: SD
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return 2;
            when 5 | 6 | 14     => return 1;
            when others         => return 0;
        end case;
    end function;
    function f_ku(t : std_logic_vector(3 downto 0)) return integer is
    begin
        case f_cls(t) is when 2 => return 52429; when 1 => return 39322; when others => return 26214; end case;
    end function;
    function f_kv(t : std_logic_vector(3 downto 0)) return integer is
    begin
        case f_cls(t) is when 2 => return 104858; when 1 => return 78643; when others => return 52429; end case;
    end function;
    function f_kuy(t : std_logic_vector(3 downto 0)) return integer is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return 116530;
            when 4          => return 58982;
            when 8          => return 98304;
            when 12         => return 49152;
            when 5 | 6 | 14 => return 39322;
            when 1 | 2 | 10 => return 52429;
            when others     => return 26214;
        end case;
    end function;
    function f_kb(t : std_logic_vector(3 downto 0)) return integer is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return 3787;
            when 4          => return 1917;
            when 8          => return 3195;
            when 12         => return 1597;
            when 5 | 6 | 14 => return 958;
            when 1 | 2 | 10 => return 852;
            when others     => return 426;
        end case;
    end function;
    function f_dv(t : std_logic_vector(3 downto 0)) return integer is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return 6474;
            when 4          => return 3277;
            when 8          => return 5461;
            when 12         => return 2731;
            when 5 | 6 | 14 => return 2185;
            when 1 | 2 | 10 => return 2913;
            when others     => return 1456;
        end case;
    end function;
    function f_ux(t : std_logic_vector(3 downto 0)) return integer is
    begin
        if f_cls(t) = 2 then return -576; else return -768; end if;
    end function;
    function f_vx(t : std_logic_vector(3 downto 0)) return integer is
    begin
        if f_cls(t) = 2 then return 1152; else return 1536; end if;
    end function;
    function f_kac(t : std_logic_vector(3 downto 0)) return integer is
    begin
        if f_cls(t) = 0 then return -25559; else return -38339; end if;
    end function;
    function f_kas(t : std_logic_vector(3 downto 0)) return integer is
    begin
        case f_cls(t) is when 2 => return -28754; when 1 => return -21566; when others => return -14377; end case;
    end function;
    function f_kaa(t : std_logic_vector(3 downto 0)) return integer is
    begin
        case f_cls(t) is when 2 => return 1704; when 1 => return 958; when others => return 426; end case;
    end function;
    function f_f0q(t : std_logic_vector(3 downto 0)) return integer is
    begin
        case f_cls(t) is when 2 => return 1817536; when 1 => return 1363152; when others => return 908768; end case;
    end function;
    function f_km0(t : std_logic_vector(3 downto 0)) return integer is   -- 43 - log2(pixel size) x 64 / 3
    begin
        case f_cls(t) is when 2 => return 22; when 1 => return 31; when others => return 43; end case;
    end function;
    -- the Osc lattice's line step: (ys x 121) >> 8
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(537, 10);
            when 4          => return to_unsigned(272, 10);
            when 8          => return to_unsigned(453, 10);
            when 12         => return to_unsigned(226, 10);
            when 5 | 6 | 14 => return to_unsigned(181, 10);
            when 1 | 2 | 10 => return to_unsigned(242, 10);
            when others     => return to_unsigned(121, 10);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- 24-bit phase -> 12-bit fold: 4095 at the cell edge, 0 at the centre
    function f_tri12(u : unsigned(23 downto 0)) return unsigned is
    begin
        if u(23) = '1' then return u(22 downto 11); else return not u(22 downto 11); end if;
    end function;
    function f_tri10a(p : unsigned(10 downto 0)) return unsigned is      -- 11-bit phase triangle
    begin
        if p(10) = '1' then return not p(9 downto 0); else return p(9 downto 0); end if;
    end function;
    function f_tri10b(p : unsigned(11 downto 0)) return unsigned is      -- 12-bit phase triangle
    begin
        if p(11) = '1' then return not p(10 downto 1); else return p(10 downto 1); end if;
    end function;
    type t_ped is array (0 to 2) of integer;
    constant C_PED : t_ped := (15, 33, 51);    -- x the channel gains 1, 1.25, 1.5

    -- luma cell store: one 12-bit vertical-slew state per 8-pixel cell (read a cell ahead of the write)
    type t_lb is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(7 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(15 downto 0) := x"0000";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(15 downto 0) := x"0000";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav, s_f1, s_f2 : std_logic := '0';
    signal s_lc    : unsigned(2 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(7 downto 0) := (others => '0');
    signal s_acc, s_sum8 : unsigned(10 downto 0) := (others => '0');
    signal s_cur, s_old, s_new : unsigned(11 downto 0) := (others => '0');
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_h2  : unsigned(11 downto 0) := (others => '0');
    signal s_la, s_lbh : unsigned(9 downto 0) := (others => '0');
    signal s_caml  : unsigned(11 downto 0) := (others => '0');


    -- sine ROM (quarter wave, 4096 per cycle) and its read pipeline
    signal s_ang  : unsigned(11 downto 0) := (others => '0');
    signal r_s_a  : unsigned(9 downto 0) := (others => '0');
    signal r_s_q  : unsigned(15 downto 0) := (others => '0');
    signal r_q1, r_q2 : std_logic := '0';
    signal r_sp1, r_sp2 : std_logic := '0';
    signal rs_m   : unsigned(15 downto 0) := (others => '0');
    signal rs_s   : std_logic := '0';

    -- output filter stores: the chroma box's 8-back ring, and a luma ring read 4 back
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da : t_dl := (others => (others => '0'));
    attribute no_rw_check of da : signal is true;
    type t_dly is array (0 to 15) of std_logic_vector(15 downto 0);
    signal dy : t_dly := (others => (others => '0'));
    attribute no_rw_check of dy : signal is true;
    signal s_dp  : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_dr  : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_dr4 : unsigned(3 downto 0) := to_unsigned(14, 4);
    signal s_qa, s_fa : std_logic_vector(31 downto 0) := (others => '0');
    signal s_qy, s_fy : std_logic_vector(15 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(820 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(48 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(0, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(600 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_blur, s_sgn, s_max, s_bars, s_osc : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field LFO phases
    signal z_sw  : unsigned(15 downto 0) := (others => '0');            -- Navigator sway, 50 s
    signal z_tl  : unsigned(10 downto 0) := (others => '0');            -- Keychain threshold LFO, 9 s
    signal z_tsp : unsigned(11 downto 0) := (others => '0');            -- Contour spread lean, 23 s
    signal z_phv : unsigned(19 downto 0) := (others => '0');            -- DWO3 V-range sine roll, 7 s

    -- per-field values
    signal z_ssw, z_c, z_s : unsigned(15 downto 0) := (others => '0');
    signal z_sws, z_cs, z_ss : std_logic := '0';
    signal z_th  : unsigned(11 downto 0) := (others => '0');
    signal z_km, z_sp : signed(10 downto 0) := (others => '0');
    signal z_sp0 : unsigned(6 downto 0) := (others => '0');
    type t_u9x3 is array (0 to 2) of unsigned(8 downto 0);
    signal c_t12 : signed(12 downto 0) := to_signed(1968, 13);
    signal c_gm  : unsigned(3 downto 0) := (others => '0');
    signal c_sh5 : std_logic := '1';
    type t_u3x3 is array (0 to 2) of unsigned(2 downto 0);
    type t_u4x3 is array (0 to 2) of unsigned(3 downto 0);
    type t_u2x3 is array (0 to 2) of unsigned(1 downto 0);
    signal c_ix  : t_u3x3 := (others => (others => '0'));
    signal c_f   : t_u4x3 := (others => (others => '0'));
    signal c_ar  : t_u2x3 := (others => (others => '0'));
    signal c_fr  : t_u3x3 := (others => (others => '0'));
    type t_u11x3 is array (0 to 2) of unsigned(10 downto 0);
    type t_u8x3 is array (0 to 2) of unsigned(7 downto 0);
    signal c_dk  : t_u8x3 := (others => (others => '0'));
    signal c_du, c_dv, c_lu : signed(14 downto 0) := (others => '0');
    signal c_lv  : signed(15 downto 0) := (others => '0');
    signal c_fb  : signed(17 downto 0) := (others => '0');
    signal c_coff : unsigned(10 downto 0) := (others => '0');

    -- field serial multiply: acc = +- a x b (mod 2^32)
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : signed(24 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_ng  : std_logic := '0';
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pp  : std_logic_vector(2 downto 0) := (others => '0');
    signal z_ld  : std_logic := '0';
    signal r_ka  : unsigned(7 downto 0) := (others => '0');
    signal r_kq  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_k5q : std_logic_vector(15 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine: the rotated ramps' line starts; the DWO3 frequency of the next line
    signal e_ul, e_vl : unsigned(20 downto 0) := (others => '0');
    signal e_fl, e_fnext : signed(24 downto 0) := (others => '0');
    signal e_phv : unsigned(19 downto 0) := (others => '0');
    signal e_cnt : unsigned(3 downto 0) := (others => '0');
    signal e_tm  : unsigned(20 downto 0) := (others => '0');
    signal e_sg  : std_logic := '0';

    -- beam (stage 1)
    signal b_u, b_v : unsigned(20 downto 0) := (others => '0');
    signal b_ph  : unsigned(19 downto 0) := (others => '0');
    signal b_f   : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    signal p2_ta, p2_tb : unsigned(11 downto 0) := (others => '0');
    signal p2_ck : signed(12 downto 0) := (others => '0');
    signal p5_cf, p6_cf : signed(11 downto 0) := (others => '0');
    signal p3_s  : unsigned(12 downto 0) := (others => '0');
    signal p3_te : signed(14 downto 0) := (others => '0');
    signal p3_m2 : std_logic := '0';
    signal p4_d  : signed(15 downto 0) := (others => '0');
    signal p5_key : unsigned(9 downto 0) := (others => '0');
    signal p6_x  : signed(13 downto 0) := (others => '0');
    signal c_x0  : signed(13 downto 0) := (others => '0');
    signal p7_xp, p8_xp : signed(13 downto 0) := (others => '0');
    signal p8_t  : signed(13 downto 0) := (others => '0');
    signal p9_xg : signed(14 downto 0) := (others => '0');
    signal p10_yb, p11_yb, p12_yb, p13_yb : signed(13 downto 0) := (others => '0');
    signal s_k3  : signed(16 downto 0) := (others => '0');
    signal s_k6  : signed(19 downto 0) := (others => '0');
    signal s_k9  : signed(22 downto 0) := (others => '0');
    signal s_y0  : signed(13 downto 0) := (others => '0');
    type t_s13 is array (0 to 2) of signed(12 downto 0);
    type t_s14 is array (0 to 2) of signed(13 downto 0);
    type t_s18 is array (0 to 2) of signed(17 downto 0);
    signal p11_a, p11_b, p12_a, p13_a : t_s13 := (others => (others => '0'));
    signal p12_d : t_s14 := (others => (others => '0'));
    signal p13_p : t_s18 := (others => (others => '0'));
    signal p14_x : t_s14 := (others => (others => '0'));
    type t_u10x3 is array (0 to 2) of unsigned(9 downto 0);
    signal p15_v : t_u10x3 := (others => (others => '0'));
    type t_kq is array (6 to 15) of unsigned(1 downto 0);
    signal p_kq  : t_kq := (others => (others => '0'));
    signal p16_m, p17_a, p17_b, p18_a, p19_a : t_u9x3 := (others => (others => '0'));
    signal p16_m1p, p16_m2p, p16_m2pp : unsigned(8 downto 0) := (others => '0');
    type t_d10x3 is array (0 to 2) of signed(9 downto 0);
    signal p18_d : t_d10x3 := (others => (others => '0'));
    type t_s13x3 is array (0 to 2) of signed(12 downto 0);
    signal p19_p : t_s13x3 := (others => (others => '0'));
    signal p20_r, p20_g, p20_b : unsigned(7 downto 0) := (others => '0');
    signal p21_a, p21_b : unsigned(11 downto 0) := (others => '0');
    signal p21_r, p21_bb, p22_r, p22_bb : unsigned(7 downto 0) := (others => '0');
    signal p22_yp : unsigned(7 downto 0) := (others => '0');
    signal p23_ya : unsigned(9 downto 0) := (others => '0');
    signal p23_yb : signed(7 downto 0) := (others => '0');
    signal p23_dr, p23_db : signed(8 downto 0) := (others => '0');
    signal p24_y, p24_u, p24_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_ly1, s_ly2 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KU : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KV : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel
    -- the syncs' C_LAT delay as a block-RAM ring: written at sr_wp, read (registered) C_LAT - 1 entries back
    type t_sr is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr_ram : t_sr := (others => "1111");
    attribute no_rw_check of sr_ram : signal is true;
    signal sr_wp : unsigned(7 downto 0) := to_unsigned(0, 8);
    signal sr_rp : unsigned(7 downto 0) := to_unsigned(256 - (C_LAT - 1), 8);
    signal sr_q  : std_logic_vector(3 downto 0) := "1111";

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;


    -- sine: s_ang -> (A) address / quadrant -> (B) ROM -> (C) magnitude rs_m and sign rs_s (ready 4 clocks after s_ang)
    p_sin : process(clk)
        variable v_i : unsigned(9 downto 0);
        variable v_sq : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            v_i := s_ang(9 downto 0);
            if s_ang(10) = '1' then
                r_s_a <= (not v_i) + 1;
                if v_i = 0 then r_sp1 <= '1'; else r_sp1 <= '0'; end if;
            else
                r_s_a <= v_i; r_sp1 <= '0';
            end if;
            r_q1 <= s_ang(11);
            r_s_q <= C_OS_SIN(to_integer(r_s_a));
            r_q2 <= r_q1; r_sp2 <= r_sp1;
            if r_sp2 = '1' then v_sq := to_unsigned(32767, 16); else v_sq := r_s_q; end if;
            rs_m <= v_sq; rs_s <= r_q2;
        end if;
    end process p_sin;

    p_sr : process(clk)
    begin
        if rising_edge(clk) then
            sr_q <= sr_ram(to_integer(sr_rp));
            sr_ram(to_integer(sr_wp)) <= '1' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            sr_wp <= sr_wp + 1; sr_rp <= sr_rp + 1;
        end if;
    end process p_sr;

    p_kc : process(clk)
    begin
        if rising_edge(clk) then
            r_kq  <= C_OS_KC(to_integer(r_ka));
            r_k5q <= C_OS_K5(to_integer(s_g5(12 downto 5)));
        end if;
    end process p_kc;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_g1, v_g3, v_g4, v_g5, v_gp : unsigned(9 downto 0);
        variable v_tl : unsigned(9 downto 0);
        variable v_ts : unsigned(9 downto 0);
        variable v_t  : signed(14 downto 0);
        variable v_kk : signed(11 downto 0);
        variable v_tm : unsigned(15 downto 0);
        variable v_ts1 : std_logic;
        variable v_d7 : unsigned(6 downto 0);
        variable v_t10 : unsigned(9 downto 0);
        variable v_sp7 : unsigned(6 downto 0);
        variable v_g45 : unsigned(12 downto 0);
        variable v_dgx : unsigned(11 downto 0);
        variable v_kc : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xsr <= f_xs(s_tid);

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
            elsif s_zc /= 4095 then
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
                s_blur <= registers_in(6)(0);
                s_sgn  <= registers_in(6)(1);
                s_max  <= registers_in(6)(2);
                s_bars <= registers_in(6)(3);
                s_osc  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                z_sw   <= z_sw + 22;
                z_tl   <= z_tl + 4;
                z_tsp  <= z_tsp + 3;
                z_phv  <= z_phv + 2497;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);
            if s_xs = 2 then c_sh5 <= '1'; else c_sh5 <= '0'; end if;

            -- shared glide (1/8 of the gap per field, +-2 deadband; +-4 on the camera channel): four clocks
            -- per channel, run from the zone start by its own phase / channel counters
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 3      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_GLIDE_BYPASS then
                            null;
                        elsif s_gd > 32 or s_gd < -32 or (z_gc /= 3 and (s_gd > 16 or s_gd < -16)) then
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
                            when 3      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial multiply slots: slot 0 is decoded at zc 40, slot k >= 1 at 70 + 24 (k - 1); the constant
            -- operand comes from a ROM (slot x 16 + timing id), the trig operand and the sign are muxed three clocks
            -- after the decode, loaded the clock after, 16 iterations, stored at decode + 22.  Trig operands go in as
            -- |x| >> 1 (Q14) with the sign on the product.
            z_pp(0) <= '0';
            if s_zone = '1' then
                if s_zc = 40 then
                    z_sel <= to_unsigned(0, 4); z_pp(0) <= '1';
                elsif s_zc >= 70 and s_zc <= 358 then
                    case to_integer(s_zc) is
                        when 70  => z_sel <= to_unsigned(1, 4);  z_pp(0) <= '1';
                        when 94  => z_sel <= to_unsigned(2, 4);  z_pp(0) <= '1';
                        when 118 => z_sel <= to_unsigned(3, 4);  z_pp(0) <= '1';
                        when 142 => z_sel <= to_unsigned(4, 4);  z_pp(0) <= '1';
                        when 166 => z_sel <= to_unsigned(5, 4);  z_pp(0) <= '1';
                        when 190 => z_sel <= to_unsigned(6, 4);  z_pp(0) <= '1';
                        when 214 => z_sel <= to_unsigned(7, 4);  z_pp(0) <= '1';
                        when 238 => z_sel <= to_unsigned(8, 4);  z_pp(0) <= '1';
                        when 262 => z_sel <= to_unsigned(9, 4);  z_pp(0) <= '1';
                        when 286 => z_sel <= to_unsigned(10, 4); z_pp(0) <= '1';
                        when 310 => z_sel <= to_unsigned(11, 4); z_pp(0) <= '1';
                        when 334 => z_sel <= to_unsigned(12, 4); z_pp(0) <= '1';
                        when others => null;
                    end case;
                end if;
            end if;
            z_pp(2 downto 1) <= z_pp(1 downto 0);
            r_ka <= z_sel & unsigned(s_tid);
            z_ld <= z_pp(2);
            if z_pp(2) = '1' then
                case to_integer(z_sel) is
                    when 0      => v_tm := z_ssw; v_ts1 := z_sws;
                    when 1 | 4 | 5 | 8 | 9 => v_tm := z_c; v_ts1 := z_cs;
                    when others => v_tm := z_s; v_ts1 := z_ss;
                end case;
                z_fb <= '0' & v_tm(15 downto 1);
                if z_sel = 2 then z_ng <= not v_ts1; else z_ng <= v_ts1; end if;
                z_fa <= signed(r_kq(24 downto 0));
                if z_sel = 12 then z_fb <= resize(s_g6(12 downto 7), 16); z_ng <= '0'; end if;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                if z_ng = '1' then f_a <= unsigned(-resize(z_fa, 32)); else f_a <= unsigned(resize(z_fa, 32)); end if;
                f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- sine reads in the zone (rs_m / rs_s are ready 4 clocks after s_ang is set)
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 => s_ang <= z_sw(15 downto 4);
                    when 63 => s_ang <= z_th + 1024;
                    when 64 => s_ang <= z_th;
                    when others => null;
                end case;
            end if;
            if e_cnt = 1 then s_ang <= e_phv(19 downto 8); end if;

            v_g1 := s_g1(12 downto 3); v_g3 := unsigned(registers_in(2)); v_g4 := unsigned(registers_in(3));
            v_g5 := s_g5(12 downto 3); v_gp := s_gp(12 downto 3);
            v_tl := f_tri10a(z_tl);
            v_ts := f_tri10b(z_tsp);
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 34 =>
                        v_t10 := resize(v_g1(9 downto 2), 10) + resize(v_g1(9 downto 3), 10) + resize(v_tl(9 downto 4), 10)
                                 + resize(v_tl(9 downto 7), 10) + 46;
                        if s_osc = '1' then c_t12 <= signed(resize(v_t10 & "00", 13)) - 1536;
                        else c_t12 <= signed(resize(v_t10 & "00", 13)); end if;
                        z_km  <= signed(resize(v_gp(9 downto 3), 11)) + signed(resize(v_gp(9 downto 6), 11))
                                 - signed(resize(v_gp(9 downto 9), 11)) + to_signed(f_km0(s_tid), 11);
                        v_sp7 := resize(v_g3(9 downto 5), 7) + resize(v_g3(9 downto 7), 7) + resize(v_g3(9 downto 9), 7)
                                 + resize(v_ts(9 downto 8), 7);
                        z_sp0 <= v_sp7;
                        v_g45 := shift_left(resize(v_g4, 13), 2) + resize(v_g4, 13);
                        v_d7 := v_g45(12 downto 6);               -- Curtain Mix: the dry key, short channel loudest
                        c_dk(0) <= resize(v_d7, 8) + resize(v_d7(6 downto 1), 8);
                        c_dk(1) <= resize(v_d7, 8);
                        c_dk(2) <= resize(v_d7(6 downto 1), 8);
                        v_dgx := shift_left(resize(v_gp(8 downto 0), 12), 2) + resize(v_gp(8 downto 0), 12);
                        if s_blur = '1' then c_gm <= to_unsigned(8, 4);
                        elsif v_gp(9) = '1' then c_gm <= resize(v_dgx(11 downto 9), 4);
                        else c_gm <= (others => '0'); end if;
                    when 35 =>
                        if z_sp0 > 2 then z_sp <= signed(resize(z_sp0, 11)) - 2; else z_sp <= (others => '0'); end if;
                    when 36 =>
                        for r in 0 to 2 loop                      -- SMX3 path from the K5 ROM
                            c_ar(r) <= unsigned(r_k5q(5 * r + 4 downto 5 * r + 3));
                            c_fr(r) <= unsigned(r_k5q(5 * r + 2 downto 5 * r));
                        end loop;
                    when 37 =>
                        for r in 0 to 2 loop
                            if r = 0 then v_kk := resize(z_km, 12) - resize(z_sp, 12);
                            elsif r = 1 then v_kk := resize(z_km, 12);
                            else v_kk := resize(z_km, 12) + resize(z_sp, 12); end if;
                            if v_kk < 0 then v_kc := (others => '0');
                            elsif v_kk > 191 then v_kc := to_unsigned(191, 9);
                            else v_kc := unsigned(v_kk(8 downto 0)); end if;
                            c_ix(r) <= v_kc(8 downto 6);
                            c_f(r)  <= v_kc(5 downto 2);
                        end loop;
                    -- slot results
                    when 62 =>
                        v_t := signed(f_acc(28 downto 14));
                        z_th <= unsigned(signed(shift_left(resize(s_g2(12 downto 3), 12), 2)) + resize(v_t, 12));
                    when 92  => c_du <= signed(f_acc(31 downto 17));
                    when 116 => c_dv <= signed(f_acc(31 downto 17));
                    when 140 => c_lu <= signed(f_acc(31 downto 17));
                    when 164 => c_lv <= signed(f_acc(31 downto 16));
                    when 332 => c_fb <= signed(f_acc(31 downto 14));
                    when 356 => c_coff <= f_acc(12 downto 2);
                    when others => null;
                end case;
                -- the sine reads
                if s_zc = 34 then z_ssw <= rs_m; z_sws <= rs_s; end if;
                if s_zc = 67 then z_c <= rs_m; z_cs <= rs_s; end if;
                if s_zc = 68 then z_s <= rs_m; z_ss <= rs_s; end if;
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
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- ((Y >> 2) - 16), averaged over 8-pixel cells, each cell slewed down the
    -- screen (one pole, 1/16 per line; the state lives in the cell store), then
    -- slewed across by two one-pole filters (1/8 per pixel) and scaled by K6
    -- (the Osc made bipolar about its mean).  All three filters decay, so none
    -- needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(12 downto 0);
        variable v_p : unsigned(13 downto 0);
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

            -- 1: sample
            if s_osc = '1' then
                s_lin <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lin <= v_l - 16; else s_lin <= (others => '0'); end if;
            end if;
            s_lav   <= data_in.avid;
            s_lc    <= s_wcol(2 downto 0);
            s_lcell <= s_wcol(10 downto 3);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= s_wcol(10 downto 3) + 1;
            else
                s_wcol  <= (others => '0');
                s_lb_ra <= (others => '0');
            end if;
            -- 2: sum eight; at the eighth, take the cell's state (read a cell ahead) for the update
            s_f1 <= '0';
            if s_lav = '0' then
                s_cur <= unsigned(s_lb_q(11 downto 0));
            else
                if s_lc = 0 then s_acc <= resize(s_lin, 11); else s_acc <= s_acc + s_lin; end if;
                if s_lc = 7 then
                    s_sum8  <= s_acc + s_lin;
                    s_old   <= s_cur;
                    s_cur   <= unsigned(s_lb_q(11 downto 0));
                    s_fcell <= s_lcell;
                    s_f1    <= '1';
                end if;
            end if;
            -- 3: state += (cell average * 16 - state) / 16
            v_d := signed(resize(s_sum8 & '0', 13)) - signed(resize(s_old, 13));
            v_d := signed(resize(s_old, 13)) + shift_right(v_d, 4);
            s_new   <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);

            -- pixel side: the cell stream (the line above's state), two slews across, x K6
            s_lv <= unsigned(s_lb_q(11 downto 0));
            v_e  := signed(resize(s_lv, 13)) - signed(resize(s_h2, 13));
            v_e  := signed(resize(s_h2, 13)) + shift_right(v_e, 3);
            s_h2 <= unsigned(v_e(11 downto 0));
            s_la  <= s_h2(7 downto 4) * s_g6(12 downto 7);
            s_lbh <= s_h2(11 downto 8) * s_g6(12 downto 7);
            v_p := resize(s_la, 14) + shift_left(resize(s_lbh, 14), 4);   -- luma x K6
            if s_osc = '1' then                -- the Osc bipolar about its mean, in offset binary (+2048)
                s_caml <= v_p(13 downto 2) - resize(c_coff, 12) + 2048;
            else
                s_caml <= v_p(13 downto 2);
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: the rotated ramps' values at each line start (loaded in the
    -- zone, advanced at each line start) and the DWO3's frequency for the next
    -- line: fA + j fB + depth x sin(V phase), the sine from the ROM, the depth by
    -- shifts (in 10 clocks after the line start)
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 64 then          -- the field's start values, summed in place below
                e_ul  <= to_unsigned(1677721, 21);
                e_vl  <= to_unsigned(1468006, 21);
                e_fl  <= to_signed(f_f0q(s_tid), 25);
                e_phv <= z_phv;
            elsif s_zone = '1' and (s_zc = 188 or s_zc = 212) then
                e_ul  <= e_ul + f_acc(22 downto 2);
            elsif s_zone = '1' and (s_zc = 236 or s_zc = 260) then
                e_vl  <= e_vl + f_acc(22 downto 2);
            elsif s_zone = '1' and (s_zc = 284 or s_zc = 308) then
                e_fl  <= e_fl + resize(signed(f_acc(31 downto 10)), 25);
            elsif g_start = '1' then
                e_ul  <= e_ul + unsigned(resize(c_lu, 21));
                e_vl  <= e_vl + unsigned(resize(c_lv, 21));
                e_fl  <= e_fl + resize(c_fb, 25);
                e_phv <= e_phv + to_unsigned(f_dv(s_tid), 20);
            end if;
            if (s_zone = '1' and s_zc = 470) or g_start = '1' then
                e_cnt <= to_unsigned(1, 4);
            elsif e_cnt /= 0 then
                e_cnt <= e_cnt + 1;
            end if;
            if e_cnt = 6 then
                e_tm <= shift_left(resize(rs_m, 21), 3) - resize(shift_right(rs_m, 2), 21) + resize(shift_right(rs_m, 6), 21);
                e_sg <= rs_s;
            end if;
            if e_cnt = 7 then
                case to_integer(s_xs) is
                    when 3      => e_tm <= e_tm + shift_right(e_tm, 1);
                    when 4      => e_tm <= shift_left(e_tm, 1);
                    when others => null;
                end case;
            end if;
            if e_cnt = 8 then
                if e_sg = '1' then e_fnext <= e_fl - signed(resize(e_tm, 25));
                else e_fnext <= e_fl + signed(resize(e_tm, 25)); end if;
                e_cnt <= (others => '0');
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the Navigator-rotated ramps u, v (2^24 per fold cycle) and the
    -- DWO3 phase (hard reset at each line start; f += a per pixel = FM by the
    -- rotated H ramp)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_u <= e_ul; b_v <= e_vl;
                b_ph <= (others => '0'); b_f <= unsigned(e_fnext(23 downto 8));
            elsif g_avid = '1' then
                b_u  <= b_u + unsigned(resize(c_du, 21));
                b_v  <= b_v + unsigned(resize(c_dv, 21));
                b_ph <= b_ph + resize(b_f, 20);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Filter bank: one-pole low-passes 2^-k (k = 3, 6, 9) of the stage-10
    -- signal, held at zero off the generated avid (the per-line reset and resync)
    ---------------------------------------------------------------------------
    p_bank : process(clk)
    begin
        if rising_edge(clk) then
            if s_gav_sr(8) = '1' then
                s_k3 <= s_k3 - shift_right(s_k3, 3) + resize(p10_yb, 17);
                s_k6 <= s_k6 - shift_right(s_k6, 6) + resize(p10_yb, 20);
                s_k9 <= s_k9 - shift_right(s_k9, 9) + resize(p10_yb, 23);
                s_y0 <= p10_yb;
            else
                s_k3 <= (others => '0'); s_k6 <= (others => '0'); s_k9 <= (others => '0'); s_y0 <= (others => '0');
            end if;
        end if;
    end process p_bank;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel; stage s computes from
    -- the stage s-1 pixel, whose generated avid is s_gav_sr(s - 3))
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s   : unsigned(12 downto 0);
        variable v_k   : signed(15 downto 0);
        variable v_a, v_b : signed(12 downto 0);
        variable v_d   : signed(17 downto 0);
        variable v_e   : signed(12 downto 0);
        variable v_e2  : signed(12 downto 0);
        variable v_v   : signed(15 downto 0);
        variable v_x   : signed(13 downto 0);
        variable v_r   : signed(14 downto 0);
        variable v_src : t_u9x3;
        variable v_dk  : unsigned(7 downto 0);
        variable v_mm  : unsigned(10 downto 0);
        variable v_f   : signed(12 downto 0);
        variable v_row : unsigned(11 downto 0);
        variable v_rc  : unsigned(11 downto 0);
        variable v_y   : unsigned(11 downto 0);
        variable v_uv  : signed(11 downto 0);
        variable v_ta  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: folds (DSG3 H and V, or the DWO3 triangle); the camera's Keychain-CV term
            if s_bars = '1' then p2_ta <= f_tri12(b_ph & "0000"); else p2_ta <= f_tri12(b_u & "000"); end if;
            p2_tb <= f_tri12(b_v & "000");
            if s_osc = '1' then                -- 3/4 of the offset-binary Osc (its 1536 is folded into c_t12)
                p2_ck <= signed(resize(shift_right(s_caml, 1), 13) + resize(shift_right(s_caml, 2), 13));
            else
                p2_ck <= -signed(resize(shift_right(s_caml, 1), 13));
            end if;
            -- 3: DSG3 Average (the sum) or Maximum (twice the larger: S9) ; the bare DWO3 triangle (S10, S9 off) ;
            --    the Keychain threshold (+ the CV: the Osc grows the figures, the camera shrinks them)
            if p2_ta > p2_tb then v_ta := p2_ta; else v_ta := p2_tb; end if;
            if s_max = '1' then p3_s <= v_ta & '0';
            elsif s_bars = '1' then p3_s <= resize(p2_ta, 13);
            else p3_s <= resize(p2_ta, 13) + resize(p2_tb, 13); end if;
            p3_m2 <= s_max or not s_bars;
            p3_te <= resize(c_t12, 15) + resize(p2_ck, 15);
            -- 4: comparator input
            if p3_m2 = '0' then p4_d <= resize(p3_te, 16) - signed(resize(p3_s, 16));
            else p4_d <= shift_left(resize(p3_te, 16), 1) - signed(resize(p3_s, 16)); end if;
            -- 5: the key, with a 1-px coverage edge (gain 2^5 at 1080, 2^4 at 720p / SD)
            if c_sh5 = '1' then
                if p4_d >= 16 then p5_key <= to_unsigned(1023, 10);
                elsif p4_d < -16 then p5_key <= (others => '0');
                else v_k := p4_d + 16; p5_key <= unsigned(v_k(4 downto 0)) & "00000"; end if;
            else
                if p4_d >= 32 then p5_key <= to_unsigned(1023, 10);
                elsif p4_d < -32 then p5_key <= (others => '0');
                else v_k := p4_d + 32; p5_key <= unsigned(v_k(5 downto 0)) & "0000"; end if;
            end if;
            if s_osc = '1' then                               -- the camera into the filter input (Osc: 1/4; its
                p5_cf <= signed(resize(shift_right(s_caml, 4), 12) + resize(shift_right(s_caml, 5), 12));   -- offset cancels)
            else
                p5_cf <= signed(resize(shift_right(s_caml, 2), 12) + resize(shift_right(s_caml, 3), 12));
            end if;
            -- 6: key + camera = the filter input
            p6_x <= signed(resize(p5_key, 14)) + resize(p5_cf, 14);
            p6_cf <= p5_cf;
            v_s := resize(p5_key, 13) + 256;                  -- the key's level for Curtain Mix: 0, 1/2, 1
            if v_s(10) = '1' then p_kq(6) <= "10"; else p_kq(6) <= '0' & v_s(9); end if;
            for i in 7 to 15 loop p_kq(i) <= p_kq(i - 1); end loop;
            -- 7: minus the camera's first sample on the line (the integrators' pre-charge: no streak from the
            --    left border on a camera picture; the key starts from blanking, as Contour sees it)
            if s_gav_sr(4) = '1' and s_gav_sr(5) = '0' then
                c_x0 <= resize(p6_cf, 14); p7_xp <= p6_x - resize(p6_cf, 14);
            elsif s_gav_sr(4) = '1' then
                p7_xp <= p6_x - c_x0;
            else
                p7_xp <= (others => '0');
            end if;
            -- 8, 9: x (2 - 2^(gm - 5)), the gain ahead of the filters (2.0 .. 1.5 over P12's top half; Blur: x 1)
            case to_integer(c_gm) is
                when 0      => p8_t <= (others => '0');
                when 1      => p8_t <= shift_right(p7_xp, 4);
                when 2      => p8_t <= shift_right(p7_xp, 3);
                when 3      => p8_t <= shift_right(p7_xp, 2);
                when 4      => p8_t <= shift_right(p7_xp, 1);
                when others => p8_t <= p7_xp;
            end case;
            p8_xp <= p7_xp;
            p9_xg <= shift_left(resize(p8_xp, 15), 1) - resize(p8_t, 15);
            -- 10: to the bank's scale (the bank integrates it at the next clock)
            if s_gav_sr(7) = '1' then p10_yb <= resize(shift_right(p9_xg, 2), 14); else p10_yb <= (others => '0'); end if;
            -- 11: each channel's two adjacent bank members (the low-pass before this pixel)
            for c in 0 to 2 loop
                case to_integer(c_ix(c)) is
                    when 0 => v_a := s_y0(13 downto 1);       v_b := s_k3(16 downto 4);
                    when 1 => v_a := s_k3(16 downto 4);       v_b := s_k6(19 downto 7);
                    when others => v_a := s_k6(19 downto 7);  v_b := s_k9(22 downto 10);
                end case;
                p11_a(c) <= v_a; p11_b(c) <= v_b;
            end loop;
            p11_yb <= p10_yb;
            -- 12, 13: crossfade (b - a) f / 16
            for c in 0 to 2 loop
                p12_d(c) <= resize(p11_b(c), 14) - resize(p11_a(c), 14);
                p12_a(c) <= p11_a(c);
                v_d := (others => '0');
                if c_f(c)(0) = '1' then v_d := v_d + resize(p12_d(c), 18); end if;
                if c_f(c)(1) = '1' then v_d := v_d + shift_left(resize(p12_d(c), 18), 1); end if;
                if c_f(c)(2) = '1' then v_d := v_d + shift_left(resize(p12_d(c), 18), 2); end if;
                if c_f(c)(3) = '1' then v_d := v_d + shift_left(resize(p12_d(c), 18), 3); end if;
                p13_p(c) <= v_d;
                p13_a(c) <= p12_a(c);
            end loop;
            p12_yb <= p11_yb; p13_yb <= p12_yb;
            -- 14: the filter output: high-pass (Contour) = y - lp, or low-pass + the camera's first sample
            --     (Detail Blur)
            for c in 0 to 2 loop
                v_e := resize(shift_right(p13_p(c), 4), 13);
                if s_blur = '0' then
                    p14_x(c) <= resize(p13_yb(13 downto 1), 14) - resize(p13_a(c), 14) - resize(v_e, 14);
                elsif s_gav_sr(11) = '1' then
                    p14_x(c) <= resize(p13_a(c), 14) + resize(v_e, 14) + resize(shift_right(c_x0, 3), 14);
                else
                    p14_x(c) <= resize(p13_a(c), 14) + resize(v_e, 14);
                end if;
            end loop;
            -- 15: Curtain rectifier on the filter output: Both = |x| ; Signed = pedestal + x / 2  (x in half units)
            for c in 0 to 2 loop
                v_x := p14_x(c);
                if s_sgn = '0' then v_v := shift_left(resize(abs(v_x), 16), 1);
                else v_v := resize(v_x, 16) + to_signed(C_PED(c), 16); end if;
                if v_v < 0 then p15_v(c) <= (others => '0');
                elsif v_v > 1023 then p15_v(c) <= to_unsigned(1023, 10);
                else p15_v(c) <= unsigned(v_v(9 downto 0)); end if;
            end loop;
            -- 16: Curtain Mix (the dry key, K4, short channel loudest) and the gain (the longer channels louder:
            --     1, 1.25, 1.5), clamped at 511; then the path delay (the longer channels pass more modules: 1, 2 px)
            for c in 0 to 2 loop
                if s_gav_sr(13) = '0' then v_dk := (others => '0');
                else
                    case to_integer(p_kq(15)) is
                        when 0      => v_dk := (others => '0');
                        when 1      => v_dk := shift_right(c_dk(c), 1);
                        when others => v_dk := c_dk(c);
                    end case;
                end if;
                if c = 0 then v_mm := resize(p15_v(c), 11) + resize(v_dk, 11);
                elsif c = 1 then v_mm := resize(p15_v(c), 11) + resize(shift_right(p15_v(c), 2), 11) + resize(v_dk, 11);
                else v_mm := resize(p15_v(c), 11) + resize(shift_right(p15_v(c), 1), 11) + resize(v_dk, 11); end if;
                if v_mm > 511 then p16_m(c) <= to_unsigned(511, 9); else p16_m(c) <= v_mm(8 downto 0); end if;
            end loop;
            p16_m1p <= p16_m(1); p16_m2p <= p16_m(2); p16_m2pp <= p16_m2p;
            -- 17 .. 20: SMX3 rows: R, G, B each crossfade two neighbouring sources ; the encoder soft clip
            v_src(0) := p16_m(0); v_src(1) := p16_m1p; v_src(2) := p16_m2pp;
            for r in 0 to 2 loop
                case to_integer(c_ar(r)) is
                    when 0 => p17_a(r) <= v_src(0); p17_b(r) <= v_src(1);
                    when 1 => p17_a(r) <= v_src(1); p17_b(r) <= v_src(2);
                    when 2 => p17_a(r) <= v_src(2); p17_b(r) <= (others => '0');
                    when others => p17_a(r) <= (others => '0'); p17_b(r) <= v_src(0);
                end case;
                p18_d(r) <= signed(resize(p17_b(r), 10)) - signed(resize(p17_a(r), 10));
                p18_a(r) <= p17_a(r);
                v_f := (others => '0');
                if c_fr(r)(0) = '1' then v_f := v_f + resize(p18_d(r), 13); end if;
                if c_fr(r)(1) = '1' then v_f := v_f + shift_left(resize(p18_d(r), 13), 1); end if;
                if c_fr(r)(2) = '1' then v_f := v_f + shift_left(resize(p18_d(r), 13), 2); end if;
                p19_p(r) <= v_f;
                p19_a(r) <= p18_a(r);
                v_r := signed(resize(p19_a(r), 15)) + resize(shift_right(p19_p(r), 3), 15);     -- 0 .. 511
                v_row := unsigned(v_r(11 downto 0));
                if v_row > 192 then v_rc := 192 + shift_right(v_row - 192, 2); else v_rc := v_row; end if;
                if v_rc > 255 then v_rc := to_unsigned(255, 12); end if;
                if r = 0 then p20_r <= v_rc(7 downto 0); elsif r = 1 then p20_g <= v_rc(7 downto 0); else p20_b <= v_rc(7 downto 0); end if;
            end loop;
            -- 21, 22: Y' = (5 R + 9 G + 2 B) / 16
            p21_a <= shift_left(resize(p20_r, 12), 2) + resize(p20_r, 12) + shift_left(resize(p20_b, 12), 1);
            p21_b <= shift_left(resize(p20_g, 12), 3) + resize(p20_g, 12);
            p21_r <= p20_r; p21_bb <= p20_b;
            v_y := p21_a + p21_b;
            p22_yp <= v_y(11 downto 4); p22_r <= p21_r; p22_bb <= p21_bb;
            -- 23, 24: Y 64 + 3.4375 Y' ; U (Cr) 512 + 2.5 (R - Y') ; V (Cb) 512 + 1.984 (B - Y')
            p23_ya <= resize(p22_yp, 10) + shift_left(resize(p22_yp, 10), 1) + 64;
            p23_yb <= signed(resize(shift_right(p22_yp, 1), 8)) - signed(resize(shift_right(p22_yp, 4), 8));
            p23_dr <= signed(resize(p22_r, 9)) - signed(resize(p22_yp, 9));
            p23_db <= signed(resize(p22_bb, 9)) - signed(resize(p22_yp, 9));
            p24_y  <= unsigned(signed(p23_ya) + resize(p23_yb, 10));
            v_uv := shift_left(resize(p23_dr, 12), 1) + resize(shift_right(p23_dr, 1), 12) + 512;
            p24_u <= unsigned(v_uv(9 downto 0));     -- 64 .. 960: a slice, not resize
            v_uv := shift_left(resize(p23_db, 12), 1) - resize(shift_right(p23_db, 6), 12) + 512;
            p24_v <= unsigned(v_uv(9 downto 0));
            -- 25: black off the generated avid
            if s_gav_sr(22) = '0' then
                s_y14 <= C_KY; s_u14 <= C_KU; s_v14 <= C_KV;
            else
                s_y14 <= p24_y; s_u14 <= p24_u; s_v14 <= p24_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(22);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output filter stores: the chroma box's input 8 pixels back, the luma 4 back
    ---------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_dp <= s_dp + 1; s_dr <= s_dr + 1; s_dr4 <= s_dr4 + 1;
            s_qa <= da(to_integer(s_dr));
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y14) & std_logic_vector(s_u14) & std_logic_vector(s_v14);
            s_qy <= dy(to_integer(s_dr4));
            dy(to_integer(s_dp)) <= "000000" & std_logic_vector(s_y14);
            s_fa <= s_qa; s_fy <= s_qy;
        end if;
    end process p_dl;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through one [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums, with the per-line resync; the HD HDMI chroma fix), luma
    -- through [1 2 1] centred on the same pixel (kept sharp: the heads are
    -- hairlines), luma 13/16, and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ly : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the black (the off-avid input) once 8 blank samples are in, so a
            -- glitched sample can't offset the colours for good.  C+1 sums, C+2 output.
            if s_bah = "00000000" then
                s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bau <= s_bau + s_u14 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v14 - unsigned(s_fa(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 14) + s_baud; s_u16 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v16 <= v_sc(13 downto 4);
            -- luma [1 2 1] centred 4 px back (with the chroma box)
            s_ly1 <= unsigned(s_fy(9 downto 0)); s_ly2 <= s_ly1;
            v_ly := resize(unsigned(s_fy(9 downto 0)), 12) + shift_left(resize(s_ly1, 12), 1) + resize(s_ly2, 12);
            s_y16 <= v_ly(11 downto 2);
            -- C+3: luma 13/16
            if s_y16 > 64 then v_dl := s_y16 - 64; else v_dl := (others => '0'); end if;
            s_y19 <= s_y16 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u19 <= s_u16; s_v19 <= s_v16;
            -- C+4: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y19; s_u21 <= s_u19; s_v21 <= s_v19;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= sr_q(0);
    data_out.vsync_n <= sr_q(1);
    data_out.field_n <= sr_q(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture overshoot;
