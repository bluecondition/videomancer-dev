-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Decade v0.1.0 -- Vellum.  After Keuffel & Esser log-log graph paper, slide-rule scales and a semilog plot
-- of exponential growth: a sheet of log paper whose decades converge on an origin just off the lower-left
-- corner, every decade ruled at log10(n), with a plotted family of power laws over it.
--
--   per axis   s = |x - x0| (half virtual px),  L = A log2(s) - drift  (decades, Q18 mod 1),  A = P log10(2)
--              semi-log x: L = s / 1024 - drift
--   lines      at frac(L) = log10(n) (K4: decades / 1-2-5 / 1..9 / halves); class decade / five / fine
--   AA         pixel distance = |dL| / |dL/dpx| (x: finite difference per real pixel; y: per line), coverage
--              = clamp(width / 2 + 1/2 - distance) in 17 levels; where the lines crowd an axis the paper
--              fades to flat grey.  The x rulings and the curves divide by a shift and a 3-bit mantissa
--              multiply; the y rulings are one tone per line, computed by the line engine.
--   curves     phase = 2 Ly - 2k A log2(sx) + camera + drift share   (log-log: y = c x^k, semi-log: y = c b^x),
--              k from P12; AA from the phase gradient (previous pixel, line above from a phase store)
--   camera     F = low-passed luma x K6 is plotted as data: the curve family's phase + 2.5 F cycles (the
--              curves become the picture's contour lines); the paper stays exact
--
--   K1 X / K2 Y  the origin (default just off the lower-left corner; on screen the paper mirrors)
--   K3 Scale     decades per screen (P = 0.88 .. 4.4, exponential)          K4 Lines   graduation, 4 steps
--   K5 Drift     endless zoom toward / away from the origin, bipolar (Hypnos speed, 2^-12 .. 2^-5 decade/field)
--   K6 Camera    camera (and Osc) strength                                  S7 Glide   Off / On
--   S8 Paper     Log-log / Semi-log      S9 Tone  Grey / Ink      S10 Weight Bold / Fine      S11 Source Video / Osc
--   P12 Plot     the exponent of the plotted family: flat .. 45 degrees .. steep
--
-- Chassis from Kinetic riley ([A] timing / virtual coordinates / generated avid, [B] knobs + glide,
-- [C] per-field serial multiplier + line engine, [D] source field (dazzle's gain-table version with the Osc
-- line-0 fix), [F] luma output conditioning).  The integer model .vellum_work/rb_decade/imodel.py mirrors
-- every arithmetic step.  Stage S: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock);
-- pixel n is screen column c = n + 3; output column j is [1 2 1] over pixels j .. j + 2 and the generated
-- avid lands 4 clocks after the source avid, so column j shows at screen column j + 4 = the centre pixel's c.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.decade_pkg.all;
use work.all;

architecture decade of program_top is
    constant C_LAT : integer := 40;   -- = processing_delay_clks; the output register is stage C_LAT + 1

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps
    ---------------------------------------------------------------------------
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
    -- index of the leading one (0 for 0 or 1)
    function f_lod(v : unsigned) return unsigned is
        variable r : unsigned(3 downto 0) := (others => '0');
    begin
        for i in 0 to v'length - 1 loop
            if v(v'low + i) = '1' then r := to_unsigned(i, 4); end if;
        end loop;
        return r;
    end function;
    function f_abs18(v : unsigned(17 downto 0)) return unsigned is       -- |wrap18(v)|
    begin
        if v(17) = '1' then return (not v) + 1; else return v; end if;
    end function;
    -- nearest drawn line (index in halves) from the decade value v (log: 16 .. 160 = 1 .. 10, lin: tenths x 16)
    function f_idx(v : unsigned(7 downto 0); md : unsigned(1 downto 0); lin : std_logic) return unsigned is
        variable a : unsigned(8 downto 0);
    begin
        case to_integer(md) is
            when 0 =>
                if lin = '0' then
                    if v < 51 then return to_unsigned(2, 5); else return to_unsigned(20, 5); end if;
                else
                    if v < 80 then return to_unsigned(0, 5); else return to_unsigned(20, 5); end if;
                end if;
            when 1 =>
                if lin = '0' then
                    if v < 23 then return to_unsigned(2, 5);
                    elsif v < 51 then return to_unsigned(4, 5);
                    elsif v < 113 then return to_unsigned(10, 5);
                    else return to_unsigned(20, 5); end if;
                else
                    if v < 40 then return to_unsigned(0, 5);
                    elsif v < 120 then return to_unsigned(10, 5);
                    else return to_unsigned(20, 5); end if;
                end if;
            when 2 =>
                a := resize(v, 9) + 8;
                return a(7 downto 4) & '0';
            when others =>
                a := resize(v, 9) + 4;
                return a(7 downto 3);
        end case;
    end function;
    -- the AA divide by a shifted 3-bit mantissa: K2 = round(256 / (8.5 + m))
    function f_k2(m : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(m) is
            when 0 => return to_unsigned(30, 5);
            when 1 => return to_unsigned(27, 5);
            when 2 => return to_unsigned(24, 5);
            when 3 => return to_unsigned(22, 5);
            when 4 => return to_unsigned(20, 5);
            when 5 => return to_unsigned(19, 5);
            when 6 => return to_unsigned(18, 5);
            when others => return to_unsigned(17, 5);
        end case;
    end function;
    -- a x 2^(5 - E), 8 bits (exact for a < 2^(E + 3))
    function f_as(a : unsigned(18 downto 0); e : unsigned(3 downto 0)) return unsigned is
        variable w : unsigned(23 downto 0);
    begin
        w := shift_left(resize(a, 24), 5);
        w := shift_right(w, to_integer(e));
        return w(7 downto 0);
    end function;
    -- the 3 bits below the leading one of g (E = its index)
    function f_m3(g : unsigned(15 downto 0); e : unsigned(3 downto 0)) return unsigned is
        variable w : unsigned(18 downto 0);
    begin
        w := shift_left(resize(g, 19), 3);
        w := shift_right(w, to_integer(e));
        return w(2 downto 0);
    end function;

    -- the density-fade weight in quarters: v = g >> sh; v >= 32 -> 4; 16 <= v < 32 -> v(3 .. 2); else 0
    function f_fade(g : unsigned(14 downto 0); sh : unsigned(3 downto 0)) return unsigned is
        variable w : unsigned(15 downto 0);
    begin
        w := '0' & g;
        case to_integer(sh) is
            when 6 =>
                if w(15 downto 11) /= 0 then return to_unsigned(4, 3); elsif w(10) = '1' then return '0' & w(9 downto 8); end if;
            when 7 =>
                if w(15 downto 12) /= 0 then return to_unsigned(4, 3); elsif w(11) = '1' then return '0' & w(10 downto 9); end if;
            when 8 =>
                if w(15 downto 13) /= 0 then return to_unsigned(4, 3); elsif w(12) = '1' then return '0' & w(11 downto 10); end if;
            when 9 =>
                if w(15 downto 14) /= 0 then return to_unsigned(4, 3); elsif w(13) = '1' then return '0' & w(12 downto 11); end if;
            when 10 =>
                if w(15) = '1' then return to_unsigned(4, 3); elsif w(14) = '1' then return '0' & w(13 downto 12); end if;
            when others =>
                null;
        end case;
        return to_unsigned(0, 3);
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_yl   : unsigned(11 downto 0) := to_unsigned(256, 12);   -- field-latched y step
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_first : std_logic := '1';

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to 15) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k): pixels 0 .. W - 1 at stage k + 2
    -- hsync, vsync and the generated avid delayed C_LAT - 1 in a block-RAM ring (+ one register = C_LAT)
    type t_sy is array (0 to 255) of unsigned(3 downto 0);
    signal sy_ram : t_sy := (others => "0011");
    attribute no_rw_check of sy_ram : signal is true;
    signal sy_wp  : unsigned(7 downto 0) := (others => '0');
    signal sy_q, sy_f : unsigned(3 downto 0) := "0011";
    signal o_hs, o_vs, o_av : std_logic := '1';
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 26) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: holds (K1 K2 K3 K6 P12 K5), glides, switches
    -- knob state in a block-RAM register file: 0 .. 5 the holds, 8 .. 13 the glides (K1 K2 K3 K6 P12 K5)
    type t_kr is array (0 to 255) of unsigned(15 downto 0);
    signal kr_ram : t_kr := (8 => to_unsigned(512 * 16, 16), 9 => to_unsigned(512 * 16, 16),
                             10 => to_unsigned(689 * 16, 16), 11 => to_unsigned(1023 * 16, 16),
                             12 => to_unsigned(460 * 16, 16), 13 => to_unsigned((670 - 512) * 16, 16),
                             others => (others => '0'));
    attribute no_rw_check of kr_ram : signal is true;
    signal kr_ra, kr_wa : unsigned(3 downto 0) := (others => '0');
    signal kr_q, kr_f, kr_wd : unsigned(15 downto 0) := (others => '0');
    signal kr_we, kr_run : std_logic := '0';
    signal kr_ch : unsigned(2 downto 0) := (others => '0');
    signal kr_c  : unsigned(3 downto 0) := (others => '0');
    signal z_hv, z_hh : unsigned(9 downto 0) := (others => '0');
    signal t_x0, t_y0 : signed(18 downto 0) := to_signed(-4800, 19);
    signal z_gm, z_gd, z_gs : signed(15 downto 0) := (others => '0');
    signal z_rd, z_g6 : unsigned(15 downto 0) := (others => '0');
    signal s_glide, s_semi, s_ink, s_fine, s_osc : std_logic := '0';
    signal s_md : unsigned(1 downto 0) := "10";

    -- [C] ROM port (sequencer / fill / line engine)
    signal r_a : unsigned(8 downto 0) := (others => '0');
    signal r_q, r_f : unsigned(15 downto 0) := (others => '0');
    signal r_az : unsigned(8 downto 0) := (others => '0');

    -- [C] per-field serial multiplier (signed multiplicand, unsigned multiplier) and constants
    signal f_a, f_acc, f_a2, f_acc2 : unsigned(39 downto 0) := (others => '0');
    signal f_b : unsigned(19 downto 0) := (others => '0');

    signal c_vA  : unsigned(12 downto 0) := to_unsigned(4608, 13);
    signal c_A   : unsigned(18 downto 0) := (others => '0');
    signal c_e3  : unsigned(9 downto 0) := (others => '0');
    signal c_E3m : unsigned(17 downto 0) := (others => '0');
    signal c_kq  : unsigned(16 downto 0) := (others => '0');
    signal c_Ak  : unsigned(21 downto 0) := (others => '0');
    signal c_v   : signed(10 downto 0) := (others => '0');
    signal c_av  : unsigned(9 downto 0) := (others => '0');
    signal c_u   : unsigned(8 downto 0) := (others => '0');
    signal c_ls  : unsigned(8 downto 0) := (others => '0');
    signal c_zero, c_vpos : std_logic := '0';
    signal c_spd : unsigned(19 downto 0) := (others => '0');
    signal c_dst : signed(20 downto 0) := (others => '0');
    signal c_dlt, c_phc : unsigned(23 downto 0) := (others => '0');
    signal c_dx0 : signed(14 downto 0) := (others => '0');
    signal c_x0h, c_y0h : signed(14 downto 0) := (others => '0');
    signal c_G6 : unsigned(13 downto 0) := (others => '0');
    signal c_cwx0, c_cwx1, c_cwx2, c_cwy0, c_cwy1, c_cwy2, c_cwc : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal c_shx, c_shy : unsigned(3 downto 0) := to_unsigned(7, 4);
    signal c_iir : unsigned(2 downto 0) := to_unsigned(6, 3);
    signal c_xpre : unsigned(9 downto 0) := to_unsigned(5, 10);
    signal c_M, c_Mc : unsigned(9 downto 0) := to_unsigned(320, 10);

    -- [C] log tables: T, D, Tk, Dk by mantissa (filled every field)
    type t_td is array (0 to 127) of unsigned(63 downto 0);
    signal td_ram : t_td := (others => (others => '0'));
    attribute no_rw_check of td_ram : signal is true;
    signal td_q, td_wd : unsigned(63 downto 0) := (others => '0');
    signal td_wa : unsigned(6 downto 0) := (others => '0');
    signal td_we, ft_run : std_logic := '0';
    signal ft_i : unsigned(7 downto 0) := (others => '0');
    signal ft_c : unsigned(4 downto 0) := (others => '0');
    signal ft_pp : unsigned(19 downto 0) := (others => '0');
    signal ft_kp : unsigned(22 downto 0) := (others => '0');

    -- [C] the y engine: |y - y0| for this line and the one before, injected into the x path as two samples
    -- ahead of the pre-pixel (the x path computes Ly, its per-line gradient and the y rulings' tone)
    signal e_vy   : signed(19 downto 0) := (others => '0');
    signal e_d    : signed(15 downto 0) := (others => '0');
    signal y_a, y_b : unsigned(12 downto 0) := to_unsigned(1, 13);
    signal e_first, l_first : std_logic := '0';
    signal l_ly   : unsigned(17 downto 0) := (others => '0');
    signal l_gy   : unsigned(14 downto 0) := to_unsigned(1, 15);
    signal l_ty   : unsigned(10 downto 0) := (others => '0');
    -- A e and Ak e (mod 2^18) by octave, filled every field
    type t_ae is array (0 to 255) of unsigned(35 downto 0);
    signal ae_ram : t_ae := (others => (others => '0'));
    attribute no_rw_check of ae_ram : signal is true;
    signal ae_q, ae_wd : unsigned(35 downto 0) := (others => '0');
    signal ae_wa : unsigned(3 downto 0) := (others => '0');
    signal ae_we, ae_run : std_logic := '0';
    signal ae_aa, ae_ak : unsigned(17 downto 0) := (others => '0');

    -- [C] beam and the x path
    signal b_dx : signed(14 downto 0) := (others => '0');
    signal b_col : unsigned(10 downto 0) := (others => '0');
    signal s3_sx, s4_sx : unsigned(12 downto 0) := to_unsigned(1, 13);
    signal s4_e, s5_e, s6_e : unsigned(3 downto 0) := (others => '0');
    signal s5_m : unsigned(12 downto 0) := (others => '0');
    signal s5_sl : unsigned(1 downto 0) := (others => '0');
    signal s6_m : unsigned(12 downto 0) := (others => '0');
    signal s8_ae, s8_ake : unsigned(17 downto 0) := (others => '0');
    signal s7_ml, s8_ml : unsigned(4 downto 0) := (others => '0');
    signal s8_td : unsigned(63 downto 0) := (others => '0');
    signal s9_dpa, s9_dpb : unsigned(16 downto 0) := (others => '0');
    signal s9_kpa, s9_kpb : unsigned(19 downto 0) := (others => '0');
    signal s9_te, s9_tke, s10_te, s10_tke : unsigned(17 downto 0) := (others => '0');
    signal s10_ip : unsigned(11 downto 0) := (others => '0');
    signal s10_ipk : unsigned(14 downto 0) := (others => '0');
    signal s11_lx, s11_klx, s11_lin, s11_dpq, s12_dpq : unsigned(17 downto 0) := (others => '0');
    signal s12_lx, s12_klx, s13_lx, s14_lx, s15_fx, s16_fx : unsigned(17 downto 0) := (others => '0');
    signal s13_gd : unsigned(17 downto 0) := (others => '0');
    signal s13_u, s14_u : unsigned(7 downto 0) := (others => '0');
    signal vx_q, s14_v : unsigned(7 downto 0) := (others => '0');
    signal s14_g, s15_g, s16_g, s17_g : unsigned(14 downto 0) := (others => '0');
    signal s15_ix : unsigned(4 downto 0) := (others => '0');
    signal s15_af, s15_ad : unsigned(2 downto 0) := (others => '0');
    signal s16_lp, lp_q : unsigned(18 downto 0) := (others => '0');
    signal lp_a : unsigned(7 downto 0) := (others => '0');
    signal w_ix : unsigned(4 downto 0);
    signal w_bk : std_logic;
    signal s17_ax : unsigned(18 downto 0) := (others => '0');

    -- camera at the pixel path
    type t_lb is array (0 to 511) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(8 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal cq_a, cq_qa, cq_qb, cq_qb6 : unsigned(15 downto 0) := (others => '0');
    signal cq_w3, cq_w4, cq_w5, cq_w6 : unsigned(1 downto 0) := (others => '0');
    signal cq_j03, cq_j04 : std_logic := '0';
    signal cq_d : signed(16 downto 0) := (others => '0');
    signal cq_md : signed(18 downto 0) := (others => '0');
    signal cq_qb7 : unsigned(15 downto 0) := (others => '0');
    signal cq_f, cf9 : unsigned(15 downto 0) := (others => '0');
    signal cd10 : unsigned(17 downto 0) := (others => '0');
    signal c_pq2 : unsigned(17 downto 0) := to_unsigned(131072, 18);

    -- curves
    signal s13_ph, s14_ph : unsigned(17 downto 0) := (others => '0');
    signal s14_d, s15_d, s16_d, s17_d : unsigned(15 downto 0) := (others => '0');
    signal s14_px, s15_px, s15_py : unsigned(15 downto 0) := (others => '0');
    signal s14_p12, s14_ab, ph_q : unsigned(11 downto 0) := (others => '0');
    signal s16_mx, s16_mn : unsigned(15 downto 0) := (others => '0');
    signal s17_gc : unsigned(15 downto 0) := (others => '0');
    type t_ph is array (0 to 2047) of unsigned(11 downto 0);
    signal ph_ram : t_ph := (others => (others => '0'));
    attribute no_rw_check of ph_ram : signal is true;
    signal ph_ra, ph_wa : unsigned(10 downto 0) := (others => '0');
    signal ph_wd : unsigned(11 downto 0) := (others => '0');
    signal ph_we : std_logic := '0';

    -- AA (x and curves): 18 E / far, 19 a x 2^(5-E) and the mantissa, 20 partial products, 21 q
    signal a18_ax, a18_ac : unsigned(18 downto 0) := (others => '0');
    signal a18_gx, a18_gc : unsigned(15 downto 0) := (others => '0');
    signal a18_ex, a18_ec : unsigned(3 downto 0) := (others => '0');
    signal a18_fx, a18_fc, a19_fx, a19_fc, a20_fx, a20_fc : std_logic := '0';
    signal a19_sx, a19_sc : unsigned(7 downto 0) := (others => '0');
    signal a19_mx, a19_mc : unsigned(2 downto 0) := (others => '0');
    signal a20_px, a20_qx, a20_pc, a20_qc : unsigned(12 downto 0) := (others => '0');
    type t_c2 is array (16 to 22) of unsigned(1 downto 0);
    type t_a5 is array (16 to 23) of unsigned(2 downto 0);
    signal xc : t_c2 := (others => (others => '0'));
    signal xaf, xad : t_a5 := (others => (others => '0'));
    signal s21_qx, s21_qc : unsigned(6 downto 0) := (others => '0');
    type t_g5 is array (18 to 22) of unsigned(2 downto 0);
    signal gac : t_g5 := (others => (others => '0'));

    -- coverage and tone (stages 22 .. 27)
    signal s22_cvx, s22_cvc : unsigned(4 downto 0) := (others => '0');
    signal s23_tx, s23_tc : unsigned(10 downto 0) := (others => '0');
    signal s23_cx : unsigned(1 downto 0) := (others => '0');
    signal s24_tpx, s24_tc, s25_tpx, s25_tc, s26_tpx, s26_tc : unsigned(10 downto 0) := (others => '0');
    signal s24_apx, s24_ac, s25_apx, s25_ac : unsigned(2 downto 0) := (others => '0');
    signal s23_ac : unsigned(2 downto 0) := (others => '0');
    signal s25_dx, s25_dc : signed(11 downto 0) := (others => '0');
    signal s26_px, s26_pc, s26_qx, s26_qc : signed(14 downto 0) := (others => '0');
    signal s27_tx, s27_tc : unsigned(10 downto 0) := (others => '0');
    signal s28_pp, s28_tc, s29_pp : unsigned(10 downto 0) := (others => '0');
    signal s29_c5 : unsigned(4 downto 0) := (others => '0');
    signal s29_dp : unsigned(10 downto 0) := (others => '0');
    signal s29_s : unsigned(11 downto 0) := (others => '0');
    signal s30_a, s30_b : unsigned(15 downto 0) := (others => '0');
    signal s30_pp, s31_o : unsigned(10 downto 0) := (others => '0');
    signal s32_y, s33_y, s34_y, s36_y, s37_y, s38_y, s39_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s35_fa : unsigned(11 downto 0) := (others => '0');
    signal s35_fb : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_v1, s_v2, s_v2d, s_v3, s_v4, s_ld, s_fl, s_seen, s_efl, s_ffl : std_logic := '0';
    type t_gt is array (0 to 255) of unsigned(15 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt_ram : signal is true;
    signal gt_q, s_tv, gt_wd : unsigned(15 downto 0) := (others => '0');
    signal gt_wa, gt_i : unsigned(7 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_acc : unsigned(21 downto 0) := (others => '0');
    signal s_col, s_c2, s_bcol, s_ecol, s_fcol : unsigned(10 downto 0) := (others => '0');
    signal s_hq, s_hq4 : unsigned(15 downto 0) := (others => '0');
    signal s_ev, s_fv : std_logic := '0';
    signal s_eb : unsigned(15 downto 0) := (others => '0');
    signal s_eq, s_fq : unsigned(15 downto 0) := (others => '0');
    signal s_dd : signed(16 downto 0) := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_q <= C_DC_SEQ(to_integer(r_a));
            r_f <= r_q;
        end if;
    end process p_rom;
    r_a <= r_az;
    -- table writes straight from the fill registers (entry i - 1 at sub-step 23 of entry i)
    td_we <= '1' when ft_run = '1' and ft_c = 23 and ft_i /= 0 else '0';
    td_wa <= ft_i(6 downto 0) - 1;
    td_wd <= '0' & ft_pp(17 downto 0) & (f_acc(27 downto 16) - ft_pp(11 downto 0))
             & ft_kp(17 downto 0) & (f_acc2(30 downto 16) - ft_kp(14 downto 0));
    ae_we <= ae_run; ae_wd <= ae_aa & ae_ak;
    gt_we <= gt_run; gt_wa <= gt_i; gt_wd <= resize(gt_acc(21 downto 10), 16);

    -- the nearest drawn line (logic of the stage-14 value) addresses the line-position ROM
    w_bk <= s_semi and not s_st(13);
    w_ix <= f_idx(s14_u, s_md, '1') when w_bk = '1' else f_idx(s14_v, s_md, '0');
    lp_a <= "00" & w_bk & w_ix;
    p_lprom : process(clk)
    begin
        if rising_edge(clk) then
            lp_q <= C_DC_LP(to_integer(lp_a));
        end if;
    end process p_lprom;

    p_vrom : process(clk)
    begin
        if rising_edge(clk) then
            vx_q <= C_DC_V(to_integer(s12_lx(17 downto 9)));
        end if;
    end process p_vrom;

    -- knob register file (zone only)
    p_krram : process(clk)
    begin
        if rising_edge(clk) then
            kr_q <= kr_ram(to_integer(kr_ra));
            kr_f <= kr_q;
            if kr_we = '1' then kr_ram(to_integer(kr_wa)) <= kr_wd; end if;
        end if;
    end process p_krram;

    -- sync ring: written every clock, read C_LAT - 3 behind (+ the read register and one fabric copy)
    p_syram : process(clk)
    begin
        if rising_edge(clk) then
            sy_q <= sy_ram(to_integer(sy_wp - (C_LAT - 3)));
            sy_ram(to_integer(sy_wp)) <= '0' & g_avid & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_syram;

    -- log tables: written in the zone only, read by the pixel path (stage 6 address)
    p_tdram : process(clk)
    begin
        if rising_edge(clk) then
            td_q <= td_ram(to_integer(s6_m(11 downto 5)));
            if td_we = '1' then td_ram(to_integer(td_wa)) <= td_wd; end if;
        end if;
    end process p_tdram;

    -- A e / Ak e by octave: written in the zone only, read by the pixel path (stage 6 address)
    p_aeram : process(clk)
    begin
        if rising_edge(clk) then
            ae_q <= ae_ram(to_integer(s6_e));
            if ae_we = '1' then ae_ram(to_integer(ae_wa)) <= ae_wd; end if;
        end if;
    end process p_aeram;

    -- curve phase store (the line above): read at stage 12, written at stage 14 (never the same address)
    p_phram : process(clk)
    begin
        if rising_edge(clk) then
            ph_q <= ph_ram(to_integer(ph_ra));
            if ph_we = '1' then ph_ram(to_integer(ph_wa)) <= ph_wd; end if;
        end if;
    end process p_phram;

    -- gain table: written in the zone, read by the source path
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(s_sl));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
        end if;
    end process p_gtram;

    -- source store (quarter horizontal resolution): port 1 = the one-pole's own read (ahead of its write),
    -- port 2 = the pixel path (ahead of this line's write: it reads the previous line)
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_c2(10 downto 2)));
            lb_q2 <= lb_ram(to_integer(b_col(10 downto 2)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- per-field constants as logic of the field-latched switches and video mode
    ---------------------------------------------------------------------------
    p_const : process(s_xs, s_yl, s_md, s_semi, s_ink, s_fine)
    begin

                        case to_integer(s_xs) is
                            when 4      => s_oxs <= to_unsigned(204, 9); c_xpre <= to_unsigned(490, 10);
                            when 3      => s_oxs <= to_unsigned(153, 9); c_xpre <= to_unsigned(7, 10);
                            when others => s_oxs <= to_unsigned(102, 9); c_xpre <= to_unsigned(5, 10);
                        end case;
                        case to_integer(s_yl) is
                            when 256        => c_iir <= to_unsigned(6, 3);
                            when 384 | 512  => c_iir <= to_unsigned(5, 3);
                            when others     => c_iir <= to_unsigned(4, 3);
                        end case;
                        if s_ink = '1' then c_M <= to_unsigned(512, 10); c_Mc <= to_unsigned(512, 10);
                        else c_M <= to_unsigned(320, 10); c_Mc <= to_unsigned(448, 10); end if;
                        -- fine-line fade shift (14 - sh): log [5 4 7 8], lin [5 3 6 7]
                        case to_integer(s_md) is
                            when 0 => c_shy <= to_unsigned(9, 4);
                            when 1 => c_shy <= to_unsigned(10, 4);
                            when 2 => c_shy <= to_unsigned(7, 4);
                            when others => c_shy <= to_unsigned(6, 4);
                        end case;
                        if s_semi = '0' then
                            case to_integer(s_md) is
                                when 0 => c_shx <= to_unsigned(9, 4);
                                when 1 => c_shx <= to_unsigned(10, 4);
                                when 2 => c_shx <= to_unsigned(7, 4);
                                when others => c_shx <= to_unsigned(6, 4);
                            end case;
                        else
                            case to_integer(s_md) is
                                when 0 => c_shx <= to_unsigned(9, 4);
                                when 1 => c_shx <= to_unsigned(11, 4);
                                when 2 => c_shx <= to_unsigned(8, 4);
                                when others => c_shx <= to_unsigned(7, 4);
                            end case;
                        end if;
                            -- widths (1/16 px): 8 + (w8 r) >> 8, w8 = Bold 48 32 20 24 / Fine 20 16 16 16, by mode

                        if s_fine = '0' then
                            case to_integer(s_xs) is
                                when 4 => c_cwx0 <= to_unsigned(32, 7); c_cwx1 <= to_unsigned(24, 7); c_cwx2 <= to_unsigned(18, 7); c_cwc <= to_unsigned(20, 7);
                                when 3 => c_cwx0 <= to_unsigned(40, 7); c_cwx1 <= to_unsigned(29, 7); c_cwx2 <= to_unsigned(21, 7); c_cwc <= to_unsigned(24, 7);
                                when others => c_cwx0 <= to_unsigned(56, 7); c_cwx1 <= to_unsigned(40, 7); c_cwx2 <= to_unsigned(28, 7); c_cwc <= to_unsigned(32, 7);
                            end case;
                            case to_integer(s_yl) is
                                when 256 => c_cwy0 <= to_unsigned(56, 7); c_cwy1 <= to_unsigned(40, 7); c_cwy2 <= to_unsigned(28, 7);
                                when 384 => c_cwy0 <= to_unsigned(40, 7); c_cwy1 <= to_unsigned(29, 7); c_cwy2 <= to_unsigned(21, 7);
                                when 512 => c_cwy0 <= to_unsigned(32, 7); c_cwy1 <= to_unsigned(24, 7); c_cwy2 <= to_unsigned(18, 7);
                                when 576 => c_cwy0 <= to_unsigned(29, 7); c_cwy1 <= to_unsigned(22, 7); c_cwy2 <= to_unsigned(16, 7);
                                when 480 => c_cwy0 <= to_unsigned(33, 7); c_cwy1 <= to_unsigned(25, 7); c_cwy2 <= to_unsigned(18, 7);
                                when 960 => c_cwy0 <= to_unsigned(20, 7); c_cwy1 <= to_unsigned(16, 7); c_cwy2 <= to_unsigned(16, 7);
                                when others => c_cwy0 <= to_unsigned(18, 7); c_cwy1 <= to_unsigned(16, 7); c_cwy2 <= to_unsigned(16, 7);
                            end case;
                        else
                            case to_integer(s_xs) is
                                when 4 => c_cwx0 <= to_unsigned(18, 7); c_cwx1 <= to_unsigned(16, 7); c_cwx2 <= to_unsigned(16, 7); c_cwc <= to_unsigned(16, 7);
                                when 3 => c_cwx0 <= to_unsigned(21, 7); c_cwx1 <= to_unsigned(18, 7); c_cwx2 <= to_unsigned(18, 7); c_cwc <= to_unsigned(18, 7);
                                when others => c_cwx0 <= to_unsigned(28, 7); c_cwx1 <= to_unsigned(24, 7); c_cwx2 <= to_unsigned(24, 7); c_cwc <= to_unsigned(24, 7);
                            end case;
                            case to_integer(s_yl) is
                                when 256 => c_cwy0 <= to_unsigned(28, 7); c_cwy1 <= to_unsigned(24, 7); c_cwy2 <= to_unsigned(24, 7);
                                when 384 => c_cwy0 <= to_unsigned(21, 7); c_cwy1 <= to_unsigned(18, 7); c_cwy2 <= to_unsigned(18, 7);
                                when 480 | 512 => c_cwy0 <= to_unsigned(18, 7); c_cwy1 <= to_unsigned(16, 7); c_cwy2 <= to_unsigned(16, 7);
                                when others => c_cwy0 <= to_unsigned(16, 7); c_cwy1 <= to_unsigned(16, 7); c_cwy2 <= to_unsigned(16, 7);
                            end case;
                        end if;

    end process p_const;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] CONTROLS + [C] the per-field sequencer (one small
    -- operation per zone clock; the zone is quasi-static but timed at the
    -- pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_r  : unsigned(9 downto 0);
        variable v_e  : unsigned(17 downto 0);
        variable v_s  : unsigned(19 downto 0);
        variable v_ch : integer range 0 to 5;
        variable v_ld : std_logic;
        variable v_fa : unsigned(23 downto 0);
        variable v_fb : unsigned(19 downto 0);
        variable v_fa2 : unsigned(21 downto 0);
        variable v_v  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- [A] vblank zone: no avid for 1024 clocks
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

            -- [A] width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            g_wm1 <= s_w - 1;

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_semi  <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_osc   <= registers_in(6)(4);
                s_xs <= s_xsr; s_yl <= s_ys;
                -- K4 Lines: four steps, a new one latched 8 counts inside it (the two ends always)
                v_r := unsigned(registers_in(3));
                if s_first = '1' or (v_r(9 downto 8) /= s_md and
                    ((v_r(7 downto 3) /= "00000" and v_r(7 downto 3) /= "11111") or
                     v_r(9 downto 3) = "0000000" or v_r(9 downto 3) = "1111111")) then
                    s_md <= v_r(9 downto 8);
                end if;
                s_ot <= s_ot + 1;
            end if;

            -- zone sequencer
            v_ld := '0'; v_fa := (others => '0'); v_fb := (others => '0'); v_fa2 := (others => '0');
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 2 =>
                        s_oys <= resize(shift_right(resize(s_yl, 19) * to_unsigned(121, 7), 8), 10);
                    -- the origin: x0 = (K1q4 139) >> 5 - 40384, y0 = (K2q4 86) >> 5 - 26816 (Q4 h), from the glides
                    when 70 => kr_ra <= to_unsigned(8, 4);
                    when 73 => v_fa := resize(kr_f, 24); v_fb := to_unsigned(139, 20); v_ld := '1';
                    when 93 => t_x0 <= signed(resize(shift_right(f_acc, 5), 19)) - 40384;
                               kr_ra <= to_unsigned(9, 4);
                    when 96 => v_fa := resize(kr_f, 24); v_fb := to_unsigned(86, 20); v_ld := '1';
                               kr_ra <= to_unsigned(10, 4);
                    when 99 => z_rd <= kr_f;                                    -- K3 glide
                               kr_ra <= to_unsigned(12, 4);
                    when 102 => c_e3 <= resize(shift_right(shift_left(resize(kr_f, 21), 1) + resize(kr_f, 21), 6), 10);   -- P12
                               kr_ra <= to_unsigned(13, 4);
                    when 105 => c_v <= resize(shift_right(signed(kr_f), 4), 11);   -- the Drift speed
                               kr_ra <= to_unsigned(11, 4);
                    when 108 => z_g6 <= kr_f;                                   -- K6
                    when 116 => t_y0 <= signed(resize(shift_right(f_acc, 5), 19)) - 26816;
                    when 117 =>
                        c_x0h <= resize(shift_right(t_x0, 4), 15);
                        c_y0h <= resize(shift_right(t_y0, 4), 15);
                    when 118 => c_dx0 <= signed(resize(c_xpre, 15)) - c_x0h;
                    -- A = P log10 2 = 2^(18 + la / 256) (Q18), la = ((K3q4 149) >> 12) - 491
                    when 120 => v_fa := resize(z_rd, 24); v_fb := to_unsigned(149, 20); v_ld := '1';
                    when 140 => c_vA <= resize(shift_right(f_acc, 12), 13) + (4608 - 491);
                    when 141 =>
                        r_az <= '0' & c_vA(7 downto 0);
                    when 142 => r_az <= '0' & c_e3(7 downto 0);
                    when 144 =>
                        case to_integer(c_vA(9 downto 8)) is
                            when 0      => c_A <= resize(r_f & '0', 19);      -- I = 16
                            when 1      => c_A <= resize(r_f & "00", 19);     -- I = 17
                            when others => c_A <= r_f & "000";                -- I = 18
                        end case;
                    when 145 =>
                        v_e := shift_left(resize(r_f, 18), to_integer(c_e3(9 downto 8)));
                        c_E3m <= v_e - 32768;
                    when 146 => v_fa := resize(c_E3m, 24); v_fb := to_unsigned(17922, 20); v_ld := '1';      -- k
                    -- Drift speed: |v| < 24 still, else 2^-12 .. 2^-5 decade per field (Q24)
                    when 147 =>
                        c_vpos <= '0';
                        if c_v > 0 then c_vpos <= '1'; end if;
                        v_v := abs(c_v);
                        c_av <= unsigned(v_v(9 downto 0));
                    when 148 =>
                        if c_av < 24 then c_zero <= '1'; c_u <= (others => '0');
                        else c_zero <= '0'; c_u <= resize(c_av - 24, 9); end if;
                    when 149 => c_ls <= c_u - shift_right(c_u, 4) - shift_right(c_u, 6);
                    when 150 => r_az <= '0' & c_ls(5 downto 0) & "00";
                    when 153 =>
                        v_s := shift_left(resize(shift_right(r_f, 3), 20), to_integer(c_ls(8 downto 6)));
                        if c_zero = '1' then c_spd <= (others => '0'); else c_spd <= v_s; end if;
                    when 154 =>
                        if c_vpos = '1' then c_dst <= -signed(resize(c_spd, 21)); else c_dst <= signed(resize(c_spd, 21)); end if;
                    when 155 => c_dlt <= c_dlt + unsigned(resize(c_dst, 24));
                    when 166 => c_kq <= resize(shift_right(f_acc, 15), 17);
                    when 170 => v_fa := resize(c_A, 24); v_fb := resize(c_kq, 20); v_ld := '1';         -- Ak = 2k A
                    when 190 => c_Ak <= resize(shift_right(f_acc, 14), 22);
                    when 194 => v_fa := unsigned(resize(c_dst, 24)); v_fb := resize(c_kq, 20); v_ld := '1'; -- drift share of the curves
                    when 214 => c_phc <= c_phc + unsigned(resize(shift_right(signed(f_acc), 14), 24));
                    when 215 => c_pq2 <= c_phc(23 downto 6) + to_unsigned(131072, 18);
                    -- camera gain: G6 = (K6q4 75) >> 8, Osc 2.5 x
                    when 218 => v_fa := resize(z_g6, 24); v_fb := to_unsigned(75, 20); v_ld := '1';
                    when 238 => c_G6 <= resize(shift_right(f_acc, 8), 14);
                    when 239 =>
                        if s_osc = '1' then c_G6 <= shift_left(c_G6, 1) + shift_right(c_G6, 1); end if;
                    when others => null;
                end case;
            end if;

            -- [B] knob holds (+-1 count; the first field latches unconditionally) and glides, six channels of 10
            -- clocks (zone 4 .. 63) on the register file: 1/16 of the gap per field (the Drift speed 1/8), the rest
            -- snapped once under 1 count (X / Y: under 8 counts, so a position never moves slower than 1 px per
            -- field); Glide off snaps
            kr_we <= '0';
            if s_zone = '1' and s_zc = 4 then
                kr_run <= '1'; kr_ch <= (others => '0'); kr_c <= (others => '0');
            elsif kr_run = '1' then
                kr_c <= kr_c + 1;
                case to_integer(kr_c) is
                    when 0 =>
                        kr_ra <= '0' & kr_ch;
                        case to_integer(kr_ch) is
                            when 0 => z_hv <= unsigned(registers_in(0));
                            when 1 => z_hv <= unsigned(registers_in(1));
                            when 2 => z_hv <= unsigned(registers_in(2));
                            when 3 => z_hv <= unsigned(registers_in(5));
                            when 4 => z_hv <= unsigned(registers_in(7));
                            when others => z_hv <= unsigned(registers_in(4));
                        end case;
                    when 3 =>
                        z_hh <= kr_f(9 downto 0);
                        kr_ra <= '1' & kr_ch;
                    when 4 =>
                        if s_first = '1' or resize(z_hv, 11) > resize(z_hh, 11) + 1 or resize(z_hv, 11) + 1 < resize(z_hh, 11) then
                            z_hh <= z_hv; kr_wd <= resize(z_hv, 16);
                        else
                            kr_wd <= resize(z_hh, 16);
                        end if;
                        kr_we <= '1'; kr_wa <= '0' & kr_ch;
                    when 6 =>
                        z_gm <= signed(kr_f);
                    when 7 =>
                        if kr_ch = 5 then z_gd <= shift_left(resize(signed('0' & z_hh) - 512, 16), 4) - z_gm;
                        else z_gd <= signed(resize(z_hh & "0000", 16)) - z_gm; end if;
                    when 8 =>
                        -- snap: glide off, or the gap's top bits are all sign (|gap| < 128 for X / Y, < 16 else)
                        if s_glide = '0' or (kr_ch(2 downto 1) = "00" and (z_gd(15 downto 7) = "000000000" or z_gd(15 downto 7) = "111111111"))
                           or (kr_ch(2 downto 1) /= "00" and (z_gd(15 downto 4) = "000000000000" or z_gd(15 downto 4) = "111111111111")) then
                            z_gs <= z_gd;
                        elsif kr_ch = 5 then z_gs <= shift_right(z_gd, 3);
                        else z_gs <= shift_right(z_gd, 4); end if;
                    when 9 =>
                        kr_we <= '1'; kr_wa <= '1' & kr_ch; kr_wd <= unsigned(z_gm + z_gs);
                        kr_c <= (others => '0'); kr_ch <= kr_ch + 1;
                        if kr_ch = 5 then kr_run <= '0'; s_first <= '0'; end if;
                    when others => null;
                end case;
            end if;

            -- [C] per-field fill of the log tables (zone 520 .. 3616, 24 clocks per entry): P(i) = (A L2(i)) >> 16
            -- and Pk(i) = (Ak L2(i)) >> 16 on the serial multiplier's two accumulators; entry i - 1 =
            -- (P(i-1) mod 2^18, P(i) - P(i-1), the same for Pk)
            if s_zone = '1' and s_zc = 520 then
                ft_run <= '1'; ft_i <= (others => '0'); ft_c <= (others => '0');
            elsif ft_run = '1' then
                ft_c <= ft_c + 1;
                case to_integer(ft_c) is
                    when 0 =>
                        r_az <= "10" & ft_i(6 downto 0);
                    when 3 =>
                        v_fa := resize(c_A, 24); v_fa2 := c_Ak; v_ld := '1';
                        if ft_i(7) = '1' then v_fb := to_unsigned(65536, 20); else v_fb := resize(r_f, 20); end if;
                    when 23 =>
                        ft_pp <= resize(shift_right(f_acc, 16), 20); ft_kp <= resize(shift_right(f_acc2, 16), 23);
                        ft_c <= (others => '0');
                        if ft_i = 128 then ft_run <= '0'; end if;
                        ft_i <= ft_i + 1;
                    when others => null;
                end case;
            end if;

            -- the per-field serial multiplier (20 iterations; signed a x unsigned b, mod 2^40; the second
            -- accumulator serves the table fill)
            if v_ld = '1' then
                f_acc <= (others => '0'); f_acc2 <= (others => '0');
                f_a <= unsigned(resize(signed(v_fa), 40)); f_a2 <= resize(v_fa2, 40); f_b <= v_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; f_acc2 <= f_acc2 + f_a2; end if;
                f_a <= f_a(38 downto 0) & '0'; f_a2 <= f_a2(38 downto 0) & '0';
                f_b <= '0' & f_b(19 downto 1);
            end if;

            -- [A] generated-avid and start-pulse shift registers (the late taps come from the ring)
            s_gav_sr(0) <= g_avid;
            for i in 1 to 15 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
            sy_wp <= sy_wp + 1;
            sy_f <= sy_q;                                        -- delay C_LAT - 1
            o_hs <= sy_f(0); o_vs <= sy_f(1); o_av <= sy_f(2);  -- delay C_LAT
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 25);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] per-field fill of the log tables (zone 520 .. 3616, 24 clocks per
    -- entry): P(i) = (A L2(i)) >> 16 and Pk(i) = (Ak L2(i)) >> 16 on two serial
    -- multipliers; entry i - 1 = (P(i-1) mod 2^18, P(i) - P(i-1), the same for Pk)
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_dp : unsigned(19 downto 0);
        variable v_dk : unsigned(22 downto 0);
    begin
        if rising_edge(clk) then
            -- A e / Ak e for e = 0 .. 15 by accumulation (zone 260 .. 276)
            if s_zone = '1' and s_zc = 260 then
                ae_run <= '1'; ae_wa <= (others => '0'); ae_aa <= to_unsigned(0, 18) - c_dlt(23 downto 6); ae_ak <= (others => '0');
            elsif ae_run = '1' then
                ae_wa <= ae_wa + 1;
                ae_aa <= ae_aa + c_A(17 downto 0); ae_ak <= ae_ak + c_Ak(17 downto 0);
                if ae_wa = 15 then ae_run <= '0'; end if;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width)
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
    -- [C] the y engine: |y - y0| (h) of the line before and of this line, ready at each g_start (the
    -- zone prepares lines -1 and 0); the x path takes them as two samples ahead of the pre-pixel
    ---------------------------------------------------------------------------
    p_yeng : process(clk)
        variable v_y : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 3650 then
                e_vy <= resize(signed('0' & shift_right(s_yl, 1)), 20) - signed(resize(s_yl, 20));
            elsif (s_zone = '1' and s_zc = 3653) or s_st(2) = '1' then
                e_vy <= e_vy + signed(resize(s_yl, 20));
            end if;
            if (s_zone = '1' and (s_zc = 3651 or s_zc = 3654)) or s_st(3) = '1' then
                v_y := resize(shift_right(e_vy, 7), 16);
                e_d <= 2160 - v_y - resize(c_y0h, 16);
            end if;
            if (s_zone = '1' and (s_zc = 3652 or s_zc = 3655)) or s_st(4) = '1' then
                if e_d(15) = '1' then v_y := -e_d; else v_y := e_d; end if;
                if v_y = 0 then y_b <= to_unsigned(1, 13); else y_b <= unsigned(v_y(12 downto 0)); end if;
            end if;
            if (s_zone = '1' and s_zc = 3655) or s_st(2) = '1' then
                y_a <= y_b;
            end if;
            if s_zone = '1' and s_zc = 3655 then e_first <= '1';
            elsif g_start = '1' then e_first <= '0'; end if;
            if g_start = '1' then l_first <= e_first; end if;
        end if;
    end process p_yeng;

    ---------------------------------------------------------------------------
    -- [C] BEAM + x path (stages 2 .. 17)
    ---------------------------------------------------------------------------
    p_x : process(clk)
        variable v_sa : unsigned(3 downto 0);
        variable v_s  : unsigned(12 downto 0);
        variable v_d  : unsigned(11 downto 0);
        variable v_k  : unsigned(14 downto 0);
        variable v_t  : unsigned(17 downto 0);
        variable v_u  : unsigned(17 downto 0);
        variable v_l  : unsigned(23 downto 0);
        variable v_a  : unsigned(14 downto 0);
        variable v_dd : signed(19 downto 0);
        variable v_n  : signed(14 downto 0);
        variable v_sh : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: beam (the pre-pixel loads 2 clocks after g_start: the two y samples go first)
            if s_st(1) = '1' then
                b_dx <= c_dx0; b_col <= to_unsigned(2, 11);
            else
                b_dx <= b_dx + signed(resize(s_xs, 15)); b_col <= b_col + 1;
            end if;
            -- the semi-log copies hold pixel n at stage 10
            -- 3: s = |x - x0| (h), at least 1
            if b_dx(14) = '1' then v_n := -b_dx; else v_n := b_dx; end if;
            v_s := unsigned(v_n(12 downto 0));
            if s_st(0) = '1' then s3_sx <= y_a;                 -- y sample: the line before
            elsif s_st(1) = '1' then s3_sx <= y_b;              -- y sample: this line
            elsif v_s = 0 then s3_sx <= to_unsigned(1, 13); else s3_sx <= v_s; end if;
            -- 4: leading one
            s4_e <= f_lod(s3_sx); s4_sx <= s3_sx;
            -- 5: normalise (coarse)
            v_sa := 12 - s4_e;
            s5_m <= shift_left(s4_sx, to_integer(v_sa(3 downto 2) & "00"));
            s5_sl <= v_sa(1 downto 0);
            s5_e <= s4_e;
            -- 6: normalise (fine): the mantissa; table address = m(11 .. 5)
            s6_m <= shift_left(s5_m, to_integer(s5_sl));
            s6_e <= s5_e;
            -- 7: the tables' own registers (p_tdram, p_aeram)
            s7_ml <= s6_m(4 downto 0);
            -- 8: fabric copies of the table words
            s8_td <= td_q; s8_ml <= s7_ml;
            s8_ae <= ae_q(35 downto 18); s8_ake <= ae_q(17 downto 0);
            -- 9: interpolation partial products; A e + T
            v_d := s8_td(44 downto 33);           -- D
            v_k := s8_td(14 downto 0);            -- Dk
            v_t := (others => '0');
            if s8_ml(0) = '1' then v_t := v_t + resize(v_d, 18); end if;
            if s8_ml(1) = '1' then v_t := v_t + resize(v_d & '0', 18); end if;
            if s8_ml(2) = '1' then v_t := v_t + resize(v_d & "00", 18); end if;
            s9_dpa <= v_t(16 downto 0);
            v_t := (others => '0');
            if s8_ml(3) = '1' then v_t := v_t + resize(v_d & "000", 18); end if;
            if s8_ml(4) = '1' then v_t := v_t + resize(v_d & "0000", 18); end if;
            s9_dpb <= v_t(16 downto 0);
            v_u := (others => '0');
            if s8_ml(0) = '1' then v_u := v_u + resize(v_k, 18); end if;
            if s8_ml(1) = '1' then v_u := v_u + resize(v_k & '0', 18); end if;
            if s8_ml(2) = '1' then v_u := v_u + resize(v_k & "00", 18); end if;
            s9_kpa <= resize(v_u, 20);
            s9_kpb <= (others => '0');
            if s8_ml(3) = '1' and s8_ml(4) = '1' then s9_kpb <= resize(v_k & "000", 20) + resize(v_k & "0000", 20);
            elsif s8_ml(3) = '1' then s9_kpb <= resize(v_k & "000", 20);
            elsif s8_ml(4) = '1' then s9_kpb <= resize(v_k & "0000", 20); end if;
            s9_te  <= s8_ae + s8_td(62 downto 45);          -- A e + T
            s9_tke <= s8_ake + s8_td(32 downto 15);         -- Ak e + Tk
            -- 10
            s10_ip  <= resize(shift_right(resize(s9_dpa, 18) + resize(s9_dpb, 18), 5), 12);
            s10_ipk <= resize(shift_right(resize(s9_kpa, 21) + resize(s9_kpb, 21), 5), 15);
            s10_te <= s9_te; s10_tke <= s9_tke;
            -- 11: log-log Lx, kLx (the drift is in the A e table);  semi-log: s / 1024 - drift
            s11_lx  <= s10_te + resize(s10_ip, 18);
            s11_klx <= s10_tke + resize(s10_ipk, 18);
            v_n := b_dx - signed(resize(s_xs & "000", 15));      -- the beam is 8 pixels ahead of stage 11
            if v_n(14) = '1' then v_n := -v_n; end if;
            v_s := unsigned(v_n(12 downto 0));
            if v_s = 0 then v_s := to_unsigned(1, 13); end if;
            s11_lin <= (v_s(9 downto 0) & "00000000") - c_dlt(23 downto 6);
            s11_dpq <= cd10 + c_pq2;
            -- 12: the x coordinate, log-log or semi-log (the y samples always log)
            if s_semi = '1' and s_st(9) = '0' and s_st(10) = '0' then s12_lx <= s11_lin;
            else s12_lx <= s11_lx; end if;
            s12_klx <= s11_klx;
            s12_dpq <= s11_dpq;
            -- 13: the decade-value ROM (p_vrom); the x gradient against the previous pixel; lin bank value
            s13_lx <= s12_lx;
            s13_gd <= s12_lx - s13_lx;
            v_t := resize(s12_lx(17 downto 9) & "0000000", 18) + resize(s12_lx(17 downto 9) & "00000", 18) + 80;
            s13_u <= v_t(16 downto 9);
            -- 14
            s14_v <= vx_q; s14_u <= s13_u; s14_lx <= s13_lx;
            v_t := f_abs18(s13_gd);
            if v_t > 16384 then s14_g <= to_unsigned(16384, 15); else s14_g <= v_t(14 downto 0); end if;
            -- 15: nearest drawn line; fade weights from the gradient
            s15_ix <= w_ix;
            s15_fx <= s14_lx;
            if s14_g = 0 then s15_g <= to_unsigned(1, 15); else s15_g <= s14_g; end if;
            -- fade weight, quarters: (g >> sh) >= 32 -> 4, in 16 .. 31 -> its bits 3 .. 2, else 0
            if s_st(13) = '1' then v_sh := c_shy; else v_sh := c_shx; end if;     -- the y sample
            s15_af <= f_fade(s14_g, v_sh);
            s15_ad <= f_fade(s14_g, to_unsigned(9, 4));
            -- 16: the line's position and class
            s16_lp <= lp_q;                                     -- fabric copy of the ROM word
            if s15_ix = 20 or (s15_ix = 2 and (s_semi = '0' or s_st(14) = '1')) or (s15_ix = 0 and s_semi = '1' and s_st(14) = '0') then xc(16) <= "00";
            elsif s15_ix = 10 then xc(16) <= "01"; else xc(16) <= "10"; end if;
            s16_fx <= s15_fx; s16_g <= s15_g;
            xaf(16) <= s15_af; xad(16) <= s15_ad;
            -- 17: |frac - line|
            v_dd := signed(resize(s16_fx, 20)) - signed(resize(s16_lp, 20));
            if v_dd(19) = '1' then s17_ax <= unsigned(resize(-v_dd, 19)); else s17_ax <= unsigned(resize(v_dd, 19)); end if;
            s17_g <= s16_g;
            for k in 17 to 22 loop xc(k) <= xc(k - 1); end loop;
            for k in 17 to 23 loop xaf(k) <= xaf(k - 1); xad(k) <= xad(k - 1); end loop;
        end if;
    end process p_x;

    ---------------------------------------------------------------------------
    -- camera at the pixel path (stages 3 .. 10): the quarter-resolution store,
    -- interpolated:  F = qb + ((qa - qb)(w + 1)) >> 2
    ---------------------------------------------------------------------------
    p_cam : process(clk)
        variable v_m : signed(18 downto 0);
        variable v_f : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: store read issued at stage 2 (address = column / 4); 4: fabric copy
            cq_w3 <= b_col(1 downto 0);
            if b_col(10 downto 2) = 0 then cq_j03 <= '1'; else cq_j03 <= '0'; end if;
            cq_a <= lb_q2; cq_w4 <= cq_w3; cq_j04 <= cq_j03;
            -- 5: this quarter column and the one before
            cq_qa <= cq_a;
            if cq_j04 = '1' then cq_qb <= cq_a;
            elsif cq_w4 = 0 then cq_qb <= cq_qa; end if;
            cq_w5 <= cq_w4;
            -- 6: qa - qb
            cq_d <= signed(resize(cq_qa, 17)) - signed(resize(cq_qb, 17));
            cq_qb6 <= cq_qb; cq_w6 <= cq_w5;
            -- 7: (qa - qb)(w + 1)
            case to_integer(cq_w6) is
                when 0 => v_m := resize(cq_d, 19);
                when 1 => v_m := shift_left(resize(cq_d, 19), 1);
                when 2 => v_m := resize(cq_d, 19) + shift_left(resize(cq_d, 19), 1);
                when others => v_m := shift_left(resize(cq_d, 19), 2);
            end case;
            cq_md <= v_m; cq_qb7 <= cq_qb6;
            -- 8: F (Q12.4)
            v_f := signed(resize(cq_qb7, 19)) + shift_right(cq_md, 2);
            cq_f <= unsigned(v_f(15 downto 0));
            -- 9: carried; 10: the curves' phase share = 40 F (cycles Q18)
            cf9 <= cq_f;
            cd10 <= shift_left(resize(cf9, 18), 5) + shift_left(resize(cf9, 18), 3);
        end if;
    end process p_cam;

    ---------------------------------------------------------------------------
    -- curves (13 .. 17), AA (18 .. 21), coverage and tone (22 .. 27)
    ---------------------------------------------------------------------------
    p_grad : process(clk)
        variable v_t  : unsigned(17 downto 0);
        variable v_p  : unsigned(15 downto 0);
        variable v_g  : unsigned(16 downto 0);
        variable v_cw : unsigned(6 downto 0);
        variable v_q  : unsigned(6 downto 0);
        variable v_c  : signed(7 downto 0);
        variable v_a  : unsigned(14 downto 0);
        variable v_s  : signed(16 downto 0);
        variable v_k  : unsigned(4 downto 0);
        variable v_w  : unsigned(12 downto 0);
        variable v_q12 : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            ---------------- curves: 13 .. 17
            -- 13: phase = 2 Ly - kLx + camera share + drift share + 1/2
            s13_ph <= (l_ly(16 downto 0) & '0') - s12_klx + s12_dpq;
            -- the store: read address = pixel index at stage 12, write at stage 14
            if s_st(12) = '1' then ph_ra <= (others => '0'); else ph_ra <= ph_ra + 1; end if;
            -- the y sample's results for this line: Ly (stage 12), its gradient (14), the rulings' tone (27)
            if s_st(11) = '1' then l_ly <= s12_lx; end if;
            if s_st(13) = '1' then l_gy <= s14_g; end if;
            if s_st(26) = '1' then l_ty <= s27_tx; end if;
            -- 14: distance to the nearest curve (Q16), the x gradient, the line above
            s14_ph <= s13_ph;
            if s13_ph(17) = '0' then s14_d <= s13_ph(17 downto 2);
            else v_t := to_unsigned(0, 18) - s13_ph; s14_d <= v_t(17 downto 2); end if;
            v_t := f_abs18(s13_ph - s14_ph);
            s14_px <= v_t(17 downto 2);
            s14_p12 <= s13_ph(17 downto 6);
            s14_ab <= ph_q;
            ph_wd <= s13_ph(17 downto 6);                       -- written at stage 14 (pixels 0 .. W + 1)
            ph_we <= s_gav_sr(13) or s_gav_sr(15);
            if s_st(14) = '1' then ph_wa <= (others => '0'); else ph_wa <= ph_wa + 1; end if;
            -- 15: the y gradient (the first line of a field uses the per-line one)
            s15_d <= s14_d; s15_px <= s14_px;
            if l_first = '1' then s15_py <= resize(shift_right(l_gy, 1), 16);
            else v_q12 := s14_p12 - s14_ab;
                if v_q12(11) = '1' then v_q12 := (not v_q12) + 1; end if;
                s15_py <= v_q12 & "0000"; end if;
            -- 16: max, min
            s16_d <= s15_d;
            if s15_px > s15_py then s16_mx <= s15_px; s16_mn <= s15_py; else s16_mx <= s15_py; s16_mn <= s15_px; end if;
            -- 17: |grad| = max + min/4 + min/8, 1 .. 32768
            s17_d <= s16_d;
            v_g := resize(s16_mx, 17) + resize(shift_right(s16_mn, 2), 17) + resize(shift_right(s16_mn, 3), 17);
            if v_g >= 32768 then s17_gc <= to_unsigned(32768, 16);
            elsif v_g = 0 then s17_gc <= to_unsigned(1, 16);
            else s17_gc <= v_g(15 downto 0); end if;

            ---------------- AA: q = 16 a / g (1/16 px), 64 = far (a >= 4 g)
            -- 18: the leading one of g, the far tests
            a18_ax <= s17_ax; a18_gx <= resize(s17_g, 16); a18_ex <= f_lod(resize(s17_g, 16));
            if resize(s17_ax, 21) >= shift_left(resize(s17_g, 21), 2) then a18_fx <= '1'; else a18_fx <= '0'; end if;
            a18_ac <= resize(s17_d, 19); a18_gc <= s17_gc; a18_ec <= f_lod(s17_gc);
            if resize(s17_d, 21) >= shift_left(resize(s17_gc, 21), 2) then a18_fc <= '1'; else a18_fc <= '0'; end if;
            v_a := resize(shift_right(s17_gc, 9), 15);
            if v_a < 16 then gac(18) <= (others => '0');
            elsif v_a >= 32 then gac(18) <= to_unsigned(4, 3);
            else gac(18) <= '0' & v_a(3 downto 2); end if;
            -- 19: a x 2^(5 - E) (8 bits), the mantissa of g
            a19_sx <= f_as(a18_ax, a18_ex); a19_mx <= f_m3(a18_gx, a18_ex); a19_fx <= a18_fx;
            a19_sc <= f_as(a18_ac, a18_ec); a19_mc <= f_m3(a18_gc, a18_ec); a19_fc <= a18_fc;
            gac(19) <= gac(18);
            -- 20: a_s x K2 (two partial sums)
            v_k := f_k2(a19_mx);
            v_w := (others => '0');
            if v_k(0) = '1' then v_w := v_w + resize(a19_sx, 13); end if;
            if v_k(1) = '1' then v_w := v_w + resize(a19_sx & '0', 13); end if;
            if v_k(2) = '1' then v_w := v_w + resize(a19_sx & "00", 13); end if;
            a20_px <= v_w;
            v_w := (others => '0');
            if v_k(3) = '1' then v_w := v_w + resize(a19_sx & "000", 13); end if;
            if v_k(4) = '1' then v_w := v_w + resize(a19_sx & "0000", 13); end if;
            a20_qx <= v_w;
            v_k := f_k2(a19_mc);
            v_w := (others => '0');
            if v_k(0) = '1' then v_w := v_w + resize(a19_sc, 13); end if;
            if v_k(1) = '1' then v_w := v_w + resize(a19_sc & '0', 13); end if;
            if v_k(2) = '1' then v_w := v_w + resize(a19_sc & "00", 13); end if;
            a20_pc <= v_w;
            v_w := (others => '0');
            if v_k(3) = '1' then v_w := v_w + resize(a19_sc & "000", 13); end if;
            if v_k(4) = '1' then v_w := v_w + resize(a19_sc & "0000", 13); end if;
            a20_qc <= v_w;
            a20_fx <= a19_fx; a20_fc <= a19_fc;
            gac(20) <= gac(19);
            -- 21: q = min((a_s K2) >> 6, 63), far -> 64
            v_w := a20_px + a20_qx;
            if a20_fx = '1' then s21_qx <= to_unsigned(64, 7);
            elsif v_w(12) = '1' then s21_qx <= to_unsigned(63, 7);
            else s21_qx <= '0' & v_w(11 downto 6); end if;
            v_w := a20_pc + a20_qc;
            if a20_fc = '1' then s21_qc <= to_unsigned(64, 7);
            elsif v_w(12) = '1' then s21_qc <= to_unsigned(63, 7);
            else s21_qc <= '0' & v_w(11 downto 6); end if;
            gac(21) <= gac(20);
            gac(22) <= gac(21);

            ---------------- 22: coverage = clamp(cw - q, 0, 16)
            if s_st(20) = '1' then                                -- the y sample
                case to_integer(xc(21)) is
                    when 0 => v_cw := c_cwy0; when 1 => v_cw := c_cwy1; when others => v_cw := c_cwy2;
                end case;
            else
                case to_integer(xc(21)) is
                    when 0 => v_cw := c_cwx0; when 1 => v_cw := c_cwx1; when others => v_cw := c_cwx2;
                end case;
            end if;
            v_c := signed(resize(v_cw, 8)) - signed(resize(s21_qx, 8));
            if v_c < 0 then s22_cvx <= (others => '0'); elsif v_c > 16 then s22_cvx <= to_unsigned(16, 5);
            else s22_cvx <= unsigned(v_c(4 downto 0)); end if;
            v_c := signed(resize(c_cwc, 8)) - signed(resize(s21_qc, 8));
            if v_c < 0 then s22_cvc <= (others => '0'); elsif v_c > 16 then s22_cvc <= to_unsigned(16, 5);
            else s22_cvc <= unsigned(v_c(4 downto 0)); end if;
            -- 23: tone = cov x class tone (Grey 1024 / 640 / 384, Ink 1024)
            if s_ink = '1' or xc(22) = 0 then s23_tx <= resize(s22_cvx & "000000", 11);
            elsif xc(22) = 1 then s23_tx <= resize(s22_cvx & "00000", 11) + resize(s22_cvx & "000", 11);
            else s23_tx <= resize(s22_cvx & "0000", 11) + resize(s22_cvx & "000", 11); end if;
            s23_tc <= resize(s22_cvc & "000000", 11);
            s23_cx <= xc(22); s23_ac <= gac(22);
            -- 24: the fade's start tone and weight: decade lines max(t, M af / 16) faded by ad; the rest by max(af, ad)
            if s23_cx = 0 then
                if s_ink = '1' then v_a := resize(xaf(23) & "0000000", 15);                          -- 512 af / 4
                else v_a := resize(xaf(23) & "000000", 15) + resize(xaf(23) & "0000", 15); end if;    -- 320 af / 4
                if resize(s23_tx, 15) > v_a then s24_tpx <= s23_tx; else s24_tpx <= v_a(10 downto 0); end if;
                s24_apx <= xad(23);
            else
                s24_tpx <= s23_tx;
                if xaf(23) > xad(23) then s24_apx <= xaf(23); else s24_apx <= xad(23); end if;
            end if;
            s24_tc <= s23_tc; s24_ac <= s23_ac;
            -- 25: M - t
            s25_dx <= signed(resize(c_M, 12)) - signed(resize(s24_tpx, 12));
            s25_dc <= signed(resize(c_Mc, 12)) - signed(resize(s24_tc, 12));
            s25_tpx <= s24_tpx; s25_tc <= s24_tc; s25_apx <= s24_apx; s25_ac <= s24_ac;
            -- 26: (M - t) x weight (0 .. 4 quarters), two partial sums
            v_s := (others => '0');
            if s25_apx(0) = '1' then v_s := v_s + resize(s25_dx, 17); end if;
            if s25_apx(1) = '1' then v_s := v_s + shift_left(resize(s25_dx, 17), 1); end if;
            s26_px <= v_s(14 downto 0);
            if s25_apx(2) = '1' then s26_qx <= shift_left(resize(s25_dx, 15), 2); else s26_qx <= (others => '0'); end if;
            v_s := (others => '0');
            if s25_ac(0) = '1' then v_s := v_s + resize(s25_dc, 17); end if;
            if s25_ac(1) = '1' then v_s := v_s + shift_left(resize(s25_dc, 17), 1); end if;
            s26_pc <= v_s(14 downto 0);
            if s25_ac(2) = '1' then s26_qc <= shift_left(resize(s25_dc, 15), 2); else s26_qc <= (others => '0'); end if;
            s26_tpx <= s25_tpx; s26_tc <= s25_tc;
            -- 27: t + ((M - t) w) >> 2
            v_s := resize(shift_right(s26_px + s26_qx, 2), 17) + signed(resize(s26_tpx, 17));
            s27_tx <= unsigned(v_s(10 downto 0));
            v_s := resize(shift_right(s26_pc + s26_qc, 2), 17) + signed(resize(s26_tc, 17));
            s27_tc <= unsigned(v_s(10 downto 0));
        end if;
    end process p_grad;

    ---------------------------------------------------------------------------
    -- [E] COMPOSE (28 .. 33) and [F] OUTPUT CONDITIONING (34 .. 41): luma
    -- [1 2 1]/4, 13/16 toward black, blanking gate; U = V = 512
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_u  : unsigned(15 downto 0);
        variable v_w  : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            -- 28: paper = max(x rulings, this line's y rulings)
            if s27_tx > l_ty then s28_pp <= s27_tx; else s28_pp <= l_ty; end if;
            s28_tc <= s27_tc;
            -- 29: Grey: c5 = (tc + 32) >> 6, 1024 - paper;  Ink: paper + tc
            v_u := resize(s28_tc, 16) + 32;
            s29_c5 <= v_u(10 downto 6);
            s29_dp <= to_unsigned(1024, 11) - s28_pp;
            s29_s <= resize(s28_pp, 12) + resize(s28_tc, 12);
            s29_pp <= s28_pp;
            -- 30: Grey partials (1024 - paper) c5;  Ink: triangle
            v_u := (others => '0');
            if s29_c5(0) = '1' then v_u := v_u + resize(s29_dp, 16); end if;
            if s29_c5(1) = '1' then v_u := v_u + resize(s29_dp & '0', 16); end if;
            if s29_c5(2) = '1' then v_u := v_u + resize(s29_dp & "00", 16); end if;
            s30_a <= v_u;
            v_u := (others => '0');
            if s29_c5(3) = '1' then v_u := v_u + resize(s29_dp & "000", 16); end if;
            if s29_c5(4) = '1' then v_u := v_u + resize(s29_dp & "0000", 16); end if;
            s30_b <= v_u;
            if s_ink = '1' then
                if s29_s <= 1024 then s30_pp <= s29_s(10 downto 0);
                else v_y := to_unsigned(2048, 12) - s29_s; s30_pp <= v_y(10 downto 0); end if;
            else
                s30_pp <= s29_pp;
            end if;
            -- 31
            if s_ink = '1' then s31_o <= s30_pp;
            else v_u := shift_right(s30_a + s30_b, 4); s31_o <= s30_pp + v_u(10 downto 0); end if;
            -- 32: Y = 64 + out - out/8 - out/64 - out/256  (876 / 1024)
            v_w := resize(s31_o, 21) - resize(shift_right(s31_o, 3), 21) - resize(shift_right(s31_o, 6), 21)
                   - resize(shift_right(s31_o, 8), 21);
            s32_y <= resize(v_w, 10) + 64;
            -- 33, 34: delay; 35: [1 2 1] taps for column j = pixels j (s34), j + 1 (s33), j + 2 (s32)
            s33_y <= s32_y; s34_y <= s33_y;
            s35_fa <= resize(s32_y, 12) + resize(s34_y, 12) + 2;
            s35_fb <= s33_y;
            -- 36: [1 2 1] / 4
            v_y := s35_fa + shift_left(resize(s35_fb, 12), 1);
            s36_y <= v_y(11 downto 2);
            -- 37: 13/16 toward black
            if s36_y > 64 then v_dl := s36_y - 64; else v_dl := (others => '0'); end if;
            s37_y <= s36_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- 38: padding to the one latency; 39: blanking gate (the generated avid, delayed C_LAT - 1)
            s38_y <= s37_y;
            if sy_f(2) = '1' then s39_y <= s38_y; else s39_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s39_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= o_hs;
    data_out.vsync_n <= o_vs;
    data_out.field_n <= s_field_q;
    data_out.avid    <= o_av;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> gain table -> 64-px box
    -- (edge-replicated, reloaded at every line start) -> vertical one-pole at
    -- quarter resolution (the first line of a field loads it)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(15 downto 0);
        variable v_dl : signed(16 downto 0);
        variable v_n : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then   -- both phases restart from this field's drift
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
            end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_osc = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid;
            -- gain table (p_gtram), then its fabric copy: s_tv is the sample of column s_col
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_tv <= gt_q;
            if s_v2 = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- horizontal one-pole (1/32 per pixel, Q12.4), loaded with the first sample of each line
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_hq <= s_tv(11 downto 0) & "0000";
            elsif s_v2 = '1' then
                s_hq <= unsigned(signed(s_hq) + resize(shift_right(signed(resize(s_tv(11 downto 0) & "0000", 17)) - signed(resize(s_hq, 17)), 5), 16));
            end if;
            s_v3 <= s_v2; s_c2 <= s_col;
            s_v4 <= s_v3; s_bcol <= s_c2; s_hq4 <= s_hq;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif s_ld = '1' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- vertical one-pole on the columns 3 mod 4: q += (box - q) >> sh (6 at 1080p, 5 at 720p / 1080i,
            -- 4 at SD), Q12.4
            s_eb <= s_hq4;
            s_eq <= lb_q1;
            if s_bcol(1 downto 0) = "11" then s_ev <= s_v4; else s_ev <= '0'; end if;
            s_ecol <= s_bcol; s_efl <= s_fl;
            s_dd <= signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            if s_efl = '1' then s_fq <= s_eb; else s_fq <= s_eq; end if;
            s_fv <= s_ev; s_fcol <= s_ecol; s_ffl <= s_efl;
            if s_ffl = '1' then v_dl := (others => '0');
            else v_dl := shift_right(s_dd, to_integer(c_iir)); end if;
            v_n := signed(resize(s_fq, 18)) + resize(v_dl, 18);
            lb_wd <= unsigned(v_n(15 downto 0));
            lb_we <= s_fv; lb_wa <= s_fcol(10 downto 2);

            -- gain table fill (zone 260 .. 516): T(i) = (i G6) >> 10 by accumulation
            if s_zone = '1' and s_zc = 260 then
                gt_run <= '1'; gt_i <= (others => '0'); gt_acc <= (others => '0');
            elsif gt_run = '1' then
                gt_acc <= gt_acc + resize(c_G6, 22); gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

end architecture decade;
