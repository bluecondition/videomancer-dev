-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Dazzle v0.1.0 -- Hardline.  First World War dazzle camouflage (Norman Wilkinson's ship schemes; Edward
-- Wadsworth, "Dazzle-ships in Drydock at Liverpool", 1919): the screen cut into hard-edged shards, each
-- filled with bold stripes that run against its neighbours'.  Black and white only (U = V = 512).
--
--   cut k (k < Cuts):  s_k = cos(a_k) (x - X) + sin(a_k) (y - Y) - Scatter * dist_k      a_k = base_k + rate_k * Turn
--   code   = the sign bits of the s_k; a ROM (designed offline so that flipping any ONE bit turns the stripes
--            by at least 26 degrees) gives the shard's fill: phase = (U or U/2 or 0) +- (V or V/2 or 0) + q/4,
--            U = (x - X) / pitch, V = (y - Y) / pitch: stripes at 0, 90, +-45, +-26.6, +-63.4 degrees at
--            pitches x0.71 .. x2 of Period
--   ink    = |frac(phase + camera) - 1/2| < weight / 2, anti-aliased analytically from the phase gradient
--            (the camera's own gradient included); Checks = stripes XOR the perpendicular stripes
--   cuts   = the pixel blends toward the shard across the nearest cut by its sub-pixel distance
--
--   K1  X        the pivot the cuts fan around, +-1536 virtual px                    (glided)
--   K2  Y        +-1024 virtual px                                                   (glided)
--   K3  Period   stripe pitch 12 .. 200 virtual px, exponential                      (glided)
--   K4  Cuts     2 .. 8 cuts (stepped, hysteresis)
--   K5  Turn     swings every cut about the pivot at its own rate (x1, x-1, x2, x-2, x3, x-3, x4, x-4;
--                x1 = +-45 degrees), centre = the base arrangement                   (glided)
--   K6  Camera   camera (or Osc) luma added to the stripe phase, 0 .. 4 periods      (glided)
--   S7  Glide    Off = the knob value of this field, On = 1/16 of the gap per field
--   S8  Fill     Stripes / Checks
--   S9  Angles   Free / Square (cuts snapped to 45 degree steps, stripes 0 / 45 / 90 / 135 only)
--   S10 Weight   Bold (equal bands) / Fine (1/6 duty, at least 2 px)
--   S11 Source   Video / Osc
--   P12 SCATTER  0 = every cut through the pivot (a pinwheel), 100 % = each cut pushed off the pivot by its
--                own distance (up to 0.6 screen heights)                             (glided)
--
-- Sections (the integer model .hardline_work/rb_dazzle/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide, Cuts hysteresis, switches
--   [C] FIELD SEQUENCER                  one serial multiplier, 9 x 8 slots of 24 clocks: pitch, pivot, then per
--                                        cut sin / cos (interpolated quarter-wave ROM), the DDA step, wrap and seed
--   [D] SOURCE FIELD                     Osc lattice, gain table, 32-px box, vertical one-pole in a line store
--   [E] CUTS                             8 affine DDAs (pixel units, Q14), nearest-cut distance, shard code
--   [F] FILL                             fill ROM, phases, analytic gradient, coverage divider, cut blend
--   [G] OUTPUT CONDITIONING              luma [1 2 1]/4, luma 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  The cut DDAs hold DDA pixel i at T + 6 + i; a register "at stage S" holds
-- pixel i at T + 6 + i + S; the coverage is stage 19, the luma stage 21, the output register stage 26.
-- Output column j is the [1 2 1] filter centred on DDA pixel j + 1.  One latency (32) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.dazzle_pkg.all;
use work.all;

architecture dazzle of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
    constant C_DEF_K5  : integer := 512;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 640;

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
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- interlaced modes (field lines are two pixels tall): 480i, 576i, 1080i
    function f_il(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others             => return '0';
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- a + b or a - b on ONE carry chain (the low column makes the carry-in)
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;
    -- luma of a coverage 0 .. 16: 64 + (o * 876 + 8) / 16
    function f_luma(o : unsigned(4 downto 0)) return unsigned is
    begin
        case to_integer(o) is
            when 0      => return to_unsigned(64, 10);
            when 1      => return to_unsigned(119, 10);
            when 2      => return to_unsigned(174, 10);
            when 3      => return to_unsigned(228, 10);
            when 4      => return to_unsigned(283, 10);
            when 5      => return to_unsigned(338, 10);
            when 6      => return to_unsigned(393, 10);
            when 7      => return to_unsigned(447, 10);
            when 8      => return to_unsigned(502, 10);
            when 9      => return to_unsigned(557, 10);
            when 10     => return to_unsigned(612, 10);
            when 11     => return to_unsigned(666, 10);
            when 12     => return to_unsigned(721, 10);
            when 13     => return to_unsigned(776, 10);
            when 14     => return to_unsigned(831, 10);
            when 15     => return to_unsigned(885, 10);
            when others => return to_unsigned(940, 10);
        end case;
    end function;
    -- a phase component: 0, the cycle fraction of a, or of a / 2 (a = cycles, Q1.24)
    function f_sel(s : unsigned(1 downto 0); a : unsigned(24 downto 0)) return unsigned is
    begin
        if s = "01" then return a(23 downto 12);
        elsif s = "10" then return a(24 downto 13);
        else return to_unsigned(0, 12); end if;
    end function;
    -- the same choice for a gradient (cycles per pixel, Q12)
    function f_selg(s : unsigned(1 downto 0); g : unsigned(10 downto 0)) return unsigned is
    begin
        if s = "01" then return g;
        elsif s = "10" then return '0' & g(10 downto 1);
        else return to_unsigned(0, 11); end if;
    end function;
    -- |frac - 1/2| of a phase (Q12): 0 .. 2048
    function f_af(p : unsigned(11 downto 0)) return unsigned is
    begin
        if p(11) = '1' then return '0' & p(10 downto 0);
        else return to_unsigned(2048, 12) - resize(p(10 downto 0), 12); end if;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ilr, s_il : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 4) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: +-1 hold (10 bits), glide (Q3)
    signal h_x : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal h_y : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal h_p : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal h_t : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal h_c : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal h_s : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal g_x : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal g_y : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal g_p : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal g_t : unsigned(12 downto 0) := to_unsigned(C_DEF_K5 * 8, 13);
    signal g_c : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);
    signal g_s : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(13 downto 0) := (others => '0');
    signal s_gr, s_gh : unsigned(9 downto 0) := (others => '0');
    signal z_gp, z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_chk, s_sq, s_fine, s_src : std_logic := '0';
    signal s_zn  : unsigned(2 downto 0) := to_unsigned(4, 3);          -- Cuts - 2
    signal z_v7  : unsigned(12 downto 0) := to_unsigned(4606, 13);
    signal z_up, z_dn : std_logic := '0';
    signal s_act : std_logic_vector(7 downto 0) := "00111111";

    -- [C] ROMs
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q, sn_f : unsigned(15 downto 0) := (others => '0');

    -- [C] sequencer and its serial multiplier
    signal q_run, q_go, q_cut : std_logic := '0';
    signal q_k  : unsigned(3 downto 0) := (others => '0');
    signal q_ck, q_j : unsigned(2 downto 0) := (others => '0');
    signal q_ph : unsigned(4 downto 0) := (others => '0');
    signal f_a, f_acc : signed(39 downto 0) := (others => '0');
    signal z_fa : signed(23 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_shl  : unsigned(2 downto 0) := (others => '0');
    signal z_sc   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_rq   : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal z_m    : unsigned(11 downto 0) := to_unsigned(1928, 12);
    signal z_w48  : unsigned(13 downto 0) := to_unsigned(7688, 14);
    signal z_apx, z_apy : unsigned(15 downto 0) := (others => '0');
    signal z_ra   : unsigned(2 downto 0) := (others => '0');
    signal z_rn   : std_logic := '0';
    signal z_m1, z_m2 : signed(16 downto 0) := (others => '0');
    signal z_th   : unsigned(15 downto 0) := (others => '0');
    signal z_p    : unsigned(14 downto 0) := (others => '0');
    signal z_ov, z_ov1, z_sneg, z_cneg : std_logic := '0';
    signal z_fs, z_fc : unsigned(3 downto 0) := (others => '0');
    signal z_sm, z_cm : unsigned(14 downto 0) := (others => '0');
    signal z_sdf, z_cdf : unsigned(5 downto 0) := (others => '0');
    signal z_dy, z_seed : signed(27 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_invp : unsigned(19 downto 0) := (others => '0');      -- cycles per h, Q24
    signal c_iu   : unsigned(21 downto 0) := (others => '0');      -- cycles per pixel, Q24
    signal c_iv   : unsigned(22 downto 0) := (others => '0');      -- cycles per line, Q24
    signal z_pv   : signed(16 downto 0) := (others => '0');        -- the pivot in pixel widths, Q3 (x, then y)
    signal c_pxn, c_pyn : std_logic := '0';                        -- its signs; the magnitudes are z_apx, z_apy
    signal c_sd   : unsigned(12 downto 0) := (others => '0');      -- Scatter, scaled to the raster
    signal c_u0, c_v0 : unsigned(24 downto 0) := (others => '0');  -- phase at DDA pixel 0 / line 0 (Q1.24)

    -- [E] cut table: (cut, 0) = the pixel step, (cut, 1) = the line wrap, (cut, 2) = the field seed
    type t_cr is array (0 to 255) of signed(31 downto 0);
    signal cr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_ra, cr_wa : unsigned(4 downto 0) := (others => '0');
    signal cr_q, cr_wd : signed(31 downto 0) := (others => '0');
    signal cr_we : std_logic := '0';

    -- [E] cut DDAs and their loader
    type t_s28x8 is array (0 to 7) of signed(27 downto 0);
    signal ca_inc, ca_acc : t_s28x8 := (others => (others => '0'));
    signal ca_ld, ca_en : std_logic_vector(7 downto 0) := (others => '0');
    signal ca_rst : std_logic := '0';
    signal l_kick, l_run, l_seed : std_logic := '0';
    signal l_t : unsigned(3 downto 0) := (others => '0');
    signal l_v1, l_v2, l_o1, l_o2 : std_logic := '0';
    signal l_c1, l_c2 : unsigned(2 downto 0) := (others => '0');
    signal b_run, b_end, b_d6q : std_logic := '0';
    signal b_cnt, b_mm1 : unsigned(11 downto 0) := (others => '0');
    signal b_d : std_logic_vector(1 to 6) := (others => '0');
    signal b_U, b_V : unsigned(24 downto 0) := (others => '0');

    -- [E] nearest cut
    type t_u4x8 is array (0 to 7) of unsigned(3 downto 0);
    signal x1_v : t_u4x8 := (others => "1000");
    signal x1_sg : std_logic_vector(7 downto 0) := (others => '0');
    type t_u4x4 is array (0 to 3) of unsigned(3 downto 0);
    signal x2_m : t_u4x4 := (others => "1000");
    signal x3_ma, x3_mb : unsigned(3 downto 0) := "1000";
    signal x2_code, x2_mask, x3_code, x3_mask : std_logic_vector(7 downto 0) := (others => '0');
    type t_cc is array (4 to 19) of unsigned(3 downto 0);
    signal x_sm : t_cc := (others => "1000");

    -- [F] fill ROM (two copies: this shard, the shard across the nearest cut)
    signal fa_a, fb_a : unsigned(8 downto 0) := (others => '0');
    signal fa_q, fb_q, x6_fa, x6_fb : unsigned(7 downto 0) := (others => '0');
    signal c_gu, c_gv : unsigned(10 downto 0);                     -- the stripe gradient per pixel / per line, Q12
    signal x7_p1, x7_p2, x7_q1, x7_q2, x7_ada, x7_adb : unsigned(11 downto 0) := (others => '0');
    signal x7_gx1, x7_gy1, x7_gx2, x7_gy2 : signed(12 downto 0) := (others => '0');
    signal x8_p1, x8_p2, x8_q1, x8_q2 : unsigned(11 downto 0) := (others => '0');
    signal x8_g1, x8_g2 : unsigned(12 downto 0) := (others => '0');
    signal x9_w1, x9_w2, x10_w1, x10_w2 : unsigned(12 downto 0) := (others => '0');
    signal x9_d1, x9_d2, x10_t1, x10_t2 : signed(12 downto 0) := (others => '0');
    signal x10_pos : std_logic := '0';
    signal x13_d : signed(12 downto 0) := (others => '0');
    signal x13_w, x14_w, x15_w, x14_wh : unsigned(12 downto 0) := (others => '0');
    signal x14_d  : signed(12 downto 0) := (others => '0');
    signal x15_e  : signed(14 downto 0) := (others => '0');
    type t_dv13 is array (0 to 4) of unsigned(12 downto 0);
    signal dv_r, dv_w : t_dv13 := (others => (others => '0'));
    type t_dv4 is array (0 to 4) of unsigned(3 downto 0);
    signal dv_q : t_dv4 := (others => (others => '0'));
    signal dv_z, dv_f : std_logic_vector(0 to 4) := (others => '0');
    signal x21_cov, x22_o : unsigned(4 downto 0) := (others => '0');
    signal x_bb : std_logic_vector(9 to 19) := (others => '0');

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
    signal gt_q, s_tv, s_tvd, s_sfirst, gt_wd : unsigned(15 downto 0) := (others => '0');
    signal gt_wa, gt_i : unsigned(7 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_acc : unsigned(21 downto 0) := (others => '0');
    signal s_col, s_c2, s_bcol, s_ecol, s_fcol : unsigned(10 downto 0) := (others => '0');
    signal bx_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_f : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_b64 : std_logic := '0';                                  -- 64-px box on the 1920-wide rasters
    signal s_df   : signed(16 downto 0) := (others => '0');
    signal s_bsum : unsigned(21 downto 0) := (others => '0');
    signal s_ev, s_fv : std_logic := '0';
    signal s_eb, s_eq, s_fq : unsigned(15 downto 0) := (others => '0');
    signal s_dd : signed(16 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(15 downto 0) := (others => '0');
    type t_ly is array (0 to 1023) of signed(7 downto 0);
    signal ly_ram : t_ly := (others => (others => '0'));
    attribute no_rw_check of ly_ram : signal is true;
    signal ly_q2, ly_wd, x_a0, x6_ly : signed(7 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');
    signal x_q0, x_q1, x_q2 : unsigned(15 downto 0) := (others => '0');
    signal x6_lp : unsigned(11 downto 0) := (others => '0');
    signal x6_lx : signed(11 downto 0) := (others => '0');

    -- [G] output conditioning
    signal s_y23, s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y26, s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    c_gu <= '0' & c_iu(21 downto 12);
    c_gv <= c_iv(22 downto 12);

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_DZ_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_DZ_SIN(to_integer(sn_a));
            sn_f   <= sn_q;
        end if;
    end process p_rom;

    p_fill_a : process(clk)
    begin
        if rising_edge(clk) then
            fa_q <= C_DZ_FILL(to_integer(fa_a));
        end if;
    end process p_fill_a;
    p_fill_b : process(clk)
    begin
        if rising_edge(clk) then
            fb_q <= C_DZ_FILL(to_integer(fb_a));
        end if;
    end process p_fill_b;

    -- cut table: written by the sequencer in the zone, read by the loader (kick, line ends)
    p_crram : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= cr_ram(to_integer(cr_ra));
            if cr_we = '1' then cr_ram(to_integer(cr_wa)) <= cr_wd; end if;
        end if;
    end process p_crram;

    -- gain table: written in the zone, read by the source path
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(s_sl));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
        end if;
    end process p_gtram;

    -- box delay: read 30 (or 62) samples behind the write (+ two registers = 32 or 64)
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_v2 = '1' then bx_ram(to_integer(bx_wp)) <= s_tv; end if;
        end if;
    end process p_bxram;

    -- source line store (half horizontal resolution): port 1 = the one-pole's own read (ahead of its
    -- write), port 2 = the pixel path (behind it)
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_c2(10 downto 1)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;
    -- the one-pole's step per line (the camera's vertical phase gradient), same addresses
    p_lyram : process(clk)
    begin
        if rising_edge(clk) then
            ly_q2 <= ly_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then ly_ram(to_integer(lb_wa)) <= ly_wd; end if;
        end if;
    end process p_lyram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the field sequencer
    -- (one small operation per zone clock; the zone is quasi-static but timed at
    -- the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k   : unsigned(9 downto 0);
        variable v_rate : signed(3 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_lo, v_hi : unsigned(13 downto 0);
        variable v_e   : signed(11 downto 0);
        variable v_s14 : signed(13 downto 0);
        variable v_a, v_b : signed(16 downto 0);
        variable v_sh  : signed(39 downto 0);
        variable v_t   : signed(27 downto 0);
        variable v_u   : unsigned(24 downto 0);
        variable v_r   : unsigned(15 downto 0);
        variable v_i   : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_ilr <= f_il(s_tid);

            -- [A] vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- [A] width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_chk   <= registers_in(6)(1);
                s_sq    <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_il    <= s_ilr;
                if s_xsr = 2 then s_b64 <= '1'; else s_b64 <= '0'; end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); z_sc <= to_unsigned(16384, 16);
                    when 3      => s_oxs <= to_unsigned(153, 9); z_sc <= to_unsigned(21845, 16);
                    when others => s_oxs <= to_unsigned(102, 9); z_sc <= to_unsigned(32768, 16);
                end case;
                -- Cuts: 7 steps with hysteresis (enter a step only 8 counts past its edge; the two ends
                -- are reached because the band lies on the entry side)
                if z_up = '1' or z_dn = '1' then s_zn <= z_v7(12 downto 10); end if;
            end if;
            -- (the comparison itself runs free, one step per clock, so no knob carry chain sits on the latch)
            v_k := unsigned(registers_in(3));
            z_v7 <= (v_k & "000") - resize(v_k, 13);
            v_lo := resize(s_zn & "0000000000", 14);
            v_hi := v_lo + 1080;
            if resize(z_v7, 14) >= v_hi then z_up <= '1'; else z_up <= '0'; end if;
            if resize(z_v7, 14) + 56 < v_lo then z_dn <= '1'; else z_dn <= '0'; end if;
            g_wm1 <= s_w - 1;
            b_mm1 <= s_w + 7;
            z_m   <= s_w + 8;
            z_w48 <= (s_w & "00") + 8;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            case to_integer(s_xs) is               -- line height in pixel widths, Q10
                when 4      => z_rq <= resize(s_ys & '0', 12);
                when 3      => z_rq <= to_unsigned(1024, 12);
                when others => z_rq <= resize(s_ys & "00", 12);
            end case;
            case to_integer(s_zn) is
                when 0      => s_act <= "00000011";
                when 1      => s_act <= "00000111";
                when 2      => s_act <= "00001111";
                when 3      => s_act <= "00011111";
                when 4      => s_act <= "00111111";
                when 5      => s_act <= "01111111";
                when others => s_act <= "11111111";
            end case;

            -- [B] shared hold + glide, five clocks per channel.  Hold: the knob moves the held value only by
            -- 2 counts or more (or to either end).  Glide On: 1/16 of the gap per field, never slower than
            -- one step (1/8 count), landing exactly.  Glide Off: the held value, this field.
            q_go <= '0';
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= g_x; s_gh <= h_x; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= g_y; s_gh <= h_y; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= g_p; s_gh <= h_p; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= g_t; s_gh <= h_t; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= g_c; s_gh <= h_c; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= g_s; s_gh <= h_s; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "001";
                    when 1 =>
                        v_d := signed(resize(s_gr, 11)) - signed(resize(s_gh, 11));
                        if s_gr = 0 or s_gr = 1023 or v_d > 1 or v_d < -1 then s_gh <= s_gr; end if;
                        z_gp <= "010";
                    when 2 =>
                        s_gd <= signed(resize(s_gh & "000", 14)) - signed(resize(s_gm, 14));
                        z_gp <= "011";
                    when 3 =>
                        if s_glide = '1' then
                            if s_gd > 16 or s_gd < -16 then s_gd <= shift_right(s_gd, 4);
                            elsif s_gd > 0 then s_gd <= to_signed(1, 14);
                            elsif s_gd < 0 then s_gd <= to_signed(-1, 14);
                            end if;
                        end if;
                        z_gp <= "100";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => g_x <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_x <= s_gh;
                            when 1      => g_y <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_y <= s_gh;
                            when 2      => g_p <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_p <= s_gh;
                            when 3      => g_t <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_t <= s_gh;
                            when 4      => g_c <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_c <= s_gh;
                            when others => g_s <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_s <= s_gh;
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; q_go <= '1'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] the sequencer: slots of 24 clocks; q_k = 0 holds the 8 global slots, q_k = 1 .. 8 the 8 slots of
            -- cut q_ck = q_k - 1.  Phase 0 pre-muxes the operands, 1 loads the multiplier, 2 .. 17 iterate, the
            -- product is read at 19.
            l_kick <= '0';
            if q_go = '1' then
                q_run <= '1'; q_k <= (others => '0'); q_ck <= (others => '0'); q_cut <= '0';
                q_j <= (others => '0'); q_ph <= (others => '0');
            elsif q_run = '1' then
                if q_ph = 23 then
                    q_ph <= (others => '0');
                    q_j <= q_j + 1;
                    if q_j = 7 then
                        q_k <= q_k + 1; q_ck <= q_k(2 downto 0); q_cut <= '1';
                        if q_k = 8 then q_run <= '0'; l_kick <= '1'; end if;
                    end if;
                else
                    q_ph <= q_ph + 1;
                end if;
            end if;

            -- operand pre-mux (phase 0)
            if q_run = '1' and q_ph = 0 then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                if q_cut = '0' then
                    case to_integer(q_j) is
                        when 0      => z_fa <= signed(resize(g_p, 24));    z_fb <= to_unsigned(1040, 16);     -- octaves
                        when 1      => z_fa <= signed(resize(c_invp, 24)); z_fb <= resize(s_ys, 16);          -- cycles per line
                        when 2      =>                                                                        -- pivot x: 3 virtual px per count
                            v_s14 := signed(resize(g_x, 14)) - 4096;                                          -- (4 v - v, never v + 2 v: a self-add)
                            z_fa <= resize(shift_left(resize(v_s14, 16), 2) - resize(v_s14, 16), 24); z_fb <= z_sc;
                        when 3      =>                                                                        -- pivot y: 2 virtual px per count
                            v_s14 := signed(resize(g_y, 14)) - 4096;
                            z_fa <= resize(shift_left(resize(v_s14, 16), 1) + 4320, 24); z_fb <= z_sc;
                        when 4      => z_fa <= signed(resize(g_s, 24));    z_fb <= z_sc;                      -- scatter
                        when 5      => z_fa <= signed(resize(c_iu, 24));   z_fb <= z_apx;                     -- phase at pixel 0
                        when 6      => z_fa <= signed(resize(c_iu, 24));   z_fb <= z_apy;                     -- phase at line 0
                        when others => null;
                    end case;
                else
                    case to_integer(q_j) is
                        when 1      => z_fa <= signed(resize(z_sdf, 24));  z_fb <= resize(z_fs, 16);          -- sine interpolation
                        when 2      => z_fa <= signed(resize(z_cdf, 24));  z_fb <= resize(z_fc, 16);          -- cosine
                        when 3      => z_fa <= signed(resize(z_rq, 24));   z_fb <= resize(z_sm, 16);          -- line step
                        when 4      => z_fa <= signed(resize(z_apx, 24));  z_fb <= resize(z_cm, 16);          -- cos * |pivot x|
                        when 5      => z_fa <= signed(resize(z_apy, 24));  z_fb <= resize(z_sm, 16);          -- sin * |pivot y|
                        when 6      => z_fa <= resize(C_DZ_DIST(to_integer(q_ck)), 24); z_fb <= resize(c_sd, 16);   -- scatter distance
                        when 7      => z_fa <= signed(resize(z_m, 24));    z_fb <= resize(z_cm, 16);          -- steps per line * cos
                        when others => null;
                    end case;
                end if;
            end if;
            if q_ph = 1 then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 40); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- results and the small steps between the products
            cr_we <= '0';
            if q_run = '1' and q_cut = '0' then
                case to_integer(q_j) is
                    when 0 =>
                        case to_integer(q_ph) is
                            when 19 => z_oct <= unsigned(f_acc(23 downto 13));
                            when 20 =>
                                v_e := to_signed(106, 12) - signed(resize(z_oct, 12));
                                r_ex_a <= unsigned(v_e(7 downto 0));
                                z_shl <= unsigned(v_e(10 downto 8)) + 4;
                            when 23 =>
                                v_i := resize(r_exf, 20);
                                case to_integer(z_shl) is
                                    when 4      => c_invp <= shift_left(v_i, 4);
                                    when 3      => c_invp <= shift_left(v_i, 3);
                                    when 2      => c_invp <= shift_left(v_i, 2);
                                    when 1      => c_invp <= shift_left(v_i, 1);
                                    when others => c_invp <= v_i;
                                end case;
                            when others => null;
                        end case;
                    when 1 =>
                        if q_ph = 19 then
                            c_iv <= unsigned(f_acc(29 downto 7));
                            case to_integer(s_xs) is
                                when 4      => c_iu <= shift_left(resize(c_invp, 22), 2);
                                when 3      => c_iu <= shift_left(resize(c_invp, 22), 1) + resize(c_invp, 22);
                                when others => c_iu <= shift_left(resize(c_invp, 22), 1);
                            end case;
                        end if;
                    when 2 =>
                        if q_ph = 19 then
                            z_pv <= signed(resize(z_w48, 17)) + f_acc(31 downto 15);
                        elsif q_ph = 20 then
                            c_pxn <= z_pv(16);
                            if z_pv(16) = '1' then z_apx <= unsigned(-z_pv(15 downto 0)); else z_apx <= unsigned(z_pv(15 downto 0)); end if;
                        end if;
                    when 3 =>
                        if q_ph = 19 then
                            z_pv <= f_acc(31 downto 15);
                        elsif q_ph = 20 then
                            c_pyn <= z_pv(16);
                            if z_pv(16) = '1' then z_apy <= unsigned(-z_pv(15 downto 0)); else z_apy <= unsigned(z_pv(15 downto 0)); end if;
                        end if;
                    when 4 =>
                        if q_ph = 19 then c_sd <= unsigned(f_acc(27 downto 15)); end if;
                    when 5 =>
                        if q_ph = 19 then
                            v_u := unsigned(f_acc(27 downto 3));
                            c_u0 <= f_addsub(to_unsigned(0, 25), v_u, not c_pxn);
                        end if;
                    when 6 =>
                        if q_ph = 19 then
                            v_u := unsigned(f_acc(27 downto 3));
                            c_v0 <= f_addsub(to_unsigned(0, 25), v_u, not c_pyn);
                        end if;
                    when others => null;
                end case;
            elsif q_run = '1' then
                case to_integer(q_j) is
                    when 0 =>                      -- the cut's angle, then sine and cosine from the quarter-wave ROM
                        case to_integer(q_ph) is
                            when 1 =>
                                v_rate := C_DZ_RATE(to_integer(q_ck));
                                z_rn <= v_rate(3);
                                if v_rate(3) = '1' then z_ra <= unsigned(-v_rate(2 downto 0)); else z_ra <= unsigned(v_rate(2 downto 0)); end if;
                                v_s14 := signed(resize(g_t, 14)) - 4096;           -- Turn about its centre: x1 = +-45 degrees
                                z_m1 <= shift_left(resize(v_s14, 17), 1);
                            when 2 =>              -- |rate| * turn: one add of gated operands per step
                                if z_ra(0) = '1' then v_a := z_m1; else v_a := (others => '0'); end if;
                                if z_ra(1) = '1' then v_b := shift_left(z_m1, 1); else v_b := (others => '0'); end if;
                                z_m2 <= v_a + v_b;
                            when 3 =>
                                if z_ra(2) = '1' then v_b := shift_left(z_m1, 2); else v_b := (others => '0'); end if;
                                z_m2 <= z_m2 + v_b;
                            when 4 =>
                                z_th <= f_addsub(C_DZ_BASE(to_integer(q_ck)), unsigned(z_m2(15 downto 0)), z_rn);
                            when 5 =>              -- Square: snap to 45 degrees
                                if s_sq = '1' then
                                    v_r := z_th + 4096;
                                    z_th <= v_r(15 downto 13) & "0000000000000";
                                end if;
                            when 6 =>              -- sine: quadrant fold
                                z_sneg <= z_th(15);
                                if z_th(14) = '1' then z_p <= to_unsigned(16384, 15) - resize(z_th(13 downto 0), 15);
                                else z_p <= resize(z_th(13 downto 0), 15); end if;
                            when 7 =>
                                sn_a <= z_p(13 downto 4); z_ov <= z_p(14); z_fs <= z_p(3 downto 0);
                                if z_p(14) = '1' or z_p(13 downto 4) = 1023 then z_ov1 <= '1'; else z_ov1 <= '0'; end if;
                            when 8 =>
                                sn_a <= sn_a + 1;
                            when 10 =>
                                if z_ov = '1' then z_sm <= to_unsigned(16384, 15); else z_sm <= sn_f(14 downto 0); end if;
                            when 11 =>
                                if z_ov = '1' then z_sdf <= (others => '0');
                                elsif z_ov1 = '1' then z_sdf <= resize(to_unsigned(16384, 15) - z_sm, 6);
                                else z_sdf <= resize(sn_f(14 downto 0) - z_sm, 6); end if;
                            when 12 =>             -- cosine = sine a quarter turn on
                                v_r := z_th + 16384;
                                z_cneg <= v_r(15);
                                if v_r(14) = '1' then z_p <= to_unsigned(16384, 15) - resize(v_r(13 downto 0), 15);
                                else z_p <= resize(v_r(13 downto 0), 15); end if;
                            when 13 =>
                                sn_a <= z_p(13 downto 4); z_ov <= z_p(14); z_fc <= z_p(3 downto 0);
                                if z_p(14) = '1' or z_p(13 downto 4) = 1023 then z_ov1 <= '1'; else z_ov1 <= '0'; end if;
                            when 14 =>
                                sn_a <= sn_a + 1;
                            when 16 =>
                                if z_ov = '1' then z_cm <= to_unsigned(16384, 15); else z_cm <= sn_f(14 downto 0); end if;
                            when 17 =>
                                if z_ov = '1' then z_cdf <= (others => '0');
                                elsif z_ov1 = '1' then z_cdf <= resize(to_unsigned(16384, 15) - z_cm, 6);
                                else z_cdf <= resize(sn_f(14 downto 0) - z_cm, 6); end if;
                            when others => null;
                        end case;
                    when 1 =>
                        if q_ph = 19 then z_sm <= z_sm + resize(unsigned(f_acc(10 downto 4)) + ("000000" & f_acc(3)), 15); end if;
                    when 2 =>
                        if q_ph = 19 then z_cm <= z_cm + resize(unsigned(f_acc(10 downto 4)) + ("000000" & f_acc(3)), 15); end if;
                    when 3 =>
                        if q_ph = 19 then
                            v_t := signed(resize(unsigned(f_acc(37 downto 10)), 28));
                            z_dy <= signed(f_addsub(to_unsigned(0, 28), unsigned(v_t), z_sneg));
                        end if;
                    when 4 =>
                        if q_ph = 19 then
                            v_t := f_acc(30 downto 3);
                            z_seed <= signed(f_addsub(to_unsigned(0, 28), unsigned(v_t), not (z_cneg xor c_pxn)));
                        end if;
                    when 5 =>
                        if q_ph = 19 then
                            v_t := f_acc(30 downto 3);
                            z_seed <= signed(f_addsub(unsigned(z_seed), unsigned(v_t), not (z_sneg xor c_pyn)));
                        end if;
                    when 6 =>
                        if q_ph = 19 then
                            v_t := f_acc(25 downto 0) & "00";
                            z_seed <= z_seed - v_t;
                        end if;
                    when others =>
                        case to_integer(q_ph) is
                            when 20 =>             -- the pixel step: +-cos
                                cr_we <= '1'; cr_wa <= q_ck & "00";
                                cr_wd <= resize(signed(f_addsub(to_unsigned(0, 28), resize(z_cm, 28), z_cneg)), 32);
                            when 21 =>             -- the wrap: line step - steps * pixel step
                                cr_we <= '1'; cr_wa <= q_ck & "01";
                                cr_wd <= resize(signed(f_addsub(unsigned(z_dy), unsigned(f_acc(27 downto 0)), not z_cneg)), 32);
                            when 22 => cr_we <= '1'; cr_wa <= q_ck & "10"; cr_wd <= resize(z_seed, 32);
                            when others => null;
                        end case;
                end case;
            end if;

            -- [A] sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the
            -- core on every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 3);
        end if;
    end process p_ctrl;

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
    -- [E] CUTS.  Eight affine DDAs in pixel widths (Q13.14): each adds its own
    -- increment register on every clock of the line window (W + 8 steps), then
    -- once more with the line wrap parked in that register (wrap = line step
    -- - steps * pixel step), so the adder has no input mux.  The increments
    -- come from the cut table one word per clock.  The kick (end of the field
    -- sequencer) clears the accumulators and adds the field seed the same way.
    ---------------------------------------------------------------------------
    p_load : process(clk)
    begin
        if rising_edge(clk) then
            -- line window
            b_end <= '0';
            if s_st(4) = '1' then
                b_run <= '1'; b_cnt <= (others => '0');
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = b_mm1 then b_run <= '0'; b_end <= '1'; end if;
            end if;
            b_d <= b_run & b_d(1 to 5);
            b_d6q <= b_d(6);

            -- loader: 16 table reads (cut, wrap or seed), (cut, step)
            ca_rst <= l_kick;
            if l_kick = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '1';
            elsif b_end = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '0';
            elsif l_run = '1' then
                l_t <= l_t + 1;
                if l_t = 15 then l_run <= '0'; end if;
            end if;
            if l_t(0) = '1' then cr_ra <= l_t(3 downto 1) & "00";
            elsif l_seed = '1' then cr_ra <= l_t(3 downto 1) & "10";
            else cr_ra <= l_t(3 downto 1) & "01"; end if;
            -- the word addressed now is in cr_q two clocks on: it goes into the cut's increment register
            -- then (ca_ld), and a wrap / seed word is added one clock after that (ca_en), on the same edge
            -- that puts the pixel step back
            l_v1 <= l_run; l_c1 <= l_t(3 downto 1); l_o1 <= l_t(0);
            l_v2 <= l_v1;  l_c2 <= l_c1;            l_o2 <= l_o1;
            for k in 0 to 7 loop
                if l_v1 = '1' and l_c1 = k then ca_ld(k) <= '1'; else ca_ld(k) <= '0'; end if;
                if l_v2 = '1' and l_o2 = '0' and l_c2 = k then ca_en(k) <= '1'; else ca_en(k) <= '0'; end if;
            end loop;

            -- stripe phases: U runs along the line, V steps once per line
            if b_d(6) = '1' then b_U <= b_U + resize(c_iu, 25); else b_U <= c_u0; end if;
            if l_kick = '1' then b_V <= c_v0;
            elsif b_d6q = '1' and b_d(6) = '0' then b_V <= b_V + resize(c_iv, 25); end if;
        end if;
    end process p_load;

    g_cut : for k in 0 to 7 generate
        p_cut : process(clk)
            variable v_m : std_logic_vector(26 downto 0);
            variable v_n : std_logic;
        begin
            if rising_edge(clk) then
                if ca_ld(k) = '1' then ca_inc(k) <= cr_q(27 downto 0); end if;
                if ca_rst = '1' then
                    ca_acc(k) <= (others => '0');
                elsif b_run = '1' or ca_en(k) = '1' then
                    ca_acc(k) <= ca_acc(k) + ca_inc(k);
                end if;
                -- stage 1: side, and the distance to the cut as 0 .. 7 (sixteenths of a pixel; eighths when
                -- interlaced) or 8 = not near
                for i in 0 to 26 loop v_m(i) := ca_acc(k)(i) xor ca_acc(k)(27); end loop;
                if s_il = '1' then
                    if unsigned(v_m(26 downto 14)) = 0 then v_n := '1'; else v_n := '0'; end if;
                else
                    if unsigned(v_m(26 downto 13)) = 0 then v_n := '1'; else v_n := '0'; end if;
                end if;
                if s_act(k) = '0' then
                    x1_sg(k) <= '0'; x1_v(k) <= "1000";
                else
                    x1_sg(k) <= not ca_acc(k)(27);
                    if v_n = '0' then x1_v(k) <= "1000";
                    elsif s_il = '1' then x1_v(k) <= '0' & unsigned(v_m(13 downto 11));
                    else x1_v(k) <= '0' & unsigned(v_m(12 downto 10)); end if;
                end if;
            end if;
        end process p_cut;
    end generate g_cut;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> gain table (K6: phase in
    -- Q3.13 cycles) -> box of 32 px (64 px on the 1920-wide rasters), edge-
    -- replicated and reloaded at every line start -> vertical one-pole (1/32
    -- per line; 1/64 at 1080p, 1/16 at 480i / 576i) in the line store at half
    -- horizontal resolution; the step per line is stored beside it.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(15 downto 0);
        variable v_dl : signed(16 downto 0);
        variable v_n : signed(17 downto 0);
        variable v_qs : unsigned(16 downto 0);
        variable v_qd : signed(16 downto 0);
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
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);                           -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid;
            -- gain table (p_gtram), then its fabric copy: s_tv is the sample of column s_col
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_tv <= gt_q;
            if s_v2 = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 (64) samples; the samples before the line start count as the first one
            if s_v2 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            elsif s_b64 = '1' then bx_wp <= (others => '0'); bx_rp <= to_unsigned(194, 8);
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(226, 8); end if;
            bx_f <= bx_q;
            if s_bcnt(6) = '0' and (s_b64 = '1' or s_bcnt(5) = '0') then v_tap := s_sfirst; else v_tap := bx_f; end if;
            s_df <= signed(resize(s_tv, 17)) - signed(resize(v_tap, 17));
            s_tvd <= s_tv; s_v3 <= s_v2; s_c2 <= s_col;
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_sfirst <= s_tv;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_v2 = '1' then
                if s_bcnt(6) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_ld = '1' then
                if s_b64 = '1' then s_bsum <= shift_left(resize(s_tvd, 22), 6);
                else s_bsum <= shift_left(resize(s_tvd, 22), 5); end if;
            elsif s_v3 = '1' then
                s_bsum <= unsigned(signed(s_bsum) + resize(s_df, 22));
            end if;
            s_v4 <= s_v3; s_bcol <= s_c2;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif s_ld = '1' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- vertical one-pole on the odd columns: q += (box - q) >> 5 (6 at 1080p, 4 at 480i / 576i)
            if s_b64 = '1' then s_eb <= s_bsum(21 downto 6); else s_eb <= s_bsum(20 downto 5); end if;
            s_eq <= lb_q1;
            s_ev <= s_v4 and s_bcol(0); s_ecol <= s_bcol; s_efl <= s_fl;
            s_dd <= signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            if s_efl = '1' then s_fq <= s_eb; else s_fq <= s_eq; end if;
            s_fv <= s_ev; s_fcol <= s_ecol; s_ffl <= s_efl;
            if s_ffl = '1' then v_dl := (others => '0');
            elsif s_b64 = '1' and s_il = '0' then v_dl := shift_right(s_dd, 6);
            elsif s_b64 = s_il then v_dl := shift_right(s_dd, 5);
            else v_dl := shift_right(s_dd, 4); end if;
            v_n := signed(resize(s_fq, 18)) + resize(v_dl, 18);
            lb_wd <= unsigned(v_n(15 downto 0));
            if v_dl(16 downto 10) /= "0000000" and v_dl(16 downto 10) /= "1111111" then
                if v_dl(16) = '1' then ly_wd <= to_signed(-128, 8); else ly_wd <= to_signed(127, 8); end if;
            else
                ly_wd <= v_dl(10 downto 3);
            end if;
            lb_we <= s_fv; lb_wa <= s_fcol(10 downto 1);

            -- gain table fill (zone, after the glides): T(i) = (i * K6) >> 6 (Osc: >> 5) by accumulation
            gt_we <= '0';
            if q_go = '1' then
                gt_run <= '1'; gt_i <= (others => '0'); gt_acc <= (others => '0');
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_i;
                if s_src = '1' then gt_wd <= gt_acc(20 downto 5); else gt_wd <= gt_acc(21 downto 6); end if;
                gt_acc <= gt_acc + resize(g_c, 22); gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;

            -- pixel path: the store's column counter, three fabric copies of the read, and at stage 6 the
            -- interpolated phase (Q12), its horizontal slope and the stored vertical slope
            if b_d(5) = '1' then lb_rc <= lb_rc + 1; else lb_rc <= (others => '0'); end if;
            x_q0 <= lb_q2; x_q1 <= x_q0; x_q2 <= x_q1;
            x_a0 <= ly_q2;
            v_qs := resize(x_q0, 17) + resize(x_q1, 17);
            x6_lp <= v_qs(13 downto 2);
            v_qd := signed(resize(x_q0, 17)) - signed(resize(x_q2, 17));
            x6_lx <= v_qd(13 downto 2);
            x6_ly <= x_a0;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] nearest cut (stages 2 .. 4) and [F] FILL (stages 4 .. 21)
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_ua, v_vb, v_q : unsigned(1 downto 0);
        variable v_sg : std_logic;
        variable v_g  : signed(12 downto 0);
        variable v_ax, v_ay : unsigned(12 downto 0);
        variable v_w5 : unsigned(15 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_dh : unsigned(10 downto 0);
        variable v_bs, v_bd : unsigned(12 downto 0);
        variable v_mg : signed(12 downto 0);
        variable v_b1, v_b2 : std_logic;
        variable v_cc, v_cn : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: minimum of pairs; the code and the near-cut mask
            for p in 0 to 3 loop
                if x1_v(2 * p + 1) < x1_v(2 * p) then x2_m(p) <= x1_v(2 * p + 1); else x2_m(p) <= x1_v(2 * p); end if;
            end loop;
            x2_code <= x1_sg;
            for k in 0 to 7 loop x2_mask(k) <= not x1_v(k)(3); end loop;
            -- 3
            if x2_m(1) < x2_m(0) then x3_ma <= x2_m(1); else x3_ma <= x2_m(0); end if;
            if x2_m(3) < x2_m(2) then x3_mb <= x2_m(3); else x3_mb <= x2_m(2); end if;
            x3_code <= x2_code; x3_mask <= x2_mask;
            -- 4: the distance to the nearest cut; ROM addresses of this shard and of the one across
            if x3_mb < x3_ma then x_sm(4) <= x3_mb; else x_sm(4) <= x3_ma; end if;
            for i in 5 to 19 loop x_sm(i) <= x_sm(i - 1); end loop;
            fa_a <= s_sq & unsigned(x3_code);
            fb_a <= s_sq & unsigned(x3_code xor x3_mask);
            -- 5: the ROMs' own registers; 6: fabric copies
            x6_fa <= fa_q; x6_fb <= fb_q;

            -- 7: phase = (U part) +- (V part), and the perpendicular phase for Checks; the camera term;
            -- the phase gradient per pixel and per line with the camera's slopes
            v_ua := x6_fa(1 downto 0); v_vb := x6_fa(3 downto 2); v_sg := x6_fa(4); v_q := x6_fa(6 downto 5);
            x7_p1 <= f_addsub(f_sel(v_ua, b_U), f_sel(v_vb, b_V), v_sg);
            x7_p2 <= f_addsub(f_sel(v_vb, b_U), f_sel(v_ua, b_V), not v_sg);
            x7_ada <= x6_lp + (v_q & "0000000000");
            x7_gx1 <= signed(resize(f_selg(v_ua, c_gu), 13)) + resize(x6_lx, 13);
            x7_gx2 <= signed(resize(f_selg(v_vb, c_gu), 13)) + resize(x6_lx, 13);
            v_g := resize(x6_ly & "00", 13);
            x7_gy1 <= signed(f_addsub(unsigned(v_g), resize(f_selg(v_vb, c_gv), 13), v_sg));
            x7_gy2 <= signed(f_addsub(unsigned(v_g), resize(f_selg(v_ua, c_gv), 13), not v_sg));
            v_ua := x6_fb(1 downto 0); v_vb := x6_fb(3 downto 2); v_sg := x6_fb(4); v_q := x6_fb(6 downto 5);
            x7_q1 <= f_addsub(f_sel(v_ua, b_U), f_sel(v_vb, b_V), v_sg);
            x7_q2 <= f_addsub(f_sel(v_vb, b_U), f_sel(v_ua, b_V), not v_sg);
            x7_adb <= x6_lp + (v_q & "0000000000");

            -- 8: the four phases; |gradient| = |d/dx| + |d/dy| (cycles per pixel, Q12)
            x8_p1 <= x7_p1 + x7_ada; x8_p2 <= x7_p2 + x7_ada;
            x8_q1 <= x7_q1 + x7_adb; x8_q2 <= x7_q2 + x7_adb;
            if x7_gx1(12) = '1' then v_ax := unsigned(-x7_gx1); else v_ax := unsigned(x7_gx1); end if;
            if x7_gy1(12) = '1' then v_ay := unsigned(-x7_gy1); else v_ay := unsigned(x7_gy1); end if;
            x8_g1 <= v_ax + v_ay;
            if x7_gx2(12) = '1' then v_ax := unsigned(-x7_gx2); else v_ax := unsigned(x7_gx2); end if;
            if x7_gy2(12) = '1' then v_ay := unsigned(-x7_gy2); else v_ay := unsigned(x7_gy2); end if;
            x8_g2 <= v_ax + v_ay;

            -- 9: the signed distance to the band edge, d = half-width - |frac - 1/2| (positive inside the ink), on
            -- one carry chain: (half-width) - frac' in the upper half cycle, (half-width - 1/2) + frac' in the
            -- lower.  The half-width is never under one pixel of gradient; the transition width is widened past
            -- 1/4 cycle per pixel so stripes fade to grey before they alias.  The shard across the cut is
            -- evaluated without anti-aliasing (the sign of the same sum).
            if x8_g1(12 downto 10) /= "000" then
                v_w5 := shift_left(resize(x8_g1, 16), 2) + resize(x8_g1, 16) - 4096;
                if v_w5(15 downto 13) /= "000" then x9_w1 <= (others => '1'); else x9_w1 <= v_w5(12 downto 0); end if;
                v_dh := to_unsigned(1024, 11);
            else
                x9_w1 <= x8_g1;
                if s_fine = '0' then v_dh := to_unsigned(1024, 11);
                elsif x8_g1(9 downto 0) > 341 then v_dh := '0' & x8_g1(9 downto 0);
                else v_dh := to_unsigned(341, 11); end if;
            end if;
            v_bs := (not x8_p1(11)) & (not x8_p1(11)) & v_dh;
            x9_d1 <= signed(f_addsub(v_bs, resize(x8_p1(10 downto 0), 13), x8_p1(11)));
            if x8_g2(12 downto 10) /= "000" then
                v_w5 := shift_left(resize(x8_g2, 16), 2) + resize(x8_g2, 16) - 4096;
                if v_w5(15 downto 13) /= "000" then x9_w2 <= (others => '1'); else x9_w2 <= v_w5(12 downto 0); end if;
                v_dh := to_unsigned(1024, 11);
            else
                x9_w2 <= x8_g2;
                if s_fine = '0' then v_dh := to_unsigned(1024, 11);
                elsif x8_g2(9 downto 0) > 341 then v_dh := '0' & x8_g2(9 downto 0);
                else v_dh := to_unsigned(341, 11); end if;
            end if;
            v_bs := (not x8_p2(11)) & (not x8_p2(11)) & v_dh;
            x9_d2 <= signed(f_addsub(v_bs, resize(x8_p2(10 downto 0), 13), x8_p2(11)));
            -- across the cut: ink where |frac - 1/2| < the nominal half-width, i.e. (half-width - 1) - |..| >= 0
            if s_fine = '1' then v_dh := to_unsigned(340, 11); else v_dh := to_unsigned(1023, 11); end if;
            v_bs := (not x8_q1(11)) & (not x8_q1(11)) & v_dh;
            v_bd := f_addsub(v_bs, resize(x8_q1(10 downto 0), 13), x8_q1(11));
            v_b1 := not v_bd(12);
            v_bs := (not x8_q2(11)) & (not x8_q2(11)) & v_dh;
            v_bd := f_addsub(v_bs, resize(x8_q2(10 downto 0), 13), x8_q2(11));
            v_b2 := not v_bd(12);
            if s_chk = '0' then x_bb(9) <= v_b1;
            elsif s_fine = '1' then x_bb(9) <= v_b1 or v_b2;
            else x_bb(9) <= v_b1 xor v_b2; end if;
            for i in 10 to 19 loop x_bb(i) <= x_bb(i - 1); end loop;

            -- 10: Bold Checks (the XOR of the two stripe sets) works on the magnitudes
            if s_chk = '1' and s_fine = '0' and x9_d1(12) = '1' then x10_t1 <= -x9_d1; else x10_t1 <= x9_d1; end if;
            if s_chk = '1' and s_fine = '0' and x9_d2(12) = '1' then x10_t2 <= -x9_d2; else x10_t2 <= x9_d2; end if;
            x10_pos <= x9_d1(12) xor x9_d2(12);
            x10_w1 <= x9_w1; x10_w2 <= x9_w2;
            -- 11: the shard's distance and transition width.  Stripes: the first phase.  Fine Checks (a grid:
            -- the union) the larger distance.  Bold Checks the nearer edge, inside where exactly one set inks.
            if s_chk = '0' then
                x13_d <= x10_t1; x13_w <= x10_w1;
            elsif s_fine = '1' then
                if x10_t2 > x10_t1 then x13_d <= x10_t2; x13_w <= x10_w2; else x13_d <= x10_t1; x13_w <= x10_w1; end if;
            else
                if x10_t2 < x10_t1 then v_mg := x10_t2; x13_w <= x10_w2; else v_mg := x10_t1; x13_w <= x10_w1; end if;
                if x10_pos = '1' then x13_d <= v_mg; else x13_d <= -v_mg; end if;
            end if;

            -- 12 .. 19: coverage = clamp(16 (d + w / 2) / w + 1/2) by a 4-step divider
            x14_d <= x13_d;
            x14_wh <= shift_right(x13_w, 1) + shift_right(x13_w, 5);
            x14_w <= x13_w;
            x15_e <= resize(x14_d, 15) + signed(resize(x14_wh, 15));
            x15_w <= x14_w;
            dv_z(0) <= x15_e(14);
            if x15_e >= signed(resize(x15_w, 15)) then dv_f(0) <= '1'; else dv_f(0) <= '0'; end if;
            dv_r(0) <= unsigned(x15_e(12 downto 0));
            dv_w(0) <= x15_w;
            for i in 1 to 4 loop
                v_t := dv_r(i - 1) & '0';
                if v_t >= resize(dv_w(i - 1), 14) then
                    dv_r(i) <= resize(v_t - resize(dv_w(i - 1), 14), 13);
                    dv_q(i) <= dv_q(i - 1)(2 downto 0) & '1';
                else
                    dv_r(i) <= v_t(12 downto 0);
                    dv_q(i) <= dv_q(i - 1)(2 downto 0) & '0';
                end if;
                dv_w(i) <= dv_w(i - 1);
                dv_z(i) <= dv_z(i - 1); dv_f(i) <= dv_f(i - 1);
            end loop;
            if dv_z(4) = '1' then x21_cov <= (others => '0');
            elsif dv_f(4) = '1' then x21_cov <= to_unsigned(16, 5);
            else x21_cov <= '0' & dv_q(4); end if;

            -- 20: the cut: this shard covers cc / 16 of the pixel, the shard across (ink or ground) the rest
            v_cc := to_unsigned(8, 5) + resize(x_sm(19), 5);
            v_cn := to_unsigned(8, 5) - resize(x_sm(19), 5);
            if x_bb(19) = '1' then
                if x21_cov > v_cn then x22_o <= x21_cov; else x22_o <= v_cn; end if;
            else
                if x21_cov < v_cc then x22_o <= x21_cov; else x22_o <= v_cc; end if;
            end if;
            -- 21
            s_y23 <= f_luma(x22_o);
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [G] OUTPUT CONDITIONING: luma [1 2 1]/4, luma 13/16 toward black, blanking
    -- gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd1 <= s_y23; s_yd2 <= s_yd1;
            s_ya <= resize(s_y23, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y26 <= v_y(11 downto 2);
            if s_y26 > 64 then v_dl := s_y26 - 64; else v_dl := (others => '0'); end if;
            s_y27 <= s_y26 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y28 <= s_y27;
            else
                s_y28 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture dazzle;
