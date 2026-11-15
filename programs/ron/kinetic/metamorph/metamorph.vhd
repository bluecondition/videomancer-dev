-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Metamorph v0.1.0 -- Kinetic.  After M. C. Escher's "Metamorphosis II", "Sky and Water I" and "Day and
-- Night": a checkerboard whose edges deform by degrees until the squares are interlocking figures.
--
-- Element: a tile.  State: the morph amount A, a continuous field that the wave carries across the picture.
--
--   X' = X + A f(Y)        Y' = Y - A f(X)         (X, Y in tiles from the screen centre; f = the edge profile,
--                                                    one tile long, so the tiles always tessellate)
--   colour = parity(floor X') xor parity(floor Y')
--   A      = Morph + Depth * wave(theta) + source   (signed: a negative A turns the tiles the other way)
--   theta  = Spread * X + phase  (Linear)   /   Spread * (|X| + |Y|) + phase - 1/4  (Radial: diamond rings)
--   AA     = each edge family's distance to its nearest edge, in 1/8 pixels: |frac| / |grad| without a
--            divider (leading one of the gradient, a 4-entry reciprocal, a window of the distance); the
--            gradient is the finite difference along the line plus |A| * |f(Y) - f(Y of the line above)|;
--            9 coverage levels, XOR blend of the two families
--   Detail = an echo contour and a diamond eye in the tile's own (deformed) coordinates, in the other colour
--   Outline= an accent line on every edge, from the same pixel distances
--
--   K1  Scale    tile pitch 64 .. 430 virtual px (exponential, glided)
--   K2  Shape    the edge profile: S-curve > bird hook > zigzag > jigsaw knob, morphing in eighths
--   K3  Depth    the wave's swing of A (square law, 0 .. 1/2 tile)
--   K4  Rate     wave travel, bipolar, centre = still
--   K5  Spread   phase step per tile (square law, 0 .. 1/2 cycle)
--   K6  Colour   the 16 shared Kinetic palettes (ground, A; the accent on the outline)
--   S7  Wave     Smooth / Snap (dwell at the extremes)
--   S8  Field    Linear (the metamorphosis sweeps across) / Radial (rings from the centre)
--   S9  Outline  Off / On
--   S10 Detail   Off / On (echo contour + eye)
--   S11 Source   Video / Osc (luma^2 adds to A: the tiles deform where the picture is bright)
--   P12 MORPH    base A: 0 % a checkerboard .. 100 % deep interlock
--
-- Chassis from riley.vhd ([A] timing and virtual coordinates, [B] controls, the per-field sequencer's
-- serial multiplier, [D] source field, [F] output conditioning); the wave engine is reduced to first-order
-- DDAs (two along the line, two down the screen, all counting down from positive start values) and the
-- geometry is new.  The integer model .kinetic_work/rb_metamorph/imodel.py mirrors every stage.
-- Stage numbering as in riley: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).
--   5, 6 theta   7 .. 11 wave -> Depth table   12, 13 A   14, 15 A * f (hi / lo partial products)   16 X', Y'
--   17 .. 19 gradients, edge distances   20 .. 22 distance in 1/8 px   23 soft bits, outline
--   24 .. 27 XOR blends (edges, then detail)   28 the outline's share   29 .. 31 colour tables, sum
--   32 .. 35 output conditioning (p_out; its comments keep riley's stage numbers 26 .. 29).
--   Detail: 17 .. 22 ring / eye distance, 23 .. 25 coverage.   One latency (42) in every mode.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.metamorph_pkg.all;
use work.all;

architecture metamorph of program_top is
    constant C_LAT : integer := 42;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 341;
    constant C_DEF_K3  : integer := 600;
    constant C_DEF_K5  : integer := 420;
    constant C_DEF_P12 : integer := 350;

    constant C_CY8 : integer := 138240;     -- screen centre y, Q8 virtual lines (540)
    -- detail geometry in the tile's own coordinates (Q12 tile units, from the tile centre)
    constant C_RE  : integer := 1344;       -- echo contour: Chebyshev radius
    constant C_EX  : integer := 448;        -- eye centre
    constant C_EY  : integer := -384;
    constant C_RHO : integer := 380;        -- eye radius (diamond)

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

    attribute no_rw_check : boolean;

    -- Pixel distance without a divider: q = 8 n / w (1/8 px, clipped to 31) from the leading one of w, the two
    -- bits below it (a 4-entry reciprocal: 14, 12, 10, 9 sixteenths) and a window of n.
    -- f_pe: leading-one position of w (4 bits) & the two bits below it
    function f_pe(w : unsigned(10 downto 0)) return unsigned is
        variable v_w : unsigned(12 downto 0);
        variable v_r : unsigned(5 downto 0);
    begin
        v_w := w & "00";
        v_r := (others => '0');
        for e in 0 to 10 loop
            if w(e) = '1' then v_r := to_unsigned(e, 4) & v_w(e + 1 downto e); end if;
        end loop;
        return v_r;
    end function;
    -- f_win: (n * 8) >> e as 6 bits, bit 6 = it does not fit
    function f_win(n : unsigned(12 downto 0); e : unsigned(3 downto 0)) return unsigned is
        variable v_n : unsigned(26 downto 0);
        variable v_r : unsigned(6 downto 0);
    begin
        v_n := "00000000000" & n & "000";
        v_r := (others => '0');
        for k in 0 to 10 loop
            if to_integer(e) = k then
                v_r(5 downto 0) := v_n(k + 5 downto k);
                if v_n(k + 16 downto k + 6) /= 0 then v_r(6) := '1'; end if;
            end if;
        end loop;
        return v_r;
    end function;
    -- f_rq: the window times the reciprocal, clipped to 31
    function f_rq(x : unsigned(6 downto 0); mt : unsigned(1 downto 0)) return unsigned is
        variable v_x, v_a, v_b, v_s : unsigned(9 downto 0);
        variable v_sub : std_logic;
    begin
        v_x := resize(x(5 downto 0), 10);
        v_sub := '0';
        case mt is
            when "00"   => v_a := shift_left(v_x, 4); v_b := shift_left(v_x, 1); v_sub := '1';
            when "01"   => v_a := shift_left(v_x, 3); v_b := shift_left(v_x, 2);
            when "10"   => v_a := shift_left(v_x, 3); v_b := shift_left(v_x, 1);
            when others => v_a := shift_left(v_x, 3); v_b := v_x;
        end case;
        v_s := f_addsub(v_a, v_b, v_sub);
        if x(6) = '1' or v_s(9) = '1' then return to_unsigned(31, 5); end if;
        return v_s(8 downto 4);
    end function;
    -- f_soft: coverage 0 .. 8 from the distance (1/8 px) and the side
    function f_soft(q : unsigned(4 downto 0); b : std_logic) return unsigned is
    begin
        if b = '1' then
            if q >= 4 then return to_unsigned(8, 4); else return to_unsigned(4, 4) + q(3 downto 0); end if;
        else
            if q >= 4 then return to_unsigned(0, 4); else return to_unsigned(4, 4) - q(3 downto 0); end if;
        end if;
    end function;
    -- f_oln: outline coverage 0 .. 8 = clamp(ol - q)
    function f_oln(q : unsigned(4 downto 0); ol : unsigned(4 downto 0)) return unsigned is
        variable v_o : unsigned(4 downto 0);
    begin
        v_o := ol - q;
        if q >= ol then return to_unsigned(0, 4); end if;
        if v_o >= 8 then return to_unsigned(8, 4); end if;
        return v_o(3 downto 0);
    end function;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    -- the beam's pre-pixel (one step left of x0), by video mode: its distance left of the centre (h), that
    -- squared, minus the first difference of dx^2 there, and the second difference
    signal w_dxm  : unsigned(10 downto 0);
    signal w_qx0  : unsigned(21 downto 0);
    signal w_d0m  : unsigned(13 downto 0);
    signal w_dd   : unsigned(5 downto 0);

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
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k): pixels 0 .. W - 1 at stage k + 2
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 8) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(C_DEF_K5 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_oln, s_det, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := (others => '0');

    -- [C] ROMs: C_MM_TAB (2^(i/256), then the palettes), C_MM_SIN (a quarter wave), C_MM_PRO (edge profiles)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q : unsigned(15 downto 0) := (others => '0');
    signal pr_a : unsigned(9 downto 0) := (others => '0');
    signal pr_q, pr_qf : signed(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_sh   : signed(3 downto 0) := (others => '0');
    signal z_sh4  : unsigned(2 downto 0) := (others => '0');
    signal z_s3   : unsigned(14 downto 0) := (others => '0');
    signal z_hw   : unsigned(11 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_invp : unsigned(20 downto 0) := (others => '0');      -- tiles per h (half virtual px), Q24
    signal c_sp   : unsigned(27 downto 0) := (others => '0');      -- spread, cycles per h, Q32
    signal c_D    : unsigned(7 downto 0) := (others => '0');       -- wave depth, A units (1/512 tile)
    signal c_base : unsigned(7 downto 0) := (others => '0');       -- base morph, A units
    signal c_sy   : unsigned(10 downto 0) := to_unsigned(64, 11);  -- tile units (Q12) per line
    signal c_hw   : unsigned(8 downto 0) := to_unsigned(88, 9);    -- echo line half-width, tile units
    signal c_rstep : unsigned(27 downto 0) := (others => '0');     -- phase step per field, Q32
    signal c_ph   : unsigned(31 downto 0) := (others => '0');      -- wave phase, Q32 cycles
    signal c_phl  : unsigned(19 downto 0) := (others => '0');      -- phase (Q20), a quarter back in Radial
    signal c_U0, c_Ud : unsigned(23 downto 0) := (others => '0');  -- -theta along x: pre-pixel value, step (Q4.20 cycles)
    signal c_T0, c_Td : unsigned(23 downto 0) := (others => '0');  -- -theta along y: line 0, step per line
    signal c_V0, c_Vd : unsigned(21 downto 0) := (others => '0');  -- -X: pre-pixel value, step (Q2.20 tiles)
    signal c_Y0, c_ystep : unsigned(21 downto 0) := (others => '0');   -- -Y: line 0, step per line
    signal w_ol : unsigned(4 downto 0);                            -- outline: 4 + 4 * width in pixels

    -- [E] per-field palette: ground, colour - ground (0 = A, 2 = accent)
    type t_s11x3 is array (0 to 2) of signed(10 downto 0);
    signal c_gy, c_gu, c_gv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_dy, c_du, c_dv : t_s11x3 := (others => (others => '0'));

    -- [C] line engine (two DDAs down the screen, the shape lookup for f(Y))
    signal e_lc   : unsigned(3 downto 0) := (others => '1');
    signal e_zk   : std_logic := '0';
    signal e_first : std_logic := '0';
    signal n_y    : unsigned(21 downto 0) := (others => '0');      -- next line: -Y (Q2.20 tiles)
    signal n_tv   : unsigned(23 downto 0) := (others => '0');      -- next line: -theta along y (signed)
    signal n_av, n_th, l_th : unsigned(19 downto 0) := (others => '0');
    signal n_fy   : signed(11 downto 0) := (others => '0');        -- next line: f(Y)
    signal n_dfy  : signed(12 downto 0) := (others => '0');        -- next line: f(Y) - f(Y of the line above)
    signal n_df6, l_df6 : unsigned(5 downto 0) := (others => '0'); -- |n_dfy| / 16, clamped
    signal l_y    : unsigned(13 downto 0) := (others => '0');      -- this line: Y (Q2.12)
    signal l_fy   : signed(9 downto 0) := (others => '0');         -- this line: f(Y) / 2

    -- [C] shape table: the blended edge profile, 256 points per tile, refilled every field (two copies: the
    -- pixel path reads f(X), the line engine reads f(Y))
    type t_sh is array (0 to 255) of signed(15 downto 0);
    signal sg_ram, sf_ram : t_sh := (others => (others => '0'));
    attribute no_rw_check of sg_ram : signal is true;
    attribute no_rw_check of sf_ram : signal is true;
    signal sg_q, sf_q, sf_qf, sh_wd : signed(15 downto 0) := (others => '0');
    signal sh_wa, sh_t : unsigned(7 downto 0) := (others => '0');
    signal sh_we, sh_run : std_logic := '0';
    signal sh_ph : unsigned(2 downto 0) := (others => '0');
    signal sh_i  : unsigned(1 downto 0) := (others => '0');
    signal sh_fr : unsigned(2 downto 0) := (others => '0');
    signal sh_va : signed(11 downto 0) := (others => '0');
    signal sh_vd : signed(12 downto 0) := (others => '0');
    signal sh_p1, sh_p2, sh_vs : signed(15 downto 0) := (others => '0');

    -- [C] Depth table: (D * i) >> 8 for the wave magnitude i, refilled every field by an accumulator
    type t_dt is array (0 to 255) of unsigned(7 downto 0);
    signal dt_ram : t_dt := (others => (others => '0'));
    attribute no_rw_check of dt_ram : signal is true;
    signal dt_q, dt_wd, dt_wa, dt_i : unsigned(7 downto 0) := (others => '0');
    signal dt_acc : unsigned(15 downto 0) := (others => '0');
    signal dt_we, dt_run : std_logic := '0';

    -- [E] colour tables, refilled every field: ta = ground -> A in 9 steps (Y U V), tc = the accent's share
    -- (signed dY dU dV) in 9 steps
    type t_tb is array (0 to 255) of unsigned(31 downto 0);
    signal ta_ram, tc_ram : t_tb := (others => (others => '0'));
    attribute no_rw_check of ta_ram : signal is true;
    attribute no_rw_check of tc_ram : signal is true;
    signal ta_q, tc_q, s30_ta, s30_tc : unsigned(31 downto 0) := (others => '0');
    signal ta_wd, tc_wd : unsigned(31 downto 0);
    signal tb_i : unsigned(3 downto 0) := (others => '0');
    signal tb_run : std_logic := '0';
    signal tb_ay, tb_au, tb_av, tb_cy, tb_cu, tb_cv : signed(14 downto 0) := (others => '0');

    -- [C] beam DDAs and the pixel path
    signal b_U : unsigned(23 downto 0) := (others => '0');
    signal b_V : unsigned(21 downto 0) := (others => '0');
    signal s6_th : unsigned(19 downto 0) := (others => '0');
    signal s7_neg, s8_neg, s9_neg, s10_neg, s11_neg : std_logic := '0';
    signal s9_mag, s11_dw, s12_src : unsigned(7 downto 0) := (others => '0');
    signal s12_a  : signed(10 downto 0) := (others => '0');
    signal s13_A  : signed(9 downto 0) := (others => '0');
    signal s13_g  : signed(9 downto 0) := (others => '0');
    signal s14_pyh, s14_pxh : signed(14 downto 0) := (others => '0');
    signal s14_pyl, s14_pxl : signed(15 downto 0) := (others => '0');
    signal s14_aa4 : unsigned(3 downto 0) := (others => '0');
    signal s15_dx, s15_dy : signed(13 downto 0) := (others => '0');
    signal s15_q  : unsigned(9 downto 0) := (others => '0');
    signal s12_x, s13_x, s14_x, s15_x : unsigned(13 downto 0) := (others => '0');
    signal s16_xp, s16_yp, s17_xp, s17_yp, s18_xp, s18_yp : unsigned(13 downto 0) := (others => '0');
    signal s16_gy, s17_gy, s18_gy, s18_agx, s18_agy : unsigned(10 downto 0) := (others => '0');
    signal s17_gxx, s17_gxy : signed(13 downto 0) := (others => '0');
    signal s19_wx, s19_wy, s19_nx, s19_ny, s20_nx, s20_ny : unsigned(10 downto 0) := (others => '0');
    signal s19_bx, s19_by, s20_bx, s20_by, s21_bx, s21_by, s22_bx, s22_by : std_logic := '0';
    signal s20_pex, s20_pey : unsigned(5 downto 0) := (others => '0');
    signal s21_xwx, s21_xwy : unsigned(6 downto 0) := (others => '0');
    signal s21_mtx, s21_mty : unsigned(1 downto 0) := (others => '0');
    signal s22_qx, s22_qy : unsigned(4 downto 0) := (others => '0');
    signal s23_px, s23_py, s23_olx, s23_oly : unsigned(3 downto 0) := (others => '0');
    signal s24_m, s26_m : unsigned(7 downto 0) := (others => '0');
    signal s24_s, s26_s : unsigned(4 downto 0) := (others => '0');
    signal s24_ol, s25_ol, s26_ol, s27_ol, s27_nol, s28_ol : unsigned(3 downto 0) := (others => '0');
    signal s25_p, s27_p, s28_pp : unsigned(3 downto 0) := (others => '0');

    -- detail: echo contour + eye
    signal s17_axc, s17_ayc, s18_m : unsigned(10 downto 0) := (others => '0');
    signal s17_ex, s17_ey : signed(12 downto 0) := (others => '0');
    signal s18_aex, s18_aey : unsigned(11 downto 0) := (others => '0');
    signal s19_es : unsigned(12 downto 0) := (others => '0');
    signal s18_sel, s19_sel : std_logic := '0';
    signal s19_mr : signed(11 downto 0) := (others => '0');
    signal s20_dr : signed(12 downto 0) := (others => '0');
    signal s20_de, s21_d : signed(13 downto 0) := (others => '0');
    signal s20_w  : unsigned(10 downto 0) := (others => '0');
    signal s21_pe, s22_pe : unsigned(5 downto 0) := (others => '0');
    signal s22_n  : unsigned(10 downto 0) := (others => '0');
    signal s22_sg, s23_sg, s24_sg : std_logic := '0';
    signal s23_xw : unsigned(6 downto 0) := (others => '0');
    signal s23_mt : unsigned(1 downto 0) := (others => '0');
    signal s24_dq : unsigned(4 downto 0) := (others => '0');
    signal s25_det : unsigned(3 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal s_col : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb8 : unsigned(7 downto 0) := (others => '0');
    signal s_eq : unsigned(11 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');
    signal s8_q, s8_qp, s9_l, s11_d8, s12_d8 : unsigned(7 downto 0) := (others => '0');
    -- weight table: duty / 256 by source level, refilled every field (a second-order DDA, no multiplier)
    type t_wt is array (0 to 255) of unsigned(7 downto 0);
    signal wt_ram : t_wt := (others => (others => '0'));
    attribute no_rw_check of wt_ram : signal is true;
    signal wt_q, wt_wd, wt_wa, wt_i : unsigned(7 downto 0) := (others => '0');
    signal wt_we, wt_run : std_logic := '0';
    signal wt_acc : unsigned(23 downto 0) := (others => '0');
    signal wt_d   : unsigned(17 downto 0) := (others => '0');

    -- [E] colour out (stage 31; the names p_out reads)
    signal s25_y, s25_u, s25_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cu, s_cv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_yd : t_u10(1 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv2, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4100, 13);
    signal s_hist : std_logic_vector(7 downto 0) := (others => '0');
    signal s26_ya : unsigned(11 downto 0) := (others => '0');
    signal s26_yb : unsigned(9 downto 0) := (others => '0');
    signal s27_y, s28_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s27_u, s27_v, s28_u, s28_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s29_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s29_u, s29_v : unsigned(9 downto 0) := to_unsigned(512, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_MM_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_MM_SIN(to_integer(sn_a));
            pr_q   <= C_MM_PRO(to_integer(pr_a));
            pr_qf  <= pr_q;
        end if;
    end process p_rom;

    -- Depth table: written in the vblank zone only, read by the pixel path (stage 9 address)
    p_dtram : process(clk)
    begin
        if rising_edge(clk) then
            dt_q <= dt_ram(to_integer(s9_mag));
            if dt_we = '1' then dt_ram(to_integer(dt_wa)) <= dt_wd; end if;
        end if;
    end process p_dtram;

    -- shape tables: written in the vblank zone only
    p_sgram : process(clk)
    begin
        if rising_edge(clk) then
            sg_q <= sg_ram(to_integer(b_V(19 downto 12)));
            if sh_we = '1' then sg_ram(to_integer(sh_wa)) <= sh_wd; end if;
        end if;
    end process p_sgram;
    p_sfram : process(clk)
    begin
        if rising_edge(clk) then
            sf_q <= sf_ram(to_integer(n_y(19 downto 12)));
            if sh_we = '1' then sf_ram(to_integer(sh_wa)) <= sh_wd; end if;
        end if;
    end process p_sfram;

    -- colour tables: written in the vblank zone only, read by the pixel path (stage 28 address)
    p_taram : process(clk)
    begin
        if rising_edge(clk) then
            ta_q <= ta_ram(to_integer(s28_pp));
            if tb_run = '1' then ta_ram(to_integer(tb_i)) <= ta_wd; end if;
        end if;
    end process p_taram;
    p_tcram : process(clk)
    begin
        if rising_edge(clk) then
            tc_q <= tc_ram(to_integer(s28_ol));
            if tb_run = '1' then tc_ram(to_integer(tb_i)) <= tc_wd; end if;
        end if;
    end process p_tcram;

    -- weight table: written in the vblank zone only, read by the pixel path
    p_wtram : process(clk)
    begin
        if rising_edge(clk) then
            wt_q <= wt_ram(to_integer(s9_l));
            if wt_we = '1' then wt_ram(to_integer(wt_wa)) <= wt_wd; end if;
        end if;
    end process p_wtram;

    -- box delay: read 32 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    -- luma line store (half horizontal resolution): port 1 = the IIR's own read (2 addresses ahead of its
    -- write), port 2 = the pixel path (2 or 3 addresses behind it)
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 1)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- [A] the beam's pre-pixel by video mode; [C] the six coefficients by Field / Axis
    --   Linear across: theta = f0 dx + q dx^2 + sp dy        Linear down: theta = sp dx + f0 dy + q dy^2
    --   Radial:        theta = q (dx^2 + dy^2) + (sp / 4) v   (v = dy across, dx down)
    ---------------------------------------------------------------------------
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);
    w_qx0 <= to_unsigned(2085136, 22) when s_xs = 4 else to_unsigned(3697929, 22) when s_xs = 3 else to_unsigned(3694084, 22);
    w_d0m <= to_unsigned(11536, 14) when s_xs = 4 else to_unsigned(11529, 14) when s_xs = 3 else to_unsigned(7684, 14);
    w_dd  <= to_unsigned(32, 6) when s_xs = 4 else to_unsigned(18, 6) when s_xs = 3 else to_unsigned(8, 6);

    ta_wd <= "00" & unsigned(tb_ay(12 downto 3)) & unsigned(tb_au(12 downto 3)) & unsigned(tb_av(12 downto 3));
    tc_wd <= '0' & unsigned(tb_cy(13 downto 3)) & unsigned(tb_cu(12 downto 3)) & unsigned(tb_cv(12 downto 3));
    w_ol <= to_unsigned(12, 5) when s_xs = 4 else to_unsigned(16, 5) when s_xs = 3 else to_unsigned(20, 5);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_c  : signed(10 downto 0);
        variable v_8  : unsigned(11 downto 0);
        variable v_r  : unsigned(27 downto 0);
        variable v_pi : unsigned(3 downto 0);
        variable v_pd : signed(10 downto 0);
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

            -- [B] rate: bipolar with a +-8 deadband
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

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_snap  <= registers_in(6)(0);
                s_rad  <= registers_in(6)(1);
                s_oln <= registers_in(6)(2);
                s_det <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- palette: 16 steps with hysteresis (leave a step only 8 past its edge)
                v_k := unsigned(registers_in(5));
                v_c := signed(resize(v_k, 11)) - signed(resize(s_pal & "100000", 11));
                if v_c > 40 or v_c < -40 then s_pal <= v_k(9 downto 6); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
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
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at 50 + 24 k).  The slot number is decoded at 30 + 24 k and the operands are
            -- pre-muxed into z_fa / z_fb at 31 + 24 k.  Slot 3 takes the previous product from the accumulator.
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
                    when 0  => z_fa <= resize(s_g1, 40);  z_fb <= to_unsigned(704, 16);                  -- octaves above pitch 64
                    when 1  => z_fa <= resize(c_invp, 40); z_fb <= resize(s_ys, 16);                     -- Y step per line
                    when 2  => z_fa <= resize(s_g5(12 downto 3), 40); z_fb <= resize(s_g5(12 downto 3), 16);  -- spread^2
                    when 3  => z_fa <= resize(c_invp, 40); z_fb <= f_acc(20 downto 5);                   -- spread per h
                    when 4  => z_fa <= resize(c_sp, 40);  z_fb <= resize(w_dxm, 16);                     -- theta at the pre-pixel
                    when 5  => z_fa <= resize(c_sp, 40);  z_fb <= resize(s_xs, 16);                      -- its step per pixel
                    when 6  => z_fa <= resize(s_g3, 40);  z_fb <= resize(s_g3, 16);                      -- depth^2
                    when 7  => z_fa <= resize(c_sp, 40);  z_fb <= resize(s_ys, 16);                      -- theta step per line
                    when 8  => z_fa <= resize(s_vm, 40);  z_fb <= to_unsigned(897, 16);                  -- rate exponent
                    when 9  => z_fa <= resize(r_exf, 40); z_fb <= to_unsigned(9321, 16);                 -- rate step
                    when 10 =>                                                                           -- ... << its octave
                        z_fa <= resize(f_acc(29 downto 8), 40);
                        z_fb <= shift_left(to_unsigned(1, 16), to_integer(z_sh4));
                    when 11 => z_fa <= resize(c_sp, 40);  z_fb <= to_unsigned(1080, 16);                 -- theta at line 0
                    when 12 => z_fa <= resize(c_invp, 40); z_fb <= resize(w_dxm, 16);                    -- X at the pre-pixel
                    when 13 => z_fa <= resize(c_invp, 40); z_fb <= resize(s_xs, 16);                     -- its step
                    when others => z_fa <= resize(c_invp, 40); z_fb <= to_unsigned(1080, 16);            -- Y at line 0
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20 =>                        -- motion (last field's step)
                        c_ph <= f_addsub(c_ph, resize(c_rstep, 32), s_vneg);
                    when 21 =>                        -- base morph: 0 .. 224 A units
                        c_base <= resize(shift_right(s_gp, 5), 8) - resize(shift_right(s_gp, 8), 8);
                    when 22 =>                        -- Radial starts a quarter cycle back: the centre at the trough
                        if s_rad = '1' then c_phl <= c_ph(31 downto 12) - x"40000";
                        else c_phl <= c_ph(31 downto 12); end if;
                    when 38 =>                        -- shape position: profile (2 bits) . eighths (3 bits)
                        z_s3 <= resize(s_g2 & '0', 15) + resize(s_g2, 15) + 512;

                    -- pitch: tiles per h = 2^17 * 2^-oct
                    when 50  => z_oct <= f_acc(23 downto 13);
                    when 51  =>
                        v_e := -signed(resize(z_oct, 12));
                        r_ex_a <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(11 downto 8);
                    when 54  =>
                        case to_integer(z_sh) is
                            when 0      => c_invp <= resize(r_exf & "00", 21);
                            when -1     => c_invp <= resize(r_exf & "0", 21);
                            when -2     => c_invp <= resize(r_exf, 21);
                            when others => c_invp <= resize(r_exf(15 downto 1), 21);
                        end case;
                    when 74  => c_sy <= f_acc(29 downto 19); c_ystep <= f_acc(32 downto 11);
                    when 75  => z_hw <= resize(c_sy, 12) + resize(shift_right(c_sy, 1), 12);
                    when 76  =>
                        if z_hw < 88 then c_hw <= to_unsigned(88, 9);
                        elsif z_hw > 320 then c_hw <= to_unsigned(320, 9);
                        else c_hw <= z_hw(8 downto 0); end if;
                    when 122 => c_sp <= f_acc(35 downto 8);
                    when 146 => c_U0 <= f_acc(35 downto 12);
                    when 170 => c_Ud <= f_acc(35 downto 12);
                    when 194 => c_D <= f_acc(25 downto 18);
                    when 218 => c_Td <= resize(f_acc(39 downto 19), 24);
                    when 242 => r_ex_a <= '0' & f_acc(15 downto 8); z_sh4 <= f_acc(18 downto 16);
                    when 290 =>
                        if s_vm = 0 then c_rstep <= (others => '0');
                        else c_rstep <= f_acc(27 downto 0); end if;
                    when 314 => c_T0 <= f_acc(35 downto 12);
                    when 338 => c_V0 <= f_acc(25 downto 4);
                    when 362 => c_Vd <= f_acc(25 downto 4);
                    when 386 => c_Y0 <= f_acc(25 downto 4);
                    when others => null;
                end case;
            end if;

            -- [E] palette (zc 512 .. 563): 12 words from the table, one every 4 clocks: ground Y U V, then
            -- A / B / accent stored as differences from the ground
            if s_zone = '1' and s_zc(11 downto 6) = "001000" and s_zc(1 downto 0) = "00" then
                v_pi := s_zc(5 downto 2);
                r_ex_a <= '1' & (resize(s_pal & "000", 8) + resize(s_pal & "00", 8) + resize(v_pi, 8));
                v_pd := signed(resize(r_exf(9 downto 0), 11));
                case to_integer(v_pi) is
                    when 1  => c_gy <= r_exf(9 downto 0);
                    when 2  => c_gu <= r_exf(9 downto 0);
                    when 3  => c_gv <= r_exf(9 downto 0);
                    when 4  => c_dy(0) <= v_pd - signed(resize(c_gy, 11));
                    when 5  => c_du(0) <= v_pd - signed(resize(c_gu, 11));
                    when 6  => c_dv(0) <= v_pd - signed(resize(c_gv, 11));
                    when 7  => c_dy(1) <= v_pd - signed(resize(c_gy, 11));
                    when 8  => c_du(1) <= v_pd - signed(resize(c_gu, 11));
                    when 9  => c_dv(1) <= v_pd - signed(resize(c_gv, 11));
                    when 10 => c_dy(2) <= v_pd - signed(resize(c_gy, 11));
                    when 11 => c_du(2) <= v_pd - signed(resize(c_gu, 11));
                    when 12 => c_dv(2) <= v_pd - signed(resize(c_gv, 11));
                    when others => null;
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
            s_st <= g_start & s_st(0 to 7);
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
    -- [C] WAVE ENGINE, line part: two DDAs down the screen, stepped at each line
    -- start for the NEXT line (and reset in the zone for line 0):
    --   n_y  = Y (tiles);  n_fy = f(Y) from the shape table;  n_dfy = f(Y) - f(Y of the line above)
    --   n_tv = theta along y;  n_th = the line's part of theta = phase (+ |n_tv| in Radial)
    -- The beam and the pixel path take them at the next g_start.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_a : unsigned(12 downto 0);
        variable v_ya, v_yb : unsigned(21 downto 0);
        variable v_ta, v_tb : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 2200 then e_zk <= '1'; end if;    -- after the field constants and the shape fill
            sf_qf <= sf_q;

            -- the two line DDAs: load in the zone, one step per line (operands muxed, one subtract each)
            if e_zk = '1' then v_ya := c_Y0; v_yb := (others => '0'); v_ta := c_T0; v_tb := (others => '0');
            else v_ya := n_y; v_yb := c_ystep; v_ta := n_tv; v_tb := c_Td; end if;
            if e_zk = '1' or (g_start = '0' and e_lc = 0) then
                n_y <= f_addsub(v_ya, v_yb, '1'); n_tv <= f_addsub(v_ta, v_tb, '1');
            end if;

            if e_zk = '1' then
                e_lc <= to_unsigned(1, 4);
                e_first <= '1';
            elsif g_start = '1' then
                e_lc <= (others => '0');
                e_first <= '0';
            else
                if e_lc /= 15 then e_lc <= e_lc + 1; end if;
                case to_integer(e_lc) is
                    when 1 =>
                        if n_tv(23) = '1' then n_av <= not n_tv(19 downto 0); else n_av <= n_tv(19 downto 0); end if;
                    when 2 =>
                        if s_rad = '1' then n_th <= c_phl + n_av; else n_th <= c_phl; end if;
                    when 4 =>                     -- the shape RAM's answer (address 1, register 2, fabric copy 3)
                        if e_first = '1' then n_dfy <= (others => '0');
                        else n_dfy <= resize(sf_qf(11 downto 0), 13) - resize(n_fy, 13); end if;
                    when 5 =>
                        n_fy <= sf_qf(11 downto 0);
                        v_a := unsigned(abs(n_dfy));
                        if v_a(12 downto 10) /= "000" then n_df6 <= (others => '1'); else n_df6 <= v_a(9 downto 4); end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [C] WAVE ENGINE, beam: free-running DDAs, loaded with the pre-pixel's
    -- values so that pixel 0 arrives at each one's stage.
    --   U = theta along x, stage 5 (theta at stage 6)       V = X, stage 11
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_op : unsigned(19 downto 0);
        variable v_ua, v_ub : unsigned(23 downto 0);
        variable v_va, v_vb : unsigned(21 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                l_y <= n_y(21 downto 8); l_fy <= n_fy(10 downto 1); l_df6 <= n_df6; l_th <= n_th;
            end if;
            -- the DDAs count down (they hold -theta_x and -X: no negated start values; the tiling is only mirrored)
            if s_st(2) = '1' then v_ua := c_U0; v_ub := (others => '0'); else v_ua := b_U; v_ub := c_Ud; end if;
            b_U <= f_addsub(v_ua, v_ub, '1');
            if s_st(8) = '1' then v_va := c_V0; v_vb := (others => '0'); else v_va := b_V; v_vb := c_Vd; end if;
            b_V <= f_addsub(v_va, v_vb, '1');
            -- 6: theta = line part + x part (Radial: |x part|, so the index is |dx| + |dy|: diamond rings)
            if s_rad = '1' and b_U(23) = '1' then v_op := not b_U(19 downto 0); else v_op := b_U(19 downto 0); end if;
            s6_th <= f_addsub(l_th, v_op, not s_rad);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [C] shape table fill (zone, zc 40 .. 2088): entry t = a(t) + (b(t) - a(t)) * eighths / 8, a and b the
    -- two neighbouring profiles; 8 clocks per entry, one small operation each.
    ---------------------------------------------------------------------------
    p_shape : process(clk)
        variable v_g1, v_g2 : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            sh_we <= '0';
            if s_zone = '1' and s_zc = 40 then
                sh_run <= '1'; sh_t <= (others => '0'); sh_ph <= (others => '0');
                sh_i <= z_s3(14 downto 13); sh_fr <= z_s3(12 downto 10);
            elsif sh_run = '1' then
                sh_ph <= sh_ph + 1;
                case to_integer(sh_ph) is
                    when 0 => pr_a <= sh_i & sh_t;
                    when 1 => pr_a <= (sh_i + 1) & sh_t;
                    when 3 => sh_va <= pr_qf(11 downto 0);
                    when 4 => sh_vd <= resize(pr_qf(11 downto 0), 13) - resize(sh_va, 13);
                    when 5 =>
                        if sh_fr(2) = '1' then v_g1 := shift_left(resize(sh_vd, 16), 2); else v_g1 := (others => '0'); end if;
                        if sh_fr(1) = '1' then v_g2 := shift_left(resize(sh_vd, 16), 1); else v_g2 := (others => '0'); end if;
                        sh_p1 <= v_g1 + v_g2;
                        if sh_fr(0) = '1' then v_g1 := resize(sh_vd, 16); else v_g1 := (others => '0'); end if;
                        sh_p2 <= v_g1 + (shift_left(resize(sh_va, 16), 3) + 4);
                    when 6 => sh_vs <= sh_p1 + sh_p2;
                    when 7 =>
                        sh_we <= '1'; sh_wa <= sh_t; sh_wd <= resize(sh_vs(15 downto 3), 16);
                        sh_t <= sh_t + 1;
                        if sh_t = 255 then sh_run <= '0'; end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_shape;

    ---------------------------------------------------------------------------
    -- [C] pixel path (stages 7 .. 28): wave -> A -> X', Y' -> edge distances in
    -- pixels -> soft parity bits, outline -> XOR blends
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_w12 : unsigned(11 downto 0);
        variable v_sn  : unsigned(13 downto 0);
        variable v_a   : signed(11 downto 0);
        variable v_ab  : unsigned(9 downto 0);
        variable v_ah  : signed(4 downto 0);
        variable v_al  : signed(5 downto 0);
        variable v_p   : signed(20 downto 0);
        variable v_d   : signed(13 downto 0);
        variable v_u   : unsigned(13 downto 0);
        variable v_ws  : unsigned(11 downto 0);
        variable v_m   : unsigned(7 downto 0);
        variable v_s5  : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- 7: quarter-wave address from theta
            s7_neg <= s6_th(19);
            if s6_th(18) = '1' then sn_a <= not s6_th(17 downto 8); else sn_a <= s6_th(17 downto 8); end if;
            -- 8: the sine ROM's own register (p_rom)
            s8_neg <= s7_neg;
            -- 9: wave magnitude, 8 bits; Snap = 2.5 x, clipped (dwell at the extremes)
            v_w12 := sn_q(13 downto 2);
            if s_snap = '1' then
                v_sn := resize(v_w12 & '0', 14) + resize(v_w12(11 downto 1), 14);
                if v_sn(13 downto 12) /= "00" then s9_mag <= (others => '1'); else s9_mag <= v_sn(11 downto 4); end if;
            else
                s9_mag <= v_w12(11 downto 4);
            end if;
            s9_neg <= s8_neg;
            -- 10: Depth * wave from the Depth table (p_dtram)
            s10_neg <= s9_neg;
            -- 11: fabric copy
            s11_dw <= dt_q; s11_neg <= s10_neg;
            -- 12: base +- wave
            if s11_neg = '1' then s12_a <= signed(resize(c_base, 11)) - signed(resize(s11_dw, 11));
            else s12_a <= signed(resize(c_base, 11)) + signed(resize(s11_dw, 11)); end if;
            s12_src <= s11_d8;
            -- 13: A = that + source, clamped to +-511 (1/512 tile); the shape RAM's fabric copy (f / 2)
            v_a := resize(s12_a, 12) + signed(resize(s12_src, 12));
            if v_a > 511 then s13_A <= to_signed(511, 10);
            elsif v_a < -511 then s13_A <= to_signed(-511, 10);
            else s13_A <= v_a(9 downto 0); end if;
            s13_g <= sg_q(10 downto 1);
            -- 14: A * f(X) and A * f(Y) as hi / lo partial products; |A|
            v_ah := s13_A(9 downto 5);
            v_al := signed('0' & s13_A(4 downto 0));
            s14_pyh <= v_ah * s13_g;
            s14_pyl <= v_al * s13_g;
            s14_pxh <= v_ah * l_fy;
            s14_pxl <= v_al * l_fy;
            v_ab := unsigned(abs(s13_A));
            s14_aa4 <= v_ab(8 downto 5);
            -- 15: the displacements (tile units Q12); |A| * |df|
            v_p := shift_left(resize(s14_pyh, 21), 5) + resize(s14_pyl, 21);
            s15_dy <= v_p(19 downto 6);
            v_p := shift_left(resize(s14_pxh, 21), 5) + resize(s14_pxl, 21);
            s15_dx <= v_p(19 downto 6);
            s15_q <= s14_aa4 * l_df6;
            s12_x <= b_V(21 downto 8); s13_x <= s12_x; s14_x <= s13_x; s15_x <= s14_x;
            -- 16: X' = X + A f(Y), Y' = Y - A f(X)  (Q2.12 tiles); X' gradient down the screen
            s16_xp <= s15_x + unsigned(s15_dx);
            s16_yp <= l_y - unsigned(s15_dy);
            if s15_q(9) = '1' then s16_gy <= (others => '1'); else s16_gy <= s15_q(8 downto 0) & "00"; end if;
            -- 17: gradients along the line by finite differences
            s17_gxx <= signed(s16_xp - s17_xp);
            s17_gxy <= signed(s16_yp - s17_yp);
            s17_xp <= s16_xp; s17_yp <= s16_yp; s17_gy <= s16_gy;
            -- 18: magnitudes, clamped to half a tile per pixel
            v_d := s17_gxx;
            if v_d(13) = '1' then v_u := unsigned(-v_d); else v_u := unsigned(v_d); end if;
            if v_u(13 downto 11) /= "000" then s18_agx <= (others => '1'); else s18_agx <= v_u(10 downto 0); end if;
            v_d := s17_gxy;
            if v_d(13) = '1' then v_u := unsigned(-v_d); else v_u := unsigned(v_d); end if;
            if v_u(13 downto 11) /= "000" then s18_agy <= (others => '1'); else s18_agy <= v_u(10 downto 0); end if;
            s18_gy <= s17_gy; s18_xp <= s17_xp; s18_yp <= s17_yp;
            -- 19: |grad| of each family (tile units per pixel); distance to the nearest edge; cell bit
            v_ws := resize(s18_agx, 12) + resize(s18_gy, 12);
            if v_ws(11) = '1' then s19_wx <= (others => '1'); else s19_wx <= v_ws(10 downto 0); end if;
            v_ws := resize(s18_agy, 12) + resize(c_sy, 12);
            if v_ws(11) = '1' then s19_wy <= (others => '1'); else s19_wy <= v_ws(10 downto 0); end if;
            if s18_xp(11) = '1' then s19_nx <= not s18_xp(10 downto 0); else s19_nx <= s18_xp(10 downto 0); end if;
            if s18_yp(11) = '1' then s19_ny <= not s18_yp(10 downto 0); else s19_ny <= s18_yp(10 downto 0); end if;
            s19_bx <= s18_xp(12); s19_by <= s18_yp(12);
            -- 20 .. 22: q = 8 * distance / |grad| (1/8 px, 31 = 3.9 px or more)
            s20_pex <= f_pe(s19_wx); s20_nx <= s19_nx; s20_bx <= s19_bx;
            s20_pey <= f_pe(s19_wy); s20_ny <= s19_ny; s20_by <= s19_by;
            s21_xwx <= f_win(resize(s20_nx, 13), s20_pex(5 downto 2)); s21_mtx <= s20_pex(1 downto 0); s21_bx <= s20_bx;
            s21_xwy <= f_win(resize(s20_ny, 13), s20_pey(5 downto 2)); s21_mty <= s20_pey(1 downto 0); s21_by <= s20_by;
            s22_qx <= f_rq(s21_xwx, s21_mtx); s22_bx <= s21_bx;
            s22_qy <= f_rq(s21_xwy, s21_mty); s22_by <= s21_by;
            -- 23: soft parity bit (0 .. 8) and outline coverage of each family
            s23_px <= f_soft(s22_qx, s22_bx); s23_olx <= f_oln(s22_qx, w_ol);
            s23_py <= f_soft(s22_qy, s22_by); s23_oly <= f_oln(s22_qy, w_ol);
            -- 24, 25: XOR blend of the two families: p = px + py - px py / 4
            s24_m <= s23_px * s23_py;
            s24_s <= resize(s23_px, 5) + resize(s23_py, 5);
            if s_oln = '0' then s24_ol <= (others => '0');
            elsif s23_olx >= s23_oly then s24_ol <= s23_olx;
            else s24_ol <= s23_oly; end if;
            v_m := shift_right(s24_m + 2, 2);
            v_s5 := s24_s - v_m(4 downto 0);
            s25_p <= v_s5(3 downto 0);
            s25_ol <= s24_ol;
            -- 26, 27: XOR blend with the detail
            s26_m <= s25_p * s25_det;
            s26_s <= resize(s25_p, 5) + resize(s25_det, 5);
            s26_ol <= s25_ol;
            v_m := shift_right(s26_m + 2, 2);
            v_s5 := s26_s - v_m(4 downto 0);
            s27_p <= v_s5(3 downto 0);
            s27_ol <= s26_ol; s27_nol <= to_unsigned(8, 4) - s26_ol;
            -- 28: the outline takes its share
            v_m := shift_right(s27_p * s27_nol + 4, 3);
            s28_pp <= v_m(3 downto 0);
            s28_ol <= s27_ol;

            -- Depth table fill (zone, zc 200 .. 456): T(i) = (D * i) >> 8 by accumulation
            dt_we <= '0';
            if s_zone = '1' and s_zc = 200 then
                dt_run <= '1'; dt_i <= (others => '0'); dt_acc <= (others => '0');
            elsif dt_run = '1' then
                dt_we <= '1'; dt_wa <= dt_i; dt_wd <= dt_acc(15 downto 8);
                dt_acc <= dt_acc + c_D; dt_i <= dt_i + 1;
                if dt_i = 255 then dt_run <= '0'; end if;
            end if;

            -- column counter of the luma line store's pixel-path port
            if s_st(3) = '1' or s_st(4) = '1' then lb_rc <= (others => '0'); else lb_rc <= lb_rc + 1; end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Detail (stages 17 .. 25): an echo contour (Chebyshev ring) and an eye
    -- (diamond dot) in the tile's own coordinates; the larger signed distance
    -- over the gradient of the nearer edge family = coverage 0 .. 8
    ---------------------------------------------------------------------------
    p_det : process(clk)
        variable v_c : signed(11 downto 0);
        variable v_e : unsigned(12 downto 0);
        variable v_r : unsigned(11 downto 0);
        variable v_n : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 17: |centred coordinate|; offset from the eye
            if s16_xp(11) = '1' then s17_axc <= s16_xp(10 downto 0); else s17_axc <= not s16_xp(10 downto 0); end if;
            if s16_yp(11) = '1' then s17_ayc <= s16_yp(10 downto 0); else s17_ayc <= not s16_yp(10 downto 0); end if;
            v_c := signed((not s16_xp(11)) & s16_xp(10 downto 0));
            s17_ex <= resize(v_c, 13) - C_EX;
            v_c := signed((not s16_yp(11)) & s16_yp(10 downto 0));
            s17_ey <= resize(v_c, 13) - C_EY;
            -- 18
            if s17_axc >= s17_ayc then s18_m <= s17_axc; s18_sel <= '1'; else s18_m <= s17_ayc; s18_sel <= '0'; end if;
            v_e := unsigned(abs(s17_ex)); s18_aex <= v_e(11 downto 0);
            v_e := unsigned(abs(s17_ey)); s18_aey <= v_e(11 downto 0);
            -- 19
            s19_mr <= signed(resize(s18_m, 12)) - C_RE;
            s19_es <= resize(s18_aex, 13) + resize(s18_aey, 13);
            s19_sel <= s18_sel;
            -- 20: signed distances inside the ring line / the eye; the gradient that applies
            v_r := unsigned(abs(s19_mr));
            s20_dr <= signed(resize(c_hw, 13)) - signed(resize(v_r, 13));
            s20_de <= to_signed(C_RHO, 14) - signed(resize(s19_es, 14));
            if s19_sel = '1' then s20_w <= s19_wx; else s20_w <= s19_wy; end if;
            -- 21: the nearer feature
            if resize(s20_dr, 14) >= s20_de then s21_d <= resize(s20_dr, 14); else s21_d <= s20_de; end if;
            s21_pe <= f_pe(s20_w);
            -- 22
            v_n := unsigned(abs(s21_d));
            if v_n(13 downto 11) /= "000" then s22_n <= (others => '1'); else s22_n <= v_n(10 downto 0); end if;
            s22_sg <= s21_d(13); s22_pe <= s21_pe;
            -- 23, 24: distance in 1/8 px
            s23_xw <= f_win(resize(s22_n, 13), s22_pe(5 downto 2)); s23_mt <= s22_pe(1 downto 0); s23_sg <= s22_sg;
            s24_dq <= f_rq(s23_xw, s23_mt); s24_sg <= s23_sg;
            -- 25: coverage 0 .. 8
            if s_det = '0' then s25_det <= (others => '0'); else s25_det <= f_soft(s24_dq, not s24_sg); end if;
        end if;
    end process p_det;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 32-px box, edge-replicated
    -- and reloaded at every line start -> vertical 1/8 IIR in the line store
    -- (half horizontal resolution).  The pixel path reads the store, interpolates
    -- and looks the weight up: stages 6 .. 12.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d2, v_q2 : signed(13 downto 0);
        variable v_a : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
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

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 samples; the samples before the line start count as the first one
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(225, 8); end if;
            if s_bcnt < 32 then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                s_bsum <= shift_left(resize(s_sl, 13), 5);
                s_bcnt <= to_unsigned(1, 6);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 32 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on the odd columns, 12-bit state (Q8.4): q += (16 box - q + 16) >> 5
            s_eb8 <= s_bsum(12 downto 5); s_eq <= lb_q1(11 downto 0);
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d2 := signed(resize(s_eb8 & "0000", 14)) - signed(resize(s_eq, 14)) + 16;
            v_q2 := signed(resize(s_eq, 14)) + shift_right(v_d2, 5);
            lb_wd <= "0000" & unsigned(v_q2(11 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- pixel path: 8 fabric copies of this and the last read; 9 the mean (a linear interpolation of
            -- the half-resolution store); 10 the weight table; 11, 12 fabric copies (duty / 256)
            s8_q <= lb_q2(11 downto 4); s8_qp <= s8_q;
            v_a := resize(s8_q, 9) + resize(s8_qp, 9) + 1;
            s9_l <= v_a(8 downto 1);
            s11_d8 <= wt_q;
            s12_d8 <= s11_d8;

            -- source table fill (zone, zc 224 .. 480): A units by source level, (320 l^2) >> 16, the square by
            -- second differences
            wt_we <= '0';
            if s_zone = '1' and s_zc = 224 then
                wt_run <= '1'; wt_i <= (others => '0'); wt_acc <= (others => '0'); wt_d <= to_unsigned(320, 18);
            elsif wt_run = '1' then
                wt_we <= '1'; wt_wa <= wt_i;
                wt_wd <= wt_acc(23 downto 16);
                wt_acc <= wt_acc + wt_d; wt_d <= wt_d + 640; wt_i <= wt_i + 1;
                if wt_i = 255 then wt_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] COLOUR (stages 29 .. 31): two per-field tables, summed:
    --   ta(pp) = (8 ground + 4 + (A - ground) pp) >> 3        tc(ol) = ((accent - ground) ol + 4) >> 3
    -- The tables are refilled in the zone after the palette load (zc 600 .. 609) by accumulators.
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_y : signed(11 downto 0);
        variable v_c : signed(11 downto 0);
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 600 then
                tb_run <= '1'; tb_i <= (others => '0');
                tb_ay <= signed(resize(c_gy & "100", 15)); tb_au <= signed(resize(c_gu & "100", 15));
                tb_av <= signed(resize(c_gv & "100", 15));
                tb_cy <= to_signed(4, 15); tb_cu <= to_signed(4, 15); tb_cv <= to_signed(4, 15);
            elsif tb_run = '1' then
                tb_ay <= tb_ay + c_dy(0); tb_au <= tb_au + c_du(0); tb_av <= tb_av + c_dv(0);
                tb_cy <= tb_cy + c_dy(2); tb_cu <= tb_cu + c_du(2); tb_cv <= tb_cv + c_dv(2);
                tb_i <= tb_i + 1;
                if tb_i = 8 then tb_run <= '0'; end if;
            end if;

            -- 29: the tables' own registers (p_taram, p_tcram); 30: fabric copies
            s30_ta <= ta_q; s30_tc <= tc_q;
            -- 31: the sum; the ground colour outside pixels 0 .. W + 7
            v_win := s_gav_sr(28) or s_gav_sr(36);
            if v_win = '1' then
                v_y := signed(resize(s30_ta(29 downto 20), 12)) + resize(signed(s30_tc(30 downto 20)), 12);
                s25_y <= unsigned(v_y(9 downto 0));
                v_c := signed(resize(s30_ta(19 downto 10), 12)) + resize(signed(s30_tc(19 downto 10)), 12);
                s25_u <= unsigned(v_c(9 downto 0));
                v_c := signed(resize(s30_ta(9 downto 0), 12)) + resize(signed(s30_tc(9 downto 0)), 12);
                s25_v <= unsigned(v_c(9 downto 0));
            else
                s25_y <= c_gy; s25_u <= c_gu; s25_v <= c_gv;
            end if;
            s_hist <= s_hist(6 downto 0) & v_win;
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 26 .. 29): chroma [1 2 2 2 2 2 2 2 1]/16 as
    -- two adjacent running 8-sums (resynced to the ground colour on every
    -- line), luma [1 2 1]/4 delayed to the chroma's group delay (4 pixels),
    -- luma 13/16 toward black, blanking gate.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 26: running 8-sums (+ 4: rounds the 9-tap sum); they snap to 8 x ground once 8 samples outside
            -- the picture are in, so a glitched sample can't offset the colours for good
            s_cu(0) <= s25_u; s_cv(0) <= s25_v;
            for i in 1 to 7 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            if s_hist = "00000000" then
                s_bu <= (c_gu & "000") + 4; s_bv2 <= (c_gv & "000") + 4;
            else
                s_bu  <= s_bu + s25_u - s_cu(7);
                s_bv2 <= s_bv2 + s25_v - s_cv(7);
            end if;
            s_bud <= s_bu; s_bvd <= s_bv2;
            -- luma taps y(n - 3), y(n - 4), y(n - 5)
            s_yd(1) <= s25_y;
            for i in 2 to 5 loop s_yd(i) <= s_yd(i - 1); end loop;
            s26_ya <= resize(s_yd(3), 12) + resize(s_yd(5), 12) + 2;
            s26_yb <= s_yd(4);
            -- 27
            v_sc := resize(s_bu, 14) + s_bud;  s27_u <= v_sc(13 downto 4);
            v_sc := resize(s_bv2, 14) + s_bvd; s27_v <= v_sc(13 downto 4);
            v_y := s26_ya + shift_left(resize(s26_yb, 12), 1);
            s27_y <= v_y(11 downto 2);
            -- 28: luma 13/16 toward black
            if s27_y > 64 then v_dl := s27_y - 64; else v_dl := (others => '0'); end if;
            s28_y <= s27_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s28_u <= s27_u; s28_v <= s27_v;
            -- 29: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s29_y <= s28_y; s29_u <= s28_u; s29_v <= s28_v;
            else
                s29_y <= to_unsigned(64, 10); s29_u <= to_unsigned(512, 10); s29_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s29_y);
    data_out.u       <= std_logic_vector(s29_u);
    data_out.v       <= std_logic_vector(s29_v);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture metamorph;
