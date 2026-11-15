-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Outrun v0.1.0 -- Kinetic.  The neon perspective grid of 1980s computer graphics: a glowing grid floor
-- running to a horizon, a banded sun, optionally a mirrored ceiling.  Built on the riley chassis.
--
-- Element: a z row of the grid.  State: the terrain height under it.
--
--   s      = |row - horizon|                      (virtual rows)
--   z      = Zk / s,  u = (x - centre) * Uk / s   (cells; one divide and a few products per LINE)
--   theta  = phase + Spread * (z + |u|)                       (Field = Linear: hills roll toward the viewer)
--            phase - Spread * (|z - zc| + |u|)                (Field = Radial: diamond ripples from the point
--                                                              200 rows below the horizon)
--   h      = Depth * wave(theta) + source                     (cells)
--   z'     = z + flight - h                                   (the height as a displacement of the sampled z)
--   lines  where frac(z') or frac(u') is near 0: anti-aliased core + halo, weight and alpha per line
--
--   K1  Scale    grid pitch 60 .. 600 virtual px at 512 rows below the horizon (exponential, glided)
--   K2  Shape    cell aspect 1:2 .. 4:1 (wide tiles .. squares .. long lanes)
--   K3  Horizon  horizon height, 25 % .. 75 % of the screen
--   K4  Rate     flight / wave speed, bipolar, centre = still
--   K5  Spread   wave cycles per cell, square law, 0 .. 1/2
--   K6  Colour   the 16 shared Kinetic palettes (ground, A = glow, B = grid, accent = sun top)
--   S7  Wave     Smooth (sine) / Step (terraces)
--   S8  Field    Linear / Radial
--   S9  Space    Sky / Tunnel (mirrored ceiling)
--   S10 Sun      On / Off
--   S11 Source   Video (luma lifts the terrain and tints the sky) / Osc
--   P12 TERRAIN  wave depth, square law: flat .. 1.5 cells
--
-- Sections (the integer model .kinetic_work/rb_outrun/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   (riley chassis)
--   [B] CONTROLS                         (riley chassis)
--   [C] ENGINE                           per-field sequencer (one serial multiplier), per-line engine (a
--                                        divider, one 24-step serial multiplier, a normaliser; computes the
--                                        NEXT line), beam DDAs, the pixel path
--   [D] SOURCE FIELD                     Osc lattice, 64-px box, vertical IIR line store
--   [E] BLEND                            17-step shift-add blend from a per-line base colour
--   [F] OUTPUT CONDITIONING              (riley chassis)
--
-- Stage numbering as in riley: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).
-- One latency (36) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.outrun_pkg.all;
use work.all;

architecture outrun of program_top is
    constant C_LAT : integer := 36;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 341;
    constant C_DEF_K3  : integer := 626;
    constant C_DEF_K5  : integer := 591;
    constant C_DEF_P12 : integer := 590;

    constant C_SUNUP : integer := 22272;      -- sun centre above the horizon, Q8 rows (87)
    constant C_SUNR2 : integer := 270400;     -- sun radius^2 in h^2 (260 virtual px)

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
    -- a + b or a - b on ONE carry chain
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;
    -- per-axis line constants from r = cells per pixel (Q14): alpha 0 .. 16, half-width (Q14 cells)
    function f_al(r : unsigned(15 downto 0)) return unsigned is
        variable v_t : unsigned(11 downto 0);
    begin
        if r >= 2816 then return to_unsigned(0, 5); end if;
        v_t := to_unsigned(2816, 12) - r(11 downto 0);
        if v_t(11 downto 6) >= 16 then return to_unsigned(16, 5); end if;
        return '0' & v_t(9 downto 6);
    end function;
    function f_hw(r : unsigned(15 downto 0)) return unsigned is
    begin
        if r(15 downto 13) /= "000" then return to_unsigned(8191, 13); end if;
        if r < 256 then return to_unsigned(256, 13); end if;
        return r(12 downto 0);
    end function;
    -- distance to the nearest line, less the half-width (Q14 cells)
    function f_edge(f : unsigned(13 downto 0); hw : unsigned(12 downto 0)) return signed is
        variable v_d : unsigned(12 downto 0);
    begin
        if f(13) = '1' then v_d := not f(12 downto 0); else v_d := f(12 downto 0); end if;
        return signed(resize(v_d, 14)) - signed(resize(hw, 14));
    end function;
    -- (e << 3) >> p, clipped to 9 bits
    function f_bar(e : signed(13 downto 0); p : unsigned(3 downto 0)) return signed is
        variable v_s : signed(16 downto 0);
    begin
        v_s := shift_right(shift_left(resize(e, 17), 3), to_integer(p));
        if v_s(16) = '0' and v_s(15 downto 8) /= "00000000" then return to_signed(255, 9); end if;
        if v_s(16) = '1' and v_s(15 downto 8) /= "11111111" then return to_signed(-256, 9); end if;
        return v_s(8 downto 0);
    end function;
    -- w * (4 + n) / 4
    function f_man(w : signed(8 downto 0); n : unsigned(1 downto 0)) return signed is
        variable v_a : signed(11 downto 0);
    begin
        v_a := (others => '0');
        if n(0) = '1' then v_a := resize(w, 12); end if;
        if n(1) = '1' then v_a := v_a + shift_left(resize(w, 12), 1); end if;
        return resize(w, 10) + resize(shift_right(v_a, 2), 10);
    end function;
    -- core coverage (clamped to the line's alpha) or halo, whichever is larger
    function f_cov(v : signed(9 downto 0); al : unsigned(4 downto 0)) return unsigned is
        variable v_c : signed(10 downto 0);
        variable v_cov, v_h, v_hm : unsigned(4 downto 0);
    begin
        v_c := to_signed(8, 11) - resize(v, 11);
        if v >= 8 then v_cov := (others => '0');
        elsif v_c >= signed(resize(al, 11)) then v_cov := al;
        else v_cov := unsigned(v_c(4 downto 0)); end if;
        if v(9) = '1' then
            v_h := (others => '0');
        else
            case to_integer(unsigned(v(8 downto 4))) is
                when 0      => v_h := to_unsigned(6, 5);
                when 1      => v_h := to_unsigned(5, 5);
                when 2      => v_h := to_unsigned(4, 5);
                when 3      => v_h := to_unsigned(3, 5);
                when 4 | 5  => v_h := to_unsigned(2, 5);
                when 6 | 7  => v_h := to_unsigned(1, 5);
                when others => v_h := to_unsigned(0, 5);
            end case;
        end if;
        v_hm := '0' & al(4 downto 1);
        if v_h > v_hm then v_h := v_hm; end if;
        if v_h > v_cov then return v_h; end if;
        return v_cov;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    -- the beam's pre-pixel by video mode: pixels left of the centre, that distance in h squared, minus the
    -- first difference of the square there, and the second difference
    signal w_npre : unsigned(9 downto 0);
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
    signal s_st       : std_logic_vector(0 to 15) := (others => '0');   -- s_st(k) = g_start delayed k + 1

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
    signal s_stp, s_rad, s_tun, s_sunoff, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(4, 4);

    -- [C] ROMs: C_OR_TAB (2^(i/256), then the palettes), C_OR_SIN (a quarter wave)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_a8   : signed(10 downto 0) := (others => '0');
    signal z_sh   : signed(3 downto 0) := (others => '0');
    signal z_sh4  : unsigned(2 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_invp : unsigned(20 downto 0) := (others => '0');
    signal c_uk   : unsigned(20 downto 0) := (others => '0');
    signal c_zk   : unsigned(21 downto 0) := (others => '0');
    signal c_zc   : unsigned(21 downto 0) := (others => '0');      -- Radial: the ripple centre (cells, Q14)
    signal c_ai   : unsigned(15 downto 0) := (others => '0');      -- 2^-a, Q14
    signal c_sp   : unsigned(14 downto 0) := (others => '0');
    signal c_kx   : unsigned(15 downto 0) := (others => '0');
    signal c_F    : unsigned(11 downto 0) := (others => '0');
    signal c_gq   : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal c_yh8  : unsigned(18 downto 0) := to_unsigned(153630, 19);
    signal c_rstep : unsigned(27 downto 0) := (others => '0');
    signal c_ph   : unsigned(31 downto 0) := (others => '0');

    -- [E] per-field palette: ground, (A, B, accent) - ground, A - accent
    type t_s11x3 is array (0 to 2) of signed(10 downto 0);
    signal c_gy, c_gu, c_gv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_dy, c_du, c_dv : t_s11x3 := (others => (others => '0'));
    signal c_aay, c_aau, c_aav : signed(10 downto 0) := (others => '0');
    signal w_d2y, w_d2u, w_d2v : signed(10 downto 0);

    -- [C] line engine
    signal e_lc   : unsigned(9 downto 0) := (others => '1');
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_d    : signed(20 downto 0) := (others => '0');
    signal s_sk   : signed(20 downto 0) := to_signed(22016, 21);   -- sun centre offset less one line (Q8 rows)
    signal e_sqr, e_sq : unsigned(11 downto 0) := to_unsigned(32, 12);
    signal e_dv   : std_logic := '0';
    signal dv_r   : unsigned(11 downto 0) := (others => '0');
    signal dv_q   : unsigned(21 downto 0) := (others => '0');
    signal e_ts   : unsigned(5 downto 0) := (others => '0');
    signal e_p4   : unsigned(7 downto 0) := (others => '0');
    signal e_th4  : signed(12 downto 0) := (others => '0');
    signal e_dy   : signed(20 downto 0) := (others => '0');
    signal e_m1, e_m2 : signed(13 downto 0) := (others => '0');
    signal e_tlt  : std_logic := '0';
    signal m_a, m_acc : unsigned(43 downto 0) := (others => '0');
    signal m_b    : unsigned(23 downto 0) := (others => '0');
    signal e_zl   : unsigned(21 downto 0) := (others => '0');
    signal e_zt   : unsigned(15 downto 0) := (others => '0');
    signal e_dupx : unsigned(25 downto 0) := (others => '0');
    signal e_r16, e_nr : unsigned(15 downto 0) := (others => '0');
    signal e_np   : unsigned(3 downto 0) := (others => '0');
    -- next line / this line (the sun's values are written before any sun pixel of the line in flight
    -- can use them, so they need no second copy)
    signal n_u0, n_du, l_du : unsigned(23 downto 0) := (others => '0');
    signal n_J0   : unsigned(25 downto 0) := (others => '0');
    signal n_dJ, l_dJ : unsigned(19 downto 0) := (others => '0');
    signal n_thl, l_thl : unsigned(17 downto 0) := (others => '0');
    signal n_zf, l_zf : unsigned(13 downto 0) := (others => '0');
    signal n_alz, n_alu, l_alz, l_alu : unsigned(4 downto 0) := (others => '0');
    signal n_hwz, n_hwu, l_hwz, l_hwu : unsigned(12 downto 0) := (others => '0');
    signal n_pz, n_pu, l_pz, l_pu : unsigned(3 downto 0) := (others => '0');
    signal n_nz, n_nu, l_nz, l_nu : unsigned(1 downto 0) := (others => '0');
    signal n_gl, l_gl : unsigned(5 downto 0) := (others => '0');
    signal n_ceil, l_ceil, n_par, l_par : std_logic := '0';
    signal n_hw2  : signed(20 downto 0) := (others => '0');
    signal n_sa   : unsigned(4 downto 0) := (others => '0');
    signal n_d1y, n_d1u, n_d1v : signed(10 downto 0) := (others => '0');

    -- [C] beam DDAs and the pixel path
    signal b_J : unsigned(25 downto 0) := (others => '0');
    signal b_U : unsigned(23 downto 0) := (others => '0');
    signal b_Q, b_D : unsigned(21 downto 0) := (others => '0');
    signal s3_th : unsigned(17 downto 0) := (others => '0');
    signal s4_neg, s5_neg, s6_neg, s7_neg, s8_neg, s9_neg : std_logic := '0';
    signal s6_mag : unsigned(11 downto 0) := (others => '0');
    signal s8_lo, s8_hi : unsigned(15 downto 0) := (others => '0');
    signal s9_fw : unsigned(14 downto 0) := (others => '0');
    signal s10_h : signed(15 downto 0) := (others => '0');
    signal s11_zp : unsigned(13 downto 0) := (others => '0');
    signal s12_ez, s15_eu : signed(13 downto 0) := (others => '0');
    signal s13_wz, s16_wu : signed(8 downto 0) := (others => '0');
    signal s14_vz, s17_vu : signed(9 downto 0) := (others => '0');
    signal s15_gz, s16_gz, s17_gz, s18_gz, s18_gu, s19_ag, s20_ag : unsigned(4 downto 0) := (others => '0');
    signal s20_cs, s21_a1, s21_a2 : unsigned(4 downto 0) := (others => '0');
    signal s19_e : signed(23 downto 0) := (others => '0');

    -- [C] Fold table: (F * i) >> 2 for i = 0 .. 63, refilled every field by an accumulator
    type t_ft is array (0 to 255) of unsigned(15 downto 0);
    signal fl_ram, fh_ram : t_ft := (others => (others => '0'));
    attribute no_rw_check of fl_ram : signal is true;
    attribute no_rw_check of fh_ram : signal is true;
    signal fl_q, fh_q, ft_wd : unsigned(15 downto 0) := (others => '0');
    signal ft_wa, ft_i : unsigned(5 downto 0) := (others => '0');
    signal ft_acc : unsigned(17 downto 0) := (others => '0');
    signal ft_we, ft_run : std_logic := '0';

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
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_bsum : unsigned(13 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb8, s_eq : unsigned(7 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(7 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(7 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');
    signal s8_q, s8_qp, s9_l : unsigned(7 downto 0) := (others => '0');

    -- [E] blend
    signal s22_cov : unsigned(4 downto 0) := (others => '0');
    signal s22_dy, s22_du, s22_dv : signed(10 downto 0) := (others => '0');
    signal s23_ay, s23_au, s23_av, s23_by, s23_bu, s23_bv : signed(15 downto 0) := (others => '0');
    signal s24_y, s24_u, s24_v : signed(15 downto 0) := (others => '0');
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
            r_ex_q <= C_OR_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_OR_SIN(to_integer(sn_a));
        end if;
    end process p_rom;

    -- Fold tables: written in the vblank zone only, read by the pixel path (stage 6 address)
    p_flram : process(clk)
    begin
        if rising_edge(clk) then
            fl_q <= fl_ram(to_integer(s6_mag(5 downto 0)));
            if ft_we = '1' then fl_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_flram;
    p_fhram : process(clk)
    begin
        if rising_edge(clk) then
            fh_q <= fh_ram(to_integer(s6_mag(11 downto 6)));
            if ft_we = '1' then fh_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_fhram;

    -- box delay: read 64 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    -- luma line store (half horizontal resolution): port 1 = the IIR's own read, port 2 = the pixel path
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 1)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- [A] the beam's pre-pixel by video mode
    ---------------------------------------------------------------------------
    w_npre <= to_unsigned(361, 10) when s_xs = 4 else to_unsigned(641, 10) when s_xs = 3 else to_unsigned(961, 10);
    w_qx0 <= to_unsigned(2085136, 22) when s_xs = 4 else to_unsigned(3697929, 22) when s_xs = 3 else to_unsigned(3694084, 22);
    w_d0m <= to_unsigned(11536, 14) when s_xs = 4 else to_unsigned(11529, 14) when s_xs = 3 else to_unsigned(7684, 14);
    w_dd  <= to_unsigned(32, 6) when s_xs = 4 else to_unsigned(18, 6) when s_xs = 3 else to_unsigned(8, 6);
    -- the second blend target: the grid colour (B) on the floor and in the tunnel, the haze colour (A) in the sky
    w_d2y <= c_dy(1) when s_tun = '1' or l_ceil = '0' else c_dy(0);
    w_d2u <= c_du(1) when s_tun = '1' or l_ceil = '0' else c_du(0);
    w_d2v <= c_dv(1) when s_tun = '1' or l_ceil = '0' else c_dv(0);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_c  : signed(10 downto 0);
        variable v_3  : unsigned(14 downto 0);
        variable v_r  : unsigned(27 downto 0);
        variable v_pi : unsigned(3 downto 0);
        variable v_pd : signed(10 downto 0);
        variable v_gq : unsigned(8 downto 0);
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
            elsif s_zc /= 2047 then
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
                s_stp    <= registers_in(6)(0);
                s_rad    <= registers_in(6)(1);
                s_tun    <= registers_in(6)(2);
                s_sunoff <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
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
            -- load + 17 (stored at 50 + 24 k)
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
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g1, 40);  z_fb <= to_unsigned(850, 16);       -- octaves above pitch 60
                    when 1  => z_fa <= resize(c_invp, 40); z_fb <= to_unsigned(17476, 16);    -- Uk
                    when 2  => z_fa <= resize(c_uk, 40);  z_fb <= c_ai;                       -- Zk = Uk * 2^-a
                    when 3  => z_fa <= resize(s_g5(12 downto 3), 40); z_fb <= resize(s_g5(12 downto 3), 16);  -- spread^2
                    when 4  => z_fa <= resize(c_sp, 40);  z_fb <= c_ai;                       -- Radial kx
                    when 5  => z_fa <= resize(s_gp, 40);  z_fb <= resize(s_gp, 16);           -- depth^2
                    when 6  => z_fa <= resize(s_vm, 40);  z_fb <= to_unsigned(897, 16);       -- rate exponent
                    when 7  => z_fa <= resize(r_exf, 40); z_fb <= to_unsigned(9321, 16);      -- rate step
                    when 8  => z_fa <= resize(s_g3, 40);  z_fb <= to_unsigned(135, 16);       -- horizon
                    when 9  => z_fa <= resize(c_zk, 40); z_fb <= to_unsigned(41943, 16);      -- z at 200 rows below the horizon
                    when others => z_fa <= resize(c_F, 40); z_fb <= '0' & c_sp;               -- Depth * Spread
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
                    when 21 =>                        -- aspect: Q8 octaves, -256 .. 511
                        v_3 := resize(s_g2, 15) + shift_left(resize(s_g2, 15), 1);
                        z_a8 <= signed(resize(v_3(14 downto 5), 11)) - 256;
                    when 50  => z_oct <= f_acc(23 downto 13);
                    when 51  =>
                        v_e := -signed(resize(z_oct, 12));
                        r_ex_a <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(11 downto 8);
                    when 54  =>
                        case to_integer(z_sh) is
                            when 0      => c_invp <= r_exf & "00000";
                            when -1     => c_invp <= resize(r_exf & "0000", 21);
                            when -2     => c_invp <= resize(r_exf & "000", 21);
                            when -3     => c_invp <= resize(r_exf & "00", 21);
                            when others => c_invp <= resize(r_exf & "0", 21);
                        end case;
                        v_e := -resize(z_a8, 12);
                        r_ex_a <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(11 downto 8);
                    when 57  =>                       -- 2^-a, Q14
                        case to_integer(z_sh) is
                            when 1      => c_ai <= r_exf;
                            when 0      => c_ai <= '0' & r_exf(15 downto 1);
                            when -1     => c_ai <= "00" & r_exf(15 downto 2);
                            when others => c_ai <= "000" & r_exf(15 downto 3);
                        end case;
                    when 74  => c_uk <= f_acc(34 downto 14);
                    when 98  => c_zk <= f_acc(35 downto 14);
                    when 122 => c_sp <= f_acc(19 downto 5);
                    when 146 =>
                        if s_rad = '1' then c_kx <= f_acc(29 downto 14); else c_kx <= '0' & c_sp; end if;
                    when 170 => c_F <= resize(f_acc(25 downto 16), 12) - resize(f_acc(25 downto 18), 12);
                    when 194 => r_ex_a <= '0' & f_acc(15 downto 8); z_sh4 <= f_acc(18 downto 16);
                    when 218 =>
                        v_r := resize(f_acc(29 downto 8), 28);
                        if s_vm = 0 then c_rstep <= (others => '0');
                        else c_rstep <= shift_left(v_r, to_integer(z_sh4)); end if;
                    when 242 => c_yh8 <= to_unsigned(69120, 19) + resize(f_acc(20 downto 3), 19);
                    when 266 => c_zc <= f_acc(37 downto 16);
                    when 290 =>                       -- 1 + Depth * 2 pi * Spread (Q4, 1 .. 4): the rows' worst compression
                        v_gq := resize(f_acc(26 downto 19), 9) + resize(f_acc(26 downto 20), 9);
                        if v_gq > 48 then c_gq <= to_unsigned(64, 7);
                        else c_gq <= to_unsigned(16, 7) + v_gq(6 downto 0); end if;
                    when 570 =>
                        c_aay <= c_dy(0) - c_dy(2); c_aau <= c_du(0) - c_du(2); c_aav <= c_dv(0) - c_dv(2);
                    when others => null;
                end case;
            end if;

            -- [E] palette (zc 512 .. 563): 12 words from the table, one every 4 clocks: ground Y U V, then
            -- A / B / accent stored as differences from the ground
            if s_zone = '1' and s_zc(10 downto 6) = "01000" and s_zc(1 downto 0) = "00" then
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

            -- [A] sync delay, generated-avid and start-pulse shift registers
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 14);
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
    -- [C] ENGINE, line part: kicked at each line start (and once in the zone for line 0), computes the
    -- NEXT line's constants: a 26-step divider (2^26 / s), fourteen products on one 24-step serial
    -- multiplier (result at load + 25), a serial normaliser for the two anti-aliasing gains, the sun's
    -- colour and the glow.  About 355 clocks.  The beam copies n_* to l_* at the next g_start.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ad : unsigned(19 downto 0);
        variable v_r2 : unsigned(12 downto 0);
        variable v_t1, v_t2 : unsigned(5 downto 0);
        variable v_v : signed(20 downto 0);
        variable v_dh : unsigned(12 downto 0);
        variable v_ady : unsigned(9 downto 0);
        variable v_m1 : signed(13 downto 0);
        variable v_r16 : unsigned(15 downto 0);
        variable v_zl : unsigned(21 downto 0);
        variable v_zd : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;     -- after the field constants
            s_sk <= to_signed(C_SUNUP, 21) - signed(resize(s_ys, 21));

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                n_par <= '0';
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                n_par <= not n_par;
            elsif e_lc /= 1023 then
                e_lc <= e_lc + 1;
            end if;

            -- the multiplier
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(42 downto 0) & '0';
            m_b <= '0' & m_b(23 downto 1);
            -- the divider (restoring, one quotient bit per clock, 26 clocks)
            if e_dv = '1' then
                v_r2 := dv_r & '0';
                if v_r2 >= resize(e_sq, 13) then
                    dv_r <= resize(v_r2 - resize(e_sq, 13), 12);
                    dv_q <= dv_q(20 downto 0) & '1';
                else
                    dv_r <= v_r2(11 downto 0);
                    dv_q <= dv_q(20 downto 0) & '0';
                end if;
            end if;
            -- the normaliser
            if e_nr(15) = '0' and e_np /= 0 then
                e_nr <= e_nr(14 downto 0) & '0';
                e_np <= e_np - 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    e_d  <= signed(resize(e_vy, 21)) - signed(resize(c_yh8, 21));
                when 2 =>
                    if e_d(20) = '1' then v_ad := unsigned(not e_d(19 downto 0)); else v_ad := unsigned(e_d(19 downto 0)); end if;
                    n_ceil <= e_d(20);
                    if v_ad(19 downto 18) /= "00" then e_sqr <= (others => '1'); else e_sqr <= v_ad(17 downto 6); end if;
                    -- the sun's row from its centre (for the line now starting: the sun's values are used in
                    -- the same line, see above)
                    e_dy <= e_d + s_sk;
                when 3 =>
                    if e_sqr < 32 then e_sq <= to_unsigned(32, 12); else e_sq <= e_sqr; end if;
                    dv_r <= to_unsigned(1, 12); dv_q <= (others => '0'); e_dv <= '1';
                    -- the horizon glow
                    if e_sqr(11 downto 9) = "000" then v_t1 := to_unsigned(32, 6) - resize(e_sqr(8 downto 4), 6); else v_t1 := (others => '0'); end if;
                    if e_sqr < 48 then v_t2 := to_unsigned(24, 6) - resize(e_sqr(5 downto 1), 6); else v_t2 := (others => '0'); end if;
                    n_gl <= v_t1 + v_t2;
                    -- the sun: gradient position, stripe phase, half-chord^2 (a 10-bit square: stable long before 27)
                    v_v := e_dy + 66560;
                    if v_v(20) = '1' then e_ts <= (others => '0');
                    elsif v_v(19 downto 17) /= "000" then e_ts <= (others => '1');
                    else e_ts <= unsigned(v_v(16 downto 11)); end if;
                    if e_dy(20) = '1' then v_dh := unsigned(not e_dy(19 downto 7)); else v_dh := unsigned(e_dy(19 downto 7)); end if;
                    if v_dh(12 downto 10) /= "000" then v_ady := (others => '1'); else v_ady := v_dh(9 downto 0); end if;
                    m_a <= resize(v_ady, 44); m_b <= resize(v_ady, 24); m_acc <= (others => '0');
                    e_p4 <= unsigned(e_dy(11 downto 4)) - c_ph(31 downto 24);
                    v_v := e_dy + 10240;
                    e_th4 <= v_v(20 downto 8);
                when 4 =>                     -- the sun's stripes
                    e_m1 <= signed(resize(e_p4, 14)) - resize(e_th4, 14);
                    e_m2 <= to_signed(256, 14) - signed(resize(e_p4, 14));
                    if e_th4 < 32 then e_tlt <= '1'; else e_tlt <= '0'; end if;
                when 5 =>
                    if e_m2 < e_m1 then v_m1 := e_m2; else v_m1 := e_m1; end if;
                    if e_tlt = '1' then n_sa <= to_unsigned(16, 5);
                    elsif v_m1 < 0 then n_sa <= (others => '0');
                    elsif v_m1 > 16 then n_sa <= to_unsigned(16, 5);
                    else n_sa <= unsigned(v_m1(4 downto 0)); end if;
                when 27 =>                    -- the sun's gradient: accent + (A - accent) * ts >> 6
                    n_hw2 <= to_signed(C_SUNR2, 21) - signed(resize(m_acc(19 downto 0), 21));
                    m_a <= unsigned(resize(c_aay, 44)); m_b <= resize(e_ts, 24); m_acc <= (others => '0');
                when 29 =>
                    e_dv <= '0';
                when 52 =>
                    n_d1y <= c_dy(2) + signed(m_acc(16 downto 6));
                    m_a <= unsigned(resize(c_aau, 44)); m_b <= resize(e_ts, 24); m_acc <= (others => '0');
                when 77 =>
                    n_d1u <= c_du(2) + signed(m_acc(16 downto 6));
                    m_a <= unsigned(resize(c_aav, 44)); m_b <= resize(e_ts, 24); m_acc <= (others => '0');
                when 102 =>                   -- du (cells per h, Q32) = Uk * q >> 10
                    n_d1v <= c_dv(2) + signed(m_acc(16 downto 6));
                    m_a <= resize(c_uk, 44); m_b <= resize(dv_q, 24); m_acc <= (others => '0');
                when 127 =>                   -- du per pixel (Q24) = du * xs >> 8
                    m_a <= resize(m_acc(41 downto 10), 44); m_b <= resize(s_xs, 24); m_acc <= (others => '0');
                when 152 =>                   -- u at the pre-pixel = 1/2 - du_px * npre
                    e_dupx <= m_acc(33 downto 8);
                    m_a <= resize(m_acc(31 downto 8), 44); m_b <= resize(w_npre, 24); m_acc <= (others => '0');
                when 153 =>
                    n_du <= e_dupx(23 downto 0);
                    e_r16 <= e_dupx(25 downto 10); e_nr <= e_dupx(25 downto 10); e_np <= (others => '1');
                when 154 =>
                    n_alu <= f_al(e_r16); n_hwu <= f_hw(e_r16);
                when 170 =>
                    n_pu <= e_np; n_nu <= not e_nr(14 downto 13);
                when 177 =>                   -- z (cells, Q14) = Zk * q >> 17
                    n_u0 <= to_unsigned(8388608, 24) - m_acc(23 downto 0);
                    m_a <= resize(c_zk, 44); m_b <= resize(dv_q, 24); m_acc <= (others => '0');
                when 202 =>                   -- cells per virtual row = z * q >> 24
                    if m_acc(43 downto 39) /= "00000" then v_zl := (others => '1'); else v_zl := m_acc(38 downto 17); end if;
                    e_zl <= v_zl;
                    m_a <= resize(v_zl, 44); m_b <= resize(dv_q, 24); m_acc <= (others => '0');
                when 203 =>
                    n_zf <= e_zl(13 downto 0) + c_ph(28 downto 15);
                    v_zd := signed(resize(e_zl, 23)) - signed(resize(c_zc, 23));
                    if s_rad = '0' then e_zt <= e_zl(21 downto 6);
                    elsif v_zd(22) = '1' then e_zt <= unsigned(not v_zd(21 downto 6));
                    else e_zt <= unsigned(v_zd(21 downto 6)); end if;
                when 227 =>                   -- cells per line
                    m_a <= resize(m_acc(43 downto 24), 44); m_b <= resize(s_ys, 24); m_acc <= (others => '0');
                when 252 =>                   -- widened by the wave's worst compression (so no row gets thinner than a line)
                    if m_acc(31 downto 24) /= x"00" then v_r16 := (others => '1'); else v_r16 := m_acc(23 downto 8); end if;
                    m_a <= resize(v_r16, 44); m_b <= resize(c_gq, 24); m_acc <= (others => '0');
                when 277 =>                   -- theta at the centre = phase +- Spread * z
                    if m_acc(22 downto 20) /= "000" then v_r16 := (others => '1'); else v_r16 := m_acc(19 downto 4); end if;
                    e_r16 <= v_r16; e_nr <= v_r16; e_np <= (others => '1');
                    m_a <= resize(c_sp, 44); m_b <= resize(e_zt, 24); m_acc <= (others => '0');
                when 278 =>
                    n_alz <= f_al(e_r16); n_hwz <= f_hw(e_r16);
                when 294 =>
                    n_pz <= e_np; n_nz <= not e_nr(14 downto 13);
                when 302 =>                   -- theta step per pixel = kx * du_px >> 22
                    n_thl <= f_addsub(c_ph(31 downto 14), m_acc(23 downto 6), s_rad);
                    m_a <= resize(e_dupx, 44); m_b <= resize(c_kx, 24); m_acc <= (others => '0');
                when 327 =>                   -- and its value at the pre-pixel
                    n_dJ <= m_acc(41 downto 22);
                    m_a <= resize(m_acc(41 downto 22), 44); m_b <= resize(w_npre, 24); m_acc <= (others => '0');
                when 352 =>
                    n_J0 <= to_unsigned(0, 26) - m_acc(25 downto 0);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [C] ENGINE, beam: the line constants are taken over at g_start; free-running DDAs, loaded with
    -- the pre-pixel's values so that pixel 0 arrives at each one's stage.
    --   J = the x part of theta (signed), stage 2      U = the lane coordinate, stage 14
    --   Q = x^2 for the sun, stage 18
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                l_du <= n_du; l_dJ <= n_dJ; l_thl <= n_thl; l_zf <= n_zf;
                l_alz <= n_alz; l_hwz <= n_hwz; l_pz <= n_pz; l_nz <= n_nz;
                l_alu <= n_alu; l_hwu <= n_hwu; l_pu <= n_pu; l_nu <= n_nu;
                l_gl <= n_gl; l_ceil <= n_ceil; l_par <= n_par;
            end if;
            if g_start = '1' then b_J <= n_J0; else b_J <= b_J + resize(l_dJ, 26); end if;
            if s_st(11) = '1' then b_U <= n_u0; else b_U <= b_U + l_du; end if;
            if s_st(15) = '1' then
                b_Q <= w_qx0; b_D <= to_unsigned(0, 22) - resize(w_d0m, 22);
            else
                b_Q <= b_Q + b_D; b_D <= b_D + resize(w_dd, 22);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [C] ENGINE, pixel path (stages 3 .. 21)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_aj : unsigned(17 downto 0);
        variable v_fw : unsigned(21 downto 0);
        variable v_hs : unsigned(12 downto 0);
        variable v_ag : unsigned(5 downto 0);
        variable v_c  : signed(23 downto 0);
        variable v_gd : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: theta = centre value +- |J|
            if b_J(25) = '1' then v_aj := not b_J(17 downto 0); else v_aj := b_J(17 downto 0); end if;
            s3_th <= f_addsub(l_thl, v_aj, s_rad);
            -- 4: quarter-wave address
            s4_neg <= s3_th(17);
            if s3_th(16) = '1' then sn_a <= not s3_th(15 downto 6); else sn_a <= s3_th(15 downto 6); end if;
            -- 5: the sine ROM's own register (p_rom)
            s5_neg <= s4_neg;
            -- 6: fabric copy, Smooth / Step (12-bit magnitude)
            if s_stp = '1' then s6_mag <= sn_q(13) & "10000000000"; else s6_mag <= sn_q(13 downto 2); end if;
            s6_neg <= s5_neg;
            -- 7: Depth * (low half) and Depth * (high half) from the Fold tables
            s7_neg <= s6_neg;
            -- 8: fabric copies
            s8_lo <= fl_q; s8_hi <= fh_q; s8_neg <= s7_neg;
            -- 9: the wave's height (cells, Q12)
            v_fw := resize(s8_lo, 22) + (s8_hi & "000000");
            s9_fw <= v_fw(21 downto 7);
            s9_neg <= s8_neg;
            -- 10: height = +-wave + source
            if s_src = '1' then v_hs := s9_l & "00000"; else v_hs := "00" & s9_l & "000"; end if;
            s10_h <= signed(f_addsub(resize(v_hs, 16), resize(s9_fw, 16), s9_neg));
            -- 11: z' = z - h
            s11_zp <= l_zf - (unsigned(s10_h(11 downto 0)) & "00");
            -- 12 .. 15: the z rows
            s12_ez <= f_edge(s11_zp, l_hwz);
            s13_wz <= f_bar(s12_ez, l_pz);
            s14_vz <= f_man(s13_wz, l_nz);
            s15_gz <= f_cov(s14_vz, l_alz);
            s16_gz <= s15_gz; s17_gz <= s16_gz; s18_gz <= s17_gz;
            -- 15 .. 18: the lanes
            s15_eu <= f_edge(b_U(23 downto 10), l_hwu);
            s16_wu <= f_bar(s15_eu, l_pu);
            s17_vu <= f_man(s16_wu, l_nu);
            s18_gu <= f_cov(s17_vu, l_alu);
            -- 19: the grid's alpha; the sun's disc; 20: copies
            v_ag := resize(s18_gz, 6) + resize(s18_gu, 6);
            if v_ag > 16 then s19_ag <= to_unsigned(16, 5); else s19_ag <= v_ag(4 downto 0); end if;
            s19_e <= resize(n_hw2, 24) - signed(resize(b_Q, 24));
            if s_xs = 2 then v_c := shift_right(s19_e, 7) + 8; else v_c := shift_right(s19_e, 8) + 8; end if;
            if v_c < 0 then s20_cs <= (others => '0');
            elsif v_c > 16 then s20_cs <= to_unsigned(16, 5);
            else s20_cs <= unsigned(v_c(4 downto 0)); end if;
            s20_ag <= s19_ag;
            -- 21: the two candidates.  Grid (floor, tunnel) or sky tint (the source, read 11 pixels ahead),
            -- plus the horizon glow, checkerboard-dithered (every row has the same mean); the sun.
            if s_tun = '1' or l_ceil = '0' then v_ag := '0' & s20_ag; else v_ag := "00" & s9_l(7 downto 4); end if;
            v_gd := resize(l_gl, 7) + resize((lb_rc(0) xor l_par) & '1', 7);
            v_ag := v_ag + resize(v_gd(6 downto 2), 6);
            if v_ag > 16 then s21_a2 <= to_unsigned(16, 5); else s21_a2 <= v_ag(4 downto 0); end if;
            if s_sunoff = '1' or l_ceil = '0' then s21_a1 <= (others => '0');
            elsif s20_cs > n_sa then s21_a1 <= n_sa;
            else s21_a1 <= s20_cs; end if;

            -- Fold table fill (zone, zc 200 .. 264): T(i) = (F * i) >> 2 by accumulation
            ft_we <= '0';
            if s_zone = '1' and s_zc = 200 then
                ft_run <= '1'; ft_i <= (others => '0'); ft_acc <= (others => '0');
            elsif ft_run = '1' then
                ft_we <= '1'; ft_wa <= ft_i; ft_wd <= ft_acc(17 downto 2);
                ft_acc <= ft_acc + c_F; ft_i <= ft_i + 1;
                if ft_i = 63 then ft_run <= '0'; end if;
            end if;

            -- column counter of the luma line store's pixel-path read
            if s_st(3) = '1' or s_st(4) = '1' then lb_rc <= (others => '0'); else lb_rc <= lb_rc + 1; end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 64-px box, edge-replicated and reloaded at every
    -- line start -> vertical 1/8 IIR in the line store (half horizontal resolution).  The pixel path
    -- reads the store and interpolates: s9_l at stage 9.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d : signed(9 downto 0);
        variable v_a : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
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

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 64 samples; the samples before the line start count as the first one
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(193, 8); end if;
            if s_bcnt < 64 then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                s_bsum <= shift_left(resize(s_sl, 14), 6);
                s_bcnt <= to_unsigned(1, 7);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 64 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on the odd columns: q += (box - q + 4) >> 3
            s_eb8 <= s_bsum(13 downto 6); s_eq <= lb_q1;
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb8, 10)) - signed(resize(s_eq, 10)) + 4;
            v_a := unsigned(signed(resize(s_eq, 9)) + resize(shift_right(v_d, 3), 9));
            lb_wd <= v_a(7 downto 0);
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- pixel path: 8 fabric copies of this and the last read; 9 the mean
            s8_q <= lb_q2; s8_qp <= s8_q;
            v_a := resize(s8_q, 9) + resize(s8_qp, 9) + 1;
            if s_gav_sr(6) = '1' then s9_l <= v_a(8 downto 1); end if;      -- held past the line end
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] BLEND (stages 22 .. 25): base + (difference * alpha + 8) / 16.  The sun (difference 1) wins
    -- where its alpha is at least the grid's / the sky tint's (difference 2).
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_g  : signed(15 downto 0);
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            -- 22
            if s21_a1 /= 0 and s21_a1 >= s21_a2 then
                s22_cov <= s21_a1; s22_dy <= n_d1y; s22_du <= n_d1u; s22_dv <= n_d1v;
            else
                s22_cov <= s21_a2; s22_dy <= w_d2y; s22_du <= w_d2u; s22_dv <= w_d2v;
            end if;
            -- 23: difference * alpha as two sums of gated shifts
            if s22_cov(4) = '1' then
                s23_ay <= shift_left(resize(s22_dy, 16), 4); s23_by <= (others => '0');
                s23_au <= shift_left(resize(s22_du, 16), 4); s23_bu <= (others => '0');
                s23_av <= shift_left(resize(s22_dv, 16), 4); s23_bv <= (others => '0');
            else
                if s22_cov(3) = '1' then v_g := shift_left(resize(s22_dy, 16), 3); else v_g := (others => '0'); end if;
                if s22_cov(2) = '1' then s23_ay <= v_g + shift_left(resize(s22_dy, 16), 2); else s23_ay <= v_g; end if;
                if s22_cov(1) = '1' then v_g := shift_left(resize(s22_dy, 16), 1); else v_g := (others => '0'); end if;
                if s22_cov(0) = '1' then s23_by <= v_g + resize(s22_dy, 16); else s23_by <= v_g; end if;
                if s22_cov(3) = '1' then v_g := shift_left(resize(s22_du, 16), 3); else v_g := (others => '0'); end if;
                if s22_cov(2) = '1' then s23_au <= v_g + shift_left(resize(s22_du, 16), 2); else s23_au <= v_g; end if;
                if s22_cov(1) = '1' then v_g := shift_left(resize(s22_du, 16), 1); else v_g := (others => '0'); end if;
                if s22_cov(0) = '1' then s23_bu <= v_g + resize(s22_du, 16); else s23_bu <= v_g; end if;
                if s22_cov(3) = '1' then v_g := shift_left(resize(s22_dv, 16), 3); else v_g := (others => '0'); end if;
                if s22_cov(2) = '1' then s23_av <= v_g + shift_left(resize(s22_dv, 16), 2); else s23_av <= v_g; end if;
                if s22_cov(1) = '1' then v_g := shift_left(resize(s22_dv, 16), 1); else v_g := (others => '0'); end if;
                if s22_cov(0) = '1' then s23_bv <= v_g + resize(s22_dv, 16); else s23_bv <= v_g; end if;
            end if;
            -- 24
            s24_y <= s23_ay + s23_by;
            s24_u <= s23_au + s23_bu;
            s24_v <= s23_av + s23_bv;
            -- 25: base + (difference * alpha + 8) / 16; the base colour outside pixels 0 .. W + 7
            v_win := s_gav_sr(22) or s_gav_sr(30);
            if v_win = '1' then
                v_g := signed(resize(c_gy & "1000", 16)) + s24_y; s25_y <= unsigned(v_g(13 downto 4));
                v_g := signed(resize(c_gu & "1000", 16)) + s24_u; s25_u <= unsigned(v_g(13 downto 4));
                v_g := signed(resize(c_gv & "1000", 16)) + s24_v; s25_v <= unsigned(v_g(13 downto 4));
            else
                s25_y <= c_gy; s25_u <= c_gu; s25_v <= c_gv;
            end if;
            s_hist <= s_hist(6 downto 0) & v_win;
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 26 .. 29): chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent running
    -- 8-sums (resynced to the line's base colour on every line), luma [1 2 1]/4 delayed to the chroma's
    -- group delay (4 pixels), luma 13/16 toward black, blanking gate.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 26
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

end architecture outrun;
