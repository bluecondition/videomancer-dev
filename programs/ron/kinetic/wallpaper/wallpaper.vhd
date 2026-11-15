-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Wallpaper v0.1.0 -- Kinetic.  The wallpaper groups: one small asymmetric motif (a disc, a bar and a wedge)
-- repeated under every combination of translations, rotations, mirrors and glides that a square lattice
-- allows.  Twelve groups in rising symmetry: p1 p2 pm pg cm pmm pmg pgg cmm p4 p4g p4m.
--
-- Element: a lattice cell (2 x 2 quadrants, one motif copy per quadrant).  State: the motif's own rotation
-- (the wave) and its scale (the source); the copies in the cell follow through the group's mirrors and turns.
--
--   (U, V)    = the lattice coordinate (an accumulator pair: the lattice can turn and breathe)
--   (a, b)    = the position inside the quadrant, folded by the group's element for that quadrant (a flip of
--               a, a flip of b, a swap), then across the quadrant's diagonal for p4g / p4m
--   (x, y)    = rot(-psi_cell) * (a, b) * g_quadrant          psi = Depth * wave(phase + Spread * index)
--   d         = min(bar, wedge, disc signed distances at (x, y)) + erosion       coverage = clamp(1/2 - d / px)
--
--   K1  Scale    cell pitch 144 .. 576 virtual px (exponential, glided)
--   K2  Shape    the motif: Pin .. Flag .. Sail .. Dot .. Slab, interpolated
--   K3  Depth    the wave's swing of the motif's rotation, square law, 0 .. half a turn
--   K4  Rate     wave travel, bipolar, centre = still
--   K5  Colour   the 16 shared Kinetic palettes (hysteresis)  [K6 camera pass: was K6; the old K5 Spread,
--                the phase step per cell (square law, 0 .. 1/2 cycle), is fixed at its default 500]
--   K6  Camera   how strongly the held source sets the motif's size: linear gain G = K6 / 256 on the size
--                code c (g = 31 - c, 16 = the full quadrant).  Video: c = 7 + G 9 L / 16; Osc: c = 7 + G (L - 7)
--                (twice-ish the camera's swing, as before); c clamped to 0 .. 23 (g 31 .. 8: motifs from about
--                half a quadrant up to twice it, cropped at the quadrant edge).  0 = none (every motif at the
--                black-input size, g 24), 25 % = v0.1.0 exactly, 50 % (default) = twice.  Glided in the shared
--                glide (channel 3, formerly Spread); one 4 x 8 multiply on the source side before the hold RAM.
--   S7  Wave     Smooth (sine) / Snap (dwell and flip)
--   S8  Field    Linear (index = i + j) / Radial (|i| + |j| from the screen centre)
--   S9  Spin     Off / On: the lattice turns and breathes
--   S10 Mesh     Off / On: the quadrant boundaries (and the fold diagonals) as thin accent lines
--   S11 Source   Video / Osc: luma held per quadrant -> motif scale
--   P12 SYMMETRY the 12 groups, with hysteresis and an eased hand-over (the motif erodes away, the group
--                changes, the motif grows back)
--
-- The chassis is riley's (Kinetic pilot): [A] timing and virtual coordinates, [B] controls, the per-field
-- sequencer and the per-line engine of [C], the Osc and box of [D], the blend of [E] and [F] output
-- conditioning.  The integer model .kinetic_work/rb_wallpaper/imodel.py mirrors every section.
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixels 0 .. W + 7 are rendered, and output column j is the 9-tap filter
-- over pixels j .. j + 8.  s_gav_sr(S - 2) flags pixels 0 .. W - 1 at stage S.  One latency (35) in every mode.
-- Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 500 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.wallpaper_pkg.all;
use work.all;

architecture wallpaper of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(500, 10));   -- K6 camera pass: fixed knob
    constant C_LAT : integer := 35;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 850;
    constant C_DEF_K2  : integer := 256;
    constant C_DEF_K3  : integer := 480;
    constant C_DEF_K6  : integer := 512;   -- K6 Camera (glide channel 3, formerly K5 Spread)

    constant C_CY8 : integer := 138240;     -- screen centre y, Q8 virtual lines (540)

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
    -- |v| as v xor sign (a negative v gives -v - 1), without the sign bit
    function f_absx(v : signed) return unsigned is
        variable v_r : unsigned(v'length - 2 downto 0);
    begin
        if v(v'high) = '1' then v_r := unsigned(not v(v'high - 1 downto v'low));
        else v_r := unsigned(v(v'high - 1 downto v'low)); end if;
        return v_r;
    end function;
    -- quarter-wave address of a 4096-step angle
    function f_fold(a : unsigned(11 downto 0)) return unsigned is
    begin
        if a(10) = '1' then return not a(9 downto 0); else return a(9 downto 0); end if;
    end function;

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
    signal w_dxm  : unsigned(10 downto 0);     -- the beam's pre-pixel: its distance left of the centre (h)

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

    -- [B] knobs: glided (Q3), raw rate, switches, palette, group
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal s_gcam : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);   -- K6 Camera, glided (Q3)
    signal c_cm   : unsigned(7 downto 0) := to_unsigned(72, 8);               -- size-code gain, x / 64
    signal c_cb   : signed(5 downto 0) := to_signed(7, 6);                    -- size-code base
    signal s_kq   : unsigned(7 downto 0) := (others => '0');                  -- 9 K6 / 64 (Video) or 7 K6 / 256 (Osc)
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(1 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_spin, s_mesh, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := (others => '0');
    signal s_grp, s_tgt : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal s_first : std_logic := '1';
    signal c_ero : unsigned(7 downto 0) := (others => '0');       -- hand-over erosion, motif units

    -- ROMs: C_WP_TAB (2^(i/256), the palettes, the figures), C_WP_SIN (a quarter wave), C_WP_SQ (squares)
    signal r_ex_a : unsigned(9 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q : unsigned(15 downto 0) := (others => '0');
    signal sq_ax, sq_ay : unsigned(8 downto 0) := (others => '0');
    signal sq_qx, sq_qy : unsigned(15 downto 0) := (others => '0');

    -- per-field serial multiplier and scratch
    signal f_a, f_acc : unsigned(39 downto 0) := (others => '0');
    signal f_b : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_sh   : signed(3 downto 0) := (others => '0');
    signal z_sh4  : unsigned(2 downto 0) := (others => '0');
    signal z_s3, z_cm, z_sm : unsigned(13 downto 0) := (others => '0');
    signal z_n3, z_cn, z_sn : std_logic := '0';
    signal z_l2   : unsigned(5 downto 0) := (others => '0');

    -- per-field constants
    signal c_phi  : unsigned(15 downto 0) := to_unsigned(6000, 16);   -- spin angle (a 16-bit turn)
    signal c_k, c_kz : unsigned(19 downto 0) := (others => '0');      -- cells per h, Q26 (kz: with the breathing)
    signal c_A, c_B : unsigned(19 downto 0) := (others => '0');       -- |k cos|, |k sin|
    signal c_sA, c_sB : std_logic := '0';
    signal c_stu, c_stv : unsigned(22 downto 0) := (others => '0');   -- |step per pixel|, Q6.26 cells (signs: sA, not sB)
    signal c_lu, c_lv : unsigned(23 downto 0) := (others => '0');     -- |step per line| (signs: sB, sA)
    constant c_sp : unsigned(15 downto 0) := to_unsigned(7812, 16);   -- spread, Q16 cycles per cell: Spread fixed at 500 (500^2 >> 5)
    signal c_dep  : unsigned(11 downto 0) := (others => '0');         -- swing, 4096ths of a turn
    signal c_rstep : unsigned(27 downto 0) := (others => '0');        -- phase step per field, Q32
    signal c_ph   : unsigned(31 downto 0) := (others => '0');         -- wave phase, Q32 cycles
    signal c_elat : signed(3 downto 0) := (others => '0');            -- AA code: half-octaves of 1/16 px per unit
    type t_par is array (0 to 9) of signed(9 downto 0);
    signal c_par  : t_par := (others => (others => '0'));             -- 9 cx, 8 cy, 7 R, 6 bx, 5 by, 4 hw, 3 hh, 2 x0, 1 y0, 0 c
    signal c_R2q  : unsigned(13 downto 0) := (others => '0');         -- R^2 / 4
    signal c_cls  : unsigned(1 downto 0) := (others => '0');          -- 2 R near 181 / 256 / 362

    -- the second sequencer (figure interpolation, wave table fill) and its serial multiplier
    signal x_run : std_logic := '0';
    signal x_ph  : unsigned(1 downto 0) := (others => '0');
    signal x_n   : unsigned(5 downto 0) := (others => '0');
    signal x_c   : unsigned(4 downto 0) := (others => '0');
    signal x_k0  : signed(9 downto 0) := (others => '0');
    signal x_th  : unsigned(15 downto 0) := (others => '0');
    signal x_psi : unsigned(11 downto 0) := (others => '0');
    signal x_neg, x_cneg, x_sneg : std_logic := '0';
    signal x_cos : signed(7 downto 0) := (others => '0');

    -- [E] per-field palette: the ground, and a table of the 3 colours x 17 coverage steps, pre-blended
    signal c_gy, c_gu, c_gv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal t_dy, t_du, t_dv : signed(10 downto 0) := (others => '0');
    signal t_ay, t_au, t_av : unsigned(13 downto 0) := (others => '0');
    type t_bl is array (0 to 255) of unsigned(31 downto 0);
    signal bl_ram : t_bl := (others => (others => '0'));
    attribute no_rw_check of bl_ram : signal is true;
    signal bl_q, bl_wd : unsigned(31 downto 0) := (others => '0');
    signal bl_wa : unsigned(6 downto 0) := (others => '0');
    signal bl_we : std_logic := '0';

    -- the line start values (the lattice coordinate at the pre-pixel), stepped once per line
    signal n_U, n_V : unsigned(31 downto 0) := (others => '0');

    -- beam DDAs
    signal b_U, b_V : unsigned(31 downto 0) := (others => '0');

    -- wave table: (cos, sin) of the motif rotation by element index, refilled every field
    type t_wt is array (0 to 255) of unsigned(15 downto 0);
    signal wt_ram : t_wt := (others => (others => '0'));
    attribute no_rw_check of wt_ram : signal is true;
    signal wt_q, wt_wd : unsigned(15 downto 0) := (others => '0');
    signal wt_ra, wt_wa : unsigned(5 downto 0) := (others => '0');
    signal wt_we : std_logic := '0';

    -- hold RAM: the scale level of every quadrant, two banks by field (read the last field's, write this one's)
    type t_hd is array (0 to 2047) of unsigned(7 downto 0);   -- K6 pass: the 5-bit size code, padded to 8
    signal h_ram : t_hd := (others => (others => '0'));
    attribute no_rw_check of h_ram : signal is true;
    signal h_ra, h_wa : unsigned(10 downto 0) := (others => '0');
    signal h_q, h_wd : unsigned(7 downto 0) := (others => '0');
    signal h_we, s_bank : std_logic := '0';

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal s_lnow : unsigned(3 downto 0) := (others => '0');
    signal s_lp   : unsigned(11 downto 0) := (others => '0');   -- level x c_cm
    signal s_cnow : unsigned(4 downto 0) := to_unsigned(7, 5);  -- size code: c_cb + level x c_cm / 64, 0 .. 23

    -- geometry pipeline (stages 3 .. 21)
    signal g3_a, g3_b, g4_a, g4_b, g5_a, g5_b, g6_a, g6_b : signed(9 downto 0) := (others => '0');
    signal g3_ij : unsigned(9 downto 0) := (others => '0');
    signal g4_dm : unsigned(8 downto 0) := (others => '0');
    signal g4_t  : signed(10 downto 0) := (others => '0');
    signal g5_dg : unsigned(9 downto 0) := (others => '0');
    signal g5_dm : unsigned(8 downto 0) := (others => '0');
    signal g6_dm : unsigned(7 downto 0) := (others => '0');
    signal g7_wm : unsigned(8 downto 0) := (others => '0');
    signal g8_wm : unsigned(10 downto 0) := (others => '0');
    signal g9_cm, g20_cm : unsigned(4 downto 0) := (others => '0');
    signal g6_gc, g17_gc : std_logic := '0';
    signal g6_gp, g17_gp : std_logic := '0';   -- K6 pass: a motif enlarged past 1.41 x takes a half-octave finer AA
    -- delay rings (16 deep, one free-running pointer): mesh coverage + AA code bit (11 stages); filtered chroma (8)
    type t_dl is array (0 to 255) of unsigned(7 downto 0);
    signal dl_ram : t_dl := (others => (others => '0'));
    attribute no_rw_check of dl_ram : signal is true;
    signal dl_q : unsigned(7 downto 0) := (others => '0');
    type t_cd is array (0 to 255) of unsigned(15 downto 0);
    signal cd_ram : t_cd := (others => (others => '0'));
    attribute no_rw_check of cd_ram : signal is true;
    signal cd_q, cd_f : unsigned(15 downto 0) := (others => '0');
    signal r_wp : unsigned(3 downto 0) := (others => '0');
    signal g5_cs, g6_cs : unsigned(15 downto 0) := (others => '0');
    signal g5_L : unsigned(4 downto 0) := to_unsigned(7, 5);   -- the size code c (g = 31 - c)
    signal g6_g : unsigned(4 downto 0) := to_unsigned(24, 5);
    signal g7_ag, g7_bg : signed(10 downto 0) := (others => '0');
    signal g7_c, g7_s : signed(7 downto 0) := (others => '0');
    signal g8_p1, g8_p2, g8_p3, g8_p4 : signed(18 downto 0) := (others => '0');
    signal g9_x, g9_y : signed(11 downto 0) := (others => '0');
    signal g10_dx, g10_dy, g10_bx, g10_by, g10_w1, g10_w2, g10_s : signed(12 downto 0) := (others => '0');
    signal g11_bu, g11_bv, g11_m1, g11_t, g12_tk, g12_m1 : signed(12 downto 0) := (others => '0');
    signal g12_db, g13_db, g13_dw, g14_d01, g15_d01, g16_d01 : signed(9 downto 0) := (others => '0');
    signal g14_i01, g15_i01, g16_i01 : std_logic := '0';
    signal g13_sx, g13_sy : unsigned(15 downto 0) := (others => '0');
    signal g14_r2 : unsigned(16 downto 0) := (others => '0');
    signal g15_v4 : signed(15 downto 0) := (others => '0');
    signal g16_dd, g17_dm : signed(10 downto 0) := (others => '0');
    signal g17_id, g18_id, g19_id, g20_id : unsigned(1 downto 0) := (others => '0');
    signal g18_de : signed(8 downto 0) := (others => '0');
    signal g18_q, g19_q : signed(2 downto 0) := (others => '0');
    signal g18_r : std_logic := '0';
    signal g19_wv : signed(9 downto 0) := (others => '0');
    signal g20_w : signed(11 downto 0) := (others => '0');

    -- [E] blend
    signal s21_idx : unsigned(1 downto 0) := (others => '0');
    signal s21_cov : unsigned(4 downto 0) := (others => '0');
    signal b23 : unsigned(25 downto 0) := (others => '0');
    signal s25_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s25_u, s25_v : unsigned(7 downto 0) := to_unsigned(128, 8);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yd : t_u10(1 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv2, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1025, 11);
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
            r_ex_q <= C_WP_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_WP_SIN(to_integer(sn_a));
            sq_qx  <= C_WP_SQ(to_integer(sq_ax));
            sq_qy  <= C_WP_SQ(to_integer(sq_ay));
        end if;
    end process p_rom;

    -- wave table: written in the vblank zone only, read by the pixel path
    p_wtram : process(clk)
    begin
        if rising_edge(clk) then
            wt_q <= wt_ram(to_integer(resize(wt_ra, 8)));
            if wt_we = '1' then wt_ram(to_integer(resize(wt_wa, 8))) <= wt_wd; end if;
        end if;
    end process p_wtram;

    -- hold RAM: the read bank is never the write bank
    p_hram : process(clk)
    begin
        if rising_edge(clk) then
            h_q <= h_ram(to_integer(h_ra));
            if h_we = '1' then h_ram(to_integer(h_wa)) <= h_wd; end if;
        end if;
    end process p_hram;

    -- blend table: written in the vblank zone only, read by the pixel path (stage 21 address, stage 22 data)
    p_blram : process(clk)
    begin
        if rising_edge(clk) then
            bl_q <= bl_ram(to_integer(resize(s21_idx & s21_cov, 8)));
            if bl_we = '1' then bl_ram(to_integer(resize(bl_wa, 8))) <= bl_wd; end if;
        end if;
    end process p_blram;

    -- delay rings: read 9 (6) addresses behind the write = 11 (8) stages with the two registers
    p_dlram : process(clk)
    begin
        if rising_edge(clk) then
            dl_q <= dl_ram(to_integer(resize(r_wp - 9, 8)));
            dl_ram(to_integer(resize(r_wp, 8))) <= unsigned'("0") & g6_gp & g6_gc & g9_cm;
            cd_q <= cd_ram(to_integer(resize(r_wp - 6, 8)));
            cd_ram(to_integer(resize(r_wp, 8))) <= s25_u & s25_v;
            r_wp <= r_wp + 1;
        end if;
    end process p_dlram;

    -- box delay: read 32 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    -- [A] the beam's pre-pixel by video mode
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + the per-field
    -- sequencers (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_c  : signed(10 downto 0);
        variable v_r  : unsigned(27 downto 0);
        variable v_pi : unsigned(3 downto 0);
        variable v_pd : signed(10 downto 0);
        variable v_a12 : unsigned(11 downto 0);
        variable v_p3 : unsigned(15 downto 0);
        variable v_3  : unsigned(11 downto 0);
        variable v_td : signed(12 downto 0);
        variable v_b2 : unsigned(1 downto 0);
        variable v_ms : unsigned(5 downto 0);
        variable v_el : signed(7 downto 0);
        variable v_sg : unsigned(2 downto 0);
        variable v_fx : unsigned(9 downto 0);
        variable v_pm : unsigned(11 downto 0);
        variable v_mg : signed(7 downto 0);
        variable v_df : signed(10 downto 0);
        variable v_zu, v_zv, v_sub : std_logic;
        variable v_op : unsigned(31 downto 0);
        variable v_blk : unsigned(1 downto 0);
        variable v_st : unsigned(4 downto 0);
        variable v_pa : unsigned(7 downto 0);
        variable v_d11 : signed(10 downto 0);
        variable v_m17 : unsigned(16 downto 0);
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
                s_snap <= registers_in(6)(0);
                s_rad  <= registers_in(6)(1);
                s_spin <= registers_in(6)(2);
                s_mesh <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_bank <= not s_bank;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- palette: 16 steps with hysteresis (leave a step only 8 past its edge)
                v_k := unsigned(registers_in(4));
                v_c := signed(resize(v_k, 11)) - signed(resize(s_pal & "100000", 11));
                if v_c > 40 or v_c < -40 then s_pal <= v_k(9 downto 6); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "00"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when others => s_gm <= s_gcam; s_gr <= unsigned(registers_in(5));   -- K6 Camera
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        -- deadband +-2 knob steps, +-4 on the camera (channel 3) so pot noise cannot shimmer
                        if (s_gd > 32 or s_gd < -32) or (z_gc /= 3 and (s_gd > 16 or s_gd < -16)) then
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
                            when others => s_gcam <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 3 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k is decoded at zc = 30 + 24 k, loads at 31 + 24 k, 16 iterations, result
            -- stable from load + 17 (stored at 50 + 24 k).  The second sequencer (below) borrows the multiplier
            -- after the last slot.
            z_pre <= '0';
            if s_zone = '1' and s_zc(10 downto 9) = "00" then
                case to_integer(s_zc(8 downto 0)) is
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
                    -- (slot 12, Spread^2, is gone: Spread is fixed, c_sp is a constant)
                    when 342 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            if z_pre = '1' then
                f_acc <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => f_a <= resize(s_g1, 40);  f_b <= to_unsigned(513, 16);                  -- octaves above the smallest pitch
                    when 1  => f_a <= resize(c_k, 40);   f_b <= resize(z_s3, 16);                      -- the breathing
                    when 2  => f_a <= resize(c_kz, 40);  f_b <= resize(z_cm, 16);                      -- |k cos|
                    when 3  => f_a <= resize(c_kz, 40);  f_b <= resize(z_sm, 16);                      -- |k sin|
                    when 4  => f_a <= resize(c_A, 40);   f_b <= resize(w_dxm, 16);                     -- line 0's pre-pixel: x part
                    when 5  => f_a <= resize(c_B, 40);   f_b <= resize(w_dxm, 16);
                    when 6  => f_a <= resize(c_B, 40);   f_b <= to_unsigned(1080, 16);                 -- y part (1080 h above the centre)
                    when 7  => f_a <= resize(c_A, 40);   f_b <= to_unsigned(1080, 16);
                    when 8  => f_a <= resize(c_A, 40);   f_b <= resize(s_xs, 16);                      -- steps per pixel
                    when 9  => f_a <= resize(c_B, 40);   f_b <= resize(s_xs, 16);
                    when 10 => f_a <= resize(c_B, 40);   f_b <= resize(s_ys, 16);                      -- steps per line (* 128)
                    when 11 => f_a <= resize(c_A, 40);   f_b <= resize(s_ys, 16);
                    when 13 => f_a <= resize(s_g3(12 downto 3), 40); f_b <= resize(s_g3(12 downto 3), 16);  -- depth^2
                    when 14 => f_a <= resize(s_vm, 40);  f_b <= to_unsigned(897, 16);                  -- rate exponent
                    when others => f_a <= resize(r_exf, 40); f_b <= to_unsigned(9321, 16);             -- rate step
                end case;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- the line start values: line 0's are built in the zone (two products each), then one step per
            -- line at every g_start.  One adder each.
            --   U = A dx + B dy,  V = -B dx + A dy   (dx = -dxm at the pre-pixel, dy = -1080 h on line 0)
            v_zu := '0'; v_zv := '0'; v_sub := '0';
            if s_zone = '1' and s_zc(10 downto 8) = "000" then
                case to_integer(s_zc(7 downto 0)) is
                    when 146 => v_zu := '1'; v_sub := not c_sA;
                    when 170 => v_zv := '1'; v_sub := c_sB;
                    when 194 => v_zu := '1'; v_sub := not c_sB;
                    when 218 => v_zv := '1'; v_sub := not c_sA;
                    when others => null;
                end case;
            end if;
            if v_zu = '1' then v_op := f_acc(31 downto 0); else v_op := resize(c_lu, 32); v_sub := c_sB; end if;
            if s_zone = '1' and s_zc = 100 then n_U <= (others => '0');
            elsif v_zu = '1' or g_start = '1' then n_U <= f_addsub(n_U, v_op, v_sub); end if;
            v_sub := '0';
            if s_zone = '1' and s_zc(10 downto 8) = "000" then
                case to_integer(s_zc(7 downto 0)) is
                    when 170 => v_sub := c_sB;
                    when 218 => v_sub := not c_sA;
                    when others => null;
                end case;
            end if;
            if v_zv = '1' then v_op := f_acc(31 downto 0); else v_op := resize(c_lv, 32); v_sub := c_sA; end if;
            if s_zone = '1' and s_zc = 100 then n_V <= (others => '0');
            elsif v_zv = '1' or g_start = '1' then n_V <= f_addsub(n_V, v_op, v_sub); end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- the spin angle's sines: 3 phi (breathing), cos phi, sin phi
                    when 2 =>
                        v_p3 := shift_left(c_phi, 1) + c_phi;
                        v_a12 := v_p3(15 downto 4);
                        sn_a <= f_fold(v_a12); z_n3 <= v_a12(11);
                    when 3 =>
                        v_a12 := c_phi(15 downto 4) + 1024;
                        sn_a <= f_fold(v_a12); z_cn <= v_a12(11);
                    when 4 =>
                        z_s3 <= sn_q(13 downto 0);
                        v_a12 := c_phi(15 downto 4);
                        sn_a <= f_fold(v_a12); z_sn <= v_a12(11);
                    when 5 => z_cm <= sn_q(13 downto 0);
                    when 6 => z_sm <= sn_q(13 downto 0);

                    -- K6 camera: size-code gain c_cm (x / 64) and base c_cb, G = K6 / 256:
                    -- Video c = 7 + L 36 G / 64 (= 9 K6 / 64 x L / 64); Osc c = 7 - 7 G + L 64 G / 64
                    when 30 =>                        -- one adder: Video 9 x (8 K6), Osc 7 x (8 K6)
                        v_m17 := f_addsub(resize(s_gcam & "000", 17), resize(s_gcam, 17), s_src);
                        if s_src = '1' then s_kq <= "000" & v_m17(15 downto 11); else s_kq <= v_m17(16 downto 9); end if;
                    when 31 =>
                        if s_src = '1' then
                            c_cm <= s_gcam(12 downto 5); c_cb <= to_signed(7, 6) - signed(resize(s_kq(4 downto 0), 6));
                        else
                            c_cm <= s_kq; c_cb <= to_signed(7, 6);
                        end if;
                    when 20 =>                        -- motion (last field's step)
                        c_ph <= f_addsub(c_ph, resize(c_rstep, 32), s_vneg);
                    when 21 =>
                        c_phi <= c_phi + 16;
                    when 22 =>                        -- P12: 12 groups with hysteresis (the first field latches outright)
                        v_3 := resize(unsigned(registers_in(7)) & '0', 12) + resize(unsigned(registers_in(7)), 12);
                        v_td := signed(resize(v_3, 13)) - signed(resize(s_tgt & "10000000", 13));
                        if s_first = '1' or v_td > 158 or v_td < -158 then s_tgt <= v_3(11 downto 8); end if;
                    when 23 =>                        -- hand-over: erode the motif away, change group, grow it back
                        if s_first = '1' then
                            s_grp <= s_tgt; s_first <= '0';
                        elsif s_grp /= s_tgt then
                            if c_ero >= 192 then s_grp <= s_tgt; else c_ero <= c_ero + 24; end if;
                        elsif c_ero /= 0 then
                            c_ero <= c_ero - 24;
                        end if;

                    -- pitch: cells per h = 2^26 / (2 * 144) * 2^-oct
                    when 50  => z_oct <= f_acc(23 downto 13);
                    when 51  =>
                        v_e := -signed(resize(z_oct, 12) + 44);
                        r_ex_a <= "00" & unsigned(v_e(7 downto 0)); z_sh <= v_e(11 downto 8);
                    when 54  =>
                        case to_integer(z_sh) is
                            when -1     => c_k <= resize(r_exf & "00", 20);
                            when -2     => c_k <= resize(r_exf & "0", 20);
                            when others => c_k <= resize(r_exf, 20);
                        end case;
                    when 74  =>
                        if s_spin = '1' then c_kz <= f_addsub(c_k, f_acc(35 downto 16), z_n3);
                        else c_kz <= c_k; end if;
                    when 76  =>                       -- 2 log2(kz), rounded to half-octaves
                        if c_kz(18) = '1' then v_ms := to_unsigned(36, 6); v_b2 := c_kz(17 downto 16);
                        elsif c_kz(17) = '1' then v_ms := to_unsigned(34, 6); v_b2 := c_kz(16 downto 15);
                        elsif c_kz(16) = '1' then v_ms := to_unsigned(32, 6); v_b2 := c_kz(15 downto 14);
                        else v_ms := to_unsigned(30, 6); v_b2 := c_kz(14 downto 13); end if;
                        case v_b2 is
                            when "00"   => z_l2 <= v_ms;
                            when "11"   => z_l2 <= v_ms + 2;
                            when others => z_l2 <= v_ms + 1;
                        end case;
                    when 77  =>
                        v_el := to_signed(36, 8) - signed(resize(z_l2, 8));
                        if s_xs = 4 then v_el := v_el - 3; elsif s_xs = 3 then v_el := v_el - 1; end if;
                        c_elat <= v_el(3 downto 0);
                    when 98  =>
                        if s_spin = '1' then c_A <= f_acc(33 downto 14); c_sA <= z_cn;
                        else c_A <= c_kz; c_sA <= '0'; end if;
                    when 122 =>
                        if s_spin = '1' then c_B <= f_acc(33 downto 14); c_sB <= z_sn;
                        else c_B <= (others => '0'); c_sB <= '0'; end if;
                    when 242 => c_stu <= f_acc(22 downto 0);
                    when 266 => c_stv <= f_acc(22 downto 0);
                    when 290 => c_lu <= f_acc(30 downto 7);
                    when 314 => c_lv <= f_acc(30 downto 7);
                    when 362 => c_dep <= f_acc(20 downto 9);
                    when 386 => r_ex_a <= "00" & f_acc(15 downto 8); z_sh4 <= f_acc(18 downto 16);
                    when 410 =>
                        v_r := resize(f_acc(29 downto 8), 28);
                        if s_vm = 0 then c_rstep <= (others => '0');
                        else c_rstep <= shift_left(v_r, to_integer(z_sh4)); end if;
                    when 420 =>                       -- start the second sequencer
                        x_run <= '1'; x_ph <= "00"; x_n <= (others => '0'); x_c <= (others => '0');
                    when others => null;
                end case;
            end if;

            -- the second sequencer (it borrows the serial multiplier above):
            --   phase 0: the ten figure parameters, interpolated between two neighbouring figures (14 clocks each)
            --   phase 1: R^2 / 4 and the disc's normalising class
            --   phase 2: the 64 wave-table entries: wave(theta) -> psi = swing * wave -> (cos psi, sin psi) (20 clocks each)
            wt_we <= '0';
            if x_run = '1' then
                x_c <= x_c + 1;
                case x_ph is
                    when "00" =>
                        v_sg := resize(s_g2(12 downto 11), 3);
                        case to_integer(x_c) is
                            when 0 => r_ex_a <= "100" & v_sg & x_n(3 downto 0);
                            when 1 => r_ex_a <= "100" & (v_sg + 1) & x_n(3 downto 0);
                            when 3 => x_k0 <= signed(r_exf(9 downto 0));
                            when 4 =>
                                v_df := resize(signed(r_exf(9 downto 0)), 11) - resize(x_k0, 11);
                                f_a <= unsigned(resize(v_df, 40));
                                f_b <= resize(s_g2(10 downto 3), 16);
                                f_acc <= (others => '0');
                            when 13 =>
                                c_par(0) <= x_k0 + signed(f_acc(17 downto 8));
                                for i in 1 to 9 loop c_par(i) <= c_par(i - 1); end loop;
                                x_c <= (others => '0');
                                if x_n = 9 then x_ph <= "01"; x_n <= (others => '0');
                                else x_n <= x_n + 1; end if;
                            when others => null;
                        end case;
                    when "01" =>
                        case to_integer(x_c) is
                            when 0 =>
                                f_a <= resize(unsigned(c_par(7)), 40);
                                f_b <= resize(unsigned(c_par(7)), 16);
                                f_acc <= (others => '0');
                            when 12 =>
                                c_R2q <= f_acc(15 downto 2);
                                if c_par(7) < 108 then c_cls <= "00";
                                elsif c_par(7) < 152 then c_cls <= "01";
                                else c_cls <= "10"; end if;
                                if s_rad = '1' then x_th <= c_ph(31 downto 16);
                                else x_th <= c_ph(31 downto 16) - (c_sp(10 downto 0) & "00000"); end if;
                                x_ph <= "10"; x_c <= (others => '0'); x_n <= (others => '0');
                            when others => null;
                        end case;
                    when others =>
                        case to_integer(x_c) is
                            when 0 =>
                                v_fx := f_fold(x_th(15 downto 4));
                                if s_snap = '1' then
                                    if v_fx(9 downto 8) /= "00" then v_fx := (others => '1');
                                    else v_fx := v_fx(7 downto 0) & "00"; end if;
                                end if;
                                sn_a <= v_fx; x_neg <= x_th(15);
                            when 2 =>
                                f_a <= resize(sn_q(13 downto 0), 40);
                                f_b <= resize(c_dep, 16);
                                f_acc <= (others => '0');
                            when 15 =>
                                v_pm := f_acc(25 downto 14);
                                if x_neg = '1' then x_psi <= to_unsigned(0, 12) - v_pm; else x_psi <= v_pm; end if;
                            when 16 =>
                                v_a12 := x_psi + 1024;
                                sn_a <= f_fold(v_a12); x_cneg <= v_a12(11);
                            when 17 =>
                                sn_a <= f_fold(x_psi); x_sneg <= x_psi(11);
                            when 18 =>
                                v_mg := signed(resize(sn_q(13 downto 7), 8));
                                if x_cneg = '1' then x_cos <= -v_mg; else x_cos <= v_mg; end if;
                            when 19 =>
                                v_mg := signed(resize(sn_q(13 downto 7), 8));
                                if x_sneg = '1' then v_mg := -v_mg; end if;
                                wt_wd <= unsigned(x_cos) & unsigned(v_mg);
                                wt_we <= '1';
                                if s_rad = '1' then
                                    wt_wa <= x_n; x_th <= x_th - c_sp;
                                else
                                    wt_wa <= x_n xor "100000"; x_th <= x_th + c_sp;
                                end if;
                                x_c <= (others => '0');
                                if x_n = 63 then x_run <= '0'; else x_n <= x_n + 1; end if;
                            when others => null;
                        end case;
                end case;
            end if;

            -- [E] palette and blend table (zc 1024 .. 1151): four blocks of 32 clocks.  Block 0 reads the ground;
            -- blocks 1 .. 3 read colour A / B / accent, take its difference from the ground and write the 17
            -- coverage steps  ground + (difference * cov + 8) / 16  by accumulation (Y 10 bits, U and V 8 bits).
            bl_we <= '0';
            if s_zone = '1' and s_zc(10 downto 7) = "1000" then
                v_blk := s_zc(6 downto 5); v_st := s_zc(4 downto 0);
                v_pa := resize(s_pal & "000", 8) + resize(s_pal & "00", 8) + resize(v_blk & '0', 8) + resize(v_blk, 8);
                case to_integer(v_st) is
                    when 0 => r_ex_a <= "01" & v_pa;
                    when 1 => r_ex_a <= "01" & (v_pa + 1);
                    when 2 => r_ex_a <= "01" & (v_pa + 2);
                    when 3 =>
                        v_d11 := signed(resize(r_exf(9 downto 0), 11)) - signed(resize(c_gy, 11));
                        if v_blk = 0 then c_gy <= r_exf(9 downto 0); else t_dy <= v_d11; t_ay <= c_gy & "1000"; end if;
                    when 4 =>
                        v_d11 := signed(resize(r_exf(9 downto 0), 11)) - signed(resize(c_gu, 11));
                        if v_blk = 0 then c_gu <= r_exf(9 downto 0); else t_du <= v_d11; t_au <= c_gu & "1000"; end if;
                    when 5 =>
                        v_d11 := signed(resize(r_exf(9 downto 0), 11)) - signed(resize(c_gv, 11));
                        if v_blk = 0 then c_gv <= r_exf(9 downto 0); else t_dv <= v_d11; t_av <= c_gv & "1000"; end if;
                    when 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 | 24 =>
                        if v_blk /= 0 then
                            bl_we <= '1';
                            bl_wa <= (v_blk - 1) & (v_st - 8);
                            bl_wd <= "000000" & t_ay(13 downto 4) & t_au(13 downto 6) & t_av(13 downto 6);
                            t_ay <= unsigned(signed(t_ay) + resize(t_dy, 14));
                            t_au <= unsigned(signed(t_au) + resize(t_du, 14));
                            t_av <= unsigned(signed(t_av) + resize(t_dv, 14));
                        end if;
                    when others => null;
                end case;
            end if;

            -- [A] sync delay and generated-avid shift registers (field_n is tied high by the core on every
            -- input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
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

    -- Beam: two free-running DDAs, loaded with the pre-pixel's values (pixel 0 at stage 2)
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_U <= n_U; b_V <= n_V;
            else
                b_U <= f_addsub(b_U, resize(c_stu, 32), c_sA);
                b_V <= f_addsub(b_V, resize(c_stv, 32), not c_sB);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- GEOMETRY (stages 3 .. 21)
    --    3  quadrant-local (a, b) with the group's element applied; wave-table and hold-RAM addresses
    --    4  the diagonal fold of p4g / p4m; the hold RAM's write (at the quadrant centre)
    --    5  fabric copies of the two RAM reads        6  the scale factor g
    --    7  (a, b) * g                                8  four products            9  (x, y) in the motif frame
    --   10 .. 16  the three signed distances          17 the nearest primitive    18 erosion
    --   19, 20  distance -> 1/16 px (half-octave code per element)                21 coverage, mesh on top
    ---------------------------------------------------------------------------
    p_geo : process(clk)
        variable v_qx, v_qy, v_r1, v_fx, v_fy, v_sw : std_logic;
        variable v_a, v_b : signed(9 downto 0);
        variable v_ic, v_jc : signed(5 downto 0);
        variable v_s11 : signed(10 downto 0);
        variable v_ua, v_ub : unsigned(8 downto 0);
        variable v_ad : unsigned(9 downto 0);
        variable v_g6 : signed(5 downto 0);
        variable v_p16 : signed(15 downto 0);
        variable v_sum : signed(19 downto 0);
        variable v_u12 : unsigned(11 downto 0);
        variable v_m13 : signed(12 downto 0);
        variable v_v4 : signed(17 downto 0);
        variable v_u : signed(15 downto 0);
        variable v_e12 : signed(11 downto 0);
        variable v_e4 : signed(3 downto 0);
        variable v_w9 : unsigned(8 downto 0);
        variable v_cov : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: the group's element for this quadrant: flip a, flip b, swap
            v_qx := b_U(25); v_qy := b_V(25); v_r1 := b_V(26);
            v_sw := '0';
            case to_integer(s_grp) is
                when 0      => v_fx := '0';            v_fy := '0';                          -- p1
                when 1      => v_fx := v_qx;           v_fy := v_qx;                         -- p2
                when 2      => v_fx := v_qx;           v_fy := '0';                          -- pm
                when 3      => v_fx := v_qy;           v_fy := '0';                          -- pg
                when 4      => v_fx := v_qx xor v_qy;  v_fy := '0';                          -- cm
                when 6      => v_fx := v_qx xor v_qy;  v_fy := v_qy;                         -- pmg
                when 7      => v_fx := v_qy;           v_fy := v_qx;                         -- pgg
                when 8      => v_fx := v_qx xor v_r1;  v_fy := v_qy;                         -- cmm
                when 9 | 10 => v_fx := v_qx;           v_fy := v_qy; v_sw := v_qx xor v_qy;  -- p4, p4g
                when others => v_fx := v_qx;           v_fy := v_qy;                         -- pmm, p4m
            end case;
            v_a := signed(b_U(24 downto 15)); v_a(9) := not v_a(9);
            v_b := signed(b_V(24 downto 15)); v_b(9) := not v_b(9);
            if v_fx = '1' then v_a := not v_a; end if;
            if v_fy = '1' then v_b := not v_b; end if;
            if v_sw = '1' then g3_a <= v_b; g3_b <= v_a; else g3_a <= v_a; g3_b <= v_b; end if;
            -- the element (a cell) and its index in the Field
            v_ic := signed(b_U(31 downto 26)); v_jc := signed(b_V(31 downto 26));
            if s_rad = '1' then wt_ra <= resize(f_absx(v_ic), 6) + resize(f_absx(v_jc), 6);
            else wt_ra <= unsigned(v_ic + v_jc); end if;
            h_ra  <= (not s_bank) & b_V(29 downto 25) & b_U(29 downto 25);
            g3_ij <= b_V(29 downto 25) & b_U(29 downto 25);

            -- 4: the diagonal fold; the hold RAM's write; the mesh's distances
            v_s11 := resize(g3_a, 11) + resize(g3_b, 11);
            if s_grp = 11 and g3_b > g3_a then
                g4_a <= g3_b; g4_b <= g3_a;
            elsif s_grp = 10 and v_s11(10) = '0' then
                g4_a <= not g3_b; g4_b <= not g3_a;
            else
                g4_a <= g3_a; g4_b <= g3_b;
            end if;
            v_ua := f_absx(g3_a); v_ub := f_absx(g3_b);
            if v_ua(8 downto 6) = "000" and v_ub(8 downto 6) = "000" and s_gav_sr(1) = '1' then h_we <= '1'; else h_we <= '0'; end if;
            h_wa <= s_bank & g3_ij;
            h_wd <= "000" & s_cnow;
            if v_ua > v_ub then g4_dm <= not v_ua; else g4_dm <= not v_ub; end if;
            if s_grp = 11 then g4_t <= resize(g3_a, 11) - resize(g3_b, 11);
            else g4_t <= v_s11 + 1; end if;

            -- 5: fabric copies of the wave table and the hold RAM
            g5_cs <= wt_q; g5_L <= h_q(4 downto 0);
            g5_a <= g4_a; g5_b <= g4_b;
            v_ad := f_absx(g4_t);
            g5_dg <= shift_right(v_ad, 1) + shift_right(v_ad, 3) + shift_right(v_ad, 4);
            g5_dm <= g4_dm;

            -- 6: the scale factor (16 = full size) g = 31 - c (the K6 gain is already in the stored size code:
            -- at K6 = 25 % Video 24 - 9 L / 16, Osc 31 - L, as v0.1.0); the AA code a half-octave coarser for a
            -- small motif (g >= 23), finer for a big one (g <= 11)
            g6_g <= not g5_L;
            if g5_L <= 8 then g6_gc <= '1'; else g6_gc <= '0'; end if;
            if g5_L >= 20 then g6_gp <= '1'; else g6_gp <= '0'; end if;
            g6_a <= g5_a; g6_b <= g5_b; g6_cs <= g5_cs;
            if (s_grp = 10 or s_grp = 11) and resize(g5_dg, 10) < resize(g5_dm, 10) then
                if g5_dg(9 downto 8) /= "00" then g6_dm <= (others => '1'); else g6_dm <= g5_dg(7 downto 0); end if;
            else
                if g5_dm(8) = '1' then g6_dm <= (others => '1'); else g6_dm <= g5_dm(7 downto 0); end if;
            end if;

            -- 7: (a, b) * g / 16
            v_g6 := signed(resize(g6_g, 6));
            v_p16 := g6_a * v_g6; g7_ag <= v_p16(14 downto 4);
            v_p16 := g6_b * v_g6; g7_bg <= v_p16(14 downto 4);
            g7_c <= signed(g6_cs(15 downto 8)); g7_s <= signed(g6_cs(7 downto 0));
            if c_elat(0) = '1' then g7_wm <= resize(g6_dm, 9) + resize(shift_right(g6_dm, 1), 9);
            else g7_wm <= resize(g6_dm, 9); end if;

            -- 8: the rotation's four products
            g8_p1 <= g7_ag * g7_c; g8_p2 <= g7_bg * g7_s;
            g8_p3 <= g7_ag * g7_s; g8_p4 <= g7_bg * g7_c;
            case to_integer(c_elat(3 downto 1)) is
                when 2      => g8_wm <= shift_left(resize(g7_wm, 11), 2);
                when 1      => g8_wm <= shift_left(resize(g7_wm, 11), 1);
                when 0      => g8_wm <= resize(g7_wm, 11);
                when -1     => g8_wm <= resize(shift_right(g7_wm, 1), 11);
                when -2     => g8_wm <= resize(shift_right(g7_wm, 2), 11);
                when others => g8_wm <= resize(shift_right(g7_wm, 3), 11);
            end case;

            -- 9: (x, y) in the motif frame; the mesh's coverage (a 2.5-pixel line)
            v_sum := resize(g8_p1, 20) + resize(g8_p2, 20); g9_x <= v_sum(18 downto 7);
            v_sum := resize(g8_p4, 20) - resize(g8_p3, 20); g9_y <= v_sum(18 downto 7);
            if s_mesh = '0' or g8_wm >= 28 then g9_cm <= (others => '0');
            elsif g8_wm <= 12 then g9_cm <= to_unsigned(16, 5);
            else g9_cm <= to_unsigned(28, 5) - g8_wm(4 downto 0); end if;
            -- the delay ring's fabric copies: g9_cm 11 stages on (stage 20), g6_gc 11 stages on (stage 17)
            g20_cm <= dl_q(4 downto 0); g17_gc <= dl_q(5); g17_gp <= dl_q(6);

            -- 10: offsets to the primitives
            g10_dx <= resize(g9_x, 13) - resize(c_par(9), 13);
            g10_dy <= resize(g9_y, 13) - resize(c_par(8), 13);
            g10_bx <= resize(g9_x, 13) - resize(c_par(6), 13);
            g10_by <= resize(g9_y, 13) - resize(c_par(5), 13);
            g10_w1 <= resize(c_par(2), 13) - resize(g9_x, 13);
            g10_w2 <= resize(c_par(1), 13) - resize(g9_y, 13);
            g10_s  <= resize(g9_x, 13) + resize(g9_y, 13);

            -- 11: disc: squares ROM addresses; bar: two slab distances; wedge: two legs, the hypotenuse's offset
            v_u12 := f_absx(g10_dx);
            if v_u12(11 downto 9) /= "000" then sq_ax <= (others => '1'); else sq_ax <= v_u12(8 downto 0); end if;
            v_u12 := f_absx(g10_dy);
            if v_u12(11 downto 9) /= "000" then sq_ay <= (others => '1'); else sq_ay <= v_u12(8 downto 0); end if;
            g11_bu <= signed(resize(f_absx(g10_bx), 13)) - resize(c_par(4), 13);
            g11_bv <= signed(resize(f_absx(g10_by), 13)) - resize(c_par(3), 13);
            if g10_w1 > g10_w2 then g11_m1 <= g10_w1; else g11_m1 <= g10_w2; end if;
            g11_t <= g10_s - resize(c_par(0), 13);

            -- 12: (the squares ROM's register); bar distance; the hypotenuse's distance (x 0.6875)
            if g11_bu > g11_bv then v_m13 := g11_bu; else v_m13 := g11_bv; end if;
            if v_m13 > 511 then g12_db <= to_signed(511, 10);
            elsif v_m13 < -512 then g12_db <= to_signed(-512, 10);
            else g12_db <= v_m13(9 downto 0); end if;
            g12_tk <= shift_right(g11_t, 1) + shift_right(g11_t, 3) + shift_right(g11_t, 4);
            g12_m1 <= g11_m1;

            -- 13: fabric copies of the squares; wedge distance
            g13_sx <= sq_qx; g13_sy <= sq_qy;
            if g12_m1 > g12_tk then v_m13 := g12_m1; else v_m13 := g12_tk; end if;
            if v_m13 > 511 then g13_dw <= to_signed(511, 10);
            elsif v_m13 < -512 then g13_dw <= to_signed(-512, 10);
            else g13_dw <= v_m13(9 downto 0); end if;
            g13_db <= g12_db;

            -- 14: r^2 / 4; the nearer of bar and wedge
            g14_r2 <= resize(g13_sx, 17) + resize(g13_sy, 17);
            if g13_dw < g13_db then g14_d01 <= g13_dw; g14_i01 <= '1';
            else g14_d01 <= g13_db; g14_i01 <= '0'; end if;

            -- 15: (r^2 - R^2) / 4
            v_v4 := signed(resize(g14_r2, 18)) - signed(resize(c_R2q, 18));
            if v_v4 > 32767 then g15_v4 <= to_signed(32767, 16);
            else g15_v4 <= v_v4(15 downto 0); end if;
            g15_d01 <= g14_d01; g15_i01 <= g14_i01;

            -- 16: the disc's distance: / (2 R), 2 R rounded to 181 / 256 / 362
            if c_cls = "10" then v_u := shift_right(g15_v4, 1); else v_u := g15_v4; end if;
            if c_cls = "01" then
                g16_dd <= resize(shift_right(v_u, 6), 11);
            else
                g16_dd <= resize(shift_right(v_u, 6), 11) + resize(shift_right(v_u, 8), 11) + resize(shift_right(v_u, 9), 11);
            end if;
            g16_d01 <= g15_d01; g16_i01 <= g15_i01;

            -- 17: the nearest primitive: 0 bar (colour A), 1 wedge (colour B), 2 disc (accent)
            if g16_dd < resize(g16_d01, 11) then g17_dm <= g16_dd; g17_id <= "10";
            else g17_dm <= resize(g16_d01, 11); g17_id <= '0' & g16_i01; end if;

            -- 18: erosion (the hand-over), clamped; the element's AA code
            v_e12 := resize(g17_dm, 12) + signed(resize(c_ero, 12));
            if v_e12 > 255 then g18_de <= to_signed(255, 9);
            elsif v_e12 < -256 then g18_de <= to_signed(-256, 9);
            else g18_de <= v_e12(8 downto 0); end if;
            if g17_gc = '1' then v_e4 := c_elat - 1;
            elsif g17_gp = '1' and c_elat < 5 then v_e4 := c_elat + 1;
            else v_e4 := c_elat; end if;
            g18_q <= v_e4(3 downto 1); g18_r <= v_e4(0);
            g18_id <= g17_id;

            -- 19: x 1.5 on the odd half-octaves
            if g18_r = '1' then g19_wv <= resize(g18_de, 10) + resize(shift_right(g18_de, 1), 10);
            else g19_wv <= resize(g18_de, 10); end if;
            g19_q <= g18_q; g19_id <= g18_id;

            -- 20: the octave shift: distance in 1/16 px
            case to_integer(g19_q) is
                when 2      => g20_w <= shift_left(resize(g19_wv, 12), 2);
                when 1      => g20_w <= shift_left(resize(g19_wv, 12), 1);
                when 0      => g20_w <= resize(g19_wv, 12);
                when -1     => g20_w <= resize(shift_right(g19_wv, 1), 12);
                when -2     => g20_w <= resize(shift_right(g19_wv, 2), 12);
                when others => g20_w <= resize(shift_right(g19_wv, 3), 12);
            end case;
            g20_id <= g19_id;

            -- 21: coverage 0 .. 16; the mesh wins where it covers more
            if g20_w >= 8 then v_cov := (others => '0');
            elsif g20_w <= -8 then v_cov := to_unsigned(16, 5);
            else v_cov := unsigned(to_signed(8, 5) - g20_w(4 downto 0)); end if;
            if g20_cm > v_cov then s21_cov <= g20_cm; s21_idx <= "10";
            else s21_cov <= v_cov; s21_idx <= g20_id; end if;
        end if;
    end process p_geo;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 32-px box, edge-replicated
    -- and reloaded at every line start -> a 4-bit level, written to the hold RAM
    -- when the beam crosses a quadrant's centre.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_b : unsigned(8 downto 0);
        variable v_c8 : signed(7 downto 0);
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

            -- level 0 .. 15 = box * 5 / 64
            v_b := resize(s_bsum(12 downto 5), 9) + resize(s_bsum(12 downto 7), 9);
            if v_b(8) = '1' then s_lnow <= (others => '1'); else s_lnow <= v_b(7 downto 4); end if;

            -- K6 camera: the size code c = c_cb + level x c_cm / 64, clamped to 0 .. 23 (g 31 .. 8).  One 4 x 8
            -- multiply in its own stage, then the add and clamp (the held sample moves 2 px right: invisible)
            s_lp <= s_lnow * c_cm;
            v_c8 := resize(c_cb, 8) + signed(resize(s_lp(11 downto 6), 8));
            if v_c8 < 0 then s_cnow <= (others => '0');
            elsif v_c8 > 23 then s_cnow <= to_unsigned(23, 5);
            else s_cnow <= unsigned(v_c8(4 downto 0)); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] AA BLEND (stages 22 .. 24): the pre-blended table (22: its register), a fabric copy (23), the ground
    -- colour outside pixels 0 .. W + 7 (24).  Chroma is carried at 8 bits from here to the output filter.
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            b23 <= bl_q(25 downto 0);
            v_win := s_gav_sr(21) or s_gav_sr(29);
            if v_win = '1' then
                s25_y <= b23(25 downto 16); s25_u <= b23(15 downto 8); s25_v <= b23(7 downto 0);
            else
                s25_y <= c_gy; s25_u <= c_gu(9 downto 2); s25_v <= c_gv(9 downto 2);
            end if;
            s_hist <= s_hist(6 downto 0) & v_win;
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 25 .. 28): chroma [1 2 2 2 2 2 2 2 1]/16 as
    -- two adjacent running 8-sums (resynced to the ground colour on every
    -- line), luma [1 2 1]/4 delayed to the chroma's group delay (4 pixels),
    -- luma 13/16 toward black, blanking gate.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- running 8-sums of the 8-bit chroma (+ 1: rounds the 9-tap sum); they snap to 8 x ground once 8 samples outside
            -- the picture are in, so a glitched sample can't offset the colours for good
            cd_f <= cd_q;                 -- the chroma of 8 pixels back (the delay ring's fabric copy)
            if s_hist = "00000000" then
                s_bu <= (c_gu(9 downto 2) & "000") + 1; s_bv2 <= (c_gv(9 downto 2) & "000") + 1;
            else
                s_bu  <= s_bu + s25_u - cd_f(15 downto 8);
                s_bv2 <= s_bv2 + s25_v - cd_f(7 downto 0);
            end if;
            s_bud <= s_bu; s_bvd <= s_bv2;
            -- luma taps y(n - 3), y(n - 4), y(n - 5)
            s_yd(1) <= s25_y;
            for i in 2 to 5 loop s_yd(i) <= s_yd(i - 1); end loop;
            s26_ya <= resize(s_yd(3), 12) + resize(s_yd(5), 12) + 2;
            s26_yb <= s_yd(4);
            -- 27
            v_sc := resize(s_bu, 12) + s_bud;  s27_u <= v_sc(11 downto 2);
            v_sc := resize(s_bv2, 12) + s_bvd; s27_v <= v_sc(11 downto 2);
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

end architecture wallpaper;
