-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Riley v0.1.0 -- Kinetic pilot.  After Bridget Riley's black-and-white wave paintings ("Current" 1964,
-- "Fall" 1963, "Cataract 3" 1967) and her zigzags: parallel bands whose shared wave tightens along its
-- length, so the eye sees shimmer and depth where the slopes crowd.
--
-- Element: a band.  State: its displacement across the band axis.
--
--   PHI(u, v) = v / pitch + Fold * wave(theta)       band = floor(PHI), edge distance from frac(PHI)
--   theta     = f0 (u + c u^2 / 2U) + Spread * v / pitch + phase                (Field = Linear)
--               f0 (1 + 3c/4) r^2 / 2R + (Spread / 4) * v / pitch + phase       (Field = Radial: a pond whose
--                                                                                 rings crowd outwards)
--   weight    = K3 + min(1, luma^2 * 2) * head-room   (the camera swells and thins the bands: an engraving)
--   coverage  = clamp(1/2 + (weight / 2 - |frac(PHI) - 1/2|) / |grad PHI|)      17 steps, per pixel
--
--   K1  Scale    band pitch 8 .. 160 virtual lines (exponential, glided; the wavelength follows as sqrt)
--   K2  Chirp    how much the wavelength tightens along the band (bipolar: tight left / even / tight right;
--                Radial: how fast the rings crowd)
--   K3  Weight   band duty 10 % .. 90 % (the source adds to it)
--   K4  Rate     wave travel, bipolar, centre = still (60 s per cycle .. 2 cycles per second)
--   K5  Spread   crest slant per band (Radial: slides the pond's centre along the band axis), square law
--   K6  Colour   the 16 shared Kinetic palettes (hysteresis)
--   S7  Wave     Smooth (sine) / Zigzag (triangle)
--   S8  Field    Linear / Radial
--   S9  Bands    Duo (ground + A) / Quad (bands step through A, B, accent, ground)
--   S10 Axis     Across (horizontal bands) / Down (vertical bands)
--   S11 Source   Video / Osc
--   P12 FOLD     wave depth, square law: ruled lines .. 6 bands (the bands fold over into moire)
--
-- The whole phase field is one bi-quadratic,  theta = J1 dx + J2 dx^2 + K1 dy + K2 dy^2 + phase  (dx, dy from
-- the screen centre), and the band coordinate is linear,  pv = jPV dx + kPV dy;  Field and Axis only choose
-- the six coefficients.  The x parts run as beam DDAs (second order for theta), the y parts are computed
-- per line and become the DDAs' start values, so the pixel path has no multiplier at all: Fold * wave comes
-- from a table refilled every field, and so does the weight curve.
--
-- The chassis, in sections (the integer model .kinetic_work/rb_riley/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         glides, switches, rate deadband, palette hysteresis
--   [C] WAVE ENGINE                      per-field sequencer (one serial multiplier), per-line engine (one
--                                        serial multiplier, computes the NEXT line's start values), beam
--                                        DDAs, theta -> wave -> Fold table -> PHI, the finite-difference
--                                        gradient (previous pixel, line above)
--   [D] SOURCE FIELD                     Osc lattice, 32-px box, vertical IIR line store, weight table
--   [E] PALETTE + AA BLEND               coverage divider, 17-step shift-add blend
--   [F] OUTPUT CONDITIONING              chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 at the same group delay,
--                                        luma 13/16, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixel n sits at x = x0 + n xs half virtual px, pixels 0 .. W + 7 are
-- rendered, and output column j is the 9-tap filter over pixels j .. j + 8 (centre j + 4), so both picture
-- edges are filtered from real content; the generated avid lands 4 clocks after the delayed source avid
-- (the shell's parity-locked start), which puts column j at screen column j + 4 and the picture centre on
-- the screen centre.  s_gav_sr(S - 2) flags pixels 0 .. W - 1 at stage S.  One latency (36) in every mode.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.riley_pkg.all;
use work.all;

architecture riley of program_top is
    constant C_LAT : integer := 36;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 415;
    constant C_DEF_K2  : integer := 760;
    constant C_DEF_K3  : integer := 384;
    constant C_DEF_K5  : integer := 330;
    constant C_DEF_P12 : integer := 384;

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
    signal s_zig, s_rad, s_quad, s_down, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := (others => '0');

    -- [C] ROMs: C_RL_TAB (2^(i/256), then the palettes), C_RL_SIN (a quarter wave)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_sh   : signed(3 downto 0) := (others => '0');
    signal z_sh4  : unsigned(2 downto 0) := (others => '0');
    signal z_cm   : unsigned(8 downto 0) := (others => '0');
    signal z_q    : unsigned(24 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_invp : unsigned(20 downto 0) := (others => '0');      -- bands per h, Q24
    signal c_f0   : unsigned(23 downto 0) := (others => '0');      -- wave cycles per h, Q32
    signal c_sp   : unsigned(27 downto 0) := (others => '0');      -- spread, cycles per h, Q32
    signal c_q2   : unsigned(15 downto 0) := (others => '0');      -- quadratic coefficient, 2^-35 cycles per h^2
    signal c_F    : unsigned(11 downto 0) := (others => '0');      -- fold, bands Q9
    signal c_k3d  : unsigned(7 downto 0) := to_unsigned(102, 8);   -- weight at black, duty / 256
    signal c_span : unsigned(7 downto 0) := to_unsigned(141, 8);   -- head-room above it
    signal c_rstep : unsigned(27 downto 0) := (others => '0');     -- phase step per field, Q32
    signal c_ph   : unsigned(31 downto 0) := (others => '0');      -- wave phase, Q32 cycles
    signal c_cneg : std_logic := '0';                              -- chirp sign
    signal c_P0, c_Pd0, c_Pdd : unsigned(35 downto 0) := (others => '0');   -- theta DDA at the pre-pixel (Q36 cycles)
    signal c_V0, c_Vd : unsigned(27 downto 0) := (others => '0');  -- band-coordinate DDA (Q4.24 bands)
    -- the six coefficients, chosen by Field and Axis (combinational)
    signal w_J1, w_K1 : unsigned(27 downto 0);                     -- theta linear, along x / y (Q32 per h)
    signal w_J2, w_K2 : unsigned(15 downto 0);                     -- theta quadratic
    signal w_jPV, w_kPV : unsigned(20 downto 0);                   -- band coordinate
    signal w_qneg : std_logic;                                     -- sign of the quadratic term

    -- [E] per-field palette: ground, band colour - ground
    type t_s11x3 is array (0 to 2) of signed(10 downto 0);
    signal c_gy, c_gu, c_gv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_dy, c_du, c_dv : t_s11x3 := (others => (others => '0'));

    -- [C] line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_d    : signed(19 downto 0) := (others => '0');
    signal e_sy, e_first : std_logic := '0';
    signal e_ady  : unsigned(14 downto 0) := (others => '0');
    signal m_a, m_acc : unsigned(45 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal n_pu   : unsigned(23 downto 0) := (others => '0');      -- next line: theta at the pre-pixel (Q24)
    signal n_v0   : unsigned(27 downto 0) := (others => '0');      -- next line: band coordinate at the pre-pixel
    signal l_first : std_logic := '0';

    -- [C] beam DDAs and the pixel path
    signal b_P, b_Pd : unsigned(35 downto 0) := (others => '0');
    signal b_V : unsigned(27 downto 0) := (others => '0');
    signal s3_tri, s4_tri : unsigned(11 downto 0) := (others => '0');
    signal s5_mag : unsigned(11 downto 0) := (others => '0');
    signal s3_neg, s4_neg, s5_neg, s6_neg, s7_neg, s8_neg : std_logic := '0';
    signal s7_lo, s7_hi : unsigned(15 downto 0) := (others => '0');
    signal s8_fw : unsigned(14 downto 0) := (others => '0');
    signal s9_phi, s10_phi, s11_phi, s12_phi : unsigned(15 downto 0) := (others => '0');
    signal s10_gx, s10_gy : signed(15 downto 0) := (others => '0');
    signal s11_agx, s11_agy, s12_w : unsigned(12 downto 0) := (others => '0');

    -- [C] Fold table: (F * i) >> 2 for i = 0 .. 63, refilled every field by an accumulator; the pixel path
    -- looks up both halves of the wave magnitude (two copies: one read port each)
    type t_ft is array (0 to 255) of unsigned(15 downto 0);
    signal fl_ram, fh_ram : t_ft := (others => (others => '0'));
    attribute no_rw_check of fl_ram : signal is true;
    attribute no_rw_check of fh_ram : signal is true;
    signal fl_q, fh_q, ft_wd : unsigned(15 downto 0) := (others => '0');
    signal ft_wa, ft_i : unsigned(5 downto 0) := (others => '0');
    signal ft_acc : unsigned(17 downto 0) := (others => '0');
    signal ft_we, ft_run : std_logic := '0';

    -- [C] PHI line store (the line above, for the vertical gradient)
    type t_ph is array (0 to 2047) of unsigned(15 downto 0);
    signal ph_ram : t_ph := (others => (others => '0'));
    attribute no_rw_check of ph_ram : signal is true;
    signal ph_ra, ph_wa : unsigned(10 downto 0) := (others => '0');
    signal ph_q, s9_ab, ph_wd : unsigned(15 downto 0) := (others => '0');
    signal ph_we : std_logic := '0';

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
    signal s_eb8, s_eq : unsigned(7 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(7 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(7 downto 0) := (others => '0');
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
    signal wt_d   : unsigned(16 downto 0) := (others => '0');

    -- [E] coverage and blend
    signal s13_weff, s14_weff, s15_weff, s14_wh : unsigned(12 downto 0) := (others => '0');
    signal s13_dh : unsigned(10 downto 0) := (others => '0');
    signal s13_af : unsigned(11 downto 0) := (others => '0');
    signal s14_d  : signed(12 downto 0) := (others => '0');
    signal s15_e  : signed(14 downto 0) := (others => '0');
    type t_dv13 is array (0 to 4) of unsigned(12 downto 0);
    signal dv_r, dv_w : t_dv13 := (others => (others => '0'));
    type t_dv4 is array (0 to 4) of unsigned(3 downto 0);
    signal dv_q : t_dv4 := (others => (others => '0'));
    signal dv_z, dv_f : std_logic_vector(0 to 4) := (others => '0');
    type t_idx is array (13 to 21) of unsigned(1 downto 0);
    signal s_idx : t_idx := (others => "00");
    signal s21_cov, s22_cov : unsigned(4 downto 0) := (others => '0');
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
            r_ex_q <= C_RL_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_RL_SIN(to_integer(sn_a));
        end if;
    end process p_rom;

    -- Fold tables: written in the vblank zone only, read by the pixel path (stage 5 address)
    p_flram : process(clk)
    begin
        if rising_edge(clk) then
            fl_q <= fl_ram(to_integer(s5_mag(5 downto 0)));
            if ft_we = '1' then fl_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_flram;
    p_fhram : process(clk)
    begin
        if rising_edge(clk) then
            fh_q <= fh_ram(to_integer(s5_mag(11 downto 6)));
            if ft_we = '1' then fh_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_fhram;

    -- weight table: written in the vblank zone only, read by the pixel path
    p_wtram : process(clk)
    begin
        if rising_edge(clk) then
            wt_q <= wt_ram(to_integer(s9_l));
            if wt_we = '1' then wt_ram(to_integer(wt_wa)) <= wt_wd; end if;
        end if;
    end process p_wtram;

    -- PHI line store: read 3 pixels ahead of the write, never the same address
    p_phram : process(clk)
    begin
        if rising_edge(clk) then
            ph_q <= ph_ram(to_integer(ph_ra));
            if ph_we = '1' then ph_ram(to_integer(ph_wa)) <= ph_wd; end if;
        end if;
    end process p_phram;

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

    w_J1 <= (others => '0') when s_rad = '1' and s_down = '0' else
            resize(c_sp(27 downto 2), 28) when s_rad = '1' else
            c_sp when s_down = '1' else resize(c_f0, 28);
    w_K1 <= (others => '0') when s_rad = '1' and s_down = '1' else
            resize(c_sp(27 downto 2), 28) when s_rad = '1' else
            resize(c_f0, 28) when s_down = '1' else c_sp;
    w_J2 <= c_q2 when s_rad = '1' or s_down = '0' else (others => '0');
    w_K2 <= c_q2 when s_rad = '1' or s_down = '1' else (others => '0');
    w_jPV <= c_invp when s_down = '1' else (others => '0');
    w_kPV <= (others => '0') when s_down = '1' else c_invp;
    w_qneg <= c_cneg and not s_rad;

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
                s_zig  <= registers_in(6)(0);
                s_rad  <= registers_in(6)(1);
                s_quad <= registers_in(6)(2);
                s_down <= registers_in(6)(3);
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
            -- pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.  Slots
            -- 2 and 5 take the previous product straight from the accumulator.
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
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g1, 40);  z_fb <= to_unsigned(1107, 16);                 -- octaves above pitch 8
                    when 1  =>                                                                           -- |chirp| Q16
                        z_fa <= resize(z_cm, 40);
                        if s_rad = '1' then z_fb <= to_unsigned(92, 16); else z_fb <= to_unsigned(122, 16); end if;
                    when 2  => z_fa <= resize(c_f0, 40);  z_fb <= f_acc(15 downto 0);                    -- f0 * |chirp|
                    when 3  =>                                                                           -- the quadratic coefficient
                        z_fa <= resize(z_q, 40);
                        if s_rad = '1' then z_fb <= to_unsigned(422, 16);
                        elsif s_down = '1' then z_fb <= to_unsigned(485, 16);
                        else z_fb <= to_unsigned(273, 16); end if;
                    when 4  => z_fa <= resize(s_g5(12 downto 3), 40); z_fb <= resize(s_g5(12 downto 3), 16);  -- spread^2
                    when 5  => z_fa <= resize(c_invp, 40); z_fb <= f_acc(20 downto 5);                   -- spread per h
                    when 6  => z_fa <= resize(s_gp, 40);  z_fb <= resize(s_gp, 16);                      -- fold^2
                    when 7  => z_fa <= resize(s_g3(12 downto 3), 40); z_fb <= to_unsigned(205, 16);      -- weight
                    when 8  => z_fa <= resize(s_vm, 40);  z_fb <= to_unsigned(897, 16);                  -- rate exponent
                    when 9  => z_fa <= resize(r_exf, 40); z_fb <= to_unsigned(9321, 16);                 -- rate step
                    when 10 => z_fa <= resize(w_J1, 40);  z_fb <= resize(w_dxm, 16);                     -- theta at the pre-pixel
                    when 11 => z_fa <= resize(w_qx0, 40); z_fb <= w_J2;
                    when 12 => z_fa <= resize(w_J1, 40);  z_fb <= resize(s_xs, 16);                      -- its first difference
                    when 13 => z_fa <= resize(w_d0m, 40); z_fb <= w_J2;
                    when 14 => z_fa <= resize(w_J2, 40);  z_fb <= resize(w_dd, 16);                      -- its second difference
                    when 15 => z_fa <= resize(w_jPV, 40); z_fb <= resize(w_dxm, 16);                     -- band coordinate there
                    when others => z_fa <= resize(w_jPV, 40); z_fb <= resize(s_xs, 16);                  -- and its step
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
                    when 21 =>                        -- chirp: |k2 - 512| - 8 past a +-8 deadband
                        v_k := s_g2(12 downto 3);
                        if v_k > 520 then z_cm <= resize(v_k - 520, 9); c_cneg <= '0';
                        elsif v_k < 504 then z_cm <= resize(504 - v_k, 9); c_cneg <= '1';
                        else z_cm <= (others => '0'); c_cneg <= '0'; end if;

                    -- pitch: bands per h = 2^20 * 2^-oct
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
                            when -4     => c_invp <= resize(r_exf & "0", 21);
                            when others => c_invp <= resize(r_exf, 21);
                        end case;
                        -- wavelength: f0 = 2^23.5 * 2^(-oct / 2) cycles per h (Q32)
                        v_e := to_signed(128, 12) - signed(resize(z_oct(10 downto 1), 12));
                        r_ex_a <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(11 downto 8);
                    when 57  =>
                        case to_integer(z_sh) is
                            when 0      => c_f0 <= r_exf & x"00";
                            when -1     => c_f0 <= resize(r_exf & "0000000", 24);
                            when others => c_f0 <= resize(r_exf & "000000", 24);
                        end case;
                    when 98  =>                       -- Linear: f0 |c|;  Radial: f0 (1 +- 3/4 |c|)
                        if s_rad = '1' then z_q <= f_addsub(resize(c_f0, 25), resize(f_acc(39 downto 16), 25), c_cneg);
                        else z_q <= resize(f_acc(39 downto 16), 25); end if;
                    when 122 => c_q2 <= f_acc(32 downto 17);
                    when 170 => c_sp <= f_acc(35 downto 8);
                    when 194 =>
                        v_8 := f_acc(25 downto 14);
                        c_F <= v_8 - shift_right(v_8, 2);
                    when 218 => c_k3d <= to_unsigned(26, 8) + f_acc(17 downto 10);
                    when 219 => c_span <= to_unsigned(243, 8) - c_k3d;
                    when 242 => r_ex_a <= '0' & f_acc(15 downto 8); z_sh4 <= f_acc(18 downto 16);
                    when 266 =>
                        v_r := resize(f_acc(29 downto 8), 28);
                        if s_vm = 0 then c_rstep <= (others => '0');
                        else c_rstep <= shift_left(v_r, to_integer(z_sh4)); end if;
                    when 290 => c_P0 <= f_addsub(to_unsigned(0, 36), f_acc(31 downto 0) & "0000", '1');
                    when 314 => c_P0 <= f_addsub(c_P0, f_acc(34 downto 0) & '0', w_qneg);
                    when 338 => c_Pd0 <= f_acc(31 downto 0) & "0000";
                    when 362 => c_Pd0 <= f_addsub(c_Pd0, f_acc(34 downto 0) & '0', not w_qneg);
                    when 386 => c_Pdd <= f_addsub(to_unsigned(0, 36), f_acc(34 downto 0) & '0', w_qneg);
                    when 410 => c_V0 <= f_addsub(to_unsigned(0, 28), f_acc(27 downto 0), '1');
                    when 434 => c_Vd <= f_acc(27 downto 0);
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
    -- [C] WAVE ENGINE, line part: kicked at each line start (and once in the
    -- zone for line 0), computes the NEXT line's beam start values on one
    -- serial multiplier (16 iterations, result at load + 17):
    --   n_pu = theta at the pre-pixel = P0 + phase +- K2 dy^2 +- K1 |dy|   (Q24 cycles; P0's low 12 bits ride along)
    --   n_v0 = band coordinate at the pre-pixel = V0 +- kPV |dy|
    -- The beam loads them at the next g_start (theta) and 6 clocks later (band coordinate); the engine
    -- rewrites them only from its step 20 on, so no second copy is needed.
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;     -- after the field constants

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_first <= '1';
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_first <= '0';
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(44 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_d <= signed(resize(e_vy, 20)) - C_CY8;
                when 2 =>                     -- |dy| in h, 4 fraction bits
                    e_sy <= e_d(19);
                    if e_d(19) = '1' then e_ady <= resize(shift_right(unsigned(-e_d), 3), 15);
                    else e_ady <= resize(shift_right(unsigned(e_d), 3), 15); end if;
                when 3 =>                     -- dy^2
                    m_a <= resize(e_ady, 46); m_b <= resize(e_ady, 16); m_acc <= (others => '0');
                when 20 =>                    -- K2 * dy^2
                    n_pu <= c_P0(35 downto 12) + c_ph(31 downto 8);
                    m_a <= resize(m_acc(29 downto 0), 46); m_b <= w_K2; m_acc <= (others => '0');
                when 37 =>                    -- K1 * |dy|
                    n_pu <= f_addsub(n_pu, m_acc(42 downto 19), w_qneg);
                    m_a <= resize(w_K1, 46); m_b <= resize(e_ady, 16); m_acc <= (others => '0');
                when 54 =>                    -- kPV * |dy|
                    n_pu <= f_addsub(n_pu, m_acc(35 downto 12), e_sy);
                    m_a <= resize(w_kPV, 46); m_b <= resize(e_ady, 16); m_acc <= (others => '0');
                when 71 =>
                    n_v0 <= f_addsub(c_V0, m_acc(31 downto 4), e_sy);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [C] WAVE ENGINE, beam: free-running DDAs, loaded with the pre-pixel's
    -- values so that pixel 0 arrives at each one's stage.
    --   P = theta (second order: the chirp), stage 2       V = the band coordinate, stage 8
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_P <= n_pu & c_P0(11 downto 0); b_Pd <= c_Pd0;
                l_first <= e_first;
            else
                b_P <= b_P + b_Pd; b_Pd <= b_Pd + c_Pdd;
            end if;
            if s_st(5) = '1' then
                b_V <= n_v0;
            else
                b_V <= b_V + c_Vd;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [C] WAVE ENGINE, pixel path (stages 3 .. 12)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a  : unsigned(15 downto 0);
        variable v_fw : unsigned(21 downto 0);
        variable v_w  : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: quarter-wave address / triangle magnitude from theta (b_P's top 24 bits)
            s3_neg <= b_P(35);
            if b_P(34) = '1' then
                sn_a <= not b_P(33 downto 24); s3_tri <= not b_P(33 downto 22);
            else
                sn_a <= b_P(33 downto 24); s3_tri <= b_P(33 downto 22);
            end if;
            -- 4: the sine ROM's own register (p_rom)
            s4_tri <= s3_tri; s4_neg <= s3_neg;
            -- 5: fabric copy, Smooth / Zigzag (12-bit magnitude)
            if s_zig = '1' then s5_mag <= s4_tri; else s5_mag <= sn_q(13 downto 2); end if;
            s5_neg <= s4_neg;
            -- 6: Fold * (low half) and Fold * (high half) from the Fold tables (p_flram, p_fhram)
            s6_neg <= s5_neg;
            -- 7: fabric copies
            s7_lo <= fl_q; s7_hi <= fh_q; s7_neg <= s6_neg;
            -- 8: the displacement (bands, Q12)
            v_fw := resize(s7_lo, 22) + (s7_hi & "000000");
            s8_fw <= v_fw(21 downto 7);
            s8_neg <= s7_neg;
            -- 9: PHI (Q4.12, mod 16 bands); the line above from the store
            s9_phi <= f_addsub(b_V(27 downto 12), resize(s8_fw, 16), s8_neg);
            s9_ab <= ph_q;
            -- 10: gradient by finite differences (per real pixel, so the AA is right in every video mode)
            s10_gx <= signed(s9_phi - s10_phi);
            s10_gy <= signed(s9_phi - s9_ab);
            s10_phi <= s9_phi;
            ph_wd <= s9_phi;
            ph_we <= s_gav_sr(7) or s_gav_sr(15);
            -- 11: magnitudes, clamped to 2 bands per pixel; no line above on the first line
            if s10_gx(15) = '1' then v_a := unsigned(-s10_gx); else v_a := unsigned(s10_gx); end if;
            if v_a(15 downto 13) /= "000" then s11_agx <= (others => '1'); else s11_agx <= v_a(12 downto 0); end if;
            if s10_gy(15) = '1' then v_a := unsigned(-s10_gy); else v_a := unsigned(s10_gy); end if;
            if l_first = '1' then s11_agy <= (others => '0');
            elsif v_a(15 downto 13) /= "000" then s11_agy <= (others => '1');
            else s11_agy <= v_a(12 downto 0); end if;
            s11_phi <= s10_phi;
            -- 12: w = |grad PHI| (bands per pixel, Q12)
            v_w := resize(s11_agx, 14) + resize(s11_agy, 14);
            if v_w(13) = '1' then s12_w <= (others => '1'); else s12_w <= v_w(12 downto 0); end if;
            s12_phi <= s11_phi;

            -- Fold table fill (zone, zc 200 .. 264): T(i) = (F * i) >> 2 by accumulation
            ft_we <= '0';
            if s_zone = '1' and s_zc = 200 then
                ft_run <= '1'; ft_i <= (others => '0'); ft_acc <= (others => '0');
            elsif ft_run = '1' then
                ft_we <= '1'; ft_wa <= ft_i; ft_wd <= ft_acc(17 downto 2);
                ft_acc <= ft_acc + c_F; ft_i <= ft_i + 1;
                if ft_i = 63 then ft_run <= '0'; end if;
            end if;

            -- column counters of the two line stores
            if s_st(5) = '1' then ph_ra <= (others => '0'); else ph_ra <= ph_ra + 1; end if;
            if s_st(8) = '1' then ph_wa <= (others => '0'); else ph_wa <= ph_wa + 1; end if;
            if s_st(3) = '1' or s_st(4) = '1' then lb_rc <= (others => '0'); else lb_rc <= lb_rc + 1; end if;
        end if;
    end process p_pix;

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
        variable v_d : signed(9 downto 0);
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

            -- vertical IIR on the odd columns: q += (box - q + 4) >> 3
            s_eb8 <= s_bsum(12 downto 5); s_eq <= lb_q1;
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb8, 10)) - signed(resize(s_eq, 10)) + 4;
            v_a := unsigned(signed(resize(s_eq, 9)) + resize(shift_right(v_d, 3), 9));
            lb_wd <= v_a(7 downto 0);
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- pixel path: 8 fabric copies of this and the last read; 9 the mean (a linear interpolation of
            -- the half-resolution store); 10 the weight table; 11, 12 fabric copies (duty / 256)
            s8_q <= lb_q2; s8_qp <= s8_q;
            v_a := resize(s8_q, 9) + resize(s8_qp, 9) + 1;
            s9_l <= v_a(8 downto 1);
            s11_d8 <= wt_q;
            s12_d8 <= s11_d8;

            -- weight table fill (zone, zc 224 .. 480): W(l) = K3 + min(span * l^2 / 2^15, span), the square by
            -- second differences
            wt_we <= '0';
            if s_zone = '1' and s_zc = 224 then
                wt_run <= '1'; wt_i <= (others => '0'); wt_acc <= (others => '0'); wt_d <= resize(c_span, 17);
            elsif wt_run = '1' then
                wt_we <= '1'; wt_wa <= wt_i;
                if wt_acc(23 downto 15) > resize(c_span, 9) then wt_wd <= c_k3d + c_span;
                else wt_wd <= c_k3d + wt_acc(22 downto 15); end if;
                wt_acc <= wt_acc + wt_d; wt_d <= wt_d + shift_left(resize(c_span, 17), 1); wt_i <= wt_i + 1;
                if wt_i = 255 then wt_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] PALETTE + AA BLEND (stages 13 .. 25)
    --   weff = w, widened past 1/4 band per pixel so the bands fade to their mean before they alias
    --   dh   = half the band weight, held to at least one pixel of band and one of gap
    --   cov  = clamp(16 (d + weff / 2) / weff + 1/2), d = dh - |frac(PHI) - 1/2|
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_w5 : unsigned(15 downto 0);
        variable v_hi : unsigned(11 downto 0);
        variable v_dh : unsigned(10 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_g  : signed(15 downto 0);
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            -- 13
            if s12_w(12 downto 10) /= "000" then
                v_w5 := shift_left(resize(s12_w, 16), 2) + resize(s12_w, 16) - 4096;
                if v_w5(15 downto 13) /= "000" then s13_weff <= (others => '1'); else s13_weff <= v_w5(12 downto 0); end if;
                s13_dh <= to_unsigned(1024, 11);
            else
                s13_weff <= s12_w;
                v_hi := to_unsigned(2048, 12) - resize(s12_w, 12);
                v_dh := s12_d8 & "000";
                if v_dh < s12_w(10 downto 0) then s13_dh <= s12_w(10 downto 0);
                elsif resize(v_dh, 12) > v_hi then s13_dh <= v_hi(10 downto 0);
                else s13_dh <= v_dh; end if;
            end if;
            if s12_phi(11) = '1' then s13_af <= '0' & s12_phi(10 downto 0);
            else s13_af <= to_unsigned(2048, 12) - resize(s12_phi(10 downto 0), 12); end if;
            if s_quad = '1' then s_idx(13) <= s12_phi(13 downto 12); else s_idx(13) <= "00"; end if;
            for i in 14 to 21 loop s_idx(i) <= s_idx(i - 1); end loop;
            -- 14
            s14_d <= signed(resize(s13_dh, 13)) - signed(resize(s13_af, 13));
            s14_wh <= shift_right(s13_weff, 1) + shift_right(s13_weff, 5);
            s14_weff <= s13_weff;
            -- 15
            s15_e <= resize(s14_d, 15) + signed(resize(s14_wh, 15));
            s15_weff <= s14_weff;
            -- 16: range tests; 17 .. 20: four quotient bits
            dv_z(0) <= s15_e(14);
            if s15_e >= signed(resize(s15_weff, 15)) then dv_f(0) <= '1'; else dv_f(0) <= '0'; end if;
            dv_r(0) <= unsigned(s15_e(12 downto 0));
            dv_w(0) <= s15_weff;
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
            -- 21: coverage 0 .. 16
            if dv_z(4) = '1' then s21_cov <= (others => '0');
            elsif dv_f(4) = '1' then s21_cov <= to_unsigned(16, 5);
            else s21_cov <= '0' & dv_q(4); end if;
            -- 22: the band's colour, as its difference from the ground
            s22_cov <= s21_cov;
            case to_integer(s_idx(21)) is
                when 0      => s22_dy <= c_dy(0); s22_du <= c_du(0); s22_dv <= c_dv(0);
                when 1      => s22_dy <= c_dy(1); s22_du <= c_du(1); s22_dv <= c_dv(1);
                when 2      => s22_dy <= c_dy(2); s22_du <= c_du(2); s22_dv <= c_dv(2);
                when others => s22_dy <= (others => '0'); s22_du <= (others => '0'); s22_dv <= (others => '0');
            end case;
            -- 23: difference * coverage as two sums of gated shifts
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
            -- 25: ground + (difference * coverage + 8) / 16; the ground colour outside pixels 0 .. W + 7
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

end architecture riley;
