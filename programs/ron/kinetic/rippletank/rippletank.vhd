-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Rippletank v0.1.0 -- Kinetic.  After the physics ripple tank and the zone-plate moire of op art: two to four
-- wave sources whose ring systems interfere, steered like a phased array.
--
-- Element: a source.  State: its phase phi_j = Depth * sin(lfo + Spread * j).
--
--   theta_j(x, y) = q |x - s_j|^2 - phase - phi_j        (a QUADRATIC pond per source: no per-pixel square root)
--                 = Z(x, y) + A_j(x, y),   Z = q (dx^2 + dy^2) - phase,   A_j = -2 q s_j . x + q |s_j|^2 - phi_j
--   F = sum_j cos(theta_j)   or   max_j cos(theta_j)     (+ the source luma)
--
--   K1  Spacing     distance between the sources, 11 .. 362 virtual px (exponential, glided)
--   K2  Sources     2 / 3 / 4
--   K3  Depth       phase swing of the steering wave, square law, 0 .. 1 cycle
--   K4  Rate        rings run outwards / inwards, bipolar, centre = still; the steering wave runs at half the
--                   ring rate and the sources orbit the centre at 1/16 of it
--   K5  Spread      phase step between sources, square law, 0 .. 1/2 cycle
--   K6  Colour      the 16 shared Kinetic palettes (hysteresis)
--   S7  Wave        Smooth (shaded) / Hard (two-tone, anti-aliased)
--   S8  Field       Line (an array through the centre) / Ring (on a circle, turned a quarter)
--   S9  Mode        Sum (interference) / Max (overlapping ponds)
--   S10 Look        Fill / Lines (the nodal contour F = 0)
--   S11 Source      Video / Osc
--   P12 WAVELENGTH  ring density, exponential: 1.2 .. 18 rings from the centre to the top edge
--
-- Z is one second-order beam DDA (its line-start value from a per-line engine), each A_j a first-order DDA, so
-- the pixel path has no multiplier: four quarter-wave cosine ROMs, a sum / max tree, a tone stage whose gain
-- follows the local ring pitch (from the DDA's own first differences), and a 256-entry colour table refilled
-- every field (ground -> A -> accent).
--
-- Sections (the integer model .kinetic_work/rb_rippletank/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         glides, switches, rate deadband, palette hysteresis
--   [C] WAVE ENGINE                      per-field sequencer (one serial multiplier), per-line engine, beam DDAs,
--                                        cosines, sum / max, tone
--   [D] SOURCE FIELD                     Osc lattice, 32-px box, vertical IIR line store
--   [E] COLOUR TABLE                     per-field fill, lookup
--   [F] OUTPUT CONDITIONING              chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 at the same group delay,
--                                        luma 13/16, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixels 0 .. W + 7 are rendered and output column j is the 9-tap filter
-- over pixels j .. j + 8.  s_gav_sr(S - 2) flags pixels 0 .. W - 1 at stage S.  One latency (28) in every mode.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.rippletank_pkg.all;
use work.all;

architecture rippletank of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 600;
    constant C_DEF_K3  : integer := 720;
    constant C_DEF_K5  : integer := 720;
    constant C_DEF_P12 : integer := 400;

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
    signal s_st       : std_logic_vector(0 to 4) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(C_DEF_K5 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(1 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_hard, s_ring, s_max, s_lines, s_src : std_logic := '0';
    signal s_ge3, s_ge4 : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := (others => '0');
    signal w_hlc : unsigned(3 downto 0);

    -- [C] ROMs: C_RT_TAB (2^(i/256), then the palettes), C_RT_SIN (a quarter sine, per field), C_RT_COS (pixels)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q, sn_f : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier, slot sequencer and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_run, z_pre, z_ld, z_st, z_nw : std_logic := '0';
    signal z_t   : unsigned(4 downto 0) := (others => '0');
    signal z_sel : unsigned(5 downto 0) := (others => '0');
    signal z_j   : unsigned(1 downto 0);
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_sh4 : unsigned(2 downto 0) := (others => '0');
    signal z_Bj, z_gx, z_gy : unsigned(24 downto 0) := (others => '0');
    signal z_qr, z_A0 : unsigned(23 downto 0) := (others => '0');
    signal z_ws  : unsigned(11 downto 0) := (others => '0');
    signal z_sc, z_ss, z_sw : std_logic := '0';
    signal w_mag2, w_mmax : unsigned(1 downto 0);
    signal w_boff : unsigned(11 downto 0);

    -- [C] per-field constants
    signal c_q30  : unsigned(15 downto 0) := (others => '0');      -- zone-plate coefficient, 2^-30 cycles per h^2
    signal c_d14  : unsigned(13 downto 0) := (others => '0');      -- source spacing, h Q4
    signal c_dep  : unsigned(15 downto 0) := (others => '0');      -- phase swing, cycles Q16
    signal c_B    : unsigned(23 downto 0) := (others => '0');      -- 2 q d, cycles per h Q28
    signal c_qd2  : unsigned(29 downto 0) := (others => '0');      -- q d^2, cycles Q24
    signal c_spr  : unsigned(11 downto 0) := (others => '0');      -- phase step per source, 1/4096 cycle
    signal c_gm   : unsigned(8 downto 0) := (others => '0');       -- the sources' own fringe frequency, 1/1024 cycle per px
    signal c_rstep : unsigned(27 downto 0) := (others => '0');     -- phase step per field, Q32
    signal c_ph   : unsigned(35 downto 0) := (others => '0');      -- ring phase, Q32 cycles (4 integer bits: lfo, orbit)
    signal c_P0, c_Pd0, c_Pdd : unsigned(35 downto 0) := (others => '0');   -- Z DDA at the pre-pixel (Q36 cycles)
    type t_u24x4 is array (0 to 3) of unsigned(23 downto 0);
    signal c_Ax, c_Ay : t_u24x4 := (others => (others => '0'));    -- A_j steps per pixel / per line (Q24 cycles)

    -- [E] per-field palette: ground, colour - ground; the colour table
    type t_s11x3 is array (0 to 2) of signed(10 downto 0);
    signal c_gy, c_gu, c_gv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal c_dy, c_du, c_dv : t_s11x3 := (others => (others => '0'));
    signal c_ey, c_eu, c_ev : signed(13 downto 0) := (others => '0');
    signal ct_ay, ct_au, ct_av : signed(18 downto 0) := (others => '0');
    signal ct_i  : unsigned(7 downto 0) := (others => '0');
    signal ct_m7 : unsigned(2 downto 0) := (others => '0');
    signal ct_run : std_logic := '0';
    type t_cr is array (0 to 255) of unsigned(15 downto 0);
    signal cry_ram, cruv_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of cry_ram : signal is true;
    attribute no_rw_check of cruv_ram : signal is true;
    signal cr_qy, cr_quv, cr_wy, cr_wuv : unsigned(15 downto 0) := (others => '0');
    signal cr_wa : unsigned(7 downto 0) := (others => '0');
    signal cr_we : std_logic := '0';

    -- [C] line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_d    : signed(19 downto 0) := (others => '0');
    signal e_first, e_pass1 : std_logic := '0';
    signal e_ady  : unsigned(14 downto 0) := (others => '0');
    signal m_a, m_acc : unsigned(45 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal n_pu, e_pprev : unsigned(23 downto 0) := (others => '0');   -- next line: Z at the pre-pixel (Q24)
    signal e_dz   : signed(23 downto 0) := (others => '0');
    signal e_zy, e_zy1, l_zy : unsigned(8 downto 0) := (others => '0');
    signal n_a    : t_u24x4 := (others => (others => '0'));            -- next line: A_j at the pre-pixel

    -- [C] beam DDAs and the pixel path
    signal b_P, b_Pd : unsigned(35 downto 0) := (others => '0');
    signal b_A : t_u24x4 := (others => (others => '0'));
    type t_u16x4 is array (0 to 3) of unsigned(15 downto 0);
    type t_u8x4 is array (0 to 3) of unsigned(7 downto 0);
    type t_s9x4 is array (0 to 3) of signed(8 downto 0);
    signal s3_th : t_u16x4 := (others => (others => '0'));
    signal cs_a, cs_q : t_u8x4 := (others => (others => '0'));
    signal s4_neg, s5_neg : std_logic_vector(0 to 3) := (others => '0');
    signal s6_c : t_s9x4 := (others => (others => '0'));
    signal s3_zx, s4_ga, s4_gb, s6_G : unsigned(8 downto 0) := (others => '0');
    signal s5_g : unsigned(9 downto 0) := (others => '0');
    type t_n is array (7 to 13) of unsigned(3 downto 0);
    signal s_n : t_n := (others => (others => '0'));
    type t_att is array (7 to 11) of unsigned(2 downto 0);
    signal s_att : t_att := (others => (others => '0'));
    signal s7_s01, s7_s23 : signed(9 downto 0) := (others => '0');
    signal s7_m01, s7_m23, s8_max : signed(8 downto 0) := (others => '0');
    signal s8_sum : signed(10 downto 0) := (others => '0');
    signal s9_F, s10_F : signed(9 downto 0) := (others => '0');
    signal s10_t : signed(10 downto 0) := (others => '0');
    signal s11_x, s12_xa : signed(7 downto 0) := (others => '0');
    signal s13_h : signed(8 downto 0) := (others => '0');
    signal s13_si, s14_si, s15_idx : unsigned(7 downto 0) := (others => '0');
    signal s14_hs : signed(12 downto 0) := (others => '0');
    signal s14_base : unsigned(11 downto 0) := (others => '0');
    signal s17_y, s17_u, s17_v : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    signal s8_q, s8_qp, s9_l : unsigned(7 downto 0) := (others => '0');

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
            r_ex_q <= C_RT_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_RT_SIN(to_integer(sn_a));
            sn_f   <= sn_q;
        end if;
    end process p_rom;

    -- four cosine ROMs, one per source (stage 5)
    p_cos0 : process(clk) begin if rising_edge(clk) then cs_q(0) <= C_RT_COS(to_integer(cs_a(0))); end if; end process;
    p_cos1 : process(clk) begin if rising_edge(clk) then cs_q(1) <= C_RT_COS(to_integer(cs_a(1))); end if; end process;
    p_cos2 : process(clk) begin if rising_edge(clk) then cs_q(2) <= C_RT_COS(to_integer(cs_a(2))); end if; end process;
    p_cos3 : process(clk) begin if rising_edge(clk) then cs_q(3) <= C_RT_COS(to_integer(cs_a(3))); end if; end process;

    -- colour table: written in the vblank zone only, read by the pixel path (stage 16)
    p_cryram : process(clk)
    begin
        if rising_edge(clk) then
            cr_qy <= cry_ram(to_integer(s15_idx));
            if cr_we = '1' then cry_ram(to_integer(cr_wa)) <= cr_wy; end if;
        end if;
    end process p_cryram;
    p_cruvram : process(clk)
    begin
        if rising_edge(clk) then
            cr_quv <= cruv_ram(to_integer(s15_idx));
            if cr_we = '1' then cruv_ram(to_integer(cr_wa)) <= cr_wuv; end if;
        end if;
    end process p_cruvram;

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
    -- [A] the beam's pre-pixel by video mode; [C] the source layout
    ---------------------------------------------------------------------------
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);
    w_qx0 <= to_unsigned(2085136, 22) when s_xs = 4 else to_unsigned(3697929, 22) when s_xs = 3 else to_unsigned(3694084, 22);
    w_d0m <= to_unsigned(11536, 14) when s_xs = 4 else to_unsigned(11529, 14) when s_xs = 3 else to_unsigned(7684, 14);
    w_dd  <= to_unsigned(32, 6) when s_xs = 4 else to_unsigned(18, 6) when s_xs = 3 else to_unsigned(8, 6);
    -- half virtual px per line, rounded up to 2 / 3 / 4 / 8 (scales the sources' fringe frequency to pixels)
    w_hlc <= to_unsigned(8, 4) when s_ys >= 900 else to_unsigned(4, 4) when s_ys >= 450 else
             to_unsigned(3, 4) when s_ys >= 320 else to_unsigned(2, 4);

    -- source z_j: distance from the centre in half spacings (w_mag2) and its angle past the orbit angle (w_boff)
    --   Line: N = 2: -1/2, +1/2;  N = 3: -1, 0, +1;  N = 4: -3/2, -1/2, +1/2, +3/2  (a negative side = half a turn)
    --   Ring: one spacing out, a quarter turn + j / N turns
    z_j <= (not z_sel(4)) & z_sel(3);
    w_mmax <= "10" when s_ring = '1' else "11" when s_ge4 = '1' else "10" when s_ge3 = '1' else "01";
    p_geo : process(z_j, s_ring, s_ge3, s_ge4)
    begin
        w_mag2 <= "00"; w_boff <= (others => '0');
        if s_ring = '1' then
            w_mag2 <= "10";
            if s_ge4 = '1' then
                w_boff <= (z_j + 1) & "0000000000";
            elsif s_ge3 = '1' then
                case z_j is
                    when "00"   => w_boff <= to_unsigned(1024, 12);
                    when "01"   => w_boff <= to_unsigned(2389, 12);
                    when "10"   => w_boff <= to_unsigned(3754, 12);
                    when others => w_boff <= to_unsigned(1023, 12);
                end case;
            else
                if z_j(0) = '1' then w_boff <= to_unsigned(3072, 12); else w_boff <= to_unsigned(1024, 12); end if;
            end if;
        elsif s_ge4 = '1' then
            if z_j(0) = z_j(1) then w_mag2 <= "11"; else w_mag2 <= "01"; end if;
            if z_j(1) = '0' then w_boff <= to_unsigned(2048, 12); end if;
        elsif s_ge3 = '1' then
            if z_j = "00" then w_mag2 <= "10"; w_boff <= to_unsigned(2048, 12);
            elsif z_j = "10" then w_mag2 <= "10"; end if;
        else
            if z_j = "00" then w_mag2 <= "01"; w_boff <= to_unsigned(2048, 12);
            elsif z_j = "01" then w_mag2 <= "01"; end if;
        end if;
    end process p_geo;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : unsigned(11 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_c  : signed(10 downto 0);
        variable v_r  : unsigned(27 downto 0);
        variable v_pi : unsigned(3 downto 0);
        variable v_pd : signed(10 downto 0);
        variable v_ang : unsigned(11 downto 0);
        variable v_lk : std_logic;
        variable v_t24 : unsigned(23 downto 0);
        variable v_g22 : unsigned(21 downto 0);
        variable v_q32 : unsigned(31 downto 0);
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
                s_hard  <= registers_in(6)(0);
                s_ring  <= registers_in(6)(1);
                s_max   <= registers_in(6)(2);
                s_lines <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- palette: 16 steps with hysteresis (leave a step only 8 past its edge)
                v_k := unsigned(registers_in(5));
                v_c := signed(resize(v_k, 11)) - signed(resize(s_pal & "100000", 11));
                if v_c > 40 or v_c < -40 then s_pal <= v_k(9 downto 6); end if;
                -- source count: 2 / 3 / 4 with hysteresis
                v_k := unsigned(registers_in(1));
                if v_k >= 349 then s_ge3 <= '1'; elsif v_k < 333 then s_ge3 <= '0'; end if;
                if v_k >= 690 then s_ge4 <= '1'; elsif v_k < 674 then s_ge4 <= '0'; end if;
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
                            when 1      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 2      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
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
                            when 1      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 3 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] the slot sequencer: 48 slots of 24 clocks from zc = 30.  In a slot: t = 1 operands are
            -- muxed (z_pre), t = 2 the multiplier loads (z_ld), 16 iterations, the product is stable from
            -- t = 19 and stored at t = 20 (z_st).  Slots 0 .. 15 are the common constants, 16 + 8 j + step
            -- the eight steps of source j.
            z_pre <= '0'; z_st <= '0'; z_nw <= '0';
            if s_zone = '1' and s_zc = 30 then
                z_run <= '1'; z_t <= (others => '0'); z_sel <= (others => '0');
            elsif z_run = '1' then
                if z_t = 23 then
                    z_t <= (others => '0'); z_sel <= z_sel + 1;
                    if z_sel = 47 then z_run <= '0'; end if;
                else
                    z_t <= z_t + 1;
                end if;
                if z_t = 0 then z_pre <= '1'; end if;
                if z_t = 19 then z_st <= '1'; end if;
            end if;
            z_ld <= z_pre;

            v_lk := '0'; v_ang := (others => '0');
            if z_pre = '1' then
                if z_sel(5 downto 4) = "00" then
                    case to_integer(z_sel(3 downto 0)) is
                        when 0  => z_fa <= resize(s_gp, 40);  z_fb <= to_unsigned(1001, 16);               -- wavelength octaves
                        when 1  => z_fa <= resize(s_g1, 40);  z_fb <= to_unsigned(1281, 16);               -- spacing octaves
                        when 2  => z_fa <= resize(s_g3(12 downto 3), 40); z_fb <= resize(s_g3(12 downto 3), 16);   -- depth^2
                        when 3  => z_fa <= resize(c_q30, 40); z_fb <= resize(c_d14, 16);                   -- 2 q d
                        when 4  => z_fa <= resize(c_B, 40);   z_fb <= resize(c_d14, 16);                   -- q d^2
                        when 5  => z_fa <= resize(s_g5(12 downto 3), 40); z_fb <= resize(s_g5(12 downto 3), 16);   -- spread^2
                        when 6  => z_fa <= resize(s_vm, 40);  z_fb <= to_unsigned(897, 16);                -- rate exponent
                        when 7  => z_fa <= resize(r_exf, 40); z_fb <= to_unsigned(9321, 16);               -- rate step
                        when 8  => z_fa <= resize(w_qx0, 40); z_fb <= c_q30;                               -- Z at the pre-pixel
                        when 9  => z_fa <= resize(w_d0m, 40); z_fb <= c_q30;                               -- its first difference
                        when 10 => z_fa <= resize(w_dd, 40);  z_fb <= c_q30;                               -- its second difference
                        when others => z_fa <= resize(z_Bj, 40); z_fb <= resize(w_hlc, 16);                -- fringe frequency
                    end case;
                else
                    case to_integer(z_sel(2 downto 0)) is
                        when 0  =>                                                                         -- prepare source j
                            case w_mag2 is
                                when "00"   => z_Bj <= (others => '0'); z_qr <= (others => '0');
                                when "01"   => z_Bj <= resize(c_B(23 downto 1), 25); z_qr <= c_qd2(25 downto 2);
                                when "10"   => z_Bj <= resize(c_B, 25); z_qr <= c_qd2(23 downto 0);
                                when others => z_Bj <= resize(c_B, 25) + resize(c_B(23 downto 1), 25);
                                               z_qr <= (c_qd2(22 downto 0) & '0') + c_qd2(25 downto 2);
                            end case;
                            if z_j = "00" then z_ws <= c_ph(32 downto 21); else z_ws <= z_ws + c_spr; end if;
                            v_ang := c_ph(35 downto 24) + w_boff + 1024; v_lk := '1';                      -- cos(beta)
                            z_sc <= v_ang(11);
                        when 1  => z_fa <= resize(z_Bj, 40);  z_fb <= sn_f;                                -- gx
                        when 2  => z_fa <= resize(z_Bj, 40);  z_fb <= sn_f;                                -- gy
                        when 3  => z_fa <= resize(c_dep, 40); z_fb <= sn_f;                                -- phi
                        when 4  => z_fa <= resize(z_gx, 40);  z_fb <= resize(s_xs, 16);                    -- A step per pixel
                        when 5  => z_fa <= resize(z_gy, 40);  z_fb <= resize(s_ys, 16);                    -- A step per line
                        when 6  => z_fa <= resize(z_gx, 40);  z_fb <= resize(w_dxm, 16);                   -- A at the pre-pixel
                        when others => z_fa <= resize(z_gy, 40); z_fb <= to_unsigned(1080, 16);            -- ... on line 0
                    end case;
                end if;
            end if;
            if z_ld = '1' and z_sel(5 downto 4) /= "00" then
                if z_sel(2 downto 0) = "001" then                                                          -- sin(beta)
                    v_ang := c_ph(35 downto 24) + w_boff; v_lk := '1'; z_ss <= v_ang(11);
                elsif z_sel(2 downto 0) = "010" then                                                       -- the steering wave
                    v_ang := z_ws; v_lk := '1'; z_sw <= v_ang(11);
                end if;
            end if;
            if v_lk = '1' then
                if v_ang(10) = '1' then sn_a <= not v_ang(9 downto 0); else sn_a <= v_ang(9 downto 0); end if;
            end if;

            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' and s_zc = 20 then           -- motion (last field's step); rate > 0 runs the rings outwards
                c_ph <= f_addsub(c_ph, resize(c_rstep, 36), not s_vneg);
            end if;

            v_t24 := f_acc(27 downto 4);
            if z_st = '1' then
                if z_sel(5 downto 4) = "00" then
                    case to_integer(z_sel(3 downto 0)) is
                        when 0  =>
                            v_e := f_acc(24 downto 13) + 2588;
                            r_ex_a <= '0' & v_e(7 downto 0); z_sh <= resize(15 - v_e(11 downto 8), 3);
                        when 1  =>
                            c_q30 <= shift_right(r_exf, to_integer(z_sh));
                            v_e := f_acc(24 downto 13) + 1152;
                            r_ex_a <= '0' & v_e(7 downto 0); z_sh <= resize(11 - v_e(11 downto 8), 3);
                        when 2  =>
                            c_d14 <= resize(shift_right(r_exf, to_integer(z_sh)), 14);
                            c_dep <= f_acc(19 downto 4);
                        when 3  => c_B <= f_acc(28 downto 5);
                        when 4  => c_qd2 <= f_acc(38 downto 9);
                        when 5  => c_spr <= f_acc(20 downto 9);
                        when 6  => r_ex_a <= '0' & f_acc(15 downto 8); z_sh4 <= f_acc(18 downto 16);
                        when 7  =>
                            v_r := resize(f_acc(29 downto 8), 28);
                            if s_vm = 0 then c_rstep <= (others => '0');
                            else c_rstep <= shift_left(v_r, to_integer(z_sh4)); end if;
                        when 8  => c_P0 <= f_acc(29 downto 0) & "000000";
                        when 9  => c_Pd0 <= f_addsub(to_unsigned(0, 36), f_acc(29 downto 0) & "000000", '1');
                        when 10 =>
                            c_Pdd <= f_acc(29 downto 0) & "000000";
                            case w_mmax is
                                when "01"   => z_Bj <= resize(c_B(23 downto 1), 25);
                                when "10"   => z_Bj <= resize(c_B, 25);
                                when others => z_Bj <= resize(c_B, 25) + resize(c_B(23 downto 1), 25);
                            end case;
                        when others =>
                            v_g22 := f_acc(39 downto 18);
                            if v_g22 > 511 then c_gm <= to_unsigned(511, 9); else c_gm <= v_g22(8 downto 0); end if;
                    end case;
                else
                    case to_integer(z_sel(2 downto 0)) is
                        when 1  => z_gx <= f_acc(38 downto 14);
                        when 2  => z_gy <= f_acc(38 downto 14);
                        when 3  => z_A0 <= f_addsub(to_unsigned(0, 24), f_acc(29 downto 6), not z_sw);
                        when 4  =>
                            c_Ax(to_integer(z_j)) <= f_addsub(to_unsigned(0, 24), v_t24, not z_sc);
                            z_A0 <= z_A0 + z_qr;
                        when 5  => c_Ay(to_integer(z_j)) <= f_addsub(to_unsigned(0, 24), f_acc(34 downto 11), not z_ss);
                        when 6  => z_A0 <= f_addsub(z_A0, v_t24, z_sc);
                        when 7  => z_A0 <= f_addsub(z_A0, v_t24, z_ss); z_nw <= '1';
                        when others => null;
                    end case;
                end if;
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

            -- [E] the colour table (zc 1298 .. 1556): 0 .. 223 ground -> A in 128 steps (4 of every 7 entries
            -- step), 224 .. 255 A -> accent in 32 steps of 4/128
            cr_we <= '0';
            if s_zone = '1' and s_zc = 1298 then
                c_ey <= shift_left(resize(c_dy(2), 14) - resize(c_dy(0), 14), 2);
                c_eu <= shift_left(resize(c_du(2), 14) - resize(c_du(0), 14), 2);
                c_ev <= shift_left(resize(c_dv(2), 14) - resize(c_dv(0), 14), 2);
            end if;
            if s_zone = '1' and s_zc = 1300 then
                ct_run <= '1'; ct_i <= (others => '0'); ct_m7 <= (others => '0');
                ct_ay <= signed(resize(c_gy & "0000000", 19));
                ct_au <= signed(resize(c_gu & "0000000", 19));
                ct_av <= signed(resize(c_gv & "0000000", 19));
            elsif ct_run = '1' then
                cr_we <= '1'; cr_wa <= ct_i;
                cr_wy <= resize(unsigned(ct_ay(16 downto 7)), 16);
                cr_wuv <= unsigned(ct_au(16 downto 9)) & unsigned(ct_av(16 downto 9));
                if ct_i(7 downto 5) /= "111" then
                    if ct_m7 = 1 or ct_m7 = 3 or ct_m7 = 5 or ct_m7 = 6 then
                        ct_ay <= ct_ay + resize(c_dy(0), 19);
                        ct_au <= ct_au + resize(c_du(0), 19);
                        ct_av <= ct_av + resize(c_dv(0), 19);
                    end if;
                else
                    ct_ay <= ct_ay + resize(c_ey, 19);
                    ct_au <= ct_au + resize(c_eu, 19);
                    ct_av <= ct_av + resize(c_ev, 19);
                end if;
                if ct_m7 = 6 then ct_m7 <= (others => '0'); else ct_m7 <= ct_m7 + 1; end if;
                ct_i <= ct_i + 1;
                if ct_i = 255 then ct_run <= '0'; end if;
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
    -- [C] WAVE ENGINE, line part: kicked at each line start (and once in the
    -- zone for line 0), computes the NEXT line's Z at the pre-pixel on one
    -- serial multiplier:  n_pu = P0 - phase + q dy^2  (Q24 cycles), and the
    -- change of Z per line (for the tone stage's gain).  The A_j just step.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_a : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;     -- after the field constants

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_first <= '1'; e_pass1 <= '0';
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_first <= '0'; e_pass1 <= e_first;
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
                    if e_d(19) = '1' then e_ady <= resize(shift_right(unsigned(-e_d), 3), 15);
                    else e_ady <= resize(shift_right(unsigned(e_d), 3), 15); end if;
                when 3 =>                     -- dy^2
                    m_a <= resize(e_ady, 46); m_b <= resize(e_ady, 16); m_acc <= (others => '0');
                when 19 =>
                    e_pprev <= n_pu;
                when 20 =>                    -- q * dy^2
                    n_pu <= c_P0(35 downto 12) + c_ph(31 downto 8);
                    m_a <= resize(m_acc(29 downto 0), 46); m_b <= c_q30; m_acc <= (others => '0');
                when 37 =>
                    n_pu <= n_pu + m_acc(37 downto 14);
                when 38 =>
                    e_dz <= signed(n_pu - e_pprev);
                when 39 =>                    -- |Z step per line|, 1/1024 cycle; line 0 takes line 1's of the last field
                    if e_dz(23) = '1' then v_a := unsigned(-e_dz); else v_a := unsigned(e_dz); end if;
                    if v_a(23) = '1' then v_a := (others => '1'); end if;
                    if e_first = '1' then e_zy <= e_zy1; else e_zy <= v_a(22 downto 14); end if;
                    if e_pass1 = '1' then e_zy1 <= v_a(22 downto 14); end if;
                when others => null;
            end case;

            if z_nw = '1' then
                n_a(to_integer(z_j)) <= z_A0;
            elsif g_start = '1' then
                for j in 0 to 3 loop n_a(j) <= n_a(j) + c_Ay(j); end loop;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [C] WAVE ENGINE, beam: free-running DDAs, loaded with the pre-pixel's
    -- values so that pixel 0 is at stage 2.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_P <= n_pu & c_P0(11 downto 0); b_Pd <= c_Pd0;
                for j in 0 to 3 loop b_A(j) <= n_a(j); end loop;
                l_zy <= e_zy;
            else
                b_P <= b_P + b_Pd; b_Pd <= b_Pd + c_Pdd;
                for j in 0 to 3 loop b_A(j) <= b_A(j) + c_Ax(j); end loop;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [C] WAVE ENGINE, pixel path (stages 3 .. 15) and [E] the colour lookup
    -- (16, 17).  G = the local ring frequency in 1/1024 cycle per pixel:
    -- octagonal |grad Z| from the beam's own first difference and the line
    -- engine's, plus the sources' fringe frequency.  n = floor(log2 G) sets the
    -- Hard gain (an edge about 1 .. 2 px wide everywhere); past 1/4 cycle per
    -- pixel the field fades to its mean instead of aliasing.
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_z  : unsigned(9 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_g  : unsigned(10 downto 0);
        variable v_c  : signed(8 downto 0);
        variable v_m  : signed(10 downto 0);
        variable v_x  : signed(10 downto 0);
        variable v_ax : unsigned(7 downto 0);
        variable v_h16 : signed(15 downto 0);
        variable v_i  : signed(13 downto 0);
        variable v_act : std_logic;
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            -- 3: theta_j (16 bits of a cycle); |Z step per pixel|
            for j in 0 to 3 loop s3_th(j) <= b_P(35 downto 20) + b_A(j)(23 downto 8); end loop;
            v_z := b_Pd(35 downto 26);
            if v_z(9) = '1' then v_z := unsigned(-signed(v_z)); end if;
            if v_z(9) = '1' then s3_zx <= (others => '1'); else s3_zx <= v_z(8 downto 0); end if;
            -- 4: quarter-wave address and sign of the cosine; max / min of the two Z steps
            for j in 0 to 3 loop
                v_p := s3_th(j)(15 downto 6);
                if v_p(8) = '1' then cs_a(j) <= not v_p(7 downto 0); else cs_a(j) <= v_p(7 downto 0); end if;
                s4_neg(j) <= v_p(9) xor v_p(8);
            end loop;
            if s3_zx > l_zy then s4_ga <= s3_zx; s4_gb <= l_zy; else s4_ga <= l_zy; s4_gb <= s3_zx; end if;
            -- 5: the ROMs' own registers (p_cos0 .. 3)
            s5_neg <= s4_neg;
            s5_g <= resize(s4_ga, 10) + resize(s4_gb(8 downto 1), 10);
            -- 6: signed cosines (a source that is off: 0 for the sum, the minimum for the max)
            for j in 0 to 3 loop
                if j < 2 then v_act := '1'; elsif j = 2 then v_act := s_ge3; else v_act := s_ge4; end if;
                v_c := signed(resize(cs_q(j), 9));
                if v_act = '0' then
                    if s_max = '1' then s6_c(j) <= to_signed(-128, 9); else s6_c(j) <= (others => '0'); end if;
                elsif s5_neg(j) = '1' then s6_c(j) <= -v_c;
                else s6_c(j) <= v_c; end if;
            end loop;
            v_g := resize(s5_g, 11) + resize(c_gm, 11);
            if v_g > 511 then s6_G <= (others => '1'); else s6_G <= v_g(8 downto 0); end if;
            -- 7: pair sums and maxima; n = floor(log2 G), the fade code
            s7_s01 <= resize(s6_c(0), 10) + resize(s6_c(1), 10);
            s7_s23 <= resize(s6_c(2), 10) + resize(s6_c(3), 10);
            if s6_c(0) > s6_c(1) then s7_m01 <= s6_c(0); else s7_m01 <= s6_c(1); end if;
            if s6_c(2) > s6_c(3) then s7_m23 <= s6_c(2); else s7_m23 <= s6_c(3); end if;
            s_n(7) <= (others => '0');
            for i in 1 to 8 loop
                if s6_G(i) = '1' then s_n(7) <= to_unsigned(i, 4); end if;
            end loop;
            if s6_G(8) = '0' then s_att(7) <= "100";
            else s_att(7) <= '0' & (not s6_G(7 downto 6)); end if;
            for i in 8 to 13 loop s_n(i) <= s_n(i - 1); end loop;
            for i in 8 to 11 loop s_att(i) <= s_att(i - 1); end loop;
            -- 8
            s8_sum <= resize(s7_s01, 11) + resize(s7_s23, 11);
            if s7_m01 > s7_m23 then s8_max <= s7_m01; else s8_max <= s7_m23; end if;
            -- 9: the field, +-127 = +-1
            if s_max = '1' then
                s9_F <= shift_left(resize(s8_max, 10), 1) - 127;
            elsif s_ge3 = '0' then
                s9_F <= s8_sum(10 downto 1);
            else
                s9_F <= resize(shift_right(s8_sum, 2), 10) + resize(shift_right(s8_sum, 3), 10);
            end if;
            -- 10: + 3/4 of the source luma
            s10_t <= resize(s9_F, 11) + signed(resize(s9_l(7 downto 1), 11)) + signed(resize(s9_l(7 downto 2), 11));
            s10_F <= s9_F;
            -- 11: the luma lifts the field as far as colour A, never into the accent
            if s10_t > 95 then v_m := to_signed(95, 11); else v_m := s10_t; end if;
            if resize(s10_F, 11) > v_m then v_x := resize(s10_F, 11); else v_x := v_m; end if;
            if v_x > 127 then s11_x <= to_signed(127, 8);
            elsif v_x < -128 then s11_x <= to_signed(-128, 8);
            else s11_x <= v_x(7 downto 0); end if;
            -- 12: fade
            if s_att(11)(2) = '1' then
                s12_xa <= s11_x;
            elsif s_att(11)(1) = '1' and s_att(11)(0) = '1' then
                s12_xa <= shift_right(s11_x, 1) + shift_right(s11_x, 2);
            elsif s_att(11)(1) = '1' then
                s12_xa <= shift_right(s11_x, 1);
            elsif s_att(11)(0) = '1' then
                s12_xa <= shift_right(s11_x, 2);
            else
                s12_xa <= (others => '0');
            end if;
            -- 13: Fill: the field itself; Lines: its distance from zero.  Smooth index.
            if s12_xa(7) = '1' then v_ax := unsigned(-s12_xa); else v_ax := unsigned(s12_xa); end if;
            if s_lines = '1' then
                s13_h <= -signed(resize(v_ax, 9));
                if v_ax(7) = '1' then s13_si <= (others => '0'); else s13_si <= (not v_ax(6 downto 0)) & '1'; end if;
            else
                s13_h <= resize(s12_xa, 9);
                s13_si <= unsigned((not s12_xa(7)) & s12_xa(6 downto 0));
            end if;
            -- 14: Hard: gain 2^(7 - n), saturated; the line's half width (at least a pixel)
            if s_n(13) = 8 then
                v_h16 := resize(shift_right(s13_h, 1), 16);
            else
                v_h16 := shift_left(resize(s13_h, 16), 7 - to_integer(s_n(13)(2 downto 0)));
            end if;
            if v_h16 > 4095 then s14_hs <= to_signed(4095, 13);
            elsif v_h16 < -4096 then s14_hs <= to_signed(-4096, 13);
            else s14_hs <= v_h16(12 downto 0); end if;
            if s_lines = '0' then
                s14_base <= to_unsigned(112, 12);
            else
                case to_integer(s_n(13)) is
                    when 0      => s14_base <= to_unsigned(112 + 3072, 12);
                    when 1      => s14_base <= to_unsigned(112 + 1536, 12);
                    when 2      => s14_base <= to_unsigned(112 + 768, 12);
                    when 3      => s14_base <= to_unsigned(112 + 384, 12);
                    when 4      => s14_base <= to_unsigned(112 + 192, 12);
                    when others => s14_base <= to_unsigned(112 + 128, 12);
                end case;
            end if;
            s14_si <= s13_si;
            -- 15: the colour index
            v_i := signed(resize(s14_base, 14)) + resize(s14_hs, 14);
            if s_hard = '0' then s15_idx <= s14_si;
            elsif v_i < 0 then s15_idx <= (others => '0');
            elsif v_i > 223 then s15_idx <= to_unsigned(223, 8);
            else s15_idx <= unsigned(v_i(7 downto 0)); end if;
            -- 16: the colour table's own registers (p_cryram, p_cruvram)
            -- 17: fabric copy; the ground colour outside pixels 0 .. W + 7
            v_win := s_gav_sr(14) or s_gav_sr(22);
            if v_win = '1' then
                s17_y <= cr_qy(9 downto 0);
                s17_u <= cr_quv(15 downto 8) & "00";
                s17_v <= cr_quv(7 downto 0) & "00";
            else
                s17_y <= c_gy; s17_u <= c_gu; s17_v <= c_gv;
            end if;
            s_hist <= s_hist(6 downto 0) & v_win;

            -- column counter of the luma line store
            if s_st(3) = '1' or s_st(4) = '1' then lb_rc <= (others => '0'); else lb_rc <= lb_rc + 1; end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 32-px box, edge-replicated
    -- and reloaded at every line start -> vertical 1/8 IIR in the line store
    -- (half horizontal resolution).  The pixel path reads the store and interpolates
    -- (stage 9).
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
            -- the half-resolution store)
            s8_q <= lb_q2; s8_qp <= s8_q;
            v_a := resize(s8_q, 9) + resize(s8_qp, 9) + 1;
            s9_l <= v_a(8 downto 1);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 18 .. 21): chroma [1 2 2 2 2 2 2 2 1]/16 as
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
            s_cu(0) <= s17_u; s_cv(0) <= s17_v;
            for i in 1 to 7 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            if s_hist = "00000000" then
                s_bu <= (c_gu & "000") + 4; s_bv2 <= (c_gv & "000") + 4;
            else
                s_bu  <= s_bu + s17_u - s_cu(7);
                s_bv2 <= s_bv2 + s17_v - s_cv(7);
            end if;
            s_bud <= s_bu; s_bvd <= s_bv2;
            -- luma taps y(n - 3), y(n - 4), y(n - 5)
            s_yd(1) <= s17_y;
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

end architecture rippletank;
