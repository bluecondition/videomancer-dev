-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ironwork v0.1.1 -- Heirloom phase 3 (video clips), from the still WIN_20230717_23_51_09 at 7.5 s.
--
-- The still: a magenta / green / black lace of contour ironwork.  Two tiles across (944 px, mirrored), rows of
-- 515 lines: bold green hearts and diamond eyes on black in the lattice rows, fine bent stripes in between
-- (lime lines on black, salmon / red / magenta bands), pink gothic arches at the bottom, torn by line-timing
-- slips.  The clip cycles the colours (~0.35 cycles/s) and sways the stripes' bend.
--
-- The patch, as fitted (ironwork.py has every table):
--   u, v   = H triangle (tile 944 px, A axes at x 524) and V triangle (row 515, troughs at y 497)
--   r      = sqrt((u + d0)^2 + (1.8 v)^2) in rings of 205 px: an offset-circle field (d0 = 236): lenses on
--            the A axes, X crossings on the B axes, diamond eyes at the B lattice corners, arches below
--   S      = a fast H oscillator (37.5 px), frequency x (1 - 0.075 cos(2 pi (y - yw) / 1024)): the bend
--   phi    = frac(r / Pr + 0.045 + As (1 - V)(S - 1/2) + etch * luma)
--   ci     = (phi in [0.10, 0.66) ? hX(y) + slope : hY(y) + slope) + cycle      two colourisers per line
--   colour = PAL[ci + 0.2 (S - 1/2)], keyed black where S < THR[ci] (the lace)
--
--   K1 Tiles   tile width (lens / eye columns): x1/3 .. x3 (512 = the still)
--   K2 Rows    row height (lattice rows): x1/3 .. x3
--   K3 Lens    the lens offset d0: circles (0) .. lenses (512) .. long flat arches (1023)
--   K4 Cycle   bipolar: colour cycle and stripe sway speed (700 = the clip, ~0.35 cycles/s)
--   K5 Rings   ring density: x1/3 .. x3
--   K6 Etch    how far the camera (or the Osc) bends the rings and the lace
--   S7 Tear    line-timing slips On / Off
--   S8 Palette Ironwork / Verdigris
--   S9 Weave   Lace (vertical stripes) / Mesh (crossed with horizontal ones)
--   S10 Key    Lace / Negative (the stripe key inverted)
--   S11 Source Video / Osc
--   P12 LACE   0 = broad calm bars over clean rings; 512 = the still; 100 % = fine filigree shredding the rings
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a 1024-entry ring table
-- X(u) = ((u + d0) / Pr)^2 refilled per field by a second-order DDA, a per-line engine (computes the NEXT
-- line: (1.8 v / Pr)^2, the stripe frequency and start, the tile start, the colour score, the tear), a
-- 24-stage pixel path (table, square root by normalise + ROM, lace, colourisers, palette, key).
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (U and V
-- through one [1 2 2 2 2 2 2 2 1]/16 box with the running-sum resync, Y [1 2 1]/4 on the same centre, luma
-- 13/16) before the blanking gate.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.ironwork_pkg.all;
use work.all;

architecture ironwork of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    constant C_HC  : integer := 16137407;   -- tile phase at the screen centre (Q24)
    constant C_VC  : integer := 9789424;    -- row phase at virtual line 540 (Q24)
    constant C_ORR : integer := 184;        -- ring offset 0.045 (Q12)
    constant C_XA  : integer := 410;        -- type X starts at 0.10 (Q12)
    constant C_WX  : integer := 2294;       -- type X width 0.56 (Q12)

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
    -- stripe triangle: 255 at phase 0 / 1, 0 at 1/2
    function f_tri8(p : unsigned(23 downto 0)) return unsigned is
    begin
        if p(23) = '1' then return p(22 downto 15); else return not p(22 downto 15); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_lz_a  : unsigned(8 downto 0) := (others => '0');            -- shared table: zone side (exp2)
    signal r_ll_a  : unsigned(8 downto 0) := (others => '0');            -- line side (cos, colour score)
    signal r_lr_q  : unsigned(15 downto 0) := (others => '0');
    signal r_sq_a  : unsigned(9 downto 0) := (others => '0');
    signal r_sq_q  : unsigned(12 downto 0) := (others => '0');
    signal r_pl_a  : unsigned(8 downto 0) := (others => '0');
    signal r_pl_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_th_a  : unsigned(8 downto 0) := (others => '0');
    signal r_th_q  : unsigned(7 downto 0) := (others => '0');

    -- ring table X(u) = ((u + d0) / Pr)^2, Q12.8, written by the per-field DDA in the vblank zone only
    type t_xt is array (0 to 1023) of unsigned(19 downto 0);
    signal xt : t_xt := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of xt : signal is true;
    signal t_we, t_run : std_logic := '0';
    signal t_wa, t_ra : unsigned(9 downto 0) := (others => '0');
    signal t_wd : unsigned(19 downto 0) := (others => '0');
    signal t_rq : unsigned(19 downto 0) := (others => '0');
    signal t_i  : unsigned(9 downto 0) := (others => '0');
    signal t_x, t_d, t_dd : unsigned(35 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
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
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_tearoff, s_pal, s_mesh, s_neg, s_src : std_logic := '0';

    -- per-field constants
    signal z_e1, z_e2, z_ea, z_eg, z_ep : signed(11 downto 0) := (others => '0');   -- exponents, Q8 octaves
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_cx   : unsigned(17 downto 0) := (others => '0');            -- exp slot constant
    signal z_sx   : unsigned(2 downto 0) := (others => '0');             -- its shift, exponent + 4
    signal c_kx   : unsigned(15 downto 0) := to_unsigned(8886, 16);     -- tile phase per h (Q24)
    signal c_kxp  : unsigned(17 downto 0) := to_unsigned(17772, 18);    -- per pixel
    signal c_kyv  : unsigned(16 downto 0) := to_unsigned(32577, 17);    -- row phase per virtual line (Q24)
    signal c_kyl  : unsigned(18 downto 0) := to_unsigned(32577, 19);    -- per line
    signal c_vp0  : unsigned(23 downto 0) := (others => '0');           -- row phase of line 0
    signal c_al   : unsigned(18 downto 0) := to_unsigned(37723, 19);    -- alpha, ring units per table step (Q24)
    signal c_c3   : unsigned(10 downto 0) := to_unsigned(256, 11);      -- lens offset / quarter tile, Q2.8
    signal c_be   : unsigned(23 downto 0) := (others => '0');           -- beta = d0 / Pr, Q6.18
    signal c_ga   : unsigned(15 downto 0) := to_unsigned(9261, 16);     -- gamma = 1.8 Py / 2 Pr, Q4.12
    signal c_aa   : unsigned(13 downto 0) := (others => '0');           -- alpha^2, Q.24
    signal c_ab   : unsigned(23 downto 0) := (others => '0');           -- alpha beta, Q.24
    signal c_x0   : unsigned(35 downto 0) := (others => '0');           -- beta^2, Q12.24
    signal c_k0   : unsigned(18 downto 0) := to_unsigned(223696, 19);   -- stripe phase per h (Q24)
    signal c_v2   : unsigned(17 downto 0) := (others => '0');           -- speed^2
    signal c_eg   : unsigned(7 downto 0) := (others => '0');            -- etch gain
    signal c_as   : unsigned(7 downto 0) := to_unsigned(90, 8);         -- lace depth into the rings (Q8 ring)
    signal c_tw   : unsigned(6 downto 0) := (others => '0');            -- threshold lift (lace > 50 %), added
    signal c_oh   : unsigned(15 downto 0) := (others => '0');           -- colour cycle (Q16)
    signal c_yw   : unsigned(17 downto 0) := (others => '0');           -- sway phase (Q8 virtual lines, mod 1024)
    signal c_ky, c_ku, c_kv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- palette black
    signal c_seed : unsigned(31 downto 0) := x"2545F491";
    signal c_hold : unsigned(5 downto 0) := (others => '0');

    -- field serial multiplier (24 iterations, result at load + 25)
    signal f_a   : unsigned(47 downto 0) := (others => '0');
    signal f_b   : unsigned(23 downto 0) := (others => '0');
    signal f_acc : unsigned(47 downto 0) := (others => '0');
    signal z_fa  : unsigned(23 downto 0) := (others => '0');
    signal z_fb  : unsigned(23 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');            -- next line, Q8 virtual lines
    signal e_vp   : unsigned(23 downto 0) := (others => '0');            -- next line's row phase
    signal e_v16  : unsigned(15 downto 0) := (others => '0');            -- V, Q16
    signal e_m16  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal e_b    : unsigned(15 downto 0) := (others => '0');            -- 1.8 v / Pr, Q4.12
    signal e_kk   : unsigned(18 downto 0) := (others => '0');
    signal e_vi   : unsigned(5 downto 0) := (others => '0');
    signal e_vf   : unsigned(7 downto 0) := (others => '0');
    signal e_x0, e_x1, e_y0, e_y1 : unsigned(7 downto 0) := (others => '0');
    signal e_dx, e_dy : unsigned(7 downto 0) := (others => '0');
    signal e_sx, e_sy : std_logic := '0';
    signal m_a    : unsigned(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    -- tear generator
    signal t_rx   : unsigned(31 downto 0) := x"2545F491";
    signal t_rem  : unsigned(14 downto 0) := (others => '0');           -- band lines left, Q8 virtual lines
    signal t_gap  : unsigned(16 downto 0) := (others => '0');           -- lines to the next band, Q8 virtual lines
    signal t_off  : signed(17 downto 0) := (others => '0');             -- band offset, Q8 h
    signal t_dl   : unsigned(1 downto 0) := (others => '0');            -- shear: 00 = -1, 11 = +1 h per virtual line
    signal t_adv  : std_logic := '0';                                   -- draw new tear numbers on this line
    signal e_th   : signed(9 downto 0) := (others => '0');              -- next line's tear (h)
    -- next line's values (loaded at g_start)
    signal n_y    : unsigned(15 downto 0) := (others => '0');            -- (1.8 v / Pr)^2, Q8.8
    signal n_sp0  : unsigned(23 downto 0) := (others => '0');
    signal n_ksp  : unsigned(20 downto 0) := (others => '0');
    signal n_hp0  : unsigned(23 downto 0) := (others => '0');
    signal n_hx, n_hy : unsigned(7 downto 0) := (others => '0');
    signal n_asl  : unsigned(7 downto 0) := (others => '0');
    signal n_sy   : unsigned(7 downto 0) := (others => '0');

    -- beam
    signal b_hp, b_sp : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_y   : unsigned(15 downto 0) := (others => '0');
    signal l_ksp : unsigned(20 downto 0) := (others => '0');
    signal l_hx, l_hy, l_asl, l_sy : unsigned(7 downto 0) := (others => '0');

    -- pixel path (stage numbers in the names)
    signal p2_sp, p3_sp, p4_sp, p5_sp, p6_sp : unsigned(15 downto 0) := (others => '0');   -- stripe phase, Q16
    signal p4_x   : unsigned(19 downto 0) := (others => '0');
    signal p4_lea, p4_leb : unsigned(11 downto 0) := (others => '0');
    signal p5_s2, p6_s2 : unsigned(19 downto 0) := (others => '0');
    signal p5_le  : unsigned(15 downto 0) := (others => '0');
    signal p6_le, p7_le, p8_le, p9_le : unsigned(11 downto 0) := (others => '0');
    signal p6_k, p7_k, p8_k, p9_k : unsigned(2 downto 0) := (others => '0');
    type t_s8 is array (natural range <>) of unsigned(7 downto 0);
    signal p_s    : t_s8(7 to 12) := (others => (others => '0'));       -- stripe value S at stages 7 .. 12
    signal p9_r   : unsigned(12 downto 0) := (others => '0');
    signal p10_phi, p11_phi : unsigned(11 downto 0) := (others => '0');
    signal p10_q  : unsigned(11 downto 0) := (others => '0');
    signal p10_ma, p10_mb : unsigned(10 downto 0) := (others => '0');
    signal p10_neg, p11_neg : std_logic := '0';
    signal p11_pa : unsigned(11 downto 0) := (others => '0');
    signal p12_pa : unsigned(11 downto 0) := (others => '0');
    signal p13_inx : std_logic := '0';
    signal p13_sx : unsigned(7 downto 0) := (others => '0');
    signal p13_sy : unsigned(7 downto 0) := (others => '0');
    signal p13_bt, p14_bt : unsigned(7 downto 0) := (others => '0');
    signal p14_ci : unsigned(7 downto 0) := (others => '0');
    signal p15_ic, p16_ic, p17_ic : unsigned(8 downto 0) := (others => '0');
    signal p17_th, p18_t : unsigned(7 downto 0) := (others => '0');
    signal p19_d  : signed(9 downto 0) := (others => '0');
    signal p19_lo, p19_hi : std_logic := '0';
    signal p20_pal : std_logic_vector(31 downto 0) := (others => '0');
    signal p20_lv : unsigned(1 downto 0) := (others => '0');            -- 0 black, 1 half, 2 full
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two 8-sums, the 8-back samples from the chroma
    -- ring), luma [1 2 1]/4 on the same centre
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- block-RAM rings, one write pointer: syncs (22 back + 2 registers = C_LAT), chroma (6 back + 2 = the
    -- 8-sample box delay), stripe value S (4 back + 2: stage 12 -> stage 18)
    type t_ring4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal r_sring : t_ring4 := (others => (others => '1'));
    attribute no_rw_check of r_sring : signal is true;
    type t_ring16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_cring : t_ring16 := (others => x"8080");
    attribute no_rw_check of r_cring : signal is true;
    type t_ring8 is array (0 to 255) of std_logic_vector(7 downto 0);
    signal r_pring : t_ring8 := (others => (others => '0'));
    attribute no_rw_check of r_pring : signal is true;
    signal r_wp : unsigned(7 downto 0) := (others => '0');
    signal r_sq, r_sq2 : std_logic_vector(3 downto 0) := (others => '1');
    signal r_cq, r_cq2 : std_logic_vector(15 downto 0) := x"8080";
    signal r_pq, r_pq2 : std_logic_vector(7 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ring table, ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_xt : process(clk)
    begin
        if rising_edge(clk) then
            t_rq <= xt(to_integer(t_ra));
            if t_we = '1' then xt(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_xt;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_sring(to_integer(r_wp)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            r_sq <= r_sring(to_integer(r_wp - 22));
            r_sq2 <= r_sq;
            r_cring(to_integer(r_wp)) <= std_logic_vector(s_u21(9 downto 2)) & std_logic_vector(s_v21(9 downto 2));
            r_cq <= r_cring(to_integer(r_wp - 6));
            r_cq2 <= r_cq;
            r_pring(to_integer(r_wp)) <= std_logic_vector(p_s(12));
            r_pq <= r_pring(to_integer(r_wp - 4));
            r_pq2 <= r_pq;
            r_wp <= r_wp + 1;
        end if;
    end process p_ring;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_lr_q <= C_IW_LROM(to_integer(r_lz_a or r_ll_a));
            r_sq_q <= C_IW_SQRT(to_integer(r_sq_a));
            r_pl_q <= C_IW_PAL(to_integer(r_pl_a));
            r_th_q <= C_IW_THR(to_integer(r_th_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(11 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_r  : unsigned(31 downto 0);
        variable v_t  : unsigned(47 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_x  : unsigned(23 downto 0);
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
                s_tearoff <= registers_in(6)(0);
                s_pal     <= registers_in(6)(1);
                s_mesh    <= registers_in(6)(2);
                s_neg     <= registers_in(6)(3);
                s_src     <= registers_in(6)(4);
                s_xs      <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters
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

            -- serial products: slot k loads at zc = 64 + 28 k, 24 iterations, result stable from load + 25
            -- (stored at load + 26).  The exp2 ROM is addressed at load - 4, the slot number decoded at
            -- load - 2 and the operands pre-muxed into z_fa / z_fb at load - 1.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 62  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 90  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 118 => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 146 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 202 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 258 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 286 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 314 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 370 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 398 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 426 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            -- exp slots: (C << (sh + 4)) * mt, so every result is acc >> 19 (one shared barrel)
            v_x := shift_left(resize(z_cx, 24), to_integer(z_sx));
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= v_x;                            z_fb <= resize(z_mt, 24);          -- kx
                    when 1  => z_fa <= v_x;                            z_fb <= resize(z_mt, 24);          -- kyv
                    when 2  => z_fa <= resize(c_kyv, 24);              z_fb <= resize(s_ys, 24);          -- kyl
                    when 3  => z_fa <= resize(c_kyv, 24);              z_fb <= to_unsigned(540, 24);      -- vp0
                    when 4  => z_fa <= v_x;                            z_fb <= resize(z_mt, 24);          -- alpha
                    when 5  => z_fa <= resize(c_al, 24);               z_fb <= resize(c_c3, 24);          -- beta
                    when 6  => z_fa <= v_x;                            z_fb <= resize(z_mt, 24);          -- gamma
                    when 7  => z_fa <= resize(c_al, 24);               z_fb <= resize(c_al, 24);          -- alpha^2
                    when 8  => z_fa <= resize(c_al, 24);               z_fb <= c_be;                      -- alpha beta
                    when 9  => z_fa <= resize(c_be, 24);               z_fb <= c_be;                      -- beta^2
                    when 10 => z_fa <= v_x;                            z_fb <= resize(z_mt, 24);          -- k0
                    when 11 => z_fa <= resize(s_vm, 24);               z_fb <= resize(s_vm, 24);          -- speed^2
                    when 12 => z_fa <= resize(s_g6(12 downto 5), 24);  z_fb <= resize(s_g6(12 downto 5), 24); -- etch^2
                    when others =>                                                                        -- lace depth
                        if s_gp(12 downto 3) > 512 then
                            z_fa <= resize(s_gp(12 downto 3) - 512, 24); z_fb <= to_unsigned(115, 24);
                        else
                            z_fa <= resize(s_gp(12 downto 3), 24);       z_fb <= to_unsigned(90, 24);
                        end if;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 48); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(46 downto 0) & '0';
                f_b <= '0' & f_b(23 downto 1);
            end if;

            r_lz_a <= (others => '0');
            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps); tear seed hold; palette black
                    when 20 =>
                        v_r := c_seed;
                        if s_vneg = '1' then
                            c_oh <= c_oh - resize(shift_right(c_v2, 6) - shift_right(c_v2, 8), 16);
                            c_yw <= c_yw - (shift_right(c_v2, 5) + shift_right(c_v2, 6));
                        else
                            c_oh <= c_oh + resize(shift_right(c_v2, 6) - shift_right(c_v2, 8), 16);
                            c_yw <= c_yw + (shift_right(c_v2, 5) + shift_right(c_v2, 6));
                        end if;
                        if c_hold = 0 then
                            v_r := v_r xor shift_left(v_r, 13);
                            v_r := v_r xor shift_right(v_r, 17);
                            v_r := v_r xor shift_left(v_r, 5);
                            c_seed <= v_r;
                            c_hold <= resize(v_r(4 downto 0), 6) + 8;
                        else
                            c_hold <= c_hold - 1;
                        end if;
                        if s_pal = '1' then v_k := C_IW_K1; else v_k := C_IW_K0; end if;
                        c_ky <= unsigned(v_k(31 downto 22)); c_ku <= unsigned(v_k(21 downto 12)); c_kv <= unsigned(v_k(11 downto 2));
                    -- exponents (Q8 octaves) from the glided knobs: x 2^(+-1.585) over each knob
                    when 40 =>
                        v_d := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        z_e1 <= shift_right(v_d, 1) + shift_right(v_d, 2) + shift_right(v_d, 5) + shift_right(v_d, 7);
                        v_d := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        z_e2 <= shift_right(v_d, 1) + shift_right(v_d, 2) + shift_right(v_d, 5) + shift_right(v_d, 7);
                        v_d := signed(resize(s_g5(12 downto 3), 12)) - 512;
                        z_eg <= shift_right(v_d, 1) + shift_right(v_d, 2) + shift_right(v_d, 5) + shift_right(v_d, 7);
                        -- lace: stripes 2 octaves wider at 0, 1 octave finer at 1023
                        v_d := signed(resize(s_gp(12 downto 3), 12)) - 512;
                        if v_d < 0 then z_ep <= v_d; else z_ep <= shift_right(v_d, 1); end if;
                        -- lens: d0 / (tile / 4) = 0 .. 1 below 512, 1 .. 4 above
                        if s_g3(12 downto 3) > 512 then
                            c_c3 <= to_unsigned(256, 11) + resize(s_g3(12 downto 3) - 512, 11) + resize(shift_right(s_g3(12 downto 3) - 512, 1), 11);
                        else
                            c_c3 <= resize(shift_right(s_g3(12 downto 3), 1), 11);
                        end if;
                        -- threshold lift
                        if s_gp(12 downto 3) > 512 then c_tw <= resize(shift_right(s_gp(12 downto 3) - 512, 3), 7);
                        else c_tw <= (others => '0'); end if;
                    when 41 =>
                        z_ea <= z_eg - z_e1;          -- alpha ~ rings / tiles
                        v_e := z_eg - z_e2;           -- gamma ~ rings / rows, capped at 2.73 octaves (16 bits)
                        if v_e > 700 then z_eg <= to_signed(700, 12); else z_eg <= v_e; end if;
                    -- exp2 lookups at load - 4: mantissa at load - 2 (the shift is read from the exponent at the store)
                    when 60  => r_lz_a <= '0' & unsigned(z_e1(7 downto 0)); z_cx <= to_unsigned(8886, 18); z_sx <= unsigned(z_e1(10 downto 8)) + 4;
                    when 62  => z_mt <= r_lr_q;
                    when 88  => r_lz_a <= '0' & unsigned(z_e2(7 downto 0)); z_cx <= to_unsigned(32577, 18); z_sx <= unsigned(z_e2(10 downto 8)) + 4;
                    when 90  => z_mt <= r_lr_q;
                    when 170 => r_lz_a <= '0' & unsigned(z_ea(7 downto 0)); z_cx <= to_unsigned(37723, 18); z_sx <= unsigned(z_ea(10 downto 8)) + 4;
                    when 172 => z_mt <= r_lr_q;
                    when 226 => r_lz_a <= '0' & unsigned(z_eg(7 downto 0)); z_cx <= to_unsigned(9261, 18); z_sx <= unsigned(z_eg(10 downto 8)) + 4;
                    when 228 => z_mt <= r_lr_q;
                    when 338 => r_lz_a <= '0' & unsigned(z_ep(7 downto 0)); z_cx <= to_unsigned(223696, 18); z_sx <= unsigned(z_ep(10 downto 8)) + 4;
                    when 340 => z_mt <= r_lr_q;
                    -- results (load + 26): value = acc >> (15 - sh)
                    when others => null;
                end case;
                case to_integer(s_zc) is
                    when 90  => c_kx  <= f_acc(34 downto 19);       -- 8886 2^e1
                    when 91 =>
                        case to_integer(s_xs) is
                            when 4      => c_kxp <= resize(c_kx & "00", 18);
                            when 3      => c_kxp <= resize(c_kx & '0', 18) + resize(c_kx, 18);
                            when others => c_kxp <= resize(c_kx & '0', 18);
                        end case;
                    when 118 => c_kyv <= f_acc(35 downto 19);       -- 32577 2^e2
                    when 146 => c_kyl <= f_acc(26 downto 8);
                    when 174 => c_vp0 <= to_unsigned(C_VC, 24) - f_acc(23 downto 0);
                    when 202 => c_al  <= f_acc(37 downto 19);       -- 37723 2^ea
                    when 230 => c_be <= f_acc(28 downto 5);
                    when 258 => c_ga  <= f_acc(34 downto 19);       -- 9261 2^eg
                    when 286 => c_aa <= f_acc(37 downto 24);
                    when 314 => c_ab <= f_acc(41 downto 18);
                    when 342 => c_x0 <= f_acc(47 downto 12);
                    when 370 => c_k0  <= f_acc(37 downto 19);       -- 223696 2^ep
                    when 398 => c_v2 <= f_acc(17 downto 0);
                    when 426 => c_eg <= f_acc(15 downto 8);
                    when 454 =>
                        if s_gp(12 downto 3) > 512 then c_as <= f_acc(16 downto 9) + 90;
                        else c_as <= f_acc(16 downto 9); end if;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Ring table fill (vblank zone, zc 480 .. 1503): X(i) = (alpha i + beta)^2 by a second-order DDA,
    -- X(i + 1) = X(i) + D(i), D(i + 1) = D(i) + 2 alpha^2, in Q12.24; the table keeps Q12.8
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            t_we <= '0';
            if s_zone = '1' and s_zc = 470 then
                t_x  <= c_x0;
                t_d  <= resize(c_aa, 36) + resize(c_ab & '0', 36);
                t_dd <= resize(c_aa & '0', 36);
            end if;
            if s_zone = '1' and s_zc = 480 then
                t_run <= '1'; t_i <= (others => '0');
            elsif t_run = '1' then
                t_we <= '1'; t_wa <= t_i; t_wd <= t_x(35 downto 16);
                t_x <= t_x + t_d; t_d <= t_d + t_dd; t_i <= t_i + 1;
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
    -- write of (luma >> 2) - 16
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
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17): the row term, the stripe frequency and start, the tile start,
    -- the colour score, the lace depth, the mesh stripe and the tear.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_r  : unsigned(31 downto 0);
        variable v_v  : unsigned(23 downto 0);
        variable v_c  : signed(13 downto 0);
        variable v_w  : signed(12 downto 0);
        variable v_q  : unsigned(23 downto 0);
        variable v_p  : signed(9 downto 0);
        variable v_e  : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1540 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_vp <= c_vp0;
                e_lc <= (others => '0');
                t_rx <= c_seed; t_rem <= (others => '0'); t_gap <= resize(c_seed(7 downto 0) & x"00", 17); t_adv <= '1';
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_vp <= e_vp + resize(c_kyl, 24);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            r_ll_a <= (others => '0');
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if e_vp(23) = '1' then v_v := e_vp - to_unsigned(8388608, 24);
                    else v_v := to_unsigned(8388608, 24) - e_vp; end if;
                    if v_v(23) = '1' then e_v16 <= (others => '1'); else e_v16 <= v_v(22 downto 7); end if;
                    v_q := resize(e_vy, 24) - resize(c_yw, 24) - to_unsigned(125440, 24);   -- sway phase: bend centred on line 490
                    r_ll_a <= '1' & v_q(17 downto 10);
                    if t_adv = '1' then t_rx <= t_rx xor shift_left(t_rx, 13); end if;
                when 2 => if t_adv = '1' then t_rx <= t_rx xor shift_right(t_rx, 17); end if;
                when 3 =>
                    if t_adv = '1' then t_rx <= t_rx xor shift_left(t_rx, 5); end if;
                    -- m = 1 - 0.075 cos, Q1.15 (the shared table's word is valid in this clock only)
                    v_c := resize(signed(r_lr_q(7 downto 0)), 14);
                    v_c := shift_left(v_c, 4) + shift_left(v_c, 1) + v_c;
                    v_e := to_signed(32768, 17) - resize(v_c, 17);
                    e_m16 <= unsigned(v_e(15 downto 0));
                when 4 =>
                    -- tear, counted in virtual lines (s_ys per output line): the first band starts 0 .. 255 lines into
                    -- the field, the next ones 160 .. 415 lines after the previous band ends (2 - 4 bands, ~16 % of
                    -- the lines); a band is 24 .. 87 lines of the same picture shifted 40 .. 292 h (20 .. 146 virtual
                    -- px), straight or sheared smoothly by 1 h per virtual line.  The numbers are drawn only when a band
                    -- starts (t_adv), so the layout in virtual lines is the same at every resolution.
                    t_adv <= '0';
                    if s_tearoff = '1' then
                        t_rem <= (others => '0');
                    elsif t_rem /= 0 then
                        if t_rem > s_ys then t_rem <= t_rem - s_ys; else t_rem <= (others => '0'); end if;
                        if t_dl = "00" then t_off <= t_off - signed(resize(s_ys, 18));
                        elsif t_dl = "11" then t_off <= t_off + signed(resize(s_ys, 18)); end if;
                    elsif t_gap > s_ys then
                        t_gap <= t_gap - s_ys;
                    else
                        t_rem <= (resize(t_rx(5 downto 0), 7) + 24) & x"00";
                        v_p := signed(resize(t_rx(11 downto 6) & "00", 10)) + 40;
                        if t_rx(12) = '1' then t_off <= -(resize(v_p, 10) & x"00"); else t_off <= resize(v_p, 10) & x"00"; end if;
                        t_dl <= t_rx(14 downto 13);
                        t_gap <= (resize(t_rx(22 downto 15), 9) + 160) & x"00";
                        t_adv <= '1';
                    end if;
                when 5 =>
                    if t_rem = 0 or s_tearoff = '1' then e_th <= (others => '0');
                    else e_th <= t_off(17 downto 8); end if;
                    m_a <= resize(e_v16, 40); m_b <= c_ga; m_acc <= (others => '0');                -- V gamma
                when 22 =>
                    e_b <= m_acc(31 downto 16);
                    m_a <= resize(m_acc(31 downto 16), 40); m_b <= m_acc(31 downto 16); m_acc <= (others => '0');
                when 39 =>
                    n_y <= m_acc(31 downto 16);                                                       -- (V gamma)^2
                    m_a <= resize(c_k0, 40); m_b <= e_m16; m_acc <= (others => '0');                   -- k0 m
                when 56 =>
                    e_kk <= m_acc(33 downto 15);
                    m_a <= resize(m_acc(33 downto 15), 40);
                    m_b <= unsigned(resize(signed(resize(s_x0, 13)) + resize(e_th, 13) + 440, 16));     -- x - x0 (h), >= 6
                    m_acc <= (others => '0');
                when 57 =>
                    case to_integer(s_xs) is
                        when 4      => n_ksp <= resize(e_kk & "00", 21);
                        when 3      => n_ksp <= resize(e_kk & '0', 21) + resize(e_kk, 21);
                        when others => n_ksp <= resize(e_kk & '0', 21);
                    end case;
                when 73 =>
                    n_sp0 <= m_acc(23 downto 0);
                    v_w := to_signed(1920, 13) - signed(resize(s_x0, 13)) - resize(e_th, 13);
                    m_a <= resize(c_kx, 40); m_b <= unsigned(resize(v_w, 16)); m_acc <= (others => '0');   -- tile start
                when 90 =>
                    n_hp0 <= to_unsigned(C_HC, 24) - m_acc(23 downto 0);
                    m_a <= resize(e_vy, 40); m_b <= to_unsigned(3884, 16); m_acc <= (others => '0');      -- score segment
                when 107 =>
                    if m_acc(30) = '1' or m_acc(29 downto 24) = 63 then e_vi <= to_unsigned(62, 6); e_vf <= (others => '1');
                    else e_vi <= m_acc(29 downto 24); e_vf <= m_acc(23 downto 16); end if;
                when 108 => r_ll_a <= "100" & e_vi;                  -- hX[i]
                when 109 => r_ll_a <= "100" & (e_vi + 1);            -- hX[i + 1]
                when 110 => r_ll_a <= "101" & e_vi;  e_x0 <= r_lr_q(15 downto 8);
                when 111 => r_ll_a <= "101" & (e_vi + 1); e_x1 <= r_lr_q(15 downto 8);
                when 112 => e_y0 <= r_lr_q(15 downto 8);
                when 113 => e_y1 <= r_lr_q(15 downto 8);
                when 114 =>
                    e_dx <= e_x1 - e_x0; e_dy <= e_y1 - e_y0;
                when 115 =>
                    e_sx <= e_dx(7); e_sy <= e_dy(7);
                    if e_dx(7) = '1' then m_a <= resize(0 - e_dx, 40); else m_a <= resize(e_dx, 40); end if;
                    m_b <= resize(e_vf, 16); m_acc <= (others => '0');
                when 132 =>
                    if e_sx = '1' then e_x1 <= e_x0 - m_acc(15 downto 8); else e_x1 <= e_x0 + m_acc(15 downto 8); end if;
                    if e_dy(7) = '1' then m_a <= resize(0 - e_dy, 40); else m_a <= resize(e_dy, 40); end if;
                    m_b <= resize(e_vf, 16); m_acc <= (others => '0');
                when 149 =>
                    if e_sy = '1' then e_y1 <= e_y0 - m_acc(15 downto 8); else e_y1 <= e_y0 + m_acc(15 downto 8); end if;
                    m_a <= resize(c_as, 40); m_b <= to_unsigned(256, 16) - resize(e_v16(15 downto 8), 16); m_acc <= (others => '0');
                when 150 => n_hx <= e_x1 + c_oh(15 downto 8) - 27;    -- + colour cycle, - type X slope centre
                when 151 => n_hy <= e_y1 + c_oh(15 downto 8) + 7;
                when 166 =>
                    n_asl <= m_acc(15 downto 8);
                    m_a <= resize(c_k0, 40); m_b <= e_vy(18 downto 3); m_acc <= (others => '0');           -- mesh stripe
                when 183 =>
                    n_sy <= f_tri8(m_acc(27 downto 4));
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: tile phase and stripe phase DDAs (start values from the line engine)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_hp <= n_hp0; b_sp <= n_sp0; b_px <= (others => '0');
                l_y <= n_y; l_ksp <= n_ksp; l_hx <= n_hx; l_hy <= n_hy; l_asl <= n_asl; l_sy <= n_sy;
            elsif g_avid = '1' then
                b_hp <= b_hp + resize(c_kxp, 24);
                b_sp <= b_sp + resize(l_ksp, 24);
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u  : unsigned(9 downto 0);
        variable v_s  : unsigned(20 downto 0);
        variable v_sv : signed(8 downto 0);
        variable v_mg : unsigned(6 downto 0);
        variable v_t  : unsigned(14 downto 0);
        variable v_b  : signed(13 downto 0);
        variable v_ph : unsigned(11 downto 0);
        variable v_tt : unsigned(8 downto 0);
        variable v_sx : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: tile fold -> table address
            if b_hp(23) = '1' then v_u := b_hp(22 downto 13); else v_u := not b_hp(22 downto 13); end if;
            t_ra  <= v_u;
            p2_sp <= b_sp(23 downto 8);
            -- 3: table (EBR), luma (line buffer)
            p3_sp <= p2_sp;
            -- 4: fabric copy; etch partials
            p4_x  <= t_rq;
            p4_sp <= p3_sp;
            p4_lea <= resize(unsigned(s_lb_q(3 downto 0)) * c_eg, 12);
            p4_leb <= resize(unsigned(s_lb_q(7 downto 4)) * c_eg, 12);
            -- 5: r^2 / Pr^2 = X + Y (saturated); etch
            v_s := resize(p4_x, 21) + resize(l_y, 21);
            if v_s(20) = '1' then p5_s2 <= (others => '1'); else p5_s2 <= v_s(19 downto 0); end if;
            p5_le <= resize(p4_lea, 16) + shift_left(resize(p4_leb, 16), 4);
            p5_sp <= p4_sp;
            -- 6: normalise (even shift); lace phase + etch (le << 10 in Q24)
            if p5_s2(19 downto 18) /= "00" then p6_k <= to_unsigned(5, 3);
            elsif p5_s2(17 downto 16) /= "00" then p6_k <= to_unsigned(4, 3);
            elsif p5_s2(15 downto 14) /= "00" then p6_k <= to_unsigned(3, 3);
            elsif p5_s2(13 downto 12) /= "00" then p6_k <= to_unsigned(2, 3);
            elsif p5_s2(11 downto 10) /= "00" then p6_k <= to_unsigned(1, 3);
            else p6_k <= to_unsigned(0, 3); end if;
            p6_s2 <= p5_s2;
            p6_sp <= p5_sp + shift_left(p5_le, 2);
            p6_le <= p5_le(13 downto 2);
            -- 7: mantissa -> sqrt ROM address; stripe value (mesh: crossed with the line's stripe)
            case to_integer(p6_k) is
                when 5      => r_sq_a <= p6_s2(19 downto 10);
                when 4      => r_sq_a <= p6_s2(17 downto 8);
                when 3      => r_sq_a <= p6_s2(15 downto 6);
                when 2      => r_sq_a <= p6_s2(13 downto 4);
                when 1      => r_sq_a <= p6_s2(11 downto 2);
                when others => r_sq_a <= p6_s2(9 downto 0);
            end case;
            p7_k <= p6_k;
            if p6_sp(15) = '1' then v_sx := p6_sp(14 downto 7); else v_sx := not p6_sp(14 downto 7); end if;
            if s_mesh = '1' and l_sy > v_sx then p_s(7) <= l_sy; else p_s(7) <= v_sx; end if;
            p7_le <= p6_le;
            -- 8: sqrt ROM (EBR)
            p8_k <= p7_k; p8_le <= p7_le;
            -- 9: fabric copy
            p9_r <= r_sq_q; p9_k <= p8_k; p9_le <= p8_le;
            -- 10: r / Pr (Q12, fraction only); lace partials; ring offset + etch
            case to_integer(p9_k) is
                when 5      => v_ph := p9_r(6 downto 0) & "00000";
                when 4      => v_ph := p9_r(7 downto 0) & "0000";
                when 3      => v_ph := p9_r(8 downto 0) & "000";
                when 2      => v_ph := p9_r(9 downto 0) & "00";
                when 1      => v_ph := p9_r(10 downto 0) & '0';
                when others => v_ph := p9_r(11 downto 0);
            end case;
            p10_phi <= v_ph;
            v_sv := signed(resize(p_s(9), 9)) - 128;
            if v_sv < 0 then v_mg := unsigned(resize(-v_sv, 7)); p10_neg <= '1';
            else v_mg := unsigned(v_sv(6 downto 0)); p10_neg <= '0'; end if;
            p10_ma <= resize(l_asl(3 downto 0) * v_mg, 11);
            p10_mb <= resize(l_asl(7 downto 4) * v_mg, 11);
            p10_q <= to_unsigned(C_ORR, 12) + p9_le;
            -- 11: phase + offset + etch; lace term
            p11_phi <= p10_phi + p10_q;
            v_t := resize(p10_ma, 15) + shift_left(resize(p10_mb, 15), 4);
            p11_pa <= resize(shift_right(v_t, 4), 12);
            p11_neg <= p10_neg;
            -- 12: + lace (signed), - type X start
            if p11_neg = '1' then p12_pa <= p11_phi - p11_pa - C_XA;
            else p12_pa <= p11_phi + p11_pa - C_XA; end if;
            -- 13: type and slopes; colour jitter 0.2 (S - 1/2)
            if p12_pa < C_WX then p13_inx <= '1'; else p13_inx <= '0'; end if;
            p13_sx <= resize(shift_right(p12_pa, 6) + shift_right(p12_pa, 7), 8);
            p13_sy <= resize(shift_right(p12_pa - C_WX, 7), 8);
            v_sv := signed(resize(p_s(12), 9)) - 128;
            v_b := shift_left(resize(v_sv, 14), 5) + shift_left(resize(v_sv, 14), 4) + shift_left(resize(v_sv, 14), 1) + resize(v_sv, 14);
            p13_bt <= unsigned(resize(shift_right(v_b, 8), 8));          -- (S - 128) * 51 / 256
            -- 14: colouriser index (the line's score already holds the cycle and the slope centres)
            if p13_inx = '1' then p14_ci <= l_hx + p13_sx;
            else p14_ci <= l_hy - p13_sy; end if;
            p14_bt <= p13_bt;
            -- 15: threshold address; colour index
            r_th_a <= '0' & p14_ci;
            p15_ic <= s_pal & (p14_ci + p14_bt);
            -- 16: threshold ROM (EBR)
            p16_ic <= p15_ic;
            -- 17: fabric copy
            p17_th <= r_th_q; p17_ic <= p16_ic;
            -- 18: palette address; T' = T + lift (saturated)
            r_pl_a <= p17_ic;
            v_tt := resize(p17_th, 9) + resize(c_tw, 9);
            if v_tt(8) = '1' then p18_t <= (others => '1'); else p18_t <= v_tt(7 downto 0); end if;
            -- 19: palette ROM (EBR); key distance (S of this pixel from the ring)
            p19_d <= signed(resize(unsigned(r_pq2), 10)) - signed(resize(p18_t, 10));
            if p18_t < 8 then p19_lo <= '1'; else p19_lo <= '0'; end if;
            if p18_t >= 250 then p19_hi <= '1'; else p19_hi <= '0'; end if;
            -- 20: fabric copy; key level (0 black, 1 half, 2 full), Negative swaps black and full
            p20_pal <= r_pl_q;
            if p19_hi = '0' and (p19_lo = '1' or p19_d >= 20) then
                if s_neg = '1' then p20_lv <= "00"; else p20_lv <= "10"; end if;
            elsif p19_hi = '0' and p19_d >= 0 then
                p20_lv <= "01";
            else
                if s_neg = '1' then p20_lv <= "10"; else p20_lv <= "00"; end if;
            end if;
            -- 21: compose; black outside the generated avid
            if s_gav_sr(18) = '0' or p20_lv = "00" then
                s_y21 <= c_ky; s_u21 <= c_ku; s_v21 <= c_kv;
            elsif p20_lv = "01" then
                s_y21 <= resize(shift_right(resize(unsigned(p20_pal(31 downto 22)), 11) + c_ky, 1), 10);
                s_u21 <= resize(shift_right(resize(unsigned(p20_pal(21 downto 12)), 11) + c_ku, 1), 10);
                s_v21 <= resize(shift_right(resize(unsigned(p20_pal(11 downto 2)), 11) + c_kv, 1), 10);
            else
                s_y21 <= unsigned(p20_pal(31 downto 22)); s_u21 <= unsigned(p20_pal(21 downto 12)); s_v21 <= unsigned(p20_pal(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(18);
            -- stripe value delay line (stages 8 .. 12; the ring carries it on to stage 18)
            for i in 8 to 12 loop p_s(i) <= p_s(i - 1); end loop;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning (the HD HDMI bar fix): U and V through one
    -- [1 2 2 2 2 2 2 2 1]/16 box (two adjacent 8-sums, resynced to the palette
    -- black once 8 blank samples are in), Y through [1 2 1]/4 on the same centre
    -- (4 pixels back), luma 13/16, two delay stages and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ly : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 22: chroma 8-sums (8-bit samples; the ring returns the sample 8 back), luma window
            s_cay(0) <= s_y21;
            for i in 1 to 5 loop s_cay(i) <= s_cay(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= c_ku(9 downto 2) & "000"; s_bav <= c_kv(9 downto 2) & "000";
            else
                s_bau <= s_bau + s_u21(9 downto 2) - unsigned(r_cq2(15 downto 8));
                s_bav <= s_bav + s_v21(9 downto 2) - unsigned(r_cq2(7 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            -- 23: outputs (palette black off the generated avid)
            if s_gav_sr(20) = '1' then
                v_sc := resize(s_bau, 12) + s_baud; s_u23 <= v_sc(11 downto 2);
                v_sc := resize(s_bav, 12) + s_bavd; s_v23 <= v_sc(11 downto 2);
                v_ly := resize(s_cay(3), 12) + shift_left(resize(s_cay(4), 12), 1) + resize(s_cay(5), 12);
                s_y23 <= v_ly(11 downto 2);
            else
                s_y23 <= c_ky; s_u23 <= c_ku; s_v23 <= c_kv;
            end if;
            -- 24: luma 13/16
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u23; s_v24 <= s_v23;
            -- 25: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= r_sq2(0);
    data_out.vsync_n <= r_sq2(1);
    data_out.field_n <= r_sq2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ironwork;
