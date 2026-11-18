-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Hinge v0.1.0 -- Straightedge.  The rotating-squares auxetic mechanism (Grima and Evans, 2000; Ron Resch's
-- folded sheets; kinetic facades): a lattice of rigid squares hinged at their corners.  As alternate squares
-- turn opposite ways the sheet opens: pointed rhombic holes grow between the squares and the lattice pitch
-- swells as s (cos t + sin t).  Every edge on the screen is a straight line.
--
--   cell (i, j) of pitch a = s (cos t + sin t), local (lx, ly) from the cell centre;
--   u = lx cos t + sg ly sin t,  v = ly cos t - sg lx sin t;  inside the square when max(|u|, |v|) < half side
--   Hinged:   half side s / 2, sg = +1 / -1 by the parity of i + j (the mechanism)
--   Pinwheel: the same pitch, sg the same for every square and the half side grown to a cos t / 2, so each
--             corner touches its neighbour's side and the holes are small tilted squares (the Pythagorean
--             tiling); a pixel past its own square's side belongs to the neighbour across that side or to
--             the hole
--   t = P12 * 90 degrees, folded at 45 (a square turned by t is the square turned by t - 90)
--
--   K1  X       lattice pan, +-1.5 cells              K2  Y   the same, vertically
--   K3  Scale   square side 64 .. 724 virtual px (exponential)
--   K4  Nest    1 .. 6 nested square bands in every square (stepped)
--   K5  Drift   the lattice glides on a diagonal (x at twice the y rate) and the bands stream into (right) or
--               out of (left) every square's centre; centre +-24 = still; 2^-12 .. 2^-4 of two cells per field
--   K6  Camera  a halftone: every 32-virtual-px screen block holds one luma level, which sets the size of the
--               small square in that block.  Bold: the small squares shrink with the luma and cut the sheet;
--               Fine: they grow with it, as dots inside the outlined squares.  Default 100 %
--   S7  Glide   Off = the knobs act this field;  On = 1/16 of the gap per field (Drift speed 1/8)
--   S8  Turn    Hinged / Pinwheel
--   S9  Tone    Grey (white and light-grey squares, bands one step darker) / Ink (black and white)
--   S10 Weight  Bold (filled squares, equal bands) / Fine (outlines of the squares and of every band)
--   S11 Source  Video / Osc (the shell's drifting triangle lattice at twice the camera level)
--   P12 Open    the turn angle 0 .. 90 degrees: closed sheet, checkerboard on the diagonal at 50 %, closed again
--
-- No multiplier in the pixel path except the final tone * coverage (shift-adds).  The beam runs a cell
-- counter and lx cos t, lx sin t (Q14 virtual px) that step back by one cell at every cell crossing; the line
-- engine keeps ly cos t and ly sin t the same way.  The camera dots need only the pixel's phase in its block.
-- All coefficients come once per field from one serial multiplier and a sin / cos table.
--
-- Sections (the integer model .straightedge_work/rb_hinge/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide (S7), switches, Nest zones, Drift speed glide
--   [C] FIELD SEQUENCER                  one serial multiplier: size, angle, pitch, start values, band and
--                                        camera constants, the dot size table
--   [L] LINE ENGINE                      the row's ly cos t, ly sin t, parity; the block row and its phase
--   [P] PIXEL PATH                       accumulators -> (u, v) -> Pinwheel neighbour -> edge distance ->
--                                        bands, coverage, the camera dot -> tone
--   [D] SOURCE FIELD                     luma (or Osc) -> 32-px box -> vertical one-pole per block column ->
--                                        4-bit grid (32 virtual px blocks), two banks ping-ponged per field
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 toward black, blanking gate; U = V = 512
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  Pixels
-- 0 .. W + 1 are rendered; output column j is the [1 2 1] filter over pixels j .. j + 2 and lands on screen
-- column j + 4 (the generated avid starts 4 clocks after the delayed source avid), so pixel m sits at screen
-- column m + 3.  s_gav_sr(S - 3) flags pixels 0 .. W - 1 on their way into a stage-S register.  One latency
-- (24) in every mode.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.hinge_pkg.all;
use work.all;

architecture hinge of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
    constant C_DEF_K5  : integer := 660;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 228;

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
    function f_as(a, b : signed; sub : std_logic) return signed is
        variable v_a, v_b, v_r : signed(a'length downto 0);
        variable v_o : signed(a'length - 1 downto 0);
    begin
        v_a := a & '1';
        if sub = '1' then v_b := (not b) & '1'; else v_b := b & '0'; end if;
        v_r := v_a + v_b;
        v_o := v_r(a'length downto 1);
        return v_o;
    end function;
    -- |x| as x xor sign (one's complement)
    function f_oabs(x : signed) return unsigned is
        variable v_x : signed(x'length - 1 downto 0);
        variable v_o : unsigned(x'length - 2 downto 0);
    begin
        v_x := x;
        if v_x(v_x'high) = '1' then v_o := unsigned(not v_x(v_x'high - 1 downto 0));
        else v_o := unsigned(v_x(v_x'high - 1 downto 0)); end if;
        return v_o;
    end function;
    -- a signed distance (Q4 virtual px) saturated to -32 .. 31: all the coverage needs
    function f_sat(x : signed) return signed is
        variable v_x : signed(x'length - 1 downto 0);
        variable v_s : signed(6 downto 0);
        variable v_hi : boolean;
        variable v_lo : boolean;
    begin
        v_x := x;
        v_hi := false; v_lo := false;
        for i in 5 to v_x'high - 1 loop
            if v_x(i) = '1' then v_hi := true; end if;
            if v_x(i) = '0' then v_lo := true; end if;
        end loop;
        if v_x(v_x'high) = '0' and v_hi then v_s := to_signed(31, 7);
        elsif v_x(v_x'high) = '1' and v_lo then v_s := to_signed(-32, 7);
        else v_s := resize(v_x(5 downto 0), 7); end if;
        return v_s;
    end function;
    -- coverage 0 .. 16 from the saturated distance: clamp(8 + 16 d / pixel), pixel = xs / 2 virtual px
    function f_cv(s : signed(6 downto 0); xs : unsigned(2 downto 0)) return unsigned is
        variable v_t : signed(6 downto 0);
        variable v_o : unsigned(4 downto 0);
    begin
        if xs = 2 then v_t := s;
        elsif xs = 4 then v_t := shift_right(s, 1);
        else v_t := shift_right(s, 1) + shift_right(s, 3) + shift_right(s, 5); end if;
        v_t := v_t + 8;
        if v_t(6) = '1' then v_o := (others => '0');
        elsif v_t > 16 then v_o := to_unsigned(16, 5);
        else v_o := unsigned(v_t(4 downto 0)); end if;
        return v_o;
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

    -- [B] knobs: +-1 hold (10 bits), glide (Q3)
    signal h_1 : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal h_2 : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal h_3 : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal h_5 : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal h_6 : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal h_p : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal g_1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal g_2 : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal g_3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal g_6 : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);
    signal g_p : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(13 downto 0) := (others => '0');
    signal s_gr, s_gh : unsigned(9 downto 0) := (others => '0');
    signal z_gp, z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_pin, s_ink, s_fine, s_src : std_logic := '0';
    signal s_ish3 : std_logic := '1';                              -- the vertical one-pole's shift: 3 (else 2)
    signal s_bank : std_logic := '0';                              -- the grid bank written this field
    signal s_vtg, s_vdd : signed(16 downto 0) := (others => '0');
    signal s_vgs : signed(16 downto 0) := to_signed((C_DEF_K5 - 512) * 64, 17);   -- Drift speed glide, Q6
    signal z_n6  : unsigned(12 downto 0) := (others => '0');
    signal z_nz  : unsigned(2 downto 0) := (others => '0');        -- Nest - 1

    -- ROM: 2^(i/256), sin, cos
    signal r_ex_a : unsigned(9 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');

    -- [C] the serial multiplier and the sequencer's scratch
    signal f_a, f_acc : unsigned(41 downto 0) := (others => '0');
    signal f_b : unsigned(19 downto 0) := (others => '0');
    signal z_fa : unsigned(20 downto 0) := (others => '0');
    signal z_fb : unsigned(19 downto 0) := (others => '0');
    signal z_sx : std_logic := '0';
    signal z_sel, z_selb : unsigned(4 downto 0) := (others => '0');   -- two copies: one per operand mux
    attribute keep : boolean;
    attribute keep of z_sel : signal is true;
    attribute keep of z_selb : signal is true;
    signal z_rcp : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_pre, z_ld : std_logic := '0';
    signal z_av  : unsigned(9 downto 0) := (others => '0');
    signal z_vz, z_vn : std_logic := '1';
    signal z_ls  : unsigned(9 downto 0) := (others => '0');
    signal z_spd : unsigned(13 downto 0) := (others => '0');
    signal z_ssh : unsigned(2 downto 0) := (others => '0');
    signal z_D   : unsigned(19 downto 0) := (others => '0');       -- Drift: 2^18 = one cell of x glide
    signal z_oct : unsigned(9 downto 0) := (others => '0');
    signal z_flip : std_logic := '0';
    signal z_ang : unsigned(12 downto 0) := (others => '0');
    signal z_s0, z_c0 : unsigned(15 downto 0) := (others => '0');
    signal z_ds, z_dc : unsigned(9 downto 0) := (others => '0');
    signal z_fac : unsigned(16 downto 0) := (others => '0');
    signal z_io  : unsigned(4 downto 0) := (others => '0');
    signal z_pg  : unsigned(12 downto 0) := (others => '0');
    signal z_pp  : unsigned(17 downto 0) := (others => '0');
    signal z_t17 : unsigned(16 downto 0) := (others => '0');
    signal z_l, z_lo : signed(20 downto 0) := (others => '0');
    signal z_par, z_lp : std_logic := '0';
    signal z_Pd  : unsigned(14 downto 0) := (others => '0');
    signal z_npsi : unsigned(9 downto 0) := (others => '0');
    signal z_stp : unsigned(12 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_s9  : unsigned(19 downto 0) := (others => '0');       -- square side, Q9 virtual px
    signal c_sn, c_cs : unsigned(15 downto 0) := (others => '0');  -- sin, cos of the folded turn, Q15
    signal c_a8  : unsigned(18 downto 0) := to_unsigned(65536, 19);   -- lattice pitch, Q8 virtual px
    signal c_CA, c_SA : unsigned(23 downto 0) := (others => '0');  -- a cos, a sin, Q14
    signal c_hf  : unsigned(13 downto 0) := (others => '0');       -- half side, Q4
    signal c_h4  : signed(15 downto 0) := (others => '0');         -- half side less the joint inset
    signal c_px0 : std_logic := '0';
    signal c_F0, c_Fw : signed(20 downto 0) := (others => '0');
    signal c_Fs  : signed(20 downto 0) := to_signed(256, 21);
    signal c_P0, c_Q0, c_Pw, c_Qw : signed(25 downto 0) := (others => '0');
    signal c_cst, c_sst : unsigned(15 downto 0) := (others => '0');
    signal c_cys, c_sys : unsigned(17 downto 0) := (others => '0');
    signal c_wb, c_lw, c_won : unsigned(13 downto 0) := to_unsigned(32, 14);
    signal c_Pd  : unsigned(14 downto 0) := (others => '0');
    signal c_Pd15 : unsigned(15 downto 0) := (others => '0');     -- one and a half periods
    signal c_offc : unsigned(15 downto 0) := (others => '0');
    signal c_ho  : signed(17 downto 0) := (others => '0');         -- half side + stream offset
    signal c_hw  : unsigned(12 downto 0) := (others => '0');
    signal c_n1  : std_logic := '1';                               -- Nest = 1
    signal c_bxlo : unsigned(5 downto 0) := (others => '0');
    signal c_bxhi : unsigned(5 downto 0) := to_unsigned(59, 6);
    signal c_xn0 : unsigned(7 downto 0) := to_unsigned(18, 8);     -- pixel 1's phase in its block, 1/4 vpx
    signal c_kn0 : unsigned(7 downto 0) := to_unsigned(6, 8);      -- the source's second sample, the same
    signal c_col0 : unsigned(5 downto 0) := (others => '0');

    -- [C] the dot size table (16 levels), filled in the zone
    type t_tb is array (0 to 255) of signed(15 downto 0);
    signal tb_ram : t_tb := (others => (others => '0'));
    attribute no_rw_check of tb_ram : signal is true;
    signal tb_q, tb_wd : signed(15 downto 0) := (others => '0');
    signal tb_wa : unsigned(3 downto 0) := (others => '0');
    signal tb_we : std_logic := '0';
    signal t_acc : unsigned(16 downto 0) := (others => '0');
    signal t_i   : unsigned(4 downto 0) := (others => '0');
    signal t_run, t_va, t_ge, t_z : std_logic := '0';
    signal t_ia  : unsigned(3 downto 0) := (others => '0');
    signal t_mn, t_mx : unsigned(8 downto 0) := (others => '0');

    -- [L] line engine
    signal e_st  : unsigned(1 downto 0) := (others => '1');
    signal e_wa, e_wb : std_logic := '0';
    signal n_G   : signed(20 downto 0) := (others => '0');
    signal n_T, n_R : signed(25 downto 0) := (others => '0');
    signal l_T, l_R : signed(15 downto 0) := (others => '0');
    signal n_py, l_py, n_first, l_first, l_rev : std_logic := '0';
    signal l_ay  : unsigned(8 downto 0) := (others => '0');
    signal n_yq  : unsigned(12 downto 0) := (others => '0');
    signal n_row, l_row : unsigned(6 downto 0) := (others => '0');

    -- [P] pixel path
    signal b_F   : signed(20 downto 0) := (others => '0');
    signal b_P, b_Q : signed(25 downto 0) := (others => '0');
    signal b_px  : std_logic := '0';
    signal b_xn  : unsigned(7 downto 0) := (others => '0');
    signal b_col : unsigned(5 downto 0) := (others => '0');
    signal b_xq  : unsigned(6 downto 0) := (others => '0');
    signal s2_u, s2_v, s3_u, s3_v, s4_u, s4_v, s5_u, s5_v : signed(15 downto 0) := (others => '0');
    signal s2_ax : unsigned(7 downto 0) := (others => '0');
    signal s2_par, s2_sgn, s3_par, s3_sgn, s4_par, s4_nb : std_logic := '0';
    signal s3_f : std_logic_vector(3 downto 0) := (others => '0');
    signal s2_col : unsigned(5 downto 0) := (others => '0');
    signal s4_mu, s4_mv : unsigned(13 downto 0) := (others => '0');
    signal s4_bu, s4_bv : std_logic := '0';
    signal s3_am : unsigned(9 downto 0) := (others => '0');
    type t_am is array (4 to 7) of unsigned(9 downto 0);
    signal s_am : t_am := (others => (others => '0'));
    signal s3_ga : unsigned(11 downto 0) := (others => '0');
    signal s5_L  : unsigned(3 downto 0) := (others => '0');
    signal s_parp : std_logic_vector(5 to 14) := (others => '0');
    signal s6_mx : unsigned(14 downto 0) := (others => '0');
    signal s7_d, s8_x4, s8_dm : signed(15 downto 0) := (others => '0');
    signal s8_ds, s9_x4s, s9_dms, s12_xs : signed(6 downto 0) := (others => '0');
    signal s7_hm : signed(15 downto 0) := (others => '0');
    signal s7_e, s8_e, s9_e : unsigned(16 downto 0) := (others => '0');
    signal s10_q : signed(17 downto 0) := (others => '0');
    type t_cov is array (natural range <>) of unsigned(4 downto 0);
    signal s_covd : t_cov(9 to 14) := (others => (others => '0'));
    signal s_covm : t_cov(10 to 14) := (others => (others => '0'));
    signal s_edge : t_cov(11 to 13) := (others => (others => '0'));
    signal s10_ce : unsigned(4 downto 0) := (others => '0');
    signal s11_x : signed(17 downto 0) := (others => '0');
    signal s13_ln, s14_cst, s15_cov, s16_cov : unsigned(4 downto 0) := (others => '0');
    signal s15_m : unsigned(10 downto 0) := (others => '0');
    signal s16_m73 : unsigned(17 downto 0) := (others => '0');
    signal s17_pa, s17_pb : unsigned(22 downto 0) := (others => '0');
    signal s18_s : unsigned(9 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_l8, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    -- block columns
    signal k_n   : unsigned(7 downto 0) := (others => '0');
    signal k_col, k_evc, e_col : unsigned(5 downto 0) := (others => '0');
    signal k_ev, e_v : std_logic := '0';
    -- the one-pole state per block column (Q8.4)
    type t_st is array (0 to 255) of unsigned(15 downto 0);
    signal st_ram : t_st := (others => (others => '0'));
    attribute no_rw_check of st_ram : signal is true;
    signal st_q, st_wd : unsigned(15 downto 0) := (others => '0');
    signal st_wa : unsigned(5 downto 0) := (others => '0');
    signal st_we : std_logic := '0';
    signal s_eb : unsigned(7 downto 0) := (others => '0');
    signal s_eq : unsigned(11 downto 0) := (others => '0');
    -- the grid: 2 banks x 32 rows x 64 columns x 4 bits
    type t_gr is array (0 to 4095) of unsigned(3 downto 0);
    signal gr_ram : t_gr := (others => (others => '0'));
    attribute no_rw_check of gr_ram : signal is true;
    signal gr_q, gr_wd : unsigned(3 downto 0) := (others => '0');
    signal gr_wa : unsigned(11 downto 0) := (others => '0');
    signal gr_we : std_logic := '0';

    -- [F] output conditioning
    signal s19_y, s20_y, s21_y, s22_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s22_ya : unsigned(11 downto 0) := (others => '0');
    signal s23_y, s24_y, s25_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROM and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_HG_ROM(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
        end if;
    end process p_rom;

    -- box delay: read 32 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_l8; end if;
        end if;
    end process p_bxram;

    -- one-pole state: read at the column about to end, written three clocks after it ended
    p_stram : process(clk)
    begin
        if rising_edge(clk) then
            st_q <= st_ram(to_integer(resize(k_col, 8)));
            if st_we = '1' then st_ram(to_integer(resize(st_wa, 8))) <= st_wd; end if;
        end if;
    end process p_stram;

    -- the grid: the pixel path reads the bank written last field, the source writes the other
    p_grram : process(clk)
    begin
        if rising_edge(clk) then
            gr_q <= gr_ram(to_integer(s3_ga));
            if gr_we = '1' then gr_ram(to_integer(gr_wa)) <= gr_wd; end if;
        end if;
    end process p_grram;

    -- dot size table: written in the zone only
    p_tbram : process(clk)
    begin
        if rising_edge(clk) then
            tb_q <= tb_ram(to_integer(resize(s5_L, 8)));
            if tb_we = '1' then tb_ram(to_integer(resize(tb_wa, 8))) <= tb_wd; end if;
        end if;
    end process p_tbram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(10 downto 0);
        variable v_v  : signed(10 downto 0);
        variable v_n11 : signed(10 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_g  : unsigned(16 downto 0);
        variable v_p  : unsigned(17 downto 0);
        variable v_x0 : signed(20 downto 0);
        variable v_m  : unsigned(10 downto 0);
        variable v_cs : unsigned(23 downto 0);
        variable v_raw : unsigned(8 downto 0);
        variable v_c18 : unsigned(17 downto 0);
        variable v_sl : unsigned(13 downto 0);
        variable v_bias : signed(15 downto 0);
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

            -- [A][B] per-field latches; the grid banks swap here (once per field: the zone needs 1024
            -- clocks without active video, so serrated vsync cannot fire it twice)
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_pin   <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_bank  <= not s_bank;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); c_Fs <= to_signed(512, 21);
                                   c_bxlo <= to_unsigned(7, 6); c_bxhi <= to_unsigned(51, 6); c_col0 <= to_unsigned(7, 6);
                                   c_xn0 <= to_unsigned(92 + 8, 8); c_kn0 <= to_unsigned(68 + 8, 8);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_Fs <= to_signed(384, 21);
                                   c_bxlo <= to_unsigned(0, 6); c_bxhi <= to_unsigned(59, 6); c_col0 <= to_unsigned(0, 6);
                                   c_xn0 <= to_unsigned(21 + 6, 8); c_kn0 <= to_unsigned(3 + 6, 8);
                    when others => s_oxs <= to_unsigned(102, 9); c_Fs <= to_signed(256, 21);
                                   c_bxlo <= to_unsigned(0, 6); c_bxhi <= to_unsigned(59, 6); c_col0 <= to_unsigned(0, 6);
                                   c_xn0 <= to_unsigned(14 + 4, 8); c_kn0 <= to_unsigned(2 + 4, 8);
                end case;
                if s_ys <= 512 then s_ish3 <= '1'; else s_ish3 <= '0'; end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared hold + glide, five clocks per channel.  Hold: the knob moves the held value only by
            -- 2 counts or more (or to either end).  Glide On: 1/16 of the gap per field, never slower than
            -- one step (1/8 count; X and Y two steps), landing exactly.  Glide Off: the held value, this field.
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= g_1; s_gh <= h_1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= g_2; s_gh <= h_2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= g_3; s_gh <= h_3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= g_6; s_gh <= h_6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= g_p; s_gh <= h_p; s_gr <= unsigned(registers_in(7));
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
                            if z_gc(2 downto 1) = "00" then                -- X, Y: at least 2 steps
                                if s_gd > 32 or s_gd < -32 then s_gd <= shift_right(s_gd, 4);
                                elsif s_gd > 2 then s_gd <= to_signed(2, 14);
                                elsif s_gd < -2 then s_gd <= to_signed(-2, 14);
                                end if;
                            else
                                if s_gd > 16 or s_gd < -16 then s_gd <= shift_right(s_gd, 4);
                                elsif s_gd > 1 then s_gd <= to_signed(1, 14);
                                elsif s_gd < -1 then s_gd <= to_signed(-1, 14);
                                end if;
                            end if;
                        end if;
                        z_gp <= "100";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => g_1 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_1 <= s_gh;
                            when 1      => g_2 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_2 <= s_gh;
                            when 2      => g_3 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_3 <= s_gh;
                            when 3      => g_6 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_6 <= s_gh;
                            when others => g_p <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_p <= s_gh;
                        end case;
                        z_gp <= "000";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] serial products: a slot is decoded at zc = P (below), its operands are pre-muxed one
            -- clock later, it loads the clock after that, runs 20 iterations and is stored at P + 24.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 52  => z_sel <= to_unsigned(1, 5);  z_selb <= to_unsigned(1, 5);  z_pre <= '1';
                    when 80  => z_sel <= to_unsigned(2, 5);  z_selb <= to_unsigned(2, 5);  z_pre <= '1';
                    when 108 => z_sel <= to_unsigned(3, 5);  z_selb <= to_unsigned(3, 5);  z_pre <= '1';
                    when 136 => z_sel <= to_unsigned(4, 5);  z_selb <= to_unsigned(4, 5);  z_pre <= '1';
                    when 164 => z_sel <= to_unsigned(5, 5);  z_selb <= to_unsigned(5, 5);  z_pre <= '1';
                    when 194 => z_sel <= to_unsigned(6, 5);  z_selb <= to_unsigned(6, 5);  z_pre <= '1';
                    when 252 => z_sel <= to_unsigned(7, 5);  z_selb <= to_unsigned(7, 5);  z_pre <= '1';
                    when 280 => z_sel <= to_unsigned(8, 5);  z_selb <= to_unsigned(8, 5);  z_pre <= '1';
                    when 310 => z_sel <= to_unsigned(6, 5);  z_selb <= to_unsigned(6, 5);  z_pre <= '1';
                    when 368 => z_sel <= to_unsigned(7, 5);  z_selb <= to_unsigned(7, 5);  z_pre <= '1';
                    when 396 => z_sel <= to_unsigned(8, 5);  z_selb <= to_unsigned(8, 5);  z_pre <= '1';
                    when 424 => z_sel <= to_unsigned(12, 5); z_selb <= to_unsigned(12, 5); z_pre <= '1';
                    when 452 => z_sel <= to_unsigned(13, 5); z_selb <= to_unsigned(13, 5); z_pre <= '1';
                    when 480 => z_sel <= to_unsigned(14, 5); z_selb <= to_unsigned(14, 5); z_pre <= '1';
                    when 508 => z_sel <= to_unsigned(15, 5); z_selb <= to_unsigned(15, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_sx <= '0';
                case to_integer(z_sel) is
                    when 1  => z_fa <= resize(z_ds, 21);                                         -- sin
                    when 2  => z_fa <= resize(z_dc, 21);                                         -- cos
                    when 3  => z_fa <= resize(c_s9, 21);                                         -- the pitch
                    when 4 | 5 => z_fa <= resize(c_a8, 21);                                      -- a cos, a sin
                    -- signed operands are sign-extended into the 42-bit shifter: the product is exact mod 2^42
                    when 6  => z_fa <= unsigned(resize(signed((not z_t17(15)) & z_t17(14 downto 0)), 21));
                               z_sx <= '1';                                                      -- the centre's lx / ly
                    when 7 | 8 => z_fa <= unsigned(z_l); z_sx <= '1';                            -- l cos, l sin
                    when 12 => z_fa <= resize(c_cs, 21);                                         -- per field line
                    when 13 => z_fa <= resize(c_sn, 21);
                    when 14 => z_fa <= resize(c_hf, 21);                                         -- band width
                    when others => z_fa <= resize(z_Pd, 21);                                     -- stream offset
                end case;
                case to_integer(z_selb) is
                    when 1 | 2  => z_fb <= resize(z_ang(4 downto 0), 20);
                    when 3      => z_fb <= resize(z_fac, 20);
                    when 4 | 7  => z_fb <= resize(c_cs, 20);
                    when 5 | 8  => z_fb <= resize(c_sn, 20);
                    when 6      => z_fb <= resize(c_a8, 20);
                    when 12 | 13 => z_fb <= resize(s_ys, 20);
                    when 14     => z_fb <= resize(z_rcp, 20);
                    when others => z_fb <= resize(z_npsi, 20);
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                if z_sx = '1' then f_a <= unsigned(resize(signed(z_fa), 42)); else f_a <= resize(z_fa, 42); end if;
                f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(40 downto 0) & '0';
                f_b <= '0' & f_b(19 downto 1);
            end if;

            -- the dot size table: 16 entries, two clocks deep.  Bold: level 0 covers everything, then the dot
            -- shrinks from the block-filling size;  Fine: no dot, then one that grows.
            v_bias := signed(resize(s_xs & "00", 16)) + 4;
            tb_we <= t_va; tb_wa <= t_ia;
            if s_fine = '1' then
                if t_ge = '1' then tb_wd <= signed(resize(t_mn, 16)) - v_bias;
                else tb_wd <= to_signed(-512, 16); end if;
            elsif t_z = '1' then tb_wd <= to_signed(2047, 16);
            else tb_wd <= signed(resize(t_mx, 16)) + v_bias; end if;
            t_va <= '0';
            if t_run = '1' then
                v_raw := t_acc(16 downto 8);
                if v_raw >= 64 and t_i /= 0 then t_ge <= '1'; else t_ge <= '0'; end if;
                if t_i = 0 then t_z <= '1'; else t_z <= '0'; end if;
                if v_raw < 205 then t_mn <= v_raw; else t_mn <= to_unsigned(205, 9); end if;
                if v_raw > 64 then t_mx <= v_raw; else t_mx <= to_unsigned(64, 9); end if;
                t_ia <= t_i(3 downto 0); t_va <= '1';
                if s_fine = '1' then t_acc <= t_acc + resize(z_stp, 17);
                elsif t_i = 0 then null;                                  -- Bold: level 1 still fills its block
                elsif t_acc < resize(z_stp, 17) then t_acc <= (others => '0');
                else t_acc <= t_acc - resize(z_stp, 17); end if;
                if t_i = 15 then t_run <= '0'; end if;
                t_i <= t_i + 1;
            end if;

            -- bring the local coordinate into its cell: 32 conditional steps of one cell (zc 219 .. 250 for y,
            -- 335 .. 366 for x; the window flag is registered so the range decode is off the adder's path)
            if (s_zc >= 218 and s_zc <= 249) or (s_zc >= 334 and s_zc <= 365) then z_lp <= '1'; else z_lp <= '0'; end if;
            if z_lp = '1' and z_l < z_lo then z_l <= z_l + signed(resize(c_a8, 21)); z_par <= not z_par; end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- [B] Drift: hold, then the speed glide (1/8 of the gap per field), Q6
                    when 33 =>
                        v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(h_5, 11));
                        if unsigned(registers_in(4)) = 0 or unsigned(registers_in(4)) = 1023 or v_d > 1 or v_d < -1 then
                            h_5 <= unsigned(registers_in(4));
                        end if;
                    when 34 => s_vtg <= shift_left(resize(signed('0' & h_5) - 512, 17), 6);
                    when 35 => s_vdd <= s_vtg - s_vgs;
                    when 36 =>
                        if s_glide = '1' then s_vgs <= s_vgs + shift_right(s_vdd, 3);
                        else s_vgs <= s_vtg; end if;
                    -- [B] Drift speed: |v| < 24 still; else 2^-12 .. 2^-4 of two cells per field
                    when 37 =>
                        v_v := resize(shift_right(s_vgs, 6), 11);
                        z_vn <= v_v(10);
                        v_n11 := -v_v;
                        if v_v(10) = '1' then z_av <= unsigned(v_n11(9 downto 0)); else z_av <= unsigned(v_v(9 downto 0)); end if;
                        -- [B] Nest: six zones; a new zone is taken 8 counts inside it, or at either end
                        v_k := unsigned(registers_in(3));
                        z_n6 <= resize(v_k & "00", 13) + resize(v_k & '0', 13);
                        -- Scale: octaves above 64 virtual px, Q8
                        v_g := shift_left(resize(g_3, 17), 3) - resize(g_3, 17);
                        z_oct <= v_g(15 downto 6);
                    when 38 =>
                        if z_av < 24 then z_vz <= '1'; z_av <= (others => '0');
                        else z_vz <= '0'; z_av <= z_av - 24; end if;
                        v_k := unsigned(registers_in(3));
                        if v_k = 0 or v_k = 1023 or (z_n6(9 downto 0) >= 48 and z_n6(9 downto 0) <= 975) then
                            z_nz <= z_n6(12 downto 10);
                        end if;
                    when 39 => z_ls <= z_av + shift_right(z_av, 5);
                    when 40 => r_ex_a <= "00" & z_ls(5 downto 0) & "00";
                    when 41 =>
                        r_ex_a <= "00" & z_oct(7 downto 0);
                        -- Open: the turn, folded at 45 degrees
                        if g_p(12 downto 3) = 1023 then z_flip <= '1'; z_ang <= (others => '0');
                        elsif g_p > 4096 then z_flip <= '1'; z_ang <= (not g_p) + 1;
                        else z_flip <= '0'; z_ang <= g_p; end if;
                        if z_nz = 0 then c_n1 <= '1'; else c_n1 <= '0'; end if;
                        case to_integer(z_nz) is                      -- 1 / Nest, Q16
                            when 1      => z_rcp <= to_unsigned(32768, 16);
                            when 2      => z_rcp <= to_unsigned(21845, 16);
                            when 3      => z_rcp <= to_unsigned(16384, 16);
                            when 4      => z_rcp <= to_unsigned(13107, 16);
                            when others => z_rcp <= to_unsigned(10923, 16);
                        end case;
                    when 43 =>
                        if z_vz = '1' then z_spd <= (others => '0');
                        else z_spd <= resize(r_exf(15 downto 9), 14); end if;
                        z_ssh <= z_ls(8 downto 6);
                    when 44 =>
                        case to_integer(z_oct(9 downto 8)) is
                            when 0      => c_s9 <= resize(r_exf, 20);
                            when 1      => c_s9 <= resize(r_exf & '0', 20);
                            when 2      => c_s9 <= resize(r_exf & "00", 20);
                            when others => c_s9 <= resize(r_exf & "000", 20);
                        end case;
                        case to_integer(z_ssh) is
                            when 0      => null;
                            when 1      => z_spd <= shift_left(z_spd, 1);
                            when 2      => z_spd <= shift_left(z_spd, 2);
                            when 3      => z_spd <= shift_left(z_spd, 3);
                            when 4      => z_spd <= shift_left(z_spd, 4);
                            when 5      => z_spd <= shift_left(z_spd, 5);
                            when 6      => z_spd <= shift_left(z_spd, 6);
                            when others => z_spd <= shift_left(z_spd, 7);
                        end case;
                    when 45 =>
                        z_D <= f_addsub(z_D, resize(z_spd & '0', 20), z_vn);
                        r_ex_a <= to_unsigned(256, 10) + resize(z_ang(12 downto 5), 10);
                    when 46 => r_ex_a <= to_unsigned(257, 10) + resize(z_ang(12 downto 5), 10);
                    when 47 => r_ex_a <= to_unsigned(512, 10) + resize(z_ang(12 downto 5), 10);
                    when 48 => r_ex_a <= to_unsigned(513, 10) + resize(z_ang(12 downto 5), 10); z_s0 <= r_exf;
                    when 49 => z_ds <= resize(r_exf - z_s0, 10);
                    when 50 => z_c0 <= r_exf;
                    when 51 => z_dc <= resize(z_c0 - r_exf, 10);
                    when 76  => c_sn <= z_s0 + f_acc(20 downto 5);
                    when 104 => c_cs <= z_c0 - f_acc(20 downto 5);
                    when 105 => z_fac <= resize(c_cs, 17) + resize(c_sn, 17);
                    when 132 => c_a8 <= f_acc(34 downto 16);
                    when 160 => c_CA <= f_acc(32 downto 9);
                    when 188 => c_SA <= f_acc(32 downto 9);
                    when 189 =>                       -- half side (Q4); the joint inset (0 .. 1 virtual px)
                        if s_pin = '1' then
                            c_hf <= resize(c_CA(23 downto 11), 14);
                            v_cs := c_CA - c_SA;
                            if v_cs(23 downto 13) > 16 then z_io <= to_unsigned(16, 5); else z_io <= v_cs(17 downto 13); end if;
                        else
                            c_hf <= c_s9(19 downto 6);
                            if c_SA(23 downto 13) > 16 then z_io <= (others => '0');
                            else z_io <= to_unsigned(16, 5) - c_SA(17 downto 13); end if;
                        end if;
                    when 190 =>
                        c_h4 <= signed(resize(c_hf, 16)) - signed(resize(z_io, 16));
                        z_lo <= -signed(resize(c_a8(18 downto 1), 21));
                        z_pg <= g_2;
                    -- the lattice coordinate of the screen centre: pan (+-1.5 cells) less the glide, Q16 cells
                    -- mod 2; then line 0's local y: the centre's, less the half screen, brought into its cell
                    -- (g - 4096) * 24 + 32768 = g * 24 - 65536: the 17-bit result is g * 24 with bit 16 inverted
                    when 191 | 306 => z_pp <= resize(z_pg & "0000", 18) + resize(z_pg & "000", 18);
                    when 192 =>
                        v_p := z_pp - resize(z_D(19 downto 3), 18);
                        z_t17 <= (not v_p(16)) & v_p(15 downto 0);
                    when 218 =>
                        z_l <= resize(signed(f_acc(35 downto 16)), 21) + signed(resize(s_ys(11 downto 1), 21)) - 138240;
                        z_par <= z_t17(16);
                    -- 276, 304, 306: the line engine takes line 0's ly cos, ly sin, lead and parity
                    when 305 =>
                        z_l <= z_l - z_lo;
                        z_pg <= g_1;
                    when 307 =>
                        v_p := z_pp - resize(z_D(19 downto 2), 18);
                        z_t17 <= (not v_p(16)) & v_p(15 downto 0);
                    when 334 =>                       -- pixel 0's local x
                        case to_integer(s_xs) is
                            when 4      => v_x0 := to_signed(-182528, 21);
                            when 3      => v_x0 := to_signed(-244416, 21);
                            when others => v_x0 := to_signed(-244864, 21);
                        end case;
                        z_l <= resize(signed(f_acc(35 downto 16)), 21) + v_x0;
                        z_par <= z_t17(16);
                    when 392 => c_P0 <= signed(f_acc(34 downto 9));
                    when 420 => c_Q0 <= signed(f_acc(34 downto 9)); c_px0 <= z_par;
                    when 421 => z_l <= z_l - z_lo;
                    when 422 =>
                        c_F0 <= z_l - signed(resize(c_a8, 21)) + c_Fs;
                        c_Fw <= c_Fs - signed(resize(c_a8, 21));
                    when 448 => c_cys <= f_acc(26 downto 9);
                    when 476 => c_sys <= f_acc(26 downto 9);
                    when 477 =>                       -- per pixel: cos * xs / 2, sin * xs / 2 (Q14)
                        case to_integer(s_xs) is
                            when 4      => c_cst <= c_cs; c_sst <= c_sn;
                            when 3      =>
                                v_c18 := resize(c_cs & '0', 18) + resize(c_cs, 18); c_cst <= v_c18(17 downto 2);
                                v_c18 := resize(c_sn & '0', 18) + resize(c_sn, 18); c_sst <= v_c18(17 downto 2);
                            when others => c_cst <= '0' & c_cs(15 downto 1); c_sst <= '0' & c_sn(15 downto 1);
                        end case;
                    when 478 =>
                        c_Pw <= signed(resize(c_cst, 26)) - signed(resize(c_CA, 26));
                        c_Qw <= signed(resize(c_sst, 26)) - signed(resize(c_SA, 26));
                    when 504 =>
                        if z_nz = 0 then c_wb <= c_hf; else c_wb <= f_acc(29 downto 16); end if;
                    when 505 =>
                        if s_fine = '1' then z_Pd <= resize(c_wb, 15); z_npsi <= not z_D(16 downto 7);
                        else z_Pd <= c_wb & '0'; z_npsi <= not z_D(17 downto 8); end if;
                        if c_wb(13 downto 3) > 32 then c_lw <= resize(c_wb(13 downto 3), 14);
                        else c_lw <= to_unsigned(32, 14); end if;
                    when 506 =>
                        if s_fine = '1' then c_won <= c_lw; else c_won <= c_wb; end if;
                    when 507 => c_hw <= c_won(13 downto 1);
                    when 532 =>                       -- the stripe's centre lands on the half period
                        c_offc <= resize(f_acc(24 downto 10), 16) + resize(z_Pd(14 downto 1), 16) - resize(c_hw, 16);
                        c_Pd <= z_Pd;
                        c_Pd15 <= resize(z_Pd, 16) + resize(z_Pd(14 downto 1), 16);
                    when 535 => c_ho <= resize(c_h4, 18) + signed(resize(c_offc, 18));
                    when 533 =>                       -- the dot's shrink (Bold) / growth (Fine) per level, Q12
                        v_sl := g_6(12 downto 3) & "0000";
                        if s_fine = '1' then z_stp <= resize(v_sl(13 downto 2), 13) + resize(v_sl(13 downto 4), 13);
                        else z_stp <= resize(v_sl(13 downto 2), 13) + resize(v_sl(13 downto 5), 13) + resize(v_sl(13 downto 9), 13); end if;
                    when 534 =>
                        if s_fine = '1' then t_acc <= (others => '0'); else t_acc <= (16 => '1', others => '0'); end if;
                        t_i <= (others => '0'); t_run <= '1';
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

    ---------------------------------------------------------------------------
    -- [L] LINE ENGINE: the row's values are taken at the line start; the NEXT
    -- line's are stepped right after, in two clocks (step, then one cell / one
    -- block back where the line crossed).  Line 0's come straight from the
    -- sequencer's products in the zone.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' then
                e_st <= "11";
                case to_integer(s_zc) is
                    when 276 => n_T <= signed(f_acc(34 downto 9));
                    when 304 => n_R <= signed(f_acc(34 downto 9));
                    when 306 =>
                        n_G <= z_l - signed(resize(c_a8, 21)) + signed(resize(s_ys, 21));
                        n_py <= z_par;
                        n_yq <= to_unsigned(1024, 13) + resize(s_ys(11 downto 1), 13);
                        n_row <= (others => '1');
                        n_first <= '1';
                    when others => null;
                end case;
            elsif g_start = '1' then
                l_T <= n_T(25 downto 10); l_R <= n_R(25 downto 10); l_py <= n_py;
                l_ay <= f_oabs(signed('0' & n_yq(12 downto 4)) - 256);
                l_row <= n_row; l_first <= n_first;
                e_st <= "00";
            elsif e_st = "00" then
                e_st <= "01";
                n_first <= '0';
                n_G <= n_G + signed(resize(s_ys, 21)); e_wa <= not n_G(20);
                n_T <= n_T + signed(resize(c_cys, 26)); n_R <= n_R + signed(resize(c_sys, 26));
                v_y := resize(n_yq, 14) + resize(s_ys, 14);
                n_yq <= v_y(12 downto 0); e_wb <= v_y(13); l_rev <= v_y(13);
            elsif e_st = "01" then
                e_st <= "11";
                if e_wa = '1' then
                    n_G <= n_G - signed(resize(c_a8, 21));
                    n_T <= n_T - signed(resize(c_CA, 26)); n_R <= n_R - signed(resize(c_SA, 26));
                    n_py <= not n_py;
                end if;
                if e_wb = '1' then n_row <= n_row + 1; end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [P] PIXEL PATH
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u, v_v : signed(15 downto 0);
        variable v_au, v_av : unsigned(14 downto 0);
        variable v_k : signed(6 downto 0);
        variable v_w : std_logic;
        variable v_thr : signed(15 downto 0);
        variable v_e : signed(17 downto 0);
        variable v_m4, v_m2, v_m1 : unsigned(17 downto 0);
        variable v_bx : unsigned(5 downto 0);
        variable v_by : unsigned(4 downto 0);
        variable v_s6 : signed(6 downto 0);
        variable v_cst : unsigned(4 downto 0);
        variable v_base : unsigned(10 downto 0);
        variable v_c6, v_c5 : unsigned(10 downto 0);
        variable v_ton96 : std_logic;
        variable v_g0, v_g1, v_g2, v_g3 : unsigned(22 downto 0);
        variable v_sum : unsigned(22 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: the beam accumulators.  b_F is the NEXT pixel's distance past the cell's far edge: when it is
            -- not negative the next pixel is in the next cell, and lx cos, lx sin step back by one cell.  b_xn is
            -- the next pixel's phase in its camera block (bit 7: it is in the next block).
            if g_start = '1' then
                b_F <= c_F0; b_P <= c_P0; b_Q <= c_Q0; b_px <= c_px0;
            elsif b_F(20) = '0' then
                b_F <= b_F + c_Fw; b_P <= b_P + c_Pw; b_Q <= b_Q + c_Qw; b_px <= not b_px;
            else
                b_F <= b_F + c_Fs; b_P <= b_P + signed(resize(c_cst, 26)); b_Q <= b_Q + signed(resize(c_sst, 26));
            end if;
            if g_start = '1' then
                b_xn <= c_xn0; b_col <= c_col0;
                b_xq <= resize(c_xn0 - resize(s_xs & '0', 8), 7);
            else
                b_xq <= b_xn(6 downto 0);
                b_xn <= resize(b_xn(6 downto 0), 8) + resize(s_xs & '0', 8);
                if b_xn(7) = '1' then b_col <= b_col + 1; end if;
            end if;
            -- 2: the square's own frame: u = lx cos + sg ly sin, v = ly cos - sg lx sin (Q4); the camera dot:
            -- the pixel's distance from its block's centre along x
            v_w := z_flip xor ((b_px xor l_py) and not s_pin);
            s2_u <= f_as(b_P(25 downto 10), l_R, v_w);
            s2_v <= f_as(l_T, b_Q(25 downto 10), not v_w);
            v_k := signed((not b_xq(6)) & b_xq(5 downto 0));                 -- phase - 64, quarter virtual px
            s2_ax <= f_oabs(v_k) & v_k(6) & v_k(6);                          -- |.| in Q4
            s2_par <= b_px xor l_py; s2_sgn <= v_w; s2_col <= b_col;
            -- 3: Pinwheel: past which side of its own square is the pixel?
            v_thr := signed(resize(c_hf, 16));
            s3_f <= "0000";
            if s_pin = '1' then
                if s2_u > v_thr then s3_f(0) <= '1'; end if;
                if (not s2_u) > v_thr then s3_f(1) <= '1'; end if;
                if s2_v > v_thr then s3_f(2) <= '1'; end if;
                if (not s2_v) > v_thr then s3_f(3) <= '1'; end if;
            end if;
            s3_u <= s2_u; s3_v <= s2_v; s3_par <= s2_par; s3_sgn <= s2_sgn;
            -- 3: the dot: max distance from the block centre; the block's grid address
            if resize(s2_ax, 9) > l_ay then s3_am <= resize(s2_ax, 10); else s3_am <= resize(l_ay, 10); end if;
            for i in 4 to 7 loop
                if i = 4 then s_am(4) <= s3_am; else s_am(i) <= s_am(i - 1); end if;
            end loop;
            if s2_col < c_bxlo then v_bx := c_bxlo;
            elsif s2_col > c_bxhi then v_bx := c_bxhi;
            else v_bx := s2_col; end if;
            if l_row(6) = '1' then v_by := (others => '0');
            elsif l_row(5) = '1' then v_by := (others => '1');
            else v_by := l_row(4 downto 0); end if;
            s3_ga <= (not s_bank) & v_by & v_bx;
            -- 4: the neighbour's offset (magnitude and sign)
            if s3_f(0) = '1' then
                s4_mu <= c_CA(23 downto 10); s4_bu <= '1'; s4_mv <= c_SA(23 downto 10); s4_bv <= s3_sgn;
            elsif s3_f(1) = '1' then
                s4_mu <= c_CA(23 downto 10); s4_bu <= '0'; s4_mv <= c_SA(23 downto 10); s4_bv <= not s3_sgn;
            elsif s3_f(2) = '1' then
                s4_mu <= c_SA(23 downto 10); s4_bu <= not s3_sgn; s4_mv <= c_CA(23 downto 10); s4_bv <= '1';
            elsif s3_f(3) = '1' then
                s4_mu <= c_SA(23 downto 10); s4_bu <= s3_sgn; s4_mv <= c_CA(23 downto 10); s4_bv <= '0';
            else
                s4_mu <= (others => '0'); s4_bu <= '0'; s4_mv <= (others => '0'); s4_bv <= '0';
            end if;
            if s3_f = "0000" then s4_nb <= '0'; else s4_nb <= '1'; end if;
            s4_u <= s3_u; s4_v <= s3_v; s4_par <= s3_par;
            -- 5: in the owning square's frame; the held level (the grid's own register is stage 4)
            s5_u <= f_as(s4_u, signed(resize(s4_mu, 16)), s4_bu);
            s5_v <= f_as(s4_v, signed(resize(s4_mv, 16)), s4_bv);
            s_parp(5) <= s4_par xor s4_nb;
            for i in 6 to 14 loop s_parp(i) <= s_parp(i - 1); end loop;
            s5_L <= gr_q;
            -- 6: max(|u|, |v|)
            v_au := f_oabs(s5_u); v_av := f_oabs(s5_v);
            if v_au > v_av then s6_mx <= v_au; else s6_mx <= v_av; end if;
            -- 7: the distance inside the square (Q4); the band coordinate = that + the stream offset (never
            -- negative where anything shows); the dot's half size (the table's own register is stage 6)
            s7_d <= c_h4 - signed(resize(s6_mx, 16));
            v_e := c_ho - signed(resize(s6_mx, 18));
            if v_e(17) = '1' then s7_e <= (others => '0'); else s7_e <= unsigned(v_e(16 downto 0)); end if;
            s7_hm <= tb_q;
            -- 8 .. 10: e mod one period (4, 2, 1 periods); the last step also takes off the half period, so
            -- q is the distance from the stripe's centre
            v_m4 := resize(c_Pd & "00", 18); v_m2 := resize(c_Pd & '0', 18); v_m1 := resize(c_Pd, 18);
            if resize(s7_e, 18) >= v_m4 then s8_e <= resize(resize(s7_e, 18) - v_m4, 17); else s8_e <= s7_e; end if;
            if resize(s8_e, 18) >= v_m2 then s9_e <= resize(resize(s8_e, 18) - v_m2, 17); else s9_e <= s8_e; end if;
            if resize(s9_e, 18) >= v_m1 then s10_q <= signed(resize(s9_e, 18)) - signed(resize(c_Pd15, 18));
            else s10_q <= signed(resize(s9_e, 18)) - signed(resize(c_Pd(14 downto 1), 18)); end if;
            -- 8: the square's edge distance, saturated; the Fine outline's inner edge; the dot's edge distance
            s8_ds <= f_sat(s7_d);
            s8_x4 <= signed(resize(c_lw, 16)) - s7_d;
            s8_dm <= s7_hm - signed(resize(s_am(7), 16));
            -- 9: the square's coverage
            s_covd(9) <= f_cv(s8_ds, s_xs);
            for i in 10 to 14 loop s_covd(i) <= s_covd(i - 1); end loop;
            s9_x4s <= f_sat(s8_x4);
            s9_dms <= f_sat(s8_dm);
            -- 10: the outline's inner edge and the dot's coverage
            s10_ce <= f_cv(s9_x4s, s_xs);
            s_covm(10) <= f_cv(s9_dms, s_xs);
            for i in 11 to 14 loop s_covm(i) <= s_covm(i - 1); end loop;
            -- 11: the outline: coverage of 0 <= d < line width; the distance inside the stripe
            v_s6 := signed(resize(s_covd(10), 7)) + signed(resize(s10_ce, 7)) - 16;
            if v_s6(6) = '1' then s_edge(11) <= (others => '0'); else s_edge(11) <= unsigned(v_s6(4 downto 0)); end if;
            for i in 12 to 13 loop s_edge(i) <= s_edge(i - 1); end loop;
            s11_x <= signed(resize(c_hw, 18)) - signed(resize(f_oabs(s10_q), 18));
            -- 12, 13: the stripe's coverage
            s12_xs <= f_sat(s11_x);
            s13_ln <= f_cv(s12_xs, s_xs);
            -- 14: the band / outline coverage before the camera
            if s_fine = '1' then
                if c_n1 = '0' and s13_ln > s_edge(13) then s14_cst <= s13_ln; else s14_cst <= s_edge(13); end if;
            elsif c_n1 = '1' then s14_cst <= to_unsigned(16, 5);
            else s14_cst <= s13_ln; end if;
            -- 15: tone (x 16) and coverage.  Bold: the dots mask the square.  Fine: the dots are added to the
            -- outlines.
            v_ton96 := s_ink or not s_parp(14);
            v_cst := s14_cst;
            if s_fine = '1' then
                if s_covm(14) > s14_cst then v_cst := s_covm(14); end if;
                s15_cov <= s_covd(14);
                v_base := (others => '0');
                v_c6 := resize(v_cst & "000000", 11);
                if v_ton96 = '1' then v_c5 := resize(v_cst & "00000", 11); else v_c5 := (others => '0'); end if;
            else
                if s_covm(14) < s_covd(14) then s15_cov <= s_covm(14); else s15_cov <= s_covd(14); end if;
                if s_ink = '1' then
                    v_base := (others => '0');
                    v_c6 := resize(v_cst & "000000", 11); v_c5 := resize(v_cst & "00000", 11);
                else
                    if v_ton96 = '1' then v_base := to_unsigned(1024, 11); else v_base := to_unsigned(512, 11); end if;
                    v_c6 := (others => '0'); v_c5 := resize(v_cst & "00000", 11);
                end if;
            end if;
            s15_m <= v_base + v_c6 + v_c5;
            -- 16: * 73
            s16_m73 <= resize(s15_m & "000000", 18) + resize(s15_m & "000", 18) + resize(s15_m, 18);
            s16_cov <= s15_cov;
            -- 17: * coverage (0 .. 16) as gated shifts
            v_g0 := (others => '0'); v_g1 := (others => '0'); v_g2 := (others => '0'); v_g3 := (others => '0');
            if s16_cov(0) = '1' then v_g0 := resize(s16_m73, 23); end if;
            if s16_cov(1) = '1' then v_g1 := resize(s16_m73 & '0', 23); end if;
            if s16_cov(2) = '1' then v_g2 := resize(s16_m73 & "00", 23); end if;
            if s16_cov(3) = '1' then v_g3 := resize(s16_m73 & "000", 23); end if;
            if s16_cov(4) = '1' then v_g3 := resize(s16_m73 & "0000", 23); end if;
            s17_pa <= v_g0 + v_g1; s17_pb <= v_g2 + v_g3;
            -- 18: (tone * coverage * 73) >> 11; the luma is 64 + that (white 940), added in the output's first stage
            v_sum := s17_pa + s17_pb;
            s18_s <= v_sum(20 downto 11);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice, doubled) -> box of 32 samples,
    -- edge-replicated and reloaded at every line start -> at each 32-virtual-px
    -- block's last pixel a vertical one-pole per block column (reloaded on the
    -- field's first line) -> on each block row's last line its top four bits go
    -- into the grid bank being written.  The pixel path reads the other bank.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_u : signed(12 downto 0);
        variable v_q : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart from this
            -- field's drift in the zone (riley's lattice carried the last field's x drift on line 0)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice at twice the level
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then
                if s_osum(7) = '1' then s_l8 <= (others => '1'); else s_l8 <= s_osum(6 downto 0) & '0'; end if;
            elsif v_l > 16 then s_l8 <= v_l - 16;
            else s_l8 <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;

            -- box: sum of the last 32 samples; the samples before the line start count as the first one
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(225, 8); end if;
            if s_bcnt(5) = '0' then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_l8;
                s_bsum <= s_l8 & "00000";
                s_bcnt <= to_unsigned(1, 6);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_l8 - v_tap;
                if s_bcnt(5) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;

            -- block columns: k_n is the next sample's phase in its block (bit 7: it starts a new block, so
            -- this sample is the block's last)
            k_ev <= '0';
            if s_sv = '0' then
                k_n <= c_kn0; k_col <= c_col0;
            elsif k_n(7) = '1' then
                k_n <= resize(k_n(6 downto 0), 8) + resize(s_xs & '0', 8);
                k_ev <= '1'; k_evc <= k_col; k_col <= k_col + 1;
            else
                k_n <= k_n + resize(s_xs & '0', 8);
            end if;

            -- the one-pole at the block's last pixel: q += (box - q) >> sh, Q8.4 (first line: q = box)
            s_eb <= s_bsum(12 downto 5); s_eq <= st_q(11 downto 0);
            e_v <= k_ev; e_col <= k_evc;
            v_d := signed(resize(s_eb & "0000", 13)) - signed(resize(s_eq, 13));
            if s_ish3 = '1' then v_u := shift_right(v_d, 3); else v_u := shift_right(v_d, 2); end if;
            if l_first = '1' then v_q := s_eb & "0000";
            else v_q := unsigned(signed(s_eq) + v_u(11 downto 0)); end if;
            st_wd <= resize(v_q, 16); st_wa <= e_col; st_we <= e_v;
            gr_wd <= v_q(11 downto 8);
            gr_wa <= s_bank & l_row(4 downto 0) & e_col;
            if e_v = '1' and l_rev = '1' and l_row(6 downto 5) = "00" then gr_we <= '1'; else gr_we <= '0'; end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: black outside pixels 0 .. W + 1 (19), luma
    -- [1 2 1] / 4 (22, 23), 13/16 toward black (24), blanking gate (25).
    -- Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(16) = '1' or s_gav_sr(18) = '1' then s19_y <= to_unsigned(64, 10) + s18_s; else s19_y <= to_unsigned(64, 10); end if;
            s20_y <= s19_y; s21_y <= s20_y;
            s22_ya <= resize(s19_y, 12) + resize(s21_y, 12) + 2;
            s22_yb <= s20_y;
            v_y := s22_ya + shift_left(resize(s22_yb, 12), 1);
            s23_y <= v_y(11 downto 2);
            if s23_y > 64 then v_dl := s23_y - 64; else v_dl := (others => '0'); end if;
            s24_y <= s23_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then s25_y <= s24_y; else s25_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s25_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture hinge;
