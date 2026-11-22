-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Hatch v0.1.0 -- Straightedge.  Sol LeWitt's "Lines in Four Directions" and the wall drawings that lay out
-- every combination of vertical, horizontal and the two diagonal lines; the engraver's cross-hatch tone scale.
-- The screen is ruled into square panels; each panel is a window on four continuous families of fine parallel
-- lines (V, H and the diagonals x + y, x - y through the same crossings, so all four make one eight-pointed
-- lattice) and switches on one of LeWitt's 15 combinations.  Tone comes only from how many families are on.
-- Luma only (U = V = 512).
--
--   family f:  A_f = the line phase in pixel widths (a DDA per family, modulo its spacing P or P / sqrt 2)
--   coverage   clamp(T - |distance to the line centre| + 1/2) per family, T per cell; union = the maximum
--   panel      (i, j) about the centre (X, Y); Grid: combination (i + 4 j) mod 15, Frames: max(|i|, |j|) mod 15
--   layers     P12 thins every panel to one direction (0 %) or fills it to all four (100 %), panel by panel
--   camera     the luma of each quarter-panel cell (the previous field, one 32-px block at the cell centre) pulls
--              the cell's layer count toward a luma ladder (0 .. 4 families, then 4 families thickening)
--
--   K1  X        pan of the panel grid (the lines stay put)                                  (glided)
--   K2  Y                                                                                    (glided)
--   K3  Scale    line pitch 16 .. 128 virtual px, exponential                                (glided)
--   K4  Panel    panel size 122 .. 1953 virtual px, exponential (one panel fills the screen) (glided)
--   K5  Drift    the lattice slides rigidly (V phase rate 1, H -1/2, the diagonals 1/2 and 3/2): Hypnos SPEED,
--                still within +-24 counts of the centre, 2^-12 .. 2^-4 period per field
--   K6  Camera   how strongly the held luma sets each cell's layers                          (glided)
--   S7  Glide    Off = the knob value of this field, On = 1/16 of the gap per field (speed: 1/8)
--   S8  Form     Grid / Frames
--   S9  Tone     Grey (white lines on black) / Ink (black lines on white)
--   S10 Weight   Bold (a quarter of each family's spacing) / Fine (2 px hairlines)
--   S11 Source   Video / Osc
--   P12 LAYERS   0 % = one direction per panel, 50 % = LeWitt's set, 100 % = all four everywhere   (glided)
--
-- Sections (the integer model .straightedge_work/rb_hatch/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide, switches
--   [C] FIELD SEQUENCER                  pitch, half widths, cell size, grid origin, drift, seeds, line steps
--   [D] SOURCE FIELD AND CAMERA GRID     luma or the Osc lattice, box, vertical one-pole, 32-px block samples
--   [E] CELLS                            cell / panel DDAs, combination, camera read, layer level, mask, T
--   [F] FAMILIES                         three pixel DDAs + three line accumulators, coverage, union, luma
--   [G] OUTPUT CONDITIONING              luma [1 2 1]/4, luma 13/16 toward black, blanking gate
--
-- Timing: g_start is sampled at edge E0; "period k" is the clock after edge Ek.  The cell DDA holds pixel i in
-- period i, the family DDAs in period 9 + i; the luma (s_y23) is pixel i in period 18 + i.  One latency (24)
-- in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.hatch_pkg.all;
use work.all;

architecture hatch of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 200;
    constant C_DEF_K4  : integer := 211;
    constant C_DEF_K5  : integer := 680;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;

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
    -- the Osc lattice's y step: (y step * 121) >> 8
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
    function f_oxs(xs : unsigned(2 downto 0)) return unsigned is
    begin
        if xs = 4 then return to_unsigned(204, 9); elsif xs = 3 then return to_unsigned(153, 9); else return to_unsigned(102, 9); end if;
    end function;
    -- the camera writer: the first block centre (plus the box lag) and the first block column of a line
    function f_qx0(xs : unsigned(2 downto 0)) return signed is
    begin
        if xs = 4 then return to_signed(-24, 8); elsif xs = 3 then return to_signed(-50, 8); else return to_signed(-60, 8); end if;
    end function;
    function f_bc0(xs : unsigned(2 downto 0)) return unsigned is
    begin
        if xs = 4 then return to_unsigned(7, 6); else return to_unsigned(0, 6); end if;
    end function;
    -- the pitch's scale: 16 virtual px in pixel widths, Q14, per 2^15 of mantissa, << 12
    function f_sc(xs : unsigned(2 downto 0)) return unsigned is
    begin
        if xs = 4 then return to_unsigned(16384, 16); elsif xs = 3 then return to_unsigned(21845, 16); else return to_unsigned(32768, 16); end if;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    function f_bitrev(v : unsigned(3 downto 0)) return unsigned is
    begin
        return v(0) & v(1) & v(2) & v(3);
    end function;
    function f_bayer(j, i : unsigned(1 downto 0)) return unsigned is
        type t_b is array (0 to 15) of integer range 0 to 15;
        constant C_B : t_b := (0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5);
    begin
        return to_unsigned(C_B(to_integer(j & i)), 4);
    end function;
    -- luma of the union coverage m (0 .. 16): (876 m + 8) >> 4
    function f_yv(m : unsigned(4 downto 0)) return unsigned is
        type t_y is array (0 to 31) of integer range 0 to 1023;
        constant C_Y : t_y := (0, 55, 110, 164, 219, 274, 329, 383, 438, 493, 548, 602, 657, 712, 767, 821, 876,
                               876, 876, 876, 876, 876, 876, 876, 876, 876, 876, 876, 876, 876, 876, 876);
    begin
        return to_unsigned(C_Y(to_integer(m)), 10);
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
    signal s_st       : std_logic_vector(0 to 8) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: glide accumulators (Q6) and their "moving" flags
    signal g_x : unsigned(15 downto 0) := to_unsigned(C_DEF_K1 * 64, 16);
    signal g_y : unsigned(15 downto 0) := to_unsigned(C_DEF_K2 * 64, 16);
    signal g_p : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);
    signal g_k : unsigned(15 downto 0) := to_unsigned(C_DEF_K4 * 64, 16);
    signal g_c : unsigned(15 downto 0) := to_unsigned(C_DEF_K6 * 64, 16);
    signal g_s : unsigned(15 downto 0) := to_unsigned(C_DEF_P12 * 64, 16);
    signal s_vgs : signed(16 downto 0) := to_signed((C_DEF_K5 - 512) * 64, 17);   -- Drift speed glide, Q6
    signal s_gm, s_gd : signed(16 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_gmv, s_ge : std_logic := '0';
    signal z_mv : std_logic_vector(7 downto 0) := (others => '0');
    signal z_gp, z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_frames, s_ink, s_fine, s_src : std_logic := '0';

    -- [C] ROM
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');

    -- [C] sequencer, its serial multiplier (right-shifting product) and its loop
    signal q_go, q_run, q_kick : std_logic := '0';
    signal q_st : unsigned(6 downto 0) := (others => '0');
    signal m_a, m_hi : unsigned(17 downto 0) := (others => '0');
    signal m_b, m_lo : unsigned(15 downto 0) := (others => '0');
    signal m_cnt : unsigned(4 downto 0) := (others => '0');
    signal l_a : signed(21 downto 0) := (others => '0');
    signal l_m : unsigned(17 downto 0) := (others => '0');
    signal l_q : signed(7 downto 0) := (others => '0');
    signal z_xq : unsigned(17 downto 0) := (others => '0');
    signal z_oct : unsigned(1 downto 0) := (others => '0');
    signal z_fr : unsigned(7 downto 0) := (others => '0');
    signal z_e255 : std_logic := '0';
    signal z_e0, z_m : unsigned(16 downto 0) := (others => '0');
    signal z_ed : unsigned(8 downto 0) := (others => '0');
    signal z_v : signed(20 downto 0) := (others => '0');
    signal z_pk, z_dp, z_s : unsigned(17 downto 0) := (others => '0');
    signal z_f : unsigned(1 downto 0) := (others => '0');
    signal z_vn, z_vz : std_logic := '0';
    signal z_av, z_u : unsigned(9 downto 0) := (others => '0');
    signal z_ls : unsigned(8 downto 0) := (others => '0');
    signal z_spd : unsigned(15 downto 0) := (others => '0');
    signal z_scn : unsigned(2 downto 0) := (others => '0');
    signal z_rq : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal z_lsd : unsigned(11 downto 0) := (others => '0');
    signal d_v, d_h : unsigned(19 downto 0) := (others => '0');          -- the drift phases (one period = 2^20)
    signal z_d : unsigned(19 downto 0) := (others => '0');
    signal z_ta, z_td, z_dtv, z_dtd : unsigned(10 downto 0) := (others => '0');
    signal q_sd : signed(19 downto 0) := (others => '0');                -- a seed for the line accumulators
    signal q_ld : std_logic_vector(3 downto 0) := (others => '0');
    -- per-field results
    signal c_P, c_Pd : unsigned(17 downto 0) := to_unsigned(24576, 18);  -- the spacings, Q10 pixel widths
    signal c_incwv, c_incwd : signed(19 downto 0) := (others => '0');
    signal c_lsd1, c_lsd2, c_lswh, c_lswd1, c_lswd2 : signed(19 downto 0) := (others => '0');
    type t_t5 is array (0 to 4) of unsigned(10 downto 0);
    signal c_tv, c_td : t_t5 := (others => (others => '0'));
    signal c_C : unsigned(13 downto 0) := to_unsigned(1729, 14);
    signal c_xsc : signed(15 downto 0) := (others => '0');
    signal c_rx : unsigned(13 downto 0) := (others => '0');
    signal c_c40, c_r40 : unsigned(1 downto 0) := (others => '0');
    signal c_i0 : signed(6 downto 0) := (others => '0');
    signal c_j0 : signed(5 downto 0) := (others => '0');
    signal c_i150, c_j40 : unsigned(3 downto 0) := (others => '0');
    signal c_pcx0 : signed(17 downto 0) := (others => '0');
    signal c_py0 : unsigned(16 downto 0) := (others => '0');
    signal c_pcy0 : signed(19 downto 0) := (others => '0');
    signal c_u16 : unsigned(4 downto 0) := (others => '0');
    signal c_up : std_logic := '1';
    signal c_s16 : signed(7 downto 0) := (others => '0');
    signal c_k6 : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal c_sd4 : std_logic := '0';                                   -- the SD rasters (x step 4)
    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_cv, s_tv, s_tvd, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_v1, s_v2, s_v2d, s_v3, s_v4, s_v5, s_v6, s_v7, s_v7d : std_logic := '0';
    signal s_ld, s_fl, s_seen, s_efl, s_ffl : std_logic := '0';
    signal s_col, s_c2, s_bcol, s_ecol, s_fcol : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_f : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal c_b32 : std_logic := '1';                                  -- 32-px box on the 1920-wide rasters, else 16
    signal c_iir : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal s_df   : signed(8 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal s_ev, s_fv : std_logic := '0';
    signal s_eb, s_eq, s_fq : unsigned(15 downto 0) := (others => '0');
    signal s_dd : signed(16 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';

    -- [D] camera grid: two banks of 64 x 64 blocks of 32 virtual px, 4 bits
    type t_cm is array (0 to 8191) of unsigned(3 downto 0);
    signal cm_ram : t_cm := (others => (others => '0'));
    attribute no_rw_check of cm_ram : signal is true;
    signal cm_ra, cm_wa : unsigned(12 downto 0) := (others => '0');
    signal cm_q, cm_f, cm_wd, s_lw : unsigned(3 downto 0) := (others => '0');
    signal cm_we, s_wb : std_logic := '0';
    signal s_qx : signed(7 downto 0) := (others => '0');
    signal s_bc : unsigned(5 downto 0) := (others => '0');
    signal s_qy : unsigned(13 downto 0) := (others => '0');
    signal s_row : unsigned(5 downto 0) := (others => '0');

    -- [E] cells
    signal e_px  : unsigned(13 downto 0) := (others => '0');
    signal e_c4  : unsigned(1 downto 0) := (others => '0');
    signal e_ip  : signed(6 downto 0) := (others => '0');
    signal e_i15 : unsigned(3 downto 0) := (others => '0');
    signal e_pcx : signed(17 downto 0) := (others => '0');
    signal e_py  : unsigned(16 downto 0) := (others => '0');
    signal e_r4  : unsigned(1 downto 0) := (others => '0');
    signal e_jp  : signed(5 downto 0) := (others => '0');
    signal e_j4  : unsigned(3 downto 0) := (others => '0');
    signal e_pcy : signed(19 downto 0) := (others => '0');
    signal e_row : unsigned(5 downto 0) := (others => '0');
    signal e_aj  : unsigned(5 downto 0) := (others => '0');
    signal e1_col : unsigned(5 downto 0) := (others => '0');
    signal e1_cg : unsigned(4 downto 0) := (others => '0');
    signal e1_ai : unsigned(6 downto 0) := (others => '0');
    signal e1_phg, e2_phg, e2_cg : unsigned(3 downto 0) := (others => '0');
    signal e2_ring : unsigned(6 downto 0) := (others => '0');
    signal e3_ci, e4_ci, e5_ci, e6_ci, e7_ci, e8_ci, e9_ci : unsigned(3 downto 0) := (others => '0');
    signal e3_phi, e4_phi, e5_phi, e6_phi, e7_phi : unsigned(3 downto 0) := (others => '0');
    signal e4_b1 : unsigned(1 downto 0) := (others => '0');
    signal e4_a, e5_a, e6_a : unsigned(5 downto 0) := (others => '0');
    signal e4_t : unsigned(3 downto 0) := (others => '0');
    signal e5_nc, e6_nc, e7_nc : signed(8 downto 0) := (others => '0');
    signal e5_dp : signed(8 downto 0) := (others => '0');
    signal e6_df : signed(8 downto 0) := (others => '0');
    signal e7_pr : signed(14 downto 0) := (others => '0');
    signal e7_port, e8_port : std_logic := '0';
    signal e8_lv : signed(9 downto 0) := (others => '0');
    signal e9_nf : unsigned(2 downto 0) := (others => '0');
    signal e9_w : unsigned(2 downto 0) := (others => '0');
    signal e10_tv, e10_td : unsigned(10 downto 0) := (others => '0');
    signal e10_mk, e11_mk : std_logic_vector(3 downto 0) := (others => '0');

    -- [F] families
    signal b_run, b_end : std_logic := '0';
    signal b_cnt : unsigned(11 downto 0) := (others => '0');
    signal la_v, la_h, la_d1, la_d2 : signed(19 downto 0) := (others => '0');
    signal a_v, a_d1, a_d2 : signed(19 downto 0) := (others => '0');
    type t_s14x4 is array (0 to 3) of signed(13 downto 0);
    signal x1_u, x2_e : t_s14x4 := (others => (others => '0'));
    type t_u5x4 is array (0 to 3) of unsigned(4 downto 0);
    signal x3_c : t_u5x4 := (others => (others => '0'));
    signal x4_a, x4_b, x5_m : unsigned(4 downto 0) := (others => '0');
    signal x6_y, x7_y, x8_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- [G] output conditioning
    signal s_y23, s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y26, s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_HT_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
        end if;
    end process p_rom;

    -- box delay: read 30 (or 14) samples behind the write (+ two registers = 32 or 16)
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_v2 = '1' then bx_ram(to_integer(bx_wp)) <= s_tv; end if;
        end if;
    end process p_bxram;

    -- the vertical one-pole's line store (half horizontal resolution): read ahead of its own write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_c2(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- camera grid: the scan writes one bank, the cell path reads the other (the bank bit toggles per field)
    cm_ra <= (not s_wb) & e_row & e1_col;
    p_cmram : process(clk)
    begin
        if rising_edge(clk) then
            cm_q <= cm_ram(to_integer(cm_ra));
            if cm_we = '1' then cm_ram(to_integer(cm_wa)) <= cm_wd; end if;
        end if;
    end process p_cmram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_st, v_tg : signed(16 downto 0);
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
                s_glide  <= registers_in(6)(0);
                s_frames <= registers_in(6)(1);
                s_ink    <= registers_in(6)(2);
                s_fine   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                s_il     <= s_ilr;
                s_wb     <= not s_wb;                 -- the camera grid's write bank
                if s_xsr = 2 then c_b32 <= '1'; else c_b32 <= '0'; end if;
                -- the one-pole's shift: about half a block of lines in every mode
                if s_ys <= 256 then c_iir <= to_unsigned(4, 3);
                elsif s_ys <= 640 then c_iir <= to_unsigned(3, 3);
                else c_iir <= to_unsigned(2, 3); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

            -- [B] shared hold + glide, five clocks per channel (X, Y, Scale, Panel, Camera, Layers, then the Drift
            -- speed):
            --   hold: a channel starts to move only when its knob is two counts or more from where the channel
            --         rests (pot jitter), or sits at an end of its travel; once moving it lands exactly
            --   glide On: 1/16 of the gap per field, never slower than one count (the speed: 1/8 of the gap)
            --   glide Off: the knob value of THIS field start (no easing, no lag)
            q_go <= '0';
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                z_gp <= z_gp + 1;
                case to_integer(z_gp) is
                    when 0 =>
                        s_gmv <= z_mv(to_integer(z_gc));
                        case to_integer(z_gc) is
                            when 0      => s_gm <= signed('0' & g_x); s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= signed('0' & g_y); s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= signed('0' & g_p); s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= signed('0' & g_k); s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= signed('0' & g_c); s_gr <= unsigned(registers_in(5));
                            when 5      => s_gm <= signed('0' & g_s); s_gr <= unsigned(registers_in(7));
                            when others => s_gm <= s_vgs;             s_gr <= unsigned(registers_in(4));
                        end case;
                    when 1 =>                      -- the gap to the knob (the speed is the knob about its centre)
                        v_tg := (others => '0');
                        v_tg(15 downto 6) := signed(s_gr);
                        if z_gc = 6 then v_tg(15) := not v_tg(15); v_tg(16) := not s_gr(9); end if;
                        s_gd <= v_tg - s_gm;
                        if s_gr = 0 or s_gr = 1023 then s_ge <= '1'; else s_ge <= '0'; end if;
                    when 2 =>
                        if s_gd >= 128 or s_gd <= -128 or (s_ge = '1' and s_gd /= 0) then s_gmv <= '1'; end if;
                    when 3 =>
                        if s_gmv = '0' then
                            s_gd <= (others => '0');
                        elsif s_glide = '0' or (s_gd <= 64 and s_gd >= -64) then
                            s_gmv <= '0';                                  -- the whole gap: it lands
                        elsif z_gc = 6 then
                            s_gd <= shift_right(s_gd, 3);
                        elsif s_gd < 1024 and s_gd > -1024 then
                            if s_gd(16) = '1' then s_gd <= to_signed(-64, 17); else s_gd <= to_signed(64, 17); end if;
                        else
                            s_gd <= shift_right(s_gd, 4);
                        end if;
                    when others =>
                        v_st := s_gm + s_gd;
                        z_mv(to_integer(z_gc)) <= s_gmv;
                        case to_integer(z_gc) is
                            when 0      => g_x <= unsigned(v_st(15 downto 0));
                            when 1      => g_y <= unsigned(v_st(15 downto 0));
                            when 2      => g_p <= unsigned(v_st(15 downto 0));
                            when 3      => g_k <= unsigned(v_st(15 downto 0));
                            when 4      => g_c <= unsigned(v_st(15 downto 0));
                            when 5      => g_s <= unsigned(v_st(15 downto 0));
                            when others => s_vgs <= v_st;
                        end case;
                        z_gp <= "000";
                        if z_gc = 6 then z_grun <= '0'; q_go <= '1'; end if;
                        z_gc <= z_gc + 1;
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
    -- [C] THE FIELD SEQUENCER: one small operation per zone clock (the zone is quasi-static but timed at the
    -- pixel clock), one serial 16-step multiplier, one add / subtract loop (a += m while a < 0, a -= m while
    -- a >= m).  It starts at every zone (after the glide) from state 0 and ends with the kick.
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_ld  : signed(21 downto 0);
        variable v_v   : signed(10 downto 0);
        variable v_n11 : signed(10 downto 0);
        variable v_17  : signed(16 downto 0);
        variable v_18  : signed(17 downto 0);
        variable v_t   : unsigned(18 downto 0);
        variable v_sa, v_sb, v_sr : signed(19 downto 0);
        variable v_ta, v_tdd : unsigned(10 downto 0);
        variable v_dq  : signed(16 downto 0);
        variable v_u   : unsigned(15 downto 0);
        variable v_k6  : unsigned(16 downto 0);
        variable v_pr  : unsigned(33 downto 0);
        variable v_s   : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            q_kick <= '0';
            q_ld <= (others => '0');
            -- the multiplier: m_cnt <= 16 with fresh m_a, m_b and a cleared product starts it; the product
            -- m_hi & m_lo is complete when m_cnt = 0
            if m_cnt /= 0 then
                if m_b(0) = '1' then v_t := resize(m_hi, 19) + resize(m_a, 19); else v_t := resize(m_hi, 19); end if;
                m_hi <= v_t(18 downto 1);
                m_lo <= v_t(0) & m_lo(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
                m_cnt <= m_cnt - 1;
            end if;
            v_pr := m_hi & m_lo;
            -- the loop's step; the shared adders of the half-width tables; the shared subtractor
            v_ld := l_a - signed(resize(l_m, 22));
            v_ta := z_ta + z_dtv;
            v_tdd := z_td + z_dtd;
            case to_integer(q_st) is
                when 13     => v_sa := to_signed(1024, 20);       v_sb := signed(resize(c_P, 20));
                when 14     => v_sa := to_signed(724, 20);        v_sb := signed(resize(c_Pd, 20));
                when 63     => v_sa := signed(resize(z_rq, 20));  v_sb := signed(resize(c_P, 20));
                when 64     => v_sa := signed(resize(z_lsd, 20)); v_sb := signed(resize(c_Pd, 20));
                when 65     => v_sa := signed(resize(c_Pd, 20));  v_sb := signed(resize(z_lsd, 20));
                when others => v_sa := (others => '0');           v_sb := signed(resize(z_lsd, 20));
            end case;
            v_sr := v_sa - v_sb;
            if q_go = '1' then
                q_run <= '1'; q_st <= (others => '0');
            elsif q_run = '1' then
                q_st <= q_st + 1;
                case to_integer(q_st) is
                    -- pitch: 16 * 2^(3 k / 1024) virtual px, Q10 pixel widths
                    when 0 => z_xq <= resize(g_p, 18) + resize(g_p & '0', 18);
                    when 1 =>
                        r_ex_a <= z_xq(15 downto 8); z_oct <= z_xq(17 downto 16); z_fr <= z_xq(7 downto 0);
                        if z_xq(15 downto 8) = 255 then z_e255 <= '1'; else z_e255 <= '0'; end if;
                    when 2 => r_ex_a <= r_ex_a + 1;
                    when 3 => null;
                    when 4 => z_e0 <= resize(r_exf, 17);
                    when 5 =>
                        if z_e255 = '1' then v_18 := to_signed(65536, 18); else v_18 := signed(resize(r_exf, 18)); end if;
                        z_ed <= unsigned(resize(v_18 - signed(resize(z_e0, 18)), 9));
                    when 6 =>
                        m_a <= resize(z_ed, 18); m_b <= resize(z_fr, 16); m_hi <= (others => '0'); m_lo <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    when 7 =>
                        if m_cnt /= 0 then q_st <= q_st; else z_m <= z_e0 + resize(v_pr(16 downto 8), 17); end if;
                    when 8 =>
                        m_a <= resize(z_m, 18); m_b <= f_sc(s_xs); m_hi <= (others => '0'); m_lo <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    when 9 =>
                        if m_cnt /= 0 then q_st <= q_st;
                        else
                            case to_integer(z_oct) is
                                when 0      => c_P <= v_pr(33 downto 16);
                                when 1      => c_P <= v_pr(32 downto 16) & '0';
                                when others => c_P <= v_pr(31 downto 16) & "00";
                            end case;
                        end if;
                    when 10 =>
                        m_a <= c_P; m_b <= to_unsigned(46341, 16); m_hi <= (others => '0'); m_lo <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    when 11 =>
                        if m_cnt /= 0 then q_st <= q_st; else c_Pd <= v_pr(33 downto 16); end if;
                    when 12 =>
                        -- line half widths (Q4 pixels) per weight step: Bold P / 8 + w P / 32, Fine 1 px + w (P / 16 - 1/4)
                        if s_fine = '1' then
                            c_tv(0) <= to_unsigned(16, 11); c_td(0) <= to_unsigned(16, 11);
                            z_ta <= to_unsigned(16, 11); z_td <= to_unsigned(16, 11);
                            z_dtv <= resize(c_P(17 downto 10), 11) - 4;
                            z_dtd <= resize(c_Pd(17 downto 10), 11) - 4;
                        else
                            c_tv(0) <= resize(c_P(17 downto 9), 11); c_td(0) <= resize(c_Pd(17 downto 9), 11);
                            z_ta <= resize(c_P(17 downto 9), 11); z_td <= resize(c_Pd(17 downto 9), 11);
                            z_dtv <= resize(c_P(17 downto 11), 11);
                            z_dtd <= resize(c_Pd(17 downto 11), 11);
                        end if;
                    when 13 => c_tv(1) <= v_ta; c_td(1) <= v_tdd; z_ta <= v_ta; z_td <= v_tdd; c_incwv <= v_sr;
                    when 14 => c_tv(2) <= v_ta; c_td(2) <= v_tdd; z_ta <= v_ta; z_td <= v_tdd; c_incwd <= v_sr;
                    when 15 => c_tv(3) <= v_ta; c_td(3) <= v_tdd; z_ta <= v_ta; z_td <= v_tdd;
                    when 16 => c_tv(4) <= v_ta; c_td(4) <= v_tdd;
                    -- cell = panel / 4: 30.5 * 2^(k / 256) virtual px, h units Q4
                    when 17 =>
                        r_ex_a <= g_k(13 downto 6); z_oct <= g_k(15 downto 14); z_fr <= "00" & g_k(5 downto 0);
                        if g_k(13 downto 6) = 255 then z_e255 <= '1'; else z_e255 <= '0'; end if;
                    when 18 => r_ex_a <= r_ex_a + 1;
                    when 19 => null;
                    when 20 => z_e0 <= resize(r_exf, 17);
                    when 21 =>
                        if z_e255 = '1' then v_18 := to_signed(65536, 18); else v_18 := signed(resize(r_exf, 18)); end if;
                        z_ed <= unsigned(resize(v_18 - signed(resize(z_e0, 18)), 9));
                    when 22 =>
                        m_a <= resize(z_ed, 18); m_b <= resize(z_fr, 16); m_hi <= (others => '0'); m_lo <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    when 23 =>
                        if m_cnt /= 0 then q_st <= q_st; else z_m <= z_e0 + resize(v_pr(16 downto 6), 17); end if;
                    when 24 =>
                        m_a <= resize(z_m, 18); m_b <= to_unsigned(977, 16); m_hi <= (others => '0'); m_lo <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    when 25 =>
                        if m_cnt /= 0 then q_st <= q_st;
                        else
                            case to_integer(z_oct) is
                                when 0      => c_C <= resize(v_pr(26 downto 15), 14);
                                when 1      => c_C <= resize(v_pr(26 downto 15) & '0', 14);
                                when 2      => c_C <= resize(v_pr(26 downto 15) & "00", 14);
                                when others => c_C <= v_pr(25 downto 15) & "000";
                            end case;
                        end if;
                    -- the grid origin: panel (0, 0) is centred on (X, Y)
                    when 26 =>
                        if s_xs = 4 then c_sd4 <= '1'; else c_sd4 <= '0'; end if;
                        v_17 := signed('0' & g_x) - to_signed(32768, 17);
                        z_v <= shift_left(resize(v_17, 21), 4) - resize(v_17, 21);         -- 15 (X - 32768)
                    when 27 =>
                        c_xsc <= signed(resize(s_xs & "0000", 16)) - signed(resize(c_C, 16));
                        -- a = x0 - cx + 2 C,  cx = 30720 + 15 v / 16  (x0 = 7680 h on SD)
                        if c_sd4 = '1' then v_18 := to_signed(7680 - 30720, 18); else v_18 := to_signed(-30720, 18); end if;
                        l_a <= resize(v_18, 22) - resize(shift_right(z_v, 4), 22) + signed(resize(c_C & '0', 22));
                        l_m <= resize(c_C, 18); l_q <= (others => '0');
                    when 28 | 33 | 36 | 38 | 53 =>       -- the loop
                        q_st <= q_st;
                        if l_a(21) = '1' then
                            l_a <= l_a + signed(resize(l_m, 22)); l_q <= l_q - 1;
                        elsif v_ld(21) = '0' then
                            l_a <= v_ld; l_q <= l_q + 1;
                        else
                            q_st <= q_st + 1;
                        end if;
                    when 29 =>
                        c_rx <= unsigned(l_a(13 downto 0));
                        c_c40 <= unsigned(l_q(1 downto 0));
                        c_i0 <= resize(shift_right(l_q, 2), 7);
                        if c_sd4 = '1' then v_18 := to_signed(7680, 18); else v_18 := (others => '0'); end if;
                        c_pcx0 <= v_18 - resize(l_a(17 downto 0), 18) + signed(resize(c_C(13 downto 1), 18));
                    when 30 =>
                        v_17 := signed('0' & g_y) - to_signed(32768, 17);
                        z_v <= shift_left(resize(v_17, 21), 3) - resize(v_17, 21);         -- 7 (Y - 32768)
                        l_a <= resize(shift_left(resize(v_17, 21), 2), 22);                -- 4 (Y - 32768)
                    when 31 =>
                        -- cy (1/256 virtual px) = 138240 + 4 v + 7 v / 32;  b = 16 C - cy
                        l_a <= signed(resize(c_C & "0000", 22)) - to_signed(138240, 22) - l_a - resize(shift_right(z_v, 5), 22);
                    when 32 =>
                        l_m <= resize(c_C & "000", 18); l_q <= (others => '0');
                    when 34 =>
                        c_py0 <= unsigned(l_a(16 downto 0));
                        c_r40 <= unsigned(l_q(1 downto 0));
                        c_j0 <= resize(shift_right(l_q, 2), 6);
                        c_pcy0 <= signed(resize(c_C & "00", 20)) - resize(l_a(19 downto 0), 20);
                    when 35 =>                     -- i0 mod 15
                        l_a <= resize(c_i0, 22); l_m <= to_unsigned(15, 18); l_q <= (others => '0');
                    when 37 =>                     -- 4 j0 mod 15
                        c_i150 <= unsigned(l_a(3 downto 0));
                        l_a <= resize(c_j0 & "00", 22); l_q <= (others => '0');
                    when 39 =>
                        c_j40 <= unsigned(l_a(3 downto 0));
                    -- Drift (Hypnos SPEED): |v| < 24 still, else 2^-12 .. 2^-4 period per field
                    when 40 =>
                        v_v := resize(shift_right(s_vgs, 6), 11);
                        v_n11 := -v_v;
                        z_vn <= v_v(10);
                        if v_v(10) = '1' then z_av <= unsigned(v_n11(9 downto 0)); else z_av <= unsigned(v_v(9 downto 0)); end if;
                    when 41 =>
                        if z_av < 24 then z_vz <= '1'; z_u <= (others => '0');
                        else z_vz <= '0'; z_u <= z_av - 24; end if;
                    when 42 => z_ls <= resize(z_u + shift_right(z_u, 5), 9);
                    when 43 => z_ls <= z_ls + resize(shift_right(z_u, 6), 9);
                    when 44 => r_ex_a <= z_ls(5 downto 0) & "00";
                    when 45 | 46 => null;
                    when 47 =>
                        if z_vz = '1' then z_spd <= (others => '0');
                        else z_spd <= resize(r_exf(15 downto 7), 16); end if;
                        z_scn <= z_ls(8 downto 6);
                    when 48 =>                     -- the octave: a serial shift
                        if z_scn /= 0 then z_spd <= shift_left(z_spd, 1); z_scn <= z_scn - 1; q_st <= q_st; end if;
                    when 49 =>                     -- the lattice moves rigidly: V by the step, H by minus half of it
                        if z_vn = '1' then
                            d_v <= d_v - resize(z_spd, 20); d_h <= d_h + resize(z_spd(15 downto 1), 20);
                        else
                            d_v <= d_v + resize(z_spd, 20); d_h <= d_h - resize(z_spd(15 downto 1), 20);
                        end if;
                    -- the four families' seeds: (minus the geometric phase) mod P, + drift * P, + P / 2, into [-P, 0)
                    when 50 =>
                        z_f <= "00";
                    when 51 =>                     -- this family's phase, spacing and geometric phase (for the loop)
                        l_q <= (others => '0');
                        case to_integer(z_f) is
                            when 0      => z_d <= d_v;       z_pk <= c_P;  l_m <= c_P;  l_a <= signed(resize(C_HT_GV(to_integer(s_xs)), 22));
                            when 1      => z_d <= d_h;       z_pk <= c_P;  l_m <= c_P;  l_a <= signed(resize(C_HT_GH(to_integer(s_xs)), 22));
                            when 2      => z_d <= d_v + d_h; z_pk <= c_Pd; l_m <= c_Pd; l_a <= signed(resize(C_HT_GD1(to_integer(s_xs)), 22));
                            when others => z_d <= d_v - d_h; z_pk <= c_Pd; l_m <= c_Pd; l_a <= signed(resize(C_HT_GD2(to_integer(s_xs)), 22));
                        end case;
                    when 52 =>                     -- drift * P on the multiplier while the loop reduces the phase
                        m_a <= z_pk; m_b <= z_d(19 downto 4); m_hi <= (others => '0'); m_lo <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    when 54 =>
                        if m_cnt /= 0 then q_st <= q_st; else z_dp <= v_pr(33 downto 16); end if;
                    when 55 =>                     -- s = P - (g mod P)
                        z_s <= z_pk - unsigned(l_a(17 downto 0));
                    when 56 =>
                        v_s := resize(z_s, 19) + resize(z_dp, 19);
                        if v_s >= resize(z_pk, 19) then z_s <= resize(v_s - resize(z_pk, 19), 18); else z_s <= resize(v_s, 18); end if;
                    when 57 =>
                        v_s := resize(z_s, 19) + resize(z_pk(17 downto 1), 19);
                        if v_s >= resize(z_pk, 19) then z_s <= resize(v_s - resize(z_pk, 19), 18); else z_s <= resize(v_s, 18); end if;
                    when 58 =>
                        q_sd <= signed(resize(z_s, 20)) - signed(resize(z_pk, 20));
                        q_ld(to_integer(z_f)) <= '1';
                        z_f <= z_f + 1;
                        if z_f /= 3 then q_st <= to_unsigned(51, 7); end if;
                    -- the line steps: H rq, D1 rq / sqrt 2, D2 the spacing less that; and their "less a period"s
                    when 59 =>
                        case to_integer(s_xs) is
                            when 2      => z_rq <= resize(s_ys & "00", 12);
                            when 3      => z_rq <= to_unsigned(1024, 12);
                            when others => z_rq <= resize(s_ys & '0', 12);
                        end case;
                    when 60 =>
                        m_a <= resize(z_rq, 18); m_b <= to_unsigned(724, 16); m_hi <= (others => '0'); m_lo <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    when 61 =>
                        if m_cnt /= 0 then q_st <= q_st; else z_lsd <= v_pr(21 downto 10); end if;
                    when 62 => c_lsd1 <= signed(resize(z_lsd, 20));
                    when 63 => c_lswh <= v_sr;
                    when 64 => c_lswd1 <= v_sr;
                    when 65 => c_lsd2 <= v_sr;
                    when 66 => c_lswd2 <= v_sr;
                    -- Layers (P12): detent of 8 counts at the centre, u16 = 0 .. 16 either way; Camera: 0 .. 32
                    when 67 =>
                        if g_s(15) = '1' then c_up <= '1'; v_u := g_s - 32768; else c_up <= '0'; v_u := 32768 - g_s; end if;
                        v_dq := signed('0' & v_u) - 512;
                        if v_dq(16) = '1' then
                            c_u16 <= (others => '0');
                        else
                            v_dq := v_dq + shift_right(v_dq, 5);
                            if v_dq(16 downto 11) >= 16 then c_u16 <= to_unsigned(16, 5); else c_u16 <= unsigned(v_dq(15 downto 11)); end if;
                        end if;
                        v_k6 := resize(g_c, 17) + 1024;
                        c_k6 <= v_k6(16 downto 11);
                    when 68 =>
                        if c_up = '1' then c_s16 <= signed(resize(c_u16 & '0', 8)); else c_s16 <= -signed(resize(c_u16 & '0', 8)); end if;
                    when 69 =>
                        q_kick <= '1'; q_run <= '0';
                    when others => null;
                end case;
            end if;
        end if;
    end process p_seq;

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
    -- [D] SOURCE FIELD: luma (or the Osc lattice, x 4 about its middle) -> box of 32 px (16 px at 720p and SD),
    -- edge-replicated and reloaded at every line start -> vertical one-pole (1/16 per line at 1080p, 1/8 at
    -- 720p / 1080i / 480p / 576p, 1/4 at 480i / 576i) at half horizontal resolution.
    -- CAMERA GRID: on every line the one-pole's value at each 32 x 32 virtual-px block's centre column (plus
    -- the box lag) is written, 4 bits, to the block's cell of the write bank; the last line of a block row stays.
    -- Block columns are canvas columns (SD starts at block 7).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_dl : signed(16 downto 0);
        variable v_n : signed(17 downto 0);
        variable v_qy : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + f_oxs(s_xs);
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then   -- both phases restart from this field's drift
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);    -- ot * 96
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
            -- Osc: (lattice - 96) * 4, clamped (nearly two-level); Video: as it is.  s_tv is the sample of column s_col
            if s_src = '0' then s_cv <= s_sl;
            elsif s_sl < 96 then s_cv <= (others => '0');
            elsif s_sl >= 160 then s_cv <= (others => '1');
            else
                v_l := s_sl - 96;
                s_cv <= v_l(5 downto 0) & "00";
            end if;
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_tv <= s_cv;
            if s_v2 = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 (16) samples; the samples before the line start count as the first one
            if s_v2 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            elsif c_b32 = '1' then bx_wp <= (others => '0'); bx_rp <= to_unsigned(226, 8);
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(242, 8); end if;
            bx_f <= bx_q;
            if s_bcnt(6 downto 5) = "00" and (c_b32 = '1' or s_bcnt(4) = '0') then v_tap := s_sfirst; else v_tap := bx_f; end if;
            s_df <= signed(resize(s_tv, 9)) - signed(resize(v_tap, 9));
            s_tvd <= s_tv; s_v3 <= s_v2; s_c2 <= s_col;
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_sfirst <= s_tv;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_v2 = '1' then
                if s_bcnt(6) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_ld = '1' then
                if c_b32 = '1' then s_bsum <= shift_left(resize(s_tvd, 13), 5);
                else s_bsum <= shift_left(resize(s_tvd, 13), 4); end if;
            elsif s_v3 = '1' then
                s_bsum <= unsigned(signed(s_bsum) + resize(s_df, 13));
            end if;
            s_v4 <= s_v3; s_bcol <= s_c2;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif s_ld = '1' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- vertical one-pole on the odd columns (8.8): q += (box - q) >> shift
            if c_b32 = '1' then s_eb <= s_bsum & "000"; else s_eb <= s_bsum(11 downto 0) & "0000"; end if;
            s_eq <= lb_q1;
            s_ev <= s_v4 and s_bcol(0); s_ecol <= s_bcol; s_efl <= s_fl;
            s_dd <= signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            if s_efl = '1' then s_fq <= s_eb; else s_fq <= s_eq; end if;
            s_fv <= s_ev; s_fcol <= s_ecol; s_ffl <= s_efl;
            if s_ffl = '1' then v_dl := (others => '0');
            elsif c_iir = 4 then v_dl := shift_right(s_dd, 4);
            elsif c_iir = 3 then v_dl := shift_right(s_dd, 3);
            else v_dl := shift_right(s_dd, 2); end if;
            v_n := signed(resize(s_fq, 18)) + resize(v_dl, 18);
            lb_wd <= unsigned(v_n(15 downto 0));
            lb_we <= s_fv; lb_wa <= s_fcol(10 downto 1);
            s_v5 <= s_v4; s_v6 <= s_v5; s_v7 <= s_v6; s_v7d <= s_v7;

            -- camera grid: block centres along the line (h units: 64 per block), block rows down the field
            cm_we <= '0';
            if s_zone = '1' then
                s_row <= (others => '0'); s_qy <= (others => '0');
                s_bc <= f_bc0(s_xs); s_qx <= f_qx0(s_xs);
            elsif lb_we = '1' then
                s_lw <= lb_wd(15 downto 12);
                if s_qx(7) = '0' then
                    cm_we <= '1'; cm_wa <= s_wb & s_row & s_bc; cm_wd <= lb_wd(15 downto 12);
                    s_qx <= s_qx + signed(resize(s_xs & '0', 8)) - 64;
                    s_bc <= s_bc + 1;
                else
                    s_qx <= s_qx + signed(resize(s_xs & '0', 8));
                end if;
            elsif s_v7 = '0' and s_v7d = '1' then          -- line end: the pending block takes the last value
                cm_we <= '1'; cm_wa <= s_wb & s_row & s_bc; cm_wd <= s_lw;
                s_bc <= f_bc0(s_xs); s_qx <= f_qx0(s_xs);
                v_qy := s_qy + resize(s_ys, 14);
                if v_qy >= 8192 then
                    s_qy <= v_qy - 8192;
                    if s_row /= 63 then s_row <= s_row + 1; end if;
                else
                    s_qy <= v_qy;
                end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] CELLS.  The cell DDA (h units, Q4) holds pixel i in period i: the cell (a quarter panel) and its
    -- centre column; the vertical state moves on at every line end.  Then the stages c1 .. c11:
    --   c1 the block column (clamped) and the grid combination / ring parts;  c2 the camera word, the ring;
    --   c3 its fabric copy, the combination index and the dither;  c4 the combination's size, a = K6 x (0, 1/2,
    --   1), the luma's layer level t;  c5 the panel's own level (Layers) and the camera's level;  c6 their
    --   difference;  c7 a x difference;  c8 the cell's level + dither;  c9 / 16, clamped -> families and weight
    --   step;  c10 the mask and the two half widths;  c11 the mask again.
    ---------------------------------------------------------------------------
    p_cell : process(clk)
        variable v_pw  : signed(14 downto 0);
        variable v_cg  : unsigned(4 downto 0);
        variable v_ai  : signed(6 downto 0);
        variable v_r   : unsigned(6 downto 0);
        variable v_m   : unsigned(1 downto 0);
        variable v_mu  : unsigned(6 downto 0);
        variable v_t   : unsigned(3 downto 0);
        variable v_col : signed(7 downto 0);
        variable v_yw  : signed(17 downto 0);
        variable v_lv  : signed(9 downto 0);
        variable v_d   : signed(5 downto 0);
        variable v_cmb : unsigned(9 downto 0);
        variable v_rk  : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            -- the cell DDA along the line (pixel 0 in period 0)
            if g_start = '1' then
                e_px <= c_rx; e_c4 <= c_c40; e_ip <= c_i0; e_i15 <= c_i150; e_pcx <= c_pcx0;
            else
                v_pw := signed(resize(e_px, 15)) + resize(c_xsc, 15);
                if v_pw(14) = '0' then
                    e_px <= unsigned(v_pw(13 downto 0));
                    e_pcx <= e_pcx + signed(resize(c_C, 18));
                    e_c4 <= e_c4 + 1;
                    if e_c4 = 3 then
                        e_ip <= e_ip + 1;
                        if e_i15 = 14 then e_i15 <= (others => '0'); else e_i15 <= e_i15 + 1; end if;
                    end if;
                else
                    e_px <= e_px + resize(s_xs & "0000", 14);
                end if;
            end if;
            -- the vertical state: set by the kick, moved on at every line end
            if q_kick = '1' then
                e_py <= c_py0; e_r4 <= c_r40; e_jp <= c_j0; e_j4 <= c_j40; e_pcy <= c_pcy0;
            elsif b_end = '1' then
                v_yw := signed(resize(e_py, 18)) + signed(resize(s_ys, 18)) - signed(resize(c_C & "000", 18));
                if v_yw(17) = '0' then
                    e_py <= unsigned(v_yw(16 downto 0));
                    e_pcy <= e_pcy + signed(resize(c_C & "000", 20));
                    e_r4 <= e_r4 + 1;
                    if e_r4 = 3 then
                        e_jp <= e_jp + 1;
                        if e_j4 >= 11 then e_j4 <= e_j4 - 11; else e_j4 <= e_j4 + 4; end if;
                    end if;
                else
                    e_py <= e_py + resize(s_ys, 17);
                end if;
            end if;
            if e_pcy(19) = '1' then e_row <= (others => '0');
            elsif e_pcy(19 downto 13) > 33 then e_row <= to_unsigned(33, 6);
            else e_row <= unsigned(e_pcy(18 downto 13)); end if;
            if e_jp(5) = '1' then e_aj <= unsigned(-e_jp); else e_aj <= unsigned(e_jp); end if;

            -- c1: the block column, clamped to the visible canvas (SD shows columns 7 .. 52)
            v_col := e_pcx(17 downto 10);
            if c_sd4 = '1' then
                if v_col < 7 then e1_col <= to_unsigned(7, 6);
                elsif v_col > 52 then e1_col <= to_unsigned(52, 6);
                else e1_col <= unsigned(v_col(5 downto 0)); end if;
            else
                if v_col < 0 then e1_col <= to_unsigned(0, 6);
                elsif v_col > 59 then e1_col <= to_unsigned(59, 6);
                else e1_col <= unsigned(v_col(5 downto 0)); end if;
            end if;
            e1_cg <= resize(e_i15, 5) + resize(e_j4, 5);
            v_ai := e_ip;
            if v_ai(6) = '1' then e1_ai <= unsigned(-v_ai); else e1_ai <= unsigned(v_ai); end if;
            e1_phg <= f_bayer(unsigned(e_jp(1 downto 0)), unsigned(e_ip(1 downto 0)));
            -- c2 (the camera word arrives: cm_q)
            v_cg := e1_cg;
            if v_cg >= 15 then v_cg := v_cg - 15; end if;
            e2_cg <= v_cg(3 downto 0);
            if e1_ai > resize(e_aj, 7) then e2_ring <= e1_ai; else e2_ring <= resize(e_aj, 7); end if;
            e2_phg <= e1_phg;
            -- c3
            cm_f <= cm_q;
            if s_frames = '1' then
                v_r := e2_ring;
                if v_r >= 45 then v_r := v_r - 45; elsif v_r >= 30 then v_r := v_r - 30; elsif v_r >= 15 then v_r := v_r - 15; end if;
                e3_ci <= v_r(3 downto 0);
                e3_phi <= f_bitrev(e2_ring(3 downto 0));
            else
                e3_ci <= e2_cg;
                e3_phi <= e2_phg;
            end if;
            -- c4
            v_cmb := C_HT_CMB(to_integer(e3_ci));
            e4_b1 <= v_cmb(9 downto 8);
            e4_ci <= e3_ci; e4_phi <= e3_phi;
            if cm_f = 0 then e4_a <= (others => '0');
            elsif cm_f = 1 then e4_a <= '0' & c_k6(5 downto 1);
            else e4_a <= c_k6; end if;
            v_t := C_HT_TT(to_integer(cm_f));
            if s_ink = '1' then e4_t <= 8 - v_t; else e4_t <= v_t; end if;
            -- c5: the panel's level 16 b +- m u16 and the camera's 16 t + s16
            if c_up = '1' then v_m := not e4_b1; else v_m := e4_b1; end if;         -- 4 - b or b - 1
            case v_m is
                when "00"   => v_mu := (others => '0');
                when "01"   => v_mu := resize(c_u16, 7);
                when "10"   => v_mu := resize(c_u16 & '0', 7);
                when others => v_mu := resize(c_u16 & '0', 7) + resize(c_u16, 7);
            end case;
            if c_up = '1' then
                e5_nc <= signed(resize(e4_b1 & "0000", 9)) + 16 + signed(resize(v_mu, 9));
            else
                e5_nc <= signed(resize(e4_b1 & "0000", 9)) + 16 - signed(resize(v_mu, 9));
            end if;
            e5_dp <= signed(resize(e4_t & "0000", 9)) + resize(c_s16, 9);
            e5_a <= e4_a; e5_ci <= e4_ci; e5_phi <= e4_phi;
            -- c6
            e6_df <= e5_dp - e5_nc;
            e6_nc <= e5_nc; e6_a <= e5_a; e6_ci <= e5_ci; e6_phi <= e5_phi;
            -- c7
            e7_pr <= resize(signed('0' & e6_a) * e6_df, 15);
            e7_nc <= e6_nc; e7_ci <= e6_ci; e7_phi <= e6_phi;
            if e6_a >= 16 then e7_port <= '1'; else e7_port <= '0'; end if;
            -- c8
            e8_lv <= resize(e7_nc, 10) + resize(shift_right(e7_pr, 5), 10) + signed(resize(e7_phi, 10));
            e8_ci <= e7_ci; e8_port <= e7_port;
            -- c9: / 16, clamped to [0, 8] (camera) or [1, 4] (the panel's own level)
            v_lv := shift_right(e8_lv, 4);
            v_d := v_lv(5 downto 0);
            if e8_port = '1' then
                if v_d < 0 then v_d := (others => '0'); elsif v_d > 8 then v_d := to_signed(8, 6); end if;
            else
                if v_d < 1 then v_d := to_signed(1, 6); elsif v_d > 4 then v_d := to_signed(4, 6); end if;
            end if;
            if v_d > 4 then
                e9_nf <= to_unsigned(4, 3); e9_w <= resize(unsigned(v_d) - 4, 3);
            else
                e9_nf <= unsigned(v_d(2 downto 0)); e9_w <= (others => '0');
            end if;
            e9_ci <= e8_ci;
            -- c10
            v_cmb := C_HT_CMB(to_integer(e9_ci));
            for f in 0 to 3 loop
                v_rk := v_cmb(2 * f + 1 downto 2 * f);
                if resize(v_rk, 3) < e9_nf then e10_mk(f) <= '1'; else e10_mk(f) <= '0'; end if;
            end loop;
            e10_tv <= c_tv(to_integer(e9_w));
            e10_td <= c_td(to_integer(e9_w));
            -- c11
            e11_mk <= e10_mk;
        end if;
    end process p_cell;

    ---------------------------------------------------------------------------
    -- [F] FAMILIES.  Late-wrap DDAs in pixel widths (Q10): A lives in [-P, 0), or in [0, step) on the one
    -- pixel after the sum reached 0 (that pixel sits half a period from every line, where no edge can be).
    -- V steps 1 px per pixel, the diagonals 1/sqrt 2; H is constant along a line.  The line accumulators of H,
    -- D1 and D2 step once per line end the same way; the sequencer loads all four seeds.  A pixel DDA holds
    -- pixel i in period 9 + i (loaded from its line accumulator at edge E9).  Then: u = (A + P/2) >> 6 (Q4),
    -- e = T - |u| (one add or subtract), halved on interlaced rasters for the three families that cross field
    -- lines, the coverage clamp(e + 8, 0, 16) under the cell's mask, the maximum, the luma.
    ---------------------------------------------------------------------------
    p_fam : process(clk)
        variable v_u  : signed(19 downto 0);
        variable v_tt : signed(13 downto 0);
        variable v_e  : signed(13 downto 0);
        variable v_c  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- line window: W + 8 steps
            b_end <= '0';
            if s_st(8) = '1' then
                b_run <= '1'; b_cnt <= to_unsigned(4088, 12);
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = g_wm1 then b_run <= '0'; b_end <= '1'; end if;
            end if;
            -- line accumulators (seeded by the sequencer in the zone)
            if q_ld(0) = '1' then la_v <= q_sd; end if;
            if q_ld(1) = '1' then
                la_h <= q_sd;
            elsif b_end = '1' then
                if la_h(19) = '0' then la_h <= la_h + c_lswh; else la_h <= la_h + signed(resize(z_rq, 20)); end if;
            end if;
            if q_ld(2) = '1' then
                la_d1 <= q_sd;
            elsif b_end = '1' then
                if la_d1(19) = '0' then la_d1 <= la_d1 + c_lswd1; else la_d1 <= la_d1 + c_lsd1; end if;
            end if;
            if q_ld(3) = '1' then
                la_d2 <= q_sd;
            elsif b_end = '1' then
                if la_d2(19) = '0' then la_d2 <= la_d2 + c_lswd2; else la_d2 <= la_d2 + c_lsd2; end if;
            end if;
            -- pixel DDAs
            if s_st(8) = '1' then
                a_v <= la_v; a_d1 <= la_d1; a_d2 <= la_d2;
            elsif b_run = '1' then
                if a_v(19) = '0' then a_v <= a_v + c_incwv; else a_v <= a_v + 1024; end if;
                if a_d1(19) = '0' then a_d1 <= a_d1 + c_incwd; else a_d1 <= a_d1 + 724; end if;
                if a_d2(19) = '0' then a_d2 <= a_d2 + c_incwd; else a_d2 <= a_d2 + 724; end if;
            end if;
            -- stage 1: the signed distance from the line centre, Q4
            v_u := a_v + signed(resize(c_P(17 downto 1), 20));    x1_u(0) <= v_u(19 downto 6);
            v_u := la_h + signed(resize(c_P(17 downto 1), 20));   x1_u(1) <= v_u(19 downto 6);
            v_u := a_d1 + signed(resize(c_Pd(17 downto 1), 20));  x1_u(2) <= v_u(19 downto 6);
            v_u := a_d2 + signed(resize(c_Pd(17 downto 1), 20));  x1_u(3) <= v_u(19 downto 6);
            for f in 0 to 3 loop
                -- stage 2: e = T - |u| on one carry chain
                if f < 2 then v_tt := signed(resize(e10_tv, 14)); else v_tt := signed(resize(e10_td, 14)); end if;
                if x1_u(f)(13) = '1' then x2_e(f) <= v_tt + x1_u(f); else x2_e(f) <= v_tt - x1_u(f); end if;
                -- stage 3: halved for H, D1, D2 on interlaced rasters; the coverage under the cell's mask
                if s_il = '1' and f > 0 then v_e := shift_right(x2_e(f), 1); else v_e := x2_e(f); end if;
                v_c := v_e + 8;
                if e11_mk(f) = '0' or v_c(13) = '1' then x3_c(f) <= (others => '0');
                elsif v_c > 16 then x3_c(f) <= to_unsigned(16, 5);
                else x3_c(f) <= unsigned(v_c(4 downto 0)); end if;
            end loop;
            -- stages 4, 5: the union
            if x3_c(0) > x3_c(1) then x4_a <= x3_c(0); else x4_a <= x3_c(1); end if;
            if x3_c(2) > x3_c(3) then x4_b <= x3_c(2); else x4_b <= x3_c(3); end if;
            if x4_a > x4_b then x5_m <= x4_a; else x5_m <= x4_b; end if;
            -- stage 6: the luma (Grey: white lines on black; Ink: black lines on white); 7, 8: registers; 9: s_y23
            if s_ink = '1' then x6_y <= to_unsigned(940, 10) - f_yv(x5_m); else x6_y <= to_unsigned(64, 10) + f_yv(x5_m); end if;
            x7_y <= x6_y;
            x8_y <= x7_y;
            s_y23 <= x8_y;
        end if;
    end process p_fam;

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

end architecture hatch;
