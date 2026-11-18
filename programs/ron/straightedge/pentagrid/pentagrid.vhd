-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Pentagrid v0.1.0 -- Straightedge.  N. G. de Bruijn's pentagrid (1981, the construction behind the Penrose
-- tilings), Francois Morellet's superimposed grids, Islamic quasi-periodic star patterns: three to eight
-- families of crisp parallel bands fanned through a half turn; every family is a translucent film and the tone
-- is the number of films over the pixel.  Luma only (U = V = 512).
--
--   family k (k < N):  u_k = distance (pixels, modulo the period P) from the nearest band axis of
--                            n_k . (x - X, y - Y) + P d_k,   n_k at  (k - (N - 1) / 2) * Fan,  d_k the drift phase
--   region            = |u_k| < T        (Bold: the black gap between two bands, T = P / 4;  Fine: the line, T = P / 12)
--   Stairs            = the regions |u_k| < T / 2 and < 3 T / 2 make a four-level sawtooth across the period
--   S                 = the summed coverage of the regions (16 per region), anti-aliased per edge
--   tone              = a per-field table of S: Grey Bold the count as flat steps, Grey Fine films "over" at
--                       50 %, Ink the parity (a triangle of S)
--   camera            = the filtered luma is sampled once per 34 x 34 virtual-px block into a two-bank grid,
--                       read a field later, quantised to four levels, held across the block: it sets T there
--
--   K1  X        pan of the lattice (and the pivot of the fan), +-1024 virtual px          (glided)
--   K2  Y        +-576 virtual px                                                           (glided)
--   K3  Scale    band period 54 .. 864 virtual px, exponential (216 at the centre)          (glided)
--   K4  Order    3 .. 8 families (stepped, hysteresis)
--   K5  Drift    every family streams along its own normal at its own rate (1, -3/4, 2/3, -1/2, 5/4, -7/8,
--                9/8, -5/8): Hypnos SPEED, still within +-24 counts of the centre, 2^-12 .. 2^-4 period per field
--   K6  Camera   how far a bright block narrows the gaps (Fine: widens the lines)           (glided)
--   S7  Glide    Off = the knob value of this field, On = 1/16 of the gap per field (speed: 1/8)
--   S8  Step     Flat / Stairs
--   S9  Tone     Grey / Ink
--   S10 Weight   Bold (equal bands) / Fine (lines of 1/6 of the period)
--   S11 Source   Video / Osc
--   P12 FAN      the angle between neighbouring families: 0 = all parallel (a barcode; the periods spread),
--                50 % = a half turn / N (the star lattice), 100 % = a full turn / N         (glided)
--
-- Sections (the integer model .straightedge_work/rb_pentagrid/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide, drift speed, Order hysteresis, switches
--   [C] FIELD SEQUENCER                  one serial multiplier, (N + 1) x 16 slots of 24 clocks: period, fan, pan,
--                                        then per family sin / cos, the pixel step, the seed and the line wrap
--                                        (both reduced modulo the period), the drift phase
--   [D] SOURCE FIELD AND CAMERA GRID     luma or the Osc lattice, box, vertical one-pole, block samples
--   [E] FAMILIES                         8 affine DDAs in pixels that wrap modulo the period; two edges each
--   [F] SUM AND TONE                     adder tree, tone table
--   [G] OUTPUT CONDITIONING              luma [1 2 1]/4, luma 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  The DDAs hold DDA pixel i at T + 6 + i; a register "at stage S" holds pixel i
-- at T + 6 + i + S; the luma is stage 9.  Output column j is the [1 2 1] filter centred on DDA pixel j + 1.
-- One latency (20) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.pentagrid_pkg.all;
use work.all;

architecture pentagrid of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
    constant C_DEF_K5  : integer := 680;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;

    constant C_WHITE : integer := 60160;      -- Q6 luma
    constant C_BLACK : integer := 4096;

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
    -- constants of the three pixel widths (x step 2 / 3 / 4 half virtual px): pixels per virtual px (Q15), the
    -- period's scale (54 virtual px in Q9 pixels / 65536), its inverse, the Osc lattice's x step, and the first
    -- block centre less half the box (h units)
    function f_sc(xs : unsigned(2 downto 0)) return unsigned is
    begin
        if xs = 4 then return to_unsigned(16384, 16); elsif xs = 3 then return to_unsigned(21845, 16); else return to_unsigned(32768, 16); end if;
    end function;
    function f_cp(xs : unsigned(2 downto 0)) return unsigned is
    begin
        if xs = 4 then return to_unsigned(27648, 16); elsif xs = 3 then return to_unsigned(36864, 16); else return to_unsigned(55296, 16); end if;
    end function;
    function f_ci(xs : unsigned(2 downto 0)) return unsigned is
    begin
        if xs = 4 then return to_unsigned(38836, 16); elsif xs = 3 then return to_unsigned(29127, 16); else return to_unsigned(19418, 16); end if;
    end function;
    function f_oxs(xs : unsigned(2 downto 0)) return unsigned is
    begin
        if xs = 4 then return to_unsigned(204, 9); elsif xs = 3 then return to_unsigned(153, 9); else return to_unsigned(102, 9); end if;
    end function;
    function f_qxi(xs : unsigned(2 downto 0)) return signed is
    begin
        if xs = 4 then return to_signed(-58, 8); elsif xs = 3 then return to_signed(-52, 8); else return to_signed(-62, 8); end if;
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
    -- coverage of one edge from its signed distance e (1/16 pixel; 1/8 when soft): clamp(e + 8, 0, 16)
    function f_cov(e : signed(13 downto 0); soft : std_logic) return unsigned is
        variable v_hi  : std_logic_vector(10 downto 0);
        variable v_lo  : std_logic_vector(3 downto 0);
        variable v_all0, v_all1 : std_logic;
        variable v_c   : unsigned(4 downto 0);
    begin
        if soft = '1' then
            v_hi := e(13) & std_logic_vector(e(13 downto 4));
            v_lo := std_logic_vector(e(4 downto 1));
        else
            v_hi := std_logic_vector(e(13 downto 3));
            v_lo := std_logic_vector(e(3 downto 0));
        end if;
        v_all0 := '1'; v_all1 := '1';
        for i in 0 to 10 loop
            if v_hi(i) = '1' then v_all0 := '0'; else v_all1 := '0'; end if;
        end loop;
        if v_all0 = '1' or v_all1 = '1' then
            v_c := (others => '0');
            v_c(3) := not v_lo(3);
            v_c(2 downto 0) := unsigned(v_lo(2 downto 0));
            return v_c;
        elsif e(13) = '0' then
            return to_unsigned(16, 5);
        else
            return to_unsigned(0, 5);
        end if;
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

    -- [B] knobs: glide accumulators (Q6) and their "moving" flags
    signal g_x : unsigned(15 downto 0) := to_unsigned(C_DEF_K1 * 64, 16);
    signal g_y : unsigned(15 downto 0) := to_unsigned(C_DEF_K2 * 64, 16);
    signal g_p : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);
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
    signal s_stairs, s_ink, s_fine, s_src : std_logic := '0';
    signal s_zn  : unsigned(2 downto 0) := to_unsigned(2, 3);          -- Order - 3
    signal s_n   : unsigned(3 downto 0) := to_unsigned(5, 4);
    signal z_v6  : unsigned(12 downto 0) := to_unsigned(2562, 13);
    signal z_up, z_dn : std_logic := '0';
    signal s_act : std_logic_vector(7 downto 0) := "00011111";

    -- [C] ROMs
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q, sn_f : unsigned(15 downto 0) := (others => '0');

    -- [C] sequencer and its serial multiplier
    signal q_run, q_go, q_cut : std_logic := '0';
    signal q_k  : unsigned(3 downto 0) := (others => '0');
    signal q_ck : unsigned(2 downto 0) := (others => '0');
    signal q_j  : unsigned(3 downto 0) := (others => '0');
    signal q_ph : unsigned(4 downto 0) := (others => '0');
    signal f_a, f_acc : signed(43 downto 0) := (others => '0');
    signal z_fa : signed(27 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    -- mode constants
    signal z_rq   : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal z_m2   : unsigned(11 downto 0) := to_unsigned(1930, 12);
    signal z_w44  : unsigned(13 downto 0) := to_unsigned(7684, 14);
    -- period
    signal z_oct  : unsigned(1 downto 0) := (others => '0');
    signal z_fr   : unsigned(5 downto 0) := (others => '0');
    signal z_e255, z_e000 : std_logic := '0';
    signal z_e0, z_cio : unsigned(15 downto 0) := (others => '0');
    signal z_g    : unsigned(16 downto 0) := (others => '0');          -- scratch: a result that feeds the next product
    signal z_tsa : unsigned(11 downto 0) := (others => '0');
    signal c_p    : unsigned(23 downto 0) := (others => '0');          -- the period, Q14 pixels
    signal c_invp : unsigned(15 downto 0) := (others => '0');          -- 1 / period, Q20
    signal c_t0   : unsigned(11 downto 0) := (others => '0');          -- the region's half width, Q4
    signal c_ts   : unsigned(10 downto 0) := (others => '0');          -- its step per camera level
    -- fan
    signal z_fz   : std_logic_vector(3 downto 0) := (others => '0');
    signal z_fv, z_fw : unsigned(15 downto 0) := (others => '0');
    signal z_s10  : unsigned(10 downto 0) := (others => '0');
    signal z_phi, z_th : unsigned(17 downto 0) := (others => '0');
    -- pan
    signal z_y9   : unsigned(19 downto 0) := (others => '0');
    signal z_yv   : signed(15 downto 0) := (others => '0');
    signal z_pv   : signed(16 downto 0) := (others => '0');
    signal z_ax, z_ay : unsigned(15 downto 0) := (others => '0');
    signal c_xn, c_yn : std_logic := '0';
    -- drift speed
    signal z_vn, z_vz : std_logic := '0';
    signal z_av, z_u : unsigned(9 downto 0) := (others => '0');
    signal z_ls   : unsigned(8 downto 0) := (others => '0');
    signal z_spd  : unsigned(15 downto 0) := (others => '0');
    signal z_scn  : unsigned(2 downto 0) := (others => '0');
    -- per family
    signal z_D    : unsigned(19 downto 0) := (others => '0');
    signal z_p    : unsigned(16 downto 0) := (others => '0');
    signal z_ov, z_ov1, z_sneg, z_cneg : std_logic := '0';
    signal z_ff, z_df : unsigned(5 downto 0) := (others => '0');
    signal z_sm, z_cm, z_lo : unsigned(14 downto 0) := (others => '0');   -- sine and cosine, then the line and pixel steps
    signal z_r, z_dy : unsigned(15 downto 0) := (others => '0');
    signal z_soft, z_dyn : std_logic := '0';
    signal z_x, z_o : signed(27 downto 0) := (others => '0');          -- the work register and its operand
    signal z_qm   : unsigned(8 downto 0) := (others => '0');
    signal z_qn   : std_logic := '0';
    signal z_xclr, z_xadd, z_xsub, z_xsg, z_xcnd : std_logic := '0';
    signal c_soft : std_logic_vector(7 downto 0) := (others => '0');

    -- [E] family table: (k, 0) = the pixel step, (k, 1) = the step less a period, (k, 2) = the line wrap,
    -- (k, 3) = the field seed, (k, 4) = the drift phase
    type t_cr is array (0 to 255) of signed(31 downto 0);
    signal cr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_ra, cr_wa : unsigned(5 downto 0) := (others => '0');
    signal cr_q, cr_wd : signed(31 downto 0) := (others => '0');
    signal cr_we : std_logic := '0';

    -- [E] family DDAs and their loader
    type t_s25x8 is array (0 to 7) of signed(24 downto 0);
    signal ca_acc : t_s25x8 := (others => (others => '0'));
    type t_u15x8 is array (0 to 7) of unsigned(14 downto 0);
    signal ca_inc : t_u15x8 := (others => (others => '0'));              -- the pixel step (0 .. 1.22 px, Q14)
    type t_s20x8 is array (0 to 7) of signed(19 downto 0);
    signal ca_incw : t_s20x8 := (others => (others => '0'));             -- the step less a period, bits 24 .. 5
    signal ca_ld, ca_ldp, ca_pk, ca_nm : std_logic_vector(7 downto 0) := (others => '0');
    signal ca_rst : std_logic := '0';
    signal l_kick, l_run, l_seed : std_logic := '0';
    signal l_t : unsigned(4 downto 0) := (others => '0');
    signal l_v1, l_v2 : std_logic := '0';
    signal l_c1, l_c2 : unsigned(2 downto 0) := (others => '0');
    signal l_p1, l_p2 : unsigned(1 downto 0) := (others => '0');
    signal b_run, b_end : std_logic := '0';
    signal b_cnt : unsigned(11 downto 0) := (others => '0');

    -- [E] edges
    type t_s15x8 is array (0 to 7) of signed(14 downto 0);
    signal x1_u : t_s15x8 := (others => (others => '0'));
    type t_s14x8 is array (0 to 7) of signed(13 downto 0);
    signal x2_ea, x2_eb : t_s14x8 := (others => (others => '0'));
    signal x2_sg, x3_sg : std_logic_vector(7 downto 0) := (others => '0');
    type t_u5x8 is array (0 to 7) of unsigned(4 downto 0);
    signal x3_ca, x3_cb : t_u5x8 := (others => (others => '0'));
    type t_u7x8 is array (0 to 7) of unsigned(6 downto 0);
    signal x4_s : t_u7x8 := (others => (others => '0'));
    type t_u8x4 is array (0 to 3) of unsigned(7 downto 0);
    signal x5_s : t_u8x4 := (others => (others => '0'));
    signal x6_sa, x6_sb, x7_s : unsigned(8 downto 0) := (others => '0');

    -- [F] tone table
    type t_tn is array (0 to 511) of unsigned(15 downto 0);
    signal tn_ram : t_tn := (others => to_unsigned(64, 16));
    attribute no_rw_check of tn_ram : signal is true;
    signal tn_q, tn_wd : unsigned(15 downto 0) := to_unsigned(64, 16);
    signal tn_wa, t_i : unsigned(8 downto 0) := (others => '0');
    signal tn_we, t_run, t_ph : std_logic := '0';
    signal t_acc  : unsigned(16 downto 0) := (others => '0');
    signal t_step : signed(12 downto 0) := (others => '0');
    signal t_fa, t_fb, t_fc : unsigned(10 downto 0) := (others => '0');
    signal t_lo, t_hi : unsigned(4 downto 0) := (others => '0');
    signal t_sl : unsigned(11 downto 0) := (others => '0');

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

    -- [D] camera grid: two banks of 32 x 64 blocks, 8 bits
    type t_cm is array (0 to 4095) of unsigned(7 downto 0);
    signal cm_ram : t_cm := (others => (others => '0'));
    attribute no_rw_check of cm_ram : signal is true;
    signal cm_ra, cm_wa : unsigned(11 downto 0) := (others => '0');
    signal cm_q, cm_f, cm_wd, s_lw : unsigned(7 downto 0) := (others => '0');
    signal cm_we, s_wb : std_logic := '0';
    signal s_qx : signed(7 downto 0) := (others => '0');
    signal s_bc  : unsigned(5 downto 0) := (others => '0');
    signal s_qy, d_qy : unsigned(13 downto 0) := (others => '0');
    signal s_row, d_row : unsigned(4 downto 0) := (others => '0');
    signal d_x   : signed(7 downto 0) := (others => '0');
    signal d_col : unsigned(5 downto 0) := (others => '0');
    signal d_nb, d_nb1, d_nb2 : std_logic := '0';
    signal d_q   : unsigned(1 downto 0) := (others => '0');
    signal x_qts : unsigned(12 downto 0) := (others => '0');
    signal x_t   : unsigned(12 downto 0) := (others => '0');
    signal x_ta, x_tb : signed(13 downto 0) := (others => '0');

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
            r_ex_q <= C_PG_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_PG_SIN(to_integer(sn_a));
            sn_f   <= sn_q;
        end if;
    end process p_rom;

    -- family table: written by the sequencer in the zone, read by it and by the loader (kick, line ends)
    p_crram : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= cr_ram(to_integer(resize(cr_ra, 8)));
            if cr_we = '1' then cr_ram(to_integer(resize(cr_wa, 8))) <= cr_wd; end if;
        end if;
    end process p_crram;

    -- tone table: written in the zone, read by the pixel path
    p_tnram : process(clk)
    begin
        if rising_edge(clk) then
            tn_q <= tn_ram(to_integer(x7_s));
            if tn_we = '1' then tn_ram(to_integer(tn_wa)) <= tn_wd; end if;
        end if;
    end process p_tnram;

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

    -- camera grid: the scan writes one bank, the pixel path reads the other (the bank bit toggles per field)
    cm_ra <= (not s_wb) & d_row & d_col;
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
        variable v_k   : unsigned(9 downto 0);
        variable v_lo, v_hi : unsigned(13 downto 0);
        variable v_st, v_tg : signed(16 downto 0);
        variable v_ri  : unsigned(3 downto 0);
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
                s_stairs <= registers_in(6)(1);
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
                -- Order: 6 steps with hysteresis (enter a step only 8 counts past its edge; the two ends
                -- are reached because the band lies on the entry side)
                if z_up = '1' or z_dn = '1' then s_zn <= z_v6(12 downto 10); end if;
            end if;
            -- (the comparison itself runs free, one step per clock, so no knob carry chain sits on the latch)
            v_k := unsigned(registers_in(3));
            z_v6 <= (v_k & "000") - resize(v_k & '0', 13);
            v_lo := resize(s_zn & "0000000000", 14);
            v_hi := v_lo + 1072;
            if resize(z_v6, 14) >= v_hi then z_up <= '1'; else z_up <= '0'; end if;
            if resize(z_v6, 14) + 48 < v_lo then z_dn <= '1'; else z_dn <= '0'; end if;
            s_n <= resize(s_zn, 4) + 3;
            g_wm1 <= s_w - 1;
            z_m2  <= s_w + 10;
            z_w44 <= (s_w & "00") + 4;
            s_oys <= f_oys(s_tid);
            case to_integer(s_xs) is               -- line height in pixel widths, Q10
                when 4      => z_rq <= resize(s_ys & '0', 12);
                when 3      => z_rq <= to_unsigned(1024, 12);
                when others => z_rq <= resize(s_ys & "00", 12);
            end case;
            case to_integer(s_zn) is
                when 0      => s_act <= "00000111";
                when 1      => s_act <= "00001111";
                when 2      => s_act <= "00011111";
                when 3      => s_act <= "00111111";
                when 4      => s_act <= "01111111";
                when others => s_act <= "11111111";
            end case;
            -- the Bold / Grey tone ramp of this Order and Step
            v_ri := s_stairs & s_zn;
            t_lo <= C_PG_LO(to_integer(v_ri));
            t_hi <= C_PG_HI(to_integer(v_ri));
            t_sl <= C_PG_SL(to_integer(v_ri));

            -- [B] shared hold + glide, five clocks per channel (X, Y, Scale, Camera, Fan, then the Drift speed):
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
                            when 3      => s_gm <= signed('0' & g_c); s_gr <= unsigned(registers_in(5));
                            when 4      => s_gm <= signed('0' & g_s); s_gr <= unsigned(registers_in(7));
                            when others => s_gm <= s_vgs;             s_gr <= unsigned(registers_in(4));
                        end case;
                    when 1 =>                      -- the gap to the knob (the speed is the knob about its centre)
                        v_tg := (others => '0');
                        v_tg(15 downto 6) := signed(s_gr);
                        if z_gc = 5 then v_tg(15) := not v_tg(15); v_tg(16) := not s_gr(9); end if;
                        s_gd <= v_tg - s_gm;
                        if s_gr = 0 or s_gr = 1023 then s_ge <= '1'; else s_ge <= '0'; end if;
                    when 2 =>
                        if s_gd >= 128 or s_gd <= -128 or (s_ge = '1' and s_gd /= 0) then s_gmv <= '1'; end if;
                    when 3 =>
                        if s_gmv = '0' then
                            s_gd <= (others => '0');
                        elsif s_glide = '0' or (s_gd <= 64 and s_gd >= -64) then
                            s_gmv <= '0';                                  -- the whole gap: it lands
                        elsif z_gc = 5 then
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
                            when 3      => g_c <= unsigned(v_st(15 downto 0));
                            when 4      => g_s <= unsigned(v_st(15 downto 0));
                            when others => s_vgs <= v_st;
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; q_go <= '1'; end if;
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
            s_st <= g_start & s_st(0 to 3);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] THE FIELD SEQUENCER.  Slots of 24 clocks; q_k = 0 holds the 16 global slots, q_k = 1 .. N the 16
    -- slots of family q_ck = q_k - 1.  Phase 0 pre-muxes the operands, 1 loads the multiplier, 2 .. 17
    -- iterate, the product is read at 19.  (One small operation per zone clock: the zone is quasi-static
    -- but timed at the pixel clock.)
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_17  : unsigned(16 downto 0);
        variable v_r   : unsigned(7 downto 0);
        variable v_s17 : signed(16 downto 0);
        variable v_d   : signed(3 downto 0);
        variable v_rt  : signed(6 downto 0);
        variable v_v   : signed(10 downto 0);
        variable v_n11 : signed(10 downto 0);
        variable v_q   : signed(9 downto 0);
        variable v_clr, v_add, v_sub, v_sg, v_cnd : std_logic;
        variable v_tadd, v_tsub : std_logic;
        variable v_top : unsigned(17 downto 0);
        variable v_tv  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            l_kick <= '0';
            if q_go = '1' then
                q_run <= '1'; q_k <= (others => '0'); q_ck <= (others => '0'); q_cut <= '0';
                q_j <= (others => '0'); q_ph <= (others => '0');
            elsif q_run = '1' then
                if q_ph = 23 then
                    q_ph <= (others => '0');
                    q_j <= q_j + 1;
                    if q_j = 15 then
                        q_k <= q_k + 1; q_ck <= q_k(2 downto 0); q_cut <= '1';
                        if q_k = s_n then q_run <= '0'; l_kick <= '1'; end if;
                    end if;
                else
                    q_ph <= q_ph + 1;
                end if;
            end if;

            -- operand pre-mux (phase 0): the multiplicand (signed, wide) and the multiplier (16 bits)
            if q_run = '1' and q_ph = 0 then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                if q_cut = '0' then
                    case to_integer(q_j) is
                        when 1 | 2 | 3 | 5 | 7 => z_fa <= signed(resize(z_g, 28));
                        when 4      => z_fa <= signed(resize(c_p, 28));
                        when 9      =>                                     -- pan x: 2 virtual px per count
                            v_s17 := signed('0' & g_x) - 32768;
                            z_fa <= resize(shift_right(v_s17, 2), 28);
                        when 10     => z_fa <= resize(z_yv, 28);           -- pan y
                        when others => null;
                    end case;
                    case to_integer(q_j) is
                        when 1      => z_fb <= resize(z_fr, 16);           -- exp interpolation
                        when 2      => z_fb <= f_cp(s_xs);                       -- period mantissa, Q9 px
                        when 3      => z_fb <= z_cio;                      -- 1 / period
                        when 4      => z_fb <= to_unsigned(21845, 16);     -- / 3
                        when 5      => z_fb <= g_c;                        -- camera step
                        when 7      => z_fb <= C_PG_KN(to_integer(s_n));   -- fan step
                        when 9 | 10 => z_fb <= f_sc(s_xs);
                        when others => null;
                    end case;
                else
                    case to_integer(q_j) is
                        when 1 | 2       => z_fa <= signed(resize(z_df, 28));   -- sine / cosine interpolation
                        when 3           => z_fa <= signed(resize(z_s10, 28));  -- period spread
                        when 4 | 7 | 12  => z_fa <= signed(resize(z_cm, 28));   -- pixel step; cos * |pan x|; steps per line * step
                        when 5 | 6 | 8   => z_fa <= signed(resize(z_sm, 28));   -- line step; sin * |pan y|
                        when 9 | 11 | 14 => z_fa <= signed(resize(c_p, 28));    -- drift phase * period; q * period
                        when 10 | 13     => z_fa <= z_x;                        -- seed / period, wrap / period
                        when 15          => z_fa <= signed(resize(z_spd, 28));  -- drift step
                        when others      => null;
                    end case;
                    v_d := C_PG_DELT(to_integer(q_ck));
                    v_rt := C_PG_RATE(to_integer(q_ck));
                    case to_integer(q_j) is
                        when 1 | 2   => z_fb <= resize(z_ff, 16);
                        when 3       =>
                            if v_d(3) = '1' then z_fb <= resize(unsigned(-v_d), 16); else z_fb <= resize(unsigned(v_d), 16); end if;
                        when 4 | 5   => z_fb <= z_r;
                        when 6       => z_fb <= resize(z_rq, 16);
                        when 7       => z_fb <= z_ax;
                        when 8       => z_fb <= z_ay;
                        when 9       => z_fb <= z_D(19 downto 4);
                        when 10 | 13 => z_fb <= c_invp;
                        when 11 | 14 => z_fb <= resize(z_qm & "000", 16);
                        when 12      => z_fb <= resize(z_m2 & "000", 16);
                        when 15      =>
                            if v_rt(6) = '1' then z_fb <= resize(unsigned(-v_rt), 16); else z_fb <= resize(unsigned(v_rt), 16); end if;
                        when others  => null;
                    end case;
                end if;
            end if;
            if q_ph = 1 then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 44); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(42 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- results and the small steps between the products
            cr_we <= '0';
            v_tadd := '0'; v_tsub := '0';                    -- the angle: z_th +- (step or half step)
            if q_run = '1' and q_cut = '0' then
                case to_integer(q_j) is
                    when 0 =>                      -- the period's octave and mantissa (2^(i/256) ROM, three reads)
                        case to_integer(q_ph) is
                            when 1 =>
                                r_ex_a <= g_p(13 downto 6); z_oct <= g_p(15 downto 14); z_fr <= g_p(5 downto 0);
                                if g_p(13 downto 6) = 255 then z_e255 <= '1'; else z_e255 <= '0'; end if;
                                if g_p(13 downto 6) = 0 then z_e000 <= '1'; else z_e000 <= '0'; end if;
                                z_cio <= f_ci(s_xs);
                            when 2 =>
                                r_ex_a <= r_ex_a + 1;
                                if z_oct /= 0 then z_cio <= shift_right(z_cio, 1); end if;
                            when 3 =>
                                if z_oct(1) = '1' then z_cio <= shift_right(z_cio, 1); end if;
                            when 4 =>
                                z_e0 <= r_exf;
                                if z_oct = 3 then z_cio <= shift_right(z_cio, 1); end if;
                            when 5 =>
                                if z_e255 = '1' then v_17 := to_unsigned(65536, 17); else v_17 := resize(r_exf, 17); end if;
                                z_g <= v_17 - resize(z_e0, 17);
                            when others => null;
                        end case;
                    when 1 =>
                        if q_ph = 19 then
                            v_r := unsigned(f_acc(13 downto 6)) + ("0000000" & f_acc(5));
                            z_g <= resize(z_e0, 17) + resize(v_r, 17);
                        end if;
                    when 2 =>                      -- the period = mantissa << (5 + octave); meanwhile the third read: 2^-(i/256)
                        if q_ph = 1 then
                            r_ex_a <= to_unsigned(0, 8) - g_p(13 downto 6);
                        elsif q_ph = 5 then
                            if z_e000 = '1' then z_g <= to_unsigned(65535, 17); else z_g <= resize(r_exf, 17); end if;
                        elsif q_ph = 19 then
                            case to_integer(z_oct) is
                                when 0      => c_p <= shift_left(resize(unsigned(f_acc(31 downto 16)), 24), 5);
                                when 1      => c_p <= shift_left(resize(unsigned(f_acc(31 downto 16)), 24), 6);
                                when 2      => c_p <= shift_left(resize(unsigned(f_acc(31 downto 16)), 24), 7);
                                when others => c_p <= shift_left(resize(unsigned(f_acc(31 downto 16)), 24), 8);
                            end case;
                        end if;
                    when 3 =>
                        if q_ph = 19 then c_invp <= unsigned(f_acc(31 downto 16)); end if;
                    when 4 =>                      -- the region's half width: P / 4 (Bold) or P / 12 (Fine), Q4
                        if q_ph = 19 then
                            if s_fine = '1' then c_t0 <= unsigned(f_acc(39 downto 28)); else c_t0 <= c_p(23 downto 12); end if;
                        elsif q_ph = 20 then
                            z_g <= resize(c_t0, 17);
                        end if;
                    when 5 =>                      -- its step per camera level (three steps reach the limit)
                        v_tv := unsigned(f_acc(27 downto 16));         -- (the product holds until the next slot)
                        if q_ph = 19 then
                            if s_fine = '1' then z_tsa <= shift_right(v_tv, 1) + shift_right(v_tv, 4);
                            else z_tsa <= shift_right(v_tv, 2) + shift_right(v_tv, 5); end if;
                        elsif q_ph = 20 then
                            if s_fine = '1' then c_ts <= resize(z_tsa + shift_right(v_tv, 6), 11);
                            else c_ts <= resize(z_tsa, 11); end if;
                        end if;
                    when 6 =>                      -- Fan: the slider with detents at 0, 50 and 100 %
                        case to_integer(q_ph) is
                            when 1 =>
                                if g_s < 512 then z_fz(0) <= '1'; else z_fz(0) <= '0'; end if;
                                if g_s < 32256 then z_fz(1) <= '1'; else z_fz(1) <= '0'; end if;
                                if g_s < 33280 then z_fz(2) <= '1'; else z_fz(2) <= '0'; end if;
                                if g_s < 65024 then z_fz(3) <= '1'; else z_fz(3) <= '0'; end if;
                            when 2 =>
                                if z_fz(1) = '1' then z_fv <= g_s - 512; else z_fv <= g_s - 33280; end if;
                            when 3 => z_fw <= z_fv + shift_right(z_fv, 5);
                            when 4 => z_fw <= z_fw + shift_right(z_fv, 10);
                            when 5 =>
                                if z_fz(0) = '1' then z_g <= (others => '0');
                                elsif z_fz(1) = '1' then z_g <= resize(z_fw, 17);
                                elsif z_fz(2) = '1' then z_g <= to_unsigned(32768, 17);
                                elsif z_fz(3) = '1' then z_g <= "01" & z_fw(14 downto 0);
                                else z_g <= to_unsigned(65536, 17); end if;
                            when 6 =>              -- the period spread fades out by 50 %
                                if z_g(16 downto 15) = "00" then
                                    v_17 := to_unsigned(32768, 17) - resize(z_g(14 downto 0), 17);
                                    z_s10 <= v_17(15 downto 5);
                                else
                                    z_s10 <= (others => '0');
                                end if;
                            when others => null;
                        end case;
                    when 7 =>
                        if q_ph = 19 then z_phi <= unsigned(f_acc(32 downto 15)); end if;
                    when 8 =>                      -- the first family's angle: - (N - 1) half steps
                        if q_ph = 0 then z_th <= (others => '0');
                        elsif q_ph < s_n then v_tadd := '1'; v_tsub := '1'; end if;
                    when 9 =>
                        case to_integer(q_ph) is
                            when 2 =>              -- pan y: 9/8 virtual px per count (9 g, unsigned: no sign bits to add)
                                z_y9 <= resize(g_y & "000", 20) + resize(g_y, 20);
                            when 3 =>              -- 4320 - 9 * 32768 / 64 = -288
                                z_yv <= signed(resize(z_y9(19 downto 6), 16)) - signed(resize(s_ys(11 downto 6), 16)) - 288;
                            when 19 => z_pv <= signed(resize(z_w44, 17)) + f_acc(31 downto 15);
                            when 20 =>
                                c_xn <= z_pv(16);
                                if z_pv(16) = '1' then z_ax <= unsigned(-z_pv(15 downto 0)); else z_ax <= unsigned(z_pv(15 downto 0)); end if;
                            when others => null;
                        end case;
                    when 10 =>
                        if q_ph = 19 then
                            z_pv <= f_acc(31 downto 15);
                        elsif q_ph = 20 then
                            c_yn <= z_pv(16);
                            if z_pv(16) = '1' then z_ay <= unsigned(-z_pv(15 downto 0)); else z_ay <= unsigned(z_pv(15 downto 0)); end if;
                        end if;
                    when 11 =>                     -- Drift (Hypnos SPEED): |v| < 24 still, else 2^-12 .. 2^-4 period per field
                        case to_integer(q_ph) is
                            when 1 =>
                                v_v := resize(shift_right(s_vgs, 6), 11);
                                v_n11 := -v_v;
                                z_vn <= v_v(10);
                                if v_v(10) = '1' then z_av <= unsigned(v_n11(9 downto 0)); else z_av <= unsigned(v_v(9 downto 0)); end if;
                            when 2 =>
                                if z_av < 24 then z_vz <= '1'; z_u <= (others => '0');
                                else z_vz <= '0'; z_u <= z_av - 24; end if;
                            when 3 => z_ls <= resize(z_u + shift_right(z_u, 5), 9);
                            when 4 => z_ls <= z_ls + resize(shift_right(z_u, 6), 9);
                            when 5 => r_ex_a <= z_ls(5 downto 0) & "00";
                            when 8 =>
                                if z_vz = '1' then z_spd <= (others => '0');
                                else z_spd <= resize(r_exf(15 downto 7), 16); end if;
                                z_scn <= z_ls(8 downto 6);
                            when 9 | 10 | 11 | 12 | 13 | 14 | 15 =>        -- the octave: a serial shift
                                if z_scn /= 0 then z_spd <= shift_left(z_spd, 1); z_scn <= z_scn - 1; end if;
                            when others => null;
                        end case;
                    when others => null;
                end case;
            elsif q_run = '1' then
                case to_integer(q_j) is
                    when 0 | 1 =>                  -- 0: the family's drift phase and its sine; 1: its cosine
                        case to_integer(q_ph) is
                            when 4 =>
                                if q_j(0) = '0' then z_D <= unsigned(cr_q(19 downto 0)); end if;
                            when 6 =>              -- quadrant fold (the cosine is the sine a quarter turn on)
                                if q_j(0) = '0' then
                                    z_sneg <= z_th(17);
                                    v_r(0) := z_th(16);
                                else
                                    z_cneg <= z_th(17) xor z_th(16);
                                    v_r(0) := not z_th(16);
                                end if;
                                if v_r(0) = '1' then z_p <= to_unsigned(65536, 17) - resize(z_th(15 downto 0), 17);
                                else z_p <= resize(z_th(15 downto 0), 17); end if;
                            when 7 =>
                                sn_a <= z_p(15 downto 6); z_ov <= z_p(16);
                                if q_j(0) = '0' then z_ff <= z_p(5 downto 0); end if;      -- (the cosine's is taken at 12:
                                if z_p(16) = '1' or z_p(15 downto 6) = 1023 then z_ov1 <= '1'; else z_ov1 <= '0'; end if;
                            when 8 =>
                                sn_a <= sn_a + 1;
                            when 10 =>
                                if q_j(0) = '0' then
                                    if z_ov = '1' then z_sm <= to_unsigned(16384, 15); else z_sm <= sn_f(14 downto 0); end if;
                                else
                                    if z_ov = '1' then z_cm <= to_unsigned(16384, 15); else z_cm <= sn_f(14 downto 0); end if;
                                end if;
                                z_lo <= sn_f(14 downto 0);
                            when 11 =>
                                if z_ov = '1' then z_df <= (others => '0');
                                elsif z_ov1 = '1' then z_df <= resize(to_unsigned(16384, 15) - z_lo, 6);
                                else z_df <= resize(sn_f(14 downto 0) - z_lo, 6); end if;
                            when 12 =>             --  the multiplier has both sine operands by now)
                                z_ff <= z_p(5 downto 0);
                            when 18 =>
                                if q_j(0) = '1' then v_tadd := '1'; end if;      -- the next family's angle
                            when 19 =>
                                if q_j(0) = '1' then
                                    z_sm <= z_sm + resize(unsigned(f_acc(11 downto 6)) + ("00000" & f_acc(5)), 15);
                                end if;
                            when others => null;
                        end case;
                    when 2 =>
                        if q_ph = 19 then z_cm <= z_cm + resize(unsigned(f_acc(11 downto 6)) + ("00000" & f_acc(5)), 15); end if;
                    when 3 =>                      -- distances scale by 1 + spread * delta (the period by its inverse)
                        if q_ph = 19 then
                            v_d := C_PG_DELT(to_integer(q_ck));
                            z_r <= f_addsub(to_unsigned(32768, 16), unsigned(f_acc(15 downto 0)), v_d(3));
                        end if;
                    when 4 =>                      -- the pixel step (the family is flipped so that it is >= 0)
                        if q_ph = 19 then z_cm <= unsigned(f_acc(29 downto 15)); end if;
                    when 5 =>
                        if q_ph = 19 then z_sm <= unsigned(f_acc(29 downto 15)); end if;
                    when 6 =>
                        if q_ph = 19 then
                            z_dy <= unsigned(f_acc(25 downto 10));
                        elsif q_ph = 20 then
                            z_dyn <= z_sneg xor z_cneg;
                            if s_il = '1' and z_dy >= 20480 then z_soft <= '1'; else z_soft <= '0'; end if;
                        end if;
                    when 7 | 8 | 9 | 12 => null;   -- - |cos| X, - +-sin Y, +- the drift; the line step less a line of pixel steps
                    when 10 | 13 =>
                        if q_ph = 19 then
                            v_q := f_acc(43 downto 34);
                            z_qn <= v_q(9);
                            if v_q(9) = '1' then v_q := -v_q; end if;
                            z_qm <= unsigned(v_q(8 downto 0));
                        end if;
                    when 11 | 14 =>                -- the remainder: x - q P, then into [-P, 0) (seed) or [0, P) (wrap)
                        if q_ph = 23 then
                            cr_we <= '1'; cr_wd <= resize(z_x, 32);
                            if q_j = 11 then cr_wa <= q_ck & "011"; else cr_wa <= q_ck & "010"; end if;
                        end if;
                    when others =>                 -- 15: the drift phase moves on; the table rows
                        case to_integer(q_ph) is
                            when 19 =>
                                v_rt := C_PG_RATE(to_integer(q_ck));
                                z_D <= f_addsub(z_D, resize(unsigned(f_acc(21 downto 5)), 20), z_vn xor v_rt(6));
                            when 20 =>
                                cr_we <= '1'; cr_wa <= q_ck & "000"; cr_wd <= resize(z_x, 32);           -- the step
                            when 21 => cr_we <= '1'; cr_wa <= q_ck & "001"; cr_wd <= resize(z_x, 32);    -- the step less a period
                            when 22 => cr_we <= '1'; cr_wa <= q_ck & "100"; cr_wd <= signed(resize(z_D, 32));
                            when 23 => c_soft(to_integer(q_ck)) <= z_soft;
                            when others => null;
                        end case;
                end case;
                -- the work register's operand, a phase ahead of its use
                if q_ph = 18 then
                    case to_integer(q_j) is
                        when 7 | 8 | 11 | 14 => z_o <= f_acc(30 downto 3);
                        when 9               => z_o <= f_acc(43 downto 16);
                        when 12              => z_o <= signed(resize(z_dy, 28));
                        when 15              => z_o <= signed(resize(z_cm, 28));
                        when others          => null;
                    end case;
                elsif q_ph = 19 then
                    case to_integer(q_j) is
                        when 11 | 14 | 15    => z_o <= signed(resize(c_p, 28));
                        when 12              => z_o <= f_acc(30 downto 3);
                        when others          => null;
                    end case;
                end if;
            end if;
            -- the work register: z_x <= 0, z_x +- z_o, or toward zero by a period (the remainder's range).  Its
            -- controls are decoded a clock ahead, so only the sign of z_x itself sits in front of the carry chain.
            v_clr := '0'; v_add := '0'; v_sub := '0'; v_sg := '0'; v_cnd := '0';
            if q_run = '1' and q_cut = '1' then
                case to_integer(q_j) is
                    when 7 =>
                        if q_ph = 17 then v_clr := '1';
                        elsif q_ph = 18 then v_add := '1'; v_sub := not c_xn; end if;
                    when 8 =>
                        if q_ph = 18 then v_add := '1'; v_sub := not (z_sneg xor c_yn xor z_cneg); end if;
                    when 9 =>
                        if q_ph = 18 then v_add := '1'; v_sub := z_cneg; end if;
                    when 11 | 14 =>                -- x - q P; two steps toward zero; a third if on the wrong side
                        if q_ph = 18 then v_add := '1'; v_sub := not z_qn;
                        elsif q_ph = 19 or q_ph = 20 then v_sg := '1';
                        elsif q_ph = 21 then v_cnd := '1'; end if;
                    when 12 =>
                        if q_ph = 17 then v_clr := '1';
                        elsif q_ph = 18 then v_add := '1'; v_sub := z_dyn;
                        elsif q_ph = 19 then v_add := '1'; v_sub := '1'; end if;
                    when 15 =>
                        if q_ph = 17 then v_clr := '1';
                        elsif q_ph = 18 then v_add := '1';
                        elsif q_ph = 19 then v_add := '1'; v_sub := '1'; end if;
                    when others => null;
                end case;
            end if;
            z_xclr <= v_clr; z_xadd <= v_add; z_xsub <= v_sub; z_xsg <= v_sg; z_xcnd <= v_cnd;
            if z_xsg = '1' or z_xcnd = '1' then v_sub := not z_x(27); else v_sub := z_xsub; end if;
            if z_xclr = '1' then
                z_x <= (others => '0');
            elsif z_xadd = '1' or z_xsg = '1' or (z_xcnd = '1' and z_x(27) /= q_j(0)) then
                z_x <= signed(f_addsub(unsigned(z_x), unsigned(z_o), v_sub));
            end if;
            if v_tsub = '1' then v_top := '0' & z_phi(17 downto 1); else v_top := z_phi; end if;
            if v_tadd = '1' then z_th <= f_addsub(z_th, v_top, v_tsub); end if;
        end if;
    end process p_seq;

    ---------------------------------------------------------------------------
    -- [F] the tone table, refilled every field: 512 entries (index = the summed coverage, 16 per region),
    -- piecewise linear with a new slope every 16 entries.
    --   Ink:         a triangle (the parity), Bold starts white, Fine starts black
    --   Grey / Fine: films "over" at 50 %: the step halves per film (Stairs: per three thirds of a film)
    --   Grey / Bold: white down to black between the levels lo and hi of this Order
    ---------------------------------------------------------------------------
    p_tone : process(clk)
        variable v_s  : unsigned(4 downto 0);
        variable v_a  : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            tn_we <= '0';
            if q_go = '1' then
                t_run <= '1'; t_i <= (others => '0'); t_ph <= '0';
            elsif t_run = '1' then
                if t_ph = '0' then
                    v_s := t_i(8 downto 4);
                    if t_i(3 downto 0) = 0 then
                        if s_ink = '1' then
                            if (v_s(0) xor s_fine) = '1' then t_acc <= to_unsigned(C_BLACK, 17); t_step <= to_signed(3504, 13);
                            else t_acc <= to_unsigned(C_WHITE, 17); t_step <= to_signed(-3504, 13); end if;
                        elsif s_fine = '1' then
                            if v_s = 0 then
                                t_acc <= to_unsigned(C_BLACK, 17);
                                if s_stairs = '1' then
                                    t_step <= to_signed(723, 13);
                                    t_fa <= to_unsigned(574, 11); t_fb <= to_unsigned(455, 11); t_fc <= to_unsigned(361, 11);
                                else
                                    t_step <= to_signed(1752, 13);
                                    t_fa <= to_unsigned(876, 11);
                                end if;
                            else
                                t_step <= signed(resize(t_fa, 13));
                                if s_stairs = '1' then
                                    t_fa <= t_fb; t_fb <= t_fc; t_fc <= shift_right(t_fa, 1);
                                else
                                    t_fa <= shift_right(t_fa, 1);
                                end if;
                            end if;
                        else
                            if v_s < t_lo then t_acc <= to_unsigned(C_WHITE, 17); t_step <= (others => '0');
                            elsif v_s >= t_hi then t_acc <= to_unsigned(C_BLACK, 17); t_step <= (others => '0');
                            elsif v_s = t_lo then t_acc <= to_unsigned(C_WHITE, 17); t_step <= -signed(resize(t_sl, 13));
                            else t_step <= -signed(resize(t_sl, 13)); end if;
                        end if;
                    end if;
                    t_ph <= '1';
                else
                    v_a := t_acc + 32;
                    tn_we <= '1'; tn_wa <= t_i; tn_wd <= resize(v_a(15 downto 6), 16);
                    t_acc <= unsigned(signed(t_acc) + resize(t_step, 17));
                    t_i <= t_i + 1;
                    t_ph <= '0';
                    if t_i = 511 then t_run <= '0'; end if;
                end if;
            end if;
        end if;
    end process p_tone;

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
    -- [E] FAMILIES.  Eight affine DDAs in pixel widths (Q10.14) that live in [-P, 0): each adds its pixel
    -- step on every clock of the line window (W + 8 steps), or the step less a period on the clock after it
    -- has reached 0, so it is the distance modulo the period, exactly (the one late pixel sits half a period
    -- from every band axis, where no edge can be).  At a line end the loader puts the line wrap (the line step
    -- less the pixel steps of a line, reduced modulo the period by the sequencer) through the "less a period"
    -- register for one add, then two ordinary steps bring the sum back into range.  The kick (end of the field
    -- sequencer) clears the accumulators and adds the field seed the same way.
    ---------------------------------------------------------------------------
    p_load : process(clk)
        variable v_k : unsigned(2 downto 0);
        variable v_p : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            -- line window: W + 8 steps
            b_end <= '0';
            if s_st(4) = '1' then
                b_run <= '1'; b_cnt <= to_unsigned(4088, 12);
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = g_wm1 then b_run <= '0'; b_end <= '1'; end if;
            end if;

            -- loader: three table reads per family (seed or wrap, step less a period, step) and a spare clock
            ca_rst <= l_kick;
            if l_kick = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '1';
            elsif b_end = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '0';
            elsif l_run = '1' then
                l_t <= l_t + 1;
                if l_t = 31 then l_run <= '0'; end if;
            end if;
            v_k := l_t(4 downto 2); v_p := l_t(1 downto 0);
            if q_run = '1' then
                if q_j = 15 then cr_ra <= q_ck & "111"; else cr_ra <= q_ck & "100"; end if;      -- the sequencer reads the drift phase
            elsif v_p = 0 then
                if l_seed = '1' then cr_ra <= v_k & "011"; else cr_ra <= v_k & "010"; end if;
            elsif v_p = 1 then cr_ra <= v_k & "001";
            else cr_ra <= v_k & "000"; end if;
            -- the word addressed now is in cr_q two clocks on, and goes into a register of its family then:
            -- seed / wrap (ldp), then the add of it while the step less a period is restored (pk), then the
            -- step (ld), then two ordinary steps (nm; not after the seed, which is in range as it is)
            l_v1 <= l_run; l_c1 <= v_k; l_p1 <= v_p;
            l_v2 <= l_v1;  l_c2 <= l_c1; l_p2 <= l_p1;
            for k in 0 to 7 loop
                if l_v1 = '1' and l_c1 = k and l_p1 = 0 then ca_ldp(k) <= '1'; else ca_ldp(k) <= '0'; end if;
                if l_v1 = '1' and l_c1 = k and l_p1 = 1 then ca_pk(k) <= '1'; else ca_pk(k) <= '0'; end if;
                if l_v1 = '1' and l_c1 = k and l_p1 = 2 then ca_ld(k) <= '1'; else ca_ld(k) <= '0'; end if;
                if l_seed = '0' and ((l_v1 = '1' and l_c1 = k and l_p1 = 3) or (l_v2 = '1' and l_c2 = k and l_p2 = 3)) then
                    ca_nm(k) <= '1';
                else
                    ca_nm(k) <= '0';
                end if;
            end loop;
        end if;
    end process p_load;

    g_fam : for k in 0 to 7 generate
        p_fam : process(clk)
            variable v_op : signed(24 downto 0);
            variable v_l : std_logic;
            variable v_s : unsigned(6 downto 0);
            variable v_dd : signed(5 downto 0);
        begin
            if rising_edge(clk) then
                if ca_ldp(k) = '1' or ca_pk(k) = '1' then ca_incw(k) <= cr_q(24 downto 5); end if;
                if ca_ldp(k) = '1' or ca_ld(k) = '1' then ca_inc(k)(4 downto 0) <= unsigned(cr_q(4 downto 0)); end if;
                if ca_ld(k) = '1' then ca_inc(k)(14 downto 5) <= unsigned(cr_q(14 downto 5)); end if;
                -- (the period is a multiple of 32, so the two steps share their low five bits)
                v_op := signed(resize(ca_inc(k), 25));
                if ca_acc(k)(24) = '0' or ca_pk(k) = '1' then v_op(24 downto 5) := ca_incw(k); end if;
                if ca_rst = '1' then
                    ca_acc(k) <= (others => '0');
                elsif ca_pk(k) = '1' or b_run = '1' or ca_nm(k) = '1' then
                    ca_acc(k) <= ca_acc(k) + v_op;
                end if;
                -- stage 1: the distance from the band axis, Q4 (-P/2 .. P/2)
                x1_u(k) <= ca_acc(k)(24 downto 10) + signed(resize(c_p(23 downto 11), 15));
                -- stage 2: the two edges, e = T - |u|
                x2_ea(k) <= signed(f_addsub(unsigned(x_ta), unsigned(x1_u(k)(13 downto 0)), not x1_u(k)(14)));
                x2_eb(k) <= signed(f_addsub(unsigned(x_tb), unsigned(x1_u(k)(13 downto 0)), not x1_u(k)(14)));
                x2_sg(k) <= x1_u(k)(14);
                -- stage 3: their coverages
                x3_ca(k) <= f_cov(x2_ea(k), c_soft(k));
                x3_cb(k) <= f_cov(x2_eb(k), c_soft(k));
                x3_sg(k) <= x2_sg(k);
                -- stage 4: the family's value.  Flat: the region (B is off).  Stairs: the sawtooth 3, 2, 1, 0 as
                -- 3 B - A on the falling side and A + B on the other; Ink needs only the parity, A + B.
                -- (with d = B - A that is 2 B + d and 2 B - d)
                v_l := s_stairs and (not s_ink) and x3_sg(k);
                v_dd := signed(resize(x3_cb(k), 6)) - signed(resize(x3_ca(k), 6));
                if s_act(k) = '0' then
                    x4_s(k) <= (others => '0');
                else
                    x4_s(k) <= f_addsub(resize(x3_cb(k) & '0', 7), unsigned(resize(v_dd, 7)), not v_l);
                end if;
            end if;
        end process p_fam;
    end generate g_fam;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice, x 4 about its middle) -> box of 32 px (16 px at 720p and SD),
    -- edge-replicated and reloaded at every line start -> vertical one-pole (1/16 per line at 1080p, 1/8 at
    -- 720p / 1080i / 480p / 576p, 1/4 at 480i / 576i) at half horizontal resolution.
    -- CAMERA GRID: on every line the one-pole's value at each 34 x 34 virtual-px block's centre column (plus
    -- half the box) is written to the block's cell of the write bank; the last line of a block row stays.
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

            -- camera grid: block centres along the line (h units: 68 per block), block rows down the field
            cm_we <= '0';
            if s_zone = '1' then
                s_row <= (others => '0'); s_qy <= (others => '0');
                s_bc <= (others => '0'); s_qx <= f_qxi(s_xs);
            elsif lb_we = '1' then
                s_lw <= lb_wd(15 downto 8);
                if s_qx(7) = '0' then
                    cm_we <= '1'; cm_wa <= s_wb & s_row & s_bc; cm_wd <= lb_wd(15 downto 8);
                    s_qx <= s_qx + signed(resize(s_xs & '0', 8)) - 68;
                    s_bc <= s_bc + 1;
                else
                    s_qx <= s_qx + signed(resize(s_xs & '0', 8));
                end if;
            elsif s_v7 = '0' and s_v7d = '1' then          -- line end: the pending block takes the last value
                cm_we <= '1'; cm_wa <= s_wb & s_row & s_bc; cm_wd <= s_lw;
                s_bc <= (others => '0'); s_qx <= f_qxi(s_xs);
                v_qy := s_qy + resize(s_ys, 14);
                if v_qy >= 8704 then
                    s_qy <= v_qy - 8704;
                    if s_row /= 31 then s_row <= s_row + 1; end if;
                else
                    s_qy <= v_qy;
                end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] the pixel path's side of the camera grid: the block of the pixel, its sample of the last field
    -- (registered, then quantised to four levels and HELD across the block), and the region half width there:
    -- Bold T = T0 - level * step (the gaps close), Fine T = T0 + level * step (the lines widen).
    -- The block address of DDA pixel i is issued at T + 1 + i; the thresholds are valid at stage 1.
    ---------------------------------------------------------------------------
    p_cam : process(clk)
        variable v_n, v_m : signed(7 downto 0);
        variable v_qy : unsigned(13 downto 0);
        variable v_t2 : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                d_x <= -signed(resize(s_xs, 8)); d_col <= (others => '0'); d_nb <= '1';
            else
                v_n := d_x + signed(resize(s_xs, 8));
                v_m := v_n - 68;
                if v_m(7) = '0' then d_x <= v_m; d_col <= d_col + 1; d_nb <= '1';
                else d_x <= v_n; d_nb <= '0'; end if;
            end if;
            if s_zone = '1' then
                d_qy <= (others => '0'); d_row <= (others => '0');
            elsif b_end = '1' then
                v_qy := d_qy + resize(s_ys, 14);
                if v_qy >= 8704 then
                    d_qy <= v_qy - 8704;
                    if d_row /= 31 then d_row <= d_row + 1; end if;
                else
                    d_qy <= v_qy;
                end if;
            end if;
            cm_f <= cm_q;
            d_nb1 <= d_nb; d_nb2 <= d_nb1;
            if d_nb2 = '1' then                    -- levels at 72, 104 and 136 of 219 (white)
                if cm_f < 72 then d_q <= "00";
                elsif cm_f < 104 then d_q <= "01";
                elsif cm_f < 136 then d_q <= "10";
                else d_q <= "11"; end if;
            end if;
            case d_q is
                when "00"   => x_qts <= (others => '0');
                when "01"   => x_qts <= resize(c_ts, 13);
                when "10"   => x_qts <= resize(c_ts & '0', 13);
                when others => x_qts <= resize(c_ts & '0', 13) + resize(c_ts, 13);
            end case;
            x_t <= f_addsub(resize(c_t0, 13), x_qts, not s_fine);
            v_t2 := shift_right(x_t, 1);
            if s_stairs = '1' then
                x_ta <= signed(resize(v_t2, 14));
                x_tb <= signed(resize(x_t, 14) + resize(v_t2, 14));
            else
                x_ta <= signed(resize(x_t, 14));
                x_tb <= to_signed(-32, 14);
            end if;
        end if;
    end process p_cam;

    ---------------------------------------------------------------------------
    -- [F] SUM AND TONE (stages 5 .. 9)
    ---------------------------------------------------------------------------
    p_sum : process(clk)
    begin
        if rising_edge(clk) then
            for p in 0 to 3 loop
                x5_s(p) <= resize(x4_s(2 * p), 8) + resize(x4_s(2 * p + 1), 8);
            end loop;
            x6_sa <= resize(x5_s(0), 9) + resize(x5_s(1), 9);
            x6_sb <= resize(x5_s(2), 9) + resize(x5_s(3), 9);
            x7_s <= x6_sa + x6_sb;
            -- 8: the table's own register; 9: its fabric copy
            s_y23 <= tn_q(9 downto 0);
        end if;
    end process p_sum;

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

end architecture pentagrid;
