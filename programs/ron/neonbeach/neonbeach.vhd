-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- NEON BEACH v0.1.0 -- Pixel Beach's side-on shorebreak drawn by Neonscape's wire engine.
--
-- A cross-section of a beach: sky with a sun or moon, clouds, gulls and stars; under the horizon a strip of
-- distant sea that mirrors them; in front of it the water cut through like a glass tank, its layers following
-- the surface; under everything the sand, a mesh of quads.  Every pixel gets a REGION ID (object + facet), a
-- wire is drawn wherever the ID differs from a neighbour (left, or the line above: one line store), and a
-- region is filled from its own colour ramp when the fill is on (Neonscape's engine).
--
--   sky half (rows above the horizon, d = rows above it)
--     sky            the colour follows the sun: two per-field keys (zenith, horizon) lerped from eight
--                    keyframes by the sun's elevation (computed by running the colour path in vblank)
--     sun / moon     half-chord table, radius 128, raised and set by the slider; slits roll down the sun,
--                    a second disc bites the moon into a crescent
--     clouds, gulls  span-coded sprites in drifting lattice cells (slot A); gulls roost when it is dark
--     stars          hashed cells, twinkling; more of them as the sun goes down, all of them at night
--   sea strip       every line under the horizon redraws the sky objects at the same distance (a mirror),
--                    dimmed and broken by drifting ripple gaps
--   water           between the surface and the sea bed, both read per pixel from a knot table (block RAM,
--                    16 virtual px a knot, lerped): layers at k / 8 of the local depth (fewer where it is
--                    shallow), columns every 64 px.  v0.1.0: a flat surface at the Tide level
--   sand            under the bed line: cells 32 px wide and 32 rows deep, the rows parallel to the sand's
--                    surface
--   crab            span-coded, folded about its centre line (slot B)
--
--   K1 Rhythm  K2 Tide  K3 Foam  K4 Chop  K5 Swell  K6 Camera
--   S7 Fill (Wires / Fill)  S8 Colour (Neon / Ice)  S9 Crab  S10 Sky (Day / Night)  S11 Source
--   P12 Sun    elevation of the sun or moon; the sky, the palette and the stars follow it
--   (v0.1.0: Tide and the controls below K5 are live; Rhythm, Foam, Chop and Swell come with the waves)
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  One latency
-- (33) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).  Chassis ([A] timing, virtual
-- 1080-line coordinates, generated avid, output conditioning) from neonscape / riley / outrun.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.neonbeach_pkg.all;
use work.all;

architecture neonbeach of program_top is
    constant C_LAT : integer := 33;   -- = processing_delay_clks

    constant C_HZ    : integer := 356;      -- the horizon (virtual row)
    constant C_SL0   : integer := 626;      -- still sea level at Tide 50 %
    constant C_SUNX  : integer := 240;      -- sun / moon centre, virtual px right of the screen centre
    constant C_CRX   : integer := 560;      -- crab centre, virtual px from the canvas's left edge (v0.1.0: fixed)
    constant C_CRY   : integer := 830;      -- crab top row
    constant C_DEF_P12 : integer := 614;

    constant C_CM1 : std_logic_vector(15 downto 0) := x"5A93";   -- clouds
    constant C_BM  : std_logic_vector(15 downto 0) := x"1242";   -- gulls
    -- sheen: which pulses show, by (pulse number, object): about three in five
    constant C_SHG : std_logic_vector(0 to 63) := x"B6D5_E39A_6F4B_D2ED";

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
    function f_max4(a, b : unsigned(3 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;
    -- signed 8-bit delta times one bit of t, shifted: a gated operand for the colour lerp
    function f_gate(d : signed(7 downto 0); b : std_logic; sh : natural) return signed is
        variable v : signed(13 downto 0);
    begin
        if b = '1' then v := shift_left(resize(d, 14), sh); else v := (others => '0'); end if;
        return v;
    end function;
    -- a signed 11-bit knot difference times one bit of the fraction, shifted (the surface / bed lerp)
    function f_gk(d : signed(10 downto 0); b : std_logic; sh : natural) return signed is
        variable v : signed(14 downto 0);
    begin
        if b = '1' then v := shift_left(resize(d, 15), sh); else v := (others => '0'); end if;
        return v;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(12 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_th2, s_th3, s_tv2 : std_logic := '0';     -- wire thickness by mode
    signal w_x0   : signed(13 downto 0);                -- the left edge, half units from the centre

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
    signal s_st       : std_logic_vector(1 to 16) := (others => '0');   -- s_st(k) = g_start delayed k

    -- [B] controls
    signal s_fsw, s_theme, s_croff, s_night, s_src : std_logic := '0';
    signal s_k2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_gs : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);    -- sun glide (Q3)
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);         -- camera glide
    signal s_gp : unsigned(12 downto 0) := (others => '0');                   -- fill glide
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(1 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- [C] per-field values
    signal c_t    : unsigned(15 downto 0) := (others => '0');     -- field counter
    signal c_e    : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);   -- sun / moon elevation
    signal c_sunc : signed(11 downto 0) := to_signed(70, 12);     -- its centre, rows above the horizon
    signal c_sl   : unsigned(10 downto 0) := to_unsigned(C_SL0, 11);      -- sea level row
    signal c_fp   : unsigned(13 downto 0) := (others => '0');     -- fill position (Q3)
    signal c_cgn  : unsigned(3 downto 0) := (others => '0');      -- camera gain
    signal c_sa1, c_sab : unsigned(13 downto 0) := (others => '0');   -- cloud and gull drift
    signal c_tod  : unsigned(1 downto 0) := "00";                 -- 0 day, 1 low sun, 2 night
    signal c_pal  : unsigned(2 downto 0) := "000";                -- theme & time of day
    signal c_sden : unsigned(1 downto 0) := "00";                 -- star density
    signal c_hi, c_mid, c_lo : std_logic := '0';
    signal c_ebird, c_esun, c_emoon, c_ecrab : std_logic := '0';
    -- the sky keys: the colour path is run on two made-up pixels in vblank (zenith, horizon keyframes lerped
    -- by the elevation) and the results are kept as the sky ramp's c0 and delta
    signal c_inj  : std_logic := '0';
    signal c_injk : std_logic_vector(1 downto 0) := "00";
    signal c_injent : unsigned(4 downto 0) := (others => '0');
    signal c_injt : unsigned(5 downto 0) := (others => '0');
    signal k_zy, k_zu, k_zv, k_hy, k_hu, k_hv : unsigned(7 downto 0) := x"40";
    signal l_skyc : std_logic_vector(47 downto 0) := (others => '0');

    -- [C] line ROM / colour ROM / line store
    signal lr_a  : unsigned(9 downto 0) := (others => '0');
    signal lr_q, lr_qf : unsigned(15 downto 0) := (others => '0');
    signal pal_a : unsigned(7 downto 0) := (others => '0');
    signal pal_q, pal_qf : unsigned(47 downto 0) := (others => '0');
    type t_lb is array (0 to 2047) of std_logic_vector(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q, lb_qf : std_logic_vector(15 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_wa : unsigned(10 downto 0) := (others => '0');
    signal lb_wd : std_logic_vector(15 downto 0) := (others => '0');

    -- [C] knot table: per knot (16 virtual px), the surface row, the step to the next knot's, the bed row,
    -- its step, the foam.  The filler writes it in vblank; the pixel path reads it in active video.
    type t_kr is array (0 to 255) of std_logic_vector(47 downto 0);
    signal kr_ram : t_kr := (others => (others => '0'));
    attribute no_rw_check of kr_ram : signal is true;
    signal kr_a  : unsigned(7 downto 0) := (others => '0');
    signal kr_q, kr_qf : std_logic_vector(47 downto 0) := (others => '0');
    signal kr_we : std_logic := '0';
    signal kr_wa : unsigned(7 downto 0) := (others => '0');
    signal kr_wd : std_logic_vector(47 downto 0) := (others => '0');

    -- [C] filler (v0.1.0: a flat surface at the sea level)
    signal f_run, f_first : std_logic := '0';
    signal f_k   : unsigned(6 downto 0) := (others => '0');
    signal f_st  : unsigned(1 downto 0) := (others => '0');
    signal f_bq, f_b, f_s, f_bn, f_sn : unsigned(9 downto 0) := (others => '0');

    -- [C] line engine
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_lc   : unsigned(6 downto 0) := (others => '1');
    signal e_ln   : unsigned(1 downto 0) := (others => '0');
    signal e_s    : signed(11 downto 0) := (others => '0');
    signal e_d    : unsigned(9 downto 0) := (others => '0');
    signal e_ab, e_mir, e_gq, e_kick : std_logic := '0';
    signal e_sa, e_sb : unsigned(7 downto 0) := (others => '0');
    signal e_tsun : unsigned(6 downto 0) := (others => '0');
    signal e_tb   : unsigned(5 downto 0) := (others => '0');
    signal e_asel : std_logic := '0';
    signal e_dyu  : signed(11 downto 0) := (others => '0');
    signal e_c1, e_bd : signed(10 downto 0) := (others => '0');
    signal e_cr   : signed(11 downto 0) := (others => '0');
    signal e_rm   : unsigned(8 downto 0) := (others => '0');
    signal e_aa, e_ba : unsigned(9 downto 0) := (others => '0');
    signal e_aen, e_ben, e_son, e_bon : std_logic := '0';
    signal e_asc  : unsigned(13 downto 0) := (others => '0');
    signal e_x0   : signed(13 downto 0) := (others => '0');
    signal e_fc   : unsigned(13 downto 0) := (others => '0');

    -- this line's constants (written by the line engine in blanking) and the next line's start values
    signal l_d   : unsigned(9 downto 0) := (others => '0');
    signal l_row : unsigned(10 downto 0) := (others => '0');
    signal l_mir, l_ab : std_logic := '0';
    signal l_sw, l_bw : unsigned(8 downto 0) := (others => '0');
    signal l_son, l_bon : std_logic := '0';
    signal l_tsun : unsigned(6 downto 0) := (others => '0');
    signal l_tsky : unsigned(5 downto 0) := (others => '0');
    type t_row is array (0 to 3) of unsigned(15 downto 0);
    signal l_arow, l_brow : t_row := (others => (others => '0'));
    signal l_asel : std_logic := '0';
    signal l_ta, l_tb : unsigned(6 downto 0) := (others => '0');
    signal l_fill : std_logic_vector(7 downto 0) := (others => '0');
    signal l_rip, l_v1, l_v2 : std_logic := '0';
    signal l_oy : unsigned(6 downto 0) := (others => '0');
    signal l_od, l_pc : unsigned(13 downto 0) := (others => '0');             -- sheen: per-line phase terms
    signal n_xa0, n_xw0, n_pa0 : unsigned(13 downto 0) := (others => '0');
    signal n_xb0 : signed(13 downto 0) := (others => '0');
    signal n_x0  : signed(12 downto 0) := (others => '0');

    -- [D] source (camera luma or the Osc lattice), smoothed and scaled
    signal s_cin : unsigned(5 downto 0) := (others => '0');
    signal s_cf  : unsigned(9 downto 0) := (others => '0');
    signal s_cg  : unsigned(5 downto 0) := (others => '0');      -- the gained source, 0 .. 59

    -- [C] beam counters
    signal b_xw, b_xa, b_pa : unsigned(13 downto 0) := (others => '0');
    signal b_xb : signed(13 downto 0) := (others => '0');
    signal b_x  : signed(12 downto 0) := (others => '0');
    signal b_px : unsigned(10 downto 0) := (others => '0');
    signal s18_np : unsigned(8 downto 0) := (others => '0');
    signal s18_ix, s18_wo : unsigned(2 downto 0) := (others => '0');
    signal s18_ng : std_logic := '0';
    signal s19_n6 : unsigned(5 downto 0) := (others => '0');

    -- water and sand (stages 2 .. 14)
    type t_f4 is array (2 to 5) of unsigned(3 downto 0);
    signal w_f   : t_f4 := (others => (others => '0'));
    type t_x7 is array (2 to 10) of unsigned(6 downto 0);
    signal w_xb  : t_x7 := (others => (others => '0'));     -- column parities (64 px, 32 px) & the sand x (5)
    signal w5_ps1, w5_ps2 : signed(14 downto 0) := (others => '0');
    signal w5_s0, w5_b0 : unsigned(9 downto 0) := (others => '0');
    signal w5_db : signed(7 downto 0) := (others => '0');
    signal w6_ps, w6_pb1, w6_pb2, w7_pb : signed(14 downto 0) := (others => '0');
    signal w6_s0, w6_b0, w7_b0 : signed(11 downto 0) := (others => '0');
    signal w7_srf, w8_srf, w8_bed, w8_n : signed(11 downto 0) := (others => '0');
    signal w9_n, w9_dd, w9_sd : unsigned(9 downto 0) := (others => '0');
    signal w9_inw, w9_ins : std_logic := '0';
    type t_r10 is array (10 to 12) of unsigned(9 downto 0);
    signal wd_r, wd_d : t_r10 := (others => (others => '0'));
    type t_q3 is array (10 to 12) of unsigned(2 downto 0);
    signal wd_q : t_q3 := (others => (others => '0'));
    type t_t7 is array (10 to 13) of unsigned(6 downto 0);
    signal wd_wt, wd_st : t_t7 := (others => (others => '0'));
    type t_s3 is array (10 to 13) of std_logic_vector(2 downto 0);
    signal wd_ss : t_s3 := (others => (others => '0'));       -- sand facets: column, row, diagonal
    signal wd_inw, wd_ins, wd_cp : std_logic_vector(10 to 13) := (others => '0');
    signal w13_lay : std_logic := '0';
    signal w14_in, s14_in : std_logic := '0';
    signal w14_sub : std_logic_vector(1 downto 0) := "00";
    signal s14_sub : std_logic_vector(2 downto 0) := "000";
    signal w14_t, s14_t : unsigned(6 downto 0) := (others => '0');

    -- sun, stars
    signal s12_xr, s12_xr2 : signed(12 downto 0) := (others => '0');
    signal s12_ax, s12_ax2 : unsigned(11 downto 0) := (others => '0');
    signal u14_in : std_logic := '0';
    signal t12_h, t13_h : unsigned(11 downto 0) := (others => '0');
    signal t12_x, t13_x : unsigned(2 downto 0) := (others => '0');
    signal t14_on : std_logic := '0';
    signal t14_tw : unsigned(1 downto 0) := (others => '0');
    -- the sky's ramp positions, registered ahead of the merge
    signal k14_skt : unsigned(5 downto 0) := (others => '0');
    signal k14_cg2 : unsigned(6 downto 0) := (others => '0');
    signal k14_cam : std_logic := '0';

    -- sprite slots
    signal a12_xs, b12_xs : unsigned(7 downto 0) := (others => '0');
    signal a12_pr, b12_pr : std_logic := '0';
    signal a13_g, b13_g : std_logic_vector(5 downto 0) := (others => '0');
    signal a13_pr, b13_pr : std_logic := '0';
    signal a14_z, b14_z : unsigned(1 downto 0) := (others => '0');

    -- merge, edges, colour
    signal s15_id : std_logic_vector(7 downto 0) := (others => '0');
    signal s15_ramp : unsigned(2 downto 0) := (others => '0');
    signal s15_t : unsigned(6 downto 0) := (others => '0');
    signal s15_fon, s15_sky : std_logic := '0';
    signal s15_ovl : unsigned(1 downto 0) := (others => '0');
    signal s15_tw : unsigned(1 downto 0) := (others => '0');
    signal i_d1, i_d2, i_d3 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_g15 : std_logic_vector(3 downto 0) := (others => '0');
    signal s16_e : std_logic := '0';
    signal s16_m1, s16_m2, s16_m3 : unsigned(3 downto 0) := (others => '0');
    signal s16_ramp : unsigned(2 downto 0) := (others => '0');
    signal s16_t : unsigned(6 downto 0) := (others => '0');
    signal s16_fon, s16_sky : std_logic := '0';
    signal s16_ovl, s16_tw : unsigned(1 downto 0) := (others => '0');
    signal s17_ent : unsigned(4 downto 0) := (others => '0');
    signal s17_pn  : unsigned(2 downto 0) := (others => '0');
    signal s17_t6, s18_t6, s19_t6, s20_t6 : unsigned(5 downto 0) := (others => '0');
    signal s17_blk, s18_blk, s19_blk, s20_blk, s21_blk, s22_blk, s23_blk : std_logic := '1';
    signal s17_w, s18_w, s19_w, s20_w, s21_w, s22_w, s23_w : std_logic := '0';
    signal s17_sky, s18_sky, s19_sky : std_logic := '0';
    signal s17_dim, s18_dim, s19_dim, s20_dim, s21_dim, s22_dim : std_logic := '0';
    type t_ik is array (17 to 23) of std_logic_vector(1 downto 0);
    signal s_ik : t_ik := (others => "00");
    signal s24_y, s24_u, s24_v, h_u, h_v : unsigned(9 downto 0) := (others => '0');
    signal h_c : unsigned(2 downto 0) := (others => '0');
    type t_s3p is array (0 to 2) of signed(13 downto 0);
    signal s21_pa, s21_pb, s21_pc : t_s3p := (others => (others => '0'));
    type t_c3 is array (0 to 2) of unsigned(7 downto 0);
    signal s21_c0 : t_c3 := (others => (others => '0'));
    type t_r3 is array (0 to 2) of signed(10 downto 0);
    signal s22_s : t_r3 := (others => (others => '0'));
    signal s23_y, s23_u, s23_v : unsigned(9 downto 0) := (others => '0');

    -- [F] output conditioning
    -- the chroma filter's 8-pixel delay: a circular buffer in one block RAM (U and V are 8-bit here)
    type t_cr is array (0 to 255) of std_logic_vector(15 downto 0);
    signal cr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_q, cr_qf : std_logic_vector(15 downto 0) := x"8080";
    signal cr_wp : unsigned(7 downto 0) := to_unsigned(6, 8);
    signal cr_rp : unsigned(7 downto 0) := to_unsigned(0, 8);
    -- and the luma's first five pixels of delay, in a second one
    signal yr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of yr_ram : signal is true;
    signal yr_q : std_logic_vector(15 downto 0) := x"0040";
    signal yr_rp : unsigned(7 downto 0) := to_unsigned(3, 8);
    signal s_bu, s_bv2, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4100, 13);
    type t_y5 is array (5 to 7) of unsigned(9 downto 0);
    signal s_yd : t_y5 := (others => to_unsigned(64, 10));
    signal s_hist : std_logic_vector(7 downto 0) := (others => '0');
    signal o1_ya : unsigned(11 downto 0) := (others => '0');
    signal o1_yb, o2_y, o2_u, o2_v, o3_y, o3_u, o3_v, o4_y, o4_u, o4_v : unsigned(9 downto 0) := (others => '0');
begin

    -- NOTE: the order of the processes below is deliberate.  yosys abc9 (with ABC's &mfs) hits an assert
    -- ("Boxes are not in a topological order", then count_id) depending only on the order cells reach
    -- ABC; this order passes all six configs.  After any edit run .neonbeach_work/check6.sh, and if a
    -- config fails, try another order with .neonbeach_work/abc9_order.py.

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + the per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : unsigned(9 downto 0);
        variable v_s  : signed(11 downto 0);
        variable v_c  : signed(11 downto 0);
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
            elsif s_zc /= 8191 then
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

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_fsw   <= registers_in(6)(0);
                s_theme <= registers_in(6)(1);
                s_croff <= registers_in(6)(2);
                s_night <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_k2    <= unsigned(registers_in(1));
                s_xs    <= s_xsr;
                -- wire thickness: 3 x 2 at 1080p, 2 x 2 at 720p, 3 x 1 at 1080i, 1 x 1 at SD
                if s_xsr = 2 then s_th2 <= '1'; s_th3 <= '1';
                elsif s_xsr = 3 then s_th2 <= '1'; s_th3 <= '0';
                else s_th2 <= '0'; s_th3 <= '0'; end if;
                if s_ys <= 384 then s_tv2 <= '1'; else s_tv2 <= '0'; end if;
            end if;
            g_wm1 <= s_w - 1;

            -- [B] shared glide (1/8 of the gap per field, +-2 deadband): fill (to the Fill switch), camera, sun
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "00"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_gp; s_gr <= (others => s_fsw);
                            when 1      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gs; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        -- outside the deadband (|gap| >= 32, as bit tests: a signed compare's carry fans out)
                        if (s_gd(14) = '0' and s_gd(13 downto 5) /= "000000000") or (s_gd(14) = '1' and s_gd(13 downto 5) /= "111111111") then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gs <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 2 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] per-field constants, one small step per state
            c_inj <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>
                        c_t <= c_t + 1;
                        c_e <= s_gs(12 downto 3);
                    when 41 =>                              -- the body's centre: -150 .. +217 rows
                        v_e := c_e;
                        c_sunc <= signed(resize(v_e(9 downto 2), 12)) + signed(resize(v_e(9 downto 3), 12))
                                  - signed(resize(v_e(9 downto 6), 12)) - 150;
                    -- time of day thresholds (with hysteresis), one compare a state
                    when 47 => if c_e > 616 then c_hi <= '1'; else c_hi <= '0'; end if;
                    when 48 => if c_e < 584 and c_e > 176 then c_mid <= '1'; else c_mid <= '0'; end if;
                    when 49 => if c_e < 144 then c_lo <= '1'; else c_lo <= '0'; end if;
                    when 42 =>                              -- sea level: 626 -+ 96 rows
                        v_s := signed(resize(s_k2, 12)) - 512;
                        v_c := shift_right(v_s + shift_left(v_s, 1), 4);
                        c_sl <= unsigned(to_signed(C_SL0, 11) - v_c(10 downto 0));
                    when 43 =>
                        c_fp <= resize(s_gp, 14);
                        c_cgn <= s_g6(12 downto 9);
                    when 44 =>                              -- clouds and gulls drift on their own
                        c_sa1 <= c_t(13 downto 0);
                        c_sab <= c_t(11 downto 0) & "00";
                    when 45 =>                              -- time of day (with hysteresis): day, low sun, night
                        if s_night = '1' then c_tod <= "10";
                        elsif c_hi = '1' then c_tod <= "00";
                        elsif c_mid = '1' then c_tod <= "01";
                        elsif c_lo = '1' then c_tod <= "10";
                        end if;
                        c_esun <= not s_night; c_emoon <= s_night; c_ecrab <= not s_croff;
                        if s_night = '0' and c_e(9 downto 6) /= "0000" then c_ebird <= '1'; else c_ebird <= '0'; end if;
                        if s_night = '1' or c_e(9 downto 6) = "0000" then c_sden <= "11";
                        elsif c_e(9 downto 6) = "0001" then c_sden <= "10";
                        elsif c_e(9 downto 6) = "0010" then c_sden <= "01";
                        else c_sden <= "00"; end if;
                    when 46 =>
                        c_pal <= s_theme & c_tod;
                    -- the sky keys: zenith keyframe, then horizon keyframe, lerped by the elevation (day: all
                    -- eight keyframes over the slider; night: the first two)
                    when 50 | 51 =>
                        c_inj <= '1';
                        if s_night = '1' then
                            c_injent <= s_zc(0) & "0000"; c_injt <= c_e(9 downto 4);
                        else
                            c_injent <= s_zc(0) & '0' & c_e(9 downto 7); c_injt <= c_e(6 downto 1);
                        end if;
                        if s_zc(0) = '0' then c_injk <= "01"; else c_injk <= "10"; end if;
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
            s_st <= g_start & s_st(1 to 15);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's avid rise, field-latched width)
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
    -- [C] LINE ENGINE: kicked when a line's last pixel has left stage 1 (and once in the zone for line 0),
    -- computes the NEXT line's constants in horizontal blanking.  The l_* constants are written at state 32
    -- or later, after the previous line's last pixel has left the stages that read them.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y   : unsigned(10 downto 0);
        variable v_d   : unsigned(11 downto 0);
        variable v_a   : unsigned(11 downto 0);
        variable v_b   : signed(11 downto 0);
        variable v_row : unsigned(7 downto 0);
        variable v_t   : unsigned(9 downto 0);
        variable v_g3, v_g5, v_g32 : unsigned(4 downto 0);
        variable v_th  : unsigned(4 downto 0);
        variable v_a4  : unsigned(11 downto 0);
        variable v_rm  : signed(11 downto 0);
        variable v_b4  : signed(11 downto 0);
        variable v_w   : signed(4 downto 0);
        variable v_by  : unsigned(7 downto 0);
        variable v_fo  : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;     -- after the field constants
            e_gq <= g_avid;
            e_kick <= e_gq and not g_avid;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_ln <= (others => '0');
            elsif e_kick = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                if e_ln /= 3 then e_ln <= e_ln + 1; end if;
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            -- fill flags (states 40 .. 47, ramp = the state's low bits): ramp i is filled out to
            -- (fill - order x 1024) rows from the horizon; the sun goes first, then the sky, then ramps 2 .. 7
            if e_lc(6 downto 3) = "0101" then
                if resize(e_d, 14) + e_fc < c_fp then l_fill(to_integer(e_lc(2 downto 0))) <= '1';
                else l_fill(to_integer(e_lc(2 downto 0))) <= '0'; end if;
            end if;
            v_fo := e_lc(2 downto 0) + 1;
            case to_integer(v_fo) is
                when 1      => e_fc <= to_unsigned(0, 14);        -- sun
                when 0      => e_fc <= to_unsigned(1024, 14);     -- sky
                when others => e_fc <= resize(v_fo & "0000000000", 14);
            end case;

            v_y := e_vy(18 downto 8);
            case to_integer(e_lc) is
                when 0 =>
                    e_s <= signed(resize(v_y, 12)) - C_HZ;
                when 1 =>
                    if e_s(11) = '1' then v_d := unsigned(-e_s); else v_d := unsigned(e_s); end if;
                    if v_d(11 downto 10) /= "00" then e_d <= (others => '1'); else e_d <= v_d(9 downto 0); end if;
                    e_ab <= e_s(11);
                when 2 =>
                    -- under the horizon every line is a mirror line: the sky objects at the same distance
                    e_mir <= not e_ab;
                    e_dyu <= signed(resize(e_d, 12)) - c_sunc;
                    e_c1  <= signed(resize(e_d, 11)) - 200;
                    e_bd  <= signed(resize(e_d, 11)) - 60;
                    e_cr  <= signed(resize(v_y, 12)) - C_CRY;
                    v_rm := to_signed(C_HZ, 12) - signed(resize(e_d, 12));
                    if v_rm(11) = '1' then e_rm <= (others => '0'); else e_rm <= unsigned(v_rm(8 downto 0)); end if;
                when 3 =>
                    -- sun / moon (radius 128): half-chord address, the sun's slits, ramp position
                    if e_dyu(11) = '1' then v_a := unsigned(-e_dyu); else v_a := unsigned(e_dyu); end if;
                    e_son <= '0';
                    e_sa <= v_a(6 downto 0) & '0';
                    if v_a < 128 then
                        if c_emoon = '1' then
                            e_son <= '1';
                        elsif c_esun = '1' then
                            -- slits below the centre: period 32 rows, opening with depth, rolling
                            v_g3 := v_a(4 downto 0) + c_t(5 downto 1);
                            v_th := v_a(6 downto 2) + resize(v_a(6 downto 3), 5);
                            if e_dyu(11) = '0' or v_g3 >= v_th then e_son <= '1'; end if;
                        end if;
                    end if;
                    -- across the disc, 0 .. 126.  (An elevation term added here tripped yosys abc9's box-order
                    -- assert in every config: the netlist is shape-sensitive, see SUMMARY.md.)
                    v_b := e_dyu + 128;
                    e_tb <= unsigned(v_b(7 downto 2));
                    -- slot A: the cloud or gull band this line crosses
                    e_aen <= '0'; e_asel <= '0'; e_asc <= c_sa1;
                    e_aa <= (others => '0');
                    if e_c1(10 downto 7) = "0000" and e_c1(6 downto 5) /= "11" then          -- 0 <= e_c1 < 96 as bit tests
                        v_row := to_unsigned(47, 8) - unsigned(e_c1(8 downto 1));
                        e_aa <= to_unsigned(128, 10) + (v_row(7 downto 0) & "00");
                        e_aen <= '1';
                    elsif c_ebird = '1' and e_bd(10 downto 6) = "00000" then                  -- 0 <= e_bd < 64
                        v_row := to_unsigned(31, 8) - unsigned(e_bd(8 downto 1));
                        if c_t(4) = '1' then e_aa <= to_unsigned(448, 10) + (v_row(7 downto 0) & "00");
                        else e_aa <= to_unsigned(320, 10) + (v_row(7 downto 0) & "00"); end if;
                        e_asel <= '1'; e_asc <= c_sab;
                        e_aen <= '1';
                    end if;
                    -- slot B: the crab
                    e_ben <= '0';
                    e_ba <= (others => '0');
                    if c_ecrab = '1' and e_cr(11 downto 7) = "00000" and e_cr(6 downto 5) /= "11" then   -- 0 <= e_cr < 96
                        if c_t(5) = '1' then e_ba <= to_unsigned(768, 10) + (unsigned(e_cr(6 downto 1)) & "00");
                        else e_ba <= to_unsigned(576, 10) + (unsigned(e_cr(6 downto 1)) & "00"); end if;
                        e_ben <= '1';
                    end if;
                when 4 =>                                    -- the moon's bite: a disc 24 rows up
                    e_tsun <= e_tb & '0';                 -- (a low sun is redder through the low-sun palette)
                    v_b4 := e_dyu - 24;
                    if v_b4(11) = '1' then v_a4 := unsigned(-v_b4); else v_a4 := unsigned(v_b4); end if;
                    e_sb <= v_a4(6 downto 0) & '0';
                    if c_emoon = '1' and v_a4 < 128 then e_bon <= '1'; else e_bon <= '0'; end if;
                when 5 =>                                    -- the line's left edge (the sea strip wobbles it)
                    v_w := (others => '0');
                    if e_mir = '1' then
                        v_g5 := resize(e_d(4 downto 1) + c_t(5 downto 2), 5);
                        if v_g5(3) = '1' then v_w := signed('0' & (not v_g5(2 downto 0)) & '0') - 7;
                        else v_w := signed('0' & v_g5(2 downto 0) & '0') - 7; end if;
                    end if;
                    e_x0 <= w_x0 + resize(v_w, 14);
                when 6 =>
                    n_xa0 <= unsigned(e_x0) + e_asc;
                    n_xw0 <= unsigned(w_x0 + 1920);          -- water and sand: half units from the canvas's left
                when 7 =>
                    n_x0 <= e_x0(12 downto 0);
                    n_xb0 <= signed(n_xw0) - (2 * C_CRX);    -- the crab: half units from its centre line
                when 12 =>                                   -- the sheen's diagonal phase: x + y + 4 t
                    n_pa0 <= unsigned(e_x0) + resize(v_y & '0', 14) + (c_t(10 downto 0) & "000");
                when 32 =>
                    -- the line's constants (the previous line has left the pixel path by now)
                    l_d <= e_d; l_ab <= e_ab; l_mir <= e_mir; l_row <= v_y;
                    l_son <= e_son; l_bon <= e_bon; l_tsun <= e_tsun;
                    l_asel <= e_asel;
                    -- slot A's ramp positions: zone 1, zone 2 (gulls: wings, body; clouds: top, shelf)
                    if e_asel = '1' then l_ta <= to_unsigned(70, 7); l_tb <= to_unsigned(93, 7);
                    else l_ta <= to_unsigned(104, 7); l_tb <= to_unsigned(127, 7); end if;
                    -- the sky ramp's position: 0 at the top, 63 at the horizon (by the mirrored row below it)
                    v_t := resize(e_rm(8 downto 3), 10) + resize(e_rm(8 downto 4), 10) - resize(e_rm(8 downto 7), 10);
                    if v_t > 63 then l_tsky <= (others => '1'); else l_tsky <= v_t(5 downto 0); end if;
                    -- sea strip ripples: gaps of a 16-row period that open with depth, drifting
                    v_g32 := resize(e_d(3 downto 0) + c_t(4 downto 1), 5);
                    if e_mir = '1' and v_g32(3 downto 0) <= e_d(9 downto 6) then l_rip <= '1'; else l_rip <= '0'; end if;
                    if e_ln /= 0 then l_v1 <= '1'; else l_v1 <= '0'; end if;
                    if e_ln(1) = '1' and s_tv2 = '1' then l_v2 <= '1'; else l_v2 <= '0'; end if;
                    l_oy <= f_tri7(v_y(9 downto 2) + c_t(8 downto 1));
                    lr_a <= "000" & e_sa(7 downto 1);
                when 33 => lr_a <= "000" & e_sb(7 downto 1);
                when 34 => lr_a <= e_aa;
                when 35 => lr_a <= e_aa(9 downto 2) & "01";
                    if e_sa(0) = '1' then v_by := lr_qf(15 downto 8); else v_by := lr_qf(7 downto 0); end if;
                    l_sw <= "00" & v_by(7 downto 1);
                when 36 => lr_a <= e_aa(9 downto 2) & "10";
                    if e_sb(0) = '1' then v_by := lr_qf(15 downto 8); else v_by := lr_qf(7 downto 0); end if;
                    l_bw <= "00" & v_by(7 downto 1);
                when 37 => lr_a <= e_aa(9 downto 2) & "11";
                    l_arow(0) <= lr_qf;
                when 38 => lr_a <= e_ba;
                    l_arow(1) <= lr_qf;
                when 39 => lr_a <= e_ba(9 downto 2) & "01";
                    l_arow(2) <= lr_qf;
                when 40 => lr_a <= e_ba(9 downto 2) & "10";
                    if e_aen = '1' then l_arow(3) <= lr_qf; else l_arow(3) <= (others => '0'); end if;
                when 41 => lr_a <= e_ba(9 downto 2) & "11";
                    l_brow(0) <= lr_qf;
                when 42 => l_brow(1) <= lr_qf;
                when 43 => l_brow(2) <= lr_qf;
                when 44 =>
                    if e_ben = '1' then l_brow(3) <= lr_qf; else l_brow(3) <= (others => '0'); end if;
                -- sheen: the other directions.  x + 4 t (the diagonal phase less y) and y - 2 t
                when 45 => l_od <= to_unsigned(0, 14) - resize(v_y & '0', 14);
                when 46 => l_pc <= resize(v_y & '0', 14) - (c_t(11 downto 0) & "00");
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [C] BEAM COUNTERS: each layer's x (half units), loaded so that its results reach stage 14
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1'  then b_xw <= n_xw0; else b_xw <= b_xw + s_xs; end if;      -- stage 1
            if s_st(10) = '1' then                                                       -- stage 11
                b_xa <= n_xa0; b_xb <= n_xb0; b_x <= n_x0;
            else
                b_xa <= b_xa + s_xs;
                b_xb <= b_xb + signed(resize(s_xs, 14));
                b_x <= b_x + signed(resize(s_xs, 13));
            end if;
            if s_st(12) = '1' then b_px <= (others => '0'); else b_px <= b_px + 1; end if;   -- stage 13
            if s_st(16) = '1' then b_pa <= n_pa0; else b_pa <= b_pa + s_xs; end if;      -- stage 17
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [D] SOURCE: camera luma (or the Osc lattice), a one-pole smoother along the line, the K6 gain
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(5 downto 0);
        variable v_o : unsigned(7 downto 0);
        variable v_e : signed(10 downto 0);
        variable v_p : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_l := unsigned(data_in.y(9 downto 4));
            if s_src = '1' then
                v_o := resize(f_tri7(unsigned(b_x(9 downto 2)) + c_t(7 downto 0)), 8) + resize(l_oy, 8);
                s_cin <= v_o(7 downto 2);
            elsif v_l > 4 then
                s_cin <= v_l - 4;
            else
                s_cin <= (others => '0');
            end if;
            if s_avq = '1' then
                v_e := signed(resize(s_cin & "0000", 11)) - signed(resize(s_cf, 11));
                v_e := signed(resize(s_cf, 11)) + shift_right(v_e, 3);
                s_cf <= unsigned(v_e(9 downto 0));
            end if;
            v_p := (others => '0');
            if c_cgn(0) = '1' then v_p := v_p + resize(s_cf(9 downto 4), 10); end if;
            if c_cgn(1) = '1' then v_p := v_p + resize(s_cf(9 downto 4) & '0', 10); end if;
            if c_cgn(2) = '1' then v_p := v_p + resize(s_cf(9 downto 4) & "00", 10); end if;
            if c_cgn(3) = '1' then v_p := v_p + resize(s_cf(9 downto 4) & "000", 10); end if;
            s_cg <= v_p(9 downto 4);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent running 8-sums (resynced to
    -- neutral in blanking), luma [1 2 1]/4 at the chroma's group delay (+ 2 px for the glow), luma 13/16
    -- toward black, blanking gate.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_iu, v_iv, v_iy : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- the avid of the stage-24 pixel; neutral outside it
            if s_gav_sr(22) = '1' then
                v_iy := s24_y; v_iu := s24_u; v_iv := s24_v;
            else
                v_iy := to_unsigned(64, 10); v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10);
            end if;
            s_hist <= s_hist(6 downto 0) & s_gav_sr(22);
            -- cr_qf = the input 8 pixels back (written at t, read at t + 6 into cr_q, fabric register at t + 7)
            cr_ram(to_integer(cr_wp)) <= std_logic_vector(v_iu(9 downto 2)) & std_logic_vector(v_iv(9 downto 2));
            cr_q  <= cr_ram(to_integer(cr_rp));
            cr_qf <= cr_q;
            cr_wp <= cr_wp + 1; cr_rp <= cr_rp + 1;
            if s_hist = "00000000" and s_gav_sr(22) = '0' then
                s_bu <= to_unsigned(4100, 13); s_bv2 <= to_unsigned(4100, 13);
            else
                s_bu  <= s_bu + v_iu - (unsigned(cr_qf(15 downto 8)) & "00");
                s_bv2 <= s_bv2 + v_iv - (unsigned(cr_qf(7 downto 0)) & "00");
            end if;
            s_bud <= s_bu; s_bvd <= s_bv2;
            -- luma taps y(n - 5), y(n - 6), y(n - 7): the chroma's group delay plus the glow's 2 px.
            -- s_yd(5) = the input 5 pixels back (written at t, read at t + 3 into yr_q, fabric register at t + 4)
            yr_ram(to_integer(cr_wp)) <= "000000" & std_logic_vector(v_iy);
            yr_q  <= yr_ram(to_integer(yr_rp));
            yr_rp <= yr_rp + 1;
            s_yd(5) <= unsigned(yr_q(9 downto 0));
            for i in 6 to 7 loop s_yd(i) <= s_yd(i - 1); end loop;
            o1_ya <= resize(s_yd(5), 12) + resize(s_yd(7), 12) + 2;
            o1_yb <= s_yd(6);
            v_sc := resize(s_bu, 14) + s_bud;  o2_u <= v_sc(13 downto 4);
            v_sc := resize(s_bv2, 14) + s_bvd; o2_v <= v_sc(13 downto 4);
            v_y := o1_ya + shift_left(resize(o1_yb, 12), 1);
            o2_y <= v_y(11 downto 2);
            -- luma 13/16 toward black
            if o2_y > 64 then v_dl := o2_y - 64; else v_dl := (others => '0'); end if;
            o3_y <= o2_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            o3_u <= o2_u; o3_v <= o2_v;
            -- blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                o4_y <= o3_y; o4_u <= o3_u; o4_v <= o3_v;
            else
                o4_y <= to_unsigned(64, 10); o4_u <= to_unsigned(512, 10); o4_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;


    ---------------------------------------------------------------------------
    -- [C] PIXEL PATH, stages 2 .. 14: one block per object, each ending in a few bits at stage 14
    ---------------------------------------------------------------------------
    p_obj : process(clk)
        variable v_xn  : signed(12 downto 0);
        variable v_xr  : signed(12 downto 0);
        variable v_a12 : unsigned(11 downto 0);
        variable v_hh  : unsigned(11 downto 0);
        variable v_cell : unsigned(3 downto 0);
        variable v_ax  : unsigned(12 downto 0);
        variable v_bx  : signed(12 downto 0);
        variable v_ds  : signed(10 downto 0);
        variable v_db  : signed(10 downto 0);
        variable v_n, v_sd, v_dd : signed(11 downto 0);
        variable v_r2  : unsigned(10 downto 0);
        variable v_sk  : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------------
            -- water and sand: the knot table read (2 .. 4), the surface and bed lerps (5 .. 7), the camera
            -- lift (8), the inside tests (9), the layer divide (10 .. 12), level of detail (13)
            ----------------------------------------------------------------
            -- 2
            if s_zone = '1' then kr_a <= (others => '0'); else kr_a <= '0' & b_xw(11 downto 5); end if;
            w_f(2) <= b_xw(4 downto 1);
            w_xb(2) <= b_xw(7) & b_xw(6 downto 1);
            for i in 3 to 5 loop w_f(i) <= w_f(i - 1); end loop;
            for i in 3 to 10 loop w_xb(i) <= w_xb(i - 1); end loop;
            -- 5 .. 8: the steps times the fraction, as gated operands.  The bed's lerp runs one stage behind
            -- the surface's: with both sums gated by the same fraction bits in one stage, yosys abc9 hit its
            -- box-order assert in five of the six configs.
            v_ds := signed(kr_qf(37 downto 27));
            w5_ps1 <= f_gk(v_ds, w_f(4)(0), 0) + f_gk(v_ds, w_f(4)(1), 1);
            w5_ps2 <= f_gk(v_ds, w_f(4)(2), 2) + f_gk(v_ds, w_f(4)(3), 3);
            w5_s0 <= unsigned(kr_qf(47 downto 38));
            w5_b0 <= unsigned(kr_qf(26 downto 17));
            w5_db <= signed(kr_qf(16 downto 9));
            -- 6 (the camera raises the sand by up to 118 rows: folded into the bed's base here)
            v_db := resize(w5_db, 11);
            w6_ps <= w5_ps1 + w5_ps2;
            w6_pb1 <= f_gk(v_db, w_f(5)(0), 0) + f_gk(v_db, w_f(5)(1), 1);
            w6_pb2 <= f_gk(v_db, w_f(5)(2), 2) + f_gk(v_db, w_f(5)(3), 3);
            w6_s0 <= signed(resize(w5_s0, 12)) - signed(resize(s_cg & "00", 12));   -- the surface up to 236 rows
            w6_b0 <= signed(resize(w5_b0, 12)) - signed(resize(s_cg & '0', 12));    -- the sand up to 118
            -- 7
            w7_srf <= w6_s0 + resize(shift_right(w6_ps, 4), 12);
            w7_pb <= w6_pb1 + w6_pb2;
            w7_b0 <= w6_b0;
            -- 8: rows under the surface; the bed.  (n, the rows under the sand and the depth are linearly
            -- dependent: computed in one stage they gave ABC's &mfs a redundancy to exploit, which broke
            -- abc9's carry-box order.  n goes one stage early.)
            w8_n   <= signed(resize(l_row, 12)) - w7_srf;
            w8_srf <= w7_srf;
            w8_bed <= w7_b0 + resize(shift_right(w7_pb, 4), 12);
            -- 9: the local depth, rows under the sand line
            v_n  := w8_n;
            v_sd := signed(resize(l_row, 12)) - w8_bed;
            v_dd := w8_bed - w8_srf;
            w9_inw <= (not v_n(11)) and v_sd(11);
            w9_ins <= not v_sd(11);
            if v_n(11) = '1' then w9_n <= (others => '0');
            elsif v_n(10) = '1' then w9_n <= (others => '1');
            else w9_n <= unsigned(v_n(9 downto 0)); end if;
            if v_dd(11) = '1' then w9_dd <= (others => '0');
            elsif v_dd(10) = '1' then w9_dd <= (others => '1');
            else w9_dd <= unsigned(v_dd(9 downto 0)); end if;
            if v_sd(11) = '1' then w9_sd <= (others => '0');
            elsif v_sd(10) = '1' then w9_sd <= (others => '1');
            else w9_sd <= unsigned(v_sd(9 downto 0)); end if;
            -- 10 .. 12: q = floor(8 n / depth), one bit a stage (n < depth inside the water)
            v_r2 := w9_n & '0';
            if v_r2 >= resize(w9_dd, 11) then
                wd_r(10) <= resize(v_r2 - resize(w9_dd, 11), 10); wd_q(10) <= "001";
            else
                wd_r(10) <= v_r2(9 downto 0); wd_q(10) <= "000";
            end if;
            wd_d(10) <= w9_dd;
            for i in 11 to 12 loop
                v_r2 := wd_r(i - 1) & '0';
                if v_r2 >= resize(wd_d(i - 1), 11) then
                    wd_r(i) <= resize(v_r2 - resize(wd_d(i - 1), 11), 10); wd_q(i) <= wd_q(i - 1)(1 downto 0) & '1';
                else
                    wd_r(i) <= v_r2(9 downto 0); wd_q(i) <= wd_q(i - 1)(1 downto 0) & '0';
                end if;
                wd_d(i) <= wd_d(i - 1);
            end loop;
            -- 10: the water's ramp position (bright at the surface), the sand's (by depth) and its facets
            if w9_n(9 downto 8) /= "00" then wd_wt(10) <= (others => '0');
            else wd_wt(10) <= to_unsigned(127, 7) - w9_n(7 downto 1); end if;
            wd_st(10) <= to_unsigned(127, 7) - w9_sd(9 downto 3);
            wd_ss(10) <= w_xb(9)(5) & w9_sd(5) & '0';
            wd_inw(10) <= w9_inw; wd_ins(10) <= w9_ins; wd_cp(10) <= w_xb(9)(6);
            for i in 11 to 13 loop
                wd_wt(i) <= wd_wt(i - 1); wd_st(i) <= wd_st(i - 1); wd_ss(i) <= wd_ss(i - 1);
                wd_inw(i) <= wd_inw(i - 1); wd_ins(i) <= wd_ins(i - 1); wd_cp(i) <= wd_cp(i - 1);
            end loop;
            -- 13: level of detail: eight layers where the water is 128 rows deep or more, four, two, none
            if wd_d(12)(9 downto 7) /= "000" then w13_lay <= wd_q(12)(0);
            elsif wd_d(12)(6) = '1' then w13_lay <= wd_q(12)(1);
            elsif wd_d(12)(5) = '1' then w13_lay <= wd_q(12)(2);
            else w13_lay <= '0'; end if;
            -- 14
            w14_in <= wd_inw(13); w14_sub <= w13_lay & wd_cp(13); w14_t <= wd_wt(13);
            s14_in <= wd_ins(13); s14_sub <= wd_ss(13); s14_t <= wd_st(13);

            ----------------------------------------------------------------
            -- sun / moon
            ----------------------------------------------------------------
            -- (one carry chain a stage: the old chained subtract / abs / subtract / abs in one stage, on two
            -- related values, broke abc9's carry-box order under ABC's &mfs)
            -- 12: x from the disc's centre and from the bite's, both straight from the counter
            s12_xr  <= resize(b_x(12 downto 1), 13) - C_SUNX;
            s12_xr2 <= resize(b_x(12 downto 1), 13) - (C_SUNX + 50);
            -- 13: their magnitudes
            v_xn := -s12_xr;
            if s12_xr(12) = '1' then s12_ax <= unsigned(v_xn(11 downto 0)); else s12_ax <= unsigned(s12_xr(11 downto 0)); end if;
            v_xn := -s12_xr2;
            if s12_xr2(12) = '1' then s12_ax2 <= unsigned(v_xn(11 downto 0)); else s12_ax2 <= unsigned(s12_xr2(11 downto 0)); end if;
            -- 14
            u14_in <= '0';
            if l_son = '1' and s12_ax < resize(l_sw, 12) then
                if l_bon = '0' or s12_ax2 >= resize(l_bw, 12) then u14_in <= '1'; end if;
            end if;

            ----------------------------------------------------------------
            -- stars: 32 x 32 cells, up to three in thirty-two hold a 4 x 4 star; one phase in four it is dimmed
            ----------------------------------------------------------------
            -- 12
            v_a12 := unsigned(b_x(12 downto 6)) & l_d(9 downto 5);
            t12_h <= v_a12 + shift_left(v_a12, 3) + 693;
            t12_x <= unsigned(b_x(5 downto 3));
            -- 13
            v_hh := t12_h xor ("0000" & t12_h(11 downto 4)) xor ("0000000" & t12_h(11 downto 7));
            t13_h <= v_hh + shift_left(v_hh, 5) + 1447;
            t13_x <= t12_x;
            -- 14
            v_hh := t13_h xor ("000000" & t13_h(11 downto 6));
            if v_hh(11 downto 7) < resize(c_sden, 5) and v_hh(2 downto 0) = t13_x and v_hh(5 downto 3) = l_d(4 downto 2) then
                t14_on <= '1';
            else
                t14_on <= '0';
            end if;
            if v_hh(7 downto 6) = c_t(6 downto 5) then t14_tw <= "00"; else t14_tw <= "11"; end if;

            ----------------------------------------------------------------
            -- the sky's ramp positions: filled, the line's position lifted by the camera (clamped to the
            -- segment); wires only, the camera alone
            ----------------------------------------------------------------
            v_sk := resize(l_tsky, 7) + resize(s_cg, 7);
            if v_sk(6) = '1' then k14_skt <= (others => '1'); else k14_skt <= v_sk(5 downto 0); end if;
            k14_cg2 <= s_cg & '0';
            if s_cg /= 0 then k14_cam <= '1'; else k14_cam <= '0'; end if;

            ----------------------------------------------------------------
            -- sprite slots: span-coded rows.  A = clouds / gulls (2 virtual px a unit), B = the crab (folded)
            ----------------------------------------------------------------
            -- 12: lattice cell, presence, x inside the sprite
            v_cell := b_xa(13 downto 10);
            a12_xs <= b_xa(9 downto 2);
            if l_asel = '0' then a12_pr <= C_CM1(to_integer(v_cell)); else a12_pr <= C_BM(to_integer(v_cell)); end if;
            v_bx := b_xb(13 downto 1);
            if v_bx(12) = '1' then v_ax := unsigned(-v_bx); else v_ax := unsigned(v_bx); end if;
            b12_xs <= v_ax(7 downto 0);
            if v_ax(12 downto 8) = "00000" then b12_pr <= '1'; else b12_pr <= '0'; end if;
            -- 13: thermometer against the six boundaries
            for i in 0 to 2 loop
                if a12_xs >= l_arow(i)(7 downto 0)  then a13_g(2 * i) <= '1';     else a13_g(2 * i) <= '0'; end if;
                if a12_xs >= l_arow(i)(15 downto 8) then a13_g(2 * i + 1) <= '1'; else a13_g(2 * i + 1) <= '0'; end if;
                if b12_xs >= l_brow(i)(7 downto 0)  then b13_g(2 * i) <= '1';     else b13_g(2 * i) <= '0'; end if;
                if b12_xs >= l_brow(i)(15 downto 8) then b13_g(2 * i + 1) <= '1'; else b13_g(2 * i + 1) <= '0'; end if;
            end loop;
            a13_pr <= a12_pr; b13_pr <= b12_pr;
            -- 14: the zone of the interval
            if a13_pr = '0' then a14_z <= "00";
            elsif a13_g(5) = '1' then a14_z <= l_arow(3)(13 downto 12);
            elsif a13_g(4) = '1' then a14_z <= l_arow(3)(11 downto 10);
            elsif a13_g(3) = '1' then a14_z <= l_arow(3)(9 downto 8);
            elsif a13_g(2) = '1' then a14_z <= l_arow(3)(7 downto 6);
            elsif a13_g(1) = '1' then a14_z <= l_arow(3)(5 downto 4);
            elsif a13_g(0) = '1' then a14_z <= l_arow(3)(3 downto 2);
            else a14_z <= l_arow(3)(1 downto 0); end if;
            if b13_pr = '0' then b14_z <= "00";
            elsif b13_g(5) = '1' then b14_z <= l_brow(3)(13 downto 12);
            elsif b13_g(4) = '1' then b14_z <= l_brow(3)(11 downto 10);
            elsif b13_g(3) = '1' then b14_z <= l_brow(3)(9 downto 8);
            elsif b13_g(2) = '1' then b14_z <= l_brow(3)(7 downto 6);
            elsif b13_g(1) = '1' then b14_z <= l_brow(3)(5 downto 4);
            elsif b13_g(0) = '1' then b14_z <= l_brow(3)(3 downto 2);
            else b14_z <= l_brow(3)(1 downto 0); end if;
        end if;
    end process p_obj;

    ---------------------------------------------------------------------------
    -- ROMs, the ID line store and the knot table (one write statement each; registered reads, re-registered
    -- in fabric).  The sky's filled pixels take the per-field sky keys in place of the colour table's word.
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lr_q   <= C_NB_LROM(to_integer(lr_a));
            lr_qf  <= lr_q;
            pal_q  <= C_NB_PAL(to_integer(pal_a));
            if s19_sky = '1' then pal_qf <= unsigned(l_skyc); else pal_qf <= pal_q; end if;
        end if;
    end process p_rom;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            lb_q  <= lb_ram(to_integer(b_px));
            lb_qf <= lb_q;
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lb;

    p_kr : process(clk)
    begin
        if rising_edge(clk) then
            kr_q  <= kr_ram(to_integer(kr_a));
            kr_qf <= kr_q;
            if kr_we = '1' then kr_ram(to_integer(kr_wa)) <= kr_wd; end if;
        end if;
    end process p_kr;

    w_x0 <= to_signed(-1440, 14) when s_xs = 4 else to_signed(-1920, 14);

    ---------------------------------------------------------------------------
    -- [C] FILLER (vblank): writes the knot table from the right edge to the left.  v0.1.0: the surface is
    -- the sea level wherever the bed is below it, and the bed itself on the beach.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_ds : signed(10 downto 0);
        variable v_db : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            kr_we <= '0';
            f_bq <= C_NB_BED(to_integer(f_k));
            if s_zone = '1' and s_zc = 100 then
                f_run <= '1'; f_k <= to_unsigned(127, 7); f_st <= "00"; f_first <= '1';
            elsif f_run = '1' then
                case f_st is
                    when "00" => f_st <= "01";
                    when "01" => f_b <= f_bq; f_st <= "10";
                    when "10" =>
                        if f_b > c_sl(9 downto 0) then f_s <= c_sl(9 downto 0); else f_s <= f_b; end if;
                        f_st <= "11";
                    when others =>
                        if f_first = '1' then
                            v_ds := (others => '0'); v_db := (others => '0');
                        else
                            v_ds := signed(resize(f_sn, 11)) - signed(resize(f_s, 11));
                            v_db := signed(resize(f_bn, 11)) - signed(resize(f_b, 11));
                        end if;
                        kr_we <= '1';
                        kr_wa <= '0' & f_k;
                        kr_wd <= std_logic_vector(f_s) & std_logic_vector(v_ds) & std_logic_vector(f_b)
                                 & std_logic_vector(v_db(7 downto 0)) & "000000000";
                        f_sn <= f_s; f_bn <= f_b; f_first <= '0';
                        if f_k = 0 then f_run <= '0'; else f_k <= f_k - 1; end if;
                        f_st <= "00";
                end case;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [E] MERGE, EDGES, COLOUR (stages 15 .. 24)
    ---------------------------------------------------------------------------
    p_mix : process(clk)
        variable v_obj  : unsigned(3 downto 0);
        variable v_sub  : std_logic_vector(3 downto 0);
        variable v_ramp : unsigned(2 downto 0);
        variable v_t    : unsigned(7 downto 0);
        variable v_fon, v_sky : std_logic;
        variable v_e    : std_logic;
        variable v_w    : unsigned(3 downto 0);
        variable v_tri  : unsigned(4 downto 0);
        variable v_src  : std_logic_vector(1 downto 0);
        variable v_sc   : std_logic;
        variable v_ph   : unsigned(13 downto 0);
        variable v_gi   : unsigned(5 downto 0);
        variable v_sum  : signed(13 downto 0);
        variable v_c    : signed(10 downto 0);
        variable v_d8   : signed(7 downto 0);
        type t_o3 is array (0 to 2) of unsigned(7 downto 0);
        variable v_out  : t_o3;
        variable v_y10  : unsigned(9 downto 0);
        variable v_k9   : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------------
            -- 15: who owns the pixel, nearest first: crab, sand, water, clouds / gulls, sun / moon, sky
            ----------------------------------------------------------------
            -- (no adds in this stage: every position arrives registered.  An add, a clamp and per-arm offsets
            -- here tripped yosys abc9's box-order assert in the full build.)
            v_obj := "0000"; v_sub := "0000"; v_ramp := "000"; v_sky := '0';
            if l_fill(0) = '1' then                      -- the sky's per-field colours (the camera lifts it)
                v_fon := '1'; v_sky := '1';
                v_t := resize(k14_skt, 8);
            elsif k14_cam = '1' then                     -- wires only: the camera lights the sky ramp
                v_fon := '1';
                v_t := resize(k14_cg2, 8);
            else
                v_fon := '0';
                v_t := (others => '0');
            end if;
            if b14_z /= 0 then
                v_obj := "1001"; v_sub := "00" & std_logic_vector(b14_z); v_ramp := "111";
                if b14_z = "01" then v_sub(2) := l_brow(3)(14); end if;       -- creases across the shell
                case b14_z is
                    when "01"   => v_t := to_unsigned(64, 8);
                    when "10"   => v_t := to_unsigned(8, 8);
                    when others => v_t := to_unsigned(100, 8);
                end case;
                v_fon := l_fill(7); v_sky := '0';
            elsif s14_in = '1' then
                v_obj := "1000"; v_sub := '0' & s14_sub; v_ramp := "101";
                v_t := resize(s14_t, 8); v_fon := l_fill(5); v_sky := '0';
            elsif w14_in = '1' then
                v_obj := "0101"; v_sub := "00" & w14_sub; v_ramp := "010";
                v_t := resize(w14_t, 8); v_fon := l_fill(2); v_sky := '0';
            elsif a14_z /= 0 then
                v_obj := "0010"; v_sub := "00" & std_logic_vector(a14_z); v_ramp := "110";
                if a14_z = "10" then v_t := resize(l_tb, 8); else v_t := resize(l_ta, 8); end if;
                v_fon := l_fill(6); v_sky := '0';
            elsif u14_in = '1' then
                v_obj := "0001"; v_ramp := "001"; v_t := resize(l_tsun, 8); v_fon := l_fill(1); v_sky := '0';
            end if;
            s15_id <= std_logic_vector(v_obj) & v_sub;
            s15_ramp <= v_ramp; s15_t <= v_t(6 downto 0); s15_fon <= v_fon; s15_sky <= v_sky;
            if t14_on = '1' and v_obj = 0 then s15_ovl <= "01"; else s15_ovl <= "00"; end if;
            s15_tw <= t14_tw;

            -- the ID history along the line, and the line store (this line's ID above last line's)
            i_d1 <= s15_id; i_d2 <= i_d1; i_d3 <= i_d2;
            s_g15 <= s_g15(2 downto 0) & s_gav_sr(13);
            lb_we <= s_gav_sr(13);
            if s_st(15) = '1' then lb_wa <= (others => '0'); else lb_wa <= lb_wa + 1; end if;     -- pixel index at stage 16
            lb_wd <= lb_qf(7 downto 0) & s15_id;

            ----------------------------------------------------------------
            -- 16: a wire wherever the ID changed within the last pixels or lines; its colour = the nearest
            -- object among the neighbours
            ----------------------------------------------------------------
            v_e := '0';
            if s_g15(0) = '1' and s15_id /= i_d1 then v_e := '1'; end if;
            if s_g15(1) = '1' and s_th2 = '1' and s15_id /= i_d2 then v_e := '1'; end if;
            if s_g15(2) = '1' and s_th3 = '1' and s15_id /= i_d3 then v_e := '1'; end if;
            if l_v1 = '1' and s15_id /= lb_qf(7 downto 0) then v_e := '1'; end if;
            if l_v2 = '1' and s15_id /= lb_qf(15 downto 8) then v_e := '1'; end if;
            s16_e  <= v_e;
            s16_m1 <= f_max4(unsigned(s15_id(7 downto 4)), unsigned(i_d1(7 downto 4)));
            s16_m2 <= f_max4(unsigned(i_d2(7 downto 4)), unsigned(i_d3(7 downto 4)));
            s16_m3 <= f_max4(unsigned(lb_qf(7 downto 4)), unsigned(lb_qf(15 downto 12)));
            s16_ramp <= s15_ramp; s16_t <= s15_t; s16_fon <= s15_fon; s16_sky <= s15_sky;
            s16_ovl <= s15_ovl; s16_tw <= s15_tw;

            ----------------------------------------------------------------
            -- 17: colour table entry and lerp position; the sea strip's ripple gaps; the made-up sky pixels
            ----------------------------------------------------------------
            v_w := f_max4(f_max4(s16_m1, s16_m2), s16_m3);
            s17_blk <= '0'; s17_w <= '0'; s17_sky <= '0'; s17_pn <= c_pal; s_ik(17) <= "00";
            if s16_e = '1' then
                s17_ent <= '1' & v_w; s17_w <= '1';
                s17_t6 <= (others => '0');
            elsif s16_ovl = "01" then
                s17_ent <= to_unsigned(27, 5); s17_t6 <= s16_tw & "0000";
            elsif s16_fon = '1' then
                s17_ent <= '0' & s16_ramp & s16_t(6); s17_t6 <= s16_t(5 downto 0);
                s17_sky <= s16_sky;
            else
                s17_ent <= (others => '0'); s17_t6 <= (others => '0');
                s17_blk <= '1';
            end if;
            -- the mirror (objects below the water's number): ripple gaps and dimming
            if l_rip = '1' and v_w < 5 then s17_blk <= '1'; s17_w <= '0'; end if;
            if l_mir = '1' and v_w < 5 then s17_dim <= '1'; else s17_dim <= '0'; end if;
            if c_inj = '1' then
                s17_ent <= c_injent; s17_t6 <= c_injt; s17_pn <= s_theme & "11";
                s17_blk <= '0'; s17_w <= '0'; s17_sky <= '0'; s17_dim <= '0';
                s_ik(17) <= c_injk;
            end if;

            -- 18 / 19: the colour table (address register, EBR, fabric register)
            pal_a <= s17_pn & s17_ent;
            s18_t6 <= s17_t6; s18_blk <= s17_blk;
            s19_t6 <= s18_t6; s19_blk <= s18_blk;
            s20_blk <= s19_blk;                          -- pal_q at 19, pal_qf at 20
            s18_w <= s17_w; s19_w <= s18_w; s20_w <= s19_w; s21_w <= s20_w; s22_w <= s21_w; s23_w <= s22_w;
            s18_sky <= s17_sky; s19_sky <= s18_sky;
            s18_dim <= s17_dim; s19_dim <= s18_dim; s20_dim <= s19_dim; s21_dim <= s20_dim; s22_dim <= s21_dim;
            for i in 18 to 23 loop s_ik(i) <= s_ik(i - 1); end loop;

            ----------------------------------------------------------------
            -- 18 .. 20: SHEEN, the light that runs along the wires (a wire's lerp position toward the
            -- palette's hot colour): a 64-px triangle pulse in every 256 of a phase.  Sun and clouds / gulls
            -- read the diagonal phase with every pulse shown; water, foam, wash, sand and crab each read
            -- their own phase and scale, and a table keyed by the pulse's number and the object lets about
            -- three pulses in five through.
            --   A  x + y + 4 t   "/" fronts      C  y - 2 t   bands moving down      D  x + 4 t   bands moving left
            ----------------------------------------------------------------
            case s17_ent(3 downto 0) is
                when "0101" => v_src := "11"; v_sc := '0';       -- water          D, 256
                when "0110" => v_src := "11"; v_sc := '0';       -- foam           D, 256
                when "0111" => v_src := "11"; v_sc := '1';       -- wash           D, 512
                when "1000" => v_src := "10"; v_sc := '1';       -- sand           C, 512
                when "1001" => v_src := "11"; v_sc := '0';       -- crab           D, 256
                when others => v_src := "00"; v_sc := '0';
            end case;
            if unsigned(s17_ent(3 downto 0)) < 5 then
                v_src := "00"; v_sc := '0'; s18_ng <= '1';
            else
                s18_ng <= '0';
            end if;
            if v_src = "11" then v_ph := b_pa + l_od; else v_ph := b_pa; end if;
            if v_src = "10" then v_ph := l_pc; end if;
            if v_sc = '1' then s18_np <= v_ph(9 downto 1); s18_ix <= v_ph(12 downto 10);
            else s18_np <= v_ph(8 downto 0); s18_ix <= v_ph(11 downto 9); end if;
            s18_wo <= s17_ent(2 downto 0);
            -- 19: the pulse (a triangle, 0 .. 31)
            v_gi := s18_ix & s18_wo;
            if s18_np(6) = '1' then v_tri := not s18_np(5 downto 1); else v_tri := s18_np(5 downto 1); end if;
            if s18_np(8 downto 7) /= "00" or (s18_ng = '0' and C_SHG(to_integer(v_gi)) = '0') then
                v_tri := (others => '0');
            end if;
            s19_n6 <= v_tri & '0';
            -- 20
            if s19_w = '1' then s20_t6 <= s19_n6; else s20_t6 <= s19_t6; end if;

            ----------------------------------------------------------------
            -- 21 / 22: c0 + (delta x t) >> 5, the product as three pairs of gated operands
            ----------------------------------------------------------------
            for k in 0 to 2 loop
                v_d8 := signed(pal_qf(23 - 8 * k downto 16 - 8 * k));
                s21_pa(k) <= f_gate(v_d8, s20_t6(0), 0) + f_gate(v_d8, s20_t6(1), 1);
                s21_pb(k) <= f_gate(v_d8, s20_t6(2), 2) + f_gate(v_d8, s20_t6(3), 3);
                s21_pc(k) <= f_gate(v_d8, s20_t6(4), 4) + f_gate(v_d8, s20_t6(5), 5);
                s21_c0(k) <= pal_qf(47 - 8 * k downto 40 - 8 * k);
            end loop;
            s21_blk <= s20_blk;
            for k in 0 to 2 loop
                v_sum := s21_pa(k) + s21_pb(k) + s21_pc(k);
                s22_s(k) <= signed(resize(s21_c0(k), 11)) + resize(shift_right(v_sum, 5), 11);
            end loop;
            s22_blk <= s21_blk;

            -- 23: clamp, black, the mirror's dimming
            for k in 0 to 2 loop
                v_c := s22_s(k);
                if v_c(10) = '1' then v_out(k) := (others => '0');
                elsif v_c(9 downto 8) /= "00" then v_out(k) := (others => '1');
                else v_out(k) := unsigned(v_c(7 downto 0)); end if;
            end loop;
            v_y10 := v_out(0) & "00";
            if s22_dim = '1' and v_y10 > 64 then
                v_y10 := v_y10 - shift_right(v_y10 - 64, 2);
            end if;
            if s22_blk = '1' then
                s23_y <= to_unsigned(64, 10); s23_u <= to_unsigned(512, 10); s23_v <= to_unsigned(512, 10);
            else
                if v_y10 < 64 then s23_y <= to_unsigned(64, 10); else s23_y <= v_y10; end if;
                s23_u <= v_out(1) & "00"; s23_v <= v_out(2) & "00";
            end if;
            s23_blk <= s22_blk;

            -- the sky keys from the made-up pixels, and the sky ramp word (c0 = zenith, delta = half the
            -- step to the horizon)
            if s_ik(23) = "01" then k_zy <= s23_y(9 downto 2); k_zu <= s23_u(9 downto 2); k_zv <= s23_v(9 downto 2); end if;
            if s_ik(23) = "10" then k_hy <= s23_y(9 downto 2); k_hu <= s23_u(9 downto 2); k_hv <= s23_v(9 downto 2); end if;
            l_skyc(47 downto 40) <= std_logic_vector(k_zy);
            l_skyc(39 downto 32) <= std_logic_vector(k_zu);
            l_skyc(31 downto 24) <= std_logic_vector(k_zv);
            v_k9 := signed(resize(k_hy, 9)) - signed(resize(k_zy, 9)); l_skyc(23 downto 16) <= std_logic_vector(v_k9(8 downto 1));
            v_k9 := signed(resize(k_hu, 9)) - signed(resize(k_zu, 9)); l_skyc(15 downto 8)  <= std_logic_vector(v_k9(8 downto 1));
            v_k9 := signed(resize(k_hv, 9)) - signed(resize(k_zv, 9)); l_skyc(7 downto 0)   <= std_logic_vector(v_k9(8 downto 1));

            -- 24: neon glow.  A wire's chroma is held over the black pixels that follow it (4 px), so the
            -- wide chroma filter below returns a wire at close to full saturation instead of a pastel, with
            -- a faint coloured halo on black; the luma is delayed 2 px to sit in the middle of it.
            s24_y <= s23_y; s24_u <= s23_u; s24_v <= s23_v;
            if s23_w = '1' then
                h_u <= s23_u; h_v <= s23_v; h_c <= to_unsigned(4, 3);
            elsif s23_blk = '1' and h_c /= 0 then
                s24_u <= h_u; s24_v <= h_v; h_c <= h_c - 1;
            else
                h_c <= (others => '0');
            end if;
        end if;
    end process p_mix;
    data_out.y       <= std_logic_vector(o4_y);
    data_out.u       <= std_logic_vector(o4_u);
    data_out.v       <= std_logic_vector(o4_v);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture neonbeach;
