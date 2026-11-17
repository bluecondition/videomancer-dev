-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- NEONSCAPE v0.1.0 -- wireframe neon landscapes (proof of concept).
--
-- A scene of layered objects, each drawn the same way: every pixel gets a REGION ID (object + facet), a wire is
-- drawn wherever the ID differs from a neighbour (left, or the line above: one line store), and a region is
-- filled from its own colour ramp when its gradient is switched in.  Hidden lines fall out for free (a nearer
-- object owns the pixel), every object gets its own wire colour, and a new object costs only its ID logic.
--
--   sky half (rows above the horizon, d = rows above it)
--     sky            vertical ramp; stars (hashed cells, twinkling); rain (falling dashes in hashed columns)
--     sun / moon     half-chord from a table per line; the sun's lower half is cut by rolling slits, the moon
--                    is a crescent (a second disc bites it)
--     clouds, birds  span-coded sprites in drifting lattice cells (slot A)
--     far range      a hashed polyline ridge; contour lines parallel to the ridge flow downhill
--     city           hashed towers, each with its own wire pattern (floors, bays, both), masts, lit windows
--                    that climb
--     near range     a hashed polyline ridge with a draped mesh: columns at the knots, rows at k / 8 of the
--                    local height (a 9-stage pipelined divide), flowing uphill; lit and shaded slopes
--     palms          span-coded sprite lattice (slot B), swaying
--   floor half (d = rows below the horizon)
--     grid           one divide per line: du = G / d cells per pixel, z = 1024 du; lanes from a per-pixel
--                    accumulator, rows from z + flight (bent per pixel by the camera)
--     road, car      the centre lanes with a dashed line; a span-coded car (rear view, or side view in Pan)
--     lake           (S10) the sky half mirrored: every sky object is a function of d, so it is free;
--                    ripple gaps and a per-line wobble
--   tunnel           (Land = Tunnel) the grid on both halves
--
--   K1  Land     Open / Hills / Ranges / Metro / Valley / Palms / Strip / Tunnel
--   K2  Sky      Sun / Sun Clouds / Sun Birds / Dusk / Moon / Moon Clouds / Storm / Stars
--   K3  Horizon  25 % .. 75 % of the screen
--   K4  Speed    bipolar, centre = still: flight (Drive) or lateral travel with parallax (Pan)
--   K5  Colour   8 palettes (8 fill ramps of 3 keys + a wire colour per object each)
--   K6  Camera   camera (or Osc) strength, three effects: lights the sky ramp, bends the grid's rows,
--                lifts the ridges and roofs
--   S7  Gradient On / Off (pure wireframe)
--   S8  Shade    Smooth / Sharp (ramps cut to 8 hard bands)
--   S9  View     Drive / Pan
--   S10 Ground   Grid / Lake
--   S11 Source   Video / Osc
--   P12 FILL     the rolling gradient scale: object by object (sun, sky, far range, city, near range, floor,
--                sprites) the fill wipes out from the horizon; 0 % = wires only, 100 % = everything filled
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  One latency
-- (33) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).  Chassis ([A] timing, virtual
-- 1080-line coordinates, generated avid, output conditioning) from riley / outrun.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.neonscape_pkg.all;
use work.all;

architecture neonscape of program_top is
    constant C_LAT : integer := 33;   -- = processing_delay_clks

    constant C_DEF_K3  : integer := 600;
    constant C_DEF_P12 : integer := 512;
    constant C_G       : integer := 2883584;      -- grid: du = C_G / d (cells per virtual px, Q20): 2.75 cells of camera height

    -- lattice presence / flip masks (bit = cell index)
    constant C_PMASK : std_logic_vector(15 downto 0) := x"6DB6";   -- palms: none in the two centre cells
    constant C_PFLIP : std_logic_vector(15 downto 0) := x"1C4A";
    constant C_CM1   : std_logic_vector(15 downto 0) := x"5A93";   -- clouds, band 1
    constant C_CM2   : std_logic_vector(15 downto 0) := x"A64D";   -- clouds, band 2
    constant C_BM    : std_logic_vector(15 downto 0) := x"1242";   -- birds

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
    -- 8-zone stepped knob with hysteresis; a value under 8 is the firmware's label index at program load
    function f_zone8(k : unsigned(9 downto 0); cur : unsigned(2 downto 0)) return unsigned is
        variable v_c : signed(10 downto 0);
    begin
        if k < 8 then return k(2 downto 0); end if;
        v_c := signed(resize(k, 11)) - signed(resize(cur & "1000000", 11));
        if v_c > 72 or v_c < -72 then return k(9 downto 7); end if;
        return cur;
    end function;
    -- signed 8-bit delta times one bit of t, shifted: a gated operand for the colour lerp
    function f_gate(d : signed(7 downto 0); b : std_logic; sh : natural) return signed is
        variable v : signed(13 downto 0);
    begin
        if b = '1' then v := shift_left(resize(d, 14), sh); else v := (others => '0'); end if;
        return v;
    end function;

    -- lattice presence by slot A's band (0, 1 = cloud bands, 2 = birds)
    function f_amask(sel : unsigned(1 downto 0); cell : unsigned(3 downto 0)) return std_logic is
    begin
        case sel is
            when "00"   => return C_CM1(to_integer(cell));
            when "01"   => return C_CM2(to_integer(cell));
            when others => return C_BM(to_integer(cell));
        end case;
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
    signal s_land, s_sky, s_pal : unsigned(2 downto 0) := (others => '0');
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);     -- horizon glide (Q3)
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);         -- camera glide
    signal s_gp : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);    -- fill glide
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(1 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_goff, s_sharp, s_pan, s_lake, s_src : std_logic := '0';
    -- scene enables
    signal c_efar, c_ecity, c_enear, c_epalm, c_eroad, c_tun : std_logic := '0';
    signal c_esun, c_emoon, c_estar, c_ecl1, c_ecl2, c_ebird, c_erain : std_logic := '0';

    -- [C] per-field values
    signal c_t    : unsigned(15 downto 0) := (others => '0');     -- field counter
    signal c_hz   : unsigned(10 downto 0) := to_unsigned(600, 11);
    signal c_bot  : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal c_cart : signed(11 downto 0) := to_signed(248, 12);    -- the rear-view car's top row below the horizon
    signal c_fp   : unsigned(13 downto 0) := (others => '0');     -- fill position (Q3)
    signal c_cgn  : unsigned(3 downto 0) := (others => '0');      -- camera gain
    signal c_zph  : unsigned(16 downto 0) := (others => '0');     -- flight phase (cells, Q10)
    signal c_uph  : unsigned(28 downto 0) := (others => '0');     -- lateral floor phase (cells, Q20)
    signal c_pan  : unsigned(21 downto 0) := (others => '0');     -- lateral position (virtual px, Q6)
    signal c_sunc : unsigned(8 downto 0) := to_unsigned(144, 9);  -- sun / moon centre, rows above the horizon
    signal c_sx   : signed(11 downto 0) := (others => '0');       -- and its x
    signal c_sn, c_sf, c_sc, c_sb, c_sa1, c_sa2, c_sab : unsigned(13 downto 0) := (others => '0');   -- layer scrolls
    signal c_swm  : unsigned(2 downto 0) := (others => '0');      -- palm sway
    signal c_swn  : std_logic := '0';

    -- [C] line ROM / colour ROM / line store
    signal lr_a  : unsigned(10 downto 0) := (others => '0');
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

    -- [C] line engine
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_lc   : unsigned(6 downto 0) := (others => '1');
    signal e_ln   : unsigned(1 downto 0) := (others => '0');
    signal e_s    : signed(11 downto 0) := (others => '0');
    signal e_d    : unsigned(9 downto 0) := (others => '0');
    signal e_ab   : std_logic := '0';
    signal e_sky, e_obj, e_low, e_mir, e_gq, e_kick : std_logic := '0';
    signal e_sa, e_sb : unsigned(7 downto 0) := (others => '0');
    signal e_tsun : unsigned(6 downto 0) := (others => '0');
    signal e_asel : unsigned(1 downto 0) := "00";
    signal e_bfold, e_bunit : std_logic := '0';
    signal e_dyu  : signed(10 downto 0) := (others => '0');
    signal e_c1, e_c2, e_bd : signed(10 downto 0) := (others => '0');
    signal e_cr, e_cs : signed(11 downto 0) := (others => '0');
    signal e_aa, e_ba : unsigned(10 downto 0) := (others => '0');
    signal e_aen, e_ben, e_son, e_bon : std_logic := '0';
    signal e_asc  : unsigned(13 downto 0) := (others => '0');
    signal e_bpalm : std_logic := '0';
    signal dv_n   : unsigned(21 downto 0) := (others => '0');
    signal dv_q   : unsigned(21 downto 0) := (others => '0');
    signal dv_r   : unsigned(10 downto 0) := (others => '0');
    signal dv_d   : unsigned(9 downto 0) := (others => '1');
    signal e_du   : unsigned(21 downto 0) := (others => '0');
    signal e_m    : unsigned(28 downto 0) := (others => '0');
    signal e_x0   : signed(13 downto 0) := (others => '0');
    signal e_fc   : unsigned(13 downto 0) := (others => '0');

    -- this line's constants (written by the line engine in blanking) and the next line's start values
    signal l_d : unsigned(9 downto 0) := (others => '0');
    signal l_mir, l_flr, l_obj, l_ab : std_logic := '0';
    signal l_sw, l_bw : unsigned(8 downto 0) := (others => '0');
    signal l_son, l_bon : std_logic := '0';
    signal l_tsun, l_tsky, l_tflr, l_tfar, l_tnr : unsigned(6 downto 0) := (others => '0');
    signal l_tcty : unsigned(5 downto 0) := (others => '0');
    type t_row is array (0 to 3) of unsigned(15 downto 0);
    signal l_arow, l_brow : t_row := (others => (others => '0'));
    signal l_asel : unsigned(1 downto 0) := "00";
    signal l_bunit, l_bfold : std_logic := '0';
    signal l_ta : unsigned(6 downto 0) := (others => '0');
    signal l_dup : unsigned(23 downto 0) := (others => '0');
    signal n_u0 : unsigned(28 downto 0) := (others => '0');
    signal l_zf : unsigned(11 downto 0) := (others => '0');
    signal l_lod : unsigned(2 downto 0) := "000";
    signal l_gfar, l_proad, l_pdash : std_logic := '0';
    signal l_fill : std_logic_vector(7 downto 0) := (others => '0');
    signal l_rip, l_v1, l_v2 : std_logic := '0';
    signal l_wf : unsigned(2 downto 0) := (others => '0');
    signal l_rf : unsigned(3 downto 0) := (others => '0');
    signal l_oy : unsigned(6 downto 0) := (others => '0');
    signal n_xn0, n_xf0, n_xc0, n_xa0, n_xb0, n_xr0 : unsigned(13 downto 0) := (others => '0');
    signal n_x0 : signed(12 downto 0) := (others => '0');
    signal n_pp0 : unsigned(8 downto 0) := (others => '0');

    -- [D] source (camera luma or the Osc lattice), smoothed and scaled
    signal s_cin : unsigned(5 downto 0) := (others => '0');
    signal s_cf  : unsigned(9 downto 0) := (others => '0');
    signal s_cg  : unsigned(5 downto 0) := (others => '0');      -- the gained source, 0 .. 59

    -- [C] beam counters
    signal b_xn, b_xf, b_xc, b_xa, b_xb, b_xr : unsigned(13 downto 0) := (others => '0');
    signal b_x  : signed(12 downto 0) := (others => '0');
    signal b_u  : unsigned(28 downto 0) := (others => '0');
    signal b_px : unsigned(10 downto 0) := (others => '0');
    signal b_pxd1, b_pxd2 : unsigned(10 downto 0) := (others => '0');
    signal b_pp : unsigned(8 downto 0) := (others => '0');

    -- near range
    signal n2_a0, n2_a1 : unsigned(2 downto 0) := (others => '0');
    signal n2_f  : unsigned(6 downto 0) := (others => '0');
    signal n2_cp : std_logic := '0';
    signal n3_p  : unsigned(9 downto 0) := (others => '0');
    signal n3_a0 : unsigned(2 downto 0) := (others => '0');
    signal n3_neg, n3_ris, n3_cp : std_logic := '0';
    type t_dvh is array (4 to 13) of unsigned(8 downto 0);
    type t_dvr is array (4 to 13) of unsigned(9 downto 0);
    type t_dvq is array (4 to 13) of unsigned(8 downto 0);
    signal nd_h : t_dvh := (others => (others => '0'));
    signal nd_r : t_dvr := (others => (others => '0'));
    signal nd_q : t_dvq := (others => (others => '0'));
    signal nd_in, nd_ris, nd_cp : std_logic_vector(4 to 13) := (others => '0');
    signal n14_in, n14_bp, n14_cp, n14_ris : std_logic := '0';

    -- far range
    signal f11_a0, f11_a1 : unsigned(2 downto 0) := (others => '0');
    signal f11_f : unsigned(5 downto 0) := (others => '0');
    signal f11_cp : std_logic := '0';
    signal f12_p : unsigned(8 downto 0) := (others => '0');
    signal f12_a0 : unsigned(2 downto 0) := (others => '0');
    signal f12_neg, f12_cp : std_logic := '0';
    signal f13_h : unsigned(9 downto 0) := (others => '0');
    signal f13_cp : std_logic := '0';
    signal f14_in, f14_bp, f14_cp : std_logic := '0';

    -- city
    signal c12_e : unsigned(15 downto 0) := (others => '0');
    signal c12_an, c12_aw, c13_ant : std_logic := '0';
    signal c13_ha : unsigned(9 downto 0) := (others => '0');
    signal c13_st : std_logic_vector(1 downto 0) := "00";
    signal c12_half, c12_cp, c12_wi, c12_c0 : std_logic := '0';
    signal c12_wc : unsigned(2 downto 0) := (others => '0');
    signal c13_h : unsigned(9 downto 0) := (others => '0');
    signal c13_bp : std_logic_vector(1 downto 0) := "00";
    signal c13_cp, c13_lit : std_logic := '0';
    signal c14_in, c14_lit : std_logic := '0';
    signal c14_sub : std_logic_vector(3 downto 0) := (others => '0');

    -- sun, stars, rain
    signal s12_ax, s12_ax2 : unsigned(11 downto 0) := (others => '0');
    signal s13_in, s14_in : std_logic := '0';
    signal t12_h : unsigned(11 downto 0) := (others => '0');
    signal t13_h : unsigned(11 downto 0) := (others => '0');
    signal t12_x, t13_x : unsigned(2 downto 0) := (others => '0');
    signal t14_on : std_logic := '0';
    signal t14_tw : unsigned(1 downto 0) := (others => '0');
    signal r13_e : unsigned(4 downto 0) := (others => '0');
    signal r13_nw, r14_on : std_logic := '0';

    -- sprite slots
    signal a12_xs, b12_xs : unsigned(7 downto 0) := (others => '0');
    signal a12_pr, b12_pr : std_logic := '0';
    signal a13_g, b13_g : std_logic_vector(5 downto 0) := (others => '0');
    signal a13_pr, b13_pr : std_logic := '0';
    signal a14_z, b14_z : unsigned(1 downto 0) := (others => '0');

    -- floor
    signal g14_up, g14_zp, g14_rd, g14_dc, g14_zh : std_logic := '0';

    -- merge, edges, colour
    signal s15_id : std_logic_vector(7 downto 0) := (others => '0');
    signal s15_ramp : unsigned(2 downto 0) := (others => '0');
    signal s15_t : unsigned(6 downto 0) := (others => '0');
    signal s15_fon : std_logic := '0';
    signal s15_ovl : unsigned(1 downto 0) := (others => '0');
    signal s15_tw : unsigned(1 downto 0) := (others => '0');
    signal i_d1, i_d2, i_d3 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_g15 : std_logic_vector(3 downto 0) := (others => '0');
    signal s16_e : std_logic := '0';
    signal s16_m1, s16_m2, s16_m3 : unsigned(3 downto 0) := (others => '0');
    signal s16_ramp : unsigned(2 downto 0) := (others => '0');
    signal s16_t : unsigned(6 downto 0) := (others => '0');
    signal s16_fon : std_logic := '0';
    signal s16_ovl, s16_tw : unsigned(1 downto 0) := (others => '0');
    signal s17_ent : unsigned(4 downto 0) := (others => '0');
    signal s17_t6, s18_t6, s19_t6, s20_t6 : unsigned(5 downto 0) := (others => '0');
    signal s17_blk, s18_blk, s19_blk, s20_blk, s21_blk, s22_blk, s23_blk : std_logic := '1';
    signal s17_w, s18_w, s19_w, s20_w, s21_w, s22_w, s23_w : std_logic := '0';
    signal s24_y, s24_u, s24_v, h_u, h_v : unsigned(9 downto 0) := (others => '0');
    signal h_c : unsigned(2 downto 0) := (others => '0');
    type t_s3 is array (0 to 2) of signed(13 downto 0);
    signal s21_pa, s21_pb, s21_pc : t_s3 := (others => (others => '0'));
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
    signal s_bu, s_bv2, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4100, 13);
    type t_y5 is array (1 to 7) of unsigned(9 downto 0);
    signal s_yd : t_y5 := (others => to_unsigned(64, 10));
    signal s_hist : std_logic_vector(7 downto 0) := (others => '0');
    signal o1_ya : unsigned(11 downto 0) := (others => '0');
    signal o1_yb, o2_y, o2_u, o2_v, o3_y, o3_u, o3_v, o4_y, o4_u, o4_v : unsigned(9 downto 0) := (others => '0');
begin

    ---------------------------------------------------------------------------
    -- ROMs and the ID line store (one write statement; registered reads, re-registered in fabric)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lr_q   <= C_NS_LROM(to_integer(lr_a));
            lr_qf  <= lr_q;
            pal_q  <= C_NS_PAL(to_integer(pal_a));
            pal_qf <= pal_q;
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

    w_x0 <= to_signed(-1440, 14) when s_xs = 4 else to_signed(-1920, 14);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + the per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_p4 : unsigned(3 downto 0);
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

            -- [B] speed: bipolar with a +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_goff  <= registers_in(6)(0);
                s_sharp <= registers_in(6)(1);
                s_pan   <= registers_in(6)(2);
                s_lake  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_land  <= f_zone8(unsigned(registers_in(0)), s_land);
                s_sky   <= f_zone8(unsigned(registers_in(1)), s_sky);
                s_pal   <= f_zone8(unsigned(registers_in(4)), s_pal);
                -- wire thickness: 3 x 2 at 1080p, 2 x 2 at 720p, 3 x 1 at 1080i, 1 x 1 at SD
                if s_xsr = 2 then s_th2 <= '1'; s_th3 <= '1';
                elsif s_xsr = 3 then s_th2 <= '1'; s_th3 <= '0';
                else s_th2 <= '0'; s_th3 <= '0'; end if;
                if s_ys <= 384 then s_tv2 <= '1'; else s_tv2 <= '0'; end if;
            end if;
            g_wm1 <= s_w - 1;

            -- scene tables (free-running decode of the latched zones)
            c_efar <= '0'; c_ecity <= '0'; c_enear <= '0'; c_epalm <= '0'; c_eroad <= '0'; c_tun <= '0';
            case to_integer(s_land) is
                when 0      => null;                                                            -- Open
                when 1      => c_efar <= '1';                                                   -- Hills
                when 2      => c_efar <= '1'; c_enear <= '1';                                   -- Ranges
                when 3      => c_ecity <= '1';                                                  -- Metro
                when 4      => c_ecity <= '1'; c_enear <= '1';                                  -- Valley
                when 5      => c_enear <= '1'; c_epalm <= '1'; c_eroad <= '1';                  -- Palms
                when 6      => c_ecity <= '1'; c_epalm <= '1'; c_eroad <= '1';                  -- Strip
                when others => c_tun <= '1'; c_eroad <= '1';                                    -- Tunnel
            end case;
            c_esun <= '0'; c_emoon <= '0'; c_estar <= '0'; c_ecl1 <= '0'; c_ecl2 <= '0'; c_ebird <= '0'; c_erain <= '0';
            case to_integer(s_sky) is
                when 0      => c_esun <= '1';                                                   -- Sun
                when 1      => c_esun <= '1'; c_ecl1 <= '1';                                    -- Sun Clouds
                when 2      => c_esun <= '1'; c_ebird <= '1';                                   -- Sun Birds
                when 3      => c_esun <= '1'; c_estar <= '1'; c_ecl1 <= '1'; c_ebird <= '1';    -- Dusk
                when 4      => c_emoon <= '1'; c_estar <= '1';                                  -- Moon
                when 5      => c_emoon <= '1'; c_estar <= '1'; c_ecl1 <= '1'; c_ecl2 <= '1';    -- Moon Clouds
                when 6      => c_emoon <= '1'; c_ecl1 <= '1'; c_ecl2 <= '1'; c_erain <= '1';    -- Storm
                when others => c_estar <= '1'; c_ebird <= '1';                                  -- Stars
            end case;
            if c_emoon = '1' then
                c_sunc <= to_unsigned(330, 9); c_sx <= to_signed(380, 12);
            else
                c_sunc <= to_unsigned(144, 9); c_sx <= to_signed(0, 12);
            end if;

            -- [B] shared glide (1/8 of the gap per field, +-2 deadband): horizon, camera, fill
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "00"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 1      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when 0      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 2 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] per-field constants, one small step per state
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>
                        c_t <= c_t + 1;
                        if s_pan = '0' then                 -- Drive: flight
                            if s_vneg = '1' then c_zph <= c_zph - resize(s_vm(8 downto 1), 17);
                            else c_zph <= c_zph + resize(s_vm(8 downto 1), 17); end if;
                        end if;
                    when 41 =>
                        if s_pan = '1' then                 -- Pan: lateral travel
                            if s_vneg = '1' then c_pan <= c_pan - resize(s_vm, 22);
                            else c_pan <= c_pan + resize(s_vm, 22); end if;
                        end if;
                    when 42 =>
                        if s_pan = '1' then
                            if s_vneg = '1' then c_uph <= c_uph - shift_left(resize(s_vm, 29), 9);
                            else c_uph <= c_uph + shift_left(resize(s_vm, 29), 9); end if;
                        end if;
                    when 43 =>                              -- horizon: 270 + k / 2 + k / 32 rows
                        c_hz <= to_unsigned(270, 11) + resize(s_g3(12 downto 4), 11) + resize(s_g3(12 downto 8), 11);
                    when 44 =>
                        c_bot <= to_unsigned(1080, 11) - c_hz;
                    when 45 =>                              -- the car sits 40 rows off the bottom, bobbing a row
                        if c_t(3) = '1' then c_cart <= signed(resize(c_bot, 12)) - 232;
                        else c_cart <= signed(resize(c_bot, 12)) - 233; end if;
                    when 46 =>
                        if s_goff = '1' then c_fp <= (others => '0'); else c_fp <= resize(s_gp, 14); end if;
                        c_cgn <= s_g6(12 downto 9);
                    when 47 =>                              -- layer scrolls (half units): far .. near, palms fastest
                        c_sf <= c_pan(21 downto 8);
                        c_sc <= c_pan(20 downto 7);
                        c_sn <= c_pan(19 downto 6);
                        c_sb <= c_pan(18 downto 5);
                    when 48 =>                              -- clouds and birds drift on their own
                        c_sa1 <= resize(c_pan(21 downto 9), 14) + c_t(13 downto 0);
                        c_sab <= resize(c_pan(21 downto 9), 14) + (c_t(11 downto 0) & "00");
                    when 49 =>
                        c_sa2 <= c_sa1 + ('0' & c_t(13 downto 1)) + 3000;
                        v_p4 := c_t(6 downto 3);            -- palm sway: a triangle, +-7
                        if c_t(7) = '1' then v_p4 := not v_p4; end if;
                        c_swm <= v_p4(3 downto 1);
                        c_swn <= c_t(8);
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
    -- computes the NEXT line's constants in 48 clocks of horizontal blanking: one 22-step divide, ten table
    -- reads, shifts and compares.  The l_* constants are written at state 32 or later, after the previous
    -- line's last pixel has cleared the 31-stage pixel path, so there is one copy of each.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y   : unsigned(10 downto 0);
        variable v_d   : unsigned(11 downto 0);
        variable v_a   : unsigned(10 downto 0);
        variable v_b   : signed(10 downto 0);
        variable v_r2  : unsigned(11 downto 0);
        variable v_row : unsigned(7 downto 0);
        variable v_t   : unsigned(9 downto 0);
        variable v_g   : unsigned(4 downto 0);
        variable v_th  : unsigned(4 downto 0);
        variable v_sw  : unsigned(8 downto 0);
        variable v_o   : unsigned(13 downto 0);
        variable v_w   : signed(4 downto 0);
        variable v_zz  : unsigned(21 downto 0);
        variable v_l   : unsigned(2 downto 0);
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

            -- the divider (restoring, one quotient bit per clock): states 3 .. 24
            if e_lc >= 3 and e_lc <= 24 then
                v_r2 := dv_r & dv_n(21);
                dv_n <= dv_n(20 downto 0) & '0';
                if v_r2 >= resize(dv_d, 12) then
                    dv_r <= resize(v_r2 - resize(dv_d, 12), 11);
                    dv_q <= dv_q(20 downto 0) & '1';
                else
                    dv_r <= v_r2(10 downto 0);
                    dv_q <= dv_q(20 downto 0) & '0';
                end if;
            end if;

            -- fill flags (states 40 .. 47, ramp = the state's low bits): ramp i is filled out to
            -- (fill - order x 1120) rows from the horizon (8 rows a knob count)
            if e_lc(6 downto 3) = "0101" then
                if resize(e_d, 14) + e_fc < c_fp then l_fill(to_integer(e_lc(2 downto 0))) <= '1';
                else l_fill(to_integer(e_lc(2 downto 0))) <= '0'; end if;
            end if;
            case to_integer(e_lc(2 downto 0) + 1) is
                when 1      => e_fc <= to_unsigned(0, 14);        -- sun
                when 0      => e_fc <= to_unsigned(1120, 14);     -- sky
                when 2      => e_fc <= to_unsigned(2240, 14);     -- far range
                when 3      => e_fc <= to_unsigned(3360, 14);     -- city
                when 4      => e_fc <= to_unsigned(4480, 14);     -- near range
                when 5      => e_fc <= to_unsigned(5600, 14);     -- floor
                when others => e_fc <= to_unsigned(6720, 14);     -- flora, car
            end case;

            v_y := e_vy(18 downto 8);
            case to_integer(e_lc) is
                when 0 =>
                    e_s <= signed(resize(v_y, 12)) - signed(resize(c_hz, 12));
                when 1 =>
                    if e_s(11) = '1' then v_d := unsigned(-e_s); else v_d := unsigned(e_s); end if;
                    if v_d(11 downto 10) /= "00" then e_d <= (others => '1'); else e_d <= v_d(9 downto 0); end if;
                    e_ab <= e_s(11);
                when 2 =>
                    -- which half is this line: sky objects (above, or mirrored in the lake), floor objects
                    e_sky <= e_ab or s_lake;
                    e_obj <= (e_ab or s_lake) and not c_tun;
                    e_low <= not e_ab and not s_lake;
                    e_mir <= not e_ab and s_lake;
                    e_dyu <= signed(resize(e_d, 11)) - signed(resize(c_sunc, 11));
                    e_c1  <= signed(resize(e_d, 11)) - 330;
                    e_c2  <= signed(resize(e_d, 11)) - 470;
                    e_bd  <= signed(resize(e_d, 11)) - 250;
                    e_cr  <= e_s - c_cart;
                    e_cs  <= signed(resize(v_y, 12)) - 943;
                    dv_n <= to_unsigned(C_G, 22); dv_r <= (others => '0'); dv_q <= (others => '0');
                    if e_d = 0 then dv_d <= to_unsigned(1, 10); else dv_d <= e_d; end if;
                when 3 =>
                    -- sun / moon: half-chord address, slits, ramp position
                    if e_dyu(10) = '1' then v_a := unsigned(-e_dyu); else v_a := unsigned(e_dyu); end if;
                    e_son <= '0';
                    if c_emoon = '1' then
                        e_sa <= v_a(6 downto 0) & '0';
                        if v_a < 128 and e_sky = '1' then e_son <= '1'; end if;
                        v_b := e_dyu + 128;
                        e_tsun <= unsigned(v_b(7 downto 1));
                    else
                        e_sa <= v_a(7 downto 0);
                        if v_a < 256 and e_sky = '1' and c_esun = '1' then
                            -- slits below the centre: period 32 rows, opening with depth, rolling
                            v_g := v_a(4 downto 0) + c_t(5 downto 1);
                            v_th := v_a(7 downto 3) + resize(v_a(7 downto 4), 5);
                            if e_dyu(10) = '0' or v_g >= v_th then e_son <= '1'; end if;
                        end if;
                        v_b := e_dyu + 256;
                        e_tsun <= unsigned(v_b(8 downto 2));
                    end if;
                    -- slot A: the cloud or bird band this line crosses
                    e_aen <= '0'; e_asel <= "00"; e_asc <= c_sa1;
                    e_aa <= (others => '0');
                    if e_obj = '1' then
                        if c_ecl1 = '1' and e_c1 >= 0 and e_c1 < 96 then
                            v_row := to_unsigned(47, 8) - unsigned(e_c1(8 downto 1));
                            e_aa <= to_unsigned(1600, 11) + (v_row & "00");
                            e_aen <= '1';
                        elsif c_ecl2 = '1' and e_c2 >= 0 and e_c2 < 96 then
                            v_row := to_unsigned(47, 8) - unsigned(e_c2(8 downto 1));
                            e_aa <= to_unsigned(1600, 11) + (v_row & "00");
                            e_asel <= "01"; e_asc <= c_sa2;
                            e_aen <= '1';
                        elsif c_ebird = '1' and e_bd >= 0 and e_bd < 32 then
                            v_row := to_unsigned(31, 8) - unsigned(e_bd(7 downto 0));
                            if c_t(4) = '1' then e_aa <= to_unsigned(1920, 11) + (v_row & "00");
                            else e_aa <= to_unsigned(1792, 11) + (v_row & "00"); end if;
                            e_asel <= "10"; e_asc <= c_sab;
                            e_aen <= '1';
                        end if;
                    end if;
                    -- slot B: palms above the horizon, the car below it
                    e_ben <= '0'; e_bpalm <= '0'; e_bfold <= '0'; e_bunit <= '0';
                    e_ba <= (others => '0');
                    if e_obj = '1' and c_epalm = '1' and e_d < 384 then
                        v_row := to_unsigned(191, 8) - e_d(8 downto 1);
                        e_ba <= to_unsigned(256, 11) + (v_row & "00");
                        e_ben <= '1'; e_bpalm <= '1';
                    elsif e_low = '1' and c_eroad = '1' and s_pan = '0' and e_cr >= 0 and e_cr < 192 then
                        e_ba <= to_unsigned(1024, 11) + (unsigned(e_cr(7 downto 1)) & "00");
                        e_bfold <= '1';
                        e_ben <= '1';
                    elsif e_low = '1' and c_eroad = '1' and s_pan = '1' and e_cs >= 0 and e_cs < 96 then
                        e_ba <= to_unsigned(1408, 11) + (unsigned(e_cs(6 downto 1)) & "00");
                        e_bunit <= '1';
                        e_ben <= '1';
                    end if;
                when 4 =>                                    -- the moon's bite: a disc 24 rows up
                    v_b := e_dyu - 24;
                    if v_b(10) = '1' then v_a := unsigned(-v_b); else v_a := unsigned(v_b); end if;
                    e_sb <= v_a(6 downto 0) & '0';
                    if c_emoon = '1' and v_a < 128 then e_bon <= '1'; else e_bon <= '0'; end if;
                when 5 =>                                    -- the line's left edge (the lake wobbles it)
                    v_w := (others => '0');
                    if e_mir = '1' then
                        v_g := resize(e_d(4 downto 1) + c_t(5 downto 2), 5);
                        if v_g(3) = '1' then v_w := signed('0' & (not v_g(2 downto 0)) & '0') - 7;
                        else v_w := signed('0' & v_g(2 downto 0) & '0') - 7; end if;
                    end if;
                    e_x0 <= w_x0 + resize(v_w, 14);
                when 6  => n_xn0 <= unsigned(e_x0) + c_sn;
                when 7  => n_xf0 <= unsigned(e_x0) + c_sf;
                when 8  => n_xc0 <= unsigned(e_x0) + c_sc;
                when 9  => n_xa0 <= unsigned(e_x0) + e_asc;
                when 10 =>
                    if e_bpalm = '1' then                    -- palms sway: the crown moves, the foot stays
                        v_sw := (others => '0');
                        if c_swm(0) = '1' then v_sw := v_sw + resize(e_d(8 downto 4), 9); end if;
                        if c_swm(1) = '1' then v_sw := v_sw + resize(e_d(8 downto 4) & '0', 9); end if;
                        if c_swm(2) = '1' then v_sw := v_sw + resize(e_d(8 downto 4) & "00", 9); end if;
                        v_o := resize(v_sw(8 downto 2), 14);
                        if c_swn = '1' then n_xb0 <= unsigned(e_x0) + c_sb - v_o;
                        else n_xb0 <= unsigned(e_x0) + c_sb + v_o; end if;
                    else
                        n_xb0 <= unsigned(e_x0) + 512;       -- the side-view car: 512 wide about the centre
                    end if;
                when 11 =>
                    n_x0 <= e_x0(12 downto 0);
                    n_xr0 <= unsigned(e_x0) + resize(v_y(10 downto 1), 14) + (c_t(9 downto 0) & "00");
                when 12 =>
                    n_pp0 <= unsigned(e_x0(8 downto 0)) + (v_y(7 downto 0) & '0') + (c_t(5 downto 0) & "000");
                when 26 =>
                    e_du <= dv_q;
                when 27 =>                                   -- du x half the width
                    if s_xs = 4 then
                        e_m <= shift_left(resize(e_du, 29), 9) + shift_left(resize(e_du, 29), 7);
                    else
                        e_m <= shift_left(resize(e_du, 29), 10) - shift_left(resize(e_du, 29), 6);
                    end if;
                when 28 =>
                    if s_xs = 4 then
                        e_m <= e_m + shift_left(resize(e_du, 29), 6) + shift_left(resize(e_du, 29), 4);
                    end if;
                when 29 =>
                    n_u0 <= c_uph - e_m;
                when 32 =>
                    -- the line's constants (the previous line has left the pixel path by now)
                    l_d <= e_d; l_ab <= e_ab; l_obj <= e_obj; l_mir <= e_mir;
                    l_flr <= e_low or (e_ab and c_tun);
                    l_son <= e_son; l_bon <= e_bon; l_tsun <= e_tsun;
                    l_asel <= e_asel; l_bfold <= e_bfold; l_bunit <= e_bunit;
                    if e_asel = "10" then l_ta <= (others => '0'); else l_ta <= to_unsigned(104, 7); end if;
                    -- per-pixel lane step
                    case to_integer(s_xs) is
                        when 4      => l_dup <= resize(e_du & '0', 24);
                        when 3      => l_dup <= resize(e_du, 24) + resize(e_du(21 downto 1), 24);
                        when others => l_dup <= resize(e_du, 24);
                    end case;
                    -- grid level of detail: toward the horizon the lanes and rows double their pitch every
                    -- factor of 1.41 in distance, so rows stay about 6 or more lines apart and so do the lanes at the screen's edge
                    v_zz := e_du + resize(c_zph, 22);
                    -- (thresholds 128 / 96 / 64 / 48 / 32 / 24 / 16 rows: bit tests)
                    if e_d(9 downto 7) /= "000" then v_l := "000";
                    elsif e_d(6 downto 5) = "11" then v_l := "001";
                    elsif e_d(6) = '1' then v_l := "010";
                    elsif e_d(5 downto 4) = "11" then v_l := "011";
                    elsif e_d(5) = '1' then v_l := "100";
                    elsif e_d(4 downto 3) = "11" then v_l := "101";
                    elsif e_d(4) = '1' then v_l := "110";
                    else v_l := "111"; end if;
                    l_lod <= v_l;
                    if v_l = "111" then l_gfar <= '1'; else l_gfar <= '0'; end if;
                    case v_l is
                        when "000"  => l_zf <= v_zz(11 downto 0);
                        when "001"  => l_zf <= v_zz(12 downto 1);
                        when "010"  => l_zf <= v_zz(13 downto 2);
                        when "011"  => l_zf <= v_zz(14 downto 3);
                        when "100"  => l_zf <= v_zz(15 downto 4);
                        when "101"  => l_zf <= v_zz(16 downto 5);
                        when others => l_zf <= v_zz(17 downto 6);
                    end case;
                    -- ramp positions
                    v_t := shift_right(e_d, 2);
                    if v_t > 127 then l_tsky <= (others => '0'); else l_tsky <= to_unsigned(127, 7) - v_t(6 downto 0); end if;
                    if e_d > 127 then l_tflr <= (others => '0'); else l_tflr <= to_unsigned(127, 7) - e_d(6 downto 0); end if;
                    if e_d(9 downto 8) /= "00" then l_tfar <= (others => '1'); else l_tfar <= e_d(7 downto 1); end if;
                    if e_d(9) = '1' then l_tcty <= (others => '1'); else l_tcty <= e_d(8 downto 3); end if;
                    if v_t > 95 then l_tnr <= to_unsigned(95, 7); else l_tnr <= v_t(6 downto 0); end if;
                    -- lake ripples: gaps of a 16-row period that open with depth, drifting
                    v_g := resize(e_d(3 downto 0) + c_t(4 downto 1), 5);
                    if e_mir = '1' and v_g(3 downto 0) <= e_d(9 downto 6) then l_rip <= '1'; else l_rip <= '0'; end if;
                    -- the Pan road: a band near the bottom of the screen, a dashed stripe in its middle
                    l_proad <= '0'; l_pdash <= '0';
                    if e_low = '1' and c_eroad = '1' and s_pan = '1' and v_y >= 905 and v_y < 1056 then
                        l_proad <= '1';
                        if v_y >= 976 and v_y < 984 then l_pdash <= '1'; end if;
                    end if;
                    if e_ln /= 0 then l_v1 <= '1'; else l_v1 <= '0'; end if;
                    if e_ln(1) = '1' and s_tv2 = '1' then l_v2 <= '1'; else l_v2 <= '0'; end if;
                    l_wf <= e_d(7 downto 5) + c_t(8 downto 6);
                    l_rf <= v_y(7 downto 4) + c_t(3 downto 0) + c_t(4 downto 1);
                    l_oy <= f_tri7(v_y(9 downto 2) + c_t(8 downto 1));
                    lr_a <= "000" & e_sa;
                when 33 => lr_a <= "000" & e_sb;
                when 34 => lr_a <= e_aa;
                when 35 => lr_a <= e_aa + 1;
                    if c_emoon = '1' then l_sw <= "00" & lr_qf(7 downto 1); else l_sw <= '0' & lr_qf(7 downto 0); end if;
                when 36 => lr_a <= e_aa + 2;
                    l_bw <= "00" & lr_qf(7 downto 1);
                when 37 => lr_a <= e_aa + 3;
                    l_arow(0) <= lr_qf;
                when 38 => lr_a <= e_ba;
                    l_arow(1) <= lr_qf;
                when 39 => lr_a <= e_ba + 1;
                    l_arow(2) <= lr_qf;
                when 40 => lr_a <= e_ba + 2;
                    if e_aen = '1' then l_arow(3) <= lr_qf; else l_arow(3) <= (others => '0'); end if;
                when 41 => lr_a <= e_ba + 3;
                    l_brow(0) <= lr_qf;
                when 42 => l_brow(1) <= lr_qf;
                when 43 => l_brow(2) <= lr_qf;
                when 44 =>
                    if e_ben = '1' then l_brow(3) <= lr_qf; else l_brow(3) <= (others => '0'); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

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
    -- [C] BEAM COUNTERS: each layer's world x (half units), loaded so that its results reach stage 14
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1'  then b_xn <= n_xn0; else b_xn <= b_xn + s_xs; end if;      -- stage 1
            if s_st(9) = '1'  then b_xf <= n_xf0; else b_xf <= b_xf + s_xs; end if;      -- stage 10
            if s_st(10) = '1' then
                b_xc <= n_xc0; b_xa <= n_xa0; b_xb <= n_xb0; b_x <= n_x0;                -- stage 11
            else
                b_xc <= b_xc + s_xs; b_xa <= b_xa + s_xs; b_xb <= b_xb + s_xs;
                b_x <= b_x + signed(resize(s_xs, 13));
            end if;
            if s_st(11) = '1' then b_xr <= n_xr0; else b_xr <= b_xr + s_xs; end if;      -- stage 12
            if s_st(12) = '1' then                                                       -- stage 13
                b_u <= n_u0; b_px <= (others => '0');
            else
                b_u <= b_u + resize(l_dup, 29); b_px <= b_px + 1;
            end if;
            b_pxd1 <= b_px; b_pxd2 <= b_pxd1;
            if s_st(15) = '1' then b_pp <= n_pp0; else b_pp <= b_pp + s_xs; end if;      -- stage 16
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [C] PIXEL PATH, stages 2 .. 14: one block per object, each ending in a few bits at stage 14
    ---------------------------------------------------------------------------
    p_obj : process(clk)
        variable v_e6  : unsigned(5 downto 0);
        variable v_df  : signed(3 downto 0);
        variable v_dm  : unsigned(2 downto 0);
        variable v_dn  : signed(3 downto 0);
        variable v_xn  : signed(12 downto 0);
        variable v_g0, v_g1, v_g2 : unsigned(9 downto 0);
        variable v_h   : signed(11 downto 0);
        variable v_r2  : unsigned(10 downto 0);
        variable v_q   : unsigned(8 downto 0);
        variable v_e9  : unsigned(8 downto 0);
        variable v_h4  : unsigned(3 downto 0);
        variable v_xr  : signed(12 downto 0);
        variable v_a12 : unsigned(11 downto 0);
        variable v_hh  : unsigned(11 downto 0);
        variable v_cell : unsigned(3 downto 0);
        variable v_xs  : unsigned(7 downto 0);
        variable v_pr  : std_logic;
        variable v_ax  : unsigned(11 downto 0);
        variable v_e5  : unsigned(4 downto 0);
        variable v_n4  : unsigned(3 downto 0);
        variable v_zs  : unsigned(11 downto 0);
        variable v_wi  : unsigned(5 downto 0);
        variable v_c   : unsigned(9 downto 0);
        variable v_st  : std_logic_vector(1 downto 0);
        variable v_an  : std_logic;
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------------
            -- near range: knots 128 px apart, 64 rows a unit; mesh rows at k / 8 of the height
            ----------------------------------------------------------------
            -- 2
            v_e6 := C_NS_NEAR(to_integer(b_xn(13 downto 8)));
            n2_a0 <= v_e6(2 downto 0); n2_a1 <= v_e6(5 downto 3);
            n2_f  <= b_xn(7 downto 1);
            n2_cp <= b_xn(7);
            -- 3: |A1 - A0| x f as three gated operands
            v_df := signed(resize(n2_a1, 4)) - signed(resize(n2_a0, 4));
            v_dn := -v_df;
            if v_df(3) = '1' then v_dm := unsigned(v_dn(2 downto 0)); else v_dm := unsigned(v_df(2 downto 0)); end if;
            v_g0 := (others => '0'); v_g1 := (others => '0'); v_g2 := (others => '0');
            if v_dm(0) = '1' then v_g0 := resize(n2_f, 10); end if;
            if v_dm(1) = '1' then v_g1 := resize(n2_f & '0', 10); end if;
            if v_dm(2) = '1' then v_g2 := resize(n2_f & "00", 10); end if;
            n3_p <= v_g0 + v_g1 + v_g2;
            n3_neg <= v_df(3);
            if v_df > 0 then n3_ris <= '1'; else n3_ris <= '0'; end if;
            n3_a0 <= n2_a0; n3_cp <= n2_cp;
            -- 4: the height (rows), lifted by the camera
            if n3_neg = '1' then
                v_h := signed(resize(n3_a0 & "000000", 12)) - signed(resize(n3_p(9 downto 1), 12));
            else
                v_h := signed(resize(n3_a0 & "000000", 12)) + signed(resize(n3_p(9 downto 1), 12));
            end if;
            v_h := v_h + signed(resize(s_cg & "00", 12));
            if v_h(11 downto 9) /= "000" then nd_h(4) <= to_unsigned(511, 9); else nd_h(4) <= unsigned(v_h(8 downto 0)); end if;
            nd_ris(4) <= n3_ris; nd_cp(4) <= n3_cp;
            -- 5: inside test and the divide's start
            if l_d(9) = '0' and l_d(8 downto 0) < nd_h(4) then nd_in(5) <= '1'; else nd_in(5) <= '0'; end if;
            nd_r(5) <= resize(l_d(8 downto 0), 10);
            nd_h(5) <= nd_h(4); nd_q(5) <= (others => '0');
            nd_ris(5) <= nd_ris(4); nd_cp(5) <= nd_cp(4);
            -- 6 .. 13: eight of the nine quotient bits, one per stage (q = 512 d / h)
            for i in 6 to 13 loop
                v_r2 := nd_r(i - 1) & '0';
                if v_r2 >= resize(nd_h(i - 1), 11) then
                    nd_r(i) <= resize(v_r2 - resize(nd_h(i - 1), 11), 10);
                    nd_q(i) <= nd_q(i - 1)(7 downto 0) & '1';
                else
                    nd_r(i) <= v_r2(9 downto 0);
                    nd_q(i) <= nd_q(i - 1)(7 downto 0) & '0';
                end if;
                nd_h(i) <= nd_h(i - 1); nd_in(i) <= nd_in(i - 1);
                nd_ris(i) <= nd_ris(i - 1); nd_cp(i) <= nd_cp(i - 1);
            end loop;
            -- 14: the ninth bit, the flow phase; bands of 1/8 of the height
            v_r2 := nd_r(13) & '0';
            if v_r2 >= resize(nd_h(13), 11) then v_q := nd_q(13)(7 downto 0) & '1'; else v_q := nd_q(13)(7 downto 0) & '0'; end if;
            v_q := v_q - c_t(8 downto 0);
            n14_bp <= v_q(6);
            n14_in <= nd_in(13) and c_enear; n14_ris <= nd_ris(13); n14_cp <= nd_cp(13);

            ----------------------------------------------------------------
            -- far range: knots 64 px apart, 32 rows a unit; contour lines 32 rows apart below the ridge
            ----------------------------------------------------------------
            -- 11
            v_e6 := C_NS_FAR(to_integer(b_xf(12 downto 7)));
            f11_a0 <= v_e6(2 downto 0); f11_a1 <= v_e6(5 downto 3);
            f11_f  <= b_xf(6 downto 1);
            f11_cp <= b_xf(6);
            -- 12
            v_df := signed(resize(f11_a1, 4)) - signed(resize(f11_a0, 4));
            v_dn := -v_df;
            if v_df(3) = '1' then v_dm := unsigned(v_dn(2 downto 0)); else v_dm := unsigned(v_df(2 downto 0)); end if;
            v_g0 := (others => '0'); v_g1 := (others => '0'); v_g2 := (others => '0');
            if v_dm(0) = '1' then v_g0 := resize(f11_f, 10); end if;
            if v_dm(1) = '1' then v_g1 := resize(f11_f & '0', 10); end if;
            if v_dm(2) = '1' then v_g2 := resize(f11_f & "00", 10); end if;
            v_g0 := v_g0 + v_g1 + v_g2;
            f12_p <= v_g0(8 downto 0);
            f12_neg <= v_df(3); f12_a0 <= f11_a0; f12_cp <= f11_cp;
            -- 13
            if f12_neg = '1' then
                v_h := signed(resize(f12_a0 & "00000", 12)) - signed(resize(f12_p(8 downto 1), 12));
            else
                v_h := signed(resize(f12_a0 & "00000", 12)) + signed(resize(f12_p(8 downto 1), 12));
            end if;
            v_h := v_h + signed(resize(s_cg & '0', 12)) + 24;
            f13_h <= unsigned(v_h(9 downto 0));
            f13_cp <= f12_cp;
            -- 14
            if l_d < f13_h then f14_in <= c_efar; else f14_in <= '0'; end if;
            v_c := f13_h - l_d + resize(c_t(6 downto 1), 10);
            f14_bp <= v_c(5);
            f14_cp <= f13_cp;

            ----------------------------------------------------------------
            -- city: cells 128 px wide holding one wide tower or two narrow ones; floors and bays of 32
            ----------------------------------------------------------------
            -- 12
            c12_e <= C_NS_CITY(to_integer(b_xc(13 downto 8)));
            c12_half <= b_xc(7); c12_cp <= b_xc(6); c12_c0 <= b_xc(8);
            c12_wc <= b_xc(8 downto 6);
            c12_wi <= b_xc(5) xor b_xc(4);
            if b_xc(6 downto 2) = "10000" then c12_an <= '1'; else c12_an <= '0'; end if;     -- a narrow tower's centre
            if b_xc(7 downto 2) = "100000" then c12_aw <= '1'; else c12_aw <= '0'; end if;    -- a wide tower's centre
            -- 13
            if c12_e(8) = '0' and c12_half = '1' then
                v_h4 := c12_e(7 downto 4); v_st := std_logic_vector(c12_e(12 downto 11)); v_an := c12_e(14) and c12_an;
            else
                v_h4 := c12_e(3 downto 0); v_st := std_logic_vector(c12_e(10 downto 9));
                if c12_e(8) = '1' then v_an := c12_e(13) and c12_aw; else v_an := c12_e(13) and c12_an; end if;
            end if;
            c13_h  <= resize(v_h4 & "00000", 10) + resize(s_cg & "00", 10);
            c13_ha <= resize(v_h4 & "00000", 10) + resize(s_cg & "00", 10) + 56;
            c13_ant <= v_an; c13_st <= v_st;
            c13_bp <= c12_c0 & (c12_half and not c12_e(8));
            c13_cp <= c12_cp;
            v_wi := c12_wc & l_wf;
            c13_lit <= C_NS_WIN(to_integer(v_wi)) and c12_wi and (l_d(4) xor l_d(3));
            -- 14
            -- (the tower's style picks its wire pattern: floors, bays, both or neither; some carry a mast)
            if l_d < c13_h or (c13_ant = '1' and l_d < c13_ha) then c14_in <= c_ecity; else c14_in <= '0'; end if;
            c14_sub <= c13_bp & (l_d(5) and c13_st(1)) & (c13_cp and c13_st(0));
            c14_lit <= c13_lit;

            ----------------------------------------------------------------
            -- sun / moon
            ----------------------------------------------------------------
            -- 12
            v_xr := resize(b_x(12 downto 1), 13) - resize(c_sx, 13);
            v_xn := -v_xr;
            if v_xr(12) = '1' then s12_ax <= unsigned(v_xn(11 downto 0)); else s12_ax <= unsigned(v_xr(11 downto 0)); end if;
            v_xr := v_xr - 50;
            v_xn := -v_xr;
            if v_xr(12) = '1' then s12_ax2 <= unsigned(v_xn(11 downto 0)); else s12_ax2 <= unsigned(v_xr(11 downto 0)); end if;
            -- 13
            s13_in <= '0';
            if l_son = '1' and s12_ax < resize(l_sw, 12) then
                if l_bon = '0' or s12_ax2 >= resize(l_bw, 12) then s13_in <= '1'; end if;
            end if;
            -- 14
            s14_in <= s13_in;

            ----------------------------------------------------------------
            -- stars: 32 x 32 cells, one in sixteen holds a 4 x 4 star; one phase in four it is dimmed
            ----------------------------------------------------------------
            -- 12
            v_a12 := unsigned(b_x(12 downto 6)) & l_d(9 downto 5);
            t12_h <= v_a12 + shift_left(v_a12, 3) + shift_left(v_a12, 7) + 693;
            t12_x <= unsigned(b_x(5 downto 3));
            -- 13
            v_hh := t12_h xor ("0000" & t12_h(11 downto 4));
            t13_h <= v_hh + shift_left(v_hh, 5) + shift_left(v_hh, 2) + 1447;
            t13_x <= t12_x;
            -- 14
            v_hh := t13_h xor ("0000000" & t13_h(11 downto 7));
            if v_hh(11 downto 8) = "0000" and v_hh(2 downto 0) = t13_x and v_hh(5 downto 3) = l_d(4 downto 2) then
                t14_on <= c_estar;
            else
                t14_on <= '0';
            end if;
            if v_hh(7 downto 6) = c_t(6 downto 5) then t14_tw <= "00"; else t14_tw <= "11"; end if;

            ----------------------------------------------------------------
            -- rain: 16-px columns, slanted; dashes of 96 rows in every 256, falling
            ----------------------------------------------------------------
            -- 13
            r13_e <= C_NS_RAIN(to_integer(b_xr(9 downto 5)));
            if b_xr(4 downto 2) = "000" then r13_nw <= '1'; else r13_nw <= '0'; end if;
            -- 14
            v_n4 := l_rf + r13_e(3 downto 0);
            if r13_e(4) = '1' and r13_nw = '1' and v_n4 < 6 then r14_on <= c_erain; else r14_on <= '0'; end if;

            ----------------------------------------------------------------
            -- sprite slots: span-coded rows.  A = clouds / birds, B = palms / car
            ----------------------------------------------------------------
            -- 12: lattice cell, presence, x inside the sprite
            if l_asel(1) = '0' then v_cell := b_xa(13 downto 10); a12_xs <= b_xa(9 downto 2);
            else v_cell := b_xa(12 downto 9); a12_xs <= b_xa(8 downto 1); end if;
            a12_pr <= f_amask(l_asel, v_cell);

            if b_x(12) = '1' then v_ax := unsigned(-b_x(12 downto 1)); else v_ax := unsigned(b_x(12 downto 1)); end if;
            if l_bunit = '1' then                           -- the side-view car: the one cell about the centre
                v_xs := b_xb(9 downto 2);
                if b_xb(13 downto 10) = "0000" then v_pr := '1'; else v_pr := '0'; end if;
            else                                            -- palms
                v_cell := b_xb(12 downto 9); v_xs := b_xb(8 downto 1);
                v_pr := C_PMASK(to_integer(v_cell));
                if C_PFLIP(to_integer(v_cell)) = '1' then v_xs := not v_xs; end if;
            end if;
            if l_bfold = '1' then
                v_xs := v_ax(7 downto 0);
                if v_ax(11 downto 8) = "0000" then v_pr := '1'; else v_pr := '0'; end if;
            end if;
            b12_xs <= v_xs; b12_pr <= v_pr;
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

            ----------------------------------------------------------------
            -- floor: lanes from the accumulator, rows from z + flight + the camera's lift
            ----------------------------------------------------------------
            -- 14
            case l_lod is
                when "000"  => g14_up <= b_u(20);
                when "001"  => g14_up <= b_u(21);
                when "010"  => g14_up <= b_u(22);
                when "011"  => g14_up <= b_u(23);
                when "100"  => g14_up <= b_u(24);
                when "101"  => g14_up <= b_u(25);
                when "110"  => g14_up <= b_u(26);
                when others => g14_up <= '0';
            end case;
            if b_u(28 downto 21) = "00000000" or b_u(28 downto 21) = "11111111" then g14_rd <= '1'; else g14_rd <= '0'; end if;
            if (b_u(28 downto 20) = "000000000" and b_u(19 downto 17) = "000")
               or (b_u(28 downto 20) = "111111111" and b_u(19 downto 17) = "111") then g14_dc <= '1'; else g14_dc <= '0'; end if;
            v_zs := l_zf + (s_cg & "00000");
            g14_zp <= v_zs(10);
            g14_zh <= v_zs(9);
        end if;
    end process p_obj;

    ---------------------------------------------------------------------------
    -- [E] MERGE, EDGES, COLOUR (stages 15 .. 24)
    ---------------------------------------------------------------------------
    p_mix : process(clk)
        variable v_obj  : unsigned(3 downto 0);
        variable v_sub  : std_logic_vector(3 downto 0);
        variable v_ramp : unsigned(2 downto 0);
        variable v_t    : unsigned(7 downto 0);
        variable v_fon  : std_logic;
        variable v_road, v_dash : std_logic;
        variable v_e    : std_logic;
        variable v_w    : unsigned(3 downto 0);
        variable v_tri  : unsigned(4 downto 0);
        variable v_sum  : signed(13 downto 0);
        variable v_c    : signed(10 downto 0);
        variable v_d8   : signed(7 downto 0);
        variable v_o    : unsigned(7 downto 0);
        type t_o3 is array (0 to 2) of unsigned(7 downto 0);
        variable v_out  : t_o3;
        variable v_y10  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------------
            -- 15: who owns the pixel
            ----------------------------------------------------------------
            v_obj := "0000"; v_sub := "0000"; v_ramp := "000";
            v_t := resize(s_cg & '0', 8);
            if l_fill(0) = '1' then v_t := v_t + resize(l_tsky, 8); end if;
            if l_fill(0) = '1' or s_cg /= 0 then v_fon := '1'; else v_fon := '0'; end if;
            if l_flr = '1' then
                -- floor half (and the tunnel's ceiling)
                v_road := (l_proad or (c_eroad and not s_pan and g14_rd and not l_gfar)) and not l_ab;
                v_dash := (l_pdash and g14_up) or (not s_pan and g14_dc and g14_zh and not l_gfar);
                if b14_z /= 0 and (l_bfold = '1' or l_bunit = '1') then
                    v_obj := "1001"; v_sub := "00" & std_logic_vector(b14_z); v_ramp := "111";
                    case b14_z is
                        when "01"   => v_t := to_unsigned(60, 8);
                        when "10"   => v_t := to_unsigned(6, 8);
                        when others => if c_t(5) = '1' then v_t := to_unsigned(127, 8); else v_t := to_unsigned(92, 8); end if;
                    end case;
                    v_fon := l_fill(7);
                elsif l_ab = '1' and s14_in = '1' then
                    v_obj := "0001"; v_ramp := "001"; v_t := resize(l_tsun, 8); v_fon := l_fill(1);
                elsif v_road = '1' then
                    v_obj := "1000"; v_sub := "0000"; v_sub(0) := v_dash and c_eroad; v_ramp := "101";
                    v_t := resize(l_tflr(6 downto 2), 8); v_fon := l_fill(5);
                else
                    v_obj := "0111"; v_ramp := "101";
                    v_sub := "0000"; v_sub(1) := g14_up; v_sub(0) := g14_zp and not l_gfar;
                    v_t := resize(l_tflr, 8); v_fon := l_fill(5);
                end if;
            else
                -- sky half (and its mirror in the lake): nearest first
                if b14_z /= 0 and l_obj = '1' then
                    v_obj := "0110"; v_sub := "00" & std_logic_vector(b14_z); v_ramp := "110";
                    if b14_z = "10" then v_t := to_unsigned(52, 8); else v_t := to_unsigned(28, 8); end if;
                    v_fon := l_fill(6);
                elsif n14_in = '1' and l_obj = '1' then
                    v_obj := "0101"; v_sub := "0000"; v_sub(1) := n14_bp; v_sub(0) := n14_cp; v_ramp := "100";
                    v_t := resize(l_tnr, 8);
                    if n14_ris = '1' then v_t := v_t + 32; end if;
                    v_fon := l_fill(4);
                elsif c14_in = '1' and l_obj = '1' then
                    v_obj := "0100"; v_sub := c14_sub; v_ramp := "011";
                    if c14_lit = '1' then v_t := to_unsigned(127, 8); else v_t := resize(l_tcty, 8); end if;
                    v_fon := l_fill(3);
                elsif f14_in = '1' and l_obj = '1' then
                    v_obj := "0011"; v_sub := "0000"; v_sub(1) := f14_bp; v_sub(0) := f14_cp; v_ramp := "010";
                    v_t := resize(l_tfar, 8); v_fon := l_fill(2);
                elsif a14_z /= 0 and l_obj = '1' then
                    v_obj := "0010"; v_sub := "00" & std_logic_vector(a14_z); v_ramp := "110";
                    v_t := resize(l_ta, 8);
                    if a14_z = "10" then v_t := v_t + 23; end if;
                    v_fon := l_fill(6);
                elsif s14_in = '1' then
                    v_obj := "0001"; v_ramp := "001"; v_t := resize(l_tsun, 8); v_fon := l_fill(1);
                end if;
            end if;
            if v_t(7) = '1' then v_t := to_unsigned(127, 8); end if;
            if s_sharp = '1' then v_t(3 downto 0) := "1000"; end if;
            s15_id <= std_logic_vector(v_obj) & v_sub;
            s15_ramp <= v_ramp; s15_t <= v_t(6 downto 0); s15_fon <= v_fon;
            if r14_on = '1' then s15_ovl <= "10";
            elsif t14_on = '1' and v_obj = 0 and l_flr = '0' then s15_ovl <= "01";
            else s15_ovl <= "00"; end if;
            s15_tw <= t14_tw;

            -- the ID history along the line, and the line store (this line's ID above last line's)
            i_d1 <= s15_id; i_d2 <= i_d1; i_d3 <= i_d2;
            s_g15 <= s_g15(2 downto 0) & s_gav_sr(13);
            lb_we <= s_gav_sr(13);
            lb_wa <= b_pxd2;
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
            s16_ramp <= s15_ramp; s16_t <= s15_t; s16_fon <= s15_fon; s16_ovl <= s15_ovl; s16_tw <= s15_tw;

            ----------------------------------------------------------------
            -- 17: colour table entry and lerp position.  A sheen travels along every wire: a 64-px pulse
            -- in every 256 along the diagonal lifts the wire toward the palette's hot colour.
            ----------------------------------------------------------------
            v_w := f_max4(f_max4(s16_m1, s16_m2), s16_m3);
            if b_pp(6) = '1' then v_tri := not b_pp(5 downto 1); else v_tri := b_pp(5 downto 1); end if;
            s17_blk <= '0'; s17_w <= '0';
            if s16_ovl = "10" then
                s17_ent <= to_unsigned(28, 5); s17_t6 <= (others => '0');
            elsif s16_e = '1' then
                s17_ent <= '1' & v_w; s17_w <= '1';
                if b_pp(8 downto 7) = "00" then s17_t6 <= v_tri & '0'; else s17_t6 <= (others => '0'); end if;
            elsif s16_ovl = "01" then
                s17_ent <= to_unsigned(27, 5); s17_t6 <= s16_tw & "0000";
            elsif s16_fon = '1' then
                s17_ent <= '0' & s16_ramp & s16_t(6); s17_t6 <= s16_t(5 downto 0);
            else
                s17_ent <= (others => '0'); s17_t6 <= (others => '0');
                s17_blk <= '1';
            end if;
            if l_rip = '1' then s17_blk <= '1'; s17_w <= '0'; end if;

            -- 18 / 19: the colour table (address register, EBR, fabric register)
            pal_a <= s_pal & s17_ent;
            s18_t6 <= s17_t6; s18_blk <= s17_blk;
            s19_t6 <= s18_t6; s19_blk <= s18_blk;
            s20_t6 <= s19_t6; s20_blk <= s19_blk;        -- pal_q at 19, pal_qf at 20
            s18_w <= s17_w; s19_w <= s18_w; s20_w <= s19_w; s21_w <= s20_w; s22_w <= s21_w; s23_w <= s22_w;

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

            -- 23: clamp, black, the lake's dimming
            for k in 0 to 2 loop
                v_c := s22_s(k);
                if v_c(10) = '1' then v_out(k) := (others => '0');
                elsif v_c(9 downto 8) /= "00" then v_out(k) := (others => '1');
                else v_out(k) := unsigned(v_c(7 downto 0)); end if;
            end loop;
            v_y10 := v_out(0) & "00";
            if l_mir = '1' and v_y10 > 64 then
                v_y10 := v_y10 - shift_right(v_y10 - 64, 2);
            end if;
            if s22_blk = '1' then
                s23_y <= to_unsigned(64, 10); s23_u <= to_unsigned(512, 10); s23_v <= to_unsigned(512, 10);
            else
                if v_y10 < 64 then s23_y <= to_unsigned(64, 10); else s23_y <= v_y10; end if;
                s23_u <= v_out(1) & "00"; s23_v <= v_out(2) & "00";
            end if;
            s23_blk <= s22_blk;

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

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent running 8-sums (resynced to
    -- neutral in blanking), luma [1 2 1]/4 at the chroma's group delay (+ 2 px for the glow), luma 13/16 toward black, blanking gate.
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
            -- luma taps y(n - 5), y(n - 6), y(n - 7): the chroma's group delay plus the glow's 2 px
            s_yd(1) <= v_iy;
            for i in 2 to 7 loop s_yd(i) <= s_yd(i - 1); end loop;
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

    data_out.y       <= std_logic_vector(o4_y);
    data_out.u       <= std_logic_vector(o4_u);
    data_out.v       <= std_logic_vector(o4_v);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture neonscape;
