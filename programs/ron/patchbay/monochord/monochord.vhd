-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Monochord v0.1.0 -- Patchbay: the harmonics of a vibrating string.
--
-- The patch:
--   V staircase (row counter)  -> harmonic number h_n = F (1 + n s)
--   H-locked sine VCO, hard-synced to H, h_n half-waves across the screen (+ a travel phase)
--      -> full-wave rectifier |sin| -> VCA (Pluck, a slight per-row unevenness; camera luma swells it)
--      = the standing-wave envelope e
--   in-row V triangle |v| and e -> window comparator: the lens chain (fringe, AA edge, bright dwell ramp, body)
--   per-row LFO (rate rising with h) -> VCA on the signed sine -> narrow comparator: the white-hot string
--      (Strobe: the string at this instant and at its two extremes, the envelope's outline; no fill)
--   the VCO's phase near a node -> node comparator: beads at the nodes, fret ticks through them
--   harmonic number -> colourizer hue;  two strings per pixel (own row + the nearer neighbour, or Fan: the
--   fundamental and the top row's harmonic on one full-height string) -> additive mixer (clipped: flat tops)
--
--   K1  Fundamental  half-waves of row 1 across the screen, 1 .. 4 (detents at the integers, glided)
--   K2  Rows         2 .. 8 strings
--   K3  Partials     the harmonic step per row: 0 (all alike) .. 1 (1 2 3 4 ..) .. 2 (odd partials 1 3 5 7 ..)
--   K4  Speed        bipolar: the lobes travel along the strings either way, the LFOs run faster
--   K5  Colour       colourizer rotation and spread across the harmonics (Amber: which phosphor)
--   K6  Camera       box-filtered luma (square law) swells the strings' amplitude where the picture is bright
--   S7  String       Blur (filled lobes) / Strobe (the string now and at its two extremes, no fill)
--   S8  Stack        Ladder (rows apart) / Fan (two harmonics superposed on one string)
--   S9  Nodes        Off / On (beads at every node and fret ticks through them)
--   S10 Colours      Spectrum / Amber (one phosphor colour)
--   S11 Source       Video / Osc
--   P12 PLUCK        amplitude: slivers (0 %) .. lobes filling their rows (50 %) .. 1.6x the row height (100 %)
--
-- Engine: vblank-zone sequencer (one serial multiplier, a shared glide, an 8-row loop that fills a row table in
-- block RAM); a line engine in each hblank (row tracker, 10 table reads into the two strings' line registers, the
-- bead / fret widths); two 17-stage string evaluators per pixel; the additive mixer; the shell's
-- output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) and blanking gate.
-- Virtual 1080-line rows; every string spans the screen width at every resolution; on coarse rasters (1080i,
-- SD) the strings run in units of 384 / ys virtual lines so the thin features keep 720p's line count.
-- Generated avid; one latency (24).  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.monochord_pkg.all;
use work.all;

architecture monochord of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

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
    -- the Osc lattice's line step (ys * 121 / 256)
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
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- coarse rasters run the strings in units of 384 / ys virtual lines (Q16, 0 = unity)
    function f_kscale(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(22114, 16);   -- 480i  (ys 1138)
            when 4          => return to_unsigned(43691, 16);   -- 480p  (576)
            when 8          => return to_unsigned(26214, 16);   -- 576i  (960)
            when 12         => return to_unsigned(52429, 16);   -- 576p  (480)
            when 1 | 2 | 10 => return to_unsigned(49152, 16);   -- 1080i (512)
            when others     => return to_unsigned(0, 16);
        end case;
    end function;
    -- bead scale: node-distance units per (sixteenth of a) half string unit = 8192 / (virtual px per pixel x scale)
    function f_kx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(12140, 14);
            when 4          => return to_unsigned(6144, 14);
            when 8          => return to_unsigned(10240, 14);
            when 12         => return to_unsigned(5120, 14);
            when 1 | 2 | 10 => return to_unsigned(10923, 14);
            when 5 | 6 | 14 => return to_unsigned(5461, 14);
            when others     => return to_unsigned(8192, 14);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- row step of the Rows knob: 0 .. 6
    function f_ridx(p : unsigned(10 downto 0)) return unsigned is
        variable v : unsigned(13 downto 0);
    begin
        v := shift_left(resize(p, 14), 3) - resize(p, 14);       -- 7 p
        return v(12 downto 10);
    end function;
    type t_u3  is array (0 to 1) of unsigned(2 downto 0);
    type t_u4  is array (0 to 1) of unsigned(3 downto 0);
    type t_u6  is array (0 to 1) of unsigned(5 downto 0);
    type t_u7  is array (0 to 1) of unsigned(6 downto 0);
    type t_u8  is array (0 to 1) of unsigned(7 downto 0);
    type t_u9  is array (0 to 1) of unsigned(8 downto 0);
    type t_u10 is array (0 to 1) of unsigned(9 downto 0);
    type t_u11 is array (0 to 1) of unsigned(10 downto 0);
    type t_u12 is array (0 to 1) of unsigned(11 downto 0);
    type t_u13 is array (0 to 1) of unsigned(12 downto 0);
    type t_u14 is array (0 to 1) of unsigned(13 downto 0);
    type t_u16 is array (0 to 1) of unsigned(15 downto 0);
    type t_u20 is array (0 to 1) of unsigned(19 downto 0);
    type t_s10 is array (0 to 1) of signed(9 downto 0);
    type t_s12 is array (0 to 1) of signed(11 downto 0);
    type t_s13 is array (0 to 1) of signed(12 downto 0);
    type t_v24 is array (0 to 1) of std_logic_vector(23 downto 0);

    -- luma cell store: one 12-bit vertical-slew state per 8-pixel cell (read a cell ahead of the write)
    type t_lb is array (0 to 511) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(8 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(15 downto 0) := x"0000";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(15 downto 0) := x"0000";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav, s_f1, s_f2 : std_logic := '0';
    signal s_lc    : unsigned(2 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(8 downto 0) := (others => '0');
    signal s_acc, s_sum8 : unsigned(10 downto 0) := (others => '0');
    signal s_cur, s_old, s_new : unsigned(11 downto 0) := (others => '0');
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_h1    : unsigned(15 downto 0) := (others => '0');
    -- camera swell: square law through a table, times the per-field gain (Q1 string units)
    signal r_sqa   : unsigned(7 downto 0) := (others => '0');
    signal r_sqq   : unsigned(15 downto 0) := (others => '0');
    signal s_sq8   : unsigned(7 downto 0) := (others => '0');
    signal s_swa, s_swb : unsigned(13 downto 0) := (others => '0');
    signal s_sw    : unsigned(11 downto 0) := (others => '0');

    -- row table (block RAM): 8 words per row; written by the sequencer in the zone, read by the line engine
    -- (and the sequencer's LFO read-modify-write) through one time-shared port; never read where it writes
    signal rt : t_c_mc_rt0 := C_MC_RT0;
    attribute no_rw_check of rt : signal is true;
    signal t_we : std_logic := '0';
    signal t_wa, t_ra : unsigned(7 downto 0) := (others => '0');
    signal t_wd : unsigned(15 downto 0) := (others => '0');
    signal t_rq, t_rf : unsigned(15 downto 0) := (others => '0');

    -- ROMs (one read port each; registered address and data)
    signal r_sa : t_u10 := (others => (others => '0'));
    signal r_sq : t_u10 := (others => (others => '0'));
    signal r_pa : t_u9 := (others => (others => '0'));
    signal r_pq : t_v24 := (others => (others => '0'));

    -- output filter stores: the two 8-pixel delay lines of the boxes (16-deep rings, read 6 behind the write
    -- plus a fabric register = 8 pixels)
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da, db : t_dl := (others => (others => '0'));
    attribute no_rw_check of da : signal is true;
    attribute no_rw_check of db : signal is true;
    signal s_dp : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_dr : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_qa, s_qb, s_fa, s_fb : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_ysr  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ksr : unsigned(15 downto 0) := (others => '0');            -- coarse-raster scale (Q16, 0 = unity)
    signal s_kxr : unsigned(13 downto 0) := to_unsigned(8192, 14);
    signal s_crs  : std_logic := '0';                                   -- 480i / 576i: the small bead table
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw Rows / Speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(0, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(600 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k2 : unsigned(9 downto 0) := to_unsigned(658, 10);
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_strobe, s_fan, s_nodes, s_pal, s_src : std_logic := '0';

    -- per-field values
    signal z_iu, z_id : unsigned(2 downto 0) := (others => '0');
    signal s_ri   : unsigned(2 downto 0) := to_unsigned(4, 3);         -- rows - 2
    signal c_r1   : unsigned(2 downto 0) := to_unsigned(5, 3);         -- rows - 1
    signal z_hix  : unsigned(2 downto 0) := to_unsigned(4, 3);         -- row-height index (7 = Fan: the screen)
    signal z_t1   : unsigned(11 downto 0) := (others => '0');
    signal z_t3   : unsigned(10 downto 0) := (others => '0');
    signal z_fi, z_si : unsigned(1 downto 0) := (others => '0');
    signal z_fsn, z_ssn : std_logic := '0';
    signal z_flo, z_slo : unsigned(1 downto 0) := (others => '0');
    signal z_ffr, z_sfr : unsigned(9 downto 0) := (others => '0');
    signal z_ahi  : std_logic := '0';
    signal z_ap   : unsigned(9 downto 0) := (others => '0');
    signal c_f    : unsigned(10 downto 0) := to_unsigned(256, 11);     -- fundamental, Q8
    signal c_s    : unsigned(9 downto 0) := to_unsigned(256, 10);      -- partial step, Q8
    signal c_fs   : unsigned(11 downto 0) := to_unsigned(256, 12);     -- F s, Q8
    signal c_arel : unsigned(9 downto 0) := to_unsigned(600, 10);      -- amplitude / row height, Q10
    signal c_hq8  : unsigned(18 downto 0) := to_unsigned(46080, 19);   -- row height (Fan: the screen), Q8 units
    signal z_hq1  : unsigned(10 downto 0) := to_unsigned(360, 11);     -- row height (Fan: half the screen), Q1
    signal c_ab   : unsigned(10 downto 0) := (others => '0');          -- amplitude, Q1
    signal c_km   : unsigned(10 downto 0) := to_unsigned(360, 11);     -- amplitude clamp, Q1
    signal z_g    : unsigned(11 downto 0) := (others => '0');
    signal c_g    : unsigned(9 downto 0) := (others => '0');           -- camera gain
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_d    : unsigned(27 downto 0) := (others => '0');          -- travel phase
    signal c_lk   : unsigned(10 downto 0) := to_unsigned(240, 11);     -- LFO gain
    signal c_rot  : unsigned(11 downto 0) := to_unsigned(1536, 12);    -- colourizer rotation, Q8 sixteenths
    signal c_sp   : unsigned(9 downto 0) := to_unsigned(640, 10);      -- spread per harmonic, Q8 sixteenths
    signal c_amb  : unsigned(3 downto 0) := (others => '0');           -- Amber: the phosphor

    -- row loop (zone): z_j step, z_r row
    signal z_rrun : std_logic := '0';
    signal z_j    : unsigned(6 downto 0) := (others => '0');
    signal z_r    : unsigned(2 downto 0) := (others => '0');
    signal z_h    : unsigned(14 downto 0) := (others => '0');          -- this row's harmonic, Q8
    signal z_hd   : unsigned(14 downto 0) := (others => '0');          -- + LFO detune
    signal z_w    : unsigned(10 downto 0) := (others => '0');          -- this row's amplitude weight, Q10
    signal z_inc  : unsigned(15 downto 0) := (others => '0');
    signal z_t0h  : unsigned(3 downto 0) := (others => '0');
    signal z_hue  : unsigned(3 downto 0) := (others => '0');
    signal z_hq   : unsigned(11 downto 0) := (others => '0');
    signal z_rate : unsigned(13 downto 0) := (others => '0');
    signal z_thl, z_thn : unsigned(15 downto 0) := (others => '0');
    signal z_cs1 : std_logic := '0';
    signal z_cm1 : unsigned(6 downto 0) := (others => '0');
    signal z_ra  : unsigned(7 downto 0) := (others => '0');
    signal z_cosa : unsigned(7 downto 0) := (others => '0');
    signal z_cos  : std_logic := '0';

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(31 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
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

    -- line engine (hblank) and the two strings' line registers
    signal e_lc  : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_adv, e_zk : std_logic := '0';
    signal e_ry  : unsigned(18 downto 0) := (others => '0');
    signal e_rn  : unsigned(3 downto 0) := (others => '0');
    signal e_vo  : signed(11 downto 0) := (others => '0');            -- Q1
    signal e_vneg : std_logic := '0';
    signal e_ei  : unsigned(0 downto 0) := (others => '0');
    signal e_wi  : unsigned(2 downto 0) := (others => '0');
    signal e_rd  : std_logic := '0';
    signal e_ei1, e_ei2, e_ei3, e_ei4 : unsigned(0 downto 0) := (others => '0');
    signal e_wi1, e_wi2, e_wi3, e_wi4 : unsigned(2 downto 0) := (others => '0');
    signal e_rd1, e_rd2, e_rd3, e_rd4 : std_logic := '0';
    signal e_ta  : unsigned(7 downto 0) := (others => '0');
    signal e_tb  : unsigned(11 downto 0) := (others => '0');
    signal e_tc  : unsigned(11 downto 0) := (others => '0');
    signal n_v   : t_s12 := (others => (others => '0'));
    signal n_av  : t_u11 := (others => (others => '0'));
    signal n_r   : t_u3 := (others => (others => '0'));
    signal n_val : std_logic_vector(1 downto 0) := (others => '0');
    signal l_inc : t_u16 := (others => (others => '0'));
    signal b_th  : t_u20 := (others => (others => '0'));
    signal l_a   : t_u12 := (others => (others => '0'));
    signal l_av  : t_u11 := (others => (others => '0'));
    signal l_m1  : t_u7 := (others => (others => '0'));
    signal l_s1  : std_logic_vector(1 downto 0) := (others => '0');
    signal l_hue : t_u4 := (others => (others => '0'));
    signal l_v   : t_s12 := (others => (others => '0'));
    signal l_kb  : t_u14 := (others => (others => '0'));
    signal l_tc, l_tf : t_u12 := (others => (others => '0'));
    signal l_val : std_logic_vector(1 downto 0) := (others => '0');

    -- string evaluators (stage numbers: 1 = the phase DDA)
    signal p2_p : t_u12 := (others => (others => '0'));
    signal p2_sg, p3_sg, p4_sg, p5_sg, p6_sg : std_logic_vector(1 downto 0) := (others => '0');
    signal p3_wt, p4_wt, p5_wt, p6_wt, p7_wt, p8_wt, p9_wt, p10_wt : std_logic_vector(1 downto 0) := (others => '0');   -- bead / fret white
    signal p3_k  : t_u13 := (others => (others => '0'));
    signal p4_s  : t_u10 := (others => (others => '0'));
    signal p4_k  : t_u11 := (others => (others => '0'));
    signal p5_ma, p5_mb : t_u16 := (others => (others => '0'));
    signal p6_m0 : t_u11 := (others => (others => '0'));
    signal p7_ra : t_u13 := (others => (others => '0'));
    signal p7_rb : t_u12 := (others => (others => '0'));
    signal p7_d0 : t_s12 := (others => (others => '0'));
    signal p7_s1, p8_s1 : std_logic_vector(1 downto 0) := (others => '0');
    signal p8_m1 : t_u11 := (others => (others => '0'));
    signal p8_lf, p9_lf, p10_lf : t_u4 := (others => (others => '0'));
    signal p8_ad, p9_ad, p10_ad : t_u4 := (others => (others => '0'));
    signal p8_fn, p9_fn, p10_fn : std_logic_vector(1 downto 0) := (others => '0');
    signal p9_t1 : t_s13 := (others => (others => '0'));
    signal p10_a1 : t_u4 := (others => (others => '0'));
    signal p11_l, p12_l, p13_l, p14_l : t_u4 := (others => (others => '0'));
    signal p17_q : t_v24 := (others => (others => '0'));
    signal p18_y : unsigned(11 downto 0) := (others => '0');
    signal p18_u, p18_v : signed(11 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KU : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KV : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel
    -- the syncs' C_LAT delay: a block-RAM ring (written at s_swp, read 22 behind, then a fabric register)
    type t_sd is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sd : t_sd := (others => "1110");
    attribute no_rw_check of sd : signal is true;
    signal s_swp : unsigned(7 downto 0) := (others => '0');
    signal s_sra : unsigned(7 downto 0) := to_unsigned(234, 8);
    signal s_sq, s_sf : std_logic_vector(3 downto 0) := "1110";

begin

    ---------------------------------------------------------------------------
    -- Memories: luma cell store, row table, ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rt : process(clk)
    begin
        if rising_edge(clk) then
            t_rq <= rt(to_integer(t_ra));
            if t_we = '1' then rt(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_rt;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_sq(0) <= C_MC_SIN(to_integer(r_sa(0)));
            r_sq(1) <= C_MC_SIN(to_integer(r_sa(1)));
            s_sq <= sd(to_integer(s_sra));
            sd(to_integer(s_swp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & '0';
            r_pq(0) <= C_MC_PAL(to_integer(r_pa(0)));
            r_pq(1) <= C_MC_PAL(to_integer(r_pa(1)));
            r_sqq   <= C_MC_SQ(to_integer(r_sqa));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer and the row loop (one small
    -- operation per zone clock; quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_o  : signed(12 downto 0);
        variable v_t  : unsigned(11 downto 0);
        variable v_fr : signed(11 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_wi : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_ksr <= f_kscale(s_tid);
            s_kxr <= f_kx(s_tid);
            s_oys <= f_oys(s_tid);
            if s_ys > 384 then s_ysr <= to_unsigned(384, 12); else s_ysr <= s_ys; end if;

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
            s_k2 <= unsigned(registers_in(1));
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
                s_strobe <= registers_in(6)(0);
                s_fan    <= registers_in(6)(1);
                s_nodes  <= registers_in(6)(2);
                s_pal    <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_g5     <= unsigned(registers_in(4)) & "000";                -- Colour: stepped (16 hues), no glide
                if s_ys >= 960 then s_crs <= '1'; else s_crs <= '0'; end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;

            -- shared glide of K1, K3, K6, P12 (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 2      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when 2      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 3 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial multiplier: per-field slot k loads at zc = 48 + 24 k (decoded at 46 + 24 k, operands
            -- pre-muxed at 47 + 24 k), result stable from load + 17, stored at load + 18.  The row loop loads
            -- at j = 2, 20, 38, 56, 74, 92 (decoded at j - 2).
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 46  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 94  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 118 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 142 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 166 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 190 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 214 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 238 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 262 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 286 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 310 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 334 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            if z_rrun = '1' then
                case to_integer(z_j) is
                    when 0  => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 18 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 36 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 54 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when 72 => z_sel <= to_unsigned(20, 5); z_pre <= '1';
                    when 90 => z_sel <= to_unsigned(21, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  =>                                                                            -- row height x scale
                        case to_integer(z_hix) is
                            when 0      => z_fa <= to_unsigned(138240, 32);
                            when 7      => z_fa <= to_unsigned(276480, 32);
                            when 1      => z_fa <= to_unsigned(92160, 32);
                            when 2      => z_fa <= to_unsigned(69120, 32);
                            when 3      => z_fa <= to_unsigned(55296, 32);
                            when 4      => z_fa <= to_unsigned(46080, 32);
                            when 5      => z_fa <= to_unsigned(39497, 32);
                            when others => z_fa <= to_unsigned(34560, 32);
                        end case;
                        z_fb <= "00" & s_ksr(15 downto 2);
                    when 2  => z_fa <= to_unsigned(334, 32);         z_fb <= resize(z_ffr, 16);          -- F fraction
                    when 3  => z_fa <= to_unsigned(303, 32);         z_fb <= resize(z_sfr, 16);          -- s fraction
                    when 4  =>                                                                            -- Pluck
                        z_fa <= resize(z_ap, 32);
                        if z_ahi = '1' then z_fb <= to_unsigned(615, 16); else z_fb <= to_unsigned(901, 16); end if;
                    when 5  => z_fa <= resize(c_f, 32);              z_fb <= resize(c_s, 16);            -- F s
                    when 6  => z_fa <= resize(z_hq1, 32);            z_fb <= resize(c_arel, 16);         -- amplitude
                    when 7  => z_fa <= resize(z_hq1, 32);            z_fb <= resize(c_arel, 16) + 307;   -- camera
                    when 8  => z_fa <= resize(z_g, 32);              z_fb <= resize(s_g6(12 downto 3), 16);
                    when 9  => z_fa <= resize(z_g, 32);              z_fb <= to_unsigned(294, 16);
                    when 10 => z_fa <= resize(s_vm, 32);             z_fb <= resize(s_vm, 16);           -- speed^2
                    when 11 => z_fa <= resize(c_vsq, 32);            z_fb <= to_unsigned(679, 16);       -- travel step
                    when 12 => z_fa <= resize(c_vsq, 32);            z_fb <= to_unsigned(729, 16);       -- LFO gain
                    when 16 =>                                                                            -- inc
                        case to_integer(s_xsr) is
                            when 4      => z_fa <= to_unsigned(186414, 32);
                            when 3      => z_fa <= to_unsigned(104858, 32);
                            when others => z_fa <= to_unsigned(69905, 32);
                        end case;
                        z_fb <= resize(z_h, 16);
                    when 17 => z_fa <= resize(c_d, 32);              z_fb <= resize(z_h, 16);            -- theta0
                    when 18 => z_fa <= resize(c_ab, 32);             z_fb <= resize(z_w, 16);            -- A_n
                    when 19 => z_fa <= resize(c_sp, 32);             z_fb <= resize(z_h, 16);            -- hue
                    when 20 => z_fa <= resize(c_lk, 32);             z_fb <= resize(z_hd, 16);           -- LFO rate
                    when others => z_fa <= resize(z_inc, 32);        z_fb <= resize(s_kxr, 16);          -- bead scale
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 24 =>
                        z_t1 <= shift_left(resize(s_g1(12 downto 3), 12), 1) + resize(s_g1(12 downto 3), 12);   -- 3 k1
                        z_t3 <= shift_left(resize(s_g3(12 downto 3), 11), 1);                                    -- 2 k3
                        -- rows with +-8 codes of hysteresis (the ends always reach the end zones)
                        if s_k2 > 8 then z_iu <= f_ridx(resize(s_k2, 11) - 8); else z_iu <= (others => '0'); end if;
                        if s_k2 < 1015 then z_id <= f_ridx(resize(s_k2, 11) + 8); else z_id <= to_unsigned(6, 3); end if;
                        -- Pluck: 0.06 .. 0.5 over the lower half, 0.5 .. 0.8 over the upper (slot 4)
                        z_ahi <= s_gp(12);
                        if s_gp(12) = '1' then z_ap <= resize(s_gp(11 downto 3), 10); else z_ap <= s_gp(12 downto 3); end if;
                    when 25 =>
                        z_fi <= resize(shift_right(resize(z_t1, 13) + 512, 10), 2);
                        z_si <= resize(shift_right(resize(z_t3, 12) + 512, 10), 2);
                        if z_iu > s_ri then s_ri <= z_iu; elsif z_id < s_ri then s_ri <= z_id; end if;
                    when 26 =>
                        -- detents: flat within +-120 (F) / +-80 (s) of an integer, linear between
                        v_o := signed(resize(z_t1, 13)) - signed(resize(z_fi & "0000000000", 13));
                        if v_o > -120 and v_o < 120 then z_fsn <= '1'; else z_fsn <= '0'; end if;
                        z_flo <= z_t1(11 downto 10);
                        v_fr := signed(resize(z_t1(9 downto 0), 12)) - 120;
                        if v_fr < 0 then z_ffr <= (others => '0');
                        elsif v_fr > 784 then z_ffr <= to_unsigned(784, 10);
                        else z_ffr <= unsigned(v_fr(9 downto 0)); end if;
                        v_o := signed(resize(z_t3, 13)) - signed(resize(z_si & "0000000000", 13));
                        if v_o > -80 and v_o < 80 then z_ssn <= '1'; else z_ssn <= '0'; end if;
                        z_slo <= resize(z_t3(10 downto 10), 2);
                        v_fr := signed(resize(z_t3(9 downto 0), 12)) - 80;
                        if v_fr < 0 then z_sfr <= (others => '0');
                        elsif v_fr > 864 then z_sfr <= to_unsigned(864, 10);
                        else z_sfr <= unsigned(v_fr(9 downto 0)); end if;
                        c_r1 <= s_ri + 1;
                        if s_fan = '1' then z_hix <= "111"; else z_hix <= s_ri; end if;
                    when 27 =>
                        c_rot <= shift_left(resize(s_g5(12 downto 3), 12), 1) + resize(s_g5(12 downto 3), 12);
                        c_sp  <= resize(s_g5(12 downto 4), 10) + 384;
                    when 28 =>
                        v_t := c_rot + 2688;
                        c_amb <= v_t(11 downto 8);

                    when 66 =>
                        if s_ksr /= 0 then c_hq8 <= resize(f_acc(31 downto 14), 19);
                        else
                            case to_integer(z_hix) is
                                when 0      => c_hq8 <= to_unsigned(138240, 19);
                                when 1      => c_hq8 <= to_unsigned(92160, 19);
                                when 2      => c_hq8 <= to_unsigned(69120, 19);
                                when 3      => c_hq8 <= to_unsigned(55296, 19);
                                when 4      => c_hq8 <= to_unsigned(46080, 19);
                                when 5      => c_hq8 <= to_unsigned(39497, 19);
                                when 7      => c_hq8 <= to_unsigned(276480, 19);
                                when others => c_hq8 <= to_unsigned(34560, 19);
                            end case;
                        end if;
                    when 67 =>
                        -- amplitude scale: the row height in Q1 units (Fan: half the screen); the clamp
                        if s_fan = '1' then z_hq1 <= c_hq8(18 downto 8); c_km <= c_hq8(18 downto 8) + shift_right(c_hq8(18 downto 8), 5);
                        else z_hq1 <= c_hq8(17 downto 7); c_km <= c_hq8(17 downto 7); end if;
                    when 114 =>
                        if z_fsn = '1' then c_f <= shift_left(resize(z_fi, 11), 8) + 256;
                        else c_f <= shift_left(resize(z_flo, 11), 8) + 256 + resize(f_acc(19 downto 10), 11); end if;
                    when 138 =>
                        if z_ssn = '1' then c_s <= shift_left(resize(z_si, 10), 8);
                        else c_s <= shift_left(resize(z_slo, 10), 8) + resize(f_acc(19 downto 10), 10); end if;
                    when 162 =>
                        if z_ahi = '1' then c_arel <= resize(f_acc(18 downto 10), 10) + 512;
                        else c_arel <= resize(f_acc(19 downto 10), 10) + 61; end if;
                    when 186 => c_fs <= f_acc(19 downto 8);
                    when 210 => c_ab <= f_acc(20 downto 10);
                    when 234 => z_g <= f_acc(21 downto 10);
                    when 258 => z_g <= f_acc(21 downto 10);
                    when 259 =>
                        if s_src = '1' then z_g <= shift_left(z_g, 1) + shift_right(z_g, 1); end if;
                    when 282 => c_g <= f_acc(19 downto 10);
                    when 306 => c_vsq <= f_acc(17 downto 0);
                    when 330 =>
                        -- travel: right of centre the lobes move right (the phase at the left edge falls)
                        if s_vneg = '1' then c_d <= c_d + resize(f_acc(27 downto 16), 28); else c_d <= c_d - resize(f_acc(27 downto 16), 28); end if;
                    when 354 => c_lk <= resize(f_acc(27 downto 18), 11) + 240;
                    when others => null;
                end case;
            end if;

            -- row loop: 8 rows x 112 clocks from zc 360
            t_we <= '0';
            z_cos <= '0';
            if s_zone = '1' and s_zc = 360 then
                z_rrun <= '1'; z_j <= (others => '0'); z_r <= (others => '0'); z_h <= resize(c_f, 15);
            elsif z_rrun = '1' then
                if z_j = 111 then
                    z_j <= (others => '0');
                    z_r <= z_r + 1;
                    z_h <= z_h + resize(c_fs, 15);
                    if z_r = 7 then z_rrun <= '0'; end if;
                else
                    z_j <= z_j + 1;
                end if;
                case to_integer(z_j) is
                    when 20 =>
                        z_inc <= f_acc(31 downto 16);
                        t_we <= '1'; t_wa <= "00" & z_r & "000"; t_wd <= f_acc(31 downto 16);         -- w0 inc
                    when 30 =>                      -- amplitude weight: ladder unevenness, or Fan's harmonic weights
                        if s_fan = '1' and z_r = 0 then v_wi := "01" & z_r;
                        elsif s_fan = '1' and z_r = c_r1 then v_wi := "10" & z_r;
                        else v_wi := "00" & z_r; end if;
                        z_w <= C_MC_W(to_integer(v_wi));
                    when 38 =>
                        t_we <= '1'; t_wa <= "00" & z_r & "001"; t_wd <= f_acc(23 downto 8);          -- w1 theta0 lo
                        z_t0h <= f_acc(27 downto 24);
                    when 56 =>
                        t_we <= '1'; t_wa <= "00" & z_r & "010"; t_wd <= z_t0h & f_acc(21 downto 10); -- w2 theta0 hi, A
                    when 60 =>
                        z_hd <= unsigned(signed(z_h) + resize(C_MC_DN(to_integer(z_r)), 15));
                    when 74 =>
                        z_hq <= c_rot + f_acc(19 downto 8);
                    when 75 =>
                        if s_pal = '1' then z_hue <= c_amb; else z_hue <= z_hq(11 downto 8); end if;
                    when 80 =>
                        z_ra <= "00" & z_r & "111";                                                    -- LFO phase read
                    when 88 =>
                        z_thl <= t_rf;
                    when 92 =>
                        if f_acc(23 downto 8) > 10923 then z_rate <= to_unsigned(10923, 14);
                        else z_rate <= f_acc(21 downto 8); end if;
                    when 93 =>
                        z_thn <= z_thl + resize(z_rate, 16);
                    when 94 =>
                        t_we <= '1'; t_wa <= "00" & z_r & "111"; t_wd <= z_thn;                        -- w7 LFO phase
                        v_p := z_thn(15 downto 6) + 256;                                                -- cos = sin + 90
                        z_cs1 <= v_p(9);
                        if v_p(8) = '1' then z_cosa <= not v_p(7 downto 0); else z_cosa <= v_p(7 downto 0); end if;
                        z_cos <= '1';
                    when 98 =>                      -- the sine ROM data of the lookup is in evaluator 0's stage-4 register
                        z_cm1 <= p4_s(0)(9 downto 3);
                    when 99 =>
                        t_we <= '1'; t_wa <= "00" & z_r & "011"; t_wd <= "0000" & z_cs1 & z_cm1 & z_hue; -- w3 c1, hue
                    when 110 =>
                        t_we <= '1'; t_wa <= "00" & z_r & "100"; t_wd <= "00" & f_acc(29 downto 16);  -- w4 bead scale
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_swp <= s_swp + 1; s_sra <= s_sra + 1; s_sf <= s_sq;
        end if;
    end process p_ctrl;

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
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- ((Y >> 2) - 16), averaged over 8-pixel cells, each cell slewed down the
    -- screen (one pole, 1/32 per line; the state lives in the cell store), then
    -- slewed across by a one-pole filter (1/16 per pixel); then squared
    -- (table) and scaled by the camera gain: the swell added to every string's
    -- amplitude.  All three filters decay, so none needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(16 downto 0);
        variable v_p : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
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

            -- 1: sample
            if s_src = '1' then
                s_lin <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lin <= v_l - 16; else s_lin <= (others => '0'); end if;
            end if;
            s_lav   <= data_in.avid;
            s_lc    <= s_wcol(2 downto 0);
            s_lcell <= resize(s_wcol(10 downto 3), 9);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= resize(s_wcol(10 downto 3), 9) + 1;
            else
                s_wcol  <= (others => '0');
                s_lb_ra <= (others => '0');
            end if;
            -- 2: sum eight; at the eighth, take the cell's state (read a cell ahead) for the update
            s_f1 <= '0';
            if s_lav = '0' then
                s_cur <= unsigned(s_lb_q(11 downto 0));
            else
                if s_lc = 0 then s_acc <= resize(s_lin, 11); else s_acc <= s_acc + s_lin; end if;
                if s_lc = 7 then
                    s_sum8  <= s_acc + s_lin;
                    s_old   <= s_cur;
                    s_cur   <= unsigned(s_lb_q(11 downto 0));
                    s_fcell <= s_lcell;
                    s_f1    <= '1';
                end if;
            end if;
            -- 3: state += (cell average * 16 - state) / 32
            v_d := signed(resize(s_sum8 & '0', 13)) - signed(resize(s_old, 13));
            v_d := signed(resize(s_old, 13)) + shift_right(v_d, 5);
            s_new   <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);

            -- pixel side: the cell stream (the line above's state), two slews across
            s_lv <= unsigned(s_lb_q(11 downto 0));
            v_e  := signed(resize(s_lv & "0000", 17)) - signed(resize(s_h1, 17));
            v_e  := signed(resize(s_h1, 17)) + shift_right(v_e, 4);
            s_h1 <= unsigned(v_e(15 downto 0));
            -- square law (table), then x the camera gain (operands split), Q1 string units
            r_sqa  <= s_h1(15 downto 8);
            s_sq8  <= r_sqq(15 downto 8);
            s_swa  <= s_sq8(7 downto 4) * c_g;
            s_swb  <= s_sq8(3 downto 0) * c_g;
            v_p := shift_left(resize(s_swa, 20), 4) + resize(s_swb, 20);
            s_sw <= v_p(17 downto 6);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: in each hblank (from the generated avid's fall; and once in
    -- the zone for line 0) it advances the row tracker to the next line, picks
    -- the two strings (own row + the nearer neighbour; Fan: the fundamental and
    -- the last row) with their in-row offsets v, reads their row words (10 reads)
    -- straight into the line registers 19 .. 28 clocks after the fall (the last
    -- pixel has left every stage that reads them; the next line starts 64+ clocks
    -- after the fall), then forms the bead / fret widths in node-distance units
    -- (a 6-step serial multiply per string).  The phase DDAs are loaded here too
    -- and step during the generated avid.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_a12 : signed(11 downto 0);
        variable v_q8  : signed(19 downto 0);
        variable v_t   : unsigned(15 downto 0);
        variable v_u   : unsigned(12 downto 0);
        variable v_e   : integer range 0 to 1;
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_ry <= (others => '0'); e_rn <= (others => '0');
                e_lc <= to_unsigned(1, 6); e_adv <= '0';
            elsif s_gav_sr(0) = '1' and g_avid = '0' then
                e_lc <= to_unsigned(1, 6); e_adv <= '1';
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            e_rd <= '0';
            case to_integer(e_lc) is
                when 1 =>
                    if e_adv = '1' then e_ry <= e_ry + resize(s_ysr, 19); end if;
                when 2 =>
                    if e_ry >= c_hq8 then e_ry <= e_ry - c_hq8; e_rn <= e_rn + 1; end if;
                when 3 =>
                    -- in-row offset (Fan: the row is the whole screen), Q1 (floor of the Q8 value / 128)
                    v_q8 := signed(resize(e_ry, 20)) - signed(resize(c_hq8(18 downto 1), 20));
                    e_vo <= v_q8(18 downto 7); e_vneg <= v_q8(19);
                when 4 =>
                    if s_fan = '1' then
                        n_v(0) <= e_vo; n_r(0) <= (others => '0'); n_val(0) <= '1';
                        n_v(1) <= e_vo; n_r(1) <= c_r1;            n_val(1) <= '1';
                    else
                        n_v(0) <= e_vo; n_r(0) <= e_rn(2 downto 0);
                        if e_rn <= resize(c_r1, 4) then n_val(0) <= '1'; else n_val(0) <= '0'; end if;
                        if e_vneg = '1' then
                            n_v(1) <= e_vo + signed(resize(c_hq8(18 downto 7), 12)); n_r(1) <= e_rn(2 downto 0) - 1;
                            if e_rn /= 0 and e_rn <= resize(c_r1, 4) + 1 then n_val(1) <= '1'; else n_val(1) <= '0'; end if;
                        else
                            n_v(1) <= e_vo - signed(resize(c_hq8(18 downto 7), 12)); n_r(1) <= e_rn(2 downto 0) + 1;
                            if e_rn < resize(c_r1, 4) then n_val(1) <= '1'; else n_val(1) <= '0'; end if;
                        end if;
                    end if;
                when 5 =>
                    for e in 0 to 1 loop
                        v_a12 := abs(n_v(e));
                        n_av(e) <= unsigned(v_a12(10 downto 0));
                    end loop;
                when 14 =>
                    e_ei <= "0"; e_wi <= (others => '0'); e_rd <= '1';
                when 15 to 23 =>
                    if e_wi = 4 then e_wi <= (others => '0'); e_ei <= "1"; else e_wi <= e_wi + 1; end if;
                    e_rd <= '1';
                when others => null;
            end case;
            if e_lc = 22 then
                l_v <= n_v; l_av <= n_av; l_val <= n_val;
            end if;

            -- bead and fret widths in node-distance units: the bead's half-width at the centre is 2.5 kb (480i /
            -- 576i: kb), 5/8 of that in a second tier of |v| (a rounded blob); the fret 10 kb / 16.
            -- String 0 only (the own row; Fan: the fundamental) at lc 24 .. 26 (its kb lands at 23).
            if e_lc = 24 then
                v_e := 0;
                if s_crs = '1' then v_t := resize(l_kb(v_e), 16);
                else v_t := shift_left(resize(l_kb(v_e), 16), 1) + shift_right(resize(l_kb(v_e), 16), 1); end if;
                if v_t(15 downto 12) /= "0000" then e_tb <= (others => '1'); else e_tb <= v_t(11 downto 0); end if;
                v_t := shift_right(shift_left(resize(l_kb(v_e), 16), 3) + shift_left(resize(l_kb(v_e), 16), 1), 4);  -- 10 kb / 16
                if v_t(15 downto 12) /= "0000" then l_tf(v_e) <= (others => '1'); else l_tf(v_e) <= v_t(11 downto 0); end if;
            end if;
            if e_lc = 25 then
                v_e := 0;
                if (s_crs = '1' and l_av(v_e) < 12) or (s_crs = '0' and l_av(v_e) < 28) then e_tc <= e_tb;
                elsif (s_crs = '1' and l_av(v_e) < 16) or (s_crs = '0' and l_av(v_e) < 40) then
                    e_tc <= shift_right(e_tb, 1) + shift_right(e_tb, 3);
                else e_tc <= (others => '0'); end if;
            end if;
            if e_lc = 26 then l_tc(0) <= e_tc; end if;

            -- the read port: the sequencer's LFO reads during the row loop, the line engine otherwise
            if e_ei = "0" then e_ta <= "00" & n_r(0) & e_wi; else e_ta <= "00" & n_r(1) & e_wi; end if;
            if z_rrun = '1' then t_ra <= z_ra; else t_ra <= e_ta; end if;
            e_rd1 <= e_rd; e_ei1 <= e_ei; e_wi1 <= e_wi;
            e_rd2 <= e_rd1; e_ei2 <= e_ei1; e_wi2 <= e_wi1;
            e_rd3 <= e_rd2; e_ei3 <= e_ei2; e_wi3 <= e_wi2;
            e_rd4 <= e_rd3; e_ei4 <= e_ei3; e_wi4 <= e_wi3;
            t_rf <= t_rq;
            -- e_ta is registered one clock after (e_ei, e_wi), t_ra one more, t_rq one more, t_rf one more:
            -- the word for the tag (e_ei, e_wi) of clock c is in t_rf at clock c + 4 (the tags delayed by 4)
            for e in 0 to 1 loop
                if e_rd4 = '1' and to_integer(e_ei4) = e then
                    case to_integer(e_wi4) is
                        when 0      => l_inc(e) <= t_rf;
                        when 1      => b_th(e)(15 downto 0) <= t_rf;
                        when 2      => b_th(e)(19 downto 16) <= t_rf(15 downto 12); l_a(e) <= t_rf(11 downto 0);
                        when 3      => l_s1(e) <= t_rf(11); l_m1(e) <= t_rf(10 downto 4); l_hue(e) <= t_rf(3 downto 0);
                        when others => l_kb(e) <= t_rf(13 downto 0);
                    end case;
                elsif g_avid = '1' then
                    b_th(e) <= b_th(e) + resize(l_inc(e), 20);
                end if;
            end loop;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- String evaluators (stage 1 = the phase DDA, in p_line)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p   : unsigned(11 downto 0);
        variable v_m   : unsigned(20 downto 0);
        variable v_r   : unsigned(18 downto 0);
        variable v_q   : signed(12 downto 0);
        variable v_a   : signed(12 downto 0);
        variable v_d   : signed(11 downto 0);
        variable v_l   : unsigned(3 downto 0);
        variable v_dm  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            for e in 0 to 1 loop
                -- 2: fold the phase: the 12-bit distance from the nearest node (4095 at the antinode) addresses
                -- the quarter table (its top 10 bits) and drives the node comparators
                if b_th(e)(18) = '1' then v_p := not b_th(e)(17 downto 6); else v_p := b_th(e)(17 downto 6); end if;
                if e = 0 and z_cos = '1' then r_sa(e) <= z_cosa & "00"; else r_sa(e) <= v_p(11 downto 2); end if;
                p2_p(e) <= v_p; p2_sg(e) <= b_th(e)(19);
                -- 3: table (EBR); node comparators (white: bead core, or a fret on the own row / Fan's fundamental;
                -- halo around the bead); amplitude + camera swell
                p3_sg(e) <= p2_sg(e);
                if e = 0 and s_nodes = '1' and (p2_p(e) < l_tc(e) or p2_p(e) < l_tf(e)) then p3_wt(e) <= '1'; else p3_wt(e) <= '0'; end if;
                p3_k(e) <= resize(l_a(e), 13) + resize(s_sw, 13);
                -- 4: fabric copy of |sin|; amplitude clamp
                p4_s(e) <= r_sq(e);
                if p3_k(e) > resize(c_km, 13) then p4_k(e) <= c_km; else p4_k(e) <= p3_k(e)(10 downto 0); end if;
                p4_sg(e) <= p3_sg(e); p4_wt(e) <= p3_wt(e);
                -- 5: envelope products (operand split)
                p5_ma(e) <= p4_k(e) * p4_s(e)(9 downto 5);
                p5_mb(e) <= p4_k(e) * p4_s(e)(4 downto 0);
                p5_sg(e) <= p4_sg(e); p5_wt(e) <= p4_wt(e);
                -- 6: e = K0 |s| (Q1)
                v_m := shift_left(resize(p5_ma(e), 21), 5) + resize(p5_mb(e), 21);
                p6_m0(e) <= v_m(20 downto 10);
                p6_sg(e) <= p5_sg(e); p6_wt(e) <= p5_wt(e);
                -- 7: LFO x envelope (operand split); distance inside the lens
                p7_ra(e) <= l_m1(e) * p6_m0(e)(10 downto 5);
                p7_rb(e) <= l_m1(e) * p6_m0(e)(4 downto 0);
                p7_d0(e) <= signed(resize(p6_m0(e), 12)) - signed(resize(l_av(e), 12));
                p7_s1(e) <= p6_sg(e) xor l_s1(e);
                p7_wt(e) <= p6_wt(e);
                -- 8: the string's offset; fill level; |d0| (saturated) for the Strobe outline
                v_r := shift_left(resize(p7_ra(e), 19), 5) + resize(p7_rb(e), 19);
                p8_m1(e) <= v_r(17 downto 7);
                if    p7_d0(e) < -8 then v_l := to_unsigned(0, 4);
                elsif p7_d0(e) < -5 then v_l := to_unsigned(1, 4);
                elsif p7_d0(e) < -2 then v_l := to_unsigned(2, 4);
                elsif p7_d0(e) < 1  then v_l := to_unsigned(3, 4);
                elsif p7_d0(e) < 7  then v_l := to_unsigned(4, 4);
                elsif p7_d0(e) < 11 then v_l := to_unsigned(5, 4);
                elsif p7_d0(e) < 16 then v_l := to_unsigned(6, 4);
                elsif p7_d0(e) < 23 then v_l := to_unsigned(7, 4);
                elsif p7_d0(e) < 31 then v_l := to_unsigned(8, 4);
                elsif p7_d0(e) < 40 then v_l := to_unsigned(9, 4);
                else                     v_l := to_unsigned(10, 4); end if;
                if s_fan = '1' then
                    if p7_d0(e) >= 31 then v_l := to_unsigned(14, 4);
                    elsif p7_d0(e) >= 16 then v_l := to_unsigned(15, 4); end if;
                end if;
                p8_lf(e) <= v_l;
                if p7_d0(e) > -1 then p8_fn(e) <= '1'; else p8_fn(e) <= '0'; end if;
                v_d := abs(p7_d0(e));
                if v_d > 15 then p8_ad(e) <= to_unsigned(15, 4); else p8_ad(e) <= unsigned(v_d(3 downto 0)); end if;
                p8_s1(e) <= p7_s1(e); p8_wt(e) <= p7_wt(e);
                -- 9: offset of the pixel from the string (Q1)
                if p8_s1(e) = '1' then v_q := -signed(resize(p8_m1(e), 13)); else v_q := signed(resize(p8_m1(e), 13)); end if;
                p9_t1(e) <= v_q - resize(l_v(e), 13);
                p9_lf(e) <= p8_lf(e); p9_ad(e) <= p8_ad(e); p9_fn(e) <= p8_fn(e);
                p9_wt(e) <= p8_wt(e);
                -- 10: distances, saturated at 15
                v_a := abs(p9_t1(e));
                if v_a > 15 then p10_a1(e) <= to_unsigned(15, 4); else p10_a1(e) <= unsigned(v_a(3 downto 0)); end if;
                p10_lf(e) <= p9_lf(e); p10_ad(e) <= p9_ad(e); p10_fn(e) <= p9_fn(e);
                p10_wt(e) <= p9_wt(e);
                -- 11: the mode's level (Blur: fill, then the hot line; Strobe: the string and the envelope's outline,
                -- i.e. the string at this instant and at its two extremes); beads and
                -- frets over it; nothing for a missing string
                if s_strobe = '1' then
                    v_dm := p10_a1(e);
                    if p10_ad(e) < v_dm then v_dm := p10_ad(e); end if;
                    if v_dm < 4 then v_l := to_unsigned(13, 4);
                    elsif v_dm < 9 then v_l := to_unsigned(11, 4);
                    elsif v_dm < 13 then v_l := to_unsigned(3, 4);
                    else v_l := to_unsigned(0, 4); end if;
                else
                    v_l := p10_lf(e);
                    if p10_a1(e) < 12 and p10_fn(e) = '1' then v_l := to_unsigned(11, 4); end if;
                    if p10_a1(e) < 5 and p10_fn(e) = '1' then v_l := to_unsigned(13, 4); end if;
                end if;
                if p10_wt(e) = '1' then v_l := to_unsigned(12, 4); end if;
                if l_val(e) = '0' then p11_l(e) <= (others => '0'); else p11_l(e) <= v_l; end if;
                -- 12 .. 14: delay (the one latency, 24); 15: colourizer address; 16: colourizer (EBR); 17: fabric copy
                p12_l(e) <= p11_l(e); p13_l(e) <= p12_l(e); p14_l(e) <= p13_l(e);
                r_pa(e) <= s_pal & l_hue(e) & p14_l(e);
                p17_q(e) <= r_pq(e);
            end loop;
            -- 18: the additive mixer (dY x 4, dU x 4, dV x 4 per string)
            p18_y <= shift_left(resize(unsigned(p17_q(0)(23 downto 16)), 12), 2) + shift_left(resize(unsigned(p17_q(1)(23 downto 16)), 12), 2) + 64;
            p18_u <= shift_left(resize(signed(p17_q(0)(15 downto 8)), 12), 2) + shift_left(resize(signed(p17_q(1)(15 downto 8)), 12), 2) + 512;
            p18_v <= shift_left(resize(signed(p17_q(0)(7 downto 0)), 12), 2) + shift_left(resize(signed(p17_q(1)(7 downto 0)), 12), 2) + 512;
            -- 19: clip (the colourizer's flat tops); black off the generated avid
            if s_gav_sr(16) = '0' then
                s_y19 <= C_KY; s_u19 <= C_KU; s_v19 <= C_KV;
            else
                if p18_y > 940 then s_y19 <= to_unsigned(940, 10); else s_y19 <= p18_y(9 downto 0); end if;
                if p18_u < 24 then s_u19 <= to_unsigned(24, 10); elsif p18_u > 1000 then s_u19 <= to_unsigned(1000, 10);
                else s_u19 <= unsigned(p18_u(9 downto 0)); end if;
                if p18_v < 24 then s_v19 <= to_unsigned(24, 10); elsif p18_v > 1000 then s_v19 <= to_unsigned(1000, 10);
                else s_v19 <= unsigned(p18_v(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output filter stores
    ---------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_dp <= s_dp + 1; s_dr <= s_dr + 1;
            s_qa <= da(to_integer(s_dr));
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y19) & std_logic_vector(s_u19) & std_logic_vector(s_v19);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y21) & std_logic_vector(s_u21) & std_logic_vector(s_v21);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the rack's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16, and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the black (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y19 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u19 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v19 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= C_KY; s_u21 <= C_KU; s_v21 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y21 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u21 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v21 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v23 <= v_sc(13 downto 4);
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
    data_out.hsync_n <= s_sf(3);
    data_out.vsync_n <= s_sf(2);
    data_out.field_n <= s_sf(1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture monochord;
