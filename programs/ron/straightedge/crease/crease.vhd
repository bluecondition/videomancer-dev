-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Crease v0.1.1 -- Straightedge.  A striped sheet of paper folded flat along straight creases (Bridget Riley's
-- zigzags, Frank Stella's notched stripes, herringbone parquet).  Each fold reflects the half of the sheet
-- beyond its crease, so the stripes kink into sharp chevrons at every crease and stay continuous across it.
-- Luma only (U = V = 512).  Every edge on the screen is a straight line.
--
--   fold k:    p' = p - 2 n_k max(0, n_k . p - c_k)          (k = 1 .. Folds, fold 1 first, p from the pivot)
--   phase   =  d(Angle) . p_folded / (4 Scale) + Drift       (cycles; one cycle = four bands)
--   With the fold decisions (s1 s2 s3) = the region code, the folded point is M p + t: the phase of each of
--   the 8 regions is an affine function of the screen (8 phase DDAs) and each crease test is a half plane with
--   a constant unit normal (7 sign DDAs).  No per-pixel multiplier.
--   Pleat:  one fold repeated: the region alternates every N stripe cycles of (phase 0 - phase 1), so every
--           chevron is identical and the stripes meet exactly at every crease.
--
--   K1  X        the pivot the creases are arranged round, +-704 virtual px          (glided)
--   K2  Y        +-480 virtual px                                                    (glided)
--   K3  Scale    band 12 .. 300 virtual px, exponential                              (glided)
--   K4  Folds    1 / 2 / 3 creases; Pleat: 3 / 2 / 1 stripe cycles per strip         (stepped, hysteresis)
--   K5  Drift    the stripes stream along the sheet and round every bend; centre +-24 = still, else
--                2^-12 .. 2^-4 cycle per field                                       (speed glided 1/8)
--   K6  Camera   the block's luma (32 x 32 virtual px, held, one field old) sets the band weight in 4 flat levels:
--                the light bands widen, the dark ones thin (never under half a band; a line never over 3/4)
--   S7  Glide    Off = this field's knob / On = 1/16 of the gap per field
--   S8  Fold     Creases / Pleat
--   S9  Tone     Grey (black, dark, light, white) / Ink
--   S10 Weight   Bold (equal bands) / Fine (lines of 1/6 band, at least 2 pixels)
--   S11 Source   Video / Osc
--   P12 ANGLE    the direction of the stripes on the sheet, a half turn             (glided)
--
-- Sections (the integer model .straightedge_work/rb_crease/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide, Folds hysteresis, Drift speed, switches
--   [C] FIELD SEQUENCER                  one serial multiplier, slots of 20 clocks: scale, pivot, angle, then
--                                        per region the direction (interpolated quarter-wave ROM), the phase
--                                        steps, offset and seed, then the seven crease tests
--   [D] SOURCE FIELD AND CAMERA GRID     Osc lattice, gain table, box, per-block vertical one-pole, level grid
--   [E] DDAS                             15 affine DDAs, region code, phase select
--   [F] TONE                             band / line engine, nearest-edge coverage (4-step divider), luma
--   [G] OUTPUT CONDITIONING              luma [1 2 1]/4, luma 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  The sign DDAs hold DDA pixel i at T + 6 + i; a register "at stage S" holds
-- pixel i at T + 6 + i + S; the luma is stage 13.  Output column j is the [1 2 1] filter centred on DDA
-- pixel j + 1.  One latency (24) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.crease_pkg.all;
use work.all;

architecture crease of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
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
    -- 2^24 / (the y step): virtual lines -> field lines
    function f_ry(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(14743, 17);
            when 4          => return to_unsigned(29127, 17);
            when 8          => return to_unsigned(17476, 17);
            when 12         => return to_unsigned(34953, 17);
            when 5 | 6 | 14 => return to_unsigned(43691, 17);
            when 1 | 2 | 10 => return to_unsigned(32768, 17);
            when others     => return to_unsigned(65536, 17);
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
    -- the tone of band b (Ink: white; Fine Grey: lines of dark, light, white, light; Grey Bold: the band itself)
    function f_tl(b : unsigned(1 downto 0); ink, fine : std_logic) return unsigned is
    begin
        if ink = '1' then
            return "11";
        elsif fine = '1' then
            if b = "10" then return "11";
            elsif b = "00" then return "01";
            else return "10"; end if;
        else
            return b;
        end if;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ryr, z_ry : unsigned(16 downto 0) := to_unsigned(65536, 17);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
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

    -- [B] knobs: +-1 hold (10 bits), glide (Q3)
    signal h_x : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal h_y : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal h_p : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal h_c : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal h_s : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal g_x : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal g_y : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal g_p : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal g_c : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);
    signal g_s : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(13 downto 0) := (others => '0');
    signal s_gr, s_gh : unsigned(9 downto 0) := (others => '0');
    signal z_gp, z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun, z_drun : std_logic := '0';
    signal z_dp  : unsigned(4 downto 0) := (others => '0');
    signal s_glide : std_logic := '1';
    signal s_pleat, s_ink, s_fine, s_src, s_ib : std_logic := '0';
    -- Folds: three zones with hysteresis
    signal z_v3  : unsigned(11 downto 0) := (others => '1');
    signal s_nf  : unsigned(1 downto 0) := "10";                       -- Folds - 1
    signal s_nv  : std_logic := '0';
    signal s_m2, s_m3 : std_logic := '0';
    -- Drift
    signal s_k5  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vtg, s_vgs : signed(16 downto 0) := (others => '0');
    signal s_vneg, s_vz : std_logic := '0';
    signal s_av, s_u, s_ls : unsigned(9 downto 0) := (others => '0');
    signal s_scnt : unsigned(2 downto 0) := (others => '0');
    signal c_spd  : unsigned(18 downto 0) := (others => '0');
    signal c_drift : unsigned(21 downto 0) := (others => '0');

    -- [C] ROMs
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q, sn_f : unsigned(15 downto 0) := (others => '0');
    signal kr_a : unsigned(5 downto 0) := (others => '0');
    signal kr_q, kr_f : unsigned(15 downto 0) := (others => '0');

    -- [C] sequencer and its serial multiplier
    signal q_run, q_go, q_reg, q_sgn : std_logic := '0';
    signal q_k, q_j, q_d : unsigned(3 downto 0) := (others => '0');
    signal q_ph : unsigned(4 downto 0) := (others => '0');
    signal f_a, f_acc : signed(37 downto 0) := (others => '0');
    signal z_fa : signed(21 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_shr  : unsigned(2 downto 0) := (others => '0');
    signal z_sc   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_s11  : unsigned(15 downto 0) := to_unsigned(45056, 16);
    signal z_rq   : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal z_m    : unsigned(11 downto 0) := to_unsigned(1928, 12);
    signal z_w44  : unsigned(13 downto 0) := to_unsigned(7684, 14);
    signal z_yv   : unsigned(12 downto 0) := to_unsigned(4320, 13);
    signal z_kn   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sgl  : std_logic := '0';
    signal z_th   : unsigned(15 downto 0) := (others => '0');
    signal z_p    : unsigned(14 downto 0) := (others => '0');
    signal z_ov, z_ov1, z_sneg, z_cneg : std_logic := '0';
    signal z_fs, z_fc : unsigned(3 downto 0) := (others => '0');
    signal z_sm, z_cm : unsigned(14 downto 0) := (others => '0');
    signal z_sdf, z_cdf : unsigned(5 downto 0) := (others => '0');
    signal z_ax : unsigned(18 downto 0) := (others => '0');
    signal z_sx, z_sy, z_seed : signed(24 downto 0) := (others => '0');
    signal z_sy0 : signed(19 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_invp : unsigned(15 downto 0) := (others => '0');      -- cycles per h, Q22
    signal c_iu   : unsigned(17 downto 0) := (others => '0');      -- cycles per pixel, Q22
    signal c_iv   : unsigned(18 downto 0) := (others => '0');      -- cycles per line, Q22
    signal c_gcx, c_gsy : signed(17 downto 0) := (others => '0');  -- cos / sin (Angle) in cycles per virtual px, Q22
    signal c_X    : unsigned(13 downto 0) := (others => '0');      -- the pivot: DDA pixel index, Q3
    signal c_Y    : unsigned(14 downto 0) := (others => '0');      -- field line index, Q4
    signal c_th   : unsigned(15 downto 0) := (others => '0');      -- the stripe direction, 65536 per turn

    -- [E] DDA table: (dda, 0) = the pixel step, (dda, 1) = the line wrap, (dda, 2) = the field seed
    type t_cr is array (0 to 255) of signed(31 downto 0);
    signal cr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_ra, cr_wa : unsigned(5 downto 0) := (others => '0');
    signal cr_q, cr_wd : signed(31 downto 0) := (others => '0');
    signal cr_we : std_logic := '0';
    -- the anti-aliasing footprint of each region (bands per pixel, Q12)
    type t_wr is array (0 to 255) of unsigned(15 downto 0);
    signal wr_ram : t_wr := (others => (others => '0'));
    attribute no_rw_check of wr_ram : signal is true;
    signal wr_q, wr_wd : unsigned(15 downto 0) := (others => '0');
    signal wr_wa : unsigned(2 downto 0) := (others => '0');
    signal wr_we : std_logic := '0';

    -- [E] DDAs (0 .. 7 the region phases: cycles, Q22; 8 .. 14 the crease tests: pixel widths, Q12) and their loader
    type t_s22x8 is array (0 to 7) of signed(21 downto 0);
    signal pa_inc, pa_acc : t_s22x8 := (others => (others => '0'));
    type t_s25x7 is array (0 to 6) of signed(24 downto 0);
    signal ca_inc, ca_acc : t_s25x7 := (others => (others => '0'));
    signal ca_ld, ca_en : std_logic_vector(14 downto 0) := (others => '0');
    signal ca_rst : std_logic := '0';
    signal l_kick, l_run, l_seed : std_logic := '0';
    signal l_t : unsigned(4 downto 0) := (others => '0');
    signal l_v1, l_v2, l_o1, l_o2 : std_logic := '0';
    signal l_c1, l_c2 : unsigned(3 downto 0) := (others => '0');
    signal b_run, b_end : std_logic := '0';
    signal b_cnt, b_mm1 : unsigned(11 downto 0) := (others => '0');
    signal b_d : std_logic_vector(1 to 2) := (others => '0');
    signal px_a : unsigned(12 downto 0) := (others => '0');

    -- [E][F] pixel pipeline
    signal x1_code : unsigned(2 downto 0) := (others => '0');
    signal x2_ph, x3_ph : unsigned(13 downto 0) := (others => '0');
    signal x3_w, x4_w, x5_w, x6_w, x7_w, x8_w, x9_w, x10_w : unsigned(11 downto 0) := (others => '0');
    signal x3_k, x4_b : unsigned(1 downto 0) := (others => '0');
    signal x4_ts, x4_eh, x5_ts, x5_nts, x6_d1, x6_d2 : unsigned(13 downto 0) := (others => '0');
    signal x4_np : signed(14 downto 0) := (others => '0');
    signal x5_da : signed(15 downto 0) := (others => '0');
    signal s_gb : std_logic := '1';
    signal x5_full, x6_full, x6_inl : std_logic := '0';
    signal x5_tb, x5_tp, x5_tn, x6_tb, x6_tp, x6_tn : unsigned(1 downto 0) := (others => '0');
    signal x7_d : unsigned(11 downto 0) := (others => '0');
    type t_u2 is array (7 to 11) of unsigned(1 downto 0);
    signal x_own : t_u2 := (others => "00");
    type t_s3 is array (7 to 11) of signed(2 downto 0);
    signal x_dl : t_s3 := (others => "000");
    signal x8_f8, x9_f8, x10_f8, x11_f8 : std_logic := '0';
    signal x8_r, x9_r, x10_r : unsigned(11 downto 0) := (others => '0');
    signal x9_q, x10_q, x11_q : unsigned(2 downto 0) := (others => '0');
    signal x12_v : unsigned(5 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_v1, s_v2, s_v2d, s_v3, s_v4, s_ld, s_fl, s_seen : std_logic := '0';
    type t_gt is array (0 to 255) of unsigned(15 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt_ram : signal is true;
    signal gt_q, s_tv, s_tvd, s_sfirst, gt_wd : unsigned(15 downto 0) := (others => '0');
    signal gt_wa, gt_i : unsigned(7 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_acc : unsigned(21 downto 0) := (others => '0');
    signal z_gk  : unsigned(13 downto 0) := (others => '0');
    signal bx_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_f : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_b32 : std_logic := '1';                                  -- 32-px box on the 1920-wide rasters, else 16
    signal s_ish : unsigned(1 downto 0) := "00";                      -- the one-pole's shift - 2
    signal s_df   : signed(16 downto 0) := (others => '0');
    signal s_bsum : unsigned(21 downto 0) := (others => '0');
    -- block tracking and the per-block vertical one-pole
    signal s_xa  : unsigned(12 downto 0) := (others => '0');
    signal s_ya  : unsigned(18 downto 0) := (others => '0');
    signal s_yf  : std_logic := '1';
    signal s_bank : std_logic := '0';
    signal e_val, f_val, f_old, g_val, g_old : unsigned(11 downto 0) := (others => '0');
    signal e_bx, f_bx, g_bx : unsigned(5 downto 0) := (others => '0');
    signal e_v, f_ev, g_ev, g_fl : std_logic := '0';
    signal s_dd : signed(12 downto 0) := (others => '0');
    type t_ir is array (0 to 255) of unsigned(11 downto 0);
    signal ir_ram : t_ir := (others => (others => '0'));
    attribute no_rw_check of ir_ram : signal is true;
    signal ir_q, ir_f, ir_wd : unsigned(11 downto 0) := (others => '0');
    signal ir_wa : unsigned(5 downto 0) := (others => '0');
    signal ir_we : std_logic := '0';
    -- camera grid: two banks of 64 x 64 blocks, 2 bits
    type t_gr is array (0 to 8191) of unsigned(1 downto 0);
    signal gr_ram : t_gr := (others => (others => '0'));
    attribute no_rw_check of gr_ram : signal is true;
    signal gr_ra, gr_wa : unsigned(12 downto 0) := (others => '0');
    signal gr_q, gr_wd : unsigned(1 downto 0) := (others => '0');
    signal gr_we : std_logic := '0';

    -- [G] output conditioning
    signal s_y23, s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya3 : unsigned(11 downto 0) := (others => '0');
    signal s_y26, s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_CR_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_CR_SIN(to_integer(sn_a));
            sn_f   <= sn_q;
            kr_q   <= C_CR_K(to_integer(kr_a));
            kr_f   <= kr_q;
        end if;
    end process p_rom;

    -- DDA table: written by the sequencer in the zone, read by the loader (kick, line ends)
    p_crram : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= cr_ram(to_integer(cr_ra));
            if cr_we = '1' then cr_ram(to_integer(cr_wa)) <= cr_wd; end if;
        end if;
    end process p_crram;

    -- footprint table: written in the zone, read by the pixel path
    p_wrram : process(clk)
    begin
        if rising_edge(clk) then
            wr_q <= wr_ram(to_integer(x1_code));
            if wr_we = '1' then wr_ram(to_integer(wr_wa)) <= wr_wd; end if;
        end if;
    end process p_wrram;

    -- gain table: written in the zone, read by the source path
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(s_sl));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
        end if;
    end process p_gtram;

    -- box delay: read 30 (or 14) samples behind the write (+ two registers = 32 or 16)
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_v2 = '1' then bx_ram(to_integer(bx_wp)) <= s_tv; end if;
        end if;
    end process p_bxram;

    -- the one-pole's state, one word per block column: read at the block under the source beam, written a
    -- few clocks after that block has ended (the read has moved on by then)
    p_irram : process(clk)
    begin
        if rising_edge(clk) then
            ir_q <= ir_ram(to_integer(s_xa(12 downto 7)));
            if ir_we = '1' then ir_ram(to_integer(ir_wa)) <= ir_wd; end if;
        end if;
    end process p_irram;

    -- camera grid: this field writes one bank, the pixel path reads the other
    p_grram : process(clk)
    begin
        if rising_edge(clk) then
            gr_q <= gr_ram(to_integer(gr_ra));
            if gr_we = '1' then gr_ram(to_integer(gr_wa)) <= gr_wd; end if;
        end if;
    end process p_grram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the field sequencer
    -- (one small operation per zone clock; the zone is quasi-static but timed at
    -- the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k   : unsigned(9 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_e   : unsigned(12 downto 0);
        variable v_s14 : signed(13 downto 0);
        variable v_v   : signed(10 downto 0);
        variable v_nv  : signed(10 downto 0);
        variable v_r   : unsigned(15 downto 0);
        variable v_row : unsigned(3 downto 0);
        variable v_a18 : unsigned(17 downto 0);
        variable v_w   : unsigned(19 downto 0);
        variable v_nk  : boolean;
        variable v_pl0 : boolean;
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_ryr <= f_ry(s_tid);
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

            -- [A] width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- [B] Folds (K4): three zones of K4 * 3 / 1024; a new zone is taken 24 counts inside it, or at
            -- either end of the knob, or on the first field.  (The product runs free.)
            v_k := unsigned(registers_in(3));
            z_v3 <= resize(v_k & '0', 12) + resize(v_k, 12);

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_pleat <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_k5    <= unsigned(registers_in(4));
                s_xs    <= s_xsr;
                z_ry    <= s_ryr;
                s_bank  <= not s_bank;
                if s_xsr = 2 then s_b32 <= '1'; else s_b32 <= '0'; end if;
                if s_ys < 384 then s_ish <= "10"; elsif s_ys < 960 then s_ish <= "01"; else s_ish <= "00"; end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); z_sc <= to_unsigned(16384, 16); z_s11 <= to_unsigned(22528, 16);
                    when 3      => s_oxs <= to_unsigned(153, 9); z_sc <= to_unsigned(21845, 16); z_s11 <= to_unsigned(30037, 16);
                    when others => s_oxs <= to_unsigned(102, 9); z_sc <= to_unsigned(32768, 16); z_s11 <= to_unsigned(45056, 16);
                end case;
                if s_nv = '0' or (z_v3(11 downto 10) /= s_nf and
                                  ((z_v3(9 downto 0) >= 24 and z_v3(9 downto 0) <= 1000) or
                                   (z_v3(11 downto 10) = 0 and z_v3(9 downto 0) < 24) or
                                   (z_v3(11 downto 10) = 2 and z_v3(9 downto 0) > 1000))) then
                    s_nf <= z_v3(11 downto 10);
                    s_nv <= '1';
                end if;
            end if;
            s_ib <= s_ink and not s_fine;
            s_gb <= not s_ink and not s_fine;
            if s_pleat = '0' and s_nf /= 0 then s_m2 <= '1'; else s_m2 <= '0'; end if;
            if s_pleat = '0' and s_nf = 2 then s_m3 <= '1'; else s_m3 <= '0'; end if;
            g_wm1 <= s_w - 1;
            b_mm1 <= s_w + 7;
            z_m   <= s_w + 8;
            z_w44 <= (s_w & "00") + 4;
            if s_src = '1' then z_gk <= resize(g_c & '0', 14) - resize(shift_right(g_c, 2), 14);    -- Osc: 7/4
            else z_gk <= resize(g_c - shift_right(g_c, 3), 14); end if;                         -- camera: 7/8
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            case to_integer(s_xs) is               -- line height in pixel widths, Q10
                when 4      => z_rq <= resize(s_ys & '0', 12);
                when 3      => z_rq <= to_unsigned(1024, 12);
                when others => z_rq <= resize(s_ys & "00", 12);
            end case;
            case to_integer(s_nf) is               -- Pleat: 4 / N, Q13 (N = 3, 2, 1 stripe cycles per strip)
                when 0      => z_kn <= to_unsigned(10923, 16);
                when 1      => z_kn <= to_unsigned(16384, 16);
                when others => z_kn <= to_unsigned(32768, 16);
            end case;
            -- the pivot's height in virtual lines, Q3: 540 + (K2 - 512) * 15 / 16
            v_s14 := signed(resize(g_y, 14)) - 4096;
            v_s14 := to_signed(4320, 14) + v_s14 - shift_right(v_s14, 4);
            z_yv <= unsigned(v_s14(12 downto 0));

            -- [B] shared hold + glide, five clocks per channel.  Hold: the knob moves the held value only by
            -- 2 counts or more (or to either end).  Glide On: 1/16 of the gap per field, never slower than
            -- one count, landing exactly.  Glide Off: the held value, this field.  Then the Drift speed.
            q_go <= '0';
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1'; z_drun <= '0';
            elsif z_grun = '1' then
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= g_x; s_gh <= h_x; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= g_y; s_gh <= h_y; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= g_p; s_gh <= h_p; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= g_c; s_gh <= h_c; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= g_s; s_gh <= h_s; s_gr <= unsigned(registers_in(7));
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
                            if s_gd >= 128 or s_gd <= -128 then s_gd <= shift_right(s_gd, 4);
                            elsif s_gd > 8 then s_gd <= to_signed(8, 14);
                            elsif s_gd < -8 then s_gd <= to_signed(-8, 14);
                            end if;
                        end if;
                        z_gp <= "100";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => g_x <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_x <= s_gh;
                            when 1      => g_y <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_y <= s_gh;
                            when 2      => g_p <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_p <= s_gh;
                            when 3      => g_c <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_c <= s_gh;
                            when others => g_s <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_s <= s_gh;
                        end case;
                        z_gp <= "000";
                        if z_gc = 4 then z_grun <= '0'; z_drun <= '1'; z_dp <= (others => '0'); end if;
                        z_gc <= z_gc + 1;
                end case;
            elsif z_drun = '1' then
                -- [B] Drift (Hypnos v1.1.0 SPEED): the knob about its centre glides 1/8 of the gap per field
                -- (Off: this field's knob); |v| < 24 = still, else 2^-12 .. 2^-4 cycle per field, exponential
                -- (mantissa from the 2^(i/256) table, shifted by the octave one bit per clock); the phase
                -- advances once per field
                z_dp <= z_dp + 1;
                case to_integer(z_dp) is
                    when 0 => s_vtg <= shift_left(resize(signed('0' & s_k5) - 512, 17), 6);
                    when 1 =>
                        if s_glide = '1' then s_vgs <= s_vgs + shift_right(s_vtg - s_vgs, 3);
                        else s_vgs <= s_vtg; end if;
                    when 2 =>
                        v_v := resize(shift_right(s_vgs, 6), 11);
                        s_vneg <= v_v(10);
                        v_nv := -v_v;
                        if v_v(10) = '1' then s_av <= unsigned(v_nv(9 downto 0)); else s_av <= unsigned(v_v(9 downto 0)); end if;
                    when 3 =>
                        if s_av < 24 then s_vz <= '1'; s_u <= (others => '0');
                        else s_vz <= '0'; s_u <= s_av - 24; end if;
                    when 4 => s_ls <= s_u + shift_right(s_u, 5) + shift_right(s_u, 6);
                    when 5 => r_ex_a <= s_ls(5 downto 0) & "00";
                    when 6 | 7 => null;
                    when 8 =>
                        if s_vz = '1' then c_spd <= (others => '0');
                        else c_spd <= shift_left(resize(r_exf(15 downto 7), 19), 2); end if;
                        s_scnt <= s_ls(8 downto 6);
                    when 9 | 10 | 11 | 12 | 13 | 14 | 15 =>
                        if s_scnt /= 0 then
                            c_spd <= shift_left(c_spd, 1);
                            s_scnt <= s_scnt - 1;
                        end if;
                    when others =>
                        c_drift <= f_addsub(c_drift, resize(c_spd, 22), s_vneg);
                        z_drun <= '0'; q_go <= '1';
                end case;
            end if;

            -- [C] the sequencer: slots of 20 clocks.  q_k = 0 holds 6 global slots; q_k = 1 .. 8 the region
            -- q_d = q_k - 1 (slots 0 .. 9, the first also 10 and 11); q_k = 9 .. 15 the crease test q_d - 8
            -- (slots 0, 1, 2, 7, 8, 9).  Phase 0 pre-muxes the operands, 1 loads the multiplier, 2 .. 17
            -- iterate, the product is read at 18.
            l_kick <= '0';
            if q_go = '1' then
                q_run <= '1'; q_k <= (others => '0'); q_j <= (others => '0'); q_ph <= (others => '0');
                q_reg <= '0'; q_sgn <= '0'; q_d <= (others => '0');
            elsif q_run = '1' then
                if q_ph = 19 then
                    q_ph <= (others => '0');
                    v_nk := false;
                    if q_k = 0 then
                        if q_j = 5 then v_nk := true; else q_j <= q_j + 1; end if;
                    elsif q_reg = '1' then
                        if q_j = 11 or (q_j = 9 and q_k /= 1) then v_nk := true; else q_j <= q_j + 1; end if;
                    else
                        if q_j = 2 then q_j <= to_unsigned(7, 4);
                        elsif q_j = 9 then v_nk := true;
                        else q_j <= q_j + 1; end if;
                    end if;
                    if v_nk then
                        q_j <= (others => '0');
                        q_k <= q_k + 1;
                        q_d <= q_k;
                        if q_k < 8 then q_reg <= '1'; q_sgn <= '0'; else q_reg <= '0'; q_sgn <= '1'; end if;
                        if q_k = 15 then q_run <= '0'; l_kick <= '1'; end if;
                    end if;
                else
                    q_ph <= q_ph + 1;
                end if;
            end if;
            v_pl0 := s_pleat = '1' and q_d = 8;                    -- the Pleat's strip counter replaces crease test 0

            -- operand pre-mux (phase 0).  kr_f holds the row's word the slot needs (set at phase 17 of the slot before).
            if q_run = '1' and q_ph = 0 then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                if q_k = 0 then
                    case to_integer(q_j) is
                        when 0      => z_fa <= signed(resize(g_p, 22));    z_fb <= to_unsigned(1190, 16);     -- octaves
                        when 2      => z_fa <= signed(resize(c_invp, 22)); z_fb <= resize(s_ys, 16);          -- cycles per line
                        when 3      => z_fa <= resize(signed(resize(g_x, 14)) - 4096, 22); z_fb <= z_s11;     -- pivot x
                        when 4      => z_fa <= signed(resize(z_ry, 22));   z_fb <= resize(z_yv, 16);          -- pivot y
                        when others => null;
                    end case;
                elsif q_reg = '1' then
                    case to_integer(q_j) is
                        when 1      => z_fa <= signed(resize(z_sdf, 22));  z_fb <= resize(z_fs, 16);          -- sine interpolation
                        when 2      => z_fa <= signed(resize(z_cdf, 22));  z_fb <= resize(z_fc, 16);          -- cosine
                        when 3      => z_fa <= signed(resize(c_iu, 22));   z_fb <= resize(z_cm, 16);          -- phase per pixel
                        when 4      => z_fa <= signed(resize(c_iv, 22));   z_fb <= resize(z_sm, 16);          -- phase per line
                        when 5 | 6  => z_fb <= '0' & kr_f(14 downto 0);                                       -- fold offset . direction
                            if q_j(0) = '1' then z_fa <= resize(c_gcx, 22); else z_fa <= resize(c_gsy, 22); end if;
                        when 7      => z_fa <= z_sx(21 downto 0);          z_fb <= resize(c_X, 16);           -- step * pivot
                        when 8      => z_fa <= z_sy(21 downto 0);          z_fb <= resize(c_Y, 16);
                        when 9      => z_fa <= z_sx(21 downto 0);          z_fb <= resize(z_m, 16);           -- steps per line
                        when 10     => z_fa <= signed(resize(c_invp & '0', 22)); z_fb <= resize(z_cm, 16);    -- cos per virtual px
                        when 11     => z_fa <= signed(resize(c_invp & '0', 22)); z_fb <= resize(z_sm, 16);
                        when others => null;
                    end case;
                else
                    case to_integer(q_j) is
                        when 1      =>
                            if v_pl0 then z_fa <= resize(z_sy0, 22); z_fb <= z_kn;
                            else z_fa <= resize(signed(kr_f), 22); z_fb <= resize(z_rq, 16); end if;          -- line step
                        when 2      =>
                            if not v_pl0 then z_fa <= resize(signed(kr_f), 22); z_fb <= z_sc; end if;         -- distance
                        when 7      => z_fa <= z_sx(21 downto 0);          z_fb <= resize(c_X, 16);
                        when 8      => z_fa <= z_sy(21 downto 0);          z_fb <= resize(c_Y, 16);
                        when 9      => z_fa <= z_sx(21 downto 0);          z_fb <= resize(z_m, 16);
                        when others => null;
                    end case;
                end if;
            end if;
            if q_ph = 1 then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 38); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(36 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- results and the small steps between the products
            cr_we <= '0'; wr_we <= '0';
            if q_run = '1' and q_k = 0 then
                case to_integer(q_j) is
                    when 0 =>
                        if q_ph = 18 then
                            z_oct <= unsigned(f_acc(23 downto 13));
                        elsif q_ph = 19 then
                            v_e := to_unsigned(4458, 13) - resize(z_oct, 13);
                            r_ex_a <= v_e(7 downto 0);
                            z_shr <= resize(to_unsigned(17, 5) - v_e(12 downto 8), 3);
                        end if;
                    when 1 =>
                        if q_ph = 3 then
                            case to_integer(z_shr) is
                                when 0      => c_invp <= r_exf;
                                when 1      => c_invp <= resize(r_exf(15 downto 1), 16);
                                when 2      => c_invp <= resize(r_exf(15 downto 2), 16);
                                when 3      => c_invp <= resize(r_exf(15 downto 3), 16);
                                when 4      => c_invp <= resize(r_exf(15 downto 4), 16);
                                when others => c_invp <= resize(r_exf(15 downto 5), 16);
                            end case;
                        elsif q_ph = 4 then
                            case to_integer(s_xs) is
                                when 4      => c_iu <= shift_left(resize(c_invp, 18), 2);
                                when 3      => c_iu <= shift_left(resize(c_invp, 18), 1) + resize(c_invp, 18);
                                when others => c_iu <= shift_left(resize(c_invp, 18), 1);
                            end case;
                        end if;
                    when 2 =>
                        if q_ph = 18 then c_iv <= unsigned(f_acc(25 downto 7)); end if;
                    when 3 =>
                        if q_ph = 18 then c_X <= unsigned(signed(z_w44) + f_acc(28 downto 15)); end if;
                    when 4 =>
                        if q_ph = 18 then c_Y <= unsigned(f_acc(29 downto 15)) - 8; end if;
                    when others =>
                        if q_ph = 2 then c_th <= C_CR_TH0 + resize(g_s & "00", 16); end if;
                end case;
            elsif q_run = '1' then
                -- the row of fold constants: word 0 and 1 in slot 0, then at phase 17 the word the next slot's
                -- product needs (it is in kr_f from phase 0 of that slot through its phase 19)
                if s_pleat = '1' and q_d = 1 then v_row := "1111"; else v_row := q_d; end if;
                kr_a(5 downto 2) <= v_row;
                if q_j = 0 and q_ph = 0 then kr_a(1 downto 0) <= "00";
                elsif q_j = 0 and q_ph = 1 then kr_a(1 downto 0) <= "01";
                elsif q_ph = 17 then
                    if (q_sgn = '1' and q_j = 1) or (q_reg = '1' and q_j = 4) then kr_a(1 downto 0) <= "10";
                    elsif q_reg = '1' and q_j = 5 then kr_a(1 downto 0) <= "11"; end if;
                end if;
                if q_reg = '1' then
                    case to_integer(q_j) is
                        when 0 =>                  -- the region's direction, then sine and cosine from the quarter-wave ROM
                            case to_integer(q_ph) is
                                when 1 =>          -- the seed starts from the Drift phase (Pleat: the odd sheet's constant)
                                    z_seed <= signed(resize(c_drift, 25));
                                    if s_pleat = '1' and q_d = 1 then
                                        case to_integer(s_nf) is
                                            when 0      => z_seed <= signed(resize(c_drift - shift_left(to_unsigned(1, 22), 19), 25));
                                            when 1      => z_seed <= signed(resize(c_drift - shift_left(to_unsigned(6, 22), 19), 25));
                                            when others => z_seed <= signed(resize(c_drift - shift_left(to_unsigned(3, 22), 19), 25));
                                        end case;
                                    end if;
                                when 3 => z_sgl <= kr_f(0);
                                when 4 => z_th <= f_addsub(kr_f, c_th, z_sgl);
                                when 6 =>          -- sine: quadrant fold
                                    z_sneg <= z_th(15);
                                    if z_th(14) = '1' then z_p <= to_unsigned(16384, 15) - resize(z_th(13 downto 0), 15);
                                    else z_p <= resize(z_th(13 downto 0), 15); end if;
                                when 7 =>
                                    sn_a <= z_p(13 downto 4); z_ov <= z_p(14); z_fs <= z_p(3 downto 0);
                                    if z_p(14) = '1' or z_p(13 downto 4) = 1023 then z_ov1 <= '1'; else z_ov1 <= '0'; end if;
                                when 8 =>
                                    sn_a <= sn_a + 1;
                                when 10 =>
                                    if z_ov = '1' then z_sm <= to_unsigned(16384, 15); else z_sm <= sn_f(14 downto 0); end if;
                                when 11 =>
                                    if z_ov = '1' then z_sdf <= (others => '0');
                                    elsif z_ov1 = '1' then z_sdf <= resize(to_unsigned(16384, 15) - z_sm, 6);
                                    else z_sdf <= resize(sn_f(14 downto 0) - z_sm, 6); end if;
                                when 12 =>         -- cosine = sine a quarter turn on
                                    v_r := z_th + 16384;
                                    z_cneg <= v_r(15);
                                    if v_r(14) = '1' then z_p <= to_unsigned(16384, 15) - resize(v_r(13 downto 0), 15);
                                    else z_p <= resize(v_r(13 downto 0), 15); end if;
                                when 13 =>
                                    sn_a <= z_p(13 downto 4); z_ov <= z_p(14); z_fc <= z_p(3 downto 0);
                                    if z_p(14) = '1' or z_p(13 downto 4) = 1023 then z_ov1 <= '1'; else z_ov1 <= '0'; end if;
                                when 14 =>
                                    sn_a <= sn_a + 1;
                                when 16 =>
                                    if z_ov = '1' then z_cm <= to_unsigned(16384, 15); else z_cm <= sn_f(14 downto 0); end if;
                                when 17 =>
                                    if z_ov = '1' then z_cdf <= (others => '0');
                                    elsif z_ov1 = '1' then z_cdf <= resize(to_unsigned(16384, 15) - z_cm, 6);
                                    else z_cdf <= resize(sn_f(14 downto 0) - z_cm, 6); end if;
                                when others => null;
                            end case;
                        when 1 =>
                            if q_ph = 18 then z_sm <= z_sm + resize(unsigned(f_acc(10 downto 4)) + ("000000" & f_acc(3)), 15); end if;
                        when 2 =>
                            if q_ph = 18 then z_cm <= z_cm + resize(unsigned(f_acc(10 downto 4)) + ("000000" & f_acc(3)), 15); end if;
                        when 3 =>
                            if q_ph = 18 then
                                z_ax <= unsigned(f_acc(32 downto 14));
                            elsif q_ph = 19 then
                                z_sx <= signed(f_addsub(to_unsigned(0, 25), resize(z_ax, 25), z_cneg));
                            end if;
                        when 4 =>
                            if q_ph = 18 then      -- and the footprint |d/dx| + |d/dy| in bands, Q12
                                z_sy <= signed(f_addsub(to_unsigned(0, 25), resize(unsigned(f_acc(32 downto 14)), 25), z_sneg));
                                v_w := resize(z_ax, 20) + resize(unsigned(f_acc(32 downto 14)), 20);
                                wr_we <= '1'; wr_wa <= q_d(2 downto 0); wr_wd <= resize(v_w(19 downto 8), 16);
                            elsif q_ph = 19 then   -- Pleat: the strip counter runs on sheet 0's line step
                                if q_d = 0 then z_sy0 <= z_sy(19 downto 0); end if;
                            end if;
                        when 5 | 6 =>
                            if q_ph = 18 then
                                z_seed(21 downto 0) <= signed(f_addsub(unsigned(z_seed(21 downto 0)), unsigned(f_acc(24 downto 3)), kr_f(15)));
                            end if;
                        when 10 =>
                            if q_ph = 18 then
                                v_a18 := unsigned(f_acc(31 downto 14));
                                c_gcx <= signed(f_addsub(to_unsigned(0, 18), v_a18, z_cneg));
                            end if;
                        when 11 =>
                            if q_ph = 18 then
                                v_a18 := unsigned(f_acc(31 downto 14));
                                c_gsy <= signed(f_addsub(to_unsigned(0, 18), v_a18, z_sneg));
                            end if;
                        when others => null;
                    end case;
                else
                    case to_integer(q_j) is
                        when 0 =>
                            if q_ph = 3 then z_sx <= resize(signed(kr_f), 25); end if;                        -- cos
                        when 1 =>
                            if q_ph = 18 then
                                if v_pl0 then z_sy <= f_acc(37 downto 13);
                                else z_sy <= f_acc(34 downto 10); end if;
                            end if;
                        when 2 =>
                            if q_ph = 18 then
                                if v_pl0 then
                                    z_sx <= (others => '0');
                                    z_seed <= to_signed(3145728, 25);
                                else
                                    z_seed <= signed(f_addsub(to_unsigned(0, 25), unsigned(f_acc(30 downto 6)), '1'));
                                end if;
                            end if;
                        when others => null;
                    end case;
                end if;
                -- slots 7 .. 9 (both kinds): seed -= step * pivot, wrap = line step - steps * pixel step, the table
                case to_integer(q_j) is
                    when 7 =>
                        if q_ph = 18 then z_seed <= z_seed - f_acc(27 downto 3); end if;
                    when 8 =>
                        if q_ph = 18 then z_seed <= z_seed - f_acc(28 downto 4); end if;
                    when 9 =>
                        case to_integer(q_ph) is
                            when 17 => cr_we <= '1'; cr_wa <= q_d & "00"; cr_wd <= resize(z_sx, 32);
                            when 18 => cr_we <= '1'; cr_wa <= q_d & "01"; cr_wd <= resize(z_sy - f_acc(24 downto 0), 32);
                            when 19 => cr_we <= '1'; cr_wa <= q_d & "10"; cr_wd <= resize(z_seed, 32);
                            when others => null;
                        end case;
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
    -- [E] DDAS.  Fifteen affine DDAs: each adds its own increment register on
    -- every clock of the line window (W + 8 steps), then once more with the
    -- line wrap parked in that register (wrap = line step - steps * pixel
    -- step), so the adder has no input mux.  The increments come from the DDA
    -- table one word per clock.  The kick (end of the field sequencer) clears
    -- the accumulators and adds the field seed the same way.  The region phases
    -- run one clock behind the crease tests (they are selected by the code).
    ---------------------------------------------------------------------------
    p_load : process(clk)
        variable v_x2 : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- line window
            b_end <= '0';
            if s_st(4) = '1' then
                b_run <= '1'; b_cnt <= (others => '0');
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = b_mm1 then b_run <= '0'; b_end <= '1'; end if;
            end if;
            b_d <= b_run & b_d(1);

            -- loader: 30 table reads (dda, wrap or seed), (dda, step)
            ca_rst <= l_kick;
            if l_kick = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '1';
            elsif b_end = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '0';
            elsif l_run = '1' then
                l_t <= l_t + 1;
                if l_t = 29 then l_run <= '0'; end if;
            end if;
            if l_t(0) = '1' then cr_ra <= l_t(4 downto 1) & "00";
            elsif l_seed = '1' then cr_ra <= l_t(4 downto 1) & "10";
            else cr_ra <= l_t(4 downto 1) & "01"; end if;
            -- the word addressed now is in cr_q two clocks on: it goes into the DDA's increment register
            -- then (ca_ld), and a wrap / seed word is added one clock after that (ca_en), on the same edge
            -- that puts the pixel step back
            l_v1 <= l_run; l_c1 <= l_t(4 downto 1); l_o1 <= l_t(0);
            l_v2 <= l_v1;  l_c2 <= l_c1;            l_o2 <= l_o1;
            for k in 0 to 14 loop
                if l_v1 = '1' and l_c1 = k then ca_ld(k) <= '1'; else ca_ld(k) <= '0'; end if;
                if l_v2 = '1' and l_o2 = '0' and l_c2 = k then ca_en(k) <= '1'; else ca_en(k) <= '0'; end if;
            end loop;

            -- the block column of the pixel at stage 0 (output column = DDA pixel - 1), in quarter virtual px
            v_x2 := s_xs & '0';
            if b_run = '0' then px_a <= (others => '0');
            elsif b_d(1) = '0' then px_a <= resize(s_xs, 13);
            else px_a <= px_a + resize(v_x2, 13); end if;
        end if;
    end process p_load;

    g_pha : for k in 0 to 7 generate
        p_pha : process(clk)
        begin
            if rising_edge(clk) then
                if ca_ld(k) = '1' then pa_inc(k) <= cr_q(21 downto 0); end if;
                if ca_rst = '1' then
                    pa_acc(k) <= (others => '0');
                elsif b_d(1) = '1' or ca_en(k) = '1' then
                    pa_acc(k) <= pa_acc(k) + pa_inc(k);
                end if;
            end if;
        end process p_pha;
    end generate g_pha;

    g_sgn : for k in 0 to 6 generate
        p_sgn : process(clk)
        begin
            if rising_edge(clk) then
                if ca_ld(k + 8) = '1' then ca_inc(k) <= cr_q(24 downto 0); end if;
                if ca_rst = '1' then
                    ca_acc(k) <= (others => '0');
                elsif b_run = '1' or ca_en(k + 8) = '1' then
                    ca_acc(k) <= ca_acc(k) + ca_inc(k);
                end if;
            end if;
        end process p_sgn;
    end generate g_sgn;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD AND CAMERA GRID: luma (or the Osc lattice) -> gain table
    -- (K6) -> box of 32 px (16 px on the narrower rasters), edge-replicated and
    -- reloaded at every line start -> sampled at the last pixel of each block
    -- (32 virtual px) -> vertical one-pole per block column -> 2-bit level,
    -- written to this field's bank of the grid on every line (the last line of
    -- a block row stays).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(15 downto 0);
        variable v_dl : signed(12 downto 0);
        variable v_n : signed(13 downto 0);
        variable v_avg : unsigned(15 downto 0);
        variable v_bx : unsigned(5 downto 0);
        variable v_x2 : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
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
            -- gain table (p_gtram), then its fabric copy
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_tv <= gt_q;

            -- box: sum of the last 32 (16) samples; the samples before the line start count as the first one
            if s_v2 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            elsif s_b32 = '1' then bx_wp <= (others => '0'); bx_rp <= to_unsigned(226, 8);
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(242, 8); end if;
            bx_f <= bx_q;
            if s_bcnt(5) = '0' and (s_b32 = '1' or s_bcnt(4) = '0') then v_tap := s_sfirst; else v_tap := bx_f; end if;
            s_df <= signed(resize(s_tv, 17)) - signed(resize(v_tap, 17));
            s_tvd <= s_tv; s_v3 <= s_v2;
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_sfirst <= s_tv;
                s_bcnt <= to_unsigned(1, 6);
            elsif s_v2 = '1' then
                if s_bcnt(5) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_ld = '1' then
                if s_b32 = '1' then s_bsum <= shift_left(resize(s_tvd, 22), 5);
                else s_bsum <= shift_left(resize(s_tvd, 22), 4); end if;
            elsif s_v3 = '1' then
                s_bsum <= unsigned(signed(s_bsum) + resize(s_df, 22));
            end if;
            s_v4 <= s_v3;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif s_ld = '1' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- the block row of this line (virtual lines, Q8; the row is bits 18 .. 13)
            if g_avr = '1' then
                if s_yf = '1' then s_ya <= resize(s_ys(11 downto 1), 19); else s_ya <= s_ya + resize(s_ys, 19); end if;
                s_yf <= '0';
            elsif s_zone = '1' then
                s_yf <= '1';
            end if;

            -- the block column under the box (s_bsum is the box of the column s_xa belongs to while s_v4)
            v_x2 := s_xs & '0';
            if s_v4 = '1' then s_xa <= s_xa + resize(v_x2, 13); else s_xa <= resize(s_xs, 13); end if;
            v_bx := s_xa(12 downto 7);
            if s_b32 = '1' then v_avg := s_bsum(20 downto 5); else v_avg := s_bsum(19 downto 4); end if;
            e_val <= v_avg(15 downto 4); e_bx <= v_bx; e_v <= s_v4;
            ir_f <= ir_q;
            -- a block has ended: its last box value, and the one-pole's old value
            if e_v = '1' and (s_v4 = '0' or v_bx /= e_bx) then f_ev <= '1'; else f_ev <= '0'; end if;
            f_val <= e_val; f_bx <= e_bx; f_old <= ir_f;
            s_dd <= signed(resize(f_val, 13)) - signed(resize(f_old, 13));
            g_val <= f_val; g_old <= f_old; g_bx <= f_bx; g_ev <= f_ev; g_fl <= s_fl;
            -- q += (box - q) >> 4 (3 at 720p / 1080i / 480p / 576p, 2 at 480i / 576i)
            case to_integer(s_ish) is
                when 2      => v_dl := shift_right(s_dd, 4);
                when 1      => v_dl := shift_right(s_dd, 3);
                when others => v_dl := shift_right(s_dd, 2);
            end case;
            if g_fl = '1' then v_n := signed(resize(g_val, 14));
            else v_n := signed(resize(g_old, 14)) + resize(v_dl, 14); end if;
            ir_we <= g_ev; ir_wa <= g_bx; ir_wd <= unsigned(v_n(11 downto 0));
            gr_we <= g_ev; gr_wa <= s_bank & s_ya(18 downto 13) & g_bx;
            if v_n(11 downto 10) /= "00" then gr_wd <= "11"; else gr_wd <= unsigned(v_n(9 downto 8)); end if;

            -- gain table fill (zone, after the glides): T(i) = (i * 7/8 K6) >> 6 (Osc: 7/4 K6) by accumulation
            gt_we <= '0';
            if q_go = '1' then
                gt_run <= '1'; gt_i <= (others => '0'); gt_acc <= (others => '0');
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_i;
                gt_wd <= gt_acc(21 downto 6);
                gt_acc <= gt_acc + resize(z_gk, 22); gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] region code and phase select (stages 1, 2) and [F] TONE (stages 3 .. 13)
    ---------------------------------------------------------------------------
    p_tone : process(clk)
        variable v_s1, v_s2, v_s3, v_t2, v_t3 : std_logic;
        variable v_k : unsigned(1 downto 0);
        variable v_f, v_2w : unsigned(13 downto 0);
        variable v_lo : boolean;
        variable v_d : unsigned(13 downto 0);
        variable v_own, v_nb : unsigned(1 downto 0);
        variable v_r2 : unsigned(12 downto 0);
        variable v_q, v_m : unsigned(3 downto 0);
        variable v_mg : unsigned(1 downto 0);
        variable v_pa, v_pb : unsigned(5 downto 0);
        variable v_p : unsigned(5 downto 0);
        variable v_v : unsigned(9 downto 0);
        variable v_nda : signed(15 downto 0);
        variable v_g1, v_g2, v_g3 : unsigned(13 downto 0);
        variable v_nd : signed(2 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: the fold decisions.  Fold 1: the sign of test 0 (Pleat: the strip parity); fold 2: test 1 or 2;
            -- fold 3: one of tests 3 .. 6
            if s_pleat = '1' then v_s1 := ca_acc(0)(23); else v_s1 := not ca_acc(0)(24); end if;
            if v_s1 = '1' then v_t2 := ca_acc(2)(24); else v_t2 := ca_acc(1)(24); end if;
            v_s2 := s_m2 and not v_t2;
            if v_s2 = '1' then
                if v_s1 = '1' then v_t3 := ca_acc(6)(24); else v_t3 := ca_acc(5)(24); end if;
            else
                if v_s1 = '1' then v_t3 := ca_acc(4)(24); else v_t3 := ca_acc(3)(24); end if;
            end if;
            v_s3 := s_m3 and not v_t3;
            x1_code <= v_s3 & v_s2 & v_s1;
            -- camera grid read: the other bank, this pixel's block
            gr_ra <= (not s_bank) & s_ya(18 downto 13) & px_a(12 downto 7);

            -- 2: the region's phase (bands, Q2.12)
            case to_integer(x1_code) is
                when 0      => x2_ph <= unsigned(pa_acc(0)(21 downto 8));
                when 1      => x2_ph <= unsigned(pa_acc(1)(21 downto 8));
                when 2      => x2_ph <= unsigned(pa_acc(2)(21 downto 8));
                when 3      => x2_ph <= unsigned(pa_acc(3)(21 downto 8));
                when 4      => x2_ph <= unsigned(pa_acc(4)(21 downto 8));
                when 5      => x2_ph <= unsigned(pa_acc(5)(21 downto 8));
                when 6      => x2_ph <= unsigned(pa_acc(6)(21 downto 8));
                when others => x2_ph <= unsigned(pa_acc(7)(21 downto 8));
            end case;

            -- 3: fabric copies of the footprint and the camera level
            x3_ph <= x2_ph;
            x3_w  <= wr_q(11 downto 0);
            x3_k  <= gr_q;

            -- 4: the band or line the pixel is in: its start (as -start), its end - 1, the position.
            -- Grey Bold: the black, dark and light bands are g wide (1, 5/6, 2/3, 1/2 band by the camera level) and
            -- white takes the rest of the four-band cycle, so every cycle keeps all four tones.
            -- Fine: the line is the last 1/6, 1/3, 1/2, 3/4 of its band, never under two pixels.
            -- Ink Bold: the white band is the last 1, 1.25, 1.5, 1.75 bands of the two.
            v_k := x3_k;
            v_2w := resize(x3_w & '0', 14);
            case to_integer(v_k) is
                when 0      => v_g1 := to_unsigned(4096, 14); v_g2 := to_unsigned(8192, 14); v_g3 := to_unsigned(12288, 14);
                when 1      => v_g1 := to_unsigned(3413, 14); v_g2 := to_unsigned(6826, 14); v_g3 := to_unsigned(10239, 14);
                when 2      => v_g1 := to_unsigned(2731, 14); v_g2 := to_unsigned(5462, 14); v_g3 := to_unsigned(8193, 14);
                when others => v_g1 := to_unsigned(2048, 14); v_g2 := to_unsigned(4096, 14); v_g3 := to_unsigned(6144, 14);
            end case;
            if s_gb = '1' then
                x4_ts <= x3_ph;
                if x3_ph >= v_g3 then
                    x4_np <= -signed(resize(v_g3, 15)); x4_eh <= to_unsigned(16383, 14); x4_b <= "11";
                elsif x3_ph >= v_g2 then
                    x4_np <= -signed(resize(v_g2, 15)); x4_eh <= v_g3 - 1; x4_b <= "10";
                elsif x3_ph >= v_g1 then
                    x4_np <= -signed(resize(v_g1, 15)); x4_eh <= v_g2 - 1; x4_b <= "01";
                else
                    x4_np <= to_signed(0, 15); x4_eh <= v_g1 - 1; x4_b <= "00";
                end if;
            elsif s_fine = '1' then
                case to_integer(v_k) is
                    when 0      => v_f := to_unsigned(683, 14);
                    when 1      => v_f := to_unsigned(1365, 14);
                    when 2      => v_f := to_unsigned(2048, 14);
                    when others => v_f := to_unsigned(3072, 14);
                end case;
                if x3_w(11) = '1' and x3_w(10 downto 0) /= 0 then x4_np <= to_signed(0, 15);       -- 2 w > one band
                elsif v_2w > v_f then x4_np <= signed(resize(v_2w, 15)) - 4096;
                else x4_np <= signed(resize(v_f, 15)) - 4096; end if;
                x4_ts <= "00" & x3_ph(11 downto 0);
                x4_eh <= to_unsigned(4095, 14);
                x4_b <= x3_ph(13 downto 12);
            else
                case to_integer(v_k) is
                    when 0      => x4_np <= to_signed(-4096, 15);
                    when 1      => x4_np <= to_signed(-3072, 15);
                    when 2      => x4_np <= to_signed(-2048, 15);
                    when others => x4_np <= to_signed(-1024, 15);
                end case;
                x4_ts <= '0' & x3_ph(12 downto 0);
                x4_eh <= to_unsigned(8191, 14);
                x4_b <= x3_ph(13 downto 12);
            end if;
            x4_w <= x3_w;

            -- 5: the position in the line (>= 0 inside), the distance to the end, the three tones
            x5_da <= signed(resize(x4_ts, 16)) + resize(x4_np, 16);
            x5_ts <= x4_ts;
            x5_nts <= x4_eh - x4_ts;
            if s_gb = '1' or x4_np = 0 then x5_full <= '1'; else x5_full <= '0'; end if;
            x5_tb <= f_tl(x4_b, s_ink, s_fine);
            x5_tp <= f_tl(x4_b - 1, s_ink, s_fine);
            x5_tn <= f_tl(x4_b + 1, s_ink, s_fine);
            x5_w <= x4_w;

            -- 6: the two edge distances
            if x5_da(15) = '0' then
                x6_d1 <= unsigned(x5_da(13 downto 0)); x6_d2 <= x5_nts; x6_inl <= '1';
            else
                v_nda := -x5_da;
                x6_d1 <= unsigned(v_nda(13 downto 0)); x6_d2 <= x5_ts; x6_inl <= '0';
            end if;
            x6_full <= x5_full; x6_tb <= x5_tb; x6_tp <= x5_tp; x6_tn <= x5_tn; x6_w <= x5_w;

            -- 7: the nearer edge, this side's tone and the tone across it
            v_lo := x6_d1 <= x6_d2;
            if v_lo then v_d := x6_d1; else v_d := x6_d2; end if;
            if v_d(13 downto 12) /= "00" then x7_d <= (others => '1'); else x7_d <= v_d(11 downto 0); end if;
            if x6_inl = '1' then
                v_own := x6_tb;
                if x6_full = '0' then v_nb := "00";
                elsif v_lo then v_nb := x6_tp;
                else v_nb := x6_tn; end if;
            else
                v_own := "00";
                if v_lo then v_nb := x6_tb; else v_nb := x6_tp; end if;
            end if;
            x_own(7) <= v_own;
            x_dl(7) <= signed(resize(v_nb, 3)) - signed(resize(v_own, 3));
            x7_w <= x6_w;

            -- 8 .. 11: q = min(8, 16 d / w): this side's share of the pixel is (8 + q) / 16
            if resize(x7_d & '0', 13) >= resize(x7_w, 13) then x8_f8 <= '1'; else x8_f8 <= '0'; end if;
            x8_r <= x7_d(10 downto 0) & '0'; x8_w <= x7_w;
            v_r2 := x8_r & '0';
            if v_r2 >= resize(x8_w, 13) then x9_r <= resize(v_r2 - resize(x8_w, 13), 12); x9_q <= "001";
            else x9_r <= v_r2(11 downto 0); x9_q <= "000"; end if;
            x9_f8 <= x8_f8; x9_w <= x8_w;
            v_r2 := x9_r & '0';
            if v_r2 >= resize(x9_w, 13) then x10_r <= resize(v_r2 - resize(x9_w, 13), 12); x10_q <= x9_q(1 downto 0) & '1';
            else x10_r <= v_r2(11 downto 0); x10_q <= x9_q(1 downto 0) & '0'; end if;
            x10_f8 <= x9_f8; x10_w <= x9_w;
            v_r2 := x10_r & '0';
            if v_r2 >= resize(x10_w, 13) then x11_q <= x10_q(1 downto 0) & '1';
            else x11_q <= x10_q(1 downto 0) & '0'; end if;
            x11_f8 <= x10_f8;
            for i in 8 to 11 loop x_own(i) <= x_own(i - 1); x_dl(i) <= x_dl(i - 1); end loop;

            -- 12: the tone in sixteenths of a step: 16 own + (across - own) * (8 - q)
            if x11_f8 = '1' then v_m := "0000"; else v_m := to_unsigned(8, 4) - resize(x11_q, 4); end if;
            v_nd := -x_dl(11);
            if x_dl(11)(2) = '1' then v_mg := unsigned(v_nd(1 downto 0)); else v_mg := unsigned(x_dl(11)(1 downto 0)); end if;
            if v_mg(0) = '1' then v_pa := resize(v_m, 6); else v_pa := (others => '0'); end if;
            if v_mg(1) = '1' then v_pb := resize(v_m & '0', 6); else v_pb := (others => '0'); end if;
            v_p := v_pa + v_pb;
            x12_v <= f_addsub(x_own(11) & "0000", v_p, x_dl(11)(2));

            -- 13: luma = 64 + 876 v / 48
            v_v := resize(x12_v, 10);
            s_y23 <= to_unsigned(64, 10) + shift_left(v_v, 4) + shift_left(v_v, 1) + shift_right(v_v, 2);
        end if;
    end process p_tone;

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
            s_ya3 <= resize(s_y23, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya3 + shift_left(resize(s_yb, 12), 1);
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

end architecture crease;
