-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Koch v0.1.0 -- Straightedge.  Helge von Koch's curve (1904) and the snowflake built from it, as drawn in a
-- mathematics plate.  Luma only (U = V = 512 on every pixel); every edge on the screen is a straight line.
--
-- The picture is a fold cascade in "heights" (distances to straight lines; barycentric coordinates of the
-- equilateral triangle on a segment), where every step is a sort, an add or a negation:
--     a1, a2, a3      inward distances from the base triangle's sides (a1 + a2 + a3 = H0, its altitude)
--     (U, W)          = (a_max + a_min, a_mid + a_min): the nearest side's frame; Y0 = -a_min
--     level j         M, m = sort(U, W);  t = M - 2/3 H(j-1);  t < 0: (U, W) = (-t, m + t)  (the spike)
--                                                              else   (U, W) = (t, m)
--     Y = H(n) - U - W after the n levels that are on; the growing level's fold is truncated at h, so its
--     spikes rise out of the edges as truncated triangles
-- Y is the distance to the curve (> 0 outside).  Inside, bands of -Y; outside, bands of Z = Y after level 1
-- (bold echoes of the hexagram; Dive: Z = Y0, the plain corner).  The unit is u = P / 2^k of a pixel (P the band
-- period), so a band is 2^k LSBs and the band number and the distance to its edge are bit fields.
--
--   K1  X        the flake's centre (+-960 virtual px) / the tip (+-600)           (glided)
--   K2  Y        (+-540 / +-337)                                                   (glided)
--   K3  Scale    side 199 .. 6360 virtual px (zooming into the right notch) / zoom offset     (glided)
--   K4  Bands    band period 16 .. 256 virtual px, exponential                     (glided)
--   K5  Drift    bipolar speed (Hypnos): bands stream out of (into) the curve; Dive: + the endless zoom
--   K6  Camera   a screen of little hexagram flakes, one per 32-px block, sized by the block's luma (glided);
--                a white film in Grey, XOR in Ink
--   S7  Glide    Off / On      S8 View   Flake / Dive       S9 Tone   Grey / Ink
--   S10 Weight   Bold / Fine   S11 Source Video / Osc
--   P12 Depth    the generations, continuous: triangle -> the full curve          (glided)
--
-- Sections (the integer model .straightedge_work/rb_koch/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width / height, generated avid
--   [B] CONTROLS                         switches (the holds and glides run in [C])
--   [C] FIELD ENGINE                     Hardline flute's microcoded engine (koch.py): holds, glides, drift,
--                                        every per-field constant
--   [L] LINE ENGINE                      s = a1 + a2 of the next line; the star screen's row
--   [D] SOURCE FIELD                     Osc lattice, gain table, 32-px box, block-column one-pole, camera grid
--   [K] THE CASCADE                      front F1 .. F3, seven levels (sort / fold / accumulate)
--   [E] BANDS, RIM, STARS, BLENDS        band number and edge distance, coverage, three blends
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  DDA pixel i is in the beam at T + 8 + i, its fold distance Y at T + 28 + i,
-- the blended luma at T + 43 + i, the output register at T + 49 + j for column j (the [1 2 1] centred on DDA
-- pixel j + 1).  One latency (48) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.koch_pkg.all;
use work.all;

architecture koch of program_top is
    constant C_LAT : integer := 48;   -- = processing_delay_clks

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
    function f_il(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others             => return '0';
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- the four flat greys
    function f_g4(n : unsigned(1 downto 0)) return unsigned is
    begin
        case to_integer(n) is
            when 0      => return to_unsigned(64, 10);
            when 1      => return to_unsigned(356, 10);
            when 2      => return to_unsigned(648, 10);
            when others => return to_unsigned(940, 10);
        end case;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width / height
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
    signal s_lcnt : unsigned(10 downto 0) := (others => '0');
    signal s_h    : unsigned(10 downto 0) := to_unsigned(1080, 11);

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- sync delay: a block-RAM ring, read C_LAT - 2 behind the write (+ the read register = C_LAT - 1, + one
    -- fabric register = C_LAT)
    type t_dl is array (0 to 255) of std_logic_vector(3 downto 0);
    signal dl_ram : t_dl := (others => "0011");
    attribute no_rw_check of dl_ram : signal is true;
    signal dl_wa : unsigned(7 downto 0) := (others => '0');
    signal dl_q, dl_q2 : std_logic_vector(3 downto 0) := "0011";
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 31) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] switches
    signal s_glide : std_logic := '1';
    signal s_dive, s_ink, s_fine, s_src : std_logic := '0';
    signal s_wbank : std_logic := '0';
    signal s_vsh : unsigned(2 downto 0) := "100";

    -- [C] the per-field engine (Hardline flute's): 37-bit accumulator, 128 x 32 register file, serial
    -- multiplier and shift, microprogram and table in block RAM
    constant OP_LD   : std_logic_vector(3 downto 0) := x"0";
    constant OP_ADD  : std_logic_vector(3 downto 0) := x"1";
    constant OP_SUB  : std_logic_vector(3 downto 0) := x"2";
    constant OP_MUL  : std_logic_vector(3 downto 0) := x"3";
    constant OP_LDI  : std_logic_vector(3 downto 0) := x"4";
    constant OP_ADDI : std_logic_vector(3 downto 0) := x"5";
    constant OP_SUBI : std_logic_vector(3 downto 0) := x"6";
    constant OP_MULI : std_logic_vector(3 downto 0) := x"7";
    constant OP_ST   : std_logic_vector(3 downto 0) := x"8";
    constant OP_IN   : std_logic_vector(3 downto 0) := x"9";
    constant OP_OUT  : std_logic_vector(3 downto 0) := x"A";
    constant OP_SHR  : std_logic_vector(3 downto 0) := x"B";
    constant OP_SKIP : std_logic_vector(3 downto 0) := x"C";
    constant OP_JMP  : std_logic_vector(3 downto 0) := x"D";
    constant OP_CALL : std_logic_vector(3 downto 0) := x"E";
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";      -- 0 return, 1 halt, 2 normalise
    signal rf_ram : t_c_ko_rfi := C_KO_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run, e_rund : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc, e_lr : unsigned(9 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";
    signal x_mul, x_shr, x_nrm, x_halt, x_out, x_st : std_logic := '0';
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal w_in  : unsigned(15 downto 0);
    signal ft_adr : unsigned(8 downto 0) := (others => '0');
    signal ft_q  : std_logic_vector(15 downto 0) := (others => '0');
    -- the engine's outputs (per-field constants)
    type t_s18a is array (natural range <>) of signed(17 downto 0);
    signal c_nk  : t_s18a(1 to 7) := (others => (others => '0'));      -- -K_j = -2/3 H(j-1); 0 = the level is off
    signal c_md  : std_logic_vector(13 downto 0) := (others => '0');   -- level modes: 00 off, 01 whole, 10 growing
    signal c_nh  : signed(17 downto 0) := (others => '0');             -- -h of the growing level
    signal c_p0, c_s0 : signed(17 downto 0) := (others => '0');
    signal c_dp  : unsigned(15 downto 0) := (others => '0');
    signal c_ds  : unsigned(16 downto 0) := (others => '0');
    signal c_h0, c_hz, c_hf : unsigned(16 downto 0) := (others => '0');   -- altitudes: the triangle, after level 1, after the last level on
    signal c_doff : unsigned(13 downto 0) := (others => '0');
    signal c_kb  : unsigned(2 downto 0) := "011";
    signal c_mu  : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal c_rw  : unsigned(11 downto 0) := (others => '0');
    signal c_hw  : unsigned(9 downto 0) := (others => '0');
    signal c_vx0 : unsigned(11 downto 0) := (others => '0');
    signal c_g6  : unsigned(12 downto 0) := (others => '0');

    -- [L] line engine
    signal l_kick, e_kr : std_logic := '0';
    signal e_lc  : unsigned(4 downto 0) := to_unsigned(31, 5);
    signal e_s   : signed(27 downto 0) := (others => '0');             -- s = a1 + a2, Q10
    signal e_a3  : signed(18 downto 0) := (others => '0');             -- H0 - s
    signal e_ns  : signed(17 downto 0) := (others => '0');             -- s - H0
    signal e_vy  : unsigned(13 downto 0) := (others => '0');           -- the star screen's line: Q8 within the block row
    signal e_rb  : unsigned(10 downto 0) := (others => '0');           -- row * 60

    -- [D] source field (riley's / dazzle's front: Osc lattice, gain table, box)
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
    signal gt_acc : signed(23 downto 0) := (others => '0');
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal bx_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_f : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    constant s_b64 : std_logic := '0';                                 -- the box is 32 px on every raster
    signal s_df   : signed(16 downto 0) := (others => '0');
    signal s_bsum : unsigned(21 downto 0) := (others => '0');
    signal s_eb : unsigned(15 downto 0) := (others => '0');
    signal s_v5, s_v5d : std_logic := '0';
    signal sx_l  : unsigned(5 downto 0) := (others => '0');
    signal sx_col, sx_pcol : unsigned(5 downto 0) := (others => '0');
    signal sx_cy, sx_pend : std_logic := '0';
    signal sy_l  : unsigned(13 downto 0) := (others => '0');
    signal sy_base : unsigned(10 downto 0) := (others => '0');
    signal sy_seen : std_logic := '0';
    type t_cs is array (0 to 63) of unsigned(15 downto 0);
    signal cs_ram : t_cs := (others => (others => '0'));
    attribute no_rw_check of cs_ram : signal is true;
    signal cs_q, cs_f, cs_b, cs_wd : unsigned(15 downto 0) := (others => '0');
    signal cs_d  : signed(16 downto 0) := (others => '0');
    signal cs_c  : unsigned(5 downto 0) := (others => '0');
    signal cs_st : std_logic_vector(2 downto 0) := "000";
    signal cs_we, cs_fl : std_logic := '0';
    -- the camera grid: two banks of 60 x 34 blocks x 4 bits (the star size), ping-ponged per field
    type t_cm is array (0 to 4095) of unsigned(3 downto 0);
    signal cm_ram : t_cm := (others => (others => '0'));
    attribute no_rw_check of cm_ram : signal is true;
    signal cm_wa, cm_ra : unsigned(11 downto 0) := (others => '0');
    signal cm_wd, cm_q, cm_f : unsigned(3 downto 0) := (others => '0');
    signal cm_we : std_logic := '0';

    -- [K] the cascade
    signal gb_p  : signed(27 downto 0) := (others => '0');             -- p = a1 - a2, Q10
    signal gb_s  : signed(17 downto 0) := (others => '0');
    signal gb_a3 : signed(18 downto 0) := (others => '0');
    signal gb_ns : signed(17 downto 0) := (others => '0');
    signal gb_vy : unsigned(7 downto 0) := (others => '0');             -- the star line within its block row, Q3
    signal gb_rb : unsigned(10 downto 0) := (others => '0');
    signal f1_ap : signed(17 downto 0) := (others => '0');
    signal f2_e, f2_f : signed(18 downto 0) := (others => '0');
    signal f3_he, f3_wf, f3_ne : signed(17 downto 0) := (others => '0');
    signal f3_sel : std_logic := '0';
    signal f4_y  : signed(17 downto 0) := (others => '0');              -- Y0
    signal lv_U, lv_W, lv_g : t_s18a(0 to 7) := (others => (others => '0'));
    signal lv_hi, lv_lo, lv_ga : t_s18a(1 to 7) := (others => (others => '0'));
    signal e1_sum, e1_r, e2_y : signed(17 downto 0) := (others => '0');
    signal z1_sum, z1_y : signed(17 downto 0) := (others => '0');
    -- the echo field's delay: a 16-deep ring (sign & low 14 bits), read 8 (Dive: 12) behind the write
    type t_zr is array (0 to 15) of std_logic_vector(15 downto 0);
    signal zr_ram : t_zr := (others => (others => '0'));
    attribute no_rw_check of zr_ram : signal is true;
    signal zr_wa, zr_ra : unsigned(3 downto 0) := (others => '0');
    signal zr_wd, zr_q, zr_f : std_logic_vector(15 downto 0) := (others => '0');

    -- [E] bands, rim, stars, blends
    signal s1_ins, s2_ins, s3_ins : std_logic := '0';
    signal s1_a  : unsigned(13 downto 0) := (others => '0');
    signal s1_r1, s1_y : signed(17 downto 0) := (others => '0');
    signal s2_phi : unsigned(13 downto 0) := (others => '0');
    signal s2_mn : signed(17 downto 0) := (others => '0');
    signal s3_dr, s4_dr : signed(6 downto 0) := (others => '0');
    signal s3_b  : unsigned(1 downto 0) := (others => '0');
    signal s3_half : std_logic := '0';
    signal s3_d  : unsigned(10 downto 0) := (others => '0');
    signal s4_ds : signed(6 downto 0) := (others => '0');
    signal s4_own, s4_far, s5_own, s5_far, s6_own, s6_far : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s5_pb, s5_pr : signed(12 downto 0) := (others => '0');
    signal s6_cb : unsigned(4 downto 0) := (others => '0');
    signal cr_p  : unsigned(19 downto 0) := (others => '0');           -- the rim coverage, 4 x 5 bits, T + 34 .. T + 37
    signal c_rt  : unsigned(9 downto 0) := to_unsigned(940, 10);       -- the rim's tone
    type t_s11a is array (1 to 3) of signed(10 downto 0);
    type t_s16a is array (1 to 3) of signed(15 downto 0);
    type t_u10a is array (1 to 3) of unsigned(9 downto 0);
    type t_u5a is array (1 to 3) of unsigned(4 downto 0);
    signal bl_df : t_s11a := (others => (others => '0'));
    signal bl_pa, bl_pb : t_s16a := (others => (others => '0'));
    signal bl_cv : t_u5a := (others => (others => '0'));
    signal bl_b1, bl_b2, bl_y : t_u10a := (others => to_unsigned(64, 10));
    -- the star screen
    signal v0_lx : unsigned(5 downto 0) := (others => '0');
    signal v0_col : unsigned(5 downto 0) := (others => '0');
    signal v1_qx : signed(8 downto 0) := (others => '0');
    signal v2_p2, v2_p3, v3_mx, v3_mn, v3_p3, v4_mx, v4_mn, v5_hd, v6_sd : signed(9 downto 0) := (others => '0');
    signal v4_r, v5_r, v6_r, v7_r : unsigned(3 downto 0) := (others => '0');
    signal v7_c  : signed(11 downto 0) := (others => '0');
    signal v8_cs : unsigned(4 downto 0) := (others => '0');

    -- [F] output conditioning
    signal s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y45, s_y46, s_y47 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    -- sync delay ring: written every clock, read C_LAT - 2 behind
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            dl_wa <= dl_wa + 1;
            dl_ram(to_integer(dl_wa)) <= '0' & g_avid & data_in.vsync_n & data_in.hsync_n;
            dl_q <= dl_ram(to_integer(dl_wa - (C_LAT - 2)));
            dl_q2 <= dl_q;
        end if;
    end process p_dl;

    -- gain table: written in the zone, read by the source path
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(s_sl));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
        end if;
    end process p_gtram;

    -- box delay: read 30 (or 62) samples behind the write (+ two registers = 32 or 64)
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_v2 = '1' then bx_ram(to_integer(bx_wp)) <= s_tv; end if;
        end if;
    end process p_bxram;

    -- the block-column store of the source field (read-modify-write, four clocks apart)
    p_csram : process(clk)
    begin
        if rising_edge(clk) then
            cs_q <= cs_ram(to_integer(cs_c));
            if cs_we = '1' then cs_ram(to_integer(cs_c)) <= cs_wd; end if;
        end if;
    end process p_csram;

    -- the camera grid: this field writes one bank, the star screen reads the other
    p_cmram : process(clk)
    begin
        if rising_edge(clk) then
            cm_q <= cm_ram(to_integer(cm_ra));
            if cm_we = '1' then cm_ram(to_integer(cm_wa)) <= cm_wd; end if;
        end if;
    end process p_cmram;

    -- the echo field's delay ring: written every clock
    p_zrram : process(clk)
    begin
        if rising_edge(clk) then
            zr_q <= zr_ram(to_integer(zr_ra));
            zr_ram(to_integer(zr_wa)) <= zr_wd;
        end if;
    end process p_zrram;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] switches
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
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

            -- [A] width (avid pixels per line, max over the field) and height (active lines per field)
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            if s_zgo = '1' then
                s_lcnt <= (others => '0');
            elsif data_in.avid = '1' and s_avq = '0' and s_lcnt /= 2047 then
                s_lcnt <= s_lcnt + 1;
            end if;

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_h <= s_lcnt;
                s_glide <= registers_in(6)(0);
                s_dive  <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_il    <= s_ilr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                s_wbank <= not s_wbank;             -- the camera grid's banks swap once per field
            end if;
            g_wm1 <= s_w - 1;
            if s_ys <= 256 then s_vsh <= "100"; elsif s_ys <= 576 then s_vsh <= "011"; else s_vsh <= "010"; end if;
            case to_integer(unsigned(s_tid)) is              -- the Osc lattice's line step, ys * 121 / 256
                when 0          => s_oys <= to_unsigned(537, 10);
                when 4          => s_oys <= to_unsigned(272, 10);
                when 8          => s_oys <= to_unsigned(453, 10);
                when 12         => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10);
                when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others     => s_oys <= to_unsigned(121, 10);
            end case;
            -- line 0 once the engine has halted
            e_rund <= e_run;
            l_kick <= e_rund and not e_run;

            -- (the syncs and the generated avid are delayed in the dl_ram ring, p_dl)
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 30);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] THE PER-FIELD ENGINE (Hardline flute's).  Started at every zone start
    -- from address 0; three clocks per micro-op (register-file read, the ROM word
    -- copied to fabric / operand, decode and next address / execute), serial
    -- multiply (18 steps) and shift.  ACC = 37 bits; MUL = ACC(17:0) x operand
    -- (17:0), both signed, by add-and-shift-right with the last step subtracting.
    -- koch.py holds the microprogram, its assembler and its emulator.
    ---------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(s_w, 16)             when x"8",
        resize(s_h, 16)             when x"9",
        resize(s_ys, 16)            when x"A",
        unsigned(ft_q)              when x"B",
        resize(e_cnt, 16)           when x"C",
        resize(s_xs, 16)            when x"D",
        resize(unsigned(registers_in(to_integer(unsigned(e_irf(2 downto 0))))), 16) when others;

    e_out <= '1' when e_run = '1' and e_ph = "10" and x_out = '1' else '0';

    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_gate, v_sub, v_shift, v_last, v_cond, v_fetch, v_nz : std_logic;
        variable v_bx  : signed(36 downto 0);
        variable v_t   : signed(37 downto 0);
        variable v_sum : signed(36 downto 0);
    begin
        if rising_edge(clk) then
            v_op := e_irf(15 downto 12);
            v_arg := e_irf(11 downto 0);
            v_fetch := '0';
            ft_q <= C_KO_FT(to_integer(ft_adr));
            if e_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(e_ir(6 downto 0))));
            end if;
            if e_run = '1' and e_ph = "10" and x_st = '1' then
                rf_ram(to_integer(unsigned(e_irf(6 downto 0)))) <= std_logic_vector(e_acc(31 downto 0));
            end if;

            if s_zgo = '1' then
                e_run <= '1'; e_pc <= (others => '0'); e_ph <= "11";
            elsif e_run = '1' then
                case e_ph is
                    when "11" =>                          -- fetch (first micro-op)
                        v_fetch := '1';
                        e_ph <= "00";
                    when "00" =>                          -- register-file read
                        e_irf <= e_ir;
                        e_ph <= "01";
                    when "01" =>                          -- operand, accumulator clear, next address
                        case v_op is
                            when OP_LD | OP_ADD | OP_SUB => e_b <= resize(signed(rf_q), 37);
                            when OP_LDI | OP_ADDI | OP_SUBI => e_b <= resize(signed(v_arg), 37);
                            when OP_IN   => e_b <= signed(resize(w_in, 37));
                            when OP_MUL  => e_b <= shift_left(resize(signed(rf_q(17 downto 0)), 37), 18);
                            when OP_MULI => e_b <= shift_left(resize(signed(v_arg), 37), 18);
                            when others  => null;
                        end case;
                        if v_op = OP_LD or v_op = OP_LDI or v_op = OP_IN then
                            e_acc <= (others => '0');
                        elsif v_op = OP_MUL or v_op = OP_MULI then
                            e_acc(36 downto 18) <= (others => '0');
                        end if;
                        if v_op = OP_MUL or v_op = OP_MULI then e_cnt <= to_unsigned(17, 5);
                        elsif v_op = OP_SHR then e_cnt <= unsigned(v_arg(4 downto 0)) - 1;
                        elsif v_op = OP_SPC then e_cnt <= (others => '0'); end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := e_acc(36);
                            when "001"  => v_cond := s_glide;
                            when "010"  => v_cond := s_ink;
                            when "011"  => v_cond := s_fine;
                            when others => v_cond := s_dive;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then e_pc <= e_pc + 2;
                        elsif v_op = OP_JMP then e_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then e_pc <= unsigned(v_arg(9 downto 0)); e_lr <= e_pc + 1;
                        elsif v_op = OP_SPC and v_arg(1 downto 0) = "00" then e_pc <= e_lr;
                        else e_pc <= e_pc + 1; end if;
                        x_mul <= '0'; x_shr <= '0'; x_nrm <= '0'; x_halt <= '0'; x_out <= '0'; x_st <= '0';
                        x_gate <= '0'; x_subr <= '0'; x_shift <= '0'; x_cz <= '0';
                        case v_op is
                            when OP_LD | OP_ADD | OP_LDI | OP_ADDI | OP_IN => x_gate <= '1';
                            when OP_SUB | OP_SUBI => x_gate <= '1'; x_subr <= '1';
                            when OP_MUL | OP_MULI => x_mul <= '1'; x_shift <= '1'; x_gate <= e_acc(0);
                            when OP_SHR  =>
                                x_shr <= '1'; x_shift <= '1';
                                if v_arg(4 downto 0) = "00001" then x_cz <= '1'; end if;
                            when OP_ST   => x_st <= '1';
                            when OP_OUT  => x_out <= '1';
                            when OP_SPC  => x_nrm <= v_arg(1); x_halt <= v_arg(0) and not v_arg(1);
                            when others  => null;
                        end case;
                        e_ph <= "10";
                    when others =>                        -- execute (the adder sees registers only)
                        v_gate := x_gate; v_sub := x_subr; v_shift := x_shift; v_last := '1';
                        if e_acc(36 downto 5) = 0 then v_nz := '0'; else v_nz := '1'; end if;
                        if v_gate = '0' then v_bx := (others => '0');
                        elsif v_sub = '1' then v_bx := not e_b;
                        else v_bx := e_b; end if;
                        v_t := (e_acc & '1') + (v_bx & (v_sub and v_gate));
                        v_sum := v_t(37 downto 1);
                        if x_mul = '1' or x_shr = '1' then
                            if x_cz = '0' then v_last := '0'; end if;
                            e_cnt <= e_cnt - 1;
                            if e_cnt = 1 then x_cz <= '1'; x_subr <= x_mul; end if;   -- the next step is the last
                            if x_mul = '1' then x_gate <= v_sum(1); end if;           -- the next multiplier bit
                        elsif x_nrm = '1' and v_nz = '1' then          -- normalise: shift right until < 32
                            v_shift := '1'; v_last := '0';
                            e_cnt <= e_cnt + 1;
                        end if;
                        if x_halt = '1' then e_run <= '0'; end if;
                        if v_shift = '1' then e_acc <= v_sum(36) & v_sum(36 downto 1);
                        else e_acc <= v_sum; end if;
                        if v_last = '1' then
                            v_fetch := '1';
                            e_ph <= "00";
                        end if;
                end case;
            end if;
            if v_fetch = '1' then
                e_ir <= C_KO_UC(to_integer(e_pc));
            end if;

            -- the engine's output registers (per-field constants)
            if e_out = '1' then
                case to_integer(unsigned(e_irf(4 downto 0))) is
                    when 0  => ft_adr <= unsigned(e_acc(8 downto 0));
                    when 1  => c_nk(1) <= e_acc(17 downto 0);
                    when 2  => c_nk(2) <= e_acc(17 downto 0);
                    when 3  => c_nk(3) <= e_acc(17 downto 0);
                    when 4  => c_nk(4) <= e_acc(17 downto 0);
                    when 5  => c_nk(5) <= e_acc(17 downto 0);
                    when 6  => c_nk(6) <= e_acc(17 downto 0);
                    when 7  => c_nk(7) <= e_acc(17 downto 0);
                    when 8  => c_md   <= std_logic_vector(e_acc(13 downto 0));
                    when 9  => c_nh   <= e_acc(17 downto 0);
                    when 10 => c_p0   <= e_acc(17 downto 0);
                    when 11 => c_dp   <= unsigned(e_acc(15 downto 0));
                    when 12 => c_s0   <= e_acc(17 downto 0);
                    when 13 => c_ds   <= unsigned(e_acc(16 downto 0));
                    when 14 => c_h0   <= unsigned(e_acc(16 downto 0));
                    when 15 => c_doff <= unsigned(e_acc(13 downto 0));
                    when 16 => c_kb   <= unsigned(e_acc(2 downto 0));
                    when 17 => c_mu   <= unsigned(e_acc(4 downto 0));
                    when 18 => c_rw   <= unsigned(e_acc(11 downto 0));
                    when 19 => c_hw   <= unsigned(e_acc(9 downto 0));
                    when 20 => c_vx0  <= unsigned(e_acc(11 downto 0));
                    when 21 => c_g6   <= unsigned(e_acc(12 downto 0));
                    when 22 => c_hz   <= unsigned(e_acc(16 downto 0));
                    when 23 => c_hf   <= unsigned(e_acc(16 downto 0));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_eng;

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
    -- [D] SOURCE FIELD (riley's, with dazzle's Osc fix): luma (or the Osc lattice) ->
    -- gain table (K6) -> box of 32 px -> a vertical
    -- one-pole per 32-px block column (64 x 16 store) -> the star size (0 .. 15),
    -- written into the camera grid's write bank at every block end.  The grid is
    -- read by the next field's star screen (the banks swap at the zone start).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(15 downto 0);
        variable v_x7 : unsigned(6 downto 0);
        variable v_n  : signed(17 downto 0);
        variable v_ev : std_logic;
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then   -- both phases restart from this field's drift
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);
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
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_tv <= gt_q;
            if s_v2 = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 (64) samples; the samples before the line start count as the first one
            if s_v2 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            elsif s_b64 = '1' then bx_wp <= (others => '0'); bx_rp <= to_unsigned(194, 8);
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(226, 8); end if;
            bx_f <= bx_q;
            if s_bcnt(6) = '0' and (s_b64 = '1' or s_bcnt(5) = '0') then v_tap := s_sfirst; else v_tap := bx_f; end if;
            s_df <= signed(resize(s_tv, 17)) - signed(resize(v_tap, 17));
            s_tvd <= s_tv; s_v3 <= s_v2;
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_sfirst <= s_tv;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_v2 = '1' then
                if s_bcnt(6) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_ld = '1' then
                if s_b64 = '1' then s_bsum <= shift_left(resize(s_tvd, 22), 6);
                else s_bsum <= shift_left(resize(s_tvd, 22), 5); end if;
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

            -- the box average of the pixel in the "v5" slot
            if s_b64 = '1' then s_eb <= s_bsum(21 downto 6); else s_eb <= s_bsum(20 downto 5); end if;
            s_v5 <= s_v4; s_v5d <= s_v5;

            -- the block columns of the source raster (half virtual px, 64 per block)
            v_ev := '0';
            if s_v5 = '1' then
                v_x7 := resize(sx_l, 7) + resize(s_xs, 7);
                sx_l <= v_x7(5 downto 0);
                if v_x7(6) = '1' then sx_col <= sx_col + 1; end if;
                sx_cy <= v_x7(6);
                sx_pcol <= sx_col;
                v_ev := sx_cy;                       -- this pixel is the first of a new block
            else
                sx_l <= c_vx0(5 downto 0); sx_col <= c_vx0(11 downto 6);
                sx_cy <= '0';
            end if;
            if s_v5 = '0' and s_v5d = '1' then sx_pend <= '1'; end if;      -- the line's last block
            -- the block rows (Q8 virtual lines, 32 lines per block): stepped at every line start but the first
            if s_zone = '1' then
                sy_l <= "000" & s_ys(11 downto 1);
                sy_base <= (others => '0'); sy_seen <= '0';
            elsif s_v5 = '1' and s_v5d = '0' then
                if sy_seen = '1' then
                    sy_l <= ('0' & sy_l(12 downto 0)) + resize(s_ys, 14);
                end if;
                sy_seen <= '1';
            elsif sy_l(13) = '1' then
                sy_l(13) <= '0';
                sy_base <= sy_base + 60;
            end if;

            -- the block store: q += (box - q) >> sh (the first line loads it), then the grid write
            cs_we <= '0'; cm_we <= '0';
            case cs_st is
                when "000" =>
                    if v_ev = '1' then
                        cs_b <= s_eb; cs_c <= sx_pcol; cs_st <= "001"; cs_fl <= s_fl;
                    elsif sx_pend = '1' and s_v5 = '0' then
                        cs_b <= s_eb; cs_c <= sx_pcol; cs_st <= "001"; cs_fl <= s_fl;
                        sx_pend <= '0';
                    end if;
                when "001" => cs_st <= "010";            -- the store's read register
                when "010" => cs_f <= cs_q; cs_st <= "011";
                when "011" =>
                    cs_d <= signed(resize(cs_b, 17)) - signed(resize(cs_f, 17));
                    cs_st <= "100";
                when "100" =>
                    if cs_fl = '1' then v_n := signed(resize(cs_b, 18));
                    else v_n := signed(resize(cs_f, 18)) + resize(shift_right(cs_d, to_integer(s_vsh)), 18); end if;
                    cs_wd <= unsigned(v_n(15 downto 0)); cs_we <= '1';
                    if v_n(15 downto 12) /= "0000" then cm_wd <= "1111"; else cm_wd <= unsigned(v_n(11 downto 8)); end if;
                    cm_wa <= s_wbank & (sy_base + resize(cs_c, 11));
                    cm_we <= '1';
                    cs_st <= "000";
                when others => cs_st <= "000";
            end case;

            -- gain table fill (zone, after the engine): T(i) = max(0, (i - 64) * 2 K6 * 8) >> 9 (Osc: >> 8), by
            -- accumulation: dark areas keep the clean figure, bright ones reach the full star
            gt_we <= '0';
            if e_rund = '1' and e_run = '0' then
                gt_run <= '1'; gt_i <= (others => '0');
                gt_acc <= to_signed(0, 24) - signed(resize(c_g6 & "0000000", 24));
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_i;
                if gt_acc(23) = '1' then gt_wd <= (others => '0');
                elsif s_src = '1' then gt_wd <= resize(unsigned(gt_acc(21 downto 8)), 16);
                else gt_wd <= resize(unsigned(gt_acc(21 downto 9)), 16); end if;
                gt_acc <= gt_acc + signed(resize(c_g6 & '0', 24)); gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [L] LINE ENGINE: s = a1 + a2 of the next line (and H0 - s, s - H0), the star
    -- screen's line within its block row and the row's base address.  Loaded by the
    -- kick after the field engine, stepped at line step 12 (after the beam has
    -- copied this line's values at T + 7).
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            case to_integer(e_lc) is
                when 12 =>
                    if e_kr = '0' then
                        e_s <= e_s + signed(resize(c_ds, 28));
                        e_vy <= ('0' & e_vy(12 downto 0)) + resize(s_ys, 14);
                    end if;
                when 13 =>
                    e_a3 <= signed(resize(c_h0, 19)) - resize(e_s(27 downto 10), 19);
                    e_ns <= e_s(27 downto 10) - signed(resize(c_h0, 18));
                    if e_vy(13) = '1' then
                        e_vy(13) <= '0';
                        e_rb <= e_rb + 60;
                    end if;
                when others => null;
            end case;
            if l_kick = '1' then
                e_s <= shift_left(resize(c_s0, 28), 10);
                e_vy <= "000" & s_ys(11 downto 1);
                e_rb <= (others => '0');
                e_lc <= (others => '0'); e_kr <= '1';
            elsif g_start = '1' then
                e_lc <= (others => '0'); e_kr <= '0';
            elsif e_lc /= 31 then
                e_lc <= e_lc + 1;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [K] THE CASCADE.  The beam: p = a1 - a2 along the line (the same start on
    -- every line), s = a1 + a2 per line.  Pixel 0 at T + 8.
    --   F1 .. F4  |p|; e, f = the smaller / larger of a1, a2; the nearest side's frame
    --             (U, W) = (a3 < e ? H0 - e : s, H0 - f), Y0 = -min(a3, e)   (Dive: never a3)
    --   level j   A: hi, lo = sort(U, W)    B: t = hi - K_j; on and t < 0: (U, W) = (~t, lo + t + 1)
    --             (the sum U + W stays exact); the growing level's min(t, 0) rides along as g
    --   E1, E2    Y = HF - U - W + max(0, -h - g): the fold distance, the growing level truncated at h
    -- (U, W) after level j are valid at T + 12 + 2 j, Y at T + 28.  Z (the echo field:
    -- HZ - U - W after level 1, the hexagram; Dive: Y0) is delayed in a block-RAM ring to T + 28.
    ---------------------------------------------------------------------------
    p_casc : process(clk)
        variable v_p  : signed(17 downto 0);
        variable v20, w20 : signed(19 downto 0);
        variable v19 : signed(18 downto 0);
        variable v_t, v_g, v_r : signed(17 downto 0);
        variable v_w  : signed(18 downto 0);
        variable v_neg : std_logic;
    begin
        if rising_edge(clk) then
            -- the beam
            if s_st(6) = '1' then
                gb_p <= shift_left(resize(c_p0, 28), 10);
                gb_s <= e_s(27 downto 10); gb_a3 <= e_a3; gb_ns <= e_ns;
                gb_vy <= e_vy(12 downto 5); gb_rb <= e_rb;
            else
                gb_p <= gb_p + signed(resize(c_dp, 28));
            end if;
            -- F1: |p| (ones' complement: one LSB is 1/16 pixel or less)
            v_p := gb_p(27 downto 10);
            if v_p(17) = '1' then f1_ap <= not v_p; else f1_ap <= v_p; end if;
            -- F2: e = floor((s - |p|) / 2), f = floor((s + |p| + 1) / 2)
            v20 := resize(gb_s, 20) - resize(f1_ap, 20);
            f2_e <= v20(19 downto 1);
            w20 := resize(gb_s, 20) + resize(f1_ap, 20) + 1;
            f2_f <= w20(19 downto 1);
            -- F3: which side is nearest; H0 - e, H0 - f, -e
            if gb_a3 < f2_e and s_dive = '0' then f3_sel <= '1'; else f3_sel <= '0'; end if;
            v19 := signed(resize(c_h0, 19)) - f2_e;
            f3_he <= v19(17 downto 0);
            v19 := signed(resize(c_h0, 19)) - f2_f;
            f3_wf <= v19(17 downto 0);
            f3_ne <= not f2_e(17 downto 0);
            -- F4: the frame and Y0
            if f3_sel = '1' then lv_U(0) <= f3_he; f4_y <= gb_ns;
            else lv_U(0) <= gb_s; f4_y <= f3_ne; end if;
            lv_W(0) <= f3_wf;
            -- the levels
            for j in 1 to 7 loop
                if lv_U(j - 1) >= lv_W(j - 1) then
                    lv_hi(j) <= lv_U(j - 1); lv_lo(j) <= lv_W(j - 1);
                else
                    lv_hi(j) <= lv_W(j - 1); lv_lo(j) <= lv_U(j - 1);
                end if;
                lv_ga(j) <= lv_g(j - 1);
                v_t := lv_hi(j) + c_nk(j);
                v_neg := v_t(17) and (c_md(2 * j - 1) or c_md(2 * j - 2));
                if v_neg = '1' then lv_U(j) <= not v_t; v_g := v_t;
                else lv_U(j) <= v_t; v_g := (others => '0'); end if;
                v_w := (lv_lo(j) & '1') + (v_g & v_neg);                 -- lo + t + 1, or lo
                lv_W(j) <= v_w(18 downto 1);
                if c_md(2 * j - 1) = '1' then lv_g(j) <= v_g; else lv_g(j) <= lv_ga(j); end if;
            end loop;
            -- E1 (T + 27), E2 (T + 28): the fold distance
            e1_sum <= lv_U(7) + lv_W(7);
            e1_r <= c_nh - lv_g(7);
            if e1_r(17) = '0' then v_r := e1_r; else v_r := (others => '0'); end if;
            e2_y <= (signed(resize(c_hf, 18)) - e1_sum) + v_r;
            -- Z: after level 1 (T + 16), or Y0 (Dive, T + 12), into the ring; out at T + 28
            z1_sum <= lv_U(1) + lv_W(1);
            z1_y <= signed(resize(c_hz, 18)) - z1_sum;
            zr_wa <= zr_wa + 1;
            if s_dive = '1' then
                zr_wd <= '0' & f4_y(17) & std_logic_vector(f4_y(13 downto 0));
                zr_ra <= zr_wa - 12;
            else
                zr_wd <= '0' & z1_y(17) & std_logic_vector(z1_y(13 downto 0));
                zr_ra <= zr_wa - 8;
            end if;
            zr_f <= zr_q;
        end if;
    end process p_casc;

    ---------------------------------------------------------------------------
    -- [E] BANDS AND RIM.  A = -Y inside (Y < -12 LSBs, about a pixel), Z outside;
    -- phase = A - stream; a band is 2^k LSBs (k = kb + 7): the band number is bits
    -- k + 1 .. k, the distance to the nearest band edge the folded bits below.
    -- Bold: tones by band, anti-aliased against the neighbouring band.  Fine: lines
    -- of half width hw on every band edge.  The rim: the white band rw wide just
    -- inside the curve (Ink Bold: two pixels of the inside's first tone), composited
    -- over the bands, so the curve itself is anti-aliased against the outside tone.
    -- Coverage = clamp(8 + d mu / 16) sixteenths, d in LSBs, mu / 256 = the LSB in pixels.
    ---------------------------------------------------------------------------
    p_band : process(clk)
        variable v_y   : signed(17 downto 0);
        variable v_ny : signed(17 downto 0);
        variable v_b, v_nb : unsigned(1 downto 0);
        variable v_half : std_logic;
        variable v_x   : unsigned(13 downto 0);
        variable v_d   : unsigned(10 downto 0);
        variable v_f   : signed(11 downto 0);
        variable v_ti, v_to, v_ni, v_no : unsigned(9 downto 0);
        variable v_c   : signed(12 downto 0);
        variable v_n14 : unsigned(13 downto 0);
        variable v_b0  : std_logic;
    begin
        if rising_edge(clk) then
            -- S1 (T + 29): inside / outside, the band argument, the rim's inner distance.  The echo field is
            -- the ring's Z, or Y itself while level 1 is not whole (and never in front of the curve)
            v_y := e2_y;
            if v_y < -12 then
                s1_ins <= '1'; s1_a <= unsigned(not v_y(13 downto 0));
            else
                s1_ins <= '0';
                if s_dive = '1' or c_md(1 downto 0) = "01" then
                    if zr_f(14) = '1' then s1_a <= (others => '0');
                    else s1_a <= unsigned(zr_f(13 downto 0)); end if;
                elsif v_y(17) = '1' then s1_a <= (others => '0');
                else s1_a <= unsigned(v_y(13 downto 0)); end if;
            end if;
            s1_r1 <= v_y + signed(resize(c_rw, 18));
            s1_y <= v_y;
            -- S2 (T + 30): the phase; the rim distance min(-Y, rw + Y) (saturated at S3)
            s2_phi <= s1_a - c_doff;
            v_ny := not s1_y;
            if v_ny < s1_r1 then s2_mn <= v_ny; else s2_mn <= s1_r1; end if;
            s2_ins <= s1_ins;
            -- S3 (T + 31): band number, half, distance to the nearer band edge (LSBs)
            case to_integer(c_kb) is
                when 0      => v_b := s2_phi(8 downto 7);   v_half := s2_phi(6);
                when 1      => v_b := s2_phi(9 downto 8);   v_half := s2_phi(7);
                when 2      => v_b := s2_phi(10 downto 9);  v_half := s2_phi(8);
                when 3      => v_b := s2_phi(11 downto 10); v_half := s2_phi(9);
                when 4      => v_b := s2_phi(12 downto 11); v_half := s2_phi(10);
                when others => v_b := s2_phi(13 downto 12); v_half := s2_phi(11);
            end case;
            if v_half = '1' then v_x := not s2_phi; else v_x := s2_phi; end if;
            v_d := v_x(10 downto 0);
            case to_integer(c_kb) is
                when 0      => v_d(10 downto 6) := (others => '0');
                when 1      => v_d(10 downto 7) := (others => '0');
                when 2      => v_d(10 downto 8) := (others => '0');
                when 3      => v_d(10 downto 9) := (others => '0');
                when 4      => v_d(10) := '0';
                when others => null;
            end case;
            s3_b <= v_b; s3_half <= v_half; s3_d <= v_d;
            if s2_mn > 63 then s3_dr <= to_signed(63, 7);
            elsif s2_mn < -64 then s3_dr <= to_signed(-64, 7);
            else s3_dr <= s2_mn(6 downto 0); end if;
            s3_ins <= s2_ins;
            -- S4 (T + 32): the signed edge distance (Bold: into the band; Fine: into the line), the tones
            if s_fine = '0' then
                if s3_d > 63 then s4_ds <= to_signed(63, 7);
                else s4_ds <= signed('0' & s3_d(5 downto 0)); end if;
            else
                v_f := signed(resize(c_hw, 12)) - signed(resize(s3_d, 12));
                if v_f > 63 then s4_ds <= to_signed(63, 7);
                elsif v_f < -64 then s4_ds <= to_signed(-64, 7);
                else s4_ds <= v_f(6 downto 0); end if;
            end if;
            if s3_half = '1' then v_nb := s3_b + 1; else v_nb := s3_b - 1; end if;
            if s_ink = '1' then
                if s3_b(0) = '1' then v_ti := to_unsigned(64, 10); v_to := to_unsigned(940, 10);
                else v_ti := to_unsigned(940, 10); v_to := to_unsigned(64, 10); end if;
                if v_nb(0) = '1' then v_ni := to_unsigned(64, 10); v_no := to_unsigned(940, 10);
                else v_ni := to_unsigned(940, 10); v_no := to_unsigned(64, 10); end if;
            else
                v_ti := f_g4(not s3_b); v_to := f_g4('0' & s3_b(0));
                v_ni := f_g4(not v_nb); v_no := f_g4('0' & v_nb(0));
            end if;
            if s_fine = '1' then
                if s_ink = '1' then s4_own <= to_unsigned(940, 10);
                elsif s3_ins = '1' then s4_own <= to_unsigned(648, 10);
                else s4_own <= to_unsigned(356, 10); end if;
                s4_far <= to_unsigned(64, 10);
            elsif s3_ins = '1' then
                s4_own <= v_ti; s4_far <= v_ni;
            else
                s4_own <= v_to; s4_far <= v_no;
            end if;
            s4_dr <= s3_dr;
            -- S5 (T + 33): d * mu
            s5_pb <= s4_ds * signed('0' & c_mu);
            s5_pr <= s4_dr * signed('0' & c_mu);
            s5_own <= s4_own; s5_far <= s4_far;
            -- S6 (T + 34): the coverages (sixteenths)
            v_c := shift_right(s5_pb, 4) + 8;
            if v_c < 0 then s6_cb <= (others => '0');
            elsif v_c > 16 then s6_cb <= to_unsigned(16, 5);
            else s6_cb <= unsigned(v_c(4 downto 0)); end if;
            v_c := shift_right(s5_pr, 4) + 8;
            cr_p(19 downto 5) <= cr_p(14 downto 0);
            if v_c < 0 then cr_p(4 downto 0) <= (others => '0');
            elsif v_c > 16 then cr_p(4 downto 0) <= to_unsigned(16, 5);
            else cr_p(4 downto 0) <= unsigned(v_c(4 downto 0)); end if;
            s6_own <= s5_own; s6_far <= s5_far;
            -- the rim's tone (per field): white; Ink Bold: the inside's tone at the curve
            v_n14 := to_unsigned(0, 14) - c_doff;
            case to_integer(c_kb) is
                when 0      => v_b0 := v_n14(7);
                when 1      => v_b0 := v_n14(8);
                when 2      => v_b0 := v_n14(9);
                when 3      => v_b0 := v_n14(10);
                when 4      => v_b0 := v_n14(11);
                when others => v_b0 := v_n14(12);
            end case;
            if s_ink = '1' and s_fine = '0' and v_b0 = '1' then c_rt <= to_unsigned(64, 10);
            else c_rt <= to_unsigned(940, 10); end if;
        end if;
    end process p_band;

    ---------------------------------------------------------------------------
    -- [E] THE STAR SCREEN (the camera): one little hexagram flake per 32-px block of
    -- the virtual canvas, its size R (0 .. 15 virtual px, held for the whole block)
    -- read from last field's camera grid.  Heights from the block centre (Q3):
    -- p1 = y, p2 = (sqrt(3) x - y) / 2, p3 = -(sqrt(3) x + y) / 2; the hexagram is
    -- min(max p, -min p) < R / 2.  Block column counters hold pixel 0 at T + 32.
    ---------------------------------------------------------------------------
    p_star : process(clk)
        variable v_x7 : unsigned(6 downto 0);
        variable v_v  : signed(8 downto 0);
        variable v_ly, v_lh : signed(9 downto 0);
        variable v_nm : signed(9 downto 0);
        variable v_ds : signed(9 downto 0);
        variable v_sd : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- v0: the block column and the position in it (half virtual px)
            if s_st(30) = '1' then
                v0_lx <= c_vx0(5 downto 0); v0_col <= c_vx0(11 downto 6);
            else
                v_x7 := resize(v0_lx, 7) + resize(s_xs, 7);
                v0_lx <= v_x7(5 downto 0);
                if v_x7(6) = '1' then v0_col <= v0_col + 1; end if;
            end if;
            -- v1: the grid address (the read bank); sqrt(3)/2 x in Q3 = 3.5 (lx - 32)
            cm_ra <= (not s_wbank) & (gb_rb + resize(v0_col, 11));
            v_v := resize(signed((not v0_lx(5)) & v0_lx(4 downto 0)), 9);
            v1_qx <= shift_left(v_v, 2) - shift_right(v_v, 1);
            -- v2: p2, p3 (p1 = the line's y)
            v_ly := resize(signed((not gb_vy(7)) & gb_vy(6 downto 0)), 10);
            v_lh := shift_right(v_ly, 1);
            v2_p2 <= resize(v1_qx, 10) - v_lh;
            v2_p3 <= (to_signed(0, 10) - resize(v1_qx, 10)) - v_lh;
            -- v3, v4: max and min of the three; the size from the grid (fabric register, then held)
            cm_f <= cm_q;
            if v2_p2 > v_ly then v3_mx <= v2_p2; v3_mn <= v_ly; else v3_mx <= v_ly; v3_mn <= v2_p2; end if;
            v3_p3 <= v2_p3;
            if v3_p3 > v3_mx then v4_mx <= v3_p3; else v4_mx <= v3_mx; end if;
            if v3_p3 < v3_mn then v4_mn <= v3_p3; else v4_mn <= v3_mn; end if;
            v4_r <= cm_f;
            -- v5: the hexagram's height function
            v_nm := to_signed(0, 10) - v4_mn;
            if v4_mx < v_nm then v5_hd <= v4_mx; else v5_hd <= v_nm; end if;
            v5_r <= v4_r;
            -- v6: signed distance into the star (Fine: into its 2.5-px outline)
            v_ds := v5_hd - signed(resize(v5_r & "00", 10));
            if s_fine = '1' then v6_sd <= to_signed(10, 10) - abs(v_ds);
            else v6_sd <= to_signed(0, 10) - v_ds; end if;
            v6_r <= v5_r;
            -- v7, v8: coverage in sixteenths (a pixel is 1 / 1.5 / 2 virtual px)
            v_sd := resize(v6_sd, 12);
            case to_integer(s_xs) is
                when 2      => v7_c <= shift_left(v_sd, 1) + 8;
                when 3      => v7_c <= v_sd + shift_right(v_sd, 2) + shift_right(v_sd, 3) + 8;
                when others => v7_c <= v_sd + 8;
            end case;
            v7_r <= v6_r;
            if v7_r < 3 or v7_c < 0 then v8_cs <= (others => '0');
            elsif v7_c > 16 then v8_cs <= to_unsigned(16, 5);
            else v8_cs <= unsigned(v7_c(4 downto 0)); end if;
        end if;
    end process p_star;

    ---------------------------------------------------------------------------
    -- [E] THE THREE BLENDS, y = base + ((target - base) cov + 8) / 16, three clocks each:
    --   1  the band (far -> own)                T + 35 .. 37
    --   2  the rim (-> the rim tone)            T + 38 .. 40
    --   3  the stars (Grey: -> white; Ink: -> the inverse, 1004 - y)  T + 41 .. 43
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_g : signed(15 downto 0);
        variable v_s : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            bl_df(1) <= signed(resize(s6_own, 11)) - signed(resize(s6_far, 11));
            bl_cv(1) <= s6_cb; bl_b1(1) <= s6_far;
            bl_df(2) <= signed(resize(c_rt, 11)) - signed(resize(bl_y(1), 11));
            bl_cv(2) <= cr_p(19 downto 15); bl_b1(2) <= bl_y(1);
            if s_ink = '1' then bl_df(3) <= to_signed(1004, 11) - signed(resize(bl_y(2) & '0', 11));     -- Ink: XOR
            else bl_df(3) <= to_signed(940, 11) - signed(resize(bl_y(2), 11)); end if;                  -- Grey: a white film
            bl_cv(3) <= v8_cs; bl_b1(3) <= bl_y(2);
            for k in 1 to 3 loop
                if bl_cv(k)(4) = '1' then
                    bl_pa(k) <= shift_left(resize(bl_df(k), 16), 4); bl_pb(k) <= (others => '0');
                else
                    if bl_cv(k)(3) = '1' then v_g := shift_left(resize(bl_df(k), 16), 3); else v_g := (others => '0'); end if;
                    if bl_cv(k)(2) = '1' then bl_pa(k) <= v_g + shift_left(resize(bl_df(k), 16), 2); else bl_pa(k) <= v_g; end if;
                    if bl_cv(k)(1) = '1' then v_g := shift_left(resize(bl_df(k), 16), 1); else v_g := (others => '0'); end if;
                    if bl_cv(k)(0) = '1' then bl_pb(k) <= v_g + resize(bl_df(k), 16); else bl_pb(k) <= v_g; end if;
                end if;
                bl_b2(k) <= bl_b1(k);
                v_s := signed(resize(bl_b2(k), 16)) + shift_right(bl_pa(k) + bl_pb(k) + 8, 4);
                bl_y(k) <= unsigned(v_s(9 downto 0));
            end loop;
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: luma [1 2 1]/4, luma 13/16 toward black, blanking
    -- gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd1 <= bl_y(3); s_yd2 <= s_yd1;
            s_ya <= resize(bl_y(3), 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y45 <= v_y(11 downto 2);
            if s_y45 > 64 then v_dl := s_y45 - 64; else v_dl := (others => '0'); end if;
            s_y46 <= s_y45 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if dl_q(2) = '1' then
                s_y47 <= s_y46;
            else
                s_y47 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y47);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= dl_q2(0);
    data_out.vsync_n <= dl_q2(1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= dl_q2(2);

end architecture koch;
