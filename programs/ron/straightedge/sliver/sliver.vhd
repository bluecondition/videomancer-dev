-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Sliver v0.1.0 -- Straightedge.  Carmen Herrera's hard-edged wedges, El Lissitzky's Prouns, searchlight
-- beams crossing: a few long pointed triangles, each a translucent film of light, crossing the whole frame.
-- Black, white and greys only (U = V = 512).
--
--   sliver w (w < Count): apex  = G + Spread * dist_w * (cos A_w, sin A_w),   A_w = psi_w + theta_w
--                         axis  = A_w + dphi_w (pointing back across the frame, past the gathering point G)
--                         edges = the two lines through the apex at axis +- half-angle:
--                                 e = cos(th) (x - apex.x) + sin(th) (y - apex.y),  th = axis +- (alpha - 90 deg)
--   coverage  = clamp(c1 + c2 - 1), c = clamp(e + 1/2): exact for the slab between the two edges, so a needle
--               thins to nothing at its point.  Bowtie = |c1 + c2 - 1| (the wedge and its mirror).
--               Fine = the same with the edges moved out and in by half the line width, subtracted.
--   tone      = Grey: every sliver adds a third of white (clipped), Ink: the slivers XOR (a triangle of the
--               summed coverage)
--   motion    = theta_w, one 24-bit accumulator per sliver (in the edge table's block RAM), advanced once per
--               field by rate_w * speed: every sliver wheels about the gathering point at its own rate
--   camera    = the box-filtered luma sampled once per 32 x 32 virtual-px block into a two-bank block-RAM
--               grid, read back the next field and held across the block: each block is cut by one diagonal
--               into two triangles, and the luma lays 0, 1 or 2 flat films on them (the lower triangle a
--               quarter step ahead of the upper, so tone regions meet in saw-tooth borders)
--
--   K1  X        the gathering point, +-1024 virtual px                               (glided)
--   K2  Y        +-576 virtual px                                                     (glided)
--   K3  Scale    the half-angle of every sliver, 2.8 .. 28 degrees, exponential       (glided)
--   K4  Count    2 .. 6 slivers (stepped, hysteresis)
--   K5  Drift    bipolar speed: +-24 counts still, else 2^-12 .. 2^-6 turn per field  (speed glided)
--   K6  Camera   how far the camera's luma reaches up the film steps                  (glided)
--   S7  Glide    Off = the knob value of this field, On = 1/16 of the gap per field (Drift 1/8)
--   S8  Form     Wedge / Bowtie
--   S9  Tone     Grey / Ink
--   S10 Weight   Bold (filled) / Fine (8 virtual px outlines)
--   S11 Source   Video / Osc
--   P12 SPREAD   0 = every apex on the gathering point (a pinwheel), 100 % = each apex pushed out along its
--                own direction by its own distance (0.64 .. 0.9 screen heights)   (glided)
--
-- Sections (the integer model .straightedge_work/rb_sliver/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide, Count hysteresis, switches
--   [C] FIELD SEQUENCER                  one serial multiplier, slots of 24 clocks: 6 global slots, then per
--                                        sliver 6 apex slots and 7 slots for each of its two edges
--   [D] SOURCE FIELD                     Osc lattice, box, vertical one-pole in a line store, block grid
--   [E] EDGES                            12 affine DDAs (pixel units, Q14), per-sliver coverage, the mosaic
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, luma 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  The DDAs hold DDA pixel i at T + 6 + i; a register "at stage S" holds
-- pixel i at T + 6 + i + S; the luma is stage 9, the output register stage 14.  Output column j is the
-- [1 2 1] filter centred on DDA pixel j + 1.  One latency (20) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.sliver_pkg.all;
use work.all;

architecture sliver of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
    constant C_DEF_K5  : integer := 650;
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
    -- luma of a coverage 0 .. 16: 64 + (o * 876 + 8) / 16
    function f_luma(o : unsigned(4 downto 0)) return unsigned is
    begin
        case to_integer(o) is
            when 0      => return to_unsigned(64, 10);
            when 1      => return to_unsigned(119, 10);
            when 2      => return to_unsigned(174, 10);
            when 3      => return to_unsigned(228, 10);
            when 4      => return to_unsigned(283, 10);
            when 5      => return to_unsigned(338, 10);
            when 6      => return to_unsigned(393, 10);
            when 7      => return to_unsigned(447, 10);
            when 8      => return to_unsigned(502, 10);
            when 9      => return to_unsigned(557, 10);
            when 10     => return to_unsigned(612, 10);
            when 11     => return to_unsigned(666, 10);
            when 12     => return to_unsigned(721, 10);
            when 13     => return to_unsigned(776, 10);
            when 14     => return to_unsigned(831, 10);
            when 15     => return to_unsigned(885, 10);
            when others => return to_unsigned(940, 10);
        end case;
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

    -- [B] knobs: +-1 hold (10 bits), glide (Q3)
    signal h_x : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal h_y : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal h_p : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal h_t : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal h_c : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal h_s : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal g_x : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal g_y : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal g_p : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal g_t : unsigned(12 downto 0) := to_unsigned(C_DEF_K5 * 8, 13);
    signal g_c : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);
    signal g_s : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(13 downto 0) := (others => '0');
    signal s_gr, s_gh : unsigned(9 downto 0) := (others => '0');
    signal z_gp, z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_bow, s_ink, s_fine, s_src : std_logic := '0';
    signal s_zn  : unsigned(2 downto 0) := to_unsigned(2, 3);          -- Count - 2
    signal z_v5  : unsigned(12 downto 0) := to_unsigned(2560, 13);
    signal z_up, z_dn : std_logic := '0';
    signal s_act : std_logic_vector(5 downto 0) := "001111";

    -- [C] ROMs
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q, sn_f : unsigned(15 downto 0) := (others => '0');

    -- [C] sequencer and its serial multiplier
    signal q_run, q_go, q_cut, q_gdone : std_logic := '0';
    signal q_sec : unsigned(1 downto 0) := (others => '0');
    signal q_s, q_w : unsigned(2 downto 0) := (others => '0');
    signal q_ph : unsigned(4 downto 0) := (others => '0');
    signal f_a, f_acc : signed(39 downto 0) := (others => '0');
    signal z_fa : signed(23 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_oct  : unsigned(9 downto 0) := (others => '0');
    signal z_ex   : unsigned(10 downto 0) := (others => '0');
    signal z_sc   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_rq   : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal z_m    : unsigned(11 downto 0) := to_unsigned(1928, 12);
    signal z_w48  : unsigned(13 downto 0) := to_unsigned(7688, 14);
    signal z_px, z_py, z_ax : signed(16 downto 0) := (others => '0');   -- gathering point, apex: pixel widths, Q3
    signal z_apx, z_apy : unsigned(15 downto 0) := (others => '0');     -- |apex|
    signal c_pxn, c_pyn : std_logic := '0';                             -- its signs
    signal z_R    : unsigned(12 downto 0) := (others => '0');
    signal z_av   : unsigned(12 downto 0) := (others => '0');
    signal z_vn, z_vz, z_rn : std_logic := '0';
    signal z_spd  : unsigned(17 downto 0) := (others => '0');
    signal z_ra   : unsigned(4 downto 0) := (others => '0');
    signal z_m2   : unsigned(22 downto 0) := (others => '0');
    signal z_phi  : unsigned(23 downto 0) := (others => '0');
    signal z_A, z_th : unsigned(15 downto 0) := (others => '0');
    signal c_al, z_alw : unsigned(13 downto 0) := (others => '0');
    signal z_p    : unsigned(14 downto 0) := (others => '0');
    signal z_ov, z_ov1, z_sneg, z_cneg : std_logic := '0';
    signal z_fs, z_fc : unsigned(3 downto 0) := (others => '0');
    signal z_sm, z_cm : unsigned(14 downto 0) := (others => '0');
    signal z_sdf, z_cdf : unsigned(5 downto 0) := (others => '0');
    signal z_dy, z_seed : signed(27 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_sd   : unsigned(12 downto 0) := (others => '0');      -- Spread, scaled to the raster
    signal c_G    : unsigned(11 downto 0) := (others => '0');      -- the camera table's step
    signal c_hw2  : unsigned(7 downto 0) := (others => '0');       -- Fine: the line width in coverage units
    signal c_bias : unsigned(17 downto 0) := to_unsigned(8192, 18);
    signal c_x0   : unsigned(11 downto 0) := (others => '0');
    signal c_msh  : unsigned(1 downto 0) := "01";                  -- mosaic AA: 0 = /2, 1 = x1, 2 = x2
    signal c_vsh  : unsigned(1 downto 0) := "00";                  -- one-pole shift: 4 - c_vsh

    -- [E] edge table: (edge, 0) = the pixel step, (edge, 1) = the line wrap, (edge, 2) = the field seed;
    -- words 48 .. 53 = the slivers' phase accumulators
    type t_cr is array (0 to 255) of signed(31 downto 0);
    signal cr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_ra, cr_wa : unsigned(5 downto 0) := (others => '0');
    signal cr_q, cr_wd : signed(31 downto 0) := (others => '0');
    signal cr_we : std_logic := '0';

    -- [E] edge DDAs and their loader
    type t_s28x12 is array (0 to 11) of signed(27 downto 0);
    signal ca_inc, ca_acc : t_s28x12 := (others => (others => '0'));
    signal ca_ld, ca_en : std_logic_vector(11 downto 0) := (others => '0');
    signal ca_rst : std_logic := '0';
    signal l_kick, l_run, l_seed : std_logic := '0';
    signal l_t : unsigned(4 downto 0) := (others => '0');
    signal l_v1, l_v2, l_o1, l_o2 : std_logic := '0';
    signal l_c1, l_c2 : unsigned(3 downto 0) := (others => '0');
    signal b_run, b_end, b_e1, b_e2, b_e3 : std_logic := '0';
    signal b_cnt, b_mm1 : unsigned(11 downto 0) := (others => '0');

    -- [E] coverage
    type t_u8x12 is array (0 to 11) of unsigned(7 downto 0);
    signal x1_es : t_u8x12 := (others => (others => '0'));
    type t_u5x12 is array (0 to 11) of unsigned(4 downto 0);
    signal x2_p, x2_m : t_u5x12 := (others => (others => '0'));
    type t_u6x6 is array (0 to 5) of unsigned(5 downto 0);
    signal x3_o, x3_i : t_u6x6 := (others => (others => '0'));
    type t_u5x6 is array (0 to 5) of unsigned(4 downto 0);
    signal x4_c : t_u5x6 := (others => (others => '0'));
    signal x5_a, x5_b, x5_c : unsigned(5 downto 0) := (others => '0');
    signal x6_sum : unsigned(6 downto 0) := (others => '0');
    signal x7_tot : unsigned(7 downto 0) := (others => '0');
    signal x8_v   : unsigned(4 downto 0) := (others => '0');
    signal x8_tc  : unsigned(5 downto 0) := (others => '0');

    -- [E] the mosaic (camera)
    signal m_yv   : unsigned(18 downto 0) := (others => '0');      -- this line's virtual y, Q8
    signal m_rb0  : std_logic := '0';
    signal m_base : unsigned(10 downto 0) := (others => '0');      -- 60 * block row
    signal m_xr   : unsigned(11 downto 0) := (others => '0');      -- the DDA pixel's virtual x, h
    signal m_bank : std_logic := '0';
    type t_grid is array (0 to 4095) of unsigned(7 downto 0);
    signal gr_ram : t_grid := (others => (others => '0'));
    attribute no_rw_check of gr_ram : signal is true;
    signal gr_ra, gr_wa : unsigned(11 downto 0) := (others => '0');
    signal gr_q, gr_f, gr_wd : unsigned(7 downto 0) := (others => '0');
    signal gr_we : std_logic := '0';
    signal m_xw   : unsigned(11 downto 0) := (others => '0');
    signal x1_d8  : signed(9 downto 0) := (others => '0');
    signal x2_ca, x3_ca, x4_ca, x5_ca : unsigned(4 downto 0) := (others => '0');
    signal x6_mos : unsigned(5 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_t0, s_tv, s_tvd, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_v1, s_v2, s_v2d, s_v3, s_v4, s_ld, s_fl, s_seen, s_efl, s_ffl : std_logic := '0';
    type t_gt is array (0 to 255) of unsigned(15 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt_ram : signal is true;
    signal gt_q, gt_wd : unsigned(15 downto 0) := (others => '0');
    signal gt_f : unsigned(3 downto 0) := (others => '0');
    signal gt_wa, gt_i : unsigned(7 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_acc : unsigned(19 downto 0) := (others => '0');
    signal s_col, s_c2, s_bcol, s_ecol, s_fcol : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_f : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_b32 : std_logic := '1';                                  -- 32-px box on the 1920-wide rasters, else 16
    signal s_df   : signed(8 downto 0) := (others => '0');
    signal s_bsum : unsigned(13 downto 0) := (others => '0');
    signal s_ev, s_fv : std_logic := '0';
    signal s_eb, s_eq, s_fq : unsigned(15 downto 0) := (others => '0');
    signal s_dd : signed(16 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';

    -- [F] output conditioning
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
            r_ex_q <= C_SV_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_SV_SIN(to_integer(sn_a));
            sn_f   <= sn_q;
        end if;
    end process p_rom;

    -- edge table: written by the sequencer in the zone, read by the sequencer (phases) and the loader
    p_crram : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= cr_ram(to_integer(cr_ra));
            if cr_we = '1' then cr_ram(to_integer(cr_wa)) <= cr_wd; end if;
        end if;
    end process p_crram;

    -- camera table (block luma -> film steps): written in the zone, read by the pixel path
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(gr_f));
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

    -- source line store (half horizontal resolution): the one-pole's own read, ahead of its write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_c2(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- the block grid: two banks of 34 x 60 blocks; this field writes one bank and reads the other
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
        variable v_rate : signed(5 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_lo, v_hi : unsigned(13 downto 0);
        variable v_s14 : signed(13 downto 0);
        variable v_a, v_b : unsigned(22 downto 0);
        variable v_t   : signed(27 downto 0);
        variable v_r   : unsigned(15 downto 0);
        variable v_i   : unsigned(18 downto 0);
        variable v_sp  : unsigned(17 downto 0);
        variable v_hw  : unsigned(6 downto 0);
        variable v_last : std_logic;
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
                s_glide <= registers_in(6)(0);
                s_bow   <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_il    <= s_ilr;
                if s_xsr = 2 then s_b32 <= '1'; else s_b32 <= '0'; end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); z_sc <= to_unsigned(16384, 16); c_x0 <= to_unsigned(480, 12);
                    when 3      => s_oxs <= to_unsigned(153, 9); z_sc <= to_unsigned(21845, 16); c_x0 <= to_unsigned(0, 12);
                    when others => s_oxs <= to_unsigned(102, 9); z_sc <= to_unsigned(32768, 16); c_x0 <= to_unsigned(0, 12);
                end case;
                -- Count: 5 steps with hysteresis (enter a step only 8 counts past its edge; the two ends
                -- are reached because the band lies on the entry side)
                if z_up = '1' or z_dn = '1' then s_zn <= z_v5(12 downto 10); end if;
            end if;
            -- (the comparison itself runs free, one step per clock, so no knob carry chain sits on the latch)
            v_k := unsigned(registers_in(3));
            z_v5 <= resize(v_k & "00", 13) + resize(v_k, 13);
            v_lo := resize(s_zn & "0000000000", 14);
            v_hi := v_lo + 1064;
            if resize(z_v5, 14) >= v_hi then z_up <= '1'; else z_up <= '0'; end if;
            if resize(z_v5, 14) + 40 < v_lo then z_dn <= '1'; else z_dn <= '0'; end if;
            g_wm1 <= s_w - 1;
            b_mm1 <= s_w + 7;
            z_m   <= s_w + 8;
            z_w48 <= (s_w & "00") + 8;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            case to_integer(s_xs) is               -- line height in pixel widths, Q10
                when 4      => z_rq <= resize(s_ys & '0', 12);
                when 3      => z_rq <= to_unsigned(1024, 12);
                when others => z_rq <= resize(s_ys & "00", 12);
            end case;
            case to_integer(s_zn) is
                when 0      => s_act <= "000011";
                when 1      => s_act <= "000111";
                when 2      => s_act <= "001111";
                when 3      => s_act <= "011111";
                when others => s_act <= "111111";
            end case;
            -- Fine: half the line width (4 virtual px) in coverage units; the seed bias (1/2 pixel + that)
            if s_fine = '0' then v_hw := (others => '0');
            elsif s_xs = 4 then v_hw := to_unsigned(32, 7);
            elsif s_xs = 3 then v_hw := to_unsigned(43, 7);
            else v_hw := to_unsigned(64, 7); end if;
            if s_il = '1' then
                c_hw2  <= '0' & v_hw(6 downto 1) & '0';
                c_bias <= shift_left(resize(v_hw(6 downto 1), 18) + 8, 11);
            else
                c_hw2  <= v_hw & '0';
                c_bias <= shift_left(resize(v_hw, 18) + 8, 10);
            end if;
            -- mosaic edge scale and the one-pole's rate, by raster
            if s_xs = 2 and s_il = '0' then c_msh <= "10";
            elsif s_xs = 4 and s_il = '1' then c_msh <= "00";
            else c_msh <= "01"; end if;
            if s_xs = 4 then
                if s_il = '1' then c_vsh <= "10"; else c_vsh <= "01"; end if;
            else
                if s_il = '1' then c_vsh <= "01"; else c_vsh <= "00"; end if;
            end if;

            -- [B] shared hold + glide, five clocks per channel.  Hold: the knob moves the held value only by
            -- 2 counts or more (or to either end).  Glide On: 1/16 of the gap per field (the Drift speed 1/8),
            -- never slower than half a count, landing exactly.  Glide Off: the held value, this field.
            q_go <= '0';
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= g_x; s_gh <= h_x; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= g_y; s_gh <= h_y; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= g_p; s_gh <= h_p; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= g_t; s_gh <= h_t; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= g_c; s_gh <= h_c; s_gr <= unsigned(registers_in(5));
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
                            if s_gd > 64 or s_gd < -64 then
                                if z_gc = 3 then s_gd <= shift_right(s_gd, 3); else s_gd <= shift_right(s_gd, 4); end if;
                            elsif s_gd > 4 then s_gd <= to_signed(4, 14);
                            elsif s_gd < -4 then s_gd <= to_signed(-4, 14);
                            end if;
                        end if;
                        z_gp <= "100";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => g_x <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_x <= s_gh;
                            when 1      => g_y <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_y <= s_gh;
                            when 2      => g_p <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_p <= s_gh;
                            when 3      => g_t <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_t <= s_gh;
                            when 4      => g_c <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_c <= s_gh;
                            when others => g_s <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_s <= s_gh;
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; q_go <= '1'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] the sequencer: slots of 24 clocks.  q_cut = '0': the 6 global slots.  Then per sliver q_w:
            -- section 0 = 6 apex slots, sections 1 and 2 = 7 slots for each edge.  Phase 0 pre-muxes the
            -- operands, 1 loads the multiplier, 2 .. 17 iterate, the product is read at 19.  Started once per
            -- field (the zone), so the phase accumulators advance once per field.
            l_kick <= '0'; q_gdone <= '0';
            if (q_sec = 0 and q_s = 5) or (q_sec /= 0 and q_s = 6) then v_last := '1'; else v_last := '0'; end if;
            if q_go = '1' then
                q_run <= '1'; q_cut <= '0'; q_sec <= "00"; q_s <= "000"; q_w <= "000"; q_ph <= (others => '0');
            elsif q_run = '1' then
                if q_ph = 23 then
                    q_ph <= (others => '0');
                    if v_last = '1' then
                        q_s <= "000";
                        if q_cut = '0' then
                            q_cut <= '1'; q_gdone <= '1';
                        elsif q_sec = 2 then
                            q_sec <= "00";
                            if q_w = 5 then q_run <= '0'; l_kick <= '1'; else q_w <= q_w + 1; end if;
                        else
                            q_sec <= q_sec + 1;
                        end if;
                    else
                        q_s <= q_s + 1;
                    end if;
                else
                    q_ph <= q_ph + 1;
                end if;
            end if;

            -- operand pre-mux (phase 0)
            if q_run = '1' and q_ph = 0 then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                if q_cut = '0' then
                    case to_integer(q_s) is
                        when 0      => z_fa <= signed(resize(g_p, 24));    z_fb <= to_unsigned(851, 16);      -- opening, 1/256 octaves
                        when 1      =>                                                                        -- gathering x: 2 virtual px per count
                            v_s14 := signed(resize(g_x, 14)) - 4096;
                            z_fa <= resize(shift_left(resize(v_s14, 16), 1), 24); z_fb <= z_sc;
                        when 2      =>                                                                        -- gathering y: 1.125 virtual px per count
                            v_s14 := signed(resize(g_y, 14)) - 4096;
                            z_fa <= resize(resize(v_s14, 16) + resize(shift_right(v_s14, 3), 16) + 4320, 24); z_fb <= z_sc;
                        when 3      => z_fa <= signed(resize(g_s, 24));    z_fb <= z_sc;                      -- spread
                        when 4      => z_fa <= signed(resize(z_av, 24));   z_fb <= to_unsigned(6436, 16);     -- speed, 1/256 octaves
                        when others => z_fa <= signed(resize(g_c, 24));    z_fb <= resize(to_unsigned(16368, 14) - resize(g_c, 14), 16);   -- camera gain
                    end case;
                elsif q_s = 1 then
                    z_fa <= signed(resize(z_sdf, 24)); z_fb <= resize(z_fs, 16);                              -- sine interpolation
                elsif q_s = 2 then
                    z_fa <= signed(resize(z_cdf, 24)); z_fb <= resize(z_fc, 16);                              -- cosine
                elsif q_sec = 0 then
                    case to_integer(q_s) is
                        when 3      => z_fa <= signed(resize(c_sd, 24));   z_fb <= resize(C_SV_DKQ(to_integer(q_w)), 16);   -- apex distance
                        when 4      => z_fa <= signed(resize(z_R, 24));    z_fb <= resize(z_cm, 16);
                        when 5      => z_fa <= signed(resize(z_R, 24));    z_fb <= resize(z_sm, 16);
                        when others => null;
                    end case;
                else
                    case to_integer(q_s) is
                        when 3      => z_fa <= signed(resize(z_rq, 24));   z_fb <= resize(z_sm, 16);          -- line step
                        when 4      => z_fa <= signed(resize(z_apx, 24));  z_fb <= resize(z_cm, 16);          -- cos * |apex x|
                        when 5      => z_fa <= signed(resize(z_apy, 24));  z_fb <= resize(z_sm, 16);          -- sin * |apex y|
                        when 6      => z_fa <= signed(resize(z_m, 24));    z_fb <= resize(z_cm, 16);          -- steps per line * cos
                        when others => null;
                    end case;
                end if;
            end if;
            if q_ph = 1 then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 40); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- results and the small steps between the products
            cr_we <= '0';
            if q_run = '1' and q_cut = '0' then
                case to_integer(q_s) is
                    when 0 =>                      -- half-angle = (2^(oct / 256) << int) >> 6, 65536 per turn
                        case to_integer(q_ph) is
                            when 19 => z_oct <= unsigned(f_acc(22 downto 13));
                            when 20 => r_ex_a <= z_oct(7 downto 0);
                            when 23 =>
                                v_i := resize(r_exf, 19);
                                case to_integer(z_oct(9 downto 8)) is
                                    when 3      => c_al <= resize(shift_right(shift_left(v_i, 3), 6), 14);
                                    when 2      => c_al <= resize(shift_right(shift_left(v_i, 2), 6), 14);
                                    when 1      => c_al <= resize(shift_right(shift_left(v_i, 1), 6), 14);
                                    when others => c_al <= resize(shift_right(v_i, 6), 14);
                                end case;
                            when others => null;
                        end case;
                    when 1 =>
                        if q_ph = 19 then z_px <= signed(resize(z_w48, 17)) + f_acc(31 downto 15); end if;
                    when 2 =>
                        if q_ph = 19 then z_py <= f_acc(31 downto 15); end if;
                    when 3 =>
                        case to_integer(q_ph) is
                            when 1 =>              -- Drift: sign and magnitude about the centre
                                v_s14 := signed(resize(g_t, 14)) - 4096;
                                z_vn <= v_s14(13);
                                if v_s14(13) = '1' then z_av <= unsigned(-v_s14(12 downto 0)); else z_av <= unsigned(v_s14(12 downto 0)); end if;
                            when 2 =>              -- +-24 counts = still
                                if z_av < 192 then z_vz <= '1'; z_av <= (others => '0');
                                else z_vz <= '0'; z_av <= z_av - 192; end if;
                            when 19 => c_sd <= unsigned(f_acc(27 downto 15));
                            when others => null;
                        end case;
                    when 4 =>                      -- speed = (mantissa >> 3) << octave: 2^-12 .. 2^-6 turn per field
                        case to_integer(q_ph) is
                            when 19 => z_ex <= unsigned(f_acc(24 downto 14));
                            when 20 => r_ex_a <= z_ex(7 downto 0);
                            when 23 =>
                                v_sp := resize(r_exf(15 downto 3), 18);
                                if z_vz = '1' then z_spd <= (others => '0');
                                else
                                    case to_integer(z_ex(10 downto 8)) is
                                        when 0      => z_spd <= v_sp;
                                        when 1      => z_spd <= shift_left(v_sp, 1);
                                        when 2      => z_spd <= shift_left(v_sp, 2);
                                        when 3      => z_spd <= shift_left(v_sp, 3);
                                        when 4      => z_spd <= shift_left(v_sp, 4);
                                        when others => z_spd <= shift_left(v_sp, 5);
                                    end case;
                                end if;
                            when others => null;
                        end case;
                    when others =>
                        if q_ph = 19 then c_G <= unsigned(f_acc(25 downto 14)); end if;
                end case;
            elsif q_run = '1' then
                if q_s = 0 then
                    -- slot 0 of a section: the angle, then its sine and cosine from the quarter-wave ROM
                    if q_sec = 0 then
                        case to_integer(q_ph) is
                            when 1 =>
                                v_rate := C_SV_RATE(to_integer(q_w));
                                z_rn <= v_rate(5) xor z_vn;
                                if v_rate(5) = '1' then z_ra <= unsigned(-v_rate(4 downto 0)); else z_ra <= unsigned(v_rate(4 downto 0)); end if;
                            when 2 =>              -- the phase accumulator (fabric copy of the table read); |rate| * speed
                                z_phi <= unsigned(cr_q(23 downto 0));
                                if z_ra(0) = '1' then v_a := resize(z_spd, 23); else v_a := (others => '0'); end if;
                                if z_ra(1) = '1' then v_b := shift_left(resize(z_spd, 23), 1); else v_b := (others => '0'); end if;
                                z_m2 <= v_a + v_b;
                            when 3 =>
                                if z_ra(2) = '1' then v_b := shift_left(resize(z_spd, 23), 2); else v_b := (others => '0'); end if;
                                z_m2 <= z_m2 + v_b;
                            when 4 =>
                                if z_ra(3) = '1' then v_b := shift_left(resize(z_spd, 23), 3); else v_b := (others => '0'); end if;
                                z_m2 <= z_m2 + v_b;
                            when 5 =>
                                if z_ra(4) = '1' then v_b := shift_left(resize(z_spd, 23), 4); else v_b := (others => '0'); end if;
                                z_m2 <= z_m2 + v_b;
                            when 6 =>              -- one step per field, wrapping exactly one turn (2^24)
                                z_phi <= f_addsub(z_phi, resize(z_m2(22 downto 4), 24), z_rn);
                            when 7 =>
                                cr_we <= '1'; cr_wa <= "110" & q_w; cr_wd <= signed(resize(z_phi, 32));
                                v_r := C_SV_PSI(to_integer(q_w)) + z_phi(23 downto 8);
                                z_A <= v_r; z_th <= v_r;
                            when others => null;
                        end case;
                    else
                        case to_integer(q_ph) is
                            when 1 => z_th <= z_A + C_SV_DPHI(to_integer(q_w));
                            when 2 =>
                                case to_integer(C_SV_OPF(to_integer(q_w))) is
                                    when 1      => z_alw <= c_al - shift_right(c_al, 2);
                                    when 2      => z_alw <= c_al + shift_right(c_al, 2);
                                    when 3      => z_alw <= shift_right(c_al, 1);
                                    when others => z_alw <= c_al;
                                end case;
                            when 3 => z_th <= f_addsub(z_th, resize(z_alw, 16), q_sec(1));
                            when 4 =>
                                if q_sec(1) = '1' then z_th <= z_th + 16384; else z_th <= z_th + 49152; end if;
                            when others => null;
                        end case;
                    end if;
                    case to_integer(q_ph) is
                        when 8 =>                  -- sine: quadrant fold
                            z_sneg <= z_th(15);
                            if z_th(14) = '1' then z_p <= to_unsigned(16384, 15) - resize(z_th(13 downto 0), 15);
                            else z_p <= resize(z_th(13 downto 0), 15); end if;
                        when 9 =>
                            sn_a <= z_p(13 downto 4); z_ov <= z_p(14); z_fs <= z_p(3 downto 0);
                            if z_p(14) = '1' or z_p(13 downto 4) = 1023 then z_ov1 <= '1'; else z_ov1 <= '0'; end if;
                        when 10 =>
                            sn_a <= sn_a + 1;
                        when 12 =>
                            if z_ov = '1' then z_sm <= to_unsigned(16384, 15); else z_sm <= sn_f(14 downto 0); end if;
                        when 13 =>
                            if z_ov = '1' then z_sdf <= (others => '0');
                            elsif z_ov1 = '1' then z_sdf <= resize(to_unsigned(16384, 15) - z_sm, 6);
                            else z_sdf <= resize(sn_f(14 downto 0) - z_sm, 6); end if;
                        when 14 =>                 -- cosine = sine a quarter turn on
                            v_r := z_th + 16384;
                            z_cneg <= v_r(15);
                            if v_r(14) = '1' then z_p <= to_unsigned(16384, 15) - resize(v_r(13 downto 0), 15);
                            else z_p <= resize(v_r(13 downto 0), 15); end if;
                        when 15 =>
                            sn_a <= z_p(13 downto 4); z_ov <= z_p(14); z_fc <= z_p(3 downto 0);
                            if z_p(14) = '1' or z_p(13 downto 4) = 1023 then z_ov1 <= '1'; else z_ov1 <= '0'; end if;
                        when 16 =>
                            sn_a <= sn_a + 1;
                        when 18 =>
                            if z_ov = '1' then z_cm <= to_unsigned(16384, 15); else z_cm <= sn_f(14 downto 0); end if;
                        when 19 =>
                            if z_ov = '1' then z_cdf <= (others => '0');
                            elsif z_ov1 = '1' then z_cdf <= resize(to_unsigned(16384, 15) - z_cm, 6);
                            else z_cdf <= resize(sn_f(14 downto 0) - z_cm, 6); end if;
                        when others => null;
                    end case;
                elsif q_s = 1 then
                    if q_ph = 19 then z_sm <= z_sm + resize(unsigned(f_acc(10 downto 4)) + ("000000" & f_acc(3)), 15); end if;
                elsif q_s = 2 then
                    if q_ph = 19 then z_cm <= z_cm + resize(unsigned(f_acc(10 downto 4)) + ("000000" & f_acc(3)), 15); end if;
                elsif q_sec = 0 then
                    -- the apex: gathering point + distance * (cos A, sin A), then sign and magnitude
                    case to_integer(q_s) is
                        when 3 =>
                            if q_ph = 19 then z_R <= unsigned(f_acc(21 downto 9)); end if;
                        when 4 =>
                            if q_ph = 19 then
                                z_ax <= signed(f_addsub(unsigned(z_px), resize(unsigned(f_acc(27 downto 14)), 17), z_cneg));
                            elsif q_ph = 20 then
                                c_pxn <= z_ax(16);
                                if z_ax(16) = '1' then z_apx <= unsigned(-z_ax(15 downto 0)); else z_apx <= unsigned(z_ax(15 downto 0)); end if;
                            end if;
                        when others =>
                            if q_ph = 19 then
                                z_ax <= signed(f_addsub(unsigned(z_py), resize(unsigned(f_acc(27 downto 14)), 17), z_sneg));
                            elsif q_ph = 20 then
                                c_pyn <= z_ax(16);
                                if z_ax(16) = '1' then z_apy <= unsigned(-z_ax(15 downto 0)); else z_apy <= unsigned(z_ax(15 downto 0)); end if;
                            end if;
                    end case;
                else
                    -- an edge: the line step, the seed (minus the apex along the normal, plus the bias), the
                    -- pixel step and the wrap, written to the edge table
                    case to_integer(q_s) is
                        when 3 =>
                            if q_ph = 19 then
                                v_t := signed(resize(unsigned(f_acc(37 downto 10)), 28));
                                z_dy <= signed(f_addsub(to_unsigned(0, 28), unsigned(v_t), z_sneg));
                            end if;
                        when 4 =>
                            if q_ph = 19 then
                                v_t := f_acc(30 downto 3);
                                z_seed <= signed(f_addsub(to_unsigned(0, 28), unsigned(v_t), not (z_cneg xor c_pxn)));
                            end if;
                        when 5 =>
                            if q_ph = 19 then
                                v_t := f_acc(30 downto 3);
                                z_seed <= signed(f_addsub(unsigned(z_seed), unsigned(v_t), not (z_sneg xor c_pyn)));
                            end if;
                        when others =>
                            case to_integer(q_ph) is
                                when 19 => z_seed <= z_seed + signed(resize(c_bias, 28));
                                when 20 =>             -- the pixel step: +-cos
                                    cr_we <= '1'; cr_wa <= q_w & q_sec(1) & "00";
                                    cr_wd <= resize(signed(f_addsub(to_unsigned(0, 28), resize(z_cm, 28), z_cneg)), 32);
                                when 21 =>             -- the wrap: line step - steps * pixel step
                                    cr_we <= '1'; cr_wa <= q_w & q_sec(1) & "01";
                                    cr_wd <= resize(signed(f_addsub(unsigned(z_dy), unsigned(f_acc(27 downto 0)), not z_cneg)), 32);
                                when 22 => cr_we <= '1'; cr_wa <= q_w & q_sec(1) & "10"; cr_wd <= resize(z_seed, 32);
                                when others => null;
                            end case;
                    end case;
                end if;
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
    -- [E] EDGES.  Twelve affine DDAs in pixel widths (Q13.14): each adds its own
    -- increment register on every clock of the line window (W + 8 steps), then
    -- once more with the line wrap parked in that register (wrap = line step
    -- - steps * pixel step), so the adder has no input mux.  The increments
    -- come from the edge table one word per clock.  The kick (end of the field
    -- sequencer) clears the accumulators and adds the field seed the same way.
    ---------------------------------------------------------------------------
    p_load : process(clk)
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
            b_e1 <= b_end; b_e2 <= b_e1; b_e3 <= b_e2;

            -- loader: 24 table reads (edge, wrap or seed), (edge, step)
            ca_rst <= l_kick;
            if l_kick = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '1';
            elsif b_end = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '0';
            elsif l_run = '1' then
                l_t <= l_t + 1;
                if l_t = 23 then l_run <= '0'; end if;
            end if;
            if q_run = '1' then                    -- the sequencer reads its sliver's phase word in phase 0
                if q_ph = 0 then cr_ra <= "110" & q_w; else cr_ra <= (others => '1'); end if;
            elsif l_t(0) = '1' then cr_ra <= l_t(4 downto 1) & "00";
            elsif l_seed = '1' then cr_ra <= l_t(4 downto 1) & "10";
            else cr_ra <= l_t(4 downto 1) & "01"; end if;
            -- the word addressed now is in cr_q two clocks on: it goes into the edge's increment register
            -- then (ca_ld), and a wrap / seed word is added one clock after that (ca_en), on the same edge
            -- that puts the pixel step back
            l_v1 <= l_run; l_c1 <= l_t(4 downto 1); l_o1 <= l_t(0);
            l_v2 <= l_v1;  l_c2 <= l_c1;            l_o2 <= l_o1;
            for k in 0 to 11 loop
                if l_v1 = '1' and l_c1 = k then ca_ld(k) <= '1'; else ca_ld(k) <= '0'; end if;
                if l_v2 = '1' and l_o2 = '0' and l_c2 = k then ca_en(k) <= '1'; else ca_en(k) <= '0'; end if;
            end loop;

            -- the line's virtual y and block row (shared by the grid's write and read sides): stepped after
            -- the line's last pixel has left both; the DDA pixel's virtual x
            if s_zone = '1' then
                m_yv <= (others => '0'); m_rb0 <= '0'; m_base <= (others => '0');
            elsif b_e2 = '1' then
                m_yv <= m_yv + resize(s_ys, 19);
            elsif b_e3 = '1' then
                m_rb0 <= m_yv(13);
                if m_yv(13) /= m_rb0 then m_base <= m_base + 60; end if;
            end if;
            if s_zgo = '1' then m_bank <= not m_bank; end if;
            if b_run = '1' then m_xr <= m_xr + resize(s_xs, 12); else m_xr <= c_x0; end if;
        end if;
    end process p_load;

    g_edge : for k in 0 to 11 generate
        p_edge : process(clk)
            variable v_hi : std_logic;
        begin
            if rising_edge(clk) then
                if ca_ld(k) = '1' then ca_inc(k) <= cr_q(27 downto 0); end if;
                if ca_rst = '1' then
                    ca_acc(k) <= (others => '0');
                elsif b_run = '1' or ca_en(k) = '1' then
                    ca_acc(k) <= ca_acc(k) + ca_inc(k);
                end if;
                -- stage 1: the edge value in sixteenths of a pixel (eighths when interlaced), clamped to 0 .. 255
                if s_il = '1' then
                    if ca_acc(k)(26 downto 19) /= "00000000" then v_hi := '1'; else v_hi := '0'; end if;
                else
                    if ca_acc(k)(26 downto 18) /= "000000000" then v_hi := '1'; else v_hi := '0'; end if;
                end if;
                if ca_acc(k)(27) = '1' then x1_es(k) <= (others => '0');
                elsif v_hi = '1' then x1_es(k) <= (others => '1');
                elsif s_il = '1' then x1_es(k) <= unsigned(ca_acc(k)(18 downto 11));
                else x1_es(k) <= unsigned(ca_acc(k)(17 downto 10)); end if;
                -- stage 2: coverage of the outer edge (the seed carries + 1/2 pixel + half the line width) and
                -- of the inner edge (a line width further in)
                if x1_es(k)(7 downto 4) /= "0000" then x2_p(k) <= to_unsigned(16, 5);
                else x2_p(k) <= '0' & x1_es(k)(3 downto 0); end if;
                if x1_es(k) < c_hw2 then x2_m(k) <= (others => '0');
                elsif x1_es(k) - c_hw2 >= 16 then x2_m(k) <= to_unsigned(16, 5);
                else x2_m(k) <= resize(x1_es(k) - c_hw2, 5); end if;
            end if;
        end process p_edge;
    end generate g_edge;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> box of 32 px (16 px off the
    -- 1920-wide rasters), edge-replicated and reloaded at every line start ->
    -- vertical one-pole (1/16 per line; 1/8 at 1080i and progressive SD, 1/4 at
    -- 480i / 576i) in the line store at half horizontal resolution -> the block
    -- grid: on the lower half of every block row each stored column overwrites
    -- its block's cell, so the cell ends as the block's last column and line.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_dl : signed(16 downto 0);
        variable v_n : signed(17 downto 0);
        variable v_t4 : unsigned(4 downto 0);
        variable v_fa, v_fb : unsigned(1 downto 0);
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
            -- two registers on: s_tv is the sample of column s_col
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_t0 <= s_sl; s_tv <= s_t0;
            if s_v2 = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 (16) samples; the samples before the line start count as the first one
            if s_v2 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            elsif s_b32 = '1' then bx_wp <= (others => '0'); bx_rp <= to_unsigned(226, 8);
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(242, 8); end if;
            bx_f <= bx_q;
            if s_bcnt(5) = '0' and (s_b32 = '1' or s_bcnt(4) = '0') then v_tap := s_sfirst; else v_tap := bx_f; end if;
            s_df <= signed(resize(s_tv, 9)) - signed(resize(v_tap, 9));
            s_tvd <= s_tv; s_v3 <= s_v2; s_c2 <= s_col;
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_sfirst <= s_tv;
                s_bcnt <= to_unsigned(1, 6);
            elsif s_v2 = '1' then
                if s_bcnt(5) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_ld = '1' then
                if s_b32 = '1' then s_bsum <= shift_left(resize(s_tvd, 14), 5);
                else s_bsum <= shift_left(resize(s_tvd, 14), 4); end if;
            elsif s_v3 = '1' then
                s_bsum <= unsigned(signed(s_bsum) + resize(s_df, 14));
            end if;
            s_v4 <= s_v3; s_bcol <= s_c2;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif s_ld = '1' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- vertical one-pole on the odd columns: q += (box - q) >> 4 (3, 2), Q8.8
            if s_b32 = '1' then s_eb <= s_bsum(12 downto 0) & "000"; else s_eb <= s_bsum(11 downto 0) & "0000"; end if;
            s_eq <= lb_q1;
            s_ev <= s_v4 and s_bcol(0); s_ecol <= s_bcol; s_efl <= s_fl;
            s_dd <= signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            if s_efl = '1' then s_fq <= s_eb; else s_fq <= s_eq; end if;
            s_fv <= s_ev; s_fcol <= s_ecol; s_ffl <= s_efl;
            if s_ffl = '1' then v_dl := (others => '0');
            elsif c_vsh = "00" then v_dl := shift_right(s_dd, 4);
            elsif c_vsh = "01" then v_dl := shift_right(s_dd, 3);
            else v_dl := shift_right(s_dd, 2); end if;
            v_n := signed(resize(s_fq, 18)) + resize(v_dl, 18);
            lb_wd <= unsigned(v_n(15 downto 0));
            lb_we <= s_fv; lb_wa <= s_fcol(10 downto 1);

            -- the block grid's write side: m_xw = the stored column's virtual x (h)
            if s_ld = '1' then
                m_xw <= c_x0 + resize(s_xs, 12);
            elsif lb_we = '1' then
                m_xw <= m_xw + resize(s_xs & '0', 12);
            end if;
            gr_we <= lb_we and m_yv(12);
            gr_wa <= m_bank & (m_base + resize(m_xw(11 downto 6), 11));
            gr_wd <= lb_wd(15 downto 8);

            -- camera table fill (zone, after the global slots): block luma -> film steps on the block's two
            -- triangles.  T4 = (luma * G) >> 16 quarter steps; lower triangle (T4 + 1) / 4 films, upper T4 / 4,
            -- at most 2.
            gt_we <= '0';
            if q_gdone = '1' then
                gt_run <= '1'; gt_i <= (others => '0'); gt_acc <= (others => '0');
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_i;
                v_t4 := resize(gt_acc(19 downto 16), 5) + 1;
                if v_t4(4 downto 2) >= 2 then v_fa := "10"; else v_fa := v_t4(3 downto 2); end if;
                if gt_acc(19 downto 18) >= 2 then v_fb := "10"; else v_fb := gt_acc(19 downto 18); end if;
                gt_wd <= "000000000000" & v_fa & v_fb;
                gt_acc <= gt_acc + resize(c_G, 20); gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] coverage (stages 3 .. 9) and the mosaic (stages 1 .. 6)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u8 : unsigned(7 downto 0);
        variable v_col : unsigned(5 downto 0);
        variable v_ds : signed(10 downto 0);
        variable v_o, v_i, v_d : signed(6 downto 0);
        variable v_po, v_pi : unsigned(4 downto 0);
        variable v_t : unsigned(5 downto 0);
        variable v_tot : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- mosaic, stage 1: the grid address of the pixel's block (the bank written last field); the
            -- diagonal's edge value in eighths of a virtual px (alternate blocks take the other diagonal)
            if m_xr(11 downto 8) = "1111" then v_col := to_unsigned(59, 6); else v_col := m_xr(11 downto 6); end if;
            gr_ra <= (not m_bank) & (m_base + resize(v_col, 11));
            if (m_yv(13) xor m_xr(6)) = '1' then v_u8 := m_xr(5 downto 0) & "00";
            else v_u8 := not (m_xr(5 downto 0) & "00"); end if;
            x1_d8 <= signed(resize(v_u8, 10)) - signed(resize(m_yv(12 downto 5), 10));
            -- 2: the lower triangle's coverage (the grid RAM's own register runs alongside)
            case c_msh is
                when "10"   => v_ds := shift_left(resize(x1_d8, 11), 1);
                when "00"   => v_ds := resize(shift_right(x1_d8, 1), 11);
                when others => v_ds := resize(x1_d8, 11);
            end case;
            v_ds := v_ds + 8;
            if v_ds(10) = '1' then x2_ca <= (others => '0');
            elsif v_ds(9 downto 4) /= "000000" then x2_ca <= to_unsigned(16, 5);
            else x2_ca <= '0' & unsigned(v_ds(3 downto 0)); end if;
            -- 3: fabric copy of the block's luma; 4: the camera table; 5: its fabric copy
            gr_f <= gr_q;
            gt_f <= gt_q(3 downto 0);
            x3_ca <= x2_ca; x4_ca <= x3_ca; x5_ca <= x4_ca;
            -- 6: the block's films: upper-triangle films everywhere, one more on the lower triangle
            if gt_f(3 downto 2) /= gt_f(1 downto 0) then
                x6_mos <= resize(gt_f(1 downto 0) & "0000", 6) + resize(x5_ca, 6);
            else
                x6_mos <= resize(gt_f(1 downto 0) & "0000", 6);
            end if;

            -- 3: per sliver, the sums of its two outer and its two inner edge coverages
            for w in 0 to 5 loop
                x3_o(w) <= resize(x2_p(2 * w), 6) + resize(x2_p(2 * w + 1), 6);
                x3_i(w) <= resize(x2_m(2 * w), 6) + resize(x2_m(2 * w + 1), 6);
            end loop;
            -- 4: the sliver's coverage.  Wedge = clamp(c1 + c2 - 1); Bowtie = |c1 + c2 - 1|; Fine = outer
            -- minus inner (Bowtie: the two full lines, added and clipped)
            for w in 0 to 5 loop
                v_o := signed(resize(x3_o(w), 7)) - 16;
                v_i := signed(resize(x3_i(w), 7)) - 16;
                v_d := signed(resize(x3_o(w), 7)) - signed(resize(x3_i(w), 7));
                if v_o(6) = '0' then v_po := unsigned(v_o(4 downto 0)); else v_po := (others => '0'); end if;
                if v_i(6) = '0' then v_pi := unsigned(v_i(4 downto 0)); else v_pi := (others => '0'); end if;
                if s_act(w) = '0' then
                    x4_c(w) <= (others => '0');
                elsif s_fine = '1' then
                    if s_bow = '0' then x4_c(w) <= v_po - v_pi;
                    elsif v_d >= 16 then x4_c(w) <= to_unsigned(16, 5);
                    else x4_c(w) <= unsigned(v_d(4 downto 0)); end if;
                elsif s_bow = '1' and v_o(6) = '1' then
                    x4_c(w) <= resize(unsigned(-v_o), 5);
                else
                    x4_c(w) <= v_po;
                end if;
            end loop;
            -- 5, 6: the six coverages summed
            x5_a <= resize(x4_c(0), 6) + resize(x4_c(1), 6);
            x5_b <= resize(x4_c(2), 6) + resize(x4_c(3), 6);
            x5_c <= resize(x4_c(4), 6) + resize(x4_c(5), 6);
            x6_sum <= resize(x5_a, 7) + resize(x5_b, 7) + resize(x5_c, 7);
            -- 7: plus the mosaic's films (Grey Fine: a line counts as two films)
            if s_fine = '1' and s_ink = '0' then x7_tot <= resize(x6_sum & '0', 8) + resize(x6_mos, 8);
            else x7_tot <= resize(x6_sum, 8) + resize(x6_mos, 8); end if;
            -- 8: Grey: films add, clipped at three (white).  Ink: the layers XOR: a triangle of the sum.
            if x7_tot >= 48 then x8_tc <= to_unsigned(48, 6); else x8_tc <= x7_tot(5 downto 0); end if;
            v_t := resize(x7_tot(4 downto 0), 6);
            if v_t > 16 then x8_v <= resize(to_unsigned(32, 6) - v_t, 5); else x8_v <= v_t(4 downto 0); end if;
            -- 9: luma.  Grey: 64 + 18.25 per sixteenth of a film
            if s_ink = '1' then
                s_y23 <= f_luma(x8_v);
            else
                s_y23 <= to_unsigned(64, 10) + shift_left(resize(x8_tc, 10), 4) + shift_left(resize(x8_tc, 10), 1)
                         + resize(shift_right(x8_tc, 2), 10);
            end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: luma [1 2 1]/4, luma 13/16 toward black, blanking
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

end architecture sliver;
