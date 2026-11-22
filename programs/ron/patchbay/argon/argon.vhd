-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Argon v0.1.0 -- Patchbay: shape generators into glow, a neon sign of pure geometry.
--
-- The patch:
--   H ramp^2 + V ramp^2 (a circle generator) -> (r^2 - R^2) / 2R            = the ring's distance
--   H, V ramps -> rotators -> analog max                                     = the triangle's distance
--   H, V ramps -> rotators -> |.| -> analog max                              = the hexagon's distance
--   (S7 Square: two rotators each, |.|: a square and a diamond)
--   camera luma -> 128 / 96-virtual-px cells + vertical lowpass -> subtracted from all three distances
--   each |distance| -> window comparator + glow lowpass, as a per-tube table of RGB light against
--   |distance| (white core, gas-coloured body, exponential halo, wide wall wash), rebuilt every field
--   three tubes -> additive RGB mixer -> the camera's shoulder (white-hot cores) -> Y / U / V
--   LFSR + sequencer -> per-tube brightness (Ballast Flicker); Motor -> the two polygons counter-rotate
--
--   K1  Size      the polygon nest (x0.35 .. x1.6 of the inscribed composition); the ring follows a little
--   K2  Motor     counter-rotation rate, bipolar (square law, +-8 deadband)
--   K3  Glow      halo length 5 .. 190 px
--   K4  Gas       the colour set, glided round five gases
--   K5  Tube      core thickness
--   K6  Camera    camera strength (0 = the black-input picture)
--   S7  Figure    Round (ring, triangle, hexagon) / Square (ring, square, diamond)
--   S8  Tubing    Single / Double
--   S9  Ballast   Steady / Flicker
--   S10 Wall      Dark / Wash
--   S11 Source    Video / Osc
--   P12 VOLTAGE   core width, core light, halo strength and length: barely struck .. over-voltage
--
-- Engine: a microcoded patch computer (argon.py: one serial multiplier, 32-bit instructions, a 256 x 32
-- register file) runs once per field; part 1 in the vblank zone (edge DDA coefficients, the ring's
-- quadratic DDA, apothems, gap, camera gain), part 2 during the field (table constants), then the table
-- writer rebuilds the three tube tables (256 entries of RGB) in the back bank; banks swap at the zone.
-- Pixel path: four 26-bit edge accumulators (the third edge of each figure is their sum / difference), the
-- ring DDA, max trees, compander, table reads, mixer, shoulder, matrix; then the shell's output
-- conditioning (its 8-sample delays in block-RAM rings).
-- Virtual 1080-line coordinates.  Generated avid; output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16) before the blanking gate.  Colours in
-- the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.argon_pkg.all;
use work.all;

architecture argon of program_top is
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
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- Osc lattice: phase per field line = 242 / 256 per virtual line (a 271-line period)
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1075, 11);
            when 4          => return to_unsigned(544, 11);
            when 8          => return to_unsigned(907, 11);
            when 12         => return to_unsigned(453, 11);
            when 5 | 6 | 14 => return to_unsigned(363, 11);
            when 1 | 2 | 10 => return to_unsigned(484, 11);
            when others     => return to_unsigned(242, 11);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- ones' complement magnitude of a signed value (|v| - 1 for negatives: 1/16 px, invisible)
    function f_oabs(v : signed) return unsigned is
        variable x : signed(v'length - 1 downto 0);
    begin
        x := v;
        if x(x'high) = '1' then return unsigned(not x); else return unsigned(x); end if;
    end function;
    -- analog max of three (parallel compares)
    function f_max3(a, b, c : signed(15 downto 0)) return signed is
    begin
        if a > b and a > c then return a;
        elsif b > c then return b;
        else return c; end if;
    end function;
    -- the glow tables' index: 0.5 px steps to 64 px, 2 px to 192, 8 px to 448, 32 px to 1472 (a in Q4)
    function f_comp(a : unsigned(14 downto 0)) return unsigned is
        variable r : unsigned(8 downto 0);
    begin
        if a < 1024 then r := resize(a(9 downto 3), 9);
        elsif a < 3072 then r := resize(a(11 downto 5), 9) + 96;
        elsif a < 7168 then r := resize(a(12 downto 7), 9) + 168;
        elsif a < 23552 then r := resize(a(14 downto 9), 9) + 210;
        else r := to_unsigned(255, 9); end if;
        return r(7 downto 0);
    end function;
    -- the camera's shoulder: identity to 0.75, then slopes 1/4 and 1/16, flat at 1.0 (= 384)
    function f_tone(x : unsigned(11 downto 0)) return unsigned is
        variable r : unsigned(11 downto 0);
    begin
        if x < 288 then r := x;
        elsif x < 596 then r := 288 + shift_right(x - 288, 2);
        elsif x < 900 then r := 365 + shift_right(x - 596, 4);
        else r := to_unsigned(384, 12); end if;
        return r(8 downto 0);
    end function;
    -- shift codes of the patch computer
    function f_xs(c : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(c)) is
            when 0  => return to_unsigned(0, 5);
            when 1  => return to_unsigned(1, 5);
            when 2  => return to_unsigned(2, 5);
            when 3  => return to_unsigned(3, 5);
            when 4  => return to_unsigned(6, 5);
            when 5  => return to_unsigned(8, 5);
            when 6  => return to_unsigned(10, 5);
            when 7  => return to_unsigned(11, 5);
            when 8  => return to_unsigned(12, 5);
            when 9  => return to_unsigned(14, 5);
            when 10 => return to_unsigned(15, 5);
            when 11 => return to_unsigned(16, 5);
            when 12 => return to_unsigned(19, 5);
            when 13 => return to_unsigned(20, 5);
            when 14 => return to_unsigned(25, 5);
            when others => return to_unsigned(28, 5);
        end case;
    end function;

    ---------------------------------------------------------------------------
    -- memories
    ---------------------------------------------------------------------------
    -- register file of the patch computer (power-up image: constants, persistent variables)
    signal rf     : t_ar_rom32(0 to 255) := C_AR_RF;
    attribute no_rw_check : boolean;
    attribute no_rw_check of rf : signal is true;
    signal rf_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal rf_we  : std_logic := '0';
    signal rf_wa  : unsigned(7 downto 0) := (others => '0');
    signal rf_wd  : signed(31 downto 0) := (others => '0');
    -- glow tables: bank & index -> B G R (10 bits each)
    type t_tab is array (0 to 511) of std_logic_vector(31 downto 0);
    signal tab0, tab1, tab2 : t_tab := (others => (others => '0'));
    attribute no_rw_check of tab0 : signal is true;
    attribute no_rw_check of tab1 : signal is true;
    attribute no_rw_check of tab2 : signal is true;
    signal t_ra0, t_ra1, t_ra2 : unsigned(8 downto 0) := (others => '0');
    signal t_q0, t_q1, t_q2 : std_logic_vector(31 downto 0) := (others => '0');
    signal t_we   : std_logic_vector(2 downto 0) := "000";
    signal t_wa   : unsigned(8 downto 0) := (others => '0');
    signal t_wd   : std_logic_vector(31 downto 0) := (others => '0');
    -- camera cell store (vertical lowpass state per cell, Q7 px)
    type t_cram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal cram   : t_cram := (others => (others => '0'));
    attribute no_rw_check of cram : signal is true;
    signal cr_ra, cr_wa : unsigned(7 downto 0) := (others => '0');
    signal cr_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal cr_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal cr_we  : std_logic := '0';
    -- output-conditioning delay rings (8 samples: read pointer = write - 6, then a fabric register)
    type t_oram is array (0 to 255) of std_logic_vector(31 downto 0);
    signal oa_ram, ob_ram : t_oram := (others => (others => '0'));
    attribute no_rw_check of oa_ram : signal is true;
    attribute no_rw_check of ob_ram : signal is true;
    signal o_wp   : unsigned(7 downto 0) := (others => '0');
    signal oa_q, ob_q, oa_f, ob_f : std_logic_vector(31 downto 0) := (others => '0');
    -- ROMs
    signal r_uc_a : unsigned(7 downto 0) := (others => '0');
    signal r_uc0, r_uc1 : std_logic_vector(15 downto 0) := (others => '0');
    signal r_ex_a, w_exa : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_sn_a : unsigned(8 downto 0) := (others => '0');
    signal r_sn_q : std_logic_vector(15 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- shell: timing, zone, width, knobs
    ---------------------------------------------------------------------------
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);    -- P12 Voltage (glided)
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6 : unsigned(9 downto 0) := (others => '0');
    signal s_sq, s_dbl, s_flk, s_wash, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_grun, z_grund : std_logic := '0';
    signal s_wh  : unsigned(9 downto 0) := to_unsigned(960, 10);
    signal s_cs  : std_logic := '1';                                    -- camera cell = 128 px (else 64)
    signal s_kv  : unsigned(2 downto 0) := to_unsigned(5, 3);          -- camera vertical lowpass shift

    -- Ballast
    signal f_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal f_ep, f_gap : unsigned(5 downto 0) := (others => '0');
    signal f_tube : unsigned(1 downto 0) := "00";
    signal f_go   : std_logic := '0';
    signal f_b0, f_b1, f_b2 : unsigned(8 downto 0) := to_unsigned(256, 9);

    ---------------------------------------------------------------------------
    -- the patch computer
    ---------------------------------------------------------------------------
    type t_ust is (U_IDLE, U_F1, U_F2, U_R1, U_R2, U_R3, U_R4, U_EX, U_MUL, U_MSH, U_SHF, U_FIN, U_ROM1, U_ROM2,
                   U_LD1, U_LD2, U_LD3, U_WB, U_SYNC);
    signal u_st : t_ust := U_IDLE;
    attribute fsm_encoding : string;
    attribute fsm_encoding of u_st : signal is "none";
    signal u_pc   : unsigned(7 downto 0) := (others => '0');
    signal u_ir   : std_logic_vector(31 downto 0) := (others => '0');
    signal u_x    : unsigned(4 downto 0) := (others => '0');
    signal u_ra   : unsigned(7 downto 0) := (others => '0');
    signal u_a, u_b : signed(31 downto 0) := (others => '0');
    signal u_p    : signed(47 downto 0) := (others => '0');
    signal u_cnt  : unsigned(4 downto 0) := (others => '0');
    signal u_dir, u_neg, u_rom : std_logic := '0';
    signal u_sh   : signed(7 downto 0) := (others => '0');
    signal u_n    : std_logic := '0';
    signal u_done, u_syncp : std_logic := '0';
    signal u_in   : signed(31 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- bus sequencer (constants, edge seeds, hblank wraps), accumulators, ring
    ---------------------------------------------------------------------------
    signal w_ra   : unsigned(7 downto 0) := (others => '0');
    signal w_cst, w_run, w_seed, w_pre, w_sdone : std_logic := '0';
    signal w_ci   : unsigned(3 downto 0) := (others => '0');
    signal w_k    : unsigned(2 downto 0) := (others => '0');
    signal w_p    : unsigned(1 downto 0) := (others => '0');
    signal w_c1, w_c2, w_c3 : std_logic := '0';
    signal w_ci1, w_ci2, w_ci3 : unsigned(3 downto 0) := (others => '0');
    signal w_v1, w_v2, w_v3 : std_logic := '0';
    signal w_k1, w_k2, w_k3 : unsigned(2 downto 0) := (others => '0');
    signal w_p1, w_p2, w_p3 : unsigned(1 downto 0) := (others => '0');
    signal w_sd1, w_sd2, w_sd3 : std_logic := '0';
    signal c_bus  : std_logic_vector(31 downto 0) := (others => '0');
    signal bus_own_d, bus_hold : std_logic := '0';
    signal b_run, b_rund, b_run1, b_run2, g_st1, g_st2 : std_logic := '0';
    signal b_cnt  : unsigned(11 downto 0) := (others => '0');
    type t_s26 is array (0 to 3) of signed(25 downto 0);
    signal b_acc, b_op : t_s26 := (others => (others => '0'));
    signal c_dd, c_gg : signed(31 downto 0) := (others => '0');   -- c_dd = D's operand (DD, or the wrap)
    signal c_ha, c_hb : signed(15 downto 0) := (others => '0');
    signal c_gap  : unsigned(15 downto 0) := (others => '0');
    signal c_cg   : unsigned(11 downto 0) := (others => '0');
    signal r_f, r_d, r_f0, r_g : signed(31 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- Osc lattice
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(204, 9);
    signal s_oys : unsigned(10 downto 0) := to_unsigned(242, 11);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- table writer
    ---------------------------------------------------------------------------
    signal wr_run, wr_done : std_logic := '0';
    signal wr_front : std_logic := '0';
    signal wr_ra  : unsigned(7 downto 0) := (others => '0');
    signal wr_i   : unsigned(7 downto 0) := (others => '0');
    signal wr_s   : unsigned(4 downto 0) := (others => '0');
    signal wr_ph  : unsigned(2 downto 0) := (others => '0');
    signal wr_cnt : unsigned(4 downto 0) := (others => '0');
    signal m_a    : signed(17 downto 0) := (others => '0');
    signal m_p    : signed(39 downto 0) := (others => '0');     -- [hi 24 | lo 16 = B]: P = hi_init + A B
    signal w_dl   : unsigned(14 downto 0) := (others => '0');
    signal w_pc   : unsigned(15 downto 0) := (others => '0');
    signal w_core : unsigned(10 downto 0) := (others => '0');
    signal w_e    : unsigned(15 downto 0) := (others => '0');
    signal w_x    : unsigned(15 downto 0) := (others => '0');
    signal w_lc   : signed(23 downto 0) := (others => '0');
    signal w_vcc  : signed(23 downto 0) := (others => '0');
    signal w_qf   : unsigned(15 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- camera block
    ---------------------------------------------------------------------------
    signal k_px   : unsigned(6 downto 0) := (others => '0');
    signal k_cell : unsigned(4 downto 0) := (others => '0');
    signal k_sum  : signed(15 downto 0) := (others => '0');
    signal k_go, k_first, k_fl, k_neg : std_logic := '0';
    signal k_lc   : unsigned(4 downto 0) := (others => '0');
    signal k_cnt  : unsigned(3 downto 0) := (others => '0');
    signal k_p    : unsigned(20 downto 0) := (others => '0');   -- [hi 12 | lo 9 = |mean|]: gain x |mean|
    signal k_old  : signed(15 downto 0) := (others => '0');
    signal k_job, k_two : std_logic := '0';
    signal k_tgt  : signed(16 downto 0) := (others => '0');
    signal q_px   : unsigned(6 downto 0) := (others => '0');
    signal q_cell : unsigned(4 downto 0) := (others => '0');
    signal q_nc   : unsigned(4 downto 0) := to_unsigned(15, 5);
    signal q_rv1, q_rv2, q_rv3, q_rg1, q_rg2, q_rg3, q_ro1, q_ro2, q_ro3, q_last : std_logic := '0';
    signal q_rq   : std_logic_vector(15 downto 0) := (others => '0');
    signal q_fn   : signed(15 downto 0) := (others => '0');
    signal q_v    : signed(22 downto 0) := (others => '0');
    signal q_step : signed(17 downto 0) := (others => '0');
    signal q_cam  : signed(11 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    type t_s16x3 is array (0 to 2) of signed(15 downto 0);
    type t_s18x3 is array (0 to 2) of signed(17 downto 0);
    type t_u15x3 is array (0 to 2) of unsigned(14 downto 0);
    type t_u8x3 is array (0 to 2) of unsigned(7 downto 0);
    signal p2_a, p2_b : t_s16x3 := (others => (others => '0'));

    signal p3_ma, p3_mb : signed(15 downto 0) := (others => '0');
    signal p3_hac, p3_hbc : signed(16 downto 0) := (others => '0');
    signal p4_d   : t_s18x3 := (others => (others => '0'));
    signal p5_a   : t_u15x3 := (others => (others => '0'));
    signal p6_a   : t_u15x3 := (others => (others => '0'));
    signal p7_i, p8_i : t_u8x3 := (others => (others => '0'));
    signal p11_0, p11_1, p11_2 : std_logic_vector(29 downto 0) := (others => '0');
    signal p12_r, p12_g, p12_b : unsigned(11 downto 0) := (others => '0');
    signal p13_r, p13_g, p13_b : unsigned(8 downto 0) := (others => '0');
    signal p14_a, p14_bc : unsigned(12 downto 0) := (others => '0');
    signal p14_tb2 : unsigned(9 downto 0) := (others => '0');
    signal p14_r, p14_b, p15_r, p15_b : unsigned(8 downto 0) := (others => '0');
    signal p15_y  : unsigned(8 downto 0) := (others => '0');
    signal p16_dr, p16_db : unsigned(9 downto 0) := (others => '0');
    signal p16_ya : unsigned(10 downto 0) := (others => '0');
    signal p16_yb : unsigned(3 downto 0) := (others => '0');
    signal p17_u1, p17_u2, p17_v1, p17_v2 : unsigned(10 downto 0) := (others => '0');
    signal p17_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p18_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p18_u, p18_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bay, s_bayd, s_bby, s_bbyd : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21, s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hi, s_vi, s_ho, s_vo : std_logic_vector(3 downto 0) := (others => '1');   -- hsync / vsync: 4 FFs
                                                                                       -- + the two rings + 4 FFs
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories (one write statement each; the reads never meet a write)
    ---------------------------------------------------------------------------
    p_rf : process(clk)
        variable v_a : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if bus_own_d = '1' then v_a := w_ra;
            elsif wr_run = '1' then v_a := wr_ra;
            else v_a := u_ra; end if;
            rf_q <= rf(to_integer(v_a));
            if rf_we = '1' then rf(to_integer(rf_wa)) <= std_logic_vector(rf_wd); end if;
        end if;
    end process p_rf;

    p_tab0 : process(clk)
    begin
        if rising_edge(clk) then
            t_q0 <= tab0(to_integer(t_ra0));
            if t_we(0) = '1' then tab0(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_tab0;
    p_tab1 : process(clk)
    begin
        if rising_edge(clk) then
            t_q1 <= tab1(to_integer(t_ra1));
            if t_we(1) = '1' then tab1(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_tab1;
    p_tab2 : process(clk)
    begin
        if rising_edge(clk) then
            t_q2 <= tab2(to_integer(t_ra2));
            if t_we(2) = '1' then tab2(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_tab2;

    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= cram(to_integer(cr_ra));
            if cr_we = '1' then cram(to_integer(cr_wa)) <= cr_wd; end if;
        end if;
    end process p_cram;

    p_oa : process(clk)
    begin
        if rising_edge(clk) then
            oa_q <= oa_ram(to_integer(o_wp - 6));
            oa_ram(to_integer(o_wp)) <= s_hi(3) & s_vi(3) & std_logic_vector(s_v19) & std_logic_vector(s_u19) & std_logic_vector(s_y19);
        end if;
    end process p_oa;
    p_ob : process(clk)
    begin
        if rising_edge(clk) then
            ob_q <= ob_ram(to_integer(o_wp - 6));
            ob_ram(to_integer(o_wp)) <= oa_f(31 downto 30) & std_logic_vector(s_v21) & std_logic_vector(s_u21) & std_logic_vector(s_y21);
        end if;
    end process p_ob;

    p_rom : process(clk)
        variable v_ea : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            r_uc0 <= C_AR_UC0(to_integer(r_uc_a));
            r_uc1 <= C_AR_UC1(to_integer(r_uc_a));
            if wr_run = '1' then v_ea := w_exa; else v_ea := r_ex_a; end if;
            r_ex_q <= C_AR_EXP2(to_integer(v_ea));
            r_sn_q <= C_AR_SIN(to_integer(r_sn_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, width, glides (K4, K6, P12), per-mode constants
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_oys <= f_oys(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            s_k1 <= unsigned(registers_in(0)); s_k2 <= unsigned(registers_in(1));
            s_k3 <= unsigned(registers_in(2)); s_k4 <= unsigned(registers_in(3));
            s_k5 <= unsigned(registers_in(4)); s_k6 <= unsigned(registers_in(5));

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_sq   <= registers_in(6)(0);
                s_dbl  <= registers_in(6)(1);
                s_flk  <= registers_in(6)(2);
                s_wash <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(408, 9); s_wh <= to_unsigned(720, 10);
                                   s_cs <= '0'; q_nc <= to_unsigned(11, 5);
                    when 3      => s_oxs <= to_unsigned(306, 9); s_wh <= to_unsigned(960, 10);
                                   s_cs <= '0'; q_nc <= to_unsigned(20, 5);
                    when others => s_oxs <= to_unsigned(204, 9); s_wh <= to_unsigned(960, 10);
                                   s_cs <= '1'; q_nc <= to_unsigned(15, 5);
                end case;
                case to_integer(s_ys) is
                    when 256 | 384 => s_kv <= to_unsigned(5, 3);
                    when 512       => s_kv <= to_unsigned(4, 3);
                    when others    => s_kv <= to_unsigned(3, 3);
                end case;
            end if;
            g_wm1 <= s_w - 1;

            -- P12 glide (1/8 of the gap per field, +-2 deadband), run from the zone start
            z_grund <= z_grun;
            if s_zgo = '1' then
                z_gp <= "11"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "11" =>
                        s_gd <= signed(resize(unsigned(registers_in(7)) & "000", 15)) - signed(resize(s_gp, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then s_gd <= shift_right(s_gd, 3);
                        else s_gd <= (others => '0'); end if;
                        z_gp <= "10";
                    when others =>
                        s_gp <= unsigned(signed(s_gp) + s_gd(12 downto 0));
                        z_grun <= '0';
                end case;
            end if;

            s_hi <= s_hi(2 downto 0) & data_in.hsync_n; s_vi <= s_vi(2 downto 0) & data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Ballast: an LFSR and a sequencer choose each tube's brightness per field (after the glides).
    -- Flicker: one tube at a time stutters for 30..61 fields, then a 20..51-field gap.  Steady: below
    -- 30 % Voltage an occasional one-field dip.
    ---------------------------------------------------------------------------
    p_flk : process(clk)
        variable v_l : unsigned(15 downto 0);
        variable v_lv : unsigned(8 downto 0);
        variable v_t : unsigned(1 downto 0);
        variable v_gp : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            f_go <= z_grund and not z_grun;
            if f_go = '1' then
                if f_lfsr(0) = '1' then v_l := ('0' & f_lfsr(15 downto 1)) xor x"B400";
                else v_l := '0' & f_lfsr(15 downto 1); end if;
                f_lfsr <= v_l;
                case to_integer(v_l(2 downto 0)) is
                    when 0 | 1  => v_lv := to_unsigned(0, 9);
                    when 2      => v_lv := to_unsigned(40, 9);
                    when 3      => v_lv := to_unsigned(100, 9);
                    when 6      => v_lv := to_unsigned(16, 9);
                    when 7      => v_lv := to_unsigned(180, 9);
                    when others => v_lv := to_unsigned(256, 9);
                end case;
                f_b0 <= to_unsigned(256, 9); f_b1 <= to_unsigned(256, 9); f_b2 <= to_unsigned(256, 9);
                v_gp := s_gp(12 downto 3);
                if s_flk = '1' then
                    if f_ep /= 0 then
                        f_ep <= f_ep - 1;
                        case f_tube is
                            when "00"   => f_b0 <= v_lv;
                            when "01"   => f_b1 <= v_lv;
                            when others => f_b2 <= v_lv;
                        end case;
                    elsif f_gap /= 0 then
                        f_gap <= f_gap - 1;
                    else
                        if v_l(5 downto 4) = "11" then f_tube <= "00"; else f_tube <= v_l(5 downto 4); end if;
                        f_ep  <= resize(v_l(10 downto 6), 6) + 30;
                        f_gap <= resize(v_l(15 downto 11), 6) + 20;
                    end if;
                else
                    f_ep <= (others => '0'); f_gap <= (others => '0');
                    if v_gp < 307 and v_l(7 downto 0) < resize(shift_right(307 - v_gp, 5), 8) then
                        if v_l(9 downto 8) = "11" then v_t := "00"; else v_t := v_l(9 downto 8); end if;
                        case v_t is
                            when "00"   => f_b0 <= to_unsigned(140, 9);
                            when "01"   => f_b1 <= to_unsigned(140, 9);
                            when others => f_b2 <= to_unsigned(140, 9);
                        end case;
                    end if;
                end if;
            end if;
        end if;
    end process p_flk;

    ---------------------------------------------------------------------------
    -- The patch computer (argon.py documents the instruction set).  Started from PC 0 at every zone start;
    -- part 1 ends in SYNC (the bus sequencer then loads the geometry), part 2 ends in END (the table writer
    -- then runs).  Operand reads wait while the bus sequencer owns the register file's read port.
    ---------------------------------------------------------------------------
    p_in : process(clk)
        variable v : signed(31 downto 0);
        function sw(b : std_logic) return signed is
        begin
            if b = '1' then return to_signed(-1, 32); else return to_signed(0, 32); end if;
        end function;
    begin
        if rising_edge(clk) then
            case to_integer(unsigned(u_ir(11 downto 4))) is
                when 0  => v := signed(resize(s_k1, 32));
                when 1  => v := signed(resize(s_k2, 32));
                when 2  => v := signed(resize(s_k3, 32));
                when 3  => v := signed(resize(s_k4, 32));
                when 4  => v := signed(resize(s_k5, 32));
                when 5  => v := signed(resize(s_k6, 32));
                when 6  => v := signed(resize(s_gp(12 downto 3), 32));
                when 7  => v := sw(s_sq);
                when 8  => v := sw(s_dbl);
                when 9  => v := sw(s_flk);
                when 10 => v := sw(s_wash);
                when 11 => v := sw(s_src);
                when 12 => v := signed(resize(s_xs, 32));
                when 13 => v := signed(resize(s_ys, 32));
                when 14 => v := signed(resize(s_w, 32));
                when 15 => v := signed(resize(s_wh, 32));
                when 16 => v := signed(resize(f_b0, 32));
                when 17 => v := signed(resize(f_b1, 32));
                when 18 => v := signed(resize(f_b2, 32));
                when others => v := (others => '0');
            end case;
            u_in <= v;
            u_x <= f_xs(u_ir(3 downto 0));
        end if;
    end process p_in;

    p_cpu : process(clk)
        variable v_op  : integer range 0 to 15;
        variable v_s   : signed(32 downto 0);
        variable v_al  : signed(32 downto 0);
        variable v_sub : std_logic;
        variable v_ang : unsigned(15 downto 0);
        variable v_idx : unsigned(8 downto 0);
        variable v_sh  : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            rf_we <= '0';
            u_done <= '0';
            u_syncp <= '0';
            v_op := to_integer(unsigned(u_ir(31 downto 28)));
            if s_zgo = '1' then
                u_pc <= (others => '0'); r_uc_a <= (others => '0'); u_st <= U_F1;
            else
                case u_st is
                    when U_IDLE => null;
                    when U_F1 => u_st <= U_F2;                                   -- ROM reads r_uc_a
                    when U_F2 => u_ir <= r_uc1 & r_uc0; u_st <= U_R1;
                    when U_R1 =>
                        if bus_hold = '0' then u_ra <= unsigned(u_ir(19 downto 12)); u_st <= U_R2; end if;
                    when U_R2 => u_ra <= unsigned(u_ir(11 downto 4)); u_st <= U_R3;
                    when U_R3 => u_a <= signed(rf_q); u_st <= U_R4;
                    when U_R4 => u_b <= signed(rf_q); u_st <= U_EX;
                    when U_EX =>
                        case v_op is
                            when 0 =>                                                   -- END
                                u_done <= '1'; u_st <= U_IDLE;
                            when 1 | 2 =>                                               -- MULU / MULS
                                u_p <= (others => '0'); u_cnt <= to_unsigned(16, 5); u_st <= U_MUL;
                            when 3 | 5 =>                                               -- ADD / SUB: B << x
                                u_cnt <= u_x; u_dir <= '1'; u_st <= U_SHF;
                            when 4 | 6 =>                                               -- ADDR / SUBR: B >>> x
                                u_cnt <= u_x; u_dir <= '0'; u_st <= U_SHF;
                            when 10 =>                                                  -- EXP2
                                r_ex_a <= unsigned(u_a(7 downto 0));
                                u_sh <= resize(u_a(31 downto 8), 8) + signed(resize(u_x, 8)) - 15;
                                u_rom <= '0'; u_st <= U_ROM1;
                            when 11 =>                                                  -- SIN
                                v_ang := unsigned(u_a(31 downto 16));
                                if u_x = 1 then v_ang := v_ang + 16384; end if;
                                v_idx := v_ang(13 downto 5);
                                if v_ang(14) = '1' then v_idx := 511 - v_idx; end if;
                                r_sn_a <= v_idx; u_neg <= v_ang(15);
                                u_rom <= '1'; u_st <= U_ROM1;
                            when 12 =>                                                  -- IN
                                u_b <= u_in; u_st <= U_FIN;
                            when 13 =>                                                  -- LDX
                                u_st <= U_LD1;
                            when 15 =>                                                  -- SYNC
                                u_syncp <= '1'; u_st <= U_SYNC;
                            when others =>                                              -- MIN / MAX / SEL
                                u_st <= U_FIN;
                        end case;
                    when U_MUL =>
                        -- right-shift form: acc(47:16) +- A, then >>> 1; bit 15 of a signed B subtracts
                        if u_b(0) = '1' then
                            if v_op = 2 and u_cnt = 1 then v_s := resize(u_p(47 downto 16), 33) - resize(u_a, 33);
                            else v_s := resize(u_p(47 downto 16), 33) + resize(u_a, 33); end if;
                        else
                            v_s := resize(u_p(47 downto 16), 33);
                        end if;
                        u_p <= v_s & u_p(15 downto 1);
                        u_b <= shift_right(u_b, 1);
                        u_cnt <= u_cnt - 1;
                        if u_cnt = 1 then u_cnt <= u_x; u_st <= U_MSH; end if;
                    when U_MSH =>
                        if u_cnt = 0 then
                            rf_wd <= u_p(31 downto 0); u_st <= U_WB;
                        else
                            u_p <= shift_right(u_p, 1); u_cnt <= u_cnt - 1;
                        end if;
                    when U_SHF =>
                        if u_cnt = 0 then
                            u_st <= U_FIN;
                        elsif u_dir = '1' then
                            u_b <= shift_left(u_b, 1); u_cnt <= u_cnt - 1;
                        else
                            u_b <= shift_right(u_b, 1); u_cnt <= u_cnt - 1;
                        end if;
                    when U_ROM1 => u_st <= U_ROM2;                                -- ROM address -> data
                    when U_ROM2 =>                                                -- data -> fabric
                        if u_rom = '1' then
                            u_b <= signed(resize(unsigned(r_sn_q), 32)); u_a <= (others => '0');
                            u_st <= U_FIN;
                        else
                            u_b <= signed(resize(unsigned(r_ex_q), 32));
                            if u_sh >= 0 then
                                u_cnt <= unsigned(u_sh(4 downto 0)); u_dir <= '1';
                            elsif u_sh < -31 then
                                u_b <= (others => '0'); u_cnt <= (others => '0'); u_dir <= '0';
                            else
                                v_sh := -u_sh;
                                u_cnt <= unsigned(v_sh(4 downto 0)); u_dir <= '0';
                            end if;
                            u_st <= U_SHF;
                        end if;
                    when U_FIN =>
                        -- the one ALU adder: A +- B (B already shifted); MIN / MAX use its sign
                        if v_op = 5 or v_op = 6 or v_op = 7 or v_op = 8 or (v_op = 11 and u_neg = '1') then v_sub := '1';
                        else v_sub := '0'; end if;
                        if v_sub = '1' then v_al := resize(u_a, 33) - resize(u_b, 33);
                        else v_al := resize(u_a, 33) + resize(u_b, 33); end if;
                        case v_op is
                            when 7 =>  if v_al(32) = '1' then rf_wd <= u_a; else rf_wd <= u_b; end if;
                            when 8 =>  if v_al(32) = '1' then rf_wd <= u_b; else rf_wd <= u_a; end if;
                            when 9 =>  if u_n = '1' then rf_wd <= u_a; else rf_wd <= u_b; end if;
                            when 10 | 12 | 13 => rf_wd <= u_b;
                            when others => rf_wd <= v_al(31 downto 0);
                        end case;
                        u_st <= U_WB;
                    when U_LD1 =>
                        if bus_hold = '0' then
                            u_ra <= unsigned(u_a(7 downto 0)) + unsigned(u_ir(11 downto 4)); u_st <= U_LD2;
                        end if;
                    when U_LD2 => u_st <= U_LD3;
                    when U_LD3 => u_b <= signed(rf_q); u_st <= U_FIN;
                    when U_WB =>
                        if u_ir(27 downto 20) /= x"00" then rf_we <= '1'; end if;
                        rf_wa <= unsigned(u_ir(27 downto 20));
                        if v_op /= 9 then u_n <= rf_wd(31); end if;
                        u_pc <= u_pc + 1; r_uc_a <= u_pc + 1; u_st <= U_F1;
                    when U_SYNC =>
                        if w_sdone = '1' then u_pc <= u_pc + 1; r_uc_a <= u_pc + 1; u_st <= U_F1; end if;
                    when others => u_st <= U_IDLE;
                end case;
            end if;
        end if;
    end process p_cpu;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid rise, field-latched width)
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
    -- Bus sequencer.  After SYNC: the constants pass (RF 0x98..0xA0: ring and pixel constants), then the
    -- seed pass (per edge: seed, line step (not added), pixel step), then the patch computer resumes.  After
    -- each pixel run: per edge wrap (added), line step (added), pixel step.  The word for an address issued
    -- at clock t is on c_bus at t + 3.
    ---------------------------------------------------------------------------
    p_bus : process(clk)
        variable v_ld, v_st : std_logic;
    begin
        if rising_edge(clk) then
            w_sdone <= '0';
            b_rund <= b_run;
            bus_own_d <= w_cst or w_run;
            -- pixel run: a fixed count per line
            if g_start = '1' then
                b_run <= '1'; b_cnt <= (others => '0');
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = g_wm1 then b_run <= '0'; end if;
            end if;
            -- the patch computer / writer must not start a read in the last clocks of the run
            if b_run = '1' and b_cnt + 8 >= g_wm1 then w_pre <= '1';
            elsif w_run = '1' then w_pre <= '0'; end if;

            if u_syncp = '1' then
                w_cst <= '1'; w_ci <= (others => '0');
            elsif w_cst = '1' then
                w_ci <= w_ci + 1;
                if w_ci = 8 then w_cst <= '0'; w_run <= '1'; w_seed <= '1'; w_k <= "000"; w_p <= "00"; end if;
            elsif b_rund = '1' and b_run = '0' then
                w_run <= '1'; w_seed <= '0'; w_k <= "000"; w_p <= "00";
            elsif w_run = '1' then
                w_p <= w_p + 1;
                if w_p = 3 then
                    w_k <= w_k + 1;
                    if w_k = 4 then w_run <= '0'; end if;
                end if;
            end if;
            -- address
            if w_cst = '1' then
                w_ra <= x"98" + resize(w_ci, 8);
            else
                case w_p is
                    when "00"   => if w_seed = '1' then w_ra <= "100" & w_k & "11"; else w_ra <= "100" & w_k & "00"; end if;
                    when "01"   => w_ra <= "100" & w_k & "01";
                    when others => w_ra <= "100" & w_k & "10";
                end case;
            end if;
            w_c1 <= w_cst; w_ci1 <= w_ci; w_v1 <= w_run; w_k1 <= w_k; w_p1 <= w_p; w_sd1 <= w_seed;
            w_c2 <= w_c1;  w_ci2 <= w_ci1; w_v2 <= w_v1; w_k2 <= w_k1; w_p2 <= w_p1; w_sd2 <= w_sd1;
            w_c3 <= w_c2;  w_ci3 <= w_ci2; w_v3 <= w_v2; w_k3 <= w_k2; w_p3 <= w_p2; w_sd3 <= w_sd2;
            c_bus <= rf_q;
            if w_v3 = '1' and w_sd3 = '1' and w_k3 = 4 and w_p3 = 3 then w_sdone <= '1'; end if;
            -- ring: f += D >> 8, D += DD along the line; at the line start f = F0(line), D = D0, and the
            -- line DDA steps: F0 += G >> 8, G += GG.  The constants pass loads F0, G for line 0.
            if w_c3 = '1' and w_ci3 = 2 then
                r_f0 <= signed(c_bus);
            elsif g_st2 = '1' then
                r_f0 <= r_f0 + shift_right(r_g, 8);
            end if;
            if w_c3 = '1' and w_ci3 = 3 then
                r_g <= signed(c_bus);
            elsif g_st2 = '1' then
                r_g <= r_g + c_gg;
            end if;
            -- (the ring runs two clocks behind the edges: its f for a pixel is ready at stage 3)
            g_st1 <= g_start; g_st2 <= g_st1; b_run1 <= b_run; b_run2 <= b_run1;
            if g_st2 = '1' then
                r_f <= r_f0;
            elsif b_run2 = '1' then
                r_f <= r_f + shift_right(r_d, 8);
            end if;
            -- D: the fifth bus edge (seed D0, wrap -N DD each line, operand DD), stepped with the ring's run
            v_ld := '0'; v_st := '0';
            if w_v3 = '1' and w_k3 = 4 then
                case w_p3 is
                    when "00"   => v_ld := '1';
                    when "01"   => v_ld := '1'; v_st := '1';
                    when "10"   => v_ld := '1'; v_st := not w_sd3;
                    when others => null;
                end case;
            end if;
            if v_ld = '1' then c_dd <= signed(c_bus); end if;
            if w_v3 = '1' and w_sd3 = '1' and w_k3 = 4 and w_p3 = "00" then
                r_d <= (others => '0');
            elsif b_run2 = '1' or v_st = '1' then
                r_d <= r_d + c_dd;
            end if;
            -- the other constants
            if w_c3 = '1' then
                case to_integer(w_ci3) is
                    when 4      => c_gg <= signed(c_bus);
                    when 5      => c_ha <= signed(c_bus(15 downto 0));
                    when 6      => c_hb <= signed(c_bus(15 downto 0));
                    when 7      => c_gap <= unsigned(c_bus(15 downto 0));
                    when 8      => c_cg <= unsigned(c_bus(11 downto 0));
                    when others => null;
                end case;
            end if;
            -- edge accumulators and operand registers
            for k in 0 to 3 loop
                v_ld := '0'; v_st := '0';
                if w_v3 = '1' and to_integer(w_k3) = k then
                    case w_p3 is
                        when "00"   => v_ld := '1';
                        when "01"   => v_ld := '1'; v_st := '1';
                        when "10"   => v_ld := '1'; v_st := not w_sd3;
                        when others => null;
                    end case;
                end if;
                if v_ld = '1' then b_op(k) <= signed(c_bus(25 downto 0)); end if;
                if w_v3 = '1' and w_sd3 = '1' and to_integer(w_k3) = k and w_p3 = "00" then
                    b_acc(k) <= (others => '0');
                elsif b_run = '1' or v_st = '1' then
                    b_acc(k) <= b_acc(k) + b_op(k);
                end if;
            end loop;
        end if;
    end process p_bus;
    bus_hold <= w_cst or w_run or w_pre or u_syncp;

    ---------------------------------------------------------------------------
    -- Table writer: after END, rebuilds the three tube tables in the back bank, one entry at a time.  One
    -- serial multiplier (P = [acc | B], 16 steps of hi += A, >>> 1: P = acc + A B).  Per entry: the core
    -- ramp, a gaussian body, the exponential halo and wash (the EXP2 ROM), lc = (vb body + vh halo + vw
    -- wash) >> 10 (shapes as Q10), then per tube R/G/B = (Vc core + colour lc) >> 10.  Banks swap at the zone
    -- start when it has finished (it is aborted otherwise).  Constants are read from the register file
    -- (waiting while the bus sequencer owns its read port).
    ---------------------------------------------------------------------------
    p_wr : process(clk)
        variable v_s : signed(39 downto 0);
        variable v_d : signed(17 downto 0);
        variable v_e : unsigned(15 downto 0);
        variable v_h : signed(24 downto 0);
    begin
        if rising_edge(clk) then
            t_we <= "000";
            if s_zgo = '1' then
                if wr_done = '1' then wr_front <= not wr_front; end if;
                wr_run <= '0'; wr_done <= '0';
            elsif u_done = '1' then
                wr_run <= '1'; wr_i <= (others => '0'); wr_s <= (others => '0'); wr_ph <= (others => '0');
            elsif wr_run = '1' then
                case wr_ph is
                    when "100" =>                                   -- multiply: 16 steps
                        if m_p(0) = '1' then v_h := resize(m_p(39 downto 16), 25) + resize(m_a, 25);
                        else v_h := resize(m_p(39 downto 16), 25); end if;
                        m_p <= v_h & m_p(15 downto 1);
                        wr_cnt <= wr_cnt - 1;
                        if wr_cnt = 1 then wr_ph <= "101"; end if;
                    when "110" =>                                   -- exp2neg: E[(-e) & 255] >> ceil(e / 256)
                        if wr_cnt = 0 then wr_ph <= "101";
                        else w_x <= '0' & w_x(15 downto 1); wr_cnt <= wr_cnt - 1; end if;
                    when "000" =>                                   -- step setup: the constant's address
                        case to_integer(wr_s) is
                            when 0      => wr_ra <= x"B0";          -- Pc
                            when 1      => wr_ra <= x"B1";          -- icw
                            when 2      => wr_ra <= x"B2";          -- kb
                            when 5      => wr_ra <= x"B4";          -- vb
                            when 6      => wr_ra <= x"B3";          -- kh
                            when 8      => wr_ra <= x"B5";          -- vh
                            when 11     => wr_ra <= x"B6";          -- vw
                            when 12 | 16 | 20 => wr_ra <= x"B8" + resize(shift_right(wr_s - 12, 2), 8);
                            when 13 | 17 | 21 => wr_ra <= x"BB" + resize(shift_right(wr_s - 13, 2), 8);
                            when 14 | 18 | 22 => wr_ra <= x"BE" + resize(shift_right(wr_s - 14, 2), 8);
                            when others => wr_ra <= x"C1" + resize(shift_right(wr_s - 15, 2), 8);
                        end case;
                        if wr_s = 0 then
                            -- the entry's distance (Q4): bin centres of the compander
                            if wr_i < 128 then w_dl <= resize(wr_i(6 downto 0) & "100", 15);
                            elsif wr_i < 192 then w_dl <= to_unsigned(1040, 15) + shift_left(resize(wr_i(5 downto 0), 15), 5);
                            elsif wr_i < 224 then w_dl <= to_unsigned(3136, 15) + shift_left(resize(wr_i(4 downto 0), 15), 7);
                            else w_dl <= to_unsigned(7424, 15) + shift_left(resize(wr_i(4 downto 0), 15), 9); end if;
                        end if;
                        if bus_hold = '0' then wr_ph <= "001"; end if;
                    when "001" => wr_ph <= "010";                   -- RAM address -> data
                    when "010" =>                                   -- data -> fabric
                        w_qf <= unsigned(rf_q(15 downto 0)); wr_ph <= "011";
                    when "011" =>                                   -- operands
                        wr_cnt <= to_unsigned(16, 5);
                        wr_ph <= "100";
                        case to_integer(wr_s) is
                            when 0 =>                                   -- (Pc)
                                w_pc <= w_qf; wr_ph <= "101";
                            when 1 =>                                   -- core: (Pc - d) icw
                                v_d := signed(resize(w_pc, 18)) - signed(resize(w_dl, 18));
                                m_a <= v_d; m_p <= x"000000" & signed(w_qf);
                            when 2 | 6 =>                               -- tb = d kb >> 16 / eh = d kh >> 12
                                m_a <= signed(resize(w_dl, 18)); m_p <= x"000000" & signed(w_qf);
                            when 3 =>                                   -- eb = tb^2
                                m_a <= signed(resize(w_e(7 downto 0), 18)); m_p <= x"000000" & signed(resize(w_e(7 downto 0), 16));
                            when 9 =>                                   -- ew = d KW >> 12
                                m_a <= signed(resize(w_dl, 18)); m_p <= x"000000" & to_signed(338, 16);
                            when 4 | 7 | 10 =>                          -- exp2neg of the exponent in w_e
                                w_exa <= unsigned(not w_e(7 downto 0)) + 1;
                                wr_cnt <= (others => '0'); wr_ph <= "111";
                            when 5 =>                                   -- lc = vb body ...
                                m_a <= signed(resize(w_qf, 18)); m_p <= x"000000" & signed(resize(w_x(15 downto 5), 16));
                            when 8 =>                                   -- ... + vh halo
                                m_a <= signed(resize(w_qf, 18)); m_p <= w_lc & signed(resize(w_x(15 downto 5), 16));
                            when 11 =>                                  -- ... + vw wash
                                m_a <= signed(resize(w_qf, 18)); m_p <= w_lc & signed(resize(w_x(15 downto 5), 16));
                            when 12 | 16 | 20 =>                        -- Vc core
                                m_a <= signed(resize(w_qf, 18)); m_p <= x"000000" & signed(resize(w_core, 16));
                            when others =>                              -- vcc + colour lc
                                m_a <= signed(resize(w_qf, 18)); m_p <= w_vcc & w_lc(15 downto 0);
                        end case;
                    when "111" =>                                   -- EXP2 ROM: address -> data -> shift count
                        if wr_cnt = 0 then
                            wr_cnt <= "00001";
                        else
                            w_x <= unsigned(r_ex_q);
                            v_e := w_e;
                            if v_e(15 downto 12) /= "0000" then
                                w_x <= (others => '0'); wr_cnt <= (others => '0');
                            elsif v_e(7 downto 0) = 0 then
                                wr_cnt <= v_e(12 downto 8);
                            else
                                wr_cnt <= v_e(12 downto 8) + 1;
                            end if;
                            wr_ph <= "110";
                        end if;
                    when others =>                                  -- "101": results
                        v_s := m_p;
                        case to_integer(wr_s) is
                            when 1 =>
                                -- core = clamp((P >> 8) + 512, 0, 1024), by bit tests (t = P >> 8 in [-512, 511]:
                                -- t + 512 = t(9:0) with bit 9 flipped)
                                if m_p(39) = '1' and m_p(38 downto 17) /= "1111111111111111111111" then
                                    w_core <= (others => '0');
                                elsif m_p(39) = '0' and m_p(38 downto 17) /= "0000000000000000000000" then
                                    w_core <= to_unsigned(1024, 11);
                                else
                                    w_core <= '0' & (unsigned(m_p(17 downto 8)) xor "1000000000");
                                end if;
                            when 2 =>                                   -- tb (clamped 255) -> w_e(7:0)
                                if m_p(39 downto 24) /= 0 then w_e <= x"00FF";
                                else w_e <= x"00" & unsigned(m_p(23 downto 16)); end if;
                            when 3 => w_e <= unsigned(m_p(15 downto 0));
                            when 6 | 9 =>                               -- eh / ew (clamped 65535)
                                if m_p(39 downto 28) /= 0 then w_e <= (others => '1');
                                else w_e <= unsigned(m_p(27 downto 12)); end if;
                            when 4 | 7 | 10 => null;                    -- shapes in w_x
                            when 5 | 8 => w_lc <= m_p(23 downto 0);
                            when 11 => w_lc <= resize(shift_right(m_p(23 downto 0), 10), 24);
                            when 12 | 16 | 20 => w_vcc <= m_p(23 downto 0);
                            when 13 | 17 | 21 =>
                                if m_p(39 downto 20) /= 0 then t_wd(9 downto 0) <= (others => '1');
                                else t_wd(9 downto 0) <= std_logic_vector(m_p(19 downto 10)); end if;
                            when 14 | 18 | 22 =>
                                if m_p(39 downto 20) /= 0 then t_wd(19 downto 10) <= (others => '1');
                                else t_wd(19 downto 10) <= std_logic_vector(m_p(19 downto 10)); end if;
                            when others =>                              -- 15 / 19 / 23: blue, write the entry
                                t_wa <= (not wr_front) & wr_i;
                                if m_p(39 downto 20) /= 0 then t_wd(29 downto 20) <= (others => '1');
                                else t_wd(29 downto 20) <= std_logic_vector(m_p(19 downto 10)); end if;
                                t_wd(31 downto 30) <= "00";
                                if wr_s = 15 then t_we <= "001"; elsif wr_s = 19 then t_we <= "010"; else t_we <= "100"; end if;
                        end case;
                        if wr_s = 23 then
                            wr_s <= (others => '0');
                            wr_i <= wr_i + 1;
                            if wr_i = 255 then wr_run <= '0'; wr_done <= '1'; end if;
                        else
                            wr_s <= wr_s + 1;
                        end if;
                        wr_ph <= "000";
                end case;
            end if;
        end if;
    end process p_wr;

    ---------------------------------------------------------------------------
    -- Source side: Osc lattice; camera cells (128 / 64 px: the mean luma), gain (one serial multiply per
    -- cell), vertical one-pole lowpass in the cell store (the first line of the field seeds it)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : signed(9 downto 0);
        variable v_sum : signed(15 downto 0);
        variable v_m : signed(15 downto 0);
        variable v_n : signed(16 downto 0);
        variable v_h : unsigned(12 downto 0);
        variable v_c : signed(16 downto 0);
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
            s_osum <= resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);

            -- the cell sums
            if s_src = '1' then
                v_l := signed(resize(s_osum, 10)) - 127;
            elsif unsigned(data_in.y(9 downto 2)) > 16 then
                v_l := signed(resize(unsigned(data_in.y(9 downto 2)), 10)) - 16;
            else
                v_l := (others => '0');
            end if;
            k_go <= '0';
            if data_in.avid = '1' then
                if k_px = 0 then v_sum := resize(v_l, 16); else v_sum := k_sum + resize(v_l, 16); end if;
                k_sum <= v_sum;
                if (s_cs = '1' and k_px = 127) or (s_cs = '0' and k_px = 63) then
                    k_px <= (others => '0');
                    -- the cell's mean: |sum| >> 7 or 6 (sign kept)
                    if s_cs = '1' then v_m := shift_right(v_sum, 7); else v_m := shift_right(v_sum, 6); end if;
                    k_neg <= v_m(15);
                    if v_m(15) = '1' then v_m := -v_m; end if;
                    k_p <= resize(unsigned(v_m(8 downto 0)), 21);
                    k_lc <= k_cell; k_go <= '1'; k_cell <= k_cell + 1;
                else
                    k_px <= k_px + 1;
                end if;
            else
                k_px <= (others => '0'); k_cell <= (others => '0');
            end if;
            if s_zone = '1' then k_first <= '1';
            elsif s_avq = '1' and data_in.avid = '0' then k_first <= '0'; end if;

            -- the cell job: cam = gain x mean >> 8 (Q4 px), F = F + ((cam << 3) - F) >> kv (Q7), store
            cr_we <= '0';
            if k_go = '1' then
                k_cnt <= to_unsigned(9, 4); k_fl <= k_first; k_job <= '1';
            elsif k_job = '1' then
                if k_cnt /= 0 then
                    if k_p(0) = '1' then v_h := resize(k_p(20 downto 9), 13) + resize(c_cg, 13);
                    else v_h := resize(k_p(20 downto 9), 13); end if;
                    k_p <= v_h & k_p(8 downto 1);
                    k_cnt <= k_cnt - 1;
                    k_two <= '0';
                elsif k_two = '0' then
                    -- target (cam << 3, Q7): gain x |mean| >> 8, signed
                    v_c := signed(resize(k_p(20 downto 8), 17));
                    if k_neg = '1' then v_c := -v_c; end if;
                    k_tgt <= shift_left(v_c, 3);
                    k_two <= '1';
                else
                    if k_fl = '1' then v_n := k_tgt;
                    else v_n := resize(k_old, 17) + shift_right(k_tgt - resize(k_old, 17), to_integer(s_kv)); end if;
                    cr_wd <= std_logic_vector(v_n(15 downto 0)); cr_wa <= resize(k_lc, 8); cr_we <= '1';
                    k_job <= '0';
                end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Camera read side (generated pixels).  At each cell start c the store's F(c+1) is read; segment c runs
    -- from cell start c + 4 px for one cell, v stepping by (F(c+1) - F(c)) / cell (exact: the segment ends
    -- on F(c+1)).  At mid-cell the old F(c) is read for the source side's lowpass.  cam = v >> 10 (Q4 px).
    ---------------------------------------------------------------------------
    p_camr : process(clk)
        variable v_c1 : unsigned(4 downto 0);
        variable v_df : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            q_rv1 <= '0'; q_rg1 <= '0'; q_ro1 <= '0';
            if g_start = '1' then
                cr_ra <= (others => '0'); q_rg1 <= '1';
                q_px <= (others => '0'); q_cell <= (others => '0');
            elsif b_run = '1' then
                if q_px = 0 then
                    v_c1 := q_cell + 1;
                    if v_c1 < q_nc then cr_ra <= resize(v_c1, 8); q_rv1 <= '1'; q_last <= '0';
                    else q_last <= '1'; end if;
                elsif q_px = 32 then
                    cr_ra <= resize(q_cell, 8); q_ro1 <= '1';
                end if;
                if (s_cs = '1' and q_px = 127) or (s_cs = '0' and q_px = 63) then
                    q_px <= (others => '0'); q_cell <= q_cell + 1;
                else
                    q_px <= q_px + 1;
                end if;
            end if;
            q_rv2 <= q_rv1; q_rv3 <= q_rv2; q_rg2 <= q_rg1; q_rg3 <= q_rg2; q_ro2 <= q_ro1; q_ro3 <= q_ro2;
            q_rq <= cr_q;
            if q_ro3 = '1' then k_old <= signed(q_rq); end if;
            -- arrivals (fabric copy of the store's data)
            if q_rg3 = '1' then
                q_fn <= signed(q_rq);
                q_v <= shift_left(resize(signed(q_rq), 23), 7); q_step <= (others => '0');
            else
                if q_rv3 = '1' then
                    q_fn <= signed(q_rq);
                    v_df := resize(signed(q_rq), 18) - resize(q_fn, 18);
                    if s_cs = '1' then q_step <= v_df; else q_step <= shift_left(v_df, 1); end if;
                elsif b_run = '1' and q_px = 3 and q_last = '1' then
                    q_step <= (others => '0');
                end if;
                if b_run = '1' then q_v <= q_v + resize(q_step, 23); end if;
            end if;
            q_cam <= resize(shift_right(q_v, 10), 12);
        end if;
    end process p_camr;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the accumulators' value for the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u0, v_u1, v_u2, v_w0, v_w1, v_w2 : signed(15 downto 0);
        variable v_x  : signed(16 downto 0);
        variable v_a  : unsigned(15 downto 0);
        variable v_e  : signed(16 downto 0);
        variable v_o  : unsigned(16 downto 0);
        variable v_t  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: the edge functions (Q11.4); the third edge of each figure: triangle -(u0 + u1), hexagon w1 - w0,
            -- square / diamond = the first; |.| for the hexagon / diamond and the square
            v_u0 := b_acc(0)(24 downto 9); v_u1 := b_acc(1)(24 downto 9);
            v_w0 := b_acc(2)(24 downto 9); v_w1 := b_acc(3)(24 downto 9);
            if s_sq = '1' then
                v_u2 := v_u0; v_w2 := v_w0;
            else
                v_x := -(resize(v_u0, 17) + resize(v_u1, 17)); v_u2 := v_x(15 downto 0);
                v_x := resize(v_w1, 17) - resize(v_w0, 17);    v_w2 := v_x(15 downto 0);
            end if;
            if s_sq = '1' then
                p2_a(0) <= signed(f_oabs(v_u0)); p2_a(1) <= signed(f_oabs(v_u1)); p2_a(2) <= signed(f_oabs(v_u2));
            else
                p2_a(0) <= v_u0; p2_a(1) <= v_u1; p2_a(2) <= v_u2;
            end if;
            p2_b(0) <= signed(f_oabs(v_w0)); p2_b(1) <= signed(f_oabs(v_w1)); p2_b(2) <= signed(f_oabs(v_w2));
            -- 3: analog max of three; the apothems pick up the camera term
            p3_ma <= f_max3(p2_a(0), p2_a(1), p2_a(2));
            p3_mb <= f_max3(p2_b(0), p2_b(1), p2_b(2));
            p3_hac <= resize(c_ha, 17) + resize(q_cam, 17);
            p3_hbc <= resize(c_hb, 17) + resize(q_cam, 17);
            -- 4: signed distances (the ring's f arrives here directly; the camera term one pixel later)
            p4_d(0) <= resize(r_f(31 downto 16), 18) - resize(q_cam, 18);
            p4_d(1) <= resize(p3_ma, 18) - resize(p3_hac, 18);
            p4_d(2) <= resize(p3_mb, 18) - resize(p3_hbc, 18);
            for t in 0 to 2 loop
                -- 5: |d|, saturated to 15 bits
                v_a := resize(f_oabs(p4_d(t)), 16);
                if f_oabs(p4_d(t)) > 32767 then p5_a(t) <= (others => '1'); else p5_a(t) <= v_a(14 downto 0); end if;
                -- 6: Tubing Double: | |d| - gap |
                v_e := signed(resize(p5_a(t), 17)) - signed(resize(c_gap, 17));
                v_o := f_oabs(v_e);
                if s_dbl = '1' then p6_a(t) <= v_o(14 downto 0); else p6_a(t) <= p5_a(t); end if;
                -- 7: table index; 8: a register stage (the latency is a multiple of 4)
                p7_i(t) <= f_comp(p6_a(t));
                p8_i(t) <= p7_i(t);
            end loop;
            -- 9: table addresses (front bank & index); 10: the tables' output registers
            t_ra0 <= wr_front & p8_i(0);
            t_ra1 <= wr_front & p8_i(1);
            t_ra2 <= wr_front & p8_i(2);
            -- 11: fabric copies
            p11_0 <= t_q0(29 downto 0); p11_1 <= t_q1(29 downto 0); p11_2 <= t_q2(29 downto 0);
            -- 12: the additive mixer
            p12_r <= resize(unsigned(p11_0(9 downto 0)), 12) + resize(unsigned(p11_1(9 downto 0)), 12) + resize(unsigned(p11_2(9 downto 0)), 12);
            p12_g <= resize(unsigned(p11_0(19 downto 10)), 12) + resize(unsigned(p11_1(19 downto 10)), 12) + resize(unsigned(p11_2(19 downto 10)), 12);
            p12_b <= resize(unsigned(p11_0(29 downto 20)), 12) + resize(unsigned(p11_1(29 downto 20)), 12) + resize(unsigned(p11_2(29 downto 20)), 12);
            -- 13: the shoulder
            p13_r <= f_tone(p12_r); p13_g <= f_tone(p12_g); p13_b <= f_tone(p12_b);
            -- 14, 15: Y' = (5 R + 9 G + 2 B) / 16
            p14_a <= resize(p13_r & "00", 13) + resize(p13_r, 13);
            p14_bc <= resize(p13_g & "000", 13) + resize(p13_g, 13);
            p14_tb2 <= p13_b & '0';
            p14_r <= p13_r; p14_b <= p13_b;
            v_a := resize(p14_a, 16) + resize(p14_bc, 16) + resize(p14_tb2, 16);
            p15_y <= v_a(12 downto 4);
            p15_r <= p14_r; p15_b <= p14_b;
            -- 16 .. 18: Y10 = 64 + 2.28125 Y', U10 = 512 + 1.656 (R - Y'), V10 = 512 + 1.3125 (B - Y'), with the
            -- differences offset by +512 so every shift-add is unsigned (bit-exact: 512 divides by 2, 8, 16, 32)
            p16_dr <= resize(p15_r, 10) + 512 - resize(p15_y, 10);
            p16_db <= resize(p15_b, 10) + 512 - resize(p15_y, 10);
            p16_ya <= resize(p15_y & '0', 11) + resize(p15_y(8 downto 2), 11);
            p16_yb <= p15_y(8 downto 5);
            p17_u1 <= resize(p16_dr, 11) + resize(p16_dr(9 downto 1), 11);
            p17_u2 <= resize(p16_dr(9 downto 3), 11) + resize(p16_dr(9 downto 5), 11);
            p17_v1 <= resize(p16_db, 11) + resize(p16_db(9 downto 2), 11);
            p17_v2 <= resize(p16_db(9 downto 4), 11);
            p17_y <= resize(p16_ya, 10) + resize(p16_yb, 10) + 64;
            v_t := p17_u1 + p17_u2 - 336;
            p18_u <= v_t(9 downto 0);
            v_t := p17_v1 + p17_v2 - 160;
            p18_v <= v_t(9 downto 0);
            p18_y <= p17_y;
            -- 19: compose; black outside the generated avid
            if s_gav_sr(16) = '0' then
                s_y19 <= C_KY; s_u19 <= C_KC; s_v19 <= C_KC;
            else
                s_y19 <= p18_y; s_u19 <= p18_u; s_v19 <= p18_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (each two adjacent 8-sums;
    -- the HD HDMI bar fix), luma 13/16 and the blanking gate.  The running sums snap back to black once 8
    -- blank samples are in (resync).  The 8-sample delays are block-RAM rings.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            o_wp <= o_wp + 1;
            oa_f <= oa_q; ob_f <= ob_q;
            s_ho <= s_ho(2 downto 0) & ob_f(31); s_vo <= s_vo(2 downto 0) & ob_f(30);
            -- box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KC & "000"; s_bav <= C_KC & "000";
            else
                s_bay <= s_bay + s_y19 - unsigned(oa_f(9 downto 0));
                s_bau <= s_bau + s_u19 - unsigned(oa_f(19 downto 10));
                s_bav <= s_bav + s_v19 - unsigned(oa_f(29 downto 20));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= C_KY; s_u21 <= C_KC; s_v21 <= C_KC;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KC & "000"; s_bbv <= C_KC & "000";
            else
                s_bby <= s_bby + s_y21 - unsigned(ob_f(9 downto 0));
                s_bbu <= s_bbu + s_u21 - unsigned(ob_f(19 downto 10));
                s_bbv <= s_bbv + s_v21 - unsigned(ob_f(29 downto 20));
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
    data_out.hsync_n <= s_ho(3);
    data_out.vsync_n <= s_vo(3);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture argon;
