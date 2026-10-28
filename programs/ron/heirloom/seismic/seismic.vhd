-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Seismic v0.5.0 -- Heirloom #44 (2026-10-27 rebuild), from WIN_20230610_00_10_54.
--
-- The photo: one FM horizontal oscillator (a chirp: its frequency is a horizontal ramp folded at
-- x ~1100-1350, so the stripes run from 120 px down to 42 px and back out) against a clipped
-- vertical triangle (five ribbons, 205 lines), keyed per channel:
--   R  the carrier key (white stripes on the cyan ribbons, wide humps at the sides), its "off"
--      triangle narrowing into the gaps;
--   G  the ribbon key (soft tubes) and, in the window x ~390..1480, the gap lobes with one dark
--      magenta cusp per carrier cycle;
--   B  a fold of an elliptical distance (the big inverted-V envelope), blended into the gap colour.
-- The gap colour is a 1-D colourizer of the UNWRAPPED carrier phase (cyan, white, yellow, lime,
-- mint, cyan, yellow-green, black: C_TAB*), gated by a per-strip segment table (C_SEG): the gap
-- plateau is vertically flat, so every x transition there is a hard-edged block.
-- The frame is 24 strips of 45 lines; each strip's sweep (shift, fold, left ramp, ribbon width)
-- steps through an 8-strip staircase (time samples of the patch).  Speed rolls the staircase.
--
--   K1 Rows     ribbons 0.5x .. 2x (5.3 = the photo)
--   K2 Sweep    chirp rate 0.42x .. 2.34x
--   K3 Lobes    gap layer: 0 = black gaps, 50 % = the photo, max = the window spans the frame
--   K4 Speed    bipolar (still at the centre): the staircase rolls, the stripes drift, the ribbons roll
--   K5 Width    ribbon width 0.55x .. 1.45x
--   K6 Melt     how far the camera (or Osc) bends the stripes and the ribbons
--   S7 Blocks   On / Off      S8 Fold  Fold / Ramp      S9 Palette  Photo / Swap (R <-> B)
--   S10 Steps   Staircase / Shuffle      S11 Source  Video / Osc
--   P12 QUAKE   0 = calm aligned strips (no blocks); 50 % = the photo; 100 % = triple shifts + slips
--
-- Engine: a small microcoded machine (one 32-bit accumulator, a 64 x 32 register file and the
-- microcode in block RAM, one barrel shifter, one Booth serial multiplier) runs the field program
-- (vblank) and, at every line start, the line program that computes the NEXT line's carrier
-- anchor, fold constants, ribbon phase and envelope offset.  The microcode is assembled by
-- seismic.py (FIELD / LINE) into seismic_pkg.vhd (C_UC).
--
-- Virtual coordinates: x spans the width (1920 virtual px), y = 1080 virtual lines (timing-ID
-- steps).  Generated avid; output conditioning ([1 2 1] luma, [1 2 2 2 2 2 2 2 1]/16 chroma, luma
-- 13/16); blanking gate.  Latency 24 clocks in every mode.  Colours: u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.seismic_pkg.all;
use work.all;

architecture seismic of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

    function f_sx(t : std_logic_vector(3 downto 0)) return unsigned is   -- virtual px per pixel, Q12
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(10923, 14);
            when 5 | 6 | 14     => return to_unsigned(6144, 14);
            when others         => return to_unsigned(4096, 14);
        end case;
    end function;
    function f_sy(t : std_logic_vector(3 downto 0)) return unsigned is   -- virtual lines per line, Q12
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(18204, 15);
            when 4          => return to_unsigned(9216, 15);
            when 8          => return to_unsigned(15360, 15);
            when 12         => return to_unsigned(7680, 15);
            when 5 | 6 | 14 => return to_unsigned(6144, 15);
            when 1 | 2 | 10 => return to_unsigned(8192, 15);
            when others     => return to_unsigned(4096, 15);
        end case;
    end function;
    -- 16-bit phase -> triangle 0..255
    function f_tri(p : unsigned(15 downto 0)) return unsigned is
    begin
        if p(15) = '1' then return p(14 downto 7); else return not p(14 downto 7); end if;
    end function;
    function f_clip8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;
    -- |wrap(d)| for an 8-bit phase difference -> 0..128
    function f_wd(d : unsigned(7 downto 0)) return unsigned is
    begin
        if d(7) = '1' then return resize(256 - resize(d, 9), 8); else return d; end if;
    end function;
    -- arithmetic right shift by 0..15 of a 47-bit value, low 32 bits (largest step first)
    function f_bar(x : signed(46 downto 0); s : unsigned(3 downto 0)) return signed is
        variable v8 : signed(38 downto 0);
        variable v4 : signed(34 downto 0);
        variable v2 : signed(32 downto 0);
        variable v1 : signed(31 downto 0);
    begin
        if s(3) = '1' then v8 := x(46 downto 8); else v8 := x(38 downto 0); end if;
        if s(2) = '1' then v4 := v8(38 downto 4); else v4 := v8(34 downto 0); end if;
        if s(1) = '1' then v2 := v4(34 downto 2); else v2 := v4(32 downto 0); end if;
        if s(0) = '1' then v1 := v2(32 downto 1); else v1 := v2(31 downto 0); end if;
        return v1;
    end function;

    -- microcode op codes (seismic.py OPS)
    constant OP_LDI  : integer := 1;
    constant OP_ADDI : integer := 2;
    constant OP_LD   : integer := 3;
    constant OP_ADD  : integer := 4;
    constant OP_SUB  : integer := 5;
    constant OP_ST   : integer := 6;
    constant OP_OUT  : integer := 7;
    constant OP_IN   : integer := 8;
    constant OP_MAX0 : integer := 9;
    constant OP_MUL  : integer := 10;
    constant OP_MULI : integer := 11;
    constant OP_MULP : integer := 12;
    constant OP_END  : integer := 15;

    -- ROM read registers (registered reads + a second fabric register)
    signal r_ta : unsigned(6 downto 0) := (others => '0');
    signal r_tr, r_tg, r_tb, r_tr2, r_tg2, r_tb2 : std_logic_vector(7 downto 0) := (others => '0');
    signal r_ha : unsigned(7 downto 0) := (others => '0');
    signal r_hq, r_hq2 : std_logic_vector(7 downto 0) := (others => '0');
    signal r_sa : unsigned(7 downto 0) := (others => '0');
    signal r_sq, r_sq2 : std_logic_vector(15 downto 0) := (others => '0');

    -- timing
    signal t_sx : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal t_sy : unsigned(14 downto 0) := to_unsigned(4096, 15);

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K1, K2, K3, K5, P12 (Q.3)
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (others => to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- field values (knobs, drift phases)
    signal f_x1, f_x2, f_x3, f_x5, f_p : unsigned(9 downto 0);    -- slices of the glide bank
    signal f_v     : signed(9 downto 0) := (others => '0');       -- speed, deadbanded
    signal f_m6    : unsigned(5 downto 0) := (others => '0');
    signal f_blk, f_ramp, f_swap, f_shuf, f_osc : std_logic := '0';
    signal f_tau   : unsigned(20 downto 0) := (others => '0');    -- Q16 strips, mod 24
    signal f_thc   : unsigned(23 downto 0) := (others => '0');    -- carrier drift, Q24 cycles
    signal f_ox0   : unsigned(15 downto 0) := (others => '0');
    -- field values written by the engine (OUT)
    signal f_s1r   : unsigned(13 downto 0) := to_unsigned(436, 14);
    signal f_flr   : unsigned(15 downto 0) := to_unsigned(1887, 16);     -- Q20
    signal f_frr   : unsigned(15 downto 0) := to_unsigned(1677, 16);     -- Q20
    signal f_ginc  : signed(19 downto 0) := (others => '0');             -- fold term step, Q28
    signal f_dx0   : signed(21 downto 0) := (others => '0');
    signal f_k2x   : unsigned(12 downto 0) := (others => '0');
    signal f_kxo   : unsigned(8 downto 0) := to_unsigned(93, 9);
    signal f_lr1, f_lr2, f_lr3, f_lb1, f_lb2, f_lb3 : unsigned(5 downto 0) := (others => '0');
    signal f_xlw, f_xrw : unsigned(10 downto 0) := (others => '0');
    signal f_lob   : unsigned(7 downto 0) := to_unsigned(255, 8);

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- engine control: kicks, line / strip state (hardwired), next-line (n_*) and current-line (c_*) values
    signal e_kf, e_kl, e_go : std_logic := '0';
    signal e_mode : std_logic_vector(1 downto 0) := "00";    -- 01 field program, 10 line 0, 00 line
    signal e_s1, e_s2, e_s3 : std_logic := '0';
    signal e_avf : std_logic := '0';
    signal e_V    : unsigned(23 downto 0) := (others => '0');   -- Q12 virtual lines
    signal e_k    : unsigned(4 downto 0) := (others => '0');
    signal e_sb   : unsigned(14 downto 0) := to_unsigned(878, 15);
    signal e_a    : signed(12 downto 0) := (others => '0');
    signal e_kr   : unsigned(4 downto 0) := (others => '0');
    signal e_hsh  : signed(7 downto 0) := (others => '0');
    signal e_w0   : std_logic := '0';
    signal c_l0  : signed(23 downto 0) := (others => '0');      -- Q24
    signal c_ca  : signed(20 downto 0) := (others => '0');      -- Q20
    signal c_g0  : signed(29 downto 0) := (others => '0');      -- Q28
    signal c_xa  : signed(15 downto 0) := (others => '0');
    signal c_xf  : signed(16 downto 0) := (others => '0');
    signal c_psi : signed(27 downto 0) := (others => '0');      -- Q20
    signal c_phv : unsigned(11 downto 0) := (others => '0');    -- 1/4096 cycle
    signal c_hgw : unsigned(8 downto 0) := to_unsigned(112, 9);
    signal c_cy  : signed(30 downto 0) := (others => '0');
    signal c_oy  : unsigned(15 downto 0) := (others => '0');
    signal n_sga : unsigned(7 downto 0) := (others => '0');

    -- engine pipeline: fetch (u_q, block RAM) -> d1 (RF read) -> d2 (barrel, port) -> x (execute)
    signal u_pc  : unsigned(7 downto 0) := (others => '0');
    signal u_ent : unsigned(7 downto 0) := (others => '0');
    signal u_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal u_d1, u_d2, u_x : std_logic_vector(31 downto 0) := (others => '0');
    signal u_v1, u_v2, u_v3, u_vx, u_run, u_done : std_logic := '0';
    signal u_xmul : std_logic := '0';
    signal u_adv, u_stall : std_logic;
    type t_rf is array(0 to 63) of std_logic_vector(31 downto 0);
    signal r_rf : t_rf := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_rf : signal is true;
    signal rf_q, rf_q2 : std_logic_vector(31 downto 0) := (others => '0');
    signal rf_we : std_logic := '0';
    signal b_q   : signed(31 downto 0) := (others => '0');
    signal p_q   : signed(18 downto 0) := (others => '0');
    signal e_acc : signed(31 downto 0) := (others => '0');
    signal m_st  : unsigned(1 downto 0) := (others => '0');
    signal m_h   : signed(25 downto 0) := (others => '0');
    signal m_l   : std_logic_vector(18 downto 0) := (others => '0');
    signal m_qb  : std_logic := '0';
    signal m_cnt : unsigned(4 downto 0) := (others => '0');

    -- segment walker
    signal w_cur, w_nxt, w_first : std_logic_vector(15 downto 0) := (others => '0');
    signal w_ld  : std_logic_vector(2 downto 0) := (others => '0');
    signal w_pre : std_logic_vector(2 downto 0) := (others => '0');

    -- beam
    signal b_x8  : unsigned(19 downto 0) := (others => '0');   -- Q8 virtual px
    signal b_L   : signed(23 downto 0) := (others => '0');     -- Q24 cycles per real px
    signal b_X2  : signed(30 downto 0) := (others => '0');     -- Q8 px^2 (relative to x = 0)
    signal b_dx  : signed(23 downto 0) := (others => '0');
    signal b_ox  : unsigned(15 downto 0) := (others => '0');
    signal b_psi : signed(27 downto 0) := (others => '0');     -- Q20
    signal b_g   : signed(29 downto 0) := (others => '0');     -- fold term, Q28
    signal b_on  : std_logic := '0';
    signal b_st  : std_logic_vector(4 downto 0) := (others => '0');
    signal bq_L, br_L : signed(19 downto 0) := (others => '0');       -- Q20 from here on
    signal bq_fa, br_fa : signed(20 downto 0) := (others => '0');
    signal bq_fc, br_fc : signed(21 downto 0) := (others => '0');
    signal bt_f  : signed(21 downto 0) := (others => '0');
    signal bq_sa, bq_sc, br_sa, br_sc : std_logic := '0';
    signal b_lv  : unsigned(5 downto 0) := (others => '0');

    -- video
    signal v_y1, v_d2 : unsigned(9 downto 0) := (others => '0');
    signal v_l3 : unsigned(7 downto 0) := (others => '0');
    signal s_avg : unsigned(15 downto 0) := (others => '0');
    signal s_acnt : unsigned(5 downto 0) := (others => '0');
    signal s_adf : signed(16 downto 0) := (others => '0');
    signal s_adv : std_logic := '0';

    -- pixel path
    signal p1_psi, p2_psi : signed(15 downto 0) := (others => '0');   -- Q8 cycles
    signal p1_E, p2_E : signed(14 downto 0) := (others => '0');       -- E / 65536
    signal p1_dl : signed(7 downto 0) := (others => '0');
    signal p1_L8 : unsigned(7 downto 0) := (others => '0');
    signal p1_lv, p2_lv, p3_lv, p4_lv, p5_lv, p6_lv, p7_lv : unsigned(5 downto 0) := (others => '0');
    signal p2_pm : signed(13 downto 0) := (others => '0');
    signal p3_psim : signed(15 downto 0) := (others => '0');
    signal p3_m1, p3_m2 : signed(11 downto 0) := (others => '0');
    signal p3_aE : unsigned(14 downto 0) := (others => '0');
    signal p3_En : std_logic := '0';
    signal p4_ps : unsigned(7 downto 0) := (others => '0');
    signal p4_phv : unsigned(11 downto 0) := (others => '0');
    signal p4_be, p5_be, p6_be, p7_be, p8_be : unsigned(7 downto 0) := (others => '0');
    signal p5_ps, p6_ps, p7_ps : unsigned(7 downto 0) := (others => '0');
    signal p5_w, p6_w, p7_w : unsigned(7 downto 0) := (others => '0');
    signal p6_hr : unsigned(5 downto 0) := (others => '0');
    signal p6_cu : unsigned(6 downto 0) := (others => '0');
    signal p7_cr : unsigned(7 downto 0) := (others => '0');
    signal p8_hr : unsigned(6 downto 0) := (others => '0');
    signal p8_dr : unsigned(7 downto 0) := (others => '0');
    signal p6_zc : signed(9 downto 0) := (others => '0');
    signal p6_ab : unsigned(6 downto 0) := (others => '0');
    signal p7_hr : unsigned(6 downto 0) := (others => '0');
    signal p7_z  : signed(11 downto 0) := (others => '0');
    signal p7_sg : unsigned(7 downto 0) := (others => '0');
    signal p7_ab : unsigned(7 downto 0) := (others => '0');
    signal p7_iw, p8_iw, p9_iw, p10_iw : std_logic := '0';
    signal p8_core, p8_mr, p8_mb, p8_sg, p8_ab, p8_d2 : unsigned(7 downto 0) := (others => '0');
    signal p8_hd : unsigned(7 downto 0) := (others => '0');
    signal p8_lr, p8_lg, p8_lb : unsigned(5 downto 0) := (others => '0');
    signal p8_cr, p8_cg, p8_cb : unsigned(7 downto 0) := (others => '0');
    signal p9_gr, p9_gg : unsigned(13 downto 0) := (others => '0');
    signal p9_bm : unsigned(13 downto 0) := (others => '0');
    signal p9_abp : signed(13 downto 0) := (others => '0');
    signal p9_mr, p9_rk, p9_core, p9_sg, p9_be, p9_cu, p9_c2 : unsigned(7 downto 0) := (others => '0');
    signal p9_gon, p10_gon : std_logic := '0';
    signal p9_lbm, p10_lbm : unsigned(5 downto 0) := (others => '0');
    signal p10_x, p10_bw, p10_bm, p10_core, p10_cu : unsigned(7 downto 0) := (others => '0');
    signal p10_rk : unsigned(5 downto 0) := (others => '0');
    signal p10_gw : unsigned(12 downto 0) := (others => '0');
    signal p11_rp, p11_b2 : unsigned(13 downto 0) := (others => '0');
    signal p11_g0, p11_bm : unsigned(7 downto 0) := (others => '0');
    signal p11_cu : unsigned(7 downto 0) := (others => '1');
    signal p12_r, p12_b, p12_g0 : unsigned(7 downto 0) := (others => '0');
    signal p13_t1 : unsigned(13 downto 0) := (others => '0');
    signal p13_t2 : unsigned(11 downto 0) := (others => '0');
    signal p13_r, p13_b, p14_r, p14_b : unsigned(7 downto 0) := (others => '0');
    signal p14_y : unsigned(14 downto 0) := (others => '0');
    signal p15_y : unsigned(10 downto 0) := (others => '0');
    signal p15_cr, p15_cb : signed(10 downto 0) := (others => '0');
    signal p16_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    -- chroma 8 clocks back: a 256 x 16 ring in block RAM (U, V at 8 bits)
    type t_cring is array(0 to 255) of std_logic_vector(15 downto 0);
    signal r_cr : t_cring := (others => x"8080");
    attribute no_rw_check of r_cr : signal is true;
    signal c_wp : unsigned(7 downto 0) := (others => '0');
    signal c_rp : unsigned(7 downto 0) := to_unsigned(250, 8);
    signal c_rq, c_rq2 : std_logic_vector(15 downto 0) := x"8080";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y17, s_y18, s_y19, s_y20, s_y21, s_y22x, s_yf22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel
    -- hsync / vsync / field delayed C_LAT clocks through a 256 x 4 block-RAM ring
    type t_sring is array(0 to 255) of std_logic_vector(3 downto 0);
    signal r_sr : t_sring := (others => "1111");
    attribute no_rw_check of r_sr : signal is true;
    signal y_wp : unsigned(7 downto 0) := (others => '0');
    signal y_rp : unsigned(7 downto 0) := to_unsigned(256 - (C_LAT - 2), 8);
    signal y_rq, y_rq2 : std_logic_vector(3 downto 0) := "1111";

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered reads; a second fabric register in the pixel path)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_tr <= C_TABR(to_integer('0' & r_ta)); r_tg <= C_TABG(to_integer('0' & r_ta)); r_tb <= C_TABB(to_integer('0' & r_ta));
            r_tr2 <= r_tr; r_tg2 <= r_tg; r_tb2 <= r_tb;
            r_hq <= C_HD(to_integer(r_ha)); r_hq2 <= r_hq;
            r_sq <= C_SEG(to_integer(r_sa)); r_sq2 <= r_sq;
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls: vblank zone, width, glides, field latches and drift phases
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_t  : unsigned(21 downto 0);
    begin
        if rising_edge(clk) then
            t_sx <= f_sx(registers_in(8)(3 downto 0));
            t_sy <= f_sy(registers_in(8)(3 downto 0));

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
            elsif s_zc /= 1023 then
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
            g_wm1 <= s_w - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K4 Speed: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 8 then f_v <= resize(v_x - 8, 10);
                elsif v_x < -8 then f_v <= resize(v_x + 8, 10);
                else f_v <= (others => '0'); end if;
                f_m6   <= unsigned(registers_in(5)(9 downto 4));
                f_blk  <= not registers_in(6)(0);
                f_ramp <= registers_in(6)(1);
                f_swap <= registers_in(6)(2);
                f_shuf <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- drift: the staircase rolls (mod 24 strips), the stripes drift, the Osc moves
                v_t := resize(f_tau, 22) + unsigned(resize(shift_left(resize(f_v, 22), 3) + shift_left(resize(f_v, 22), 2) + resize(f_v, 22), 22));
                if f_v < 0 then
                    if v_t(21) = '1' then v_t := v_t + to_unsigned(24 * 65536, 22); end if;
                else
                    if v_t >= 24 * 65536 then v_t := v_t - to_unsigned(24 * 65536, 22); end if;
                end if;
                f_tau <= v_t(20 downto 0);
                f_thc <= f_thc + unsigned(resize(shift_left(resize(f_v, 24), 8) + shift_left(resize(f_v, 24), 6) + shift_left(resize(f_v, 24), 3), 24));
                f_ox0 <= f_ox0 + 262;
            end if;

            -- glides: ease 1/8 of the gap per field (deadband 4); one channel per four zone clocks (4..23)
            if s_zone = '1' and s_zc >= 4 and s_zc < 24 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when others => s_g(4) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
        end if;
    end process p_ctrl;

    f_x1 <= s_g(0)(12 downto 3); f_x2 <= s_g(1)(12 downto 3); f_x3 <= s_g(2)(12 downto 3);
    f_x5 <= s_g(3)(12 downto 3); f_p <= s_g(4)(12 downto 3);

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 2/3 clocks after the source's avid rise, field-latched width)
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
            g_avd <= g_avid;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Engine control (hardwired): the field program at vblank zone clock 64, then line 0's values
    -- (LCOMP), then the line program (copy line 0, compute line 1).  After every line the line
    -- program copies the next line's values to the beam (hblank) and computes the line after.
    -- Strip index, strip phase (a), slip hash and segment row of the line being computed.
    ---------------------------------------------------------------------------
    p_kick : process(clk)
        variable v_m : unsigned(11 downto 0);
        variable v_k : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 63 then e_kf <= '1'; else e_kf <= '0'; end if;
            -- line kick: 11 clocks after the generated avid falls (the last pixel has left the beam);
            -- the copy prologue is done ~40 clocks after the fall (fits SD's 138-clock hblank)
            e_avf <= s_gav_sr(9);
            if s_gav_sr(9) = '0' and e_avf = '1' then e_kl <= '1'; else e_kl <= '0'; end if;
            e_go <= '0'; e_s1 <= '0'; e_w0 <= '0';
            if e_kf = '1' then
                u_ent <= (others => '0'); e_go <= '1'; e_mode <= "01";
            elsif e_kl = '1' or (u_done = '1' and e_mode = "10") then
                -- copy the next line's values to the beam, then compute the line after
                if e_kl = '0' then e_w0 <= '1'; end if;      -- line 0: segment prefetch
                e_V <= e_V + resize(t_sy, 24);
                u_ent <= to_unsigned(U_LINE, 8); e_go <= '1'; e_mode <= "00"; e_s1 <= '1';
            elsif u_done = '1' and e_mode = "01" then
                e_V <= (others => '0'); e_k <= (others => '0'); e_sb <= to_unsigned(878, 15);
                u_ent <= to_unsigned(U_LCOMP, 8); e_go <= '1'; e_mode <= "10"; e_s1 <= '1';
            end if;
            -- strip index of the next line
            e_s2 <= e_s1; e_s3 <= e_s2;
            if e_s1 = '1' then
                if e_V(22 downto 8) >= e_sb and e_k < 23 then
                    e_k <= e_k + 1; e_sb <= e_sb + 720;
                end if;
            end if;
            -- strip phase mu (staircase or shuffle, rolled by Speed) -> a = mu - 3.5 strips; slip hash
            if e_s2 = '1' then
                if f_shuf = '1' then v_m := unsigned(C_SHUF(to_integer(e_k)));
                else v_m := resize(e_k + 2, 3) & "000000000"; end if;
                v_m := v_m + f_tau(18 downto 7);
                e_a <= signed(resize(v_m, 13)) - 1792;
                v_k := resize(e_k, 6) + 24 - resize(f_tau(20 downto 16), 6);
                if v_k >= 24 then v_k := v_k - 24; end if;
                if v_k >= 24 then v_k := v_k - 24; end if;
                e_kr <= v_k(4 downto 0);
                e_hsh <= signed(C_HSH(to_integer(e_k)));
            end if;
            if e_s3 = '1' then n_sga <= e_kr & "000"; end if;
        end if;
    end process p_kick;

    ---------------------------------------------------------------------------
    -- Engine (microcoded; seismic.py assembles C_UC).  Four stages, one instruction per clock:
    --   fetch  u_q  <= C_UC(pc)                 (block RAM)
    --   d1     u_d1 <= u_q;  rf_q <= RF(reg)     (block RAM)
    --   d2     u_d2 <= u_d1; rf_q2 <= rf_q
    --   b      u_x  <= u_d2; b_q <= RF >>> sh (inverted for SUB); p_q <= port
    --   x      execute on e_acc.  A multiply (MUL/MULI/MULP) stalls the pipeline: Booth radix 2,
    --          19 steps, then the product goes through the barrel (>>> sh) into b_q and e_acc.
    ---------------------------------------------------------------------------
    u_stall <= '1' when u_vx = '1' and u_xmul = '1' and m_st /= 3 else '0';
    u_adv   <= not u_stall;

    p_ucode : process(clk)
    begin
        if rising_edge(clk) then
            if u_adv = '1' then
                u_q  <= C_UC(to_integer(u_pc));
                rf_q <= r_rf(to_integer(unsigned(u_q(23 downto 18))));
            end if;
            if rf_we = '1' then
                r_rf(to_integer(unsigned(u_x(23 downto 18)))) <= std_logic_vector(e_acc);
            end if;
        end if;
    end process p_ucode;
    rf_we <= '1' when u_vx = '1' and to_integer(unsigned(u_x(31 downto 28))) = OP_ST else '0';

    p_eng : process(clk)
        variable v_op  : integer range 0 to 15;
        variable v_bi  : signed(46 downto 0);
        variable v_sh  : unsigned(3 downto 0);
        variable v_b   : signed(31 downto 0);
        variable v_ao  : std_logic_vector(31 downto 0);
        variable v_s33 : unsigned(32 downto 0);
        variable v_ma  : std_logic_vector(25 downto 0);
        variable v_mo  : std_logic_vector(25 downto 0);
        variable v_s27 : unsigned(26 downto 0);
        variable v_hh  : std_logic_vector(25 downto 0);
        variable v_ms, v_me : std_logic;
        variable v_p44 : std_logic_vector(43 downto 0);
    begin
        if rising_edge(clk) then
            u_done <= '0';
            if e_go = '1' then
                u_pc <= u_ent; u_run <= '1';
                u_v1 <= '0'; u_v2 <= '0'; u_v3 <= '0'; u_vx <= '0'; m_st <= "00";
            else
                ------------------------------------------------ advance the pipeline
                if u_adv = '1' then
                    if u_run = '1' then u_pc <= u_pc + 1; end if;
                    u_v1 <= u_run; u_v2 <= u_v1; u_v3 <= u_v2; u_vx <= u_v3;
                    u_d1 <= u_q; u_d2 <= u_d1; u_x <= u_d2;
                    rf_q2 <= rf_q;
                    v_op := to_integer(unsigned(u_d2(31 downto 28)));
                    if v_op = OP_MUL or v_op = OP_MULI or v_op = OP_MULP then u_xmul <= '1'; else u_xmul <= '0'; end if;
                    -- stage b: barrel (RF operand; shift 0 for MUL), port value
                    if v_op = OP_MUL then v_sh := (others => '0'); else v_sh := unsigned(u_d2(27 downto 24)); end if;
                    v_b := f_bar(resize(signed(rf_q2), 47), v_sh);
                    if v_op = OP_SUB then v_b := not v_b; end if;
                    b_q <= v_b;
                    case to_integer(unsigned(u_d2(23 downto 18))) is
                        when 0 => p_q <= signed(resize(f_x1, 19));
                        when 1 => p_q <= signed(resize(f_x2, 19));
                        when 2 => p_q <= signed(resize(f_x3, 19));
                        when 3 => p_q <= signed(resize(f_x5, 19));
                        when 4 => p_q <= signed(resize(f_p, 19));
                        when 5 => p_q <= signed(resize(t_sx, 19));
                        when 6 => p_q <= resize(f_v, 19);
                        when 7 => p_q <= signed(resize(e_V(22 downto 8), 19));
                        when 8 => p_q <= resize(e_a, 19);
                        when others => p_q <= resize(e_hsh, 19);
                    end case;
                end if;
                if m_st = 2 then   -- multiply finished: the product through the barrel
                    v_p44 := std_logic_vector(m_h(24 downto 0)) & m_l;
                    v_bi := resize(signed(v_p44), 47);
                    b_q <= f_bar(v_bi, unsigned(u_x(27 downto 24)));
                end if;

                ------------------------------------------------ execute
                if u_vx = '1' then
                    v_op := to_integer(unsigned(u_x(31 downto 28)));
                    if v_op = OP_ADDI then v_ao := std_logic_vector(resize(signed(u_x(27 downto 0)), 32));
                    else v_ao := std_logic_vector(b_q); end if;
                    if v_op = OP_SUB then v_ms := '1'; else v_ms := '0'; end if;
                    v_s33 := unsigned(std_logic_vector(e_acc) & '1') + unsigned(v_ao & v_ms);
                    case v_op is
                        when OP_LDI => e_acc <= resize(signed(u_x(27 downto 0)), 32);
                        when OP_ADDI | OP_ADD | OP_SUB => e_acc <= signed(v_s33(32 downto 1));
                        when OP_LD => e_acc <= b_q;
                        when OP_IN => e_acc <= resize(p_q, 32);
                        when OP_MAX0 => if e_acc(31) = '1' then e_acc <= (others => '0'); end if;
                        when OP_OUT =>
                            case to_integer(unsigned(u_x(23 downto 18))) is
                                when 0  => f_s1r <= unsigned(e_acc(13 downto 0));
                                when 1  => f_flr <= unsigned(e_acc(19 downto 4));
                                when 2  => f_frr <= unsigned(e_acc(19 downto 4));
                                when 3  => f_dx0 <= e_acc(21 downto 0);
                                when 4  => f_k2x <= unsigned(e_acc(12 downto 0));
                                when 5  => f_kxo <= unsigned(e_acc(8 downto 0));
                                when 6  => f_lr1 <= unsigned(e_acc(8 downto 3)); f_lb1 <= unsigned(e_acc(7 downto 2));
                                when 7  => f_lr2 <= unsigned(e_acc(8 downto 3)); f_lb2 <= unsigned(e_acc(7 downto 2));
                                when 8  =>
                                    f_lr3 <= unsigned(e_acc(8 downto 3));
                                    if e_acc(8) = '1' then f_lb3 <= (others => '1'); else f_lb3 <= unsigned(e_acc(7 downto 2)); end if;
                                when 9  => f_xlw <= unsigned(e_acc(10 downto 0));
                                when 10 => f_xrw <= unsigned(e_acc(10 downto 0));
                                when 11 => f_lob <= unsigned(e_acc(7 downto 0));
                                when 12 => c_l0  <= e_acc(23 downto 0);
                                when 13 => c_ca  <= e_acc(24 downto 4);
                                when 14 => c_g0  <= e_acc(29 downto 0);
                                when 15 => c_xa  <= e_acc(15 downto 0);
                                when 16 => c_xf  <= e_acc(16 downto 0);
                                when 17 => c_psi <= e_acc(31 downto 4);
                                when 18 => c_phv <= unsigned(e_acc(23 downto 12));
                                when 19 => c_hgw <= unsigned(e_acc(8 downto 0));
                                when 20 => c_cy  <= e_acc(30 downto 0);
                                when 21 => c_oy  <= unsigned(e_acc(15 downto 0));
                                when others => f_ginc <= e_acc(19 downto 0);
                            end case;
                        when OP_MUL | OP_MULI | OP_MULP =>
                            case to_integer(m_st) is
                                when 0 =>
                                    m_h <= (others => '0'); m_qb <= '0'; m_cnt <= (others => '0'); m_st <= "01";
                                    if v_op = OP_MUL then m_l <= std_logic_vector(b_q(18 downto 0));
                                    elsif v_op = OP_MULI then m_l <= u_x(18 downto 0);
                                    else m_l <= std_logic_vector(p_q); end if;
                                when 1 =>
                                    -- Booth radix 2: (l0, q) = 10 -> h - a, 01 -> h + a
                                    v_ms := m_l(0) and not m_qb;
                                    v_me := m_l(0) xor m_qb;
                                    v_ma := std_logic_vector(resize(e_acc(24 downto 0), 26));
                                    if v_me = '0' then v_mo := (others => '0');
                                    elsif v_ms = '1' then v_mo := not v_ma;
                                    else v_mo := v_ma; end if;
                                    v_s27 := unsigned(std_logic_vector(m_h) & '1') + unsigned(v_mo & v_ms);
                                    v_hh := std_logic_vector(v_s27(26 downto 1));
                                    m_h <= signed(v_hh(25) & v_hh(25 downto 1));
                                    m_l <= v_hh(0) & m_l(18 downto 1);
                                    m_qb <= m_l(0);
                                    m_cnt <= m_cnt + 1;
                                    if m_cnt = 18 then m_st <= "10"; end if;
                                when 2 =>
                                    m_st <= "11";
                                when others =>
                                    e_acc <= b_q; m_st <= "00";
                            end case;
                        when OP_END =>
                            u_run <= '0'; u_v1 <= '0'; u_v2 <= '0'; u_v3 <= '0'; u_vx <= '0'; u_done <= '1';
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Video luma (L8 = 0..255 over black .. white)
    ---------------------------------------------------------------------------
    p_vid : process(clk)
        variable v_l : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_y1 <= unsigned(data_in.y);
            if v_y1 > 64 then v_d2 <= v_y1 - 64; else v_d2 <= (others => '0'); end if;
            v_l := resize(v_d2(9 downto 2), 11) + resize(v_d2(9 downto 5), 11) + resize(v_d2(9 downto 6), 11);
            if v_l > 255 then v_l3 <= (others => '1'); else v_l3 <= v_l(7 downto 0); end if;
        end if;
    end process p_vid;

    ---------------------------------------------------------------------------
    -- Beam: the FM carrier (2nd-order DDA with the three-segment frequency law), the Benv
    -- squared-distance DDA, the Osc phase and the segment walker
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_x : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- stage A: x, the linear frequency L (Q24), the fold term g (Q28), the squared distance, Osc phase
            if g_start = '1' then
                b_x8 <= (others => '0');
                b_L  <= c_l0;
                b_g  <= c_g0;
                b_X2 <= (others => '0');
                b_dx <= resize(f_dx0, 24);
                b_ox <= f_ox0;
            else
                b_x8 <= b_x8 + resize(t_sx(13 downto 4), 20);
                b_L  <= b_L + signed('0' & f_s1r);
                b_g  <= b_g + resize(f_ginc, 30);
                b_X2 <= b_X2 + resize(b_dx, 31);
                b_dx <= b_dx + signed('0' & f_k2x);
                b_ox <= b_ox + f_kxo;
            end if;
            -- stage B (Q20): segment flags; FA = L + cA; FC = g >= 0 ? g : -g/2
            if signed('0' & b_x8(19 downto 4)) < resize(c_xa, 17) then bq_sa <= '1'; else bq_sa <= '0'; end if;
            if signed('0' & b_x8(19 downto 4)) >= c_xf and f_ramp = '0' then bq_sc <= '1'; else bq_sc <= '0'; end if;
            bq_L  <= b_L(23 downto 4);
            bq_fa <= resize(b_L(23 downto 4), 21) + c_ca;
            if b_g(29) = '0' then bq_fc <= resize(b_g(29 downto 8), 22); else bq_fc <= resize(shift_right(-b_g, 9), 22); end if;
            -- stage C: floors
            if bq_fa < signed('0' & f_flr) then br_fa <= signed(resize('0' & f_flr, 21)); else br_fa <= bq_fa; end if;
            if bq_fc < signed('0' & f_frr) then br_fc <= signed(resize('0' & f_frr, 22)); else br_fc <= bq_fc; end if;
            br_L <= bq_L; br_sa <= bq_sa; br_sc <= bq_sc;
            -- stage D: select
            if br_sa = '1' then bt_f <= resize(br_fa, 22);
            elsif br_sc = '1' then bt_f <= br_fc;
            else bt_f <= resize(br_L, 22); end if;
            -- phase (Q20): starts when the first valid F arrives (the first pixels repeat psi0)
            b_st <= b_st(3 downto 0) & g_start;
            if g_start = '1' then
                b_psi <= c_psi; b_on <= '0';
            elsif b_st(2) = '1' then
                b_on <= '1';
            elsif b_on = '1' then
                b_psi <= b_psi + resize(bt_f, 28);
            end if;

            -- segment walker (2-px units): prefetch the next line's first two entries in hblank
            -- (w_first = entry 0; r_sa left on entry 1), then one entry per advance: on an advance
            -- w_nxt takes the entry at r_sa (stable for 3 clocks) and r_sa steps on
            w_ld <= w_ld(1 downto 0) & '0';
            w_pre <= w_pre(1 downto 0) & '0';
            if (g_avid = '0' and g_avd = '1') or e_w0 = '1' then   -- hblank / after line 0's program
                r_sa <= n_sga; w_pre(0) <= '1';
            elsif w_pre(0) = '1' then
                r_sa <= r_sa + 1;
            end if;
            if w_pre(2) = '1' then w_first <= r_sq2; end if;
            if g_start = '1' then
                w_cur <= w_first; w_nxt <= r_sq2; r_sa <= r_sa + 1; w_ld(0) <= '1';
            elsif g_avid = '1' then
                v_x := b_x8(19 downto 9);
                if v_x >= unsigned('0' & w_cur(15 downto 6)) and w_ld(2 downto 0) = "000" then
                    w_cur <= w_nxt; w_nxt <= r_sq2; r_sa <= r_sa + 1; w_ld(0) <= '1';
                end if;
            end if;
            -- gap levels (2 bits each: R, G, B), plain window when blocks are off, widen bands
            v_x := b_x8(19 downto 9);
            if f_blk = '1' and f_p >= 128 then
                b_lv <= unsigned(w_cur(5 downto 0));
            elsif v_x >= 195 and v_x < 740 then
                b_lv <= "111111";
            else
                b_lv <= "000000";
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = b_psi)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : signed(9 downto 0);
        variable v_s   : signed(12 downto 0);
        variable v_u   : unsigned(8 downto 0);
        variable v_w   : unsigned(8 downto 0);
        variable v_t   : signed(13 downto 0);
        variable v_lv  : unsigned(5 downto 0);
        variable v_x   : unsigned(10 downto 0);
        variable v_r, v_gg, v_bb : unsigned(15 downto 0);
        variable v_cs  : signed(17 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_be  : unsigned(14 downto 0);
        variable v_8   : unsigned(7 downto 0);
        variable v_ix  : signed(9 downto 0);
        variable v_dd  : signed(8 downto 0);
        variable v_e   : signed(30 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: source luma (or Osc), dl; Benv distance; widen bands
            if f_osc = '1' then
                v_u := resize(f_tri(b_ox), 9) + resize(f_tri(c_oy), 9);
                p1_L8 <= v_u(8 downto 1);
            else
                p1_L8 <= v_l3;
            end if;
            v_dd := signed(resize(p1_L8, 9)) - signed(resize(s_avg(15 downto 8), 9));
            p1_dl <= v_dd(8 downto 1);
            p1_psi <= b_psi(27 downto 12);
            v_e := b_X2 + c_cy;
            p1_E <= v_e(30 downto 16);
            v_lv := b_lv; v_x := b_x8(19 downto 9);
            if (v_x >= shift_right(f_xlw, 1) and v_x < 195) or (v_x >= 740 and v_x < shift_right(f_xrw, 1)) then
                if v_lv(5 downto 4) = "00" then v_lv(5 downto 4) := "10"; end if;
                if v_lv(3 downto 2) = "00" then v_lv(3 downto 2) := "10"; end if;
                if v_lv(1 downto 0) = "00" then v_lv(1 downto 0) := "10"; end if;
            end if;
            p1_lv <= v_lv;
            -- running mean of the source (one sample in 64 active pixels): avg += (L << 8 - avg) / 1024
            s_adv <= '0';
            if s_gav_sr(0) = '1' then
                s_acnt <= s_acnt + 1;
                if s_acnt = 0 then s_adv <= '1'; end if;
            end if;
            s_adf <= signed(resize(p1_L8 & x"00", 17)) - signed(resize(s_avg, 17));
            if s_adv = '1' then
                s_avg <= unsigned(signed(s_avg) + resize(shift_right(s_adf, 10), 16));
            end if;

            -- 2: melt product (dl/2 x Melt/16)
            p2_pm <= resize(p1_dl * signed('0' & f_m6), 14);
            p2_psi <= p1_psi; p2_E <= p1_E; p2_lv <= p1_lv;

            -- 3: melt into the phase (2 cycles at full) and the vertical (0.585 cycle); |E|
            p3_psim <= p2_psi + resize(shift_right(p2_pm, 4), 16);
            p3_m1 <= resize(shift_right(p2_pm, 2), 12) + resize(shift_right(p2_pm, 5), 12);
            p3_m2 <= resize(shift_right(p2_pm, 6), 12) - resize(shift_right(p2_pm, 9), 12);
            if p2_E(14) = '1' then p3_aE <= unsigned(-p2_E); p3_En <= '1';
            else p3_aE <= unsigned(p2_E); p3_En <= '0'; end if;
            p3_lv <= p2_lv;

            -- 4: colourizer address, carrier phase byte, vertical phase, Benv
            v_ix := resize(shift_right(p3_psim, 6), 10) + 32;
            if v_ix < 0 then r_ta <= (others => '0'); elsif v_ix > 127 then r_ta <= (others => '1');
            else r_ta <= unsigned(v_ix(6 downto 0)); end if;
            p4_ps <= unsigned(p3_psim(7 downto 0)) + f_thc(23 downto 16);
            p4_phv <= c_phv + unsigned(p3_m1 + p3_m2);
            if p3_En = '1' then v_be := shift_right(p3_aE, 3) - shift_right(p3_aE, 7);
            else v_be := shift_right(p3_aE, 3) - shift_right(p3_aE, 6); end if;
            if v_be > 255 then p4_be <= (others => '1'); else p4_be <= v_be(7 downto 0); end if;
            p4_lv <= p3_lv;

            -- 5: W (0 ribbon centre .. 255 gap centre); cusp ROM address
            if p4_phv(11) = '1' then v_w := 511 - resize(p4_phv(11 downto 3), 9); else v_w := resize(p4_phv(11 downto 3), 9); end if;
            if v_w > 255 then p5_w <= (others => '1'); else p5_w <= v_w(7 downto 0); end if;
            p5_ps <= p4_ps; p5_be <= p4_be; p5_lv <= p4_lv;

            -- 6: R off-triangle half-width and centre lean; ribbon key distance; B blend weight
            r_ha <= p5_w;
            if p5_w >= 169 then p6_hr <= (others => '0');
            elsif p5_w <= 118 then p6_hr <= to_unsigned(51, 6);
            else p6_hr <= resize(169 - p5_w, 6); end if;
            if p5_w <= 115 then p6_cu <= (others => '0');
            elsif p5_w >= 179 then p6_cu <= to_unsigned(64, 7);
            else p6_cu <= resize(p5_w - 115, 7); end if;
            p6_zc <= signed('0' & c_hgw) - signed(resize(p5_w, 10));
            if p5_w <= 128 then p6_ab <= (others => '0');
            elsif p5_w >= 205 then p6_ab <= to_unsigned(77, 7);
            else p6_ab <= resize(p5_w - 128, 7); end if;
            p6_w <= p5_w; p6_ps <= p5_ps; p6_be <= p5_be; p6_lv <= p5_lv;

            -- 7: hR, cR, z = 2.84 zc, the gap ramp, aB
            p7_hr <= resize(p6_hr, 7) + resize(shift_right(p6_hr, 3), 7) + resize(shift_right(p6_hr, 4), 7) + resize(shift_right(p6_hr, 6), 7);
            p7_cr <= to_unsigned(108, 8) - resize(p6_cu - shift_right(p6_cu, 3) - shift_right(p6_cu, 4), 8);
            p7_z <= shift_left(resize(p6_zc, 12), 1) + resize(shift_right(p6_zc, 1), 12) + resize(shift_right(p6_zc, 2), 12)
                    + resize(shift_right(p6_zc, 3), 12) - resize(shift_right(p6_zc, 5), 12);
            v_t := shift_left(signed(resize(p6_w, 14)), 3) + shift_left(signed(resize(p6_w, 14)), 1) - 1072;
            p7_sg <= f_clip8(v_t);
            p7_ab <= resize(shift_left(resize(p6_ab, 9), 1) + resize(p6_ab, 9) + resize(shift_right(p6_ab, 2), 9) + resize(shift_right(p6_ab, 4), 9), 8);
            if p6_w >= 102 then p7_iw <= '1'; else p7_iw <= '0'; end if;
            p7_w <= p6_w; p7_ps <= p6_ps; p7_be <= p6_be; p7_lv <= p6_lv;

            -- 8: |ps - cR|, ribbon masks, cusp half-width (+ the Lobes fade), levels, colourizer outputs
            p8_dr <= f_wd(p7_ps - p7_cr); p8_hr <= p7_hr;
            p8_core <= f_clip8(resize(p7_z, 13) + 128);
            p8_mr <= f_clip8(resize(p7_z, 13) + 270);
            p8_mb <= f_clip8(resize(p7_z, 13) + 241);
            if p7_w > 128 then v_u := resize(unsigned(r_hq2), 9) + resize(shift_right(not f_lob, 1), 9);
            else v_u := resize(unsigned(r_hq2), 9); end if;
            if v_u > 128 then p8_hd <= to_unsigned(128, 8); else p8_hd <= v_u(7 downto 0); end if;
            p8_d2 <= f_wd(p7_ps - 26);
            case to_integer(p7_lv(5 downto 4)) is
                when 0 => p8_lr <= (others => '0'); when 1 => p8_lr <= f_lr1; when 2 => p8_lr <= f_lr2; when others => p8_lr <= f_lr3;
            end case;
            case to_integer(p7_lv(3 downto 2)) is
                when 0 => p8_lg <= (others => '0'); when 1 => p8_lg <= f_lr1; when 2 => p8_lg <= f_lr2; when others => p8_lg <= f_lr3;
            end case;
            case to_integer(p7_lv(1 downto 0)) is
                when 0 => p8_lb <= (others => '0'); when 1 => p8_lb <= f_lb1; when 2 => p8_lb <= f_lb2; when others => p8_lb <= f_lb3;
            end case;
            p8_cr <= unsigned(r_tr2); p8_cg <= unsigned(r_tg2); p8_cb <= unsigned(r_tb2);
            p8_sg <= p7_sg; p8_ab <= p7_ab; p8_be <= p7_be; p8_iw <= p7_iw;

            -- 9: gap levels x colourizer (level/32), Benv x mask, the B blend term, cusp terms c1, c2
            p9_gr <= p8_cr * p8_lr;
            p9_gg <= p8_cg * p8_lg;
            p9_bm <= p8_be * p8_mb(7 downto 2);
            p9_abp <= resize(signed('0' & p8_ab(7 downto 3)) * (signed(resize(p8_cb, 9)) - signed(resize(p8_be, 9))), 14);
            v_d := signed(resize(p8_hd, 10)) - signed(resize(p8_d2, 10));
            v_s := shift_left(resize(v_d, 13), 4) + resize(shift_right(v_d, 1), 13) + 128;
            p9_cu <= f_clip8(v_s);
            v_t := shift_left(signed(resize(p8_hd, 14)), 5) + signed(resize(p8_hd, 14));
            p9_c2 <= f_clip8(v_t);
            if p8_lg /= 0 then p9_gon <= '1'; else p9_gon <= '0'; end if;
            p9_lbm <= p8_lb;
            p9_mr <= p8_mr - shift_right(p8_mr, 2) + shift_right(p8_mr, 5);
            v_s := shift_left(signed(resize(p8_dr, 13)) - signed(resize(p8_hr, 13)), 2) + 128;
            p9_rk <= f_clip8(v_s);
            p9_core <= p8_core; p9_sg <= p8_sg; p9_be <= p8_be; p9_iw <= p8_iw;

            -- 10: X = max(0.78 mR, gR in the window); G gap term; B blend; cusp = min(c1, c2)
            if p9_gr(13 downto 5) > 255 then v_8 := (others => '1'); else v_8 := p9_gr(12 downto 5); end if;
            if p9_iw = '1' and v_8 > p9_mr then p10_x <= v_8; else p10_x <= p9_mr; end if;
            if p9_gg(13 downto 5) > 255 then v_8 := (others => '1'); else v_8 := p9_gg(12 downto 5); end if;
            v_u := resize(p9_sg, 9) + 8;
            p10_gw <= v_8 * v_u(8 downto 4);
            v_t := signed(resize(p9_be, 14)) + resize(shift_right(p9_abp, 5), 14);
            p10_bw <= f_clip8(v_t);
            if p9_gon = '0' then p10_cu <= (others => '0');
            elsif p9_c2 < p9_cu then p10_cu <= p9_c2;
            else p10_cu <= p9_cu; end if;
            p10_rk <= p9_rk(7 downto 2);
            p10_bm <= p9_bm(13 downto 6); p10_core <= p9_core; p10_gon <= p9_gon; p10_lbm <= p9_lbm; p10_iw <= p9_iw;

            -- 11: R = Rk x X; G0; B gap term
            p11_rp <= p10_rk * p10_x;
            if p10_gon = '1' and p10_gw(11 downto 4) > p10_core then p11_g0 <= p10_gw(11 downto 4); else p11_g0 <= p10_core; end if;
            p11_cu <= not (p10_cu - shift_right(p10_cu, 2) + shift_right(p10_cu, 5));
            if p10_iw = '1' then p11_b2 <= p10_lbm * p10_bw; else p11_b2 <= (others => '0'); end if;
            p11_bm <= p10_bm;

            -- 12: R; G = min(G0, 1 - cusp) (the cusp darkens G); B = max(Benv mask, gap B)
            p12_r <= p11_rp(13 downto 6);
            if p11_cu < p11_g0 then p12_g0 <= p11_cu; else p12_g0 <= p11_g0; end if;
            if p11_b2(13 downto 6) > p11_bm then p12_b <= p11_b2(13 downto 6); else p12_b <= p11_bm; end if;

            -- 13: RGB -> YUV (S9 Swap exchanges R and B).  64 Yf = 19 R + 38 G + 7 B (BT.601 luma within
            --     1 %), as 18 R + 36 G and 7 B + R + 2 G
            if f_swap = '1' then v_r := resize(p12_b, 16); v_bb := resize(p12_r, 16);
            else v_r := resize(p12_r, 16); v_bb := resize(p12_b, 16); end if;
            v_gg := resize(p12_g0, 16);
            p13_t1 <= resize(shift_left(v_r, 4) + shift_left(v_r, 1), 14) + resize(shift_left(v_gg, 5) + shift_left(v_gg, 2), 14);
            p13_t2 <= resize(shift_left(v_bb, 3) - v_bb, 12) + resize(v_r + shift_left(v_gg, 1), 12);
            p13_r <= v_r(7 downto 0); p13_b <= v_bb(7 downto 0);
            -- 14
            p14_y <= resize(p13_t1, 15) + p13_t2;
            p14_r <= p13_r; p14_b <= p13_b;
            -- 15: Yf in Q2 (0 .. 1020); Y10 - 64 = 4.25 Yf (16/13 luma pre-compensation);
            --     Cr4 = 4 (R - Yf), Cb4 = 4 (B - Yf)
            p15_y  <= resize(p14_y(14 downto 4), 11) + resize(p14_y(14 downto 8), 11);
            p15_cr <= signed(resize(p14_r & "00", 11)) - signed(resize(p14_y(14 downto 4), 11));
            p15_cb <= signed(resize(p14_b & "00", 11)) - signed(resize(p14_y(14 downto 4), 11));
            -- 16: Y (cap 940); U = Cr = 512 + 2.5 (R - Yf); V = Cb = 512 + 2 (B - Yf)
            if p15_y > 876 then p16_y <= to_unsigned(940, 10); else p16_y <= to_unsigned(64, 10) + p15_y(9 downto 0); end if;
            v_cs := resize(shift_right(p15_cr, 1), 18) + resize(shift_right(p15_cr, 3), 18) + 512;
            if v_cs < 0 then p16_u <= (others => '0'); elsif v_cs > 1023 then p16_u <= (others => '1');
            else p16_u <= unsigned(v_cs(9 downto 0)); end if;
            v_cs := resize(shift_right(p15_cb, 1), 18) + 512;
            if v_cs < 0 then p16_v <= (others => '0'); elsif v_cs > 1023 then p16_v <= (others => '1');
            else p16_v <= unsigned(v_cs(9 downto 0)); end if;

            -- 17..22: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums,
            -- luma [1 2 1]; centre at stage 21 -> 22)
            -- (U, V at 8 bits; the sample 8 clocks back comes from the block-RAM ring)
            s_boxu <= s_boxu + p16_u(9 downto 2) - unsigned(c_rq2(15 downto 8));
            s_boxv <= s_boxv + p16_v(9 downto 2) - unsigned(c_rq2(7 downto 0));
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u22 <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v22 <= v_sc(11 downto 2);
            s_y17 <= p16_y; s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19;
            s_y21 <= s_y20; s_y22x <= s_y21;
            v_sy := resize(s_y20, 12) + shift_left(resize(s_y21, 12), 1) + resize(s_y22x, 12);
            s_yf22 <= v_sy(11 downto 2);
            -- 23: luma toward black (white 940 -> 777); blanking gate
            if s_yf22 > 64 then v_dl := s_yf22 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(21) = '1' then
                s_y23 <= s_yf22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
            -- 24
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;
        end if;
    end process p_pix;

    -- syncs: written at y_wp, read C_LAT - 2 entries back (+ the read and fabric registers = C_LAT clocks)
    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            r_sr(to_integer(y_wp)) <= '1' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            y_rq <= r_sr(to_integer(y_rp));
            y_rq2 <= y_rq;
            y_wp <= y_wp + 1; y_rp <= y_rp + 1;
        end if;
    end process p_sring;

    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            r_cr(to_integer(c_wp)) <= std_logic_vector(p16_u(9 downto 2)) & std_logic_vector(p16_v(9 downto 2));
            c_rq <= r_cr(to_integer(c_rp));
            c_rq2 <= c_rq;
            c_wp <= c_wp + 1; c_rp <= c_rp + 1;
        end if;
    end process p_cring;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= y_rq2(0);
    data_out.vsync_n <= y_rq2(1);
    data_out.field_n <= y_rq2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture seismic;
