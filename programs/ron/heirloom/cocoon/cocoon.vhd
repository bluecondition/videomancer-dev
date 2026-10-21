-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cocoon v0.5.0 -- Heirloom #2, rebuilt 2026-10-21 from WIN_20220829_23_30_37.
--
-- The photo: light-gray flanks of posterized camera video; between them a
-- diamond glow (red core -> maroon -> navy); a full-width horizontal band of
-- torn, vividly colourized video; and over it the cocoon: a shoulder-tapered,
-- flat-topped shape filled with a woven texture -- rows of crimson dashes on
-- a pink-red core, orange / yellow arcs in the wings, a crimson ring and a
-- green arc at the right end.  Dotted lines mark the band edges.
--
-- Mechanism (fitted to the photo, see SUMMARY.md):
--   glow   d = |x - 984| + 1.53 |y - 554|, colourized (a signed DDA)
--   field  F = |y - 562| / s + G(1125 + (x - 1125) / s)
--          G(x) = 0.6 soft(1050 - x) + 0.8 soft(x - 1200)  (a ROM)
--   arcs   tri(F / pa); weave = rows of dashes (pitch py) x a 78 px
--          oscillator, odd rows shifted half a period, inside the band
--   key    |dy| < 416 s and x > the shoulder (|dy|), cut at the window
--   video  luma posterized + colourized: gray / olive / navy at the flanks,
--          green / pink / orange in the band, torn sideways per line
--
--   K1 Weave   dash period (78 px at the default)
--   K2 Rows    dash-row pitch (22 lines)
--   K3 Arcs    arc spacing (30 F-units)
--   K4 Glow    the red glow's size
--   K5 Tear    the band's video tear and the texture's line jitter
--   K6 Speed   bipolar: dashes slide, rows scroll, arcs stream (centre still)
--   S7 Mirror  the cocoon points right
--   S8 Palette Fire / Ice (generated colours with U and V exchanged)
--   S9 Band    the torn video band across the screen (default On)
--   S10 Source Video / Osc (folded oscillators replace the camera)
--   S11 Bend   the source luma bends the cocoon's field
--   P12 Cocoon size: a seed in a slit (0) .. the photo (50 %) .. the cocoon
--              bursts and fills the window (100 %)
--
-- Geometry in virtual 1080-line units derived from the MEASURED raster
-- (1080 . 256 / active lines per line, square pixels in HD, 2 px per pixel
-- in SD); the window and band ramps scale with the width.  Generated avid;
-- output conditioning before the blanking gate.  Latency 24 clocks.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cocoon_pkg.all;
use work.all;

architecture cocoon of program_top is
    constant C_LAT : integer := 24;

    -- bipolar triangle of a 16-bit phase: -128 .. 128, +128 at phase 0
    function f_tri8(p : unsigned(15 downto 0)) return signed is
        variable a : unsigned(7 downto 0);
    begin
        if p(15) = '1' then
            a := '0' & p(14 downto 8);
        else
            a := to_unsigned(128, 8) - resize(p(14 downto 8), 8);
        end if;
        return resize(signed(resize(a, 10) sll 1) - 128, 9);
    end function;

    function f_clamp8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;

    -- ---------------------------------------------------------------- RAM ---
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal lb_wa, lb_ra : unsigned(10 downto 0) := (others => '0');
    signal lb_wd : std_logic_vector(7 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_q  : std_logic_vector(7 downto 0) := (others => '0');

    signal g_ra   : unsigned(9 downto 0) := (others => '0');
    signal xp_ra  : unsigned(7 downto 0) := (others => '0');
    signal tex_ra : unsigned(9 downto 0) := (others => '0');
    signal glw_ra : unsigned(7 downto 0) := (others => '0');
    signal vid_ra : unsigned(5 downto 0) := (others => '0');
    signal g_q : std_logic_vector(9 downto 0) := (others => '0');
    signal xp_q : std_logic_vector(15 downto 0) := (others => '0');
    signal t_qy, t_qu, t_qv : std_logic_vector(7 downto 0) := (others => '0');
    signal w_qy, w_qu, w_qv : std_logic_vector(7 downto 0) := (others => '0');
    signal v_qy, v_qu, v_qv : std_logic_vector(7 downto 0) := (others => '0');

    -- -------------------------------------------------- raster / controls ---
    signal s_avq : std_logic := '0';
    signal s_idle : unsigned(10 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_lcnt : unsigned(11 downto 0) := (others => '0');
    signal c_W : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal c_H : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_sd : std_logic := '0';
    type t_g is array (0 to 5) of unsigned(9 downto 0);
    -- glides (K1 K2 K3 K4 K5 P12), starting at the defaults
    signal gl : t_g := (to_unsigned(576, 10), to_unsigned(571, 10), to_unsigned(568, 10),
                        to_unsigned(682, 10), to_unsigned(512, 10), to_unsigned(512, 10));
    signal gl_n : unsigned(4 downto 0) := (others => '0');
    signal gl_c, gl_ch : integer range 0 to 5 := 0;
    signal gl_ph : integer range 0 to 2 := 0;
    signal gl_k, gl_g : unsigned(9 downto 0) := (others => '0');
    signal gl_d, gl_s : signed(10 downto 0) := (others => '0');
    signal gl_lo, gl_hi, gl_mv : std_logic := '0';
    signal c_v : signed(10 downto 0) := (others => '0');            -- speed
    signal c_va : unsigned(10 downto 0) := (others => '0');
    signal c_mir, c_ice, c_band, c_osc, c_bend : std_logic := '0';

    -- ------------------------------------------------- serial mul / div ---
    -- operand selects: one-hot over the sources, registered every clock from
    -- the state number (no enable); the fetch is a flat AND-OR
    signal sel_a : std_logic_vector(20 downto 0) := (others => '0');
    signal sel_b : std_logic_vector(26 downto 0) := (others => '0');
    signal o_sq : std_logic := '0';
    signal o_ng : unsigned(1 downto 0) := (others => '0');
    signal m_go, m_busy, m_n : std_logic := '0';
    signal r_p : unsigned(9 downto 0) := (others => '0');
    signal r_pb : unsigned(9 downto 0) := (others => '0');
    signal m_a, m_acc : unsigned(37 downto 0) := (others => '0');
    signal m_b : unsigned(12 downto 0) := (others => '0');
    signal m_cnt : unsigned(3 downto 0) := (others => '0');
    signal m_res : signed(38 downto 0) := (others => '0');
    signal di_n : unsigned(19 downto 0) := (others => '0');
    signal di_d : unsigned(11 downto 0) := (others => '0');
    signal d_go, d_busy : std_logic := '0';
    signal d_num, d_q : unsigned(19 downto 0) := (others => '0');
    signal d_rem : unsigned(12 downto 0) := (others => '0');
    signal d_cnt : unsigned(4 downto 0) := (others => '0');

    -- ----------------------------------------------------------- sequencer ---
    signal q_st : unsigned(6 downto 0) := (others => '0');
    signal q_wait : unsigned(4 downto 0) := (others => '0');
    signal q_first : std_logic := '0';
    signal r_k : unsigned(9 downto 0) := (others => '0');
    signal r_a : unsigned(22 downto 0) := (others => '0');
    signal r_neg, r_bn : std_logic := '0';
    signal r_b : unsigned(10 downto 0) := (others => '0');
    signal r_s : signed(22 downto 0) := (others => '0');
    signal c_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal c_xs8 : unsigned(11 downto 0) := to_unsigned(512, 12);
    signal c_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal c_xg0 : unsigned(20 downto 0) := (others => '0');
    signal c_xgst : signed(12 downto 0) := to_signed(512, 13);
    signal c_wstep : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal c_incw : unsigned(10 downto 0) := to_unsigned(386, 11);
    signal c_incr : unsigned(12 downto 0) := to_unsigned(1365, 13);
    signal c_inca : unsigned(12 downto 0) := to_unsigned(728, 13);
    signal c_gsc : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal c_tsig : unsigned(5 downto 0) := (others => '0');
    signal c_jsig : unsigned(6 downto 0) := (others => '0');
    signal c_s256 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_invs : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal c_bh, c_top : unsigned(9 downto 0) := (others => '0');
    signal c_kR, c_xcut : unsigned(10 downto 0) := (others => '0');
    signal c_phw, c_pha : unsigned(15 downto 0) := (others => '0');
    signal c_phr : unsigned(16 downto 0) := (others => '0');
    signal c_incwpx : signed(22 downto 0) := (others => '0');
    signal c_incrln : unsigned(16 downto 0) := (others => '0');
    signal c_row0 : unsigned(16 downto 0) := (others => '0');
    signal c_st0 : unsigned(23 downto 0) := (others => '0');
    signal c_xsst : signed(15 downto 0) := (others => '0');
    signal c_gd0 : signed(22 downto 0) := (others => '0');
    signal c_gdst : signed(12 downto 0) := (others => '0');
    signal c_t : unsigned(15 downto 0) := (others => '0');            -- field counter (oscillator time)

    -- --------------------------------------------------------- line engine ---
    signal e_vy : unsigned(19 downto 0) := (others => '0');          -- next line, 1/256
    signal e_yl : unsigned(10 downto 0) := (others => '0');
    signal e_ady, e_agy, e_aby : unsigned(10 downto 0) := (others => '0');
    signal e_dgy : unsigned(11 downto 0) := (others => '0');
    signal e_sft : unsigned(5 downto 0) := (others => '0');
    signal e_R : signed(12 downto 0) := (others => '0');
    signal e_rng : unsigned(31 downto 0) := x"2545F491";
    signal e_run : unsigned(3 downto 0) := (others => '0');
    signal e_big : std_logic := '0';
    signal e_off : signed(10 downto 0) := (others => '0');
    signal e_tear : signed(10 downto 0) := (others => '0');
    signal e_js : signed(6 downto 0) := (others => '0');
    signal e_p : unsigned(15 downto 0) := (others => '0');
    signal n_adys : unsigned(10 downto 0) := (others => '0');
    signal n_dgyg : unsigned(11 downto 0) := (others => '0');
    signal n_inb, n_gate, n_edge, n_dotr, n_keyrow : std_logic := '0';
    signal n_xsh : signed(12 downto 0) := (others => '0');
    signal n_jit : signed(6 downto 0) := (others => '0');
    signal n_rowph : unsigned(16 downto 0) := (others => '0');
    signal n_st : unsigned(23 downto 0) := (others => '0');
    signal n_xs0 : signed(22 downto 0) := (others => '0');
    signal n_rp0 : unsigned(10 downto 0) := (others => '0');
    signal n_xoff : signed(11 downto 0) := (others => '0');
    signal n_o1t, n_p2y, n_p3 : unsigned(15 downto 0) := (others => '0');
    signal l_adys : unsigned(10 downto 0) := (others => '0');
    signal l_dgyg : unsigned(11 downto 0) := (others => '0');
    signal l_inb, l_gate, l_edge, l_dotr, l_keyrow : std_logic := '0';
    signal l_xsh : signed(12 downto 0) := (others => '0');
    signal l_jit : signed(6 downto 0) := (others => '0');
    signal l_rowph : unsigned(16 downto 0) := (others => '0');
    signal l_xoff : signed(11 downto 0) := (others => '0');
    signal l_o1t, l_p2y, l_p3 : unsigned(15 downto 0) := (others => '0');

    -- ------------------------------------------------------ generated avid ---
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- --------------------------------------------------------- source side ---
    signal s_wcol, s_wc1 : unsigned(10 downto 0) := (others => '0');
    signal s_we1 : std_logic := '0';
    signal s_y1 : signed(11 downto 0) := (others => '0');

    -- --------------------------------------------------------------- beam ---
    signal b_xg : unsigned(20 downto 0) := (others => '0');
    signal b_xs : signed(22 downto 0) := (others => '0');
    signal b_xw : unsigned(19 downto 0) := (others => '0');
    signal b_ph : unsigned(23 downto 0) := (others => '0');
    signal b_rp : unsigned(10 downto 0) := (others => '0');
    signal b_xc : unsigned(2 downto 0) := (others => '0');
    signal b_gdl : signed(22 downto 0) := (others => '0');            -- late (stage 9)

    -- --------------------------------------------------------- pixel path ---
    type t_sl is array (natural range <>) of std_logic;
    type t_u5 is array (natural range <>) of unsigned(4 downto 0);
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_u16 is array (natural range <>) of unsigned(15 downto 0);
    type t_s9 is array (natural range <>) of signed(8 downto 0);
    signal p1_xg : unsigned(11 downto 0) := (others => '0');
    signal p1_xj : signed(13 downto 0) := (others => '0');
    signal p1_xw : unsigned(11 downto 0) := (others => '0');
    signal p1_xwt : signed(12 downto 0) := (others => '0');
    signal p_ph : t_u16(1 to 3) := (others => (others => '0'));
    signal p_dot : t_sl(1 to 15) := (others => '0');
    signal p_key : t_sl(2 to 15) := (others => '0');
    signal p_bank : t_sl(2 to 10) := (others => '0');
    signal p2_zz : signed(8 downto 0) := (others => '0');
    signal p2_q1, p2_q2x, p2_q3 : unsigned(15 downto 0) := (others => '0');
    signal p3_g : unsigned(9 downto 0) := (others => '0');
    signal p3_xp : std_logic_vector(15 downto 0) := (others => '0');
    signal p3_rph : unsigned(16 downto 0) := (others => '0');
    signal p3_t1, p3_t2x, p3_t3 : signed(8 downto 0) := (others => '0');
    type t_s4 is array (natural range <>) of signed(3 downto 0);
    signal p_zg : t_s4(3 to 7) := (others => (others => '0'));       -- the fine zigzag (weave harmonic 4)
    signal p_lum : t_u8(3 to 10) := (others => (others => '0'));
    signal p_fr : t_u11(4 to 7) := (others => (others => '0'));
    signal p4_vyf : unsigned(7 downto 0) := (others => '0');
    signal p4_oph : unsigned(15 downto 0) := (others => '0');
    signal p4_tw5 : unsigned(4 downto 0) := (others => '0');
    signal p_k16 : t_u5(4 to 13) := (others => (others => '0'));
    signal p_fl : t_sl(4 to 7) := (others => '0');
    signal p4_q2 : unsigned(15 downto 0) := (others => '0');
    signal p4_t1, p4_t3, p5_t1, p5_t3 : signed(8 downto 0) := (others => '0');
    signal p_ox : t_s9(5 to 9) := (others => (others => '0'));
    signal p_inrow : t_sl(5 to 9) := (others => '0');
    signal p5_t2 : signed(8 downto 0) := (others => '0');
    signal p5_thv, p6_thv : unsigned(8 downto 0) := (others => '0');
    signal p5_tw31, p6_tw31 : std_logic := '0';
    signal p6_lo : unsigned(7 downto 0) := (others => '0');
    signal p7_bend : signed(8 downto 0) := (others => '0');
    signal p7_vshow : std_logic := '0';
    signal p_f : t_u10(8 to 10) := (others => (others => '0'));
    signal p8_ut, p8_uv, p8_full : std_logic := '0';
    signal p_ut, p_uv, p_full : t_sl(9 to 13) := (others => '0');
    signal p9_pr : unsigned(18 downto 0) := (others => '0');
    signal p9_thr : signed(9 downto 0) := (others => '0');
    signal p10_pa : unsigned(15 downto 0) := (others => '0');
    signal p10_dash, p10_gap : std_logic := '0';
    signal p10_dg : unsigned(12 downto 0) := (others => '0');
    signal t13_y, t13_u, t13_v, w13_y, w13_u, w13_v, v13_y, v13_u, v13_v : unsigned(7 downto 0) := (others => '0');
    signal s14_y, s14_u, s14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s14_k : unsigned(4 downto 0) := (others => '0');
    signal s15_y, s15_u, s15_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s16_y, s16_u, s16_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: the chroma 8-clock tap from a 16-entry EBR ring
    type t_cr is array (0 to 255) of std_logic_vector(19 downto 0);
    signal cr_ram : t_cr := (others => "10000000001000000000");
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_wp, cr_ra : unsigned(3 downto 0) := (others => '0');
    signal cr_q, cr_f : std_logic_vector(19 downto 0) := "10000000001000000000";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y17, s_y18, s_y19, s_y20, s_y21, s_y22x, s_yf22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- RAM / ROMs (all read addresses registered; each EBR output register is
    -- followed by a fabric copy before any logic)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            lb_q <= lb(to_integer(lb_ra));
            if lb_we = '1' then lb(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lb;

    -- read at wp - 6: the EBR register plus a fabric copy make the tap 8 clocks
    p_cr : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= cr_ram(to_integer(cr_ra));
            cr_ram(to_integer(cr_wp)) <= std_logic_vector(s16_u) & std_logic_vector(s16_v);
        end if;
    end process p_cr;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            g_q  <= C_G(to_integer(g_ra));
            xp_q <= C_XP(to_integer(xp_ra));
            t_qy <= C_TY(to_integer(tex_ra));
            t_qu <= C_TU(to_integer(tex_ra));
            t_qv <= C_TV(to_integer(tex_ra));
            w_qy <= C_GY(to_integer(glw_ra));
            w_qu <= C_GU(to_integer(glw_ra));
            w_qv <= C_GV(to_integer(glw_ra));
            v_qy <= C_VY(to_integer(vid_ra));
            v_qu <= C_VU(to_integer(vid_ra));
            v_qv <= C_VV(to_integer(vid_ra));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Raster measurement, vblank zone (no avid for 1024 clocks), per-field
    -- latches, knob glides (one shared datapath, two clocks per channel)
    ---------------------------------------------------------------------------
    p_meas : process(clk)
        variable v_k : signed(10 downto 0);
        variable v_d, v_s : signed(10 downto 0);
        variable v_i : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1024 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1024 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            if data_in.avid = '1' and s_avq = '0' and s_lcnt /= 4095 then
                s_lcnt <= s_lcnt + 1;
            end if;
            if registers_in(8)(1 downto 0) = "00" then s_sd <= '1'; else s_sd <= '0'; end if;

            if s_zgo = '1' then
                if s_wmax < 256 then c_W <= to_unsigned(1920, 12); else c_W <= s_wmax; end if;
                if s_lcnt < 200 or s_lcnt > 1100 then c_H <= to_unsigned(1080, 11);
                else c_H <= s_lcnt(10 downto 0); end if;
                s_wmax <= (others => '0');
                s_lcnt <= (others => '0');
                v_k := signed('0' & unsigned(registers_in(5))) - 512;
                if v_k < 8 and v_k > -8 then c_v <= (others => '0'); c_va <= (others => '0');
                else c_v <= v_k; c_va <= unsigned(abs(v_k)); end if;
                c_mir  <= registers_in(6)(0);
                c_ice  <= registers_in(6)(1);
                c_band <= registers_in(6)(2);
                c_osc  <= registers_in(6)(3);
                c_bend <= registers_in(6)(4);
                c_t    <= c_t + 1;
                gl_n   <= to_unsigned(1, 5);
                gl_ch  <= 0;
                gl_ph  <= 0;
            elsif gl_n /= 0 then
                if gl_n = 18 then gl_n <= (others => '0'); else gl_n <= gl_n + 1; end if;
                if gl_ph = 0 then
                    -- fetch: K1..K5 = registers 0..4, then P12
                    v_i := gl_ch;
                    gl_c <= v_i;
                    case v_i is
                        when 0 => gl_k <= unsigned(registers_in(0));
                        when 1 => gl_k <= unsigned(registers_in(1));
                        when 2 => gl_k <= unsigned(registers_in(2));
                        when 3 => gl_k <= unsigned(registers_in(3));
                        when 4 => gl_k <= unsigned(registers_in(4));
                        when others => gl_k <= unsigned(registers_in(7));
                    end case;
                    gl_g <= gl(v_i);
                    gl_ph <= 1;
                elsif gl_ph = 1 then
                    -- difference, its 1/8 step (at least 1), the rail and deadband tests
                    v_d := signed('0' & gl_k) - signed('0' & gl_g);
                    v_s := shift_right(v_d, 3);
                    if v_s = 0 then
                        if v_d > 0 then v_s := to_signed(1, 11); else v_s := to_signed(-1, 11); end if;
                    end if;
                    gl_s <= v_s;
                    if gl_k < 4 then gl_lo <= '1'; else gl_lo <= '0'; end if;
                    if gl_k > 1019 then gl_hi <= '1'; else gl_hi <= '0'; end if;
                    if v_d >= 4 or v_d <= -4 then gl_mv <= '1'; else gl_mv <= '0'; end if;
                    gl_ph <= 2;
                else
                    -- ease 1/8 toward the pot beyond a +-4 deadband, snap at the rails
                    v_s := signed('0' & gl_g) + gl_s;
                    if gl_lo = '1' then
                        gl(gl_c) <= (others => '0');
                    elsif gl_hi = '1' then
                        gl(gl_c) <= (others => '1');
                    elsif gl_mv = '1' then
                        gl(gl_c) <= unsigned(v_s(9 downto 0));
                    end if;
                    gl_ph <= 0;
                    if gl_ch /= 5 then gl_ch <= gl_ch + 1; else gl_ch <= 0; end if;
                end if;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_meas;

    ---------------------------------------------------------------------------
    -- Serial multiplier (13 clocks) and divider (20 clocks)
    ---------------------------------------------------------------------------
    p_opmap : process(clk)
        variable v_a : integer range 0 to 20;
        variable v_b : integer range 0 to 26;
        variable v_q : std_logic;
        variable v_n : integer range 0 to 3;
    begin
        if rising_edge(clk) then
            case to_integer(q_st) is
                when   4 => v_a :=  0; v_b :=  0; v_q := '0'; v_n := 0;
                when   7 => v_a :=  1; v_b :=  0; v_q := '1'; v_n := 0;
                when   8 => v_a :=  6; v_b :=  5; v_q := '0'; v_n := 0;
                when   9 => v_a :=  2; v_b :=  0; v_q := '1'; v_n := 0;
                when  10 => v_a :=  6; v_b :=  6; v_q := '0'; v_n := 0;
                when  11 => v_a :=  3; v_b :=  0; v_q := '1'; v_n := 0;
                when  12 => v_a :=  6; v_b :=  7; v_q := '0'; v_n := 0;
                when  13 => v_a :=  4; v_b :=  8; v_q := '0'; v_n := 0;
                when  14 => v_a :=  5; v_b :=  0; v_q := '1'; v_n := 0;
                when  15 => v_a :=  6; v_b :=  9; v_q := '0'; v_n := 0;
                when  16 => v_a :=  7; v_b := 10; v_q := '0'; v_n := 0;
                when  17 => v_a :=  8; v_b := 11; v_q := '0'; v_n := 0;
                when  20 => v_a :=  9; v_b := 12; v_q := '0'; v_n := 0;
                when  21 => v_a :=  9; v_b := 13; v_q := '0'; v_n := 0;
                when  22 => v_a :=  9; v_b := 14; v_q := '0'; v_n := 0;
                when  23 => v_a :=  9; v_b := 15; v_q := '0'; v_n := 0;
                when  24 => v_a := 10; v_b := 16; v_q := '0'; v_n := 1;
                when  25 => v_a := 10; v_b := 17; v_q := '0'; v_n := 1;
                when  26 => v_a := 10; v_b := 18; v_q := '0'; v_n := 1;
                when  27 => v_a := 11; v_b :=  0; v_q := '0'; v_n := 0;
                when  28 => v_a := 12; v_b := 19; v_q := '0'; v_n := 0;
                when  29 => v_a := 13; v_b := 17; v_q := '0'; v_n := 0;
                when  30 => v_a := 14; v_b := 16; v_q := '0'; v_n := 0;
                when  31 => v_a := 15; v_b := 20; v_q := '0'; v_n := 0;
                when  32 => v_a := 15; v_b := 21; v_q := '0'; v_n := 0;
                when  33 => v_a := 16; v_b := 21; v_q := '0'; v_n := 2;
                when 101 => v_a := 17; v_b := 20; v_q := '0'; v_n := 0;
                when 102 => v_a := 18; v_b := 21; v_q := '0'; v_n := 0;
                when 103 => v_a := 19; v_b :=  0; v_q := '1'; v_n := 0;
                when 106 => v_a := 16; v_b := 23; v_q := '0'; v_n := 2;
                when 112 => v_a := 16; v_b := 24; v_q := '0'; v_n := 2;
                when 118 => v_a := 16; v_b := 25; v_q := '0'; v_n := 2;
                when 119 => v_a := 20; v_b := 26; v_q := '0'; v_n := 3;
                when 121 => v_a := 20; v_b := 16; v_q := '0'; v_n := 3;
                when 122 => v_a := 16; v_b := 20; v_q := '0'; v_n := 2;
                when others => v_a := 0; v_b := 0; v_q := '0'; v_n := 0;
            end case;
            sel_a <= (others => '0'); sel_a(v_a) <= '1';
            sel_b <= (others => '0'); sel_b(v_b) <= '1';
            o_sq <= v_q;
            o_ng <= to_unsigned(v_n, 2);
        end if;
    end process p_opmap;

    p_mul : process(clk)
        variable v_fa : unsigned(23 downto 0);
        variable v_fb : unsigned(12 downto 0);
        function fa(s : std_logic; v : unsigned) return unsigned is
        begin
            return (23 downto 0 => s) and resize(v, 24);
        end function;
        function fb(s : std_logic; v : unsigned) return unsigned is
        begin
            return (12 downto 0 => s) and resize(v, 13);
        end function;
    begin
        if rising_edge(clk) then
            if m_go = '1' then
                v_fa := fa(sel_a(0), c_W) or fa(sel_a(1), gl(0)) or fa(sel_a(2), gl(1)) or fa(sel_a(3), gl(2))
                     or fa(sel_a(4), gl(3)) or fa(sel_a(5), gl(4)) or fa(sel_a(6), m_acc(19 downto 10))
                     or fa(sel_a(7), r_k) or fa(sel_a(8), r_p) or fa(sel_a(9), c_s256) or fa(sel_a(10), c_va)
                     or fa(sel_a(11), c_incw) or fa(sel_a(12), c_incr) or fa(sel_a(13), to_unsigned(562, 10))
                     or fa(sel_a(14), c_xg0) or fa(sel_a(15), c_xs8) or fa(sel_a(16), r_a)
                     or fa(sel_a(17), e_ady) or fa(sel_a(18), e_dgy) or fa(sel_a(19), e_sft) or fa(sel_a(20), r_b);
                v_fb := fb(sel_b(0), c_xs8) or fb(sel_b(5), to_unsigned(1436, 11)) or fb(sel_b(6), to_unsigned(5188, 13))
                     or fb(sel_b(7), to_unsigned(4733, 13)) or fb(sel_b(8), to_unsigned(230, 8))
                     or fb(sel_b(9), to_unsigned(40, 6)) or fb(sel_b(10), to_unsigned(112, 7))
                     or fb(sel_b(11), r_pb) or fb(sel_b(12), to_unsigned(247, 8)) or fb(sel_b(13), to_unsigned(416, 9))
                     or fb(sel_b(14), to_unsigned(473, 9)) or fb(sel_b(15), to_unsigned(444, 9))
                     or fb(sel_b(16), c_incw) or fb(sel_b(17), c_incr) or fb(sel_b(18), c_inca)
                     or fb(sel_b(19), c_ys) or fb(sel_b(20), c_invs) or fb(sel_b(21), c_gsc)
                     or fb(sel_b(23), to_unsigned(326, 9)) or fb(sel_b(24), c_tsig) or fb(sel_b(25), c_jsig)
                     or fb(sel_b(26), c_wstep);
                if o_sq = '1' then v_fb := v_fa(12 downto 0); end if;
                m_a    <= resize(v_fa, 38);
                m_b    <= v_fb;
                m_acc  <= (others => '0');
                case to_integer(o_ng) is
                    when 1      => m_n <= c_v(10);
                    when 2      => m_n <= r_neg;
                    when 3      => m_n <= r_bn;
                    when others => m_n <= '0';
                end case;
                m_cnt  <= to_unsigned(13, 4);
                m_busy <= '1';
            elsif m_busy = '1' then
                if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                m_a <= shift_left(m_a, 1);
                m_b <= shift_right(m_b, 1);
                m_cnt <= m_cnt - 1;
                if m_cnt = 1 then m_busy <= '0'; end if;
            end if;
            if m_n = '1' then m_res <= -signed(resize(m_acc, 39)); else m_res <= signed(resize(m_acc, 39)); end if;
        end if;
    end process p_mul;

    p_div : process(clk)
        variable v_r : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            if d_go = '1' then
                d_num  <= di_n;
                d_rem  <= (others => '0');
                d_q    <= (others => '0');
                d_cnt  <= to_unsigned(20, 5);
                d_busy <= '1';
            elsif d_busy = '1' then
                v_r := d_rem(11 downto 0) & d_num(19);
                if v_r >= resize(di_d, 13) then
                    v_r := v_r - resize(di_d, 13);
                    d_q <= d_q(18 downto 0) & '1';
                else
                    d_q <= d_q(18 downto 0) & '0';
                end if;
                d_rem <= v_r;
                d_num <= d_num(18 downto 0) & '0';
                d_cnt <= d_cnt - 1;
                if d_cnt = 1 then d_busy <= '0'; end if;
            end if;
        end if;
    end process p_div;

    ---------------------------------------------------------------------------
    -- Sequencer: the per-field set-up (states 1..34, kicked by the vblank
    -- zone after the glides) and the per-line engine (100..125, kicked at each
    -- line's g_start, computing the NEXT line).  One small operation per
    -- state; a product or quotient stalls the sequencer with a fixed wait.
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_p : signed(38 downto 0);
        variable v_y : unsigned(11 downto 0);
        variable v_s : signed(12 downto 0);
        variable v_w16 : unsigned(15 downto 0);
        variable v_r : signed(15 downto 0);
        variable v_o : signed(11 downto 0);
        variable v_a : signed(22 downto 0);
        variable v_u : unsigned(15 downto 0);

        procedure mgo is
        begin
            m_go   <= '1';
            q_wait <= to_unsigned(17, 5);
        end procedure;
        procedure dvd(n : unsigned; d : unsigned) is
        begin
            di_n   <= resize(n, 20);
            di_d   <= resize(d, 12);
            d_go   <= '1';
            q_wait <= to_unsigned(23, 5);
        end procedure;
    begin
        if rising_edge(clk) then
            m_go <= '0';
            d_go <= '0';
            -- the signed product of the last multiply (registered in p_mul)
            v_p := m_res;

            if s_zgo = '1' then
                q_st   <= to_unsigned(1, 7);
                q_wait <= to_unsigned(24, 5);             -- the glides first
            elsif q_wait /= 0 then
                q_wait <= q_wait - 1;
            elsif g_start = '1' and q_st = 0 then
                q_st    <= to_unsigned(100, 7);
                q_first <= '0';
            else
                case to_integer(q_st) is
                    -- ------------------------------------------- per field ---
                    when 1 =>
                        dvd(to_unsigned(276480, 20), c_H);
                        q_st <= q_st + 1;
                    when 2 =>
                        c_ys <= d_q(11 downto 0);
                        dvd(to_unsigned(491520, 20), c_W);
                        q_st <= q_st + 1;
                    when 3 =>
                        c_wstep <= d_q(11 downto 0);
                        if s_sd = '1' then c_xs8 <= to_unsigned(1024, 12);
                        else c_xs8 <= c_ys(10 downto 0) & '0'; end if;
                        q_st <= q_st + 1;
                    when 4 =>
                        mgo;
                        if c_mir = '1' then c_xgst <= -signed(resize(c_xs8, 13));
                        else c_xgst <= signed(resize(c_xs8, 13)); end if;
                        q_st <= q_st + 1;
                    when 5 =>
                        c_vw <= m_acc(20 downto 9);
                        q_st <= q_st + 1;
                    when 6 =>
                        if c_vw >= 1920 then c_xg0 <= (others => '0');
                        else c_xg0 <= resize(to_unsigned(491520, 20) - (resize(c_vw, 20) sll 8), 21); end if;
                        q_st <= q_st + 1;
                    when 7 =>
                        if c_mir = '1' then c_xg0 <= to_unsigned(983040, 21) - c_xg0; end if;
                        mgo;
                        q_st <= q_st + 1;
                    when 8 =>
                        mgo;
                        q_st <= q_st + 1;
                    when 9 =>
                        c_incw <= to_unsigned(386, 11) + m_acc(20 downto 10);
                        mgo;
                        q_st <= q_st + 1;
                    when 10 =>
                        mgo;
                        q_st <= q_st + 1;
                    when 11 =>
                        c_incr <= to_unsigned(1365, 13) + m_acc(22 downto 10);
                        mgo;
                        q_st <= q_st + 1;
                    when 12 =>
                        mgo;
                        q_st <= q_st + 1;
                    when 13 =>
                        c_inca <= to_unsigned(728, 13) + m_acc(22 downto 10);
                        mgo;
                        q_st <= q_st + 1;
                    when 14 =>
                        c_gsc <= to_unsigned(410, 9) - m_acc(18 downto 10);
                        mgo;
                        q_st <= q_st + 1;
                    when 15 =>
                        r_k <= m_acc(19 downto 10);
                        mgo;
                        q_st <= q_st + 1;
                    when 16 =>
                        c_tsig <= m_acc(15 downto 10);
                        if gl(5) <= 512 then r_p <= gl(5); r_pb <= to_unsigned(399, 10);
                        else r_p <= gl(5) - 512; r_pb <= to_unsigned(614, 10); end if;
                        mgo;
                        q_st <= q_st + 1;
                    when 17 =>
                        c_jsig <= m_acc(16 downto 10);
                        mgo;
                        q_st <= q_st + 1;
                    when 18 =>
                        if gl(5) <= 512 then c_s256 <= to_unsigned(56, 10) + m_acc(19 downto 10);
                        else c_s256 <= to_unsigned(256, 10) + m_acc(19 downto 10); end if;
                        q_st <= q_st + 1;
                    when 19 =>
                        dvd(to_unsigned(65536, 20), c_s256);
                        q_st <= q_st + 1;
                    when 20 =>
                        c_invs <= d_q(10 downto 0);
                        mgo;
                        q_st <= q_st + 1;
                    when 21 =>
                        if m_acc(17 downto 8) > 560 then c_bh <= to_unsigned(560, 10);
                        else c_bh <= m_acc(17 downto 8); end if;
                        mgo;
                        q_st <= q_st + 1;
                    when 22 =>
                        if m_acc(18 downto 8) > 540 then c_top <= to_unsigned(540, 10);
                        else c_top <= m_acc(17 downto 8); end if;
                        mgo;
                        q_st <= q_st + 1;
                    when 23 =>
                        c_kR <= m_acc(18 downto 8);
                        mgo;
                        q_st <= q_st + 1;
                    when 24 =>
                        if m_acc(17 downto 8) > 444 then c_xcut <= to_unsigned(1585, 11);
                        else c_xcut <= to_unsigned(1141, 11) + m_acc(18 downto 8); end if;
                        mgo;
                        q_st <= q_st + 1;
                    when 25 =>
                        c_phw <= c_phw + unsigned(v_p(23 downto 8));
                        mgo;
                        q_st <= q_st + 1;
                    when 26 =>
                        c_phr <= c_phr + unsigned(v_p(26 downto 10));
                        mgo;
                        q_st <= q_st + 1;
                    when 27 =>
                        c_pha <= c_pha + unsigned(v_p(24 downto 9));
                        mgo;
                        q_st <= q_st + 1;
                    when 28 =>
                        if c_mir = '1' then c_incwpx <= -signed(resize(m_acc(23 downto 1), 23));
                        else c_incwpx <= signed(resize(m_acc(23 downto 1), 23)); end if;
                        mgo;
                        q_st <= q_st + 1;
                    when 29 =>
                        c_incrln <= m_acc(24 downto 8);
                        mgo;
                        q_st <= q_st + 1;
                    when 30 =>
                        c_row0 <= c_phr - m_acc(16 downto 0);
                        mgo;
                        q_st <= q_st + 1;
                    when 31 =>
                        c_st0 <= m_acc(32 downto 9) + (c_phw & x"00");
                        mgo;
                        q_st <= q_st + 1;
                    when 32 =>
                        if c_mir = '1' then c_xsst <= -signed(resize(m_acc(22 downto 8), 16));
                        else c_xsst <= signed(resize(m_acc(22 downto 8), 16)); end if;
                        v_a := signed(resize(c_xg0, 23)) - 503808;
                        r_a <= unsigned(abs(v_a)); r_neg <= v_a(22);
                        mgo;
                        q_st <= q_st + 1;
                    when 33 =>
                        if c_mir = '1' then c_gdst <= -signed(resize(m_acc(20 downto 9), 13));
                        else c_gdst <= signed(resize(m_acc(20 downto 9), 13)); end if;
                        mgo;
                        q_st <= q_st + 1;
                    when 34 =>
                        c_gd0 <= v_p(31 downto 9);
                        e_vy    <= (others => '0');
                        q_first <= '1';
                        q_st    <= to_unsigned(100, 7);

                    -- --------------------------------- per line (the NEXT one) ---
                    when 100 =>
                        v_y := e_vy(19 downto 8);
                        e_yl  <= v_y(10 downto 0);
                        e_ady <= resize(unsigned(abs(signed('0' & v_y) - 562)), 11);
                        e_agy <= resize(unsigned(abs(signed('0' & v_y) - 554)), 11);
                        e_aby <= resize(unsigned(abs(signed('0' & v_y) - 565)), 11);
                        q_st <= q_st + 1;
                    when 101 =>
                        e_dgy <= resize(e_agy, 12) + resize(e_agy(10 downto 1), 12) + resize(e_agy(10 downto 5), 12);
                        if c_band = '1' and e_aby < c_bh then n_inb <= '1'; else n_inb <= '0'; end if;
                        if c_band = '1' and e_aby + 10 < c_bh then n_gate <= '1'; else n_gate <= '0'; end if;
                        v_s := signed(resize(e_aby, 13)) - signed(resize(c_bh, 13));
                        if c_band = '1' and v_s < 4 and v_s > -4 then n_edge <= '1'; else n_edge <= '0'; end if;
                        if c_band = '1' and v_s < 2 and v_s > -2 then n_dotr <= '1'; else n_dotr <= '0'; end if;
                        if e_ady < c_top then n_keyrow <= '1'; else n_keyrow <= '0'; end if;
                        v_s := signed(resize(e_ady, 13)) - signed(resize(c_top, 13)) + 40;
                        if v_s <= 0 then e_sft <= (others => '0');
                        elsif v_s > 63 then e_sft <= to_unsigned(63, 6);
                        else e_sft <= unsigned(v_s(5 downto 0)); end if;
                        mgo;
                        q_st <= q_st + 1;
                    when 102 =>
                        n_adys <= m_acc(18 downto 8);
                        mgo;
                        q_st <= q_st + 1;
                    when 103 =>
                        n_dgyg <= m_acc(19 downto 8);
                        mgo;
                        q_st <= q_st + 1;
                    when 104 =>
                        v_w16 := resize(m_acc(11 downto 0), 16);
                        v_w16 := shift_left(v_w16, 3) + shift_left(v_w16, 2) + v_w16;   -- x 13
                        e_R <= signed(resize(c_kR, 13)) - signed(resize(e_ady, 13))
                               - signed(resize(v_w16(15 downto 10), 13));
                        q_st <= q_st + 1;
                    when 105 =>
                        r_a <= resize(unsigned(abs(e_R)), 23);
                        r_neg <= e_R(12);
                        if e_R < 0 then n_keyrow <= '0'; end if;
                        q_st <= q_st + 1;
                    when 106 =>
                        mgo;
                        q_st <= q_st + 1;
                    when 107 =>
                        n_xsh <= to_signed(1141, 13) - v_p(20 downto 8);
                        q_st <= q_st + 1;
                    when 108 | 114 =>
                        e_rng <= e_rng xor (e_rng(18 downto 0) & "0000000000000");
                        q_st <= q_st + 1;
                    when 109 | 115 =>
                        e_rng <= e_rng xor ("00000000000000000" & e_rng(31 downto 17));
                        q_st <= q_st + 1;
                    when 110 | 116 =>
                        e_rng <= e_rng xor (e_rng(26 downto 0) & "00000");
                        q_st <= q_st + 1;
                    when 111 =>
                        if e_run = 0 then
                            e_run <= resize(to_unsigned(1, 4) + e_rng(2 downto 0) + resize(e_rng(4 downto 3), 3), 4);
                            if e_rng(8 downto 5) < 3 then e_big <= '1'; else e_big <= '0'; end if;
                            v_o := signed(resize(e_rng(16 downto 9), 12)) + signed(resize(e_rng(24 downto 17), 12)) - 255;
                            r_a <= resize(unsigned(abs(v_o)), 23);
                            r_neg <= v_o(11);
                            q_st <= q_st + 1;
                        else
                            e_run <= e_run - 1;
                            q_st <= to_unsigned(114, 7);
                        end if;
                    when 112 =>
                        mgo;
                        q_st <= q_st + 1;
                    when 113 =>
                        v_r := v_p(22 downto 7);
                        if e_big = '1' then e_off <= resize(shift_left(v_r, 2), 11);
                        else e_off <= resize(v_r, 11); end if;
                        q_st <= q_st + 1;
                    when 117 =>
                        if n_inb = '1' then e_tear <= e_off; else e_tear <= (others => '0'); end if;
                        v_o := signed(resize(e_rng(7 downto 0), 12)) - 128;
                        r_a <= resize(unsigned(abs(v_o)), 23);
                        r_neg <= v_o(11);
                        q_st <= q_st + 1;
                    when 118 =>
                        mgo;
                        if e_tear < 0 then
                            n_rp0 <= resize(unsigned(signed(resize(c_W, 13)) + resize(e_tear, 13)), 11);
                        else
                            n_rp0 <= unsigned(e_tear);
                        end if;
                        r_b <= resize(unsigned(abs(e_tear)), 11);
                        r_bn <= e_tear(10);
                        q_st <= q_st + 1;
                    when 119 =>
                        v_r := v_p(25 downto 10);
                        if n_inb = '0' then
                            v_r := shift_right(shift_left(v_r, 2) + v_r, 3);
                        end if;
                        e_js <= resize(v_r, 7);
                        mgo;
                        q_st <= q_st + 1;
                    when 120 =>
                        n_xoff <= v_p(19 downto 8);
                        n_jit <= e_js;
                        r_b <= resize(unsigned(abs(resize(e_js, 8))), 11);
                        r_bn <= e_js(6);
                        r_s <= signed(resize(c_xg0, 23)) - 576000 + shift_left(resize(e_js, 23), 9);
                        q_st <= q_st + 1;
                    when 121 =>
                        mgo;
                        r_a <= unsigned(abs(r_s));
                        r_neg <= r_s(22);
                        q_st <= q_st + 1;
                    when 122 =>
                        n_st <= c_st0 + shift_left(resize(unsigned(v_p(15 downto 0)), 24), 8);
                        mgo;
                        q_st <= q_st + 1;
                    when 123 =>
                        n_xs0 <= 576000 + v_p(30 downto 8);
                        if q_first = '1' then n_rowph <= c_row0;
                        else n_rowph <= n_rowph + c_incrln; end if;
                        -- oscillator line terms
                        v_u := resize(e_yl, 16);
                        e_p <= shift_left(v_u, 7) + shift_left(v_u, 6) + shift_left(v_u, 5) + shift_left(c_t, 7);  -- 224 y + 128 t
                        q_st <= q_st + 1;
                    when 124 =>
                        n_o1t <= unsigned(shift_left(resize(f_tri8(e_p), 16), 7)) + shift_left(c_t, 6);
                        v_u := resize(e_yl, 16);
                        n_p2y <= shift_left(v_u, 8) + shift_left(v_u, 5) - shift_left(c_t, 7);          -- 288 y - 128 t
                        n_p3  <= shift_left(c_t, 7) + shift_left(c_t, 6) - shift_left(v_u, 7);         -- 192 t - 128 y
                        q_st <= q_st + 1;
                    when 125 =>
                        e_vy <= e_vy + c_ys;
                        q_st <= (others => '0');
                    when others =>
                        q_st <= (others => '0');
                end case;
            end if;
        end if;
    end process p_seq;

    ---------------------------------------------------------------------------
    -- Generated avid (damier / cubist recipe)
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
            g_wm1 <= c_W - 1;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Source side: one line of camera luma, (Y - 64) x 0.281, clamped
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
            s_y1  <= signed(resize(unsigned(data_in.y), 12)) - 64;
            s_wc1 <= s_wcol;
            s_we1 <= data_in.avid;
            v_l := shift_right(s_y1, 2) + shift_right(s_y1, 5);
            lb_wd <= std_logic_vector(f_clamp8(v_l));
            lb_wa <= s_wc1;
            lb_we <= s_we1;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: per-pixel accumulators (loaded at g_start from the line engine),
    -- and the late glow accumulator (stage 9, reloaded while its stage is idle)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_xg <= c_xg0;
                b_xs <= n_xs0;
                b_xw <= (others => '0');
                b_ph <= n_st;
                b_rp <= n_rp0;
                b_xc <= (others => '0');
                l_adys <= n_adys; l_dgyg <= n_dgyg; l_inb <= n_inb; l_gate <= n_gate;
                l_edge <= n_edge; l_dotr <= n_dotr; l_keyrow <= n_keyrow; l_xsh <= n_xsh;
                l_jit <= n_jit; l_rowph <= n_rowph; l_xoff <= n_xoff;
                l_o1t <= n_o1t; l_p2y <= n_p2y; l_p3 <= n_p3;
            elsif g_avid = '1' then
                b_xg <= unsigned(signed(b_xg) + resize(c_xgst, 21));
                b_xs <= b_xs + resize(c_xsst, 23);
                b_xw <= b_xw + c_wstep;
                b_ph <= unsigned(signed(b_ph) + resize(c_incwpx, 24));
                if b_rp = c_W(10 downto 0) - 1 then b_rp <= (others => '0'); else b_rp <= b_rp + 1; end if;
                b_xc <= b_xc + 1;
            end if;
            if s_gav_sr(8) = '0' then
                b_gdl <= c_gd0;
            else
                b_gdl <= b_gdl + resize(c_gdst, 23);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path.  S0 = the clock where the beam accumulators hold the pixel;
    -- S24 = the output register.
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_e : signed(15 downto 0);
        variable v_x : signed(15 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_lev : unsigned(2 downto 0);
        variable v_ta : signed(8 downto 0);
        variable v_lo : unsigned(7 downto 0);
        variable v_b9 : signed(8 downto 0);
        variable v_sum : signed(10 downto 0);
        variable v_m : signed(16 downto 0);
        variable v_a11 : signed(10 downto 0);
        variable v_b14 : signed(13 downto 0);
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ab : signed(22 downto 0);
        variable v_z9 : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            -- S1: ROM / RAM addresses, coordinates
            if b_xs < 0 then g_ra <= (others => '0');
            elsif b_xs(22 downto 20) /= "000" then g_ra <= (others => '1');
            else g_ra <= unsigned(b_xs(19 downto 10)); end if;
            lb_ra <= b_rp;
            xp_ra <= b_xw(18 downto 11);
            p1_xg <= b_xg(20 downto 9);
            p1_xj <= signed(resize(b_xg(20 downto 9), 14)) + resize(l_jit, 14);
            p1_xw <= b_xw(19 downto 8);
            p1_xwt <= signed(resize(b_xw(19 downto 8), 13)) + resize(l_xoff, 13);
            p_ph(1) <= b_ph(23 downto 8);
            p_dot(1) <= not b_xc(2);

            -- S2: key, bank; weave zigzag; oscillator phases
            if l_keyrow = '1' and resize(p1_xj, 15) > resize(l_xsh, 15) and p1_xg < c_xcut and p1_xw < 1585 then
                p_key(2) <= '1';
            else
                p_key(2) <= '0';
            end if;
            if p1_xj >= 1050 or l_inb = '0' then p_bank(2) <= '1'; else p_bank(2) <= '0'; end if;
            p2_zz <= f_tri8(p_ph(1));
            v_x := resize(p1_xwt, 16);
            p2_q1  <= unsigned(shift_left(v_x, 7)) + l_o1t;
            p2_q2x <= unsigned(shift_left(v_x, 6) + shift_left(v_x, 5)) - shift_left(c_t, 6);
            p2_q3  <= unsigned(shift_left(v_x, 7) + shift_left(v_x, 6)) + l_p3;

            -- S3: ROM fabric copies; row phase; oscillator triangles
            p3_g  <= unsigned(g_q);
            p3_xp <= xp_q;
            p_lum(3) <= unsigned(lb_q);
            v_e := resize(p2_zz, 16);
            v_e := shift_left(v_e, 6) + shift_left(v_e, 4) + shift_left(v_e, 3) + shift_left(v_e, 2);  -- x 92
            p3_rph <= unsigned(signed(l_rowph) + resize(v_e, 17));
            v_z9 := f_tri8(p_ph(2)(13 downto 0) & "00");
            p_zg(3) <= v_z9(8 downto 5);                                 -- +-4 F-units
            p3_t1  <= f_tri8(p2_q1);
            p3_t2x <= f_tri8(p2_q2x);
            p3_t3  <= f_tri8(p2_q3);

            -- S4: F_raw; weave row geometry; x profile; oscillator 2
            p_fr(4) <= l_adys + resize(p3_g, 11);
            if p3_rph(15) = '1' then p4_vyf <= '0' & p3_rph(14 downto 8);
            else p4_vyf <= to_unsigned(128, 8) - resize(p3_rph(14 downto 8), 8); end if;
            if p3_rph(16) = '1' then p4_oph <= p_ph(3) + 49152; else p4_oph <= p_ph(3) + 16384; end if;
            if c_mir = '1' then p4_tw5 <= unsigned(p3_xp(9 downto 5)); else p4_tw5 <= unsigned(p3_xp(4 downto 0)); end if;
            p_k16(4) <= unsigned(p3_xp(14 downto 10));
            p_fl(4)  <= p3_xp(15);
            v_e := resize(p3_t2x, 16);
            p4_q2 <= l_p2y + unsigned(shift_left(v_e, 7) + shift_left(v_e, 5));                     -- x 160
            p4_t1 <= p3_t1;
            p4_t3 <= p3_t3;

            -- S5: dash phase; band-key threshold; oscillator 2 triangle
            p_ox(5) <= f_tri8(p4_oph);
            if p4_vyf < 102 then p_inrow(5) <= '1'; else p_inrow(5) <= '0'; end if;
            v_t := shift_left(resize(p4_tw5 & "000", 16), 7) + shift_left(resize(p4_tw5 & "000", 16), 5)
                   - resize(p4_tw5 & "000", 16);
            p5_thv <= to_unsigned(77, 9) + resize(v_t(15 downto 8), 9);
            if p4_tw5 = 31 then p5_tw31 <= '1'; else p5_tw31 <= '0'; end if;
            p5_t2 <= f_tri8(p4_q2);
            p5_t1 <= p4_t1;
            p5_t3 <= p4_t3;

            -- S6: oscillator luma
            v_sum := resize(shift_right(p5_t1, 1), 11) + resize(shift_right(p5_t2, 1), 11)
                     + resize(shift_right(p5_t3, 2), 11) + 128;
            p6_lo <= f_clamp8(v_sum);
            p6_thv <= p5_thv;
            p6_tw31 <= p5_tw31;

            -- S7: the source luma (camera or oscillators); bend; band key
            if c_osc = '1' then v_lo := p6_lo; else v_lo := p_lum(6); end if;
            p_lum(7) <= v_lo;
            v_b9 := signed('0' & v_lo) - 128;
            p7_bend <= shift_right(v_b9, 2) + shift_right(v_b9, 3);
            if l_inb = '1' and resize(v_lo, 9) > p6_thv and p6_tw31 = '0' then p7_vshow <= '1';
            else p7_vshow <= '0'; end if;

            -- S8: F; the layer select (texture / video / glow)
            v_g := signed(resize(p_fr(7), 13));
            if c_bend = '1' then v_g := v_g + resize(p7_bend, 13); end if;
            v_g := v_g + resize(p_zg(7), 13);
            if v_g < 0 then p_f(8) <= (others => '0');
            elsif v_g > 1023 then p_f(8) <= (others => '1');
            else p_f(8) <= unsigned(v_g(9 downto 0)); end if;
            -- band: texture or keyed video at full gain; outside it the cocoon is windowed like the
            -- glow (it darkens toward the window edge and hands over to the flank video)
            if (l_inb = '1' and p7_vshow = '0') or (l_inb = '0' and p_key(7) = '1' and p_fl(7) = '0') then
                p8_ut <= '1';
            else
                p8_ut <= '0';
            end if;
            if p7_vshow = '1' or (l_inb = '0' and p_fl(7) = '1') then p8_uv <= '1'; else p8_uv <= '0'; end if;
            p8_full <= l_inb;

            -- S9: the arc product F x inca / 16; the dash threshold
            p9_pr <= p_f(8) * c_inca(12 downto 4);
            p9_thr <= signed(resize(shift_right(resize(p_f(8), 12) + shift_left(resize(p_f(8), 12), 1), 4), 10)) - 20;

            -- S10: arc phase; dashes and gaps; the glow distance (late accumulator)
            p10_pa <= resize(shift_left(resize(p9_pr, 23), 4), 16) - c_pha;
            if p_inrow(9) = '1' and resize(p_ox(9), 10) > p9_thr then p10_dash <= '1'; else p10_dash <= '0'; end if;
            if p_inrow(9) = '1' and resize(p_ox(9), 10) < p9_thr - 80 then p10_gap <= '1'; else p10_gap <= '0'; end if;
            if b_gdl < 0 then v_ab := -b_gdl; else v_ab := b_gdl; end if;
            p10_dg <= resize(unsigned(v_ab(20 downto 8)), 13) + l_dgyg;

            -- S11: level and ROM addresses
            v_ta := f_tri8(p10_pa);
            if v_ta > 45 then v_lev := "001";
            elsif v_ta < -45 then v_lev := "011";
            else v_lev := "010"; end if;
            if l_gate = '1' then
                if p10_dash = '1' then v_lev := "000";
                elsif p10_gap = '1' then v_lev := "100"; end if;
            end if;
            tex_ra <= p_bank(10) & p_f(10)(9 downto 4) & v_lev;
            if p10_dg(12 downto 11) /= "00" then glw_ra <= (others => '1');
            else glw_ra <= p10_dg(10 downto 3); end if;
            vid_ra <= l_inb & p_lum(10)(7 downto 3);

            -- S12: ROM registers (p_rom).  S13: fabric copies
            t13_y <= unsigned(t_qy); t13_u <= unsigned(t_qu); t13_v <= unsigned(t_qv);
            w13_y <= unsigned(w_qy); w13_u <= unsigned(w_qu); w13_v <= unsigned(w_qv);
            v13_y <= unsigned(v_qy); v13_u <= unsigned(v_qu); v13_v <= unsigned(v_qv);

            -- S14: the layer (Ice exchanges U and V of the generated colours)
            if p_ut(13) = '1' then
                s14_y <= t13_y & "00";
                if c_ice = '1' then s14_u <= t13_v & "00"; s14_v <= t13_u & "00";
                else s14_u <= t13_u & "00"; s14_v <= t13_v & "00"; end if;
            elsif p_uv(13) = '1' then
                s14_y <= v13_y & "00"; s14_u <= v13_u & "00"; s14_v <= v13_v & "00";
            else
                s14_y <= w13_y & "00";
                if c_ice = '1' then s14_u <= w13_v & "00"; s14_v <= w13_u & "00";
                else s14_u <= w13_u & "00"; s14_v <= w13_v & "00"; end if;
            end if;
            if p_full(13) = '1' then s14_k <= to_unsigned(16, 5); else s14_k <= p_k16(13); end if;

            -- S15: the window gain (flank video fades out, the glow fades in)
            v_a11 := signed(resize(s14_y, 11)) - 64;
            v_m := v_a11 * signed('0' & s14_k);
            v_m := shift_right(v_m, 4) + 64;  s15_y <= unsigned(v_m(9 downto 0));
            v_a11 := signed(resize(s14_u, 11)) - 512;
            v_m := v_a11 * signed('0' & s14_k);
            v_m := shift_right(v_m, 4) + 512; s15_u <= unsigned(v_m(9 downto 0));
            v_a11 := signed(resize(s14_v, 11)) - 512;
            v_m := v_a11 * signed('0' & s14_k);
            v_m := shift_right(v_m, 4) + 512; s15_v <= unsigned(v_m(9 downto 0));

            -- S16: band-edge dim and dots
            v_y := s15_y; v_u := s15_u; v_v := s15_v;
            if l_edge = '1' and p_key(15) = '0' then
                if l_dotr = '1' and p_dot(15) = '1' then
                    v_y := to_unsigned(940, 10); v_u := to_unsigned(580, 10); v_v := to_unsigned(504, 10);
                else
                    v_a11 := signed('0' & s15_y) - 64;
                    v_b14 := shift_left(resize(v_a11, 14), 1) + resize(v_a11, 14);
                    v_b14 := shift_right(v_b14, 3) + 64;  v_y := unsigned(v_b14(9 downto 0));
                    v_a11 := signed('0' & s15_u) - 512;
                    v_b14 := shift_left(resize(v_a11, 14), 1) + resize(v_a11, 14);
                    v_b14 := shift_right(v_b14, 3) + 512; v_u := unsigned(v_b14(9 downto 0));
                    v_a11 := signed('0' & s15_v) - 512;
                    v_b14 := shift_left(resize(v_a11, 14), 1) + resize(v_a11, 14);
                    v_b14 := shift_right(v_b14, 3) + 512; v_v := unsigned(v_b14(9 downto 0));
                end if;
            end if;
            s16_y <= v_y; s16_u <= v_u; s16_v <= v_v;

            -- S17 .. S24: output conditioning (centre at S21 -> S22), gate at S23
            cr_wp <= cr_wp + 1;
            cr_ra <= cr_wp - 5;
            cr_f  <= cr_q;
            s_boxu <= s_boxu + s16_u - unsigned(cr_f(19 downto 10));
            s_boxv <= s_boxv + s16_v - unsigned(cr_f(9 downto 0));
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u22 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v22 <= v_sc(13 downto 4);
            s_y17 <= s16_y; s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19;
            s_y21 <= s_y20; s_y22x <= s_y21;
            v_sy := resize(s_y20, 12) + shift_left(resize(s_y21, 12), 1) + resize(s_y22x, 12);
            s_yf22 <= v_sy(11 downto 2);
            if s_yf22 > 64 then v_dl := s_yf22 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(21) = '1' then
                s_y23 <= s_yf22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;

            -- delays
            for i in 2 to 3 loop p_ph(i) <= p_ph(i - 1); end loop;
            for i in 2 to 15 loop p_dot(i) <= p_dot(i - 1); end loop;
            for i in 3 to 15 loop p_key(i) <= p_key(i - 1); end loop;
            for i in 3 to 10 loop p_bank(i) <= p_bank(i - 1); end loop;
            for i in 4 to 6 loop p_lum(i) <= p_lum(i - 1); end loop;
            for i in 8 to 10 loop p_lum(i) <= p_lum(i - 1); end loop;
            for i in 5 to 7 loop p_fr(i) <= p_fr(i - 1); end loop;
            for i in 4 to 7 loop p_zg(i) <= p_zg(i - 1); end loop;
            for i in 5 to 13 loop p_k16(i) <= p_k16(i - 1); end loop;
            for i in 5 to 7 loop p_fl(i) <= p_fl(i - 1); end loop;
            for i in 6 to 9 loop p_ox(i) <= p_ox(i - 1); p_inrow(i) <= p_inrow(i - 1); end loop;
            for i in 9 to 10 loop p_f(i) <= p_f(i - 1); end loop;
            p_ut(9) <= p8_ut; p_uv(9) <= p8_uv; p_full(9) <= p8_full;
            for i in 10 to 13 loop
                p_ut(i) <= p_ut(i - 1); p_uv(i) <= p_uv(i - 1); p_full(i) <= p_full(i - 1);
            end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cocoon;
